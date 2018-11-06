// SPDX-License-Identifier: GPL-2.0
/*
 * intel-qep.c - Intel Quadrature Encoder Driver
 *
 * Copyright (C) 2019 Intel Corporation - https://www.intel.com
 *
 * Author: Felipe Balbi <felipe.balbi@linux.intel.com>
 */
#include <linux/bitops.h>
#include <linux/counter.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <linux/sysfs.h>

#define INTEL_QEPCON		0x00
#define INTEL_QEPFLT		0x04
#define INTEL_QEPCOUNT		0x08
#define INTEL_QEPMAX		0x0c
#define INTEL_QEPWDT		0x10
#define INTEL_QEPCAPDIV		0x14
#define INTEL_QEPCNTR		0x18
#define INTEL_QEPCAPBUF		0x1c
#define INTEL_QEPINT_STAT	0x20
#define INTEL_QEPINT_MASK	0x24

/* QEPCON */
#define INTEL_QEPCON_EN		BIT(0)
#define INTEL_QEPCON_FLT_EN	BIT(1)
#define INTEL_QEPCON_EDGE_A	BIT(2)
#define INTEL_QEPCON_EDGE_B	BIT(3)
#define INTEL_QEPCON_EDGE_INDX	BIT(4)
#define INTEL_QEPCON_SWPAB	BIT(5)
#define INTEL_QEPCON_OP_MODE	BIT(6)
#define INTEL_QEPCON_PH_ERR	BIT(7)
#define INTEL_QEPCON_COUNT_RST_MODE BIT(8)
#define INTEL_QEPCON_INDX_GATING_MASK GENMASK(10, 9)
#define INTEL_QEPCON_INDX_GATING(n) (((n) & 3) << 9)
#define INTEL_QEPCON_INDX_PAL_PBL INTEL_QEPCON_INDX_GATING(0)
#define INTEL_QEPCON_INDX_PAL_PBH INTEL_QEPCON_INDX_GATING(1)
#define INTEL_QEPCON_INDX_PAH_PBL INTEL_QEPCON_INDX_GATING(2)
#define INTEL_QEPCON_INDX_PAH_PBH INTEL_QEPCON_INDX_GATING(3)
#define INTEL_QEPCON_CAP_MODE	BIT(11)
#define INTEL_QEPCON_FIFO_THRE_MASK GENMASK(14, 12)
#define INTEL_QEPCON_FIFO_THRE(n) ((((n) - 1) & 7) << 12)
#define INTEL_QEPCON_FIFO_EMPTY	BIT(15)

/* QEPFLT */
#define INTEL_QEPFLT_MAX_COUNT(n) ((n) & 0x1fffff)

/* QEPINT */
#define INTEL_QEPINT_FIFOCRIT	BIT(5)
#define INTEL_QEPINT_FIFOENTRY	BIT(4)
#define INTEL_QEPINT_QEPDIR	BIT(3)
#define INTEL_QEPINT_QEPRST_UP	BIT(2)
#define INTEL_QEPINT_QEPRST_DOWN BIT(1)
#define INTEL_QEPINT_WDT	BIT(0)

#define INTEL_QEP_DIRECTION_FORWARD 1
#define INTEL_QEP_DIRECTION_BACKWARD !INTEL_QEP_DIRECTION_FORWARD

#define INTEL_QEP_COUNTER_EXT_RW(_name) \
{ \
	.name = #_name, \
	.read = _name##_read, \
	.write = _name##_write, \
}

#define INTEL_QEP_COUNTER_EXT_RO(_name) \
{ \
	.name = #_name, \
	.read = _name##_read, \
}

#define INTEL_QEP_COUNTER_COUNT_EXT_RW(_name) \
{ \
	.name = #_name, \
	.read = _name##_read, \
	.write = _name##_write, \
}

#define INTEL_QEP_COUNTER_COUNT_EXT_RO(_name) \
{ \
	.name = #_name, \
	.read = _name##_read, \
}

struct intel_qep {
	struct counter_device counter;
	struct mutex lock;
	struct pci_dev *pci;
	struct device *dev;
	void __iomem *regs;
	u32 interrupt;
	int direction;
	bool enabled;
};

#define counter_to_qep(c)	(container_of((c), struct intel_qep, counter))

static inline u32 intel_qep_readl(void __iomem *base, u32 offset)
{
	return readl(base + offset);
}

static inline void intel_qep_writel(void __iomem *base, u32 offset, u32 value)
{
	writel(value, base + offset);
}

static const struct pci_device_id intel_qep_id_table[] = {
	/* EHL */
	{ PCI_VDEVICE(INTEL, 0x4bc3), },
	{ PCI_VDEVICE(INTEL, 0x4b81), },
	{ PCI_VDEVICE(INTEL, 0x4b82), },
	{ PCI_VDEVICE(INTEL, 0x4b83), },
	{  } /* Terminating Entry */
};
MODULE_DEVICE_TABLE(pci, intel_qep_id_table);

static void intel_qep_init(struct intel_qep *qep, bool reset)
{
	u32 reg;

	reg = intel_qep_readl(qep->regs, INTEL_QEPCON);
	reg &= ~INTEL_QEPCON_EN;
	intel_qep_writel(qep->regs, INTEL_QEPCON, reg);

	/* make sure periperal is disabled by reading one more time */
	reg = intel_qep_readl(qep->regs, INTEL_QEPCON);

	if (reset) {
		reg &= ~(INTEL_QEPCON_OP_MODE | INTEL_QEPCON_FLT_EN);
		reg |= INTEL_QEPCON_EDGE_A | INTEL_QEPCON_EDGE_B |
			INTEL_QEPCON_EDGE_INDX | INTEL_QEPCON_COUNT_RST_MODE;
	}

	intel_qep_writel(qep->regs, INTEL_QEPCON, reg);

	intel_qep_writel(qep->regs, INTEL_QEPWDT, 0x1000);
	intel_qep_writel(qep->regs, INTEL_QEPINT_MASK, 0x0);

	qep->direction = INTEL_QEP_DIRECTION_FORWARD;
}

static irqreturn_t intel_qep_irq_thread(int irq, void *_qep)
{
	struct intel_qep	*qep = _qep;
	u32			stat;

	mutex_lock(&qep->lock);

	stat = qep->interrupt;
	if (stat & INTEL_QEPINT_FIFOCRIT)
		dev_dbg(qep->dev, "Fifo Critical\n");

	if (stat & INTEL_QEPINT_FIFOENTRY)
		dev_dbg(qep->dev, "Fifo Entry\n");

	if (stat & INTEL_QEPINT_QEPDIR)
		qep->direction = !qep->direction;

	if (stat & INTEL_QEPINT_QEPRST_UP)
		qep->direction = INTEL_QEP_DIRECTION_FORWARD;

	if (stat & INTEL_QEPINT_QEPRST_DOWN)
		qep->direction = INTEL_QEP_DIRECTION_BACKWARD;

	if (stat & INTEL_QEPINT_WDT)
		dev_dbg(qep->dev, "Watchdog\n");

	intel_qep_writel(qep->regs, INTEL_QEPINT_MASK, 0x00);
	mutex_unlock(&qep->lock);

	return IRQ_HANDLED;
}

static irqreturn_t intel_qep_irq(int irq, void *_qep)
{
	struct intel_qep	*qep = _qep;
	u32			stat;

	stat = intel_qep_readl(qep->regs, INTEL_QEPINT_STAT);
	if (stat) {
		qep->interrupt = stat;
		intel_qep_writel(qep->regs, INTEL_QEPINT_MASK, 0xffffffff);
		intel_qep_writel(qep->regs, INTEL_QEPINT_STAT, stat);
		return IRQ_WAKE_THREAD;
	}

	return IRQ_HANDLED;
}

enum intel_qep_synapse_action {
	INTEL_QEP_SYNAPSE_ACTION_RISING_EDGE,
	INTEL_QEP_SYNAPSE_ACTION_FALLING_EDGE,
};

static enum counter_synapse_action intel_qep_synapse_actions[] = {
	[INTEL_QEP_SYNAPSE_ACTION_RISING_EDGE] =
	COUNTER_SYNAPSE_ACTION_RISING_EDGE,
	
	[INTEL_QEP_SYNAPSE_ACTION_FALLING_EDGE] =
	COUNTER_SYNAPSE_ACTION_FALLING_EDGE,
};

enum intel_qep_count_function {
	INTEL_QEP_ENCODER_MODE_NORMAL,
	INTEL_QEP_ENCODER_MODE_SWAPPED,
};

static const enum counter_count_function intel_qep_count_functions[] = {
	[INTEL_QEP_ENCODER_MODE_NORMAL] =
	COUNTER_COUNT_FUNCTION_QUADRATURE_X4,

	[INTEL_QEP_ENCODER_MODE_SWAPPED] =
	COUNTER_COUNT_FUNCTION_QUADRATURE_X4_SWAPPED,
};

static int intel_qep_count_read(struct counter_device *counter,
		struct counter_count *count,
		struct counter_count_read_value *val)
{
	struct intel_qep *const qep = counter->priv;
	uint32_t cntval;

	cntval = intel_qep_readl(qep, INTEL_QEPCOUNT);
	counter_count_read_value_set(val, COUNTER_COUNT_POSITION, &cntval);

	return 0;
}

static int intel_qep_count_write(struct counter_device *counter,
		struct counter_count *count,
		struct counter_count_write_value *val)
{
	struct intel_qep *const qep = counter->priv;
	u32 cnt;
	int err;

	err = counter_count_write_value_get(&cnt, COUNTER_COUNT_POSITION, val);
	if (err)
		return err;

	intel_qep_writel(qep->regs, INTEL_QEPMAX, cnt);

	return 0;
}

static int intel_qep_function_get(struct counter_device *counter,
		struct counter_count *count, size_t *function)
{
	struct intel_qep *qep = counter_to_qep(counter);
	u32 reg;

	reg = intel_qep_readl(qep->regs, INTEL_QEPCON);
	if (reg & INTEL_QEPCON_SWPAB)
		*function = INTEL_QEP_ENCODER_MODE_SWAPPED;
	else
		*function = INTEL_QEP_ENCODER_MODE_NORMAL;

	return 0;
}

static int intel_qep_function_set(struct counter_device *counter,
		struct counter_count *count, size_t function)
{
	struct intel_qep *qep = counter_to_qep(counter);
	u32 reg;

	reg = intel_qep_readl(qep->regs, INTEL_QEPCON);
	if (function == INTEL_QEP_ENCODER_MODE_SWAPPED)
		reg |= INTEL_QEPCON_SWPAB;
	else
		reg &= ~INTEL_QEPCON_SWPAB;
	intel_qep_writel(qep->regs, INTEL_QEPCON, reg);

	return 0;
}

static int intel_qep_action_get(struct counter_device *counter,
		struct counter_count *count, struct counter_synapse *synapse,
		size_t *action)
{
	struct intel_qep *qep = counter_to_qep(counter);
	u32 reg;

	reg = intel_qep_readl(qep->regs, INTEL_QEPCON);

	*action = reg & synapse->signal->id ?
		INTEL_QEP_SYNAPSE_ACTION_RISING_EDGE :
		INTEL_QEP_SYNAPSE_ACTION_FALLING_EDGE;

	return 0;
}

static int intel_qep_action_set(struct counter_device *counter,
		struct counter_count *count,
		struct counter_synapse *synapse, size_t action)
{
	struct intel_qep *qep = counter_to_qep(counter);
	u32 reg;

	reg = intel_qep_readl(qep->regs, INTEL_QEPCON);

	if (action == INTEL_QEP_SYNAPSE_ACTION_RISING_EDGE)
		reg |= synapse->signal->id;
	else
		reg &= ~synapse->signal->id;

	intel_qep_writel(qep->regs, INTEL_QEPCON, reg);

	return 0;
}

static const struct counter_ops intel_qep_counter_ops = {
	.count_read = intel_qep_count_read,
	.count_write = intel_qep_count_write,

	.function_get = intel_qep_function_get,
	.function_set = intel_qep_function_set,

	.action_get = intel_qep_action_get,
	.action_set = intel_qep_action_set,
};

static struct counter_signal intel_qep_signals[] = {
	{
		.id = INTEL_QEPCON_EDGE_A,
		.name = "Phase A",
	},
	{
		.id = INTEL_QEPCON_EDGE_B,
		.name = "Phase B",
	},
	{
		.id = INTEL_QEPCON_EDGE_INDX,
		.name = "Index",
	},
};

static struct counter_synapse intel_qep_count_synapses[] = {
	{
		.actions_list = intel_qep_synapse_actions,
		.num_actions = ARRAY_SIZE(intel_qep_synapse_actions),
		.signal = &intel_qep_signals[0],
	},
	{
		.actions_list = intel_qep_synapse_actions,
		.num_actions = ARRAY_SIZE(intel_qep_synapse_actions),
		.signal = &intel_qep_signals[1],
	},
	{
		.actions_list = intel_qep_synapse_actions,
		.num_actions = ARRAY_SIZE(intel_qep_synapse_actions),
		.signal = &intel_qep_signals[2],
	},
};

static ssize_t ceiling_read(struct counter_device *counter,
		struct counter_count *count, void *priv, char *buf)
{
	struct intel_qep *qep = counter_to_qep(counter);
	u32 reg;

	reg = intel_qep_readl(qep->regs, INTEL_QEPMAX);

	return snprintf(buf, PAGE_SIZE, "%d\n", reg);
}

static ssize_t ceiling_write(struct counter_device *counter,
		struct counter_count *count, void *priv, const char *buf,
		size_t len)
{
	struct intel_qep *qep = counter_to_qep(counter);
	u32 max;
	int ret;

	ret = kstrtou32(buf, 0, &max);
	if (ret < 0)
		return ret;

	intel_qep_writel(qep->regs, INTEL_QEPMAX, max);

	return len;
}

static ssize_t enable_read(struct counter_device *counter,
		struct counter_count *count, void *priv, char *buf)
{
	struct intel_qep *qep = counter_to_qep(counter);
	u32 reg;

	reg = intel_qep_readl(qep->regs, INTEL_QEPCON);

	return snprintf(buf, PAGE_SIZE, "%d\n", !!(reg & INTEL_QEPCON_EN));
}

static ssize_t enable_write(struct counter_device *counter,
		struct counter_count *count, void *priv, const char *buf,
		size_t len)
{
	struct intel_qep *qep = counter_to_qep(counter);
	u32 reg;
	u32 val;
	int ret;

	ret = kstrtou32(buf, 0, &val);
	if (ret < 0)
		return ret;

	reg = intel_qep_readl(qep->regs, INTEL_QEPCON);

	if (val)
		reg |= INTEL_QEPCON_EN;
	else
		reg &= ~INTEL_QEPCON_EN;

	intel_qep_writel(qep->regs, INTEL_QEPCON, reg);

	return len;
}

static ssize_t direction_read(struct counter_device *counter,
		struct counter_count *count, void *priv, char *buf)
{
	struct intel_qep *qep = counter_to_qep(counter);

	return snprintf(buf, PAGE_SIZE, "%s\n", qep->direction ?
			"forward" : "backward");
}

static const struct counter_count_ext intel_qep_count_ext[] = {
	INTEL_QEP_COUNTER_COUNT_EXT_RW(ceiling),
	INTEL_QEP_COUNTER_COUNT_EXT_RW(enable),
	INTEL_QEP_COUNTER_COUNT_EXT_RO(direction),
};

static struct counter_count intel_qep_counter_count[] = {
	{
		.id = 0,
		.name = "Channel 1 Count",
		.functions_list = intel_qep_count_functions,
		.num_functions = ARRAY_SIZE(intel_qep_count_functions),
		.synapses = intel_qep_count_synapses,
		.num_synapses = ARRAY_SIZE(intel_qep_count_synapses),
		.ext = intel_qep_count_ext,
		.num_ext = ARRAY_SIZE(intel_qep_count_ext),
	},
};

static ssize_t noise_read(struct counter_device *counter, void *priv, char *buf)
{
	struct intel_qep *qep = counter_to_qep(counter);
	u32 reg;

	reg = intel_qep_readl(qep->regs, INTEL_QEPCON);

	if (!(reg & INTEL_QEPCON_FLT_EN))
		return snprintf(buf, PAGE_SIZE, "0\n");

	reg = intel_qep_readl(qep->regs, INTEL_QEPFLT);

	return snprintf(buf, PAGE_SIZE, "%d\n", INTEL_QEPFLT_MAX_COUNT(reg));
}

static ssize_t noise_write(struct counter_device *counter, void *priv,
		const char *buf, size_t len)
{
	struct intel_qep *qep = counter_to_qep(counter);
	u32 reg;
	u32 max;
	int ret;

	ret = kstrtou32(buf, 0, &max);
	if (ret < 0)
		return ret;

	if (max > 0x1fffff)
		max = 0x1ffff;

	reg = intel_qep_readl(qep->regs, INTEL_QEPCON);

	if (max == 0) {
		reg &= ~INTEL_QEPCON_FLT_EN;
	} else {
		reg |= INTEL_QEPCON_FLT_EN;
		intel_qep_writel(qep->regs, INTEL_QEPFLT, max);
	}

	intel_qep_writel(qep->regs, INTEL_QEPCON, reg);
	
	return len;
}

static ssize_t preset_read(struct counter_device *counter, void *priv, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0\n");
}

static ssize_t preset_enable_read(struct counter_device *counter, void *priv,
		char *buf)
{
	struct intel_qep *qep = counter_to_qep(counter);
	u32 reg;

	reg = intel_qep_readl(qep->regs, INTEL_QEPCON);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			!(reg & INTEL_QEPCON_COUNT_RST_MODE));
}

static ssize_t preset_enable_write(struct counter_device *counter, void *priv,
		const char *buf, size_t len)
{
	struct intel_qep *qep = counter_to_qep(counter);
	u32 reg;
	u32 val;
	int ret;

	ret = kstrtou32(buf, 0, &val);
	if (ret < 0)
		return ret;

	reg = intel_qep_readl(qep->regs, INTEL_QEPCON);

	if (val)
		reg &= ~INTEL_QEPCON_COUNT_RST_MODE;
	else
		reg |= INTEL_QEPCON_COUNT_RST_MODE;

	intel_qep_writel(qep->regs, INTEL_QEPCON, reg);
	
	return len;
}

static const struct counter_device_ext intel_qep_ext[] = {
	INTEL_QEP_COUNTER_EXT_RW(noise),
	INTEL_QEP_COUNTER_EXT_RO(preset),
	INTEL_QEP_COUNTER_EXT_RW(preset_enable)
};

static int intel_qep_probe(struct pci_dev *pci, const struct pci_device_id *id)
{
	struct intel_qep	*qep;
	struct device		*dev = &pci->dev;
	void __iomem		*regs;
	int			ret;
	int			irq;

	qep = devm_kzalloc(dev, sizeof(*qep), GFP_KERNEL);
	if (!qep)
		return -ENOMEM;

	ret = pcim_enable_device(pci);
	if (ret)
		return ret;

	pci_set_master(pci);

	ret = pcim_iomap_regions(pci, BIT(0), pci_name(pci));
	if (ret)
		return ret;

	regs = pcim_iomap_table(pci)[0];
	if (!regs)
		return -ENOMEM;

	qep->pci = pci;
	qep->dev = dev;
	qep->regs = regs;
	mutex_init(&qep->lock);

	intel_qep_init(qep, true);
	pci_set_drvdata(pci, qep);

	qep->counter.name = pci_name(pci);
	qep->counter.parent = dev;
	qep->counter.ops = &intel_qep_counter_ops;
	qep->counter.counts = intel_qep_counter_count;
	qep->counter.num_counts = ARRAY_SIZE(intel_qep_counter_count);
	qep->counter.signals = intel_qep_signals;
	qep->counter.num_signals = ARRAY_SIZE(intel_qep_signals);
	qep->counter.ext = intel_qep_ext;
	qep->counter.num_ext = ARRAY_SIZE(intel_qep_ext);
	qep->counter.priv = qep;

	ret = counter_register(&qep->counter);
	if (ret)
		return ret;

	ret = pci_alloc_irq_vectors(pci, 1, 1, PCI_IRQ_ALL_TYPES);
	if (ret < 0)
		goto err_irq_vectors;

	irq = pci_irq_vector(pci, 0);
	ret = devm_request_threaded_irq(&pci->dev, irq, intel_qep_irq,
			intel_qep_irq_thread, IRQF_SHARED | IRQF_TRIGGER_RISING,
			"intel-qep", qep);
	if (ret)
		goto err_irq;

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_put_noidle(dev);
	pm_runtime_allow(dev);

	return 0;

err_irq:
	pci_free_irq_vectors(pci);

err_irq_vectors:
	counter_unregister(&qep->counter);

	return ret;
}

static void intel_qep_remove(struct pci_dev *pci)
{
	struct intel_qep	*qep = pci_get_drvdata(pci);
	struct device		*dev = &pci->dev;

	pm_runtime_forbid(dev);
	pm_runtime_get_noresume(dev);

	intel_qep_writel(qep->regs, INTEL_QEPCON, 0);
	pci_free_irq_vectors(pci);
	counter_unregister(&qep->counter);
}

#ifdef CONFIG_PM_SLEEP
static int intel_qep_suspend(struct device *dev)
{
	return 0;
}

static int intel_qep_resume(struct device *dev)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct intel_qep *qep = pci_get_drvdata(pdev);

	intel_qep_init(qep, false);

	return 0;
}

static int intel_qep_runtime_suspend(struct device *dev)
{
	return 0;
}

static int intel_qep_runtime_resume(struct device *dev)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct intel_qep *qep = pci_get_drvdata(pdev);

	intel_qep_init(qep, false);

	return 0;
}
#endif

static const struct dev_pm_ops intel_qep_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(intel_qep_suspend,
				intel_qep_resume)
	SET_RUNTIME_PM_OPS(intel_qep_runtime_suspend, intel_qep_runtime_resume,
				NULL)
};

static struct pci_driver intel_qep_driver = {
	.name		= "intel-qep",
	.id_table	= intel_qep_id_table,
	.probe		= intel_qep_probe,
	.remove		= intel_qep_remove,
	.driver = {
		.pm = &intel_qep_pm_ops,
	}
};

module_pci_driver(intel_qep_driver);

MODULE_AUTHOR("Felipe Balbi <felipe.balbi@linux.intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Intel Quadrature Encoder Driver");
