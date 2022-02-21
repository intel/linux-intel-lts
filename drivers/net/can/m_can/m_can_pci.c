// SPDX-License-Identifier: GPL-2.0
/*
 * PCI Specific M_CAN Glue
 *
 * Copyright (C) 2018-2020 Intel Corporation
 * Author: Felipe Balbi (Intel)
 * Author: Jarkko Nikula <jarkko.nikula@linux.intel.com>
 * Author: Raymond Tan <raymond.tan@intel.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>

#include "m_can.h"

#define M_CAN_PCI_MMIO_BAR		0

#define M_CAN_CLOCK_FREQ_EHL		200000000
#define CTL_CSR_INT_CTL_OFFSET		0x508

struct m_can_pci_priv {
	struct m_can_classdev cdev;

	void __iomem *base;
};
#define M_CAN_D0I3C	0x8000
#define M_CAN_CGSR	0x8004

#define M_CAN_D0I3_CIP BIT(0)
#define M_CAN_D0I3_EN BIT(2)
#define M_CAN_D0I3_RR BIT(3)
#define M_CAN_CGSR_CG BIT(16)

static inline struct m_can_pci_priv *cdev_to_priv(struct m_can_classdev *cdev)
{
	return container_of(cdev, struct m_can_pci_priv, cdev);
}

static u32 iomap_read_reg(struct m_can_classdev *cdev, int reg)
{
	struct m_can_pci_priv *priv = cdev_to_priv(cdev);

	return readl(priv->base + reg);
}

static int iomap_read_fifo(struct m_can_classdev *cdev, int offset, void *val, size_t val_count)
{
	struct m_can_pci_priv *priv = cdev_to_priv(cdev);
	void __iomem *src = priv->base + offset;

	while (val_count--) {
		*(unsigned int *)val = ioread32(src);
		val += 4;
		src += 4;
	}

	return 0;
}

static int iomap_write_reg(struct m_can_classdev *cdev, int reg, int val)
{
	struct m_can_pci_priv *priv = cdev_to_priv(cdev);

	writel(val, priv->base + reg);

	return 0;
}

static int iomap_write_fifo(struct m_can_classdev *cdev, int offset,
			    const void *val, size_t val_count)
{
	struct m_can_pci_priv *priv = cdev_to_priv(cdev);
	void __iomem *dst = priv->base + offset;

	while (val_count--) {
		iowrite32(*(unsigned int *)val, dst);
		val += 4;
		dst += 4;
	}

	return 0;
}

static struct m_can_ops m_can_pci_ops = {
	.read_reg = iomap_read_reg,
	.write_reg = iomap_write_reg,
	.write_fifo = iomap_write_fifo,
	.read_fifo = iomap_read_fifo,
};

static int m_can_pci_probe(struct pci_dev *pci, const struct pci_device_id *id)
{
	struct device *dev = &pci->dev;
	struct m_can_classdev *mcan_class;
	struct m_can_pci_priv *priv;
	void __iomem *base;
	int ret;

	ret = pcim_enable_device(pci);
	if (ret)
		return ret;

	pci_set_master(pci);

	ret = pcim_iomap_regions(pci, BIT(M_CAN_PCI_MMIO_BAR), pci_name(pci));
	if (ret)
		return ret;

	base = pcim_iomap_table(pci)[M_CAN_PCI_MMIO_BAR];

	if (!base) {
		dev_err(dev, "failed to map BARs\n");
		return -ENOMEM;
	}

	mcan_class = m_can_class_allocate_dev(&pci->dev,
					      sizeof(struct m_can_pci_priv));
	if (!mcan_class)
		return -ENOMEM;

	priv = cdev_to_priv(mcan_class);

	priv->base = base;

	ret = pci_alloc_irq_vectors(pci, 1, 1, PCI_IRQ_ALL_TYPES);
	if (ret < 0)
		return ret;

	mcan_class->dev = &pci->dev;
	mcan_class->net->irq = pci_irq_vector(pci, 0);
	mcan_class->pm_clock_support = 1;
	mcan_class->can.clock.freq = id->driver_data;
	mcan_class->ops = &m_can_pci_ops;

	pci_set_drvdata(pci, mcan_class);

	ret = m_can_class_register(mcan_class);
	if (ret)
		goto err;

	/* Enable interrupt control at CAN wrapper IP */
	writel(0x1, base + CTL_CSR_INT_CTL_OFFSET);

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_put_noidle(dev);
	pm_runtime_allow(dev);

	return 0;

err:
	pci_free_irq_vectors(pci);
	return ret;
}

static void m_can_pci_remove(struct pci_dev *pci)
{
	struct m_can_classdev *mcan_class = pci_get_drvdata(pci);
	struct m_can_pci_priv *priv = cdev_to_priv(mcan_class);

	pm_runtime_forbid(&pci->dev);
	pm_runtime_get_noresume(&pci->dev);

	/* Disable interrupt control at CAN wrapper IP */
	writel(0x0, priv->base + CTL_CSR_INT_CTL_OFFSET);

	m_can_class_unregister(mcan_class);
	pci_free_irq_vectors(pci);
}

static __maybe_unused int m_can_pci_suspend(struct device *dev)
{
	return m_can_class_suspend(dev);
}

static __maybe_unused int m_can_pci_resume(struct device *dev)
{
	return m_can_class_resume(dev);
}

static int __maybe_unused m_can_pci_runtime_suspend(struct device *dev)
{
	struct m_can_classdev *mcan_class = dev_get_drvdata(dev);
	u32 d0i3c_reg;
	u32 cgsr_reg;
	unsigned long j0, j1, delay;

	delay = msecs_to_jiffies(100);
	j0 = jiffies;
	j1 = j0 + delay;

	cgsr_reg = iomap_read_reg(mcan_class, M_CAN_CGSR);
	iomap_write_reg(mcan_class, M_CAN_CGSR, M_CAN_CGSR_CG);

	d0i3c_reg = iomap_read_reg(mcan_class, M_CAN_D0I3C);

	if (d0i3c_reg & M_CAN_D0I3_CIP) {
		dev_info(dev, "%s d0i3c CIP detected", __func__);
	} else {
		iomap_write_reg(mcan_class, M_CAN_D0I3C, M_CAN_D0I3_EN);
		d0i3c_reg = iomap_read_reg(mcan_class, M_CAN_D0I3C);
	}

	while (time_before(jiffies, j1)) {
		d0i3c_reg = iomap_read_reg(mcan_class, M_CAN_D0I3C);
		if (!(d0i3c_reg & M_CAN_D0I3_CIP))
			break;
	}

	if (d0i3c_reg & M_CAN_D0I3_CIP)
		dev_info(dev, "%s d0i3c timeout waiting CIP to be cleared", __func__);

	return 0;
}

static int __maybe_unused m_can_pci_runtime_resume(struct device *dev)
{
	struct m_can_classdev *mcan_class = dev_get_drvdata(dev);
	u32 d0i3c_reg;
	u32 cgsr_reg;

	cgsr_reg = iomap_read_reg(mcan_class, M_CAN_CGSR);

	if (cgsr_reg & M_CAN_CGSR_CG)
		iomap_write_reg(mcan_class, M_CAN_CGSR, (cgsr_reg & ~M_CAN_CGSR_CG));

	d0i3c_reg = iomap_read_reg(mcan_class, M_CAN_D0I3C);

	if (d0i3c_reg & M_CAN_D0I3_CIP) {
		dev_info(dev, "%s d0i3c CIP detected", __func__);
	} else {
		if (d0i3c_reg & M_CAN_D0I3_EN)
			d0i3c_reg &= ~M_CAN_D0I3_EN;

		if (d0i3c_reg & M_CAN_D0I3_RR)
			d0i3c_reg |= M_CAN_D0I3_RR;

		iomap_write_reg(mcan_class, M_CAN_D0I3C, d0i3c_reg);
		d0i3c_reg = iomap_read_reg(mcan_class, M_CAN_D0I3C);
	}

	return 0;
}

static const struct dev_pm_ops m_can_pci_pm_ops = {
	SET_RUNTIME_PM_OPS(m_can_pci_runtime_suspend,
			   m_can_pci_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(m_can_pci_suspend, m_can_pci_resume)
};

static const struct pci_device_id m_can_pci_id_table[] = {
	{ PCI_VDEVICE(INTEL, 0x4bc1), M_CAN_CLOCK_FREQ_EHL, },
	{ PCI_VDEVICE(INTEL, 0x4bc2), M_CAN_CLOCK_FREQ_EHL, },
	{  }	/* Terminating Entry */
};
MODULE_DEVICE_TABLE(pci, m_can_pci_id_table);

static struct pci_driver m_can_pci_driver = {
	.name = "m_can_pci",
	.probe = m_can_pci_probe,
	.remove = m_can_pci_remove,
	.id_table = m_can_pci_id_table,
	.driver = {
		.pm = &m_can_pci_pm_ops,
	},
};

module_pci_driver(m_can_pci_driver);

MODULE_AUTHOR("Felipe Balbi (Intel)");
MODULE_AUTHOR("Jarkko Nikula <jarkko.nikula@linux.intel.com>");
MODULE_AUTHOR("Raymond Tan <raymond.tan@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CAN bus driver for Bosch M_CAN controller on PCI bus");
