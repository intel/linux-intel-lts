// SPDX-License-Identifier: GPL-2.0
/**
 * pwm-dwc.c - DesignWare PWM Controller
 *
 * Copyright (C) 2018 Intel Corporation
 *
 * Author: Felipe Balbi <felipe.balbi@linux.intel.com>
 */

#include <linux/bitops.h>
#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <linux/pwm.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>

#define PCI_DEVICE_ID_EHLLP	0x4bb7

#define DWC_TIM_LD_CNT(n)	((n) * 0x14)
#define DWC_TIM_LD_CNT2(n)	(((n) * 4) + 0xb0)
#define DWC_TIM_CUR_VAL(n)	(((n) * 0x14) + 0x04)
#define DWC_TIM_CTRL(n)		(((n) * 0x14) + 0x08)
#define DWC_TIM_EOI(n)		(((n) * 0x14) + 0x0c)
#define DWC_TIM_INT_STS(n)	(((n) * 0x14) + 0x10)

#define DWC_TIMERS_INT_STS	0xa0
#define DWC_TIMERS_EOI		0xa4
#define DWC_TIMERS_RAW_INT_STS	0xa8
#define DWC_TIMERS_COMP_VERSION	0xac

#define DWC_TIMERS_TOTAL  8

/* Timer Control Register */
#define DWC_TIM_CTRL_EN		BIT(0)
#define DWC_TIM_CTRL_MODE	BIT(1)
#define DWC_TIM_CTRL_MODE_FREE	(0 << 1)
#define DWC_TIM_CTRL_MODE_USER	(1 << 1)
#define DWC_TIM_CTRL_INT_MASK	BIT(2)
#define DWC_TIM_CTRL_PWM	BIT(3)

struct dwc_pwm_driver_data {
	unsigned long clk_period_ns;
	int npwm;
};

struct dwc_pwm {
	struct pwm_chip pwm;
	struct device *dev;
	struct mutex lock;

	unsigned long clk_period_ns;
	unsigned int version;

	void __iomem *base;

	u32 saved_registers[24];
};
#define to_dwc(p)	(container_of((p), struct dwc_pwm, pwm))

static inline u32 dwc_readl(void __iomem *base, u32 offset)
{
	return readl(base + offset);
}

static inline void dwc_writel(void __iomem *base, u32 offset, u32 value)
{
	writel(value, base + offset);
}

static void __dwc_configure(struct dwc_pwm *dwc, int pwm, int duty_ns,
		int period_ns)
{
	u32 ctrl;
	u32 high;
	u32 low;

	high = DIV_ROUND_CLOSEST(duty_ns, dwc->clk_period_ns) - 1;
	low = DIV_ROUND_CLOSEST(period_ns - duty_ns, dwc->clk_period_ns) - 1;

	dwc_writel(dwc->base, DWC_TIM_LD_CNT(pwm), low);
	dwc_writel(dwc->base, DWC_TIM_LD_CNT2(pwm), high);

	ctrl = DWC_TIM_CTRL_MODE_USER | DWC_TIM_CTRL_PWM;
	dwc_writel(dwc->base, DWC_TIM_CTRL(pwm), ctrl);
}

static u32 __dwc_duty_ns(struct dwc_pwm *dwc, int pwm)
{
	u32 duty;

	duty = dwc_readl(dwc->base, DWC_TIM_LD_CNT2(pwm));
	duty *= dwc->clk_period_ns;
	duty += 1;

	return duty;
}

static u32 __dwc_period_ns(struct dwc_pwm *dwc, int pwm, u32 duty)
{
	u32 period;

	period = dwc_readl(dwc->base, DWC_TIM_LD_CNT(pwm));
	period *= dwc->clk_period_ns;
	period += 1 + duty;

	return period;
}

static bool __dwc_is_enabled(struct dwc_pwm *dwc, int pwm)
{
	return !!dwc_readl(dwc->base, DWC_TIM_CTRL(pwm)) & DWC_TIM_CTRL_EN;
}

static void __dwc_set_enable(struct dwc_pwm *dwc, int pwm, int enabled)
{
	u32 reg;

	reg = dwc_readl(dwc->base, DWC_TIM_CTRL(pwm));

	if (enabled)
		reg |= DWC_TIM_CTRL_EN;
	else
		reg &= ~DWC_TIM_CTRL_EN;

	dwc_writel(dwc->base, DWC_TIM_CTRL(pwm), reg);
}

static int dwc_pwm_apply(struct pwm_chip *pwm, struct pwm_device *pdev,
		const struct pwm_state *state)
{
	struct dwc_pwm *dwc = to_dwc(pwm);

	mutex_lock(&dwc->lock);
	if (state->enabled) {
		if (!pwm_is_enabled(pdev))
			pm_runtime_get_sync(dwc->dev);
	} else if (pwm_is_enabled(pdev)) {
		pm_runtime_mark_last_busy(dwc->dev);
		pm_runtime_put_autosuspend(dwc->dev);
	}
	__dwc_configure(dwc, pdev->hwpwm, state->duty_cycle, state->period);
	__dwc_set_enable(dwc, pdev->hwpwm, state->enabled);
	mutex_unlock(&dwc->lock);

	return 0;
}

static void dwc_pwm_get_state(struct pwm_chip *pwm, struct pwm_device *pdev,
			  struct pwm_state *state)
{
	struct dwc_pwm *dwc = to_dwc(pwm);

	mutex_lock(&dwc->lock);
	state->enabled = __dwc_is_enabled(dwc, pdev->hwpwm);
	state->duty_cycle = __dwc_duty_ns(dwc, pdev->hwpwm);
	state->period = __dwc_period_ns(dwc, pdev->hwpwm, state->duty_cycle);
	mutex_unlock(&dwc->lock);
}

static const struct pwm_ops dwc_pwm_ops = {
	.apply		= dwc_pwm_apply,
	.get_state	= dwc_pwm_get_state,
	.owner		= THIS_MODULE,
};

static int dwc_pci_probe(struct pci_dev *pci, const struct pci_device_id *id)
{
	struct dwc_pwm_driver_data *data;
	struct dwc_pwm *dwc;
	struct device *dev;
	int ret;
	int i;

	data = (struct dwc_pwm_driver_data *) id->driver_data;
	dev = &pci->dev;

	dwc = devm_kzalloc(&pci->dev, sizeof(*dwc), GFP_KERNEL);
	if (!dwc) {
		ret = -ENOMEM;
		goto err0;
	}

	dwc->dev = dev;
	dwc->clk_period_ns = data->clk_period_ns;

	ret = pcim_enable_device(pci);
	if (ret)
		return ret;

	pci_set_master(pci);

	ret = pcim_iomap_regions(pci, BIT(0), pci_name(pci));
	if (ret)
		return ret;

	dwc->base = pcim_iomap_table(pci)[0];
	if (!dwc->base) {
		ret = -ENOMEM;
		goto err1;
	}

	dwc->version = dwc_readl(dwc->base, DWC_TIMERS_COMP_VERSION);

	/* mask all interrupts and disable all timers */
	for (i = 0; i < data->npwm; i++) {
		dwc_writel(dwc->base, DWC_TIM_CTRL(i), 0);
		dwc_writel(dwc->base, DWC_TIM_LD_CNT(i), 0);
		dwc_writel(dwc->base, DWC_TIM_CUR_VAL(i), 0);
	}

	mutex_init(&dwc->lock);
	pci_set_drvdata(pci, dwc);

	dwc->pwm.dev = dev;
	dwc->pwm.ops = &dwc_pwm_ops;
	dwc->pwm.npwm = data->npwm;
	dwc->pwm.base = -1;

	ret = pwmchip_add(&dwc->pwm);
	if (ret)
		goto err2;

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_put_noidle(dev);
	pm_runtime_allow(dev);

	return 0;

err2:
	pci_iounmap(pci, dwc->base);

err1:
	pci_release_region(pci, 0);

err0:
	return ret;
}

static void dwc_pci_remove(struct pci_dev *pci)
{
	struct dwc_pwm *dwc = pci_get_drvdata(pci);
	int i;

	pm_runtime_forbid(&pci->dev);
	pm_runtime_get_noresume(&pci->dev);

	for (i = 0; i < dwc->pwm.npwm; i++)
		pwm_disable(&dwc->pwm.pwms[i]);

	pwmchip_remove(&dwc->pwm);
	pci_iounmap(pci, dwc->base);
	pci_release_region(pci, 0);
}

#ifdef CONFIG_PM_SLEEP
static int dwc_pci_suspend(struct device *dev)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct dwc_pwm *dwc = pci_get_drvdata(pdev);
	int i, index_base;

	for (i = 0; i < DWC_TIMERS_TOTAL; i++) {
		index_base = i * 3;
		dwc->saved_registers[index_base] =
			dwc_readl(dwc->base, DWC_TIM_LD_CNT(i));
		dwc->saved_registers[index_base+1] =
			dwc_readl(dwc->base, DWC_TIM_LD_CNT2(i));
		dwc->saved_registers[index_base+2] =
			dwc_readl(dwc->base, DWC_TIM_CTRL(i));
	}

	return 0;
}

static int dwc_pci_resume(struct device *dev)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct dwc_pwm *dwc = pci_get_drvdata(pdev);
	int i, index_base;

	for (i = 0; i < DWC_TIMERS_TOTAL; i++) {
		index_base = i * 3;
		dwc_writel(dwc->base, DWC_TIM_LD_CNT(i),
			dwc->saved_registers[index_base]);
		dwc_writel(dwc->base, DWC_TIM_LD_CNT2(i),
			dwc->saved_registers[index_base+1]);
		dwc_writel(dwc->base, DWC_TIM_CTRL(i),
			dwc->saved_registers[index_base+2]);
	}

	return 0;
}

static int dwc_pci_runtime_suspend(struct device *dev)
{
	/*
	 * The PCI core will handle transition to D3 automatically. We only
	 * need to provide runtime PM hooks for that to happen.
	 */
	return 0;
}

static int dwc_pci_runtime_resume(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops pwm_dwc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dwc_pci_suspend, dwc_pci_resume)
	SET_RUNTIME_PM_OPS(dwc_pci_runtime_suspend,
			   dwc_pci_runtime_resume, NULL)
};

static const struct dwc_pwm_driver_data ehl_driver_data = {
	.npwm = 8,
	.clk_period_ns = 10,
};

static const struct pci_device_id dwc_pci_id_table[] = {
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_EHLLP),
	  (kernel_ulong_t) &ehl_driver_data,
	},
	{  }	/* Terminating Entry */
};
MODULE_DEVICE_TABLE(pci, dwc_pci_id_table);

static struct pci_driver dwc_pwm_driver = {
	.name		= "pwm-dwc",
	.probe		= dwc_pci_probe,
	.remove		= dwc_pci_remove,
	.id_table	= dwc_pci_id_table,
	.driver = {
		.pm = &pwm_dwc_pm_ops,
	},
};

module_pci_driver(dwc_pwm_driver);

MODULE_AUTHOR("Felipe Balbi <felipe.balbi@linux.intel.com>");
MODULE_DESCRIPTION("DesignWare PWM Controller");
MODULE_LICENSE("GPL v2");
