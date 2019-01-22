// SPDX-License-Identifier: GPL-2.0-only
/*
 * Watchdog driver for Intel Keem Bay non-secure watchdog.
 * Copyright (C) 2019 Intel Corporation
 */

#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/watchdog.h>

/* Non-secure watchdog register offsets */
#define TIM_WATCHDOG		0x0
	#define LOAD_MIN	0x00000001
	#define LOAD_MAX	0xffffffff
#define TIM_WATCHDOG_INT_THRES	0x4
#define TIM_WDOG_EN		0x8
	#define WDT_DISABLE	0x0
	#define WDT_ENABLE	0x1
#define TIM_SAFE		0xc
	#define WDT_UNLOCK	0xf1d0dead

#define WATCHDOG_TIMEOUT	5 /* seconds */

static unsigned int timeout = WATCHDOG_TIMEOUT;
module_param(timeout, int, 0);
MODULE_PARM_DESC(timeout, "Watchdog timeout period in seconds (default = "
		 __MODULE_STRING(WATCHDOG_TIMEOUT) ")");

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default = "
		 __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

struct keembay_wdt {
	void __iomem		*reg_base;
	struct watchdog_device	wdd;
	struct clk		*clk;
	u32			rate;
	int			to_irq;
	int			th_irq;
};

static inline u32 keembay_wdt_readl(struct keembay_wdt *keembay, u32 offset)
{
	return readl(keembay->reg_base + offset);
}

static inline void keembay_wdt_writel(struct keembay_wdt *keembay, u32 offset,
				      u32 val)
{
	writel(WDT_UNLOCK, keembay->reg_base + TIM_SAFE);
	writel(val, keembay->reg_base + offset);
}

static void keembay_wdt_set_timeout_reg(struct watchdog_device *wdog, bool ping)
{
	struct keembay_wdt *wdt = watchdog_get_drvdata(wdog);
	u32 th_val = 0;

	if (ping)
		keembay_wdt_writel(wdt, TIM_WATCHDOG,
				   wdog->timeout * wdt->rate);

	if (wdog->pretimeout != 0)
		th_val = wdog->timeout - wdog->pretimeout;
	keembay_wdt_writel(wdt, TIM_WATCHDOG_INT_THRES, th_val * wdt->rate);

	if (!ping)
		keembay_wdt_writel(wdt, TIM_WATCHDOG,
				   wdog->timeout * wdt->rate);
}

static int keembay_wdt_start(struct watchdog_device *wdog)
{
	struct keembay_wdt *wdt = watchdog_get_drvdata(wdog);

	keembay_wdt_set_timeout_reg(wdog, 0);
	keembay_wdt_writel(wdt, TIM_WDOG_EN, WDT_ENABLE);

	return 0;
}

static int keembay_wdt_stop(struct watchdog_device *wdog)
{
	struct keembay_wdt *wdt = watchdog_get_drvdata(wdog);

	keembay_wdt_writel(wdt, TIM_WDOG_EN, WDT_DISABLE);

	return 0;
}

static int keembay_wdt_ping(struct watchdog_device *wdog)
{
	keembay_wdt_set_timeout_reg(wdog, 1);

	return 0;
}

static int keembay_wdt_set_timeout(struct watchdog_device *wdog,
				   u32 t)
{
	u32 actual = min(t, wdog->max_timeout);

	wdog->timeout = actual;
	keembay_wdt_set_timeout_reg(wdog, 0);

	return 0;
}

static int keembay_wdt_set_pretimeout(struct watchdog_device *wdog,
				      u32 t)
{
	if (t != 0 && (t < wdog->min_timeout || t >= wdog->timeout))
		return -EINVAL;

	wdog->pretimeout = t;
	keembay_wdt_set_timeout_reg(wdog, 0);

	return 0;
}

static unsigned int keembay_wdt_get_timeleft(struct watchdog_device *wdog)
{
	struct keembay_wdt *wdt = watchdog_get_drvdata(wdog);

	return keembay_wdt_readl(wdt, TIM_WATCHDOG) / wdt->rate;
}

/*
 * Non-secure Watchdog TO/TH interrupt handler
 *
 * A53SS TIM_GEN_CONFIG register is in SECURE bank. Use SMC call to clear
 * the interrupt bits.
 *
 * The format to use is as below,
 * SMC(PLATFORM_SIP_SVC_WDT_ISR_CLEAR, mask, 0, 0, 0, 0, 0, 0);
 * where,
 * (1) PLATFORM_SIP_SVC_WDT_ISR_CLEAR= 0x8200ff18
 * (2) mask corresponds to bit position in hex
 *     where
 *     0x100: clear bit 8 (WDOG_TH_INT_CLR)
 *     0x200: clear bit 9 (WDOG_TO_INT_CLR)
 *     0x300: clear bit 8 and 9
 */
static irqreturn_t keembay_wdt_to_isr(int irq, void *dev_id)
{
	struct keembay_wdt *wdt = (struct keembay_wdt *)dev_id;
	struct arm_smccc_res res;

	/* write a new TIM_WATCHDOG value greater than 0 */
	keembay_wdt_writel(wdt, TIM_WATCHDOG, 0x1);
	/* clear bit 9 (WDOG_TO_INT_CLR) */
	arm_smccc_smc(0x8200ff18, 0x300, 0, 0, 0, 0, 0, 0, &res);

	/* print critical log message, and reboot */
	pr_crit("Intel Keem Bay non-secure watchdog timeout.\n");
	emergency_restart();

	return IRQ_HANDLED;
}

static irqreturn_t keembay_wdt_th_isr(int irq, void *dev_id)
{
	struct keembay_wdt *wdt = (struct keembay_wdt *)dev_id;
	struct arm_smccc_res res;
	u32 th_val = 0;

	/* write a new TIM_WATCHDOG value greater than TIM_WATCHDOG_INT_THRES */
	if (wdt->wdd.pretimeout != 0)
		th_val = wdt->wdd.timeout - wdt->wdd.pretimeout;
	keembay_wdt_writel(wdt, TIM_WATCHDOG, th_val * wdt->rate + 1);
	/* clear bit 8 (WDOG_TH_INT_CLR) */
	arm_smccc_smc(0x8200ff18, 0x300, 0, 0, 0, 0, 0, 0, &res);

	pr_crit("Intel Keem Bay non-secure watchdog pre-timeout.\n");
	watchdog_notify_pretimeout(&wdt->wdd);

	return IRQ_HANDLED;
}

static const struct watchdog_info keembay_wdt_info = {
	.identity	= "Intel Keem Bay Watchdog Timer",
	.options	= WDIOF_SETTIMEOUT |
			  WDIOF_PRETIMEOUT |
			  WDIOF_MAGICCLOSE |
			  WDIOF_KEEPALIVEPING,
};

static const struct watchdog_ops keembay_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= keembay_wdt_start,
	.stop		= keembay_wdt_stop,
	.ping		= keembay_wdt_ping,
	.set_timeout	= keembay_wdt_set_timeout,
	.set_pretimeout	= keembay_wdt_set_pretimeout,
	.get_timeleft	= keembay_wdt_get_timeleft,
};

static int keembay_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct keembay_wdt *wdt;
	int ret;

	wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	wdt->reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(wdt->reg_base))
		return PTR_ERR(wdt->reg_base);

	wdt->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(wdt->clk)) {
		dev_err(dev, "Clock not found.\n");
		return PTR_ERR(wdt->clk);
	}

	wdt->rate = clk_get_rate(wdt->clk);
	if (!wdt->rate) {
		dev_err(dev, "Failed to get clock rate.\n");
		return -EINVAL;
	}

	wdt->th_irq = platform_get_irq_byname(pdev, "threshold");
	if (wdt->th_irq < 0) {
		dev_err(dev, "Failed to get IRQ for threshold\n");
		return wdt->th_irq;
	}

	ret = devm_request_irq(&pdev->dev, wdt->th_irq, keembay_wdt_th_isr, 0,
			       "keembay-wdt", pdev);
	if (ret) {
		dev_err(dev, "Failed to request IRQ for threshold\n");
		return ret;
	}

	wdt->to_irq = platform_get_irq_byname(pdev, "timeout");
	if (wdt->to_irq < 0) {
		dev_err(dev, "Failed to get IRQ for timeout\n");
		return wdt->to_irq;
	}

	ret = devm_request_irq(&pdev->dev, wdt->to_irq, keembay_wdt_to_isr, 0,
			       "keembay-wdt", pdev);
	if (ret) {
		dev_err(dev, "Failed to request IRQ for timeout\n");
		return ret;
	}

	wdt->wdd.parent		= &pdev->dev;
	wdt->wdd.info		= &keembay_wdt_info;
	wdt->wdd.ops		= &keembay_wdt_ops;
	wdt->wdd.min_timeout	= 1;
	wdt->wdd.max_timeout	= 0xffffffff / wdt->rate;
	wdt->wdd.timeout	= WATCHDOG_TIMEOUT;
	watchdog_set_drvdata(&wdt->wdd, wdt);

	watchdog_set_nowayout(&wdt->wdd, nowayout);
	watchdog_init_timeout(&wdt->wdd, timeout, &pdev->dev);
	keembay_wdt_set_timeout(&wdt->wdd, wdt->wdd.timeout);

	ret = watchdog_register_device(&wdt->wdd);
	if (ret) {
		dev_err(dev, "Failed to register watchdog device.\n");
		return ret;
	}

	platform_set_drvdata(pdev, wdt);
	dev_info(dev, "Initial timeout %d sec%s\n",
		 wdt->wdd.timeout, nowayout ? ", nowayout." : ".");

	return 0;
}

static int __maybe_unused keembay_wdt_suspend(struct device *dev)
{
	struct keembay_wdt *wdt = dev_get_drvdata(dev);

	if (watchdog_active(&wdt->wdd))
		return keembay_wdt_stop(&wdt->wdd);

	return 0;
}

static int __maybe_unused keembay_wdt_resume(struct device *dev)
{
	struct keembay_wdt *wdt = dev_get_drvdata(dev);

	if (watchdog_active(&wdt->wdd))
		return keembay_wdt_start(&wdt->wdd);

	return 0;
}

static SIMPLE_DEV_PM_OPS(keembay_wdt_pm_ops, keembay_wdt_suspend,
			 keembay_wdt_resume);

#ifdef CONFIG_OF
static const struct of_device_id keembay_wdt_match[] = {
	{ .compatible = "intel,keembay-wdt" },
	{ },
};
MODULE_DEVICE_TABLE(of, keembay_wdt_match);
#endif

static struct platform_driver keembay_wdt_driver = {
	.probe		= keembay_wdt_probe,
	.driver		= {
		.name		= "keembay_wdt",
		.of_match_table	= keembay_wdt_match,
		.pm		= &keembay_wdt_pm_ops,
	},
};
module_platform_driver(keembay_wdt_driver);

MODULE_DESCRIPTION("Intel Keem Bay SoC watchdog driver");
MODULE_AUTHOR("Wan Ahmad Zainie <wan.ahmad.zainie.wan.mohamad@intel.com");
MODULE_LICENSE("GPL v2");

