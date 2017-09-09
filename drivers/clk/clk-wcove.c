/*
 * WhiskeyCove PMIC 32kHz clock
 *
 * Copyright (C) 2017, Intel Corporation
 * Authors: Jukka Laitinen <jukka.laitinen@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/acpi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/core.h>
#include <linux/mfd/intel_soc_pmic_bxtwc.h>
#include <linux/mfd/intel_soc_pmic.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>

/* Clock configuration register */
#define BXTWC_CLKCONFIG		0x4FCC
#define BXTWC_SLP0D		BIT(0)
#define BXTWC_SLP0DLVL		BIT(1)
#define BXTWC_SLP1D		BIT(2)
#define BXTWC_SLP1DLVL		BIT(3)
#define BXTWC_SLP2D		BIT(4)
#define BXTWC_SLP2DLVL		BIT(5)

#define to_clk_wcove(_hw) container_of(_hw, struct clk_wcove, hw)

struct clk_wcove {
	struct clk_hw hw;
	struct intel_soc_pmic *pmic;
	struct clk_lookup* clock;
	int is_enabled;
	unsigned long fixed_rate;
	unsigned long fixed_accuracy;
};

static int clk_wcove_enable(struct clk_hw *hw)
{
	to_clk_wcove(hw)->is_enabled = 1;
	return 0;
};

void clk_wcove_disable(struct clk_hw *hw)
{
	to_clk_wcove(hw)->is_enabled = 0;
};

static int clk_wcove_is_enabled(struct clk_hw *hw)
{
	return to_clk_wcove(hw)->is_enabled;
}

static int clk_wcove_prepare(struct clk_hw *hw)
{
	struct clk_wcove *clk = to_clk_wcove(hw);
	/* Enable sleep clock */
	regmap_update_bits(clk->pmic->regmap, BXTWC_CLKCONFIG,
			   BXTWC_SLP1D, 0);
	return 0;
};

void clk_wcove_unprepare(struct clk_hw *hw)
{
	struct clk_wcove *clk = to_clk_wcove(hw);
	/* Disable sleep clock */
	regmap_update_bits(clk->pmic->regmap, BXTWC_CLKCONFIG,
			   BXTWC_SLP1D, BXTWC_SLP1D);
};

static unsigned long clk_wcove_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	return to_clk_wcove(hw)->fixed_rate;
}

static unsigned long clk_wcove_recalc_accuracy(struct clk_hw *hw,
		unsigned long parent_accuracy)
{
	return to_clk_wcove(hw)->fixed_accuracy;
}

const struct clk_ops clk_wcove_ops = {
	.enable = clk_wcove_enable,
	.disable = clk_wcove_disable,
	.prepare = clk_wcove_prepare,
	.unprepare = clk_wcove_unprepare,
	.is_enabled = clk_wcove_is_enabled,
	.recalc_rate = clk_wcove_recalc_rate,
	.recalc_accuracy = clk_wcove_recalc_accuracy,
};




static int wcove_clk_probe(struct platform_device *pdev)
{
	struct clk *clk;
	struct clk_wcove *clk_wc;
	struct clk_init_data clk_init;

	clk_wc = devm_kzalloc(&pdev->dev, sizeof(*clk_wc), GFP_KERNEL);
	if (!clk_wc)
		return -ENOMEM;

	clk_init.ops = &clk_wcove_ops;
	clk_init.flags = CLK_IS_BASIC;
	clk_init.name = "wcove_32k_clk";
	clk_init.num_parents = 0;
	clk_wc->pmic = dev_get_drvdata(pdev->dev.parent);
	clk_wc->fixed_rate = 32768;
	clk_wc->fixed_accuracy = 0;
	clk_wc->hw.init = &clk_init;

	clk = clk_register(&pdev->dev, &clk_wc->hw);
	if (IS_ERR(clk)) {
		devm_kfree(&pdev->dev, clk_wc);
		return PTR_ERR(clk);
	}
	dev_set_drvdata(&pdev->dev, clk_wc);
	clk_wc->clock = clkdev_create(clk, "wcove_32k_clk", NULL);

	return 0;
}

static int wcove_clk_remove(struct platform_device *pdev)
{
	struct clk_wcove *clk_wc = dev_get_drvdata(&pdev->dev);
	clkdev_drop(clk_wc->clock);
	clk_unregister(clk_wc->hw.clk);
	return 0;
}

static struct platform_driver intel_wc_clk_driver = {
	.driver = {
		.name = "clk_wcove",
	},
	.probe = wcove_clk_probe,
	.remove = wcove_clk_remove,
};

module_platform_driver(intel_wc_clk_driver);

MODULE_AUTHOR("Jukka Laitinen <jukka.laitinen@intel.com>");
MODULE_DESCRIPTION("WhiskeyCove 32kHz clock driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:clk_wcove");
