// SPDX-License-Identifier: GPL-2.0
/*
 * Intel Keem Bay PWM driver
 *
 * Copyright (C) 2020 Intel Corporation
 * Authors: Lai Poey Seng <poey.seng.lai@intel.com>
 *          Vineetha G. Jaya Kumaran <vineetha.g.jaya.kumaran@intel.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/regmap.h>

#define TOTAL_PWM_CHANNELS		6
#define LEAD_IN_DEFAULT			0
#define PWM_COUNT_MAX			65535

#define KEEMBAY_PWM_EN_BIT		31

/* Mask */
#define KEEMBAY_PWM_RPT_CNT_MASK	GENMASK(15, 0)
#define KEEMBAY_PWM_LEAD_IN_MASK	GENMASK(30, 16)
#define KEEMBAY_PWM_HIGH_MASK		GENMASK(31, 16)
#define KEEMBAY_PWM_LOW_MASK		GENMASK(15, 0)

/* PWM Register offset */
#define PWM_LEADIN0_OFFSET		0x00
#define PWM_LEADIN1_OFFSET		0x04
#define PWM_LEADIN2_OFFSET		0x08
#define PWM_LEADIN3_OFFSET		0x0c
#define PWM_LEADIN4_OFFSET		0x10
#define PWM_LEADIN5_OFFSET		0x14

#define PWM_HIGHLOW0_OFFSET		0x20
#define PWM_HIGHLOW1_OFFSET		0x24
#define PWM_HIGHLOW2_OFFSET		0x28
#define PWM_HIGHLOW3_OFFSET		0x2c
#define PWM_HIGHLOW4_OFFSET		0x30
#define PWM_HIGHLOW5_OFFSET		0x34

struct keembay_pwm {
	struct pwm_chip chip;
	struct device *dev;
	struct clk *clk;
	void __iomem *regmap;
};

static inline struct keembay_pwm *to_keembay_pwm_dev(struct pwm_chip *chip)
{
	return container_of(chip, struct keembay_pwm, chip);
}

static inline void keembay_pwm_enable_channel(struct keembay_pwm *priv, int ch)
{
	u32 buff, offset;
	void __iomem *address;

	offset = PWM_LEADIN0_OFFSET + ch * 4;
	address = priv->regmap + offset;
	buff = readl(address);
	buff |= BIT(KEEMBAY_PWM_EN_BIT);
	writel(buff, address);
}

static inline void keembay_pwm_disable_channel(struct keembay_pwm *priv, int ch)
{
	u32 buff, offset;
	void __iomem *address;

	offset = PWM_LEADIN0_OFFSET + ch * 4;
	address = priv->regmap + offset;
	buff = readl(address);
	buff &= ~BIT(KEEMBAY_PWM_EN_BIT);
	writel(buff, address);
}

static inline void keembay_pwm_update_bits(struct keembay_pwm *priv, u32 mask,
					   u32 val, u32 reg, int ch)
{
	u32 buff, offset, tmp;
	void __iomem *address;

	offset = reg + ch * 4;
	address = priv->regmap + offset;
	buff = readl(address);
	tmp = buff & ~mask;
	tmp |= FIELD_PREP(mask, val);
	writel(tmp, address);
}

static inline u32 keembay_pwm_config_min(struct keembay_pwm *priv)
{
	unsigned long long divd, divs;

	divd = NSEC_PER_SEC;
	divs = clk_get_rate(priv->clk);
	do_div(divd, divs);

	return (u32)divd;
}

static inline u16 keembay_pwm_config_duty_cycle(struct keembay_pwm *priv,
						int duty_ns, u32 ns_min)
{
	unsigned long long divd;

	divd = duty_ns;
	do_div(divd, ns_min);
	if ((u16)divd == 0)
		return 0;

	return (u16)divd - 1;
}

static inline u16 keembay_pwm_config_period(struct keembay_pwm *priv,
					    int period_ns,
					    int duty_ns,
					    u32 ns_min)
{
	unsigned long long divd;

	divd = period_ns - duty_ns;
	do_div(divd, ns_min);
	if ((u16)divd == 0)
		return 0;

	return (u16)divd - 1;
}

/*
 *	For calculating "high time" register value:
 *	High time (quotient only) = duty_cycle / ns_min
 *
 *	For calculating "low time" register value:
 *	Low time (quotient only) = (period - duty_cycle) / ns_min
 *
 *	All values used are in nanoseconds for calculation.
 */
static int keembay_pwm_config(struct keembay_pwm *priv, int ch,
			      int duty_ns, int period_ns, int count)
{
	u32 ns_min;
	u16 pwm_h_count, pwm_l_count;

	/* Write to lead in */
	keembay_pwm_update_bits(priv, KEEMBAY_PWM_LEAD_IN_MASK,
				LEAD_IN_DEFAULT,
				PWM_LEADIN0_OFFSET, ch);

	/* Write the number of PWM pulse repetition */
	keembay_pwm_update_bits(priv, KEEMBAY_PWM_RPT_CNT_MASK, count,
				PWM_LEADIN0_OFFSET, ch);

	/* Calculate min */
	ns_min = keembay_pwm_config_min(priv);

	/* For duty cycle */
	pwm_h_count = keembay_pwm_config_duty_cycle(priv, duty_ns, ns_min);

	/* Write to high registers */
	keembay_pwm_update_bits(priv, KEEMBAY_PWM_HIGH_MASK, pwm_h_count,
				PWM_HIGHLOW0_OFFSET, ch);

	/* For period */
	pwm_l_count = keembay_pwm_config_period(priv, period_ns, duty_ns,
						ns_min);

	/* Write to low registers */
	keembay_pwm_update_bits(priv, KEEMBAY_PWM_LOW_MASK, pwm_l_count,
				PWM_HIGHLOW0_OFFSET, ch);

	return 0;
}

static int keembay_pwm_enable(struct keembay_pwm *priv, int ch)
{
	int ret;

	ret = clk_enable(priv->clk);
	if (ret)
		return ret;

	/* Enable channel */
	keembay_pwm_enable_channel(priv, ch);

	return 0;
}

static void keembay_pwm_disable(struct keembay_pwm *priv, int ch)
{
	/* Disable channel */
	keembay_pwm_disable_channel(priv, ch);

	clk_disable(priv->clk);
}

static int keembay_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			     const struct pwm_state *state)
{
	struct keembay_pwm *priv = to_keembay_pwm_dev(chip);

	if (!state->enabled && pwm_is_enabled(pwm)) {
		keembay_pwm_disable(priv, pwm->hwpwm);
		return 0;
	}

	if (state->count > PWM_COUNT_MAX)
		return -EINVAL;

	if (state->polarity != pwm_get_polarity(pwm))
		return -ENOSYS;

	keembay_pwm_config(priv, pwm->hwpwm, state->duty_cycle,
			   state->period, state->count);

	if (state->enabled && !pwm_is_enabled(pwm))
		return keembay_pwm_enable(priv, pwm->hwpwm);

	return 0;
}

static const struct pwm_ops keembay_pwm_ops = {
	.owner = THIS_MODULE,
	.apply = keembay_pwm_apply,
};

static int keembay_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct keembay_pwm *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(priv->clk))
		return PTR_ERR(priv->clk);

	/*
	 * Prepare clock here, and carry out clock enabling/disabling
	 * during channel enablement/disablement.
	 * The clock will not be unprepared due to shared usage with GPIO.
	 */
	ret = clk_prepare(priv->clk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to prepare PWM clock\n");
		return ret;
	}

	priv->regmap = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	priv->chip.base = -1;
	priv->chip.dev = dev;
	priv->chip.ops = &keembay_pwm_ops;
	priv->chip.npwm = TOTAL_PWM_CHANNELS;

	ret = pwmchip_add(&priv->chip);
	if (ret < 0) {
		dev_err(dev, "Failed to add PWM chip: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, priv);

	return 0;
}

static int keembay_pwm_remove(struct platform_device *pdev)
{
	struct keembay_pwm *priv = platform_get_drvdata(pdev);
	unsigned int i;

	for (i = 0; i < priv->chip.npwm; i++)
		pwm_disable(&priv->chip.pwms[i]);

	pwmchip_remove(&priv->chip);

	return 0;
}

static const struct of_device_id keembay_pwm_of_match[] = {
	{ .compatible = "intel,keembay-pwm" },
	{ }
};
MODULE_DEVICE_TABLE(of, keembay_pwm_of_match);

static struct platform_driver keembay_pwm_driver = {
	.probe	= keembay_pwm_probe,
	.remove	= keembay_pwm_remove,
	.driver	= {
		.name = "pwm-keembay",
		.of_match_table = keembay_pwm_of_match,
	},
};
module_platform_driver(keembay_pwm_driver);

MODULE_ALIAS("platform:keembay");
MODULE_DESCRIPTION("Intel Keem Bay PWM driver");
MODULE_LICENSE("GPL v2");
