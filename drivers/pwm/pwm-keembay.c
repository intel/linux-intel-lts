// SPDX-License-Identifier: GPL-2.0
/*
 * Intel Keem Bay PWM driver
 *
 * Copyright (C) 2020 Intel Corporation
 * Authors: Lai Poey Seng <poey.seng.lai@intel.com>
 *          Vineetha G. Jaya Kumaran <vineetha.g.jaya.kumaran@intel.com>
 *
 * Limitation:
 * - Upon disabling a channel, the currently running
 *   period will not be completed. However, upon
 *   reconfiguration of the duty cycle/period, the
 *   currently running period will be completed first.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/regmap.h>

#define KMB_TOTAL_PWM_CHANNELS		6
#define KMB_PWM_COUNT_MAX		0xffff
#define KMB_PWM_EN_BIT			BIT(31)

/* Mask */
#define KMB_PWM_HIGH_MASK		GENMASK(31, 16)
#define KMB_PWM_LOW_MASK		GENMASK(15, 0)
#define KMB_PWM_COUNT_MASK		GENMASK(31, 0)

/* PWM Register offset */
#define KMB_PWM_LEADIN_OFFSET(ch)	(0x00 + 4 * (ch))
#define KMB_PWM_HIGHLOW_OFFSET(ch)	(0x20 + 4 * (ch))

struct keembay_pwm {
	struct pwm_chip chip;
	struct device *dev;
	struct clk *clk;
	void __iomem *base;
};

static inline struct keembay_pwm *to_keembay_pwm_dev(struct pwm_chip *chip)
{
	return container_of(chip, struct keembay_pwm, chip);
}

static inline void keembay_pwm_update_bits(struct keembay_pwm *priv, u32 mask,
					   u32 val, u32 offset)
{
	u32 buff = readl(priv->base + offset);

	buff = u32_replace_bits(buff, val, mask);
	writel(buff, priv->base + offset);
}

static void keembay_pwm_enable(struct keembay_pwm *priv, int ch)
{
	keembay_pwm_update_bits(priv, KMB_PWM_EN_BIT, 1,
				KMB_PWM_LEADIN_OFFSET(ch));
}

static void keembay_pwm_disable(struct keembay_pwm *priv, int ch)
{
	keembay_pwm_update_bits(priv, KMB_PWM_EN_BIT, 0,
				KMB_PWM_LEADIN_OFFSET(ch));
}

static void keembay_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
				  struct pwm_state *state)
{
	struct keembay_pwm *priv = to_keembay_pwm_dev(chip);
	unsigned long long pwm_h_count, pwm_l_count;
	unsigned long clk_rate;
	u32 buff;

	clk_rate = clk_get_rate(priv->clk);

	/* Read channel enabled status */
	buff = readl(priv->base + KMB_PWM_LEADIN_OFFSET(pwm->hwpwm));
	if (buff & KMB_PWM_EN_BIT)
		state->enabled = true;
	else
		state->enabled = false;

	/* Read period and duty cycle */
	buff = readl(priv->base + KMB_PWM_HIGHLOW_OFFSET(pwm->hwpwm));
	pwm_l_count = FIELD_GET(KMB_PWM_LOW_MASK, buff) * NSEC_PER_SEC;
	pwm_h_count = FIELD_GET(KMB_PWM_HIGH_MASK, buff) * NSEC_PER_SEC;
	state->duty_cycle = DIV_ROUND_UP_ULL(pwm_h_count, clk_rate);
	state->period = DIV_ROUND_UP_ULL(pwm_h_count + pwm_l_count, clk_rate);
}

static int keembay_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			     const struct pwm_state *state)
{
	struct keembay_pwm *priv = to_keembay_pwm_dev(chip);
	struct pwm_state current_state;
	u16 pwm_h_count, pwm_l_count;
	unsigned long long div;
	unsigned long clk_rate;
	u32 pwm_count = 0;

	keembay_pwm_get_state(chip, pwm, &current_state);

	if (state->polarity != PWM_POLARITY_NORMAL)
		return -ENOSYS;

	if (!state->enabled && current_state.enabled) {
		keembay_pwm_disable(priv, pwm->hwpwm);
		return 0;
	}

	/*
	 * The upper 16 bits of the KMB_PWM_HIGHLOW_OFFSET register contain
	 * the high time of the waveform, while the last 16 bits contain
	 * the low time of the waveform, in terms of clock cycles.
	 *
	 * high time = clock rate * duty cycle
	 * low time =  clock rate * (period - duty cycle)
	 *
	 * e.g. For period 50us, duty cycle 30us, and clock rate 500MHz:
	 * high time = 500MHz * 30us = 0x3A98
	 * low time = 500MHz * 20us = 0x2710
	 * Value written to KMB_PWM_HIGHLOW_OFFSET = 0x3A982710
	 */

	clk_rate = clk_get_rate(priv->clk);

	/* Configure waveform high time */
	div = clk_rate * state->duty_cycle;
	div = DIV_ROUND_CLOSEST_ULL(div, NSEC_PER_SEC);
	if (div > KMB_PWM_COUNT_MAX)
		return -ERANGE;

	pwm_h_count = div;

	/* Configure waveform low time */
	div = clk_rate * (state->period - state->duty_cycle);
	div = DIV_ROUND_CLOSEST_ULL(div, NSEC_PER_SEC);
	if (div > KMB_PWM_COUNT_MAX)
		return -ERANGE;

	pwm_l_count = div;

	pwm_count = FIELD_PREP(KMB_PWM_HIGH_MASK, pwm_h_count) |
		    FIELD_PREP(KMB_PWM_LOW_MASK, pwm_l_count);

	writel(pwm_count, priv->base + KMB_PWM_HIGHLOW_OFFSET(pwm->hwpwm));

	if (state->enabled && !current_state.enabled)
		keembay_pwm_enable(priv, pwm->hwpwm);

	return 0;
}

static const struct pwm_ops keembay_pwm_ops = {
	.owner = THIS_MODULE,
	.apply = keembay_pwm_apply,
	.get_state = keembay_pwm_get_state,
};

static int keembay_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct keembay_pwm *priv;
	int ret, ch;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(priv->clk)) {
		dev_err(dev, "Failed to get clock\n");
		return PTR_ERR(priv->clk);
	}

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->chip.base = -1;
	priv->chip.dev = dev;
	priv->chip.ops = &keembay_pwm_ops;
	priv->chip.npwm = KMB_TOTAL_PWM_CHANNELS;

	ret = pwmchip_add(&priv->chip);
	if (ret) {
		dev_err(dev, "Failed to add PWM chip: %pe\n", ERR_PTR(ret));
		return ret;
	}

	/* Ensure enable bit for each channel is cleared at boot */
	for (ch = 0; ch < KMB_TOTAL_PWM_CHANNELS; ch++)
		keembay_pwm_disable(priv, ch);

	platform_set_drvdata(pdev, priv);

	return 0;
}

static int keembay_pwm_remove(struct platform_device *pdev)
{
	struct keembay_pwm *priv = platform_get_drvdata(pdev);

	return pwmchip_remove(&priv->chip);
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

MODULE_ALIAS("platform:pwm-keembay");
MODULE_DESCRIPTION("Intel Keem Bay PWM driver");
MODULE_LICENSE("GPL v2");
