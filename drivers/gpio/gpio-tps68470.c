/*
 * gpio-tps68470.c
 *
 * GPIO driver for TPS68470 PMIC
 *
 * Copyright (C) 2017 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 17, 0)
#include <linux/gpio/driver.h>
#else
#include <linux/gpio/machine.h>
#endif
#include <linux/mfd/core.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/mfd/tps68470.h>

/*
 * s_enable = gpio7
 * s_idle   = gpio8
 * s_resetn = gpio9
 */
#define TPS68470_N_LOGIC_OUTPUT	3
#define TPS68470_N_REGULAR_GPIO	7
/* FIXME: hiding daisy-chain config as gpio for now */
#define TPS68470_N_I2C_DC_AS_GPIO 1
#define TPS68470_N_GPIO	(TPS68470_N_LOGIC_OUTPUT + TPS68470_N_REGULAR_GPIO + \
			 TPS68470_N_I2C_DC_AS_GPIO)

struct tps68470_gpio_data {
	struct tps68470 *tps68470;
	struct gpio_chip gc;
};

#define to_gpio_data(gc) container_of(gc, struct tps68470_gpio_data, gc)

static int tps68470_gpio_get(struct gpio_chip *gc, unsigned offset)
{
	struct tps68470_gpio_data *tps68470_gpio = to_gpio_data(gc);
	struct tps68470 *tps = tps68470_gpio->tps68470;
	unsigned int reg;
	int val, ret;

	if (offset >= TPS68470_N_GPIO)
		return -EINVAL;

	if (offset >= TPS68470_N_REGULAR_GPIO &&
	    offset < TPS68470_N_LOGIC_OUTPUT + TPS68470_N_REGULAR_GPIO) {
		reg = TPS68470_REG_SGPO;
		offset -= TPS68470_N_REGULAR_GPIO;
	} else if (offset >=
		   TPS68470_N_LOGIC_OUTPUT + TPS68470_N_REGULAR_GPIO) {
		/* read i2c daisy-chaining status */
		ret = tps68470_reg_read(tps, TPS68470_REG_S_I2C_CTL, &val);
		return val & 0x01;
	}

	ret = tps68470_reg_read(tps, reg, &val);
	if (ret) {
		dev_err(tps->dev, "reg 0x%x read failed\n", reg);
		return ret;
	}
	ret = val >> offset & 0x01;

	return ret;
}

static void tps68470_gpio_set(struct gpio_chip *gc, unsigned offset,
			      int value)
{
	struct tps68470_gpio_data *tps68470_gpio = to_gpio_data(gc);
	struct tps68470 *tps = tps68470_gpio->tps68470;
	unsigned int reg = TPS68470_REG_GPDO;

	if (offset >= TPS68470_N_GPIO)
		return;

	if (offset >= TPS68470_N_REGULAR_GPIO &&
	    offset < TPS68470_N_LOGIC_OUTPUT + TPS68470_N_REGULAR_GPIO) {
		reg = TPS68470_REG_SGPO;
		offset -= TPS68470_N_REGULAR_GPIO;
	} else if (offset >=
		   TPS68470_N_LOGIC_OUTPUT + TPS68470_N_REGULAR_GPIO) {
		/* enable i2c daisy-chaining */
		offset = 0;
		reg = TPS68470_REG_S_I2C_CTL;
	}

	if (value)
		tps68470_set_bits(tps, reg, value << offset, value << offset);
	else
		tps68470_clear_bits(tps, reg, 1 << offset);
}

static int tps68470_gpio_output(struct gpio_chip *gc, unsigned offset,
				int value)
{
	struct tps68470_gpio_data *tps68470_gpio = to_gpio_data(gc);
	struct tps68470 *tps = tps68470_gpio->tps68470;

	if (offset > TPS68470_N_GPIO)
		return -EINVAL;

	/* rest are always outputs */
	if (offset >= TPS68470_N_REGULAR_GPIO)
		return 0;

	/* Set the initial value */
	tps68470_gpio_set(gc, offset, value);

	return tps68470_set_bits(tps, TPS68470_GPIO_CTL_REG_A(offset),
				 TPS68470_GPIO_MODE_MASK,
				 TPS68470_GPIO_MODE_OUT_CMOS);
}

static int tps68470_gpio_input(struct gpio_chip *gc, unsigned offset)
{
	struct tps68470_gpio_data *tps68470_gpio = to_gpio_data(gc);
	struct tps68470 *tps = tps68470_gpio->tps68470;

	/* rest are always outputs */
	if (offset >= TPS68470_N_REGULAR_GPIO)
		return -EINVAL;

	return tps68470_clear_bits(tps, TPS68470_GPIO_CTL_REG_A(offset),
				   TPS68470_GPIO_MODE_MASK);
}

static int tps68470_gpio_request(struct gpio_chip *gc, unsigned offset)
{
	struct tps68470_gpio_data *tps68470_gpio = to_gpio_data(gc);
	struct tps68470 *tps = tps68470_gpio->tps68470;

	dev_dbg(tps->dev, "%s offset:%d\n", __func__, offset);
	return 0;
}

static void tps68470_gpio_free(struct gpio_chip *gc, unsigned offset)
{
	struct tps68470_gpio_data *tps68470_gpio = to_gpio_data(gc);
	struct tps68470 *tps = tps68470_gpio->tps68470;

	dev_dbg(tps->dev, "%s offset:%d\n", __func__, offset);
}

struct gpiod_lookup_table gpios_table = {
	.dev_id = NULL,
	.table = {
		GPIO_LOOKUP("tps68470-gpio", 0, "gpio.0", GPIO_ACTIVE_HIGH),
		GPIO_LOOKUP("tps68470-gpio", 1, "gpio.1", GPIO_ACTIVE_HIGH),
		GPIO_LOOKUP("tps68470-gpio", 2, "gpio.2", GPIO_ACTIVE_HIGH),
		GPIO_LOOKUP("tps68470-gpio", 3, "gpio.3", GPIO_ACTIVE_HIGH),
		GPIO_LOOKUP("tps68470-gpio", 4, "gpio.4", GPIO_ACTIVE_HIGH),
		GPIO_LOOKUP("tps68470-gpio", 5, "gpio.5", GPIO_ACTIVE_HIGH),
		GPIO_LOOKUP("tps68470-gpio", 6, "gpio.6", GPIO_ACTIVE_HIGH),
		GPIO_LOOKUP("tps68470-gpio", 7, "s_enable", GPIO_ACTIVE_HIGH),
		GPIO_LOOKUP("tps68470-gpio", 8, "s_idle", GPIO_ACTIVE_HIGH),
		GPIO_LOOKUP("tps68470-gpio", 9, "s_resetn", GPIO_ACTIVE_HIGH),
		/* FIXME: Hiding daisy-chain enable behing gpio for now */
		GPIO_LOOKUP("tps68470-gpio", 10, "daisy-chain",
			    GPIO_ACTIVE_HIGH),
		{ },
	},
};

static struct gpio_chip tps68470_gpio_chip = {
	.label			= "tps68470-gpio",
	.owner			= THIS_MODULE,
	.direction_input	= tps68470_gpio_input,
	.direction_output	= tps68470_gpio_output,
	.get			= tps68470_gpio_get,
	.set			= tps68470_gpio_set,
	.request		= tps68470_gpio_request,
	.free			= tps68470_gpio_free,
	.can_sleep		= true,
	.ngpio			= TPS68470_N_GPIO,
	.base			= -1,
};

static int tps68470_gpio_probe(struct platform_device *pdev)
{
	struct tps68470 *tps68470 = dev_get_drvdata(pdev->dev.parent);
	struct tps68470_gpio_data *tps68470_gpio;
	int i, ret;

	tps68470_gpio = devm_kzalloc(&pdev->dev, sizeof(*tps68470_gpio),
				     GFP_KERNEL);
	if (tps68470_gpio == NULL)
		return -ENOMEM;

	tps68470_gpio->tps68470 = tps68470;
	tps68470_gpio->gc = tps68470_gpio_chip;
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 1, 0)
	tps68470_gpio->gc.dev = &pdev->dev;
#else
	tps68470_gpio->gc.parent = &pdev->dev;
#endif
	ret = gpiochip_add(&tps68470_gpio->gc);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register gpio_chip: %d\n", ret);
		return ret;
	}

	gpiod_add_lookup_table(&gpios_table);

	platform_set_drvdata(pdev, tps68470_gpio);

	for (i = 0; i < tps68470_gpio->gc.ngpio; i++)
		tps68470_gpio_set(&tps68470_gpio->gc, i, 0);

	dev_info(tps68470->dev, "Registered %s GPIO\n", pdev->name);

	return ret;
}

static int tps68470_gpio_remove(struct platform_device *pdev)
{
	struct tps68470_gpio_data *tps68470_gpio = platform_get_drvdata(pdev);
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 1, 0)
	gpiod_remove_lookup_table(&gpios_table);
#endif
	gpiochip_remove(&tps68470_gpio->gc);

	return 0;
}

static struct platform_driver tps68470_gpio_driver = {
	.driver = {
		.name = "tps68470-gpio",
	},
	.probe = tps68470_gpio_probe,
	.remove = tps68470_gpio_remove,
};

static int __init tps68470_gpio_init(void)
{
	return platform_driver_register(&tps68470_gpio_driver);
}
subsys_initcall(tps68470_gpio_init);

static void __exit tps68470_gpio_exit(void)
{
	platform_driver_unregister(&tps68470_gpio_driver);
}
module_exit(tps68470_gpio_exit);

MODULE_AUTHOR("Zaikuo Wang <zaikuo.wang@intel.com>");
MODULE_AUTHOR("Tianshu Qiu <tian.shu.qiu@intel.com>");
MODULE_AUTHOR("Jian Xu Zheng <jian.xu.zheng@intel.com>");
MODULE_AUTHOR("Yuning Pu <yuning.pu@intel.com>");
MODULE_AUTHOR("Antti Laakso <antti.laakso@intel.com>");
MODULE_DESCRIPTION("GPIO driver for TPS68470 PMIC");
MODULE_ALIAS("platform:tps68470-gpio");
MODULE_LICENSE("GPL");
