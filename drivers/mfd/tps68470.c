/*
 * TPS68470 chip family multi-function driver
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
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/acpi.h>

#include <linux/mfd/core.h>
#include <linux/mfd/tps68470.h>

static const struct mfd_cell tps68470s[] = {
	{
		.name = "tps68470-gpio",
	},
	{
		.name = "tps68470-clk",
	},
	{
		.name = "tps68470-regulator",
	},
};

/**
 * tps68470_reg_read: Read a single tps68470 register.
 *
 * @tps: Device to read from.
 * @reg: Register to read.
 * @val: Contains the value
 */
int tps68470_reg_read(struct tps68470 *tps, unsigned int reg,
			unsigned int *val)
{
	return regmap_read(tps->regmap, reg, val);
}
EXPORT_SYMBOL_GPL(tps68470_reg_read);

/**
 * tps68470_reg_write: Write a single tps68470 register.
 *
 * @tps68470: Device to write to.
 * @reg: Register to write to.
 * @val: Value to write.
 */
int tps68470_reg_write(struct tps68470 *tps, unsigned int reg,
			unsigned int val)
{
	return regmap_write(tps->regmap, reg, val);
}
EXPORT_SYMBOL_GPL(tps68470_reg_write);

/**
 * tps68470_update_bits: Modify bits w.r.t mask and val.
 *
 * @tps68470: Device to write to.
 * @reg: Register to read-write to.
 * @mask: Mask.
 * @val: Value to write.
 */
static int tps68470_update_bits(struct tps68470 *tps, unsigned int reg,
		unsigned int mask, unsigned int val)
{
	int ret;
	unsigned int data;

	dev_err(tps->dev, "Write for reg 0x%x 0x%x, 0x%x\n", reg, mask, val);

	ret = tps68470_reg_read(tps, reg, &data);
	if (ret) {
		dev_err(tps->dev, "Read from reg 0x%x failed\n", reg);
		return ret;
	}

	data &= ~mask;
	data |= val & mask;

	ret = tps68470_reg_write(tps, reg, data);
	if (ret)
		dev_err(tps->dev, "Write for reg 0x%x failed\n", reg);

	return ret;
}

int tps68470_set_bits(struct tps68470 *tps, unsigned int reg,
		unsigned int mask, unsigned int val)
{
	return tps68470_update_bits(tps, reg, mask, val);
}
EXPORT_SYMBOL_GPL(tps68470_set_bits);

int tps68470_clear_bits(struct tps68470 *tps, unsigned int reg,
		unsigned int mask)
{
	return tps68470_update_bits(tps, reg, mask, 0);
}
EXPORT_SYMBOL_GPL(tps68470_clear_bits);

static const struct regmap_config tps68470_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = TPS68470_REG_MAX,
};

static int tps68470_probe(struct i2c_client *client,
			  const struct i2c_device_id *ids)
{
	struct tps68470 *tps;
	unsigned int version;
	int ret;

	tps = devm_kzalloc(&client->dev, sizeof(*tps), GFP_KERNEL);
	if (!tps)
		return -ENOMEM;

	i2c_set_clientdata(client, tps);
	tps->dev = &client->dev;

	tps->regmap = devm_regmap_init_i2c(client, &tps68470_regmap_config);
	if (IS_ERR(tps->regmap)) {
		ret = PTR_ERR(tps->regmap);
		dev_err(tps->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	ret = tps68470_reg_read(tps, TPS68470_REG_REVID, &version);
	if (ret < 0) {
		dev_err(tps->dev,
			"Failed to read revision register: %d\n", ret);
		goto fail;
	}

	ret = mfd_add_devices(tps->dev, -1, tps68470s,
			      ARRAY_SIZE(tps68470s), NULL, 0, NULL);
	if (ret < 0) {
		dev_err(tps->dev, "mfd_add_devices failed: %d\n", ret);
		return ret;
	}

	dev_info(tps->dev, "TPS68470 Vendor code: %#x Major rev: %d Minor rev: %d\n",
			(version & TPS68470_REVID_VENDOR_MASK) >> 5,
			(version & TPS68470_REVID_MAJOR_REV_MASK) >> 3,
			version & TPS68470_REVID_MINOR_REV_MASK);

	tps68470_reg_write(tps, TPS68470_REG_RESET, 0xff);
	/* FIXME: configure these dynamically */
	tps68470_reg_write(tps, TPS68470_REG_S_I2C_CTL, 2);
	tps68470_reg_write(tps, TPS68470_REG_GPCTL5A, 2);
	tps68470_reg_write(tps, TPS68470_REG_GPCTL6A, 2);
	/*
	 * When SDA and SCL are routed to GPIO1 and GPIO2,
	 * the mode for these GPIOs
	 * must be configured using their respective GPCTLxA
	 * registers as inputs with
	 * no pull-ups.
	 */
	tps68470_reg_write(tps, TPS68470_REG_GPCTL1A, 0);
	tps68470_reg_write(tps, TPS68470_REG_GPCTL2A, 0);

	return 0;
fail:
	mfd_remove_devices(tps->dev);

	return ret;
}

static int tps68470_remove(struct i2c_client *client)
{
	struct tps68470 *tps = i2c_get_clientdata(client);

	mfd_remove_devices(tps->dev);

	return 0;
}

static const struct i2c_device_id tps68470_id_table[] = {
	{ TPS68470_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, tps68470_id_table);

static const struct acpi_device_id tps68470_acpi_ids[] = {
	{ "INT3472" },
	{ },
};

MODULE_DEVICE_TABLE(acpi, tps68470_acpi_ids);

static struct i2c_driver tps68470_driver = {
	.driver		= {
		.name	= TPS68470_NAME,
		.owner   = THIS_MODULE,
		.acpi_match_table = ACPI_PTR(tps68470_acpi_ids),
	},
	.id_table	= tps68470_id_table,
	.probe		= tps68470_probe,
	.remove		= tps68470_remove,
};

static int __init tps68470_init(void)
{
	return i2c_add_driver(&tps68470_driver);
}
subsys_initcall(tps68470_init);

static void __exit tps68470_exit(void)
{
	i2c_del_driver(&tps68470_driver);
}
module_exit(tps68470_exit);

MODULE_AUTHOR("Zaikuo Wang <zaikuo.wang@intel.com>");
MODULE_AUTHOR("Tianshu Qiu <tian.shu.qiu@intel.com>");
MODULE_AUTHOR("Jian Xu Zheng <jian.xu.zheng@intel.com>");
MODULE_AUTHOR("Yuning Pu <yuning.pu@intel.com>");
MODULE_AUTHOR("Rajmohan Mani <rajmohan.mani@intel.com>");
MODULE_DESCRIPTION("TPS68470 chip family multi-function driver");
MODULE_LICENSE("GPL v2");
