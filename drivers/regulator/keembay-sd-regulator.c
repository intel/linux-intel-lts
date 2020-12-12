// SPDX-License-Identifier: GPL-2.0
/*
 * Intel Keem Bay SD Regulator
 *
 * Copyright (C) 2020, Intel Corporation
 * Author: Muhammad Husaini Zulkifli <Muhammad.Husaini.Zulkifli@intel.com>
 */

#include <linux/err.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#include <linux/firmware/intel/keembay.h>

static int keembay_regulator_set_voltage(struct regulator_dev *dev,
					int min_uV, int max_uV,
					unsigned *selector)
{
	int tmp_volt;

	if (min_uV == KEEMBAY_IOV_1_8V_uV && max_uV == KEEMBAY_IOV_1_8V_uV)
		tmp_volt = KEEMBAY_SET_1V8_IO_RAIL;
	else
		tmp_volt = KEEMBAY_SET_3V3_IO_RAIL;

	return keembay_set_io_rail_supplied_voltage(tmp_volt);
}

static int keembay_regulator_get_voltage(struct regulator_dev *dev)
{
	int ret;

	ret = keembay_get_io_rail_supplied_voltage();

	return ret ? KEEMBAY_IOV_1_8V_uV : KEEMBAY_IOV_3_3V_uV;
}

static const struct regulator_ops keembay_regulator_voltage_ops = {
	.get_voltage = keembay_regulator_get_voltage,
	.set_voltage = keembay_regulator_set_voltage,
};

static int keembay_regulator_probe(struct platform_device *pdev)
{
	struct regulator_desc *desc;
	struct regulator_init_data *init_data;
	struct regulator_config config = { };
	struct regulator_dev *rdev;
	struct device *dev = &pdev->dev;

	desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;

	init_data = of_get_regulator_init_data(dev, dev->of_node, desc);
	if (!init_data)
		return -EINVAL;

	desc->name = dev_name(dev);
	desc->type = REGULATOR_VOLTAGE;
	desc->owner = THIS_MODULE;
	desc->ops = &keembay_regulator_voltage_ops;

	config.dev = dev;
	config.init_data = init_data;
	config.of_node = dev->of_node;

	rdev = devm_regulator_register(dev, desc, &config);
	if (IS_ERR(rdev))
		return dev_err_probe(dev, PTR_ERR(rdev),
				     "Failed to register Keem Bay SD regulator.\n");

	return 0;
}

static const struct of_device_id regulator_keembay_of_match[] = {
	{ .compatible = "regulator-keembay-sd" },
	{}
};
MODULE_DEVICE_TABLE(of, regulator_keembay_of_match);

static struct platform_driver keembay_regulator_driver = {
	.probe		= keembay_regulator_probe,
	.driver		= {
		.name		= "keembay-sd-regulator",
		.of_match_table = regulator_keembay_of_match,
	},
};

static int __init keembay_regulator_init(void)
{
	return platform_driver_register(&keembay_regulator_driver);
}

/*
 * Using subsys_initcall to ensure that Keem Bay regulator platform driver
 * is initialized before device driver try to utilize it.
 */
subsys_initcall(keembay_regulator_init);

static void __exit keembay_regulator_exit(void)
{
	platform_driver_unregister(&keembay_regulator_driver);
}
module_exit(keembay_regulator_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Intel Keem Bay SD Regulator");
MODULE_AUTHOR("Muhammad Husaini Zulkifli <Muhammad.Husaini.Zulkifli@intel.com>");
