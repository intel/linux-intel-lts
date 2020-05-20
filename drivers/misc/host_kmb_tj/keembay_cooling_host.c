// SPDX-License-Identifier: GPL-2.0-only
/*


 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/acpi.h>
#include <linux/thermal.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include "thermal_hwmon.h"
#include "thermal_core.h"



struct keembay_cooling_data {
	struct thermal_cooling_device *cooling_dev;
};

static int
keembay_cooling_get_max_state(struct thermal_cooling_device *cooling_dev,
			      unsigned long *state)
{
	*state = 4;
	return 0;
}

static int
keembay_cooling_set_cur_state(struct thermal_cooling_device *cooling_dev,
			      unsigned long state)
{
state = 0;
printk(KERN_WARNING "keembay_cooling_set_cur_state\n");
return 0;
				  }

static int
keembay_cooling_get_cur_state(struct thermal_cooling_device *cooling_dev,
			      unsigned long *state)
{
	//printk(KERN_WARNING "keembay_cooling_get_cur_state\n");
 *state = 0;
 return 0;
}

static const struct thermal_cooling_device_ops keembay_cooling_ops = {
	.get_max_state = keembay_cooling_get_max_state,
	.get_cur_state = keembay_cooling_get_cur_state,
	.set_cur_state = keembay_cooling_set_cur_state,

};

static int keembay_cooling_probe(struct platform_device *pdev)
{
	struct keembay_cooling_data *d;
	int ret;
		printk(KERN_WARNING "keembay_thermal_cooling_probe_start\n");
		d = devm_kzalloc(&pdev->dev, sizeof(*d), GFP_KERNEL);
	if (!d) {
			printk(KERN_WARNING "keembay_thermal_cooling_dev_kzalloc_failed\n");
			return -ENOMEM;
		}
		printk(KERN_WARNING "keembay_thermal_cooling_kzalloc\n");
	d->cooling_dev = thermal_cooling_device_register("keembay_thermal",
							 d, &keembay_cooling_ops);
	if (IS_ERR(d->cooling_dev)) {
		ret = PTR_ERR(d->cooling_dev);
		dev_err(&pdev->dev,
			"failed to register thermal zone device %d\n", ret);
			printk(KERN_WARNING "keembay_thermal_cooling_register_failed\n");
		}
		printk(KERN_WARNING "keembay_thermal_cooling_register\n");

	return 0;
}

static int keembay_cooling_remove(struct platform_device *pdev)
{
	struct keembay_cooling_data *d = platform_get_drvdata(pdev);

	thermal_cooling_device_unregister(d->cooling_dev);
	return 0;
}

static const struct of_device_id keembay_cooling_id_table[] = {
	{ .compatible = "intel,keembay-dummy" },
	{}
};

MODULE_DEVICE_TABLE(of, keembay_cooling_id_table);

static struct platform_driver keembay_cooling_driver = {
	.probe = keembay_cooling_probe,
	.remove = keembay_cooling_remove,
	.driver = {
		.name = "keembay_cooling",
		.of_match_table = keembay_cooling_id_table,
	},
};

module_platform_driver(keembay_cooling_driver);

MODULE_AUTHOR("Sandeep Singh <sandeep1.singh@intel.com>");
MODULE_DESCRIPTION("keembay thermal driver");
MODULE_LICENSE("GPL");


