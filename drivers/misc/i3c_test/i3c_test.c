// SPDX-License-Identifier: GPL-2.0
/*
 * Intel Keem Bay I3C Test Driver for Introspect.
 * ID mapping done based on the value of dcr.
 *
 * Copyright (C) 2020 Intel Corporation
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i3c/device.h>
#include <linux/i3c/master.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/regmap.h>

static const struct i3c_device_id i3c_test_ids[] = {
	I3C_CLASS(0x0, (void *)ST_LSM6DSO_ID),
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(i3c, i3c_test_ids);

static int i3c_test_write(struct i3c_device *i3cdev, const void *data,
			  size_t count)
{
	int ret;
	struct i3c_priv_xfer xfers[] = {
		{
			.rnw = false,
			.len = count,
			.data.out = data,
		},
	};

	ret = i3c_device_do_priv_xfers(i3cdev, xfers, 1);
	return ret;
}

static int i3c_test_read(struct i3c_device *i3cdev, void *reg,
			 size_t reg_size, void *val, size_t val_size)
{
	char rbuff[4];

	struct i3c_priv_xfer xfers[] = {
		{
			.rnw = true,
			.len = val_size,
			.data.in = (void *)rbuff,
		},
	};

	int ret = 0;

	ret = i3c_device_do_priv_xfers(i3cdev, xfers, 1);
	dev_info(&i3cdev->dev, "read done: %d, %d, %d, %d\n", rbuff[0],
		 rbuff[1], rbuff[2], rbuff[3]);
	return ret;
}

static int i3c_test_probe(struct i3c_device *i3cdev)
{
	struct i3c_device_info info;
	int wbuff = 0x11223344;
	int speed;

	i3c_device_get_info(i3cdev, &info);
	dev_info(&i3cdev->dev, "i3c_test: %s, bcr: %x\n", __func__, info.bcr);
	dev_info(&i3cdev->dev, "i3c_test: %s, dcr: %x\n", __func__, info.dcr);
	dev_info(&i3cdev->dev, "i3c_test: %s, pid: %llx\n", __func__, info.pid);
	dev_info(&i3cdev->dev, "i3c_test: %s, static address: %x\n",
		 __func__, info.static_addr);
	dev_info(&i3cdev->dev, "i3c_test: %s, dyn_addr: %x\n",
		 __func__, info.dyn_addr);

	dev_info(&i3cdev->dev, "i3c_test probe done\n");
	for (speed = 0; speed < 5; speed++) {
		dev_info(&i3cdev->dev, "i3c: SDR%d Private Transfer\n ", speed);
		i3cdev->desc->info.max_write_ds = speed;
		i3cdev->desc->info.max_read_ds = speed;
		i3c_test_write(i3cdev, &wbuff, sizeof(wbuff));
		i3c_test_read(i3cdev, NULL, 2, NULL, 2);
	}

	return 0;
}

static struct i3c_driver i3c_test_driver = {
	.driver = {
		.name = "i3c_test",
	},
	.probe = i3c_test_probe,
	.id_table = i3c_test_ids,
};
module_i3c_driver(i3c_test_driver);

MODULE_AUTHOR("D, Lakshmi Sowjanya <lakshmi.sowjanya.d@intel.com>");
MODULE_DESCRIPTION("KMB I3C Test Driver");
MODULE_LICENSE("GPL v2");
