// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2016--2022 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <linux/acpi.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/gpio-regulator.h>
#include <linux/regulator/machine.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/clkdev.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <media/ipu-isys.h>
#include "ipu.h"
#include <media/ipu-acpi-pdata.h>
#include <media/ipu-acpi.h>
#include <media/ar0234.h>
#include <media/lt6911uxc.h>
#include <media/imx390.h>
#include <media/ti960.h>
#include <media/d4xx.h>

static LIST_HEAD(devices);

static struct ipu_camera_module_data *add_device_to_list(
	struct list_head *devices)
{
	struct ipu_camera_module_data *cam_device;

	cam_device = kzalloc(sizeof(*cam_device), GFP_KERNEL);
	if (!cam_device)
		return NULL;

	list_add(&cam_device->list, devices);
	return cam_device;
}

static const struct ipu_acpi_devices supported_devices[] = {
/*
 *	{ "ACPI ID", sensor_name, get_sensor_pdata, NULL, 0, TYPE, serdes_name },	// Custom HID
 */
	{ "INTC10C0", AR0234_NAME, get_sensor_pdata, NULL, 0, TYPE_DIRECT, NULL },	// AR0234 HID
	{ "INTC10B1", LT6911UXC_NAME, get_sensor_pdata, NULL, 0, TYPE_DIRECT, NULL },	// Lontium HID
	{ "INTC10C1", IMX390_NAME, get_sensor_pdata, NULL, 0, TYPE_SERDES, TI960_NAME },// IMX390 HID
	{ "INTC10CD", D457_NAME, get_sensor_pdata, NULL, 0, TYPE_DIRECT, NULL },	// D457 HID
	/* for later usage */
//	{ "INTC10CD", D457_NAME, get_sensor_pdata, NULL, 0, TYPE_SERDES, MAX9296_NAME },// D457 HID
};

static int get_table_index(struct device *device, const __u8 *acpi_name)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_devices); i++) {
		if (!strcmp(acpi_name, supported_devices[i].hid_name))
			return i;
	}

	return -ENODEV;
}

/* List of ACPI devices what we can handle */
/* Must match with HID in BIOS option. Add new sensor if required */
static const struct acpi_device_id ipu_acpi_match[] = {
/*
 *	{ "AR0234A", 0 },	// Custom HID
 */
	{ "INTC10C0", 0 },	// AR0234 HID
	{ "INTC10B1", 0 },	// Lontium HID
	{ "INTC10C1", 0 },	// IMX390 HID
	{ "INTC10CD", 0 },	// D457 HID
	{},
};
static int ipu_acpi_get_pdata(struct i2c_client *client,
				 const struct acpi_device_id *acpi_id,
				 struct ipu_i2c_helper *helper)
{
	struct ipu_camera_module_data *camdata;
	int index = get_table_index(&client->dev, acpi_id->id);

	if (index < 0) {
		dev_err(&client->dev,
			"Device is not in supported devices list\n");
		return -ENODEV;
	}

	camdata = add_device_to_list(&devices);
	if (!camdata)
		return -ENOMEM;

	strlcpy(client->name, supported_devices[index].real_driver,
		sizeof(client->name));

	dev_info(&client->dev, "Getting BIOS data for %s", client->name);

	supported_devices[index].get_platform_data(
		client, camdata, helper,
		supported_devices[index].priv_data,
		supported_devices[index].priv_size,
		supported_devices[index].connect,
		supported_devices[index].serdes_name);

	return 0;
}

static int ipu_i2c_test(struct device *dev, void *priv)
{
	struct i2c_client *client = i2c_verify_client(dev);
	const struct acpi_device_id *acpi_id;

	/*
	 * Check that we are handling only I2C devices which really has
	 * ACPI data and are one of the devices which we want to handle
	 */

	if (!ACPI_COMPANION(dev) || !client)
		return 0;

	acpi_id = acpi_match_device(ipu_acpi_match, dev);
	if (!acpi_id) {
		dev_err(dev, "acpi id not found, return 0");
		return 0;
	}

	/*
	 * Skip if platform data has already been added.
	 * Probably ACPI data overruled by kernel platform data
	 */
	if (client->dev.platform_data)
		return 0;

	/* Looks that we got what we are looking for */
	if (ipu_acpi_get_pdata(client, acpi_id, priv))
		dev_err(dev, "Failed to process ACPI data");

	/* Don't return error since we want to process remaining devices */

	/* Unregister matching client */
	i2c_unregister_device(client);

	return 0;
}

/* Scan all i2c devices and pick ones which we can handle */

/* Try to get all IPU related devices mentioned in BIOS and all related information
 * If there is existing ipu_isys_subdev_pdata, update the existing pdata
 * If not, return a new generated existing pdata
 */

int ipu_get_acpi_devices(void *driver_data,
				struct device *dev,
				struct ipu_isys_subdev_pdata **spdata,
				struct ipu_isys_subdev_pdata **built_in_pdata,
				int (*fn)
				(struct device *, void *,
				 struct ipu_isys_csi2_config *csi2,
				 bool reprobe))
{
	struct ipu_i2c_helper helper = {
		.fn = fn,
		.driver_data = driver_data,
	};
	int rval;

	if (!built_in_pdata)
		dev_dbg(dev, "Built-in pdata not found");
	else {
		dev_dbg(dev, "Built-in pdata found");
		ptr_built_in_pdata = *built_in_pdata;
	}

	if ((!fn) || (!driver_data))
		return -ENODEV;

	rval = i2c_for_each_dev(&helper, ipu_i2c_test);
	if (rval < 0)
		return rval;

	if (!built_in_pdata) {
		dev_dbg(dev, "Return ACPI generated pdata");
		*spdata = get_acpi_subdev_pdata();
	} else
		dev_dbg(dev, "Return updated built-in pdata");

	return 0;
}
EXPORT_SYMBOL(ipu_get_acpi_devices);

static int __init ipu_acpi_init(void)
{
	ptr_built_in_pdata = NULL;
	return 0;
}

static void __exit ipu_acpi_exit(void)
{
}

module_init(ipu_acpi_init);
module_exit(ipu_acpi_exit);

MODULE_AUTHOR("Samu Onkalo <samu.onkalo@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("IPU6 ACPI support");

