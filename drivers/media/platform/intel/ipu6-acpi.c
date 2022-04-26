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

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <media/ipu-isys.h>
#include "ipu.h"

#include <media/ipu-acpi.h>
#include <media/ar0234.h>
#include <media/lt6911uxc.h>

#define HID_BUFFER_SIZE 32
#define VCM_BUFFER_SIZE 32

#define LOOP_SIZE 10
#define SUFFIX_BASE 96
static LIST_HEAD(devices);
static LIST_HEAD(new_devs);

struct ipu_i2c_helper {
	int (*fn)(struct device *dev, void *priv,
		  struct ipu_isys_csi2_config *csi2,
		  bool reprobe);
	void *driver_data;
};

struct ipu_i2c_new_dev {
	struct list_head list;
	struct i2c_board_info info;
	unsigned short int bus;
};

struct ipu_camera_module_data {
	struct list_head list;
	struct ipu_isys_subdev_info sd;
	struct ipu_isys_csi2_config csi2;
	unsigned int ext_clk;
	void *pdata; /* Ptr to generated platform data*/
	void *priv; /* Private for specific subdevice */
};

struct ipu_acpi_devices {
	const char *hid_name;
	const char *real_driver;
	int (*get_platform_data)(struct i2c_client *client,
				 struct ipu_camera_module_data *data,
				 struct ipu_i2c_helper *helper,
				 void *priv, size_t size);
	void *priv_data;
	size_t priv_size;
//	const struct intel_ipu_regulator *regulators;
};

/* acpi_subdev_pdata as pdata */

static struct ipu_isys_clk_mapping clk_mapping[] = {
	{ CLKDEV_INIT(NULL, NULL, NULL), NULL }
};

struct ipu_isys_subdev_pdata acpi_subdev_pdata = {
	.subdevs = (struct ipu_isys_subdev_info *[]) {
		NULL,
	},
	.clk_map = clk_mapping,
};

struct ipu_isys_subdev_pdata *ptr_built_in_pdata;

/*
 * Add a request to create new i2c devices later on. i2c_new_device can't be
 * directly called from functions which are called by i2c_for_each_dev
 * function. Both takes a same mutex inside i2c core code.
 */
static int add_new_i2c(unsigned short addr, unsigned short  bus,
		       unsigned short flags, char *name, void *pdata)
{
	struct ipu_i2c_new_dev *newdev = kzalloc(sizeof(*newdev), GFP_KERNEL);

	if (!newdev)
		return -ENOMEM;

	newdev->info.flags = flags;
	newdev->info.addr = addr;
	newdev->bus = bus;
	newdev->info.platform_data = pdata;
	strlcpy(newdev->info.type, name, sizeof(newdev->info.type));

	list_add(&newdev->list, &new_devs);
	return 0;
}

static int get_string_dsdt_data(struct device *dev, const u8 *dsdt,
				int func, char *out, unsigned int size)
{
	struct acpi_handle *dev_handle = ACPI_HANDLE(dev);
	union acpi_object *obj;
	int ret = -ENODEV;

	obj = acpi_evaluate_dsm(dev_handle, (void *)dsdt, 0, func, NULL);
	if (!obj) {
		dev_err(dev, "No dsdt field\n");
		return -ENODEV;
	}
	dev_dbg(dev, "ACPI type %d", obj->type);

	if ((obj->type != ACPI_TYPE_STRING) || !obj->string.pointer)
		goto exit;

	strlcpy(out, obj->string.pointer,
		min((unsigned int)(obj->string.length + 1), size));
	dev_info(dev, "DSDT string id: %s\n", out);

	ret = 0;
exit:
	ACPI_FREE(obj);
	return ret;
}

static int get_integer_dsdt_data(struct device *dev, const u8 *dsdt,
				 int func, u64 *out)
{
	struct acpi_handle *dev_handle = ACPI_HANDLE(dev);
	union acpi_object *obj;

	obj = acpi_evaluate_dsm(dev_handle, (void *)dsdt, 0, func, NULL);
	if (!obj) {
		dev_err(dev, "No dsdt\n");
		return -ENODEV;
	}
	dev_dbg(dev, "ACPI type %d", obj->type);

	if (obj->type != ACPI_TYPE_INTEGER) {
		ACPI_FREE(obj);
		return -ENODEV;
	}
	*out = obj->integer.value;
	ACPI_FREE(obj);
	return 0;
}

static int read_acpi_block(struct device *dev, char *id, void *data, u32 size)
{
	union acpi_object *obj;
	struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
	struct acpi_handle *dev_handle = ACPI_HANDLE(dev);
	int status;
	u32 buffer_length;

	status = acpi_evaluate_object(dev_handle, id, NULL, &buffer);
	if (!ACPI_SUCCESS(status))
		return -ENODEV;

	obj = (union acpi_object *)buffer.pointer;
	if (!obj || obj->type != ACPI_TYPE_BUFFER) {
		dev_err(dev, "Could't read acpi buffer\n");
		status = -ENODEV;
		goto err;
	}

	if (obj->buffer.length > size) {
		dev_err(dev, "Given buffer is too small\n");
		status = -ENODEV;
		goto err;
	}

	memcpy(data, obj->buffer.pointer, min(size, obj->buffer.length));
	buffer_length = obj->buffer.length;
	kfree(buffer.pointer);

	return buffer_length;
err:
	kfree(buffer.pointer);
	return status;
}

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

static int ipu_acpi_get_gpio_data(struct device *dev, struct ipu_gpio_info *gpio, int size,
				u64 *gpio_num)
{
	const u8 dsdt_cam_gpio[] = {
		0x40, 0x46, 0x23, 0x79, 0x10, 0x9e, 0xea, 0x4f,
		0xa5, 0xc1, 0xB5, 0xaa, 0x8b, 0x19, 0x75, 0x6f };

	int i = 0, j = 0, retries = 0, loop = 0;
	u64 num_gpio;

	int rval = get_integer_dsdt_data(dev, dsdt_cam_gpio, 1, &num_gpio);

	if (rval < 0) {
		dev_err(dev, "Failed to get number of GPIO pins\n");
		return rval;
	}

	dev_dbg(dev, "Num of gpio found = %lld", num_gpio);

	if (num_gpio == 0) {
		*gpio_num = num_gpio;
		return rval;
	}

	if (num_gpio > size) {
		dev_err(dev, "Incorrect number of GPIO pins\n");
		return rval;
	}

	/* repeat until all gpio pin is saved */
	while (i < num_gpio && loop <= LOOP_SIZE) {
		u64 data;
		struct gpio_desc *desc = NULL;

		rval = get_integer_dsdt_data(dev, dsdt_cam_gpio, i + 2, &data);

		if (rval < 0) {
			dev_err(dev, "No gpio data\n");
			return -ENODEV;
		}

		gpio[i].func = (data & 0xff);
		gpio[i].valid = FALSE;

		desc = gpiod_get_index(dev, NULL, i + retries, GPIOD_ASIS);

		if (!IS_ERR(desc)) {
			unsigned short pin = desc_to_gpio(desc);
			bool save = TRUE;

			/* always save first GPIO pin */
			if (i == 0)
				save = TRUE;

			/* check subsequent GPIO pin for replicate */
			else {
				for (j = 0; j <= i; j++) {
					/* retry if same as previous pin */
					if (gpio[j].pin == pin) {
						retries++;
						save = FALSE;
						gpiod_put(desc);
						break;
					}
				}
			}

			/* save into array */
			if (save == TRUE) {
				gpio[i].pin = pin;
				gpio[i].valid = TRUE;
				gpiod_put(desc);
				i++;
				retries = 0;
			}
		}
		loop++;
	}
	*gpio_num = num_gpio;

	return rval;
}

static int ipu_acpi_get_i2c_info(struct device *dev, struct ipu_i2c_info *i2c, int size)
{
	const u8 dsdt_cam_i2c[] = {
		0x49, 0x75, 0x25, 0x26, 0x71, 0x92, 0xA4, 0x4C,
		0xBB, 0x43, 0xC4, 0x89, 0x9D, 0x5A, 0x48, 0x81};

	u64 num_i2c;
	int i;
	int rval = get_integer_dsdt_data(dev, dsdt_cam_i2c, 1, &num_i2c);

	if (rval < 0) {
		dev_err(dev, "Failed to get number of I2C\n");
		return -ENODEV;
	}

	for (i = 0; i < num_i2c && i < size; i++) {
		u64 data;

		rval = get_integer_dsdt_data(dev, dsdt_cam_i2c, i + 2,
					     &data);

		if (rval < 0) {
			dev_err(dev, "Failed to get I2C data\n");
			return -ENODEV;
		}

		i2c[i].bus = ((data >> 24) & 0xff);
		i2c[i].addr = (data >> 8) & 0xff;

		dev_dbg(dev, "ACPI camera option: i2c bus %d addr %x",
			i2c[i].bus, i2c[i].addr);
	}
	return 0;
}

static int match_depend(struct device *dev, const void *data)
{
	return (dev && dev->fwnode == data) ? 1 : 0;
}

#define MAX_CONSUMERS 1
static int ipu_acpi_get_control_logic_data(struct device *dev,
					struct control_logic_data **ctl_data)
{
	/* CLDB data */
	struct control_logic_data_packed ctl_logic_data;
	int ret = read_acpi_block(dev, "CLDB", &ctl_logic_data,
				sizeof(ctl_logic_data));

	if (ret < 0) {
		dev_err(dev, "no such CLDB block");
		return ret;
	}

	(*ctl_data)->type = ctl_logic_data.controllogictype;
	(*ctl_data)->id = ctl_logic_data.controllogicid;
	(*ctl_data)->sku = ctl_logic_data.sensorcardsku;

	dev_dbg(dev, "CLDB data version %d clid %d cltype %d sku %d",
		ctl_logic_data.version,
		ctl_logic_data.controllogictype,
		ctl_logic_data.controllogicid,
		ctl_logic_data.sensorcardsku);

	/* GPIO data */
	ret = ipu_acpi_get_gpio_data(dev, (*ctl_data)->gpio, ARRAY_SIZE((*ctl_data)->gpio),
				&((*ctl_data)->gpio_num));

	if (ret < 0) {
		dev_err(dev, "Failed to get GPIO data");
		return ret;
	}
	return 0;
}

static int ipu_acpi_get_dep_data(struct device *dev,
			     struct control_logic_data *ctl_data)
{
	struct acpi_handle *dev_handle = ACPI_HANDLE(dev);
	struct acpi_handle_list dep_devices;
	acpi_status status;
	int i;
	int rval;

	ctl_data->completed = false;

	if (!acpi_has_method(dev_handle, "_DEP")) {
		dev_err(dev, "ACPI does not have _DEP method");
		return 0;
	}

	status = acpi_evaluate_reference(dev_handle, "_DEP", NULL,
					 &dep_devices);

	if (ACPI_FAILURE(status)) {
		dev_err(dev, "Failed to evaluate _DEP.\n");
		return -ENODEV;
	}

	for (i = 0; i < dep_devices.count; i++) {
		struct acpi_device *device;
		struct acpi_device_info *info;
		struct device *p_dev;
		int match;

		status = acpi_get_object_info(dep_devices.handles[i], &info);
		if (ACPI_FAILURE(status)) {
			dev_err(dev, "Error reading _DEP device info\n");
			continue;
		}

		match = info->valid & ACPI_VALID_HID &&
			!strcmp(info->hardware_id.string, "INT3472");

		kfree(info);

		if (!match)
			continue;

		/* Process device IN3472 created by acpi */
		if (acpi_bus_get_device(dep_devices.handles[i], &device)) {
			dev_err(dev, "INT3472 does not have dep device");
			return -ENODEV;
		}

		dev_dbg(dev, "Depend ACPI device found: %s\n",
			dev_name(&device->dev));

		p_dev = bus_find_device(&platform_bus_type, NULL,
					&device->fwnode, match_depend);

		if (p_dev) {
			dev_err(dev, "Dependent platform device found %s\n",
				dev_name(p_dev));

			/* obtain Control Logic Data from BIOS */
			rval = ipu_acpi_get_control_logic_data(p_dev, &ctl_data);

			if (rval) {
				dev_err(dev, "Error getting Control Logic Data");
				return rval;
			} else
				ctl_data->completed = true;
		} else
			dev_err(dev, "Dependent platform device not found\n");
	}

	if (!ctl_data->completed)
		dev_err(dev, "No control logic data available");

	return 0;
}

int ipu_acpi_get_cam_data(struct device *dev,
			     struct sensor_bios_data *sensor)
{
	/* SSDB */
	struct sensor_bios_data_packed sensor_data;

	int ret = read_acpi_block(dev, "SSDB", &sensor_data,
				  sizeof(sensor_data));

	if (ret < 0) {
		dev_err(dev, "Fail to read from SSDB");
		return ret;
	}

	/* Xshutdown is not part of the ssdb data */
	sensor->link = sensor_data.link;
	sensor->lanes = sensor_data.lanes;
	sensor->pprval = sensor_data.pprval;

	dev_dbg(dev, "sensor ACPI data: name %s link %d, lanes %d ",
		dev_name(dev), sensor->link, sensor->lanes);

	/* I2C */
	ret = ipu_acpi_get_i2c_info(dev, sensor->i2c, ARRAY_SIZE(sensor->i2c));

	if (ret < 0)
		return ret;

	return 0;
}
EXPORT_SYMBOL(ipu_acpi_get_cam_data);

static void add_local_subdevs(struct ipu_isys_subdev_info *new_subdev_info)
{
	struct ipu_isys_subdev_pdata *ptr_acpi_subdev_pdata = &acpi_subdev_pdata;
	int i = 0;

	while (i <= MAX_ACPI_SENSOR_NUM) {
		if (!ptr_acpi_subdev_pdata->subdevs[i]) {
			ptr_acpi_subdev_pdata->subdevs[i] = new_subdev_info;
			ptr_acpi_subdev_pdata->subdevs[i+1] = NULL;
			break;
		}
		i++;
	}
}

static bool update_subdev(struct device dev,
			struct ipu_isys_subdev_info *new_subdev_info,
			struct ipu_isys_subdev_info **sd_info)
{
	struct lt6911uxc_platform_data *ori_pdata =
					(*sd_info)->i2c.board_info.platform_data;

	struct lt6911uxc_platform_data *new_pdata =
					new_subdev_info->i2c.board_info.platform_data;

	/* csi2 */
	if ((*sd_info)->csi2->port != new_subdev_info->csi2->port)
		dev_info(&dev, "CSI2 Port %d -> %d",
			(*sd_info)->csi2->port, new_subdev_info->csi2->port);

	if ((*sd_info)->csi2->nlanes != new_subdev_info->csi2->nlanes)
		dev_info(&dev, "CSI2 nlanes %d -> %d",
			(*sd_info)->csi2->nlanes, new_subdev_info->csi2->nlanes);

	/* i2c */
	if ((*sd_info)->i2c.board_info.addr != new_subdev_info->i2c.board_info.addr)
		dev_info(&dev, "I2C board_info addr %x -> %x",
			(*sd_info)->i2c.board_info.addr, new_subdev_info->i2c.board_info.addr);

	if (strcmp((*sd_info)->i2c.i2c_adapter_bdf, new_subdev_info->i2c.i2c_adapter_bdf) != 0)
		dev_info(&dev, "I2C bdf %s -> %s",
			(*sd_info)->i2c.i2c_adapter_bdf, new_subdev_info->i2c.i2c_adapter_bdf);

	/* platform data */
	if (ori_pdata->port != new_pdata->port)
		dev_info(&dev, "Pdata port %d -> %d",
			ori_pdata->port, new_pdata->port);

	if (ori_pdata->lanes != new_pdata->lanes)
		dev_info(&dev, "Pdata lanes %d -> %d",
			ori_pdata->lanes, new_pdata->lanes);

	if (ori_pdata->i2c_slave_address != new_pdata->i2c_slave_address)
		dev_info(&dev, "Pdata I2C_slave_addr %x -> %x",
			ori_pdata->i2c_slave_address, new_pdata->i2c_slave_address);

	if (ori_pdata->irq_pin != new_pdata->irq_pin)
		dev_info(&dev, "Pdata irq_pin %d -> %d",
			ori_pdata->irq_pin, new_pdata->irq_pin);

	if (strcmp(ori_pdata->irq_pin_name, new_pdata->irq_pin_name) != 0)
		dev_info(&dev, "Pdata irq_pin_name %s -> %s",
			ori_pdata->irq_pin_name, new_pdata->irq_pin_name);

	if (ori_pdata->reset_pin != new_pdata->reset_pin)
		dev_info(&dev, "Pdata reset_pin %d -> %d",
			ori_pdata->reset_pin, new_pdata->reset_pin);

	if (ori_pdata->detect_pin != new_pdata->detect_pin)
		dev_info(&dev, "Pdata detect_pin %d -> %d",
			ori_pdata->detect_pin, new_pdata->detect_pin);

	(*sd_info)->csi2->port = new_subdev_info->csi2->port;
	(*sd_info)->csi2->nlanes = new_subdev_info->csi2->nlanes;
	(*sd_info)->i2c.board_info.addr = new_subdev_info->i2c.board_info.addr;

	strlcpy((*sd_info)->i2c.i2c_adapter_bdf, new_subdev_info->i2c.i2c_adapter_bdf,
		sizeof((*sd_info)->i2c.i2c_adapter_bdf));

	ori_pdata->port = new_pdata->port;
	ori_pdata->lanes = new_pdata->lanes;
	ori_pdata->i2c_slave_address = new_pdata->i2c_slave_address;

	ori_pdata->irq_pin = new_pdata->irq_pin;

	strlcpy(ori_pdata->irq_pin_name, new_pdata->irq_pin_name, sizeof(new_pdata->irq_pin_name));
	ori_pdata->reset_pin = new_pdata->reset_pin;
	ori_pdata->detect_pin = new_pdata->detect_pin;

	return true;
}

int compare_subdev(struct device dev,
			struct ipu_isys_subdev_info *new_subdev,
			struct ipu_isys_subdev_info *existing_subdev)
{
	/* check for ACPI HID in existing pdata */
	if (existing_subdev->acpi_hid) {
		/* compare with HID for User Custom */
		if (!strcmp(existing_subdev->acpi_hid, dev_name(&dev))) {
			dev_info(&dev, "Found matching sensor : %s", dev_name(&dev));
			return 0;
		}
	}

	/* compare sensor type */
	if (!strcmp(existing_subdev->i2c.board_info.type,
			new_subdev->i2c.board_info.type)) {
		struct lt6911uxc_platform_data *existing_pdata, *new_pdata;

		existing_pdata = (struct lt6911uxc_platform_data *)
					existing_subdev->i2c.board_info.platform_data;

		new_pdata = (struct lt6911uxc_platform_data *)
					new_subdev->i2c.board_info.platform_data;

		if (existing_pdata->suffix == new_pdata->suffix) {
			dev_info(&dev, "Found matching sensor : %s %c",
				existing_subdev->i2c.board_info.type,
				existing_pdata->suffix);
			return 0;
		}
	}
	return -1;
}

void update_pdata(struct device dev,
			struct ipu_isys_subdev_info *new_subdev)
{
	struct ipu_isys_subdev_info *acpi_subdev;

	acpi_subdev = new_subdev;

	/* update local ipu_isys_subdev_pdata */
	add_local_subdevs(acpi_subdev);

	/* if there is existing pdata, update the existing one */
	if (ptr_built_in_pdata) {
		struct ipu_isys_subdev_info **subdevs, *sd_info;

		for (subdevs = ptr_built_in_pdata->subdevs; *subdevs; subdevs++) {
			sd_info = *subdevs;

			if (!compare_subdev(dev, acpi_subdev, sd_info))
				update_subdev(dev, acpi_subdev, &sd_info);
		}
	}
}

int ar0234_populate(struct device dev,
			struct ipu_isys_subdev_info **ar0234_sd,
			struct ipu_i2c_info i2c,
			char sensor_name[I2C_NAME_SIZE],
			struct sensor_bios_data cam_data,
			struct control_logic_data ctl_data)
{
	struct ar0234_platform_data *pdata;
	struct ipu_isys_csi2_config *csi2_config;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	csi2_config = kzalloc(sizeof(*csi2_config), GFP_KERNEL);
	if (!csi2_config) {
		kfree(pdata);
		return -ENOMEM;
	}

	/* csi2 */
	csi2_config->nlanes = cam_data.lanes;
	csi2_config->port = cam_data.link;
	(*ar0234_sd)->csi2 = csi2_config;

	/* i2c */
	(*ar0234_sd)->i2c.board_info.addr = i2c.addr;
	strlcpy((*ar0234_sd)->i2c.board_info.type, sensor_name, I2C_NAME_SIZE);
	strlcpy((*ar0234_sd)->i2c.i2c_adapter_bdf, dev_name(dev.parent->parent->parent),
		sizeof((*ar0234_sd)->i2c.i2c_adapter_bdf));

	/* platform_data */
	pdata->port = cam_data.link;
	pdata->lanes = cam_data.lanes;
	pdata->i2c_slave_address = cam_data.i2c[0].addr;

	/* gpio */
	pdata->irq_pin = -1;
	pdata->gpios[0] = -1;
	pdata->gpios[1] = 0;
	pdata->gpios[2] = 0;
	pdata->gpios[3] = 0;

	switch (cam_data.link) {
	case 1:
		pdata->suffix = 'a';
		break;
	case 2:
		pdata->suffix = 'b';
		break;
	case 4:
		pdata->suffix = 'c';
		break;
	case 5:
		pdata->suffix = 'd';
		break;
	default:
		dev_err(&dev, "INVALID MIPI PORT");
		break;
	}

	(*ar0234_sd)->i2c.board_info.platform_data = pdata;

	return 0;
}

int lt6911uxc_populate(struct device dev,
			struct ipu_isys_subdev_info **lt6911uxc_sd,
			struct ipu_i2c_info i2c,
			char sensor_name[I2C_NAME_SIZE],
			struct sensor_bios_data cam_data,
			struct control_logic_data ctl_data)
{
	struct lt6911uxc_platform_data *pdata;
	struct ipu_isys_csi2_config *csi2_config;
	int i;
	bool irq_set = false;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	csi2_config = kzalloc(sizeof(*csi2_config), GFP_KERNEL);
	if (!csi2_config) {
		kfree(pdata);
		return -ENOMEM;
	}

	/* csi2 */
	csi2_config->nlanes = cam_data.lanes;
	csi2_config->port = cam_data.link;
	(*lt6911uxc_sd)->csi2 = csi2_config;

	/* i2c */
	(*lt6911uxc_sd)->i2c.board_info.addr = i2c.addr;
	strlcpy((*lt6911uxc_sd)->i2c.board_info.type, sensor_name, I2C_NAME_SIZE);
	strlcpy((*lt6911uxc_sd)->i2c.i2c_adapter_bdf, dev_name(dev.parent->parent->parent),
		sizeof((*lt6911uxc_sd)->i2c.i2c_adapter_bdf));

	/* platform data */
	pdata->port = cam_data.link;
	pdata->lanes = cam_data.lanes;
	pdata->i2c_slave_address = cam_data.i2c[0].addr;

	/* use ascii */
	if (cam_data.link > 0)
		pdata->suffix = cam_data.link + SUFFIX_BASE;
	else
		dev_err(&dev, "INVALID MIPI PORT");

	/* gpio */
	if (ctl_data.completed && ctl_data.gpio_num > 0) {
		for (i = 0; i < ctl_data.gpio_num; i++) {
			/* check for RESET selection in BIOS */
			if (ctl_data.gpio[i].valid && ctl_data.gpio[i].func == GPIO_RESET)
				pdata->reset_pin = ctl_data.gpio[i].pin;
			/* check for READY_STAT selection in BIOS */
			if (ctl_data.gpio[i].valid && ctl_data.gpio[i].func == GPIO_READY_STAT) {
				pdata->irq_pin = ctl_data.gpio[i].pin;
				pdata->irq_pin_flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
							IRQF_ONESHOT;
				strlcpy(pdata->irq_pin_name, "READY_STAT", sizeof("READY_STAT"));
				irq_set = true;
			}
			/* check for HDMI_DETECT selection in BIOS */
			if (ctl_data.gpio[i].valid && ctl_data.gpio[i].func == GPIO_HDMI_DETECT)
				pdata->detect_pin = ctl_data.gpio[i].pin;
		}
	}

	if (!irq_set)
		pdata->irq_pin = -1;

	pdata->gpios[0] = -1;
	pdata->gpios[1] = 0;
	pdata->gpios[2] = 0;
	pdata->gpios[3] = 0;

	(*lt6911uxc_sd)->i2c.board_info.platform_data = pdata;

	// insert pdata into local here.
	update_pdata(dev, *lt6911uxc_sd);

	/* pprval is used as an indicator to enumerate dummy port for HDMI use case */
	if (cam_data.pprval != cam_data.link) {
		struct ipu_isys_subdev_info *lt6911uxc_sd_dummy;
		struct lt6911uxc_platform_data *pdata_dummy;
		struct ipu_isys_csi2_config *csi2_config_dummy;
		unsigned short addr_dummy = 0x11;

		lt6911uxc_sd_dummy = kzalloc(sizeof(*lt6911uxc_sd_dummy), GFP_KERNEL);
		if (!lt6911uxc_sd_dummy) {
			kfree(pdata);
			kfree(csi2_config);
			return -ENOMEM;
		}

		pdata_dummy = kzalloc(sizeof(*pdata_dummy), GFP_KERNEL);
		if (!pdata_dummy) {
			kfree(pdata);
			kfree(csi2_config);
			kfree(lt6911uxc_sd_dummy);
			return -ENOMEM;
		}

		csi2_config_dummy = kzalloc(sizeof(*csi2_config_dummy), GFP_KERNEL);
		if (!csi2_config_dummy) {
			kfree(pdata);
			kfree(csi2_config);
			kfree(lt6911uxc_sd_dummy);
			kfree(pdata_dummy);
			return -ENOMEM;
		}

		pdata_dummy->port = cam_data.pprval;
		pdata_dummy->lanes = cam_data.lanes;
		pdata_dummy->i2c_slave_address = addr_dummy;
		pdata_dummy->suffix = cam_data.pprval + SUFFIX_BASE;
		pdata_dummy->irq_pin = -1;

		csi2_config_dummy->nlanes = cam_data.lanes;
		csi2_config_dummy->port = cam_data.pprval;

		lt6911uxc_sd_dummy->csi2 = csi2_config_dummy;
		lt6911uxc_sd_dummy->i2c.board_info.addr = addr_dummy;
		strlcpy(lt6911uxc_sd_dummy->i2c.board_info.type, sensor_name, I2C_NAME_SIZE);
		strlcpy(lt6911uxc_sd_dummy->i2c.i2c_adapter_bdf, dev_name(dev.parent->parent->parent),
			sizeof(lt6911uxc_sd_dummy->i2c.i2c_adapter_bdf));

		lt6911uxc_sd_dummy->i2c.board_info.platform_data = pdata_dummy;

		/* update pdata */
		update_pdata(dev, lt6911uxc_sd_dummy);

	}

	return 0;
}

static int get_lt6911uxc_pdata(struct i2c_client *client,
			       struct ipu_camera_module_data *data,
			       struct ipu_i2c_helper *helper,
			       void *priv, size_t size)
{
	struct sensor_bios_data cam_data;
	struct control_logic_data ctl_data;
	struct ipu_isys_subdev_info *lt6911uxc_sd;
	int rval;

	cam_data.dev = &client->dev;

	lt6911uxc_sd = kzalloc(sizeof(*lt6911uxc_sd), GFP_KERNEL);
	if (!lt6911uxc_sd)
		return -ENOMEM;

	/* get sensor info from ssdb table generated from BIOS, save in sensor */
	rval = ipu_acpi_get_cam_data(&client->dev, &cam_data);
	if (rval) {
		kfree(lt6911uxc_sd);
		return rval;
	}

	rval = ipu_acpi_get_dep_data(&client->dev, &ctl_data);
	if (rval) {
		kfree(lt6911uxc_sd);
		return rval;
	}

	rval = lt6911uxc_populate(client->dev, &lt6911uxc_sd, cam_data.i2c[0],
				client->name, cam_data, ctl_data);
	if (rval) {
		kfree(lt6911uxc_sd);
		return rval;
	}

	client->dev.platform_data = lt6911uxc_sd;

	return rval;
}

static int get_ar0234_pdata(struct i2c_client *client,
			       struct ipu_camera_module_data *data,
			       struct ipu_i2c_helper *helper,
			       void *priv, size_t size)
{
	struct sensor_bios_data cam_data;
	struct control_logic_data ctl_data;
	struct ipu_isys_subdev_info *ar0234_sd;
	int rval;

	cam_data.dev = &client->dev;

	ar0234_sd = kzalloc(sizeof(*ar0234_sd), GFP_KERNEL);

	if (!ar0234_sd)
		return -ENOMEM;

	/* get sensor info from ssdb table generated from BIOS, save in sensor */
	rval = ipu_acpi_get_cam_data(&client->dev, &cam_data);
	if (rval) {
		kfree(ar0234_sd);
		return rval;
	}

	rval = ipu_acpi_get_dep_data(&client->dev, &ctl_data);
	if (rval) {
		kfree(ar0234_sd);
		return rval;
	}

	rval = ar0234_populate(client->dev, &ar0234_sd, cam_data.i2c[0],
				client->name, cam_data, ctl_data);
	if (rval) {
		kfree(ar0234_sd);
		return rval;
	}

	client->dev.platform_data = ar0234_sd;

	return rval;
}

static const struct ipu_acpi_devices supported_devices[] = {
/*
 *	{ "AR0234A", AR0234_NAME, get_ar0234_pdata, NULL, 0 },
 *	{ "AR0234B", AR0234_NAME, get_ar0234_pdata, NULL, 0 },
 *	{ "AR0234C", AR0234_NAME, get_ar0234_pdata, NULL, 0 },
 *	{ "AR0234D", AR0234_NAME, get_ar0234_pdata, NULL, 0 },
 *	{ "LT6911A", LT6911UXC_NAME, get_lt6911uxc_pdata, NULL, 0 },
 *	{ "LT6911B", LT6911UXC_NAME, get_lt6911uxc_pdata, NULL, 0 },
 *	{ "LT6911C", LT6911UXC_NAME, get_lt6911uxc_pdata, NULL, 0 },
 */
	{ "INTC10B1", LT6911UXC_NAME, get_lt6911uxc_pdata, NULL, 0 },	// HID for Lontium
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
 *	{ "AR0234A", 0 },
 *	{ "AR0234B", 0 },
 *	{ "AR0234C", 0 },
 *	{ "AR0234D", 0 },
 *	{ "LT6911A", 0 },
 *	{ "LT6911B", 0 },
 *	{ "LT6911C", 0 },
 */
	{ "INTC10B1", 0 },	// HID for Lontium
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
		supported_devices[index].priv_size);

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
		*spdata = &acpi_subdev_pdata;
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
MODULE_DESCRIPTION("IPU4 ACPI support");

