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
#include <linux/platform_device.h>
#include <media/ipu-acpi-pdata.h>
#include <media/ipu-acpi.h>

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

int ipu_acpi_get_gpio_data(struct device *dev, struct ipu_gpio_info *gpio, int size,
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

int ipu_acpi_get_i2c_info(struct device *dev, struct ipu_i2c_info *i2c, int size, u64 *num)
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

	*num = num_i2c;

	return 0;
}

static int match_depend(struct device *dev, const void *data)
{
	return (dev && dev->fwnode == data) ? 1 : 0;
}

int ipu_acpi_get_control_logic_data(struct device *dev,
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
		ctl_logic_data.controllogicid,
		ctl_logic_data.controllogictype,
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

int ipu_acpi_get_dep_data(struct device *dev,
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
			}

			ctl_data->completed = true;
		} else
			dev_err(dev, "Dependent platform device not found\n");
	}

	if (!ctl_data->completed) {
		ctl_data->type = CL_EMPTY;
		dev_err(dev, "No control logic data available");
	}

	return 0;
}
EXPORT_SYMBOL(ipu_acpi_get_dep_data);

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
	sensor->pprunit = sensor_data.pprunit;

	dev_dbg(dev, "sensor ACPI data: name %s link %d, lanes %d pprval %d pprunit %x",
		dev_name(dev), sensor->link, sensor->lanes, sensor->pprval, sensor->pprunit);

	/* I2C */
	ret = ipu_acpi_get_i2c_info(dev, sensor->i2c, ARRAY_SIZE(sensor->i2c), &sensor->i2c_num);

	if (ret < 0)
		return ret;

	return 0;
}
EXPORT_SYMBOL(ipu_acpi_get_cam_data);

MODULE_AUTHOR("Samu Onkalo <samu.onkalo@intel.com>");
MODULE_AUTHOR("Khai Wen Ng <khai.wen.ng@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("IPU6 ACPI support");

