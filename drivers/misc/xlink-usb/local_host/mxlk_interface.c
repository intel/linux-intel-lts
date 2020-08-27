// SPDX-License-Identifier: GPL-2.0-only
/*****************************************************************************
 *
 * Intel Keem Bay XLink VPU USB Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/xlink_drv_inf.h>
#include "u_xlink.h"

#define USB_VENDOR_ID_INTEL_KEEMBAY  0x03e7
#define MXLK_DRIVER_NAME "mxlk_usb_epf"

int xlink_usb_get_device_list(uint32_t *sw_device_id_list,
			       uint32_t *num_devices)
{
	int pid = 0;
	u32 xlink_usb_sw_id;

	xlink_usb_sw_id = usb_sw_id(pid);
	if (xlink_usb_sw_id != 0) {
		*num_devices = 1;
		*sw_device_id_list = xlink_usb_sw_id;
	} else {
		*num_devices = 0;
	}
	return 0;
}
EXPORT_SYMBOL(xlink_usb_get_device_list);

int xlink_usb_get_device_name(uint32_t sw_device_id, char *device_name,
			       size_t name_size)
{
	int mxlk;

	mxlk = usb_get_by_id(sw_device_id);
	if (!mxlk)
		return -ENODEV;

	memset(device_name, 0, name_size);
	if (name_size > strlen(MXLK_DRIVER_NAME))
		name_size = strlen(MXLK_DRIVER_NAME);
	strncpy(device_name, MXLK_DRIVER_NAME, name_size);

	return 0;
}
EXPORT_SYMBOL(xlink_usb_get_device_name);

int xlink_usb_get_device_status(uint32_t sw_device_id,
				 uint32_t *device_status)
{
	int mxlk;

	mxlk = usb_get_by_id(sw_device_id);
	if (!mxlk) {
		*device_status = 1;
		return -ENODEV;
	}
	*device_status = 4;
	return 0;
}
EXPORT_SYMBOL(xlink_usb_get_device_status);

int xlink_usb_boot_device(uint32_t sw_device_id, const char *binary_name)
{
	return 0;
}
EXPORT_SYMBOL(xlink_usb_boot_device);

int xlink_usb_connect(uint32_t sw_device_id)
{
	int mxlk;

	mxlk = usb_get_by_id(sw_device_id);
	if (!mxlk)
		return -ENODEV;

	return 0;
}
EXPORT_SYMBOL(xlink_usb_connect);

int xlink_usb_read(uint32_t sw_device_id, void *data, size_t *const size,
		    uint32_t timeout)
{
	int mxlk;

	mxlk = usb_get_by_id(sw_device_id);
	if (!mxlk)
		return -ENODEV;
	return vpu_read(data, size, timeout);
}
EXPORT_SYMBOL(xlink_usb_read);

int xlink_usb_write(uint32_t sw_device_id, void *data, size_t *const size,
		     uint32_t timeout)
{
	int mxlk;
	int rc;

	mxlk = usb_get_by_id(sw_device_id);
	if (!mxlk)
		return -ENODEV;
	rc = vpu_write1(data, size, timeout);
	if (rc < 0)
		return rc;
	return vpu_close1(data);
}
EXPORT_SYMBOL(xlink_usb_write);

int xlink_usb_reset_device(uint32_t sw_device_id)
{
	return 0;
}
EXPORT_SYMBOL(xlink_usb_reset_device);
