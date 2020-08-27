// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel Keem Bay XLink VPU USB host Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2, as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/errno.h>
#include <linux/module.h>
#include "vpu-cdc-acm.h"
#include <linux/xlink_drv_inf.h>

int xlink_usb_read(uint32_t sw_device_id, void *data, size_t *const size,
		    uint32_t timeout)
{
	return vpu_read_host(sw_device_id, data, size, timeout);
}
EXPORT_SYMBOL(xlink_usb_read);

int xlink_usb_write(uint32_t sw_device_id, void *data, size_t *const size,
		     uint32_t timeout)
{
	int rc;

	rc = vpu_write_host(sw_device_id, data, size, timeout);
	if (rc < 0)
		return rc;
	return vpu_close_host(sw_device_id, size);
}
EXPORT_SYMBOL(xlink_usb_write);

int xlink_usb_get_device_list(uint32_t *sw_device_id_list,
			       uint32_t *num_devices)
{
	*num_devices = usb_get_device_num(sw_device_id_list);
	return 0;
}
EXPORT_SYMBOL(xlink_usb_get_device_list);

int xlink_usb_get_device_name(uint32_t sw_device_id, char *device_name,
			       size_t name_size)
{
	return usb_get_device_name_by_id(sw_device_id, device_name, name_size);
}
EXPORT_SYMBOL(xlink_usb_get_device_name);

int xlink_usb_get_device_status(uint32_t sw_device_id,
				 uint32_t *device_status)
{
	usb_get_device_status_by_id(sw_device_id, device_status);
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
	return usb_connect_device(sw_device_id);
}
EXPORT_SYMBOL(xlink_usb_connect);

int xlink_usb_reset_device(uint32_t sw_device_id)
{
	int rc;

	rc = vpu_write_reset_host(sw_device_id);
	if (rc != 0)
		return rc;
	rc = vpu_close_reset_host(sw_device_id);
	if (rc != 0)
		return rc;
	return usb_device_reset(sw_device_id);
}
EXPORT_SYMBOL(xlink_usb_reset_device);
