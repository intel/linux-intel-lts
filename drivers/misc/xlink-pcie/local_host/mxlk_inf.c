// SPDX-License-Identifier: GPL-2.0-only
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/xlink_drv_inf.h>
#include "../common/mxlk_core.h"
#include "mxlk_epf.h"

int xlink_pcie_get_device_list(uint32_t *sw_device_id_list,
			       uint32_t *num_devices)
{
	if (xlink_sw_id != 0) {
		*num_devices = 1;
		*sw_device_id_list = xlink_sw_id;
	} else {
		*num_devices = 0;
	}

	return 0;
}
EXPORT_SYMBOL(xlink_pcie_get_device_list);

int xlink_pcie_get_device_name(uint32_t sw_device_id, char *device_name,
			       size_t name_size)
{
	struct mxlk *mxlk = mxlk_core_get_by_id(sw_device_id);

	if (!mxlk)
		return -ENODEV;

	memset(device_name, 0, name_size);
	if (name_size > strlen(MXLK_DRIVER_NAME))
		name_size = strlen(MXLK_DRIVER_NAME);
	strncpy(device_name, MXLK_DRIVER_NAME, name_size);

	return 0;
}
EXPORT_SYMBOL(xlink_pcie_get_device_name);

int xlink_pcie_get_device_status(uint32_t sw_device_id, uint32_t *device_status)
{
	struct mxlk *mxlk = mxlk_core_get_by_id(sw_device_id);

	if (!mxlk)
		return -ENODEV;

	switch (mxlk->status) {
	case MXLK_STATUS_READY:
	case MXLK_STATUS_RUN:
		*device_status = _XLINK_DEV_READY;
		break;
	case MXLK_STATUS_ERROR:
		*device_status = _XLINK_DEV_ERROR;
		break;
	default:
		*device_status = _XLINK_DEV_BUSY;
		break;
	}

	return 0;
}
EXPORT_SYMBOL(xlink_pcie_get_device_status);

int xlink_pcie_boot_device(uint32_t sw_device_id, const char *binary_name)
{
	return 0;
}
EXPORT_SYMBOL(xlink_pcie_boot_device);

int xlink_pcie_connect(uint32_t sw_device_id)
{
	struct mxlk *mxlk = mxlk_core_get_by_id(sw_device_id);

	if (!mxlk)
		return -ENODEV;

	if (mxlk->status != MXLK_STATUS_RUN)
		return -EIO;

	return 0;
}
EXPORT_SYMBOL(xlink_pcie_connect);

int xlink_pcie_read(uint32_t sw_device_id, void *data, size_t *const size,
		    uint32_t timeout)
{
	struct mxlk *mxlk = mxlk_core_get_by_id(sw_device_id);

	if (!mxlk)
		return -ENODEV;

	return mxlk_core_read(mxlk, data, size, timeout);
}
EXPORT_SYMBOL(xlink_pcie_read);

int xlink_pcie_write(uint32_t sw_device_id, void *data, size_t *const size,
		     uint32_t timeout)
{
	struct mxlk *mxlk = mxlk_core_get_by_id(sw_device_id);

	if (!mxlk)
		return -ENODEV;

	return mxlk_core_write(mxlk, data, size, timeout);
}
EXPORT_SYMBOL(xlink_pcie_write);

int xlink_pcie_reset_device(uint32_t sw_device_id)
{
	return 0;
}
EXPORT_SYMBOL(xlink_pcie_reset_device);
