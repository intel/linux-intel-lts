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
#include "mxlk_pci.h"

int xlink_pcie_get_device_list(uint32_t *sw_device_id_list,
			       uint32_t *num_devices)
{
	*num_devices = mxlk_get_device_num(sw_device_id_list);

	return 0;
}
EXPORT_SYMBOL(xlink_pcie_get_device_list);

int xlink_pcie_get_device_name(uint32_t sw_device_id, char *device_name,
			       size_t name_size)
{
	return mxlk_get_device_name_by_id(sw_device_id, device_name, name_size);
}
EXPORT_SYMBOL(xlink_pcie_get_device_name);

int xlink_pcie_get_device_status(uint32_t sw_device_id, uint32_t *device_status)
{
	int rc;
	u32 status;

	rc = mxlk_get_device_status_by_id(sw_device_id, &status);
	if (rc)
		return rc;

	switch (status) {
	case MXLK_STATUS_READY:
	case MXLK_STATUS_RUN:
		*device_status = _XLINK_DEV_READY;
		break;
	case MXLK_STATUS_ERROR:
		*device_status = _XLINK_DEV_ERROR;
		break;
	case MXLK_STATUS_RECOVERY:
		*device_status = _XLINK_DEV_RECOVERY;
		break;
	case MXLK_STATUS_OFF:
		*device_status = _XLINK_DEV_OFF;
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
	return mxlk_pci_boot_device(sw_device_id, binary_name);
}
EXPORT_SYMBOL(xlink_pcie_boot_device);

int xlink_pcie_connect(uint32_t sw_device_id)
{
	return mxlk_pci_connect_device(sw_device_id);
}
EXPORT_SYMBOL(xlink_pcie_connect);

int xlink_pcie_read(uint32_t sw_device_id, void *data, size_t *const size,
		    uint32_t timeout)
{
	return mxlk_pci_read(sw_device_id, data, size, timeout);
}
EXPORT_SYMBOL(xlink_pcie_read);

int xlink_pcie_write(uint32_t sw_device_id, void *data, size_t *const size,
		     uint32_t timeout)
{
	return mxlk_pci_write(sw_device_id, data, size, timeout);
}
EXPORT_SYMBOL(xlink_pcie_write);

int xlink_pcie_reset_device(uint32_t sw_device_id)
{
	return mxlk_pci_reset_device(sw_device_id);
}
EXPORT_SYMBOL(xlink_pcie_reset_device);
