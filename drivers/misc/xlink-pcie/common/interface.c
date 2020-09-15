// SPDX-License-Identifier: GPL-2.0-only
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#include <linux/xlink_drv_inf.h>

#include "core.h"
#include "xpcie.h"

extern int intel_xpcie_pci_boot_device(u32 id, const char *binary_name);

/* Define xpcie driver interface API */
int xlink_pcie_get_device_list(u32 *sw_device_id_list, u32 *num_devices)
{
	if (!sw_device_id_list || !num_devices)
		return -EINVAL;

	*num_devices = intel_xpcie_get_device_num(sw_device_id_list);

	return 0;
}
EXPORT_SYMBOL(xlink_pcie_get_device_list);

int xlink_pcie_get_device_name(u32 sw_device_id, char *device_name,
			       size_t name_size)
{
	if (!device_name)
		return -EINVAL;

	return intel_xpcie_get_device_name_by_id(sw_device_id,
						 device_name, name_size);
}
EXPORT_SYMBOL(xlink_pcie_get_device_name);

int xlink_pcie_get_device_status(u32 sw_device_id, u32 *device_status)
{
	u32 status;
	int rc;

	if (!device_status)
		return -EINVAL;

	rc = intel_xpcie_get_device_status_by_id(sw_device_id, &status);
	if (rc)
		return rc;

	switch (status) {
	case XPCIE_STATUS_READY:
	case XPCIE_STATUS_RUN:
		*device_status = _XLINK_DEV_READY;
		break;
	case XPCIE_STATUS_ERROR:
		*device_status = _XLINK_DEV_ERROR;
		break;
	case XPCIE_STATUS_RECOVERY:
		*device_status = _XLINK_DEV_RECOVERY;
		break;
	case XPCIE_STATUS_OFF:
		*device_status = _XLINK_DEV_OFF;
		break;
	default:
		*device_status = _XLINK_DEV_BUSY;
		break;
	}

	return 0;
}
EXPORT_SYMBOL(xlink_pcie_get_device_status);

int xlink_pcie_boot_device(u32 sw_device_id, const char *binary_name)
{
#ifdef XLINK_PCIE_REMOTE
	return intel_xpcie_pci_boot_device(sw_device_id, binary_name);
#else
	return 0;
#endif
}
EXPORT_SYMBOL(xlink_pcie_boot_device);

int xlink_pcie_connect(u32 sw_device_id)
{
	return intel_xpcie_pci_connect_device(sw_device_id);
}
EXPORT_SYMBOL(xlink_pcie_connect);

int xlink_pcie_read(u32 sw_device_id, void *data, size_t *const size,
		    u32 timeout)
{
	if (!data || !size)
		return -EINVAL;

	return intel_xpcie_pci_read(sw_device_id, data, size, timeout);
}
EXPORT_SYMBOL(xlink_pcie_read);

int xlink_pcie_write(u32 sw_device_id, void *data, size_t *const size,
		     u32 timeout)
{
	if (!data || !size)
		return -EINVAL;

	return intel_xpcie_pci_write(sw_device_id, data, size, timeout);
}
EXPORT_SYMBOL(xlink_pcie_write);

int xlink_pcie_reset_device(u32 sw_device_id)
{
	return intel_xpcie_pci_reset_device(sw_device_id);
}
EXPORT_SYMBOL(xlink_pcie_reset_device);

int xlink_pcie_register_device_event(u32 sw_device_id,
				     xlink_device_event event_notif_fn)
{
	if (!event_notif_fn)
		return -EINVAL;

	return intel_xpcie_pci_register_device_event(sw_device_id,
						     event_notif_fn);
}
EXPORT_SYMBOL(xlink_pcie_register_device_event);

int xlink_pcie_unregister_device_event(u32 sw_device_id)
{
	return intel_xpcie_pci_unregister_device_event(sw_device_id);
}
EXPORT_SYMBOL(xlink_pcie_unregister_device_event);
