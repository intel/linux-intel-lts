/* SPDX-License-Identifier: GPL-2.0-only */
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#ifndef _XLINK_DRV_INF_H_
#define _XLINK_DRV_INF_H_

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/types.h>

#define XLINK_DEV_INF_TYPE_MASK		GENMASK(27, 24)
#define XLINK_DEV_PHYS_ID_MASK		GENMASK(23, 8)
#define XLINK_DEV_TYPE_MASK		GENMASK(6, 4)
#define XLINK_DEV_PCIE_ID_MASK		GENMASK(3, 1)
#define XLINK_DEV_FUNC_MASK		GENMASK(0, 0)

enum xlink_device_inf_type {
	XLINK_DEV_INF_PCIE = 1,
};

enum xlink_device_type {
	XLINK_DEV_TYPE_KMB = 0,
};

enum xlink_device_pcie {
	XLINK_DEV_PCIE_0 = 0,
};

enum xlink_device_func {
	XLINK_DEV_FUNC_VPU = 0,
};

enum _xlink_device_status {
	_XLINK_DEV_OFF,
	_XLINK_DEV_ERROR,
	_XLINK_DEV_BUSY,
	_XLINK_DEV_RECOVERY,
	_XLINK_DEV_READY
};

enum xlink_device_event_type {
	NOTIFY_DEVICE_DISCONNECTED,
	NOTIFY_DEVICE_CONNECTED,
	NUM_EVENT_TYPE
};

typedef int (*xlink_device_event)(u32 sw_device_id,
				  enum xlink_device_event_type event_type);

int xlink_pcie_get_device_list(u32 *sw_device_id_list,
			       u32 *num_devices);
int xlink_pcie_get_device_name(u32 sw_device_id, char *device_name,
			       size_t name_size);
int xlink_pcie_get_device_status(u32 sw_device_id,
				 u32 *device_status);
int xlink_pcie_boot_device(u32 sw_device_id, const char *binary_name);
int xlink_pcie_connect(u32 sw_device_id);
int xlink_pcie_read(u32 sw_device_id, void *data, size_t *const size,
		    u32 timeout);
int xlink_pcie_write(u32 sw_device_id, void *data, size_t *const size,
		     u32 timeout);
int xlink_pcie_reset_device(u32 sw_device_id);
int xlink_pcie_register_device_event(u32 sw_device_id,
				     xlink_device_event event_notif_fn);
int xlink_pcie_unregister_device_event(u32 sw_device_id);
#endif
