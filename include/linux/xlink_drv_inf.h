/* SPDX-License-Identifier: GPL-2.0-only */
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2019 Intel Corporation
 *
 ****************************************************************************/

#ifndef _XLINK_DRV_INF_H_
#define _XLINK_DRV_INF_H_

#include <linux/types.h>

#define XLINK_DEV_INF_TYPE_SHIFT (24)
#define XLINK_DEV_INT_TYPE_MASK (0x7)
#define XLINK_DEV_PHYS_ID_SHIFT (8)
#define XLINK_DEV_PHYS_ID_MASK (0xFFFF)
#define XLINK_DEV_TYPE_SHIFT (4)
#define XLINK_DEV_TYPE_MASK (0xF)
#define XLINK_DEV_SLICE_ID_SHIFT (1)
#define XLINK_DEV_SLICE_ID_MASK (0x7)
#define XLINK_DEV_FUNC_SHIFT (0)
#define XLINK_DEV_FUNC_MASK (0x1)

enum xlink_device_inf_type {
	XLINK_DEV_INF_IPC = 0,
	XLINK_DEV_INF_PCIE = 1,
	XLINK_DEV_INF_USB = 2,
	XLINK_DEV_INF_ETH = 3
};

enum xlink_device_type {
	XLINK_DEV_TYPE_KMB = 0,
	XLINK_DEV_TYPE_THB_PRIME = 1,
	XLINK_DEV_TYPE_THB_STANDARD = 2,
	XLINK_DEV_TYPE_OYB = 3,
	XLINK_DEV_TYPE_MTL = 4,
	XLINK_DEV_TYPE_STF = 5
};

enum xlink_device_slice {
	XLINK_DEV_SLICE_0 = 0,
	XLINK_DEV_SLICE_1 = 1,
	XLINK_DEV_SLICE_2 = 2,
	XLINK_DEV_SLICE_3 = 3
};

enum xlink_device_func {
	XLINK_DEV_FUNC_VPU = 0,
	XLINK_DEV_FUNC_MEDIA = 1
};

enum _xlink_device_status {
	_XLINK_DEV_OFF,
	_XLINK_DEV_ERROR,
	_XLINK_DEV_BUSY,
	_XLINK_DEV_RECOVERY,
	_XLINK_DEV_READY
};

int xlink_pcie_get_device_list(uint32_t *sw_device_id_list,
			       uint32_t *num_devices);
int xlink_pcie_get_device_name(uint32_t sw_device_id, char *device_name,
			       size_t name_size);
int xlink_pcie_get_device_status(uint32_t sw_device_id,
				 uint32_t *device_status);
int xlink_pcie_boot_device(uint32_t sw_device_id, const char *binary_name);
int xlink_pcie_connect(uint32_t sw_device_id);
int xlink_pcie_read(uint32_t sw_device_id, void *data, size_t *const size,
		    uint32_t timeout);
int xlink_pcie_write(uint32_t sw_device_id, void *data, size_t *const size,
		     uint32_t timeout);
int xlink_pcie_reset_device(uint32_t sw_device_id);

int xlink_usb_get_device_list(uint32_t *sw_device_id_list,
			       uint32_t *num_devices);
int xlink_usb_get_device_name(uint32_t sw_device_id, char *device_name,
			       size_t name_size);
int xlink_usb_get_device_status(uint32_t sw_device_id,
				 uint32_t *device_status);
int xlink_usb_boot_device(uint32_t sw_device_id, const char *binary_name);
int xlink_usb_connect(uint32_t sw_device_id);
int xlink_usb_read(uint32_t sw_device_id, void *data, size_t *const size,
		    uint32_t timeout);
int xlink_usb_write(uint32_t sw_device_id, void *data, size_t *const size,
		     uint32_t timeout);
int xlink_usb_reset_device(uint32_t sw_device_id);

#endif
