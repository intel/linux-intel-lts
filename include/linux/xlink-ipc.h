/* SPDX-License-Identifier: GPL-2.0-only */
/*****************************************************************************
 *
 * Intel Keem Bay xlink IPC Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#ifndef _XLINK_IPC_H_
#define _XLINK_IPC_H_

#include <linux/types.h>

struct xlink_ipc_context {
	u16 chan;
	u8 is_volatile;
};

int xlink_ipc_connect(u32 sw_device_id);

int xlink_ipc_write(u32 sw_device_id, void *data, size_t * const size,
		    u32 timeout, void *context);

int xlink_ipc_read(u32 sw_device_id, void *data, size_t * const size,
		   u32 timeout, void *context);

int xlink_ipc_get_device_list(u32 *sw_device_id_list, u32 *num_devices);

int xlink_ipc_get_device_name(u32 sw_device_id, char *device_name,
			      size_t name_size);

int xlink_ipc_get_device_status(u32 sw_device_id, u32 *device_status);

int xlink_ipc_boot_device(u32 sw_device_id, const char *binary_name);

int xlink_ipc_reset_device(u32 sw_device_id);

int xlink_ipc_open_channel(u32 sw_device_id, u32 channel);

int xlink_ipc_close_channel(u32 sw_device_id, u32 channel);

int xlink_ipc_register_for_events(u32 sw_device_id,
				  int (*callback)(u32 sw_device_id, u32 event));

int xlink_ipc_unregister_for_events(u32 sw_device_id);

int xlink_ipc_get_device_mode(u32 sw_device_id, u32 *power_mode);

int xlink_ipc_set_device_mode(u32 sw_device_id, u32 power_mode);

#endif /* _XLINK_IPC_H_ */
