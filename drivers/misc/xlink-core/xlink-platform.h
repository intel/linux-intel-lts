/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * xlink Linux Kernel Platform API
 *
 * Copyright (C) 2018-2019 Intel Corporation
 *
 */
#ifndef __XLINK_PLATFORM_H
#define __XLINK_PLATFORM_H

#include "xlink-defs.h"

int xlink_platform_connect(u32 interface, u32 sw_device_id);

int xlink_platform_write(u32 interface, u32 sw_device_id, void *data,
			 size_t * const size, u32 timeout, void *context);

int xlink_platform_read(u32 interface, u32 sw_device_id, void *data,
			size_t * const size, u32 timeout, void *context);

int xlink_platform_reset_device(u32 interface, u32 sw_device_id);

int xlink_platform_boot_device(u32 interface, u32 sw_device_id,
			       const char *binary_name);

int xlink_platform_get_device_name(u32 interface, u32 sw_device_id,
				   char *device_name, size_t name_size);

int xlink_platform_get_device_list(u32 interface, u32 *sw_device_id_list,
				   u32 *num_devices);

int xlink_platform_get_device_status(u32 interface, u32 sw_device_id,
				     u32 *device_status);

int xlink_platform_set_device_mode(u32 interface, u32 sw_device_id,
				   u32 power_mode);

int xlink_platform_get_device_mode(u32 interface, u32 sw_device_id,
				   u32 *power_mode);

int xlink_platform_open_channel(u32 interface,  u32 sw_device_id,
				u32 channel);

int xlink_platform_close_channel(u32 interface,  u32 sw_device_id,
				 u32 channel);

int xlink_platform_register_for_events(u32 interface, u32 sw_device_id,
				       xlink_device_event_cb event_notif_fn);

int xlink_platform_unregister_for_events(u32 interface, u32 sw_device_id);

enum xlink_memory_region {
	XLINK_NORMAL_MEMORY = 0,
	XLINK_CMA_MEMORY
};

void *xlink_platform_allocate(struct device *dev, dma_addr_t *handle,
			      u32 size, u32 alignment,
			      enum xlink_memory_region region);

void xlink_platform_deallocate(struct device *dev, void *buf,
			       dma_addr_t handle, u32 size, u32 alignment,
			       enum xlink_memory_region region);

#endif /* __XLINK_PLATFORM_H */
