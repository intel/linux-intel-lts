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

int xlink_platform_connect(uint32_t interface, uint32_t sw_device_id);

int xlink_platform_write(uint32_t interface, uint32_t sw_device_id, void *data,
		size_t * const size, uint32_t timeout, void *context);

int xlink_platform_read(uint32_t interface, uint32_t sw_device_id, void *data,
		size_t * const size, uint32_t timeout, void *context);

int xlink_platform_reset_device(uint32_t interface, uint32_t sw_device_id);

int xlink_platform_boot_device(uint32_t interface, uint32_t sw_device_id,
		const char *binary_name);

int xlink_platform_get_device_name(uint32_t interface, uint32_t sw_device_id,
		char *device_name, size_t name_size);

int xlink_platform_get_device_list(uint32_t interface,
		uint32_t *sw_device_id_list, uint32_t *num_devices);

int xlink_platform_get_device_status(uint32_t interface, uint32_t sw_device_id,
		uint32_t *device_status);

int xlink_platform_set_device_mode(uint32_t interface, uint32_t sw_device_id,
		uint32_t power_mode);

int xlink_platform_get_device_mode(uint32_t interface, uint32_t sw_device_id,
		uint32_t *power_mode);

int xlink_platform_open_channel(uint32_t interface,  uint32_t sw_device_id,
		uint32_t channel);

int xlink_platform_close_channel(uint32_t interface,  uint32_t sw_device_id,
		uint32_t channel);

int xlink_platform_register_for_events(uint32_t interface, uint32_t sw_device_id,
		xlink_device_event_cb event_notif_fn);

int xlink_platform_unregister_for_events(uint32_t interface, uint32_t sw_device_id);

enum xlink_memory_region {
	XLINK_NORMAL_MEMORY = 0,
	XLINK_CMA_MEMORY
};

void *xlink_platform_allocate(struct device *dev, dma_addr_t *handle,
		uint32_t size, uint32_t alignment, enum xlink_memory_region region);

void xlink_platform_deallocate(struct device *dev, void *buf,
		dma_addr_t handle, uint32_t size, uint32_t alignment,
		enum xlink_memory_region region);

#endif /* __XLINK_PLATFORM_H */
