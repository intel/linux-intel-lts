/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * xlink Linux Kernel Platform API
 *
 * Copyright (C) 2018-2019 Intel Corporation
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/xlink_drv_inf.h>

#include "xlink-platform.h"

#if IS_ENABLED(CONFIG_XLINK_LOCAL_HOST)
#include <linux/xlink-ipc.h>
//int xlink_pcie_register_device_event(uint32_t sw_device_id,
//	xlink_device_event event_notif_fn) { return -1; };
//int xlink_pcie_unregister_device_event(uint32_t sw_device_id) { return -1; }
#else /* !CONFIG_XLINK_LOCAL_HOST */
static inline int xlink_ipc_connect(u32 sw_device_id) { return -1; }
static inline int xlink_ipc_write(u32 sw_device_id, void *data,
		size_t * const size, u32 timeout, void *context)  { return -1; }
static inline int xlink_ipc_read(u32 sw_device_id, void *data,
		size_t * const size, u32 timeout, void *context)  { return -1; }
static inline int xlink_ipc_get_device_list(u32 *sw_device_id_list,
		u32 *num_devices) { return -1; }
static inline int xlink_ipc_get_device_name(u32 sw_device_id,
		char *device_name, size_t name_size) { return -1; }
static inline int xlink_ipc_get_device_status(u32 sw_device_id,
		u32 *device_status) { return -1; }
static inline int xlink_ipc_boot_device(u32 sw_device_id,
		const char *binary_path) { return -1; }
static inline int xlink_ipc_reset_device(u32 sw_device_id) { return -1; }
static inline int xlink_ipc_open_channel(u32 sw_device_id,
		u32 channel) { return -1; }
static inline int xlink_ipc_close_channel(u32 sw_device_id,
		u32 channel) { return -1; }
static inline int xlink_ipc_register_for_events(u32 sw_device_id,
		int (*callback)(u32 sw_device_id, u32 event)) { return -1; }
static inline int xlink_ipc_unregister_for_events(u32 sw_device_id) { return -1; }

static inline int xlink_ipc_get_device_mode(u32 sw_device_id, u32 *power_mode) { return -1; }

static inline int xlink_ipc_set_device_mode(u32 sw_device_id, u32 power_mode) { return -1; }

#endif /* CONFIG_XLINK_LOCAL_HOST */

/*
 * xlink low-level driver interface arrays
 *
 * note: array indices based on xlink_interface enum definition
 */

int (*connect_fcts[NMB_OF_INTERFACES])(uint32_t) = \
		{xlink_ipc_connect, xlink_pcie_connect, NULL, NULL};

int (*write_fcts[NMB_OF_INTERFACES])(uint32_t, void *, size_t * const, uint32_t) = \
		{NULL, xlink_pcie_write, NULL, NULL};

int (*read_fcts[NMB_OF_INTERFACES])(uint32_t, void *, size_t * const, uint32_t) = \
		{NULL, xlink_pcie_read, NULL, NULL};

int (*reset_fcts[NMB_OF_INTERFACES])(uint32_t) = \
		{xlink_ipc_reset_device, xlink_pcie_reset_device, NULL, NULL};

int (*boot_fcts[NMB_OF_INTERFACES])(uint32_t, const char *) = \
		{xlink_ipc_boot_device, xlink_pcie_boot_device, NULL, NULL};

int (*dev_name_fcts[NMB_OF_INTERFACES])(uint32_t, char *, size_t) = \
		{xlink_ipc_get_device_name, xlink_pcie_get_device_name, NULL, NULL};

int (*dev_list_fcts[NMB_OF_INTERFACES])(uint32_t *, uint32_t *) = \
		{xlink_ipc_get_device_list, xlink_pcie_get_device_list, NULL, NULL};

int (*dev_status_fcts[NMB_OF_INTERFACES])(uint32_t, uint32_t *) = \
		{xlink_ipc_get_device_status, xlink_pcie_get_device_status, NULL, NULL};

int (*dev_set_mode_fcts[NMB_OF_INTERFACES])(uint32_t, uint32_t) = \
		{xlink_ipc_set_device_mode, NULL, NULL, NULL};

int (*dev_get_mode_fcts[NMB_OF_INTERFACES])(uint32_t, uint32_t *) = \
		{xlink_ipc_get_device_mode, NULL, NULL, NULL};

int (*open_chan_fcts[NMB_OF_INTERFACES])(uint32_t, uint32_t) = \
		{xlink_ipc_open_channel, NULL, NULL, NULL};

int (*close_chan_fcts[NMB_OF_INTERFACES])(uint32_t, uint32_t) = \
		{xlink_ipc_close_channel, NULL, NULL, NULL};


int (*register_for_events_fcts[NMB_OF_INTERFACES])(uint32_t,
		int (*callback)(u32 sw_device_id, u32 event)) = {xlink_ipc_register_for_events,
				xlink_pcie_register_device_event, NULL, NULL};

int (*unregister_for_events_fcts[NMB_OF_INTERFACES])(uint32_t) = {
		xlink_ipc_unregister_for_events, xlink_pcie_unregister_device_event, NULL, NULL};

/*
 * xlink low-level driver interface
 */

int xlink_platform_connect(uint32_t interface, uint32_t sw_device_id)
{
	if (interface >= NMB_OF_INTERFACES || !connect_fcts[interface])
		return -1;

	return connect_fcts[interface](sw_device_id);
}
int xlink_platform_write(uint32_t interface, uint32_t sw_device_id, void *data,
		size_t * const size, uint32_t timeout, void *context)
{
	if (interface == IPC_INTERFACE)
		return xlink_ipc_write(sw_device_id, data, size, timeout, context);

	if (interface >= NMB_OF_INTERFACES || !write_fcts[interface])
		return -1;

	return write_fcts[interface](sw_device_id, data, size, timeout);
}

int xlink_platform_read(uint32_t interface, uint32_t sw_device_id, void *data,
		size_t * const size, uint32_t timeout, void *context)
{
	if (interface == IPC_INTERFACE)
		return xlink_ipc_read(sw_device_id, data, size, timeout, context);

	if (interface >= NMB_OF_INTERFACES || !read_fcts[interface])
		return -1;

	return read_fcts[interface](sw_device_id, data, size, timeout);
}

int xlink_platform_reset_device(uint32_t interface, uint32_t sw_device_id)
{
	if (interface >= NMB_OF_INTERFACES || !reset_fcts[interface])
		return -1;

	return reset_fcts[interface](sw_device_id);
}

int xlink_platform_boot_device(uint32_t interface, uint32_t sw_device_id,
		const char *binary_name)
{
	if (interface >= NMB_OF_INTERFACES || !boot_fcts[interface])
		return -1;

	return boot_fcts[interface](sw_device_id, binary_name);
}

int xlink_platform_get_device_name(uint32_t interface, uint32_t sw_device_id,
		char *device_name, size_t name_size)
{
	if (interface >= NMB_OF_INTERFACES || !dev_name_fcts[interface])
		return -1;

	return dev_name_fcts[interface](sw_device_id, device_name, name_size);
}

int xlink_platform_get_device_list(uint32_t interface,
		uint32_t *sw_device_id_list, uint32_t *num_devices)
{
	if (interface >= NMB_OF_INTERFACES || !dev_list_fcts[interface])
		return -1;

	return dev_list_fcts[interface](sw_device_id_list, num_devices);
}

int xlink_platform_get_device_status(uint32_t interface, uint32_t sw_device_id,
		uint32_t *device_status)
{
	if (interface >= NMB_OF_INTERFACES || !dev_status_fcts[interface])
		return -1;

	return dev_status_fcts[interface](sw_device_id, device_status);
}

int xlink_platform_set_device_mode(uint32_t interface, uint32_t sw_device_id,
		uint32_t power_mode)
{
	if (interface >= NMB_OF_INTERFACES || !dev_set_mode_fcts[interface])
		return -1;

	return dev_set_mode_fcts[interface](sw_device_id, power_mode);
}

int xlink_platform_get_device_mode(uint32_t interface, uint32_t sw_device_id,
		uint32_t *power_mode)
{
	if (interface >= NMB_OF_INTERFACES || !dev_get_mode_fcts[interface])
		return -1;

	return dev_get_mode_fcts[interface](sw_device_id, power_mode);
}

int xlink_platform_open_channel(uint32_t interface, uint32_t sw_device_id,
		uint32_t channel)
{
	if (interface >= NMB_OF_INTERFACES || !open_chan_fcts[interface])
		return -1;

	return open_chan_fcts[interface](sw_device_id, channel);
}

int xlink_platform_close_channel(uint32_t interface, uint32_t sw_device_id,
		uint32_t channel)
{
	if (interface >= NMB_OF_INTERFACES || !close_chan_fcts[interface])
		return -1;

	return close_chan_fcts[interface](sw_device_id, channel);
}

int xlink_platform_register_for_events(uint32_t interface, uint32_t sw_device_id,
		xlink_device_event_cb event_notif_fn)
{
	if (interface >= NMB_OF_INTERFACES || !register_for_events_fcts[interface])
		return -1;

	return register_for_events_fcts[interface](sw_device_id, event_notif_fn);
}

int xlink_platform_unregister_for_events(uint32_t interface, uint32_t sw_device_id)
{
	if (interface >= NMB_OF_INTERFACES || !unregister_for_events_fcts[interface])
		return -1;

	return unregister_for_events_fcts[interface](sw_device_id);
}




void *xlink_platform_allocate(struct device *dev, dma_addr_t *handle,
		uint32_t size, uint32_t alignment,
		enum xlink_memory_region region)
{
#if defined(CONFIG_XLINK_PSS) || !defined(CONFIG_XLINK_LOCAL_HOST)
	*handle = 0;
	return kzalloc(size, GFP_KERNEL);
#else
	void *p;
	if (region == XLINK_CMA_MEMORY) {
		// size needs to be at least 4097 to be allocated from CMA
		size = (size < PAGE_SIZE * 2) ? (PAGE_SIZE * 2) : size;
		p = dma_alloc_coherent(dev, size, handle, GFP_KERNEL);
		return p;
	} else {
		*handle = 0;
		return kzalloc(size, GFP_KERNEL);
	}
#endif
}

void xlink_platform_deallocate(struct device *dev, void *buf,
		dma_addr_t handle, uint32_t size, uint32_t alignment,
		enum xlink_memory_region region)
{
#if defined(CONFIG_XLINK_PSS) || !defined(CONFIG_XLINK_LOCAL_HOST)
	if (buf) {
		kfree(buf);
	}
#else
	if (region == XLINK_CMA_MEMORY) {
		// size needs to be at least 4097 to be allocated from CMA
		size = (size < PAGE_SIZE * 2) ? (PAGE_SIZE * 2) : size;
		dma_free_coherent(dev, size, buf, handle);
	} else {
		if (buf) {
			kfree(buf);
		}
	}
#endif
}
