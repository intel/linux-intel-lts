// SPDX-License-Identifier: GPL-2.0-only
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
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/xlink_drv_inf.h>

#include "xlink-platform.h"

#ifdef CONFIG_XLINK_LOCAL_HOST
#include <linux/xlink-ipc.h>
#else /* !CONFIG_XLINK_LOCAL_HOST */

static inline int xlink_ipc_connect(u32 sw_device_id)
{ return -1; }

static inline int xlink_ipc_write(u32 sw_device_id, void *data,
				  size_t * const size, u32 timeout, void *context)
{ return -1; }

static inline int xlink_ipc_read(u32 sw_device_id, void *data,
				 size_t * const size, u32 timeout, void *context)
{ return -1; }

static inline int xlink_ipc_open_channel(u32 sw_device_id,
					 u32 channel)
{ return -1; }

static inline int xlink_ipc_close_channel(u32 sw_device_id,
					  u32 channel)
{ return -1; }

#endif /* CONFIG_XLINK_LOCAL_HOST */

/*
 * xlink low-level driver interface arrays
 *
 * note: array indices based on xlink_interface enum definition
 */

static int (*connect_fcts[NMB_OF_INTERFACES])(u32) = {
		xlink_ipc_connect, xlink_pcie_connect, NULL, NULL};

static int (*write_fcts[NMB_OF_INTERFACES])(u32, void *, size_t * const, u32) = {
		NULL, xlink_pcie_write, NULL, NULL};

static int (*read_fcts[NMB_OF_INTERFACES])(u32, void *, size_t * const, u32) = {
		NULL, xlink_pcie_read, NULL, NULL};

static int (*open_chan_fcts[NMB_OF_INTERFACES])(u32, u32) = {
		xlink_ipc_open_channel, NULL, NULL, NULL};

static int (*close_chan_fcts[NMB_OF_INTERFACES])(u32, u32) = {
		xlink_ipc_close_channel, NULL, NULL, NULL};

/*
 * xlink low-level driver interface
 */

int xlink_platform_connect(u32 interface, u32 sw_device_id)
{
	if (interface >= NMB_OF_INTERFACES || !connect_fcts[interface])
		return -1;

	return connect_fcts[interface](sw_device_id);
}

int xlink_platform_write(u32 interface, u32 sw_device_id, void *data,
			 size_t * const size, u32 timeout, void *context)
{
	if (interface == IPC_INTERFACE)
		return xlink_ipc_write(sw_device_id, data, size, timeout,
				context);

	if (interface >= NMB_OF_INTERFACES || !write_fcts[interface])
		return -1;

	return write_fcts[interface](sw_device_id, data, size, timeout);
}

int xlink_platform_read(u32 interface, u32 sw_device_id, void *data,
			size_t * const size, u32 timeout, void *context)
{
	if (interface == IPC_INTERFACE)
		return xlink_ipc_read(sw_device_id, data, size, timeout,
				context);

	if (interface >= NMB_OF_INTERFACES || !read_fcts[interface])
		return -1;

	return read_fcts[interface](sw_device_id, data, size, timeout);
}

int xlink_platform_open_channel(u32 interface, u32 sw_device_id,
				u32 channel)
{
	if (interface >= NMB_OF_INTERFACES || !open_chan_fcts[interface])
		return -1;

	return open_chan_fcts[interface](sw_device_id, channel);
}

int xlink_platform_close_channel(u32 interface, u32 sw_device_id,
				 u32 channel)
{
	if (interface >= NMB_OF_INTERFACES || !close_chan_fcts[interface])
		return -1;

	return close_chan_fcts[interface](sw_device_id, channel);
}

void *xlink_platform_allocate(struct device *dev, dma_addr_t *handle,
			      u32 size, u32 alignment,
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
	}
	*handle = 0;
	return kzalloc(size, GFP_KERNEL);
#endif
}

void xlink_platform_deallocate(struct device *dev, void *buf,
			       dma_addr_t handle, u32 size, u32 alignment,
			       enum xlink_memory_region region)
{
#if defined(CONFIG_XLINK_PSS) || !defined(CONFIG_XLINK_LOCAL_HOST)
	kfree(buf);
#else
	if (region == XLINK_CMA_MEMORY) {
		// size needs to be at least 4097 to be allocated from CMA
		size = (size < PAGE_SIZE * 2) ? (PAGE_SIZE * 2) : size;
		dma_free_coherent(dev, size, buf, handle);
	} else {
		kfree(buf);
	}
#endif
}
