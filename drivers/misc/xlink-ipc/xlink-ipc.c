// SPDX-License-Identifier: GPL-2.0-only
/*
 * xlink Linux Kernel Platform API
 *
 * Copyright (C) 2018-2020 Intel Corporation
 *
 */
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/keembay-vpu-ipc.h>

#include <linux/xlink-ipc.h>

#define XLINK_IPC_MAX_DEVICE_NAME_SIZE	12

/* used to extract fields from and create xlink sw device id */
#define SW_DEVICE_ID_INTERFACE_SHIFT	24U
#define SW_DEVICE_ID_INTERFACE_MASK	0x7
#define SW_DEVICE_ID_INTERFACE_SMASK \
		(SW_DEVICE_ID_INTERFACE_MASK << SW_DEVICE_ID_INTERFACE_SHIFT)
#define SW_DEVICE_ID_INTERFACE_IPC_VALUE 0x0
#define SW_DEVICE_ID_INTERFACE_IPC_SVALUE \
		(SW_DEVICE_ID_INTERFACE_IPC_VALUE << SW_DEVICE_ID_INTERFACE_SHIFT)
#define SW_DEVICE_ID_VPU_ID_SHIFT 1U
#define SW_DEVICE_ID_VPU_ID_MASK 0x7
#define SW_DEVICE_ID_VPU_ID_SMASK \
		(SW_DEVICE_ID_VPU_ID_MASK << SW_DEVICE_ID_VPU_ID_SHIFT)
#define GET_VPU_ID_FROM_SW_DEVICE_ID(id) \
		(((id) >> SW_DEVICE_ID_VPU_ID_SHIFT) & SW_DEVICE_ID_VPU_ID_MASK)
#define GET_SW_DEVICE_ID_FROM_VPU_ID(id) \
		((((id) << SW_DEVICE_ID_VPU_ID_SHIFT) & SW_DEVICE_ID_VPU_ID_SMASK) \
		| SW_DEVICE_ID_INTERFACE_IPC_SVALUE)

/* the maximum buffer size for volatile xlink operations */
#define XLINK_MAX_BUF_SIZE 128U

/* indices used to retrieve reserved memory from the dt */
#define LOCAL_XLINK_IPC_BUFFER_IDX	0
#define REMOTE_XLINK_IPC_BUFFER_IDX	1

/* index used to retrieve the vpu ipc device phandle from the dt */
#define VPU_IPC_DEVICE_PHANDLE_IDX	1

/* the timeout (in ms) used to wait for the vpu ready message */
#define XLINK_VPU_WAIT_FOR_READY_MS 3000

#define IP_CONTROL_CHANNEL (0x0A)
static struct mutex get_device_mode_lock;
static struct mutex set_device_mode_lock;

/* xlink buffer memory region */
struct xlink_buf_mem {
	struct device *dev;	/* child device managing the memory region */
	void *vaddr;		/* the virtual address of the memory region */
	dma_addr_t dma_handle;	/* the physical address of the memory region */
	size_t size;		/* the size of the memory region */
};

/* xlink buffer pool */
struct xlink_buf_pool {
	void *buf;		/* pointer to the start of pool area */
	size_t buf_cnt;		/* pool size (i.e., number of buffers) */
	size_t idx;		/* current index */
	spinlock_t lock;	/* the lock protecting this pool */
};

/* xlink ipc device */
struct xlink_ipc_dev {
	struct platform_device *pdev;		/* pointer to platform device */
	u32 vpu_id;				/* The VPU ID defined in the device tree */
	u32 sw_device_id;			/* the sw device id */
	const char *device_name;		/* the vpu device name */
	struct xlink_buf_mem local_xlink_mem;	/* tx buffer memory region */
	struct xlink_buf_mem remote_xlink_mem;	/* rx buffer memory region */
	struct xlink_buf_pool xlink_buf_pool;	/* tx buffer pool */
	struct device *vpu_dev;			/* pointer to vpu ipc device */
	int (*callback)(u32 sw_device_id, u32 event);
};

/* Events that can be notified via callback, when registered. */
enum xlink_vpu_event {
	XLINK_VPU_NOTIFY_DISCONNECT = 0,
	XLINK_VPU_NOTIFY_CONNECT,
	XLINK_VPU_NOTIFY_MSS_WDT,
	XLINK_VPU_NOTIFY_NCE_WDT,
	NUM_EVENT_TYPE
};

#define VPU_ID 0
#define VPU_DEVICE_NAME "vpu-0"
#define VPU_SW_DEVICE_ID 0
static struct xlink_ipc_dev *xlink_dev;

/*
 * Functions related to reserved-memory sub-devices.
 */

/*
 * xlink_reserved_memory_remove() - Removes the reserved memory sub-devices.
 *
 * @xlink_dev: [in] The xlink ipc device with reserved memory sub-devices.
 */
static void xlink_reserved_memory_remove(struct xlink_ipc_dev *xlink_dev)
{
	device_unregister(xlink_dev->local_xlink_mem.dev);
	device_unregister(xlink_dev->remote_xlink_mem.dev);
}

/*
 * xlink_reserved_mem_release() - Reserved memory release callback function.
 *
 * @dev: [in] The reserved memory sub-device.
 */
static void xlink_reserved_mem_release(struct device *dev)
{
	of_reserved_mem_device_release(dev);
}

/*
 * get_xlink_reserved_mem_size() - Gets the size of the reserved memory region.
 *
 * @dev: [in] The device the reserved memory region is allocated to.
 * @idx: [in] The reserved memory region's index in the phandle table.
 *
 * Return: The reserved memory size, 0 on failure.
 */
static resource_size_t get_xlink_reserved_mem_size(struct device *dev, int idx)
{
	struct device_node *np;
	struct resource mem;
	int rc;

	np = of_parse_phandle(dev->of_node, "memory-region", idx);
	if (!np) {
		dev_err(dev, "Couldn't find memory-region %d\n", idx);
		return 0;
	}

	rc = of_address_to_resource(np, 0, &mem);
	if (rc) {
		dev_err(dev, "Couldn't map address to resource %d\n", idx);
		return 0;
	}
	return resource_size(&mem);
}

/*
 * init_xlink_reserved_mem_dev() - Initializes the reserved memory sub-devices.
 *
 * @dev:	[in] The parent device of the reserved memory sub-device.
 * @name:	[in] The name to assign to the memory region.
 * @idx:	[in] The reserved memory region index in the phandle table.
 *
 * Return: The initialized sub-device, NULL on failure.
 */
static struct device *init_xlink_reserved_mem_dev(struct device *dev,
						  const char *name, int idx)
{
	struct device *child;
	int rc;

	child = devm_kzalloc(dev, sizeof(*child), GFP_KERNEL);
	if (!child)
		return NULL;

	device_initialize(child);
	dev_set_name(child, "%s:%s", dev_name(dev), name);
	dev_err(dev, " dev_name %s, name %s\n", dev_name(dev), name);
	child->parent = dev;
	child->dma_mask = dev->dma_mask;
	child->coherent_dma_mask = dev->coherent_dma_mask;
	/* set up dma configuration using information from parent's dt node */
	rc = of_dma_configure(child, dev->of_node, true);
	if (rc)
		return NULL;
	child->release = xlink_reserved_mem_release;

	rc = device_add(child);
	if (rc)
		goto err;
	rc = of_reserved_mem_device_init_by_idx(child, dev->of_node, idx);
	if (rc) {
		dev_err(dev, "Couldn't get reserved memory with idx = %d, %d\n",
			idx, rc);
		device_del(child);
		goto err;
	}
	return child;

err:
	put_device(child);
	return NULL;
}

/*
 * xlink_reserved_memory_init() - Initialize reserved memory for the device.
 *
 * @xlink_dev:	[in] The xlink ipc device the reserved memory is allocated to.
 *
 * Return: 0 on success, negative error code otherwise.
 */
static int xlink_reserved_memory_init(struct xlink_ipc_dev *xlink_dev)
{
	struct device *dev = &xlink_dev->pdev->dev;
	struct xlink_buf_mem *lxm = &xlink_dev->local_xlink_mem;
	struct xlink_buf_mem *rxm = &xlink_dev->remote_xlink_mem;

	lxm->dev = init_xlink_reserved_mem_dev(dev, "xlink_local_reserved",
					       LOCAL_XLINK_IPC_BUFFER_IDX);
	if (!lxm->dev)
		return -ENOMEM;

	lxm->size = get_xlink_reserved_mem_size(dev, LOCAL_XLINK_IPC_BUFFER_IDX);

	rxm->dev = init_xlink_reserved_mem_dev(dev, "xlink_remote_reserved",
					       REMOTE_XLINK_IPC_BUFFER_IDX);
	if (!rxm->dev) {
		device_unregister(xlink_dev->local_xlink_mem.dev);
		return -ENOMEM;
	}

	rxm->size = get_xlink_reserved_mem_size(dev, REMOTE_XLINK_IPC_BUFFER_IDX);

	return 0;
}

/*
 * xlink_reserved_memory_alloc() - Allocate reserved memory for the device.
 *
 * @xlink_dev:	[in] The xlink ipc device.
 *
 * Return: 0 on success, negative error code otherwise.
 */
static int xlink_reserved_memory_alloc(struct xlink_ipc_dev *xlink_dev)
{
	struct xlink_buf_mem *lxm = &xlink_dev->local_xlink_mem;
	struct xlink_buf_mem *rxm = &xlink_dev->remote_xlink_mem;

	lxm->vaddr = dmam_alloc_coherent(xlink_dev->local_xlink_mem.dev,
					 xlink_dev->local_xlink_mem.size,
					 &xlink_dev->local_xlink_mem.dma_handle,
					 GFP_KERNEL);
	if (!lxm->vaddr) {
		dev_err(&xlink_dev->pdev->dev,
			"Failed to allocate from local reserved memory.\n");
		return -ENOMEM;
	}
	rxm->vaddr = dmam_alloc_coherent(xlink_dev->remote_xlink_mem.dev,
					 xlink_dev->remote_xlink_mem.size,
					 &xlink_dev->remote_xlink_mem.dma_handle,
					 GFP_KERNEL);
	if (!rxm->vaddr) {
		dev_err(&xlink_dev->pdev->dev,
			"Failed to allocate from remote reserved memory.\n");
		return -ENOMEM;
	}

	return 0;
}

/*
 * init_xlink_buf_pool() - Initialize the device's tx buffer pool.
 *
 * @xlink_dev:	[in] The xlink ipc device.
 *
 * Return: 0 on success.
 */
static int init_xlink_buf_pool(struct xlink_ipc_dev *xlink_dev)
{
	struct xlink_buf_mem *mem = &xlink_dev->local_xlink_mem;
	struct xlink_buf_pool *xbufpool = &xlink_dev->xlink_buf_pool;

	memset(mem->vaddr, 0, mem->size);
	xbufpool->buf = mem->vaddr;
	xbufpool->buf_cnt = mem->size / XLINK_MAX_BUF_SIZE;
	xbufpool->idx = 0;
	dev_info(&xlink_dev->pdev->dev, "xlink Buffer Pool size: %zX\n",
		 xbufpool->buf_cnt);
	spin_lock_init(&xbufpool->lock);

	return 0;
}

/*
 * xlink_phys_to_virt() - Convert an xlink physical addresses to a virtual one.
 *
 * @xlink_mem:	[in] The memory region where the physical address is located.
 * @paddr:		[in] The physical address to convert to a virtual one.
 *
 * Return:		The corresponding virtual address, or NULL if the
 *				physical address is not in the expected memory
 *				range.
 */
static void *xlink_phys_to_virt(const struct xlink_buf_mem *xlink_mem,
				u32 paddr)
{
	if (unlikely(paddr < xlink_mem->dma_handle) ||
	    paddr >= (xlink_mem->dma_handle + xlink_mem->size))
		return NULL;

	return xlink_mem->vaddr + (paddr - xlink_mem->dma_handle);
}

/*
 * xlink_virt_to_phys() - Convert an xlink virtual addresses to a physical one.
 *
 * @xlink_mem:	[in]  The memory region where the physical address is located.
 * @vaddr:		[in]  The virtual address to convert to a physical one.
 * @paddr:		[out] Where to store the computed physical address.
 *
 * Return: 0 on success, negative error code otherwise.
 */
static int xlink_virt_to_phys(struct xlink_buf_mem *xlink_mem, void *vaddr,
			      u32 *paddr)
{
	if (unlikely((xlink_mem->dma_handle + xlink_mem->size) > 0xFFFFFFFF))
		return -EINVAL;
	if (unlikely(vaddr < xlink_mem->vaddr ||
		     vaddr >= (xlink_mem->vaddr + xlink_mem->size)))
		return -EINVAL;
	*paddr = xlink_mem->dma_handle + (vaddr - xlink_mem->vaddr);

	return 0;
}

/*
 * get_next_xlink_buf() - Get next xlink buffer from an xlink device's pool.
 *
 * @xlink_dev:	[in]  The xlink ipc device to get a buffer from.
 * @buf:		[out] Where to store the reference to the next buffer.
 * @size:		[in]  The size of the buffer to get.
 *
 * Return: 0 on success, negative error code otherwise.
 */
static int get_next_xlink_buf(struct xlink_ipc_dev *xlink_dev, void **buf,
			      int size)
{
	struct xlink_buf_pool *pool;
	unsigned long flags;

	if (!xlink_dev)
		return -ENODEV;

	if (size > XLINK_MAX_BUF_SIZE)
		return -EINVAL;

	pool = &xlink_dev->xlink_buf_pool;

	spin_lock_irqsave(&pool->lock, flags);
	if (pool->idx == pool->buf_cnt) {
		/* reached end of buffers - wrap around */
		pool->idx = 0;
	}
	*buf = pool->buf + (pool->idx * XLINK_MAX_BUF_SIZE);
	pool->idx++;
	spin_unlock_irqrestore(&pool->lock, flags);
	return 0;
}

/*
 * Functions related to the vpu ipc device reference.
 */

/*
 * vpu_ipc_device_put() - Release the vpu ipc device held by the xlink device.
 *
 * @xlink_dev:	[in] The xlink ipc device.
 */
static void vpu_ipc_device_put(struct xlink_ipc_dev *xlink_dev)
{
	put_device(xlink_dev->vpu_dev);
}

/*
 * vpu_ipc_device_get() - Get the vpu ipc device reference for the xlink device.
 *
 * @xlink_dev:	[in] The xlink ipc device.
 *
 * Return: 0 on success, negative error code otherwise.
 */
static int vpu_ipc_device_get(struct xlink_ipc_dev *xlink_dev)
{
	struct device *dev = &xlink_dev->pdev->dev;
	struct platform_device *pdev;
	struct device_node *np;

	np = of_parse_phandle(dev->of_node, "intel,keembay-vpu-ipc", 0);
	if (!np)
		return -ENODEV;

	pdev = of_find_device_by_node(np);
	if (!pdev) {
		dev_info(dev, "IPC device not probed\n");
		of_node_put(np);
		return -EPROBE_DEFER;
	}

	xlink_dev->vpu_dev = get_device(&pdev->dev);
	of_node_put(np);

	dev_info(dev, "Using IPC device: %s\n", dev_name(xlink_dev->vpu_dev));
	return 0;
}

/*
 * xlink platform api - ipc interface functions
 */

/*
 * xlink_ipc_connect() - platform connect interface.
 *
 * @sw_device_id:	[in]  The sw device id of the device to connect to.
 *
 * Return: 0 on success, negative error code otherwise.
 */
int xlink_ipc_connect(u32 sw_device_id)
{
	if (!xlink_dev)
		return -ENODEV;

	return 0;
}
EXPORT_SYMBOL_GPL(xlink_ipc_connect);

/*
 * xlink_ipc_write() - platform write interface.
 *
 * @sw_device_id:	[in]     The sw device id of the device to write to.
 * @data:			[in]     The data buffer to write.
 * @size:			[in-out] The amount of data to write/written.
 * @timeout:		[in]     The time (in ms) to wait before timing out.
 * @context:		[in]     The ipc operation context.
 *
 * Return: 0 on success, negative error code otherwise.
 */
int xlink_ipc_write(u32 sw_device_id, void *data, size_t * const size,
		    u32 timeout, void *context)
{
	struct xlink_ipc_context *ctx = context;
	void *vaddr = NULL;
	u32 paddr;
	int rc;

	if (!ctx)
		return -EINVAL;

	if (!xlink_dev)
		return -ENODEV;

	if (ctx->is_volatile) {
		rc = get_next_xlink_buf(xlink_dev, &vaddr, XLINK_MAX_BUF_SIZE);
		if (rc)
			return rc;
		memcpy(vaddr, data, *size);
		rc = xlink_virt_to_phys(&xlink_dev->local_xlink_mem, vaddr,
					&paddr);
		if (rc)
			return rc;
	} else {
		paddr = *(u32 *)data;
	}
	rc = intel_keembay_vpu_ipc_send(xlink_dev->vpu_dev,
					KMB_VPU_IPC_NODE_LEON_MSS, ctx->chan,
					paddr, *size);

	return rc;
}
EXPORT_SYMBOL_GPL(xlink_ipc_write);

/*
 * xlink_ipc_read() - platform read interface.
 *
 * @sw_device_id:	[in]     The sw device id of the device to read from.
 * @data:			[out]    The data buffer to read into.
 * @size:			[in-out] The amount of data to read/was read.
 * @timeout:		[in]     The time (in ms) to wait before timing out.
 * @context:		[in]     The ipc operation context.
 *
 * Return: 0 on success, negative error code otherwise.
 */
int xlink_ipc_read(u32 sw_device_id, void *data, size_t * const size,
		   u32 timeout, void *context)
{
	struct xlink_ipc_context *ctx = context;
	u32 addr = 0;
	void *vaddr;
	int rc;

	if (!ctx)
		return -EINVAL;

	if (!xlink_dev)
		return -ENODEV;

	rc = intel_keembay_vpu_ipc_recv(xlink_dev->vpu_dev,
					KMB_VPU_IPC_NODE_LEON_MSS, ctx->chan,
					&addr, size, timeout);

	if (ctx->is_volatile) {
		vaddr = xlink_phys_to_virt(&xlink_dev->remote_xlink_mem, addr);
		if (vaddr)
			memcpy(data, vaddr, *size);
		else
			return -ENXIO;
	} else {
		*(u32 *)data = addr;
	}
	return rc;
}
EXPORT_SYMBOL_GPL(xlink_ipc_read);

/*
 * xlink_ipc_get_device_list() - platform get device list interface.
 *
 * @sw_device_id_list:	[out]  The list of devices found.
 * @num_devices:		[out]  The number of devices found.
 *
 * Return: 0 on success, negative error code otherwise.
 */
int xlink_ipc_get_device_list(u32 *sw_device_id_list, u32 *num_devices)
{
	int i = 0;

	if (!sw_device_id_list || !num_devices)
		return -EINVAL;

	if (xlink_dev) {
		*sw_device_id_list = xlink_dev->sw_device_id;
		i++;
	}

	*num_devices = i;
	return 0;
}
EXPORT_SYMBOL_GPL(xlink_ipc_get_device_list);

/*
 * xlink_ipc_get_device_name() - platform get device name interface.
 *
 * @sw_device_id:	[in]  The sw device id of the device to get name of.
 * @device_name:	[out] The name of the xlink ipc device.
 * @name_size:		[in]  The maximum size of the name.
 *
 * Return: 0 on success, negative error code otherwise.
 */
int xlink_ipc_get_device_name(u32 sw_device_id, char *device_name,
			      size_t name_size)
{
	size_t size;

	if (!device_name)
		return -EINVAL;

	if (!xlink_dev)
		return -ENODEV;

	size = (name_size > XLINK_IPC_MAX_DEVICE_NAME_SIZE)
			? XLINK_IPC_MAX_DEVICE_NAME_SIZE
			: name_size;
	strncpy(device_name, xlink_dev->device_name, size);
	return 0;
}
EXPORT_SYMBOL_GPL(xlink_ipc_get_device_name);

/*
 * xlink_ipc_get_device_status() - platform get device status interface.
 *
 * @sw_device_id:	[in]  The sw device id of the device to get status of.
 * @device_status:	[out] The device status.
 *
 * Return: 0 on success, negative error code otherwise.
 */
int xlink_ipc_get_device_status(u32 sw_device_id, u32 *device_status)
{
	if (!device_status)
		return -EINVAL;

	if (!xlink_dev)
		return -ENODEV;

	*device_status = intel_keembay_vpu_status(xlink_dev->vpu_dev);
	return 0;
}
EXPORT_SYMBOL_GPL(xlink_ipc_get_device_status);

static void kernel_callback(struct device *dev, enum intel_keembay_vpu_event event)
{
	if ((enum xlink_vpu_event)event >= NUM_EVENT_TYPE)
		return;

	if (xlink_dev) {
		if (xlink_dev->callback)
			xlink_dev->callback(xlink_dev->sw_device_id, event);
	}
}

/*
 * xlink_ipc_register_for_events() - platform register for events
 *
 * @sw_device_id:	[in]	The sw device id of the device to get status of.
 * @callback:		[in]	Callback function for events
 *
 * Return: 0 on success, negative error code otherwise.
 */
int xlink_ipc_register_for_events(u32 sw_device_id,
				  int (*callback)(u32 sw_device_id, enum xlink_vpu_event event))
{
	int rc;

	if (!xlink_dev)
		return -ENODEV;
	xlink_dev->callback = callback;
	rc = intel_keembay_vpu_register_for_events(xlink_dev->vpu_dev, kernel_callback);
	return rc;
}
EXPORT_SYMBOL_GPL(xlink_ipc_register_for_events);
/*
 * xlink_ipc_unregister_for_events() - platform register for events
 *
 * @sw_device_id:	[in]	The sw device id of the device to get status of.
 *
 * Return: 0 on success, negative error code otherwise.
 */
int xlink_ipc_unregister_for_events(u32 sw_device_id)
{
	int rc;

	if (!xlink_dev)
		return -ENODEV;
	rc = intel_keembay_vpu_unregister_for_events(xlink_dev->vpu_dev);
	return rc;
}
EXPORT_SYMBOL_GPL(xlink_ipc_unregister_for_events);

/*
 * xlink_ipc_boot_device() - platform boot device interface.
 *
 * @sw_device_id:	[in] The sw device id of the device to boot.
 * @binary_name:	[in] The file name of the firmware binary to boot.
 *
 * Return: 0 on success, negative error code otherwise.
 */
int xlink_ipc_boot_device(u32 sw_device_id, const char *binary_name)
{
	enum intel_keembay_vpu_state state;
	int rc;

	if (!binary_name)
		return -EINVAL;

	if (!xlink_dev)
		return -ENODEV;

	pr_info("\nStart VPU 0x%x - %s\n", sw_device_id, binary_name);
	rc = intel_keembay_vpu_startup(xlink_dev->vpu_dev, binary_name);
	if (rc) {
		pr_err("Failed to start VPU: %d\n", rc);
		return -EBUSY;
	}
	pr_info("Successfully started VPU!\n");

	/* Wait for VPU to be READY */
	rc = intel_keembay_vpu_wait_for_ready(xlink_dev->vpu_dev,
					      XLINK_VPU_WAIT_FOR_READY_MS);
	if (rc) {
		pr_err("Tried to start VPU but never got READY.\n");
		return -EBUSY;
	}
	pr_info("Successfully synchronised state with VPU!\n");

	/* Check state */
	state = intel_keembay_vpu_status(xlink_dev->vpu_dev);
	if (state != KEEMBAY_VPU_READY) {
		pr_err("VPU was not ready, it was %d\n", state);
		return -EBUSY;
	}
	pr_info("VPU was ready.\n");
	return 0;
}
EXPORT_SYMBOL_GPL(xlink_ipc_boot_device);

/*
 * xlink_ipc_reset_device() - platform reset device interface.
 *
 * @sw_device_id:	[in] The sw device id of the device to reset.
 *
 * Return: 0 on success, negative error code otherwise.
 */
int xlink_ipc_reset_device(u32 sw_device_id)
{
	enum intel_keembay_vpu_state state;
	int rc;

	if (!xlink_dev)
		return -ENODEV;

	/* stop the vpu */
	rc = intel_keembay_vpu_stop(xlink_dev->vpu_dev);
	if (rc) {
		pr_err("Failed to stop VPU: %d\n", rc);
		return -EBUSY;
	}
	pr_info("Successfully stopped VPU!\n");

	/* check state */
	state = intel_keembay_vpu_status(xlink_dev->vpu_dev);
	if (state != KEEMBAY_VPU_OFF) {
		pr_err("VPU was not OFF after stop request, it was %d\n",
		       state);
		return -EBUSY;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(xlink_ipc_reset_device);

/*
 * xlink_ipc_open_channel() - platform open channel interface.
 *
 * @sw_device_id:	[in] The sw device id of the device to open channel to.
 * @channel:		[in] The channel id to open.
 *
 * Return: 0 on success, negative error code otherwise.
 */
int xlink_ipc_open_channel(u32 sw_device_id, u32 channel)
{
	int rc;

	if (!xlink_dev)
		return -ENODEV;

	rc = intel_keembay_vpu_ipc_open_channel(xlink_dev->vpu_dev,
						KMB_VPU_IPC_NODE_LEON_MSS, channel);
	return rc;
}
EXPORT_SYMBOL_GPL(xlink_ipc_open_channel);

/*
 * xlink_ipc_close_channel() - platform close channel interface.
 *
 * @sw_device_id:	[in] The sw device id of the device to close channel to.
 * @channel:		[in] The channel id to close.
 *
 * Return: 0 on success, negative error code otherwise.
 */
int xlink_ipc_close_channel(u32 sw_device_id, u32 channel)
{
	int rc;

	if (!xlink_dev)
		return -ENODEV;

	rc = intel_keembay_vpu_ipc_close_channel(xlink_dev->vpu_dev,
						 KMB_VPU_IPC_NODE_LEON_MSS, channel);
	return rc;
}
EXPORT_SYMBOL_GPL(xlink_ipc_close_channel);

enum control_channel_commands {
	GET_DEVICE_PWR_MODE_REQ,
	GET_DEVICE_PWR_MODE_RESP,
	SET_DEVICE_PWR_MODE,
	SET_DEVICE_PWR_MODE_RESP
};

#define CONTROL_CHAN_TIMEOUT 5000

struct control_channel_cmd {
	u32 command;
	u8 data[128];
};

int xlink_ipc_get_device_mode(u32 sw_device_id, u32 *power_mode)
{
	int rc = 0, sc = 0;
	struct xlink_ipc_context ipc = {0};
	struct control_channel_cmd *cmd;
	u8 databuf[XLINK_MAX_BUF_SIZE];
	size_t size;

	rc = xlink_ipc_open_channel(sw_device_id, IP_CONTROL_CHANNEL);
	if (rc == 0 || rc == -EEXIST) {
		mutex_lock(&get_device_mode_lock);
		ipc.chan = IP_CONTROL_CHANNEL;
		ipc.is_volatile = 1;
		cmd = (struct control_channel_cmd *)databuf;
		cmd->command = GET_DEVICE_PWR_MODE_REQ;
		size = 4;
		rc = xlink_ipc_write(sw_device_id, databuf, &size, 0, &ipc);
		if (rc) {
			pr_info("%s:Error IPC write %d\n", __func__, rc);
			goto getdev_error;
		}
		memset(databuf, 0, sizeof(databuf));
		rc = xlink_ipc_read(sw_device_id, databuf, &size, CONTROL_CHAN_TIMEOUT, &ipc);
		if (rc || size > XLINK_MAX_BUF_SIZE) {
			pr_info("%s:Error IPC read rc %d size %d\n", __func__,
				rc, (int)size);
			rc = EINVAL;
			goto getdev_error;
		} else {
			if (cmd->command == GET_DEVICE_PWR_MODE_RESP) {
				*power_mode = cmd->data[1];
				rc = (cmd->data[0]) ? EINVAL : 0;
			} else {
				rc = EINVAL;
				pr_info("%s:Error command not expected cmd=%d\n",
					__func__, cmd->command);
			}
		}
getdev_error:
		sc = xlink_ipc_close_channel(sw_device_id, IP_CONTROL_CHANNEL);
		if (sc) {
			pr_info("%s:Error closing control channel %d\n",
				__func__, IP_CONTROL_CHANNEL);
			rc = EINVAL;
		}
		mutex_unlock(&get_device_mode_lock);
	} else
		pr_info("%s:Error opening control channel %d\n",
			__func__, IP_CONTROL_CHANNEL);
	if (rc)
		rc = -EINVAL;
	else
		rc = 0;
	return rc;

}
EXPORT_SYMBOL(xlink_ipc_get_device_mode);

int xlink_ipc_set_device_mode(u32 sw_device_id, u32 power_mode)
{
	int rc = 0, sc = 0;
	struct xlink_ipc_context ipc = {0};
	struct control_channel_cmd *cmd;
	u8 databuf[XLINK_MAX_BUF_SIZE];
	size_t size;

	rc = xlink_ipc_open_channel(sw_device_id, IP_CONTROL_CHANNEL);
	if (rc == 0 || rc == -EEXIST) {
		mutex_lock(&set_device_mode_lock);
		ipc.chan = IP_CONTROL_CHANNEL;
		ipc.is_volatile = 1;
		cmd = (struct control_channel_cmd *)databuf;
		cmd->command = SET_DEVICE_PWR_MODE;
		cmd->data[0] = power_mode;
		size = 5;
		rc = xlink_ipc_write(sw_device_id, databuf, &size, 0, &ipc);
		if (rc) {
			pr_info("%s:Error IPC write %d\n", __func__, rc);
			goto setdev_error;
		}
		memset(databuf, 0, sizeof(databuf));
		// wait for response
		rc = xlink_ipc_read(sw_device_id, databuf, &size, CONTROL_CHAN_TIMEOUT, &ipc);
		if (rc || size > XLINK_MAX_BUF_SIZE) {
			pr_info("%s:Error IPC read %d size %d\n",
				__func__, rc, (int)size);
			rc = EINVAL;
			goto setdev_error;
		} else {
			if (cmd->command == SET_DEVICE_PWR_MODE_RESP) {
				rc = (cmd->data[0]) ? EINVAL : 0;
			} else {
				pr_info("%s:Error command not expected cmd=%d\n",
					__func__, cmd->command);
				rc = EINVAL;
			}
		}
setdev_error:
		sc = xlink_ipc_close_channel(sw_device_id, IP_CONTROL_CHANNEL);
		if (sc) {
			pr_info("%s:Error closing control channel %d\n",
				__func__, IP_CONTROL_CHANNEL);
			rc = EINVAL;
		}
		mutex_unlock(&set_device_mode_lock);
	} else
		pr_info("%s:Error opening control channel %d rc = %d\n",
			__func__, IP_CONTROL_CHANNEL, rc);
	if (rc)
		rc = EINVAL;
	else
		rc = 0;
	return rc;
}
EXPORT_SYMBOL(xlink_ipc_set_device_mode);

/*
 * xlink ipc driver functions
 */

static int keembay_xlink_ipc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int rc;

	/* allocate device data structure */
	xlink_dev = kzalloc(sizeof(*xlink_dev), GFP_KERNEL);
	if (!xlink_dev)
		return -ENOMEM;

	xlink_dev->pdev = pdev;
	dev_info(dev, "Keem Bay xlink IPC driver probed.\n");

	/* grab reserved memory regions and assign to child devices */
	rc = xlink_reserved_memory_init(xlink_dev);
	if (rc < 0) {
		dev_err(&pdev->dev,
			"Failed to set up reserved memory regions.\n");
		goto r_cleanup;
	}

	/* allocate memory from the reserved memory regions */
	rc = xlink_reserved_memory_alloc(xlink_dev);
	if (rc < 0) {
		dev_err(&pdev->dev,
			"Failed to allocate reserved memory regions.\n");
		goto r_cleanup;
	}

	/* init the xlink buffer pool used for rx/tx */
	init_xlink_buf_pool(xlink_dev);

	/* get reference to vpu ipc device */
	rc = vpu_ipc_device_get(xlink_dev);
	if (rc)
		goto r_cleanup;

	/* get device id */
	rc = of_property_read_u32(dev->of_node, "intel,keembay-vpu-ipc-id",
				  &xlink_dev->vpu_id);
	if (rc) {
		dev_err(dev, "Cannot get VPU ID from DT.\n");
		goto r_cleanup;
	}

	/* assign a sw device id */
	xlink_dev->sw_device_id = GET_SW_DEVICE_ID_FROM_VPU_ID
			(xlink_dev->vpu_id);

	/* assign a device name */
	rc = of_property_read_string(dev->of_node, "intel,keembay-vpu-ipc-name",
				     &xlink_dev->device_name);
	if (rc) {
		/* only warn for now; we will enforce this in the future */
		dev_warn(dev, "VPU name not defined in DT, using %s as default.\n",
			 VPU_DEVICE_NAME);
		dev_warn(dev, "WARNING: additional VPU devices may fail probing.\n");
		xlink_dev->device_name = VPU_DEVICE_NAME;
	}

	/* get platform data reference */
	platform_set_drvdata(pdev, xlink_dev);

	dev_info(dev, "Device id=%u sw_device_id=0x%x name=%s probe complete.\n",
		 xlink_dev->vpu_id, xlink_dev->sw_device_id,
			xlink_dev->device_name);
	mutex_init(&get_device_mode_lock);
	mutex_init(&set_device_mode_lock);

	return 0;

r_cleanup:
	xlink_reserved_memory_remove(xlink_dev);
	return rc;
}

static int keembay_xlink_ipc_remove(struct platform_device *pdev)
{
	struct xlink_ipc_dev *xlink_dev = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	/*
	 * no need to de-alloc xlink mem (local_xlink_mem and remote_xlink_mem)
	 * since it was allocated with dmam_alloc
	 */
	xlink_reserved_memory_remove(xlink_dev);

	/* release vpu ipc device */
	vpu_ipc_device_put(xlink_dev);

	dev_info(dev, "Keem Bay xlink IPC driver removed.\n");
	return 0;
}

static const struct of_device_id keembay_xlink_ipc_of_match[] = {
	{
		.compatible = "intel,keembay-xlink-ipc",
	},
	{}
};

static struct platform_driver keembay_xlink_ipc_driver = {
	.driver = {
			.name = "keembay-xlink-ipc",
			.of_match_table = keembay_xlink_ipc_of_match,
		},
	.probe = keembay_xlink_ipc_probe,
	.remove = keembay_xlink_ipc_remove,
};
module_platform_driver(keembay_xlink_ipc_driver);

MODULE_DESCRIPTION("Keem Bay xlink IPC Driver");
MODULE_AUTHOR("Ryan Carnaghi <ryan.r.carnaghi@intel.com>");
MODULE_LICENSE("GPL v2");
