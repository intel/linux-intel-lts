// SPDX-License-Identifier: GPL-2.0-only
/*
 * xlink Core Driver.
 *
 * Copyright (C) 2018-2019 Intel Corporation
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/mod_devicetable.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/kref.h>

#ifdef CONFIG_XLINK_LOCAL_HOST
#include <linux/xlink-ipc.h>
#endif

#include "xlink-core.h"
#include "xlink-defs.h"
#include "xlink-dispatcher.h"
#include "xlink-ioctl.h"
#include "xlink-multiplexer.h"
#include "xlink-platform.h"

// xlink version number
#define XLINK_VERSION_MAJOR		0
#define XLINK_VERSION_MINOR		1
#define XLINK_VERSION_REVISION		2
#define XLINK_VERSION_SUB_REV		"a"

// timeout in milliseconds used to wait for the reay message from the VPU
#ifdef CONFIG_XLINK_PSS
#define XLINK_VPU_WAIT_FOR_READY (3000000)
#else
#define XLINK_VPU_WAIT_FOR_READY (3000)
#endif

// device, class, and driver names
#define DEVICE_NAME	"xlnk"
#define CLASS_NAME	"xlkcore"
#define DRV_NAME	"xlink-driver"

// used to determine if an API was called from user or kernel space
#define CHANNEL_SET_USER_BIT(chan)	((chan) |= (1 << 15))
#define CHANNEL_USER_BIT_IS_SET(chan)	((chan) & (1 << 15))
#define CHANNEL_CLEAR_USER_BIT(chan)	((chan) &= ~(1 << 15))

static dev_t xdev;
static struct class *dev_class;

static int xlink_open(struct inode *inode, struct file *filp)
{
	// create context data for recording & leakage handling
	struct session_context *ctx;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (IS_ERR_OR_NULL(ctx))
		return PTR_ERR(ctx);
	INIT_LIST_HEAD(&ctx->list);
	mutex_init(&ctx->lock);
	filp->private_data = ctx;
	return 0;
}

static int xlink_release_chans(int id, void *ptr)
{
	int rc = 0;
	struct xlink_handle *pdevH = (struct xlink_handle *)ptr;

	if (pdevH) {
		rc = xlink_close_channel(pdevH, id);
		pr_info("[xlink-core]: xlink_close_channel(%d) rc=%d\n", id, rc);
	}
	return 0;
}

static int xlink_release(struct inode *inode, struct file *filp)
{
	struct session_context *ctx = (struct session_context *)filp->private_data;
	struct session_context *sess = NULL, *tmp = NULL;

	mutex_lock(&ctx->lock);
	list_for_each_entry_safe(sess, tmp, &ctx->list, list) {
		xlink_release_chans(sess->chan, sess->ptr);
		list_del(&sess->list);
		kfree(sess);
	}
	mutex_unlock(&ctx->lock);
	mutex_destroy(&ctx->lock);
	kfree(ctx);
	return 0;
}

static long xlink_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

static struct mutex dev_event_lock;

static const struct file_operations fops = {
		.owner		= THIS_MODULE,
		.unlocked_ioctl = xlink_ioctl,
		.open		= xlink_open,
		.release	= xlink_release,
};

//struct xlink_link {
//	u32 id;
//	struct xlink_handle handle;
//	struct kref refcount;
//};

struct xlink_attr {
	unsigned long value;
	u32 sw_dev_id;
};

struct keembay_xlink_dev {
	struct device *dev;
	struct xlink_link links[XLINK_MAX_CONNECTIONS];
	struct cdev cdev;
	u32 nmb_connected_links;
	struct mutex lock;  // protect access to xlink_dev
	struct xlink_attr eventx[4];
};

struct event_info {
	struct list_head list;
	u32 sw_device_id;
	u32 event_type;
	u32 user_flag;
	xlink_device_event_cb event_notif_fn;
};

/* sysfs attribute functions */

static ssize_t eventx_show(struct device *dev, struct device_attribute *attr,
			   int index, char *buf)
{
	struct keembay_xlink_dev *xlink_dev = dev_get_drvdata(dev);
	struct xlink_attr *a = &xlink_dev->eventx[index];

	return sysfs_emit(buf, "0x%x 0x%lx\n", a->sw_dev_id, a->value);
}

static ssize_t event0_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return eventx_show(dev, attr, 0, buf);
}

static ssize_t event1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return eventx_show(dev, attr, 1, buf);
}

static ssize_t event2_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return eventx_show(dev, attr, 2, buf);
}

static ssize_t event3_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return eventx_show(dev, attr, 3, buf);
}

static DEVICE_ATTR_RO(event0);
static DEVICE_ATTR_RO(event1);
static DEVICE_ATTR_RO(event2);
static DEVICE_ATTR_RO(event3);
static struct attribute *xlink_sysfs_entries[] = {
	&dev_attr_event0.attr,
	&dev_attr_event1.attr,
	&dev_attr_event2.attr,
	&dev_attr_event3.attr,
	NULL,
};

static const struct attribute_group xlink_sysfs_group = {
	.attrs = xlink_sysfs_entries,
};

static struct event_info ev_info;

/*
 * global variable pointing to our xlink device.
 *
 * This is meant to be used only when platform_get_drvdata() cannot be used
 * because we lack a reference to our platform_device.
 */
static struct keembay_xlink_dev *xlink;

/*
 * get_next_link() - Searches the list of links to find the next available.
 *
 * Note: This function is only used in xlink_connect, where the xlink mutex is
 * already locked.
 *
 * Return: the next available link, or NULL if maximum connections reached.
 */
static struct xlink_link *get_next_link(void)
{
	struct xlink_link *link = NULL;
	int i;

	for (i = 0; i < XLINK_MAX_CONNECTIONS; i++) {
		if (xlink->links[i].handle.sw_device_id == XLINK_INVALID_SW_DEVICE_ID) {
			link = &xlink->links[i];
			break;
		}
	}
	return link;
}

/*
 * get_link_by_sw_device_id()
 *
 * Searches the list of links to find a link by sw device id.
 *
 * Return: the handle, or NULL if the handle is not found.
 */
struct xlink_link *get_link_by_sw_device_id(u32 sw_device_id)
{
	struct xlink_link *link = NULL;
	int i;

	mutex_lock(&xlink->lock);
	for (i = 0; i < XLINK_MAX_CONNECTIONS; i++) {
		if (xlink->links[i].handle.sw_device_id == sw_device_id) {
			link = &xlink->links[i];
			break;
		}
	}
	mutex_unlock(&xlink->lock);
	return link;
}

// For now , do nothing and leave for further consideration
static void release_after_kref_put(struct kref *ref) {}

/* Driver probing. */
static int kmb_xlink_init(void)
{
	struct device *dev;
	int rc, i;

	pr_info("Keem Bay xlink v%d.%d.%d:%s\n", XLINK_VERSION_MAJOR,
		XLINK_VERSION_MINOR, XLINK_VERSION_REVISION, XLINK_VERSION_SUB_REV);

	/*Allocating Major number*/
	if ((alloc_chrdev_region(&xdev, 0, 1, "xlinkdev")) < 0) {
		pr_info("Cannot allocate major number\n");
		return -1;
	}
	pr_info("Major = %d Minor = %d\n", MAJOR(xdev),
		MINOR(xdev));

	/*Creating struct class*/
	dev_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(dev_class)) {
		pr_info("Cannot create the struct class - Err %ld\n",
			PTR_ERR(dev_class));
		goto r_class;
	}

	/*Creating device*/
	dev = device_create(dev_class, NULL, xdev, NULL, DEVICE_NAME);
	if (IS_ERR(dev)) {
		dev_info(dev, "Cannot create the Device 1 - Err %ld\n",
			 PTR_ERR(dev));
		goto r_device;
	}
	dev_info(dev, "Device Driver Insert...Done!!!\n");

	xlink = devm_kzalloc(dev, sizeof(*xlink), GFP_KERNEL);
	if (!xlink)
		goto r_device;

	xlink->dev = dev;

	// initialize multiplexer
	rc = xlink_multiplexer_init(dev);
	if (rc != X_LINK_SUCCESS) {
		pr_err("Multiplexer initialization failed\n");
		goto r_multiplexer;
	}

	// initialize dispatcher
	rc = xlink_dispatcher_init(dev);
	if (rc != X_LINK_SUCCESS) {
		pr_err("Dispatcher initialization failed\n");
		goto r_dispatcher;
	}
	// initialize xlink data structure
	xlink->nmb_connected_links = 0;
	mutex_init(&xlink->lock);
	for (i = 0; i < XLINK_MAX_CONNECTIONS; i++) {
		xlink->links[i].id = i;
		xlink->links[i].handle.sw_device_id =
				XLINK_INVALID_SW_DEVICE_ID;
	}

	/*Creating cdev structure*/
	cdev_init(&xlink->cdev, &fops);

	/*Adding character device to the system*/
	if ((cdev_add(&xlink->cdev, xdev, 1)) < 0) {
		dev_info(dev, "Cannot add the device to the system\n");
		goto r_char;
	}
	INIT_LIST_HEAD(&ev_info.list);

	rc = devm_device_add_group(dev, &xlink_sysfs_group);
	if (rc) {
		dev_err(dev, "failed to create sysfs entries: %d\n", rc);
		return rc;
	}
	mutex_init(&dev_event_lock);
	return 0;

r_char:
	xlink_dispatcher_destroy();
r_dispatcher:
	xlink_multiplexer_destroy();
r_multiplexer:
	device_destroy(dev_class, xdev);
r_device:
	class_destroy(dev_class);
r_class:
	unregister_chrdev_region(xdev, 1);
	return -1;
}
module_init(kmb_xlink_init);

/* Driver removal. */
static int kmb_xlink_remove(void)
{
	int rc;

	mutex_lock(&xlink->lock);
	// destroy multiplexer
	rc = xlink_multiplexer_destroy();
	if (rc != X_LINK_SUCCESS)
		pr_err("Multiplexer destroy failed\n");
	rc = xlink_dispatcher_destroy();
	if (rc != X_LINK_SUCCESS)
		pr_err("Dispatcher destroy failed\n");

	mutex_unlock(&xlink->lock);
	mutex_destroy(&xlink->lock);
	// unregister and destroy device
	unregister_chrdev_region(xdev, 1);
	device_destroy(dev_class, xdev);
	cdev_del(&xlink->cdev);
	class_destroy(dev_class);
	pr_info("XLink Driver removed\n");
	return 0;
}

/*
 * IOCTL function for User Space access to xlink kernel functions
 *
 */
static long xlink_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int rc;
	struct session_context *sess_ctx = (struct session_context *)file->private_data;

	switch (cmd) {
	case XL_CONNECT:
		rc = ioctl_connect(arg);
		break;
	case XL_OPEN_CHANNEL:
		rc = ioctl_open_channel(arg, sess_ctx);
		break;
	case XL_DATA_READY_CALLBACK:
		rc = ioctl_data_ready_callback(arg);
		break;
	case XL_DATA_CONSUMED_CALLBACK:
		rc = ioctl_data_consumed_callback(arg);
		break;
	case XL_READ_DATA:
		rc = ioctl_read_data(arg);
		break;
	case XL_READ_TO_BUFFER:
		rc = ioctl_read_to_buffer(arg);
		break;
	case XL_WRITE_DATA:
		rc = ioctl_write_data(arg);
		break;
	case XL_WRITE_VOLATILE:
		rc = ioctl_write_volatile_data(arg);
		break;
	case XL_WRITE_CONTROL_DATA:
		rc = ioctl_write_control_data(arg);
		break;
	case XL_RELEASE_DATA:
		rc = ioctl_release_data(arg);
		break;
	case XL_CLOSE_CHANNEL:
		rc = ioctl_close_channel(arg, sess_ctx);
		break;
	case XL_START_VPU:
		rc = ioctl_start_vpu(arg);
		break;
	case XL_STOP_VPU:
		rc = ioctl_stop_vpu();
		break;
	case XL_RESET_VPU:
		rc = ioctl_stop_vpu();
		break;
	case XL_DISCONNECT:
		rc = ioctl_disconnect(arg);
		break;
	case XL_GET_DEVICE_NAME:
		rc = ioctl_get_device_name(arg);
		break;
	case XL_GET_DEVICE_LIST:
		rc = ioctl_get_device_list(arg);
		break;
	case XL_GET_DEVICE_STATUS:
		rc = ioctl_get_device_status(arg);
		break;
	case XL_BOOT_DEVICE:
		rc = ioctl_boot_device(arg);
		break;
	case XL_RESET_DEVICE:
		rc = ioctl_reset_device(arg);
		break;
	case XL_GET_DEVICE_MODE:
		rc = ioctl_get_device_mode(arg);
		break;
	case XL_SET_DEVICE_MODE:
		rc = ioctl_set_device_mode(arg);
		break;
	case XL_REGISTER_DEV_EVENT:
		rc = ioctl_register_device_event(arg);
		break;
	case XL_UNREGISTER_DEV_EVENT:
		rc = ioctl_unregister_device_event(arg);
		break;
	}
	if (rc)
		return -EIO;
	else
		return 0;
}

/*
 * xlink Kernel API.
 */
enum xlink_error xlink_stop_vpu(void)
{
#ifdef CONFIG_XLINK_LOCAL_HOST
	int rc;

	rc = xlink_ipc_reset_device(0x0); // stop vpu slice 0
	if (rc)
		return X_LINK_ERROR;
#endif
	return X_LINK_SUCCESS;
}
EXPORT_SYMBOL_GPL(xlink_stop_vpu);
enum xlink_error xlink_start_vpu(char *filename)
{
#ifdef CONFIG_XLINK_LOCAL_HOST
	int rc;

	rc = xlink_ipc_boot_device(0x0, filename); // start vpu slice 0
	if (rc)
		return X_LINK_ERROR;
#endif
	return X_LINK_SUCCESS;
}
EXPORT_SYMBOL_GPL(xlink_start_vpu);

enum xlink_error xlink_initialize(void)
{
	return X_LINK_SUCCESS;
}
EXPORT_SYMBOL_GPL(xlink_initialize);

enum xlink_error xlink_connect(struct xlink_handle *handle)
{
	struct xlink_link *link;
	enum xlink_error rc;
	int interface;

	if (!xlink || !handle)
		return X_LINK_ERROR;

	link = get_link_by_sw_device_id(handle->sw_device_id);
	mutex_lock(&xlink->lock);
	if (!link) {
		link = get_next_link();
		if (!link) {
			pr_err("max connections reached %d\n",
			       XLINK_MAX_CONNECTIONS);
			mutex_unlock(&xlink->lock);
			return X_LINK_ERROR;
		}
		// platform connect
		interface = get_interface_from_sw_device_id(handle->sw_device_id);
		rc = xlink_platform_connect(interface, handle->sw_device_id);
		if (rc) {
			pr_err("platform connect failed %d\n", rc);
			mutex_unlock(&xlink->lock);
			return X_LINK_ERROR;
		}
		// set link handle reference and link id
		link->handle = *handle;
		xlink->nmb_connected_links++;
		kref_init(&link->refcount);
		if (interface != IPC_INTERFACE) {
			rc = xlink_dispatcher_start(link->id, &link->handle);
			if (rc) {
				pr_err("dispatcher start failed\n");
				goto r_cleanup;
			}
		}
		rc = xlink_multiplexer_connect(link->id);
		if (rc) {
			pr_err("multiplexer connect failed\n");
			goto r_cleanup;
		}
		pr_info("dev 0x%x connected - dev_type %d - nmb_connected_links %d\n",
			link->handle.sw_device_id,
			link->handle.dev_type,
			xlink->nmb_connected_links);
	} else {
		pr_info("dev 0x%x ALREADY connected - dev_type %d\n",
			link->handle.sw_device_id,
			link->handle.dev_type);
		kref_get(&link->refcount);
		*handle = link->handle;
	}
	mutex_unlock(&xlink->lock);
	return X_LINK_SUCCESS;

r_cleanup:
	link->handle.sw_device_id = XLINK_INVALID_SW_DEVICE_ID;
	mutex_unlock(&xlink->lock);
	return X_LINK_ERROR;
}
EXPORT_SYMBOL_GPL(xlink_connect);

enum xlink_error xlink_data_available_event(struct xlink_handle *handle,
					    u16 chan,
					    xlink_event data_available_event)
{
	struct xlink_event *event;
	struct xlink_link *link;
	enum xlink_error rc;
	int event_queued = 0;
	char origin = 'K';

	if (!xlink || !handle)
		return X_LINK_ERROR;

	if (CHANNEL_USER_BIT_IS_SET(chan))
		origin  = 'U';     // function called from user space
	CHANNEL_CLEAR_USER_BIT(chan);  // restore proper channel value

	link = get_link_by_sw_device_id(handle->sw_device_id);
	if (!link)
		return X_LINK_ERROR;
	event = xlink_create_event(link->id, XLINK_DATA_READY_CALLBACK_REQ,
				   &link->handle, chan, 0, 0);
	if (!event)
		return X_LINK_ERROR;
	event->data = data_available_event;
	event->callback_origin = origin;
	if (!data_available_event)
		event->calling_pid = NULL; // disable callbacks on this channel
	else
		event->calling_pid = current;
	rc = xlink_multiplexer_tx(event, &event_queued);
	if (!event_queued)
		xlink_destroy_event(event);
	return rc;
}
EXPORT_SYMBOL_GPL(xlink_data_available_event);
enum xlink_error xlink_data_consumed_event(struct xlink_handle *handle,
					   u16 chan,
					   xlink_event data_consumed_event)
{
	struct xlink_event *event;
	struct xlink_link *link;
	enum xlink_error rc;
	int event_queued = 0;
	char origin = 'K';

	if (!xlink || !handle)
		return X_LINK_ERROR;

	if (CHANNEL_USER_BIT_IS_SET(chan))
		origin  = 'U';     // function called from user space
	CHANNEL_CLEAR_USER_BIT(chan);  // restore proper channel value

	link = get_link_by_sw_device_id(handle->sw_device_id);
	if (!link)
		return X_LINK_ERROR;
	event = xlink_create_event(link->id, XLINK_DATA_CONSUMED_CALLBACK_REQ,
				   &link->handle, chan, 0, 0);
	if (!event)
		return X_LINK_ERROR;
	event->data = data_consumed_event;
	event->callback_origin = origin;
	if (!data_consumed_event)
		event->calling_pid = NULL; // disable callbacks on this channel
	else
		event->calling_pid = current;
	rc = xlink_multiplexer_tx(event, &event_queued);
	if (!event_queued)
		xlink_destroy_event(event);
	return rc;
}
EXPORT_SYMBOL_GPL(xlink_data_consumed_event);
enum xlink_error xlink_open_channel(struct xlink_handle *handle,
				    u16 chan, enum xlink_opmode mode,
				    u32 data_size, u32 timeout)
{
	struct xlink_event *event;
	struct xlink_link *link;
	int event_queued = 0;
	enum xlink_error rc;

	if (!xlink || !handle)
		return X_LINK_ERROR;

	link = get_link_by_sw_device_id(handle->sw_device_id);
	if (!link)
		return X_LINK_ERROR;

	event = xlink_create_event(link->id, XLINK_OPEN_CHANNEL_REQ,
				   &link->handle, chan, data_size, timeout);
	if (!event)
		return X_LINK_ERROR;

	event->data = (void *)mode;
	rc = xlink_multiplexer_tx(event, &event_queued);
	if (!event_queued)
		xlink_destroy_event(event);
	return rc;
}
EXPORT_SYMBOL_GPL(xlink_open_channel);

enum xlink_error xlink_close_channel(struct xlink_handle *handle,
				     u16 chan)
{
	struct xlink_event *event;
	struct xlink_link *link;
	enum xlink_error rc;
	int event_queued = 0;

	if (!xlink || !handle)
		return X_LINK_ERROR;

	link = get_link_by_sw_device_id(handle->sw_device_id);
	if (!link)
		return X_LINK_ERROR;

	event = xlink_create_event(link->id, XLINK_CLOSE_CHANNEL_REQ,
				   &link->handle, chan, 0, 0);
	if (!event)
		return X_LINK_ERROR;

	rc = xlink_multiplexer_tx(event, &event_queued);
	if (!event_queued)
		xlink_destroy_event(event);

	return rc;
}
EXPORT_SYMBOL_GPL(xlink_close_channel);

static enum xlink_error do_xlink_write_data(struct xlink_handle *handle,
					    u16 chan, u8 const *pmessage,
					    u32 size, u32 user_flag)
{
	struct xlink_event *event;
	struct xlink_link *link;
	enum xlink_error rc;
	int event_queued = 0;
	dma_addr_t paddr = 0;
	u32 addr;

	if (!xlink || !handle)
		return X_LINK_ERROR;

	if (size > XLINK_MAX_DATA_SIZE)
		return X_LINK_ERROR;

	link = get_link_by_sw_device_id(handle->sw_device_id);
	if (!link)
		return X_LINK_ERROR;

	event = xlink_create_event(link->id, XLINK_WRITE_REQ, &link->handle,
				   chan, size, 0);
	if (!event)
		return X_LINK_ERROR;
	event->user_data = user_flag;

	if (chan < XLINK_IPC_MAX_CHANNELS &&
	    event->interface == IPC_INTERFACE) {
		/* only passing message address across IPC interface */
		if (user_flag) {
			if (get_user(addr, (u32 __user *)pmessage)) {
				xlink_destroy_event(event);
				return X_LINK_ERROR;
			}
			event->data = &addr;
		} else {
			event->data = &pmessage;
		}
		rc = xlink_multiplexer_tx(event, &event_queued);
		xlink_destroy_event(event);
	} else {
		if (user_flag) {
			event->data = xlink_platform_allocate(xlink->dev, &paddr,
							      size,
							      XLINK_PACKET_ALIGNMENT,
							      XLINK_NORMAL_MEMORY);
			if (!event->data) {
				xlink_destroy_event(event);
				return X_LINK_ERROR;
			}
			if (copy_from_user(event->data, (void __user *)pmessage, size)) {
				xlink_platform_deallocate(xlink->dev,
							  event->data, paddr, size,
							  XLINK_PACKET_ALIGNMENT,
							  XLINK_NORMAL_MEMORY);
				xlink_destroy_event(event);
				return X_LINK_ERROR;
			}
			event->paddr = paddr;
		} else {
			event->data = (u8 *)pmessage;
			event->paddr = 0;
		}
		rc = xlink_multiplexer_tx(event, &event_queued);
		if (!event_queued) {
			if (user_flag) {
				xlink_platform_deallocate(xlink->dev,
							  event->data, paddr, size,
							  XLINK_PACKET_ALIGNMENT,
							  XLINK_NORMAL_MEMORY);
			}
			xlink_destroy_event(event);
		}
	}
	return rc;
}

enum xlink_error xlink_write_data(struct xlink_handle *handle,
				  u16 chan, u8 const *pmessage, u32 size)
{
	enum xlink_error rc = 0;

	rc = do_xlink_write_data(handle, chan, pmessage, size, 0);
	return rc;
}
EXPORT_SYMBOL_GPL(xlink_write_data);

enum xlink_error xlink_write_data_user(struct xlink_handle *handle,
				       u16 chan, u8 const *pmessage, u32 size)
{
	enum xlink_error rc = 0;

	rc = do_xlink_write_data(handle, chan, pmessage, size, 1);
	return rc;
}

enum xlink_error xlink_write_control_data(struct xlink_handle *handle,
					  u16 chan, u8 const *pmessage,
					  u32 size)
{
	struct xlink_event *event;
	struct xlink_link *link;
	int event_queued = 0;
	enum xlink_error rc;

	if (!xlink || !handle)
		return X_LINK_ERROR;

	if (chan < XLINK_IPC_MAX_CHANNELS) {
		if (size > XLINK_MAX_CONTROL_DATA_SIZE)
			return X_LINK_ERROR;
	}
	if (chan >= XLINK_IPC_MAX_CHANNELS) {
		if (size > XLINK_MAX_CONTROL_DATA_PCIE_SIZE)
			return X_LINK_ERROR;
	}

	link = get_link_by_sw_device_id(handle->sw_device_id);
	if (!link)
		return X_LINK_ERROR;
	event = xlink_create_event(link->id, XLINK_WRITE_CONTROL_REQ,
				   &link->handle, chan, size, 0);
	if (!event)
		return X_LINK_ERROR;
	memcpy(event->header.control_data, pmessage, size);
	rc = xlink_multiplexer_tx(event, &event_queued);
	if (!event_queued)
		xlink_destroy_event(event);
	return rc;
}
EXPORT_SYMBOL_GPL(xlink_write_control_data);

static enum xlink_error do_xlink_write_volatile(struct xlink_handle *handle,
						u16 chan, u8 const *message,
						u32 size, u32 user_flag)
{
	enum xlink_error rc = 0;
	struct xlink_link *link = NULL;
	struct xlink_event *event = NULL;
	int event_queued = 0;
	dma_addr_t paddr;
	int region = 0;

	if (!xlink || !handle)
		return X_LINK_ERROR;

	if (size > XLINK_MAX_BUF_SIZE)
		return X_LINK_ERROR;

	link = get_link_by_sw_device_id(handle->sw_device_id);
	if (!link)
		return X_LINK_ERROR;

	event = xlink_create_event(link->id, XLINK_WRITE_VOLATILE_REQ,
				   &link->handle, chan, size, 0);
	if (!event)
		return X_LINK_ERROR;

	region = XLINK_NORMAL_MEMORY;
	event->data = xlink_platform_allocate(xlink->dev, &paddr, size,
					      XLINK_PACKET_ALIGNMENT, region);
	if (!event->data) {
		xlink_destroy_event(event);
		return X_LINK_ERROR;
	}
	memcpy(event->data, message, size);
	event->user_data = user_flag;
	event->paddr = paddr;
	rc = xlink_multiplexer_tx(event, &event_queued);
	if (!event_queued) {
		xlink_platform_deallocate(xlink->dev, event->data, paddr, size,
					  XLINK_PACKET_ALIGNMENT, region);
		xlink_destroy_event(event);
	}
	return rc;
}

enum xlink_error xlink_write_volatile_user(struct xlink_handle *handle,
					   u16 chan, u8 const *message,
					   u32 size)
{
	enum xlink_error rc = 0;

	rc = do_xlink_write_volatile(handle, chan, message, size, 1);
	return rc;
}

enum xlink_error xlink_write_volatile(struct xlink_handle *handle,
				      u16 chan, u8 const *message, u32 size)
{
	enum xlink_error rc = 0;

	rc = do_xlink_write_volatile(handle, chan, message, size, 0);
	return rc;
}
EXPORT_SYMBOL_GPL(xlink_write_volatile);

enum xlink_error xlink_read_data(struct xlink_handle *handle,
				 u16 chan, u8 **pmessage, u32 *size)
{
	struct xlink_event *event;
	struct xlink_link *link;
	int event_queued = 0;
	enum xlink_error rc;

	if (!xlink || !handle)
		return X_LINK_ERROR;

	link = get_link_by_sw_device_id(handle->sw_device_id);
	if (!link)
		return X_LINK_ERROR;

	event = xlink_create_event(link->id, XLINK_READ_REQ, &link->handle,
				   chan, *size, 0);
	if (!event)
		return X_LINK_ERROR;

	event->pdata = (void **)pmessage;
	event->length = size;
	rc = xlink_multiplexer_tx(event, &event_queued);
	if (!event_queued)
		xlink_destroy_event(event);
	return rc;
}
EXPORT_SYMBOL_GPL(xlink_read_data);

enum xlink_error xlink_read_data_to_buffer(struct xlink_handle *handle,
					   u16 chan, u8 *const message, u32 *size)
{
	enum xlink_error rc = 0;
	struct xlink_link *link = NULL;
	struct xlink_event *event = NULL;
	int event_queued = 0;

	if (!xlink || !handle)
		return X_LINK_ERROR;

	link = get_link_by_sw_device_id(handle->sw_device_id);
	if (!link)
		return X_LINK_ERROR;

	event = xlink_create_event(link->id, XLINK_READ_TO_BUFFER_REQ,
				   &link->handle, chan, *size, 0);
	if (!event)
		return X_LINK_ERROR;

	event->data = message;
	event->length = size;
	rc = xlink_multiplexer_tx(event, &event_queued);
	if (!event_queued)
		xlink_destroy_event(event);
	return rc;
}
EXPORT_SYMBOL_GPL(xlink_read_data_to_buffer);

enum xlink_error xlink_release_data(struct xlink_handle *handle,
				    u16 chan, u8 * const data_addr)
{
	struct xlink_event *event;
	struct xlink_link *link;
	int event_queued = 0;
	enum xlink_error rc;

	if (!xlink || !handle)
		return X_LINK_ERROR;

	link = get_link_by_sw_device_id(handle->sw_device_id);
	if (!link)
		return X_LINK_ERROR;

	event = xlink_create_event(link->id, XLINK_RELEASE_REQ, &link->handle,
				   chan, 0, 0);
	if (!event)
		return X_LINK_ERROR;

	event->data = data_addr;
	rc = xlink_multiplexer_tx(event, &event_queued);
	if (!event_queued)
		xlink_destroy_event(event);
	return rc;
}
EXPORT_SYMBOL_GPL(xlink_release_data);

enum xlink_error xlink_disconnect(struct xlink_handle *handle)
{
	struct xlink_link *link;
	int interface;
	enum xlink_error rc = 0;

	if (!xlink || !handle)
		return X_LINK_ERROR;

	link = get_link_by_sw_device_id(handle->sw_device_id);
	if (!link)
		return X_LINK_ERROR;

	if (kref_put_mutex(&link->refcount, release_after_kref_put,
			   &xlink->lock)) {
		// stop dispatcher
		interface = get_interface_from_sw_device_id(link->handle.sw_device_id);
		if (interface != IPC_INTERFACE) {
			// stop dispatcher
			rc = xlink_dispatcher_stop(link->id);
			if (rc != X_LINK_SUCCESS) {
				pr_err("dispatcher stop failed\n");
				mutex_unlock(&xlink->lock);
				return X_LINK_ERROR;
			}
		}
		// deinitialize multiplexer connection
		rc = xlink_multiplexer_disconnect(link->id);
		if (rc) {
			pr_err("multiplexer disconnect failed\n");
			mutex_unlock(&xlink->lock);
			return X_LINK_ERROR;
		}
		// invalidate link handle reference
		link->handle.sw_device_id = XLINK_INVALID_SW_DEVICE_ID;
		xlink->nmb_connected_links--;
		mutex_unlock(&xlink->lock);
	}
	return rc;
}
EXPORT_SYMBOL_GPL(xlink_disconnect);

enum xlink_error xlink_get_device_list(u32 *sw_device_id_list,
				       u32 *num_devices)
{
	u32 interface_nmb_devices = 0;
	enum xlink_error rc;
	int i;

	if (!xlink)
		return X_LINK_ERROR;
	if (!sw_device_id_list || !num_devices)
		return X_LINK_ERROR;
	/* loop through each interface and combine the lists */
	for (i = 0; i < NMB_OF_INTERFACES; i++) {
		rc = xlink_platform_get_device_list(i, sw_device_id_list,
						    &interface_nmb_devices);
		if (!rc) {
			*num_devices += interface_nmb_devices;
			sw_device_id_list += interface_nmb_devices;
		}
		interface_nmb_devices = 0;
	}
	return X_LINK_SUCCESS;
}
EXPORT_SYMBOL_GPL(xlink_get_device_list);
enum xlink_error xlink_get_device_name(struct xlink_handle *handle, char *name,
				       size_t name_size)
{
	enum xlink_error rc;
	int interface;

	if (!xlink || !handle)
		return X_LINK_ERROR;
	if (!name || !name_size)
		return X_LINK_ERROR;
	interface = get_interface_from_sw_device_id(handle->sw_device_id);
	if (interface == NULL_INTERFACE)
		return X_LINK_ERROR;
	rc = xlink_platform_get_device_name(interface, handle->sw_device_id,
					    name, name_size);
	if (rc)
		rc = X_LINK_ERROR;
	else
		rc = X_LINK_SUCCESS;
	return rc;
}
EXPORT_SYMBOL_GPL(xlink_get_device_name);
enum xlink_error xlink_get_device_status(struct xlink_handle *handle,
					 u32 *device_status)
{
	enum xlink_error rc;
	u32 interface;

	if (!xlink)
		return X_LINK_ERROR;
	if (!device_status)
		return X_LINK_ERROR;
	interface = get_interface_from_sw_device_id(handle->sw_device_id);
	if (interface == NULL_INTERFACE)
		return X_LINK_ERROR;
	rc = xlink_platform_get_device_status(interface, handle->sw_device_id,
					      device_status);
	if (rc)
		rc = X_LINK_ERROR;
	else
		rc = X_LINK_SUCCESS;
	return rc;
}
EXPORT_SYMBOL_GPL(xlink_get_device_status);
enum xlink_error xlink_boot_device(struct xlink_handle *handle,
				   const char *binary_name)
{
	enum xlink_error rc;
	u32 interface;

	if (!xlink || !handle)
		return X_LINK_ERROR;
	if (!binary_name)
		return X_LINK_ERROR;
	interface = get_interface_from_sw_device_id(handle->sw_device_id);
	if (interface == NULL_INTERFACE)
		return X_LINK_ERROR;
	rc = xlink_platform_boot_device(interface, handle->sw_device_id,
					binary_name);
	if (rc)
		rc = X_LINK_ERROR;
	else
		rc = X_LINK_SUCCESS;
	return rc;
}
EXPORT_SYMBOL_GPL(xlink_boot_device);
enum xlink_error xlink_reset_device(struct xlink_handle *handle)
{
	enum xlink_error rc;
	u32 interface;

	if (!xlink || !handle)
		return X_LINK_ERROR;
	interface = get_interface_from_sw_device_id(handle->sw_device_id);
	if (interface == NULL_INTERFACE)
		return X_LINK_ERROR;
	rc = xlink_platform_reset_device(interface, handle->sw_device_id);
	if (rc)
		rc = X_LINK_ERROR;
	else
		rc = X_LINK_SUCCESS;
	return rc;
}
EXPORT_SYMBOL_GPL(xlink_reset_device);
enum xlink_error xlink_set_device_mode(struct xlink_handle *handle,
				       enum xlink_device_power_mode power_mode)
{
	enum xlink_error rc;
	u32 interface;

	if (!xlink || !handle)
		return X_LINK_ERROR;
	interface = get_interface_from_sw_device_id(handle->sw_device_id);
	if (interface == NULL_INTERFACE)
		return X_LINK_ERROR;
	rc = xlink_platform_set_device_mode(interface, handle->sw_device_id,
					    power_mode);
	if (rc)
		rc = X_LINK_ERROR;
	else
		rc = X_LINK_SUCCESS;
	return rc;
}
EXPORT_SYMBOL_GPL(xlink_set_device_mode);

enum xlink_error xlink_get_device_mode(struct xlink_handle *handle,
				       enum xlink_device_power_mode *power_mode)
{
	enum xlink_error rc;
	u32 interface;

	if (!xlink || !handle)
		return X_LINK_ERROR;
	interface = get_interface_from_sw_device_id(handle->sw_device_id);
	if (interface == NULL_INTERFACE)
		return X_LINK_ERROR;
	rc = xlink_platform_get_device_mode(interface, handle->sw_device_id,
					    power_mode);
	if (rc)
		rc = X_LINK_ERROR;
	else
		rc = X_LINK_SUCCESS;
	return rc;
}
EXPORT_SYMBOL_GPL(xlink_get_device_mode);

static void xlink_device_cleanup(uint32_t sw_device_id)
{
	enum xlink_error rc = 0;
	int interface = NULL_INTERFACE;
	int i;

	mutex_lock(&xlink->lock);
	for (i = 0; i < XLINK_MAX_CONNECTIONS; i++) {
		if (xlink->links[i].handle.sw_device_id == sw_device_id) {
			interface = get_interface_from_sw_device_id(sw_device_id);
			if (interface != IPC_INTERFACE) {
				// stop dispatcher
				rc = xlink_dispatcher_stop(xlink->links[i].id);
				if (rc != X_LINK_SUCCESS) {
					pr_err("%s: dispatcher stop failed\n", __func__);
					return;
				}
			}
			// deinitialize multiplexer connection
			rc = xlink_multiplexer_disconnect(xlink->links[i].id);
			if (rc) {
				pr_err("%s: multiplexer disconnect failed\n", __func__);
				return;
			}
			xlink->links[i].handle.sw_device_id = XLINK_INVALID_SW_DEVICE_ID;
			xlink->nmb_connected_links--;
			break;
		}
	}
	mutex_unlock(&xlink->lock);
}

static int xlink_device_event_handler(u32 sw_device_id, u32 event_type)
{
	struct event_info *events = NULL;
	xlink_device_event_cb event_cb;
	bool found = false;
	char event_attr[7];

	mutex_lock(&dev_event_lock);
	// find sw_device_id, event_type in list
	list_for_each_entry(events, &ev_info.list, list) {
		if (events) {
			if (events->sw_device_id == sw_device_id &&
			    events->event_type == event_type) {
				event_cb = events->event_notif_fn;
				found = true;
				break;
			}
		}
	}
	if (found) {
		if (events->user_flag) {
			xlink->eventx[events->event_type].value = events->event_type;
			xlink->eventx[events->event_type].sw_dev_id = sw_device_id;
			sprintf(event_attr, "event%d", events->event_type);
			sysfs_notify(&xlink->dev->kobj, NULL, event_attr);
		} else {
			if (event_cb) {
				event_cb(sw_device_id, event_type);
			} else {
				pr_info("No callback found for sw_device_id : 0x%x event type %d\n",
					sw_device_id, event_type);
				mutex_unlock(&dev_event_lock);
				return X_LINK_ERROR;
			}
		}
		pr_info("sysfs_notify event %d swdev_id %xs\n",
			events->event_type, sw_device_id);
	}
	mutex_unlock(&dev_event_lock);
	if (GET_INTERFACE_FROM_SW_DEVICE_ID(sw_device_id) ==
			SW_DEVICE_ID_PCIE_INTERFACE) {
		switch (event_type) {
		case 0:
			xlink_device_cleanup(sw_device_id);
			break;
		case 1:
			break;
		}
	}
return X_LINK_SUCCESS;
}

static bool event_registered(u32 sw_dev_id, u32 event, u32 user_flag)
{
	struct event_info *events = NULL;

	list_for_each_entry(events, &ev_info.list, list) {
		if (events) {
			if (events->sw_device_id == sw_dev_id &&
			    events->event_type == event &&
			    events->user_flag == user_flag) {
				return true;
			}
		}
	}
return false;
}

static enum xlink_error do_xlink_register_device_event(struct xlink_handle *handle,
						       u32 *event_list,
						       u32 num_events,
						       xlink_device_event_cb event_notif_fn,
						       u32 user_flag)
{
	struct event_info *events;
	u32 interface;
	u32 event;
	int i;

	if (num_events < 0 || num_events >= NUM_REG_EVENTS)
		return X_LINK_ERROR;
	for (i = 0; i < num_events; i++) {
		events = kzalloc(sizeof(*events), GFP_KERNEL);
		if (!events)
			return X_LINK_ERROR;
		event = *event_list;
		events->sw_device_id = handle->sw_device_id;
		events->event_notif_fn = event_notif_fn;
		events->event_type = *event_list++;
		events->user_flag = user_flag;
		if (user_flag) {
			/* only add to list once if userspace */
			/* xlink userspace handles multi process callbacks */
			if (event_registered(handle->sw_device_id, event, user_flag)) {
				pr_info("xlink-core: Event 0x%x - %d already registered\n",
					handle->sw_device_id, event);
				kfree(events);
				continue;
			}
		}
		pr_info("xlink-core:Events: sw_device_id 0x%x event %d fn %p user_flag %d\n",
			events->sw_device_id, events->event_type,
			events->event_notif_fn, events->user_flag);
		list_add_tail(&events->list, &ev_info.list);
	}
	if (num_events > 0) {
		interface = get_interface_from_sw_device_id(handle->sw_device_id);
		if (interface == NULL_INTERFACE)
			return X_LINK_ERROR;
		xlink_platform_register_for_events(interface, handle->sw_device_id,
						   xlink_device_event_handler);
	}
	return X_LINK_SUCCESS;
}

enum xlink_error xlink_register_device_event_user(struct xlink_handle *handle,
						  u32 *event_list, u32 num_events,
						  xlink_device_event_cb event_notif_fn)
{
	enum xlink_error rc;

	rc = do_xlink_register_device_event(handle, event_list, num_events,
					    event_notif_fn, 1);
	return rc;
}

enum xlink_error xlink_register_device_event(struct xlink_handle *handle,
					     u32 *event_list, u32 num_events,
					     xlink_device_event_cb event_notif_fn)
{
	enum xlink_error rc;

	rc = do_xlink_register_device_event(handle, event_list, num_events,
					    event_notif_fn, 0);
	return rc;
}
EXPORT_SYMBOL_GPL(xlink_register_device_event);

enum xlink_error xlink_unregister_device_event(struct xlink_handle *handle,
					       u32 *event_list,
					       u32 num_events)
{
	struct event_info *events = NULL;
	u32 interface;
	int found = 0;
	int count = 0;
	int i;

	if (num_events < 0 || num_events >= NUM_REG_EVENTS)
		return X_LINK_ERROR;
	for (i = 0; i < num_events; i++) {
		list_for_each_entry(events, &ev_info.list, list) {
			if (events->sw_device_id == handle->sw_device_id &&
			    events->event_type == event_list[i]) {
				found = 1;
				break;
			}
		}
		if (!events || !found)
			return X_LINK_ERROR;
		pr_info("removing event %d for sw_device_id 0x%x\n",
			events->event_type, events->sw_device_id);
		list_del(&events->list);
		kfree(events);
	}
	// check if any events left for this sw_device_id
	// are still registered ( in list )
	list_for_each_entry(events, &ev_info.list, list) {
		if (events) {
			if (events->sw_device_id == handle->sw_device_id) {
				count++;
				break;
			}
		}
	}
	if (count == 0) {
		interface = get_interface_from_sw_device_id(handle->sw_device_id);
		if (interface == NULL_INTERFACE)
			return X_LINK_ERROR;
		xlink_platform_unregister_for_events(interface, handle->sw_device_id);
	}
	return X_LINK_SUCCESS;
}
EXPORT_SYMBOL_GPL(xlink_unregister_device_event);

static void kmb_xlink_exit(void)
{
	kmb_xlink_remove();
}
module_exit(kmb_xlink_exit);

MODULE_DESCRIPTION("Keem Bay xlink Kernel Driver");
MODULE_AUTHOR("Seamus Kelly <seamus.kelly@intel.com>");
MODULE_LICENSE("GPL v2");
