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
#include <linux/platform_device.h>
#include <linux/mod_devicetable.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/kref.h>

#ifdef CONFIG_XLINK_LOCAL_HOST
#include <linux/xlink-ipc.h>
#endif

#include "xlink-defs.h"
#include "xlink-multiplexer.h"
#include "xlink-dispatcher.h"
#include "xlink-platform.h"

// xlink version number
#define XLINK_VERSION_MAJOR		0
#define XLINK_VERSION_MINOR		1
#define XLINK_VERSION_REVISION	3
#define XLINK_VERSION_SUB_REV	"a"

// timeout in milliseconds used to wait for the reay message from the VPU
#ifdef CONFIG_XLINK_PSS
#define XLINK_VPU_WAIT_FOR_READY (3000000)
#else
#define XLINK_VPU_WAIT_FOR_READY (3000)
#endif

// device, class, and driver names
#define DEVICE_NAME "xlnk"
#define CLASS_NAME 	"xlkcore"
#define DRV_NAME	"xlink-driver"

// used to determine if an API was called from user or kernel space
#define CHANNEL_SET_USER_BIT(chan) (chan |= (1 << 15))
#define CHANNEL_USER_BIT_IS_SET(chan) (chan & (1 << 15))
#define CHANNEL_CLEAR_USER_BIT(chan) (chan &= ~(1 << 15))

static dev_t xdev;
static struct class *dev_class;
static struct cdev xlink_cdev;
static struct mutex get_device_mode_lock;
static struct mutex set_device_mode_lock;


static long xlink_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static enum xlink_error xlink_write_data_user(struct xlink_handle *handle,
		uint16_t chan, uint8_t const *pmessage, uint32_t size);

static enum xlink_error xlink_write_volatile_user(struct xlink_handle *handle,
		uint16_t chan, uint8_t const *message, uint32_t size);
static enum xlink_error do_xlink_write_volatile(struct xlink_handle *handle,
		uint16_t chan, uint8_t const *message, uint32_t size, uint32_t user_flag);

static enum xlink_error xlink_register_device_event_user(struct xlink_handle *handle,
		uint32_t *event_list, uint32_t num_events,
		xlink_device_event_cb event_notif_fn);
static enum xlink_error do_xlink_register_device_event(struct xlink_handle *handle,
		uint32_t *event_list, uint32_t num_events,
		xlink_device_event_cb event_notif_fn, uint32_t user_flag);
static struct mutex dev_event_lock;


static const struct file_operations fops = {
		.owner 			= THIS_MODULE,
		.unlocked_ioctl = xlink_ioctl,
};

struct xlink_link {
	uint32_t id;
	struct xlink_handle handle;
	struct kref refcount;
};

struct keembay_xlink_dev {
	struct platform_device *pdev;
	struct xlink_link links[XLINK_MAX_CONNECTIONS];
	uint32_t nmb_connected_links;
	struct mutex lock;
};

struct event_info {
	uint32_t sw_device_id;
	uint32_t event_type;
	uint32_t user_flag;
	xlink_device_event_cb event_notif_fn;
	struct list_head list;
};

static struct event_info ev_info;
static struct kobject *ev_kobj;

struct xlink_attr {
    struct attribute attr;
    unsigned long value;
    uint32_t sw_dev_id;
};

static struct xlink_attr event0 = {
    .attr.name = "event0",
    .attr.mode = 0644,
    .value = 0,
};
static struct xlink_attr event1 = {
    .attr.name = "event1",
    .attr.mode = 0644,
    .value = 0,
};
static struct xlink_attr event2 = {
    .attr.name = "event2",
    .attr.mode = 0644,
    .value = 0,
};
static struct xlink_attr event3 = {
    .attr.name = "event3",
    .attr.mode = 0644,
    .value = 0,
};

static struct attribute *ev_attr[] = {
    &event0.attr,
    &event1.attr,
    &event2.attr,
    &event3.attr,
    NULL
};

static ssize_t show(struct kobject *kobj, struct attribute *attr, char *buf)
{
    struct xlink_attr *a = container_of(attr, struct xlink_attr, attr);
    return scnprintf(buf, PAGE_SIZE, "0x%x 0x%lx\n", a->sw_dev_id, a->value);
}

//static ssize_t store(struct kobject *kobj, struct attribute *attr, const char *buf, size_t len)
//{
//    struct xlink_attr *a = container_of(attr, struct xlink_attr, attr);
//
//    sscanf(buf, "%ld", &a->value);
//    printk("sysfs_notify store %s = %lx\n", a->attr.name, a->value);
//    //sysfs_notify(ev_kobj, NULL, "notify");
//    return sizeof(int);
//}
//
static struct sysfs_ops ev_ops = {
    .show = show,
//    .store = store,
};
static struct kobj_type ev_type = {
    .sysfs_ops = &ev_ops,
    .default_attrs = ev_attr,
};

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
	int i = 0;
	struct xlink_link *link = NULL;
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
static struct xlink_link *get_link_by_sw_device_id(uint32_t sw_device_id)
{
	int i = 0;
	struct xlink_link *link = NULL;
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
static int kmb_xlink_probe(struct platform_device *pdev)
{
	int rc, i;
	struct keembay_xlink_dev *xlink_dev;
	struct device *dev_ret;

	dev_info(&pdev->dev, "KeemBay xlink v%d.%d.%d:%s\n", XLINK_VERSION_MAJOR,
		 XLINK_VERSION_MINOR, XLINK_VERSION_REVISION, XLINK_VERSION_SUB_REV);

	xlink_dev = devm_kzalloc(&pdev->dev, sizeof(*xlink), GFP_KERNEL);
	if (!xlink_dev)
		return -ENOMEM;

	xlink_dev->pdev = pdev;

	// initialize multiplexer
	rc = xlink_multiplexer_init(xlink_dev->pdev);
	if (rc != X_LINK_SUCCESS) {
		printk(KERN_DEBUG "Multiplexer initialization failed\n");
		goto r_multiplexer;
	}

	// initialize dispatcher
	rc = xlink_dispatcher_init(xlink_dev->pdev);
	if (rc != X_LINK_SUCCESS) {
		printk(KERN_DEBUG "Dispatcher initialization failed\n");
		goto r_dispatcher;
	}

	// initialize xlink data structure
	xlink_dev->nmb_connected_links = 0;
	mutex_init(&xlink_dev->lock);
	for (i = 0; i < XLINK_MAX_CONNECTIONS; i++) {
		xlink_dev->links[i].id = i;
		xlink_dev->links[i].handle.sw_device_id = XLINK_INVALID_SW_DEVICE_ID;
	}

	platform_set_drvdata(pdev, xlink_dev);

	/* Set the global reference to our device. */
	xlink = xlink_dev;

	/*Allocating Major number*/
	if ((alloc_chrdev_region(&xdev, 0, 1, "xlinkdev")) < 0) {
		dev_info(&pdev->dev, "Cannot allocate major number\n");
		goto r_dispatcher;
	}
	dev_info(&pdev->dev, "Major = %d Minor = %d\n", MAJOR(xdev), MINOR(xdev));

	/*Creating struct class*/
	dev_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(dev_class)) {
		dev_info(&pdev->dev, "Cannot create the struct class - Err %ld\n",
				PTR_ERR(dev_class));
		goto r_class;
	}

	/*Creating device*/
	dev_ret = device_create(dev_class, NULL, xdev, NULL, DEVICE_NAME);
	if (IS_ERR(dev_ret)) {
		dev_info(&pdev->dev, "Cannot create the Device 1 - Err %ld\n",
				PTR_ERR(dev_ret));
		goto r_device;
	}
	dev_info(&pdev->dev, "Device Driver Insert...Done!!!\n");

	/*Creating cdev structure*/
	cdev_init(&xlink_cdev, &fops);

	/*Adding character device to the system*/
	if ((cdev_add(&xlink_cdev, xdev, 1)) < 0) {
		dev_info(&pdev->dev, "Cannot add the device to the system\n");
		goto r_class;
	}
	INIT_LIST_HEAD(&ev_info.list);

	/* create sysfs fds for event notification */
	ev_kobj = kzalloc(sizeof(*ev_kobj), GFP_KERNEL);
	if (ev_kobj) {
		kobject_init(ev_kobj, &ev_type);
		if (kobject_add(ev_kobj, NULL, "%s", "xlink-core-events")) {
			printk("xlink-event: kobject_add() failed\n");
			kobject_put(ev_kobj);
			ev_kobj = NULL;
		} else
			pr_info("xlink-event kobject added\n");
	}
	mutex_init(&dev_event_lock);
	mutex_init(&get_device_mode_lock);
	mutex_init(&set_device_mode_lock);
	return 0;

r_device:
	class_destroy(dev_class);
r_class:
	unregister_chrdev_region(xdev, 1);
r_dispatcher:
	xlink_dispatcher_destroy();
r_multiplexer:
	xlink_multiplexer_destroy();
	return -1;
}

/* Driver removal. */
static int kmb_xlink_remove(struct platform_device *pdev)
{
	int rc = 0;

	mutex_lock(&xlink->lock);
	// destroy multiplexer
	rc = xlink_multiplexer_destroy();
	if (rc != X_LINK_SUCCESS) {
		printk(KERN_DEBUG "Multiplexer destroy failed\n");
	}

	// stop dispatchers and destroy
	rc = xlink_dispatcher_destroy();
	if (rc != X_LINK_SUCCESS) {
		printk(KERN_DEBUG "Dispatcher destroy failed\n");
	}
	mutex_unlock(&xlink->lock);
	mutex_destroy(&xlink->lock);
	// unregister and destroy device
	unregister_chrdev_region(xdev, 1);
	device_destroy(dev_class, xdev);
	cdev_del(&xlink_cdev);
	class_destroy(dev_class);
	kobject_del(ev_kobj);
	printk(KERN_DEBUG "XLink Driver removed\n");
	return 0;
}

/*
 * IOCTL function for User Space access to xlink kernel functions
 *
 */

static long xlink_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int rc = 0;
	int interface = NULL_INTERFACE;
	struct xlink_handle devH = {0};
	struct xlinkopenchannel op = {0};
	struct xlinkwritedata wr = {0};
	struct xlinkreaddata rd = {0};
	struct xlinkreadtobuffer rdtobuf = {0};
	struct xlinkconnect con = {0};
	struct xlinkrelease rel = {0};
	struct xlinkstartvpu startvpu = {0};
	struct xlinkcallback cb = {0};
	struct xlinkgetdevicename devn = {0};
	struct xlinkgetdevicelist devl = {0};
	struct xlinkgetdevicestatus devs = {0};
	struct xlinkbootdevice boot = {0};
	struct xlinkresetdevice res = {0};
	struct xlinkdevmode devm = {0};
	struct xlinkregdevevent regdevevent = {0};
	uint32_t *ev_list;
	uint32_t num_events = 0;
	uint8_t *rdaddr;
	uint32_t size;
	uint8_t reladdr;
	uint8_t volbuf[XLINK_MAX_BUF_SIZE];
	uint8_t *control_buf;
	char filename[64];
	char name[XLINK_MAX_DEVICE_NAME_SIZE];
	uint32_t sw_device_id_list[XLINK_MAX_DEVICE_LIST_SIZE];
	uint32_t num_devices = 0;
	uint32_t device_status = 0;
	uint32_t device_mode = 0;
	struct xlink_link *link = NULL;

	switch (cmd) {
	case XL_CONNECT:
		if (copy_from_user(&con, (int32_t *)arg, sizeof(struct xlinkconnect)))
			return -EFAULT;
		if (copy_from_user(&devH, (struct xlink_handle *)con.handle,
				sizeof(struct xlink_handle)))
			return -EFAULT;
		rc = xlink_connect(&devH);
		if (!rc) {
			if (copy_to_user((struct xlink_handle *)con.handle, &devH,
					sizeof(struct xlink_handle)))
				return -EFAULT;
		}
		if (copy_to_user(con.return_code, (void *)&rc, sizeof(rc)))
			return -EFAULT;
		break;
	case XL_OPEN_CHANNEL:
		if (copy_from_user(&op, (int32_t *)arg,
				sizeof(struct xlinkopenchannel)))
			return -EFAULT;
		if (copy_from_user(&devH, (struct xlink_handle *)op.handle,
				sizeof(struct xlink_handle)))
			return -EFAULT;
		rc = xlink_open_channel(&devH, op.chan, op.mode, op.data_size,
				op.timeout);
		if (copy_to_user(op.return_code, (void *)&rc, sizeof(rc)))
			return -EFAULT;
		break;
	case XL_DATA_READY_CALLBACK:
		if (copy_from_user(&cb, (int32_t *)arg,
				sizeof(struct xlinkcallback)))
			return -EFAULT;
		if (copy_from_user(&devH, (struct xlink_handle *)cb.handle,
				sizeof(struct xlink_handle)))
			return -EFAULT;
		CHANNEL_SET_USER_BIT(cb.chan); // set top bit to indicate a user space call
		rc = xlink_data_available_event(&devH, cb.chan, cb.callback);
		if (copy_to_user(cb.return_code, (void *)&rc, sizeof(rc)))
			return -EFAULT;
		break;
	case XL_DATA_CONSUMED_CALLBACK:
		if (copy_from_user(&cb, (int32_t *)arg,
				sizeof(struct xlinkcallback)))
			return -EFAULT;
		if (copy_from_user(&devH, (struct xlink_handle *)cb.handle,
				sizeof(struct xlink_handle)))
			return -EFAULT;
		CHANNEL_SET_USER_BIT(cb.chan); // set top bit to indicate a user space call
		rc = xlink_data_consumed_event(&devH, cb.chan, cb.callback);
		if (copy_to_user(cb.return_code, (void *)&rc, sizeof(rc)))
			return -EFAULT;
		break;
	case XL_READ_DATA:
		if (copy_from_user(&rd, (int32_t *)arg, sizeof(struct xlinkreaddata)))
			return -EFAULT;
		if (copy_from_user(&devH, (struct xlink_handle *)rd.handle,
					sizeof(struct xlink_handle)))
			return -EFAULT;
		rc = xlink_read_data(&devH, rd.chan, &rdaddr, &size);
		if (!rc) {
			interface = get_interface_from_sw_device_id(devH.sw_device_id);
			if (interface == IPC_INTERFACE) {
				if (copy_to_user(rd.pmessage, (void *)&rdaddr,
						sizeof(uint32_t)))
					return -EFAULT;
			} else {
				if (copy_to_user(rd.pmessage, (void *)rdaddr, size))
					return -EFAULT;
			}
			if (copy_to_user(rd.size, (void *)&size, sizeof(size)))
				return -EFAULT;
			// this only releases a packet if we are remote host and
			// rd.chan is a passthru ipc channel
			link = get_link_by_sw_device_id(devH.sw_device_id);
			if (link) {
				rc = core_release_packet_from_channel(link->id, rd.chan,
						rdaddr);
			}
		}
		if (copy_to_user(rd.return_code, (void *)&rc, sizeof(rc)))
			return -EFAULT;
		break;
	case XL_READ_TO_BUFFER:

		if (copy_from_user(&rdtobuf, (int32_t *)arg,
				sizeof(struct xlinkreadtobuffer)))
			return -EFAULT;
		if (copy_from_user(&devH, (struct xlink_handle *)rdtobuf.handle,
				sizeof(struct xlink_handle)))
			return -EFAULT;
		rc = xlink_read_data_to_buffer(&devH, rdtobuf.chan, (uint8_t *)volbuf,
				&size);
		if (!rc) {
			if (copy_to_user(rdtobuf.pmessage, (void *)volbuf, size))
				return -EFAULT;
			if (copy_to_user(rdtobuf.size, (void *)&size, sizeof(size)))
				return -EFAULT;
		}
		if (copy_to_user(rdtobuf.return_code, (void *)&rc, sizeof(rc)))
			return -EFAULT;
		break;
	case XL_WRITE_DATA:
		if (copy_from_user(&wr, (int32_t *)arg, sizeof(struct xlinkwritedata)))
			return -EFAULT;
		if (copy_from_user(&devH, (struct xlink_handle *)wr.handle,
				sizeof(struct xlink_handle)))
			return -EFAULT;
		if (wr.size <= XLINK_MAX_DATA_SIZE) {
			rc = xlink_write_data_user(&devH, wr.chan, wr.pmessage, wr.size);
			if (copy_to_user(wr.return_code, (void *)&rc, sizeof(rc)))
				return -EFAULT;
		} else {
			return -EFAULT;
		}
		break;
	case XL_WRITE_VOLATILE:
		if (copy_from_user(&wr, (int32_t *)arg, sizeof(struct xlinkwritedata)))
			return -EFAULT;
		if (copy_from_user(&devH, (struct xlink_handle *)wr.handle,
				sizeof(struct xlink_handle)))
			return -EFAULT;
		if (wr.size <= XLINK_MAX_BUF_SIZE) {
			if (copy_from_user(volbuf, (char *)wr.pmessage, wr.size))
				return -EFAULT;
			rc = xlink_write_volatile_user(&devH, wr.chan, volbuf, wr.size);
			if (copy_to_user(wr.return_code, (void *)&rc, sizeof(rc)))
				return -EFAULT;
		} else {
			return -EFAULT;
		}
		break;
	case XL_WRITE_CONTROL_DATA:
		if (copy_from_user(&wr, (int32_t *)arg, sizeof(struct xlinkwritedata)))
			return -EFAULT;
		if (copy_from_user(&devH, (struct xlink_handle *)wr.handle,
				sizeof(struct xlink_handle)))
			return -EFAULT;
		if (wr.size <= XLINK_MAX_CONTROL_DATA_PCIE_SIZE) {
			control_buf = (uint8_t *)kzalloc(
					XLINK_MAX_CONTROL_DATA_PCIE_SIZE,
					GFP_KERNEL);
			if (!control_buf)
				return -ENOMEM;
			if (copy_from_user(control_buf,
					(char *)wr.pmessage, wr.size)) {
				kfree(control_buf);
				return -EFAULT;
			}
			rc = xlink_write_control_data(&devH, wr.chan,
						control_buf, wr.size);
			kfree(control_buf);
			if (copy_to_user(wr.return_code, (void *)&rc,
						sizeof(rc)))
				return -EFAULT;
		} else {
			//return -EFAULT;
			rc = X_LINK_ERROR;
			if (copy_to_user(wr.return_code, (void *)&rc,
						sizeof(rc)))
				return -EFAULT;
			return X_LINK_ERROR;
		}
		break;
	case XL_RELEASE_DATA:
		if (copy_from_user(&rel, (int32_t *)arg, sizeof(struct xlinkrelease)))
			return -EFAULT;
		if (copy_from_user(&devH, (struct xlink_handle *)rel.handle,
				sizeof(struct xlink_handle)))
			return -EFAULT;
		if (rel.addr) {
			if (get_user(reladdr, (uint32_t *const)rel.addr))
				return -EFAULT;
			rc = xlink_release_data(&devH, rel.chan, (uint8_t *)&reladdr);
		} else {
			rc = xlink_release_data(&devH, rel.chan, NULL);
		}
		if (copy_to_user(rel.return_code, (void *)&rc, sizeof(rc)))
			return -EFAULT;
		break;
	case XL_CLOSE_CHANNEL:
		if (copy_from_user(&op, (int32_t *)arg,
				sizeof(struct xlinkopenchannel)))
			return -EFAULT;
		if (copy_from_user(&devH, (struct xlink_handle *)op.handle,
				sizeof(struct xlink_handle)))
			return -EFAULT;
		rc = xlink_close_channel(&devH, op.chan);
		if (copy_to_user(op.return_code, (void *)&rc, sizeof(rc)))
			return -EFAULT;
		break;
	case XL_START_VPU:
		if (copy_from_user(&startvpu, (int32_t *)arg,
				sizeof(struct xlinkstartvpu)))
			return -EFAULT;
		if (startvpu.namesize > sizeof(filename))
			return -EINVAL;
		memset(filename, 0, sizeof(filename));
		if (copy_from_user(filename, startvpu.filename, startvpu.namesize))
			return -EFAULT;
		rc = xlink_start_vpu(filename);
		if (copy_to_user(startvpu.return_code, (void *)&rc, sizeof(rc)))
			return -EFAULT;
		break;
	case XL_STOP_VPU:
		rc = xlink_stop_vpu();
		break;
	case XL_RESET_VPU:
		rc = xlink_stop_vpu();
		break;
	case XL_DISCONNECT:
		if (copy_from_user(&con, (int32_t *)arg, sizeof(struct xlinkconnect)))
			return -EFAULT;
		if (copy_from_user(&devH, (struct xlink_handle *)con.handle,
				sizeof(struct xlink_handle)))
			return -EFAULT;
		rc = xlink_disconnect(&devH);
		if (copy_to_user(con.return_code, (void *)&rc, sizeof(rc)))
			return -EFAULT;
		break;
	case XL_GET_DEVICE_NAME:
		if (copy_from_user(&devn, (int32_t *)arg,
				sizeof(struct xlinkgetdevicename)))
			return -EFAULT;
		if (copy_from_user(&devH, (struct xlink_handle *)devn.handle,
				sizeof(struct xlink_handle)))
			return -EFAULT;
		rc = xlink_get_device_name(&devH, name, devn.name_size);
		if (!rc) {
			if (copy_to_user(devn.name, (void *)name, devn.name_size))
				return -EFAULT;
		}
		if (copy_to_user(devn.return_code, (void *)&rc, sizeof(rc)))
			return -EFAULT;
		break;
	case XL_GET_DEVICE_LIST:
		if (copy_from_user(&devl, (int32_t *)arg,
				sizeof(struct xlinkgetdevicelist)))
			return -EFAULT;
		rc = xlink_get_device_list(sw_device_id_list, &num_devices);
		if (!rc && (num_devices <= XLINK_MAX_DEVICE_LIST_SIZE)) {
			/* TODO: this next copy is dangerous! we have no idea how large
			   the devl.sw_device_id_list buffer is provided by the user.
			   if num_devices is too large, the copy will overflow the buffer.
			 */
			if (copy_to_user(devl.sw_device_id_list, (void *)sw_device_id_list,
					(sizeof(*sw_device_id_list) * num_devices)))
				return -EFAULT;
			if (copy_to_user(devl.num_devices, (void *)&num_devices,
					(sizeof(num_devices))))
				return -EFAULT;
		}
		if (copy_to_user(devl.return_code, (void *)&rc, sizeof(rc)))
			return -EFAULT;
		break;
	case XL_GET_DEVICE_STATUS:
		if (copy_from_user(&devs, (int32_t *)arg,
				sizeof(struct xlinkgetdevicestatus)))
			return -EFAULT;
		if (copy_from_user(&devH, (struct xlink_handle *)devs.handle,
				sizeof(struct xlink_handle)))
			return -EFAULT;
		rc = xlink_get_device_status(&devH, &device_status);
		if (!rc) {
			if (copy_to_user(devs.device_status, (void *)&device_status,
					sizeof(device_status)))
				return -EFAULT;
		}
		if (copy_to_user(devs.return_code, (void *)&rc, sizeof(rc)))
			return -EFAULT;
		break;
	case XL_BOOT_DEVICE:
		if (copy_from_user(&boot, (int32_t *)arg,
				sizeof(struct xlinkbootdevice)))
			return -EFAULT;
		if (copy_from_user(&devH, (struct xlink_handle *)boot.handle,
				sizeof(struct xlink_handle)))
			return -EFAULT;
		if (boot.binary_name_size > sizeof(filename))
			return -EINVAL;
		memset(filename, 0, sizeof(filename));
		if (copy_from_user(filename, boot.binary_name, boot.binary_name_size))
			return -EFAULT;
		rc = xlink_boot_device(&devH, filename);
		if (copy_to_user(boot.return_code, (void *)&rc, sizeof(rc)))
			return -EFAULT;
		break;
	case XL_RESET_DEVICE:
		if (copy_from_user(&res, (int32_t *)arg,
				sizeof(struct xlinkresetdevice)))
			return -EFAULT;
		if (copy_from_user(&devH, (struct xlink_handle *)res.handle,
				sizeof(struct xlink_handle)))
			return -EFAULT;
		rc = xlink_reset_device(&devH);
		if (copy_to_user(res.return_code, (void *)&rc, sizeof(rc)))
			return -EFAULT;
		break;
	case XL_GET_DEVICE_MODE:
		if (copy_from_user(&devm, (int32_t *)arg,
				sizeof(struct xlinkdevmode)))
			return -EFAULT;
		if (copy_from_user(&devH, (struct xlink_handle *)devm.handle,
				sizeof(struct xlink_handle)))
			return -EFAULT;
		rc = xlink_get_device_mode(&devH, &device_mode);
		if (!rc) {
			if (copy_to_user(devm.device_mode, (void *)&device_mode,
					sizeof(device_mode)))
				return -EFAULT;
		}
		if (copy_to_user(devm.return_code, (void *)&rc, sizeof(rc)))
			return -EFAULT;
		break;
	case XL_SET_DEVICE_MODE:
		if (copy_from_user(&devm, (int32_t *)arg,
				sizeof(struct xlinkdevmode)))
			return -EFAULT;
		if (copy_from_user(&devH, (struct xlink_handle *)devm.handle,
				sizeof(struct xlink_handle)))
			return -EFAULT;
		if (copy_from_user(&device_mode, (uint32_t *)devm.device_mode,
				sizeof(device_mode)))
			return -EFAULT;
		rc = xlink_set_device_mode(&devH, device_mode);
		if (copy_to_user(devm.return_code, (void *)&rc, sizeof(rc)))
			return -EFAULT;
		break;
	case XL_REGISTER_DEV_EVENT:
		if (copy_from_user(&regdevevent, (int32_t *)arg,
				sizeof(struct xlinkregdevevent)))
			return -EFAULT;
		if (copy_from_user(&devH, (struct xlink_handle *)regdevevent.handle,
				sizeof(struct xlink_handle)))
			return -EFAULT;
		num_events = regdevevent.num_events;
		ev_list = kzalloc((num_events*sizeof(uint32_t)), GFP_KERNEL);
		if (copy_from_user(ev_list, (uint32_t *)regdevevent.event_list,
				(num_events*sizeof(uint32_t))))
			return -EFAULT;
		rc = xlink_register_device_event_user(&devH, ev_list, num_events, NULL);
		if (copy_to_user(regdevevent.return_code, (void *)&rc, sizeof(rc)))
			return -EFAULT;
		break;
	case XL_UNREGISTER_DEV_EVENT:
		if (copy_from_user(&regdevevent, (int32_t *)arg,
				sizeof(struct xlinkregdevevent)))
			return -EFAULT;
		if (copy_from_user(&devH, (struct xlink_handle *)regdevevent.handle,
				sizeof(struct xlink_handle)))
			return -EFAULT;
		num_events = regdevevent.num_events;
		ev_list = kzalloc((num_events*sizeof(uint32_t)), GFP_KERNEL);
		if (copy_from_user(ev_list, (uint32_t *)regdevevent.event_list,
				(num_events*sizeof(uint32_t))))
			return -EFAULT;
		rc = xlink_unregister_device_event(&devH, ev_list, num_events);
		if (copy_to_user(regdevevent.return_code, (void *)&rc, sizeof(rc)))
			return -EFAULT;
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
	int rc = 0;
	rc = xlink_ipc_reset_device(0x0); // stop vpu slice 0
	if (rc)
		return X_LINK_ERROR;
#endif
	return X_LINK_SUCCESS;
}
EXPORT_SYMBOL(xlink_stop_vpu);

enum xlink_error xlink_start_vpu(char *filename)
{
#ifdef CONFIG_XLINK_LOCAL_HOST
	int rc = 0;
	rc = xlink_ipc_boot_device(0x0, filename); // start vpu slice 0
	if (rc)
		return X_LINK_ERROR;
#endif
	return X_LINK_SUCCESS;
}
EXPORT_SYMBOL(xlink_start_vpu);

enum xlink_error xlink_initialize(void)
{
	return X_LINK_SUCCESS;
}
EXPORT_SYMBOL(xlink_initialize);

enum xlink_error xlink_connect(struct xlink_handle *handle)
{
	enum xlink_error rc = 0;
	struct xlink_link *link = NULL;
	int interface = NULL_INTERFACE;

	if (!xlink || !handle)
		return X_LINK_ERROR;

	link = get_link_by_sw_device_id(handle->sw_device_id);
	mutex_lock(&xlink->lock);
	if (!link) {
		link = get_next_link();
		if (!link) {
			printk(KERN_DEBUG "xlink_connect - max connections reached%d\n",
					XLINK_MAX_CONNECTIONS);
			mutex_unlock(&xlink->lock);
			return X_LINK_ERROR;
		}
		// platform connect
		interface = get_interface_from_sw_device_id(handle->sw_device_id);
		rc = xlink_platform_connect(interface, handle->sw_device_id);
		if (rc) {
			printk(KERN_DEBUG "xlink_connect - platform connect failed %d\n", rc);
			mutex_unlock(&xlink->lock);
			return X_LINK_ERROR;
		}
		// set link handle reference and link id
		link->handle = *handle;
		xlink->nmb_connected_links++;
		kref_init(&link->refcount);
		if (interface != IPC_INTERFACE) {
			// start dispatcher
			rc = xlink_dispatcher_start(link->id, &link->handle);
			if (rc) {
				printk(KERN_INFO "xlink_connect - dispatcher start failed\n");
				goto r_cleanup;
			}
		}
		// initialize multiplexer connection
		rc = xlink_multiplexer_connect(link->id);
		if (rc) {
			printk(KERN_INFO "xlink_connect - multiplexer connect failed\n");
			goto r_cleanup;
		}
		printk(KERN_INFO "dev 0x%x connected - dev_type %d - nmb_connected_links %d\n",
				link->handle.sw_device_id, link->handle.dev_type,
				xlink->nmb_connected_links);
	} else {
		// already connected
		printk(KERN_INFO "dev 0x%x ALREADY connected - dev_type %d\n",
				link->handle.sw_device_id, link->handle.dev_type);
		kref_get(&link->refcount);
		*handle = link->handle;
	}
	mutex_unlock(&xlink->lock);
	// TODO: implement ping
	return X_LINK_SUCCESS;

r_cleanup:
	link->handle.sw_device_id = XLINK_INVALID_SW_DEVICE_ID;
	mutex_unlock(&xlink->lock);
	return X_LINK_ERROR;

}
EXPORT_SYMBOL(xlink_connect);

enum xlink_error xlink_data_available_event(struct xlink_handle *handle,
		uint16_t chan, xlink_event data_available_event)
{
	enum xlink_error rc = 0;
	struct xlink_link *link = NULL;
	struct xlink_event *event = NULL;
	int event_queued = 0;
	char origin = 'K';

	if (CHANNEL_USER_BIT_IS_SET(chan))
		origin = 'U';     // function called from user space
	CHANNEL_CLEAR_USER_BIT(chan);  // restore proper channel value

	if (!xlink || !handle)
		return X_LINK_ERROR;

	link = get_link_by_sw_device_id(handle->sw_device_id);
	if (!link)
		return X_LINK_ERROR;

	event = xlink_create_event(link->id, XLINK_DATA_READY_CALLBACK_REQ,
			&link->handle, chan, 0, 0);
	if (!event)
		return X_LINK_ERROR;

	event->data = data_available_event;
	event->callback_origin = origin;
	if (!data_available_event)	// if NULL disable callbacks for this channel
		event->calling_pid = NULL;
	else
		event->calling_pid = current;
	rc = xlink_multiplexer_tx(event, &event_queued);
	if (!event_queued) {
		xlink_destroy_event(event);
	}
	return rc;
}
EXPORT_SYMBOL(xlink_data_available_event);


enum xlink_error xlink_data_consumed_event(struct xlink_handle *handle,
		uint16_t chan, xlink_event data_consumed_event)
{
	enum xlink_error rc = 0;
	struct xlink_link *link = NULL;
	struct xlink_event *event = NULL;
	int event_queued = 0;
	char origin = 'K';

	if (chan & (1 << 15))
		origin = 'U';    // user space call
	chan &= ~(1 << 15);  // clear top bit


	if (!xlink || !handle)
		return X_LINK_ERROR;

	link = get_link_by_sw_device_id(handle->sw_device_id);
	if (!link)
		return X_LINK_ERROR;

	event = xlink_create_event(link->id, XLINK_DATA_CONSUMED_CALLBACK_REQ,
			&link->handle, chan, 0, 0);
	if (!event)
		return X_LINK_ERROR;

	event->data = data_consumed_event;
	event->callback_origin = origin;
	if (!data_consumed_event)	// if NULL disable callbacks for this channel
		event->calling_pid = NULL;
	else
		event->calling_pid = current;

	rc = xlink_multiplexer_tx(event, &event_queued);
	if (!event_queued) {
		xlink_destroy_event(event);
	}
	return rc;
}
EXPORT_SYMBOL(xlink_data_consumed_event);

enum xlink_error xlink_open_channel(struct xlink_handle *handle,
		uint16_t chan, enum xlink_opmode mode, uint32_t data_size,
		uint32_t timeout)
{
	enum xlink_error rc = 0;
	struct xlink_link *link = NULL;
	struct xlink_event *event = NULL;
	int event_queued = 0;

	if (!xlink || !handle)
		return X_LINK_ERROR;

	trace_xlink_open_channel(handle->sw_device_id, chan);
	link = get_link_by_sw_device_id(handle->sw_device_id);
	if (!link)
		return X_LINK_ERROR;

	event = xlink_create_event(link->id, XLINK_OPEN_CHANNEL_REQ,
			&link->handle, chan, data_size, timeout);
	if (!event)
		return X_LINK_ERROR;

	event->data = (void *)mode;
	rc = xlink_multiplexer_tx(event, &event_queued);
	if (!event_queued) {
		xlink_destroy_event(event);
	}
	trace_xlink_open_channel_completion(handle->sw_device_id, chan);
	return rc;
}
EXPORT_SYMBOL(xlink_open_channel);

enum xlink_error xlink_close_channel(struct xlink_handle *handle,
		uint16_t chan)
{
	enum xlink_error rc = 0;
	struct xlink_link *link = NULL;
	struct xlink_event *event = NULL;
	int event_queued = 0;

	if (!xlink || !handle)
		return X_LINK_ERROR;

	trace_xlink_close_channel(handle->sw_device_id, chan);
	link = get_link_by_sw_device_id(handle->sw_device_id);
	if (!link)
		return X_LINK_ERROR;

	event = xlink_create_event(link->id, XLINK_CLOSE_CHANNEL_REQ,
			&link->handle, chan, 0, 0);
	if (!event)
		return X_LINK_ERROR;

	rc = xlink_multiplexer_tx(event, &event_queued);
	if (!event_queued) {
		xlink_destroy_event(event);
	}
	trace_xlink_close_channel(handle->sw_device_id, chan);
	return rc;
}
EXPORT_SYMBOL(xlink_close_channel);

enum xlink_error xlink_write_data(struct xlink_handle *handle,
		uint16_t chan, uint8_t const *pmessage, uint32_t size)
{
	enum xlink_error rc = 0;
	struct xlink_link *link = NULL;
	struct xlink_event *event = NULL;
	int event_queued = 0;

	if (!xlink || !handle)
		return X_LINK_ERROR;

	trace_xlink_write_data(handle->sw_device_id, chan, size);
	if (size > XLINK_MAX_DATA_SIZE)
		return X_LINK_ERROR;

	link = get_link_by_sw_device_id(handle->sw_device_id);
	if (!link)
		return X_LINK_ERROR;

	event = xlink_create_event(link->id, XLINK_WRITE_REQ, &link->handle, chan,
			size, 0);
	if (!event)
		return X_LINK_ERROR;

	if (chan < XLINK_IPC_MAX_CHANNELS && event->interface == IPC_INTERFACE) {
		/* only passing message address across IPC interface */
		event->data = &pmessage;
		rc = xlink_multiplexer_tx(event, &event_queued);
		xlink_destroy_event(event);
	} else {
		event->data = (uint8_t *)pmessage;
		event->paddr = 0;
		rc = xlink_multiplexer_tx(event, &event_queued);
		if (!event_queued) {
			xlink_destroy_event(event);
		}
	}
	trace_xlink_write_data_completion(handle->sw_device_id, chan, size);
	return rc;
}
EXPORT_SYMBOL(xlink_write_data);

static enum xlink_error xlink_write_data_user(struct xlink_handle *handle,
		uint16_t chan, uint8_t const *pmessage, uint32_t size)
{
	enum xlink_error rc = 0;
	struct xlink_link *link = NULL;
	struct xlink_event *event = NULL;
	int event_queued = 0;
	dma_addr_t paddr = 0;
	uint32_t addr;

	if (!xlink || !handle)
		return X_LINK_ERROR;

	trace_xlink_write_data_user(handle->sw_device_id, chan, size);
	if (size > XLINK_MAX_DATA_SIZE)
		return X_LINK_ERROR;

	link = get_link_by_sw_device_id(handle->sw_device_id);
	if (!link)
		return X_LINK_ERROR;

	event = xlink_create_event(link->id, XLINK_WRITE_REQ, &link->handle, chan,
			size, 0);
	if (!event)
		return X_LINK_ERROR;
	event->user_data = 1;

	if (chan < XLINK_IPC_MAX_CHANNELS && event->interface == IPC_INTERFACE) {
		/* only passing message address across IPC interface */
		if (get_user(addr, (uint32_t *)pmessage)) {
			xlink_destroy_event(event);
			return X_LINK_ERROR;
		}
		event->data = &addr;
		rc = xlink_multiplexer_tx(event, &event_queued);
		xlink_destroy_event(event);
	} else {
		event->data = xlink_platform_allocate(&xlink->pdev->dev, &paddr, size,
				XLINK_PACKET_ALIGNMENT, XLINK_NORMAL_MEMORY);
		if (!event->data) {
			xlink_destroy_event(event);
			return X_LINK_ERROR;
		}
		if (copy_from_user(event->data, (char *)pmessage, size)) {
			xlink_platform_deallocate(&xlink->pdev->dev, event->data, paddr,
					size, XLINK_PACKET_ALIGNMENT, XLINK_NORMAL_MEMORY);
			xlink_destroy_event(event);
			return X_LINK_ERROR;
		}
		event->paddr = paddr;
		rc = xlink_multiplexer_tx(event, &event_queued);
		if (!event_queued) {
			xlink_platform_deallocate(&xlink->pdev->dev, event->data, paddr,
					size, XLINK_PACKET_ALIGNMENT, XLINK_NORMAL_MEMORY);
			xlink_destroy_event(event);
		}
	}
	trace_xlink_write_data_user_completion(handle->sw_device_id,
							chan, size);
	return rc;
}

enum xlink_error xlink_write_control_data(struct xlink_handle *handle,
		uint16_t chan, uint8_t const *pmessage, uint32_t size)
{
	enum xlink_error rc = 0;
	struct xlink_link *link = NULL;
	struct xlink_event *event = NULL;
	int event_queued = 0;

	if (!xlink || !handle)
		return X_LINK_ERROR;

	trace_xlink_write_control(handle->sw_device_id, chan, size);
	if (chan < XLINK_IPC_MAX_CHANNELS) {
		if (size > XLINK_MAX_CONTROL_DATA_SIZE)
			return X_LINK_ERROR; // TODO: XLink Parameter Error
	}
	if (chan >= XLINK_IPC_MAX_CHANNELS) {
		if (size > XLINK_MAX_CONTROL_DATA_PCIE_SIZE)
			return X_LINK_ERROR; // TODO: XLink Parameter Error
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
	if (!event_queued) {
		xlink_destroy_event(event);
	}
	trace_xlink_write_control_completion(handle->sw_device_id, chan, size);
	return rc;
}
EXPORT_SYMBOL(xlink_write_control_data);

static enum xlink_error xlink_write_volatile_user(struct xlink_handle *handle,
		uint16_t chan, uint8_t const *message, uint32_t size)
{
	enum xlink_error rc = 0;
	rc = do_xlink_write_volatile(handle, chan, message, size, 1);
	return rc;
}

enum xlink_error xlink_write_volatile(struct xlink_handle *handle,
		uint16_t chan, uint8_t const *message, uint32_t size)
{
	enum xlink_error rc = 0;
	rc = do_xlink_write_volatile(handle, chan, message, size, 0);
	return rc;
}
EXPORT_SYMBOL(xlink_write_volatile);


static enum xlink_error do_xlink_write_volatile(struct xlink_handle *handle,
		uint16_t chan, uint8_t const *message, uint32_t size, uint32_t user_flag)
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
		return X_LINK_ERROR; // TODO: XLink Parameter Error

	link = get_link_by_sw_device_id(handle->sw_device_id);
	if (!link)
		return X_LINK_ERROR;

	event = xlink_create_event(link->id, XLINK_WRITE_VOLATILE_REQ,
			&link->handle, chan, size, 0);
	if (!event)
		return X_LINK_ERROR;

	region = XLINK_NORMAL_MEMORY;
	event->data = xlink_platform_allocate(&xlink->pdev->dev, &paddr, size,
			XLINK_PACKET_ALIGNMENT, region);
	if (!event->data) {
		xlink_destroy_event(event);
		return X_LINK_ERROR;
	} else {
		memcpy(event->data, message, size);
	}
	event->user_data = user_flag;
	event->paddr = paddr;
	rc = xlink_multiplexer_tx(event, &event_queued);
	if (!event_queued) {
		xlink_platform_deallocate(&xlink->pdev->dev, event->data, paddr, size,
			XLINK_PACKET_ALIGNMENT, region);
		xlink_destroy_event(event);
	}
	return rc;
}

enum xlink_error xlink_write_data_crc(struct xlink_handle *handle,
		uint16_t chan, const uint8_t *message,
		uint32_t size)
{
	enum xlink_error rc = 0;
	/* To be implemented */
	return rc;
}
EXPORT_SYMBOL(xlink_write_data_crc);

enum xlink_error xlink_read_data(struct xlink_handle *handle,
		uint16_t chan, uint8_t **pmessage, uint32_t *size)
{
	enum xlink_error rc = 0;
	struct xlink_link *link = NULL;
	struct xlink_event *event = NULL;
	int event_queued = 0;

	if (!xlink || !handle)
		return X_LINK_ERROR;

	trace_xlink_read(handle->sw_device_id, chan, *size);
	link = get_link_by_sw_device_id(handle->sw_device_id);
	if (!link)
		return X_LINK_ERROR;

	trace_xlink_read_mutex(handle->sw_device_id, chan, *size);
	event = xlink_create_event(link->id, XLINK_READ_REQ, &link->handle, chan,
			*size, 0);

	trace_xlink_read_alloc_event(handle->sw_device_id, chan, *size);
	if (!event)
		return X_LINK_ERROR;

	event->pdata = (void **)pmessage;
	event->length = size;
	rc = xlink_multiplexer_tx(event, &event_queued);
	if (!event_queued) {
		xlink_destroy_event(event);
	}
	trace_xlink_read_data_completion(handle->sw_device_id, chan, *size);
	return rc;
}
EXPORT_SYMBOL(xlink_read_data);

enum xlink_error xlink_read_data_to_buffer(struct xlink_handle *handle,
		uint16_t chan, uint8_t * const message, uint32_t *size)
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
	if (!event_queued) {
		xlink_destroy_event(event);
	}
	return rc;
}
EXPORT_SYMBOL(xlink_read_data_to_buffer);

enum xlink_error xlink_read_data_to_buffer_crc(struct xlink_handle *handle,
		uint16_t chan, uint8_t * const message, uint32_t *size)
{
	enum xlink_error rc = 0;
	/* To be implemented */
	return rc;
}
EXPORT_SYMBOL(xlink_read_data_to_buffer_crc);

enum xlink_error xlink_release_data(struct xlink_handle *handle,
		uint16_t chan, uint8_t * const data_addr)
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

	event = xlink_create_event(link->id, XLINK_RELEASE_REQ, &link->handle,
			chan, 0, 0);
	if (!event)
		return X_LINK_ERROR;

	event->data = data_addr;
	rc = xlink_multiplexer_tx(event, &event_queued);
	if (!event_queued) {
		xlink_destroy_event(event);
	}
	return rc;
}
EXPORT_SYMBOL(xlink_release_data);

enum xlink_error xlink_disconnect(struct xlink_handle *handle)
{
	enum xlink_error rc = 0;
	struct xlink_link *link = NULL;
	int interface = NULL_INTERFACE;

	if (!xlink || !handle)
		return X_LINK_ERROR;

	link = get_link_by_sw_device_id(handle->sw_device_id);
	if (!link)
		return X_LINK_ERROR;

	// decrement refcount, if count is 0 lock mutex and disconnect
	if (kref_put_mutex(&link->refcount, release_after_kref_put, &xlink->lock)) {
		// stop dispatcher
		interface = get_interface_from_sw_device_id(link->handle.sw_device_id);
		if (interface != IPC_INTERFACE) {
			// stop dispatcher
			rc = xlink_dispatcher_stop(link->id);
			if (rc != X_LINK_SUCCESS) {
				printk(KERN_DEBUG "xlink_disconnect - dispatcher stop failed\n");
				mutex_unlock(&xlink->lock);
				return X_LINK_ERROR;
			}
		}
		// deinitialize multiplexer connection
		rc = xlink_multiplexer_disconnect(link->id);
		if (rc) {
			printk(KERN_INFO "xlink_disconnect - multiplexer disconnect failed\n");
			mutex_unlock(&xlink->lock);
			return X_LINK_ERROR;
		}
		// TODO: reset device?
		// invalidate link handle reference
		link->handle.sw_device_id = XLINK_INVALID_SW_DEVICE_ID;
		xlink->nmb_connected_links--;
		mutex_unlock(&xlink->lock);
	}
	return rc;
}
EXPORT_SYMBOL(xlink_disconnect);

enum xlink_error xlink_get_device_list(uint32_t *sw_device_id_list,
		uint32_t *num_devices)
{
	enum xlink_error rc = 0;
	uint32_t interface_nmb_devices = 0;
	int i = 0;

	if (!xlink)
		return X_LINK_ERROR;

	if (!sw_device_id_list || !num_devices)
		return X_LINK_ERROR;

	/* loop through each interface and combine the lists */
	for (i = 0; i < NMB_OF_INTERFACES; i++) {
		rc = xlink_platform_get_device_list(i, sw_device_id_list, &interface_nmb_devices);
		if (!rc) {
			*num_devices += interface_nmb_devices;
			sw_device_id_list += interface_nmb_devices;
		}
		interface_nmb_devices = 0;
	}
	return X_LINK_SUCCESS;
}
EXPORT_SYMBOL(xlink_get_device_list);

enum xlink_error xlink_get_device_name(struct xlink_handle *handle, char *name,
		size_t name_size)
{
	enum xlink_error rc = 0;
	int interface = 0;

	if (!xlink || !handle)
		return X_LINK_ERROR;

	if (!name || !name_size)
		return X_LINK_ERROR;

	interface = get_interface_from_sw_device_id(handle->sw_device_id);
	if (interface == NULL_INTERFACE)
		return X_LINK_ERROR;

	rc = xlink_platform_get_device_name(interface, handle->sw_device_id, name,
			name_size);
	if (rc)
		rc = X_LINK_ERROR;
	else
		rc = X_LINK_SUCCESS;
	return rc;
}
EXPORT_SYMBOL(xlink_get_device_name);

enum xlink_error xlink_get_device_status(struct xlink_handle *handle,
		uint32_t *device_status)
{
	enum xlink_error rc = 0;
	uint32_t interface = 0;

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
EXPORT_SYMBOL(xlink_get_device_status);

enum xlink_error xlink_boot_device(struct xlink_handle *handle,
		const char *binary_name)
{
	enum xlink_error rc = 0;
	uint32_t interface = 0;

	if (!xlink || !handle)
		return X_LINK_ERROR;

	if (!binary_name)
		return X_LINK_ERROR;

	interface = get_interface_from_sw_device_id(handle->sw_device_id);
	if (interface == NULL_INTERFACE)
		return X_LINK_ERROR;

	rc = xlink_platform_boot_device(interface, handle->sw_device_id, binary_name);
	if (rc)
		rc = X_LINK_ERROR;
	else
		rc = X_LINK_SUCCESS;
	return rc;
}
EXPORT_SYMBOL(xlink_boot_device);

enum xlink_error xlink_reset_device(struct xlink_handle *handle)
{
	enum xlink_error rc = 0;
	uint32_t interface = 0;

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
EXPORT_SYMBOL(xlink_reset_device);

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
enum xlink_error xlink_set_device_mode(struct xlink_handle *handle,
		enum xlink_device_power_mode power_mode)
{
	enum xlink_error rc = 0;
	uint32_t interface = 0;

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
EXPORT_SYMBOL(xlink_set_device_mode);

enum xlink_error xlink_get_device_mode(struct xlink_handle *handle,
		uint32_t *power_mode)
{
	enum xlink_error rc = 0;
	uint32_t interface = 0;

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
EXPORT_SYMBOL(xlink_get_device_mode);

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
					pr_err(KERN_DEBUG "%s:xlink_disconnect - dispatcher stop failed\n", __func__);
					return;
				}
			}
			// deinitialize multiplexer connection
			rc = xlink_multiplexer_disconnect(xlink->links[i].id);
			if (rc) {
				pr_err(KERN_INFO "%s:xlink_disconnect - multiplexer disconnect failed\n", __func__);
				return;
			}
			xlink->links[i].handle.sw_device_id = XLINK_INVALID_SW_DEVICE_ID;
			xlink->nmb_connected_links--;
			break;
		}
	}
	mutex_unlock(&xlink->lock);
}

static int xlink_device_event_handler(uint32_t sw_device_id, uint32_t event_type)
{
	struct event_info *events = NULL;
	xlink_device_event_cb event_cb;
	char event_attr[50];

	mutex_lock(&dev_event_lock);
	// find sw_device_id, event_type in list
	list_for_each_entry(events, &ev_info.list, list) {
		if (events->sw_device_id == sw_device_id && events->event_type == event_type) {
			event_cb = events->event_notif_fn;
			if (events->user_flag) {
				switch (events->event_type) {
				case 0:
					event0.value = events->event_type;
					event0.sw_dev_id = sw_device_id;
					sprintf(event_attr, "event%d", events->event_type);
					break;
				case 1:
					event1.value = events->event_type;
					event1.sw_dev_id = sw_device_id;
					sprintf(event_attr, "event%d", events->event_type);
					break;
				case 2:
					event2.value = events->event_type;
					event2.sw_dev_id = sw_device_id;
					sprintf(event_attr, "event%d", events->event_type);
					break;
				case 3:
					event3.value = events->event_type;
					event3.sw_dev_id = sw_device_id;
					sprintf(event_attr, "event%d", events->event_type);
					break;
				}
				pr_info("Event raised for sw_device_id : 0x%x event type %d\n", events->sw_device_id, events->event_type);
				pr_info("sysfs_notify %s\n", event_attr);
				sysfs_notify(ev_kobj, NULL, event_attr);
			} else {
				if (event_cb)
					event_cb(sw_device_id, event_type);
				else {
					pr_info("No callback found for sw_device_id : 0x%x event type %d\n", sw_device_id, event_type);
					mutex_unlock(&dev_event_lock);
					return X_LINK_ERROR;
				}
			}
		}
	}
	mutex_unlock(&dev_event_lock);
	if (GET_INTERFACE_FROM_SW_DEVICE_ID(sw_device_id) ==
			SW_DEVICE_ID_PCIE_INTERFACE) {
		switch (event_type) {
		case 0:
			xlink_device_cleanup(sw_device_id);
			break;
		case 1: /* TODO: add if anything required for this state */
			break;
		}
	}

return X_LINK_SUCCESS;
}

static bool event_registered(uint32_t sw_dev_id, uint32_t event,
		uint32_t user_flag)
{
	struct event_info *events = NULL;

	list_for_each_entry(events, &ev_info.list, list) {
		if (events->sw_device_id == sw_dev_id &&
				events->event_type == event &&
				events->user_flag == user_flag) {
			return true;
		}
	}
return false;
}

static enum xlink_error xlink_register_device_event_user(
		struct xlink_handle *handle, uint32_t *event_list, uint32_t num_events,
		xlink_device_event_cb event_notif_fn)
{
	enum xlink_error rc = 0;
	rc = do_xlink_register_device_event(handle, event_list, num_events,
			event_notif_fn, 1);
	return rc;
}

enum xlink_error xlink_register_device_event(struct xlink_handle *handle,
		uint32_t *event_list, uint32_t num_events,
		xlink_device_event_cb event_notif_fn)
{
	enum xlink_error rc = 0;
	rc = do_xlink_register_device_event(handle, event_list, num_events,
			event_notif_fn, 0);
	return rc;
}
EXPORT_SYMBOL(xlink_register_device_event);

static enum xlink_error do_xlink_register_device_event(
		struct xlink_handle *handle, uint32_t *event_list, uint32_t num_events,
		xlink_device_event_cb event_notif_fn, uint32_t user_flag)
{
	int i = 0;
	uint32_t interface = 0;
	uint32_t event;
	struct event_info *events = NULL;

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
				pr_info("xlink-core: Event 0x%x - %d "
						"already registered\n", handle->sw_device_id, event);
				continue;
			}
		}
		pr_info("xlink-core:Events : sw_device_id 0x%x"
				" event %d fn %p user_flag %d\n",
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
enum xlink_error xlink_unregister_device_event(struct xlink_handle *handle,
		uint32_t *event_list, uint32_t num_events)
{
	int i = 0;
	int count = 0;
	uint32_t interface = 0;
	struct event_info *events = NULL;
	int found = 0;

	for (i = 0; i < num_events; i++) {
		list_for_each_entry(events, &ev_info.list, list) {
			if (events->sw_device_id == handle->sw_device_id &&
					events->event_type == event_list[i]) {
				found = 1;
				break;
			}
		}
		if (!events || !found) {
			return X_LINK_ERROR;
		}
		pr_info("removing event %d for sw_device_id 0x%x\n",
				events->event_type, events->sw_device_id);
		list_del(&events->list);
		kfree(events);
	}
	// check if any events left for this sw_device_id
	// are still registered ( in list )
	list_for_each_entry(events, &ev_info.list, list) {
		if (events->sw_device_id == handle->sw_device_id) {
			count++;
			break;
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
EXPORT_SYMBOL(xlink_unregister_device_event);

/* Device tree driver match. */
static const struct of_device_id kmb_xlink_of_match[] = {
	{
		.compatible = "intel, keembay-xlink",
	},
	{}
};

/* The xlink driver is a platform device. */
static struct platform_driver kmb_xlink_driver = {
	.probe = kmb_xlink_probe,
	.remove = kmb_xlink_remove,
	.driver = {
			.name = DRV_NAME,
			.of_match_table = kmb_xlink_of_match,
		},
};

/*
 * The remote host system will need to create an xlink platform
 * device for the platform driver to match with
 */
#ifndef CONFIG_XLINK_LOCAL_HOST
static struct platform_device pdev;
void kmb_xlink_release(struct device *dev) { return; }
#endif

static int kmb_xlink_init(void)
{
	int rc = 0;
	rc = platform_driver_register(&kmb_xlink_driver);
#ifndef CONFIG_XLINK_LOCAL_HOST
	pdev.dev.release = kmb_xlink_release;
	pdev.name = DRV_NAME;
	pdev.id = -1;
	if (!rc) {
		rc = platform_device_register(&pdev);
		if (rc) {
			platform_driver_unregister(&kmb_xlink_driver);
		}
	}
#endif
	return rc;
}
module_init(kmb_xlink_init);

static void kmb_xlink_exit(void)
{
#if !defined(CONFIG_XLINK_LOCAL_HOST)
	int rc, idx;
	uint32_t sw_device_id_list[XLINK_MAX_DEVICE_LIST_SIZE];
	uint32_t interface_nmb_devices = 0;

	/*
	 * Unregister PCIE events
	 */
	rc = xlink_platform_get_device_list(PCIE_INTERFACE,
			sw_device_id_list,
			&interface_nmb_devices);
	for (idx = 0; idx < interface_nmb_devices; idx++) {
		xlink_platform_unregister_for_events(PCIE_INTERFACE,
				sw_device_id_list[idx]);
	}
#endif /* !CONFIG_XLINK_LOCAL_HOST */


#ifndef CONFIG_XLINK_LOCAL_HOST
	platform_device_unregister(&pdev);
#endif
	platform_driver_unregister(&kmb_xlink_driver);
}
module_exit(kmb_xlink_exit);

MODULE_DESCRIPTION("KeemBay xlink Kernel Driver");
MODULE_AUTHOR("Seamus Kelly <seamus.kelly@intel.com>");
MODULE_LICENSE("GPL v2");
