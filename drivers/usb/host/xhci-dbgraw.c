// SPDX-License-Identifier: GPL-2.0+
/**
 * Raw DbC for xHCI debug capability
 *
 * Copyright (C) 2019 Intel Corporation
 *
 * Author: Rajaram Regupathy <rajaram.regupathy@intel.com>
 * Author: Abhilash K V <abhilash.k.v@intel.com>
 * Author: Prabhat Chand Pandey <prabhat.chand.pandey@intel.com>
 */

#include <linux/idr.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/sizes.h>
#include <linux/slab.h>

#include "xhci.h"
#include "xhci-dbgcap.h"

#define DBC_XHCI_MINORS     8
#define DBC_STR_FUNC_RAW    "RAW"
#define DBC_RAW_BULK_BUFFER_SIZE  (SZ_64K)

static DEFINE_IDR(dbc_minors);

struct dbc_dev {
	struct mutex dev_excl;
	struct mutex read_excl;
	struct mutex write_excl;

	wait_queue_head_t read_wq;
	wait_queue_head_t write_wq;

	int error;
	bool in_use;
	char name[16];
	struct xhci_dbc *dbc;
	struct miscdevice misc_dev;
};

static void xhci_dbc_free_req(struct dbc_ep *dep, struct dbc_request *req)
{
	kfree(req->buf);
	dbc_free_request(dep, req);
}

struct dbc_request *xhci_dbc_alloc_requests(struct dbc_ep *dep,
		void (*fn)(struct xhci_hcd *, struct dbc_request *))
{
	struct dbc_request *req;

	req = dbc_alloc_request(dep, GFP_KERNEL);
	if (!req)
		return req;

	req->length = DBC_RAW_BULK_BUFFER_SIZE;
	req->buf = kmalloc(req->length, GFP_KERNEL);
	if (!req->buf)
		xhci_dbc_free_req(dep, req);

	req->complete = fn;

	return req;
}

static void dbc_complete_in(struct xhci_hcd *xhci,
				struct dbc_request *req)
{
	struct xhci_dbc *dbc = (struct xhci_dbc *) xhci->dbc;
	struct dbc_dev *dev = (struct dbc_dev *) dbc->func_priv;

	if (req->status)
		dev->error = req->status;

	wake_up(&dev->write_wq);
}

static void dbc_complete_out(struct xhci_hcd *xhci,
				struct dbc_request *req)
{
	struct xhci_dbc *dbc = (struct xhci_dbc *) xhci->dbc;
	struct dbc_dev *dev = (struct dbc_dev *) dbc->func_priv;

	if (req->status)
		dev->error = req->status;

	wake_up(&dev->read_wq);
}

static ssize_t dbc_read(struct file *fp, char __user *buf,
				size_t count, loff_t *pos)
{
	int status = 0;
	struct dbc_dev *dev = (struct dbc_dev *) fp->private_data;
	struct xhci_dbc   *dbc = dev->dbc;
	struct dbc_request *req;
	struct dbc_port   *port = &dbc->port;
	int r = count, xfer;
	int ret;

	if (dbc->state != DS_CONFIGURED)
		return -EAGAIN;

	port->in = get_in_ep(dbc->xhci);

	mutex_lock(&dev->read_excl);

	req = xhci_dbc_alloc_requests(port->in, dbc_complete_out);
	if (!req) {
		r = -ENOMEM;
		goto alloc_fail;
	}

	req->actual = 0;

	xfer = min_t(size_t, count, DBC_RAW_BULK_BUFFER_SIZE);
	req->length = xfer;

	status = dbc_ep_queue(port->in, req, GFP_ATOMIC);
	if (status) {
		dev->error = status;
		r = status;
		goto request_fail;
	}

	ret = wait_event_interruptible(dev->read_wq,
			(req->status != -EINPROGRESS));

	if (ret < 0) {
		r = ret;
		goto request_fail;
	}

	if (dev->error) {
		r = dev->error;
		goto request_fail;
	}

	xfer = (req->actual < count) ? req->actual : count;
	if (!req->actual) {
		r = 0;
	} else {
		r = copy_to_user(buf, req->buf, xfer);
		if (!r)
			r = xfer;
	}

request_fail:
	xhci_dbc_free_req(port->in, req);
alloc_fail:
	mutex_unlock(&dev->read_excl);

	return r;
}

static ssize_t dbc_write(struct file *fp, const char __user *buf,
				size_t count, loff_t *pos)
{
	int status = 0;
	struct dbc_dev *dev = (struct dbc_dev *) fp->private_data;
	struct xhci_dbc *dbc = dev->dbc;
	struct dbc_request *req = 0;
	struct dbc_port   *port = &dbc->port;
	int r = count, xfer;
	int ret;

	if (dbc->state != DS_CONFIGURED)
		return -EAGAIN;

	port->out = get_out_ep(dbc->xhci);

	mutex_lock(&dev->write_excl);

	/* get an idle tx request to use */
	req = xhci_dbc_alloc_requests(port->out, dbc_complete_in);
	if (!req) {
		r = -ENOMEM;
		goto alloc_fail;
	}

	req->actual = 0;
	xfer = min_t(size_t, count, DBC_RAW_BULK_BUFFER_SIZE);

	ret = copy_from_user(req->buf, buf, xfer);
	if (ret) {
		r = ret;
		goto request_fail;
	}
	r = xfer;
	req->length = xfer;
	status = dbc_ep_queue(port->out, req, GFP_ATOMIC);
	if (status) {
		dev->error = status;
		r = status;
		goto request_fail;
	}

	ret = wait_event_interruptible(dev->write_wq,
			(req->status != -EINPROGRESS));
	if (ret < 0)
		r = ret;

request_fail:
	xhci_dbc_free_req(port->out, req);
alloc_fail:
	mutex_unlock(&dev->write_excl);

	return r;
}

static int dbc_open(struct inode *ip, struct file *fp)
{
	struct dbc_dev *dbc_dev;
	struct xhci_dbc *dbc;
	int r = 0;

	dbc_dev = container_of(fp->private_data, struct dbc_dev, misc_dev);

	mutex_lock(&dbc_dev->dev_excl);
	if (dbc_dev->in_use) {
		r = -EBUSY;
		goto err;
	}

	dbc = dbc_dev->dbc;
	if (!dbc) {
		r =  -ENODEV;
		goto err;
	}

	dbc_dev->in_use = true;
	fp->private_data = dbc_dev;

	/* clear the error latch */
	dbc_dev->error = 0;
err:
	mutex_unlock(&dbc_dev->dev_excl);

	return r;
}

static int dbc_release(struct inode *ip, struct file *fp)
{
	struct dbc_dev *dbc_dev = (struct dbc_dev *) fp->private_data;


	mutex_lock(&dbc_dev->dev_excl);
	dbc_dev->in_use = false;
	fp->private_data = NULL;
	mutex_unlock(&dbc_dev->dev_excl);

	return 0;
}

static const struct file_operations dbc_fops = {
	.owner = THIS_MODULE,
	.read = dbc_read,
	.write = dbc_write,
	.open = dbc_open,
	.release = dbc_release,
};

static int dbc_raw_register_device(struct xhci_hcd *xhci)
{
	struct device *devm = xhci->main_hcd->self.controller;
	struct xhci_dbc *dbc = xhci->dbc;
	struct dbc_dev *dev;
	int ret;
	int id;

	dev = devm_kzalloc(devm, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	mutex_init(&dev->dev_excl);
	mutex_init(&dev->read_excl);
	mutex_init(&dev->write_excl);

	init_waitqueue_head(&dev->read_wq);
	init_waitqueue_head(&dev->write_wq);

	id = idr_alloc(&dbc_minors, dbc, 0, DBC_XHCI_MINORS,
			GFP_KERNEL);
	if (id < 0)
		return id;
	snprintf(dev->name, sizeof(dev->name), "dbc_raw%d", id);
	dev->misc_dev.name = dev->name;
	dev->misc_dev.minor = MISC_DYNAMIC_MINOR;

	dev->misc_dev.fops = &dbc_fops;

	ret = misc_register(&dev->misc_dev);
	if (ret) {
		kfree(dev->misc_dev.name);
		pr_err("failed to register misc dev: %d\n", ret);
		goto error;
	}

	dev->dbc = dbc;
	dbc->func_priv = (void *) dev;
	return ret;

error:
	idr_remove(&dbc_minors, id);

	return ret;
}

static void dbc_raw_unregister_device(struct xhci_hcd *xhci)
{
	struct xhci_dbc *dbc = xhci->dbc;
	struct dbc_dev *dbc_dev = (struct dbc_dev *) dbc->func_priv;

	if (dbc_dev) {
		idr_remove(&dbc_minors, dbc_dev->misc_dev.minor);
		misc_deregister(&dbc_dev->misc_dev);
	}
	idr_destroy(&dbc_minors);
}

static int dbc_raw_run(struct xhci_dbc *dbc)
{
	return dbc_raw_register_device(dbc->xhci);
}

static int dbc_raw_stop(struct xhci_dbc *dbc)
{
	dbc_raw_unregister_device(dbc->xhci);
	return 0;
}

static struct dbc_function raw_func = {
	.owner = THIS_MODULE,
	.string = {
		.manufacturer = DBC_STR_MANUFACTURER,
		.product = DBC_STR_PRODUCT,
		.serial = DBC_STR_SERIAL,
	},
	.protocol = DBC_PROTOCOL,
	.vid = DBC_VENDOR_ID,
	.pid = DBC_PRODUCT_ID,
	.device_rev = DBC_DEVICE_REV,
	.func_name = DBC_STR_FUNC_RAW,

	.run = dbc_raw_run,
	.stop = dbc_raw_stop,
};

static int __init xhci_dbc_raw_init(void)
{
	return xhci_dbc_register_function(&raw_func);
}

static void __exit xhci_dbc_raw_fini(void)
{
	xhci_dbc_unregister_function();
}

late_initcall(xhci_dbc_raw_init);
module_exit(xhci_dbc_raw_fini);

MODULE_DESCRIPTION("xHCI DbC raw driver");
MODULE_AUTHOR("Rajaram Regupathy");
MODULE_LICENSE("GPL v2");
