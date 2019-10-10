// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright(c) 2016-2019, Intel Corporation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/cdev.h>
#include <linux/printk.h>
#include <linux/mei_cl_bus.h>
#include <linux/notifier.h>

#include "dal_dev.h"
#include "dal_cdev.h"

/* KDI user space devices major and minor numbers */
static dev_t dal_devt;

/**
 * dal_dev_open - dal cdev open function
 *
 * @inode: pointer to inode structure
 * @fp: pointer to file structure
 *
 * Return: 0 on success
 *         <0 on failure
 */
static int dal_dev_open(struct inode *inode, struct file *fp)
{
	int ret;
	struct dal_device *ddev;

	ddev = container_of(inode->i_cdev, struct dal_device, cdev);
	if (!ddev)
		return -ENODEV;

	/* single open */
	if (test_and_set_bit(DAL_DEV_OPENED, &ddev->status))
		return -EBUSY;

	ret = dal_dc_setup(ddev, DAL_INTF_CDEV);
	if (ret)
		goto err;

	fp->private_data = ddev->clients[DAL_INTF_CDEV];

	return nonseekable_open(inode, fp);

err:
	clear_bit(DAL_DEV_OPENED, &ddev->status);
	return ret;
}

/**
 * dal_dev_release - dal cdev release function
 *
 * @inode: pointer to inode structure
 * @fp: pointer to file structure
 *
 * Return: 0 on success
 *         <0 on failure
 */
static int dal_dev_release(struct inode *inode, struct file *fp)
{
	struct dal_client *dc = fp->private_data;
	struct dal_device *ddev = dc->ddev;

	if (mutex_lock_interruptible(&ddev->context_lock)) {
		dev_dbg(&ddev->dev, "signal interrupted\n");
		return -ERESTARTSYS;
	}

	dal_dc_destroy(ddev, dc->intf);

	mutex_unlock(&ddev->context_lock);

	clear_bit(DAL_DEV_OPENED, &ddev->status);

	return 0;
}

/**
 * dal_dev_read - dal cdev read function
 *
 * @fp: pointer to file structure
 * @buf: pointer to user buffer
 * @count: buffer length
 * @off: data offset in buffer
 *
 * Return: >=0 data length on success
 *         <0 on failure
 */
static ssize_t dal_dev_read(struct file *fp, char __user *buf,
			    size_t count, loff_t *off)
{
	struct dal_client *dc = fp->private_data;
	struct dal_device *ddev = dc->ddev;
	int ret;
	size_t r_len, len;
	unsigned int copied;

	if (!buf)
		return -EINVAL;

	if (kfifo_is_empty(&dc->read_queue))
		if (fp->f_flags & O_NONBLOCK)
			return -EAGAIN;

	ret = dal_wait_for_read(dc);
	if (ret)
		return ret;

	r_len = kfifo_out(&dc->read_queue, &len, sizeof(len));
	if (r_len != sizeof(len) || len > count) {
		dev_dbg(&ddev->dev, "could not copy buffer: src size = %zd, dest size = %zu\n",
			len, count);
		return -EMSGSIZE;
	}

	/**
	 * count is the user buffer size, len is the msg size,
	 * if we reach here then len <= count,
	 * we can copy the whole msg to the user because his
	 * buffer is big enough
	 */
	ret = kfifo_to_user(&dc->read_queue, buf, len, &copied);
	if (ret) {
		dev_dbg(&ddev->dev, "copy_to_user() failed\n");
		return ret;
	}

	return copied;
}

/**
 * dal_dev_write - dal cdev write function
 *
 * @fp: pointer to file structure
 * @buff: pointer to user buffer
 * @count: buffer length
 * @off: data offset in buffer
 *
 * Return: >=0 data length on success
 *         <0 on failure
 */
static ssize_t dal_dev_write(struct file *fp, const char __user *buff,
			     size_t count, loff_t *off)
{
	struct dal_device *ddev;
	struct dal_client *dc = fp->private_data;
	void *data;
	int ret;

	ddev = dc->ddev;

	if (count > DAL_MAX_BUFFER_SIZE) {
		dev_dbg(&ddev->dev, "count is too big, count = %zu\n", count);
		return -EMSGSIZE;
	}

	if (count == 0)
		return 0;

	if (!buff)
		return -EINVAL;

	data =  memdup_user(buff, count);
	if (IS_ERR(data))
		return PTR_ERR(data);

	ret = dal_write(dc, data, count, 0);

	kfree(data);

	return ret;
}

static const struct file_operations mei_dal_fops = {
	.owner    = THIS_MODULE,
	.open     = dal_dev_open,
	.release  = dal_dev_release,
	.read     = dal_dev_read,
	.write    = dal_dev_write,
	.llseek   = no_llseek,
};

/**
 * dal_dev_del - delete dal cdev
 *
 * @ddev: dal device
 */
void dal_dev_del(struct dal_device *ddev)
{
	cdev_del(&ddev->cdev);
}

/**
 * dal_dev_setup - initialize dal cdev
 *
 * @ddev: dal device
 */
void dal_dev_setup(struct dal_device *ddev)
{
	dev_t devno;

	cdev_init(&ddev->cdev, &mei_dal_fops);
	devno = MKDEV(MAJOR(dal_devt), ddev->device_id);
	ddev->cdev.owner = THIS_MODULE;
	ddev->dev.devt = devno;
	ddev->cdev.kobj.parent = &ddev->dev.kobj;
}

/**
 * dal_dev_add - add dal cdev
 *
 * @ddev: dal device
 *
 * Return: 0 on success
 *         <0 on failure
 */
int dal_dev_add(struct dal_device *ddev)
{
	return cdev_add(&ddev->cdev, ddev->dev.devt, 1);
}

/**
 * dal_dev_init - allocate dev_t number
 *
 * Return: 0 on success
 *         <0 on failure
 */
int __init dal_dev_init(void)
{
	return alloc_chrdev_region(&dal_devt, 0, DAL_MEI_DEVICE_MAX, "dal");
}

/**
 * dal_dev_exit - unregister allocated dev_t number
 */
void dal_dev_exit(void)
{
	unregister_chrdev_region(dal_devt, DAL_MEI_DEVICE_MAX);
}
