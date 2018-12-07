/*
 * Intel HDCP Virtio Linux driver
 * Copyright (c) 2017-2018, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

/*
 * HDCP Virtualization
 *
 *              +---------------+
 *              |               |
 *   +----------+----------+    |    +------------------+
 *   |       ACRN DM       |    |    |     Media APP    |
 *   | +-----------------+ |    |    +------------------+
 *   | |  HDCP Backend   | |    |              |
 *   | +-----------------+ |    |    +------------------+
 *   +---------------------+    |    |  HDCP Libraries  |
 *              |               |    +------------------+
 *              |               |              |
 *     +------------------+     |    +------------------+
 *     | HDCP SOS Daemon  |     |    | HDCP UOS Daemon  |
 *     +------------------+     |    +------------------+
 *                              |
 *    Service OS User Space     |     User OS User Space
 *                              |
 *  --------------------------  |  ---------------------------
 *                              |
 *   Service OS Kernel Space    |    User OS Kernel Space
 *                              |
 *     +------------------+     |    +------------------+
 *     | i915 HDCP Driver |     |    |  HDCP Front End  |
 *     +------------------+     |    +------------------+
 *                              |             |
 *                              +-------------+
 *
 * Above diagram illustrates the HDCP architecture in ACRN. In SOS, HDCP
 * library being used by media app. In UOS, HDCP Daemon gets the HDCP
 * request by open/read/write /dev/hdcp0 which is created by HDCP
 * frontend, instead of accessing GPU. Then the HDCP frontend sends the
 * requests to the HDCP backend thru virtio mechanism. HDCP backend talks to
 * HDCP SOS daemon that will ask HDCP Kernel Driver to execute the requsted
 * operation.
 *
 */

#include <linux/err.h>
#include <linux/scatterlist.h>
#include <linux/spinlock.h>
#include <linux/virtio.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/pm_runtime.h>
#include <linux/virtio_config.h>
#include <linux/virtio_ids.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>

static DEFINE_IDA(hdcp_index_ida);

#define HDCP_MSG_SIZE 72
#define HDCP_MAX_DEVS 1

struct hdcp_msg {
	uint32_t bytes[HDCP_MSG_SIZE];
};

struct virthdcp_info {
	struct virtio_device *vdev;

	dev_t dev;
	struct cdev cdev;
	struct class *class;

	struct virtqueue *msg_vq;

	struct completion have_data;
	char name[25];
	unsigned int data_avail;
	struct hdcp_msg *buf;
	int index;
	bool busy;
	bool hdcp_dev_register_done;
};


static void hdcp_send_done(struct virtqueue *vq)
{
	struct virthdcp_info *vi = vq->vdev->priv;

	/* We can get spurious callbacks, e.g. shared IRQs + virtio_pci. */
	if (!virtqueue_get_buf(vi->msg_vq, &vi->data_avail))
		return;

	complete(&vi->have_data);
}

/* The host will fill buf with hdcp message */
static void register_buf(struct virthdcp_info *vi, u8 *buf, size_t size)
{
	struct scatterlist sg;

	sg_init_one(&sg, buf, size);

	/* There should always be room for one buffer. */
	virtqueue_add_outbuf(vi->msg_vq, &sg, 1, buf, GFP_KERNEL);

	virtqueue_kick(vi->msg_vq);
}

static int virtio_close(struct inode *inode, struct file *filp)
{
	/* Initialize hdcp instance with device params from inode */
	struct virthdcp_info *vi;

	vi = container_of(inode->i_cdev, struct virthdcp_info, cdev);

	if (!vi)
		return -ENODEV;

	filp->private_data = NULL;

	return 0;
}

static int virtio_open(struct inode *inode, struct file *filp)
{
	/* Initialize hdcp instance with device params from inode */
	struct virthdcp_info *vi;

	vi = container_of(inode->i_cdev, struct virthdcp_info, cdev);

	if (!vi)
		return -ENODEV;

	filp->private_data = vi;

	return 0;
}

static ssize_t virtio_write(struct file *filp, const char __user *ubuf,
				size_t length, loff_t *offset)
{
	int ret;
	struct virthdcp_info *vi;

	vi = (struct virthdcp_info *)filp->private_data;
	if (!vi)
		return -ENODEV;

	ret = copy_from_user(vi->buf, ubuf, sizeof(struct hdcp_msg));
	if (ret < 0)
		return -EFAULT;

	if (!vi->busy) {
		vi->busy = true;
		register_buf(vi, (u8 *)(vi->buf), sizeof(struct hdcp_msg));
	}

	ret = wait_for_completion_killable(&vi->have_data);
	if (ret < 0)
		return ret;

	vi->busy = false;

	return sizeof(struct hdcp_msg);
}

static ssize_t virtio_read(struct file *filp, char __user  *ubuf,
				size_t size, loff_t *ppos)
{
	int ret;
	struct virthdcp_info *vi;

	vi = (struct virthdcp_info *)filp->private_data;

	ret = copy_to_user(ubuf, vi->buf, sizeof(struct hdcp_msg));
	if (ret < 0)
		return -EFAULT;

	return vi->data_avail;
}

static const struct file_operations hdcp_fops = {
	.owner		= THIS_MODULE,
	.read		= virtio_read,
	.write		= virtio_write,
	.open		= virtio_open,
	.release	= virtio_close,
};

/* For now we just need one virt queue for send and receive message
 * from back end driver */
static int virthdcp_init_vqs(struct virthdcp_info *vi)
{
	int err;
	struct virtqueue *vqs[1];
	vq_callback_t *cbs[] = { hdcp_send_done };
	const char *names[] = { "hdcp_msg_virtqueue" };

	err = virtio_find_vqs(vi->vdev, 1, vqs, cbs, names, NULL);
	if (err)
		return err;

	vi->msg_vq = vqs[0];

	return 0;
}

static int probe_common(struct virtio_device *vdev)
{
	int err, index;
	struct virthdcp_info *vi = NULL;

	vi = kzalloc(sizeof(struct virthdcp_info), GFP_KERNEL);
	if (!vi)
		return -ENOMEM;

	vi->buf = kmalloc(sizeof(struct hdcp_msg), GFP_KERNEL);
	if (!vi->buf) {
		err = -ENOMEM;
		goto err_alloc_buf;
	}

	vi->index = index = ida_simple_get(&hdcp_index_ida, 0, 0, GFP_KERNEL);
	if (index < 0) {
		err = index;
		goto err_ida;
	}

	sprintf(vi->name, "virtio_hdcp.%d", index);
	init_completion(&vi->have_data);

	vi->vdev = vdev;
	vdev->priv = vi;

	err = virthdcp_init_vqs(vi);
	if (err)
		goto err_find_vqs;

	return 0;

err_find_vqs:
	ida_simple_remove(&hdcp_index_ida, index);
err_ida:
	kfree(vi->buf);
err_alloc_buf:
	kfree(vi);

	return err;
}

static void remove_common(struct virtio_device *vdev)
{
	struct virthdcp_info *vi = vdev->priv;

	cdev_del(&vi->cdev);
	device_destroy(vi->class, vi->dev);
	class_destroy(vi->class);
	unregister_chrdev_region(vi->dev, HDCP_MAX_DEVS);

	vi->data_avail = 0;
	complete(&vi->have_data);
	vdev->config->reset(vdev);
	vi->busy = false;

	vdev->config->del_vqs(vdev);
	ida_simple_remove(&hdcp_index_ida, vi->index);
	kfree(vi->buf);
	kfree(vi);
}

static int virthdcp_probe(struct virtio_device *vdev)
{
	return probe_common(vdev);
}

static void virthdcp_remove(struct virtio_device *vdev)
{
	remove_common(vdev);
}

static void virthdcp_scan(struct virtio_device *vdev)
{
	struct virthdcp_info *vi = vdev->priv;
	int ret;
	struct device *hdcp_device;
	struct class *hdcp_class;
	dev_t hdcp_dev;

	/* Create device class for this device */
	hdcp_class = class_create(THIS_MODULE, "hdcp");
	if (!hdcp_class) {
		dev_err(&vdev->dev, "Failed to register hdcp device class\n");
		return;
	}

	/* Allocate char device for this hdcp driver */
	ret = alloc_chrdev_region(&hdcp_dev, 0, HDCP_MAX_DEVS, "hdcp");
	if (ret < 0) {
		dev_err(&vdev->dev, "Error allocating char device\n");
		goto err_alloc_chrdev;
		return;
	}

	dev_set_drvdata(&vdev->dev, vi);

	/* Initialize character device */
	cdev_init(&vi->cdev, &hdcp_fops);
	vi->cdev.owner = THIS_MODULE;

	ret = cdev_add(&vi->cdev, hdcp_dev, 1);
	if (ret < 0) {
		dev_err(&vdev->dev, "chardev registration failed\n");
		goto err_add_cdev;
	}

	/* Create device node */
	hdcp_device = device_create(hdcp_class, &vdev->dev, hdcp_dev,
				NULL, "hdcp%d", MINOR(hdcp_dev));
	if (!hdcp_device) {
		dev_err(&vdev->dev, "Cannot create device file.\n");
		goto err_create_device;
	}

	vi->dev = hdcp_dev;
	vi->class = hdcp_class;
	vi->hdcp_dev_register_done = true;
	return;

err_create_device:
	cdev_del(&vi->cdev);
err_add_cdev:
	unregister_chrdev_region(hdcp_dev, HDCP_MAX_DEVS);
err_alloc_chrdev:
	class_destroy(hdcp_class);

	vi->hdcp_dev_register_done = false;
}

#ifdef CONFIG_PM_SLEEP
static int virthdcp_freeze(struct virtio_device *vdev)
{
	remove_common(vdev);
	return 0;
}

static int virthdcp_restore(struct virtio_device *vdev)
{
	return probe_common(vdev);
}
#endif

static struct virtio_device_id id_table[] = {
	{ VIRTIO_ID_HDCP, VIRTIO_DEV_ANY_ID },
	{ 0 },
};

static struct virtio_driver virtio_hdcp_driver = {
	.driver.name =	KBUILD_MODNAME,
	.driver.owner =	THIS_MODULE,
	.id_table =	id_table,
	.probe =	virthdcp_probe,
	.remove =	virthdcp_remove,
	.scan =		virthdcp_scan,
#ifdef CONFIG_PM_SLEEP
	.freeze =	virthdcp_freeze,
	.restore =	virthdcp_restore,
#endif
};

module_virtio_driver(virtio_hdcp_driver);
MODULE_DEVICE_TABLE(virtio, id_table);
MODULE_DESCRIPTION("Virtio HDCP Front driver");
MODULE_LICENSE("GPL");
