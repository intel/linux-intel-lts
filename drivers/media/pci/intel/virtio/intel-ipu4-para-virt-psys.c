// SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
/*
 * Copyright (C) 2018 Intel Corporation
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/highmem.h>
#include <linux/init_task.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/poll.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0)
#include <linux/sched.h>
#else
#include <uapi/linux/sched/types.h>
#endif
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 8, 0)
#include <linux/dma-attrs.h>
#else
#include <linux/dma-mapping.h>
#endif

#include "intel-ipu4-para-virt-psys.h"
#include "intel-ipu4-virtio-common.h"
#include "intel-ipu4-virtio-fe-request-queue.h"
#include "intel-ipu4-virtio-fe-payload.h"

#define IPU_PSYS_NUM_DEVICES		4
#define IPU_PSYS_NAME	"intel-ipu4-psys"
static dev_t virt_psys_dev_t;

static DECLARE_BITMAP(virt_psys_devices, IPU_PSYS_NUM_DEVICES);
static DEFINE_MUTEX(psys_mutex);

static struct ipu_psys_capability caps = {
	.version = 1,
	.driver = "ipu-psys",
};

static long ipu_get_manifest(struct ipu_psys_manifest *manifest,
			     struct virt_ipu_psys_fh *fh) {

    return 0;
}

static unsigned int virt_psys_poll(struct file *file,
                          struct poll_table_struct *wait)
{
    unsigned int  res = 0;

    return res;
}
long virt_psys_compat_ioctl32(struct file *file, unsigned int cmd,
                         unsigned long arg)
{
    int err = 0;

    if (err)
	return err;

    return 0;
}
static long virt_psys_ioctl(struct file *file, unsigned int cmd,
                            unsigned long arg)
{
    union {
		struct ipu_psys_buffer buf;
		struct ipu_psys_command cmd;
		struct ipu_psys_event ev;
		struct ipu_psys_capability caps;
		struct ipu_psys_manifest m;
	} karg;

    int err = 0;
    struct virt_ipu_psys_fh *fh = file->private_data;
    void __user *up = (void __user *)arg;
	bool copy = (cmd != IPU_IOC_MAPBUF && cmd != IPU_IOC_UNMAPBUF);

	if (copy) {
		if (_IOC_SIZE(cmd) > sizeof(karg))
			return -ENOTTY;

		if (_IOC_DIR(cmd) & _IOC_WRITE) {
			err = copy_from_user(&karg, up, _IOC_SIZE(cmd));
			if (err)
				return -EFAULT;
		}
	}
	switch (cmd) {
	case IPU_IOC_MAPBUF:
		//err = ipu_psys_mapbuf(arg, fh);
		break;
	case IPU_IOC_UNMAPBUF:
		//err = ipu_psys_unmapbuf(arg, fh);
		break;
	case IPU_IOC_QUERYCAP:
		karg.caps = caps;
		break;
	case IPU_IOC_GETBUF:
		//err = ipu_psys_getbuf(&karg.buf, fh);
		break;
	case IPU_IOC_PUTBUF:
		//err = ipu_psys_putbuf(&karg.buf, fh);
		break;
	case IPU_IOC_QCMD:
		//err = ipu_psys_kcmd_new(&karg.cmd, fh);
		break;
	case IPU_IOC_DQEVENT:
		//err = ipu_ioctl_dqevent(&karg.ev, fh, file->f_flags);
		break;
	case IPU_IOC_GET_MANIFEST:
		err = ipu_get_manifest(&karg.m, fh);
		break;
	default:
		err = -ENOTTY;
		break;
	}
    if (err)
		return err;

    return 0;
}
static int virt_psys_open(struct inode *inode, struct file *file)
{
    struct virt_ipu_psys *psys = inode_to_ipu_psys(inode);
    struct virt_ipu_psys_fh *fh;
    struct ipu4_virtio_req *req;
    struct ipu4_virtio_ctx *fe_ctx = psys->ctx;
    int op[2];
    int rval = 0;

    pr_debug("virt psys open\n");

    fh = kzalloc(sizeof(*fh), GFP_KERNEL);
    if (!fh)
      return -ENOMEM;
    mutex_init(&fh->bs_mutex);

    fh->psys = psys;
    file->private_data = fh;

    req = ipu4_virtio_fe_req_queue_get();
    if (!req) {
	dev_err(&psys->dev, "Virtio Req buffer failed\n");
	return -ENOMEM;
    }
    op[0] = 0;

    intel_ipu4_virtio_create_req(req, IPU4_CMD_DEVICE_OPEN, &op[0]);

    rval = fe_ctx->bknd_ops->send_req(fe_ctx->domid, req, true,
				      IPU_VIRTIO_QUEUE_1);
    if (rval) {
	dev_err(&psys->dev, "Failed to PSYS open virtual device\n");
	ipu4_virtio_fe_req_queue_put(req);
	return rval;
    }
    ipu4_virtio_fe_req_queue_put(req);

    return rval;
}

static int virt_psys_release(struct inode *inode, struct file *file)
{
    int rval = 0;

    return rval;
}
static const struct file_operations virt_psys_fops = {
	.open = virt_psys_open,
	.release = virt_psys_release,
	.unlocked_ioctl = virt_psys_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = virt_psys_compat_ioctl32,
#endif
	.poll = virt_psys_poll,
	.owner = THIS_MODULE,
};

static void virt_psys_dev_release(struct device *dev)
{
}
void virt_psys_exit(void)
{
    unregister_chrdev_region(virt_psys_dev_t, IPU_PSYS_NUM_DEVICES);

    pr_notice("Virtual psys device unregistered\n");

}

int virt_psys_init(struct ipu4_virtio_ctx *fe_ctx)
{
    struct virt_ipu_psys *psys;
    unsigned int minor;
    int rval = -E2BIG;

    if (!fe_ctx)
    	return -ENOMEM;

    rval = alloc_chrdev_region(&virt_psys_dev_t, 0,
			IPU_PSYS_NUM_DEVICES, IPU_PSYS_NAME);
    if (rval) {
	pr_err("can't alloc psys chrdev region (%d)\n", rval);
		return rval;
    }
    mutex_lock(&psys_mutex);

    minor = find_next_zero_bit(virt_psys_devices, IPU_PSYS_NUM_DEVICES, 0);
    if (minor == IPU_PSYS_NUM_DEVICES) {
      pr_err("too many devices\n");
      goto out_unlock;
    }

    psys = kzalloc(sizeof(*psys), GFP_KERNEL);
    if (!psys) {
      rval = -ENOMEM;
      goto out_unlock;
    }

    cdev_init(&psys->cdev, &virt_psys_fops);
    psys->cdev.owner = virt_psys_fops.owner;

    rval = cdev_add(&psys->cdev, MKDEV(MAJOR(virt_psys_dev_t), minor), 1);
    if (rval) {
      pr_err("cdev_add failed (%d)\n", rval);
      goto out_unlock;
    }

    set_bit(minor, virt_psys_devices);

    mutex_init(&psys->mutex);
    psys->dev.devt = MKDEV(MAJOR(virt_psys_dev_t), minor);
    psys->dev.release = virt_psys_dev_release;
    dev_set_name(&psys->dev, "ipu-psys%d", minor);
    rval = device_register(&psys->dev);
    if (rval < 0) {
        dev_err(&psys->dev, "psys device_register failed\n");
        goto out_mutex_destroy;
    }
    /* Add the hw stepping information to caps */
    strlcpy(caps.dev_model, IPU_MEDIA_DEV_MODEL_NAME,
	    sizeof(caps.dev_model));

    psys->ctx = fe_ctx;

    pr_info("psys probe minor: %d\n", minor);

out_mutex_destroy:
    mutex_destroy(&psys->mutex);
    cdev_del(&psys->cdev);
out_unlock:
    mutex_unlock (&psys_mutex);
    return rval;
}

