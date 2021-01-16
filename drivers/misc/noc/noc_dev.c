// SPDX-License-Identifier: GPL-2.0-only
/*
 * NOC Device Kernel module.
 *
 * Copyright (C) 2020 Intel Corporation
 */

#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/ioctl.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/noc_uapi.h>
#include "noc_driver.h"

static long noc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int rc = 0;
	struct flexnoc_setup setup_data = {0};
	struct flexnoc_probestart probe_data = {0};
	struct flexnoc_countercapture capture_data = {0};

	if ((int32_t *)arg == NULL) {
		pr_err("NOC: Null pointer from user\n");
		return -EINVAL;
	}

	switch (cmd) {
	case NOC_SETUP:
		if (copy_from_user(&setup_data,
			(int32_t *)arg, sizeof(struct flexnoc_setup))) {
			return -EFAULT;
		}
		/* noc-setup */
		rc =  flex_noc_setup(setup_data.noc_type, setup_data.counter,
						setup_data.traceport);
		setup_data.ret_id = rc;

		if (copy_to_user((struct flexnoc_setup *) arg,
			&setup_data, sizeof(struct flexnoc_setup))) {
			return -EFAULT;
		}
	break;

	case NOC_PROBE_START:
		if (copy_from_user(&probe_data, (int32_t *)arg,
			sizeof(struct flexnoc_probestart))) {
			return -EFAULT;
		}
		/* probe-start */
		rc = flexnoc_probe_start(probe_data.noc_type,
			probe_data.captime);
		probe_data.ret_id = rc;

		if (copy_to_user((struct flexnoc_probestart *) arg,
			&probe_data, sizeof(struct flexnoc_probestart))) {
			return -EFAULT;
		}
	break;

	case NOC_COUNTER_CAPTURE:
		if (copy_from_user(&capture_data, (int32_t *)arg,
			sizeof(struct flexnoc_countercapture))) {
			return -EFAULT;
		}
		rc = flexnoc_counter_capture(capture_data.noc_type,
			capture_data.counter, &capture_data.bw_res);
		capture_data.ret_id = rc;

		if (copy_to_user((struct flexnoc_countercapture *)arg,
			&capture_data, sizeof(struct flexnoc_countercapture))) {
			return -EFAULT;
		}
	break;

	default:
		break;
	}

	return 0;
}

static const struct file_operations noc_fops = {
	.owner  = THIS_MODULE,
	.unlocked_ioctl = noc_ioctl,
};

int intel_noc_cdev_init(struct noc_device *nocdev)
{
	/*Allocating Major number*/
	if ((alloc_chrdev_region(&nocdev->cdev, 0, 1, "nocdev")) < 0) {
		pr_err("Cannot allocate major number\n");
		return -EINVAL;
	}

	pr_info("Major = %d Minor = %d\n", MAJOR(nocdev->cdev),
			MINOR(nocdev->cdev));

	/*Creating cdev structure*/
	cdev_init(&nocdev->noc_cdev, &noc_fops);

	/*Adding character device to the system*/
	if ((cdev_add(&nocdev->noc_cdev, nocdev->cdev, 1)) < 0) {
		pr_err("Cannot add the device to the system\n");
		goto r_class;
	}

	/*Creating struct class*/
	nocdev->dev_class = class_create(THIS_MODULE, "noc_class");
	if (nocdev->dev_class == NULL) {
		pr_err("Cannot create the struct class\n");
		goto r_class;
	}

	/*Creating device*/
	if ((device_create(nocdev->dev_class, NULL, nocdev->cdev,
		NULL, "noc")) == NULL) {
		pr_err("Cannot create the Device 1\n");
		goto r_device;
	}

	return 0;

r_device:
	class_destroy(nocdev->dev_class);
r_class:
	unregister_chrdev_region(nocdev->cdev, 1);
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(intel_noc_cdev_init);

MODULE_DESCRIPTION("KeemBay NOC Char driver interface");
MODULE_AUTHOR("Sudarshan Ravula <sudarshan.ravula@intel.com>");
MODULE_LICENSE("GPL v2");
