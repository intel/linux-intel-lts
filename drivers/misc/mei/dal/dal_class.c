// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright(c) 2016-2019 Intel Corporation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>

/*
 * this class contains the 3 mei_cl_device, ivm, sdm, rtm.
 * it is initialized during dal_probe and is used by the kernel space kdi
 * to send/recv data to/from mei.
 *
 * this class must be initialized before the kernel space kdi uses it.
 */
static struct class *dal_class;

/**
 * mei_dal_exit - module exit function
 */
static void __exit mei_dal_exit(void)
{
	class_destroy(dal_class);
}

/**
 * mei_dal_init - module init function
 *
 * Return: 0 on success
 *         <0 on failure
 */
static int __init mei_dal_init(void)
{
	dal_class = class_create(THIS_MODULE, "dal");
	if (IS_ERR(dal_class)) {
		pr_err("couldn't create class\n");
		return PTR_ERR(dal_class);
	}

	return 0;
}

module_init(mei_dal_init);
module_exit(mei_dal_exit);

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("Intel(R) MEI Dynamic Application Loader (DAL)");
MODULE_LICENSE("GPL v2");
