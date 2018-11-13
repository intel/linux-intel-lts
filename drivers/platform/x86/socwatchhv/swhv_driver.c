/*

  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY

  Copyright(c) 2014 - 2018 Intel Corporation.

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  Contact Information:
  SoC Watch Developer Team <socwatchdevelopers@intel.com>
  Intel Corporation,
  1300 S Mopac Expwy,
  Austin, TX 78746

  BSD LICENSE

  Copyright(c) 2014 - 2018 Intel Corporation.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#define MOD_AUTHOR "SoCWatch Team"
#define MOD_DESC "SoCWatch kernel module to communicate with hypervisors"

#include "swhv_defines.h"
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/version.h>
#include <linux/io.h>
#include <linux/uaccess.h>

#include "swhv_driver.h"
#include "swhv_ioctl.h"
#include "swhv_structs.h"
#if HYPERVISOR == MOBILEVISOR
#include "swhv_mobilevisor.h"
#include "swhv_mobilevisor_buffer.h"
#elif HYPERVISOR == ACRN
#include "swhv_acrn.h"
#endif

/* *******************************************
 * Compile-time constants
 * *******************************************
 */
/* *******************************************
 * Local data structures.
 * *******************************************
 */
#if defined(HAVE_COMPAT_IOCTL) && defined(CONFIG_X86_64)
#include <linux/compat.h>
/*
 * Helper struct used to translate IOCTLs
 * from 32b user programs in 64b kernels.
 */
struct spdrv_ioctl_arg32 {
	pw_s32_t in_len;
	pw_s32_t out_len;
	compat_caddr_t in_arg;
	compat_caddr_t out_arg;
};
#endif // COMPAT && x64

static int sp_dev_major_num = -1;
static dev_t sp_dev;
static struct cdev *sp_cdev;
static struct class *sp_class;

/* *******************************************
 * Variables.
 * *******************************************
 */

/* Per-CPU variable containing the currently running vcpu. */
//static DEFINE_PER_CPU(int, curr_vcpu) = 0;

/* *******************************************
 * Function definitions.
 * *******************************************
 */

static long swhv_handle_cmd(u32 __user *remote_cmd)
{
	u32 local_cmd = 0;
	long status = 0;

	if (get_user(local_cmd, remote_cmd)) {
		pw_pr_error("ERROR: couldn't copy in remote command!\n");
		return -1;
	}
	switch (local_cmd) {
	case SWHVDRV_CMD_START:
		pw_pr_debug("RECEIVED CMD START!\n");
		status = swhv_start();
		break;
	case SWHVDRV_CMD_STOP:
		pw_pr_debug("RECEIVED CMD STOP!\n");
		status = swhv_stop();
		break;
	default:
		pw_pr_error(
			"ERROR: invalid command %d passed to the SoFIA driver!\n",
			local_cmd);
		status = -1;
		break;
	}
	return status;
};

long swhv_get_version(u64 __user *remote_args)
{
	u64 local_version = (u64)SWHVDRV_VERSION_MAJOR << 32 |
			    (u64)SWHVDRV_VERSION_MINOR << 16 |
			    (u64)SWHVDRV_VERSION_OTHER;

	return put_user(local_version, remote_args);
};

#if defined(CONFIG_COMPAT) && defined(CONFIG_X86_64)
#define MATCH_IOCTL(num, pred) ((num) == (pred) || (num) == (pred##32))
#else
#define MATCH_IOCTL(num, pred) ((num) == (pred))
#endif

static long handle_ioctl(unsigned int ioctl_num,
			 struct spdrv_ioctl_arg __user *remote_args)
{
	long status = 0;
	struct spdrv_ioctl_arg local_args;

	int local_in_len, local_out_len;

	if (copy_from_user(&local_args, remote_args, sizeof(local_args))) {
		pw_pr_error("ERROR: couldn't copy in remote args!\n");
		return -1;
	}
	pw_pr_debug("Invoking IOCTL!\n");

	local_in_len = local_args.in_len;
	local_out_len = local_args.out_len;

	switch (ioctl_num) {
	case SWHVDRV_OPERATION_CMD:
		status = swhv_handle_cmd((u32 __user *)local_args.in_arg);
		break;

	case SWHVDRV_OPERATION_CONFIGURE:
		pw_pr_debug("Trying to configure driver!\n");
		status = swhv_configure(
			(struct swhv_driver_interface_msg __user *)
				local_args.in_arg,
			local_in_len);
		break;

	case SWHVDRV_OPERATION_VERSION:
		pw_pr_debug("Trying to get driver version!\n");
		status = swhv_get_version((u64 __user *)local_args.out_arg);
		break;

	case SWHVDRV_OPERATION_CLOCK:
		pw_pr_debug("Trying to get hypervisor type!\n");
		status = swhv_get_clock((u32 __user *)local_args.in_arg,
					(u64 __user *)local_args.out_arg);
		break;

	case SWHVDRV_OPERATION_TOPOLOGY:
		pw_pr_debug("Trying to get CPU topology!\n");
		status = swhv_get_topology((u64 __user *)local_args.out_arg);
		break;

	case SWHVDRV_OPERATION_CPUCOUNT:
		pw_pr_debug("Trying to get CPU count!\n");
		status = swhv_get_cpu_count((u32 __user *)local_args.out_arg);
		break;

	case SWHVDRV_OPERATION_HYPERVISOR_TYPE:
		pw_pr_debug("Trying to get hypervisor type!\n");
		status = swhv_get_hypervisor_type(
			(u32 __user *)local_args.out_arg);
		break;

	case SWHVDRV_OPERATION_MSR_READ:
		pw_pr_debug("Trying to do MSR read!\n");
		status = swhv_msr_read((u32 __user *)local_args.in_arg,
				       (u64 __user *)local_args.out_arg);
		break;
	case SWHVDRV_OPERATION_POLL:
		pw_pr_debug("Polling tick!\n");
		status = swhv_collection_poll();
		break;
	}
	return status;
}

static long device_unlocked_ioctl(struct file *filep, unsigned int ioctl_num,
				  unsigned long ioctl_param)
{
	return handle_ioctl(_IOC_NR(ioctl_num),
			    (struct spdrv_ioctl_arg __user *)ioctl_param);
};

#if defined(HAVE_COMPAT_IOCTL) && defined(CONFIG_X86_64)
static long device_compat_ioctl(struct file *file, unsigned int ioctl_num,
				unsigned long ioctl_param)
{
	struct spdrv_ioctl_arg32 __user *remote_args32 =
		compat_ptr(ioctl_param);
	struct spdrv_ioctl_arg __user *remote_args =
		compat_alloc_user_space(sizeof(*remote_args));
	int tmp;
	u32 data;

	if (!remote_args) {
		return -1;
	}
	if (get_user(tmp, &remote_args32->in_len) ||
	    put_user(tmp, &remote_args->in_len)) {
		return -1;
	}
	if (get_user(tmp, &remote_args32->out_len) ||
	    put_user(tmp, &remote_args->out_len)) {
		return -1;
	}
	if (get_user(data, &remote_args32->in_arg) ||
	    put_user(compat_ptr(data), &remote_args->in_arg)) {
		return -1;
	}
	if (get_user(data, &remote_args32->out_arg) ||
	    put_user(compat_ptr(data), &remote_args->out_arg)) {
		return -1;
	}
	return handle_ioctl(_IOC_NR(ioctl_num), remote_args);
};
#endif // COMPAT && x64

static int device_open(struct inode *inode, struct file *file)
{
	return device_open_i(inode, file);
}

static ssize_t
device_read(struct file *file, /* see include/linux/fs.h */
	    char __user *buffer, /* buffer to be filled with data */
	    size_t length, /* length of the buffer */
	    loff_t *offset)
{
	return device_read_i(file, buffer, length, offset);
}

static struct file_operations s_fops = {
	.open = &device_open,
	.read = &device_read,
	.unlocked_ioctl = &device_unlocked_ioctl,
#if defined(HAVE_COMPAT_IOCTL) && defined(CONFIG_X86_64)
	.compat_ioctl = &device_compat_ioctl,
#endif // COMPAT && x64
};

static void cleanup_error(void)
{
	unregister_chrdev(sp_dev_major_num, SWHV_DEVICE_NAME);
	device_destroy(sp_class, sp_dev);
	class_destroy(sp_class);
	unregister_chrdev_region(sp_dev, 1);
	cdev_del(sp_cdev);
}

int __init swhv_load_driver(void)
{
	int error;
	struct device *dev;

	// create the char device "sp"
	alloc_chrdev_region(&sp_dev, 0, 1, SWHV_DEVICE_NAME);
	sp_dev_major_num = MAJOR(sp_dev);
	sp_class = class_create(THIS_MODULE, SWHV_DEVICE_NAME);
	if (IS_ERR(sp_class)) {
		error = PTR_ERR(sp_class);
		pw_pr_error("Error registering sp class\n");
		goto cleanup_return_error;
	}

	dev = device_create(sp_class, NULL, sp_dev, NULL, SWHV_DEVICE_NAME);
	if (dev == NULL) {
		error = PTR_ERR(dev);
		pw_pr_error("Error during call to device_create\n");
		goto cleanup_return_error;
	}

	sp_cdev = cdev_alloc();
	if (sp_cdev == NULL) {
		error = -ENOMEM;
		pw_pr_error("Error allocating character device\n");
		goto cleanup_return_error;
	}
	sp_cdev->owner = THIS_MODULE;
	sp_cdev->ops = &s_fops;
	if (cdev_add(sp_cdev, sp_dev, 1) < 0) {
		error = -1;
		pw_pr_error("Error registering device driver\n");
		goto cleanup_return_error;
	}

	error = swhv_load_driver_i();
	if (error < 0) {
		pw_pr_error("Error initializing device driver\n");
		goto cleanup_return_error;
	}

	return 0;

cleanup_return_error:
	cleanup_error_i();

	// release char device
	cleanup_error();
	return error;
}

static void __exit swhv_unload_driver(void)
{
	swhv_unload_driver_i();

	// release char device
	cleanup_error();
}

module_init(swhv_load_driver);
module_exit(swhv_unload_driver);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR(MOD_AUTHOR);
MODULE_DESCRIPTION(MOD_DESC);
