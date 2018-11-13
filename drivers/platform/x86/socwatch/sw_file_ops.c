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
#include <linux/module.h> // try_module_get
#include <linux/fs.h> // inode
#include <linux/device.h> // class_create
#include <linux/cdev.h> // cdev_alloc
#include <linux/version.h> // LINUX_VERSION_CODE
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 12, 0)
#include <asm/uaccess.h> // copy_to_user
#else
#include <linux/uaccess.h> // copy_to_user
#endif // LINUX_VERSION_CODE
#include <linux/wait.h> // wait_event_interruptible
#include <linux/sched.h> // TASK_INTERRUPTIBLE

#include "sw_kernel_defines.h"
#include "sw_types.h"
#include "sw_structs.h"
#include "sw_file_ops.h"
#include "sw_ioctl.h"
#include "sw_output_buffer.h"

/* -------------------------------------------------
 * Compile time constants.
 * -------------------------------------------------
 */
/*
 * Get current command.
 */
#define GET_CMD() ((*s_file_ops->get_current_cmd)())
/*
 * Check if we're currently collecting data.
 */
#define IS_COLLECTING()                                                        \
	({                                                                     \
		sw_driver_collection_cmd_t __cmd = GET_CMD();                  \
		bool __val = (__cmd == SW_DRIVER_START_COLLECTION ||           \
			      __cmd == SW_DRIVER_RESUME_COLLECTION);           \
		__val;                                                         \
	})
/*
 * Check if we're currently paused.
 */
#define IS_SLEEPING()                                                          \
	({                                                                     \
		sw_driver_collection_cmd_t __cmd = GET_CMD();                  \
		bool __val = __cmd == SW_DRIVER_PAUSE_COLLECTION;              \
		__val;                                                         \
	})
/* -------------------------------------------------
 * Typedefs
 * -------------------------------------------------
 */
typedef unsigned long sw_bits_t;

/* -------------------------------------------------
 *  Local function declarations.
 * -------------------------------------------------
 */
static int sw_device_open_i(struct inode *inode, struct file *file);
static int sw_device_release_i(struct inode *inode, struct file *file);
static ssize_t sw_device_read_i(struct file *file, char __user *buffer,
				size_t length, loff_t *offset);
static long sw_device_unlocked_ioctl_i(struct file *filp,
				       unsigned int ioctl_num,
				       unsigned long ioctl_param);
#if defined(CONFIG_COMPAT) && defined(CONFIG_X86_64)
static long sw_device_compat_ioctl_i(struct file *file, unsigned int ioctl_num,
				     unsigned long ioctl_param);
#endif

/*
 * File operations exported by the driver.
 */
static struct file_operations s_fops = {
	.open = &sw_device_open_i,
	.read = &sw_device_read_i,
	.unlocked_ioctl = &sw_device_unlocked_ioctl_i,
#if defined(CONFIG_COMPAT) && defined(CONFIG_X86_64)
	.compat_ioctl = &sw_device_compat_ioctl_i,
#endif // COMPAT && x64
	.release = &sw_device_release_i,
};
/*
 * Character device file MAJOR
 * number -- we're now obtaining
 * this dynamically.
 */
static int apwr_dev_major_num = -1;
/*
 * Variables to create the character device file
 */
static dev_t apwr_dev;
static struct cdev *apwr_cdev;
static struct class *apwr_class;
/*
 * Operations exported by the main driver.
 */
static struct sw_file_ops *s_file_ops;
/*
 * Is the device open right now? Used to prevent
 * concurent access into the same device.
 */
#define DEV_IS_OPEN 0 // see if device is in use
static volatile sw_bits_t dev_status;

/*
 * File operations.
 */
/*
 * Service an "open(...)" call from user-space.
 */
static int sw_device_open_i(struct inode *inode, struct file *file)
{
	/*
	 * We don't want to talk to two processes at the same time
	 */
	if (test_and_set_bit(DEV_IS_OPEN, &dev_status)) {
		// Device is busy
		return -EBUSY;
	}

	if (!try_module_get(THIS_MODULE)) {
		// No such device
		return -ENODEV;
	}
	pw_pr_debug("OK, allowed client open!\n");
	return PW_SUCCESS;
}

/*
 * Service a "close(...)" call from user-space.
 */
static int sw_device_release_i(struct inode *inode, struct file *file)
{
	/*
	 * Did the client just try to zombie us?
	 */
	int retVal = PW_SUCCESS;

	if (IS_COLLECTING()) {
		pw_pr_error(
			"ERROR: Detected ongoing collection on a device release!\n");
		retVal = (*s_file_ops->stop_handler)();
	}
	module_put(THIS_MODULE);
	/*
	 * We're now ready for our next caller
	 */
	clear_bit(DEV_IS_OPEN, &dev_status);
	return retVal;
}

static ssize_t sw_device_read_i(struct file *file, char __user *user_buffer,
				size_t length, loff_t *offset)
{
	size_t bytes_read = 0;
	u32 val = 0;

	if (!user_buffer) {
		pw_pr_error(
			"ERROR: \"read\" called with an empty user_buffer?!\n");
		return -PW_ERROR;
	}
	do {
		val = SW_ALL_WRITES_DONE_MASK;
		if (wait_event_interruptible(
			    sw_reader_queue,
			    (sw_any_seg_full(&val,
					     (*s_file_ops->should_flush)()) ||
			     (!IS_COLLECTING() && !IS_SLEEPING())))) {
			pw_pr_error("wait_event_interruptible error\n");
			return -ERESTARTSYS;
		}
		pw_pr_debug(KERN_INFO "After wait: val = %u\n", val);
	} while (val == SW_NO_DATA_AVAIL_MASK);
	/*
	 * Are we done producing/consuming?
	 */
	if (val == SW_ALL_WRITES_DONE_MASK) {
		return 0; // "0" ==> EOF
	}
	/*
	 * Copy the buffer contents into userspace.
	 */
	bytes_read = sw_consume_data(
		val, user_buffer,
		length); // 'read' returns # of bytes actually read
	if (unlikely(bytes_read == 0)) {
		/* Cannot be EOF since that has already been checked above */
		return -EIO;
	}
	return bytes_read;
}

/*
 * (1) Handle 32b IOCTLs in 32b kernel-space.
 * (2) Handle 64b IOCTLs in 64b kernel-space.
 */
static long sw_device_unlocked_ioctl_i(struct file *filp,
				       unsigned int ioctl_num,
				       unsigned long ioctl_param)
{
	struct sw_driver_ioctl_arg __user *remote_args =
		(struct sw_driver_ioctl_arg __user *)ioctl_param;
	struct sw_driver_ioctl_arg local_args;

	if (copy_from_user(&local_args, remote_args, sizeof(local_args))) {
		pw_pr_error("ERROR copying ioctl args from userspace\n");
		return -PW_ERROR;
	}
	return (*s_file_ops->ioctl_handler)(ioctl_num, &local_args);
};

#if defined(CONFIG_COMPAT) && defined(CONFIG_X86_64)
#include <linux/compat.h>
/*
 * Helper struct for use in translating
 * IOCTLs from 32b user programs in 64b
 * kernels.
 */
#pragma pack(push, 1)
struct sw_driver_ioctl_arg32 {
	pw_s32_t in_len;
	pw_s32_t out_len;
	compat_caddr_t in_arg;
	compat_caddr_t out_arg;
};
#pragma pack(pop)

/*
 * Handle 32b IOCTLs in 64b kernel-space.
 */
static long sw_device_compat_ioctl_i(struct file *file, unsigned int ioctl_num,
				     unsigned long ioctl_param)
{
	struct sw_driver_ioctl_arg32 __user *remote_args32 =
		compat_ptr(ioctl_param);
	struct sw_driver_ioctl_arg local_args;
	u32 data;

	if (get_user(local_args.in_len, &remote_args32->in_len)) {
		return -PW_ERROR;
	}
	if (get_user(local_args.out_len, &remote_args32->out_len)) {
		return -PW_ERROR;
	}
	if (get_user(data, &remote_args32->in_arg)) {
		return -PW_ERROR;
	}
	local_args.in_arg = (char *)(unsigned long)data;
	if (get_user(data, &remote_args32->out_arg)) {
		return -PW_ERROR;
	}
	local_args.out_arg = (char *)(unsigned long)data;
	return (*s_file_ops->ioctl_handler)(ioctl_num, &local_args);
}
#endif

/*
 * Device creation, deletion operations.
 */
int sw_register_dev(struct sw_file_ops *ops)
{
	int ret;
	/*
	 * Ensure we have valid handlers!
	 */
	if (!ops) {
		pw_pr_error("NULL file ops?!\n");
		return -PW_ERROR;
	}

	/*
	 * Create the character device
	 */
	ret = alloc_chrdev_region(&apwr_dev, 0, 1, PW_DEVICE_NAME);
	apwr_dev_major_num = MAJOR(apwr_dev);
	apwr_class = class_create(THIS_MODULE, "apwr");
	if (IS_ERR(apwr_class)) {
		printk(KERN_ERR "Error registering apwr class\n");
	}

	device_create(apwr_class, NULL, apwr_dev, NULL, PW_DEVICE_NAME);
	apwr_cdev = cdev_alloc();
	if (apwr_cdev == NULL) {
		printk("Error allocating character device\n");
		return ret;
	}
	apwr_cdev->owner = THIS_MODULE;
	apwr_cdev->ops = &s_fops;
	if (cdev_add(apwr_cdev, apwr_dev, 1) < 0) {
		printk("Error registering device driver\n");
		return ret;
	}
	s_file_ops = ops;

	return ret;
}

void sw_unregister_dev(void)
{
	/*
	 * Remove the device
	 */
	unregister_chrdev(apwr_dev_major_num, PW_DEVICE_NAME);
	device_destroy(apwr_class, apwr_dev);
	class_destroy(apwr_class);
	unregister_chrdev_region(apwr_dev, 1);
	cdev_del(apwr_cdev);
}
