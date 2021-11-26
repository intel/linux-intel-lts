// SPDX-License-Identifier: GPL-2.0-only
/*  Copyright (C) 2020 Intel Corporation
 *
 *  Purpose: Intel Keem Bay NOC bandwidth measurement interface
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/arm-smccc.h>
#include <linux/compiler_types.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/noc_uapi.h>
#include <linux/of_device.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include "keembay-bwmon.h"

/* Filter and counter offset */
static const int f_offset[] = {0x44, 0x80, 0xbc, 0xf8};
static const int c_offset[] = {0x134, 0x148, 0x15c, 0x170};
static struct noc_device noc_dev;

static inline u32 noc_readl(u32 offset)
{
	struct arm_smccc_res res;

	arm_smccc_smc(PLATFORM_SIP_SVC_DSS_NOC_PROBE_READ, offset,
		      0, 0, 0, 0, 0, 0, &res);
	return res.a1;
}

static inline void noc_writel(u32 offset, u32 value)
{
	struct arm_smccc_res res;

	arm_smccc_smc(PLATFORM_SIP_SVC_DSS_NOC_PROBE_WRITE, offset,
		      value, 0, 0, 0, 0, 0, &res);
}

/**
 * flex_noc_setup() - Setup two counters for trace ports
 * @noc: NOC type to setup counters
 * @counter: Counter number to set up counter n and n+1
 * @trace_port: trace port number to setup counters
 *
 *  Returns 0 on successful setup, -EINVAL on failure
 */
int flex_noc_setup(enum noc_ss_type noc, enum noc_counter counter, int trace_port)
{
	int offset;

	if (noc >= NOC_TYPE_MAX || counter >= NOC_COUNTER_MAX)
		return -EINVAL;

	offset = f_offset[counter / 2];

	/* Stop ongoing stats */
	noc_writel(MAINCTL, 0);
	noc_writel(CFGCTL, 0);

	/* Setup trace port and counters port select */
	noc_writel(TRACEPORTSEL, trace_port);
	noc_writel(c_offset[counter] + C_PORTSEL, trace_port);
	noc_writel(c_offset[counter + 1] + C_PORTSEL, trace_port);

	/* Setup counter sources & triggers, Alarm mode - OFF */
	noc_writel(c_offset[counter] + C_SRC, COUNTERS_0_SRC_VAL);
	noc_writel(c_offset[counter] + C_ALARMMODE, COUNTERS_ALARMMODE_VAL);
	noc_writel(c_offset[counter + 1] + C_SRC, COUNTERS_1_SRC_VAL);
	noc_writel(c_offset[counter + 1] + C_ALARMMODE,
		   COUNTERS_ALARMMODE_VAL);

	/* Setup filters - RouteID mask, addr base, window size */
	noc_writel(offset + F_ROUTEIDBASE, 0);
	noc_writel(offset + F_ROUTEIDMASK, 0);
	noc_writel(offset + F_ADDRBASE_LOW, 0);
	noc_writel(offset + F_ADDRBASE_HIGH, 0);
	noc_writel(offset + F_WINDOWSIZE, FILTER_WINDOW_VAL);
	noc_writel(offset + F_OPCODE, FILTER_OPCODE_VAL);
	noc_writel(offset + F_STATUS, FILTER_STATUS_VAL);
	noc_writel(offset + F_LENGTH, FILTER_OPCODE_VAL);

	return 0;
}

/**
 * flexnoc_probe_start() - Start two counters for the NOC probe
 * @noc: NOC type to setup counters
 *
 * This function will start the counters.  When this
 * function returns NOC_PROBE_CAPTURE_STARTED, it is guaranteed that NOC
 * is setup for probing counters.
 *
 *  Returns NOC_PROBE_CAPTURE_STARTED on starting counters or
 *  NOC_PROBE_ERR_INVALID_ARGS on invalid arguments
 */
enum noc_status flexnoc_probe_start(enum noc_ss_type noc)
{
	if (noc >= NOC_TYPE_MAX)
		return NOC_PROBE_ERR_INVALID_ARGS;

	/* Setting up probe */
	noc_writel(MAINCTL, (1 << MAINCTL_STATEN_POS));
	noc_writel(FILTERLUT, 1);
	noc_writel(STATALARMMIN, 0);
	noc_writel(STATALARMMAX, 1);
	noc_writel(STATALARMEN, 1);
	noc_writel(MAINCTL, ((1 << MAINCTL_STATEN_POS) |
			     (1 << MAINCTL_ALARM_EN_POS) |
			     (1 << MAINCTL_ALWAYS_CHAINABLE_POS)));
	noc_writel(STATPERIOD, NOC_STATPERIOD_VAL);
	noc_writel(FILTERLUT, 0x00000001);
	noc_writel(CFGCTL, 0x00000001);

	return NOC_PROBE_CAPTURE_STARTED;
}

/**
 * flexnoc_counterp_capture() - Capture the counter statistic values
 * @noc: NOC type to setup counters
 * @counter:  Counter number to capture statistics values for n and n+1
 * @value: statistics values read are returned in this address passed
 *
 * This function will return the statistics value of started counters.
 * When this function returns NOC_PROBE_COMPLETED, it is guaranteed that NOC
 * counters are idle and finished probing.
 * Algo : The values should not returned when counters are active/running.
 * Once the counter is frozen, the values are good to read. There is an
 * iteration logic implemented to check this. An maximum timeout config
 * is provided to for capture timeout - NOC_CAPTURE_TIMEOUT_MSEC
 *
 *  Returns NOC_PROBE_COMPLETED if the counters are stopped or
 *  NOC_PROBE_ERR_IN_PROGRESS if counters are still running
 */
enum noc_status flexnoc_counter_capture(enum noc_ss_type noc,
					enum noc_counter counter, u32  *value)
{
	unsigned long timeout;
	u32 c0_0, c0_1;

	if (noc >= NOC_TYPE_MAX ||
	    counter >= NOC_COUNTER_MAX  ||
	    !value)
		return NOC_PROBE_ERR_INVALID_ARGS;

	timeout = jiffies + msecs_to_jiffies(NOC_CAPTURE_TIMEOUT_MSEC);
	do {
		c0_0 = noc_readl((c_offset[counter] + C_VAL));
		usleep_range(10000, 11000);
		c0_1 = noc_readl((c_offset[counter] + C_VAL));
		/* If mainctrl is zero , return error */
		if (noc_readl(MAINCTL) == 0)
			return NOC_PROBE_ERR_IN_PROGRESS;
		/* If counters are zero, keep reading */
		if (0 == c0_0 && 0 == c0_1) {
			break;
		} else if (c0_0 != c0_1) {
			continue;
		} else {
			/* counters look good break the while */
			break;
		}
	} while (time_before(jiffies, timeout));

	if (c0_0 != c0_1)
		return NOC_PROBE_ERR_IN_PROGRESS;

	c0_0 = noc_readl((c_offset[counter] + C_VAL));
	c0_1 = noc_readl((c_offset[counter + 1] + C_VAL));
	*value = (c0_0 | (c0_1 << 16));

	return NOC_PROBE_COMPLETED;
}

static long noc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct flexnoc_countercapture capture_data;
	void __user *argp = (void __user *)arg;
	struct flexnoc_probestart probe_data;
	struct flexnoc_setup setup_data;
	int rc;

	if (!arg) {
		pr_err("NOC: Null pointer from user\n");
		return -EINVAL;
	}
	switch (cmd) {
	case NOC_SETUP:
		if (copy_from_user(&setup_data,
				   argp, sizeof(setup_data))) {
			return -EFAULT;
		}
		rc =  flex_noc_setup(setup_data.noc_type, setup_data.counter,
				     setup_data.traceport);
		setup_data.ret_id = rc;

		if (copy_to_user(argp,
				 &setup_data, sizeof(setup_data))) {
			return -EFAULT;
		}
	break;
	case NOC_PROBE_START:
		if (copy_from_user(&probe_data, argp,
				   sizeof(probe_data))) {
			return -EFAULT;
		}
		rc = flexnoc_probe_start(probe_data.noc_type);
		probe_data.ret_id = rc;

		if (copy_to_user(argp,
				 &probe_data, sizeof(probe_data))) {
			return -EFAULT;
		}
	break;
	case NOC_COUNTER_CAPTURE:
		if (copy_from_user(&capture_data, argp,
				   sizeof(capture_data))) {
			return -EFAULT;
		}
		rc = flexnoc_counter_capture(capture_data.noc_type,
					     capture_data.counter,
					     &capture_data.bw_res);
		capture_data.ret_id = rc;

		if (copy_to_user(argp, &capture_data,
				 sizeof(capture_data))) {
			return -EFAULT;
		}
	break;
	default:
		return -EINVAL;
	}
	return 0;
}

static const struct file_operations noc_fops = {
	.owner  = THIS_MODULE,
	.unlocked_ioctl = noc_ioctl,
};

static int intel_noc_cdev_init(struct noc_device *nocdev)
{
	if ((alloc_chrdev_region(&nocdev->cdev, 0, 1, "nocdev")) < 0) {
		pr_err("Cannot allocate major number for NOC\n");
		return -EINVAL;
	}

	cdev_init(&nocdev->noc_cdev, &noc_fops);
	if ((cdev_add(&nocdev->noc_cdev, nocdev->cdev, 1)) < 0) {
		pr_err("Cannot add NOC device to the system\n");
		goto r_class;
	}

	nocdev->dev_class = class_create(THIS_MODULE, "noc_class");
	if (!nocdev->dev_class) {
		pr_err("Cannot create the NOC class\n");
		cdev_del(&nocdev->noc_cdev);
		goto r_class;
	}

	if ((device_create(nocdev->dev_class, NULL, nocdev->cdev,
			   NULL, "noc")) == NULL) {
		pr_err("Cannot create NOC device\n");
		cdev_del(&nocdev->noc_cdev);
		goto r_cdev;
	}

	return 0;

r_cdev:
	class_destroy(nocdev->dev_class);
r_class:
	unregister_chrdev_region(nocdev->cdev, 1);
	return -EINVAL;
}

static void intel_noc_cdev_exit(struct noc_device *nocdev)
{
	cdev_del(&nocdev->noc_cdev);
	class_destroy(nocdev->dev_class);
	unregister_chrdev_region(nocdev->cdev, 1);
}

static int __init noc_driver_module_init(void)
{
	int ret;

	ret = intel_noc_cdev_init(&noc_dev);
	if (ret)
		pr_err("NOC char device init failed\n");
	return ret;
}

static void noc_driver_module_exit(void)
{
	intel_noc_cdev_exit(&noc_dev);
}

module_init(noc_driver_module_init);
module_exit(noc_driver_module_exit);

MODULE_DESCRIPTION("Intel Keem Bay NOC interconnect driver");
MODULE_AUTHOR("Pandith N <pandith.n@intel.com>");
MODULE_AUTHOR("Sudarshan Ravula <sudarshan.ravula@intel.com>");
MODULE_LICENSE("GPL");
