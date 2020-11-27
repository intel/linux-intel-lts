/* SPDX-License-Identifier: GPL-2.0-only
 * VPUMGR Kernel module - common definition
 * Copyright (C) 2020-2021 Intel Corporation
 */
#ifndef _VPU_COMMON_H
#define _VPU_COMMON_H
#include <linux/cdev.h>
#include <linux/platform_device.h>

#include <uapi/misc/vpumgr.h>

#include "vpu_vcm.h"

#define FWNAME_MAX_LENGTH  128

/* there will be one such device for each HW instance */
struct vpumgr_device {
	struct device *sdev;
	struct device *dev;
	dev_t devnum;
	struct cdev cdev;
	struct platform_device *pdev;

	struct vcm_dev vcm;
	struct dentry *debugfs_root;

	struct mutex client_mutex; /* protect client_list */
	struct list_head client_list;

	char fwname[FWNAME_MAX_LENGTH];
};

#define XLINK_INVALID_SW_DEVID  0xDEADBEEF

#endif
