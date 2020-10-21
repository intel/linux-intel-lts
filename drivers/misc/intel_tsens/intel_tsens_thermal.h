/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *
 * Intel tsens thermal Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 */

#ifndef _LINUX_INTEL_TSENS_H
#define _LINUX_INTEL_TSENS_H

#include <linux/platform_device.h>
#include <linux/thermal.h>

struct intel_tsens_plat_data {
	const char *name;
	void __iomem *base_addr;
	int (*get_temp)(struct platform_device *pdev, int type, int *temp);
	void *pdata;
};

struct intel_tsens_plat_info {
	const char *plat_name;
	int id;
	struct platform_device *pdev;
	void __iomem *base_addr;
};

struct intel_tsens_i2c_plat_data {
	int (*get_temp)(int type, int *temp, void *pdata);
	void *pdata;
};

/* TSENS i2c platform data */
extern struct intel_tsens_i2c_plat_data i2c_plat_data;

#endif /* _LINUX_INTEL_TSENS_H */
