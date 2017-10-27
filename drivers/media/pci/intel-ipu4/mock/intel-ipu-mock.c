/*
 * Copyright (c) 2017 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/device.h>

#include "intel-ipu-mock.h"

static int intel_ipu_get_sim_type(void)
{
	return SIM_MOCK;
}

static int intel_ipu_mock_get_secure_mode(void)
{
	/* skip handling for security related features as no CSE HW */
	return 0;
}

static int intel_ipu_mock_ipc_reset(struct device *dev)
{
	/* Skip handling for security related features as no CSE HW */
	return 0;
}

static int intel_ipu_mock_start_tsc(void)
{
	/* Skip handling as no Hammock Harbor HW */
	return 0;
}

const struct intel_ipu_sim_ctrl sim_ctrl_ops = {
	.get_sim_type = intel_ipu_get_sim_type,
	.get_secure_mode = intel_ipu_mock_get_secure_mode,
	.ipc_reset = intel_ipu_mock_ipc_reset,
	.start_tsc = intel_ipu_mock_start_tsc,
};
EXPORT_SYMBOL_GPL(sim_ctrl_ops);

static int __init intel_ipu_mock_init(void)
{
	return 0;
}

static void __exit intel_ipu_mock_exit(void)
{
}

module_init(intel_ipu_mock_init);
module_exit(intel_ipu_mock_exit);

MODULE_AUTHOR("Yunliang Ding <yunliang.ding@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Intel IPU MOCK library");
