/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2020 Intel Corporation
 */

#ifndef INTEL_GT_ERRORS_SYSFS_H
#define INTEL_GT_ERRORS_SYSFS_H

struct intel_gt;
struct kobject;

void intel_gt_sysfs_register_errors(struct intel_gt *gt,
				    struct kobject *parent);

#endif /* INTEL_GT_ERRORS_SYSFS_H */
