/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2022 Intel Corporation
 */

#ifndef INTEL_CCS_MODE_H
#define INTEL_CCS_MODE_H

#include "intel_engine_types.h"

struct dentry;
struct intel_gt;

void intel_gt_apply_ccs_mode(struct intel_gt *gt);
int intel_gt_configure_ccs_mode(struct intel_gt *gt,
				intel_engine_mask_t config);

void intel_gt_init_ccs_mode(struct intel_gt *gt);
void intel_gt_park_ccs_mode(struct intel_gt *gt,
			    struct intel_engine_cs *engine);
void intel_gt_debugfs_register_ccs_mode(struct intel_gt *gt,
					struct dentry *root);
void intel_gt_fini_ccs_mode(struct intel_gt *gt);

#endif /* INTEL_CCS_MODE_H */
