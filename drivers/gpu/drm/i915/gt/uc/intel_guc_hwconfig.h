// SPDX-License-Identifier: MIT
/*
 * Copyright © 2020 Intel Corporation
 */

#ifndef _INTEL_GUC_HWCONFIG_H_
#define _INTEL_GUC_HWCONFIG_H_

#include "i915_vma_types.h"

struct intel_guc;

struct intel_guc_hwconfig {
	u32 size;
	void *ptr;
};

int intel_guc_hwconfig_init(struct intel_guc_hwconfig *hwconfig);
void intel_guc_hwconfig_fini(struct intel_guc_hwconfig *hwconfig);
u32 intel_guc_hwconfig_get_value(struct intel_guc_hwconfig *hwconfig, u32 key);

#endif /* _INTEL_GUC_HWCONFIG_H_ */
