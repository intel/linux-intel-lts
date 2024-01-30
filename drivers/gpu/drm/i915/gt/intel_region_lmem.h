/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2019 Intel Corporation
 */

#ifndef __INTEL_REGION_LMEM_H
#define __INTEL_REGION_LMEM_H

#include <linux/types.h>

struct intel_gt;

struct intel_memory_region *intel_gt_setup_lmem(struct intel_gt *gt);
int intel_get_tile_range(struct intel_gt *gt,
			 resource_size_t *lmem_base,
			 resource_size_t *lmem_size);

#endif /* !__INTEL_REGION_LMEM_H */
