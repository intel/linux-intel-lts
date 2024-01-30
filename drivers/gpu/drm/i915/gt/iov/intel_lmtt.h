/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2020 Intel Corporation
 */

#ifndef __INTEL_LMTT_H__
#define __INTEL_LMTT_H__

#include <linux/types.h>

struct drm_i915_gem_object;
struct intel_lmtt;
struct intel_lmtt_pt;
struct intel_lmtt_ops;

extern const struct intel_lmtt_ops xehpsdv_lmtt_ops;
extern const struct intel_lmtt_ops pvc_lmtt_ops;

#define LMTT_PT_SHIFT		16
#define LMTT_PAGE_SHIFT		21
#define LMTT_PTE_VALID		BIT(0)

struct intel_lmtt {
	struct intel_lmtt_pt *pd;

	const struct intel_lmtt_ops *ops;
};

struct intel_lmtt_pt {
	struct drm_i915_gem_object *obj;
	struct intel_lmtt_pt **entry;
};

struct intel_lmtt_ops {
	unsigned int (*lmtt_root_pd_level)(void);
	unsigned int (*lmtt_pte_num)(unsigned int level);
	resource_size_t (*lmtt_pte_size)(unsigned int level);
	unsigned int (*lmtt_pte_shift)(unsigned int level);
	unsigned int (*lmtt_pte_idx)(u64 addr, unsigned int level);
	u64 (*lmtt_pte_encode)(unsigned long offset, u32 flags, unsigned int level);
	void (*lmtt_pte_write)(void *pt_vaddr, u64 value, unsigned int n,
			       unsigned int level);
	struct intel_lmtt_pt *(*lmtt_leaf_pt)(struct intel_lmtt *lmtt, u64 addr,
					      unsigned int vf);
};

int intel_lmtt_init(struct intel_lmtt *lmtt);
void intel_lmtt_fini(struct intel_lmtt *lmtt);
void intel_lmtt_init_hw(struct intel_lmtt *lmtt);
int intel_lmtt_update_entries(struct intel_lmtt *lmtt, unsigned int vf);

resource_size_t intel_lmtt_estimate_pt_size(struct intel_lmtt *lmtt, u64 size);

#endif
