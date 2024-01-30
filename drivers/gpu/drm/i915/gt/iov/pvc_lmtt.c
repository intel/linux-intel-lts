// SPDX-License-Identifier: MIT
/*
 * Copyright © 2020 Intel Corporation
 */

#include <linux/sizes.h>
#include <linux/types.h>

#include "intel_lmtt.h"
#include "i915_gem.h"

typedef u64 pvc_lmtt_pde_t;
typedef u64 pvc_lmtt_l2e_t;
typedef u32 pvc_lmtt_pte_t;

#define PVC_LMTT_PDE_SHIFT	4
#define PVC_LMTT_PDE_ADDR	GENMASK_ULL(33, PVC_LMTT_PDE_SHIFT)
#define PVC_LMTT_PDE_MAX	64 /* PDE index 0 is unused */

#define PVC_LMTT_L2E_MAX	SZ_8K

#define PVC_LMTT_PTE_SHIFT	5
#define PVC_LMTT_PTE_ADDR	GENMASK(29, PVC_LMTT_PTE_SHIFT)
#define PVC_LMTT_PTE_MAX	SZ_16K

static unsigned int pvc_lmtt_root_pd_level(void)
{
	return 2;
}

static unsigned int pvc_lmtt_pte_num(unsigned int level)
{
	switch (level) {
	case 2:
		return PVC_LMTT_PDE_MAX;
	case 1:
		return PVC_LMTT_L2E_MAX;
	case 0:
		return PVC_LMTT_PTE_MAX;
	default:
		MISSING_CASE(level);
		return 0;
	}
}

static resource_size_t pvc_lmtt_pte_size(unsigned int level)
{
	switch (level) {
	case 2:
		return sizeof(pvc_lmtt_pde_t);
	case 1:
		return sizeof(pvc_lmtt_l2e_t);
	case 0:
		return sizeof(pvc_lmtt_pte_t);
	default:
		MISSING_CASE(level);
		return 0;
	}
}

static unsigned int pvc_lmtt_pte_shift(unsigned int level)
{
	switch (level) {
	case 1:
		return ilog2(SZ_32G);
	case 0:
		return ilog2(SZ_2M);
	default:
		MISSING_CASE(level);
		return 0;
	}
}

static unsigned int pvc_lmtt_pte_idx(u64 addr, unsigned int level)
{
	addr >>= pvc_lmtt_pte_shift(level);

	switch (level) {
	case 1: /* SZ_32G increments */
		return addr & (PVC_LMTT_L2E_MAX - 1);
	case 0: /* SZ_2M increments */
		return addr & (PVC_LMTT_PTE_MAX - 1);
	default:
		MISSING_CASE(level);
		return 0;
	}
}

static u64
pvc_lmtt_pte_encode(unsigned long offset, u32 flags, unsigned int level)
{
	switch (level) {
	case 0:
		GEM_BUG_ON(!IS_ALIGNED(offset, SZ_2M));
		offset >>= LMTT_PAGE_SHIFT;
		offset <<= PVC_LMTT_PTE_SHIFT;
		GEM_BUG_ON(offset & ~PVC_LMTT_PTE_ADDR);
		break;
	case 1:
	case 2:
		GEM_BUG_ON(!IS_ALIGNED(offset, SZ_64K));
		offset >>= LMTT_PT_SHIFT;
		offset <<= PVC_LMTT_PDE_SHIFT;
		GEM_BUG_ON(offset & ~PVC_LMTT_PDE_ADDR);
		break;
	default:
		GEM_BUG_ON(true);
	}

	return offset | flags;
}

static void
pvc_lmtt_pte_write(void *pt_vaddr, u64 value,
		   unsigned int n, unsigned int level)
{
	pvc_lmtt_pte_t *pte = pt_vaddr;
	pvc_lmtt_pde_t *pde = pt_vaddr;

	switch (level) {
	case 0:
		pte[n] = value;
		break;
	case 1:
	case 2:
		pde[n] = value;
		break;
	default:
		MISSING_CASE(level);
	}
}

static struct intel_lmtt_pt *
pvc_lmtt_leaf_pt(struct intel_lmtt *lmtt, u64 addr, unsigned int vf)
{
	struct intel_lmtt_pt *pt = lmtt->pd->entry[vf];

	return pt->entry[pvc_lmtt_pte_idx(addr, 1)];
}

const struct intel_lmtt_ops pvc_lmtt_ops = {
	.lmtt_root_pd_level = pvc_lmtt_root_pd_level,
	.lmtt_pte_num = pvc_lmtt_pte_num,
	.lmtt_pte_size = pvc_lmtt_pte_size,
	.lmtt_pte_shift = pvc_lmtt_pte_shift,
	.lmtt_pte_idx = pvc_lmtt_pte_idx,
	.lmtt_pte_encode = pvc_lmtt_pte_encode,
	.lmtt_pte_write = pvc_lmtt_pte_write,
	.lmtt_leaf_pt = pvc_lmtt_leaf_pt,
};
