// SPDX-License-Identifier: MIT
/*
 * Copyright © 2020 Intel Corporation
 */

#include <linux/sizes.h>
#include <linux/types.h>

#include "intel_lmtt.h"
#include "i915_gem.h"

typedef u32 xehpsdv_lmtt_pde_t;
typedef u32 xehpsdv_lmtt_pte_t;

#define XEHPSDV_LMTT_PDE_SHIFT	4
#define XEHPSDV_LMTT_PDE_ADDR	GENMASK(24, XEHPSDV_LMTT_PDE_SHIFT)
#define XEHPSDV_LMTT_PDE_MAX	64 /* PDE index 0 is unused */

#define XEHPSDV_LMTT_PTE_SHIFT	5
#define XEHPSDV_LMTT_PTE_ADDR	GENMASK(20, XEHPSDV_LMTT_PTE_SHIFT)
#define XEHPSDV_LMTT_PTE_MAX	SZ_64K

static unsigned int xehpsdv_lmtt_root_pd_level(void)
{
	return 1;
}

static unsigned int xehpsdv_lmtt_pte_num(unsigned int level)
{
	switch (level) {
	case 1:
		return XEHPSDV_LMTT_PDE_MAX;
	case 0:
		return XEHPSDV_LMTT_PTE_MAX;
	default:
		MISSING_CASE(level);
		return 0;
	}
}

static resource_size_t xehpsdv_lmtt_pte_size(unsigned int level)
{
	switch (level) {
	case 1:
		return sizeof(xehpsdv_lmtt_pde_t);
	case 0:
		return sizeof(xehpsdv_lmtt_pte_t);
	default:
		MISSING_CASE(level);
		return 0;
	}
}

static unsigned int xehpsdv_lmtt_pte_shift(unsigned int level)
{
	switch (level) {
	case 0:
		return ilog2(SZ_2M);
	default:
		MISSING_CASE(level);
		return 0;
	}
}

static unsigned int xehpsdv_lmtt_pte_idx(u64 addr, unsigned int level)
{
	addr >>= xehpsdv_lmtt_pte_shift(level);

	switch (level) {
	case 0: /* SZ_2M increments */
		return addr & (XEHPSDV_LMTT_PTE_MAX - 1);
	default:
		MISSING_CASE(level);
		return 0;
	}
}

static u64
xehpsdv_lmtt_pte_encode(unsigned long offset, u32 flags, unsigned int level)
{
	switch (level) {
	case 0:
		GEM_BUG_ON(!IS_ALIGNED(offset, SZ_2M));
		offset >>= LMTT_PAGE_SHIFT;
		offset <<= XEHPSDV_LMTT_PTE_SHIFT;
		GEM_BUG_ON(offset & ~XEHPSDV_LMTT_PTE_ADDR);
		break;
	case 1:
		GEM_BUG_ON(!IS_ALIGNED(offset, SZ_64K));
		offset >>= LMTT_PT_SHIFT;
		offset <<= XEHPSDV_LMTT_PDE_SHIFT;
		GEM_BUG_ON(offset & ~XEHPSDV_LMTT_PDE_ADDR);
		break;
	default:
		GEM_BUG_ON(true);
	}

	return offset | flags;
}

static void
xehpsdv_lmtt_pte_write(void *pt_vaddr, u64 value,
		       unsigned int n, unsigned int level)
{
	xehpsdv_lmtt_pte_t *pte = pt_vaddr;

	switch (level) {
	case 0:
	case 1:
		pte[n] = value;
		break;
	default:
		MISSING_CASE(level);
	}
}

static struct intel_lmtt_pt *
xehpsdv_lmtt_leaf_pt(struct intel_lmtt *lmtt, u64 addr, unsigned int vf)
{
	return lmtt->pd->entry[vf];
}

const struct intel_lmtt_ops xehpsdv_lmtt_ops = {
	.lmtt_root_pd_level = xehpsdv_lmtt_root_pd_level,
	.lmtt_pte_num = xehpsdv_lmtt_pte_num,
	.lmtt_pte_size = xehpsdv_lmtt_pte_size,
	.lmtt_pte_shift = xehpsdv_lmtt_pte_shift,
	.lmtt_pte_idx = xehpsdv_lmtt_pte_idx,
	.lmtt_pte_encode = xehpsdv_lmtt_pte_encode,
	.lmtt_pte_write = xehpsdv_lmtt_pte_write,
	.lmtt_leaf_pt = xehpsdv_lmtt_leaf_pt,
};
