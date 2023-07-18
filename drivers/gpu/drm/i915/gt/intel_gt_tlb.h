/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2023 Intel Corporation
 */

#ifndef INTEL_GT_TLB_H
#define INTEL_GT_TLB_H

#include <linux/seqlock.h>
#include <linux/types.h>

#include "intel_gt_types.h"

void intel_gt_tlb_init(struct intel_gt *gt);
void intel_gt_tlb_fini(struct intel_gt *gt);

void intel_gt_tlb_invalidate(struct intel_gt *gt, u32 seqno);

static inline u32 intel_gt_tlb_seqno(const struct intel_gt *gt)
{
	return seqprop_sequence(&gt->tlb.seqno);
}

static inline u32 intel_gt_tlb_next_seqno_full(const struct intel_gt *gt)
{
	return intel_gt_tlb_seqno(gt) | 1;
}

#endif /* INTEL_GT_TLB_H */
