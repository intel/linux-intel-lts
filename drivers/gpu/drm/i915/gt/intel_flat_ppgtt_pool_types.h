/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2020 Intel Corporation
 */

#ifndef INTEL_FLAT_PPGTT_POOL_TYPES_H
#define INTEL_FLAT_PPGTT_POOL_TYPES_H

#include <linux/list.h>
#include <linux/types.h>
#include <linux/wait.h>

#include "i915_selftest.h"

#define INTEL_FLAT_PPGTT_MAX_PINNED_OBJS	1024
#define INTEL_FLAT_PPGTT_BB_OBJ_SIZE		SZ_32K

/* One page corresponding to each level of PT to hold scratch for blitter to
 * copy from for pvc MEMCOPY command.
 */
#define INTEL_FLAT_PPGTT_BB_SCRATCH_OFFSET	(INTEL_FLAT_PPGTT_BB_OBJ_SIZE - 5 * SZ_4K)
#define INTEL_FLAT_PPGTT_MAX_PTE_ENTRIES	((INTEL_FLAT_PPGTT_BB_OBJ_SIZE >> 5) - 2)
/* 16 DW per FAST_COLOR_BLT */
#define INTEL_FLAT_PPGTT_MAX_FILL_PTE_ENTRIES	((INTEL_FLAT_PPGTT_BB_OBJ_SIZE >> 6) - 2)

#define INTEL_FLAT_PPGTT_MAX_SCRATCH_PTE_ENTRIES	((INTEL_FLAT_PPGTT_BB_SCRATCH_OFFSET >> 6) - 2)

struct i915_address_space;
struct i915_request;
struct i915_vma;
struct intel_flat_ppgtt_pool;

struct intel_flat_ppgtt_request_pool {
	struct intel_flat_ppgtt_pool *fpp;
	struct list_head prq_list;
};

struct intel_pte_bo {
	struct i915_vma *vma;

	/* Fence to wait upon for last submission to update ppgtt */
	struct i915_request *wait;
	struct list_head link;
	u32 *cmd;
};

struct intel_flat_ppgtt_pool {
	struct list_head free_list;
	wait_queue_head_t bind_wq;
	struct list_head prq_list;
	spinlock_t rq_lock;
	I915_SELFTEST_DECLARE(struct {
		atomic_t count;
	} st;)
};

#endif /* INTEL_FLAT_PPGTT_POOL_TYPES_H */
