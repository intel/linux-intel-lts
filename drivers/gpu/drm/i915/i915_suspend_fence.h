/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2021 Intel Corporation
 */
#ifndef __I915_SUSPEND_FENCE_H__
#define __I915_SUSPEND_FENCE_H__

#include <linux/types.h>

#include "gt/intel_context_types.h"

#include "i915_active.h"
#include "i915_sw_fence_work.h"

struct i915_request;

/**
 * struct i915_suspend_fence - suspend fence ops structure
 */
struct i915_suspend_fence_ops {
	/**
	 * revalidate - Revalidate objects and vmas before restart
	 * @ce: The suspended context.
	 *
	 * Take whatever actions needed for the vmas accessed by the
	 * request to be up-to-date and bound.
	 */
	void (*revalidate)(struct intel_context *ce);
};

/**
 * struct i915_suspend_fence - suspend fence structure
 *
 * @suspend: The struct dma_fence_work that defines the fence.
 * @ce: The context which is suspended and resumed.
 * @block_requested: Set to true if context suspend initiated.
 * @submit_wq: Preallocated wait que entry for the request submit_fence await.
 * @block_wq: Preallocated wait queue entry for the suspend fence await.
 * @suspend_work: Work struct used to push suspend work to a workqueue.
 * @ops: Pointer to the ops struct for this suspend fence.
 */
struct i915_suspend_fence {
	struct dma_fence_work base;
	struct intel_context *ce;
	bool block_requested;
	wait_queue_entry_t submit_wq;
	wait_queue_entry_t block_wq;
	struct work_struct suspend_work;
	const struct i915_suspend_fence_ops *ops;
};

static inline bool dma_fence_is_suspend(const struct dma_fence *fence)
{
	return fence->ops == &i915_cpu_fence_ops &&
		test_bit(I915_SUSPEND_FENCE, &fence->flags);
}

struct dma_fence *
i915_suspend_fence_init(struct i915_suspend_fence *sfence,
			struct intel_context *ce,
			const struct i915_suspend_fence_ops *ops);

void i915_suspend_fence_retire_dma_fence(struct dma_fence *fence);
#endif
