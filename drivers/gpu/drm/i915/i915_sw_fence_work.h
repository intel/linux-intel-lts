/* SPDX-License-Identifier: MIT */

/*
 * Copyright © 2019 Intel Corporation
 */

#ifndef I915_SW_FENCE_WORK_H
#define I915_SW_FENCE_WORK_H

#include <linux/dma-fence.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include "i915_request.h"
#include "i915_sw_fence.h"

struct dma_fence_work;

struct dma_fence_work_ops {
	const char *name;
	int (*work)(struct dma_fence_work *f);
	bool (*enable_signaling)(struct dma_fence_work *f);
	void (*complete)(struct dma_fence_work *f);
	void (*release)(struct dma_fence_work *f);

	bool no_error_propagation:1;
	bool rcu_release:1;
};

struct dma_fence_work {
	struct i915_request rq;

	union {
		struct work_struct work;
		struct rcu_work rcu_work;
	};
	const struct dma_fence_work_ops *ops;
};

enum {
	DMA_FENCE_WORK_IMM = __I915_FENCE_FLAG_LAST__,
	I915_SUSPEND_FENCE,
};

void __dma_fence_work_init(struct dma_fence_work *f,
			   const struct dma_fence_work_ops *ops,
			   struct i915_sched_engine *se,
			   const char *name,
			   struct lock_class_key *key);
#ifdef CONFIG_LOCKDEP
#define dma_fence_work_init(f, ops, se)				\
do {								\
	static struct lock_class_key __key;			\
								\
	__dma_fence_work_init((f), (ops), (se), #f, &__key);		\
} while (0)
#else
#define dma_fence_work_init(f, ops, se)				\
	__dma_fence_work_init((f), (ops), se, NULL, NULL)
#endif
void dma_fence_work_chain(struct dma_fence_work *f, struct dma_fence *signal);

static inline void dma_fence_work_commit(struct dma_fence_work *f)
{
	i915_sw_fence_commit(&f->rq.submit);
}

static inline void
dma_fence_work_commit_imm_if(struct dma_fence_work *f, bool cond)
{
	if (atomic_read(&f->rq.submit.pending) <= cond)
		__set_bit(DMA_FENCE_WORK_IMM, &f->rq.fence.flags);

	dma_fence_work_commit(f);
}

/**
 * dma_fence_work_commit_imm: Commit the fence, and if possible execute locally.
 * @f: the fenced worker
 *
 * Instead of always scheduling a worker to execute the callback (see
 * dma_fence_work_commit()), we try to execute the callback immediately in
 * the local context. It is required that the fence be committed before it
 * is published, and that no other threads try to tamper with the number
 * of asynchronous waits on the fence (or else the callback will be
 * executed in the wrong context, i.e. not the callers).
 */
static inline void dma_fence_work_commit_imm(struct dma_fence_work *f)
{
	dma_fence_work_commit_imm_if(f, true);
}

#endif /* I915_SW_FENCE_WORK_H */
