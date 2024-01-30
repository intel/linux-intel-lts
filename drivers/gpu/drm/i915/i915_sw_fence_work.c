// SPDX-License-Identifier: MIT

/*
 * Copyright © 2019 Intel Corporation
 */

#include "i915_drv.h"
#include "i915_sw_fence_work.h"

static struct workqueue_struct *get_wq(struct dma_fence_work *f)
{
	return f->rq.sched_engine->private_data;
}

static void fence_complete(struct dma_fence_work *f)
{
	i915_request_mark_complete(&f->rq);

	if (f->ops->complete)
		f->ops->complete(f);
	dma_fence_signal(&f->rq.fence);
}

static void fence_work(struct work_struct *work)
{
	struct dma_fence_work *f = container_of(work, typeof(*f), work);

	if (!f->rq.fence.error && f->ops->work) {
		int err;

		err = f->ops->work(f);
		if (err)
			dma_fence_set_error(&f->rq.fence, err);
	}

	fence_complete(f);
	dma_fence_put(&f->rq.fence);
}

static int
fence_notify(struct i915_sw_fence *fence, enum i915_sw_fence_notify state)
{
	struct dma_fence_work *f = container_of(fence, typeof(*f), rq.submit);

	switch (state) {
	case FENCE_COMPLETE:
		if (fence->error && !f->ops->no_error_propagation)
			dma_fence_set_error(&f->rq.fence, fence->error);

		dma_fence_get(&f->rq.fence);
		if (test_bit(DMA_FENCE_WORK_IMM, &f->rq.fence.flags))
			fence_work(&f->work);
		else
			queue_work(get_wq(f), &f->work);
		break;

	case FENCE_FREE:
		dma_fence_put(&f->rq.fence);
		break;
	}

	return NOTIFY_DONE;
}

static const char *get_driver_name(struct dma_fence *fence)
{
	return "dma-fence";
}

static const char *get_timeline_name(struct dma_fence *fence)
{
	struct dma_fence_work *f = container_of(fence, typeof(*f), rq.fence);

	return f->ops->name ?: "work";
}

static void fence_free(struct rcu_head *rcu)
{
	struct dma_fence_work *f = container_of(rcu, typeof(*f), rq.fence.rcu);

	i915_sched_node_retire(&f->rq.sched);
	i915_sw_fence_fini(&f->rq.submit);

	kfree(f);
}

static void rcu_fence_free(struct work_struct *wrk)
{
	struct dma_fence_work *f = container_of(wrk, typeof(*f), rcu_work.work);

	f->ops->release(f);
	fence_free(&f->rq.fence.rcu);
}

static void fence_release(struct dma_fence *fence)
{
	struct dma_fence_work *f = container_of(fence, typeof(*f), rq.fence);

	if (f->ops->rcu_release) {
		GEM_BUG_ON(!f->ops->release);
		INIT_RCU_WORK(&f->rcu_work, rcu_fence_free);
		queue_rcu_work(get_wq(f), &f->rcu_work);
		return;
	}

	if (f->ops->release)
		f->ops->release(f);

	call_rcu(&f->rq.fence.rcu, fence_free);
}

static bool fence_enable_signaling(struct dma_fence *fence)
{
	struct dma_fence_work *f = container_of(fence, typeof(*f), rq.fence);

	if (f->ops->enable_signaling)
		return f->ops->enable_signaling(f);

	return true;
}

const struct dma_fence_ops i915_cpu_fence_ops = {
	.get_driver_name = get_driver_name,
	.get_timeline_name = get_timeline_name,
	.enable_signaling = fence_enable_signaling,
	.release = fence_release,
};

void __dma_fence_work_init(struct dma_fence_work *f,
			   const struct dma_fence_work_ops *ops,
			   struct i915_sched_engine *se,
			   const char *name,
			   struct lock_class_key *key)
{
	static const u32 dummy_seqno = -1;

	BUILD_BUG_ON(DMA_FENCE_WORK_IMM >= BITS_PER_TYPE(f->rq.fence.flags));

	f->ops = ops;

	f->rq.engine = NULL;
	f->rq.sched_engine = se;
	i915_sched_node_init(&f->rq.sched);
	lockdep_set_class_and_name(&f->rq.sched.lock, key, name);
	f->rq.sched.flags = I915_SCHED_HAS_EXTERNAL_CHAIN;
	f->rq.execution_mask = -1;
	f->rq.emitted_jiffies = jiffies;

	dma_fence_init(&f->rq.fence, &i915_cpu_fence_ops, &f->rq.sched.lock, 0, 0);
	f->rq.hwsp_seqno = &dummy_seqno;
	f->rq.fence.flags = BIT(I915_FENCE_FLAG_INITIAL_BREADCRUMB);
	if (!ops->work)
		f->rq.fence.flags |= BIT(DMA_FENCE_WORK_IMM);

	i915_sw_fence_init(&f->rq.submit, fence_notify);
	INIT_WORK(&f->work, fence_work);
}

void dma_fence_work_chain(struct dma_fence_work *f, struct dma_fence *signal)
{
	struct i915_request *rq = &f->rq;

	if (!signal)
		return;

	if (!__i915_sw_fence_await_dma_fence(&rq->submit, signal, &rq->dmaq))
		return;

	if (dma_fence_is_i915(signal))
		__i915_sched_node_add_dependency(&rq->sched,
						 &to_request(signal)->sched,
						 &rq->dep,
						 0);
}
