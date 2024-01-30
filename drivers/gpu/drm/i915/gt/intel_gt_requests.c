// SPDX-License-Identifier: MIT
/*
 * Copyright © 2019 Intel Corporation
 */

#include <linux/workqueue.h>

#include "i915_drv.h" /* for_each_engine() */
#include "i915_request.h"
#include "intel_breadcrumbs.h"
#include "intel_engine_heartbeat.h"
#include "intel_engine_pm.h"
#include "intel_execlists_submission.h"
#include "intel_gt.h"
#include "intel_gt_pm.h"
#include "intel_gt_requests.h"
#include "intel_timeline.h"

static void retire_requests(const struct intel_timeline *tl)
{
	struct i915_request *rq, *rn;

	list_for_each_entry_safe(rq, rn, &tl->requests, link)
		if (!i915_request_retire(rq))
			return;
}

static bool timeline_retire(struct intel_timeline *tl)
{
	if (list_empty(&tl->requests))
		return true;

	if (mutex_trylock(&tl->mutex)) {
		retire_requests(tl);
		mutex_unlock(&tl->mutex);
	}

	return list_empty(&tl->requests);
}

static bool engine_idle(const struct intel_engine_cs *engine)
{
	return timeline_retire(engine->kernel_context->timeline);
}

static bool flush_submission(struct intel_gt *gt)
{
	struct intel_engine_cs *engine;
	enum intel_engine_id id;
	bool idle = true;

	if (!intel_gt_pm_is_awake(gt))
		return true;

	for_each_engine(engine, gt, id) {
		if (!intel_engine_pm_is_awake(engine))
			continue;

		intel_engine_flush_submission(engine);
		intel_engine_signal_breadcrumbs(engine);

		/* Flush the background retirement and idle barriers */
		flush_work(&engine->retire_work);
		flush_delayed_work(&engine->wakeref.work);

		/* Is the idle barrier still outstanding? */
		if (engine->kernel_context)
			idle &= engine_idle(engine);
	}

	return idle;
}

static void engine_retire(struct work_struct *work)
{
	struct intel_engine_cs *engine =
		container_of(work, typeof(*engine), retire_work);
	struct intel_timeline *tl = xchg(&engine->retire, NULL);

	do {
		struct intel_timeline *next = xchg(&tl->retire, NULL);

		/*
		 * Our goal here is to retire _idle_ timelines as soon as
		 * possible (as they are idle, we do not expect userspace
		 * to be cleaning up anytime soon).
		 *
		 * If the timeline is currently locked, either it is being
		 * retired elsewhere or about to be!
		 */
		if (mutex_trylock(&tl->mutex)) {
			retire_requests(tl);
			mutex_unlock(&tl->mutex);
		}
		intel_timeline_put(tl);

		GEM_BUG_ON(!next);
		tl = ptr_mask_bits(next, 1);
	} while (tl);
}

static bool add_retire(struct intel_engine_cs *engine,
		       struct intel_timeline *tl)
{
#define STUB ((struct intel_timeline *)1)
	struct intel_timeline *first;

	/*
	 * We open-code a llist here to include the additional tag [BIT(0)]
	 * so that we know when the timeline is already on a
	 * retirement queue: either this engine or another.
	 */

	if (cmpxchg(&tl->retire, NULL, STUB)) /* already queued */
		return false;

	intel_timeline_get(tl);
	first = READ_ONCE(engine->retire);
	do
		tl->retire = ptr_pack_bits(first, 1, 1);
	while (!try_cmpxchg(&engine->retire, &first, tl));

	return !first;
}

void intel_engine_add_retire(struct intel_engine_cs *engine,
			     struct intel_timeline *tl)
{
	/* We don't deal well with the engine disappearing beneath us */
	GEM_BUG_ON(intel_engine_is_virtual(engine));

	if (add_retire(engine, tl))
		schedule_work(&engine->retire_work);
}

void intel_engine_init_retire(struct intel_engine_cs *engine)
{
	INIT_WORK(&engine->retire_work, engine_retire);
}

void intel_engine_fini_retire(struct intel_engine_cs *engine)
{
	flush_work(&engine->retire_work);
	GEM_BUG_ON(engine->retire);
}

/**
 * intel_gt_retire_requests_timeout - try to retire all completed requests
 * @gt - the GT from which to retire outstanding requests
 * @remain - How long we should wait in total for any outstanding request
 *
 * Look through all the active timelines/contexts on the GT, and try to
 * retire all the requests. If @remain is provided, any incomplete
 * timeline will be waited on upto the remaining timeout (in jiffies).
 * Any time not consumed by waiting is returned via @remain.
 *
 * Returns true if all requests were retired and the GT is now idle.
 */
bool intel_gt_retire_requests_timeout(struct intel_gt *gt, long *remain)
{
	struct intel_gt_timelines *timelines = &gt->timelines;
	long timeout = remain ? *remain : -ETIME;
	struct intel_timeline *tl;
	bool idle = true;

	if (gt->i915->quiesce_gpu)
		return true;

	if (remain)
		flush_submission(gt); /* kick the ksoftirqd tasklets */

	rcu_read_lock();
	list_for_each_entry_rcu(tl, &timelines->active_list, link) {
		if (!intel_timeline_get_if_active(tl)) /* pin the link */
			continue;
		rcu_read_unlock();

		if (timeout > 0) {
			struct dma_fence *fence;

			fence = i915_active_fence_get(&tl->last_request);
			if (fence) {
				timeout = dma_fence_wait_timeout(fence,
								 true,
								 timeout);
				dma_fence_put(fence);
			}
		}

		/* Retirement is best effort */
		idle &= timeline_retire(tl);

		rcu_read_lock();
		intel_timeline_put_active(tl);
	}
	rcu_read_unlock();

	if (remain) {
		idle &= flush_submission(gt); /* Wait, there's more! */
		*remain = max(timeout, 0l);
	}

	return idle;
}

static void retire_work_handler(struct work_struct *work)
{
	struct intel_gt *gt =
		container_of(work, typeof(*gt), requests.retire_work.work);

	schedule_delayed_work(&gt->requests.retire_work,
			      round_jiffies_up_relative(HZ));
	intel_gt_retire_requests(gt);
}

void intel_gt_init_requests(struct intel_gt *gt)
{
	INIT_DELAYED_WORK(&gt->requests.retire_work, retire_work_handler);
}

void intel_gt_park_requests(struct intel_gt *gt)
{
	cancel_delayed_work(&gt->requests.retire_work);
}

void intel_gt_unpark_requests(struct intel_gt *gt)
{
	schedule_delayed_work(&gt->requests.retire_work,
			      round_jiffies_up_relative(HZ));
}

void intel_gt_fini_requests(struct intel_gt *gt)
{
	/* Wait until the work is marked as finished before unloading! */
	cancel_delayed_work_sync(&gt->requests.retire_work);

	flush_work(&gt->watchdog.work);
}

void intel_gt_watchdog_work(struct work_struct *work)
{
	struct intel_gt *gt =
		container_of(work, typeof(*gt), watchdog.work);
	struct i915_request *rq, *rn;
	struct llist_node *first;

	first = llist_del_all(&gt->watchdog.list);
	if (!first)
		return;

	llist_for_each_entry_safe(rq, rn, first, watchdog.link) {
		if (!i915_request_completed(rq)) {
			struct dma_fence *f = &rq->fence;

			pr_notice("Fence expiration time out i915-%s:%s:%llx!\n",
				  f->ops->get_driver_name(f),
				  f->ops->get_timeline_name(f),
				  f->seqno);
			i915_request_cancel(rq, -EINTR);
		}
		i915_request_put(rq);
	}
}
