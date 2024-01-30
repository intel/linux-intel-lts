// SPDX-License-Identifier: MIT
/*
 * Copyright © 2020 Intel Corporation
 */

#include "gem/i915_gem_internal.h"
#include "gem/i915_gem_object.h"

#include "i915_drv.h"
#include "intel_flat_ppgtt_pool.h"
#include "intel_gt.h"
#include "intel_gt_pm.h"
#include "intel_ring.h"

#define fpp_to_gt(fpp__) container_of((fpp__), struct intel_gt, fpp)

static void free_pool(struct intel_flat_ppgtt_request_pool *pool)
{
	if (!pool)
		return;

	if (!list_empty(&pool->prq_list)) {
		struct intel_flat_ppgtt_pool *fpp = pool->fpp;
		struct i915_request *rq, *rn;

		spin_lock(&fpp->rq_lock);
		if (!list_empty(&pool->prq_list)) {
			rq = list_first_entry(&pool->prq_list,
					typeof(*rq),
					fpp.vma_link);
			rn = list_last_entry(&pool->prq_list,
					typeof(*rn),
					fpp.vma_link);
			__list_del(rq->fpp.gt_link.prev, rn->fpp.gt_link.next);
		}
		spin_unlock(&fpp->rq_lock);

		list_for_each_entry_safe(rq, rn, &pool->prq_list, fpp.vma_link)
			i915_request_free(rq);
	}

	kfree(pool);
}

void intel_flat_ppgtt_request_pool_clean(struct i915_vma *vma)
{
	if (vma->pool)
		free_pool(xchg(&vma->pool, NULL));
}

static void binder_throttle(struct intel_context *ce)
{
	struct intel_ring *ring = ce->ring;
	struct i915_request *it, *rq = NULL;

	rcu_read_lock();
	list_for_each_entry_reverse(it, &ce->timeline->requests, link) {
		if (__i915_request_is_complete(it) || it->ring != ring)
			break;

		if (__intel_ring_space(it->postfix,
				       ring->emit, ring->size) < ring->size / 2) {
			rq = i915_request_get_rcu(it);
			break;
		}
	}
	rcu_read_unlock();

	if (rq) {
		__i915_request_wait(rq, 0, MAX_SCHEDULE_TIMEOUT);
		i915_request_put(rq);
	}
}

struct i915_request *
intel_flat_ppgtt_get_request(struct intel_flat_ppgtt_pool *fpp)
{
	struct intel_gt *gt = container_of(fpp, struct intel_gt, fpp);
	enum intel_engine_id id = gt->rsvd_bcs;
	struct intel_context *ce = gt->engine[id]->bind_context;
	struct i915_request *rq;
	int err;

	I915_SELFTEST_DECLARE(atomic_inc(&fpp->st.count));

	spin_lock(&fpp->rq_lock);
	rq = list_first_entry_or_null(&fpp->prq_list, struct i915_request, fpp.gt_link);
	if (rq) {
		list_del(&rq->fpp.gt_link);
		list_del(&rq->fpp.vma_link);
	}
	spin_unlock(&fpp->rq_lock);

	if (unlikely(!rq)) {
		rq = i915_request_alloc(GFP_ATOMIC | __GFP_NOWARN);
		if (!rq)
			return ERR_PTR(-ENOMEM);
	}

	while ((err = i915_request_construct(rq, ce, BIT(I915_FENCE_FLAG_NONBLOCKING))) == -EAGAIN)
		binder_throttle(ce);

	if (err) {
		i915_request_free(rq);
		rq = ERR_PTR(err);
	}

	return rq;
}

static unsigned long __pd_count(u64 size, int shift)
{
	return (size + 2 * (BIT_ULL(shift) - 1)) >> shift;
}

static unsigned long pd_count(struct i915_address_space *vm, u64 size)
{
	unsigned long count;
	int shift, n;

	shift = vm->pd_shift;
	if (!shift)
		return 0;

	count = __pd_count(size, shift);
	for (n = 1; n < vm->top; n++) {
		shift += ilog2(I915_PDES); /* Each PD holds 512 entries */
		count += __pd_count(size, shift);
	}

	return count;
}

static unsigned long pt_count(struct i915_address_space *vm, u64 size)
{
	return size >> 12;
}

/* TODO: proper error handling, do proper cleanup */
void intel_flat_ppgtt_allocate_requests(struct i915_vma *vma, bool clear)
{
	struct intel_flat_ppgtt_request_pool *pool;
	struct intel_gt *gt = vma->vm->gt;
	struct intel_engine_cs *engine;
	struct intel_context *ce;
	LIST_HEAD(fpp_list);
	int count;

	engine = gt->engine[gt->rsvd_bcs];
	if (!engine)
		return;

	ce = engine->bind_context;
	if (!ce || !i915_is_level4_wa_active(gt))
		return;

	/*
	 * GFP_ATOMIC is used in this function inorder for allocation to go through
	 * to prevent fs_reclaim recursive cicular dependency warning which happens
	 * occur for unbind in shrinker context.
	 * When unbind is removed from shrinker context, GFP_ATOMIC will be replace
	 * with GFP_KERNEL.
	 */
	pool = kmalloc(sizeof(*pool), GFP_ATOMIC);
	if (!pool)
		return;

	pool->fpp = &gt->fpp;
	INIT_LIST_HEAD(&pool->prq_list);

	/*
	 * Each rq will submit one batch vma, and each batch vma can update
	 * max of 128 PTE's. Note the vma->node.size may be larger than the
	 * vma->size; we will only insert PTE for the actual pages present
	 * (vma->size) leaving the reset of the allocated node pointing to
	 * scratch (reusing the existing scratch page directories). We
	 * only need to preallocate enough PTE to cover
	 * 	size = min(vma->size, vma->node.size) == vma->size.
	 *
	 * Maximum number of PTE's to be updated for size is
	 * 	max_ptes = size >> 12.
	 * Number of batch vma/requests needed
	 * 	count = max_ptes >> 7;
	 * For bind we need to allocate requests needed for alloc vma range and
	 * also for the inserting entries.
	 *
	 * For unbind we only allocate requests for cleaning.
	 * Also account for any eviction that may be needed.
	 */
	count = pt_count(vma->vm, vma->size) >> 7;
	count += pd_count(vma->vm, vma->size) >> 7;
	count += !clear;
	do {
		struct i915_request *rq;

		rq = i915_request_alloc(GFP_ATOMIC | __GFP_NOWARN);
		if (!rq)
			break;

		list_add(&rq->fpp.vma_link, &pool->prq_list);
		list_add(&rq->fpp.gt_link, &fpp_list);
	} while (count--);

	pool = xchg(&vma->pool, pool);

	spin_lock(&gt->fpp.rq_lock);
	list_splice_tail(&fpp_list, &gt->fpp.prq_list);
	spin_unlock(&gt->fpp.rq_lock);

	free_pool(pool);
}

struct intel_pte_bo *
intel_flat_ppgtt_get_pte_bo(struct intel_flat_ppgtt_pool *fpp)
{
	struct intel_pte_bo *it;
	DEFINE_WAIT(wait);

	GEM_BUG_ON(!intel_gt_pm_is_awake(fpp_to_gt(fpp)));

	spin_lock(&fpp->bind_wq.lock);
	__add_wait_queue(&fpp->bind_wq, &wait);
	do {
		__set_current_state(TASK_UNINTERRUPTIBLE);

		it = list_first_entry_or_null(&fpp->free_list,
					      typeof(*it), link);
		if (it)
			break;

		spin_unlock(&fpp->bind_wq.lock);

		schedule();

		spin_lock(&fpp->bind_wq.lock);
	} while (1);
	list_del(&it->link);

	__remove_wait_queue(&fpp->bind_wq, &wait);
	__set_current_state(TASK_RUNNING);

	spin_unlock(&fpp->bind_wq.lock);

	return it;
}

void intel_flat_ppgtt_put_pte_bo(struct intel_flat_ppgtt_pool *fpp, struct intel_pte_bo *bo)
{
	/* We want to be sure we will release the request in the future */
	GEM_BUG_ON(!intel_gt_pm_is_awake(fpp_to_gt(fpp)));

	spin_lock(&fpp->bind_wq.lock);

	if (bo->wait)
		list_add_tail(&bo->link, &fpp->free_list);
	else
		list_add(&bo->link, &fpp->free_list);

	if (list_is_first(&bo->link, &fpp->free_list))
		wake_up_locked(&fpp->bind_wq);

	spin_unlock(&fpp->bind_wq.lock);
}

void intel_flat_ppgtt_pool_park(struct intel_flat_ppgtt_pool *fpp)
{
	struct intel_pte_bo *it;

	if (list_empty(&fpp->free_list))
		return;

	/* get_pte_bo/put_pte_bo are only called while holding the GT wakeref */
	list_for_each_entry_reverse(it, &fpp->free_list, link) {
		struct i915_request *rq;

		rq = fetch_and_zero(&it->wait);
		if (!rq) /* only process new entries to the free_list */
			break;

		if (!IS_ERR(rq))
			i915_request_put(rq);
	}
}

void intel_flat_ppgtt_pool_fini(struct intel_flat_ppgtt_pool *fpp)
{
	struct intel_gt *gt = container_of(fpp, struct intel_gt, fpp);
	struct intel_pte_bo *item, *temp;

	gt->i915->bind_ctxt_ready = false;

	list_for_each_entry_safe(item, temp, &fpp->free_list, link) {
		i915_vma_unpin_and_release(&item->vma, I915_VMA_RELEASE_MAP);
		if (!IS_ERR_OR_NULL(item->wait))
			i915_request_put(item->wait);
		kfree(item);
	}
	INIT_LIST_HEAD(&fpp->free_list);

	if (!list_empty(&fpp->prq_list)) {
		struct i915_request *rq, *rn;

		spin_lock(&fpp->rq_lock);
		list_for_each_entry_safe(rq, rn, &fpp->prq_list, fpp.gt_link) {
			list_del(&rq->fpp.vma_link);
			i915_request_free(rq);
		}
		INIT_LIST_HEAD(&fpp->prq_list);
		spin_unlock(&fpp->rq_lock);
	}

	/*
	 * Flush the i915 wq to ensure objects freed from above unpin on vm are
	 * all freed to avoid the race with below remove node which is called
	 * without any locking.
	 */
	i915_gem_drain_freed_objects(gt->i915);
}

int intel_flat_ppgtt_pool_init(struct intel_flat_ppgtt_pool *fpp,
			       struct i915_address_space *vm)
{
	struct intel_gt *gt = container_of(fpp, struct intel_gt, fpp);
	struct drm_i915_gem_object *obj;
	struct intel_pte_bo *item;
	int i, ret;

	if (!i915_is_mem_wa_enabled(gt->i915, I915_WA_USE_FLAT_PPGTT_UPDATE))
		return 0;

	for (i = 0; i < INTEL_FLAT_PPGTT_MAX_PINNED_OBJS; i++) {
		item = kmalloc(sizeof(*item), GFP_KERNEL);
		if (!item) {
			ret = -ENOMEM;
			goto err;
		}

		obj = i915_gem_object_create_internal(gt->i915,
						      INTEL_FLAT_PPGTT_BB_OBJ_SIZE);
		if (IS_ERR(obj)) {
			ret = PTR_ERR(obj);
			goto err_item;
		}

		item->vma = i915_vma_instance(obj, vm, NULL);
		if (IS_ERR(item->vma)) {
			ret = PTR_ERR(item->vma);
			goto err_obj;
		}

		item->cmd = i915_gem_object_pin_map_unlocked(obj, I915_MAP_WC);
		if (IS_ERR(item->cmd)) {
			ret = PTR_ERR(item->cmd);
			goto err_obj;
		}

		ret = i915_vma_pin(item->vma, 0, 0, PIN_USER | PIN_ZONE_48);
		if (ret)
			goto err_map;

		item->wait = NULL;
		list_add(&item->link, &fpp->free_list);
	}

	drm_info(&gt->i915->drm, "Using level-4 WA gt %d\n", gt->info.id);
	return 0;

err_map:
	i915_gem_object_unpin_map(obj);
err_obj:
	i915_gem_object_put(obj);
err_item:
	kfree(item);
err:
	intel_flat_ppgtt_pool_fini(fpp);
	return ret;
}

void intel_flat_ppgtt_pool_init_early(struct intel_flat_ppgtt_pool *fpp)
{
	init_waitqueue_head(&fpp->bind_wq);
	INIT_LIST_HEAD(&fpp->free_list);
	INIT_LIST_HEAD(&fpp->prq_list);
	spin_lock_init(&fpp->rq_lock);
}
