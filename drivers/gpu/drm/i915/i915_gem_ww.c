// SPDX-License-Identifier: MIT
/*
 * Copyright © 2020 Intel Corporation
 */

#include <linux/dma-resv.h>

#include "gem/i915_gem_object.h"

#include "i915_gem_ww.h"
#include "intel_memory_region.h"

void i915_gem_ww_ctx_init(struct i915_gem_ww_ctx *ww, bool intr)
{
	ww_acquire_init(&ww->ctx, &reservation_ww_class);
	INIT_LIST_HEAD(&ww->obj_list);

	ww->intr = intr;
	ww->contended = NULL;
	ww->contended_evict = false;
}

static void
__update_lru(struct drm_i915_gem_object *obj, struct intel_memory_region *mem)
{
	struct list_head *list;

	if (!i915_gem_object_has_pages(obj)) {
		INIT_LIST_HEAD(&obj->mm.region.link);
		return;
	}

	if (obj->mm.madv != I915_MADV_WILLNEED)
		list = &mem->objects.purgeable;
	else
		list = &mem->objects.list;

	list_add_tail(&obj->mm.region.link, list);
}

static void update_lru(struct drm_i915_gem_object *obj)
{
	struct intel_memory_region *mem;

	mem = obj->mm.region.mem;
	if (!mem)
		return;

	spin_lock(&mem->objects.lock);
	__list_del_entry(&obj->mm.region.link);
	__update_lru(obj, mem);
	spin_unlock(&mem->objects.lock);
}

static void put_obj_list(struct list_head *list)
{
	struct drm_i915_gem_object *obj, *next;

	list_for_each_entry_safe(obj, next, list, obj_link) {
		i915_gem_object_unlock(obj);
		i915_gem_object_put(obj);
	}
	INIT_LIST_HEAD(list);
}

static void i915_gem_ww_ctx_unlock_all(struct i915_gem_ww_ctx *ww)
{
	put_obj_list(&ww->obj_list);
}

void i915_gem_ww_unlock_single(struct drm_i915_gem_object *obj)
{
	update_lru(obj);
	list_del(&obj->obj_link);
	i915_gem_object_unlock(obj);
	i915_gem_object_put(obj);
}

void i915_gem_ww_ctx_fini(struct i915_gem_ww_ctx *ww)
{
	i915_gem_ww_ctx_unlock_all(ww);
	GEM_BUG_ON(ww->contended);
	ww_acquire_fini(&ww->ctx);
}

int __must_check i915_gem_ww_ctx_backoff(struct i915_gem_ww_ctx *ww)
{
	struct drm_i915_gem_object *obj = ww->contended;
	int ret = 0;

	if (GEM_WARN_ON(!obj))
		return -EINVAL;

	i915_gem_ww_ctx_unlock_all(ww);
	if (ww->intr)
		ret = dma_resv_lock_slow_interruptible(obj->base.resv, &ww->ctx);
	else
		dma_resv_lock_slow(obj->base.resv, &ww->ctx);
	if (ret) {
		i915_gem_object_put(obj);
		goto out;
	}

	/*
	 * Unlocking the contended lock again, if it was locked for eviction.
	 * We will most likely not need it in the retried transaction.
	 */
	if (ww->contended_evict) {
		dma_resv_unlock(obj->base.resv);
		i915_gem_object_put(obj);
	} else {
		obj->evict_locked = false;
		list_add_tail(&obj->obj_link, &ww->obj_list);
	}

out:
	ww->contended = NULL;
	return ret;
}

int
__i915_gem_object_lock_to_evict(struct drm_i915_gem_object *obj,
				struct i915_gem_ww_ctx *ww)
{
	int err;

	if (ww)
		err = dma_resv_lock_interruptible(obj->base.resv, &ww->ctx);
	else
		err = dma_resv_trylock(obj->base.resv) ? 0 : -EBUSY;
	if (err == -EDEADLK) {
		ww->contended_evict = true;
		ww->contended = i915_gem_object_get(obj);
	}

	return err;
}
