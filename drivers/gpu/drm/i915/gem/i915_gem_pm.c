/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2019 Intel Corporation
 */

#include "gem/i915_gem_context.h"
#include "gem/i915_gem_pm.h"
#include "gt/intel_gt.h"
#include "gt/intel_gt_pm.h"
#include "gt/intel_gt_requests.h"

#include "i915_driver.h"
#include "i915_drv.h"
#include "i915_gem_evict.h"

#if defined(CONFIG_X86)
#include <asm/smp.h>
#else
#define wbinvd_on_all_cpus() \
	pr_warn(DRIVER_NAME ": Missing cache flush in %s\n", __func__)
#endif

static int perma_pinned_swapout(struct drm_i915_gem_object *obj)
{
	struct drm_i915_private *i915 = to_i915(obj->base.dev);
	struct drm_i915_gem_object *dst;
	int err = -EINVAL;

	assert_object_held(obj);
	dst = i915_gem_object_create_shmem(i915, obj->base.size);
	if (IS_ERR(dst))
		return PTR_ERR(dst);

	/* Temporarily borrow the lock */
	dst->base.resv = obj->base.resv;

	err = i915_gem_object_memcpy(dst, obj);
	if (!err)
		obj->swapto = dst;
	else
		i915_gem_object_put(dst);

	return err;
}

static int perma_pinned_swapin(struct drm_i915_gem_object *obj)
{
	struct drm_i915_gem_object *src;
	int err = -EINVAL;

	assert_object_held(obj);
	src = obj->swapto;

	err = i915_gem_object_memcpy(obj, src);
	if (!err) {
		obj->swapto = NULL;
		i915_gem_object_put(src);
	}

	return err;
}

static int lmem_suspend(struct drm_i915_private *i915)
{
	struct intel_memory_region *mem;
	int id;

	if (i915->quiesce_gpu)
		return 0;

	for_each_memory_region(mem, i915, id) {
		struct drm_i915_gem_object *obj;
		struct list_head *phases[] = {
			&mem->objects.purgeable,
			&mem->objects.list,
			NULL,
		}, **phase = phases;

		if (mem->type != INTEL_MEMORY_LOCAL)
			continue;

		/* singlethreaded suspend; list immutable */
		do list_for_each_entry(obj, *phase, mm.region.link) {
			int err;

			if (!i915_gem_object_has_pinned_pages(obj))
				continue;

			/* Skip dead objects, let their pages rot */
			if (!kref_get_unless_zero(&obj->base.refcount))
				continue;

			i915_gem_object_lock(obj, NULL);
			err = perma_pinned_swapout(obj);
			i915_gem_object_unlock(obj);

			i915_gem_object_put(obj);
			if (err)
				return err;
		} while (*++phase);
	}

	return 0;
}

static int lmem_resume(struct drm_i915_private *i915)
{
	struct intel_memory_region *mem;
	int id;

	for_each_memory_region(mem, i915, id) {
		struct drm_i915_gem_object *obj;
		struct list_head *phases[] = {
			&mem->objects.purgeable,
			&mem->objects.list,
			NULL,
		}, **phase = phases;

		if (mem->type != INTEL_MEMORY_LOCAL)
			continue;

		/* singlethreaded resume; list immutable */
		do list_for_each_entry(obj, *phase, mm.region.link) {
			int err;

			if (!obj->swapto ||
			    !i915_gem_object_has_pinned_pages(obj))
				continue;

			if (!kref_get_unless_zero(&obj->base.refcount))
				continue;

			i915_gem_object_lock(obj, NULL);
			err = perma_pinned_swapin(obj);
			i915_gem_object_unlock(obj);

			i915_gem_object_put(obj);
			if (err)
				return err;
		} while (*++phase);
	}

	return 0;
}

static void suspend_ppgtt_mappings(struct drm_i915_private *i915)
{
	struct i915_gem_context *ctx;

	rcu_read_lock();
	list_for_each_entry_rcu(ctx, &i915->gem.contexts.list, link) {
		struct i915_address_space *vm;

		if (!kref_get_unless_zero(&ctx->ref))
			continue;
		rcu_read_unlock();

		vm = i915_gem_context_get_eb_vm(ctx);
		if (vm) {
			mutex_lock(&vm->mutex);
			GEM_WARN_ON(i915_gem_evict_vm(vm));
			mutex_unlock(&vm->mutex);

			i915_vm_put(vm);
		}

		rcu_read_lock();
		i915_gem_context_put(ctx);
	}
	rcu_read_unlock();
}

void i915_gem_suspend(struct drm_i915_private *i915)
{
	struct intel_gt *gt;
	unsigned int i;

	GEM_TRACE("%s\n", dev_name(i915->drm.dev));

	intel_wakeref_auto(&to_gt(i915)->ggtt->userfault_wakeref, 0);
	flush_workqueue(i915->wq);

	i915_sriov_suspend_prepare(i915);

	/*
	 * We have to flush all the executing contexts to main memory so
	 * that they can saved in the hibernation image. To ensure the last
	 * context image is coherent, we have to switch away from it. That
	 * leaves the i915->kernel_context still active when
	 * we actually suspend, and its image in memory may not match the GPU
	 * state. Fortunately, the kernel_context is disposable and we do
	 * not rely on its state.
	 */
	for_each_gt(gt, i915, i)
		intel_gt_suspend_prepare(gt);

	suspend_ppgtt_mappings(i915);

	i915_gem_drain_freed_objects(i915);
}

int i915_gem_suspend_late(struct drm_i915_private *i915)
{
	struct drm_i915_gem_object *obj;
	struct list_head *phases[] = {
		&i915->mm.shrink_list,
		&i915->mm.purge_list,
		NULL
	}, **phase;
	struct intel_gt *gt;
	unsigned long flags;
	unsigned int i;
	bool flush = false;
	int err;

	/*
	 * Neither the BIOS, ourselves or any other kernel
	 * expects the system to be in execlists mode on startup,
	 * so we need to reset the GPU back to legacy mode. And the only
	 * known way to disable logical contexts is through a GPU reset.
	 *
	 * So in order to leave the system in a known default configuration,
	 * always reset the GPU upon unload and suspend. Afterwards we then
	 * clean up the GEM state tracking, flushing off the requests and
	 * leaving the system in a known idle state.
	 *
	 * Note that is of the upmost importance that the GPU is idle and
	 * all stray writes are flushed *before* we dismantle the backing
	 * storage for the pinned objects.
	 *
	 * However, since we are uncertain that resetting the GPU on older
	 * machines is a good idea, we don't - just in case it leaves the
	 * machine in an unusable condition.
	 */

	for_each_gt(gt, i915, i)
		intel_gt_suspend_late(gt);

	/* Swapout all the residual objects _after_ idling the firmware */
	err = lmem_suspend(i915);
	if (err)
		return err;

	spin_lock_irqsave(&i915->mm.obj_lock, flags);
	for (phase = phases; *phase; phase++) {
		list_for_each_entry(obj, *phase, mm.link) {
			if (!(obj->cache_coherent & I915_BO_CACHE_COHERENT_FOR_READ))
				flush |= (obj->read_domains & I915_GEM_DOMAIN_CPU) == 0;
			__start_cpu_write(obj); /* presume auto-hibernate */
		}
	}
	spin_unlock_irqrestore(&i915->mm.obj_lock, flags);
	if (flush)
		wbinvd_on_all_cpus();

	return 0;
}

int i915_gem_freeze(struct drm_i915_private *i915)
{
	/* Discard all purgeable objects, let userspace recover those as
	 * required after resuming.
	 */
	i915_gem_shrink_all(i915);

	return 0;
}

int i915_gem_freeze_late(struct drm_i915_private *i915)
{
	struct drm_i915_gem_object *obj;
	intel_wakeref_t wakeref;

	/*
	 * Called just before we write the hibernation image.
	 *
	 * We need to update the domain tracking to reflect that the CPU
	 * will be accessing all the pages to create and restore from the
	 * hibernation, and so upon restoration those pages will be in the
	 * CPU domain.
	 *
	 * To make sure the hibernation image contains the latest state,
	 * we update that state just before writing out the image.
	 *
	 * To try and reduce the hibernation image, we manually shrink
	 * the objects as well, see i915_gem_freeze()
	 */

	with_intel_runtime_pm(&i915->runtime_pm, wakeref)
		i915_gem_shrink(i915, -1UL, NULL, ~0);
	i915_gem_drain_freed_objects(i915);

	wbinvd_on_all_cpus();
	list_for_each_entry(obj, &i915->mm.shrink_list, mm.link)
		__start_cpu_write(obj);

	return 0;
}

void i915_gem_resume_early(struct drm_i915_private *i915)
{
	struct intel_gt *gt;
	unsigned int i;

	GEM_TRACE("%s\n", dev_name(i915->drm.dev));

	if (lmem_resume(i915))
		drm_err(&i915->drm,
			"failed to restore pinned objects in local memory\n");

	for_each_gt(gt, i915, i)
		intel_gt_resume_early(gt);
}

void i915_gem_resume(struct drm_i915_private *i915)
{
	struct intel_gt *gt;
	int i, j;

	GEM_TRACE("%s\n", dev_name(i915->drm.dev));

	/*
	 * As we didn't flush the kernel context before suspend, we cannot
	 * guarantee that the context image is complete. So let's just reset
	 * it and start again.
	 */
	for_each_gt(gt, i915, i) {
		if (intel_gt_resume(gt))
			goto err_wedged;

		/*
		 * FIXME: this should be moved to a delayed work because it
		 * takes too long, but for now we're doing it here as this is
		 * the easiest place to put it without doing throw-away work.
		 */
		intel_uc_init_hw_late(&gt->uc);
	}

	i915_sriov_resume(i915);

	return;

err_wedged:
	for_each_gt(gt, i915, j) {
		if (!intel_gt_is_wedged(gt)) {
			dev_err(i915->drm.dev,
				"Failed to re-initialize GPU[%u], declaring it wedged!\n",
				j);
			intel_gt_set_wedged(gt);
		}

		if (j == i)
			break;
	}
}

int i915_gem_idle_engines(struct drm_i915_private *i915)
{
	struct intel_gt *gt;
	unsigned int i;
	int ret = 0;

	if (!i915_is_mem_wa_enabled(i915, I915_WA_IDLE_GPU_BEFORE_UPDATE))
		return 0;

	/* Disable scheduling on engines */
	for_each_gt(gt, i915, i) {
		ret = intel_gt_idle_engines_start(gt, false);
		if (ret)
			break;
	}

	if (!ret) {
		for_each_gt(gt, i915, i)
			intel_gt_idle_engines_wait(gt);
	}

	return ret;
}

int i915_gem_resume_engines(struct drm_i915_private *i915)
{
	struct intel_gt *gt;
	unsigned int i;
	int ret = 0;

	if (!i915_is_mem_wa_enabled(i915, I915_WA_IDLE_GPU_BEFORE_UPDATE))
		return 0;

	/* Enable scheduling on engines */
	for_each_gt(gt, i915, i) {
		ret = intel_gt_idle_engines_start(gt, true);
		if (ret)
			break;
	}

	return ret;
}
