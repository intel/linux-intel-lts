// SPDX-License-Identifier: MIT
/*
 * Copyright © 2020 Intel Corporation
 */

#include <linux/sched/mm.h>

#include "i915_selftest.h"

#include "gt/intel_engine_pm.h"
#include "selftests/igt_flush_test.h"
#include "selftests/igt_spinner.h"

static int suspend_request(struct intel_gt *gt, struct intel_context *ce,
			   struct igt_spinner *spin)

{
	struct i915_request *rq;
	struct intel_engine_cs *engine = ce->engine;
	struct i915_suspend_fence *sfence;
	signed long timeout;
	struct i915_address_space *vm = ce->vm;
	int err = 0;
	struct dma_fence *fence;
	unsigned long delay = HZ / 2;

	pr_info("Running suspend test on engine %s.\n", engine->name);
	sfence = kzalloc(sizeof(*sfence), GFP_KERNEL);
	if (!sfence)
		return -ENOMEM;

	rq = igt_spinner_create_request(spin, ce, MI_ARB_CHECK);
	if (IS_ERR(rq)) {
		kfree(sfence);
		return PTR_ERR(rq);
	}

	set_bit(I915_VM_HAS_PERSISTENT_BINDS, &vm->flags);
	set_bit(I915_FENCE_FLAG_LR, &rq->fence.flags);
	init_and_set_context_suspend_fence(ce, sfence);
	sfence = NULL;
	i915_request_get(rq);

	dma_fence_enable_sw_signaling(&ce->sfence->base.rq.fence);
	fence = dma_fence_get(&ce->sfence->base.rq.fence);
	i915_request_add(rq);

	timeout = dma_fence_wait_timeout(fence, true, delay);
	dma_fence_put(fence);
	if (timeout == 0) {
		pr_err("%s: Initial request suspension timeout.\n",
		       engine->name);
		err = -ETIME;
		goto out;
	}
	if (timeout < 0) {
		err = timeout;
		goto out;
	}

	if (!igt_wait_for_spinner(spin, rq)) {
		pr_err("%s: spinner didn't start when submitted.\n",
		       engine->name);
		intel_gt_set_wedged(gt);
		err = -ETIME;
		goto out;
	}

	engine = READ_ONCE(rq->engine);
	/* We have our request executing, now suspend it. */

	i915_vm_lock_objects(vm, NULL);

	fs_reclaim_acquire(GFP_KERNEL);
	timeout = dma_fence_wait_timeout(&ce->sfence->base.rq.fence, true, delay);
	fs_reclaim_release(GFP_KERNEL);
	if (timeout <= 0) {
		pr_err("%s: Suspend running request timeout.\n", engine->name);
		i915_gem_object_unlock(vm->root_obj);
		err = timeout ?: -ETIME;
		goto out;
	}

	/*
	 * The request is now either fully suspended or complete.
	 * Since it's a spinner, we do assume it's not complete.
	 * Have the spinner terminate when it is resumed.
	 */
	igt_spinner_end(spin);

	/* Check that the suspended request did not complete */
	msleep(200);
	if (i915_request_completed(rq)) {
		pr_err("%s: suspended request completed!\n",
		       engine->name);
		i915_gem_object_unlock(vm->root_obj);
		err = -EIO;
		goto out;
	}

	/* But completes on resume */
	i915_gem_object_unlock(vm->root_obj);
	if (i915_request_wait(rq, 0, delay) < 0) {
		pr_err("%s: resumed request did not complete!\n",
		       engine->name);
		intel_gt_set_wedged(gt);
		err = -ETIME;
	}
out:
	clear_bit(I915_VM_HAS_PERSISTENT_BINDS, &vm->flags);
	igt_spinner_end(spin);
	i915_request_put(rq);

	return err;
}

static int live_suspend_fence(void *arg)
{
	struct drm_i915_private *i915 = arg;
	struct intel_gt *gt = to_gt(i915);
	struct intel_engine_cs *engine;
	enum intel_engine_id id;
	struct igt_spinner spin;
	int err = 0;

	if (!(gt->i915->caps.scheduler & I915_SCHEDULER_CAP_PREEMPTION))
		return 0;

	if (!intel_uc_uses_guc_submission(&gt->uc))
		return 0;

	if (igt_spinner_init(&spin, gt))
		return -ENOMEM;

	for_each_engine(engine, gt, id) {
		struct intel_context *ce;

		if (!intel_engine_has_preemption(engine))
			continue;

		ce = intel_context_create(engine);
		if (IS_ERR(ce)) {
			err = PTR_ERR(ce);
			break;
		}

		err = suspend_request(gt, ce, &spin);
		intel_context_put(ce);
		if (err)
			break;
	}

	igt_spinner_fini(&spin);
	return err;
}


int i915_gem_execbuffer_live_selftests(struct drm_i915_private *i915)
{
	static const struct i915_subtest tests[] = {
		SUBTEST(live_suspend_fence),
	};

	if (intel_gt_is_wedged(to_gt(i915)))
		return 0;

	return i915_live_subtests(tests, i915);
}
