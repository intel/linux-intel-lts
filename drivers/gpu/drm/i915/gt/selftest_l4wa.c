// SPDX-License-Identifier: MIT
/*
 * Copyright © 2021 Intel Corporation
 */

#include "i915_selftest.h"
#include "intel_gt_pm.h"

static int l4wa_sanitycheck(void *arg)
{
	struct intel_gt *gt = arg;
	struct drm_i915_gem_object *obj;
	intel_wakeref_t wakeref;
	struct i915_vma *vma;
	int err = 0;
	int count;

	obj = i915_gem_object_create_internal(gt->i915, SZ_4K);
	if (IS_ERR(obj))
		return PTR_ERR(obj);

	count = atomic_read(&gt->fpp.st.count);

	with_intel_gt_pm(gt, wakeref)
		vma = i915_gem_object_ggtt_pin(obj, gt->ggtt, NULL, 0, 0, 0);
	if (IS_ERR(vma))
		err = PTR_ERR(vma);

	if (err == 0 && atomic_read(&gt->fpp.st.count) == count) {
		pr_err("L4WA not used for active ggtt pin!\n");
		err = -EINVAL;
	}

	i915_gem_object_put(obj);
	return err;
}

int intel_gtt_l4wa_live_selftests(struct drm_i915_private *i915)
{
	static const struct i915_subtest tests[] = {
		SUBTEST(l4wa_sanitycheck),
	};
	struct intel_gt *gt;
	unsigned int i;

	if (!i915_is_mem_wa_enabled(i915, I915_WA_USE_FLAT_PPGTT_UPDATE))
		return 0;

	if (!i915->bind_ctxt_ready) {
		drm_err(&i915->drm,
			"L4WA not enabled, bind_ctxt_ready? %s!\n",
			str_yes_no(i915->bind_ctxt_ready));
		return -EINVAL;
	}

	for_each_gt(gt, i915, i) {
		int err;

		if (!gt->lmem)
			continue;

		if (!gt->engine[gt->rsvd_bcs]->bind_context) {
			drm_err(&i915->drm,
				"L4WA not setup on gt%d, no bind contexts!\n",
				gt->info.id);
			return -EINVAL;
		}

		if (intel_gt_is_wedged(gt))
			continue;

		err = intel_gt_live_subtests(tests, gt);
		if (err)
			return err;
	}

	return 0;
}
