// SPDX-License-Identifier: MIT
/*
 * Copyright © 2019 Intel Corporation
 */

#include "../i915_selftest.h"

#include "gem/i915_gem_region.h"

static int igt_lmem_create_remote_tiles(void *arg)
{
	struct i915_gem_context *ctx = arg;
	struct drm_i915_private *i915 = ctx->i915;
	int i, n = 0, err = 0;

	/* test if we can create gem objects from remote tiles */
	static const u32 remote_tiles[] = { INTEL_REGION_LMEM_1 };
	struct drm_i915_gem_object *remote_obj[ARRAY_SIZE(remote_tiles)];

	memset(remote_obj, 0, sizeof(remote_obj));
	for (i = 0; i < ARRAY_SIZE(remote_tiles); ++i) {
		struct intel_memory_region *mem = i915->mm.regions[remote_tiles[i]];
		struct drm_i915_gem_object *obj;

		if (!mem)
			continue;

		obj = i915_gem_object_create_region(mem, PAGE_SIZE, 0);
		if (IS_ERR(obj))
			return PTR_ERR(obj);

		pr_info("Creating gem object from tile[%d]\n", i);
		remote_obj[n++] = obj;
		err = i915_gem_object_pin_pages_unlocked(obj);
		if (err)
			goto out_put;

		i915_gem_object_unpin_pages(obj);
	}
out_put:
	for (i = 0; i < n; ++i)
		i915_gem_object_put(remote_obj[i]);

	return err;
}

int intel_remote_tiles_live_selftests(struct drm_i915_private *i915)
{
	static const struct i915_subtest tests[] = {
		SUBTEST(igt_lmem_create_remote_tiles),
	};
	struct intel_runtime_pm *rpm = &i915->runtime_pm;
	struct i915_gem_context *ctx;
	struct file *file;
	intel_wakeref_t wakeref;
	int err;

	if (!HAS_LMEM(i915)) {
		pr_info("device lacks LMEM support, skipping\n");
		return 0;
	}

	if (!HAS_REMOTE_TILES(i915)) {
		pr_info("device lacks remote tile support, skipping\n");
		return 0;
	}

	if (intel_gt_is_wedged(to_gt(i915)))
		return 0;

	file = mock_file(i915);
	if (IS_ERR(file))
		return PTR_ERR(file);

	mutex_lock(&i915->drm.struct_mutex);
	wakeref = intel_runtime_pm_get(rpm);

	ctx = live_context(i915, file);
	if (IS_ERR(ctx)) {
		err = PTR_ERR(ctx);
		goto out_unlock;
	}

	err = i915_subtests(tests, ctx);

out_unlock:
	intel_runtime_pm_put(rpm, wakeref);
	mutex_unlock(&i915->drm.struct_mutex);
	fput(file);

	return err;
}
