// SPDX-License-Identifier: MIT
/*
 * Copyright © 2020 Intel Corporation
 */

#include "i915_drv.h"
#include "i915_selftest.h"

#include "selftest_display.h"

int intel_display_live_selftests(struct drm_i915_private *i915)
{
	static const struct i915_subtest tests[] = {
		SUBTEST(intel_pcode_read_qgv_points_test),
	};

	return i915_subtests(tests, i915);
}
