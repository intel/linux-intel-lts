/* SPDX-License-Identifier: GPL-2.0 */

#ifndef selftest
#define selftest(x, y)
#endif

/*
 * List each unit test as selftest(name, function)
 *
 * The name is used as both an enum and expanded as subtest__name to create
 * a module parameter. It must be unique and legal for a C identifier.
 *
 * The function should be of type int function(void). It may be conditionally
 * compiled using #if IS_ENABLED(CONFIG_DRM_I915_SELFTEST).
 *
 * Tests are executed in order by igt/i915_selftest
 */
selftest(sanitycheck, i915_wip_sanitycheck) /* keep first (igt selfcheck) */
selftest(gt_gtt, intel_gtt_wip_selftests)
selftest(lmem, i915_gem_lmem_wip_selftests)
selftest(live_scheduler, i915_scheduler_live_selftests)
selftest(perf_scheduler, i915_scheduler_perf_selftests)
