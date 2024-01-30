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
selftest(sanitycheck, i915_live_sanitycheck) /* keep first (igt selfcheck) */
selftest(uncore, intel_uncore_live_selftests)
selftest(workarounds, intel_workarounds_live_selftests)
selftest(gt_engines, intel_engine_live_selftests)
selftest(gt_timelines, intel_timeline_live_selftests)
selftest(gt_ccs_mode, intel_gt_ccs_mode_live_selftests)
selftest(gt_contexts, intel_context_live_selftests)
selftest(gt_lrc, intel_lrc_live_selftests)
selftest(gt_mocs, intel_mocs_live_selftests)
selftest(gt_pm, intel_gt_pm_live_selftests)
selftest(gt_heartbeat, intel_heartbeat_live_selftests)
selftest(gt_gtt, intel_gtt_live_selftests)
selftest(gtt_l4wa, intel_gtt_l4wa_live_selftests)
selftest(gt_tlb, intel_tlb_live_selftests)
selftest(requests, i915_request_live_selftests)
selftest(active, i915_active_live_selftests)
selftest(objects, i915_gem_object_live_selftests)
selftest(shmem, i915_gem_shmem_live_selftests)
selftest(lmem, i915_gem_lmem_live_selftests)
selftest(mman, i915_gem_mman_live_selftests)
selftest(dmabuf, i915_gem_dmabuf_live_selftests)
selftest(vma, i915_vma_live_selftests)
selftest(coherency, i915_gem_coherency_live_selftests)
selftest(gtt, i915_gem_gtt_live_selftests)
selftest(gem, i915_gem_live_selftests)
selftest(obj_lock, i915_gem_obj_lock_live_selftests)
selftest(evict, i915_gem_evict_live_selftests)
selftest(hugepages, i915_gem_huge_page_live_selftests)
selftest(gem_contexts, i915_gem_context_live_selftests)
selftest(gem_execbuf, i915_gem_execbuffer_live_selftests)
selftest(client, i915_gem_client_blt_live_selftests)
selftest(reset, intel_reset_live_selftests)
selftest(display, intel_display_live_selftests)
selftest(memory_region, intel_memory_region_live_selftests)
selftest(memory_region_cross_tile, intel_memory_region_cross_tile_live_selftests)
selftest(remote_tiles, intel_remote_tiles_live_selftests)
selftest(hangcheck, intel_hangcheck_live_selftests)
selftest(execlists, intel_execlists_live_selftests)
selftest(ring_submission, intel_ring_submission_live_selftests)
#if IS_ENABLED(CONFIG_DRM_I915_DEBUGGER)
selftest(debugger, i915_debugger_live_selftests)
#endif
selftest(perf, i915_perf_live_selftests)
selftest(slpc, intel_slpc_live_selftests)
selftest(guc, intel_guc_live_selftests)
selftest(guc_multi_lrc, intel_guc_multi_lrc_live_selftests)
selftest(guc_hang, intel_guc_hang_check)
selftest(iov_adverse_events, selftest_live_iov_events)
selftest(iov_ggtt, intel_iov_ggtt_live_selftests)
selftest(iov_provisioning, selftest_live_iov_provisioning)
selftest(iov_relay, selftest_live_iov_relay)
selftest(semaphores, intel_semaphore_live_selftests)
selftest(guc_doorbells, intel_guc_doorbells_live_selftests)
selftest(iov_service, selftest_live_iov_service)
/* Here be dragons: keep last to run last! */
selftest(late_gt_pm, intel_gt_pm_late_selftests)
