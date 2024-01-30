// SPDX-License-Identifier: MIT
/*
 * Copyright(c) 2022 Intel Corporation. All rights reserved.
 */

#include "i915_driver.h"
#include "gt/intel_context.h"
#include "gt/intel_gpu_commands.h"
#include "gt/intel_gt.h"
#include "gt/intel_ring.h"
#include "gt/iov/intel_iov_provisioning.h"
#include "selftests/igt_spinner.h"
#include "selftests/intel_scheduler_helpers.h"

static int generate_and_count_events(struct intel_iov *iov, enum intel_iov_threshold threshold,
				     int (*generate)(struct intel_gt *, unsigned int),
				     unsigned int generate_arg, u32 th_value, u32 sample_period,
				     bool track, bool loop, u32 expected_events)
{
	struct intel_gt *gt = iov_to_gt(iov);
	u32 events_before, reported_events;
	int err, i;

	GEM_BUG_ON(!intel_iov_is_pf(iov));
	GEM_BUG_ON(!sample_period);

	/* make sure that sampling is disabled before starting actual subtest */
	err = intel_iov_provisioning_set_sample_period(iov, 0);
	if (err) {
		IOV_SELFTEST_ERROR(iov, "failed to disable adverse event sampling (%pe)\n",
				   ERR_PTR(err));
		return err;
	}

	err = intel_iov_provisioning_set_threshold(iov, PFID, threshold, th_value);
	if (err) {
		IOV_SELFTEST_ERROR(iov, "failed to setup threshold %s to %u (%pe)\n",
				   intel_iov_threshold_to_string(threshold), th_value,
				   ERR_PTR(err));
		return err;
	}
	if (track) {
		err = intel_iov_provisioning_set_sample_period(iov, sample_period);
		if (err) {
			IOV_SELFTEST_ERROR(iov, "failed to setup sample period to %u ms (%pe)\n",
					   sample_period, ERR_PTR(err));
			return err;
		}
	}

	events_before = iov->pf.state.data[PFID].adverse_events[threshold];

	for (i = 0; i < 1 + loop; i++) {
		ktime_t start = ktime_get();
		s64 elapsed;

		err = generate(gt, generate_arg);
		if (err) {
			IOV_SELFTEST_ERROR(iov, "failed to generate adverse workload %s (%pe)\n",
					   intel_iov_threshold_to_string(threshold), ERR_PTR(err));
			return err;
		}

		elapsed = ktime_ms_delta(ktime_get(), start);
		if (elapsed > sample_period) {
			err = -ETIME;
			IOV_SELFTEST_ERROR(iov, "%s adverse workload generation time exceeded sample period (%pe)\n",
					   intel_iov_threshold_to_string(threshold), ERR_PTR(err));
			return err;
		}

		/* wait for the next sample period */
		mdelay(sample_period - elapsed);
	}

	reported_events = iov->pf.state.data[PFID].adverse_events[threshold] - events_before;

	err = intel_iov_provisioning_set_sample_period(iov, 0);
	if (err) {
		IOV_SELFTEST_ERROR(iov, "failed to disable adverse event sampling (%pe)\n",
				   ERR_PTR(err));
		return err;
	}

	err = intel_iov_provisioning_set_threshold(iov, PFID, threshold, 0);
	if (err) {
		IOV_SELFTEST_ERROR(iov, "failed to disable threshold %s (%pe)\n",
				   intel_iov_threshold_to_string(threshold), ERR_PTR(err));
		return err;
	}

	if (reported_events != expected_events) {
		IOV_SELFTEST_ERROR(iov, "threshold %s: invalid reported events %u (expected %u)\n",
				   intel_iov_threshold_to_string(threshold), reported_events,
				   expected_events);
		IOV_SELFTEST_ERROR(iov, "workload: %s, tracking enabled: %s, repeated sample window: %s\n",
				   expected_events ? "abuse" : "normal", str_yes_no(track),
				   str_yes_no(loop));

		return -ENXIO;
	}

	return 0;
}

static int generate_engine_resets(struct intel_gt *gt, unsigned int count)
{
	struct intel_engine_cs *engine = intel_selftest_find_any_engine(gt);
	struct intel_selftest_saved_policy saved;
	struct intel_context *ce;
	struct igt_spinner spin;
	struct i915_request *rq;
	int err, i;

	/* enable engine reset */
	err = intel_selftest_modify_policy(engine, &saved, SELFTEST_SCHEDULER_MODIFY_ENGINE_RESET);
	if (err)
		return err;

	err = igt_spinner_init(&spin, gt);
	if (err)
		goto out_policy;

	ce = intel_context_create(engine);
	if (IS_ERR(ce)) {
		err = PTR_ERR(ce);
		goto out_spin;
	}

	for (i = 0; i < count; i++) {
		rq = igt_spinner_create_request(&spin, ce, MI_NOOP);
		if (IS_ERR(rq)) {
			err = PTR_ERR(rq);
			goto out_ce;
		}

		i915_request_get(rq);
		i915_request_add(rq);

		if (!igt_wait_for_spinner(&spin, rq)) {
			err = -ETIME;
			goto out_rq;
		}

		/* wait for reset */
		err = intel_selftest_wait_for_rq(rq);
		if (err)
			goto out_rq;

		i915_request_put(rq);
	}

	goto out_ce;

out_rq:
	i915_request_put(rq);
out_ce:
	intel_context_put(ce);
out_spin:
	igt_spinner_fini(&spin);
out_policy:
	intel_selftest_restore_policy(engine, &saved);

	return err;
}

static int generate_h2g_interrupts(struct intel_gt *gt, unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; i++)
		intel_guc_notify(&gt->uc.guc);

	return 0;
}

static int generate_gt2g_interrupts(struct intel_gt *gt, unsigned int count)
{
	struct intel_engine_cs *engine;
	enum intel_engine_id id;
	struct i915_request *rq;
	u32 *cs;
	unsigned int i;

	for_each_engine(engine, gt, id) {
		rq = intel_context_create_request(engine->kernel_context);
		if (IS_ERR(rq))
			return PTR_ERR(rq);

		cs = intel_ring_begin(rq, count);
		if (IS_ERR(cs))
			return PTR_ERR(cs);

		for (i = 0; i < count; i++)
			*cs++ = MI_USER_INTERRUPT;

		intel_ring_advance(rq, cs);
		i915_request_get(rq);
		i915_request_add(rq);
		if (i915_request_wait(rq, 0, HZ) < 0) {
			i915_request_put(rq);
			return -ETIME;
		}
		i915_request_put(rq);
	}

	return 0;
}

/*
 * XXX: when workload generators are implemented for all thresholds, IOV_THRESHOLDS() should be
 * used instead.
 */
#define THRESHOLD_TCS(threshold) \
	threshold(ENGINE_RESET, engine_reset) \
	threshold(H2G_STORM, guc_storm) \
	threshold(IRQ_STORM, irq_storm) \
	/*end*/

#define __testcases(K, N) \
	TC(N, K, track, abuse, noloop, 1) \
	TC(N, K, track, abuse, loop, 2) \
	TC(N, K, track, normal, noloop, 0) \
	TC(N, K, track, normal, loop, 0) \
	TC(N, K, notrack, abuse, noloop, 0) \
	TC(N, K, notrack, abuse, loop, 0)

enum tc_workload { normal, abuse };
enum tc_tracking { notrack, track };
enum tc_loop { noloop, loop };

static const struct threshold_testcase {
	int (*generate)(struct intel_gt *gt, unsigned int arg);
	struct {
		unsigned int generate_arg;
		u32 th_value;
		u32 sample_period;
	} normal, abuse;
} tc[IOV_THRESHOLD_MAX] = {
	[IOV_THRESHOLD_ENGINE_RESET] = {
		.generate = generate_engine_resets,
		.normal = { .generate_arg = 1, .th_value = 2, .sample_period = 15000 },
		.abuse = { .generate_arg = 2, .th_value = 1, .sample_period = 15000 },
	},
	[IOV_THRESHOLD_H2G_STORM] = {
		.generate = generate_h2g_interrupts,
		.normal = { .generate_arg = 1000, .th_value = 2000, .sample_period = 100 },
		.abuse = { .generate_arg = 1000, .th_value = 100, .sample_period = 100 },
	},
	[IOV_THRESHOLD_IRQ_STORM] = {
		.generate = generate_gt2g_interrupts,
		.normal = { .generate_arg = 300, .th_value = 600, .sample_period = 100 },
		.abuse = { .generate_arg = 300, .th_value = 60, .sample_period = 100 },
	},
};

#define TC(NM, KEY, TR, WL, LP, C) \
static int NM##_##TR##_##WL##_##LP(void *arg) { \
	struct intel_iov *iov = arg; \
	\
	return generate_and_count_events(iov, IOV_THRESHOLD_##KEY, \
					 tc[IOV_THRESHOLD_##KEY].generate, \
					 tc[IOV_THRESHOLD_##KEY].WL.generate_arg, \
					 tc[IOV_THRESHOLD_##KEY].WL.th_value, \
					 tc[IOV_THRESHOLD_##KEY].WL.sample_period, \
					 TR, LP, C); \
}

THRESHOLD_TCS(__testcases)
#undef TC

int selftest_live_iov_events(struct drm_i915_private *i915)
{
	static const struct i915_subtest tests[] = {
#define TC(NM, KEY, TR, WL, LP, C) SUBTEST(NM##_##TR##_##WL##_##LP),
		THRESHOLD_TCS(__testcases)
#undef TC
	};
	intel_wakeref_t wakeref;
	int err = 0;

	if (!IS_SRIOV_PF(i915))
		return 0;

	if (i915_sriov_pf_status(i915) < 0)
		return -EHOSTDOWN;

	with_intel_runtime_pm(&i915->runtime_pm, wakeref) {
		struct intel_gt *gt;
		unsigned int id;

		for_each_gt(gt, i915, id) {
			struct intel_iov *iov = &gt->iov;

			pr_info(DRIVER_NAME ": Running subtests on gt%d\n", gt->info.id);
			err = intel_iov_provisioning_force_vgt_mode(iov);
			if (err)
				break;
			err = i915_subtests(tests, &gt->iov);
			if (err)
				break;
		}
	}

	return err;
}
