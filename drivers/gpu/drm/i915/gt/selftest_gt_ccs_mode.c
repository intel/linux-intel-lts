// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright © 2022 Intel Corporation
 */

#include "i915_selftest.h"

#include "intel_gpu_commands.h"
#include "intel_gt.h"
#include "intel_gt_pm.h"
#include "intel_engine_pm.h"

#include "selftests/i915_random.h"
#include "selftests/igt_reset.h"
#include "selftests/igt_spinner.h"
#include "selftests/intel_scheduler_helpers.h"

static int random_compute(struct intel_gt *gt,
			  struct intel_engine_cs *engines[],
			  int width,
			  struct rnd_state *prng)
{
	struct intel_engine_cs *all[MAX_ENGINE_INSTANCE + 1];
	struct intel_engine_cs **class = gt->engine_class[COMPUTE_CLASS];
	int i, j;

	for (i = j = 0; i <= MAX_ENGINE_INSTANCE; i++) {
		if (!class[i] || !(CCS_MASK(gt) & BIT(i)))
			continue;

		all[j++] = class[i];
	}
	if (j < width)
		return -EINVAL;

	i915_prandom_shuffle(all, sizeof(*all), j, prng);
	memcpy(engines, all, width * sizeof(*all));

	return 0;
}

static intel_engine_mask_t
random_config(struct intel_gt *gt, int width, struct rnd_state *prng)
{
	struct intel_engine_cs *engines[MAX_ENGINE_INSTANCE + 1];
	intel_engine_mask_t config;
	int i;

	if (random_compute(gt, engines, width, prng) < 0)
		return 0;

	pr_info("Using %d engines: ", width);
	config = 0;
	for (i = 0; i < width; i++) {
		pr_cont("%s%s", i ? ", " : "", engines[i]->name);
		config |= engines[i]->mask;
	}
	pr_cont(" -> config:%x\n", config);

	GEM_BUG_ON(hweight32(config) != width);
	return config;
}

static u32 apply_ccs_mode(struct intel_gt *gt, u32 config)
{
       u32 ccs_mode;

       mutex_lock(&gt->ccs.mutex);
       __intel_gt_apply_ccs_mode(gt, config);
       ccs_mode = intel_uncore_read(gt->uncore, XEHP_CCS_MODE);
       pr_info("GT%d (config:%08x): CCS_MODE:%x\n",
               gt->info.id, config, ccs_mode);
       mutex_unlock(&gt->ccs.mutex);

       return ccs_mode;
}

static int
live_ccs_mode(struct intel_gt *gt, int num_engines, struct rnd_state *prng)
{
	u32 count[MAX_ENGINE_INSTANCE + 1] = {};
	intel_engine_mask_t config;
	intel_wakeref_t wf;
	int slices_per_engine;
	u32 ccs_mode;
	int err = 0;
	int i;

	/*
	 * Check that we configure the CCS mode for the respective number of
	 * slices per engine for different configurations
	 */

	config = random_config(gt, num_engines, prng);
	if (!config)
		return 0;

	wf = intel_gt_pm_get(gt);

	ccs_mode = apply_ccs_mode(gt, config);
	for (i = 0; i < PVC_NUM_CSLICES_PER_TILE; i++) {
		int inst = (ccs_mode >> (XEHP_CCS_MODE_CSLICE_WIDTH * i)) & XEHP_CCS_MODE_CSLICE_MASK;
		bool disabled = inst == XEHP_CCS_MODE_CSLICE_MASK;

		pr_info("slice:%d, instance=%d%s\n", i, inst,
			disabled ? " (disabled)" : "");
		if (disabled)
			continue; /* disabled */

		if (!(CCS_MASK(gt) & BIT(inst))) {
			pr_err("CCS_MODE:%08x: unknown instance:%d enabled for slice:%d slice",
			       ccs_mode, inst, i);
			err = -EINVAL;
		}

		count[inst]++;
	}

	slices_per_engine = hweight32(CCS_MASK(gt)) / num_engines;
	for (i = 0; i < ARRAY_SIZE(count); i++) {
		if (!(config & BIT(i + CCS0)))
			continue;

		if (count[i] != slices_per_engine) {
			pr_err("ccs%d owns %d slices, expected %d; config requested:%x, result:%x\n",
			       i, count[i], slices_per_engine,
			       config, gt->ccs.config);
			err = -EINVAL;
		}
	}

	intel_gt_pm_put(gt, wf);
	return err;
}

static int live_ccs_mode_1(void *arg)
{
	I915_RND_STATE(prng);

	return live_ccs_mode(arg, 1, &prng);
}

static int live_ccs_mode_2(void *arg)
{
	I915_RND_STATE(prng);

	return live_ccs_mode(arg, 2, &prng);
}

static int live_ccs_mode_3(void *arg)
{
	I915_RND_STATE(prng);

	return live_ccs_mode(arg, 3, &prng);
}

static int live_ccs_mode_4(void *arg)
{
	I915_RND_STATE(prng);

	return live_ccs_mode(arg, 4, &prng);
}

static int random_bit(unsigned long mask, struct rnd_state *prng)
{
	int num_bits = hweight_long(mask);
	int i;

	if (!num_bits)
		return BITS_PER_LONG;

	num_bits = i915_prandom_u32_max_state(num_bits, prng);
	for_each_set_bit(i, &mask, BITS_PER_LONG) {
		if (num_bits-- == 0)
			return i;
	}

	return BITS_PER_LONG;
}

static int live_ccs_fused(void *arg)
{
	struct intel_gt *gt = arg;
	intel_engine_mask_t saved = gt->info.engine_mask;
	unsigned long engines = saved & GENMASK(CCS0 + I915_MAX_CCS, CCS0);
	I915_RND_STATE(prng);
	int num_slices = 0;
	int err = 0;
	int n;

	/* Check the CCS_MODE computation with randomly fused slices */

	gt->info.engine_mask &= ~engines;
	while ((n = random_bit(engines, &prng)) != BITS_PER_LONG) {
		pr_info("GT%d, enabling slice/engine %d\n",
			gt->info.id, n - CCS0);
		gt->info.engine_mask |= BIT(n);
		engines &= ~BIT(n);
		num_slices++;

		for (n = 1; n <= num_slices; n++) {
			err = live_ccs_mode(arg, n, &prng);
			if (err)
				goto out;
		}
	}

out:
	gt->info.engine_mask = saved;
	return err;
}

static int live_ccs_active(void *arg)
{
	struct intel_gt *gt = arg;
	struct intel_engine_cs *engine;
	enum intel_engine_id id;
	intel_wakeref_t wf;
	int err = 0;

	/* If any CCS engine is active then we cannot change mode */

	wf = intel_gt_pm_get(gt);

	gt->ccs.config = 0;

	for_each_engine(engine, gt, id) {
		int ret;

		if (engine->class != COMPUTE_CLASS)
			continue;

		pr_info("Trying to enable ALL_CCS for %s\n", engine->name);
		intel_engine_pm_get(engine);
		gt->ccs.active = ALL_CCS(gt) & ~engine->mask;
		ret = intel_gt_configure_ccs_mode(gt, ALL_CCS(gt));
		intel_engine_pm_put(engine);

		if (ret != -EBUSY) {
			pr_err("%s: Did not report busy on trying to change CCS mode with active engines, err:%d\n",
			       engine->name, ret);
			err = -EINVAL;
		}
		if (gt->ccs.config) {
			pr_err("%s: CCS mode changed despite active:%08x engines: config:%08x",
			       engine->name, gt->ccs.active, gt->ccs.config);
			gt->ccs.config = 0;
			err = -EINVAL;
		}

		/* We should mark the engine as idle when releasing the wf */
		GEM_BUG_ON(gt->ccs.active & engine->mask);
	}

	/* All engines should now be idle */
	intel_gt_park_ccs_mode(gt, NULL);

	if (err == 0) {
		err = intel_gt_configure_ccs_mode(gt, ALL_CCS(gt));
		if (err) {
			pr_err("Failed to configure CCS mode while idle, active:%x, err:%d\n",
			       gt->ccs.active, err);
			err = -EINVAL;
		}

		if (gt->ccs.config != ALL_CCS(gt)) {
			pr_err("Failed to configure CCS mode while idle, config:%x\n",
			       gt->ccs.config);
			err = -EINVAL;
		}
	}

	intel_gt_pm_put(gt, wf);

	intel_gt_pm_wait_for_idle(gt);
	if (gt->ccs.config) {
		pr_err("Failed to reset CCS config on idling, config:%x\n",
		       gt->ccs.config);
		err = -EINVAL;
	}

	return err;
}

static int live_ccs_reactive(void *arg)
{
	struct intel_gt *gt = arg;
	struct intel_engine_cs *engine;
	enum intel_engine_id id;
	intel_wakeref_t wf;
	int err = 0;

	/*
	 * Check that when use a single engine within a configuration,
	 * we always make that engine as active in the CCS mode.
	 */

	wf = intel_gt_pm_get(gt);

	gt->ccs.active = 0;
	gt->ccs.config = ALL_CCS(gt);

	for_each_engine(engine, gt, id) {
		if (engine->class != COMPUTE_CLASS)
			continue;

		pr_info("Re-enabling CCS config:%x for %s\n",
			gt->ccs.config, engine->name);
		GEM_BUG_ON(!(gt->ccs.config & engine->mask));
		WRITE_ONCE(gt->ccs.active, 0);

		intel_engine_pm_get(engine);

		/* ccs does not become active until we configure the mode */
		GEM_BUG_ON(gt->ccs.active);

		err = intel_gt_configure_ccs_mode(gt, engine->mask);
		if (err) {
			pr_err("%s: Reported busy on trying to sub-select active CCS mode, err:%d\n",
			       engine->name, err);
			err = -EINVAL;
		}

		if ((gt->ccs.active & engine->mask) == 0) {
			pr_err("%s: CCS not marked active:%x for current engine\n",
			       engine->name, gt->ccs.active);
			err = -EINVAL;
		}

		intel_engine_pm_put(engine);
	}

	intel_gt_pm_put(gt, wf);

	return err;
}

static int live_ccs_gt_reset(struct intel_gt *gt, bool active)
{
	intel_engine_mask_t config, tmp;
	struct intel_engine_cs *engine;
	I915_RND_STATE(prng);
	u32 before, after;
	int err = 0;

	/*
	 * After a reset we expect the CCS mode to be restored for the
	 * currently active configuration. If idle, before the next
	 * HW execution.
	 */

	if (!intel_has_gpu_reset(gt))
		return 0;

	config = random_config(gt, 2, &prng);
	if (!config)
		return 0;

	for_each_engine_masked(engine, gt, config, tmp)
		intel_engine_pm_get(engine);

	err = intel_gt_configure_ccs_mode(gt, config);
	if (err) {
		pr_err("Failed to setup config:%08x, ccs.config:%08x, ccs.active:%08x\n",
		       config, gt->ccs.config, gt->ccs.active);
		err = -EINVAL;
		goto out;
	}
	GEM_BUG_ON(gt->ccs.config != config);
	GEM_BUG_ON(gt->ccs.active != config);

	/* We want a non-trivial configuration */
	before = intel_uncore_read(gt->uncore, XEHP_CCS_MODE);
	GEM_BUG_ON((before ^ XEHP_CCS_MODE_CSLICE_0_3_MASK) == 0);

	if (!active)
		intel_gt_park_ccs_mode(gt, NULL); /* force idling */

	igt_global_reset_lock(gt);
	intel_gt_reset(gt, config, "CCS mode");
	igt_global_reset_unlock(gt);

	if (active) {
		/*
		 * With an active reset, computation is expected to continue
		 * immediately following the reset, so CCS_MODE must be restored
		 * for the compute kernels to function.
		 */
		after = intel_uncore_read(gt->uncore, XEHP_CCS_MODE);
		if (after != before) {
			pr_err("CCS_MODE lost across an active GT reset, before:%08x, after:%08x\n",
			       before, after);
			err = -EINVAL;
		}
	}

	/* A new submission after the reset should always restore CCS_MODE */
	err = intel_gt_configure_ccs_mode(gt, config);
	if (err) {
		pr_err("Failed to reapply config:%08x after reset, ccs.config:%08x, ccs.active:%08x\n",
		       config, gt->ccs.config, gt->ccs.active);
		err = -EINVAL;
	}

	after = intel_uncore_read(gt->uncore, XEHP_CCS_MODE);
	if (after != before) {
		pr_err("CCS_MODE lost across on resubmission after GT reset, before:%08x, after:%08x\n",
		       before, after);
		err = -EINVAL;
	}

out:
	for_each_engine_masked(engine, gt, config, tmp)
		intel_engine_pm_put(engine);

	return err;
}

static int live_ccs_gt_reset_active(void *arg)
{
       return live_ccs_gt_reset(arg, true);
}

static int live_ccs_gt_reset_idle(void *arg)
{
       return live_ccs_gt_reset(arg, false);
}

static int live_ccs_sim_reset(void *arg)
{
	struct intel_gt *gt = arg;
	struct intel_engine_cs *engine;
	u32 expected, ccs_mode;
	intel_wakeref_t wf;
	int err = 0;

	/*
	 * Simulate a race condition where a user submits a new compute
	 * execbuf concurrent with a GT reset. Do we write the expected
	 * CCS_MODE prior to re-enabling HW submission?
	 */

	engine = gt->engine_class[COMPUTE_CLASS][__ffs(CCS_MASK(gt))];
	GEM_BUG_ON(!engine);

	with_intel_gt_pm(gt, wf)
		expected = apply_ccs_mode(gt, engine->mask);
	intel_gt_pm_wait_for_idle(gt);
	GEM_BUG_ON(gt->ccs.config | gt->ccs.active); /* no cheating! */

	intel_engine_pm_get(engine);
	err = intel_gt_configure_ccs_mode(gt, engine->mask);
	if (err)
		goto out;

	/* After configuring a fresh config, it must be currently tracked */
	GEM_BUG_ON(!gt->ccs.config);
	GEM_BUG_ON(!gt->ccs.active);

	/*
	 * A: Simulate a reset, leaving it exposed to a race with execbuf.
	 *
	 * The GT (engine) reset will reset the CCS_MODE to default (all
	 * disabled). The normal intel_gt_reset() will try to restore the
	 * CCS_MODE (via intel_gt_init_hw), but since we are looking at the
	 * race condition between the reset and execbuf, we leave it open.
	 *
	 * Serialisation between execbuf and the point restoring CCS_MODE
	 * inside reset is handled by gt->ccs.mutex;
	 */
	intel_uncore_write(gt->uncore,
			   XEHP_CCS_MODE,
			   XEHP_CCS_MODE_CSLICE_0_3_MASK);

	/*
	 * B: Simulate execbuf; concurrent with the reset.
	 *
	 * We do not attempt to change config, so the configure will
	 * attempt to reuse the existing CCS_MODE.
	 */
	err = intel_gt_configure_ccs_mode(gt, engine->mask);
	GEM_BUG_ON(err);
	if (!(gt->ccs.active & engine->mask)) {
		pr_err("GT%d(%s) ccs.active:%08x not set after configuring for an identical config:%08x\n",
		       gt->info.id, engine->name, gt->ccs.active, gt->ccs.config);
		err = -EINVAL;
		goto out;
	}

	/*
	 * A: Simulate intel_init_gt_hw()
	 *
	 * intel_init_gt_hw is called (after reset) to prepare the HW before
	 * submission. That calls intel_gt_apply_ccs_mode().
	 */
	intel_gt_apply_ccs_mode(gt);

	ccs_mode = intel_uncore_read(gt->uncore, XEHP_CCS_MODE);
	if (ccs_mode != expected) {
		pr_err("GT%d(%s) CCS_MODE:%x not restored to expected:%x after simulated reset\n",
		       gt->info.id, engine->name, ccs_mode, expected);
		err = -EINVAL;
	}

	/*
	 * Now the real intel_gt_reset will re-enable HW command submission.
	 *
	 * We expect the CCS_MODE will be kept now until all current compute
	 * activity ceases.
	 [3: thread] [gfx-internal-devel] [PATCH] ccs[124] -- the missing patch                                                 */
	if (!(gt->ccs.config & engine->mask)) {
		pr_err("GT%d(%s) config:%08x does not contain the submission engine:%08x\n",
		       gt->info.id, engine->name, gt->ccs.config, engine->mask);
		err = -EINVAL;
	}
	if (!(gt->ccs.active & engine->mask)) {
		pr_err("GT%d(%s) active:%08x does not contain the submission engine:%08x\n",
		       gt->info.id, engine->name, gt->ccs.active, engine->mask);
		err = -EINVAL;
	}

out:
	intel_engine_pm_put(engine);
	return err;
}

static int live_ccs_engine_reset(void *arg)
{
	struct intel_gt *gt = arg;
	intel_engine_mask_t config, tmp;
	struct intel_engine_cs *engine;
	struct igt_spinner spin;
	I915_RND_STATE(prng);
	intel_wakeref_t wf;
	u32 ccs_mode;
	int err;

	/*
	 * Check that the CCS_MODE is restored for an engine reset,
	 * as computation is expected to continue without intervention
	 * from the driver (for GuC mediated resets).
	 */

	config = random_config(gt, 2, &prng);
	if (!config)
		return 0;

	wf = intel_gt_pm_get(gt);
	igt_global_reset_lock(gt);

	err = igt_spinner_init(&spin, gt);
	if (err)
		goto out;

	ccs_mode = apply_ccs_mode(gt, config);
	for_each_engine_masked(engine, gt, config, tmp) {
		struct intel_selftest_saved_policy saved;
		struct intel_context *ce;
		struct i915_request *rq;

		err = intel_selftest_modify_policy(engine, &saved,
						   SELFTEST_SCHEDULER_MODIFY_FAST_RESET);
		if (err)
			break;

		ce = intel_context_create(engine);
		if (IS_ERR(ce)) {
			err = PTR_ERR(ce);
			goto restore;
		}

		rq = igt_spinner_create_request(&spin, ce, MI_NOOP);
		if (IS_ERR(rq)) {
			err = PTR_ERR(rq);
			goto err_ce;
		}

		i915_request_get(rq);
		i915_request_add(rq);
		if (!igt_wait_for_spinner(&spin, rq)) {
			err = -ETIMEDOUT;
			goto err_rq;
		}

		/* Ensure the spinner hasn't aborted */
		if (i915_request_completed(rq)) {
			err = -EIO;
			goto err_rq;
		}

		err = intel_selftest_wait_for_rq(rq);
		if (err == 0) {
			u32 after = intel_uncore_read(gt->uncore, XEHP_CCS_MODE);

			if (after != ccs_mode) {
				pr_err("CCS mode configuration lost across a engine reset (%s), before:%08x, after:%08x\n",
				       ce->engine->name, ccs_mode, after);
				err = -EINVAL;
			}
		}
err_rq:
		i915_request_put(rq);
		igt_spinner_end(&spin);
err_ce:
		intel_context_put(ce);
restore:
		intel_selftest_restore_policy(engine, &saved);
		if (err)
			break;
	}

out:
	igt_spinner_fini(&spin);
	igt_global_reset_unlock(gt);
	intel_gt_pm_put(gt, wf);
	return err;
}

int intel_gt_ccs_mode_live_selftests(struct drm_i915_private *i915)
{
	static const struct i915_subtest tests[] = {
		SUBTEST(live_ccs_mode_1),
		SUBTEST(live_ccs_mode_2),
		SUBTEST(live_ccs_mode_3),
		SUBTEST(live_ccs_mode_4),
		SUBTEST(live_ccs_fused),
		SUBTEST(live_ccs_active),
		SUBTEST(live_ccs_reactive),
		SUBTEST(live_ccs_gt_reset_active),
		SUBTEST(live_ccs_gt_reset_idle),
		SUBTEST(live_ccs_engine_reset),
		SUBTEST(live_ccs_sim_reset),
	};
	struct intel_gt *gt;
	int i;

	for_each_gt(gt, i915, i) {
		int err;

		if (intel_gt_is_wedged(gt))
			continue;

		if (!CCS_MASK(gt))
			continue;

		if (!needs_ccs_mode(gt))
			continue;

		err =  intel_gt_live_subtests(tests, gt);
		if (err)
			return err;
	}

	return 0;
}
