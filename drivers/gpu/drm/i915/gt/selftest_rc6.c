// SPDX-License-Identifier: MIT
/*
 * Copyright © 2019 Intel Corporation
 */

#include "intel_context.h"
#include "intel_engine_pm.h"
#include "intel_gpu_commands.h"
#include "intel_gt_requests.h"
#include "intel_ring.h"
#include "selftest_rc6.h"

#include "selftests/i915_random.h"
#include "selftests/librapl.h"

static u64 rc6_residency(struct intel_rc6 *rc6)
{
	struct intel_gt *gt = rc6_to_gt(rc6);
	i915_reg_t reg;
	u64 result;

	/* XXX VLV_GT_MEDIA_RC6? */

	if (gt->type == GT_MEDIA)
		reg = MTL_MEDIA_MC6;
	else
		reg = GEN6_GT_GFX_RC6;

	result = intel_rc6_residency_ns(rc6, reg);
	if (HAS_RC6p(rc6_to_i915(rc6)))
		result += intel_rc6_residency_ns(rc6, GEN6_GT_GFX_RC6p);
	if (HAS_RC6pp(rc6_to_i915(rc6)))
		result += intel_rc6_residency_ns(rc6, GEN6_GT_GFX_RC6pp);

	intel_uncore_forcewake_flush(rc6_to_uncore(rc6), FORCEWAKE_ALL);

	return result;
}

static void enable_render_c6_counter(struct intel_gt *gt, int enable)
{
	intel_uncore_write(gt->uncore, RENDER_C6_RSDNCY_CNTR_MSB,
			   enable ? RENDER_C6_RSDNCY_CNTR_ENABLE :
			   ~RENDER_C6_RSDNCY_CNTR_ENABLE);
}

static void enable_media_c6_counter(struct intel_gt *gt, int enable)
{
	intel_uncore_write(gt->uncore, MEDIA_C6_RSDNCY_CNTR_MSB,
			   enable ? MEDIA_C6_RSDNCY_CNTR_ENABLE :
			   ~MEDIA_C6_RSDNCY_CNTR_ENABLE);
}

static bool render_gated(struct intel_gt *gt)
{
	u32 cpg_status = intel_uncore_read(gt->uncore, GEN9_PWRGT_DOMAIN_STATUS);

	return (!(cpg_status & GEN9_PWRGT_RENDER_STATUS_MASK));
}

static bool media_gated(struct intel_gt *gt)
{
	u32 cpg_status = intel_uncore_read(gt->uncore, GEN9_PWRGT_DOMAIN_STATUS);

	return (!(cpg_status & GEN9_PWRGT_MEDIA_STATUS_MASK));
}

static u64 render_c6_residency(struct intel_gt *gt)
{
	u32 lsb, msb;

	lsb = intel_uncore_read(gt->uncore, RENDER_C6_RSDNCY_CNTR_LSB);
	msb = intel_uncore_read(gt->uncore, RENDER_C6_RSDNCY_CNTR_MSB) & 0x0000FFFF;

	return lsb | ((u64)msb << 32);
}

static u64 media_c6_residency(struct intel_gt *gt)
{
	u32 lsb, msb;

	lsb = intel_uncore_read(gt->uncore, MEDIA_C6_RSDNCY_CNTR_LSB);
	msb = intel_uncore_read(gt->uncore, MEDIA_C6_RSDNCY_CNTR_MSB) & 0x0000FFFF;

	return lsb | ((u64)msb << 32);
}

/*
 * Check if Render can show residency in msec time
 */
static bool check_render_c6(struct intel_gt *gt, int msec)
{
	u64 res[2];

	enable_render_c6_counter(gt, true);

	res[0] = render_c6_residency(gt);
	msleep(msec);
	res[1] = render_c6_residency(gt);

	enable_render_c6_counter(gt, false);

	return (res[1] != res[0]);
}

/*
 * Check if Media can show residency in msec time
 */
static bool check_media_c6(struct intel_gt *gt, int msec)
{
	u64 res[2];

	enable_media_c6_counter(gt, true);

	res[0] = media_c6_residency(gt);
	msleep(msec);
	res[1] = media_c6_residency(gt);

	enable_media_c6_counter(gt, false);

	return (res[1] != res[0]);
}

static void get_media_fwake(struct intel_gt *gt)
{
	intel_engine_mask_t emask;
	int i;

	if (GRAPHICS_VER(gt->i915) < 11) {
		intel_uncore_forcewake_get(gt->uncore, FORCEWAKE_MEDIA);
	} else {
		/* For Gen11+, need to separately wake VD/VEBoxes */
		emask = gt->info.engine_mask;
		for (i = 0; i < I915_MAX_VCS; i++) {
			if (!__HAS_ENGINE(emask, _VCS(i)))
				continue;

			intel_uncore_forcewake_get(gt->uncore, FORCEWAKE_MEDIA_VDBOX0 + i);
		}

		/* Now VEBoxes */
		for (i = 0; i < I915_MAX_VECS; i++) {
			if (!__HAS_ENGINE(emask, _VECS(i)))
				continue;
			intel_uncore_forcewake_get(gt->uncore, FORCEWAKE_MEDIA_VEBOX0 + i);
		}
	}
}

static void put_media_fwake(struct intel_gt *gt)
{
	intel_engine_mask_t emask;
	int i;

	if (GRAPHICS_VER(gt->i915) < 11) {
		intel_uncore_forcewake_put(gt->uncore, FORCEWAKE_MEDIA);
	} else {
		/* For Gen11+, need to separately wake VD/VEBoxes */
		emask = gt->info.engine_mask;
		for (i = 0; i < I915_MAX_VCS; i++) {
			if (!__HAS_ENGINE(emask, _VCS(i)))
				continue;

			intel_uncore_forcewake_put(gt->uncore, FORCEWAKE_MEDIA_VDBOX0 + i);
		}

		/* Now VEBoxes */
		for (i = 0; i < I915_MAX_VECS; i++) {
			if (!__HAS_ENGINE(emask, _VECS(i)))
				continue;
			intel_uncore_forcewake_put(gt->uncore, FORCEWAKE_MEDIA_VEBOX0 + i);
		}
	}
}

int live_render_pg(void *arg)
{
	struct intel_gt *gt = arg;
	intel_wakeref_t wakeref;
	u32 cpg_enable;
	int err = 0;

	if (IS_VALLEYVIEW(gt->i915) || IS_CHERRYVIEW(gt->i915) ||
	    (GRAPHICS_VER(gt->i915) < 6))
		return 0;

	if (gt->type == GT_MEDIA)
		return 0;

	/*
	 * Wa_16015496043, Wa_16015476723 requires to hold forcewake (no rc6)
	 * across all selftests, preventing this test from functioning.
	 */
	if (pvc_needs_rc6_wa(gt->i915))
		return 0;

	wakeref = intel_runtime_pm_get(gt->uncore->rpm);

	/* skip if render c6 is disabled */
	cpg_enable = intel_uncore_read(gt->uncore, GEN9_PG_ENABLE);
	if (!(cpg_enable & GEN9_RENDER_PG_ENABLE))
		goto out_unlock;

	/* Check if we are in Render C6 first */
	if (!render_gated(gt)) {
		drm_err(&gt->i915->drm, "Render is not in C6");
		err = -EINVAL;
		goto out_unlock;
	}

	/* Forcewake GT and media, Render should still be asleep */
	intel_uncore_forcewake_get(gt->uncore, FORCEWAKE_GT);
	get_media_fwake(gt);
	if (!render_gated(gt)) {
		drm_err(&gt->i915->drm, "Render is not in C6 after GT/Media fwake");
		err = -EINVAL;
		goto out_fwake;
	}

	/* Check if render residency counter is incrementing */
	if (!check_render_c6(gt, 100)) {
		drm_err(&gt->i915->drm, "Render residency not seen");
		err = -EINVAL;
		goto out_fwake;
	}

	/* Lastly, get render fwake and ensure it is ungated */
	intel_uncore_forcewake_get(gt->uncore, FORCEWAKE_RENDER);
	if (render_gated(gt)) {
		drm_err(&gt->i915->drm, "Render fwake did not ungate Render");
		err = -EINVAL;
	}
	intel_uncore_forcewake_put(gt->uncore, FORCEWAKE_RENDER);

out_fwake:
	intel_uncore_forcewake_put(gt->uncore, FORCEWAKE_GT);
	put_media_fwake(gt);
out_unlock:
	intel_runtime_pm_put(gt->uncore->rpm, wakeref);
	return err;
}

/* Forcewake every media sub-well and validate that it wakes up media domain */
static int check_media_ungate(struct intel_gt *gt)
{
	intel_engine_mask_t emask;
	int ret = 0;
	int i;

	if (GRAPHICS_VER(gt->i915) < 11) {
		/* Get media fwake and ensure it is ungated */
		intel_uncore_forcewake_get(gt->uncore, FORCEWAKE_MEDIA);
		if (media_gated(gt)) {
			drm_err(&gt->i915->drm, "Media fwake did not ungate media");
			ret = -EINVAL;
		}
		intel_uncore_forcewake_put(gt->uncore, FORCEWAKE_MEDIA);

		return ret;
	}

	/* For Gen11+, need to separately wake VD/VEBoxes */
	emask = gt->info.engine_mask;
	for (i = 0; i < I915_MAX_VCS; i++) {
		if (!__HAS_ENGINE(emask, _VCS(i)))
			continue;

		intel_uncore_forcewake_get(gt->uncore, FORCEWAKE_MEDIA_VDBOX0 + i);
		if (media_gated(gt)) {
			drm_err(&gt->i915->drm, "Media(vdbox %d) fwake did not wake Media", i);
			ret = -EINVAL;
		}
		intel_uncore_forcewake_put(gt->uncore, FORCEWAKE_MEDIA_VDBOX0 + i);
	}

	/* Now VEBoxes */
	for (i = 0; i < I915_MAX_VECS; i++) {
		if (!__HAS_ENGINE(emask, _VECS(i)))
			continue;

		intel_uncore_forcewake_get(gt->uncore, FORCEWAKE_MEDIA_VEBOX0 + i);
		if (media_gated(gt)) {
			drm_err(&gt->i915->drm, "Media(vebox %d) fwake did not wake Media", i);
			ret = -EINVAL;
		}
		intel_uncore_forcewake_put(gt->uncore, FORCEWAKE_MEDIA_VEBOX0 + i);
	}

	return ret;
}

int live_media_pg(void *arg)
{
	struct intel_gt *gt = arg;
	intel_wakeref_t wakeref;
	u32 cpg_enable;
	int err = 0;

	if (IS_VALLEYVIEW(gt->i915) || IS_CHERRYVIEW(gt->i915) ||
	    (GRAPHICS_VER(gt->i915) < 6))
		return 0;

	if ((MEDIA_VER(gt->i915) >= 13) && (gt->type == GT_PRIMARY))
		return 0;

	/*
	 * Wa_16015496043, Wa_16015476723 requires to hold forcewake (no rc6)
	 * across all selftests, preventing this test from functioning.
	 */
	if (pvc_needs_rc6_wa(gt->i915))
		return 0;

	wakeref = intel_runtime_pm_get(gt->uncore->rpm);

	/* skip if media c6 is disabled */
	cpg_enable = intel_uncore_read(gt->uncore, GEN9_PG_ENABLE);
	if (!(cpg_enable & GEN9_MEDIA_PG_ENABLE))
		goto out_unlock;

	/* Check if we are in Media C6 first */
	if (!media_gated(gt)) {
		drm_err(&gt->i915->drm, "Media is not in C6");
		err = -EINVAL;
		goto out_unlock;
	}

	/* Forcewake GT and Render (if present), Media should still be asleep */
	if (MEDIA_VER(gt->i915) >= 13)
		intel_uncore_forcewake_get(gt->uncore, FORCEWAKE_GT);
	else
		intel_uncore_forcewake_get(gt->uncore, FORCEWAKE_GT | FORCEWAKE_RENDER);

	if (!media_gated(gt)) {
		drm_err(&gt->i915->drm, "Media is not in C6 after GT/Render fwake");
		err = -EINVAL;
		goto out_fwake;
	}

	/* Check if Media residency counter is incrementing */
	if (!check_media_c6(gt, 100)) {
		drm_err(&gt->i915->drm, "Media residency not seen");
		err = -EINVAL;
		goto out_fwake;
	}

	err = check_media_ungate(gt);

out_fwake:
	if (MEDIA_VER(gt->i915) >= 13)
		intel_uncore_forcewake_put(gt->uncore, FORCEWAKE_GT);
	else
		intel_uncore_forcewake_put(gt->uncore, FORCEWAKE_GT | FORCEWAKE_RENDER);
out_unlock:
	intel_runtime_pm_put(gt->uncore->rpm, wakeref);
	return err;
}

int live_rc6_manual(void *arg)
{
	struct intel_gt *gt = arg;
	struct intel_rc6 *rc6 = &gt->rc6;
	u64 rc0_power, rc6_power;
	intel_wakeref_t wakeref;
	bool has_power;
	ktime_t dt;
	u64 res[2];
	int err = 0;

	/*
	 * Our claim is that we can "encourage" the GPU to enter rc6 at will.
	 * Let's try it!
	 */

	if (!rc6->enabled)
		return 0;

	/* bsw/byt use a PCU and decouple RC6 from our manual control */
	if (IS_VALLEYVIEW(gt->i915) || IS_CHERRYVIEW(gt->i915))
		return 0;
	/*
	 * Wa_16015496043, Wa_16015476723 requires to hold forcewake (no rc6)
	 * across all selftests, preventing this test from functioning.
	 */
	if (pvc_needs_rc6_wa(gt->i915))
		return 0;

	has_power = librapl_supported(gt->i915);
	wakeref = intel_runtime_pm_get(gt->uncore->rpm);

	/* Force RC6 off for starters */
	__intel_rc6_disable(rc6);
	intel_uncore_forcewake_flush(rc6_to_uncore(rc6), FORCEWAKE_ALL);
	msleep(1); /* wakeup is not immediate, takes about 100us on icl */

	res[0] = rc6_residency(rc6);

	dt = ktime_get();
	rc0_power = librapl_energy_uJ(gt->i915);
	msleep(100);
	rc0_power = librapl_energy_uJ(gt->i915) - rc0_power;
	dt = ktime_sub(ktime_get(), dt);
	res[1] = rc6_residency(rc6);
	if ((res[1] - res[0]) >> 10) {
		pr_err("RC6 residency increased by %lldus while disabled for 100ms!\n",
		       (res[1] - res[0]) >> 10);
		err = -EINVAL;
		goto out_unlock;
	}

	if (has_power) {
		rc0_power = div64_u64(NSEC_PER_SEC * rc0_power,
				      ktime_to_ns(dt));
		if (!rc0_power) {
			pr_err("No power measured while in RC0\n");
			err = -EINVAL;
			goto out_unlock;
		}
	}

	/* Manually enter RC6 */
	intel_rc6_park(rc6);

	res[0] = rc6_residency(rc6);
	if (wait_for((rc6_residency(rc6) > res[0]), 10)) {
		pr_err("Did not enter RC6! RC6_STATE=%08x, RC6_CONTROL=%08x, residency=%lld\n",
		       intel_uncore_read_fw(gt->uncore, GEN6_RC_STATE),
		       intel_uncore_read_fw(gt->uncore, GEN6_RC_CONTROL),
		       res[0]);
		err = -EINVAL;
		goto out_unpark;
	}

	res[0] = rc6_residency(rc6);
	dt = ktime_get();
	rc6_power = librapl_energy_uJ(gt->i915);
	msleep(100);
	rc6_power = librapl_energy_uJ(gt->i915) - rc6_power;
	dt = ktime_sub(ktime_get(), dt);
	res[1] = rc6_residency(rc6);
	if (res[1] == res[0]) {
		pr_err("Did not enter RC6! RC6_STATE=%08x, RC6_CONTROL=%08x, residency=%lld\n",
		       intel_uncore_read_fw(gt->uncore, GEN6_RC_STATE),
		       intel_uncore_read_fw(gt->uncore, GEN6_RC_CONTROL),
		       res[0]);
		err = -EINVAL;
	}

	if (has_power) {
		rc6_power = div64_u64(NSEC_PER_SEC * rc6_power,
				      ktime_to_ns(dt));
		pr_info("GPU consumed %llduW in RC0 and %llduW in RC6\n",
			rc0_power, rc6_power);
		if (2 * rc6_power > rc0_power) {
			pr_info("GPU leaked energy while in RC6!\n");
			err = -EINTR;
		}
	}

	/* Restore what should have been the original state! */

out_unpark:
	intel_rc6_unpark(rc6);

out_unlock:
	intel_runtime_pm_put(gt->uncore->rpm, wakeref);
	return err;
}

static const u32 *__live_rc6_ctx(struct intel_context *ce)
{
	struct i915_request *rq;
	const u32 *result;
	u32 cmd;
	u32 *cs;

	rq = intel_context_create_request(ce);
	if (IS_ERR(rq))
		return ERR_CAST(rq);

	cs = intel_ring_begin(rq, 4);
	if (IS_ERR(cs)) {
		i915_request_add(rq);
		return cs;
	}

	cmd = MI_STORE_REGISTER_MEM | MI_USE_GGTT;
	if (GRAPHICS_VER(rq->engine->i915) >= 8)
		cmd++;

	*cs++ = cmd;
	*cs++ = i915_mmio_reg_offset(GEN8_RC6_CTX_INFO);
	*cs++ = ce->timeline->hwsp_offset + 8;
	*cs++ = 0;
	intel_ring_advance(rq, cs);

	result = rq->hwsp_seqno + 2;
	i915_request_add(rq);

	return result;
}

static struct intel_engine_cs **
randomised_engines(struct intel_gt *gt,
		   struct rnd_state *prng,
		   unsigned int *count)
{
	struct intel_engine_cs *engine, **engines;
	enum intel_engine_id id;
	int n;

	n = 0;
	for_each_engine(engine, gt, id)
		n++;
	if (!n)
		return NULL;

	engines = kmalloc_array(n, sizeof(*engines), GFP_KERNEL);
	if (!engines)
		return NULL;

	n = 0;
	for_each_engine(engine, gt, id)
		engines[n++] = engine;

	i915_prandom_shuffle(engines, sizeof(*engines), n, prng);

	*count = n;
	return engines;
}

int live_rc6_ctx_wa(void *arg)
{
	struct intel_gt *gt = arg;
	struct intel_engine_cs **engines;
	unsigned int n, count;
	I915_RND_STATE(prng);
	int err = 0;

	/* A read of CTX_INFO upsets rc6. Poke the bear! */
	if (GRAPHICS_VER(gt->i915) < 8)
		return 0;

	engines = randomised_engines(gt, &prng, &count);
	if (!engines)
		return 0;

	for (n = 0; n < count; n++) {
		struct intel_engine_cs *engine = engines[n];
		int pass;

		for (pass = 0; pass < 2; pass++) {
			struct intel_context *ce;
			unsigned int resets =
				i915_reset_engine_count(engine);
			const u32 *res;

			/* Use a sacrifical context */
			ce = intel_context_create(engine);
			if (IS_ERR(ce)) {
				err = PTR_ERR(ce);
				goto out;
			}

			intel_engine_pm_get(engine);
			res = __live_rc6_ctx(ce);
			intel_engine_pm_put(engine);
			intel_context_put(ce);
			if (IS_ERR(res)) {
				err = PTR_ERR(res);
				goto out;
			}

			if (intel_gt_wait_for_idle(gt, HZ / 5) == -ETIME) {
				intel_gt_set_wedged(gt);
				err = -ETIME;
				goto out;
			}

			intel_gt_pm_wait_for_idle(gt);
			pr_debug("%s: CTX_INFO=%0x\n",
				 engine->name, READ_ONCE(*res));

			if (resets !=
			    i915_reset_engine_count(engine)) {
				pr_err("%s: GPU reset required\n",
				       engine->name);
				add_taint_for_CI(gt->i915, TAINT_WARN);
				err = -EIO;
				goto out;
			}
		}
	}

out:
	kfree(engines);
	return err;
}
