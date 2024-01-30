// SPDX-License-Identifier: MIT
/*
 * Copyright © 2021 Intel Corporation
 */

#include "gt/intel_gt.h"
#include "gt/intel_ring.h"
#include "gt/intel_gpu_commands.h"
#include "selftests/igt_flush_test.h"

#include "i915_selftest.h"

static int emit_srm(struct i915_request *rq, i915_reg_t reg, u32 *out)
{
	u32 *cs;

	cs = intel_ring_begin(rq, 4);
	if (IS_ERR(cs))
		return PTR_ERR(cs);

	*cs++ = MI_STORE_REGISTER_MEM_GEN8 | MI_USE_GGTT;
	*cs++ = i915_mmio_reg_offset(reg);
	*cs++ = i915_ggtt_offset(rq->engine->status_page.vma) +
		offset_in_page(out);
	*cs++ = 0;

	intel_ring_advance(rq, cs);

	return 0;
}

/*
 * Note that accesses to MCR registers originating from engine instructions
 * are unaffected by CPU steering controls.  We never touch the engine steering,
 * so writes are always multicast and reads are always from some non-terminated
 * instance.
 */
static int emit_srm_mcr(struct i915_request *rq, i915_mcr_reg_t mreg, u32 *out)
{
	i915_reg_t r = _MMIO(mreg.reg);

	return emit_srm(rq, r, out);
}

static u32 mmio_read(struct intel_engine_cs *engine,
		     i915_mcr_reg_t reg)
{
	intel_wakeref_t wakeref;
	u32 val = 0;

	with_intel_runtime_pm(engine->uncore->rpm, wakeref)
		val = intel_gt_mcr_read_any(engine->gt, reg);

	return val;
}

static int test_debug_enable(struct drm_i915_private *i915,
			     const bool enable)
{

	struct intel_engine_cs *engine;
	int err = 0, ret = 0;
	u32 td_ctl, ctl_mask;
	u32 row_chicken, row_chicken_mask;

	ctl_mask = TD_CTL_BREAKPOINT_ENABLE |
		TD_CTL_FORCE_THREAD_BREAKPOINT_ENABLE |
		TD_CTL_FEH_AND_FEE_ENABLE;

	if (GRAPHICS_VER_FULL(i915) >= IP_VER(12, 50))
		ctl_mask |= TD_CTL_GLOBAL_DEBUG_ENABLE;

	ctl_mask = enable ? ctl_mask : 0;
	row_chicken_mask = 0;

	if (GRAPHICS_VER_FULL(i915) >= IP_VER(12, 50))
		row_chicken_mask = enable ? REG_BIT(5) : 0;

	/*
	 * For exceptions and attention notification to work, we have
	 * to ensure various bits are configured globally and in each
	 * context. While these should be checked on application by
	 * the workaround handlers, we want an explicit checklist
	 * of known eudbg workarounds.
	 */
	for_each_uabi_engine(engine, i915) {
		struct intel_context *ce;
		struct i915_request *rq;
		u32 *result;

		if (engine->class != RENDER_CLASS &&
		    engine->class != COMPUTE_CLASS)
			continue;

		result = memset32(engine->status_page.addr + 1000,
				  0xabadbadb, 2);

		ce = intel_context_create(engine);
		if (IS_ERR(ce))
			return PTR_ERR(ce);

		rq = intel_context_create_request(ce);
		intel_context_put(ce);
		if (IS_ERR(rq))
			return PTR_ERR(rq);

		err = emit_srm_mcr(rq, TD_CTL, result);
		err |= emit_srm_mcr(rq, GEN8_ROW_CHICKEN, result + 1);

		i915_request_get(rq);
		i915_request_add(rq);
		if (err == 0 && i915_request_wait(rq, 0, HZ) < 0)
			err = -ETIME;
		i915_request_put(rq);
		if (err)
			return err;

		td_ctl = READ_ONCE(*result);
		row_chicken = READ_ONCE(*(result + 1));

		pr_info("%s %s TD_CTL (srm): %08x\n", engine->name,
			enable ? "enable" : "disable", td_ctl);

		pr_info("%s %s GEN8_ROW_CHICKEN_CTL (srm): %08x\n", engine->name,
			enable ? "enable" : "disable", row_chicken);

		if (td_ctl != ctl_mask) {
			pr_err("%s TD_CTL (srm) %s error 0x%08x vs 0x%08x (expected)\n",
			       engine->name, enable ? "enable" : "disable",
			       td_ctl, ctl_mask);
			ret |= -EINVAL;
		}

		if ((row_chicken & REG_BIT(5)) != row_chicken_mask) {
			pr_err("%s GEN8_ROW_CHICKEN (srm) %s error 0x%08x vs 0x%08x (expected)\n",
			       engine->name, enable ? "enable" : "disable",
			       row_chicken, row_chicken_mask);
			ret |= -EINVAL;
		}

	}

	/* Settle to provoke power state transition */
	err = igt_flush_test(i915);
	if (err)
		return err;

	for_each_uabi_engine(engine, i915) {
		if (engine->class != RENDER_CLASS &&
		    engine->class != COMPUTE_CLASS)
			continue;

		td_ctl = mmio_read(engine, TD_CTL);
		row_chicken = mmio_read(engine, GEN8_ROW_CHICKEN);

		pr_info("%s %s TD_CTL (mmio): %08x\n", engine->name,
			enable ? "enable" : "disable", td_ctl);

		pr_info("%s %s GEN8_ROW_CHICKEN_CTL (mmio): %08x\n", engine->name,
			enable ? "enable" : "disable", row_chicken);

		if (td_ctl != ctl_mask) {
			pr_err("%s TD_CTL (mmio) %s error 0x%08x vs 0x%08x (expected)\n",
			       engine->name, enable ? "enable" : "disable",
			       td_ctl, ctl_mask);
			ret |= -EINVAL;
		}

		if ((row_chicken & REG_BIT(5)) != row_chicken_mask) {
			pr_err("%s GEN8_ROW_CHICKEN (mmio) %s error 0x%08x vs 0x%08x (expected)\n",
			       engine->name, enable ? "enable" : "disable",
			       row_chicken, row_chicken_mask);
			ret |= -EINVAL;
		}
	}

	return ret;
}

static int debug_hw_enable(void *arg)
{
	struct drm_i915_private *i915 = arg;
	const bool preset = i915->debuggers.enable_eu_debug;
	int ret;

	if (GRAPHICS_VER(i915) < 12)
		return 0;

	pr_info("Validating initial i915.enable_eu_debug=%d setting\n", preset);
	ret = test_debug_enable(i915, preset);
	if (ret)
		return ret;

	pr_info("Toggling enable_eu_debug=%d\n", !preset);
	ret = i915_debugger_enable(i915, !preset);
	if (ret)
		return ret;

	ret = test_debug_enable(i915, !preset);
	if (ret)
		return ret;

	pr_info("Restoring enable_eu_debug=%d default\n", preset);
	ret = i915_debugger_enable(i915, preset);
	if (ret)
		return ret;

	return test_debug_enable(i915, preset);
}

int i915_debugger_live_selftests(struct drm_i915_private *i915)
{
	static const struct i915_subtest tests[] = {
		SUBTEST(debug_hw_enable),
	};

	pr_info("sizeof(i915_debugger)=%zd\n", sizeof(struct i915_debugger));

	if (intel_gt_is_wedged(to_gt(i915)))
		return 0;

	return i915_subtests(tests, i915);
}
