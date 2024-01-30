// SPDX-License-Identifier: MIT
/*
 * Copyright © 2014-2019 Intel Corporation
 */

#include "gem/i915_gem_lmem.h"
#include "gem/i915_gem_region.h"
#include "gt/intel_gt.h"
#include "gt/intel_gt_irq.h"
#include "gt/intel_gt_pm_irq.h"
#include "gt/intel_gt_regs.h"
#include "intel_guc.h"
#include "intel_guc_ads.h"
#include "intel_guc_capture.h"
#include "intel_guc_print.h"
#include "intel_guc_slpc.h"
#include "intel_guc_submission.h"
#include "i915_drv.h"
#include "i915_irq.h"

#ifdef CONFIG_DRM_I915_DEBUG_GUC
#define GUC_DEBUG(_guc, _fmt, ...) \
	drm_dbg(&guc_to_gt(_guc)->i915->drm, "GUC%u: " _fmt, guc_to_gt(_guc)->info.id, ##__VA_ARGS__)
#else
#define GUC_DEBUG(_guc, _fmt, ...) typecheck(struct intel_guc *, _guc)
#endif

static const struct intel_guc_ops guc_ops_default;
static const struct intel_guc_ops guc_ops_vf;

static struct i915_vma *guc_vma_from_obj(struct intel_guc *guc,
					 struct drm_i915_gem_object *obj,
					 u32 bias);

/**
 * DOC: GuC
 *
 * The GuC is a microcontroller inside the GT HW, introduced in gen9. The GuC is
 * designed to offload some of the functionality usually performed by the host
 * driver; currently the main operations it can take care of are:
 *
 * - Authentication of the HuC, which is required to fully enable HuC usage.
 * - Low latency graphics context scheduling (a.k.a. GuC submission).
 * - GT Power management.
 *
 * The enable_guc module parameter can be used to select which of those
 * operations to enable within GuC. Note that not all the operations are
 * supported on all gen9+ platforms.
 *
 * Enabling the GuC is not mandatory and therefore the firmware is only loaded
 * if at least one of the operations is selected. However, not loading the GuC
 * might result in the loss of some features that do require the GuC (currently
 * just the HuC, but more are expected to land in the future).
 */

void intel_guc_notify(struct intel_guc *guc)
{
	struct intel_gt *gt = guc_to_gt(guc);

	/*
	 * On Gen11+, the value written to the register is passes as a payload
	 * to the FW. However, the FW currently treats all values the same way
	 * (H2G interrupt), so we can just write the value that the HW expects
	 * on older gens.
	 */
	raw_reg_write(gt->uncore->regs, guc->notify_reg, GUC_SEND_TRIGGER);
}

static inline i915_reg_t guc_send_reg(struct intel_guc *guc, u32 i)
{
	GEM_BUG_ON(!guc->send_regs.base);
	GEM_BUG_ON(!guc->send_regs.count);
	GEM_BUG_ON(i >= guc->send_regs.count);

	return _MMIO(guc->send_regs.base + 4 * i);
}

void intel_guc_init_send_regs(struct intel_guc *guc)
{
	struct intel_gt *gt = guc_to_gt(guc);
	enum forcewake_domains fw_domains = 0;
	unsigned int i;

	GEM_BUG_ON(!guc->send_regs.base);
	GEM_BUG_ON(!guc->send_regs.count);

	for (i = 0; i < guc->send_regs.count; i++) {
		fw_domains |= intel_uncore_forcewake_for_reg(gt->uncore,
					guc_send_reg(guc, i),
					FW_REG_READ | FW_REG_WRITE);
	}
	guc->send_regs.fw_domains = fw_domains;

	/* XXX: move to init_early when safe to call IS_SRIOV_VF */
	if (IS_SRIOV_VF(guc_to_gt(guc)->i915))
		guc->ops = &guc_ops_vf;
}

static void gen9_reset_guc_interrupts(struct intel_guc *guc)
{
	struct intel_gt *gt = guc_to_gt(guc);

	assert_rpm_wakelock_held(&gt->i915->runtime_pm);

	spin_lock_irq(gt->irq_lock);
	gen6_gt_pm_reset_iir(gt, gt->pm_guc_events);
	spin_unlock_irq(gt->irq_lock);
}

static void gen9_enable_guc_interrupts(struct intel_guc *guc)
{
	struct intel_gt *gt = guc_to_gt(guc);

	assert_rpm_wakelock_held(&gt->i915->runtime_pm);
	guc->interrupts.enabled = true;

	spin_lock_irq(gt->irq_lock);
	guc_WARN_ON_ONCE(guc, intel_uncore_read(gt->uncore, GEN8_GT_IIR(2)) &
			 gt->pm_guc_events);
	gen6_gt_pm_enable_irq(gt, gt->pm_guc_events);
	spin_unlock_irq(gt->irq_lock);
}

static void gen9_disable_guc_interrupts(struct intel_guc *guc)
{
	struct intel_gt *gt = guc_to_gt(guc);

	assert_rpm_wakelock_held(&gt->i915->runtime_pm);
	guc->interrupts.enabled = false;

	spin_lock_irq(gt->irq_lock);

	gen6_gt_pm_disable_irq(gt, gt->pm_guc_events);

	spin_unlock_irq(gt->irq_lock);
	intel_synchronize_irq(gt->i915);

	gen9_reset_guc_interrupts(guc);
}

static bool __gen11_reset_guc_interrupts(struct intel_gt *gt)
{
	u32 irq = gt->type == GT_MEDIA ? GEN12_GUCM : GEN11_GUC;

	lockdep_assert_held(gt->irq_lock);
	return gen11_gt_reset_one_iir(gt, 0, irq);
}

static void gen11_reset_guc_interrupts(struct intel_guc *guc)
{
	struct intel_gt *gt = guc_to_gt(guc);

	spin_lock_irq(gt->irq_lock);
	__gen11_reset_guc_interrupts(gt);
	spin_unlock_irq(gt->irq_lock);
}

/* Wa:16014207253 */
static void fake_int_timer_start(struct intel_gt *gt)
{
	if (atomic_read(&gt->fake_int.boost))
		gt->fake_int.delay = gt->fake_int.delay_fast;
	else
		gt->fake_int.delay = gt->fake_int.delay_slow;

	hrtimer_start(&gt->fake_int.timer, ns_to_ktime(gt->fake_int.delay), HRTIMER_MODE_REL);
}

static void fake_int_timer_stop(struct intel_gt *gt)
{
	gt->fake_int.delay = 0;
	hrtimer_cancel(&gt->fake_int.timer);
}

static void gen11_enable_fake_interrupts(struct intel_guc *guc)
{
	struct intel_gt *gt = guc_to_gt(guc);

	guc->interrupts.enabled = true;

	spin_lock_irq(gt->irq_lock);
	WARN_ON_ONCE(gen11_gt_reset_one_iir(gt, 0, GEN11_GUC));

	/* FIXME: Connect timer to GT PM */
	fake_int_timer_start(gt);
	spin_unlock_irq(gt->irq_lock);
}

static void gen11_disable_fake_interrupts(struct intel_guc *guc)
{
	struct intel_gt *gt = guc_to_gt(guc);

	guc->interrupts.enabled = false;

	spin_lock_irq(gt->irq_lock);

	fake_int_timer_stop(gt);

	spin_unlock_irq(gt->irq_lock);
	intel_synchronize_irq(gt->i915);

	gen11_reset_guc_interrupts(guc);
}

void intel_guc_init_fake_interrupts(struct intel_guc *guc)
{
	guc->interrupts.enable = gen11_enable_fake_interrupts;
	guc->interrupts.disable = gen11_disable_fake_interrupts;
}

static void gen11_enable_guc_interrupts(struct intel_guc *guc)
{
	struct intel_gt *gt = guc_to_gt(guc);

	guc->interrupts.enabled = true;

	spin_lock_irq(gt->irq_lock);
	__gen11_reset_guc_interrupts(gt);
	spin_unlock_irq(gt->irq_lock);
}

static void gen11_disable_guc_interrupts(struct intel_guc *guc)
{
	struct intel_gt *gt = guc_to_gt(guc);

	guc->interrupts.enabled = false;
	intel_synchronize_irq(gt->i915);

	gen11_reset_guc_interrupts(guc);
}

static void guc_dead_worker_func(struct work_struct *w)
{
	struct intel_guc *guc = container_of(w, struct intel_guc, dead_guc_worker);

	intel_gt_handle_error(guc_to_gt(guc), ALL_ENGINES, I915_ERROR_CAPTURE, "dead GuC");
}

void intel_guc_init_early(struct intel_guc *guc)
{
	struct intel_gt *gt = guc_to_gt(guc);
	struct drm_i915_private *i915 = gt->i915;

	intel_uc_fw_init_early(&guc->fw, INTEL_UC_FW_TYPE_GUC);
	intel_guc_ct_init_early(&guc->ct);
	intel_guc_log_init_early(&guc->log);
	intel_guc_submission_init_early(guc);
	intel_guc_slpc_init_early(&guc->slpc);
	intel_guc_rc_init_early(guc);

	INIT_WORK(&guc->dead_guc_worker, guc_dead_worker_func);

	mutex_init(&guc->send_mutex);
	spin_lock_init(&guc->irq_lock);
	if (GRAPHICS_VER(i915) >= 11) {
		guc->interrupts.reset = gen11_reset_guc_interrupts;
		guc->interrupts.enable = gen11_enable_guc_interrupts;
		guc->interrupts.disable = gen11_disable_guc_interrupts;
		if (gt->type == GT_MEDIA) {
			guc->notify_reg = MEDIA_GUC_HOST_INTERRUPT;
			guc->send_regs.base = i915_mmio_reg_offset(MEDIA_SOFT_SCRATCH(0));
		} else {
			guc->notify_reg = GEN11_GUC_HOST_INTERRUPT;
			guc->send_regs.base = i915_mmio_reg_offset(GEN11_SOFT_SCRATCH(0));
		}

		guc->send_regs.count = GEN11_SOFT_SCRATCH_COUNT;
	} else {
		guc->notify_reg = GUC_SEND_INTERRUPT;
		guc->interrupts.reset = gen9_reset_guc_interrupts;
		guc->interrupts.enable = gen9_enable_guc_interrupts;
		guc->interrupts.disable = gen9_disable_guc_interrupts;
		guc->send_regs.base = i915_mmio_reg_offset(SOFT_SCRATCH(0));
		guc->send_regs.count = GUC_MAX_MMIO_MSG_LEN;
		BUILD_BUG_ON(GUC_MAX_MMIO_MSG_LEN > SOFT_SCRATCH_COUNT);
	}

	intel_guc_enable_msg(guc, INTEL_GUC_RECV_MSG_EXCEPTION |
				  INTEL_GUC_RECV_MSG_CRASH_DUMP_POSTED);

	guc->ops = &guc_ops_default;
}

void intel_guc_init_late(struct intel_guc *guc)
{
	intel_guc_ads_init_late(guc);
}

static u32 guc_ctl_debug_flags(struct intel_guc *guc)
{
	u32 level = intel_guc_log_get_level(&guc->log);
	u32 flags = 0;

	if (!GUC_LOG_LEVEL_IS_VERBOSE(level))
		flags |= GUC_LOG_DISABLED;
	else
		flags |= GUC_LOG_LEVEL_TO_VERBOSITY(level) <<
			 GUC_LOG_VERBOSITY_SHIFT;

	return flags;
}

static u32 guc_ctl_feature_flags(struct intel_guc *guc)
{
	u32 flags = 0;

	if (!intel_guc_submission_is_used(guc))
		flags |= GUC_CTL_DISABLE_SCHEDULER;

	if (intel_guc_slpc_is_used(guc))
		flags |= GUC_CTL_ENABLE_SLPC;

	/* FIXME: Should we enable this feature unconditionally? */
	if (HAS_UM_QUEUES(guc_to_gt(guc)->i915))
		flags |= GUC_CTL_ENABLE_RESET_ON_CAT;

	flags |= i915_modparams.guc_feature_flags;

	return flags;
}

static u32 guc_ctl_log_params_flags(struct intel_guc *guc)
{
	struct intel_guc_log *log = &guc->log;
	u32 offset, flags;

	GEM_BUG_ON(!log->sizes_initialised);

	offset = intel_guc_ggtt_offset(guc, log->vma) >> PAGE_SHIFT;

	flags = GUC_LOG_VALID |
		GUC_LOG_NOTIFY_ON_HALF_FULL |
		log->sizes[GUC_LOG_SECTIONS_DEBUG].flag |
		log->sizes[GUC_LOG_SECTIONS_CAPTURE].flag |
		(log->sizes[GUC_LOG_SECTIONS_CRASH].count << GUC_LOG_CRASH_SHIFT) |
		(log->sizes[GUC_LOG_SECTIONS_DEBUG].count << GUC_LOG_DEBUG_SHIFT) |
		(log->sizes[GUC_LOG_SECTIONS_CAPTURE].count << GUC_LOG_CAPTURE_SHIFT) |
		(offset << GUC_LOG_BUF_ADDR_SHIFT);

	return flags;
}

static u32 guc_ctl_ads_flags(struct intel_guc *guc)
{
	u32 ads = intel_guc_ggtt_offset(guc, guc->ads_vma) >> PAGE_SHIFT;
	u32 flags = ads << GUC_ADS_ADDR_SHIFT;

	return flags;
}

static u32 guc_ctl_wa_flags(struct intel_guc *guc)
{
	struct intel_gt *gt = guc_to_gt(guc);
	u32 flags = 0;

	/* Wa_22012773006:gen11,gen12 < XeHP */
	if (GRAPHICS_VER(gt->i915) >= 11 &&
	    GRAPHICS_VER_FULL(gt->i915) < IP_VER(12, 50))
		flags |= GUC_WA_POLLCS;

	/* Wa_16011759253:dg2_g10:a0 */
	/* Wa_22011383443:pvc - Also needed for PVC BD A0 */
	if (IS_DG2_GRAPHICS_STEP(gt->i915, G10, STEP_A0, STEP_B0) ||
	    (IS_PVC_BD_STEP(gt->i915, STEP_A0, STEP_B0)))
		flags |= GUC_WA_GAM_CREDITS;

	/* Wa_14014475959 */
	if (IS_MTL_GRAPHICS_STEP(gt->i915, M, STEP_A0, STEP_B0) ||
	    IS_DG2(gt->i915))
		flags |= GUC_WA_HOLD_CCS_SWITCHOUT;

	/*
	 * Wa_14012197797:dg2_g10:a0,dg2_g11:a0
	 * Wa_22011391025:dg2_g10,dg2_g11,dg2_g12
	 *
	 * The same WA bit is used for both and 22011391025 is applicable to
	 * all DG2.
	 */
	if (IS_DG2(gt->i915))
		flags |= GUC_WA_DUAL_QUEUE;

	/*
	 * Wa_22011802037: graphics version 11/12
	 * GUC_WA_PRE_PARSER causes media workload hang for PVC A0 and PCIe
	 * errors. Disable this for PVC A0 steppings.
	 */
	if ((IS_MTL_GRAPHICS_STEP(gt->i915, M, STEP_A0, STEP_B0) ||
	    (GRAPHICS_VER(gt->i915) >= 11 &&
	    GRAPHICS_VER_FULL(gt->i915) < IP_VER(12, 70))) &&
	    !IS_PVC_BD_STEP(gt->i915, STEP_A0, STEP_B0))
		flags |= GUC_WA_PRE_PARSER;

	/* Wa_16011777198:dg2 */
	if (IS_DG2_GRAPHICS_STEP(gt->i915, G10, STEP_A0, STEP_C0) ||
	    IS_DG2_GRAPHICS_STEP(gt->i915, G11, STEP_A0, STEP_B0))
		flags |= GUC_WA_RCS_RESET_BEFORE_RC6;

	/*
	 * Wa_22012727170:dg2_g10[a0-c0), dg2_g11[a0..), pvc[ctxt_a0..ctxt_b0)
	 * Wa_22012727685:dg2_g11[a0..)
	 *
	 * This WA is applicable to PVC CT A0, but causes media regressions.
	 * Drop the WA for PVC.
	 */
	if (IS_DG2_GRAPHICS_STEP(gt->i915, G10, STEP_A0, STEP_C0) ||
	    IS_DG2_GRAPHICS_STEP(gt->i915, G11, STEP_A0, STEP_FOREVER))
		flags |= GUC_WA_CONTEXT_ISOLATION;

	/* Wa_16015675438, Wa_18020744125 */
	if (!RCS_MASK(gt))
		flags |= GUC_WA_RCS_REGS_IN_CCS_REGS_LIST;

	/*
	 * Wa_13010778322
	 * FIXME: this WA seems to apply to other platforms as well, but the
	 * state in the database is still not confirmed.
	 */
	if (IS_DG2(gt->i915) || IS_PONTEVECCHIO(gt->i915))
		flags |= GUC_WA_ENABLE_TSC_CHECK_ON_RC6;

	/*
	 * Wa_1509372804: PVC, Apply WA in render force wake step by
	 * GUC FW before any work submission to CCS engines
	 */
	if (IS_PVC_CT_STEP(gt->i915, STEP_B0, STEP_C0) &&
	    gt->rc6.supported)
		flags |= GUC_WA_RENDER_RST_RC6_EXIT;

	return flags;
}

static u32 guc_ctl_devid(struct intel_guc *guc)
{
	struct drm_i915_private *i915 = guc_to_gt(guc)->i915;

	return (INTEL_DEVID(i915) << 16) | INTEL_REVID(i915);
}

/*
 * Initialise the GuC parameter block before starting the firmware
 * transfer. These parameters are read by the firmware on startup
 * and cannot be changed thereafter.
 */
static void guc_init_params(struct intel_guc *guc)
{
	u32 *params = guc->params;
	int i;

	BUILD_BUG_ON(sizeof(guc->params) != GUC_CTL_MAX_DWORDS * sizeof(u32));

	params[GUC_CTL_LOG_PARAMS] = guc_ctl_log_params_flags(guc);
	params[GUC_CTL_FEATURE] = guc_ctl_feature_flags(guc);
	params[GUC_CTL_DEBUG] = guc_ctl_debug_flags(guc);
	params[GUC_CTL_ADS] = guc_ctl_ads_flags(guc);
	params[GUC_CTL_WA] = guc_ctl_wa_flags(guc);
	params[GUC_CTL_DEVID] = guc_ctl_devid(guc);

	for (i = 0; i < GUC_CTL_MAX_DWORDS; i++)
		guc_dbg(guc, "param[%2d] = %#x\n", i, params[i]);
}

static int guc_action_register_g2g_buffer(struct intel_guc *guc, u32 type, u32 dst,
					  u32 desc_addr, u32 buff_addr, u32 size)
{
	u32 request[HOST2GUC_REGISTER_G2G_REQUEST_MSG_LEN] = {
		FIELD_PREP(GUC_HXG_MSG_0_ORIGIN, GUC_HXG_ORIGIN_HOST) |
		FIELD_PREP(GUC_HXG_MSG_0_TYPE, GUC_HXG_TYPE_REQUEST) |
		FIELD_PREP(GUC_HXG_REQUEST_MSG_0_ACTION, GUC_ACTION_HOST2GUC_REGISTER_G2G),
		FIELD_PREP(HOST2GUC_REGISTER_G2G_REQUEST_MSG_1_SIZE, size / SZ_4K - 1) |
		FIELD_PREP(HOST2GUC_REGISTER_G2G_REQUEST_MSG_1_TYPE, type) |
		FIELD_PREP(HOST2GUC_REGISTER_G2G_REQUEST_MSG_1_DEST, dst),
		FIELD_PREP(HOST2GUC_REGISTER_G2G_REQUEST_MSG_2_DESC_ADDR, desc_addr),
		FIELD_PREP(HOST2GUC_REGISTER_G2G_REQUEST_MSG_3_BUFF_ADDR, buff_addr),
	};

	GEM_BUG_ON(type != GUC_G2G_TYPE_IN && type != GUC_G2G_TYPE_OUT);
	GEM_BUG_ON(size % SZ_4K);

	return intel_guc_send(guc, request, ARRAY_SIZE(request));
}

static int guc_action_deregister_g2g_buffer(struct intel_guc *guc, u32 type, u32 dst)
{
	u32 request[HOST2GUC_DEREGISTER_G2G_REQUEST_MSG_LEN] = {
		FIELD_PREP(GUC_HXG_MSG_0_ORIGIN, GUC_HXG_ORIGIN_HOST) |
		FIELD_PREP(GUC_HXG_MSG_0_TYPE, GUC_HXG_TYPE_REQUEST) |
		FIELD_PREP(GUC_HXG_REQUEST_MSG_0_ACTION, GUC_ACTION_HOST2GUC_DEREGISTER_G2G),
		FIELD_PREP(HOST2GUC_DEREGISTER_G2G_REQUEST_MSG_1_TYPE, type) |
		FIELD_PREP(HOST2GUC_DEREGISTER_G2G_REQUEST_MSG_1_DEST, dst),
	};

	GEM_BUG_ON(type != GUC_G2G_TYPE_IN && type != GUC_G2G_TYPE_OUT);

	return intel_guc_send(guc, request, ARRAY_SIZE(request));
}

static unsigned int g2g_num(struct intel_guc *guc, u32 instance, u32 type)
{
	u32 local, remote;

	if (type == GUC_G2G_TYPE_IN) {
		local = instance;
		remote = guc_to_gt(guc)->info.id;
	} else {
		local = guc_to_gt(guc)->info.id;
		remote = instance;
	}
	GEM_BUG_ON(local == remote);

	if (remote > local)
		remote--;

	return remote * local;
}

#define G2G_BUFFER_SIZE	(SZ_4K)
#define G2G_DESC_SIZE (64)
#define G2G_DESC_AREA_SIZE (SZ_4K)
static int guc_g2g_register(struct intel_guc *guc, u32 instance, u32 type)
{
	unsigned int num = g2g_num(guc, instance, type);
	u32 base, desc, buf;

	base = intel_guc_ggtt_offset(guc, guc->g2g.vma);
	desc = base + num * G2G_DESC_SIZE;
	buf = base + G2G_DESC_AREA_SIZE + num * G2G_BUFFER_SIZE;

	return guc_action_register_g2g_buffer(guc, type, instance, desc, buf, G2G_BUFFER_SIZE);
}

static void guc_g2g_deregister(struct intel_guc *guc, u32 instance, u32 type)
{
	int err;

	err = guc_action_deregister_g2g_buffer(guc, type, instance);
	GEM_WARN_ON(err);
}

static u32 guc_g2g_size(struct intel_guc *guc)
{
	struct intel_gt *gt = guc_to_gt(guc);
	unsigned int tiles = gt->i915->remote_tiles + 1;

	return tiles * gt->i915->remote_tiles * G2G_BUFFER_SIZE + G2G_DESC_AREA_SIZE;
}

/*
 * Allocate the buffer used for guc-to-guc communication.
 * Root tile owns the allocation. Individual buffer size must be 4K aligned.
 *
 *   +--------+-----------------------------+------------------------------+
 *   | offset | contents                    | size                         |
 *   +========+=============================+==============================+
 *   | 0x0000 | Descriptors                 |  4K                          |
 *   |        |                             |                              |
 *   +--------+-----------------------------+------------------------------+
 *   | 0x1000 | Buffers                     | num_tiles*(num_tiles-1)*n*4K |
 *   |        |                             |                              |
 *   +--------+-----------------------------+------------------------------+
 */
static int guc_g2g_create(struct intel_guc *guc)
{
	struct intel_gt *gt = guc_to_gt(guc);
	int err;

	BUILD_BUG_ON(I915_MAX_GT * (I915_MAX_GT - 1) *
		     G2G_DESC_SIZE > G2G_DESC_AREA_SIZE);

	if (!gt->i915->remote_tiles)
		return 0;

	if (gt->info.id != 0) {
		struct intel_guc *root_guc = &to_root_gt(gt->i915)->uc.guc;
		struct i915_vma *vma;

		vma = guc_vma_from_obj(guc, root_guc->g2g.obj, 0);
		if (IS_ERR(vma))
			return PTR_ERR(vma);

		guc->g2g.vma = i915_vma_get(vma);
		__i915_gem_object_pin_pages(guc->g2g.vma->obj);
		return 0;
	}

	err = intel_guc_allocate_and_map_vma(guc, guc_g2g_size(guc),
					     &guc->g2g.vma, &guc->g2g.vaddr);
	if (unlikely(err))
		return err;

	guc->g2g.obj = guc->g2g.vma->obj;
	memset(guc->g2g.vaddr, 0, G2G_DESC_AREA_SIZE);

	return 0;
}

static void guc_g2g_destroy(struct intel_guc *guc)
{
	struct intel_gt *gt = guc_to_gt(guc);

	if (!gt->i915->remote_tiles)
		return;

	i915_vma_unpin_and_release(&guc->g2g.vma, I915_VMA_RELEASE_MAP);
}

int intel_guc_g2g_register(struct intel_guc *guc)
{
	struct intel_gt *remote_gt, *gt = guc_to_gt(guc);
	unsigned int i, j;
	int err;

	for_each_gt(remote_gt, gt->i915, i) {
		if (i == gt->info.id)
			continue;

		err = guc_g2g_register(guc, i, GUC_G2G_TYPE_IN);
		if (err)
			goto err_deregister;

		err = guc_g2g_register(guc, i, GUC_G2G_TYPE_OUT);
		if (err) {
			guc_g2g_deregister(guc, i, GUC_G2G_TYPE_IN);
			goto err_deregister;
		}
	}

	return 0;

err_deregister:
	for_each_gt(remote_gt, gt->i915, j) {
		if (j == gt->info.id)
			continue;

		if (j >= i)
			break;

		guc_g2g_deregister(guc, j, GUC_G2G_TYPE_OUT);
		guc_g2g_deregister(guc, j, GUC_G2G_TYPE_IN);
	}

	return err;
}

/*
 * Initialise the GuC parameter block before starting the firmware
 * transfer. These parameters are read by the firmware on startup
 * and cannot be changed thereafter.
 */
void intel_guc_write_params(struct intel_guc *guc)
{
	struct intel_uncore *uncore = guc_to_gt(guc)->uncore;
	int i;

	/*
	 * All SOFT_SCRATCH registers are in FORCEWAKE_GT domain and
	 * they are power context saved so it's ok to release forcewake
	 * when we are done here and take it again at xfer time.
	 */
	intel_uncore_forcewake_get(uncore, FORCEWAKE_GT);

	intel_uncore_write(uncore, SOFT_SCRATCH(0), 0);

	for (i = 0; i < GUC_CTL_MAX_DWORDS; i++)
		intel_uncore_write(uncore, SOFT_SCRATCH(1 + i), guc->params[i]);

	intel_uncore_forcewake_put(uncore, FORCEWAKE_GT);
}

void intel_guc_dump_time_info(struct intel_guc *guc, struct drm_printer *p)
{
	struct intel_gt *gt = guc_to_gt(guc);
	intel_wakeref_t wakeref;
	u32 stamp = 0;
	u64 ktime;

	with_intel_runtime_pm(&gt->i915->runtime_pm, wakeref)
		stamp = intel_uncore_read(gt->uncore, GUCPMTIMESTAMP);
	ktime = ktime_get_boottime_ns();

	drm_printf(p, "Kernel timestamp: 0x%08llX [%llu]\n", ktime, ktime);
	drm_printf(p, "GuC timestamp: 0x%08X [%u]\n", stamp, stamp);
	drm_printf(p, "CS timestamp frequency: %u Hz, %u ns\n",
		   gt->clock_frequency, gt->clock_period_ns);
}

static int __guc_init(struct intel_guc *guc)
{
	int ret;

	ret = intel_uc_fw_init(&guc->fw);
	if (ret)
		goto out;

	ret = intel_guc_log_create(&guc->log);
	if (ret)
		goto err_fw;

	ret = intel_guc_capture_init(guc);
	if (ret)
		goto err_log;

	ret = intel_guc_ads_create(guc);
	if (ret)
		goto err_capture;

	GEM_BUG_ON(!guc->ads_vma);

	ret = intel_guc_ct_init(&guc->ct);
	if (ret)
		goto err_ads;

	if (intel_guc_submission_is_used(guc)) {
		/*
		 * This is stuff we need to have available at fw load time
		 * if we are planning to enable submission later
		 */
		ret = intel_guc_submission_init(guc);
		if (ret)
			goto err_ct;
	}

	if (intel_guc_slpc_is_used(guc)) {
		ret = intel_guc_slpc_init(&guc->slpc);
		if (ret)
			goto err_submission;
	}

	ret = guc_g2g_create(guc);
	if (ret)
		goto err_slpc;

	/* now that everything is perma-pinned, initialize the parameters */
	guc_init_params(guc);

	intel_uc_fw_change_status(&guc->fw, INTEL_UC_FIRMWARE_LOADABLE);

	return 0;

err_slpc:
	if (intel_guc_slpc_is_used(guc))
		intel_guc_slpc_fini(&guc->slpc);
err_submission:
	intel_guc_submission_fini(guc);
err_ct:
	intel_guc_ct_fini(&guc->ct);
err_ads:
	intel_guc_ads_destroy(guc);
err_capture:
	intel_guc_capture_destroy(guc);
err_log:
	intel_guc_log_destroy(&guc->log);
err_fw:
	intel_uc_fw_fini(&guc->fw);
out:
	guc_probe_error(guc, "failed with %pe\n", ERR_PTR(ret));
	return ret;
}

static void __guc_fini(struct intel_guc *guc)
{
	if (!intel_uc_fw_is_loadable(&guc->fw))
		return;

	guc_g2g_destroy(guc);

	if (intel_guc_slpc_is_used(guc))
		intel_guc_slpc_fini(&guc->slpc);

	if (intel_guc_submission_is_used(guc))
		intel_guc_submission_fini(guc);

	intel_guc_ct_fini(&guc->ct);

	intel_guc_ads_destroy(guc);
	intel_guc_capture_destroy(guc);
	intel_guc_log_destroy(&guc->log);
	intel_uc_fw_fini(&guc->fw);
}

static int __vf_guc_init(struct intel_guc *guc)
{
	struct intel_gt *gt = guc_to_gt(guc);
	int err;

	GEM_BUG_ON(!IS_SRIOV_VF(gt->i915));

	err = intel_guc_ct_init(&guc->ct);
	if (err)
		return err;

	/* GuC submission is mandatory for VFs */
	err = intel_guc_submission_init(guc);
	if (err)
		goto err_ct;

	/*
	 * Disable slpc controls for VF. This cannot be done in
	 * __guc_slpc_selected since the VF probe is not complete
	 * at that point.
	 */
	guc->slpc.supported = false;
	guc->slpc.selected = false;

	/* Disable GUCRC for VF */
	guc->rc_supported = false;

	return 0;

err_ct:
	intel_guc_ct_fini(&guc->ct);
	return err;
}

static void __vf_guc_fini(struct intel_guc *guc)
{
	struct intel_gt *gt = guc_to_gt(guc);

	GEM_BUG_ON(!IS_SRIOV_VF(gt->i915));

	intel_guc_submission_fini(guc);
	intel_guc_ct_fini(&guc->ct);
}

/*
 * This function implements the MMIO based host to GuC interface.
 */
int intel_guc_send_mmio(struct intel_guc *guc, const u32 *request, u32 len,
			u32 *response_buf, u32 response_buf_size)
{
	struct intel_uncore *uncore = guc_to_gt(guc)->uncore;
	u32 header;
	int i;
	int ret;

	ret = i915_inject_probe_error(uncore->i915, -ENXIO);
	if (ret)
		return ret;

	GEM_BUG_ON(!len);
	GEM_BUG_ON(len > guc->send_regs.count);

	GEM_BUG_ON(FIELD_GET(GUC_HXG_MSG_0_ORIGIN, request[0]) != GUC_HXG_ORIGIN_HOST);
	GEM_BUG_ON(FIELD_GET(GUC_HXG_MSG_0_TYPE, request[0]) != GUC_HXG_TYPE_REQUEST);

	mutex_lock(&guc->send_mutex);
	intel_uncore_forcewake_get(uncore, guc->send_regs.fw_domains);

	GUC_DEBUG(guc, "mmio sending %*ph\n", len * 4, request);

retry:
	for (i = 0; i < len; i++)
		intel_uncore_write(uncore, guc_send_reg(guc, i), request[i]);

	intel_uncore_posting_read(uncore, guc_send_reg(guc, i - 1));

	intel_guc_notify(guc);

	/*
	 * No GuC command should ever take longer than 10ms.
	 * Fast commands should still complete in 10us.
	 */
	ret = __intel_wait_for_register_fw(uncore,
					   guc_send_reg(guc, 0),
					   GUC_HXG_MSG_0_ORIGIN,
					   FIELD_PREP(GUC_HXG_MSG_0_ORIGIN,
						      GUC_HXG_ORIGIN_GUC),
					   10, 10, &header);
	if (unlikely(ret)) {
timeout:
		guc_err(guc, "mmio request %#x: no reply %x\n",
			request[0], header);
		goto out;
	}

	if (FIELD_GET(GUC_HXG_MSG_0_TYPE, header) == GUC_HXG_TYPE_NO_RESPONSE_BUSY) {
		int loop = IS_SRIOV_VF(uncore->i915) ? 20 : 1;

#define done ({ header = intel_uncore_read(uncore, guc_send_reg(guc, 0)); \
		FIELD_GET(GUC_HXG_MSG_0_ORIGIN, header) != GUC_HXG_ORIGIN_GUC || \
		FIELD_GET(GUC_HXG_MSG_0_TYPE, header) != GUC_HXG_TYPE_NO_RESPONSE_BUSY; })

busy_loop:
		ret = wait_for(done, 1000);
		if (unlikely(ret && --loop)) {
			guc_dbg(guc, "mmio request %#x: still busy, countdown %u\n",
				request[0], loop);
			goto busy_loop;
		}
		if (unlikely(ret))
			goto timeout;
		if (unlikely(FIELD_GET(GUC_HXG_MSG_0_ORIGIN, header) !=
				       GUC_HXG_ORIGIN_GUC))
			goto proto;
#undef done
	}

	if (FIELD_GET(GUC_HXG_MSG_0_TYPE, header) == GUC_HXG_TYPE_NO_RESPONSE_RETRY) {
		u32 reason = FIELD_GET(GUC_HXG_RETRY_MSG_0_REASON, header);

		guc_dbg(guc, "mmio request %#x: retrying, reason %u\n",
			request[0], reason);
		goto retry;
	}

	if (FIELD_GET(GUC_HXG_MSG_0_TYPE, header) == GUC_HXG_TYPE_RESPONSE_FAILURE) {
		u32 hint = FIELD_GET(GUC_HXG_FAILURE_MSG_0_HINT, header);
		u32 error = FIELD_GET(GUC_HXG_FAILURE_MSG_0_ERROR, header);

		if (error == INTEL_GUC_RESPONSE_VF_MIGRATED) {
			guc_err(guc, "mmio request %#x: migrated!\n", request[0]);
			i915_sriov_vf_start_migration_recovery(uncore->i915);
			ret = -EREMOTEIO;
			goto out;
		}

		guc_err(guc, "mmio request %#x: failure %x/%u\n",
			request[0], error, hint);
		ret = -ENXIO;
		goto out;
	}

	if (FIELD_GET(GUC_HXG_MSG_0_TYPE, header) != GUC_HXG_TYPE_RESPONSE_SUCCESS) {
proto:
		guc_err(guc, "mmio request %#x: unexpected reply %#x\n",
			request[0], header);
		ret = -EPROTO;
		goto out;
	}

	if (response_buf) {
		int count = min(response_buf_size, guc->send_regs.count);

		GEM_BUG_ON(!count);

		response_buf[0] = header;

		for (i = 1; i < count; i++)
			response_buf[i] = intel_uncore_read(uncore,
							    guc_send_reg(guc, i));
		GUC_DEBUG(guc, "mmio received %*ph\n", count * 4, response_buf);

		/* Use number of copied dwords as our return value */
		ret = count;
	} else {
		GUC_DEBUG(guc, "mmio received %*ph\n", 4, &header);

		/* Use data from the GuC response as our return value */
		ret = FIELD_GET(GUC_HXG_RESPONSE_MSG_0_DATA0, header);
	}

out:
	intel_uncore_forcewake_put(uncore, guc->send_regs.fw_domains);
	mutex_unlock(&guc->send_mutex);

	return ret;
}
ALLOW_ERROR_INJECTION(intel_guc_send_mmio, ERRNO);

static void guc_dump_crash_registers(struct intel_guc *guc)
{
	struct intel_gt *gt = guc_to_gt(guc);
	intel_wakeref_t wakeref;
	u32 addr;

	guc_info(guc, "Crash register dump:\n");

	with_intel_runtime_pm(gt->uncore->rpm, wakeref) {
		for (addr = GUCDBG_RANGE_START; addr < GUCDBG_RANGE_END; addr += sizeof(u32) * 4) {
			u32 val[4], i;

			for (i = 0; i < 4; i++) {
				i915_reg_t reg = _MMIO(addr + sizeof(u32) * i);

				val[i] = intel_uncore_read(gt->uncore, reg);
			}

			guc_info(guc, "  [%#04x] %#010x %#010x %#010x %#010x\n",
				 addr, val[0], val[1], val[2], val[3]);
		}
	}
}

int intel_guc_crash_process_msg(struct intel_guc *guc, u32 action)
{
	if (action == INTEL_GUC_ACTION_NOTIFY_CRASH_DUMP_POSTED)
		guc_err(guc, "Crash dump notification\n");
	else if (action == INTEL_GUC_ACTION_NOTIFY_EXCEPTION)
		guc_err(guc, "Exception notification\n");
	else
		guc_err(guc, "Unknown crash notification\n");

	guc_dump_crash_registers(guc);
	queue_work(system_unbound_wq, &guc->dead_guc_worker);

	return 0;
}

int intel_guc_to_host_process_recv_msg(struct intel_guc *guc,
				       const u32 *payload, u32 len)
{
	u32 msg;

	if (unlikely(!len))
		return -EPROTO;

	/* Make sure to handle only enabled messages */
	msg = payload[0] & guc->msg_enabled_mask;

	if (msg & INTEL_GUC_RECV_MSG_CRASH_DUMP_POSTED)
		guc_err(guc, "Received early crash dump notification!\n");
	if (msg & INTEL_GUC_RECV_MSG_EXCEPTION)
		guc_err(guc, "Received early exception notification!\n");

	if (msg & (INTEL_GUC_RECV_MSG_CRASH_DUMP_POSTED | INTEL_GUC_RECV_MSG_EXCEPTION)) {
		guc_dump_crash_registers(guc);
		queue_work(system_unbound_wq, &guc->dead_guc_worker);
	}

	return 0;
}

/**
 * intel_guc_set_schedule_mode() - request GuC insert scheduling delays
 * @guc:	the guc
 * @mode:	the scheduling mode
 * @delay:	the delay in milliseconds
 *
 * The user can request a stall by providing a non-zero value for @delay.
 * When this occurs, the GuC will continue inserting scheduling stalls
 * until such a time as the user sets the stall @delay to zero.
 * Once @delay is set to zero, the GuC will return to normal scheduling.
 */
int intel_guc_set_schedule_mode(struct intel_guc *guc,
				enum intel_guc_scheduler_mode mode, u32 delay)
{
	u32 action[] = {
		INTEL_GUC_ACTION_SET_SCHEDULING_MODE,
		mode,
		delay
	};

	return intel_guc_send(guc, action, ARRAY_SIZE(action));
}

/**
 * intel_guc_auth_huc() - Send action to GuC to authenticate HuC ucode
 * @guc: intel_guc structure
 * @rsa_offset: rsa offset w.r.t ggtt base of huc vma
 *
 * Triggers a HuC firmware authentication request to the GuC via intel_guc_send
 * INTEL_GUC_ACTION_AUTHENTICATE_HUC interface. This function is invoked by
 * intel_huc_auth().
 *
 * Return:	non-zero code on error
 */
int intel_guc_auth_huc(struct intel_guc *guc, u32 rsa_offset)
{
	u32 action[] = {
		INTEL_GUC_ACTION_AUTHENTICATE_HUC,
		rsa_offset
	};

	return intel_guc_send(guc, action, ARRAY_SIZE(action));
}

/**
 * intel_guc_suspend() - notify GuC entering suspend state
 * @guc:	the guc
 */
int intel_guc_suspend(struct intel_guc *guc)
{
	struct intel_gt *gt = guc_to_gt(guc);
	bool fake_int = false;
	int ret;
	u32 action[] = {
		INTEL_GUC_ACTION_CLIENT_SOFT_RESET,
	};

	if (!intel_guc_is_ready(guc))
		return 0;

	if (gt->i915->quiesce_gpu)
		return 0;

	/* Wa:16014207253 */
	if (gt->fake_int.enabled && gt->fake_int.delay) {
		/*
		 * Prevent the fake interrupt timer from causing the CTB
		 * code to complain about GUC_CTB_STATUS_UNUSED being set
		 * while apparently receiving G2H interrupts.
		 */
		fake_int = true;
		fake_int_timer_stop(gt);
	}

	if (intel_guc_submission_is_used(guc)) {
		/*
		 * This H2G MMIO command tears down the GuC in two steps. First it will
		 * generate a G2H CTB for every active context indicating a reset. In
		 * practice the i915 shouldn't ever get a G2H as suspend should only be
		 * called when the GPU is idle. Next, it tears down the CTBs and this
		 * H2G MMIO command completes.
		 *
		 * Don't abort on a failure code from the GuC. Keep going and do the
		 * clean up in santize() and re-initialisation on resume and hopefully
		 * the error here won't be problematic.
		 */
		ret = intel_guc_send_mmio(guc, action, ARRAY_SIZE(action), NULL, 0);
		if (ret && ret != -EREMOTEIO)
			guc_err(guc, "suspend: RESET_CLIENT action failed with %pe\n",
				ERR_PTR(ret));
	}

	/* Signal that the GuC isn't running. */
	intel_guc_sanitize(guc);

	if (fake_int)
		fake_int_timer_start(gt);

	return 0;
}

/**
 * intel_guc_resume() - notify GuC resuming from suspend state
 * @guc:	the guc
 */
int intel_guc_resume(struct intel_guc *guc)
{
	/*
	 * NB: This function can still be called even if GuC submission is
	 * disabled, e.g. if GuC is enabled for HuC authentication only. Thus,
	 * if any code is later added here, it must be support doing nothing
	 * if submission is disabled (as per intel_guc_suspend).
	 */
	return 0;
}

/**
 * DOC: GuC Memory Management
 *
 * GuC can't allocate any memory for its own usage, so all the allocations must
 * be handled by the host driver. GuC accesses the memory via the GGTT, with the
 * exception of the top and bottom parts of the 4GB address space, which are
 * instead re-mapped by the GuC HW to memory location of the FW itself (WOPCM)
 * or other parts of the HW. The driver must take care not to place objects that
 * the GuC is going to access in these reserved ranges. The layout of the GuC
 * address space is shown below:
 *
 * ::
 *
 *     +===========> +====================+ <== FFFF_FFFF
 *     ^             |      Reserved      |
 *     |             +====================+ <== GUC_GGTT_TOP
 *     |             |                    |
 *     |             |        DRAM        |
 *    GuC            |                    |
 *  Address    +===> +====================+ <== GuC ggtt_pin_bias
 *   Space     ^     |                    |
 *     |       |     |                    |
 *     |      GuC    |        GuC         |
 *     |     WOPCM   |       WOPCM        |
 *     |      Size   |                    |
 *     |       |     |                    |
 *     v       v     |                    |
 *     +=======+===> +====================+ <== 0000_0000
 *
 * The lower part of GuC Address Space [0, ggtt_pin_bias) is mapped to GuC WOPCM
 * while upper part of GuC Address Space [ggtt_pin_bias, GUC_GGTT_TOP) is mapped
 * to DRAM. The value of the GuC ggtt_pin_bias is the GuC WOPCM size.
 */

static struct i915_vma *guc_vma_from_obj(struct intel_guc *guc,
					 struct drm_i915_gem_object *obj, u32 bias)
{
	struct intel_gt *gt = guc_to_gt(guc);
	struct i915_vma *vma;
	unsigned int flags;
	int ret;

	vma = i915_vma_instance(obj, &gt->ggtt->vm, NULL);
	if (IS_ERR(vma))
		return vma;

	flags = max(bias, i915_ggtt_pin_bias(vma)) | PIN_OFFSET_BIAS;
	ret = i915_ggtt_pin(vma, NULL, 0, flags);
	if (ret) {
		vma = ERR_PTR(ret);
		return vma;
	}

	return i915_vma_make_unshrinkable(vma);
}

/**
 * intel_guc_allocate_vma_with_bias() - Allocate a GGTT VMA for GuC usage above
 * 					the provided offset
 * @guc:	the guc
 * @size:	size of area to allocate (both virtual space and memory)
 * @bias:	minimum GGTT offset to be used for the vma
 *
 * This is a wrapper to create an object for use with the GuC, to be used for
 * GuC objects with particular base offset requirements, and guarantees that the
 * object is pinned above both the provided bias and ggtt_pin_bias. For objects
 * with no extra offset requirements, use intel_guc_allocate_vma() instead.
 *
 * Return:	A i915_vma if successful, otherwise an ERR_PTR.
 */
struct i915_vma *intel_guc_allocate_vma_with_bias(struct intel_guc *guc,
						  u32 size, u32 bias)
{
	/*
	 * All objects may trigger the I915_WA_FORCE_SMEM_OBJECT depending on
	 * the platform. For the few cases this isn't desired,
	 * __intel_guc_allocate_vma_with_bias() has to be used directly
	 */
	bool force_smem = i915_is_mem_wa_enabled(guc_to_gt(guc)->i915,
						 I915_WA_FORCE_SMEM_OBJECT);

	return __intel_guc_allocate_vma_with_bias(guc, size, bias, force_smem);
}

struct i915_vma *__intel_guc_allocate_vma_with_bias(struct intel_guc *guc,
						    u32 size, u32 bias, bool force_smem)
{
	struct intel_gt *gt = guc_to_gt(guc);
	struct drm_i915_gem_object *obj;
	struct i915_vma *vma;

	if (HAS_LMEM(gt->i915) && !force_smem)
		obj = intel_gt_object_create_lmem(gt, size,
						  I915_BO_ALLOC_CONTIGUOUS);
	else
		obj = i915_gem_object_create_shmem(gt->i915, size);
	if (IS_ERR(obj))
		return ERR_CAST(obj);

	/*
	 * Wa_22016122933: For MTL the shared memory needs to be mapped
	 * as WC on CPU side and UC (PAT index 2) on GPU side
	 */
	if (IS_METEORLAKE(gt->i915))
		i915_gem_object_set_cache_coherency(obj, I915_CACHE_NONE);

	obj->flags |= I915_BO_CPU_CLEAR;
	vma = guc_vma_from_obj(guc, obj, bias);
	if (IS_ERR(vma))
		i915_gem_object_put(obj);

	return vma;
}

/**
 * intel_guc_allocate_vma() - Allocate a GGTT VMA for GuC usage
 * @guc:	the guc
 * @size:	size of area to allocate (both virtual space and memory)
 *
 * This is a wrapper to create an object for use with the GuC. In order to
 * use it inside the GuC, an object needs to be pinned lifetime, so we allocate
 * both some backing storage and a range inside the Global GTT. We must pin
 * it in the GGTT somewhere other than than [0, GUC ggtt_pin_bias) because that
 * range is reserved inside GuC.
 *
 * Return:	A i915_vma if successful, otherwise an ERR_PTR.
 */
struct i915_vma *intel_guc_allocate_vma(struct intel_guc *guc, u32 size)
{
	return intel_guc_allocate_vma_with_bias(guc, size, 0);
}

/**
 * intel_guc_allocate_and_map_vma() - Allocate and map VMA for GuC usage
 * @guc:	the guc
 * @size:	size of area to allocate (both virtual space and memory)
 * @out_vma:	return variable for the allocated vma pointer
 * @out_vaddr:	return variable for the obj mapping
 *
 * This wrapper calls intel_guc_allocate_vma() and then maps the allocated
 * object with I915_MAP_WB.
 *
 * Return:	0 if successful, a negative errno code otherwise.
 */
int intel_guc_allocate_and_map_vma(struct intel_guc *guc, u32 size,
				   struct i915_vma **out_vma, void **out_vaddr)
{
	bool force_smem = i915_is_mem_wa_enabled(guc_to_gt(guc)->i915,
						 I915_WA_FORCE_SMEM_OBJECT);

	return __intel_guc_allocate_and_map_vma(guc, size, force_smem,
						out_vma, out_vaddr);
}

int __intel_guc_allocate_and_map_vma(struct intel_guc *guc, u32 size,
				     bool force_smem,
				     struct i915_vma **out_vma, void **out_vaddr)
{
	struct i915_vma *vma;
	void *vaddr;

	vma = __intel_guc_allocate_vma_with_bias(guc, size, 0, force_smem);
	if (IS_ERR(vma))
		return PTR_ERR(vma);

	vaddr = i915_gem_object_pin_map_unlocked(vma->obj,
						 i915_coherent_map_type(guc_to_gt(guc)->i915,
									vma->obj, true));
	if (IS_ERR(vaddr)) {
		i915_vma_unpin_and_release(&vma, 0);
		return PTR_ERR(vaddr);
	}

	*out_vma = vma;
	*out_vaddr = vaddr;

	return 0;
}

static int __guc_action_self_cfg(struct intel_guc *guc, u16 key, u16 len, u64 value)
{
	u32 request[HOST2GUC_SELF_CFG_REQUEST_MSG_LEN] = {
		FIELD_PREP(GUC_HXG_MSG_0_ORIGIN, GUC_HXG_ORIGIN_HOST) |
		FIELD_PREP(GUC_HXG_MSG_0_TYPE, GUC_HXG_TYPE_REQUEST) |
		FIELD_PREP(GUC_HXG_REQUEST_MSG_0_ACTION, GUC_ACTION_HOST2GUC_SELF_CFG),
		FIELD_PREP(HOST2GUC_SELF_CFG_REQUEST_MSG_1_KLV_KEY, key) |
		FIELD_PREP(HOST2GUC_SELF_CFG_REQUEST_MSG_1_KLV_LEN, len),
		FIELD_PREP(HOST2GUC_SELF_CFG_REQUEST_MSG_2_VALUE32, lower_32_bits(value)),
		FIELD_PREP(HOST2GUC_SELF_CFG_REQUEST_MSG_3_VALUE64, upper_32_bits(value)),
	};
	int ret;

	GEM_BUG_ON(len > 2);
	GEM_BUG_ON(len == 1 && upper_32_bits(value));

	/* Self config must go over MMIO */
	ret = intel_guc_send_mmio(guc, request, ARRAY_SIZE(request), NULL, 0);

	if (unlikely(ret < 0))
		return ret;
	if (unlikely(ret > 1))
		return -EPROTO;
	if (unlikely(!ret))
		return -ENOKEY;

	return 0;
}

static int __guc_self_cfg(struct intel_guc *guc, u16 key, u16 len, u64 value)
{
	int err = __guc_action_self_cfg(guc, key, len, value);

	if (unlikely(err))
		guc_probe_error(guc, "Unsuccessful self-config (%pe) key %#hx value %#llx\n",
				ERR_PTR(err), key, value);
	return err;
}

int intel_guc_self_cfg32(struct intel_guc *guc, u16 key, u32 value)
{
	return __guc_self_cfg(guc, key, 1, value);
}

int intel_guc_self_cfg64(struct intel_guc *guc, u16 key, u64 value)
{
	return __guc_self_cfg(guc, key, 2, value);
}

static long must_wait_woken(struct wait_queue_entry *wq_entry, long timeout)
{
	/*
	 * This is equivalent to wait_woken() with the exception that
	 * we do not wake up early if the kthread task has been completed.
	 * As we are called from page reclaim in any task context,
	 * we may be invoked from stopped kthreads, but we *must*
	 * complete the wait from the HW .
	 *
	 * A second problem is that since we are called under reclaim
	 * and wait_woken() inspected the thread state, it makes an invalid
	 * assumption that all PF_KTHREAD tasks have set_kthread_struct()
	 * called upon them, and will trigger a GPF in is_kthread_should_stop().
	 */
	do {
		set_current_state(TASK_UNINTERRUPTIBLE);
		if (READ_ONCE(wq_entry->flags) & WQ_FLAG_WOKEN)
			break;

		timeout = schedule_timeout(timeout);
	} while (timeout);
	__set_current_state(TASK_RUNNING);

	/* See wait_woken() and woken_wake_function() */
	smp_store_mb(wq_entry->flags, wq_entry->flags & ~WQ_FLAG_WOKEN);

	return timeout;
}

static int guc_send_invalidate_tlb(struct intel_guc *guc, u32 *action, u32 size)
{
	struct intel_guc_tlb_wait _wq, *wq = &_wq;
	DEFINE_WAIT_FUNC(wait, woken_wake_function);
	struct intel_gt *gt = guc_to_gt(guc);
	u64 rstcnt = atomic_read(&gt->reset.engines_reset_count);
	int err = 0;
	u32 seqno;

	init_waitqueue_head(&_wq.wq);

	if (xa_alloc_cyclic_irq(&guc->tlb_lookup, &seqno, wq,
				xa_limit_32b, &guc->next_seqno,
				GFP_ATOMIC | __GFP_NOWARN) < 0) {
		/* Under severe memory pressure? Serialise TLB allocations */
		xa_lock_irq(&guc->tlb_lookup);
		wq = xa_load(&guc->tlb_lookup, guc->serial_slot);
		wait_event_lock_irq(wq->wq,
				    !READ_ONCE(wq->status),
				    guc->tlb_lookup.xa_lock);
		/*
		 * Update wq->status under lock to ensure only one waiter can
		 * issue the tlb invalidation command using the serial slot at a
		 * time. The condition is set to false before releasing the lock
		 * so that other caller continue to wait until woken up again.
		 */
		wq->status = 1;
		xa_unlock_irq(&guc->tlb_lookup);

		seqno = guc->serial_slot;
	}

	action[1] = seqno;

	add_wait_queue(&wq->wq, &wait);

	err = intel_guc_send_busy_loop(guc, action, size, G2H_LEN_DW_INVALIDATE_TLB, true);
	if (err) {
		/*
		 * XXX: Failure of tlb invalidation is critical and would
		 * warrant a gt reset.
		 */
		goto out;
	}

	intel_boost_fake_int_timer(gt, true);

/*
 * GuC has a timeout of 1ms for a tlb invalidation response from GAM. On a
 * timeout GuC drops the request and has no mechanism to notify the host about
 * the timeout. So keep a larger timeout that accounts for this individual
 * timeout and max number of outstanding invalidation requests that can be
 * queued in CT buffer.
 */
#define OUTSTANDING_GUC_TIMEOUT_PERIOD  (HZ)
	if (!must_wait_woken(&wait, OUTSTANDING_GUC_TIMEOUT_PERIOD)) {
		/*
		 * XXX: Failure of tlb invalidation is critical and would
		 * warrant a gt reset.
		 */
		if (gt->reset.flags == 0 &&
		    rstcnt == atomic_read(&gt->reset.engines_reset_count))
			drm_info(&gt->i915->drm,
				 "tlb invalidation response timed out for seqno %u\n", seqno);

		err = -ETIME;
	}
out:
	remove_wait_queue(&wq->wq, &wait);
	if (seqno != guc->serial_slot)
		xa_erase_irq(&guc->tlb_lookup, seqno);

	intel_boost_fake_int_timer(gt, false);

	return err;
}

 /* Full TLB invalidation */
int intel_guc_invalidate_tlb_full(struct intel_guc *guc,
				  enum intel_guc_tlb_inval_mode mode)
{
	u32 action[] = {
		INTEL_GUC_ACTION_TLB_INVALIDATION,
		0,
		INTEL_GUC_TLB_INVAL_FULL << INTEL_GUC_TLB_INVAL_TYPE_SHIFT |
			mode << INTEL_GUC_TLB_INVAL_MODE_SHIFT |
			INTEL_GUC_TLB_INVAL_FLUSH_CACHE,
	};

	if (!INTEL_GUC_SUPPORTS_TLB_INVALIDATION(guc))
		return -EINVAL;

	return guc_send_invalidate_tlb(guc, action, ARRAY_SIZE(action));
}

/*
 * Selective TLB Invalidation for Address Range:
 * TLB's in the Address Range is Invalidated across all engines.
 */
int intel_guc_invalidate_tlb_page_selective(struct intel_guc *guc,
					    enum intel_guc_tlb_inval_mode mode,
					    u64 start, u64 length, u32 asid)
{
	u64 vm_total = BIT_ULL(INTEL_INFO(guc_to_gt(guc)->i915)->ppgtt_size);

	/*
	 * For page selective invalidations, this specifies the number of contiguous
	 * PPGTT pages that needs to be invalidated.
	 */
	u32 address_mask = length >= vm_total ? 0 : ilog2(length) - ilog2(SZ_4K);
	u32 action[] = {
		INTEL_GUC_ACTION_TLB_INVALIDATION,
		0,
		INTEL_GUC_TLB_INVAL_PAGE_SELECTIVE << INTEL_GUC_TLB_INVAL_TYPE_SHIFT |
			mode << INTEL_GUC_TLB_INVAL_MODE_SHIFT |
			INTEL_GUC_TLB_INVAL_FLUSH_CACHE,
		asid,
		length >= vm_total ? 1 : lower_32_bits(start),
		upper_32_bits(start),
		address_mask,
	};

	if (!INTEL_GUC_SUPPORTS_TLB_INVALIDATION_SELECTIVE(guc))
		return -EINVAL;

	GEM_BUG_ON(length < SZ_4K);
	GEM_BUG_ON(!is_power_of_2(length));
	GEM_BUG_ON(!IS_ALIGNED(start, length));
	GEM_BUG_ON(range_overflows(start, length, vm_total));

	return guc_send_invalidate_tlb(guc, action, ARRAY_SIZE(action));
}

/*
 * Selective TLB Invalidation for Context:
 * Invalidates all TLB's for a specific context across all engines.
 */
int intel_guc_invalidate_tlb_page_selective_ctx(struct intel_guc *guc,
						enum intel_guc_tlb_inval_mode mode,
						u64 start, u64 length, u32 ctxid)
{
	u64 vm_total = BIT_ULL(INTEL_INFO(guc_to_gt(guc)->i915)->ppgtt_size);
	u32 address_mask = (ilog2(length) - ilog2(I915_GTT_PAGE_SIZE_4K));
	u32 full_range = vm_total == length;
	u32 action[] = {
		INTEL_GUC_ACTION_TLB_INVALIDATION,
		0,
		INTEL_GUC_TLB_INVAL_PAGE_SELECTIVE_CTX << INTEL_GUC_TLB_INVAL_TYPE_SHIFT |
			mode << INTEL_GUC_TLB_INVAL_MODE_SHIFT |
			INTEL_GUC_TLB_INVAL_FLUSH_CACHE,
		ctxid,
		full_range ? full_range : lower_32_bits(start),
		full_range ? 0 : upper_32_bits(start),
		full_range ? 0 : address_mask,
	};

	if (!INTEL_GUC_SUPPORTS_TLB_INVALIDATION_SELECTIVE(guc))
		return -EINVAL;

	GEM_BUG_ON(length < SZ_4K);
	GEM_BUG_ON(!is_power_of_2(length));
	GEM_BUG_ON(length & GENMASK(ilog2(SZ_16M) - 1, ilog2(SZ_2M) + 1));
	GEM_BUG_ON(!IS_ALIGNED(start, length));
	GEM_BUG_ON(range_overflows(start, length, vm_total));

	return guc_send_invalidate_tlb(guc, action, ARRAY_SIZE(action));
}

/*
 * Guc TLB Invalidation: Invalidate the TLB's of GuC itself.
 */
int intel_guc_invalidate_tlb_guc(struct intel_guc *guc,
				 enum intel_guc_tlb_inval_mode mode)
{
	u32 action[] = {
		INTEL_GUC_ACTION_TLB_INVALIDATION,
		0,
		INTEL_GUC_TLB_INVAL_GUC << INTEL_GUC_TLB_INVAL_TYPE_SHIFT |
			mode << INTEL_GUC_TLB_INVAL_MODE_SHIFT |
			INTEL_GUC_TLB_INVAL_FLUSH_CACHE,
	};

	if (!INTEL_GUC_SUPPORTS_TLB_INVALIDATION(guc))
		return -EINVAL;

	return guc_send_invalidate_tlb(guc, action, ARRAY_SIZE(action));
}

int intel_guc_invalidate_tlb_all(struct intel_guc *guc)
{
	u32 action[] = {
		INTEL_GUC_ACTION_TLB_INVALIDATION_ALL,
		0,
		INTEL_GUC_TLB_INVAL_MODE_HEAVY << INTEL_GUC_TLB_INVAL_MODE_SHIFT |
		INTEL_GUC_TLB_INVAL_FLUSH_CACHE,
	};

	GEM_BUG_ON(!INTEL_GUC_SUPPORTS_TLB_INVALIDATION(guc));

	return guc_send_invalidate_tlb(guc, action, ARRAY_SIZE(action));
}

/**
 * intel_guc_load_status - dump information about GuC load status
 * @guc: the GuC
 * @p: the &drm_printer
 *
 * Pretty printer for GuC load status.
 */
void intel_guc_load_status(struct intel_guc *guc, struct drm_printer *p)
{
	struct intel_gt *gt = guc_to_gt(guc);
	struct intel_uncore *uncore = gt->uncore;
	intel_wakeref_t wakeref;

	if (!intel_guc_is_supported(guc)) {
		drm_printf(p, "GuC not supported\n");
		return;
	}

	if (!intel_guc_is_wanted(guc)) {
		drm_printf(p, "GuC disabled\n");
		return;
	}

	intel_uc_fw_dump(&guc->fw, p);

	if (IS_SRIOV_VF(guc_to_gt(guc)->i915))
		return;

	with_intel_runtime_pm(uncore->rpm, wakeref) {
		u32 status = intel_uncore_read(uncore, GUC_STATUS);
		u32 i;

		drm_printf(p, "GuC status 0x%08x:\n", status);
		drm_printf(p, "\tBootrom status = 0x%x\n",
			   (status & GS_BOOTROM_MASK) >> GS_BOOTROM_SHIFT);
		drm_printf(p, "\tuKernel status = 0x%x\n",
			   (status & GS_UKERNEL_MASK) >> GS_UKERNEL_SHIFT);
		drm_printf(p, "\tMIA Core status = 0x%x\n",
			   (status & GS_MIA_MASK) >> GS_MIA_SHIFT);
		drm_puts(p, "Scratch registers:\n");
		for (i = 0; i < 16; i++) {
			drm_printf(p, "\t%2d: \t0x%x\n",
				   i, intel_uncore_read(uncore, SOFT_SCRATCH(i)));
		}
	}
}

void intel_guc_print_info(struct intel_guc *guc, struct drm_printer *p)
{
	intel_guc_load_status(guc, p);

	if (intel_guc_submission_is_used(guc)) {
		drm_printf(p, "GuC Interrupts: { count: %lu, total: %lluns, avg: %luns, max: %luns }\n",
			   READ_ONCE(guc->stats.irq.count),
			   READ_ONCE(guc->stats.irq.total),
			   ewma_irq_time_read(&guc->stats.irq.avg),
			   READ_ONCE(guc->stats.irq.max));
		intel_guc_ct_print_info(&guc->ct, p);
		intel_guc_submission_print_info(guc, p);
		intel_guc_ads_print_policy_info(guc, p);
		intel_guc_submission_print_context_info(guc, p);
	}
}

static const struct intel_guc_ops guc_ops_default = {
	.init = __guc_init,
	.fini = __guc_fini,
};

static const struct intel_guc_ops guc_ops_vf = {
	.init = __vf_guc_init,
	.fini = __vf_guc_fini,
};
