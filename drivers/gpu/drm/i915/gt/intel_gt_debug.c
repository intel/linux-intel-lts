// SPDX-License-Identifier: MIT
/*
 * Copyright © 2022 Intel Corporation
 */

#include "i915_drv.h"
#include "intel_uncore.h"
#include "gt/intel_gt_debug.h"
#include "gt/intel_gt_mcr.h"
#include "gt/intel_gt_regs.h"
#include "gt/intel_gt_pm.h"
#include "gt/intel_gt.h"

static int intel_gt_for_each_compute_slice_subslice_fw(struct intel_gt *gt,
						       int (*fn)(struct intel_gt *gt,
								 void *data,
								 unsigned int slice,
								 unsigned int subslice,
								 bool subslice_present),
						       void *data)
{
	struct intel_uncore * const uncore = gt->uncore;
	struct sseu_dev_info *sseu = &gt->info.sseu;
	unsigned int dss, group, instance;
	bool present;
	int lastdss;
	int ret = 0;

	GEM_WARN_ON(!intel_sseu_subslice_total(sseu));

	lockdep_assert_held(&uncore->lock);

	if (GRAPHICS_VER_FULL(gt->i915) >= IP_VER(12, 50))
		lastdss = intel_sseu_highest_xehp_dss(sseu->subslice_mask);
	else
		lastdss = sseu->max_slices * sseu->max_subslices - 1;

	for_each_possible_ss_steering(dss, gt, group, instance, present) {
		if (dss > lastdss)
			break;

		ret = fn(gt, data, group, instance, present);
		if (ret)
			return ret;
	}

	return 0;
}

/**
 * intel_gt_for_each_compute_slice_subslice - Walk slices and sublices with MCR
 *
 * @gt: pointer to struct intel_gt
 * @write: if writes are going to be done
 * @fn: callback function for each slice/subslice with flag if present
 * @data: arbitrary data to be used by the callback
 *
 * Return: 0 if walk completed. nonzero if the callback returned nonzero
 *
 */
int intel_gt_for_each_compute_slice_subslice(struct intel_gt *gt,
					     int (*fn)(struct intel_gt *gt,
						       void *data,
						       unsigned int slice,
						       unsigned int subslice,
						       bool subslice_present),
					     void *data)
{
	const enum forcewake_domains fw_domains = FORCEWAKE_RENDER | FORCEWAKE_GT;
	struct intel_uncore * const uncore = gt->uncore;
	intel_wakeref_t wakeref;
	unsigned long flags;
	int ret = 0;

	with_intel_runtime_pm(gt->uncore->rpm, wakeref) {
		intel_gt_mcr_lock(gt, &flags);
		spin_lock(&uncore->lock);
		intel_uncore_forcewake_get__locked(uncore, fw_domains);

		ret = intel_gt_for_each_compute_slice_subslice_fw(gt, fn, data);

		intel_uncore_forcewake_put__locked(uncore, fw_domains);
		spin_unlock(&uncore->lock);
		intel_gt_mcr_unlock(gt, flags);
	}

	return ret;
}

static int read_first_attention_ss_fw(struct intel_gt *gt, void *data,
				      unsigned int group, unsigned int instance,
				      bool ss_present)
{
	unsigned int row;

	if (!ss_present)
		return 0;

	for (row = 0; row < 2; row++) {
		u32 val;

		val = intel_gt_mcr_read_fw(gt, TD_ATT(row), group, instance);

		if (val)
			return 1;
	}

	return 0;
}

/*
 * On EU_ATT register there are two rows with 4 eus each with 8 threads per eu.
 * For example on some TGL there is one slice and 6 sublices. This makes 48 eus.
 * However the sseu reports 16 eus per subslice. This is explained by
 * lockstep execution units so there are 2 eus working in pairs.
 * With this in mind the total execution unit number matches but our attention
 * resolution is then half.
 */

#define MAX_EUS_PER_ROW 4u
#define MAX_THREADS 8u

/*
 * Using the userspace view for slice/subslices seems wrong but this is only
 * for userspace to match the bitmask sizes. When we divide the actual
 * gslices for hw access, sizes should match.
 *
 */
int intel_gt_eu_attention_bitmap_size(struct intel_gt *gt)
{
	const struct sseu_dev_info * const sseu = &gt->info.sseu;

	BUILD_BUG_ON(MAX_EUS_PER_ROW * TD_EU_ATTENTION_MAX_ROWS * MAX_THREADS !=
		     2 * sizeof(u32) * BITS_PER_BYTE);

	return sseu->max_slices * sseu->max_subslices *
		TD_EU_ATTENTION_MAX_ROWS * MAX_THREADS *
		MAX_EUS_PER_ROW / BITS_PER_BYTE;
}

struct attn_read_iter {
	struct intel_gt *gt;
	unsigned int i;

	unsigned int size;
	u8 *bits;
};

static int read_eu_attentions_ss_fw(struct intel_gt *gt, void *data,
				    unsigned int group, unsigned int instance, bool present)
{
	struct attn_read_iter * const iter = data;
	unsigned int row;

	for (row = 0; row < TD_EU_ATTENTION_MAX_ROWS; row++) {
		u32 val;

		if (iter->i >= iter->size)
			return 0;

		if (GEM_WARN_ON((iter->i + sizeof(val)) >
				(intel_gt_eu_attention_bitmap_size(gt))))
			return -EIO;

		if (present)
			val = intel_gt_mcr_read_fw(gt, TD_ATT(row), group, instance);
		else
			val = 0;

		memcpy(&iter->bits[iter->i], &val, sizeof(val));
		iter->i += sizeof(val);
	}

	return 0;
}

int intel_gt_eu_attention_bitmap(struct intel_gt *gt,
				 u8 *bits,
				 unsigned int bitmap_size)
{
	struct attn_read_iter iter = {
		.gt = gt,
		.i = 0,
		.size = bitmap_size,
		.bits = bits
	};

	intel_gt_for_each_compute_slice_subslice(gt,
						 read_eu_attentions_ss_fw,
						 &iter);

	return 0;
}

int intel_gt_invalidate_l3_mmio(struct intel_gt *gt)
{
	const unsigned int timeout_us = 5000;
	struct intel_uncore * const uncore = gt->uncore;
	const enum forcewake_domains fw =
		intel_uncore_forcewake_for_reg(uncore, GEN7_MISCCPCTL,
					       FW_REG_READ | FW_REG_WRITE) |
		intel_uncore_forcewake_for_reg(uncore, GEN11_GLBLINVL,
					       FW_REG_READ | FW_REG_WRITE);
	u32 cpctl, cpctl_org, inv, mask;
	int ret;

	mask = GEN12_DOP_CLOCK_GATE_RENDER_ENABLE;
	if (GRAPHICS_VER_FULL(gt->i915) >= IP_VER(12, 50))
		mask |= GEN8_DOP_CLOCK_GATE_CFCLK_ENABLE;

	spin_lock_irq(&uncore->lock);
	intel_uncore_forcewake_get__locked(uncore, fw);

	cpctl_org = intel_uncore_read_fw(uncore, GEN7_MISCCPCTL);
	if (cpctl_org & mask)
		intel_uncore_write_fw(uncore, GEN7_MISCCPCTL, cpctl_org & ~mask);

	cpctl = intel_uncore_read_fw(uncore, GEN7_MISCCPCTL);
	if (cpctl & mask) {
		ret = cpctl & GEN12_DOP_CLOCK_GATE_LOCK ? -EACCES : -ENXIO;
		/*
		 * XXX: We need to bail out as there are gens
		 * that wont survive invalidate without disabling
		 * the gating of above clocks. The resulting hang is
		 * is catastrophic and we lose the gpu in a way
		 * that even reset wont help.
		 */
		goto out;
	}

	/*
	 * XXX: There is w/a in dg2 that wants to use invalidate
	 * during context switches. We should be safe against concurrent
	 * io as we check active invalidation before we proceed
	 */
	ret = __intel_wait_for_register_fw(uncore, GEN11_GLBLINVL,
					   GEN11_L3_GLOBAL_INVALIDATE, 0,
					   timeout_us, 0, &inv);
	if (ret)
		goto out;

	intel_uncore_write_fw(uncore, GEN11_GLBLINVL,
			      inv | GEN11_L3_GLOBAL_INVALIDATE);

	ret = __intel_wait_for_register_fw(uncore, GEN11_GLBLINVL,
					   GEN11_L3_GLOBAL_INVALIDATE, 0,
					   timeout_us, 0, &inv);

out:
	if (cpctl_org != cpctl)
		intel_uncore_write_fw(uncore, GEN7_MISCCPCTL, cpctl_org);

	intel_uncore_forcewake_put__locked(uncore, fw);
	spin_unlock_irq(&uncore->lock);

	return ret;
}

/**
 * intel_gt_eu_threads_needing_attention - Query host attention
 *
 * @gt: pointer to struct intel_gt
 *
 * Return: 1 if threads waiting host attention.
 */
int intel_gt_eu_threads_needing_attention(struct intel_gt *gt)
{
	return intel_gt_for_each_compute_slice_subslice(gt,
							read_first_attention_ss_fw,
							NULL);
}
