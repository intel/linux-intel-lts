// SPDX-License-Identifier: MIT
/*
 * Copyright © 2021 Intel Corporation
 */

#include <linux/types.h>

#include "gt/intel_gt.h"
#include "intel_gsc_uc.h"
#include "i915_drv.h"

static bool gsc_engine_supported(struct intel_gt *gt)
{
	return gt->info.engine_mask ?
		HAS_ENGINE(gt, GSC0) :
		INTEL_INFO(gt->i915)->platform_engine_mask & BIT(GSC0);
}

void intel_gsc_uc_init_early(struct intel_gsc_uc *gsc)
{
	intel_uc_fw_init_early(&gsc->fw, INTEL_UC_FW_TYPE_GSC);

	/* we can arrive here from i915_driver_early_probe for primary
	 * GT with it being not fully setup hence check device info's
	 * engine mask
	 */
	if (!gsc_engine_supported(gsc_uc_to_gt(gsc))) {
		intel_uc_fw_change_status(&gsc->fw, INTEL_UC_FIRMWARE_NOT_SUPPORTED);
		return;
	}
}

int intel_gsc_uc_init(struct intel_gsc_uc *gsc)
{
	static struct lock_class_key gsc_lock;
	struct intel_gt *gt = gsc_uc_to_gt(gsc);
	struct drm_i915_private *i915 = gt->i915;
	struct intel_engine_cs *engine = gt->engine[GSC0];
	struct intel_context *ce;
	struct i915_vma *vma;
	int err;

	err = intel_uc_fw_init(&gsc->fw);
	if (err)
		goto out;

	vma = intel_guc_allocate_vma(&gt->uc.guc, SZ_8M);
	if (IS_ERR(vma)) {
		err = PTR_ERR(vma);
		goto out_fw;
	}

	gsc->local = vma;

	ce = intel_engine_create_pinned_context(engine, engine->gt->vm, SZ_4K,
						I915_GEM_HWS_GSC_ADDR,
						&gsc_lock, "gsc_context");
	if (IS_ERR(ce)) {
		drm_err(&gt->i915->drm,
			"failed to create GSC CS ctx for FW communication\n");
		err =  PTR_ERR(ce);
		goto out_vma;
	}

	gsc->ce = ce;

	intel_uc_fw_change_status(&gsc->fw, INTEL_UC_FIRMWARE_LOADABLE);

	return 0;

out_vma:
	i915_vma_unpin_and_release(&gsc->local, 0);
out_fw:
	intel_uc_fw_fini(&gsc->fw);
out:
	i915_probe_error(i915, "failed with %d\n", err);
	return err;
}

void intel_gsc_uc_fini(struct intel_gsc_uc *gsc)
{
	if (!intel_uc_fw_is_loadable(&gsc->fw))
		return;

	if (gsc->ce)
		intel_engine_destroy_pinned_context(fetch_and_zero(&gsc->ce));

	i915_vma_unpin_and_release(&gsc->local, 0);
	intel_uc_fw_fini(&gsc->fw);
}
