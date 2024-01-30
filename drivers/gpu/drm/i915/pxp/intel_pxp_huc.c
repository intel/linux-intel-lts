// SPDX-License-Identifier: MIT
/*
 * Copyright(c) 2021, Intel Corporation. All rights reserved.
 */

#include <drm/i915_drm.h>

#include "gem/i915_gem_region.h"
#include "gt/intel_gt.h"

#include "i915_drv.h"
#include "intel_pxp.h"
#include "intel_pxp_huc.h"
#include "intel_pxp_tee.h"
#include "intel_pxp_types.h"
#include "intel_pxp_tee_interface.h"

int intel_pxp_huc_load_and_auth(struct intel_pxp *pxp)
{
	struct intel_gt *gt = pxp_to_gt(pxp);
	struct intel_huc *huc = &gt->uc.huc;
	struct pxp_tee_start_huc_auth_in huc_in = {0};
	struct pxp_tee_start_huc_auth_out huc_out = {0};
	u8 client_id = 0;
	u8 fence_id = 0;
	int err;

	huc_in.header.api_version = PXP_TEE_43_APIVER;
	huc_in.header.command_id  = PXP_TEE_43_START_HUC_AUTH;
	huc_in.header.status      = 0;
	huc_in.header.buffer_len  = sizeof(huc_in.huc_base_address);

	if (intel_uc_supports_gsc_uc(&gt->uc)) {
		huc_in.huc_base_address = huc->fw.dummy.node.start;

		err = intel_pxp_gsc_fw_message(pxp, &huc_in, sizeof(huc_in),
					       &huc_out, sizeof(huc_out));
	} else if (pxp->pxp_component) {
		huc_in.huc_base_address = i915_gem_object_get_dma_address(huc->fw.obj, 0);

		err = intel_pxp_tee_stream_message(pxp, client_id, fence_id,
						   &huc_in, sizeof(huc_in),
						   &huc_out, sizeof(huc_out));
	} else {
		err = -ENODEV;
	}
	if (err < 0) {
		drm_err(&gt->i915->drm,
			"Failed to send HuC load and auth command to GSC [%d]!\n",
			err);
		return err;
	}

	/*
	 * HuC does sometimes survive suspend/resume (it depends on how "deep"
	 * a sleep state the device reaches) so we can end up here on resume
	 * with HuC already loaded, in which case the GSC will return
	 * PXP_STATUS_OP_NOT_PERMITTED. We can therefore consider the GuC
	 * correctly transferred in this scenario; if the same error is ever
	 * returned with HuC not loaded we'll still catch it when we check the
	 * authentication bit later.
	 */
	if (huc_out.header.status != PXP_STATUS_SUCCESS &&
	    huc_out.header.status != PXP_STATUS_OP_NOT_PERMITTED) {
		drm_err(&gt->i915->drm,
			"HuC load failed with GSC error = 0x%x\n",
			huc_out.header.status);
		return -EPROTO;
	}

	return 0;
}
