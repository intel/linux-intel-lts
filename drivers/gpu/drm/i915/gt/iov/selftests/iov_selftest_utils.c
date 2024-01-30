// SPDX-License-Identifier: MIT
/*
 * Copyright © 2021 Intel Corporation
 */

#include "gt/intel_gtt.h"
#include "gt/iov/intel_iov_utils.h"

#include "iov_selftest_utils.h"

bool intel_iov_check_ggtt_vfid(struct intel_iov *iov, void __iomem *pte_addr, u16 vfid)
{
	GEM_BUG_ON(!HAS_SRIOV(iov_to_i915(iov)));

	if (i915_ggtt_has_xehpsdv_pte_vfid_mask(iov_to_gt(iov)->ggtt))
		return vfid == FIELD_GET(XEHPSDV_GGTT_PTE_VFID_MASK, gen8_get_pte(pte_addr));
	else
		return vfid == FIELD_GET(TGL_GGTT_PTE_VFID_MASK, gen8_get_pte(pte_addr));
}
