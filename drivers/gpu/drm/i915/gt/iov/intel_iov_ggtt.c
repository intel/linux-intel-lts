// SPDX-License-Identifier: MIT
/*
 * Copyright © 2023 Intel Corporation
 */

#include "abi/iov_actions_mmio_abi.h"
#include "intel_iov_ggtt.h"
#include "intel_iov_types.h"
#include "intel_iov_query.h"
#include "intel_iov_utils.h"

void intel_iov_ggtt_vf_init_early(struct intel_iov *iov)
{
	GEM_BUG_ON(!intel_iov_is_vf(iov));

	mutex_init(&iov->vf.ptes_buffer.lock);
}

void intel_iov_ggtt_vf_release(struct intel_iov *iov)
{
	GEM_BUG_ON(!intel_iov_is_vf(iov));

	mutex_destroy(&iov->vf.ptes_buffer.lock);
}

static bool is_next_ggtt_offset(struct intel_iov *iov, u32 offset)
{
	struct intel_iov_vf_ggtt_ptes *buffer = &iov->vf.ptes_buffer;

	return (offset - (buffer->num_copies + buffer->count) == buffer->offset);
}

static bool is_pte_duplicatable(struct intel_iov *iov, gen8_pte_t pte)
{
	struct intel_iov_vf_ggtt_ptes *buffer = &iov->vf.ptes_buffer;

	return (buffer->ptes[buffer->count - 1] == pte);
}

static bool is_pte_replicable(struct intel_iov *iov, gen8_pte_t pte)
{
	struct intel_iov_vf_ggtt_ptes *buffer = &iov->vf.ptes_buffer;

	u64 new_gfn = FIELD_GET(GEN12_GGTT_PTE_ADDR_MASK, pte);
	u64 buffer_gfn = FIELD_GET(GEN12_GGTT_PTE_ADDR_MASK, buffer->ptes[buffer->count - 1]);
	u64 new_flags = FIELD_GET(MTL_GGTT_PTE_PAT_MASK, pte);
	u64 buffer_flags = FIELD_GET(MTL_GGTT_PTE_PAT_MASK, buffer->ptes[buffer->count - 1]);

	return (new_flags == buffer_flags && new_gfn - (buffer->num_copies + 1) == buffer_gfn);
}

int intel_iov_ggtt_vf_update_pte(struct intel_iov *iov, u32 offset, gen8_pte_t pte)
{
	struct intel_iov_vf_ggtt_ptes *buffer = &iov->vf.ptes_buffer;
	u8 max_copies = FIELD_MAX(VF2PF_MMIO_UPDATE_GGTT_REQUEST_MSG_1_NUM_COPIES);
	u8 max_ptes = MMIO_UPDATE_GGTT_MAX_PTES;
	u32 pte_offset = (offset >> PAGE_SHIFT) - (iov->vf.config.ggtt_base >> PAGE_SHIFT);

	BUILD_BUG_ON(MMIO_UPDATE_GGTT_MODE_DUPLICATE != VF2PF_UPDATE_GGTT32_MODE_DUPLICATE);
	BUILD_BUG_ON(MMIO_UPDATE_GGTT_MODE_REPLICATE != VF2PF_UPDATE_GGTT32_MODE_REPLICATE);
	BUILD_BUG_ON(MMIO_UPDATE_GGTT_MODE_DUPLICATE_LAST !=
		     VF2PF_UPDATE_GGTT32_MODE_DUPLICATE_LAST);
	BUILD_BUG_ON(MMIO_UPDATE_GGTT_MODE_REPLICATE_LAST !=
		     VF2PF_UPDATE_GGTT32_MODE_REPLICATE_LAST);

	GEM_BUG_ON(!intel_iov_is_vf(iov));

	if (intel_guc_ct_enabled(&iov_to_guc(iov)->ct))
		max_ptes = VF2PF_UPDATE_GGTT_MAX_PTES;

	if (!buffer->count) {
		buffer->offset = pte_offset;
		buffer->ptes[buffer->count++] = pte;
		buffer->count = 1;
		buffer->num_copies = 0;
		/**
		 * If num_copies is equal to 0, then the value
		 * of the MODE field is no matter.
		 * Let's set MODE as invalid so that we can check later
		 * if this field is set as expected.
		 */
		buffer->mode = VF_RELAY_UPDATE_GGTT_MODE_INVALID;
	} else if (!is_next_ggtt_offset(iov, pte_offset) || buffer->num_copies == max_copies) {
		goto flush;
	} else if (!buffer->num_copies) {
		if (is_pte_duplicatable(iov, pte)) {
			buffer->mode = (buffer->count == 1) ? MMIO_UPDATE_GGTT_MODE_DUPLICATE :
							      MMIO_UPDATE_GGTT_MODE_DUPLICATE_LAST;
			buffer->num_copies++;
		} else if (is_pte_replicable(iov, pte)) {
			buffer->mode = (buffer->count == 1) ? MMIO_UPDATE_GGTT_MODE_REPLICATE :
							      MMIO_UPDATE_GGTT_MODE_REPLICATE_LAST;
			buffer->num_copies++;
		} else {
			if (buffer->count == max_ptes)
				goto flush;

			buffer->ptes[buffer->count++] = pte;
		}
	} else if (buffer->count == 1 &&
		   buffer->mode == MMIO_UPDATE_GGTT_MODE_DUPLICATE &&
		   is_pte_duplicatable(iov, pte)) {
		buffer->num_copies++;
	} else if (buffer->count == 1 &&
		   buffer->mode == MMIO_UPDATE_GGTT_MODE_REPLICATE &&
		   is_pte_replicable(iov, pte)) {
		buffer->num_copies++;
	} else if (buffer->mode == MMIO_UPDATE_GGTT_MODE_DUPLICATE_LAST &&
		   is_pte_duplicatable(iov, pte)) {
		buffer->num_copies++;
	} else if (buffer->mode == MMIO_UPDATE_GGTT_MODE_REPLICATE_LAST &&
		 is_pte_replicable(iov, pte)) {
		buffer->num_copies++;
	/*
	 * If we operate in a mode that is not *_LAST
	 * (according to the ABI below the value of 2), then we
	 * have a chance to add some more PTEs to our request before
	 * send.
	 */
	} else if (buffer->mode < 2 && buffer->count != max_ptes) {
		buffer->ptes[buffer->count++] = pte;
	} else {
		goto flush;
	}

	return 0;
flush:
	intel_iov_ggtt_vf_flush_ptes(iov);
	intel_iov_ggtt_vf_update_pte(iov, offset, pte);
	return 0;
}

int intel_iov_ggtt_vf_flush_ptes(struct intel_iov *iov)
{
	struct intel_iov_vf_ggtt_ptes *buffer = &iov->vf.ptes_buffer;
	int err;

	GEM_BUG_ON(!intel_iov_is_vf(iov));

	if (!buffer->count)
		return 0;

	intel_iov_query_update_ggtt_ptes(iov);
	buffer->count = 0;

	return err;
}
