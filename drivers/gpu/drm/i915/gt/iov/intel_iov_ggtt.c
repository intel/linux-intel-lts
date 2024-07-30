// SPDX-License-Identifier: MIT
/*
 * Copyright © 2023 Intel Corporation
 */

#include "abi/iov_actions_mmio_abi.h"
#include "intel_iov_ggtt.h"
#include "intel_iov_types.h"
#include "intel_iov_query.h"
#include "intel_iov_utils.h"

static gen8_pte_t prepare_pattern_pte(gen8_pte_t source_pte, u16 vfid)
{
	return (source_pte & MTL_GGTT_PTE_PAT_MASK) | i915_ggtt_prepare_vf_pte(vfid);
}

static struct scatterlist *
sg_add_addr(struct sg_table *st, struct scatterlist *sg, dma_addr_t addr)
{
	if (!sg)
		sg = st->sgl;

	st->nents++;
	sg_set_page(sg, NULL, I915_GTT_PAGE_SIZE, 0);
	sg_dma_address(sg) = addr;
	sg_dma_len(sg) = I915_GTT_PAGE_SIZE;
	return sg_next(sg);
}

static struct scatterlist *
sg_add_ptes(struct sg_table *st, struct scatterlist *sg, gen8_pte_t source_pte, u16 count,
	    bool duplicated)
{
	dma_addr_t pfn = FIELD_GET(GEN12_GGTT_PTE_ADDR_MASK, source_pte);

	while (count--)
		if (duplicated)
			sg = sg_add_addr(st, sg, pfn << PAGE_SHIFT);
		else
			sg = sg_add_addr(st, sg, pfn++ << PAGE_SHIFT);

	return sg;
}

static struct scatterlist *
sg_add_pte(struct sg_table *st, struct scatterlist *sg, gen8_pte_t source_pte)
{
	return sg_add_ptes(st, sg, source_pte, 1, false);
}

int intel_iov_ggtt_pf_update_vf_ptes(struct intel_iov *iov, u32 vfid, u32 pte_offset, u8 mode,
				     u16 num_copies, gen8_pte_t *ptes, u16 count)
{
	struct drm_mm_node *node = &iov->pf.provisioning.configs[vfid].ggtt_region;
	u64 ggtt_addr = node->start + pte_offset * I915_GTT_PAGE_SIZE_4K;
	u64 ggtt_addr_end = ggtt_addr + count * I915_GTT_PAGE_SIZE_4K - 1;
	u64 vf_ggtt_end = node->start + node->size - 1;
	gen8_pte_t pte_pattern = prepare_pattern_pte(*(ptes), vfid);
	struct sg_table *st;
	struct scatterlist *sg;
	bool is_duplicated;
	u16 n_ptes;
	int err;
	int i;

	GEM_BUG_ON(!intel_iov_is_pf(iov));
	/* XXX: All PTEs must have the same flags */
	for (i = 0; i < count; i++)
		GEM_BUG_ON(prepare_pattern_pte(ptes[i], vfid) != pte_pattern);

	if (!count)
		return -EINVAL;

	if (ggtt_addr_end > vf_ggtt_end)
		return -ERANGE;

	n_ptes = num_copies ? num_copies + count : count;

	st = kmalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	if (sg_alloc_table(st, n_ptes, GFP_KERNEL)) {
		kfree(st);
		return -ENOMEM;
	}

	sg = st->sgl;
	st->nents = 0;

	/*
	 * To simplify the code, always let at least one PTE be updated by
	 * a function to duplicate or replicate.
	 */
	num_copies++;
	count--;

	is_duplicated = mode == MMIO_UPDATE_GGTT_MODE_DUPLICATE ||
			mode == MMIO_UPDATE_GGTT_MODE_DUPLICATE_LAST;

	switch (mode) {
	case MMIO_UPDATE_GGTT_MODE_DUPLICATE:
	case MMIO_UPDATE_GGTT_MODE_REPLICATE:
		sg = sg_add_ptes(st, sg, *(ptes++), num_copies, is_duplicated);

		while (count--)
			sg = sg_add_pte(st, sg, *(ptes++));
		break;
	case MMIO_UPDATE_GGTT_MODE_DUPLICATE_LAST:
	case MMIO_UPDATE_GGTT_MODE_REPLICATE_LAST:
		while (count--)
			sg = sg_add_pte(st, sg, *(ptes++));

		sg = sg_add_ptes(st, sg, *(ptes++), num_copies, is_duplicated);
		break;
	default:
		err = -EINVAL;
		goto cleanup;
	}

	err = i915_ggtt_sgtable_update_ptes(iov_to_gt(iov)->ggtt, ggtt_addr, st, n_ptes,
					    pte_pattern);
cleanup:
	sg_free_table(st);
	kfree(st);
	if (err < 0)
		return err;

	IOV_DEBUG(iov, "PF updated GGTT for %d PTE(s) from VF%u\n", n_ptes, vfid);
	return n_ptes;
}

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

void intel_iov_ggtt_vf_update_pte(struct intel_iov *iov, u32 offset, gen8_pte_t pte)
{
	struct intel_iov_vf_ggtt_ptes *buffer = &iov->vf.ptes_buffer;
	u16 max_copies = FIELD_MAX(VF2PF_MMIO_UPDATE_GGTT_REQUEST_MSG_1_NUM_COPIES);
	u16 max_ptes = MMIO_UPDATE_GGTT_MAX_PTES;
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
		buffer->ptes[0] = pte;
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

	return;
flush:
	intel_iov_ggtt_vf_flush_ptes(iov);
	intel_iov_ggtt_vf_update_pte(iov, offset, pte);
}

void intel_iov_ggtt_vf_flush_ptes(struct intel_iov *iov)
{
	struct intel_iov_vf_ggtt_ptes *buffer = &iov->vf.ptes_buffer;

	GEM_BUG_ON(!intel_iov_is_vf(iov));

	if (!buffer->count)
		return;

	intel_iov_query_update_ggtt_ptes(iov);
	buffer->count = 0;
}
