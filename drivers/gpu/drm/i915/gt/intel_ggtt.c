// SPDX-License-Identifier: MIT
/*
 * Copyright © 2020 Intel Corporation
 */

#include <linux/pgtable.h>
#include <asm/set_memory.h>
#include <asm/smp.h>
#include <linux/types.h>
#include <linux/stop_machine.h>

#include <drm/i915_drm.h>
#include <drm/intel-gtt.h>

#include "gem/i915_gem_lmem.h"

#include "iov/abi/iov_actions_prelim_abi.h"
#include "iov/intel_iov.h"
#include "iov/intel_iov_relay.h"
#include "iov/intel_iov_utils.h"

#include "intel_ggtt_gmch.h"
#include "intel_gt.h"
#include "intel_gt_regs.h"
#include "intel_pci_config.h"
#include "i915_drv.h"
#include "i915_pci.h"
#include "i915_scatterlist.h"
#include "i915_utils.h"
#include "i915_vgpu.h"
#include "gen8_ppgtt.h"

#include "intel_flat_ppgtt_pool.h"
#include "intel_gpu_commands.h"
#include "intel_gt.h"
#include "intel_gtt.h"
#include "intel_gt_pm.h"
#include "intel_ring.h"

static int
i915_get_ggtt_vma_pages(struct i915_vma *vma);

static intel_wakeref_t l4wa_pm_get(struct intel_gt *gt)
{
	if (!i915_is_level4_wa_active(gt))
		return 0;

	return intel_gt_pm_get_if_awake_l4(gt);
}

static void i915_ggtt_color_adjust(const struct drm_mm_node *node,
				   unsigned long color,
				   u64 *start,
				   u64 *end)
{
	if (i915_node_color_differs(node, color))
		*start += I915_GTT_PAGE_SIZE;

	/*
	 * Also leave a space between the unallocated reserved node after the
	 * GTT and any objects within the GTT, i.e. we use the color adjustment
	 * to insert a guard page to prevent prefetches crossing over the
	 * GTT boundary.
	 */
	node = list_next_entry(node, node_list);
	if (node->color != color)
		*end -= I915_GTT_PAGE_SIZE;
}

static int ggtt_init_hw(struct i915_ggtt *ggtt)
{
	struct drm_i915_private *i915 = ggtt->vm.i915;

	ggtt->vm.is_ggtt = true;

	/* Only VLV supports read-only GGTT mappings */
	ggtt->vm.has_read_only = IS_VALLEYVIEW(i915);

	if (!HAS_LLC(i915) && !HAS_PPGTT(i915))
		ggtt->vm.mm.color_adjust = i915_ggtt_color_adjust;

	if (ggtt->mappable_end) {
		if (!io_mapping_init_wc(&ggtt->iomap,
					ggtt->gmadr.start,
					ggtt->mappable_end)) {
			ggtt->vm.cleanup(&ggtt->vm);
			return -EIO;
		}

		ggtt->mtrr = arch_phys_wc_add(ggtt->gmadr.start,
					      ggtt->mappable_end);
	}

	intel_ggtt_init_fences(ggtt);

	return 0;
}

/**
 * i915_ggtt_init_hw - Initialize GGTT hardware
 * @i915: i915 device
 */
int i915_ggtt_init_hw(struct drm_i915_private *i915)
{
	struct intel_gt *gt;
	unsigned int i;
	int ret;

	for_each_gt(gt, i915, i) {
		/*
		 * Media GT shares primary GT's GGTT which is already
		 * initialized
		 */
		if (gt->type == GT_MEDIA) {
			drm_WARN_ON(&i915->drm, gt->ggtt != to_gt(i915)->ggtt);
			continue;
		}
		/*
		 * Note that we use page colouring to enforce a guard page at
		 * the end of the address space. This is required as the CS may
		 * prefetch beyond the end of the batch buffer, across the page
		 * boundary, and beyond the end of the GTT if we do not provide
		 * a guard.
		 */
		ret = ggtt_init_hw(gt->ggtt);
		if (ret)
			return ret;
	}

	return 0;
}

/**
 * i915_ggtt_suspend_vm - Suspend the memory mappings for a GGTT or DPT VM
 * @vm: The VM to suspend the mappings for
 *
 * Suspend the memory mappings for all objects mapped to HW via the GGTT or a
 * DPT page table.
 */
void i915_ggtt_suspend_vm(struct i915_address_space *vm)
{
	struct i915_vma *vma, *vn;
	int open;

	drm_WARN_ON(&vm->i915->drm, !vm->is_ggtt && !vm->is_dpt);

	mutex_lock(&vm->mutex);

	/* Skip rewriting PTE on VMA unbind. */
	open = atomic_xchg(&vm->open, 0);

	list_for_each_entry_safe(vma, vn, &vm->bound_list, vm_link) {
		GEM_BUG_ON(!drm_mm_node_allocated(&vma->node));
		i915_vma_wait_for_bind(vma);

		if (i915_vma_is_pinned(vma))
			continue;

		if (!i915_vma_is_bound(vma, I915_VMA_GLOBAL_BIND)) {
			__i915_vma_evict(vma);
			drm_mm_remove_node(&vma->node);
		}
	}

	vm->clear_range(vm, 0, vm->total);

	atomic_set(&vm->open, open);

	mutex_unlock(&vm->mutex);
}

void i915_ggtt_suspend(struct i915_ggtt *ggtt)
{
	struct intel_gt *gt;

	i915_ggtt_suspend_vm(&ggtt->vm);
	ggtt->invalidate(ggtt);

	list_for_each_entry(gt, &ggtt->gt_list, ggtt_link)
		intel_gt_check_and_clear_faults(gt);
}

void gen6_ggtt_invalidate(struct i915_ggtt *ggtt)
{
	struct intel_uncore *uncore = ggtt->vm.gt->uncore;

	spin_lock_irq(&uncore->lock);
	intel_uncore_write_fw(uncore, GFX_FLSH_CNTL_GEN6, GFX_FLSH_CNTL_EN);
	intel_uncore_read_fw(uncore, GFX_FLSH_CNTL_GEN6);
	spin_unlock_irq(&uncore->lock);
}

static void gen8_ggtt_invalidate(struct i915_ggtt *ggtt)
{
	struct intel_uncore *uncore;
	struct intel_gt *gt;

	list_for_each_entry(gt, &ggtt->gt_list, ggtt_link) {
		uncore = gt->uncore;
		/*
		 * Note that as an uncached mmio write, this will flush the
		 * WCB of the writes into the GGTT before it triggers the invalidate.
		 */
		intel_uncore_write_fw(uncore, GFX_FLSH_CNTL_GEN6, GFX_FLSH_CNTL_EN);
	}
}

static void guc_ggtt_ct_invalidate(struct i915_ggtt *ggtt)
{
	struct intel_gt *gt = ggtt->vm.gt;
	struct intel_uncore *uncore = gt->uncore;
	intel_wakeref_t wakeref;

	with_intel_runtime_pm_if_active(uncore->rpm, wakeref) {
		struct intel_guc *guc = &gt->uc.guc;
		int err = -ENODEV;

		if (guc->ct.enabled)
			err = intel_guc_invalidate_tlb_guc(guc, INTEL_GUC_TLB_INVAL_MODE_HEAVY);

		if (err) {
			intel_uncore_write_fw(uncore, PVC_GUC_TLB_INV_DESC1,
					      PVC_GUC_TLB_INV_DESC1_INVALIDATE);
			intel_uncore_write_fw(uncore, PVC_GUC_TLB_INV_DESC0,
					      PVC_GUC_TLB_INV_DESC0_VALID);
		}
	}
}

static void guc_ggtt_invalidate(struct i915_ggtt *ggtt)
{
	struct drm_i915_private *i915 = ggtt->vm.i915;

	gen8_ggtt_invalidate(ggtt);

	if (HAS_ASID_TLB_INVALIDATION(i915)) {
		guc_ggtt_ct_invalidate(ggtt);
	} else if (GRAPHICS_VER(i915) >= 12) {
		struct intel_gt *gt;

		list_for_each_entry(gt, &ggtt->gt_list, ggtt_link)
			intel_uncore_write_fw(gt->uncore,
					      GEN12_GUC_TLB_INV_CR,
					      GEN12_GUC_TLB_INV_CR_INVALIDATE);
	} else {
		intel_uncore_write_fw(ggtt->vm.gt->uncore,
				      GEN8_GTCR, GEN8_GTCR_INVALIDATE);
	}
}

static void gen12vf_ggtt_invalidate(struct i915_ggtt *ggtt)
{
	struct intel_gt *gt;

	list_for_each_entry(gt, &ggtt->gt_list, ggtt_link) {
		struct intel_guc *guc = &gt->uc.guc;
		intel_wakeref_t wakeref;

		if (!guc->ct.enabled)
			continue;
		with_intel_runtime_pm(gt->uncore->rpm, wakeref)
			intel_guc_invalidate_tlb_guc(guc, INTEL_GUC_TLB_INVAL_MODE_HEAVY);
	}
}

u64 mtl_ggtt_pte_encode(dma_addr_t addr,
			unsigned int pat_index,
			u32 flags)
{
	gen8_pte_t pte = addr | GEN8_PAGE_PRESENT;

	GEM_BUG_ON(addr & ~GEN12_GGTT_PTE_ADDR_MASK);

	if (flags & PTE_LM)
		pte |= GEN12_GGTT_PTE_LM;

	if (pat_index & 1)
		pte |= MTL_GGTT_PTE_PAT0;

	if ((pat_index >> 1) & 1)
		pte |= MTL_GGTT_PTE_PAT1;

	return pte;
}

u64 gen8_ggtt_pte_encode(dma_addr_t addr,
			 unsigned int pat_index,
			 u32 flags)
{
	gen8_pte_t pte = addr | GEN8_PAGE_PRESENT;

	GEM_BUG_ON(addr & ~GEN12_GGTT_PTE_ADDR_MASK);

	if (flags & PTE_LM)
		pte |= GEN12_GGTT_PTE_LM;

	return pte;
}

void gen8_set_pte(void __iomem *addr, gen8_pte_t pte)
{
	writeq(pte, addr);
}

gen8_pte_t gen8_get_pte(void __iomem *addr)
{
	return readq(addr);
}

u64 ggtt_addr_to_pte_offset(u64 ggtt_addr)
{
	GEM_BUG_ON(!IS_ALIGNED(ggtt_addr, I915_GTT_PAGE_SIZE_4K));

	return (ggtt_addr / I915_GTT_PAGE_SIZE_4K) * sizeof(gen8_pte_t);
}

static void gen8_ggtt_insert_page(struct i915_address_space *vm,
				  dma_addr_t addr,
				  u64 offset,
				  unsigned int pat_index,
				  u32 flags)
{
	struct i915_ggtt *ggtt = i915_vm_to_ggtt(vm);
	gen8_pte_t __iomem *pte =
		(gen8_pte_t __iomem *)ggtt->gsm + offset / I915_GTT_PAGE_SIZE;

	gen8_set_pte(pte, ggtt->vm.pte_encode(addr, pat_index, flags));

	ggtt->invalidate(ggtt);
}

static void gen8_ggtt_insert_page_wa(struct i915_address_space *vm,
				     dma_addr_t addr,
				     u64 offset,
				     unsigned int pat_index,
				     u32 flags)
{
	GEM_WARN_ON(i915_gem_idle_engines(vm->i915));

	gen8_ggtt_insert_page(vm, addr, offset, pat_index, flags);

	GEM_WARN_ON(i915_gem_resume_engines(vm->i915));
}

void __gen8_ggtt_insert_page_wa_bcs(struct i915_ggtt *ggtt, u32 vfid,
				    dma_addr_t addr, u64 offset,
				    unsigned int pat_index, u32 flags)
{
	struct intel_gt *gt = ggtt->vm.gt;
	struct i915_request *rq;
	u64 pte_encode;
	u32 *cs;

	/* inserts a page with addr at offset in GGTT */
	rq = intel_flat_ppgtt_get_request(&gt->fpp);
	if (IS_ERR(rq)) {
		drm_info(&gt->i915->drm, "Fallback to legacy way ggtt update\n");
		goto err;
	}

	cs = intel_ring_begin(rq, 4);
	if (IS_ERR(cs)) {
		drm_err(&gt->i915->drm, "Cannot reserve space on ring %ld\n", PTR_ERR(cs));
		i915_request_set_error_once(rq, PTR_ERR(cs));
		goto err_rq;
	}

	pte_encode = ggtt->vm.pte_encode(addr, pat_index, flags) |
		     FIELD_PREP(XEHPSDV_GGTT_PTE_VFID_MASK, vfid);

	*cs++ = MI_UPDATE_GTT | 2;
	*cs++ = offset;
	*cs++ = lower_32_bits(pte_encode);
	*cs++ = upper_32_bits(pte_encode);

	intel_ring_advance(rq, cs);

	i915_request_get(rq);
	i915_request_add(rq);

	__i915_request_wait(rq, 0, MAX_SCHEDULE_TIMEOUT);

	i915_request_put(rq);
	ggtt->invalidate(ggtt);
	return;

err_rq:
	i915_request_add(rq);
err:
	gen8_ggtt_insert_page(&ggtt->vm, addr, offset, pat_index, flags);
}

static void gen8_ggtt_insert_page_wa_bcs(struct i915_address_space *vm,
					 dma_addr_t addr, u64 offset,
					 unsigned int pat_index, u32 flags)
{
	struct intel_gt *gt = vm->gt;
	struct i915_ggtt *ggtt = i915_vm_to_ggtt(vm);
	intel_wakeref_t wakeref;

	wakeref = l4wa_pm_get(gt);
	if (!wakeref) {
		gen8_ggtt_insert_page(vm, addr, offset, pat_index, flags);
		return;
	}

	__gen8_ggtt_insert_page_wa_bcs(ggtt, 0, addr, offset, pat_index, flags);

	intel_gt_pm_put_async_l4(gt, wakeref);
}

static void gen8_ggtt_insert_entries(struct i915_address_space *vm,
				     struct i915_vm_pt_stash *stash,
				     struct i915_vma *vma,
				     unsigned int pat_index,
				     u32 flags)
{
	gen8_pte_t pte_encode;
	struct i915_ggtt *ggtt = i915_vm_to_ggtt(vm);
	gen8_pte_t __iomem *gte;
	gen8_pte_t __iomem *end;
	struct sgt_iter iter;
	dma_addr_t addr;

	pte_encode = ggtt->vm.pte_encode(0, pat_index, flags);

	/*
	 * Note that we ignore PTE_READ_ONLY here. The caller must be careful
	 * not to allow the user to override access to a read only page.
	 */

	gte = (gen8_pte_t __iomem *)ggtt->gsm;
	gte += vma->node.start / I915_GTT_PAGE_SIZE;

	end = gte + vma->guard / I915_GTT_PAGE_SIZE;
	while (gte < end)
		gen8_set_pte(gte++, i915_vm_ggtt_scratch0_encode(vm));

	end += (vma->node.size - vma->guard) / I915_GTT_PAGE_SIZE;
	for_each_sgt_daddr(addr, iter, vma->pages)
		gen8_set_pte(gte++, pte_encode | addr);
	GEM_BUG_ON(gte > end);

	/* Fill the allocated but "unused" space beyond the end of the buffer */
	while (gte < end)
		gen8_set_pte(gte++, i915_vm_ggtt_scratch0_encode(vm));

	/*
	 * We want to flush the TLBs only after we're certain all the PTE
	 * updates have finished.
	 */
	ggtt->invalidate(ggtt);
}

static void gen8_ggtt_insert_entries_wa(struct i915_address_space *vm,
					struct i915_vm_pt_stash *stash,
					struct i915_vma *vma,
					unsigned int pat_index,
					u32 flags)
{
	GEM_WARN_ON(i915_gem_idle_engines(vm->i915));

	gen8_ggtt_insert_entries(vm, stash, vma, pat_index, flags);

	GEM_WARN_ON(i915_gem_resume_engines(vm->i915));
}

static void gen8_ggtt_insert_entries_wa_bcs(struct i915_address_space *vm,
					    struct i915_vm_pt_stash *stash,
					    struct i915_vma *vma,
					    unsigned int pat_index,
					    u32 flags)
{
	gen8_pte_t pte_encode;
	struct i915_ggtt *ggtt = i915_vm_to_ggtt(vm);
	struct intel_gt *gt = vm->gt;
	unsigned int num_entries, max_entries;
	struct i915_request *wait = NULL;
	intel_wakeref_t wakeref;
	unsigned int count = 0;
	struct sgt_iter iter;
	u64 start, guard;
	dma_addr_t addr;
	u32 *cs;

	pte_encode = ggtt->vm.pte_encode(0, pat_index, flags);

	wakeref = l4wa_pm_get(gt);
	if (!wakeref) {
		gen8_ggtt_insert_entries(vm, stash, vma, pat_index, flags);
		return;
	}

	max_entries = vma->node.size / I915_GTT_PAGE_SIZE;
	start = vma->node.start;

	guard = start + vma->guard;
	while (start < guard) {
		struct i915_request *rq;

		num_entries = min_t(u32, 128, guard - start);
		max_entries -= num_entries;

		rq = intel_flat_ppgtt_get_request(&gt->fpp);
		if (IS_ERR(rq)) {
			drm_info(&gt->i915->drm, "Fallback to legacy way to update ggtt\n");
			goto flat_err;
		}

		cs = intel_ring_begin(rq, 2 * num_entries + 2);
		if (IS_ERR(cs)) {
			drm_err(&gt->i915->drm, "Cannot reserve space on ring %ld\n", PTR_ERR(cs));
			i915_request_set_error_once(rq, PTR_ERR(cs));
			i915_request_add(rq);
			goto flat_err;
		}
		*cs++ = MI_UPDATE_GTT | (2 * num_entries);
		*cs++ = start;
		memset64((u64 *)cs, i915_vm_ggtt_scratch0_encode(vm), num_entries);
		cs += num_entries * 2;
		intel_ring_advance(rq, cs);

		start += I915_GTT_PAGE_SIZE * num_entries;

		i915_request_put(wait);
		wait = i915_request_get(rq);
		i915_request_add(rq);
	}

	iter = __sgt_iter(vma->pages->sgl, true);
	do {
		struct i915_request *rq;

		count = 0;
		num_entries = min_t(u32, 128, max_entries);
		/*
		 * Note that we ignore PTE_READ_ONLY here. The caller must be careful
		 * not to allow the user to override access to a read only page.
		 */
		rq = intel_flat_ppgtt_get_request(&gt->fpp);
		if (IS_ERR(rq)) {
			drm_info(&gt->i915->drm, "Fallback to legacy way to update ggtt\n");
			goto flat_err;
		}

		cs = intel_ring_begin(rq, (2 * num_entries) + 2);
		if (IS_ERR(cs)) {
			drm_err(&gt->i915->drm, "Cannot reserve space on ring %ld\n", PTR_ERR(cs));
			i915_request_set_error_once(rq, PTR_ERR(cs));
			i915_request_add(rq);
			goto flat_err;
		}
		*cs++ = MI_UPDATE_GTT | (2 * num_entries);
		*cs++ = start;

		for_each_daddr(addr, iter) {
			if (count == num_entries)
				break;
			*cs++ = lower_32_bits(pte_encode | addr);
			*cs++ = upper_32_bits(pte_encode | addr);
			count++;
		}

		for (; count < num_entries; count++) {
			*cs++ = lower_32_bits(i915_vm_ggtt_scratch0_encode(vm));
			*cs++ = upper_32_bits(i915_vm_ggtt_scratch0_encode(vm));
		}
		intel_ring_advance(rq, cs);

		i915_request_put(wait);
		wait = i915_request_get(rq);
		i915_request_add(rq);

		max_entries -= num_entries;
		start += (I915_GTT_PAGE_SIZE * num_entries);
	} while (max_entries);

	__i915_request_wait(wait, 0, MAX_SCHEDULE_TIMEOUT);
	i915_request_put(wait);
	intel_gt_pm_put_async_l4(gt, wakeref);
	/*
	 * We want to flush the TLBs only after we're certain all the PTE
	 * updates have finished.
	 */
	ggtt->invalidate(ggtt);
	return;

flat_err:
	if (wait) {
		__i915_request_wait(wait, 0, MAX_SCHEDULE_TIMEOUT);
		i915_request_put(wait);
	}
	intel_gt_pm_put_async_l4(gt, wakeref);
	gen8_ggtt_insert_entries(vm, stash, vma, pat_index, flags);
}

static void gen12_vf_ggtt_insert_page_wa_vfpf(struct i915_address_space *vm,
					      dma_addr_t addr,
					      u64 offset,
					      unsigned int pat_index,
					      u32 flags)
{
	struct intel_gt *gt = vm->gt;
	u32 request[VF2PF_PF_L4_WA_UPDATE_GGTT_REQUEST_MSG_LEN];
	u32 response[VF2PF_PF_L4_WA_UPDATE_GGTT_RESPONSE_MSG_LEN];

	GEM_BUG_ON(!intel_iov_is_vf(&gt->iov));

	if (!unlikely(i915_is_level4_wa_active(gt))) {
		gen8_ggtt_insert_page(vm, addr, offset, pat_index, flags);
		return;
	}

	request[0] = FIELD_PREP(GUC_HXG_MSG_0_ORIGIN, GUC_HXG_ORIGIN_HOST) |
		     FIELD_PREP(GUC_HXG_MSG_0_TYPE, GUC_HXG_TYPE_REQUEST) |
		     FIELD_PREP(GUC_HXG_REQUEST_MSG_0_ACTION,
				IOV_ACTION_VF2PF_PF_L4_WA_UPDATE_GGTT) |
		     FIELD_PREP(VF2PF_PF_L4_WA_UPDATE_GGTT_REQUEST_MSG_0_MBZ, 0);
	request[1] = FIELD_PREP(VF2PF_PF_L4_WA_UPDATE_GGTT_REQUEST_MSG_1_OFFSET_LO,
				lower_32_bits(offset));
	request[2] = FIELD_PREP(VF2PF_PF_L4_WA_UPDATE_GGTT_REQUEST_MSG_2_OFFSET_HI,
				upper_32_bits(offset));
	request[3] = FIELD_PREP(VF2PF_PF_L4_WA_UPDATE_GGTT_REQUEST_MSG_3_PAT_INDEX, pat_index);
	request[4] = FIELD_PREP(VF2PF_PF_L4_WA_UPDATE_GGTT_REQUEST_MSG_4_PTE_FLAGS, flags);
	request[5] = FIELD_PREP(VF2PF_PF_L4_WA_UPDATE_GGTT_REQUEST_MSG_5_ADDR_LO,
				lower_32_bits(addr));
	request[6] = FIELD_PREP(VF2PF_PF_L4_WA_UPDATE_GGTT_REQUEST_MSG_6_ADDR_HI,
				upper_32_bits(addr));

	intel_iov_relay_send_to_pf(&gt->iov.relay,
				   request, ARRAY_SIZE(request),
				   response, ARRAY_SIZE(response));
}

static void gen12_vf_ggtt_insert_entries_wa_vfpf(struct i915_address_space *vm,
						 struct i915_vm_pt_stash *stash,
						 struct i915_vma *vma,
						 unsigned int pat_index,
						 u32 flags)
{
	struct intel_gt *gt = vm->gt;
	struct i915_ggtt *ggtt = i915_vm_to_ggtt(vm);
	struct sgt_iter iter;
	dma_addr_t addr;
	u64 offset;

	GEM_BUG_ON(!intel_iov_is_vf(&gt->iov));

	if (!unlikely(i915_is_level4_wa_active(gt))) {
		gen8_ggtt_insert_entries(vm, stash, vma, pat_index, flags);
		return;
	}

	offset = i915_vma_offset(vma);
	for_each_sgt_daddr(addr, iter, vma->pages) {
		gen12_vf_ggtt_insert_page_wa_vfpf(vm, addr, offset, pat_index, flags);
		offset += I915_GTT_PAGE_SIZE;
	}

	ggtt->invalidate(ggtt);
}

static void gen6_ggtt_insert_page(struct i915_address_space *vm,
				  dma_addr_t addr,
				  u64 offset,
				  unsigned int pat_index,
				  u32 flags)
{
	struct i915_ggtt *ggtt = i915_vm_to_ggtt(vm);
	gen6_pte_t __iomem *pte =
		(gen6_pte_t __iomem *)ggtt->gsm + offset / I915_GTT_PAGE_SIZE;

	iowrite32(vm->pte_encode(addr, pat_index, flags), pte);

	ggtt->invalidate(ggtt);
}

/*
 * Binds an object into the global gtt with the specified cache level.
 * The object will be accessible to the GPU via commands whose operands
 * reference offsets within the global GTT as well as accessible by the GPU
 * through the GMADR mapped BAR (i915->mm.gtt->gtt).
 */
static void gen6_ggtt_insert_entries(struct i915_address_space *vm,
				     struct i915_vm_pt_stash *stash,
				     struct i915_vma *vma,
				     unsigned int pat_index,
				     u32 flags)
{
	struct i915_ggtt *ggtt = i915_vm_to_ggtt(vm);
	gen6_pte_t __iomem *gte;
	gen6_pte_t __iomem *end;
	struct sgt_iter iter;
	dma_addr_t addr;

	gte = (gen6_pte_t __iomem *)ggtt->gsm;
	gte += vma->node.start / I915_GTT_PAGE_SIZE;

	end = gte + vma->guard / I915_GTT_PAGE_SIZE;
	while (gte < end)
		gen8_set_pte(gte++, i915_vm_ggtt_scratch0_encode(vm));

	end += (vma->node.size - vma->guard) / I915_GTT_PAGE_SIZE;
	for_each_sgt_daddr(addr, iter, vma->pages)
		iowrite32(vm->pte_encode(addr, pat_index, flags), gte++);
	GEM_BUG_ON(gte > end);

	/* Fill the allocated but "unused" space beyond the end of the buffer */
	while (gte < end)
		iowrite32(i915_vm_ggtt_scratch0_encode(vm), gte++);

	/*
	 * We want to flush the TLBs only after we're certain all the PTE
	 * updates have finished.
	 */
	ggtt->invalidate(ggtt);
}

static void gen8_ggtt_clear_range(struct i915_address_space *vm,
				  u64 start, u64 length)
{
	struct i915_ggtt *ggtt = i915_vm_to_ggtt(vm);
	unsigned long first_entry = start / I915_GTT_PAGE_SIZE;
	unsigned long num_entries = length / I915_GTT_PAGE_SIZE;
	gen8_pte_t __iomem *pte =
		(gen8_pte_t __iomem *)ggtt->gsm + first_entry;
	const gen8_pte_t scratch = i915_vm_ggtt_scratch0_encode(vm);

	while (num_entries--)
		iowrite32(scratch, pte++);
}

static void nop_clear_range(struct i915_address_space *vm,
			    u64 start, u64 length)
{
}

static void bxt_vtd_ggtt_wa(struct i915_address_space *vm)
{
	/*
	 * Make sure the internal GAM fifo has been cleared of all GTT
	 * writes before exiting stop_machine(). This guarantees that
	 * any aperture accesses waiting to start in another process
	 * cannot back up behind the GTT writes causing a hang.
	 * The register can be any arbitrary GAM register.
	 */
	intel_uncore_posting_read_fw(vm->gt->uncore, GFX_FLSH_CNTL_GEN6);
}

struct insert_page {
	struct i915_address_space *vm;
	dma_addr_t addr;
	u64 offset;
	unsigned int pat_index;
};

static int bxt_vtd_ggtt_insert_page__cb(void *_arg)
{
	struct insert_page *arg = _arg;

	gen8_ggtt_insert_page(arg->vm, arg->addr, arg->offset,
			      arg->pat_index, 0);
	bxt_vtd_ggtt_wa(arg->vm);

	return 0;
}

static void bxt_vtd_ggtt_insert_page__BKL(struct i915_address_space *vm,
					  dma_addr_t addr,
					  u64 offset,
					  unsigned int pat_index,
					  u32 unused)
{
	struct insert_page arg = { vm, addr, offset, pat_index };

	stop_machine(bxt_vtd_ggtt_insert_page__cb, &arg, NULL);
}

struct insert_entries {
	struct i915_address_space *vm;
	struct i915_vm_pt_stash *stash;
	struct i915_vma *vma;
	unsigned int pat_index;
	u32 flags;
};

static int bxt_vtd_ggtt_insert_entries__cb(void *_arg)
{
	struct insert_entries *arg = _arg;

	gen8_ggtt_insert_entries(arg->vm, arg->stash, arg->vma, arg->pat_index, arg->flags);
	bxt_vtd_ggtt_wa(arg->vm);

	return 0;
}

static void bxt_vtd_ggtt_insert_entries__BKL(struct i915_address_space *vm,
					     struct i915_vm_pt_stash *stash,
					     struct i915_vma *vma,
					     unsigned int pat_index,
					     u32 flags)
{
	struct insert_entries arg = { vm, stash, vma, pat_index, flags };

	stop_machine(bxt_vtd_ggtt_insert_entries__cb, &arg, NULL);
}

static void gen6_ggtt_clear_range(struct i915_address_space *vm,
				  u64 start, u64 length)
{
	struct i915_ggtt *ggtt = i915_vm_to_ggtt(vm);
	unsigned int first_entry = start / I915_GTT_PAGE_SIZE;
	unsigned int num_entries = length / I915_GTT_PAGE_SIZE;
	gen6_pte_t scratch_pte, __iomem *gtt_base =
		(gen6_pte_t __iomem *)ggtt->gsm + first_entry;
	const int max_entries = ggtt_total_entries(ggtt) - first_entry;
	int i;

	if (WARN(num_entries > max_entries,
		 "First entry = %d; Num entries = %d (max=%d)\n",
		 first_entry, num_entries, max_entries))
		num_entries = max_entries;

	scratch_pte = i915_vm_ggtt_scratch0_encode(vm);
	for (i = 0; i < num_entries; i++)
		iowrite32(scratch_pte, &gtt_base[i]);
}

void intel_ggtt_bind_vma(struct i915_address_space *vm,
			 struct i915_vm_pt_stash *stash,
			 struct i915_vma *vma,
			 unsigned int pat_index,
			 u32 flags)
{
	struct drm_i915_gem_object *obj = vma->obj;
	u32 pte_flags;

	if (i915_vma_is_bound(vma, ~flags & I915_VMA_BIND_MASK))
		return;

	/* Applicable to VLV (gen8+ do not support RO in the GGTT) */
	pte_flags = 0;
	if (i915_gem_object_is_readonly(obj))
		pte_flags |= PTE_READ_ONLY;
	if (i915_gem_object_is_lmem(obj) || i915_gem_object_has_fabric(obj))
		pte_flags |= PTE_LM;

	vm->insert_entries(vm, stash, vma, pat_index, pte_flags);
	vma->page_sizes = I915_GTT_PAGE_SIZE;
}

static void ggtt_bind_vma_wa(struct i915_address_space *vm,
			     struct i915_vm_pt_stash *stash,
			     struct i915_vma *vma,
			     unsigned int pat_index,
			     u32 flags)
{
	GEM_WARN_ON(i915_gem_idle_engines(vma->vm->i915));

	intel_ggtt_bind_vma(vm, stash, vma, pat_index, flags);

	GEM_WARN_ON(i915_gem_resume_engines(vma->vm->i915));
}

void intel_ggtt_unbind_vma(struct i915_address_space *vm, struct i915_vma *vma)
{
	vm->clear_range(vm, vma->node.start, vma->size);
}

static void ggtt_unbind_vma_wa(struct i915_address_space *vm,
			       struct i915_vma *vma)
{
	GEM_WARN_ON(i915_gem_idle_engines(vma->vm->i915));

	intel_ggtt_unbind_vma(vm, vma);

	GEM_WARN_ON(i915_gem_resume_engines(vma->vm->i915));
}

static int ggtt_reserve_guc_top(struct i915_ggtt *ggtt)
{
	if (!intel_uc_uses_guc(&ggtt->vm.gt->uc))
		return 0;

	GEM_BUG_ON(ggtt->vm.total <= GUC_GGTT_TOP);
	return i915_ggtt_balloon(ggtt, GUC_GGTT_TOP, ggtt->vm.total,
				 &ggtt->uc_fw);
}

static void ggtt_release_guc_top(struct i915_ggtt *ggtt)
{
	i915_ggtt_deballoon(ggtt, &ggtt->uc_fw);
}

static void cleanup_init_ggtt(struct i915_ggtt *ggtt)
{
	ggtt_release_guc_top(ggtt);
	if (drm_mm_node_allocated(&ggtt->error_capture))
		drm_mm_remove_node(&ggtt->error_capture);
	mutex_destroy(&ggtt->error_mutex);
}

static int init_ggtt(struct i915_ggtt *ggtt)
{
	/*
	 * Let GEM Manage all of the aperture.
	 *
	 * However, leave one page at the end still bound to the scratch page.
	 * There are a number of places where the hardware apparently prefetches
	 * past the end of the object, and we've seen multiple hangs with the
	 * GPU head pointer stuck in a batchbuffer bound at the last page of the
	 * aperture.  One page should be enough to keep any prefetching inside
	 * of the aperture.
	 */
	unsigned long hole_start, hole_end;
	struct drm_mm_node *entry;
	int ret;

	/*
	 * GuC requires all resources that we're sharing with it to be placed in
	 * non-WOPCM memory. If GuC is not present or not in use we still need a
	 * small bias as ring wraparound at offset 0 sometimes hangs. No idea
	 * why.
	 */
	ggtt->pin_bias = max_t(u32, I915_GTT_PAGE_SIZE,
			       intel_wopcm_guc_size(&ggtt->vm.gt->wopcm));

	ret = intel_vgt_balloon(ggtt);
	if (ret)
		return ret;

	ret = intel_iov_init_ggtt(&ggtt->vm.gt->iov);
	if (ret)
		return ret;

	mutex_init(&ggtt->error_mutex);
	if (ggtt->mappable_end) {
		/*
		 * Reserve a mappable slot for our lockless error capture.
		 *
		 * We strongly prefer taking address 0x0 in order to protect
		 * other critical buffers against accidental overwrites,
		 * as writing to address 0 is a very common mistake.
		 *
		 * Since 0 may already be in use by the system (e.g. the BIOS
		 * framebuffer), we let the reservation fail quietly and hope
		 * 0 remains reserved always.
		 *
		 * If we fail to reserve 0, and then fail to find any space
		 * for an error-capture, remain silent. We can afford not
		 * to reserve an error_capture node as we have fallback
		 * paths, and we trust that 0 will remain reserved. However,
		 * the only likely reason for failure to insert is a driver
		 * bug, which we expect to cause other failures...
		 *
		 * Since CPU can perform speculative reads on error capture
		 * (write-combining allows it) add scratch page after error
		 * capture to avoid DMAR errors.
		 */
		ggtt->error_capture.size = 2 * I915_GTT_PAGE_SIZE;
		ggtt->error_capture.color = I915_COLOR_UNEVICTABLE;
		if (drm_mm_reserve_node(&ggtt->vm.mm, &ggtt->error_capture))
			drm_mm_insert_node_in_range(&ggtt->vm.mm,
						    &ggtt->error_capture,
						    ggtt->error_capture.size, 0,
						    ggtt->error_capture.color,
						    0, ggtt->mappable_end,
						    DRM_MM_INSERT_LOW);
	}
	if (drm_mm_node_allocated(&ggtt->error_capture)) {
		u64 start = ggtt->error_capture.start;
		u64 size = ggtt->error_capture.size;

		ggtt->vm.scratch_range(&ggtt->vm, start, size);
		drm_dbg(&ggtt->vm.i915->drm,
			"Reserved GGTT:[%llx, %llx] for use by error capture\n",
			start, start + size);
	}

	/*
	 * The upper portion of the GuC address space has a sizeable hole
	 * (several MB) that is inaccessible by GuC. Reserve this range within
	 * GGTT as it can comfortably hold GuC/HuC firmware images.
	 */
	ret = ggtt_reserve_guc_top(ggtt);
	if (ret)
		goto err;

	/* Clear any non-preallocated blocks */
	drm_mm_for_each_hole(entry, &ggtt->vm.mm, hole_start, hole_end) {
		drm_dbg(&ggtt->vm.i915->drm,
			"clearing unused GTT space: [%lx, %lx]\n",
			hole_start, hole_end);
		ggtt->vm.clear_range(&ggtt->vm, hole_start,
				     hole_end - hole_start);
	}

	/* And finally clear the reserved guard page */
	ggtt->vm.clear_range(&ggtt->vm, ggtt->vm.total - PAGE_SIZE, PAGE_SIZE);

	return 0;

err:
	cleanup_init_ggtt(ggtt);
	return ret;
}

static void aliasing_gtt_bind_vma(struct i915_address_space *vm,
				  struct i915_vm_pt_stash *stash,
				  struct i915_vma *vma,
				  unsigned int pat_index,
				  u32 flags)
{
	u32 pte_flags;

	/* Currently applicable only to VLV */
	pte_flags = 0;
	if (vma->vm->has_read_only && i915_gem_object_is_readonly(vma->obj))
		pte_flags |= PTE_READ_ONLY;

	if (flags & I915_VMA_LOCAL_BIND)
		ppgtt_bind_vma(&i915_vm_to_ggtt(vm)->alias->vm,
			       stash, vma, pat_index, flags);

	if (flags & I915_VMA_GLOBAL_BIND)
		vm->insert_entries(vm, stash, vma, pat_index, pte_flags);
}

static void aliasing_gtt_unbind_vma(struct i915_address_space *vm,
				    struct i915_vma *vma)
{
	if (i915_vma_is_bound(vma, I915_VMA_GLOBAL_BIND))
		vm->clear_range(vm, i915_ggtt_offset(vma), vma->size);

	if (i915_vma_is_bound(vma, I915_VMA_LOCAL_BIND))
		ppgtt_unbind_vma(&i915_vm_to_ggtt(vm)->alias->vm, vma);
}

static int init_aliasing_ppgtt(struct i915_ggtt *ggtt)
{
	struct i915_vm_pt_stash stash = {};
	struct i915_ppgtt *ppgtt;
	int err;

	ppgtt = i915_ppgtt_create(ggtt->vm.gt, 0);
	if (IS_ERR(ppgtt))
		return PTR_ERR(ppgtt);

	if (GEM_WARN_ON(ppgtt->vm.total < ggtt->vm.total)) {
		err = -ENODEV;
		goto err_ppgtt;
	}

	err = i915_vm_alloc_pt_stash(&ppgtt->vm, &stash, ggtt->vm.total);
	if (err)
		goto err_ppgtt;

	i915_gem_object_lock(ppgtt->pd->pt.base, NULL);
	err = i915_vm_map_pt_stash(&ppgtt->vm, &stash);
	i915_gem_object_unlock(ppgtt->pd->pt.base);
	if (err)
		goto err_stash;

	/*
	 * Note we only pre-allocate as far as the end of the global
	 * GTT. On 48b / 4-level page-tables, the difference is very,
	 * very significant! We have to preallocate as GVT/vgpu does
	 * not like the page directory disappearing.
	 */
	ppgtt->vm.allocate_va_range(&ppgtt->vm, &stash, 0, ggtt->vm.total);

	ggtt->alias = ppgtt;
	ggtt->vm.bind_async_flags |= ppgtt->vm.bind_async_flags;

	GEM_BUG_ON(ggtt->vm.vma_ops.bind_vma != intel_ggtt_bind_vma);
	ggtt->vm.vma_ops.bind_vma = aliasing_gtt_bind_vma;

	GEM_BUG_ON(ggtt->vm.vma_ops.unbind_vma != intel_ggtt_unbind_vma);
	ggtt->vm.vma_ops.unbind_vma = aliasing_gtt_unbind_vma;

	i915_vm_free_pt_stash(&ppgtt->vm, &stash);
	return 0;

err_stash:
	i915_vm_free_pt_stash(&ppgtt->vm, &stash);
err_ppgtt:
	i915_vm_put(&ppgtt->vm);
	return err;
}

static void fini_aliasing_ppgtt(struct i915_ggtt *ggtt)
{
	struct i915_ppgtt *ppgtt;

	ppgtt = fetch_and_zero(&ggtt->alias);
	if (!ppgtt)
		return;

	i915_vm_put(&ppgtt->vm);

	ggtt->vm.vma_ops.bind_vma   = intel_ggtt_bind_vma;
	ggtt->vm.vma_ops.unbind_vma = intel_ggtt_unbind_vma;
}

int i915_init_ggtt(struct drm_i915_private *i915)
{
	struct intel_gt *gt;
	unsigned int i, j;
	int ret;

	for_each_gt(gt, i915, i) {
		/*
		 * Media GT shares primary GT's GGTT which is already
		 * initialized
		 */
		if (gt->type == GT_MEDIA) {
			drm_WARN_ON(&i915->drm, gt->ggtt != to_gt(i915)->ggtt);
			continue;
		}
		ret = init_ggtt(gt->ggtt);
		if (ret)
			goto err;
	}

	if (INTEL_PPGTT(i915) == INTEL_PPGTT_ALIASING) {
		ret = init_aliasing_ppgtt(to_gt(i915)->ggtt);
		if (ret)
			goto err;
	}

	return 0;

err:
	for_each_gt(gt, i915, j) {
		if (j == i)
			break;
		cleanup_init_ggtt(gt->ggtt);
	}

	return ret;
}

static void ggtt_cleanup_hw(struct i915_ggtt *ggtt)
{
	struct i915_vma *vma, *vn;

	atomic_set(&ggtt->vm.open, 0);

	rcu_barrier(); /* flush the RCU'ed__i915_vm_release */
	flush_workqueue(ggtt->vm.i915->wq);

	mutex_lock(&ggtt->vm.mutex);

	list_for_each_entry_safe(vma, vn, &ggtt->vm.bound_list, vm_link)
		WARN_ON_ONCE(__i915_vma_unbind(vma));

	if (drm_mm_node_allocated(&ggtt->error_capture))
		drm_mm_remove_node(&ggtt->error_capture);
	mutex_destroy(&ggtt->error_mutex);

	ggtt_release_guc_top(ggtt);
	intel_iov_fini_ggtt(&ggtt->vm.gt->iov);
	intel_vgt_deballoon(ggtt);

	ggtt->vm.cleanup(&ggtt->vm);

	mutex_unlock(&ggtt->vm.mutex);
	i915_address_space_fini(&ggtt->vm);

	arch_phys_wc_del(ggtt->mtrr);

	if (ggtt->iomap.size)
		io_mapping_fini(&ggtt->iomap);
}

/**
 * i915_ggtt_driver_release - Clean up GGTT hardware initialization
 * @i915: i915 device
 */
void i915_ggtt_driver_release(struct drm_i915_private *i915)
{
	struct i915_ggtt *ggtt = to_gt(i915)->ggtt;
	struct intel_gt *gt;
	unsigned int i;

	fini_aliasing_ppgtt(ggtt);

	intel_ggtt_fini_fences(ggtt);

	for_each_gt(gt, i915, i) {
		if (gt->type == GT_MEDIA)
			continue;

		ggtt_cleanup_hw(gt->ggtt);
	}
}

/**
 * i915_ggtt_driver_late_release - Cleanup of GGTT that needs to be done after
 * all free objects have been drained.
 * @i915: i915 device
 */
void i915_ggtt_driver_late_release(struct drm_i915_private *i915)
{
	struct intel_gt *gt;
	unsigned int i;

	for_each_gt(gt, i915, i) {
		struct i915_ggtt *ggtt = gt->ggtt;

		if (gt->type == GT_MEDIA)
			continue;

		kfree(ggtt);
	}
}

static unsigned int gen6_get_total_gtt_size(u16 snb_gmch_ctl)
{
	snb_gmch_ctl >>= SNB_GMCH_GGMS_SHIFT;
	snb_gmch_ctl &= SNB_GMCH_GGMS_MASK;
	return snb_gmch_ctl << 20;
}

static unsigned int gen8_get_total_gtt_size(u16 bdw_gmch_ctl)
{
	bdw_gmch_ctl >>= BDW_GMCH_GGMS_SHIFT;
	bdw_gmch_ctl &= BDW_GMCH_GGMS_MASK;
	if (bdw_gmch_ctl)
		bdw_gmch_ctl = 1 << bdw_gmch_ctl;

#ifdef CONFIG_X86_32
	/* Limit 32b platforms to a 2GB GGTT: 4 << 20 / pte size * I915_GTT_PAGE_SIZE */
	if (bdw_gmch_ctl > 4)
		bdw_gmch_ctl = 4;
#endif

	return bdw_gmch_ctl << 20;
}

static unsigned int chv_get_total_gtt_size(u16 gmch_ctrl)
{
	gmch_ctrl >>= SNB_GMCH_GGMS_SHIFT;
	gmch_ctrl &= SNB_GMCH_GGMS_MASK;

	if (gmch_ctrl)
		return 1 << (20 + gmch_ctrl);

	return 0;
}

static unsigned int gen6_gttmmadr_size(struct drm_i915_private *i915)
{
	/*
	 * GEN6: GTTMMADR size is 4MB and GTTADR starts at 2MB offset
	 * GEN8: GTTMMADR size is 16MB and GTTADR starts at 8MB offset
	 */
	GEM_BUG_ON(GRAPHICS_VER(i915) < 6);
	return (GRAPHICS_VER(i915) < 8) ? SZ_4M : SZ_16M;
}

static unsigned int gen6_gttadr_offset(struct drm_i915_private *i915)
{
	return gen6_gttmmadr_size(i915) / 2;
}

static int ggtt_probe_common(struct i915_ggtt *ggtt, u64 size)
{
	struct drm_i915_private *i915 = ggtt->vm.i915;
	phys_addr_t phys_addr;
	int ret;

	ret = i915_address_space_init(&ggtt->vm, VM_CLASS_GGTT);
	if (ret)
		return ret;

	phys_addr = ggtt->vm.gt->phys_addr + gen6_gttadr_offset(i915);

	/*
	 * On BXT+/ICL+ writes larger than 64 bit to the GTT pagetable range
	 * will be dropped. For WC mappings in general we have 64 byte burst
	 * writes when the WC buffer is flushed, so we can't use it, but have to
	 * resort to an uncached mapping. The WC issue is easily caught by the
	 * readback check when writing GTT PTE entries.
	 */
	if (IS_GEN9_LP(i915) || GRAPHICS_VER(i915) >= 11)
		ggtt->gsm = ioremap(phys_addr, size);
	else
		ggtt->gsm = ioremap_wc(phys_addr, size);
	if (!ggtt->gsm) {
		drm_err(&i915->drm, "Failed to map the ggtt page table\n");
		return -ENOMEM;
	}

	ret = i915_vm_setup_scratch0(&ggtt->vm);
	if (ret) {
		drm_err(&i915->drm, "Scratch setup failed\n");
		/* iounmap will also get called at remove, but meh */
		iounmap(ggtt->gsm);
		return ret;
	}

	if (ggtt->vm.scratch[0] && i915_gem_object_is_lmem(ggtt->vm.scratch[0]))
		/* we rely on scratch in SMEM to clean stale LMEM for the WA */
		GEM_DEBUG_WARN_ON(intel_ggtt_needs_same_mem_type_within_cl_wa(i915));

	return 0;
}

int ggtt_set_pages(struct i915_vma *vma)
{
	int ret;

	GEM_BUG_ON(vma->pages);

	ret = i915_get_ggtt_vma_pages(vma);
	if (ret)
		return ret;

	vma->page_sizes = vma->obj->mm.page_sizes;

	return 0;
}

static void gen6_gmch_remove(struct i915_address_space *vm)
{
	struct i915_ggtt *ggtt = i915_vm_to_ggtt(vm);

	iounmap(ggtt->gsm);
	i915_vm_free_scratch(vm);
}

static struct resource pci_resource(struct pci_dev *pdev, int bar)
{
	return (struct resource)DEFINE_RES_MEM(pci_resource_start(pdev, bar),
					       pci_resource_len(pdev, bar));
}

static int gen8_gmch_probe(struct i915_ggtt *ggtt)
{
	struct drm_i915_private *i915 = ggtt->vm.i915;
	struct pci_dev *pdev = to_pci_dev(i915->drm.dev);
	unsigned int size;
	u16 snb_gmch_ctl;

	if (!HAS_LMEM(i915) && !HAS_BAR2_SMEM_STOLEN(i915)) {
		if (!i915_pci_resource_valid(pdev, GTT_APERTURE_BAR))
			return -ENXIO;

		ggtt->gmadr = pci_resource(pdev, GTT_APERTURE_BAR);
		ggtt->mappable_end = resource_size(&ggtt->gmadr);
	}

	pci_read_config_word(pdev, SNB_GMCH_CTRL, &snb_gmch_ctl);
	if (IS_CHERRYVIEW(i915))
		size = chv_get_total_gtt_size(snb_gmch_ctl);
	else
		size = gen8_get_total_gtt_size(snb_gmch_ctl);

	ggtt->vm.alloc_pt_dma = alloc_pt_dma;
	ggtt->vm.alloc_scratch_dma = alloc_pt_dma;

	ggtt->vm.total = (size / sizeof(gen8_pte_t)) * I915_GTT_PAGE_SIZE;
	ggtt->vm.cleanup = gen6_gmch_remove;
	ggtt->vm.clear_range = nop_clear_range;
	ggtt->vm.scratch_range = gen8_ggtt_clear_range;

	if (i915_is_mem_wa_enabled(i915, I915_WA_USE_FLAT_PPGTT_UPDATE)) {
		ggtt->vm.insert_entries = gen8_ggtt_insert_entries_wa_bcs;
		ggtt->vm.insert_page = gen8_ggtt_insert_page_wa_bcs;
		ggtt->vm.bind_async_flags =
			I915_VMA_GLOBAL_BIND |
			I915_VMA_LOCAL_BIND |
			I915_VMA_ERROR;
	} else if (i915_is_mem_wa_enabled(i915, I915_WA_IDLE_GPU_BEFORE_UPDATE)) {
		ggtt->vm.insert_entries = gen8_ggtt_insert_entries_wa;
		ggtt->vm.insert_page = gen8_ggtt_insert_page_wa;
	} else  {
		ggtt->vm.insert_entries = gen8_ggtt_insert_entries;
		ggtt->vm.insert_page = gen8_ggtt_insert_page;
	}

	/*
	 * Serialize GTT updates with aperture access on BXT if VT-d is on,
	 * and always on CHV.
	 */
	if (intel_vm_no_concurrent_access_wa(i915)) {
		ggtt->vm.insert_entries = bxt_vtd_ggtt_insert_entries__BKL;
		ggtt->vm.insert_page    = bxt_vtd_ggtt_insert_page__BKL;
		ggtt->vm.bind_async_flags =
			I915_VMA_GLOBAL_BIND | I915_VMA_LOCAL_BIND;
	}

	if (intel_uc_wants_guc(&ggtt->vm.gt->uc))
		ggtt->invalidate = guc_ggtt_invalidate;
	else
		ggtt->invalidate = gen8_ggtt_invalidate;

	if (i915_is_mem_wa_enabled(i915, I915_WA_IDLE_GPU_BEFORE_UPDATE)) {
		ggtt->vm.vma_ops.bind_vma = ggtt_bind_vma_wa;
		ggtt->vm.vma_ops.unbind_vma  = ggtt_unbind_vma_wa;
	} else {
		ggtt->vm.vma_ops.bind_vma = intel_ggtt_bind_vma;
		ggtt->vm.vma_ops.unbind_vma  = intel_ggtt_unbind_vma;
	}
	ggtt->vm.vma_ops.set_pages   = ggtt_set_pages;
	ggtt->vm.vma_ops.clear_pages = clear_pages;

	if (GRAPHICS_VER_FULL(i915) >= IP_VER(12, 70))
		ggtt->vm.pte_encode = mtl_ggtt_pte_encode;
	else
		ggtt->vm.pte_encode = gen8_ggtt_pte_encode;

	return ggtt_probe_common(ggtt, size);
}

/*
 * For pre-gen8 platforms pat_index is the same as enum i915_cache_level,
 * so these PTE encode functions are left with using cache_level.
 * See translation table LEGACY_CACHELEVEL.
 */
static u64 snb_pte_encode(dma_addr_t addr,
			  enum i915_cache_level level,
			  u32 flags)
{
	gen6_pte_t pte = GEN6_PTE_ADDR_ENCODE(addr) | GEN6_PTE_VALID;

	switch (level) {
	case I915_CACHE_L3_LLC:
	case I915_CACHE_LLC:
		pte |= GEN6_PTE_CACHE_LLC;
		break;
	case I915_CACHE_NONE:
		pte |= GEN6_PTE_UNCACHED;
		break;
	default:
		MISSING_CASE(level);
	}

	return pte;
}

static u64 ivb_pte_encode(dma_addr_t addr,
			  enum i915_cache_level level,
			  u32 flags)
{
	gen6_pte_t pte = GEN6_PTE_ADDR_ENCODE(addr) | GEN6_PTE_VALID;

	switch (level) {
	case I915_CACHE_L3_LLC:
		pte |= GEN7_PTE_CACHE_L3_LLC;
		break;
	case I915_CACHE_LLC:
		pte |= GEN6_PTE_CACHE_LLC;
		break;
	case I915_CACHE_NONE:
		pte |= GEN6_PTE_UNCACHED;
		break;
	default:
		MISSING_CASE(level);
	}

	return pte;
}

static u64 byt_pte_encode(dma_addr_t addr,
			  enum i915_cache_level level,
			  u32 flags)
{
	gen6_pte_t pte = GEN6_PTE_ADDR_ENCODE(addr) | GEN6_PTE_VALID;

	if (!(flags & PTE_READ_ONLY))
		pte |= BYT_PTE_WRITEABLE;

	if (level != I915_CACHE_NONE)
		pte |= BYT_PTE_SNOOPED_BY_CPU_CACHES;

	return pte;
}

static u64 hsw_pte_encode(dma_addr_t addr,
			  enum i915_cache_level level,
			  u32 flags)
{
	gen6_pte_t pte = HSW_PTE_ADDR_ENCODE(addr) | GEN6_PTE_VALID;

	if (level != I915_CACHE_NONE)
		pte |= HSW_WB_LLC_AGE3;

	return pte;
}

static u64 iris_pte_encode(dma_addr_t addr,
			   enum i915_cache_level level,
			   u32 flags)
{
	gen6_pte_t pte = HSW_PTE_ADDR_ENCODE(addr) | GEN6_PTE_VALID;

	switch (level) {
	case I915_CACHE_NONE:
		break;
	case I915_CACHE_WT:
		pte |= HSW_WT_ELLC_LLC_AGE3;
		break;
	default:
		pte |= HSW_WB_ELLC_LLC_AGE3;
		break;
	}

	return pte;
}

static int gen6_gmch_probe(struct i915_ggtt *ggtt)
{
	struct drm_i915_private *i915 = ggtt->vm.i915;
	struct pci_dev *pdev = to_pci_dev(i915->drm.dev);
	unsigned int size;
	u16 snb_gmch_ctl;

	if (!i915_pci_resource_valid(pdev, GTT_APERTURE_BAR))
		return -ENXIO;

	ggtt->gmadr = pci_resource(pdev, GTT_APERTURE_BAR);
	ggtt->mappable_end = resource_size(&ggtt->gmadr);

	/*
	 * 64/512MB is the current min/max we actually know of, but this is
	 * just a coarse sanity check.
	 */
	if (ggtt->mappable_end < (64<<20) || ggtt->mappable_end > (512<<20)) {
		drm_err(&i915->drm, "Unknown GMADR size (%pa)\n",
			&ggtt->mappable_end);
		return -ENXIO;
	}

	pci_read_config_word(pdev, SNB_GMCH_CTRL, &snb_gmch_ctl);

	size = gen6_get_total_gtt_size(snb_gmch_ctl);
	ggtt->vm.total = (size / sizeof(gen6_pte_t)) * I915_GTT_PAGE_SIZE;

	ggtt->vm.alloc_pt_dma = alloc_pt_dma;
	ggtt->vm.alloc_scratch_dma = alloc_pt_dma;

	ggtt->vm.clear_range = nop_clear_range;
	if (!HAS_FULL_PPGTT(i915))
		ggtt->vm.clear_range = gen6_ggtt_clear_range;
	ggtt->vm.scratch_range = gen6_ggtt_clear_range;
	ggtt->vm.insert_page = gen6_ggtt_insert_page;
	ggtt->vm.insert_entries = gen6_ggtt_insert_entries;
	ggtt->vm.cleanup = gen6_gmch_remove;

	ggtt->invalidate = gen6_ggtt_invalidate;

	if (HAS_EDRAM(i915))
		ggtt->vm.pte_encode = iris_pte_encode;
	else if (IS_HASWELL(i915))
		ggtt->vm.pte_encode = hsw_pte_encode;
	else if (IS_VALLEYVIEW(i915))
		ggtt->vm.pte_encode = byt_pte_encode;
	else if (GRAPHICS_VER(i915) >= 7)
		ggtt->vm.pte_encode = ivb_pte_encode;
	else
		ggtt->vm.pte_encode = snb_pte_encode;

	ggtt->vm.vma_ops.bind_vma    = intel_ggtt_bind_vma;
	ggtt->vm.vma_ops.unbind_vma  = intel_ggtt_unbind_vma;

	return ggtt_probe_common(ggtt, size);
}

static int gen12vf_ggtt_probe(struct i915_ggtt *ggtt)
{
	struct drm_i915_private *i915 = ggtt->vm.i915;

	GEM_BUG_ON(!IS_SRIOV_VF(i915));
	GEM_BUG_ON(GRAPHICS_VER(i915) < 12);

	/* there is no apperture on VFs */
	ggtt->gmadr = (struct resource) DEFINE_RES_MEM(0, 0);
	ggtt->mappable_end = 0;

	ggtt->vm.alloc_pt_dma = alloc_pt_dma;
	ggtt->vm.alloc_scratch_dma = alloc_pt_dma;

	/* safe guess as native expects the same minimum */
	ggtt->vm.total = 1ULL << (ilog2(GUC_GGTT_TOP - 1) + 1); /* roundup_pow_of_two(GUC_GGTT_TOP); */

	if (GRAPHICS_VER_FULL(i915) >= IP_VER(12, 70))
		ggtt->vm.pte_encode = mtl_ggtt_pte_encode;
	else
		ggtt->vm.pte_encode = gen8_ggtt_pte_encode;
	ggtt->vm.clear_range = nop_clear_range;
	if (i915_is_mem_wa_enabled(i915, I915_WA_USE_FLAT_PPGTT_UPDATE)) {
		ggtt->vm.insert_page = gen12_vf_ggtt_insert_page_wa_vfpf;
		ggtt->vm.insert_entries = gen12_vf_ggtt_insert_entries_wa_vfpf;
	} else {
		ggtt->vm.insert_page = gen8_ggtt_insert_page;
		ggtt->vm.insert_entries = gen8_ggtt_insert_entries;
	}
	ggtt->vm.cleanup = gen6_gmch_remove;

	ggtt->vm.vma_ops.bind_vma    = intel_ggtt_bind_vma;
	ggtt->vm.vma_ops.unbind_vma  = intel_ggtt_unbind_vma;
	ggtt->vm.vma_ops.set_pages   = ggtt_set_pages;
	ggtt->vm.vma_ops.clear_pages = clear_pages;

	ggtt->invalidate = gen12vf_ggtt_invalidate;

	return ggtt_probe_common(ggtt, sizeof(gen8_pte_t) *
				 (ggtt->vm.total >> PAGE_SHIFT));
}

static int ggtt_probe_hw(struct i915_ggtt *ggtt, struct intel_gt *gt)
{
	struct drm_i915_private *i915 = gt->i915;
	int ret;

	ggtt->vm.gt = gt;
	ggtt->vm.i915 = i915;
	ggtt->vm.dma = i915->drm.dev;

	if (IS_SRIOV_VF(i915))
		ret = gen12vf_ggtt_probe(ggtt);
	else if (GRAPHICS_VER(i915) <= 5)
		ret = intel_ggtt_gmch_probe(ggtt);
	else if (GRAPHICS_VER(i915) < 8)
		ret = gen6_gmch_probe(ggtt);
	else
		ret = gen8_gmch_probe(ggtt);
	if (ret)
		return ret;

	if ((ggtt->vm.total - 1) >> 32) {
		drm_err(&i915->drm,
			"We never expected a Global GTT with more than 32bits"
			" of address space! Found %lldM!\n",
			ggtt->vm.total >> 20);
		ggtt->vm.total = 1ULL << 32;
		ggtt->mappable_end =
			min_t(u64, ggtt->mappable_end, ggtt->vm.total);
	}

	if (ggtt->mappable_end > ggtt->vm.total) {
		drm_err(&i915->drm,
			"mappable aperture extends past end of GGTT,"
			" aperture=%pa, total=%llx\n",
			&ggtt->mappable_end, ggtt->vm.total);
		ggtt->mappable_end = ggtt->vm.total;
	}

	/*
	 * GMADR is the PCI mmio aperture into the global GTT. Likely only
	 * available for non-local memory, 0-remote-tiled hw. Anyway this will
	 * be initialized at least once as tile0.
	 */
	drm_dbg(&i915->drm, "GGTT size = %lluM\n", ggtt->vm.total >> 20);
	drm_dbg(&i915->drm, "GMADR size = %lluM\n", (u64)ggtt->mappable_end >> 20);
	drm_dbg(&i915->drm, "DSM size = %lluM\n",
		(u64)resource_size(&intel_graphics_stolen_res) >> 20);
	INIT_LIST_HEAD(&ggtt->gt_list);
	return 0;
}

/**
 * i915_ggtt_probe_hw - Probe GGTT hardware location
 * @i915: i915 device
 */
int i915_ggtt_probe_hw(struct drm_i915_private *i915)
{
	struct i915_ggtt *ggtt;
	struct intel_gt *gt;
	unsigned int i;
	int ret;

	for_each_gt(gt, i915, i) {
		ggtt = gt->ggtt;

		/*
		 * Media GT shares primary GT's GGTT
		 */
		if (gt->type == GT_MEDIA) {
			ggtt = to_gt(i915)->ggtt;
			intel_gt_init_ggtt(gt, ggtt);
			continue;
		}

		if (!ggtt)
			ggtt = kzalloc(sizeof(*ggtt), GFP_KERNEL);

		if (!ggtt) {
			ret = -ENOMEM;
			goto err;
		}

		ret = ggtt_probe_hw(ggtt, gt);
		if (ret) {
			if (ggtt != gt->ggtt)
				kfree(ggtt);
			goto err;
		}

		intel_gt_init_ggtt(gt, ggtt);
	}

	if (i915_vtd_active(i915))
		drm_info(&i915->drm, "VT-d active for gfx access\n");

	return 0;

err:
	for_each_gt(gt, i915, i) {
		if (gt->type == GT_MEDIA)
			continue;

		kfree(gt->ggtt);
	}

	return ret;
}

int i915_ggtt_enable_hw(struct drm_i915_private *i915)
{
	if (GRAPHICS_VER(i915) < 6)
		return intel_ggtt_gmch_enable_hw(i915);

	return 0;
}

/**
 * i915_ggtt_resume_vm - Restore the memory mappings for a GGTT or DPT VM
 * @vm: The VM to restore the mappings for
 *
 * Restore the memory mappings for all objects mapped to HW via the GGTT or a
 * DPT page table.
 *
 * Returns %true if restoring the mapping for any object that was in a write
 * domain before suspend.
 */
bool i915_ggtt_resume_vm(struct i915_address_space *vm)
{
	struct i915_vma *vma;
	bool write_domain_objs = false;
	int open;

	drm_WARN_ON(&vm->i915->drm, !vm->is_ggtt && !vm->is_dpt);

	/* First fill our portion of the GTT with scratch pages */
	vm->clear_range(vm, 0, vm->total);

	/* Skip rewriting PTE on VMA unbind. */
	open = atomic_xchg(&vm->open, 0);

	/* clflush objects bound into the GGTT and rebind them. */
	list_for_each_entry(vma, &vm->bound_list, vm_link) {
		struct drm_i915_gem_object *obj = vma->obj;
		unsigned int was_bound =
			atomic_read(&vma->flags) & I915_VMA_BIND_MASK;

		GEM_BUG_ON(!was_bound);
		vma->ops->bind_vma(vm, NULL, vma,
				   obj ? obj->pat_index :
					 i915_gem_get_pat_index(vm->i915,
							I915_CACHE_NONE),
				   was_bound);
		if (obj) { /* only used during resume => exclusive access */
			write_domain_objs |= fetch_and_zero(&obj->write_domain);
			obj->read_domains |= I915_GEM_DOMAIN_GTT;
		}
	}

	atomic_set(&vm->open, open);

	return write_domain_objs;
}

void i915_ggtt_resume(struct i915_ggtt *ggtt)
{
	struct intel_gt *gt;
	bool flush;

	list_for_each_entry(gt, &ggtt->gt_list, ggtt_link)
		intel_gt_check_and_clear_faults(gt);

	flush = i915_ggtt_resume_vm(&ggtt->vm);

	if (drm_mm_node_allocated(&ggtt->error_capture))
		ggtt->vm.scratch_range(&ggtt->vm, ggtt->error_capture.start,
				       ggtt->error_capture.size);

	ggtt->invalidate(ggtt);

	if (flush)
		wbinvd_on_all_cpus();

	intel_ggtt_restore_fences(ggtt);
}

static struct scatterlist *
rotate_pages(struct drm_i915_gem_object *obj, unsigned int offset,
	     unsigned int width, unsigned int height,
	     unsigned int src_stride, unsigned int dst_stride,
	     struct sg_table *st,
	     struct scatterlist *sg,
	     struct scatterlist **end)
{
	unsigned int column, row;
	pgoff_t src_idx;

	for (column = 0; column < width; column++) {
		unsigned int left;

		src_idx = src_stride * (height - 1) + column + offset;
		for (row = 0; row < height; row++) {
			/*
			 * We don't need the pages, but need to initialize
			 * the entries so the sg list can be happily traversed.
			 * The only thing we need are DMA addresses.
			 */
			sg_set_page(sg, NULL, I915_GTT_PAGE_SIZE, 0);
			sg_dma_address(sg) =
				i915_gem_object_get_dma_address(obj, src_idx);
			sg_dma_len(sg) = I915_GTT_PAGE_SIZE;

			*end = sg;
			sg = sg_next(sg);
			st->nents++;

			src_idx -= src_stride;
		}

		left = (dst_stride - height) * I915_GTT_PAGE_SIZE;
		if (!left)
			continue;

		/*
		 * The DE ignores the PTEs for the padding tiles, the sg entry
		 * here is just a conenience to indicate how many padding PTEs
		 * to insert at this spot.
		 */
		sg_set_page(sg, NULL, left, 0);
		sg_dma_address(sg) = 0;
		sg_dma_len(sg) = left;

		*end = sg;
		sg = sg_next(sg);
		st->nents++;
	}

	return sg;
}

static noinline struct sg_table *
intel_rotate_pages(struct intel_rotation_info *rot_info,
		   struct drm_i915_gem_object *obj)
{
	unsigned int size = intel_rotation_info_size(rot_info);
	struct drm_i915_private *i915 = to_i915(obj->base.dev);
	struct scatterlist *sg, *end;
	struct sg_table *st;
	int ret = -ENOMEM;
	int i;

	/* Allocate target SG list. */
	st = kmalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		goto err_st_alloc;

	ret = sg_alloc_table(st, size, GFP_KERNEL);
	if (ret)
		goto err_sg_alloc;

	st->nents = 0;
	end = sg = st->sgl;

	for (i = 0 ; i < ARRAY_SIZE(rot_info->plane); i++)
		sg = rotate_pages(obj, rot_info->plane[i].offset,
				  rot_info->plane[i].width, rot_info->plane[i].height,
				  rot_info->plane[i].src_stride,
				  rot_info->plane[i].dst_stride,
				  st, sg, &end);

	sg_mark_end(end);
	return st;

err_sg_alloc:
	kfree(st);
err_st_alloc:

	drm_dbg(&i915->drm, "Failed to create rotated mapping for object size %zu! (%ux%u tiles, %u pages)\n",
		obj->base.size, rot_info->plane[0].width,
		rot_info->plane[0].height, size);

	return ERR_PTR(ret);
}

static struct scatterlist *
add_padding_pages(unsigned int count,
		  struct sg_table *st,
		  struct scatterlist *sg,
		  struct scatterlist **end)
{
	/*
	 * The DE ignores the PTEs for the padding tiles, the sg entry
	 * here is just a convenience to indicate how many padding PTEs
	 * to insert at this spot.
	 */
	sg_set_page(sg, NULL, count * I915_GTT_PAGE_SIZE, 0);
	sg_dma_address(sg) = 0;
	sg_dma_len(sg) = count * I915_GTT_PAGE_SIZE;

	*end = sg;
	sg = sg_next(sg);
	st->nents++;

	return sg;
}

static struct scatterlist *
remap_tiled_color_plane_pages(struct drm_i915_gem_object *obj,
			      unsigned long offset, unsigned int alignment_pad,
			      unsigned int width, unsigned int height,
			      unsigned int src_stride, unsigned int dst_stride,
			      struct sg_table *st,
			      struct scatterlist *sg,
			      struct scatterlist **end,
			      unsigned int *gtt_offset)
{
	unsigned int row;

	if (!width || !height)
		return sg;

	if (alignment_pad)
		sg = add_padding_pages(alignment_pad, st, sg, end);

	for (row = 0; row < height; row++) {
		unsigned int left = width * I915_GTT_PAGE_SIZE;

		while (left) {
			dma_addr_t addr;
			unsigned int length;

			/*
			 * We don't need the pages, but need to initialize
			 * the entries so the sg list can be happily traversed.
			 * The only thing we need are DMA addresses.
			 */

			addr = i915_gem_object_get_dma_address_len(obj, offset, &length);
			length = min(left, length);

			sg_set_page(sg, NULL, length, 0);
			sg_dma_address(sg) = addr;
			sg_dma_len(sg) = length;

			*end = sg;
			sg = sg_next(sg);
			st->nents++;

			offset += length / I915_GTT_PAGE_SIZE;
			left -= length;
		}

		offset += src_stride - width;

		left = (dst_stride - width) * I915_GTT_PAGE_SIZE;
		if (!left)
			continue;

		sg = add_padding_pages(left >> PAGE_SHIFT, st, sg, end);
	}

	*gtt_offset += alignment_pad + dst_stride * height;

	return sg;
}

static struct scatterlist *
remap_contiguous_pages(struct drm_i915_gem_object *obj,
		       pgoff_t obj_offset,
		       pgoff_t page_count,
		       struct sg_table *st,
		       struct scatterlist *sg)
{
	struct scatterlist *iter;
	unsigned int offset;

	iter = i915_gem_object_get_sg_dma(obj, obj_offset, &offset);
	GEM_BUG_ON(!iter);

	do {
		unsigned long len;

		len = sg_dma_len(iter) - (offset << PAGE_SHIFT);
		len = min(len, page_count << PAGE_SHIFT);
		GEM_BUG_ON(overflows_type(len, sg->length));

		sg_set_page(sg, NULL, len, 0);
		sg_dma_address(sg) =
			sg_dma_address(iter) + (offset << PAGE_SHIFT);
		sg_dma_len(sg) = len;

		st->nents++;
		page_count -= len >> PAGE_SHIFT;
		if (page_count == 0)
			return sg;

		sg = __sg_next(sg);
		iter = __sg_next(iter);
		offset = 0;
	} while (1);
}

void intel_partial_pages_for_sg_table(struct drm_i915_gem_object *obj,
				      struct sg_table *st,
				      pgoff_t obj_offset,
				      pgoff_t page_count,
				      struct scatterlist **sgl)
{
	struct scatterlist *sg;

	GEM_BUG_ON(!st);
	st->nents = 0;

	sg = remap_contiguous_pages(obj, obj_offset, page_count, st, st->sgl);
	sg_mark_end(sg);

	if (sgl)
		*sgl = sg;
}

static struct scatterlist *
remap_linear_color_plane_pages(struct drm_i915_gem_object *obj,
			       pgoff_t obj_offset, unsigned int alignment_pad,
			       unsigned int size,
			       struct sg_table *st,
			       struct scatterlist *sg,
			       struct scatterlist **end,
			       unsigned int *gtt_offset)
{
	if (!size)
		return sg;

	if (alignment_pad)
		sg = add_padding_pages(alignment_pad, st, sg, end);

	sg = remap_contiguous_pages(obj, obj_offset, size, st, sg);

	*end = sg;
	sg = sg_next(sg);

	*gtt_offset += alignment_pad + size;
	return sg;
}

static struct scatterlist *
remap_color_plane_pages(const struct intel_remapped_info *rem_info,
			struct drm_i915_gem_object *obj,
			int color_plane,
			struct sg_table *st,
			struct scatterlist *sg,
			struct scatterlist **end,
			unsigned int *gtt_offset)
{
	unsigned int alignment_pad = 0;

	if (rem_info->plane_alignment)
		alignment_pad = ALIGN(*gtt_offset, rem_info->plane_alignment) - *gtt_offset;

	if (rem_info->plane[color_plane].linear)
		sg = remap_linear_color_plane_pages(obj,
						    rem_info->plane[color_plane].offset,
						    alignment_pad,
						    rem_info->plane[color_plane].size,
						    st, sg, end,
						    gtt_offset);

	else
		sg = remap_tiled_color_plane_pages(obj,
						   rem_info->plane[color_plane].offset,
						   alignment_pad,
						   rem_info->plane[color_plane].width,
						   rem_info->plane[color_plane].height,
						   rem_info->plane[color_plane].src_stride,
						   rem_info->plane[color_plane].dst_stride,
						   st, sg, end,
						   gtt_offset);

	return sg;
}

static noinline struct sg_table *
intel_remap_pages(struct intel_remapped_info *rem_info,
		  struct drm_i915_gem_object *obj)
{
	unsigned int size = intel_remapped_info_size(rem_info);
	struct drm_i915_private *i915 = to_i915(obj->base.dev);
	struct scatterlist *sg, *end;
	unsigned int gtt_offset = 0;
	struct sg_table *st;
	int ret = -ENOMEM;
	int i;

	/* Allocate target SG list. */
	st = kmalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		goto err_st_alloc;

	ret = sg_alloc_table(st, size, GFP_KERNEL);
	if (ret)
		goto err_sg_alloc;

	st->nents = 0;
	end = sg = st->sgl;

	for (i = 0 ; i < ARRAY_SIZE(rem_info->plane); i++)
		sg = remap_color_plane_pages(rem_info, obj, i, st, sg, &end, &gtt_offset);

	sg_mark_end(end);
	i915_sg_trim(st);

	return st;

err_sg_alloc:
	kfree(st);
err_st_alloc:

	drm_dbg(&i915->drm, "Failed to create remapped mapping for object size %zu! (%ux%u tiles, %u pages)\n",
		obj->base.size, rem_info->plane[0].width,
		rem_info->plane[0].height, size);

	return ERR_PTR(ret);
}

static noinline struct sg_table *
intel_partial_pages(const struct i915_ggtt_view *view,
		    struct drm_i915_gem_object *obj)
{
	struct sg_table *st;
	int ret;

	st = kmalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		return ERR_PTR(-ENOMEM);

	ret = sg_alloc_table(st, view->partial.size, GFP_KERNEL);
	if (ret) {
		kfree(st);
		return ERR_PTR(ret);
	}

	intel_partial_pages_for_sg_table(obj, st, view->partial.offset,
					 view->partial.size, NULL);
	i915_sg_trim(st);

	return st;
}

static int
i915_get_ggtt_vma_pages(struct i915_vma *vma)
{
	int ret;

	/*
	 * The vma->pages are only valid within the lifespan of the borrowed
	 * obj->mm.pages. When the obj->mm.pages sg_table is regenerated, so
	 * must be the vma->pages. A simple rule is that vma->pages must only
	 * be accessed when the obj->mm.pages are pinned.
	 */
	GEM_BUG_ON(!i915_gem_object_has_pinned_pages(vma->obj));

	if (vma->ggtt_view.type != I915_GGTT_VIEW_NORMAL) {
		ret = i915_gem_object_migrate_sync(vma->obj);
		if (ret)
			return ret;
	}

	switch (vma->ggtt_view.type) {
	default:
		GEM_BUG_ON(vma->ggtt_view.type);
		fallthrough;
	case I915_GGTT_VIEW_NORMAL:
		vma->pages = vma->obj->mm.pages;
		return 0;

	case I915_GGTT_VIEW_ROTATED:
		vma->pages =
			intel_rotate_pages(&vma->ggtt_view.rotated, vma->obj);
		break;

	case I915_GGTT_VIEW_REMAPPED:
		vma->pages =
			intel_remap_pages(&vma->ggtt_view.remapped, vma->obj);
		break;

	case I915_GGTT_VIEW_PARTIAL:
		vma->pages = intel_partial_pages(&vma->ggtt_view, vma->obj);
		break;
	}

	ret = 0;
	if (IS_ERR(vma->pages)) {
		ret = PTR_ERR(vma->pages);
		vma->pages = NULL;
		drm_err(&vma->vm->i915->drm,
			"Failed to get pages for VMA view type %u (%d)!\n",
			vma->ggtt_view.type, ret);
	}
	return ret;
}

/**
 * i915_ggtt_balloon - reserve fixed space in an GGTT
 * @ggtt: the &struct i915_ggtt
 * @start: start offset inside the GGTT,
 *          must be #I915_GTT_MIN_ALIGNMENT aligned
 * @end: end offset inside the GGTT,
 *        must be #I915_GTT_PAGE_SIZE aligned
 * @node: the &struct drm_mm_node
 *
 * i915_ggtt_balloon() tries to reserve the @node from @start to @end inside
 * GGTT the address space.
 *
 * Returns: 0 on success, -ENOSPC if no suitable hole is found.
 */
int i915_ggtt_balloon(struct i915_ggtt *ggtt, u64 start, u64 end,
		      struct drm_mm_node *node)
{
	u64 size = end - start;
	int err;

	GEM_BUG_ON(start >= end);
	drm_dbg(&ggtt->vm.i915->drm, "%sGGTT [%#llx-%#llx] %lluK\n",
		"ballooning ", start, end, size / SZ_1K);

	err = i915_gem_gtt_reserve(&ggtt->vm, node, size, start,
				   I915_COLOR_UNEVICTABLE, PIN_NOEVICT);
	if (unlikely(err)) {
		intel_gt_log_driver_error(ggtt->vm.gt, INTEL_GT_DRIVER_ERROR_GGTT,
					  "%sGGTT [%#llx-%#llx] %lluK\n",
					  "Failed to balloon ", node->start,
					  node->start + node->size, node->size / SZ_1K);
		return err;
	}

	ggtt->vm.reserved += node->size;
	return 0;
}

bool i915_ggtt_has_xehpsdv_pte_vfid_mask(struct i915_ggtt *ggtt)
{
	return GRAPHICS_VER_FULL(ggtt->vm.i915) < IP_VER(12, 50);
}

void i915_ggtt_deballoon(struct i915_ggtt *ggtt, struct drm_mm_node *node)
{
	if (!drm_mm_node_allocated(node))
		return;

	drm_dbg(&ggtt->vm.i915->drm, "%sGGTT [%#llx-%#llx] %lluK\n",
		"deballooning ", node->start, node->start + node->size,
		node->size / SZ_1K);

	GEM_BUG_ON(ggtt->vm.reserved < node->size);
	ggtt->vm.reserved -= node->size;
	drm_mm_remove_node(node);
}

static gen8_pte_t tgl_prepare_vf_pte_vfid(u16 vfid)
{
	GEM_BUG_ON(!FIELD_FIT(TGL_GGTT_PTE_VFID_MASK, vfid));

	return FIELD_PREP(TGL_GGTT_PTE_VFID_MASK, vfid);
}

static gen8_pte_t xehpsdv_prepare_vf_pte_vfid(u16 vfid)
{
	GEM_BUG_ON(!FIELD_FIT(XEHPSDV_GGTT_PTE_VFID_MASK, vfid));

	return FIELD_PREP(XEHPSDV_GGTT_PTE_VFID_MASK, vfid);
}

static gen8_pte_t prepare_vf_pte_vfid(struct i915_ggtt *ggtt, u16 vfid)
{
	if (i915_ggtt_has_xehpsdv_pte_vfid_mask(ggtt))
		return tgl_prepare_vf_pte_vfid(vfid);
	else
		return xehpsdv_prepare_vf_pte_vfid(vfid);
}

static gen8_pte_t prepare_vf_pte(struct i915_ggtt *ggtt, u16 vfid)
{
	return prepare_vf_pte_vfid(ggtt, vfid) | GEN8_PAGE_PRESENT;
}

void i915_ggtt_set_space_owner(struct i915_ggtt *ggtt, u16 vfid,
			       const struct drm_mm_node *node)
{
	gen8_pte_t __iomem *gtt_entries = ggtt->gsm;
	const gen8_pte_t pte = prepare_vf_pte(ggtt, vfid);
	u64 base = node->start;
	u64 size = node->size;

	GEM_BUG_ON(!IS_SRIOV_PF(ggtt->vm.i915));
	GEM_BUG_ON(base % PAGE_SIZE);
	GEM_BUG_ON(size % PAGE_SIZE);

	drm_dbg(&ggtt->vm.i915->drm, "GGTT VF%u [%#llx-%#llx] %lluK\n",
		vfid, base, base + size, size / SZ_1K);

	gtt_entries += base >> PAGE_SHIFT;
	while (size) {
		gen8_set_pte(gtt_entries++, pte);
		size -= PAGE_SIZE;
	}

	ggtt->invalidate(ggtt);
}

static inline unsigned int __ggtt_size_to_ptes_size(u64 ggtt_size)
{
	GEM_BUG_ON(!IS_ALIGNED(ggtt_size, I915_GTT_MIN_ALIGNMENT));

	return (ggtt_size >> PAGE_SHIFT) * sizeof(gen8_pte_t);
}

static void ggtt_pte_clear_vfid(void *buf, u64 size)
{
	while (size) {
		*(gen8_pte_t *)buf &= ~XEHPSDV_GGTT_PTE_VFID_MASK;

		buf += sizeof(gen8_pte_t);
		size -= sizeof(gen8_pte_t);
	}
}

/**
 * i915_ggtt_save_ptes - copy GGTT PTEs to preallocated buffer
 * @ggtt: the &struct i915_ggtt
 * @node: the &struct drm_mm_node - the @node->start is used as the start offset for save
 * @buf: preallocated buffer in which PTEs will be saved
 * @size: size of prealocated buffer (in bytes)
 *        - must be sizeof(gen8_pte_t) aligned
 * @flags: function flags:
 *         - #I915_GGTT_SAVE_PTES_NO_VFID BIT - save PTEs without VFID
 *
 * Returns: size of the buffer used (or needed if both @buf and @size are (0)) to store all PTEs
 *          for a given node, -EINVAL if one of @buf or @size is 0.
 */
int i915_ggtt_save_ptes(struct i915_ggtt *ggtt, const struct drm_mm_node *node, void *buf,
			unsigned int size, unsigned int flags)
{
	gen8_pte_t __iomem *gtt_entries = ggtt->gsm;

	if (!buf && !size)
		return __ggtt_size_to_ptes_size(node->size);

	if (!buf || !size)
		return -EINVAL;

	GEM_BUG_ON(!IS_ALIGNED(size, sizeof(gen8_pte_t)));
	GEM_WARN_ON(size > __ggtt_size_to_ptes_size(SZ_4G));

	if (size < __ggtt_size_to_ptes_size(node->size))
		return -ENOSPC;
	size = __ggtt_size_to_ptes_size(node->size);

	gtt_entries += node->start >> PAGE_SHIFT;

	memcpy_fromio(buf, gtt_entries, size);

	if (flags & I915_GGTT_SAVE_PTES_NO_VFID)
		ggtt_pte_clear_vfid(buf, size);

	return size;
}

/**
 * i915_ggtt_restore_ptes() -  restore GGTT PTEs from buffer
 * @ggtt: the &struct i915_ggtt
 * @node: the &struct drm_mm_node - the @node->start is used as the start offset for restore
 * @buf: buffer from which PTEs will be restored
 * @size: size of prealocated buffer (in bytes)
 *        - must be sizeof(gen8_pte_t) aligned
 * @flags: function flags:
 *         - #I915_GGTT_RESTORE_PTES_VFID_MASK - VFID for restored PTEs
 *         - #I915_GGTT_RESTORE_PTES_NEW_VFID - restore PTEs with new VFID
 *           (from #I915_GGTT_RESTORE_PTES_VFID_MASK)
 *
 * Returns: 0 on success, -ENOSPC if @node->size is less than size.
 */
int i915_ggtt_restore_ptes(struct i915_ggtt *ggtt, const struct drm_mm_node *node, const void *buf,
			   unsigned int size, unsigned int flags)
{
	gen8_pte_t __iomem *gtt_entries = ggtt->gsm;
	u32 vfid = FIELD_GET(I915_GGTT_RESTORE_PTES_VFID_MASK, flags);
	gen8_pte_t pte;

	GEM_BUG_ON(!size);
	GEM_BUG_ON(!IS_ALIGNED(size, sizeof(gen8_pte_t)));

	if (size > __ggtt_size_to_ptes_size(node->size))
		return -ENOSPC;

	gtt_entries += node->start >> PAGE_SHIFT;

	while (size) {
		pte = *(gen8_pte_t *)buf;
		if (flags & I915_GGTT_RESTORE_PTES_NEW_VFID)
			pte |= prepare_vf_pte_vfid(ggtt, vfid);
		gen8_set_pte(gtt_entries++, pte);

		buf += sizeof(gen8_pte_t);
		size -= sizeof(gen8_pte_t);
	}

	ggtt->invalidate(ggtt);

	return 0;
}
