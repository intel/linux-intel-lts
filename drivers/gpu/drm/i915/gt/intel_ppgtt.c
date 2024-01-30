// SPDX-License-Identifier: MIT
/*
 * Copyright © 2020 Intel Corporation
 */

#include <linux/slab.h>

#include "gem/i915_gem_lmem.h"

#include "i915_trace.h"
#include "intel_gt.h"
#include "intel_gtt.h"
#include "intel_tlb.h"
#include "gen6_ppgtt.h"
#include "gen8_ppgtt.h"

struct i915_page_table *alloc_pt(struct i915_address_space *vm, int sz)
{
	struct i915_page_table *pt;

	pt = kmalloc(sizeof(*pt), I915_GFP_ALLOW_FAIL);
	if (unlikely(!pt))
		return ERR_PTR(-ENOMEM);

	pt->base = vm->alloc_pt_dma(vm, sz);
	if (IS_ERR(pt->base)) {
		kfree(pt);
		return ERR_PTR(-ENOMEM);
	}

	pt->is_compact = false;
	atomic_set(&pt->used, 0);
	return pt;
}

struct i915_page_directory *__alloc_pd(int count)
{
	struct i915_page_directory *pd;

	pd = kzalloc(sizeof(*pd), I915_GFP_ALLOW_FAIL);
	if (unlikely(!pd))
		return NULL;

	pd->entry = kcalloc(count, sizeof(*pd->entry), I915_GFP_ALLOW_FAIL);
	if (unlikely(!pd->entry)) {
		kfree(pd);
		return NULL;
	}

	spin_lock_init(&pd->lock);
	return pd;
}

struct i915_page_directory *alloc_pd(struct i915_address_space *vm)
{
	struct i915_page_directory *pd;

	pd = __alloc_pd(I915_PDES);
	if (unlikely(!pd))
		return ERR_PTR(-ENOMEM);

	pd->pt.base = vm->alloc_pt_dma(vm, I915_GTT_PAGE_SIZE_4K);
	if (IS_ERR(pd->pt.base)) {
		kfree(pd->entry);
		kfree(pd);
		return ERR_PTR(-ENOMEM);
	}

	return pd;
}

void free_px(struct i915_address_space *vm, struct i915_page_table *pt, int lvl)
{
	BUILD_BUG_ON(offsetof(struct i915_page_directory, pt));

	if (lvl) {
		struct i915_page_directory *pd =
			container_of(pt, typeof(*pd), pt);
		kfree(pd->entry);
	}

	if (pt->base)
		i915_gem_object_put(pt->base);

	kfree(pt);
}

static void
write_dma_entry(struct drm_i915_gem_object * const pdma,
		const unsigned short idx,
		const u64 encoded_entry)
{
	bool needs_flush;
	u64 * const vaddr = __px_vaddr(pdma, &needs_flush);

	WRITE_ONCE(vaddr[idx], encoded_entry);
	if (needs_flush)
		drm_clflush_virt_range(&vaddr[idx], sizeof(u64));
}

void
__set_pd_entry(struct i915_page_directory * const pd,
	       const unsigned short idx,
	       struct i915_page_table * const to,
	       u64 (*encode)(const dma_addr_t, const enum i915_cache_level))
{
	/* Each thread pre-pins the pd, and we may have a thread per pde. */
	GEM_BUG_ON(atomic_read(px_used(pd)) > NALLOC * I915_PDES);

	atomic_inc(px_used(pd));
	pd->entry[idx] = to;
	write_dma_entry(px_base(pd), idx, encode(px_dma(to), I915_CACHE_LLC));
}

void
clear_pd_entry(struct i915_page_directory * const pd,
	       const unsigned short idx, u64 scratch_encode)
{
	GEM_BUG_ON(atomic_read(px_used(pd)) == 0);

	write_dma_entry(px_base(pd), idx, scratch_encode);
	pd->entry[idx] = NULL;
	atomic_dec(px_used(pd));
}

bool
release_pd_entry(struct i915_page_directory * const pd,
		 const unsigned short idx,
		 struct i915_page_table * const pt,
		 u64 scratch_encode)
{
	bool free = false;

	if (atomic_add_unless(&pt->used, -1, 1))
		return false;

	spin_lock(&pd->lock);
	if (atomic_dec_and_test(&pt->used)) {
		clear_pd_entry(pd, idx, scratch_encode);
		free = true;
	}
	spin_unlock(&pd->lock);

	return free;
}

int i915_ppgtt_init_hw(struct intel_gt *gt)
{
	struct drm_i915_private *i915 = gt->i915;

	gtt_write_workarounds(gt);

	if (GRAPHICS_VER(i915) == 6)
		gen6_ppgtt_enable(gt);
	else if (GRAPHICS_VER(i915) == 7)
		gen7_ppgtt_enable(gt);

	return 0;
}

static struct i915_ppgtt *
__ppgtt_create(struct intel_gt *gt, u32 flags)
{
	if (GRAPHICS_VER(gt->i915) < 8)
		return gen6_ppgtt_create(gt);
	else
		return gen8_ppgtt_create(gt, flags);
}

struct i915_ppgtt *i915_ppgtt_create(struct intel_gt *gt, u32 flags)
{
	struct i915_ppgtt *ppgtt;

	ppgtt = __ppgtt_create(gt, flags);
	if (IS_ERR(ppgtt))
		return ppgtt;

	trace_i915_ppgtt_create(&ppgtt->vm);

	return ppgtt;
}

void ppgtt_bind_vma(struct i915_address_space *vm,
		    struct i915_vm_pt_stash *stash,
		    struct i915_vma *vma,
		    unsigned int pat_index,
		    u32 flags)
{
	u32 pte_flags;

	/* Paper over race with vm_unbind */
	if (!drm_mm_node_allocated(&vma->node))
		return;

	if (!(flags & PIN_RESIDENT)) {
		/*
		 * Force the next access to this vam to trigger a pagefault. This
		 * only installs a NULL PTE, and will *not* populate TLB.
		 */
		gen12_init_fault_scratch(vm,
					 i915_vma_offset(vma),
					 i915_vma_size(vma),
					 false);
		return;
	}

	if (!test_bit(I915_VMA_ALLOC_BIT, __i915_vma_flags(vma))) {
		GEM_BUG_ON(vma->size > i915_vma_size(vma));
		vm->allocate_va_range(vm, stash,
				      i915_vma_offset(vma),
				      vma->size);
		set_bit(I915_VMA_ALLOC_BIT, __i915_vma_flags(vma));
	}

	/* Applicable to VLV, and gen8+ */
	pte_flags = 0;
	if (vma->vm->has_read_only && i915_gem_object_is_readonly(vma->obj))
		pte_flags |= PTE_READ_ONLY;
	if (i915_gem_object_is_lmem(vma->obj) ||
	    i915_gem_object_has_fabric(vma->obj))
		pte_flags |= (vma->vm->top == 4 ? PTE_LM | PTE_AE : PTE_LM);

	vm->insert_entries(vm, stash, vma, pat_index, pte_flags);

	/* Flush the PTE writes to memory */
	i915_write_barrier(vm->i915);

	/*
	 * If there were virtual address range mapped to scratch page, all
	 * the vm_bind after should invalid the scratch mapping, as this
	 * range might have already been cached in TLB.
	 */
	if (vm->invalidate_tlb_scratch) {
		struct intel_gt *gt;
		int i;

		for_each_gt(gt, vm->i915, i) {
			if (!atomic_read(&vm->active_contexts_gt[i]))
				continue;

			intel_gt_invalidate_tlb_range(gt, vm,
						      i915_vma_offset(vma),
						      i915_vma_size(vma));
		}
	}
}

static void ppgtt_bind_vma_wa(struct i915_address_space *vm,
			      struct i915_vm_pt_stash *stash,
			      struct i915_vma *vma,
			      unsigned int pat_index,
			      u32 flags)
{
	GEM_WARN_ON(i915_gem_idle_engines(vm->i915));

	ppgtt_bind_vma(vm, stash, vma, pat_index, flags);

	GEM_WARN_ON(i915_gem_resume_engines(vm->i915));
}

static void vma_invalidate_tlb(struct i915_vma *vma)
{
	struct i915_address_space *vm = vma->vm;
	struct drm_i915_gem_object *obj;
	struct intel_gt *gt;
	int id;

	obj = vma->obj;
	if (!obj)
		return;

	/*
	 * Before we release the pages that were bound by this vma, we
	 * must invalidate all the TLBs that may still have a reference
	 * back to our physical address. It only needs to be done once,
	 * so after updating the PTE to point away from the pages, record
	 * the most recent TLB invalidation seqno, and if we have not yet
	 * flushed the TLBs upon release, perform a full invalidation.
	 */
	for_each_gt(gt, vm->i915, id) {
		WRITE_ONCE(obj->mm.tlb[id], 0);
		if (!atomic_read(&vm->active_contexts_gt[id]))
			continue;

		if (!intel_gt_invalidate_tlb_range(gt, vm,
						   i915_vma_offset(vma),
						   i915_vma_size(vma)))
			WRITE_ONCE(obj->mm.tlb[id],
				   intel_gt_next_invalidate_tlb_full(vm->gt));
	}
}

void ppgtt_unbind_vma(struct i915_address_space *vm, struct i915_vma *vma)
{
	if (!test_and_clear_bit(I915_VMA_ALLOC_BIT, __i915_vma_flags(vma)))
		return;

	vm->clear_range(vm, i915_vma_offset(vma), vma->size);
	vma_invalidate_tlb(vma);
}

static void ppgtt_unbind_vma_wa(struct i915_address_space *vm,
				struct i915_vma *vma)
{
	GEM_WARN_ON(i915_gem_idle_engines(vma->vm->i915));

	ppgtt_unbind_vma(vm, vma);

	GEM_WARN_ON(i915_gem_resume_engines(vma->vm->i915));
}

static unsigned long pd_count(u64 size, int shift)
{
	/* Beware later misalignment */
	return (size + 2 * (BIT_ULL(shift) - 1)) >> shift;
}

u64 i915_vm_estimate_pt_size(struct i915_address_space *vm, u64 size)
{
	return pd_count(size, vm->pd_shift) * I915_GTT_PAGE_SIZE_4K;
}

int i915_vm_alloc_pt_stash(struct i915_address_space *vm,
			   struct i915_vm_pt_stash *stash,
			   u64 size)
{
	unsigned long count;
	int shift, n, pt_sz;

	shift = vm->pd_shift;
	if (!shift)
		return 0;

	pt_sz = stash->pt_sz;
	if (!pt_sz)
		pt_sz = I915_GTT_PAGE_SIZE_4K;
	else
		GEM_BUG_ON(!IS_DGFX(vm->i915));

	GEM_BUG_ON(!is_power_of_2(pt_sz));

	count = pd_count(size, shift);
	while (count--) {
		struct i915_page_table *pt;

		pt = alloc_pt(vm, pt_sz);
		if (IS_ERR(pt)) {
			i915_vm_free_pt_stash(vm, stash);
			return PTR_ERR(pt);
		}

		pt->stash = stash->pt[0];
		stash->pt[0] = pt;
	}

	for (n = 1; n < vm->top; n++) {
		shift += ilog2(I915_PDES); /* Each PD holds 512 entries */
		count = pd_count(size, shift);
		while (count--) {
			struct i915_page_directory *pd;

			pd = alloc_pd(vm);
			if (IS_ERR(pd)) {
				i915_vm_free_pt_stash(vm, stash);
				return PTR_ERR(pd);
			}

			pd->pt.stash = stash->pt[1];
			stash->pt[1] = &pd->pt;
		}
	}

	return 0;
}

int i915_vm_map_pt_stash(struct i915_address_space *vm,
			 struct i915_vm_pt_stash *stash)
{
	struct i915_page_table *pt;
	int n, err;

	for (n = 0; n < ARRAY_SIZE(stash->pt); n++) {
		for (pt = stash->pt[n]; pt; pt = pt->stash) {
			err = map_pt_dma_locked(vm, pt->base);
			if (err)
				return err;
		}
	}

	return 0;
}

void i915_vm_free_pt_stash(struct i915_address_space *vm,
			   struct i915_vm_pt_stash *stash)
{
	struct i915_page_table *pt;
	int n;

	for (n = 0; n < ARRAY_SIZE(stash->pt); n++) {
		while ((pt = stash->pt[n])) {
			stash->pt[n] = pt->stash;
			free_px(vm, pt, n);
		}
	}
}

int ppgtt_set_pages(struct i915_vma *vma)
{
	GEM_BUG_ON(vma->pages);

	vma->pages = vma->obj->mm.pages;
	vma->page_sizes = vma->obj->mm.page_sizes;

	return 0;
}

int ppgtt_init(struct i915_ppgtt *ppgtt, struct intel_gt *gt)
{
	struct drm_i915_private *i915 = gt->i915;
	bool enable_wa = i915_is_mem_wa_enabled(i915, I915_WA_IDLE_GPU_BEFORE_UPDATE);
	unsigned int ppgtt_size = INTEL_INFO(i915)->ppgtt_size;
	int err;

	ppgtt->vm.gt = gt;
	ppgtt->vm.i915 = i915;
	ppgtt->vm.dma = i915->drm.dev;
	ppgtt->vm.total = BIT_ULL(ppgtt_size);

	if (ppgtt_size > 48)
		ppgtt->vm.top = 4;
	else if (ppgtt_size > 32)
		ppgtt->vm.top = 3;
	else if (ppgtt_size == 32)
		ppgtt->vm.top = 2;
	else
		ppgtt->vm.top = 1;

	err = i915_address_space_init(&ppgtt->vm, VM_CLASS_PPGTT);
	if (err)
		return err;

	ppgtt->vm.vma_ops.bind_vma    = enable_wa ? ppgtt_bind_vma_wa : ppgtt_bind_vma;
	ppgtt->vm.vma_ops.unbind_vma  = enable_wa ? ppgtt_unbind_vma_wa : ppgtt_unbind_vma;
	ppgtt->vm.vma_ops.set_pages   = ppgtt_set_pages;
	ppgtt->vm.vma_ops.clear_pages = clear_pages;

	return 0;
}
