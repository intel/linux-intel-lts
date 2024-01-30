// SPDX-License-Identifier: MIT
/*
 * Copyright © 2022 Intel Corporation
 */

#include "i915_selftest.h"

#include "gem/i915_gem_internal.h"
#include "gem/i915_gem_region.h"

#include "gen8_engine_cs.h"
#include "i915_gem_ww.h"
#include "intel_engine_regs.h"
#include "intel_gpu_commands.h"
#include "intel_context.h"
#include "intel_gt.h"
#include "intel_ring.h"

#include "selftests/igt_flush_test.h"
#include "selftests/i915_random.h"

static void clear_dw(struct i915_vma *vma, u64 addr, u32 val)
{
	GEM_BUG_ON(addr < i915_vma_offset(vma));
	GEM_BUG_ON(addr >= i915_vma_offset(vma) + i915_vma_size(vma));
	memset32(page_mask_bits(vma->obj->mm.mapping) +
		 (addr - i915_vma_offset(vma)), val, 1);
}

static int
pte_tlbinv(struct intel_context *ce,
	   struct i915_vma *va,
	   struct i915_vma *vb,
	   u64 align,
	   void (*tlbinv)(struct i915_address_space *vm, u64 addr, u64 length),
	   u64 length,
	   struct rnd_state *prng)
{
	const unsigned int pat_index =
		i915_gem_get_pat_index(ce->vm->i915, I915_CACHE_NONE);
	const int use_64b = GRAPHICS_VER(ce->vm->i915) >= 8;
	struct drm_i915_gem_object *batch;
	struct i915_request *rq;
	struct i915_vma *vma;
	int retries;
	u64 addr;
	int err;
	u32 *cs;

	batch = i915_gem_object_create_internal(ce->vm->i915, 4096);
	if (IS_ERR(batch))
		return PTR_ERR(batch);

	vma = i915_vma_instance(batch, ce->vm, NULL);
	if (IS_ERR(vma)) {
		err = PTR_ERR(vma);
		goto out;
	}

	err = i915_vma_pin(vma, 0, 0, PIN_USER | PIN_ZONE_48);
	if (err)
		goto out;

	retries = 5;
	do {
		addr = igt_random_offset(prng,
					 i915_vma_offset(vma),
					 /* upper limit for MI_BB_START */
					 min(ce->vm->total, BIT_ULL(48)),
					 va->size, 4);

		err = i915_vma_pin(va,  0, 0, (addr & -align) | PIN_OFFSET_FIXED | PIN_USER);
	} while (err == -ENOSPC && --retries);
	if (err) {
		err = 0;
		goto out;
	}
	GEM_BUG_ON(i915_vma_offset(va) != (addr & -align));
	vb->node = va->node; /* overwrites the _same_ PTE  */

	if (align == SZ_64K) {
		u64 end = addr + va->size;

		/*
		 * SZ_64K pages on dg1 require that the whole PT be marked
		 * containing 64KiB entries. So we make sure that our vma
		 * covers the whole PT, despite being randomly aligned to 64KiB
		 * and restrict our sampling to the 2MiB PT within where
		 * we know that we will be using 64KiB pages.
		 */
		addr = round_up(addr & -align, SZ_2M);
		addr |=	igt_random_offset(prng, 0, end - addr, 4, 4);
	}

	if (addr - i915_vma_offset(va) >= i915_vma_size(va))
		addr = igt_random_offset(prng,
					 i915_vma_offset(va),
					 i915_vma_offset(va) + i915_vma_size(va),
					 4, 4);

	pr_info("%s(%s): Sampling %llx, with alignment %llx, using PTE size %x, invalidate:%llx+%llx\n",
		ce->engine->name, va->obj->mm.region.mem->name ?: "smem",
		addr, align, va->page_sizes,
		addr & -length, length);

	cs = i915_gem_object_pin_map_unlocked(batch, I915_MAP_WC);
	*cs++ = MI_NOOP; /* for later termination */

	/* Sample the target to see if we spot an incorrect page */
	*cs++ = MI_CONDITIONAL_BATCH_BUFFER_END | MI_DO_COMPARE | (1 + use_64b);
	*cs++ = -2; /* break if *addr < -1 */
	*cs++ = lower_32_bits(addr);
	*cs++ = upper_32_bits(addr);
	clear_dw(va, addr, -1);
	clear_dw(vb, addr, 0);

	/* Keep sampling until we get bored */
	*cs++ = MI_BATCH_BUFFER_START | BIT(8) | use_64b;
	*cs++ = lower_32_bits(i915_vma_offset(vma));
	*cs++ = upper_32_bits(i915_vma_offset(vma));

	i915_gem_object_flush_map(batch);

	rq = i915_request_create(ce);
	if (IS_ERR(rq)) {
		err = PTR_ERR(rq);
		goto out_va;
	}

	err = rq->engine->emit_bb_start(rq, i915_vma_offset(vma), 0, 0);
	if (err) {
		i915_request_add(rq);
		goto out_va;
	}

	i915_request_get(rq);
	i915_request_add(rq);

	/* Short sleep to sanitycheck the batch is spinning before we begin */
	msleep(10);
	if (va == vb) {
		if (!i915_request_completed(rq)) {
			pr_err("Semaphore sanitycheck failed\n");
			err = -EIO;
		}
	} else if (!i915_request_completed(rq)) {
		struct i915_vm_pt_stash stash = {};
		unsigned int pte_flags = 0;
		struct i915_gem_ww_ctx ww;

		err = i915_vm_alloc_pt_stash(ce->vm, &stash, vb->size);
		if (err)
			goto err;

		for_i915_gem_ww(&ww, err, false) {
			err = i915_vm_lock_objects(ce->vm, &ww);
			if (err)
				continue;

			err = i915_vm_map_pt_stash(ce->vm, &stash);
		}
		if (err) {
			i915_vm_free_pt_stash(ce->vm, &stash);
			goto err;
		}

		/* Flip the PTE between A and B */
		if (i915_gem_object_is_lmem(vb->obj))
			pte_flags |= PTE_LM;
		ce->vm->insert_entries(ce->vm, &stash, vb, pat_index, pte_flags);

		/* Flush the PTE update to concurrent HW */
		tlbinv(ce->vm, addr & -length, length);

		if (wait_for(i915_request_completed(rq), HZ / 2)) {
			pr_err("%s: Request did not complete; the COND_BBE did not read the updated PTE\n",
			       ce->engine->name);
			err = -EINVAL;
		}

		i915_vm_free_pt_stash(ce->vm, &stash);
	} else {
		pr_err("Spinner sanitycheck failed\n");
		err = -EIO;
	}
err:
	i915_request_put(rq);

	cs = page_mask_bits(batch->mm.mapping);
	*cs = MI_BATCH_BUFFER_END;
	wmb();

out_va:
	if (vb != va)
		memset(&vb->node, 0, sizeof(vb->node));
	i915_vma_unpin(va);
	if (i915_vma_unbind(va))
		err = -EIO;
out:
	i915_gem_object_put(batch);
	return err;
}

static struct drm_i915_gem_object *create_lmem(struct intel_gt *gt)
{
	return i915_gem_object_create_lmem(gt->i915, SZ_1G, I915_BO_ALLOC_CONTIGUOUS);
}

static struct drm_i915_gem_object *create_smem(struct intel_gt *gt)
{
	/*
	 * SZ_64K pages require covering the whole 2M PT (gen8 to tgl/dg1).
	 * While that does not require the whole 2M block to be contiguous
	 * it is easier to make it so, since we need that for SZ_2M pagees.
	 * Since we randomly offset the start of the vma, we need a 4M object
	 * so that there is a 2M range within it is suitable for SZ_64K PTE.
	 */
	return i915_gem_object_create_internal(gt->i915, SZ_4M);
}

static int
mem_tlbinv(struct intel_gt *gt,
	   struct drm_i915_gem_object *(*create_fn)(struct intel_gt *),
	   void (*tlbinv)(struct i915_address_space *vm, u64 addr, u64 length))
{
	struct intel_engine_cs *engine;
	struct drm_i915_gem_object *A, *B;
	struct i915_ppgtt *ppgtt;
	struct i915_vma *va, *vb;
	enum intel_engine_id id;
	I915_RND_STATE(prng);
	LIST_HEAD(discard);
	void *vaddr;
	int err;

	if (GRAPHICS_VER(gt->i915) < 6) /* MI_CONDITIONAL_BB_END & bcs */
		return 0;

	/*
	 * Check that the TLB invalidate is able to revoke an active
	 * page. We load a page into a spinning COND_BBE loop and then
	 * remap that page to a new physical address. The old address, and
	 * so the loop keeps spinning, is retained in the TLB cache until
	 * we issue an invalidate.
	 */

	A = create_fn(gt);
	if (IS_ERR(A))
		return PTR_ERR(A);

	vaddr = i915_gem_object_pin_map_unlocked(A, I915_MAP_WC);
	if (IS_ERR(vaddr)) {
		err = PTR_ERR(vaddr);
		goto out_a;
	}

	/* Allocate a second physical address significantly different from A */
	do {
		B = create_fn(gt);
		if (IS_ERR(B)) {
			err = PTR_ERR(B);
			goto out_a;
		}

		err = i915_gem_object_pin_pages_unlocked(B);
		if (err)
			goto out_b;

		if (upper_32_bits(i915_gem_object_get_dma_address(A, 0)) !=
		    upper_32_bits(i915_gem_object_get_dma_address(B, 0)))
			break;

		list_add(&B->st_link, &discard);
	} while (1);

	vaddr = i915_gem_object_pin_map_unlocked(B, I915_MAP_WC);
	if (IS_ERR(vaddr)) {
		err = PTR_ERR(vaddr);
		goto out_b;
	}

	GEM_BUG_ON(A->base.size != B->base.size);
	if (!sg_is_last(A->mm.pages->sgl) || !sg_is_last(B->mm.pages->sgl))
		pr_warn("Failed to allocate contiguous pages for size %zx\n",
			A->base.size);

	ppgtt = i915_ppgtt_create(gt, 0);
	if (IS_ERR(ppgtt)) {
		err = PTR_ERR(ppgtt);
		goto out_b;
	}

	va = i915_vma_instance(A, &ppgtt->vm, NULL);
	if (IS_ERR(va)) {
		err = PTR_ERR(va);
		goto out_vm;
	}

	vb = i915_vma_instance(B, &ppgtt->vm, NULL);
	if (IS_ERR(vb)) {
		err = PTR_ERR(vb);
		goto out_vm;
	}
	/* manual prep as we overwrite va's GTT range with vb later */
	ppgtt_set_pages(vb);

	err = 0;
	for_each_engine(engine, gt, id) {
		struct i915_gem_ww_ctx ww;
		struct intel_context *ce;
		int bit;

		ce = intel_context_create(engine);
		if (IS_ERR(ce)) {
			err = PTR_ERR(ce);
			break;
		}

		i915_vm_put(ce->vm);
		ce->vm = i915_vm_get(&ppgtt->vm);

		for_i915_gem_ww(&ww, err, true)
			err = intel_context_pin_ww(ce, &ww);
		if (err == 0) {
			for_each_set_bit(bit,
					 (unsigned long *)&INTEL_INFO(gt->i915)->page_sizes,
					 BITS_PER_TYPE(INTEL_INFO(gt->i915)->page_sizes)) {
				int len;

				/* sanitycheck the semaphore wake up */
				err = pte_tlbinv(ce, va, va,
						 BIT_ULL(bit),
						 NULL, SZ_4K,
						 &prng);
				if (err)
					goto err_unpin;

				for (len = 2; len <= INTEL_INFO(gt->i915)->ppgtt_size; len *= 2) {
					err = pte_tlbinv(ce, va, vb,
							BIT_ULL(bit),
							tlbinv,
							BIT_ULL(len),
							&prng);
					if (err)
						goto err_unpin;
				}

				if (len != INTEL_INFO(gt->i915)->ppgtt_size) {
					len = INTEL_INFO(gt->i915)->ppgtt_size;
					err = pte_tlbinv(ce, va, vb,
							BIT_ULL(bit),
							tlbinv,
							BIT_ULL(len),
							&prng);
					if (err)
						goto err_unpin;
				}
			}
err_unpin:
			intel_context_unpin(ce);
		}

		intel_context_put(ce);
		if (err)
			break;
	}

	if (igt_flush_test(gt->i915))
		err = -EIO;

out_vm:
	i915_vm_put(&ppgtt->vm);
out_b:
	i915_gem_object_put(B);
out_a:
	i915_gem_object_put(A);
	list_for_each_entry_safe(A, B, &discard, st_link)
		i915_gem_object_put(A);
	return err;
}

static void tlbinv_full(struct i915_address_space *vm, u64 addr, u64 length)
{
	intel_gt_invalidate_tlb_full(vm->gt, intel_gt_tlb_seqno(vm->gt) | 1);
}

static int invalidate_full(void *arg)
{
	struct intel_gt *gt = arg;
	int err;

	if (GRAPHICS_VER(gt->i915) < 8)
		return 0; /* TLB invalidate not implemented */

	err = mem_tlbinv(gt, create_smem, tlbinv_full);
	if (err == 0)
		err = mem_tlbinv(gt, create_lmem, tlbinv_full);
	if (err == -ENODEV || err == -ENXIO)
		err = 0;

	return err;
}

static void tlbinv_range(struct i915_address_space *vm, u64 addr, u64 length)
{
	if (!intel_gt_invalidate_tlb_range(vm->gt, vm, addr, length))
		pr_err("range invalidate failed\n");
}

static bool has_invalidate_range(struct intel_gt *gt)
{
	struct i915_address_space *vm = gt->vm;
	intel_wakeref_t wf;
	bool result = false;

	with_intel_gt_pm(gt, wf)
		result = intel_gt_invalidate_tlb_range(gt, vm, 0, vm->total);

	return result;
}

static int invalidate_range(void *arg)
{
	struct intel_gt *gt = arg;
	int err;

	if (!has_invalidate_range(gt))
		return 0;

	err = mem_tlbinv(gt, create_smem, tlbinv_range);
	if (err == 0)
		err = mem_tlbinv(gt, create_lmem, tlbinv_range);
	if (err == -ENODEV || err == -ENXIO)
		err = 0;

	return err;
}

int intel_tlb_live_selftests(struct drm_i915_private *i915)
{
	static const struct i915_subtest tests[] = {
		SUBTEST(invalidate_full),
		SUBTEST(invalidate_range),
	};
	struct intel_gt *gt;
	unsigned int i;

	for_each_gt(gt, i915, i) {
		int err;

		if (intel_gt_is_wedged(gt))
			continue;

		err = intel_gt_live_subtests(tests, gt);
		if (err)
			return err;
	}

	return 0;
}

static int tlb_page_size(void *arg)
{
	int start, size, offset;

	for (start = 0; start < 57; start++) {
		for (size = 0; size <= 57 - start; size++) {
			for (offset = 0; offset <= size; offset++) {
				u64 len = BIT(size);
				u64 addr = BIT(start) + len - BIT(offset);
				u64 expected_start = addr;
				u64 expected_end = addr + len - 1;
				int err = 0;

				if (addr + len < addr)
					continue;

				len = tlb_page_selective_size(&addr, len);
				if (addr > expected_start) {
					pr_err("(start:%d, size:%d, offset:%d, range:[%llx, %llx]) invalidate range:[%llx + %llx] after start:%llx\n",
					       start, size, offset,
					       expected_start, expected_end,
					       addr, len,
					       expected_start);
					err = -EINVAL;
				}

				if (addr + len < expected_end) {
					pr_err("(start:%d, size:%d, offset:%d, range:[%llx, %llx]) invalidate range:[%llx + %llx] before end:%llx\n",
					       start, size, offset,
					       expected_start, expected_end,
					       addr, len,
					       expected_end);
					err = -EINVAL;
				}

				if (err)
					return err;
			}
		}
	}

	return 0;
}

int intel_tlb_mock_selftests(void)
{
	static const struct i915_subtest tests[] = {
		SUBTEST(tlb_page_size),
	};

	return i915_subtests(tests, NULL);
}
