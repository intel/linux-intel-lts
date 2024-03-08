// SPDX-License-Identifier: MIT
/*
 * Copyright © 2019 Intel Corporation
 */

#include <linux/dma-fence-array.h>

#include <uapi/drm/i915_drm.h>

#include "gt/gen8_engine_cs.h"
#include "gt/gen8_ppgtt.h"
#include "gt/intel_context.h"
#include "gt/intel_engine_pm.h"
#include "gt/intel_engine_regs.h"
#include "gt/intel_gpu_commands.h"
#include "gt/intel_gt.h"
#include "gt/intel_gt_clock_utils.h"
#include "gt/intel_gt_pm.h"
#include "gt/intel_lrc.h"
#include "gt/intel_ring.h"
#include "gt/intel_rps.h"

#include "i915_driver.h"
#include "i915_gem_internal.h"
#include "i915_gem_lmem.h"
#include "i915_gem_mman.h"
#include "i915_gem_region.h"
#include "i915_memcpy.h"
#include "i915_sw_fence.h"
#include "i915_sw_fence_work.h"
#include "intel_memory_region.h"

static inline bool use_flat_ccs(const struct intel_gt *gt)
{
	/* If the device is wedged, [stale] indirect CCS is inaccessible */
	return HAS_FLAT_CCS(gt->i915) && !intel_gt_is_wedged(gt);
}

static bool object_needs_flat_ccs(const struct drm_i915_gem_object *obj)
{
	if (!(obj->flags & I915_BO_ALLOC_USER))
		return false;

	if (obj->memory_mask & BIT(INTEL_REGION_SMEM))
		return false;

	return use_flat_ccs(obj->mm.region.mem->gt);
}

static int block_wait(struct i915_buddy_block *block)
{
	struct dma_fence *f;
	int err = 0;

	f = i915_active_fence_get(&block->active);
	if (unlikely(f)) {
		if (dma_fence_is_i915(f))
			i915_request_set_priority(to_request(f),
						  I915_PRIORITY_MAX);

		if (dma_fence_wait(f, true) < 0)
			err = -EINTR;

		dma_fence_put(f);
	}

	return err;
}

struct await_fences {
	struct dma_fence_work base;
	struct await_chain {
		struct i915_sw_dma_fence_cb cb;
		struct i915_dependency dep;
	} cb[];
};

static void __await_fence(struct i915_request *rq,
			  struct dma_fence *f,
			  struct i915_sw_dma_fence_cb *cb,
			  struct i915_dependency *dep)
{
	if (!__i915_sw_fence_await_dma_fence(&rq->submit, f, cb))
		return;

	if (dma_fence_is_i915(f))
		__i915_sched_node_add_dependency(&rq->sched,
						 &to_request(f)->sched,
						 dep,
						 0);
}

static void __await_chain(struct i915_request *rq,
			  struct dma_fence *f,
			  struct await_chain *c)
{
	__await_fence(rq, f, &c->cb, &c->dep);
}

static void await_chain(struct await_fences *a, struct dma_fence *f, int slot)
{
	__await_chain(&a->base.rq, f, &a->cb[slot]);
}

static const struct dma_fence_work_ops await_ops = {
	.name = "await",
};

static const struct dma_fence_work_ops await_noerror_ops = {
	.name = "await",
	.no_error_propagation = true,
};

static struct await_fences *
await_create(struct i915_sched_engine *se, int count, unsigned int flags)
#define AWAIT_NO_ERROR BIT(0)
{
	struct await_fences *a;

	a = kmalloc(struct_size(a, cb, count), I915_GFP_ALLOW_FAIL);
	if (!a)
		return a;

	dma_fence_work_init(&a->base,
			    flags & AWAIT_NO_ERROR ? &await_noerror_ops : &await_ops,
			    se);

	return a;
}

static int
await_migration(struct dma_fence_work *wrk, struct drm_i915_gem_object *obj)
{
	struct dma_fence *f;

	f = i915_active_fence_get_or_error(&obj->mm.migrate);
	if (IS_ERR_OR_NULL(f))
		return PTR_ERR(f);

	dma_fence_work_chain(wrk, f);
	dma_fence_put(f);
	return 0;
}

static u32 *emit_timestamp(struct i915_request *rq, u32 *cs, int gpr)
{
	*cs++ = MI_LOAD_REGISTER_REG | MI_LRR_SOURCE_CS_MMIO | MI_LRI_LRM_CS_MMIO;
	*cs++ = i915_mmio_reg_offset(RING_TIMESTAMP_UDW(0));
	*cs++ = i915_mmio_reg_offset(GEN8_RING_CS_GPR_UDW(0, gpr));

	*cs++ = MI_LOAD_REGISTER_REG | MI_LRR_SOURCE_CS_MMIO | MI_LRI_LRM_CS_MMIO;
	*cs++ = i915_mmio_reg_offset(RING_TIMESTAMP(0));
	*cs++ = i915_mmio_reg_offset(GEN8_RING_CS_GPR(0, gpr));

	return cs;
}

static int emit_start_timestamp(struct i915_request *rq)
{
	u32 *cs;

	cs = intel_ring_begin(rq, 6);
	if (IS_ERR(cs))
		return PTR_ERR(cs);

	cs = emit_timestamp(rq, cs, 0);

	intel_ring_advance(rq, cs);
	return 0;
}

static u32 *emit_mem_fence(struct i915_request *rq, u32 *cs)
{
	u32 scratch = i915_ggtt_offset(rq->engine->gt->scratch);

	return gen8_emit_ggtt_write(cs, 0, scratch, 0);
}

static int emit_update_counters(struct i915_request *rq, u64 size, int idx)
{
	u32 global = i915_ggtt_offset(rq->engine->gt->counters.vma);
	u32 *cs;

	cs = intel_ring_begin(rq, 20 + (size ? 8 : 0));
	if (IS_ERR(cs))
		return PTR_ERR(cs);

	cs = emit_mem_fence(rq, cs);
	cs = emit_timestamp(rq, cs, 1);

	/* Compute elapsed time (end - start) */
	*cs++ = MI_MATH(4);
	*cs++ = MI_MATH_LOAD(MI_MATH_REG_SRCA, MI_MATH_REG(1));
	*cs++ = MI_MATH_LOAD(MI_MATH_REG_SRCB, MI_MATH_REG(0));
	*cs++ = MI_MATH_SUB;
	*cs++ = MI_MATH_STORE(MI_MATH_REG(0), MI_MATH_REG_ACCU);

	/* Increment cycle counters */
	*cs++ = MI_ATOMIC | MI_ATOMIC_ADD64 | MI_ATOMIC64 | MI_USE_GGTT;
	*cs++ = global + idx * sizeof(u64);
	*cs++ = 0;

	if (size) { /* Increment byte counters */
		*cs++ = MI_LOAD_REGISTER_IMM(2) | MI_LRI_LRM_CS_MMIO;
		*cs++ = i915_mmio_reg_offset(GEN8_RING_CS_GPR(0, 0));
		*cs++ = lower_32_bits(size);
		*cs++ = i915_mmio_reg_offset(GEN8_RING_CS_GPR_UDW(0, 0));
		*cs++ = upper_32_bits(size);

		*cs++ = MI_ATOMIC | MI_ATOMIC_ADD64 | MI_ATOMIC64 | MI_USE_GGTT;
		*cs++ = global + (idx + 1) * sizeof(u64);
		*cs++ = 0;
	}

	*cs++ = MI_ARB_CHECK;
	*cs++ = MI_NOOP;

	intel_ring_advance(rq, cs);
	return 0;
}

static struct intel_context *
__get_blitter_context(const struct intel_gt *gt, int idx)
{
	return gt->engine[idx] ? gt->engine[idx]->blitter_context : NULL;
}

static struct intel_context *
get_blitter_context(const struct intel_gt *gt, int idx)
{
	struct intel_context *ce;

	if (intel_gt_is_wedged(gt) || gt->suspend)
		return NULL;

	ce = __get_blitter_context(gt, idx);
	if (!ce || !ce->private)
		return NULL;

	return ce;
}

static struct intel_context *get_clear_alloc_context(const struct intel_gt *gt)
{
	/*
	 * Use for higher priority clears along the user critical path.
	 *
	 * We use distinct contexts (where available) to split the work
	 * between background opportunistic clears and those clears that
	 * are required for immediate use. This should allow us to
	 * reschedule that work ahead of the background clears.
	 */
	return get_blitter_context(gt, BCS0);
}

static struct intel_context *get_clear_free_context(const struct intel_gt *gt)
{
	/* Use for lower priority background clears */
	return get_blitter_context(gt, gt->rsvd_bcs);
}

static struct intel_context *get_clear_fault_context(const struct intel_gt *gt)
{
	return get_clear_free_context(gt);
}

static struct intel_context *get_clear_idle_context(const struct intel_gt *gt)
{
	/* On idle, we should see no contention and can use any engine */
	return get_blitter_context(gt, BCS0);
}

static struct intel_context *get_swapin_context(const struct intel_gt *gt)
{
	return get_blitter_context(gt, gt->rsvd_bcs);
}

static struct intel_context *get_swapout_context(const struct intel_gt *gt)
{
	return get_blitter_context(gt, gt->rsvd_bcs);
}

static struct i915_request *
chain_request(struct i915_request *rq, struct i915_request *chain)
{
	struct intel_timeline *tl = rq->context->timeline;
	int prio;

	GEM_BUG_ON(rq == chain);

	/*
	 * Hold the request until the next is chained. We need
	 * a complete chain in order to propagate any error to the
	 * final fence, and into the obj->mm.migrate. If we drop
	 * the error at any point (due to a completed request),
	 * then we may continue to use the uninitialised contents.
	 */

	lockdep_assert_held(&tl->mutex);
	lockdep_unpin_lock(&tl->mutex, rq->cookie);

	i915_sw_fence_await(&rq->submit);
	i915_request_get(rq);

	prio = I915_PRIORITY_MIN; /* run in the background */
	if (rq->engine->instance) /* isolated copy engine */
		prio = I915_PRIORITY_MAX; /* guc: global prio across engines! */

	trace_i915_request_add(rq);
	__i915_request_commit(rq);
	__i915_request_queue(rq, prio);

	if (chain) {
		i915_sw_fence_complete(&chain->submit);
		i915_request_put(chain);
	}

	return rq;
}

static bool __submit_request(const struct i915_request *rq, unsigned int pkt)
{
	const struct intel_ring *ring = rq->ring;

	pkt += ring->size >> 2; /* throttle the ring up into ~4 requests */

	/* Will adding another operation cause the request to overflow? */
	return intel_ring_direction(ring, ring->emit + pkt, rq->head) < 0;
}

static bool
submit_request(struct i915_request *rq,
	       struct i915_request **chain,
	       unsigned int pkt)
{
	if (!__submit_request(rq, pkt) && rq->ring->space > pkt)
		return false;

	*chain = chain_request(rq, *chain);
	return true;
}

void __iomem *
i915_gem_object_lmem_io_map_page_atomic(struct drm_i915_gem_object *obj,
					unsigned long n)
{
	struct intel_memory_region *mem = obj->mm.region.mem;
	resource_size_t offset;

	offset = i915_gem_object_get_dma_address(obj, n);
	offset -= mem->region.start;

	return io_mapping_map_atomic_wc(&mem->iomap, offset);
}

void __iomem *
i915_gem_object_lmem_io_map(struct drm_i915_gem_object *obj,
			    unsigned long n,
			    unsigned long size)
{
	struct intel_memory_region *mem = obj->mm.region.mem;
	struct scatterlist *sg;
	resource_size_t offset;
	unsigned int pfn;

	sg = i915_gem_object_get_sg_dma(obj, n, &pfn);
	GEM_BUG_ON(size > sg_dma_len(sg) - (pfn << PAGE_SHIFT));

	offset = sg_dma_address(sg) + (pfn << PAGE_SHIFT);
	offset -= mem->region.start;

	return io_mapping_map_wc(&mem->iomap, offset, size);
}

unsigned long i915_gem_object_lmem_offset(struct drm_i915_gem_object *obj)
{
	GEM_BUG_ON(!(obj->flags & I915_BO_ALLOC_CONTIGUOUS));
	return i915_gem_object_get_dma_address(obj, 0);
}

/**
 * i915_gem_object_validates_to_lmem - Whether the object is resident in
 * lmem when pages are present.
 * @obj: The object to check.
 *
 * Migratable objects residency may change from under us if the object is
 * not pinned or locked. This function is intended to be used to check whether
 * the object can only reside in lmem when pages are present.
 *
 * Return: Whether the object is always resident in lmem when pages are
 * present.
 */
bool i915_gem_object_validates_to_lmem(struct drm_i915_gem_object *obj)
{
	struct intel_memory_region *mr = READ_ONCE(obj->mm.region.mem);

	return !i915_gem_object_migratable(obj) &&
		mr && (mr->type == INTEL_MEMORY_LOCAL ||
		       mr->type == INTEL_MEMORY_STOLEN);
}

/**
 * i915_gem_object_is_lmem - Whether the object is resident in
 * lmem
 * @obj: The object to check.
 *
 * Even if an object is allowed to migrate and change memory region,
 * this function checks whether it will always be present in lmem when
 * valid *or* if that's not the case, whether it's currently resident in lmem.
 * For migratable and evictable objects, the latter only makes sense when
 * the object is locked.
 *
 * Return: Whether the object migratable but resident in lmem, or not
 * migratable and will be present in lmem when valid.
 */
bool i915_gem_object_is_lmem(const struct drm_i915_gem_object *obj)
{
	struct intel_memory_region *mr = READ_ONCE(obj->mm.region.mem);
#if 0
#ifdef CONFIG_LOCKDEP
	if (i915_gem_object_migratable(obj) &&
	    i915_gem_object_evictable(obj))
		assert_object_held(obj);
#endif
#endif
	return mr && (mr->type == INTEL_MEMORY_LOCAL ||
		      mr->type == INTEL_MEMORY_STOLEN);
}

struct drm_i915_gem_object *
i915_gem_object_create_lmem_from_data(struct intel_memory_region *region,
				      const void *data, size_t size)
{
	struct drm_i915_gem_object *obj;
	void *map;

	obj = i915_gem_object_create_region(region,
					    round_up(size, PAGE_SIZE),
					    I915_BO_ALLOC_CONTIGUOUS);
	if (IS_ERR(obj))
		return obj;

	map = i915_gem_object_pin_map_unlocked(obj, I915_MAP_WC);
	if (IS_ERR(map)) {
		i915_gem_object_put(obj);
		return map;
	}

	memcpy(map, data, size);

	i915_gem_object_flush_map(obj);
	__i915_gem_object_release_map(obj);

	return obj;
}

static int
update_active_blocks(struct i915_request *rq,
		     struct i915_buddy_mm *mm,
		     struct i915_buddy_block *block,
		     u64 offset)
{
	do {
		struct i915_buddy_block *prev;
		u64 end;
		int err;

		err = i915_active_fence_set(&block->active, rq);
		if (err < 0)
			return err;

		if (i915_buddy_block_offset(block) <= offset)
			return 0;

		prev = list_prev_entry(block, link);
		end = i915_buddy_block_offset(prev);
		end += i915_buddy_block_size(mm, prev);
		if (end != i915_buddy_block_offset(block))
			return 0;

		block = prev;
	} while (1);
}

static u32 *__emit_flush(u32 *cs, unsigned int flags)
{
	*cs++ = (MI_FLUSH_DW + 1) | flags;
	*cs++ = LRC_PPHWSP_SCRATCH_ADDR;
	*cs++ = 0;
	*cs++ = 0;

	return cs;
}

static int emit_flush(struct i915_request *rq, unsigned int flags)
{
	u32 *cs;

	cs = intel_ring_begin(rq, 4);
	if (IS_ERR(cs))
		return PTR_ERR(cs);

	cs = __emit_flush(cs, flags);
	intel_ring_advance(rq, cs);

	return 0;
}

static int emit_tlb_invalidate(struct i915_request *rq)
{
	return emit_flush(rq,
			  MI_INVALIDATE_TLB |
			  MI_FLUSH_DW_STORE_INDEX |
			  MI_FLUSH_DW_OP_STOREDW);
}

static int
pvc_emit_swap(struct i915_request *rq,
	      u64 lmem_offset, u64 smem_offset, u32 size,
	      bool to_smem)
{
	struct intel_gt *gt = rq->engine->gt;
	u64 dst_offset, src_offset;
	int src_mocs, dst_mocs;
	u32 mocs;
	u32 *cs;

	cs = intel_ring_begin(rq, 10);
	if (IS_ERR(cs))
		return PTR_ERR(cs);

	*cs++ = PVC_MEM_COPY_CMD | MS_MATRIX | (10 - 2);

	*cs++ = PAGE_SIZE - 1;
	*cs++ = (size >> PAGE_SHIFT) - 1;
	*cs++ = PAGE_SIZE - 1;
	*cs++ = PAGE_SIZE - 1;

	dst_offset = lmem_offset;
	src_offset = smem_offset;
	if (to_smem)
		swap(dst_offset, src_offset);
	*cs++ = lower_32_bits(src_offset);
	*cs++ = upper_32_bits(src_offset);
	*cs++ = lower_32_bits(dst_offset);
	*cs++ = upper_32_bits(dst_offset);

	src_mocs = gt->mocs.wb_index;
	dst_mocs = gt->mocs.uc_index;
	if (to_smem)
		swap(src_mocs, dst_mocs);

	mocs = 0;
	mocs |= FIELD_PREP(MC_SRC_MOCS_INDEX_MASK, src_mocs);
	mocs |= FIELD_PREP(MC_DST_MOCS_INDEX_MASK, dst_mocs);
	*cs++ = mocs;

	intel_ring_advance(rq, cs);
	return 0;
}

static int
xy_emit_swap(struct i915_request *rq,
	     u64 lmem_offset, u64 smem_offset, u32 size,
	     bool to_smem)
{
	u64 dst_offset, src_offset;
	u32 *cs;

	dst_offset = lmem_offset;
	src_offset = smem_offset;
	if (to_smem)
		swap(dst_offset, src_offset);

	cs = intel_ring_begin(rq, 10);
	if (IS_ERR(cs))
		return PTR_ERR(cs);

	*cs++ = GEN9_XY_FAST_COPY_BLT_CMD | (10 - 2);
	*cs++ = BLT_DEPTH_32 | PAGE_SIZE;
	*cs++ = 0;
	*cs++ = (size >> PAGE_SHIFT) << 16 | PAGE_SIZE / 4;
	*cs++ = lower_32_bits(dst_offset);
	*cs++ = upper_32_bits(dst_offset);
	*cs++ = 0;
	*cs++ = PAGE_SIZE;
	*cs++ = lower_32_bits(src_offset);
	*cs++ = upper_32_bits(src_offset);

	intel_ring_advance(rq, cs);
	return 0;
}

static int
emit_pte(struct i915_request *rq,
	 struct sgt_iter *pte,
	 u64 pd_offset,
	 u64 encode,
	 int count)
{
	int pkt;
	u32 *cs;

	GEM_BUG_ON(!count);
	GEM_BUG_ON(!pte->sgp);
	GEM_BUG_ON(!pte->dma);

	pkt = 2 * count + 3 * DIV_ROUND_UP(count, 256);
	pkt = ALIGN(pkt, 2);
	cs = intel_ring_begin(rq, pkt);
	if (IS_ERR(cs))
		return PTR_ERR(cs);

	for (pkt = 0; pkt < count; pkt += 256) {
		int len = min(count - pkt, 256);
		u32 *head = cs++, hdr = len;

		*cs++ = lower_32_bits(pd_offset + 8 * pkt);
		*cs++ = upper_32_bits(pd_offset + 8 * pkt);
		while (len--) {
			u64 dma = encode | (pte->dma + pte->curr);

			GEM_BUG_ON(!pte->sgp);
			GEM_BUG_ON(!pte->dma);

			*cs++ = lower_32_bits(dma);
			*cs++ = upper_32_bits(dma);

			pte->curr += SZ_4K;
			if (pte->curr < pte->max)
				continue;

			*pte = __sgt_iter(__sg_next(pte->sgp), true);
			if (unlikely(!pte->dma)) {
				hdr -= len;
				count = 0;
				break;
			}
		}

		*head = MI_STORE_DATA_IMM | REG_BIT(21) | MI_POSTED | (2 * hdr + 1);
	}

	if (offset_in_page(cs) & 4)
		*cs++ = MI_NOOP;

	GEM_BUG_ON(ptrdiff(cs, rq->ring->vaddr) > rq->ring->emit);
	rq->ring->emit = ptrdiff(cs, rq->ring->vaddr);
	intel_ring_advance(rq, cs);
	return 0;
}

static struct lmem_iter {
	struct i915_buddy_mm *mm;
	struct list_head *blocks;
	struct i915_buddy_block *block;
	u64 curr, dma, max;
} __lmem_iter(struct i915_buddy_mm *mm, struct list_head *blocks)
{
	struct i915_buddy_block *block =
		list_first_entry(blocks, struct i915_buddy_block, link);

	return (struct lmem_iter){
		.mm = mm,
		.blocks = blocks,
		.block = block,
		.max = i915_buddy_block_size(mm, block),
		.dma = i915_buddy_block_offset(block),
	};
}

static int __lmem_iter_next(struct lmem_iter *it, struct i915_request *rq)
{
	it->block = list_next_entry(it->block, link);
	if (unlikely(list_is_head(&it->block->link, it->blocks)))
		return 0;

	it->curr = 0;
	it->max = i915_buddy_block_size(it->mm, it->block);
	it->dma = i915_buddy_block_offset(it->block);

	return i915_active_fence_set(&it->block->active, rq) ?: 1;
}

static int
emit_lmem_pte(struct i915_request *rq,
	      struct lmem_iter *pte,
	      u64 pd_offset,
	      u64 encode,
	      int count,
	      int *length)
{
	int err = 0;
	int pkt;
	u32 *cs;

	GEM_BUG_ON(!IS_ALIGNED(pd_offset, SZ_4K));
	GEM_BUG_ON(list_is_head(&pte->block->link, pte->blocks));

	pkt = 2 * count + 3 * DIV_ROUND_UP(count, 256);
	pkt = ALIGN(pkt, 2);
	cs = intel_ring_begin(rq, pkt);
	if (IS_ERR(cs))
		return PTR_ERR(cs);

	*length = 0;

	for (pkt = 0; pkt < count; pkt += 256) {
		int len = min(count - pkt, 256);
		u32 *head = cs++, hdr = len;

		*cs++ = lower_32_bits(pd_offset + 8 * pkt);
		*cs++ = upper_32_bits(pd_offset + 8 * pkt);
		while (len--) {
			u64 dma = encode | (pte->dma + pte->curr);

			*cs++ = lower_32_bits(dma);
			*cs++ = upper_32_bits(dma);

			pte->curr += SZ_4K;
			if (pte->curr < pte->max)
				continue;

			err = __lmem_iter_next(pte, rq);
			if (err <= 0) {
				hdr -= len;
				count = 0;
				break;
			}
		}

		*length += hdr;
		*head = MI_STORE_DATA_IMM | REG_BIT(21) | MI_POSTED | (2 * hdr + 1);
	}

	if (offset_in_page(cs) & 4)
		*cs++ = MI_NOOP;

	GEM_BUG_ON(ptrdiff(cs, rq->ring->vaddr) > rq->ring->emit);
	rq->ring->emit = ptrdiff(cs, rq->ring->vaddr);
	intel_ring_advance(rq, cs);
	return err;
}

static u32 ccs_direction(bool to_smem)
{
	u32 src_access, dst_access;

	src_access = DIRECT_ACCESS;
	dst_access = INDIRECT_ACCESS;
	if (to_smem)
		swap(dst_access, src_access);

	return src_access << SRC_ACCESS_TYPE_SHIFT | dst_access << DST_ACCESS_TYPE_SHIFT;
}

static int num_ccs_blocks(unsigned int size)
{
	GEM_BUG_ON(!IS_ALIGNED(size, SZ_64K));
	return size >> 16;
}

static int
ccs_emit_swap(struct i915_request *rq,
	      u64 lmem_offset, u64 smem_offset, u32 size,
	      bool to_smem)
{
	const u32 mocs = REG_FIELD_PREP(XY_CSC_BLT_MOCS_INDEX_MASK_XEHP,
					rq->engine->gt->mocs.uc_index);
	u64 dst_offset, src_offset;
	u32 *cs;

	GEM_BUG_ON(!size);
	GEM_BUG_ON(size > SZ_64M);

	cs = intel_ring_begin(rq, 6);
	if (IS_ERR(cs))
		return PTR_ERR(cs);

	*cs++ = XY_CTRL_SURF_COPY_BLT |
		REG_FIELD_PREP(CCS_SIZE_MASK_XEHP, num_ccs_blocks(size) - 1) |
		ccs_direction(to_smem);

	dst_offset = lmem_offset;
	src_offset = smem_offset;
	if (to_smem)
		swap(dst_offset, src_offset);

	*cs++ = lower_32_bits(src_offset);
	*cs++ = upper_32_bits(src_offset) | mocs;
	*cs++ = lower_32_bits(dst_offset);
	*cs++ = upper_32_bits(dst_offset) | mocs;

	*cs++ = MI_NOOP;

	intel_ring_advance(rq, cs);
	return emit_flush(rq, MI_FLUSH_DW_LLC | MI_FLUSH_DW_CCS);
}

static unsigned int pte_pkt(u32 len)
{
	return 2 * len + 3 * DIV_ROUND_UP(len, 256);
}

static unsigned int swap_pkt(u32 len)
{
	unsigned int pkt = SZ_4K;

	pkt += pte_pkt(len >> PAGE_SHIFT);

	return pkt << 2;
}

static unsigned int ccs_pkt(u32 len)
{
	unsigned int pkt = SZ_4K;

	pkt += pte_pkt(len);
	len = ALIGN(len, 256) >> 8;
	pkt += pte_pkt(len);

	return pkt << 2;
}

static int
swap_blt(struct intel_context *ce,
	 struct drm_i915_gem_object *lmem,
	 struct i915_buddy_mm *mm,
	 struct list_head *blocks,
	 struct drm_i915_gem_object *smem,
	 bool to_smem,
	 bool throttle,
	 struct i915_request **out)
{
	const bool use_pvc_memcpy = HAS_LINK_COPY_ENGINES(ce->engine->i915);
	const bool use_flat_ccs = !use_pvc_memcpy && object_needs_flat_ccs(lmem);
	const struct intel_migrate_window *w = ce->private;
	const u64 encode = ce->vm->pte_encode(0, smem->pat_index, 0);
	u64 pte_window, pte_end, pd_offset = 0;
	const u32 step = w->swap_chunk;
	struct i915_buddy_block *block;
	u64 remain = lmem->base.size;
	struct i915_request *rq;
	struct sgt_iter it_smem;
	int err = 0;

	GEM_BUG_ON(ce->ring->size < SZ_256K);
	GEM_BUG_ON(ce->vm != ce->engine->gt->vm);
	GEM_BUG_ON(!drm_mm_node_allocated(&ce->engine->gt->flat));
	GEM_BUG_ON(!IS_ALIGNED(step, PAGE_SIZE));

	if (throttle) { /* fallback to CPU to avoid heavy contention */
		err = intel_context_throttle(ce, 0);
		if (err)
			return err;
	}

	GEM_BUG_ON(smem->base.size < remain);
	it_smem = __sgt_iter(smem->mm.pages->sgl, true);
	GEM_BUG_ON(!it_smem.dma);

	mutex_lock(&ce->timeline->mutex);
	intel_context_enter(ce);

	rq = i915_request_create_locked(ce, I915_GFP_ALLOW_FAIL);
	if (IS_ERR(rq)) {
		err = PTR_ERR(rq);
		goto exit;
	}

	pte_end = w->node.start + w->node.size;
	pte_window = pte_end;

	list_for_each_entry(block, blocks, link) {
		u64 sz = i915_buddy_block_size(mm, block);
		u64 offset = i915_buddy_block_offset(block);

		do { /* re-coalesce split blocks */
			struct i915_buddy_block *next =
				list_next_entry(block, link);

			err = i915_active_fence_set(&block->active, rq);
			if (err < 0)
				goto skip;

			if (list_is_head(&next->link, blocks))
				break;

			if (i915_buddy_block_offset(next) != offset + sz)
				break;

			sz += i915_buddy_block_size(mm, next);
			block = next;
		} while (1);

		sz = min(remain, sz);
		remain -= sz;

		do {
			u32 length = min_t(u64, sz, step);

			GEM_BUG_ON(offset < ce->engine->gt->flat.start);
			GEM_BUG_ON(offset + length > ce->engine->gt->flat.start + ce->engine->gt->flat.size);
			GEM_BUG_ON(!IS_ALIGNED(length, PAGE_SHIFT));

			if (submit_request(rq, out, swap_pkt(length))) {
				rq = i915_request_create_locked(ce, I915_GFP_ALLOW_FAIL);
				if (IS_ERR(rq)) {
					err = PTR_ERR(rq);
					goto exit;
				}

				err = update_active_blocks(rq,
							   mm, block,
							   offset);
				if (err)
					goto skip;
			}

			err = emit_start_timestamp(rq);
			if (err)
				goto skip;

			if (pte_window + length > pte_end) {
				pd_offset = w->pd_offset;
				pte_window = w->node.start;
				GEM_BUG_ON(pte_window + length > pte_end);

				err = emit_tlb_invalidate(rq);
				if (err)
					goto skip;
			}

			err = emit_pte(rq, &it_smem,
				       pd_offset, encode,
				       length >> PAGE_SHIFT);
			if (err)
				goto skip;

			if (use_pvc_memcpy)
				err = pvc_emit_swap(rq,
						    offset, pte_window, length,
						    to_smem);
			else
				err = xy_emit_swap(rq,
						   offset, pte_window, length,
						   to_smem);
			if (err)
				goto skip;

			err = emit_update_counters(rq, length, INTEL_GT_SWAPIN_CYCLES + 2 * to_smem);
			if (err)
				goto skip;

			pte_window += length;
			pd_offset += length >> PAGE_SHIFT << 3;
			GEM_BUG_ON(pte_window > pte_end);
			GEM_BUG_ON(pd_offset > w->pd_offset + sg_dma_len(w->obj->mm.pages->sgl));

			offset += length;
			sz -= length;
		} while (sz);
	}
	GEM_BUG_ON(remain);

	if (use_flat_ccs) {
		struct lmem_iter it_lmem = __lmem_iter(mm, blocks);
		const u64 lmem_encode =
			ce->vm->pte_encode(0, i915_gem_get_pat_index(ce->engine->i915, I915_CACHING_NONE), PTE_LM);
		const int count =
			ALIGN(min_t(u32, step, w->node.size / 2) >> PAGE_SHIFT,
			      512);
		const u64 ccs_pte_offset = w->node.size / 2;
		const u64 ccs_pd_offset = ccs_pte_offset >> PAGE_SHIFT << 3;

		pd_offset = w->pd_offset;
		pte_end = w->node.start + ccs_pte_offset;
		pte_window = pte_end;
		remain = lmem->base.size >> PAGE_SHIFT;

		err = i915_active_fence_set(&it_lmem.block->active, rq);
		if (err < 0)
			goto skip;

		do {
			u64 lmem_window;
			int length;

			if (submit_request(rq, out, ccs_pkt(count))) {
				struct i915_buddy_block *block = it_lmem.block;

				rq = i915_request_create_locked(ce, I915_GFP_ALLOW_FAIL);
				if (IS_ERR(rq)) {
					err = PTR_ERR(rq);
					goto exit;
				}

				err = i915_active_fence_set(&block->active, rq);
				if (err < 0)
					goto skip;
			}

			err = emit_start_timestamp(rq);
			if (err)
				goto skip;

			if (pte_window + (count << PAGE_SHIFT) > pte_end) {
				pd_offset = w->pd_offset;
				pte_window = w->node.start;

				err = emit_tlb_invalidate(rq);
				if (err)
					goto skip;
			}
			GEM_BUG_ON(pd_offset - w->pd_offset > ccs_pd_offset);

			/* XY_CSC needs 1M contiguous blocks */
			if (IS_ALIGNED(it_lmem.dma + it_lmem.curr, SZ_1M) &&
			    it_lmem.max - it_lmem.curr > SZ_1M) {
				length = round_down(it_lmem.max - it_lmem.curr, SZ_1M) >> PAGE_SHIFT;
				length = min(length, count);
				if (!length)
					goto lmem_pte;

				lmem_window = it_lmem.dma + it_lmem.curr;
				it_lmem.curr += length << PAGE_SHIFT;
				GEM_BUG_ON(it_lmem.curr > it_lmem.max);

				if (it_lmem.curr == it_lmem.max) {
					err = __lmem_iter_next(&it_lmem, rq);
					if (err < 0)
						goto skip;
				}
			} else {
lmem_pte:
				err = emit_lmem_pte(rq, &it_lmem,
						    pd_offset + ccs_pd_offset,
						    lmem_encode,
						    count, &length);
				if (err < 0)
					goto skip;

				lmem_window = pte_window + ccs_pte_offset;
			}

			length = min_t(u64, length, remain);
			remain -= length;

			err = emit_pte(rq, &it_smem,
				       pd_offset, encode,
				       ALIGN(length, 256) >> 8);
			if (err)
				goto skip;

			err = ccs_emit_swap(rq,
					    lmem_window,
					    pte_window,
					    length << PAGE_SHIFT,
					    to_smem);
			if (err)
				goto skip;

			err = emit_update_counters(rq, 0, INTEL_GT_SWAPIN_CYCLES + 2 * to_smem);
			if (err)
				goto skip;

			pd_offset += length << 3;
			pte_window += length << PAGE_SHIFT;
			GEM_BUG_ON(pte_window > pte_end);
		} while (!list_is_head(&it_lmem.block->link, blocks));
	}
skip:
	*out = chain_request(rq, *out);
exit:
	if (*out) {
		struct i915_vma *ref;

		ref = i915_vma_instance(smem, ce->vm, NULL);
		if (!IS_ERR(ref))
			i915_active_add_request(&ref->active, *out);

		i915_gem_object_migrate_prepare(smem, &(*out)->fence);
	}

	intel_context_exit(ce);
	mutex_unlock(&ce->timeline->mutex);

	return err;
}

static void
swap_cpu_block(struct intel_memory_region *mem,
	       struct i915_buddy_block *block,
	       struct sgt_iter *it,
	       bool to_smem)
{
	u64 length = i915_buddy_block_size(&mem->mm, block);
	void __iomem *iova;
	dma_addr_t daddr;
	u64 offset;

	GEM_BUG_ON(!it->sgp);

	daddr = i915_buddy_block_offset(block);
	daddr -= mem->region.start;

	iova = io_mapping_map_wc(&mem->iomap, daddr, length);
	for (offset = 0; offset < length; offset += PAGE_SIZE) {
		struct page *page = pfn_to_page(it->pfn + (it->curr >> PAGE_SHIFT));
		void *va;

		va = kmap_atomic(page);
		if (to_smem)
			i915_memcpy_from_wc(va, iova + offset, PAGE_SIZE);
		else
			memcpy(iova + offset, va, PAGE_SIZE);
		kunmap_atomic(va);
		cond_resched();

		it->curr += PAGE_SIZE;
		if (it->curr >= it->max) {
			*it = __sgt_iter(__sg_next(it->sgp), false);
			if (unlikely(!it->sgp))
				break;
		}
	}
	io_mapping_unmap(iova);
}

struct swap_cpu_work {
	struct dma_fence_work base;
	struct intel_memory_region *mem;
	struct drm_i915_gem_object *smem;
	long count;
	bool dir;
	struct swap_block {
		struct i915_buddy_block *block;
		struct await_chain chain;
	} blocks[];
};

static int swap_cpu_work(struct dma_fence_work *base)
{
	struct swap_cpu_work *wrk = container_of(base, typeof(*wrk), base);
	struct sgt_iter it = __sgt_iter(wrk->smem->mm.pages->sgl, false);
	long n;

	for (n = 0; n < wrk->count; n++)
		swap_cpu_block(wrk->mem, wrk->blocks[n].block, &it, wrk->dir);

	wmb();
	return 0;
}

static void swap_cpu_release(struct dma_fence_work *base)
{
	struct swap_cpu_work *wrk = container_of(base, typeof(*wrk), base);

	i915_gem_object_unpin_pages(wrk->smem);
	i915_gem_object_put(wrk->smem);
	intel_memory_region_put(wrk->mem);
}

static const struct dma_fence_work_ops swap_cpu_ops = {
	.name = "swap",
	.work = swap_cpu_work,
	.release = swap_cpu_release,
};

static int
async_swap(struct drm_i915_gem_object *lmem,
	   struct drm_i915_gem_object *smem,
	   bool to_smem,
	   struct dma_fence_work **out)
{
	struct i915_buddy_block *block;
	struct swap_cpu_work *wrk;
	long count;

	GEM_BUG_ON(!i915_gem_object_has_pinned_pages(smem));

	count = 0;
	list_for_each_entry(block, &lmem->mm.blocks, link)
		count++;

	wrk = kmalloc(struct_size(wrk, blocks, count), I915_GFP_ALLOW_FAIL);
	if (!wrk)
		return -ENOMEM;

	dma_fence_work_init(&wrk->base, &swap_cpu_ops,
			    to_i915(lmem->base.dev)->mm.sched);
	wrk->mem = intel_memory_region_get(lmem->mm.region.mem);
	wrk->smem = i915_gem_object_get(smem);
	__i915_gem_object_pin_pages(wrk->smem);
	wrk->count = count;
	wrk->dir = to_smem;

	count = 0;
	list_for_each_entry(block, &lmem->mm.blocks, link) {
		struct swap_block *b = &wrk->blocks[count++];
		struct dma_fence *f;

		b->block = block;

		f = __i915_active_fence_fetch_set(&block->active,
						  &wrk->base.rq.fence);
		if (!f)
			continue;

		__await_chain(&wrk->base.rq, f, &b->chain);
		dma_fence_put(f);
	}

	i915_gem_object_migrate_prepare(smem, &wrk->base.rq.fence);
	*out = &wrk->base;
	return 0;
}

static void ras_error(struct drm_i915_private *i915, int err)
{
	if (err != -EINTR && err != -ERESTARTSYS)
		i915_silent_driver_error(i915, I915_DRIVER_ERROR_OBJECT_MIGRATION);
}

static int lmem_swapout(struct drm_i915_gem_object *lmem)
{
	struct drm_i915_private *i915 = to_i915(lmem->base.dev);
	const bool use_flat_ccs = object_needs_flat_ccs(lmem);
	struct drm_i915_gem_object *smem;
	struct intel_context *ce;
	int err;

	smem = fetch_and_zero(&lmem->swapto);
	if (smem && smem->mm.madv == __I915_MADV_PURGED)
		i915_gem_object_put(fetch_and_zero(&smem));
	if (!smem) {
		u64 size;

		size = lmem->base.size;
		if (use_flat_ccs)
			size += size >> 8;

		smem = i915_gem_object_create_shmem(i915, size);
		if (IS_ERR(smem))
			return PTR_ERR(smem);

		i915_gem_object_share_resv(lmem, smem);
	}

	assert_object_held(smem);
	smem->mm.madv = I915_MADV_WILLNEED;
	lmem->swapto = smem;

	err = i915_gem_object_pin_pages_sync(smem); /* XXX defer? */
	if (err)
		goto err;

	err = -EIO;
	ce = get_swapout_context(lmem->mm.region.mem->gt);
	if (ce) {
		struct i915_request *rq = NULL;

		err = swap_blt(ce, lmem,
			       &lmem->mm.region.mem->mm, &lmem->mm.blocks,
			       smem, true, !use_flat_ccs /* backoff under load */,
			       &rq);
		if (rq) {
			i915_sw_fence_complete(&rq->submit);
			i915_request_put(rq);
		}
	}
	if (err && !use_flat_ccs) {
		struct dma_fence_work *f = NULL;

		err = async_swap(lmem, smem, true, &f);
		if (f)
			dma_fence_work_commit(f);
	}

	i915_gem_object_unpin_pages(smem);

	if (err) {
err:
		ras_error(i915, err);
		smem->mm.madv = I915_MADV_DONTNEED;
		__i915_gem_object_put_pages(smem);
		i915_gem_object_put(smem);
		lmem->swapto = NULL;
	}

	return err;
}

static inline bool
small_sync_swapin(const struct drm_i915_gem_object *obj,
		  unsigned int flags,
		  const struct intel_context *ce)
{
	/*
	 * Prefer to use the CPU where copying the data directly will take
	 * less time than constructing a request and submitting it to the
	 * GPU to do the same (and waiting synchronously for it).
	 */
	if (!(flags & I915_BO_SYNC_HINT) && intel_context_is_active(ce))
		return obj->base.size <= SZ_64K;
	else
		return obj->base.size <= SZ_8M;
}

struct swap_work {
	struct dma_fence_work base;
	struct await_fences *proxy;
	struct drm_i915_gem_object *lmem;
	struct drm_i915_gem_object *smem;
};

static int swap_work(struct dma_fence_work *base)
{
	struct swap_work *wrk = container_of(base, typeof(*wrk), base);
	const bool use_flat_ccs = object_needs_flat_ccs(wrk->lmem);
	struct drm_i915_gem_object *lmem = wrk->lmem;
	struct drm_i915_gem_object *smem = wrk->smem;
	struct intel_context *ce;
	int err = -EIO;

	GEM_BUG_ON(!i915_gem_object_has_pages(smem));

	ce = get_swapin_context(lmem->mm.region.mem->gt);
	if (ce && !use_flat_ccs && small_sync_swapin(lmem, lmem->flags, ce))
		ce = NULL;
	if (ce) {
		struct i915_request *rq = NULL;

		err = swap_blt(ce, lmem,
			       &lmem->mm.region.mem->mm, &lmem->mm.blocks,
			       smem, false, !use_flat_ccs,
			       &rq);
		if (rq) {
			await_chain(wrk->proxy, &rq->fence, 0);
			i915_gem_object_migrate_prepare(lmem, &rq->fence);
			i915_sw_fence_complete(&rq->submit);
			i915_request_put(rq);
		}
	}
	if (err && !use_flat_ccs) {
		struct dma_fence_work *f = NULL;

		err = async_swap(lmem, smem, false, &f);
		if (f) {
			await_chain(wrk->proxy, &f->rq.fence, 0);
			i915_gem_object_migrate_prepare(lmem, &f->rq.fence);
			dma_fence_work_commit_imm(f);
		}
	}

	return err;
}

static void swap_release(struct dma_fence_work *base)
{
	struct swap_work *wrk = container_of(base, typeof(*wrk), base);
	struct drm_i915_gem_object *smem = wrk->smem;

	i915_gem_object_unpin_pages(smem);
	i915_gem_object_put(smem);
}

static const struct dma_fence_work_ops swap_ops = {
	.name = "swap",
	.work = swap_work,
	.release = swap_release,
};

static int lmem_swapin(struct drm_i915_gem_object *lmem)
{
	struct i915_sched_engine *se = to_i915(lmem->base.dev)->mm.sched;
	struct drm_i915_gem_object *smem = lmem->swapto;
	struct swap_work *sw;
	int err;

	GEM_BUG_ON(smem->mm.madv != I915_MADV_WILLNEED);

	sw = kmalloc(sizeof(*sw), GFP_KERNEL);
	if (!sw)
		return -ENOMEM;

	sw->proxy = await_create(se, 1, 0);
	if (!sw->proxy) {
		err = -ENOMEM;
		goto err;
	}

	dma_fence_work_init(&sw->base, &swap_ops, se);
	sw->lmem = lmem;
	sw->smem = smem;

	err = i915_gem_object_pin_pages(smem);
	if (err)
		goto err;

	err = await_migration(&sw->base, smem);
	if (err)
		goto err_unpin;

	i915_gem_object_migrate_prepare(lmem, &sw->proxy->base.rq.fence);
	i915_gem_object_migrate_prepare(smem, &sw->proxy->base.rq.fence);

	dma_fence_work_chain(&sw->proxy->base, &sw->base.rq.fence);
	dma_fence_work_commit(&sw->proxy->base);

	smem->mm.madv = I915_MADV_DONTNEED;
	lmem->swapto = want_init_on_alloc(0) ? i915_gem_object_get(smem) : NULL;
	i915_gem_object_release_mmap(lmem);

	dma_fence_work_commit_imm_if(&sw->base,
				     lmem->flags & I915_BO_SYNC_HINT);
	return 0;

err_unpin:
	i915_gem_object_unpin_pages(smem);
err:
	kfree(sw->proxy);
	kfree(sw);
	ras_error(to_i915(lmem->base.dev), err);
	return err;
}

static int
emit_ccs_clear(struct i915_request *rq, u64 offset, u32 length)
{
	u32 mocs = REG_FIELD_PREP(XY_CSC_BLT_MOCS_INDEX_MASK_XEHP,
				  rq->engine->gt->mocs.uc_index);
	u64 zero = offset;
	int err;

	err = emit_flush(rq, 0);
	if (err)
		return err;

	do {
		u32 blocks = min_t(u32, length, SZ_64M);
		u32 *cs;

		cs = intel_ring_begin(rq, 6);
		if (IS_ERR(cs))
			return PTR_ERR(cs);

		*cs++ = XY_CTRL_SURF_COPY_BLT |
			DIRECT_ACCESS << SRC_ACCESS_TYPE_SHIFT |
			INDIRECT_ACCESS << DST_ACCESS_TYPE_SHIFT |
			REG_FIELD_PREP(CCS_SIZE_MASK_XEHP,
				       num_ccs_blocks(blocks) - 1);

		*cs++ = lower_32_bits(zero);
		*cs++ = upper_32_bits(zero) | mocs;
		*cs++ = lower_32_bits(offset);
		*cs++ = upper_32_bits(offset) | mocs;
		*cs++ = MI_NOOP;

		intel_ring_advance(rq, cs);

		offset += SZ_64M;
		length -= blocks;
	} while (length);

	return emit_flush(rq, MI_FLUSH_DW_LLC | MI_FLUSH_DW_CCS);
}

#define MAX_PAGE_SHIFT 16ul

static int
pvc_emit_clear(struct i915_request *rq, u64 offset, u32 size, u32 page_shift)
{
	u32 mocs;
	u32 *cs;

	cs = intel_ring_begin(rq, 8);
	if (IS_ERR(cs))
		return PTR_ERR(cs);

	*cs++ = PVC_MEM_SET_CMD | MS_MATRIX | (7 - 2);

	*cs++ = BIT(page_shift) - 1;
	*cs++ = (size >> page_shift) - 1;
	*cs++ = BIT(page_shift) - 1;

	*cs++ = lower_32_bits(offset);
	*cs++ = upper_32_bits(offset);

	mocs = rq->engine->gt->mocs.uc_index;
	mocs = FIELD_PREP(MS_MOCS_INDEX_MASK, mocs);
	*cs++ = mocs;

	*cs++ = MI_NOOP;
	intel_ring_advance(rq, cs);

	return 0;
}

static int
xy_emit_clear(struct i915_request *rq, u64 offset, u32 size, u32 page_shift)
{
	u32 mocs = 0;
	int len;
	u32 *cs;

	GEM_BUG_ON(page_shift > 18); /* max stride */
	GEM_BUG_ON(BIT(page_shift) / 4 > S16_MAX); /* max width */
	GEM_BUG_ON(size >> page_shift > S16_MAX); /* max height */

	len = 11;
	if (GRAPHICS_VER_FULL(rq->engine->i915) >= IP_VER(12, 50)) {
		mocs = rq->engine->gt->mocs.uc_index << 1;
		mocs = FIELD_PREP(XY_FAST_COLOR_BLT_MOCS_MASK, mocs);
		len = 16;
	}

	cs = intel_ring_begin(rq, 16);
	if (IS_ERR(cs))
		return PTR_ERR(cs);

	*cs++ = GEN9_XY_FAST_COLOR_BLT_CMD |
		XY_FAST_COLOR_BLT_DEPTH_32 |
		(len - 2);
	*cs++ = mocs | (BIT(page_shift) - 1);
	*cs++ = 0;
	*cs++ = size >> page_shift << 16 | BIT(page_shift) / 4;
	*cs++ = lower_32_bits(offset);
	*cs++ = upper_32_bits(offset);
	*cs++ = 0;
	*cs++ = 0;
	*cs++ = 0;
	*cs++ = 0;
	*cs++ = 0;
	*cs++ = 0;
	*cs++ = 0;
	*cs++ = 0;
	*cs++ = 0;
	*cs++ = 0;

	intel_ring_advance(rq, cs);
	return 0;
}

static void retire_requests(struct intel_timeline *tl)
{
	struct i915_request *rq, *rn;

	list_for_each_entry_safe(rq, rn, &tl->requests, link)
		if (!i915_request_retire(rq))
			break;
}

static int
clear_blt(struct intel_context *ce,
	  struct drm_i915_gem_object *fence,
	  struct i915_buddy_mm *mm,
	  struct list_head *blocks,
	  int counter,
	  bool dirty,
	  struct i915_request **out)
{
	const bool use_pvc_memset = HAS_LINK_COPY_ENGINES(ce->engine->i915);
	const bool use_ccs_clear =
		!use_pvc_memset && HAS_FLAT_CCS(ce->engine->i915);
	const struct intel_migrate_window *w = ce->private;
	const u32 step = w->clear_chunk;
	struct i915_buddy_block *block;
	struct i915_request *rq;
	int err = 0;

	GEM_BUG_ON(ce->ring->size < SZ_64K);
	GEM_BUG_ON(ce->vm != ce->engine->gt->vm);
	GEM_BUG_ON(!drm_mm_node_allocated(&ce->engine->gt->flat));

	mutex_lock(&ce->timeline->mutex);
	intel_context_enter(ce);

	/* We expect at least one block does require clearing */
	rq = i915_request_create_locked(ce, I915_GFP_ALLOW_FAIL);
	if (IS_ERR(rq)) {
		err = PTR_ERR(rq);
		goto exit;
	}

	list_for_each_entry(block, blocks, link) {
		u64 sz = i915_buddy_block_size(mm, block);
		u64 offset = i915_buddy_block_offset(block);

		if (!dirty && i915_buddy_block_is_clear(block)) {
			struct dma_fence *f = i915_active_fence_get(&block->active);
			if (f) {
				i915_request_await_dma_fence(rq, f);
				dma_fence_put(f);
			}

			continue;
		}

		do { /* re-coalesce split blocks (3.5G => [3G | 1G | .5G]) */
			struct i915_buddy_block *next =
				list_next_entry(block, link);

			err = i915_active_fence_set(&block->active, rq);
			if (err < 0)
				goto skip;

			/*
			 * We track both the clear flag and any persistent
			 * errors from the request stored in block->active.
			 * If the blt request does not complete the clear, we
			 * disregard the flag in i915_buddy_block_is_clear().
			 */
			__i915_buddy_block_set_clear(block);

			if (list_is_head(&next->link, blocks))
				break;

			if (i915_buddy_block_offset(next) != offset + sz)
				break;

			if (!dirty && i915_buddy_block_is_clear(next))
				break; /* skip over the next cleared block */

			sz += i915_buddy_block_size(mm, next);
			block = next;
		} while (1);

		do {
			u32 length = min_t(u64, sz, step);
			int page_shift = min(__ffs(length), MAX_PAGE_SHIFT);

			GEM_BUG_ON(offset < ce->engine->gt->flat.start);
			GEM_BUG_ON(offset + length > ce->engine->gt->flat.start + ce->engine->gt->flat.size);
			GEM_BUG_ON(!IS_ALIGNED(length, PAGE_SHIFT));

			if (length >> page_shift > S16_MAX) {
				page_shift = min(__ffs(sz), MAX_PAGE_SHIFT);
				length = round_down(length, BIT(page_shift));
			}

			if (submit_request(rq, out, SZ_4K)) {
				rq = i915_request_create_locked(ce, I915_GFP_ALLOW_FAIL);
				if (IS_ERR(rq)) {
					err = PTR_ERR(rq);
					goto exit;
				}

				err = update_active_blocks(rq,
							   mm, block,
							   offset);
				if (err)
					goto skip;

				if (fence) {
					err = i915_request_await_object(rq, fence, true);
					if (err)
						goto skip;

					fence = NULL;
				}
			}

			err = emit_start_timestamp(rq);
			if (err)
				goto skip;

			if (use_pvc_memset)
				err = pvc_emit_clear(rq, offset, length, page_shift);
			else
				err = xy_emit_clear(rq, offset, length, page_shift);
			if (err == 0 && use_ccs_clear)
				err = emit_ccs_clear(rq, offset, length);
			if (err)
				goto skip;

			err = emit_update_counters(rq, length, counter);
			if (err)
				goto skip;

			GEM_BUG_ON(__submit_request(rq, 0));
			sz -= length;
			offset += length;
		} while (sz);
	}
skip:
	*out = chain_request(rq, *out);
exit:
	retire_requests(ce->timeline);
	intel_context_exit(ce);
	mutex_unlock(&ce->timeline->mutex);

	return err;
}

static int
clear_cpu(struct intel_memory_region *mem, struct list_head *blocks, u64 value)
{
	const bool is_clear = value == 0 && !use_flat_ccs(mem->gt);
	struct i915_buddy_block *block;

	list_for_each_entry(block, blocks, link) {
		u64 length = i915_buddy_block_size(&mem->mm, block);
		void __iomem *vaddr;
		dma_addr_t daddr;

		if (signal_pending(current) || block_wait(block))
			return -EINTR;

		if (value == 0 && i915_buddy_block_is_clear(block))
			continue;

		daddr = i915_buddy_block_offset(block);
		daddr -= mem->region.start;

		vaddr = io_mapping_map_wc(&mem->iomap, daddr, length);
		memset64((void __force *)vaddr, value, length / sizeof(u64));
		io_mapping_unmap(vaddr);

		i915_buddy_block_set_clear(block, is_clear);
		cond_resched();
	}

	wmb();
	return 0;
}

static inline bool
small_sync_clear(const struct drm_i915_gem_object *obj,
		 unsigned int flags,
		 const struct intel_context *ce)
{
	/* Assume exec + sync latency ~2ms and WC bw of ~4GiB/s */
	if (!(flags & I915_BO_SYNC_HINT) && intel_context_is_active(ce))
		return obj->base.size <= SZ_64K;
	else
		return obj->base.size <= SZ_32M;
}

static inline bool
use_cpu_clear(struct drm_i915_gem_object *obj, unsigned int flags)
{
	/*
	 * Is this object eligible for using the CPU for its clear?
	 *
	 * In some cases, such as before we have initialised the blitter
	 * engine or context, we cannot use the GPU and must use the CPU.
	 *
	 * This may be hinted by the caller (setting the CPU_CLEAR on object
	 * construction) and verified by ourselves.
	 */
	if (!(flags & I915_BO_CPU_CLEAR))
		return false;

	/*
	 * If the object needs to use FLAT_CCS, we have to use the blitter
	 * to clear out the reserved portion of lmem via indirect access.
	 * [We could find the reserved chunks for flat-ccs and do those clears
	 * directly, under most circumstances, but haven't yet.]
	 */
	return !object_needs_flat_ccs(obj);
}

struct clear_work {
	struct dma_fence_work base;
	struct drm_i915_gem_object *lmem;
	struct await_chain cb[];
};

static int clear_work(struct dma_fence_work *base)
{
	struct clear_work *wrk = container_of(base, typeof(*wrk), base);
	struct drm_i915_gem_object *lmem = wrk->lmem;

	return clear_cpu(lmem->mm.region.mem, &lmem->mm.blocks, 0);
}

static const struct dma_fence_work_ops clear_ops = {
	.name = "clear",
	.work = clear_work,
	.no_error_propagation = true,
};

static int async_clear(struct drm_i915_gem_object *obj)
{
	struct i915_buddy_block *block;
	struct clear_work *c;
	int count;

	count = 0;
	list_for_each_entry(block, &obj->mm.blocks, link)
		count++;

	c = kmalloc(struct_size(c, cb, count), I915_GFP_ALLOW_FAIL);
	if (!c)
		return -ENOMEM;

	dma_fence_work_init(&c->base, &clear_ops,
			    to_i915(obj->base.dev)->mm.sched);
	c->lmem = obj;

	count = 0;
	list_for_each_entry(block, &obj->mm.blocks, link) {
		struct dma_fence *f;

		f = i915_active_fence_get(&block->active);
		if (!f)
			continue;

		__await_chain(&c->base.rq, f, &c->cb[count++]);
		dma_fence_put(f);
	}

	i915_gem_object_migrate_prepare(obj, &c->base.rq.fence);
	dma_fence_work_commit_imm_if(&c->base,
				     obj->flags & I915_BO_SYNC_HINT ||
				     obj->base.size <= SZ_64K);

	return 0;
}

static int async_blt(struct drm_i915_gem_object *obj, struct intel_context *ce)
{
	struct intel_memory_region *mem = obj->mm.region.mem;
	struct i915_request *rq = NULL;
	int err;

	err = clear_blt(ce, NULL,
			&mem->mm, &obj->mm.blocks,
			INTEL_GT_CLEAR_ALLOC_CYCLES, false,
			&rq);
	if (rq) {
		i915_gem_object_migrate_prepare(obj, &rq->fence);
		i915_sw_fence_complete(&rq->submit);
		i915_request_put(rq);
	}

	/* recycling dirty memory; proactively clear on release */
	set_bit(INTEL_MEMORY_CLEAR_FREE, &mem->flags);
	return err;
}

static int
blocks_await(struct i915_sched_engine *se,
	     const struct list_head *blocks,
	     unsigned int flags,
	     struct await_fences **out)
{
	struct i915_buddy_block *block;
	struct await_fences *a;
	int count;

	count = 0;
	list_for_each_entry(block, blocks, link)
		count += i915_buddy_block_is_active(block);
	if (!count)
		return 0;

	a = await_create(se, count, flags);
	if (!a)
		return -ENOMEM;

	count = 0;
	list_for_each_entry(block, blocks, link) {
		struct dma_fence *f;

		f = i915_active_fence_get(&block->active);
		if (!f)
			continue;

		await_chain(a, f, count++);
		dma_fence_put(f);
	}
	if (!count) {
		kfree(a);
		return 0;
	}

	*out = a;
	return 0;
}

static int await_blt(struct drm_i915_gem_object *obj, unsigned int flags)
{
	struct await_fences *f = NULL;
	int err;

	err = blocks_await(to_i915(obj->base.dev)->mm.sched,
			   &obj->mm.blocks, flags, &f);
	if (f) {
		i915_gem_object_migrate_prepare(obj, &f->base.rq.fence);
		dma_fence_work_commit(&f->base);
	}

	return err;
}

static inline bool blocks_dirty(const struct list_head *blocks)
{
       struct i915_buddy_block *block;

       list_for_each_entry(block, blocks, link) {
	       if (!i915_buddy_block_is_clear(block))
		       return true;
       }

       return false;
}

static int lmem_clear(struct drm_i915_gem_object *obj)
{
	struct intel_memory_region *mem = obj->mm.region.mem;
	unsigned int flags = obj->flags;
	struct intel_gt *gt = mem->gt;
	struct intel_context *ce;
	intel_wakeref_t wf = 0;
	int err = 0;

	if (flags & I915_BO_SKIP_CLEAR)
		return await_blt(obj, AWAIT_NO_ERROR);

	if (!blocks_dirty(&obj->mm.blocks))
		return await_blt(obj, 0);

	if (!IS_ENABLED(CONFIG_DRM_I915_CHICKEN_ASYNC_GET_PAGES))
		flags |= I915_BO_SYNC_HINT;

	ce = NULL;
	if (flags & (I915_BO_ALLOC_USER | I915_BO_CPU_CLEAR)) {
		if (flags & I915_BO_FAULT_CLEAR)
			ce = get_clear_fault_context(gt);
		else
			ce = get_clear_alloc_context(gt);
		if (!ce ||
		    /* Prefer to use the CPU to avoid sync latency */
		    small_sync_clear(obj, flags, ce) ||
		    /* Saturated? Use the CPU instead (safety valve) */
		    intel_context_throttle(ce, 0))
			flags |= I915_BO_CPU_CLEAR;
	}

	/* Avoid misspending PCI credits between the GT; must use BLT clears */
	if (ce && gt->info.id > 0 && intel_gt_pm_is_awake(gt))
		flags &= ~I915_BO_CPU_CLEAR;

	/* Clear is too small to benefit from waking up the GPU */
	if (ce && obj->base.size < SZ_2M && !(wf = intel_gt_pm_get_if_awake(gt)))
		flags |= I915_BO_CPU_CLEAR;

	if (use_cpu_clear(obj, flags))
		err = async_clear(obj);
	else if (ce)
		err = async_blt(obj, ce);
	else if (flags & I915_BO_CPU_CLEAR)
		err = -EIO; /* clear required use of the blitter */
	else
		err = await_blt(obj, AWAIT_NO_ERROR);

	if (wf)
		intel_gt_pm_put(gt, wf);

	return err;
}

/**
 * i915_gem_object_clear_lmem - Clear local memory using the bliter
 * @obj - the lmem object (and flat-ccs) to be cleared (fill with 0)
 *
 * Clears the lmem backing store of the object, and any implicit flat-ccs
 * storage, reporting an error if the object has no lmem storage or if
 * the blitter is unusable. The blitter operation is queued to HW, with
 * the completion fence stored on the object. If it is required to know
 * the result of clearing the lmem, wait upon i915_gem_object_migrate_sync().
 */
int i915_gem_object_clear_lmem(struct drm_i915_gem_object *obj)
{
	struct intel_context *ce;
	int err;

	if (!i915_gem_object_is_lmem(obj))
		return -EINVAL;

	ce = get_clear_alloc_context(obj->mm.region.mem->gt);
	if (!ce)
		return -EINVAL;

	err = i915_gem_object_lock_interruptible(obj, NULL);
	if (err)
		return err;

	if (obj->mm.pages) {
		struct i915_request *rq = NULL;

		err = clear_blt(ce, obj,
				&obj->mm.region.mem->mm, &obj->mm.blocks,
				INTEL_GT_CLEAR_ALLOC_CYCLES, true,
				&rq);
		if (rq) {
			i915_gem_object_migrate_prepare(obj, &rq->fence);
			i915_sw_fence_complete(&rq->submit);
			i915_request_put(rq);
		}
	}

	i915_gem_object_unlock(obj);

	return err;
}

static bool is_swapped(struct drm_i915_gem_object *obj)
{
	struct drm_i915_gem_object *swp;

	swp = obj->swapto;
	if (!swp)
		return false;

	if (swp->mm.madv != I915_MADV_WILLNEED ||
	    i915_gem_object_migrate_has_error(swp)) {
		i915_gem_object_put(swp);
		obj->swapto = NULL;
		return false;
	}

	return true;
}

static int lmem_get_pages(struct drm_i915_gem_object *obj)
{
	unsigned int page_sizes;
	struct sg_table *pages;
	int err;

	pages = i915_gem_object_get_pages_buddy(obj, &page_sizes);
	if (IS_ERR(pages))
		return PTR_ERR(pages);

	if (is_swapped(obj))
		err = lmem_swapin(obj);
	else
		err = lmem_clear(obj);
	if (err)
		goto err;

	__i915_gem_object_set_pages(obj, pages, page_sizes);
	return 0;

err:
	i915_gem_object_put_pages_buddy(obj, pages, false);
	return err;
}

static bool freed(const struct drm_i915_gem_object *obj)
{
	if (kref_read(&obj->base.refcount) == 0)
		return true;

	if (obj->flags & I915_BO_ALLOC_USER && !i915_gem_object_inuse(obj))
		return true;

	return false;
}

static bool need_swap(const struct drm_i915_gem_object *obj)
{
	if (i915_gem_object_migrate_has_error(obj))
		return false;

	if (i915_gem_object_is_volatile(obj))
		return false;

	if (obj->mm.madv != I915_MADV_WILLNEED)
		return false;

	return !freed(obj);
}

static int
lmem_put_pages(struct drm_i915_gem_object *obj, struct sg_table *pages)
{
	struct intel_memory_region *mem = obj->mm.region.mem;
	unsigned int clear = BIT(INTEL_MEMORY_CLEAR_FREE);
	bool dirty;

	if (need_swap(obj)) {
		int err;

		err = lmem_swapout(obj);
		if (err)
			return err;

		clear = 0;
	}

	i915_gem_object_migrate_decouple(obj);
	obj->flags &= ~I915_BO_SYNC_HINT;

	/*
	 * Clear-on-free.
	 *
	 * We always clear user objects before use to avoid leaking any stale
	 * information from previous local memory users. We do this immediately
	 * before the first access and so may incur a large synchronisation
	 * penalty. However, if we can clear the memory on freeing it, ahead
	 * of time, we can avoid the penalty upon reuse. The counter point is
	 * that not all memory is reused, nor does all memory need to be
	 * cleared, so clearing ahead of time may in fact cause us to use more
	 * blitter/memory bandwidth.  Further, since we track clears per
	 * buddy-block, if we clear 8GiB but only need 1GiB, for example, the
	 * next allocation pays the full latency and bandwidth cost of that
	 * 8GiB clear. In addition to the extra memory bandwidth that we
	 * may consume, doing so incurs a power cost; moving that cost
	 * may have even greater impact (on e.g. thermals) and throttling.
	 *
	 * We apply a couple of heuristics when to clear:
	 *
	 * - Only clear user objects as an indication that this memory is
	 *   likely to be cleared again on reuse (assuming that kernel objects
	 *   are reused by kernel objects, user objects reused by user objects.
	 *
	 * - Avoid clearing upon eviction / after swapping. If we are in the
	 *   middle of an eviction, these pages are likely to be reused by
	 *   something being swapped in, thus do not need to be cleared. Or if
	 *   we are being evicted to be reused by a clear buffer, that clear
	 *   will need to performed on the active pages and thus require a
	 *   synchronisation penalty anyway.
	 *
	 * - Only begin proactive clear-on-free once we encounter eviction
	 *   pressure for the current workload. This optimises small workloads
	 *   which only require clear-on-idle to avoid extraneous background
	 *   memory bandwidth and power utilisation during their execution.
	 */
	dirty = true;
	if (IS_ENABLED(CONFIG_DRM_I915_CHICKEN_CLEAR_ON_FREE) &&
	    mem->flags & clear &&
	    !(obj->mm.page_sizes & (mem->min_page_size - 1)) &&
	    freed(obj)) {
		struct intel_gt *gt = mem->gt;
		intel_wakeref_t wf;

		with_intel_gt_pm_if_awake(gt, wf) {
			struct i915_request *rq = NULL;
			struct intel_context *ce;

			ce = get_clear_free_context(gt);
			if (unlikely(!ce))
				continue;

			/*
			 * Don't saturate the ring if we are opportunistically
			 * clearing; leave some space and bandwidth for
			 * clear-on-alloc if required.
			 */
			if (intel_context_throttle(ce, 0))
				continue;

			dirty = clear_blt(ce, NULL,
					  &mem->mm, &obj->mm.blocks,
					  INTEL_GT_CLEAR_FREE_CYCLES, true,
					  &rq);

			if (rq) {
				dma_fence_enable_sw_signaling(&rq->fence);
				i915_sw_fence_complete(&rq->submit);
				i915_request_put(rq);
			}
		}
	}

	return i915_gem_object_put_pages_buddy(obj, pages, dirty);
}

static int
i915_ww_pin_lock_interruptible(struct drm_i915_gem_object *obj)
{
	struct i915_gem_ww_ctx ww;
	int ret;

	for_i915_gem_ww(&ww, ret, true) {
		ret = i915_gem_object_lock(obj, &ww);
		if (ret)
			continue;

		ret = i915_gem_object_pin_pages_sync(obj);
		if (ret)
			continue;

		ret = i915_gem_object_set_to_wc_domain(obj, false);
		if (ret)
			goto out_unpin;

		ret = i915_gem_object_wait(obj,
					   I915_WAIT_INTERRUPTIBLE,
					   MAX_SCHEDULE_TIMEOUT);
		if (!ret)
			continue;

out_unpin:
		i915_gem_object_unpin_pages(obj);

		/* Unlocking is done implicitly */
	}

	return ret;
}

static int lmem_pread(struct drm_i915_gem_object *obj,
		      const struct drm_i915_gem_pread *arg)
{
	struct drm_i915_private *i915 = to_i915(obj->base.dev);
	struct intel_runtime_pm *rpm = &i915->runtime_pm;
	intel_wakeref_t wakeref;
	char __user *user_data;
	unsigned int offset;
	unsigned long idx;
	u64 remain;
	int ret;

	ret = i915_gem_object_wait(obj,
				   I915_WAIT_INTERRUPTIBLE,
				   MAX_SCHEDULE_TIMEOUT);
	if (ret)
		return ret;

	ret = i915_ww_pin_lock_interruptible(obj);
	if (ret)
		return ret;

	wakeref = intel_runtime_pm_get(rpm);

	remain = arg->size;
	user_data = u64_to_user_ptr(arg->data_ptr);
	offset = offset_in_page(arg->offset);
	for (idx = arg->offset >> PAGE_SHIFT; remain; idx++) {
		unsigned long unwritten;
		void __iomem *vaddr;
		int length;

		length = remain;
		if (offset + length > PAGE_SIZE)
			length = PAGE_SIZE - offset;

		vaddr = i915_gem_object_lmem_io_map_page_atomic(obj, idx);
		if (!vaddr) {
			ret = -ENOMEM;
			goto out_put;
		}
		unwritten = __copy_to_user_inatomic(user_data,
						    (void __force *)vaddr + offset,
						    length);
		io_mapping_unmap_atomic(vaddr);
		if (unwritten) {
			vaddr = i915_gem_object_lmem_io_map_page(obj, idx);
			if (!IS_ERR_OR_NULL(vaddr)) {
				unwritten = copy_to_user(user_data,
							 (void __force *)vaddr + offset,
							 length);
				io_mapping_unmap(vaddr);
			}
		}
		if (unwritten) {
			ret = -EFAULT;
			goto out_put;
		}

		remain -= length;
		user_data += length;
		offset = 0;
	}

out_put:
	intel_runtime_pm_put(rpm, wakeref);
	i915_gem_object_unpin_pages(obj);

	return ret;
}

static int lmem_pwrite(struct drm_i915_gem_object *obj,
		       const struct drm_i915_gem_pwrite *arg)
{
	struct drm_i915_private *i915 = to_i915(obj->base.dev);
	struct intel_runtime_pm *rpm = &i915->runtime_pm;
	intel_wakeref_t wakeref;
	char __user *user_data;
	unsigned int offset;
	unsigned long idx;
	u64 remain;
	int ret;

	ret = i915_gem_object_wait(obj,
				   I915_WAIT_INTERRUPTIBLE,
				   MAX_SCHEDULE_TIMEOUT);
	if (ret)
		return ret;

	ret = i915_ww_pin_lock_interruptible(obj);
	if (ret)
		return ret;

	wakeref = intel_runtime_pm_get(rpm);

	remain = arg->size;
	user_data = u64_to_user_ptr(arg->data_ptr);
	offset = offset_in_page(arg->offset);
	for (idx = arg->offset >> PAGE_SHIFT; remain; idx++) {
		unsigned long unwritten;
		void __iomem *vaddr;
		int length;

		length = remain;
		if (offset + length > PAGE_SIZE)
			length = PAGE_SIZE - offset;

		vaddr = i915_gem_object_lmem_io_map_page_atomic(obj, idx);
		if (!vaddr) {
			ret = -ENOMEM;
			goto out_put;
		}

		unwritten = __copy_from_user_inatomic_nocache((void __force *)vaddr + offset,
							      user_data, length);
		io_mapping_unmap_atomic(vaddr);
		if (unwritten) {
			vaddr = i915_gem_object_lmem_io_map_page(obj, idx);
			if (!IS_ERR_OR_NULL(vaddr)) {
				unwritten = copy_from_user((void __force *)vaddr + offset,
							   user_data, length);
				io_mapping_unmap(vaddr);
			}
		}
		if (unwritten) {
			ret = -EFAULT;
			goto out_put;
		}

		remain -= length;
		user_data += length;
		offset = 0;
	}

out_put:
	intel_runtime_pm_put(rpm, wakeref);
	i915_gem_object_unpin_pages(obj);

	return ret;
}

const struct drm_i915_gem_object_ops i915_gem_lmem_obj_ops = {
	.name = "i915_gem_object_lmem",
	.flags = I915_GEM_OBJECT_HAS_IOMEM,

	.get_pages = lmem_get_pages,
	.put_pages = lmem_put_pages,
	.release = i915_gem_object_release_memory_region,

	.pread = lmem_pread,
	.pwrite = lmem_pwrite,
};

void __iomem *
i915_gem_object_lmem_io_map_page(struct drm_i915_gem_object *obj,
				 unsigned long n)
{
	struct intel_memory_region *mem = obj->mm.region.mem;
	resource_size_t offset;
	int err;

	err = i915_gem_object_migrate_sync(obj);
	if (err)
		return IO_ERR_PTR(err);

	offset = i915_gem_object_get_dma_address(obj, n);
	offset -= mem->region.start;

	return io_mapping_map_wc(&mem->iomap, offset, PAGE_SIZE);
}

struct drm_i915_gem_object *
i915_gem_object_create_lmem(struct drm_i915_private *i915,
			    resource_size_t size,
			    unsigned int flags)
{
	return i915_gem_object_create_region(to_gt(i915)->lmem, size, flags);
}

int __i915_gem_lmem_object_init(struct intel_memory_region *mem,
				struct drm_i915_gem_object *obj,
				resource_size_t size,
				unsigned int flags)
{
	static struct lock_class_key lock_class;
	struct drm_i915_private *i915 = mem->i915;

	drm_gem_private_object_init(&i915->drm, &obj->base, size);
	i915_gem_object_init(obj, &i915_gem_lmem_obj_ops, &lock_class, flags);

	obj->read_domains = I915_GEM_DOMAIN_WC | I915_GEM_DOMAIN_GTT;

	i915_gem_object_set_cache_coherency(obj, I915_CACHE_NONE);

	i915_gem_object_init_memory_region(obj, mem);

	return 0;
}

static bool
buddy_list_remove(struct i915_buddy_list *bl, struct list_head *list)
{
	struct i915_buddy_link *pos, *n;
	bool ret = false;

	if (list_empty(&bl->list))
		return false;

	spin_lock(&bl->lock);
	WRITE_ONCE(bl->defrag, true);
	list_for_each_entry_safe(pos, n, &bl->list, link) {
		if (unlikely(!pos->list)) { /* defrag bookmark! */
			list_del_init(&pos->link);
			continue;
		}

		GEM_BUG_ON(pos->list != bl);
		WRITE_ONCE(pos->list, NULL);
	}
	if (!list_empty(&bl->list)) {
		list_replace_init(&bl->list, list);
		ret = true;
	}
	spin_unlock(&bl->lock);

	return ret;
}

static void
buddy_list_add(struct list_head *old, struct i915_buddy_list *bl)
{
	struct i915_buddy_link *pos;

	spin_lock(&bl->lock);
	list_for_each_entry(pos, old, link) {
		GEM_BUG_ON(pos->list);
		WRITE_ONCE(pos->list, bl);
	}
	list_splice_tail(old, &bl->list);
	WRITE_ONCE(bl->defrag, true);
	spin_unlock(&bl->lock);
}

bool i915_gem_lmem_park(struct intel_memory_region *mem)
{
	struct i915_request *rq = NULL;
	struct i915_buddy_list *bl;
	struct intel_context *ce;
	struct list_head dirty;
	int i, min_order;

	if (!IS_ENABLED(CONFIG_DRM_I915_CHICKEN_CLEAR_ON_IDLE))
		return false;

	ce = get_clear_idle_context(mem->gt);
	if (!ce || !ce->private)
		return false;

	i915_buddy_defrag(&mem->mm, 0, UINT_MAX);

	/* Gradually clear (upto half each pass) local memory */
	min_order = ilog2(mem->min_page_size) - ilog2(mem->mm.chunk_size);
	for (i = mem->mm.max_order; i >= min_order; i--) {
		bl = &mem->mm.dirty_list[i];
		if (buddy_list_remove(bl, &dirty))
			break;
	}
	if (i < min_order) {
		clear_bit(INTEL_MEMORY_CLEAR_FREE, &mem->flags);
		return false;
	}

	__intel_wakeref_defer_park(&mem->gt->wakeref);
	reinit_completion(&mem->parking);
	mutex_unlock(&mem->gt->wakeref.mutex);

	if (clear_blt(ce, NULL,
		      &mem->mm, &dirty,
		      INTEL_GT_CLEAR_IDLE_CYCLES, false,
		      &rq) == 0)
		bl = &mem->mm.clear_list[i];

	buddy_list_add(&dirty, bl);

	if (rq) {
		dma_fence_enable_sw_signaling(&rq->fence); /* fast retire */
		i915_sw_fence_complete(&rq->submit);
		i915_request_put(rq);
	}

	complete_all(&mem->parking);
	mutex_lock(&mem->gt->wakeref.mutex);
	return __intel_wakeref_resume_park(&mem->gt->wakeref);
}

static void attach_window(struct intel_gt *gt, struct intel_migrate_window *w)
{
	struct intel_engine_cs *engine;
	enum intel_engine_id id;

	for_each_engine(engine, gt, id) {
		struct intel_context *ce;

		ce = engine->blitter_context;
		if (ce)
			ce->private = w;
	}
}

static int setup_pte_window(struct intel_gt *gt, struct intel_migrate_window *w)
{
	struct drm_i915_gem_object *obj;
	int err;

	w->clear_chunk = SZ_64M;
	w->swap_chunk = SZ_32M;

	obj = i915_gem_object_create_region(gt->lmem,
					    w->swap_chunk >> (PAGE_SHIFT - 3),
					    I915_BO_ALLOC_CONTIGUOUS |
					    I915_BO_ALLOC_VOLATILE);
	if (IS_ERR(obj))
		return PTR_ERR(obj);

	err = i915_gem_object_pin_pages_unlocked(obj);
	if (err)
		goto err;

	err = intel_flat_lmem_ppgtt_insert_window(gt->vm, obj, &w->node);
	if (err)
		goto err;

	w->obj = obj;
	w->pd_offset = sg_dma_address(obj->mm.pages->sgl);
	drm_dbg(&gt->i915->drm,
		"GT%d created %zdKiB PTE [%lldMiB] window @ lmem:%llx, gtt:%llx\n",
		gt->info.id,
		w->obj->base.size >> 10,
		w->node.size >> 20,
		w->pd_offset, w->node.start);

	GEM_BUG_ON(w->node.size > sg_dma_len(obj->mm.pages->sgl) << PAGE_SHIFT >> 3);
	GEM_BUG_ON(w->swap_chunk > w->node.size);

	attach_window(gt, w);
	return 0;

err:
	i915_gem_object_put(obj);
	return err;
}

static void cleanup_pte_window(struct intel_migrate_window *w)
{
	struct drm_i915_gem_object *obj;

	obj = fetch_and_zero(&w->obj);
	if (!obj)
		return;

	if (drm_mm_node_allocated(&w->node))
		drm_mm_remove_node(&w->node);

	i915_gem_object_unpin_pages(obj);
	i915_gem_object_put(obj);
}

static struct drm_i915_gem_object *
create_smem(struct intel_gt *gt, unsigned long sz)
{
	struct drm_i915_gem_object *obj;
	int err;

	obj = i915_gem_object_create_internal(gt->i915, sz);
	if (IS_ERR(obj))
		return obj;

	err = i915_gem_object_pin_pages_unlocked(obj);
	if (err)
		goto err;

	return obj;

err:
	i915_gem_object_put(obj);
	return ERR_PTR(err);
}

void i915_gem_init_lmem(struct intel_gt *gt)
{
	const long quantum_ns = 1000000; /* 1ms */
	struct intel_migrate *m = &gt->migrate;
	struct drm_i915_gem_object *smem;
	struct intel_migrate_window *w;
	struct i915_request *rq = NULL;
	struct intel_context *ce;
	intel_wakeref_t wf;
	LIST_HEAD(blocks);
	u64 cycles;
	int err = 0;

	if (!gt->lmem)
		return;

	w = &m->window;
	if (setup_pte_window(gt, w))
		return;

	ce = get_clear_alloc_context(gt);
	if (!ce)
		return;

	GEM_BUG_ON(ce->private != w);

	wf = intel_gt_pm_get(gt);
	intel_rps_boost(&gt->rps);

	err = __intel_memory_region_get_pages_buddy(gt->lmem, NULL,
						    SZ_16M, 0,
						    &blocks);
	if (err)
		goto err_wf;

	cycles = -READ_ONCE(gt->counters.map[INTEL_GT_CLEAR_ALLOC_CYCLES]);
	err = clear_blt(ce, NULL, &gt->lmem->mm, &blocks,
			INTEL_GT_CLEAR_ALLOC_CYCLES, true,
			&rq);
	if (rq) {
		i915_sw_fence_complete(&rq->submit);
		if (i915_request_wait(rq, 0, HZ) < 0)
			err = -ETIME;
		else
			err = err ?: rq->fence.error;
		i915_request_put(rq);
		rq = NULL;
	}
	cycles += READ_ONCE(gt->counters.map[INTEL_GT_CLEAR_ALLOC_CYCLES]);
	cycles = intel_gt_clock_interval_to_ns(gt, cycles);
	if (err == 0 && cycles) {
		u64 chunk_size;

		dev_info(gt->i915->drm.dev,
			 "GT%d: %s %s clear bandwidth:%lld MB/s\n",
			 gt->info.id, gt->lmem->name, ce->engine->name,
			 div_u64(mul_u32_u32(1000, SZ_16M), cycles));

		chunk_size = div_u64(mul_u32_u32(quantum_ns, SZ_16M), cycles);
		chunk_size = max_t(u64, chunk_size, SZ_64K);
		chunk_size = roundup_pow_of_two(chunk_size + 1);
		w->clear_chunk = min_t(u64, chunk_size, SZ_2G);
		drm_dbg(&gt->i915->drm,
			"GT%d: %s %s clear chunk size:%luKiB\n",
			gt->info.id, gt->lmem->name, ce->engine->name,
			w->clear_chunk >> 10);
	}

	smem = ERR_PTR(-ENODEV);
	if (w->obj)
		smem = create_smem(gt, SZ_16M);
	if (!IS_ERR(smem)) {
		cycles = -READ_ONCE(gt->counters.map[INTEL_GT_SWAPOUT_CYCLES]);
		err = swap_blt(ce, smem /* dummy */,
			       &gt->lmem->mm, &blocks,
			       smem, true, false,
			       &rq);
		if (rq) {
			i915_sw_fence_complete(&rq->submit);
			if (i915_request_wait(rq, 0, HZ) < 0)
				err = -ETIME;
			else
				err = err ?: rq->fence.error;
			i915_request_put(rq);
			rq = NULL;
		}
		cycles += READ_ONCE(gt->counters.map[INTEL_GT_SWAPOUT_CYCLES]);
		cycles = intel_gt_clock_interval_to_ns(gt, cycles);
		if (err == 0 && cycles) {
			u64 chunk_size;

			dev_info(gt->i915->drm.dev,
				 "GT%d: %s %s swap bandwidth:%lld MB/s\n",
				 gt->info.id, gt->lmem->name, ce->engine->name,
				 div_u64(mul_u32_u32(1000, SZ_16M), cycles));

			chunk_size =
				div_u64(mul_u32_u32(quantum_ns, SZ_16M), cycles);
			chunk_size = max_t(u64, chunk_size, SZ_64K);
			chunk_size = roundup_pow_of_two(chunk_size);
			w->swap_chunk = min_t(u64, chunk_size, w->node.size);
			drm_dbg(&gt->i915->drm,
				"GT%d: %s %s swap chunk size:%luKiB\n",
				gt->info.id, gt->lmem->name, ce->engine->name,
				w->swap_chunk >> 10);
		}
		i915_gem_object_put(smem);
	}

	__intel_memory_region_put_pages_buddy(gt->lmem, &blocks, false);
err_wf:
	intel_rps_cancel_boost(&gt->rps);
	intel_gt_pm_put(gt, wf);
}

enum {
	BITS = 0,
	VALUE,
	DATA,
};

static u32 *emit_xor_or(u32 *cs, u32 x, u64 offset)
{
	*cs++ = MI_LOAD_REGISTER_IMM(2) | MI_LRI_LRM_CS_MMIO;
	*cs++ = i915_mmio_reg_offset(GEN8_RING_CS_GPR(0, VALUE));
	*cs++ = x;
	*cs++ = i915_mmio_reg_offset(GEN8_RING_CS_GPR_UDW(0, VALUE));
	*cs++ = x;

	*cs++ = MI_LOAD_REGISTER_MEM_GEN8 | MI_LRI_LRM_CS_MMIO;
	*cs++ = i915_mmio_reg_offset(GEN8_RING_CS_GPR(0, DATA));
	*cs++ = lower_32_bits(offset);
	*cs++ = upper_32_bits(offset);
	*cs++ = MI_LOAD_REGISTER_MEM_GEN8 | MI_LRI_LRM_CS_MMIO;
	*cs++ = i915_mmio_reg_offset(GEN8_RING_CS_GPR_UDW(0, DATA));
	*cs++ = lower_32_bits(offset + 4);
	*cs++ = upper_32_bits(offset + 4);

	*cs++ = MI_MATH(8);
	*cs++ = MI_MATH_LOAD(MI_MATH_REG_SRCA, MI_MATH_REG(VALUE));
	*cs++ = MI_MATH_LOAD(MI_MATH_REG_SRCB, MI_MATH_REG(DATA));
	*cs++ = MI_MATH_XOR;
	*cs++ = MI_MATH_STORE(MI_MATH_REG(DATA), MI_MATH_REG_ACCU);
	*cs++ = MI_MATH_LOAD(MI_MATH_REG_SRCA, MI_MATH_REG(BITS));
	*cs++ = MI_MATH_LOAD(MI_MATH_REG_SRCB, MI_MATH_REG(DATA));
	*cs++ = MI_MATH_OR;
	*cs++ = MI_MATH_STORE(MI_MATH_REG(BITS), MI_MATH_REG_ACCU);

	return cs;
}

static int
set_gpr(struct intel_context *ce,
	int gpr, u64 val,
	struct i915_request **chain)
{
	struct i915_request *rq;
	u32 *cs;

	rq = i915_request_create_locked(ce, GFP_KERNEL);
	if (IS_ERR(rq))
		return PTR_ERR(rq);

	cs = intel_ring_begin(rq, 6);
	if (IS_ERR(cs)) {
		i915_request_add(rq);
		return PTR_ERR(cs);
	}

	*cs++ = MI_LOAD_REGISTER_IMM(2) | MI_LRI_LRM_CS_MMIO;
	*cs++ = i915_mmio_reg_offset(GEN8_RING_CS_GPR(0, gpr));
	*cs++ = lower_32_bits(val);
	*cs++ = i915_mmio_reg_offset(GEN8_RING_CS_GPR_UDW(0, gpr));
	*cs++ = upper_32_bits(val);
	*cs++ = MI_NOOP;
	intel_ring_advance(rq, cs);

	*chain = chain_request(rq, *chain);
	return 0;
}

static int
get_gpr(struct intel_context *ce,
	int gpr, u32 offset,
	struct i915_request **chain)
{
	struct i915_request *rq;
	u32 *cs;

	rq = i915_request_create_locked(ce, GFP_KERNEL);
	if (IS_ERR(rq))
		return PTR_ERR(rq);

	cs = intel_ring_begin(rq, 8);
	if (IS_ERR(cs)) {
		i915_request_add(rq);
		return PTR_ERR(cs);
	}

	*cs++ = MI_STORE_REGISTER_MEM_GEN8 | MI_USE_GGTT | MI_LRI_LRM_CS_MMIO;
	*cs++ = i915_mmio_reg_offset(GEN8_RING_CS_GPR(0, gpr));
	*cs++ = offset;
	*cs++ = 0;
	*cs++ = MI_STORE_REGISTER_MEM_GEN8 | MI_USE_GGTT | MI_LRI_LRM_CS_MMIO;
	*cs++ = i915_mmio_reg_offset(GEN8_RING_CS_GPR_UDW(0, gpr));
	*cs++ = offset + 4;
	*cs++ = 0;
	intel_ring_advance(rq, cs);

	*chain = chain_request(rq, *chain);
	return 0;
}

static void *hwsp(struct intel_context *ce, int offset)
{
	void *va;

	va = ce->lrc_reg_state;
	va -= PAGE_SIZE;
	va += offset;

	return va;
}

static u32 hwsp_offset(struct intel_context *ce, void *va)
{
	return i915_ggtt_offset(ce->state) + offset_in_page(va);
}

static int
run_alone(struct intel_gt *gt, intel_engine_mask_t ex, u32 offset)
{
	struct intel_engine_cs *engine;
	intel_engine_mask_t tmp;

	for_each_engine_masked(engine, gt, gt->info.engine_mask & ~ex, tmp) {
		struct i915_request *rq;
		u32 *cs;

		rq = intel_engine_create_kernel_request(engine);
		if (IS_ERR(rq))
			return PTR_ERR(rq);

		cs = intel_ring_begin(rq, 8);
		if (IS_ERR(cs)) {
			i915_request_add(rq);
			return PTR_ERR(cs);
		}

		/* We have begun! */
		*cs++ = MI_ARB_ON_OFF | MI_ARB_DISABLE;
		*cs++ = MI_ATOMIC | MI_USE_GGTT | MI_ATOMIC_DEC;
		*cs++ = offset;
		*cs++ = 0;

		/* Wait for completion */
		*cs++ = MI_SEMAPHORE_WAIT |
			MI_SEMAPHORE_GLOBAL_GTT |
			MI_SEMAPHORE_POLL |
			MI_SEMAPHORE_SAD_EQ_SDD;
		*cs++ = 0xffffffff;
		*cs++ = offset + 4;
		*cs++ = 0;

		intel_ring_advance(rq, cs);

		i915_request_set_priority(rq, I915_PRIORITY_UNPREEMPTABLE);
		i915_request_add(rq);
	}

	return 0;
}

static int
wait_for_run_alone(struct intel_context *ce,
		   u32 *sema,
		   struct i915_request **chain)
{
	const u32 offset = hwsp_offset(ce, sema);
	struct i915_request *rq;
	u32 *cs;

	rq = i915_request_create_locked(ce, GFP_KERNEL);
	if (IS_ERR(rq))
		return PTR_ERR(rq);

	cs = intel_ring_begin(rq, 8);
	if (IS_ERR(cs)) {
		i915_request_add(rq);
		return PTR_ERR(cs);
	}

	/* We have begun! */
	*cs++ = MI_ARB_ON_OFF | MI_ARB_DISABLE;
	*cs++ = MI_ATOMIC | MI_USE_GGTT | MI_ATOMIC_DEC;
	*cs++ = offset;
	*cs++ = 0;

	/* Wait for everyone */
	*cs++ = MI_SEMAPHORE_WAIT |
		MI_SEMAPHORE_GLOBAL_GTT |
		MI_SEMAPHORE_POLL |
		MI_SEMAPHORE_SAD_EQ_SDD;
	*cs++ = 0;
	*cs++ = offset;
	*cs++ = 0;

	intel_ring_advance(rq, cs);

	GEM_BUG_ON(sema[0] == 0);
	GEM_BUG_ON(sema[1] != 0);

	*chain = chain_request(rq, *chain);
	return 0;
}

static unsigned int
__max_order(const struct i915_buddy_mm *mm, unsigned long n_pages)
{
	if (n_pages >> mm->max_order)
		return mm->max_order;

	return __fls(n_pages);
}

static u32 expand_u32_from_u8(u32 x)
{
	return x << 24 | x << 16 | x << 8 | x;
}

static int suboffset(int i, int len, int sz)
{
	/* Leave space for a 64b read */
	return prandom_u32_max(min(len - i * sz, sz)) & -8;
}

int i915_gem_lmemtest(struct intel_gt *gt, u64 *error_bits)
{
	static const u8 values[] = { 0, 0x0f, 0xa3, 0x5c, 0xf0, 0xff };
	struct list_head *phases[] = {
		&gt->lmem->objects.list,
		&gt->lmem->objects.purgeable,
		NULL
	}, **phase = phases;
	struct intel_memory_region_link bookmark = {};
	struct intel_memory_region_link *pos;
	struct i915_buddy_block *swp, *block;
	struct i915_request *last = NULL;
	struct intel_memory_region *mr;
	struct drm_mm_node *node, *nn;
	const int semaphore = 0x800;
	struct intel_context *ce;
	struct drm_mm pinned;
	intel_wakeref_t wf;
	u64 swp_offset;
	u64 start, end;
	int err = 0;
	u32 *sema;

	mr = gt->lmem;
	if (!mr)
		return 0;

	wf = intel_gt_pm_get(gt);
	intel_rps_boost(&gt->rps);

	ce = get_blitter_context(gt, BCS0); /* use the fastest engine */
	if (!ce) {
		err = -EIO;
		goto err_wf;
	}

	/* Allocate temporary storage for contents */
	swp = i915_buddy_alloc(&mr->mm, __max_order(&mr->mm, SZ_16M >> ilog2(mr->mm.chunk_size)));
	if (IS_ERR(swp)) {
		err = PTR_ERR(swp);
		goto err_wf;
	}
	GEM_BUG_ON(i915_buddy_block_size(&mr->mm, swp) != SZ_16M);
	swp_offset = i915_buddy_block_offset(swp);

	/* Track all pinned blocks in use by the kernel; these are vital */
	drm_mm_init(&pinned, gt->flat.start, mr->total);
	node = kzalloc(sizeof(*node), GFP_KERNEL);
	if (!node) {
		err = -ENOMEM;
		goto err_buddy;
	}
	node->start = swp_offset;
	node->size = i915_buddy_block_size(&mr->mm, swp);
	err = drm_mm_reserve_node(&pinned, node);
	if (err)
		goto err_buddy;

	spin_lock(&mr->objects.lock);
	do list_for_each_entry(pos, *phase, link) {
		struct drm_i915_gem_object *obj;
		struct list_head *blocks;

		if (!pos->mem)
			continue;

		/* Only skip testing memory regions pinned by the kernel */
		obj = container_of(pos, typeof(*obj), mm.region);
		if (obj->flags & I915_BO_ALLOC_USER ||
		    !i915_gem_object_has_pinned_pages(obj))
			continue;

		list_add(&bookmark.link, &pos->link);
		blocks = &obj->mm.blocks;
		spin_unlock(&mr->objects.lock);

		list_for_each_entry(block, blocks, link) {
			node = kzalloc(sizeof(*node), GFP_KERNEL);
			if (!node) {
				err = -ENOMEM;
				goto err_buddy;
			}
			node->start = i915_buddy_block_offset(block);
			node->size = i915_buddy_block_size(&mr->mm, block);
			err = drm_mm_reserve_node(&pinned, node);
			if (err)
				goto err_buddy;
		}

		spin_lock(&mr->objects.lock);
		__list_del_entry(&bookmark.link);
		pos = &bookmark;
	} while (*++phase);
	spin_unlock(&mr->objects.lock);

	/* Stall execution on all other engines */
	sema = memset32(hwsp(ce, semaphore), 0, 4);
	sema[0] = hweight32(gt->info.engine_mask);
	i915_write_barrier(gt->i915);

	err = run_alone(gt, ce->engine->mask, hwsp_offset(ce, sema));
	if (err)
		goto err_sema;

	/* Destructively write test every block not used by the kernel */
	mutex_lock(&ce->timeline->mutex);
	intel_context_enter(ce);

	err = wait_for_run_alone(ce, sema, &last);
	if (err)
		goto out;

	err = set_gpr(ce, BITS, mr->memtest, &last);
	if (err)
		goto out;

	drm_mm_for_each_hole(node, &pinned, start, end) {
		while (start < end) {
			const u32 len = min_t(u64, end - start, SZ_16M);
			const int sample = DIV_ROUND_UP(len, SZ_2M);
			struct i915_request *rq;
			u32 mocs = 0;
			int v, i;
			u32 *cs;

			rq = i915_request_create_locked(ce, GFP_KERNEL);
			if (IS_ERR(rq)) {
				err = PTR_ERR(rq);
				goto out;
			}

			cs = intel_ring_begin(rq, ARRAY_SIZE(values) * (20 + 22 * sample) + 2 * 10);
			if (IS_ERR(cs)) {
				err = PTR_ERR(cs);
				last = chain_request(rq, last);
				goto out;
			}

			/* Keep a copy of the original user data */
			*cs++ = GEN9_XY_FAST_COPY_BLT_CMD | (10 - 2);
			*cs++ = BLT_DEPTH_32 | PAGE_SIZE | mocs;
			*cs++ = 0;
			*cs++ = len >> PAGE_SHIFT << 16 | PAGE_SIZE / 4;
			*cs++ = lower_32_bits(swp_offset);
			*cs++ = upper_32_bits(swp_offset);
			*cs++ = 0;
			*cs++ = PAGE_SIZE | mocs;
			*cs++ = lower_32_bits(start);
			*cs++ = upper_32_bits(start);

			/* Overwrite with a few altenating bit patterns */
			for (v = 0; v < ARRAY_SIZE(values); v++) {
				u32 x = expand_u32_from_u8(values[v]);
				int pkt;

				pkt = 16;
				if (GRAPHICS_VER_FULL(gt->i915) < IP_VER(12, 50))
					pkt = 11;

				*cs++ = GEN9_XY_FAST_COLOR_BLT_CMD |
					XY_FAST_COLOR_BLT_DEPTH_32 |
					(pkt - 2);
				*cs++ = mocs | (PAGE_SIZE - 1);
				*cs++ = 0;
				*cs++ = len >> PAGE_SHIFT << 16 | PAGE_SIZE / 4;
				*cs++ = lower_32_bits(start);
				*cs++ = upper_32_bits(start);
				*cs++ = 0;
				*cs++ = x;
				*cs++ = 0;
				*cs++ = 0;
				*cs++ = 0;
				*cs++ = 0;
				*cs++ = 0;
				*cs++ = 0;
				*cs++ = 0;
				*cs++ = 0;

				cs = __emit_flush(cs, 0);

				/* Randomly sample for bit errors */
				for (i = 0; i < sample; i++) {
					u64 addr;

					addr = start;
					addr += i * SZ_2M;
					addr += suboffset(i, len, SZ_2M);

					cs = emit_xor_or(cs, x, addr);
				}
			}

			/* Restore user contents */
			*cs++ = GEN9_XY_FAST_COPY_BLT_CMD | (10 - 2);
			*cs++ = BLT_DEPTH_32 | PAGE_SIZE | mocs;
			*cs++ = 0;
			*cs++ = len >> PAGE_SHIFT << 16 | PAGE_SIZE / 4;
			*cs++ = lower_32_bits(start);
			*cs++ = upper_32_bits(start);
			*cs++ = 0;
			*cs++ = PAGE_SIZE | mocs;
			*cs++ = lower_32_bits(swp_offset);
			*cs++ = upper_32_bits(swp_offset);

			intel_ring_advance(rq, cs);
			last = chain_request(rq, last);

			start += len;
		}
	}
out:
	if (err == 0)
		err = get_gpr(ce, BITS, hwsp_offset(ce, sema + 2), &last);
	if (last) {
		i915_sw_fence_complete(&last->submit);
		i915_request_wait(last, 0, MAX_SCHEDULE_TIMEOUT);
		if (err == 0)
			err = last->fence.error;
		i915_request_put(last);
	}
	if (err == 0)
		memcpy(error_bits, sema + 2, sizeof(*error_bits));
	intel_context_exit(ce);
	mutex_unlock(&ce->timeline->mutex);

err_sema:
	WRITE_ONCE(sema[1], 0xffffffff);
	i915_write_barrier(gt->i915);

err_buddy:
	drm_mm_for_each_node_safe(node, nn, &pinned)
		kfree(node);
	i915_buddy_free(&mr->mm, swp);
err_wf:
	intel_rps_cancel_boost(&gt->rps);
	intel_gt_pm_put(gt, wf);
	return err;
}

void i915_gem_fini_lmem(struct intel_gt *gt)
{
	struct intel_migrate *m = &gt->migrate;

	cleanup_pte_window(&m->window);
}

static struct i915_request *
copy_blt(struct intel_context *ce,
	 struct drm_i915_gem_object *lmem,
	 struct drm_i915_gem_object *other,
	 bool to_other,
	 bool nowait)
{
	const bool use_pvc_memcpy = HAS_LINK_COPY_ENGINES(ce->engine->i915);
	const bool use_flat_ccs =
		!use_pvc_memcpy &&
		object_needs_flat_ccs(lmem) &&
		object_needs_flat_ccs(other);
	const u64 encode =
		ce->vm->pte_encode(0,
				   i915_gem_get_pat_index(ce->engine->i915, I915_CACHING_NONE),
				   i915_gem_object_is_lmem(other) ? PTE_LM : 0);
	struct i915_buddy_mm *mm = &lmem->mm.region.mem->mm;
	const struct intel_migrate_window *w = ce->private;
	struct i915_request *last = NULL, *rq;
	u64 pd_offset = 0, pte_window, pte_end;
	const u32 step = w->swap_chunk;
	struct i915_buddy_block *block;
	struct sgt_iter it_pte;
	struct i915_vma *ref;
	int err;

	GEM_BUG_ON(ce->ring->size < SZ_256K);
	GEM_BUG_ON(ce->vm != ce->engine->gt->vm);
	GEM_BUG_ON(!drm_mm_node_allocated(&ce->engine->gt->flat));
	GEM_BUG_ON(!IS_ALIGNED(step, PAGE_SIZE));

	if (GEM_WARN_ON(use_flat_ccs))
		return ERR_PTR(-EINVAL);

	if (GEM_WARN_ON(other->base.size < lmem->base.size))
		return ERR_PTR(-EINVAL);

	err = intel_context_throttle(ce, MAX_SCHEDULE_TIMEOUT);
	if (err)
		return ERR_PTR(err);

	ref = i915_vma_instance(other, ce->vm, NULL);
	if (IS_ERR(ref))
		return ERR_CAST(ref);

	it_pte = __sgt_iter(other->mm.pages->sgl, true);
	GEM_BUG_ON(!it_pte.dma);

	mutex_lock(&ce->timeline->mutex);
	intel_context_enter(ce);

	rq = i915_request_create_locked(ce, I915_GFP_ALLOW_FAIL);
	if (IS_ERR(rq)) {
		err = PTR_ERR(rq);
		goto exit;
	}

	if (!nowait) {
		err = i915_request_await_object(rq, lmem, !to_other);
		if (err)
			goto skip;

		err = i915_request_await_object(rq, other, to_other);
		if (err)
			goto skip;
	}

	pte_end = w->node.start + w->node.size;
	pte_window = pte_end;

	list_for_each_entry(block, &lmem->mm.blocks, link) {
		u64 sz = i915_buddy_block_size(mm, block);
		u64 offset = i915_buddy_block_offset(block);

		do { /* re-coalesce split blocks */
			struct i915_buddy_block *next =
				list_next_entry(block, link);

			err = i915_active_fence_set(&block->active, rq);
			if (err < 0)
				goto skip;

			if (list_is_head(&next->link, &lmem->mm.blocks))
				break;

			if (i915_buddy_block_offset(next) != offset + sz)
				break;

			sz += i915_buddy_block_size(mm, next);
			block = next;
		} while (1);

		do {
			u32 length = min_t(u64, sz, step);

			GEM_BUG_ON(offset < ce->engine->gt->flat.start);
			GEM_BUG_ON(offset + length > ce->engine->gt->flat.start + ce->engine->gt->flat.size);

			if (submit_request(rq, &last, swap_pkt(length))) {
				rq = i915_request_create_locked(ce, I915_GFP_ALLOW_FAIL);
				if (IS_ERR(rq)) {
					err = PTR_ERR(rq);
					goto exit;
				}

				err = update_active_blocks(rq,
							   mm, block,
							   offset);
				if (err)
					goto skip;
			}

			err = emit_start_timestamp(rq);
			if (err)
				goto skip;

			if (pte_window + length > pte_end) {
				pd_offset = w->pd_offset;
				pte_window = w->node.start;

				err = emit_tlb_invalidate(rq);
				if (err)
					goto skip;
			}

			err = emit_pte(rq, &it_pte,
				       pd_offset, encode,
				       length >> PAGE_SHIFT);
			if (err)
				goto skip;

			if (use_pvc_memcpy)
				err = pvc_emit_swap(rq,
						    offset, pte_window, length,
						    to_other);
			else
				err = xy_emit_swap(rq,
						   offset, pte_window, length,
						   to_other);
			if (err)
				goto skip;

			err = emit_update_counters(rq, length, INTEL_GT_COPY_CYCLES);
			if (err)
				goto skip;

			pte_window += length;
			pd_offset += length >> PAGE_SHIFT << 3;
			GEM_BUG_ON(pte_window > pte_end);

			offset += length;
			sz -= length;
		} while (sz);
	}
skip:
	last = chain_request(rq, last);
exit:
	if (last) {
		i915_gem_object_migrate_prepare(other, &last->fence);
		i915_active_add_request(&ref->active, last);
		i915_sw_fence_complete(&last->submit);
	}

	intel_context_exit(ce);
	mutex_unlock(&ce->timeline->mutex);

	if (err) {
		if (last)
			i915_request_put(last);
		last = ERR_PTR(err);
	}

	return last;
}

struct i915_request *
i915_gem_object_copy_lmem(struct drm_i915_gem_object *lmem,
			  struct drm_i915_gem_object *other,
			  bool to_other,
			  bool nowait)
{
	struct intel_context *ce;
	struct i915_request *rq;
	int err;

	assert_object_held(lmem);
	assert_object_held(other);

	GEM_BUG_ON(!i915_gem_object_has_pages(lmem));

	if (!i915_gem_object_is_lmem(lmem)) {
		to_other = !to_other;
		swap(lmem, other);
	}
	if (GEM_WARN_ON(!i915_gem_object_is_lmem(lmem)))
		return ERR_PTR(-EINVAL);

	ce = get_swapin_context(lmem->mm.region.mem->gt);
	if (!ce)
		return ERR_PTR(-EIO);

	if (lmem->base.size == other->base.size) {
		if (to_other)
			other->flags |= I915_BO_SKIP_CLEAR;
		else
			lmem->flags |= I915_BO_SKIP_CLEAR;
	}

	err = i915_gem_object_pin_pages(lmem);
	if (err)
		return ERR_PTR(err);

	err = i915_gem_object_pin_pages_sync(other); /* XXX defer? */
	if (err) {
		rq = ERR_PTR(err);
		goto unpin;
	}

	rq = copy_blt(ce, lmem, other, to_other, nowait);

	i915_gem_object_unpin_pages(other);
unpin:
	i915_gem_object_unpin_pages(lmem);

	return rq;
}

#if IS_ENABLED(CONFIG_DRM_I915_SELFTEST)
#include "selftests/i915_gem_lmem.c"
#endif
