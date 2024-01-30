// SPDX-License-Identifier: MIT
/*
 * Copyright © 2022 Intel Corporation
 */

#include "i915_selftest.h"

#include "gem/i915_gem_region.h"

#include "gen8_engine_cs.h"
#include "i915_gem_ww.h"
#include "intel_engine_regs.h"
#include "intel_gpu_commands.h"
#include "intel_context.h"
#include "intel_gt.h"
#include "intel_gt_clock_utils.h"
#include "intel_ring.h"
#include "intel_rps.h"

#include "selftests/igt_flush_test.h"
#include "selftests/i915_random.h"

static u32 *emit_timestamp(struct i915_request *rq, u32 *cs, int gpr)
{
	u32 base = rq->engine->mmio_base;

	*cs++ = MI_LOAD_REGISTER_REG;
	*cs++ = i915_mmio_reg_offset(RING_TIMESTAMP_UDW(base));
	*cs++ = i915_mmio_reg_offset(GEN8_RING_CS_GPR_UDW(base, gpr));

	*cs++ = MI_LOAD_REGISTER_REG;
	*cs++ = i915_mmio_reg_offset(RING_TIMESTAMP(base));
	*cs++ = i915_mmio_reg_offset(GEN8_RING_CS_GPR(base, gpr));

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

static u32 *emit_mem_fence(struct i915_request *rq, u32 *cs, u64 addr)
{
	return __gen8_emit_flush_dw(cs, 0, addr + 64, MI_FLUSH_DW_OP_STOREDW);
}

static int emit_write_elaspsed(struct i915_request *rq, u64 addr)
{
	u32 *cs;

	cs = intel_ring_begin(rq, 20);
	if (IS_ERR(cs))
		return PTR_ERR(cs);

	cs = emit_mem_fence(rq, cs, addr + 64);
	cs = emit_timestamp(rq, cs, 1);

	/* Compute elapsed time (end - start) */
	*cs++ = MI_MATH(4);
	*cs++ = MI_MATH_LOAD(MI_MATH_REG_SRCA, MI_MATH_REG(1));
	*cs++ = MI_MATH_LOAD(MI_MATH_REG_SRCB, MI_MATH_REG(0));
	*cs++ = MI_MATH_SUB;
	*cs++ = MI_MATH_STORE(MI_MATH_REG(0), MI_MATH_REG_ACCU);

	/* Increment cycle counters */
	*cs++ = MI_STORE_REGISTER_MEM_GEN8;
	*cs++ = i915_mmio_reg_offset(GEN8_RING_CS_GPR(rq->engine->mmio_base, 0));
	*cs++ = lower_32_bits(addr);
	*cs++ = upper_32_bits(addr);

	*cs++ = MI_NOOP;

	intel_ring_advance(rq, cs);
	return 0;
}

static int emit_memset(struct i915_request *rq, struct i915_vma *vma)
{
	u64 offset = i915_vma_offset(vma);
	u32 *cs;
	int len;

	if (HAS_LINK_COPY_ENGINES(rq->engine->i915))
		len = 8;
	else if (GRAPHICS_VER_FULL(rq->engine->i915) >= IP_VER(12, 50))
		len = 16;
	else
		len = 12;

	cs = intel_ring_begin(rq, len);
	if (IS_ERR(cs))
		return PTR_ERR(cs);

	if (HAS_LINK_COPY_ENGINES(rq->engine->i915)) {
		*cs++ = PVC_MEM_SET_CMD | MS_MATRIX | (7 - 2);
		*cs++ = PAGE_SIZE - 1;
		*cs++ = (vma->size >> 12) - 1;
		*cs++ = PAGE_SIZE - 1;
		*cs++ = lower_32_bits(offset);
		*cs++ = upper_32_bits(offset);
		*cs++ = 0;
		*cs++ = MI_NOOP;
	} else if (GRAPHICS_VER_FULL(rq->engine->i915) >= IP_VER(12, 50)) {
		*cs++ = GEN9_XY_FAST_COLOR_BLT_CMD |
			XY_FAST_COLOR_BLT_DEPTH_32 |
			(16 - 2);
		*cs++ = PAGE_SIZE - 1;
		*cs++ = 0;
		*cs++ = vma->size >> PAGE_SHIFT << 16 | PAGE_SIZE / 4;
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
	} else {
		*cs++ = GEN9_XY_FAST_COLOR_BLT_CMD |
			XY_FAST_COLOR_BLT_DEPTH_32 |
			(11 - 2);
		*cs++ = PAGE_SIZE - 1;
		*cs++ = 0;
		*cs++ = vma->size >> PAGE_SHIFT << 16 | PAGE_SIZE / 4;
		*cs++ = lower_32_bits(offset);
		*cs++ = upper_32_bits(offset);
		*cs++ = 0;
		*cs++ = 0;
		*cs++ = 0;
		*cs++ = 0;
		*cs++ = 0;
		*cs++ = 0;
	}

	intel_ring_advance(rq, cs);
	return 0;
}

static int direct_op(struct intel_gt *gt, int (*op)(struct intel_context *ce, struct drm_i915_gem_object *obj))
{
	struct intel_engine_cs *engine;
	struct drm_i915_gem_object *obj;
	enum intel_engine_id id;
	int err = 0;
	void *va;

	/*
	 * We create a direct 1:1 mapping of device memory into the
	 * kernel's vm. This allows us to write directly into lmem
	 * without having to bind any vma by simply writing to its
	 * device address.
	 */

	if (!drm_mm_node_allocated(&gt->flat))
		return 0;

	obj = i915_gem_object_create_region(gt->lmem, rounddown_pow_of_two(gt->lmem->total - 1), 0);
	if (IS_ERR(obj))
		return 0;

	pr_info("Created an %zd MiB lmem object on gt%d\n",
		obj->base.size >> 20, gt->info.id);

	va = i915_gem_object_pin_map_unlocked(obj, I915_MAP_WC);
	if (IS_ERR(va)) {
		err = PTR_ERR(va);
		goto out;
	}

	for_each_engine(engine, gt, id) {
		struct intel_context *ce;
		struct i915_gem_ww_ctx ww;

		ce = intel_context_create(engine);
		if (IS_ERR(ce))
			return PTR_ERR(ce);

		if (ce->vm != gt->vm) {
			intel_context_put(ce);
			return -ENXIO;
		}

		for_i915_gem_ww(&ww, err, true)
			err = intel_context_pin_ww(ce, &ww);
		if (err == 0) {
			err = op(ce, obj);
			intel_context_unpin(ce);
		}
		intel_context_put(ce);
		if (err)
			break;
	}

out:
	i915_gem_object_put(obj);
	if (igt_flush_test(gt->i915))
		err = -EIO;
	return err;
}

static int __direct_store(struct intel_context *ce,
			  struct drm_i915_gem_object *obj)
{
	u32 * const va = page_mask_bits(obj->mm.mapping);
	const int count = ilog2(obj->base.size) - PAGE_SHIFT;
	struct i915_request *rq;
	long timeout;
	int i, err;
	u32 *cs;

	rq = i915_request_create(ce);
	if (IS_ERR(rq))
		return PTR_ERR(rq);

	cs = intel_ring_begin(rq, count * 2 * 4);
	if (IS_ERR(cs)) {
		i915_request_add(rq);
		return PTR_ERR(cs);
	}

	for (i = 0; i <= count; i++) {
		unsigned long page;
		u64 address;

		page = BIT(i) - 1;
		va[page * PAGE_SIZE / sizeof(*va)] = STACK_MAGIC;
		address = i915_gem_object_get_dma_address(obj, page);

		*cs++ = MI_STORE_DWORD_IMM_GEN4;
		*cs++ = lower_32_bits(address);
		*cs++ = upper_32_bits(address);
		*cs++ = i;

		if (i > 0 && i < count) {
			page = BIT(i);
			va[page * PAGE_SIZE / sizeof(*va)] = STACK_MAGIC;
			address = i915_gem_object_get_dma_address(obj, page);

			*cs++ = MI_STORE_DWORD_IMM_GEN4;
			*cs++ = lower_32_bits(address);
			*cs++ = upper_32_bits(address);
			*cs++ = ~i;
		}
	}

	intel_ring_advance(rq, cs);

	i915_request_get(rq);
	i915_request_add(rq);
	timeout = i915_request_wait(rq, I915_WAIT_INTERRUPTIBLE, HZ);
	i915_request_put(rq);
	if (timeout < 0)
		return timeout;

	err = 0;
	for (i = 0; i <= count; i++) {
		unsigned long page;
		u32 value;

		page = BIT(i) - 1;
		value = va[page * PAGE_SIZE / sizeof(*va)];
		if (value != i) {
			pr_err("%s: Invalid found:%x, expected:%x at page:%lx, dma-address:%llx\n",
			       ce->engine->name,
			       value, i, page,
			       (u64)i915_gem_object_get_dma_address(obj, page));
			err = -EINVAL;
		}

		if (i > 0 && i < count) {
			page = BIT(i);
			value = va[page * PAGE_SIZE / sizeof(*va)];
			if (value != ~i) {
				pr_err("%s: Invalid found:%x, expected:%x found at page:%lx, dma-address:%llx\n",
						ce->engine->name,
						value, ~i, page,
						(u64)i915_gem_object_get_dma_address(obj, page));
				err = -EINVAL;
			}
		}
	}

	return err;
}

static int direct_store(void *arg)
{
	return direct_op(arg, __direct_store);
}

static int __direct_mov(struct intel_context *ce,
			struct drm_i915_gem_object *obj)
{
	u32 * const va = page_mask_bits(obj->mm.mapping);
	const int count = ilog2(obj->base.size) - PAGE_SHIFT;
	struct i915_request *rq;
	long timeout;
	int i, err;
	u32 *cs;

	rq = i915_request_create(ce);
	if (IS_ERR(rq))
		return PTR_ERR(rq);

	cs = intel_ring_begin(rq, count * 2 * 6);
	if (IS_ERR(cs)) {
		i915_request_add(rq);
		return PTR_ERR(cs);
	}

	for (i = 0; i <= count; i++) {
		unsigned long page;
		u64 address;

		page = BIT(i) - 1;
		va[page * PAGE_SIZE / sizeof(*va)] = STACK_MAGIC;
		address = i915_gem_object_get_dma_address(obj, page);

		*cs++ = MI_LOAD_REGISTER_IMM(1) | MI_LRI_LRM_CS_MMIO;
		*cs++ = i915_mmio_reg_offset(GEN8_RING_CS_GPR(0, 0));
		*cs++ = i;

		*cs++ = MI_ATOMIC | MI_ATOMIC_MOVE;
		*cs++ = lower_32_bits(address);
		*cs++ = upper_32_bits(address);

		if (i > 0 && i < count) {
			page = BIT(i);
			va[page * PAGE_SIZE / sizeof(*va)] = STACK_MAGIC;
			address = i915_gem_object_get_dma_address(obj, page);

			*cs++ = MI_LOAD_REGISTER_IMM(1) | MI_LRI_LRM_CS_MMIO;
			*cs++ = i915_mmio_reg_offset(GEN8_RING_CS_GPR(0, 0));
			*cs++ = ~i;

			*cs++ = MI_ATOMIC | MI_ATOMIC_MOVE;
			*cs++ = lower_32_bits(address);
			*cs++ = upper_32_bits(address);
		}
	}

	intel_ring_advance(rq, cs);

	i915_request_get(rq);
	i915_request_add(rq);
	timeout = i915_request_wait(rq, I915_WAIT_INTERRUPTIBLE, HZ);
	i915_request_put(rq);
	if (timeout < 0)
		return timeout;

	err = 0;
	for (i = 0; i <= count; i++) {
		unsigned long page;
		u32 value;

		page = BIT(i) - 1;
		value = va[page * PAGE_SIZE / sizeof(*va)];
		if (value != i) {
			pr_err("%s: Invalid found:%x, expected:%x at page:%lx, dma-address:%llx\n",
			       ce->engine->name,
			       value, i, page,
			       (u64)i915_gem_object_get_dma_address(obj, page));
			err = -EINVAL;
		}

		if (i > 0 && i < count) {
			page = BIT(i);
			value = va[page * PAGE_SIZE / sizeof(*va)];
			if (value != ~i) {
				pr_err("%s: Invalid found:%x, expected:%x found at page:%lx, dma-address:%llx\n",
						ce->engine->name,
						value, ~i, page,
						(u64)i915_gem_object_get_dma_address(obj, page));
				err = -EINVAL;
			}
		}
	}

	return err;
}

static int direct_mov(void *arg)
{
	return direct_op(arg, __direct_mov);
}

static int __direct_inc(struct intel_context *ce,
			struct drm_i915_gem_object *obj)
{
	u32 * const va = page_mask_bits(obj->mm.mapping);
	const int count = ilog2(obj->base.size) - PAGE_SHIFT;
	struct i915_request *rq;
	long timeout;
	int i, err;
	u32 *cs;

	rq = i915_request_create(ce);
	if (IS_ERR(rq))
		return PTR_ERR(rq);

	cs = intel_ring_begin(rq, count * 2 * 3);
	if (IS_ERR(cs)) {
		i915_request_add(rq);
		return PTR_ERR(cs);
	}

	for (i = 0; i <= count; i++) {
		unsigned long page;
		u64 address;

		page = BIT(i) - 1;
		va[page * PAGE_SIZE / sizeof(*va)] = i;
		address = i915_gem_object_get_dma_address(obj, page);

		*cs++ = MI_ATOMIC | MI_ATOMIC_INC;
		*cs++ = lower_32_bits(address);
		*cs++ = upper_32_bits(address);

		if (i > 0 && i < count) {
			page = BIT(i);
			va[page * PAGE_SIZE / sizeof(*va)] = i;
			address = i915_gem_object_get_dma_address(obj, page);

			*cs++ = MI_ATOMIC | MI_ATOMIC_INC;
			*cs++ = lower_32_bits(address);
			*cs++ = upper_32_bits(address);
		}
	}

	intel_ring_advance(rq, cs);

	i915_request_get(rq);
	i915_request_add(rq);
	timeout = i915_request_wait(rq, I915_WAIT_INTERRUPTIBLE, HZ);
	i915_request_put(rq);
	if (timeout < 0)
		return timeout;

	err = 0;
	for (i = 0; i <= count; i++) {
		unsigned long page;
		u32 value;

		page = BIT(i) - 1;
		value = va[page * PAGE_SIZE / sizeof(*va)];
		if (value != i + 1) {
			pr_err("%s: Invalid found:%x, expected:%x at page:%lx, dma-address:%llx\n",
			       ce->engine->name,
			       value, i + 1, page,
			       (u64)i915_gem_object_get_dma_address(obj, page));
			err = -EINVAL;
		}

		if (i > 0 && i < count) {
			page = BIT(i);
			value = va[page * PAGE_SIZE / sizeof(*va)];
			if (value != i + 1) {
				pr_err("%s: Invalid found:%x, expected:%x found at page:%lx, dma-address:%llx\n",
						ce->engine->name,
						value, i + 1, page,
						(u64)i915_gem_object_get_dma_address(obj, page));
				err = -EINVAL;
			}
		}
	}

	return err;
}

static int direct_inc(void *arg)
{
	return direct_op(arg, __direct_inc);
}

static int __direct_dec(struct intel_context *ce,
			struct drm_i915_gem_object *obj)
{
	u32 * const va = page_mask_bits(obj->mm.mapping);
	const int count = ilog2(obj->base.size) - PAGE_SHIFT;
	struct i915_request *rq;
	long timeout;
	int i, err;
	u32 *cs;

	rq = i915_request_create(ce);
	if (IS_ERR(rq))
		return PTR_ERR(rq);

	cs = intel_ring_begin(rq, count * 2 * 3);
	if (IS_ERR(cs)) {
		i915_request_add(rq);
		return PTR_ERR(cs);
	}

	for (i = 0; i <= count; i++) {
		unsigned long page;
		u64 address;

		page = BIT(i) - 1;
		va[page * PAGE_SIZE / sizeof(*va)] = i;
		address = i915_gem_object_get_dma_address(obj, page);

		*cs++ = MI_ATOMIC | MI_ATOMIC_DEC;
		*cs++ = lower_32_bits(address);
		*cs++ = upper_32_bits(address);

		if (i > 0 && i < count) {
			page = BIT(i);
			va[page * PAGE_SIZE / sizeof(*va)] = i;
			address = i915_gem_object_get_dma_address(obj, page);

			*cs++ = MI_ATOMIC | MI_ATOMIC_DEC;
			*cs++ = lower_32_bits(address);
			*cs++ = upper_32_bits(address);
		}
	}

	intel_ring_advance(rq, cs);

	i915_request_get(rq);
	i915_request_add(rq);
	timeout = i915_request_wait(rq, I915_WAIT_INTERRUPTIBLE, HZ);
	i915_request_put(rq);
	if (timeout < 0)
		return timeout;

	err = 0;
	for (i = 0; i <= count; i++) {
		unsigned long page;
		u32 value;

		page = BIT(i) - 1;
		value = va[page * PAGE_SIZE / sizeof(*va)];
		if (value != i - 1) {
			pr_err("%s: Invalid found:%x, expected:%x at page:%lx, dma-address:%llx\n",
			       ce->engine->name,
			       value, i - 1, page,
			       (u64)i915_gem_object_get_dma_address(obj, page));
			err = -EINVAL;
		}

		if (i > 0 && i < count) {
			page = BIT(i);
			value = va[page * PAGE_SIZE / sizeof(*va)];
			if (value != i - 1) {
				pr_err("%s: Invalid found:%x, expected:%x found at page:%lx, dma-address:%llx\n",
						ce->engine->name,
						value, i - 1, page,
						(u64)i915_gem_object_get_dma_address(obj, page));
				err = -EINVAL;
			}
		}
	}

	return err;
}

static int direct_dec(void *arg)
{
	return direct_op(arg, __direct_dec);
}

static void clear_dw(struct i915_vma *vma, u64 addr, u32 val)
{
	GEM_BUG_ON(addr < i915_vma_offset(vma));
	GEM_BUG_ON(addr >= i915_vma_offset(vma) + i915_vma_size(vma));
	memset32(page_mask_bits(vma->obj->mm.mapping) +
	       (addr - i915_vma_offset(vma)), val, 1);
}

static int
pte_write_tearing(struct intel_context *ce,
		  struct i915_vma *va,
		  struct i915_vma *vb,
		  u64 align,
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

	if (ce->engine->class != COPY_ENGINE_CLASS) /* MI invalidate TLB */
		return 0;

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

	cs = i915_gem_object_pin_map_unlocked(batch, I915_MAP_WC);
	*cs++ = MI_NOOP; /* for later termination */

	/* Sample the target to see if we spot an incorrect page */
	cs = __gen8_emit_flush_dw(cs, 0, i915_vma_offset(vma) + 4000,
				  MI_INVALIDATE_TLB | MI_FLUSH_DW_OP_STOREDW);
	*cs++ = MI_CONDITIONAL_BATCH_BUFFER_END | MI_DO_COMPARE | (1 + use_64b);
	*cs++ = -2; /* break if *addr < -1 */
	*cs++ = lower_32_bits(addr);
	*cs++ = upper_32_bits(addr);
	clear_dw(va, addr, -1);
	clear_dw(vb, addr, -1);

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

	pr_info("%s(%s): Sampling %llx, with alignment %llx, using PTE size %x\n",
		ce->engine->name, va->obj->mm.region.mem->name ?: "smem",
		addr, align, va->page_sizes);
	if (va->page_sizes != align && va->page_sizes > va->size) {
		dma_addr_t dma = i915_gem_object_get_dma_address(va->obj, 0);

		pr_warn("%s(%s): Failed to insert a suitably large PTE for dma addr:%pa\n",
			ce->engine->name, va->obj->mm.region.mem->name ?: "smem",
			&dma);
	}

	/* Short sleep to sanitycheck the batch is spinning before we begin */
	msleep(10);
	if (!i915_request_completed(rq)) {
		struct i915_vma *vv[] = { va, vb };
		IGT_TIMEOUT(end_time);

		while (!__igt_timeout(end_time, NULL)) {
			struct i915_vm_pt_stash stash = {};
			unsigned int pte_flags = 0;
			struct i915_gem_ww_ctx ww;

			err = i915_vm_alloc_pt_stash(ce->vm, &stash, vv[0]->size);
			if (err)
				break;

			for_i915_gem_ww(&ww, err, false) {
				err = i915_vm_lock_objects(ce->vm, &ww);
				if (err)
					continue;

				err = i915_vm_map_pt_stash(ce->vm, &stash);
			}
			if (err) {
				i915_vm_free_pt_stash(ce->vm, &stash);
				break;
			}

			/* Flip the PTE between A and B */
			if (i915_gem_object_is_lmem(vv[0]->obj))
				pte_flags |= PTE_LM;
			ce->vm->insert_entries(ce->vm, &stash, vv[0], pat_index, pte_flags);

			i915_vm_free_pt_stash(ce->vm, &stash);

			/* Check if the semaphore read anywhere other than A|B */
			if (i915_request_completed(rq)) {
				pr_err("Request completed early; invalid sample detected by %s with alignment 0x%llx\n",
				       ce->engine->name, align);
				err = -EINVAL;
				break;
			}

			swap(vv[0], vv[1]);
		}
	} else {
		pr_err("Spinner sanitycheck failed\n");
		err = -EIO;
	}
	i915_request_put(rq);

	cs = page_mask_bits(batch->mm.mapping);
	*cs = MI_BATCH_BUFFER_END;
	wmb();

out_va:
	memset(&vb->node, 0, sizeof(vb->node));
	i915_vma_unpin(va);
	if (i915_vma_unbind(va))
		err = -EIO;
out:
	i915_gem_object_put(batch);
	return err;
}

static int
pte_invalid_read(struct intel_context *ce,
		 struct i915_vma *va,
		 struct i915_vma *vb,
		 u64 align,
		 struct rnd_state *prng)
{
	const int use_64b = GRAPHICS_VER(ce->vm->i915) >= 8;
	struct drm_i915_gem_object *batch;
	struct i915_request *rq;
	struct i915_vma *vma;
	int retries;
	u64 addr;
	int err;
	u32 *cs;

	if (ce->engine->class != COPY_ENGINE_CLASS) /* MI invalidate TLB */
		return 0;

	batch = i915_gem_object_create_internal(ce->vm->i915, 4096);
	if (IS_ERR(batch))
		return PTR_ERR(batch);

	vma = i915_vma_instance(batch, ce->vm, NULL);
	if (IS_ERR(vma)) {
		err = PTR_ERR(vma);
		goto out;
	}

	err = i915_vma_pin(vma, 0, 0, PIN_USER);
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

	if (align == SZ_64K) {
		u64 end = addr + va->size;

		addr = round_up(addr & -align, SZ_2M);
		addr |=	igt_random_offset(prng, 0, end - addr, 4, 4);
	}

	if (addr - i915_vma_offset(va) >= i915_vma_size(va))
		addr = igt_random_offset(prng,
					 i915_vma_offset(va),
					 i915_vma_offset(va) + i915_vma_size(va),
					 4, 4);

	pr_info("%s(%s): Sampling %llx, with alignment %llx, using PTE size %x\n",
		ce->engine->name, va->obj->mm.region.mem->name ?: "smem",
		addr, align, va->page_sizes);
	if (va->page_sizes != align && va->page_sizes > va->size) {
		dma_addr_t dma = i915_gem_object_get_dma_address(va->obj, 0);

		pr_warn("%s(%s): Failed to insert a suitably large PTE for dma addr:%pa\n",
			ce->engine->name, va->obj->mm.region.mem->name ?: "smem",
			&dma);
	}

	cs = i915_gem_object_pin_map_unlocked(batch, I915_MAP_WC);
	*cs++ = MI_NOOP; /* for later termination */

	/* Sample the target to see if we spot an incorrect page */
	cs = __gen8_emit_flush_dw(cs, 0, i915_vma_offset(vma) + 4000,
				  MI_INVALIDATE_TLB | MI_FLUSH_DW_OP_STOREDW);
	*cs++ = MI_CONDITIONAL_BATCH_BUFFER_END | MI_DO_COMPARE | (1 + use_64b);
	*cs++ = 0; /* end if *addr == 0 */
	*cs++ = lower_32_bits(addr);
	*cs++ = upper_32_bits(addr);
	clear_dw(va, addr, va->vm->poison);

	/* Keep sampling until we get bored */
	*cs++ = MI_BATCH_BUFFER_START | BIT(8) | use_64b;
	*cs++ = lower_32_bits(i915_vma_offset(vma));
	*cs++ = upper_32_bits(i915_vma_offset(vma));

	i915_gem_object_flush_map(batch);

	i915_vma_unpin(va);
	err = i915_vma_unbind(va);
	if (err)
		goto out;

	rq = i915_request_create(ce);
	if (IS_ERR(rq)) {
		err = PTR_ERR(rq);
		goto out;
	}

	err = rq->engine->emit_bb_start(rq, i915_vma_offset(vma), 0, 0);
	if (err) {
		i915_request_add(rq);
		goto out;
	}

	i915_request_get(rq);
	i915_request_add(rq);

	/* Short sleep to sanitycheck the batch is spinning before we begin */
	msleep(10);
	if (!i915_request_completed(rq)) {
		IGT_TIMEOUT(end_time);

		while (err == 0 && !__igt_timeout(end_time, NULL)) {
			err = i915_vma_pin(va, 0, 0, (addr & -align) | PIN_OFFSET_FIXED | PIN_USER);
			if (err == 0) {
				i915_vma_unpin(va);
				err = i915_vma_unbind(va);
			}

			/* Check if the semaphore read anywhere else */
			if (i915_request_completed(rq)) {
				pr_err("Request completed early; invalid sample detected by %s with alignment 0x%llx\n",
				       ce->engine->name, align);
				err = -EINVAL;
				break;
			}
		}
	} else {
		pr_err("Spinner sanitycheck failed\n");
		err = -EIO;
	}
	i915_request_put(rq);

	if (i915_vma_pin(va, 0, 0, (addr & -align) | PIN_OFFSET_FIXED | PIN_USER))
		pr_err("%s: Failed to restore semaphore upon exit\n", ce->engine->name);
	cs = page_mask_bits(batch->mm.mapping);
	*cs = MI_BATCH_BUFFER_END;
	wmb();

	i915_vma_unpin(va);
	if (i915_vma_unbind(va))
		err = -EIO;
out:
	i915_gem_object_put(batch);
	return err;
}

static struct drm_i915_gem_object *create_lmem(struct intel_gt *gt)
{
	return intel_gt_object_create_lmem(gt, SZ_1G, I915_BO_ALLOC_CONTIGUOUS);
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
mem_write_tearing(struct intel_gt *gt,
		  struct drm_i915_gem_object *(*create_fn)(struct intel_gt *),
		  int (*pte_fn)(struct intel_context *ce,
				struct i915_vma *va,
				struct i915_vma *vb,
				u64 align,
				struct rnd_state *prng),
		  unsigned int flags)
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

	GEM_BUG_ON(!is_power_of_2(A->base.size));
	GEM_BUG_ON(A->base.size != B->base.size);
	if (!sg_is_last(A->mm.pages->sgl) || !sg_is_last(B->mm.pages->sgl))
		pr_warn("Failed to allocate contiguous pages for size %zx\n",
			A->base.size);

	ppgtt = i915_ppgtt_create(gt, flags);
	if (IS_ERR(ppgtt)) {
		err = PTR_ERR(ppgtt);
		goto out_b;
	}
	if (ppgtt->vm.poison != -1 && ppgtt->vm.scratch[0]) {
		struct drm_i915_gem_object *scratch = ppgtt->vm.scratch[0];

		ppgtt->vm.poison = 0xffffffff;
		memset32(__px_vaddr(scratch, NULL), ppgtt->vm.poison, scratch->base.size / sizeof(u32));
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
			unsigned long sizes = INTEL_INFO(gt->i915)->page_sizes;

			for_each_set_bit(bit, &sizes, BITS_PER_LONG) {
				err = pte_fn(ce, va, vb, BIT_ULL(bit), &prng);
				if (err)
					break;
			}
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

static int write_tearing(void *arg)
{
	struct intel_gt *gt = arg;
	int err;

	/*
	 * Our goal is to try and detect if the HW sees partial PTE updates
	 * (write tearing where the the HW reads the 64b PTE as 2 separate 32b
	 * dwords, and in doing so may see different upper/lower dwords). This
	 * may be due to either the HW or CPU performing the PTE read/write
	 * as two 32b operations instead of a single 64b operation.
	 *
	 * We use 3 pages: scratch, A and B. The entire GTT is filled with
	 * scratch, any invalid virtual address will read scratch, but an invalid
	 * physical address may be anywhere. Using the system memory and iommu
	 * should detect stray physical address lookups, but using local
	 * memory we are more likely to be able to allocate huge pages. And
	 * then we try and switch the target physical address between pointing
	 * at A and B constantly and check that all HW reads sample only A and
	 * B respectively. If we see scratch or other CAT error, then we know
	 * the HW formed a different physical address than A or B.
	 */

	err = mem_write_tearing(gt, create_smem, pte_write_tearing, 0);
	if (err == 0)
		err = mem_write_tearing(gt, create_lmem, pte_write_tearing, 0);
	if (err == -ENODEV || err == -ENXIO)
		err = 0;

	return err;
}

static int invalid_read(void *arg)
{
	struct intel_gt *gt = arg;
	int err;

	/*
	 * Try to sample an unbound address while simultaneusly binding
	 * an object there. If we update the PTE in the GTT incorrectly,
	 * it is possible for the HW to see an invalid entry during the
	 * update and stray into the wilds.
	 */

	err = mem_write_tearing(gt, create_smem, pte_invalid_read, 0);
	if (err == 0)
		err = mem_write_tearing(gt, create_lmem, pte_invalid_read, 0);
	if (err == -ENODEV || err == -ENXIO)
		err = 0;

	return err;
}

static int invalid_fault(void *arg)
{
	struct intel_gt *gt = arg;
	int err;

	/*
	 * Similar to invalid_read, try to sample an unbound address
	 * triggering a pagefault, while simultaneously attempting
	 * to rebind that address.
	 */

	if (!HAS_RECOVERABLE_PAGE_FAULT(gt->i915))
		return 0;

	err = mem_write_tearing(gt, create_smem, pte_invalid_read,
				PRELIM_I915_VM_CREATE_FLAGS_ENABLE_PAGE_FAULT);
	if (err == 0)
		err = mem_write_tearing(gt, create_lmem, pte_invalid_read,
					PRELIM_I915_VM_CREATE_FLAGS_ENABLE_PAGE_FAULT);
	if (err == -ENODEV || err == -ENXIO)
		err = 0;

	return err;
}

static int __pat_speed(struct intel_context *ce,
		       struct drm_i915_gem_object *obj)
{
	struct intel_gt *gt = ce->engine->gt;
	struct i915_gem_ww_ctx ww;
	struct i915_vma *vma;
	intel_wakeref_t wf;
	int cache_level;
	u64 *cycles;
	int err;

	cycles = i915_gem_object_pin_map_unlocked(obj, I915_MAP_WC);
	if (IS_ERR(cycles))
		return PTR_ERR(cycles);

	vma = i915_vma_instance(obj, ce->vm, 0);
	if (IS_ERR(vma)) {
		err = PTR_ERR(vma);
		goto out_map;
	}

	err = i915_vma_pin(vma, 0, obj->base.size, PIN_USER);
	if (err)
		goto out_map;

	wf = intel_gt_pm_get(gt);
	intel_rps_boost(&gt->rps);

	for (cache_level = 0; cache_level < I915_MAX_CACHE_LEVEL; cache_level++) {
		struct i915_vm_pt_stash stash = {};
		struct i915_request *rq;
		unsigned int pte_flags;
		unsigned int fill_bw;
		uint64_t elapsed;
		int pat_index;

		err = i915_vm_alloc_pt_stash(ce->vm, &stash, vma->size);
		if (err)
			break;

		for_i915_gem_ww(&ww, err, false) {
			err = i915_vm_lock_objects(ce->vm, &ww);
			if (err)
				continue;

			err = i915_vm_map_pt_stash(ce->vm, &stash);
		}
		if (err) {
			i915_vm_free_pt_stash(ce->vm, &stash);
			break;
		}

		pte_flags = 0;
		if (i915_gem_object_is_lmem(obj))
			pte_flags |= PTE_LM;

		pat_index = i915_gem_get_pat_index(gt->i915, cache_level);

		ce->vm->insert_entries(ce->vm, &stash, vma, pat_index, pte_flags);
		i915_vm_free_pt_stash(ce->vm, &stash);

		rq = i915_request_create(ce);
		if (IS_ERR(rq)) {
			err = PTR_ERR(rq);
			break;
		}

		err = emit_start_timestamp(rq);
		if (err == 0)
			err = emit_memset(rq, vma);
		if (err == 0)
			err = emit_write_elaspsed(rq, i915_vma_offset(vma));

		i915_request_get(rq);
		i915_request_add(rq);
		if (i915_request_wait(rq, I915_WAIT_INTERRUPTIBLE, 5 * HZ) < 0)
			err = -ETIME;
		i915_request_put(rq);
		if (err)
			break;

		elapsed = intel_gt_clock_interval_to_ns(gt, READ_ONCE(*cycles));
		if (!elapsed)
			continue;

		fill_bw = div_u64(mul_u64_u32_shr(obj->base.size, NSEC_PER_SEC, 20), elapsed);

		dev_info(gt->i915->drm.dev,
			 "GT%d: %s, cache-level:%d, pat-index:%d, size:%zx, time:%lldns, GPU fill:%dMiB/s\n",
			 gt->info.id, obj->mm.region.mem->name,
			 cache_level, pat_index,
			 obj->base.size, elapsed, fill_bw);
	}

	intel_rps_cancel_boost(&gt->rps);
	intel_gt_pm_put(gt, wf);

	i915_vma_unpin(vma);
out_map:
	i915_gem_object_unpin_map(obj);
	return err;
}

static int pat_speed(void *arg)
{
	struct intel_gt *gt = arg;
	struct drm_i915_gem_object *obj;
	struct intel_context *ce;
	int err = 0;

	if (GRAPHICS_VER(gt->i915) < 9)
		return 0;

	ce = NULL;
	if (gt->engine_class[COPY_ENGINE_CLASS][0])
		ce = gt->engine_class[COPY_ENGINE_CLASS][0]->kernel_context;
	if (!ce)
		return 0;

	if (gt->lmem) {
		obj = i915_gem_object_create_region(gt->lmem, SZ_1G, 0);
		if (!IS_ERR(obj)) {
			err = __pat_speed(ce, obj);
			i915_gem_object_put(obj);
			if (err)
				goto out;
		}
	}

	obj = i915_gem_object_create_internal(gt->i915, SZ_64M);
	if (!IS_ERR(obj)) {
		err = __pat_speed(ce, obj);
		i915_gem_object_put(obj);
		if (err)
			goto out;
	}

out:
	if (igt_flush_test(gt->i915))
		err = -EIO;
	return err;
}

int intel_gtt_live_selftests(struct drm_i915_private *i915)
{
	static const struct i915_subtest tests[] = {
		SUBTEST(direct_store),
		SUBTEST(direct_mov),
		SUBTEST(direct_inc),
		SUBTEST(direct_dec),
		SUBTEST(pat_speed),
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

int intel_gtt_wip_selftests(struct drm_i915_private *i915)
{
	static const struct i915_subtest tests[] = {
		SUBTEST(write_tearing),
		SUBTEST(invalid_read),
		SUBTEST(invalid_fault),
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
