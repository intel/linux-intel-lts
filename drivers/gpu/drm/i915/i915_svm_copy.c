// SPDX-License-Identifier: MIT
/*
 * Copyright © 2019 Intel Corporation
 */

#include "gt/intel_context.h"
#include "gt/intel_engine_pm.h"
#include "gt/intel_gpu_commands.h"
#include "gt/intel_gt.h"
#include "gt/intel_gt_buffer_pool.h"

#include "i915_drv.h"
#include "i915_svm.h"

static int intel_emit_vma_mark_active(struct i915_vma *vma,
				      struct i915_request *rq)
{
	int err;

	err = i915_request_await_object(rq, vma->obj, false);
	if (err == 0)
		err = i915_vma_move_to_active(vma, rq, 0);
	if (unlikely(err))
		return err;

	return intel_gt_buffer_pool_mark_active(vma->private, rq);
}

static void intel_emit_vma_release(struct intel_context *ce,
				   struct i915_vma *vma)
{
	i915_vma_unpin(vma);
	intel_gt_buffer_pool_put(vma->private);
	intel_engine_pm_put(ce->engine);
}

static struct i915_vma *
intel_emit_svm_copy_blt(struct intel_context *ce,
			struct i915_gem_ww_ctx *ww,
			u64 src_start, u64 dst_start, u64 buff_size)
{
	struct drm_i915_private *i915 = ce->vm->i915;
	const u32 block_size = SZ_8M; /* ~1ms at 8GiB/s preemption delay */
	struct intel_gt_buffer_pool_node *pool;
	struct i915_vma *batch;
	u64 count, rem;
	u32 size, *cmd;
	int err;

	GEM_BUG_ON(intel_engine_is_virtual(ce->engine));
	intel_engine_pm_get(ce->engine);

	if (GRAPHICS_VER(i915) < 8)
		return ERR_PTR(-ENOTSUPP);

	count = div_u64(round_up(buff_size, block_size), block_size);
	size = (1 + 11 * count) * sizeof(u32);
	size = round_up(size, PAGE_SIZE);
	pool = intel_gt_get_buffer_pool(ce->engine->gt, size, I915_MAP_WC);
	if (IS_ERR(pool)) {
		err = PTR_ERR(pool);
		goto out_pm;
	}

	err = i915_gem_object_lock(pool->obj, ww);
	if (err)
		goto out_put;

	batch = i915_vma_instance(pool->obj, ce->vm, NULL);
	if (IS_ERR(batch)) {
		err = PTR_ERR(batch);
		goto out_put;
	}

	err = i915_vma_pin_ww(batch, ww, 0, 0, PIN_USER);
	if (unlikely(err))
		goto out_put;

	/* we pinned the pool, mark it as such */
	intel_gt_buffer_pool_mark_used(pool);

	cmd = i915_gem_object_pin_map(pool->obj, pool->type);
	if (IS_ERR(cmd)) {
		err = PTR_ERR(cmd);
		goto out_unpin;
	}

	rem = buff_size;
	do {
		size = min_t(u64, rem, block_size);
		GEM_BUG_ON(size >> PAGE_SHIFT > S16_MAX);

		if (GRAPHICS_VER(i915) >= 9) {
			*cmd++ = GEN9_XY_FAST_COPY_BLT_CMD | (10 - 2);
			*cmd++ = BLT_DEPTH_32 | PAGE_SIZE;
			*cmd++ = 0;
			*cmd++ = size >> PAGE_SHIFT << 16 | PAGE_SIZE / 4;
			*cmd++ = lower_32_bits(dst_start);
			*cmd++ = upper_32_bits(dst_start);
			*cmd++ = 0;
			*cmd++ = PAGE_SIZE;
			*cmd++ = lower_32_bits(src_start);
			*cmd++ = upper_32_bits(src_start);
		} else if (GRAPHICS_VER(i915) >= 8) {
			*cmd++ = XY_SRC_COPY_BLT_CMD |
				 BLT_WRITE_RGBA | (10 - 2);
			*cmd++ = BLT_DEPTH_32 | BLT_ROP_SRC_COPY | PAGE_SIZE;
			*cmd++ = 0;
			*cmd++ = size >> PAGE_SHIFT << 16 | PAGE_SIZE / 4;
			*cmd++ = lower_32_bits(dst_start);
			*cmd++ = upper_32_bits(dst_start);
			*cmd++ = 0;
			*cmd++ = PAGE_SIZE;
			*cmd++ = lower_32_bits(src_start);
			*cmd++ = upper_32_bits(src_start);
		}

		/* Allow ourselves to be preempted in between blocks */
		*cmd++ = MI_ARB_CHECK;

		src_start += size;
		dst_start += size;
		rem -= size;
	} while (rem);

	*cmd = MI_BATCH_BUFFER_END;
	intel_gt_chipset_flush(ce->vm->gt);

	i915_gem_object_unpin_map(pool->obj);

	batch->private = pool;
	return batch;

out_unpin:
	i915_vma_unpin(batch);
out_put:
	intel_gt_buffer_pool_put(pool);
out_pm:
	intel_engine_pm_put(ce->engine);
	return ERR_PTR(err);
}

int i915_svm_copy_blt(struct intel_context *ce,
		      struct i915_gem_ww_ctx *ww,
		      u64 src_start, u64 dst_start, u64 size,
		      struct dma_fence **fence)
{
	struct i915_request *rq;
	struct i915_vma *batch;
	int err;

	DRM_DEBUG_DRIVER("src_start 0x%llx dst_start 0x%llx size 0x%llx\n",
			 src_start, dst_start, size);

	intel_engine_pm_get(ce->engine);
	err = intel_context_pin_ww(ce, ww);
	if (err)
		goto out;

	batch = intel_emit_svm_copy_blt(ce, ww, src_start, dst_start, size);
	if (IS_ERR(batch)) {
		err = PTR_ERR(batch);
		goto out_ctx;
	}

	rq = i915_request_create(ce);
	if (IS_ERR(rq)) {
		err = PTR_ERR(rq);
		goto out_batch;
	}

	err = intel_emit_vma_mark_active(batch, rq);
	if (unlikely(err))
		goto out_request;

	if (ce->engine->emit_init_breadcrumb) {
		err = ce->engine->emit_init_breadcrumb(rq);
		if (unlikely(err))
			goto out_request;
	}

	err = rq->engine->emit_bb_start(rq,
					i915_vma_offset(batch),
					i915_vma_size(batch),
					0);
out_request:
	if (unlikely(err))
		i915_request_set_error_once(rq, err);
	else
		*fence = &rq->fence;

	i915_request_add(rq);
out_batch:
	i915_gem_ww_unlock_single(batch->obj);
	intel_emit_vma_release(ce, batch);
out_ctx:
	intel_context_unpin(ce);
out:
	intel_engine_pm_put(ce->engine);
	return err;
}
