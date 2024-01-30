/*
 * Copyright © 2016 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 */

#include <linux/sched/mm.h>
#include <drm/drm_gem.h>

#include "display/intel_frontbuffer.h"
#include "gem/i915_gem_lmem.h"
#include "gem/i915_gem_tiling.h"
#include "gem/i915_gem_vm_bind.h"
#include "gt/gen8_ppgtt.h"
#include "gt/intel_engine.h"
#include "gt/intel_engine_heartbeat.h"
#include "gt/intel_flat_ppgtt_pool.h"
#include "gt/intel_gt.h"
#include "gt/intel_gt_requests.h"
#include "gt/intel_gt_pm.h"
#include "gt/intel_tlb.h"

#include "i915_debugger.h"
#include "i915_drv.h"
#include "i915_gem_evict.h"
#include "i915_sw_fence_work.h"
#include "i915_trace.h"
#include "i915_vma.h"
#include "gem/i915_gem_vm_bind.h"

static struct kmem_cache *slab_vmas;

struct i915_vma *i915_vma_alloc(void)
{
	return kmem_cache_zalloc(slab_vmas, GFP_KERNEL);
}

void i915_vma_free(struct i915_vma *vma)
{
	return kmem_cache_free(slab_vmas, vma);
}

#if IS_ENABLED(CONFIG_DRM_I915_ERRLOG_GEM) && IS_ENABLED(CONFIG_DRM_DEBUG_MM)

#include <linux/stackdepot.h>

static void vma_print_allocator(struct i915_vma *vma, const char *reason)
{
	char buf[512];

	if (!vma->node.stack) {
		DRM_DEBUG_DRIVER("vma.node [%08llx + %08llx] %s: unknown owner\n",
				 vma->node.start, vma->node.size, reason);
		return;
	}

	stack_depot_snprint(vma->node.stack, buf, sizeof(buf), 0);
	DRM_DEBUG_DRIVER("vma.node [%08llx + %08llx] %s: inserted at %s\n",
			 vma->node.start, vma->node.size, reason, buf);
}

#else

static void vma_print_allocator(struct i915_vma *vma, const char *reason)
{
}

#endif

static inline struct i915_vma *active_to_vma(struct i915_active *ref)
{
	return container_of(ref, typeof(struct i915_vma), active);
}

static int __i915_vma_active(struct i915_active *ref)
{
	return i915_vma_open(active_to_vma(ref)) ? 0 : -ENOENT;
}

static void __i915_vma_retire(struct i915_active *ref)
{
	i915_vma_close(active_to_vma(ref));
}

static struct i915_vma *
vma_create(struct drm_i915_gem_object *obj,
	   struct i915_address_space *vm,
	   const struct i915_ggtt_view *view)
{
	struct i915_vma *pos = ERR_PTR(-E2BIG);
	struct i915_vma *vma;
	struct rb_node *rb, **p;

	/* The aliasing_ppgtt should never be used directly! */
	GEM_BUG_ON(vm == &vm->gt->ggtt->alias->vm);

	vma = i915_vma_alloc();
	if (vma == NULL)
		return ERR_PTR(-ENOMEM);

	kref_init(&vma->ref);
	mutex_init(&vma->pages_mutex);
	vma->vm = i915_vm_get(vm);
	vma->ops = &vm->vma_ops;
	vma->obj = obj;
	vma->resv = obj->base.resv;
	vma->size = obj->base.size;
	vma->display_alignment = I915_GTT_MIN_ALIGNMENT;

	i915_active_init(&vma->active, __i915_vma_active, __i915_vma_retire, 0);

	/* Declare ourselves safe for use inside shrinkers */
	if (IS_ENABLED(CONFIG_LOCKDEP)) {
		fs_reclaim_acquire(GFP_KERNEL);
		might_lock(&vma->active.mutex);
		fs_reclaim_release(GFP_KERNEL);
	}

	INIT_LIST_HEAD(&vma->closed_link);
	vma->pool = NULL;

	if (view && view->type != I915_GGTT_VIEW_NORMAL) {
		vma->ggtt_view = *view;
		if (view->type == I915_GGTT_VIEW_PARTIAL) {
			GEM_BUG_ON(range_overflows_t(u64,
						     view->partial.offset,
						     view->partial.size,
						     obj->base.size >> PAGE_SHIFT));
			vma->size = view->partial.size;
			vma->size <<= PAGE_SHIFT;
			GEM_BUG_ON(vma->size > obj->base.size);
		} else if (view->type == I915_GGTT_VIEW_ROTATED) {
			vma->size = intel_rotation_info_size(&view->rotated);
			vma->size <<= PAGE_SHIFT;
		} else if (view->type == I915_GGTT_VIEW_REMAPPED) {
			vma->size = intel_remapped_info_size(&view->remapped);
			vma->size <<= PAGE_SHIFT;
		}
	}

	if (unlikely(vma->size > vm->total))
		goto err_vma;

	GEM_BUG_ON(!IS_ALIGNED(vma->size, I915_GTT_PAGE_SIZE));

	spin_lock(&obj->vma.lock);

	if (i915_is_ggtt(vm)) {
		if (unlikely(overflows_type(vma->size, u32)))
			goto err_unlock;

		vma->fence_size = i915_gem_fence_size(vm->i915, vma->size,
						      i915_gem_object_get_tiling(obj),
						      i915_gem_object_get_stride(obj));
		if (unlikely(vma->fence_size < vma->size || /* overflow */
			     vma->fence_size > vm->total))
			goto err_unlock;

		GEM_BUG_ON(!IS_ALIGNED(vma->fence_size, I915_GTT_MIN_ALIGNMENT));

		vma->fence_alignment = i915_gem_fence_alignment(vm->i915, vma->size,
								i915_gem_object_get_tiling(obj),
								i915_gem_object_get_stride(obj));
		GEM_BUG_ON(!is_power_of_2(vma->fence_alignment));

		__set_bit(I915_VMA_GGTT_BIT, __i915_vma_flags(vma));
	}

	if (!i915_vma_is_ggtt(vma) &&
	    (view && view->type == I915_GGTT_VIEW_PARTIAL))
		goto skip_rb_insert;

	rb = NULL;
	p = &obj->vma.tree.rb_node;
	while (*p) {
		long cmp;

		rb = *p;
		pos = rb_entry(rb, struct i915_vma, obj_node);

		/*
		 * If the view already exists in the tree, another thread
		 * already created a matching vma, so return the older instance
		 * and dispose of ours.
		 */
		cmp = i915_vma_compare(pos, vm, view);
		if (cmp < 0)
			p = &rb->rb_right;
		else if (cmp > 0)
			p = &rb->rb_left;
		else
			goto err_unlock;
	}
	rb_link_node(&vma->obj_node, rb, p);
	rb_insert_color(&vma->obj_node, &obj->vma.tree);

skip_rb_insert:
	if (i915_vma_is_ggtt(vma))
		/*
		 * We put the GGTT vma at the start of the vma-list, followed
		 * by the ppGGTT vma. This allows us to break early when
		 * iterating over only the GGTT vma for an object, see
		 * for_each_ggtt_vma()
		 */
		list_add(&vma->obj_link, &obj->vma.list);
	else
		list_add_tail(&vma->obj_link, &obj->vma.list);

	spin_unlock(&obj->vma.lock);

	spin_lock_init(&vma->metadata_lock);
	INIT_LIST_HEAD(&vma->metadata_list);
	INIT_LIST_HEAD(&vma->vm_bind_link);
	INIT_LIST_HEAD(&vma->vm_capture_link);
	return vma;

err_unlock:
	spin_unlock(&obj->vma.lock);
err_vma:
	i915_vm_put(vm);
	i915_vma_free(vma);
	return pos;
}

static struct i915_vma *
i915_vma_lookup(struct drm_i915_gem_object *obj,
	   struct i915_address_space *vm,
	   const struct i915_ggtt_view *view)
{
	struct rb_node *rb;

	rb = obj->vma.tree.rb_node;
	while (rb) {
		struct i915_vma *vma = rb_entry(rb, struct i915_vma, obj_node);
		long cmp;

		cmp = i915_vma_compare(vma, vm, view);
		if (cmp == 0)
			return vma;

		if (cmp < 0)
			rb = rb->rb_right;
		else
			rb = rb->rb_left;
	}

	return NULL;
}

/**
 * i915_vma_instance - return the singleton instance of the VMA
 * @obj: parent &struct drm_i915_gem_object to be mapped
 * @vm: address space in which the mapping is located
 * @view: additional mapping requirements
 *
 * i915_vma_instance() looks up an existing VMA of the @obj in the @vm with
 * the same @view characteristics. If a match is not found, one is created.
 * Once created, the VMA is kept until either the object is freed, or the
 * address space is closed.
 *
 * Returns the vma, or an error pointer.
 */
struct i915_vma *
i915_vma_instance(struct drm_i915_gem_object *obj,
		  struct i915_address_space *vm,
		  const struct i915_ggtt_view *view)
{
	struct i915_vma *vma = NULL;

	GEM_BUG_ON(!atomic_read(&vm->open));

	if (i915_is_ggtt(vm) || !view ||
	    view->type != I915_GGTT_VIEW_PARTIAL) {
		spin_lock(&obj->vma.lock);
		vma = i915_vma_lookup(obj, vm, view);
		spin_unlock(&obj->vma.lock);
	}

	/* vma_create() will resolve the race if another creates the vma */
	if (unlikely(!vma))
		vma = vma_create(obj, vm, view);

	GEM_BUG_ON(!IS_ERR(vma) && i915_vma_compare(vma, vm, view));
	return vma;
}

struct i915_vma_work {
	struct dma_fence_work base;
	struct i915_address_space *vm;
	struct i915_vm_pt_stash stash;
	struct i915_vma *vma;
	unsigned int pat_index;
	unsigned int flags;
};

static int
i915_vma_work_set_vm(struct i915_vma_work *work,
		     struct i915_vma *vma,
		     struct i915_gem_ww_ctx *ww,
		     unsigned int flags)
{
	if (vma->vm->allocate_va_range && flags & PIN_RESIDENT) {
		int err;

		/* Allocate enough page directories to used PTE */
		err = i915_vm_alloc_pt_stash(vma->vm, &work->stash, vma->size);
		if (err)
			return err;

		err = i915_vm_lock_objects(vma->vm, ww);
		if (err)
			return err;

		err = i915_vm_map_pt_stash(vma->vm, &work->stash);
		if (err)
			return err;
	}

	return 0;
}

static int __vma_bind(struct dma_fence_work *work)
{
	struct i915_vma_work *vw = container_of(work, typeof(*vw), base);
	struct i915_vma *vma = vw->vma;

	vma->ops->bind_vma(vw->vm, &vw->stash, vma, vw->pat_index, vw->flags);

	return 0;
}

static void __vma_user_fence_signal(struct i915_vma *vma, int err)
{
	struct i915_sw_fence *f;

	f = fetch_and_zero(&vma->bind_fence);
	if (!f)
		return;

	/* Do not update the userfence value if we do not submit the bind */
	if (err == -ERESTARTSYS)
		i915_sw_fence_set_error_once(f, err);

	i915_sw_fence_complete(f);
}

static void __vma_complete(struct dma_fence_work *work)
{
	struct i915_vma_work *vw = container_of(work, typeof(*vw), base);
	struct i915_vma *vma;

	vma = vw->vma;
	if (!vma)
		return;

	if (work->rq.fence.error && work->rq.fence.error != -ERESTARTSYS) {
		GEM_BUG_ON(test_bit(DMA_FENCE_FLAG_SIGNALED_BIT,
				    &work->rq.fence.flags));
		set_bit(I915_VMA_ERROR_BIT, __i915_vma_flags(vma));
		if (vma->vm->scratch_range)
			vma->vm->scratch_range(vma->vm, vma->node.start, vma->node.size);
	}

	__vma_user_fence_signal(vma, work->rq.fence.error);
	intel_flat_ppgtt_request_pool_clean(vma);

	i915_active_release(&vma->active);
}

static void __vma_release(struct dma_fence_work *work)
{
	struct i915_vma_work *vw = container_of(work, typeof(*vw), base);

	i915_vm_free_pt_stash(vw->vm, &vw->stash);
	i915_vm_put(vw->vm);
}

static const struct dma_fence_work_ops bind_ops = {
	.name = "bind",
	.work = __vma_bind,
	.complete = __vma_complete,
	.release = __vma_release,
};

static struct i915_vma_work *i915_vma_work(struct i915_address_space *vm)
{
	struct i915_vma_work *vw;

	vw = kzalloc(sizeof(*vw), GFP_KERNEL);
	if (!vw)
		return NULL;

	dma_fence_work_init(&vw->base, &bind_ops, vm->i915->sched);
	vw->base.rq.fence.error = -ERESTARTSYS; /* disable the worker by default */
	vw->vm = i915_vm_get(vm);

	return vw;
}

int i915_vma_wait_for_bind(struct i915_vma *vma)
{
	struct dma_fence *fence;
	int err;

	fence = i915_active_fence_get_or_error(&vma->active.excl);
	if (likely(!fence))
		return 0;

	if (IS_ERR(fence))
		return PTR_ERR(fence);

	err = dma_fence_wait(fence, true) ? -EINTR : fence->error;
	dma_fence_put(fence);

	return err;
}

/**
 * __i915_vma_bind - Sets up PTEs for an VMA in it's corresponding address space.
 * @vma: VMA to map
 * @pat_index: PAT index to set in PTE
 * @flags: flags like global or local mapping
 * @work: preallocated worker for allocating and binding the PTE
 *
 * DMA addresses are taken from the scatter-gather table of this object (or of
 * this VMA in case of non-default GGTT views) and PTE entries set up.
 * Note that DMA addresses are also the only part of the SG table we care about.
 */
int __i915_vma_bind(struct i915_vma *vma,
		    unsigned int pat_index,
		    u32 flags,
		    struct i915_vma_work *work)
{
	u32 bind_flags;
	u32 vma_flags;

	GEM_BUG_ON(!drm_mm_node_allocated(&vma->node));
	GEM_BUG_ON(vma->size > i915_vma_size(vma));

	if (GEM_DEBUG_WARN_ON(range_overflows(vma->node.start,
					      vma->node.size,
					      vma->vm->total)))
		return -ENODEV;

	if (GEM_DEBUG_WARN_ON(!flags))
		return -EINVAL;

	bind_flags = flags;
	bind_flags &= I915_VMA_GLOBAL_BIND | I915_VMA_LOCAL_BIND;

	vma_flags = atomic_read(&vma->flags);
	vma_flags &= I915_VMA_GLOBAL_BIND | I915_VMA_LOCAL_BIND;

	bind_flags &= ~vma_flags;
	if (bind_flags == 0)
		return 0;

	trace_i915_vma_bind(vma, bind_flags);
	if (work) {
		struct dma_fence *prev;

		work->vma = vma;
		work->pat_index = pat_index;
		work->flags = bind_flags;
		__i915_active_acquire(&vma->active);

		/*
		 * Note we only want to chain up to the migration fence on
		 * the pages (not the object itself). As we don't track that,
		 * yet, we have to use the exclusive fence instead.
		 *
		 * Also note that we do not want to track the async vma as
		 * part of the obj->resv->excl_fence as it only affects
		 * execution and not content or object's backing store lifetime.
		 */
		prev = i915_active_set_exclusive(&vma->active,
						 &work->base.rq.fence);
		if (!prev) {
			/*
			 * Before we publish the backing store to any and all
			 * observers via the ppGTT, wait for all residual data
			 * to be cleared or overwritten by swapin.
			 */
			prev = i915_active_fence_get_or_error(&vma->obj->mm.migrate);
		}
		if (prev) {
			if (IS_ERR(prev))
				return PTR_ERR(prev);

			dma_fence_work_chain(&work->base, prev);
			dma_fence_put(prev);
		}

		work->base.rq.fence.error = 0; /* enable the queue_work() */
	} else {
		vma->ops->bind_vma(vma->vm, NULL, vma, pat_index, bind_flags);
	}

	atomic_or(bind_flags, &vma->flags);
	return 0;
}

void __iomem *i915_vma_pin_iomap(struct i915_vma *vma)
{
	void __iomem *ptr;
	int err;

	GEM_BUG_ON(!i915_vma_is_ggtt(vma));
	GEM_BUG_ON(!i915_vma_is_bound(vma, I915_VMA_GLOBAL_BIND));

	ptr = READ_ONCE(vma->iomap);
	if (ptr == NULL) {
		if (i915_vma_is_map_and_fenceable(vma)) {
			err = i915_vma_wait_for_bind(vma);
			if (err)
				goto err;

			ptr = io_mapping_map_wc(&i915_vm_to_ggtt(vma->vm)->iomap,
						i915_vma_offset(vma),
						i915_vma_size(vma));
		} else {
			ptr = (void __iomem *)
				i915_gem_object_pin_map(vma->obj, I915_MAP_WC);
			if (IS_ERR(ptr)) {
				err = PTR_ERR(ptr);
				goto err;
			}
			ptr = page_pack_bits(ptr, 1);
		}

		if (ptr == NULL) {
			err = -ENOMEM;
			goto err;
		}

		if (unlikely(cmpxchg(&vma->iomap, NULL, ptr))) {
			if (page_unmask_bits(ptr))
				__i915_gem_object_release_map(vma->obj);
			else
				io_mapping_unmap(ptr);
			ptr = vma->iomap;
		}
	}

	__i915_vma_pin(vma);

	err = i915_vma_pin_fence(vma);
	if (err)
		goto err_unpin;

	if (page_unmask_bits(ptr))
		i915_vma_set_ggtt_write(vma);

	/* NB Access through the GTT requires the device to be awake. */
	return page_mask_bits(ptr);

err_unpin:
	__i915_vma_unpin(vma);
err:
	return IO_ERR_PTR(err);
}

void i915_vma_flush_writes(struct i915_vma *vma)
{
	if (i915_vma_unset_ggtt_write(vma))
		intel_gt_flush_ggtt_writes(vma->vm->gt);
}

void i915_vma_unpin_iomap(struct i915_vma *vma)
{
	GEM_BUG_ON(vma->iomap == NULL);

	/* XXX We keep the mapping until __i915_vma_unbind()/evict() */

	i915_vma_flush_writes(vma);

	i915_vma_unpin_fence(vma);
	i915_vma_unpin(vma);
}

void i915_vma_unpin_and_release(struct i915_vma **p_vma, unsigned int flags)
{
	struct i915_vma *vma;
	struct drm_i915_gem_object *obj;

	vma = fetch_and_zero(p_vma);
	if (!vma)
		return;

	obj = vma->obj;
	GEM_BUG_ON(!obj);

	i915_vma_unpin(vma);

	if (flags & I915_VMA_RELEASE_MAP)
		i915_gem_object_unpin_map(obj);

	i915_gem_object_put(obj);
}

/**
 * i915_vma_misplaced - check the current placement against requested
 * @vma - i915_vma to compare
 * @size - the size of the range required in the GTT for this vma access, or 0
 * @alignment - the required virtual address alignment or 0 for default
 * @flags - placement flags, see i915_vma_pin()
 *
 * i915_vma_misplaced() checks that if the vma is currently bound,
 * the current drm_mm_node (offset and size) match the caller's required
 * placement restraints, as would passed to i915_vma_pin().
 *
 * Normally, i915_vma_pin() would unbind any conflicting and /unpinned/ vma,
 * but sometimes we would prefer to e.g. use a secondary vma if this vma
 * itself would not satisfy the callers constraints, and thus would like
 * to check if the placement is valid for itself.
 *
 * Returns true if the vma is currently bound and its placement in the GTT
 * does not match the passed in constraints.
 */
bool i915_vma_misplaced(const struct i915_vma *vma,
			u64 size, u64 alignment, u64 flags)
{
	u64 act_size = __i915_vma_size(vma);
	u64 act_offset = __i915_vma_offset(vma);

	if (!drm_mm_node_allocated(&vma->node))
		return false;

	if (test_bit(I915_VMA_ERROR_BIT, __i915_vma_flags(vma)))
		return true;

	if (act_size < size)
		return true;

	GEM_BUG_ON(alignment && !is_power_of_2(alignment));
	if (alignment && !IS_ALIGNED(act_offset, alignment))
		return true;

	if (flags & PIN_MAPPABLE && !i915_vma_is_map_and_fenceable(vma))
		return true;

	if (flags & PIN_OFFSET_BIAS && act_offset < (flags & PIN_OFFSET_MASK))
		return true;

	if (flags & PIN_OFFSET_FIXED && act_offset != (flags & PIN_OFFSET_MASK))
		return true;

	if (flags & PIN_OFFSET_GUARD && vma->guard < (flags & PIN_OFFSET_MASK))
		return true;

	return false;
}

void __i915_vma_set_map_and_fenceable(struct i915_vma *vma)
{
	bool mappable, fenceable;

	GEM_BUG_ON(!i915_vma_is_ggtt(vma));
	GEM_BUG_ON(!vma->fence_size);

	fenceable = (i915_vma_size(vma) >= vma->fence_size &&
		     IS_ALIGNED(i915_vma_offset(vma), vma->fence_alignment));

	mappable = (i915_ggtt_offset(vma) + vma->fence_size <=
		    i915_vm_to_ggtt(vma->vm)->mappable_end);

	if (mappable && fenceable)
		set_bit(I915_VMA_CAN_FENCE_BIT, __i915_vma_flags(vma));
	else
		clear_bit(I915_VMA_CAN_FENCE_BIT, __i915_vma_flags(vma));
}

bool i915_gem_valid_gtt_space(struct i915_vma *vma, unsigned long color)
{
	struct drm_mm_node *node = &vma->node;
	struct drm_mm_node *other;

	/* Only valid to be called on an already inserted vma */
	GEM_BUG_ON(!drm_mm_node_allocated(node));
	GEM_BUG_ON(list_empty(&node->node_list));

	/*
	 * On some machines we have to be careful when putting differing types
	 * of snoopable memory together to avoid the prefetcher crossing memory
	 * domains and dying. During vm initialisation, we decide whether or not
	 * these constraints apply and set the drm_mm.color_adjust
	 * appropriately.
	 */
	if (i915_vm_has_cache_coloring(vma->vm)) {
		other = list_prev_entry(node, node_list);
		if (i915_node_color_differs(other, color) &&
		    !drm_mm_hole_follows(other))
			return false;

		other = list_next_entry(node, node_list);
		if (i915_node_color_differs(other, color) &&
		    !drm_mm_hole_follows(node))
			return false;
	/*
	 * On XEHPSDV we need to make sure we are not mixing LMEM and SMEM objects
	 * in the same page-table, i.e mixing 64K and 4K gtt pages in the same
	 * page-table.
	 */
	} else if (i915_vm_has_memory_coloring(vma->vm)) {
		other = list_prev_entry(node, node_list);
		if (i915_node_color_differs(other, color) &&
		    !drm_mm_hole_follows(other) &&
		    !IS_ALIGNED(other->start + other->size, SZ_2M))
			return false;

		other = list_next_entry(node, node_list);
		if (i915_node_color_differs(other, color) &&
		    !drm_mm_hole_follows(node) &&
		    !IS_ALIGNED(other->start, SZ_2M))
			return false;
	}

	return true;
}

/**
 * i915_vma_insert - finds a slot for the vma in its address space
 * @vma: the vma
 * @size: requested size in bytes (can be larger than the VMA)
 * @alignment: required alignment
 * @flags: mask of PIN_* flags to use
 *
 * First we try to allocate some free space that meets the requirements for
 * the VMA. Failiing that, if the flags permit, it will evict an old VMA,
 * preferrably the oldest idle entry to make room for the new VMA.
 *
 * Returns:
 * 0 on success, negative error code otherwise.
 */
static int
i915_vma_insert(struct i915_vma *vma, u64 size, u64 alignment, u64 flags)
{
	unsigned long color, guard;
	u64 start, end;
	int ret;

	GEM_BUG_ON(i915_vma_is_bound(vma, I915_VMA_GLOBAL_BIND | I915_VMA_LOCAL_BIND));
	GEM_BUG_ON(drm_mm_node_allocated(&vma->node));
	GEM_BUG_ON(hweight64(flags & (PIN_OFFSET_GUARD | PIN_OFFSET_FIXED | PIN_OFFSET_BIAS)) > 1);

	size = max(size, vma->size);
	alignment = max_t(typeof(alignment), alignment, vma->display_alignment);
	if (flags & PIN_MAPPABLE) {
		size = max_t(typeof(size), size, vma->fence_size);
		alignment = max_t(typeof(alignment),
				  alignment, vma->fence_alignment);
	}

	if (i915_is_ggtt(vma->vm) &&
	    intel_ggtt_needs_same_mem_type_within_cl_wa(vma->vm->i915)) {
		size = round_up(size, I915_GTT_PAGE_SIZE_64K);
		alignment = round_up(alignment, I915_GTT_PAGE_SIZE_64K);
	}

	GEM_BUG_ON(!IS_ALIGNED(size, I915_GTT_PAGE_SIZE));
	GEM_BUG_ON(!IS_ALIGNED(alignment, I915_GTT_MIN_ALIGNMENT));
	GEM_BUG_ON(!is_power_of_2(alignment));

	guard = vma->guard; /* retain guard across rebinds */
	if (flags & PIN_OFFSET_GUARD) {
		GEM_BUG_ON(overflows_type(flags & PIN_OFFSET_MASK, u32));
		guard = max_t(u32, guard, flags & PIN_OFFSET_MASK);
	}
	guard = ALIGN(guard, alignment);

	start = flags & PIN_OFFSET_BIAS ? flags & PIN_OFFSET_MASK : 0;
	GEM_BUG_ON(!IS_ALIGNED(start, I915_GTT_PAGE_SIZE));

	end = vma->vm->total;
	if (flags & PIN_MAPPABLE)
		end = min_t(u64, end, i915_vm_to_ggtt(vma->vm)->mappable_end);
	if (flags & PIN_ZONE_32)
		end = min_t(u64, end, BIT_ULL(32) - I915_GTT_PAGE_SIZE);
	if (flags & PIN_ZONE_48)
		end = min_t(u64, end, BIT_ULL(48) - I915_GTT_PAGE_SIZE);
	GEM_BUG_ON(!IS_ALIGNED(end, I915_GTT_PAGE_SIZE));
	GEM_BUG_ON(2 * guard > end);

	/* If binding the object/GGTT view requires more space than the entire
	 * aperture has, reject it early before evicting everything in a vain
	 * attempt to find space.
	 */
	if (size > end - 2 * guard) {
		DRM_DEBUG("Attempting to bind an object larger than the aperture: request=%llu > %s aperture=%llu\n",
			  size, flags & PIN_MAPPABLE ? "mappable" : "total",
			  end);
		return -ENOSPC;
	}

	color = 0;
	if (vma->obj) {
		if (HAS_64K_PAGES(vma->vm->i915) && i915_gem_object_is_lmem(vma->obj))
			alignment = max(alignment, I915_GTT_PAGE_SIZE_64K);

		if (i915_vm_has_cache_coloring(vma->vm))
			color = vma->obj->pat_index;
		else if (i915_vm_has_memory_coloring(vma->vm))
			color = i915_gem_object_is_lmem(vma->obj);
	}

	if (flags & PIN_OFFSET_FIXED) {
		u64 offset = flags & PIN_OFFSET_MASK;

		if (!IS_ALIGNED(offset, alignment) ||
		    range_overflows(offset, size, end))
			return -EINVAL;

		/*
		 * The caller knows not of the guard added by others and
		 * requests for the offset of the start of its buffer
		 * to be fixed, which may not be the same as the position
		 * of the vma->node due to the guard pages.
		 */
		if (offset < guard || offset + size > end - guard)
			return -ENOSPC;

		ret = i915_gem_gtt_reserve(vma->vm, &vma->node,
					   size + 2 * guard,
					   offset - guard,
					   color, flags);
		if (ret)
			return ret;
	} else {
		size += 2 * guard;

		/*
		 * For the non-softpin path, the kernel is allowed to
		 * fiddle with the alignment and padding if it means
		 * we have a better chance of utilising huge-GTT-pages
		 * when we later bind this vma in the ppGTT.

		 * We only support huge gtt pages through the 48b PPGTT,
		 * however we also don't want to force any alignment for
		 * objects which need to be tightly packed into the low 32bits.
		 *
		 * Note that we assume that GGTT are limited to 4GiB for the
		 * forseeable future. See also i915_ggtt_offset().
		 */
		if (upper_32_bits(end - 1) && vma->size > I915_GTT_PAGE_SIZE) {
			u64 page_alignment;

			/*
			 * If we lack PS64 support then we can't mix 64K
			 * and 4K PTEs in the same page-table (2M block),
			 * but on platforms which need memory coloring,
			 * we use 2M coloring to separate 4K and 64K pages
			 * into * different 2M blocks. In all other cases,
			 * to avoid the ugliness and complexity of coloring
			 * we opt for just aligning 64K objects to 2M.
			 *
			 * In the case of PS64, we can enable 64K pages
			 * at the pte level, and so we can minimally align
			 * to 64K if we think that will also give us 64K
			 * GTT pages.
			 */
			if (HAS_64K_PAGES(vma->vm->i915) &&
			    vma->size < I915_GTT_PAGE_SIZE_2M)
				page_alignment = I915_GTT_PAGE_SIZE_64K;
			else
				page_alignment = rounddown_pow_of_two(vma->size);

			/*
			 * Check we don't expand for the limited Global GTT
			 * (mappable aperture is even more precious!). This
			 * also checks that we exclude the aliasing-ppgtt.
			 */
			GEM_BUG_ON(i915_vma_is_ggtt(vma));

			alignment = max(alignment, page_alignment);

			/*
			 * On platforms which need memory coloring we already
			 * ensure that we don't mix 64K and 4K GTT pages in
			 * the same 2M block, and on such platforms we support
			 * some form of PS64(even if it's only for system
			 * memory), so opportunistically adding 2M padding to
			 * ensure 64K GTT pages doesn't help us.
			 *
			 * On platforms which support PS64 for both local and
			 * system memory, the whole idea of adding 2M padding
			 * is completely irrelevant.
			 */
			if (!HAS_64K_PAGES(vma->vm->i915))
				size = round_up(size, I915_GTT_PAGE_SIZE_2M);
		}

		/*
		 * We observe GPU hangs if we place a batch (from userspace) at
		 * the very top of the GTT, as the CS parser may prefetch past
		 * the end of the GTT. In order to avoid this, we restrict
		 * ourselves from assigning the last page of the GTT to
		 * userspace. (They are free to assign the address to
		 * themselves with softpin.)
		 *
		 * However, GGTT and ppGTT are not the only vm we handle. DPT
		 * is used as an indirection page table for framebuffers,
		 * and is only as large as the framebuffer itself. We cannot
		 * reduce the effective DPT size as there are no spare pages.
		 * To only restrict userspace buffers and not affect DPT
		 * assignments, we only apply the restriction to PIN_USER.
		 */
		if (flags & PIN_USER)
			end = min(end, vma->vm->total - I915_GTT_PAGE_SIZE);

		ret = i915_gem_gtt_insert(vma->vm, &vma->node,
					  size, alignment, color,
					  start, end, flags);
		if (ret)
			return ret;

		GEM_BUG_ON(vma->node.start < start);
		GEM_BUG_ON(vma->node.start + vma->node.size > end);
	}
	GEM_BUG_ON(!drm_mm_node_allocated(&vma->node));
	GEM_BUG_ON(!i915_gem_valid_gtt_space(vma, color));

	list_add_tail(&vma->vm_link, &vma->vm->bound_list);
	vma->guard = guard;

	i915_debugger_vma_insert(vma->vm->client, vma);
	return 0;
}

static void
i915_vma_detach(struct i915_vma *vma)
{
	GEM_BUG_ON(!drm_mm_node_allocated(&vma->node));
	GEM_BUG_ON(i915_vma_is_bound(vma, I915_VMA_GLOBAL_BIND | I915_VMA_LOCAL_BIND));

	/*
	 * And finally now the object is completely decoupled from this
	 * vma, we can drop its hold on the backing storage and allow
	 * it to be reaped by the shrinker.
	 */
	list_del(&vma->vm_link);
}

static bool try_qad_pin(struct i915_vma *vma, unsigned int flags)
{
	unsigned int bound;
	bool pinned = true;

	bound = atomic_read(&vma->flags);
	do {
		if (unlikely(flags & ~bound))
			return false;

		if (unlikely(bound & (I915_VMA_OVERFLOW | I915_VMA_ERROR)))
			return false;

		if (!(bound & I915_VMA_PIN_MASK))
			goto unpinned;

		GEM_BUG_ON(((bound + 1) & I915_VMA_PIN_MASK) == 0);
	} while (!atomic_try_cmpxchg(&vma->flags, &bound, bound + 1));

	return true;

unpinned:
	/*
	 * If pin_count==0, but we are bound, check under the lock to avoid
	 * racing with a concurrent i915_vma_unbind().
	 */
	mutex_lock(&vma->vm->mutex);
	do {
		if (unlikely(bound & (I915_VMA_OVERFLOW | I915_VMA_ERROR))) {
			pinned = false;
			break;
		}

		if (unlikely(flags & ~bound)) {
			pinned = false;
			break;
		}
	} while (!atomic_try_cmpxchg(&vma->flags, &bound, bound + 1));
	mutex_unlock(&vma->vm->mutex);

	return pinned;
}

static int vma_get_pages(struct i915_vma *vma)
{
	int err = 0;
	bool pinned_pages = false;

	if (atomic_add_unless(&vma->pages_count, 1, 0))
		return 0;

	if (vma->obj) {
		err = i915_gem_object_pin_pages(vma->obj);
		if (err)
			return err;
		/*
		 * This is set such that we will unpin on error
		 * or for the faultable VM case (see below).
		 */
		pinned_pages = true;
	}

	/* Allocations ahoy! */
	if (mutex_lock_interruptible(&vma->pages_mutex)) {
		err = -EINTR;
		goto unpin;
	}

	if (!atomic_read(&vma->pages_count)) {
		err = vma->ops->set_pages(vma);
		if (err)
			goto unlock;
		/*
		 * For faultable VM, always unpin on return as we
		 * don't need object pinned beyond above set_pages().
		 * Else this clears pinned_pages, so pin count will
		 * persist until vma_put_pages()/vma_unbind_pages.
		 */
		if (!i915_vm_page_fault_enabled(vma->vm))
			pinned_pages = false;
	}
	atomic_inc(&vma->pages_count);

unlock:
	mutex_unlock(&vma->pages_mutex);
unpin:
	if (pinned_pages)
		__i915_gem_object_unpin_pages(vma->obj);

	return err;
}

static void __vma_put_pages(struct i915_vma *vma, unsigned int count)
{
	/* We allocate under vma_get_pages, so beware the shrinker */
	mutex_lock_nested(&vma->pages_mutex, SINGLE_DEPTH_NESTING);
	GEM_BUG_ON(atomic_read(&vma->pages_count) < count);
	if (atomic_sub_return(count, &vma->pages_count) == 0) {
		vma->ops->clear_pages(vma);
		GEM_BUG_ON(vma->pages);
		if (vma->obj && !i915_vm_page_fault_enabled(vma->vm))
			i915_gem_object_unpin_pages(vma->obj);
	}
	mutex_unlock(&vma->pages_mutex);
}

static void vma_put_pages(struct i915_vma *vma)
{
	if (atomic_add_unless(&vma->pages_count, -1, 1))
		return;

	__vma_put_pages(vma, 1);
}

static void vma_unbind_pages(struct i915_vma *vma)
{
	unsigned int count;

	lockdep_assert_held(&vma->vm->mutex);

	/* The upper portion of pages_count is the number of bindings */
	count = atomic_read(&vma->pages_count);
	count >>= I915_VMA_PAGES_BIAS;
	if (!i915_vm_page_fault_enabled(vma->vm))
		GEM_BUG_ON(!count);

	if (count)
		__vma_put_pages(vma, count | count << I915_VMA_PAGES_BIAS);
}

/**
 * i915_vma_bind - Updates page tables for this vma.
 * @vma: vma to bind
 * @ww: ww context for object lock
 */
int i915_vma_bind(struct i915_vma *vma, struct i915_gem_ww_ctx *ww)
{
	struct i915_address_space *vm = vma->vm;
	struct i915_vma_work *work;
	int err;

	assert_object_held(vma->obj);
	err = vma_get_pages(vma);
	if (err)
		return err;
	GEM_BUG_ON(!vma->pages);

	work = i915_vma_work(vm);
	if (!work) {
		err = -ENOMEM;
		goto err_pages;
	}

	err = i915_vma_work_set_vm(work, vma, ww, PIN_RESIDENT);
	if (err)
		goto err_fence;

	err = mutex_lock_interruptible(&vm->mutex);
	if (err)
		goto err_fence;

	err = i915_active_acquire(&vma->active);
	if (err)
		goto err_unlock;

	err = __i915_vma_bind(vma,
			      vma->obj->pat_index,
			      PIN_USER | PIN_RESIDENT,
			      work);
	if (err)
		goto err_active;

	atomic_add(I915_VMA_PAGES_ACTIVE, &vma->pages_count);
	GEM_BUG_ON(!i915_vma_is_bound(vma, PIN_RESIDENT));

err_active:
	i915_active_release(&vma->active);
err_unlock:
	mutex_unlock(&vm->mutex);
err_fence:
	dma_fence_work_commit(&work->base);
err_pages:
	vma_put_pages(vma);
	return err;
}

int i915_vma_pin_ww(struct i915_vma *vma, struct i915_gem_ww_ctx *ww,
		    u64 size, u64 alignment, u64 flags)
{
	struct i915_vma_work *work;
	intel_wakeref_t wakeref = 0;
	bool make_resident = false;
	unsigned int bound;
	int err;

#ifdef CONFIG_PROVE_LOCKING
	if (debug_locks && !WARN_ON(!ww) && vma->resv)
		assert_vma_held(vma);
#endif

	BUILD_BUG_ON(PIN_GLOBAL != I915_VMA_GLOBAL_BIND);
	BUILD_BUG_ON(PIN_USER != I915_VMA_LOCAL_BIND);

	GEM_BUG_ON(!(flags & (PIN_USER | PIN_GLOBAL)));

	/* First try and grab the pin without rebinding the vma */
	if (try_qad_pin(vma, flags & I915_VMA_BIND_MASK))
		return 0;

	/*
	 * Restrict faults to persistent vmas unless faults are enabled using
	 * modparam enable_pagefault.
	 * XXX: Remove this when we formalize the faulting support on legacy
	 * path
	 */
	if (!i915_is_ggtt(vma->vm)) {
		if (!i915_vm_page_fault_enabled(vma->vm) ||
		    (!vma->vm->i915->params.enable_pagefault &&
		     !i915_vma_is_persistent(vma)))
			flags |= PIN_RESIDENT;
	}

	/*
	 * We conflate PIN_RESIDENT == PIN_GLOBAL so we must always
	 * bind the backing store when using GGTT. This is reasonable as we
	 * have no pagefaulting support for the GGTT, and so we know that
	 * all GGTT vma need to be resident (including all legacy GGTT HW).
	 */
	if (i915_is_ggtt(vma->vm) || flags & PIN_RESIDENT) {
		err = vma_get_pages(vma);
		if (err)
			return err;
		GEM_BUG_ON(!vma->pages);
		make_resident = true;
	}

	if (flags & PIN_GLOBAL)
		wakeref = intel_runtime_pm_get(&vma->vm->i915->runtime_pm);

	intel_flat_ppgtt_allocate_requests(vma, false);

	work = i915_vma_work(vma->vm);
	if (!work) {
		err = -ENOMEM;
		goto err_rpm;
	}

	err = i915_vma_work_set_vm(work, vma, ww, flags);
	if (err)
		goto err_fence;

	/*
	 * Differentiate between user/kernel vma inside the aliasing-ppgtt.
	 *
	 * We conflate the Global GTT with the user's vma when using the
	 * aliasing-ppgtt, but it is still vitally important to try and
	 * keep the use cases distinct. For example, userptr objects are
	 * not allowed inside the Global GTT as that will cause lock
	 * inversions when we have to evict them the mmu_notifier callbacks -
	 * but they are allowed to be part of the user ppGTT which can never
	 * be mapped. As such we try to give the distinct users of the same
	 * mutex, distinct lockclasses [equivalent to how we keep i915_ggtt
	 * and i915_ppgtt separate].
	 *
	 * NB this may cause us to mask real lock inversions -- while the
	 * code is safe today, lockdep may not be able to spot future
	 * transgressions.
	 */
	err = mutex_lock_interruptible_nested(&vma->vm->mutex,
					      !(flags & PIN_GLOBAL));
	if (err)
		goto err_fence;

	/* No more allocations allowed now we hold vm->mutex */

	if (unlikely(i915_vma_is_closed(vma))) {
		err = -ENOENT;
		goto err_unlock;
	}

	if (unlikely(i915_vma_misplaced(vma, size, alignment, flags))) {
		err = -EBUSY;
		if (!(flags & (PIN_NONBLOCK | PIN_NOEVICT)))
			err = __i915_vma_unbind(vma);
		if (err)
			goto err_unlock;
	}

	bound = atomic_read(&vma->flags);
	if (unlikely(bound & I915_VMA_ERROR)) {
		err = i915_active_fence_has_error(&vma->active.excl) ?: -EFAULT;
		goto err_unlock;
	}

	if (unlikely(!((bound + 1) & I915_VMA_PIN_MASK))) {
		err = -EAGAIN; /* pins are meant to be fairly temporary */
		goto err_unlock;
	}

	if (unlikely(!(flags & ~bound & I915_VMA_BIND_MASK))) {
		__i915_vma_pin(vma);
		goto err_unlock;
	}

	err = i915_active_acquire(&vma->active);
	if (err)
		goto err_unlock;

	if (!(bound & I915_VMA_BIND_MASK)) {
		/* For faultable VM, vma->node may already be allocated */
		if (!drm_mm_node_allocated(&vma->node)) {
			err = i915_vma_insert(vma, size, alignment, flags);
			if (err)
				goto err_active;
		}

		if (i915_is_ggtt(vma->vm))
			__i915_vma_set_map_and_fenceable(vma);
	}

	err = __i915_vma_bind(vma,
			      vma->obj ?
			      vma->obj->pat_index :
			      i915_gem_get_pat_index(vma->vm->i915,
						     I915_CACHE_NONE),
			      flags,
			      work);
	if (err)
		goto err_remove;

	/* There should only be at most 2 active bindings (user, global) */
	if (make_resident) {
		GEM_BUG_ON(bound + I915_VMA_PAGES_ACTIVE < bound);
		atomic_add(I915_VMA_PAGES_ACTIVE, &vma->pages_count);
	}

	list_move_tail(&vma->vm_link, &vma->vm->bound_list);

	__i915_vma_pin(vma);
	GEM_BUG_ON(!i915_vma_is_pinned(vma));
	GEM_BUG_ON(!i915_vma_is_bound(vma, flags));
	GEM_BUG_ON(i915_vma_misplaced(vma, size, alignment, flags));

err_remove:
	if (!i915_vma_is_bound(vma, I915_VMA_BIND_MASK)) {
		i915_vma_detach(vma);
		drm_mm_remove_node(&vma->node);
	}
err_active:
	i915_active_release(&vma->active);
err_unlock:
	mutex_unlock(&vma->vm->mutex);
err_fence:
	dma_fence_work_commit_imm_if(&work->base, i915_vma_is_ggtt(vma));
err_rpm:
	if (wakeref)
		intel_runtime_pm_put(&vma->vm->i915->runtime_pm, wakeref);
	if (make_resident)
		vma_put_pages(vma);

	return err;
}

static void flush_idle_contexts(struct i915_ggtt *ggtt)
{
	struct intel_engine_cs *engine;
	enum intel_engine_id id;
	struct intel_gt *gt;

	list_for_each_entry_rcu(gt, &ggtt->gt_list, ggtt_link) {
		for_each_engine(engine, gt, id)
			intel_engine_flush_barriers(engine);

		intel_gt_retire_requests(gt);
	}
}

int i915_ggtt_pin(struct i915_vma *vma, struct i915_gem_ww_ctx *ww,
		  u32 align, unsigned int flags)
{
	struct i915_address_space *vm = vma->vm;
	int err;

	GEM_BUG_ON(!i915_vma_is_ggtt(vma));

#ifdef CONFIG_LOCKDEP
	WARN_ON(!ww && vma->resv && dma_resv_held(vma->resv));
#endif

	do {
		if (ww)
			err = i915_vma_pin_ww(vma, ww, 0, align, flags | PIN_GLOBAL);
		else
			err = i915_vma_pin(vma, 0, align, flags | PIN_GLOBAL);
		if (err != -ENOSPC) {
			if (!err) {
				err = i915_vma_wait_for_bind(vma);
				if (err)
					i915_vma_unpin(vma);
			}
			return err;
		}

		/* Unlike i915_vma_pin, we don't take no for an answer! */
		flush_idle_contexts(i915_vm_to_ggtt(vm));
		if (mutex_lock_interruptible(&vm->mutex) == 0) {
			i915_gem_evict_vm(vm);
			mutex_unlock(&vm->mutex);
		}
	} while (1);
}

int i915_ggtt_pin_for_gt(struct i915_vma *vma, struct i915_gem_ww_ctx *ww,
			 u32 align, unsigned int flags)
{
	int retry = 3;
	int err;

	GEM_BUG_ON(flags & (PIN_OFFSET_FIXED | PIN_OFFSET_GUARD));
	GEM_BUG_ON(flags & PAGE_MASK);

	/*
	 * During execution we have to avoid certain address ranges,
	 * as these are aliased into the GuC, and historically we
	 * have seen problems with ring bufferss placed at offset 0.
	 */
	flags |= i915_ggtt_pin_bias(vma) | PIN_OFFSET_BIAS | PIN_GLOBAL;

	do {
		if (ww)
			err = i915_vma_pin_ww(vma, ww, 0, align, flags);
		else
			err = i915_vma_pin(vma, 0, align, flags);
		if (!err || err == -EDEADLK)
			return err;

		flush_idle_contexts(i915_vm_to_ggtt(vma->vm));
	} while (--retry);

	return err;
}

void i915_vma_close(struct i915_vma *vma)
{
	struct i915_address_space *vm = vma->vm;
	struct i915_vma_clock *clock = &vm->gt->vma_clock;
	unsigned long flags;

	/*
	 * We defer actually closing, unbinding and destroying the VMA until
	 * the next idle point, or if the object is freed in the meantime. By
	 * postponing the unbind, we allow for it to be resurrected by the
	 * client, avoiding the work required to rebind the VMA. This is
	 * advantageous for DRI, where the client/server pass objects
	 * between themselves, temporarily opening a local VMA to the
	 * object, and then closing it again. The same object is then reused
	 * on the next frame (or two, depending on the depth of the swap queue)
	 * causing us to rebind the VMA once more. This ends up being a lot
	 * of wasted work for the steady state.
	 */
	GEM_BUG_ON(!atomic_read(&vma->open_count));
	if (atomic_dec_and_lock_irqsave(&vma->open_count,
					&clock->lock,
					flags)) {
		struct drm_i915_gem_object *obj = vma->obj;
		bool inuse =
			!i915_vma_is_persistent(vma) &&
			i915_gem_object_inuse(obj) &&
			!i915_is_ggtt_or_dpt(vm);
		bool first = false;

		if (inuse) {
			first = list_empty(&clock->age[0]);
			list_add(&vma->closed_link, &clock->age[0]);
			__i915_vma_get(vma);
		}
		spin_unlock_irqrestore(&clock->lock, flags);
		__i915_vma_put(vma);

		i915_vm_close(vm);
		i915_gem_object_put(obj);

		if (first)
			queue_delayed_work(vm->i915->wq, &clock->work,
					   round_jiffies_up_relative(HZ));
	}
}

void i915_vma_unpublish(struct i915_vma *vma)
{
	lockdep_assert_held(&vma->vm->mutex);

	/* Drop the original ref held for a published vma, just once */
	if (!i915_vma_set_purged(vma))
		return;

	if (drm_mm_node_allocated(&vma->node)) {
		GEM_BUG_ON(i915_vma_is_active(vma));

		atomic_and(~I915_VMA_PIN_MASK, &vma->flags);
		__i915_vma_evict(vma);

		drm_mm_remove_node(&vma->node);
	}

	if (vma->obj) {
		struct drm_i915_gem_object *obj = vma->obj;

		spin_lock(&obj->vma.lock);
		list_del(&vma->obj_link);
		if (!RB_EMPTY_NODE(&vma->obj_node))
			rb_erase(&vma->obj_node, &obj->vma.tree);
		spin_unlock(&obj->vma.lock);
	}

	__i915_vma_put(vma);
}

void i915_vma_release(struct kref *ref)
{
	struct i915_vma *vma = container_of(ref, typeof(*vma), ref);

	GEM_BUG_ON(i915_vma_is_active(vma));
	GEM_BUG_ON(drm_mm_node_allocated(&vma->node));
	GEM_BUG_ON(i915_active_fence_isset(&vma->active.excl));
	GEM_BUG_ON(!list_empty(&vma->vm_bind_link));

	if (unlikely(vma->bind_fence)) {
		i915_sw_fence_set_error_once(vma->bind_fence, -EINVAL);
		i915_sw_fence_complete(vma->bind_fence);
	}

	intel_flat_ppgtt_request_pool_clean(vma);
	i915_vm_put(vma->vm);

	i915_active_fini(&vma->active);
	i915_vma_metadata_free(vma);
	i915_vma_free(vma);
}

struct i915_vma *i915_vma_open(struct i915_vma *vma)
{
	if (!atomic_add_unless(&vma->open_count, 1, 0)) {
		struct i915_vma_clock *clock = &vma->vm->gt->vma_clock;
		unsigned long flags;

		spin_lock_irqsave(&clock->lock, flags);
		if (!atomic_add_unless(&vma->open_count, 1, 0)) {
			if (!i915_vma_tryget(vma)) {
				vma = NULL;
			} else if (!i915_vm_tryopen(vma->vm)) {
				i915_vma_put(vma);
				vma = NULL;
			} else {
				__i915_vma_get(vma);
				if (!list_empty(&vma->closed_link)) {
					list_del_init(&vma->closed_link);
					__i915_vma_put(vma);
				}
				atomic_set_release(&vma->open_count, 1);
			}
		}
		spin_unlock_irqrestore(&clock->lock, flags);
	}

	return vma;
}

static void i915_vma_clock(struct work_struct *w)
{
	struct i915_vma_clock *clock =
		container_of(w, typeof(*clock), work.work);
	struct intel_gt *gt = container_of(clock, typeof(*gt), vma_clock);
	struct i915_vma *vma;

	/*
	 * A very simple clock aging algorithm: we keep the user's closed
	 * vma alive for a couple of timer ticks before destroying them.
	 * This serves a shortlived cache so that frequently reused VMA
	 * are kept alive between frames and we skip having to rebing them.
	 *
	 * When closed, we insert the vma into age[0]. Upon completion of
	 * a timer tick, it is moved to age[1]. At the start of each timer
	 * tick, we destroy all the old vma that were accumulated into age[1]
	 * and have not been reused. All destroyed vma have therefore been
	 * unused for more than 1 tick (at least a second), and at most 2
	 * ticks (we expect the average to be 1.5 ticks).
	 *
	 * https://en.wikipedia.org/wiki/Page_replacement_algorithm#Clock
	 */

	down_write(&clock->sem);
	spin_lock_irq(&clock->lock);
	while ((vma = list_first_entry_or_null(&clock->age[1],
					       typeof(*vma),
					       closed_link))) {
		struct i915_address_space *vm = vma->vm;

		list_del_init(&vma->closed_link);
		spin_unlock_irq(&clock->lock);

		mutex_lock(&vm->mutex);
		i915_vma_unpublish(vma);
		mutex_unlock(&vm->mutex);

		__i915_vma_put(vma);
		spin_lock_irq(&clock->lock);
	}
	list_replace_init(&clock->age[0], &clock->age[1]);
	spin_unlock_irq(&clock->lock);
	up_write(&clock->sem);

	if (!list_empty(&clock->age[1]))
		queue_delayed_work(gt->i915->wq, &clock->work,
				   round_jiffies_up_relative(HZ));
}

void i915_vma_clock_init_early(struct i915_vma_clock *clock)
{
	spin_lock_init(&clock->lock);
	INIT_LIST_HEAD(&clock->age[0]);
	INIT_LIST_HEAD(&clock->age[1]);
	init_rwsem(&clock->sem);

	INIT_DELAYED_WORK(&clock->work, i915_vma_clock);
}

void i915_vma_clock_flush(struct i915_vma_clock *clock)
{
       do {
	       if (cancel_delayed_work_sync(&clock->work))
		       i915_vma_clock(&clock->work.work);
       } while (delayed_work_pending(&clock->work));
}

void i915_vma_clock_fini(struct i915_vma_clock *clock)
{
	i915_vma_clock_flush(clock);
	GEM_BUG_ON(!list_empty(&clock->age[1]));
	GEM_BUG_ON(!list_empty(&clock->age[0]));
}

static void __i915_vma_iounmap(struct i915_vma *vma)
{
	GEM_BUG_ON(i915_vma_is_pinned(vma));

	if (vma->iomap == NULL)
		return;

	if (page_unmask_bits(vma->iomap))
		__i915_gem_object_release_map(vma->obj);
	else
		io_mapping_unmap(vma->iomap);
	vma->iomap = NULL;
}

void i915_vma_revoke_mmap(struct i915_vma *vma)
{
	struct drm_vma_offset_node *node;
	u64 vma_offset;

	if (!i915_vma_has_userfault(vma))
		return;

	GEM_BUG_ON(!i915_vma_is_map_and_fenceable(vma));
	GEM_BUG_ON(!vma->obj->userfault_count);

	node = &vma->mmo->vma_node;
	vma_offset = vma->ggtt_view.partial.offset << PAGE_SHIFT;
	unmap_mapping_range(vma->vm->i915->drm.anon_inode->i_mapping,
			    drm_vma_node_offset_addr(node) + vma_offset,
			    vma->size,
			    1);

	i915_vma_unset_userfault(vma);
	if (!--vma->obj->userfault_count)
		list_del(&vma->obj->userfault_link);
}

int __i915_vma_move_to_active(struct i915_vma *vma, struct i915_request *rq)
{
	int err;

	GEM_BUG_ON(!i915_vma_is_pinned(vma));

	/* Wait for the vma to be bound before we start! */
	err = __i915_request_await_bind(rq, vma);
	if (err)
		return err;

	return i915_active_add_request(&vma->active, rq);
}

int _i915_vma_move_to_active(struct i915_vma *vma,
			     struct i915_request *rq,
			     struct dma_fence *fence,
			     unsigned int flags)
{
	struct drm_i915_gem_object *obj = vma->obj;
	int err;

	assert_object_held(obj);

	if (!i915_vma_is_persistent(vma)) {
		err = __i915_vma_move_to_active(vma, rq);
		if (unlikely(err))
			return err;

		GEM_BUG_ON(!i915_vma_is_active(vma));
	}

	if (flags & EXEC_OBJECT_WRITE) {
		struct intel_frontbuffer *front;

		front = __intel_frontbuffer_get(obj);
		if (unlikely(front)) {
			if (intel_frontbuffer_invalidate(front, ORIGIN_CS))
				i915_active_add_request(&front->write, rq);
			intel_frontbuffer_put(front);
		}

		if (fence) {
			dma_resv_add_excl_fence(vma->resv, fence);
			obj->write_domain = I915_GEM_DOMAIN_RENDER;
			obj->read_domains = 0;
		}
	} else {
		if (!(flags & __EXEC_OBJECT_NO_RESERVE)) {
			err = dma_resv_reserve_shared(vma->resv, 1);
			if (unlikely(err))
				return err;
		}

		if (fence) {
			dma_resv_add_shared_fence(vma->resv, fence);
			obj->write_domain = 0;
		}
	}

	if (flags & EXEC_OBJECT_NEEDS_FENCE && vma->fence)
		i915_active_add_request(&vma->fence->active, rq);

	obj->read_domains |= I915_GEM_GPU_DOMAINS;

	return 0;
}

void __i915_vma_evict(struct i915_vma *vma)
{
	struct i915_address_space *vm = vma->vm;

	GEM_BUG_ON(i915_vma_is_pinned(vma));

	i915_debugger_vma_evict(vm->client, vma);

	if (i915_vma_is_map_and_fenceable(vma)) {
		/* Force a pagefault for domain tracking on next user access */
		i915_vma_revoke_mmap(vma);

		/*
		 * Check that we have flushed all writes through the GGTT
		 * before the unbind, other due to non-strict nature of those
		 * indirect writes they may end up referencing the GGTT PTE
		 * after the unbind.
		 *
		 * Note that we may be concurrently poking at the GGTT_WRITE
		 * bit from set-domain, as we mark all GGTT vma associated
		 * with an object. We know this is for another vma, as we
		 * are currently unbinding this one -- so if this vma will be
		 * reused, it will be refaulted and have its dirty bit set
		 * before the next write.
		 */
		i915_vma_flush_writes(vma);

		/* release the fence reg _after_ flushing */
		i915_vma_revoke_fence(vma);

		clear_bit(I915_VMA_CAN_FENCE_BIT, __i915_vma_flags(vma));
	}

	__i915_vma_iounmap(vma);

	GEM_BUG_ON(vma->fence);
	GEM_BUG_ON(i915_vma_has_userfault(vma));

	if (likely(atomic_read(&vm->open))) {
		trace_i915_vma_unbind(vma);
		vma->ops->unbind_vma(vm, vma);

		if (!list_empty(&vma->vm_bind_link)) {
			GEM_BUG_ON(!i915_vma_is_persistent(vma));
			assert_object_held(vm->root_obj);
			list_move_tail(&vma->vm_bind_link, &vm->vm_bind_list);
		}
	}
	atomic_and(~(I915_VMA_BIND_MASK | I915_VMA_ERROR | I915_VMA_GGTT_WRITE),
		   &vma->flags);

	if (!i915_vm_page_fault_enabled(vm) ||
	    i915_vma_is_purged(vma) ||
	    !i915_vma_is_persistent(vma))
		i915_vma_detach(vma);

	vma_unbind_pages(vma);
}

int __i915_vma_unbind(struct i915_vma *vma)
{
	struct i915_address_space *vm = vma->vm;
	int ret;

	lockdep_assert_held(&vm->mutex);

	if (!drm_mm_node_allocated(&vma->node))
		return 0;

	if (i915_vma_is_pinned(vma)) {
		vma_print_allocator(vma, "is pinned");
		return -EAGAIN;
	}

	/*
	 * After confirming that no one else is pinning this vma, wait for
	 * any laggards who may have crept in during the wait (through
	 * a residual pin skipping the vm->mutex) to complete.
	 */
	ret = i915_vma_sync(vma);
	if (ret)
		return ret;

	__i915_vma_evict(vma);

	if (!i915_vm_page_fault_enabled(vm) ||
	    i915_vma_is_purged(vma) ||
	    !i915_vma_is_persistent(vma))
		drm_mm_remove_node(&vma->node);

	return 0;
}

int i915_vma_unbind(struct i915_vma *vma)
{
	struct i915_address_space *vm = vma->vm;
	intel_wakeref_t wakeref = 0;
	int err;

	/* Optimistic wait before taking the mutex */
	err = i915_vma_sync(vma);
	if (err)
		return err;

	if (!drm_mm_node_allocated(&vma->node))
		return 0;

	if (i915_vma_is_pinned(vma)) {
		vma_print_allocator(vma, "is pinned");
		return -EAGAIN;
	}

	if (i915_vma_is_bound(vma, I915_VMA_GLOBAL_BIND))
		/* XXX not always required: nop_clear_range */
		wakeref = intel_runtime_pm_get(&vm->i915->runtime_pm);

	intel_flat_ppgtt_allocate_requests(vma, true);
	err = mutex_lock_interruptible_nested(&vma->vm->mutex, !wakeref);
	if (err)
		goto out_rpm;

	err = __i915_vma_unbind(vma);
	mutex_unlock(&vm->mutex);

out_rpm:
	if (wakeref)
		intel_runtime_pm_put(&vm->i915->runtime_pm, wakeref);

	intel_flat_ppgtt_request_pool_clean(vma);
	return err;
}

/**
 * i915_vma_prefetch - Prefetch a vma to desired memory region
 * @vma: vma to prefetch
 * @mem: the destination memory region to prefetch to
 *
 * Prefetch vma's backing store to desired memory region, and
 * bind vma to gpu synchronously
 */
int i915_vma_prefetch(struct i915_vma *vma, struct intel_memory_region *mem)
{
	struct i915_gem_ww_ctx ww;
	int err;

	if (!i915_gem_object_can_migrate(vma->obj, mem->id))
		return -EINVAL;

	if (i915_gem_object_is_userptr(vma->obj))
		return -EINVAL;

	for_i915_gem_ww(&ww, err, true) {
		err = i915_gem_object_lock(vma->vm->root_obj, &ww);
		if (err)
			continue;

		err = i915_gem_object_lock(vma->obj, &ww);
		if (err)
			continue;

		if (vma->obj->mm.region.mem->id != mem->id)
			err = i915_gem_object_migrate_region(vma->obj, &ww, &mem, 1);
		if (err)
			continue;

		if (!i915_vma_is_bound(vma, PIN_RESIDENT))
			err = i915_vma_bind(vma, &ww);
	}

	return err;
}

struct i915_vma *i915_vma_make_unshrinkable(struct i915_vma *vma)
{
	i915_gem_object_make_unshrinkable(vma->obj);
	return vma;
}

void i915_vma_make_shrinkable(struct i915_vma *vma)
{
	i915_gem_object_make_shrinkable(vma->obj);
}

void i915_vma_make_purgeable(struct i915_vma *vma)
{
	i915_gem_object_make_purgeable(vma->obj);
}

#if IS_ENABLED(CONFIG_DRM_I915_SELFTEST)
#include "selftests/i915_vma.c"
#endif

void i915_vma_module_exit(void)
{
	kmem_cache_destroy(slab_vmas);
}

int __init i915_vma_module_init(void)
{
	slab_vmas = KMEM_CACHE(i915_vma, SLAB_HWCACHE_ALIGN);
	if (!slab_vmas)
		return -ENOMEM;

	return 0;
}
