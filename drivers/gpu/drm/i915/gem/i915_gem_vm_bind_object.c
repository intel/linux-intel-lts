// SPDX-License-Identifier: MIT
/*
 * Copyright © 2019 Intel Corporation
 */

#include <linux/interval_tree_generic.h>
#include <linux/sched/mm.h>

#include "gt/gen8_engine_cs.h"

#include "i915_drv.h"
#include "i915_gem_context.h"
#include "i915_gem_gtt.h"
#include "i915_gem_vm_bind.h"
#include "i915_sw_fence_work.h"
#include "i915_user_extensions.h"
#include "i915_debugger.h"

struct user_fence {
	struct mm_struct *mm;
	void __user *ptr;
	u64 val;
};

struct vm_bind_user_ext {
	struct user_fence bind_fence;
	struct i915_address_space *vm;
	struct list_head metadata_list;
	struct drm_i915_gem_object *obj;
};

#define START(x) ((x)->node.start)
#define LAST(x) ((x)->node.start + (x)->node.size - 1)

INTERVAL_TREE_DEFINE(struct i915_vma, rb, u64, __subtree_last,
		     START, LAST, static inline, i915_vm_bind_it)

#undef START
#undef LAST

struct vm_bind_user_fence {
	struct dma_fence_work base;
	struct user_fence user_fence;
	struct wait_queue_head *wq;
};

static int ufence_work(struct dma_fence_work *work)
{
	struct vm_bind_user_fence *vb =
		container_of(work, struct vm_bind_user_fence, base);
	struct user_fence *ufence = &vb->user_fence;
	struct mm_struct *mm;
	int ret = -EFAULT;

	mm = ufence->mm;
	if (!mm)
		return 0;

	if (mmget_not_zero(mm)) {
		kthread_use_mm(mm);
		if (copy_to_user(ufence->ptr, &ufence->val, sizeof(ufence->val)) == 0) {
			wake_up_all(vb->wq);
			ret = 0;
		}
		kthread_unuse_mm(mm);
		mmput(mm);
	}

	return ret;
}

static void ufence_release(struct dma_fence_work *work)
{
	struct user_fence *ufence =
		&container_of(work, struct vm_bind_user_fence, base)->user_fence;

	if (ufence->mm)
		mmdrop(ufence->mm);
}

static const struct dma_fence_work_ops ufence_ops = {
	.name = "ufence",
	.work = ufence_work,
	.release = ufence_release,
	.no_error_propagation = true,
	.rcu_release = true,
};

static struct i915_sw_fence *
ufence_create(struct i915_address_space *vm, struct vm_bind_user_ext *arg)
{
	struct vm_bind_user_fence *vb;
	struct dma_fence *prev;

	vb = kzalloc(sizeof(*vb), GFP_KERNEL | __GFP_NOWARN);
	if (!vb)
		return NULL;

	dma_fence_work_init(&vb->base, &ufence_ops, vm->i915->sched);
	if (arg->bind_fence.mm) {
		vb->user_fence = arg->bind_fence;
		mmgrab(vb->user_fence.mm);
	}
	vb->wq = &vm->i915->user_fence_wq;

	i915_sw_fence_await(&vb->base.rq.submit); /* signaled by vma_bind */

	/* Preserve the user's write ordering of the user fence seqno */
	prev = __i915_active_fence_fetch_set(&vm->user_fence,
					     &vb->base.rq.fence);
	if (prev) {
		dma_fence_work_chain(&vb->base, prev);
		dma_fence_put(prev);
	}

	dma_fence_work_commit(&vb->base);
	return &vb->base.rq.submit;
}

static int __vm_bind_fence(struct vm_bind_user_ext *arg, u64 addr, u64 val)
{
	u64 x, __user *ptr = u64_to_user_ptr(addr);

	if (arg->bind_fence.mm)
		return -EINVAL;

	if (!access_ok(ptr, sizeof(x)))
		return -EFAULT;

	/* Natural alignment, no page crossings */
	if (!IS_ALIGNED(addr, sizeof(val)))
		return -EINVAL;

	arg->bind_fence.ptr = ptr;
	arg->bind_fence.val = val;
	arg->bind_fence.mm = current->mm;

	return get_user(x, ptr);
}

static int vm_bind_sync_fence(struct i915_user_extension __user *base,
			      void *data)
{
	struct prelim_drm_i915_vm_bind_ext_sync_fence ext;

	if (copy_from_user(&ext, base, sizeof(ext)))
		return -EFAULT;

	return __vm_bind_fence(data, ext.addr, ext.val);
}

static int vm_bind_user_fence(struct i915_user_extension __user *base,
			      void *data)
{
	struct prelim_drm_i915_vm_bind_ext_user_fence ext;

	if (copy_from_user(&ext, base, sizeof(ext)))
		return -EFAULT;

	return __vm_bind_fence(data, ext.addr, ext.val);
}

static int vm_bind_ext_uuid(struct i915_user_extension __user *base,
			    void *data)
{
	struct prelim_drm_i915_vm_bind_ext_uuid __user *ext =
		container_of_user(base, typeof(*ext), base);
	struct vm_bind_user_ext *arg = data;
	struct i915_drm_client *client = arg->vm->client;
	struct i915_vma_metadata *metadata;
	struct i915_uuid_resource *uuid;
	u32 handle;

	if (get_user(handle, &ext->uuid_handle))
		return -EFAULT;

	metadata  = kzalloc(sizeof(*metadata), GFP_KERNEL);
	if (!metadata)
		return -ENOMEM;

	xa_lock(&client->uuids_xa);
	uuid = xa_load(&client->uuids_xa, handle);
	if (!uuid) {
		xa_unlock(&client->uuids_xa);
		kfree(metadata);
		return -ENOENT;
	}
	metadata->uuid = uuid;
	i915_uuid_get(uuid);
	atomic_inc(&uuid->bind_count);
	xa_unlock(&client->uuids_xa);

	list_add_tail(&metadata->vma_link, &arg->metadata_list);
	return 0;
}

#define TGL_MAX_PAT_INDEX	3
#define PVC_MAX_PAT_INDEX	7
#define MTL_MAX_PAT_INDEX	4

static int vm_bind_set_pat(struct i915_user_extension __user *base,
			   void *data)
{
	struct prelim_drm_i915_vm_bind_ext_set_pat ext;
	struct vm_bind_user_ext *arg = data;
	struct drm_i915_private *i915 = arg->vm->i915;
	__u64 max_pat_index;

	if (copy_from_user(&ext, base, sizeof(ext)))
		return -EFAULT;

	if (IS_METEORLAKE(i915))
		max_pat_index = MTL_MAX_PAT_INDEX;
	else if (IS_PONTEVECCHIO(i915))
		max_pat_index = PVC_MAX_PAT_INDEX;
	else if (GRAPHICS_VER(i915) >= 12)
		max_pat_index = TGL_MAX_PAT_INDEX;
	else
		/* For legacy platforms pat_index is a value of
		 * enum i915_cache_level
		 */
		max_pat_index = I915_CACHE_WT;

	if (ext.pat_index > max_pat_index)
		return -EINVAL;

	/*
	 * FIXME: Object should be locked here. And if the ioctl fails,
	 * we probably should revert the change made here.
	 */

	/*
	 * By design, the UMD's are passing in the PAT indices which can
	 * be directly used to set the corresponding bits in PTE.
	 */
	arg->obj->pat_index = ext.pat_index;

	return 0;
}

static const i915_user_extension_fn vm_bind_extensions[] = {
	[PRELIM_I915_USER_EXT_MASK(PRELIM_I915_VM_BIND_EXT_SYNC_FENCE)] = vm_bind_sync_fence,
	[PRELIM_I915_USER_EXT_MASK(PRELIM_I915_VM_BIND_EXT_USER_FENCE)] = vm_bind_user_fence,
	[PRELIM_I915_USER_EXT_MASK(PRELIM_I915_VM_BIND_EXT_UUID)] = vm_bind_ext_uuid,
	[PRELIM_I915_USER_EXT_MASK(PRELIM_I915_VM_BIND_EXT_SET_PAT)] = vm_bind_set_pat,
};

static void metadata_list_free(struct list_head *list)
{
	struct i915_vma_metadata *metadata, *next;

	list_for_each_entry_safe(metadata, next, list, vma_link) {
		list_del_init(&metadata->vma_link);
		atomic_dec(&metadata->uuid->bind_count);
		i915_uuid_put(metadata->uuid);
		kfree(metadata);
	}
}

void i915_vma_metadata_free(struct i915_vma *vma)
{
	if (list_empty(&vma->metadata_list))
		return;

	spin_lock(&vma->metadata_lock);
	metadata_list_free(&vma->metadata_list);
	INIT_LIST_HEAD(&vma->metadata_list);
	spin_unlock(&vma->metadata_lock);
}

struct i915_vma *
i915_gem_vm_bind_lookup_vma(struct i915_address_space *vm, u64 va)
{
	struct i915_vma *vma, *temp;

	assert_vm_bind_held(vm);

	vma = i915_vm_bind_it_iter_first(&vm->va, va, va);
	/* Working around compiler error, remove later */
	if (vma)
		temp = i915_vm_bind_it_iter_next(vma, va + vma->size, -1);
	return vma;
}

static void i915_gem_vm_bind_unpublish(struct i915_vma *vma)
{
	struct i915_address_space *vm = vma->vm;

	mutex_lock_nested(&vm->mutex, SINGLE_DEPTH_NESTING);
	i915_vma_unpublish(vma);
	mutex_unlock(&vm->mutex);
}

static void i915_gem_vm_bind_release(struct i915_vma *vma)
{
	struct drm_i915_gem_object *obj = vma->obj;

	__i915_vma_put(vma);
	i915_gem_object_put(obj);
}

static void i915_gem_vm_bind_remove(struct i915_vma *vma)
{
	assert_vm_bind_held(vma->vm);
	GEM_BUG_ON(list_empty(&vma->vm_bind_link));

	spin_lock(&vma->vm->vm_capture_lock);
	if (!list_empty(&vma->vm_capture_link))
		list_del_init(&vma->vm_capture_link);
	spin_unlock(&vma->vm->vm_capture_lock);

	list_del_init(&vma->vm_bind_link);

	i915_vm_bind_it_remove(vma, &vma->vm->va);
}

void i915_gem_vm_unbind_all(struct i915_address_space *vm)
{
	struct i915_vma *vma, *vn;

	i915_gem_vm_bind_lock(vm);
	i915_gem_object_lock(vm->root_obj, NULL);
	list_for_each_entry_safe(vma, vn, &vm->vm_bind_list, vm_bind_link) {
		i915_gem_vm_bind_remove(vma);
		i915_gem_vm_bind_release(vma);
	}
	list_for_each_entry_safe(vma, vn, &vm->vm_bound_list, vm_bind_link) {
		i915_gem_vm_bind_remove(vma);
		i915_gem_vm_bind_release(vma);
	}
	i915_gem_object_unlock(vm->root_obj);
	i915_gem_vm_bind_unlock(vm);
}

struct unbind_work {
	struct dma_fence_work base;
	struct list_head unbind_link;
	struct i915_vma *vma;
};

static int unbind(struct dma_fence_work *work)
{
	struct unbind_work *w = container_of(work, typeof(*w), base);
	struct i915_address_space *vm = w->vma->vm;

	i915_gem_object_lock(vm->root_obj, NULL);
	i915_gem_vm_bind_unpublish(w->vma);
	i915_gem_object_unlock(vm->root_obj);

	return 0;
}

static void release(struct dma_fence_work *work)
{
	struct unbind_work *w = container_of(work, typeof(*w), base);

	__i915_vma_put(w->vma);
}

static const struct dma_fence_work_ops unbind_ops = {
	.name = "unbind",
	.work = unbind,
	.release = release,
};

static struct dma_fence_work *queue_unbind(struct i915_vma *vma,
					   struct list_head *unbind_head)
{
	struct dma_fence *prev;
	struct unbind_work *w;

	w = kzalloc(sizeof(*w), GFP_KERNEL);
	if (!w)
		return NULL;

	dma_fence_work_init(&w->base, &unbind_ops, vma->vm->i915->sched);
	w->vma = __i915_vma_get(vma);
	INIT_LIST_HEAD(&w->unbind_link);
	list_add_tail(&w->unbind_link, unbind_head);

	prev = i915_active_set_exclusive(&vma->active, &w->base.rq.fence);
	if (!IS_ERR_OR_NULL(prev)) {
		dma_fence_work_chain(&w->base, prev);
		dma_fence_put(prev);
	}

	return &w->base;
}

static int verify_adjacent_segments(struct i915_vma *vma, u64 length)
{
	struct i915_vma *vma_head = vma;
	struct i915_vma *vma_end = NULL;
	u64 vma_size = 0;
	int ret;

	/*
	 * We have to unbind from the start of the mapping.
	 * Partial unbinds are not supported.
	 */
	if (vma->adjacent_start != vma)
		return -EINVAL;

	/*
	 * For segmented BOs, we created many smaller binds. Perform adjacency
	 * search to find rest of VMAs for the VA range.
	 * Don't append VMAs until active_acquire() completed so that easy to
	 * release active references on error.
	 */
	while (vma && (vma_size < length)) {
		ret = i915_active_acquire(&vma->active);
		if (ret) {
			vma_end = vma;
			goto out_del;
		}

		vma_size += vma->size;
		vma = vma->adjacent_next;
	}

	/*
	 * If we reached end of adjacency list, vma will be NULL.
	 * If not NULL, then user attempted a partial unbind (error).
	 */
	vma_end = vma;

	if (vma_size == length && !vma_end)
		return 0; /* success */

	ret = -EINVAL;
out_del:
	for (vma = vma_head; vma && vma != vma_end; vma = vma->adjacent_next)
		i915_active_release(&vma->active);
	return ret;
}

static void
lut_remove(struct drm_i915_gem_object *obj, struct i915_address_space *vm)
{
	struct i915_lut_handle bookmark = {};
	struct i915_lut_handle *lut, *ln;
	LIST_HEAD(close);

	spin_lock(&obj->lut_lock);
	list_for_each_entry_safe(lut, ln, &obj->lut_list, obj_link) {
		struct i915_gem_context *ctx = lut->ctx;

		if (ctx && ctx->client == vm->client) {
			i915_gem_context_get(ctx);
			list_move(&lut->obj_link, &close);
		}

		/* Break long locks, and carefully continue on from this spot */
		if (&ln->obj_link != &obj->lut_list) {
			list_add_tail(&bookmark.obj_link, &ln->obj_link);
			if (cond_resched_lock(&obj->lut_lock))
				list_safe_reset_next(&bookmark, ln, obj_link);
			__list_del_entry(&bookmark.obj_link);
		}
	}
	spin_unlock(&obj->lut_lock);

	list_for_each_entry_safe(lut, ln, &close, obj_link) {
		struct i915_gem_context *ctx = lut->ctx;
		struct i915_vma *vma;

		mutex_lock(&ctx->lut_mutex);
		vma = radix_tree_delete(&ctx->handles_vma, lut->handle);
		if (vma) {
			GEM_BUG_ON(vma->obj != obj);
			GEM_BUG_ON(!atomic_read(&vma->open_count));
			i915_vma_close(vma);
		}
		mutex_unlock(&ctx->lut_mutex);

		i915_gem_context_put(lut->ctx);
		i915_lut_handle_free(lut);
		i915_gem_object_put(obj);
	}
}

int i915_gem_vm_unbind_obj(struct i915_address_space *vm,
			   struct prelim_drm_i915_gem_vm_bind *va)
{
	struct i915_vma *vma, *vma_head, *vma_next;
	struct unbind_work *uwn, *uw = NULL;
	LIST_HEAD(unbind_head);
	int ret;

	/* Handle is not used and must be 0 */
	if (va->handle)
		return -EINVAL;

	i915_debugger_wait_on_discovery(vm->i915, vm->client);

	va->start = intel_noncanonical_addr(INTEL_PPGTT_MSB(vm->i915),
					    va->start);
	/* XXX: Support async and delayed unbind */
	ret = i915_gem_vm_bind_lock_interruptible(vm);
	if (ret)
		return ret;

	vma = i915_gem_vm_bind_lookup_vma(vm, va->start);
	if (!vma) {
		ret = -ENOENT;
		goto out_unlock;
	}
	vma_head = vma;

	if (!i915_gem_object_is_segment(vma->obj)) {
		if (vma->size != va->length) {
			ret = -EINVAL;
			goto out_unlock;
		}

		if (test_bit(I915_VMA_HAS_LUT_BIT, __i915_vma_flags(vma)))
			lut_remove(vma->obj, vm);

		mutex_lock(&vm->mutex);
		ret = i915_active_acquire(&vma->active);
		mutex_unlock(&vm->mutex);
	} else {
		/*
		 * For support of segmented BOs, we make EAGAIN checks for each
		 * segment and test if va->length matches size of the aggregate
		 * set of VMAs. This will do active_acquire() for each segment;
		 * but on error, these are released before returning.
		 */
		mutex_lock(&vm->mutex);
		ret = verify_adjacent_segments(vma, va->length);
		mutex_unlock(&vm->mutex);
	}
	if (ret)
		goto out_unlock;

	/*
	 * As this uses dma_fence_work, we use multiple workers (one per VMA)
	 * as each worker advances the vma->active timeline.
	 */
	for (vma = vma_head; vma; vma = vma->adjacent_next) {
		if (!queue_unbind(vma, &unbind_head)) {
			ret = -ENOMEM;
			break;
		}
	}

	i915_gem_object_lock(vm->root_obj, NULL);
	for (vma = vma_head; vma; vma = vma_next) {
		set_bit(I915_VMA_ERROR_BIT, __i915_vma_flags(vma));
		i915_active_release(&vma->active);
		vma_next = vma->adjacent_next;
		if (!ret) {
			vma->adjacent_next = NULL;
			i915_gem_vm_bind_remove(vma);
			i915_gem_vm_bind_release(vma);
		}
	}
	i915_gem_object_unlock(vm->root_obj);

out_unlock:
	i915_gem_vm_bind_unlock(vm);
	list_for_each_entry_safe(uw, uwn, &unbind_head, unbind_link) {
		i915_sw_fence_set_error_once(&uw->base.rq.submit, ret);
		dma_fence_work_commit(&uw->base);
	}

	return ret;
}

static struct i915_vma *vm_create_vma(struct i915_address_space *vm,
				      struct drm_i915_gem_object *obj,
				      u64 start, u64 offset, u64 size)
{
	struct i915_ggtt_view view;
	struct i915_vma *vma;

	view.type = I915_GGTT_VIEW_PARTIAL;
	view.partial.offset = offset >> PAGE_SHIFT;
	view.partial.size = size >> PAGE_SHIFT;
	vma = i915_vma_instance(obj, vm, &view);
	if (IS_ERR(vma))
		return vma;

	vma->node.start = start;
	vma->node.size = size;
	__set_bit(I915_VMA_PERSISTENT_BIT, __i915_vma_flags(vma));

	return __i915_vma_get(vma);
}

static struct dma_fence *get_unbind_fence(struct drm_mm_node *node)
{
	struct i915_vma *vma;

	if (node->color == I915_COLOR_UNEVICTABLE)
		return NULL;

	vma = container_of(node, typeof(*vma), node);
	return i915_active_fence_get(&vma->active.excl);
}

static int wait_on_unbind(struct i915_address_space *vm, u64 start, u64 end)
{
	int err;

	/*
	 * XXX Wait for old evictions before grabbing locks.
	 */

	err = mutex_lock_interruptible(&vm->mutex);
	if (err)
		return err;

	do {
		struct drm_mm_node *node;
		struct dma_fence *fence;

		node = __drm_mm_interval_first(&vm->mm, start, end - 1);
		if (node->start >= end)
			break;

		start = node->start + node->size;

		fence = get_unbind_fence(node);
		if (likely(!fence))
			continue;

		mutex_unlock(&vm->mutex);

		err = dma_fence_wait(fence, true);
		dma_fence_put(fence);

		if (start >= end || err)
			return err;

		mutex_lock(&vm->mutex);
	} while (1);

	mutex_unlock(&vm->mutex);
	return 0;
}

/*
 * To support segmented BOs, we add VMAs to vma_head as we can now optionally
 * create a list of many smaller binds.
 */
static int vm_bind_get_vmas(struct i915_address_space *vm,
			    struct drm_i915_gem_object *obj,
			    struct prelim_drm_i915_gem_vm_bind *va,
			    struct i915_sw_fence *bind_fence,
			    struct list_head *vma_head)
{
	struct drm_i915_gem_object *vma_obj;
	struct i915_vma *vma, *vn;
	int err = 0, vma_count = 0;

	/*
	 * If segmented BO, divide VA range into multiple smaller binds,
	 * with one per segment of the BO.
	 */
	if (i915_gem_object_has_segments(obj)) {
		unsigned long offset, remaining, start, vma_size;
		struct rb_node *node;

		start = va->start;
		remaining = va->length;

		vma_obj = i915_gem_object_lookup_segment(obj, va->offset, &offset);
		/*
		 * Cannot fail as va->offset already validated; but if somehow
		 * fail here, treat as object not found.
		 */
		if (GEM_WARN_ON(!vma_obj))
			return -ENOENT;

		for (node = &vma_obj->segment_node; node && remaining;
		     node = rb_next(node)) {
			vma_obj = rb_entry(node, typeof(*vma_obj), segment_node);
			vma_size = min_t(size_t, vma_obj->base.size - offset,
					 remaining);
			/*
			 * VMA points to segment BO, allowing faults/migration
			 * to be segment based ('chunk granular')
			 */
			vma = vm_create_vma(vm, vma_obj, start, offset, vma_size);
			if (IS_ERR(vma)) {
				err = PTR_ERR(vma);
				break;
			}
			vma_count++;

			if (bind_fence) {
				/*
				 * If multiple VMAs, add reference to fence so
				 * that we signal waiters when the last of the
				 * set of VMAs is bound. This was done already
				 * during ufence_create() for the first VMA.
				 */
				if (vma_count > 1)
					i915_sw_fence_await(bind_fence);
				vma->bind_fence = bind_fence;
			}
			list_add_tail(&vma->vm_bind_link, vma_head);

			remaining -= vma_size;
			start += vma_size;
			offset = 0;
		}

		if (err) {
			list_for_each_entry_safe(vma, vn, vma_head, vm_bind_link) {
				list_del_init(&vma->vm_bind_link);
				__i915_vma_put(vma);
			}
		}
	} else {
		/* no segments, will create single VMA */
		vma = vm_create_vma(vm, obj, va->start, va->offset, va->length);
		if (IS_ERR(vma)) {
			err = PTR_ERR(vma);
		} else {
			vma->bind_fence = bind_fence;
			list_add_tail(&vma->vm_bind_link, vma_head);
		}
	}

	return err;
}

static int vma_bind_insert(struct i915_vma *vma, u64 pin_flags)
{
	struct i915_address_space *vm = vma->vm;
	struct i915_gem_ww_ctx ww;
	int ret = 0;

	for_i915_gem_ww(&ww, ret, true) {
		if (pin_flags) {
			if (!i915_vm_page_fault_enabled(vm)) {
				ret = i915_gem_object_lock(vm->root_obj, &ww);
				if (ret)
					continue;
			}

			ret = i915_gem_object_lock(vma->obj, &ww);
			if (ret)
				continue;

			ret = i915_vma_pin_ww(vma, &ww, 0, 0,
					      vma->node.start | pin_flags);
			if (ret)
				continue;

			/*
			 * ULLs may be busyspinning waiting for the new buffer,
			 * preventing our blitter clear, unless we preempt it
			 * ahead.  Since there's no dependency between the
			 * immediate bind, and the existing running context, we
			 * have to generate our own priority-inheritance.
			 */
			if (vma->bind_fence) {
				struct dma_fence *f;

				f = i915_active_fence_get(&vma->active.excl);
				if (f) {
					if (dma_fence_is_i915(f))
						i915_request_set_priority(to_request(f),
									  I915_PRIORITY_MAX);
					dma_fence_put(f);
				}
			}

			if (pin_flags & PIN_RESIDENT)
				set_bit(I915_VMA_RESIDENT_BIT, __i915_vma_flags(vma));

			__i915_vma_unpin(vma);
		}

		ret = i915_gem_object_lock(vm->root_obj, &ww);
		if (ret)
			continue;

		list_move_tail(&vma->vm_bind_link, &vm->vm_bind_list);
		i915_vm_bind_it_insert(vma, &vm->va);
	}

	return ret;
}

int i915_gem_vm_bind_obj(struct i915_address_space *vm,
			 struct prelim_drm_i915_gem_vm_bind *va,
			 struct drm_file *file)
{
	struct vm_bind_user_ext ext = {
		.metadata_list = LIST_HEAD_INIT(ext.metadata_list),
		.vm = vm,
	};
	struct i915_vma *adjacency_start = NULL, *vma_prev = NULL;
	struct i915_sw_fence *bind_fence = NULL;
	struct drm_i915_gem_object *obj;
	struct i915_vma *vma, *vma_next;
	struct i915_vma *first_vma = NULL;
	LIST_HEAD(vma_head);
	u64 pin_flags = 0;
	int ret;

	obj = i915_gem_object_lookup(file, va->handle);
	if (!obj)
		return -ENOENT;

	if (!va->length ||
	    !IS_ALIGNED(va->offset | va->length,
			i915_gem_object_max_page_size(obj)) ||
	    range_overflows_t(u64, va->offset, va->length, obj->base.size)) {
		ret = -EINVAL;
		goto put_obj;
	}

	if (obj->vm && obj->vm != vm) {
		ret = -EPERM;
		goto put_obj;
	}

	ext.obj = obj;
	ret = i915_user_extensions(u64_to_user_ptr(va->extensions),
				   vm_bind_extensions,
				   ARRAY_SIZE(vm_bind_extensions),
				   &ext);
	if (ret)
		goto put_obj;

	va->start = intel_noncanonical_addr(INTEL_PPGTT_MSB(vm->i915),
					    va->start);
	ret = wait_on_unbind(vm, va->start, va->start + va->length);
	if (ret)
		goto put_obj;

	i915_debugger_wait_on_discovery(vm->i915, vm->client);

	ret = i915_gem_vm_bind_lock_interruptible(vm);
	if (ret)
		goto put_obj;

	/*
	 * Verify VA isn't already in use (bound) for this VM.
	 * FIXME: This only tests start address and should be checking
	 * that no VMAs intersect the address range.
	 */
	if (i915_gem_vm_bind_lookup_vma(vm, va->start)) {
		ret = -EEXIST;
		goto unlock_vm;
	}

	if (va->flags & PRELIM_I915_GEM_VM_BIND_IMMEDIATE) {
		pin_flags = PIN_OFFSET_FIXED | PIN_USER;
		if (va->flags & PRELIM_I915_GEM_VM_BIND_MAKE_RESIDENT)
			pin_flags |= PIN_RESIDENT;

		bind_fence = ufence_create(vm, &ext);
		if (!bind_fence) {
			ret = -ENOMEM;
			goto unlock_vm;
		}
	} else {
		/* bind during next execbuf, user fence here is invalid */
		if (ext.bind_fence.mm) {
			ret = -EINVAL;
			goto unlock_vm;
		}
	}

	/*
	 * In @vma_head, returns a list of many VMAs for the case
	 * of segmented BO which has VMA per segment.
	 * For the normal case of non-segmented BO, @vma_head will
	 * just contain one VMA.
	 */
	ret = vm_bind_get_vmas(vm, obj, va, bind_fence, &vma_head);
	if (ret)
		goto unlock_vm;
	GEM_BUG_ON(list_empty(&vma_head));

	/*
	 * TODO: follow-on work needed for debugger integration,
	 * for now put the metadata_list in first VMA.
	 */
	first_vma = vma = list_first_entry(&vma_head, typeof(*vma), vm_bind_link);
	if (!list_empty(&ext.metadata_list)) {
		spin_lock(&vma->metadata_lock);
		list_splice_tail_init(&ext.metadata_list, &vma->metadata_list);
		spin_unlock(&vma->metadata_lock);
	}
	/*
	 * The metadata is in VMA so debugger can prepare the event properly
	 * for the IMMEDIATE binds.
	 * Deferred VM-BINDs is not supported by EU Debugger
	 */
	if (va->flags & PRELIM_I915_GEM_VM_BIND_IMMEDIATE)
		i915_debugger_vma_prepare(vm->client, vma);

	list_for_each_entry_safe(vma, vma_next, &vma_head, vm_bind_link) {
		/* Hold object reference until vm_unbind */
		i915_gem_object_get(vma->obj);

		/* apply pat_index from parent */
		vma->obj->pat_index = obj->pat_index;

		/* store first vma to prevent partial unbinds */
		vma->adjacent_start = first_vma;

		ret = vma_bind_insert(vma, pin_flags);
		if (ret)
			break;

		/* TODO: capture should contain single aggregated address range */
		if (va->flags & PRELIM_I915_GEM_VM_BIND_CAPTURE) {
			spin_lock(&vm->vm_capture_lock);
			list_add_tail(&vma->vm_capture_link, &vm->vm_capture_list);
			spin_unlock(&vm->vm_capture_lock);
		}

		/* adjacency list is for use in unbind and capture_vma */
		if (vma_prev)
			vma_prev->adjacent_next = vma;
		else
			adjacency_start = vma;
		vma_prev = vma;
	}

	set_bit(I915_VM_HAS_PERSISTENT_BINDS, &vm->flags);

	if (ret) {
		i915_gem_object_lock(vm->root_obj, NULL);

		/* cleanup VMAs where vma_bind_insert() succeeded */
		for (vma = adjacency_start; vma; vma = vma_next) {
			vma_next = vma->adjacent_next;
			i915_gem_vm_bind_remove(vma);
			i915_gem_vm_bind_release(vma);
		}

		/* cleanup VMAs which failed or didn't attempt vma_bind_insert() */
		list_for_each_entry_safe(vma, vma_next, &vma_head, vm_bind_link) {
			list_del_init(&vma->vm_bind_link);
			i915_gem_vm_bind_unpublish(vma);
			i915_gem_vm_bind_release(vma);
		}

		i915_gem_object_unlock(vm->root_obj);
	}

unlock_vm:
	/* after dropping this lock, user can vm_unbind_obj */
	i915_gem_vm_bind_unlock(vm);
	if (va->flags & PRELIM_I915_GEM_VM_BIND_IMMEDIATE)
		i915_debugger_vma_finalize(vm->client, first_vma, ret);

put_obj:
	i915_gem_object_put(obj);
	metadata_list_free(&ext.metadata_list);
	return ret;
}
