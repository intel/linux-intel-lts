// SPDX-License-Identifier: MIT
/*
 * Copyright © 2012-2023 Intel Corporation
 */

#include <linux/mmu_context.h>
#include <linux/pagevec.h>
#include <linux/swap.h>
#include <linux/sched/mm.h>

#include "i915_drv.h"
#include "i915_gem_ioctls.h"
#include "i915_gem_object.h"
#include "i915_gem_region.h"
#include "i915_scatterlist.h"
#include "i915_sw_fence_work.h"

#ifndef MAX_STACK_ALLOC
#define MAX_STACK_ALLOC 512
#endif

#if IS_ENABLED(CONFIG_MMU_NOTIFIER)
static bool i915_gem_userptr_invalidate(struct mmu_interval_notifier *mni,
					const struct mmu_notifier_range *range,
					unsigned long cur_seq)
{
	if (range->event == MMU_NOTIFY_UNMAP)
		mmu_interval_set_seq(mni, cur_seq);
	return true;
}

static const struct mmu_interval_notifier_ops i915_gem_userptr_notifier_ops = {
	.invalidate = i915_gem_userptr_invalidate,
};

static int
i915_gem_userptr_init__mmu_notifier(struct drm_i915_gem_object *obj)
{
	return mmu_interval_notifier_insert(&obj->userptr.notifier, current->mm,
					    obj->userptr.ptr, obj->base.size,
					    &i915_gem_userptr_notifier_ops);
}

static void
i915_gem_userptr_release(struct drm_i915_gem_object *obj)
{
	i915_gem_object_release_memory_region(obj);
	if (!obj->userptr.notifier.mm)
		return;

	mmu_interval_notifier_remove(&obj->userptr.notifier);
	obj->userptr.notifier.mm = NULL;
}

#else
static int
i915_gem_userptr_init__mmu_notifier(struct drm_i915_gem_object *obj)
{
	obj->userptr.notifier.mm = current->mm;
	mmgrab(current->mm);
	return 0;
}

static void
i915_gem_userptr_release(struct drm_i915_gem_object *obj)
{
	i915_gem_object_release_memory_region(obj);
	if (!obj->userptr.notifier.mm)
		return;

	mmdrop(obj->userptr.notifier.mm);
}

#define mmu_interval_read_begin(n) 0
#define mmu_interval_read_retry(n, seq) false
#endif

struct userptr_work {
	struct dma_fence_work base;
	struct drm_i915_gem_object *obj;
	struct sg_table *pages;
};

struct userptr_chunk {
	struct work_struct work;
	struct mmu_interval_notifier *notifier;
	struct i915_sw_fence *fence;
	unsigned long addr, count;
};

static int __userptr_chunk(struct mmu_interval_notifier *notifier,
			   struct scatterlist *sg,
			   unsigned long start,
			   unsigned long max,
			   unsigned long flags)
{
	struct page *pages[MAX_STACK_ALLOC / sizeof(struct page *)];
	unsigned long count = 0;
	int err;

	/*
	 * Currently when we break out of multi-threaded mode (FOLL_FAST_ONLY)
	 * we completely replay in single-threaded mode, clearing any
	 * in-progress chunking.
	 *
	 * A possible optimization here would be to keep the chunking that has
	 * already happened to this point and only replay the pages which
	 * haven't yet been pinned. For now, take the brute force approach.
	 */

	kthread_use_mm(notifier->mm);
	do {
		unsigned long addr = start + (count << PAGE_SHIFT);
		int n = min_t(int, max - count, ARRAY_SIZE(pages));

		err = pin_user_pages_fast(addr, n, flags, pages);
		if (err <= 0) {
			if (flags & FOLL_FAST_ONLY)
				err = -EAGAIN;
			GEM_BUG_ON(err == 0);
			goto out;
		}

		for (n = 0; n < err; n++) {
			GEM_BUG_ON(!sg || !pages[n]);
			sg_set_page(sg, pages[n], PAGE_SIZE, 0);
			sg = __sg_next(sg);
		}
		count += n;
	} while (count < max);

	err = 0;
out:
	kthread_unuse_mm(notifier->mm);
	return err;
}

static void userptr_chunk(struct work_struct *wrk)
{
	struct userptr_chunk *chunk = container_of(wrk, typeof(*chunk), work);
	struct mmu_interval_notifier *notifier = chunk->notifier;
	struct i915_sw_fence *fence = chunk->fence;
	unsigned long count = chunk->count;
	unsigned long addr = chunk->addr;
	int err;

	err = __userptr_chunk(notifier,
			      memset(chunk, 0, sizeof(*chunk)),
			      addr & PAGE_MASK, count,
			      (addr & ~PAGE_MASK) | FOLL_FAST_ONLY);
	i915_sw_fence_set_error_once(fence, err);
	i915_sw_fence_complete(fence);
}

static void userptr_queue(struct userptr_chunk *chunk)
{
	if (IS_ENABLED(CONFIG_DRM_I915_CHICKEN_PARALLEL_USERPTR))
		queue_work(system_unbound_wq, &chunk->work);
	else
		userptr_chunk(&chunk->work);
}

static void unpin_sg(struct sg_table *sgt)
{
	struct scatterlist *sg;

	for (sg = sgt->sgl; sg; sg = __sg_next(sg)) {
		unsigned long pfn;
		struct page *page;

		page = sg_page(sg);
		if (!page)
			continue;

		for (pfn = 0; pfn < sg->length >> PAGE_SHIFT; pfn++)
			unpin_user_page(nth_page(page, pfn));

		sg_set_page(sg, NULL, 0, 0);
	}
}

static int userptr_work(struct dma_fence_work *base)
{
	struct userptr_work *wrk = container_of(base, typeof(*wrk), base);
	struct drm_i915_gem_object *obj = wrk->obj;
	unsigned long use_threads = FOLL_FAST_ONLY;
	struct sg_table *sgt = wrk->pages;
	struct userptr_chunk *chunk;
	struct i915_sw_fence fence;
	unsigned long seq, addr, n;
	struct scatterlist *sg;
	int err;

	addr = obj->userptr.ptr;
	if (!i915_gem_object_is_readonly(obj))
		addr |= FOLL_WRITE | FOLL_FORCE;
	BUILD_BUG_ON((FOLL_WRITE | FOLL_FORCE) & PAGE_MASK);

	if (!mmget_not_zero(obj->userptr.notifier.mm))
		return -EFAULT;

restart: /* Spread the pagefaulting across the cores (~4MiB per core) */
	err = 0;
	chunk = NULL;
	i915_sw_fence_init_onstack(&fence);
	seq = mmu_interval_read_begin(&obj->userptr.notifier);
	for (n = 0, sg = sgt->sgl; use_threads && n + SG_MAX_SINGLE_ALLOC < sgt->orig_nents;) {
		if (chunk == NULL) {
			chunk = memset(sg, 0, sizeof(*chunk));

			i915_sw_fence_await(&fence);
			chunk->fence = &fence;
			chunk->addr = addr + (n << PAGE_SHIFT);
			chunk->count = -n;
			chunk->notifier = &obj->userptr.notifier;
			INIT_WORK(&chunk->work, userptr_chunk);
		}

		sg += I915_MAX_CHAIN_ALLOC;
		GEM_BUG_ON(!sg_is_chain(sg));
		sg = sg_chain_ptr(sg);

		/* PMD-split locks (2M), try to minimise lock contention */
		n += I915_MAX_CHAIN_ALLOC;
		if (((addr + (n << PAGE_SHIFT) - 1) ^ chunk->addr) & SZ_4M) {
			chunk->count += n;
			userptr_queue(chunk);
			chunk = NULL;
		}

		if (READ_ONCE(fence.error))
			break;
	}
	i915_sw_fence_commit(&fence);

	/* Leaving the last chunk for ourselves */
	if (READ_ONCE(fence.error)) {
		/* Do nothing more if already in error */
		if (chunk) {
			memset(chunk, 0, sizeof(*chunk));
			i915_sw_fence_complete(&fence);
		}
	} else if (chunk) {
		chunk->count += sgt->orig_nents;
		userptr_chunk(&chunk->work);
	} else {
		err = __userptr_chunk(&obj->userptr.notifier, sg,
				      (addr & PAGE_MASK) + (n << PAGE_SHIFT),
				      sgt->orig_nents - n,
				      (addr & ~PAGE_MASK) | use_threads);
	}

	if (n) {
		i915_sw_fence_set_error_once(&fence, err);
		i915_sw_fence_wait(&fence);
		err = fence.error;
	}

	if (err == 0 && mmu_interval_read_retry(&obj->userptr.notifier, seq))
		err = -EAGAIN;
	i915_sw_fence_fini(&fence);
	if (err)
		goto err;

	obj->mm.page_sizes =
		i915_sg_compact(sgt, i915_gem_sg_segment_size(obj));

	if (i915_gem_object_can_bypass_llc(obj))
		drm_clflush_sg(sgt);

	err = i915_gem_gtt_prepare_pages(obj, sgt);
	if (err) {
err:		unpin_sg(sgt);

		if (err == -EAGAIN) {
			use_threads = 0;
			goto restart;
		}

		sg_mark_end(sgt->sgl);
		sgt->nents = 0;
	}

	mmput(obj->userptr.notifier.mm);
	return err;
}

static const struct dma_fence_work_ops userptr_ops = {
	.name = "userptr",
	.work = userptr_work,
};

static int
probe_range(struct mm_struct *mm, unsigned long addr, unsigned long len)
{
	const unsigned long end = addr + len;
	struct vm_area_struct *vma;
	int ret = -EFAULT;

	if (!mmap_read_trylock(mm))
		return 0;

	for (vma = find_vma(mm, addr); vma; vma = vma->vm_next) {
		if (vma->vm_start > addr)
			break;

		if (vma->vm_flags & (VM_IO | VM_PFNMAP))
			break;

		if (vma->vm_end >= end) {
			ret = 0;
			break;
		}

		addr = vma->vm_end;
	}

	mmap_read_unlock(mm);
	return ret;
}

static int i915_gem_userptr_get_pages(struct drm_i915_gem_object *obj)
{
	unsigned int num_pages; /* limited by sg_alloc_table */
	struct userptr_work *wrk;
	struct sg_table *st;
	int err;

	err = probe_range(obj->userptr.notifier.mm,
			  obj->userptr.ptr,
			  obj->base.size);
	if (err)
		return err;

	if (!safe_conversion(&num_pages, obj->base.size >> PAGE_SHIFT))
		return -E2BIG;

	st = kmalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	err = sg_alloc_table(st, num_pages, I915_GFP_ALLOW_FAIL);
	if (err)
		goto err_free;

	wrk = kmalloc(sizeof(*wrk), GFP_KERNEL);
	if (!wrk) {
		err = -ENOMEM;
		goto err_sg;
	}
	dma_fence_work_init(&wrk->base, &userptr_ops,
			    to_i915(obj->base.dev)->mm.sched);
	wrk->obj = obj;
	wrk->pages = st;

	obj->cache_dirty = false;
	__i915_gem_object_set_pages(obj, st, PAGE_SIZE); /* placeholder */
	atomic64_sub(obj->base.size, &obj->mm.region.mem->avail);

	i915_gem_object_migrate_prepare(obj, &wrk->base.rq.fence);
	dma_fence_work_commit(&wrk->base);
	return 0;

err_sg:
	sg_free_table(st);
err_free:
	kfree(st);
	return err;
}

static int
i915_gem_userptr_put_pages(struct drm_i915_gem_object *obj,
			   struct sg_table *pages)
{
	struct sgt_iter sgt_iter;
	struct pagevec pvec;
	struct page *page;
	bool dirty;

	if (!i915_gem_object_migrate_finish(obj))
		i915_gem_gtt_finish_pages(obj, pages);

	__i915_gem_object_release_shmem(obj, pages, false);

	/*
	 * We always mark objects as dirty when they are used by the GPU,
	 * just in case. However, if we set the vma as being read-only we know
	 * that the object will never have been written to.
	 */
	dirty = !i915_gem_object_is_readonly(obj);

	pagevec_init(&pvec);
	for_each_sgt_page(page, sgt_iter, pages) {
		if (!pagevec_add(&pvec, page)) {
			unpin_user_pages_dirty_lock(pvec.pages,
						    pagevec_count(&pvec),
						    true);
			pagevec_reinit(&pvec);
		}
	}
	if (pagevec_count(&pvec))
		unpin_user_pages_dirty_lock(pvec.pages,
					    pagevec_count(&pvec),
					    true);

	atomic64_add(obj->base.size, &obj->mm.region.mem->avail);

	sg_free_table(pages);
	kfree(pages);
	return 0;
}

static int
i915_gem_userptr_dmabuf_export(struct drm_i915_gem_object *obj)
{
	drm_dbg(obj->base.dev, "Exporting userptr no longer allowed\n");

	return -EINVAL;
}

static int
i915_gem_userptr_pwrite(struct drm_i915_gem_object *obj,
			const struct drm_i915_gem_pwrite *args)
{
	drm_dbg(obj->base.dev, "pwrite to userptr no longer allowed\n");

	return -EINVAL;
}

static int
i915_gem_userptr_pread(struct drm_i915_gem_object *obj,
		       const struct drm_i915_gem_pread *args)
{
	drm_dbg(obj->base.dev, "pread from userptr no longer allowed\n");

	return -EINVAL;
}

static const struct drm_i915_gem_object_ops i915_gem_userptr_ops = {
	.name = "i915_gem_object_userptr",
	.flags = I915_GEM_OBJECT_IS_SHRINKABLE |
		 I915_GEM_OBJECT_NO_MMAP,
	.get_pages = i915_gem_userptr_get_pages,
	.put_pages = i915_gem_userptr_put_pages,
	.dmabuf_export = i915_gem_userptr_dmabuf_export,
	.pwrite = i915_gem_userptr_pwrite,
	.pread = i915_gem_userptr_pread,
	.release = i915_gem_userptr_release,
};

/*
 * Creates a new mm object that wraps some normal memory from the process
 * context - user memory.
 *
 * We impose several restrictions upon the memory being mapped
 * into the GPU.
 * 1. It must be page aligned (both start/end addresses, i.e ptr and size).
 * 2. It must be normal system memory, not a pointer into another map of IO
 *    space (e.g. it must not be a GTT mmapping of another object).
 * 3. We only allow a bo as large as we could in theory map into the GTT,
 *    that is we limit the size to the total size of the GTT.
 * 4. The bo is marked as being snoopable. The backing pages are left
 *    accessible directly by the CPU, but reads and writes by the GPU may
 *    incur the cost of a snoop (unless you have an LLC architecture).
 *
 * Synchronisation between multiple users and the GPU is left to userspace
 * through the normal set-domain-ioctl. The kernel will enforce that the
 * GPU relinquishes the VMA before it is returned back to the system
 * i.e. upon free(), munmap() or process termination. However, the userspace
 * malloc() library may not immediately relinquish the VMA after free() and
 * instead reuse it whilst the GPU is still reading and writing to the VMA.
 * Caveat emptor.
 *
 * Also note, that the object created here is not currently a "first class"
 * object, in that several ioctls are banned. These are the CPU access
 * ioctls: mmap(), pwrite and pread. In practice, you are expected to use
 * direct access via your pointer rather than use those ioctls. Another
 * restriction is that we do not allow userptr surfaces to be pinned to the
 * hardware and so we reject any attempt to create a framebuffer out of a
 * userptr.
 *
 * If you think this is a good interface to use to pass GPU memory between
 * drivers, please use dma-buf instead. In fact, wherever possible use
 * dma-buf instead.
 */
int
i915_gem_userptr_ioctl(struct drm_device *dev,
		       void *data,
		       struct drm_file *file)
{
	static struct lock_class_key lock_class;
	struct drm_i915_private *i915 = to_i915(dev);
	struct drm_i915_gem_userptr *args = data;
	struct drm_i915_gem_object *obj;
	u32 handle;
	int ret;

	if (!HAS_LLC(i915) && !HAS_SNOOP(i915)) {
		/* We cannot support coherent userptr objects on hw without
		 * LLC and broken snooping.
		 */
		return -ENODEV;
	}

	if (args->flags & ~(I915_USERPTR_READ_ONLY |
			    I915_USERPTR_UNSYNCHRONIZED))
		return -EINVAL;

	if (i915_gem_object_size_2big(args->user_size))
		return -E2BIG;

	if (!args->user_size ||
	    offset_in_page(args->user_ptr | args->user_size))
		return -EINVAL;

	if (!access_ok(u64_to_user_ptr(args->user_ptr), args->user_size))
		return -EFAULT;

	if (args->flags & I915_USERPTR_UNSYNCHRONIZED)
		return -ENODEV;

	if (args->flags & I915_USERPTR_READ_ONLY) {
		/*
		 * On almost all of the older hw, we cannot tell the GPU that
		 * a page is readonly.
		 */
		if (!to_gt(i915)->vm->has_read_only)
			return -ENODEV;
	}

	i915_gem_flush_free_objects(i915);

	obj = i915_gem_object_alloc();
	if (obj == NULL)
		return -ENOMEM;

	drm_gem_private_object_init(dev, &obj->base, args->user_size);
	i915_gem_object_init(obj, &i915_gem_userptr_ops, &lock_class,
			     I915_BO_STRUCT_PAGE |
			     I915_BO_ALLOC_USER);
	obj->read_domains = I915_GEM_DOMAIN_CPU;
	obj->write_domain = I915_GEM_DOMAIN_CPU;
	i915_gem_object_set_cache_coherency(obj, I915_CACHE_LLC);

	obj->userptr.ptr = args->user_ptr;
	if (args->flags & I915_USERPTR_READ_ONLY)
		i915_gem_object_set_readonly(obj);

	i915_gem_object_init_memory_region(obj,
					   i915->mm.regions[INTEL_REGION_SMEM]);

	/* And keep a pointer to the current->mm for resolving the user pages
	 * at binding. This means that we need to hook into the mmu_notifier
	 * in order to detect if the mmu is destroyed.
	 */
	ret = i915_gem_userptr_init__mmu_notifier(obj);
	if (ret == 0)
		ret = drm_gem_handle_create(file, &obj->base, &handle);

	/* drop reference from allocate - handle holds it now */
	i915_gem_object_put(obj);
	if (ret)
		return ret;

	args->handle = handle;
	return 0;
}
