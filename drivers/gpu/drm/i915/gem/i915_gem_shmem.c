/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2014-2016 Intel Corporation
 */

#include <linux/pagevec.h>
#include <linux/shmem_fs.h>
#include <linux/swap.h>

#include <asm-generic/getorder.h>

#include <drm/drm_cache.h>

#include "gem/i915_gem_region.h"
#include "i915_drv.h"
#include "i915_gem_object.h"
#include "i915_gem_tiling.h"
#include "i915_gemfs.h"
#include "i915_scatterlist.h"
#include "i915_sw_fence_work.h"
#include "i915_trace.h"

struct shmem_work {
	struct dma_fence_work base;
	struct drm_i915_gem_object *obj;
	struct sg_table *pages;
};

struct shmem_chunk {
	struct scatterlist *sg;
	struct work_struct work;
	struct intel_memory_region *mem;
	struct address_space *mapping;
	struct i915_sw_fence *fence;
	unsigned long idx, end;
	unsigned long flags;
#define SHMEM_CLEAR	BIT(0)
#define SHMEM_CLFLUSH	BIT(1)
};

static struct page *
shmem_get_page(struct intel_memory_region *mem,
	       struct address_space *mapping,
	       unsigned long idx)
{
	struct page *page;
	gfp_t gfp;

	/*
	 * Our bo are always dirty and so we require
	 * kswapd to reclaim our pages (direct reclaim
	 * does not effectively begin pageout of our
	 * buffers on its own). However, direct reclaim
	 * only waits for kswapd when under allocation
	 * congestion. So as a result __GFP_RECLAIM is
	 * unreliable and fails to actually reclaim our
	 * dirty pages -- unless you try over and over
	 * again with !__GFP_NORETRY. However, we still
	 * want to fail this allocation rather than
	 * trigger the out-of-memory killer and for
	 * this we want __GFP_RETRY_MAYFAIL.
	 */
	gfp = mapping_gfp_constraint(mapping, ~__GFP_RECLAIM);
	page = shmem_read_mapping_page_gfp(mapping, idx, gfp);
	if (!IS_ERR(page))
		return page;

	/* Preferentially reap our own buffer objects before swapping */
	intel_memory_region_evict(mem, NULL, SZ_2M, PAGE_SIZE);

	/*
	 * We've tried hard to allocate the memory by reaping
	 * our own buffer, now let the real VM do its job and
	 * go down in flames if truly OOM.
	 *
	 * However, since graphics tend to be disposable,
	 * defer the oom here by reporting the ENOMEM back
	 * to userspace.
	 */
	gfp = mapping_gfp_constraint(mapping, ~__GFP_RETRY_MAYFAIL);
	return shmem_read_mapping_page_gfp(mapping, idx, gfp);
}

static int __shmem_chunk(struct scatterlist *sg,
			 struct intel_memory_region *mem,
			 struct address_space *mapping,
			 unsigned long idx,
			 unsigned long end,
			 unsigned long flags,
			 int *error)
{
	GEM_BUG_ON(idx >= end);

	do {
		struct page *page = sg_page(sg);

		if (!page) {
			/* try to backoff quickly if any of our threads fail */
			if (READ_ONCE(*error))
				return *error;

			GEM_BUG_ON(!mapping);
			page = shmem_get_page(mem, mapping, idx);
			if (IS_ERR(page))
				return PTR_ERR(page);

			sg_set_page(sg, page, PAGE_SIZE, 0);
		}

		if (flags) {
			void *ptr = kmap_atomic(page);

			if (flags & SHMEM_CLEAR)
				memset(ptr, 0, sg->length);
			if (flags & SHMEM_CLFLUSH)
				clflush_cache_range(ptr, sg->length);

			kunmap_atomic(ptr);
		}

		if (++idx == end)
			return 0;

		sg = __sg_next(sg);
		GEM_BUG_ON(!sg);
	} while (1);
}

static void shmem_chunk(struct work_struct *wrk)
{
	struct shmem_chunk *chunk = container_of(wrk, typeof(*chunk), work);
	struct intel_memory_region *mem = chunk->mem;
	struct address_space *mapping = chunk->mapping;
	struct i915_sw_fence *fence = chunk->fence;
	struct scatterlist *sg = chunk->sg;
	unsigned long flags = chunk->flags;
	unsigned long idx = chunk->idx;
	unsigned long end = chunk->end;

	if (sg == (void *)chunk)
		memset(chunk, 0, sizeof(*chunk));
	else
		kunmap(sg_page(sg));

	if (!READ_ONCE(fence->error)) {
		int err;

		err = __shmem_chunk(sg, mem, mapping,
				    idx, end, flags,
				    &fence->error);
		i915_sw_fence_set_error_once(fence, err);
	}

	i915_sw_fence_complete(fence);
}

static void shmem_queue(struct shmem_chunk *chunk)
{
	if (IS_ENABLED(CONFIG_DRM_I915_CHICKEN_PARALLEL_SHMEMFS))
		queue_work(system_unbound_wq, &chunk->work);
	else
		shmem_chunk(&chunk->work);
}

static int shmem_create(struct shmem_work *wrk)
{
	const unsigned int limit = boot_cpu_data.x86_cache_size << 9 ?: SZ_8M;
	const uint32_t max_segment = i915_sg_segment_size();
	struct drm_i915_gem_object *obj = wrk->obj;
	const int nid = dev_to_node(obj->base.dev->dev);
	u64 remain = obj->base.size, start;
	struct sg_table *sgt = wrk->pages;
	struct shmem_chunk *chunk = NULL;
	struct i915_sw_fence fence;
	int min_order = MAX_ORDER;
	struct scatterlist *sg;
	unsigned long flags;
	unsigned long n;
	gfp_t gfp;
	int err;

	flags = 0;
	if ((obj->flags & (I915_BO_ALLOC_USER | I915_BO_CPU_CLEAR)) &&
	    !(obj->flags & I915_BO_SKIP_CLEAR)) {
		flags |= SHMEM_CLEAR;
		if (i915_gem_object_can_bypass_llc(obj) ||
		    !(obj->cache_coherent & I915_BO_CACHE_COHERENT_FOR_WRITE))
			flags |= SHMEM_CLFLUSH;
	}

	i915_sw_fence_init_onstack(&fence);

	gfp = mapping_gfp_constraint(obj->base.filp->f_mapping, ~__GFP_RECLAIM);

	n = 0;
	sg = sgt->sgl;
	obj->mm.page_sizes = 0;
	do {
		int order = ilog2(min_t(u64, remain, max_segment)) - PAGE_SHIFT;
		struct page *page;

		order = min(order, min_order);
		do {
			page = alloc_pages_node(nid, gfp, order);
			if (page || gfp & __GFP_DIRECT_RECLAIM)
				break;

			if (order > get_order(SZ_2M))
				order = get_order(SZ_2M);
			else if (order > get_order(SZ_64K))
				order = get_order(SZ_64K);
			else
				order = 0;

			if (order <= PAGE_ALLOC_COSTLY_ORDER)
				gfp |= __GFP_KSWAPD_RECLAIM;
			if (order == 0) {
				intel_memory_region_evict(obj->mm.region.mem,
							  NULL, SZ_2M, PAGE_SIZE);
				gfp |= __GFP_DIRECT_RECLAIM;
			}

			min_order = min(min_order, order);
		} while (1);
		if (!page) {
			i915_sw_fence_set_error_once(&fence, -ENOMEM);
			break;
		}

		SetPagePrivate2(page);
		sg_set_page(sg, page, BIT(order + PAGE_SHIFT), 0);
		obj->mm.page_sizes |= sg->length;

		if (flags && chunk == NULL) {
			chunk = kmap(page);
			if (!chunk) {
				i915_sw_fence_set_error_once(&fence, -ENOMEM);
				break;
			}

			i915_sw_fence_await(&fence);
			chunk->sg = sg;
			chunk->fence = &fence;
			chunk->idx = n;
			chunk->flags = flags;
			INIT_WORK(&chunk->work, shmem_chunk);

			start = remain;
		}
		n++;

		GEM_BUG_ON(sg->length > remain);
		remain -= sg->length;
		if (!remain)
			break;

		if (sg_is_last(sg)) {
			struct scatterlist *chain;

			chain = (void *)__get_free_page(I915_GFP_ALLOW_FAIL);
			if (!chain) {
				i915_sw_fence_set_error_once(&fence, -ENOMEM);
				break;
			}
			sg_init_table(chain, SG_MAX_SINGLE_ALLOC);

			sg_unmark_end(memcpy(chain, sg, sizeof(*sg)));
			__sg_chain(sg, chain);
			sgt->orig_nents += I915_MAX_CHAIN_ALLOC;

			if (chunk && chunk->sg == sg)
				chunk->sg = chain;

			GEM_BUG_ON(sg_chain_ptr(sg) != chain);
			sg = chain;

			cond_resched();
		}
		GEM_BUG_ON(sg_is_last(sg));
		sg++;

		if (chunk && start - remain > limit) {
			chunk->end = n;
			shmem_queue(chunk);
			chunk = NULL;
		}
	} while (1);
	i915_sw_fence_commit(&fence);

	sg_mark_end(sg);
	sgt->nents = n;
	GEM_BUG_ON(sgt->nents > sgt->orig_nents);

	/* Leaving the last chunk for ourselves */
	if (chunk) {
		chunk->end = n;
		shmem_chunk(&chunk->work);
	}

	i915_sw_fence_wait(&fence);
	err = fence.error;

	i915_sw_fence_fini(&fence);
	if (err)
		goto err;

	err = i915_gem_gtt_prepare_pages(obj, sgt);
	if (err)
		goto err;

	return 0;

err:
	for (sg = sgt->sgl; sg; sg = __sg_next(sg)) {
		struct page *page;

		page = sg_page(sg);
		if (!page)
			break;

		ClearPagePrivate2(page);
		__free_pages(page, get_order(sg->length));
		sg_set_page(sg, NULL, 0, 0);
	}
	sg_mark_end(sgt->sgl);
	sgt->nents = 0;

	return err;
}

static int shmem_swapin(struct shmem_work *wrk)
{
	struct drm_i915_gem_object *obj = wrk->obj;
	const unsigned int num_pages = obj->base.size >> PAGE_SHIFT;
	struct address_space *mapping = obj->base.filp->f_mapping;
	struct sg_table *sgt = wrk->pages;
	struct shmem_chunk *chunk = NULL;
	struct i915_sw_fence fence;
	struct scatterlist *sg;
	unsigned long flags;
	unsigned long n;
	int err;

	flags = 0;
	if (i915_gem_object_can_bypass_llc(obj) ||
	    !(obj->cache_coherent & I915_BO_CACHE_COHERENT_FOR_WRITE))
		flags |= SHMEM_CLFLUSH;

	err = 0;
	i915_sw_fence_init_onstack(&fence);
	for (n = 0, sg = sgt->sgl; n + SG_MAX_SINGLE_ALLOC < num_pages;) {
		struct scatterlist *chain;

		if (chunk == NULL) {
			chunk = memset(sg, 0, sizeof(*chunk));

			i915_sw_fence_await(&fence);
			chunk->sg = sg;
			chunk->fence = &fence;
			chunk->mem = obj->mm.region.mem;
			chunk->mapping = mapping;
			chunk->idx = n;
			chunk->flags = flags;
			INIT_WORK(&chunk->work, shmem_chunk);
		}

		sg += I915_MAX_CHAIN_ALLOC;
		GEM_BUG_ON(!sg_is_last(sg));

		chain = (void *)__get_free_page(I915_GFP_ALLOW_FAIL);
		if (!chain) {
			i915_sw_fence_set_error_once(&fence, -ENOMEM);
			break;
		}
		sg_init_table(chain, SG_MAX_SINGLE_ALLOC);

		__sg_chain(sg, chain);
		sgt->orig_nents += I915_MAX_CHAIN_ALLOC;
		sg = chain;

		n += I915_MAX_CHAIN_ALLOC;
		if (n - chunk->idx > SZ_2M >> PAGE_SHIFT) {
			chunk->end = n;
			shmem_queue(chunk);
			chunk = NULL;
		}

		cond_resched();
	}
	i915_sw_fence_commit(&fence);

	/* Leaving the last chunk for ourselves */
	if (chunk) {
		chunk->end = num_pages;
		shmem_chunk(&chunk->work);
	} else {
		err = __shmem_chunk(sg, obj->mm.region.mem, mapping,
				    n, num_pages, flags,
				    &fence.error);
	}

	if (n) {
		i915_sw_fence_set_error_once(&fence, err);
		i915_sw_fence_wait(&fence);
		err = fence.error;
	}

	i915_sw_fence_fini(&fence);
	if (err)
		goto err;

	obj->mm.page_sizes =
		i915_sg_compact(sgt, i915_gem_sg_segment_size(obj));

	if (i915_gem_object_needs_bit17_swizzle(obj))
		i915_gem_object_do_bit_17_swizzle(obj, sgt);

	err = i915_gem_gtt_prepare_pages(obj, sgt);
	if (err)
		goto err;

	return 0;

err:
	for (sg = sgt->sgl; sg; sg = __sg_next(sg)) {
		unsigned long pfn, end;
		struct page *page;

		page = sg_page(sg);
		if (!page)
			continue;

		pfn = 0;
		end = sg->length >> PAGE_SHIFT;
		do {
			put_page(nth_page(page, pfn));
		} while (++pfn < end);

		sg_set_page(sg, NULL, 0, 0);
	}
	sg_mark_end(sgt->sgl);
	sgt->nents = 0;

	/*
	 * shmemfs first checks if there is enough memory to allocate the page
	 * and reports ENOSPC should there be insufficient, along with the usual
	 * ENOMEM for a genuine allocation failure.
	 *
	 * We use ENOSPC in our driver to mean that we have run out of aperture
	 * space and so want to translate the error from shmemfs back to our
	 * usual understanding of ENOMEM.
	 */
	if (err == -ENOSPC)
		err = -ENOMEM;

	return err;
}

static int shmem_work(struct dma_fence_work *base)
{
	struct shmem_work *wrk = container_of(base, typeof(*wrk), base);

	if (IS_ENABLED(CONFIG_DRM_I915_CHICKEN_NUMA_ALLOC) &&
	    !wrk->obj->base.filp->f_inode->i_blocks)
		return shmem_create(wrk);
	else
		return shmem_swapin(wrk);
}

static void shmem_work_release(struct dma_fence_work *base)
{
	struct shmem_work *wrk = container_of(base, typeof(*wrk), base);

	i915_gem_object_put(wrk->obj);
}

static const struct dma_fence_work_ops shmem_ops = {
	.name = "shmem",
	.work = shmem_work,
	.release = shmem_work_release,
};

static int shmem_get_pages(struct drm_i915_gem_object *obj)
{
	struct intel_memory_region *mem = obj->mm.region.mem;
	struct shmem_work *wrk;
	unsigned int num_pages;
	struct sg_table *st;
	int err;

	if (!safe_conversion(&num_pages, obj->base.size >> PAGE_SHIFT))
		return -E2BIG;

	/*
	 * If there's no chance of allocating enough pages for the whole
	 * object, bail early.
	 */
	if (num_pages > totalram_pages())
		return -E2BIG;

	st = kmalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	err = sg_alloc_table(st,
			     min_t(unsigned int, num_pages, SG_MAX_SINGLE_ALLOC),
			     I915_GFP_ALLOW_FAIL);
	if (err)
		goto err_free;

	wrk = kmalloc(sizeof(*wrk), GFP_KERNEL);
	if (!wrk) {
		err = -ENOMEM;
		goto err_sg;
	}
	dma_fence_work_init(&wrk->base, &shmem_ops,
			    to_i915(obj->base.dev)->mm.sched);
	wrk->obj = i915_gem_object_get(obj);
	wrk->pages = st;

	i915_gem_object_migrate_prepare(obj, &wrk->base.rq.fence);
	atomic64_sub(obj->base.size, &mem->avail);
	mapping_set_unevictable(obj->base.filp->f_mapping);

	obj->cache_dirty = false;
	__i915_gem_object_set_pages(obj, st, PAGE_SIZE); /* placeholder */

	dma_fence_work_commit_imm_if(&wrk->base,
				     obj->flags & I915_BO_SYNC_HINT ||
				     obj->base.size <= SZ_64K);
	return 0;

err_sg:
	sg_free_table(st);
err_free:
	kfree(st);
	return err;
}

static void
shmem_truncate(struct drm_i915_gem_object *obj)
{
	/*
	 * Our goal here is to return as much of the memory as
	 * is possible back to the system as we are called from OOM.
	 * To do this we must instruct the shmfs to drop all of its
	 * backing pages, *now*.
	 */
	shmem_truncate_range(file_inode(obj->base.filp), 0, (loff_t)-1);
}

static void
shmem_writeback(struct drm_i915_gem_object *obj)
{
	struct address_space *mapping;
	struct writeback_control wbc = {
		.sync_mode = WB_SYNC_NONE,
		.nr_to_write = SWAP_CLUSTER_MAX,
		.range_start = 0,
		.range_end = LLONG_MAX,
		.for_reclaim = 1,
	};
	unsigned long i;

	/*
	 * Leave mmapings intact (GTT will have been revoked on unbinding,
	 * leaving only CPU mmapings around) and add those pages to the LRU
	 * instead of invoking writeback so they are aged and paged out
	 * as normal.
	 */
	mapping = obj->base.filp->f_mapping;

	/* Begin writeback on each dirty page */
	for (i = 0; i < obj->base.size >> PAGE_SHIFT; i++) {
		struct page *page;

		page = find_lock_page(mapping, i);
		if (!page)
			continue;

		if (!page_mapped(page) && clear_page_dirty_for_io(page)) {
			int ret;

			SetPageReclaim(page);
			ret = mapping->a_ops->writepage(page, &wbc);
			if (!PageWriteback(page))
				ClearPageReclaim(page);
			if (!ret)
				goto put;
		}
		unlock_page(page);
put:
		put_page(page);
	}
}

void
__i915_gem_object_release_shmem(struct drm_i915_gem_object *obj,
				struct sg_table *pages,
				bool needs_clflush)
{
	GEM_BUG_ON(obj->mm.madv == __I915_MADV_PURGED);

	if (needs_clflush &&
	    (obj->read_domains & I915_GEM_DOMAIN_CPU) == 0 &&
	    !(obj->cache_coherent & I915_BO_CACHE_COHERENT_FOR_READ))
		drm_clflush_sg(pages);

	__start_cpu_write(obj);
}

static void check_release_pagevec(struct pagevec *pvec)
{
	check_move_unevictable_pages(pvec);
	__pagevec_release(pvec);
	cond_resched();
}

static void page_release(struct page *page, struct pagevec *pvec)
{
	if (!pagevec_add(pvec, page))
		check_release_pagevec(pvec);
}

static bool need_swap(const struct drm_i915_gem_object *obj)
{
	if (i915_gem_object_migrate_has_error(obj))
		return false;

	if (mapping_mapped(obj->base.filp->f_mapping))
		return true;

	if (kref_read(&obj->base.refcount) == 0)
		return false;

	if (i915_gem_object_is_volatile(obj) ||
	    obj->mm.madv != I915_MADV_WILLNEED)
		return false;

	if (obj->flags & I915_BO_ALLOC_USER && !i915_gem_object_inuse(obj))
		return false;

	return true;
}

/* inlined delete_from_page_cache() to aide porting to different kernels */
static void i915_delete_from_page_cache(struct page *page)
{
	struct address_space *mapping = page_mapping(page);
	XA_STATE(xas, &mapping->i_pages, page->index);

	GEM_BUG_ON(!PageLocked(page));
	xas_lock_irq(&xas);

	__dec_lruvec_page_state(page, NR_FILE_PAGES);
	__dec_lruvec_page_state(page, NR_SHMEM);

	xas_set_order(&xas, page->index, 0);
	xas_store(&xas, NULL);
	xas_init_marks(&xas);

	/* Leave page->index set: truncation lookup relies upon it */
	page->mapping = NULL;
	mapping->nrpages--;

	xas_unlock_irq(&xas);
	put_page(page);
}

/* inlined add_to_page_cache_locked() to aide porting to different kernels */
static int i915_add_to_page_cache_locked(struct page *page,
					 struct address_space *mapping,
					 pgoff_t offset,
					 gfp_t gfp)
{
	XA_STATE(xas, &mapping->i_pages, offset);
	int error;

	get_page(page);
	page->mapping = mapping;
	page->index = offset;

	do {
		unsigned int order = xa_get_order(xas.xa, xas.xa_index);
		void *entry, *old = NULL;

		if (order > thp_order(page))
			xas_split_alloc(&xas, xa_load(xas.xa, xas.xa_index),
					order, gfp);
		xas_lock_irq(&xas);
		xas_for_each_conflict(&xas, entry) {
			old = entry;
			if (!xa_is_value(entry)) {
				xas_set_err(&xas, -EEXIST);
				goto unlock;
			}
		}

		if (old) {
			/* entry may have been split before we acquired lock */
			order = xa_get_order(xas.xa, xas.xa_index);
			if (order > thp_order(page)) {
				xas_split(&xas, old, order);
				xas_reset(&xas);
			}
		}

		xas_store(&xas, page);
		if (xas_error(&xas))
			goto unlock;

		mapping->nrpages++;

		__inc_lruvec_page_state(page, NR_FILE_PAGES);
unlock:
		xas_unlock_irq(&xas);
	} while (xas_nomem(&xas, gfp));

	if (xas_error(&xas)) {
		error = xas_error(&xas);
		goto error;
	}

	return 0;
error:
	/* Leave page->index set: truncation relies upon it */
	page->mapping = NULL;
	put_page(page);
	return error;
}

int i915_gem_object_put_pages_shmem(struct drm_i915_gem_object *obj, struct sg_table *pages)
{
	struct intel_memory_region *mem = obj->mm.region.mem;
	struct address_space *mapping = obj->base.filp->f_mapping;
	struct inode *inode = obj->base.filp->f_inode;
	bool do_swap = need_swap(obj);
	struct sgt_iter sgt_iter;
	struct pagevec pvec;
	struct page *page;

	mapping_clear_unevictable(mapping);
	i915_gem_object_migrate_finish(obj);

	if (!pages->nents)
		goto empty;

	pagevec_init(&pvec);
	if (inode->i_blocks) {
		bool clflush;

		clflush = false;
		if (i915_gem_object_can_bypass_llc(obj) ||
		    !(obj->cache_coherent & I915_BO_CACHE_COHERENT_FOR_WRITE))
			clflush = true;

		for_each_sgt_page(page, sgt_iter, pages) {
			GEM_BUG_ON(PagePrivate2(page));

			if (do_swap) {
				if (clflush) {
					void *ptr = kmap_atomic(page);
					clflush_cache_range(ptr, PAGE_SIZE);
					kunmap_atomic(ptr);
				}
				set_page_dirty(page);
				mark_page_accessed(page);
			} else {
				cancel_dirty_page(page);
			}

			page_release(page, &pvec);
		}
	} else if (do_swap) { /* instantiate shmemfs backing store for swap */
		struct scatterlist *sg;
		long idx = 0;

		for (sg = pages->sgl; sg; sg = __sg_next(sg)) {
			int order = get_order(sg->length);
			int i;

			page = sg_page(sg);
			if (TestClearPagePrivate2(page) && order)
				split_page(page, order);

			for (i = 0; i < BIT(order); i++) {
				struct page *p = nth_page(page, i);

				lock_page(p);

				SetPageUptodate(p);
				set_page_dirty(p);
				mark_page_accessed(p);

				if (i915_add_to_page_cache_locked(p, mapping, idx, I915_GFP_ALLOW_FAIL)) {
					unlock_page(p);

					if (pagevec_count(&pvec))
						check_release_pagevec(&pvec);

					mapping_set_unevictable(mapping);

					while (idx--) {
						p = find_lock_page(mapping, idx);
						GEM_BUG_ON(!p);

						cancel_dirty_page(page);
						i915_delete_from_page_cache(p);
						unlock_page(p);
					}

					GEM_BUG_ON(mapping->nrpages);
					return -ENOMEM;
				}

				if (!PageLRU(p)) {
					__SetPageSwapBacked(p);
					lru_cache_add(p);
				}

				inc_lruvec_page_state(p, NR_SHMEM);
				unlock_page(p);
				idx++;

				page_release(p, &pvec);
			}
		}

		SHMEM_I(inode)->alloced = mapping->nrpages;
		inode->i_blocks = mapping->nrpages * (PAGE_SIZE / 512);
	} else {
		struct scatterlist *sg;

		/* XXX clear-on-free? */
		for (sg = pages->sgl; sg; sg = __sg_next(sg)) {
			int order = get_order(sg->length);
			int i;

			page = sg_page(sg);
			if (TestClearPagePrivate2(page)) {
				__free_pages(page, order);
				continue;
			}

			for (i = 0; i < BIT(order); i++)
				page_release(nth_page(page, i), &pvec);
		}
	}
	if (pagevec_count(&pvec))
		check_release_pagevec(&pvec);

	i915_gem_gtt_finish_pages(obj, pages);
	if (do_swap && i915_gem_object_needs_bit17_swizzle(obj))
		i915_gem_object_save_bit_17_swizzle(obj, pages);

empty:
	atomic64_add(obj->base.size, &mem->avail);

	sg_free_table(pages);
	kfree(pages);
	return 0;
}

static int
shmem_put_pages(struct drm_i915_gem_object *obj, struct sg_table *pages)
{
	if (likely(i915_gem_object_has_struct_page(obj)))
		return i915_gem_object_put_pages_shmem(obj, pages);
	else
		return i915_gem_object_put_pages_phys(obj, pages);
}

static int
shmem_pwrite(struct drm_i915_gem_object *obj,
	     const struct drm_i915_gem_pwrite *arg)
{
	struct address_space *mapping = obj->base.filp->f_mapping;
	char __user *user_data = u64_to_user_ptr(arg->data_ptr);
	u64 remain, offset;
	unsigned int pg;

	/* Caller already validated user args */
	GEM_BUG_ON(!access_ok(user_data, arg->size));

	if (!i915_gem_object_has_struct_page(obj))
		return i915_gem_object_pwrite_phys(obj, arg);

	/*
	 * Before we instantiate/pin the backing store for our use, we
	 * can prepopulate the shmemfs filp efficiently using a write into
	 * the pagecache. We avoid the penalty of instantiating all the
	 * pages, important if the user is just writing to a few and never
	 * uses the object on the GPU, and using a direct write into shmemfs
	 * allows it to avoid the cost of retrieving a page (either swapin
	 * or clearing-before-use) before it is overwritten.
	 */
	if (i915_gem_object_has_pages(obj))
		return -ENODEV;

	if (obj->mm.madv != I915_MADV_WILLNEED)
		return -EFAULT;

	/*
	 * Before the pages are instantiated the object is treated as being
	 * in the CPU domain. The pages will be clflushed as required before
	 * use, and we can freely write into the pages directly. If userspace
	 * races pwrite with any other operation; corruption will ensue -
	 * that is userspace's prerogative!
	 */

	remain = arg->size;
	offset = arg->offset;
	pg = offset_in_page(offset);

	do {
		unsigned int len, unwritten;
		struct page *page;
		void *data, *vaddr;
		int err;
		char c;

		len = PAGE_SIZE - pg;
		if (len > remain)
			len = remain;

		/* Prefault the user page to reduce potential recursion */
		err = __get_user(c, user_data);
		if (err)
			return err;

		err = __get_user(c, user_data + len - 1);
		if (err)
			return err;

		err = pagecache_write_begin(obj->base.filp, mapping,
					    offset, len, 0,
					    &page, &data);
		if (err < 0)
			return err;

		vaddr = kmap_atomic(page);
		unwritten = __copy_from_user_inatomic(vaddr + pg,
						      user_data,
						      len);
		kunmap_atomic(vaddr);

		err = pagecache_write_end(obj->base.filp, mapping,
					  offset, len, len - unwritten,
					  page, data);
		if (err < 0)
			return err;

		/* We don't handle -EFAULT, leave it to the caller to check */
		if (unwritten)
			return -ENODEV;

		remain -= len;
		user_data += len;
		offset += len;
		pg = 0;
	} while (remain);

	return 0;
}

static int
shmem_pread(struct drm_i915_gem_object *obj,
	    const struct drm_i915_gem_pread *arg)
{
	if (!i915_gem_object_has_struct_page(obj))
		return i915_gem_object_pread_phys(obj, arg);

	return -ENODEV;
}

static void shmem_release(struct drm_i915_gem_object *obj)
{
	if (obj->flags & I915_BO_STRUCT_PAGE)
		i915_gem_object_release_memory_region(obj);

	fput(obj->base.filp);
}

const struct drm_i915_gem_object_ops i915_gem_shmem_ops = {
	.name = "i915_gem_object_shmem",
	.flags = I915_GEM_OBJECT_IS_SHRINKABLE,

	.get_pages = shmem_get_pages,
	.put_pages = shmem_put_pages,
	.truncate = shmem_truncate,
	.writeback = shmem_writeback,

	.pwrite = shmem_pwrite,
	.pread = shmem_pread,

	.release = shmem_release,
};

static int __create_shmem(struct drm_i915_private *i915,
			  struct drm_gem_object *obj,
			  resource_size_t size)
{
	unsigned long flags = VM_NORESERVE;
	struct file *filp;

	drm_gem_private_object_init(&i915->drm, obj, size);

	if (i915->mm.gemfs)
		filp = shmem_file_setup_with_mnt(i915->mm.gemfs, "i915", size,
						 flags);
	else
		filp = shmem_file_setup("i915", size, flags);
	if (IS_ERR(filp))
		return PTR_ERR(filp);

	i_size_write(filp->f_inode, size);
	obj->filp = filp;
	return 0;
}

static int shmem_object_init(struct intel_memory_region *mem,
			     struct drm_i915_gem_object *obj,
			     resource_size_t size,
			     unsigned int flags)
{
	static struct lock_class_key lock_class;
	struct drm_i915_private *i915 = mem->i915;
	struct address_space *mapping;
	unsigned int cache_level;
	gfp_t mask;
	int ret;

	ret = __create_shmem(i915, &obj->base, size);
	if (ret)
		return ret;

	mask = GFP_HIGHUSER | __GFP_RECLAIMABLE;
	if (IS_I965GM(i915) || IS_I965G(i915)) {
		/* 965gm cannot relocate objects above 4GiB. */
		mask &= ~__GFP_HIGHMEM;
		mask |= __GFP_DMA32;
	}
	mask |= __GFP_RETRY_MAYFAIL | __GFP_NOWARN;

	mapping = obj->base.filp->f_mapping;
	mapping_set_gfp_mask(mapping, mask);
	GEM_BUG_ON(!(mapping_gfp_mask(mapping) & __GFP_RECLAIM));

	i915_gem_object_init(obj, &i915_gem_shmem_ops, &lock_class,
			     flags | I915_BO_STRUCT_PAGE);
	obj->write_domain = I915_GEM_DOMAIN_CPU;
	obj->read_domains = I915_GEM_DOMAIN_CPU;

	/*
	 * Soft-pinned buffers need to be 1-way coherent from MTL onward
	 * because GPU is no longer snooping CPU cache by default. Make it
	 * default setting and let others to modify as needed later
	 */
	if (IS_DGFX(i915) || HAS_LLC(i915) || GRAPHICS_VER_FULL(i915) >= IP_VER(12, 70))
		/*
		 * On some devices, we can have the GPU use the LLC (the CPU
		 * cache) for about a 10% performance improvement
		 * compared to uncached.  Graphics requests other than
		 * display scanout are coherent with the CPU in
		 * accessing this cache.  This means in this mode we
		 * don't need to clflush on the CPU side, and on the
		 * GPU side we only need to flush internal caches to
		 * get data visible to the CPU.
		 *
		 * However, we maintain the display planes as UC, and so
		 * need to rebind when first used as such.
		 */
		cache_level = I915_CACHE_LLC;
	else
		cache_level = I915_CACHE_NONE;

	i915_gem_object_set_cache_coherency(obj, cache_level);

	i915_gem_object_init_memory_region(obj, mem);

	return 0;
}

struct drm_i915_gem_object *
i915_gem_object_create_shmem(struct drm_i915_private *i915,
			     resource_size_t size)
{
	return i915_gem_object_create_region(i915->mm.regions[INTEL_REGION_SMEM],
					     size, 0);
}

/* Allocate a new GEM object and fill it with the supplied data */
struct drm_i915_gem_object *
i915_gem_object_create_shmem_from_data(struct drm_i915_private *dev_priv,
				       const void *data, resource_size_t size)
{
	struct drm_i915_gem_object *obj;
	struct file *file;
	resource_size_t offset;
	int err;

	obj = i915_gem_object_create_shmem(dev_priv, round_up(size, PAGE_SIZE));
	if (IS_ERR(obj))
		return obj;

	GEM_BUG_ON(obj->write_domain != I915_GEM_DOMAIN_CPU);

	file = obj->base.filp;
	offset = 0;
	do {
		unsigned int len = min_t(typeof(size), size, PAGE_SIZE);
		struct page *page;
		void *pgdata, *vaddr;

		err = pagecache_write_begin(file, file->f_mapping,
					    offset, len, 0,
					    &page, &pgdata);
		if (err < 0)
			goto fail;

		vaddr = kmap(page);
		memcpy(vaddr, data, len);
		kunmap(page);

		err = pagecache_write_end(file, file->f_mapping,
					  offset, len, len,
					  page, pgdata);
		if (err < 0)
			goto fail;

		size -= len;
		data += len;
		offset += len;
	} while (size);

	return obj;

fail:
	i915_gem_object_put(obj);
	return ERR_PTR(err);
}

static int init_shmem(struct intel_memory_region *mem)
{
	i915_gemfs_init(mem->i915);
	intel_memory_region_set_name(mem, "system");

	return 0; /* We have fallback to the kernel mnt if gemfs init failed. */
}

static void release_shmem(struct intel_memory_region *mem)
{
	i915_gemfs_fini(mem->i915);
}

static const struct intel_memory_region_ops shmem_region_ops = {
	.init = init_shmem,
	.release = release_shmem,
	.init_object = shmem_object_init,
};

static u64 total_pages(struct intel_gt *gt)
{
	struct drm_i915_private *i915 = gt->i915;
	int nid;

	nid = dev_to_node(i915->drm.dev);
	if (nid != NUMA_NO_NODE) {
		dev_info(i915->drm.dev,
			 "Attaching to %luMiB of system memory on node %d\n",
			 node_present_pages(nid) >> (20 - PAGE_SHIFT),
			 nid);
	}

	return (u64)totalram_pages() << PAGE_SHIFT;
}

struct intel_memory_region *i915_gem_shmem_setup(struct intel_gt *gt,
						 u16 type, u16 instance)
{
	return intel_memory_region_create(gt, 0,
					  total_pages(gt),
					  PAGE_SIZE, 0, 0,
					  type, instance,
					  &shmem_region_ops);
}

bool i915_gem_object_is_shmem(const struct drm_i915_gem_object *obj)
{
	return obj->ops == &i915_gem_shmem_ops;
}

#if IS_ENABLED(CONFIG_DRM_I915_SELFTEST)
#include "selftests/i915_gem_shmem.c"
#endif
