/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2014-2016 Intel Corporation
 */

#include <drm/drm_cache.h>

#include "gt/intel_gt.h"
#include "gt/intel_tlb.h"

#include "i915_drv.h"
#include "i915_gem_object.h"
#include "i915_scatterlist.h"
#include "i915_gem_lmem.h"
#include "i915_gem_mman.h"

unsigned int i915_gem_sg_segment_size(const struct drm_i915_gem_object *obj)
{
	/*
	 * Internal device memory is not passed through dma-mapping, so
	 * we are only limited by the maximum page size.
	 */
	if (i915_gem_object_is_lmem(obj))
		return rounddown_pow_of_two(UINT_MAX);

	return rounddown_pow_of_two(i915_sg_segment_size());
}

void __i915_gem_object_set_pages(struct drm_i915_gem_object *obj,
				 struct sg_table *pages,
				 unsigned int sg_page_sizes)
{
	struct drm_i915_private *i915 = to_i915(obj->base.dev);
	struct intel_memory_region *mem;
	bool shrinkable;

	assert_object_held_shared(obj);

	if (i915_gem_object_is_volatile(obj))
		obj->mm.madv = I915_MADV_DONTNEED;

	/* Make the pages coherent with the GPU (flushing any swapin). */
	if (obj->cache_dirty) {
		obj->write_domain = 0;
		if (i915_gem_object_has_struct_page(obj))
			drm_clflush_sg(pages);
		obj->cache_dirty = false;
	}

	i915_gem_object_set_backing_store(obj);
	obj->mm.pages = pages;
	obj->mm.get_page.sg_pos = pages->sgl;
	obj->mm.get_dma_page.sg_pos = pages->sgl;

	GEM_BUG_ON(!sg_page_sizes);
	obj->mm.page_sizes = sg_page_sizes;

	shrinkable = i915_gem_object_is_shrinkable(obj);

	if (i915_gem_object_is_tiled(obj) &&
	    i915->gem_quirks & GEM_QUIRK_PIN_SWIZZLED_PAGES) {
		GEM_BUG_ON(i915_gem_object_has_tiling_quirk(obj));
		i915_gem_object_set_tiling_quirk(obj);
		GEM_BUG_ON(!list_empty(&obj->mm.link));
		atomic_inc(&obj->mm.shrink_pin);
		shrinkable = false;
	}

	if (shrinkable) {
		struct list_head *list;
		unsigned long flags;

		assert_object_held(obj);
		spin_lock_irqsave(&i915->mm.obj_lock, flags);

		i915->mm.shrink_count++;
		i915->mm.shrink_memory += obj->base.size;

		if (obj->mm.madv != I915_MADV_WILLNEED)
			list = &i915->mm.purge_list;
		else
			list = &i915->mm.shrink_list;
		list_add_tail(&obj->mm.link, list);

		atomic_set(&obj->mm.shrink_pin, 0);
		spin_unlock_irqrestore(&i915->mm.obj_lock, flags);
	}

	mem = obj->mm.region.mem;
	if (mem) {
		struct list_head *list;

		if (obj->mm.madv != I915_MADV_WILLNEED)
			list = &mem->objects.purgeable;
		else
			list = &mem->objects.list;

		spin_lock(&mem->objects.lock);
		list_move_tail(&obj->mm.region.link, list);
		spin_unlock(&mem->objects.lock);
	}
}

static void add_to_evictions(struct drm_i915_gem_object *obj)
{
	struct intel_memory_region *mem;

	mem = obj->mm.region.mem;
	if (!mem)
		return;

	GEM_BUG_ON(i915_gem_object_has_pages(obj));
	spin_lock(&mem->objects.lock);
	list_add_tail(&obj->mm.region.link, &mem->objects.list);
	spin_unlock(&mem->objects.lock);
}

static void remove_from_evictions(struct drm_i915_gem_object *obj)
{
	struct intel_memory_region *mem;

	mem = obj->mm.region.mem;
	if (!mem)
		return;

	GEM_BUG_ON(i915_gem_object_has_pages(obj));
	spin_lock(&mem->objects.lock);
	list_del_init(&obj->mm.region.link);
	spin_unlock(&mem->objects.lock);
}

int ____i915_gem_object_get_pages(struct drm_i915_gem_object *obj)
{
	int err;

	assert_object_held_shared(obj);

	if (unlikely(obj->mm.madv != I915_MADV_WILLNEED)) {
		drm_dbg(obj->base.dev,
			"Attempting to obtain a purgeable object\n");
		return -EFAULT;
	}

	add_to_evictions(obj);

	err = obj->ops->get_pages(obj);
	if (err)
		remove_from_evictions(obj);

	if (!IS_ENABLED(CONFIG_DRM_I915_CHICKEN_ASYNC_GET_PAGES) && err == 0)
		err = i915_gem_object_migrate_sync(obj);

	return err;
}

/* Ensure that the associated pages are gathered from the backing storage
 * and pinned into our object. i915_gem_object_pin_pages() may be called
 * multiple times before they are released by a single call to
 * i915_gem_object_unpin_pages() - once the pages are no longer referenced
 * either as a result of memory pressure (reaping pages under the shrinker)
 * or as the object is itself released.
 */
int __i915_gem_object_get_pages(struct drm_i915_gem_object *obj)
{
	int err;

	assert_object_held(obj);

	assert_object_held_shared(obj);

	if (unlikely(!i915_gem_object_has_pages(obj))) {
		GEM_BUG_ON(i915_gem_object_has_pinned_pages(obj));

		err = ____i915_gem_object_get_pages(obj);
		if (err)
			return err;

		smp_mb__before_atomic();
	}
	atomic_inc(&obj->mm.pages_pin_count);

	return 0;
}

int i915_gem_object_pin_pages_sync(struct drm_i915_gem_object *obj)
{
	int err;

	/* Hint that any fresh pages will be acquired synchronously */
	obj->flags |= I915_BO_SYNC_HINT;

	err = i915_gem_object_pin_pages(obj);
	if (err)
		return err;

	err = i915_gem_object_migrate_sync(obj);
	if (err)
		goto err;

	return 0;

err:
	i915_gem_object_unpin_pages(obj);
	return err;
}

int i915_gem_object_pin_pages_unlocked(struct drm_i915_gem_object *obj)
{
	struct i915_gem_ww_ctx ww;
	int err;

	i915_gem_ww_ctx_init(&ww, true);
retry:
	err = i915_gem_object_lock(obj, &ww);
	if (!err)
		err = i915_gem_object_pin_pages_sync(obj);

	if (err == -EDEADLK) {
		err = i915_gem_ww_ctx_backoff(&ww);
		if (!err)
			goto retry;
	}
	i915_gem_ww_ctx_fini(&ww);
	return err;
}

/* Immediately discard the backing storage */
void i915_gem_object_truncate(struct drm_i915_gem_object *obj)
{
	if (obj->ops->truncate)
		obj->ops->truncate(obj);

	obj->mm.madv = __I915_MADV_PURGED;
	obj->mm.pages = ERR_PTR(-EFAULT);
}

/* Try to discard unwanted pages */
void i915_gem_object_writeback(struct drm_i915_gem_object *obj)
{
	assert_object_held_shared(obj);
	GEM_BUG_ON(i915_gem_object_has_pages(obj));

	if (obj->ops->writeback)
		obj->ops->writeback(obj);
}

void __i915_gem_object_reset_page_iter(struct drm_i915_gem_object *obj,
				       struct sg_table *pages)
{
	struct radix_tree_iter iter;
	void __rcu **slot;

	rcu_read_lock();
	radix_tree_for_each_slot(slot, &obj->mm.get_page.radix, &iter, 0)
		radix_tree_delete(&obj->mm.get_page.radix, iter.index);
	radix_tree_for_each_slot(slot, &obj->mm.get_dma_page.radix, &iter, 0)
		radix_tree_delete(&obj->mm.get_dma_page.radix, iter.index);
	rcu_read_unlock();

	obj->mm.get_page.sg_pos = pages ? pages->sgl : NULL;
	obj->mm.get_page.sg_idx = 0;
	obj->mm.get_dma_page.sg_pos = pages ? pages->sgl : NULL;
	obj->mm.get_dma_page.sg_idx = 0;
}

static bool is_iomap_addr(struct drm_i915_gem_object *obj, void *ptr)
{
	struct intel_memory_region *mem;

	mem = obj->mm.region.mem;
	if (!mem)
		return false;

	return ptrdiff(ptr, mem->iomap.iomem) < mem->iomap.size;
}

static void unmap_object(struct drm_i915_gem_object *obj, void *ptr)
{
	if (is_iomap_addr(obj, ptr))
		return;

	if (is_vmalloc_addr(ptr))
		vunmap(ptr);
}

static void flush_tlb_invalidate(struct drm_i915_gem_object *obj)
{
	struct drm_i915_private *i915 = to_i915(obj->base.dev);
	struct intel_gt *gt;
	int id;

	for_each_gt(gt, i915, id) {
		if (!obj->mm.tlb[id])
			continue;

		intel_gt_invalidate_tlb_full(gt, obj->mm.tlb[id]);
		obj->mm.tlb[id] = 0;
	}
}

struct sg_table *
__i915_gem_object_unset_pages(struct drm_i915_gem_object *obj)
{
	struct sg_table *pages;

	assert_object_held_shared(obj);

	pages = fetch_and_zero(&obj->mm.pages);
	if (IS_ERR_OR_NULL(pages))
		return pages;

	if (i915_gem_object_is_volatile(obj))
		obj->mm.madv = I915_MADV_WILLNEED;

	i915_gem_object_make_unshrinkable(obj);

	if (!list_empty(&obj->mm.region.link)) {
		struct intel_memory_region *mem = obj->mm.region.mem;

		spin_lock(&mem->objects.lock);
		list_del_init(&obj->mm.region.link);
		spin_unlock(&mem->objects.lock);
	}

	if (obj->mm.mapping) {
		unmap_object(obj, page_mask_bits(obj->mm.mapping));
		obj->mm.mapping = NULL;
	}

	__i915_gem_object_reset_page_iter(obj, NULL);

	flush_tlb_invalidate(obj);

	return pages;
}

int __i915_gem_object_put_pages(struct drm_i915_gem_object *obj)
{
	struct sg_table *pages;
	int err;

	if (i915_gem_object_has_pinned_pages(obj))
		return -EBUSY;

	/*
	 * ->put_pages might need to allocate memory for the bit17 swizzle
	 * array, hence protect them from being reaped by removing them from gtt
	 * lists early.
	 */
	pages = __i915_gem_object_unset_pages(obj);
	if (IS_ERR_OR_NULL(pages))
		return 0;

	i915_gem_object_release_mmap_offset(obj);

	err = obj->ops->put_pages(obj, pages);
	if (err) {
		__i915_gem_object_set_pages(obj, pages, obj->mm.page_sizes);
		return err;
	}

	if (obj->mm.madv != I915_MADV_WILLNEED)
		i915_gem_object_truncate(obj);

	if (kref_read(&obj->base.refcount)) /* prune stale fences */
		dma_resv_add_excl_fence(obj->base.resv, NULL);

	return 0;
}

/* The 'mapping' part of i915_gem_object_pin_map() below */
static void *i915_gem_object_map_page(struct drm_i915_gem_object *obj,
				      enum i915_map_type type)
{
	unsigned long n_pages = obj->base.size >> PAGE_SHIFT, i;
	struct page *stack[32], **pages = stack, *page;
	struct sgt_iter iter;
	pgprot_t pgprot;
	void *vaddr;

	switch (type) {
	default:
		MISSING_CASE(type);
		fallthrough;	/* to use PAGE_KERNEL anyway */
	case I915_MAP_WB:
		/*
		 * On 32b, highmem using a finite set of indirect PTE (i.e.
		 * vmap) to provide virtual mappings of the high pages.
		 * As these are finite, map_new_virtual() must wait for some
		 * other kmap() to finish when it runs out. If we map a large
		 * number of objects, there is no method for it to tell us
		 * to release the mappings, and we deadlock.
		 *
		 * However, if we make an explicit vmap of the page, that
		 * uses a larger vmalloc arena, and also has the ability
		 * to tell us to release unwanted mappings. Most importantly,
		 * it will fail and propagate an error instead of waiting
		 * forever.
		 *
		 * So if the page is beyond the 32b boundary, make an explicit
		 * vmap.
		 */
		if (sg_is_last(obj->mm.pages->sgl) &&
		    !PageHighMem(sg_page(obj->mm.pages->sgl)))
			return page_address(sg_page(obj->mm.pages->sgl));

		pgprot = PAGE_KERNEL;
		break;
	case I915_MAP_WC:
		pgprot = pgprot_writecombine(PAGE_KERNEL_IO);
		break;
	}

	if (n_pages > ARRAY_SIZE(stack)) {
		/* Too big for stack -- allocate temporary array instead */
		pages = kvmalloc_array(n_pages, sizeof(*pages), GFP_KERNEL);
		if (!pages)
			return ERR_PTR(-ENOMEM);
	}

	i = 0;
	for_each_sgt_page(page, iter, obj->mm.pages)
		pages[i++] = page;
	vaddr = vmap(pages, n_pages, 0, pgprot);
	if (pages != stack)
		kvfree(pages);

	return vaddr ?: ERR_PTR(-ENOMEM);
}

static void *i915_gem_object_map_pfn(struct drm_i915_gem_object *obj,
				     enum i915_map_type type)
{
	struct intel_memory_region *mem = obj->mm.region.mem;
	resource_size_t iomap = mem->iomap.base - mem->region.start;
	unsigned long n_pfn = obj->base.size >> PAGE_SHIFT;
	unsigned long stack[32], *pfns = stack, i;
	struct sgt_iter iter;
	dma_addr_t addr;
	void *vaddr;

	if (type != I915_MAP_WC)
		return ERR_PTR(-ENODEV);

	/* A single contiguous block of lmem? Reuse the io_mapping */
	if (sg_is_last(obj->mm.pages->sgl))
		return (void __force *)i915_gem_object_lmem_io_map(obj, 0, obj->base.size);

	if (n_pfn > ARRAY_SIZE(stack)) {
		/* Too big for stack -- allocate temporary array instead */
		pfns = kvmalloc_array(n_pfn, sizeof(*pfns), GFP_KERNEL);
		if (!pfns)
			return ERR_PTR(-ENOMEM);
	}

	i = 0;
	for_each_sgt_daddr(addr, iter, obj->mm.pages)
		pfns[i++] = (iomap + addr) >> PAGE_SHIFT;
	vaddr = vmap_pfn(pfns, n_pfn, pgprot_writecombine(PAGE_KERNEL_IO));
	if (pfns != stack)
		kvfree(pfns);

	return vaddr ?: ERR_PTR(-ENOMEM);
}

/* get, pin, and map the pages of the object into kernel space */
void *i915_gem_object_pin_map(struct drm_i915_gem_object *obj,
			      enum i915_map_type type)
{
	enum i915_map_type has_type;
	bool pinned;
	void *ptr;
	int err;

	if (!i915_gem_object_has_struct_page(obj) &&
	    !i915_gem_object_type_has(obj, I915_GEM_OBJECT_HAS_IOMEM))
		return ERR_PTR(-ENXIO);

	assert_object_held(obj);

	pinned = !(type & I915_MAP_OVERRIDE);
	type &= ~I915_MAP_OVERRIDE;

	if (!atomic_inc_not_zero(&obj->mm.pages_pin_count)) {
		if (unlikely(!i915_gem_object_has_pages(obj))) {
			GEM_BUG_ON(i915_gem_object_has_pinned_pages(obj));

			obj->flags |= I915_BO_SYNC_HINT;

			err = ____i915_gem_object_get_pages(obj);
			if (err)
				return ERR_PTR(err);

			smp_mb__before_atomic();
		}
		atomic_inc(&obj->mm.pages_pin_count);
		pinned = false;
	}
	GEM_BUG_ON(!i915_gem_object_has_pages(obj));

	ptr = page_unpack_bits(obj->mm.mapping, &has_type);
	if (ptr && has_type != type) {
		if (pinned) {
			ptr = ERR_PTR(-EBUSY);
			goto err_unpin;
		}

		unmap_object(obj, ptr);

		ptr = obj->mm.mapping = NULL;
	}

	err = i915_gem_object_migrate_sync(obj);
	if (err) {
		ptr = ERR_PTR(err);
		goto err_unpin;
	}

	if (!ptr) {
		if (GEM_WARN_ON(type == I915_MAP_WC && !pat_enabled()))
			ptr = ERR_PTR(-ENODEV);
		else if (i915_gem_object_has_struct_page(obj))
			ptr = i915_gem_object_map_page(obj, type);
		else
			ptr = i915_gem_object_map_pfn(obj, type);
		if (IS_ERR(ptr))
			goto err_unpin;

		obj->mm.mapping = page_pack_bits(ptr, type);
	}

	GEM_BUG_ON(i915_gem_object_has_migrate(obj));
	GEM_BUG_ON(!i915_gem_object_mem_idle(obj));
	return ptr;

err_unpin:
	atomic_dec(&obj->mm.pages_pin_count);
	GEM_BUG_ON(!IS_ERR(ptr));
	return ptr;
}

void *i915_gem_object_pin_map_unlocked(struct drm_i915_gem_object *obj,
				       enum i915_map_type type)
{
	struct i915_gem_ww_ctx ww;
	void *ret = NULL;
	int err;

	for_i915_gem_ww(&ww, err, false) {
		err = i915_gem_object_lock(obj, &ww);
		if (err)
			continue;

		ret = i915_gem_object_pin_map(obj, type);
		if (IS_ERR(ret))
			err = PTR_ERR(ret);
		/* Implicit unlock */
	}
	if (err)
		return ERR_PTR(err);

	return ret;
}

void __i915_gem_object_flush_map(struct drm_i915_gem_object *obj,
				 unsigned long offset,
				 unsigned long size)
{
	enum i915_map_type has_type;
	void *ptr;

	GEM_BUG_ON(!i915_gem_object_has_pinned_pages(obj));
	GEM_BUG_ON(range_overflows_t(typeof(obj->base.size),
				     offset, size, obj->base.size));

	wmb(); /* let all previous writes be visible to coherent partners */

	if (obj->cache_coherent & I915_BO_CACHE_COHERENT_FOR_WRITE)
		return;

	ptr = page_unpack_bits(obj->mm.mapping, &has_type);
	if (has_type == I915_MAP_WC)
		return;

	drm_clflush_virt_range(ptr + offset, size);
	if (size == obj->base.size) {
		obj->write_domain &= ~I915_GEM_DOMAIN_CPU;
		obj->cache_dirty = false;
	}
}

void __i915_gem_object_release_map(struct drm_i915_gem_object *obj)
{
	GEM_BUG_ON(!obj->mm.mapping);

	/*
	 * We allow removing the mapping from underneath pinned pages!
	 *
	 * Furthermore, since this is an unsafe operation reserved only
	 * for construction time manipulation, we ignore locking prudence.
	 */
	unmap_object(obj, page_mask_bits(fetch_and_zero(&obj->mm.mapping)));

	i915_gem_object_unpin_map(obj);
}

struct scatterlist *
(__i915_gem_object_get_sg)(struct drm_i915_gem_object *obj,
			 struct i915_gem_object_page_iter *iter,
			 pgoff_t n,
			 unsigned int *offset)
{
	const bool dma = iter == &obj->mm.get_dma_page;
	unsigned int idx, count;
	struct scatterlist *sg;

	might_sleep_if(n);
	GEM_BUG_ON(n >= obj->base.size >> PAGE_SHIFT);
	if (!i915_gem_object_has_pinned_pages(obj))
		assert_object_held(obj);

	/* Skip the search and caching for the base address */
	sg = obj->mm.pages->sgl;
	if (likely(n == 0 ||
		   n <  (dma ? __sg_dma_page_count(sg) : __sg_page_count(sg)))) {
		*offset = n;
		return sg;
	}

	/* As we iterate forward through the sg, we record each entry in a
	 * radixtree for quick repeated (backwards) lookups. If we have seen
	 * this index previously, we will have an entry for it.
	 *
	 * Initial lookup is O(N), but this is amortized to O(1) for
	 * sequential page access (where each new request is consecutive
	 * to the previous one). Repeated lookups are O(lg(obj->base.size)),
	 * i.e. O(1) with a large constant!
	 */
	if (n < READ_ONCE(iter->sg_idx))
		goto lookup;

	mutex_lock(&iter->lock);

	/* We prefer to reuse the last sg so that repeated lookup of this
	 * (or the subsequent) sg are fast - comparing against the last
	 * sg is faster than going through the radixtree.
	 */

	sg = iter->sg_pos;
	idx = iter->sg_idx;
	count = dma ? __sg_dma_page_count(sg) : __sg_page_count(sg);

	while (idx + count <= n) {
		void *entry;
		unsigned long i;
		int ret;

		/* If we cannot allocate and insert this entry, or the
		 * individual pages from this range, cancel updating the
		 * sg_idx so that on this lookup we are forced to linearly
		 * scan onwards, but on future lookups we will try the
		 * insertion again (in which case we need to be careful of
		 * the error return reporting that we have already inserted
		 * this index).
		 */
		ret = radix_tree_insert(&iter->radix, idx, sg);
		if (ret && ret != -EEXIST)
			goto scan;

		entry = xa_mk_value(idx);
		for (i = 1; i < count; i++) {
			ret = radix_tree_insert(&iter->radix, idx + i, entry);
			if (ret && ret != -EEXIST)
				goto scan;
		}

		idx += count;
		sg = ____sg_next(sg);
		count = dma ? __sg_dma_page_count(sg) : __sg_page_count(sg);
	}

scan:
	iter->sg_pos = sg;
	iter->sg_idx = idx;

	mutex_unlock(&iter->lock);

	if (unlikely(n < idx)) /* insertion completed by another thread */
		goto lookup;

	/* In case we failed to insert the entry into the radixtree, we need
	 * to look beyond the current sg.
	 */
	while (idx + count <= n) {
		idx += count;
		sg = ____sg_next(sg);
		count = dma ? __sg_dma_page_count(sg) : __sg_page_count(sg);
	}

	*offset = n - idx;
	return sg;

lookup:
	rcu_read_lock();

	sg = radix_tree_lookup(&iter->radix, n);
	GEM_BUG_ON(!sg);

	/* If this index is in the middle of multi-page sg entry,
	 * the radix tree will contain a value entry that points
	 * to the start of that range. We will return the pointer to
	 * the base page and the offset of this page within the
	 * sg entry's range.
	 */
	*offset = 0;
	if (unlikely(xa_is_value(sg))) {
		unsigned long base = xa_to_value(sg);

		sg = radix_tree_lookup(&iter->radix, base);
		GEM_BUG_ON(!sg);

		*offset = n - base;
	}

	rcu_read_unlock();

	return sg;
}

struct page *
(i915_gem_object_get_page)(struct drm_i915_gem_object *obj, pgoff_t n)
{
	struct scatterlist *sg;
	unsigned int offset;

	GEM_BUG_ON(!i915_gem_object_has_struct_page(obj));

	sg = i915_gem_object_get_sg(obj, n, &offset);
	return nth_page(sg_page(sg), offset);
}

dma_addr_t
(i915_gem_object_get_dma_address_len)(struct drm_i915_gem_object *obj,
				      pgoff_t n, unsigned int *len)
{
	struct scatterlist *sg;
	unsigned int offset;

	sg = i915_gem_object_get_sg_dma(obj, n, &offset);

	if (len)
		*len = sg_dma_len(sg) - (offset << PAGE_SHIFT);

	return sg_dma_address(sg) + (offset << PAGE_SHIFT);
}

dma_addr_t
(i915_gem_object_get_dma_address)(struct drm_i915_gem_object *obj, pgoff_t n)
{
	return i915_gem_object_get_dma_address_len(obj, n, NULL);
}
