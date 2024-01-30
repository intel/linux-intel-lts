// SPDX-License-Identifier: MIT
/*
 * Copyright © 2019 Intel Corporation
 */

#include <linux/prandom.h>

#include <uapi/drm/i915_drm.h>

#include "gt/intel_gt_requests.h"
#include "gem/i915_gem_object.h"

#include "i915_buddy.h"
#include "intel_memory_region.h"
#include "i915_drv.h"
#include "i915_svm.h"

static const struct {
	u16 class;
	u16 instance;
} intel_region_map[] = {
	[INTEL_REGION_SMEM] = {
		.class = INTEL_MEMORY_SYSTEM,
		.instance = 0,
	},
	[INTEL_REGION_LMEM_0] = {
		.class = INTEL_MEMORY_LOCAL,
		.instance = 0,
	},
	[INTEL_REGION_LMEM_1] = {
		.class = INTEL_MEMORY_LOCAL,
		.instance = 1,
	},
	[INTEL_REGION_STOLEN] = {
		.class = INTEL_MEMORY_STOLEN,
		.instance = 0,
	},
};

static int __iopagetest(struct intel_memory_region *mem,
			u8 __iomem *va, int pagesize,
			u8 value, resource_size_t offset,
			const void *caller)
{
	int byte = prandom_u32_max(pagesize);
	u8 result[3];

	memset_io(va, value, pagesize); /* or GPF! */
	wmb();

	result[0] = ioread8(va);
	result[1] = ioread8(va + byte);
	result[2] = ioread8(va + pagesize - 1);
	if (memchr_inv(result, value, sizeof(result))) {
		dev_err(mem->i915->drm.dev,
			"Failed to read back from memory region:%pR at [%pa + %pa] for %ps; wrote %x, read (%x, %x, %x)\n",
			&mem->region, &mem->io_start, &offset, caller,
			value, result[0], result[1], result[2]);
		return -EINVAL;
	}

	return 0;
}

static int iopagetest(struct intel_memory_region *mem,
		      resource_size_t offset,
		      const void *caller)
{
	const u8 val[] = { 0x0, 0xa5, 0xc3, 0xf0 };
	void __iomem *va;
	int err;
	int i;

	va = ioremap_wc(mem->io_start + offset, PAGE_SIZE);
	if (!va) {
		dev_err(mem->i915->drm.dev,
			"Failed to ioremap memory region [%pa + %pa] for %ps\n",
			&mem->io_start, &offset, caller);
		return -EFAULT;
	}

	for (i = 0; i < ARRAY_SIZE(val); i++) {
		err = __iopagetest(mem, va, PAGE_SIZE, val[i], offset, caller);
		if (err)
			break;

		err = __iopagetest(mem, va, PAGE_SIZE, ~val[i], offset, caller);
		if (err)
			break;
	}

	iounmap(va);
	return err;
}

static resource_size_t random_page(resource_size_t last)
{
	/* Limited to low 44b (16TiB), but should suffice for a spot check */
	return prandom_u32_max(last >> PAGE_SHIFT) << PAGE_SHIFT;
}

static int iomemtest(struct intel_memory_region *mem,
		     bool test_all,
		     const void *caller)
{
	resource_size_t last, page;
	int err;

	if (mem->io_size < PAGE_SIZE)
		return 0;

	last = mem->io_size - PAGE_SIZE;

	/*
	 * Quick test to check read/write access to the iomap (backing store).
	 *
	 * Write a byte, read it back. If the iomapping fails, we expect
	 * a GPF preventing further execution. If the backing store does not
	 * exist, the read back will return garbage. We check a couple of pages,
	 * the first and last of the specified region to confirm the backing
	 * store + iomap does cover the entire memory region; and we check
	 * a random offset within as a quick spot check for bad memory.
	 */

	if (test_all) {
		for (page = 0; page <= last; page += PAGE_SIZE) {
			err = iopagetest(mem, page, caller);
			if (err)
				return err;
		}
	} else {
		err = iopagetest(mem, 0, caller);
		if (err)
			return err;

		err = iopagetest(mem, last, caller);
		if (err)
			return err;

		err = iopagetest(mem, random_page(last), caller);
		if (err)
			return err;
	}

	return 0;
}

struct intel_memory_region *
intel_memory_region_lookup(struct drm_i915_private *i915,
			   u16 class, u16 instance)
{
	struct intel_memory_region *mr;
	int id;

	/* XXX: consider maybe converting to an rb tree at some point */
	for_each_memory_region(mr, i915, id) {
		if (mr->type == class && mr->instance == instance)
			return mr;
	}

	return NULL;
}

static void
intel_memory_region_free_pages(struct intel_memory_region *mem,
			       struct list_head *blocks,
			       bool dirty)
{
	struct i915_buddy_block *block, *on;
	u64 avail = 0;

	list_for_each_entry_safe(block, on, blocks, link) {
		avail += i915_buddy_block_size(&mem->mm, block);
		if (dirty && __i915_buddy_block_is_clear(block))
			i915_buddy_block_set_clear(block, false);
		i915_buddy_mark_free(&mem->mm, block);
	}
	INIT_LIST_HEAD(blocks);

	atomic64_add(avail, &mem->avail);
}

void
__intel_memory_region_put_pages_buddy(struct intel_memory_region *mem,
				      struct list_head *blocks,
				      bool dirty)
{
	if (unlikely(list_empty(blocks)))
		return;

	intel_memory_region_free_pages(mem, blocks, dirty);
}

static void __intel_memory_region_put_block_work(struct work_struct *work)
{
	struct intel_memory_region *mem =
		container_of(work, struct intel_memory_region, pd_put.work);
	struct llist_node *freed = llist_del_all(&mem->pd_put.blocks);
	struct i915_buddy_block *block, *bn;
	struct list_head blocks;

	INIT_LIST_HEAD(&blocks);

	llist_for_each_entry_safe(block, bn, freed, freed)
		list_add(&block->link, &blocks);

	__intel_memory_region_put_pages_buddy(mem, &blocks, true);
}

void
__intel_memory_region_put_block_buddy(struct i915_buddy_block *block)
{
	struct intel_memory_region *mem = block->private;

	if (llist_add(&block->freed, &mem->pd_put.blocks))
		schedule_work(&mem->pd_put.work);
}

void intel_memory_region_print(struct intel_memory_region *mem,
			       resource_size_t target,
			       struct drm_printer *p)
{
	const struct {
		const char *name;
		struct list_head *list;
	} objects[] = {
		{ "purgeable", &mem->objects.purgeable },
		{ "evictable", &mem->objects.list },
		{}
	}, *o;
	int i;

	drm_printf(p, "memory region: %s\n", mem->name);
	drm_printf(p, "  parking: %s\n", str_yes_no(!completion_done(&mem->parking)));
	drm_printf(p, "  clear-on-free: %s\n", str_enabled_disabled(test_bit(INTEL_MEMORY_CLEAR_FREE, &mem->flags)));
	drm_printf(p, "  total:  %llu\n", mem->total);
	drm_printf(p, "  avail:  %llu\n", atomic64_read(&mem->avail));
	if (atomic64_read(&mem->evict))
		drm_printf(p, "  evict:  %llu\n", atomic64_read(&mem->evict));
	if (target)
		drm_printf(p, "  target: %llu\n", target);

	for (o = objects; o->name; o++) {
		resource_size_t active = 0, pinned = 0, avail = 0;
		struct drm_i915_gem_object *obj;
		unsigned long bookmark = 0;
		unsigned long empty = 0;
		unsigned long count = 0;

		spin_lock(&mem->objects.lock);
		list_for_each_entry(obj, o->list, mm.region.link) {
			if (!obj->mm.region.mem) {
				bookmark++;
				continue;
			}

			if (!i915_gem_object_has_pages(obj)) {
				empty++;
				continue;
			}

			if (i915_gem_object_is_active(obj))
				active += obj->base.size;
			else if (i915_gem_object_has_pinned_pages(obj))
				pinned += obj->base.size;
			else
				avail += obj->base.size;
			count++;
		}
		spin_unlock(&mem->objects.lock);
		if (!count)
			continue;

		drm_printf(p,
			   "  %s: { bookmark:%lu, empty:%lu, count:%lu, active:%llu, pinned:%llu, avail:%llu }\n",
			   o->name, bookmark, empty, count, active, pinned, avail);
	}

	if (mem->mm.size) {
		drm_printf(p, "  free:\n");
		for (i = 0; i <= mem->mm.max_order; i++) {
			const u64 sz = mem->mm.chunk_size << i;
			struct i915_buddy_block *block;
			struct i915_buddy_list *bl;
			u64 clear, dirty;
			long count = 0;

			dirty = 0;
			bl = &mem->mm.dirty_list[i];
			spin_lock(&bl->lock);
			list_for_each_entry(block, &bl->list, link) {
				if (!block->node.list)
					continue;

				dirty += sz;
				count++;
			}
			spin_unlock(&bl->lock);

			clear = 0;
			bl = &mem->mm.clear_list[i];
			spin_lock(&bl->lock);
			list_for_each_entry(block, &bl->list, link) {
				if (!block->node.list)
					continue;

				clear += sz;
				count++;
			}
			spin_unlock(&bl->lock);

			if (!count)
				continue;

			drm_printf(p, "  - [%d]: { count:%lu, clear:%llu, dirty:%llu }\n",
				   i, count, clear, dirty);
		}
	}
}

static bool i915_gem_object_allows_eviction(struct drm_i915_gem_object *obj)
{
	/* Only evict user lmem only objects if overcommit is enabled */
	if (!(obj->flags & I915_BO_ALLOC_USER))
		return true;

	if (obj->memory_mask & REGION_SMEM)
		return true;

	return i915_allows_overcommit(to_i915(obj->base.dev));
}

int intel_memory_region_evict(struct intel_memory_region *mem,
			      struct i915_gem_ww_ctx *ww,
			      resource_size_t target,
			      int chunk)
{
	struct list_head *phases[] = {
		/*
		 * Purgeable objects are deemed to be free by userspace
		 * and exist purely as a means to cache pages. They are
		 * a resource that we can reallocate from as userspace
		 * must revalidate the purgeable object prior to use,
		 * and be prepared to recover if the content was lost.
		 *
		 * It is always preferrable to reuse the purgeable objects
		 * as we we can immediately reallocate their pages without
		 * swapping them out to shmemfs, even to prefer waiting
		 * for those to complete prior to looking at inactive
		 * objects, as those inactive objects will need to be swapped
		 * out and so impose their own execution barrier, similar
		 * to waiting for work completion on the purgeable lists.
		 */
		&mem->objects.purgeable,
		&mem->objects.list,
		&mem->objects.purgeable,
		NULL,
	};
	struct intel_memory_region_link bookmark = {};
	struct intel_memory_region_link *pos;
	struct list_head **phase = phases;
	resource_size_t found = 0;
	LIST_HEAD(blocks);
	bool keepalive;
	long timeout;
	int err = 0;

	/*
	 * Eviction uses a per-object mutex to reclaim pages, so how do
	 * we ensure both deadlock avoidance and that all clients can
	 * take a turn at evicting every object? ww_mutex.
	 *
	 * The deadlock avoidance algorithm built into ww_mutex essentially
	 * provides each client with a ticket for the order in which
	 * they can reclaim objects from the memory region. (If there's
	 * a conflict between two clients, the lowest ticket holder has
	 * priority.) And by ensuring that we never lose track of objects
	 * that are known to the memory region, both objects that are in
	 * the process of being allocated and of being evicted, all clients
	 * that need eviction may scan all objects and in doing so provide
	 * a fair ordering across all current evictions and future allocations.
	 */
next:
	timeout = 0;
	keepalive = ww && *phase == &mem->objects.list;
	if (ww && phase > phases && !keepalive)
		timeout = msecs_to_jiffies(CONFIG_DRM_I915_FENCE_TIMEOUT);
	spin_lock(&mem->objects.lock);
	list_for_each_entry(pos, *phase, link) {
		struct drm_i915_gem_object *obj;

		if (unlikely(signal_pending(current))) {
			err = -EINTR;
			break;
		}

		if (unlikely(!pos->mem)) /* skip over other bookmarks */
			continue;

		/* only segment BOs should be in mem->objects.list */
		obj = container_of(pos, typeof(*obj), mm.region);
		GEM_BUG_ON(i915_gem_object_has_segments(obj));

		if (i915_gem_object_is_framebuffer(obj))
			continue;

		if (!i915_gem_object_allows_eviction(obj))
			continue;

		if (ww && ww == i915_gem_get_locking_ctx(obj)) {
			/* Repeat, waiting for the active objects */
			if (keepalive) {
				timeout = msecs_to_jiffies(CONFIG_DRM_I915_FENCE_TIMEOUT);
				keepalive = false;
			}
			continue;
		}

		list_add(&bookmark.link, &pos->link);
		if (keepalive) {
			list_move_tail(&pos->link, *phase);

			if (i915_gem_get_locking_ctx(obj))
				goto delete_bookmark;

			if (i915_gem_object_is_active(obj))
				goto delete_bookmark;
		}

		if (!i915_gem_object_get_rcu(obj)) {
			list_del_init(&pos->link);
			list_replace_init(&obj->mm.blocks, &blocks);
			found += obj->base.size;
			obj = NULL;
		}

		spin_unlock(&mem->objects.lock);

		if (!obj) {
			__intel_memory_region_put_pages_buddy(mem,
							      &blocks,
							      true);
			goto relock;
		}

		/* Flush activity prior to grabbing locks */
		timeout = __i915_gem_object_wait(obj,
						 I915_WAIT_INTERRUPTIBLE |
						 I915_WAIT_ALL,
						 timeout);
		if (timeout < 0) {
			timeout = 0;
			goto put;
		}

		err = __i915_gem_object_lock_to_evict(obj, ww);
		if (err)
			goto put;

		if (!i915_gem_object_has_pages(obj))
			goto unlock;

		if (i915_gem_object_is_active(obj))
			goto unlock;

		i915_gem_object_move_notify(obj);

		err = i915_gem_object_unbind(obj, ww, I915_GEM_OBJECT_UNBIND_KEEP_RESIDENT);
		if (err == 0)
			err = __i915_gem_object_put_pages(obj);
		if (err == 0) {
			/* conservative estimate of reclaimed pages */
			GEM_TRACE("%s:{ target:%pa, found:%pa, evicting:%zu, remaining timeout:%ld }\n",
				  mem->name, &target, &found, obj->base.size, timeout);
			found += obj->base.size;
		}

unlock:
		i915_gem_object_unlock(obj);
put:
		i915_gem_object_put(obj);
relock:
		cond_resched();
		spin_lock(&mem->objects.lock);
delete_bookmark:
		__list_del_entry(&bookmark.link);
		if (err == -EDEADLK)
			break;

		err = 0;
		if (found >= target)
			break;

		pos = &bookmark;
	}
	spin_unlock(&mem->objects.lock);
	if (err || found >= target)
		return err;

	/* And try to release all stale kernel objects */
	intel_gt_retire_requests(mem->gt);
	if (*++phase)
		goto next;

	/*
	 * Keep retrying the allocation until there is nothing more to evict.
	 *
	 * If we have made any forward progress towards completing our
	 * allocation; retry. On the next pass, especially if we are competing
	 * with other threads, we may find more to evict and succeed. It is
	 * not until there is nothing left to evict on this pass and make
	 * no forward progress, do we conclude that it is better to report
	 * failure.
	 */
	if (found || i915_buddy_defrag(&mem->mm, chunk, chunk))
		return 0;

	if (IS_ENABLED(CONFIG_DRM_I915_DEBUG)) {
		struct drm_printer p = drm_info_printer(mem->gt->i915->drm.dev);

		intel_memory_region_print(mem, target, &p);
	}

	return -ENXIO;
}

static unsigned int
__max_order(const struct intel_memory_region *mem, unsigned long n_pages)
{
	if (n_pages >> mem->mm.max_order)
		return mem->mm.max_order;

	return __fls(n_pages);
}

static bool
available_chunks(const struct intel_memory_region *mem, resource_size_t sz)
{
	/*
	 * Only allow this client to take from the pool of freed chunks
	 * only if there is enough memory available to satisfy all
	 * current allocation requests. If there is not enough memory
	 * available, we need to evict objects and we want to prevent
	 * the next allocation stealing our reclaimed chunk before we
	 * can use it for ourselves. So effectively when eviction
	 * has been started, every allocation goes through the eviction
	 * loop and will be ordered fairly by the ww_mutex, ensuring
	 * all clients continue to make forward progress.
	 */
	return atomic64_read(&mem->avail) >= sz;
}

int
__intel_memory_region_get_pages_buddy(struct intel_memory_region *mem,
				      struct i915_gem_ww_ctx *ww,
				      resource_size_t size,
				      unsigned int flags,
				      struct list_head *blocks)
{
	unsigned int min_order = 0;
	unsigned long n_pages;
	unsigned int order;
	int err = 0;

	GEM_BUG_ON(!IS_ALIGNED(size, mem->mm.chunk_size));

	GEM_BUG_ON((flags & (I915_ALLOC_CHUNK_4K |
			     I915_ALLOC_CHUNK_64K |
			     I915_ALLOC_CHUNK_2M |
			     I915_ALLOC_CHUNK_1G)) &&
		   (flags & I915_ALLOC_CHUNK_MIN_PAGE_SIZE));

	if (flags & I915_ALLOC_CHUNK_1G)
		min_order = ilog2(SZ_1G) - ilog2(mem->mm.chunk_size);
	else if (flags & I915_ALLOC_CHUNK_2M)
		min_order = ilog2(SZ_2M) - ilog2(mem->mm.chunk_size);
	else if (flags & I915_ALLOC_CHUNK_64K)
		min_order = ilog2(SZ_64K) - ilog2(mem->mm.chunk_size);
	else if (flags & I915_ALLOC_CHUNK_4K)
		min_order = ilog2(SZ_4K) - ilog2(mem->mm.chunk_size);
	else if (flags & I915_ALLOC_CHUNK_MIN_PAGE_SIZE)
		min_order = ilog2(mem->min_page_size) - ilog2(mem->mm.chunk_size);

	if (flags & I915_ALLOC_CONTIGUOUS) {
		size = roundup_pow_of_two(size);
		min_order = ilog2(size) - ilog2(mem->mm.chunk_size);
	}

	if (size > mem->mm.size)
		return -E2BIG;

	n_pages = size >> ilog2(mem->mm.chunk_size);
	order = __max_order(mem, n_pages);
	GEM_BUG_ON(order < min_order);

	/* Reserve the memory we reclaim for ourselves! */
	if (!available_chunks(mem, atomic64_add_return(size, &mem->evict)))
		goto evict;

	do {
		resource_size_t sz = mem->mm.chunk_size << order;
		struct i915_buddy_block *block;

		block = __i915_buddy_alloc(&mem->mm, order, flags);
		if (!IS_ERR(block)) {
			GEM_BUG_ON(i915_buddy_block_order(block) != order);
			GEM_BUG_ON(i915_buddy_block_is_free(block));
			atomic64_sub(sz, &mem->avail);
			list_add_tail(&block->link, blocks);
			block->private = mem;

			n_pages -= BIT(order);
			if (!n_pages) {
				atomic64_sub(size, &mem->evict);
				return 0;
			}

			while (!(n_pages >> order))
				order--;

			continue;
		}

		if (wait_for_completion_interruptible(&mem->parking)) {
			err = -EINTR;
			break;
		}

		if (i915_buddy_defrag(&mem->mm, min_order, order))
			/* Merged a few blocks, try again */
			continue;

		if (order-- == min_order) {
evict:			sz = n_pages * mem->mm.chunk_size;
			err = intel_memory_region_evict(mem, ww, sz, min_order);
			if (err)
				break;

			/* Make these chunks available for defrag */
			intel_memory_region_free_pages(mem, blocks, false);
			n_pages = size >> ilog2(mem->mm.chunk_size);
			order = __max_order(mem, n_pages);
		}

		if (signal_pending(current)) {
			err = -EINTR;
			break;
		}
	} while (1);

	intel_memory_region_free_pages(mem, blocks, false);
	atomic64_sub(size, &mem->evict);
	return err;
}

struct i915_buddy_block *
__intel_memory_region_get_block_buddy(struct intel_memory_region *mem,
				      resource_size_t size,
				      unsigned int flags)
{
	struct i915_buddy_block *block;
	LIST_HEAD(blocks);
	int ret;

	ret = __intel_memory_region_get_pages_buddy(mem, NULL, size, flags, &blocks);
	if (ret)
		return ERR_PTR(ret);

	block = list_first_entry(&blocks, typeof(*block), link);
	list_del_init(&block->link);
	return block;
}

int intel_memory_region_init_buddy(struct intel_memory_region *mem,
				   u64 start, u64 end, u64 chunk)
{
	return i915_buddy_init(&mem->mm, start, end, chunk);
}

void intel_memory_region_release_buddy(struct intel_memory_region *mem)
{
	i915_buddy_free_list(&mem->mm, &mem->reserved);
	i915_buddy_fini(&mem->mm);
}

int intel_memory_region_reserve(struct intel_memory_region *mem,
				u64 offset, u64 size)
{
	int ret;

	/* offset is relative to the region, but the buddy is absolute */
	ret = i915_buddy_alloc_range(&mem->mm, &mem->reserved,
				     mem->region.start + offset, size);
	if (!ret)
		atomic64_sub(size, &mem->avail);

	return ret;
}

struct intel_memory_region *
intel_memory_region_by_type(struct drm_i915_private *i915,
			    enum intel_memory_type mem_type)
{
	struct intel_memory_region *mr;
	int id;

	for_each_memory_region(mr, i915, id)
		if (mr->type == mem_type)
			return mr;

	return NULL;
}

static int intel_memory_region_memtest(struct intel_memory_region *mem,
				       void *caller)
{
	struct drm_i915_private *i915 = mem->i915;
	int err = 0;

	if (!mem->io_start)
		return 0;

	if (IS_ENABLED(CONFIG_DRM_I915_DEBUG_GEM) || i915->params.memtest)
		err = iomemtest(mem, i915->params.memtest, caller);

	return err;
}

struct intel_memory_region *
intel_memory_region_create(struct intel_gt *gt,
			   resource_size_t start,
			   resource_size_t size,
			   resource_size_t min_page_size,
			   resource_size_t io_start,
			   resource_size_t io_size,
			   u16 type,
			   u16 instance,
			   const struct intel_memory_region_ops *ops)
{
	struct intel_memory_region *mem;
	int err;

	mem = kzalloc(sizeof(*mem), GFP_KERNEL);
	if (!mem)
		return ERR_PTR(-ENOMEM);

	mem->gt = gt;
	mem->i915 = gt->i915;
	mem->region = (struct resource)DEFINE_RES_MEM(start, size);
	mem->io_start = io_start;
	mem->io_size = io_size;
	mem->min_page_size = min_page_size;
	mem->ops = ops;
	mem->total = size;
	atomic64_set(&mem->avail, size);
	mem->type = type;
	mem->instance = instance;

	INIT_WORK(&mem->pd_put.work, __intel_memory_region_put_block_work);

	spin_lock_init(&mem->objects.lock);
	INIT_LIST_HEAD(&mem->objects.list);
	INIT_LIST_HEAD(&mem->objects.purgeable);

	INIT_LIST_HEAD(&mem->reserved);

	init_completion(&mem->parking);
	complete_all(&mem->parking);

	spin_lock_init(&mem->acct_lock);

	if (ops->init) {
		err = ops->init(mem);
		if (err)
			goto err_free;
	}

	err = intel_memory_region_memtest(mem, (void *)_RET_IP_);
	if (err)
		goto err_release;

	kref_init(&mem->kref);
	return mem;

err_release:
	if (mem->ops->release)
		mem->ops->release(mem);
err_free:
	kfree(mem);
	return ERR_PTR(err);
}

void intel_memory_region_set_name(struct intel_memory_region *mem,
				  const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(mem->name, sizeof(mem->name), fmt, ap);
	va_end(ap);
}

static void __intel_memory_region_destroy(struct kref *kref)
{
	struct intel_memory_region *mem =
		container_of(kref, typeof(*mem), kref);

	/* Flush any pending work to free blocks region */
	flush_work(&mem->pd_put.work);

	if (mem->ops->release)
		mem->ops->release(mem);

	kfree(mem);
}

struct intel_memory_region *
intel_memory_region_get(struct intel_memory_region *mem)
{
	kref_get(&mem->kref);
	return mem;
}

void intel_memory_region_put(struct intel_memory_region *mem)
{
	kref_put(&mem->kref, __intel_memory_region_destroy);
}

/* Global memory region registration -- only slight layer inversions! */

int intel_memory_regions_hw_probe(struct drm_i915_private *i915)
{
	int err, i;

	/* All platforms currently have system memory */
	GEM_BUG_ON(!HAS_REGION(i915, REGION_SMEM));

	if (IS_DGFX(i915)) {
		if (IS_ENABLED(CONFIG_DRM_I915_PCIE_STRICT_WRITE_ORDERING))
			pcie_capability_clear_word(to_pci_dev(i915->drm.dev), PCI_EXP_DEVCTL,
						   PCI_EXP_DEVCTL_RELAX_EN);
		else
			pcie_capability_set_word(to_pci_dev(i915->drm.dev), PCI_EXP_DEVCTL,
						 PCI_EXP_DEVCTL_RELAX_EN);
	}

	for (i = 0; i < ARRAY_SIZE(i915->mm.regions); i++) {
		struct intel_memory_region *mem = ERR_PTR(-ENODEV);
		struct intel_gt *gt;
		u16 type, instance;

		if (!HAS_REGION(i915, BIT(i)))
			continue;

		type = intel_region_map[i].class;
		instance = intel_region_map[i].instance;
		gt = to_root_gt(i915);

		switch (type) {
		case INTEL_MEMORY_SYSTEM:
			mem = i915_gem_shmem_setup(gt, type, instance);
			break;
		case INTEL_MEMORY_STOLEN:
			mem = i915_gem_stolen_setup(gt, type, instance);
			if (!IS_ERR(mem))
				i915->mm.stolen_region = mem;
			break;
		default:
			continue;
		}

		if (IS_ERR(mem)) {
			drm_err(&i915->drm,
				"Failed to setup global region %d type=%d (%pe)\n", i, type, mem);
			continue;
		}

		GEM_BUG_ON(intel_region_map[i].instance);

		mem->id = i;
		mem->instance = 0;
		i915->mm.regions[i] = mem;
	}

	err = 0;
	if (!intel_memory_region_by_type(i915, INTEL_MEMORY_SYSTEM)) {
		drm_err(&i915->drm,
			"Failed to setup system memory, unable to continue\n");
		intel_memory_regions_driver_release(i915);
		err = -ENODEV;
	}

	return err;
}

int intel_memory_regions_resume_early(struct drm_i915_private *i915)
{
	int ret = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(i915->mm.regions); i++) {
		struct intel_memory_region *region;
		int err = 0;

		region = i915->mm.regions[i];
		if (!region)
			continue;

		err = intel_memory_region_memtest(region, (void *)_RET_IP_);
		if (err && !ret)
			ret = err;

		i915_buddy_discard_clears(&region->mm);
	}

	return ret;
}

void intel_memory_regions_driver_release(struct drm_i915_private *i915)
{
	int i;

	/* flush pending work that might use the memory regions */
	flush_workqueue(i915->wq);

	for (i = 0; i < ARRAY_SIZE(i915->mm.regions); i++) {
		struct intel_memory_region *region =
			fetch_and_zero(&i915->mm.regions[i]);

		if (region)
			intel_memory_region_put(region);
	}
}

int intel_memory_regions_add_svm(struct drm_i915_private  *i915)
{
	struct intel_memory_region *mr;
	enum intel_region_id id;
	int ret = 0;

	mutex_lock(&i915->svm_init_mutex);
	for_each_memory_region(mr, i915, id) {
		if (mr->type != INTEL_MEMORY_LOCAL || mr->devmem)
			continue;

		ret = i915_svm_devmem_add(mr);
		if (ret)
			break;
	}
	mutex_unlock(&i915->svm_init_mutex);
	return ret;
}

void intel_memory_regions_remove(struct drm_i915_private *i915)
{
	struct intel_memory_region *mr;
	enum intel_region_id id;

	for_each_memory_region(mr, i915, id)
		i915_svm_devmem_remove(mr);
}

#if IS_ENABLED(CONFIG_DRM_I915_SELFTEST)
#include "selftests/intel_memory_region.c"
#include "selftests/mock_region.c"
#endif
