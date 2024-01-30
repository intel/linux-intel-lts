// SPDX-License-Identifier: MIT
/*
 * Copyright © 2021 Intel Corporation
 */

#include <linux/kmemleak.h>

#include "i915_buddy.h"

#include "i915_gem.h"
#include "i915_utils.h"

static struct kmem_cache *slab_blocks;

static struct i915_buddy_block *
i915_block_alloc(struct i915_buddy_mm *mm,
		 struct i915_buddy_block *parent,
		 u64 hdr)
{
	struct i915_buddy_block *block;

	block = kmem_cache_zalloc(slab_blocks, GFP_KERNEL);
	if (!block)
		return NULL;

	block->header = hdr;
	block->parent = parent;

	INIT_ACTIVE_FENCE(&block->active);

	GEM_BUG_ON(block->header & I915_BUDDY_HEADER_UNUSED);
	return block;
}

static void i915_block_free(struct i915_buddy_block *block)
{
	i915_active_fence_fini(&block->active);
	kmem_cache_free(slab_blocks, block);
}

static void __clear_free(struct i915_buddy_block *block)
{
	GEM_BUG_ON(!block->node.list);
	lockdep_assert_held(&block->node.list->lock);

	WRITE_ONCE(block->node.list, NULL);
	list_del(&block->node.link);
}

static bool
clear_free(struct i915_buddy_block *block, struct i915_buddy_list *bl)
{
	bool ret = false;

	spin_lock(&bl->lock);
	if (likely(READ_ONCE(block->node.list) == bl)) {
		__clear_free(block);
		ret = true;
	}
	spin_unlock(&bl->lock);

	return ret;
}

static struct i915_buddy_block *
__get_buddy(const struct i915_buddy_block *block,
	    const struct i915_buddy_block *parent)
{
	return parent->left == block ? parent->right : parent->left;
}

static bool needs_defrag(const struct i915_buddy_block *block,
			 const struct i915_buddy_list *bl)
{
	return __get_buddy(block, block->parent)->node.list == bl;
}

static void __mark_free(struct i915_buddy_list *bl,
			struct i915_buddy_block *block)
{
	struct list_head *head;

	spin_lock(&bl->lock);
	GEM_BUG_ON(block->node.list);
	head = &bl->list;
	if (i915_buddy_block_is_active(block)) {
		if (!bl->defrag && block->parent && needs_defrag(block, bl))
			WRITE_ONCE(bl->defrag, true);

		head = head->prev;
	}
	list_add(&block->node.link, head);
	WRITE_ONCE(block->node.list, bl);
	spin_unlock(&bl->lock);
}

void i915_buddy_mark_free(struct i915_buddy_mm *mm,
			  struct i915_buddy_block *block)
{
	struct i915_buddy_list *bl =
		&(i915_buddy_block_is_clear(block) ? mm->clear_list : mm->dirty_list)[i915_buddy_block_order(block)];

	GEM_BUG_ON(i915_buddy_block_order(block) > mm->max_order);

	__mark_free(bl, block);
}

static struct i915_buddy_list *create_lists(int count)
{
	struct i915_buddy_list *lists;
	int i;

	lists = kmalloc_array(count, sizeof(*lists), GFP_KERNEL);
	if (!lists)
		return NULL;

	for (i = 0; i < count; ++i) {
		INIT_LIST_HEAD(&lists[i].list);
		spin_lock_init(&lists[i].lock);
		lists[i].defrag = false;
	}

	return lists;
}

int i915_buddy_init(struct i915_buddy_mm *mm, u64 start, u64 end, u64 chunk)
{
	struct i915_buddy_block **roots;
	unsigned int i, max_order;
	u64 offset, size;

	if (GEM_WARN_ON(range_overflows(start, chunk, end)))
		return -EINVAL;

	if (chunk < PAGE_SIZE || !is_power_of_2(chunk))
		return -EINVAL;

	/*
	 * We want the addresses we return to be naturally aligned, i.e.
	 *
	 *     IS_ALIGNED(block->offset, block->size).
	 *
	 * This is important when we use large chunks (e.g. 1G) and
	 * require the physical address to also be aligned to the chunk,
	 * e.g. huge page support in ppGTT.
	 */
	offset = round_up(start, chunk);
	size = round_down(end, chunk);
	if (size <= offset)
		return -EINVAL;

	size -= offset;

	mm->size = size;
	mm->chunk_size = chunk;

	max_order = ilog2(size) - ilog2(chunk);
	GEM_BUG_ON(max_order > I915_BUDDY_MAX_ORDER);
	mm->max_order = 0;

	mm->dirty_list = create_lists(max_order + 1);
	if (!mm->dirty_list)
		return -ENOMEM;

	mm->clear_list = create_lists(max_order + 1);
	if (!mm->clear_list)
		goto out_free_dirty;

	roots = kmalloc_array(2 * max_order + 1, sizeof(*roots), GFP_KERNEL);
	if (!roots)
		goto out_free_clear;

	/*
	 * Split into power-of-two blocks, in case we are given a size that is
	 * not itself a power-of-two, or a base address that is not naturally
	 * aligned.
	 */
	i = 0;
	do {
		struct i915_buddy_block *root;
		unsigned long order;

		order = ilog2(size);
		if (offset)
			order = min(order, __ffs64(offset));
		GEM_BUG_ON(order < ilog2(chunk));
		GEM_BUG_ON(order > ilog2(chunk) + max_order);

		root = i915_block_alloc(mm, NULL, offset | (order - ilog2(chunk)));
		if (!root)
			goto out_free_roots;

		GEM_BUG_ON(i915_buddy_block_size(mm, root) < chunk);
		GEM_BUG_ON(i915_buddy_block_size(mm, root) > size);

		if (order > mm->max_order)
			mm->max_order = order;

		i915_buddy_mark_free(mm, root);
		roots[i++] = root;
		GEM_BUG_ON(i > 2 * max_order + 1);

		offset += BIT_ULL(order);
		size -= BIT_ULL(order);
	} while (size);

	mm->roots = krealloc(roots, i * sizeof(*roots), GFP_KERNEL);
	if (!mm->roots) /* Can't reduce our allocation, keep it all! */
		mm->roots = roots;
	mm->n_roots = i;

	GEM_BUG_ON(mm->max_order < ilog2(chunk));
	mm->max_order -= ilog2(chunk);

	return 0;

out_free_roots:
	while (i--)
		i915_block_free(roots[i]);
	kfree(roots);
out_free_clear:
	kfree(mm->clear_list);
out_free_dirty:
	kfree(mm->dirty_list);
	return -ENOMEM;
}

static struct i915_buddy_block *
split_block(struct i915_buddy_mm *mm, struct i915_buddy_block *block)
{
	struct dma_fence *f;
	u64 hdr = block->header - 1;

	GEM_BUG_ON(!i915_buddy_block_order(block));
	GEM_BUG_ON(block->node.list);
	GEM_BUG_ON(hdr & (mm->chunk_size << (hdr & I915_BUDDY_ORDER)));

	block->left = i915_block_alloc(mm, block, hdr);
	if (!block->left)
		return ERR_PTR(-ENOMEM);

	hdr |= mm->chunk_size << (hdr & I915_BUDDY_ORDER);
	block->right = i915_block_alloc(mm, block, hdr);
	if (!block->right) {
		i915_block_free(block->left);
		return ERR_PTR(-ENOMEM);
	}

	f = i915_active_fence_get(&block->active);
	if (f) {
		__i915_active_fence_set(&block->left->active, f);
		__i915_active_fence_set(&block->right->active, f);
		dma_fence_put(f);
	}

	i915_buddy_mark_free(mm, block->right);

	return block->left;
}

static const struct dma_fence *block_fence(const struct i915_buddy_block *block)
{
	struct dma_fence *f = rcu_access_pointer(block->active.fence);

	return IS_ERR(f) ? NULL : f;
}

static bool
share_fence(const struct i915_buddy_block *a, const struct i915_buddy_block *b)
{
	return block_fence(a) == block_fence(b);
}

static unsigned int
__i915_buddy_free(struct i915_buddy_mm *mm,
		  struct i915_buddy_block *block,
		  bool dirty)
{
	const bool clear = !dirty && i915_buddy_block_is_clear(block);
	struct i915_buddy_block *parent;
	int order;

	/*
	 * __i915_buddy_free() processes both a block that is being freed
	 * and needs to be placed onto a freelist, and also during
	 * i915_buddy_defrag() to process already freed blocks to see if
	 * we can then merge (previously active) chunks.
	 */

	while ((parent = block->parent)) { /* combine freed buddies */
		struct i915_buddy_block *buddy = __get_buddy(block, parent);
		struct i915_buddy_list *bl;

		GEM_BUG_ON(block->node.list);

		bl = READ_ONCE(buddy->node.list);
		if (!bl)
			break;

		if (i915_buddy_block_is_clear(buddy) != clear)
			break;

		/* Beware fragmentation! */
		if (!share_fence(block, buddy)) {
			WRITE_ONCE(bl->defrag, true);
			break;
		}

		if (!clear_free(buddy, bl))
			break;

		__i915_active_fence_replace(&block->active, &parent->active);
		__i915_active_fence_replace(&buddy->active, &parent->active);

		i915_block_free(block);
		i915_block_free(buddy);

		block = parent;
	}
	GEM_BUG_ON(block->node.list);
	order = i915_buddy_block_order(block);

	if (clear != __i915_buddy_block_is_clear(block))
		i915_buddy_block_set_clear(block, clear);

	i915_buddy_mark_free(mm, block);
	return order;
}

void i915_buddy_free(struct i915_buddy_mm *mm,
		     struct i915_buddy_block *block)
{
	__i915_buddy_free(mm, block, true);
}

static void __i915_buddy_free_list(struct i915_buddy_mm *mm,
				   struct list_head *objects,
				   bool dirty)
{
	struct i915_buddy_block *block, *on;

	list_for_each_entry_safe(block, on, objects, link) {
		__i915_buddy_free(mm, block, dirty);
		cond_resched();
	}
	INIT_LIST_HEAD(objects);
}

void i915_buddy_free_list(struct i915_buddy_mm *mm, struct list_head *objects)
{
	__i915_buddy_free_list(mm, objects, true);
}

bool i915_buddy_defrag(struct i915_buddy_mm *mm,
		       unsigned int min_order,
		       unsigned int max_order)
{
	struct i915_buddy_link bookmark = {};
	struct i915_buddy_block *block;
	bool merged = false;
	unsigned int i;

	/* Any blocks that could be split into max_order? */
	for (i = max_order; i <= mm->max_order; i++) {
		if (!list_empty(&mm->clear_list[i].list) ||
		    !list_empty(&mm->dirty_list[i].list))
			return true;
	}

	/* Any blocks that could be merged to form max_order? */
	i = min(max_order, mm->max_order + 1);
	while (i-- && !merged) {
		struct i915_buddy_list *lists[] = {
			&mm->clear_list[i],
			&mm->dirty_list[i],
			NULL,
		}, **it, *list;

		for (it = lists; (list = *it); it++) {
			if (!fetch_and_zero(&list->defrag) && i >= min_order)
				continue;

			spin_lock(&list->lock);
			list_for_each_entry(block, &list->list, node.link) {
				if (unlikely(!block->node.list))
					continue;

				if (unlikely(!block->parent))
					continue;

				if (i915_buddy_block_is_active(block))
					break;

				GEM_BUG_ON(block->node.list != list);
				list_add(&bookmark.link, &block->node.link);
				__clear_free(block);
				spin_unlock(&list->lock);

				merged |= __i915_buddy_free(mm, block, i < min_order) >= max_order;

				spin_lock(&list->lock);
				if (unlikely(list_empty(&bookmark.link)))
					break; /* iteration was interrupted */

				__list_del_entry(&bookmark.link);
				block = container_of(&bookmark, typeof(*block), node);
			}
			spin_unlock(&list->lock);
		}
	}

	return merged;
}

static struct i915_buddy_block *buddy_list_first(struct list_head *head)
{
	struct i915_buddy_link *pos;

	list_for_each_entry(pos, head, link)
		if (pos->list) /* skip over bookmarks */
			return container_of(pos, struct i915_buddy_block, node);

	return NULL;
}

static struct i915_buddy_block *
__mark_used(const struct i915_buddy_mm *mm, struct i915_buddy_block *block)
{
	GEM_BUG_ON(block->node.list);
	kmemleak_update_trace(block);
	return block;
}

/*
 * Allocate power-of-two block. The order value here translates to:
 *
 *   0 = 2^0 * mm->chunk_size
 *   1 = 2^1 * mm->chunk_size
 *   2 = 2^2 * mm->chunk_size
 *   ...
 */
struct i915_buddy_block *
__i915_buddy_alloc(struct i915_buddy_mm *mm, unsigned int order, unsigned int flags)
{
	struct i915_buddy_list *lists[2] = { mm->dirty_list, mm->clear_list };
	struct i915_buddy_block *block;
	unsigned int n;

	if (flags & I915_BUDDY_ALLOC_WANT_CLEAR)
		swap(lists[0], lists[1]);

	/*
	 * Scan for the first idle chunk larger enough for the request,
	 * preferring to only return a cleared chunk to those requiring
	 * zeroed memory (i.e. user allocations).
	 */
	for (n = 0; n < ARRAY_SIZE(lists); n++) {
		struct i915_buddy_block *active = NULL;
		unsigned int i;

		for (i = order; i <= mm->max_order; ++i) {
			struct i915_buddy_list *bl = &lists[n][i];

			if (list_empty(&bl->list))
				continue;

			spin_lock(&bl->lock);
			block = buddy_list_first(&bl->list);
			if (block) {
				GEM_BUG_ON(block->node.list != bl);

				if (!i915_buddy_block_is_active(block) ||
				    flags & I915_BUDDY_ALLOC_ALLOW_ACTIVE) {
					__clear_free(block);
					spin_unlock(&bl->lock);
					if (active)
						i915_buddy_mark_free(mm, active);
					goto found;
				}

				if (!active) {
					__clear_free(block);
					active = block;
				}
			}
			spin_unlock(&bl->lock);
		}

		if (active) {
			block = active;
			goto found;
		}
	}
	return ERR_PTR(-ENOSPC);

found:
	GEM_BUG_ON(i915_buddy_block_order(block) < order);
	do {
		struct i915_buddy_block *low;

		if (i915_buddy_block_order(block) == order)
			return __mark_used(mm, block);

		low = split_block(mm, block);
		if (IS_ERR(low)) {
			i915_buddy_mark_free(mm, block);
			return low;
		}

		GEM_BUG_ON(i915_buddy_block_order(low) != i915_buddy_block_order(block) - 1);
		GEM_BUG_ON(i915_buddy_block_offset(low) != i915_buddy_block_offset(block));
		block = low;
	} while (1);
}

static inline bool overlaps(u64 s1, u64 e1, u64 s2, u64 e2)
{
	return s1 <= e2 && e1 >= s2;
}

static inline bool contains(u64 s1, u64 e1, u64 s2, u64 e2)
{
	return s1 <= s2 && e1 >= e2;
}

static struct i915_buddy_block *next_dfs_block(struct list_head *list)
{
	struct i915_buddy_block *block;

	block = list_first_entry_or_null(list, typeof(*block), link);
	if (!block)
		return NULL;

	GEM_BUG_ON(block->node.list);
	list_del(&block->link);
	return block;
}

static int add_dfs_block(struct i915_buddy_mm *mm,
			 struct i915_buddy_block *block,
			 u64 start, u64 end,
			 struct list_head *dfs)
{
	u64 block_start = i915_buddy_block_offset(block);
	u64 block_end = block_start + i915_buddy_block_size(mm, block) - 1;
	struct i915_buddy_list *bl;

	if (!overlaps(start, end, block_start, block_end))
		return 0;

	if (!i915_buddy_block_is_split(block)) {
		bl = READ_ONCE(block->node.list);
		if (!bl || !clear_free(block, bl))
			return -ENOSPC;
	}

	GEM_BUG_ON(block->node.list);
	list_add(&block->link, dfs);
	return 0;
}

static int alloc_range(struct i915_buddy_mm *mm,
		       struct i915_buddy_block *block,
		       struct list_head *out,
		       u64 start, u64 end)
{
	LIST_HEAD(dfs);
	int err;

	err = add_dfs_block(mm, block, start, end, &dfs);
	if (err)
		return err;

	while ((block = next_dfs_block(&dfs))) {
		u64 b_start = i915_buddy_block_offset(block);
		u64 b_end = b_start + i915_buddy_block_size(mm, block) - 1;

		if (contains(start, end, b_start, b_end)) {
			list_add_tail(&__mark_used(mm, block)->link, out);
			continue;
		}

		if (!i915_buddy_block_is_split(block)) {
			struct i915_buddy_block *low;

			low = split_block(mm, block);
			if (IS_ERR(low)) {
				i915_buddy_mark_free(mm, block);
				return PTR_ERR(low);
			}
			i915_buddy_mark_free(mm, low);
		}

		err = add_dfs_block(mm, block->right, start, end, &dfs);
		if (err)
			return err;

		err = add_dfs_block(mm, block->left, start, end, &dfs);
		if (err)
			return err;
	}

	return 0;
}

/*
 * Allocate range. Note that it's safe to chain together multiple alloc_ranges
 * with the same blocks list.
 *
 * Intended for pre-allocating portions of the address space, for example to
 * reserve a block for the initial framebuffer or similar, hence the expectation
 * here is that i915_buddy_alloc() is still the main vehicle for
 * allocations, so if that's not the case then the drm_mm range allocator is
 * probably a much better fit, and so you should probably go use that instead.
 */
int i915_buddy_alloc_range(struct i915_buddy_mm *mm,
			   struct list_head *blocks,
			   u64 start, u64 size)
{
	LIST_HEAD(allocated);
	u64 end;
	int err;
	int i;

	if (GEM_WARN_ON(start + size <= start))
		return -EINVAL;

	end = start + size;
	start = round_down(start, mm->chunk_size);
	end = round_up(end, mm->chunk_size);
	end -= 1; /* inclusive bounds testing */

	for (i = 0; i < mm->n_roots; ++i) {
		err = alloc_range(mm, mm->roots[i], &allocated, start, end);
		if (err)
			goto err_free;
	}

	list_splice_tail(&allocated, blocks);
	return 0;

err_free:
	__i915_buddy_free_list(mm, &allocated, false);
	return err;
}

void i915_buddy_discard_clears(struct i915_buddy_mm *mm)
{
	if (!mm->size)
		return;

	/* Recombine all split blocks */
	i915_buddy_defrag(mm, UINT_MAX, UINT_MAX);
}

void i915_buddy_fini(struct i915_buddy_mm *mm)
{
	int i;

	i915_buddy_discard_clears(mm);

	for (i = 0; i < mm->n_roots; ++i)
		i915_block_free(mm->roots[i]);

	kfree(mm->roots);
	kfree(mm->clear_list);
	kfree(mm->dirty_list);
}

#if IS_ENABLED(CONFIG_DRM_I915_SELFTEST)
#include "selftests/i915_buddy.c"
#endif

void i915_buddy_module_exit(void)
{
	kmem_cache_destroy(slab_blocks);
}

int __init i915_buddy_module_init(void)
{
	slab_blocks = KMEM_CACHE(i915_buddy_block, 0);
	if (!slab_blocks)
		return -ENOMEM;

	return 0;
}
