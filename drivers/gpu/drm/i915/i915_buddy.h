/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2021 Intel Corporation
 */

#ifndef __I915_BUDDY_H__
#define __I915_BUDDY_H__

#include <linux/bitops.h>
#include <linux/list.h>
#include <linux/llist.h>
#include <linux/slab.h>

#include "i915_active.h"

/* 512 bits (one per pages) supports 2MB blocks */
#define I915_BUDDY_MAX_PAGES   512

struct i915_buddy_block {
#define I915_BUDDY_OFFSET	 	GENMASK_ULL(63, 12)
#define I915_BUDDY_CLEAR_BIT		11
/* Free to be used, if needed in the future */
#define I915_BUDDY_HEADER_UNUSED	GENMASK_ULL(9, 6)
#define I915_BUDDY_ORDER	 	GENMASK_ULL(5, 0)
	u64 header;

	struct i915_buddy_block *left;
	struct i915_buddy_block *right;
	struct i915_buddy_block *parent;

	void *private; /* owned by creator */

	struct i915_active_fence active;

	/*
	 * While the block is allocated by the user through
	 * i915_buddy_alloc*, the user has ownership of the link, for
	 * example to maintain within a list, if so desired. As soon as
	 * the block is freed with i915_buddy_free* ownership is given
	 * back to the mm.
	 */
	union {
		struct i915_buddy_link {
			struct list_head link;
			struct i915_buddy_list *list;
		} node;
		struct list_head link;

		/*
		 * XXX: consider moving this somewhere specific to the pd
		 * stuff. In an ideal world we would like to keep i915_buddy as
		 * non-i915 specific as possible and in this case the delayed
		 * freeing is only required for our pd handling, which is only
		 * one part of our overall i915_buddy use.
		 */
		struct llist_node freed;
	};

	unsigned long pfn_first;
	/*
	 * FIXME: There are other alternatives to bitmap. Like splitting the
	 * block into contiguous 4K sized blocks. But it is part of bigger
	 * issues involving partially invalidating large mapping, freeing the
	 * blocks etc., revisit.
	 */
	unsigned long bitmap[BITS_TO_LONGS(I915_BUDDY_MAX_PAGES)];
};

/* Order-zero must be at least PAGE_SIZE */
#define I915_BUDDY_MAX_ORDER (63 - PAGE_SHIFT)

struct i915_buddy_list {
	struct list_head list;
	spinlock_t lock;
	bool defrag;
};

/*
 * Binary Buddy System.
 */
struct i915_buddy_mm {
	/* Maintain a free list for each order. */
	struct i915_buddy_list *clear_list;
	struct i915_buddy_list *dirty_list;

	/*
	 * Maintain explicit binary tree(s) to track the allocation of the
	 * address space. This gives us a simple way of finding a buddy block
	 * and performing the potentially recursive merge step when freeing a
	 * block.  Nodes are either allocated or free, in which case they will
	 * also exist on the respective free list.
	 */
	struct i915_buddy_block **roots;

	/*
	 * Anything from here is public, and remains static for the lifetime of
	 * the mm. Everything above is considered do-not-touch.
	 */
	unsigned int n_roots;
	unsigned int max_order;

	/* Must be at least PAGE_SIZE */
	u64 chunk_size;
	u64 size;
};

static inline u64
i915_buddy_block_offset(const struct i915_buddy_block *block)
{
	return READ_ONCE(block->header) & I915_BUDDY_OFFSET;
}

static inline unsigned int
i915_buddy_block_order(const struct i915_buddy_block *block)
{
	return READ_ONCE(block->header) & I915_BUDDY_ORDER;
}

static inline const unsigned long *
__i915_buddy_header_read(const struct i915_buddy_block *block)
{
	return (const unsigned long *)&block->header;
}

static inline unsigned long *
__i915_buddy_header_state(struct i915_buddy_block *block)
{
	return (unsigned long *)&block->header;
}

static inline bool
i915_buddy_block_is_free(const struct i915_buddy_block *block)
{
	return READ_ONCE(block->node.list);
}

static inline bool
i915_buddy_block_is_split(const struct i915_buddy_block *block)
{
	return block->left;
}

static inline bool
__i915_buddy_block_is_clear(const struct i915_buddy_block *block)
{
	return test_bit(I915_BUDDY_CLEAR_BIT, __i915_buddy_header_read(block));
}

static inline bool
i915_buddy_block_is_clear(const struct i915_buddy_block *block)
{
	if (!__i915_buddy_block_is_clear(block))
		return false;

	/* Did the last operation to clear this block fail? */
	return !i915_active_fence_has_error(&block->active);
}

static inline void
__i915_buddy_block_set_clear(struct i915_buddy_block *block)
{
	__set_bit(I915_BUDDY_CLEAR_BIT, __i915_buddy_header_state(block));
}

static inline void
__i915_buddy_block_clr_clear(struct i915_buddy_block *block)
{
	__clear_bit(I915_BUDDY_CLEAR_BIT, __i915_buddy_header_state(block));
}

static inline void
i915_buddy_block_set_clear(struct i915_buddy_block *block, bool state)
{
	if (state)
		__i915_buddy_block_set_clear(block);
	else
		__i915_buddy_block_clr_clear(block);
}

static inline u64
i915_buddy_block_size(const struct i915_buddy_mm *mm,
		      const struct i915_buddy_block *block)
{
	return mm->chunk_size << i915_buddy_block_order(block);
}

static inline bool
i915_buddy_block_is_active(const struct i915_buddy_block *block)
{
       return i915_active_fence_isset(&block->active);
}

int i915_buddy_init(struct i915_buddy_mm *mm, u64 start, u64 end, u64 chunk);

void i915_buddy_discard_clears(struct i915_buddy_mm *mm);
void i915_buddy_fini(struct i915_buddy_mm *mm);

enum {
	I915_BUDDY_ALLOC_CLEAR_BIT = 0,
	I915_BUDDY_ALLOC_ACTIVE_BIT,
	__I915_BUDDY_ALLOC_USER_BITS
};

struct i915_buddy_block *
__i915_buddy_alloc(struct i915_buddy_mm *mm, unsigned int order, unsigned int flags);
#define I915_BUDDY_ALLOC_WANT_CLEAR BIT(I915_BUDDY_ALLOC_CLEAR_BIT)
#define I915_BUDDY_ALLOC_ALLOW_ACTIVE BIT(I915_BUDDY_ALLOC_ACTIVE_BIT)

static inline struct i915_buddy_block *
i915_buddy_alloc(struct i915_buddy_mm *mm, unsigned int order)
{
	return __i915_buddy_alloc(mm, order, 0);
}

int i915_buddy_alloc_range(struct i915_buddy_mm *mm,
			   struct list_head *blocks,
			   u64 start, u64 size);

void i915_buddy_mark_free(struct i915_buddy_mm *mm,
			  struct i915_buddy_block *block);

void i915_buddy_free(struct i915_buddy_mm *mm, struct i915_buddy_block *block);
void i915_buddy_free_list(struct i915_buddy_mm *mm, struct list_head *objects);

bool i915_buddy_defrag(struct i915_buddy_mm *mm,
		       unsigned int min_order,
		       unsigned int max_order);

void i915_buddy_module_exit(void);
int i915_buddy_module_init(void);

#endif
