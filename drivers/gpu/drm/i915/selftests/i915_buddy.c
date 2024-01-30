// SPDX-License-Identifier: MIT
/*
 * Copyright © 2019 Intel Corporation
 */

#include <linux/prime_numbers.h>
#include <linux/sizes.h>
#include <linux/string_helpers.h>

#include "../i915_selftest.h"
#include "i915_random.h"

static struct i915_buddy_block *
get_buddy(struct i915_buddy_block *block)
{
	return block->parent ? __get_buddy(block, block->parent) : NULL;
}

static void __igt_dump_block(struct i915_buddy_mm *mm,
			     struct i915_buddy_block *block,
			     bool buddy)
{
	pr_err("block info: header=%llx, size=%llx root=%s buddy=%s\n",
	       block->header,
	       i915_buddy_block_size(mm, block),
	       str_yes_no(!block->parent),
	       str_yes_no(buddy));
}

static void igt_dump_block(struct i915_buddy_mm *mm,
			   struct i915_buddy_block *block)
{
	struct i915_buddy_block *buddy;

	__igt_dump_block(mm, block, false);

	buddy = get_buddy(block);
	if (buddy)
		__igt_dump_block(mm, buddy, true);
}

static int igt_check_block(struct i915_buddy_mm *mm,
			   struct i915_buddy_block *block)
{
	struct i915_buddy_block *buddy;
	u64 block_size;
	u64 offset;
	int err = 0;

	block_size = i915_buddy_block_size(mm, block);
	offset = i915_buddy_block_offset(block);

	if (block_size < mm->chunk_size) {
		pr_err("block size smaller than min size\n");
		err = -EINVAL;
	}

	if (!is_power_of_2(block_size)) {
		pr_err("block size not power of two\n");
		err = -EINVAL;
	}

	if (!IS_ALIGNED(block_size, mm->chunk_size)) {
		pr_err("block size not aligned to min size\n");
		err = -EINVAL;
	}

	if (!IS_ALIGNED(offset, mm->chunk_size)) {
		pr_err("block offset not aligned to min size\n");
		err = -EINVAL;
	}

	if (!IS_ALIGNED(offset, block_size)) {
		pr_err("block offset not aligned to block size\n");
		err = -EINVAL;
	}

	buddy = get_buddy(block);

	if (!buddy && block->parent) {
		pr_err("buddy has gone fishing\n");
		err = -EINVAL;
	}

	if (buddy) {
		if (i915_buddy_block_offset(buddy) != (offset ^ block_size)) {
			pr_err("buddy has wrong offset\n");
			err = -EINVAL;
		}

		if (i915_buddy_block_size(mm, buddy) != block_size) {
			pr_err("buddy size mismatch\n");
			err = -EINVAL;
		}
	}

	return err;
}

static int igt_check_blocks(struct i915_buddy_mm *mm,
			    struct list_head *blocks,
			    u64 expected_size,
			    bool is_contiguous)
{
	struct i915_buddy_block *block;
	struct i915_buddy_block *prev;
	u64 total;
	int err = 0;

	block = NULL;
	prev = NULL;
	total = 0;

	list_for_each_entry(block, blocks, link) {
		err = igt_check_block(mm, block);

		if (i915_buddy_block_is_free(block)) {
			pr_err("block is still on a freelist\n"),
			err = -EINVAL;
		}

		if (is_contiguous && prev) {
			u64 prev_block_size;
			u64 prev_offset;
			u64 offset;

			prev_offset = i915_buddy_block_offset(prev);
			prev_block_size = i915_buddy_block_size(mm, prev);
			offset = i915_buddy_block_offset(block);

			if (offset != (prev_offset + prev_block_size)) {
				pr_err("block offset mismatch\n");
				err = -EINVAL;
			}
		}

		if (err)
			break;

		total += i915_buddy_block_size(mm, block);
		prev = block;
	}

	if (!err) {
		if (total != expected_size) {
			pr_err("size mismatch, expected=%llx, found=%llx\n",
			       expected_size, total);
			err = -EINVAL;
		}
		return err;
	}

	if (prev) {
		pr_err("prev block, dump:\n");
		igt_dump_block(mm, prev);
	}

	if (block) {
		pr_err("bad block, dump:\n");
		igt_dump_block(mm, block);
	}

	return err;
}

static int igt_check_mm(struct i915_buddy_mm *mm)
{
	struct i915_buddy_block *root;
	struct i915_buddy_block *prev;
	unsigned int i;
	u64 total;
	int err = 0;

	if (!mm->n_roots) {
		pr_err("n_roots is zero\n");
		return -EINVAL;
	}

	if (mm->n_roots != hweight64(mm->size)) {
		pr_err("n_roots mismatch, n_roots=%u, expected=%lu\n",
		       mm->n_roots, hweight64(mm->size));
		return -EINVAL;
	}

	root = NULL;
	prev = NULL;
	total = 0;

	for (i = 0; i < mm->n_roots; ++i) {
		struct i915_buddy_block *block;
		unsigned int order;

		root = mm->roots[i];
		if (!root) {
			pr_err("root(%u) is NULL\n", i);
			err = -EINVAL;
			break;
		}

		err = igt_check_block(mm, root);

		if (!i915_buddy_block_is_free(root)) {
			pr_err("root not free\n");
			err = -EINVAL;
		}

		order = i915_buddy_block_order(root);

		if (!i) {
			if (order != mm->max_order) {
				pr_err("max order root missing\n");
				err = -EINVAL;
			}
		}

		if (prev) {
			u64 prev_block_size;
			u64 prev_offset;
			u64 offset;

			prev_offset = i915_buddy_block_offset(prev);
			prev_block_size = i915_buddy_block_size(mm, prev);
			offset = i915_buddy_block_offset(root);

			if (offset != (prev_offset + prev_block_size)) {
				pr_err("root offset mismatch\n");
				err = -EINVAL;
			}
		}

		block = list_first_entry_or_null(&mm->dirty_list[order].list,
						 struct i915_buddy_block,
						 node.link);
		if (block != root) {
			pr_err("root mismatch at order=%u\n", order);
			err = -EINVAL;
		}

		if (err)
			break;

		prev = root;
		total += i915_buddy_block_size(mm, root);
	}

	if (!err) {
		if (total != mm->size) {
			pr_err("expected mm size=%llx, found=%llx\n", mm->size,
			       total);
			err = -EINVAL;
		}
		return err;
	}

	if (prev) {
		pr_err("prev root(%u), dump:\n", i - 1);
		igt_dump_block(mm, prev);
	}

	if (root) {
		pr_err("bad root(%u), dump:\n", i);
		igt_dump_block(mm, root);
	}

	return err;
}

static void igt_mm_config(u64 *size, u64 *chunk_size)
{
	I915_RND_STATE(prng);
	u32 s, ms;

	/* Nothing fancy, just try to get an interesting bit pattern */

	prandom_seed_state(&prng, i915_selftest.random_seed);

	/* Let size be a random number of pages up to 8 GB (2M pages) */
	s = 1 + i915_prandom_u32_max_state((BIT(33 - 12)) - 1, &prng);
	/* Let the chunk size be a random power of 2 less than size */
	ms = BIT(i915_prandom_u32_max_state(ilog2(s), &prng));
	/* Round size down to the chunk size */
	s &= -ms;

	/* Convert from pages to bytes */
	*chunk_size = (u64)ms << 12;
	*size = (u64)s << 12;
}

static int igt_buddy_alloc_smoke(void *arg)
{
	struct i915_buddy_mm mm;
	IGT_TIMEOUT(end_time);
	I915_RND_STATE(prng);
	u64 chunk_size;
	u64 mm_size;
	int *order;
	int err, i;

	igt_mm_config(&mm_size, &chunk_size);

	pr_info("buddy_init with size=%llx, chunk_size=%llx\n", mm_size, chunk_size);

	err = i915_buddy_init(&mm, 0, mm_size, chunk_size);
	if (err) {
		pr_err("buddy_init failed(%d)\n", err);
		return err;
	}

	order = i915_random_order(mm.max_order + 1, &prng);
	if (!order)
		goto out_fini;

	for (i = 0; i <= mm.max_order; ++i) {
		struct i915_buddy_block *block;
		int max_order = order[i];
		bool timeout = false;
		LIST_HEAD(blocks);
		int order;
		u64 total;

		err = igt_check_mm(&mm);
		if (err) {
			pr_err("pre-mm check failed, abort\n");
			break;
		}

		pr_info("filling from max_order=%u\n", max_order);

		order = max_order;
		total = 0;

		do {
retry:
			block = i915_buddy_alloc(&mm, order);
			if (IS_ERR(block)) {
				err = PTR_ERR(block);
				if (err == -ENOMEM) {
					pr_info("buddy_alloc hit -ENOMEM with order=%d\n",
						order);
				} else {
					if (order--) {
						err = 0;
						goto retry;
					}

					pr_err("buddy_alloc with order=%d failed(%d)\n",
					       order, err);
				}

				break;
			}

			list_add_tail(&block->link, &blocks);

			if (i915_buddy_block_order(block) != order) {
				pr_err("buddy_alloc order mismatch\n");
				err = -EINVAL;
				break;
			}

			total += i915_buddy_block_size(&mm, block);

			if (__igt_timeout(end_time, NULL)) {
				timeout = true;
				break;
			}
		} while (total < mm.size);

		if (!err)
			err = igt_check_blocks(&mm, &blocks, total, false);

		i915_buddy_free_list(&mm, &blocks);

		if (!err) {
			err = igt_check_mm(&mm);
			if (err)
				pr_err("post-mm check failed\n");
		}

		if (err || timeout)
			break;

		cond_resched();
	}

	if (err == -ENOMEM)
		err = 0;

	kfree(order);
out_fini:
	i915_buddy_fini(&mm);

	return err;
}

static int igt_buddy_alloc_alignment(void *arg)
{
	struct i915_buddy_block *block;
	struct i915_buddy_mm mm;
	unsigned int start, size, chunk;
	unsigned int order;
	LIST_HEAD(blocks);
	I915_RND_STATE(prng);
	int err;

	/*
	 * Request a buddy for a misaligned base, and we expect blocks
	 * to be naturally aligned.
	 */

	start = prandom_u32_state(&prng);
	size = PAGE_SHIFT + i915_prandom_u32_max_state(63 - PAGE_SHIFT - 2, &prng) + 2;
	chunk = PAGE_SHIFT + i915_prandom_u32_max_state(size - PAGE_SHIFT - 2, &prng);
	pr_debug("%s: using start:%u, size:%d, chunk:%d\n",
		 __func__, start, size, chunk);

	err = i915_buddy_init(&mm,
			      start, start + BIT_ULL(size), BIT_ULL(chunk));
	if (err)
		return err;

	for (order = mm.max_order; order; order--) {
		block = i915_buddy_alloc(&mm, order);
		if (block == ERR_PTR(-ENOSPC)) { /* full! */
			i915_buddy_free_list(&mm, &blocks);
			continue;
		}
		if (IS_ERR(block)) {
			pr_info("buddy_alloc hit -ENOMEM with order:%d, max_order:%d, pass:%d\n",
				order, mm.max_order, 0);
			err = PTR_ERR(block);
			goto err;
		}
		list_add_tail(&block->link, &blocks);

		if (!IS_ALIGNED(i915_buddy_block_offset(block),
				i915_buddy_block_size(&mm, block))) {
			pr_err("pass:%d, order:%d block->size:%llx, offset:%llx is not aligned\n",
			       0, order,
			       i915_buddy_block_size(&mm, block),
			       i915_buddy_block_offset(block));
			err = -EINVAL;
			goto err;
		}
	}

	for (; order <= mm.max_order; order++) {
		block = i915_buddy_alloc(&mm, order);
		if (block == ERR_PTR(-ENOSPC)) { /* full! */
			i915_buddy_free_list(&mm, &blocks);
			continue;
		}
		if (IS_ERR(block)) {
			pr_info("buddy_alloc hit -ENOMEM with order:%d, pass:%d\n",
				order, 1);
			err = PTR_ERR(block);
			goto err;
		}
		list_add_tail(&block->link, &blocks);

		if (!IS_ALIGNED(i915_buddy_block_offset(block),
				i915_buddy_block_size(&mm, block))) {
			pr_err("pass:%d, order:%d block->size:%llx, offset:%llx is not aligned\n",
			       1, order,
			       i915_buddy_block_size(&mm, block),
			       i915_buddy_block_offset(block));
			err = -EINVAL;
			goto err;
		}
	}

err:
	i915_buddy_free_list(&mm, &blocks);
	i915_buddy_fini(&mm);
	return err;
}

static int igt_buddy_alloc_pessimistic(void *arg)
{
	const unsigned int max_order = 16;
	struct i915_buddy_block *block, *bn;
	struct i915_buddy_mm mm;
	unsigned int order;
	LIST_HEAD(blocks);
	int err;

	/*
	 * Create a pot-sized mm, then allocate one of each possible
	 * order within. This should leave the mm with exactly one
	 * page left.
	 */

	err = i915_buddy_init(&mm, 0, PAGE_SIZE << max_order, PAGE_SIZE);
	if (err) {
		pr_err("buddy_init failed(%d)\n", err);
		return err;
	}
	GEM_BUG_ON(mm.max_order != max_order);

	for (order = 0; order < max_order; order++) {
		block = i915_buddy_alloc(&mm, order);
		if (IS_ERR(block)) {
			pr_info("buddy_alloc hit -ENOMEM with order=%d\n",
				order);
			err = PTR_ERR(block);
			goto err;
		}

		list_add_tail(&block->link, &blocks);
	}

	/* And now the last remaining block available */
	block = i915_buddy_alloc(&mm, 0);
	if (IS_ERR(block)) {
		pr_info("buddy_alloc hit -ENOMEM on final alloc\n");
		err = PTR_ERR(block);
		goto err;
	}
	list_add_tail(&block->link, &blocks);

	/* Should be completely full! */
	for (order = max_order; order--; ) {
		block = i915_buddy_alloc(&mm, order);
		if (!IS_ERR(block)) {
			pr_info("buddy_alloc unexpectedly succeeded at order %d, it should be full!",
				order);
			list_add_tail(&block->link, &blocks);
			err = -EINVAL;
			goto err;
		}
	}

	block = list_last_entry(&blocks, typeof(*block), link);
	list_del(&block->link);
	i915_buddy_free(&mm, block);

	/* As we free in increasing size, we make available larger blocks */
	order = 1;
	list_for_each_entry_safe(block, bn, &blocks, link) {
		list_del(&block->link);
		i915_buddy_free(&mm, block);

		block = i915_buddy_alloc(&mm, order);
		if (IS_ERR(block)) {
			pr_info("buddy_alloc (realloc) hit -ENOMEM with order=%d\n",
				order);
			err = PTR_ERR(block);
			goto err;
		}
		i915_buddy_free(&mm, block);
		order++;
	}

	/* To confirm, now the whole mm should be available */
	block = i915_buddy_alloc(&mm, max_order);
	if (IS_ERR(block)) {
		pr_info("buddy_alloc (realloc) hit -ENOMEM with order=%d\n",
			max_order);
		err = PTR_ERR(block);
		goto err;
	}
	i915_buddy_free(&mm, block);

err:
	i915_buddy_free_list(&mm, &blocks);
	i915_buddy_fini(&mm);
	return err;
}

static int igt_buddy_alloc_optimistic(void *arg)
{
	const int max_order = 16;
	struct i915_buddy_block *block;
	struct i915_buddy_mm mm;
	LIST_HEAD(blocks);
	int order;
	int err;

	/*
	 * Create a mm with one block of each order available, and
	 * try to allocate them all.
	 */

	err = i915_buddy_init(&mm,
			      0, PAGE_SIZE * ((1 << (max_order + 1)) - 1),
			      PAGE_SIZE);
	if (err) {
		pr_err("buddy_init failed(%d)\n", err);
		return err;
	}
	GEM_BUG_ON(mm.max_order != max_order);

	for (order = 0; order <= max_order; order++) {
		block = i915_buddy_alloc(&mm, order);
		if (IS_ERR(block)) {
			pr_info("buddy_alloc hit -ENOMEM with order=%d\n",
				order);
			err = PTR_ERR(block);
			goto err;
		}

		list_add_tail(&block->link, &blocks);
	}

	/* Should be completely full! */
	block = i915_buddy_alloc(&mm, 0);
	if (!IS_ERR(block)) {
		pr_info("buddy_alloc unexpectedly succeeded, it should be full!");
		list_add_tail(&block->link, &blocks);
		err = -EINVAL;
		goto err;
	}

err:
	i915_buddy_free_list(&mm, &blocks);
	i915_buddy_fini(&mm);
	return err;
}

static int igt_buddy_alloc_pathological(void *arg)
{
	const int max_order = 16;
	struct i915_buddy_block *block;
	struct i915_buddy_mm mm;
	LIST_HEAD(blocks);
	LIST_HEAD(holes);
	int order, top;
	int err;

	/*
	 * Create a pot-sized mm, then allocate one of each possible
	 * order within. This should leave the mm with exactly one
	 * page left. Free the largest block, then whittle down again.
	 * Eventually we will have a fully 50% fragmented mm.
	 */

	err = i915_buddy_init(&mm, 0, PAGE_SIZE << max_order, PAGE_SIZE);
	if (err) {
		pr_err("buddy_init failed(%d)\n", err);
		return err;
	}
	GEM_BUG_ON(mm.max_order != max_order);

	for (top = max_order; top; top--) {
		/* Make room by freeing the largest allocated block */
		block = list_first_entry_or_null(&blocks, typeof(*block), link);
		if (block) {
			list_del(&block->link);
			i915_buddy_free(&mm, block);
		}

		for (order = top; order--; ) {
			block = i915_buddy_alloc(&mm, order);
			if (IS_ERR(block)) {
				pr_info("buddy_alloc hit -ENOMEM with order=%d, top=%d\n",
					order, top);
				err = PTR_ERR(block);
				goto err;
			}
			list_add_tail(&block->link, &blocks);
		}

		/* There should be one final page for this sub-allocation */
		block = i915_buddy_alloc(&mm, 0);
		if (IS_ERR(block)) {
			pr_info("buddy_alloc hit -ENOMEM for hole\n");
			err = PTR_ERR(block);
			goto err;
		}
		list_add_tail(&block->link, &holes);

		block = i915_buddy_alloc(&mm, top);
		if (!IS_ERR(block)) {
			pr_info("buddy_alloc unexpectedly succeeded at top-order %d/%d, it should be full!",
				top, max_order);
			list_add_tail(&block->link, &blocks);
			err = -EINVAL;
			goto err;
		}
	}

	i915_buddy_free_list(&mm, &holes);

	/* Nothing larger than blocks of chunk_size now available */
	for (order = 1; order <= max_order; order++) {
		block = i915_buddy_alloc(&mm, order);
		if (!IS_ERR(block)) {
			pr_info("buddy_alloc unexpectedly succeeded at order %d, it should be full!",
				order);
			list_add_tail(&block->link, &blocks);
			err = -EINVAL;
			goto err;
		}
	}

err:
	list_splice_tail(&holes, &blocks);
	i915_buddy_free_list(&mm, &blocks);
	i915_buddy_fini(&mm);
	return err;
}

static int igt_buddy_alloc_range(void *arg)
{
	struct i915_buddy_mm mm;
	unsigned long page_num;
	LIST_HEAD(blocks);
	u64 chunk_size;
	u64 offset;
	u64 size;
	u64 rem;
	int err;

	igt_mm_config(&size, &chunk_size);

	pr_info("buddy_init with size=%llx, chunk_size=%llx\n", size, chunk_size);

	err = i915_buddy_init(&mm, 0, size, chunk_size);
	if (err) {
		pr_err("buddy_init failed(%d)\n", err);
		return err;
	}

	err = igt_check_mm(&mm);
	if (err) {
		pr_err("pre-mm check failed, abort, abort, abort!\n");
		goto err_fini;
	}

	rem = mm.size;
	offset = 0;

	for_each_prime_number_from(page_num, 1, ULONG_MAX - 1) {
		struct i915_buddy_block *block;
		LIST_HEAD(tmp);

		size = min(page_num * mm.chunk_size, rem);

		err = i915_buddy_alloc_range(&mm, &tmp, offset, size);
		if (err) {
			if (err == -ENOMEM) {
				pr_info("alloc_range hit -ENOMEM with size=%llx\n",
					size);
			} else {
				pr_err("alloc_range with offset=%llx, size=%llx failed(%d)\n",
				       offset, size, err);
			}

			break;
		}

		block = list_first_entry_or_null(&tmp,
						 struct i915_buddy_block,
						 link);
		if (!block) {
			pr_err("alloc_range has no blocks\n");
			err = -EINVAL;
			break;
		}

		if (i915_buddy_block_offset(block) != offset) {
			pr_err("alloc_range start offset mismatch, found=%llx, expected=%llx\n",
			       i915_buddy_block_offset(block), offset);
			err = -EINVAL;
		}

		if (!err)
			err = igt_check_blocks(&mm, &tmp, size, true);

		list_splice_tail(&tmp, &blocks);

		if (err)
			break;

		offset += size;

		rem -= size;
		if (!rem)
			break;

		cond_resched();
	}

	if (err == -ENOMEM)
		err = 0;

	i915_buddy_free_list(&mm, &blocks);

	if (!err) {
		err = igt_check_mm(&mm);
		if (err)
			pr_err("post-mm check failed\n");
	}

err_fini:
	i915_buddy_fini(&mm);

	return err;
}

static int igt_buddy_alloc_range_small_misaligned(void *arg)
{
	static const struct { u16 start, size; } ranges[] = {
		{ 0, 1 },
		{ 0, 4095 },
		{ 0, 4096 },
		{ 0, 4097 },

		{ 1, 1 },
		{ 1, 4094 },
		{ 1, 4095 },
		{ 1, 4096 },

		{ 4094, 1 },
		{ 4094, 2 },
		{ 4094, 3 },

		{ 4095, 1 },
		{ 4095, 2 },
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(ranges); i++) {
		struct i915_buddy_block *block;
		struct i915_buddy_mm mm;
		LIST_HEAD(blocks);
		int err;

		err = i915_buddy_init(&mm, 0, 4096, 4096);
		if (err)
			return err;

		/* We expect to round to the block and so reserve the whole mm */
		err = i915_buddy_alloc_range(&mm, &blocks,
					     ranges[i].start, ranges[i].size);
		if (err) {
			pr_err("min buddy_alloc_range(%d, %d) failed(%d)\n",
			       err, ranges[i].start, ranges[i].size);
			goto out;
		}

		err = -EINVAL;

		if (!list_is_singular(&blocks)) {
			pr_err("More than one block returned from a singular buddy!\n");
			goto out;
		}

		block = list_first_entry(&blocks, struct i915_buddy_block, link);

		if (i915_buddy_block_offset(block) != 0) {
			pr_err("Reservation was not extended to 0\n");
			goto out;
		}

		if (i915_buddy_block_size(&mm, block) != 4096) {
			pr_err("Reservation was not extended to 4096\n");
			goto out;
		}

		err = 0;
out:
		i915_buddy_free_list(&mm, &blocks);
		i915_buddy_fini(&mm);
		if (err)
			return err;
	}

	return 0;
}

static int igt_buddy_alloc_range_misaligned(void *arg)
{
	u64 start, size, chunk_size;
	struct i915_buddy_mm mm;
	LIST_HEAD(blocks);
	u64 offset;
	int err;

	igt_mm_config(&size, &chunk_size);

	pr_info("buddy_init with offset=%llx, size=%llx, chunk_size=%llx\n", size / 2, size, chunk_size);

	start = size / 2;
	err = i915_buddy_init(&mm, start, start + size, chunk_size);
	if (err) {
		pr_err("buddy_init failed(%d)\n", err);
		return err;
	}

	offset = 0; /* alloc_range is always relative to the buddy */
	for_each_prime_number_from(size, 1, ULONG_MAX - 1) {
		LIST_HEAD(tmp);

		err = i915_buddy_alloc_range(&mm, &tmp, offset, size);
		if (err == -ENOMEM) {
			pr_info("alloc_range hit -ENOMEM with size=%llx\n",
				size);
			break;
		}

		if (!err) {
			struct i915_buddy_block *block;

			block = list_first_entry_or_null(&tmp,
							 struct i915_buddy_block,
							 link);
			if (!block) {
				if (offset + size > start) {
					pr_err("alloc_range has no blocks\n");
					err = -EINVAL;
					break;
				}
			} else {
				u64 expected = start + round_down(offset, chunk_size);

				if (i915_buddy_block_offset(block) != expected) {
					pr_err("alloc_range start offset mismatch, found=%llx, expected=%llx\n",
					       i915_buddy_block_offset(block), expected);
					err = -EINVAL;
				}

				if (!err)
					err = igt_check_blocks(&mm, &tmp, size, true);

				list_splice_tail(&tmp, &blocks);
			}

			if (err)
				break;
		}

		offset += size;
		if (offset > 2 * size)
			break;

		cond_resched();
	}

	if (err == -ENOMEM)
		err = 0;

	i915_buddy_free_list(&mm, &blocks);
	i915_buddy_fini(&mm);

	return err;
}

static int igt_buddy_alloc_limit(void *arg)
{
	struct i915_buddy_block *block;
	struct i915_buddy_mm mm;
	const u64 size = U64_MAX;
	int err;

	err = i915_buddy_init(&mm, 0, size, PAGE_SIZE);
	if (err)
		return err;

	if (mm.max_order != I915_BUDDY_MAX_ORDER) {
		pr_err("mm.max_order(%d) != %d\n",
		       mm.max_order, I915_BUDDY_MAX_ORDER);
		err = -EINVAL;
		goto out_fini;
	}

	block = i915_buddy_alloc(&mm, mm.max_order);
	if (IS_ERR(block)) {
		err = PTR_ERR(block);
		goto out_fini;
	}

	if (i915_buddy_block_order(block) != mm.max_order) {
		pr_err("block order(%d) != %d\n",
		       i915_buddy_block_order(block), mm.max_order);
		err = -EINVAL;
		goto out_free;
	}

	if (i915_buddy_block_size(&mm, block) !=
	    BIT_ULL(mm.max_order) * PAGE_SIZE) {
		pr_err("block size(%llu) != %llu\n",
		       i915_buddy_block_size(&mm, block),
		       BIT_ULL(mm.max_order) * PAGE_SIZE);
		err = -EINVAL;
		goto out_free;
	}

out_free:
	i915_buddy_free(&mm, block);
out_fini:
	i915_buddy_fini(&mm);
	return err;
}

int i915_buddy_mock_selftests(void)
{
	static const struct i915_subtest tests[] = {
		SUBTEST(igt_buddy_alloc_alignment),
		SUBTEST(igt_buddy_alloc_pessimistic),
		SUBTEST(igt_buddy_alloc_optimistic),
		SUBTEST(igt_buddy_alloc_pathological),
		SUBTEST(igt_buddy_alloc_smoke),
		SUBTEST(igt_buddy_alloc_range),
		SUBTEST(igt_buddy_alloc_range_small_misaligned),
		SUBTEST(igt_buddy_alloc_range_misaligned),
		SUBTEST(igt_buddy_alloc_limit),
	};

	return i915_subtests(tests, NULL);
}
