/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2016 Intel Corporation
 */

#include <linux/kmemleak.h>
#include <linux/slab.h>

#include <drm/drm_mm.h>

#include "i915_buddy.h"
#include "i915_scatterlist.h"
#include "i915_ttm_buddy_manager.h"

void i915_sg_trim(struct sg_table *sgt)
{
	struct scatterlist *sg;
	unsigned int n, end;

	GEM_BUG_ON(sgt->nents > sgt->orig_nents);
	if (sgt->nents == sgt->orig_nents)
		return;

	n = 0;
	end = 0;
	sg = sgt->sgl;
	do {
		struct scatterlist *chain;

		if (sgt->orig_nents - n <= SG_MAX_SINGLE_ALLOC)
			break;

		if (end == 0 && n + SG_MAX_SINGLE_ALLOC >= sgt->nents)
			end = n + SG_MAX_SINGLE_ALLOC;

		chain = sg_chain_ptr(sg + I915_MAX_CHAIN_ALLOC);
		if (n >= sgt->nents) {
			kmemleak_free(sg);
			free_page((unsigned long)sg);
		}

		n += I915_MAX_CHAIN_ALLOC;
		if (sgt->nents == n + 1) {
			sg[I915_MAX_CHAIN_ALLOC] = *chain;
			GEM_BUG_ON(!sg_is_last(sg + I915_MAX_CHAIN_ALLOC));
			GEM_BUG_ON(end != sgt->nents);
		}

		sg = chain;
	} while (1);
	if (!end)
		return;

	if (n >= sgt->nents) {
		if (sgt->orig_nents - n == SG_MAX_SINGLE_ALLOC) {
			kmemleak_free(sg);
			free_page((unsigned long)sg);
		} else {
			kfree(sg);
		}
	}

	sgt->orig_nents = end;
}

unsigned long i915_sg_compact(struct sg_table *st, unsigned long max)
{
	struct scatterlist *sg, *cur = NULL;
	unsigned long sizes = 0;
	unsigned long pfn = -1;

	GEM_BUG_ON(!IS_ALIGNED(max, PAGE_SIZE));
	if (GEM_WARN_ON(!st->orig_nents))
		return 0;

	st->nents = 0;
	for (sg = st->sgl; sg; sg = __sg_next(sg)) {
		if (!sg->length)
			continue;

		if (page_to_pfn(sg_page(sg)) == pfn && cur->length < max) {
			cur->length += PAGE_SIZE;
		} else {
			if (cur) {
				sizes |= cur->length;
				cur = __sg_next(cur);
			} else {
				cur = st->sgl;
			}
			sg_set_page(cur, sg_page(sg), sg->length, 0);
			sg_dma_address(cur) = sg_dma_address(sg);
			sg_dma_len(cur) = sg_dma_len(sg);
			st->nents++;

			pfn = page_to_pfn(sg_page(sg));
		}
		pfn++;
	}
	if (unlikely(!cur))
		cur = memset(st->sgl, 0, sizeof(*cur));
	sizes |= cur->length;
	sg_mark_end(cur);

	i915_sg_trim(st);
	return sizes;
}

/**
 * i915_sg_from_mm_node - Create an sg_table from a struct drm_mm_node
 * @node: The drm_mm_node.
 * @region_start: An offset to add to the dma addresses of the sg list.
 *
 * Create a struct sg_table, initializing it from a struct drm_mm_node,
 * taking a maximum segment length into account, splitting into segments
 * if necessary.
 *
 * Return: A pointer to a kmalloced struct sg_table on success, negative
 * error code cast to an error pointer on failure.
 */
struct sg_table *i915_sg_from_mm_node(const struct drm_mm_node *node,
				      u64 region_start)
{
	const u64 max_segment = SZ_1G; /* Do we have a limit on this? */
	u64 segment_pages = max_segment >> PAGE_SHIFT;
	u64 block_size, offset, prev_end;
	struct sg_table *st;
	struct scatterlist *sg;

	st = kmalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		return ERR_PTR(-ENOMEM);

	if (sg_alloc_table(st, DIV_ROUND_UP(node->size, segment_pages),
			   GFP_KERNEL)) {
		kfree(st);
		return ERR_PTR(-ENOMEM);
	}

	sg = st->sgl;
	st->nents = 0;
	prev_end = (resource_size_t)-1;
	block_size = node->size << PAGE_SHIFT;
	offset = node->start << PAGE_SHIFT;

	while (block_size) {
		u64 len;

		if (offset != prev_end || sg->length >= max_segment) {
			if (st->nents)
				sg = __sg_next(sg);

			sg_dma_address(sg) = region_start + offset;
			sg_dma_len(sg) = 0;
			sg->length = 0;
			st->nents++;
		}

		len = min(block_size, max_segment - sg->length);
		sg->length += len;
		sg_dma_len(sg) += len;

		offset += len;
		block_size -= len;

		prev_end = offset;
	}

	sg_mark_end(sg);
	i915_sg_trim(st);

	return st;
}

/**
 * i915_sg_from_buddy_resource - Create an sg_table from a struct
 * i915_buddy_block list
 * @res: The struct i915_ttm_buddy_resource.
 * @region_start: An offset to add to the dma addresses of the sg list.
 *
 * Create a struct sg_table, initializing it from struct i915_buddy_block list,
 * taking a maximum segment length into account, splitting into segments
 * if necessary.
 *
 * Return: A pointer to a kmalloced struct sg_table on success, negative
 * error code cast to an error pointer on failure.
 */
struct sg_table *i915_sg_from_buddy_resource(struct ttm_resource *res,
					     u64 region_start)
{
	struct i915_ttm_buddy_resource *bman_res = to_ttm_buddy_resource(res);
	const u64 size = res->num_pages << PAGE_SHIFT;
	const u64 max_segment = rounddown(UINT_MAX, PAGE_SIZE);
	struct i915_buddy_mm *mm = bman_res->mm;
	struct list_head *blocks = &bman_res->blocks;
	struct i915_buddy_block *block;
	struct scatterlist *sg;
	struct sg_table *st;
	resource_size_t prev_end;

	GEM_BUG_ON(list_empty(blocks));

	st = kmalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		return ERR_PTR(-ENOMEM);

	if (sg_alloc_table(st, res->num_pages, GFP_KERNEL)) {
		kfree(st);
		return ERR_PTR(-ENOMEM);
	}

	sg = st->sgl;
	st->nents = 0;
	prev_end = (resource_size_t)-1;

	list_for_each_entry(block, blocks, link) {
		u64 block_size, offset;

		block_size = min_t(u64, size, i915_buddy_block_size(mm, block));
		offset = i915_buddy_block_offset(block);

		while (block_size) {
			u64 len;

			if (offset != prev_end || sg->length >= max_segment) {
				if (st->nents)
					sg = __sg_next(sg);

				sg_dma_address(sg) = region_start + offset;
				sg_dma_len(sg) = 0;
				sg->length = 0;
				st->nents++;
			}

			len = min(block_size, max_segment - sg->length);
			sg->length += len;
			sg_dma_len(sg) += len;

			offset += len;
			block_size -= len;

			prev_end = offset;
		}
	}

	sg_mark_end(sg);
	i915_sg_trim(st);

	return st;
}

#if IS_ENABLED(CONFIG_DRM_I915_SELFTEST)
#include "selftests/scatterlist.c"
#endif
