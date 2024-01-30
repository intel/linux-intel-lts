// SPDX-License-Identifier: MIT
/*
 * Copyright © 2019 Intel Corporation
 */

#include "i915_drv.h"
#include "i915_drm_client.h"
#include "i915_gem_mman.h"
#include "i915_gem_region.h"
#include "i915_trace.h"
#include "intel_memory_region.h"

int
i915_gem_object_put_pages_buddy(struct drm_i915_gem_object *obj,
				struct sg_table *pages,
				bool dirty)
{
	struct intel_memory_region *mem = obj->mm.region.mem;

	spin_lock(&mem->objects.lock);
	list_del_init(&obj->mm.region.link);
	spin_unlock(&mem->objects.lock);

	__intel_memory_region_put_pages_buddy(mem, &obj->mm.blocks, dirty);
	i915_drm_client_make_resident(obj, false);

	sg_free_table(pages);
	kfree(pages);

	return 0;
}

struct sg_table *
i915_gem_object_get_pages_buddy(struct drm_i915_gem_object *obj,
				unsigned int *page_sizes)
{
	const u64 max_segment = i915_gem_sg_segment_size(obj);
	struct i915_gem_ww_ctx *ww = i915_gem_get_locking_ctx(obj);
	struct intel_memory_region *mem = obj->mm.region.mem;
	struct list_head *blocks = &obj->mm.blocks;
	resource_size_t size = obj->base.size;
	resource_size_t prev_end;
	struct i915_buddy_block *block;
	unsigned int flags;
	struct sg_table *st;
	struct scatterlist *sg;
	unsigned int sg_page_sizes;
	pgoff_t num_pages; /* implicitly limited by sg_alloc_table */
	int ret;

	if (!safe_conversion(&num_pages, /* worst case number of sg required */
			     round_up(size, mem->min_page_size) >>
			     ilog2(mem->min_page_size)))
		 return ERR_PTR(-E2BIG);

	if (size > mem->total)
		return ERR_PTR(-E2BIG);

	st = kmalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		return ERR_PTR(-ENOMEM);

	if (sg_alloc_table(st,
			   min_t(unsigned int, num_pages, SG_MAX_SINGLE_ALLOC),
			   I915_GFP_ALLOW_FAIL)) {
		kfree(st);
		return ERR_PTR(-ENOMEM);
	}

	flags = 0;
	if (obj->flags & I915_BO_ALLOC_CHUNK_1G) {
		flags = I915_ALLOC_CHUNK_1G;
	} else if (obj->flags & I915_BO_ALLOC_CHUNK_2M) {
		flags = I915_ALLOC_CHUNK_2M;
	} else if (obj->flags & I915_BO_ALLOC_CHUNK_64K) {
		flags = I915_ALLOC_CHUNK_64K;
	} else if (obj->flags & I915_BO_ALLOC_CHUNK_4K) {
		flags = I915_ALLOC_CHUNK_4K;
	} else if (!(obj->flags & I915_BO_ALLOC_IGNORE_MIN_PAGE_SIZE)) {
		flags = I915_ALLOC_CHUNK_MIN_PAGE_SIZE;
	}
	if (obj->flags & I915_BO_ALLOC_CONTIGUOUS)
		flags |= I915_ALLOC_CONTIGUOUS;
	if (obj->flags & (I915_BO_ALLOC_USER | I915_BO_CPU_CLEAR))
		flags |= I915_BUDDY_ALLOC_WANT_CLEAR;
	if (obj->swapto) {
		flags &= ~I915_BUDDY_ALLOC_WANT_CLEAR;
		flags |= I915_BUDDY_ALLOC_ALLOW_ACTIVE;
	}

	ret = __intel_memory_region_get_pages_buddy(mem, ww, size, flags,
						    blocks);
	if (ret)
		goto err_free_sg;

	GEM_BUG_ON(list_empty(blocks));

	sg = st->sgl;
	st->nents = 0;
	sg_page_sizes = 0;
	prev_end = (resource_size_t)-1;

	list_for_each_entry(block, blocks, link) {
		u64 block_size, offset;

		block_size = min_t(u64, size,
				   i915_buddy_block_size(&mem->mm, block));
		offset = i915_buddy_block_offset(block);

		while (block_size) {
			u64 len;

			if (offset != prev_end || sg->length >= max_segment) {
				if (st->nents) {
					sg_dma_len(sg) = sg->length;
					sg_page_sizes |= sg->length;

					if (sg_is_last(sg)) {
						struct scatterlist *chain;

						chain = (void *)__get_free_page(I915_GFP_ALLOW_FAIL);
						if (!chain) {
							ret = -ENOMEM;
							goto err_free_sg;
						}
						sg_init_table(chain, SG_MAX_SINGLE_ALLOC);

						sg_unmark_end(memcpy(chain, sg, sizeof(*sg)));
						__sg_chain(sg, chain);
						st->orig_nents += I915_MAX_CHAIN_ALLOC;

						GEM_BUG_ON(sg_chain_ptr(sg) != chain);
						sg = chain;
					}
					GEM_BUG_ON(sg_is_last(sg));
					sg++;
				}

				sg_dma_address(sg) = offset;
				sg->length = 0;
				st->nents++;
			}

			len = min(block_size, max_segment - sg->length);
			sg->length += len;

			offset += len;
			block_size -= len;

			prev_end = offset;
		}
	}

	sg_dma_len(sg) = sg->length;
	sg_page_sizes |= sg->length;
	sg_mark_end(sg);

	i915_drm_client_make_resident(obj, true);

	*page_sizes = sg_page_sizes;
	return st;

err_free_sg:
	sg_free_table(st);
	kfree(st);
	return ERR_PTR(ret);
}

void i915_gem_object_init_memory_region(struct drm_i915_gem_object *obj,
					struct intel_memory_region *mem)
{
	GEM_BUG_ON(i915_gem_object_has_pages(obj));

	obj->mm.region.mem = intel_memory_region_get(mem);
	INIT_LIST_HEAD(&obj->mm.blocks);

	if (obj->base.size <= mem->min_page_size)
		obj->flags |= I915_BO_ALLOC_CONTIGUOUS;
}

void i915_gem_object_release_memory_region(struct drm_i915_gem_object *obj)
{
	struct intel_memory_region *mem;

	mem = fetch_and_zero(&obj->mm.region.mem);
	if (!mem)
		return;

	/* Added to the region list before get_pages failed? */
	if (!list_empty(&obj->mm.region.link)) {
		spin_lock(&mem->objects.lock);
		list_del_init(&obj->mm.region.link);
		spin_unlock(&mem->objects.lock);
	}

	intel_memory_region_put(mem);
}

struct drm_i915_gem_object *
i915_gem_object_create_region(struct intel_memory_region *mem,
			      resource_size_t size,
			      unsigned int flags)
{
	struct drm_i915_gem_object *obj;
	int err;

	/*
	 * NB: Our use of resource_size_t for the size stems from using struct
	 * resource for the mem->region. We might need to revisit this in the
	 * future.
	 */

	if (!mem)
		return ERR_PTR(-ENODEV);

	if (flags & I915_BO_ALLOC_CHUNK_2M)
		size = round_up(size, SZ_2M);
	else if (flags & I915_BO_ALLOC_CHUNK_64K)
		size = round_up(size, SZ_64K);
	else if (!(flags & I915_BO_ALLOC_IGNORE_MIN_PAGE_SIZE))
		size = round_up(size, mem->min_page_size);

	GEM_BUG_ON(!size);
	GEM_BUG_ON(!IS_ALIGNED(size, I915_GTT_MIN_ALIGNMENT));

	if (i915_gem_object_size_2big(size) || size > mem->total)
		return ERR_PTR(-E2BIG);

	obj = i915_gem_object_alloc();
	if (!obj)
		return ERR_PTR(-ENOMEM);

	err = mem->ops->init_object(mem, obj, size, flags);
	if (err)
		goto err_object_free;

	trace_i915_gem_object_create(obj, 0);
	return obj;

err_object_free:
	i915_gem_object_free(obj);
	return ERR_PTR(err);
}
