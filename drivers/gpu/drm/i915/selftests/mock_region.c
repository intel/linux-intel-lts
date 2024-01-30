// SPDX-License-Identifier: MIT
/*
 * Copyright © 2019-2021 Intel Corporation
 */

#include <linux/scatterlist.h>

#include <drm/ttm/ttm_placement.h>

#include "gem/i915_gem_region.h"
#include "intel_memory_region.h"

#include "mock_region.h"

static int mock_region_put_pages(struct drm_i915_gem_object *obj,
				 struct sg_table *pages)
{
	return i915_gem_object_put_pages_buddy(obj, pages, true);
}

static int mock_region_get_pages(struct drm_i915_gem_object *obj)
{
	struct sg_table *pages;
	unsigned int sizes;

	pages = i915_gem_object_get_pages_buddy(obj, &sizes);
	if (IS_ERR(pages))
		return PTR_ERR(pages);

	__i915_gem_object_set_pages(obj, pages, sizes);
	return 0;
}

static const struct drm_i915_gem_object_ops mock_region_obj_ops = {
	.name = "mock-region",
	.get_pages = mock_region_get_pages,
	.put_pages = mock_region_put_pages,
	.release = i915_gem_object_release_memory_region,
};

static int mock_object_init(struct intel_memory_region *mem,
			    struct drm_i915_gem_object *obj,
			    resource_size_t size,
			    unsigned int flags)
{
	static struct lock_class_key lock_class;
	struct drm_i915_private *i915 = mem->i915;

	if (size > resource_size(&mem->region))
		return -E2BIG;

	drm_gem_private_object_init(&i915->drm, &obj->base, size);
	i915_gem_object_init(obj, &mock_region_obj_ops, &lock_class, flags);

	obj->read_domains = I915_GEM_DOMAIN_CPU | I915_GEM_DOMAIN_GTT;

	i915_gem_object_set_cache_coherency(obj, I915_CACHE_NONE);

	i915_gem_object_init_memory_region(obj, mem);

	return 0;
}

#if 0
static void mock_region_fini(struct intel_memory_region *mem)
{
	struct drm_i915_private *i915 = mem->i915;
	int instance = mem->instance;

	intel_region_ttm_fini(mem);
	ida_free(&i915->selftest.mock_region_instances, instance);
}
#endif

static int mock_init_region(struct intel_memory_region *mem)
{
	return intel_memory_region_init_buddy(mem,
					      mem->region.start,
					      mem->region.end + 1,
					      PAGE_SIZE);
}

static const struct intel_memory_region_ops mock_region_ops = {
	.init = mock_init_region,
	.release = intel_memory_region_release_buddy,
	.init_object = mock_object_init,
};

struct intel_memory_region *
mock_region_create(struct intel_gt *gt,
		   resource_size_t start,
		   resource_size_t size,
		   resource_size_t min_page_size,
		   resource_size_t io_start,
		   resource_size_t io_size)
{
	return intel_memory_region_create(gt, start, size, min_page_size,
					  io_start, io_size,
					  INTEL_MEMORY_MOCK, 0,
					  &mock_region_ops);
}
