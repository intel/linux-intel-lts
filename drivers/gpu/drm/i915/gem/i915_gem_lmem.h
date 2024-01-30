/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2019 Intel Corporation
 */

#ifndef __I915_GEM_LMEM_H
#define __I915_GEM_LMEM_H

#include <linux/types.h>

struct drm_i915_private;
struct drm_i915_gem_object;
struct intel_gt;
struct intel_memory_region;

extern const struct drm_i915_gem_object_ops i915_gem_lmem_obj_ops;

void __iomem *
i915_gem_object_lmem_io_map(struct drm_i915_gem_object *obj,
			    unsigned long n,
			    unsigned long size);
void __iomem *i915_gem_object_lmem_io_map_page(struct drm_i915_gem_object *obj,
					       unsigned long n);
void __iomem *
i915_gem_object_lmem_io_map_page_atomic(struct drm_i915_gem_object *obj,
					unsigned long n);

unsigned long i915_gem_object_lmem_offset(struct drm_i915_gem_object *obj);

bool i915_gem_object_is_lmem(const struct drm_i915_gem_object *obj);

struct drm_i915_gem_object *
i915_gem_object_create_lmem_from_data(struct intel_memory_region *region,
				      const void *data, size_t size);

struct drm_i915_gem_object *
i915_gem_object_create_lmem(struct drm_i915_private *i915,
			    resource_size_t size,
			    unsigned int flags);
int __i915_gem_lmem_object_init(struct intel_memory_region *mem,
				struct drm_i915_gem_object *obj,
				resource_size_t size,
				unsigned int flags);

int i915_gem_object_clear_lmem(struct drm_i915_gem_object *obj);
struct i915_request *
i915_gem_object_copy_lmem(struct drm_i915_gem_object *lmem,
			  struct drm_i915_gem_object *other,
			  bool to_other,
			  bool nowait);

void i915_gem_init_lmem(struct intel_gt *gt);
bool i915_gem_lmem_park(struct intel_memory_region *mem);
void i915_gem_fini_lmem(struct intel_gt *gt);

int i915_gem_lmemtest(struct intel_gt *gt, u64 *error);

#endif /* !__I915_GEM_LMEM_H */
