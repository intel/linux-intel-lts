/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2019 Intel Corporation
 */

#ifndef __INTEL_MEMORY_REGION_H__
#define __INTEL_MEMORY_REGION_H__

#include <linux/kref.h>
#include <linux/ioport.h>
#include <linux/mutex.h>
#include <linux/io-mapping.h>
#include <drm/drm_mm.h>
#include <uapi/drm/i915_drm.h>

#include "i915_buddy.h"

struct i915_gem_ww_ctx;
struct drm_i915_private;
struct drm_i915_gem_object;
struct intel_memory_region;
struct sg_table;
struct ttm_resource;

enum intel_memory_type {
	INTEL_MEMORY_SYSTEM = I915_MEMORY_CLASS_SYSTEM,
	INTEL_MEMORY_LOCAL = I915_MEMORY_CLASS_DEVICE,
	INTEL_MEMORY_SHARED_LOCAL,
	INTEL_MEMORY_STOLEN,
	INTEL_MEMORY_MOCK,
};

enum intel_region_id {
	INTEL_REGION_SMEM = 0,
	INTEL_REGION_LMEM_0,
	INTEL_REGION_LMEM_1,
	INTEL_REGION_STOLEN,
	INTEL_REGION_UNKNOWN, /* Should be last */
};

#define REGION_SMEM     BIT(INTEL_REGION_SMEM)
#define REGION_LMEM     (BIT(INTEL_REGION_LMEM_0) | BIT(INTEL_REGION_LMEM_1))
#define REGION_STOLEN	BIT(INTEL_REGION_STOLEN)

enum {  /* Extends i915_buddy_mm */
	I915_ALLOC_CONTIGUOUS_BIT = __I915_BUDDY_ALLOC_USER_BITS,
	I915_ALLOC_MIN_PAGE_SIZE_BIT,
	I915_ALLOC_CHUNK_4K_BIT,
	I915_ALLOC_CHUNK_64K_BIT,
	I915_ALLOC_CHUNK_2M_BIT,
	I915_ALLOC_CHUNK_1G_BIT,
};
#define I915_ALLOC_CONTIGUOUS		BIT(I915_ALLOC_CONTIGUOUS_BIT)
#define I915_ALLOC_CHUNK_MIN_PAGE_SIZE	BIT(I915_ALLOC_MIN_PAGE_SIZE_BIT)
#define I915_ALLOC_CHUNK_4K		BIT(I915_ALLOC_CHUNK_4K_BIT)
#define I915_ALLOC_CHUNK_64K		BIT(I915_ALLOC_CHUNK_64K_BIT)
#define I915_ALLOC_CHUNK_2M		BIT(I915_ALLOC_CHUNK_2M_BIT)
#define I915_ALLOC_CHUNK_1G		BIT(I915_ALLOC_CHUNK_1G_BIT)

#define for_each_memory_region(mr, i915, id) \
	for (id = 0; id < ARRAY_SIZE((i915)->mm.regions); id++) \
		for_each_if((mr) = (i915)->mm.regions[id])

enum {
	INTEL_MEMORY_OVERCOMMIT_SHARED = 0,
	INTEL_MEMORY_OVERCOMMIT_LMEM
};

struct intel_memory_region_ops {
	unsigned int flags;

	int (*init)(struct intel_memory_region *mem);
	void (*release)(struct intel_memory_region *mem);

	int (*init_object)(struct intel_memory_region *mem,
			   struct drm_i915_gem_object *obj,
			   resource_size_t size,
			   unsigned int flags);
};

enum intel_memory_region_flags {
	INTEL_MEMORY_CLEAR_FREE = 0,
};

struct intel_memory_region {
	struct drm_i915_private *i915;

	struct i915_devmem *devmem;
	const struct intel_memory_region_ops *ops;

	struct io_mapping iomap;
	struct resource region;

	struct i915_buddy_mm mm;

	struct {
		struct work_struct work;
		struct llist_head blocks;
	} pd_put;

	struct kref kref;

	resource_size_t io_start;
	resource_size_t io_size;
	resource_size_t min_page_size;
	resource_size_t total;
	atomic64_t avail;
	atomic64_t evict;

	spinlock_t acct_lock;
	resource_size_t acct_limit[2];
	resource_size_t acct_user[2];

	/* Track actual LMEM size, including stolen memory */
	resource_size_t actual_physical_mem;

	unsigned long flags;

	u16 type;
	u16 instance;
	enum intel_region_id id;
	char name[16];
	struct intel_gt *gt; /* GT closest to this region. */
	u64 memtest;

	struct list_head reserved;

	struct {
		spinlock_t lock; /* Protects access to objects */
		struct list_head list;
		struct list_head purgeable;
	} objects;

	struct completion parking;

	bool is_range_manager;
	bool private; /* not for userspace */

	void *region_private;
};

struct intel_memory_region *
intel_memory_region_lookup(struct drm_i915_private *i915,
			   u16 class, u16 instance);

int intel_memory_region_init_buddy(struct intel_memory_region *mem,
				   u64 start, u64 end, u64 chunk);
void intel_memory_region_release_buddy(struct intel_memory_region *mem);

int __intel_memory_region_get_pages_buddy(struct intel_memory_region *mem,
					  struct i915_gem_ww_ctx *ww,
					  resource_size_t size,
					  unsigned int flags,
					  struct list_head *blocks);
struct i915_buddy_block *
__intel_memory_region_get_block_buddy(struct intel_memory_region *mem,
				      resource_size_t size,
				      unsigned int flags);
void __intel_memory_region_put_pages_buddy(struct intel_memory_region *mem,
					   struct list_head *blocks,
					   bool dirty);
void __intel_memory_region_put_block_buddy(struct i915_buddy_block *block);

int intel_memory_region_evict(struct intel_memory_region *mem,
			      struct i915_gem_ww_ctx *ww,
			      resource_size_t target,
			      int chunk);

struct intel_memory_region *
intel_memory_region_create(struct intel_gt *gt,
			   resource_size_t start,
			   resource_size_t size,
			   resource_size_t min_page_size,
			   resource_size_t io_start,
			   resource_size_t io_size,
			   u16 type,
			   u16 instance,
			   const struct intel_memory_region_ops *ops);

struct intel_memory_region *
intel_memory_region_get(struct intel_memory_region *mem);
void intel_memory_region_put(struct intel_memory_region *mem);

int intel_memory_regions_hw_probe(struct drm_i915_private *i915);
int intel_memory_regions_resume_early(struct drm_i915_private *i915);
void intel_memory_regions_driver_release(struct drm_i915_private *i915);
struct intel_memory_region *
intel_memory_region_by_type(struct drm_i915_private *i915,
			    enum intel_memory_type mem_type);

__printf(2, 3) void
intel_memory_region_set_name(struct intel_memory_region *mem,
			     const char *fmt, ...);

int intel_memory_region_reserve(struct intel_memory_region *mem,
				u64 offset, u64 size);

void intel_memory_region_print(struct intel_memory_region *mem,
			       resource_size_t target,
			       struct drm_printer *p);

int intel_memory_regions_add_svm(struct drm_i915_private *i915);
void intel_memory_regions_remove(struct drm_i915_private *i915);

static inline void intel_memory_region_flush(struct intel_memory_region *mem)
{
	/* Flush any pending work to free blocks region */
	flush_work(&mem->pd_put.work);
}

#endif
