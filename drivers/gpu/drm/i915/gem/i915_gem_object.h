/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2016 Intel Corporation
 */

#ifndef __I915_GEM_OBJECT_H__
#define __I915_GEM_OBJECT_H__

#include <drm/drm_gem.h>
#include <drm/drm_file.h>
#include <drm/drm_device.h>

#include "display/intel_frontbuffer.h"
#include "i915_buddy.h"
#include "i915_gem_object_types.h"
#include "i915_gem_gtt.h"
#include "i915_vma_types.h"
#include "i915_gem_ww.h"
#include "i915_drm_client.h"
#include "intel_memory_region.h"

#define obj_to_i915(obj__) to_i915((obj__)->base.dev)

#define i915_gem_object_first_segment(obj__) \
	rb_entry_safe(rb_first_cached(&(obj__)->segments), typeof(*(obj__)), segment_node)

#define for_each_object_segment(sobj__, obj__) \
	for ((sobj__) = i915_gem_object_first_segment((obj__)); \
	     (sobj__); \
	     (sobj__) = rb_entry_safe(rb_next(&(sobj__)->segment_node), typeof(*(sobj__)), segment_node))

static inline bool i915_gem_object_size_2big(u64 size)
{
	struct drm_i915_gem_object *obj;

	if (overflows_type(size, obj->base.size))
		return true;

	return false;
}

unsigned int i915_gem_get_pat_index(struct drm_i915_private *i915,
				    enum i915_cache_level level);
bool i915_gem_object_has_cache_level(const struct drm_i915_gem_object *obj,
				     enum i915_cache_level lvl);
void i915_gem_init__objects(struct drm_i915_private *i915);
u32 i915_gem_object_max_page_size(const struct drm_i915_gem_object *obj);

void i915_objects_module_exit(void);
int i915_objects_module_init(void);

struct drm_i915_gem_object *i915_gem_object_alloc(void);
void i915_gem_object_free(struct drm_i915_gem_object *obj);

void i915_gem_object_init(struct drm_i915_gem_object *obj,
			  const struct drm_i915_gem_object_ops *ops,
			  struct lock_class_key *key,
			  unsigned alloc_flags);
struct drm_i915_gem_object *
i915_gem_object_create_shmem(struct drm_i915_private *i915,
			     resource_size_t size);
struct drm_i915_gem_object *
i915_gem_object_create_shmem_from_data(struct drm_i915_private *i915,
				       const void *data, resource_size_t size);

extern const struct drm_i915_gem_object_ops i915_gem_shmem_ops;

void __i915_gem_object_release_shmem(struct drm_i915_gem_object *obj,
				     struct sg_table *pages,
				     bool needs_clflush);

int i915_gem_object_pwrite_phys(struct drm_i915_gem_object *obj,
				const struct drm_i915_gem_pwrite *args);
int i915_gem_object_pread_phys(struct drm_i915_gem_object *obj,
			       const struct drm_i915_gem_pread *args);

int i915_gem_object_attach_phys(struct drm_i915_gem_object *obj, int align);
int i915_gem_object_put_pages_shmem(struct drm_i915_gem_object *obj,
				    struct sg_table *pages);
int i915_gem_object_put_pages_phys(struct drm_i915_gem_object *obj,
				   struct sg_table *pages);

enum intel_region_id;
int i915_gem_object_prepare_move(struct drm_i915_gem_object *obj,
				 struct i915_gem_ww_ctx *ww);
bool i915_gem_object_can_migrate(struct drm_i915_gem_object *obj,
				 enum intel_region_id id);
void i915_gem_object_move_notify(struct drm_i915_gem_object *obj);
int i915_gem_object_migrate(struct drm_i915_gem_object *obj,
			    enum intel_region_id id,
			    bool nowait);
int i915_gem_object_memcpy(struct drm_i915_gem_object *dst,
			   struct drm_i915_gem_object *src);
int i915_gem_object_migrate_region(struct drm_i915_gem_object *obj,
				   struct i915_gem_ww_ctx *ww,
				   struct intel_memory_region *const *regions,
				   int size);
int i915_gem_object_migrate_to_smem(struct drm_i915_gem_object *obj,
				    struct i915_gem_ww_ctx *ww,
				    bool check_placement);

void i915_gem_flush_free_objects(struct drm_i915_private *i915);

struct drm_i915_gem_object *
i915_gem_object_lookup_segment(struct drm_i915_gem_object *obj, unsigned long offset,
			       unsigned long *adjusted_offset);
void i915_gem_object_add_segment(struct drm_i915_gem_object *obj,
				 struct drm_i915_gem_object *new_obj,
				 struct drm_i915_gem_object *prev_obj,
				 unsigned long offset);
void i915_gem_object_release_segments(struct drm_i915_gem_object *obj);

void __i915_gem_object_reset_page_iter(struct drm_i915_gem_object *obj,
				       struct sg_table *pages);

struct sg_table *
__i915_gem_object_unset_pages(struct drm_i915_gem_object *obj);
void i915_gem_object_truncate(struct drm_i915_gem_object *obj);

/**
 * i915_gem_object_lookup_rcu - look up a temporary GEM object from its handle
 * @filp: DRM file private date
 * @handle: userspace handle
 *
 * Returns:
 *
 * A pointer to the object named by the handle if such exists on @filp, NULL
 * otherwise. This object is only valid whilst under the RCU read lock, and
 * note carefully the object may be in the process of being destroyed.
 */
static inline struct drm_i915_gem_object *
i915_gem_object_lookup_rcu(struct drm_file *file, u32 handle)
{
#ifdef CONFIG_LOCKDEP
	WARN_ON(debug_locks && !lock_is_held(&rcu_lock_map));
#endif
	return idr_find(&file->object_idr, handle);
}

static inline struct drm_i915_gem_object *
i915_gem_object_get_rcu(struct drm_i915_gem_object *obj)
{
	if (obj && !kref_get_unless_zero(&obj->base.refcount))
		obj = NULL;

	return obj;
}

static inline struct drm_i915_gem_object *
i915_gem_object_lookup(struct drm_file *file, u32 handle)
{
	struct drm_i915_gem_object *obj;

	rcu_read_lock();
	obj = i915_gem_object_lookup_rcu(file, handle);
	obj = i915_gem_object_get_rcu(obj);
	rcu_read_unlock();

	return obj;
}

__deprecated
struct drm_gem_object *
drm_gem_object_lookup(struct drm_file *file, u32 handle);

__attribute__((nonnull))
static inline struct drm_i915_gem_object *
i915_gem_object_get(struct drm_i915_gem_object *obj)
{
	drm_gem_object_get(&obj->base);
	return obj;
}

__attribute__((nonnull))
static inline void
i915_gem_object_put(struct drm_i915_gem_object *obj)
{
	__drm_gem_object_put(&obj->base);
}

int i915_gem_object_account(struct drm_i915_gem_object *obj);
void i915_gem_object_unaccount(struct drm_i915_gem_object *obj);

static inline u32
i915_gem_object_get_accounting(const struct drm_i915_gem_object *obj)
{
	if (obj->memory_mask & REGION_SMEM)
		return INTEL_MEMORY_OVERCOMMIT_SHARED;
	else
		return INTEL_MEMORY_OVERCOMMIT_LMEM;
}

#define assert_object_held(obj) dma_resv_assert_held((obj)->base.resv)

/*
 * If more than one potential simultaneous locker, assert held.
 */
static inline void assert_object_held_shared(struct drm_i915_gem_object *obj)
{
	/*
	 * Note mm list lookup is protected by
	 * kref_get_unless_zero().
	 */
	if (IS_ENABLED(CONFIG_LOCKDEP) &&
	    kref_read(&obj->base.refcount) > 0)
		assert_object_held(obj);
}

static inline int __i915_gem_object_lock(struct drm_i915_gem_object *obj,
					 struct i915_gem_ww_ctx *ww,
					 bool intr)
{
	int ret;

	if (intr)
		ret = dma_resv_lock_interruptible(obj->base.resv, ww ? &ww->ctx : NULL);
	else
		ret = dma_resv_lock(obj->base.resv, ww ? &ww->ctx : NULL);

	if (!ret && ww) {
		i915_gem_object_get(obj);
		list_add_tail(&obj->obj_link, &ww->obj_list);
		obj->evict_locked = false;
	}

	if (ret == -EALREADY) {
		ret = 0;
		/* We've already evicted an object needed for this batch. */
		if (obj->evict_locked) {
			list_move_tail(&obj->obj_link, &ww->obj_list);
			obj->evict_locked = false;
		}
	}

	if (ret == -EDEADLK) {
		ww->contended_evict = false;
		i915_gem_object_get(obj);
		ww->contended = obj;
	}

	return ret;
}

static inline int i915_gem_object_lock(struct drm_i915_gem_object *obj,
				       struct i915_gem_ww_ctx *ww)
{
	return __i915_gem_object_lock(obj, ww, ww && ww->intr);
}

static inline int i915_gem_object_lock_interruptible(struct drm_i915_gem_object *obj,
						     struct i915_gem_ww_ctx *ww)
{
	WARN_ON(ww && !ww->intr);
	return __i915_gem_object_lock(obj, ww, true);
}

static inline bool i915_gem_object_trylock(struct drm_i915_gem_object *obj)
{
	return dma_resv_trylock(obj->base.resv);
}

static inline void i915_gem_object_unlock(struct drm_i915_gem_object *obj)
{
	if (obj->ops->adjust_lru)
		obj->ops->adjust_lru(obj);

	dma_resv_unlock(obj->base.resv);
}

static inline bool
i915_gem_object_has_segments(const struct drm_i915_gem_object *obj)
{
	return !RB_EMPTY_ROOT(&obj->segments.rb_root);
}

static inline bool
i915_gem_object_is_segment(const struct drm_i915_gem_object *obj)
{
	return !RB_EMPTY_NODE(&obj->segment_node);
}

static inline void
i915_gem_object_set_backing_store(struct drm_i915_gem_object *obj)
{
	obj->flags |= I915_BO_HAS_BACKING_STORE;
}

static inline bool
i915_gem_object_has_backing_store(const struct drm_i915_gem_object *obj)
{
	return obj->flags & I915_BO_HAS_BACKING_STORE;
}

static inline bool
i915_gem_object_is_exported(struct drm_i915_gem_object *obj)
{
	return obj->base.dma_buf;
}

static inline void
i915_gem_object_set_fabric(struct drm_i915_gem_object *obj)
{
	obj->flags |= I915_BO_FABRIC;
}

static inline void
i915_gem_object_clear_fabric(struct drm_i915_gem_object *obj)
{
	obj->flags &= ~I915_BO_FABRIC;
}

static inline bool
i915_gem_object_has_fabric(const struct drm_i915_gem_object *obj)
{
	return obj->flags & I915_BO_FABRIC;
}

static inline void
i915_gem_object_set_readonly(struct drm_i915_gem_object *obj)
{
	obj->flags |= I915_BO_READONLY;
}

static inline bool
i915_gem_object_is_readonly(const struct drm_i915_gem_object *obj)
{
	return obj->flags & I915_BO_READONLY;
}

static inline bool
i915_gem_object_allows_atomic_system(struct drm_i915_gem_object *obj)
{
	return obj->mm.madv_atomic == I915_BO_ATOMIC_SYSTEM;
}

static inline bool
i915_gem_object_allows_atomic_device(struct drm_i915_gem_object *obj)
{
	/* GPU atomics allowed with either ATOMIC_DEVICE or ATOMIC_SYSTEM */
	return obj->mm.madv_atomic == I915_BO_ATOMIC_DEVICE ||
	       obj->mm.madv_atomic == I915_BO_ATOMIC_SYSTEM;
}

static inline bool
i915_gem_object_test_preferred_location(struct drm_i915_gem_object *obj,
					enum intel_region_id region_id)
{

	if (!obj->mm.preferred_region)
		return false;

	return obj->mm.preferred_region->id == region_id;
}

static inline bool
i915_gem_object_is_contiguous(const struct drm_i915_gem_object *obj)
{
	return obj->flags & I915_BO_ALLOC_CONTIGUOUS;
}

static inline bool
i915_gem_object_is_volatile(const struct drm_i915_gem_object *obj)
{
	return obj->flags & I915_BO_ALLOC_VOLATILE;
}

static inline void
i915_gem_object_set_volatile(struct drm_i915_gem_object *obj)
{
	obj->flags |= I915_BO_ALLOC_VOLATILE;
}

static inline bool
i915_gem_object_has_tiling_quirk(struct drm_i915_gem_object *obj)
{
	return test_bit(I915_TILING_QUIRK_BIT, &obj->flags);
}

static inline void
i915_gem_object_set_tiling_quirk(struct drm_i915_gem_object *obj)
{
	set_bit(I915_TILING_QUIRK_BIT, &obj->flags);
}

static inline void
i915_gem_object_clear_tiling_quirk(struct drm_i915_gem_object *obj)
{
	clear_bit(I915_TILING_QUIRK_BIT, &obj->flags);
}

static inline bool
i915_gem_object_is_protected(const struct drm_i915_gem_object *obj)
{
	return obj->flags & I915_BO_PROTECTED;
}

static inline bool
i915_gem_object_type_has(const struct drm_i915_gem_object *obj,
			 unsigned long flags)
{
	return obj->ops->flags & flags;
}

static inline bool
i915_gem_object_has_struct_page(const struct drm_i915_gem_object *obj)
{
	return obj->flags & I915_BO_STRUCT_PAGE;
}

static inline bool
i915_gem_object_has_iomem(const struct drm_i915_gem_object *obj)
{
	return i915_gem_object_type_has(obj, I915_GEM_OBJECT_HAS_IOMEM);
}

static inline bool
i915_gem_object_is_shrinkable(const struct drm_i915_gem_object *obj)
{
	return i915_gem_object_type_has(obj, I915_GEM_OBJECT_IS_SHRINKABLE);
}

static inline bool
i915_gem_object_is_proxy(const struct drm_i915_gem_object *obj)
{
	return i915_gem_object_type_has(obj, I915_GEM_OBJECT_IS_PROXY);
}

static inline bool
i915_gem_object_never_mmap(const struct drm_i915_gem_object *obj)
{
	return i915_gem_object_type_has(obj, I915_GEM_OBJECT_NO_MMAP);
}

static inline bool
i915_gem_object_is_framebuffer(const struct drm_i915_gem_object *obj)
{
	return READ_ONCE(obj->frontbuffer);
}

static inline unsigned int
i915_gem_object_get_tiling(const struct drm_i915_gem_object *obj)
{
	return obj->tiling_and_stride & TILING_MASK;
}

static inline bool
i915_gem_object_is_tiled(const struct drm_i915_gem_object *obj)
{
	return i915_gem_object_get_tiling(obj) != I915_TILING_NONE;
}

static inline unsigned int
i915_gem_object_get_stride(const struct drm_i915_gem_object *obj)
{
	return obj->tiling_and_stride & STRIDE_MASK;
}

static inline unsigned int
i915_gem_tile_height(unsigned int tiling)
{
	GEM_BUG_ON(!tiling);
	return tiling == I915_TILING_Y ? 32 : 8;
}

static inline unsigned int
i915_gem_object_get_tile_height(const struct drm_i915_gem_object *obj)
{
	return i915_gem_tile_height(i915_gem_object_get_tiling(obj));
}

static inline unsigned int
i915_gem_object_get_tile_row_size(const struct drm_i915_gem_object *obj)
{
	return (i915_gem_object_get_stride(obj) *
		i915_gem_object_get_tile_height(obj));
}

int i915_gem_object_set_tiling(struct drm_i915_gem_object *obj,
			       unsigned int tiling, unsigned int stride);

struct scatterlist *
__i915_gem_object_get_sg(struct drm_i915_gem_object *obj,
			 struct i915_gem_object_page_iter *iter,
			 pgoff_t  n,
			 unsigned int *offset);

#define __i915_gem_object_get_sg(obj, it, n, offset) ({ \
	exactly_pgoff_t(n); \
	(__i915_gem_object_get_sg)(obj, it, n, offset); \
})

static inline struct scatterlist *
i915_gem_object_get_sg(struct drm_i915_gem_object *obj, pgoff_t n,
		       unsigned int *offset)
{
	return __i915_gem_object_get_sg(obj, &obj->mm.get_page, n, offset);
}

#define i915_gem_object_get_sg(obj, n, offset) ({ \
	exactly_pgoff_t(n); \
	(i915_gem_object_get_sg)(obj, n, offset); \
})

static inline struct scatterlist *
i915_gem_object_get_sg_dma(struct drm_i915_gem_object *obj, pgoff_t n,
			   unsigned int *offset)
{
	return __i915_gem_object_get_sg(obj, &obj->mm.get_dma_page, n, offset);
}

#define i915_gem_object_get_sg_dma(obj, n, offset) ({ \
	exactly_pgoff_t(n); \
	(i915_gem_object_get_sg_dma)(obj, n, offset); \
})

struct page *
i915_gem_object_get_page(struct drm_i915_gem_object *obj, pgoff_t n);

#define i915_gem_object_get_page(obj, n) ({ \
	exactly_pgoff_t(n); \
	(i915_gem_object_get_page)(obj, n); \
})

struct page *
i915_gem_object_get_dirty_page(struct drm_i915_gem_object *obj, pgoff_t n);

#define i915_gem_object_get_dirty_page(obj, n) ({ \
	exactly_pgoff_t(n); \
	(i915_gem_object_get_dirty_page)(obj, n); \
})

dma_addr_t
i915_gem_object_get_dma_address_len(struct drm_i915_gem_object *obj, pgoff_t n,
				    unsigned int *len);
#define i915_gem_object_get_dma_address_len(obj, n, len) ({ \
	exactly_pgoff_t(n); \
	(i915_gem_object_get_dma_address_len)(obj, n, len); \
})

dma_addr_t
i915_gem_object_get_dma_address(struct drm_i915_gem_object *obj, pgoff_t n);

#define i915_gem_object_get_dma_address(obj, n) ({ \
	exactly_pgoff_t(n); \
	(i915_gem_object_get_dma_address)(obj, n); \
})

unsigned int i915_gem_sg_segment_size(const struct drm_i915_gem_object *obj);

void __i915_gem_object_set_pages(struct drm_i915_gem_object *obj,
				 struct sg_table *pages,
				 unsigned int sg_page_sizes);

int ____i915_gem_object_get_pages(struct drm_i915_gem_object *obj);
int __i915_gem_object_get_pages(struct drm_i915_gem_object *obj);

void __i915_gem_object_free_mmaps(struct drm_i915_gem_object *obj);

static inline int __must_check
i915_gem_object_pin_pages(struct drm_i915_gem_object *obj)
{
	assert_object_held(obj);

	if (atomic_inc_not_zero(&obj->mm.pages_pin_count))
		return 0;

	return __i915_gem_object_get_pages(obj);
}

int i915_gem_object_pin_pages_unlocked(struct drm_i915_gem_object *obj);
int i915_gem_object_pin_pages_sync(struct drm_i915_gem_object *obj);

static inline bool
i915_gem_object_has_pages(struct drm_i915_gem_object *obj)
{
	return !IS_ERR_OR_NULL(READ_ONCE(obj->mm.pages));
}

static inline void
__i915_gem_object_pin_pages(struct drm_i915_gem_object *obj)
{
	GEM_BUG_ON(!i915_gem_object_has_pages(obj));

	atomic_inc(&obj->mm.pages_pin_count);
}

static inline bool
i915_gem_object_has_pinned_pages(struct drm_i915_gem_object *obj)
{
	return atomic_read(&obj->mm.pages_pin_count);
}

static inline void
__i915_gem_object_unpin_pages(struct drm_i915_gem_object *obj)
{
	GEM_BUG_ON(!i915_gem_object_has_pages(obj));
	GEM_BUG_ON(!i915_gem_object_has_pinned_pages(obj));

	atomic_dec(&obj->mm.pages_pin_count);
}

static inline void
i915_gem_object_unpin_pages(struct drm_i915_gem_object *obj)
{
	__i915_gem_object_unpin_pages(obj);
}

int __i915_gem_object_put_pages(struct drm_i915_gem_object *obj);
void i915_gem_object_truncate(struct drm_i915_gem_object *obj);
void i915_gem_object_writeback(struct drm_i915_gem_object *obj);

/**
 * i915_gem_object_pin_map - return a contiguous mapping of the entire object
 * @obj: the object to map into kernel address space
 * @type: the type of mapping, used to select pgprot_t
 *
 * Calls i915_gem_object_pin_pages() to prevent reaping of the object's
 * pages and then returns a contiguous mapping of the backing storage into
 * the kernel address space. Based on the @type of mapping, the PTE will be
 * set to either WriteBack or WriteCombine (via pgprot_t).
 *
 * The caller is responsible for calling i915_gem_object_unpin_map() when the
 * mapping is no longer required.
 *
 * Returns the pointer through which to access the mapped object, or an
 * ERR_PTR() on error.
 */
void *__must_check i915_gem_object_pin_map(struct drm_i915_gem_object *obj,
					   enum i915_map_type type);

void *__must_check i915_gem_object_pin_map_unlocked(struct drm_i915_gem_object *obj,
						    enum i915_map_type type);

void __i915_gem_object_flush_map(struct drm_i915_gem_object *obj,
				 unsigned long offset,
				 unsigned long size);
static inline void i915_gem_object_flush_map(struct drm_i915_gem_object *obj)
{
	__i915_gem_object_flush_map(obj, 0, obj->base.size);
}

/**
 * i915_gem_object_unpin_map - releases an earlier mapping
 * @obj: the object to unmap
 *
 * After pinning the object and mapping its pages, once you are finished
 * with your access, call i915_gem_object_unpin_map() to release the pin
 * upon the mapping. Once the pin count reaches zero, that mapping may be
 * removed.
 */
static inline void i915_gem_object_unpin_map(struct drm_i915_gem_object *obj)
{
	i915_gem_object_unpin_pages(obj);
}

void __i915_gem_object_release_map(struct drm_i915_gem_object *obj);

int i915_gem_object_prepare_read(struct drm_i915_gem_object *obj,
				 unsigned int *needs_clflush);
int i915_gem_object_prepare_write(struct drm_i915_gem_object *obj,
				  unsigned int *needs_clflush);
#define CLFLUSH_BEFORE	BIT(0)
#define CLFLUSH_AFTER	BIT(1)
#define CLFLUSH_FLAGS	(CLFLUSH_BEFORE | CLFLUSH_AFTER)

static inline void
i915_gem_object_finish_access(struct drm_i915_gem_object *obj)
{
	i915_gem_object_unpin_pages(obj);
}

void i915_gem_object_set_cache_coherency(struct drm_i915_gem_object *obj,
					 unsigned int cache_level);
void i915_gem_object_set_pat_index(struct drm_i915_gem_object *obj,
				   unsigned int pat_index);
bool i915_gem_object_can_bypass_llc(struct drm_i915_gem_object *obj);
void i915_gem_object_flush_if_display(struct drm_i915_gem_object *obj);
void i915_gem_object_flush_if_display_locked(struct drm_i915_gem_object *obj);

int __must_check
i915_gem_object_set_to_wc_domain(struct drm_i915_gem_object *obj, bool write);
int __must_check
i915_gem_object_set_to_gtt_domain(struct drm_i915_gem_object *obj, bool write);
int __must_check
i915_gem_object_set_to_cpu_domain(struct drm_i915_gem_object *obj, bool write);
struct i915_vma * __must_check
i915_gem_object_pin_to_display_plane(struct drm_i915_gem_object *obj,
				     struct i915_gem_ww_ctx *ww,
				     struct i915_ggtt *ggtt,
				     const struct i915_ggtt_view *view,
				     u32 alignment,
				     unsigned int flags);

void i915_gem_object_make_unshrinkable(struct drm_i915_gem_object *obj);
void i915_gem_object_make_shrinkable(struct drm_i915_gem_object *obj);
void i915_gem_object_make_purgeable(struct drm_i915_gem_object *obj);
int i915_gem_object_set_hint(struct drm_i915_gem_object *obj,
			     struct prelim_drm_i915_gem_vm_advise *args);

static inline bool cpu_write_needs_clflush(struct drm_i915_gem_object *obj)
{
	if (obj->cache_dirty)
		return false;

	if (!(obj->cache_coherent & I915_BO_CACHE_COHERENT_FOR_WRITE))
		return true;

	/* Currently in use by HW (display engine)? Keep flushed. */
	return i915_gem_object_is_framebuffer(obj);
}

static inline void __start_cpu_write(struct drm_i915_gem_object *obj)
{
	obj->read_domains = I915_GEM_DOMAIN_CPU;
	obj->write_domain = I915_GEM_DOMAIN_CPU;
	if (cpu_write_needs_clflush(obj))
		obj->cache_dirty = true;
}

void i915_gem_fence_wait_priority(struct dma_fence *fence, int prio);

long
__i915_gem_object_wait(struct drm_i915_gem_object *obj,
		     unsigned int flags,
		     long timeout);
int i915_gem_object_wait(struct drm_i915_gem_object *obj,
			 unsigned int flags,
			 long timeout);
int i915_gem_object_wait_priority(struct drm_i915_gem_object *obj,
				  unsigned int flags,
				  int prio);

bool i915_gem_object_is_active(struct drm_i915_gem_object *obj);

void __i915_gem_object_flush_frontbuffer(struct drm_i915_gem_object *obj,
					 enum fb_op_origin origin);
void __i915_gem_object_invalidate_frontbuffer(struct drm_i915_gem_object *obj,
					      enum fb_op_origin origin);

static inline void
i915_gem_object_flush_frontbuffer(struct drm_i915_gem_object *obj,
				  enum fb_op_origin origin)
{
	if (unlikely(rcu_access_pointer(obj->frontbuffer)))
		__i915_gem_object_flush_frontbuffer(obj, origin);
}

static inline void
i915_gem_object_invalidate_frontbuffer(struct drm_i915_gem_object *obj,
				       enum fb_op_origin origin)
{
	if (unlikely(rcu_access_pointer(obj->frontbuffer)))
		__i915_gem_object_invalidate_frontbuffer(obj, origin);
}

int i915_gem_object_read_from_page(struct drm_i915_gem_object *obj, u64 offset, void *dst, int size);

bool i915_gem_object_is_shmem(const struct drm_i915_gem_object *obj);

void __i915_gem_free_object_rcu(struct rcu_head *head);

void __i915_gem_free_object(struct drm_i915_gem_object *obj);

bool i915_gem_object_evictable(struct drm_i915_gem_object *obj);

bool i915_gem_object_migratable(struct drm_i915_gem_object *obj);

bool i915_gem_object_validates_to_lmem(struct drm_i915_gem_object *obj);

/**
 * i915_gem_get_locking_ctx - Get the locking context of a locked object
 * if any.
 *
 * @obj: The object to get the locking ctx from
 *
 * RETURN: The locking context if the object was locked using a context.
 * NULL otherwise.
 */
static inline struct i915_gem_ww_ctx *
i915_gem_get_locking_ctx(const struct drm_i915_gem_object *obj)
{
	struct ww_acquire_ctx *ctx;

	ctx = READ_ONCE(obj->base.resv->lock.ctx);
	if (!ctx)
		return NULL;

	return container_of(ctx, struct i915_gem_ww_ctx, ctx);
}

#ifdef CONFIG_MMU_NOTIFIER
static inline bool
i915_gem_object_is_userptr(struct drm_i915_gem_object *obj)
{
	return obj->userptr.notifier.mm;
}
#else
static inline bool i915_gem_object_is_userptr(struct drm_i915_gem_object *obj) { return false; }
#endif

bool i915_gem_object_should_migrate_smem(struct drm_i915_gem_object *obj,
					 bool *required);
bool i915_gem_object_should_migrate_lmem(struct drm_i915_gem_object *obj,
					 enum intel_region_id dst_region_id,
					 bool is_atomic_fault);

void i915_gem_object_migrate_prepare(struct drm_i915_gem_object *obj,
				     struct dma_fence *f);
int i915_gem_object_migrate_await(struct drm_i915_gem_object *obj,
				  struct i915_request *rq);
long i915_gem_object_migrate_wait(struct drm_i915_gem_object *obj,
				  unsigned int flags,
				  long timeout);
int i915_gem_object_migrate_sync(struct drm_i915_gem_object *obj);
void i915_gem_object_migrate_decouple(struct drm_i915_gem_object *obj);
int i915_gem_object_migrate_finish(struct drm_i915_gem_object *obj);

static inline bool
i915_gem_object_has_migrate(struct drm_i915_gem_object *obj)
{
	return !i915_active_fence_is_signaled(&obj->mm.migrate);
}

static inline int
i915_gem_object_migrate_has_error(const struct drm_i915_gem_object *obj)
{
	return i915_active_fence_has_error(&obj->mm.migrate);
}

/**
 * i915_gem_object_inuse - Is this object accessible by userspace?
 *
 * An object may be published (accessible by others and userspace)
 * only if either a GEM handle to this object exists, or if this
 * object has been exported via dma-buf.
 * For BO segments, we have to test if parent BO is accessible.
 */
static inline bool
i915_gem_object_inuse(const struct drm_i915_gem_object *obj)
{
	if (obj->parent)
		obj = obj->parent;
	return READ_ONCE(obj->base.handle_count) || obj->base.dma_buf;
}

static inline bool
i915_gem_object_mem_idle(const struct drm_i915_gem_object *obj)
{
	struct i915_buddy_block *block;

	if (!obj->mm.region.mem)
		return true;

	list_for_each_entry(block, &obj->mm.blocks, link) {
		if (!i915_active_fence_is_signaled(&block->active))
			return false;
	}

	return true;
}

void i915_gem_object_share_resv(struct drm_i915_gem_object *parent,
				struct drm_i915_gem_object *child);

#endif
