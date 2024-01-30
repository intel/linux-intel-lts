// SPDX-License-Identifier: MIT
/*
 * Copyright © 2020 Intel Corporation
 */

#include <drm/drm_fourcc.h>

#include "gem/i915_gem_ioctls.h"
#include "gem/i915_gem_lmem.h"
#include "gem/i915_gem_region.h"
#include "gem/i915_gem_object.h"
#include "pxp/intel_pxp.h"

#include "i915_drv.h"
#include "i915_gem_create.h"
#include "i915_trace.h"
#include "i915_user_extensions.h"

struct create_ext {
	struct drm_i915_private *i915;
	struct drm_i915_gem_object *vanilla_object;
	unsigned long flags;
	u64 segment_size;
	u32 vm_id;
	u32 pair_id;
};

static u32 placement_mask(struct intel_memory_region *const *placements,
			  int n_placements)
{
	u32 mask = 0;
	int i;

	for (i = 0; i < n_placements; i++)
		mask |= BIT(placements[i]->id);

	GEM_BUG_ON(!mask);

	return mask;
}

u32 i915_gem_object_max_page_size(const struct drm_i915_gem_object *obj)
{
	u32 max_page_size = I915_GTT_PAGE_SIZE_4K;
	int i;

	for (i = 0; i < obj->mm.n_placements; i++) {
		struct intel_memory_region *mr = obj->mm.placements[i];

		GEM_BUG_ON(!is_power_of_2(mr->min_page_size));
		max_page_size = max_t(u32, max_page_size, mr->min_page_size);
	}

	return max_page_size;
}

static void object_set_placements(struct drm_i915_gem_object *obj,
				  struct intel_memory_region **placements,
				  unsigned int n_placements)
{
	GEM_BUG_ON(!n_placements);

	/*
	 * For the common case of one memory region, skip storing an
	 * allocated array and just point at the region directly.
	 */
	if (n_placements == 1) {
		struct intel_memory_region *mr = placements[0];
		struct drm_i915_private *i915 = mr->i915;

		obj->mm.placements = &i915->mm.regions[mr->id];
		obj->mm.n_placements = 1;
	} else {
		obj->mm.placements = placements;
		obj->mm.n_placements = n_placements;
	}

	obj->memory_mask =
		placement_mask(obj->mm.placements, obj->mm.n_placements);
}

static u64 object_limit(struct drm_i915_gem_object *obj)
{
	u64 min_region_size = U64_MAX;
	int i;

	for (i = 0; i < obj->mm.n_placements; i++) {
		struct intel_memory_region *mr = obj->mm.placements[i];

		min_region_size = min_t(u64, min_region_size, mr->total);
	}

	return min_region_size;
}

static u64 object_size_align(struct intel_memory_region *mr, u64 size, u32 *flags)
{
	unsigned long page_sz_mask = mr->i915->params.page_sz_mask;
	u32 alloc_flags = 0;

	if (mr->type == INTEL_MEMORY_LOCAL && page_sz_mask) {
		unsigned long alignment = 0;

		if (page_sz_mask & BIT(0)) {
			alloc_flags |= I915_BO_ALLOC_CHUNK_4K;
			alignment = SZ_4K;
		} else if (page_sz_mask & BIT(1)) {
			alloc_flags |= I915_BO_ALLOC_CHUNK_64K;
			alignment = SZ_64K;
		} else if (page_sz_mask & BIT(2)) {
			alloc_flags |= I915_BO_ALLOC_CHUNK_2M;
			alignment = SZ_2M;
		} else if (page_sz_mask & BIT(3)) {
			alloc_flags |= I915_BO_ALLOC_CHUNK_1G;
			alignment = SZ_1G;
		}
		size = round_up(size, alignment);
	}

	*flags |= alloc_flags;
	return size;
}

static int i915_gem_publish(struct drm_i915_gem_object *obj,
			    struct drm_file *file,
			    u64 *size_p,
			    u32 *handle_p)
{
	u64 size = obj->base.size;
	int ret;

	ret = drm_gem_handle_create(file, &obj->base, handle_p);
	/* drop reference from allocate - handle holds it now */
	i915_gem_object_put(obj);
	if (ret)
		return ret;

	*size_p = size;
	return 0;
}

/*
 * This routine will determined if BO segmentation will be enabled,
 * based on user supplied ext->segment_size and object size.
 * Must be larger than the minimum chunk size and need 2 chunks.
 * We do not require any alignment to chunk size, and so the last chunk can
 * be partial.
 */
static u64 get_object_segment_size(u64 obj_size, u64 requested_size)
{
	if (i915_modparams.disable_bo_chunking)
		return 0;

	/* require 2+ chunks */
	if (!(requested_size && obj_size >= requested_size << 1))
		return 0;

	return requested_size;
}

static int
setup_object(struct drm_i915_gem_object *obj, u64 size,
	     struct create_ext *ext)
{
	struct intel_memory_region *mr = obj->mm.placements[0];
	u64 obj_min_size, obj_segment_size = 0;
	u64 nsegments = 0;
	u32 alloc_flags;
	int ret;

	size = round_up(size, i915_gem_object_max_page_size(obj));
	if (size == 0)
		return -EINVAL;

	i915_gem_flush_free_objects(mr->i915);

	/* For most of the ABI (e.g. mmap) we think in system pages */
	GEM_BUG_ON(!IS_ALIGNED(size, PAGE_SIZE));

	if (ext)
		obj_segment_size = get_object_segment_size(size,
							   ext->segment_size);
	obj_min_size = obj_segment_size ?: size;

	if (overflows_type(size, obj->base.size) || obj_min_size > object_limit(obj))
		return -E2BIG;

	alloc_flags = i915_modparams.force_alloc_contig & ALLOC_CONTIGUOUS_LMEM ?
		I915_BO_ALLOC_CONTIGUOUS : 0;

	size = object_size_align(mr, size, &alloc_flags);
	alloc_flags |= I915_BO_ALLOC_USER;
	ret = mr->ops->init_object(mr, obj, size, alloc_flags);
	if (ret)
		return ret;

	GEM_BUG_ON(size != obj->base.size);

	if (obj_segment_size) {
		struct drm_i915_gem_object *sobj, *prev_obj = NULL;
		u64 segment_offset = 0;
		u64 segment_size;

		while (size) {
			sobj = i915_gem_object_alloc();
			if (!sobj) {
				ret = -ENOMEM;
				break;
			}

			/* point to same placement array as parent */
			object_set_placements(sobj, obj->mm.placements,
					      obj->mm.n_placements);
			segment_size = min_t(u64, obj_segment_size, size);
			ret = mr->ops->init_object(mr, sobj, segment_size, alloc_flags);
			if (ret) {
				i915_gem_object_free(sobj);
				break;
			}

			i915_gem_object_add_segment(obj, sobj, prev_obj, segment_offset);
			segment_offset += sobj->base.size;
			size -= sobj->base.size;
			prev_obj = sobj;
			nsegments++;
		}

		if (ret)
			goto err_segments;
	}

	ret = i915_gem_object_account(obj);
	if (ret) {
err_segments:
		i915_gem_object_release_segments(obj);
		obj->ops->release(obj);
		return ret;
	}

	trace_i915_gem_object_create(obj, nsegments);

	if (IS_ENABLED(CONFIG_DRM_I915_CHICKEN_CLEAR_ON_CREATE) &&
	    !i915_gem_object_has_segments(obj)) {
		struct i915_gem_ww_ctx ww;

		for_i915_gem_ww(&ww, ret, true) {
			ret = i915_gem_object_lock(obj, &ww);
			if (ret)
				continue;

			ret = i915_gem_object_pin_pages(obj); /* queue only */
			if (ret == 0)
				i915_gem_object_unpin_pages(obj);
		}

		/* error handling is deferred to use */
	}

	return 0;
}

static int account_size(struct drm_i915_private *i915, unsigned long
			region_mask, int acct, ssize_t size)
{
	int region_id;

	for_each_set_bit(region_id, &region_mask, INTEL_REGION_UNKNOWN) {
		struct intel_memory_region *mr = i915->mm.regions[region_id];
		int ret = 0;

		spin_lock(&mr->acct_lock);

		if (!i915_allows_overcommit(i915)) {
			if (size > 0 && mr->acct_limit[acct] < size)
				ret = -EPERM;
			if (!ret)
				mr->acct_limit[acct] -= size;
		}

		if (!ret)
			mr->acct_user[acct] += size;

		spin_unlock(&mr->acct_lock);

		if (ret) {
			account_size(i915, region_mask & (BIT(region_id) - 1),
				     acct, -size);

			return ret;
		}
	}

	return 0;
}

int i915_gem_object_account(struct drm_i915_gem_object *obj)
{
	if (!(obj->flags & I915_BO_ALLOC_USER))
		return 0;

	return account_size(to_i915(obj->base.dev),
			    obj->memory_mask & REGION_LMEM,
			    i915_gem_object_get_accounting(obj),
			    obj->base.size);
}

void i915_gem_object_unaccount(struct drm_i915_gem_object *obj)
{
	if (!(obj->flags & I915_BO_ALLOC_USER))
		return;

	account_size(to_i915(obj->base.dev),
		     obj->memory_mask & REGION_LMEM,
		     i915_gem_object_get_accounting(obj),
		     -obj->base.size);
}

int
i915_gem_dumb_create(struct drm_file *file,
		     struct drm_device *dev,
		     struct drm_mode_create_dumb *args)
{
	struct drm_i915_gem_object *obj;
	struct intel_memory_region *mr;
	enum intel_memory_type mem_type;
	int cpp = DIV_ROUND_UP(args->bpp, 8);
	u32 format;
	int ret;

	switch (cpp) {
	case 1:
		format = DRM_FORMAT_C8;
		break;
	case 2:
		format = DRM_FORMAT_RGB565;
		break;
	case 4:
		format = DRM_FORMAT_XRGB8888;
		break;
	default:
		return -EINVAL;
	}

	/* have to work out size/pitch and return them */
	args->pitch = ALIGN(args->width * cpp, 64);

	/* align stride to page size so that we can remap */
	if (args->pitch > intel_plane_fb_max_stride(to_i915(dev), format,
						    DRM_FORMAT_MOD_LINEAR))
		args->pitch = ALIGN(args->pitch, 4096);

	if (args->pitch < args->width)
		return -EINVAL;

	args->size = mul_u32_u32(args->pitch, args->height);

	mem_type = INTEL_MEMORY_SYSTEM;
	if (HAS_LMEM(to_i915(dev)))
		mem_type = INTEL_MEMORY_LOCAL;

	obj = i915_gem_object_alloc();
	if (!obj)
		return -ENOMEM;

	mr = intel_memory_region_by_type(to_i915(dev), mem_type);
	object_set_placements(obj, &mr, 1);

	ret = setup_object(obj, args->size, NULL);
	if (ret)
		goto object_free;

	return i915_gem_publish(obj, file, &args->size, &args->handle);

object_free:
	i915_gem_object_free(obj);
	return ret;
}

static void repr_placements(char *buf, size_t size,
			    struct intel_memory_region **placements,
			    int n_placements)
{
	int i;

	buf[0] = '\0';

	for (i = 0; i < n_placements; i++) {
		struct intel_memory_region *mr = placements[i];
		int r;

		r = snprintf(buf, size, "\n  %s -> { class: %d, inst: %d }",
			     mr->name, mr->type, mr->instance);
		if (r >= size)
			return;

		buf += r;
		size -= r;
	}
}

static int prelim_set_placements(struct prelim_drm_i915_gem_object_param *args,
			  struct create_ext *ext_data)
{
	struct drm_i915_private *i915 = ext_data->i915;
	struct prelim_drm_i915_gem_memory_class_instance __user *uregions =
		u64_to_user_ptr(args->data);
	struct drm_i915_gem_object *obj = ext_data->vanilla_object;
	struct intel_memory_region **placements;
	u32 mask;
	int i, ret = 0;

	if (!args->size) {
		DRM_DEBUG("Size is zero\n");
		ret = -EINVAL;
	}

	if (args->size > ARRAY_SIZE(i915->mm.regions)) {
		DRM_DEBUG("Too many placements\n");
		ret = -EINVAL;
	}

	if (ret)
		return ret;

	placements = kmalloc_array(args->size,
				   sizeof(struct intel_memory_region *),
				   GFP_KERNEL);
	if (!placements)
		return -ENOMEM;

	mask = 0;
	for (i = 0; i < args->size; i++) {
		struct prelim_drm_i915_gem_memory_class_instance region;
		struct intel_memory_region *mr;

		if (copy_from_user(&region, uregions, sizeof(region))) {
			ret = -EFAULT;
			goto out_free;
		}

		mr = intel_memory_region_lookup(i915,
						region.memory_class,
						region.memory_instance);
		if (!mr || mr->private) {
			DRM_DEBUG("Device is missing region { class: %d, inst: %d } at index = %d\n",
				  region.memory_class, region.memory_instance, i);
			ret = -EINVAL;
			goto out_dump;
		}

		if (mask & BIT(mr->id)) {
			DRM_DEBUG("Found duplicate placement %s -> { class: %d, inst: %d } at index = %d\n",
				  mr->name, region.memory_class,
				  region.memory_instance, i);
			ret = -EINVAL;
			goto out_dump;
		}

		placements[i] = mr;
		mask |= BIT(mr->id);

		++uregions;
	}

	if (obj->mm.placements) {
		ret = -EINVAL;
		goto out_dump;
	}

	object_set_placements(obj, placements, args->size);
	if (args->size == 1)
		kfree(placements);

	return 0;

out_dump:
	if (1) {
		char buf[256];

		if (obj->mm.placements) {
			repr_placements(buf,
					sizeof(buf),
					obj->mm.placements,
					obj->mm.n_placements);
			DRM_DEBUG("Placements were already set in previous SETPARAM. Existing placements: %s\n",
				  buf);
		}

		repr_placements(buf, sizeof(buf), placements, i);
		DRM_DEBUG("New placements(so far validated): %s\n", buf);
	}

out_free:
	kfree(placements);
	return ret;
}

static int prelim_set_pair(struct prelim_drm_i915_gem_object_param *args,
			   struct create_ext *ext_data)
{
	int ret = 0;

	/* start with no pairing */
	ext_data->pair_id = 0;

	if (!args->data) {
		DRM_DEBUG("Data should be non-zero\n");
		ret = -EINVAL;
	}

	if (ret)
		return ret;

	/*
	 * data is the "handle" of the object we are going to pair
	 * with.
	 */
	ext_data->pair_id = (u32)args->data;

	return 0;
}

static int prelim_set_chunk_size(struct prelim_drm_i915_gem_object_param *args,
				 struct create_ext *ext_data)
{
	struct drm_i915_private *i915 = ext_data->i915;
	u64 segment_size = args->data;

	/* enabled on platforms where legacy mmap is no longer supported */
	if (!(IS_DGFX(i915) || GRAPHICS_VER_FULL(i915) > IP_VER(12, 0)))
		return -EOPNOTSUPP;

	if (!segment_size)
		return -EINVAL;
	if (segment_size < I915_BO_MIN_CHUNK_SIZE)
		return -ENOSPC;
	if (!IS_ALIGNED(segment_size, I915_BO_MIN_CHUNK_SIZE))
		return -EINVAL;

	ext_data->segment_size = segment_size;
	return 0;
}

static int __create_setparam(struct prelim_drm_i915_gem_object_param *args,
			     struct create_ext *ext_data)
{
	if (!(args->param & PRELIM_I915_OBJECT_PARAM)) {
		DRM_DEBUG("Missing I915_OBJECT_PARAM namespace\n");
		return -EINVAL;
	}

	if (args->handle) {
		DRM_DEBUG("Handle should be zero\n");
		return -EINVAL;
	}

	switch (lower_32_bits(args->param)) {
	case PRELIM_I915_PARAM_MEMORY_REGIONS:
		return prelim_set_placements(args, ext_data);
	case PRELIM_I915_PARAM_SET_PAIR:
		return prelim_set_pair(args, ext_data);
	case PRELIM_I915_PARAM_SET_CHUNK_SIZE:
		return prelim_set_chunk_size(args, ext_data);
	}

	return -EINVAL;
}

static int create_setparam(struct i915_user_extension __user *base, void *data)
{
	struct prelim_drm_i915_gem_create_ext_setparam ext;

	if (copy_from_user(&ext, base, sizeof(ext)))
		return -EFAULT;

	return __create_setparam(&ext.param, data);
}

static int ext_set_vm_private(struct i915_user_extension __user *base,
			      void *data)
{
	struct prelim_drm_i915_gem_create_ext_vm_private ext;
	struct create_ext *ext_data = data;

	if (copy_from_user(&ext, base, sizeof(ext)))
		return -EFAULT;

	ext_data->vm_id = ext.vm_id;

	return 0;
}

static int ext_set_protected(struct i915_user_extension __user *base, void *data);
static const i915_user_extension_fn prelim_create_extensions[] = {
	[PRELIM_I915_USER_EXT_MASK(PRELIM_I915_GEM_CREATE_EXT_SETPARAM)] = create_setparam,
	[PRELIM_I915_USER_EXT_MASK(PRELIM_I915_GEM_CREATE_EXT_VM_PRIVATE)] = ext_set_vm_private,
	[PRELIM_I915_USER_EXT_MASK(PRELIM_I915_GEM_CREATE_EXT_PROTECTED_CONTENT)] = ext_set_protected,
};

static int attach_vm(struct drm_i915_gem_object *obj)
{
	struct i915_address_space *vm;
	int ret = 0;

	vm = obj->vm;
	if (!vm)
		return 0;

	spin_lock(&vm->priv_obj_lock);
	if (atomic_read(&vm->open))
		list_add_tail(&obj->priv_obj_link, &vm->priv_obj_list);
	else
		ret = -ENOENT;
	spin_unlock(&vm->priv_obj_lock);

	i915_gem_object_share_resv(vm->root_obj, obj);
	i915_vm_put(vm);

	return ret;
}

static int check_for_pair(struct drm_file *file, struct drm_i915_gem_object *obj,
			  struct create_ext *ext_data)
{
	struct drm_i915_gem_object *first;

	/*
	 * If pair_id is set to a handle, find the object and validate that it
	 * can be used.
	 */
	if (!ext_data->pair_id)
		return 0;

	/*
	 * For the found object, i915_gem_object_put() needs to be done in
	 * __i915_gem_free_object()
	 */
	first = i915_gem_object_lookup(file, ext_data->pair_id);
	if (!first) {
		drm_dbg(&to_i915(obj->base.dev)->drm, "Object pair not found\n");
		return -EINVAL;
	}

	/* already paired? */
	if (first->pair) {
		drm_dbg(&to_i915(obj->base.dev)->drm, "Object is already paired\n");
		goto err_out;
	}

	if (obj->mm.n_placements > 1 || first->mm.n_placements > 1) {
		drm_dbg(&to_i915(obj->base.dev)->drm, "Implicit pairs cannot migrate\n");
		goto err_out;
	}

	if (!i915_gem_object_is_lmem(obj) || !i915_gem_object_is_lmem(first)) {
		drm_dbg(&to_i915(obj->base.dev)->drm, "Implicit pairs MUST be LMEM\n");
		goto err_out;
	}

	/* obj memory regions should not be the same */
	if (obj->mm.region.mem == first->mm.region.mem) {
		drm_dbg(&to_i915(obj->base.dev)->drm, "Object pair must be in different regions\n");
		goto err_out;
	}

	/*
	 * OK, this pairing is good to go.
	 * To keep the to be paired object from getting deleted, do a _get.
	 * On close of first, do the put and break the link.
	 * Do the _put on first because _lookup did a get.
	 * NOTE!: pairing is uni-dircetional.
	 */
	i915_gem_object_get(obj);
	first->pair = obj;
	i915_gem_object_put(first);

	return 0;

err_out:
	i915_gem_object_put(first);
	return -EINVAL;
}

/**
 * Creates a new mm object and returns a handle to it.
 * @dev: drm device pointer
 * @data: ioctl data blob
 * @file: drm file pointer
 */
int
i915_gem_create_ioctl(struct drm_device *dev, void *data,
		      struct drm_file *file)
{
	struct drm_i915_private *i915 = to_i915(dev);
	struct prelim_drm_i915_gem_create_ext *args = data;
	struct create_ext ext_data = { .i915 = i915 };
	struct intel_memory_region **placements_ext;
	struct intel_memory_region *stack[1];
	struct drm_i915_gem_object *obj;
	int ret;

	obj = i915_gem_object_alloc();
	if (!obj)
		return -ENOMEM;

	ext_data.vanilla_object = obj;
	ret = i915_user_extensions(u64_to_user_ptr(args->extensions),
				   prelim_create_extensions,
				   ARRAY_SIZE(prelim_create_extensions),
				   &ext_data);
	placements_ext = obj->mm.placements;
	if (ret)
		goto object_free;

	if (ext_data.vm_id) {
		obj->vm = i915_address_space_lookup(file->driver_priv,
						    ext_data.vm_id);
		if (unlikely(!obj->vm)) {
			ret = -ENOENT;
			goto object_free;
		}
	}

	if (!placements_ext) {
		enum intel_memory_type mem_type = INTEL_MEMORY_SYSTEM;

		stack[0] = intel_memory_region_by_type(i915, mem_type);
		object_set_placements(obj, stack, 1);
	}

	ret = setup_object(obj, args->size, &ext_data);
	if (ret)
		goto vm_put;

	ret = check_for_pair(file, obj, &ext_data);
	if (ret)
		goto obj_put;

	ret = attach_vm(obj);
	if (ret)
		goto obj_put;

	/* Add any flag set by create_ext options */
	obj->flags |= ext_data.flags;

	return i915_gem_publish(obj, file, &args->size, &args->handle);

vm_put:
	if (obj->vm)
		i915_vm_put(obj->vm);
object_free:
	if (obj->mm.n_placements > 1)
		kfree(placements_ext);

	i915_gem_object_free(obj);
	return ret;

obj_put:
	i915_gem_object_put(obj);
	return ret;
}

static int set_placements(struct drm_i915_gem_create_ext_memory_regions *args,
			  struct create_ext *ext_data)
{
	struct drm_i915_private *i915 = ext_data->i915;
	struct drm_i915_gem_memory_class_instance __user *uregions =
		u64_to_user_ptr(args->regions);
	struct drm_i915_gem_object *obj = ext_data->vanilla_object;
	struct intel_memory_region **placements;
	u32 mask;
	int i, ret = 0;

	if (args->pad) {
		drm_dbg(&i915->drm, "pad should be zero\n");
		ret = -EINVAL;
	}

	if (!args->num_regions) {
		drm_dbg(&i915->drm, "num_regions is zero\n");
		ret = -EINVAL;
	}

	if (args->num_regions > ARRAY_SIZE(i915->mm.regions)) {
		drm_dbg(&i915->drm, "num_regions is too large\n");
		ret = -EINVAL;
	}

	if (ret)
		return ret;

	placements = kmalloc_array(args->num_regions,
				   sizeof(struct intel_memory_region *),
				   GFP_KERNEL);
	if (!placements)
		return -ENOMEM;

	mask = 0;
	for (i = 0; i < args->num_regions; i++) {
		struct drm_i915_gem_memory_class_instance region;
		struct intel_memory_region *mr;

		if (copy_from_user(&region, uregions, sizeof(region))) {
			ret = -EFAULT;
			goto out_free;
		}

		mr = intel_memory_region_lookup(i915,
						region.memory_class,
						region.memory_instance);
		if (!mr || mr->private) {
			drm_dbg(&i915->drm, "Device is missing region { class: %d, inst: %d } at index = %d\n",
				region.memory_class, region.memory_instance, i);
			ret = -EINVAL;
			goto out_dump;
		}

		if (mask & BIT(mr->id)) {
			drm_dbg(&i915->drm, "Found duplicate placement %s -> { class: %d, inst: %d } at index = %d\n",
				mr->name, region.memory_class,
				region.memory_instance, i);
			ret = -EINVAL;
			goto out_dump;
		}

		placements[i] = mr;
		mask |= BIT(mr->id);

		++uregions;
	}

	if (obj->mm.placements) {
		ret = -EINVAL;
		goto out_dump;
	}

	object_set_placements(obj, placements, args->num_regions);
	if (args->num_regions == 1)
		kfree(placements);

	return 0;

out_dump:
	if (1) {
		char buf[256];

		if (obj->mm.placements) {
			repr_placements(buf,
					sizeof(buf),
					obj->mm.placements,
					obj->mm.n_placements);
			drm_dbg(&i915->drm,
				"Placements were already set in previous EXT. Existing placements: %s\n",
				buf);
		}

		repr_placements(buf, sizeof(buf), placements, i);
		drm_dbg(&i915->drm, "New placements(so far validated): %s\n", buf);
	}

out_free:
	kfree(placements);
	return ret;
}

static int ext_set_placements(struct i915_user_extension __user *base,
			      void *data)
{
	struct drm_i915_gem_create_ext_memory_regions ext;

	if (copy_from_user(&ext, base, sizeof(ext)))
		return -EFAULT;

	return set_placements(&ext, data);
}

static int ext_set_protected(struct i915_user_extension __user *base, void *data)
{
	struct drm_i915_gem_create_ext_protected_content ext;
	struct create_ext *ext_data = data;

	if (copy_from_user(&ext, base, sizeof(ext)))
		return -EFAULT;

	if (ext.flags)
		return -EINVAL;

	if (!intel_pxp_is_enabled(&ext_data->i915->gt0.pxp))
		return -ENODEV;

	ext_data->flags |= I915_BO_PROTECTED;

	return 0;
}

static const i915_user_extension_fn create_extensions[] = {
	[I915_GEM_CREATE_EXT_MEMORY_REGIONS] = ext_set_placements,
	[I915_GEM_CREATE_EXT_PROTECTED_CONTENT] = ext_set_protected,
};

/**
 * Creates a new mm object and returns a handle to it.
 * @dev: drm device pointer
 * @data: ioctl data blob
 * @file: drm file pointer
 */
int
i915_gem_create_ext_ioctl(struct drm_device *dev, void *data,
			  struct drm_file *file)
{
	struct drm_i915_private *i915 = to_i915(dev);
	struct drm_i915_gem_create_ext *args = data;
	struct create_ext ext_data = { .i915 = i915 };
	struct intel_memory_region **placements_ext;
	struct drm_i915_gem_object *obj;
	int ret;

	if (args->flags)
		return -EINVAL;

	obj = i915_gem_object_alloc();
	if (!obj)
		return -ENOMEM;

	ext_data.vanilla_object = obj;
	ret = i915_user_extensions(u64_to_user_ptr(args->extensions),
				   create_extensions,
				   ARRAY_SIZE(create_extensions),
				   &ext_data);
	placements_ext = obj->mm.placements;
	if (ret)
		goto object_free;

	if (ext_data.vm_id) {
		obj->vm = i915_address_space_lookup(file->driver_priv,
						    ext_data.vm_id);
		if (unlikely(!obj->vm)) {
			ret = -ENOENT;
			goto object_free;
		}
	}

	if (!placements_ext) {
		struct intel_memory_region *mr =
			intel_memory_region_by_type(i915, INTEL_MEMORY_SYSTEM);

		object_set_placements(obj, &mr, 1);
	}

	ret = setup_object(obj, args->size, NULL);
	if (ret)
		goto vm_put;

	ret = attach_vm(obj);
	if (ret)
		goto obj_put;

	/* Add any flag set by create_ext options */
	obj->flags |= ext_data.flags;

	return i915_gem_publish(obj, file, &args->size, &args->handle);

vm_put:
	if (obj->vm)
		i915_vm_put(obj->vm);
object_free:
	if (obj->mm.n_placements > 1)
		kfree(placements_ext);
	i915_gem_object_free(obj);
	return ret;

obj_put:
	i915_gem_object_put(obj);
	return ret;
}

/*
 * Creates a new object using the similar path as DRM_I915_GEM_CREATE_EXT.
 * This function is exposed primarily for selftests
 * It is assumed that the set of placement regions has already been verified
 * to be valid.
 */
struct drm_i915_gem_object *
i915_gem_object_create_user(struct drm_i915_private *i915, u64 size,
			    struct intel_memory_region **placements,
			    unsigned int n_placements)
{
	struct drm_i915_gem_object *obj;
	int ret;

	obj = i915_gem_object_alloc();
	if (!obj)
		return ERR_PTR(-ENOMEM);

	if (n_placements > 1) {
		struct intel_memory_region **tmp;

		tmp = kmalloc_array(n_placements, sizeof(*tmp), GFP_KERNEL);
		if (!tmp) {
			ret = -ENOMEM;
			goto object_free;
		}

		memcpy(tmp, placements, sizeof(*tmp) * n_placements);
		placements = tmp;
	}

	object_set_placements(obj, placements, n_placements);
	ret = setup_object(obj, size, NULL);
	if (ret)
		goto placement_free;

	return obj;

placement_free:
	if (n_placements > 1)
		kfree(placements);

object_free:
	i915_gem_object_free(obj);
	return ERR_PTR(ret);
}
