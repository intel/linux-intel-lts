/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2019 Intel Corporation
 */

#ifndef __I915_GEM_MMAN_H__
#define __I915_GEM_MMAN_H__

#include <linux/mm_types.h>
#include <linux/types.h>

#include "gem/i915_gem_object_types.h"

struct drm_device;
struct drm_file;
struct drm_i915_gem_object;
struct file;
struct i915_mmap_offset;
struct mutex;

int i915_gem_mmap_gtt_version(void);
int i915_gem_mmap(struct file *filp, struct vm_area_struct *vma);

int i915_gem_dumb_mmap_offset(struct drm_file *file_priv,
			      struct drm_device *dev,
			      u32 handle, u64 *offset);

void i915_gem_object_release_mmap(struct drm_i915_gem_object *obj);

void __i915_gem_object_release_mmap_gtt(struct drm_i915_gem_object *obj);
void i915_gem_object_release_mmap_gtt(struct drm_i915_gem_object *obj);

void i915_gem_object_release_mmap_offset(struct drm_i915_gem_object *obj);

vm_fault_t i915_error_to_vmf_fault(int err);

struct i915_mmap_offset *
i915_gem_mmap_offset_attach(struct drm_i915_gem_object *obj,
			    enum i915_mmap_type mmap_type,
			    struct drm_file *file);
int i915_gem_update_vma_info(struct drm_i915_gem_object *obj,
			     struct i915_mmap_offset *mmo,
			     struct vm_area_struct *vma);
#endif
