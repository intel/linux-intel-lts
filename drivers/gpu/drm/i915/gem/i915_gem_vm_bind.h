/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2019 Intel Corporation
 */

#ifndef __I915_GEM_VM_BIND_H
#define __I915_GEM_VM_BIND_H

#include "i915_drv.h"

#define assert_vm_bind_held(vm)   lockdep_assert_held(&(vm)->vm_bind_lock)

static inline void i915_gem_vm_bind_lock(struct i915_address_space *vm)
{
	mutex_lock(&vm->vm_bind_lock);
}

static inline int
i915_gem_vm_bind_lock_interruptible(struct i915_address_space *vm)
{
	return mutex_lock_interruptible(&vm->vm_bind_lock);
}

static inline void i915_gem_vm_bind_unlock(struct i915_address_space *vm)
{
	mutex_unlock(&vm->vm_bind_lock);
}

struct i915_vma *
i915_gem_vm_bind_lookup_vma(struct i915_address_space *vm, u64 va);
int i915_gem_vm_bind_obj(struct i915_address_space *vm,
			 struct prelim_drm_i915_gem_vm_bind *va,
			 struct drm_file *file);
int i915_gem_vm_unbind_obj(struct i915_address_space *vm,
			   struct prelim_drm_i915_gem_vm_bind *va);

void i915_gem_vm_unbind_all(struct i915_address_space *vm);

void i915_vma_metadata_free(struct i915_vma *vma);

#endif /* __I915_GEM_VM_BIND_H */
