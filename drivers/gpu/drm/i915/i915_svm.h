/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2019 Intel Corporation
 */

#ifndef __I915_SVM_H
#define __I915_SVM_H

#include <linux/hmm.h>

#include "i915_drv.h"
#include "gt/intel_pagefault.h"

#if defined(CONFIG_DRM_I915_SVM)
struct i915_devmem {
	struct intel_memory_region *mem;
	struct dev_pagemap pagemap;
	unsigned long pfn_first;
	unsigned long pfn_last;
};

struct i915_svm {
	/* i915 address space */
	struct i915_address_space *vm;

	struct mm_struct *mm;
	struct mutex mutex; /* protects svm operations */
	struct kref ref;
};

void i915_svm_unbind_mm(struct i915_address_space *vm);
int i915_svm_bind_mm(struct i915_address_space *vm);
static inline bool i915_vm_is_svm_enabled(struct i915_address_space *vm)
{
	return vm->svm;
}

int i915_svm_copy_blt(struct intel_context *ce,
		      struct i915_gem_ww_ctx *ww,
		      u64 src_start, u64 dst_start, u64 size,
		      struct dma_fence **fence);

int i915_dmem_convert_pfn(struct drm_i915_private *dev_priv,
			  struct hmm_range *range);
int i915_svm_vm_prefetch(struct drm_i915_private *i915,
			struct prelim_drm_i915_gem_vm_prefetch *args);
int i915_svm_devmem_add(struct intel_memory_region *mem);
void i915_svm_devmem_remove(struct intel_memory_region *mem);

int i915_svm_handle_gpu_fault(struct i915_address_space *vm,
				struct intel_gt *gt,
				struct recoverable_page_fault_info *info);

int i915_devmem_migrate_vma(struct intel_memory_region *mem,
				   struct i915_gem_ww_ctx *ww,
				   struct vm_area_struct *vma,
				   unsigned long start,
				   unsigned long end);
#else
struct i915_devmem { };
struct i915_svm { };
static inline void i915_svm_unbind_mm(struct i915_address_space *vm) { }
static inline int i915_svm_bind_mm(struct i915_address_space *vm)
{ return -EOPNOTSUPP; }
static inline bool i915_vm_is_svm_enabled(struct i915_address_space *vm)
{ return false; }

static inline int i915_svm_vm_prefetch(struct drm_i915_private *i915,
			struct prelim_drm_i915_gem_vm_prefetch *args)
{ return -EOPNOTSUPP; }
static inline int i915_svm_devmem_add(struct intel_memory_region *mem)
{ return 0; }
static inline void i915_svm_devmem_remove(struct intel_memory_region *mem) { }
static inline int i915_svm_handle_gpu_fault(struct i915_address_space *vm,
				struct intel_gt *gt,
				struct recoverable_page_fault_info *info)
{ return 0; }

static inline int i915_devmem_migrate_vma(struct intel_memory_region *mem,
				   struct i915_gem_ww_ctx *ww,
				   struct vm_area_struct *vma,
				   unsigned long start,
				   unsigned long end)
{ return 0; }
#endif

#endif /* __I915_SVM_H */
