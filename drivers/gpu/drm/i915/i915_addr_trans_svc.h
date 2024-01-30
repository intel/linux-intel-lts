/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2022 Intel Corporation
 */

#ifndef __I915_ADDR_TRANS_SVC_H__
#define __I915_ADDR_TRANS_SVC_H__

#include <linux/hmm.h>
#include <linux/intel-iommu.h>
#include <linux/pci-ats.h>
#include "../../../../drivers/iommu/iommu-sva-lib.h"
#include "i915_svm.h"
#include "gt/intel_pagefault.h"
#include "i915_drv.h"

#include "gt/intel_gtt.h"

#if  IS_ENABLED(CONFIG_DRM_I915_ATS)
/* TODO: Private structure for ATS - need expansion */
struct i915_ats_priv {
	struct drm_i915_private *i915;
	struct device_domain_info *ats_info;
};

static inline int i915_get_pasid(struct iommu_sva *sva)
{
	return iommu_sva_get_pasid(sva);
}

void i915_enable_ats(struct drm_i915_private *i915);
void i915_disable_ats(struct drm_i915_private *i915);
bool i915_ats_enabled(struct drm_i915_private *dev_priv);
int i915_create_pasid(struct i915_address_space *vm);
void i915_destroy_pasid(struct i915_address_space *vm);
bool is_vm_pasid_active(struct i915_address_space *vm);
int i915_global_pasid_counter(struct drm_i915_private *i915);
int i915_handle_ats_fault_request(struct i915_address_space *vm,
				  struct recoverable_page_fault_info *info);
void intel_invalidate_devtlb_range(struct i915_address_space *vm,
				   u64 start, u64 size);

#else /* CONFIG_DRM_I915_ATS */
struct i915_ats_priv { };
static inline void i915_enable_ats(struct drm_i915_private *i915) { }
static inline void i915_disable_ats(struct drm_i915_private *i915) { }
static inline bool i915_ats_enabled(struct drm_i915_private *dev_priv)
{ return false; }
static inline int i915_create_pasid(struct i915_address_space *vm)
{ return -EOPNOTSUPP; }
static inline void i915_destroy_pasid(struct i915_address_space *vm) { }
static inline bool is_vm_pasid_active(struct i915_address_space *vm)
{ return false; }
static inline int i915_global_pasid_counter(struct drm_i915_private *i915)
{ return -EOPNOTSUPP; }
static inline int
i915_handle_ats_fault_request(struct i915_address_space *vm,
			      struct recoverable_page_fault_info *info)
{ return -EOPNOTSUPP; }
static inline void intel_invalidate_devtlb_range(struct i915_address_space *vm,
						 u64 start, u64 size){ }
#endif /* CONFIG_DRM_I915_ATS */
#endif /* __I915_ADDR_TRANS_SVC_H__ */
