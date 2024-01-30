/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2019 - 2022 Intel Corporation
 */

#ifndef _INTEL_IAF_H_
#define _INTEL_IAF_H_

/*
 * Define the maximum number of devices instances based on the amount of
 * FID space.
 *
 * XARRAY limits are "inclusive", but using this value as a range check
 * outside of xarray, makes the exclusive upper bound a little easier to
 * deal with.
 *
 * I.e.:
 * [0 - 256)
 *
 * Less than HW supports, but more than will be currently possible.
 *
 */
#define MAX_DEVICE_COUNT 256

/* Fixed Device Physical Address (DPA) size for a device/package (in GB) */
#define MAX_DPA_SIZE 128

struct drm_i915_private;

void intel_iaf_init_early(struct drm_i915_private *i915);
void intel_iaf_init_mmio(struct drm_i915_private *i915);
void intel_iaf_init(struct drm_i915_private *i915);
#if IS_ENABLED(CONFIG_AUXILIARY_BUS)
void intel_iaf_init_aux(struct drm_i915_private *i915);
#else
void intel_iaf_init_mfd(struct drm_i915_private *i915);
#endif
void intel_iaf_remove(struct drm_i915_private *i915);
int intel_iaf_pcie_error_notify(struct drm_i915_private *i915);
int intel_iaf_mapping_get(struct drm_i915_private *i915);
int intel_iaf_mapping_put(struct drm_i915_private *i915);

#endif
