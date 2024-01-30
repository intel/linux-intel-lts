/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2020 Intel Corporation
 */

#ifndef INTEL_CLOS_H
#define INTEL_CLOS_H

#include <linux/types.h>

struct drm_i915_private;
struct drm_i915_file_private;

void init_device_clos(struct drm_i915_private *dev_priv);
void uninit_device_clos(struct drm_i915_private *dev_priv);

void init_client_clos(struct drm_i915_file_private *fpriv);
void uninit_client_clos(struct drm_i915_file_private *fpriv);

int reserve_clos(struct drm_i915_file_private *fpriv, u16 *clos_index);
int free_clos(struct drm_i915_file_private *fpriv, u16 clos_index);
int reserve_cache_ways(struct drm_i915_file_private *fpriv, u16 cache_level,
			u16 clos_index, u16 *num_ways);
#endif
