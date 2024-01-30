/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2020 Intel Corporation
 */

#ifndef SELFTEST_LIBRAPL_H
#define SELFTEST_LIBRAPL_H

#include <linux/types.h>

struct drm_i915_private;

u64 librapl_energy_uJ(struct drm_i915_private *i915);

static inline bool librapl_supported(struct drm_i915_private *i915)
{
	return librapl_energy_uJ(i915);
}

#endif /* SELFTEST_LIBRAPL_H */
