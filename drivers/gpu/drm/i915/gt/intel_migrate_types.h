/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2020 Intel Corporation
 */

#ifndef __INTEL_MIGRATE_TYPES__
#define __INTEL_MIGRATE_TYPES__

#include <drm/drm_mm.h>

struct drm_i915_gem_object;
struct intel_context;

struct intel_migrate {
	struct intel_context *context;

	struct intel_migrate_window {
		struct drm_mm_node node;
		struct drm_i915_gem_object *obj;

		unsigned long clear_chunk;
		unsigned long swap_chunk;

		uint64_t pd_offset;
	} window;
};

#endif /* __INTEL_MIGRATE_TYPES__ */
