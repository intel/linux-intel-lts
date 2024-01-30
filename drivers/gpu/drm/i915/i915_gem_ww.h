/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2020 Intel Corporation
 */
#ifndef __I915_GEM_WW_H__
#define __I915_GEM_WW_H__

#include <drm/drm_drv.h>

struct i915_gem_ww_ctx {
	struct ww_acquire_ctx ctx;
	struct list_head obj_list;
	struct drm_i915_gem_object *contended;
	bool intr:1;
	bool loop:1;
	bool contended_evict:1;
};

void i915_gem_ww_ctx_init(struct i915_gem_ww_ctx *ctx, bool intr);
void i915_gem_ww_ctx_fini(struct i915_gem_ww_ctx *ctx);
int __must_check i915_gem_ww_ctx_backoff(struct i915_gem_ww_ctx *ctx);
void i915_gem_ww_unlock_single(struct drm_i915_gem_object *obj);

int
__i915_gem_object_lock_to_evict(struct drm_i915_gem_object *obj,
				struct i915_gem_ww_ctx *ww);

/* Internal functions used by the inlines! Don't use. */
static inline int __i915_gem_ww_fini(struct i915_gem_ww_ctx *ww, int err)
{
	ww->loop = 0;
	if (err == -EDEADLK) {
		err = i915_gem_ww_ctx_backoff(ww);
		if (!err)
			ww->loop = 1;
	}

	if (!ww->loop)
		i915_gem_ww_ctx_fini(ww);

	return err;
}

static inline void
__i915_gem_ww_init(struct i915_gem_ww_ctx *ww, bool intr)
{
	i915_gem_ww_ctx_init(ww, intr);
	ww->loop = 1;
}

#define for_i915_gem_ww(_ww, _err, _intr)			\
	for (__i915_gem_ww_init(_ww, _intr); (_ww)->loop;	\
	     _err = __i915_gem_ww_fini(_ww, _err))

#endif
