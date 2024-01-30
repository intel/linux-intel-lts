/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2014-2018 Intel Corporation
 */

#ifndef _INTEL_WORKAROUNDS_H_
#define _INTEL_WORKAROUNDS_H_

#include <linux/slab.h>

#include "intel_workarounds_types.h"

struct drm_i915_private;
struct drm_printer;
struct i915_request;
struct intel_engine_cs;
struct intel_gt;

static inline void intel_wa_list_free(struct i915_wa_list *wal)
{
	kfree(wal->list);
	memset(wal, 0, sizeof(*wal));
}

void intel_engine_init_ctx_wa(struct intel_engine_cs *engine);
int intel_engine_emit_ctx_wa(struct i915_request *rq);

void intel_gt_init_workarounds(struct intel_gt *gt);
void intel_gt_apply_workarounds(struct intel_gt *gt);
int intel_gt_show_workarounds(struct drm_printer *p,
			      struct intel_gt *gt,
			      const struct i915_wa_list * const wal);
bool intel_gt_verify_workarounds(struct intel_gt *gt, const char *from);

void intel_engine_init_whitelist(struct intel_engine_cs *engine);
void intel_engine_apply_whitelist(struct intel_engine_cs *engine);

void intel_engine_init_workarounds(struct intel_engine_cs *engine);
void intel_engine_apply_workarounds(struct intel_engine_cs *engine);
int intel_engine_show_workarounds(struct drm_printer *m,
				  struct intel_engine_cs *engine,
				  const struct i915_wa_list * const wal);
int intel_engine_verify_workarounds(struct intel_engine_cs *engine,
				    const char *from);

void intel_engine_allow_user_register_access(struct intel_engine_cs *engine,
					     struct i915_whitelist_reg *reg,
					     u32 count);
void intel_engine_deny_user_register_access(struct intel_engine_cs *engine,
					    struct i915_whitelist_reg *reg,
					    u32 count);

void intel_engine_debug_enable(struct intel_engine_cs *engine);
void intel_engine_debug_disable(struct intel_engine_cs *engine);

#endif
