/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2021 Intel Corporation
 */

#ifndef __I915_DEBUGGER_H__
#define __I915_DEBUGGER_H__

#include "i915_debugger_types.h"

struct drm_device;
struct drm_file;
struct i915_drm_client;
struct i915_gem_context;
struct i915_uuid_resource;
struct i915_address_space;
struct i915_vma;
struct intel_engine_cs;
struct intel_context;
struct intel_gt;

#if IS_ENABLED(CONFIG_DRM_I915_DEBUGGER)

int i915_debugger_open_ioctl(struct drm_device *dev, void *data,
			     struct drm_file *file);

void i915_debugger_init(struct drm_i915_private *i915);
void i915_debugger_fini(struct drm_i915_private *i915);

void i915_debugger_wait_on_discovery(struct drm_i915_private *i915,
				     struct i915_drm_client *client);

void i915_debugger_client_release(struct i915_drm_client *client);

void i915_debugger_client_create(struct i915_drm_client *client);
void i915_debugger_client_destroy(struct i915_drm_client *client);

void i915_debugger_context_create(struct i915_gem_context *ctx);
void i915_debugger_context_destroy(struct i915_gem_context *ctx);

void i915_debugger_uuid_create(struct i915_drm_client *client,
			       struct i915_uuid_resource *uuid);

void i915_debugger_uuid_destroy(struct i915_drm_client *client,
				struct i915_uuid_resource *uuid);

void i915_debugger_vm_create(struct i915_drm_client *client,
			     struct i915_address_space *vm);
void i915_debugger_vm_destroy(struct i915_drm_client *client,
			      struct i915_address_space *vm);

void i915_debugger_vma_insert(struct i915_drm_client *client,
			      struct i915_vma *vma);
void i915_debugger_vma_evict(struct i915_drm_client *client,
			     struct i915_vma *vma);
void i915_debugger_vma_prepare(struct i915_drm_client *client,
			       struct i915_vma *vma);
void i915_debugger_vma_finalize(struct i915_drm_client *client,
				struct i915_vma *vma,
				int error);

void i915_debugger_context_param_vm(const struct i915_drm_client *client,
				    struct i915_gem_context *ctx,
				    struct i915_address_space *vm);

void i915_debugger_context_param_engines(struct i915_gem_context *ctx);

int i915_debugger_handle_engine_attention(struct intel_engine_cs *engine);

int i915_debugger_handle_engine_page_fault(struct intel_engine_cs *engine,
					   struct i915_debugger_pagefault *pagefault);
int i915_debugger_add_pagefault_list(struct intel_engine_cs *engine,
				     struct i915_debugger_pagefault *pagefault);

bool i915_debugger_prevents_hangcheck(struct intel_engine_cs *engine);
bool i915_debugger_active_on_context(struct intel_context *context);
bool i915_debugger_active_on_engine(struct intel_engine_cs *engine);

bool i915_debugger_context_guc_debugged(struct intel_context *context);

long i915_debugger_attention_poll_interval(struct intel_engine_cs *engine);

int i915_debugger_enable(struct drm_i915_private *i915, bool enable);
int i915_debugger_allow(struct drm_i915_private *i915);
int i915_debugger_disallow(struct drm_i915_private *i915);

void i915_debugger_gpu_flush_engines(struct intel_gt *gt, u32 mask);

#else /* CONFIG_DRM_I915_DEBUGGER */

static inline int i915_debugger_open_ioctl(struct drm_device *dev, void *data,
					   struct drm_file *file)
{
	return -ENOTSUPP;
}

static inline void i915_debugger_init(struct drm_i915_private *i915) { }
static inline void i915_debugger_fini(struct drm_i915_private *i915) { }

static inline void i915_debugger_wait_on_discovery(struct drm_i915_private *i915,
						   struct i915_drm_client *client) { }

static inline void i915_debugger_client_release(struct i915_drm_client *client) { }

static inline void i915_debugger_client_create(struct i915_drm_client *client) { }
static inline void i915_debugger_client_destroy(struct i915_drm_client *client) { }

static inline void i915_debugger_context_create(struct i915_gem_context *ctx) { }
static inline void i915_debugger_context_destroy(struct i915_gem_context *ctx) { }

static inline void i915_debugger_uuid_create(struct i915_drm_client *client,
					     struct i915_uuid_resource *uuid) { }
static inline void i915_debugger_uuid_destroy(struct i915_drm_client *client,
					       struct i915_uuid_resource *uuid) { }

static inline void i915_debugger_vm_create(struct i915_drm_client *client,
					   struct i915_address_space *vm) { }
static inline void i915_debugger_vm_destroy(struct i915_drm_client *client,
					    struct i915_address_space *vm) { }

static inline void i915_debugger_vma_insert(struct i915_drm_client *client,
					    struct i915_vma *vma) { }
static inline void i915_debugger_vma_evict(struct i915_drm_client *client,
					   struct i915_vma *vma) { }

static inline void i915_debugger_vma_prepare(struct i915_drm_client *client,
					     struct i915_vma *vma) { }
static inline void i915_debugger_vma_finalize(struct i915_drm_client *client,
					      struct i915_vma *vma,
					      int error) { }

static inline void i915_debugger_context_param_vm(const struct i915_drm_client *client,
						  struct i915_gem_context *ctx,
						  struct i915_address_space *vm) { }

static inline void i915_debugger_context_param_engines(struct i915_gem_context *ctx) { }

static inline int
i915_debugger_handle_engine_attention(struct intel_engine_cs *engine)
{
	return 0;
}

static inline int
i915_debugger_handle_engine_page_fault(struct intel_engine_cs *engine,
				       struct i915_debugger_pagefault *pagefault)
{
	return 0;
}

static inline int
i915_debugger_add_pagefault_list(struct intel_engine_cs *engine,
				 struct i915_debugger_pagefault *pagefault)
{
	return 0;
}

static inline bool
i915_debugger_prevents_hangcheck(struct intel_engine_cs *engine)
{
	return false;
}

static inline bool i915_debugger_active_on_context(struct intel_context *context)
{
	return false;
}

static inline bool i915_debugger_active_on_engine(struct intel_engine_cs *engine)
{
	return false;
}

static inline bool i915_debugger_context_guc_debugged(struct intel_context *context)
{
	return false;
}

static inline long
i915_debugger_attention_poll_interval(struct intel_engine_cs *engine)
{
	return 0;
}

static inline int
i915_debugger_allow(struct drm_i915_private *i915)
{
	return 0;
}

static inline int
i915_debugger_disallow(struct drm_i915_private *i915)
{
	return 0;
}

static inline void i915_debugger_gpu_flush_engines(struct intel_gt *gt,
						   u32 mask) { }

#endif /* CONFIG_DRM_I915_DEBUGGER */

#endif /* __I915_DEBUGGER_H__ */
