/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2020 Intel Corporation
 */

#ifndef __I915_DRM_CLIENT_H__
#define __I915_DRM_CLIENT_H__

#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/pid.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/xarray.h>

#include "gt/intel_engine_types.h"

struct drm_i915_private;
struct drm_i915_file_private;

struct i915_drm_clients {
	struct drm_i915_private *i915;

	struct xarray xarray;
	u32 next_id;

	struct kobject *root;
};

struct i915_engine_busy_attribute {
	struct device_attribute attr;
	struct i915_drm_client *client;
	unsigned int engine_class;
};

struct i915_drm_client_name {
	struct rcu_head rcu;
	struct i915_drm_client *client;
	struct pid *pid;
	char name[];
};

struct i915_drm_client {
	struct kref kref;

	struct rcu_work rcu;

	struct mutex update_lock; /* Serializes name and pid updates. */

	struct drm_i915_file_private *file;

	unsigned int id;
	struct i915_drm_client_name __rcu *name;
	bool closed;

	spinlock_t ctx_lock; /* For add/remove from ctx_list. */
	struct list_head ctx_list; /* List of contexts belonging to client. */

	struct i915_drm_clients *clients;

	struct kobject *root;
	struct kobject *busy_root;
	struct kobject *devm_stats_root;
	struct {
		struct device_attribute pid;
		struct device_attribute name;
		struct device_attribute created_devm_bytes;
		struct device_attribute resident_created_devm_bytes;
		struct device_attribute imported_devm_bytes;
		struct device_attribute resident_imported_devm_bytes;
		struct i915_engine_busy_attribute busy[MAX_ENGINE_CLASS + 1];
	} attr;

	/**
	 * @past_runtime: Accumulation of pphwsp runtimes from closed contexts.
	 */
	atomic64_t past_runtime[MAX_ENGINE_CLASS + 1];
	atomic64_t created_devm_bytes;
	atomic64_t resident_created_devm_bytes;
	atomic64_t imported_devm_bytes;
	atomic64_t resident_imported_devm_bytes;

	/*
	 * A placeholder for all UUID Resources defined and registered for
	 * a single client.
	 */
	struct xarray uuids_xa;

#if IS_ENABLED(CONFIG_DRM_I915_DEBUGGER)
	u64 debugger_session;
#endif
};

void i915_drm_clients_init(struct i915_drm_clients *clients,
			   struct drm_i915_private *i915);

static inline struct i915_drm_client *
i915_drm_client_get(struct i915_drm_client *client)
{
	kref_get(&client->kref);
	return client;
}

static inline struct i915_drm_client *
i915_drm_client_get_rcu(struct i915_drm_client *client)
{
	RCU_LOCKDEP_WARN(!rcu_read_lock_held(),
			 "suspicious i915_drm_client_get_rcu() usage");

	if (client && !kref_get_unless_zero(&client->kref))
		client = NULL;

	return client;
}

void __i915_drm_client_free(struct kref *kref);

static inline void i915_drm_client_put(struct i915_drm_client *client)
{
	kref_put(&client->kref, __i915_drm_client_free);
}

void i915_drm_client_close(struct i915_drm_client *client);
void i915_drm_client_cleanup(struct i915_drm_client *client);

struct i915_drm_client *i915_drm_client_add(struct i915_drm_clients *clients,
					    struct task_struct *task,
					    struct drm_i915_file_private *file);

int i915_drm_client_update(struct i915_drm_client *client,
			   struct task_struct *task);

void i915_drm_client_init_bo(struct drm_i915_gem_object *obj);
int i915_drm_client_add_bo(struct i915_drm_client *client,
			   struct drm_i915_gem_object *obj);
void i915_drm_client_del_bo(struct i915_drm_client *client,
			    struct drm_i915_gem_object *obj);
void i915_drm_client_make_resident(struct drm_i915_gem_object *obj,
				   bool resident);
void i915_drm_client_fini_bo(struct drm_i915_gem_object *obj);

static inline const struct i915_drm_client_name *
__i915_drm_client_name(const struct i915_drm_client *client)
{
	return rcu_dereference(client->name);
}

static inline const char *
i915_drm_client_name(const struct i915_drm_client *client)
{
	return __i915_drm_client_name(client)->name;
}

static inline struct pid *
i915_drm_client_pid(const struct i915_drm_client *client)
{
	return __i915_drm_client_name(client)->pid;
}

void i915_drm_clients_fini(struct i915_drm_clients *clients);

#endif /* !__I915_DRM_CLIENT_H__ */
