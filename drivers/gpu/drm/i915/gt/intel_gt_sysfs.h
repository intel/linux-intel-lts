// SPDX-License-Identifier: MIT
/*
 * Copyright © 2020 Intel Corporation
 */

#ifndef __SYSFS_GT_H__
#define __SYSFS_GT_H__

#include <linux/ctype.h>
#include <linux/kobject.h>

#include "i915_gem.h" /* GEM_BUG_ON() */

struct intel_gt;

struct kobj_gt {
	struct kobject base;
	struct intel_gt *gt;
};

static inline bool is_object_gt(struct kobject *kobj)
{
	bool b = !strncmp(kobj->name, "gt", 2);

	GEM_BUG_ON(b && !isdigit(kobj->name[2]));

	return b;
}

struct kobject *
intel_gt_create_kobj(struct intel_gt *gt,
		     struct kobject *dir,
		     const char *name);

static inline struct intel_gt *kobj_to_gt(struct kobject *kobj)
{
	return container_of(kobj, struct kobj_gt, base)->gt;
}

void intel_gt_sysfs_register(struct intel_gt *gt);
void intel_gt_sysfs_unregister(struct intel_gt *gt);
struct intel_gt *intel_gt_sysfs_get_drvdata(struct device *dev,
					    const char *name);

int intel_gt_sysfs_reset(struct intel_gt *gt);

#endif /* SYSFS_GT_H */
