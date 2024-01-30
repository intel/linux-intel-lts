// SPDX-License-Identifier: MIT
/*
 * Copyright © 2020 Intel Corporation
 */

#include <linux/kobject.h>
#include <linux/sysfs.h>

#include "gt/intel_gt_sysfs.h"

#include "i915_drv.h"
#include "i915_sysfs.h"
#include "intel_sysfs_mem_health.h"

#include "gt/intel_gt.h"
#include "gt/intel_gt_requests.h"

static ssize_t
i915_sysfs_show(struct device *dev, struct device_attribute *attr, char *buf);

typedef ssize_t (*show)(struct device *dev, struct device_attribute *attr, char
			*buf);
typedef ssize_t (*store)(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count);

struct i915_ext_attr {
	struct device_attribute attr;
	show i915_show;
	store i915_store;
};

static const char *
memory_error_to_str(const struct intel_mem_sparing_event *mem)
{
	switch (mem->health_status) {
	case MEM_HEALTH_ALARM:
		return "MEMORY_HEALTH_ALARM";
	case MEM_HEALTH_EC_PENDING:
		return "EC_PENDING";
	case MEM_HEALTH_DEGRADED:
		return "DEGRADED";
	case MEM_HEALTH_UNKNOWN:
		return "MEMORY_HEALTH_UNKNOWN";
	case MEM_HEALTH_OKAY:
	default:
		return "OK";
	}
}

#define I915_DEVICE_ATTR_RO(_name, _show) \
	struct i915_ext_attr dev_attr_##_name = \
	{ __ATTR(_name, 0444, i915_sysfs_show, NULL), _show, NULL}

static ssize_t
device_memory_health_show(struct device *kdev, struct device_attribute *attr,
			  char *buf)
{
	struct drm_i915_private *i915 = kdev_minor_to_i915(kdev);
	const char *mem_status;

	mem_status = memory_error_to_str(&to_gt(i915)->mem_sparing);
	return sysfs_emit(buf, "%s\n", mem_status);
}

static I915_DEVICE_ATTR_RO(device_memory_health, device_memory_health_show);

static const struct attribute *mem_health_attrs[] = {
	&dev_attr_device_memory_health.attr.attr,
	NULL
};

static ssize_t
i915_sysfs_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t value;
	struct i915_ext_attr *ea = container_of(attr, struct i915_ext_attr, attr);
	struct drm_i915_private *i915 = kdev_minor_to_i915(dev);

	pvc_wa_disallow_rc6(i915);

	value = ea->i915_show(dev, attr, buf);

	pvc_wa_allow_rc6(i915);

	return value;
}

void intel_mem_health_report_sysfs(struct drm_i915_private *i915)
{
	struct device *kdev = i915->drm.primary->kdev;

	if (!HAS_MEM_SPARING_SUPPORT(i915))
		return;

	if (sysfs_create_files(&kdev->kobj, mem_health_attrs)) {
		dev_err(kdev, "Failed to add sysfs files to show memory health status\n");
		return;
	}
}
