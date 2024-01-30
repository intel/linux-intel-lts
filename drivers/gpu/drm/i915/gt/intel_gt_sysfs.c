// SPDX-License-Identifier: MIT
/*
 * Copyright © 2020 Intel Corporation
 */

#include <drm/drm_device.h>
#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/printk.h>
#include <linux/sysfs.h>

#include "i915_drv.h"
#include "i915_sysfs.h"
#include "intel_gt.h"
#include "intel_gt_types.h"
#include "intel_rc6.h"

#include "intel_sysfs_mem_health.h"
#include "intel_gt_sysfs.h"
#include "intel_gt_sysfs_pm.h"
#include "sysfs_gt_errors.h"

typedef ssize_t (*show)(struct device *dev, struct device_attribute *attr, char *buf);

struct i915_ext_attr {
	struct device_attribute attr;
	show i915_show;
};

static ssize_t
i915_sysfs_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t value;
	struct i915_ext_attr *ea = container_of(attr, struct i915_ext_attr, attr);
	struct intel_gt *gt = intel_gt_sysfs_get_drvdata(dev, attr->attr.name);

	pvc_wa_disallow_rc6(gt->i915);

	value = ea->i915_show(dev, attr, buf);

	pvc_wa_allow_rc6(gt->i915);

	return value;
}

#define I915_DEVICE_ATTR_RO(_name, _show) \
	struct i915_ext_attr dev_attr_##_name = \
	{ __ATTR(_name, 0444, i915_sysfs_show, NULL), _show}

struct intel_gt *intel_gt_sysfs_get_drvdata(struct device *dev,
					    const char *name)
{
	struct kobject *kobj = &dev->kobj;

	/*
	 * We are interested at knowing from where the interface
	 * has been called, whether it's called from gt/ or from
	 * the parent directory.
	 * From the interface position it depends also the value of
	 * the private data.
	 * If the interface is called from gt/ then private data is
	 * of the "struct intel_gt *" type, otherwise it's * a
	 * "struct drm_i915_private *" type.
	 */
	if (!is_object_gt(kobj)) {
		struct drm_i915_private *i915 = kdev_minor_to_i915(dev);

		pr_devel_ratelimited(DEPRECATED
			"%s (pid %d) is trying to access deprecated %s "
			"sysfs control, please use use gt/gt<n>/%s instead\n",
			current->comm, task_pid_nr(current), name, name);
		return to_gt(i915);
	}

	return kobj_to_gt(kobj);
}

static ssize_t
addr_range_show(struct device *kdev, struct device_attribute *attr, char *buf)
{
	struct intel_gt *gt = kobj_to_gt(&kdev->kobj);

	return sysfs_emit(buf, "%pa\n", &gt->lmem->actual_physical_mem);
}

static I915_DEVICE_ATTR_RO(addr_range, addr_range_show);

static const struct attribute *addr_range_attrs[] = {
	/* TODO: Report any other HBM Sparing sysfs per gt? */
	&dev_attr_addr_range.attr.attr,
	NULL
};

void intel_gt_sysfs_register_mem(struct intel_gt *gt, struct kobject *parent)
{
	if (!HAS_MEM_SPARING_SUPPORT(gt->i915))
		return;

	if (sysfs_create_files(parent, addr_range_attrs))
		drm_err(&gt->i915->drm, "Setting up sysfs to read total physical memory per tile failed\n");
}

static struct kobject *gt_get_parent_obj(struct intel_gt *gt)
{
	return &gt->i915->drm.primary->kdev->kobj;
}

int intel_gt_sysfs_reset(struct intel_gt *gt)
{
	/* Check if already wedged, if not then reset */
	if (intel_gt_terminally_wedged(gt))
		return -EIO;

	/* The string is appended to "Resetting chip for..." */
	intel_gt_handle_error(gt, ALL_ENGINES, I915_ERROR_CAPTURE,
			      "GT%u manually from sysfs", gt->info.id);

	return 0;
}

static ssize_t prelim_reset_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct intel_gt *gt = intel_gt_sysfs_get_drvdata(dev, attr->attr.name);
	bool val;
	int ret;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	if (!val)
		return count;

	return intel_gt_sysfs_reset(gt) ?: count;
}

static DEVICE_ATTR_WO(prelim_reset);

static ssize_t id_show(struct device *dev,
		       struct device_attribute *attr,
		       char *buf)
{
	struct intel_gt *gt = intel_gt_sysfs_get_drvdata(dev, attr->attr.name);

	return sysfs_emit(buf, "%u\n", gt->info.id);
}

static I915_DEVICE_ATTR_RO(id, id_show);

static void kobj_gt_release(struct kobject *kobj)
{
	kfree(kobj);
}

static struct kobj_type kobj_gt_type = {
	.release = kobj_gt_release,
	.sysfs_ops = &kobj_sysfs_ops
};

struct kobject *
intel_gt_create_kobj(struct intel_gt *gt, struct kobject *dir, const char *name)
{
	struct kobj_gt *kg;

	kg = kzalloc(sizeof(*kg), GFP_KERNEL);
	if (!kg)
		return NULL;

	kobject_init(&kg->base, &kobj_gt_type);
	kg->gt = gt;

	/* xfer ownership to sysfs tree */
	if (kobject_add(&kg->base, dir, "%s", name)) {
		kobject_put(&kg->base);
		return NULL;
	}

	return &kg->base; /* borrowed ref */
}

void intel_gt_sysfs_register(struct intel_gt *gt)
{
	struct kobject *dir;
	char name[80];

	/*
	 * We need to make things right with the
	 * ABI compatibility. The files were originally
	 * generated under the parent directory.
	 *
	 * We generate the files only for gt 0
	 * to avoid duplicates.
	 */
	if (!gt->info.id)
		intel_gt_sysfs_pm_init(gt, gt_get_parent_obj(gt));

	snprintf(name, sizeof(name), "gt%d", gt->info.id);

	dir = intel_gt_create_kobj(gt, gt->i915->sysfs_gt, name);
	if (!dir) {
		drm_err(&gt->i915->drm,
			"failed to initialize %s sysfs root\n", name);
		return;
	}

	gt->sysfs_defaults = kobject_create_and_add(".defaults", dir);
	if (!gt->sysfs_defaults) {
		drm_err(&gt->i915->drm, "failed to create gt sysfs .defaults\n");
		return;
	}

	if (sysfs_create_file(dir, &dev_attr_id.attr.attr))
		drm_err(&gt->i915->drm,
			"failed to create sysfs %s info files\n", name);

	if (sysfs_create_file(dir, &dev_attr_prelim_reset.attr))
		drm_warn(&gt->i915->drm,
			 "failed to create sysfs %s reset files\n", name);

	intel_gt_sysfs_pm_init(gt, dir);
	intel_gt_sysfs_register_errors(gt, dir);
	intel_gt_sysfs_register_mem(gt, dir);
}

void intel_gt_sysfs_unregister(struct intel_gt *gt)
{
}
