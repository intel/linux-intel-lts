/*
 * Copyright (C) 2017 Intel, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/acpi.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/sysfs.h>

struct android_fwdata_state {
	struct device *dev;
	struct kobject *properties_kobj;
	struct kobject *android_kobj;
	struct kobject *fstab_kobj;
	struct kobject *system_kobj;
	struct kobject *vendor_kobj;
};

static struct android_fwdata_state state;

/* Called when <sysfs_device>/properties/<subpath>/<attr> is read. */
static ssize_t property_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	const char *prefix = NULL;
	char key[128];
	const char *value = NULL;
	int ret;

	/* It would be much more convenient if show() gave us the relative path
	 * to the file being read, e.g. properties/android/fstab/system/dev,
	 * which could be easily converted to a property key.
	 * TODO: Infer the relative path from kobj and remove all hard-coded
	 * property keys.
	 */
	if (kobj == state.android_kobj) {
		prefix = "android";
	} else if (kobj == state.fstab_kobj) {
		prefix = "android.fstab";
	} else if (kobj == state.system_kobj) {
		prefix = "android.fstab.system";
	} else if (kobj == state.vendor_kobj) {
		prefix = "android.fstab.vendor";
	} else {
		pr_err("%s: Unexpected folder\n", __func__);
		return -EINVAL;
	}
	/* We don't put any file in properties/ directly, so prefix can't be
	 * empty.
	 */
	snprintf(key, sizeof(key), "%s.%s", prefix, attr->attr.name);

	ret = device_property_read_string(state.dev, key, &value);
	if (ret) {
		pr_err("%s: Failed to read property '%s', ret=%d\n", __func__,
		       key, ret);
		return ret;
	}
	return scnprintf(buf, PAGE_SIZE, "%s\n", value);
}

#define DT_COMPATIBLE_ATTR(_folder) \
	struct kobj_attribute _folder##_compatible_attr = { \
		.attr = { .name = "compatible", .mode = 0444, }, \
		.show = property_show, \
	}

static DT_COMPATIBLE_ATTR(android);
static DT_COMPATIBLE_ATTR(fstab);
static DT_COMPATIBLE_ATTR(system);
static DT_COMPATIBLE_ATTR(vendor);

#define FSTAB_PROPERTY_ATTR(_partition, _name) \
	struct kobj_attribute _partition##_##_name##_attr = { \
		.attr = { \
			.name = __stringify(_name), \
			.mode = 0444, \
		}, \
		.show = property_show, \
	}

static FSTAB_PROPERTY_ATTR(system, dev);
static FSTAB_PROPERTY_ATTR(system, type);
static FSTAB_PROPERTY_ATTR(system, mnt_flags);
static FSTAB_PROPERTY_ATTR(system, fsmgr_flags);

static FSTAB_PROPERTY_ATTR(vendor, dev);
static FSTAB_PROPERTY_ATTR(vendor, type);
static FSTAB_PROPERTY_ATTR(vendor, mnt_flags);
static FSTAB_PROPERTY_ATTR(vendor, fsmgr_flags);

static struct attribute *system_attrs[] = {
	&system_compatible_attr.attr,
	&system_dev_attr.attr,
	&system_type_attr.attr,
	&system_mnt_flags_attr.attr,
	&system_fsmgr_flags_attr.attr,
	NULL,
};

static struct attribute_group system_group = {
	.attrs = system_attrs,
};

static struct attribute *vendor_attrs[] = {
	&vendor_compatible_attr.attr,
	&vendor_dev_attr.attr,
	&vendor_type_attr.attr,
	&vendor_mnt_flags_attr.attr,
	&vendor_fsmgr_flags_attr.attr,
	NULL,
};

static struct attribute_group vendor_group = {
	.attrs = vendor_attrs,
};

static struct kobject *create_folder(struct kobject *parent, const char *name)
{
	struct kobject *kobj;

	kobj = kobject_create_and_add(name, parent);
	if (!kobj) {
		pr_err("%s: Failed to create %s/\n", __func__, name);
		return NULL;
	}
	return kobj;
}

static struct kobject *create_folder_with_file(struct kobject *parent,
					       const char *name,
					       struct kobj_attribute *attr)
{
	struct kobject *kobj;

	kobj = create_folder(parent, name);
	if (kobj) {
		/* Note: Usually drivers should use device_create_file() rather
		 * than sysfs_create_file(), but the former does not support
		 * creating the file in a subfolder.
		 */
		int ret;

		ret = sysfs_create_file(kobj, &attr->attr);
		if (ret) {
			pr_err("%s: Failed to create %s/%s: ret=%d\n", __func__,
			       name, attr->attr.name, ret);
			kobject_put(kobj);
			return NULL;
		}
	}
	return kobj;
}

static void remove_folder_with_file(struct kobject *kobj,
				    struct kobj_attribute *attr)
{
	sysfs_remove_file(kobj, &attr->attr);
	kobject_put(kobj);
}

static struct kobject *create_folder_with_files(struct kobject *parent,
						const char *name,
						struct attribute_group *group)
{
	struct kobject *kobj;

	kobj = create_folder(parent, name);
	if (kobj) {
		/* Note: Usually drivers should use device_add_groups() rather
		 * than sysfs_create_group(), but the former does not support
		 * creating the folder in a subfolder.
		 */
		int ret;

		ret = sysfs_create_group(kobj, group);
		if (ret) {
			pr_err("%s: Failed to create %s/*: ret=%d\n", __func__,
			       name, ret);
			kobject_put(kobj);
			return NULL;
		}
	}
	return kobj;
}

static void remove_folder_with_files(struct kobject *kobj,
				     struct attribute_group *group)
{
	sysfs_remove_group(kobj, group);
	kobject_put(kobj);
}

static void clean_up(void)
{
	if (state.vendor_kobj) {
		/* Delete <sysfs_device>/properties/android/fstab/vendor/ */
		remove_folder_with_files(state.vendor_kobj, &vendor_group);
		state.vendor_kobj = NULL;
	}
	if (state.system_kobj) {
		/* Delete <sysfs_device>/properties/android/fstab/system/ */
		remove_folder_with_files(state.system_kobj, &system_group);
		state.system_kobj = NULL;
	}
	if (state.fstab_kobj) {
		/* Delete <sysfs_device>/properties/android/fstab/ */
		remove_folder_with_file(state.fstab_kobj,
					&fstab_compatible_attr);
		state.fstab_kobj = NULL;
	}
	if (state.android_kobj) {
		/* Delete <sysfs_device>/properties/android/ */
		remove_folder_with_file(state.android_kobj,
					&android_compatible_attr);
		state.android_kobj = NULL;
	}
	if (state.properties_kobj) {
		/* Delete <sysfs_device>/properties/ */
		kobject_put(state.properties_kobj);
		state.properties_kobj = NULL;
	}
}

static int android_fwdata_probe(struct platform_device *pdev)
{
	int ret = -EIO;

	state.dev = &pdev->dev;
	/* Create <sysfs_device>/properties/ */
	state.properties_kobj = create_folder(&state.dev->kobj, "properties");
	if (!state.properties_kobj)
		goto out;

	/* TODO: Iterate over all device properties in firmware, and dynamically
	 * create sysfs nodes under <sysfs_device>/properties/
	 */

	/* Create <sysfs_device>/properties/android/compatible */
	state.android_kobj = create_folder_with_file(state.properties_kobj,
						     "android",
						     &android_compatible_attr);
	if (!state.android_kobj)
		goto out;

	/* Create <sysfs_device>/properties/android/fstab/compatible */
	state.fstab_kobj = create_folder_with_file(state.android_kobj, "fstab",
						   &fstab_compatible_attr);
	if (!state.fstab_kobj)
		goto out;

	if (device_property_present(state.dev, "android.fstab.system.dev")) {
		/* Firmware contains fstab config for early mount of /system */
		state.system_kobj = create_folder_with_files(state.fstab_kobj,
							     "system",
							     &system_group);
		if (!state.system_kobj)
			goto out;
	}
	if (device_property_present(state.dev, "android.fstab.vendor.dev")) {
		/* Firmware contains fstab config for early mount of /vendor */
		state.vendor_kobj = create_folder_with_files(state.fstab_kobj,
							     "vendor",
							     &vendor_group);
		if (!state.vendor_kobj)
			goto out;
	}
	return 0;

out:
	clean_up();
	return ret;
}

static int android_fwdata_remove(struct platform_device *pdev)
{
	clean_up();
	return 0;
}

static const struct acpi_device_id android_fwdata_acpi_match[] = {
	{ "ANDR0001", 0 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, android_fwdata_acpi_match);

static struct platform_driver android_fwdata_driver = {
	.probe = android_fwdata_probe,
	.remove = android_fwdata_remove,
	.driver = {
		.name = "android_fwdata",
		.owner = THIS_MODULE,
		.acpi_match_table = ACPI_PTR(android_fwdata_acpi_match),
	}
};

module_platform_driver(android_fwdata_driver);
