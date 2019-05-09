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
	struct kobject *vbmeta_kobj;
	struct kobject *fstab_kobj;
	struct kobject *system_kobj;
	struct kobject *vendor_kobj;
	struct kobject *product_kobj;
	struct kobject *odm_kobj;
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
	} else if (kobj == state.vbmeta_kobj) {
		prefix = "android.vbmeta";
	} else if (kobj == state.fstab_kobj) {
		prefix = "android.fstab";
	} else if (kobj == state.system_kobj) {
		prefix = "android.fstab.system";
	} else if (kobj == state.vendor_kobj) {
		prefix = "android.fstab.vendor";
	} else if (kobj == state.product_kobj) {
		prefix = "android.fstab.product";
	} else if (kobj == state.odm_kobj) {
		prefix = "android.fstab.odm";
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

#define DT_SIMPLE_ATTR(_prefix, _name) \
	struct kobj_attribute _prefix##_##_name##_attr = { \
		.attr = { \
			.name = __stringify(_name), \
			.mode = 0444, \
		}, \
		.show = property_show, \
	}

static DT_SIMPLE_ATTR(android, compatible);
static DT_SIMPLE_ATTR(vbmeta, compatible);
static DT_SIMPLE_ATTR(fstab, compatible);
static DT_SIMPLE_ATTR(system, compatible);
static DT_SIMPLE_ATTR(vendor, compatible);
static DT_SIMPLE_ATTR(product, compatible);
static DT_SIMPLE_ATTR(odm, compatible);

static DT_SIMPLE_ATTR(vbmeta, parts);

static struct attribute *vbmeta_attrs[] = {
	&vbmeta_compatible_attr.attr,
	&vbmeta_parts_attr.attr,
	NULL,
};

static struct attribute_group vbmeta_group = {
	.attrs = vbmeta_attrs,
};

static DT_SIMPLE_ATTR(system, dev);
static DT_SIMPLE_ATTR(system, type);
static DT_SIMPLE_ATTR(system, mnt_flags);
static DT_SIMPLE_ATTR(system, fsmgr_flags);

static DT_SIMPLE_ATTR(vendor, dev);
static DT_SIMPLE_ATTR(vendor, type);
static DT_SIMPLE_ATTR(vendor, mnt_flags);
static DT_SIMPLE_ATTR(vendor, fsmgr_flags);

static DT_SIMPLE_ATTR(product, dev);
static DT_SIMPLE_ATTR(product, type);
static DT_SIMPLE_ATTR(product, mnt_flags);
static DT_SIMPLE_ATTR(product, fsmgr_flags);

static DT_SIMPLE_ATTR(odm, dev);
static DT_SIMPLE_ATTR(odm, type);
static DT_SIMPLE_ATTR(odm, mnt_flags);
static DT_SIMPLE_ATTR(odm, fsmgr_flags);

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

static struct attribute *product_attrs[] = {
	&product_compatible_attr.attr,
	&product_dev_attr.attr,
	&product_type_attr.attr,
	&product_mnt_flags_attr.attr,
	&product_fsmgr_flags_attr.attr,
	NULL,
};

static struct attribute_group product_group = {
	.attrs = product_attrs,
};

static struct attribute *odm_attrs[] = {
	&odm_compatible_attr.attr,
	&odm_dev_attr.attr,
	&odm_type_attr.attr,
	&odm_mnt_flags_attr.attr,
	&odm_fsmgr_flags_attr.attr,
	NULL,
};

static struct attribute_group odm_group = {
	.attrs = odm_attrs,
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
	if (state.product_kobj) {
		/* Delete <sysfs_device>/properties/android/fstab/product/ */
		remove_folder_with_files(state.product_kobj, &product_group);
		state.product_kobj = NULL;
	}
	if (state.odm_kobj) {
		/* Delete <sysfs_device>/properties/android/fstab/odm/ */
		remove_folder_with_files(state.odm_kobj, &odm_group);
		state.odm_kobj = NULL;
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
	if (state.vbmeta_kobj) {
		/* Delete <sysfs_device>/properties/android/vbmeta/ */
		remove_folder_with_files(state.vbmeta_kobj, &vbmeta_group);
		state.vbmeta_kobj = NULL;
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

	if (device_property_present(state.dev, "android.vbmeta.compatible")) {
		/* Firmware contains vbmeta config for AVB 2.0 */
		state.vbmeta_kobj = create_folder_with_files(state.android_kobj,
							     "vbmeta",
							     &vbmeta_group);
		if (!state.vbmeta_kobj)
			goto out;
	}

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
	if (device_property_present(state.dev, "android.fstab.product.dev")) {
		/* Firmware contains fstab config for early mount of /product */
		state.product_kobj = create_folder_with_files(state.fstab_kobj,
							     "product",
							     &product_group);
		if (!state.product_kobj)
			goto out;
	}
	if (device_property_present(state.dev, "android.fstab.odm.dev")) {
		/* Firmware contains fstab config for early mount of /odm */
		state.odm_kobj = create_folder_with_files(state.fstab_kobj,
							     "odm",
							     &odm_group);
		if (!state.odm_kobj)
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

MODULE_AUTHOR("Yu Ning <yu.ning@intel.com>");
MODULE_AUTHOR("Biyi Li <biyix.li@intel.com>");
MODULE_DESCRIPTION("Intel fwdata driver");
MODULE_LICENSE("GPL v2");
