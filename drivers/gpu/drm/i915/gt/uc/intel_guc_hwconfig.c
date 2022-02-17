// SPDX-License-Identifier: MIT
/*
 * Copyright © 2020 Intel Corporation
 */

#include "gt/intel_gt.h"
#include "gt/intel_hwconfig_types.h"
#include "i915_drv.h"
#include "i915_memcpy.h"
#include "intel_guc_hwconfig.h"

static
inline struct intel_guc *hwconfig_to_guc(struct intel_guc_hwconfig *hwconfig)
{
	return container_of(hwconfig, struct intel_guc, hwconfig);
}

/**
 * GuC has a blob containing the device information (hwconfig), which is a
 * simple and flexible KLV (Key/Length/Value) formatted table.
 *
 * For instance it could be simple as this:
 *
 * enum device_attr
 * {
 * 	ATTR_EUS_PER_SSLICE = 0,
 * 	ATTR_SOME_MASK 	  = 1,
 * };
 *
 * static const u32 hwconfig[] =
 * {
 * 	 ATTR_EUS_PER_SSLICE,
 * 	 1,		// Value Length in DWords
 * 	 8,		// Value
 *
 * 	 ATTR_SOME_MASK,
 * 	 3,
 * 	 0x00FFFFFFFF, 0xFFFFFFFF, 0xFF000000, // Value
 * };
 * static const u32 table_size = sizeof(hwconfig) / sizeof(hwconfig[0]));
 *
 * It is important to highlight though that the device attributes ids are common
 * across multiple components including, GuC, i915 and user space components.
 * The definition of the actual and current attributes can be found in
 * the header file: intel_hwconfig_types.h
 */

static int __guc_action_get_hwconfig(struct intel_guc_hwconfig *hwconfig,
				    u32 ggtt_offset, u32 ggtt_size)
{
	struct intel_guc *guc = hwconfig_to_guc(hwconfig);
	u32 action[] = {
		INTEL_GUC_ACTION_GET_HWCONFIG,
		ggtt_offset,
		0, /* upper 32 bits of address */
		ggtt_size,
	};
	int ret;

	ret = intel_guc_send_mmio(guc, action, ARRAY_SIZE(action), NULL, 0);
	if (ret == -ENXIO)
		return -ENOENT;

	if (!ggtt_size && !ret)
		ret = -EINVAL;

	return ret;
}

static int guc_hwconfig_discover_size(struct intel_guc_hwconfig *hwconfig)
{
	int ret;

	/* Sending a query with too small a table will return the size of the table */
	ret = __guc_action_get_hwconfig(hwconfig, 0, 0);
	if (ret < 0)
		return ret;

	hwconfig->size = ret;
	return 0;
}

static int guc_hwconfig_fill_buffer(struct intel_guc_hwconfig *hwconfig)
{
	struct intel_guc *guc = hwconfig_to_guc(hwconfig);
	u32 ggtt_offset;
	int ret;
	struct i915_vma *vma;
	void *vaddr;

	GEM_BUG_ON(!hwconfig->size);

	ret = intel_guc_allocate_and_map_vma(guc, hwconfig->size, &vma, &vaddr);
	if (ret)
		return ret;

	ggtt_offset = intel_guc_ggtt_offset(guc, vma);

	ret = __guc_action_get_hwconfig(hwconfig, ggtt_offset, hwconfig->size);
	if (ret >= 0)
		memcpy(hwconfig->ptr, vaddr, hwconfig->size);

	i915_vma_unpin_and_release(&vma, I915_VMA_RELEASE_MAP);

	return ret;
}

/**
 * intel_guc_hwconfig_get_value - Get single value for a given key
 * @key: KLV's key for the attribute
 *
 * Parse our KLV table returning the single value for a given key.
 * This function is intended to return only 1 dword-sized value.
 * If used with a key where len >= 2, only the first value will be
 * returned.
 * Attributes with multiple entries are not yet needed by i915.
 */
u32 intel_guc_hwconfig_get_value(struct intel_guc_hwconfig *hwconfig, u32 key)
{
	int i, len;
	u32 *array = (u32 *)(hwconfig->ptr);

	if (key > INTEL_HWCONFIG_MAX)
		return -EINVAL;

	for (i = 0; i < hwconfig->size / sizeof(u32); i += 2 + len) {
		if (array[i] == key)
			return array[i + 2];
		len = array[i + 1];
	}

	return -ENOENT;
}

static bool has_table(struct drm_i915_private *i915)
{
	if (IS_ADLP_GT_STEP(i915, STEP_B0, STEP_FOREVER))
		return 1;

	return 0;
}

/**
 * intel_guc_hwconfig_init - Initialize the HWConfig
 *
 * Allocates and pin a GGTT buffer to be filled with the HWConfig table.
 * This buffer will be ready to be queried as needed at any time.
 */
int intel_guc_hwconfig_init(struct intel_guc_hwconfig *hwconfig)
{
	struct intel_guc *guc = hwconfig_to_guc(hwconfig);
	struct drm_i915_private *i915 = guc_to_gt(guc)->i915;
	int ret;

	if (!has_table(i915))
		return 0;

	ret = guc_hwconfig_discover_size(hwconfig);
	if (ret)
		return ret;

	hwconfig->ptr = kmalloc(hwconfig->size, GFP_KERNEL);
	if (!hwconfig->ptr) {
		hwconfig->size = 0;
		return -ENOMEM;
	}

	ret = guc_hwconfig_fill_buffer(hwconfig);
	if (ret < 0) {
		kfree(hwconfig->ptr);
		hwconfig->size = 0;
		hwconfig->ptr = NULL;
		return ret;
	}

	return 0;
}

/**
 * intel_guc_hwconfig_fini - Finalize the HWConfig
 *
 * This unpin and release the GGTT buffer containing the HWConfig table.
 * The table needs to be cached and available during the runtime, so
 * this function should only be called only when disabling guc.
 */
void intel_guc_hwconfig_fini(struct intel_guc_hwconfig *hwconfig)
{
	kfree(hwconfig->ptr);
	hwconfig->size = 0;
	hwconfig->ptr = NULL;
}
