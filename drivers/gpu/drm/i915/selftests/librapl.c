// SPDX-License-Identifier: MIT
/*
 * Copyright © 2020 Intel Corporation
 */

#include <asm/msr.h>

#include "i915_drv.h"
#include "i915_hwmon.h"
#include "librapl.h"

u64 librapl_energy_uJ(struct drm_i915_private *i915)
{
	unsigned long long power;
	u32 units;
	long energy_uJ = 0;

	if (IS_DGFX(i915)) {
#ifdef CONFIG_HWMON
		if (i915_hwmon_energy_status_get(i915, &energy_uJ))
#endif
			return 0;

	} else {
		if (rdmsrl_safe(MSR_RAPL_POWER_UNIT, &power))
			return 0;

		units = (power & 0x1f00) >> 8;

		if (rdmsrl_safe(MSR_PP1_ENERGY_STATUS, &power))
			return 0;

		energy_uJ = (1000000 * power) >> units; /* convert to uJ */
	}
	return energy_uJ;
}
