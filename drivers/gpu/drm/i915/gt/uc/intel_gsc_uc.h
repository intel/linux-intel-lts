/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2021 Intel Corporation
 */

#ifndef _INTEL_GSC_UC_H_
#define _INTEL_GSC_UC_H_

#include "intel_uc_fw.h"

struct i915_vma;
struct intel_context;

struct intel_gsc_uc {
	/* Generic uC firmware management */
	struct intel_uc_fw fw;

	/* GSC-specific additions */
	struct i915_vma *local; /* private memory for GSC usage */
	struct intel_context *ce; /* for submission to GSC FW via GSC engine */
};

void intel_gsc_uc_init_early(struct intel_gsc_uc *gsc);
int intel_gsc_uc_init(struct intel_gsc_uc *gsc);
void intel_gsc_uc_fini(struct intel_gsc_uc *gsc);

static inline int intel_gsc_uc_sanitize(struct intel_gsc_uc *gsc)
{
	intel_uc_fw_sanitize(&gsc->fw);
	return 0;
}

static inline bool intel_gsc_uc_is_supported(const struct intel_gsc_uc *gsc)
{
	return intel_uc_fw_is_supported(&gsc->fw);
}

static inline bool intel_gsc_uc_is_wanted(const struct intel_gsc_uc *gsc)
{
	return intel_uc_fw_is_enabled(&gsc->fw);
}

static inline bool intel_gsc_uc_is_used(const struct intel_gsc_uc *gsc)
{
	GEM_BUG_ON(__intel_uc_fw_status(&gsc->fw) == INTEL_UC_FIRMWARE_SELECTED);
	return intel_uc_fw_is_available(&gsc->fw) ||
	       intel_uc_fw_is_preloaded(&gsc->fw);
}

#endif
