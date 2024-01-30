/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2021 Intel Corporation
 */

#ifndef _INTEL_GSC_FW_H_
#define _INTEL_GSC_FW_H_

#include <linux/types.h>

struct intel_gsc_uc;
struct intel_uncore;

int intel_gsc_fw_upload(struct intel_gsc_uc *gsc);

int intel_gsc_fw_heci_send(struct intel_gsc_uc *gsc, u64 addr_in, u32 size_in,
			   u64 addr_out, u32 size_out);

bool intel_gsc_uc_fw_init_done(struct intel_uncore *uncore);

#endif
