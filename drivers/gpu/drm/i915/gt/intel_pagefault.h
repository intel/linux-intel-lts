// SPDX-License-Identifier: MIT
/*
 * Copyright © 2022 Intel Corporation
 */

#ifndef _GT_INTEL_PAGEFAULT_H
#define _GT_INTEL_PAGEFAULT_H

#include <linux/sizes.h>
#include <linux/types.h>

struct drm_i915_gem_object;
struct intel_gt;
struct intel_guc;

enum access_err {
	ACCESS_ERR_OK = 0,
	ACCESS_ERR_NOSUP = 1,
	ACCESS_ERR_NULLVMA = 2,
	ACCESS_ERR_USERPTR = 3,
};

enum access_type {
	ACCESS_TYPE_READ = 0,
	ACCESS_TYPE_WRITE = 1,
	ACCESS_TYPE_ATOMIC = 2,
	ACCESS_TYPE_RESERVED = 3,
};

struct recoverable_page_fault_info {
	u64 page_addr;
	u32 asid;
	u16 pdata;
	u8 vfid;
	u8 access_type;
	u8 fault_type;
	u8 fault_level;
	u8 engine_class;
	u8 engine_instance;
	u8 fault_unsuccessful;
};

struct acc_info {
	u64 va_range_base;
	u32 asid;
	u32 sub_granularity;
	u8 granularity;
	u8 vfid;
	u8 access_type;
	u8 engine_class;
	u8 engine_instance;
};

static inline int granularity_in_byte(int val)
{
	switch (val) {
	case 0:
		return SZ_128K;
	case 1:
		return SZ_2M;
	case 2:
		return SZ_16M;
	case 3:
		return SZ_64M;
	default:
		return 0;
	}
}

static inline int sub_granularity_in_byte(int val)
{
	return (granularity_in_byte(val) / 32);
}

const char *intel_pagefault_type2str(unsigned int type);

const char *intel_access_type2str(unsigned int type);
const char *intel_acc_err2str(unsigned int err);

void intel_gt_pagefault_process_cat_error_msg(struct intel_gt *gt, u32 guc_ctx_id);
int intel_gt_pagefault_process_page_fault_msg(struct intel_gt *gt, const u32 *msg, u32 len);
int intel_pagefault_req_process_msg(struct intel_guc *guc, const u32 *payload,
				    u32 len);
int intel_access_counter_req_process_msg(struct intel_guc *guc,
					 const u32 *payload, u32 len);
#endif /* _GT_INTEL_PAGEFAULT_H */
