/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2021 Intel Corporation
 */

#ifndef __INTEL_IOV_EVENT_H__
#define __INTEL_IOV_EVENT_H__

#include <linux/types.h>

struct intel_iov;

int intel_iov_event_process_guc2pf(struct intel_iov *iov, const u32 *msg, u32 len);

#endif /* __INTEL_IOV_EVENT_H__ */
