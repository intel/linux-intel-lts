/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2021 Intel Corporation
 */

#ifndef _IOV_SELFTEST_UTILS_H_
#define _IOV_SELFTEST_UTILS_H_

#include <linux/types.h>

struct intel_iov;

bool intel_iov_check_ggtt_vfid(struct intel_iov *iov, void __iomem *pte_addr, u16 vfid);

#endif /* _IOV_SELFTEST_UTILS_H_ */
