/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2023 Intel Corporation
 */

#ifndef __INTEL_IOV_GGTT_H__
#define __INTEL_IOV_GGTT_H__

#include "gt/intel_gtt.h"
#include "abi/iov_actions_mmio_abi.h"

struct intel_iov;

int intel_iov_ggtt_pf_update_vf_ptes(struct intel_iov *iov, u32 vfid, u32 pte_offset, u8 mode,
				     u16 num_copies, gen8_pte_t *ptes, u16 count);
void intel_iov_ggtt_vf_init_early(struct intel_iov *iov);
void intel_iov_ggtt_vf_release(struct intel_iov *iov);

void intel_iov_ggtt_vf_update_pte(struct intel_iov *iov, u32 offset, gen8_pte_t pte);
void intel_iov_ggtt_vf_flush_ptes(struct intel_iov *iov);

#endif /* __INTEL_IOV_GGTT_H__ */
