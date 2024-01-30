/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2022 Intel Corporation
 */

#ifndef _ABI_IOV_ACTIONS_PRELIM_ABI_H_
#define _ABI_IOV_ACTIONS_PRELIM_ABI_H_

#include "iov_messages_abi.h"

/**
 * DOC: IOV Preliminary Actions
 *
 *  _`IOV_ACTION_VF2PF_PF_L4_WA_UPDATE_GGTT` = 0x4444
 */

/**
 * DOC: VF2PF_PF_L4_WA_UPDATE_GGTT
 *
 * This `IOV Message`_ is used by the VF to update GGTT via PF.
 *
 *  +---+-------+--------------------------------------------------------------+
 *  |   | Bits  | Description                                                  |
 *  +===+=======+==============================================================+
 *  | 0 |    31 | ORIGIN = GUC_HXG_ORIGIN_HOST_                                |
 *  |   +-------+--------------------------------------------------------------+
 *  |   | 30:28 | TYPE = GUC_HXG_TYPE_REQUEST_                                 |
 *  |   +-------+--------------------------------------------------------------+
 *  |   | 27:16 | MBZ                                                          |
 *  |   +-------+--------------------------------------------------------------+
 *  |   |  15:0 | ACTION = IOV_ACTION_VF2PF_PF_L4_WA_UPDATE_GGTT_              |
 *  +---+-------+--------------------------------------------------------------+
 *  | 1 |  31:0 | **OFFSET_LO** - low bits of GGTT PTE offset                  |
 *  +---+-------+--------------------------------------------------------------+
 *  | 2 |  31:0 | **OFFSET_HI** - high bits of GGTT PTE offset                 |
 *  +---+-------+--------------------------------------------------------------+
 *  | 3 |  31:0 | **PAT_INDEX** - PAT index                                    |
 *  +---+-------+--------------------------------------------------------------+
 *  | 4 |  31:0 | **PTE_FLAGS** - PTE flags                                    |
 *  +---+-------+--------------------------------------------------------------+
 *  | 5 |  31:0 | **ADDR_LO** - low bits of address                            |
 *  +---+-------+--------------------------------------------------------------+
 *  | 6 |  31:0 | **ADDR_HI** - high bits of address                           |
 *  +---+-------+--------------------------------------------------------------+
 *
 *  +---+-------+--------------------------------------------------------------+
 *  |   | Bits  | Description                                                  |
 *  +===+=======+==============================================================+
 *  | 0 |    31 | ORIGIN = GUC_HXG_ORIGIN_HOST_                                |
 *  |   +-------+--------------------------------------------------------------+
 *  |   | 30:28 | TYPE = GUC_HXG_TYPE_RESPONSE_SUCCESS_                        |
 *  |   +-------+--------------------------------------------------------------+
 *  |   |  27:0 | MBZ                                                          |
 *  +---+-------+--------------------------------------------------------------+
 */
#define IOV_ACTION_VF2PF_PF_L4_WA_UPDATE_GGTT			0x4444

#define VF2PF_PF_L4_WA_UPDATE_GGTT_REQUEST_MSG_LEN		(GUC_HXG_MSG_MIN_LEN + 6u)
#define VF2PF_PF_L4_WA_UPDATE_GGTT_REQUEST_MSG_0_MBZ		GUC_HXG_REQUEST_MSG_0_DATA0
#define VF2PF_PF_L4_WA_UPDATE_GGTT_REQUEST_MSG_1_OFFSET_LO	GUC_HXG_REQUEST_MSG_n_DATAn
#define VF2PF_PF_L4_WA_UPDATE_GGTT_REQUEST_MSG_2_OFFSET_HI	GUC_HXG_REQUEST_MSG_n_DATAn
#define VF2PF_PF_L4_WA_UPDATE_GGTT_REQUEST_MSG_3_PAT_INDEX	GUC_HXG_REQUEST_MSG_n_DATAn
#define VF2PF_PF_L4_WA_UPDATE_GGTT_REQUEST_MSG_4_PTE_FLAGS	GUC_HXG_REQUEST_MSG_n_DATAn
#define VF2PF_PF_L4_WA_UPDATE_GGTT_REQUEST_MSG_5_ADDR_LO	GUC_HXG_REQUEST_MSG_n_DATAn
#define VF2PF_PF_L4_WA_UPDATE_GGTT_REQUEST_MSG_6_ADDR_HI	GUC_HXG_REQUEST_MSG_n_DATAn

#define VF2PF_PF_L4_WA_UPDATE_GGTT_RESPONSE_MSG_LEN		GUC_HXG_MSG_MIN_LEN

#endif /* _ABI_IOV_ACTIONS_PRELIM_ABI_H_ */
