/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2021 Intel Corporation
 */

#ifndef __GUC_ACTIONS_PF_ABI_H__
#define __GUC_ACTIONS_PF_ABI_H__

#include "guc_communication_ctb_abi.h"

/**
 * DOC: PF2GUC_UPDATE_VGT_POLICY
 *
 * This message is optionaly used by the PF to set `GuC VGT Policy KLVs`_.
 *
 * This message must be sent as `CTB HXG Message`_.
 *
 *  +---+-------+--------------------------------------------------------------+
 *  |   | Bits  | Description                                                  |
 *  +===+=======+==============================================================+
 *  | 0 |    31 | ORIGIN = GUC_HXG_ORIGIN_HOST_                                |
 *  |   +-------+--------------------------------------------------------------+
 *  |   | 30:28 | TYPE = GUC_HXG_TYPE_REQUEST_                                 |
 *  |   +-------+--------------------------------------------------------------+
 *  |   | 27:16 | DATA0 = MBZ                                                  |
 *  |   +-------+--------------------------------------------------------------+
 *  |   |  15:0 | ACTION = _`GUC_ACTION_PF2GUC_UPDATE_VGT_POLICY` = 0x5502     |
 *  +---+-------+--------------------------------------------------------------+
 *  | 1 |  31:0 | DATA1 = **CFG_ADDR_LO** - dword aligned GGTT offset that     |
 *  |   |       | represents the start of `GuC VGT Policy KLVs`_ list.         |
 *  +---+-------+--------------------------------------------------------------+
 *  | 2 |  31:0 | DATA2 = **CFG_ADDR_HI** - upper 32 bits of above offset.     |
 *  +---+-------+--------------------------------------------------------------+
 *  | 3 |  31:0 | DATA3 = **CFG_SIZE** - size (in dwords) of the config buffer |
 *  +---+-------+--------------------------------------------------------------+
 *
 *  +---+-------+--------------------------------------------------------------+
 *  |   | Bits  | Description                                                  |
 *  +===+=======+==============================================================+
 *  | 0 |    31 | ORIGIN = GUC_HXG_ORIGIN_GUC_                                 |
 *  |   +-------+--------------------------------------------------------------+
 *  |   | 30:28 | TYPE = GUC_HXG_TYPE_RESPONSE_SUCCESS_                        |
 *  |   +-------+--------------------------------------------------------------+
 *  |   |  27:0 | DATA0 = **COUNT** - number of KLVs successfully applied      |
 *  +---+-------+--------------------------------------------------------------+
 */
#define GUC_ACTION_PF2GUC_UPDATE_VGT_POLICY			0x5502

#define PF2GUC_UPDATE_VGT_POLICY_REQUEST_MSG_LEN		(GUC_HXG_REQUEST_MSG_MIN_LEN + 3u)
#define PF2GUC_UPDATE_VGT_POLICY_REQUEST_MSG_0_MBZ		GUC_HXG_REQUEST_MSG_0_DATA0
#define PF2GUC_UPDATE_VGT_POLICY_REQUEST_MSG_1_CFG_ADDR_LO	GUC_HXG_REQUEST_MSG_n_DATAn
#define PF2GUC_UPDATE_VGT_POLICY_REQUEST_MSG_2_CFG_ADDR_HI	GUC_HXG_REQUEST_MSG_n_DATAn
#define PF2GUC_UPDATE_VGT_POLICY_REQUEST_MSG_3_CFG_SIZE		GUC_HXG_REQUEST_MSG_n_DATAn

#define PF2GUC_UPDATE_VGT_POLICY_RESPONSE_MSG_LEN		GUC_HXG_RESPONSE_MSG_MIN_LEN
#define PF2GUC_UPDATE_VGT_POLICY_RESPONSE_MSG_0_COUNT		GUC_HXG_RESPONSE_MSG_0_DATA0

/**
 * DOC: PF2GUC_UPDATE_VF_CFG
 *
 * The PF2GUC_UPDATE_VF_CFG message is used by PF to provision single VF in GuC.
 *
 * This message must be sent as `CTB HXG Message`_.
 *
 *  +---+-------+--------------------------------------------------------------+
 *  |   | Bits  | Description                                                  |
 *  +===+=======+==============================================================+
 *  | 0 |    31 | ORIGIN = GUC_HXG_ORIGIN_HOST_                                |
 *  |   +-------+--------------------------------------------------------------+
 *  |   | 30:28 | TYPE = GUC_HXG_TYPE_REQUEST_                                 |
 *  |   +-------+--------------------------------------------------------------+
 *  |   | 27:16 | DATA0 = MBZ                                                  |
 *  |   +-------+--------------------------------------------------------------+
 *  |   |  15:0 | ACTION = _`GUC_ACTION_PF2GUC_UPDATE_VF_CFG` = 0x5503         |
 *  +---+-------+--------------------------------------------------------------+
 *  | 1 |  31:0 | DATA1 = **VFID** - identifier of the VF that the KLV         |
 *  |   |       | configurations are being applied to                          |
 *  +---+-------+--------------------------------------------------------------+
 *  | 2 |  31:0 | DATA2 = **CFG_ADDR_LO** - dword aligned GGTT offset that     |
 *  |   |       | represents the start of a list of virtualization related KLV |
 *  |   |       | configs that are to be applied to the VF.                    |
 *  |   |       | If this parameter is zero, the list is not parsed.           |
 *  |   |       | If full configs address parameter is zero and configs_size is|
 *  |   |       | zero associated VF config shall be reset to its default state|
 *  +---+-------+--------------------------------------------------------------+
 *  | 3 |  31:0 | DATA3 = **CFG_ADDR_HI** - upper 32 bits of configs address.  |
 *  +---+-------+--------------------------------------------------------------+
 *  | 4 |  31:0 | DATA4 = **CFG_SIZE** - size (in dwords) of the config buffer |
 *  +---+-------+--------------------------------------------------------------+
 *
 *  +---+-------+--------------------------------------------------------------+
 *  |   | Bits  | Description                                                  |
 *  +===+=======+==============================================================+
 *  | 0 |    31 | ORIGIN = GUC_HXG_ORIGIN_GUC_                                 |
 *  |   +-------+--------------------------------------------------------------+
 *  |   | 30:28 | TYPE = GUC_HXG_TYPE_RESPONSE_SUCCESS_                        |
 *  |   +-------+--------------------------------------------------------------+
 *  |   |  27:0 | DATA0 = **COUNT** - number of KLVs successfully applied      |
 *  +---+-------+--------------------------------------------------------------+
 */
#define GUC_ACTION_PF2GUC_UPDATE_VF_CFG			0x5503

#define PF2GUC_UPDATE_VF_CFG_REQUEST_MSG_LEN		(GUC_HXG_REQUEST_MSG_MIN_LEN + 4u)
#define PF2GUC_UPDATE_VF_CFG_REQUEST_MSG_0_MBZ		GUC_HXG_REQUEST_MSG_0_DATA0
#define PF2GUC_UPDATE_VF_CFG_REQUEST_MSG_1_VFID		GUC_HXG_REQUEST_MSG_n_DATAn
#define PF2GUC_UPDATE_VF_CFG_REQUEST_MSG_2_CFG_ADDR_LO	GUC_HXG_REQUEST_MSG_n_DATAn
#define PF2GUC_UPDATE_VF_CFG_REQUEST_MSG_3_CFG_ADDR_HI	GUC_HXG_REQUEST_MSG_n_DATAn
#define PF2GUC_UPDATE_VF_CFG_REQUEST_MSG_4_CFG_SIZE	GUC_HXG_REQUEST_MSG_n_DATAn

#define PF2GUC_UPDATE_VF_CFG_RESPONSE_MSG_LEN		GUC_HXG_RESPONSE_MSG_MIN_LEN
#define PF2GUC_UPDATE_VF_CFG_RESPONSE_MSG_0_COUNT	GUC_HXG_RESPONSE_MSG_0_DATA0

/**
 * DOC: GUC2PF_RELAY_FROM_VF
 *
 * The GUC2PF_RELAY_FROM_VF message is used by the GuC to forward VF/PF messages
 * received from the VF.
 *
 * This H2G message must be sent as `CTB HXG Message`_.
 *
 *  +---+-------+--------------------------------------------------------------+
 *  |   | Bits  | Description                                                  |
 *  +===+=======+==============================================================+
 *  | 0 |    31 | ORIGIN = GUC_HXG_ORIGIN_GUC_                                 |
 *  |   +-------+--------------------------------------------------------------+
 *  |   | 30:28 | TYPE = GUC_HXG_TYPE_EVENT_                                   |
 *  |   +-------+--------------------------------------------------------------+
 *  |   | 27:16 | DATA0 = MBZ                                                  |
 *  |   +-------+--------------------------------------------------------------+
 *  |   |  15:0 | ACTION = _`GUC_ACTION_GUC2PF_RELAY_FROM_VF` = 0x5100         |
 *  +---+-------+--------------------------------------------------------------+
 *  | 1 |  31:0 | DATA1 = **VFID** - source VF identifier                      |
 *  +---+-------+--------------------------------------------------------------+
 *  | 2 |  31:0 | DATA2 = **RELAY_ID** - VF/PF message ID                      |
 *  +---+-------+--------------------------------------------------------------+
 *  | 3 |  31:0 | DATA3 = **RELAY_DATA1** - VF/PF message payload data         |
 *  +---+-------+--------------------------------------------------------------+
 *  |...|       |                                                              |
 *  +---+-------+--------------------------------------------------------------+
 *  | n |  31:0 | DATAn = **RELAY_DATAx** - VF/PF message payload data         |
 *  +---+-------+--------------------------------------------------------------+
 */
#define GUC_ACTION_GUC2PF_RELAY_FROM_VF			0x5100

#define GUC2PF_RELAY_FROM_VF_EVENT_MSG_MIN_LEN		(GUC_HXG_EVENT_MSG_MIN_LEN + 2u)
#define GUC2PF_RELAY_FROM_VF_EVENT_MSG_MAX_LEN		(GUC2PF_RELAY_FROM_VF_EVENT_MSG_MIN_LEN + 60u)
#define GUC2PF_RELAY_FROM_VF_EVENT_MSG_0_MBZ		GUC_HXG_EVENT_MSG_0_DATA0
#define GUC2PF_RELAY_FROM_VF_EVENT_MSG_1_VFID		GUC_HXG_EVENT_MSG_n_DATAn
#define GUC2PF_RELAY_FROM_VF_EVENT_MSG_2_RELAY_ID	GUC_HXG_EVENT_MSG_n_DATAn
#define GUC2PF_RELAY_FROM_VF_EVENT_MSG_3_RELAY_DATA1	GUC_HXG_EVENT_MSG_n_DATAn
#define GUC2PF_RELAY_FROM_VF_EVENT_MSG_n_RELAY_DATAx	GUC_HXG_EVENT_MSG_n_DATAn

/**
 * DOC: PF2GUC_RELAY_TO_VF
 *
 * The PF2GUC_RELAY_TO_VF message is used by PF to send VF/PF messages to the VF.
 *
 * This action message must be sent over CTB as `CTB HXG Message`_.
 *
 *  +---+-------+--------------------------------------------------------------+
 *  |   | Bits  | Description                                                  |
 *  +===+=======+==============================================================+
 *  | 0 |    31 | ORIGIN = GUC_HXG_ORIGIN_HOST_                                |
 *  |   +-------+--------------------------------------------------------------+
 *  |   | 30:28 | TYPE = GUC_HXG_TYPE_REQUEST_ or GUC_HXG_TYPE_EVENT_          |
 *  |   +-------+--------------------------------------------------------------+
 *  |   | 27:16 | DATA0 = MBZ                                                  |
 *  |   +-------+--------------------------------------------------------------+
 *  |   |  15:0 | ACTION = _`GUC_ACTION_PF2GUC_RELAY_TO_VF` = 0x5101           |
 *  +---+-------+--------------------------------------------------------------+
 *  | 1 |  31:0 | DATA1 = **VFID** - target VF identifier                      |
 *  +---+-------+--------------------------------------------------------------+
 *  | 2 |  31:0 | DATA2 = **RELAY_ID** - VF/PF message ID                      |
 *  +---+-------+--------------------------------------------------------------+
 *  | 3 |  31:0 | DATA3 = **RELAY_DATA1** - VF/PF message payload data         |
 *  +---+-------+--------------------------------------------------------------+
 *  |...|       |                                                              |
 *  +---+-------+--------------------------------------------------------------+
 *  | n |  31:0 | DATAn = **RELAY_DATAx** - VF/PF message payload data         |
 *  +---+-------+--------------------------------------------------------------+
 *
 *  +---+-------+--------------------------------------------------------------+
 *  |   | Bits  | Description                                                  |
 *  +===+=======+==============================================================+
 *  | 0 |    31 | ORIGIN = GUC_HXG_ORIGIN_GUC_                                 |
 *  |   +-------+--------------------------------------------------------------+
 *  |   | 30:28 | TYPE = GUC_HXG_TYPE_RESPONSE_SUCCESS_                        |
 *  |   +-------+--------------------------------------------------------------+
 *  |   |  27:0 | DATA0 = MBZ                                                  |
 *  +---+-------+--------------------------------------------------------------+
 */
#define GUC_ACTION_PF2GUC_RELAY_TO_VF			0x5101

#define PF2GUC_RELAY_TO_VF_REQUEST_MSG_MIN_LEN		(GUC_HXG_REQUEST_MSG_MIN_LEN + 2u)
#define PF2GUC_RELAY_TO_VF_REQUEST_MSG_MAX_LEN		(PF2GUC_RELAY_TO_VF_REQUEST_MSG_MIN_LEN + 60u)
#define PF2GUC_RELAY_TO_VF_REQUEST_MSG_0_MBZ		GUC_HXG_REQUEST_MSG_0_DATA0
#define PF2GUC_RELAY_TO_VF_REQUEST_MSG_1_VFID		GUC_HXG_REQUEST_MSG_n_DATAn
#define PF2GUC_RELAY_TO_VF_REQUEST_MSG_2_RELAY_ID	GUC_HXG_REQUEST_MSG_n_DATAn
#define PF2GUC_RELAY_TO_VF_REQUEST_MSG_3_RELAY_DATA1	GUC_HXG_REQUEST_MSG_n_DATAn
#define PF2GUC_RELAY_TO_VF_REQUEST_MSG_n_RELAY_DATAx	GUC_HXG_REQUEST_MSG_n_DATAn

#define PF2GUC_RELAY_TO_VF_RESPONSE_MSG_LEN		GUC_HXG_RESPONSE_MSG_MIN_LEN
#define PF2GUC_RELAY_TO_VF_RESPONSE_DATA0_MBZ		GUC_HXG_RESPONSE_MSG_0_DATA0

/**
 * DOC: GUC2PF_ADVERSE_EVENT
 *
 * This message is used by the GuC to notify PF about adverse events.
 *
 * This G2H message must be sent as `CTB HXG Message`_.
 *
 *  +---+-------+--------------------------------------------------------------+
 *  |   | Bits  | Description                                                  |
 *  +===+=======+==============================================================+
 *  | 0 |    31 | ORIGIN = GUC_HXG_ORIGIN_GUC_                                 |
 *  |   +-------+--------------------------------------------------------------+
 *  |   | 30:28 | TYPE = GUC_HXG_TYPE_EVENT_                                   |
 *  |   +-------+--------------------------------------------------------------+
 *  |   | 27:16 | DATA0 = MBZ                                                  |
 *  |   +-------+--------------------------------------------------------------+
 *  |   |  15:0 | ACTION = _`GUC_ACTION_GUC2PF_ADVERSE_EVENT` = 0x5104         |
 *  +---+-------+--------------------------------------------------------------+
 *  | 1 |  31:0 | DATA1 = **VFID** - VF identifier                             |
 *  +---+-------+--------------------------------------------------------------+
 *  | 2 |  31:0 | DATA2 = **THRESHOLD** - key of the exceeded threshold        |
 *  +---+-------+--------------------------------------------------------------+
 */
#define GUC_ACTION_GUC2PF_ADVERSE_EVENT			0x5104

#define GUC2PF_ADVERSE_EVENT_EVENT_MSG_LEN		(GUC_HXG_EVENT_MSG_MIN_LEN + 2u)
#define GUC2PF_ADVERSE_EVENT_EVENT_MSG_0_MBZ		GUC_HXG_EVENT_MSG_0_DATA0
#define GUC2PF_ADVERSE_EVENT_EVENT_MSG_1_VFID		GUC_HXG_EVENT_MSG_n_DATAn
#define GUC2PF_ADVERSE_EVENT_EVENT_MSG_2_THRESHOLD	GUC_HXG_EVENT_MSG_n_DATAn

/**
 * DOC: GUC2PF_VF_STATE_NOTIFY
 *
 * The GUC2PF_VF_STATE_NOTIFY message is used by the GuC to notify PF about change
 * of the VF state.
 *
 * This G2H message must be sent as `CTB HXG Message`_.
 *
 *  +---+-------+--------------------------------------------------------------+
 *  |   | Bits  | Description                                                  |
 *  +===+=======+==============================================================+
 *  | 0 |    31 | ORIGIN = GUC_HXG_ORIGIN_GUC_                                 |
 *  |   +-------+--------------------------------------------------------------+
 *  |   | 30:28 | TYPE = GUC_HXG_TYPE_EVENT_                                   |
 *  |   +-------+--------------------------------------------------------------+
 *  |   | 27:16 | DATA0 = MBZ                                                  |
 *  |   +-------+--------------------------------------------------------------+
 *  |   |  15:0 | ACTION = _`GUC_ACTION_GUC2PF_VF_STATE_NOTIFY` = 0x5106       |
 *  +---+-------+--------------------------------------------------------------+
 *  | 1 |  31:0 | DATA1 = **VFID** - VF identifier                             |
 *  +---+-------+--------------------------------------------------------------+
 *  | 2 |  31:0 | DATA2 = **EVENT** - notification event:                      |
 *  |   |       |                                                              |
 *  |   |       |   - _`GUC_PF_NOTIFY_VF_FLR_START` = 1                        |
 *  |   |       |   - _`GUC_PF_NOTIFY_VF_PAUSE_COMPLETE` = 2                   |
 *  |   |       |   - _`GUC_PF_NOTIFY_VF_FIXUP_DONE` = 3                       |
 *  +---+-------+--------------------------------------------------------------+
 */
#define GUC_ACTION_GUC2PF_VF_STATE_NOTIFY		0x5106

#define GUC2PF_VF_STATE_NOTIFY_EVENT_MSG_LEN		(GUC_HXG_EVENT_MSG_MIN_LEN + 2u)
#define GUC2PF_VF_STATE_NOTIFY_EVENT_MSG_0_MBZ		GUC_HXG_EVENT_MSG_0_DATA0
#define GUC2PF_VF_STATE_NOTIFY_EVENT_MSG_1_VFID		GUC_HXG_EVENT_MSG_n_DATAn
#define GUC2PF_VF_STATE_NOTIFY_EVENT_MSG_2_EVENT	GUC_HXG_EVENT_MSG_n_DATAn
#define   GUC_PF_NOTIFY_VF_FLR_START			1
#define   GUC_PF_NOTIFY_VF_PAUSE_COMPLETE		2
#define   GUC_PF_NOTIFY_VF_FIXUP_DONE			3

/**
 * DOC: PF2GUC_VF_CONTROL
 *
 * The PF2GUC_VF_CONTROL message is used by the PF to trigger VF state change
 * maintained by the GuC.
 *
 * This H2G message must be sent as `CTB HXG Message`_.
 *
 *  +---+-------+--------------------------------------------------------------+
 *  |   | Bits  | Description                                                  |
 *  +===+=======+==============================================================+
 *  | 0 |    31 | ORIGIN = GUC_HXG_ORIGIN_HOST_                                |
 *  |   +-------+--------------------------------------------------------------+
 *  |   | 30:28 | TYPE = GUC_HXG_TYPE_REQUEST_                                 |
 *  |   +-------+--------------------------------------------------------------+
 *  |   | 27:16 | DATA0 = MBZ                                                  |
 *  |   +-------+--------------------------------------------------------------+
 *  |   |  15:0 | ACTION = _`GUC_ACTION_PF2GUC_VF_CONTROL_CMD` = 0x5506        |
 *  +---+-------+--------------------------------------------------------------+
 *  | 1 |  31:0 | DATA1 = **VFID** - VF identifier                             |
 *  +---+-------+--------------------------------------------------------------+
 *  | 2 |  31:0 | DATA2 = **COMMAND** - control command:                       |
 *  |   |       |                                                              |
 *  |   |       |   - _`GUC_PF_TRIGGER_VF_PAUSE` = 1                           |
 *  |   |       |   - _`GUC_PF_TRIGGER_VF_RESUME` = 2                          |
 *  |   |       |   - _`GUC_PF_TRIGGER_VF_STOP` = 3                            |
 *  |   |       |   - _`GUC_PF_TRIGGER_VF_FLR_DONE` = 4                        |
 *  |   |       |   - _`GUC_PF_TRIGGER_VF_RESOURCE_FIX` = 5                    |
 *  +---+-------+--------------------------------------------------------------+
 *
 *  +---+-------+--------------------------------------------------------------+
 *  |   | Bits  | Description                                                  |
 *  +===+=======+==============================================================+
 *  | 0 |    31 | ORIGIN = GUC_HXG_ORIGIN_GUC_                                 |
 *  |   +-------+--------------------------------------------------------------+
 *  |   | 30:28 | TYPE = GUC_HXG_TYPE_RESPONSE_SUCCESS_                        |
 *  |   +-------+--------------------------------------------------------------+
 *  |   |  27:0 | DATA0 = MBZ                                                  |
 *  +---+-------+--------------------------------------------------------------+
 */
#define GUC_ACTION_PF2GUC_VF_CONTROL			0x5506

#define PF2GUC_VF_CONTROL_REQUEST_MSG_LEN		(GUC_HXG_EVENT_MSG_MIN_LEN + 2u)
#define PF2GUC_VF_CONTROL_REQUEST_MSG_0_MBZ		GUC_HXG_EVENT_MSG_0_DATA0
#define PF2GUC_VF_CONTROL_REQUEST_MSG_1_VFID		GUC_HXG_EVENT_MSG_n_DATAn
#define PF2GUC_VF_CONTROL_REQUEST_MSG_2_COMMAND		GUC_HXG_EVENT_MSG_n_DATAn
#define   GUC_PF_TRIGGER_VF_PAUSE			1
#define   GUC_PF_TRIGGER_VF_RESUME			2
#define   GUC_PF_TRIGGER_VF_STOP			3
#define   GUC_PF_TRIGGER_VF_FLR_DONE			4
#define   GUC_PF_TRIGGER_VF_RESOURCE_FIX		5

/**
 * DOC: PF2GUC_SAVE_RESTORE_VF
 *
 * This message is used by the PF to migrate VF info state maintained by the GuC.
 *
 * This message must be sent as `CTB HXG Message`_.
 *
 *  +---+-------+--------------------------------------------------------------+
 *  |   | Bits  | Description                                                  |
 *  +===+=======+==============================================================+
 *  | 0 |    31 | ORIGIN = GUC_HXG_ORIGIN_HOST_                                |
 *  |   +-------+--------------------------------------------------------------+
 *  |   | 30:28 | TYPE = GUC_HXG_TYPE_REQUEST_                                 |
 *  |   +-------+--------------------------------------------------------------+
 *  |   | 27:16 | DATA0 = **OPCODE** - operation to take:                      |
 *  |   |       |                                                              |
 *  |   |       |   - _`GUC_PF_OPCODE_VF_SAVE` = 0                             |
 *  |   |       |   - _`GUC_PF_OPCODE_VF_RESTORE` = 1                          |
 *  |   +-------+--------------------------------------------------------------+
 *  |   |  15:0 | ACTION = _`GUC_ACTION_PF2GUC_SAVE_RESTORE_VF` = 0x550B       |
 *  +---+-------+--------------------------------------------------------------+
 *  | 1 |  31:0 | DATA1 = **VFID** - VF identifier                             |
 *  +---+-------+--------------------------------------------------------------+
 *  | 2 |  31:0 | DATA2 = **BUFF_LO** - lower 32-bits of GGTT offset to the 4K |
 *  |   |       | buffer where the VF info will be save to or restored from.   |
 *  +---+-------+--------------------------------------------------------------+
 *  | 3 |  31:0 | DATA3 = **BUFF_HI** - upper 32-bits of GGTT offset to the 4K |
 *  |   |       | buffer where the VF info will be save to or restored from.   |
 *  +---+-------+--------------------------------------------------------------+
 *
 *  +---+-------+--------------------------------------------------------------+
 *  |   | Bits  | Description                                                  |
 *  +===+=======+==============================================================+
 *  | 0 |    31 | ORIGIN = GUC_HXG_ORIGIN_GUC_                                 |
 *  |   +-------+--------------------------------------------------------------+
 *  |   | 30:28 | TYPE = GUC_HXG_TYPE_RESPONSE_SUCCESS_                        |
 *  |   +-------+--------------------------------------------------------------+
 *  |   |  27:0 | DATA0 = **USED** - size of buffer used (in bytes)            |
 *  +---+-------+--------------------------------------------------------------+
 */
#define GUC_ACTION_PF2GUC_SAVE_RESTORE_VF		0x550B

#define PF2GUC_SAVE_RESTORE_VF_REQUEST_MSG_LEN		(GUC_HXG_EVENT_MSG_MIN_LEN + 3u)
#define PF2GUC_SAVE_RESTORE_VF_REQUEST_MSG_0_OPCODE	GUC_HXG_EVENT_MSG_0_DATA0
#define   GUC_PF_OPCODE_VF_SAVE				0
#define   GUC_PF_OPCODE_VF_RESTORE			1
#define PF2GUC_SAVE_RESTORE_VF_REQUEST_MSG_1_VFID	GUC_HXG_EVENT_MSG_n_DATAn
#define PF2GUC_SAVE_RESTORE_VF_REQUEST_MSG_2_BUFF_LO	GUC_HXG_EVENT_MSG_n_DATAn
#define PF2GUC_SAVE_RESTORE_VF_REQUEST_MSG_3_BUFF_HI	GUC_HXG_EVENT_MSG_n_DATAn

#define PF2GUC_SAVE_RESTORE_VF_RESPONSE_MSG_LEN		GUC_HXG_RESPONSE_MSG_MIN_LEN
#define PF2GUC_SAVE_RESTORE_VF_RESPONSE_MSG_0_USED	GUC_HXG_RESPONSE_MSG_0_DATA0

#endif /* __GUC_ACTIONS_PF_ABI_H__ */
