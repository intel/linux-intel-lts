/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2021 Intel Corporation
 */

#ifndef _ABI_GUC_ACTIONS_PRIVILEGED_ABI_H
#define _ABI_GUC_ACTIONS_PRIVILEGED_ABI_H

#include "guc_communication_ctb_abi.h"

/**
 * DOC: HOST2GUC_REGISTER_G2G
 *
 * This message is used to register a buffer used for communication between
 * GuC instances in multi-tile environments.
 *
 * This message must be sent as `HXG CTB Message`_.
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
 *  |   |  15:0 | ACTION = _`GUC_ACTION_HOST2GUC_REGISTER_G2G` = 0x5202        |
 *  +---+-------+--------------------------------------------------------------+
 *  | 1 | 31:16 | RESERVED = MBZ                                               |
 *  |   +-------+--------------------------------------------------------------+
 *  |   | 15:12 | **DEST** - Tile ID associated with destination               |
 *  |   +-------+--------------------------------------------------------------+
 *  |   |  11:8 | **TYPE** - type of the G2G buffer                            |
 *  |   |       |                                                              |
 *  |   |       |   - _`GUC_G2G_TYPE_IN` = 0                                   |
 *  |   |       |   - _`GUC_G2G_TYPE_OUT` = 1                                  |
 *  |   +-------+--------------------------------------------------------------+
 *  |   |   7:0 | **SIZE** - size of the `G2G Buffer`_ in 4K units minus 1     |
 *  +---+-------+--------------------------------------------------------------+
 *  | 2 |  31:0 | **DESC_ADDR** - GGTT address of the `G2G Descriptor`_        |
 *  +---+-------+--------------------------------------------------------------+
 *  | 3 |  31:0 | **BUFF_ADDR** - GGTT address of the `G2G Buffer`_            |
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
#define GUC_ACTION_HOST2GUC_REGISTER_G2G		0x4507 /* FIXME: 0x5202 */

#define HOST2GUC_REGISTER_G2G_REQUEST_MSG_LEN		(GUC_HXG_REQUEST_MSG_MIN_LEN + 3u)
#define HOST2GUC_REGISTER_G2G_REQUEST_MSG_0_MBZ		GUC_HXG_REQUEST_MSG_0_DATA0
#define HOST2GUC_REGISTER_G2G_REQUEST_MSG_1_MBZ		(0xffff << 16)
#define HOST2GUC_REGISTER_G2G_REQUEST_MSG_1_DEST	(0xf << 12)
#define HOST2GUC_REGISTER_G2G_REQUEST_MSG_1_TYPE	(0xf << 8)
#define   GUC_G2G_TYPE_IN				0u
#define   GUC_G2G_TYPE_OUT				1u
#define HOST2GUC_REGISTER_G2G_REQUEST_MSG_1_SIZE	(0xff << 0)
#define HOST2GUC_REGISTER_G2G_REQUEST_MSG_2_DESC_ADDR	GUC_HXG_REQUEST_MSG_n_DATAn
#define HOST2GUC_REGISTER_G2G_REQUEST_MSG_3_BUFF_ADDR	GUC_HXG_REQUEST_MSG_n_DATAn

#define HOST2GUC_REGISTER_G2G_RESPONSE_MSG_LEN		GUC_HXG_RESPONSE_MSG_MIN_LEN
#define HOST2GUC_REGISTER_G2G_RESPONSE_MSG_0_MBZ	GUC_HXG_RESPONSE_MSG_0_DATA0

/**
 * DOC: HOST2GUC_DEREGISTER_G2G
 *
 * This message is used to unregister a buffer used for communication between
 * GuC instances in multi-tile environments.
 *
 * This message must be sent as `CTB H2G Message`_.
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
 *  |   |  15:0 | ACTION = _`GUC_ACTION_HOST2GUC_DEREGISTER_G2G` = 0x5203      |
 *  +---+-------+--------------------------------------------------------------+
 *  | 1 | 31:16 | RESERVED = MBZ                                               |
 *  |   +-------+--------------------------------------------------------------+
 *  |   | 15:12 | **DEST** - Tile ID associated with destination               |
 *  |   +-------+--------------------------------------------------------------+
 *  |   |  11:8 | **TYPE** - type of the G2G buffer                            |
 *  |   |       |                                                              |
 *  |   |       | see _`GUC_ACTION_HOST2GUC_REGISTER_G2G`                      |
 *  |   +-------+--------------------------------------------------------------+
 *  |   |   7:0 | RESERVED = MBZ                                               |
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
#define GUC_ACTION_HOST2GUC_DEREGISTER_G2G		0x4508 /* FIXME: 0x5203 */

#define HOST2GUC_DEREGISTER_G2G_REQUEST_MSG_LEN		(GUC_HXG_REQUEST_MSG_MIN_LEN + 1u)
#define HOST2GUC_DEREGISTER_G2G_REQUEST_MSG_0_MBZ	GUC_HXG_REQUEST_MSG_0_DATA0
#define HOST2GUC_DEREGISTER_G2G_REQUEST_MSG_1_MBZ	(0xffff << 16)
#define HOST2GUC_DEREGISTER_G2G_REQUEST_MSG_1_DEST	(0xf << 12)
#define HOST2GUC_DEREGISTER_G2G_REQUEST_MSG_1_TYPE	(0xf << 8)
#define HOST2GUC_DEREGISTER_G2G_REQUEST_MSG_1_MBZ2	(0xff << 0)

#define HOST2GUC_DEREGISTER_G2G_RESPONSE_MSG_LEN	GUC_HXG_RESPONSE_MSG_MIN_LEN
#define HOST2GUC_DEREGISTER_G2G_RESPONSE_MSG_0_MBZ	GUC_HXG_RESPONSE_MSG_0_DATA0

#endif /* _ABI_GUC_ACTIONS_PRIVILEGED_ABI_H */
