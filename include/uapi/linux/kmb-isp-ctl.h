/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Keem Bay Camera custom events and ioctls.
 *
 * Copyright (C) 2018-2020 Intel Corporation
 */
#ifndef KMB_ISP_CTL_H
#define KMB_ISP_CTL_H

#include <linux/types.h>

#define V4L2_EVENT_KMB_CAMERA_CLASS	(V4L2_EVENT_PRIVATE_START)
#define V4L2_EVENT_KMB_SENSOR_SOF	(V4L2_EVENT_KMB_CAMERA_CLASS | 0x1)
#define V4L2_EVENT_KMB_SENSOR_EOF	(V4L2_EVENT_KMB_CAMERA_CLASS | 0x2)
#define V4L2_EVENT_KMB_ISP_SOF		(V4L2_EVENT_KMB_CAMERA_CLASS | 0x3)
#define V4L2_EVENT_KMB_ISP_EOF		(V4L2_EVENT_KMB_CAMERA_CLASS | 0x4)
#define V4L2_EVENT_KMB_CFG_SKIPPED	(V4L2_EVENT_KMB_CAMERA_CLASS | 0x5)
#define V4L2_EVENT_KMB_CFG_MISSING	(V4L2_EVENT_KMB_CAMERA_CLASS | 0x6)
#define V4L2_EVENT_KMB_BUF_MISSING	(V4L2_EVENT_KMB_CAMERA_CLASS | 0x7)
#define V4L2_EVENT_KMB_GEN_ERROR	(V4L2_EVENT_KMB_CAMERA_CLASS | 0x8)

#define V4L2_CID_KMB_CAMERA_TRANSFORM_HUB	(V4L2_CID_USER_BASE | 0x1)
#define V4L2_CID_KMB_CAMERA_MODE		(V4L2_CID_USER_BASE | 0x2)
#define V4L2_CID_ANALOGUE_GAIN_SHORT		(V4L2_CID_USER_BASE | 0x3)
#define V4L2_CID_ANALOGUE_GAIN_VERY_SHORT	(V4L2_CID_USER_BASE | 0x4)
#define V4L2_CID_EXPOSURE_SHORT			(V4L2_CID_USER_BASE | 0x5)
#define V4L2_CID_EXPOSURE_VERY_SHORT		(V4L2_CID_USER_BASE | 0x6)
#define V4L2_CID_SYNC_MODE			(V4L2_CID_USER_BASE | 0x7)
#define V4L2_CID_SYNC_START			(V4L2_CID_USER_BASE | 0x8)

/**
 * kmb_isp_cfg_event_data - KMB ISP configuration event private data
 *
 * @valid: flag to indicate whether index is valid
 * @index: isp configuration buffer index
 */
struct kmb_isp_cfg_event_data {
	__u8 valid;
	__u8 index;
};

/**
 * kmb_camera_transform_hub - KMB Camera transform hub
 *
 * @KMB_CAMERA_TRANSFORM_HUB_NONE: single output mode with no resizer
 * @KMB_CAMERA_TRANSFORM_HUB_BASIC: up to two outputs, one can be resized
 * @KMB_CAMERA_TRANSFORM_HUB_FULL: up to three outputs, two can be resized
 * @KMB_CAMERA_TRANSFORM_HUB_STITCHING: 2/4 inputs stitched into up to two
 *					outputs, one of which can be resized
 * @KMB_CAMERA_TRANSFORM_HUB_EPTZ: electronic pan/tilt/zoom with one input and
 *				   up to six outputs which can be resized
 */
enum kmb_camera_transform_hub {
	KMB_CAMERA_TRANSFORM_HUB_NONE = 0,
	KMB_CAMERA_TRANSFORM_HUB_BASIC = 1,
	KMB_CAMERA_TRANSFORM_HUB_FULL = 2,
	KMB_CAMERA_TRANSFORM_HUB_STITCHING = 3,
	KMB_CAMERA_TRANSFORM_HUB_EPTZ = 4,
};

/**
 * kmb_camera_mode - KMB Camera sensor mode
 *
 * @KMB_CAMERA_MODE_ULL: ultra low light sensor mode
 * @KMB_CAMERA_MODE_HDR_2DOL: two frame digital overlap HDR mode
 * @KMB_CAMERA_MODE_HDR_3DOL: three frame digital overlap HDR mode
 * @KMB_CAMERA_MODE_MONO: monochromatic seonsor mode
 */
enum kmb_camera_mode {
	KMB_CAMERA_MODE_ULL = 0,
	KMB_CAMERA_MODE_HDR_2DOL = 1,
	KMB_CAMERA_MODE_HDR_3DOL = 2,
	KMB_CAMERA_MODE_MONO = 3
};

/**
 * kmb_hw_sync_mode - KMB sensor hw sync mode
 *
 * @KMB_HW_SYNC_NONE: No hw sync
 * @KMB_HW_SYNC_MASTER: HW master sync mode
 * @KMB_HW_SYNC_SLAVE: HW slave sync mode
 */
enum kmb_hw_sync_mode {
	KMB_HW_SYNC_NONE = 0,
	KMB_HW_SYNC_MASTER = 1,
	KMB_HW_SYNC_SLAVE = 2
};

#endif /* KMB_ISP_CTL_H */
