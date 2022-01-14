/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * kmb-isp-ctl.h - KeemBay Camera custom events and ioctls.
 *
 * Copyright (C) 2018-2019 Intel Corporation
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
#define V4L2_CID_ROW_TIME_NS			(V4L2_CID_USER_BASE | 0x9)
#define V4L2_CID_SYNC_TYPE			(V4L2_CID_USER_BASE | 0xA)
#define V4L2_CID_DIGITAL_GAIN_SHORT		(V4L2_CID_USER_BASE | 0xB)
#define V4L2_CID_DIGITAL_GAIN_VERY_SHORT	(V4L2_CID_USER_BASE | 0xC)
#define V4L2_CID_SENSOR_SERIAL_NUMBER		(V4L2_CID_USER_BASE | 0xD)

/* MIPI Specification YUV 8 Bit 420 (DT 0x18) */
#define V4L2_PIX_FMT_YUV420_MIPI8	v4l2_fourcc('Y', 'U', 'V', '8')
/* MIPI Specification YUV 10 Bit 420 (DT 0x19) */
#define V4L2_PIX_FMT_YUV420_MIPI10	v4l2_fourcc('Y', 'U', '1', '0')
/* MIPI Specification YUV 10 Bit 420 Packed (DT 0x19) */
#define V4L2_PIX_FMT_YUV420_MIPI10P	v4l2_fourcc('Y', '1', '0', 'P')
/* MIPI Specification YUV 8 Bit 420 Legacy (DT 0x1A) */
#define V4L2_PIX_FMT_YUV420_MIPI8L	v4l2_fourcc('Y', 'U', '8', 'L')
/* MIPI Specification YUV 8 Bit 420 Chroma separated pixel sampling */
#define V4L2_PIX_FMT_YUV420_MIPI8CS	v4l2_fourcc('Y', 'U', '8', 'C')
/* MIPI Specification YUV 10 Bit 420 Chroma separated pixel sampling */
#define V4L2_PIX_FMT_YUV420_MIPI10CS	v4l2_fourcc('Y', '1', '0', 'C')
/* MIPI Specification YUV 10 Bit 420 Packed Chroma separated pixel sampling */
#define V4L2_PIX_FMT_YUV420_MIPI10CSP	v4l2_fourcc('1', '0', 'C', 'P')
/* MIPI Specification YUV 8 Bit 422 (DT 0x1E) */
#define V4L2_PIX_FMT_YUV422_MIPI8	v4l2_fourcc('4', '2', '2', '8')
/* MIPI Specification YUV 10 Bit 422 (DT 0x1F) */
#define V4L2_PIX_FMT_YUV422_MIPI10	v4l2_fourcc('4', '2', '2', '1')
/* MIPI Specification YUV 10 Bit 422 Packed (DT 0x1F) */
#define V4L2_PIX_FMT_YUV422_MIPI10P	v4l2_fourcc('4', '1', '0', 'P')

/**
 * kmb_isp_cfg_event_data - KMB ISP configuration event private data
 *
 * @valid: flag to indicate whether index is valid
 * @index: isp configuration buffer index
 * @ts: isp event timestamp with nano seconds resolution
 */
struct kmb_isp_cfg_event_data {
	__u8 valid;
	__u8 index;
	__u64 ts;
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
 * @KMB_CAMERA_MODE_NO_ISP:TODO
 * @KMB_CAMERA_MODE_MONO: monochromatic sensor mode
 */
enum kmb_camera_mode {
	KMB_CAMERA_MODE_ULL = 0,
	KMB_CAMERA_MODE_HDR_2DOL = 1,
	KMB_CAMERA_MODE_HDR_3DOL = 2,
	KMB_CAMERA_MODE_NO_ISP = 3,
	KMB_CAMERA_MODE_MONO = 4,
};

/**
 * kmb_sync_mode - KMB sensor sync mode
 *
 * @KMB_SYNC_NONE: No sync
 * @KMB_SYNC_MAIN: Main sync mode
 * @KMB_SYNC_AUX: Auxiliary sync mode
 */
enum kmb_sync_mode {
	KMB_SYNC_NONE = 0,
	KMB_SYNC_MAIN = 1,
	KMB_SYNC_AUX = 2
};

/**
 * kmb_sync_type - KMB sensor sync type
 *
 * @KMB_SYNC_TYPE_HW: HW sync with XVS
 * @KMB_SYNC_TYPE_SW: SW sync with start stream command on shared CCI
 */
enum kmb_sync_type {
	KMB_SYNC_TYPE_HW = 0,
	KMB_SYNC_TYPE_SW = 1,
};

/**
 * kmb_isp_hdr_type - KMB HDR type
 *
 * @NO_HDR: non-HDR mode is selected
 * @HDR_2DOL_LI: two frame digital overlap HDR mode, Line Information
 * @HDR_2DOL_VC: two frame digital overlap HDR mode, Virtual Channel
 * @HDR_3DOL_VC: three frame digital overlap HDR mode, Virtual Channel
 * @HDR_3DOL_LI: three frame digital overlap HDR mode, Line Information
 */
enum kmb_isp_hdr_type {
	NO_HDR = 0,
	HDR_2DOL_LI = 1,
	HDR_2DOL_VC = 2,
	HDR_3DOL_VC = 3,
	HDR_3DOL_LI = 4,
};

/**
 * enum kmb_camera_test_pattern_type - KMB Smart sensor test pattern type
 * @KMB_CAMERA_TEST_PATTERN_REPEATING - Repeating pattern where each pixel has
 *                                      a value of 0 to max_val max_val depends
 *                                      on data type bit depth. Once max val is
 *                                      reached, the pattern repeats. Does not
 *                                      care about bayer pattern or YUV data
 *                                      layout.
 * @KMB_CAMERA_TEST_PATTERN_REPEATING_SEED - Generates a pattern based on host
 *                                           seed. The pattern is a pure byte
 *                                           stream, where the first byte is
 *                                           the seed used for generation for
 *                                           easier verification on host side.
 * @KMB_CAMERA_TEST_PATTERN_REPEATING_SEED_TS - Same as above, but uses current
 *                                              system timestamp as seed value.
 * @KMB_CAMERA_TEST_PATTERN_COLOR_BARS - Color bars test pattern.
 */
enum kmb_camera_test_pattern_type {
	KMB_CAMERA_TEST_PATTERN_REPEATING = 0,
	KMB_CAMERA_TEST_PATTERN_REPEATING_SEED = 1,
	KMB_CAMERA_TEST_PATTERN_REPEATING_SEED_TS = 2,
	KMB_CAMERA_TEST_PATTERN_COLOR_BARS = 3,
};

/**
 * kmb_isp_hdr_info - KMB HDR information
 *
 * @type: HDR type
 * @offset0: long exposure offset, valid only in HDR_LINE_INTERLEAVED
 * @offset1: short exposure offset, valid only in HDR_LINE_INTERLEAVED
 * @offset2: very short exposure offset, valid only in HDR_LINE_INTERLEAVED
 */
struct kmb_isp_hdr_info {
	__u16 type;
	__u32 offset0;
	__u32 offset1;
	__u32 offset2;
};

#endif /* KMB_ISP_CTL_H */
