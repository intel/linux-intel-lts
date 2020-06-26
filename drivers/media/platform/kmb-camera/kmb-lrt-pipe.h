/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Keem Bay Camera LRT pipe definitions
 *
 * Copyright (C) 2018-2020 Intel Corporation
 */
#ifndef KMB_LRT_PIPE_H
#define KMB_LRT_PIPE_H

#include "kmb-lrt-src.h"

enum {
	PIPE_TYPE_ISP_ISP_ULL = 1,
	PIPE_TYPE_ISP_ISP_2DOL,
	PIPE_TYPE_ISP_ISP_3DOL,
	PIPE_TYPE_ISP_ISP_MONO,

	PIPE_TYPE_MAX
};

enum {
	SRC_TYPE_ALLOC_VPU_DATA_MIPI = 0,
	SRC_TYPE_ALLOC_VPU_DATA_DBG,
	SRC_TYPE_ALLOC_ARM_DATA_ARM,
	SRC_TYPE_ALLOC_ARM_DATA_MIPI,
	SRC_TYPE_ALLOC_ARM_DATA_DBG,

	SRC_TYPE_ALLOC_DATA_MAX
};

enum {
	PIPE_TRANSFORM_HUB_NONE	= 0,
	PIPE_TRANSFORM_HUB_BASIC,
	PIPE_TRANSFORM_HUB_FULL,
	PIPE_TRANSFORM_HUB_STITCH,
	PIPE_TRANSFORM_HUB_EPTZ,

	PIPE_TRANSFORM_HUB_MAX
};

enum {
	PIPE_OUTPUT_ID_RAW = 0,
	PIPE_OUTPUT_ID_ISP_CTRL,
	PIPE_OUTPUT_ID_0,
	PIPE_OUTPUT_ID_1,
	PIPE_OUTPUT_ID_2,
	PIPE_OUTPUT_ID_3,
	PIPE_OUTPUT_ID_4,
	PIPE_OUTPUT_ID_5,
	PIPE_OUTPUT_ID_6,

	PIPE_OUTPUT_ID_MAX
};
// IC_EVENT_TYPE enum to define event messages

typedef struct {
	uint32_t  chID;
	icImgSize frmRes;
} channelCfg;

typedef struct __aligned(64) pipeConfigEvS {
	uint8_t	   pipeID;
	uint8_t	   pipeType;
	uint8_t	   srcType;
	uint8_t	   pipeTransHub;
	icImgSize  inIspRes;
	icImgSize  outMinRes[PIPE_OUTPUT_ID_MAX];
	icImgSize  outMaxRes[PIPE_OUTPUT_ID_MAX];
	channelCfg pipeXlinkChann[PIPE_OUTPUT_ID_MAX];
	uint8_t	   keepAspectRatio;
	uint8_t	   inDataWidth;
	uint8_t	   inDataPacked;
	uint8_t	   outDataWidth;
} pipeConfigEv;

#endif  /* KMB_LRT_PIPE_H */
