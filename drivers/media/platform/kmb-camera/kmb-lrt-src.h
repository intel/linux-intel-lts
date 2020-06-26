/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Keem Bay Camera LRT Source Configuration
 *
 * Copyright (C) 2018-2020 Intel Corporation
 */

#ifndef KMB_LRT_SRC_H
#define KMB_LRT_SRC_H

/*
 * Specific #defines
 */
typedef struct {
	uint32_t w;
	uint32_t h;
} icImgSize;

typedef struct {
	int32_t	 x1;
	int32_t	 y1;
	int32_t	 x2;
	int32_t	 y2;
} icImgRect;

// 6 hw mipi/cif input device supported by myriad2
typedef enum {
	IC_SOURCE_0 = 0,
	IC_SOURCE_1 = 1,
	IC_SOURCE_2 = 2,
	IC_SOURCE_3 = 3,
	IC_SOURCE_4 = 4,
	IC_SOURCE_5 = 5,
} icSourceInstance;

/* Bayer pattern order */
typedef enum {
	IC_BAYER_FORMAT_GRBG = 0,
	IC_BAYER_FORMAT_RGGB = 1,
	IC_BAYER_FORMAT_GBRG = 2,
	IC_BAYER_FORMAT_BGGR = 3,
} icBayerFormat;

/*
 * List of myriad receiver Id's  (unified list) for a specific sensor read the
 * documentation, as IC_MIPI_CTRL_0 can be connected just to 2 different
 * receivers
 */
typedef enum {
	IC_SIPP_DEVICE0 = 0,
	IC_SIPP_DEVICE1 = 1,
	IC_SIPP_DEVICE2 = 2,
	IC_SIPP_DEVICE3 = 3,
	IC_CIF0_DEVICE4 = 4,
	IC_CIF1_DEVICE5 = 5
} IcMipiRxCtrlRecNoT;

// all mipi controller from chip
typedef enum {
	IC_MIPI_CTRL_0 = 0,
	IC_MIPI_CTRL_1,
	IC_MIPI_CTRL_2,
	IC_MIPI_CTRL_3,
	IC_MIPI_CTRL_4,
	IC_MIPI_CTRL_5
} IcMipiRxCtrlNoT;

// all sported raw, sensor input formats
typedef enum {
	IC_IPIPE_YUV_420_B8	= 0x18,
	IC_IPIPE_RAW_8		= 0x2A, // feet to mipi CSI standard
	IC_IPIPE_RAW_10		= 0x2B,
	IC_IPIPE_RAW_12		= 0x2C,
	IC_IPIPE_RAW_14		= 0x2D,
	IC_IPIPE_EMBEDDED_8BIT	= 0x12
} IcMipiRxDataTypeT;

/*
 * Per-source configuration of parameters which can be modified dynamically
 * (the source does not need to be stopped).  Settings will take effect during
 * the next blanking interval.
 */
typedef struct {
	/*
	 * Line number upon which IC_EVENT_TYPE_LINE REACHED will be sent
	 * to the Leon OS.  Set to -1 to disable notification.
	 */
	int32_t	 notificationLine;
} icSourceConfigDynamic;

// mipiRx, receiver Configuration structure
typedef struct {
	uint32_t controllerNo;
	uint32_t noLanes;
	uint32_t laneRateMbps;
	uint32_t dataType;
	uint32_t dataMode;
	uint32_t recNrl;
} icMipiConfig;

typedef struct {
	icImgSize cameraOutputSize;
	icImgRect cropWindow;

	/*
	 * Bayer Format - Raw, Demosaic and LSC blocks should be programmed
	 * to match the Bayer order specified here.
	 */
	uint32_t bayerFormat;
	uint32_t bitsPerPixel;

	/* mipi RX data configuration	 */
	icMipiConfig	mipiRxData;

	uint32_t noExposure;
	uint32_t metadataWidth;
	uint32_t metadataHeight;
	uint32_t metadataDataType;
	uint32_t padding[13]; // keep sizeof type multiple of 64 bytes
} icSourceConfig;

#endif  /* KMB_LRT_SRC_H */
