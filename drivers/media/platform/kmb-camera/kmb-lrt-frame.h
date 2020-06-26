/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Keem Bay Camera LRT frame data
 *
 * Copyright (C) 2018-2020 Intel Corporation
 */

#ifndef KMB_LRT_FRAME_H_
#define KMB_LRT_FRAME_H_

typedef enum frameTypes {
	YUV422i,   // interleaved 8 bit
	YUV444p,   // planar 4:4:4 format
	YUV420p,   // planar 4:2:0 format
	YUV422p,   // planar 8 bit
	YUV400p,   // 8-bit greyscale
	RGBA8888,  // RGBA interleaved stored in 32 bit word
	RGB888,	// Planar 8 bit RGB data
	LUT2,	  // 1 bit  per pixel, Lookup table (used for graphics layers)
	LUT4,	  // 2 bits per pixel, Lookup table (used for graphics layers)
	LUT16,	 // 4 bits per pixel, Lookup table (used for graphics layers)
	RAW16,	 // save any raw type (8, 10, 12bit) on 16 bits
	RAW14,	 // 14bit value in 16bit storage
	RAW12,	 // 12bit value in 16bit storage
	RAW10,	 // 10bit value in 16bit storage
	RAW8,
	PACK10,	// SIPP 10bit packed format
	PACK12,	// SIPP 12bit packed format
	YUV444i,
	NV12,
	NV21,
	BITSTREAM, // used for video encoder bitstream
	HDR,
	NV12PACK10, // nv12 format with pixels encoded in pack 10
	NONE
} frameType;

typedef struct frameSpecs {
	uint16_t   type;   // Values from frameType
	uint16_t   height; // width in pixels
	uint16_t   width;  // width in pixels
	/*
	 * defined as distance in bytes from pix(y,x) to pix(y+1,x)
	 */
	uint16_t   stride;
	/*
	 * Bits per pixel (for unpacked types set 8 or 16, for NV12 set only
	 * luma pixel size)
	 */
	uint16_t   bitsPP;
} frameSpec;

typedef struct frameElements {
	frameSpec spec;
	uint64_t p1;  // Addr to first image plane
	uint64_t p2;  // Addr to second image plane (if used)
	uint64_t p3;  // Addr to third image plane  (if used)
	int64_t  ts;  // Timestamp in NS
} frameBuffer;

#endif /* KMB_LRT_FRAME_H_ */
