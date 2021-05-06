/* SPDX-License-Identifier: GPL-2.0 */
/*
 *    Hantro Metadata header file.
 *
 *    Copyright (c) 2017 - 2020, VeriSilicon Inc.
 *    Copyright (c) 2020 - 2021, Intel Corporation
 */

#ifndef HANTRO_METADATA_H
#define HANTRO_METADATA_H

#include <linux/platform_device.h>

#define HANTRO_MAGIC(ch0, ch1, ch2, ch3)                                       \
	((unsigned long)(unsigned char)(ch0) |                                 \
	 ((unsigned long)(unsigned char)(ch1) << 8) |                          \
	 ((unsigned long)(unsigned char)(ch2) << 16) |                         \
	 ((unsigned long)(unsigned char)(ch3) << 24))

#define HANTRO_IMAGE_VIV_META_DATA_MAGIC HANTRO_MAGIC('V', 'I', 'V', 'M')

/*
 * name of image format for exchange.
 */
enum viv_image_format {
	IMAGE_BYTE = 0, // pure data, not image
	IMAGE_YUV420, // 3 plane YUV420 8 bits
	IMAGE_NV12, // 2 plane NV12 8 bits
	IMAGE_UYVY, // packed YUV422 8 bits
	IMAGE_YUY2, // packed YUV422 8 bits
	IMAGE_Y8, // single plane 8 bits
	IMAGE_UV8, // single UV interleave plan 8 bits
	IMAGE_MS_P010, // 2 plane Simiplanar 10 bits, MSB valid
	IMAGE_P010, // 3 plane YUV420 10 bits, MSB valid
	IMAGE_Y210, // packed YUV422 10 bits, LSB valid
	IMAGE_Y10, // single plane 10 bits
	IMAGE_UV10, // single UV interleave plan 10 bits
	IMAGE_XRGB8888, // packed XRGB 8 bits
	IMAGE_ARGB8888,
	IMAGE_A2R10G10B10,
	IMAGE_X2R10G10B10,
	IMAGE_BAYER10, // packed
	IMAGE_BAYER12,
	IMAGE_BAYER14,
	IMAGE_BAYER16,
	IMAGE_FORMAT_MAX
};

/*
 * dec400 tile format, defines how one tile is scanned.
 */
enum viv_tile_format {
	TILE_NONE = 0, // raster scane linear
	TILE_Y_4x4, // luma 4x4
	TILE_Y_8x8, // luma 8x8
	TILE_4x4_INTERLEAVE, // U_4x4 tile followed with V4x4,
	TILE_8x8_X_MAJOR, // RGB super tile X majory
	TILE_8x8_Y_MAJOR, // RGB super tile y major
	TILE_FMT_MAX
};

/*
 * dec400 tile mode, defines the tile size for each compression block
 */
enum viv_dec_tile_mode {
	CMVDEC_TILE_MODE_8x8_X_MAJOR = 0,
	CMVDEC_TILE_MODE_8x8_Y_MAJOR = 1,
	CMVDEC_TILE_MODE_16x4 = 2,
	CMVDEC_TILE_MODE_8x4 = 3,
	CMVDEC_TILE_MODE_4x8 = 4,
	CMVDEC_TILE_MODE_4x4 = 5,
	CMVDEC_TILE_MODE_16x4_RASTER = 6,
	CMVDEC_TILE_MODE_64x4 = 7,
	CMVDEC_TILE_MODE_32x4 = 8,
	CMVDEC_TILE_MODE_256x1_RASTER = 9,
	CMVDEC_TILE_MODE_128x1_RASTER = 10,
	CMVDEC_TILE_MODE_64x4_RASTER = 11,
	CMVDEC_TILE_MODE_256x2_RASTER = 12,
	CMVDEC_TILE_MODE_128x2_RASTER = 13,
	CMVDEC_TILE_MODE_128x4_RASTER = 14,
	CMVDEC_TILE_MODE_64x1_RASTER = 15,
	CMVDEC_TILE_MODE_16x8_RASTER = 16,
	CMVDEC_TILE_MODE_8x16_RASTER = 17,
	CMVDEC_TILE_MODE_512x1_RASTER = 18,
	CMVDEC_TILE_MODE_32x4_RASTER = 19,
	CMVDEC_TILE_MODE_64x2_RASTER = 20,
	CMVDEC_TILE_MODE_32x2_RASTER = 21,
	CMVDEC_TILE_MODE_32x1_RASTER = 22,
	CMVDEC_TILE_MODE_16x1_RASTER = 23,
	CMVDEC_TILE_MODE_128x4 = 24,
	CMVDEC_TILE_MODE_256x4 = 25,
	CMVDEC_TILE_MODE_512x4 = 26,
	CMVDEC_TILE_MODE_16x16 = 27,
	CMVDEC_TILE_MODE_32x16 = 28,
};

/*
 * The surface meta data saved in meta data buffer
 */
struct viv_vidmem_metadata {
	u32 magic; // __FOURCC('V', 'I', 'V', 'M')
	u32 dmabuf_size; // DMABUF buffer size in byte (Maximum 4GB)
	u32 time_stamp; // time stamp for the DMABUF buffer

	u32 image_format; // ImageFormat, determined plane number.
	u32 compressed; // if DMABUF buffer is compressed by DEC400
	struct {
		u32 offset; // plane buffer address offset from DMABUF address
		u32 stride; // pitch in bytes
		u32 width; // width in pixels
		u32 height; // height in pixels

		u32 tile_format; // uncompressed tile format
		u32 compress_format; // tile mode for DEC400

		/*
		 * tile status buffer offset within this plane buffer. when it is 0，
		 * indicates using separate tile status buffer
		 */
		u32 ts_offset;
		/* fd of separate tile status buffer of the plane buffer */
		s32 ts_fd;
		/* valid fd of the ts buffer in consumer side. */
		s32 ts_fd2;
		/* the vpu virtual address for this ts data buffer */
		s32 ts_vaddr;

		/* gpu fastclear enabled for the plane buffer */
		u32 fc_enabled;
		/* gpu fastclear color value (lower 32 bits) for the plane buffer */
		u32 fc_value_lower;
		/* gpu fastclear color value (upper 32 bits) for the plane buffer */
		u32 fc_value_upper;
	} plane[3];
};

/*
 * The meta data location information exchange IOCTL parameters.
 */
struct hantro_metainfo_params {
	int handle;				/* the handle of current bo */
	struct viv_vidmem_metadata info;	/* the meta data */
};

struct dmapriv {
	struct viv_vidmem_metadata meta_data_info;
	u32 magic_num;	/* HANTRO_IMAGE_VIV_META_DATA_MAGIC */
	void *self;	/* ptr of parent cma obj */
};

long hantrometadata_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

#endif /* HANTRO_METADATA_H */
