/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright © 2018-2020 Intel Corporation
 */

#ifndef __KMB_VIDMEM_H__
#define __KMB_VIDMEM_H__

#define HANTRO_MAGIC(ch0, ch1, ch2, ch3) \
	    ((unsigned long)(unsigned char)(ch0) | \
	    ((unsigned long)(unsigned char)(ch1) << 8) | \
	    ((unsigned long)(unsigned char)(ch2) << 16) | \
	    ((unsigned long)(unsigned char)(ch3) << 24))

#define HANTRO_IMAGE_VIV_META_DATA_MAGIC HANTRO_MAGIC('V', 'I', 'V', 'M')

struct viv_vidmem_metadata {
	u32 magic;        // __FOURCC('v', 'i', 'v', 'm')
	u32 dmabuf_size;  // DMABUF buffer size in byte (Maximum 4GB)
	u32 time_stamp;   // time stamp for the DMABUF buffer

	u32 image_format; // ImageFormat, determined plane number.
	u32 compressed;   // if DMABUF buffer is compressed by DEC400
	struct {
	u32 offset; // plane buffer address offset from DMABUF address
	u32 stride; // pitch in bytes
	u32 width;  // width in pixels
	u32 height; // height in pixels

	u32 tile_format; // uncompressed tile format
	u32 compress_format; // tile mode for DEC400

	/** tile status buffer offset within this plane buffer. when it is 0，
	 *  indicates using separate tile status buffer
	 */
	u32 ts_offset;
	/** fd of separate tile status buffer of the plane buffer */
	u32 ts_fd;
	/** valid fd of the ts buffer in consumer side. */
	u32 ts_fd2;
	/** the vpu virtual address for this ts data buffer */
	u32 ts_vaddr;

	/** gpu fastclear enabled for the plane buffer */
	u32 fc_enabled;
	/** gpu fastclear color value (lower 32 bits) for the plane buffer */
	u32 fc_value_lower;
	/** gpu fastclear color value (upper 32 bits) for the plane buffer */
	u32 fc_value_upper;
	} plane[3];
};
#endif /*__KMB_VIDMEM_H__*/
