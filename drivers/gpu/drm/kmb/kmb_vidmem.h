// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright © 2018-2020 Intel Corporation
 */

#ifndef __KMB_VIDMEM_H__
#define __KMB_VIDMEM_H__


#define HANTRO_MAGIC(ch0, ch1, ch2, ch3) \
	    ((unsigned long)(unsigned char) (ch0) | \
	    ((unsigned long)(unsigned char) (ch1) << 8) | \
	    ((unsigned long)(unsigned char) (ch2) << 16) | \
	    ((unsigned long)(unsigned char) (ch3) << 24 ))

#define HANTRO_IMAGE_VIV_META_DATA_MAGIC HANTRO_MAGIC('V','I','V','M')


struct viv_vidmem_metadata
{
    uint32_t magic;        // __FOURCC('v', 'i', 'v', 'm')
    uint32_t dmabuf_size;  // DMABUF buffer size in byte (Maximum 4GB)
    uint32_t time_stamp;   // time stamp for the DMABUF buffer

    uint32_t image_format; // ImageFormat, determined plane number.
    uint32_t compressed;   // if DMABUF buffer is compressed by DEC400
    struct {
        uint32_t offset; // plane buffer address offset from DMABUF address
        uint32_t stride; // pitch in bytes
        uint32_t width;  // width in pixels
        uint32_t height; // height in pixels

        uint32_t tile_format; // uncompressed tile format
        uint32_t compress_format; // tile mode for DEC400

        /** tile status buffer offset within this plane buffer. when it is 0，
         *  indicates using seperate tile status buffer
         */
        uint32_t ts_offset;
        /** fd of seperate tile status buffer of the plane buffer */
        int32_t ts_fd;
        /** valid fd of the ts buffer in consumer side. */
        int32_t ts_fd2;
        /** the vpu virtual address for this ts data buffer */
        int32_t  ts_vaddr;

        /** gpu fastclear enabled for the plane buffer */
        uint32_t fc_enabled;
        /** gpu fastclear color value (lower 32 bits) for the plane buffer */
        uint32_t fc_value_lower;
        /** gpu fastclear color value (upper 32 bits) for the plane buffer */
        uint32_t fc_value_upper;
    } plane[3];
};
#endif /*__KMB_VIDMEM_H__*/
