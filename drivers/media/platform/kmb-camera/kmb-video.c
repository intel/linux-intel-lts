// SPDX-License-Identifier: GPL-2.0-only
/*
 * Keem Bay Camera Video node Driver.
 *
 * Copyright (C) 2018-2020 Intel Corporation
 */
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-mc.h>

#include "kmb-video.h"
#include "kmb-lrt-pipe.h"
#include "kmb-lrt-frame.h"

#define KMB_RT_VID_CH_DATA_SIZE		1024
#define KMB_RT_VID_CH_TIMEOUT_MS	5000
#define RT_DATA_CH_ID_VID_BASE		50
#define RT_DATA_CH_ID_RAW_IN		40

#define KMB_VIDEO_TYPE_IS_RAW(ptr_kmb_vid)		\
	(kmb_vid->type == KMB_VIDEO_RAW_INPUT ||	\
	kmb_vid->type == KMB_VIDEO_RAW_OUTPUT)

/* Kmb video format info structure */
struct kmb_video_fmt_info {
	const char *description;
	u32 code;
	u32 pixelformat;
	frameType type;
	u32 colorspace;
	unsigned int planes;
	unsigned int bpp;
	bool packed;
	unsigned int h_subsample;
	unsigned int v_subsample;
	bool contiguous_memory;
};

/* Supported video formats */
static const struct kmb_video_fmt_info video_formats[] = {
	{
		.description = "NV12",
		.code = MEDIA_BUS_FMT_YUYV8_1_5X8,
		.pixelformat = V4L2_PIX_FMT_NV12,
		.type = NV12,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.planes = 2,
		.bpp = 8,
		.packed = true,
		.h_subsample = 1,
		.v_subsample = 2,
		.contiguous_memory = true,
	},
	{
		.description = "Planar YUV 4:2:0",
		.code = MEDIA_BUS_FMT_UYYVYY8_0_5X24,
		.pixelformat = V4L2_PIX_FMT_YUV420,
		.type = YUV420p,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.planes = 3,
		.bpp = 8,
		.packed = true,
		.h_subsample = 2,
		.v_subsample = 2,
		.contiguous_memory = false,
	},
	{
		.description = "Planar YUV 4:4:4",
		.code = MEDIA_BUS_FMT_YUV8_1X24,
		.pixelformat = V4L2_PIX_FMT_YUV444,
		.type = YUV444p,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.planes = 3,
		.bpp = 8,
		.packed = false,
		.h_subsample = 1,
		.v_subsample = 1,
		.contiguous_memory = false,
	},
	{
		.description = "RAW 8 Garyscale",
		.code = MEDIA_BUS_FMT_Y8_1X8,
		.pixelformat = V4L2_PIX_FMT_GREY,
		.type = RAW8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.planes = 1,
		.bpp = 8,
		.packed = false,
		.h_subsample = 1,
		.v_subsample = 1,
		.contiguous_memory = false,
	},
	{
		.description = "RAW 10 Grayscale",
		.code = MEDIA_BUS_FMT_Y10_1X10,
		.pixelformat = V4L2_PIX_FMT_Y10,
		.type = RAW10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.planes = 1,
		.bpp = 10,
		.packed = false,
		.h_subsample = 1,
		.v_subsample = 1,
		.contiguous_memory = false,
	}
};

static const struct kmb_video_fmt_info raw_formats[] = {
	{
		.description = "10bit Bayer BGGR format",
		.code = MEDIA_BUS_FMT_SBGGR10_1X10,
		.pixelformat = V4L2_PIX_FMT_SBGGR10,
		.type = RAW10,
		.colorspace = V4L2_COLORSPACE_RAW,
		.planes = 1,
		.bpp = 10,
		.packed = false,
		.h_subsample = 1,
		.v_subsample = 1,
		.contiguous_memory = true,
	},
	{
		.description = "10bit Bayer GBRG format",
		.code = MEDIA_BUS_FMT_SGBRG10_1X10,
		.pixelformat = V4L2_PIX_FMT_SGBRG10,
		.type = RAW10,
		.colorspace = V4L2_COLORSPACE_RAW,
		.planes = 1,
		.bpp = 10,
		.packed = false,
		.h_subsample = 1,
		.v_subsample = 1,
		.contiguous_memory = true,
	},
	{
		.description = "10bit Bayer GRBG format",
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.pixelformat = V4L2_PIX_FMT_SGRBG10,
		.type = RAW10,
		.colorspace = V4L2_COLORSPACE_RAW,
		.planes = 1,
		.bpp = 10,
		.packed = false,
		.h_subsample = 1,
		.v_subsample = 1,
		.contiguous_memory = true,
	},
	{
		.description = "10bit Bayer RGGB format",
		.code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.pixelformat = V4L2_PIX_FMT_SRGGB10,
		.type = RAW10,
		.colorspace = V4L2_COLORSPACE_RAW,
		.planes = 1,
		.bpp = 10,
		.packed = false,
		.h_subsample = 1,
		.v_subsample = 1,
		.contiguous_memory = true,
	},
	{
		.description = "10bit Bayer BGGR packed format",
		.code = MEDIA_BUS_FMT_SBGGR10_1X10,
		.pixelformat = V4L2_PIX_FMT_SBGGR10P,
		.type = RAW10,
		.colorspace = V4L2_COLORSPACE_RAW,
		.planes = 1,
		.bpp = 10,
		.packed = true,
		.h_subsample = 1,
		.v_subsample = 1,
		.contiguous_memory = true,
	},
	{
		.description = "10bit Bayer GBRG packed format",
		.code = MEDIA_BUS_FMT_SGBRG10_1X10,
		.pixelformat = V4L2_PIX_FMT_SGBRG10P,
		.type = RAW10,
		.colorspace = V4L2_COLORSPACE_RAW,
		.planes = 1,
		.bpp = 10,
		.packed = true,
		.h_subsample = 1,
		.v_subsample = 1,
		.contiguous_memory = true,
	},
	{
		.description = "10bit Bayer GRBG packed format",
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.pixelformat = V4L2_PIX_FMT_SGRBG10P,
		.type = RAW10,
		.colorspace = V4L2_COLORSPACE_RAW,
		.planes = 1,
		.bpp = 10,
		.packed = true,
		.h_subsample = 1,
		.v_subsample = 1,
		.contiguous_memory = true,
	},
	{
		.description = "10bit Bayer RGGB packed format",
		.code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.pixelformat = V4L2_PIX_FMT_SRGGB10P,
		.type = RAW10,
		.colorspace = V4L2_COLORSPACE_RAW,
		.planes = 1,
		.bpp = 10,
		.packed = true,
		.h_subsample = 1,
		.v_subsample = 1,
		.contiguous_memory = true,
	},
	{
		.description = "12bit Bayer BGGR format",
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.pixelformat = V4L2_PIX_FMT_SBGGR12,
		.type = RAW12,
		.colorspace = V4L2_COLORSPACE_RAW,
		.planes = 1,
		.bpp = 12,
		.packed = false,
		.h_subsample = 1,
		.v_subsample = 1,
		.contiguous_memory = true,
	},
	{
		.description = "12bit Bayer GBRG format",
		.code = MEDIA_BUS_FMT_SGBRG12_1X12,
		.pixelformat = V4L2_PIX_FMT_SGBRG12,
		.type = RAW12,
		.colorspace = V4L2_COLORSPACE_RAW,
		.planes = 1,
		.bpp = 12,
		.packed = false,
		.h_subsample = 1,
		.v_subsample = 1,
		.contiguous_memory = true,
	},
	{
		.description = "12bit Bayer GRBG format",
		.code = MEDIA_BUS_FMT_SGRBG12_1X12,
		.pixelformat = V4L2_PIX_FMT_SGRBG12,
		.type = RAW12,
		.colorspace = V4L2_COLORSPACE_RAW,
		.planes = 1,
		.bpp = 12,
		.packed = false,
		.h_subsample = 1,
		.v_subsample = 1,
		.contiguous_memory = true,
	},
	{
		.description = "12bit Bayer RGGB format",
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.pixelformat = V4L2_PIX_FMT_SRGGB12,
		.type = RAW12,
		.colorspace = V4L2_COLORSPACE_RAW,
		.planes = 1,
		.bpp = 12,
		.packed = false,
		.h_subsample = 1,
		.v_subsample = 1,
		.contiguous_memory = true,
	},
	{
		.description = "12bit Bayer BGGR packed format",
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.pixelformat = V4L2_PIX_FMT_SBGGR12P,
		.type = RAW12,
		.colorspace = V4L2_COLORSPACE_RAW,
		.planes = 1,
		.bpp = 12,
		.packed = true,
		.h_subsample = 1,
		.v_subsample = 1,
		.contiguous_memory = true,
	},
	{
		.description = "12bit Bayer GBRG packed format",
		.code = MEDIA_BUS_FMT_SGBRG12_1X12,
		.pixelformat = V4L2_PIX_FMT_SGBRG12P,
		.type = RAW12,
		.colorspace = V4L2_COLORSPACE_RAW,
		.planes = 1,
		.bpp = 12,
		.packed = true,
		.h_subsample = 1,
		.v_subsample = 1,
		.contiguous_memory = true,
	},
	{
		.description = "12bit Bayer GRBG packed format",
		.code = MEDIA_BUS_FMT_SGRBG12_1X12,
		.pixelformat = V4L2_PIX_FMT_SGRBG12P,
		.type = RAW12,
		.colorspace = V4L2_COLORSPACE_RAW,
		.planes = 1,
		.bpp = 12,
		.packed = true,
		.h_subsample = 1,
		.v_subsample = 1,
		.contiguous_memory = true,
	},
	{
		.description = "12bit Bayer RGGB packed format",
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.pixelformat = V4L2_PIX_FMT_SRGGB12P,
		.type = RAW12,
		.colorspace = V4L2_COLORSPACE_RAW,
		.planes = 1,
		.bpp = 12,
		.packed = true,
		.h_subsample = 1,
		.v_subsample = 1,
		.contiguous_memory = true,
	},
	{
		.description = "10bit Grayscale format",
		.code = MEDIA_BUS_FMT_Y10_1X10,
		.pixelformat = V4L2_PIX_FMT_Y10,
		.type = RAW10,
		.colorspace = V4L2_COLORSPACE_RAW,
		.planes = 1,
		.bpp = 10,
		.packed = false,
		.h_subsample = 1,
		.v_subsample = 1,
		.contiguous_memory = true,
	},
};

/**
 * kmb_video_from_v4l2 - Get kmb video device file handler
 * @fh: pointer to V4L2 file handler
 *
 * Get kmb video device file handler by given V4L2 file handler
 *
 * Return: pointer to kmb video file handler
 */
static inline struct kmb_video_fh *kmb_video_from_v4l2(struct v4l2_fh *fh)
{
	return container_of(fh, struct kmb_video_fh, fh);
}

/**
 * kmb_video_get_fmt_info_by_pixfmt - Get Keem Bay video format info
 * @vid_fh: pointer to V4L2 sub-device file handle
 * @pix_fmt: v4l2 pixel format
 *
 * Return: kmb video format info or NULL
 */
static const struct kmb_video_fmt_info *
kmb_video_get_fmt_info_by_pixfmt(struct kmb_video_fh *vid_fh, u32 pix_fmt)
{
	struct kmb_video *kmb_vid = vid_fh->kmb_vid;
	const struct kmb_video_fmt_info *arr = video_formats;
	int i, n = ARRAY_SIZE(video_formats);

	if (KMB_VIDEO_TYPE_IS_RAW(kmb_vid)) {
		arr = raw_formats;
		n = ARRAY_SIZE(raw_formats);
	}

	for (i = 0; i < n; i++) {
		if (arr[i].pixelformat == pix_fmt)
			return &arr[i];
	}
	return NULL;
}

/**
 * kmb_video_get_fmt_info_by_code - Get Keem Bay video format info
 * @vid_fh: pointer to V4L2 sub-device file handle
 * @code: media bus format code
 * @packed: requested format is packed/unpacked
 *
 * Use packed flag only for raw formats
 *
 * Return: kmb video format info or NULL
 */
static const struct kmb_video_fmt_info *
kmb_video_get_fmt_info_by_code(struct kmb_video_fh *vid_fh, u32 code,
			       bool packed)
{
	struct kmb_video *kmb_vid = vid_fh->kmb_vid;
	const struct kmb_video_fmt_info *arr = video_formats;
	int i, n = ARRAY_SIZE(video_formats);

	if (KMB_VIDEO_TYPE_IS_RAW(kmb_vid)) {
		arr = raw_formats;
		n = ARRAY_SIZE(raw_formats);

		for (i = 0; i < n; i++)
			if (arr[i].code == code && arr[i].packed == packed)
				return &arr[i];
	} else {
		for (i = 0; i < n; i++)
			if (arr[i].code == code)
				return &arr[i];
	}

	return NULL;
}

/**
 * kmb_video_insert_buf - Insert output buffer in queue
 * @vid_fh: pointer to V4L2 sub-device file handle
 * @buf: video output buffer to insert
 *
 * Return: 0 if successful
 */
static void kmb_video_insert_buf(struct kmb_video_fh *vid_fh,
				 struct kmb_frame_buffer *buf)
{
	mutex_lock(&vid_fh->lock);
	INIT_LIST_HEAD(&buf->list);
	list_add_tail(&buf->list, &vid_fh->dma_queue);
	mutex_unlock(&vid_fh->lock);
}

/**
 * kmb_video_buf_discard - Discard vb buffer
 * @buf: video output buffer
 */
static void kmb_video_buf_discard(struct kmb_frame_buffer *buf)
{
	list_del(&buf->list);
	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
}

/**
 * kmb_video_process_buf - Process output buffer
 * @vid_fh: pointer to V4L2 sub-device file handle
 * @buf: video output buffer to insert
 *
 * Return: 0 if successful
 */
static int kmb_video_process_buf(struct kmb_video_fh *vid_fh,
				 struct kmb_frame_buffer *buf)
{
	struct kmb_video *kmb_vid = vid_fh->kmb_vid;
	const struct kmb_video_fmt_info *info;
	frameBuffer rt_frame_buf;
	uint32_t rt_frame_buf_size;
	int ret;

	info = kmb_video_get_fmt_info_by_pixfmt(vid_fh,
			vid_fh->active_pix.pixelformat);
	if (!info)
		return -EINVAL;

	memset(&rt_frame_buf, 0x00, sizeof(rt_frame_buf));
	rt_frame_buf_size = sizeof(rt_frame_buf);
	rt_frame_buf.spec.bitsPP = info->packed ?
				   info->bpp : ALIGN(info->bpp, 8);
	rt_frame_buf.spec.type = info->type;
	rt_frame_buf.spec.width = vid_fh->active_pix.width;
	rt_frame_buf.spec.height = vid_fh->active_pix.height;
	rt_frame_buf.spec.stride = vid_fh->active_pix.plane_fmt[0].bytesperline;
	rt_frame_buf.p1 = buf->addr[0];

	if (vid_fh->active_pix.num_planes > 1)
		rt_frame_buf.p2 = buf->addr[1];

	if (vid_fh->active_pix.num_planes > 2)
		rt_frame_buf.p3 = buf->addr[2];


	dev_dbg(&kmb_vid->video->dev,
			"Frame buffer %dx%d stride %d num_palnes %u\n",
			rt_frame_buf.spec.width, rt_frame_buf.spec.height,
			rt_frame_buf.spec.stride,
			vid_fh->active_pix.num_planes);
	dev_dbg(&kmb_vid->video->dev, "Plane addr: p1 %llx p2 %llx p3 %llx\n",
			rt_frame_buf.p1, rt_frame_buf.p2, rt_frame_buf.p3);

	ret = xlink_write_volatile(kmb_vid->ipc_dev_handler,
			kmb_vid->channel_id,
			(u8 *)&rt_frame_buf,
			rt_frame_buf_size);
	if (ret)
		dev_err(&kmb_vid->video->dev,
				"Error on buffer queue %d\n", ret);

	return ret;
}

/**
 * kmb_video_process_all_bufs - Send all output buffers to LRT for processing
 * @vid_fh: pointer to V4L2 sub-device file handle
 *
 * Send each of the output buffers in dma queue to RT for processing. In case
 * a buffer can't be processed, its state is marked as error and the buffer is
 * dequeued.
 *
 * Return: none
 */
static void kmb_video_process_all_bufs(struct kmb_video_fh *vid_fh)
{
	struct kmb_frame_buffer *buf;
	struct list_head *pos, *next;
	int ret = 0;

	mutex_lock(&vid_fh->lock);
	list_for_each_safe(pos, next, &vid_fh->dma_queue) {
		buf = list_entry(pos, struct kmb_frame_buffer, list);

		ret = kmb_video_process_buf(vid_fh, buf);
		if (ret) {
			dev_err(&vid_fh->fh.vdev->dev,
				"Cannot process output buf 0x%pad\n",
				&buf->addr[0]);
			list_del(&buf->list);
			vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
			continue;
		}
	}
	mutex_unlock(&vid_fh->lock);
}

/**
 * kmb_video_queue_output_buf - Queue isp cfg buffer
 * @vid_fh: pointer to V4L2 sub-device file handle
 * @buf: video output buffer to queue
 *
 * Return: 0 if successful
 */
static int kmb_video_queue_output_buf(struct kmb_video_fh *vid_fh,
				      struct kmb_frame_buffer *buf)
{
	int ret = 0;

	kmb_video_insert_buf(vid_fh, buf);

	mutex_lock(&vid_fh->lock);

	/* Process out buf only when device is streaming */
	if (vb2_is_streaming(&vid_fh->vb2_q)) {
		ret = kmb_video_process_buf(vid_fh, buf);
		if (ret) {
			dev_err(&vid_fh->fh.vdev->dev,
				"Fail to process output buf 0x%pad\n",
				&buf->addr[0]);
			kmb_video_buf_discard(buf);
		}
	}

	mutex_unlock(&vid_fh->lock);

	return ret;
}

/**
 * kmb_video_release_all_bufs - Release all buffers in pending queue
 * @vid_fh: pointer to V4L2 sub-device file handle
 * @state: state of the buffers
 */
static void kmb_video_release_all_bufs(struct kmb_video_fh *vid_fh,
				       enum vb2_buffer_state state)
{
	struct kmb_frame_buffer *buf;
	struct list_head *pos, *next;

	mutex_lock(&vid_fh->lock);
	list_for_each_safe(pos, next, &vid_fh->dma_queue) {
		buf = list_entry(pos, struct kmb_frame_buffer, list);
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
	mutex_unlock(&vid_fh->lock);
}

/**
 * kmb_video_remove_buf - Remove output buffer from queue
 * @vid_fh: pointer to V4L2 sub-device file handle
 * @buf: pointer to kmb_frame_buffer
 */
static void kmb_video_remove_buf(struct kmb_video_fh *vid_fh,
				 struct kmb_frame_buffer *buf)
{
	mutex_lock(&vid_fh->lock);
	list_del(&buf->list);
	mutex_unlock(&vid_fh->lock);
}

/**
 * kmb_video_find_buf_by_addr - Find buffer dma_queue by address
 * @vid_fh: pointer to V4L2 sub-device file handle
 * @addr: rt buffer address
 *
 * Return: pointer to kmb_frame_buffer if successful
 */
static struct kmb_frame_buffer *
kmb_video_find_buf_by_addr(struct kmb_video_fh *vid_fh, uint64_t addr)
{
	struct kmb_frame_buffer *buf = NULL;
	struct list_head *node, *next;

	mutex_lock(&vid_fh->lock);

	list_for_each_safe(node, next, &vid_fh->dma_queue) {
		buf = list_entry(node, struct kmb_frame_buffer, list);
		if (buf->addr[0] == addr)
			break;

		buf = NULL;
	}

	mutex_unlock(&vid_fh->lock);
	return buf;
}

/**
 * kmb_video_remote_subdev - Get remote sub-device by given media pad index
 * @video: pointer to kmb video device
 * @pad: pointer to media pad index
 *
 * Return: pointer to V4L2 sub-device
 */
static struct v4l2_subdev *kmb_video_remote_subdev(struct kmb_video *video,
						   u32 *pad)
{
	struct media_pad *remote;

	remote = media_entity_remote_pad(&video->pad);

	if (remote == NULL || !is_media_entity_v4l2_subdev(remote->entity))
		return NULL;

	if (pad)
		*pad = remote->index;

	return media_entity_to_v4l2_subdev(remote->entity);
}

/**
 * kmb_video_fmt_info_to_pix - Convert format info to multiplanar pixel format
 * @vid_fh: pointer to V4L2 sub-device file handle
 * @info: pointer to kmb video format info structure
 * @mbus_fmt: pointer to media bus frame format
 * @pix: pointer to multiplanar format definitions
 *
 * Convert kmb video format info to multiplanar pixel format
 */
static void kmb_video_fmt_info_to_pix(struct kmb_video_fh *vid_fh,
				      const struct kmb_video_fmt_info *info,
				      struct v4l2_mbus_framefmt *mbus_fmt,
				      struct v4l2_pix_format_mplane *pix)
{
	int i;
	u32 bytesperline;
	u32 sizeimage;
	u32 v_sub = 1, h_sub = 1;
	struct kmb_video *kmb_vid = vid_fh->kmb_vid;

	pix->width = mbus_fmt->width;
	pix->height = mbus_fmt->height;

	pix->pixelformat = info->pixelformat;
	pix->colorspace = info->colorspace;
	pix->num_planes = info->planes;

	if (KMB_VIDEO_TYPE_IS_RAW(kmb_vid))
		if (mbus_fmt->reserved[0])
			pix->num_planes = mbus_fmt->reserved[0];

	for (i = 0; i < pix->num_planes; i++) {
		/* subsample is not applied on the luma plane */
		if (info->packed)
			bytesperline = pix->width * info->bpp / 8 / h_sub;
		else
			bytesperline = pix->width *
				       ALIGN(info->bpp, 8) / 8 / h_sub;

		if (pix->plane_fmt[i].bytesperline < bytesperline)
			pix->plane_fmt[i].bytesperline = bytesperline;

		sizeimage =
			pix->plane_fmt[i].bytesperline * pix->height / v_sub;

		if (pix->plane_fmt[i].sizeimage < sizeimage)
			pix->plane_fmt[i].sizeimage = sizeimage;

		h_sub = info->h_subsample;
		v_sub = info->v_subsample;

		pr_debug("Pix %d #pl %d %dx%d pln[%d]: bpl %u sizeimg %u raw %d",
			  pix->pixelformat, pix->num_planes,
			  pix->width, pix->height,
			  i, pix->plane_fmt[i].bytesperline,
			  pix->plane_fmt[i].sizeimage,
			  KMB_VIDEO_TYPE_IS_RAW(kmb_vid));
	}
}

/**
 * kmb_video_get_subdev_pix_fmt - Get sub-device pixel format
 * @vid_fh: pointer to kmb video file handler
 * @pix: pointer to multiplanar format definitions
 *
 * Return: 0 if successful, -EINVAL otherwise
 */
static int kmb_video_get_subdev_pix_fmt(struct kmb_video_fh *vid_fh,
					struct v4l2_pix_format_mplane *pix)
{
	struct kmb_video *kmb_vid = vid_fh->kmb_vid;
	const struct kmb_video_fmt_info *info;
	struct v4l2_subdev_format sd_fmt;
	struct v4l2_subdev *subdev;
	u32 pad;
	int ret;

	subdev = kmb_video_remote_subdev(kmb_vid, &pad);
	if (subdev == NULL)
		return -EINVAL;

	sd_fmt.pad = pad;
	sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(subdev, pad, get_fmt, NULL, &sd_fmt);
	if (ret < 0)
		return ret;

	info = kmb_video_get_fmt_info_by_code(vid_fh, sd_fmt.format.code,
					      kmb_vid->packed_fmt);
	if (!info)
		return -EINVAL;

	kmb_video_fmt_info_to_pix(vid_fh, info, &sd_fmt.format, pix);

	return 0;
}

/**
 * kmb_video_try_subdev_pix_fmt - Negotiate sub-device pixel format
 * @vid_fh: pointer to kmb video file handler
 * @pix: pointer to multiplanar format definitions
 *
 * Return: 0 if successful, -EINVAL otherwise
 */
static int kmb_video_try_subdev_pix_fmt(struct kmb_video_fh *vid_fh,
					struct v4l2_pix_format_mplane *pix)
{
	struct kmb_video *kmb_vid = vid_fh->kmb_vid;
	const struct kmb_video_fmt_info *info;
	struct v4l2_subdev_format sd_fmt;
	struct v4l2_subdev *subdev;
	u32 pad;
	int ret;

	subdev = kmb_video_remote_subdev(kmb_vid, &pad);
	if (subdev == NULL)
		return -EINVAL;

	sd_fmt.pad = pad;
	sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(subdev, pad, get_fmt, NULL, &sd_fmt);
	if (ret < 0)
		return ret;

	info = kmb_video_get_fmt_info_by_pixfmt(vid_fh, pix->pixelformat);
	if (!info)
		return -EINVAL;

	info = kmb_video_get_fmt_info_by_code(vid_fh, sd_fmt.format.code,
					      info->packed);
	if (!info)
		return -EINVAL;

	kmb_video_fmt_info_to_pix(vid_fh, info, &sd_fmt.format, pix);

	return 0;
}

/**
 * kmb_video_get_channel_id - Get Video Node xLink channel id
 * @kmb_vid: pointer to kmb video node device
 *
 * Return: xLink channel id
 */
u32 kmb_video_get_channel_id(struct kmb_video *kmb_vid)
{
	u32 channel_id;

	mutex_lock(&kmb_vid->lock);
	channel_id = kmb_vid->channel_id;
	mutex_unlock(&kmb_vid->lock);

	return channel_id;
}

/**
 * kmb_video_alloc_pipe_cma_area - Allocate CMA area
 * @kmb_vid: pointer to kmb video node device
 *
 * Allocate continuous memory for data structures used by RT
 *
 * Return: 0 if successful
 */
static int kmb_video_alloc_pipe_cma_area(struct kmb_video *kmb_vid)
{
	kmb_vid->cma_vaddr = dma_alloc_coherent(kmb_vid->dma_dev,
			sizeof(struct pipeConfigEvS),
			(dma_addr_t *)&kmb_vid->cma_phy_addr, 0);
	if (!kmb_vid->cma_vaddr)
		return -ENOMEM;

	dev_dbg(&kmb_vid->video->dev, "OUTPUT CHANNEL CMA: phy 0x%x vaddr %p\n",
			kmb_vid->cma_phy_addr, kmb_vid->cma_vaddr);

	return 0;
}

/**
 * kmb_video_free_pipe_cma_area - Free CMA area
 * @kmb_vid: pointer to kmb video node device
 *
 * Free continuous memory for data structures used by RT
 */
static void kmb_video_free_pipe_cma_area(struct kmb_video *kmb_vid)
{
	if (kmb_vid->cma_vaddr)
		dma_free_coherent(kmb_vid->dma_dev,
				  sizeof(struct pipeConfigEvS),
				  kmb_vid->cma_vaddr, kmb_vid->cma_phy_addr);
	kmb_vid->cma_vaddr = NULL;
}

/**
 * kmb_video_close_xlink_channel - Close xlink channel for video
 * @vid_fh: pointer to video sub-device
 *
 * Return: 0 if successful
 */
static int kmb_video_close_xlink_channel(struct kmb_video_fh *vid_fh)
{
	struct kmb_video *kmb_vid = vid_fh->kmb_vid;
	int ret;

	ret = xlink_close_channel(kmb_vid->ipc_dev_handler,
				  kmb_vid->channel_id);
	if (ret) {
		dev_err(kmb_vid->dma_dev,
				"Failed to close VID channel: %d\n", ret);
		return ret;
	}

	dev_dbg(kmb_vid->dma_dev, "Closed VID channel\n");

	return 0;
}

/**
 * kmb_video_open_xlink_channel - Open xlink channel for video
 * @vid_fh: pointer to video sub-device
 *
 * Return: 0 if successful
 */
static int kmb_video_open_xlink_channel(struct kmb_video_fh *vid_fh)
{
	struct kmb_video *kmb_vid = vid_fh->kmb_vid;
	int ret;

	ret = xlink_open_channel(kmb_vid->ipc_dev_handler, kmb_vid->channel_id,
				 RXB_TXB, KMB_RT_VID_CH_DATA_SIZE,
				 KMB_RT_VID_CH_TIMEOUT_MS);
	if (ret) {
		dev_err(kmb_vid->dma_dev,
				"Failed to open VID channel: %d\n", ret);
		return ret;
	}

	dev_dbg(kmb_vid->dma_dev, "Opened VID channel %u\n",
			kmb_vid->channel_id);

	return 0;
}

/**
 * kmb_video_config_stream - Configure LRT streaming
 * @vid_fh: pointer to video sub-device
 * @mbus_fmt: pointer to media bus frame format
 *
 * Return: 0 if successful
 */
static int kmb_video_config_stream(struct kmb_video_fh *vid_fh,
				   struct v4l2_mbus_framefmt *mbus_fmt)
{
	struct kmb_video *kmb_vid = vid_fh->kmb_vid;
	int ret;

	if (vid_fh->cfg_requested &&
	    vid_fh->active_pix.width == mbus_fmt->width &&
	    vid_fh->active_pix.height == mbus_fmt->height)
		return 0;

	if (vid_fh->cfg_requested)
		kmb_vid->pipe_ops.release_config(&kmb_vid->video->entity);

	ret = kmb_vid->pipe_ops.request_config(&kmb_vid->video->entity,
					       mbus_fmt);
	if (ret)
		return ret;

	vid_fh->cfg_requested = true;

	return ret;
}

/**
 * kmb_video_queue_setup - Setup queue
 * @q: pointer to video buffer queue
 * @num_buffers: pointer to number of buffers
 * @num_planes: pointer to number of planes
 * @sizes: array of image sizes
 * @alloc_devs: pointer to array of devices
 *
 * Return: 0 if successful
 */
static int kmb_video_queue_setup(struct vb2_queue *q,
		   unsigned int *num_buffers, unsigned int *num_planes,
		   unsigned int sizes[], struct device *alloc_devs[])
{
	struct kmb_video_fh *vid_fh = vb2_get_drv_priv(q);
	int i;

	if (vid_fh->contiguous_memory) {
		*num_planes = 1;
		for (i = 0; i < vid_fh->active_pix.num_planes; i++)
			sizes[0] += vid_fh->active_pix.plane_fmt[i].sizeimage;
	} else {
		*num_planes = vid_fh->active_pix.num_planes;
		for (i = 0; i < vid_fh->active_pix.num_planes; i++)
			sizes[i] = vid_fh->active_pix.plane_fmt[i].sizeimage;
	}

	return 0;
}

/**
 * kmb_video_buffer_prepare - Buffer prepare
 * @vb: pointer to video buffer
 *
 * Called every time the buffer is queued perform to initialization required
 * before each hardware operation
 *
 * Return: 0 if successful
 */
static int kmb_video_buffer_prepare(struct vb2_buffer *vb)
{
	struct kmb_video_fh *vid_fh = vb2_get_drv_priv(vb->vb2_queue);
	unsigned int size_image = 0;
	int i;

	if (vid_fh->contiguous_memory) {
		for (i = 0; i < vid_fh->active_pix.num_planes; i++)
			size_image += vid_fh->active_pix.plane_fmt[i].sizeimage;

		vb2_set_plane_payload(vb, 0, size_image);
	} else {
		for (i = 0; i < vid_fh->active_pix.num_planes; i++)
			vb2_set_plane_payload(vb, i,
				vid_fh->active_pix.plane_fmt[i].sizeimage);
	}

	return 0;
}

/**
 * kmb_video_buf_init - Buffer initialize
 * @vb: pointer to video buffer
 *
 * Called once after allocating a buffer for additional buffer-related
 * initialization
 *
 * Return: 0 if successful
 */
static int kmb_video_buf_init(struct vb2_buffer *vb)
{
	return 0;
}

/**
 * kmb_video_buf_queue - Pass buffer vb to the driver
 * @vb: pointer to video buffer
 */
static void kmb_video_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct kmb_frame_buffer *buf = (struct kmb_frame_buffer *)vbuf;
	struct kmb_video_fh *vid_fh = vb2_get_drv_priv(vb->vb2_queue);
	struct kmb_video *vid = vid_fh->kmb_vid;
	int i;
	int ret;

	if (vid_fh->contiguous_memory) {
		buf->addr[0] = vb2_dma_contig_plane_dma_addr(vb, 0);
		for (i = 1; i < vid_fh->active_pix.num_planes; i++) {
			buf->addr[i] = buf->addr[i - 1] +
				vid_fh->active_pix.plane_fmt[i - 1].sizeimage;
		}
	} else {
		for (i = 0; i < vid_fh->active_pix.num_planes; i++)
			buf->addr[i] = vb2_dma_contig_plane_dma_addr(vb, i);
	}

	ret = kmb_video_queue_output_buf(vid_fh, buf);
	if (ret)
		dev_err(&vid->video->dev, "Fail output buf queue %d\n", ret);
}

/**
 * kmb_video_worker_thread - Video node worker thread
 * @video: pointer to KMB Video device
 *
 * Return: 0 if successful
 */
static int kmb_video_worker_thread(void *video)
{
	struct kmb_video_fh *vid_fh = video;
	struct kmb_video *vid = vid_fh->kmb_vid;
	struct kmb_frame_buffer *buf = NULL;
	frameBuffer rt_frame_buf;
	size_t rt_frame_buf_size;
	bool stopped = false;
	int ret;

	set_freezable();

	while (!kthread_should_stop()) {
		try_to_freeze();

		if (stopped) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule();
			dev_dbg(&vid->video->dev, "Thread is stopped\n");
			continue;
		}

		memset(&rt_frame_buf, 0x00, sizeof(rt_frame_buf));
		rt_frame_buf_size = sizeof(rt_frame_buf);
		ret = xlink_read_data_to_buffer(vid->ipc_dev_handler,
					     vid->channel_id,
					     (uint8_t *const)&rt_frame_buf,
					     (uint32_t *)&rt_frame_buf_size);
		if (ret != X_LINK_SUCCESS) {
			dev_dbg(&vid->video->dev, "Channel closed %d\n", ret);
			stopped = true;
			continue;
		}

		buf = kmb_video_find_buf_by_addr(vid_fh, rt_frame_buf.p1);
		if (buf) {
			dev_dbg(&vid->video->dev, "Buffer done ok %d %pad\n",
					buf->vb.vb2_buf.index, &buf->addr[0]);
			kmb_video_remove_buf(vid_fh, buf);

			buf->vb.vb2_buf.timestamp = rt_frame_buf.ts;
			vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
		} else {
			dev_err(&vid->video->dev,
					"Ouch cannot find buff %llx\n",
					rt_frame_buf.p1);
		}
	}

	return 0;
}

/**
 * kmb_video_processing_start - Start video processing
 * @vid_fh: pointer to video device file handle
 *
 * Return: 0 if successful
 */
static int kmb_video_processing_start(struct kmb_video_fh *vid_fh)
{
	struct kmb_video *kmb_vid = vid_fh->kmb_vid;
	int ret;

	ret = kmb_video_open_xlink_channel(vid_fh);
	if (ret)
		return ret;

	vid_fh->thread = kthread_run(kmb_video_worker_thread,
				     vid_fh, "kmb_vnode_thread");
	if (IS_ERR(vid_fh->thread)) {
		dev_err(&kmb_vid->video->dev, "%s Cannot start thread\n",
				__func__);
		ret = -ENOMEM;
		vid_fh->thread = NULL;
		goto error_close_xlink_channel;
	}
	dev_dbg(&kmb_vid->video->dev, "%s Thread running\n", __func__);

	return 0;

error_close_xlink_channel:
	kmb_video_close_xlink_channel(vid_fh);
	return ret;
}

/**
 * kmb_video_processing_stop - Stop video processing
 * @vid_fh: pointer to video device file handle
 *
 * Return: 0 if successful
 */
static int kmb_video_processing_stop(struct kmb_video_fh *vid_fh)
{
	struct kmb_video *kmb_vid = vid_fh->kmb_vid;
	int ret;

	kmb_video_close_xlink_channel(vid_fh);

	if (!vid_fh->thread) {
		dev_dbg(&kmb_vid->video->dev, "No thread running\n");
		return 0;
	}

	ret = kthread_stop(vid_fh->thread);
	if (ret < 0) {
		dev_err(&kmb_vid->video->dev, "%s Thread stop failed %d\n",
				__func__, ret);
	}

	vid_fh->thread = NULL;
	dev_dbg(&kmb_vid->video->dev, "%s Thread stopped\n", __func__);

	return ret;
}

/**
 * kmb_video_capture_start_streaming - Set streaming state to enable
 * @q: pointer to videobuf queue
 * @count: (unused)number of buffers owned by the driver
 *
 * Return: 0 if successful, -EPIPE - broken pipe
 */
static int kmb_video_capture_start_streaming(struct vb2_queue *q,
					     unsigned int count)
{
	struct kmb_video_fh *vid_fh = vb2_get_drv_priv(q);
	struct kmb_video *kmb_vid = vid_fh->kmb_vid;
	int ret;

	ret = kmb_vid->pipe_ops.build(&kmb_vid->video->entity);
	if (ret < 0) {
		dev_err(&kmb_vid->video->dev,
				"Cannot initialize rt pipeline\n");
		goto error_discard_all_bufs;
	}

	ret = media_pipeline_start(&kmb_vid->video->entity, kmb_vid->pipe);
	if (ret < 0)
		goto error_teardown_pipeline;

	ret = kmb_video_processing_start(vid_fh);
	if (ret < 0)
		goto error_media_pipeline_stop;

	/* Process all pending buffers on stream enable */
	kmb_video_process_all_bufs(vid_fh);

	ret = kmb_vid->pipe_ops.s_stream(&kmb_vid->video->entity, 1);
	if (ret) {
		dev_err(&kmb_vid->video->dev,
				"Fail to enable streaming %d\n", ret);
		goto error_processing_stop;
	}

	return 0;

error_processing_stop:
	kmb_video_processing_stop(vid_fh);
error_media_pipeline_stop:
	media_pipeline_stop(&kmb_vid->video->entity);
error_teardown_pipeline:
	kmb_vid->pipe_ops.teardown(&kmb_vid->video->entity);
error_discard_all_bufs:
	kmb_video_release_all_bufs(vid_fh, VB2_BUF_STATE_QUEUED);
	return ret;
}

/**
 * kmb_video_capture_stop_streaming - Set streaming state to disable
 * @q: pointer to videobuf queue
 */
static void kmb_video_capture_stop_streaming(struct vb2_queue *q)
{
	struct kmb_video_fh *vid_fh = vb2_get_drv_priv(q);
	struct kmb_video *kmb_vid = vid_fh->kmb_vid;

	kmb_vid->pipe_ops.s_stream(&kmb_vid->video->entity, 0);

	kmb_vid->pipe_ops.teardown(&kmb_vid->video->entity);

	media_pipeline_stop(&kmb_vid->video->entity);

	kmb_video_processing_stop(vid_fh);

	kmb_video_release_all_bufs(vid_fh, VB2_BUF_STATE_ERROR);
}

/**
 * kmb_video_output_start_streaming - Set streaming state to enable
 * @q: pointer to videobuf queue
 * @count: (unused)number of buffers owned by the driver
 *
 * Return: 0 if successful, -EPIPE - broken pipe
 */
static int kmb_video_output_start_streaming(struct vb2_queue *q,
					    unsigned int count)
{
	struct kmb_video_fh *vid_fh = vb2_get_drv_priv(q);
	struct kmb_video *kmb_vid = vid_fh->kmb_vid;
	int ret;

	ret = kmb_vid->pipe_ops.build(&kmb_vid->video->entity);
	if (ret < 0) {
		dev_err(&kmb_vid->video->dev,
				"Cannot initialize rt pipeline\n");
		return ret;
	}

	ret = kmb_video_processing_start(vid_fh);
	if (ret < 0)
		return ret;

	/* Process all pending buffers on stream enable */
	kmb_video_process_all_bufs(vid_fh);

	return 0;
}

/**
 * kmb_video_output_stop_streaming - Set streaming state to disable
 * @q: pointer to videobuf queue
 */
static void kmb_video_output_stop_streaming(struct vb2_queue *q)
{
	struct kmb_video_fh *vid_fh = vb2_get_drv_priv(q);
	struct kmb_video *kmb_vid = vid_fh->kmb_vid;

	kmb_vid->pipe_ops.teardown(&kmb_vid->video->entity);

	kmb_video_processing_stop(vid_fh);

	kmb_video_release_all_bufs(vid_fh, VB2_BUF_STATE_ERROR);
}

/* driver-specific operations */
static struct vb2_ops kmb_video_vb2_q_capture_ops = {
	.queue_setup     = kmb_video_queue_setup,
	.buf_prepare     = kmb_video_buffer_prepare,
	.buf_init        = kmb_video_buf_init,
	.buf_queue       = kmb_video_buf_queue,
	.start_streaming = kmb_video_capture_start_streaming,
	.stop_streaming  = kmb_video_capture_stop_streaming,
};

static struct vb2_ops kmb_video_vb2_q_output_ops = {
	.queue_setup     = kmb_video_queue_setup,
	.buf_prepare     = kmb_video_buffer_prepare,
	.buf_init        = kmb_video_buf_init,
	.buf_queue       = kmb_video_buf_queue,
	.start_streaming = kmb_video_output_start_streaming,
	.stop_streaming  = kmb_video_output_stop_streaming,
};

/**
 * kmb_video_querycap - Query device capabilities
 * @file: pointer to file containing video device data
 * @fh: pointer to V4L2 sub-device file handle
 * @cap: pointer to V4L2 device capabilities structure
 *
 * VIDIOC_QUERYCAP ioctl handler
 *
 * Return: 0 if successful
 */
static int kmb_video_querycap(struct file *file,
	void *fh, struct v4l2_capability *cap)
{
	struct kmb_video_fh *vid_fh = kmb_video_from_v4l2(fh);

	cap->bus_info[0] = 0;
	strlcpy(cap->driver, KMB_CAM_VIDEO_NAME, sizeof(cap->driver));
	strlcpy(cap->card, KMB_CAM_VIDEO_NAME, sizeof(cap->card));
	cap->device_caps = vid_fh->fh.vdev->device_caps;
	cap->capabilities = vid_fh->fh.vdev->device_caps |
			    V4L2_CAP_DEVICE_CAPS;

	return 0;
}

/**
 * kmb_video_enum_fmt - Enumerate format
 * @file: pointer to file containing video device data
 * @fh: pointer to V4L2 sub-device file handle
 * @f: pointer to V4L2 format structure
 *
 * VIDIOC_ENUM_FMT ioctl handler. Enumerate image formats for video capture
 * in multiple plain mode.
 * If a raw input/output node is enumerated, enumerate each media bus format
 * twice for a packed and unpacked pixel format
 *
 * Return: 0 if successful, -EINVAL if there is not such index
 */
static int kmb_video_enum_fmt(struct file *file,
	void *fh, struct v4l2_fmtdesc *f)
{
	struct kmb_video_fh *vid_fh = kmb_video_from_v4l2(fh);
	struct kmb_video *kmb_vid = vid_fh->kmb_vid;
	struct v4l2_subdev_mbus_code_enum mbus_code;
	const struct kmb_video_fmt_info *info;
	struct v4l2_subdev *subdev;
	int packed = 0;
	u32 pad;
	int ret;

	subdev = kmb_video_remote_subdev(kmb_vid, &pad);
	if (subdev == NULL)
		return -EINVAL;

	memset(&mbus_code, 0, sizeof(mbus_code));
	mbus_code.pad = pad;
	mbus_code.which =  V4L2_SUBDEV_FORMAT_ACTIVE;
	mbus_code.index = f->index;

	if (KMB_VIDEO_TYPE_IS_RAW(kmb_vid))
		mbus_code.index = f->index / 2;

	ret = v4l2_subdev_call(subdev, pad, enum_mbus_code, NULL, &mbus_code);
	if (ret)
		return -EINVAL;

	if (KMB_VIDEO_TYPE_IS_RAW(kmb_vid))
		packed = f->index & 0x01;

	info = kmb_video_get_fmt_info_by_code(vid_fh, mbus_code.code, packed);
	if (!info)
		return -EINVAL;

	strlcpy(f->description, info->description, sizeof(f->description));
	f->pixelformat = info->pixelformat;

	return 0;
}

/**
 * kmb_video_enum_framesizes - Enumerate frame sizes
 * @file: pointer to file containing video device data
 * @fh: pointer to V4L2 sub-device file handle
 * @fsize: pointer to V4L2 frame size enumeration
 *
 * VIDIOC_ENUM_FRAMESIZES ioctl handler
 *
 * Return: 0 if successful, -EINVAL if there is not such index
 */
static int kmb_video_enum_framesizes(struct file *file, void *fh,
				     struct v4l2_frmsizeenum *fsize)
{
	struct kmb_video_fh *vid_fh = kmb_video_from_v4l2(fh);
	struct v4l2_pix_format_mplane pix_mp;
	int ret;

	if (fsize->index > 0)
		return -EINVAL;

	memset(&pix_mp, 0, sizeof(pix_mp));
	ret = kmb_video_get_subdev_pix_fmt(vid_fh, &pix_mp);
	if (ret < 0)
		return ret;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = pix_mp.width;
	fsize->discrete.height = pix_mp.height;

	return 0;
}

/**
 * kmb_video_enum_frameintervals - Enumerate frame intervals
 * @file: pointer to file containing video device data
 * @fh: pointer to V4L2 sub-device file handle
 * @fival: pointer to V4L2 frame interval structure
 *
 * VIDIOC_ENUM_FRAMEINTERVALS ioctl handler
 *
 * Return: 0 if successful, -EINVAL if there is not such index
 */
static int kmb_video_enum_frameintervals(struct file *file, void *fh,
					 struct v4l2_frmivalenum *fival)
{
	struct kmb_video_fh *vid_fh = kmb_video_from_v4l2(fh);
	struct kmb_video *kmb_vid = vid_fh->kmb_vid;
	struct v4l2_subdev_frame_interval_enum interval;
	struct v4l2_subdev_pad_config cfg;
	struct v4l2_subdev *subdev;
	u32 pad;
	int ret;

	subdev = kmb_video_remote_subdev(kmb_vid, &pad);
	if (subdev == NULL)
		return -EINVAL;

	memset(&interval, 0, sizeof(interval));
	interval.pad = pad;
	interval.index = fival->index;
	interval.which = V4L2_SUBDEV_FORMAT_TRY;
	interval.width = fival->width;
	interval.height = fival->height;

	ret = v4l2_subdev_call(subdev, pad, enum_frame_interval,
				&cfg, &interval);
	if (ret < 0)
		return -EINVAL;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete = interval.interval;

	return 0;
}

/**
 * kmb_video_get_fmt - Get format for video capture in multiple plain mode
 * @file: pointer to file containing video device data
 * @fh: pointer to V4L2 sub-device file handle
 * @f: pointer to stream data format
 *
 * VIDIOC_G_FMT ioctl handler
 *
 * Return: 0 if successful
 */
static int kmb_video_get_fmt(struct file *file,
	void *fh, struct v4l2_format *f)
{
	struct kmb_video_fh *vid_fh = kmb_video_from_v4l2(fh);

	f->fmt.pix_mp = vid_fh->active_pix;

	return 0;
}

/**
 * kmb_video_try_fmt_vid - Negotiate format for video capture
 * @file: pointer to file containing video device data
 * @fh: pointer to V4L2 sub-device file handle
 * @f: pointer to stream data format
 *
 * VIDIOC_TRY_FMT ioctl handler. Negotiate format for video capture in
 * multiple plain mode
 *
 * Return: 0 if successful
 */
static int kmb_video_try_fmt_vid(struct file *file,
	void *fh, struct v4l2_format *f)
{
	struct kmb_video_fh *vid_fh = kmb_video_from_v4l2(fh);

	return kmb_video_try_subdev_pix_fmt(vid_fh, &f->fmt.pix_mp);
}

/**
 * kmb_video_set_fmt_vid - Set format for video capture in multiple plain mode
 * @file: pointer to file containing video device data
 * @fh: pointer to V4L2 sub-device file handle
 * @f: pointer to stream data format
 *
 * VIDIOC_S_FMT ioctl handler
 *
 * Return: 0 if successful
 */
static int kmb_video_set_fmt_vid(struct file *file,
	void *fh, struct v4l2_format *f)
{
	struct kmb_video_fh *vid_fh = kmb_video_from_v4l2(fh);
	struct kmb_video *kmb_vid = vid_fh->kmb_vid;
	const struct kmb_video_fmt_info *info;
	struct v4l2_mbus_framefmt mbus_fmt;
	int ret;

	memset(&mbus_fmt, 0, sizeof(mbus_fmt));
	info = kmb_video_get_fmt_info_by_pixfmt(vid_fh,
						f->fmt.pix_mp.pixelformat);
	if (!info) {
		v4l2_err(kmb_vid->video->v4l2_dev, "Can not get format %x\n",
				f->fmt.pix_mp.pixelformat);
		return -EINVAL;
	}

	mbus_fmt.width = f->fmt.pix_mp.width;
	mbus_fmt.height = f->fmt.pix_mp.height;
	mbus_fmt.code = info->code;

	ret = kmb_video_config_stream(vid_fh, &mbus_fmt);
	if (ret)
		return ret;

	info = kmb_video_get_fmt_info_by_code(vid_fh, mbus_fmt.code,
					      info->packed);
	if (!info) {
		v4l2_err(kmb_vid->video->v4l2_dev, "Can not get format %x\n",
				mbus_fmt.code);
		return -EINVAL;
	}

	f->fmt.pix_mp.width = mbus_fmt.width;
	f->fmt.pix_mp.height = mbus_fmt.height;
	f->fmt.pix_mp.pixelformat = info->pixelformat;

	vid_fh->active_pix = f->fmt.pix_mp;

	kmb_video_fmt_info_to_pix(vid_fh, info, &mbus_fmt, &vid_fh->active_pix);

	f->fmt.pix_mp = vid_fh->active_pix;
	vid_fh->contiguous_memory = info->contiguous_memory;
	kmb_vid->packed_fmt = info->packed;

	return 0;
}

/**
 * kmb_video_reqbufs - Initiate memory mapped or user pointer I/O
 * @file: pointer to file containing video device data
 * @fh: pointer to V4L2 sub-device file handle
 * @req: pointer to request buffers structure
 *
 * VIDIOC_REQBUFS ioctl handler
 *
 * Return: 0 if successful
 */
static int kmb_video_reqbufs(struct file *file,
	void *fh, struct v4l2_requestbuffers *req)
{
	struct kmb_video_fh *vid_fh = kmb_video_from_v4l2(fh);

	return vb2_reqbufs(&vid_fh->vb2_q, req);
}

/**
 * kmb_video_expbuf - Export dma fd of memory mapped buffer
 * @file: pointer to file containing video device data
 * @fh: pointer to V4L2 sub-device file handle
 * @expbuf: pointer to export buffer structure
 *
 * VIDIOC_EXPBUF ioctl handler
 *
 * Return: 0 if successful
 */
static int kmb_video_expbuf(struct file *file,
	void *fh, struct v4l2_exportbuffer *expbuf)
{
	struct kmb_video_fh *vid_fh = kmb_video_from_v4l2(fh);

	return vb2_expbuf(&vid_fh->vb2_q, expbuf);
}

/**
 * kmb_video_qbuf - Queue buffer
 * @file: pointer to file containing video device data
 * @fh: pointer to V4L2 sub-device file handle
 * @pb: pointer to video buffer information
 *
 * VIDIOC_QBUF ioctl handler. Enqueue an empty (capturing) or filled (output)
 * buffer in the driver's incoming queue.
 *
 * Return: 0 if successful
 */
static int kmb_video_qbuf(struct file *file, void *fh,
	struct v4l2_buffer *pb)
{
	struct kmb_video_fh *vid_fh = kmb_video_from_v4l2(fh);
	struct media_device *mdev = vid_fh->fh.vdev->v4l2_dev->mdev;

	return vb2_qbuf(&vid_fh->vb2_q, mdev, pb);
}

/**
 * kmb_video_dqbuf - Dequeue a filled buffer from the driver's outgoing queue
 * @file: pointer to file containing video device data
 * @fh: pointer to V4L2 sub-device file handle
 * @pb: pointer to video buffer information
 *
 * VIDIOC_DQBUF ioctl handler
 *
 * Return: 0 if successful
 */
static int kmb_video_dqbuf(struct file *file,
	void *fh, struct v4l2_buffer *pb)
{
	struct kmb_video_fh *vid_fh = kmb_video_from_v4l2(fh);

	return vb2_dqbuf(&vid_fh->vb2_q, pb, file->f_flags & O_NONBLOCK);
}

/**
 * kmb_video_streamon - Start streaming
 * @file: pointer to file containing video device data
 * @fh: pointer to V4L2 sub-device file handle
 * @buf_type: V4L2 buffer type
 *
 * VIDIOC_STREAMON ioctl handler
 *
 * Return: 0 if successful
 */
static int kmb_video_streamon(struct file *file,
	void *fh, enum v4l2_buf_type buf_type)
{
	struct kmb_video_fh *vid_fh = kmb_video_from_v4l2(fh);
	struct kmb_video *kmb_vid = vid_fh->kmb_vid;
	int ret;

	ret = vb2_streamon(&vid_fh->vb2_q, buf_type);
	if (ret < 0)
		dev_err(&kmb_vid->video->dev,
			"Stream ON failed! ret = %d\n", ret);

	return ret;
}

/**
 * kmb_video_streamoff - Stop streaming
 * @file: pointer to file containing video device data
 * @fh: pointer to V4L2 sub-device file handle
 * @buf_type: V4L2 buffer type
 *
 * VIDIOC_STREAMOFF ioctl handler
 *
 * Return: 0 if successful
 */
static int kmb_video_streamoff(struct file *file,
	void *fh, enum v4l2_buf_type buf_type)
{
	struct kmb_video_fh *vid_fh = kmb_video_from_v4l2(fh);

	return vb2_streamoff(&vid_fh->vb2_q, buf_type);
}

/**
 * kmb_video_query_buf - Query the status of a buffer
 * @file: pointer to file containing video device data
 * @fh: pointer to V4L2 sub-device file handle
 * @pb: pointer to video buffer information
 *
 * VIDIOC_QUERYBUF ioctl handler
 *
 * Return: 0 if successful
 */
static int kmb_video_query_buf(struct file *file, void *fh,
	struct v4l2_buffer *pb)
{
	struct kmb_video_fh *vid_fh = kmb_video_from_v4l2(fh);

	return vb2_querybuf(&vid_fh->vb2_q, pb);
}

/**
 * kmb_video_get_pixelaspect - Dummy implementation
 * @file: pointer to file containing video device data
 * @fh: pointer to V4L2 sub-device file handle
 * @buf_type: V4L2 buffer type
 * @aspect: pointer to crop pixel aspect struct
 *
 * VIDIOC_CROPCAP ioctl handler for the pixel aspect
 *
 * Return: 0 if successful
 */
static int kmb_video_get_pixelaspect(struct file *file, void *fh,
	int buf_type, struct v4l2_fract *aspect)
{
	aspect->numerator = 1;
	aspect->denominator = 1;

	return 0;
}

/**
 * kmb_video_get_parm - Get streaming parameters
 * @file: pointer to file containing video device data
 * @fh: pointer to V4L2 sub-device file handle
 * @params: pointer to V4L2 streaming parameters
 *
 * VIDIOC_G_PARM ioctl handler for streaming parameters
 *
 * Return: 0 if successful
 */
static int kmb_video_get_parm(struct file *file, void *fh,
			       struct v4l2_streamparm *params)
{
	struct kmb_video_fh *vid_fh = kmb_video_from_v4l2(fh);
	struct kmb_video *kmb_vid = vid_fh->kmb_vid;
	struct v4l2_subdev_frame_interval interval;
	struct v4l2_subdev *subdev;
	u32 pad;
	int ret;

	if (V4L2_TYPE_IS_OUTPUT(params->type))
		return -EINVAL;

	subdev = kmb_video_remote_subdev(kmb_vid, &pad);
	if (subdev == NULL)
		return -EINVAL;

	memset(&interval, 0, sizeof(interval));
	interval.pad = pad;
	interval.interval = params->parm.capture.timeperframe;
	ret = v4l2_subdev_call(subdev, video, g_frame_interval, &interval);
	if (ret)
		return -EINVAL;

	params->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	params->parm.capture.timeperframe = interval.interval;

	dev_dbg(&kmb_vid->video->dev, "Get frame interval %d/%d",
		params->parm.capture.timeperframe.numerator,
		params->parm.capture.timeperframe.denominator);

	return 0;
}

/**
 * kmb_video_set_parm - Set streaming parameters
 * @file: pointer to file containing video device data
 * @fh: pointer to V4L2 sub-device file handle
 * @params: pointer to V4L2 streaming parameters
 *
 * VIDIOC_S_PARM ioctl handler for streaming parameters
 *
 * Return: 0 if successful
 */
static int kmb_video_set_parm(struct file *file, void *fh,
			       struct v4l2_streamparm *params)
{
	struct kmb_video_fh *vid_fh = kmb_video_from_v4l2(fh);
	struct kmb_video *kmb_vid = vid_fh->kmb_vid;
	struct v4l2_subdev_frame_interval interval;
	struct v4l2_subdev *subdev;
	u32 pad;
	int ret;

	if (V4L2_TYPE_IS_OUTPUT(params->type))
		return -EINVAL;

	subdev = kmb_video_remote_subdev(kmb_vid, &pad);
	if (subdev == NULL)
		return -EINVAL;

	memset(&interval, 0, sizeof(interval));
	interval.pad = pad;
	interval.interval = params->parm.capture.timeperframe;
	ret = v4l2_subdev_call(subdev, video, s_frame_interval, &interval);
	if (ret)
		return -EINVAL;

	params->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	params->parm.capture.timeperframe = interval.interval;

	dev_dbg(&kmb_vid->video->dev, "Set frame interval %d/%d",
		params->parm.capture.timeperframe.numerator,
		params->parm.capture.timeperframe.denominator);

	return 0;
}

/* V4L2 ioctl operations */
static const struct v4l2_ioctl_ops kmb_vid_ioctl_ops = {
	.vidioc_querycap                 = kmb_video_querycap,
	.vidioc_enum_fmt_vid_cap         = kmb_video_enum_fmt,
	.vidioc_enum_fmt_vid_out         = kmb_video_enum_fmt,
	.vidioc_enum_framesizes          = kmb_video_enum_framesizes,
	.vidioc_enum_frameintervals      = kmb_video_enum_frameintervals,
	.vidioc_g_fmt_vid_cap_mplane     = kmb_video_get_fmt,
	.vidioc_try_fmt_vid_cap_mplane   = kmb_video_try_fmt_vid,
	.vidioc_s_fmt_vid_cap_mplane     = kmb_video_set_fmt_vid,
	.vidioc_g_fmt_vid_out_mplane	 = kmb_video_get_fmt,
	.vidioc_try_fmt_vid_out_mplane	 = kmb_video_try_fmt_vid,
	.vidioc_s_fmt_vid_out_mplane	 = kmb_video_set_fmt_vid,
	.vidioc_g_parm                   = kmb_video_get_parm,
	.vidioc_s_parm                   = kmb_video_set_parm,
	.vidioc_querybuf                 = kmb_video_query_buf,
	.vidioc_g_pixelaspect            = kmb_video_get_pixelaspect,
	.vidioc_reqbufs                  = kmb_video_reqbufs,
	.vidioc_qbuf                     = kmb_video_qbuf,
	.vidioc_dqbuf                    = kmb_video_dqbuf,
	.vidioc_streamon                 = kmb_video_streamon,
	.vidioc_streamoff                = kmb_video_streamoff,
	.vidioc_expbuf                   = kmb_video_expbuf
};

/**
 * kmb_video_open - Device node open
 * @file: pointer to file containing video device data
 *
 * Return: 0 if successful, -ENOMEM if there is not enough memory
 */
static int kmb_video_open(struct file *file)
{
	struct kmb_video *kmb_vid = video_drvdata(file);
	struct video_device *video = video_devdata(file);
	struct v4l2_mbus_framefmt fmt;
	struct kmb_video_fh *vid_fh;
	int ret;

	vid_fh = kzalloc(sizeof(*vid_fh), GFP_KERNEL);
	if (!vid_fh)
		return -ENOMEM;

	v4l2_fh_init(&vid_fh->fh, video);
	vid_fh->kmb_vid = kmb_vid;
	vid_fh->cfg_requested = false;
	mutex_init(&vid_fh->lock);
	mutex_init(&vid_fh->vb2_lock);
	INIT_LIST_HEAD(&vid_fh->dma_queue);

	file->private_data = &vid_fh->fh;
	v4l2_fh_add(&vid_fh->fh);

	if (kmb_vid->video->vfl_dir & VFL_DIR_TX) {
		vid_fh->vb2_q.ops = &kmb_video_vb2_q_output_ops;
		vid_fh->vb2_q.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	} else {
		vid_fh->vb2_q.ops = &kmb_video_vb2_q_capture_ops;
		vid_fh->vb2_q.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	}

	vid_fh->vb2_q.drv_priv = vid_fh;
	vid_fh->vb2_q.buf_struct_size = sizeof(struct kmb_frame_buffer);
	vid_fh->vb2_q.io_modes = VB2_MMAP | VB2_DMABUF;
	vid_fh->vb2_q.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	vid_fh->vb2_q.mem_ops = &vb2_dma_contig_memops;
	vid_fh->vb2_q.dev = kmb_vid->dma_dev;
	vid_fh->vb2_q.lock = &vid_fh->vb2_lock;

	ret = vb2_queue_init(&vid_fh->vb2_q);
	if (ret < 0) {
		dev_err(&video->dev, "Error vb2 queue init\n");
		goto error_release_v4l2_fh;
	}

	/* Fill default format. */
	memset(&fmt, 0, sizeof(fmt));
	if (KMB_VIDEO_TYPE_IS_RAW(kmb_vid)) {
		kmb_video_fmt_info_to_pix(vid_fh, &raw_formats[0], &fmt,
					  &vid_fh->active_pix);
	} else {
		struct media_pad *isp_pad;

		kmb_video_fmt_info_to_pix(vid_fh, &video_formats[0], &fmt,
					  &vid_fh->active_pix);

		isp_pad = media_entity_remote_pad(&kmb_vid->pad);
		if (!isp_pad) {
			dev_err(&video->dev, "Cannot get remote pad\n");
			ret = -ENODEV;
			goto error_release_v4l2_fh;
		}
	}

	if (kmb_vid->channel_ops.accuire_id)
		kmb_vid->channel_id =
			kmb_vid->channel_ops.accuire_id(kmb_vid->priv);

	ret = kmb_video_alloc_pipe_cma_area(kmb_vid);
	if (ret < 0)
		goto error_release_v4l2_fh;

	ret = kmb_vid->pipe_ops.s_power(&kmb_vid->video->entity, 1);
	if (ret)
		goto error_cma_free;

	/* trigger power on on pipeline subdevs */
	ret = v4l2_pipeline_pm_use(&kmb_vid->video->entity, 1);
	if (ret) {
		dev_err(&kmb_vid->video->dev,
			"Failed to set power on! %d", ret);
		goto error_power_off_pipe;
	}

	return 0;

error_power_off_pipe:
	kmb_vid->pipe_ops.s_power(&kmb_vid->video->entity, 0);
error_cma_free:
	kmb_video_free_pipe_cma_area(kmb_vid);
	if (kmb_vid->channel_ops.free_id && kmb_vid->channel_id)
		kmb_vid->channel_ops.free_id(kmb_vid->priv,
					     kmb_vid->channel_id);
	kmb_vid->channel_id = 0;
error_release_v4l2_fh:
	v4l2_fh_del(&vid_fh->fh);
	v4l2_fh_exit(&vid_fh->fh);
	kfree(vid_fh);
	return ret;
}

/**
 * kmb_video_release - Video device release
 * @file: pointer to file containing video device data
 *
 * Return: 0 if successful
 */
static int kmb_video_release(struct file *file)
{
	struct kmb_video_fh *vid_fh = kmb_video_from_v4l2(file->private_data);
	struct kmb_video *kmb_vid = vid_fh->kmb_vid;
	struct v4l2_mbus_framefmt fmt;

	if (vid_fh->cfg_requested) {
		kmb_vid->pipe_ops.release_config(&kmb_vid->video->entity);
		vid_fh->cfg_requested = false;
	}

	kmb_video_free_pipe_cma_area(kmb_vid);
	vb2_queue_release(&vid_fh->vb2_q);
	v4l2_pipeline_pm_use(&kmb_vid->video->entity, 0);
	kmb_vid->pipe_ops.s_power(&kmb_vid->video->entity, 0);
	if (kmb_vid->channel_ops.free_id && kmb_vid->channel_id)
		kmb_vid->channel_ops.free_id(kmb_vid->priv,
					     kmb_vid->channel_id);
	kmb_vid->channel_id = 0;

	/* reset remote pad format */
	memset(&fmt, 0, sizeof(fmt));
	fmt.code = raw_formats[0].code;
	kmb_video_fmt_info_to_pix(vid_fh, &raw_formats[0], &fmt,
				  &vid_fh->active_pix);
	vid_fh->contiguous_memory = raw_formats[0].contiguous_memory;

	v4l2_fh_del(&vid_fh->fh);
	v4l2_fh_exit(&vid_fh->fh);
	mutex_destroy(&vid_fh->vb2_lock);
	mutex_destroy(&vid_fh->lock);

	kfree(vid_fh);
	return 0;
}

/**
 * kmb_video_poll - Poll
 * @file: pointer to file containing video device data
 * @wait: poll table wait
 *
 * Return: 0 if successful
 */
static unsigned int kmb_video_poll(struct file *file,
	struct poll_table_struct *wait)
{
	struct kmb_video_fh *vid_fh = kmb_video_from_v4l2(file->private_data);

	return vb2_poll(&vid_fh->vb2_q, file, wait);
}

/**
 * kmb_video_mmap - Memory map
 * @file: pointer to file containing video device data
 * @vma: VMM memory area
 *
 * Return: 0 if successful
 */
static int kmb_video_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct kmb_video_fh *vid_fh = kmb_video_from_v4l2(file->private_data);

	return vb2_mmap(&vid_fh->vb2_q, vma);
}

/* FS operations for V4L2 device */
static const struct v4l2_file_operations kmb_vid_fops = {
	.owner          = THIS_MODULE,
	.open           = kmb_video_open,
	.release        = kmb_video_release,
	.poll           = kmb_video_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap           = kmb_video_mmap
};

/**
 * kmb_video_init - Initialize entity
 * @kmb_vid: pointer to kmb video device
 * @name: entity name
 * @type: kmb video node type
 *
 * Return: 0 if successful
 */
int kmb_video_init(struct kmb_video *kmb_vid, const char *name,
		   enum kmb_video_type type)
{
	int ret;

	kmb_vid->video = video_device_alloc();
	if (!kmb_vid->video) {
		dev_err(&kmb_vid->video->dev,
				"Failed to allocate video device\n");
		return -ENOMEM;
	}

	mutex_init(&kmb_vid->lock);
	mutex_init(&kmb_vid->video_lock);
	kmb_vid->channel_id = 0;
	kmb_vid->cma_vaddr = NULL;
	kmb_vid->cma_phy_addr = 0;
	kmb_vid->type = type;

	switch (type) {
	case KMB_VIDEO_RAW_INPUT:
		kmb_vid->pad.flags = MEDIA_PAD_FL_SOURCE;
		kmb_vid->video->vfl_dir = VFL_DIR_TX;
		kmb_vid->video->device_caps = V4L2_CAP_VIDEO_OUTPUT_MPLANE |
					      V4L2_CAP_STREAMING;
		break;
	case KMB_VIDEO_RAW_OUTPUT:
	case KMB_VIDEO_YUV_OUTPUT:
		kmb_vid->pad.flags = MEDIA_PAD_FL_SINK;
		kmb_vid->video->device_caps = V4L2_CAP_VIDEO_CAPTURE_MPLANE |
					      V4L2_CAP_STREAMING;
		break;
	default:
		pr_err("No such video node type %d", type);
		return -EINVAL;
	}

	ret = media_entity_pads_init(&kmb_vid->video->entity, 1, &kmb_vid->pad);
	if (ret < 0)
		return ret;

	kmb_vid->video->fops  = &kmb_vid_fops;
	kmb_vid->video->ioctl_ops = &kmb_vid_ioctl_ops;
	kmb_vid->video->minor = -1;
	kmb_vid->video->release  = video_device_release;
	kmb_vid->video->vfl_type = VFL_TYPE_GRABBER;
	kmb_vid->video->lock = &kmb_vid->video_lock;
	snprintf(kmb_vid->video->name, sizeof(kmb_vid->video->name),
		"kmb_video %s", name);

	video_set_drvdata(kmb_vid->video, kmb_vid);
	return 0;
}

/**
 * kmb_video_deinit - Free resources associated with entity
 * @kmb_vid: pointer to kmb video device
 */
void kmb_video_deinit(struct kmb_video *kmb_vid)
{
	media_entity_cleanup(&kmb_vid->video->entity);
	mutex_destroy(&kmb_vid->lock);
	mutex_destroy(&kmb_vid->video_lock);
}

/**
 * kmb_video_register - Register V4L2 device
 * @kmb_vid: pointer to kmb video device
 * @v4l2_dev: pointer to V4L2 device drivers
 *
 * Return: 0 if successful
 */
int kmb_video_register(struct kmb_video *kmb_vid,
		       struct v4l2_device *v4l2_dev)
{
	int ret;

	kmb_vid->video->v4l2_dev = v4l2_dev;
	ret = video_register_device(kmb_vid->video, VFL_TYPE_GRABBER, -1);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "Failed to register video device\n");
		goto error_vdev_release;
	}

	return 0;

error_vdev_release:
	kmb_video_deinit(kmb_vid);
	video_device_release(kmb_vid->video);
	return ret;
}

/**
 * kmb_video_unregister - Unregister V4L device
 * @kmb_vid: pointer to kmb video device
 */
void kmb_video_unregister(struct kmb_video *kmb_vid)
{
	kmb_video_deinit(kmb_vid);
	video_unregister_device(kmb_vid->video);
}
