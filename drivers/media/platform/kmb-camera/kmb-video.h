/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Keem Bay Camera Video node Driver.
 *
 * Copyright (C) 2018-2020 Intel Corporation
 */
#ifndef KMB_VIDEO_H
#define KMB_VIDEO_H

#include <linux/xlink.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-v4l2.h>

#define KMB_CAM_VIDEO_NAME "kmb-video"

/**
 * struct kmb_dma_config - KMB dma configuration structure
 * @line_length: line length
 * @num_lines: number of lines
 * @size: dma buffer size
 */
struct kmb_dma_config {
	unsigned int line_length;
	unsigned int num_lines;
	unsigned int size;
};

/**
 * struct kmb_frame_buffer - KMB frame buffer structure
 * @vb: video buffer for v4l2
 * @addr: array of dma buffer plane address
 * @list: frame buffer list
 */
struct kmb_frame_buffer {
	struct vb2_v4l2_buffer vb;
	dma_addr_t addr[3];
	struct list_head list;
};

/**
 * struct kmb_pipeline_ops - KMB pipeline operations
 * @s_power: pipeline power operation ON/OFF
 * @build: build LRT pipeline
 * @teardown: teardown LRT pipeline
 * @request_config: Request LRT pipeline configuration
 * @release_config: Release LRT pipeline configuration
 * @s_stream: set LRT pipeline stream state
 */
struct kmb_pipeline_ops {
	int (*s_power)(struct media_entity *entity, int on);
	int (*build)(struct media_entity *entity);
	void (*teardown)(struct media_entity *entity);
	int (*request_config)(struct media_entity *entity,
			      struct v4l2_mbus_framefmt *fmt);
	int (*release_config)(struct media_entity *entity);
	int (*s_stream)(struct media_entity *entity, int enable);
};

/**
 * struct kmb_video_channel_ops - xLink channel ID operations
 * @accuire_id: accuire xlink channel ID
 * @free_id: free xlink channel ID
 */
struct kmb_video_channel_ops {
	int (*accuire_id)(void *priv);
	void (*free_id)(void *priv, unsigned int id);
};

/**
 * enum kmb_video_type - KMB Video node type
 * @KMB_VIDEO_RAW_INPUT - RAW input video node
 * @KMB_VIDEO_RAW_OUTPUT - RAW output video node
 * @KMB_VIDEO_YUV_OUTPUT - YUV output video node
 */
enum kmb_video_type {
	KMB_VIDEO_RAW_INPUT,
	KMB_VIDEO_RAW_OUTPUT,
	KMB_VIDEO_YUV_OUTPUT
};

/**
 * struct kmb_video - KMB Video device structure
 * @lock: mutex
 * @video_lock: mutex serializing video operations
 * @video: pointer to V4L2 sub-device
 * @dma_dev: pointer to dma device
 * @pad: media pad graph objects
 * @pipe: pointer to media pipeline
 * @pipe_ops: pointer to KMB pipeline operations
 * @ipc_dev_handler: xLink IPC device handler
 * @channel_ops: xLink channel id operations
 * @channel_id: xLink channel id
 * @type: video node type
 * @packed_fmt: packed flag used only by raw input/output video nodes
 * @cma_phy_addr: video channel physical CMA address
 * @cma_vaddr: video channel virtual CMA address
 * @priv: pointer to private data
 */
struct kmb_video {
	struct mutex lock;
	struct mutex video_lock;
	struct video_device *video;
	struct device *dma_dev;
	struct media_pad pad;
	struct media_pipeline *pipe;
	struct kmb_pipeline_ops pipe_ops;
	struct xlink_handle *ipc_dev_handler;
	struct kmb_video_channel_ops channel_ops;
	u32 channel_id;
	enum kmb_video_type type;
	bool packed_fmt;
	u32 cma_phy_addr;
	void *cma_vaddr;
	void *priv;
};

/**
 * struct kmb_video_fh - KMB video file handler
 * @lock: mutex serializing access to fh
 * @vb2_lock: mutex serializing access to vb2 queue
 * @fh: V4L2 file handler
 * @vb2_q: video buffer queue
 * @active_pix: multiplanar format definitions
 * @contiguous_memory: Flag indicating that plane memory is contiguous
 * @dma_queue: DMA buffers queue
 * @thread: pointer to worker thread data
 * @cfg_requested: flag to indicate whether the configuration was requested
 * @kmb_vid: pointer to KMB video device
 */
struct kmb_video_fh {
	struct mutex lock;
	struct mutex vb2_lock;
	struct v4l2_fh fh;
	struct vb2_queue vb2_q;
	struct v4l2_pix_format_mplane active_pix;
	bool contiguous_memory;
	struct list_head dma_queue;
	struct task_struct *thread;
	bool cfg_requested;
	struct kmb_video *kmb_vid;
};

int kmb_video_init(struct kmb_video *kmb_vid, const char *name,
		   enum kmb_video_type type);
void kmb_video_deinit(struct kmb_video *kmb_vid);

int kmb_video_register(struct kmb_video *kmb_vid,
		       struct v4l2_device *v4l2_dev);
void kmb_video_unregister(struct kmb_video *kmb_vid);

u32 kmb_video_get_channel_id(struct kmb_video *kmb_vid);

#endif /* KMB_VIDEO_H */
