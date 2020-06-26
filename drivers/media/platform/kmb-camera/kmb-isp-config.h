/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Keem Bay Camera ISP Configuration Video Node Driver.
 *
 * Copyright (C) 2019-2020 Intel Corporation
 */
#ifndef KMB_ISP_CONFIG_VNODE_H
#define KMB_ISP_CONFIG_VNODE_H

#include <media/v4l2-dev.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-v4l2.h>

/**
 * struct kmb_isp_cfg_data - KMB ISP configuration data
 * @vb: video buffer
 * @addr: dma buffer address
 * @isp_cfgs: isp config buffers queue
 */
struct kmb_isp_cfg_data {
	struct vb2_v4l2_buffer vb;
	dma_addr_t addr;
	struct list_head isp_cfgs;
};

/**
 * struct kmb_isp_cfg_queue_ops - KMB ISP Config queue operations
 * @queue: queue an ISP config buffer
 * @flish: discard all ISP config buffers
 */
struct kmb_isp_cfg_queue_ops {
	int (*queue)(void *priv, struct kmb_isp_cfg_data *buf);
	void (*flush)(void *priv);
};

/**
 * struct kmb_isp_config - KMB camera isp config device structure
 * @lock: mutex
 * @video_out: pointer to V4L2 video device node
 * @dma_dev: pointer to dma device
 * @pad: media pad graph objects
 * @pipe: pointer to media pipeline
 * @queue_ops: ISP config buffer queue operations
 * @priv: pointer to private data
 */
struct kmb_isp_config {
	struct mutex lock;
	struct video_device *video_out;
	struct device *dma_dev;
	struct media_pad pad;
	struct media_pipeline *pipe;
	struct kmb_isp_cfg_queue_ops queue_ops;
	void *priv;
};

/**
 * struct kmb_isp_cfg_fh - KMB isp config file handler
 * @lock: mutex serializing fh operations
 * @fh: V4L2 file handler
 * @vb2_q: video buffer queue
 * @data_size: isp configuration data size
 * @kmb_cfg: pointer to KMB isp config device
 */
struct kmb_isp_cfg_fh {
	struct mutex lock;
	struct v4l2_fh fh;
	struct vb2_queue vb2_q;
	int data_size;
	struct kmb_isp_config *kmb_cfg;
};

int kmb_isp_config_init(struct kmb_isp_config *kmb_cfg);
void kmb_isp_config_deinit(struct kmb_isp_config *kmb_cfg);

int kmb_isp_config_register(struct kmb_isp_config *kmb_cfg,
			    struct v4l2_device *v4l2_dev);
void kmb_isp_config_unregister(struct kmb_isp_config *kmb_cfg);

#endif /* KMB_ISP_CONFIG_VNODE_H */
