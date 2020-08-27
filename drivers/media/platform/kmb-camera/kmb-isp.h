/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Keem Bay Camera ISP Driver.
 *
 * Copyright (C) 2018-2020 Intel Corporation
 */
#ifndef KMB_ISP_H
#define KMB_ISP_H

#include <linux/xlink.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>

#include "kmb-isp-config.h"
#include "kmb-lrt-src.h"

#define KMB_ISP_DRV_NAME "kmb-camera-isp"

#define KMB_ISP_SINK_PAD_SENSOR		0
#define KMB_ISP_SINK_PAD_CFG		1
#define KMB_ISP_SINK_PAD_RAW		2
#define KMB_ISP_SINK_PADS_NUM		3

#define KMB_ISP_SRC_PAD_RAW		KMB_ISP_SINK_PADS_NUM

/*
 * ISP source pads are connected to video nodes
 * pads are (base + index) first index is 0
 */
#define KMB_ISP_SRC_PAD_VID_BASE	(KMB_ISP_SRC_PAD_RAW + 1)
#define KMB_ISP_SRC_PADS_NUM		6
#define KMB_ISP_PADS_NUM		\
	(KMB_ISP_SINK_PADS_NUM + KMB_ISP_SRC_PADS_NUM + 1)

/* Predefined event queue length */
#define KMB_ISP_EVT_Q_LEN	8

#define KMB_ISP_MAX_SUPPORTED_FMTS	10

/**
 * struct kmb_isp_channel_ops - xLink channel ID operations
 * @accuire_id: accuire xlink channel ID
 * @free_id: free xlink channel ID
 */
struct kmb_isp_channel_ops {
	int (*accuire_id)(void *priv);
	void (*free_id)(void *priv, unsigned int id);
};

/**
 * enum source_type - Source interface type
 */
enum kmb_isp_source_type {
	KMB_ISP_NONE = 0,
	KMB_ISP_CSI2 = 1,
	KMB_ISP_LLVM = 2,
	KMB_ISP_MEMORY = 3
};

/**
 * struct kmb_isp_src_config - Source configuration
 * @type: source interface type
 * @num_phy_lanes: number of physical lanes
 * @controller_num: mipi controller number
 */
struct kmb_isp_src_config {
	enum kmb_isp_source_type type;
	u32 num_phy_lanes;
	u32 controller_num;
};

/**
 * struct kmb_isp_format - ISP formats description
 * @code: media bus format
 * @bayer_pattern: Bayer pattern
 * @bpp: bits per pixel (for the first plane if several)
 * @rx_data_type: LRT MIPI data type
 */
struct kmb_isp_format {
	u32 code;
	u32 bayer_pattern;
	u32 bpp;
	IcMipiRxDataTypeT rx_data_type;
};

/**
 * struct kmb_isp_res_range - ISP output resolution ranges
 * @min_width: minimum width supported by the LRT
 * @min_height: minimum height supported by the LRT
 * @max_width: maximum width supported by the LRT
 * @max_height: maximum height supported by the LRT
 */
struct kmb_isp_res_range {
	u32 min_width;
	u32 min_height;
	u32 max_width;
	u32 max_height;
};

/**
 * struct kmb_isp_format_range - ISP output resolution ranges
 * @range: resolution range supported by LRT
 * @num_fmts: number of supported formats
 * @code: array of supported media bus formats
 */
struct kmb_isp_format_caps {
	struct kmb_isp_res_range range;
	u32 num_fmts;
	u32 code[KMB_ISP_MAX_SUPPORTED_FMTS];
};

/**
 * struct kmb_isp_ctrls - KMB Camera isp controls structure
 * @handler: control handler
 * @transform_hub: transform hub
 * @camera_mode: camera mode
 * @update: update flag
 */
struct kmb_isp_ctrls {
	struct v4l2_ctrl_handler handler;
	struct v4l2_ctrl *transform_hub;
	struct v4l2_ctrl *camera_mode;
	u8 update;
};

/**
 * struct kmb_isp - KMB Camera ISP device structure
 * @dev: pointer to basic device structure
 * @lock: mutex
 * @thread: pointer to worker thread data
 * @ipc_dev_handler: xLink IPC device handler
 * @priv: pointer to private data
 * @channel_ops: xLink channel id operations
 * @channel_id: xLink channel id
 * @cma_phy_addr: ISP channel physical CMA address
 * @cma_vaddr: ISP channel virtual CMA address
 * @cfg_q_lock: mutex to serialize access to isp cfg bufferss queue
 * @isp_cfgs_queue: isp cfg buffers queue
 * @isp_streaming: flag to indicate isp state
 * @source_streaming: flag to indicate source state
 * @source_stopped: Completion to wait until LRT source is stopped
 * @subdev: V4L2 sub-device
 * @pads: array of media pad graph objects
 * @pipe: pointer to media pipeline related information
 * @pad_format: pad-level media bus formats
 * @pad_fmt_caps: pad format capabilities
 * @src_config: array of source configuration data
 * @ctrls: v4l2 custom controls
 * @config: KMB ISP Config vnode
 * @sequence: frame sequence number
 */
struct kmb_isp {
	struct device *dev;
	struct mutex lock;
	struct task_struct *thread;
	struct xlink_handle *ipc_dev_handler;

	void *priv;
	struct kmb_isp_channel_ops channel_ops;
	u32 channel_id;
	u32 cma_phy_addr;
	void *cma_vaddr;

	struct mutex cfg_q_lock;
	struct list_head isp_cfgs_queue;

	bool isp_streaming;
	bool source_streaming;
	struct completion source_stopped;

	struct v4l2_subdev subdev;
	struct media_pad pads[KMB_ISP_PADS_NUM];
	struct media_pipeline *pipe;

	struct v4l2_subdev_format pad_format[KMB_ISP_PADS_NUM];
	struct kmb_isp_format_caps pad_fmt_caps[KMB_ISP_PADS_NUM];
	struct kmb_isp_src_config src_config[KMB_ISP_SINK_PADS_NUM];

	struct kmb_isp_ctrls ctrls;

	struct kmb_isp_config config;
	u32 sequence;
};

void kmb_isp_init(struct kmb_isp *kmb_isp, struct device *dev);

int kmb_isp_register_entities(struct kmb_isp *kmb_isp,
			      struct v4l2_device *v4l2_dev);
void kmb_isp_unregister_entities(struct kmb_isp *kmb_isp);

u32 kmb_isp_get_channel_id(struct kmb_isp *kmb_isp);

u32 kmb_isp_get_lrt_transform_hub(struct kmb_isp *kmb_isp);
u32 kmb_isp_get_lrt_camera_mode(struct kmb_isp *kmb_isp);

int kmb_isp_set_pad_fmt_range(struct kmb_isp *kmb_isp, u32 pad,
			      struct kmb_isp_res_range *range);

const struct kmb_isp_format *kmb_isp_get_fmt_by_code(u32 code);

int kmb_isp_set_src_config(struct kmb_isp *kmb_isp, u32 pad,
			   struct kmb_isp_src_config *src_config);
#endif /* KMB_ISP_H */
