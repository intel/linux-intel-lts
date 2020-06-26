/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Keem Bay Camera Driver.
 *
 * Copyright (C) 2018-2020 Intel Corporation
 */
#ifndef KMB_MEDIA_H
#define KMB_MEDIA_H

#include <linux/xlink.h>
#include <media/v4l2-device.h>

#include "kmb-lrt-pipe.h"
#include "kmb-video.h"
#include "kmb-isp.h"

#define KMB_MAX_SENSOR_COUNT	10
#define KMB_MAX_OUTPUT_COUNT	6
#define KMB_MAX_OUTPUT_NAME_LEN	16

/**
 * struct kmb_cam_ctrl_channel - KMB Camera xlink control channel
 * @lock: Serialize access to control channel
 * @refcnt: Control channel reference count
 * @ipc_dev_handler: xLink IPC device handler
 * @cma_phy_addr: control channel physical CMA address
 * @cma_vaddr: control channel virtual CMA address
 */
struct kmb_cam_ctrl_channel {
	struct mutex lock;
	unsigned int refcnt;
	struct xlink_handle *ipc_dev_handler;
	u32 cma_phy_addr;
	void *cma_vaddr;
};

/**
 * struct kmb_cam_rt_pipeline - RT pipeline data
 * @pipe_instance_id: rt pipeline id
 * @raw_channel_id: raw channel id
 * @video_channel_id: array of video channel ids
 * @isp_channel_id: isp channel id
 * @id_alloc: pointer to id allocator
 * @pipe_cfg: rt pipeline configuration
 * @ctrl_chan: Pointer to camera control channel
 */
struct kmb_cam_rt_pipeline {
	u32 pipe_instance_id;
	u32 raw_channel_id;
	u32 video_channel_id[KMB_MAX_OUTPUT_COUNT];
	u32 isp_channel_id;
	struct ida *id_alloc;
	struct pipeConfigEvS pipe_cfg;
	struct kmb_cam_ctrl_channel *ctrl_chan;
};

/**
 * struct kmb_cam_pipeline - KMB camera sub-device set
 * @lock: serialize pipeline calls
 * @sensor: pointer to V4L2 sensor sub-device
 * @isp: KMB ISP sub-device
 * @output: KMB output video nodes array
 * @raw_input: KMB raw input video node
 * @raw_output: KMB raw output video node
 * @media_pipe: Media pipeline
 * @rt_pipe: RT pipeline
 * @config_refcnt: configure stream reference count
 * @streaming_refcnt: set stream reference count
 * @built_refcnt: pipeline built reference count
 */
struct kmb_cam_pipeline {
	struct mutex lock;
	struct v4l2_subdev *sensor;
	struct kmb_isp isp;
	struct kmb_video output[KMB_MAX_OUTPUT_COUNT];
	struct kmb_video raw_input;
	struct kmb_video raw_output;
	struct media_pipeline media_pipe;
	struct kmb_cam_rt_pipeline rt_pipe;
	unsigned int config_refcnt;
	unsigned int streaming_refcnt;
	unsigned int built_refcnt;
};

/**
 * struct kmb_cam_endpoint_data - KMB Camera configuration data
 * @remote: pointer to remote fwnode handle(sensor fwnode)
 * @ep_data: parsed endpoint data
 * @controller_num: mipi controller number
 */
struct kmb_cam_endpoint_data {
	struct fwnode_handle *remote;
	struct v4l2_fwnode_endpoint ep_data;
	u32 controller_num;
};

/**
 * struct kmb_camera - KMB Camera media device structure
 * @dev: pointer to basic device structure
 * @media_dev: media device
 * @v4l2_dev: V4L2 sub-device
 * @v4l2_notifier: array of V4L2 async notifiers
 * @pipeline: array of camera sub-devices and pipelines associated with them
 * @ep_data: array of endpoint configuration data
 * @id_alloc: xLink channel ID allocator
 * @idx: current max camera index
 * @ipc_dev_handler: xLink IPC device handler
 * @ctrl_channel: camera control channel
 */
struct kmb_camera {
	struct device *dev;
	struct media_device media_dev;
	struct v4l2_device v4l2_dev;
	struct v4l2_async_notifier v4l2_notifier[KMB_MAX_SENSOR_COUNT];

	struct kmb_cam_pipeline pipeline[KMB_MAX_SENSOR_COUNT];
	struct kmb_cam_endpoint_data ep_data[KMB_MAX_SENSOR_COUNT];
	struct ida id_alloc;
	int idx;

	struct xlink_handle ipc_dev_handler;
	struct kmb_cam_ctrl_channel ctrl_channel;
};

#endif /* KMB_MEDIA_H */
