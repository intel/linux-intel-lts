// SPDX-License-Identifier: GPL-2.0-only
/*
 * Keem Bay Camera ISP Driver.
 *
 * Copyright (C) 2018-2020 Intel Corporation
 */
#include <linux/kmb-isp-ctl.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>

#include "kmb-isp.h"
#include "kmb-lrt-cmd.h"
#include "kmb-lrt-pipe.h"

#define KMB_ISP_MAX_ERR_CNT		5
#define KMB_RT_ISP_CH_DATA_SIZE		1024
#define KMB_RT_ISP_CH_TIMEOUT_MS	5000
#define RT_DATA_CH_ID_ISP		30
#define KMB_STOP_SOURCE_TIMEOUT_MS	1000

enum kmb_isp_stop_method {
	KMB_ISP_STOP_SYNC = 0,
	KMB_ISP_STOP_FORCE = 1,
};

static const struct kmb_isp_format isp_fmt[] = {
	{
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.bayer_pattern = IC_BAYER_FORMAT_RGGB,
		.bpp = 8,
		.rx_data_type = IC_IPIPE_RAW_8,
	},
	{
		.code = MEDIA_BUS_FMT_SGRBG8_1X8,
		.bayer_pattern = IC_BAYER_FORMAT_GRBG,
		.bpp = 8,
		.rx_data_type = IC_IPIPE_RAW_8,
	},
	{
		.code = MEDIA_BUS_FMT_SGBRG8_1X8,
		.bayer_pattern = IC_BAYER_FORMAT_GBRG,
		.bpp = 8,
		.rx_data_type = IC_IPIPE_RAW_8,
	},
	{
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.bayer_pattern = IC_BAYER_FORMAT_BGGR,
		.bpp = 8,
		.rx_data_type = IC_IPIPE_RAW_8,
	},
	{
		.code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.bayer_pattern = IC_BAYER_FORMAT_RGGB,
		.bpp = 10,
		.rx_data_type = IC_IPIPE_RAW_10,
	},
	{
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.bayer_pattern = IC_BAYER_FORMAT_GRBG,
		.bpp = 10,
		.rx_data_type = IC_IPIPE_RAW_10,
	},
	{
		.code = MEDIA_BUS_FMT_SGBRG10_1X10,
		.bayer_pattern = IC_BAYER_FORMAT_GBRG,
		.bpp = 10,
		.rx_data_type = IC_IPIPE_RAW_10,
	},
	{
		.code = MEDIA_BUS_FMT_SBGGR10_1X10,
		.bayer_pattern = IC_BAYER_FORMAT_BGGR,
		.bpp = 10,
		.rx_data_type = IC_IPIPE_RAW_10,
	},
	{
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.bayer_pattern = IC_BAYER_FORMAT_RGGB,
		.bpp = 12,
		.rx_data_type = IC_IPIPE_RAW_12,
	},
	{
		.code = MEDIA_BUS_FMT_SGRBG12_1X12,
		.bayer_pattern = IC_BAYER_FORMAT_GRBG,
		.bpp = 12,
		.rx_data_type = IC_IPIPE_RAW_12,
	},
	{
		.code = MEDIA_BUS_FMT_SGBRG12_1X12,
		.bayer_pattern = IC_BAYER_FORMAT_GBRG,
		.bpp = 12,
		.rx_data_type = IC_IPIPE_RAW_12,
	},
	{
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.bayer_pattern = IC_BAYER_FORMAT_BGGR,
		.bpp = 12,
		.rx_data_type = IC_IPIPE_RAW_12,
	},
	{
		.code = MEDIA_BUS_FMT_YUYV8_1_5X8,
		.bayer_pattern = IC_BAYER_FORMAT_BGGR,
		.bpp = 8,
		.rx_data_type = IC_IPIPE_YUV_420_B8,
	},
	{
		.code = MEDIA_BUS_FMT_UYYVYY8_0_5X24,
		.bayer_pattern = IC_BAYER_FORMAT_BGGR,
		.bpp = 8,
		.rx_data_type = IC_IPIPE_YUV_420_B8,
	},
	{
		.code = MEDIA_BUS_FMT_YUV8_1X24,
		.bayer_pattern = IC_BAYER_FORMAT_BGGR,
		.bpp = 8,
		.rx_data_type = IC_IPIPE_YUV_420_B8,
	},
	{
		.code = MEDIA_BUS_FMT_Y8_1X8,
		.bayer_pattern = IC_BAYER_FORMAT_BGGR,
		.bpp = 8,
		.rx_data_type = IC_IPIPE_RAW_8,
	},
	{
		.code = MEDIA_BUS_FMT_Y10_1X10,
		.bayer_pattern = IC_BAYER_FORMAT_BGGR,
		.bpp = 10,
		.rx_data_type = IC_IPIPE_RAW_10,
	},
};

static const u32 rt_isp_thb[][2] = {
	{ PIPE_TRANSFORM_HUB_NONE, KMB_CAMERA_TRANSFORM_HUB_NONE },
	{ PIPE_TRANSFORM_HUB_BASIC, KMB_CAMERA_TRANSFORM_HUB_BASIC },
	{ PIPE_TRANSFORM_HUB_FULL, KMB_CAMERA_TRANSFORM_HUB_FULL },
	{ PIPE_TRANSFORM_HUB_STITCH, KMB_CAMERA_TRANSFORM_HUB_STITCHING },
	{ PIPE_TRANSFORM_HUB_EPTZ, KMB_CAMERA_TRANSFORM_HUB_EPTZ },
};

static const u32 rt_isp_cam_mode[][2] = {
	{ PIPE_TYPE_ISP_ISP_ULL, KMB_CAMERA_MODE_ULL },
	{ PIPE_TYPE_ISP_ISP_2DOL, KMB_CAMERA_MODE_HDR_2DOL },
	{ PIPE_TYPE_ISP_ISP_3DOL, KMB_CAMERA_MODE_HDR_3DOL },
	{ PIPE_TYPE_ISP_ISP_MONO, KMB_CAMERA_MODE_MONO },
};

/**
 * kmb_isp_print_src_cfg - Print source configuration
 * @src_cfg: LRT source configuration
 *
 * Return: none
 */
static void kmb_isp_print_src_cfg(icSourceConfig *src_cfg)
{
	pr_debug("--SOURCE CONFIGURATION--\n");
	pr_debug("\tbayerFormat %u\n", src_cfg->bayerFormat);
	pr_debug("\tbitsPerPixel %u\n", src_cfg->bitsPerPixel);
	pr_debug("\tcameraOutputSize %ux%u\n",
		src_cfg->cameraOutputSize.w,
		src_cfg->cameraOutputSize.h);
	pr_debug("\tcropWindow x1 %d x2 %d  y1 %d y2 %d\n",
		src_cfg->cropWindow.x1, src_cfg->cropWindow.x2,
		src_cfg->cropWindow.y1, src_cfg->cropWindow.y2);
	pr_debug("\tmipiRxData controllerNo %u dataMode %u\n",
		src_cfg->mipiRxData.controllerNo,
		src_cfg->mipiRxData.dataMode);
	pr_debug("\t\tdataType %u laneRateMbps %u noLanes %u\n",
		src_cfg->mipiRxData.dataType,
		src_cfg->mipiRxData.laneRateMbps,
		src_cfg->mipiRxData.noLanes);
	pr_debug("\tnoExposure %u\n", src_cfg->noExposure);
	pr_debug("\tmetadataWidth %u metadataHeight %u metadataDataType %u\n",
		src_cfg->metadataWidth, src_cfg->metadataHeight,
		src_cfg->metadataDataType);
}

/**
 * kmb_isp_get_pad_format_by_idx - Get ISP pad format by given media pad index
 * @kmb_isp: pointer to kmb isp device
 * @idx: media pad index
 *
 * Return: pointer to isp pad format
 */
static struct v4l2_subdev_format *
kmb_isp_get_pad_format_by_idx(struct kmb_isp *kmb_isp, u32 idx)
{
	if (idx < KMB_ISP_PADS_NUM)
		return &kmb_isp->pad_format[idx];

	return NULL;
}

/**
 * kmb_isp_limit_camera_modes_range - Limit available camera modes based on
 *                                    sensor capabilities
 * @kmb_isp: pointer to kmb isp device
 */
static void kmb_isp_limit_camera_modes_range(struct kmb_isp *kmb_isp)
{
	if (MEDIA_BUS_FMT_Y10_1X10 ==
	    kmb_isp->pad_fmt_caps[KMB_ISP_SINK_PAD_SENSOR].code[0]) {
		__v4l2_ctrl_modify_range(kmb_isp->ctrls.camera_mode,
			KMB_CAMERA_MODE_MONO, KMB_CAMERA_MODE_MONO, 1,
			KMB_CAMERA_MODE_MONO);
	} else {
		__v4l2_ctrl_modify_range(kmb_isp->ctrls.camera_mode,
			KMB_CAMERA_MODE_ULL, KMB_CAMERA_MODE_HDR_3DOL, 1,
			KMB_CAMERA_MODE_ULL);
	}
}

/**
 * kmb_isp_get_fmt_by_code - Get Keem Bay ISP source format
 * @code: media bus format code
 *
 * Return: kmb isp source format or NULL
 */
const struct kmb_isp_format *kmb_isp_get_fmt_by_code(u32 code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(isp_fmt); i++) {
		if (isp_fmt[i].code == code)
			return &isp_fmt[i];
	}
	return NULL;
}

/**
 * kmb_isp_alloc_pipe_cma_area - Allocate CMA area
 * @kmb_isp: pointer to KMB ISP device
 *
 * Allocate continuous memory for data structures used by RT
 *
 * Return: 0 if successful
 */
static int kmb_isp_alloc_pipe_cma_area(struct kmb_isp *kmb_isp)
{
	kmb_isp->cma_vaddr = dma_alloc_coherent(kmb_isp->dev,
			sizeof(icSourceConfig),
			(dma_addr_t *)&kmb_isp->cma_phy_addr, 0);
	if (!kmb_isp->cma_vaddr)
		return -ENOMEM;

	dev_dbg(kmb_isp->dev, "ISP CHANNEL CMA: phy 0x%x vaddr %p\n",
			kmb_isp->cma_phy_addr, kmb_isp->cma_vaddr);

	return 0;
}

/**
 * kmb_isp_free_pipe_cma_area - Free CMA area
 * @kmb_isp: pointer to KMB ISP device
 *
 * Free continuous memory for data structures used by RT
 */
static void kmb_isp_free_pipe_cma_area(struct kmb_isp *kmb_isp)
{
	if (kmb_isp->cma_vaddr)
		dma_free_coherent(kmb_isp->dev, sizeof(icSourceConfig),
				kmb_isp->cma_vaddr, kmb_isp->cma_phy_addr);
	kmb_isp->cma_vaddr = NULL;
}

/**
 * kmb_isp_reset_pad_formats - Reset ISP pads format
 * @kmb_isp: pointer to KMB ISP device
 */
static void kmb_isp_reset_pad_formats(struct kmb_isp *kmb_isp)
{
	memset(kmb_isp->pad_format, 0, sizeof(kmb_isp->pad_format));
	memset(kmb_isp->pad_fmt_caps, 0, sizeof(kmb_isp->pad_fmt_caps));

	/* For now set input pads to max range */
	kmb_isp->pad_fmt_caps[KMB_ISP_SINK_PAD_SENSOR].range.max_width =
			U32_MAX;
	kmb_isp->pad_fmt_caps[KMB_ISP_SINK_PAD_SENSOR].range.max_height =
			U32_MAX;

	kmb_isp->pad_fmt_caps[KMB_ISP_SINK_PAD_CFG].range.max_width = U32_MAX;
	kmb_isp->pad_fmt_caps[KMB_ISP_SINK_PAD_CFG].range.max_height = U32_MAX;

	kmb_isp->pad_fmt_caps[KMB_ISP_SINK_PAD_RAW].range.max_width = U32_MAX;
	kmb_isp->pad_fmt_caps[KMB_ISP_SINK_PAD_RAW].range.max_height = U32_MAX;
}

/**
 * kmb_isp_populate_pad_formats - Populate pad formats based on supported
 *                                sensor media bus formats
 * @kmb_isp: pointer to KMB ISP device
 *
 * Return: 0 if successful
 */
static int kmb_isp_populate_pad_formats(struct kmb_isp *kmb_isp)
{
	struct v4l2_subdev_mbus_code_enum mbus;
	struct media_pad *remote;
	struct v4l2_subdev *subdev;
	u32 code[30] = {0};
	int isp_pad_idx, in_fmt_cnt, out_fmt_cnt;
	int ret = 0;

	remote = media_entity_remote_pad(
			&kmb_isp->pads[KMB_ISP_SINK_PAD_SENSOR]);
	if (remote == NULL)
		return -ENOENT;

	subdev = media_entity_to_v4l2_subdev(remote->entity);
	if (subdev == NULL)
		return -ENOENT;

	in_fmt_cnt = 0;
	memset(&mbus, 0, sizeof(mbus));
	mbus.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	while (!v4l2_subdev_call(subdev, pad, enum_mbus_code,
					NULL, &mbus)) {
		switch (mbus.code) {
		case MEDIA_BUS_FMT_SRGGB8_1X8:
		case MEDIA_BUS_FMT_SGRBG8_1X8:
		case MEDIA_BUS_FMT_SGBRG8_1X8:
		case MEDIA_BUS_FMT_SBGGR8_1X8:
		case MEDIA_BUS_FMT_SRGGB10_1X10:
		case MEDIA_BUS_FMT_SGRBG10_1X10:
		case MEDIA_BUS_FMT_SGBRG10_1X10:
		case MEDIA_BUS_FMT_SBGGR10_1X10:
		case MEDIA_BUS_FMT_SRGGB12_1X12:
		case MEDIA_BUS_FMT_SGRBG12_1X12:
		case MEDIA_BUS_FMT_SGBRG12_1X12:
		case MEDIA_BUS_FMT_SBGGR12_1X12:
			code[in_fmt_cnt++] = MEDIA_BUS_FMT_YUYV8_1_5X8;
			code[in_fmt_cnt++] = MEDIA_BUS_FMT_UYYVYY8_0_5X24;
			code[in_fmt_cnt++] = MEDIA_BUS_FMT_YUV8_1X24;
			break;
		case MEDIA_BUS_FMT_Y10_1X10:
			code[in_fmt_cnt++] = MEDIA_BUS_FMT_Y8_1X8;
			code[in_fmt_cnt++] = MEDIA_BUS_FMT_Y10_1X10;
			break;
		default:
			dev_err(kmb_isp->dev, "Input format not supported! %d",
				mbus.code);
			return -EINVAL;
			/* fallthrough */
		}

		kmb_isp->pad_fmt_caps[mbus.pad].code[mbus.index] = mbus.code;
		kmb_isp->pad_fmt_caps[KMB_ISP_SRC_PAD_RAW].code[mbus.index] =
				mbus.code;
		kmb_isp->pad_fmt_caps[KMB_ISP_SINK_PAD_RAW].code[mbus.index] =
				mbus.code;

		mbus.index++;

		kmb_isp->pad_fmt_caps[mbus.pad].num_fmts = mbus.index;
		kmb_isp->pad_fmt_caps[KMB_ISP_SRC_PAD_RAW].num_fmts =
				mbus.index;
		kmb_isp->pad_fmt_caps[KMB_ISP_SINK_PAD_RAW].num_fmts =
				mbus.index;
	}

	for (isp_pad_idx = KMB_ISP_SRC_PAD_VID_BASE;
		isp_pad_idx < KMB_ISP_PADS_NUM; isp_pad_idx++) {
		for (out_fmt_cnt = 0; out_fmt_cnt < in_fmt_cnt;) {
			kmb_isp->pad_fmt_caps[isp_pad_idx].code[out_fmt_cnt] =
				code[out_fmt_cnt];
			kmb_isp->pad_fmt_caps[isp_pad_idx].num_fmts =
				++out_fmt_cnt;
		}
	}

	return ret;
}

/**
 * kmb_isp_ipc_event - Handle IPC events
 * @kmb_isp: pointer to KMB ISP device
 * @evt: ipc event
 * @evt_priv: isp cfg buffer private data
 */
static void kmb_isp_ipc_event(struct kmb_isp *kmb_isp,
		const __u32 evt,
		const struct kmb_isp_cfg_event_data evt_priv)
{
	struct v4l2_event v4l2_evt = {0};
	struct kmb_isp_cfg_event_data *prv_data =
		(struct kmb_isp_cfg_event_data *) v4l2_evt.u.data;

	prv_data->valid = evt_priv.valid;
	if (evt_priv.valid)
		prv_data->index = evt_priv.index;

	v4l2_evt.type = evt;

	v4l2_subdev_notify_event(&kmb_isp->subdev, &v4l2_evt);
	dev_dbg(kmb_isp->dev, "Notify event type:%d\n", evt);
}

/**
 * kmb_isp_insert_config - Insert isp config buffer in queue
 * @kmb_isp: pointer to KMB ISP device
 * @cfg: pointer to ISP configuration data
 */
static void kmb_isp_insert_config(struct kmb_isp *kmb_isp,
				  struct kmb_isp_cfg_data *cfg)
{
	mutex_lock(&kmb_isp->cfg_q_lock);
	INIT_LIST_HEAD(&cfg->isp_cfgs);
	list_add_tail(&cfg->isp_cfgs, &kmb_isp->isp_cfgs_queue);
	mutex_unlock(&kmb_isp->cfg_q_lock);
}

/**
 * kmb_isp_config_done - Remove isp config from internal queue and
 *                       make buf done on video buffer queue.
 * @kmb_isp: pointer to KMB ISP device
 * @cfg: pointer to ISP configuration data
 */
static void kmb_isp_config_done(struct kmb_isp *kmb_isp,
				struct kmb_isp_cfg_data *cfg)
{
	/* Remove isp config on buf done */
	mutex_lock(&kmb_isp->cfg_q_lock);
	list_del(&cfg->isp_cfgs);
	mutex_unlock(&kmb_isp->cfg_q_lock);

	vb2_buffer_done(&cfg->vb.vb2_buf, VB2_BUF_STATE_DONE);
	dev_dbg(kmb_isp->dev, "Isp cfg done %pad", &cfg->addr);
}

/**
 * kmb_isp_find_config_by_addr - Get isp config by given physical address
 * @kmb_isp: pointer to KMB ISP device
 * @addr: physical dma address
 *
 * Return: pointer to isp config data
 */
static struct kmb_isp_cfg_data *
kmb_isp_find_config_by_addr(struct kmb_isp *kmb_isp, dma_addr_t addr)
{
	struct kmb_isp_cfg_data *isp_cfg = NULL;
	struct list_head *node, *next;

	mutex_lock(&kmb_isp->cfg_q_lock);

	list_for_each_safe(node, next, &kmb_isp->isp_cfgs_queue) {
		isp_cfg = list_entry(node,
				struct kmb_isp_cfg_data, isp_cfgs);
		if (isp_cfg->addr == addr)
			break;

		isp_cfg = NULL;
	}

	mutex_unlock(&kmb_isp->cfg_q_lock);

	return isp_cfg;
}

/**
 * kmb_isp_get_lane_info - Get lane rate and number of data lanes
 * @kmb_isp: pointer to KMB ISP device
 * @subdev: pointer to sensor sub-device
 * @bpp: input format bits per pixel
 * @lane_rate_mbps: lane rate
 * @num_used_lanes: number of used data lanes
 *
 * Return: 0 if successful
 */
static int kmb_isp_get_lane_info(struct kmb_isp *kmb_isp,
				 struct v4l2_subdev *subdev, u32 bpp,
				 u32 *lane_rate_mbps, u32 *num_used_lanes)
{
	struct v4l2_ctrl *ctrl;
	struct v4l2_querymenu link_freq = {.id = V4L2_CID_LINK_FREQ, };
	struct kmb_isp_src_config *src_config;
	u32 pix_rate;
	int ret;

	ctrl = v4l2_ctrl_find(subdev->ctrl_handler, V4L2_CID_PIXEL_RATE);
	if (!ctrl) {
		dev_err(kmb_isp->dev, "Can't find V4L2_CID_PIXEL_RATE ctrl");
		return -EINVAL;
	}

	/* Avoid division of 64 bit numbers */
	pix_rate = v4l2_ctrl_g_ctrl_int64(ctrl);
	pix_rate /= 1000000;

	ctrl = v4l2_ctrl_find(subdev->ctrl_handler, V4L2_CID_LINK_FREQ);
	if (!ctrl) {
		dev_dbg(kmb_isp->dev, "Can't find V4L2_CID_LINK_FREQ ctrl");

		src_config = &kmb_isp->src_config[KMB_ISP_SINK_PAD_SENSOR];
		*num_used_lanes = src_config->num_phy_lanes;
		*lane_rate_mbps = pix_rate * bpp / *num_used_lanes;
	} else {
		link_freq.index = v4l2_ctrl_g_ctrl(ctrl);
		ret = v4l2_querymenu(subdev->ctrl_handler, &link_freq);
		if (ret)
			return ret;

		*lane_rate_mbps = link_freq.value;
		*lane_rate_mbps /= 1000000;
		*num_used_lanes = pix_rate * bpp / *lane_rate_mbps;
	}

	return 0;
}

/**
 * kmb_isp_get_meta_info - Get metadata size
 * @subdev: pointer to sensor sub-device
 * @bpp: input format bits per pixel
 * @width: input width
 * @meta_width: metadata width
 * @meta_height: metadata height
 *
 * Return: none
 */
static void kmb_isp_get_meta_info(struct v4l2_subdev *subdev, u32 bpp,
				 u32 width, u32 *meta_width, u32 *meta_height)
{
	struct v4l2_mbus_frame_desc desc;
	u32 embedded_data = 0;
	int ret;

	ret = v4l2_subdev_call(subdev, sensor, g_skip_top_lines, meta_height);
	if (ret < 0) {
		pr_debug("Couldn't get sensor metadata lines\n");
		*meta_height = 0;
		*meta_width = 0;
	}

	ret = v4l2_subdev_call(subdev, pad, get_frame_desc, 0, &desc);
	if (ret < 0)
		pr_debug("Couldn't get sensor metadata lines\n");
	else
		embedded_data = desc.entry[0].length;

	if (embedded_data) {
		*meta_width = embedded_data +
			(width * *meta_height * bpp / 8);
		*meta_height = 0;
	} else if (*meta_height) {
		*meta_width = width;
	}
}

/**
 * kmb_isp_worker_thread - ISP worker thread
 * @isp: pointer to KMB ISP sub-device
 *
 * Return: 0 if successful
 */
static int kmb_isp_worker_thread(void *isp)
{
	struct kmb_isp *kmb_isp = isp;
	struct icEvS cfg_evt;
	size_t cfg_evt_size;
	struct kmb_isp_cfg_event_data evt_priv;
	struct kmb_isp_cfg_data *isp_cfg;
	bool stopped = false;
	int ret;

	set_freezable();

	while (!kthread_should_stop()) {
		try_to_freeze();

		if (stopped) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule();
			dev_dbg(kmb_isp->dev, "Source is stopped\n");
			continue;
		}

		memset(&cfg_evt, 0x00, sizeof(cfg_evt));
		cfg_evt.ctrl = IC_EVENT_MAX;
		cfg_evt_size = sizeof(cfg_evt);
		ret = xlink_read_data_to_buffer(kmb_isp->ipc_dev_handler,
				kmb_isp->channel_id, (uint8_t *const)&cfg_evt,
				(uint32_t *)&cfg_evt_size);
		if (ret != X_LINK_SUCCESS) {
			dev_dbg(kmb_isp->dev, "Channel closed %d\n", ret);
			stopped = true;
			complete_all(&kmb_isp->source_stopped);
			continue;
		}

		isp_cfg = kmb_isp_find_config_by_addr(kmb_isp,
				cfg_evt.EvInfo.userDataBaseAddr01);
		if (isp_cfg) {
			evt_priv.index = isp_cfg->vb.vb2_buf.index;
			evt_priv.valid = 1;
		} else {
			evt_priv.index = 0;
			evt_priv.valid = 0;
		}

		switch (cfg_evt.ctrl) {
		case IC_EVENT_TYPE_READOUT_START:
			kmb_isp_ipc_event(kmb_isp, V4L2_EVENT_KMB_SENSOR_SOF,
					  evt_priv);
			break;
		case IC_EVENT_TYPE_READOUT_END:
			kmb_isp_ipc_event(kmb_isp, V4L2_EVENT_KMB_SENSOR_EOF,
					  evt_priv);
			break;
		case IC_EVENT_TYPE_ISP_START:
			kmb_isp_ipc_event(kmb_isp, V4L2_EVENT_KMB_ISP_SOF,
					  evt_priv);
			break;
		case IC_EVENT_TYPE_ISP_END:
			if (isp_cfg) {
				kmb_isp_ipc_event(kmb_isp,
						  V4L2_EVENT_KMB_ISP_EOF,
						  evt_priv);
				kmb_isp_config_done(kmb_isp, isp_cfg);
			} else {
				kmb_isp_ipc_event(kmb_isp,
						  V4L2_EVENT_KMB_GEN_ERROR,
						  evt_priv);
				dev_err(kmb_isp->dev,
					"Receive ISP_END without config %x",
					cfg_evt.EvInfo.userDataBaseAddr01);
			}
			break;
		case IC_EVENT_TYPE_STATS_READY:
			/* Do nothing */
			break;
		case IC_ERROR_SRC_MIPI_CFG_SKIPPED:
			if (isp_cfg) {
				kmb_isp_ipc_event(kmb_isp,
						  V4L2_EVENT_KMB_CFG_SKIPPED,
						  evt_priv);
				kmb_isp_config_done(kmb_isp, isp_cfg);
			} else {
				kmb_isp_ipc_event(kmb_isp,
						  V4L2_EVENT_KMB_GEN_ERROR,
						  evt_priv);
				dev_err(kmb_isp->dev,
					"Receive CFG_SKIP without config %x",
					cfg_evt.EvInfo.userDataBaseAddr01);
			}
			break;
		case IC_ERROR_SRC_MIPI_CFG_MISSING:
			kmb_isp_ipc_event(kmb_isp, V4L2_EVENT_KMB_CFG_MISSING,
					  evt_priv);
			break;
		case IC_ERROR_SRC_MIPI_OUT_BUFFERS_NOT_AVAILABLE:
			kmb_isp_ipc_event(kmb_isp, V4L2_EVENT_KMB_BUF_MISSING,
					  evt_priv);
			break;
		case IC_EVENT_TYPE_SOURCE_STOPPED:
			complete_all(&kmb_isp->source_stopped);
			stopped = true;
			dev_dbg(kmb_isp->dev, "Source stopped\n");
			break;
		default:
			dev_dbg(kmb_isp->dev, "Received event %d",
					cfg_evt.ctrl);
			break;
		}
	}
	dev_dbg(kmb_isp->dev, "ISP thread exits\n");
	return 0;
}

/**
 * kmb_isp_configure_source - ISP configure source event
 * @kmb_isp: pointer to KMB ISP sub-device
 *
 * Return: 0 if successful
 */
static int kmb_isp_configure_source(struct kmb_isp *kmb_isp)
{
	struct icEvS mipi_cfg_evt;
	size_t mipi_cfg_evt_size;
	icSourceConfig *src_cfg;
	struct v4l2_subdev_format sd_fmt;
	const struct kmb_isp_format *fmt;
	struct media_pad *remote;
	struct v4l2_subdev *subdev;
	int ret;

	remote = media_entity_remote_pad(
			&kmb_isp->pads[KMB_ISP_SINK_PAD_SENSOR]);
	if (remote == NULL)
		return -ENOENT;

	subdev = media_entity_to_v4l2_subdev(remote->entity);
	if (subdev == NULL)
		return -ENOENT;

	memset(&sd_fmt, 0, sizeof(sd_fmt));
	sd_fmt.pad = remote->index;
	sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(subdev, pad, get_fmt, NULL, &sd_fmt);
	if (ret < 0) {
		dev_err(kmb_isp->dev, "Couldn't get sensor format\n");
		return ret;
	}

	fmt = kmb_isp_get_fmt_by_code(sd_fmt.format.code);
	if (!fmt) {
		dev_info(kmb_isp->dev, "Couldn't get source format\n");
		fmt = &isp_fmt[0];
	}

	src_cfg = kmb_isp->cma_vaddr;
	memset(src_cfg, 0, sizeof(*src_cfg));

	src_cfg->bayerFormat = fmt->bayer_pattern;
	src_cfg->bitsPerPixel = fmt->bpp;

	/* Not scaled isp output is always set on first output pad */
	src_cfg->cameraOutputSize.w = sd_fmt.format.width;
	src_cfg->cameraOutputSize.h = sd_fmt.format.height;

	src_cfg->noExposure =
		sd_fmt.format.reserved[0] ? sd_fmt.format.reserved[0] : 1;

	src_cfg->cropWindow.x1 = 0;
	src_cfg->cropWindow.x2 = src_cfg->cameraOutputSize.w;
	src_cfg->cropWindow.y1 = 0;
	src_cfg->cropWindow.y2 = src_cfg->cameraOutputSize.h;

	src_cfg->mipiRxData.controllerNo =
		kmb_isp->src_config[KMB_ISP_SINK_PAD_SENSOR].controller_num;
	src_cfg->mipiRxData.dataMode = 1;
	src_cfg->mipiRxData.dataType = fmt->rx_data_type;

	/* This configuration should be negotiated with sensor */
	ret = kmb_isp_get_lane_info(kmb_isp, subdev, fmt->bpp,
				    &src_cfg->mipiRxData.laneRateMbps,
				    &src_cfg->mipiRxData.noLanes);
	if (ret < 0) {
		dev_err(kmb_isp->dev, "Couldn't get lane info\n");
		return ret;
	}

	kmb_isp_get_meta_info(subdev, fmt->bpp, sd_fmt.format.width,
			      &src_cfg->metadataWidth,
			      &src_cfg->metadataHeight);

	src_cfg->metadataDataType = IC_IPIPE_EMBEDDED_8BIT;

	dev_dbg(kmb_isp->dev,
		"icEvS ctrl IC_EVENT_TYPE_CONFIG_SOURCE channel id %d\n",
		kmb_isp->channel_id);

	kmb_isp_print_src_cfg(src_cfg);

	memset(&mipi_cfg_evt, 0, sizeof(mipi_cfg_evt));
	mipi_cfg_evt_size = sizeof(mipi_cfg_evt);
	mipi_cfg_evt.ctrl = IC_EVENT_TYPE_CONFIG_SOURCE;
	mipi_cfg_evt.EvInfo.instId = 0;
	mipi_cfg_evt.EvInfo.userDataBaseAddr01 = kmb_isp->cma_phy_addr;

	ret = xlink_write_volatile(kmb_isp->ipc_dev_handler,
			kmb_isp->channel_id,
			(u8 *)&mipi_cfg_evt,
			(uint32_t)mipi_cfg_evt_size);
	if (ret) {
		dev_err(kmb_isp->dev,
				"Error on IC_EVENT_TYPE_CONFIG_SOURCE %d\n",
				ret);
		return ret;
	}

	ret = xlink_read_data_to_buffer(kmb_isp->ipc_dev_handler,
			kmb_isp->channel_id,
			(uint8_t *const)&mipi_cfg_evt,
			(uint32_t *)&mipi_cfg_evt_size);
	if ((ret != X_LINK_SUCCESS) ||
	    (mipi_cfg_evt.ctrl != IC_EVENT_TYPE_SOURCE_CONFIGURED)) {
		dev_err(kmb_isp->dev, "Config source fail ret %d ctrl %d\n",
				ret, mipi_cfg_evt.ctrl);
		return ret ? ret : -ENODEV;
	}

	dev_dbg(kmb_isp->dev,
		"rcv icEvS ctrl IC_EVENT_TYPE_CONFIG_SOURCE channel id %d\n",
		kmb_isp->channel_id);

	kmb_isp_print_src_cfg(src_cfg);

	return 0;
}

/**
 * kmb_isp_start_source - ISP start source event
 * @kmb_isp: pointer to KMB ISP sub-device
 *
 * Return: 0 if successful
 */
static int kmb_isp_start_source(struct kmb_isp *kmb_isp)
{
	struct icEvS cfg_evt;
	size_t cfg_evt_size;
	int ret;

	BUG_ON(kmb_isp->source_streaming);

	memset(&cfg_evt, 0, sizeof(cfg_evt));
	cfg_evt_size = sizeof(cfg_evt);
	cfg_evt.ctrl = IC_EVENT_TYPE_START_SOURCE;
	cfg_evt.EvInfo.instId = 0;

	dev_dbg(kmb_isp->dev, "IC_EVENT_TYPE_START_SOURCE\n");
	ret = xlink_write_volatile(kmb_isp->ipc_dev_handler,
			kmb_isp->channel_id,
			(u8 *)&cfg_evt,
			cfg_evt_size);
	if (ret) {
		dev_err(kmb_isp->dev, "Error on IC_EVENT_TYPE_START_SOURCE %d\n",
				ret);
		return ret;
	}

	ret = xlink_read_data_to_buffer(kmb_isp->ipc_dev_handler,
			kmb_isp->channel_id,
			(uint8_t *const)&cfg_evt,
			(uint32_t *)&cfg_evt_size);
	if ((ret != X_LINK_SUCCESS) ||
	    (cfg_evt.ctrl != IC_EVENT_TYPE_SOURCE_STARTED)) {
		dev_err(kmb_isp->dev, "Start source fail %d ctrl %d\n",
				ret, cfg_evt.ctrl);
		return ret ? ret : -ENODEV;
	}

	init_completion(&kmb_isp->source_stopped);
	kmb_isp->thread = kthread_run(kmb_isp_worker_thread,
			kmb_isp, "kmb_isp_thread");
	if (IS_ERR(kmb_isp->thread)) {
		kmb_isp->thread = NULL;
		dev_err(kmb_isp->dev, "%s Thread run failed\n", __func__);
		return -ENOMEM;
	}

	kmb_isp->source_streaming = true;

	return 0;
}

/**
 * kmb_isp_stop_source - ISP stop source event
 * @kmb_isp: pointer to KMB ISP sub-device
 * @method: stop source method
 *
 * Return: 0 if successful
 */
static int kmb_isp_stop_source(struct kmb_isp *kmb_isp,
			       enum kmb_isp_stop_method method)
{
	struct icEvS cfg_evt;
	int ret;

	BUG_ON(!kmb_isp->source_streaming);

	if (method == KMB_ISP_STOP_SYNC) {
		memset(&cfg_evt, 0, sizeof(cfg_evt));
		cfg_evt.ctrl = IC_EVENT_TYPE_STOP_SOURCE;
		cfg_evt.EvInfo.instId = 0;

		ret = xlink_write_volatile(kmb_isp->ipc_dev_handler,
				kmb_isp->channel_id,
				(u8 *)&cfg_evt,
				sizeof(cfg_evt));
		if (ret) {
			dev_err(kmb_isp->dev,
				"Error IC_EVENT_TYPE_STOP_SOURCE %d\n", ret);
			return ret;
		}

		ret = wait_for_completion_timeout(&kmb_isp->source_stopped,
			msecs_to_jiffies(KMB_STOP_SOURCE_TIMEOUT_MS));
		if (ret == 0) {
			dev_err(kmb_isp->dev, "Source stopped timeout");
			return -ETIMEDOUT;
		}
	}

	if (kmb_isp->thread) {
		ret = kthread_stop(kmb_isp->thread);
		if (ret < 0) {
			dev_err(kmb_isp->dev,
				"%s Thread stop failed ret = %d\n",
				__func__, ret);
			return ret;
		}
		kmb_isp->thread = NULL;
		dev_dbg(kmb_isp->dev, "%s Thread stopped ret = %d\n",
			__func__, ret);
	}

	kmb_isp->source_streaming = false;

	return 0;
}

/**
 * kmb_isp_process_config - Process an ISP config
 * @kmb_isp: pointer to KMB ISP sub-device
 * @buf: pointer to isp config buffer
 *
 * Return: 0 if successful
 */
static int kmb_isp_process_config(struct kmb_isp *kmb_isp,
				  struct kmb_isp_cfg_data *buf)
{
	struct icEvS cfg_evt;
	uint32_t cfg_evt_size;
	int ret;

	memset(&cfg_evt, 0, sizeof(cfg_evt));
	cfg_evt_size = sizeof(cfg_evt);
	cfg_evt.ctrl = IC_EVENT_TYPE_CONFIG_ISP;
	cfg_evt.EvInfo.seqNr = kmb_isp->sequence++;
	cfg_evt.EvInfo.userDataBaseAddr01 = buf->addr;

	dev_dbg(kmb_isp->dev, "IC_EVENT_TYPE_CONFIG_ISP %pad\n", &buf->addr);

	ret = xlink_write_volatile(kmb_isp->ipc_dev_handler,
				 kmb_isp->channel_id,
				 (u8 *)&cfg_evt,
				 cfg_evt_size);
	if (ret) {
		dev_err(kmb_isp->dev, "Error on IC_EVENT_TYPE_CONFIG_ISP %d\n",
				ret);
		return ret;
	}

	/* Start source if is not running */
	if (!kmb_isp->source_streaming) {
		ret = kmb_isp_start_source(kmb_isp);
		if (ret) {
			dev_err(kmb_isp->dev, "Can not start isp source");
			return ret;
		}
	}

	return 0;
}

/**
 * kmb_isp_discard_config - Discard isp config buffer
 * @buf: pointer to isp config buffer
 *
 * Return: 0 if successful
 */
static void kmb_isp_discard_config(struct kmb_isp_cfg_data *buf)
{
	list_del(&buf->isp_cfgs);
	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
}

/**
 * kmb_isp_process_all_configs - Process all isp configs
 * @kmb_isp: pointer to KMB ISP sub-device
 *
 * Return: 0 if successful
 */
static int kmb_isp_process_all_configs(struct kmb_isp *kmb_isp)
{
	struct kmb_isp_cfg_data *buf;
	struct list_head *pos, *next;
	int ret = 0;

	mutex_lock(&kmb_isp->cfg_q_lock);
	list_for_each_safe(pos, next, &kmb_isp->isp_cfgs_queue) {
		buf = list_entry(pos, struct kmb_isp_cfg_data, isp_cfgs);

		ret = kmb_isp_process_config(kmb_isp, buf);
		if (ret) {
			dev_err(kmb_isp->dev, "Can not process isp config");
			kmb_isp_discard_config(buf);
			continue;
		}
	}
	mutex_unlock(&kmb_isp->cfg_q_lock);

	return ret;
}

/**
 * kmb_isp_discard_all_configs - Discard all isp configs
 * @kmb_isp: pointer to KMB ISP sub-device
 *
 * Return: 0 if successful
 */
static void kmb_isp_discard_all_configs(struct kmb_isp *kmb_isp)
{
	struct kmb_isp_cfg_data *buf;
	struct list_head *pos, *next;
	struct kmb_isp_cfg_event_data event;

	event.valid = 1;
	mutex_lock(&kmb_isp->cfg_q_lock);
	list_for_each_safe(pos, next, &kmb_isp->isp_cfgs_queue) {
		buf = list_entry(pos, struct kmb_isp_cfg_data, isp_cfgs);
		list_del(&buf->isp_cfgs);

		/* Notify that configuration is skipped */
		event.index = buf->vb.vb2_buf.index;
		kmb_isp_ipc_event(kmb_isp, V4L2_EVENT_KMB_CFG_SKIPPED, event);

		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	mutex_unlock(&kmb_isp->cfg_q_lock);
}

/**
 * kmb_isp_queue_isp_cfg_buf - Queue isp cfg buffer
 * @priv: pointer to KMB ISP sub-device
 * @cfg: isp cfg buffer to queue
 *
 * Return: 0 if successful
 */
static int kmb_isp_queue_isp_cfg_buf(void *priv, struct kmb_isp_cfg_data *cfg)
{
	struct kmb_isp *kmb_isp = priv;
	int ret = 0;

	kmb_isp_insert_config(kmb_isp, cfg);

	mutex_lock(&kmb_isp->lock);

	/* Process ISP configurations only when ISP is streaming */
	if (kmb_isp->isp_streaming) {
		ret = kmb_isp_process_config(kmb_isp, cfg);
		if (ret)
			dev_err(kmb_isp->dev, "Fail to process isp config\n");
	}

	mutex_unlock(&kmb_isp->lock);

	return ret;
}

/**
 * kmb_isp_queue_flush - Flush isp cfg buffers
 * @priv: pointer to KMB ISP sub-device
 *
 * Return: 0 if successful
 */
static void kmb_isp_queue_flush(void *priv)
{
	kmb_isp_discard_all_configs(priv);
}

struct kmb_isp_cfg_queue_ops isp_cfg_queue_ops = {
	.queue = kmb_isp_queue_isp_cfg_buf,
	.flush = kmb_isp_queue_flush,
};

/**
 * kmb_isp_open_xlink_channel - Open xlink channel for ISP
 * @kmb_isp: pointer to ISP sub-device
 *
 * Return: 0 if successful
 */
static int kmb_isp_open_xlink_channel(struct kmb_isp *kmb_isp)
{
	int ret;

	ret = xlink_open_channel(kmb_isp->ipc_dev_handler, kmb_isp->channel_id,
				 RXB_TXB, KMB_RT_ISP_CH_DATA_SIZE,
				 KMB_RT_ISP_CH_TIMEOUT_MS);
	if (ret) {
		dev_err(kmb_isp->dev, "Failed to open ISP channel: %d\n", ret);
		return ret;
	}
	dev_dbg(kmb_isp->dev, "Opened ISP channel %u\n", kmb_isp->channel_id);

	return 0;
}

/**
 * kmb_isp_close_xlink_channel - Close xlink channel for ISP
 * @kmb_isp: pointer to ISP sub-device
 *
 * Return: 0 if successful
 */
static int kmb_isp_close_xlink_channel(struct kmb_isp *kmb_isp)
{
	int ret;

	ret = xlink_close_channel(kmb_isp->ipc_dev_handler,
				  kmb_isp->channel_id);
	if (ret) {
		dev_err(kmb_isp->dev,
				"Failed to close ISP channel: %d\n", ret);
		return ret;
	}
	dev_dbg(kmb_isp->dev, "Closed ISP channel\n");

	return 0;
}

/**
 * kmb_isp_open - Sub-device node open
 * @subdev: pointer to V4L2 sub-device
 * @fh: pointer to V4L2 sub-device file handle
 *
 * Return: 0 if successful
 */
static int kmb_isp_open(struct v4l2_subdev *subdev,
			struct v4l2_subdev_fh *fh)
{
	return 0;
}

/**
 * kmb_isp_close - Sub-device node close
 * @subdev: pointer to V4L2 sub-device
 * @fh: pointer to V4L2 sub-device file handle
 *
 * Return: 0 if successful
 */
static int kmb_isp_close(struct v4l2_subdev *subdev,
			 struct v4l2_subdev_fh *fh)
{
	return 0;
}

/* sub-device internal operations */
static const struct v4l2_subdev_internal_ops kmb_isp_internal_ops = {
	.open = kmb_isp_open,
	.close = kmb_isp_close,
};

/**
 * kmb_isp_get_closest_res - Set format ratio to the closest possible
 * @kmb_isp: pointer to ISP sub-device
 * @pad: ISP pad index
 * @format: pointer to media bus frame format
 *
 * Return: 0 if successful
 */
static void kmb_isp_align_fmt(struct kmb_isp *kmb_isp, u32 pad,
			      struct v4l2_mbus_framefmt *format)
{
	if (pad >= KMB_ISP_PADS_NUM)
		return;

	format->width = clamp(format->width,
			      kmb_isp->pad_fmt_caps[pad].range.min_width,
			      kmb_isp->pad_fmt_caps[pad].range.max_width);

	format->height = clamp(format->height,
			      kmb_isp->pad_fmt_caps[pad].range.min_height,
			      kmb_isp->pad_fmt_caps[pad].range.max_height);

	dev_dbg(kmb_isp->dev, "Resolution for pad-idx %d was set to %ux%u\n",
		pad, format->width, format->height);
}

/**
 * kmb_isp_subdev_get_fmt - Get mbus format
 * @sd: pointer to V4L2 sub-device
 * @cfg: pointer to sub-device pad information
 * @format: pointer to pad-level media bus format
 *
 * VIDIOC_SUBDEV_G_FMT ioctl handler
 *
 * Return: 0 if successful
 */
static int kmb_isp_subdev_get_fmt(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_format *format)
{
	struct kmb_isp *kmb_isp = v4l2_get_subdevdata(sd);
	struct v4l2_subdev_format *pad_fmt;

	if (format->pad >= KMB_ISP_PADS_NUM)
		return -EINVAL;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		format->format =
			*v4l2_subdev_get_try_format(sd, cfg, format->pad);
		kmb_isp_align_fmt(kmb_isp, format->pad, &format->format);
		return 0;
	}

	pad_fmt = kmb_isp_get_pad_format_by_idx(kmb_isp, format->pad);
	if (!pad_fmt)
		return -EINVAL;

	kmb_isp_align_fmt(kmb_isp, format->pad, &pad_fmt->format);
	*format = *pad_fmt;

	return 0;
}

/**
 * kmb_isp_subdev_set_fmt - Set mbus format
 * @sd: pointer to V4L2 sub-device
 * @cfg: pointer to sub-device pad information
 * @format: pointer to pad-level media bus format
 *
 * VIDIOC_SUBDEV_S_FMT ioctl handler
 *
 * Return: 0 if successful
 */
static int kmb_isp_subdev_set_fmt(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_format *format)
{
	struct kmb_isp *kmb_isp = v4l2_get_subdevdata(sd);
	struct v4l2_subdev_format *pad_fmt;

	if (format->pad >= KMB_ISP_PADS_NUM)
		return -EINVAL;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		format->format =
			*v4l2_subdev_get_try_format(sd, cfg, format->pad);
		kmb_isp_align_fmt(kmb_isp, format->pad, &format->format);
		return 0;
	}

	pad_fmt = kmb_isp_get_pad_format_by_idx(kmb_isp, format->pad);
	if (!pad_fmt)
		return -EINVAL;

	kmb_isp_align_fmt(kmb_isp, format->pad, &format->format);
	*pad_fmt = *format;

	return 0;
}

/**
 * kmb_isp_subdev_enum_frame_interval - Enumerate frame intervals
 * @sd: pointer to V4L2 sub-device
 * @cfg: pointer to sub-device pad information
 * @fie: pointer to pad-level frame interval enumeration
 *
 * VIDIOC_SUBDEV_ENUM_FRAME_INTERVAL ioctl handler
 *
 * Return: 0 if successful
 */
static int
kmb_isp_subdev_enum_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_interval_enum *fie)
{
	struct kmb_isp *kmb_isp = v4l2_get_subdevdata(sd);
	struct v4l2_subdev_frame_interval_enum interval;
	struct v4l2_subdev *subdev;
	struct media_pad *remote;
	int ret;

	if (fie->pad >= KMB_ISP_PADS_NUM)
		return -EINVAL;

	remote = media_entity_remote_pad(
			&kmb_isp->pads[KMB_ISP_SINK_PAD_SENSOR]);
	if (remote == NULL)
		return -ENOENT;

	subdev = media_entity_to_v4l2_subdev(remote->entity);

	mutex_lock(&kmb_isp->lock);

	memcpy(&interval, fie, sizeof(*fie));
	interval.pad = 0;
	interval.which = V4L2_SUBDEV_FORMAT_TRY;
	interval.code = kmb_isp->pad_fmt_caps[KMB_ISP_SINK_PAD_SENSOR].code[0];
	ret = v4l2_subdev_call(subdev, pad, enum_frame_interval,
				cfg, &interval);
	if (ret < 0) {
		mutex_unlock(&kmb_isp->lock);
		return ret;
	}

	fie->interval = interval.interval;

	mutex_unlock(&kmb_isp->lock);

	return 0;
}

/**
 * kmb_isp_subdev_enum_mbus_code - Enumerate media bus code
 * @sd: pointer to V4L2 sub-device
 * @cfg: pointer to sub-device pad information
 * @code: pointer to pad-level media bus code
 *
 * VIDIOC_SUBDEV_ENUM_MBUS_CODE ioctl handler
 *
 * Return: 0 if successful
 */
static int kmb_isp_subdev_enum_mbus_code(struct v4l2_subdev *sd,
					struct v4l2_subdev_pad_config *cfg,
					struct v4l2_subdev_mbus_code_enum *code)
{
	struct kmb_isp *kmb_isp = v4l2_get_subdevdata(sd);

	if (code->index >= kmb_isp->pad_fmt_caps[code->pad].num_fmts)
		return -EINVAL;

	mutex_lock(&kmb_isp->lock);
	code->code = kmb_isp->pad_fmt_caps[code->pad].code[code->index];
	mutex_unlock(&kmb_isp->lock);

	return 0;
}

/**
 * kmb_isp_g_frame_interval - Get frame intervals
 * @sd: pointer to V4L2 sub-device
 * @interval: pointer to pad-level subdevice frame interval enumeration
 *
 * VIDIOC_SUBDEV_G_FRAME_INTERVAL ioctl handler
 *
 * Return: 0 if successful
 */
static int kmb_isp_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *interval)
{
	struct kmb_isp *kmb_isp = v4l2_get_subdevdata(sd);
	struct v4l2_subdev_frame_interval ival;
	struct v4l2_subdev *subdev;
	struct media_pad *remote;
	int ret;

	if (interval->pad >= KMB_ISP_PADS_NUM)
		return -EINVAL;

	remote = media_entity_remote_pad(
			&kmb_isp->pads[KMB_ISP_SINK_PAD_SENSOR]);
	if (remote == NULL)
		return -ENOENT;

	subdev = media_entity_to_v4l2_subdev(remote->entity);

	memcpy(&ival, interval, sizeof(*interval));
	ival.pad = 0;
	ret = v4l2_subdev_call(subdev, video, g_frame_interval, &ival);
	if (ret < 0) {
		dev_err(kmb_isp->dev, "Couldn't get sensor frame interval\n");
		return ret;
	}

	interval->interval = ival.interval;

	return 0;
}

/**
 * kmb_isp_s_frame_interval - Set frame intervals
 * @sd: pointer to V4L2 sub-device
 * @interval: pointer to pad-level subdevice frame interval enumeration
 *
 * VIDIOC_SUBDEV_S_FRAME_INTERVAL ioctl handler
 *
 * Return: 0 if successful
 */
static int kmb_isp_s_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *interval)
{
	struct kmb_isp *kmb_isp = v4l2_get_subdevdata(sd);
	struct v4l2_subdev_frame_interval ival;
	struct v4l2_subdev *subdev;
	struct media_pad *remote;
	int ret;

	if (interval->pad >= KMB_ISP_PADS_NUM)
		return -EINVAL;

	remote = media_entity_remote_pad(
			&kmb_isp->pads[KMB_ISP_SINK_PAD_SENSOR]);
	if (remote == NULL)
		return -ENOENT;

	subdev = media_entity_to_v4l2_subdev(remote->entity);

	memcpy(&ival, interval, sizeof(*interval));
	ival.pad = 0;
	ret = v4l2_subdev_call(subdev, video, s_frame_interval, &ival);
	if (ret < 0) {
		dev_err(kmb_isp->dev, "Couldn't set sensor frame interval\n");
		return ret;
	}

	interval->interval = ival.interval;

	return 0;
}

/**
 * kmb_isp_s_stream - Set video stream stop/start
 * @sd: pointer to V4L2 sub-device
 * @enable: stream state
 *
 * Return: 0 if successful
 */
static int kmb_isp_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct kmb_isp *kmb_isp = v4l2_get_subdevdata(sd);
	int ret;

	mutex_lock(&kmb_isp->lock);

	/* Don't send isp config on stream disable */
	if (enable) {
		ret = kmb_isp_open_xlink_channel(kmb_isp);
		if (ret) {
			dev_err(kmb_isp->dev,
					"Fail to open xlink channel %d\n", ret);
			goto error;
		}

		ret = kmb_isp_configure_source(kmb_isp);
		if (ret) {
			dev_err(kmb_isp->dev, "Fail to configure source %d\n",
					ret);
			goto error_close_xlink_channel;
		}

		/* Process all pending isp configurations on stream enable */
		ret = kmb_isp_process_all_configs(kmb_isp);
		if (ret) {
			dev_err(kmb_isp->dev, "Fail to configure source %d\n",
					ret);
			goto error_close_xlink_channel;
		}
		kmb_isp->isp_streaming = true;
	} else {
		/* Try top to stop the source synchronized */
		if (kmb_isp->source_streaming)
			kmb_isp_stop_source(kmb_isp, KMB_ISP_STOP_SYNC);

		kmb_isp_close_xlink_channel(kmb_isp);

		/* Force stop isp if still streaming after channel is closed */
		if (kmb_isp->source_streaming)
			kmb_isp_stop_source(kmb_isp, KMB_ISP_STOP_FORCE);

		/* Discard all unprocessed configuration */
		kmb_isp_discard_all_configs(kmb_isp);
		kmb_isp->isp_streaming = false;
		kmb_isp->sequence = 0;
	}

	mutex_unlock(&kmb_isp->lock);
	return 0;

error_close_xlink_channel:
	kmb_isp_close_xlink_channel(kmb_isp);
error:
	mutex_unlock(&kmb_isp->lock);
	return ret;
}

/**
 * kmb_isp_subdev_ioctl - Private ioctl handling
 * @sd: pointer to V4L2 sub-device
 * @cmd: ioctl command
 * @arg: pointer to ioctl arguments
 *
 * Return: 0 if successful, -ENOTTY if command is not supported
 */
static long kmb_isp_subdev_ioctl(struct v4l2_subdev *sd,
				 unsigned int cmd, void *arg)
{
	struct kmb_isp *kmb_isp = v4l2_get_subdevdata(sd);
	int ret;

	mutex_lock(&kmb_isp->lock);

	switch (cmd) {
	default:
		dev_dbg(kmb_isp->dev, "Unknown cmd 0x%x\n", cmd);
		ret = -ENOTTY;
		break;
	}

	mutex_unlock(&kmb_isp->lock);

	return ret;
}

/**
 * kmb_isp_s_power - Put device in power saving or normal mode
 * @sd: pointer to V4L2 sub-device
 * @on: power state (0 - power saving mode, 1 - normal operation mode)
 *
 * Return: 0 if successful
 */
static int kmb_isp_s_power(struct v4l2_subdev *sd, int on)
{
	struct kmb_isp *kmb_isp = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&kmb_isp->lock);

	if (on) {
		if (kmb_isp->channel_ops.accuire_id)
			kmb_isp->channel_id =
				kmb_isp->channel_ops.accuire_id(kmb_isp->priv);
		ret = kmb_isp_populate_pad_formats(kmb_isp);
	} else {
		if (kmb_isp->channel_ops.free_id && kmb_isp->channel_id)
			kmb_isp->channel_ops.free_id(kmb_isp->priv,
						     kmb_isp->channel_id);
		kmb_isp->channel_id = 0;
		kmb_isp_reset_pad_formats(kmb_isp);
	}

	mutex_unlock(&kmb_isp->lock);

	kmb_isp_limit_camera_modes_range(kmb_isp);

	dev_dbg(kmb_isp->dev, "%s: Set power_on %d %d\n", __func__, on, ret);
	return ret;
}

/**
 * kmb_isp_subscribe_event - Event subscription
 * @sd: pointer to V4L2 sub-device
 * @fh: pointer to V4L2 sub-device file handle
 * @sub: pointer to event subscription structure
 *
 * Request event subscription from control framework, driver will be warned
 * when control value changes
 *
 * Return: 0 if successful
 */
static int kmb_isp_subscribe_event(struct v4l2_subdev *sd,
			struct v4l2_fh *fh,
			struct v4l2_event_subscription *sub)
{
	struct kmb_isp *kmb_isp = v4l2_get_subdevdata(sd);
	int ret;

	switch (sub->type) {
	case V4L2_EVENT_KMB_SENSOR_SOF:
	case V4L2_EVENT_KMB_SENSOR_EOF:
	case V4L2_EVENT_KMB_ISP_SOF:
	case V4L2_EVENT_KMB_ISP_EOF:
	case V4L2_EVENT_KMB_CFG_SKIPPED:
	case V4L2_EVENT_KMB_CFG_MISSING:
	case V4L2_EVENT_KMB_BUF_MISSING:
	case V4L2_EVENT_KMB_GEN_ERROR:
		ret = v4l2_event_subscribe(
				fh, sub, KMB_ISP_EVT_Q_LEN, NULL);
		dev_dbg(kmb_isp->dev,
				"Subscribe to event %d\n", sub->type);
		break;
	default:
		dev_err(kmb_isp->dev, "Unknown event %d\n", sub->type);
		ret = -ENOTTY;
		break;
	}

	return ret;
}

/**
 * kmb_isp_unsubscribe_event - Remove event subscription
 * @sd: pointer to V4L2 sub-device
 * @fh: pointer to V4L2 sub-device file handle
 * @sub: pointer to event subscription structure
 *
 * Remove event subscription from the control framework
 *
 * Return: 0 if successful
 */
static int kmb_isp_unsubscribe_event(struct v4l2_subdev *sd,
			struct v4l2_fh *fh,
			struct v4l2_event_subscription *sub)
{
	return v4l2_event_unsubscribe(fh, sub);
}

/**
 * kmb_isp_set_transform_hub - Set transform hub mode
 * @kmb_isp: pointer to isp device
 * @mode: chosen transform hub mode
 *
 * Return: 0 if successful
 */
static int kmb_isp_set_transform_hub(struct kmb_isp *kmb_isp, int mode)
{
	pr_debug("%s: kmb set transform hub %d", __func__, mode);

	return 0;
}

/**
 * kmb_isp_set_camera_mode - Set camera mode
 * @kmb_isp: pointer to isp device
 * @mode: chosen camera mode
 *
 * Return: 0 if successful
 */
static int kmb_isp_set_camera_mode(struct kmb_isp *kmb_isp, int mode)
{
	if (kmb_isp->source_streaming)
		return -EBUSY;

	pr_debug("%s: kmb set camera mode %d", __func__, mode);
	return 0;
}

/**
 * kmb_isp_get_transform_hub - Get transform hub mode
 * @kmb_isp: pointer to isp device
 * @mode: current transform hub mode
 *
 * Return: 0 if successful
 */
static int kmb_isp_get_transform_hub(struct kmb_isp *kmb_isp, int *mode)
{
	pr_debug("%s: kmb get transform hub %d", __func__, *mode);

	return 0;
}

/**
 * kmb_isp_get_camera_mode - Get camera mode
 * @kmb_isp: pointer to isp device
 * @mode: current camera mode
 *
 * Return: 0 if successful
 */
static int kmb_isp_get_camera_mode(struct kmb_isp *kmb_isp, int *mode)
{
	pr_debug("%s: kmb get camera mode %d", __func__, *mode);

	return 0;
}

/**
 * kmb_isp_s_ctrl - Set new control value
 * @ctrl: pointer to control structure
 *
 * Return: 0 if successful
 */
static int kmb_isp_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = &(container_of(ctrl->handler, struct kmb_isp,
					ctrls.handler)->subdev);
	struct kmb_isp *kmb_isp = container_of(sd, struct kmb_isp, subdev);

	dev_dbg(kmb_isp->dev, "s_ctrl: %s, value: %d\n",
			ctrl->name, ctrl->val);

	mutex_lock(&kmb_isp->lock);

	switch (ctrl->id) {
	case V4L2_CID_KMB_CAMERA_TRANSFORM_HUB:
		kmb_isp_set_transform_hub(kmb_isp, ctrl->val);
		break;
	case V4L2_CID_KMB_CAMERA_MODE:
		kmb_isp_set_camera_mode(kmb_isp, ctrl->val);
		break;
	default:
		dev_dbg(kmb_isp->dev, "%s: KMB isp set %d\n",
				__func__, ctrl->id);
	}

	mutex_unlock(&kmb_isp->lock);
	return 0;
}

/**
 * kmb_isp_g_ctrl - Get control value
 * @ctrl: pointer to control structure
 *
 * Return: 0 if successful
 */
static int kmb_isp_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = &(container_of(ctrl->handler, struct kmb_isp,
					ctrls.handler)->subdev);
	struct kmb_isp *kmb_isp = container_of(sd, struct kmb_isp, subdev);

	mutex_lock(&kmb_isp->lock);
	switch (ctrl->id) {
	case V4L2_CID_KMB_CAMERA_TRANSFORM_HUB:
		kmb_isp_get_transform_hub(kmb_isp, &ctrl->val);
		break;
	case V4L2_CID_KMB_CAMERA_MODE:
		kmb_isp_get_camera_mode(kmb_isp, &ctrl->val);
		break;
	default:
		dev_dbg(kmb_isp->dev, "%s: KMB isp get %d\n",
				__func__, ctrl->id);
	}
	mutex_unlock(&kmb_isp->lock);

	return 0;
}

/* sub-device core operations */
static struct v4l2_subdev_core_ops kmb_isp_subdev_core_ops = {
	.ioctl = kmb_isp_subdev_ioctl,
	.s_power = kmb_isp_s_power,
	.subscribe_event = kmb_isp_subscribe_event,
	.unsubscribe_event = kmb_isp_unsubscribe_event,
};

/* sub-device video operations */
static struct v4l2_subdev_video_ops kmb_isp_subdev_video_ops = {
	.s_stream = kmb_isp_s_stream,
	.g_frame_interval = kmb_isp_g_frame_interval,
	.s_frame_interval = kmb_isp_s_frame_interval,
};

/* sub-device pad operations */
static struct v4l2_subdev_pad_ops kmb_isp_subdev_pad_ops = {
	.set_fmt = kmb_isp_subdev_set_fmt,
	.get_fmt = kmb_isp_subdev_get_fmt,
	.enum_mbus_code = kmb_isp_subdev_enum_mbus_code,
	.enum_frame_interval = kmb_isp_subdev_enum_frame_interval,
};

/* sub-device operations */
static const struct v4l2_subdev_ops kmb_isp_subdev_ops = {
	.core = &kmb_isp_subdev_core_ops,
	.video = &kmb_isp_subdev_video_ops,
	.pad = &kmb_isp_subdev_pad_ops,
};

/* V4L2 control operations */
static const struct v4l2_ctrl_ops kmb_isp_ctrl_ops = {
	.s_ctrl = kmb_isp_s_ctrl,
	.g_volatile_ctrl = kmb_isp_g_ctrl,
};

static const struct v4l2_ctrl_config transform_hub = {
	.ops = &kmb_isp_ctrl_ops,
	.id = V4L2_CID_KMB_CAMERA_TRANSFORM_HUB,
	.name = "Transform Hub",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = KMB_CAMERA_TRANSFORM_HUB_NONE,
	.max = KMB_CAMERA_TRANSFORM_HUB_EPTZ,
	.def = KMB_CAMERA_TRANSFORM_HUB_NONE,
	.step = 1,
	.menu_skip_mask = 0,
};

static const struct v4l2_ctrl_config camera_mode = {
	.ops = &kmb_isp_ctrl_ops,
	.id = V4L2_CID_KMB_CAMERA_MODE,
	.name = "Camera Mode",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = KMB_CAMERA_MODE_ULL,
	.max = KMB_CAMERA_MODE_MONO,
	.def = KMB_CAMERA_MODE_ULL,
	.step = 1,
	.menu_skip_mask = 0,
};

/**
 * kmb_isp_initialize_controls - Initialize handled isp controls
 * @kmb_isp: pointer to isp device
 *
 * Return: 0 if successful
 */
static int kmb_isp_initialize_controls(struct kmb_isp *kmb_isp)
{
	struct kmb_isp_ctrls *ctrls = &kmb_isp->ctrls;
	int ret;

	ret = v4l2_ctrl_handler_init(&ctrls->handler, 5);
	if (ret < 0) {
		dev_err(kmb_isp->dev, "%s: Control handler init failed %d\n",
				__func__, ret);
		return ret;
	}

	ctrls->transform_hub = v4l2_ctrl_new_custom(&ctrls->handler,
			&transform_hub, NULL);
	if (!ctrls->transform_hub) {
		dev_err(kmb_isp->dev, "%s: Init transform hub control failed\n",
				__func__);
		return -EINVAL;
	}
	ctrls->camera_mode = v4l2_ctrl_new_custom(&ctrls->handler,
			&camera_mode, NULL);
	if (!ctrls->camera_mode) {
		dev_err(kmb_isp->dev, "%s: Init camera mode control failed\n",
				__func__);
		return -EINVAL;
	}

	if (ctrls->handler.error) {
		ret = ctrls->handler.error;
		v4l2_ctrl_handler_free(&ctrls->handler);
		dev_err(kmb_isp->dev, "%s: Error %d\n", __func__, ret);
		return ret;
	}

	kmb_isp->subdev.ctrl_handler = &ctrls->handler;
	return 0;
}

/**
 * kmb_isp_init - Initialize Kmb isp subdevice
 * @kmb_isp: Pointer to kmb isp device
 * @dev: Pointer to camera device for which isp will be associated with
 */
void kmb_isp_init(struct kmb_isp *kmb_isp, struct device *dev)
{
	mutex_init(&kmb_isp->lock);
	mutex_init(&kmb_isp->cfg_q_lock);

	v4l2_subdev_init(&kmb_isp->subdev, &kmb_isp_subdev_ops);
	v4l2_set_subdevdata(&kmb_isp->subdev, kmb_isp);

	INIT_LIST_HEAD(&kmb_isp->isp_cfgs_queue);
	kmb_isp->isp_streaming = false;

	kmb_isp->dev = dev;
	kmb_isp->channel_id = 0;
}

/**
 * kmb_isp_register_entities - Register entities
 * @kmb_isp: pointer to kmb isp device
 * @v4l2_dev: pointer to V4L2 device drivers
 *
 * Return: 0 if successful
 */
int kmb_isp_register_entities(struct kmb_isp *kmb_isp,
			      struct v4l2_device *v4l2_dev)
{
	struct media_pad *pads = kmb_isp->pads;
	struct device *dev = kmb_isp->dev;
	int i;
	int ret;

	kmb_isp->subdev.internal_ops = &kmb_isp_internal_ops;
	kmb_isp->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			V4L2_SUBDEV_FL_HAS_EVENTS;
	kmb_isp->subdev.entity.function = MEDIA_INTF_T_V4L_SUBDEV;
	snprintf(kmb_isp->subdev.name,
		ARRAY_SIZE(kmb_isp->subdev.name), KMB_ISP_DRV_NAME);

	pads[KMB_ISP_SINK_PAD_RAW].flags = MEDIA_PAD_FL_SINK;
	pads[KMB_ISP_SINK_PAD_SENSOR].flags = MEDIA_PAD_FL_SINK;
	pads[KMB_ISP_SINK_PAD_CFG].flags = MEDIA_PAD_FL_SINK;
	pads[KMB_ISP_SRC_PAD_RAW].flags = MEDIA_PAD_FL_SOURCE;

	for (i = 0; i < KMB_ISP_SRC_PADS_NUM; i++)
		pads[KMB_ISP_SRC_PAD_VID_BASE + i].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&kmb_isp->subdev.entity,
				     KMB_ISP_PADS_NUM, pads);
	if (ret < 0) {
		dev_err(dev, "Fail to init media entity");
		return ret;
	}

	ret = kmb_isp_initialize_controls(kmb_isp);
	if (ret < 0) {
		dev_err(dev, "Fail to initialize isp controls");
		return ret;
	}

	ret = v4l2_device_register_subdev(v4l2_dev, &kmb_isp->subdev);
	if (ret < 0) {
		dev_err(dev, "Fail to register media entity");
		return ret;
	}

	kmb_isp->cma_vaddr = NULL;
	kmb_isp->cma_phy_addr = 0;

	ret = kmb_isp_alloc_pipe_cma_area(kmb_isp);
	if (ret < 0)
		goto error_unregister_subdev;

	/* kmb config node - isp input */
	kmb_isp->config.pipe = kmb_isp->pipe;
	kmb_isp->config.priv = (void *)kmb_isp;
	kmb_isp->config.dma_dev = kmb_isp->dev;
	kmb_isp->config.queue_ops = isp_cfg_queue_ops;

	ret = kmb_isp_config_init(&kmb_isp->config);
	if (ret < 0)
		goto error_cma_free;

	ret = kmb_isp_config_register(&kmb_isp->config, v4l2_dev);
	if (ret < 0)
		goto error_deinit_config;

	ret = media_create_pad_link(
			&kmb_isp->config.video_out->entity, 0,
			&kmb_isp->subdev.entity, KMB_ISP_SINK_PAD_CFG,
			MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
	if (ret < 0) {
		dev_err(kmb_isp->dev, "Fail to link %s->%s entities\n",
			kmb_isp->config.video_out->entity.name,
			kmb_isp->subdev.entity.name);
		goto error_unregister_config;
	}

	return 0;

error_unregister_config:
	kmb_isp_config_unregister(&kmb_isp->config);
error_deinit_config:
	kmb_isp_config_deinit(&kmb_isp->config);
error_cma_free:
	kmb_isp_free_pipe_cma_area(kmb_isp);
error_unregister_subdev:
	v4l2_device_unregister_subdev(&kmb_isp->subdev);
	return ret;
}

/**
 * kmb_isp_unregister_entities - Unregister this media's entities
 * @kmb_isp: pointer to kmb isp device
 */
void kmb_isp_unregister_entities(struct kmb_isp *kmb_isp)
{
	kmb_isp_config_unregister(&kmb_isp->config);
	mutex_destroy(&kmb_isp->cfg_q_lock);
	v4l2_ctrl_handler_free(&kmb_isp->ctrls.handler);
	kmb_isp_free_pipe_cma_area(kmb_isp);
	mutex_destroy(&kmb_isp->lock);
	v4l2_device_unregister_subdev(&kmb_isp->subdev);
}

/**
 * kmb_isp_get_channel_id - Get ISP xLink channel id
 * @kmb_isp: pointer to kmb isp device
 *
 * Return: xLink channel id
 */
u32 kmb_isp_get_channel_id(struct kmb_isp *kmb_isp)
{
	return kmb_isp->channel_id;
}

/**
 * kmb_isp_get_lrt_transform_hub - Get LRT Transform Hub
 * @kmb_isp: pointer to kmb isp device
 *
 * Return: LRT transform hub
 */
u32 kmb_isp_get_lrt_transform_hub(struct kmb_isp *kmb_isp)
{
	int transform_hub = v4l2_ctrl_g_ctrl(kmb_isp->ctrls.transform_hub);
	int i;

	for (i = 0; i < ARRAY_SIZE(rt_isp_thb); i++) {
		if (transform_hub == rt_isp_thb[i][1])
			return rt_isp_thb[i][0];
	}

	return rt_isp_thb[0][0];
}

/**
 * kmb_isp_get_lrt_camera_mode - Get LRT Camera Mode
 * @kmb_isp: pointer to kmb isp device
 *
 * Return: LRT camera mode
 */
u32 kmb_isp_get_lrt_camera_mode(struct kmb_isp *kmb_isp)
{
	int camera_mode = v4l2_ctrl_g_ctrl(kmb_isp->ctrls.camera_mode);
	int i;

	for (i = 0; i < ARRAY_SIZE(rt_isp_cam_mode); i++) {
		if (camera_mode == rt_isp_cam_mode[i][1])
			return rt_isp_cam_mode[i][0];
	}

	return rt_isp_cam_mode[0][0];
}

/**
 * kmb_isp_set_pad_fmt_range - Set pad format range
 * @kmb_isp: pointer to kmb isp device
 * @pad: Isp subdevice pad index
 * @range: Pointer to resolution range to be set
 *
 * Return: 0 if successful
 */
int kmb_isp_set_pad_fmt_range(struct kmb_isp *kmb_isp, u32 pad,
			      struct kmb_isp_res_range *range)
{
	if (pad >= KMB_ISP_PADS_NUM)
		return -EINVAL;

	mutex_lock(&kmb_isp->lock);
	kmb_isp->pad_fmt_caps[pad].range = *range;

	kmb_isp->pad_format[pad].format.width = range->min_width;
	kmb_isp->pad_format[pad].format.height = range->min_height;
	mutex_unlock(&kmb_isp->lock);

	dev_dbg(kmb_isp->dev, "Format range pad %d min %dx%d max %dx%d\n",
		pad, range->min_width, range->min_height,
		range->max_width, range->max_height);

	return 0;
}

/**
 * kmb_isp_set_src_config - Set source configuration information
 * @kmb_isp: pointer to kmb isp device
 * @pad: Isp subdevice pad index
 * @src_config: pointer to source configuration
 *
 * Return: 0 if successful
 */
int kmb_isp_set_src_config(struct kmb_isp *kmb_isp, u32 pad,
			   struct kmb_isp_src_config *src_config)
{
	if (pad >= KMB_ISP_SINK_PADS_NUM)
		return -EINVAL;

	mutex_lock(&kmb_isp->lock);
	kmb_isp->src_config[pad] = *src_config;
	mutex_unlock(&kmb_isp->lock);

	return 0;
}
