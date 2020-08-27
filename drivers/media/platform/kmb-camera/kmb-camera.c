// SPDX-License-Identifier: GPL-2.0-only
/*
 * Keem Bay Camera Driver.
 *
 * Copyright (C) 2018-2020 Intel Corporation
 */
#include <linux/of.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <media/v4l2-fwnode.h>

#include "kmb-camera.h"
#include "kmb-lrt-cmd.h"

#define KMB_CAM_XLINK_CHAN_ID_BASE	30
#define RT_CONTROL_CH_ID		27
#define KMB_RT_CTRL_CH_DATA_SIZE	1024
#define KMB_RT_CTRL_CH_TIMEOUT_MS	5000

#define to_kmb_camera_pipe(ptr_entity)					\
({									\
	struct video_device *vdev =					\
		container_of(entity, struct video_device, entity);	\
	struct kmb_video *kmb_vid =					\
		(struct kmb_video *)video_get_drvdata(vdev);		\
	struct kmb_cam_pipeline *pipe = kmb_vid->priv;			\
	pipe;								\
})

/**
 * kmb_cam_accuire_channel_id - Allocate channel id
 * @priv: callback private data
 *
 * Return: channel id
 */
static int kmb_cam_accuire_channel_id(void *priv)
{
	struct kmb_cam_pipeline *pipeline = (struct kmb_cam_pipeline *)priv;
	int id;

	id = ida_alloc_range(pipeline->rt_pipe.id_alloc,
		KMB_CAM_XLINK_CHAN_ID_BASE, U16_MAX, GFP_KERNEL);

	return id;
}

/**
 * kmb_cam_free_channel_id - Free channel id
 * @priv: callback private data
 * @id: channel id
 *
 * Return: none
 */
static void kmb_cam_free_channel_id(void *priv, unsigned int id)
{
	struct kmb_cam_pipeline *pipeline = (struct kmb_cam_pipeline *)priv;

	ida_free(pipeline->rt_pipe.id_alloc, id);
}

/* ISP channel ID operations */
struct kmb_isp_channel_ops kmb_isp_channel_ops = {
	.accuire_id = kmb_cam_accuire_channel_id,
	.free_id = kmb_cam_free_channel_id,
};

/* Video device channel ID operations */
struct kmb_video_channel_ops kmb_vid_channel_ops = {
	.accuire_id = kmb_cam_accuire_channel_id,
	.free_id = kmb_cam_free_channel_id,
};

/**
 * kmb_cam_print_pipeline_config - Print pipeline configuration
 * @cfg: LRT pipeline configuration
 * @ctrl: LRT Control type
 * @result: Flag indicating result configuration
 *
 * Return: none
 */
static void kmb_cam_print_pipeline_config(struct pipeConfigEvS *cfg,
					  u32 ctrl, bool result)
{
	int i;

	switch (ctrl) {
	case IC_EVENT_TYPE_CONFIG_ISP_PIPE:
		pr_debug("IC_EVENT_TYPE_CONFIG_ISP_PIPE %s",
			 result ? "result" : "");
		break;
	case IC_EVENT_TYPE_BUILD_ISP_PIPE:
		pr_debug("IC_EVENT_TYPE_BUILD_ISP_PIPE %s",
			 result ? "result" : "result");
		break;
	case IC_EVENT_TYPE_DELETE_ISP_PIPE:
		pr_debug("IC_EVENT_TYPE_DELETE_ISP_PIPE %s",
			 result ? "result" : "");
		break;
	default:
		pr_err("Unknown control %d\n", ctrl);
		break;
	}

	pr_debug("\tpipeID %u\n", cfg->pipeID);
	pr_debug("\tpipeType %u\n", cfg->pipeType);
	pr_debug("\tsrcType %u\n", cfg->srcType);
	pr_debug("\tpipeTransHub %u\n", cfg->pipeTransHub);
	pr_debug("\tinIspRes %ux%u\n", cfg->inIspRes.w, cfg->inIspRes.h);

	for (i = 0; i < PIPE_OUTPUT_ID_MAX; i++) {
		pr_debug("\tOUTPUT ID: %d\n", i);
		pr_debug("\t\toutMinRes %ux%u\n",
			cfg->outMinRes[i].w, cfg->outMinRes[i].h);
		pr_debug("\t\toutMaxRes %ux%u\n",
			cfg->outMaxRes[i].w, cfg->outMaxRes[i].h);
	}

	for (i = 0; i < PIPE_OUTPUT_ID_MAX; i++) {
		pr_debug("\tpipeXlinkChann: %d\n", i);
		pr_debug("\t\tchID: %u %ux%u\n",
			cfg->pipeXlinkChann[i].chID,
			cfg->pipeXlinkChann[i].frmRes.w,
			cfg->pipeXlinkChann[i].frmRes.h);
	}

	pr_debug("\tkeepAspectRatio %u\n", cfg->keepAspectRatio);
	pr_debug("\tinDataWidth %u\n", cfg->inDataWidth);
	pr_debug("\tinDataPacked %u\n", cfg->inDataPacked);
	pr_debug("\toutDataWidth %u\n", cfg->outDataWidth);
}

/**
 * kmb_cam_config_isp_src - Fill endpoint configuration from sensor fwnode
 * @kmb_cam: pointer to kmb camera device
 * @pipeline: pointer to kmb_cam_pipeline
 *
 * Return: none
 */
static void kmb_cam_config_isp_src(struct kmb_camera *kmb_cam,
				   struct kmb_cam_pipeline *pipeline)
{
	struct kmb_cam_endpoint_data *ep = NULL;
	struct kmb_isp_src_config src_config;
	int i;

	for (i = 0; i < ARRAY_SIZE(kmb_cam->ep_data); i++) {
		if (kmb_cam->ep_data[i].remote == pipeline->sensor->fwnode) {
			ep = &kmb_cam->ep_data[i];
			break;
		}
	}
	if (!ep)
		return;

	memset(&src_config, 0, sizeof(src_config));
	src_config.num_phy_lanes = ep->ep_data.bus.mipi_csi2.num_data_lanes;
	src_config.controller_num = ep->controller_num;
	switch (ep->ep_data.bus_type) {
	case V4L2_MBUS_CSI2_DPHY:
		src_config.type = KMB_ISP_CSI2;
		break;
	default:
		src_config.type = KMB_ISP_MEMORY;
		break;
	}

	kmb_isp_set_src_config(&pipeline->isp,
			KMB_ISP_SINK_PAD_SENSOR, &src_config);
}

/**
 * kmb_cam_update_isp_pads_ranges - Update isp pads format ranges
 * @cfg: LRT pipeline configuration
 * @kmb_isp: pointer to KMB ISP sub-device
 *
 * Return: none
 */
static void kmb_cam_update_isp_pads_ranges(struct pipeConfigEvS *cfg,
					   struct kmb_isp *kmb_isp)
{
	struct kmb_isp_res_range range;
	u32 isp_pad = KMB_ISP_SRC_PAD_VID_BASE;
	int ret;
	int i;

	for (i = PIPE_OUTPUT_ID_0; i < PIPE_OUTPUT_ID_MAX; i++) {
		range.min_width = cfg->outMinRes[i].w;
		range.min_height = cfg->outMinRes[i].h;
		range.max_width = cfg->outMaxRes[i].w;
		range.max_height = cfg->outMaxRes[i].h;
		ret = kmb_isp_set_pad_fmt_range(kmb_isp, isp_pad, &range);
		if (ret < 0)
			break;

		isp_pad++;
	}

	/* Update raw format range to be the same as sensor res */
	range.min_width = cfg->inIspRes.w;
	range.min_height = cfg->inIspRes.h;
	range.max_width = cfg->inIspRes.w;
	range.max_height = cfg->inIspRes.h;
	kmb_isp_set_pad_fmt_range(kmb_isp, KMB_ISP_SRC_PAD_RAW, &range);
	kmb_isp_set_pad_fmt_range(kmb_isp, KMB_ISP_SINK_PAD_RAW, &range);
	kmb_isp_set_pad_fmt_range(kmb_isp, KMB_ISP_SINK_PAD_SENSOR, &range);
}

/**
 * kmb_cam_init_pipe - Initialize kmb camera pipeline instance
 * @kmb_cam: pointer to KMB Camera device
 * @pipeline: pointer to kmb_cam_pipeline
 *
 * Return: none
 */
static void kmb_cam_init_pipe(struct kmb_camera *kmb_cam,
			      struct kmb_cam_pipeline *pipeline)
{
	pipeline->config_refcnt = 0;
	pipeline->built_refcnt = 0;
	pipeline->streaming_refcnt = 0;
	mutex_init(&pipeline->lock);

	memset(&pipeline->rt_pipe.pipe_cfg, 0,
		sizeof(pipeline->rt_pipe.pipe_cfg));

	pipeline->rt_pipe.ctrl_chan = &kmb_cam->ctrl_channel;
	pipeline->rt_pipe.id_alloc = &kmb_cam->id_alloc;
}

/**
 * kmb_cam_get_subdev_channel_ids - Get all needed channel IDs
 * @pipeline: pointer to kmb_cam_pipeline
 *
 * Return: none
 */
static void kmb_cam_get_subdev_channel_ids(struct kmb_cam_pipeline *pipeline)
{
	int i;

	pipeline->rt_pipe.raw_channel_id =
			kmb_video_get_channel_id(&pipeline->raw_input);
	if (!pipeline->rt_pipe.raw_channel_id)
		pipeline->rt_pipe.raw_channel_id =
			kmb_video_get_channel_id(&pipeline->raw_output);

	pipeline->rt_pipe.isp_channel_id =
			kmb_isp_get_channel_id(&pipeline->isp);

	for (i = 0; i < KMB_MAX_OUTPUT_COUNT; i++)
		pipeline->rt_pipe.video_channel_id[i] =
			kmb_video_get_channel_id(&pipeline->output[i]);
}

/**
 * kmb_cam_init_control_channel - Initialize camera xlink control channel
 * @kmb_cam: pointer to KMB Camera device
 *
 * Return: 0 if successful
 */
static int kmb_cam_init_control_channel(struct kmb_camera *kmb_cam)
{
	struct kmb_cam_ctrl_channel *c_chan = &kmb_cam->ctrl_channel;

	c_chan->cma_vaddr = dma_alloc_coherent(kmb_cam->dev,
			sizeof(struct pipeConfigEvS),
			(dma_addr_t *)&c_chan->cma_phy_addr, 0);
	if (!c_chan->cma_vaddr)
		return -ENOMEM;

	mutex_init(&c_chan->lock);
	c_chan->refcnt = 0;
	c_chan->ipc_dev_handler = &kmb_cam->ipc_dev_handler;

	dev_dbg(kmb_cam->dev, "CTRL CHANNEL CMA: phy 0x%x vaddr %p\n",
		c_chan->cma_phy_addr, c_chan->cma_vaddr);

	return 0;
}

/**
 * kmb_cam_destroy_control_channel - Destroy camera xlink control channel
 * @kmb_cam: pointer to KMB Camera device
 */
static void kmb_cam_destroy_control_channel(struct kmb_camera *kmb_cam)
{
	struct kmb_cam_ctrl_channel *c_chan = &kmb_cam->ctrl_channel;

	dma_free_coherent(kmb_cam->dev, sizeof(struct pipeConfigEvS),
			  c_chan->cma_vaddr, c_chan->cma_phy_addr);
	mutex_destroy(&c_chan->lock);
}

/**
 * kmb_cam_request_control_channel - Request camera xlink control channel
 * @c_chan: Camera control channel
 *
 * Return: 0 if successful
 */
static int kmb_cam_request_control_channel(struct kmb_cam_ctrl_channel *c_chan)
{
	int ret = 0;

	mutex_lock(&c_chan->lock);

	if (c_chan->refcnt == 0) {
		/* Open control channel */
		ret = xlink_open_channel(c_chan->ipc_dev_handler,
					RT_CONTROL_CH_ID,
					RXB_TXB, KMB_RT_CTRL_CH_DATA_SIZE,
					KMB_RT_CTRL_CH_TIMEOUT_MS);
		if (ret)
			goto done;

	}
	c_chan->refcnt++;

done:
	mutex_unlock(&c_chan->lock);

	return ret;
}

/**
 * kmb_cam_release_control_channel - Release camera xlink control channel
 * @c_chan: Camera control channel
 *
 * Return: 0 if successful
 */
static int kmb_cam_release_control_channel(struct kmb_cam_ctrl_channel *c_chan)
{
	int ret = 0;

	mutex_lock(&c_chan->lock);

	if (WARN_ON(c_chan->refcnt == 0))
		goto done;

	if (c_chan->refcnt == 1) {
		/* Close control channel */
		ret = xlink_close_channel(c_chan->ipc_dev_handler,
					  RT_CONTROL_CH_ID);
		if (ret)
			pr_err("Failed to close ctrl chan: %d\n", ret);

	}
	c_chan->refcnt--;
done:
	mutex_unlock(&c_chan->lock);
	return ret;
}

/**
 * kmb_cam_write_control_message - Write camera control message
 * @c_chan: Camera control channel
 * @cfg_pipe_evt: Pointer to pipe configuration
 * @ctrl_type: Control message type
 *
 * Return: 0 if successful
 */
static int kmb_cam_write_control_message(struct kmb_cam_ctrl_channel *c_chan,
					 struct pipeConfigEvS *cfg_pipe_evt,
					 u32 ctrl_type)
{
	struct pipeConfigEvS *cma_cfg_pipe_evt;
	size_t init_evt_size = sizeof(struct icEvS);
	struct icEvS init_evt;
	u32 expected_result = IC_EVENT_TYPE_SUCCESFULL;
	int ret = -EINVAL;

	mutex_lock(&c_chan->lock);

	if (WARN_ON(c_chan->refcnt == 0))
		goto exit_unlock;

	kmb_cam_print_pipeline_config(cfg_pipe_evt, ctrl_type, false);

	cma_cfg_pipe_evt = c_chan->cma_vaddr;

	/* Copy messages to cma memory */
	*cma_cfg_pipe_evt = *cfg_pipe_evt;

	memset(&init_evt, 0, sizeof(init_evt));
	init_evt.ctrl = ctrl_type;
	init_evt.EvInfo.instId = cma_cfg_pipe_evt->pipeID;
	init_evt.EvInfo.userDataBaseAddr01 = c_chan->cma_phy_addr;

	ret = xlink_write_volatile(c_chan->ipc_dev_handler,
				   RT_CONTROL_CH_ID,
				   (u8 *)&init_evt,
				   init_evt_size);
	if (ret) {
		pr_err("Error ret %d ctrl %d\n", ret, ctrl_type);
		goto exit_unlock;
	}

	/*
	 * For some reason lrt is returning config pipe as result for
	 * config pipe control. This is temporary workaround to avoid fail.
	 */
	if (ctrl_type == IC_EVENT_TYPE_CONFIG_ISP_PIPE)
		expected_result = IC_EVENT_TYPE_CONFIG_ISP_PIPE;

	ret = xlink_read_data_to_buffer(c_chan->ipc_dev_handler,
					RT_CONTROL_CH_ID,
					(u8 *const)&init_evt,
					(u32 *)&init_evt_size);
	if (ret != X_LINK_SUCCESS || init_evt.ctrl != expected_result) {
		pr_err("Error ret %d ctrl %d  %d\n", ret,
			ctrl_type, init_evt.ctrl);
		ret = ret ? ret : -ENODEV;
		goto exit_unlock;
	}

	/* Some of the settings are modified copy them back */
	*cfg_pipe_evt = *cma_cfg_pipe_evt;

	kmb_cam_print_pipeline_config(cfg_pipe_evt, ctrl_type, true);

exit_unlock:
	mutex_unlock(&c_chan->lock);
	return ret;
}

/**
 * kmb_cam_config_lrt_pipe - Negotiate LRT pipeline configuration
 * @pipeline: pointer to pipeline
 * @input_fmt: pointer to pad-level media bus format
 *
 * Return: 0 if successful
 */
static int kmb_cam_config_lrt_pipe(struct kmb_cam_pipeline *pipeline,
				   struct v4l2_subdev_format *input_fmt)
{
	const struct kmb_isp_format *fmt;
	struct pipeConfigEvS *cfg_pipe_evt = &pipeline->rt_pipe.pipe_cfg;
	int ret;

	cfg_pipe_evt->pipeID = pipeline->rt_pipe.pipe_instance_id;
	cfg_pipe_evt->pipeType =
		kmb_isp_get_lrt_camera_mode(&pipeline->isp);
	cfg_pipe_evt->pipeTransHub =
		kmb_isp_get_lrt_transform_hub(&pipeline->isp);

	cfg_pipe_evt->inIspRes.w = input_fmt->format.width;
	cfg_pipe_evt->inIspRes.h = input_fmt->format.height;

	fmt = kmb_isp_get_fmt_by_code(input_fmt->format.code);
	if (!fmt)
		return -ERANGE;

	cfg_pipe_evt->inDataWidth = fmt->bpp;

	fmt = kmb_isp_get_fmt_by_code(
		pipeline->isp.pad_fmt_caps[KMB_ISP_SRC_PAD_VID_BASE].code[0]);
	if (!fmt)
		return -ERANGE;

	cfg_pipe_evt->outDataWidth = fmt->bpp;

	ret = kmb_cam_write_control_message(pipeline->rt_pipe.ctrl_chan,
		cfg_pipe_evt, IC_EVENT_TYPE_CONFIG_ISP_PIPE);

	return ret;
}

/**
 * kmb_cam_build_pipeline - Build LRT pipeline
 * @entity: pointer to media entity
 *
 * Return: 0 if successful
 */
static int kmb_cam_build_pipeline(struct media_entity *entity)
{
	struct kmb_cam_pipeline *pipeline = to_kmb_camera_pipe(entity);
	struct pipeConfigEvS *cfg_pipe_evt = &pipeline->rt_pipe.pipe_cfg;
	struct v4l2_subdev_format pad_fmt;
	u32 isp_pad, cid_idx;
	int i;
	int ret;

	mutex_lock(&pipeline->lock);
	if (pipeline->built_refcnt > 0) {
		pipeline->built_refcnt++;
		mutex_unlock(&pipeline->lock);
		return 0;
	}

	kmb_cam_get_subdev_channel_ids(pipeline);

	if (!pipeline->rt_pipe.raw_channel_id) {
		pr_debug("no raw channel, use SRC_TYPE_ALLOC_VPU_DATA_MIPI");
		cfg_pipe_evt->srcType = SRC_TYPE_ALLOC_VPU_DATA_MIPI;
		cfg_pipe_evt->inDataPacked = true;
	} else if (pipeline->raw_input.channel_id ==
		   pipeline->rt_pipe.raw_channel_id) {
		pr_debug("raw input, use SRC_TYPE_ALLOC_ARM_DATA_ARM");
		cfg_pipe_evt->srcType = SRC_TYPE_ALLOC_ARM_DATA_ARM;
		cfg_pipe_evt->inDataPacked = pipeline->raw_input.packed_fmt;
	} else if (pipeline->raw_output.channel_id ==
		   pipeline->rt_pipe.raw_channel_id) {
		pr_debug("raw output, use SRC_TYPE_ALLOC_ARM_DATA_MIPI");
		cfg_pipe_evt->srcType = SRC_TYPE_ALLOC_ARM_DATA_MIPI;
		cfg_pipe_evt->inDataPacked = pipeline->raw_output.packed_fmt;
	}

	cfg_pipe_evt->pipeXlinkChann[PIPE_OUTPUT_ID_RAW].chID =
			pipeline->rt_pipe.raw_channel_id;
	cfg_pipe_evt->pipeXlinkChann[PIPE_OUTPUT_ID_RAW].frmRes.w =
			cfg_pipe_evt->inIspRes.w;
	cfg_pipe_evt->pipeXlinkChann[PIPE_OUTPUT_ID_RAW].frmRes.h =
			cfg_pipe_evt->inIspRes.h;

	cfg_pipe_evt->pipeXlinkChann[PIPE_OUTPUT_ID_ISP_CTRL].chID =
			pipeline->rt_pipe.isp_channel_id;
	cfg_pipe_evt->pipeXlinkChann[PIPE_OUTPUT_ID_ISP_CTRL].frmRes.w =
			cfg_pipe_evt->inIspRes.w;
	cfg_pipe_evt->pipeXlinkChann[PIPE_OUTPUT_ID_ISP_CTRL].frmRes.h =
			cfg_pipe_evt->inIspRes.h;

	cid_idx = 0;
	isp_pad = KMB_ISP_SRC_PAD_VID_BASE;
	memset(&pad_fmt, 0, sizeof(pad_fmt));
	for (i = PIPE_OUTPUT_ID_0; i < PIPE_OUTPUT_ID_MAX; i++) {
		if (cid_idx >= KMB_MAX_OUTPUT_COUNT)
			break;

		pad_fmt.pad = isp_pad++;
		pad_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
		ret = v4l2_subdev_call(&pipeline->isp.subdev,
			pad, get_fmt, NULL, &pad_fmt);
		if (ret < 0)
			break;

		cfg_pipe_evt->pipeXlinkChann[i].frmRes.w =
			pad_fmt.format.width;
		cfg_pipe_evt->pipeXlinkChann[i].frmRes.h =
			pad_fmt.format.height;

		cfg_pipe_evt->pipeXlinkChann[i].chID =
			pipeline->rt_pipe.video_channel_id[cid_idx++];
	}

	ret = kmb_cam_write_control_message(pipeline->rt_pipe.ctrl_chan,
		cfg_pipe_evt, IC_EVENT_TYPE_BUILD_ISP_PIPE);
	if (ret < 0)
		goto error_unlock;

	pipeline->built_refcnt++;

	mutex_unlock(&pipeline->lock);
	return 0;

error_unlock:
	mutex_unlock(&pipeline->lock);
	return ret;
}

/**
 * kmb_cam_teardown_pipeline - Teardown LRT pipeline
 * @entity: pointer to media entity
 *
 * Return: none
 */
static void kmb_cam_teardown_pipeline(struct media_entity *entity)
{
	struct kmb_cam_pipeline *pipeline = to_kmb_camera_pipe(entity);

	mutex_lock(&pipeline->lock);

	if (WARN_ON(pipeline->built_refcnt == 0))
		goto exit_unlock;

	if (pipeline->built_refcnt > 1) {
		pipeline->built_refcnt--;
		goto exit_unlock;
	}

	kmb_cam_write_control_message(pipeline->rt_pipe.ctrl_chan,
		&pipeline->rt_pipe.pipe_cfg, IC_EVENT_TYPE_DELETE_ISP_PIPE);

	pipeline->built_refcnt--;

exit_unlock:
	mutex_unlock(&pipeline->lock);
}

/**
 * kmb_cam_pipeline_request_config - Request LRT pipeline configuration
 * @entity: pointer to media entity
 * @fmt: pointer to media bus format
 *
 * Return: 0 if successful
 */
static int kmb_cam_pipeline_request_config(struct media_entity *entity,
					   struct v4l2_mbus_framefmt *fmt)
{
	struct v4l2_subdev_format input_fmt;
	struct kmb_cam_pipeline *pipeline = to_kmb_camera_pipe(entity);
	struct media_pad *remote;
	u32 camera_mode;
	int ret;

	mutex_lock(&pipeline->lock);

	memset(&input_fmt, 0x00, sizeof(input_fmt));

	camera_mode = kmb_isp_get_lrt_camera_mode(&pipeline->isp);
	switch (camera_mode) {
	case PIPE_TYPE_ISP_ISP_2DOL:
		fmt->reserved[0] = 2;
		break;
	case PIPE_TYPE_ISP_ISP_3DOL:
		fmt->reserved[0] = 3;
		break;
	default:
		break;
	}

	pr_debug("%s: camera mode %d reserved %d", __func__,
		camera_mode, fmt->reserved[0]);

	if (pipeline->config_refcnt == 0) {

		input_fmt.pad = 0;
		input_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
		input_fmt.format = *fmt;
		ret = v4l2_subdev_call(pipeline->sensor,
				pad, set_fmt, NULL, &input_fmt);
		if (ret < 0) {
			pr_err("Couldn't set sensor source pad fmt %d\n", ret);
			goto exit;
		}

		pr_debug("Sensor output format is set to wxh %dx%d cspc %d\n",
			input_fmt.format.width, input_fmt.format.height,
			input_fmt.format.code);

		input_fmt.pad = KMB_ISP_SINK_PAD_SENSOR;
		input_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
		ret = v4l2_subdev_call(&pipeline->isp.subdev,
				pad, set_fmt, NULL, &input_fmt);
		if (ret < 0) {
			pr_err("Couldn't set isp sens sink pad fmt %d\n", ret);
			goto exit;
		}

		ret = kmb_cam_config_lrt_pipe(pipeline, &input_fmt);
		if (ret < 0)
			goto exit;

		/* Update isp pads when pipeline is configured */
		kmb_cam_update_isp_pads_ranges(&pipeline->rt_pipe.pipe_cfg,
					       &pipeline->isp);
	}

	remote = media_entity_remote_pad(entity->pads);
	input_fmt.pad = remote->index;
	input_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	input_fmt.format = *fmt;
	ret = v4l2_subdev_call(&pipeline->isp.subdev,
			pad, set_fmt, NULL, &input_fmt);
	if (ret < 0) {
		pr_err("Couldn't get isp output source pad fmt %d\n", ret);
		goto exit;
	}

	*fmt = input_fmt.format;

	pipeline->config_refcnt++;
exit:
	mutex_unlock(&pipeline->lock);
	return ret;
}

/**
 * kmb_cam_pipeline_release_config - Release LRT pipeline configuration
 * @entity: pointer to media entity
 *
 * Return: 0 if successful
 */
static int kmb_cam_pipeline_release_config(struct media_entity *entity)
{
	struct kmb_cam_pipeline *pipeline = to_kmb_camera_pipe(entity);

	mutex_lock(&pipeline->lock);
	if (!WARN_ON(pipeline->config_refcnt == 0))
		pipeline->config_refcnt--;
	mutex_unlock(&pipeline->lock);
	return 0;
}

/**
 * kmb_cam_pipeline_s_stream - LRT pipeline set stream
 * @entity: pointer to media entity
 * @enable: state to set to the stream
 *
 * Return: 0 if successful
 */
static int kmb_cam_pipeline_s_stream(struct media_entity *entity, int enable)
{
	struct kmb_cam_pipeline *pipeline = to_kmb_camera_pipe(entity);
	int ret;

	pr_debug("%s Pipeline enable streaming %d\n", __func__, enable);

	mutex_lock(&pipeline->lock);

	if (enable) {
		if (pipeline->streaming_refcnt > 0) {
			pipeline->streaming_refcnt++;
			goto success;
		}

		ret = v4l2_subdev_call(pipeline->sensor,
				       video, s_stream, enable);
		if (ret < 0 && ret != -ENOIOCTLCMD) {
			pr_err("Cannot s_stream %d to sensor\n", enable);
			goto error;
		}

		ret = v4l2_subdev_call(&pipeline->isp.subdev,
				       video, s_stream, enable);
		if (ret < 0 && ret != -ENOIOCTLCMD) {
			pr_err("Cannot s_stream %d to ISP\n", enable);
			goto error_sensor_stream_disable;
		}

		pipeline->streaming_refcnt++;
	} else {
		BUG_ON(!pipeline->streaming_refcnt);

		if (pipeline->streaming_refcnt > 1) {
			pipeline->streaming_refcnt--;
			goto success;
		}

		ret = v4l2_subdev_call(&pipeline->isp.subdev,
				       video, s_stream, enable);
		if (ret < 0 && ret != -ENOIOCTLCMD) {
			pr_err("Cannot s_stream %d to ISP\n", enable);
			goto error;
		}

		ret = v4l2_subdev_call(pipeline->sensor,
				       video, s_stream, enable);
		if (ret < 0 && ret != -ENOIOCTLCMD) {
			pr_err("Cannot s_stream %d to sensor\n", enable);
			goto error;
		}

		pipeline->streaming_refcnt--;
	}

success:
	mutex_unlock(&pipeline->lock);
	return 0;

error_sensor_stream_disable:
	v4l2_subdev_call(pipeline->sensor, video, s_stream, 0);
error:
	mutex_unlock(&pipeline->lock);
	return ret;
}

/**
 * kmb_cam_pipeline_s_power - Power ON/OFF LRT pipeline
 * @entity: pointer to media entity
 * @on: Set to power on LRT pipeline
 *
 * Return: 0 if successful
 */
static int kmb_cam_pipeline_s_power(struct media_entity *entity, int on)
{
	struct kmb_cam_pipeline *pipeline = to_kmb_camera_pipe(entity);
	struct kmb_cam_ctrl_channel *c_chan = pipeline->rt_pipe.ctrl_chan;
	int ret;

	if (on)
		ret = kmb_cam_request_control_channel(c_chan);
	else
		ret = kmb_cam_release_control_channel(c_chan);

	return ret;
}

/* Pipeline operations */
struct kmb_pipeline_ops kmb_pipe_ops = {
	.s_power = kmb_cam_pipeline_s_power,
	.build = kmb_cam_build_pipeline,
	.teardown = kmb_cam_teardown_pipeline,
	.request_config = kmb_cam_pipeline_request_config,
	.release_config = kmb_cam_pipeline_release_config,
	.s_stream = kmb_cam_pipeline_s_stream,
};

/**
 * kmb_cam_init_subdevices - Initialize Kmb camera subdevices
 * @kmb_cam: pointer to kmb media device
 * @pipeline: pointer to kmb_cam_pipeline
 *
 * Return: 0 if successful
 */
static int kmb_cam_init_subdevices(struct kmb_camera *kmb_cam,
				   struct kmb_cam_pipeline *pipeline)
{
	char vnode_name[KMB_MAX_OUTPUT_NAME_LEN];
	int i;
	int ret;

	pipeline->isp.pipe = &pipeline->media_pipe;
	pipeline->isp.priv = (void *)pipeline;
	pipeline->isp.ipc_dev_handler = &kmb_cam->ipc_dev_handler;
	pipeline->isp.channel_ops = kmb_isp_channel_ops;
	kmb_cam_config_isp_src(kmb_cam, pipeline);

	kmb_isp_init(&pipeline->isp, kmb_cam->dev);

	pipeline->raw_input.pipe = &pipeline->media_pipe;
	pipeline->raw_input.ipc_dev_handler = &kmb_cam->ipc_dev_handler;
	pipeline->raw_input.priv = (void *)pipeline;
	pipeline->raw_input.dma_dev = kmb_cam->dev;
	pipeline->raw_input.pipe_ops = kmb_pipe_ops;
	pipeline->raw_input.channel_ops = kmb_vid_channel_ops;

	ret = kmb_video_init(&pipeline->raw_input, "isp-raw-input",
			     KMB_VIDEO_RAW_INPUT);
	if (ret < 0)
		dev_err(kmb_cam->dev, "Fail to init raw input video node\n");

	for (i = 0; i < KMB_MAX_OUTPUT_COUNT; i++) {
		memset(vnode_name, 0, KMB_MAX_OUTPUT_NAME_LEN);
		pipeline->output[i].pipe = &pipeline->media_pipe;
		pipeline->output[i].ipc_dev_handler = &kmb_cam->ipc_dev_handler;
		pipeline->output[i].priv = (void *)pipeline;
		pipeline->output[i].dma_dev = kmb_cam->dev;
		pipeline->output[i].pipe_ops = kmb_pipe_ops;
		pipeline->output[i].channel_ops = kmb_vid_channel_ops;

		snprintf(vnode_name, KMB_MAX_OUTPUT_NAME_LEN,
			 "isp-output-%d", i);
		ret = kmb_video_init(&pipeline->output[i], vnode_name,
				     KMB_VIDEO_YUV_OUTPUT);
		if (ret < 0)
			dev_err(kmb_cam->dev, "Fail to init video node\n");
	}

	pipeline->raw_output.pipe = &pipeline->media_pipe;
	pipeline->raw_output.ipc_dev_handler = &kmb_cam->ipc_dev_handler;
	pipeline->raw_output.priv = (void *)pipeline;
	pipeline->raw_output.dma_dev = kmb_cam->dev;
	pipeline->raw_output.pipe_ops = kmb_pipe_ops;
	pipeline->raw_output.channel_ops = kmb_vid_channel_ops;

	ret = kmb_video_init(&pipeline->raw_output, "isp-raw-output",
			     KMB_VIDEO_RAW_OUTPUT);
	if (ret < 0)
		dev_err(kmb_cam->dev, "Fail to init raw output video node\n");

	return ret;
}

/**
 * kmb_cam_register_entities - Register entities
 * @kmb_cam: pointer to kmb camera device
 * @pipeline: pointer to kmb_cam_pipeline
 *
 * Return: 0 if successful
 */
static int kmb_cam_register_entities(struct kmb_camera *kmb_cam,
				     struct kmb_cam_pipeline *pipeline)
{
	int i, j;
	int ret;

	ret = kmb_isp_register_entities(&pipeline->isp, &kmb_cam->v4l2_dev);
	if (ret < 0) {
		dev_err(kmb_cam->dev, "Fail to register kmb isp entity\n");
		goto error;
	}

	ret = kmb_video_register(&pipeline->raw_input, &kmb_cam->v4l2_dev);
	if (ret < 0) {
		dev_err(kmb_cam->dev, "Fail to register kmb raw input node\n");
		goto error_unregister_isp_entities;
	}

	for (i = 0; i < KMB_MAX_OUTPUT_COUNT; i++) {
		ret = kmb_video_register(&pipeline->output[i],
					 &kmb_cam->v4l2_dev);
		if (ret < 0) {
			dev_err(kmb_cam->dev,
				"Fail to register kmb video node %d\n", i);
			goto error_unregister_output_entities;
		}
	}

	ret = kmb_video_register(&pipeline->raw_output, &kmb_cam->v4l2_dev);
	if (ret < 0) {
		dev_err(kmb_cam->dev, "Fail to register kmb raw output node\n");
		goto error_unregister_output_entities;
	}

	return 0;

error_unregister_output_entities:
	for (j = 0; j < i; j++)
		kmb_video_unregister(&pipeline->output[i]);

	kmb_video_unregister(&pipeline->raw_input);
error_unregister_isp_entities:
	kmb_isp_unregister_entities(&pipeline->isp);
error:
	return ret;
}

/**
 * kmb_cam_unregister_entities - Unregister pipeline entities
 * @pipeline: pointer to kmb_cam_pipeline
 *
 * Return: none
 */
static void kmb_cam_unregister_entities(struct kmb_cam_pipeline *pipeline)
{
	int i;

	kmb_video_unregister(&pipeline->raw_input);
	for (i = 0; i < KMB_MAX_OUTPUT_COUNT; i++)
		kmb_video_unregister(&pipeline->output[i]);
	kmb_video_unregister(&pipeline->raw_output);
	kmb_isp_unregister_entities(&pipeline->isp);
	mutex_destroy(&pipeline->lock);
}

/**
 * kmb_cam_create_links - Create pad links connecting sub-devices
 * @kmb_cam: pointer to kmb camera device
 * @pipeline: pointer to kmb_cam_pipeline
 *
 * Return: 0 if successful
 */
static int kmb_cam_create_links(struct kmb_camera *kmb_cam,
				struct kmb_cam_pipeline *pipeline)
{
	int i, j;
	int ret;

	ret = media_create_pad_link(
			&pipeline->sensor->entity, 0,
			&pipeline->isp.subdev.entity, KMB_ISP_SINK_PAD_SENSOR,
			MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
	if (ret < 0) {
		dev_err(kmb_cam->dev, "Fail to link %s->%s entities\n",
				pipeline->sensor->entity.name,
				pipeline->isp.subdev.entity.name);
		return ret;
	}

	ret = media_create_pad_link(
			&pipeline->raw_input.video->entity, 0,
			&pipeline->isp.subdev.entity, KMB_ISP_SINK_PAD_RAW,
			MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
	if (ret < 0) {
		dev_err(kmb_cam->dev, "Fail to link %s->%s entities\n",
				pipeline->raw_input.video->entity.name,
				pipeline->isp.subdev.entity.name);
		goto error_remove_sensor_links;
	}

	for (i = 0; i < KMB_MAX_OUTPUT_COUNT; i++) {
		ret = media_create_pad_link(
				&pipeline->isp.subdev.entity,
				KMB_ISP_SRC_PAD_VID_BASE + i,
				&pipeline->output[i].video->entity, 0,
				MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
		if (ret < 0) {
			dev_err(kmb_cam->dev, "Fail to link %s->%s entities\n",
				pipeline->isp.subdev.entity.name,
				pipeline->output[i].video->entity.name);
			goto error_remove_video_links;
		}
	}

	ret = media_create_pad_link(
			&pipeline->isp.subdev.entity, KMB_ISP_SRC_PAD_RAW,
			&pipeline->raw_output.video->entity, 0,
			MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
	if (ret < 0) {
		dev_err(kmb_cam->dev, "Fail to link %s->%s entities\n",
				pipeline->isp.subdev.entity.name,
				pipeline->raw_output.video->entity.name);
		goto error_remove_video_links;
	}

	return 0;

error_remove_video_links:
	for (j = 0; j < i; j++)
		media_entity_remove_links(&pipeline->output[j].video->entity);

	media_entity_remove_links(&pipeline->raw_input.video->entity);
error_remove_sensor_links:
	media_entity_remove_links(&pipeline->sensor->entity);
	return ret;
}

/**
 * kmb_cam_parse_mipi_config - Parse mipi csi2 configuration from device tree
 * @kmb_cam: pointer to kmb camera device
 * @fwnode: kmb camera port fwnode handle
 * @remote: fwnode handle of the remote port
 * @idx: index of the current port
 *
 * Return: 0 if successful
 */
static inline int kmb_cam_parse_mipi_config(struct kmb_camera *kmb_cam,
					    struct fwnode_handle *fwnode,
					    struct fwnode_handle *remote,
					    int idx)
{
	struct v4l2_fwnode_endpoint ep_data;
	u32 val = 0;
	int ret;

	memset(&ep_data, 0, sizeof(ep_data));
	ep_data.bus_type = V4L2_MBUS_CSI2_DPHY;
	ret = v4l2_fwnode_endpoint_parse(fwnode, &ep_data);
	if (ret) {
		dev_err(kmb_cam->dev, "No endpoint to parse in this fwnode\n");
		return -ENOENT;
	}

	kmb_cam->ep_data[idx].ep_data = ep_data;
	kmb_cam->ep_data[idx].remote = remote;

	dev_dbg(kmb_cam->dev, "num_data_lanes %d\n",
			ep_data.bus.mipi_csi2.num_data_lanes);

	ret = fwnode_property_read_u32(fwnode, "reg", &val);
	if (ret) {
		dev_err(kmb_cam->dev, "Could not match reg %d", ret);
		return ret;
	}

	kmb_cam->ep_data[idx].controller_num = val;

	dev_dbg(kmb_cam->dev, "mipi controller %u\n", val);

	return 0;
}

/**
 * kmb_cam_bound - Bound
 * @n: pointer to V4L2 asynchronous notifier
 * @sd: pointer to V4L2 subdevice
 * @asd: pointer to V4L2 async subdevice
 *
 * This function is called when a subdevice driver has successfully probed one
 * of the subdevices.
 *
 * Return: 0 if successful
 */
static int kmb_cam_bound(struct v4l2_async_notifier *n,
			 struct v4l2_subdev *sd,
			 struct v4l2_async_subdev *asd)
{
	struct v4l2_device *v4l2_dev = n->v4l2_dev;
	struct kmb_camera *kmb_cam =
			container_of(v4l2_dev, struct kmb_camera, v4l2_dev);
	struct kmb_cam_pipeline *pipeline;
	int ret;

	if (kmb_cam->idx >= (int)(ARRAY_SIZE(kmb_cam->pipeline) - 1)) {
		dev_info(kmb_cam->dev,
				"Reached max pipeline count. Skip Sensor!");
		return 0;
	}

	pipeline = &kmb_cam->pipeline[++kmb_cam->idx];

	kmb_cam_init_pipe(kmb_cam, pipeline);
	pipeline->sensor = sd;

	/* register isp, eeprom and video nodes for each sensor */
	kmb_cam_init_subdevices(kmb_cam, pipeline);

	ret = kmb_cam_register_entities(kmb_cam, pipeline);
	if (ret < 0) {
		dev_err(kmb_cam->dev, "Fail to register kmb entities: %d\n",
				ret);
		goto error_unregister_subdevs;
	}

	ret = kmb_cam_create_links(kmb_cam, pipeline);
	if (ret < 0) {
		dev_err(kmb_cam->dev, "Fail to create links: %d\n", ret);
		goto error_remove_links;
	}

	pipeline->rt_pipe.pipe_instance_id = kmb_cam->idx;

	return 0;

error_remove_links:
	media_entity_remove_links(&pipeline->sensor->entity);
	media_entity_remove_links(&pipeline->isp.subdev.entity);
error_unregister_subdevs:
	kmb_cam_unregister_entities(pipeline);
	kmb_cam->idx--;
	return ret;
}
/**
 * kmb_cam_complete - Complete
 * @n: pointer to V4L2 asynchronous notifier
 *
 * This function is called when all sub-devices have been probed successfully.
 *
 * Return: 0 if successful
 */
static int kmb_cam_complete(struct v4l2_async_notifier *n)
{
	int ret;

	ret = v4l2_device_register_subdev_nodes(n->v4l2_dev);
	if (ret < 0) {
		ret = -ENOENT;
		pr_err("Failed to register subdevs to media v4l2 device");
	}

	return ret;
}

static const struct v4l2_async_notifier_operations notifier_ops = {
	.bound = kmb_cam_bound,
	.complete = kmb_cam_complete
};

/**
 * kmb_cam_parse_nodes - Parse nodes
 * @kmb_cam: pointer to kmb camera device
 *
 * Return: 0 if successful
 */
static int kmb_cam_parse_nodes(struct kmb_camera *kmb_cam)
{
	struct fwnode_handle *remote;
	struct fwnode_handle *fwnode = NULL;
	struct v4l2_async_notifier *n;
	struct v4l2_async_subdev *a_subdev;
	struct fwnode_handle *cam_fwnode = dev_fwnode(kmb_cam->dev);
	int ret = 0;
	int i = 0;

	fwnode = fwnode_graph_get_next_endpoint(cam_fwnode, fwnode);
	while (fwnode) {
		n = &kmb_cam->v4l2_notifier[i];
		v4l2_async_notifier_init(n);
		remote = fwnode_graph_get_remote_port_parent(fwnode);
		a_subdev = v4l2_async_notifier_add_fwnode_subdev(
				n, remote,
				sizeof(struct v4l2_async_subdev));
		if (IS_ERR(a_subdev))
			return PTR_ERR(a_subdev);

		ret = kmb_cam_parse_mipi_config(kmb_cam, fwnode, remote, i);
		if (ret < 0)
			return ret;

		n->ops = &notifier_ops;
		ret = v4l2_async_notifier_register(&kmb_cam->v4l2_dev, n);
		if (ret != 0)
			pr_err("Could not register notifier! %d", ret);

		i++;
		if (i == KMB_MAX_SENSOR_COUNT) {
			pr_err("Max notifiers count reached!!!\n");
			break;
		}
		fwnode = fwnode_graph_get_next_endpoint(cam_fwnode, fwnode);
	}

	return ret;
}

/**
 * kmb_cam_probe - Platform device binding
 * @pdev: pointer to platform device
 *
 * Return: 0 if successful,
 *         -ENOMEM if there is not enough memory
 *         -EPROBE_DEFER on probe retry
 */
static int kmb_cam_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct v4l2_device *v4l2_dev;
	struct kmb_camera *kmb_cam;
	int ret;

	dev_dbg(&pdev->dev, "%s: Probe KMB media\n", __func__);

	kmb_cam = devm_kzalloc(dev, sizeof(*kmb_cam), GFP_KERNEL);
	if (!kmb_cam)
		return -ENOMEM;

	kmb_cam->dev = dev;

	strlcpy(kmb_cam->media_dev.model, "KMB Camera Media device",
			sizeof(kmb_cam->media_dev.model));
	kmb_cam->media_dev.dev = dev;
	kmb_cam->media_dev.hw_revision = 0;
	media_device_init(&kmb_cam->media_dev);

	v4l2_dev = &kmb_cam->v4l2_dev;
	v4l2_dev->mdev = &kmb_cam->media_dev;
	strlcpy(v4l2_dev->name, "kmb-camera-media", sizeof(v4l2_dev->name));

	ret = v4l2_device_register(dev, &kmb_cam->v4l2_dev);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "Fail to register v4l2_device: %d\n", ret);
		return ret;
	}

	/* xlink channel ID allocator */
	ida_init(&kmb_cam->id_alloc);

	/* Connect to the device before opening channels */
	memset(&kmb_cam->ipc_dev_handler, 0, sizeof(kmb_cam->ipc_dev_handler));
	kmb_cam->ipc_dev_handler.dev_type = VPUIP_DEVICE;
	ret = xlink_connect(&kmb_cam->ipc_dev_handler);
	if (ret) {
		dev_err(&pdev->dev, "Failed to connect: %d\n", ret);
		goto error_v4l2_dev_unregister;
	}
	dev_dbg(&pdev->dev, "Connected to Xlink\n");

	ret = kmb_cam_init_control_channel(kmb_cam);
	if (ret < 0)
		goto error_disconnect_xlink;

	kmb_cam->idx = -1;
	ret = kmb_cam_parse_nodes(kmb_cam);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "Fail to parse nodes: %d\n", ret);
		goto error_destroy_control_channel;
	}

	platform_set_drvdata(pdev, kmb_cam);

	ret = media_device_register(&kmb_cam->media_dev);
	if (ret < 0) {
		dev_err(kmb_cam->dev, "Fail to register media device: %d\n",
				ret);
		goto error_destroy_control_channel;
	}

	return 0;

error_destroy_control_channel:
	kmb_cam_destroy_control_channel(kmb_cam);
error_disconnect_xlink:
	xlink_disconnect(&kmb_cam->ipc_dev_handler);
error_v4l2_dev_unregister:
	ida_destroy(&kmb_cam->id_alloc);
	v4l2_device_unregister(&kmb_cam->v4l2_dev);
	return ret;
}

/**
 * kmb_cam_remove - Platform device unbinding
 * @pdev: pointer to platform device
 *
 * Return: 0 if successful
 */
static int kmb_cam_remove(struct platform_device *pdev)
{
	struct kmb_camera *kmb_cam = platform_get_drvdata(pdev);
	int ret;
	int i;

	dev_dbg(&pdev->dev, "%s: Remove KMB media\n", __func__);

	kmb_cam_destroy_control_channel(kmb_cam);

	/* Disconnect from the device after closing channels */
	ret = xlink_disconnect(&kmb_cam->ipc_dev_handler);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed to disconnect: %d\n", ret);
	}
	dev_dbg(&pdev->dev, "Disconnected from Xlink\n");

	for (i = 0; i <= kmb_cam->idx; i++) {
		v4l2_async_notifier_unregister(&kmb_cam->v4l2_notifier[i]);
		kmb_cam_unregister_entities(&kmb_cam->pipeline[i]);
	}

	kmb_cam->idx = -1;
	ida_destroy(&kmb_cam->id_alloc);

	media_device_unregister(&kmb_cam->media_dev);
	media_device_cleanup(&kmb_cam->media_dev);

	return 0;
}

static const struct of_device_id kmb_cam_dt_match[] = {
	{.compatible = "intel,kmb-camera"},
	{}
};
MODULE_DEVICE_TABLE(of, kmb_cam_dt_match);

static struct platform_driver kmb_cam_drv = {
	.probe	= kmb_cam_probe,
	.remove = kmb_cam_remove,
	.driver = {
		.name = "kmb-camera",
		.owner = THIS_MODULE,
		.of_match_table = kmb_cam_dt_match,
	}
};

module_platform_driver(kmb_cam_drv);

MODULE_DESCRIPTION("Keem Bay Camera V4L2 media device");
MODULE_LICENSE("GPL v2");
