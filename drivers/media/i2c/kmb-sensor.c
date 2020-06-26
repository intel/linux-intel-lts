// SPDX-License-Identifier: GPL-2.0-only
/*
 * Keem Bay Camera Sensor Driver.
 *
 * Copyright (C) 2018-2020 Intel Corporation
 */
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-image-sizes.h>

#include "kmb-sensor.h"

/**
 * dummy_i2c_transfer - dummy function mocking i2c_transfer
 * @adap: Handle to I2C bus
 * @msgs: One or more messages to execute before STOP is issued to
 *	terminate the operation; each message begins with a START.
 * @num: Number of messages to be executed.
 *
 * Return: 2 if success
 */
static int dummy_i2c_transfer(struct i2c_adapter *adap,
			      struct i2c_msg *msgs, int num)
{
	dev_dbg(&adap->dev, "%s: Nothing to be done!\n", __func__);
	return 2;
}

/**
 * dummy_i2c_master_send - dummy function mocking i2c_master_send
 * @client: Handle to slave device
 * @buf: Data that will be written to the slave
 * @count: How many bytes to write, must be less than 64k since msg.len is u16
 *
 * Return: 4 if success
 */
static int dummy_i2c_master_send(const struct i2c_client *client,
				 const char *buf, int count)
{
	dev_dbg(&client->dev, "%s: Nothing to be done!\n", __func__);
	return 4;
}

/**
 * i2c_read - Read the value of specific address
 * @client: pointer to i2c client
 * @addr: address to read from
 * @val: pointer to read value
 *
 * Return: 0 if successful
 */
static int i2c_read(struct i2c_client *client, u16 addr, u16 *val)
{
	u16 wr, rd = 0;
	struct i2c_msg msg[] = {
		{ .addr = client->addr, .flags = 0,
		  .len = 2, .buf = (u8 *)&wr },
		{ .addr = client->addr, .flags = I2C_M_RD,
		  .len = 2, .buf = (u8 *)&rd }
	};

	dev_dbg(&client->dev, "%s: 0x%04x @ 0x%04x.\n", __func__, *val, addr);

	wr = addr;
	dummy_i2c_transfer(client->adapter, msg, 2);

	*val = rd;

	return 0;
}

/**
 * i2c_write - Write value to an address
 * @client: pointer to i2c client
 * @addr: address to write to
 * @val: value to be written
 *
 * Return: 0 if successful
 */
static int i2c_write(struct i2c_client *client, u16 addr, u16 val)
{
	u8 buf[4] = { addr >> 8, addr & 0xFF, val >> 8, val & 0xFF };

	dev_dbg(&client->dev, "%s: 0x%04x @ 0x%04X\n", __func__, val, addr);

	dummy_i2c_master_send(client, buf, 4);

	return 0;
}

/**
 * i2c_write_array - Write an array of values
 * @client: pointer to i2c client
 * @regs: pointer to an array of address - value pairs
 * @count: number of array elements
 *
 * Return: 0 if successful
 */
static int i2c_write_array(struct i2c_client *client,
			   const struct i2c_arr *regs,
			   const int count)
{
	int i;

	dev_dbg(&client->dev, "%s\n", __func__);

	if (regs == NULL || count == 0) {
		dev_err(&client->dev, "%s: Nothing to be written\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < count; i++)
		i2c_write(client, regs[i].addr, regs[i].val);

	return 0;
}

/**
 * kmb_sensor_set_params - Set stream parameters
 * @kmb_sensor: pointer to kmb sensor device
 *
 * Return: 0 if successful
 */
static int kmb_sensor_set_params(struct kmb_sensor *kmb_sensor)
{
	dev_dbg(kmb_sensor->subdev.dev, "%s\n", __func__);
	return 0;
}

/**
 * kmb_sensor_enum_mbus_code - Enum mbus format code
 * @sd: pointer to V4L2 sub-device
 * @cfg: pointer to sub-device pad information
 * @code: pointer to media bus format code enumeration
 *
 * VIDIOC_SUBDEV_ENUM_MBUS_CODE ioctl handler
 *
 * Return: 0 if successful
 */
static int kmb_sensor_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
	struct kmb_sensor *kmb_sensor =
			container_of(sd, struct kmb_sensor, subdev);

	dev_dbg(sd->dev, "%s\n", __func__);

	if (code->index > 0)
		return -EINVAL;

	mutex_lock(&kmb_sensor->lock);
	code->code = kmb_sensor->curr_fmt.code;
	mutex_unlock(&kmb_sensor->lock);
	return 0;
}

/**
 * kmb_sensor_enum_frame_sizes - Enum frame sizes
 * @sd: pointer to V4L2 sub-device
 * @cfg: pointer to sub-device pad information
 * @fse: pointer to media bus format enumeration
 *
 * VIDIOC_SUBDEV_ENUM_FRAME_SIZE ioctl handler
 *
 * Return: 0 if successful
 */
static int kmb_sensor_enum_frame_sizes(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_frame_size_enum *fse)
{
	dev_dbg(sd->dev, "%s\n", __func__);
	return 0;
}

/**
 * kmb_sensor_get_fmt - Get mbus format
 * @sd: pointer to V4L2 sub-device
 * @cfg: pointer to sub-device pad information
 * @fmt: pointer to pad-level media bus format
 *
 * VIDIOC_SUBDEV_G_FMT ioctl handler
 *
 * Return: 0 if successful
 */
static int kmb_sensor_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *fmt)
{
	struct kmb_sensor *kmb_sensor =
			container_of(sd, struct kmb_sensor, subdev);

	dev_dbg(sd->dev, "%s\n", __func__);

	mutex_lock(&kmb_sensor->lock);
	fmt->format = kmb_sensor->curr_fmt;
	mutex_unlock(&kmb_sensor->lock);

	return 0;
}

/**
 * kmb_sensor_set_fmt - Set mbus format
 * @sd: pointer to V4L2 sub-device
 * @cfg: pointer to sub-device pad information
 * @fmt: pointer to pad-level media bus format
 *
 * VIDIOC_SUBDEV_S_FMT ioctl handler
 *
 * Return: 0 if successful
 */
static int kmb_sensor_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *fmt)
{
	struct kmb_sensor *kmb_sensor =
			container_of(sd, struct kmb_sensor, subdev);

	dev_dbg(sd->dev, "%s\n", __func__);

	mutex_lock(&kmb_sensor->lock);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		cfg->try_fmt = fmt->format;
	else
		kmb_sensor->curr_fmt = fmt->format;
	mutex_unlock(&kmb_sensor->lock);

	return 0;
}

/**
 * kmb_sensor_s_stream - Set video stream stop/start
 * @sd: pointer to V4L2 sub-device
 * @enable: stream state
 *
 * Return: 0 if successful
 */
static int kmb_sensor_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct kmb_sensor *kmb_sensor =
			container_of(sd, struct kmb_sensor, subdev);
	struct kmb_sensor_ctrls *ctrls = &kmb_sensor->ctrls;
	int ret = 0;

	dev_dbg(kmb_sensor->subdev.dev, "%s: Set stream on %d\n",
			__func__, enable);

	mutex_lock(&kmb_sensor->lock);
	if (kmb_sensor->streaming != enable) {
		if (enable)
			ret = kmb_sensor_set_params(kmb_sensor);

		if (!ret && ctrls->update) {
			mutex_unlock(&kmb_sensor->lock);
			ret = v4l2_ctrl_handler_setup(&ctrls->handler);
			mutex_lock(&kmb_sensor->lock);
			if (!ret)
				ctrls->update = 0;
		}
		if (!ret)
			ret = i2c_write(client, 0, enable);
	}

	if (!ret)
		kmb_sensor->streaming = enable ? true : false;

	mutex_unlock(&kmb_sensor->lock);
	return ret;
}

/**
 * kmb_sensor_get_default_format - Get default media bus format
 * @mbus_fmt: pointer to media bus format
 *
 * Return: none
 */
static void kmb_sensor_get_default_format(struct v4l2_mbus_framefmt *mbus_fmt)
{
	mbus_fmt->width = 2616;
	mbus_fmt->height = 1964;
	mbus_fmt->colorspace = V4L2_COLORSPACE_SRGB;
	mbus_fmt->code = MEDIA_BUS_FMT_SRGGB12_1X12;
	mbus_fmt->field = V4L2_FIELD_NONE;
}

/**
 * kmb_sensor_open - Sub-device node open
 * @sd: pointer to V4L2 sub-device
 * @fh: pointer to V4L2 sub-device file handle
 *
 * Return: 0 if successful
 */
static int kmb_sensor_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct kmb_sensor *kmb_sensor =
			container_of(sd, struct kmb_sensor, subdev);

	dev_dbg(sd->dev, "%s wxh %dx%d cspc %d\n", __func__,
			kmb_sensor->curr_fmt.width, kmb_sensor->curr_fmt.height,
			kmb_sensor->curr_fmt.colorspace);

	return 0;
}

/**
 * kmb_sensor_close - Sub-device node close
 * @sd: pointer to V4L2 sub-device
 * @fh: pointer to V4L2 sub-device file handle
 *
 * Return: 0 if successful
 */
static int kmb_sensor_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	dev_dbg(sd->dev, "%s\n", __func__);
	return 0;
}

/**
 * kmb_sensor_ioctl - Private ioctl handling
 * @sd: pointer to V4L2 sub-device
 * @cmd: ioctl command
 * @arg: pointer to ioctl arguments
 *
 * Return: 0 if successful, -ENOTTY if command is not supported
 */
static long kmb_sensor_ioctl(struct v4l2_subdev *sd,
		unsigned int cmd, void *arg)
{
	int ret = 0;

	switch (cmd) {
	default:
		dev_dbg(sd->dev, "%s: Unknown cmd 0x%x\n", __func__, cmd);
		ret = -ENOTTY;
		break;
	}

	return ret;
}

/**
 * kmb_sensor_s_power - Put device in power saving or normal mode
 * @sd: pointer to V4L2 sub-device
 * @on: power state (0 - power saving mode, 1 - normal operation mode)
 *
 * Return: 0 if successful
 */
static int kmb_sensor_s_power(struct v4l2_subdev *sd, int on)
{
	struct kmb_sensor *kmb_sensor =
			container_of(sd, struct kmb_sensor, subdev);
	struct i2c_client *client = kmb_sensor->client;

	mutex_lock(&kmb_sensor->lock);
	if (kmb_sensor->power_on != on) {
		dev_dbg(sd->dev, "%s: Set power_on %d\n", __func__, on);
		i2c_write_array(client, NULL, 0);
		kmb_sensor->power_on = on;
	}
	kmb_sensor->streaming = false;
	mutex_unlock(&kmb_sensor->lock);

	dev_dbg(sd->dev, "%s: Set power_on %d\n", __func__, on);
	return 0;
}

/**
 * kmb_sensor_s_ctrl - Set new control value
 * @ctrl: pointer to control structure
 *
 * Return: 0 if successful
 */
static int kmb_sensor_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd =
			&(container_of(ctrl->handler, struct kmb_sensor,
					ctrls.handler)->subdev);
	struct kmb_sensor *kmb_sensor =
			container_of(sd, struct kmb_sensor, subdev);
	int ret = 0;

	dev_dbg(sd->dev, "s_ctrl: %s, value: %d. power: %d\n",
			 ctrl->name, ctrl->val, kmb_sensor->power_on);

	mutex_lock(&kmb_sensor->lock);

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE_ABSOLUTE:
		dev_dbg(sd->dev, "%s: set exposure\n", __func__);
		break;
	case V4L2_CID_GAIN:
		dev_dbg(sd->dev, "%s: set gain\n", __func__);
		break;
	default:
		dev_dbg(sd->dev, "%s: KMB sensor set %d\n", __func__, ctrl->id);
	}

	mutex_unlock(&kmb_sensor->lock);
	return ret;
}

/* sub-device internal operations */
static const struct v4l2_subdev_internal_ops kmb_sensor_sd_internal_ops = {
	.open = kmb_sensor_open,
	.close = kmb_sensor_close,
};

/* sub-device core operations */
static const struct v4l2_subdev_core_ops kmb_sensor_core_ops = {
	.ioctl = kmb_sensor_ioctl,
	.s_power = kmb_sensor_s_power,
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

/* sub-device pad operations */
static const struct v4l2_subdev_pad_ops kmb_sensor_pad_ops = {
	.enum_mbus_code = kmb_sensor_enum_mbus_code,
	.enum_frame_size = kmb_sensor_enum_frame_sizes,
	.get_fmt = kmb_sensor_get_fmt,
	.set_fmt = kmb_sensor_set_fmt,
};

/* sub-device video operations */
static const struct v4l2_subdev_video_ops kmb_sensor_video_ops = {
	.s_stream = kmb_sensor_s_stream,
};

/* sub-device operations */
static const struct v4l2_subdev_ops kmb_sensor_subdev_ops = {
	.core = &kmb_sensor_core_ops,
	.pad = &kmb_sensor_pad_ops,
	.video = &kmb_sensor_video_ops,
};

/* V4L2 control operations */
static const struct v4l2_ctrl_ops kmb_sensor_ctrl_ops = {
	.s_ctrl = kmb_sensor_s_ctrl,
};

/**
 * kmb_sensor_initialize_controls - Initialize handled sensor controls
 * @kmb_sensor: pointer to sensor device
 *
 * Return: 0 if successful
 */
static int kmb_sensor_initialize_controls(struct kmb_sensor *kmb_sensor)
{
	const struct v4l2_ctrl_ops *ops = &kmb_sensor_ctrl_ops;
	struct kmb_sensor_ctrls *ctrls = &kmb_sensor->ctrls;
	int ret;

	ret = v4l2_ctrl_handler_init(&ctrls->handler, 16);
	if (ret < 0)
		return ret;

	ctrls->exposure = v4l2_ctrl_new_std(&ctrls->handler, ops,
			V4L2_CID_EXPOSURE_ABSOLUTE,
			2, 1500, 1, 500);

	ctrls->gain = v4l2_ctrl_new_std(&ctrls->handler, ops,
			V4L2_CID_GAIN, 16, 64 * (16 + 15), 1, 64 * 16);

	if (ctrls->handler.error) {
		ret = ctrls->handler.error;
		v4l2_ctrl_handler_free(&ctrls->handler);
		return ret;
	}

	kmb_sensor->subdev.ctrl_handler = &ctrls->handler;
	return ret;
}

/**
 * kmb_sensor_detect - Detect V4L2 sub-device
 * @sd: pointer to V4L2 sub-device
 *
 * Return: 0 if successful, -ENODEV if there is no device
 */
static int kmb_sensor_detect(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 pid, ver;

	dev_dbg(sd->dev, "%s\n", __func__);

	kmb_sensor_s_power(sd, 1);
	usleep_range(25000, 26000);

	/* Check sensor revision */
	i2c_read(client, 0, &pid);
	i2c_read(client, 0, &ver);

	kmb_sensor_s_power(sd, 0);

	return 0;
}

/**
 * kmb_sensor_probe - I2C client device binding
 * @client: pointer to i2c client device
 * @id: pointer to i2c device id
 *
 * Return: 0 if successful, -ENOMEM if there is not enough memory
 */
static int kmb_sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct kmb_sensor *kmb_sensor;
	struct media_pad *pads;
	int ret;

	dev_dbg(&client->dev, "%s: Probe KMB sensor\n", __func__);

	kmb_sensor = devm_kzalloc(&client->dev,
			sizeof(*kmb_sensor), GFP_KERNEL);
	if (!kmb_sensor)
		return -ENOMEM;

	mutex_init(&kmb_sensor->lock);
	kmb_sensor->client = client;

	sd = &kmb_sensor->subdev;
	v4l2_i2c_subdev_init(&kmb_sensor->subdev, client,
			&kmb_sensor_subdev_ops);
	v4l2_subdev_init(&kmb_sensor->subdev, &kmb_sensor_subdev_ops);
	strlcpy(sd->name, KMB_SENSOR_DRV_NAME, sizeof(sd->name));
	v4l2_set_subdevdata(sd, client);

	kmb_sensor->subdev.internal_ops = &kmb_sensor_sd_internal_ops;
	kmb_sensor->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
				V4L2_SUBDEV_FL_HAS_EVENTS;
	kmb_sensor->subdev.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	strlcpy(kmb_sensor->subdev.name, KMB_SENSOR_DRV_NAME,
			sizeof(kmb_sensor->subdev.name));

	pads = kmb_sensor->pads;
	pads[0].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&kmb_sensor->subdev.entity,
				     KMB_SENSOR_PADS_NUM, pads);
	if (ret < 0)
		return ret;

	ret = kmb_sensor_initialize_controls(kmb_sensor);
	if (ret < 0)
		goto error_media_entity_cleanup;

	kmb_sensor_get_default_format(kmb_sensor->formats);
	kmb_sensor->curr_fmt = kmb_sensor->formats[0];

	kmb_sensor_detect(sd);

	v4l2_set_subdev_hostdata(sd, &kmb_sensor->subdev);

	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret < 0)
		goto error_free_ctrl_handler;

	return 0;

error_free_ctrl_handler:
	v4l2_ctrl_handler_free(sd->ctrl_handler);
error_media_entity_cleanup:
	media_entity_cleanup(&sd->entity);
	return ret;
}

/**
 * kmb_sensor_remove - I2C client device unbinding
 * @client: pointer to I2C client device
 *
 * Return: 0 if successful
 */
static int kmb_sensor_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct kmb_sensor *kmb_sensor =
			container_of(sd, struct kmb_sensor, subdev);

	dev_dbg(sd->dev, "%s: Remove KMB sensor\n", __func__);

	mutex_destroy(&kmb_sensor->lock);
	v4l2_async_unregister_subdev(sd);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	media_device_unregister_entity(&sd->entity);
	media_entity_cleanup(&sd->entity);

	return 0;
}

static const struct i2c_device_id kmb_sensor_id_table[] = {
	{KMB_SENSOR_DRV_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, kmb_sensor_id_table);

static struct i2c_driver kmb_sensor_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = KMB_SENSOR_DRV_NAME,
	},
	.probe = kmb_sensor_probe,
	.remove = kmb_sensor_remove,
	.id_table = kmb_sensor_id_table,
};

module_i2c_driver(kmb_sensor_i2c_driver);

MODULE_DESCRIPTION("Keem Bay I2C Sensor driver");
MODULE_LICENSE("GPL v2");
