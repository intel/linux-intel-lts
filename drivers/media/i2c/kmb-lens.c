// SPDX-License-Identifier: GPL-2.0-only
/*
 * Keem Bay Camera Lens Driver.
 *
 * Copyright (C) 2019-2020 Intel Corporation
 */
#include <linux/regulator/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "kmb-lens.h"

/**
 * dummy_i2c_transfer - dummy function mocking i2c_transfer
 * @adap: Handle to I2C bus
 * @msgs: One or more messages to execute before STOP is issued to
 *	terminate the operation; each message begins with a START.
 * @num: Number of messages to be executed.
 *
 * Return: number of messages sent on success, negative on fail
 */
static int dummy_i2c_transfer(struct i2c_adapter *adap,
		struct i2c_msg *msgs, int num)
{
	dev_dbg(&adap->dev, "%s: Nothing to be done!\n", __func__);
	return num;
}

/**
 * dummy_i2c_master_send - dummy function mocking i2c_master_send
 * @client: Handle to slave device
 * @buf: Data that will be written to the slave
 * @count: How many bytes to write, must be less than 64k since msg.len is u16
 *
 * Return: count of bytes written on success, negative on fail
 */
static int dummy_i2c_master_send(const struct i2c_client *client,
		const char *buf, int count)
{
	dev_dbg(&client->dev, "%s: Nothing to be done!\n", __func__);
	return count;
}

/**
 * i2c_read - Read the value of specific address
 * @client: pointer to i2c client
 * @addr: address to read from
 * @val: pointer to read value
 *
 * Return: 0 if successful
 */
static int __maybe_unused i2c_read(struct i2c_client *client,
		u16 addr, u16 *val)
{
	u16 wr, rd;
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
static int __maybe_unused i2c_write_array(struct i2c_client *client,
		const struct i2c_arr *regs, const int count)
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
 * kmb_lens_hw_update - Trigger hardware update
 * @kmb_lens: pointer to kmb lens device
 *
 * Return: 0 if successful
 */
static int kmb_lens_hw_update(struct kmb_lens *kmb_lens)
{
	dev_dbg(kmb_lens->subdev.dev, "%s: HW update\n", __func__);
	return 0;
}

/**
 * kmb_ir_filter - Function to control ir filter position.
 * @kmb_lens: pointer to kmb lens device
 * @on: Flag to insert/remove ir filter
 *
 * Return: 0 if successful
 */
static int kmb_ir_filter(struct kmb_lens *kmb_lens, bool on)
{
	if (!kmb_lens->phase_gpio)
		return 0;

	if (on)
		gpiod_set_value_cansleep(kmb_lens->phase_gpio, 0);
	else
		gpiod_set_value_cansleep(kmb_lens->phase_gpio, 1);

	return 0;
}

/**
 * kmb_lens_set_power - Set power on/off
 * @kmb_lens: pointer to kmb lens device
 * @on: power state
 *
 * Return: 0 if successful
 */
static int kmb_lens_set_power(struct kmb_lens *kmb_lens, bool on)
{
	if (!kmb_lens->enable_gpio)
		return 0;

	if (on)
		gpiod_set_value_cansleep(kmb_lens->enable_gpio, 1);
	else
		gpiod_set_value_cansleep(kmb_lens->enable_gpio, 0);

	return 0;
}

/**
 * kmb_lens_open - Sub-device node open
 * @sd: pointer to V4L2 sub-device
 * @fh: pointer to V4L2 sub-device file handle
 *
 * Return: 0 if successful
 */
static int kmb_lens_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct kmb_lens *kmb_lens = v4l2_get_subdevdata(sd);

	return kmb_lens_set_power(kmb_lens, true);
}

/**
 * kmb_lens_close - Sub-device node close
 * @sd: pointer to V4L2 sub-device
 * @fh: pointer to V4L2 sub-device file handle
 *
 * Return: 0 if successful
 */
static int kmb_lens_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct kmb_lens *kmb_lens = v4l2_get_subdevdata(sd);

	return kmb_lens_set_power(kmb_lens, false);
}

/**
 * kmb_lens_s_ctrl - Set new control value
 * @ctrl: pointer to control structure
 *
 * Return: 0 if successful
 */
static int kmb_lens_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = &(container_of(ctrl->handler, struct kmb_lens,
			ctrls.handler)->subdev);
	struct kmb_lens *kmb_lens = v4l2_get_subdevdata(sd);
	int ret;

	mutex_lock(&kmb_lens->lock);
	switch (ctrl->id) {
	case V4L2_CID_FOCUS_ABSOLUTE:
	case V4L2_CID_FOCUS_RELATIVE:
	case V4L2_CID_FOCUS_AUTO:
		ret = kmb_lens_hw_update(kmb_lens);
		break;
	case V4L2_CID_IR_FILTER:
		ret = kmb_ir_filter(kmb_lens, ctrl->val);
		break;
	default:
		dev_err(kmb_lens->subdev.dev, "%s: set %s . Not handled.\n",
				__func__, v4l2_ctrl_get_name(ctrl->id));
		ret = -EINVAL;
	}

	mutex_unlock(&kmb_lens->lock);
	return ret;
}

/* V4L2 control operations */
static const struct v4l2_ctrl_ops kmb_lens_ctrl_ops = {
	.s_ctrl = kmb_lens_s_ctrl,
};

static const struct v4l2_ctrl_config ir_ctrl = {
	.ops = &kmb_lens_ctrl_ops,
	.id = V4L2_CID_IR_FILTER,
	.name = "V4L2_CID_IR_FILTER",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.def = 1,
	.step = 1,
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
};

/**
 * kmb_lens_initialize_controls - Initialize handled lens controls
 * @kmb_lens: pointer to kmb lens device
 *
 * Return: 0 if successful
 */
static int kmb_lens_initialize_controls(struct kmb_lens *kmb_lens)
{
	struct kmb_lens_ctrls *ctrls = &kmb_lens->ctrls;
	int ret;

	dev_dbg(kmb_lens->subdev.dev, "%s\n", __func__);

	v4l2_ctrl_handler_init(&ctrls->handler, 4);

	ctrls->absolute_focus = v4l2_ctrl_new_std(
			&ctrls->handler, &kmb_lens_ctrl_ops,
			V4L2_CID_FOCUS_ABSOLUTE, 0, 1023, 1, 0);
	ctrls->relative_focus = v4l2_ctrl_new_std(
			&ctrls->handler, &kmb_lens_ctrl_ops,
			V4L2_CID_FOCUS_RELATIVE, 0, 1023, 1, 0);
	ctrls->auto_focus = v4l2_ctrl_new_std(
			&ctrls->handler, &kmb_lens_ctrl_ops,
			V4L2_CID_FOCUS_AUTO, 0, 1023, 1, 0);
	ctrls->ir_ctrl = v4l2_ctrl_new_custom(&ctrls->handler, &ir_ctrl, NULL);

	if (ctrls->handler.error) {
		ret = ctrls->handler.error;
		v4l2_ctrl_handler_free(&ctrls->handler);
		return ret;
	}

	kmb_lens->subdev.ctrl_handler = &ctrls->handler;

	return 0;
}

/**
 * kmb_lens_s_stream - Set video stream stop/start
 * @sd: pointer to V4L2 sub-device
 * @enable: stream state
 *
 * Return: 0 if successful
 */
static int kmb_lens_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct kmb_lens *kmb_lens = v4l2_get_subdevdata(sd);

	dev_dbg(kmb_lens->subdev.dev, "%s\n", __func__);
	return 0;
}

/**
 * kmb_lens_s_power - Put device in power saving or normal mode
 * @sd: pointer to V4L2 sub-device
 * @on: power state (0 - power saving mode, 1 - normal operation mode)
 *
 * Return: 0 if successful, else - error
 */
static int kmb_lens_s_power(struct v4l2_subdev *sd, int on)
{
	struct kmb_lens *kmb_lens = v4l2_get_subdevdata(sd);

	dev_dbg(kmb_lens->subdev.dev, "%s\n", __func__);
	return 0;
}

/* sub-device internal operations */
static const struct v4l2_subdev_internal_ops kmb_lens_sd_internal_ops = {
	.open = kmb_lens_open,
	.close = kmb_lens_close,
};

/* sub-device core operations */
static struct v4l2_subdev_core_ops kmb_lens_subdev_core_ops = {
	.s_power = kmb_lens_s_power,
};

/* sub-device video operations */
static struct v4l2_subdev_video_ops kmb_lens_subdev_video_ops = {
	.s_stream = kmb_lens_s_stream,
};

/* sub-device operations */
static const struct v4l2_subdev_ops kmb_lens_subdev_ops = {
	.core = &kmb_lens_subdev_core_ops,
	.video = &kmb_lens_subdev_video_ops,
};

/**
 * kmb_lens_probe - I2C client device binding
 * @client: pointer to i2c client device
 * @id: pointer to i2c device id
 *
 * Return: 0 if successful, -ENOMEM if there is not enough memory
 */
static int kmb_lens_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct kmb_lens *kmb_lens;
	int ret;

	dev_dbg(&client->dev, "%s: Probe KMB lens\n", __func__);

	kmb_lens = devm_kzalloc(&client->dev,
			sizeof(*kmb_lens), GFP_KERNEL);
	if (!kmb_lens)
		return -ENOMEM;

	mutex_init(&kmb_lens->lock);
	kmb_lens->client = client;

	kmb_lens->enable_gpio = devm_gpiod_get_optional(&client->dev, "enable",
							GPIOD_OUT_LOW);
	if (IS_ERR(kmb_lens->enable_gpio)) {
		dev_err(&client->dev, "failed to get enable gpio %ld",
			PTR_ERR(kmb_lens->enable_gpio));
		kmb_lens->enable_gpio = NULL;
	}

	kmb_lens->phase_gpio = devm_gpiod_get_optional(&client->dev, "phase",
						       GPIOD_OUT_LOW);
	if (IS_ERR(kmb_lens->phase_gpio)) {
		dev_err(&client->dev, "failed to get phase gpio %ld",
			PTR_ERR(kmb_lens->phase_gpio));
		kmb_lens->phase_gpio = NULL;
	}

	sd = &kmb_lens->subdev;
	v4l2_i2c_subdev_init(&kmb_lens->subdev, client,
			     &kmb_lens_subdev_ops);
	v4l2_subdev_init(&kmb_lens->subdev, &kmb_lens_subdev_ops);
	strlcpy(sd->name, KMB_LENS_DRV_NAME, sizeof(sd->name));
	sd->internal_ops = &kmb_lens_sd_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
				V4L2_SUBDEV_FL_HAS_EVENTS;
	sd->entity.function = MEDIA_ENT_F_LENS;

	v4l2_set_subdevdata(sd, kmb_lens);

	ret = kmb_lens_initialize_controls(kmb_lens);
	if (ret < 0)
		goto error_media_entity_cleanup;

	v4l2_set_subdev_hostdata(sd, &kmb_lens->subdev);

	ret = v4l2_async_register_subdev(sd);
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
 * kmb_lens_remove - I2C client device unbinding
 * @client: pointer to I2C client device
 *
 * Return: 0 if successful
 */
static int kmb_lens_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct kmb_lens *kmb_lens = v4l2_get_subdevdata(sd);

	dev_info(sd->dev, "%s: Remove KMB lens\n", __func__);

	mutex_destroy(&kmb_lens->lock);
	v4l2_async_unregister_subdev(sd);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	media_device_unregister_entity(&sd->entity);
	media_entity_cleanup(&sd->entity);

	return 0;
}

static const struct i2c_device_id kmb_lens_id_table[] = {
	{KMB_LENS_DRV_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, kmb_lens_id_table);

static struct i2c_driver kmb_lens_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = KMB_LENS_DRV_NAME,
	},
	.probe = kmb_lens_probe,
	.remove = kmb_lens_remove,
	.id_table = kmb_lens_id_table,
};

module_i2c_driver(kmb_lens_i2c_driver);

MODULE_DESCRIPTION("Keem Bay I2C Lens driver");
MODULE_LICENSE("GPL v2");
