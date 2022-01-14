// SPDX-License-Identifier: GPL-2.0-only
/*
 * kmb-lens.c - KeemBay Camera Lens Driver.
 *
 * Copyright (C) 2019 Intel Corporation
 */
#include <linux/types.h>
#include <linux/regulator/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pwm.h>
#include <linux/delay.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "kmb-lens.h"

/**
 * struct kmb_lens_iris_step - Structure representing one iris step
 * @move_dt: Iris move duty cycle
 * @stop_dt: Iris stop duty cycle
 * @time_ms: Iris step movement time
 */
struct kmb_lens_iris_step {
	u16 move_dt;
	u16 stop_dt;
	u16 time_ms;
};

/*
 * Iris increment movement lookup. To move iris from currect to next posotion
 * use current index. Example to move from position 0 to 1 use 0 index.
 */
const struct kmb_lens_iris_step iris_step_inc[] = {
	{
		/* Move 0 -> 1 position */
		.move_dt = 87,
		.stop_dt = 75,
		.time_ms = 36,
	},
	{
		/* Move 1 -> 2 position */
		.move_dt = 85,
		.stop_dt = 75,
		.time_ms = 51,
	},
	{
		/* Move 2 -> 3 position */
		.move_dt = 85,
		.stop_dt = 75,
		.time_ms = 32,
	},
	{
		/* Move 3 -> 4 position */
		.move_dt = 89,
		.stop_dt = 75,
		.time_ms = 26,
	},
};

/*
 * Iris decrement movement lookup. To move iris from current to
 * previous posotion use previous index.
 * Example to move from position 1 to 0 use 0 index.
 */
const struct kmb_lens_iris_step iris_step_dec[] = {
	{
		/* Move 1 -> 0 position */
		.move_dt = 25,
		.stop_dt = 0,
		.time_ms = 10,
	},
	{
		/* Move 2 -> 1 position */
		.move_dt = 30,
		.stop_dt = 50,
		.time_ms = 10,
	},
	{
		/* Move 3 -> 2 position */
		.move_dt = 35,
		.stop_dt = 50,
		.time_ms = 11,
	},
	{
		/* Move 4 -> 3 position */
		.move_dt = 43,
		.stop_dt = 50,
		.time_ms = 9,
	},
};

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
 * kmb_iris_move - Function to move iris.
 * @iris: Iris pwm device
 * @move_dt: Iris move duty cycle
 * @time_ms: Iris movement time
 * @stop_dt: Iris move duty cycle
 *
 * Return: 0 if successful
 */
static int kmb_iris_move(struct pwm_device *iris,
			 unsigned int move_dt,
			 unsigned long time_ms,
			 unsigned int stop_dt)
{
	struct pwm_state state;
	unsigned long time_us = time_ms * 1000;

	pwm_get_state(iris, &state);
	state.enabled = true;

	pwm_set_relative_duty_cycle(&state, move_dt, 100);
	pwm_apply_state(iris, &state);

	/*
	 * usleep range as more precise. min and max
	 * should be different add 1 microsecond tolerance
	 */
	usleep_range(time_us, time_us + 1);

	pwm_set_relative_duty_cycle(&state, stop_dt, 100);
	pwm_apply_state(iris, &state);

	/* Wait 1 ms to stabilize before next move */
	usleep_range(1000, 2000);

	return 0;
}

/**
 * kmb_iris_control - Function to control iris position.
 * @kmb_lens: pointer to kmb lens device
 * @new_pos: Iris control new position
 *
 * Return: 0 if successful
 */
static int kmb_iris_control(struct kmb_lens *kmb_lens, u8 new_pos)
{
	int curr_pos;
	int i;

	if (!kmb_lens->iris) {
		dev_err(kmb_lens->subdev.dev, "Iris is missing");
		return -EINVAL;
	}

	curr_pos = kmb_lens->iris_cur_pos;
	if (curr_pos < 0) {
		dev_err(kmb_lens->subdev.dev, "Invalid iris control");
		return curr_pos;
	}

	if (new_pos > curr_pos) {
		for (i = curr_pos; i < new_pos; i++) {
			kmb_iris_move(kmb_lens->iris,
				iris_step_inc[i].move_dt,
				iris_step_inc[i].time_ms,
				iris_step_inc[i].stop_dt);

		}
	} else {
		for (i = curr_pos - 1; i >= new_pos; i--) {
			kmb_iris_move(kmb_lens->iris,
				iris_step_dec[i].move_dt,
				iris_step_dec[i].time_ms,
				iris_step_dec[i].stop_dt);
		}
	}
	kmb_lens->iris_cur_pos = new_pos;

	return 0;
}

static void kmb_lens_move_iris(struct work_struct *work)
{
	struct kmb_lens *kmb_lens = container_of(work, struct kmb_lens,
						 work_iris);
	u32 new_pos;

	mutex_lock(&kmb_lens->lock);
	new_pos = kmb_lens->iris_new_pos;
	mutex_unlock(&kmb_lens->lock);

	kmb_iris_control(kmb_lens, kmb_lens->iris_new_pos);
}

/**
 * kmb_lens_iris_init_step - Move iris actuator
 *                           to first predefined position
 * @kmb_lens: pointer to kmb lens device
 *
 */
static void kmb_lens_iris_init_step(struct kmb_lens *kmb_lens)
{
	kmb_iris_move(kmb_lens->iris,
		iris_step_inc[0].move_dt,
		iris_step_inc[0].time_ms,
		iris_step_inc[0].stop_dt);

	kmb_iris_move(kmb_lens->iris,
		iris_step_dec[0].move_dt,
		iris_step_dec[0].time_ms,
		iris_step_dec[0].stop_dt);
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

	if (on) {
		kmb_lens->iris_cur_pos = 0;
		gpiod_set_value_cansleep(kmb_lens->enable_gpio, 1);
		if (kmb_lens->iris)
			kmb_lens_iris_init_step(kmb_lens);
	} else {
		gpiod_set_value_cansleep(kmb_lens->enable_gpio, 0);
	}

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

	if (kmb_lens->iris) {
		struct pwm_state state;
		/* Move iris to defaut position */
		pwm_get_state(kmb_lens->iris, &state);
		state.enabled = false;
		pwm_apply_state(kmb_lens->iris, &state);
	}

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
	int ret = 0;

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
	case V4L2_CID_IRIS:
		if (kmb_lens->wq) {
			kmb_lens->iris_new_pos = ctrl->val;
			queue_work(kmb_lens->wq, &kmb_lens->work_iris);
		}
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

static const struct v4l2_ctrl_config ir_filter_ctrl = {
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

static const struct v4l2_ctrl_config iris_ctrl = {
	.ops = &kmb_lens_ctrl_ops,
	.id = V4L2_CID_IRIS,
	.name = "V4L2_CID_IRIS",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = ARRAY_SIZE(iris_step_inc),
	.def = 0,
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
	/* Custom controls */
	ctrls->ir_filter_ctrl = v4l2_ctrl_new_custom(&ctrls->handler,
						     &ir_filter_ctrl, NULL);
	ctrls->iris_ctrl = v4l2_ctrl_new_custom(&ctrls->handler,
						&iris_ctrl, NULL);

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

	/* Iris control is optional */
	kmb_lens->iris = devm_pwm_get(&client->dev, NULL);
	if (IS_ERR(kmb_lens->iris)) {
		if (PTR_ERR(kmb_lens->iris) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		dev_dbg(&client->dev, "Failed to get pwm %ld",
			PTR_ERR(kmb_lens->iris));

		kmb_lens->iris = NULL;
	} else {
		struct pwm_state state;

		/* Sync up Iris control pwm state */
		pwm_init_state(kmb_lens->iris, &state);
		state.enabled = false;
		ret = pwm_apply_state(kmb_lens->iris, &state);
		if (ret) {
			dev_err(&client->dev,
				"Failed to apply pwm state %d", ret);
			return ret;
		}

		kmb_lens->wq = alloc_workqueue("lens_wq", WQ_UNBOUND, 1);
		if (kmb_lens->wq == NULL) {
			dev_err(&client->dev, "cannot create lens workqueue");
			return -ENOMEM;
		}
		INIT_WORK(&kmb_lens->work_iris, kmb_lens_move_iris);
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

	if (kmb_lens->wq)
		destroy_workqueue(kmb_lens->wq);

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

MODULE_DESCRIPTION("KeemBay I2C Lens driver");
MODULE_LICENSE("GPL v2");
