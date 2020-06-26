// SPDX-License-Identifier: GPL-2.0-only
/*
 * Keem Bay Camera Flash Driver.
 *
 * Copyright (C) 2019-2020 Intel Corporation
 */
#include <linux/regulator/consumer.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "kmb-flash.h"

#define ON			1
#define OFF			0

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
 * kmb_flash_open - Sub-device node open
 * @sd: pointer to V4L2 sub-device
 * @fh: pointer to V4L2 sub-device file handle
 *
 * Return: 0 if successful
 */
static int kmb_flash_open(struct v4l2_subdev *sd,
			struct v4l2_subdev_fh *fh)
{
	struct kmb_flash *kmb_flash = v4l2_get_subdevdata(sd);

	dev_dbg(kmb_flash->subdev.dev, "Open not yet implemented\n");
	return 0;
}

/**
 * kmb_flash_close - Sub-device node close
 * @sd: pointer to V4L2 sub-device
 * @fh: pointer to V4L2 sub-device file handle
 *
 * Return: 0 if successful
 */
static int kmb_flash_close(struct v4l2_subdev *sd,
			 struct v4l2_subdev_fh *fh)
{
	struct kmb_flash *kmb_flash = v4l2_get_subdevdata(sd);

	cancel_work_sync(&kmb_flash->work_flash);
	dev_dbg(kmb_flash->subdev.dev, "Close not yet implemented\n");
	return 0;
}

/**
 * kmb_flash_g_ctrl - Get new value for the control passed as argument
 * @ctrl: pointer to control structure
 *
 * Return: 0 if successful
 */
static int kmb_flash_g_ctrl(struct v4l2_ctrl *ctrl)
{
	if (ctrl->id != V4L2_CID_FLASH_FAULT)
		return -EINVAL;

	return 0;
}

/**
 * kmb_flash_set_led_mode - Set flash led mode
 * @kmb_flash: pointer to kmb flash device
 * @mode: flash led mode
 *
 * Return 0 if successful
 */
static int kmb_flash_set_led_mode(struct kmb_flash *kmb_flash, int mode)
{
	int ret = 0;

	switch (mode) {
	case V4L2_FLASH_LED_MODE_NONE:
		gpiod_set_value_cansleep(kmb_flash->led_gpio, OFF);
		break;
	case V4L2_FLASH_LED_MODE_TORCH:
		gpiod_set_value_cansleep(kmb_flash->led_gpio, ON);
		break;
	default:
		dev_err(kmb_flash->subdev.dev,
			"%s: unsupported flash mode\n", __func__);
		ret = -EINVAL;
	}

	return ret;
}

/**
 * kmb_flash_fire - Launches the flash
 * @work: pointer to work struct
 *
 * Return: 0 if successful
 */
static void kmb_flash_fire(struct work_struct *work)
{
	struct kmb_flash *kmb_flash = container_of(work, struct kmb_flash,
						   work_flash);
	unsigned long duration = atomic_read(&kmb_flash->duration);

	gpiod_set_value_cansleep(kmb_flash->led_gpio, ON);
	usleep_range(duration, duration + 1);
	gpiod_set_value_cansleep(kmb_flash->led_gpio, OFF);
}

/**
 * kmb_flash_s_ctrl - Set new control value
 * @ctrl: pointer to control structure
 *
 * Return: 0 if successful
 */
static int kmb_flash_s_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret = 0;
	struct v4l2_subdev *sd = &(container_of(ctrl->handler, struct kmb_flash,
			ctrls.handler)->subdev);
	struct kmb_flash *kmb_flash = v4l2_get_subdevdata(sd);

	mutex_lock(&kmb_flash->lock);
	switch (ctrl->id) {
	case V4L2_CID_FLASH_LED_MODE:
		ret = kmb_flash_set_led_mode(kmb_flash, ctrl->val);
		break;
	case V4L2_CID_FLASH_STROBE_SOURCE:
		break;
	case V4L2_CID_FLASH_STROBE:
		break;
	case V4L2_CID_FLASH_STROBE_STOP:
		break;
	case V4L2_CID_FLASH_TIMEOUT:
		atomic_set(&kmb_flash->duration, ctrl->val);
		queue_work(kmb_flash->wq, &kmb_flash->work_flash);

		break;
	case V4L2_CID_FLASH_INTENSITY:
	case V4L2_CID_FLASH_TORCH_INTENSITY:
		ret = 0;
		break;
	default:
		dev_dbg(kmb_flash->subdev.dev, "%s: set %s . Not handled.\n",
				__func__, v4l2_ctrl_get_name(ctrl->id));
		ret = -EINVAL;
	}

	mutex_unlock(&kmb_flash->lock);
	return ret;
}

/* V4L2 control operations */
static const struct v4l2_ctrl_ops kmb_flash_ctrl_ops = {
	.g_volatile_ctrl = kmb_flash_g_ctrl,
	.s_ctrl = kmb_flash_s_ctrl,
};

/**
 * kmb_flash_initialize_controls - Initialize handled flash controls
 * @kmb_flash: pointer to kmb flash device
 *
 * Return: 0 if successful
 */
static int kmb_flash_initialize_controls(struct kmb_flash *kmb_flash)
{
	struct kmb_flash_ctrls *ctrls = &kmb_flash->ctrls;
	struct v4l2_ctrl *fault;
	int ret;

	dev_dbg(kmb_flash->subdev.dev, "%s\n", __func__);

	v4l2_ctrl_handler_init(&ctrls->handler, 7);

	ctrls->led_mode = v4l2_ctrl_new_std_menu(
			&ctrls->handler, &kmb_flash_ctrl_ops,
			V4L2_CID_FLASH_LED_MODE,
			V4L2_FLASH_LED_MODE_TORCH, 0, V4L2_FLASH_LED_MODE_NONE);
	ctrls->strobe_source = v4l2_ctrl_new_std_menu(
			&ctrls->handler, &kmb_flash_ctrl_ops,
			V4L2_CID_FLASH_STROBE_SOURCE,
			V4L2_FLASH_STROBE_SOURCE_SOFTWARE, 0, 0);
	ctrls->strobe = v4l2_ctrl_new_std(
			&ctrls->handler, &kmb_flash_ctrl_ops,
			V4L2_CID_FLASH_STROBE,
			0, 0, 0, 0);
	ctrls->strobe_stop = v4l2_ctrl_new_std(
			&ctrls->handler, &kmb_flash_ctrl_ops,
			V4L2_CID_FLASH_STROBE_STOP,
			0, 0, 0, 0);
	ctrls->timeout = v4l2_ctrl_new_std(
			&ctrls->handler, &kmb_flash_ctrl_ops,
			V4L2_CID_FLASH_TIMEOUT,
			10, 500000, 10, 10000);
	ctrls->timeout->flags |= V4L2_CTRL_FLAG_VOLATILE |
			V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
	ctrls->intensity = v4l2_ctrl_new_std(
			&ctrls->handler, &kmb_flash_ctrl_ops,
			V4L2_CID_FLASH_INTENSITY,
			62500, 999999, 62500, 80000);
	ctrls->torch_intensity = v4l2_ctrl_new_std(
			&ctrls->handler, &kmb_flash_ctrl_ops,
			V4L2_CID_FLASH_TORCH_INTENSITY,
			31250, 999999, 31250, 40000);
	fault = v4l2_ctrl_new_std(&ctrls->handler, &kmb_flash_ctrl_ops,
			V4L2_CID_FLASH_FAULT, 0,
			V4L2_FLASH_FAULT_OVER_VOLTAGE
			| V4L2_FLASH_FAULT_OVER_TEMPERATURE
			| V4L2_FLASH_FAULT_SHORT_CIRCUIT
			| V4L2_FLASH_FAULT_TIMEOUT, 0, 0);

	if (fault != NULL)
		fault->flags |= V4L2_CTRL_FLAG_VOLATILE;

	if (ctrls->handler.error) {
		ret = ctrls->handler.error;
		v4l2_ctrl_handler_free(&ctrls->handler);
		dev_err(kmb_flash->subdev.dev, "%s %d ret = %d\n"
				, __func__, __LINE__, ret);
		return ret;
	}

	kmb_flash->subdev.ctrl_handler = &ctrls->handler;
	return 0;
}

/**
 * kmb_flash_s_stream - Set video stream stop/start
 * @sd: pointer to V4L2 sub-device
 * @enable: stream state
 *
 * Return: 0 if successful
 */
static int kmb_flash_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct kmb_flash *kmb_flash = v4l2_get_subdevdata(sd);

	dev_dbg(kmb_flash->subdev.dev, "%s\n", __func__);
	return 0;
}

/**
 * kmb_flash_set_power - Set power state helper function
 * @kmb_flash: pointer to kmb flash device
 * @on: power state
 *
 * Return: 0 if successful
 */
static int kmb_flash_set_power(struct kmb_flash *kmb_flash, bool on)
{
	dev_dbg(kmb_flash->subdev.dev, "%s: Set power to %d\n", __func__, on);
	return 0;
}

/**
 * kmb_flash_s_power - Put device in power saving or normal mode
 * @sd: pointer to V4L2 sub-device
 * @on: power state (0 - power saving mode, 1 - normal operation mode)
 *
 * Return: 0 if successful, else - error
 */
static int kmb_flash_s_power(struct v4l2_subdev *sd, int on)
{
	struct kmb_flash *kmb_flash = v4l2_get_subdevdata(sd);
	int ret;

	dev_dbg(kmb_flash->subdev.dev, "%s Power on %d\n", __func__, on);

	mutex_lock(&kmb_flash->power_lock);

	if (kmb_flash->power_on != on) {
		ret = kmb_flash_set_power(kmb_flash, on);
		if (!ret) {
			mutex_unlock(&kmb_flash->power_lock);
			return ret;
		}
	}

	kmb_flash->power_on = on;

	mutex_unlock(&kmb_flash->power_lock);

	return 0;
}

/* sub-device internal operations */
static const struct v4l2_subdev_internal_ops kmb_flash_sd_internal_ops = {
	.open = kmb_flash_open,
	.close = kmb_flash_close,
};

/* sub-device core operations */
static struct v4l2_subdev_core_ops kmb_flash_subdev_core_ops = {
	.s_power = kmb_flash_s_power,
};

/* sub-device video operations */
static struct v4l2_subdev_video_ops kmb_flash_subdev_video_ops = {
	.s_stream = kmb_flash_s_stream,
};

/* sub-device operations */
static const struct v4l2_subdev_ops kmb_flash_subdev_ops = {
	.core = &kmb_flash_subdev_core_ops,
	.video = &kmb_flash_subdev_video_ops,
};

/**
 * kmb_flash_probe - I2C client device binding
 * @client: pointer to i2c client device
 * @id: pointer to i2c device id
 *
 * Return: 0 if successful, -ENOMEM if there is not enough memory
 */
static int kmb_flash_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret = 0;
	struct v4l2_subdev *sd;
	struct kmb_flash *kmb_flash;

	dev_dbg(&client->dev, "%s: Probe KMB flash\n", __func__);

	kmb_flash = devm_kzalloc(&client->dev,
			sizeof(*kmb_flash), GFP_KERNEL);
	if (!kmb_flash)
		return -ENOMEM;

	mutex_init(&kmb_flash->lock);
	kmb_flash->client = client;

	kmb_flash->led_gpio = devm_gpiod_get_optional(&client->dev, "flash",
						      GPIOD_OUT_LOW);

	if (kmb_flash->led_gpio == NULL) {
		dev_err(&client->dev, "no flash has been detected");
	} else if (IS_ERR(kmb_flash->led_gpio)) {
		dev_err(&client->dev, "failed to request flash gpio %d", ret);
		return PTR_ERR(kmb_flash->led_gpio);
	}

	sd = &kmb_flash->subdev;
	v4l2_i2c_subdev_init(&kmb_flash->subdev, client,
			&kmb_flash_subdev_ops);
	v4l2_subdev_init(&kmb_flash->subdev, &kmb_flash_subdev_ops);
	strlcpy(sd->name, KMB_FLASH_DRV_NAME, sizeof(sd->name));
	sd->internal_ops = &kmb_flash_sd_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
				V4L2_SUBDEV_FL_HAS_EVENTS;
	sd->entity.function = MEDIA_ENT_F_FLASH;

	v4l2_set_subdevdata(sd, kmb_flash);

	kmb_flash->wq = alloc_workqueue("flash_work_queue", WQ_UNBOUND, 1);
	if (kmb_flash->wq == NULL) {
		ret = -ENOMEM;
		dev_err(&client->dev, "cannot create flash workqueue");
	}

	INIT_WORK(&kmb_flash->work_flash, kmb_flash_fire);
	atomic_set(&kmb_flash->duration, 0);

	ret = kmb_flash_initialize_controls(kmb_flash);
	if (ret < 0)
		goto error_media_entity_cleanup;

	v4l2_set_subdev_hostdata(sd, &kmb_flash->subdev);

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
 * kmb_flash_remove - I2C client device unbinding
 * @client: pointer to I2C client device
 *
 * Return: 0 if successful
 */
static int kmb_flash_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct kmb_flash *kmb_flash = v4l2_get_subdevdata(sd);

	dev_info(sd->dev, "%s: Remove KMB flash\n", __func__);

	mutex_destroy(&kmb_flash->lock);
	v4l2_async_unregister_subdev(sd);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	media_device_unregister_entity(&sd->entity);
	media_entity_cleanup(&sd->entity);

	return 0;
}

static const struct i2c_device_id kmb_flash_id_table[] = {
	{KMB_FLASH_DRV_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, kmb_flash_id_table);

static struct i2c_driver kmb_flash_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = KMB_FLASH_DRV_NAME,
	},
	.probe = kmb_flash_probe,
	.remove = kmb_flash_remove,
	.id_table = kmb_flash_id_table,
};

module_i2c_driver(kmb_flash_i2c_driver);

MODULE_DESCRIPTION("Keem Bay I2C Flash driver");
MODULE_LICENSE("GPL v2");
