// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 Intel Corporation.

#include <asm/unaligned.h>
#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/isx031.h>

#define to_isx031(_sd)			container_of(_sd, struct isx031, sd)

#define ISX031_REG_MODE_SELECT		0x8A01
#define ISX031_MODE_STANDBY		0x00
#define ISX031_MODE_STREAMING		0x80

#define ISX031_REG_CHIP_ID		0x6005
#define ISX031_CHIP_ID			0x05

struct isx031_reg {
	enum {
		ISX031_REG_LEN_DELAY = 0,
		ISX031_REG_LEN_08BIT = 1,
		ISX031_REG_LEN_16BIT = 2,
	} mode;
	u16 address;
	u16 val;
};

struct isx031_reg_list {
	u32 num_of_regs;
	const struct isx031_reg *regs;
};

struct isx031_link_freq_config {
	const struct isx031_reg_list reg_list;
};

struct isx031_mode {
	/* Frame width in pixels */
	u32 width;

	/* Frame height in pixels */
	u32 height;

	/* MEDIA_BUS_FMT */
	u32 code;

	/* MODE_FPS*/
	u32 fps;

	/* Sensor register settings for this resolution */
	const struct isx031_reg_list reg_list;
};

struct isx031 {
	struct v4l2_subdev sd;
	struct media_pad pad;

	/* Current mode */
	const struct isx031_mode *cur_mode;
	/* Previous mode */
	const struct isx031_mode *pre_mode;

	/* To serialize asynchronous callbacks */
	struct mutex mutex;

	/* i2c client */
	struct i2c_client *client;

	struct isx031_platform_data *platform_data;

	/* Streaming on/off */
	bool streaming;
};

static const struct isx031_reg isx031_init_reg[] = {
	{ISX031_REG_LEN_08BIT, 0xFFFF, 0x00}, // select mode
	{ISX031_REG_LEN_DELAY, 0x0000, 0x32}, // delay 50 ms
	{ISX031_REG_LEN_08BIT, 0x0171, 0x00}, // close F_EBD
	{ISX031_REG_LEN_08BIT, 0x0172, 0x00}, // close R_EBD
	{ISX031_REG_LEN_08BIT, 0x8A01, 0x00}, // mode transition to start-up state
	{ISX031_REG_LEN_DELAY, 0x0000, 0x64}, // delay 100 ms
};

static const struct isx031_reg isx031_1920_1536_30fps_reg[] = {
	{ISX031_REG_LEN_08BIT, 0x8AA8, 0x01}, // crop enable
	{ISX031_REG_LEN_08BIT, 0x8AAA, 0x80}, // H size = 1920
	{ISX031_REG_LEN_08BIT, 0x8AAB, 0x07},
	{ISX031_REG_LEN_08BIT, 0x8AAC, 0x00}, // H croped 0
	{ISX031_REG_LEN_08BIT, 0x8AAD, 0x00},
	{ISX031_REG_LEN_08BIT, 0x8AAE, 0x00}, // V size 1536
	{ISX031_REG_LEN_08BIT, 0x8AAF, 0x06},
	{ISX031_REG_LEN_08BIT, 0x8AB0, 0x00}, // V cropped 0
	{ISX031_REG_LEN_08BIT, 0x8AB1, 0x00},
	{ISX031_REG_LEN_08BIT, 0x8ADA, 0x03}, // DCROP_DATA_SEL
	{ISX031_REG_LEN_08BIT, 0xBF04, 0x01},
	{ISX031_REG_LEN_08BIT, 0xBF06, 0x80},
	{ISX031_REG_LEN_08BIT, 0xBF07, 0x07},
	{ISX031_REG_LEN_08BIT, 0xBF08, 0x00},
	{ISX031_REG_LEN_08BIT, 0xBF09, 0x00},
	{ISX031_REG_LEN_08BIT, 0xBF0A, 0x00},
	{ISX031_REG_LEN_08BIT, 0xBF0B, 0x06},
	{ISX031_REG_LEN_08BIT, 0xBF0C, 0x00},
	{ISX031_REG_LEN_08BIT, 0xBF0D, 0x00},
};

static const struct isx031_reg isx031_1920_1080_30fps_reg[] = {
	{ISX031_REG_LEN_08BIT, 0x8AA8, 0x01}, // crop enable
	{ISX031_REG_LEN_08BIT, 0x8AAA, 0x80}, // H size = 1920
	{ISX031_REG_LEN_08BIT, 0x8AAB, 0x07},
	{ISX031_REG_LEN_08BIT, 0x8AAC, 0x00}, // H croped 0
	{ISX031_REG_LEN_08BIT, 0x8AAD, 0x00},
	{ISX031_REG_LEN_08BIT, 0x8AAE, 0x38}, // V size 1080
	{ISX031_REG_LEN_08BIT, 0x8AAF, 0x04},
	{ISX031_REG_LEN_08BIT, 0x8AB0, 0xE4}, // V cropped 228*2
	{ISX031_REG_LEN_08BIT, 0x8AB1, 0x00},
	{ISX031_REG_LEN_08BIT, 0x8ADA, 0x03}, // DCROP_DATA_SEL
	{ISX031_REG_LEN_08BIT, 0xBF04, 0x01},
	{ISX031_REG_LEN_08BIT, 0xBF06, 0x80},
	{ISX031_REG_LEN_08BIT, 0xBF07, 0x07},
	{ISX031_REG_LEN_08BIT, 0xBF08, 0x00},
	{ISX031_REG_LEN_08BIT, 0xBF09, 0x00},
	{ISX031_REG_LEN_08BIT, 0xBF0A, 0x38},
	{ISX031_REG_LEN_08BIT, 0xBF0B, 0x04},
	{ISX031_REG_LEN_08BIT, 0xBF0C, 0xE4},
	{ISX031_REG_LEN_08BIT, 0xBF0D, 0x00},
};

static const struct isx031_reg isx031_1280_720_30fps_reg[] = {
	{ISX031_REG_LEN_08BIT, 0x8AA8, 0x01}, // crop enable
	{ISX031_REG_LEN_08BIT, 0x8AAA, 0x00}, // H size = 1280
	{ISX031_REG_LEN_08BIT, 0x8AAB, 0x05},
	{ISX031_REG_LEN_08BIT, 0x8AAC, 0x40}, // H croped 320*2
	{ISX031_REG_LEN_08BIT, 0x8AAD, 0x01},
	{ISX031_REG_LEN_08BIT, 0x8AAE, 0xD0}, // V size 720
	{ISX031_REG_LEN_08BIT, 0x8AAF, 0x02},
	{ISX031_REG_LEN_08BIT, 0x8AB0, 0x98}, // V cropped 408*2
	{ISX031_REG_LEN_08BIT, 0x8AB1, 0x01},
	{ISX031_REG_LEN_08BIT, 0x8ADA, 0x03}, // DCROP_DATA_SEL
	{ISX031_REG_LEN_08BIT, 0xBF04, 0x01},
	{ISX031_REG_LEN_08BIT, 0xBF06, 0x00},
	{ISX031_REG_LEN_08BIT, 0xBF07, 0x05},
	{ISX031_REG_LEN_08BIT, 0xBF08, 0x40},
	{ISX031_REG_LEN_08BIT, 0xBF09, 0x01},
	{ISX031_REG_LEN_08BIT, 0xBF0A, 0xD0},
	{ISX031_REG_LEN_08BIT, 0xBF0B, 0x02},
	{ISX031_REG_LEN_08BIT, 0xBF0C, 0x98},
	{ISX031_REG_LEN_08BIT, 0xBF0D, 0x01},
};

static const struct isx031_reg_list isx031_init_reg_list = {
	.num_of_regs = ARRAY_SIZE(isx031_init_reg),
	.regs = isx031_init_reg,
};

static const struct isx031_reg_list isx031_1920_1536_30fps_reg_list = {
	.num_of_regs = ARRAY_SIZE(isx031_1920_1536_30fps_reg),
	.regs = isx031_1920_1536_30fps_reg,
};

static const struct isx031_reg_list isx031_1920_1080_30fps_reg_list = {
	.num_of_regs = ARRAY_SIZE(isx031_1920_1080_30fps_reg),
	.regs = isx031_1920_1080_30fps_reg,
};

static const struct isx031_reg_list isx031_1280_720_30fps_reg_list = {
	.num_of_regs = ARRAY_SIZE(isx031_1280_720_30fps_reg),
	.regs = isx031_1280_720_30fps_reg,
};

static const struct isx031_mode supported_modes[] = {
	{
		.width = 1920,
		.height = 1080,
		.code = MEDIA_BUS_FMT_UYVY8_1X16,
		.fps = 30,
		.reg_list = isx031_1920_1080_30fps_reg_list,
	},
	{
		.width = 1280,
		.height = 720,
		.code = MEDIA_BUS_FMT_UYVY8_1X16,
		.fps = 30,
		.reg_list = isx031_1280_720_30fps_reg_list,
	},
	{
		.width = 1920,
		.height = 1536,
		.code = MEDIA_BUS_FMT_UYVY8_1X16,
		.fps = 30,
		.reg_list = isx031_1920_1536_30fps_reg_list,
	},
};

static int isx031_read_reg(struct isx031 *isx031, u16 reg, u16 len, u32 *val)
{
	struct i2c_client *client = isx031->client;
	struct i2c_msg msgs[2];
	u8 addr_buf[2];
	u8 data_buf[4] = {0};
	int ret;

	if (len > 4)
		return -EINVAL;

	put_unaligned_be16(reg, addr_buf);
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = sizeof(addr_buf);
	msgs[0].buf = addr_buf;
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_buf[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = get_unaligned_be32(data_buf);

	return 0;
}

static int isx031_write_reg(struct isx031 *isx031, u16 reg, u16 len, u32 val)
{
	struct i2c_client *client = isx031->client;
	u8 buf[6];

	if (len > 4)
		return -EINVAL;

	dev_dbg(&client->dev, "%s, reg %x len %x, val %x\n", __func__, reg, len, val);
	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val << 8 * (4 - len), buf + 2);
	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

static int isx031_write_reg_list(struct isx031 *isx031,
				 const struct isx031_reg_list *r_list)
{
	struct i2c_client *client = v4l2_get_subdevdata(&isx031->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < r_list->num_of_regs; i++) {
		if (r_list->regs[i].mode == ISX031_REG_LEN_DELAY) {
			msleep(r_list->regs[i].val);
			continue;
		}
		ret = isx031_write_reg(isx031, r_list->regs[i].address,
				       ISX031_REG_LEN_08BIT,
				       r_list->regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
				"failed to write reg 0x%4.4x. error = %d",
				r_list->regs[i].address, ret);
			return ret;
		}
	}

	return 0;
}

static void isx031_update_pad_format(const struct isx031_mode *mode,
				     struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = mode->code;
	fmt->field = V4L2_FIELD_NONE;
}

static int isx031_start_streaming(struct isx031 *isx031)
{
	int ret;
	struct i2c_client *client = isx031->client;
	const struct isx031_reg_list *reg_list;

	if (isx031->cur_mode != isx031->pre_mode) {
		reg_list = &isx031->cur_mode->reg_list;
		ret = isx031_write_reg_list(isx031, reg_list);
		if (ret) {
			dev_err(&client->dev, "failed to set stream mode");
			return ret;
		}
		isx031->pre_mode = isx031->cur_mode;
	} else {
		dev_dbg(&client->dev, "same mode, skip write reg list");
	}

	ret = isx031_write_reg(isx031, ISX031_REG_MODE_SELECT, 1,
			       ISX031_MODE_STREAMING);

	if (ret) {
		dev_err(&client->dev, "failed to start stream");
		return ret;
	}

	return 0;
}

static void isx031_stop_streaming(struct isx031 *isx031)
{
	struct i2c_client *client = isx031->client;

	if (isx031_write_reg(isx031, ISX031_REG_MODE_SELECT, 1,
			     ISX031_MODE_STANDBY))
		dev_err(&client->dev, "failed to stop stream");
	msleep(50);
}

static int isx031_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct isx031 *isx031 = to_isx031(sd);
	struct i2c_client *client = isx031->client;
	int ret = 0;

	if (isx031->streaming == enable)
		return 0;

	mutex_lock(&isx031->mutex);
	if (enable) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			mutex_unlock(&isx031->mutex);
			return ret;
		}

		ret = isx031_start_streaming(isx031);
		if (ret) {
			enable = 0;
			isx031_stop_streaming(isx031);
			pm_runtime_put(&client->dev);
		}
	} else {
		isx031_stop_streaming(isx031);
		pm_runtime_put(&client->dev);
	}

	isx031->streaming = enable;

	mutex_unlock(&isx031->mutex);

	return ret;
}

static int __maybe_unused isx031_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct isx031 *isx031 = to_isx031(sd);

	mutex_lock(&isx031->mutex);
	if (isx031->streaming)
		isx031_stop_streaming(isx031);

	mutex_unlock(&isx031->mutex);

	return 0;
}

static int __maybe_unused isx031_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct isx031 *isx031 = to_isx031(sd);
	int ret;

	mutex_lock(&isx031->mutex);
	if (isx031->streaming) {
		ret = isx031_start_streaming(isx031);
		if (ret) {
			isx031->streaming = false;
			isx031_stop_streaming(isx031);
			mutex_unlock(&isx031->mutex);
			return ret;
		}
	}

	mutex_unlock(&isx031->mutex);

	return 0;
}

static int isx031_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *sd_state,
			     struct v4l2_subdev_format *fmt)
{
	struct isx031 *isx031 = to_isx031(sd);
	const struct isx031_mode *mode;
	int ret = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++)
		if (supported_modes[i].code == fmt->format.code &&
		    supported_modes[i].width == fmt->format.width &&
		    supported_modes[i].height == fmt->format.height) {
			mode = &supported_modes[i];
			break;
		}

	if (i >= ARRAY_SIZE(supported_modes))
		mode = &supported_modes[0];

	mutex_lock(&isx031->mutex);

	isx031_update_pad_format(mode, &fmt->format);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		*v4l2_subdev_get_try_format(sd, sd_state, fmt->pad) = fmt->format;
	else
		isx031->cur_mode = mode;


	if (isx031->cur_mode != isx031->pre_mode) {
		ret = isx031_write_reg_list(isx031, &isx031->cur_mode->reg_list);
		if (ret)
			dev_err(&isx031->client->dev, "failed to set stream mode");
		else
			isx031->pre_mode = isx031->cur_mode;
	}

	mutex_unlock(&isx031->mutex);

	return 0;
}

static int isx031_get_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *sd_state,
			     struct v4l2_subdev_format *fmt)
{
	struct isx031 *isx031 = to_isx031(sd);

	mutex_lock(&isx031->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt->format = *v4l2_subdev_get_try_format(&isx031->sd, sd_state,
							  fmt->pad);
	else
		isx031_update_pad_format(isx031->cur_mode, &fmt->format);

	mutex_unlock(&isx031->mutex);

	return 0;
}

static int isx031_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct isx031 *isx031 = to_isx031(sd);

	mutex_lock(&isx031->mutex);
	isx031_update_pad_format(&supported_modes[0],
				 v4l2_subdev_get_try_format(sd, fh->state, 0));
	mutex_unlock(&isx031->mutex);

	return 0;
}

static const struct v4l2_subdev_video_ops isx031_video_ops = {
	.s_stream = isx031_set_stream,
};

static const struct v4l2_subdev_pad_ops isx031_pad_ops = {
	.set_fmt = isx031_set_format,
	.get_fmt = isx031_get_format,
};

static const struct v4l2_subdev_ops isx031_subdev_ops = {
	.video = &isx031_video_ops,
	.pad = &isx031_pad_ops,
};

static const struct media_entity_operations isx031_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops isx031_internal_ops = {
	.open = isx031_open,
};

static int isx031_identify_module(struct isx031 *isx031)
{
	struct i2c_client *client = isx031->client;
	int ret;
	u32 val;

	ret = isx031_read_reg(isx031, ISX031_REG_CHIP_ID,
			      ISX031_REG_LEN_08BIT, &val);
	if (ret)
		return ret;
	/* chip id not known yet */
	if (val != ISX031_CHIP_ID) {
		dev_err(&client->dev, "read chip id: %x != %x",
			val, ISX031_CHIP_ID);
		return -ENXIO;
	}

	return isx031_write_reg_list(isx031, &isx031_init_reg_list);
}

static void isx031_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct isx031 *isx031 = to_isx031(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	pm_runtime_disable(&client->dev);
	mutex_destroy(&isx031->mutex);

}

static int isx031_probe(struct i2c_client *client)
{
	struct v4l2_subdev *sd;
	struct isx031 *isx031;
	const struct isx031_reg_list *reg_list;
	int ret;

	isx031 = devm_kzalloc(&client->dev, sizeof(*isx031), GFP_KERNEL);
	if (!isx031)
		return -ENOMEM;

	isx031->client = client;
	isx031->platform_data = client->dev.platform_data;
	if (isx031->platform_data == NULL) {
		dev_err(&client->dev, "no platform data provided\n");
		return -EINVAL;
	}

	/* initialize subdevice */
	sd = &isx031->sd;
	v4l2_i2c_subdev_init(sd, client, &isx031_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	sd->internal_ops = &isx031_internal_ops;
	sd->entity.ops = &isx031_subdev_entity_ops;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* initialize subdev media pad */
	isx031->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, 1, &isx031->pad);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s : media entity init Failed %d\n", __func__, ret);
		return ret;
	}

	ret = isx031_identify_module(isx031);
	if (ret) {
		dev_err(&client->dev, "failed to find sensor: %d", ret);
		return ret;
	}

	if (isx031->platform_data->suffix)
		snprintf(isx031->sd.name, sizeof(isx031->sd.name), "isx031 %c",
			 isx031->platform_data->suffix);

	mutex_init(&isx031->mutex);

	/* 1920x1536 default */
	isx031->cur_mode = NULL;
	isx031->pre_mode = &supported_modes[0];
	reg_list = &isx031->pre_mode->reg_list;
	ret = isx031_write_reg_list(isx031, reg_list);
	if (ret) {
		dev_err(&client->dev, "failed to apply preset mode");
		goto probe_error_media_entity_cleanup;
	}
	isx031->cur_mode = isx031->pre_mode;
	ret = v4l2_async_register_subdev_sensor(&isx031->sd);
	if (ret < 0) {
		dev_err(&client->dev, "failed to register V4L2 subdev: %d",
			ret);
		goto probe_error_media_entity_cleanup;
	}

	/*
	 * Device is already turned on by i2c-core with ACPI domain PM.
	 * Enable runtime PM and turn off the device.
	 */
	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	return 0;

probe_error_media_entity_cleanup:
	media_entity_cleanup(&isx031->sd.entity);
	pm_runtime_disable(&client->dev);
	mutex_destroy(&isx031->mutex);

	return ret;
}

static const struct dev_pm_ops isx031_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(isx031_suspend, isx031_resume)
};

static const struct i2c_device_id isx031_id_table[] = {
	{ "isx031", 0 },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(i2c, isx031_id_table);

static struct i2c_driver isx031_i2c_driver = {
	.driver = {
		.name = "isx031",
		.pm = &isx031_pm_ops,
	},
	.probe = isx031_probe,
	.remove = isx031_remove,
	.id_table = isx031_id_table,
};

module_i2c_driver(isx031_i2c_driver);

MODULE_AUTHOR("Hao Yao <hao.yao@intel.com>");
MODULE_DESCRIPTION("isx031 sensor driver");
MODULE_LICENSE("GPL v2");
