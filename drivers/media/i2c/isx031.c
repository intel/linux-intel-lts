// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022-2025 Intel Corporation.

#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0)
#include <asm/unaligned.h>
#else
#include <linux/unaligned.h>
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 10, 0)
#include <media/mipi-csi2.h>
#endif
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#include "media/i2c/isx031.h"

#define to_isx031(_sd)	container_of(_sd, struct isx031, sd)

#define ISX031_OTP_TYPE_NAME_L		0x7E8A
#define ISX031_OTP_TYPE_NAME_H		0x7E8B
#define ISX031_OTP_TYPE_NAME_H_FIELD	0x0F
#define ISX031_OTP_MODULE_ID_L		0x031

#define ISX031_REG_MODE_SET_F		0x8A01
#define ISX031_MODE_STANDBY		0x00
#define ISX031_MODE_STREAMING		0x80

#define ISX031_REG_SENSOR_STATE		0x6005
#define ISX031_STATE_STREAMING		0x05
#define ISX031_STATE_STARTUP		0x02

#define ISX031_REG_MODE_SET_F_LOCK	0xBEF0
#define ISX031_MODE_UNLOCK		0x53

#define ISX031_REG_MODE_SELECT		0x8A00
#define ISX031_MODE_4LANES_60FPS	0x01
#define ISX031_MODE_4LANES_30FPS	0x17
#define ISX031_MODE_2LANES_30FPS	0x18

#define ISX031_READ_REG_RETRY_TIMEOUT	50
#define ISX031_WRITE_REG_RETRY_TIMEOUT	100
#define ISX031_PM_RETRY_TIMEOUT		10
#define ISX031_REG_SLEEP_10000US	10000	/* 10ms */
#define ISX031_REG_SLEEP_20MS		20	/* 20ms */
#define ISX031_REG_SLEEP_200MS		200	/* 200ms */

/* To serialize asynchronous callbacks */
static DEFINE_MUTEX(isx031_mutex);

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

struct isx031_drive_mode {
	int lanes;
	int fps;
	int mode;
};

static const struct isx031_drive_mode isx031_drive_modes[] = {
	{ 4, 60, ISX031_MODE_4LANES_60FPS },
	{ 4, 30, ISX031_MODE_4LANES_30FPS },
	{ 2, 30, ISX031_MODE_2LANES_30FPS },
	{}
};

struct isx031_mode {
	u32 width;	/* Frame width in pixels */
	u32 height;	/* Frame height in pixels */
	u32 code;	/* MEDIA_BUS_FMT */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 10, 0)
	u8 datatype;	/* CSI-2 data type ID */
#endif
	u32 fps;	/* MODE_FPS */

	/* Sensor register settings for a specific resolution */
	const struct isx031_reg_list reg_list;
};

struct isx031 {
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler ctrls;

	isx031_platform_data *platform_data;
	struct i2c_client *client;

	struct gpio_desc *reset_gpio;
	struct gpio_desc *fsin_gpio;
	struct media_pad pad;

	const struct isx031_mode *cur_mode;	/* Current mode */
	const struct isx031_mode *pre_mode;	/* Previous mode */

	u8 lanes;
	bool streaming;	/* Streaming on/off */
};

static const s64 isx031_link_frequencies[] = {
	300000000ULL,
};

static const struct isx031_reg isx031_init_reg[] = {
	{ISX031_REG_LEN_08BIT, 0xFFFF, 0x00}, /* Select mode */
	{ISX031_REG_LEN_08BIT, 0x0171, 0x00}, /* Close F_EBD */
	{ISX031_REG_LEN_08BIT, 0x0172, 0x00}, /* Close R_EBD */
	{}
};

static const struct isx031_reg isx031_framesync_reg[] = {
	{ISX031_REG_LEN_08BIT, 0xBF14, 0x01}, /* SG_MODE_APL */
	{ISX031_REG_LEN_08BIT, 0x8AFF, 0x0c}, /* Hi-Z (input setting or output disabled) */
	{ISX031_REG_LEN_08BIT, 0x0153, 0x00},
	{ISX031_REG_LEN_08BIT, 0x8AF0, 0x01}, /* External pulse-based sync */
	{ISX031_REG_LEN_08BIT, 0x0144, 0x00},
	{ISX031_REG_LEN_08BIT, 0x8AF1, 0x00},
	{}
};

static const struct isx031_reg isx031_1920_1536_30fps_reg[] = {
	{ISX031_REG_LEN_08BIT, 0x8AA8, 0x01}, /* Crop enable */
	{ISX031_REG_LEN_08BIT, 0x8AAA, 0x80}, /* H size = 1920 */
	{ISX031_REG_LEN_08BIT, 0x8AAB, 0x07},
	{ISX031_REG_LEN_08BIT, 0x8AAC, 0x00}, /* H croped 0 */
	{ISX031_REG_LEN_08BIT, 0x8AAD, 0x00},
	{ISX031_REG_LEN_08BIT, 0x8AAE, 0x00}, /* V size 1536 */
	{ISX031_REG_LEN_08BIT, 0x8AAF, 0x06},
	{ISX031_REG_LEN_08BIT, 0x8AB0, 0x00}, /* V cropped 0 */
	{ISX031_REG_LEN_08BIT, 0x8AB1, 0x00},
	{ISX031_REG_LEN_08BIT, 0x8ADA, 0x03}, /* DCROP_DATA_SEL */
	{ISX031_REG_LEN_08BIT, 0xBF04, 0x01},
	{ISX031_REG_LEN_08BIT, 0xBF06, 0x80},
	{ISX031_REG_LEN_08BIT, 0xBF07, 0x07},
	{ISX031_REG_LEN_08BIT, 0xBF08, 0x00},
	{ISX031_REG_LEN_08BIT, 0xBF09, 0x00},
	{ISX031_REG_LEN_08BIT, 0xBF0A, 0x00},
	{ISX031_REG_LEN_08BIT, 0xBF0B, 0x06},
	{ISX031_REG_LEN_08BIT, 0xBF0C, 0x00},
	{ISX031_REG_LEN_08BIT, 0xBF0D, 0x00},
	{}
};

static const struct isx031_reg isx031_1920_1080_30fps_reg[] = {
	{ISX031_REG_LEN_08BIT, 0x8AA8, 0x01}, /* Crop enable */
	{ISX031_REG_LEN_08BIT, 0x8AAA, 0x80}, /* H size = 1920 */
	{ISX031_REG_LEN_08BIT, 0x8AAB, 0x07},
	{ISX031_REG_LEN_08BIT, 0x8AAC, 0x00}, /* H croped 0 */
	{ISX031_REG_LEN_08BIT, 0x8AAD, 0x00},
	{ISX031_REG_LEN_08BIT, 0x8AAE, 0x38}, /* V size 1080 */
	{ISX031_REG_LEN_08BIT, 0x8AAF, 0x04},
	{ISX031_REG_LEN_08BIT, 0x8AB0, 0xE4}, /* V cropped 228*2 */
	{ISX031_REG_LEN_08BIT, 0x8AB1, 0x00},
	{ISX031_REG_LEN_08BIT, 0x8ADA, 0x03}, /* DCROP_DATA_SEL */
	{ISX031_REG_LEN_08BIT, 0xBF04, 0x01},
	{ISX031_REG_LEN_08BIT, 0xBF06, 0x80},
	{ISX031_REG_LEN_08BIT, 0xBF07, 0x07},
	{ISX031_REG_LEN_08BIT, 0xBF08, 0x00},
	{ISX031_REG_LEN_08BIT, 0xBF09, 0x00},
	{ISX031_REG_LEN_08BIT, 0xBF0A, 0x38},
	{ISX031_REG_LEN_08BIT, 0xBF0B, 0x04},
	{ISX031_REG_LEN_08BIT, 0xBF0C, 0xE4},
	{ISX031_REG_LEN_08BIT, 0xBF0D, 0x00},
	{}
};

static const struct isx031_reg isx031_1280_720_30fps_reg[] = {
	{ISX031_REG_LEN_08BIT, 0x8AA8, 0x01}, /* Crop enable */
	{ISX031_REG_LEN_08BIT, 0x8AAA, 0x00}, /* H size = 1280 */
	{ISX031_REG_LEN_08BIT, 0x8AAB, 0x05},
	{ISX031_REG_LEN_08BIT, 0x8AAC, 0x40}, /* H croped 320*2 */
	{ISX031_REG_LEN_08BIT, 0x8AAD, 0x01},
	{ISX031_REG_LEN_08BIT, 0x8AAE, 0xD0}, /* V size 720 */
	{ISX031_REG_LEN_08BIT, 0x8AAF, 0x02},
	{ISX031_REG_LEN_08BIT, 0x8AB0, 0x98}, /* V cropped 408*2 */
	{ISX031_REG_LEN_08BIT, 0x8AB1, 0x01},
	{ISX031_REG_LEN_08BIT, 0x8ADA, 0x03}, /* DCROP_DATA_SEL */
	{ISX031_REG_LEN_08BIT, 0xBF04, 0x01},
	{ISX031_REG_LEN_08BIT, 0xBF06, 0x00},
	{ISX031_REG_LEN_08BIT, 0xBF07, 0x05},
	{ISX031_REG_LEN_08BIT, 0xBF08, 0x40},
	{ISX031_REG_LEN_08BIT, 0xBF09, 0x01},
	{ISX031_REG_LEN_08BIT, 0xBF0A, 0xD0},
	{ISX031_REG_LEN_08BIT, 0xBF0B, 0x02},
	{ISX031_REG_LEN_08BIT, 0xBF0C, 0x98},
	{ISX031_REG_LEN_08BIT, 0xBF0D, 0x01},
	{}
};

static const struct isx031_reg_list isx031_init_reg_list = {
	.num_of_regs = ARRAY_SIZE(isx031_init_reg),
	.regs = isx031_init_reg,
};

static const struct isx031_reg_list isx031_framesync_reg_list = {
	.num_of_regs = ARRAY_SIZE(isx031_framesync_reg),
	.regs = isx031_framesync_reg,
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
		.width		= 1920,
		.height		= 1536,
		.code		= MEDIA_BUS_FMT_UYVY8_1X16,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 10, 0)
		.datatype	= MIPI_CSI2_DT_YUV422_8B,
#endif
		.fps		= 30,
		.reg_list	= isx031_1920_1536_30fps_reg_list,
	},
	{
		.width		= 1920,
		.height		= 1080,
		.code		= MEDIA_BUS_FMT_UYVY8_1X16,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 10, 0)
		.datatype	= MIPI_CSI2_DT_YUV422_8B,
#endif
		.fps		= 30,
		.reg_list	= isx031_1920_1080_30fps_reg_list,
	},
	{
		.width		= 1280,
		.height		= 720,
		.code		= MEDIA_BUS_FMT_UYVY8_1X16,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 10, 0)
		.datatype	= MIPI_CSI2_DT_YUV422_8B,
#endif
		.fps		= 30,
		.reg_list	= isx031_1280_720_30fps_reg_list,
	},
};

static int isx031_read_reg(struct i2c_client *client, u16 reg, u16 len, u32 *val)
{
	struct i2c_msg msgs[2];
	u8 addr_buf[2];
	u8 data_buf[4] = { 0 };
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

static int isx031_read_reg_state(struct i2c_client *client, u32 *val)
{
	int ret;
	int i;

	for (i = 0; i < ISX031_READ_REG_RETRY_TIMEOUT; i++) {
		ret = isx031_read_reg(client, ISX031_REG_SENSOR_STATE,
				      ISX031_REG_LEN_08BIT, val);
		if (!ret)
			return 0;

		usleep_range(ISX031_REG_SLEEP_10000US,
			     ISX031_REG_SLEEP_10000US + 500);
	}

	return ret;
}

static int isx031_read_reg_otp(struct i2c_client *client, u16 reg, u32 *val)
{
	int ret;
	int i;

	for (i = 0; i < ISX031_READ_REG_RETRY_TIMEOUT; i++) {
		ret = isx031_read_reg(client, reg, ISX031_REG_LEN_08BIT, val);
		if (!ret)
			return 0;

		usleep_range(ISX031_REG_SLEEP_10000US,
			     ISX031_REG_SLEEP_10000US + 500);
	}

	return ret;
}

static int isx031_write_reg(struct i2c_client *client, u16 reg, u16 len, u32 val)
{
	u8 buf[6];
	int ret;

	if (len > 4)
		return -EINVAL;

	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val << (8 * (4 - len)), buf + 2);

	ret = i2c_master_send(client, buf, len + 2);
	if (ret != len + 2)
		return -EIO;

	return 0;
}

static int isx031_write_reg_retry(struct i2c_client *client, u16 reg, u16 len, u32 val)
{
	int ret;
	int i;

	for (i = 0; i < ISX031_WRITE_REG_RETRY_TIMEOUT; i++) {
		ret = isx031_write_reg(client, reg, len, val);
		if (!ret)
			return 0;

		msleep(ISX031_REG_SLEEP_20MS);
	}

	return ret;
}

static int isx031_write_reg_list(struct i2c_client *client,
				 const struct isx031_reg_list *r_list,
				 bool is_retry)
{
	unsigned int i;
	int ret;

	for (i = 0; i < r_list->num_of_regs; i++) {
		const struct isx031_reg *reg = &r_list->regs[i];

		if (reg->mode == ISX031_REG_LEN_DELAY) {
			msleep(reg->val);
			continue;
		}

		if (is_retry)
			ret = isx031_write_reg_retry(client, reg->address,
						     ISX031_REG_LEN_08BIT,
						     reg->val);
		else
			ret = isx031_write_reg(client, reg->address,
					       ISX031_REG_LEN_08BIT,
					       reg->val);

		if (ret) {
			dev_err_ratelimited(&client->dev,
					    "write reg failed (addr=0x%04x, err=%d)\n",
					    reg->address, ret);
			return ret;
		}
	}

	return 0;
}

static int isx031_find_drive_mode(int lanes, int fps)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(isx031_drive_modes); i++) {
		if (isx031_drive_modes[i].lanes == lanes &&
		    isx031_drive_modes[i].fps == fps)
			return isx031_drive_modes[i].mode;
	}

	return -EINVAL;
}

static int isx031_set_drive_mode(struct isx031 *isx031)
{
	struct i2c_client *client = isx031->client;
	int mode, ret;

	mode = isx031_find_drive_mode(isx031->lanes, isx031->cur_mode->fps);
	if (mode < 0) {
		dev_err(&client->dev, "Failed to find drive mode\n");
		return mode;
	}

	ret = isx031_write_reg(client, ISX031_REG_MODE_SELECT, 1, mode);

	return ret;
}

static int isx031_mode_transit(struct isx031 *isx031, int state)
{
	struct i2c_client *client = isx031->client;
	int ret;
	int cur_mode, mode = ISX031_MODE_STANDBY;
	u32 val = 0;

	if (state == ISX031_STATE_STARTUP)
		mode = ISX031_MODE_STANDBY;
	else if (state == ISX031_STATE_STREAMING)
		mode = ISX031_MODE_STREAMING;
	else
		return -EINVAL;

	ret = isx031_read_reg_state(client, &val);
	if (ret) {
		dev_err(&client->dev, "Failed to read sensor state\n");
		return ret;
	}
	cur_mode = val;

	ret = isx031_set_drive_mode(isx031);
	if (ret) {
		dev_err(&client->dev, "Failed to set drive mode\n");
		return ret;
	}

	ret = isx031_write_reg(client, ISX031_REG_MODE_SET_F_LOCK, 1,
			       ISX031_MODE_UNLOCK);
	if (ret) {
		dev_err(&client->dev, "Failed to unlock mode\n");
		return ret;
	}

	ret = isx031_write_reg(client, ISX031_REG_MODE_SET_F, 1, mode);
	if (ret) {
		dev_err(&client->dev, "Failed to transit mode from 0x%x to 0x%x\n",
			cur_mode, mode);
		return ret;
	}

	ret = isx031_read_reg_state(client, &val);
	if (ret) {
		dev_err(&client->dev, "Failed to read sensor state\n");
		return ret;
	}

	return 0;
}

static int isx031_initialize_module(struct isx031 *isx031)
{
	struct i2c_client *client = isx031->client;
	int ret;
	u32 val = 0;

	/* Read sensor current state */
	ret = isx031_read_reg_state(client, &val);
	if (ret) {
		dev_err(&client->dev, "Failed to read sensor state\n");
		return ret;
	}

	/* If sensor is streaming, transition to startup before initialization */
	if (val == ISX031_STATE_STREAMING) {
		ret = isx031_mode_transit(isx031, ISX031_STATE_STARTUP);
		if (ret)
			return ret;
	}

	ret = isx031_write_reg_list(client, &isx031_init_reg_list, true);
	if (ret)
		return ret;

	if (isx031->platform_data &&
	    !isx031->platform_data->irq_pin_flags) {
		ret = isx031_write_reg_list(client, &isx031_framesync_reg_list, false);
		if (ret) {
			dev_err(&client->dev, "Failed to set framesync\n");
			return ret;
		}
	}

	return 0;
}

static int isx031_identify_module(struct i2c_client *client)
{
	u32 name_l = 0;
	u32 name_h = 0;
	u16 module_id;
	int ret;

	ret = isx031_read_reg_otp(client, ISX031_OTP_TYPE_NAME_L, &name_l);
	if (ret) {
		dev_err(&client->dev, "Failed to read OTP NAME_L register\n");
		return ret;
	}

	ret = isx031_read_reg_otp(client, ISX031_OTP_TYPE_NAME_H, &name_h);
	if (ret) {
		dev_err(&client->dev, "Failed to read OTP NAME_H register\n");
		return ret;
	}

	module_id = ((name_h & ISX031_OTP_TYPE_NAME_H_FIELD) << 8) | name_l;
	if (module_id != ISX031_OTP_MODULE_ID_L) {
		dev_err(&client->dev,
			"Invalid module ID: expected 0x%04x, got 0x%04x\n",
			ISX031_OTP_MODULE_ID_L, module_id);
		return -ENODEV;
	}

	return 0;
}

static void isx031_update_pad_format(const struct isx031_mode *mode,
				     struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = mode->code;
	fmt->field = V4L2_FIELD_ANY;
}

static int isx031_get_num_lane(struct isx031 *isx031, struct device *dev)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	int ret;

	endpoint =
		fwnode_graph_get_endpoint_by_id(dev_fwnode(dev),
						0, 0,
						FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!endpoint) {
		dev_err(dev, "No endpoint node found\n");
		return -EPROBE_DEFER;
	}

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &bus_cfg);
	if (ret) {
		dev_err(dev, "Failed to parse endpoint node: %d\n", ret);
		goto out_err;
	}

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != 2 &&
		bus_cfg.bus.mipi_csi2.num_data_lanes != 4) {
		dev_err(dev, "Only 2 or 4 data lanes are supported\n");
		ret = -EINVAL;
		goto out_err;
	}

	isx031->lanes = bus_cfg.bus.mipi_csi2.num_data_lanes;

out_err:
	v4l2_fwnode_endpoint_free(&bus_cfg);
	fwnode_handle_put(endpoint);
	return ret;
}

static int isx031_start_streaming(struct isx031 *isx031)
{
	struct i2c_client *client = isx031->client;
	const struct isx031_reg_list *reg_list;
	int ret;

	/* Apply mode registers only if mode changed */
	if (isx031->cur_mode != isx031->pre_mode) {
		reg_list = &isx031->cur_mode->reg_list;
		ret = isx031_write_reg_list(client, reg_list, true);
		if (ret) {
			dev_err(&client->dev, "Failed to set stream mode\n");
			return ret;
		}
		isx031->pre_mode = isx031->cur_mode;
	}

	ret = __v4l2_ctrl_handler_setup(&isx031->ctrls);
	if (ret) {
		dev_err(&client->dev, "Failed to setup controls\n");
		return ret;
	}

	ret = isx031_mode_transit(isx031, ISX031_STATE_STREAMING);
	if (ret) {
		dev_err(&client->dev, "Failed to start streaming\n");
		return ret;
	}

	return 0;
}

static void isx031_stop_streaming(struct isx031 *isx031)
{
	struct i2c_client *client = isx031->client;
	int ret;

	ret = isx031_mode_transit(isx031, ISX031_STATE_STARTUP);
	if (ret)
		dev_err(&client->dev, "Failed to stop streaming: %d\n", ret);
}

static int isx031_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct isx031 *isx031 = to_isx031(sd);
	struct i2c_client *client = isx031->client;
	int ret = 0;

	mutex_lock(&isx031_mutex);

	if (isx031->streaming == enable)
		goto unlock;

	if (enable) {
		ret = pm_runtime_resume_and_get(&client->dev);
		if (ret < 0)
			goto unlock;

		ret = isx031_start_streaming(isx031);
		if (ret) {
			isx031_stop_streaming(isx031);
			pm_runtime_put(&client->dev);
			goto unlock;
		}

		isx031->streaming = true;

	} else {
		isx031_stop_streaming(isx031);
		pm_runtime_put(&client->dev);
		isx031->streaming = false;
	}

unlock:
	mutex_unlock(&isx031_mutex);

	return ret;
}

static int isx031_enable_streams(struct v4l2_subdev *subdev,
				 struct v4l2_subdev_state *state,
				 u32 pad, u64 streams_mask)
{
	return isx031_set_stream(subdev, true);
}

static int isx031_disable_streams(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_state *state,
				  u32 pad, u64 streams_mask)
{
	return isx031_set_stream(subdev, false);
}

static int __maybe_unused isx031_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct isx031 *isx031 = to_isx031(sd);

	mutex_lock(&isx031_mutex);

	if (isx031->streaming)
		isx031_stop_streaming(isx031);

	mutex_unlock(&isx031_mutex);

	/* Active low gpio reset, set 1 to power off sensor */
	if (isx031->reset_gpio)
		gpiod_set_value_cansleep(isx031->reset_gpio, 1);

	return 0;
}

static int __maybe_unused isx031_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct isx031 *isx031 = to_isx031(sd);
	const struct isx031_reg_list *reg_list;
	int ret;
	int count;

	mutex_lock(&isx031_mutex);

	/* Active low gpio reset, set 0 to power on sensor,
	 * sensor must be on before resume
	 */
	if (isx031->reset_gpio) {
		for (count = 0; count < ISX031_PM_RETRY_TIMEOUT; count++) {
			gpiod_set_value_cansleep(isx031->reset_gpio, 0);
			msleep(ISX031_REG_SLEEP_200MS);

			ret = gpiod_get_value_cansleep(isx031->reset_gpio);
			if (ret == 0)
				break;
		}

		if (ret != 0) {
			dev_err(&client->dev, "Failed to power on sensor in pm resume\n");
			mutex_unlock(&isx031_mutex);
			return -ETIMEDOUT;
		}
	}
	/* S4 will clear the GPIO BIAS and CONFIG
	 * set fsin gpio output to trigger set_direction and set_config
	 * then set fsin to 0 to turn GPIO active */
	if (isx031->fsin_gpio) {
		gpiod_direction_output(isx031->fsin_gpio, 0);

		for (count = 0; count < ISX031_PM_RETRY_TIMEOUT; count++) {
			gpiod_set_value_cansleep(isx031->fsin_gpio, 0);
			msleep(ISX031_REG_SLEEP_200MS);

			ret = gpiod_get_value_cansleep(isx031->fsin_gpio);
			if (ret == 0)
				break;
		}

		if (ret != 0) {
			dev_err(&client->dev, "Failed to turn on fsin in pm resume\n");
			mutex_unlock(&isx031_mutex);
			return -ETIMEDOUT;
		}
	}

	ret = isx031_identify_module(client);
	if (ret) {
		dev_err(&client->dev, "Failed to identify sensor module: %d\n", ret);
		goto unlock;
	}

	ret = isx031_initialize_module(isx031);
	if (ret) {
		dev_err(&client->dev, "Failed to initialize sensor module: %d\n", ret);
		goto unlock;
	}

	reg_list = &isx031->cur_mode->reg_list;
	ret = isx031_write_reg_list(client, reg_list, true);
	if (ret) {
		dev_err(&client->dev, "Failed to apply cur mode in resume: %d\n", ret);
		goto unlock;
	}

	if (isx031->streaming) {
		ret = isx031_start_streaming(isx031);
		if (ret) {
			isx031->streaming = false;
			isx031_stop_streaming(isx031);
			goto unlock;
		}
	}

unlock:
	mutex_unlock(&isx031_mutex);

	return ret;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 10, 0)
static int isx031_get_frame_desc(struct v4l2_subdev *sd,
				 unsigned int pad,
				 struct v4l2_mbus_frame_desc *desc)
{
	struct isx031 *isx031 = to_isx031(sd);

	desc->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	desc->num_entries = 0;
	desc->entry[desc->num_entries].flags = V4L2_MBUS_FRAME_DESC_FL_LEN_MAX;
	desc->entry[desc->num_entries].stream = 0;
	desc->entry[desc->num_entries].pixelcode = isx031->cur_mode->code;
	desc->entry[desc->num_entries].length = 0;
	desc->entry[desc->num_entries].bus.csi2.vc = 0;
	desc->entry[desc->num_entries].bus.csi2.dt = isx031->cur_mode->datatype;
	desc->num_entries++;
	return 0;
}
#else
static int isx031_get_frame_desc(struct v4l2_subdev *sd,
				 unsigned int pad,
				 struct v4l2_mbus_frame_desc *desc)
{
	unsigned int i;

	desc->num_entries = V4L2_FRAME_DESC_ENTRY_MAX;

	for (i = 0; i < desc->num_entries; i++) {
		desc->entry[i].flags = 0;
		desc->entry[i].pixelcode = MEDIA_BUS_FMT_FIXED;
		desc->entry[i].length = 0;
	}

	return 0;
}
#endif

static int isx031_set_format(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 14, 0)
			     struct v4l2_subdev_pad_config *cfg,
#else
			     struct v4l2_subdev_state *sd_state,
#endif
			     struct v4l2_subdev_format *fmt)
{
	struct isx031 *isx031 = to_isx031(sd);
	const struct isx031_mode *mode = NULL;
	unsigned int i;

	mutex_lock(&isx031_mutex);

	/* Find the best matching mode */
	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		if (supported_modes[i].code == fmt->format.code &&
			supported_modes[i].width == fmt->format.width &&
			supported_modes[i].height == fmt->format.height) {
			mode = &supported_modes[i];
			break;
		}
	}

	/* If no exact match, use the default mode */
	if (!mode)
		mode = &supported_modes[0];

	isx031_update_pad_format(mode, &fmt->format);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 14, 0)
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#elif LINUX_VERSION_CODE < KERNEL_VERSION(6, 8, 0)
		*v4l2_subdev_get_try_format(sd, sd_state, fmt->pad) = fmt->format;
#else
		*v4l2_subdev_state_get_format(sd_state, fmt->pad) = fmt->format;
#endif
	else
		isx031->cur_mode = mode;

	mutex_unlock(&isx031_mutex);

	return 0;
}

static int isx031_get_format(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 14, 0)
			     struct v4l2_subdev_pad_config *cfg,
#else
			     struct v4l2_subdev_state *sd_state,
#endif
			     struct v4l2_subdev_format *fmt)
{
	struct isx031 *isx031 = to_isx031(sd);

	mutex_lock(&isx031_mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 14, 0)
		fmt->format = *v4l2_subdev_get_try_format(&isx031->sd, cfg,
							  fmt->pad);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(6, 8, 0)
		fmt->format = *v4l2_subdev_get_try_format(&isx031->sd, sd_state,
							  fmt->pad);
#else
		fmt->format = *v4l2_subdev_state_get_format(sd_state, fmt->pad);
#endif
	else
		isx031_update_pad_format(isx031->cur_mode, &fmt->format);

	mutex_unlock(&isx031_mutex);

	return 0;
}

static int isx031_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	mutex_lock(&isx031_mutex);

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 14, 0)
	isx031_update_pad_format(&supported_modes[0],
				 v4l2_subdev_get_try_format(sd, fh->pad, 0));
#elif LINUX_VERSION_CODE < KERNEL_VERSION(6, 8, 0)
	isx031_update_pad_format(&supported_modes[0],
				 v4l2_subdev_get_try_format(sd, fh->state, 0));
#else
	isx031_update_pad_format(&supported_modes[0],
				 v4l2_subdev_state_get_format(fh->state, 0));
#endif

	mutex_unlock(&isx031_mutex);

	return 0;
}

static const struct v4l2_subdev_video_ops isx031_video_ops = {
	.s_stream = isx031_set_stream,
};

static const struct v4l2_subdev_pad_ops isx031_pad_ops = {
	.set_fmt = isx031_set_format,
	.get_fmt = isx031_get_format,
	.get_frame_desc = isx031_get_frame_desc,
	.enable_streams = isx031_enable_streams,
	.disable_streams = isx031_disable_streams,
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

static int isx031_set_ctrl(struct v4l2_ctrl *ctrl)
{
	return 0;
};

static const struct v4l2_ctrl_ops isx031_ctrl_ops = {
	.s_ctrl = isx031_set_ctrl,
};

static int isx031_ctrls_init(struct isx031 *sensor)
{
	struct v4l2_ctrl *ctrl;
	struct v4l2_ctrl_handler *hdl = &sensor->ctrls;

	v4l2_ctrl_handler_init(hdl, 10);

	/* There's a need to set the link frequency because IPU6 dictates it. */
	ctrl = v4l2_ctrl_new_int_menu(hdl, &isx031_ctrl_ops,
				      V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(isx031_link_frequencies) - 1, 0,
				      isx031_link_frequencies);

	if (hdl->error) {
		v4l2_ctrl_handler_free(hdl);
		return hdl->error;
	}

	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	sensor->sd.ctrl_handler = hdl;

	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
static int isx031_remove(struct i2c_client *client)
#else
static void isx031_remove(struct i2c_client *client)
#endif
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	pm_runtime_disable(&client->dev);

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	return 0;
#endif
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
	if (!isx031->platform_data)
		dev_warn(&client->dev, "No platform data provided\n");

	isx031->reset_gpio = devm_gpiod_get_optional(&client->dev, "reset",
							 GPIOD_OUT_LOW);
	if (IS_ERR(isx031->reset_gpio))
		return -EPROBE_DEFER;
	if (isx031->reset_gpio)
		dev_info(&client->dev, "Reset gpio found\n");
	else
		dev_warn(&client->dev, "Reset gpio not found\n");

	isx031->fsin_gpio = devm_gpiod_get_optional(&client->dev, "fsin",
						    GPIOD_OUT_LOW);
	if (isx031->fsin_gpio)
		dev_info(&client->dev, "Fsin gpio found\n");
	else
		dev_warn(&client->dev, "Fsin gpio not found\n");

	/* Initialize subdevice */
	sd = &isx031->sd;
	v4l2_i2c_subdev_init(sd, client, &isx031_subdev_ops);

	ret = isx031_ctrls_init(isx031);
	if (ret) {
		dev_err(&client->dev, "Failed to init sensor ctrls: %d\n", ret);
		return ret;
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
#else
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
	sd->internal_ops = &isx031_internal_ops;
	sd->entity.ops = &isx031_subdev_entity_ops;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize subdev media pad */
	isx031->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, 1, &isx031->pad);
	if (ret) {
		dev_err(&client->dev, "Failed to init entity pads: %d\n", ret);
		goto err_ctrl_free;
	}

	isx031->sd.state_lock = isx031->sd.ctrl_handler->lock;
	ret = v4l2_subdev_init_finalize(&isx031->sd);
	if (ret) {
		dev_err(&client->dev, "Failed to finalize V4L2 subdev: %d\n", ret);
		goto err_media_cleanup;
	}

	if (isx031->platform_data && isx031->platform_data->suffix[0])
		snprintf(isx031->sd.name, sizeof(isx031->sd.name), "isx031 %s",
			 isx031->platform_data->suffix);

	if (isx031->platform_data && isx031->platform_data->lanes)
		isx031->lanes = isx031->platform_data->lanes;
	else {
		/* Read info from fwnode entrypoint bus cfg if no platform data */
		ret = isx031_get_num_lane(isx031, &client->dev);
		if (ret) {
			dev_err(&client->dev, "Failed to get mipi lane configuration\n");
			goto err_media_cleanup;
		}
	}

	ret = isx031_identify_module(client);
	if (ret) {
		dev_err(&client->dev, "Failed to identify sensor module: %d\n", ret);
		goto err_media_cleanup;
	}

	/* 1920x1536 default */
	isx031->pre_mode = NULL;
	isx031->cur_mode = &supported_modes[0];
	ret = isx031_initialize_module(isx031);
	if (ret) {
		dev_err(&client->dev, "Failed to initialize sensor: %d\n", ret);
		goto err_media_cleanup;
	}

	reg_list = &isx031->cur_mode->reg_list;
	ret = isx031_write_reg_list(client, reg_list, true);
	if (ret) {
		dev_err(&client->dev, "Failed to apply preset mode\n");
		goto err_media_cleanup;
	}
	isx031->pre_mode = isx031->cur_mode;

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 13, 0)
	ret = v4l2_async_register_subdev_sensor_common(&isx031->sd);
#else
	ret = v4l2_async_register_subdev_sensor(&isx031->sd);
#endif
	if (ret) {
		dev_err(&client->dev, "Failed to register V4L2 subdev: %d\n", ret);
		goto err_media_cleanup;
	}

	/*
	 * Device is already turned on by i2c-core with ACPI domain PM.
	 * Enable runtime PM and turn off the device.
	 */
	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	return 0;

err_media_cleanup:
	media_entity_cleanup(&isx031->sd.entity);
err_ctrl_free:
	v4l2_ctrl_handler_free(isx031->sd.ctrl_handler);

	return ret;
}

static const struct dev_pm_ops isx031_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(isx031_suspend, isx031_resume)
};

static const struct i2c_device_id isx031_id_table[] = {
	{ "isx031", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, isx031_id_table);

static const struct acpi_device_id isx031_acpi_ids[] = {
	{ "INTC113C" },
	{}
};

MODULE_DEVICE_TABLE(acpi, isx031_acpi_ids);

static struct i2c_driver isx031_i2c_driver = {
	.driver = {
		.name = "isx031",
		.acpi_match_table = ACPI_PTR(isx031_acpi_ids),
		.pm = &isx031_pm_ops,
	},
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 6, 0)
	.probe_new = isx031_probe,
#else
	.probe = isx031_probe,
#endif
	.remove = isx031_remove,
	.id_table = isx031_id_table,
};

module_i2c_driver(isx031_i2c_driver);

MODULE_DESCRIPTION("isx031 sensor driver");
MODULE_AUTHOR("Hao Yao <hao.yao@intel.com>");
MODULE_AUTHOR("Jonathan Lui <jonathan.ming.jun.lui@intel.com>");
MODULE_AUTHOR("Wei Khang, Goh <wei.khang1.goh@intel.com>");
MODULE_LICENSE("GPL v2");
