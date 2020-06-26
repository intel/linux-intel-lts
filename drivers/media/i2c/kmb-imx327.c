// SPDX-License-Identifier: GPL-2.0-only
/*
 * Keem Bay Camera imx327 Sensor Driver.
 *
 * Copyright (C) 2019-2020 Intel Corporation
 */

#include <asm/unaligned.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <linux/clk.h>

#define KMB_IMX327_DRV_NAME	"kmb-imx327-sensor"

/* Streaming Mode */
#define KMB_IMX327_REG_MODE_SELECT	0x3000
#define KMB_IMX327_MODE_STANDBY		0x01
#define KMB_IMX327_MODE_STREAMING	0x00

/* SW reset */
#define KMB_IMX327_REG_SW_RESET		0x3003
#define KMB_IMX327_MODE_RESET		0x01
#define KMB_IMX327_MODE_OPERATING	0x00

/* Lines per frame */
#define KMB_IMX327_REG_LPFR		0x3018
#define KMB_IMX327_LPFR_MAX		0x3FFFF

/* Chip ID */
#define KMB_IMX327_REG_ID		0x31DC
#define KMB_IMX327_REG_ID_MASK		(0x6)
#define KMB_IMX327_ID			(0x6)

/* Exposure control */
#define KMB_IMX327_REG_SHUTTER		0x3020
#define KMB_IMX327_EXPOSURE_MIN		1
#define KMB_IMX327_EXPOSURE_STEP	1
#define KMB_IMX327_EXPOSURE_DEFAULT	0x0282

/* Analog gain control */
#define KMB_IMX327_REG_AGAIN		0x3014
#define KMB_IMX327_AGAIN_MIN		0
#define KMB_IMX327_AGAIN_MAX		230
#define KMB_IMX327_AGAIN_STEP		1
#define KMB_IMX327_AGAIN_DEFAULT	0

/* Group hold register */
#define KMB_IMX327_REG_HOLD	0x3001

/**
 * struct kmb_imx327_reg - KMB imx327 Sensor register
 * @address: Register address
 * @val: Register value
 */
struct kmb_imx327_reg {
	u16 address;
	u8 val;
};

/**
 * struct kmb_imx327_reg_list - KMB imx327 Sensor register list
 * @num_of_regs: Number of registers in the list
 * @regs: Pointer to register list
 */
struct kmb_imx327_reg_list {
	u32 num_of_regs;
	const struct kmb_imx327_reg *regs;
};

/**
 * struct kmb_imx327_mode - KMB imx327 Sensor mode structure
 * @width: Frame width
 * @height: Frame height
 * @code: Format code
 * @ppln: Pixels per line
 * @lpfr: Lines per frame
 * @skip_lines: Top lines to be skipped
 * @embedded_data: Embedded data in bytes
 * @pclk: Sensor pixel clock
 * @def: Default frames per second
 * @min: Min frames per second
 * @max: Max frames per second
 * @step: Frame rate step
 * @reg_list: Register list for sensor mode
 */
struct kmb_imx327_mode {
	u32 width;
	u32 height;
	u32 code;
	u32 ppln;
	u32 lpfr;
	u32 skip_lines;
	u32 embedded_data;
	u64 pclk;
	struct {
		u32 def;
		u32 min;
		u32 max;
		u32 step;
	} fps;
	struct kmb_imx327_reg_list reg_list;
};

/**
 * struct kmb_imx327 - KMB imx327 Sensor device structure
 * @dev: pointer to generic device
 * @client: Pointer to i2c client
 * @sd: V4L2 sub-device
 * @pad: Media pad. Only one pad supported
 * @reset_gpio: Sensor reset gpio
 * @ctrl_handler: V4L2 control handler
 * @pclk_ctrl: Pointer to pixel clock control
 * @hblank_ctrl: Pointer to horizontal blanking control
 * @vblank_ctrl: Pointer to vertical blanking control
 * @exp_ctrl: Pointer to exposure control
 * @again_ctrl: Pointer to analog gain control
 * @fps: FPS to be applied on next stream on
 * @lpfr: Lines per frame for long exposure frame
 * @cur_mode: Current selected sensor mode
 * @mutex: Mutex for serializing sensor controls
 * @streaming: Flag indicating streaming state
 */
struct kmb_imx327 {
	struct device *dev;
	struct i2c_client *client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct gpio_desc *reset_gpio;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pclk_ctrl;
	struct v4l2_ctrl *hblank_ctrl;
	struct v4l2_ctrl *vblank_ctrl;
	struct {
		struct v4l2_ctrl *exp_ctrl;
		struct v4l2_ctrl *again_ctrl;
	};
	u32 fps;
	u32 lpfr;
	const struct kmb_imx327_mode *cur_mode;
	struct mutex mutex;
	bool streaming;
};

/* Sensor mode registers */
static const struct kmb_imx327_reg mode_1920x1080_regs[] = {
	{0x3000, 0x01},
	{0x3002, 0x00},
	{0x3005, 0x01},
	{0x3007, 0x43},
	{0x3009, 0x02},
	{0x300A, 0xF0},
	{0x3011, 0x02},
	{0x3018, 0x65},
	{0x3019, 0x04},
	{0x301C, 0x30},
	{0x301D, 0x11},
	{0x303A, 0x0C},
	{0x303C, 0x08},
	{0x303D, 0x00},
	{0x303E, 0x38},
	{0x303F, 0x04},
	{0x3040, 0x0C},
	{0x3041, 0x00},
	{0x3042, 0x80},
	{0x3043, 0x07},
	{0x3046, 0x01},
	{0x304B, 0x0A},
	{0x305C, 0x18},
	{0x305D, 0x03},
	{0x305E, 0x20},
	{0x305F, 0x01},
	{0x309E, 0x4A},
	{0x309F, 0x4A},
	{0x30D2, 0x19},
	{0x30D7, 0x03},
	{0x3129, 0x00},
	{0x313B, 0x61},
	{0x315E, 0x1A},
	{0x3164, 0x1A},
	{0x317C, 0x00},
	{0x31EC, 0x0E},
	{0x3404, 0x01},
	{0x3405, 0x20},
	{0x3407, 0x03},
	{0x3414, 0x0A},
	{0x3418, 0x38},
	{0x3419, 0x04},
	{0x3441, 0x0C},
	{0x3442, 0x0C},
	{0x3443, 0x03},
	{0x3444, 0x20},
	{0x3445, 0x25},
	{0x3446, 0x47},
	{0x3447, 0x00},
	{0x3448, 0x1F},
	{0x3449, 0x00},
	{0x344A, 0x17},
	{0x344B, 0x00},
	{0x344C, 0x0F},
	{0x344D, 0x00},
	{0x344E, 0x17},
	{0x344F, 0x00},
	{0x3450, 0x47},
	{0x3451, 0x00},
	{0x3452, 0x0F},
	{0x3453, 0x00},
	{0x3454, 0x0F},
	{0x3455, 0x00},
	{0x3472, 0x80},
	{0x3473, 0x07},
	{0x3480, 0x49},
};

/* Supported sensor mode configurations */
static const struct kmb_imx327_mode supported_modes[] = {
	{
		.width = 1920,
		.height = 1080,
		.ppln = 2200,
		.lpfr = 1125,
		.skip_lines = 10,
		.embedded_data = 384,
		.pclk = 74250000,
		.code = MEDIA_BUS_FMT_SGBRG12_1X12,
		.fps = {
			.def = 30,
			.max = 30,
			.min = 1,
			.step = 5,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1920x1080_regs),
			.regs = mode_1920x1080_regs,
		},
	},
};

/**
 * to_kmb_imx327 - imx327 V4L2 sub-device to kmb_imx327 device.
 * @subdev: pointer to imx327 V4L2 sub-device device
 *
 * Return: Pointer to kmb_imx327 device
 */
static inline struct kmb_imx327 *to_kmb_imx327(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct kmb_imx327, sd);
}

/**
 * kmb_imx327_read_reg - Read registers.
 * @kmb_imx327: pointer to imx327 device
 * @reg: Register address
 * @len: Length of bytes to read. Max supported bytes is 4
 * @val: Pointer to register value to be filled.
 *
 * Return: 0 if successful
 */
static int
kmb_imx327_read_reg(struct kmb_imx327 *kmb_imx327, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&kmb_imx327->sd);
	struct i2c_msg msgs[2];
	u8 addr_buf[2] = { reg >> 8, reg & 0xff };
	u8 data_buf[4] = { 0 };
	int ret;

	if (len > 4)
		return -EINVAL;

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	/* Read data from register */
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

/**
 * kmb_imx327_write_reg - Write register
 * @kmb_imx327: pointer to imx327 device
 * @reg: Register address
 * @len: Length of bytes. Max supported bytes is 4
 * @val: Register value
 *
 * Return: 0 if successful
 */
static int
kmb_imx327_write_reg(struct kmb_imx327 *kmb_imx327, u16 reg, u32 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&kmb_imx327->sd);
	u8 buf[3];
	int i;
	int ret;

	if (len > 4) {
		dev_err_ratelimited(kmb_imx327->dev,
				    "write reg 0x%4.4x invalid len %d",
				    reg, len);
		return -EINVAL;
	}

	/* Currently we can write to sensor only one byte at a time */
	for (i = 0; i < len; i++) {
		put_unaligned_be16(reg + i, buf);
		buf[2] = (val >> (8 * i)) & 0xFF;
		ret = i2c_master_send(client, buf, ARRAY_SIZE(buf));
		if (ret != ARRAY_SIZE(buf)) {
			dev_err_ratelimited(kmb_imx327->dev,
					"write reg 0x%4.4x return err %d",
					reg, ret);
			return -EIO;
		}
	}

	return 0;
}

/**
 * kmb_imx327_write_regs - Write a list of registers
 * @kmb_imx327: pointer to imx327 device
 * @regs: List of registers to be written
 * @len: Length of registers array
 *
 * Return: 0 if successful
 */
static int kmb_imx327_write_regs(struct kmb_imx327 *kmb_imx327,
			     const struct kmb_imx327_reg *regs, u32 len)
{
	int ret;
	u32 i;

	for (i = 0; i < len; i++) {
		ret = kmb_imx327_write_reg(kmb_imx327,
					   regs[i].address,
					   1, regs[i].val);
		if (ret)
			return ret;
	}

	return 0;
}

/**
 * kmb_imx327_update_fps - Update current sensor mode to match the selected FPS
 * @kmb_imx327: pointer to imx327 device
 *
 * Return: none
 */
static void kmb_imx327_update_fps(struct kmb_imx327 *kmb_imx327)
{
	u32 lpfr = (kmb_imx327->cur_mode->lpfr *
		    kmb_imx327->cur_mode->fps.def) / kmb_imx327->fps;

	if (lpfr > KMB_IMX327_LPFR_MAX)
		lpfr = KMB_IMX327_LPFR_MAX;

	if (lpfr < kmb_imx327->cur_mode->height + 25)
		lpfr = kmb_imx327->cur_mode->height + 25;

	kmb_imx327->lpfr = lpfr;

	dev_dbg(kmb_imx327->dev, "Selected FPS %d lpfr %d",
		kmb_imx327->fps, lpfr);
}

/**
 * kmb_imx327_open - Open imx327 subdevice
 * @sd: pointer to imx327 V4L2 sub-device structure
 * @fh: pointer to imx327 V4L2 sub-device file handle
 *
 * Return: 0 if successful
 */
static int kmb_imx327_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct kmb_imx327 *kmb_imx327 = to_kmb_imx327(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->pad, 0);

	mutex_lock(&kmb_imx327->mutex);

	/* Initialize try_fmt */
	try_fmt->width = kmb_imx327->cur_mode->width;
	try_fmt->height = kmb_imx327->cur_mode->height;
	try_fmt->code = kmb_imx327->cur_mode->code;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&kmb_imx327->mutex);

	return 0;
}

/**
 * kmb_imx327_get_camera_mode_by_fmt - Get the most appropriate camera
 *         mode that meets the code and resolution criteria
 * @kmb_imx327: pointer to kmb_imx327 device
 * @code: media bus format code
 * @width: frame width
 * @height: frame height
 *
 * Return: pointer to the most appropriate camera mode
 */
static const struct kmb_imx327_mode *
kmb_imx327_get_camera_mode_by_fmt(struct kmb_imx327 *kmb_imx327, u32 code,
				  u32 width, u32 height)
{
	const struct kmb_imx327_mode *mode = supported_modes;
	int n = ARRAY_SIZE(supported_modes);
	int i;

	for (i = 0; i < n; i++) {
		if (mode[i].code == code && mode[i].width == width &&
		    mode[i].height == height)
			return &mode[i];
	}

	return NULL;
}

/**
 * kmb_imx327_set_ctrl - Set subdevice control. Supported controls:
 *                       V4L2_CID_ANALOGUE_GAIN
 *                       V4L2_CID_EXPOSURE
 *                       Both controls are in one cluster.
 *
 * @ctrl: pointer to v4l2_ctrl structure
 *
 * Return: 0 if successful
 */
static int kmb_imx327_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct kmb_imx327 *kmb_imx327 =
		container_of(ctrl->handler, struct kmb_imx327, ctrl_handler);
	u32 exposure;
	u32 analog_gain;
	u32 shutter;
	u32 lpfr = kmb_imx327->lpfr;
	int ret;

	/* Set exposure and gain only if sensor is in power on state */
	if (!pm_runtime_get_if_in_use(kmb_imx327->dev))
		return 0;

	/* Handle the cluster for both controls */
	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		exposure = ctrl->val;
		analog_gain = kmb_imx327->again_ctrl->val;
		break;
	default:
		dev_dbg(kmb_imx327->dev, "Invalid control %d", ctrl->id);
		ret = -EINVAL;
		goto error_pm_runtime_put;
	}

	/* Align exposure time */
	if (exposure > (lpfr - 2))
		exposure = lpfr - 2;
	else if (exposure < 1)
		exposure = 1;

	shutter = lpfr - exposure - 1;

	dev_dbg(kmb_imx327->dev,
		"Set exposure %d analog gain %d shutter %d lpfr %d",
		exposure, analog_gain, shutter, lpfr);

	ret = kmb_imx327_write_reg(kmb_imx327, KMB_IMX327_REG_HOLD, 1, 1);
	if (ret)
		goto error_pm_runtime_put;

	ret = kmb_imx327_write_reg(kmb_imx327, KMB_IMX327_REG_LPFR,
				   3, lpfr);
	if (ret)
		goto error_release_group_hold;

	ret = kmb_imx327_write_reg(kmb_imx327, KMB_IMX327_REG_SHUTTER,
				   3, shutter);
	if (ret)
		goto error_release_group_hold;


	ret = kmb_imx327_write_reg(kmb_imx327, KMB_IMX327_REG_AGAIN,
				   1, analog_gain);
	if (ret)
		goto error_release_group_hold;

	kmb_imx327_write_reg(kmb_imx327, KMB_IMX327_REG_HOLD, 1, 0);

	pm_runtime_put(kmb_imx327->dev);

	return 0;

error_release_group_hold:
	kmb_imx327_write_reg(kmb_imx327, KMB_IMX327_REG_HOLD, 1, 0);
error_pm_runtime_put:
	pm_runtime_put(kmb_imx327->dev);
	return ret;
}

/* V4l2 subdevice control ops*/
static const struct v4l2_ctrl_ops kmb_imx327_ctrl_ops = {
	.s_ctrl = kmb_imx327_set_ctrl,
};

/**
 * kmb_imx327_enum_mbus_code - Enumerate V4L2 sub-device mbus codes
 * @sd: pointer to imx327 V4L2 sub-device structure
 * @cfg: V4L2 sub-device pad configuration
 * @fsize: V4L2 sub-device size enumeration need to be filled
 *
 * Return: 0 if successful
 */
static int kmb_imx327_enum_mbus_code(struct v4l2_subdev *sd,
				     struct v4l2_subdev_pad_config *cfg,
				     struct v4l2_subdev_mbus_code_enum *code)
{
	struct kmb_imx327 *kmb_imx327 = to_kmb_imx327(sd);

	if (code->index > 0)
		return -EINVAL;

	mutex_lock(&kmb_imx327->mutex);
	code->code = supported_modes[0].code;
	mutex_unlock(&kmb_imx327->mutex);

	return 0;
}

/**
 * kmb_imx327_enum_frame_size - Enumerate V4L2 sub-device frame sizes
 * @sd: pointer to imx327 V4L2 sub-device structure
 * @cfg: V4L2 sub-device pad configuration
 * @fsize: V4L2 sub-device size enumeration need to be filled
 *
 * Return: 0 if successful
 */
static int kmb_imx327_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fsize)
{
	struct kmb_imx327 *kmb_imx327 = to_kmb_imx327(sd);

	if (fsize->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	mutex_lock(&kmb_imx327->mutex);
	if (fsize->code != supported_modes[fsize->index].code) {
		mutex_unlock(&kmb_imx327->mutex);
		return -EINVAL;
	}
	mutex_unlock(&kmb_imx327->mutex);

	fsize->min_width = supported_modes[fsize->index].width;
	fsize->max_width = fsize->min_width;
	fsize->min_height = supported_modes[fsize->index].height;
	fsize->max_height = fsize->min_height;

	return 0;
}

/**
 * kmb_imx327_enum_frame_interval - Enumerate V4L2 sub-device frame intervals
 * @sd: pointer to imx327 V4L2 sub-device structure
 * @cfg: V4L2 sub-device pad configuration
 * @fsize: V4L2 sub-device size enumeration need to be filled
 *
 * callback for VIDIOC_SUBDEV_ENUM_FRAME_INTERVAL()
 *
 * Return: 0 if successful
 */
static int
kmb_imx327_enum_frame_interval(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_frame_interval_enum *fie)
{
	struct kmb_imx327 *kmb_imx327 = to_kmb_imx327(sd);
	const struct kmb_imx327_mode *mode;
	int fps;
	int ret = 0;

	if (fie->pad)
		return -EINVAL;

	mutex_lock(&kmb_imx327->mutex);

	if (fie->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		if (fie->code != kmb_imx327->cur_mode->code ||
		    fie->width != kmb_imx327->cur_mode->width ||
		    fie->height != kmb_imx327->cur_mode->height) {
			ret = -EINVAL;
			goto exit_unlock;
		}

		mode = kmb_imx327->cur_mode;
	} else {
		mode = kmb_imx327_get_camera_mode_by_fmt(kmb_imx327,
				fie->code, fie->width, fie->height);
		if (!mode) {
			ret = -EINVAL;
			goto exit_unlock;
		}
	}

	fps = mode->fps.step * fie->index;
	fie->interval.numerator = 1;
	fie->interval.denominator = fps;

	if (fps < mode->fps.min) {
		fie->interval.denominator = mode->fps.min;
	} else if (fps > mode->fps.max) {
		ret = -EINVAL;
		goto exit_unlock;
	}

	dev_dbg(kmb_imx327->dev, "Enum FPS %d %d/%d", fps,
		fie->interval.numerator, fie->interval.denominator);

exit_unlock:
	mutex_unlock(&kmb_imx327->mutex);
	return ret;
}

/**
 * kmb_imx327_fill_pad_format - Fill subdevice pad format
 *                              from selected sensor mode
 * @kmb_imx327: pointer to kmb_imx327 device
 * @mode: Pointer to kmb_imx327_mode sensor mode
 * @fmt: V4L2 sub-device format need to be filled
 *
 * Return: none
 */
static void kmb_imx327_fill_pad_format(struct kmb_imx327 *kmb_imx327,
				       const struct kmb_imx327_mode *mode,
				       struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.code = mode->code;
	fmt->format.field = V4L2_FIELD_NONE;
}

/**
 * kmb_imx327_skip_top_lines - Skip top lines containing metadata
 * @sd: pointer to imx327 V4L2 sub-device structure
 * @lines: number of lines to be skipped
 *
 * Return: 0 if successful
 */
static int kmb_imx327_skip_top_lines(struct v4l2_subdev *sd, u32 *lines)
{
	struct kmb_imx327 *kmb_imx327 = to_kmb_imx327(sd);

	*lines = kmb_imx327->cur_mode->skip_lines;

	return 0;
}

/**
 * kmb_imx327_get_frame_desc - Get the current low level media bus frame
 *                             parameters
 * @sd: pointer to imx327 V4L2 sub-device structure
 * @pad: V4L2 sub-device pad
 * @desc: media bus frame description
 */
static int kmb_imx327_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				     struct v4l2_mbus_frame_desc *desc)
{
	struct kmb_imx327 *kmb_imx327 = to_kmb_imx327(sd);

	desc->entry[0].flags = V4L2_MBUS_FRAME_DESC_FL_LEN_MAX;
	desc->entry[0].pixelcode = MEDIA_BUS_FMT_FIXED;
	desc->entry[0].length = kmb_imx327->cur_mode->embedded_data;
	desc->num_entries = 1;

	return 0;
}

/**
 * kmb_imx327_update_controls - Update control ranges based on streaming mode
 * @kmb_imx327: pointer to kmb_imx327 device
 * @mode: pointer to kmb_imx327_mode sensor mode
 *
 * Return: none
 */
static void kmb_imx327_update_controls(struct kmb_imx327 *kmb_imx327,
					const struct kmb_imx327_mode *mode)
{
	__v4l2_ctrl_s_ctrl(kmb_imx327->vblank_ctrl,
			   kmb_imx327->lpfr - mode->height);
	__v4l2_ctrl_s_ctrl(kmb_imx327->hblank_ctrl,
			   mode->ppln - mode->width);
	__v4l2_ctrl_modify_range(kmb_imx327->pclk_ctrl,
				mode->pclk, mode->pclk,
				1, mode->pclk);
	__v4l2_ctrl_modify_range(kmb_imx327->exp_ctrl,
				KMB_IMX327_EXPOSURE_MIN, kmb_imx327->lpfr,
				1, KMB_IMX327_EXPOSURE_DEFAULT);
}

/**
 * kmb_imx327_get_pad_format - Get subdevice pad format
 * @sd: pointer to imx327 V4L2 sub-device structure
 * @cfg: V4L2 sub-device pad configuration
 * @fmt: V4L2 sub-device format need to be set
 *
 * Return: 0 if successful
 */
static int kmb_imx327_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_format *fmt)
{
	struct kmb_imx327 *kmb_imx327 = to_kmb_imx327(sd);

	mutex_lock(&kmb_imx327->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *framefmt;

		framefmt = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		fmt->format = *framefmt;
	} else {
		kmb_imx327_fill_pad_format(kmb_imx327,
					   kmb_imx327->cur_mode,
					   fmt);
	}

	mutex_unlock(&kmb_imx327->mutex);

	return 0;
}

/**
 * kmb_imx327_set_pad_format - Set subdevice pad format
 * @sd: pointer to imx327 V4L2 sub-device structure
 * @cfg: V4L2 sub-device pad configuration
 * @fmt: V4L2 sub-device format need to be set
 *
 * Return: 0 if successful
 */
static int kmb_imx327_set_pad_format(struct v4l2_subdev *sd,
				     struct v4l2_subdev_pad_config *cfg,
				     struct v4l2_subdev_format *fmt)
{
	struct kmb_imx327 *kmb_imx327 = to_kmb_imx327(sd);
	const struct kmb_imx327_mode *mode;
	struct v4l2_mbus_framefmt *framefmt;

	mutex_lock(&kmb_imx327->mutex);

	/* Currently only one format is supported */
	fmt->format.code = supported_modes[0].code;

	mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);

	kmb_imx327->fps = mode->fps.def;
	kmb_imx327->lpfr = mode->lpfr;

	kmb_imx327_update_fps(kmb_imx327);

	kmb_imx327_fill_pad_format(kmb_imx327, mode, fmt);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		framefmt = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		*framefmt = fmt->format;
	} else {
		kmb_imx327_update_controls(kmb_imx327, mode);
		kmb_imx327->cur_mode = mode;
	}

	mutex_unlock(&kmb_imx327->mutex);

	return 0;
}

/**
 * kmb_imx327_start_streaming - Start sensor stream
 * @kmb_imx327: pointer to kmb_imx327 device
 *
 * Return: 0 if successful
 */
static int kmb_imx327_start_streaming(struct kmb_imx327 *kmb_imx327)
{
	const struct kmb_imx327_reg_list *reg_list;
	int ret;

	/* Write sensor mode registers */
	reg_list = &kmb_imx327->cur_mode->reg_list;
	ret = kmb_imx327_write_regs(kmb_imx327, reg_list->regs,
				    reg_list->num_of_regs);
	if (ret) {
		dev_err(kmb_imx327->dev, "fail to write initial registers");
		return ret;
	}

	/* Setup handler will write actual exposure and gain */
	ret =  __v4l2_ctrl_handler_setup(kmb_imx327->sd.ctrl_handler);
	if (ret) {
		dev_err(kmb_imx327->dev, "fail to setup handler");
		return ret;
	}

	/* Start streaming */
	ret = kmb_imx327_write_reg(kmb_imx327, KMB_IMX327_REG_MODE_SELECT,
				   1, KMB_IMX327_MODE_STREAMING);
	if (ret) {
		dev_err(kmb_imx327->dev, "fail to start streaming");
		return ret;
	}

	return 0;
}

/**
 * kmb_imx327_stop_streaming - Stop sensor stream
 * @kmb_imx327: pointer to kmb_imx327 device
 *
 * Return: 0 if successful
 */
static int kmb_imx327_stop_streaming(struct kmb_imx327 *kmb_imx327)
{
	return kmb_imx327_write_reg(kmb_imx327, KMB_IMX327_REG_MODE_SELECT,
				    1, KMB_IMX327_MODE_STANDBY);
}

/**
 * kmb_imx327_set_stream - Enable sensor streaming
 * @sd: pointer to imx327 subdevice
 * @enable: Set to enable sensor streaming
 *
 * Return: 0 if successful
 */
static int kmb_imx327_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct kmb_imx327 *kmb_imx327 = to_kmb_imx327(sd);
	int ret;

	mutex_lock(&kmb_imx327->mutex);

	if (kmb_imx327->streaming == enable) {
		mutex_unlock(&kmb_imx327->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_get_sync(kmb_imx327->dev);
		if (ret)
			goto error_unlock;

		ret = kmb_imx327_start_streaming(kmb_imx327);
		if (ret)
			goto error_power_off;
	} else {
		kmb_imx327_stop_streaming(kmb_imx327);
		pm_runtime_put(kmb_imx327->dev);
	}

	kmb_imx327->streaming = enable;

	mutex_unlock(&kmb_imx327->mutex);

	return 0;

error_power_off:
	pm_runtime_put(kmb_imx327->dev);
error_unlock:
	mutex_unlock(&kmb_imx327->mutex);
	return ret;
}

/**
 * kmb_imx327_get_frame_interval - Get subdevice frame interval
 * @sd: pointer to imx327 V4L2 sub-device structure
 * @interval: V4L2 sub-device current farme interval
 *
 * Return: 0 if successful
 */
static int kmb_imx327_get_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *interval)
{
	struct kmb_imx327 *kmb_imx327 = to_kmb_imx327(sd);

	if (interval->pad)
		return -EINVAL;

	mutex_lock(&kmb_imx327->mutex);
	interval->interval.numerator = 1;
	interval->interval.denominator = kmb_imx327->fps;
	mutex_unlock(&kmb_imx327->mutex);

	dev_dbg(kmb_imx327->dev, "Get frame interval %d/%d",
		interval->interval.numerator,
		interval->interval.denominator);

	return 0;
}

/**
 * kmb_imx327_set_frame_interval - Set subdevice frame interval
 * @sd: pointer to imx327 V4L2 sub-device structure
 * @interval: V4L2 sub-device farme interval to be set
 *
 * Return: 0 if successful
 */
static int kmb_imx327_set_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *interval)
{
	struct kmb_imx327 *kmb_imx327 = to_kmb_imx327(sd);
	u32 fps;

	if (interval->pad)
		return -EINVAL;

	mutex_lock(&kmb_imx327->mutex);
	fps = (u32)(interval->interval.denominator /
		interval->interval.numerator);
	if (fps < kmb_imx327->cur_mode->fps.min) {
		interval->interval.numerator = 1;
		interval->interval.denominator = kmb_imx327->cur_mode->fps.min;
	} else if (fps > kmb_imx327->cur_mode->fps.max) {
		interval->interval.numerator = 1;
		interval->interval.denominator = kmb_imx327->cur_mode->fps.max;
	}

	kmb_imx327->fps = (u32) (interval->interval.denominator /
				     interval->interval.numerator);

	kmb_imx327_update_fps(kmb_imx327);

	kmb_imx327_update_controls(kmb_imx327, kmb_imx327->cur_mode);

	mutex_unlock(&kmb_imx327->mutex);

	dev_dbg(kmb_imx327->dev, "Set frame interval %d/%d",
		interval->interval.numerator,
		interval->interval.denominator);

	return 0;
}

/**
 * kmb_imx327_power_on - Sensor power on sequence
 * @kmb_imx327: imb_imx327 device
 *
 * Return: 0 if successful
 */
static int kmb_imx327_power_on(struct kmb_imx327 *kmb_imx327)
{
	/* request optional reset pin */
	kmb_imx327->reset_gpio =
		gpiod_get_optional(kmb_imx327->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(kmb_imx327->reset_gpio)) {
		dev_err(kmb_imx327->dev, "failed to get reset gpio %ld",
			PTR_ERR(kmb_imx327->reset_gpio));
		return PTR_ERR(kmb_imx327->reset_gpio);
	}
	usleep_range(500, 600);

	/* Ignore the call if reset gpio is not present */
	if (kmb_imx327->reset_gpio)
		gpiod_set_value_cansleep(kmb_imx327->reset_gpio, 1);

	usleep_range(20000, 21000);

	return 0;
}

/**
 * kmb_imx327_power_off - Sensor power off sequence
 * @kmb_imx327: imb_imx327 device
 */
static void kmb_imx327_power_off(struct kmb_imx327 *kmb_imx327)
{
	/* Ignore the call if reset gpio is not present */
	if (kmb_imx327->reset_gpio) {
		gpiod_set_value_cansleep(kmb_imx327->reset_gpio, 0);
		gpiod_put(kmb_imx327->reset_gpio);
		kmb_imx327->reset_gpio = NULL;
	}
}

/**
 * kmb_imx327_detect - Detect imx327 sensor
 * @kmb_imx327: pointer to kmb_imx327 device
 *
 * Return: 0 if successful, -EIO if sensor id does not match
 */
static int kmb_imx327_detect(struct kmb_imx327 *kmb_imx327)
{
	int ret;
	u32 val = 0;

	ret = kmb_imx327_read_reg(kmb_imx327, KMB_IMX327_REG_ID, 2, &val);
	if (ret)
		return ret;

	if (((val >> 8) &  KMB_IMX327_REG_ID_MASK) != KMB_IMX327_ID) {
		dev_err(kmb_imx327->dev, "chip id mismatch: %x!=%x",
			KMB_IMX327_ID,
			((val >> 8) &  KMB_IMX327_REG_ID_MASK));
		return -EIO;
	}
	return 0;
}

/* V4l2 subdevice ops */
static const struct v4l2_subdev_video_ops kmb_imx327_video_ops = {
	.s_stream = kmb_imx327_set_stream,
	.g_frame_interval = kmb_imx327_get_frame_interval,
	.s_frame_interval = kmb_imx327_set_frame_interval,
};

static const struct v4l2_subdev_pad_ops kmb_imx327_pad_ops = {
	.enum_mbus_code = kmb_imx327_enum_mbus_code,
	.get_fmt = kmb_imx327_get_pad_format,
	.set_fmt = kmb_imx327_set_pad_format,
	.enum_frame_size = kmb_imx327_enum_frame_size,
	.enum_frame_interval = kmb_imx327_enum_frame_interval,
	.get_frame_desc = kmb_imx327_get_frame_desc,
};

static const struct v4l2_subdev_sensor_ops kmb_imx327_sensor_ops = {
	.g_skip_top_lines = kmb_imx327_skip_top_lines,
};

static const struct v4l2_subdev_ops kmb_imx327_subdev_ops = {
	.video = &kmb_imx327_video_ops,
	.pad = &kmb_imx327_pad_ops,
	.sensor = &kmb_imx327_sensor_ops,
};

static const struct media_entity_operations kmb_imx327_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops kmb_imx327_internal_ops = {
	.open = kmb_imx327_open,
};

/**
 * kmb_imx327_init_controls - Initialize sensor subdevice controls
 * @kmb_imx327: pointer to kmb_imx327 device
 *
 * Return: 0 if successful
 */
static int kmb_imx327_init_controls(struct kmb_imx327 *kmb_imx327)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &kmb_imx327->ctrl_handler;
	const struct kmb_imx327_mode *mode = kmb_imx327->cur_mode;
	u32 hblank;
	u32 vblank;
	int ret;

	ctrl_hdlr = &kmb_imx327->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 5);
	if (ret)
		return ret;

	/* Serialize controls with sensor device */
	ctrl_hdlr->lock = &kmb_imx327->mutex;

	/* Initialize exposure and gain */
	kmb_imx327->exp_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						 &kmb_imx327_ctrl_ops,
						 V4L2_CID_EXPOSURE,
						 KMB_IMX327_EXPOSURE_MIN,
						 mode->lpfr,
						 KMB_IMX327_EXPOSURE_STEP,
						 KMB_IMX327_EXPOSURE_DEFAULT);

	kmb_imx327->again_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						   &kmb_imx327_ctrl_ops,
						   V4L2_CID_ANALOGUE_GAIN,
						   KMB_IMX327_AGAIN_MIN,
						   KMB_IMX327_AGAIN_MAX,
						   KMB_IMX327_AGAIN_STEP,
						   KMB_IMX327_AGAIN_DEFAULT);
	v4l2_ctrl_cluster(2, &kmb_imx327->exp_ctrl);

	/* Read only controls */
	kmb_imx327->pclk_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						  &kmb_imx327_ctrl_ops,
						  V4L2_CID_PIXEL_RATE,
						  mode->pclk, mode->pclk,
						  1, mode->pclk);
	if (kmb_imx327->pclk_ctrl)
		kmb_imx327->pclk_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank = mode->lpfr - mode->height;
	kmb_imx327->vblank_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						    &kmb_imx327_ctrl_ops,
						    V4L2_CID_VBLANK, vblank,
						    vblank, 1, vblank);
	if (kmb_imx327->vblank_ctrl)
		kmb_imx327->vblank_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	hblank = mode->ppln - mode->width;
	kmb_imx327->hblank_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						&kmb_imx327_ctrl_ops,
						V4L2_CID_HBLANK, hblank,
						hblank, 1, hblank);
	if (kmb_imx327->hblank_ctrl)
		kmb_imx327->hblank_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(kmb_imx327->dev, "control init failed: %d", ret);
		goto error;
	}

	kmb_imx327->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

/* --------------- probe as i2c device -------------------- */

/**
 * kmb_imx327_i2c_resume - PM resume callback
 * @dev: i2c device
 *
 * Return: 0 if successful
 */
static int kmb_imx327_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct kmb_imx327 *kmb_imx327 = to_kmb_imx327(sd);

	return kmb_imx327_power_on(kmb_imx327);
}

/**
 * kmb_imx327_i2c_suspend - PM suspend callback
 * @dev: i2c device
 *
 * Return: 0 if successful
 */
static int kmb_imx327_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct kmb_imx327 *kmb_imx327 = to_kmb_imx327(sd);

	kmb_imx327_power_off(kmb_imx327);

	return 0;
}

/**
 * kmb_imx327_i2c_probe - I2C client device binding
 * @pdev: pointer to platform device
 *
 * Return: 0 if successful
 */
static int kmb_imx327_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct kmb_imx327 *kmb_imx327;
	int ret;

	kmb_imx327 = devm_kzalloc(&client->dev, sizeof(*kmb_imx327),
				  GFP_KERNEL);
	if (!kmb_imx327)
		return -ENOMEM;

	mutex_init(&kmb_imx327->mutex);

	kmb_imx327->dev = &client->dev;

	/* Initialize subdev */
	v4l2_i2c_subdev_init(&kmb_imx327->sd, client, &kmb_imx327_subdev_ops);

	ret = kmb_imx327_power_on(kmb_imx327);
	if (ret) {
		dev_err(&client->dev, "failed to power-on the sensor\n");
		goto error_mutex_destroy;
	}

	/* Check module identity */
	ret = kmb_imx327_detect(kmb_imx327);
	if (ret) {
		dev_err(&client->dev, "failed to find sensor: %d", ret);
		goto error_sensor_power_off;
	}

	/* Set default mode to max resolution */
	kmb_imx327->cur_mode = &supported_modes[0];
	kmb_imx327->fps = kmb_imx327->cur_mode->fps.def;
	kmb_imx327->lpfr = kmb_imx327->cur_mode->lpfr;

	ret = kmb_imx327_init_controls(kmb_imx327);
	if (ret) {
		dev_err(&client->dev, "failed to init controls: %d", ret);
		goto error_sensor_power_off;
	}

	/* Initialize subdev */
	kmb_imx327->sd.internal_ops = &kmb_imx327_internal_ops;
	kmb_imx327->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	kmb_imx327->sd.entity.ops = &kmb_imx327_subdev_entity_ops;
	kmb_imx327->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
	kmb_imx327->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&kmb_imx327->sd.entity, 1,
				     &kmb_imx327->pad);
	if (ret) {
		dev_err(&client->dev, "failed to init entity pads: %d", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor_common(&kmb_imx327->sd);
	if (ret < 0) {
		dev_err(&client->dev,
				"failed to register async subdev: %d", ret);
		goto error_media_entity;
	}

	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	return 0;

error_media_entity:
	media_entity_cleanup(&kmb_imx327->sd.entity);
error_handler_free:
	v4l2_ctrl_handler_free(kmb_imx327->sd.ctrl_handler);
error_sensor_power_off:
	kmb_imx327_power_off(kmb_imx327);
error_mutex_destroy:
	mutex_destroy(&kmb_imx327->mutex);

	return ret;
}

/**
 * kmb_imx327_i2c_remove - I2C client device unbinding
 * @client: pointer to I2C client device
 *
 * Return: 0 if successful
 */
static int kmb_imx327_i2c_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct kmb_imx327 *kmb_imx327 = to_kmb_imx327(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(&client->dev);
	pm_runtime_suspended(&client->dev);

	mutex_destroy(&kmb_imx327->mutex);

	return 0;
}

static const struct dev_pm_ops kmb_imx327_i2c_pm_ops = {
	SET_RUNTIME_PM_OPS(kmb_imx327_i2c_suspend, kmb_imx327_i2c_resume, NULL)
};

static const struct i2c_device_id kmb_imx327_i2c_id_table[] = {
	{KMB_IMX327_DRV_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, kmb_imx327_i2c_id_table);

static struct i2c_driver kmb_imx327_i2c_driver = {
	.probe = kmb_imx327_i2c_probe,
	.remove = kmb_imx327_i2c_remove,
	.id_table = kmb_imx327_i2c_id_table,
	.driver = {
		.owner = THIS_MODULE,
		.pm = &kmb_imx327_i2c_pm_ops,
		.name = KMB_IMX327_DRV_NAME,
	},
};

/* --------------- probe as platform device ----------------- */

/**
 * kmb_imx327_platform_resume - PM resume callback
 * @dev: platform device
 *
 * Return: 0 if successful
 */
static int kmb_imx327_platform_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct kmb_imx327 *kmb_imx327 = platform_get_drvdata(pdev);

	return kmb_imx327_power_on(kmb_imx327);
}

/**
 * kmb_imx327_platform_suspend - PM suspend callback
 * @dev: platform device
 *
 * Return: 0 if successful
 */
static int kmb_imx327_platform_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct kmb_imx327 *kmb_imx327 = platform_get_drvdata(pdev);

	kmb_imx327_power_off(kmb_imx327);

	return 0;
}

/**
 * kmb_imx327_get_i2c_client - Get I2C client
 * @kmb_imx327: pointer to kmb_imx327 device
 *
 * Return: 0 if successful
 */
static int kmb_imx327_get_i2c_client(struct kmb_imx327 *kmb_imx327)
{
	struct i2c_board_info info = {
		I2C_BOARD_INFO("kmb-imx327-sensor-p", 0x1A)};
	const unsigned short addr_list[] = {0x1A, 0x10,
		0x36, 0x37, I2C_CLIENT_END};
	struct i2c_adapter *i2c_adp;
	struct device_node *phandle;

	phandle = of_parse_phandle(kmb_imx327->dev->of_node, "i2c-bus", 0);
	if (!phandle)
		return -ENODEV;

	i2c_adp = of_get_i2c_adapter_by_node(phandle);
	of_node_put(phandle);
	if (!i2c_adp)
		return -EPROBE_DEFER;

	kmb_imx327->client =
		i2c_new_probed_device(i2c_adp, &info, addr_list, NULL);
	i2c_put_adapter(i2c_adp);
	if (!kmb_imx327->client)
		return -ENODEV;

	dev_dbg(kmb_imx327->dev, "Detected on i2c address %x", info.addr);
	return 0;
}

/**
 * kmb_imx327_probe - Platform device binding
 * @pdev: pointer to platform device
 *
 * Return: 0 if successful
 */
static int kmb_imx327_pdev_probe(struct platform_device *pdev)
{
	struct kmb_imx327 *kmb_imx327;
	struct gpio_descs *detect_gpios;
	int ret;

	kmb_imx327 = devm_kzalloc(&pdev->dev, sizeof(*kmb_imx327), GFP_KERNEL);
	if (!kmb_imx327)
		return -ENOMEM;

	platform_set_drvdata(pdev, kmb_imx327);

	mutex_init(&kmb_imx327->mutex);

	kmb_imx327->dev = &pdev->dev;

	/* Initialize subdev */
	v4l2_subdev_init(&kmb_imx327->sd, &kmb_imx327_subdev_ops);
	kmb_imx327->sd.owner = pdev->dev.driver->owner;
	kmb_imx327->sd.dev = &pdev->dev;

	/* request optional detect pins */
	detect_gpios =
		gpiod_get_array_optional(&pdev->dev, "detect", GPIOD_OUT_LOW);
	if (IS_ERR(detect_gpios)) {
		ret = PTR_ERR(detect_gpios);
		dev_info(&pdev->dev, "failed to get detect gpios %d", ret);
		/* Defer the probe if detect gpios are busy */
		ret = (ret == -EBUSY) ? -EPROBE_DEFER : ret;
		goto error_mutex_destroy;
	}

	ret = kmb_imx327_power_on(kmb_imx327);
	if (ret) {
		dev_dbg(&pdev->dev, "failed to power-on the sensor %d", ret);
		/* Defer the probe if resourcess are busy */
		ret = (ret == -EBUSY) ? -EPROBE_DEFER : ret;
		goto error_put_detect_gpios;
	}

	ret = kmb_imx327_get_i2c_client(kmb_imx327);
	if (ret) {
		dev_dbg(&pdev->dev, "failed to get i2c %d\n", ret);
		goto error_sensor_power_off;
	}

	v4l2_set_subdevdata(&kmb_imx327->sd, kmb_imx327->client);
	i2c_set_clientdata(kmb_imx327->client, &kmb_imx327->sd);
	v4l2_i2c_subdev_set_name(&kmb_imx327->sd, kmb_imx327->client,
				 KMB_IMX327_DRV_NAME, pdev->name);

	/* Check module identity */
	ret = kmb_imx327_detect(kmb_imx327);
	if (ret) {
		dev_err(&pdev->dev, "failed to find sensor: %d", ret);
		goto error_unregister_i2c_dev;
	}

	/* Set default mode to max resolution */
	kmb_imx327->cur_mode = &supported_modes[0];
	kmb_imx327->fps = kmb_imx327->cur_mode->fps.def;
	kmb_imx327->lpfr = kmb_imx327->cur_mode->lpfr;

	ret = kmb_imx327_init_controls(kmb_imx327);
	if (ret) {
		dev_err(&pdev->dev, "failed to init controls: %d", ret);
		goto error_unregister_i2c_dev;
	}

	/* Initialize subdev */
	kmb_imx327->sd.internal_ops = &kmb_imx327_internal_ops;
	kmb_imx327->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	kmb_imx327->sd.entity.ops = &kmb_imx327_subdev_entity_ops;
	kmb_imx327->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
	kmb_imx327->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&kmb_imx327->sd.entity, 1,
				     &kmb_imx327->pad);
	if (ret) {
		dev_err(&pdev->dev, "failed to init entity pads: %d", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor_common(&kmb_imx327->sd);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register async subdev: %d", ret);
		goto error_media_entity;
	}

	if (detect_gpios)
		gpiod_put_array(detect_gpios);

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_idle(&pdev->dev);

	dev_info(&pdev->dev, "Probe success!");
	return 0;

error_media_entity:
	media_entity_cleanup(&kmb_imx327->sd.entity);
error_handler_free:
	v4l2_ctrl_handler_free(kmb_imx327->sd.ctrl_handler);
error_unregister_i2c_dev:
	if (kmb_imx327->client)
		i2c_unregister_device(kmb_imx327->client);
error_sensor_power_off:
	kmb_imx327_power_off(kmb_imx327);
error_put_detect_gpios:
	if (detect_gpios)
		gpiod_put_array(detect_gpios);
error_mutex_destroy:
	mutex_destroy(&kmb_imx327->mutex);

	return ret;
}

/**
 * kmb_imx327_remove - Platform device unbinding
 * @pdev: pointer to platform device
 *
 * Return: 0 if successful
 */
static int kmb_imx327_pdev_remove(struct platform_device *pdev)
{
	struct kmb_imx327 *kmb_imx327 = platform_get_drvdata(pdev);
	struct v4l2_subdev *sd = &kmb_imx327->sd;

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(&pdev->dev);
	pm_runtime_suspended(&pdev->dev);

	if (kmb_imx327->client)
		i2c_unregister_device(kmb_imx327->client);

	mutex_destroy(&kmb_imx327->mutex);

	return 0;
}

static const struct dev_pm_ops kmb_imx327_platform_pm_ops = {
	SET_RUNTIME_PM_OPS(kmb_imx327_platform_suspend,
			   kmb_imx327_platform_resume, NULL)
};
static const struct of_device_id kmb_imx327_id_table[] = {
	{.compatible = "intel,kmb-imx327-sensor-p"},
	{}
};
MODULE_DEVICE_TABLE(of, kmb_imx327_id_table);

static struct platform_driver kmb_imx327_platform_driver = {
	.probe	= kmb_imx327_pdev_probe,
	.remove = kmb_imx327_pdev_remove,
	.driver = {
		.name = KMB_IMX327_DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &kmb_imx327_platform_pm_ops,
		.of_match_table = kmb_imx327_id_table,
	}
};

static int __init kmb_imx327_init(void)
{
	int ret;

	ret = i2c_add_driver(&kmb_imx327_i2c_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&kmb_imx327_platform_driver);
	if (ret)
		i2c_del_driver(&kmb_imx327_i2c_driver);

	return ret;
}

static void __exit kmb_imx327_exit(void)
{
	i2c_del_driver(&kmb_imx327_i2c_driver);
	platform_driver_unregister(&kmb_imx327_platform_driver);
}

module_init(kmb_imx327_init);
module_exit(kmb_imx327_exit);

MODULE_DESCRIPTION("Keem Bay imx327 Sensor driver");
MODULE_LICENSE("GPL v2");
