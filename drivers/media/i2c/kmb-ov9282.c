// SPDX-License-Identifier: GPL-2.0-only
/*
 * Keem Bay Camera ov9282 Sensor Driver.
 *
 * Copyright (C) 2020 Intel Corporation
 */

#include <asm/unaligned.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kmb-isp-ctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <linux/clk.h>

#define KMB_OV9282_DRV_NAME		"kmb-ov9282-sensor"

/* Streaming Mode */
#define KMB_OV9282_REG_MODE_SELECT	0x0100
#define KMB_OV9282_MODE_STANDBY		0x00
#define KMB_OV9282_MODE_STREAMING	0x01

/* SW reset */
#define KMB_OV9282_REG_SW_RESET		0x0103
#define KMB_OV9282_MODE_RESET		0x01
#define KMB_OV9282_MODE_OPERATING	0x00

/* Lines per frame */
#define KMB_OV9282_REG_LPFR		0x380E
#define KMB_OV9282_LPFR_MAX		0xFFFF

/* Chip ID */
#define KMB_OV9282_REG_ID		0x300A
#define KMB_OV9282_ID			0x9281

/* Exposure control */
#define KMB_OV9282_REG_EXPOSURE		0x3500
#define KMB_OV9282_EXPOSURE_MIN		1
#define KMB_OV9282_EXPOSURE_STEP	1
#define KMB_OV9282_EXPOSURE_DEFAULT	0x0282

/* Analog gain control */
#define KMB_OV9282_REG_AGAIN		0x3509
#define KMB_OV9282_AGAIN_MIN		0x10
#define KMB_OV9282_AGAIN_MAX		0xFF
#define KMB_OV9282_AGAIN_STEP		1
#define KMB_OV9282_AGAIN_DEFAULT	0x10

/* Group hold register */
#define KMB_OV9282_REG_HOLD		0x3308

/* Input clock rate */
#define KMB_OV9282_INCLK_RATE		24000000

/* Link frequency */
#define KMB_OV9282_LINK_FREQ_400MHz	400000000

#define KMB_OV9282_REG_MIN	0x00
#define KMB_OV9282_REG_MAX	0xFFFFF

/**
 * struct kmb_ov9282_reg - KMB ov9282 Sensor register
 * @address: Register address
 * @val: Register value
 */
struct kmb_ov9282_reg {
	u16 address;
	u8 val;
};

/**
 * struct kmb_ov9282_reg_list - KMB ov9282 Sensor register list
 * @num_of_regs: Number of registers in the list
 * @regs: Pointer to register list
 */
struct kmb_ov9282_reg_list {
	u32 num_of_regs;
	const struct kmb_ov9282_reg *regs;
};

/**
 * struct kmb_ov9282_mode - KMB ov9282 Sensor mode structure
 * @width: Frame width
 * @height: Frame height
 * @code: Format code
 * @ppln: Pixels per line
 * @lpfr: Lines per frame
 * @skip_lines: Top lines to be skipped
 * @pclk: Sensor pixel clock
 * @def: Default frames per second
 * @min: Min frames per second
 * @max: Max frames per second
 * @step: Frame rate step
 * @reg_list: Register list for sensor mode
 */
struct kmb_ov9282_mode {
	u32 width;
	u32 height;
	u32 code;
	u32 ppln;
	u32 lpfr;
	u32 skip_lines;
	u64 pclk;
	struct {
		u32 def;
		u32 min;
		u32 max;
		u32 step;
	} fps;
	struct kmb_ov9282_reg_list reg_list;
};

/**
 * struct kmb_ov9282 - KMB ov9282 Sensor device structure
 * @dev: pointer to generic device
 * @client: pointer to i2c client
 * @sd: V4L2 sub-device
 * @pad: Media pad. Only one pad supported
 * @reset_gpio: Sensor reset gpio
 * @inclk: Sensor input clock
 * @ctrl_handler: V4L2 control handler
 * @pclk_ctrl: Pointer to pixel clock control
 * @link_freq_ctrl: Pointer to link frequency control
 * @hblank_ctrl: Pointer to horizontal blanking control
 * @vblank_ctrl: Pointer to vertical blanking control
 * @exp_ctrl: Pointer to exposure control
 * @again_ctrl: Pointer to analog gain control
 * @sync_enable_ctrl: Pointer to hardware sync enable control
 * @sync_start_ctrl: Pointer to sync start control
 * @fps: FPS to be applied on next stream on
 * @lpfr: Lines per frame for long exposure frame
 * @cur_mode: Current selected sensor mode
 * @mutex: Mutex for serializing sensor controls
 * @streaming: Flag indicating streaming state
 * @sync_mode: HW sync mode
 */
struct kmb_ov9282 {
	struct device *dev;
	struct i2c_client *client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct gpio_desc *reset_gpio;
	struct clk *inclk;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pclk_ctrl;
	struct v4l2_ctrl *link_freq_ctrl;
	struct v4l2_ctrl *hblank_ctrl;
	struct v4l2_ctrl *vblank_ctrl;
	struct {
		struct v4l2_ctrl *exp_ctrl;
		struct v4l2_ctrl *again_ctrl;
	};
	struct v4l2_ctrl *sync_enable_ctrl;
	struct v4l2_ctrl *sync_start_ctrl;
	u32 fps;
	u32 lpfr;
	const struct kmb_ov9282_mode *cur_mode;
	struct mutex mutex;
	bool streaming;
	enum kmb_hw_sync_mode sync_mode;
};

static const s64 link_freq[] = {
	KMB_OV9282_LINK_FREQ_400MHz,
};

/* Sensor mode registers */
static const struct kmb_ov9282_reg mode_1280x720_regs[] = {
	{0x0302, 0x32},
	{0x030D, 0x50},
	{0x030E, 0x02},
	{0x3001, 0x00},
	{0x3004, 0x00},
	{0x3005, 0x00},
	{0x3006, 0x04},
	{0x3011, 0x0A},
	{0x3013, 0x18},
	{0x301c, 0xf0},
	{0x3022, 0x01},
	{0x3030, 0x10},
	{0x3039, 0x32},
	{0x303A, 0x00},
	{0x3500, 0x00},
	{0x3501, 0x5F},
	{0x3502, 0x1e},
	{0x3503, 0x08},
	{0x3505, 0x8c},
	{0x3507, 0x03},
	{0x3508, 0x00},
	{0x3509, 0x10},
	{0x3610, 0x80},
	{0x3611, 0xa0},
	{0x3620, 0x6e},
	{0x3632, 0x56},
	{0x3633, 0x78},
	{0x3666, 0x00},
	{0x366f, 0x5a},
	{0x3680, 0x84},
	{0x3712, 0x80},
	{0x372d, 0x22},
	{0x3731, 0x80},
	{0x3732, 0x30},
	{0x3778, 0x00},
	{0x377d, 0x22},
	{0x3788, 0x02},
	{0x3789, 0xa4},
	{0x378a, 0x00},
	{0x378b, 0x4a},
	{0x3799, 0x20},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3804, 0x05},
	{0x3805, 0x0F},
	{0x3806, 0x02},
	{0x3807, 0xdF},
	{0x3808, 0x05},
	{0x3809, 0x00},
	{0x380a, 0x02},
	{0x380b, 0xd0},
	{0x380c, 0x05},
	{0x380d, 0xFA},
	{0x380e, 0x06},
	{0x380f, 0xce},
	{0x3810, 0x00},
	{0x3811, 0x08},
	{0x3812, 0x00},
	{0x3813, 0x08},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3820, 0x3C},
	{0x3821, 0x84},
	{0x3881, 0x42},
	{0x38a8, 0x02},
	{0x38a9, 0x80},
	{0x38b1, 0x00},
	{0x38c4, 0x00},
	{0x38c5, 0xc0},
	{0x38c6, 0x04},
	{0x38c7, 0x80},
	{0x3920, 0xff},
	{0x4003, 0x40},
	{0x4008, 0x02},
	{0x4009, 0x05},
	{0x400c, 0x00},
	{0x400d, 0x03},
	{0x4010, 0x40},
	{0x4043, 0x40},
	{0x4307, 0x30},
	{0x4317, 0x00},
	{0x4501, 0x00},
	{0x4507, 0x00},
	{0x4509, 0x80},
	{0x450a, 0x08},
	{0x4601, 0x04},
	{0x470f, 0x00},
	{0x4f07, 0x00},
	{0x4800, 0x20},
	{0x5000, 0x9f},
	{0x5001, 0x00},
	{0x5e00, 0x00},
	{0x5d00, 0x07},
	{0x5d01, 0x00},
	{0x0101, 0x01},
	{0x1000, 0x03},
	{0x5a08, 0x84},
};

static const struct kmb_ov9282_reg kmb_ov9282_master[] = {
	{0x3006, 0x02},
	{0x3666, 0x00},
	{0x3823, 0x00},
};

static const struct kmb_ov9282_reg kmb_ov9282_slave[] = {
	{0x3006, 0x00},
	{0x3666, 0x00},
	{0x3823, 0x30},
};

/* Supported sensor mode configurations */
static const struct kmb_ov9282_mode supported_modes[] = {
	{
		.width = 1280,
		.height = 720,
		.ppln = 1530,
		.lpfr = 1742,
		.skip_lines = 0,
		.pclk = 80000000,
		.code = MEDIA_BUS_FMT_Y10_1X10,
		.fps = {
			.def = 30,
			.max = 60,
			.min = 1,
			.step = 5,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1280x720_regs),
			.regs = mode_1280x720_regs,
		},
	},
};

/**
 * to_kmb_ov9282 - ov9282 V4L2 sub-device to kmb_ov9282 device.
 * @subdev: pointer to ov9282 V4L2 sub-device device
 *
 * Return: Pointer to kmb_ov9282 device
 */
static inline struct kmb_ov9282 *to_kmb_ov9282(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct kmb_ov9282, sd);
}

/**
 * kmb_ov9282_read_reg - Read registers.
 * @kmb_ov9282: pointer to ov9282 device
 * @reg: Register address
 * @len: Length of bytes to read. Max supported bytes is 4
 * @val: Pointer to register value to be filled.
 *
 * Return: 0 if successful
 */
static int
kmb_ov9282_read_reg(struct kmb_ov9282 *kmb_ov9282, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&kmb_ov9282->sd);
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;

	if (len > 4)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

/**
 * kmb_ov9282_write_reg - Write register
 * @kmb_ov9282: pointer to ov9282 device
 * @reg: Register address
 * @len: Length of bytes. Max supported bytes is 4
 * @val: Register value
 *
 * Return: 0 if successful
 */
static int
kmb_ov9282_write_reg(struct kmb_ov9282 *kmb_ov9282, u16 reg, u32 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&kmb_ov9282->sd);
	int buf_i, val_i;
	u8 buf[6], *val_p;
	__be32 val_be;
	int ret;

	if (len > 4) {
		dev_err_ratelimited(kmb_ov9282->dev,
				    "write reg 0x%4.4x invalid len %d",
				    reg, len);
		return -EINVAL;
	}

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	ret = i2c_master_send(client, buf, len + 2);
	if (ret != len + 2) {
		dev_err_ratelimited(kmb_ov9282->dev,
				"write reg 0x%4.4x return err %d",
				reg, ret);
		return -EIO;
	}

	return 0;
}

/**
 * kmb_ov9282_write_regs - Write a list of registers
 * @kmb_ov9282: pointer to ov9282 device
 * @regs: List of registers to be written
 * @len: Length of registers array
 *
 * Return: 0 if successful
 */
static int kmb_ov9282_write_regs(struct kmb_ov9282 *kmb_ov9282,
			     const struct kmb_ov9282_reg *regs, u32 len)
{
	int ret;
	u32 i;

	for (i = 0; i < len; i++) {
		ret = kmb_ov9282_write_reg(kmb_ov9282,
					   regs[i].address,
					   1, regs[i].val);
		if (ret)
			return ret;
	}

	return 0;
}

/**
 * kmb_ov9282_update_fps - Update current sensor mode to match the selected FPS
 * @kmb_ov9282: pointer to ov9282 device
 *
 * Return: none
 */
static void kmb_ov9282_update_fps(struct kmb_ov9282 *kmb_ov9282)
{
	u32 lpfr = (kmb_ov9282->cur_mode->lpfr *
		    kmb_ov9282->cur_mode->fps.def) / kmb_ov9282->fps;

	if (lpfr > KMB_OV9282_LPFR_MAX)
		lpfr = KMB_OV9282_LPFR_MAX;

	/* no info in documentation */
	if (lpfr < kmb_ov9282->cur_mode->height + 12)
		lpfr = kmb_ov9282->cur_mode->height + 12;

	kmb_ov9282->lpfr = lpfr;

	dev_dbg(kmb_ov9282->dev, "Selected FPS %d lpfr %d",
		kmb_ov9282->fps, lpfr);
}

/**
 * kmb_ov9282_open - Open ov9282 subdevice
 * @sd: pointer to ov9282 V4L2 sub-device structure
 * @fh: pointer to ov9282 V4L2 sub-device file handle
 *
 * Return: 0 if successful
 */
static int kmb_ov9282_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct kmb_ov9282 *kmb_ov9282 = to_kmb_ov9282(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->pad, 0);

	mutex_lock(&kmb_ov9282->mutex);

	/* Initialize try_fmt */
	try_fmt->width = kmb_ov9282->cur_mode->width;
	try_fmt->height = kmb_ov9282->cur_mode->height;
	try_fmt->code = kmb_ov9282->cur_mode->code;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&kmb_ov9282->mutex);

	return 0;
}

/**
 * kmb_ov9282_get_camera_mode_by_fmt - Get the most appropriate camera
 *         mode that meets the code and resolution criteria
 * @kmb_ov9282: pointer to kmb_ov9282 device
 * @code: media bus format code
 * @width: frame width
 * @height: frame height
 *
 * Return: pointer to the most appropriate camera mode
 */
static const struct kmb_ov9282_mode *
kmb_ov9282_get_camera_mode_by_fmt(struct kmb_ov9282 *kmb_ov9282, u32 code,
				  u32 width, u32 height)
{
	const struct kmb_ov9282_mode *mode = supported_modes;
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
 * kmb_ov9282_set_ctrl - Set subdevice control. Supported controls:
 *                       V4L2_CID_ANALOGUE_GAIN
 *                       V4L2_CID_EXPOSURE
 *                       Both controls are in one cluster.
 *
 * @ctrl: pointer to v4l2_ctrl structure
 *
 * Return: 0 if successful
 */
static int kmb_ov9282_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct kmb_ov9282 *kmb_ov9282 =
		container_of(ctrl->handler, struct kmb_ov9282, ctrl_handler);
	u32 exposure;
	u32 analog_gain;
	u32 lpfr = kmb_ov9282->lpfr;
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_SYNC_MODE:
		kmb_ov9282->sync_mode = ctrl->val;
		dev_dbg(kmb_ov9282->dev, "V4L2_CID_SYNC_MODE %d", ctrl->val);
		return 0;
	case V4L2_CID_SYNC_START:
		if (kmb_ov9282->sync_mode != KMB_HW_SYNC_NONE) {
			ret = kmb_ov9282_write_reg(kmb_ov9282,
						KMB_OV9282_REG_MODE_SELECT,
						1, KMB_OV9282_MODE_STREAMING);
			if (ret)
				dev_err(kmb_ov9282->dev,
					"Fail to start streaming");
		}
		dev_dbg(kmb_ov9282->dev, "V4L2_CID_SYNC_START");
		return 0;
	}

	/* Set exposure and gain only if sensor is in power on state */
	if (!pm_runtime_get_if_in_use(kmb_ov9282->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		/* Handle the cluster for both controls */
		exposure = ctrl->val;
		analog_gain = kmb_ov9282->again_ctrl->val;
		break;
	default:
		dev_dbg(kmb_ov9282->dev, "Invalid control %d", ctrl->id);
		ret = -EINVAL;
		goto error_pm_runtime_put;
	}

	if (exposure > (lpfr - 12))
		exposure = lpfr - 12;
	else if (exposure < 1)
		exposure = 1;

	dev_dbg(kmb_ov9282->dev, "Set exposure %d analog gain %x lpfr %d",
		exposure, analog_gain, lpfr);

	ret = kmb_ov9282_write_reg(kmb_ov9282, KMB_OV9282_REG_HOLD, 1, 1);
	if (ret)
		goto error_pm_runtime_put;

	ret = kmb_ov9282_write_reg(kmb_ov9282, KMB_OV9282_REG_LPFR,
				   2, lpfr);
	if (ret)
		goto error_release_group_hold;

	ret = kmb_ov9282_write_reg(kmb_ov9282, KMB_OV9282_REG_EXPOSURE,
				   3, exposure << 4);
	if (ret)
		goto error_release_group_hold;

	ret = kmb_ov9282_write_reg(kmb_ov9282, KMB_OV9282_REG_AGAIN,
				   1, analog_gain);
	if (ret)
		goto error_release_group_hold;

	kmb_ov9282_write_reg(kmb_ov9282, KMB_OV9282_REG_HOLD, 1, 0);

	pm_runtime_put(kmb_ov9282->dev);

	return 0;

error_release_group_hold:
	kmb_ov9282_write_reg(kmb_ov9282, KMB_OV9282_REG_HOLD, 1, 0);
error_pm_runtime_put:
	pm_runtime_put(kmb_ov9282->dev);
	return ret;
}

/* V4l2 subdevice control ops*/
static const struct v4l2_ctrl_ops kmb_ov9282_ctrl_ops = {
	.s_ctrl = kmb_ov9282_set_ctrl,
};

static const struct v4l2_ctrl_config hw_sync_enable = {
	.ops = &kmb_ov9282_ctrl_ops,
	.id = V4L2_CID_SYNC_MODE,
	.name = "V4L2_CID_SYNC_MODE",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = KMB_HW_SYNC_NONE,
	.max = KMB_HW_SYNC_SLAVE,
	.def = KMB_HW_SYNC_NONE,
	.step = 1,
};

static const struct v4l2_ctrl_config hw_sync_start = {
	.ops = &kmb_ov9282_ctrl_ops,
	.id = V4L2_CID_SYNC_START,
	.name = "V4L2_CID_SYNC_START",
	.type = V4L2_CTRL_TYPE_BUTTON,
};

/**
 * kmb_ov9282_enum_mbus_code - Enumerate V4L2 sub-device mbus codes
 * @sd: pointer to ov9282 V4L2 sub-device structure
 * @cfg: V4L2 sub-device pad configuration
 * @fsize: V4L2 sub-device size enumeration need to be filled
 *
 * Return: 0 if successful
 */
static int kmb_ov9282_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct kmb_ov9282 *kmb_ov9282 = to_kmb_ov9282(sd);

	if (code->index > 0)
		return -EINVAL;

	mutex_lock(&kmb_ov9282->mutex);
	code->code = supported_modes[0].code;
	mutex_unlock(&kmb_ov9282->mutex);

	return 0;
}

/**
 * kmb_ov9282_enum_frame_size - Enumerate V4L2 sub-device frame sizes
 * @sd: pointer to ov9282 V4L2 sub-device structure
 * @cfg: V4L2 sub-device pad configuration
 * @fsize: V4L2 sub-device size enumeration need to be filled
 *
 * Return: 0 if successful
 */
static int kmb_ov9282_enum_frame_size(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_frame_size_enum *fsize)
{
	struct kmb_ov9282 *kmb_ov9282 = to_kmb_ov9282(sd);

	if (fsize->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	mutex_lock(&kmb_ov9282->mutex);
	if (fsize->code != supported_modes[fsize->index].code) {
		mutex_unlock(&kmb_ov9282->mutex);
		return -EINVAL;
	}
	mutex_unlock(&kmb_ov9282->mutex);

	fsize->min_width = supported_modes[fsize->index].width;
	fsize->max_width = fsize->min_width;
	fsize->min_height = supported_modes[fsize->index].height;
	fsize->max_height = fsize->min_height;

	return 0;
}

/**
 * kmb_ov9282_enum_frame_interval - Enumerate V4L2 sub-device frame intervals
 * @sd: pointer to ov9282 V4L2 sub-device structure
 * @cfg: V4L2 sub-device pad configuration
 * @fsize: V4L2 sub-device size enumeration need to be filled
 *
 * callback for VIDIOC_SUBDEV_ENUM_FRAME_INTERVAL()
 *
 * Return: 0 if successful
 */
static int
kmb_ov9282_enum_frame_interval(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_frame_interval_enum *fie)
{
	struct kmb_ov9282 *kmb_ov9282 = to_kmb_ov9282(sd);
	const struct kmb_ov9282_mode *mode;
	int fps;
	int ret = 0;

	if (fie->pad)
		return -EINVAL;

	mutex_lock(&kmb_ov9282->mutex);

	if (fie->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		if (fie->code != kmb_ov9282->cur_mode->code ||
		    fie->width != kmb_ov9282->cur_mode->width ||
		    fie->height != kmb_ov9282->cur_mode->height) {
			ret = -EINVAL;
			goto exit_unlock;
		}

		mode = kmb_ov9282->cur_mode;
	} else {
		mode = kmb_ov9282_get_camera_mode_by_fmt(kmb_ov9282,
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

	dev_dbg(kmb_ov9282->dev, "Enum FPS %d %d/%d", fps,
		fie->interval.numerator, fie->interval.denominator);

exit_unlock:
	mutex_unlock(&kmb_ov9282->mutex);
	return ret;
}

/**
 * kmb_ov9282_fill_pad_format - Fill subdevice pad format
 *                              from selected sensor mode
 * @kmb_ov9282: pointer to kmb_ov9282 device
 * @mode: Pointer to kmb_ov9282_mode sensor mode
 * @fmt: V4L2 sub-device format need to be filled
 *
 * Return: none
 */
static void kmb_ov9282_fill_pad_format(struct kmb_ov9282 *kmb_ov9282,
				       const struct kmb_ov9282_mode *mode,
				       struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.code = mode->code;
	fmt->format.field = V4L2_FIELD_NONE;
}

/**
 * kmb_ov9282_skip_top_lines - Skip top lines containing metadata
 * @sd: pointer to OV9282 V4L2 sub-device structure
 * @lines: number of lines to be skipped
 *
 * Return: 0 if successful
 */
static int kmb_ov9282_skip_top_lines(struct v4l2_subdev *sd, u32 *lines)
{
	struct kmb_ov9282 *kmb_ov9282 = to_kmb_ov9282(sd);

	*lines = kmb_ov9282->cur_mode->skip_lines;

	return 0;
}


/**
 * kmb_ov9282_update_controls - Update control ranges based on streaming mode
 * @kmb_ov9282: pointer to kmb_ov9282 device
 * @mode: pointer to kmb_ov9282_mode sensor mode
 *
 * Return: none
 */
static void kmb_ov9282_update_controls(struct kmb_ov9282 *kmb_ov9282,
					const struct kmb_ov9282_mode *mode)
{
	__v4l2_ctrl_s_ctrl(kmb_ov9282->link_freq_ctrl, 0);
	__v4l2_ctrl_s_ctrl(kmb_ov9282->vblank_ctrl,
			   kmb_ov9282->lpfr - mode->height);
	__v4l2_ctrl_s_ctrl(kmb_ov9282->hblank_ctrl,
			   mode->ppln - mode->width);
	__v4l2_ctrl_modify_range(kmb_ov9282->pclk_ctrl,
				mode->pclk, mode->pclk,
				1, mode->pclk);
	__v4l2_ctrl_modify_range(kmb_ov9282->exp_ctrl,
				KMB_OV9282_EXPOSURE_MIN,
				kmb_ov9282->lpfr - 12,
				1, KMB_OV9282_EXPOSURE_DEFAULT);
}

/**
 * kmb_ov9282_get_pad_format - Get subdevice pad format
 * @sd: pointer to ov9282 V4L2 sub-device structure
 * @cfg: V4L2 sub-device pad configuration
 * @fmt: V4L2 sub-device format need to be set
 *
 * Return: 0 if successful
 */
static int kmb_ov9282_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_format *fmt)
{
	struct kmb_ov9282 *kmb_ov9282 = to_kmb_ov9282(sd);

	mutex_lock(&kmb_ov9282->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *framefmt;

		framefmt = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		fmt->format = *framefmt;
	} else {
		kmb_ov9282_fill_pad_format(kmb_ov9282,
					   kmb_ov9282->cur_mode,
					   fmt);
	}

	mutex_unlock(&kmb_ov9282->mutex);

	return 0;
}

/**
 * kmb_ov9282_set_pad_format - Set subdevice pad format
 * @sd: pointer to ov9282 V4L2 sub-device structure
 * @cfg: V4L2 sub-device pad configuration
 * @fmt: V4L2 sub-device format need to be set
 *
 * Return: 0 if successful
 */
static int kmb_ov9282_set_pad_format(struct v4l2_subdev *sd,
				     struct v4l2_subdev_pad_config *cfg,
				     struct v4l2_subdev_format *fmt)
{
	struct kmb_ov9282 *kmb_ov9282 = to_kmb_ov9282(sd);
	const struct kmb_ov9282_mode *mode;
	struct v4l2_mbus_framefmt *framefmt;

	mutex_lock(&kmb_ov9282->mutex);

	/* Currently only one format is supported */
	fmt->format.code = supported_modes[0].code;

	mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);

	kmb_ov9282->fps = mode->fps.def;
	kmb_ov9282->lpfr = mode->lpfr;

	kmb_ov9282_update_fps(kmb_ov9282);

	kmb_ov9282_fill_pad_format(kmb_ov9282, mode, fmt);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		framefmt = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		*framefmt = fmt->format;
	} else {
		kmb_ov9282_update_controls(kmb_ov9282, mode);
		kmb_ov9282->cur_mode = mode;
	}

	mutex_unlock(&kmb_ov9282->mutex);

	return 0;
}

/**
 * kmb_ov9282_start_streaming - Start sensor stream
 * @kmb_ov9282: pointer to kmb_ov9282 device
 *
 * Return: 0 if successful
 */
static int kmb_ov9282_start_streaming(struct kmb_ov9282 *kmb_ov9282)
{
	const struct kmb_ov9282_reg_list *reg_list;
	int ret;

	/* Write sensor mode registers */
	reg_list = &kmb_ov9282->cur_mode->reg_list;
	ret = kmb_ov9282_write_regs(kmb_ov9282, reg_list->regs,
				    reg_list->num_of_regs);
	if (ret) {
		dev_err(kmb_ov9282->dev, "fail to write initial registers");
		return ret;
	}

	switch (kmb_ov9282->sync_mode) {
	case KMB_HW_SYNC_MASTER:
		ret = kmb_ov9282_write_regs(kmb_ov9282, kmb_ov9282_master,
				ARRAY_SIZE(kmb_ov9282_master));
		if (ret) {
			dev_err(kmb_ov9282->dev,
				"fail to write master registers");
			return ret;
		}
		break;
	case KMB_HW_SYNC_SLAVE:
		ret = kmb_ov9282_write_regs(kmb_ov9282, kmb_ov9282_slave,
				ARRAY_SIZE(kmb_ov9282_slave));
		if (ret) {
			dev_err(kmb_ov9282->dev,
				"fail to write slave registers");
			return ret;
		}
		break;
	default:
		break;
	}

	/* Setup handler will write actual exposure and gain */
	ret =  __v4l2_ctrl_handler_setup(kmb_ov9282->sd.ctrl_handler);
	if (ret) {
		dev_err(kmb_ov9282->dev, "fail to setup handler");
		return ret;
	}

	/* Start streaming only when hw sync is not enabled  */
	if (kmb_ov9282->sync_mode == KMB_HW_SYNC_NONE) {
		ret = kmb_ov9282_write_reg(kmb_ov9282,
					KMB_OV9282_REG_MODE_SELECT,
					1, KMB_OV9282_MODE_STREAMING);
		if (ret) {
			dev_err(kmb_ov9282->dev, "fail to start streaming");
			return ret;
		}
	}

	return 0;
}

/**
 * kmb_ov9282_stop_streaming - Stop sensor stream
 * @kmb_ov9282: pointer to kmb_ov9282 device
 *
 * Return: 0 if successful
 */
static int kmb_ov9282_stop_streaming(struct kmb_ov9282 *kmb_ov9282)
{
	return kmb_ov9282_write_reg(kmb_ov9282, KMB_OV9282_REG_MODE_SELECT,
				    1, KMB_OV9282_MODE_STANDBY);
}

/**
 * kmb_ov9282_set_stream - Enable sensor streaming
 * @sd: pointer to ov9282 subdevice
 * @enable: Set to enable sensor streaming
 *
 * Return: 0 if successful
 */
static int kmb_ov9282_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct kmb_ov9282 *kmb_ov9282 = to_kmb_ov9282(sd);
	int ret;

	mutex_lock(&kmb_ov9282->mutex);

	if (kmb_ov9282->streaming == enable) {
		mutex_unlock(&kmb_ov9282->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_get_sync(kmb_ov9282->dev);
		if (ret)
			goto error_unlock;

		ret = kmb_ov9282_start_streaming(kmb_ov9282);
		if (ret)
			goto error_power_off;
	} else {
		kmb_ov9282_stop_streaming(kmb_ov9282);
		pm_runtime_put(kmb_ov9282->dev);
	}

	kmb_ov9282->streaming = enable;

	mutex_unlock(&kmb_ov9282->mutex);

	return 0;

error_power_off:
	pm_runtime_put(kmb_ov9282->dev);
error_unlock:
	mutex_unlock(&kmb_ov9282->mutex);
	return ret;
}

/**
 * kmb_ov9282_get_frame_interval - Get subdevice frame interval
 * @sd: pointer to ov9282 V4L2 sub-device structure
 * @interval: V4L2 sub-device current farme interval
 *
 * Return: 0 if successful
 */
static int kmb_ov9282_get_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *interval)
{
	struct kmb_ov9282 *kmb_ov9282 = to_kmb_ov9282(sd);

	if (interval->pad)
		return -EINVAL;

	mutex_lock(&kmb_ov9282->mutex);
	interval->interval.numerator = 1;
	interval->interval.denominator = kmb_ov9282->fps;
	mutex_unlock(&kmb_ov9282->mutex);

	dev_dbg(kmb_ov9282->dev, "Get frame interval %d/%d",
		interval->interval.numerator,
		interval->interval.denominator);

	return 0;
}

/**
 * kmb_ov9282_set_frame_interval - Set subdevice frame interval
 * @sd: pointer to ov9282 V4L2 sub-device structure
 * @interval: V4L2 sub-device farme interval to be set
 *
 * Return: 0 if successful
 */
static int kmb_ov9282_set_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *interval)
{
	struct kmb_ov9282 *kmb_ov9282 = to_kmb_ov9282(sd);
	u32 fps;

	if (interval->pad)
		return -EINVAL;

	mutex_lock(&kmb_ov9282->mutex);
	fps = (u32)(interval->interval.denominator /
		interval->interval.numerator);
	if (fps < kmb_ov9282->cur_mode->fps.min) {
		interval->interval.numerator = 1;
		interval->interval.denominator = kmb_ov9282->cur_mode->fps.min;
	} else if (fps > kmb_ov9282->cur_mode->fps.max) {
		interval->interval.numerator = 1;
		interval->interval.denominator = kmb_ov9282->cur_mode->fps.max;
	}

	kmb_ov9282->fps = (u32) (interval->interval.denominator /
				     interval->interval.numerator);

	kmb_ov9282_update_fps(kmb_ov9282);

	mutex_unlock(&kmb_ov9282->mutex);

	dev_dbg(kmb_ov9282->dev, "Set frame interval %d/%d",
		interval->interval.numerator,
		interval->interval.denominator);

	return 0;
}

/**
 * kmb_ov9282_power_on - Sensor power on sequence
 * @kmb_ov9282: imb_ov9282 device
 *
 * Return: 0 if successful
 */
static int kmb_ov9282_power_on(struct kmb_ov9282 *kmb_ov9282)
{
	int ret;

	/* request optional reset pin */
	kmb_ov9282->reset_gpio =
		gpiod_get_optional(kmb_ov9282->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(kmb_ov9282->reset_gpio)) {
		ret = PTR_ERR(kmb_ov9282->reset_gpio);
		dev_err(kmb_ov9282->dev, "failed to get reset gpio %d", ret);
		return ret;
	}

	if (kmb_ov9282->reset_gpio)
		gpiod_set_value_cansleep(kmb_ov9282->reset_gpio, 1);

	ret = clk_prepare_enable(kmb_ov9282->inclk);
	if (ret) {
		dev_err(kmb_ov9282->dev, "fail to enable inclk\n");
		goto error_reset;
	}

	usleep_range(18000, 20000);

	return 0;

error_reset:
	if (kmb_ov9282->reset_gpio) {
		gpiod_set_value_cansleep(kmb_ov9282->reset_gpio, 0);
		gpiod_put(kmb_ov9282->reset_gpio);
		kmb_ov9282->reset_gpio = NULL;
	}

	return ret;
}

/**
 * kmb_ov9282_power_off - Sensor power off sequence
 * @kmb_ov9282: imb_ov9282 device
 */
static void kmb_ov9282_power_off(struct kmb_ov9282 *kmb_ov9282)
{
	clk_disable_unprepare(kmb_ov9282->inclk);

	if (kmb_ov9282->reset_gpio) {
		gpiod_set_value_cansleep(kmb_ov9282->reset_gpio, 0);
		gpiod_put(kmb_ov9282->reset_gpio);
		kmb_ov9282->reset_gpio = NULL;

	}
}

/**
 * kmb_ov9282_detect - Detect ov9282 sensor
 * @kmb_ov9282: pointer to kmb_ov9282 device
 *
 * Return: 0 if successful, -EIO if sensor id does not match
 */
static int kmb_ov9282_detect(struct kmb_ov9282 *kmb_ov9282)
{
	int ret;
	u32 val = 0;

	ret = kmb_ov9282_read_reg(kmb_ov9282, KMB_OV9282_REG_ID, 2, &val);
	if (ret)
		return ret;

	if (val != KMB_OV9282_ID) {
		dev_err(kmb_ov9282->dev, "chip id mismatch: %x!=%x",
			KMB_OV9282_ID, val);
		return -EIO;
	}
	return 0;
}

/* V4l2 subdevice ops */
static const struct v4l2_subdev_video_ops kmb_ov9282_video_ops = {
	.s_stream = kmb_ov9282_set_stream,
	.g_frame_interval = kmb_ov9282_get_frame_interval,
	.s_frame_interval = kmb_ov9282_set_frame_interval,
};

static const struct v4l2_subdev_pad_ops kmb_ov9282_pad_ops = {
	.enum_mbus_code = kmb_ov9282_enum_mbus_code,
	.get_fmt = kmb_ov9282_get_pad_format,
	.set_fmt = kmb_ov9282_set_pad_format,
	.enum_frame_size = kmb_ov9282_enum_frame_size,
	.enum_frame_interval = kmb_ov9282_enum_frame_interval,
};

static const struct v4l2_subdev_sensor_ops kmb_ov9282_sensor_ops = {
	.g_skip_top_lines = kmb_ov9282_skip_top_lines,
};

static const struct v4l2_subdev_ops kmb_ov9282_subdev_ops = {
	.video = &kmb_ov9282_video_ops,
	.pad = &kmb_ov9282_pad_ops,
	.sensor = &kmb_ov9282_sensor_ops,
};

static const struct media_entity_operations kmb_ov9282_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops kmb_ov9282_internal_ops = {
	.open = kmb_ov9282_open,
};

/**
 * kmb_ov9282_init_controls - Initialize sensor subdevice controls
 * @kmb_ov9282: pointer to kmb_ov9282 device
 *
 * Return: 0 if successful
 */
static int kmb_ov9282_init_controls(struct kmb_ov9282 *kmb_ov9282)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &kmb_ov9282->ctrl_handler;
	const struct kmb_ov9282_mode *mode = kmb_ov9282->cur_mode;
	u32 hblank;
	u32 vblank;
	int ret;

	ctrl_hdlr = &kmb_ov9282->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 8);
	if (ret)
		return ret;

	/* Serialize controls with sensor device */
	ctrl_hdlr->lock = &kmb_ov9282->mutex;

	/* Initialize exposure and gain */
	kmb_ov9282->exp_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						 &kmb_ov9282_ctrl_ops,
						 V4L2_CID_EXPOSURE,
						 KMB_OV9282_EXPOSURE_MIN,
						 mode->lpfr,
						 KMB_OV9282_EXPOSURE_STEP,
						 KMB_OV9282_EXPOSURE_DEFAULT);

	kmb_ov9282->again_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						   &kmb_ov9282_ctrl_ops,
						   V4L2_CID_ANALOGUE_GAIN,
						   KMB_OV9282_AGAIN_MIN,
						   KMB_OV9282_AGAIN_MAX,
						   KMB_OV9282_AGAIN_STEP,
						   KMB_OV9282_AGAIN_DEFAULT);
	v4l2_ctrl_cluster(2, &kmb_ov9282->exp_ctrl);

	/* Read only controls */
	kmb_ov9282->pclk_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						  &kmb_ov9282_ctrl_ops,
						  V4L2_CID_PIXEL_RATE,
						  mode->pclk, mode->pclk,
						  1, mode->pclk);
	if (kmb_ov9282->pclk_ctrl)
		kmb_ov9282->pclk_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	kmb_ov9282->link_freq_ctrl = v4l2_ctrl_new_int_menu(ctrl_hdlr,
						&kmb_ov9282_ctrl_ops,
						V4L2_CID_LINK_FREQ,
						ARRAY_SIZE(link_freq) - 1,
						0, link_freq);
	if (kmb_ov9282->link_freq_ctrl)
		kmb_ov9282->link_freq_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank = mode->lpfr - mode->height;
	kmb_ov9282->vblank_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						    &kmb_ov9282_ctrl_ops,
						    V4L2_CID_VBLANK,
						    KMB_OV9282_REG_MIN,
						    KMB_OV9282_REG_MAX,
						    1, vblank);
	if (kmb_ov9282->vblank_ctrl)
		kmb_ov9282->vblank_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	hblank = mode->ppln - mode->width;
	kmb_ov9282->hblank_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						    &kmb_ov9282_ctrl_ops,
						    V4L2_CID_HBLANK,
						    KMB_OV9282_REG_MIN,
						    KMB_OV9282_REG_MAX,
						    1, hblank);
	if (kmb_ov9282->hblank_ctrl)
		kmb_ov9282->hblank_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* HW sync controls */
	kmb_ov9282->sync_enable_ctrl = v4l2_ctrl_new_custom(ctrl_hdlr,
							    &hw_sync_enable,
							    NULL);
	kmb_ov9282->sync_start_ctrl = v4l2_ctrl_new_custom(ctrl_hdlr,
							   &hw_sync_start,
							   NULL);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(kmb_ov9282->dev, "control init failed: %d", ret);
		goto error;
	}

	kmb_ov9282->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	return ret;
}

/* --------------- probe as i2c device -------------------- */

/**
 * kmb_ov9282_i2c_resume - PM resume callback
 * @dev: i2c device
 *
 * Return: 0 if successful
 */
static int kmb_ov9282_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct kmb_ov9282 *kmb_ov9282 = to_kmb_ov9282(sd);

	return kmb_ov9282_power_on(kmb_ov9282);
}

/**
 * kmb_ov9282_i2c_suspend - PM suspend callback
 * @dev: i2c device
 *
 * Return: 0 if successful
 */
static int kmb_ov9282_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct kmb_ov9282 *kmb_ov9282 = to_kmb_ov9282(sd);

	kmb_ov9282_power_off(kmb_ov9282);

	return 0;
}

/**
 * kmb_ov9282_i2c_probe - I2C client device binding
 * @client: pointer to i2c client device
 * @id: pointer to i2c device id
 *
 * Return: 0 if successful
 */
static int kmb_ov9282_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct kmb_ov9282 *kmb_ov9282;
	int ret;

	kmb_ov9282 = devm_kzalloc(&client->dev, sizeof(*kmb_ov9282),
				  GFP_KERNEL);
	if (!kmb_ov9282)
		return -ENOMEM;

	mutex_init(&kmb_ov9282->mutex);

	kmb_ov9282->dev = &client->dev;

	/* Initialize subdev */
	v4l2_i2c_subdev_init(&kmb_ov9282->sd, client, &kmb_ov9282_subdev_ops);

	kmb_ov9282->reset_gpio = NULL;

	/* Get sensor input clock */
	kmb_ov9282->inclk = devm_clk_get(&client->dev, "inclk");
	if (IS_ERR(kmb_ov9282->inclk)) {
		ret = PTR_ERR(kmb_ov9282->inclk);
		dev_err(&client->dev, "could not get inclk");
		goto error_mutex_destroy;
	}

	ret = clk_set_rate(kmb_ov9282->inclk, KMB_OV9282_INCLK_RATE);
	if (ret) {
		dev_err(&client->dev, "could not set inclk frequency\n");
		goto error_mutex_destroy;
	}

	ret = kmb_ov9282_power_on(kmb_ov9282);
	if (ret) {
		dev_err(&client->dev, "failed to power-on the sensor\n");
		goto error_mutex_destroy;
	}

	/* Check module identity */
	ret = kmb_ov9282_detect(kmb_ov9282);
	if (ret) {
		dev_err(&client->dev, "failed to find sensor: %d", ret);
		goto error_sensor_power_off;
	}

	/* Set default mode to max resolution */
	kmb_ov9282->cur_mode = &supported_modes[0];
	kmb_ov9282->fps = kmb_ov9282->cur_mode->fps.def;
	kmb_ov9282->lpfr = kmb_ov9282->cur_mode->lpfr;

	ret = kmb_ov9282_init_controls(kmb_ov9282);
	if (ret) {
		dev_err(&client->dev, "failed to init controls: %d", ret);
		goto error_sensor_power_off;
	}

	/* Initialize subdev */
	kmb_ov9282->sd.internal_ops = &kmb_ov9282_internal_ops;
	kmb_ov9282->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	kmb_ov9282->sd.entity.ops = &kmb_ov9282_subdev_entity_ops;
	kmb_ov9282->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
	kmb_ov9282->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&kmb_ov9282->sd.entity, 1,
				     &kmb_ov9282->pad);
	if (ret) {
		dev_err(&client->dev, "failed to init entity pads: %d", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor_common(&kmb_ov9282->sd);
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
	media_entity_cleanup(&kmb_ov9282->sd.entity);
error_handler_free:
	v4l2_ctrl_handler_free(kmb_ov9282->sd.ctrl_handler);
error_sensor_power_off:
	kmb_ov9282_power_off(kmb_ov9282);
error_mutex_destroy:
	mutex_destroy(&kmb_ov9282->mutex);
	return ret;
}

/**
 * kmb_ov9282_i2c_remove - I2C client device unbinding
 * @client: pointer to I2C client device
 *
 * Return: 0 if successful
 */
static int kmb_ov9282_i2c_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct kmb_ov9282 *kmb_ov9282 = to_kmb_ov9282(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(&client->dev);
	pm_runtime_suspended(&client->dev);

	mutex_destroy(&kmb_ov9282->mutex);

	return 0;
}

static const struct dev_pm_ops kmb_ov9282_i2c_pm_ops = {
	SET_RUNTIME_PM_OPS(kmb_ov9282_i2c_suspend, kmb_ov9282_i2c_resume, NULL)
};

static const struct i2c_device_id kmb_ov9282_i2c_id_table[] = {
	{KMB_OV9282_DRV_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, kmb_ov9282_i2c_id_table);

static struct i2c_driver kmb_ov9282_i2c_driver = {
	.probe = kmb_ov9282_i2c_probe,
	.remove = kmb_ov9282_i2c_remove,
	.id_table = kmb_ov9282_i2c_id_table,
	.driver = {
		.owner = THIS_MODULE,
		.pm = &kmb_ov9282_i2c_pm_ops,
		.name = KMB_OV9282_DRV_NAME,
	},
};

/* --------------- probe as platform device ----------------- */

/**
 * kmb_ov9282_platform_resume - PM resume callback
 * @dev: platform device
 *
 * Return: 0 if successful
 */
static int kmb_ov9282_platform_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct kmb_ov9282 *kmb_ov9282 = platform_get_drvdata(pdev);

	return kmb_ov9282_power_on(kmb_ov9282);
}

/**
 * kmb_ov9282_platform_suspend - PM suspend callback
 * @dev: platform device
 *
 * Return: 0 if successful
 */
static int kmb_ov9282_platform_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct kmb_ov9282 *kmb_ov9282 = platform_get_drvdata(pdev);

	kmb_ov9282_power_off(kmb_ov9282);

	return 0;
}

/**
 * kmb_ov9282_get_i2c_client - Get I2C client
 * @kmb_ov9282: pointer to kmb_ov9282 device
 *
 * Return: 0 if successful
 */
static int kmb_ov9282_get_i2c_client(struct kmb_ov9282 *kmb_ov9282)
{
	struct i2c_board_info info = {
		.type = "kmb-ov9282-sensor-p" };
	const unsigned short addr_list[] = {0x60, 0x70, I2C_CLIENT_END};
	struct i2c_adapter *i2c_adp;
	struct device_node *phandle;

	phandle = of_parse_phandle(kmb_ov9282->dev->of_node, "i2c-bus", 0);
	if (!phandle)
		return -ENODEV;

	i2c_adp = of_get_i2c_adapter_by_node(phandle);
	of_node_put(phandle);
	if (!i2c_adp)
		return -EPROBE_DEFER;

	kmb_ov9282->client =
		i2c_new_probed_device(i2c_adp, &info, addr_list, NULL);
	i2c_put_adapter(i2c_adp);
	if (!kmb_ov9282->client)
		return -ENODEV;

	return 0;
}

/**
 * kmb_ov9282_pdev_probe - Platform device binding
 * @pdev: pointer to platform device
 *
 * Return: 0 if successful
 */
static int kmb_ov9282_pdev_probe(struct platform_device *pdev)
{
	struct kmb_ov9282 *kmb_ov9282;
	struct gpio_descs *detect_gpios;
	int ret;

	kmb_ov9282 = devm_kzalloc(&pdev->dev, sizeof(*kmb_ov9282),
				  GFP_KERNEL);
	if (!kmb_ov9282)
		return -ENOMEM;

	platform_set_drvdata(pdev, kmb_ov9282);

	mutex_init(&kmb_ov9282->mutex);

	kmb_ov9282->dev = &pdev->dev;

	/* Initialize subdev */
	v4l2_subdev_init(&kmb_ov9282->sd, &kmb_ov9282_subdev_ops);
	kmb_ov9282->sd.owner = pdev->dev.driver->owner;
	kmb_ov9282->sd.dev = &pdev->dev;

	/* request optional detect pins */
	detect_gpios =
		gpiod_get_array_optional(&pdev->dev, "detect", GPIOD_OUT_LOW);
	if (IS_ERR(detect_gpios)) {
		ret = PTR_ERR(detect_gpios);
		dev_err(&pdev->dev, "failed to get detect gpios %d", ret);
		ret = (ret == -EBUSY) ? -EPROBE_DEFER : ret;
		goto error_mutex_destroy;
	}

	/* Get sensor input clock */
	kmb_ov9282->inclk = devm_clk_get(&pdev->dev, "inclk");
	if (IS_ERR(kmb_ov9282->inclk)) {
		ret = PTR_ERR(kmb_ov9282->inclk);
		dev_err(&pdev->dev, "could not get inclk");
		goto error_put_detect_gpios;
	}

	ret = clk_set_rate(kmb_ov9282->inclk, KMB_OV9282_INCLK_RATE);
	if (ret) {
		dev_err(&pdev->dev, "could not set inclk frequency\n");
		goto error_put_detect_gpios;
	}

	ret = kmb_ov9282_power_on(kmb_ov9282);
	if (ret) {
		dev_dbg(&pdev->dev, "failed to power-on the sensor %d", ret);
		/* Defer the probe if resourcess are busy */
		ret = (ret == -EBUSY) ? -EPROBE_DEFER : ret;
		goto error_put_detect_gpios;
	}

	ret = kmb_ov9282_get_i2c_client(kmb_ov9282);
	if (ret) {
		dev_err(&pdev->dev, "failed to get i2c\n");
		goto error_sensor_power_off;
	}

	v4l2_set_subdevdata(&kmb_ov9282->sd, kmb_ov9282->client);
	i2c_set_clientdata(kmb_ov9282->client, &kmb_ov9282->sd);
	v4l2_i2c_subdev_set_name(&kmb_ov9282->sd, kmb_ov9282->client,
				 KMB_OV9282_DRV_NAME, pdev->name);

	/* Check module identity */
	ret = kmb_ov9282_detect(kmb_ov9282);
	if (ret) {
		dev_err(&pdev->dev, "failed to find sensor: %d", ret);
		goto error_unregister_i2c_dev;
	}

	/* Set default mode to max resolution */
	kmb_ov9282->cur_mode = &supported_modes[0];
	kmb_ov9282->fps = kmb_ov9282->cur_mode->fps.def;
	kmb_ov9282->lpfr = kmb_ov9282->cur_mode->lpfr;

	ret = kmb_ov9282_init_controls(kmb_ov9282);
	if (ret) {
		dev_err(&pdev->dev, "failed to init controls: %d", ret);
		goto error_unregister_i2c_dev;
	}

	/* Initialize subdev */
	kmb_ov9282->sd.internal_ops = &kmb_ov9282_internal_ops;
	kmb_ov9282->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	kmb_ov9282->sd.entity.ops = &kmb_ov9282_subdev_entity_ops;
	kmb_ov9282->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
	kmb_ov9282->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&kmb_ov9282->sd.entity, 1,
				     &kmb_ov9282->pad);
	if (ret) {
		dev_err(&pdev->dev, "failed to init entity pads: %d", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor_common(&kmb_ov9282->sd);
	if (ret < 0) {
		dev_err(&pdev->dev,
				"failed to register async subdev: %d", ret);
		goto error_media_entity;
	}

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_idle(&pdev->dev);

	if (detect_gpios)
		gpiod_put_array(detect_gpios);

	dev_info(&pdev->dev, "Probe success!");
	return 0;

error_media_entity:
	media_entity_cleanup(&kmb_ov9282->sd.entity);
error_handler_free:
	v4l2_ctrl_handler_free(kmb_ov9282->sd.ctrl_handler);
error_unregister_i2c_dev:
	if (kmb_ov9282->client)
		i2c_unregister_device(kmb_ov9282->client);
error_sensor_power_off:
	kmb_ov9282_power_off(kmb_ov9282);
error_put_detect_gpios:
	if (detect_gpios)
		gpiod_put_array(detect_gpios);
error_mutex_destroy:
	mutex_destroy(&kmb_ov9282->mutex);

	return ret;
}

/**
 * kmb_ov9282_pdev_remove - Platform device unbinding
 * @pdev: pointer to platform device
 *
 * Return: 0 if successful
 */
static int kmb_ov9282_pdev_remove(struct platform_device *pdev)
{
	struct kmb_ov9282 *kmb_ov9282 =  platform_get_drvdata(pdev);
	struct v4l2_subdev *sd = &kmb_ov9282->sd;

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(&pdev->dev);
	pm_runtime_suspended(&pdev->dev);

	if (kmb_ov9282->client)
		i2c_unregister_device(kmb_ov9282->client);

	mutex_destroy(&kmb_ov9282->mutex);

	return 0;
}

static const struct dev_pm_ops kmb_ov9282_platform_pm_ops = {
	SET_RUNTIME_PM_OPS(kmb_ov9282_platform_suspend,
			   kmb_ov9282_platform_resume, NULL)
};


static const struct of_device_id kmb_ov9282_id_table[] = {
	{.compatible = "intel,kmb-ov9282-sensor-p"},
	{}
};
MODULE_DEVICE_TABLE(of, kmb_ov9282_id_table);

static struct platform_driver kmb_ov9282_platform_driver = {
	.probe	= kmb_ov9282_pdev_probe,
	.remove = kmb_ov9282_pdev_remove,
	.driver = {
		.name = KMB_OV9282_DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &kmb_ov9282_platform_pm_ops,
		.of_match_table = kmb_ov9282_id_table,
	}
};

static int __init kmb_ov9282_init(void)
{
	int ret;

	ret = i2c_add_driver(&kmb_ov9282_i2c_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&kmb_ov9282_platform_driver);
	if (ret)
		i2c_del_driver(&kmb_ov9282_i2c_driver);

	return ret;
}

static void __exit kmb_ov9282_exit(void)
{
	i2c_del_driver(&kmb_ov9282_i2c_driver);
	platform_driver_unregister(&kmb_ov9282_platform_driver);
}

module_init(kmb_ov9282_init);
module_exit(kmb_ov9282_exit);
MODULE_DESCRIPTION("Keem Bay OV9282 Sensor driver");
MODULE_LICENSE("GPL v2");
