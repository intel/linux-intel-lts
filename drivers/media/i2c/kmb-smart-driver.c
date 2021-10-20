// SPDX-License-Identifier: GPL-2.0-only
/*
 * kmb-smart-driver.c - KeemBay Camera Smart Sensor Driver for simulation.
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

#define KMB_SMART_DRV_NAME		"kmb-smart-sensor"

/* Streaming Mode */
#define KMB_SMART_DRV_REG_MODE_SELECT	0x0100
#define KMB_SMART_DRV_MODE_STANDBY	0x00
#define KMB_SMART_DRV_MODE_STREAMING	0x01

/* Lines per frame */
#define KMB_SMART_DRV_REG_LPFR		0x380E
#define KMB_SMART_DRV_LPFR_MAX		0xFFFF

/* Chip ID */
#define KMB_SMART_DRV_REG_ID		0x300A
#define KMB_SMART_DRV_ID		0x9281

/* Exposure control */
#define KMB_SMART_DRV_REG_EXPOSURE	0x3500
#define KMB_SMART_DRV_EXPOSURE_MIN	1
#define KMB_SMART_DRV_EXPOSURE_STEP	1
#define KMB_SMART_DRV_EXPOSURE_DEFAULT	0x0282

/* Analog gain control */
#define KMB_SMART_DRV_REG_AGAIN		0x3509
#define KMB_SMART_DRV_AGAIN_MIN		0x10
#define KMB_SMART_DRV_AGAIN_MAX		0xFF
#define KMB_SMART_DRV_AGAIN_STEP	1
#define KMB_SMART_DRV_AGAIN_DEFAULT	0x10

/* Group hold register */
#define KMB_SMART_DRV_REG_HOLD		0x3308

/* Input clock rate */
#define KMB_SMART_DRV_INCLK_RATE	24000000

/* Link frequency */
#define KMB_SMART_DRV_LINK_FREQ_400MHz	400000000

#define KMB_SMART_DRV_REG_MIN	0x00
#define KMB_SMART_DRV_REG_MAX	0xFFFFF

/**
 * struct kmb_smart_drv_reg - KMB Smart Sensor driver register
 * @address: Register address
 * @val: Register value
 */
struct kmb_smart_drv_reg {
	u16 address;
	u8 val;
};

/**
 * struct kmb_smart_drv_reg_list - KMB Smart Sensor register list
 * @num_of_regs: Number of registers in the list
 * @regs: Pointer to register list
 */
struct kmb_smart_drv_reg_list {
	u32 num_of_regs;
	const struct kmb_smart_drv_reg *regs;
};

/**
 * struct kmb_smart_drv_mode - KMB Smart Sensor mode structure
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
struct kmb_smart_drv_mode {
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
	struct kmb_smart_drv_reg_list reg_list;
};

/**
 * struct kmb_smart_drv - KMB Smart Sensor device structure
 * @dev: pointer to generic device
 * @client: pointer to i2c client
 * @sd: V4L2 sub-device
 * @pad: Media pad. Only one pad supported
 * @reset_gpio: Sensor reset gpio
 * @inclk: Sensor input clock
 * @ctrl_handler: V4L2 control handler
 * @exp_ctrl: Pointer to exposure control
 * @again_ctrl: Pointer to analog gain control
 * @brightness: brightness
 * @gain_abs: absolute gain
 * @contrast: contrast
 * @saturation: saturation
 * @hue: hue
 * @awb: auto white balance
 * @red_bal: red balance
 * @blue_bal: blue balance
 * @gamma: gamma
 * @auto_gain: auto gain
 * @h_flip: horizontal flip
 * @v_flip: vertical flip
 * @pwr_freq: power line frequency
 * @sharpness: sharpness
 * @antibanding: banding stop filter
 * @pclk_ctrl: Pointer to pixel clock control
 * @link_freq_ctrl: Pointer to link frequency control
 * @hblank_ctrl: Pointer to horizontal blanking control
 * @vblank_ctrl: Pointer to vertical blanking control
 * @test_pattern: Pointer to test pattern control
 * @fps: FPS to be applied on next stream on
 * @lpfr: Lines per frame for long exposure frame
 * @cur_mode: Current selected sensor mode
 * @mutex: Mutex for serializing sensor controls
 * @streaming: Flag indicating streaming state
 */
struct kmb_smart_drv {
	struct device *dev;
	struct i2c_client *client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct gpio_desc *reset_gpio;
	struct clk *inclk;
	struct v4l2_ctrl_handler ctrl_handler;
	struct {
		struct v4l2_ctrl *exp_ctrl;
		struct v4l2_ctrl *again_ctrl;
	};
	struct v4l2_ctrl *brightness;
	struct v4l2_ctrl *gain_abs;
	struct v4l2_ctrl *contrast;
	struct v4l2_ctrl *saturation;
	struct v4l2_ctrl *hue;
	struct v4l2_ctrl *awb;
	struct v4l2_ctrl *red_bal;
	struct v4l2_ctrl *blue_bal;
	struct v4l2_ctrl *gamma;
	struct v4l2_ctrl *auto_gain;
	struct v4l2_ctrl *h_flip;
	struct v4l2_ctrl *v_flip;
	struct v4l2_ctrl *pwr_freq;
	struct v4l2_ctrl *sharpness;
	struct v4l2_ctrl *antibanding;
	struct v4l2_ctrl *pclk_ctrl;
	struct v4l2_ctrl *link_freq_ctrl;
	struct v4l2_ctrl *hblank_ctrl;
	struct v4l2_ctrl *vblank_ctrl;
	struct v4l2_ctrl *test_pattern;
	u32 fps;
	u32 lpfr;
	const struct kmb_smart_drv_mode *cur_mode;
	struct mutex mutex;
	bool streaming;
};

static const s64 link_freq[] = {
	KMB_SMART_DRV_LINK_FREQ_400MHz,
};

static const char * const test_patterns[] = {
	"repeating",
	"repeating w/ 0xCECA seed",
	"repeating w/ ts seed",
	"color bars",
};

/* Sensor mode registers */
static const struct kmb_smart_drv_reg mode_1280x720_regs[] = {
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

/* Supported sensor mode configurations */
static const struct kmb_smart_drv_mode supported_modes[] = {
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
 * to_kmb_smart_drv - V4L2 sub-device to kmb smart camera device.
 * @subdev: pointer to V4L2 sub-device
 *
 * Return: Pointer to kmb_smart_drv device
 */
static inline struct kmb_smart_drv *to_kmb_smart_drv(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct kmb_smart_drv, sd);
}

/**
 * kmb_smart_drv_read_reg - Read registers.
 * @smart_drv: pointer to smart camera device
 * @reg: Register address
 * @len: Length of bytes to read. Max supported bytes is 4
 * @val: Pointer to register value to be filled.
 *
 * Return: 0 if successful
 */
static int
kmb_smart_drv_read_reg(struct kmb_smart_drv *smart_drv, u16 reg, u32 len,
		       u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&smart_drv->sd);
	struct i2c_msg msgs[2] = { 0 };
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
 * kmb_smart_drv_write_reg - Write register
 * @smart_drv: pointer to smart camera device
 * @reg: Register address
 * @len: Length of bytes. Max supported bytes is 4
 * @val: Register value
 *
 * Return: 0 if successful
 */
static int
kmb_smart_drv_write_reg(struct kmb_smart_drv *smart_drv, u16 reg, u32 len,
			u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&smart_drv->sd);
	int buf_i, val_i;
	u8 buf[6], *val_p;
	__be32 val_be;
	int ret;

	if (len > 4) {
		dev_err_ratelimited(smart_drv->dev,
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
		dev_err_ratelimited(smart_drv->dev,
				"write reg 0x%4.4x return err %d",
				reg, ret);
		return -EIO;
	}

	return 0;
}

/**
 * kmb_smart_drv_write_regs - Write a list of registers
 * @smart_drv: pointer to smart camera device
 * @regs: List of registers to be written
 * @len: Length of registers array
 *
 * Return: 0 if successful
 */
static int kmb_smart_drv_write_regs(struct kmb_smart_drv *smart_drv,
				 const struct kmb_smart_drv_reg *regs, u32 len)
{
	int ret;
	u32 i;

	for (i = 0; i < len; i++) {
		ret = kmb_smart_drv_write_reg(smart_drv, regs[i].address,
					   1, regs[i].val);
		if (ret)
			return ret;
	}

	return 0;
}

/**
 * kmb_smart_drv_update_fps - Update current sensor mode to match the
 *                            selected FPS
 * @smart_drv: pointer to smart camera device
 * @mode: pointer to kmb_smart_drv_mode sensor mode
 *
 * Return: none
 */
static void kmb_smart_drv_update_fps(struct kmb_smart_drv *smart_drv,
				  const struct kmb_smart_drv_mode *mode)
{
	u32 lpfr = (mode->lpfr * mode->fps.def) / smart_drv->fps;

	if (lpfr > KMB_SMART_DRV_LPFR_MAX)
		lpfr = KMB_SMART_DRV_LPFR_MAX;

	/* no info in documentation */
	if (lpfr < mode->height + 12)
		lpfr = mode->height + 12;

	smart_drv->lpfr = lpfr;

	dev_dbg(smart_drv->dev, "Selected FPS %d lpfr %d",
		smart_drv->fps, lpfr);
}

/**
 * kmb_smart_drv_open - Open smart camera subdevice
 * @sd: pointer to smart camera V4L2 sub-device structure
 * @fh: pointer to smart camera V4L2 sub-device file handle
 *
 * Return: 0 if successful
 */
static int kmb_smart_drv_open(struct v4l2_subdev *sd,
			      struct v4l2_subdev_fh *fh)
{
	struct kmb_smart_drv *smart_drv = to_kmb_smart_drv(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->state, 0);

	mutex_lock(&smart_drv->mutex);

	/* Initialize try_fmt */
	try_fmt->width = smart_drv->cur_mode->width;
	try_fmt->height = smart_drv->cur_mode->height;
	try_fmt->code = smart_drv->cur_mode->code;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&smart_drv->mutex);

	return 0;
}

/**
 * kmb_smart_drv_get_camera_mode_by_fmt - Get the most appropriate camera
 *         mode that meets the code and resolution criteria
 * @smart_drv: pointer to smart camera device
 * @code: media bus format code
 * @width: frame width
 * @height: frame height
 *
 * Return: pointer to the most appropriate camera mode
 */
static const struct kmb_smart_drv_mode *
kmb_smart_drv_get_camera_mode_by_fmt(struct kmb_smart_drv *smart_drv, u32 code,
				  u32 width, u32 height)
{
	const struct kmb_smart_drv_mode *mode = supported_modes;
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
 * kmb_smart_drv_write_exposure_gain - Write exposure and gain
 *                                     in sensor registers
 *
 * @smart_drv: pointer to smart camera device
 * @exposure: exposure value to update register with
 * @analog_gain: gain value to update register with
 *
 * Return: 0 if successful
 */
static int
kmb_smart_drv_write_exposure_gain(struct kmb_smart_drv *smart_drv,
				  u32 exposure, const u32 analog_gain)
{
	u32 lpfr = smart_drv->lpfr;
	int ret = 0;

	if (exposure > (lpfr - 12))
		exposure = lpfr - 12;
	else if (exposure < 1)
		exposure = 1;

	dev_dbg(smart_drv->dev, "Set exposure %d analog gain %x lpfr %d",
		exposure, analog_gain, lpfr);

	ret = kmb_smart_drv_write_reg(smart_drv, KMB_SMART_DRV_REG_HOLD, 1, 1);
	if (ret)
		return ret;

	ret = kmb_smart_drv_write_reg(smart_drv, KMB_SMART_DRV_REG_LPFR,
				   2, lpfr);
	if (ret)
		goto error_release_group_hold;

	ret = kmb_smart_drv_write_reg(smart_drv, KMB_SMART_DRV_REG_EXPOSURE,
				   3, exposure << 4);
	if (ret)
		goto error_release_group_hold;

	ret = kmb_smart_drv_write_reg(smart_drv, KMB_SMART_DRV_REG_AGAIN,
				   1, analog_gain);

error_release_group_hold:
	kmb_smart_drv_write_reg(smart_drv, KMB_SMART_DRV_REG_HOLD, 1, 0);
	return ret;
}

/**
 * kmb_smart_drv_set_ctrl - Set subdevice control. Supported controls:
 *                          V4L2_CID_ANALOGUE_GAIN
 *                          V4L2_CID_EXPOSURE_ABSOLUTE
 *                          Both controls are in one cluster.
 *
 * @ctrl: pointer to v4l2_ctrl structure
 *
 * Return: 0 if successful
 */
static int kmb_smart_drv_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct kmb_smart_drv *smart_drv =
		container_of(ctrl->handler, struct kmb_smart_drv, ctrl_handler);
	u32 exposure;
	u32 analog_gain;
	int ret = 0;

	/* Set exposure and gain only if sensor is in power on state */
	if (!pm_runtime_get_if_in_use(smart_drv->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE_ABSOLUTE:
		/* Handle the cluster for both controls */
		exposure = ctrl->val;
		analog_gain = smart_drv->again_ctrl->val;
		dev_dbg(smart_drv->dev, "%s: set exposure_abs %d",
			__func__, ctrl->val);
		ret = kmb_smart_drv_write_exposure_gain(smart_drv, exposure,
							analog_gain);
		if (ret)
			goto error_pm_runtime_put;
		break;
	case V4L2_CID_BRIGHTNESS:
		dev_dbg(smart_drv->dev, "%s: set brightness %d",
			__func__, ctrl->val);
		break;
	case V4L2_CID_GAIN:
		dev_dbg(smart_drv->dev, "%s: set gain %d",
			__func__, ctrl->val);
		break;
	case V4L2_CID_CONTRAST:
		dev_dbg(smart_drv->dev, "%s: set contrast %d",
			__func__, ctrl->val);
		break;
	case V4L2_CID_SATURATION:
		dev_dbg(smart_drv->dev, "%s: set saturation %d",
			__func__, ctrl->val);
		break;
	case V4L2_CID_HUE:
		dev_dbg(smart_drv->dev, "%s: set hue %d",
			__func__, ctrl->val);
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		dev_dbg(smart_drv->dev, "%s: set awb %d",
			__func__, ctrl->val);
		break;
	case V4L2_CID_RED_BALANCE:
		dev_dbg(smart_drv->dev, "%s: set red balance %d",
			__func__, ctrl->val);
		break;
	case V4L2_CID_BLUE_BALANCE:
		dev_dbg(smart_drv->dev, "%s: set blue balance %d",
			__func__, ctrl->val);
		break;
	case V4L2_CID_GAMMA:
		dev_dbg(smart_drv->dev, "%s: set gamma %d",
			__func__, ctrl->val);
		break;
	case V4L2_CID_AUTOGAIN:
		dev_dbg(smart_drv->dev, "%s: set auto gain %d",
			__func__, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		dev_dbg(smart_drv->dev, "%s: set horizontal flip %d",
			__func__, ctrl->val);
		break;
	case V4L2_CID_VFLIP:
		dev_dbg(smart_drv->dev, "%s: set vertical flip %d",
			__func__, ctrl->val);
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY:
		dev_dbg(smart_drv->dev, "%s: set power frequency %d",
			__func__, ctrl->val);
		break;
	case V4L2_CID_SHARPNESS:
		dev_dbg(smart_drv->dev, "%s: set sharpness %d",
			__func__, ctrl->val);
		break;
	case V4L2_CID_BAND_STOP_FILTER:
		dev_dbg(smart_drv->dev, "%s: set antibanding %d",
			__func__, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		dev_dbg(smart_drv->dev, "%s: set test pattern %d",
			__func__, ctrl->val);
		break;
	default:
		dev_dbg(smart_drv->dev, "Invalid control %d", ctrl->id);
		ret = -EINVAL;
	}

error_pm_runtime_put:
	pm_runtime_put(smart_drv->dev);
	return ret;
}

/* V4l2 subdevice control ops*/
static const struct v4l2_ctrl_ops kmb_smart_drv_ctrl_ops = {
	.s_ctrl = kmb_smart_drv_set_ctrl,
};

/**
 * kmb_smart_drv_enum_mbus_code - Enumerate V4L2 sub-device mbus codes
 * @sd: pointer to smart cam V4L2 sub-device structure
 * @cfg: V4L2 sub-device pad configuration
 * @fsize: V4L2 sub-device size enumeration need to be filled
 *
 * Return: 0 if successful
 */
static int kmb_smart_drv_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
	struct kmb_smart_drv *smart_drv = to_kmb_smart_drv(sd);

	if (code->index > 0)
		return -EINVAL;

	mutex_lock(&smart_drv->mutex);
	code->code = supported_modes[0].code;
	mutex_unlock(&smart_drv->mutex);

	return 0;
}

/**
 * kmb_smart_drv_enum_frame_size - Enumerate V4L2 sub-device frame sizes
 * @sd: pointer to smart cam V4L2 sub-device structure
 * @cfg: V4L2 sub-device pad configuration
 * @fsize: V4L2 sub-device size enumeration need to be filled
 *
 * Return: 0 if successful
 */
static int kmb_smart_drv_enum_frame_size(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *cfg,
				      struct v4l2_subdev_frame_size_enum *fsize)
{
	struct kmb_smart_drv *smart_drv = to_kmb_smart_drv(sd);

	if (fsize->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	mutex_lock(&smart_drv->mutex);
	if (fsize->code != supported_modes[fsize->index].code) {
		mutex_unlock(&smart_drv->mutex);
		return -EINVAL;
	}
	mutex_unlock(&smart_drv->mutex);

	fsize->min_width = supported_modes[fsize->index].width;
	fsize->max_width = fsize->min_width;
	fsize->min_height = supported_modes[fsize->index].height;
	fsize->max_height = fsize->min_height;

	return 0;
}

/**
 * kmb_smart_drv_enum_frame_interval - Enumerate V4L2 sub-device frame intervals
 * @sd: pointer to smart cam V4L2 sub-device structure
 * @cfg: V4L2 sub-device pad configuration
 * @fsize: V4L2 sub-device size enumeration need to be filled
 *
 * callback for VIDIOC_SUBDEV_ENUM_FRAME_INTERVAL()
 *
 * Return: 0 if successful
 */
static int
kmb_smart_drv_enum_frame_interval(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *cfg,
			       struct v4l2_subdev_frame_interval_enum *fie)
{
	struct kmb_smart_drv *smart_drv = to_kmb_smart_drv(sd);
	const struct kmb_smart_drv_mode *mode;
	int fps;
	int ret = 0;

	if (fie->pad)
		return -EINVAL;

	mutex_lock(&smart_drv->mutex);

	if (fie->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		if (fie->code != smart_drv->cur_mode->code ||
		    fie->width != smart_drv->cur_mode->width ||
		    fie->height != smart_drv->cur_mode->height) {
			ret = -EINVAL;
			goto exit_unlock;
		}

		mode = smart_drv->cur_mode;
	} else {
		mode = kmb_smart_drv_get_camera_mode_by_fmt(smart_drv,
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

	dev_dbg(smart_drv->dev, "Enum FPS %d %d/%d", fps,
		fie->interval.numerator, fie->interval.denominator);

exit_unlock:
	mutex_unlock(&smart_drv->mutex);
	return ret;
}

/**
 * kmb_smart_drv_fill_pad_format - Fill subdevice pad format
 *                                 from selected sensor mode
 * @smart_drv: pointer to smart camera device
 * @mode: Pointer to kmb_smart_drv_mode sensor mode
 * @fmt: V4L2 sub-device format need to be filled
 *
 * Return: none
 */
static void kmb_smart_drv_fill_pad_format(struct kmb_smart_drv *smart_drv,
				       const struct kmb_smart_drv_mode *mode,
				       struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.code = mode->code;
	fmt->format.field = V4L2_FIELD_NONE;
}

/**
 * kmb_smart_drv_skip_top_lines - Skip top lines containing metadata
 * @sd: pointer to smart cam V4L2 sub-device structure
 * @lines: number of lines to be skipped
 *
 * Return: 0 if successful
 */
static int kmb_smart_drv_skip_top_lines(struct v4l2_subdev *sd, u32 *lines)
{
	struct kmb_smart_drv *smart_drv = to_kmb_smart_drv(sd);

	*lines = smart_drv->cur_mode->skip_lines;

	return 0;
}


/**
 * kmb_smart_drv_update_controls - Update control ranges based on streaming
 *                                 mode
 * @smart_drv: pointer to smart camera device
 * @mode: pointer to kmb_smart_drv_mode sensor mode
 *
 * Return: none
 */
static void kmb_smart_drv_update_controls(struct kmb_smart_drv *smart_drv,
					const struct kmb_smart_drv_mode *mode)
{
	__v4l2_ctrl_s_ctrl(smart_drv->link_freq_ctrl, 0);
	__v4l2_ctrl_s_ctrl(smart_drv->vblank_ctrl,
			smart_drv->lpfr - mode->height);
	__v4l2_ctrl_s_ctrl(smart_drv->hblank_ctrl,
			   mode->ppln - mode->width);
	__v4l2_ctrl_modify_range(smart_drv->pclk_ctrl,
				mode->pclk, mode->pclk,
				1, mode->pclk);
	__v4l2_ctrl_modify_range(smart_drv->exp_ctrl,
				KMB_SMART_DRV_EXPOSURE_MIN,
				smart_drv->lpfr - 12,
				1, KMB_SMART_DRV_EXPOSURE_DEFAULT);
}

/**
 * kmb_smart_drv_get_pad_format - Get subdevice pad format
 * @sd: pointer to smart cam V4L2 sub-device structure
 * @cfg: V4L2 sub-device pad configuration
 * @fmt: V4L2 sub-device format need to be set
 *
 * Return: 0 if successful
 */
static int kmb_smart_drv_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *cfg,
				 struct v4l2_subdev_format *fmt)
{
	struct kmb_smart_drv *smart_drv = to_kmb_smart_drv(sd);

	mutex_lock(&smart_drv->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *framefmt;

		framefmt = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		fmt->format = *framefmt;
	} else {
		kmb_smart_drv_fill_pad_format(smart_drv,
					   smart_drv->cur_mode,
					   fmt);
	}

	mutex_unlock(&smart_drv->mutex);

	return 0;
}

/**
 * kmb_smart_drv_set_pad_format - Set subdevice pad format
 * @sd: pointer to smart cam V4L2 sub-device structure
 * @cfg: V4L2 sub-device pad configuration
 * @fmt: V4L2 sub-device format need to be set
 *
 * Return: 0 if successful
 */
static int kmb_smart_drv_set_pad_format(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *cfg,
				     struct v4l2_subdev_format *fmt)
{
	struct kmb_smart_drv *smart_drv = to_kmb_smart_drv(sd);
	const struct kmb_smart_drv_mode *mode;
	struct v4l2_mbus_framefmt *framefmt;

	mutex_lock(&smart_drv->mutex);

	/* Currently only one format is supported */
	fmt->format.code = supported_modes[0].code;

	mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);

	smart_drv->fps = smart_drv->fps ? : mode->fps.def;
	if (smart_drv->fps > mode->fps.max)
		smart_drv->fps = mode->fps.max;
	else if (smart_drv->fps < mode->fps.min)
		smart_drv->fps = mode->fps.min;

	kmb_smart_drv_update_fps(smart_drv, mode);

	kmb_smart_drv_fill_pad_format(smart_drv, mode, fmt);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		framefmt = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		*framefmt = fmt->format;
	} else {
		kmb_smart_drv_update_controls(smart_drv, mode);
		smart_drv->cur_mode = mode;
	}

	mutex_unlock(&smart_drv->mutex);

	return 0;
}

/**
 * kmb_smart_drv_start_streaming - Start sensor stream
 * @smart_drv: pointer to smart camera device
 *
 * Return: 0 if successful
 */
static int kmb_smart_drv_start_streaming(struct kmb_smart_drv *smart_drv)
{
	const struct kmb_smart_drv_reg_list *reg_list;
	int ret;

	/* Write sensor mode registers */
	reg_list = &smart_drv->cur_mode->reg_list;
	ret = kmb_smart_drv_write_regs(smart_drv, reg_list->regs,
				    reg_list->num_of_regs);
	if (ret) {
		dev_err(smart_drv->dev, "fail to write initial registers");
		return ret;
	}

	/* Setup handler will write actual exposure and gain */
	ret =  __v4l2_ctrl_handler_setup(smart_drv->sd.ctrl_handler);
	if (ret) {
		dev_err(smart_drv->dev, "fail to setup handler");
		return ret;
	}

	ret = kmb_smart_drv_write_reg(smart_drv, KMB_SMART_DRV_REG_MODE_SELECT,
				      1, KMB_SMART_DRV_MODE_STREAMING);
	if (ret) {
		dev_err(smart_drv->dev, "fail to start streaming");
		return ret;
	}

	return 0;
}

/**
 * kmb_smart_drv_stop_streaming - Stop sensor stream
 * @smart_drv: pointer to smart camera device
 *
 * Return: 0 if successful
 */
static int kmb_smart_drv_stop_streaming(struct kmb_smart_drv *smart_drv)
{
	return kmb_smart_drv_write_reg(smart_drv,
			KMB_SMART_DRV_REG_MODE_SELECT, 1,
			KMB_SMART_DRV_MODE_STANDBY);
}

/**
 * kmb_smart_drv_set_stream - Enable sensor streaming
 * @sd: pointer to smart camera subdevice
 * @enable: Set to enable sensor streaming
 *
 * Return: 0 if successful
 */
static int kmb_smart_drv_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct kmb_smart_drv *smart_drv = to_kmb_smart_drv(sd);
	int ret;

	mutex_lock(&smart_drv->mutex);

	if (smart_drv->streaming == enable) {
		mutex_unlock(&smart_drv->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_get_sync(smart_drv->dev);
		if (ret)
			goto error_unlock;

		ret = kmb_smart_drv_start_streaming(smart_drv);
		if (ret)
			goto error_power_off;
	} else {
		kmb_smart_drv_stop_streaming(smart_drv);
		pm_runtime_put(smart_drv->dev);
	}

	smart_drv->streaming = enable;

	mutex_unlock(&smart_drv->mutex);

	return 0;

error_power_off:
	pm_runtime_put(smart_drv->dev);
error_unlock:
	mutex_unlock(&smart_drv->mutex);
	return ret;
}

/**
 * kmb_smart_drv_get_frame_interval - Get subdevice frame interval
 * @sd: pointer to smart cam V4L2 sub-device structure
 * @interval: V4L2 sub-device current farme interval
 *
 * Return: 0 if successful
 */
static int kmb_smart_drv_get_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *interval)
{
	struct kmb_smart_drv *smart_drv = to_kmb_smart_drv(sd);

	if (interval->pad)
		return -EINVAL;

	mutex_lock(&smart_drv->mutex);
	interval->interval.numerator = 1;
	interval->interval.denominator = smart_drv->fps;
	mutex_unlock(&smart_drv->mutex);

	dev_dbg(smart_drv->dev, "Get frame interval %d/%d",
		interval->interval.numerator,
		interval->interval.denominator);

	return 0;
}

/**
 * kmb_smart_drv_set_frame_interval - Set subdevice frame interval
 * @sd: pointer to smart cam V4L2 sub-device structure
 * @interval: V4L2 sub-device farme interval to be set
 *
 * Return: 0 if successful
 */
static int kmb_smart_drv_set_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *interval)
{
	struct kmb_smart_drv *smart_drv = to_kmb_smart_drv(sd);
	u32 fps;

	if (interval->pad)
		return -EINVAL;

	mutex_lock(&smart_drv->mutex);
	fps = (u32)(interval->interval.denominator /
		interval->interval.numerator);
	if (fps < smart_drv->cur_mode->fps.min) {
		interval->interval.numerator = 1;
		interval->interval.denominator = smart_drv->cur_mode->fps.min;
	} else if (fps > smart_drv->cur_mode->fps.max) {
		interval->interval.numerator = 1;
		interval->interval.denominator = smart_drv->cur_mode->fps.max;
	}

	smart_drv->fps = (u32) (interval->interval.denominator /
				     interval->interval.numerator);

	kmb_smart_drv_update_fps(smart_drv, smart_drv->cur_mode);

	kmb_smart_drv_update_controls(smart_drv, smart_drv->cur_mode);

	mutex_unlock(&smart_drv->mutex);

	dev_dbg(smart_drv->dev, "Set frame interval %d/%d",
		interval->interval.numerator,
		interval->interval.denominator);

	return 0;
}

/**
 * kmb_smart_drv_power_on - Sensor power on sequence
 * @smart_drv: smart camera device
 *
 * Return: 0 if successful
 */
static int kmb_smart_drv_power_on(struct kmb_smart_drv *smart_drv)
{
	int ret;

	/* request optional reset pin */
	smart_drv->reset_gpio =
		gpiod_get_optional(smart_drv->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(smart_drv->reset_gpio)) {
		ret = PTR_ERR(smart_drv->reset_gpio);
		dev_err(smart_drv->dev, "failed to get reset gpio %d", ret);
		return ret;
	}

	if (smart_drv->reset_gpio)
		gpiod_set_value_cansleep(smart_drv->reset_gpio, 1);

	ret = clk_prepare_enable(smart_drv->inclk);
	if (ret) {
		dev_err(smart_drv->dev, "fail to enable inclk\n");
		goto error_reset;
	}

	usleep_range(18000, 20000);

	return 0;

error_reset:
	if (smart_drv->reset_gpio) {
		gpiod_set_value_cansleep(smart_drv->reset_gpio, 0);
		gpiod_put(smart_drv->reset_gpio);
		smart_drv->reset_gpio = NULL;
	}

	return ret;
}

/**
 * kmb_smart_drv_power_off - Sensor power off sequence
 * @smart_drv: pointer to smart camera device
 */
static void kmb_smart_drv_power_off(struct kmb_smart_drv *smart_drv)
{
	clk_disable_unprepare(smart_drv->inclk);

	if (smart_drv->reset_gpio) {
		gpiod_set_value_cansleep(smart_drv->reset_gpio, 0);
		gpiod_put(smart_drv->reset_gpio);
		smart_drv->reset_gpio = NULL;

	}
}

/**
 * kmb_smart_drv_detect - Detect Smart camera simulation sensor
 * @smart_drv: pointer to smart camera device
 *
 * Return: 0 if successful, -EIO if sensor id does not match
 */
static int kmb_smart_drv_detect(struct kmb_smart_drv *smart_drv)
{
	int ret;
	u32 val = 0;

	ret = kmb_smart_drv_read_reg(smart_drv, KMB_SMART_DRV_REG_ID, 2, &val);
	if (ret)
		return ret;

	if (val != KMB_SMART_DRV_ID) {
		dev_err(smart_drv->dev, "chip id mismatch: %x!=%x",
			KMB_SMART_DRV_ID, val);
		return -EIO;
	}
	return 0;
}

/**
 * kmb_smart_drv_s_power - Set power core operation. Actual power is enabled on
 *                         set stream, this callback is used only for reset
 *                         formats and fps to default values.
 * @sd: pointer to smart camera sub-device.
 * @on: power on/off flag.
 *
 * Return: 0 if successful
 */
static int kmb_smart_drv_s_power(struct v4l2_subdev *sd, int on)
{
	struct kmb_smart_drv *smart_drv = to_kmb_smart_drv(sd);

	if (!on)
		return 0;

	mutex_lock(&smart_drv->mutex);

	/* Set default sensor mode */
	smart_drv->fps = 0;
	smart_drv->cur_mode = &supported_modes[0];
	smart_drv->lpfr = smart_drv->cur_mode->lpfr;

	mutex_unlock(&smart_drv->mutex);

	return 0;
}

/* V4l2 subdevice ops */
static struct v4l2_subdev_core_ops kmb_smart_drv_subdev_core_ops = {
	.s_power = kmb_smart_drv_s_power,
};

static const struct v4l2_subdev_video_ops kmb_smart_drv_video_ops = {
	.s_stream = kmb_smart_drv_set_stream,
	.g_frame_interval = kmb_smart_drv_get_frame_interval,
	.s_frame_interval = kmb_smart_drv_set_frame_interval,
};

static const struct v4l2_subdev_pad_ops kmb_smart_drv_pad_ops = {
	.enum_mbus_code = kmb_smart_drv_enum_mbus_code,
	.get_fmt = kmb_smart_drv_get_pad_format,
	.set_fmt = kmb_smart_drv_set_pad_format,
	.enum_frame_size = kmb_smart_drv_enum_frame_size,
	.enum_frame_interval = kmb_smart_drv_enum_frame_interval,
};

static const struct v4l2_subdev_sensor_ops kmb_smart_drv_sensor_ops = {
	.g_skip_top_lines = kmb_smart_drv_skip_top_lines,
};

static const struct v4l2_subdev_ops kmb_smart_drv_subdev_ops = {
	.core = &kmb_smart_drv_subdev_core_ops,
	.video = &kmb_smart_drv_video_ops,
	.pad = &kmb_smart_drv_pad_ops,
	.sensor = &kmb_smart_drv_sensor_ops,
};

static const struct media_entity_operations kmb_smart_drv_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops kmb_smart_drv_internal_ops = {
	.open = kmb_smart_drv_open,
};

/**
 * kmb_smart_drv_init_controls - Initialize sensor subdevice controls
 * @smart_drv: pointer to smart camera device
 *
 * Return: 0 if successful
 */
static int kmb_smart_drv_init_controls(struct kmb_smart_drv *smart_drv)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &smart_drv->ctrl_handler;
	const struct kmb_smart_drv_mode *mode = smart_drv->cur_mode;
	u32 hblank;
	u32 vblank;
	int ret;

	ctrl_hdlr = &smart_drv->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 8);
	if (ret)
		return ret;

	/* Serialize controls with sensor device */
	ctrl_hdlr->lock = &smart_drv->mutex;

	/* Initialize exposure and gain */
	smart_drv->exp_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
			&kmb_smart_drv_ctrl_ops,
			V4L2_CID_EXPOSURE_ABSOLUTE,
			KMB_SMART_DRV_EXPOSURE_MIN,
			mode->lpfr,
			KMB_SMART_DRV_EXPOSURE_STEP,
			KMB_SMART_DRV_EXPOSURE_DEFAULT);

	smart_drv->again_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
			&kmb_smart_drv_ctrl_ops,
			V4L2_CID_ANALOGUE_GAIN,
			KMB_SMART_DRV_AGAIN_MIN,
			KMB_SMART_DRV_AGAIN_MAX,
			KMB_SMART_DRV_AGAIN_STEP,
			KMB_SMART_DRV_AGAIN_DEFAULT);
	v4l2_ctrl_cluster(2, &smart_drv->exp_ctrl);

	smart_drv->brightness = v4l2_ctrl_new_std(ctrl_hdlr,
			&kmb_smart_drv_ctrl_ops, V4L2_CID_BRIGHTNESS,
			-3, 3, 1, 0);
	smart_drv->gain_abs = v4l2_ctrl_new_std(ctrl_hdlr,
			&kmb_smart_drv_ctrl_ops, V4L2_CID_GAIN,
			-3, 3, 1, 0);
	smart_drv->contrast = v4l2_ctrl_new_std(ctrl_hdlr,
			&kmb_smart_drv_ctrl_ops, V4L2_CID_CONTRAST,
			0, 127, 1, 0x20);
	smart_drv->saturation = v4l2_ctrl_new_std(ctrl_hdlr,
			&kmb_smart_drv_ctrl_ops, V4L2_CID_SATURATION,
			0, 256, 1, 0x80);
	smart_drv->hue = v4l2_ctrl_new_std(ctrl_hdlr,
			&kmb_smart_drv_ctrl_ops, V4L2_CID_HUE,
			0, 0x1f, 1, 0x10);
	smart_drv->awb = v4l2_ctrl_new_std(ctrl_hdlr,
			&kmb_smart_drv_ctrl_ops, V4L2_CID_AUTO_WHITE_BALANCE,
			0, 1, 1, 1);
	smart_drv->red_bal = v4l2_ctrl_new_std(ctrl_hdlr,
			&kmb_smart_drv_ctrl_ops, V4L2_CID_RED_BALANCE,
			0, 0xff, 1, 0x80);
	smart_drv->blue_bal = v4l2_ctrl_new_std(ctrl_hdlr,
			&kmb_smart_drv_ctrl_ops, V4L2_CID_BLUE_BALANCE,
			0, 0xff, 1, 0x80);
	smart_drv->gamma = v4l2_ctrl_new_std(ctrl_hdlr,
			&kmb_smart_drv_ctrl_ops, V4L2_CID_GAMMA,
			0, 0xff, 1, 0x12);
	smart_drv->auto_gain = v4l2_ctrl_new_std(ctrl_hdlr,
			&kmb_smart_drv_ctrl_ops, V4L2_CID_AUTOGAIN,
			0, 1, 1, 1);
	smart_drv->h_flip = v4l2_ctrl_new_std(ctrl_hdlr,
			&kmb_smart_drv_ctrl_ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
	smart_drv->v_flip = v4l2_ctrl_new_std(ctrl_hdlr,
			&kmb_smart_drv_ctrl_ops, V4L2_CID_VFLIP, 0, 1, 1, 0);
	smart_drv->pwr_freq = v4l2_ctrl_new_std_menu(ctrl_hdlr,
			&kmb_smart_drv_ctrl_ops,
			V4L2_CID_POWER_LINE_FREQUENCY,
			V4L2_CID_POWER_LINE_FREQUENCY_60HZ, ~0x7,
			V4L2_CID_POWER_LINE_FREQUENCY_50HZ);
	smart_drv->sharpness = v4l2_ctrl_new_std(ctrl_hdlr,
			&kmb_smart_drv_ctrl_ops,
			V4L2_CID_SHARPNESS, 0, 32, 1, 6);
	smart_drv->antibanding = v4l2_ctrl_new_std(ctrl_hdlr,
			&kmb_smart_drv_ctrl_ops, V4L2_CID_BAND_STOP_FILTER,
			0, 256, 1, 0);
	smart_drv->test_pattern =
		v4l2_ctrl_new_std_menu_items(ctrl_hdlr,
					     &kmb_smart_drv_ctrl_ops,
					     V4L2_CID_TEST_PATTERN,
					     ARRAY_SIZE(test_patterns) - 1, 0,
					     3, test_patterns);

	/* Read only controls */
	smart_drv->pclk_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						 &kmb_smart_drv_ctrl_ops,
						 V4L2_CID_PIXEL_RATE,
						 mode->pclk, mode->pclk,
						 1, mode->pclk);
	if (smart_drv->pclk_ctrl)
		smart_drv->pclk_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	smart_drv->link_freq_ctrl = v4l2_ctrl_new_int_menu(ctrl_hdlr,
						&kmb_smart_drv_ctrl_ops,
						V4L2_CID_LINK_FREQ,
						ARRAY_SIZE(link_freq) - 1,
						0, link_freq);
	if (smart_drv->link_freq_ctrl)
		smart_drv->link_freq_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank = mode->lpfr - mode->height;
	smart_drv->vblank_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						   &kmb_smart_drv_ctrl_ops,
						   V4L2_CID_VBLANK,
						   KMB_SMART_DRV_REG_MIN,
						   KMB_SMART_DRV_REG_MAX,
						   1, vblank);
	if (smart_drv->vblank_ctrl)
		smart_drv->vblank_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	hblank = mode->ppln - mode->width;
	smart_drv->hblank_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						   &kmb_smart_drv_ctrl_ops,
						   V4L2_CID_HBLANK,
						   KMB_SMART_DRV_REG_MIN,
						   KMB_SMART_DRV_REG_MAX,
						   1, hblank);
	if (smart_drv->hblank_ctrl)
		smart_drv->hblank_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(smart_drv->dev, "control init failed: %d", ret);
		goto error;
	}

	smart_drv->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	return ret;
}

/* --------------- probe as i2c device -------------------- */

/**
 * kmb_smart_drv_i2c_resume - PM resume callback
 * @dev: i2c device
 *
 * Return: 0 if successful
 */
static int kmb_smart_drv_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct kmb_smart_drv *smart_drv = to_kmb_smart_drv(sd);

	return kmb_smart_drv_power_on(smart_drv);
}

/**
 * kmb_smart_drv_i2c_suspend - PM suspend callback
 * @dev: i2c device
 *
 * Return: 0 if successful
 */
static int kmb_smart_drv_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct kmb_smart_drv *smart_drv = to_kmb_smart_drv(sd);

	kmb_smart_drv_power_off(smart_drv);

	return 0;
}

/**
 * kmb_smart_drv_i2c_probe - I2C client device binding
 * @client: pointer to i2c client device
 * @id: pointer to i2c device id
 *
 * Return: 0 if successful
 */
static int kmb_smart_drv_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct kmb_smart_drv *smart_drv;
	int ret;

	smart_drv = devm_kzalloc(&client->dev, sizeof(*smart_drv), GFP_KERNEL);
	if (!smart_drv)
		return -ENOMEM;

	mutex_init(&smart_drv->mutex);

	smart_drv->dev = &client->dev;

	/* Initialize subdev */
	v4l2_i2c_subdev_init(&smart_drv->sd, client, &kmb_smart_drv_subdev_ops);

	smart_drv->reset_gpio = NULL;

	/* Get sensor input clock */
	smart_drv->inclk = devm_clk_get(&client->dev, "inclk");
	if (IS_ERR(smart_drv->inclk)) {
		ret = PTR_ERR(smart_drv->inclk);
		dev_err(&client->dev, "could not get inclk");
		goto error_mutex_destroy;
	}

	ret = clk_set_rate(smart_drv->inclk, KMB_SMART_DRV_INCLK_RATE);
	if (ret) {
		dev_err(&client->dev, "could not set inclk frequency");
		goto error_mutex_destroy;
	}

	ret = kmb_smart_drv_power_on(smart_drv);
	if (ret) {
		dev_err(&client->dev, "failed to power-on the sensor");
		goto error_mutex_destroy;
	}

	/* Check module identity */
	ret = kmb_smart_drv_detect(smart_drv);
	if (ret) {
		dev_err(&client->dev, "failed to find sensor: %d", ret);
		goto error_sensor_power_off;
	}

	/* Set default mode to max resolution */
	smart_drv->cur_mode = &supported_modes[0];
	smart_drv->fps = smart_drv->cur_mode->fps.def;
	smart_drv->lpfr = smart_drv->cur_mode->lpfr;

	ret = kmb_smart_drv_init_controls(smart_drv);
	if (ret) {
		dev_err(&client->dev, "failed to init controls: %d", ret);
		goto error_sensor_power_off;
	}

	/* Initialize subdev */
	smart_drv->sd.internal_ops = &kmb_smart_drv_internal_ops;
	smart_drv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	smart_drv->sd.entity.ops = &kmb_smart_drv_subdev_entity_ops;
	smart_drv->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
	smart_drv->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&smart_drv->sd.entity, 1,
				     &smart_drv->pad);
	if (ret) {
		dev_err(&client->dev, "failed to init entity pads: %d", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&smart_drv->sd);
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
	media_entity_cleanup(&smart_drv->sd.entity);
error_handler_free:
	v4l2_ctrl_handler_free(smart_drv->sd.ctrl_handler);
error_sensor_power_off:
	kmb_smart_drv_power_off(smart_drv);
error_mutex_destroy:
	mutex_destroy(&smart_drv->mutex);
	return ret;
}

/**
 * kmb_smart_drv_i2c_remove - I2C client device unbinding
 * @client: pointer to I2C client device
 *
 * Return: 0 if successful
 */
static int kmb_smart_drv_i2c_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct kmb_smart_drv *smart_drv = to_kmb_smart_drv(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(&client->dev);
	pm_runtime_suspended(&client->dev);

	mutex_destroy(&smart_drv->mutex);

	return 0;
}

static const struct dev_pm_ops kmb_smart_drv_i2c_pm_ops = {
	SET_RUNTIME_PM_OPS(kmb_smart_drv_i2c_suspend,
			   kmb_smart_drv_i2c_resume, NULL)
};

static const struct i2c_device_id kmb_smart_drv_i2c_id_table[] = {
	{KMB_SMART_DRV_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, kmb_smart_drv_i2c_id_table);

static struct i2c_driver kmb_smart_drv_i2c_driver = {
	.probe = kmb_smart_drv_i2c_probe,
	.remove = kmb_smart_drv_i2c_remove,
	.id_table = kmb_smart_drv_i2c_id_table,
	.driver = {
		.owner = THIS_MODULE,
		.pm = &kmb_smart_drv_i2c_pm_ops,
		.name = KMB_SMART_DRV_NAME,
	},
};

/* --------------- probe as platform device ----------------- */

/**
 * kmb_smart_drv_platform_resume - PM resume callback
 * @dev: platform device
 *
 * Return: 0 if successful
 */
static int kmb_smart_drv_platform_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct kmb_smart_drv *smart_drv = platform_get_drvdata(pdev);

	return kmb_smart_drv_power_on(smart_drv);
}

/**
 * kmb_smart_drv_platform_suspend - PM suspend callback
 * @dev: platform device
 *
 * Return: 0 if successful
 */
static int kmb_smart_drv_platform_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct kmb_smart_drv *smart_drv = platform_get_drvdata(pdev);

	kmb_smart_drv_power_off(smart_drv);

	return 0;
}

/**
 * kmb_smart_drv_get_i2c_client - Get I2C client
 * @smart_drv: pointer to smart camera device
 *
 * Return: 0 if successful
 */
static int kmb_smart_drv_get_i2c_client(struct kmb_smart_drv *smart_drv)
{
	struct i2c_board_info info = {
		.type = "kmb-smart-sensor" };
	const unsigned short addr_list[] = {0x60, 0x70, I2C_CLIENT_END};
	struct i2c_adapter *i2c_adp;
	struct device_node *phandle;

	phandle = of_parse_phandle(smart_drv->dev->of_node, "i2c-bus", 0);
	if (!phandle)
		return -ENODEV;

	i2c_adp = of_get_i2c_adapter_by_node(phandle);
	of_node_put(phandle);
	if (!i2c_adp)
		return -EPROBE_DEFER;

	smart_drv->client = i2c_new_scanned_device(i2c_adp, &info, addr_list,
						   NULL);
	i2c_put_adapter(i2c_adp);

	return PTR_ERR_OR_ZERO(smart_drv->client);
}

/**
 * kmb_smart_drv_pdev_probe - Platform device binding
 * @pdev: pointer to platform device
 *
 * Return: 0 if successful
 */
static int kmb_smart_drv_pdev_probe(struct platform_device *pdev)
{
	struct kmb_smart_drv *smart_drv;
	struct gpio_descs *detect_gpios;
	int ret;

	smart_drv = devm_kzalloc(&pdev->dev, sizeof(*smart_drv),
				  GFP_KERNEL);
	if (!smart_drv)
		return -ENOMEM;

	platform_set_drvdata(pdev, smart_drv);

	mutex_init(&smart_drv->mutex);

	smart_drv->dev = &pdev->dev;

	/* Initialize subdev */
	v4l2_subdev_init(&smart_drv->sd, &kmb_smart_drv_subdev_ops);
	smart_drv->sd.owner = pdev->dev.driver->owner;
	smart_drv->sd.dev = &pdev->dev;

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
	smart_drv->inclk = devm_clk_get(&pdev->dev, "inclk");
	if (IS_ERR(smart_drv->inclk)) {
		ret = PTR_ERR(smart_drv->inclk);
		dev_err(&pdev->dev, "could not get inclk");
		goto error_put_detect_gpios;
	}

	ret = clk_set_rate(smart_drv->inclk, KMB_SMART_DRV_INCLK_RATE);
	if (ret) {
		dev_err(&pdev->dev, "could not set inclk frequency\n");
		goto error_put_detect_gpios;
	}

	ret = kmb_smart_drv_power_on(smart_drv);
	if (ret) {
		dev_dbg(&pdev->dev, "failed to power-on the sensor %d", ret);
		/* Defer the probe if resourcess are busy */
		ret = (ret == -EBUSY) ? -EPROBE_DEFER : ret;
		goto error_put_detect_gpios;
	}

	ret = kmb_smart_drv_get_i2c_client(smart_drv);
	if (ret) {
		dev_err(&pdev->dev, "failed to get i2c\n");
		goto error_sensor_power_off;
	}

	v4l2_set_subdevdata(&smart_drv->sd, smart_drv->client);
	i2c_set_clientdata(smart_drv->client, &smart_drv->sd);
	v4l2_i2c_subdev_set_name(&smart_drv->sd, smart_drv->client,
				 KMB_SMART_DRV_NAME, pdev->name);

	/* Check module identity */
	ret = kmb_smart_drv_detect(smart_drv);
	if (ret) {
		dev_err(&pdev->dev, "failed to find sensor: %d", ret);
		goto error_unregister_i2c_dev;
	}

	/* Set default mode to max resolution */
	smart_drv->cur_mode = &supported_modes[0];
	smart_drv->fps = smart_drv->cur_mode->fps.def;
	smart_drv->lpfr = smart_drv->cur_mode->lpfr;

	ret = kmb_smart_drv_init_controls(smart_drv);
	if (ret) {
		dev_err(&pdev->dev, "failed to init controls: %d", ret);
		goto error_unregister_i2c_dev;
	}

	/* Initialize subdev */
	smart_drv->sd.internal_ops = &kmb_smart_drv_internal_ops;
	smart_drv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	smart_drv->sd.entity.ops = &kmb_smart_drv_subdev_entity_ops;
	smart_drv->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
	smart_drv->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&smart_drv->sd.entity, 1,
				     &smart_drv->pad);
	if (ret) {
		dev_err(&pdev->dev, "failed to init entity pads: %d", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&smart_drv->sd);
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
	media_entity_cleanup(&smart_drv->sd.entity);
error_handler_free:
	v4l2_ctrl_handler_free(smart_drv->sd.ctrl_handler);
error_unregister_i2c_dev:
	if (smart_drv->client)
		i2c_unregister_device(smart_drv->client);
error_sensor_power_off:
	kmb_smart_drv_power_off(smart_drv);
error_put_detect_gpios:
	if (detect_gpios)
		gpiod_put_array(detect_gpios);
error_mutex_destroy:
	mutex_destroy(&smart_drv->mutex);

	return ret;
}

/**
 * kmb_smart_drv_pdev_remove - Platform device unbinding
 * @pdev: pointer to platform device
 *
 * Return: 0 if successful
 */
static int kmb_smart_drv_pdev_remove(struct platform_device *pdev)
{
	struct kmb_smart_drv *smart_drv =  platform_get_drvdata(pdev);
	struct v4l2_subdev *sd = &smart_drv->sd;

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(&pdev->dev);
	pm_runtime_suspended(&pdev->dev);

	if (smart_drv->client)
		i2c_unregister_device(smart_drv->client);

	mutex_destroy(&smart_drv->mutex);

	return 0;
}

static const struct dev_pm_ops kmb_smart_drv_platform_pm_ops = {
	SET_RUNTIME_PM_OPS(kmb_smart_drv_platform_suspend,
			   kmb_smart_drv_platform_resume, NULL)
};


static const struct of_device_id kmb_smart_drv_id_table[] = {
	{.compatible = "intel,kmb-smart-ov9282-sensor"},
	{}
};
MODULE_DEVICE_TABLE(of, kmb_smart_drv_id_table);

static struct platform_driver kmb_smart_drv_platform_driver = {
	.probe	= kmb_smart_drv_pdev_probe,
	.remove = kmb_smart_drv_pdev_remove,
	.driver = {
		.name = KMB_SMART_DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &kmb_smart_drv_platform_pm_ops,
		.of_match_table = kmb_smart_drv_id_table,
	}
};

static int __init kmb_smart_drv_init(void)
{
	int ret;

	ret = i2c_add_driver(&kmb_smart_drv_i2c_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&kmb_smart_drv_platform_driver);
	if (ret)
		i2c_del_driver(&kmb_smart_drv_i2c_driver);

	return ret;
}

static void __exit kmb_smart_drv_exit(void)
{
	i2c_del_driver(&kmb_smart_drv_i2c_driver);
	platform_driver_unregister(&kmb_smart_drv_platform_driver);
}

module_init(kmb_smart_drv_init);
module_exit(kmb_smart_drv_exit);
MODULE_DESCRIPTION("KeemBay Smart Sensor driver");
MODULE_LICENSE("GPL v2");
