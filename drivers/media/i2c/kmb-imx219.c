// SPDX-License-Identifier: GPL-2.0-only
/*
 * kmb-imx219.c - KeemBay Camera imx219 Sensor Driver.
 *
 * Copyright (C) 2019 Intel Corporation
 */

#include <asm/unaligned.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/kmb-isp-ctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define KMB_IMX219_DRV_NAME	"kmb-imx219-sensor"

/* Streaming Mode */
#define KMB_IMX219_REG_MODE_SELECT 0x0100
#define KMB_IMX219_MODE_STANDBY    0x00
#define KMB_IMX219_MODE_STREAMING  0x01

/* Chip ID */
#define KMB_IMX219_REG_ID          0x0000
#define KMB_IMX219_ID              0x0219

/* Serial number */
#define KMB_IMX219_SERIAL_ADDR		0x0162  // Dummy serial addr for now
#define KMB_IMX219_SERIAL_STEP		1
#define KMB_IMX219_SERIAL_READ_SIZE	8

/* Lines per frame */
#define KMB_IMX219_LPFR_MAX        0xFFFE

#define KMB_IMX219_REG_AGAIN       0x0157
#define KMB_IMX219_REG_SIZE_AGAIN  1

#define KMB_IMX219_AGAIN_MIN	   0
#define KMB_IMX219_AGAIN_MAX	   232
#define KMB_IMX219_AGAIN_STEP	   1
#define KMB_IMX219_AGAIN_DEFAULT   0

#define KMB_IMX219_REG_DGAIN       0x0158
#define KMB_IMX219_REG_SIZE_DGAIN  2
#define KMB_IMX219_DGAIN_MAX       0x0FFF
#define KMB_IMX219_DGAIN_MIN       0x0100
#define KMB_IMX219_DGAIN_DEFAULT   0x0100
#define KMB_IMX219_DGAIN_STEP      1

#define KMB_IMX219_FPS_MAX       30
#define KMB_IMX219_FPS_MIN       15
#define KMB_IMX219_FPS_DEFAULT   30
#define KMB_IMX219_FPS_STEP      1

#define KMB_IMX219_REG_A_CIT		0x015A
#define KMB_IMX219_REG_SIZE_A_CIT	2
#define KMB_IMX219_EXPOSURE_MIN		1
#define KMB_IMX219_EXPOSURE_STEP	1
#define KMB_IMX219_EXPOSURE_DEFAULT	0x640

#define IMX219_REG_FRM_LENGTH_LINES_A_MSB  (0x0160) /// Frame lenght lines 15-8
#define IMX219_REG_FRM_LENGTH_LINES_A_LSB  (0x0161) /// Frame lenght lines 7-0

#define IMX219_REG_FRM_LENGTH_LINES_B_MSB  (0x0260) /// Frame lenght lines 15-8
#define IMX219_REG_FRM_LENGTH_LINES_B_LSB  (0x0261) /// Frame lenght lines 7-0

#define IMX219_REG_FRM_LENGTH_LINES_SIZE 2

#define V4L2_CLUSTER_SIZE 3
#define V4L2_CTRL_HNDL_SIZE (V4L2_CLUSTER_SIZE + 4)

/* Input clock rate */
#define KMB_IMX219_INCLK_RATE	24000000

#define KMB_IMX219_REG_MIN	0x00
#define KMB_IMX219_REG_MAX	0xFFFFF

#define KMB_MINIMUM_BLANKING    32
/**
 * struct kmb_imx219_reg - KMB imx219 Sensor register
 * @address: Register address
 * @val: Register value
 */
struct kmb_imx219_reg {
	u16 address;
	u8 val;
};

/**
 * struct kmb_imx219_reg_list - KMB imx219 Sensor register list
 * @num_of_regs: Number of registers in the list
 * @regs: Pointer to register list
 */
struct kmb_imx219_reg_list {
	u32 num_of_regs;
	const struct kmb_imx219_reg *regs;
};


/**
 * struct kmb_imx219_mode - KMB imx219 Sensor mode structure
 * @width: Frame width
 * @height: Frame height
 * @code: Format code
 * @ppln: Pixels per line
 * @lpfr: Lines per frame
 * @skip_lines: Top lines to be skipped
 * @pclk: Sensor pixel clock
 * @num_lanes: Data lanes number
 * @def: Default frames per second
 * @min: Min frames per second
 * @max: Max frames per second
 * @step: Frame rate step
 * @reg_list: Register list for sensor mode
 */
struct kmb_imx219_mode {
	u32 width;
	u32 height;
	u32 code;
	u32 ppln;
	u32 lpfr;
	u32 skip_lines;
	u64 pclk;
	u32 num_lanes;
	struct {
		u32 def;
		u32 min;
		u32 max;
		u32 step;
	} fps;
	struct kmb_imx219_reg_list reg_list;
};

/**
 * struct kmb_imx219 - KMB imx219 Sensor device structure
 * @dev: pointer to generic device
 * @client: pointer to i2c client
 * @sd: V4L2 sub-device
 * @pad: Media pad. Only one pad supported
 * @reset_gpio: Sensor reset gpio
 * @power_gpio: Sensor power gpio
 * @inclk: Sensor input clock
 * @ctrl_handler: V4L2 control handler
 * @pclk_ctrl: Pointer to pixel clock control
 * @hblank_ctrl: Pointer to horizontal blanking control
 * @vblank_ctrl: Pointer to vertical blanking control
 * @serial_num_ctrl: Pointer to sensor serial number
 * @exp_ctrl: Pointer to exposure control
 * @again_ctrl: Pointer to analog gain control
 * @dgain_ctrl: Pointer to analog gain control
 * @num_lanes: Number of data lanes
 * @fps: FPS to be applied on next stream on
 * @lpfr: Lines per frame for long exposure frame
 * @serial: Sensor serial number
 * @cur_mode: Pointer to current selected sensor mode
 * @mutex: Mutex for serializing sensor controls
 * @streaming: Flag indicating streaming state
 */

struct kmb_imx219 {
	struct device *dev;
	struct i2c_client *client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *power_gpio;
	struct clk *inclk;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pclk_ctrl;
	struct v4l2_ctrl *hblank_ctrl;
	struct v4l2_ctrl *vblank_ctrl;
	struct v4l2_ctrl *serial_num_ctrl;
	struct {
		struct v4l2_ctrl *exp_ctrl;
		struct v4l2_ctrl *again_ctrl;
		struct v4l2_ctrl *dgain_ctrl;
		struct v4l2_ctrl *fps_ctrl;
	};
	u32 num_lanes;
	u32 fps;
	u32 lpfr;
	u64 serial;
	const struct kmb_imx219_mode *cur_mode;
	struct mutex mutex;

	bool streaming;
};

/* Sensor mode registers */
static const struct kmb_imx219_reg mode_4L_3280x2464_RAW10_30Hz_regs[] = {
	{0x30EB, 0x05},
	{0x30EB, 0x0C},
	{0x300A, 0xFF},
	{0x300B, 0xFF},
	{0x30EB, 0x05},
	{0x30EB, 0x09},

	{0x0114, 0x03},
	{0x0128, 0x00},
	{0x012A, 0x18},
	{0x012B, 0x00},
	{0x0152, 0x00},

	{0x0160, 0x0A},
	{0x0161, 0x83},
	{0x0162, 0x0D},
	{0x0163, 0x78},
	{0x0164, 0x00},
	{0x0165, 0x00},
	{0x0166, 0x0C},
	{0x0167, 0xCF},
	{0x0168, 0x00},
	{0x0169, 0x00},
	{0x016A, 0x09},
	{0x016B, 0x9F},
	{0x016C, 0x0C},
	{0x016D, 0xD0},
	{0x016E, 0x09},
	{0x016F, 0xA0},
	{0x0170, 0x01},
	{0x0171, 0x01},
	{0x0174, 0x00},
	{0x0175, 0x00},
	{0x018C, 0x0A},
	{0x018D, 0x0A},

	{0x0301, 0x05},
	{0x0303, 0x01},
	{0x0304, 0x03},
	{0x0305, 0x03},
	{0x0306, 0x00},
	{0x0307, 0x57},
	{0x0309, 0x0A},
	{0x030B, 0x01},
	{0x030C, 0x00},
	{0x030D, 0x5A},

	{0x455E, 0x00},
	{0x471E, 0x4B},
	{0x4767, 0x0F},
	{0x4750, 0x14},
	{0x4540, 0x00},
	{0x47B4, 0x14},
	{0x4713, 0x30},
	{0x478B, 0x10},
	{0x478F, 0x10},
	{0x4793, 0x10},
	{0x4797, 0x0E},
	{0x479B, 0x0E},
	{0x5041, 0x00},
};

static const struct kmb_imx219_reg mode_2L_3280x2464_RAW10_21Hz_regs[] = {
	{0x30EB, 0x05},
	{0x30EB, 0x0C},
	{0x300A, 0xFF},
	{0x300B, 0xFF},
	{0x30EB, 0x05},
	{0x30EB, 0x09},

	{0x0114, 0x01},
	{0x0128, 0x00},
	{0x012A, 0x18},
	{0x012B, 0x00},

	{0x0160, 0x09},
	{0x0161, 0xC1},
	{0x0162, 0x0D},
	{0x0163, 0x78},
	{0x0164, 0x00},
	{0x0165, 0x00},
	{0x0166, 0x0C},
	{0x0167, 0xCF},
	{0x0168, 0x00},
	{0x0169, 0x00},
	{0x016A, 0x09},
	{0x016B, 0x9F},
	{0x016C, 0x0C},
	{0x016D, 0xD0},
	{0x016E, 0x09},
	{0x016F, 0xA0},
	{0x0170, 0x01},
	{0x0171, 0x01},
	{0x0174, 0x00},
	{0x0175, 0x00},
	{0x018C, 0x0A},
	{0x018D, 0x0A},

	{0x0301, 0x05},
	{0x0303, 0x01},
	{0x0304, 0x03},
	{0x0305, 0x03},
	{0x0306, 0x00},
	{0x0307, 0x39},
	{0x0309, 0x0A},
	{0x030B, 0x01},
	{0x030C, 0x00},
	{0x030D, 0x72},

	{0x455E, 0x00},
	{0x471E, 0x4B},
	{0x4767, 0x0F},
	{0x4750, 0x14},
	{0x4540, 0x00},
	{0x47B4, 0x14},
	{0x4713, 0x30},
	{0x478B, 0x10},
	{0x478F, 0x10},
	{0x4793, 0x10},
	{0x4797, 0x0E},
	{0x479B, 0x0E},

	{0x5041, 0x00},
};

/* Supported sensor mode configurations */
static const struct kmb_imx219_mode supported_modes[] = {
	{
		.width = 3280,
		.height = 2464,
		.ppln = 3448,
		.lpfr = 2691,
		.skip_lines = 0,
		.pclk = 278400000,

		.code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.num_lanes = 4,
		.fps = {
			.def = 30,
			.max = 30,
			.min = 15,
			.step = 1,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_4L_3280x2464_RAW10_30Hz_regs),
			.regs = mode_4L_3280x2464_RAW10_30Hz_regs,
		},
	},
	{
		.width = 3280,
		.height = 2464,
		.ppln = 3448,
		.lpfr = 2497,
		.skip_lines = 0,
		.pclk = 182400000,

		.code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.num_lanes = 2,
		.fps = {
			.def = 21,
			.max = 21,
			.min = 15,
			.step = 1,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_2L_3280x2464_RAW10_21Hz_regs),
			.regs = mode_2L_3280x2464_RAW10_21Hz_regs,
		},
	}
};

/**
 * to_kmb_imx219 - imv219 V4L2 sub-device to kmb_imx219 device.
 * @subdev: pointer to imx219 V4L2 sub-device device
 *
 * Return: Pointer to kmb_imx219 device
 */
static inline struct kmb_imx219 *to_kmb_imx219(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct kmb_imx219, sd);
}

/**
 * kmb_imx219_read_reg - Read registers.
 * @kmb_imx219: pointer to imx219 device
 * @reg: Register address
 * @len: Length of bytes to read. Max supported bytes is 4
 * @val: Pointer to register value to be filled.
 *
 * Return: 0 if successful
 */
 //TODO: double check this function
static int
kmb_imx219_read_reg(struct kmb_imx219 *kmb_imx219, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&kmb_imx219->sd);
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
 * kmb_imx219_write_reg - Write register
 * @kmb_imx219: pointer to imx219 device
 * @reg: Register address
 * @len: Length of bytes. Max supported bytes is 4
 * @val: Register value
 *
 * Return: 0 if successful
 */
 //TODO: double check this function
static int
kmb_imx219_write_reg(struct kmb_imx219 *kmb_imx219, u16 reg, u32 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&kmb_imx219->sd);
	int buf_i, val_i;
	u8 buf[6], *val_p;
	__be32 val_be;
	int ret;

	if (len > 4) {
		dev_err_ratelimited(kmb_imx219->dev,
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
		dev_err_ratelimited(kmb_imx219->dev,
							"write reg 0x%4.4x return err %d",
							reg, ret);
		return -EIO;
	}

	return 0;
}

/**
 * kmb_imx219_write_regs - Write a list of registers
 * @kmb_imx219: pointer to imx219 device
 * @regs: List of registers to be written
 * @len: Length of registers array
 *
 * Return: 0 if successful
 */
static int kmb_imx219_write_regs(struct kmb_imx219 *kmb_imx219,
								 const struct kmb_imx219_reg *regs, u32 len)
{
	int ret;
	u32 i;

	for (i = 0; i < len; i++) {
		ret = kmb_imx219_write_reg(kmb_imx219,
					   regs[i].address,
					   1, regs[i].val);
		if (ret)
			return ret;
	}

	return 0;
}

static void kmb_imx219_set_lpfr(struct kmb_imx219 *kmb_imx219)
{
	int ret;
	u32 lpfr = kmb_imx219->lpfr;

	dev_dbg(kmb_imx219->dev, "Write FPS %d lpfr %d",
			kmb_imx219->fps, kmb_imx219->lpfr);

	ret = kmb_imx219_write_reg(kmb_imx219,
							   IMX219_REG_FRM_LENGTH_LINES_A_MSB,
							   IMX219_REG_FRM_LENGTH_LINES_SIZE,
							   lpfr);
	if (ret) {
		dev_err(kmb_imx219->dev, "kmb_imx219_set_lpfr A I2C write failed \n");
	}
}

/**
 * kmb_imx219_update_fps - Update current sensor mode to match the selected FPS
 * @kmb_imx219: pointer to imx219 device
 * @mode: pointer to kmb_imx219_mode sensor mode
 *
 * Return: none
 */
static void kmb_imx219_update_fps(struct kmb_imx219 *kmb_imx219,
				  const struct kmb_imx219_mode *mode)
{
	u32 lpfr = (mode->lpfr * mode->fps.def) / kmb_imx219->fps;


	if (lpfr > KMB_IMX219_LPFR_MAX)
		lpfr = KMB_IMX219_LPFR_MAX;

	if (lpfr < mode->height + KMB_MINIMUM_BLANKING)
		lpfr = mode->height + KMB_MINIMUM_BLANKING;

	kmb_imx219->lpfr = lpfr;

	dev_dbg(kmb_imx219->dev, "Selected FPS %d lpfr %d",
		kmb_imx219->fps, kmb_imx219->lpfr);

	kmb_imx219_set_lpfr(kmb_imx219);
}

/**
 * kmb_imx219_open - Open imx219 subdevice
 * @sd: pointer to imx219 V4L2 sub-device structure
 * @fh: pointer to imx219 V4L2 sub-device file handle
 *
 * Return: 0 if successful
 */
static int kmb_imx219_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct kmb_imx219 *kmb_imx219 = to_kmb_imx219(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->state, 0);

	mutex_lock(&kmb_imx219->mutex);

	/* Initialize try_fmt */
	try_fmt->width = kmb_imx219->cur_mode->width;
	try_fmt->height = kmb_imx219->cur_mode->height;
	try_fmt->code = kmb_imx219->cur_mode->code;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&kmb_imx219->mutex);

	return 0;
}

static int kmb_imx219_set_frame_ctrl(struct kmb_imx219 *kmb_imx219,
									 u32 exposure,
									 u32 analog_gain,
									 u32 digital_gain,
									 u32 fps)
{
	int ret;

	kmb_imx219->fps = fps;
	kmb_imx219_update_fps(kmb_imx219, kmb_imx219->cur_mode);

	// -------------------- Analaog GAIN -----------------------------------
	ret = kmb_imx219_write_reg(kmb_imx219, KMB_IMX219_REG_AGAIN,
							   KMB_IMX219_REG_SIZE_AGAIN, analog_gain);
	if (ret) {
		dev_err(kmb_imx219->dev, "kmb_imx219_set_ctrl AGAIN sensor write fail \n");
		return ret;
	}

	// -------------------- Digital GAIN -----------------------------------
	ret = kmb_imx219_write_reg(kmb_imx219, KMB_IMX219_REG_DGAIN,
							   KMB_IMX219_REG_SIZE_DGAIN, digital_gain);
	if (ret) {
		dev_err(kmb_imx219->dev, "kmb_imx219_set_ctrl DGAIN sensor write fail \n");
		return ret;
	}

	// -------------------- INTEGRATION TIME (EXPOSURE) -------------
	// CIT reg value can be: 1 to frame_length_lines-4
	if (0) {
		if (exposure < 1)
			exposure = 1;
		if (exposure > kmb_imx219->lpfr - 4)
			exposure = kmb_imx219->lpfr - 4;
	}
	ret = kmb_imx219_write_reg(kmb_imx219, KMB_IMX219_REG_A_CIT,
							   KMB_IMX219_REG_SIZE_A_CIT, exposure);
	if (ret) {
		dev_err(kmb_imx219->dev, "kmb_imx219_set_ctrl DGAIN sensor write fail \n");
		return ret;
	}

	return 0;
}

/**
 * kmb_imx219_set_ctrl - Set subdevice control. Supported controls:
 *                       V4L2_CID_ANALOGUE_GAIN
 *                       V4L2_CID_EXPOSURE
 *                       Both controls are in one cluster.
 *
 * @ctrl: pointer to v4l2_ctrl structure
 *
 * Return: 0 if successful
 */
static int kmb_imx219_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct kmb_imx219 *kmb_imx219 =
		container_of(ctrl->handler, struct kmb_imx219, ctrl_handler);
	int ret = 0;

	/* Set exposure and gain only if sensor is in power on state */
	if (!pm_runtime_get_if_in_use(kmb_imx219->dev))
		return 0;

	/* Handle the cluster for both controls */
	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
	case V4L2_CID_ANALOGUE_GAIN:
	case V4L2_CID_GAIN:
	case V4L2_CID_AUDIO_TREBLE: { // used for FPS
		u32 exposure = 0;
		u32 analog_gain = 0;
		u32 digital_gain = 0;
		u32 fps = 0;
		exposure = kmb_imx219->exp_ctrl->val;
		analog_gain = kmb_imx219->again_ctrl->val;
		digital_gain = kmb_imx219->dgain_ctrl->val;
		fps = kmb_imx219->fps_ctrl->val;
		dev_dbg(kmb_imx219->dev, "---|||--- exposure = 0x%x \n", exposure);
		dev_dbg(kmb_imx219->dev, "---|||--- analog_gain = 0x%x  \n", analog_gain);
		dev_dbg(kmb_imx219->dev, "---|||--- digital_gain = 0x%x  \n", digital_gain);
		dev_dbg(kmb_imx219->dev, "---|||--- fps = 0x%x  \n", fps);
		ret = kmb_imx219_set_frame_ctrl(kmb_imx219, exposure, analog_gain, digital_gain, fps);
	} break;
	default:
		dev_err(kmb_imx219->dev, "Invalid control %d\n", ctrl->id);
		ret = -EINVAL;
		goto error_pm_runtime_put;
	}

	pm_runtime_put(kmb_imx219->dev);
	return ret;

error_pm_runtime_put:
	pm_runtime_put(kmb_imx219->dev);
	return ret;
}

/**
 * kmb_imx219_get_serial - Reads sensor serial number
 * @kmb_imx219: pointer to kmb_imx219 device
 *
 * Return: 0 if successful
 */
static int kmb_imx219_get_serial(struct kmb_imx219 *kmb_imx219)
{
	int ret = 0, i;
	u32 val;
	u64 serial_num = 0;

	for (i = 0; i < KMB_IMX219_SERIAL_READ_SIZE; i++) {

		ret = kmb_imx219_read_reg(kmb_imx219,
					  KMB_IMX219_SERIAL_ADDR + i, 1, &val);
		if (ret) {
			dev_err(kmb_imx219->dev,
				"Cannot read serial number address");
			return ret;
		}

		serial_num = (serial_num << 8) | val;
	}

	kmb_imx219->serial = serial_num;

	return ret;
}

/* V4l2 subdevice control ops*/
static const struct v4l2_ctrl_ops kmb_imx219_ctrl_ops = {
	.s_ctrl = kmb_imx219_set_ctrl,
};

/**
 * kmb_imx219_parse_mipi_lanes - Parse mipi number of data lanes from dt
 * @kmb_imx219: pointer to kmb camera device
 *
 * Return: 0 if successful
 */
static inline int kmb_imx219_parse_mipi_lanes(struct kmb_imx219 *kmb_imx219)
{
	struct fwnode_handle *fwnode = NULL;
	struct v4l2_fwnode_endpoint ep_data;
	struct fwnode_handle *cam_fwnode = dev_fwnode(kmb_imx219->dev);
	int ret;

	fwnode = fwnode_graph_get_next_endpoint(cam_fwnode, fwnode);

	memset(&ep_data, 0, sizeof(ep_data));
	ep_data.bus_type = V4L2_MBUS_CSI2_DPHY;
	ret = v4l2_fwnode_endpoint_parse(fwnode, &ep_data);
	if (ret) {
		dev_err(kmb_imx219->dev, "No endpoint to parse in this fwnode");
		return -ENOENT;
	}

	kmb_imx219->num_lanes = ep_data.bus.mipi_csi2.num_data_lanes;
	dev_dbg(kmb_imx219->dev, "num_data_lanes %d\n", kmb_imx219->num_lanes);

	return 0;
}

/**
 * kmb_imx219_get_camera_mode_by_fmt - Get the most appropriate camera
 *         mode that meets the code and resolution criteria
 * @kmb_imx219: pointer to kmb_imx219 device
 * @code: media bus format code
 * @width: frame width
 * @height: frame height
 *
 * Return: pointer to the most appropriate camera mode
 */
static const struct kmb_imx219_mode *
kmb_imx219_get_camera_mode_by_fmt(struct kmb_imx219 *kmb_imx219, u32 code,
				  u32 width, u32 height)
{
	const struct kmb_imx219_mode *mode = supported_modes;
	int n = ARRAY_SIZE(supported_modes);
	int i;
	mode = supported_modes;
	n = ARRAY_SIZE(supported_modes);

	for (i = 0; i < n; i++) {
		if (mode[i].code == code && mode[i].width == width &&
		    mode[i].height == height)
			return &mode[i];
	}

	return NULL;
}

/**
 * kmb_imx219_filter_supported_modes - Filter supported sensor modes
 * @kmb_imx219: pointer to kmb_imx219 device
 *
 * Filter supported sensor modes based on number of CSI-2 lanes detected
 *
 * Return: 0 if successful
 */
static const struct kmb_imx219_mode *kmb_imx219_filter_supported_modes(struct kmb_imx219 *kmb_imx219)
{
	static const struct kmb_imx219_mode *modes;
	int num_modes;
	int i;


	modes = supported_modes;
	num_modes = ARRAY_SIZE(supported_modes);

	for (i = 0; i < num_modes; i++) {
		if (kmb_imx219->num_lanes == modes[i].num_lanes)
			return &modes[i];
	}

	return NULL;
}

/**
 * kmb_imx219_enum_mbus_code - Enumerate V4L2 sub-device mbus codes
 * @sd: pointer to imx219 V4L2 sub-device structure
 * @cfg: V4L2 sub-device pad configuration
 * @fsize: V4L2 sub-device size enumeration need to be filled
 *
 * Return: 0 if successful
 */
static int kmb_imx219_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct kmb_imx219 *kmb_imx219 = to_kmb_imx219(sd);

	if (code->index > 0)
		return -EINVAL;

	mutex_lock(&kmb_imx219->mutex);
	code->code = supported_modes[0].code;
	mutex_unlock(&kmb_imx219->mutex);

	return 0;
}

/**
 * kmb_imx219_enum_frame_size - Enumerate V4L2 sub-device frame sizes
 * @sd: pointer to imx219 V4L2 sub-device structure
 * @cfg: V4L2 sub-device pad configuration
 * @fsize: V4L2 sub-device size enumeration need to be filled
 *
 * Return: 0 if successful
 */
static int kmb_imx219_enum_frame_size(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *cfg,
				      struct v4l2_subdev_frame_size_enum *fsize)
{
	struct kmb_imx219 *kmb_imx219 = to_kmb_imx219(sd);

	if (fsize->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	mutex_lock(&kmb_imx219->mutex);
	if (fsize->code != supported_modes[fsize->index].code) {
		mutex_unlock(&kmb_imx219->mutex);
		return -EINVAL;
	}

	fsize->min_width = supported_modes[fsize->index].width;
	fsize->max_width = fsize->min_width;
	fsize->min_height = supported_modes[fsize->index].height;
	fsize->max_height = fsize->min_height;
	mutex_unlock(&kmb_imx219->mutex);

	return 0;
}

/**
 * kmb_imx219_enum_frame_interval - Enumerate V4L2 sub-device frame intervals
 * @sd: pointer to imx219 V4L2 sub-device structure
 * @cfg: V4L2 sub-device pad configuration
 * @fsize: V4L2 sub-device size enumeration need to be filled
 *
 * callback for VIDIOC_SUBDEV_ENUM_FRAME_INTERVAL()
 *
 * Return: 0 if successful
 */
static int
kmb_imx219_enum_frame_interval(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *cfg,
			       struct v4l2_subdev_frame_interval_enum *fie)
{
	struct kmb_imx219 *kmb_imx219 = to_kmb_imx219(sd);
	const struct kmb_imx219_mode *mode;
	int fps;
	int ret = 0;

	if (fie->pad)
		return -EINVAL;

	mutex_lock(&kmb_imx219->mutex);

	if (fie->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		if (fie->code != kmb_imx219->cur_mode->code ||
		    fie->width != kmb_imx219->cur_mode->width ||
		    fie->height != kmb_imx219->cur_mode->height) {
			ret = -EINVAL;
			goto exit_unlock;
		}

		mode = kmb_imx219->cur_mode;
	} else {
		mode = kmb_imx219_get_camera_mode_by_fmt(kmb_imx219,
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

	dev_dbg(kmb_imx219->dev, "Enum FPS %d %d/%d", fps,
		fie->interval.numerator, fie->interval.denominator);

exit_unlock:
	mutex_unlock(&kmb_imx219->mutex);
	return ret;
}

/**
 * kmb_imx219_fill_pad_format - Fill subdevice pad format
 *                              from selected sensor mode
 * @kmb_imx219: pointer to kmb_imx219 device
 * @mode: Pointer to kmb_imx219_mode sensor mode
 * @fmt: V4L2 sub-device format need to be filled
 *
 * Return: none
 */
static void kmb_imx219_fill_pad_format(struct kmb_imx219 *kmb_imx219,
				       const struct kmb_imx219_mode *mode,
				       struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.code = mode->code;
	fmt->format.field = V4L2_FIELD_NONE;
}

/**
 * kmb_imx219_skip_top_lines - Skip top lines containing metadata
 * @sd: pointer to imx219 V4L2 sub-device structure
 * @lines: number of lines to be skipped
 *
 * Return: 0 if successful
 */
static int kmb_imx219_skip_top_lines(struct v4l2_subdev *sd, u32 *lines)
{
	struct kmb_imx219 *kmb_imx219 = to_kmb_imx219(sd);

	*lines = kmb_imx219->cur_mode->skip_lines;

	return 0;
}

/**
 * kmb_imx219_update_controls - Update control ranges based on streaming mode
 * @kmb_imx219: pointer to kmb_imx219 device
 * @mode: pointer to kmb_imx219_mode sensor mode
 *
 * Return: none
 */
static void kmb_imx219_update_controls(struct kmb_imx219 *kmb_imx219,
					const struct kmb_imx219_mode *mode)
{
		__v4l2_ctrl_s_ctrl(kmb_imx219->vblank_ctrl,
				   kmb_imx219->lpfr - mode->height);
		__v4l2_ctrl_s_ctrl(kmb_imx219->hblank_ctrl,
				   mode->ppln - mode->width);
		__v4l2_ctrl_modify_range(kmb_imx219->pclk_ctrl,
					mode->pclk, mode->pclk,
					1, mode->pclk);
		__v4l2_ctrl_modify_range(kmb_imx219->exp_ctrl,
					KMB_IMX219_EXPOSURE_MIN,
					kmb_imx219->lpfr,
					1, KMB_IMX219_EXPOSURE_DEFAULT);
}

/**
 * kmb_imx219_get_pad_format - Get subdevice pad format
 * @sd: pointer to imx219 V4L2 sub-device structure
 * @cfg: V4L2 sub-device pad configuration
 * @fmt: V4L2 sub-device format need to be set
 *
 * Return: 0 if successful
 */
static int kmb_imx219_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *cfg,
				 struct v4l2_subdev_format *fmt)
{
	struct kmb_imx219 *kmb_imx219 = to_kmb_imx219(sd);

	mutex_lock(&kmb_imx219->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *framefmt;

		framefmt = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		fmt->format = *framefmt;
	} else {
		kmb_imx219_fill_pad_format(kmb_imx219,
					   kmb_imx219->cur_mode,
					   fmt);
	}

	mutex_unlock(&kmb_imx219->mutex);

	return 0;
}

/**
 * kmb_imx219_set_pad_format - Set subdevice pad format
 * @sd: pointer to imx219 V4L2 sub-device structure
 * @cfg: V4L2 sub-device pad configuration
 * @fmt: V4L2 sub-device format need to be set
 *
 * Return: 0 if successful
 */
static int
kmb_imx219_set_pad_format(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct kmb_imx219 *kmb_imx219 = to_kmb_imx219(sd);
	const struct kmb_imx219_mode *mode;
	struct v4l2_mbus_framefmt *framefmt;

	mutex_lock(&kmb_imx219->mutex);

	mode = kmb_imx219_filter_supported_modes(kmb_imx219);
	if (!mode) {
		dev_err(sd->dev, "No camera mode was selected!");
		mutex_unlock(&kmb_imx219->mutex);
		return -EINVAL;
	}

	kmb_imx219_update_fps(kmb_imx219, mode);
	kmb_imx219_fill_pad_format(kmb_imx219, mode, fmt);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		framefmt = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		*framefmt = fmt->format;
	} else {
		kmb_imx219_update_controls(kmb_imx219, mode);
		kmb_imx219->cur_mode = mode;
	}

	mutex_unlock(&kmb_imx219->mutex);

	return 0;
}

/**
 * kmb_imx219_start_streaming - Start sensor stream
 * @kmb_imx219: pointer to kmb_imx219 device
 *
 * Return: 0 if successful
 */
static int kmb_imx219_start_streaming(struct kmb_imx219 *kmb_imx219)
{
	const struct kmb_imx219_reg_list *reg_list;
	int ret;

	/* Write sensor mode registers */
	reg_list = &kmb_imx219->cur_mode->reg_list;
	ret = kmb_imx219_write_regs(kmb_imx219, reg_list->regs,
				    reg_list->num_of_regs);
	if (ret) {
		dev_err(kmb_imx219->dev, "fail to write initial registers");
		return ret;
	}

	/* Setup handler will write actual exposure and gain */
	ret =  __v4l2_ctrl_handler_setup(kmb_imx219->sd.ctrl_handler);
	if (ret) {
		dev_err(kmb_imx219->dev, "fail to setup handler");
		return ret;
	}

	//kmb_imx219_set_lpfr(kmb_imx219);

	/* Start streaming */
	ret = kmb_imx219_write_reg(kmb_imx219, KMB_IMX219_REG_MODE_SELECT,
				   1, KMB_IMX219_MODE_STREAMING);
	if (ret) {
		dev_err(kmb_imx219->dev, "fail to start streaming");
		return ret;
	}

	return 0;
}

/**
 * kmb_imx219_stop_streaming - Stop sensor stream
 * @kmb_imx219: pointer to kmb_imx219 device
 *
 * Return: 0 if successful
 */
static int kmb_imx219_stop_streaming(struct kmb_imx219 *kmb_imx219)
{
	return kmb_imx219_write_reg(kmb_imx219, KMB_IMX219_REG_MODE_SELECT,
				1, KMB_IMX219_MODE_STANDBY);
}

/**
 * kmb_imx219_set_stream - Enable sensor streaming
 * @sd: pointer to imx219 subdevice
 * @enable: Set to enable sensor streaming
 *
 * Return: 0 if successful
 */
static int kmb_imx219_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct kmb_imx219 *kmb_imx219 = to_kmb_imx219(sd);
	int ret;

	mutex_lock(&kmb_imx219->mutex);

	if (kmb_imx219->streaming == enable) {
		mutex_unlock(&kmb_imx219->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_get_sync(kmb_imx219->dev);
		if (ret)
			goto error_unlock;

		ret = kmb_imx219_start_streaming(kmb_imx219);
		if (ret)
			goto error_power_off;
	} else {
		kmb_imx219_stop_streaming(kmb_imx219);
		pm_runtime_put(kmb_imx219->dev);
	}

	kmb_imx219->streaming = enable;

	mutex_unlock(&kmb_imx219->mutex);

	return 0;

error_power_off:
	pm_runtime_put(kmb_imx219->dev);
error_unlock:
	mutex_unlock(&kmb_imx219->mutex);
	return ret;
}

/**
 * kmb_imx219_get_frame_interval - Get subdevice frame interval
 * @sd: pointer to imx219 V4L2 sub-device structure
 * @interval: V4L2 sub-device current farme interval
 *
 * Return: 0 if successful
 */
static int kmb_imx219_get_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *interval)
{
	struct kmb_imx219 *kmb_imx219 = to_kmb_imx219(sd);

	if (interval->pad)
		return -EINVAL;

	mutex_lock(&kmb_imx219->mutex);
	interval->interval.numerator = 1;
	interval->interval.denominator = kmb_imx219->fps;
	mutex_unlock(&kmb_imx219->mutex);

	return 0;
}

/**
 * kmb_imx219_set_frame_interval - Set subdevice frame interval
 * @sd: pointer to imx219 V4L2 sub-device structure
 * @interval: V4L2 sub-device farme interval to be set
 *
 * Return: 0 if successful
 */
static int kmb_imx219_set_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *interval)
{
	struct kmb_imx219 *kmb_imx219 = to_kmb_imx219(sd);
	u32 fps;

	if (interval->pad)
		return -EINVAL;

	mutex_lock(&kmb_imx219->mutex);
	fps = (u32)(interval->interval.denominator /
		interval->interval.numerator);
	if (fps < kmb_imx219->cur_mode->fps.min) {
		interval->interval.numerator = 1;
		interval->interval.denominator = kmb_imx219->cur_mode->fps.min;
	} else if (fps > kmb_imx219->cur_mode->fps.max) {
		interval->interval.numerator = 1;
		interval->interval.denominator = kmb_imx219->cur_mode->fps.max;
	}

	kmb_imx219->fps = (u32) (interval->interval.denominator /
			 interval->interval.numerator);

	kmb_imx219_update_fps(kmb_imx219, kmb_imx219->cur_mode);

	mutex_unlock(&kmb_imx219->mutex);

	dev_dbg(kmb_imx219->dev, "Set frame interval %d/%d",
		interval->interval.numerator,
		interval->interval.denominator);

	return 0;
}

/**
 * kmb_imx219_power_on - Sensor power on sequence
 * @kmb_imx219: imb_imx219 device
 *
 * Return: 0 if successful
 */
static int kmb_imx219_power_on(struct kmb_imx219 *kmb_imx219)
{
	int ret;

	/* request optional power pin */
	kmb_imx219->power_gpio =
		gpiod_get_optional(kmb_imx219->dev, "power", GPIOD_OUT_HIGH);
	if (IS_ERR(kmb_imx219->power_gpio)) {
		ret = PTR_ERR(kmb_imx219->power_gpio);
		dev_err(kmb_imx219->dev, "failed to get power gpio %d", ret);
		return ret;
	}
	gpiod_put(kmb_imx219->power_gpio);

	/* request optional reset pin */
	kmb_imx219->reset_gpio =
		gpiod_get_optional(kmb_imx219->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(kmb_imx219->reset_gpio)) {
		ret = PTR_ERR(kmb_imx219->reset_gpio);
		dev_err(kmb_imx219->dev, "failed to get reset gpio %d", ret);
		return ret;
	}

	/* Ignore the call if reset gpio is not present */
	if (kmb_imx219->reset_gpio)
		gpiod_set_value_cansleep(kmb_imx219->reset_gpio, 1);

	ret = clk_prepare_enable(kmb_imx219->inclk);
	if (ret) {
		dev_err(kmb_imx219->dev, "fail to enable inclk\n");
		goto error_reset;
	}

	usleep_range(18000, 20000);

	return 0;

error_reset:
	if (kmb_imx219->reset_gpio) {
		gpiod_set_value_cansleep(kmb_imx219->reset_gpio, 0);
		gpiod_put(kmb_imx219->reset_gpio);
		kmb_imx219->reset_gpio = NULL;
	}

	return ret;
}

/**
 * kmb_imx219_power_off - Sensor power off sequence
 * @kmb_imx219: imb_imx219 device
 */
static void kmb_imx219_power_off(struct kmb_imx219 *kmb_imx219)
{
	/* Ignore the call if reset gpio is not present */
	if (kmb_imx219->reset_gpio)
		gpiod_set_value_cansleep(kmb_imx219->reset_gpio, 0);
	usleep_range(500, 550);
	clk_disable_unprepare(kmb_imx219->inclk);

	if (kmb_imx219->reset_gpio) {
		gpiod_put(kmb_imx219->reset_gpio);
		kmb_imx219->reset_gpio = NULL;
	}
}

/**
 * kmb_imx219_detect - Detect imx219 sensor
 * @kmb_imx219: pointer to kmb_imx219 device
 *
 * Return: 0 if successful, -EIO if sensor id does not match
 */
static int kmb_imx219_detect(struct kmb_imx219 *kmb_imx219)
{
	int ret;
	u32 val;

	ret = kmb_imx219_read_reg(kmb_imx219, KMB_IMX219_REG_ID, 2, &val);
	if (ret)
		return ret;

	if (val != KMB_IMX219_ID) {
		dev_err(kmb_imx219->dev, "chip id mismatch: %x!=%x",
			KMB_IMX219_ID, val);
		return -EIO;
	}

	return 0;
}

/* V4l2 subdevice ops */
static const struct v4l2_subdev_video_ops kmb_imx219_video_ops = {
	.s_stream = kmb_imx219_set_stream,
	.g_frame_interval = kmb_imx219_get_frame_interval,
	.s_frame_interval = kmb_imx219_set_frame_interval,
};

static const struct v4l2_subdev_pad_ops kmb_imx219_pad_ops = {
	.enum_mbus_code = kmb_imx219_enum_mbus_code,
	.enum_frame_size = kmb_imx219_enum_frame_size,
	.enum_frame_interval = kmb_imx219_enum_frame_interval,
	.get_fmt = kmb_imx219_get_pad_format,
	.set_fmt = kmb_imx219_set_pad_format,
};

static const struct v4l2_subdev_sensor_ops kmb_imx219_sensor_ops = {
	.g_skip_top_lines = kmb_imx219_skip_top_lines,
};

static const struct v4l2_subdev_ops kmb_imx219_subdev_ops = {
	.video = &kmb_imx219_video_ops,
	.pad = &kmb_imx219_pad_ops,
	.sensor = &kmb_imx219_sensor_ops,
};

static const struct media_entity_operations kmb_imx219_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops kmb_imx219_internal_ops = {
	.open = kmb_imx219_open,
};

/**
 * kmb_imx219_init_controls - Initialize sensor subdevice controls
 * @kmb_imx219: pointer to kmb_imx219 device
 *
 * Return: 0 if successful
 */
static int kmb_imx219_init_controls(struct kmb_imx219 *kmb_imx219)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &kmb_imx219->ctrl_handler;
	const struct kmb_imx219_mode *mode = kmb_imx219->cur_mode;
	u32 hblank;
	u32 vblank;
	int ret;

	const struct v4l2_ctrl_config serial_number = {
		.id = V4L2_CID_SENSOR_SERIAL_NUMBER,
		.name = "V4L2_CID_SENSOR_SERIAL_NUMBER",
		.min = kmb_imx219->serial,
		.max = kmb_imx219->serial,
		.def = kmb_imx219->serial,
		.step = KMB_IMX219_SERIAL_STEP,
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.menu_skip_mask = 0,
	};

	ctrl_hdlr = &kmb_imx219->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, V4L2_CTRL_HNDL_SIZE);
	if (ret)
		return ret;

	/* Serialize controls with sensor device */
	ctrl_hdlr->lock = &kmb_imx219->mutex;

	/* Initialize exposure and gain */
	kmb_imx219->exp_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						 &kmb_imx219_ctrl_ops,
						 V4L2_CID_EXPOSURE,
						 KMB_IMX219_EXPOSURE_MIN,
						 mode->lpfr,
						 KMB_IMX219_EXPOSURE_STEP,
						 KMB_IMX219_EXPOSURE_DEFAULT);

	kmb_imx219->again_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						   &kmb_imx219_ctrl_ops,
						   V4L2_CID_ANALOGUE_GAIN,
						   KMB_IMX219_AGAIN_MIN,
						   KMB_IMX219_AGAIN_MAX,
						   KMB_IMX219_AGAIN_STEP,
						   KMB_IMX219_AGAIN_DEFAULT);

	kmb_imx219->dgain_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						   &kmb_imx219_ctrl_ops,
						   V4L2_CID_GAIN,
						   KMB_IMX219_DGAIN_MIN,
						   KMB_IMX219_DGAIN_MAX,
						   KMB_IMX219_DGAIN_STEP,
						   KMB_IMX219_DGAIN_DEFAULT);

	kmb_imx219->fps_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
											 &kmb_imx219_ctrl_ops,
											 V4L2_CID_AUDIO_TREBLE,
											 KMB_IMX219_FPS_MIN,
											 KMB_IMX219_FPS_MAX,
											 KMB_IMX219_FPS_STEP,
											 KMB_IMX219_FPS_DEFAULT);

	v4l2_ctrl_cluster(V4L2_CLUSTER_SIZE, &kmb_imx219->exp_ctrl);

	/* Read only controls */
	kmb_imx219->serial_num_ctrl = v4l2_ctrl_new_custom(ctrl_hdlr,
							   &serial_number,
							   NULL);

	if (kmb_imx219->serial_num_ctrl)
		kmb_imx219->serial_num_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	kmb_imx219->pclk_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						  &kmb_imx219_ctrl_ops,
						  V4L2_CID_PIXEL_RATE,
						  mode->pclk, mode->pclk,
						  1, mode->pclk);
	if (kmb_imx219->pclk_ctrl)
		kmb_imx219->pclk_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank = mode->lpfr - mode->height;
	kmb_imx219->vblank_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						    &kmb_imx219_ctrl_ops,
						    V4L2_CID_VBLANK,
						    KMB_IMX219_REG_MIN,
						    KMB_IMX219_REG_MAX,
						    1, vblank);
	if (kmb_imx219->vblank_ctrl)
		kmb_imx219->vblank_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	hblank = mode->ppln - mode->width;
	kmb_imx219->hblank_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						    &kmb_imx219_ctrl_ops,
						    V4L2_CID_HBLANK,
						    KMB_IMX219_REG_MIN,
						    KMB_IMX219_REG_MAX,
						    1, hblank);
	if (kmb_imx219->hblank_ctrl)
		kmb_imx219->hblank_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(kmb_imx219->dev, "control init failed: %d", ret);
		goto error;
	}

	kmb_imx219->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

/* --------------- probe as i2c device -------------------- */

/**
 * kmb_imx219_i2c_resume - PM resume callback
 * @dev: i2c device
 *
 * Return: 0 if successful
 */
static int kmb_imx219_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct kmb_imx219 *kmb_imx219 = to_kmb_imx219(sd);

	return kmb_imx219_power_on(kmb_imx219);
}

/**
 * kmb_imx219_i2c_suspend - PM suspend callback
 * @dev: i2c device
 *
 * Return: 0 if successful
 */
static int kmb_imx219_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct kmb_imx219 *kmb_imx219 = to_kmb_imx219(sd);

	kmb_imx219_power_off(kmb_imx219);

	return 0;
}

/**
 * kmb_imx219_i2c_probe - I2C client device binding
 * @client: pointer to i2c client device
 * @id: pointer to i2c device id
 *
 * Return: 0 if successful
 */
static int kmb_imx219_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct kmb_imx219 *kmb_imx219;
	int ret;

	kmb_imx219 = devm_kzalloc(&client->dev, sizeof(*kmb_imx219),
				  GFP_KERNEL);
	if (!kmb_imx219)
		return -ENOMEM;

	mutex_init(&kmb_imx219->mutex);

	kmb_imx219->dev = &client->dev;

	/* Initialize subdev */
	v4l2_i2c_subdev_init(&kmb_imx219->sd, client, &kmb_imx219_subdev_ops);

	/* Get sensor input clock */
	kmb_imx219->inclk = devm_clk_get(&client->dev, "inclk");
	if (IS_ERR(kmb_imx219->inclk)) {
		ret = PTR_ERR(kmb_imx219->inclk);
		dev_err(&client->dev, "could not get inclk");
		goto error_mutex_destroy;
	}

	ret = clk_set_rate(kmb_imx219->inclk, KMB_IMX219_INCLK_RATE);
	if (ret) {
		dev_err(&client->dev, "could not set inclk frequency\n");
		goto error_mutex_destroy;
	}

	ret = kmb_imx219_power_on(kmb_imx219);
	if (ret) {
		dev_err(&client->dev, "failed to power-on the sensor\n");
		goto error_mutex_destroy;
	}

	/* Check module identity */
	ret = kmb_imx219_detect(kmb_imx219);
	if (ret) {
		dev_err(&client->dev, "failed to find sensor: %d", ret);
		goto error_sensor_power_off;
	}

	/* Set default mode to max resolution */
	kmb_imx219->cur_mode = &supported_modes[0];
	kmb_imx219->fps = kmb_imx219->cur_mode->fps.def;
	kmb_imx219->lpfr = kmb_imx219->cur_mode->lpfr;

	ret = kmb_imx219_init_controls(kmb_imx219);
	if (ret) {
		dev_err(&client->dev, "failed to init controls: %d", ret);
		goto error_sensor_power_off;
	}

	/* Initialize subdev */
	kmb_imx219->sd.internal_ops = &kmb_imx219_internal_ops;
	kmb_imx219->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	kmb_imx219->sd.entity.ops = &kmb_imx219_subdev_entity_ops;
	kmb_imx219->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
	kmb_imx219->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&kmb_imx219->sd.entity, 1,
				     &kmb_imx219->pad);
	if (ret) {
		dev_err(&client->dev, "failed to init entity pads: %d", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&kmb_imx219->sd);
	if (ret < 0) {
		dev_err(&client->dev,
				"failed to register async subdev: %d", ret);
		goto error_media_entity;
	}

	ret = kmb_imx219_parse_mipi_lanes(kmb_imx219);
	if (ret < 0) {
		dev_err(kmb_imx219->dev, "Fail to parse device tree\n");
		goto error_media_entity;
	}

	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	return 0;

error_media_entity:
	media_entity_cleanup(&kmb_imx219->sd.entity);
error_handler_free:
	v4l2_ctrl_handler_free(kmb_imx219->sd.ctrl_handler);
error_sensor_power_off:
	kmb_imx219_power_off(kmb_imx219);
error_mutex_destroy:
	mutex_destroy(&kmb_imx219->mutex);

	return ret;
}

/**
 * kmb_imx219_i2c_remove - I2C client device unbinding
 * @client: pointer to I2C client device
 *
 * Return: 0 if successful
 */
static int kmb_imx219_i2c_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct kmb_imx219 *kmb_imx219 = to_kmb_imx219(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(&client->dev);
	pm_runtime_suspended(&client->dev);

	mutex_destroy(&kmb_imx219->mutex);

	return 0;
}

static const struct dev_pm_ops kmb_imx219_i2c_pm_ops = {
	SET_RUNTIME_PM_OPS(kmb_imx219_i2c_suspend, kmb_imx219_i2c_resume, NULL)
};

static const struct i2c_device_id kmb_imx219_i2c_id_table[] = {
	{KMB_IMX219_DRV_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, kmb_imx219_i2c_id_table);

static struct i2c_driver kmb_imx219_i2c_driver = {
	.probe = kmb_imx219_i2c_probe,
	.remove = kmb_imx219_i2c_remove,
	.id_table = kmb_imx219_i2c_id_table,
	.driver = {
		.owner = THIS_MODULE,
		.pm = &kmb_imx219_i2c_pm_ops,
		.name = KMB_IMX219_DRV_NAME,
	},
};

/* --------------- probe as platform device ----------------- */

/**
 * kmb_imx219_platform_resume - PM resume callback
 * @dev: platform device
 *
 * Return: 0 if successful
 */
static int kmb_imx219_platform_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct kmb_imx219 *kmb_imx219 = platform_get_drvdata(pdev);

	return kmb_imx219_power_on(kmb_imx219);
}

/**
 * kmb_imx219_platform_suspend - PM suspend callback
 * @dev: platform device
 *
 * Return: 0 if successful
 */
static int kmb_imx219_platform_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct kmb_imx219 *kmb_imx219 = platform_get_drvdata(pdev);

	kmb_imx219_power_off(kmb_imx219);

	return 0;
}

/**
 * kmb_imx219_get_i2c_client - Get I2C client
 * @kmb_imx219: pointer to kmb_imx219 device
 *
 * Return: 0 if successful
 */
static int kmb_imx219_get_i2c_client(struct kmb_imx219 *kmb_imx219)
{
	struct i2c_board_info info = {
		I2C_BOARD_INFO("kmb-imx219-sensor-p", 0x1A)};
	const unsigned short addr_list[] = {0x1A, I2C_CLIENT_END};
	struct i2c_adapter *i2c_adp;
	struct device_node *phandle;

	phandle = of_parse_phandle(kmb_imx219->dev->of_node, "i2c-bus", 0);
	if (!phandle)
		return -ENODEV;

	i2c_adp = of_get_i2c_adapter_by_node(phandle);
	of_node_put(phandle);
	if (!i2c_adp)
		return -EPROBE_DEFER;

	kmb_imx219->client =
		i2c_new_scanned_device(i2c_adp, &info, addr_list, NULL);
	i2c_put_adapter(i2c_adp);
	if (IS_ERR(kmb_imx219->client))
		return PTR_ERR(kmb_imx219->client);

	dev_dbg(kmb_imx219->dev, "Detected on i2c address %x", info.addr);
	return 0;
}

/**
 * kmb_imx219_pdev_probe - Platform device binding
 * @pdev: pointer to platform device
 *
 * Return: 0 if successful
 */
static int kmb_imx219_pdev_probe(struct platform_device *pdev)
{
	struct kmb_imx219 *kmb_imx219;
	struct gpio_descs *detect_gpios;
	int ret;

	kmb_imx219 = devm_kzalloc(&pdev->dev, sizeof(*kmb_imx219), GFP_KERNEL);
	if (!kmb_imx219)
		return -ENOMEM;

	platform_set_drvdata(pdev, kmb_imx219);

	mutex_init(&kmb_imx219->mutex);

	kmb_imx219->dev = &pdev->dev;

	/* Initialize subdev */
	v4l2_subdev_init(&kmb_imx219->sd, &kmb_imx219_subdev_ops);
	kmb_imx219->sd.owner = pdev->dev.driver->owner;
	kmb_imx219->sd.dev = &pdev->dev;

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

	/* Get sensor input clock */
	kmb_imx219->inclk = devm_clk_get(&pdev->dev, "inclk");
	if (IS_ERR(kmb_imx219->inclk)) {
		ret = PTR_ERR(kmb_imx219->inclk);
		dev_err(&pdev->dev, "could not get inclk");
		goto error_put_detect_gpios;
	}

	ret = clk_set_rate(kmb_imx219->inclk, KMB_IMX219_INCLK_RATE);
	if (ret) {
		dev_err(&pdev->dev, "could not set inclk frequency\n");
		goto error_put_detect_gpios;
	}

	ret = kmb_imx219_power_on(kmb_imx219);
	if (ret) {
		dev_dbg(&pdev->dev, "failed to power-on the sensor %d", ret);
		/* Defer the probe if resourcess are busy */
		ret = (ret == -EBUSY) ? -EPROBE_DEFER : ret;
		goto error_put_detect_gpios;
	}

	ret = kmb_imx219_get_i2c_client(kmb_imx219);
	if (ret) {
		dev_dbg(&pdev->dev, "failed to get i2c %d\n", ret);
		goto error_sensor_power_off;
	}

	v4l2_set_subdevdata(&kmb_imx219->sd, kmb_imx219->client);
	i2c_set_clientdata(kmb_imx219->client, &kmb_imx219->sd);
	v4l2_i2c_subdev_set_name(&kmb_imx219->sd, kmb_imx219->client,
				 KMB_IMX219_DRV_NAME, pdev->name);

	/* Check module identity */
	ret = kmb_imx219_detect(kmb_imx219);
	if (ret) {
		dev_err(&pdev->dev, "failed to find sensor: %d", ret);
		goto error_unregister_i2c_dev;
	}

	ret = kmb_imx219_get_serial(kmb_imx219);
	if (ret)
		dev_err(&pdev->dev,
			"failed to read sensor serial number: %d", ret);

	/* Set default mode to max resolution */
	kmb_imx219->cur_mode = &supported_modes[0];
	kmb_imx219->fps = kmb_imx219->cur_mode->fps.def;
	kmb_imx219->lpfr = kmb_imx219->cur_mode->lpfr;

	ret = kmb_imx219_init_controls(kmb_imx219);
	if (ret) {
		dev_err(&pdev->dev, "failed to init controls: %d", ret);
		goto error_unregister_i2c_dev;
	}

	/* Initialize subdev */
	kmb_imx219->sd.internal_ops = &kmb_imx219_internal_ops;
	kmb_imx219->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	kmb_imx219->sd.entity.ops = &kmb_imx219_subdev_entity_ops;
	kmb_imx219->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
	kmb_imx219->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&kmb_imx219->sd.entity, 1,
				     &kmb_imx219->pad);
	if (ret) {
		dev_err(&pdev->dev, "failed to init entity pads: %d", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&kmb_imx219->sd);
	if (ret < 0) {
		dev_err(&pdev->dev,
				"failed to register async subdev: %d", ret);
		goto error_media_entity;
	}

	ret = kmb_imx219_parse_mipi_lanes(kmb_imx219);
	if (ret < 0) {
		dev_err(kmb_imx219->dev, "Fail to parse device tree\n");
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
	media_entity_cleanup(&kmb_imx219->sd.entity);
error_handler_free:
	v4l2_ctrl_handler_free(kmb_imx219->sd.ctrl_handler);
error_unregister_i2c_dev:
	if (kmb_imx219->client)
		i2c_unregister_device(kmb_imx219->client);
error_sensor_power_off:
	kmb_imx219_power_off(kmb_imx219);
error_put_detect_gpios:
	if (detect_gpios)
		gpiod_put_array(detect_gpios);
error_mutex_destroy:
	mutex_destroy(&kmb_imx219->mutex);
	return ret;
}


/**
 * kmb_imx219_pdev_remove - Platform device unbinding
 * @pdev: pointer to platform device
 *
 * Return: 0 if successful
 */
static int kmb_imx219_pdev_remove(struct platform_device *pdev)
{
	struct kmb_imx219 *kmb_imx219 = platform_get_drvdata(pdev);
	struct v4l2_subdev *sd = &kmb_imx219->sd;

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(&pdev->dev);
	pm_runtime_suspended(&pdev->dev);

	if (kmb_imx219->client)
		i2c_unregister_device(kmb_imx219->client);

	mutex_destroy(&kmb_imx219->mutex);

	return 0;
}

static const struct dev_pm_ops kmb_imx219_platform_pm_ops = {
	SET_RUNTIME_PM_OPS(kmb_imx219_platform_suspend,
			   kmb_imx219_platform_resume, NULL)
};

static const struct of_device_id kmb_imx219_id_table[] = {
	{.compatible = "intel,kmb-imx219-sensor-p"},
	{}
};
MODULE_DEVICE_TABLE(of, kmb_imx219_id_table);

static struct platform_driver kmb_imx219_platform_driver = {
	.probe	= kmb_imx219_pdev_probe,
	.remove = kmb_imx219_pdev_remove,
	.driver = {
		.name = KMB_IMX219_DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &kmb_imx219_platform_pm_ops,
		.of_match_table = kmb_imx219_id_table,
	}
};

static int __init kmb_imx219_init(void)
{
	int ret;

	ret = i2c_add_driver(&kmb_imx219_i2c_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&kmb_imx219_platform_driver);
	if (ret)
		i2c_del_driver(&kmb_imx219_i2c_driver);

	return ret;
}

static void __exit kmb_imx219_exit(void)
{
	i2c_del_driver(&kmb_imx219_i2c_driver);
	platform_driver_unregister(&kmb_imx219_platform_driver);
}

module_init(kmb_imx219_init);
module_exit(kmb_imx219_exit);

MODULE_DESCRIPTION("KeemBay imx219 Sensor driver");
MODULE_LICENSE("GPL v2");
