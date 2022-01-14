// SPDX-License-Identifier: GPL-2.0-only
/*
 * kmb-mvmx.c - KeemBay Camera Smart Sensor Driver.
 *
 * Copyright (C) 2020 Intel Corporation
 */
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/kmb-isp-ctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <linux/util_macros.h>

/*
 * I2C communication protocol details:
 * The I2C interface will be used for configuration and commands - physical
 * device TBC (either I2C1 or I2C2 depending on tap point availability).
 * 7Bit address 0x66 will be used for communication
 * (this is 0xCC/0xCD when viewed with the read/write bit appended)
 */

/* Number of active MIPI lanes. Supported values are 1,2 and 4 */
#define KMB_MVMX_REG_MIPI_NUM_LANES		0x0000

/* MIPI Lane Speed in Mbps. Supported values are 100 to 1900MHz */
#define KMB_MVMX_REG_MIPI_LANE_SPEED		0x0002

/* VC to use for the MIPI transmission */
#define KMB_MVMX_REG_MIPI_VC			0x0004

/* Output data type. Supports 0x18, 0x19, 0x1A, 0x1C, 0x1D, 0x1E, 0x1F */
#define KMB_MVMX_REG_MIPI_DATA_TYPE		0x0006

/* Need to set this with 1 in order to work */
#define KMB_MVMX_REG_MIPI_TX_DMA_PACKET		0x0012

/* Writing 1 will start sensor stream, 0 will stop it */
#define KMB_MVMX_REG_START_STOP			0x0100
/* SW reset of all sensor registers */
#define KMB_MVMX_REG_RESET			0x0102

/* Read Only, should return 0xCECA when read */
#define KMB_MVMX_REG_DEVICE_ID			0x0300
#define KMB_MVMX_MVMX_ID			0xCECA

/* Output image horizontal size */
#define KMB_MVMX_REG_X_OUTPUT_SIZE		0x3808
/* Output image vertical size */
#define KMB_MVMX_REG_Y_OUTPUT_SIZE		0x380A
/*
 * Transmission line length - should be > X_OUTPUT_SIZE
 * used to configure horizontal blanking
 */
#define KMB_MVMX_REG_LINE_LENGTH_PCK		0x380C
/*
 * Transmission vertical size in lines - should be > Y_OUTPUT_SIZE
 * used to configure vertical blanking
 */
#define KMB_MVMX_REG_FRAME_LENGTH_LINES		0x380E

/*
 * Supported values are 0, 1, 2 and 3. Default is 0
 * 0 - Repeating pattern where each pixel has a value of 0 to max_val
 *     max_val depends on data type bit depth. Once max val is reached, the
 *     pattern repeats. Does not care about bayer pattern or YUV data layout.
 * 1 - Generates a pattern based on host seed. The pattern is a pure byte
 *     stream, where the first byte is the seed used for generation for
 *     easier verification on host side.
 * 2 - Same as 1, but uses current system timestamp as seed value.
 * 3 - Color bars pattern
 */
#define KMB_MVMX_REG_DATA_PATTERN_MODE		0x4000

/*
 * Updates the buffer for pattern generation. Meaningful to
 * use when DATA_PATTERN_MODE is 1 or 2
 */
#define KMB_MVMX_REG_DATA_PATTERN_UPDATE	0x4002
/* Provides the seed value to use when DATA_PATTERN_MODE = 1 */
#define KMB_MVMX_REG_DATA_PATTERN_SEED		0x4004

#define KMB_MVMX_MAX_SENSOR_WIDHT		(4000)
#define KMB_MVMX_MAX_SENSOR_HEIGHT		(3000)
#define KMB_MVMX_SENSOR_BLANKING		(300)
#define KMB_MVMX_DRV_NAME			"kmb-mvmx-sensor"
#define KMB_MVMX_NUM_PADS			1
#define KMB_MVMX_NUM_LANES			4
#define KMB_MVMX_FPS_MIN			(15)
#define KMB_MVMX_FPS_MAX			(60)

/**
 * struct kmb_mvmx - KMB Smart Sensor device structure
 * @dev: pointer to generic device
 * @client: pointer to i2c client device
 * @subdev: V4L2 sub-device
 * @pad: Media pad. Only one pad supported
 * @formats: array of frame formats on the media bus
 * @curr_fmt: current frame format
 * @ctrl_handler: V4L2 control handler
 * @exposure: Pointer to exposure control
 * @gain: Pointer to  gain control
 * @brightness: brightness
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
 * @pclk: Pointer to pixel clock control
 * @link_freq: Pointer to link frequency control
 * @test_pattern: Pointer to test pattern control
 * @mutex: Mutex for serializing sensor controls
 * @streaming: streaming state flag
 * @power_on: power on flag
 * @pattern: test pattern to write in KMB_MVMX_REG_DATA_PATTERN_MODE
 * @fps: FPS to be applied on next stream on
 */
struct kmb_mvmx {
	struct device *dev;
	struct i2c_client *client;

	struct v4l2_subdev subdev;
	struct media_pad pad;
	struct v4l2_mbus_framefmt formats[KMB_MVMX_NUM_PADS];
	struct v4l2_subdev_format curr_fmt;
	struct v4l2_ctrl_handler ctrl_handler;
	struct {
		struct v4l2_ctrl *exposure;
		struct v4l2_ctrl *gain;
	};
	struct v4l2_ctrl *brightness;
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
	struct v4l2_ctrl *pclk;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *test_pattern;
	struct mutex mutex;
	bool streaming;
	bool power_on;
	enum kmb_camera_test_pattern_type pattern;
	u32 fps;
};

/**
 * struct kmb_mvmx_mbus_code_bpp - Media bus code bpp and data type map
 * @code: Format code
 * @bpp: bits per pixel for format code
 * @data_type: IcMipiRxDataTypeT
 */
struct kmb_mvmx_mbus_code_bpp {
	u32 code;
	u32 bpp;
	u32 data_type;
};

static const s64 link_freq[] = {
	100000000,
	125000000,
	150000000,
	175000000,
	200000000,
	225000000,
	250000000,
	275000000,
	300000000,
	325000000,
	350000000,
	375000000,
	400000000,
	425000000,
	450000000,
	475000000,
	500000000,
	525000000,
	550000000,
	575000000,
	600000000,
	625000000,
	650000000,
	675000000,
	700000000,
	725000000,
	750000000,
	775000000,
	800000000,
	825000000,
	850000000,
	875000000,
	900000000,
	925000000,
	950000000,
	975000000,
	1000000000,
	1025000000,
	1050000000,
	1075000000,
	1100000000,
	1125000000,
	1150000000,
	1175000000,
	1200000000,
	1225000000,
	1250000000,
	1275000000,
	1300000000,
	1325000000,
	1350000000,
	1375000000,
	1400000000,
	1425000000,
	1450000000,
	1475000000,
	1500000000,
};

static const char * const test_patterns[] = {
	"repeating",
	"repeating w/ 0xCECA seed",
	"repeating w/ ts seed",
	"color bars",
};

static const struct kmb_mvmx_mbus_code_bpp supported_formats[] = {
	{
		.code = MEDIA_BUS_FMT_VUY8_1X24,
		.bpp = 12,
		.data_type = 0x18,
	},
	{
		.code = MEDIA_BUS_FMT_YUV10_1X30,
		.bpp = 24,
		.data_type = 0x19,
	},
	{
		.code = MEDIA_BUS_FMT_YVYU10_1X20,
		.bpp = 15,
		.data_type = 0x19,
	},
	{
		.code = MEDIA_BUS_FMT_YVYU8_2X8,
		.bpp = 12,
		.data_type = 0x1A,
	},
	{
		.code = MEDIA_BUS_FMT_YUYV8_2X8,
		.bpp = 12,
		.data_type = 0x1C,
	},
	{
		.code = MEDIA_BUS_FMT_UYVY10_1X20,
		.bpp = 24,
		.data_type = 0x1D,
	},
	{
		.code = MEDIA_BUS_FMT_VYUY10_1X20,
		.bpp = 15,
		.data_type = 0x1D,
	},
	{
		.code = MEDIA_BUS_FMT_YVYU8_1X16,
		.bpp = 16,
		.data_type = 0x1E,
	},
	{
		.code = MEDIA_BUS_FMT_VYUY10_2X10,
		.bpp = 32,
		.data_type = 0x1F,
	},
	{
		.code = MEDIA_BUS_FMT_YUYV10_2X10,
		.bpp = 20,
		.data_type = 0x1F,
	},
};

/**
 * kmb_mvmx_read_reg - Read register.
 * @kmb_mvmx: pointer to mvmx device
 * @reg: Register address
 * @len: Length of bytes to read. Max supported bytes is 4
 * @val: Pointer to register value to be filled.
 *
 * Return: 0 if successful
 */
static int
kmb_mvmx_read_reg(struct kmb_mvmx *kmb_mvmx, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&kmb_mvmx->subdev);
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
 * kmb_mvmx_write_reg - Write register
 * @kmb_mvmx: pointer to mvmx device
 * @reg: Register address
 * @len: Length of bytes. Max supported bytes is 4
 * @val: Register value
 *
 * Return: 0 if successful
 */
static int
kmb_mvmx_write_reg(struct kmb_mvmx *kmb_mvmx, u16 reg, u32 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&kmb_mvmx->subdev);
	int buf_i, val_i;
	u8 buf[6], *val_p;
	__be32 val_be;
	int ret;

	if (len > 4) {
		dev_err_ratelimited(kmb_mvmx->dev,
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
		dev_err_ratelimited(kmb_mvmx->dev,
				    "write reg 0x%4.4x return err %d",
				    reg, ret);
		return -EIO;
	}

	return 0;
}

/**
 * kmb_mvmx_enum_mbus_code - Enum mbus format code
 * @sd: pointer to V4L2 sub-device
 * @cfg: pointer to sub-device pad information
 * @code: pointer to media bus format code enumeration
 *
 * VIDIOC_SUBDEV_ENUM_MBUS_CODE ioctl handler
 *
 * Return: 0 if successful
 */
static int kmb_mvmx_enum_mbus_code(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *cfg,
				   struct v4l2_subdev_mbus_code_enum *code)
{
	struct kmb_mvmx *kmb_mvmx = container_of(sd, struct kmb_mvmx, subdev);

	dev_dbg(sd->dev, "%s\n", __func__);

	if (code->index > ARRAY_SIZE(supported_formats) - 1)
		return -EINVAL;

	mutex_lock(&kmb_mvmx->mutex);
	code->code = supported_formats[code->index].code;
	mutex_unlock(&kmb_mvmx->mutex);

	return 0;
}

/**
 * kmb_mvmx_enum_frame_sizes - Enum frame sizes
 * @sd: pointer to V4L2 sub-device
 * @cfg: pointer to sub-device pad information
 * @fse: pointer to media bus format enumeration
 *
 * VIDIOC_SUBDEV_ENUM_FRAME_SIZE ioctl handler
 *
 * Return: 0 if successful
 */
static int kmb_mvmx_enum_frame_sizes(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *cfg,
				     struct v4l2_subdev_frame_size_enum *fse)
{
	struct kmb_mvmx *kmb_mvmx = container_of(sd, struct kmb_mvmx, subdev);

	mutex_lock(&kmb_mvmx->mutex);

	if (fse->index > 0) {
		mutex_unlock(&kmb_mvmx->mutex);
		return -EINVAL;
	}

	fse->min_width = KMB_MVMX_MAX_SENSOR_WIDHT;
	fse->max_width = fse->min_width;
	fse->min_height = KMB_MVMX_MAX_SENSOR_HEIGHT;
	fse->max_height = fse->min_height;

	mutex_unlock(&kmb_mvmx->mutex);

	return 0;
}

/**
 * kmb_mvmx_get_fmt - Get mbus format
 * @sd: pointer to V4L2 sub-device
 * @cfg: pointer to sub-device pad information
 * @fmt: pointer to pad-level media bus format
 *
 * VIDIOC_SUBDEV_G_FMT ioctl handler
 *
 * Return: 0 if successful
 */
static int kmb_mvmx_get_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *cfg,
			    struct v4l2_subdev_format *fmt)
{
	struct kmb_mvmx *kmb_mvmx = container_of(sd, struct kmb_mvmx, subdev);

	dev_dbg(sd->dev, "%s\n", __func__);

	mutex_lock(&kmb_mvmx->mutex);
	fmt->format = kmb_mvmx->curr_fmt.format;
	mutex_unlock(&kmb_mvmx->mutex);

	return 0;
}

/**
 * kmb_mvmx_get_bpp_by_code - Get bpp for mbus format
 * @code: media bus format code
 *
 * Return: return bpp corresponding to code or 0
 */
static u32 kmb_mvmx_get_bpp_by_code(u32 code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(supported_formats); i++) {
		if (supported_formats[i].code == code)
			return supported_formats[i].bpp;
	}
	return 0;
}

/**
 * kmb_mvmx_get_frame_interval - Get subdevice frame interval
 * @sd: pointer to V4L2 sub-device structure
 * @interval: V4L2 sub-device current farme interval
 *
 * Return: 0 if successful
 */
static int kmb_mvmx_get_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_frame_interval *ival)
{
	struct kmb_mvmx *kmb_mvmx = container_of(sd, struct kmb_mvmx, subdev);

	if (ival->pad)
		return -EINVAL;

	mutex_lock(&kmb_mvmx->mutex);
	ival->interval.numerator = 1;
	ival->interval.denominator = kmb_mvmx->fps;
	mutex_unlock(&kmb_mvmx->mutex);

	dev_dbg(kmb_mvmx->dev, "Get frame interval %d/%d",
		ival->interval.numerator, ival->interval.denominator);

	return 0;
}

/**
 * kmb_mvmx_set_frame_interval - Set subdevice frame interval
 * @sd: pointer to V4L2 sub-device structure
 * @interval: V4L2 sub-device farme interval to be set
 *
 * Return: 0 if successful
 */
static int kmb_mvmx_set_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_frame_interval *ival)
{
	struct kmb_mvmx *kmb_mvmx = container_of(sd, struct kmb_mvmx, subdev);
	u32 fps;

	if (ival->pad)
		return -EINVAL;

	mutex_lock(&kmb_mvmx->mutex);
	fps = (u32)(ival->interval.denominator / ival->interval.numerator);

	if (fps < KMB_MVMX_FPS_MIN) {
		ival->interval.numerator = 1;
		ival->interval.denominator = KMB_MVMX_FPS_MIN;
	} else if (fps > KMB_MVMX_FPS_MAX) {
		ival->interval.numerator = 1;
		ival->interval.denominator = KMB_MVMX_FPS_MAX;
	}

	kmb_mvmx->fps = (u32)(ival->interval.denominator /
			       ival->interval.numerator);

	mutex_unlock(&kmb_mvmx->mutex);

	dev_dbg(kmb_mvmx->dev, "Set frame interval %d/%d",
		ival->interval.numerator, ival->interval.denominator);

	return 0;
}

/**
 * kmb_mvmx_get_lane_rate_mbps - Calc lane rate in mbps based on pclk and bpp.
 *                               Clip result to min/max supported lane speed
 * @pix_rate: pixel clock for selected fps
 * @bpp: bit depth of format
 * @idx: idx ot lane rate in link_freq array
 *
 * Return: Closest lane rate from link_freq array
 */
static u64 kmb_mvmx_get_lane_rate_mbps(const u32 pix_rate, const u32 bpp,
				       int *idx)
{
	u64 lane_rate_mbps, nearest_lane_rate;

	lane_rate_mbps = pix_rate * bpp * 2;
	do_div(lane_rate_mbps, KMB_MVMX_NUM_LANES);

	*idx = find_closest(lane_rate_mbps, link_freq, ARRAY_SIZE(link_freq));

	nearest_lane_rate = link_freq[*idx];

	return nearest_lane_rate;
}

/**
 * kmb_mvmx_set_fmt - Set mbus format
 * @sd: pointer to V4L2 sub-device
 * @cfg: pointer to sub-device pad information
 * @fmt: pointer to pad-level media bus format
 *
 * VIDIOC_SUBDEV_S_FMT ioctl handler
 *
 * Return: 0 if successful
 */
static int kmb_mvmx_set_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *cfg,
			    struct v4l2_subdev_format *fmt)
{
	struct kmb_mvmx *kmb_mvmx = container_of(sd, struct kmb_mvmx, subdev);
	u64 pix_rate, tmp, lane_rate_mbps;
	int link_freq_idx, ret = 0;
	u32 lane_speed_mhz, bpp, w, h, lpfr, ppln;

	dev_dbg(sd->dev, "%s\n", __func__);

	mutex_lock(&kmb_mvmx->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		cfg->pads->try_fmt = fmt->format;
	} else {
		kmb_mvmx->curr_fmt = *fmt;
		w = fmt->format.width;
		h = fmt->format.height;
		ppln = w + KMB_MVMX_SENSOR_BLANKING;
		lpfr = h + KMB_MVMX_SENSOR_BLANKING;

		if (w > KMB_MVMX_MAX_SENSOR_WIDHT ||
		    h > KMB_MVMX_MAX_SENSOR_HEIGHT) {
			dev_err(sd->dev, "Output resolution %ux%u is too big!"
				" Max supported res is %dx%d\n", w, h,
				KMB_MVMX_MAX_SENSOR_WIDHT,
				KMB_MVMX_MAX_SENSOR_HEIGHT);
			ret = -EINVAL;
			goto exit;
		}


		bpp = kmb_mvmx_get_bpp_by_code(fmt->format.code);
		if (!bpp) {
			ret = -EINVAL;
			goto exit_unlock;
		}

		/*
		 * Calc pclk according to desired framerate now that we know
		 * actual w and h
		 */
		pix_rate = ppln * lpfr * kmb_mvmx->fps;
		lane_rate_mbps = kmb_mvmx_get_lane_rate_mbps(pix_rate, bpp,
							     &link_freq_idx);
		/* lane rate could have been aligned so re-calc pclk */
		tmp = lane_rate_mbps * KMB_MVMX_NUM_LANES;
		do_div(tmp, bpp);
		pix_rate = tmp;

		mutex_unlock(&kmb_mvmx->mutex);

		ret = v4l2_ctrl_s_ctrl_int64(kmb_mvmx->pclk, pix_rate);
		if (ret)
			goto exit;

		ret = v4l2_ctrl_s_ctrl(kmb_mvmx->link_freq, link_freq_idx);
		if (ret)
			goto exit;

		mutex_lock(&kmb_mvmx->mutex);

		lane_speed_mhz = lane_rate_mbps;
		lane_speed_mhz /= 1000000;

		ret = kmb_mvmx_write_reg(kmb_mvmx,
					 KMB_MVMX_REG_MIPI_LANE_SPEED,
					 2, lane_speed_mhz);
		if (ret)
			goto exit_unlock;

		ret = kmb_mvmx_write_reg(kmb_mvmx, KMB_MVMX_REG_MIPI_NUM_LANES,
					 2, KMB_MVMX_NUM_LANES);
		if (ret)
			goto exit_unlock;

		ret = kmb_mvmx_write_reg(kmb_mvmx, KMB_MVMX_REG_X_OUTPUT_SIZE,
					 2, w);
		if (ret)
			goto exit_unlock;

		ret = kmb_mvmx_write_reg(kmb_mvmx, KMB_MVMX_REG_Y_OUTPUT_SIZE,
					 2, h);
		if (ret)
			goto exit_unlock;

		ret = kmb_mvmx_write_reg(kmb_mvmx,
					 KMB_MVMX_REG_LINE_LENGTH_PCK,
					 2, ppln);
		if (ret)
			goto exit_unlock;

		ret = kmb_mvmx_write_reg(kmb_mvmx,
					 KMB_MVMX_REG_FRAME_LENGTH_LINES,
					 2, lpfr);
		if (ret)
			goto exit_unlock;

		/* VC 0 to be used by verbal agreement */
		ret = kmb_mvmx_write_reg(kmb_mvmx, KMB_MVMX_REG_MIPI_VC, 2, 0);
		if (ret)
			goto exit_unlock;
	}

exit_unlock:
	mutex_unlock(&kmb_mvmx->mutex);
exit:
	return ret;
}

/**
 * kmb_mvmx_get_data_type_by_code - Get data type for mbus format
 * @code: media bus format code
 *
 * Return: return data type corresponding to code or 0
 */
static u32 kmb_mvmx_get_data_type_by_code(u32 code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(supported_formats); i++) {
		if (supported_formats[i].code == code)
			return supported_formats[i].data_type;
	}
	return 0;
}

/**
 * kmb_mvmx_s_stream - Set video stream stop/start
 * @sd: pointer to V4L2 sub-device
 * @enable: stream state
 *
 * Return: 0 if successful
 */
static int kmb_mvmx_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct kmb_mvmx *mvmx =
			container_of(sd, struct kmb_mvmx, subdev);
	u32 data_type;
	struct v4l2_mbus_framefmt *mbus_fmt = &mvmx->curr_fmt.format;
	enum kmb_camera_test_pattern_type pattern = mvmx->pattern;
	int ret = 0;

	dev_dbg(mvmx->subdev.dev, "%s: Set stream on %d\n",
		__func__, enable);

	mutex_lock(&mvmx->mutex);

	if (enable) {
		data_type = kmb_mvmx_get_data_type_by_code(mbus_fmt->code);
		if (!data_type) {
			dev_err(sd->dev, "%s Unknown mbus format %x",
				__func__, mbus_fmt->code);
			ret = -EINVAL;
			goto unlock_mutex;
		}

		ret = kmb_mvmx_write_reg(mvmx, KMB_MVMX_REG_MIPI_TX_DMA_PACKET,
					 2, 1);
		if (ret)
			goto unlock_mutex;

		ret = kmb_mvmx_write_reg(mvmx, KMB_MVMX_REG_MIPI_DATA_TYPE,
					 2, data_type);
		if (ret)
			goto unlock_mutex;

		ret = kmb_mvmx_write_reg(mvmx, KMB_MVMX_REG_DATA_PATTERN_MODE,
					 2, pattern);
		if (ret)
			goto unlock_mutex;

		if (pattern == KMB_CAMERA_TEST_PATTERN_REPEATING_SEED) {
			ret =
			kmb_mvmx_write_reg(mvmx,
					   KMB_MVMX_REG_DATA_PATTERN_SEED,
					   2, 0xCECA);
			if (ret)
				goto unlock_mutex;
		}

		if (pattern == KMB_CAMERA_TEST_PATTERN_REPEATING_SEED ||
		    pattern == KMB_CAMERA_TEST_PATTERN_REPEATING_SEED_TS) {
			ret =
			kmb_mvmx_write_reg(mvmx,
					   KMB_MVMX_REG_DATA_PATTERN_UPDATE,
					   2, 1);
			if (ret)
				goto unlock_mutex;
		}

		ret = kmb_mvmx_write_reg(mvmx, KMB_MVMX_REG_START_STOP, 2, 1);
		if (ret)
			goto unlock_mutex;
	} else {
		ret = kmb_mvmx_write_reg(mvmx, KMB_MVMX_REG_START_STOP, 2, 0);
		if (ret)
			goto unlock_mutex;

		ret = kmb_mvmx_write_reg(mvmx, KMB_MVMX_REG_RESET, 2, 1);
		if (ret)
			goto unlock_mutex;
	}

	mvmx->streaming = enable ? true : false;

unlock_mutex:
	mutex_unlock(&mvmx->mutex);
	return ret;
}

/**
 * kmb_mvmx_get_default_format - Get default media bus format
 * @mbus_fmt: pointer to media bus format
 *
 * Return: none
 */
static void kmb_mvmx_get_default_format(struct v4l2_mbus_framefmt *mbus_fmt)
{
	mbus_fmt->width = KMB_MVMX_MAX_SENSOR_WIDHT;
	mbus_fmt->height = KMB_MVMX_MAX_SENSOR_HEIGHT;
	mbus_fmt->colorspace = V4L2_COLORSPACE_SRGB;
	mbus_fmt->code = MEDIA_BUS_FMT_VUY8_1X24;
	mbus_fmt->field = V4L2_FIELD_NONE;
}

/**
 * kmb_mvmx_detect - Detect myriad X
 * @kmb_mvmx: pointer to kmb_mvmx device
 *
 * Return: 0 if successful, -EIO if sensor id does not match
 */
static int kmb_mvmx_detect(struct kmb_mvmx *kmb_mvmx)
{
	int ret;
	u32 val = 0;

	ret = kmb_mvmx_read_reg(kmb_mvmx, KMB_MVMX_REG_DEVICE_ID, 2, &val);
	if (ret)
		return ret;

	if (val != KMB_MVMX_MVMX_ID) {
		dev_err(kmb_mvmx->dev, "chip id mismatch: %x!=%x",
			KMB_MVMX_MVMX_ID, val);
		return -EIO;
	}
	return 0;
}

/**
 * kmb_mvmx_s_power - Put device in power saving or normal mode
 * @sd: pointer to V4L2 sub-device
 * @on: power state (0 - power saving mode, 1 - normal operation mode)
 *
 * Return: 0 if successful
 */
static int kmb_mvmx_s_power(struct v4l2_subdev *sd, int on)
{
	struct kmb_mvmx *kmb_mvmx =
			container_of(sd, struct kmb_mvmx, subdev);

	mutex_lock(&kmb_mvmx->mutex);
	if (kmb_mvmx->power_on != on) {
		dev_dbg(sd->dev, "%s: Set power_on %d\n", __func__, on);
		kmb_mvmx->power_on = on;
	}
	kmb_mvmx->streaming = false;
	mutex_unlock(&kmb_mvmx->mutex);

	dev_dbg(sd->dev, "%s: Set power_on %d\n", __func__, on);
	return 0;
}

/**
 * kmb_mvmx_s_ctrl - Set new control value
 * @ctrl: pointer to control structure
 *
 * Return: 0 if successful
 */
static int kmb_mvmx_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd =
			&(container_of(ctrl->handler, struct kmb_mvmx,
					ctrl_handler)->subdev);
	struct kmb_mvmx *kmb_mvmx =
			container_of(sd, struct kmb_mvmx, subdev);
	int ret = 0;

	dev_dbg(sd->dev, "s_ctrl: %s, value: %d. power: %d\n",
		ctrl->name, ctrl->val, kmb_mvmx->power_on);

	mutex_lock(&kmb_mvmx->mutex);

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE_ABSOLUTE:
		/* Handle the cluster for both controls */
		dev_dbg(kmb_mvmx->dev, "%s: set exposure %d gain %d",
			__func__, kmb_mvmx->exposure->val,
			kmb_mvmx->gain->val);
		break;
	case V4L2_CID_BRIGHTNESS:
		dev_dbg(kmb_mvmx->dev, "%s: set brightness %d",
			__func__, ctrl->val);
		break;
	case V4L2_CID_CONTRAST:
		dev_dbg(kmb_mvmx->dev, "%s: set contrast %d",
			__func__, ctrl->val);
		break;
	case V4L2_CID_SATURATION:
		dev_dbg(kmb_mvmx->dev, "%s: set saturation %d",
			__func__, ctrl->val);
		break;
	case V4L2_CID_HUE:
		dev_dbg(kmb_mvmx->dev, "%s: set hue %d", __func__, ctrl->val);
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		dev_dbg(kmb_mvmx->dev, "%s: set awb %d", __func__, ctrl->val);
		break;
	case V4L2_CID_RED_BALANCE:
		dev_dbg(kmb_mvmx->dev, "%s: set red balance %d",
			__func__, ctrl->val);
		break;
	case V4L2_CID_BLUE_BALANCE:
		dev_dbg(kmb_mvmx->dev, "%s: set blue balance %d",
			__func__, ctrl->val);
		break;
	case V4L2_CID_GAMMA:
		dev_dbg(kmb_mvmx->dev, "%s: set gamma %d",
			__func__, ctrl->val);
		break;
	case V4L2_CID_AUTOGAIN:
		dev_dbg(kmb_mvmx->dev, "%s: set auto gain %d",
			__func__, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		dev_dbg(kmb_mvmx->dev, "%s: set horizontal flip %d",
			__func__, ctrl->val);
		break;
	case V4L2_CID_VFLIP:
		dev_dbg(kmb_mvmx->dev, "%s: set vertical flip %d",
			__func__, ctrl->val);
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY:
		dev_dbg(kmb_mvmx->dev, "%s: set power frequency %d",
			__func__, ctrl->val);
		break;
	case V4L2_CID_SHARPNESS:
		dev_dbg(kmb_mvmx->dev, "%s: set sharpness %d",
			__func__, ctrl->val);
		break;
	case V4L2_CID_BAND_STOP_FILTER:
		dev_dbg(kmb_mvmx->dev, "%s: set antibanding %d",
			__func__, ctrl->val);
		break;
	case V4L2_CID_PIXEL_RATE:
		dev_dbg(kmb_mvmx->dev, "%s: set pclk %d",
			__func__, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		kmb_mvmx->pattern = ctrl->val;
		break;
	case V4L2_CID_LINK_FREQ:
		break;
	default:
		dev_dbg(kmb_mvmx->dev, "Invalid control %d", ctrl->id);
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&kmb_mvmx->mutex);
	return ret;
}

static const struct media_entity_operations kmb_mvmx_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/* sub-device core operations */
static const struct v4l2_subdev_core_ops kmb_mvmx_core_ops = {
	.s_power = kmb_mvmx_s_power,
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

/* sub-device pad operations */
static const struct v4l2_subdev_pad_ops kmb_mvmx_pad_ops = {
	.enum_mbus_code = kmb_mvmx_enum_mbus_code,
	.enum_frame_size = kmb_mvmx_enum_frame_sizes,
	.get_fmt = kmb_mvmx_get_fmt,
	.set_fmt = kmb_mvmx_set_fmt,
};

/* sub-device video operations */
static const struct v4l2_subdev_video_ops kmb_mvmx_video_ops = {
	.s_stream = kmb_mvmx_s_stream,
	.s_frame_interval = kmb_mvmx_set_frame_interval,
	.g_frame_interval = kmb_mvmx_get_frame_interval,
};

/* sub-device operations */
static const struct v4l2_subdev_ops kmb_mvmx_subdev_ops = {
	.core = &kmb_mvmx_core_ops,
	.pad = &kmb_mvmx_pad_ops,
	.video = &kmb_mvmx_video_ops,
};

/* V4L2 control operations */
static const struct v4l2_ctrl_ops kmb_mvmx_ctrl_ops = {
	.s_ctrl = kmb_mvmx_s_ctrl,
};

/**
 * kmb_mvmx_initialize_controls - Initialize handled sensor controls
 * @kmb_mvmx: pointer to sensor device
 *
 * Return: 0 if successful
 */
static int kmb_mvmx_initialize_controls(struct kmb_mvmx *kmb_mvmx)
{
	const struct v4l2_ctrl_ops *ops = &kmb_mvmx_ctrl_ops;
	u64 pixrate_min, pixrate_max;
	u64 lane_freq_max = link_freq[ARRAY_SIZE(link_freq) - 1];
	u64 lane_freq_min = link_freq[0];
	const u32 bpp_min = 10, bpp_max = 32;
	int ret;

	ret = v4l2_ctrl_handler_init(&kmb_mvmx->ctrl_handler, 32);
	if (ret < 0)
		return ret;

	kmb_mvmx->exposure = v4l2_ctrl_new_std(&kmb_mvmx->ctrl_handler, ops,
					       V4L2_CID_EXPOSURE_ABSOLUTE,
					       2, 1500, 1, 500);
	kmb_mvmx->gain = v4l2_ctrl_new_std(&kmb_mvmx->ctrl_handler, ops,
					   V4L2_CID_GAIN,
					   16, 64 * (16 + 15), 1, 64 * 16);
	kmb_mvmx->brightness = v4l2_ctrl_new_std(&kmb_mvmx->ctrl_handler, ops,
						 V4L2_CID_BRIGHTNESS,
						 -3, 3, 1, 0);
	kmb_mvmx->contrast = v4l2_ctrl_new_std(&kmb_mvmx->ctrl_handler, ops,
					       V4L2_CID_CONTRAST,
					       0, 127, 1, 0x20);
	kmb_mvmx->saturation = v4l2_ctrl_new_std(&kmb_mvmx->ctrl_handler, ops,
						 V4L2_CID_SATURATION,
						 0, 256, 1, 0x80);
	kmb_mvmx->hue = v4l2_ctrl_new_std(&kmb_mvmx->ctrl_handler, ops,
					  V4L2_CID_HUE,
					  0, 0x1f, 1, 0x10);
	kmb_mvmx->awb = v4l2_ctrl_new_std(&kmb_mvmx->ctrl_handler, ops,
					  V4L2_CID_AUTO_WHITE_BALANCE,
					  0, 1, 1, 1);
	kmb_mvmx->red_bal = v4l2_ctrl_new_std(&kmb_mvmx->ctrl_handler, ops,
					      V4L2_CID_RED_BALANCE,
					      0, 0xff, 1, 0x80);
	kmb_mvmx->blue_bal = v4l2_ctrl_new_std(&kmb_mvmx->ctrl_handler, ops,
					       V4L2_CID_BLUE_BALANCE,
					       0, 0xff, 1, 0x80);
	kmb_mvmx->gamma = v4l2_ctrl_new_std(&kmb_mvmx->ctrl_handler, ops,
					    V4L2_CID_GAMMA,
					    0, 0xff, 1, 0x12);
	kmb_mvmx->auto_gain = v4l2_ctrl_new_std(&kmb_mvmx->ctrl_handler, ops,
						V4L2_CID_AUTOGAIN,
						0, 1, 1, 1);
	kmb_mvmx->h_flip = v4l2_ctrl_new_std(&kmb_mvmx->ctrl_handler, ops,
					     V4L2_CID_HFLIP,
					     0, 1, 1, 0);
	kmb_mvmx->v_flip = v4l2_ctrl_new_std(&kmb_mvmx->ctrl_handler, ops,
					     V4L2_CID_VFLIP,
					     0, 1, 1, 0);
	kmb_mvmx->pwr_freq =
		v4l2_ctrl_new_std_menu(&kmb_mvmx->ctrl_handler, ops,
				       V4L2_CID_POWER_LINE_FREQUENCY,
				       V4L2_CID_POWER_LINE_FREQUENCY_60HZ,
				       ~0x7,
				       V4L2_CID_POWER_LINE_FREQUENCY_50HZ);
	kmb_mvmx->sharpness = v4l2_ctrl_new_std(&kmb_mvmx->ctrl_handler, ops,
						V4L2_CID_SHARPNESS,
						0, 32, 1, 6);
	kmb_mvmx->antibanding = v4l2_ctrl_new_std(&kmb_mvmx->ctrl_handler, ops,
						  V4L2_CID_BAND_STOP_FILTER,
						  0, 256, 1, 0);
	kmb_mvmx->test_pattern =
		v4l2_ctrl_new_std_menu_items(&kmb_mvmx->ctrl_handler,
					     ops, V4L2_CID_TEST_PATTERN,
					     ARRAY_SIZE(test_patterns) - 1, 0,
					     3, test_patterns);

	v4l2_ctrl_cluster(2, &kmb_mvmx->exposure);

	do_div(lane_freq_min, bpp_max);
	pixrate_min = lane_freq_min * KMB_MVMX_NUM_LANES;
	do_div(lane_freq_max, bpp_min);
	pixrate_max = lane_freq_max * KMB_MVMX_NUM_LANES;

	kmb_mvmx->pclk = v4l2_ctrl_new_std(&kmb_mvmx->ctrl_handler, ops,
					   V4L2_CID_PIXEL_RATE,
					   pixrate_min, pixrate_max,
					   1, pixrate_max);

	kmb_mvmx->link_freq = v4l2_ctrl_new_int_menu(&kmb_mvmx->ctrl_handler,
						     ops, V4L2_CID_LINK_FREQ,
						     ARRAY_SIZE(link_freq) - 1,
						     0, link_freq);
	if (kmb_mvmx->ctrl_handler.error) {
		ret = kmb_mvmx->ctrl_handler.error;
		v4l2_ctrl_handler_free(&kmb_mvmx->ctrl_handler);
		return ret;
	}

	kmb_mvmx->subdev.ctrl_handler = &kmb_mvmx->ctrl_handler;
	return ret;
}

/**
 * kmb_mvmx_platform_resume - PM resume callback
 * @dev: platform device
 *
 * Return: 0 if successful
 */
static int kmb_mvmx_platform_resume(struct device *dev)
{
	return 0;
}

/**
 * kmb_mvmx_platform_suspend - PM suspend callback
 * @dev: platform device
 *
 * Return: 0 if successful
 */
static int kmb_mvmx_platform_suspend(struct device *dev)
{
	return 0;
}

/**
 * kmb_mvmx_get_i2c_client - Get I2C client
 * @kmb_mvmx: pointer to kmb_mvmx device
 *
 * Return: 0 if successful
 */
static int kmb_mvmx_get_i2c_client(struct kmb_mvmx *kmb_mvmx)
{
	struct i2c_board_info info = {
		.type = "kmb-mvmx-sensor-p" };
	const unsigned short addr_list[] = {0x66, I2C_CLIENT_END};
	struct i2c_adapter *i2c_adp;
	struct device_node *phandle;

	phandle = of_parse_phandle(kmb_mvmx->dev->of_node, "i2c-bus", 0);
	if (!phandle)
		return -ENODEV;

	i2c_adp = of_get_i2c_adapter_by_node(phandle);
	of_node_put(phandle);
	if (!i2c_adp)
		return -EPROBE_DEFER;

	kmb_mvmx->client = i2c_new_scanned_device(i2c_adp, &info, addr_list,
						  NULL);
	i2c_put_adapter(i2c_adp);

	return PTR_ERR_OR_ZERO(kmb_mvmx->client);
}

/**
 * kmb_mvmx_pdev_probe - Platform device binding
 * @pdev: pointer to platform device
 *
 * Return: 0 if successful
 */
static int kmb_mvmx_pdev_probe(struct platform_device *pdev)
{
	struct kmb_mvmx *kmb_mvmx;
	int ret;

	kmb_mvmx = devm_kzalloc(&pdev->dev, sizeof(*kmb_mvmx), GFP_KERNEL);
	if (!kmb_mvmx)
		return -ENOMEM;

	platform_set_drvdata(pdev, kmb_mvmx);

	mutex_init(&kmb_mvmx->mutex);

	kmb_mvmx->dev = &pdev->dev;

	/* Initialize subdev */
	v4l2_subdev_init(&kmb_mvmx->subdev, &kmb_mvmx_subdev_ops);
	kmb_mvmx->subdev.owner = pdev->dev.driver->owner;
	kmb_mvmx->subdev.dev = &pdev->dev;

	ret = kmb_mvmx_get_i2c_client(kmb_mvmx);
	if (ret) {
		dev_err(&pdev->dev, "failed to get i2c");
		goto error_mutex_destroy;
	}

	v4l2_set_subdevdata(&kmb_mvmx->subdev, kmb_mvmx->client);
	i2c_set_clientdata(kmb_mvmx->client, &kmb_mvmx->subdev);
	v4l2_i2c_subdev_set_name(&kmb_mvmx->subdev, kmb_mvmx->client,
				 KMB_MVMX_DRV_NAME, pdev->name);

	/* Check module identity */
	ret = kmb_mvmx_detect(kmb_mvmx);
	if (ret) {
		dev_err(&pdev->dev, "failed to find sensor: %d", ret);
		goto error_unregister_i2c_dev;
	}

	/* Set default mode to max resolution */
	kmb_mvmx_get_default_format(kmb_mvmx->formats);
	kmb_mvmx->curr_fmt.format = kmb_mvmx->formats[0];
	kmb_mvmx->pattern = KMB_CAMERA_TEST_PATTERN_COLOR_BARS;
	kmb_mvmx->fps = KMB_MVMX_FPS_MIN * 2;

	ret = kmb_mvmx_initialize_controls(kmb_mvmx);
	if (ret) {
		dev_err(&pdev->dev, "failed to init controls: %d", ret);
		goto error_unregister_i2c_dev;
	}

	/* Initialize subdev */
	kmb_mvmx->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	kmb_mvmx->subdev.entity.ops = &kmb_mvmx_subdev_entity_ops;
	kmb_mvmx->subdev.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
	kmb_mvmx->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&kmb_mvmx->subdev.entity, 1,
				     &kmb_mvmx->pad);
	if (ret) {
		dev_err(&pdev->dev, "failed to init entity pads: %d", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&kmb_mvmx->subdev);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"failed to register async subdev: %d", ret);
		goto error_media_entity;
	}

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_idle(&pdev->dev);

	dev_info(&pdev->dev, "Probe success!");
	return 0;

error_media_entity:
	media_entity_cleanup(&kmb_mvmx->subdev.entity);
error_handler_free:
	v4l2_ctrl_handler_free(kmb_mvmx->subdev.ctrl_handler);
error_unregister_i2c_dev:
	if (kmb_mvmx->client)
		i2c_unregister_device(kmb_mvmx->client);
error_mutex_destroy:
	mutex_destroy(&kmb_mvmx->mutex);

	return ret;
}

/**
 * kmb_mvmx_pdev_remove - Platform device unbinding
 * @pdev: pointer to platform device
 *
 * Return: 0 if successful
 */
static int kmb_mvmx_pdev_remove(struct platform_device *pdev)
{
	struct kmb_mvmx *kmb_mvmx =  platform_get_drvdata(pdev);
	struct v4l2_subdev *sd = &kmb_mvmx->subdev;

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(&pdev->dev);
	pm_runtime_suspended(&pdev->dev);

	if (kmb_mvmx->client)
		i2c_unregister_device(kmb_mvmx->client);

	mutex_destroy(&kmb_mvmx->mutex);

	return 0;
}

static const struct dev_pm_ops kmb_mvmx_platform_pm_ops = {
	SET_RUNTIME_PM_OPS(kmb_mvmx_platform_suspend,
			   kmb_mvmx_platform_resume, NULL)
};

static const struct of_device_id kmb_mvmx_id_table[] = {
	{.compatible = "intel,kmb-mvmx-sensor-p"},
	{}
};
MODULE_DEVICE_TABLE(of, kmb_mvmx_id_table);

static struct platform_driver kmb_mvmx_platform_driver = {
	.probe	= kmb_mvmx_pdev_probe,
	.remove = kmb_mvmx_pdev_remove,
	.driver = {
		.name = KMB_MVMX_DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &kmb_mvmx_platform_pm_ops,
		.of_match_table = kmb_mvmx_id_table,
	}
};

module_platform_driver(kmb_mvmx_platform_driver);

MODULE_DESCRIPTION("KeemBay Myriad X Sensor driver");
MODULE_LICENSE("GPL v2");
