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
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/v4l2-dv-timings.h>

#include <media/v4l2-dv-timings.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <linux/types.h>

#include <linux/version.h>
#include <media/lt6911uxc.h>

#include <linux/ipu-isys.h>

/* v4l2 debug level */
static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-3)");

#define FSERIAL_CLK_4_LANE		240000000ULL
#define FSERIAL_CLK_2_LANE		144000000ULL

#define PIX_CLK_4_LANE			60000000ULL
#define PIX_CLK_2_LANE			18000000ULL

// LT6911UXC Register Setting

#define LT6911UXC_REG_VALUE_08BIT		1
#define LT6911UXC_REG_VALUE_16BIT		2
#define LT6911UXC_CHIP_ID			0x1704

#define LT6911UXC_CID_CSI_PORT	(V4L2_CID_USER_BASE | 0x1001)
#define LT6911UXC_CID_I2C_BUS	(V4L2_CID_USER_BASE | 0x1002)
#define LT6911UXC_CID_I2C_ID	(V4L2_CID_USER_BASE | 0x1003)
#define LT6911UXC_CID_I2C_SLAVE_ADDRESS	(V4L2_CID_USER_BASE | 0x1004)
#define LT6911UXC_CID_FPS	(V4L2_CID_USER_BASE | 0x1005)
#define LT6911UXC_CID_FRAME_INTERVAL	(V4L2_CID_USER_BASE | 0x1006)
#define LT6911UXC_CID_AUDIO_SAMPLING_RATE	(V4L2_CID_USER_BASE | 0x1007)
#define LT6911UXC_CID_AUDIO_PRESENT	(V4L2_CID_USER_BASE | 0x1008)

#define REG_CHIP_ID	0x8100

/* Control */
#define REG_BANK	0xFF

#define REG_ENABLE_I2C	0x80EE
#define REG_DISABLE_WD	0x8010

/* Resolution registers */
#define REG_H_TOTAL	0x867C	/* horizontal half total pixel */
#define REG_H_ACTIVE	0x8680	/* horizontal half active pixel */
#define REG_H_FP_0P5	0x8678	/* horizontal half front porch pixel */
#define REG_H_BP_0P5	0x8676	/* horizontal half back porch pixel */
#define REG_H_SW_0P5	0x8672	/* hsync half length pixel  */
#define REG_V_TOTAL	0x867A	/* vertical total lines */
#define REG_V_ACTIVE	0x867E	/* vertical active lines */
#define REG_V_BP	0x8674	/* vertical back porch lines */
#define REG_V_FP	0x8675	/* vertical front porch lines */
#define REG_V_SW	0x8671	/* vsync length lines */
#define REG_SYNC_POL	0x8670	/* hsync/vsync polarity flags */
#define REG_BKB0_A2_REG	0xB0A2
#define REG_FM1_FREQ_IN2	0x8548
#define REG_FM1_FREQ_IN1	0x8549
#define REG_FM1_FREQ_IN0	0x854A

#define REG_AD_HALF_PCLK	0x8540

#define REG_TMDS_CLK_IN2	0x8750
#define REG_TMDS_CLK_IN1	0x8751
#define REG_TMDS_CLK_IN0	0x8752

/* MIPI-TX */
#define REG_MIPI_TX_CTRL	0x811D
#define REG_MIPI_LANES		0x86A2
#define REG_MIPI_CLK_MODE	0xD468

/* Audio sample rate */
#define REG_INT_AUDIO		0x86A5
#define AUDIO_DISCONNECT	0x88
#define AUDIO_SR_HIGH		0x55
#define AUDIO_SR_LOW		0xAA
#define REG_AUDIO_SR		0xB0AB

/* Interrupts */
#define REG_INT_HDMI	0x86A3
#define REG_INT_RESPOND	0x86A6
#define INT_HDMI_STABLE	0x55
#define INT_HDMI_DISCONNECT	0x88

#define REG_INT_AUDIO	0x86A5
#define INT_AUDIO_DISCONNECT	0x88
#define INT_AUDIO_SR_HIGH	0x55
#define INT_AUDIO_SR_LOW	0xAA

/* FPS registers */
#define MASK_FMI_FREQ2	0x0F
#define MASK_VSYNC_POL	(1 << 1)
#define MASK_HSYNC_POL	(1 << 0)

/* Frame ID registers */
#define REG_FRAME_ID	0xD40E
#define REG_FRAME_STATUS	0xD414

#define LT69111UXC_IRQ_MODE
#define MAX_MIPI_PORT_USE 3

static const struct v4l2_dv_timings_cap lt6911uxc_timings_cap_4kp30 = {
	.type = V4L2_DV_BT_656_1120,
	/* keep this initialization for compatibility with GCC < 4.4.6 */
	.reserved = { 0 },
	/* Pixel clock from REF_01 p. 20. Min/max height/width are unknown */
	V4L2_INIT_BT_TIMINGS(
		160, 3840,				/* min/max width */
		120, 2160,				/* min/max height */
		25000000, 297000000,			/* min/max pixelclock */
		V4L2_DV_BT_STD_CEA861 | V4L2_DV_BT_STD_DMT |
		V4L2_DV_BT_STD_CVT,
		V4L2_DV_BT_CAP_PROGRESSIVE | V4L2_DV_BT_CAP_CUSTOM |
		V4L2_DV_BT_CAP_REDUCED_BLANKING)
};

struct lt6911uxc_mode {
	/* Frame width in pixels */
	u32 width;

	/* Frame height in pixels */
	u32 height;

	/* Horizontal timining size */
	u32 hts;

	/* Default vertical timining size */
	u32 vts_def;

	/* Min vertical timining size */
	u32 vts_min;

	/* Link frequency needed for this resolution */
	u32 link_freq_index;

	/* MEDIA_BUS_FMT */
	u32 code;

	/* REG_MIPI_LANES */
	s32 lanes;

	/* MODE_FPS*/
	u32 fps;

	/* Bit per pixel */
	u32 bpp;

	/* Pixel rate*/
	u32 pixel_clk;

	/* Byte rate*/
	u32 byte_clk;

	/* Audio sample rate*/
	u32 audio_sample_rate;
};

struct lt6911uxc_state {
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *audio_sampling_rate_ctrl;
	struct v4l2_ctrl *audio_present_ctrl;
	struct v4l2_ctrl *csi_port;
	struct v4l2_ctrl *i2c_bus;
	struct v4l2_ctrl *i2c_id;
	struct v4l2_ctrl *i2c_slave_address;
	struct v4l2_ctrl *fps;
	struct v4l2_ctrl *frame_interval;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *analogue_gain;
	struct v4l2_ctrl *digital_gain;
	struct v4l2_ctrl *strobe_source;
	struct v4l2_ctrl *strobe;
	struct v4l2_ctrl *strobe_stop;
	struct v4l2_ctrl *timeout;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *query_sub_stream;
	struct v4l2_ctrl *set_sub_stream;

	struct v4l2_dv_timings timings;
	struct v4l2_dv_timings detected_timings;

	struct i2c_client *i2c_client;

	struct media_pad pad;
	struct mutex mutex;
	struct lt6911uxc_platform_data  *platform_data;

	/* Current mode */
	struct lt6911uxc_mode *cur_mode;

	bool streaming;

	u8 bank_i2c;
	bool enable_i2c;

	u32 mbus_fmt_code;			/* current media bus format */

	u32 thread_run;
	struct task_struct *poll_task;
	bool auxiliary_port;

	s64 sub_stream;
};

static const struct v4l2_event lt6911uxc_ev_source_change = {
	.type = V4L2_EVENT_SOURCE_CHANGE,
	.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
};

static const struct v4l2_event lt6911uxc_ev_stream_end = {
	.type = V4L2_EVENT_EOS,
};

static inline struct lt6911uxc_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct lt6911uxc_state, sd);
}

static void lt6911uxc_reg_bank(struct v4l2_subdev *sd, u8 bank)
{
	struct lt6911uxc_state *lt6911uxc = to_state(sd);
	struct i2c_client *client = lt6911uxc->i2c_client;
	int ret;
	struct i2c_msg msg;
	u8 data[2];
	u8 address;

	if (lt6911uxc->bank_i2c == bank)
		return;
	dev_dbg(&client->dev, "i2c: change register bank to 0x%02X\n",
		bank);

	address = 0xFF;
	msg.addr = client->addr;
	msg.buf = data;
	msg.len = 2;
	msg.flags = 0;

	data[0] = address;
	data[1] = bank;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1) {
		dev_info(&client->dev, "%s: switch to bank 0x%x from 0x%x failed\n",
			__func__, bank, client->addr);
		return;
	}
	lt6911uxc->bank_i2c = bank;
}

static void lt6911uxc_i2c_wr8(struct v4l2_subdev *sd, u16 reg, u8 val)
{
	struct lt6911uxc_state *lt6911uxc = to_state(sd);
	struct i2c_client *client = lt6911uxc->i2c_client;
	int ret;
	struct i2c_msg msg;
	u8 data[2];
	u8 address;

	/* write register bank offset */
	u8 bank = (reg >> 8) & 0xFF;

	lt6911uxc_reg_bank(sd, bank);
	address = reg & 0xFF;
	msg.addr = client->addr;
	msg.buf = data;
	msg.len = 2;
	msg.flags = 0;

	data[0] = address;
	data[1] = val;

	ret = i2c_transfer(client->adapter, &msg, 1);

	if (ret != 1) {
		dev_info(&client->dev, "%s: write register 0x%x from 0x%x failed\n",
			__func__, reg, client->addr);
	}
	dev_dbg(&client->dev, "i2c: write register: 0x%04X = 0x%02X\n",
		reg, val);
}

static void lt6911uxc_i2c_rd(struct v4l2_subdev *sd, u16 reg, u8 *values, u32 n)
{
	struct lt6911uxc_state *lt6911uxc = to_state(sd);
	struct i2c_client *client = lt6911uxc->i2c_client;
	int ret;
	u8 reg_addr[1] = { (u8)(reg & 0xff) };
	u8 bank_addr   = (u8)((reg >> 8) & 0xFF);

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,		/* write */
			.len = 1,
			.buf = reg_addr,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,	/* read n bytes */
			.len = n,
			.buf = values,
		},
	};

	/* write register bank offset */
	lt6911uxc_reg_bank(sd, bank_addr);

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs)) {
		dev_info(&client->dev, "%s: read register 0x%04X from 0x%x failed\n",
			__func__, reg, client->addr);
	}
}

static u8 lt6911uxc_i2c_rd8(struct v4l2_subdev *sd, u16 reg)
{
	u8 val = 0;

	lt6911uxc_i2c_rd(sd, reg, &val, 1);

	dev_dbg(sd->dev, "i2c: read 0x%04X = 0x%02X\n", reg, val);
	return val;
}

static u16 lt6911uxc_i2c_rd16(struct v4l2_subdev *sd, u16 reg)
{
	u16 val = 0;
	u8 tmp;

	lt6911uxc_i2c_rd(sd, reg, (u8 *)&val, 2);
	/* high byte always at lower address -> swap */
	tmp = val & 0xFF;
	val = (val >> 8) | tmp << 8;

	dev_dbg(sd->dev, "i2c: read 0x%04X = 0x%04X\n", reg, val);
	return val;
}

static void lt6911uxc_ext_control(
	struct lt6911uxc_state *lt6911uxc,
	bool enable)
{
	if (!lt6911uxc)
		return;

	if (lt6911uxc->enable_i2c == enable)
		return;

	lt6911uxc->enable_i2c = enable;
	if (enable) {
		lt6911uxc_i2c_wr8(&lt6911uxc->sd, REG_ENABLE_I2C, 0x01);
		lt6911uxc_i2c_wr8(&lt6911uxc->sd, REG_DISABLE_WD, 0x00);
	} else
		lt6911uxc_i2c_wr8(&lt6911uxc->sd, REG_ENABLE_I2C, 0x00);
}

static int lt6911uxc_csi_enable(struct v4l2_subdev *sd, bool enable)
{
	if (enable)
		lt6911uxc_i2c_wr8(sd, REG_MIPI_TX_CTRL, 0xFB);
	else
		lt6911uxc_i2c_wr8(sd, REG_MIPI_TX_CTRL, 0x00);
	return 0;
}

static int lt6911uxc_get_audio_sampling_rate(struct lt6911uxc_state *lt6911uxc)
{
	int audio_fs, idx;
	static const int eps = 1500;
	static const int rates_default[] = {
		32000, 44100, 48000, 88200, 96000, 176400, 192000
	};

	audio_fs = lt6911uxc_i2c_rd8(&lt6911uxc->sd, REG_AUDIO_SR) * 1000;
	dev_dbg(&lt6911uxc->i2c_client->dev, "%s: Audio sample rate %d [Hz]\n",
		__func__, audio_fs);

	/* audio_fs is an approximation of sample rate - search nearest */
	for (idx = 0; idx < ARRAY_SIZE(rates_default); ++idx) {
		if ((rates_default[idx] - eps < audio_fs) &&
		    (rates_default[idx] + eps > audio_fs))
			return rates_default[idx];
	}
	dev_info(&lt6911uxc->i2c_client->dev, "%s: unhandled sampling rate %d [Hz]",
		__func__, audio_fs);
	return 0;
}

static int lt6911uxc_log_status(struct v4l2_subdev *sd)
{

#ifdef TIMINGS_ENABLE
	struct lt6911uxc_state *lt6911uxc = to_state(sd);

	v4l2_info(sd, "----- Timings -----\n");
	if (!&lt6911uxc->detected_timings.bt.width) {
		v4l2_info(sd, "no video detected\n");
	} else {
		v4l2_print_dv_timings(sd->name, "detected format: ",
				&lt6911uxc->detected_timings, true);
	}
	v4l2_print_dv_timings(sd->name, "configured format: ",
		&lt6911uxc->timings, true);
#endif
	return 0;
}

static int lt6911uxc_subscribe_event(struct v4l2_subdev *sd, struct v4l2_fh *fh,
		struct v4l2_event_subscription *sub)
{
	dev_info(sd->dev, "%s():\n", __func__);

	switch (sub->type) {
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subdev_subscribe(sd, fh, sub);
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 2, NULL);
	default:
		return -EINVAL;
	}
}

static const struct v4l2_dv_timings_cap *lt6911uxc_g_timings_cap(
		struct lt6911uxc_state *lt6911uxc)
{
	return &lt6911uxc_timings_cap_4kp30;
}

static int lt6911uxc_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct lt6911uxc_state *lt6911uxc = to_state(sd);

	*status = 0;
	*status |= lt6911uxc->streaming ? V4L2_IN_ST_NO_SIGNAL : 0;

	v4l2_dbg(1, debug, sd, "%s: status = 0x%x\n", __func__, *status);
	return 0;
}

static int __maybe_unused lt6911uxc_s_dv_timings(struct v4l2_subdev *sd,
		struct v4l2_dv_timings *timings)
{
	struct lt6911uxc_state *lt6911uxc = to_state(sd);

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	if (!v4l2_valid_dv_timings(timings, lt6911uxc_g_timings_cap(lt6911uxc),
				   NULL, NULL)) {
		v4l2_err(sd, "%s: timings out of range\n", __func__);
		return -EINVAL;
	}

	v4l2_find_dv_timings_cap(timings, lt6911uxc_g_timings_cap(lt6911uxc), 0,
				 NULL, NULL);
	memset(timings->bt.reserved, 0, sizeof(timings->bt.reserved));
	lt6911uxc->timings = *timings;
	return 0;
}

static int lt6911uxc_g_dv_timings(struct v4l2_subdev *sd,
		struct v4l2_dv_timings *timings)
{
	struct lt6911uxc_state *lt6911uxc = to_state(sd);

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	*timings = lt6911uxc->timings;
	return 0;
}

static int __maybe_unused lt6911uxc_query_dv_timings(struct v4l2_subdev *sd,
		struct v4l2_dv_timings *timings)
{
	struct lt6911uxc_state *lt6911uxc = to_state(sd);

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	if (false == lt6911uxc->streaming) {
		v4l2_warn(sd, "%s: no valid signal\n", __func__);
		return -ENOLINK;
	}

	if (!v4l2_valid_dv_timings(&lt6911uxc->detected_timings,
		lt6911uxc_g_timings_cap(lt6911uxc), NULL, NULL)) {
		v4l2_warn(sd, "%s: timings out of range\n", __func__);
		return -ERANGE;
	}

	*timings = lt6911uxc->detected_timings;
	return 0;
}

static const struct v4l2_ctrl_config lt6911uxc_ctrl_audio_sampling_rate = {
	.id = LT6911UXC_CID_AUDIO_SAMPLING_RATE,
	.name = "Audio Sampling Rate",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 32000,
	.max = 192000,
	.step = 100,
	.def = 48000,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

static const struct v4l2_ctrl_config lt6911uxc_ctrl_audio_present = {
	.id = LT6911UXC_CID_AUDIO_PRESENT,
	.name = "Audio Present",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

static u64 __maybe_unused get_hblank(struct lt6911uxc_state *lt6911uxc)
{
	u64 hblank;
	u64 pixel_rate;
	u64 pixel_clk;

	if (lt6911uxc->cur_mode->lanes == 4) {
		pixel_rate = FSERIAL_CLK_4_LANE;
		pixel_clk = PIX_CLK_4_LANE;
	} else if (lt6911uxc->cur_mode->lanes == 2) {
		pixel_rate = FSERIAL_CLK_2_LANE;
		pixel_clk = PIX_CLK_2_LANE;
	} else {
		pixel_rate = FSERIAL_CLK_4_LANE;
		pixel_clk = PIX_CLK_4_LANE;
	}

	if (pixel_clk)
		hblank = 0x128 * (pixel_rate / pixel_clk);
	else
		hblank = 1184;

	return hblank;
}

static int lt6911uxc_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct lt6911uxc_state *lt6911uxc = container_of(ctrl->handler,
				struct lt6911uxc_state, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&lt6911uxc->sd);
	s64 exposure_max;
	int ret = 0;

	/* Propagate change of current control to all related controls */
	if (ctrl->id == V4L2_CID_VBLANK) {
		/* Update max exposure while meeting expected vblanking */
		exposure_max = 1;
		__v4l2_ctrl_modify_range(lt6911uxc->exposure,
					 lt6911uxc->exposure->minimum,
					 exposure_max,
					 lt6911uxc->exposure->step,
					 lt6911uxc->cur_mode->height - 1);
	}

	/* V4L2 controls values will be applied only when power is already up */
	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		dev_dbg(&client->dev, "set analogue gain.\n");
		break;

	case V4L2_CID_DIGITAL_GAIN:
		dev_dbg(&client->dev, "set digital gain.\n");
		break;

	case V4L2_CID_EXPOSURE:
		dev_dbg(&client->dev, "set exposure time.\n");
		break;

	case V4L2_CID_VBLANK:
		dev_dbg(&client->dev, "set vblank %d\n",
			lt6911uxc->cur_mode->height + ctrl->val);
		break;
	case V4L2_CID_FLASH_STROBE_SOURCE:
		dev_dbg(&client->dev, "set led flash source %d\n", ctrl->val);
		break;

	case V4L2_CID_FLASH_STROBE:
		dev_dbg(&client->dev, "set flash strobe.\n");
		break;

	case V4L2_CID_FLASH_STROBE_STOP:
		dev_dbg(&client->dev, "turn off led %d\n", ctrl->val);
		break;

	case V4L2_CID_FLASH_TIMEOUT:
		dev_dbg(&client->dev, "set led delay\n");
		break;

	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops lt6911uxc_ctrl_ops = {
	.s_ctrl = lt6911uxc_set_ctrl,
};

static struct v4l2_ctrl_config lt6911uxc_csi_port = {
	.ops	= &lt6911uxc_ctrl_ops,
	.id	= LT6911UXC_CID_CSI_PORT,
	.type	= V4L2_CTRL_TYPE_INTEGER,
	.name	= "CSI port",
	.min	= 1,
	.max	= 5,
	.def	= 1,
	.step	= 1,
	.flags	= V4L2_CTRL_FLAG_READ_ONLY,
};

static struct v4l2_ctrl_config lt6911uxc_i2c_bus = {
	.ops	= &lt6911uxc_ctrl_ops,
	.id	= LT6911UXC_CID_I2C_BUS,
	.type	= V4L2_CTRL_TYPE_INTEGER,
	.name	= "I2C bus",
	.min	= 0,
	.max	= MINORMASK,
	.def	= 0,
	.step	= 1,
	.flags	= V4L2_CTRL_FLAG_READ_ONLY,
};

static struct v4l2_ctrl_config lt6911uxc_i2c_id = {
	.ops	= &lt6911uxc_ctrl_ops,
	.id	= LT6911UXC_CID_I2C_ID,
	.type	= V4L2_CTRL_TYPE_INTEGER,
	.name	= "I2C id",
	.min	= 0x10,
	.max	= 0x77,
	.def	= 0x10,
	.step	= 1,
	.flags	= V4L2_CTRL_FLAG_READ_ONLY,
};

static struct v4l2_ctrl_config lt6911uxc_i2c_slave_address = {
	.ops	= &lt6911uxc_ctrl_ops,
	.id	= LT6911UXC_CID_I2C_SLAVE_ADDRESS,
	.type	= V4L2_CTRL_TYPE_INTEGER,
	.name	= "I2C slave address",
	.min	= 0x0,
	.max	= 0x7f,
	.def	= 0x2b,
	.step	= 1,
	.flags	= V4L2_CTRL_FLAG_READ_ONLY,
};

static struct v4l2_ctrl_config lt6911uxc_fps = {
	.ops	= &lt6911uxc_ctrl_ops,
	.id	= LT6911UXC_CID_FPS,
	.type	= V4L2_CTRL_TYPE_INTEGER,
	.name	= "fps",
	.min	= 25,
	.max	= 60,
	.def	= 30,
	.step	= 1,
	.flags	= V4L2_CTRL_FLAG_READ_ONLY,
};

static struct v4l2_ctrl_config lt6911uxc_frame_interval = {
	.ops	= &lt6911uxc_ctrl_ops,
	.id	= LT6911UXC_CID_FRAME_INTERVAL,
	.type	= V4L2_CTRL_TYPE_INTEGER,
	.name	= "frame interval",
	.min	= 16,
	.max	= 40,
	.def	= 33,
	.step	= 1,
	.flags	= V4L2_CTRL_FLAG_READ_ONLY,
};

static struct v4l2_ctrl_config lt6911uxc_q_sub_stream = {
	.ops = &lt6911uxc_ctrl_ops,
	.id = V4L2_CID_IPU_QUERY_SUB_STREAM,
	.name = "query virtual channel",
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.max = 1,
	.min = 0,
	.def = 0,
	.menu_skip_mask = 0,
	.qmenu_int = NULL,
};

static const struct v4l2_ctrl_config lt6911uxc_s_sub_stream = {
	.ops = &lt6911uxc_ctrl_ops,
	.id = V4L2_CID_IPU_SET_SUB_STREAM,
	.name = "set virtual channel",
	.type = V4L2_CTRL_TYPE_INTEGER64,
	.max = 0xFFFF,
	.min = 0,
	.def = 0,
	.step = 1,
};

static u64 get_pixel_rate(struct lt6911uxc_state *lt6911uxc)
{
	if (lt6911uxc->cur_mode->lanes)
		return lt6911uxc->cur_mode->width * lt6911uxc->cur_mode->height *
			lt6911uxc->cur_mode->fps * 16 / lt6911uxc->cur_mode->lanes;
	else
		return 995328000; /* default value: 4K@30 */
}

#define MIPI_CSI2_TYPE_YUV422_8         0x1e

static unsigned int mbus_code_to_mipi(u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_UYVY8_1X16:
		return MIPI_CSI2_TYPE_YUV422_8;
	default:
		WARN_ON(1);
		return -EINVAL;
	}
}

static void set_sub_stream_fmt(s64 *sub_stream, u32 code)
{
       *sub_stream &= 0xFFFFFFFFFFFF0000;
       *sub_stream |= code;
}

static void set_sub_stream_h(s64 *sub_stream, u32 height)
{
       s64 val = height;
       val &= 0xFFFF;
       *sub_stream &= 0xFFFFFFFF0000FFFF;
       *sub_stream |= val << 16;
}

static void set_sub_stream_w(s64 *sub_stream, u32 width)
{
       s64 val = width;
       val &= 0xFFFF;
       *sub_stream &= 0xFFFF0000FFFFFFFF;
       *sub_stream |= val << 32;
}

static void set_sub_stream_dt(s64 *sub_stream, u32 dt)
{
       s64 val = dt;
       val &= 0xFF;
       *sub_stream &= 0xFF00FFFFFFFFFFFF;
       *sub_stream |= val << 48;
}

static void set_sub_stream_vc_id(s64 *sub_stream, u32 vc_id)
{
       s64 val = vc_id;
       val &= 0xFF;
       *sub_stream &= 0x00FFFFFFFFFFFFFF;
       *sub_stream |= val << 56;
}

static int lt6911uxc_init_controls(struct lt6911uxc_state *lt6911uxc)
{
	struct i2c_client *client = v4l2_get_subdevdata(&lt6911uxc->sd);
	struct v4l2_ctrl_handler *ctrl_hdlr;
	s64 hblank;
	int ret;

	ctrl_hdlr = &lt6911uxc->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 8);
	if (ret)
		return ret;

	ctrl_hdlr->lock = &lt6911uxc->mutex;
	lt6911uxc->link_freq =
		v4l2_ctrl_new_int_menu(ctrl_hdlr,
			&lt6911uxc_ctrl_ops,
			V4L2_CID_LINK_FREQ,
			sizeof(lt6911uxc->cur_mode->pixel_clk),
			0, (s64 *)&lt6911uxc->cur_mode->pixel_clk);
	if (ctrl_hdlr->error) {
		dev_dbg(&client->dev, "Set ctrl_hdlr, err=%d.\n",
			ctrl_hdlr->error);
		return ctrl_hdlr->error;
	}
	if (lt6911uxc->link_freq)
		lt6911uxc->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	lt6911uxc->vblank = v4l2_ctrl_new_std(ctrl_hdlr,
				&lt6911uxc_ctrl_ops,
				V4L2_CID_VBLANK, 0, 1, 1, 1);
	if (ctrl_hdlr->error) {
		dev_dbg(&client->dev, "Set ctrl_hdlr, err=%d.\n",
			ctrl_hdlr->error);
		return ctrl_hdlr->error;
	}

	lt6911uxc->analogue_gain = v4l2_ctrl_new_std(ctrl_hdlr,
			&lt6911uxc_ctrl_ops,
			V4L2_CID_ANALOGUE_GAIN, 0, 1, 1, 1);
	if (ctrl_hdlr->error) {
		dev_dbg(&client->dev, "Set ctrl_hdlr, err=%d.\n",
			ctrl_hdlr->error);
		return ctrl_hdlr->error;
	}

	lt6911uxc->digital_gain = v4l2_ctrl_new_std(ctrl_hdlr,
			&lt6911uxc_ctrl_ops,
			V4L2_CID_DIGITAL_GAIN,	0, 1, 1, 1);
	if (ctrl_hdlr->error) {
		dev_dbg(&client->dev, "Set ctrl_hdlr, err=%d.\n",
			ctrl_hdlr->error);
		return ctrl_hdlr->error;
	}

	lt6911uxc->exposure = v4l2_ctrl_new_std(ctrl_hdlr,
				&lt6911uxc_ctrl_ops,
				V4L2_CID_EXPOSURE, 0, 1, 1, 1);
	if (ctrl_hdlr->error) {
		dev_dbg(&client->dev, "Set ctrl_hdlr, err=%d.\n",
			ctrl_hdlr->error);
		return ctrl_hdlr->error;
	}
	lt6911uxc_csi_port.def = lt6911uxc->platform_data->port;
	lt6911uxc->csi_port =
		v4l2_ctrl_new_custom(ctrl_hdlr, &lt6911uxc_csi_port, NULL);
	if (ctrl_hdlr->error) {
		dev_dbg(&client->dev, "Set ctrl_hdlr, err=%d.\n",
			ctrl_hdlr->error);
		return ctrl_hdlr->error;
	}

	lt6911uxc_i2c_bus.def = i2c_adapter_id(client->adapter);
	lt6911uxc->i2c_bus =
		v4l2_ctrl_new_custom(ctrl_hdlr, &lt6911uxc_i2c_bus, NULL);
	if (ctrl_hdlr->error) {
		dev_dbg(&client->dev, "Set ctrl_hdlr, err=%d.\n",
			ctrl_hdlr->error);
		return ctrl_hdlr->error;
	}

	lt6911uxc_i2c_id.def = client->addr;
	lt6911uxc->i2c_id = v4l2_ctrl_new_custom(ctrl_hdlr,
				&lt6911uxc_i2c_id, NULL);
	if (ctrl_hdlr->error) {
		dev_dbg(&client->dev, "Set ctrl_hdlr, err=%d.\n",
			ctrl_hdlr->error);
		return ctrl_hdlr->error;
	}

	lt6911uxc_i2c_slave_address.def =
		lt6911uxc->platform_data->i2c_slave_address;
	lt6911uxc->i2c_slave_address = v4l2_ctrl_new_custom(ctrl_hdlr,
					&lt6911uxc_i2c_slave_address, NULL);
	if (ctrl_hdlr->error) {
		dev_dbg(&client->dev, "Set ctrl_hdlr, err=%d.\n",
			ctrl_hdlr->error);
		return ctrl_hdlr->error;
	}

	lt6911uxc_fps.def = lt6911uxc->cur_mode->fps;
	lt6911uxc->fps = v4l2_ctrl_new_custom(ctrl_hdlr, &lt6911uxc_fps, NULL);
	if (ctrl_hdlr->error) {
		dev_dbg(&client->dev, "Set ctrl_hdlr, err=%d.\n",
			ctrl_hdlr->error);
		return ctrl_hdlr->error;
	}

	if (lt6911uxc->cur_mode->fps)
		lt6911uxc_frame_interval.def = 1000 / lt6911uxc->cur_mode->fps;
	else
		lt6911uxc_frame_interval.def = 33;

	lt6911uxc->frame_interval = v4l2_ctrl_new_custom(ctrl_hdlr,
					&lt6911uxc_frame_interval, NULL);
	if (ctrl_hdlr->error) {
		dev_dbg(&client->dev, "Set ctrl_hdlr, err=%d.\n",
			ctrl_hdlr->error);
		return ctrl_hdlr->error;
	}

	lt6911uxc->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr,
				&lt6911uxc_ctrl_ops,
				V4L2_CID_PIXEL_RATE,
				get_pixel_rate(lt6911uxc),
				get_pixel_rate(lt6911uxc), 1,
				get_pixel_rate(lt6911uxc));
	if (ctrl_hdlr->error) {
		dev_dbg(&client->dev, "Set ctrl_hdlr, err=%d.\n",
			ctrl_hdlr->error);
		return ctrl_hdlr->error;
	}
	if (lt6911uxc->pixel_rate)
		lt6911uxc->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	hblank = 1;
	lt6911uxc->hblank = v4l2_ctrl_new_std(ctrl_hdlr,
				&lt6911uxc_ctrl_ops,
				V4L2_CID_HBLANK, 0, 1, 1, 1);
	if (ctrl_hdlr->error) {
		dev_dbg(&client->dev, "Set ctrl_hdlr, err=%d.\n",
			ctrl_hdlr->error);
		return ctrl_hdlr->error;
	}
	if (lt6911uxc->hblank)
		lt6911uxc->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* custom v4l2 audio controls */
	lt6911uxc->audio_sampling_rate_ctrl = v4l2_ctrl_new_custom(
		ctrl_hdlr, &lt6911uxc_ctrl_audio_sampling_rate, NULL);
	if (ctrl_hdlr->error) {
		dev_dbg(&client->dev, "Set audio sampling rate ctrl, err=%d.\n",
			 ctrl_hdlr->error);
		return ctrl_hdlr->error;
	}
	lt6911uxc->audio_present_ctrl = v4l2_ctrl_new_custom(ctrl_hdlr,
		&lt6911uxc_ctrl_audio_present, NULL);
	if (ctrl_hdlr->error) {
		dev_dbg(&client->dev, "Set audio present ctrl, error = %d.\n",
			ctrl_hdlr->error);
		return ctrl_hdlr->error;
	}

	lt6911uxc_q_sub_stream.qmenu_int = &lt6911uxc->sub_stream;
	lt6911uxc->query_sub_stream = v4l2_ctrl_new_custom(ctrl_hdlr, &lt6911uxc_q_sub_stream, NULL);
	if (ctrl_hdlr->error) {
		dev_dbg(&client->dev, "Set query sub stream ctrl, error = %d.\n",
			ctrl_hdlr->error);
		return ctrl_hdlr->error;
	}
	lt6911uxc->set_sub_stream = v4l2_ctrl_new_custom(ctrl_hdlr, &lt6911uxc_s_sub_stream, NULL);
	if (ctrl_hdlr->error) {
		dev_dbg(&client->dev, "Set set sub stream ctrl, error = %d.\n",
			ctrl_hdlr->error);
		return ctrl_hdlr->error;
	}

	lt6911uxc->sd.ctrl_handler = ctrl_hdlr;
	return 0;
}

static void lt6911uxc_update_pad_format(const struct lt6911uxc_mode *mode,
				     struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = mode->code;
	fmt->field = V4L2_FIELD_NONE;
}

static int lt6911uxc_start_streaming(struct lt6911uxc_state *lt6911uxc)
{
	int ret = 0;

	if (lt6911uxc->auxiliary_port == true)
		return 0;

	lt6911uxc_ext_control(lt6911uxc, true);
	lt6911uxc_csi_enable(&lt6911uxc->sd, true);
	lt6911uxc_ext_control(lt6911uxc, false);

	ret = __v4l2_ctrl_handler_setup(lt6911uxc->sd.ctrl_handler);
	if (ret)
		return ret;

	return 0;
}

static void lt6911uxc_stop_streaming(struct lt6911uxc_state *lt6911uxc)
{
	if (lt6911uxc->auxiliary_port == true)
		return;

	/*The fps of 1080p60fps will be dropped to half when the CSI disabled. */
	lt6911uxc_ext_control(lt6911uxc, true);
	lt6911uxc_csi_enable(&lt6911uxc->sd, false);
	lt6911uxc_ext_control(lt6911uxc, false);
}

static int lt6911uxc_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct lt6911uxc_state *lt6911uxc = to_state(sd);
	int ret = 0;

	if (lt6911uxc->streaming == enable)
		return 0;

	if (lt6911uxc->auxiliary_port == true)
		return 0;

	mutex_lock(&lt6911uxc->mutex);
	if (enable) {
		dev_dbg(sd->dev, "[%s()], start streaming.\n", __func__);
		ret = lt6911uxc_start_streaming(lt6911uxc);
		if (ret) {
			enable = 0;
			lt6911uxc_stop_streaming(lt6911uxc);
		}
	} else {
		dev_dbg(sd->dev, "[%s()], stop streaming.\n", __func__);
		lt6911uxc_stop_streaming(lt6911uxc);
	}
	mutex_unlock(&lt6911uxc->mutex);

	lt6911uxc->streaming = enable;

	return ret;
}

static int lt6911uxc_g_frame_interval(struct v4l2_subdev *sd,
		struct v4l2_subdev_frame_interval *fival)
{
	struct lt6911uxc_state *lt6911uxc = to_state(sd);

	fival->pad = 0;
	fival->interval.numerator = 1;
	fival->interval.denominator = lt6911uxc->cur_mode->fps;

	return 0;
}

static int lt6911uxc_set_format(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
			     struct v4l2_subdev_format *fmt)
{
	struct lt6911uxc_state *lt6911uxc = to_state(sd);
	s32 vblank_def;
	s64 hblank;

	mutex_lock(&lt6911uxc->mutex);
	lt6911uxc_update_pad_format(lt6911uxc->cur_mode, &fmt->format);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		*v4l2_subdev_get_try_format(sd, sd_state, fmt->pad) = fmt->format;
	} else {
		__v4l2_ctrl_s_ctrl(lt6911uxc->link_freq,
			lt6911uxc->cur_mode->link_freq_index);
		__v4l2_ctrl_modify_range(lt6911uxc->pixel_rate,
			25000000, 297000000, 1,
			get_pixel_rate(lt6911uxc));

		hblank = get_hblank(lt6911uxc);
		__v4l2_ctrl_modify_range(lt6911uxc->hblank,
					hblank,
					hblank,
					1,
					hblank);

		/* Update limits and set FPS to default */
		vblank_def = lt6911uxc->cur_mode->vts_def - lt6911uxc->cur_mode->height;
		__v4l2_ctrl_modify_range(lt6911uxc->vblank,
					 0,
					 0xffff - lt6911uxc->cur_mode->height, 1,
					 vblank_def);
		__v4l2_ctrl_s_ctrl(lt6911uxc->vblank, vblank_def);

		__v4l2_ctrl_s_ctrl(lt6911uxc->fps, lt6911uxc->cur_mode->fps);

		if (lt6911uxc->cur_mode->fps)
			__v4l2_ctrl_s_ctrl(lt6911uxc->frame_interval, 1000 / lt6911uxc->cur_mode->fps);
		else
			__v4l2_ctrl_s_ctrl(lt6911uxc->frame_interval, 33);
	}
	set_sub_stream_fmt(&lt6911uxc->sub_stream, fmt->format.code);
	set_sub_stream_h(&lt6911uxc->sub_stream, fmt->format.height);
	set_sub_stream_w(&lt6911uxc->sub_stream, fmt->format.width);
	set_sub_stream_dt(&lt6911uxc->sub_stream, mbus_code_to_mipi(fmt->format.code));
	set_sub_stream_vc_id(&lt6911uxc->sub_stream, 0);
	mutex_unlock(&lt6911uxc->mutex);

	return 0;
}

static int lt6911uxc_get_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *sd_state,
			     struct v4l2_subdev_format *fmt)
{
	struct lt6911uxc_state *lt6911uxc = to_state(sd);

	mutex_lock(&lt6911uxc->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt->format = *v4l2_subdev_get_try_format(&lt6911uxc->sd, sd_state,
							fmt->pad);
	else
		lt6911uxc_update_pad_format(lt6911uxc->cur_mode, &fmt->format);

	mutex_unlock(&lt6911uxc->mutex);

	return 0;
}

static int lt6911uxc_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct lt6911uxc_state *lt6911uxc = to_state(sd);

	code->code = lt6911uxc->cur_mode->code;
	return 0;
}

static int lt6911uxc_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct lt6911uxc_state *lt6911uxc = to_state(sd);

	fse->min_width = lt6911uxc->cur_mode->width;
	fse->max_width = fse->min_width;
	fse->min_height = lt6911uxc->cur_mode->height;
	fse->max_height = fse->min_height;

	return 0;
}

static int lt6911uxc_enum_frame_interval(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *sd_state,
		struct v4l2_subdev_frame_interval_enum *fie)
{
	struct lt6911uxc_state *lt6911uxc = to_state(sd);

	fie->interval.numerator = 1;
	fie->interval.denominator = lt6911uxc->cur_mode->fps;

	return 0;
}

static int lt6911uxc_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct lt6911uxc_state *lt6911uxc = to_state(sd);

	if (!lt6911uxc->auxiliary_port)
		lt6911uxc_set_stream(sd, true);
	lt6911uxc_update_pad_format(lt6911uxc->cur_mode,
			v4l2_subdev_get_try_format(sd, fh->state, 0));

	return 0;
}

static int lt6911uxc_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{

	struct lt6911uxc_state *lt6911uxc = to_state(sd);

	if (!lt6911uxc->auxiliary_port)
		lt6911uxc_set_stream(sd, false);

	return 0;
}

static const struct v4l2_subdev_internal_ops lt6911uxc_subdev_internal_ops = {
	.open = lt6911uxc_open,
	.close = lt6911uxc_close,
};

static const struct v4l2_subdev_video_ops lt6911uxc_video_ops = {
	.s_stream = lt6911uxc_set_stream,
	.g_frame_interval = lt6911uxc_g_frame_interval,
	.g_input_status	= lt6911uxc_g_input_status,
//	.s_dv_timings	= lt6911uxc_s_dv_timings,
	.g_dv_timings	= lt6911uxc_g_dv_timings,
//	.query_dv_timings	= lt6911uxc_query_dv_timings,
	.s_stream	= lt6911uxc_set_stream,
};

static const struct v4l2_subdev_pad_ops lt6911uxc_pad_ops = {
	.set_fmt = lt6911uxc_set_format,
	.get_fmt = lt6911uxc_get_format,
	.enum_mbus_code = lt6911uxc_enum_mbus_code,
	.enum_frame_size = lt6911uxc_enum_frame_size,
	.enum_frame_interval = lt6911uxc_enum_frame_interval,
};

static struct v4l2_subdev_core_ops lt6911uxc_subdev_core_ops = {
	.log_status	= lt6911uxc_log_status,
	.subscribe_event	= lt6911uxc_subscribe_event,
	.unsubscribe_event	= v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_ops lt6911uxc_subdev_ops = {
	.core = &lt6911uxc_subdev_core_ops,
	.video = &lt6911uxc_video_ops,
	.pad = &lt6911uxc_pad_ops,
};

static const struct media_entity_operations lt6911uxc_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops lt6911uxc_internal_ops = {
	.open = lt6911uxc_open,
};

static int lt6911uxc_identify_module(struct lt6911uxc_state *lt6911uxc)
{
	struct i2c_client *client = v4l2_get_subdevdata(&lt6911uxc->sd);
	u16 val;

	val = lt6911uxc_i2c_rd16(&lt6911uxc->sd, REG_CHIP_ID);

	if (val != LT6911UXC_CHIP_ID) {
		dev_err(&client->dev, "chip id mismatch: %x!=%x",
			LT6911UXC_CHIP_ID, val);
		return -ENXIO;
	}
	dev_info(&client->dev,
		"Found lt6911uxc Bridge Chip, ID is 0x%x\n", val);
	return 0;
}

static int lt6911uxc_remove(struct i2c_client *client)
{
	return 0;
}

static int lt6911uxc_video_status_update(struct lt6911uxc_state *lt6911uxc)
{
	struct i2c_client *client = v4l2_get_subdevdata(&lt6911uxc->sd);
	u16 val = 0;
	u8 int_event = 0;
	u32 tmds_clk = 0;
	u32 byte_clock = 0, pixel_clk = 0;
	u8 is_hdmi_2_0 = 0;
	u32 htotal = 0, vtotal = 0;
	u32 width = 0, height = 0;
	u32 fps = 0, lanes = 4;
	u8 fm0 = 0,  fm1 = 0, fm2 = 0;

	/* Read interrupt event */
	int_event = lt6911uxc_i2c_rd8(&lt6911uxc->sd,
			REG_INT_HDMI);
	switch (int_event) {
	case INT_HDMI_STABLE:
		dev_dbg(&client->dev, "Video signal stable\n");

		/* byte clock / MIPI clock */
		lt6911uxc_i2c_wr8(&lt6911uxc->sd,
			REG_AD_HALF_PCLK, 0x1B);
		usleep_range(10000, 10100);
		fm2 = lt6911uxc_i2c_rd8(&lt6911uxc->sd,
			REG_FM1_FREQ_IN2) & MASK_FMI_FREQ2;
		fm1 = lt6911uxc_i2c_rd8(&lt6911uxc->sd,
			REG_FM1_FREQ_IN1);
		fm0 = lt6911uxc_i2c_rd8(&lt6911uxc->sd,
			REG_FM1_FREQ_IN0);

		byte_clock = (fm2<<16 | fm1<<8 | fm0) * 1000;

		/* TMDS clock / Pixel clock*/
		val =  lt6911uxc_i2c_rd8(&lt6911uxc->sd,
			REG_TMDS_CLK_IN2);
		tmds_clk |= ((val & 0x0f) << 16);
		val =  lt6911uxc_i2c_rd8(&lt6911uxc->sd,
			REG_TMDS_CLK_IN1);
		tmds_clk |= (val << 8);
		val =  lt6911uxc_i2c_rd8(&lt6911uxc->sd,
			REG_TMDS_CLK_IN0);
		tmds_clk |= val;

		val = lt6911uxc_i2c_rd8(&lt6911uxc->sd,
			REG_BKB0_A2_REG);
		is_hdmi_2_0 = val & (1<<0) ? !0 : !!0;
		pixel_clk = (is_hdmi_2_0 ? 4 * tmds_clk : tmds_clk) * 1000;

		/* video frame size */
		htotal = lt6911uxc_i2c_rd16(&lt6911uxc->sd,
			 REG_H_TOTAL) * 2;
		vtotal = lt6911uxc_i2c_rd16(&lt6911uxc->sd,
			 REG_V_TOTAL);
		width = lt6911uxc_i2c_rd16(&lt6911uxc->sd,
			 REG_H_ACTIVE) * 2;
		height = lt6911uxc_i2c_rd16(&lt6911uxc->sd,
			 REG_V_ACTIVE);
		if (htotal && vtotal)
			fps = pixel_clk / (htotal * vtotal);
		lanes = lt6911uxc_i2c_rd8(&lt6911uxc->sd,
			 REG_MIPI_LANES);

		lt6911uxc->cur_mode->height = height;
		lt6911uxc->cur_mode->fps = fps;
		lt6911uxc->cur_mode->code = MEDIA_BUS_FMT_UYVY8_1X16;

		if (lanes == 8) {
			/* 4K60fps with 2 MIPI ports*/
			if (width >= 3840)
				lt6911uxc->cur_mode->width = width/2; /* YUV422 */
			else
				lt6911uxc->cur_mode->width = width; /* YUV420 */
			lt6911uxc->cur_mode->lanes = lanes/2;
			lt6911uxc->cur_mode->pixel_clk = pixel_clk/2;
			lt6911uxc->cur_mode->byte_clk = byte_clock/2;
		} else {
			lt6911uxc->cur_mode->width = width;
			lt6911uxc->cur_mode->lanes = lanes;
			lt6911uxc->cur_mode->pixel_clk = pixel_clk;
			lt6911uxc->cur_mode->byte_clk = byte_clock;
		}
		v4l2_subdev_notify_event(&lt6911uxc->sd,
			&lt6911uxc_ev_source_change);

		dev_dbg(&client->dev,
			"width:%u, height:%u, fps:%u, lanes: %d\n",
			lt6911uxc->cur_mode->width,
			lt6911uxc->cur_mode->height,
			lt6911uxc->cur_mode->fps,
			lt6911uxc->cur_mode->lanes);
	break;
	case INT_HDMI_DISCONNECT:
		lt6911uxc->cur_mode->width = 0;
		lt6911uxc->cur_mode->height = 0;
		lt6911uxc->cur_mode->fps = 30;
		lt6911uxc->cur_mode->pixel_clk = 0;
		lt6911uxc->cur_mode->code = MEDIA_BUS_FMT_UYVY8_1X16;
		lt6911uxc->cur_mode->byte_clk = 0;
		lt6911uxc->cur_mode->lanes = 4;
		v4l2_subdev_notify_event(&lt6911uxc->sd,
			&lt6911uxc_ev_stream_end);

		dev_dbg(&client->dev, "Video signal disconnected\n");
	break;
	default:
		dev_dbg(&client->dev, "Unhandled video= 0x%02X\n", int_event);
	return  -ENOLINK;
	}

	return 0;
}

static void lt6911uxc_audio_status_update(struct lt6911uxc_state *lt6911uxc)
{
	struct i2c_client *client = v4l2_get_subdevdata(&lt6911uxc->sd);
	u8 int_event;
	int audio_fs = 0;

	/* read interrupt event */
	int_event = lt6911uxc_i2c_rd8(&lt6911uxc->sd, REG_INT_AUDIO);
	dev_dbg(&client->dev, "Audio event:0x%02X\n", int_event);

	switch (int_event) {
	case AUDIO_DISCONNECT:
		dev_dbg(&client->dev, "Audio signal disconnected\n");
		audio_fs = 0;
		lt6911uxc->cur_mode->audio_sample_rate = 0;
		break;
	case AUDIO_SR_HIGH:
	case AUDIO_SR_LOW:
			audio_fs = lt6911uxc_get_audio_sampling_rate(lt6911uxc);
			lt6911uxc->cur_mode->audio_sample_rate = audio_fs;
			dev_dbg(&client->dev,
				"Sampling rate changed: %d\n", audio_fs);
		break;
	default:
		dev_dbg(&client->dev, "Unhandled audio= 0x%02X\n", int_event);
		// use the default value for avoiding problem
		lt6911uxc->cur_mode->audio_sample_rate = 0;
		break;
	}

	__v4l2_ctrl_s_ctrl(lt6911uxc->audio_present_ctrl,
		(lt6911uxc->cur_mode->audio_sample_rate != 0));

	if (lt6911uxc->cur_mode->audio_sample_rate)
		__v4l2_ctrl_s_ctrl(lt6911uxc->audio_sampling_rate_ctrl,
			lt6911uxc->cur_mode->audio_sample_rate);
}

static void  lt6911uxc_check_status(struct lt6911uxc_state *lt6911uxc)
{
	mutex_lock(&lt6911uxc->mutex);
	lt6911uxc_ext_control(lt6911uxc, true);
	lt6911uxc_video_status_update(lt6911uxc);
	lt6911uxc_audio_status_update(lt6911uxc);
	lt6911uxc_i2c_wr8(&lt6911uxc->sd, REG_INT_RESPOND, 1);
	lt6911uxc_ext_control(lt6911uxc, false);
	mutex_unlock(&lt6911uxc->mutex);
}

static irqreturn_t lt6911uxc_threaded_irq_fn(int irq, void *dev_id)
{
	struct v4l2_subdev *sd = dev_id;
	struct lt6911uxc_state *lt6911uxc;

	if (!sd) {
		dev_err(NULL, "Invalid dev_id argument!\n");
		return IRQ_NONE;
	}

	lt6911uxc = to_state(sd);
	if (!lt6911uxc) {
		dev_err(sd->dev, "Invalid lt6911uxc state argument!\n");
		return IRQ_NONE;
	}

	dev_dbg(sd->dev, "%s in kthread %d\n", __func__, current->pid);
	lt6911uxc_check_status(lt6911uxc);
	return IRQ_HANDLED;
}

static int lt6911uxc_probe(struct i2c_client *client)
{
	struct lt6911uxc_state *lt6911uxc;
	struct v4l2_subdev *sd;
	int ret;

	lt6911uxc = devm_kzalloc(&client->dev, sizeof(struct lt6911uxc_state),
			     GFP_KERNEL);
	if (!lt6911uxc)
		return -ENOMEM;

	lt6911uxc->cur_mode = devm_kzalloc(&client->dev, sizeof(struct lt6911uxc_mode),
	     GFP_KERNEL);
	if (!lt6911uxc->cur_mode)
		return -ENOMEM;

	memset(lt6911uxc->cur_mode, 0, sizeof(struct lt6911uxc_mode));
	lt6911uxc->cur_mode->width = 3840,
	lt6911uxc->cur_mode->height = 2160,
	lt6911uxc->cur_mode->code = MEDIA_BUS_FMT_UYVY8_1X16,
	lt6911uxc->cur_mode->lanes = 4,
	lt6911uxc->cur_mode->fps = 30,
	lt6911uxc->cur_mode->bpp = 8,
	lt6911uxc->cur_mode->pixel_clk = 297000000,
	lt6911uxc->cur_mode->byte_clk = 168000000,
	lt6911uxc->cur_mode->audio_sample_rate = 48000,

	lt6911uxc->platform_data = client->dev.platform_data;
	if (lt6911uxc->platform_data == NULL) {
		dev_err(&client->dev, "no platform data provided\n");
		return -EINVAL;
	}

	lt6911uxc->i2c_client = client;
	sd = &lt6911uxc->sd;
	v4l2_i2c_subdev_init(sd, client, &lt6911uxc_subdev_ops);

	if (lt6911uxc->platform_data->suffix)
		snprintf(lt6911uxc->sd.name,
			sizeof(lt6911uxc->sd.name), "lt6911uxc %c",
			lt6911uxc->platform_data->suffix);

	mutex_init(&lt6911uxc->mutex);

	if (-1 != lt6911uxc->platform_data->reset_pin)
		if (!gpio_get_value(lt6911uxc->platform_data->reset_pin))
			gpio_set_value(lt6911uxc->platform_data->reset_pin, 1);

	if (-1 != lt6911uxc->platform_data->irq_pin) {
		lt6911uxc->auxiliary_port = false;
		dev_info(&client->dev, "Probing lt6911uxc chip...\n");

		lt6911uxc_ext_control(lt6911uxc, true);
		ret = lt6911uxc_identify_module(lt6911uxc);
		if (ret) {
			dev_err(&client->dev, "failed to find chip: %d", ret);
			return ret;
		}
		lt6911uxc_ext_control(lt6911uxc, false);

		ret = lt6911uxc_init_controls(lt6911uxc);
		if (ret) {
			dev_info(&client->dev,  "Could not init control %d!\n", ret);
			goto probe_error_v4l2_ctrl_handler_free;
		}

		lt6911uxc->sd.internal_ops = &lt6911uxc_internal_ops;
		lt6911uxc->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
		lt6911uxc->sd.entity.ops = &lt6911uxc_subdev_entity_ops;
		lt6911uxc->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
		lt6911uxc->pad.flags = MEDIA_PAD_FL_SOURCE;
		ret = media_entity_pads_init(&lt6911uxc->sd.entity, 1, &lt6911uxc->pad);
		if (ret) {
			dev_err(&client->dev, "Init entity pads failed:%d\n", ret);
			goto probe_error_v4l2_ctrl_handler_free;
		}

		/* Setting irq */
		ret = devm_gpio_request_one(&client->dev,
				lt6911uxc->platform_data->irq_pin,
				GPIOF_OUT_INIT_HIGH, "Interrupt signal");
		if (ret) {
			dev_err(&client->dev, "IRQ pin %d (name: %s) request failed! ret: %d\n",
				lt6911uxc->platform_data->irq_pin,
				lt6911uxc->platform_data->irq_pin_name, ret);
			goto probe_error_v4l2_ctrl_handler_free;
		}

		ret = gpio_direction_input(lt6911uxc->platform_data->irq_pin);
		if (ret) {
			dev_err(&client->dev, "Set gpio pin %d direction input failed! ret: %d\n",
				lt6911uxc->platform_data->irq_pin, ret);
			goto probe_error_v4l2_ctrl_handler_free;
		}

		ret = devm_request_threaded_irq(&client->dev,
				gpio_to_irq(lt6911uxc->platform_data->irq_pin),
				NULL, lt6911uxc_threaded_irq_fn,
				lt6911uxc->platform_data->irq_pin_flags,
				lt6911uxc->platform_data->irq_pin_name, lt6911uxc);
		if (ret) {
			dev_err(&client->dev, "IRQ request failed! ret: %d\n", ret);
			goto probe_error_v4l2_ctrl_handler_free;
		}
		/* Check the current status */
		usleep_range(200000, 205000);
		lt6911uxc_check_status(lt6911uxc);
		/* Stop to transmit MIPI data firstly waiting for IPU ready */
		lt6911uxc_stop_streaming(lt6911uxc);
	} else {
		/* 4K60fps mode, the setting needs to be fixed on 1920x2160@60fps*/
		lt6911uxc->auxiliary_port = true;
		lt6911uxc->cur_mode->width = 1920,
		lt6911uxc->cur_mode->height = 2160,
		lt6911uxc->cur_mode->fps = 60,

		ret = lt6911uxc_init_controls(lt6911uxc);
		if (ret) {
			dev_info(&client->dev,  "Could not init control %d!\n", ret);
			goto probe_error_v4l2_ctrl_handler_free;
		}

		lt6911uxc->sd.internal_ops = &lt6911uxc_internal_ops;
		lt6911uxc->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
		lt6911uxc->sd.entity.ops = &lt6911uxc_subdev_entity_ops;
		lt6911uxc->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
		lt6911uxc->pad.flags = MEDIA_PAD_FL_SOURCE;
		ret = media_entity_pads_init(&lt6911uxc->sd.entity, 1, &lt6911uxc->pad);
		if (ret) {
			dev_err(&client->dev, "Init entity pads failed:%d\n", ret);
			goto probe_error_v4l2_ctrl_handler_free;
		}
	}

	lt6911uxc->sd.dev = &client->dev;
	lt6911uxc->sd.internal_ops = &lt6911uxc_subdev_internal_ops;
	lt6911uxc->sd.flags |=
			V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	ret = v4l2_async_register_subdev_sensor(&lt6911uxc->sd);
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
	dev_info(&client->dev, "End to probe lt6911uxc Bridge Chip.\n");
	return 0;

probe_error_media_entity_cleanup:
	media_entity_cleanup(&lt6911uxc->sd.entity);

probe_error_v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(lt6911uxc->sd.ctrl_handler);
	mutex_destroy(&lt6911uxc->mutex);

	return ret;
}

static int lt6911uxc_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lt6911uxc_state *lt6911uxc = to_state(sd);

	if (-1 != lt6911uxc->platform_data->reset_pin)
		if (gpio_get_value(lt6911uxc->platform_data->reset_pin))
			gpio_set_value(lt6911uxc->platform_data->reset_pin, 0);

	mutex_lock(&lt6911uxc->mutex);
	if (lt6911uxc->streaming)
		lt6911uxc_stop_streaming(lt6911uxc);

	mutex_unlock(&lt6911uxc->mutex);
	dev_dbg(sd->dev, "suspend streaming...\n");
	return 0;
}

static int lt6911uxc_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lt6911uxc_state *lt6911uxc = to_state(sd);
	int ret;

	if (-1 != lt6911uxc->platform_data->reset_pin)
		if (!gpio_get_value(lt6911uxc->platform_data->reset_pin))
			gpio_set_value(lt6911uxc->platform_data->reset_pin, 1);

	usleep_range(200000, 205000);
	//recheck the current HDMI status in case changed
	lt6911uxc_check_status(lt6911uxc);

	mutex_lock(&lt6911uxc->mutex);
	if (lt6911uxc->streaming) {
		ret = lt6911uxc_start_streaming(lt6911uxc);
		if (ret) {
			lt6911uxc->streaming = false;
			lt6911uxc_stop_streaming(lt6911uxc);
			mutex_unlock(&lt6911uxc->mutex);
			return ret;
		}
	}
	mutex_unlock(&lt6911uxc->mutex);
	dev_dbg(sd->dev, "resume streaming...\n");
	return 0;
}

static const struct dev_pm_ops lt6911uxc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lt6911uxc_suspend, lt6911uxc_resume)
};

static const struct i2c_device_id lt6911uxc_id_table[] = {
	{ "lt6911uxc", 0 },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(i2c, lt6911uxc_id_table);

static struct i2c_driver lt6911uxc_i2c_driver = {
	.driver = {
		.name = "lt6911uxc",
		.pm = &lt6911uxc_pm_ops,
	},
	.probe_new = lt6911uxc_probe,
	.remove = lt6911uxc_remove,
	.id_table = lt6911uxc_id_table,
};

module_i2c_driver(lt6911uxc_i2c_driver);

MODULE_AUTHOR("Fu Wei <wei.a.fu@intel.com>");
MODULE_DESCRIPTION("lt6911uxc HDMI to MIPI Bridge Driver");
MODULE_LICENSE("GPL v2");
