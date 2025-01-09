// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022-2024 Intel Corporation.

#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>

/* FPS registers */
#define MASK_FMI_FREQ2				0x0FFFFF

#define LT6911UXC_CHIP_ID			0x1704
#define LT6911UXC_REG_CHIP_ID			CCI_REG16(0x8100)

#define REG_BKB0_A2_REG			CCI_REG8(0xB0A2)

#define REG_ENABLE_I2C				CCI_REG8(0x80EE)
#define REG_DISABLE_WD				CCI_REG8(0x8010)
#define REG_PIX_CLK				CCI_REG24(0x8750)
#define REG_BYTE_CLK				CCI_REG24(0x8548)
#define REG_INT_RESPOND			CCI_REG8(0x86A6)

#define REG_AD_HALF_PCLK			CCI_REG8(0x8540)

#define REG_H_TOTAL	CCI_REG16(0x867C)	/* horizontal half total pixel */
#define REG_V_TOTAL	CCI_REG16(0x867A)	/* vertical total lines */

#define REG_H_ACTIVE	CCI_REG16(0x8680)	/* horizontal half active pixel */
#define REG_V_ACTIVE	CCI_REG16(0x867E)	/* vertical active lines */

#define REG_MIPI_TX_CTRL	CCI_REG8(0x811D)
#define REG_MIPI_LANES		CCI_REG8(0x86A2)

#define REG_INT_HDMI			CCI_REG8(0x86A3)
#define INT_HDMI_DISCONNECT		0x88
#define INT_HDMI_STABLE		0x55


#define REG_INT_AUDIO		CCI_REG8(0x86A5)
#define INT_AUDIO_SR_HIGH	CCI_REG8(0x55)
#define INT_AUDIO_SR_LOW	CCI_REG8(0xAA)
#define REG_AUDIO_SR		CCI_REG8(0xB0AB)

#define INT_AUDIO_DISCONNECT		0x88

#define LT6911UXC_DEFAULT_WIDTH	3840
#define LT6911UXC_DEFAULT_HEIGHT	2160
#define LT6911_PAGE_CONTROL		0xff

#define LT6911UXC_CID_AUDIO_SAMPLING_RATE	(V4L2_CID_USER_BASE | 0x1007)
#define LT6911UXC_CID_AUDIO_PRESENT		(V4L2_CID_USER_BASE | 0x1008)

static const struct regmap_range_cfg lt6911uxc_ranges[] = {
	{
		.name = "register_range",
		.range_min =  0,
		.range_max = 0xffff,
		.selector_reg = LT6911_PAGE_CONTROL,
		.selector_mask = 0xff,
		.selector_shift = 0,
		.window_start = 0,
		.window_len = 0x100,
	},
};

static const struct regmap_config lt6911uxc_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xffff,
	.ranges = lt6911uxc_ranges,
	.num_ranges = ARRAY_SIZE(lt6911uxc_ranges),
};

struct lt6911uxc_mode {
	u32 width;
	u32 height;
	u32 code;
	s32 lanes;
	u32 fps;
	s64 link_freq;

	/* Audio sample rate*/
	u32 audio_sampling_rate;
};

struct lt6911uxc {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;

	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *audio_sampling_rate;
	struct v4l2_ctrl *audio_present;
	struct lt6911uxc_mode *cur_mode;
	struct regmap *regmap;
	unsigned int irq_pin_flags;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *irq_gpio;
	struct gpio_desc *detect_gpio;

	bool auxiliary_port;
};

static const struct v4l2_event lt6911uxc_ev_source_change = {
	.type = V4L2_EVENT_SOURCE_CHANGE,
	.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
};

static const struct v4l2_event lt6911uxc_ev_stream_end = {
	.type = V4L2_EVENT_EOS,
};

static inline struct lt6911uxc *to_lt6911uxc(struct v4l2_subdev *sd)
{
	return container_of(sd, struct lt6911uxc, sd);
}

static const struct v4l2_ctrl_config lt6911uxc_audio_sampling_rate = {
	.id = LT6911UXC_CID_AUDIO_SAMPLING_RATE,
	.name = "Audio Sampling Rate",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 32000,
	.max = 192000,
	.step = 100,
	.def = 48000,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

static const struct v4l2_ctrl_config lt6911uxc_audio_present = {
	.id = LT6911UXC_CID_AUDIO_PRESENT,
	.name = "Audio Present",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

static u64 get_pixel_rate(struct lt6911uxc *lt6911uxc)
{
	u64 pixel_rate;

	if (!lt6911uxc->cur_mode->width)
		return 0;

	pixel_rate = (u64)lt6911uxc->cur_mode->width *
			lt6911uxc->cur_mode->height *
			lt6911uxc->cur_mode->fps * 16;
	do_div(pixel_rate, lt6911uxc->cur_mode->lanes);

	return pixel_rate;
}

static int lt6911uxc_status_update(struct lt6911uxc *lt6911uxc)
{
	u64 int_event;
	u64 byte_clock;
	u64 tmds_clk;
	u64 pixel_clk, is_hdmi_2_0, fps, lanes;
	u64 htotal, vtotal, half_width, height;

	struct i2c_client *client = v4l2_get_subdevdata(&lt6911uxc->sd);

	cci_write(lt6911uxc->regmap, REG_ENABLE_I2C, 0x1, NULL);
	cci_write(lt6911uxc->regmap, REG_DISABLE_WD, 0x0, NULL);
	/* Read interrupt event */
	cci_read(lt6911uxc->regmap, REG_INT_HDMI, &int_event, NULL);
	switch (int_event) {
	case INT_HDMI_STABLE:
		dev_info(&client->dev, "Video signal stable\n");

		cci_write(lt6911uxc->regmap, REG_AD_HALF_PCLK, 0x1B, NULL);
		usleep_range(10000, 10100);

		cci_read(lt6911uxc->regmap, REG_BYTE_CLK, &byte_clock, NULL);
		byte_clock = (byte_clock & MASK_FMI_FREQ2) * 1000;

		cci_read(lt6911uxc->regmap, REG_PIX_CLK, &tmds_clk, NULL);
		tmds_clk = tmds_clk & MASK_FMI_FREQ2;

		cci_read(lt6911uxc->regmap, REG_BKB0_A2_REG, &is_hdmi_2_0, NULL);
		is_hdmi_2_0 = is_hdmi_2_0 & (1<<0) ? !0 : !!0;
		pixel_clk = (is_hdmi_2_0 ? 4 * tmds_clk : tmds_clk) * 1000;

		cci_read(lt6911uxc->regmap, REG_H_TOTAL, &htotal, NULL);
		cci_read(lt6911uxc->regmap, REG_V_TOTAL, &vtotal, NULL);
		cci_read(lt6911uxc->regmap, REG_H_ACTIVE, &half_width, NULL);
		cci_read(lt6911uxc->regmap, REG_V_ACTIVE, &height, NULL);
		cci_read(lt6911uxc->regmap, REG_MIPI_LANES, &lanes, NULL);

		if (htotal && vtotal)
			fps = div_u64(pixel_clk, htotal * 2 * vtotal);

		lt6911uxc->cur_mode->height = height;
		lt6911uxc->cur_mode->width = half_width * 2;
		lt6911uxc->cur_mode->fps = fps;
		lt6911uxc->cur_mode->code = MEDIA_BUS_FMT_UYVY8_1X16;
		lt6911uxc->cur_mode->lanes = lanes;
		lt6911uxc->cur_mode->link_freq = byte_clock;


		if (lt6911uxc->cur_mode->lanes == 8) {
			/* 4K60fps with 2 MIPI ports*/
			if (lt6911uxc->cur_mode->width >= 3840)
				lt6911uxc->cur_mode->width /= 2; /* YUV422 */

			lt6911uxc->cur_mode->lanes /= 2;
			lt6911uxc->cur_mode->link_freq /= 2;
		}
		v4l2_subdev_notify_event(&lt6911uxc->sd,
			&lt6911uxc_ev_source_change);

		dev_dbg(&client->dev,  "Pixel Clk:%llu, %lld\n",
			pixel_clk, lt6911uxc->cur_mode->link_freq);
		dev_dbg(&client->dev,
			"width:%u, height:%u, fps:%u, lanes: %d\n",
			lt6911uxc->cur_mode->width,
			lt6911uxc->cur_mode->height,
			lt6911uxc->cur_mode->fps,
			lt6911uxc->cur_mode->lanes);
		break;
	case INT_HDMI_DISCONNECT:
		cci_write(lt6911uxc->regmap, REG_MIPI_TX_CTRL, 0x0, NULL);
		lt6911uxc->cur_mode->height = 0;
		lt6911uxc->cur_mode->width = 0;
		lt6911uxc->cur_mode->fps = fps;

		lt6911uxc->cur_mode->code = MEDIA_BUS_FMT_UYVY8_1X16;

		lt6911uxc->cur_mode->lanes = 4;
		lt6911uxc->cur_mode->link_freq = 0;
		v4l2_subdev_notify_event(&lt6911uxc->sd,
			&lt6911uxc_ev_stream_end);

		dev_info(&client->dev, "Video signal disconnected\n");
		break;
	default:
		dev_dbg(&client->dev, "Unhandled video= 0x%02llX\n", int_event);
		cci_write(lt6911uxc->regmap, REG_ENABLE_I2C, 0x0, NULL);
		return  -ENOLINK;
	}
	cci_write(lt6911uxc->regmap, REG_ENABLE_I2C, 0x0, NULL);
	return 0;
}

static int lt6911uxc_init_controls(struct lt6911uxc *lt6911uxc)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	int ret;

	ctrl_hdlr = &lt6911uxc->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 8);
	if (ret)
		return ret;

	lt6911uxc->link_freq =
		v4l2_ctrl_new_int_menu(ctrl_hdlr, NULL, V4L2_CID_LINK_FREQ,
				       sizeof(lt6911uxc->cur_mode->link_freq),
				       0, &lt6911uxc->cur_mode->link_freq);

	lt6911uxc->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, NULL,
				V4L2_CID_PIXEL_RATE,
				get_pixel_rate(lt6911uxc),
				get_pixel_rate(lt6911uxc), 1,
				get_pixel_rate(lt6911uxc));

	/* custom v4l2 audio controls */
	lt6911uxc->audio_sampling_rate =
		v4l2_ctrl_new_custom(ctrl_hdlr,
				     &lt6911uxc_audio_sampling_rate,
				     NULL);

	lt6911uxc->audio_present =
		v4l2_ctrl_new_custom(ctrl_hdlr,
				     &lt6911uxc_audio_present,
				     NULL);
	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		goto hdlr_free;
	}

	lt6911uxc->sd.ctrl_handler = ctrl_hdlr;
	return 0;
hdlr_free:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	return ret;
}

static void lt6911uxc_update_pad_format(const struct lt6911uxc_mode *mode,
				     struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = mode->code;
	fmt->field = V4L2_FIELD_NONE;
}

static int lt6911uxc_start_streaming(struct lt6911uxc *lt6911uxc)
{
	struct i2c_client *client = v4l2_get_subdevdata(&lt6911uxc->sd);
	int ret;

	if (lt6911uxc->auxiliary_port == true)
		return 0;

	ret = pm_runtime_resume_and_get(&client->dev);
	if (ret < 0)
		return ret;

	ret = __v4l2_ctrl_handler_setup(lt6911uxc->sd.ctrl_handler);
	if (ret)
		goto err_rpm_put;
	cci_write(lt6911uxc->regmap, REG_ENABLE_I2C, 0x1, NULL);
	cci_write(lt6911uxc->regmap, REG_DISABLE_WD, 0x00, NULL);
	cci_write(lt6911uxc->regmap, REG_MIPI_TX_CTRL, 0xFB, NULL);
	cci_write(lt6911uxc->regmap, REG_ENABLE_I2C, 0x0, NULL);

	if (ret) {
		dev_err(&client->dev, "failed to start stream\n");
		goto err_rpm_put;
	}

	return 0;

err_rpm_put:
	pm_runtime_put(&client->dev);
	return ret;
}

static int lt6911uxc_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct lt6911uxc *lt6911uxc = to_lt6911uxc(sd);
	struct v4l2_subdev_state *state;
	int ret = 0;

	if (lt6911uxc->auxiliary_port == true)
		return 0;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	if (enable) {
		dev_dbg(sd->dev, "[%s()], start streaming.\n", __func__);
		ret = lt6911uxc_start_streaming(lt6911uxc);

	} else {
		dev_dbg(sd->dev, "[%s()], stop streaming.\n", __func__);
		cci_write(lt6911uxc->regmap, REG_ENABLE_I2C, 0x1, NULL);
		cci_write(lt6911uxc->regmap, REG_DISABLE_WD, 0x00, NULL);
		cci_write(lt6911uxc->regmap, REG_MIPI_TX_CTRL, 0x0, NULL);
		cci_write(lt6911uxc->regmap, REG_ENABLE_I2C, 0x0, NULL);
	}
	v4l2_subdev_unlock_state(state);

	return ret;
}

static int lt6911uxc_set_format(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_format *fmt)
{
	struct lt6911uxc *lt6911uxc = to_lt6911uxc(sd);
	u64 pixel_rate, link_freq;

	lt6911uxc_update_pad_format(lt6911uxc->cur_mode, &fmt->format);

	*v4l2_subdev_state_get_format(sd_state, fmt->pad) = fmt->format;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	pixel_rate = get_pixel_rate(lt6911uxc);

	__v4l2_ctrl_modify_range(lt6911uxc->pixel_rate, pixel_rate,
				 pixel_rate, 1, pixel_rate);

	link_freq = lt6911uxc->cur_mode->link_freq;

	__v4l2_ctrl_modify_range(lt6911uxc->link_freq, link_freq,
				 link_freq, 1, link_freq);

	return 0;
}

static int lt6911uxc_get_format(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_format *fmt)
{
	struct lt6911uxc *lt6911uxc = to_lt6911uxc(sd);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt->format = *v4l2_subdev_state_get_format(sd_state, fmt->pad);
	else
		lt6911uxc_update_pad_format(lt6911uxc->cur_mode, &fmt->format);

	return 0;
}

static int lt6911uxc_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_mbus_code_enum *code)
{
	struct lt6911uxc *lt6911uxc = to_lt6911uxc(sd);

	code->code = lt6911uxc->cur_mode->code;
	return 0;
}

static int lt6911uxc_enum_frame_size(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_frame_size_enum *fse)
{
	struct lt6911uxc *lt6911uxc = to_lt6911uxc(sd);

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
	struct lt6911uxc *lt6911uxc = to_lt6911uxc(sd);

	fie->interval.numerator = 1;
	fie->interval.denominator = lt6911uxc->cur_mode->fps;

	return 0;
}

static int lt6911uxc_init_state(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state)
{
	struct v4l2_subdev_format fmt = {
		.which = sd_state ? V4L2_SUBDEV_FORMAT_TRY
		: V4L2_SUBDEV_FORMAT_ACTIVE,
	};

	return lt6911uxc_set_format(sd, sd_state, &fmt);
}

static const struct v4l2_subdev_video_ops lt6911uxc_video_ops = {
	.s_stream = lt6911uxc_s_stream,
};

static const struct v4l2_subdev_pad_ops lt6911uxc_pad_ops = {
	.set_fmt = lt6911uxc_set_format,
	.get_fmt = lt6911uxc_get_format,
	.enum_mbus_code = lt6911uxc_enum_mbus_code,
	.enum_frame_size = lt6911uxc_enum_frame_size,
	.enum_frame_interval = lt6911uxc_enum_frame_interval,
};

static const struct v4l2_subdev_core_ops lt6911uxc_subdev_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
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
	.init_state = lt6911uxc_init_state,
};

static void lt6911uxc_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lt6911uxc *lt6911uxc = to_lt6911uxc(sd);

	if (!lt6911uxc->auxiliary_port) {
		v4l2_async_unregister_subdev(&lt6911uxc->sd);
		v4l2_subdev_cleanup(sd);
		media_entity_cleanup(&lt6911uxc->sd.entity);
		v4l2_ctrl_handler_free(&lt6911uxc->ctrl_handler);
		pm_runtime_disable(&client->dev);
		pm_runtime_set_suspended(&client->dev);
	}
}

static int lt6911uxc_identify_module(struct lt6911uxc *lt6911uxc,
				      struct device *dev)
{
	u64 val;
	int ret = 0;

	cci_write(lt6911uxc->regmap, REG_ENABLE_I2C, 0x1, NULL);
	cci_read(lt6911uxc->regmap, LT6911UXC_REG_CHIP_ID, &val, NULL);
	cci_write(lt6911uxc->regmap, REG_ENABLE_I2C, 0x0, NULL);
	if (ret)
		return dev_err_probe(dev, ret, "fail to read chip id\n");

	if (val != LT6911UXC_CHIP_ID) {
		return dev_err_probe(dev, -ENXIO, "chip id mismatch: %x!=%x\n",
				     LT6911UXC_CHIP_ID, (u16)val);
	}

	return 0;
}

static int lt6911uxc_parse_gpio(struct lt6911uxc *lt6911uxc, struct device *dev)
{
	lt6911uxc->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_IN);
	if (IS_ERR(lt6911uxc->reset_gpio)) {
		lt6911uxc->auxiliary_port = true;
		return dev_err_probe(dev, PTR_ERR(lt6911uxc->reset_gpio),
				     "failed to get reset gpio\n");
	}

	lt6911uxc->irq_gpio = devm_gpiod_get(dev, "readystat", GPIOD_IN);
	if (IS_ERR(lt6911uxc->irq_gpio))
		return dev_err_probe(dev, PTR_ERR(lt6911uxc->irq_gpio),
				     "failed to get ready_stat gpio\n");

	lt6911uxc->detect_gpio = devm_gpiod_get(dev, "hdmidetect",
						GPIOD_OUT_HIGH);
	if (IS_ERR(lt6911uxc->detect_gpio))
		return dev_err_probe(dev, PTR_ERR(lt6911uxc->detect_gpio),
				     "failed to get hdmi_detect gpio\n");

	return 0;
}

static irqreturn_t lt6911uxc_threaded_irq_fn(int irq, void *dev_id)
{
	struct v4l2_subdev *sd = dev_id;
	struct lt6911uxc *lt6911uxc = to_lt6911uxc(sd);
	struct v4l2_subdev_state *state;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	lt6911uxc_status_update(lt6911uxc);
	v4l2_subdev_unlock_state(state);

	return IRQ_HANDLED;
}

static int lt6911uxc_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct lt6911uxc *lt6911uxc;
	int ret;

	lt6911uxc = devm_kzalloc(&client->dev, sizeof(struct lt6911uxc),
			     GFP_KERNEL);
	if (!lt6911uxc)
		return -ENOMEM;

	lt6911uxc->cur_mode = devm_kzalloc(&client->dev,
						sizeof(struct lt6911uxc_mode),
						GFP_KERNEL);
	if (!lt6911uxc->cur_mode)
		return -ENOMEM;

	lt6911uxc->regmap = devm_regmap_init_i2c(client,
						 &lt6911uxc_regmap_config);

	if (IS_ERR(lt6911uxc->regmap))
		return dev_err_probe(&client->dev, PTR_ERR(lt6911uxc->regmap),
				     "failed to init CCI\n");

	v4l2_i2c_subdev_init(&lt6911uxc->sd, client, &lt6911uxc_subdev_ops);

	ret = lt6911uxc_parse_gpio(lt6911uxc, &client->dev);
	if (ret) {
		dev_err(&client->dev, "failed to get GPIO control: %d\n", ret);
		return ret;
	}

	usleep_range(10000, 10500);

	ret = lt6911uxc_identify_module(lt6911uxc, dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to find chip\n");

	ret = lt6911uxc_init_controls(lt6911uxc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to init control\n");

	lt6911uxc->sd.dev = &client->dev;
	lt6911uxc->sd.internal_ops = &lt6911uxc_internal_ops;
	lt6911uxc->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			       V4L2_SUBDEV_FL_HAS_EVENTS;
	lt6911uxc->sd.entity.ops = &lt6911uxc_subdev_entity_ops;
	lt6911uxc->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	lt6911uxc->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&lt6911uxc->sd.entity, 1, &lt6911uxc->pad);
	if (ret) {
		dev_err(&client->dev, "Init entity pads failed:%d\n", ret);
		goto err_v4l2_ctrl_handler_free;
	}

	/*
	 * Device is already turned on by i2c-core with ACPI domain PM.
	 * Enable runtime PM and turn off the device.
	 */
	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	ret = v4l2_subdev_init_finalize(&lt6911uxc->sd);
	if (ret) {
		dev_err(dev, "failed to init v4l2 subdev: %d\n", ret);
		goto err_media_entity_cleanup;
	}

	/* Setting irq */
	lt6911uxc->irq_pin_flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
				   IRQF_ONESHOT;

	ret = request_threaded_irq(gpiod_to_irq(lt6911uxc->irq_gpio), NULL,
				   lt6911uxc_threaded_irq_fn,
				   lt6911uxc->irq_pin_flags, NULL, lt6911uxc);
	if (ret) {
		dev_err(dev, "failed to request IRQ: %d\n", ret);
		goto err_subdev_cleanup;
	}

	ret = v4l2_async_register_subdev_sensor(&lt6911uxc->sd);
	if (ret) {
		dev_err(dev, "failed to register V4L2 subdev: %d\n", ret);
		goto err_free_irq;
	}

	dev_info(&client->dev, "%s Probe Succeeded", lt6911uxc->sd.name);

	return 0;

err_free_irq:
	free_irq(gpiod_to_irq(lt6911uxc->irq_gpio), lt6911uxc);

err_subdev_cleanup:
	v4l2_subdev_cleanup(&lt6911uxc->sd);

err_media_entity_cleanup:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	media_entity_cleanup(&lt6911uxc->sd.entity);

err_v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(lt6911uxc->sd.ctrl_handler);

	return ret;
}

static const struct acpi_device_id lt6911uxc_acpi_ids[] = {
	{ "INTC10B1"},
	{}
};
MODULE_DEVICE_TABLE(acpi, lt6911uxc_acpi_ids);

static struct i2c_driver lt6911uxc_i2c_driver = {
	.driver = {
		.name = "lt6911uxc",
		.acpi_match_table = ACPI_PTR(lt6911uxc_acpi_ids),
	},
	.probe = lt6911uxc_probe,
	.remove = lt6911uxc_remove,
};

module_i2c_driver(lt6911uxc_i2c_driver);

MODULE_AUTHOR("Zou, Xiaohong.zou@intel.com");
MODULE_DESCRIPTION("lt6911uxc HDMI to MIPI Bridge Driver");
MODULE_LICENSE("GPL");
