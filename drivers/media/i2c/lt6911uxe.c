// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2023 - 2024 Intel Corporation.

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

#define LT6911UXE_CHIP_ID		0x2102
#define REG_CHIP_ID_H			CCI_REG8(0xe100)
#define REG_CHIP_ID_L			CCI_REG8(0xe101)

#define REG_ENABLE_I2C			CCI_REG8(0xe0ee)
#define REG_PIX_CLK			CCI_REG24(0xe085)
#define REG_BYTE_CLK			CCI_REG24(0xe092)
#define REG_H_TOTAL			CCI_REG16(0xe088)
#define REG_V_TOTAL			CCI_REG16(0xe08a)
#define REG_HALF_H_ACTIVE		CCI_REG16(0xe08c)
#define REG_V_ACTIVE			CCI_REG16(0xe08e)
#define REG_MIPI_TX_CTRL		CCI_REG8(0xe0b0)
#define REG_MIPI_LANES			CCI_REG8(0xe095)

/* Interrupts */
#define REG_INT_HDMI			CCI_REG8(0xe084)
#define INT_HDMI_DISCONNECT		0x0
#define INT_HDMI_STABLE			0x1
#define REG_INT_AUDIO			CCI_REG8(0x86a5)
#define REG_AUDIO_FS_H			CCI_REG8(0xe090)
#define REG_AUDIO_FS_L			CCI_REG8(0xe091)
#define AUDIO_SR_HIGH			0x55
#define AUDIO_SR_LOW			0xaa
#define INT_AUDIO_DISCONNECT		0x3

#define LT6911UXE_DEFAULT_WIDTH		3840
#define LT6911UXE_DEFAULT_HEIGHT	2160
#define LT9611_PAGE_CONTROL		0xff

#define LT6911UXE_CID_AUDIO_SAMPLING_RATE	(V4L2_CID_USER_BASE | 0x1001)
#define LT6911UXE_CID_AUDIO_PRESENT		(V4L2_CID_USER_BASE | 0x1002)

static const struct regmap_range_cfg lt9611uxe_ranges[] = {
	{
		.name = "register_range",
		.range_min =  0,
		.range_max = 0xffff,
		.selector_reg = LT9611_PAGE_CONTROL,
		.selector_mask = 0xff,
		.selector_shift = 0,
		.window_start = 0,
		.window_len = 0x100,
	},
};

static const struct regmap_config lt9611uxe_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xffff,
	.ranges = lt9611uxe_ranges,
	.num_ranges = ARRAY_SIZE(lt9611uxe_ranges),
};

struct lt6911uxe_mode {
	u32 width;
	u32 height;
	u32 code;
	s32 lanes;
	u32 fps;
	s64 link_freq;
	/* Audio sample rate*/
	u32 sample_rate;
};

struct lt6911uxe {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *sampling_rate;
	struct v4l2_ctrl *audio_present;

	struct lt6911uxe_mode *cur_mode;
	struct regmap *regmap;
	unsigned int irq_pin_flags;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *irq_gpio;
	struct gpio_desc *detect_gpio;
};

static const struct v4l2_event lt6911uxe_ev_source_change = {
	.type = V4L2_EVENT_SOURCE_CHANGE,
	.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
};

static const struct v4l2_event lt6911uxe_ev_stream_end = {
	.type = V4L2_EVENT_EOS,
};

static inline struct lt6911uxe *to_lt6911uxe(struct v4l2_subdev *sd)
{
	return container_of(sd, struct lt6911uxe, sd);
}

static const struct v4l2_ctrl_config lt6911uxe_audio_sampling_rate = {
	.id = LT6911UXE_CID_AUDIO_SAMPLING_RATE,
	.name = "Audio Sampling Rate",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 32000,
	.max = 192000,
	.step = 100,
	.def = 48000,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

static const struct v4l2_ctrl_config lt6911uxe_audio_present = {
	.id = LT6911UXE_CID_AUDIO_PRESENT,
	.name = "Audio Present",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

static u64 get_pixel_rate(struct lt6911uxe *lt6911uxe)
{
	u64 pixel_rate;

	if (!lt6911uxe->cur_mode->width)
		return 0;

	pixel_rate = (u64)lt6911uxe->cur_mode->width *
		     lt6911uxe->cur_mode->height *
		     lt6911uxe->cur_mode->fps * 16;
	do_div(pixel_rate, lt6911uxe->cur_mode->lanes);

	return pixel_rate;
}

static int lt6911uxe_get_audio_sampling_rate(struct lt6911uxe *lt6911uxe)
{
	int audio_fs, idx;
	u64 fs_h, fs_l;
	static const int eps = 1500;
	static const int rates_default[] = {
		32000, 44100, 48000, 88200, 96000, 176400, 192000
	};

	cci_read(lt6911uxe->regmap, REG_AUDIO_FS_H, &fs_h, NULL);
	cci_read(lt6911uxe->regmap, REG_AUDIO_FS_H, &fs_l, NULL);
	audio_fs = ((fs_h << 8) | fs_l);

	/* audio_fs is an approximation of sample rate - search nearest */
	for (idx = 0; idx < ARRAY_SIZE(rates_default); ++idx) {
		if ((rates_default[idx] - eps < audio_fs) &&
		    (rates_default[idx] + eps > audio_fs))
			return rates_default[idx];
	}

	return 0;
}

static int lt6911uxe_video_status_update(struct lt6911uxe *lt6911uxe)
{
	u64 int_event;
	u64 byte_clock, pixel_clk, fps, lanes;
	u64 htotal, vtotal, half_width, height;

	/* Read interrupt event */
	cci_read(lt6911uxe->regmap, REG_INT_HDMI, &int_event, NULL);
	switch (int_event) {
	case INT_HDMI_STABLE:
		cci_read(lt6911uxe->regmap, REG_BYTE_CLK, &byte_clock, NULL);
		byte_clock *= 1000;
		cci_read(lt6911uxe->regmap, REG_PIX_CLK, &pixel_clk, NULL);
		pixel_clk *= 1000;

		cci_read(lt6911uxe->regmap, REG_H_TOTAL, &htotal, NULL);
		cci_read(lt6911uxe->regmap, REG_V_TOTAL, &vtotal, NULL);
		cci_read(lt6911uxe->regmap, REG_HALF_H_ACTIVE,
			 &half_width, NULL);
		cci_read(lt6911uxe->regmap, REG_V_ACTIVE, &height, NULL);
		cci_read(lt6911uxe->regmap, REG_MIPI_LANES, &lanes, NULL);

		if (htotal && vtotal)
			fps = div_u64(pixel_clk, htotal * vtotal);

		lt6911uxe->cur_mode->height = height;
		lt6911uxe->cur_mode->width = half_width * 2;
		lt6911uxe->cur_mode->fps = fps;
		lt6911uxe->cur_mode->code = MEDIA_BUS_FMT_UYVY8_1X16;
		lt6911uxe->cur_mode->lanes = lanes;
		/* Mipi Clock Rate = ByteClock × 4, defined in lt6911uxe spec */
		lt6911uxe->cur_mode->link_freq = byte_clock * 4;
		v4l2_subdev_notify_event(&lt6911uxe->sd,
					 &lt6911uxe_ev_source_change);
		break;
	case INT_HDMI_DISCONNECT:
		cci_write(lt6911uxe->regmap, REG_MIPI_TX_CTRL, 0x0, NULL);
		lt6911uxe->cur_mode->height = 0;
		lt6911uxe->cur_mode->width = 0;
		lt6911uxe->cur_mode->fps = fps;
		lt6911uxe->cur_mode->code = MEDIA_BUS_FMT_UYVY8_1X16;
		lt6911uxe->cur_mode->lanes = 4;
		lt6911uxe->cur_mode->link_freq = 0;
		v4l2_subdev_notify_event(&lt6911uxe->sd,
					 &lt6911uxe_ev_stream_end);
		break;
	default:
		return  -ENOLINK;
	}

	return 0;
}

static void lt6911uxe_audio_status_update(struct lt6911uxe *lt6911uxe)
{
	u64 int_event;
	int audio_fs;

	/* Read interrupt event */
	cci_read(lt6911uxe->regmap, REG_INT_AUDIO, &int_event, NULL);
	switch (int_event) {
	case AUDIO_SR_HIGH:
	case AUDIO_SR_LOW:
		audio_fs = lt6911uxe_get_audio_sampling_rate(lt6911uxe);
		lt6911uxe->cur_mode->sample_rate = audio_fs;
		break;

	case INT_AUDIO_DISCONNECT:
	default:
		/* use the default value for avoiding problem */
		lt6911uxe->cur_mode->sample_rate = 0;
		break;
	}

}

static int lt6911uxe_init_controls(struct lt6911uxe *lt6911uxe)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	int ret;

	ctrl_hdlr = &lt6911uxe->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 8);
	if (ret)
		return ret;

	lt6911uxe->link_freq =
		v4l2_ctrl_new_int_menu(ctrl_hdlr, NULL, V4L2_CID_LINK_FREQ,
				       sizeof(lt6911uxe->cur_mode->link_freq),
				       0, &lt6911uxe->cur_mode->link_freq);

	lt6911uxe->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, NULL,
						  V4L2_CID_PIXEL_RATE,
						  get_pixel_rate(lt6911uxe),
						  get_pixel_rate(lt6911uxe), 1,
						  get_pixel_rate(lt6911uxe));

	/* custom v4l2 audio controls */
	lt6911uxe->sampling_rate =
		v4l2_ctrl_new_custom(ctrl_hdlr,
				     &lt6911uxe_audio_sampling_rate,
				     NULL);

	lt6911uxe->audio_present =
		v4l2_ctrl_new_custom(ctrl_hdlr,
				     &lt6911uxe_audio_present,
				     NULL);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		goto hdlr_free;
	}
	lt6911uxe->sd.ctrl_handler = ctrl_hdlr;

	return 0;
hdlr_free:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	return ret;
}

static void lt6911uxe_update_pad_format(const struct lt6911uxe_mode *mode,
					struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = mode->code;
	fmt->field = V4L2_FIELD_NONE;
}

static int lt6911uxe_start_streaming(struct lt6911uxe *lt6911uxe)
{
	struct i2c_client *client = v4l2_get_subdevdata(&lt6911uxe->sd);
	int ret;

	ret = pm_runtime_resume_and_get(&client->dev);
	if (ret < 0)
		return ret;

	ret = __v4l2_ctrl_handler_setup(lt6911uxe->sd.ctrl_handler);
	if (ret)
		goto err_rpm_put;

	cci_write(lt6911uxe->regmap, REG_MIPI_TX_CTRL, 0x1, &ret);
	if (ret) {
		dev_err(&client->dev, "failed to start stream\n");
		goto err_rpm_put;
	}

	return 0;

err_rpm_put:
	pm_runtime_put(&client->dev);
	return ret;
}

static int lt6911uxe_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct lt6911uxe *lt6911uxe = to_lt6911uxe(sd);
	struct v4l2_subdev_state *state;
	int ret = 0;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	if (enable)
		ret = lt6911uxe_start_streaming(lt6911uxe);
	else
		cci_write(lt6911uxe->regmap, REG_MIPI_TX_CTRL, 0x0, NULL);

	v4l2_subdev_unlock_state(state);

	return ret;
}

static int lt6911uxe_set_format(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_format *fmt)
{
	struct lt6911uxe *lt6911uxe = to_lt6911uxe(sd);
	u64 pixel_rate;

	lt6911uxe_video_status_update(lt6911uxe);
	lt6911uxe_audio_status_update(lt6911uxe);
	lt6911uxe_update_pad_format(lt6911uxe->cur_mode, &fmt->format);
	*v4l2_subdev_state_get_format(sd_state, fmt->pad) = fmt->format;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	pixel_rate = get_pixel_rate(lt6911uxe);
	__v4l2_ctrl_modify_range(lt6911uxe->pixel_rate, pixel_rate,
				 pixel_rate, 1, pixel_rate);
	__v4l2_ctrl_s_ctrl(lt6911uxe->audio_present,
			   (lt6911uxe->cur_mode->sample_rate != 0));

	if (lt6911uxe->cur_mode->sample_rate)
		__v4l2_ctrl_s_ctrl(lt6911uxe->sampling_rate,
				   lt6911uxe->cur_mode->sample_rate);

	return 0;
}

static int lt6911uxe_enum_mbus_code(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *sd_state,
				    struct v4l2_subdev_mbus_code_enum *code)
{
	struct lt6911uxe *lt6911uxe = to_lt6911uxe(sd);

	code->code = lt6911uxe->cur_mode->code;

	return 0;
}

static int lt6911uxe_enum_frame_size(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *sd_state,
				     struct v4l2_subdev_frame_size_enum *fse)
{
	struct lt6911uxe *lt6911uxe = to_lt6911uxe(sd);

	fse->min_width = lt6911uxe->cur_mode->width;
	fse->max_width = fse->min_width;
	fse->min_height = lt6911uxe->cur_mode->height;
	fse->max_height = fse->min_height;

	return 0;
}

static int lt6911uxe_enum_frame_interval(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *sd_state,
		struct v4l2_subdev_frame_interval_enum *fie)
{
	struct lt6911uxe *lt6911uxe = to_lt6911uxe(sd);

	fie->interval.numerator = 1;
	fie->interval.denominator = lt6911uxe->cur_mode->fps;

	return 0;
}

static int lt6911uxe_init_state(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.code = MEDIA_BUS_FMT_UYVY8_1X16,
			.width = LT6911UXE_DEFAULT_WIDTH,
			.height = LT6911UXE_DEFAULT_HEIGHT,
		},
	};

	lt6911uxe_set_format(sd, sd_state, &fmt);

	return 0;
}

static const struct v4l2_subdev_video_ops lt6911uxe_video_ops = {
	.s_stream = lt6911uxe_s_stream,
};

static const struct v4l2_subdev_pad_ops lt6911uxe_pad_ops = {
	.set_fmt = lt6911uxe_set_format,
	.get_fmt = v4l2_subdev_get_fmt,
	.enum_mbus_code = lt6911uxe_enum_mbus_code,
	.enum_frame_size = lt6911uxe_enum_frame_size,
	.enum_frame_interval = lt6911uxe_enum_frame_interval,
};

static const struct v4l2_subdev_core_ops lt6911uxe_subdev_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_ops lt6911uxe_subdev_ops = {
	.core = &lt6911uxe_subdev_core_ops,
	.video = &lt6911uxe_video_ops,
	.pad = &lt6911uxe_pad_ops,
};

static const struct media_entity_operations lt6911uxe_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops lt6911uxe_internal_ops = {
	.init_state = lt6911uxe_init_state,
};

static void lt6911uxe_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lt6911uxe *lt6911uxe = to_lt6911uxe(sd);

	v4l2_async_unregister_subdev(&lt6911uxe->sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&lt6911uxe->sd.entity);
	v4l2_ctrl_handler_free(&lt6911uxe->ctrl_handler);
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static int lt6911uxe_identify_module(struct lt6911uxe *lt6911uxe)
{
	struct i2c_client *client = v4l2_get_subdevdata(&lt6911uxe->sd);
	u64 val_h, val_l;
	u16 val;

	/* Chip ID should be confirmed when the I2C slave is active */
	cci_write(lt6911uxe->regmap, REG_ENABLE_I2C, 0x1, NULL);
	cci_read(lt6911uxe->regmap, REG_CHIP_ID_H, &val_h, NULL);
	cci_read(lt6911uxe->regmap, REG_CHIP_ID_L, &val_l, NULL);
	cci_write(lt6911uxe->regmap, REG_ENABLE_I2C, 0x0, NULL);

	val = ((val_h << 8) | val_l);

	if (val != LT6911UXE_CHIP_ID) {
		dev_err(&client->dev, "chip id mismatch: %x!=%x\n",
			LT6911UXE_CHIP_ID, val);
		return -ENXIO;
	}

	return 0;
}

static int lt6911uxe_parse_gpio(struct lt6911uxe *lt6911uxe, struct device *dev)
{
	lt6911uxe->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_IN);
	if (IS_ERR(lt6911uxe->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(lt6911uxe->reset_gpio),
				     "failed to get reset gpio\n");

	lt6911uxe->irq_gpio = devm_gpiod_get(dev, "readystat", GPIOD_IN);
	if (IS_ERR(lt6911uxe->irq_gpio))
		return dev_err_probe(dev, PTR_ERR(lt6911uxe->irq_gpio),
				     "failed to get ready_stat gpio\n");

	lt6911uxe->detect_gpio = devm_gpiod_get(dev, "hdmidetect",
						GPIOD_OUT_HIGH);
	if (IS_ERR(lt6911uxe->detect_gpio))
		return dev_err_probe(dev, PTR_ERR(lt6911uxe->detect_gpio),
				     "failed to get hdmi_detect gpio\n");

	return 0;
}

static irqreturn_t lt6911uxe_threaded_irq_fn(int irq, void *dev_id)
{
	struct v4l2_subdev *sd = dev_id;
	struct lt6911uxe *lt6911uxe;

	if (!sd)
		return IRQ_NONE;

	lt6911uxe = to_lt6911uxe(sd);
	if (!lt6911uxe) {
		dev_err(sd->dev, "Invalid lt6911uxe state argument!\n");
		return IRQ_NONE;
	}

	lt6911uxe_video_status_update(lt6911uxe);
	lt6911uxe_audio_status_update(lt6911uxe);

	return IRQ_HANDLED;
}

static void lt6911uxe_reset(struct lt6911uxe *lt6911uxe, int power)
{
	if (lt6911uxe->reset_gpio || lt6911uxe->irq_gpio)
		return;

	gpiod_set_value_cansleep(lt6911uxe->reset_gpio, power);
	gpiod_set_value_cansleep(lt6911uxe->irq_gpio, power);
}

static int lt6911uxe_probe(struct i2c_client *client)
{
	struct lt6911uxe *lt6911uxe;
	int ret;

	lt6911uxe = devm_kzalloc(&client->dev, sizeof(struct lt6911uxe),
				 GFP_KERNEL);
	if (!lt6911uxe)
		return -ENOMEM;

	lt6911uxe->cur_mode = devm_kzalloc(&client->dev,
					   sizeof(struct lt6911uxe_mode),
					   GFP_KERNEL);
	if (!lt6911uxe->cur_mode)
		return -ENOMEM;

	lt6911uxe->regmap = devm_regmap_init_i2c(client,
						 &lt9611uxe_regmap_config);
	if (IS_ERR(lt6911uxe->regmap))
		return dev_err_probe(&client->dev, PTR_ERR(lt6911uxe->regmap),
				     "failed to init CCI\n");

	v4l2_i2c_subdev_init(&lt6911uxe->sd, client, &lt6911uxe_subdev_ops);

	ret = lt6911uxe_parse_gpio(lt6911uxe, &client->dev);
	if (ret) {
		dev_err(&client->dev, "failed to get GPIO control: %d\n", ret);
		return ret;
	}

	lt6911uxe_reset(lt6911uxe, 1);
	usleep_range(10000, 10500);

	ret = lt6911uxe_identify_module(lt6911uxe);
	if (ret) {
		dev_err(&client->dev, "failed to find chip: %d\n", ret);
		return ret;
	}

	/* Setting irq */
	lt6911uxe->irq_pin_flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
				   IRQF_ONESHOT;

	ret = devm_request_threaded_irq(&client->dev,
					gpiod_to_irq(lt6911uxe->irq_gpio),
					NULL, lt6911uxe_threaded_irq_fn,
					lt6911uxe->irq_pin_flags,
					"irq_gpio", lt6911uxe);
	if (ret) {
		dev_err(&client->dev, "IRQ request failed! ret: %d\n", ret);
		goto probe_error_v4l2_ctrl_handler_free;
	}

	ret = lt6911uxe_init_controls(lt6911uxe);
	if (ret) {
		dev_info(&client->dev, "Could not init control %d!\n", ret);
		goto probe_error_v4l2_ctrl_handler_free;
	}

	lt6911uxe->sd.state_lock = lt6911uxe->ctrl_handler.lock;
	lt6911uxe->sd.dev = &client->dev;
	lt6911uxe->sd.internal_ops = &lt6911uxe_internal_ops;
	lt6911uxe->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			       V4L2_SUBDEV_FL_HAS_EVENTS;
	lt6911uxe->sd.entity.ops = &lt6911uxe_subdev_entity_ops;
	lt6911uxe->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	lt6911uxe->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&lt6911uxe->sd.entity, 1, &lt6911uxe->pad);
	if (ret) {
		dev_err(&client->dev, "Init entity pads failed: %d\n", ret);
		goto probe_error_v4l2_ctrl_handler_free;
	}

	ret = v4l2_subdev_init_finalize(&lt6911uxe->sd);
	if (ret < 0) {
		dev_err(&client->dev, "v4l2 subdev init error: %d\n", ret);
		goto probe_error_media_entity_cleanup;
	}

	/*
	 * Device is already turned on by i2c-core with ACPI domain PM.
	 * Enable runtime PM and turn off the device.
	 */
	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	ret = v4l2_async_register_subdev_sensor(&lt6911uxe->sd);
	if (ret < 0) {
		dev_err(&client->dev, "failed to register V4L2 subdev: %d\n",
			ret);
		goto rpm;
	}

	return 0;

rpm:
	pm_runtime_disable(&client->dev);
	v4l2_subdev_cleanup(&lt6911uxe->sd);

probe_error_media_entity_cleanup:
	media_entity_cleanup(&lt6911uxe->sd.entity);

probe_error_v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(lt6911uxe->sd.ctrl_handler);
	return ret;
}

static const struct acpi_device_id lt6911uxe_acpi_ids[] = {
	{"INTC10C5"},
	{}
};
MODULE_DEVICE_TABLE(acpi, lt6911uxe_acpi_ids);

static struct i2c_driver lt6911uxe_i2c_driver = {
	.driver = {
		.name = "lt6911uxe",
		.acpi_match_table = ACPI_PTR(lt6911uxe_acpi_ids),
	},
	.probe = lt6911uxe_probe,
	.remove = lt6911uxe_remove,
};

module_i2c_driver(lt6911uxe_i2c_driver);

MODULE_AUTHOR("Yan Dongcheng <dongcheng.yan@intel.com>");
MODULE_DESCRIPTION("Lontium lt6911uxe HDMI to MIPI Bridge Driver");
MODULE_LICENSE("GPL");
