// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2023 - 2025 Intel Corporation.

#include <linux/acpi.h>
#include <linux/delay.h>
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

#define LT6911GXD_CHIP_ID		0x2204
#define REG_CHIP_ID			CCI_REG16(0xe100)

#define REG_ENABLE_I2C			CCI_REG8(0xe0ee)

#define REG_PIX_CLK			CCI_REG24(0xe085)
#define REG_BYTE_CLK			CCI_REG24(0xe092)
#define REG_H_TOTAL			CCI_REG16(0xe088)
#define REG_V_TOTAL			CCI_REG16(0xe08a)
#define REG_HALF_H_ACTIVE		CCI_REG16(0xe08c)
#define REG_V_ACTIVE			CCI_REG16(0xe08e)
#define REG_MIPI_FORMAT			CCI_REG8(0xf988)
#define REG_MIPI_TX_CTRL		CCI_REG8(0xe0b0)

/* Interrupts */
#define REG_INT_HDMI			CCI_REG8(0xe084)
#define INT_VIDEO_DISAPPEAR		0x0
#define INT_VIDEO_READY			0x1

#define LT6911GXD_DEFAULT_WIDTH		3840
#define LT6911GXD_DEFAULT_HEIGHT	2160
#define LT6911GXD_DEFAULT_LANES		2
#define LT6911GXD_DEFAULT_FPS		30
#define LT6911GXD_MAX_LINK_FREQ		297000000
#define LT6911GXD_PAGE_CONTROL		0xff
#define BPP_MODE_BIT			4
#define YUV16_BIT			2

static const struct regmap_range_cfg lt6911gxd_ranges[] = {
	{
		.name = "register_range",
		.range_min =  0,
		.range_max = 0xffff,
		.selector_reg = LT6911GXD_PAGE_CONTROL,
		.selector_mask = 0xff,
		.selector_shift = 0,
		.window_start = 0,
		.window_len = 0x100,
	},
};

static const struct regmap_config lt6911gxd_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xffff,
	.ranges = lt6911gxd_ranges,
	.num_ranges = ARRAY_SIZE(lt6911gxd_ranges),
};

struct lt6911gxd_mode {
	u32 width;
	u32 height;
	u32 code;
	u32 fps;
	u32 lanes;
	s64 link_freq;
};

struct lt6911gxd {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;

	struct lt6911gxd_mode cur_mode;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *irq_gpio;
};

static const struct v4l2_event lt6911gxd_ev_source_change = {
	.type = V4L2_EVENT_SOURCE_CHANGE,
	.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
};

static const struct v4l2_event lt6911gxd_ev_stream_end = {
	.type = V4L2_EVENT_EOS,
};

static inline struct lt6911gxd *to_lt6911gxd(struct v4l2_subdev *sd)
{
	return container_of(sd, struct lt6911gxd, sd);
}

static const struct lt6911gxd_mode default_mode = {
	.width = LT6911GXD_DEFAULT_WIDTH,
	.height = LT6911GXD_DEFAULT_HEIGHT,
	.code = MEDIA_BUS_FMT_UYVY8_1X16,
	.fps = LT6911GXD_DEFAULT_FPS,
	.lanes = LT6911GXD_DEFAULT_LANES,
	.link_freq = LT6911GXD_MAX_LINK_FREQ,
};

static s64 get_pixel_rate(struct lt6911gxd *lt6911gxd)
{
	s64 pixel_rate;

	pixel_rate = (s64)lt6911gxd->cur_mode.width *
		     lt6911gxd->cur_mode.height *
		     lt6911gxd->cur_mode.fps * 16;
	do_div(pixel_rate, lt6911gxd->cur_mode.lanes);

	return pixel_rate;
}

static int lt6911gxd_status_update(struct lt6911gxd *lt6911gxd)
{
	struct i2c_client *client = v4l2_get_subdevdata(&lt6911gxd->sd);
	u64 int_event;
	u64 byte_clk, pixel_clk, fps;
	u64 htotal, vtotal, width, height;
	int timeout_cnt = 3;
	int ret = 0;

	/* Read interrupt event */
	cci_read(lt6911gxd->regmap, REG_INT_HDMI, &int_event, &ret);
	while (ret && timeout_cnt--) {
		ret = cci_read(lt6911gxd->regmap, REG_INT_HDMI,
				&int_event, NULL);
	}

	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to read interrupt event\n");

	/* TODO: add audio ready and disappear type */
	switch (int_event) {
	case INT_VIDEO_READY:
		cci_read(lt6911gxd->regmap, REG_BYTE_CLK, &byte_clk, &ret);
		byte_clk *= 1000;
		cci_read(lt6911gxd->regmap, REG_PIX_CLK, &pixel_clk, &ret);
		pixel_clk *= 1000;

		if (ret || byte_clk == 0 || pixel_clk == 0) {
			dev_err(&client->dev,
				"invalid ByteClock or PixelClock\n");
			return -EINVAL;
		}

		cci_read(lt6911gxd->regmap, REG_H_TOTAL, &htotal, &ret);
		cci_read(lt6911gxd->regmap, REG_V_TOTAL, &vtotal, &ret);
		if (ret || htotal == 0 || vtotal == 0) {
			dev_err(&client->dev, "invalid htotal or vtotal\n");
			return -EINVAL;
		}

		fps = div_u64(pixel_clk, htotal * vtotal);
		if (fps > 60) {
			dev_err(&client->dev,
				"max fps is 60, current fps: %llu\n", fps);
			return -EINVAL;
		}

		cci_read(lt6911gxd->regmap, REG_HALF_H_ACTIVE,
			 &width, &ret);
		cci_read(lt6911gxd->regmap, REG_V_ACTIVE, &height, &ret);
		if (ret || width == 0 || width > 3840 ||
		    height == 0 || height > 2160) {
			dev_err(&client->dev, "invalid width or height\n");
			return -EINVAL;
		}

		lt6911gxd->cur_mode.height = height;
		lt6911gxd->cur_mode.width = width;
		lt6911gxd->cur_mode.fps = fps;
		/* MIPI Clock Rate = ByteClock × 4, defined in lt6911gxd spec */
		lt6911gxd->cur_mode.link_freq = byte_clk * 4;
		v4l2_subdev_notify_event(&lt6911gxd->sd,
					 &lt6911gxd_ev_source_change);
		break;

	case INT_VIDEO_DISAPPEAR:
		cci_write(lt6911gxd->regmap, REG_MIPI_TX_CTRL, 0x0, NULL);
		lt6911gxd->cur_mode.height = 0;
		lt6911gxd->cur_mode.width = 0;
		lt6911gxd->cur_mode.fps = 0;
		lt6911gxd->cur_mode.link_freq = 0;
		v4l2_subdev_notify_event(&lt6911gxd->sd,
					 &lt6911gxd_ev_stream_end);
		break;

	default:
		return  -ENOLINK;
	}

	return ret;
}

/* TODO: add audio sampling rate and present control */
static int lt6911gxd_init_controls(struct lt6911gxd *lt6911gxd)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	s64 pixel_rate;
	int ret;

	ctrl_hdlr = &lt6911gxd->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 8);
	if (ret)
		return ret;

	lt6911gxd->link_freq =
		v4l2_ctrl_new_int_menu(ctrl_hdlr, NULL, V4L2_CID_LINK_FREQ,
				       sizeof(lt6911gxd->cur_mode.link_freq),
				       0, &lt6911gxd->cur_mode.link_freq);

	pixel_rate = get_pixel_rate(lt6911gxd);
	lt6911gxd->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, NULL,
						  V4L2_CID_PIXEL_RATE,
						  pixel_rate, pixel_rate, 1,
						  pixel_rate);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		goto hdlr_free;
	}
	lt6911gxd->sd.ctrl_handler = ctrl_hdlr;

	return 0;

hdlr_free:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	return ret;
}

static void lt6911gxd_update_pad_format(const struct lt6911gxd_mode *mode,
					struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = mode->code;
	fmt->field = V4L2_FIELD_NONE;
}

static int lt6911gxd_enable_streams(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    u32 pad, u64 streams_mask)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct lt6911gxd *lt6911gxd = to_lt6911gxd(sd);
	int ret;

	ret = pm_runtime_resume_and_get(&client->dev);
	if (ret < 0)
		return ret;

	cci_write(lt6911gxd->regmap, REG_MIPI_TX_CTRL, 0x1, &ret);
	if (ret) {
		dev_err(&client->dev, "failed to start stream: %d\n", ret);
		goto err_rpm_put;
	}

	return 0;

err_rpm_put:
	pm_runtime_put(&client->dev);
	return ret;
}

static int lt6911gxd_disable_streams(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state,
				     u32 pad, u64 streams_mask)
{
	struct lt6911gxd *lt6911gxd = to_lt6911gxd(sd);
	struct i2c_client *client = v4l2_get_subdevdata(&lt6911gxd->sd);
	int ret;

	ret = cci_write(lt6911gxd->regmap, REG_MIPI_TX_CTRL, 0x0, NULL);
	if (ret)
		dev_err(&client->dev, "failed to stop stream: %d\n", ret);

	pm_runtime_put(&client->dev);
	return 0;
}

static int lt6911gxd_set_format(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_format *fmt)
{
	struct lt6911gxd *lt6911gxd = to_lt6911gxd(sd);
	u64 pixel_rate, link_freq;

	lt6911gxd_update_pad_format(&lt6911gxd->cur_mode, &fmt->format);
	*v4l2_subdev_state_get_format(sd_state, fmt->pad) = fmt->format;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	pixel_rate = get_pixel_rate(lt6911gxd);
	__v4l2_ctrl_modify_range(lt6911gxd->pixel_rate, pixel_rate,
				 pixel_rate, 1, pixel_rate);

	link_freq = lt6911gxd->cur_mode.link_freq;
	__v4l2_ctrl_modify_range(lt6911gxd->link_freq, link_freq,
				 link_freq, 1, link_freq);

	return 0;
}

static int lt6911gxd_get_format(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_format *fmt)
{
	struct lt6911gxd *lt6911gxd = to_lt6911gxd(sd);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt->format = *v4l2_subdev_state_get_format(sd_state, fmt->pad);
	else
		lt6911gxd_update_pad_format(&lt6911gxd->cur_mode, &fmt->format);

	return 0;
}

static int lt6911gxd_enum_mbus_code(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *sd_state,
				    struct v4l2_subdev_mbus_code_enum *code)
{
	struct lt6911gxd *lt6911gxd = to_lt6911gxd(sd);

	if (code->index)
		return -EINVAL;

	code->code = lt6911gxd->cur_mode.code;

	return 0;
}

static int lt6911gxd_enum_frame_size(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *sd_state,
				     struct v4l2_subdev_frame_size_enum *fse)
{
	struct lt6911gxd *lt6911gxd = to_lt6911gxd(sd);

	if (fse->index != 0)
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_UYVY8_1X16)
		return -EINVAL;

	fse->min_width = lt6911gxd->cur_mode.width;
	fse->max_width = fse->min_width;
	fse->min_height = lt6911gxd->cur_mode.height;
	fse->max_height = fse->min_height;

	return 0;
}

static int lt6911gxd_enum_frame_interval(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *sd_state,
		struct v4l2_subdev_frame_interval_enum *fie)
{
	struct lt6911gxd *lt6911gxd = to_lt6911gxd(sd);

	if (fie->index != 0)
		return -EINVAL;

	fie->interval.numerator = 1;
	fie->interval.denominator = lt6911gxd->cur_mode.fps;

	return 0;
}

static int lt6911gxd_init_state(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state)
{
	struct v4l2_subdev_format fmt = {
		.which = sd_state ? V4L2_SUBDEV_FORMAT_TRY
		: V4L2_SUBDEV_FORMAT_ACTIVE,
	};

	return lt6911gxd_set_format(sd, sd_state, &fmt);
}

static const struct v4l2_subdev_video_ops lt6911gxd_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops lt6911gxd_pad_ops = {
	.set_fmt = lt6911gxd_set_format,
	.get_fmt = lt6911gxd_get_format,
	.enable_streams = lt6911gxd_enable_streams,
	.disable_streams = lt6911gxd_disable_streams,
	.enum_mbus_code = lt6911gxd_enum_mbus_code,
	.enum_frame_size = lt6911gxd_enum_frame_size,
	.enum_frame_interval = lt6911gxd_enum_frame_interval,
	.get_frame_interval = v4l2_subdev_get_frame_interval,
};

static const struct v4l2_subdev_core_ops lt6911gxd_subdev_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_ops lt6911gxd_subdev_ops = {
	.core = &lt6911gxd_subdev_core_ops,
	.video = &lt6911gxd_video_ops,
	.pad = &lt6911gxd_pad_ops,
};

static const struct media_entity_operations lt6911gxd_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops lt6911gxd_internal_ops = {
	.init_state = lt6911gxd_init_state,
};

static int lt6911gxd_fwnode_parse(struct lt6911gxd *lt6911gxd,
				  struct device *dev)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_CPHY,
	};
	int ret;

	endpoint = fwnode_graph_get_endpoint_by_id(dev_fwnode(dev), 0, 0,
						   FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!endpoint)
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "endpoint node not found\n");

	ret = v4l2_fwnode_endpoint_parse(endpoint, &bus_cfg);
	fwnode_handle_put(endpoint);
	if (ret) {
		dev_err(dev, "failed to parse endpoint node: %d\n", ret);
		goto out_err;
	}

	/*
	 * Check the number of MIPI CSI2 data lanes,
	 * lt6911gxd only support 4 lanes.
	 */
	if (bus_cfg.bus.mipi_csi2.num_data_lanes != LT6911GXD_DEFAULT_LANES) {
		dev_err(dev, "only 2 data lanes are currently supported\n");
		goto out_err;
	}

	return 0;

out_err:
	v4l2_fwnode_endpoint_free(&bus_cfg);
	return ret;
}

static int lt6911gxd_identify_module(struct lt6911gxd *lt6911gxd,
				     struct device *dev)
{
	u64 val;
	int ret = 0;

	/* Chip ID should be confirmed when the I2C slave is active */
	cci_write(lt6911gxd->regmap, REG_ENABLE_I2C, 0x1, &ret);
	cci_read(lt6911gxd->regmap, REG_CHIP_ID, &val, &ret);
	cci_write(lt6911gxd->regmap, REG_ENABLE_I2C, 0x0, &ret);
	if (ret)
		return dev_err_probe(dev, ret, "fail to read chip id\n");

	if (val != LT6911GXD_CHIP_ID) {
		return dev_err_probe(dev, -ENXIO, "chip id mismatch: %x!=%x\n",
				     LT6911GXD_CHIP_ID, (u16)val);
	}

	return 0;
}

static irqreturn_t lt6911gxd_threaded_irq_fn(int irq, void *dev_id)
{
	struct v4l2_subdev *sd = dev_id;
	struct lt6911gxd *lt6911gxd = to_lt6911gxd(sd);
	struct v4l2_subdev_state *state;
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE
	};

	state = v4l2_subdev_lock_and_get_active_state(sd);
	lt6911gxd_status_update(lt6911gxd);
	lt6911gxd_set_format(sd, state, &fmt);
	v4l2_subdev_unlock_state(state);

	return IRQ_HANDLED;
}

static void lt6911gxd_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lt6911gxd *lt6911gxd = to_lt6911gxd(sd);

	free_irq(gpiod_to_irq(lt6911gxd->irq_gpio), lt6911gxd);
	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&lt6911gxd->ctrl_handler);
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static int lt6911gxd_probe(struct i2c_client *client)
{
	struct lt6911gxd *lt6911gxd;
	struct device *dev = &client->dev;
	u64 irq_pin_flags;
	int ret;

	lt6911gxd = devm_kzalloc(dev, sizeof(*lt6911gxd), GFP_KERNEL);
	if (!lt6911gxd)
		return -ENOMEM;

	/* define default mode: 4k@60fps, changed when interrupt occurs. */
	lt6911gxd->cur_mode = default_mode;

	lt6911gxd->regmap = devm_regmap_init_i2c(client,
						 &lt6911gxd_regmap_config);
	if (IS_ERR(lt6911gxd->regmap))
		return dev_err_probe(dev, PTR_ERR(lt6911gxd->regmap),
				     "failed to init CCI\n");

	v4l2_i2c_subdev_init(&lt6911gxd->sd, client, &lt6911gxd_subdev_ops);

	lt6911gxd->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(lt6911gxd->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(lt6911gxd->reset_gpio),
				     "failed to get reset gpio\n");

	ret = lt6911gxd_fwnode_parse(lt6911gxd, dev);
	if (ret)
		return ret;

	usleep_range(10000, 10500);

	ret = lt6911gxd_identify_module(lt6911gxd, dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to find chip\n");

	ret = lt6911gxd_init_controls(lt6911gxd);
	if (ret)
		return dev_err_probe(dev, ret, "failed to init control\n");

	lt6911gxd->sd.dev = dev;
	lt6911gxd->sd.internal_ops = &lt6911gxd_internal_ops;
	lt6911gxd->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			       V4L2_SUBDEV_FL_HAS_EVENTS;
	lt6911gxd->sd.entity.ops = &lt6911gxd_subdev_entity_ops;
	lt6911gxd->sd.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	lt6911gxd->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&lt6911gxd->sd.entity, 1, &lt6911gxd->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto err_v4l2_ctrl_handler_free;
	}

	/*
	 * Device is already turned on by i2c-core with ACPI domain PM.
	 * Enable runtime PM and turn off the device.
	 */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	ret = v4l2_subdev_init_finalize(&lt6911gxd->sd);
	if (ret) {
		dev_err(dev, "failed to init v4l2 subdev: %d\n", ret);
		goto err_media_entity_cleanup;
	}

	lt6911gxd->irq_gpio = devm_gpiod_get(dev, "hpd", GPIOD_IN);
	if (IS_ERR(lt6911gxd->irq_gpio))
		return dev_err_probe(dev, PTR_ERR(lt6911gxd->irq_gpio),
				     "failed to get hpd gpio\n");
	dev_dbg(dev, "%s: use hpd irq_gpio!\n", __func__);

	/* Setting irq */
	irq_pin_flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
			IRQF_ONESHOT;

	ret = request_threaded_irq(gpiod_to_irq(lt6911gxd->irq_gpio), NULL,
				   lt6911gxd_threaded_irq_fn,
				   irq_pin_flags, NULL, lt6911gxd);
	if (ret) {
		dev_err(dev, "failed to request IRQ: %d\n", ret);
		goto err_subdev_cleanup;
	}

	ret = v4l2_async_register_subdev_sensor(&lt6911gxd->sd);
	if (ret) {
		dev_err(dev, "failed to register V4L2 subdev: %d\n", ret);
		goto err_free_irq;
	}

	return 0;

err_free_irq:
	free_irq(gpiod_to_irq(lt6911gxd->irq_gpio), lt6911gxd);

err_subdev_cleanup:
	v4l2_subdev_cleanup(&lt6911gxd->sd);

err_media_entity_cleanup:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	media_entity_cleanup(&lt6911gxd->sd.entity);

err_v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(lt6911gxd->sd.ctrl_handler);

	return ret;
}

static int lt6911gxd_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lt6911gxd *lt6911gxd = to_lt6911gxd(sd);

	/* Active low gpio reset, set 1 to power off sensor */
	gpiod_set_value_cansleep(lt6911gxd->reset_gpio, 1);

	return 0;
}

static int lt6911gxd_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lt6911gxd *lt6911gxd = to_lt6911gxd(sd);
	int ret = 0;
	int count = 0;

	if (lt6911gxd->reset_gpio != NULL) {
		do {
			gpiod_set_value_cansleep(lt6911gxd->reset_gpio, 0);
			ret = gpiod_get_value_cansleep(lt6911gxd->reset_gpio);
			usleep_range(200 * 1000, 200 * 1000 + 500);

			if (++count >= 20) {
				dev_err(&client->dev,
					"%s: failed to power on reset gpio, reset gpio is %d",
					__func__, ret);
				break;
			}
		} while (ret != 0);
	}

	return ret;
}

static const struct dev_pm_ops lt6911gxd_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lt6911gxd_suspend, lt6911gxd_resume)
};

static const struct acpi_device_id lt6911gxd_acpi_ids[] = {
	{ "INTC1124" },
	{}
};
MODULE_DEVICE_TABLE(acpi, lt6911gxd_acpi_ids);

static struct i2c_driver lt6911gxd_i2c_driver = {
	.driver = {
		.name = "lt6911gxd",
		.acpi_match_table = ACPI_PTR(lt6911gxd_acpi_ids),
		.pm = &lt6911gxd_pm_ops,
	},
	.probe = lt6911gxd_probe,
	.remove = lt6911gxd_remove,
};

module_i2c_driver(lt6911gxd_i2c_driver);

MODULE_AUTHOR("Zou, Xiaohong xiaohong.zou@intel.com");
MODULE_DESCRIPTION("Lontium lt6911gxd HDMI to MIPI Bridge Driver");
MODULE_LICENSE("GPL");
