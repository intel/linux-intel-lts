/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Keem Bay Camera Sensor Driver.
 *
 * Copyright (C) 2018-2019 Intel Corporation
 */
#ifndef KMB_SENSOR_H
#define KMB_SENSOR_H

#define KMB_SENSOR_DRV_NAME	"kmb-camera-sensor"

#define KMB_SENSOR_SRC_PAD	0
#define KMB_SENSOR_PADS_NUM	1

/**
 * struct kmb_sensor_ctrls - KMB Camera Sensor controls structure
 * @handler: control handler
 * @exposure: exposure control
 * @gain: gain control
 * @update: update flag
 */
struct kmb_sensor_ctrls {
	struct v4l2_ctrl_handler handler;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *gain;
	u8 update;
};

/**
 * struct kmb_sensor_framesize - KMB Camera Sensor frame sizes
 * @width: frame width
 * @height: frame height
 * @max_exp_lines: maximum exposure lines
 * @regs: registers
 */
struct kmb_sensor_framesize {
	u16 width;
	u16 height;
	u16 max_exp_lines;
	const u8 *regs;
};

/**
 * struct i2c_arr - Array representation of address - value pair
 * @addr: address
 * @val: value
 */
struct i2c_arr {
	u16 addr;
	u16 val;
};

/**
 * struct kmb_sensor - KMB Camera Sensor device structure
 * @client: pointer to i2c client device
 * @lock: mutex
 * @subdev: V4L2 sub-device
 * @pads: array of media pad graph objects
 * @formats: array of frame formats on the media bus
 * @bus_type: media bus type
 * @curr_fmt: current frame format
 * @ctrls: sensor controls
 * @exp_row_interval: exposure row intervals in us
 * @frame_size: pointer to KMB camera sensor frame sizes
 * @tslb_reg: YUVY sequence (pixel format) control register
 * @streaming: streaming state flag
 * @power_on: power on flag
 */
struct kmb_sensor {
	struct i2c_client *client;
	struct mutex lock;

	struct v4l2_subdev subdev;
	struct media_pad pads[KMB_SENSOR_PADS_NUM];
	struct v4l2_mbus_framefmt formats[KMB_SENSOR_PADS_NUM];
	enum v4l2_mbus_type bus_type;
	struct v4l2_mbus_framefmt curr_fmt;
	struct kmb_sensor_ctrls ctrls;

	u32 exp_row_interval;
	const struct kmb_sensor_framesize *frame_size;
	u8 tslb_reg;

	bool streaming;
	bool power_on;
};

#endif /* KMB_SENSOR_H */
