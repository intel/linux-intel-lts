/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Keem Bay Camera Lens Driver.
 *
 * Copyright (C) 2019-2020 Intel Corporation
 */
#ifndef KMB_LENS_H
#define KMB_LENS_H

#define KMB_LENS_DRV_NAME "kmb-camera-lens"

/* Custom controls */
#define V4L2_CID_IR_FILTER (V4L2_CID_USER_BASE | 0x1)

/**
 * struct kmb_lens_ctrls - KMB Camera Lens controls structure
 * @handler: control handler
 * @absolute_focus: absolute focus control
 * @relative_focus: relative focus control
 * @auto_focus: auto focus control
 * @ir_ctrl: IR control
 */
struct kmb_lens_ctrls {
	struct v4l2_ctrl_handler handler;
	struct v4l2_ctrl *absolute_focus;
	struct v4l2_ctrl *relative_focus;
	struct v4l2_ctrl *auto_focus;
	struct v4l2_ctrl *ir_ctrl;
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
 * struct kmb_lens - KMB Camera Lens device structure
 * @client: pointer to i2c client device
 * @lock: mutex
 * @enable_gpio: Enable gpio
 * @phase_gpio: Phase gpio
 * @subdev: V4L2 sub-device
 * @ctrls: lens controls
 * @power_on: power on flag
 */
struct kmb_lens {
	struct i2c_client *client;
	struct mutex lock;
	struct gpio_desc *enable_gpio;
	struct gpio_desc *phase_gpio;
	struct v4l2_subdev subdev;
	struct kmb_lens_ctrls ctrls;
};

#endif /* KMB_LENS_H */
