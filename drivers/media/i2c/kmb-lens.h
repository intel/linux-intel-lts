/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * kmb-lens.h - KeemBay Camera Lens Driver.
 *
 * Copyright (C) 2019 Intel Corporation
 */
#ifndef KMB_LENS_H
#define KMB_LENS_H

#define KMB_LENS_DRV_NAME "kmb-camera-lens"

/* Custom controls */
#define V4L2_CID_IR_FILTER (V4L2_CID_USER_BASE | 0x1)
#define V4L2_CID_IRIS      (V4L2_CID_USER_BASE | 0x2)

/**
 * struct kmb_lens_ctrls - KMB Camera Lens controls structure
 * @handler: control handler
 * @absolute_focus: absolute focus control
 * @relative_focus: relative focus control
 * @auto_focus: auto focus control
 * @ir_filter_ctrl: IR cut filter control
 * @iris_ctrl: Iris control
 */
struct kmb_lens_ctrls {
	struct v4l2_ctrl_handler handler;
	struct v4l2_ctrl *absolute_focus;
	struct v4l2_ctrl *relative_focus;
	struct v4l2_ctrl *auto_focus;
	struct v4l2_ctrl *ir_filter_ctrl;
	struct v4l2_ctrl *iris_ctrl;
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
 * @iris: Iris pwm device
 * @subdev: V4L2 sub-device
 * @ctrls: lens controls
 * @wq: Lens driver work queue
 * @iris_cur_pos: Iris current position
 * @iris_new_pos: Iris new position
 * @work_iris: Iris work
 */
struct kmb_lens {
	struct i2c_client *client;
	struct mutex lock;
	struct gpio_desc *enable_gpio;
	struct gpio_desc *phase_gpio;
	struct pwm_device *iris;
	struct v4l2_subdev subdev;
	struct kmb_lens_ctrls ctrls;
	struct workqueue_struct *wq;

	u32 iris_cur_pos;
	u32 iris_new_pos;
	struct work_struct work_iris;
};

#endif /* KMB_LENS_H */
