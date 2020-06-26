/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Keem Bay Camera Flash Driver.
 *
 * Copyright (C) 2019-2020 Intel Corporation
 */
#ifndef KMB_FLASH_H
#define KMB_FLASH_H

#include <linux/atomic.h>
#include <linux/workqueue.h>

#define KMB_FLASH_DRV_NAME "kmb-camera-flash"

/**
 * struct kmb_flash_ctrls - KMB Camera Flash controls structure
 * @handler: control handler
 * @led_mode: led mode control
 * @strobe_source: strobe source control
 * @strobe: strobe control
 * @strobe_stop: strobe stop control
 * @timeout: timeout control
 * @intensity: intensity control
 * @torch_intensity: torch intensity control
 * @update: update flag
 */
struct kmb_flash_ctrls {
	struct v4l2_ctrl_handler handler;
	struct v4l2_ctrl *led_mode;
	struct v4l2_ctrl *strobe_source;
	struct v4l2_ctrl *strobe;
	struct v4l2_ctrl *strobe_stop;
	struct v4l2_ctrl *timeout;
	struct v4l2_ctrl *intensity;
	struct v4l2_ctrl *torch_intensity;
	u8 update;
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
 * struct kmb_flash - KMB Camera Flash device structure
 * @client: pointer to i2c client device
 * @lock: mutex
 * @wq: pointer to work queue
 * @work_flash: job to be done
 * @subdev: V4L2 sub-device
 * @led_gpio: gpio for controlling a led
 * @ctrls: flash controls
 * @power_lock: mutex to protect power settings
 * @power_on: power on flag
 * @duration: duration in us
 */
struct kmb_flash {
	struct i2c_client *client;
	struct mutex lock;

	struct workqueue_struct *wq;
	struct work_struct work_flash;

	struct v4l2_subdev subdev;
	struct gpio_desc *led_gpio;
	struct kmb_flash_ctrls ctrls;

	struct mutex power_lock;
	bool power_on;
	atomic_t duration;
};

#endif /* KMB_FLASH_H */
