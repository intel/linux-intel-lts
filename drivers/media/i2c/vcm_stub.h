
/*
 * Copyright (c) 2017 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef _VCM_STUB_H_
#define _VCM_STUB_H_

#define VCM_STUB_NAME		"vcm_stub"
#define VCM_MAX_FOCUS_POS	0xFFFF

/* VCM stub device structure */
struct vcm_stub_device {
	struct i2c_client *client;
	struct v4l2_ctrl_handler ctrls_vcm;
	struct v4l2_subdev subdev_vcm;
	unsigned int current_val;
};

#endif
