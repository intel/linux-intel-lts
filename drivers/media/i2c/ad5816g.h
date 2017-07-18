/*
 * Copyright (c) 2016 Intel Corporation. All Rights Reserved.
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
#ifndef __AD5816G_H__
#define __AD5816G_H__

#include <linux/types.h>

#define AD5816G_ADDR		0x0e
#define AD5816G_NAME		"ad5816g"

#define AD5816G_MAX_FOCUS_POS		1023

/* Register Definitions */
#define AD5816G_IC_INFO			0x00
#define AD5816G_IC_VERSION		0x01
#define AD5816G_CONTROL			0x02
#define AD5816G_VCM_CODE_MSB		0x03
#define AD5816G_VCM_CODE_LSB		0x04
#define AD5816G_STATUS			0x05
#define AD5816G_MODE			0x06
#define AD5816G_VCM_FREQ		0x07
#define AD5816G_VCM_THRESHOLD		0x08

/* ARC MODE ENABLE */
#define AD5816G_ARC_EN			0x02
/* ARC RES2 MODE */
#define AD5816G_ARC_RES2		0x01
/* ARC VCM FREQ - 78.1Hz */
#define AD5816G_DEF_FREQ		0x7a
/* ARC VCM THRESHOLD - 0x08 << 1 */
#define AD5816G_DEF_THRESHOLD		0x64
#define AD5816G_ID			0x24
#define VCM_CODE_MASK			0x03ff
#define AD5816G_MODE_2_5M_SWITCH_CLOCK	0x14
/* VCM SW RESET */
#define AD5816G_RESET			0x01

#endif
