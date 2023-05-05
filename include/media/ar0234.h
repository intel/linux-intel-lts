/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2014 - 2022 Intel Corporation */

#ifndef __AR0234_H
#define __AR0234_H

#include <linux/types.h>

#define AR0234_NAME		"ar0234"

struct ar0234_platform_data {
	unsigned int port;
	unsigned int lanes;
	uint32_t i2c_slave_address;
	int irq_pin;
	unsigned int irq_pin_flags;
	char irq_pin_name[16];
	int reset_pin;
	int detect_pin;
	char suffix;
	int gpios[4];
};

#endif /* __AR0234_H  */
