/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2014 - 2022 Intel Corporation */

#ifndef __LT6911UXC_H
#define __LT6911UXC_H

#include <linux/types.h>

#define LT6911UXC_NAME             "lt6911uxc"

struct lt6911uxc_platform_data {
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

#endif /* __LT6911UXC_H  */
