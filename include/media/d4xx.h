/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2022 Intel Corporation */

#ifndef D457_H
#define D457_H

#define D457_NAME "d4xx"
#define MAX9296_NAME "MAX9296"

struct d4xx_pdata {
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

#endif
