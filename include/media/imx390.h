/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2014 - 2020 Intel Corporation */

#ifndef __IMX390_H
#define __IMX390_H

#include <linux/types.h>

#define IMX390_NAME "imx390"

struct imx390_platform_data {
	unsigned int port;
	unsigned int lanes;
	uint32_t i2c_slave_address;
	int irq_pin;
	unsigned int irq_pin_flags;
	char irq_pin_name[16];
	char suffix;
	int gpios[4];
};

#endif /* __IMX390_H  */
