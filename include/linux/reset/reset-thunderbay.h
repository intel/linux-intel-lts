/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020 Intel Corporation
 */

#ifndef _LINUX_RESET_THUNDERBAY_H_
#define _LINUX_RESET_THUNDERBAY_H_

#define THUNDERBAY_RESET_EN_OFFSET     0x0
/* deassert register offset */
#define THUNDERBAY_RESET_SET_OFFSET    0x4
/* assert register offset */
#define THUNDERBAY_RESET_CLEAR_OFFSET  0x8

#define THUNDERBAY_RESET(_reg, _bit)	\
	{				\
		.reg = (_reg),		\
		.bit = (_bit),		\
	}

struct thunderbay_rst_map {
	unsigned int reg;
	unsigned int bit;
};

struct thunderbay_rst_data {
	const struct thunderbay_rst_map *resets;
	size_t n_resets;
};

#include <linux/reset/reset-thunderbay-ss.h>
#endif
