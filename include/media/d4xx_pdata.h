/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2022 Intel Corporation */

#ifndef D457_H
#define D457_H

#define D457_NAME "d4xx"
#define MAX9296_NAME "MAX9296"

struct d4xx_subdev_info {
	struct i2c_board_info board_info;
	int i2c_adapter_id;
	unsigned short rx_port;
	unsigned short phy_i2c_addr;
	unsigned short ser_alias;
	const char suffix; /* suffix for subdevs */
};

struct d4xx_pdata {
	unsigned int subdev_num;
	struct d4xx_subdev_info *subdev_info;
	unsigned int reset_gpio;
	int FPD_gpio;
	const char suffix;
	struct i2c_board_info *deser_board_info;
};
#endif
