/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2020 Intel Corporation */

#ifndef TI960_H
#define TI960_H

#include <linux/i2c.h>
#include <linux/regmap.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>

#define TI960_NAME "ti960"

#define TI960_I2C_ADDRESS	0x32
#define TI960_I2C_ADDRESS_2	0x3d

#define PIXEL_ORDER_GRBG	0
#define PIXEL_ORDER_RGGB	1
#define PIXEL_ORDER_BGGR	2
#define PIXEL_ORDER_GBRG	3

#define NR_OF_TI960_VCS_PER_SINK_PAD 2
#define NR_OF_TI960_VCS_SOURCE_PAD 4
#define NR_OF_TI960_SOURCE_PADS	1
#define NR_OF_TI960_SINK_PADS	4
#define NR_OF_TI960_PADS \
	(NR_OF_TI960_SOURCE_PADS + NR_OF_TI960_SINK_PADS)
/* 4port * 2vc/port * 8 stream total */
#define NR_OF_TI960_STREAMS	\
	(NR_OF_TI960_SINK_PADS * NR_OF_TI960_VCS_PER_SINK_PAD \
	* NR_OF_TI960_VCS_SOURCE_PAD)
#define NR_OF_GPIOS_PER_PORT	2
#define NR_OF_TI960_GPIOS	\
	(NR_OF_TI960_SINK_PADS * NR_OF_GPIOS_PER_PORT)

#define TI960_PAD_SOURCE	4

#define TI960_MIN_WIDTH		640
#define TI960_MIN_HEIGHT	480
#define TI960_MAX_WIDTH		1920
#define TI960_MAX_HEIGHT	1200

struct ti960_csi_data_format {
	u32 code;
	u8 width;
	u8 compressed;
	u8 pixel_order;
	u8 mipi_dt_code;
};

struct ti960_subdev_info {
	struct i2c_board_info board_info;
	int i2c_adapter_id;
	unsigned short rx_port;
	unsigned short phy_i2c_addr;
	unsigned short ser_alias;
	const char suffix; /* suffix for subdevs */
};

struct ti960_pdata {
	unsigned int subdev_num;
	struct ti960_subdev_info *subdev_info;
	unsigned int reset_gpio;
	int FPD_gpio;
	const char suffix;
};

#define TI960_MAX_GPIO_POWERUP_SEQ        4

/* set this flag if this module needs serializer initialization */
#define TI960_FL_INIT_SER	BIT(0)
/* set this flag if this module has extra powerup sequence */
#define TI960_FL_POWERUP	BIT(1)
/* set this flag if this module needs reset signal */
#define TI960_FL_RESET	BIT(2)
/* set this flag if it need to init serial clk only */
#define TI960_FL_INIT_SER_CLK	BIT(4)

struct ti960_subdev_pdata {
	unsigned short i2c_addr;
	unsigned short i2c_adapter;

	unsigned int lanes;		/* Number of CSI-2 lanes */

	/* specify gpio pins of Deser for PWDN, FSIN, RESET. */
	int xshutdown;
	int fsin;
	int reset;

	/* specify gpio pins boot timing. */
	/* Bit 3 write 0/1 on GPIO3
	 * Bit 2 write 0/1 on GPIO2
	 * Bit 1 write 0/1 on GPIO1
	 * Bit 0 write 0/1 on GPIO0
	 */
	char gpio_powerup_seq[TI960_MAX_GPIO_POWERUP_SEQ];

	/* module_flags can be:
	 * TI960_FL_INIT_SER
	 * TI960_FL_POWERUP
	 * TI960_FL_RESET
	 */
	unsigned int module_flags;

	char module_name[16]; /* module name from ACPI */
	char suffix; /* suffix to identify multi sensors, abcd.. */
};

#endif
