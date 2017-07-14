/*
 * Copyright (c) 2017 Intel Corporation
 *
 * Functions to access TPS68470 power management chip.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __LINUX_MFD_TPS68470_H
#define __LINUX_MFD_TPS68470_H

#include <linux/i2c.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

/* I2C ID for TPS68470 part */
#define TPS68470_I2C_ID			0x4d
#define TPS68470_NAME			"tps68470"

/* All register addresses */
#define TPS68470_REG_GSTAT		0x01
#define TPS68470_REG_VRSTA		0x02
#define TPS68470_REG_VRSHORT		0x03
#define TPS68470_REG_INTMASK		0x04
#define TPS68470_REG_VCOSPEED		0x05
#define TPS68470_REG_POSTDIV2		0x06
#define TPS68470_REG_BOOSTDIV		0x07
#define TPS68470_REG_BUCKDIV		0x08
#define TPS68470_REG_PLLSWR		0x09
#define TPS68470_REG_XTALDIV		0x0A
#define TPS68470_REG_PLLDIV		0x0B
#define TPS68470_REG_POSTDIV		0x0C
#define TPS68470_REG_PLLCTL		0x0D
#define TPS68470_REG_PLLCTL2		0x0E
#define TPS68470_REG_CLKCFG1		0x0F
#define TPS68470_REG_CLKCFG2		0x10
#define TPS68470_REG_GPCTL0A		0x14
#define TPS68470_REG_GPCTL0B		0x15
#define TPS68470_REG_GPCTL1A		0x16
#define TPS68470_REG_GPCTL1B		0x17
#define TPS68470_REG_GPCTL2A		0x18
#define TPS68470_REG_GPCTL2B		0x19
#define TPS68470_REG_GPCTL3A		0x1A
#define TPS68470_REG_GPCTL3B		0x1B
#define TPS68470_REG_GPCTL4A		0x1C
#define TPS68470_REG_GPCTL4B		0x1D
#define TPS68470_REG_GPCTL5A		0x1E
#define TPS68470_REG_GPCTL5B		0x1F
#define TPS68470_REG_GPCTL6A		0x20
#define TPS68470_REG_GPCTL6B		0x21
#define TPS68470_REG_SGPO		0x22
#define TPS68470_REG_PITCTL		0x23
#define TPS68470_REG_WAKECFG		0x24
#define TPS68470_REG_IOWAKESTAT		0x25
#define TPS68470_REG_GPDI		0x26
#define TPS68470_REG_GPDO		0x27
#define TPS68470_REG_ILEDCTL		0x28
#define TPS68470_REG_WLEDSTAT		0x29
#define TPS68470_REG_VWLEDILIM		0x2A
#define TPS68470_REG_VWLEDVAL		0x2B
#define TPS68470_REG_WLEDMAXRER		0x2C
#define TPS68470_REG_WLEDMAXT		0x2D
#define TPS68470_REG_WLEDMAXAF		0x2E
#define TPS68470_REG_WLEDMAXF		0x2F
#define TPS68470_REG_WLEDTO		0x30
#define TPS68470_REG_VWLEDCTL		0x31
#define TPS68470_REG_WLEDTIMER_MSB	0x32
#define TPS68470_REG_WLEDTIMER_LSB	0x33
#define TPS68470_REG_WLEDC1		0x34
#define TPS68470_REG_WLEDC2		0x35
#define TPS68470_REG_WLEDCTL		0x36
#define TPS68470_REG_VCMVAL		0x3C
#define TPS68470_REG_VAUX1VAL		0x3D
#define TPS68470_REG_VAUX2VAL		0x3E
#define TPS68470_REG_VIOVAL		0x3F
#define TPS68470_REG_VSIOVAL		0x40
#define TPS68470_REG_VAVAL		0x41
#define TPS68470_REG_VDVAL		0x42
#define TPS68470_REG_S_I2C_CTL		0x43
#define TPS68470_REG_VCMCTL		0x44
#define TPS68470_REG_VAUX1CTL		0x45
#define TPS68470_REG_VAUX2CTL		0x46
#define TPS68470_REG_VACTL		0x47
#define TPS68470_REG_VDCTL		0x48
#define TPS68470_REG_RESET		0x50
#define TPS68470_REG_REVID		0xFF

#define TPS68470_REG_MAX		TPS68470_REG_REVID

/* Register field definitions */
#define TPS68470_REVID_VENDOR_MASK	0xE0
#define TPS68470_REVID_MAJOR_REV_MASK	0x18
#define TPS68470_REVID_MINOR_REV_MASK	0x07

#define TPS68470_VAVAL_AVOLT_MASK	0x7F

#define TPS68470_VDVAL_DVOLT_MASK	0x3F
#define TPS68470_VCMVAL_VCVOLT_MASK	0x7F
#define TPS68470_VIOVAL_IOVOLT_MASK	0x7F
#define TPS68470_VSIOVAL_IOVOLT_MASK	0x7F
#define TPS68470_VAUX1VAL_AUX1VOLT_MASK	0x7F
#define TPS68470_VAUX2VAL_AUX2VOLT_MASK	0x7F

#define TPS68470_VACTL_EN_MASK		BIT(0)
#define TPS68470_VDCTL_EN_MASK		BIT(0)
#define TPS68470_VCMCTL_EN_MASK		BIT(0)
#define TPS68470_S_I2C_CTL_EN_MASK	BIT(1)
#define TPS68470_VAUX1CTL_EN_MASK	BIT(0)
#define TPS68470_VAUX2CTL_EN_MASK	BIT(0)
#define TPS68470_PLL_EN_MASK		BIT(0)

#define TPS68470_OSC_EXT_CAP_SHIFT	4
#define TPS68470_OSC_EXT_CAP_DEFAULT	0x05 /* 10pf */

#define TPS68470_CLK_SRC_SHIFT		7
#define TPS68470_CLK_SRC_GPIO3		0
#define TPS68470_CLK_SRC_XTAL		1

/* swr:4ms swr_ss:79*PLL_SS_REFCLK cycles */
#define TPS68470_PLLSWR_DEFAULT		19

#define TPS68470_DRV_STR_1MA		0
#define TPS68470_DRV_STR_2MA		1
#define TPS68470_DRV_STR_4MA		2
#define TPS68470_DRV_STR_8MA		3
#define TPS68470_DRV_STR_A_SHIFT	0
#define TPS68470_DRV_STR_B_SHIFT	2

#define TPS68470_OUTPUT_XTAL_BUFFERED	1
#define TPS68470_PLL_OUTPUT_ENABLE	2
#define TPS68470_PLL_OUTPUT_SS_ENABLE	3
#define TPS68470_OUTPUT_A_SHIFT		0
#define TPS68470_OUTPUT_B_SHIFT		2

#define TPS68470_GPIO_CTL_REG_A(x)	(TPS68470_REG_GPCTL0A + (x) * 2)
#define TPS68470_GPIO_CTL_REG_B(x)	(TPS68470_REG_GPCTL0B + (x) * 2)
#define TPS68470_GPIO_MODE_MASK		(BIT(0) | BIT(1))
#define TPS68470_GPIO_MODE_IN		0
#define TPS68470_GPIO_MODE_IN_PULLUP	1
#define TPS68470_GPIO_MODE_OUT_CMOS	2
#define TPS68470_GPIO_MODE_OUT_ODRAIN	3

enum tps68470_regulator_id {
	/* converters */
	TPS68470_CORE,
	/* LDOs */
	TPS68470_ANA,
	TPS68470_VCM,
	TPS68470_VIO,
	TPS68470_VSIO,
	TPS68470_AUX1,
	TPS68470_AUX2,
};

#define TPS68470_MAX_REG_ID		TPS68470_AUX2

/* Number of converters available */
#define TPS68470_NUM_CONVERTERS		1
/* Number of LDO voltage regulators available */
#define TPS68470_NUM_LDO		6
/* Number of total regulators available */
#define TPS68470_NUM_REGULATOR	\
	(TPS68470_NUM_CONVERTERS + TPS68470_NUM_LDO)

/**
 * struct tps68470_board - packages regulator init data
 * @tps68470_regulator_data: regulator initialization values
 *
 * Board data may be used to initialize regulator.
 */
struct tps68470_board {
	struct regulator_init_data *tps68470_init_data[TPS68470_NUM_REGULATOR];
};

/**
 * struct tps68470 - tps68470 sub-driver chip access routines
 *
 * Device data may be used to access the TPS68470 chip
 */

struct tps68470 {
	struct device *dev;
	struct tps68470_board *pdata;
	unsigned long id;
	struct regulator_desc desc[TPS68470_NUM_REGULATOR];
	struct regmap *regmap;
};

static inline struct tps68470 *dev_to_tps68470(struct device *dev)
{
	return dev_get_drvdata(dev);
}

static inline unsigned long tps68470_chip_id(struct tps68470 *tps68470)
{
	return tps68470->id;
}

int tps68470_reg_read(struct tps68470 *tps, unsigned int reg,
					unsigned int *val);
int tps68470_reg_write(struct tps68470 *tps, unsigned int reg,
			unsigned int val);
int tps68470_set_bits(struct tps68470 *tps, unsigned int reg,
		unsigned int mask, unsigned int val);
int tps68470_clear_bits(struct tps68470 *tps, unsigned int reg,
		unsigned int mask);

#endif /*  __LINUX_MFD_TPS68470_H */
