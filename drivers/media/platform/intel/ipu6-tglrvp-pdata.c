// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2020 Intel Corporation
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/pci.h>

#include <media/ipu-isys.h>

#include <media/ar0234.h>
#include <media/imx390.h>

#include "ipu.h"

static void ar0234_fixup_spdata(const void *spdata_rep, void *spdata)
{
	const struct ipu_spdata_rep *rep = spdata_rep;
	struct ar0234_platform_data *platform = spdata;

	if (NULL == spdata_rep || NULL == spdata)
		return;

	platform->port = rep->port_n;
	platform->lanes = rep->lanes;
	platform->i2c_slave_address = rep->slave_addr_n;
	platform->gpios[0] = rep->gpios[0];
	platform->irq_pin = rep->irq_pin;
	platform->irq_pin_flags = rep->irq_pin_flags;
	strcpy(platform->irq_pin_name, rep->irq_pin_name);
	platform->suffix = rep->suffix;

	return;
}

#define AR0234_LANES       2
#define AR0234_I2C_ADDRESS 0x10
#define AR0234_I2C_ADDRESS_2 0x18

static struct ipu_isys_csi2_config ar0234_csi2_cfg_1 = {
	.nlanes = AR0234_LANES,
	.port = 1,
};

static struct ar0234_platform_data ar0234_pdata_1 = {
	.port = 1,
	.lanes = 2,
	.i2c_slave_address = AR0234_I2C_ADDRESS,
	.irq_pin = 338,
	.irq_pin_name = "B23",
	.irq_pin_flags = IRQF_TRIGGER_RISING
		| IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
	.suffix = 'a',
	.gpios = {258, 0, 0, 0},
};

static struct ipu_isys_subdev_info ar0234_sd_1 = {
	.csi2 = &ar0234_csi2_cfg_1,
	.i2c = {
	.board_info = {
		I2C_BOARD_INFO("ar0234", AR0234_I2C_ADDRESS),
		.platform_data = &ar0234_pdata_1,
	},
	.i2c_adapter_bdf = "0000:00:15.3",
	},
	.fixup_spdata = ar0234_fixup_spdata,
};

static struct ipu_isys_csi2_config ar0234_csi2_cfg_2 = {
	.nlanes = AR0234_LANES,
	.port = 2,
};

static struct ar0234_platform_data ar0234_pdata_2 = {
	.port = 2,
	.lanes = 2,
	.i2c_slave_address = AR0234_I2C_ADDRESS,
	.irq_pin = 330,
	.irq_pin_name = "R6",
	.irq_pin_flags = IRQF_TRIGGER_RISING
		| IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
	.suffix = 'b',
	.gpios = {501, 0, 0, 0},
};

static struct ipu_isys_subdev_info ar0234_sd_2 = {
	.csi2 = &ar0234_csi2_cfg_2,
	.i2c = {
	.board_info = {
		I2C_BOARD_INFO("ar0234", AR0234_I2C_ADDRESS),
		.platform_data = &ar0234_pdata_2,
	},
	.i2c_adapter_bdf = "0000:00:19.1",
	},
	.fixup_spdata = ar0234_fixup_spdata,
};

static struct ipu_isys_csi2_config ar0234_csi2_cfg_3 = {
	.nlanes = AR0234_LANES,
	.port = 4,
};

static struct ar0234_platform_data ar0234_pdata_3 = {
	.port = 4,
	.lanes = 2,
	.i2c_slave_address = AR0234_I2C_ADDRESS,
	.irq_pin = 332,
	.irq_pin_name = "IMGCLKOUT_3",
	.irq_pin_flags = IRQF_TRIGGER_RISING
		| IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
	.suffix = 'c',
	.gpios = {253, 0, 0, 0},
};

static struct ipu_isys_subdev_info ar0234_sd_3 = {
	.csi2 = &ar0234_csi2_cfg_3,
	.i2c = {
	.board_info = {
		I2C_BOARD_INFO("ar0234", AR0234_I2C_ADDRESS),
		.platform_data = &ar0234_pdata_3,
	},
	.i2c_adapter_bdf = "0000:00:15.2",
	},
	.fixup_spdata = ar0234_fixup_spdata,
};

static struct ipu_isys_csi2_config ar0234_csi2_cfg_4 = {
	.nlanes = AR0234_LANES,
	.port = 5,
};

static struct ar0234_platform_data ar0234_pdata_4 = {
	.port = 5,
	.lanes = 2,
	.i2c_slave_address = AR0234_I2C_ADDRESS_2,
	.irq_pin = 331,
	.irq_pin_name = "H15",
	.irq_pin_flags = IRQF_TRIGGER_RISING
		| IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
	.suffix = 'd',
	.gpios = {325, 0, 0, 0},
};

static struct ipu_isys_subdev_info ar0234_sd_4 = {
	.csi2 = &ar0234_csi2_cfg_4,
	.i2c = {
	.board_info = {
		I2C_BOARD_INFO("ar0234", AR0234_I2C_ADDRESS_2),
		.platform_data = &ar0234_pdata_4,
	},
	.i2c_adapter_bdf = "0000:00:15.2",
	},
	.fixup_spdata = ar0234_fixup_spdata,
};

#define IMX390_LANES       4
#define IMX390_I2C_ADDRESS 0x1e

static struct ipu_isys_csi2_config imx390_csi2_cfg_1 = {
	.nlanes = IMX390_LANES,
	.port = 1,
};

static struct imx390_platform_data imx390_pdata_1 = {
	.port = 1,
	.lanes = 4,
	.i2c_slave_address = IMX390_I2C_ADDRESS,
	.suffix = 'a',
};

static struct ipu_isys_subdev_info imx390_sd_1 = {
	.csi2 = &imx390_csi2_cfg_1,
	.i2c = {
	.board_info = {
		I2C_BOARD_INFO("imx390", IMX390_I2C_ADDRESS),
		.platform_data = &imx390_pdata_1,
	},
	.i2c_adapter_bdf = "0000:00:15.3",
	},
};

static struct ipu_isys_csi2_config imx390_csi2_cfg_2 = {
	.nlanes = IMX390_LANES,
	.port = 2,
};

static struct imx390_platform_data imx390_pdata_2 = {
	.port = 2,
	.lanes = 4,
	.i2c_slave_address = IMX390_I2C_ADDRESS,
	.suffix = 'b',
};

static struct ipu_isys_subdev_info imx390_sd_2 = {
	.csi2 = &imx390_csi2_cfg_2,
	.i2c = {
	.board_info = {
		I2C_BOARD_INFO("imx390", IMX390_I2C_ADDRESS),
		.platform_data = &imx390_pdata_2,
	},
	.i2c_adapter_bdf = "0000:00:19.1",
	},
};

static struct ipu_isys_clk_mapping clk_mapping[] = {
	{ CLKDEV_INIT(NULL, NULL, NULL), NULL }
};

static struct ipu_isys_subdev_pdata pdata = {
	.subdevs = (struct ipu_isys_subdev_info *[]) {
		&ar0234_sd_1,
		&ar0234_sd_2,
		&ar0234_sd_3,
		&ar0234_sd_4,
		&imx390_sd_1,
		&imx390_sd_2,
		NULL,
	},
	.clk_map = clk_mapping,
};

static void ipu6_quirk(struct pci_dev *pci_dev)
{
	pci_dev->dev.platform_data = &pdata;
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, IPU6_PCI_ID, ipu6_quirk);
