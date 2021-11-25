// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2020 Intel Corporation
#include <linux/clkdev.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/pci.h>

#include <media/ipu-isys.h>
#include <media/ar0234.h>

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
}

#define AR0234_LANES       2
#define AR0234_I2C_ADDRESS 0x10

static struct ipu_isys_csi2_config ar0234_csi2_cfg_1 = {
	.nlanes = AR0234_LANES,
	.port = 1,
};

static struct ar0234_platform_data ar0234_pdata_1 = {
	.port = 1,
	.lanes = 2,
	.i2c_slave_address = AR0234_I2C_ADDRESS,
	.irq_pin = -1,
	.irq_pin_name = "",
	.irq_pin_flags = IRQF_TRIGGER_RISING
		| IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
	.suffix = 'a',
	.gpios = {-1, 0, 0, 0},
};

static struct ipu_isys_subdev_info ar0234_sd_1 = {
	.csi2 = &ar0234_csi2_cfg_1,
	.i2c = {
	.board_info = {
		I2C_BOARD_INFO("ar0234", AR0234_I2C_ADDRESS),
		.platform_data = &ar0234_pdata_1,
	},
	.i2c_adapter_bdf = "0000:00:15.1",
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
	.irq_pin = -1,
	.irq_pin_name = "",
	.irq_pin_flags = IRQF_TRIGGER_RISING
		| IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
	.suffix = 'b',
	.gpios = {-1, 0, 0, 0},
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

static struct ipu_isys_clk_mapping clk_mapping[] = {
	{ CLKDEV_INIT(NULL, NULL, NULL), NULL }
};

static struct ipu_isys_subdev_pdata pdata = {
	.subdevs = (struct ipu_isys_subdev_info *[]) {
		&ar0234_sd_1,
		&ar0234_sd_2,
		NULL,
	},
	.clk_map = clk_mapping,
};

static void ipu6_quirk(struct pci_dev *pci_dev)
{
	dev_info(&pci_dev->dev, "%s() attach the platform data", __func__);
	pci_dev->dev.platform_data = &pdata;
}

DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, IPU6EP_ADL_P_PCI_ID, ipu6_quirk);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, IPU6EP_ADL_N_PCI_ID, ipu6_quirk);
