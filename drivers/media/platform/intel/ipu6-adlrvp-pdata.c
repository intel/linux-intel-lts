// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2022 Intel Corporation
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/pci.h>

#include <media/ipu-isys.h>
#include <media/ti960.h>
#include <media/ar0234.h>
#include <media/imx390.h>
#include <media/lt6911uxc.h>

#include "ipu.h"

static void ar0234_fixup_spdata(const void *spdata_rep, void *spdata)
{
	const struct ipu_spdata_rep *rep = spdata_rep;
	struct ar0234_platform_data *platform = spdata;

	if (spdata_rep && spdata) {
		platform->port = rep->port_n;
		platform->lanes = rep->lanes;
		platform->i2c_slave_address = rep->slave_addr_n;
		platform->gpios[0] = rep->gpios[0];
		platform->irq_pin = rep->irq_pin;
		platform->irq_pin_flags = rep->irq_pin_flags;
		strcpy(platform->irq_pin_name, rep->irq_pin_name);
		platform->suffix = rep->suffix;
	}
}

static void lt6911uxc_fixup_spdata(const void *spdata_rep, void *spdata)
{
	const struct ipu_spdata_rep *rep = spdata_rep;
	struct lt6911uxc_platform_data *platform = spdata;

	if (spdata_rep && spdata) {
		platform->port = rep->port_n;
		platform->lanes = rep->lanes;
		platform->i2c_slave_address = rep->slave_addr_n;
		platform->gpios[0] = rep->gpios[0];
		platform->irq_pin = rep->irq_pin;
		platform->irq_pin_flags = rep->irq_pin_flags;
		strcpy(platform->irq_pin_name, rep->irq_pin_name);
		platform->suffix = rep->suffix;
	}
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

#if IS_ENABLED(CONFIG_VIDEO_IMX390)
#define IMX390_LANES       4
#define IMX390_D3RCM_I2C_ADDRESS 0x1a
#define IMX390_D3RCM_I2C_ADDRESS_8BIT (IMX390_D3RCM_I2C_ADDRESS << 1)
#define IMX390_D3CM_I2C_ADDRESS 0x21
#define IMX390_D3CM_I2C_ADDRESS_8BIT (IMX390_D3CM_I2C_ADDRESS << 1)
#define IMX390_I2C_ADDRESS_3 0x1e
#define IMX390_I2C_ADDRESS_8BIT_3 (IMX390_I2C_ADDRESS_3 << 1)
#define IMX390_I2C_ADDRESS_4 0x20
#define IMX390_I2C_ADDRESS_8BIT_4 (IMX390_I2C_ADDRESS_4 << 1)

static struct ipu_isys_csi2_config imx390_csi2_cfg_1 = {
	.nlanes = IMX390_LANES,
	.port = 1,
};

static struct imx390_platform_data imx390_pdata_1 = {
	.port = 1,
	.lanes = 4,
	.i2c_slave_address = IMX390_I2C_ADDRESS_3,
	.suffix = 'a',
};

static struct ipu_isys_subdev_info imx390_sd_1 = {
	.csi2 = &imx390_csi2_cfg_1,
	.i2c = {
	.board_info = {
		I2C_BOARD_INFO("imx390", IMX390_I2C_ADDRESS_3),
		.platform_data = &imx390_pdata_1,
	},
	.i2c_adapter_bdf = "0000:00:15.1",
	},
};

static struct ipu_isys_csi2_config imx390_csi2_cfg_2 = {
	.nlanes = IMX390_LANES,
	.port = 2,
};

static struct imx390_platform_data imx390_pdata_2 = {
	.port = 2,
	.lanes = 4,
	.i2c_slave_address = IMX390_I2C_ADDRESS_3,
	.suffix = 'b',
};

static struct ipu_isys_subdev_info imx390_sd_2 = {
	.csi2 = &imx390_csi2_cfg_2,
	.i2c = {
	.board_info = {
		I2C_BOARD_INFO("imx390", IMX390_I2C_ADDRESS_3),
		.platform_data = &imx390_pdata_2,
	},
	.i2c_adapter_bdf = "0000:00:19.1",
	},
};

static struct ipu_isys_csi2_config imx390_csi2_cfg_3 = {
	.nlanes = IMX390_LANES,
	.port = 1,
};

static struct imx390_platform_data imx390_pdata_3 = {
	.port = 1,
	.lanes = 4,
	.i2c_slave_address = IMX390_I2C_ADDRESS_4,
	.suffix = 'a',
};

static struct ipu_isys_subdev_info imx390_sd_3 = {
	.csi2 = &imx390_csi2_cfg_1,
	.i2c = {
	.board_info = {
		I2C_BOARD_INFO("imx390", IMX390_I2C_ADDRESS_4),
		.platform_data = &imx390_pdata_1,
	},
	.i2c_adapter_bdf = "0000:00:15.1",
	},
};

static struct ipu_isys_csi2_config imx390_csi2_cfg_4 = {
	.nlanes = IMX390_LANES,
	.port = 2,
};

static struct imx390_platform_data imx390_pdata_4 = {
	.port = 2,
	.lanes = 4,
	.i2c_slave_address = IMX390_I2C_ADDRESS_4,
	.suffix = 'b',
};

static struct ipu_isys_subdev_info imx390_sd_4 = {
	.csi2 = &imx390_csi2_cfg_2,
	.i2c = {
	.board_info = {
		I2C_BOARD_INFO("imx390", IMX390_I2C_ADDRESS_4),
		.platform_data = &imx390_pdata_2,
	},
	.i2c_adapter_bdf = "0000:00:19.1",
	},
};
#endif

#if IS_ENABLED(CONFIG_VIDEO_TI960)
#define TI960_I2C_ADAPTER	2
#define TI960_I2C_ADAPTER_2	4
#define TI960_LANES	4

#define IMX390A_ADDRESS		0x44
#define IMX390B_ADDRESS		0x45
#define IMX390C_ADDRESS		0x46
#define IMX390D_ADDRESS		0x47

#define IMX390A_SER_ADDRESS	0x40
#define IMX390B_SER_ADDRESS	0x41
#define IMX390C_SER_ADDRESS	0x42
#define IMX390D_SER_ADDRESS	0x43

static struct ti960_subdev_pdata imx390_d3rcm_pdata_stub = {
	.lanes = 4,
	.gpio_powerup_seq = {0, 0xa, -1, -1},
	.module_flags = TI960_FL_POWERUP | TI960_FL_INIT_SER_CLK,
	.module_name = "imx390",
	.fsin = 0, /* gpio 0 used for FSIN */
};

static struct ti960_subdev_pdata imx390_d3cm_pdata_stub = {
	.lanes = 4,
	.gpio_powerup_seq = {0, 0x9, -1, -1},
	.module_flags = TI960_FL_POWERUP | TI960_FL_INIT_SER_CLK,
	.module_name = "imx390",
	.fsin = 3, /* gpio 3 used for FSIN */
};

static struct ipu_isys_csi2_config ti960_csi2_cfg = {
	.nlanes = TI960_LANES,
	.port = 1,
};

static struct ti960_subdev_info ti960_subdevs[] = {
#if IS_ENABLED(CONFIG_VIDEO_IMX390)
	/* D3RCM */
	{
		.board_info = {
			.type = "imx390",
			.addr = IMX390A_ADDRESS,
			.platform_data = &imx390_d3rcm_pdata_stub,
		},
		.rx_port = 0,
		.phy_i2c_addr = IMX390_D3RCM_I2C_ADDRESS_8BIT,
		.ser_alias = IMX390A_SER_ADDRESS,
		.suffix = 'a',
	},
	{
		.board_info = {
			.type = "imx390",
			.addr = IMX390B_ADDRESS,
			.platform_data = &imx390_d3rcm_pdata_stub,
		},
		.rx_port = 1,
		.phy_i2c_addr = IMX390_D3RCM_I2C_ADDRESS_8BIT,
		.ser_alias = IMX390B_SER_ADDRESS,
		.suffix = 'b',
	},
	{
		.board_info = {
			.type = "imx390",
			.addr = IMX390C_ADDRESS,
			.platform_data = &imx390_d3rcm_pdata_stub,
		},
		.rx_port = 2,
		.phy_i2c_addr = IMX390_D3RCM_I2C_ADDRESS_8BIT,
		.ser_alias = IMX390C_SER_ADDRESS,
		.suffix = 'c',
	},
	{
		.board_info = {
			.type = "imx390",
			.addr = IMX390D_ADDRESS,
			.platform_data = &imx390_d3rcm_pdata_stub,
		},
		.rx_port = 3,
		.phy_i2c_addr = IMX390_D3RCM_I2C_ADDRESS_8BIT,
		.ser_alias = IMX390D_SER_ADDRESS,
		.suffix = 'd',
	},
	/* D3CM */
	{
		.board_info = {
			.type = "imx390",
			.addr = IMX390A_ADDRESS,
			.platform_data = &imx390_d3cm_pdata_stub,
		},
		.rx_port = 0,
		.phy_i2c_addr = IMX390_D3CM_I2C_ADDRESS_8BIT,
		.ser_alias = IMX390A_SER_ADDRESS,
		.suffix = 'a',
	},
	{
		.board_info = {
			.type = "imx390",
			.addr = IMX390B_ADDRESS,
			.platform_data = &imx390_d3cm_pdata_stub,
		},
		.rx_port = 1,
		.phy_i2c_addr = IMX390_D3CM_I2C_ADDRESS_8BIT,
		.ser_alias = IMX390B_SER_ADDRESS,
		.suffix = 'b',
	},
	{
		.board_info = {
			.type = "imx390",
			.addr = IMX390C_ADDRESS,
			.platform_data = &imx390_d3cm_pdata_stub,
		},
		.rx_port = 2,
		.phy_i2c_addr = IMX390_D3CM_I2C_ADDRESS_8BIT,
		.ser_alias = IMX390C_SER_ADDRESS,
		.suffix = 'c',
	},
	{
		.board_info = {
			.type = "imx390",
			.addr = IMX390D_ADDRESS,
			.platform_data = &imx390_d3cm_pdata_stub,
		},
		.rx_port = 3,
		.phy_i2c_addr = IMX390_D3CM_I2C_ADDRESS_8BIT,
		.ser_alias = IMX390D_SER_ADDRESS,
		.suffix = 'd',
	},
#endif
};

static struct ti960_pdata ti960_pdata = {
	.subdev_info = ti960_subdevs,
	.subdev_num = ARRAY_SIZE(ti960_subdevs),
	.reset_gpio = 0,
	.FPD_gpio = 175,
	.suffix = 'a',
};

static struct ipu_isys_subdev_info ti960_sd = {
	.csi2 = &ti960_csi2_cfg,
	.i2c = {
		.board_info = {
			 .type = "ti960",
			 .addr = TI960_I2C_ADDRESS,
			 .platform_data = &ti960_pdata,
		},
		.i2c_adapter_bdf = "0000:00:15.1",
	}
};
#endif

#define LT6911UXC_LANES       4
#define LT6911UXC_I2C_ADDRESS 0x2B

static struct ipu_isys_csi2_config lt6911uxc_csi2_cfg_1 = {
	.nlanes = LT6911UXC_LANES,
	.port = 1,
};

static struct lt6911uxc_platform_data lt6911uxc_pdata_1 = {
	.port = 1,
	.lanes = LT6911UXC_LANES,
	.i2c_slave_address = LT6911UXC_I2C_ADDRESS,
	.irq_pin = 749,
	.irq_pin_name = "READY_STAT",
	.irq_pin_flags = IRQF_TRIGGER_RISING
		| IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
	.suffix = 'a',
	.reset_pin = 957,
	.detect_pin = 687,
	.gpios = {-1, 0, 0, 0},
};

static struct ipu_isys_subdev_info  lt6911uxc_sd_1 = {
	.csi2 = &lt6911uxc_csi2_cfg_1,
	.i2c = {
	.board_info = {
		I2C_BOARD_INFO("lt6911uxc", LT6911UXC_I2C_ADDRESS),
		.platform_data = &lt6911uxc_pdata_1,
	},
	.i2c_adapter_bdf = "0000:00:15.1",
	},
	.fixup_spdata = lt6911uxc_fixup_spdata,
};

static struct ipu_isys_csi2_config lt6911uxc_csi2_cfg_2 = {
	.nlanes = LT6911UXC_LANES,
	.port = 2,
};

static struct lt6911uxc_platform_data lt6911uxc_pdata_2 = {
	.port = 2,
	.lanes = LT6911UXC_LANES,
	.i2c_slave_address = LT6911UXC_I2C_ADDRESS,
	.irq_pin = 956,
	.irq_pin_name = "READY_STAT",
	.irq_pin_flags = IRQF_TRIGGER_RISING
		| IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
	.suffix = 'b',
	.reset_pin = 999,
	.detect_pin = 1000,
	.gpios = {-1, 0, 0, 0},
};

static struct ipu_isys_subdev_info lt6911uxc_sd_2 = {
	.csi2 = &lt6911uxc_csi2_cfg_2,
	.i2c = {
	.board_info = {
		I2C_BOARD_INFO("lt6911uxc", LT6911UXC_I2C_ADDRESS),
		.platform_data = &lt6911uxc_pdata_2,
	},
	.i2c_adapter_bdf = "0000:00:19.1",
	},
	.fixup_spdata = lt6911uxc_fixup_spdata,
};
static struct ipu_isys_clk_mapping clk_mapping[] = {
	{ CLKDEV_INIT(NULL, NULL, NULL), NULL }
};

static struct ipu_isys_subdev_pdata pdata = {
	.subdevs = (struct ipu_isys_subdev_info *[]) {
		&ar0234_sd_1,
		&ar0234_sd_2,
#if IS_ENABLED(CONFIG_VIDEO_TI960)
		//&ti960_sd,
		//&ti960_sd_2,
#endif
		&lt6911uxc_sd_1,
		&lt6911uxc_sd_2,
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
