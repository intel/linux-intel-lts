/*
 * Copyright (c) 2016-2017 Intel Corporation.
 *
 * Author: Wang, Zaikuo <zaikuo.wang@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#ifdef CONFIG_MFD_TPS68470
#include <linux/mfd/tps68470.h>
#endif
#include <media/intel-ipu4-isys.h>
#include <media/crlmodule.h>
#include "../../pci/intel-ipu4/intel-ipu4.h"

#define GPIO_BASE		429

#ifdef CONFIG_INTEL_IPU5_PIXTER
#define PIXTER_STUB_LANES	2
#define PIXTER_STUB_B_LANES	2
#define PIXTER_STUB_I2C_ADDRESS	0x7D

static struct crlmodule_platform_data pixter_stub_pdata = {
	.xshutdown = GPIO_BASE + 64,
	.lanes = PIXTER_STUB_LANES,
	.ext_clk = 24000000,
	.module_name = "PIXTER_STUB"
};

static struct intel_ipu4_isys_csi2_config pixter_stub_csi2_cfg = {
	.nlanes = PIXTER_STUB_LANES,
	.port = 0,
};

static struct intel_ipu4_isys_subdev_info pixter_stub_crl_sd = {
	.csi2 = &pixter_stub_csi2_cfg,
	.i2c = {
		.board_info = {
			I2C_BOARD_INFO(CRLMODULE_NAME, PIXTER_STUB_I2C_ADDRESS),
			.platform_data = &pixter_stub_pdata,
		},
		.i2c_adapter_id = 1,
	}
};

static struct crlmodule_platform_data pixter_stub_b_pdata = {
	.xshutdown = GPIO_BASE + 64,
	.lanes = PIXTER_STUB_B_LANES,
	.ext_clk = 24000000,
	.module_name = "PIXTER_STUB_B"
};

static struct intel_ipu4_isys_csi2_config pixter_stub_b_csi2_cfg = {
	.nlanes = PIXTER_STUB_B_LANES,
	.port = 1,
};

static struct intel_ipu4_isys_subdev_info pixter_stub_b_crl_sd = {
	.csi2 = &pixter_stub_b_csi2_cfg,
	.i2c = {
		.board_info = {
			I2C_BOARD_INFO(CRLMODULE_NAME, PIXTER_STUB_I2C_ADDRESS),
			.platform_data = &pixter_stub_b_pdata,
		},
		.i2c_adapter_id = 0,
	}
};
#endif

#ifdef CONFIG_INTEL_IPU5_IMX135
/*
 * The following imx135 platform data is for ipu5.
 */
#define IMX135_LANES		4
#define IMX135_I2C_ADDRESS	0x10

static struct crlmodule_platform_data imx135_pdata = {
	.xshutdown = GPIO_BASE + 71,
	.lanes = IMX135_LANES,
	.ext_clk = 24000000,
	.op_sys_clock = (uint64_t []){ 168000000 },
	.module_name = "IMX135_IPU5",
	.id_string = "0x0 0x135",
};

static struct intel_ipu4_isys_csi2_config imx135_csi2_cfg = {
	.nlanes = IMX135_LANES,
	.port = 0,
};

static struct intel_ipu4_isys_subdev_info imx135_crl_sd = {
	.csi2 = &imx135_csi2_cfg,
	.i2c = {
		.board_info = {
			 I2C_BOARD_INFO(CRLMODULE_NAME, IMX135_I2C_ADDRESS),
			.platform_data = &imx135_pdata,
		},
		.i2c_adapter_id = 0,
	}
};
#endif

#if IS_ENABLED(CONFIG_VIDEO_AD5816G)
#define AD5816G_VCM_ADDR	0x0e
#define AD5816G_NAME		"ad5816g"
static struct intel_ipu4_isys_subdev_info ad5816g_sd = {
	.i2c = {
		.board_info = {
			I2C_BOARD_INFO(AD5816G_NAME,  AD5816G_VCM_ADDR),
		},
		.i2c_adapter_id = 0,
	}
};
#endif

#if IS_ENABLED(CONFIG_VIDEO_VCM_STUB)
#define VCM_STUB_ADDR		0x7f
#define VCM_STUB_NAME		"vcm_stub"
static struct intel_ipu4_isys_subdev_info vcm_stub_sd = {
	.i2c = {
		.board_info = {
			I2C_BOARD_INFO(VCM_STUB_NAME,  VCM_STUB_ADDR),
		},
		.i2c_adapter_id = 0,
	}
};
#endif

#ifdef CONFIG_MFD_TPS68470
#define TPS68470_ADAPTER	0x0

struct tps68470_i2c_check_info {
	u8 bus_nr;
	u16 addr;
	struct i2c_client *client;
};

static struct i2c_board_info tps68470_info = {
	 I2C_BOARD_INFO(TPS68470_NAME, TPS68470_I2C_ID),
};

static int tps68470_i2c_check(struct device *dev, void *priv)
{
	struct i2c_client *client = i2c_verify_client(dev);
	struct tps68470_i2c_check_info *info = priv;

	if (!client)
		return 0;

	if (i2c_adapter_id(client->adapter) != info->bus_nr
	    || client->addr != info->addr)
		return 0;

	info->client = client;

	return 0;
}

static struct i2c_client *tps68470_find_i2c_client(struct i2c_adapter *adapter)
{
	struct tps68470_i2c_check_info info = {
		.bus_nr = i2c_adapter_id(adapter),
		.addr = tps68470_info.addr,
	};
	int rval;

	rval = i2c_for_each_dev(&info, tps68470_i2c_check);
	if (rval || !info.client)
		return NULL;
	return info.client;
}

/**
 * tps68470_resgister_device: register tps68470 device.
 *
 * @adatper_nr: adatper number
 */
void tps68470_register_device(int adapter_nr)
{
	struct i2c_client *client = NULL;
	struct i2c_adapter *adapter =
		i2c_get_adapter(adapter_nr);

	if (adapter == NULL) {
		pr_err("Error: not get adapter %d\n", adapter_nr);
	} else {
		client = tps68470_find_i2c_client(adapter);
		if (client) {
			dev_dbg(&client->dev, "Device exist\n");
		} else {
			pr_info("Register %s device\n", tps68470_info.type);
			request_module(I2C_MODULE_PREFIX "%s",
				tps68470_info.type);
			client = i2c_new_device(adapter, &tps68470_info);
			if (client == NULL)
				pr_err("register pmic device faied\n");
		}
	}
}

#endif

static struct intel_ipu4_isys_clk_mapping clk_mapping[] = {
	{ CLKDEV_INIT(NULL, NULL, NULL), NULL }
};

static struct intel_ipu4_isys_subdev_pdata pdata = {
	.subdevs = (struct intel_ipu4_isys_subdev_info *[]) {
#ifdef CONFIG_INTEL_IPU5_PIXTER
		&pixter_stub_crl_sd,
		&pixter_stub_b_crl_sd,
#endif
#ifdef CONFIG_INTEL_IPU5_IMX135
		&imx135_crl_sd,
#endif
#if IS_ENABLED(CONFIG_VIDEO_AD5816G)
		&ad5816g_sd,
#endif
#if IS_ENABLED(CONFIG_VIDEO_VCM_STUB)
		&vcm_stub_sd,
#endif
		NULL,
	},
	.clk_map = clk_mapping,
};

static void ipu5_quirk(struct pci_dev *pci_dev)
{
	pr_info("FPGA platform data PCI quirk for IPU5\n");
	pci_dev->dev.platform_data = &pdata;
#ifdef CONFIG_MFD_TPS68470
	tps68470_register_device(TPS68470_ADAPTER);
#endif
}

DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL,
	INTEL_IPU5_HW_FPGA_A0, ipu5_quirk);
