// SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
/*
 * Copyright (C) 2018 Intel Corporation
 */

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/pci.h>

#include <media/ipu-isys.h>
#include <media/crlmodule-lite.h>
#include <media/ti964.h>
#include <media/ti960.h>
#include <media/max9286.h>
#include "ipu.h"

#define GPIO_BASE			422

#ifdef CONFIG_INTEL_IPU4_ADV7481

#define ADV7481_CVBS_LANES		1
#define ADV7481_HDMI_LANES		4
#define ADV7481_HDMI_I2C_ADDRESS	0xe0
#define ADV7481_CVBS_I2C_ADDRESS	0xe1
static struct crlmodule_lite_platform_data adv7481_hdmi_pdata_lite = {
#if (!IS_ENABLED(CONFIG_VIDEO_INTEL_UOS))
//	xshutdown GPIO pin unavailable on ACRN UOS
	.xshutdown = GPIO_BASE + 63,
#endif
	.lanes = ADV7481_HDMI_LANES,
	.ext_clk = 24000000,
	.op_sys_clock = (uint64_t []){600000000},
	.module_name = "ADV7481 HDMI"
};
static struct ipu_isys_csi2_config adv7481_hdmi_csi2_cfg = {
	.nlanes = ADV7481_HDMI_LANES,
	.port = 0,
};
static struct ipu_isys_subdev_info adv7481_hdmi_crl_sd_lite = {
	.csi2 = &adv7481_hdmi_csi2_cfg,
	.i2c = {
		.board_info = {
			 .type = CRLMODULE_LITE_NAME,
			 .flags = I2C_CLIENT_TEN,
			 .addr = ADV7481_HDMI_I2C_ADDRESS,
			 .platform_data = &adv7481_hdmi_pdata_lite,
		},
		.i2c_adapter_id = CONFIG_INTEL_IPU4_ADV7481_I2C_ID,
	}
};

static struct crlmodule_lite_platform_data adv7481_cvbs_pdata_lite = {
#if (!IS_ENABLED(CONFIG_VIDEO_INTEL_UOS))
//	xshutdown GPIO pin unavailable on ACRN UOS
	.xshutdown = GPIO_BASE + 63,
#endif
	.lanes = ADV7481_CVBS_LANES,
	.ext_clk = 24000000,
	.op_sys_clock = (uint64_t []){600000000},
	.module_name = "ADV7481 CVBS"
};
static struct ipu_isys_csi2_config adv7481_cvbs_csi2_cfg = {
	.nlanes = ADV7481_CVBS_LANES,
	.port = 4,
};
static struct ipu_isys_subdev_info adv7481_cvbs_crl_sd_lite = {
	.csi2 = &adv7481_cvbs_csi2_cfg,
	.i2c = {
		.board_info = {
			 .type = CRLMODULE_LITE_NAME,
			 .flags = I2C_CLIENT_TEN,
			 .addr = ADV7481_CVBS_I2C_ADDRESS,
			 .platform_data = &adv7481_cvbs_pdata_lite,
		},
		.i2c_adapter_id = CONFIG_INTEL_IPU4_ADV7481_I2C_ID,
	}
};
#endif

#ifdef CONFIG_INTEL_IPU4_ADV7481_EVAL

#define ADV7481_LANES			4
//below i2c address is dummy one, to be able to register single ADV7481 chip as two sensors
#define ADV7481_I2C_ADDRESS		0xe0
#define ADV7481B_I2C_ADDRESS		0xe2

static struct crlmodule_lite_platform_data adv7481_eval_pdata_lite = {
	.xshutdown = GPIO_BASE + 63,
	.lanes = ADV7481_HDMI_LANES,
	.ext_clk = 24000000,
	.op_sys_clock = (uint64_t []){600000000},
	.module_name = "ADV7481_EVAL"
};
static struct ipu_isys_csi2_config adv7481_eval_csi2_cfg = {
	.nlanes = ADV7481_LANES,
	.port = 0,
};
static struct ipu_isys_subdev_info adv7481_eval_crl_sd_lite = {
	.csi2 = &adv7481_eval_csi2_cfg,
	.i2c = {
		.board_info = {
			 .type = CRLMODULE_LITE_NAME,
			 .flags = I2C_CLIENT_TEN,
			 .addr = ADV7481_I2C_ADDRESS,
			 .platform_data = &adv7481_eval_pdata_lite,
		},
		.i2c_adapter_id = 2,
	}
};

static struct crlmodule_lite_platform_data adv7481b_eval_pdata_lite = {
	.xshutdown = GPIO_BASE + 63,
	.lanes = ADV7481_LANES,
	.ext_clk = 24000000,
	.op_sys_clock = (uint64_t []){600000000},
	.module_name = "ADV7481B_EVAL"
};
static struct ipu_isys_csi2_config adv7481b_eval_csi2_cfg = {
	.nlanes = ADV7481_LANES,
	.port = 4,
};
static struct ipu_isys_subdev_info adv7481b_eval_crl_sd_lite = {
	.csi2 = &adv7481b_eval_csi2_cfg,
	.i2c = {
		.board_info = {
			 .type = CRLMODULE_LITE_NAME,
			 .flags = I2C_CLIENT_TEN,
			 .addr = ADV7481B_I2C_ADDRESS,
			 .platform_data = &adv7481b_eval_pdata_lite,
		},
		.i2c_adapter_id = 2,
	}
};
#endif

#ifdef CONFIG_INTEL_IPU4_MAGNA_TI964

#define MAGNA_TI964_MIPI_LANES		4
#define TI964_I2C_ADDRESS		0x3d
static struct crlmodule_lite_platform_data magna_ti964_pdata = {
	.xshutdown = GPIO_BASE + 63,
	.lanes = MAGNA_TI964_MIPI_LANES,
	.ext_clk = 24000000,
		.op_sys_clock = (uint64_t []){ 400000000 },
	.module_name = "MAGNA_TI964",
};
static struct ipu_isys_csi2_config magna_ti964_csi2_cfg = {
	.nlanes = MAGNA_TI964_MIPI_LANES,
	.port = 0,
};
static struct ipu_isys_subdev_info magna_ti964_crl_sd = {
	.csi2 = &magna_ti964_csi2_cfg,
	.i2c = {
		.board_info = {
			I2C_BOARD_INFO(CRLMODULE_LITE_NAME, TI964_I2C_ADDRESS),
			.platform_data = &magna_ti964_pdata,
			   },
		.i2c_adapter_id = 0,
	   }
};

#endif

#ifdef CONFIG_INTEL_IPU4_OV10635
#define OV10635_LANES		4
#define OV10635_I2C_PHY_ADDR	0x60 /* 0x30 for 7bit addr */
#define OV10635A_I2C_ADDRESS	0x61
#define OV10635B_I2C_ADDRESS	0x62
#define OV10635C_I2C_ADDRESS	0x63
#define OV10635D_I2C_ADDRESS	0x64

static struct crlmodule_lite_platform_data ov10635_pdata = {
		.lanes = OV10635_LANES,
		.ext_clk = 24000000,
		.op_sys_clock = (uint64_t []){ 400000000 },
		.module_name = "OV10635",
		.id_string = "0xa6 0x35",

		/*
		 * The pin number of xshutdown will be determined
		 * and replaced inside TI964 driver.
		 * The number here stands for which GPIO to connect with.
		 * 1 means to connect sensor xshutdown to GPIO1
		 */
		.xshutdown = 0,
};
#endif

#ifdef CONFIG_VIDEO_TI964_ICI
#define TI964_I2C_ADAPTER	2
#define TI964_I2C_ADAPTER_2	4
#define TI964_I2C_ADDRESS	0x3d
#define TI964_LANES		4

static struct ipu_isys_csi2_config ti964_csi2_cfg = {
	.nlanes = TI964_LANES,
	.port = 0,
};

static struct ipu_isys_csi2_config ti964_csi2_cfg_2 = {
	.nlanes = TI964_LANES,
	.port = 4,
};
static struct ti964_subdev_info ti964_subdevs[] = {
#ifdef CONFIG_INTEL_IPU4_OV10635
	{
		.board_info = {
			.type = CRLMODULE_LITE_NAME,
			.addr = OV10635A_I2C_ADDRESS,
			.platform_data = &ov10635_pdata,
		},
		.i2c_adapter_id = TI964_I2C_ADAPTER,
		.rx_port = 0,
		.phy_i2c_addr = OV10635_I2C_PHY_ADDR,
		.suffix = 'a',
	},
	{
		.board_info = {
			.type = CRLMODULE_LITE_NAME,
			.addr = OV10635B_I2C_ADDRESS,
			.platform_data = &ov10635_pdata,
		},
		.i2c_adapter_id = TI964_I2C_ADAPTER,
		.rx_port = 1,
		.phy_i2c_addr = OV10635_I2C_PHY_ADDR,
		.suffix = 'b',
	},
	{
		.board_info = {
			.type = CRLMODULE_LITE_NAME,
			.addr = OV10635C_I2C_ADDRESS,
			.platform_data = &ov10635_pdata,
		},
		.i2c_adapter_id = TI964_I2C_ADAPTER,
		.rx_port = 2,
		.phy_i2c_addr = OV10635_I2C_PHY_ADDR,
		.suffix = 'c',
	},
	{
		.board_info = {
			.type = CRLMODULE_LITE_NAME,
			.addr = OV10635D_I2C_ADDRESS,
			.platform_data = &ov10635_pdata,
		},
		.i2c_adapter_id = TI964_I2C_ADAPTER,
		.rx_port = 3,
		.phy_i2c_addr = OV10635_I2C_PHY_ADDR,
		.suffix = 'd',
	},
#endif
};
static struct ti964_subdev_info ti964_subdevs_2[] = {
#ifdef CONFIG_INTEL_IPU4_OV10635
	{
		.board_info = {
			.type = CRLMODULE_LITE_NAME,
			.addr = OV10635A_I2C_ADDRESS,
			.platform_data = &ov10635_pdata,
		},
		.i2c_adapter_id = TI964_I2C_ADAPTER_2,
		.rx_port = 0,
		.phy_i2c_addr = OV10635_I2C_PHY_ADDR,
		.suffix = 'e',
	},
	{
		.board_info = {
			.type = CRLMODULE_LITE_NAME,
			.addr = OV10635B_I2C_ADDRESS,
			.platform_data = &ov10635_pdata,
		},
		.i2c_adapter_id = TI964_I2C_ADAPTER_2,
		.rx_port = 1,
		.phy_i2c_addr = OV10635_I2C_PHY_ADDR,
		.suffix = 'f',
	},
	{
		.board_info = {
			.type = CRLMODULE_LITE_NAME,
			.addr = OV10635C_I2C_ADDRESS,
			.platform_data = &ov10635_pdata,
		},
		.i2c_adapter_id = TI964_I2C_ADAPTER_2,
		.rx_port = 2,
		.phy_i2c_addr = OV10635_I2C_PHY_ADDR,
		.suffix = 'g',
	},
	{
		.board_info = {
			.type = CRLMODULE_LITE_NAME,
			.addr = OV10635D_I2C_ADDRESS,
			.platform_data = &ov10635_pdata,
		},
		.i2c_adapter_id = TI964_I2C_ADAPTER_2,
		.rx_port = 3,
		.phy_i2c_addr = OV10635_I2C_PHY_ADDR,
		.suffix = 'h',
	},
#endif
};
static struct ti964_pdata ti964_pdata = {
	.subdev_info = ti964_subdevs,
	.subdev_num = ARRAY_SIZE(ti964_subdevs),
	.reset_gpio = GPIO_BASE + 62,
	.suffix = 'a',
};

static struct ipu_isys_subdev_info ti964_sd = {
	.csi2 = &ti964_csi2_cfg,
	.i2c = {
		.board_info = {
			 .type = "ti964",
			 .addr = TI964_I2C_ADDRESS,
			 .platform_data = &ti964_pdata,
		},
		.i2c_adapter_id = TI964_I2C_ADAPTER,
	}
};

static struct ti964_pdata ti964_pdata_2 = {
	.subdev_info = ti964_subdevs_2,
	.subdev_num = ARRAY_SIZE(ti964_subdevs_2),
	.reset_gpio = GPIO_BASE + 69,
	.suffix = 'b',
};

static struct ipu_isys_subdev_info ti964_sd_2 = {
	.csi2 = &ti964_csi2_cfg_2,
	.i2c = {
		.board_info = {
			 .type = "ti964",
			 .addr = TI964_I2C_ADDRESS,
			 .platform_data = &ti964_pdata_2,
		},
		.i2c_adapter_id = TI964_I2C_ADAPTER_2,
	}
};
#endif

#ifdef CONFIG_INTEL_IPU4_AR0231AT
#define AR0231AT_LANES			  4
#define AR0231ATA_I2C_ADDRESS	   0x11
#define AR0231ATB_I2C_ADDRESS	   0x12
#define AR0231ATC_I2C_ADDRESS	   0x13
#define AR0231ATD_I2C_ADDRESS	   0x14

static struct crlmodule_lite_platform_data ar0231at_pdata = {
		.lanes = AR0231AT_LANES,
		.ext_clk = 27000000,
		.op_sys_clock = (uint64_t[]){ 87750000 },
		.module_name = "AR0231AT",
};
#endif

#if IS_ENABLED(CONFIG_VIDEO_MAX9286_ICI)
#define DS_MAX9286_LANES				4
#define DS_MAX9286_I2C_ADAPTER			4
#define DS_MAX9286_I2C_ADDRESS			0x48

static struct ipu_isys_csi2_config max9286_csi2_cfg = {
		.nlanes = DS_MAX9286_LANES,
		.port = 4,
};

static struct max9286_subdev_i2c_info max9286_subdevs[] = {
#ifdef CONFIG_INTEL_IPU4_AR0231AT
				{
						.board_info = {
								.type = CRLMODULE_LITE_NAME,
								.addr = AR0231ATA_I2C_ADDRESS,
								.platform_data = &ar0231at_pdata,
						},
						.i2c_adapter_id = DS_MAX9286_I2C_ADAPTER,
						.suffix = 'a',
				},
				{
						.board_info = {
								.type = CRLMODULE_LITE_NAME,
								.addr = AR0231ATB_I2C_ADDRESS,
								.platform_data = &ar0231at_pdata,
						},
						.i2c_adapter_id = DS_MAX9286_I2C_ADAPTER,
						.suffix = 'b',
				},
				{
						.board_info = {
								.type = CRLMODULE_LITE_NAME,
								.addr = AR0231ATC_I2C_ADDRESS,
								.platform_data = &ar0231at_pdata,
						},
						.i2c_adapter_id = DS_MAX9286_I2C_ADAPTER,
						.suffix = 'c',
				},
				{
						.board_info = {
								.type = CRLMODULE_LITE_NAME,
								.addr = AR0231ATD_I2C_ADDRESS,
								.platform_data = &ar0231at_pdata,
						},
						.i2c_adapter_id = DS_MAX9286_I2C_ADAPTER,
						.suffix = 'd',
				},
#endif
};


static struct max9286_pdata max9286_pdata = {
		.subdev_info = max9286_subdevs,
		.subdev_num = ARRAY_SIZE(max9286_subdevs),
		.reset_gpio = GPIO_BASE + 63,
		.suffix = 'a',
};

static struct ipu_isys_subdev_info max9286_sd = {
		.csi2 = &max9286_csi2_cfg,
		.i2c = {
				.board_info = {
						.type = "max9286",
						.addr = DS_MAX9286_I2C_ADDRESS,
						.platform_data = &max9286_pdata,
				},
				.i2c_adapter_id = DS_MAX9286_I2C_ADAPTER,
		}
};
#endif

#ifdef CONFIG_INTEL_IPU4_OV495
#define OV495_LANES	4
#define OV495_I2C_PHY_ADDR	0x48
#define OV495A_I2C_ADDRESS	0x30
#define OV495B_I2C_ADDRESS	0x31
#define OV495C_I2C_ADDRESS	0x32
#define OV495D_I2C_ADDRESS	0x33

#define OV495A_SER_ADDRESS	0x58
#define OV495B_SER_ADDRESS	0x59
#define OV495C_SER_ADDRESS	0x5a
#define OV495D_SER_ADDRESS	0x5b

static struct crlmodule_lite_platform_data ov495_pdata = {
	.lanes = OV495_LANES,
	.ext_clk = 27000000,
	.op_sys_clock = (uint64_t[]){ 87750000 },
	.module_name = "OV495",
	.id_string = "0x51 0x49 0x56 0x4f",
	/*
	 * TI960 has 4 gpio pins, for PWDN, FSIN, and etc.
	 * it depends connection between serializer and sensor,
	 * please specify xshutdown, fsin as needed.
	 */
	.fsin = 2, /* gpio 2 used for FSIN */
};
#endif

#if IS_ENABLED(CONFIG_VIDEO_TI960_ICI)
#define TI960_I2C_ADAPTER	2
#define TI960_I2C_ADAPTER_2	4
#define	TI960_LANES	4

static struct ipu_isys_csi2_config ti960_csi2_cfg = {
	.nlanes = TI960_LANES,
	.port = 0,
};

static struct ipu_isys_csi2_config ti960_csi2_cfg_2 = {
	.nlanes = TI960_LANES,
	.port = 4,
};

static struct ti960_subdev_info ti960_subdevs[] = {
#ifdef CONFIG_INTEL_IPU4_OV495
	{
		.board_info = {
			.type = CRLMODULE_LITE_NAME,
			.addr = OV495A_I2C_ADDRESS,
			.platform_data = &ov495_pdata,
		},
		.i2c_adapter_id = TI960_I2C_ADAPTER,
		.rx_port = 0,
		.phy_i2c_addr = OV495_I2C_PHY_ADDR,
		.ser_alias = OV495A_SER_ADDRESS,
		.suffix = 'a',
	},
	{
		.board_info = {
			.type = CRLMODULE_LITE_NAME,
			.addr = OV495B_I2C_ADDRESS,
			.platform_data = &ov495_pdata,
		},
		.i2c_adapter_id = TI960_I2C_ADAPTER,
		.rx_port = 1,
		.phy_i2c_addr = OV495_I2C_PHY_ADDR,
		.ser_alias = OV495B_SER_ADDRESS,
		.suffix = 'b',
	},
	{
		.board_info = {
			.type = CRLMODULE_LITE_NAME,
			.addr = OV495C_I2C_ADDRESS,
			.platform_data = &ov495_pdata,
		},
		.i2c_adapter_id = TI960_I2C_ADAPTER,
		.rx_port = 2,
		.phy_i2c_addr = OV495_I2C_PHY_ADDR,
		.ser_alias = OV495C_SER_ADDRESS,
		.suffix = 'c',
	},
	{
		.board_info = {
			.type = CRLMODULE_LITE_NAME,
			.addr = OV495D_I2C_ADDRESS,
			.platform_data = &ov495_pdata,
		},
		.i2c_adapter_id = TI960_I2C_ADAPTER,
		.rx_port = 3,
		.phy_i2c_addr = OV495_I2C_PHY_ADDR,
		.ser_alias = OV495D_SER_ADDRESS,
		.suffix = 'd',
	},

#endif
};

static struct ti960_subdev_info ti960_subdevs_2[] = {
#ifdef CONFIG_INTEL_IPU4_OV495
	{
		.board_info = {
			.type = CRLMODULE_LITE_NAME,
			.addr = OV495A_I2C_ADDRESS,
			.platform_data = &ov495_pdata,
		},
		.i2c_adapter_id = TI960_I2C_ADAPTER_2,
		.rx_port = 0,
		.phy_i2c_addr = OV495_I2C_PHY_ADDR,
		.ser_alias = OV495A_SER_ADDRESS,
		.suffix = 'e',
	},
	{
		.board_info = {
			.type = CRLMODULE_LITE_NAME,
			.addr = OV495B_I2C_ADDRESS,
			.platform_data = &ov495_pdata,
		},
		.i2c_adapter_id = TI960_I2C_ADAPTER_2,
		.rx_port = 1,
		.phy_i2c_addr = OV495_I2C_PHY_ADDR,
		.ser_alias = OV495B_SER_ADDRESS,
		.suffix = 'f',
	},
	{
		.board_info = {
			.type = CRLMODULE_LITE_NAME,
			.addr = OV495C_I2C_ADDRESS,
			.platform_data = &ov495_pdata,
		},
		.i2c_adapter_id = TI960_I2C_ADAPTER_2,
		.rx_port = 2,
		.phy_i2c_addr = OV495_I2C_PHY_ADDR,
		.ser_alias = OV495C_SER_ADDRESS,
		.suffix = 'g',
	},
	{
		.board_info = {
			.type = CRLMODULE_LITE_NAME,
			.addr = OV495D_I2C_ADDRESS,
			.platform_data = &ov495_pdata,
		},
		.i2c_adapter_id = TI960_I2C_ADAPTER_2,
		.rx_port = 3,
		.phy_i2c_addr = OV495_I2C_PHY_ADDR,
		.ser_alias = OV495D_SER_ADDRESS,
		.suffix = 'h',
	},
#endif
};

static struct ti960_pdata ti960_pdata = {
	.subdev_info = ti960_subdevs,
	.subdev_num = ARRAY_SIZE(ti960_subdevs),
	.reset_gpio = GPIO_BASE + 63,
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
		.i2c_adapter_id = TI960_I2C_ADAPTER,
	}
};

static struct ti960_pdata ti960_pdata_2 = {
	.subdev_info = ti960_subdevs_2,
	.subdev_num = ARRAY_SIZE(ti960_subdevs_2),
	.reset_gpio = GPIO_BASE + 66,
	.suffix = 'b',
};

static struct ipu_isys_subdev_info ti960_sd_2 = {
	.csi2 = &ti960_csi2_cfg_2,
	.i2c = {
		.board_info = {
			 .type = "ti960",
			 .addr = TI960_I2C_ADDRESS,
			 .platform_data = &ti960_pdata_2,
		},
		.i2c_adapter_id = TI960_I2C_ADAPTER_2,
	}
};
#endif

/*
 * Map buttress output sensor clocks to sensors -
 * this should be coming from ACPI
 */
struct ipu_isys_clk_mapping p_mapping[] = {
	{ CLKDEV_INIT("0-003d", NULL, NULL), "OSC_CLK_OUT1" },
	{ CLKDEV_INIT("0-00e1", NULL, NULL), "OSC_CLK_OUT0" },
	{ CLKDEV_INIT("0-00e0", NULL, NULL), "OSC_CLK_OUT1" },
	{ CLKDEV_INIT("2-a0e0", NULL, NULL), "OSC_CLK_OUT0" },
	{ CLKDEV_INIT("2-a0e2", NULL, NULL), "OSC_CLK_OUT1" },
	{ CLKDEV_INIT(NULL, NULL, NULL), NULL }
};

static struct ipu_isys_subdev_pdata pdata = {
	.subdevs = (struct ipu_isys_subdev_info *[]) {
#ifdef CONFIG_INTEL_IPU4_ADV7481
		&adv7481_cvbs_crl_sd_lite,
		&adv7481_hdmi_crl_sd_lite,
#endif
#ifdef CONFIG_INTEL_IPU4_ADV7481_EVAL
		&adv7481_eval_crl_sd_lite,
		&adv7481b_eval_crl_sd_lite,
#endif
#ifdef CONFIG_VIDEO_TI964_ICI
		&ti964_sd,
		&ti964_sd_2,
#endif
#ifdef CONFIG_INTEL_IPU4_MAGNA_TI964
		&magna_ti964_crl_sd,
#endif
#if IS_ENABLED(CONFIG_VIDEO_MAX9286_ICI)
		&max9286_sd,
#endif
#if IS_ENABLED(CONFIG_VIDEO_TI960_ICI)
		&ti960_sd,
		&ti960_sd_2,
#endif
		NULL,
	},
	.clk_map = p_mapping,
};

static void ipu4_quirk(struct pci_dev *pci_dev)
{
	pci_dev->dev.platform_data = &pdata;
}

DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, IPU_PCI_ID,
			ipu4_quirk);
