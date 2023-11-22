/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2014 - 2022 Intel Corporation */

#ifndef MEDIA_IPU_H
#define MEDIA_IPU_H

#include <linux/i2c.h>
#include <linux/clkdev.h>
#include <linux/version.h>
#include <media/v4l2-async.h>

#define IPU_ISYS_MAX_CSI2_LANES		4

struct ipu_isys_csi2_config {
	unsigned int nlanes;
	unsigned int port;
};

struct ipu_isys_subdev_i2c_info {
	struct i2c_board_info board_info;
	int i2c_adapter_id;
	char i2c_adapter_bdf[32];
};

#if (IS_ENABLED(CONFIG_VIDEO_INTEL_IPU_USE_PLATFORMDATA) \
	&& IS_ENABLED(CONFIG_VIDEO_INTEL_IPU_PDATA_DYNAMIC_LOADING)) \
	|| IS_ENABLED(CONFIG_INTEL_IPU6_ACPI)
#define IPU_SPDATA_NAME_LEN	20
#define IPU_SPDATA_BDF_LEN	32
#define IPU_SPDATA_GPIO_NUM 	4
#define IPU_SPDATA_IRQ_PIN_NAME_LEN 16
#endif

#if IS_ENABLED(CONFIG_VIDEO_INTEL_IPU_USE_PLATFORMDATA) \
	&& IS_ENABLED(CONFIG_VIDEO_INTEL_IPU_PDATA_DYNAMIC_LOADING)
/**
 * struct ipu_spdata_rep - override subdev platform data
 *
 * @name: i2c_board_info.type
 * @i2c_adapter_bdf_o: old i2c adapter bdf
 * @slave_addr_o: old i2c slave address
 * @i2c_adapter_bdf_n: new i2c adapter bdf
 * @slave_addr_n: new i2c slave address
 *
 * identify a subdev with @name, @i2c_adapter_bdf_o and @slave_addr_o and
 * configure it to use the new  @i2c_adapter_bdf_n and @slave_addr_n
 */
struct ipu_spdata_rep {
	/* i2c old information */
	char name[IPU_SPDATA_NAME_LEN];
	unsigned int port_o;
	char i2c_adapter_bdf_o[IPU_SPDATA_BDF_LEN];
	uint32_t slave_addr_o;

	/* i2c new information */
	unsigned int port_n;
	char i2c_adapter_bdf_n[IPU_SPDATA_BDF_LEN];
	uint32_t slave_addr_n;

	/* sensor_platform */
	unsigned int lanes;
	int gpios[IPU_SPDATA_GPIO_NUM];
	int irq_pin;
	unsigned int irq_pin_flags;
	char irq_pin_name[IPU_SPDATA_IRQ_PIN_NAME_LEN];
	char suffix;
};
#endif

struct ipu_isys_subdev_info {
	struct ipu_isys_csi2_config *csi2;
	struct ipu_isys_subdev_i2c_info i2c;
#if IS_ENABLED(CONFIG_VIDEO_INTEL_IPU_USE_PLATFORMDATA) \
	&& IS_ENABLED(CONFIG_VIDEO_INTEL_IPU_PDATA_DYNAMIC_LOADING)
	void (*fixup_spdata)(const void *spdata_rep, void *spdata);
#endif
#if IS_ENABLED(CONFIG_INTEL_IPU6_ACPI)
	char *acpi_hid;
#endif
};

struct ipu_isys_clk_mapping {
	struct clk_lookup clkdev_data;
	char *platform_clock_name;
};

struct ipu_isys_subdev_pdata {
	struct ipu_isys_subdev_info **subdevs;
	struct ipu_isys_clk_mapping *clk_map;
};

struct sensor_async_subdev {
	struct v4l2_async_subdev asd;
	struct ipu_isys_csi2_config csi2;
};

#endif /* MEDIA_IPU_H */
