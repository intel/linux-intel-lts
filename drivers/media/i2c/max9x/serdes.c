/*
 * serdes.c
 *
 * Copyright (c) 2018-2020 D3 Engineering.  All rights reserved.
 * Copyright (c) 2023, Define Design Deploy Corp.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2025-2026 Intel Corporation

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/machine.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/of_gpio.h>
#include <linux/overflow.h>

#include "serdes.h"
#include "regmap-retry.h"

#include <media/ipu-acpi-pdata.h>

static const s64 max9x_op_sys_clock[] =  {
	MAX9X_LINK_FREQ_MBPS_TO_HZ(2500),
	MAX9X_LINK_FREQ_MBPS_TO_HZ(2400),
	MAX9X_LINK_FREQ_MBPS_TO_HZ(2300),
	MAX9X_LINK_FREQ_MBPS_TO_HZ(2200),
	MAX9X_LINK_FREQ_MBPS_TO_HZ(2100),
	MAX9X_LINK_FREQ_MBPS_TO_HZ(2000),
	MAX9X_LINK_FREQ_MBPS_TO_HZ(1900),
	MAX9X_LINK_FREQ_MBPS_TO_HZ(1800),
	MAX9X_LINK_FREQ_MBPS_TO_HZ(1700),
	MAX9X_LINK_FREQ_MBPS_TO_HZ(1600),
	MAX9X_LINK_FREQ_MBPS_TO_HZ(1500),
	MAX9X_LINK_FREQ_MBPS_TO_HZ(1400),
	MAX9X_LINK_FREQ_MBPS_TO_HZ(1300),
	MAX9X_LINK_FREQ_MBPS_TO_HZ(1200),
	MAX9X_LINK_FREQ_MBPS_TO_HZ(1100),
	MAX9X_LINK_FREQ_MBPS_TO_HZ(1000),
	MAX9X_LINK_FREQ_MBPS_TO_HZ(900),
	MAX9X_LINK_FREQ_MBPS_TO_HZ(800),
	MAX9X_LINK_FREQ_MBPS_TO_HZ(700),
	MAX9X_LINK_FREQ_MBPS_TO_HZ(600),
	MAX9X_LINK_FREQ_MBPS_TO_HZ(500),
	MAX9X_LINK_FREQ_MBPS_TO_HZ(400),
	MAX9X_LINK_FREQ_MBPS_TO_HZ(300),
	MAX9X_LINK_FREQ_MBPS_TO_HZ(200),
	MAX9X_LINK_FREQ_MBPS_TO_HZ(100),
	MAX9X_LINK_FREQ_MBPS_TO_HZ(80),
};

int max9x_get_ops(char dev_id,
		struct max9x_common_ops **common_ops,
		struct max9x_serial_link_ops **serial_ops,
		struct max9x_csi_link_ops **csi_ops,
		struct max9x_line_fault_ops **lf_ops,
		struct max9x_translation_ops **trans_ops)
{
	int rval = 0;

	switch (dev_id) {
	case MAX9296:
		rval = max9296_get_ops(common_ops, serial_ops, csi_ops, lf_ops, trans_ops);
		break;
	case MAX96724:
		rval = max96724_get_ops(common_ops, serial_ops, csi_ops, lf_ops, trans_ops);
		break;
	case MAX9295:
		rval = max9295_get_ops(common_ops, serial_ops, csi_ops, lf_ops, trans_ops);
		break;
	case MAX96717:
		rval = max96717_get_ops(common_ops, serial_ops, csi_ops, lf_ops, trans_ops);
		break;
	default:
		break;
	}

	return rval;
}

static struct max9x_desc max9x_chips[] = {
	[MAX9296] = {
		.dev_id = 0x94,
		.rev_reg = 0xE,
		.serdes_type = MAX9X_DESERIALIZER,
		.chip_type = MAX9296,
		.get_max9x_ops = max9x_get_ops,
	},
	[MAX96724] = {
		.dev_id = 0xA2,
		.rev_reg = 0x4C,
		.serdes_type = MAX9X_DESERIALIZER,
		.chip_type = MAX96724,
		.get_max9x_ops = max9x_get_ops,
	},
	[MAX9295] = {
		.dev_id = 0x91,
		.rev_reg = 0xE,
		.serdes_type = MAX9X_SERIALIZER,
		.chip_type = MAX9295,
		.get_max9x_ops = max9x_get_ops,
	},
	/*need to check dev_id and others when used*/
	[MAX96717] = {
		.dev_id = 0x91,
		.rev_reg = 0xE,
		.serdes_type = MAX9X_SERIALIZER,
		.chip_type = MAX96717,
		.get_max9x_ops = max9x_get_ops,
	},
};

static const struct of_device_id max9x_of_match[] = {
	{ .compatible = "max9x,max9296", .data = &max9x_chips[MAX9296] },
	{ .compatible = "max9x,max96724", .data = &max9x_chips[MAX96724] },
	{ .compatible = "max9x,max9295", .data = &max9x_chips[MAX9295] },
	{ .compatible = "max9x,max96717", .data = &max9x_chips[MAX96717] },
	{}
};
MODULE_DEVICE_TABLE(of, max9x_of_match);

static const struct i2c_device_id max9x_id[] = {
	{ "max9x", 0 },
	{ "max9296", MAX9296 },
	{ "max96724", MAX96724 },
	{ "max9295", MAX9295 },
	{ "max96717", MAX96717 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max9x_id);

typedef int (*max9x_serdes_parse_child_func)(struct max9x_common *common, struct device_node *node);

static int max9x_soft_reset(struct max9x_common *common);
static int max9x_remap_addr(struct max9x_common *common);
static int max9x_setup_gpio(struct max9x_common *common);
static int max9x_enable(struct max9x_common *common);
static int max9x_disable(struct max9x_common *common);
static int max9x_verify_devid(struct max9x_common *common);

static int max9x_enable_resume(struct max9x_common *common);
static int max9x_remap_serializers_resume(struct max9x_common *common, unsigned int link_id);
static int max9x_create_adapters_resume(struct max9x_common *common);

static int max9x_remap_serializers(struct max9x_common *common, unsigned int link_id);
static int max9x_create_adapters(struct max9x_common *common);
static int max9x_csi_link_to_pad(struct max9x_common *common, int csi_id);
static int max9x_serial_link_to_pad(struct max9x_common *common, int link_id);
static int max9x_register_v4l_subdev(struct max9x_common *common);
static int max9x_enable_serial_link(struct max9x_common *common, unsigned int link_id);
static int max9x_disable_serial_link(struct max9x_common *common, unsigned int link_id);
static int max9x_sysfs_create_get_link(struct max9x_common *common, unsigned int link_id);
static void max9x_sysfs_destroy_get_link(struct max9x_common *common, unsigned int link_id);

static int max9x_enable_line_faults(struct max9x_common *common);
static int max9x_disable_line_faults(struct max9x_common *common);
static void max9x_sysfs_destroy_line_fault_status(struct max9x_common *common, unsigned int line);

static int max9x_parse_pdata(struct max9x_common *common, struct max9x_pdata *pdata);
static int max9x_parse_serial_link_pdata(struct max9x_common *common,
					 struct max9x_serial_link_pdata *max9x_serial_link_pdata);
static int max9x_parse_video_pipe_pdata(struct max9x_common *common,
					struct max9x_video_pipe_pdata *video_pipe_pdata);
static int max9x_parse_csi_link_pdata(struct max9x_common *common, struct max9x_csi_link_pdata *csi_link_pdata);
static int max9x_parse_subdev_pdata(struct max9x_common *common, struct max9x_subdev_pdata *subdev_pdata);

static int max9x_select_i2c_chan(struct i2c_mux_core *muxc, u32 chan_id);
static int max9x_deselect_i2c_chan(struct i2c_mux_core *muxc, u32 chan_id);

static int max9x_des_isolate_serial_link(struct max9x_common *common, unsigned int link_id);
static int max9x_des_deisolate_serial_link(struct max9x_common *common, unsigned int link_id);

static ssize_t max9x_link_status_show(struct device *dev, struct device_attribute *attr, char *buf);

static int max9x_setup_translations(struct max9x_common *common);
static int max9x_disable_translations(struct max9x_common *common);

#define MAX9X_ALLOCATE_ELEMENTS(common, element_type, elements, num_elements) ({ \
	struct device *dev = common->dev; \
	int allocate_ret = 0; \
	(common)->num_elements = 0; \
	(common)->elements = NULL; \
	if ((common)->common_ops && (common)->common_ops->max_elements) { \
		(common)->num_elements = (common)->common_ops->max_elements((common), element_type); \
		if ((common)->num_elements > 0) { \
			(common)->elements = devm_kzalloc((common)->dev, \
			(common)->num_elements * sizeof(typeof(*((common)->elements))), GFP_KERNEL); \
			if (!(common)->elements) { \
				dev_err(dev, "Failed to allocated memory for " # element_type); \
				allocate_ret = -ENOMEM; \
			} \
		} \
	} \
	allocate_ret; \
})

static struct max9x_pdata *pdata_ser(struct device *dev, struct max9x_subdev_pdata *sdinfo, const char *name,
					unsigned int phys_addr, unsigned int virt_addr,
					struct gpiod_lookup *ser_gpio)
{
	struct max9x_pdata *pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);

	if (!pdata)
		return NULL;

	dev_info(dev, "ser %s phys %02x virt %02x\n", name, phys_addr, virt_addr);

	sdinfo->board_info.platform_data = pdata;
	strscpy(sdinfo->board_info.type, name, I2C_NAME_SIZE);
	sdinfo->board_info.addr = virt_addr;
	sdinfo->phys_addr = pdata->phys_addr = phys_addr;
	sdinfo->gpio = ser_gpio;

	return pdata;
}

static struct max9x_pdata *pdata_sensor(struct device *dev, struct max9x_subdev_pdata *sdinfo, const char *name,
					unsigned int phys_addr, unsigned int virt_addr)
{
	dev_info(dev, "sen %s phys %02x virt %02x\n", name, phys_addr, virt_addr);

	strscpy(sdinfo->board_info.type, name, I2C_NAME_SIZE);
	sdinfo->board_info.addr = virt_addr;
	sdinfo->phys_addr = phys_addr;

	return NULL;
}

static struct max9x_pdata *parse_ser_pdata(struct device *dev, const char *ser_name, char *suffix,
						unsigned int ser_nlanes, unsigned int phys_addr,
						unsigned int virt_addr, struct max9x_subdev_pdata *ser_sdinfo,
						unsigned int sensor_dt, struct gpiod_lookup *ser_gpio)
{
	struct max9x_pdata *ser_pdata = pdata_ser(dev, ser_sdinfo, ser_name, phys_addr, virt_addr, ser_gpio);
	struct max9x_serial_link_pdata *ser_serial_link;
	struct max9x_video_pipe_pdata *ser_video_pipe;

	if (!ser_pdata)
		return NULL;

	snprintf(ser_pdata->suffix, sizeof(ser_pdata->suffix), "%s", suffix);

	ser_pdata->num_serial_links = 1;
	ser_pdata->serial_links = devm_kzalloc(dev, ser_pdata->num_serial_links * sizeof(*ser_pdata->serial_links),
					       GFP_KERNEL);
	if (!ser_pdata->serial_links)
		return NULL;

	ser_serial_link = &ser_pdata->serial_links[0];
	ser_serial_link->link_id = 0;
	ser_serial_link->link_type = MAX9X_LINK_TYPE_GMSL2;
	ser_serial_link->rx_freq_mhz = 6000;
	ser_serial_link->tx_freq_mhz = 187;

	ser_pdata->num_video_pipes = 1;
	ser_pdata->video_pipes = devm_kzalloc(dev,
				ser_pdata->num_video_pipes * sizeof(*ser_pdata->video_pipes), GFP_KERNEL);
	if (!ser_pdata->video_pipes)
		return NULL;

	ser_video_pipe = &ser_pdata->video_pipes[0];
	ser_video_pipe->serial_link_id = 0;
	ser_video_pipe->pipe_id = ser_sdinfo->serial_link_id;
	ser_video_pipe->src_csi_id = 1; /* PHY B typically */

	ser_video_pipe->num_data_types = 1;
	ser_video_pipe->data_types = devm_kzalloc(dev,
				ser_video_pipe->num_data_types * sizeof(*ser_video_pipe->data_types), GFP_KERNEL);
	if (!ser_video_pipe->data_types)
		return NULL;
	ser_video_pipe->data_types[0] = sensor_dt;

	ser_pdata->num_csi_links = 1;
	ser_pdata->csi_links = devm_kzalloc(dev, ser_pdata->num_csi_links * sizeof(*ser_pdata->csi_links), GFP_KERNEL);
	if (!ser_pdata->csi_links)
		return NULL;

	struct max9x_csi_link_pdata *csi_link = &ser_pdata->csi_links[0];

	csi_link->link_id = 1;
	csi_link->num_lanes = ser_nlanes;

	return ser_pdata;
}

static int parse_sensor_pdata(struct device *dev, const char *sensor_name, char *suffix, unsigned int ser_nlanes,
			       unsigned int phys_addr, unsigned int virt_addr, struct max9x_subdev_pdata *ser_sdinfo,
			       struct max9x_pdata *ser_pdata)
{
	ser_pdata->num_subdevs = 1;
	ser_pdata->subdevs = devm_kzalloc(dev, ser_pdata->num_subdevs * sizeof(*ser_pdata->subdevs), GFP_KERNEL);
	if (!ser_pdata->subdevs)
		return -ENOMEM;
	pdata_sensor(dev, &ser_pdata->subdevs[0], sensor_name, phys_addr, virt_addr);

	/* Copy GPIO configuration from serializer to sensor subdev */
	ser_pdata->subdevs[0].gpio = ser_sdinfo->gpio;

	/* NOTE: i2c_dev_set_name() will prepend "i2c-" to this name */
	char *dev_name = devm_kzalloc(dev, I2C_NAME_SIZE, GFP_KERNEL);

	if (!dev_name)
		return -ENOMEM;

	snprintf(dev_name, I2C_NAME_SIZE, "%s %s", sensor_name, suffix);
	ser_pdata->subdevs[0].board_info.dev_name = dev_name;

	struct sensor_platform_data *sen_pdata = devm_kzalloc(dev, sizeof(*sen_pdata), GFP_KERNEL);

	if (!sen_pdata)
		return -ENOMEM;

	ser_pdata->subdevs[0].board_info.platform_data = sen_pdata;
	sen_pdata->lanes = ser_nlanes;
	sen_pdata->irq_pin_flags = 1;	//workaround for identify D3.
	snprintf(sen_pdata->suffix, sizeof(sen_pdata->suffix), "%s", suffix);

	return 0;
}

static void *parse_serdes_pdata(struct device *dev)
{
	/*
	 * Assumptions:
	 *   - All ports have same model of sensor
	 *   - Single-port usage will always be port 0, never port 1
	 *   - Serializer always uses serial link 0
	 */
	struct serdes_platform_data *serdes_pdata = dev->platform_data;
	unsigned int num_ports = serdes_pdata->subdev_num;
	unsigned int csi_port = serdes_pdata->des_port / 90;
	struct max9x_pdata *des_pdata = devm_kzalloc(dev, sizeof(*des_pdata), GFP_KERNEL);

	if (!des_pdata)
		return NULL;

	snprintf(des_pdata->suffix, sizeof(des_pdata->suffix), "%c", serdes_pdata->suffix);
	des_pdata->num_serial_links = num_ports;
	des_pdata->serial_links = devm_kzalloc(dev,
				des_pdata->num_serial_links * sizeof(*des_pdata->serial_links), GFP_KERNEL);
	if (!des_pdata->serial_links)
		return NULL;

	des_pdata->num_subdevs = num_ports;
	des_pdata->subdevs = devm_kzalloc(dev, des_pdata->num_subdevs * sizeof(*des_pdata->subdevs), GFP_KERNEL);
	if (!des_pdata->subdevs)
		return NULL;

	des_pdata->num_video_pipes = num_ports;
	des_pdata->video_pipes = devm_kzalloc(dev,
				des_pdata->num_video_pipes * sizeof(*des_pdata->video_pipes), GFP_KERNEL);
	if (!des_pdata->video_pipes)
		return NULL;

	for (unsigned int serial_link_id = 0; serial_link_id < des_pdata->num_serial_links; serial_link_id++) {
		struct max9x_serial_link_pdata *serial_link = &des_pdata->serial_links[serial_link_id];
		unsigned int video_pipe_id = serial_link_id;
		struct serdes_subdev_info *serdes_sdinfo = &serdes_pdata->subdev_info[serial_link_id];
		struct max9x_video_pipe_pdata *des_video_pipe = &des_pdata->video_pipes[video_pipe_id];
		struct max9x_subdev_pdata *ser_sdinfo = &des_pdata->subdevs[serial_link_id];
		const char *ser_name = serdes_pdata->ser_name;
		const char *sensor_name = serdes_sdinfo->board_info.type;
		unsigned int ser_alias = serdes_sdinfo->ser_alias;
		unsigned int sensor_alias = serdes_sdinfo->board_info.addr;
		unsigned int ser_phys_addr = serdes_sdinfo->ser_phys_addr;
		unsigned int sensor_phys_addr = serdes_sdinfo->phy_i2c_addr;
		unsigned int lanes = serdes_pdata->ser_nlanes;
		unsigned int dt = serdes_sdinfo->sensor_dt;
		struct gpiod_lookup *ser_gpio = serdes_sdinfo->ser_gpio;

		serial_link->link_id = serial_link_id;
		serial_link->link_type = MAX9X_LINK_TYPE_GMSL2;
		serial_link->rx_freq_mhz = 6000;
		serial_link->tx_freq_mhz = 187;

		des_video_pipe->serial_link_id = serial_link_id;
		des_video_pipe->pipe_id = video_pipe_id;
		des_video_pipe->src_pipe_id = video_pipe_id;
		des_video_pipe->num_maps = 3;
		des_video_pipe->maps = devm_kzalloc(dev,
					des_video_pipe->num_maps * sizeof(*des_video_pipe->maps), GFP_KERNEL);
		if (!des_video_pipe->maps)
			return NULL;

		ser_sdinfo->serial_link_id = serial_link_id;

		SET_CSI_MAP(des_video_pipe->maps, 0, 0, 0x00, video_pipe_id, 0x00, csi_port);
		SET_CSI_MAP(des_video_pipe->maps, 1, 0, 0x01, video_pipe_id, 0x01, csi_port);
		SET_CSI_MAP(des_video_pipe->maps, 2, 0, dt, video_pipe_id, dt, csi_port); /* YUV422 8-bit */

		struct max9x_pdata *ser_pdata = parse_ser_pdata(dev, ser_name, serdes_sdinfo->suffix, lanes,
								ser_phys_addr, ser_alias, ser_sdinfo, dt, ser_gpio);

		if (!ser_pdata)
			return NULL;

		if (parse_sensor_pdata(dev, sensor_name, serdes_sdinfo->suffix, lanes, sensor_phys_addr, sensor_alias,
				   ser_sdinfo, ser_pdata))
			return NULL;

	}

	des_pdata->num_csi_links = 1;
	des_pdata->csi_links = devm_kzalloc(dev, des_pdata->num_csi_links * sizeof(*des_pdata->csi_links), GFP_KERNEL);
	if (!des_pdata->csi_links)
		return NULL;

	do {
		struct max9x_csi_link_pdata *csi_link = &des_pdata->csi_links[0];

		csi_link->link_id = csi_port;
		csi_link->bus_type = serdes_pdata->bus_type;
		csi_link->num_lanes = serdes_pdata->deser_nlanes;
		csi_link->tx_rate_mbps = 2000;
		csi_link->auto_initial_deskew = true;
		csi_link->initial_deskew_width = 7;
		csi_link->auto_start = false;
		csi_link->num_maps = serdes_pdata->deser_nlanes;
		csi_link->maps = devm_kzalloc(dev, csi_link->num_maps * sizeof(*csi_link->maps), GFP_KERNEL);
		if (!csi_link->maps)
			return NULL;
		if (csi_port == 1) {
			SET_PHY_MAP(csi_link->maps, 0, 0, 1, 0); /* 0 (DA0) -> PHY1.0 */
			SET_PHY_MAP(csi_link->maps, 1, 1, 1, 1); /* 1 (DA1) -> PHY1.1 */
			if (csi_link->num_maps == 4) {
				SET_PHY_MAP(csi_link->maps, 2, 2, 0, 0); /* 2 (DA2) -> PHY0.0 */
				SET_PHY_MAP(csi_link->maps, 3, 3, 0, 1); /* 3 (DA3) -> PHY0.1 */
			}
		} else if (csi_port == 2) {
			SET_PHY_MAP(csi_link->maps, 0, 0, 2, 0); /* 0 (DA0) -> PHY2.0 */
			SET_PHY_MAP(csi_link->maps, 1, 1, 2, 1); /* 1 (DA1) -> PHY2.1 */
			if (csi_link->num_maps == 4) {
				SET_PHY_MAP(csi_link->maps, 2, 2, 3, 0); /* 2 (DA2) -> PHY3.0 */
				SET_PHY_MAP(csi_link->maps, 3, 3, 3, 1); /* 3 (DA3) -> PHY3.1 */
			}
		}
	} while (0);

	return des_pdata;
}

static int max9x_enable_resume(struct max9x_common *common)
{
	struct device *dev = common->dev;
	int ret;

	if (common->regulator_enabled)
		return 0;

	if (!IS_ERR_OR_NULL(common->vdd_regulator)) {
		ret = regulator_enable(common->vdd_regulator);
		if (ret) {
			dev_err(dev, "Failed to enable %s", MAX9X_VDD_REGULATOR_NAME);
			return ret;
		}
		common->regulator_enabled = true;
	}

	if (common->common_ops && common->common_ops->enable) {
		ret = common->common_ops->enable(common);
		if (ret)
			goto err;
	}

	return 0;

err:
	if (!IS_ERR_OR_NULL(common->reset_gpio))
		gpiod_set_value_cansleep(common->reset_gpio, 1);

	if (common->regulator_enabled && !IS_ERR_OR_NULL(common->vdd_regulator))
		regulator_disable(common->vdd_regulator);

	common->regulator_enabled = false;

	return ret;
}

static int max9x_remap_serializers_resume(struct max9x_common *common, unsigned int link_id)
{
	int ret;
	struct max9x_serdes_serial_link *serial_link = &common->serial_link[link_id];
	unsigned int phys_addr, virt_addr;
	struct i2c_client *phys_client;
	struct regmap *phys_map, *virt_map;
	struct device *dev = &serial_link->remote.client->dev;
	unsigned int val;
	const struct regmap_config regmap_config = {
		.reg_bits = 16,
		.val_bits = 8,
	};

	if (!serial_link->remote.pdata)
		return 0;

	phys_addr = serial_link->remote.pdata->phys_addr;
	virt_addr = serial_link->remote.pdata->board_info.addr;
	if (phys_addr == virt_addr)
		return 0;

	dev_info(dev, "Remap serializer from 0x%02x to 0x%02x", phys_addr, virt_addr);

	phys_client = i2c_new_dummy_device(common->client->adapter, phys_addr);
	if (IS_ERR_OR_NULL(phys_client)) {
		dev_err(dev, "Failed to create dummy client for phys_addr 0x%x", phys_addr);
		ret = PTR_ERR(phys_client);
		return ret;
	}

	phys_map = regmap_init_i2c(phys_client, &regmap_config);
	if (IS_ERR_OR_NULL(phys_map)) {
		dev_err(dev, "Failed to create dummy map for phys_addr 0x%x", phys_addr);
		ret = PTR_ERR(phys_map);
		goto err_client;
	}

	struct max9x_common *ser_common = max9x_client_to_common(serial_link->remote.client);

	virt_map = ser_common->map;
	ret = regmap_read_retry(virt_map, MAX9X_DEV_ID, &val);
	if (!ret) {
		dev_info(dev, "Remap not necessary");
		ret = 0;
		goto err_regmap;
	}

	ret = regmap_read_retry(phys_map, MAX9X_DEV_ID, &val);
	if (ret) {
		dev_err(dev, "Device not present at 0x%02x", phys_addr);
		goto err_regmap;
	} else {
		dev_info(dev, "DEV_ID before: 0x%02x", val);
	}

	ret = regmap_write_retry(phys_map, 0x00, (virt_addr & 0x7f) << 1);
	if (ret) {
		dev_err(dev, "Failed to remap serialzier from 0x%02x to 0x%02x (%d)",
				phys_addr, virt_addr, ret);
		goto err_regmap;
	}

	usleep_range(1000, 1050);

	ret = regmap_read_retry(virt_map, MAX9X_DEV_ID, &val);
	if (ret) {
		dev_err(dev, "Device not present after remap to 0x%02x", virt_addr);
		goto err_regmap;
	} else {
		dev_info(dev, "DEV_ID after: 0x%02x", val);
	}

err_regmap:
	regmap_exit(phys_map);
err_client:
	i2c_unregister_device(phys_client);

	return ret;
}

static int max9x_create_adapters_resume(struct max9x_common *common)
{
	struct device *dev = common->dev;
	unsigned int link_id;
	const unsigned int RETRY_MS_MIN = 32;
	const unsigned int RETRY_MS_MAX = 512;
	unsigned int ms;
	int err = 0;

	for (link_id = 0; link_id < common->num_serial_links; link_id++) {
		dev_info(dev, "RESUME: Serial-link %d: %senabled", link_id,
			 (common->serial_link[link_id].enabled ? "" : "not "));

		if (!common->serial_link[link_id].enabled)
			continue;

		/* This exponential retry works around a current problem in the locking code. */
		for (ms = RETRY_MS_MIN; ms <= RETRY_MS_MAX; ms <<= 1) {
			err = max9x_enable_serial_link(common, link_id);
			if (!err)
				break;

			dev_warn(dev,
				"enable link %d failed, trying again (waiting %d ms)",
				link_id, ms);
			msleep(ms);
		}
		if (ms > RETRY_MS_MAX) {
			dev_err(dev, "failed to enable link %d after multiple retries",
				link_id);
			max9x_disable_serial_link(common, link_id);
			common->serial_link[link_id].enabled = false;
		}
	}

	for (link_id = 0; link_id < common->num_serial_links; link_id++)
		max9x_setup_translations(common);

	return 0;
}

int max9x_common_resume(struct max9x_common *common)
{
	struct max9x_common *des_common = NULL;
	struct device *dev = common->dev;
	u32 des_link;
	int ret = 0;

	if (dev->platform_data && common->type == MAX9X_SERIALIZER) {
		struct max9x_pdata *pdata = dev->platform_data;
		WARN_ON(pdata->num_serial_links < 1);

		des_common = max9x_client_to_common(
			pdata->serial_links[0].des_client);
		if (des_common) {
			des_link = pdata->serial_links[0].des_link_id;
			ret = max9x_des_isolate_serial_link(des_common, des_link);
			if (ret)
				goto err_deisolate;
			ret = max9x_remap_serializers_resume(des_common, des_link);
			if (ret)
				goto err_deisolate;
		} else {
			return ret;
		}
	}

	ret = max9x_verify_devid(common);
	if (ret) {
		dev_err(dev, "can't get devid after resume");
		goto err_reset_serializer;
	}

	ret = max9x_enable_resume(common);
	if (ret) {
		dev_err(dev, "Failed to enable");
		goto err_disable;
	}

	ret = max9x_create_adapters_resume(common);
	if (ret) {
		dev_err(dev, "Failed to create adapters");
		goto err_disable;
	}

	if (common->type == MAX9X_SERIALIZER && des_common)
		max9x_des_deisolate_serial_link(des_common, des_link);

	return 0;

err_disable:
	max9x_disable(common);

err_reset_serializer:
	if (common->type == MAX9X_SERIALIZER) {
		if (common->common_ops && common->common_ops->remap_reset) {
			ret = common->common_ops->remap_reset(common);
			if (ret)
				return ret;
		}
	}

err_deisolate:
	if (common->type == MAX9X_SERIALIZER && des_common)
		max9x_des_deisolate_serial_link(des_common, des_link);

	return ret;
}

int max9x_common_suspend(struct max9x_common *common)
{
	unsigned int link_id;

	dev_dbg(common->dev, "try to suspend");

	for (link_id = 0; link_id < common->num_serial_links; link_id++)
		max9x_disable_serial_link(common, link_id);

	max9x_disable(common);

	return 0;
}

int max9x_common_init_i2c_client(struct max9x_common *common,
	struct i2c_client *client,
	const struct regmap_config *regmap_config,
	struct max9x_common_ops *common_ops,
	struct max9x_serial_link_ops *serial_link_ops,
	struct max9x_csi_link_ops *csi_link_ops,
	struct max9x_line_fault_ops *lf_ops)
{
	struct device *dev = &client->dev;
	struct i2c_adapter *adap = to_i2c_adapter(dev->parent);
	struct max9x_pdata *pdata = NULL;
	u32 phys_addr, virt_addr;
	int ret;

	common->dev = dev;
	common->client = client;

	/* If no GPIO is found this will return NULL, and will not error */
	common->reset_gpio = devm_gpiod_get_optional(dev, MAX9X_RESET_GPIO_NAME, GPIOD_OUT_HIGH);
	if (IS_ERR(common->reset_gpio)) {
		dev_err(dev, "gpiod_get failed with error: %ld", PTR_ERR(common->reset_gpio));
		return PTR_ERR(common->reset_gpio);
	}

	common->vdd_regulator = devm_regulator_get_optional(dev, MAX9X_VDD_REGULATOR_NAME);
	if (IS_ERR_OR_NULL(common->vdd_regulator))
		dev_info(dev, "Missing VDD regulator");

	common->map = devm_regmap_init_i2c(client, regmap_config);
	if (IS_ERR_OR_NULL(common->map)) {
		dev_err(dev, "Failed to create regmap.");
		return PTR_ERR(common->map);
	}

	if (dev->platform_data) {
		pdata = dev->platform_data;
		/*
		 * for serializer, the i2c phys_addr supposed to be updated as virt_addr
		 * when deserializer do max9x_remap_serializer,
		 * check here to avoid failure when directly read chipid from virt_addr
		 */
		virt_addr = common->client->addr;
		phys_addr = pdata->phys_addr ? pdata->phys_addr : virt_addr;

		if (phys_addr != virt_addr) {
			common->phys_client = i2c_new_dummy_device(common->client->adapter, phys_addr);
			if (IS_ERR_OR_NULL(common->phys_client)) {
				dev_err(dev, "Failed to create dummy device for phys_addr");
				ret = PTR_ERR(common->phys_client);
				return ret;
			}

			common->phys_map = regmap_init_i2c(common->phys_client, regmap_config);
			if (IS_ERR_OR_NULL(common->phys_map)) {
				dev_err(dev, "Failed to create dummy device regmap for phys_addr");
				ret = PTR_ERR(common->phys_client);
				goto err_phys_client;
			}
		}
	}

	ret = max9x_verify_devid(common);
	if (ret)
		goto err_phys_map;

	ret = common->des->get_max9x_ops(common->des->chip_type, &(common->common_ops),
		&(common->serial_link_ops), &(common->csi_link_ops), &(common->line_fault_ops),
		&(common->translation_ops));
	if (ret) {
		dev_err(dev, "Failed to get ops.");
		goto err_phys_map;
	}

	mutex_init(&common->link_mutex);
	mutex_init(&common->isolate_mutex);
	common->isolated_link = -1;
	common->selected_link = -1;

	ret = MAX9X_ALLOCATE_ELEMENTS(common, MAX9X_CSI_LINK, csi_link, num_csi_links);
	if (ret)
		goto err_phys_map;

	ret = MAX9X_ALLOCATE_ELEMENTS(common, MAX9X_VIDEO_PIPE, video_pipe, num_video_pipes);
	if (ret)
		goto err_phys_map;

	ret = MAX9X_ALLOCATE_ELEMENTS(common, MAX9X_SERIAL_LINK, serial_link, num_serial_links);
	if (ret)
		goto err_phys_map;

	ret = MAX9X_ALLOCATE_ELEMENTS(common, MAX9X_LINE_FAULT, line_fault, num_line_faults);
	if (ret)
		goto err_phys_map;

	common->muxc = i2c_mux_alloc(
		adap, dev,
		common->num_serial_links,
		0,
		I2C_MUX_LOCKED,
		max9x_select_i2c_chan,
		max9x_deselect_i2c_chan);
	if (IS_ERR_OR_NULL(common->muxc)) {
		dev_err(dev, "Failed to allocate mux core");
		ret = -ENOMEM;
		goto err_phys_map;
	}
	common->muxc->priv = common;

	if (common->type == MAX9X_DESERIALIZER)
		dev->platform_data = parse_serdes_pdata(dev);

	if (dev->platform_data) {
		pdata = dev->platform_data;

		dev_dbg(dev, "Parse pdata");

		ret = max9x_parse_pdata(common, pdata);
		if (ret)
			goto err_adapters;
	}

	dev_dbg(dev, "Enable");

	ret = max9x_enable(common);
	if (ret) {
		dev_err(dev, "Failed to enable");
		goto err_enable;
	}

	dev_dbg(dev, "Enable gpio");
	ret = max9x_setup_gpio(common);
	if (ret)
		goto err_enable;

	dev_dbg(dev, "Enable line faults");

	ret = max9x_enable_line_faults(common);
	if (ret) {
		dev_err(dev, "Failed to enable line faults");
		goto err_enable;
	}

	dev_dbg(dev, "Enable links");

	ret = max9x_create_adapters(common);
	if (ret)
		goto err_enable;

	ret = max9x_register_v4l_subdev(common);
	if (ret)
		goto err_enable;

	dev_dbg(dev, "Probe successfully.");

	goto err_phys_map;

err_enable:
	max9x_disable(common);

err_adapters:
	i2c_mux_del_adapters(common->muxc);

err_phys_map:
	if (common->phys_map) {
		regmap_exit(common->phys_map);
		common->phys_map = NULL;
	}

err_phys_client:
	if (common->phys_client) {
		i2c_unregister_device(common->phys_client);
		common->phys_client = NULL;
	}

	return ret;
}
EXPORT_SYMBOL(max9x_common_init_i2c_client);

void max9x_destroy(struct max9x_common *common)
{
	unsigned int link_id;
	unsigned int line;

	dev_dbg(common->dev, "Destroy");

	max9x_disable_translations(common);

	for (link_id = 0; link_id < common->num_serial_links; link_id++) {
		max9x_disable_serial_link(common, link_id);
		max9x_sysfs_destroy_get_link(common, link_id);
	}

	for (line = 0; line < common->num_line_faults; line++)
		max9x_sysfs_destroy_line_fault_status(common, line);

	max9x_disable(common);

	/* unregister devices? */

	v4l2_async_unregister_subdev(&common->v4l.sd);
	v4l2_subdev_cleanup(&common->v4l.sd);
	media_entity_cleanup(&common->v4l.sd.entity);

	i2c_mux_del_adapters(common->muxc);
	mutex_destroy(&common->link_mutex);
	mutex_destroy(&common->isolate_mutex);
	for (int i = 0; i < common->num_csi_links; i++) {
		mutex_destroy(&common->csi_link[i].csi_mutex);
	}
}
EXPORT_SYMBOL(max9x_destroy);

/*
 * max9x_serdes_mhz_to_rate()- Lookup register value for given frequency
 *
 * Return: Positive value on success, negative error on failure
 */
int max9x_serdes_mhz_to_rate(struct max9x_serdes_rate_table *table,
							int len, unsigned int freq_mhz)
{
	int i;

	for (i = 0; i < len; i++)
		if (table[i].freq_mhz == freq_mhz)
			return table[i].val;

	WARN_ONCE(true, "Invalid GMSL frequency: %d MHz", freq_mhz);

	return -EINVAL;
}
EXPORT_SYMBOL(max9x_serdes_mhz_to_rate);

struct max9x_common *max9x_sd_to_common(struct v4l2_subdev *sd)
{
	if (!sd)
		return NULL;

	struct max9x_serdes_v4l *v4l = container_of(sd, struct max9x_serdes_v4l, sd);

	return container_of(v4l, struct max9x_common, v4l);
}
EXPORT_SYMBOL(max9x_sd_to_common);

struct max9x_common *max9x_client_to_common(struct i2c_client *client)
{
	if (!client)
		return NULL;

	return max9x_sd_to_common(i2c_get_clientdata(client));
}
EXPORT_SYMBOL(max9x_client_to_common);

int max9x_soft_reset(struct max9x_common *common)
{
	if (common->common_ops && common->common_ops->soft_reset)
		return common->common_ops->soft_reset(common);

	return 0;
}

int max9x_remap_addr(struct max9x_common *common)
{
	if (common->common_ops && common->common_ops->remap_addr)
		return common->common_ops->remap_addr(common);

	return 0;
}

int max9x_setup_gpio(struct max9x_common *common)
{
	if (common->common_ops && common->common_ops->remap_addr)
		return common->common_ops->setup_gpio(common);

	return 0;
}

/*
 * max9x_enable() - Enable reset and power to des.
 */
int max9x_enable(struct max9x_common *common)
{
	struct device *dev = common->dev;
	unsigned int phys_addr, virt_addr;
	int ret;

	if (common->regulator_enabled)
		return 0;

	virt_addr = common->client->addr;
	phys_addr = (common->phys_client ? common->phys_client->addr : virt_addr);

	if (!IS_ERR_OR_NULL(common->vdd_regulator)) {
		ret = regulator_enable(common->vdd_regulator);
		if (ret) {
			dev_err(dev, "Failed to enable %s", MAX9X_VDD_REGULATOR_NAME);
			return ret;
		}
		common->regulator_enabled = true;
	}

	/* Ensure device is reset */
	if (!IS_ERR_OR_NULL(common->reset_gpio)) {
		gpiod_set_value_cansleep(common->reset_gpio, 1);
		usleep_range(1000, 1050);
		gpiod_set_value_cansleep(common->reset_gpio, 0);
	} else {
		/* No reset_gpio, device requires soft-reset */
		if (phys_addr == virt_addr) {
			/* No remapping necessary */
			ret = max9x_soft_reset(common);
			if (ret)
				goto err;
		} else if (phys_addr != virt_addr) {
			/* Try virt address first */
			ret = max9x_soft_reset(common);
			if (ret) {
				/* Nope? Try phys address next */
				struct regmap *virt_map = common->map;

				common->map = common->phys_map;
				ret = max9x_soft_reset(common);
				common->map = virt_map;
				if (ret)
					goto err;
			}
		}
	}

	dev_info(dev, "sleep for startup after soft reset");
	usleep_range(100000, 100050);

	if (phys_addr != virt_addr) {
		/* Device is now reset, but requires remap */
		ret = max9x_remap_addr(common);
		if (ret)
			goto err;
		msleep(20);
	}

	if (common->common_ops && common->common_ops->enable) {
		ret = common->common_ops->enable(common);
		if (ret)
			goto err;
	}

	return 0;

err:
	if (!IS_ERR_OR_NULL(common->reset_gpio))
		gpiod_set_value_cansleep(common->reset_gpio, 1);

	if (common->regulator_enabled && !IS_ERR_OR_NULL(common->vdd_regulator))
		regulator_disable(common->vdd_regulator);

	common->regulator_enabled = false;

	return ret;
}

/*
 * max9x_disable() - Disable reset and power to des.
 */
int max9x_disable(struct max9x_common *common)
{
	struct device *dev = common->dev;
	int ret;

	ret = max9x_disable_line_faults(common);
	if (ret) {
		dev_err(dev, "Failed to disable line faults");
		return ret;
	}

	if (common->regulator_enabled) {
		if (common->common_ops && common->common_ops->disable)
			return common->common_ops->disable(common);

		common->regulator_enabled = false;

		if (!IS_ERR_OR_NULL(common->reset_gpio))
			gpiod_set_value_cansleep(common->reset_gpio, 1);

		if (!IS_ERR_OR_NULL(common->vdd_regulator)) {
			ret = regulator_disable(common->vdd_regulator);
			if (ret) {
				dev_err(dev, "Failed to disable %s", MAX9X_VDD_REGULATOR_NAME);
				return ret;
			}
		}
	}
	return 0;
}

static int max9x_find_chip_index(unsigned int dev_id)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(max9x_chips); i++) {
		if (dev_id == max9x_chips[i].dev_id)
			return i;
	}

	return -ENODEV;
}

int max9x_verify_devid(struct max9x_common *common)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	struct regmap *phys_map = common->phys_map;
	unsigned int dev_id, dev_rev;
	int chip_type;
	int ret;

	/*
	 * Fetch and output chip name + revision
	 * try both virtual address and physical address
	 */
	ret = regmap_read_retry(map, MAX9X_DEV_ID, &dev_id);
	if (ret) {
		dev_warn(dev, "Failed to read chip ID from virtual address");
		if (phys_map) {
			ret = regmap_read(phys_map, MAX9X_DEV_ID, &dev_id);
			if (ret) {
				dev_err(dev, "Failed to read chip ID from phys address");
				return ret;
			}
		} else
			return ret;
	}

	chip_type = max9x_find_chip_index(dev_id);
	if (chip_type < 0) {
		dev_warn(dev, "Unknown chip ID 0x%x", dev_id);
		return -ENODEV;
	}
	common->des = &max9x_chips[chip_type];
	common->type = common->des->serdes_type;
	TRY(ret, regmap_read_retry(map, common->des->rev_reg, &dev_rev));
	dev_rev = FIELD_GET(MAX9X_DEV_REV_FIELD, dev_rev);

	dev_info(dev, "Detected MAX9x chip ID  0x%x revision 0x%x", dev_id, dev_rev);
	return 0;
}

/* TODO: remap not hardcode according to pdata */
int max9x_remap_serializers(struct max9x_common *common, unsigned int link_id)
{
	int ret;
	struct max9x_serdes_serial_link *serial_link = &common->serial_link[link_id];
	unsigned int phys_addr, virt_addr;
	struct i2c_client *phys_client, *virt_client;
	struct regmap *phys_map, *virt_map;
	unsigned int val;
	const struct regmap_config regmap_config = {
		.reg_bits = 16,
		.val_bits = 8,
	};

	if (!serial_link->remote.pdata)
		return 0;

	ret = max9x_select_i2c_chan(common->muxc, link_id);
	if (ret)
		return ret;

	ret = max9x_des_isolate_serial_link(common, link_id);
	if (ret)
		goto err_deisolate;

	phys_addr = serial_link->remote.pdata->phys_addr;
	virt_addr = serial_link->remote.pdata->board_info.addr;
	if (phys_addr == virt_addr)
		return 0;

	dev_info(common->dev, "Remap serializer from 0x%02x to 0x%02x", phys_addr, virt_addr);

	phys_client = i2c_new_dummy_device(common->client->adapter, phys_addr);
	if (IS_ERR_OR_NULL(phys_client)) {
		dev_err(common->dev, "Failed to create dummy client for phys_addr");
		return PTR_ERR(phys_client);
	}

	phys_map = regmap_init_i2c(phys_client, &regmap_config);
	if (IS_ERR_OR_NULL(phys_map))
		goto err_client;

	virt_client = i2c_new_dummy_device(common->client->adapter, virt_addr);
	if (IS_ERR_OR_NULL(virt_client))
		goto err_regmap;

	virt_map = regmap_init_i2c(virt_client, &regmap_config);
	if (IS_ERR_OR_NULL(virt_map))
		goto err_virt_client;

	ret = regmap_read_retry(virt_map, MAX9X_DEV_ID, &val);
	if (!ret) {
		dev_info(common->dev, "Remap not necessary");
		ret = 0;
		goto err_virt_regmap;
	}

	ret = regmap_read_retry(phys_map, MAX9X_DEV_ID, &val);
	if (ret) {
		dev_err(common->dev, "Device not present at 0x%02x", phys_addr);
		goto err_virt_regmap;
	} else {
		dev_info(common->dev, "DEV_ID before: 0x%02x", val);
	}

	ret = regmap_write_retry(phys_map, 0x00, (virt_addr & 0x7f) << 1);
	if (ret) {
		dev_err(common->dev, "Failed to remap serialzier from 0x%02x to 0x%02x (%d)",
				phys_addr, virt_addr, ret);
		goto err_virt_regmap;
	}

	usleep_range(1000, 1050);

	ret = regmap_read_retry(virt_map, MAX9X_DEV_ID, &val);
	if (ret) {
		dev_err(common->dev, "Device not present after remap to 0x%02x", virt_addr);
		goto err_virt_regmap;
	} else {
		dev_info(common->dev, "DEV_ID after: 0x%02x", val);
	}

err_virt_regmap:
	regmap_exit(virt_map);
err_virt_client:
	i2c_unregister_device(virt_client);

err_regmap:
	regmap_exit(phys_map);
err_client:
	i2c_unregister_device(phys_client);

err_deisolate:
	max9x_des_deisolate_serial_link(common, link_id);

	max9x_deselect_i2c_chan(common->muxc, link_id);

	return ret;
}

int max9x_create_adapters(struct max9x_common *common)
{
	struct device *dev = common->dev;
	unsigned int link_id;
	const unsigned int RETRY_MS_MIN = 32;
	const unsigned int RETRY_MS_MAX = 512;
	unsigned int ms;
	int err = 0;

	for (link_id = 0; link_id < common->num_serial_links; link_id++) {
		err = max9x_sysfs_create_get_link(common, link_id);
		if (err) {
			dev_err(dev, "failed to create sysfs lock status file for link %d",
				link_id);
			continue;
		}

		dev_info(dev, "Serial-link %d: %senabled",
			 link_id, (common->serial_link[link_id].enabled ? "" : "not "));

		if (!common->serial_link[link_id].enabled)
			continue;

		/* This exponential retry works around a current problem in the locking code. */
		for (ms = RETRY_MS_MIN; ms <= RETRY_MS_MAX; ms <<= 1) {
			err = max9x_enable_serial_link(common, link_id);
			if (!err)
				break;

			dev_warn(dev,
				"enable link %d failed, trying again (waiting %d ms)",
				link_id, ms);
			msleep(ms);
		}
		if (ms > RETRY_MS_MAX) {
			dev_err(dev, "failed to enable link %d after multiple retries",
				link_id);
			goto err_disable;
		}

		if (common->type == MAX9X_DESERIALIZER) {
			err = max9x_remap_serializers(common, link_id);
			if (err) {
				dev_err(dev, "failed to remap serializers on link %d", link_id);
				goto err_disable;
			}
		}

		continue;
err_disable:
		max9x_disable_serial_link(common, link_id);
		common->serial_link[link_id].enabled = false;
	}

	for (link_id = 0; link_id < common->num_serial_links; link_id++) {
		max9x_setup_translations(common);

		err = i2c_mux_add_adapter(common->muxc, 0, link_id);
		if (err) {
			dev_err(dev, "failed to add adapter for link %d",
				link_id);
			max9x_disable_serial_link(common, link_id);
			continue;
		}
	}

	return 0;
}

static void max9x_des_s_csi_link(struct max9x_common *common,
				 unsigned int serial_link_id, int enable)
{
	unsigned int video_pipe_id;
	int err = 0;

	for (video_pipe_id = 0; video_pipe_id < common->num_video_pipes;
	     video_pipe_id++) {
		struct max9x_serdes_video_pipe *video_pipe =
			&common->video_pipe[video_pipe_id];
		unsigned int map_id;

		if (!video_pipe->enabled)
			continue;

		if (video_pipe->config.src_link != serial_link_id)
			continue;

		for (map_id = 0; map_id < video_pipe->config.num_maps;
		     map_id++) {
			unsigned int csi_link_id =
				video_pipe->config.map[map_id].dst_csi;

			if (csi_link_id >= common->num_csi_links) {
				dev_warn(common->dev,
					 "Invalid dst_csi %d for video pipe %d map %d",
					 csi_link_id, video_pipe_id, map_id);
				continue;
			}

			if (common->csi_link[csi_link_id].config.auto_start)
				continue; /* Already started at probe */

			if (enable && !video_pipe->config.map[map_id].is_csi_enabled) {
				if (common->csi_link_ops->enable) {
					err = common->csi_link_ops->enable(
						common, csi_link_id);
					if (err)
						dev_warn(
							common->dev,
							"csi_link_ops->enable CSI %d failed: %d",
							csi_link_id, err);
					video_pipe->config.map[map_id].is_csi_enabled = true;
				}
			} else if (!enable && video_pipe->config.map[map_id].is_csi_enabled) {
				if (common->csi_link_ops->disable) {
					err = common->csi_link_ops->disable(
						common, csi_link_id);
					if (err)
						dev_warn(
							common->dev,
							"csi_link_ops->disable CSI %d failed: %d",
							csi_link_id, err);
					video_pipe->config.map[map_id].is_csi_enabled = false;
				}
			}
		}
	}
}

static int _max9x_s_remote_stream(struct max9x_common *common, u32 sink_pad,
				  u32 sink_stream, int enable)
{
	struct media_pad *remote_pad;
	struct v4l2_subdev *remote_sd;
	int ret = 0;

	if (sink_pad >= common->v4l.num_pads)
		return -EINVAL;

	remote_pad = media_pad_remote_pad_first(&common->v4l.pads[sink_pad]);
	if (IS_ERR_OR_NULL(remote_pad)) {
		dev_err(common->dev, "Failed to find remote pad for %s %u",
			common->v4l.sd.entity.name, sink_pad);
		return IS_ERR(remote_pad) ? PTR_ERR(remote_pad) : -ENODEV;
	}
	remote_sd = media_entity_to_v4l2_subdev(remote_pad->entity);
	if (!remote_sd) {
		dev_err(common->dev, "Failed to resolve entity %s to subdev",
			remote_pad->entity->name);
		return -ENODEV;
	}

	if (common->type == MAX9X_DESERIALIZER) {
		ret = enable ? v4l2_subdev_enable_streams(remote_sd,
							  remote_pad->index,
							  BIT(sink_stream)) :
			       v4l2_subdev_disable_streams(remote_sd,
							   remote_pad->index,
							   BIT(sink_stream));

	} else {
		ret = v4l2_subdev_call(remote_sd, video, s_stream, enable);
	}

	if (ret) {
		dev_err(common->dev, "Failed to %s stream %s %u:%u",
			enable ? "enable" : "disable", remote_sd->entity.name,
			remote_pad->index, sink_stream);
		return ret;
	}

	return ret;
}

static int _max9x_des_set_stream(struct max9x_common *common, u32 sink_pad,
				 u32 sink_stream, int enable)
{
	int rxport = 0;
	int ret = 0;

	if (sink_pad >= common->v4l.num_pads)
		return -EINVAL;

	rxport = sink_pad - common->num_csi_links;
	if (rxport < 0 || rxport >= common->num_serial_links) {
		dev_err(common->dev, "Failed to get rxport for pad: %u!",
			sink_pad);
		return -EINVAL;
	}
	if (enable)
		max9x_des_s_csi_link(common, rxport, enable);

	ret = _max9x_s_remote_stream(common, sink_pad, sink_stream, enable);
	if (ret) {
		dev_err(common->dev,
			"Failed to %s remote stream for sink %s %u:%u",
			enable ? "enable" : "disable",
			common->v4l.sd.entity.name, sink_pad, sink_stream);
		return ret;
	}
	if (!enable)
		max9x_des_s_csi_link(common, rxport, enable);

	return 0;
}

static int _max9x_ser_set_stream(struct max9x_common *common, u32 sink_pad,
				 u32 sink_stream, int enable)
{
	return _max9x_s_remote_stream(common, sink_pad, sink_stream, enable);
}

static int max9x_streams_mask_to_stream(u64 streams_mask, u32 *stream)
{
	int ret = -EINVAL;

	for (int i = 0; i < 64; i++) {
		if (streams_mask & BIT(i)) {
			*stream = i;
			ret = 0;
			break;
		}
	}
	return ret;
}

static int max9x_get_corresponding_pad_and_stream(struct v4l2_subdev_state *state, u32 pad,
				       u32 stream, u32 *another_pad,
				       u32 *another_stream)
{
	struct v4l2_subdev_route *route;

	for_each_active_route(&state->routing, route) {
		if (route->sink_pad == pad && route->sink_stream == stream) {
			*another_pad = route->source_pad;
			*another_stream = route->source_stream;
			return 0;
		} else if (route->source_pad == pad &&
			   route->source_stream == stream) {
			*another_pad = route->sink_pad;
			*another_stream = route->sink_stream;
			return 0;
		}
	}

	return -EINVAL;
}

static int _max9x_set_stream(struct v4l2_subdev *subdev,
			     struct v4l2_subdev_state *state, u32 pad,
			     u64 streams_mask, int enable)
{
	struct max9x_common *common = max9x_sd_to_common(subdev);
	u32 sink_pad;
	u32 sink_stream;
	u32 source_stream;
	int ret = 0;

	ret = max9x_streams_mask_to_stream(streams_mask, &source_stream);
	if (ret) {
		dev_err(common->dev, "Failed to get source stream!");
		return ret;
	}

	ret = max9x_get_corresponding_pad_and_stream(state, pad, source_stream,
						     &sink_pad, &sink_stream);
	if (ret) {
		dev_err(common->dev,
			"Failed to get sink pad and sink stream for %s %u:%u",
			common->v4l.sd.entity.name, pad, source_stream);
		return -EINVAL;
	}

	if (common->type == MAX9X_DESERIALIZER)
		ret = _max9x_des_set_stream(common, sink_pad, sink_stream,
					    enable);
	else
		ret = _max9x_ser_set_stream(common, sink_pad, sink_stream,
					    enable);

	return ret;
}

static int max9x_enable_streams(struct v4l2_subdev *subdev,
				 struct v4l2_subdev_state *state,
				 u32 pad, u64 streams_mask)
{
	return _max9x_set_stream(subdev, state, pad, streams_mask, true);
}

static int max9x_disable_streams(struct v4l2_subdev *subdev,
				 struct v4l2_subdev_state *state,
				 u32 pad, u64 streams_mask)
{
	return _max9x_set_stream(subdev, state, pad, streams_mask, false);
}

static struct v4l2_mbus_framefmt *__max9x_get_ffmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *v4l2_state,
			    struct v4l2_subdev_format *fmt)
{
	struct max9x_common *common = max9x_sd_to_common(sd);

	if (IS_ERR_OR_NULL(fmt)) {
		dev_err(common->dev, "Invalid fmt %p", fmt);
		return ERR_PTR(-EINVAL);
	}

	if (fmt->pad >= common->v4l.num_pads) {
		dev_err(sd->dev, "%s invalid pad %d", __func__, fmt->pad);
		return ERR_PTR(-EINVAL);
	}

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_state_get_format(v4l2_state, fmt->pad, fmt->stream);

	return &common->v4l.ffmts[fmt->pad];
}

static int max9x_get_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *v4l2_state,
			    struct v4l2_subdev_format *fmt)
{
	struct max9x_common *common = max9x_sd_to_common(sd);
	struct max9x_serdes_v4l *v4l = &common->v4l;
	struct v4l2_mbus_framefmt *ffmt;

	mutex_lock(&v4l->lock);

	ffmt = __max9x_get_ffmt(sd, v4l2_state, fmt);
	if (IS_ERR_OR_NULL(ffmt)) {
		mutex_unlock(&v4l->lock);
		return -EINVAL;
	}

	fmt->format = *ffmt;
	mutex_unlock(&v4l->lock);

	dev_info(sd->dev, "framefmt: width: %d, height: %d, code: 0x%x.",
		fmt->format.width, fmt->format.height, fmt->format.code);

	return 0;
}

static int max9x_set_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *v4l2_state,
			    struct v4l2_subdev_format *fmt)
{
	struct max9x_common *common = max9x_sd_to_common(sd);
	struct max9x_serdes_v4l *v4l = &common->v4l;
	struct v4l2_mbus_framefmt *ffmt;

	mutex_lock(&v4l->lock);

	ffmt = __max9x_get_ffmt(sd, v4l2_state, fmt);
	if (IS_ERR_OR_NULL(ffmt)) {
		mutex_unlock(&v4l->lock);
		return -EINVAL;
	}

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		ffmt->width = fmt->format.width;
		ffmt->height = fmt->format.height;
		ffmt->code = fmt->format.code;
	}
	fmt->format = *ffmt;

	mutex_unlock(&v4l->lock);

	return 0;
}

static int max9x_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				struct v4l2_mbus_frame_desc *desc)
{
	struct max9x_common *common = max9x_sd_to_common(sd);
	int ret = -EINVAL;

	if (((common->type != MAX9X_DESERIALIZER) &&
	     (common->type != MAX9X_SERIALIZER)) ||
	    pad >= common->v4l.num_pads)
		return -EINVAL;

	desc->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;

	struct v4l2_subdev_state *state =
		v4l2_subdev_lock_and_get_active_state(sd);
	struct v4l2_subdev_route *route;

	for_each_active_route(&state->routing, route) {
		if (route->source_pad != pad)
			continue;
		if (unlikely(route->sink_pad >= common->v4l.num_pads)) {
			ret = -EINVAL;
			dev_err(common->dev, "Found invalid route sink_pad!");
			goto out_unlock;
		}

		struct media_pad *remote_pad = media_pad_remote_pad_first(
			&common->v4l.pads[route->sink_pad]);
		if (!remote_pad)
			continue;
		struct v4l2_mbus_frame_desc source_desc = { 0 };

		ret = v4l2_subdev_call(
			media_entity_to_v4l2_subdev(remote_pad->entity), pad,
			get_frame_desc, remote_pad->index, &source_desc);
		if (ret) {
			dev_err(common->dev,
				"Failed to get sink pad %u remote frame desc!",
				route->sink_pad);
			goto out_unlock;
		}

		struct v4l2_mbus_frame_desc_entry *source_desc_entry = NULL;

		for (int i = 0; i < source_desc.num_entries; i++) {
			if (source_desc.entry[i].stream == route->sink_stream) {
				source_desc_entry = &source_desc.entry[i];
				break;
			}
		}
		if (!source_desc_entry) {
			ret = -EPIPE;
			dev_err(common->dev,
				"Failed to get source desc entry for pad: %u, stream: %u",
				route->sink_pad, route->sink_stream);
			goto out_unlock;
		}
		desc->entry[desc->num_entries].flags = source_desc_entry->flags;
		desc->entry[desc->num_entries].stream = route->source_stream;
		desc->entry[desc->num_entries].pixelcode =
			source_desc_entry->pixelcode;
		desc->entry[desc->num_entries].length =
			source_desc_entry->length;
		if (common->type == MAX9X_DESERIALIZER)
			desc->entry[desc->num_entries].bus.csi2.vc =
				route->sink_pad - common->num_csi_links;
		desc->entry[desc->num_entries].bus.csi2.dt =
			source_desc_entry->bus.csi2.dt;
		desc->num_entries++;
	}

out_unlock:
	v4l2_subdev_unlock_state(state);
	return ret;
}

static int max9x_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *v4l2_state,
				struct v4l2_subdev_mbus_code_enum *code)
{
	if (mbus_code_to_csi_dt(code->code) < 0)
		return -EINVAL;
	return 0;
}

static int _max9x_set_routing(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_krouting *routing)
{
	static const struct v4l2_mbus_framefmt format = {
		.width = 1920,
		.height = 1536,
		.code = MEDIA_BUS_FMT_UYVY8_1X16,
	};
	int ret;

	/*
	 * Note: we can only support up to V4L2_FRAME_DESC_ENTRY_MAX, until
	 * frame desc is made dynamically allocated.
	 */

	if (routing->num_routes > V4L2_FRAME_DESC_ENTRY_MAX)
		return -E2BIG;

	ret = v4l2_subdev_routing_validate(sd, routing,
					   V4L2_SUBDEV_ROUTING_ONLY_1_TO_1 |
					   V4L2_SUBDEV_ROUTING_NO_SINK_STREAM_MIX);
	if (ret)
		return ret;

	ret = v4l2_subdev_set_routing_with_fmt(sd, state, routing, &format);
	if (ret)
		return ret;
	return 0;
}

static int max9x_set_routing(struct v4l2_subdev *sd,
	struct v4l2_subdev_state *state,
	enum v4l2_subdev_format_whence which,
	struct v4l2_subdev_krouting *routing)
{
	_max9x_set_routing(sd, state, routing);

	return 0;
}

static int max9x_init_state(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *state)
{
	struct max9x_common *common = max9x_sd_to_common(sd);

	struct v4l2_subdev_route des_routes[] = {
		{
			.sink_pad = 5,
			.sink_stream = 0,
			.source_pad = 0,
			.source_stream = 0,
			.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		},
	};
	struct v4l2_subdev_route ser_routes[] = {
		{
			.sink_pad = 0,
			.sink_stream = 0,
			.source_pad = 2,
			.source_stream = 0,
			.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		},
	};
	struct v4l2_subdev_krouting des_routing = {
		.num_routes = ARRAY_SIZE(des_routes),
		.routes = des_routes,
	};
	struct v4l2_subdev_krouting ser_routing = {
		.num_routes = ARRAY_SIZE(ser_routes),
		.routes = ser_routes,
	};

	if (common->type == MAX9X_DESERIALIZER)
		return _max9x_set_routing(sd, state, &des_routing);
	else
		return _max9x_set_routing(sd, state, &ser_routing);
}

static const struct v4l2_subdev_pad_ops max9x_sd_pad_ops = {
	.get_fmt = max9x_get_fmt,
	.set_fmt = max9x_set_fmt,
	.get_frame_desc = max9x_get_frame_desc,
	.enum_mbus_code = max9x_enum_mbus_code,
	.set_routing = max9x_set_routing,
	.enable_streams = max9x_enable_streams,
	.disable_streams = max9x_disable_streams,
};

static struct v4l2_subdev_ops max9x_sd_ops = {
	.pad = &max9x_sd_pad_ops,
};

static int max9x_registered(struct v4l2_subdev *sd)
{
	struct max9x_common *common = max9x_sd_to_common(sd);
	struct device *dev = common->dev;
	int ret;

	for (unsigned int link_id = 0; link_id < common->num_serial_links; link_id++) {
		if (!common->serial_link[link_id].enabled) {
			dev_dbg(dev, "Serial-link %d not enabled, skipping subdevs", link_id);
			continue;
		}

		if (common->type == MAX9X_DESERIALIZER) {
			struct max9x_subdev_pdata *subdev_pdata =
				common->serial_link[link_id].remote.pdata;

			if (subdev_pdata) {
				struct max9x_pdata *ser_pdata =
					subdev_pdata->board_info.platform_data;
				struct v4l2_subdev *subdev = NULL;

				WARN_ON(ser_pdata->num_serial_links < 1);

				ser_pdata->serial_links[0].des_client = common->client;
				ser_pdata->serial_links[0].des_link_id = link_id;
				/*
				 * Isolate this link until after reset and potential address remapping,
				 * avoiding a race condition with two serializers resetting same
				 * physical i2c at the same time
				 */
				ret = max9x_des_isolate_serial_link(common, link_id);
				if (!ret)
					subdev = v4l2_i2c_new_subdev_board(
						sd->v4l2_dev,
						common->muxc->adapter[link_id],
						&subdev_pdata->board_info,
						NULL);

				ret = max9x_des_deisolate_serial_link(common, link_id);
				if (ret)
					return ret;

				if (IS_ERR_OR_NULL(subdev)) {
					dev_err(dev, "Failure registering serializer %s (0x%02x)",
						subdev_pdata->board_info.type,
						subdev_pdata->board_info.addr);
					return PTR_ERR(subdev);
				}

				dev_dbg(dev, "Registered serializer %s (0x%02x)",
					subdev_pdata->board_info.type,
					subdev_pdata->board_info.addr);

				struct max9x_common *ser_common = max9x_sd_to_common(subdev);

				int remote_pad = max9x_serial_link_to_pad(ser_common, 0);
				int local_pad = max9x_serial_link_to_pad(common, link_id);

				dev_dbg(dev, "Create link from ser link 0 (pad %d) -> des link %d (pad %d)",
						remote_pad, link_id, local_pad);

				ret = media_create_pad_link(&subdev->entity, remote_pad,
							    &sd->entity, local_pad,
							    MEDIA_LNK_FL_IMMUTABLE |
							    MEDIA_LNK_FL_ENABLED);
				if (ret) {
					dev_err(dev, "Failed creating pad link to serializer");
					return ret;
				}

				common->serial_link[link_id].remote.client = ser_common->client;
			}
		} else {
			struct max9x_pdata *pdata = dev->platform_data;

			for (unsigned int i = 0; i < pdata->num_subdevs; i++) {
				struct max9x_subdev_pdata *subdev_pdata = &pdata->subdevs[i];

				if (subdev_pdata->serial_link_id == link_id) {
					char dev_id[I2C_NAME_SIZE];

					snprintf(dev_id, sizeof(dev_id), "i2c-%s",
						 subdev_pdata->board_info.dev_name);

					dev_dbg(dev, "Registering sensor %s (%s)...",
						subdev_pdata->board_info.type, dev_id);

					struct gpiod_lookup_table *sensor_gpios;

					sensor_gpios =
						devm_kzalloc(dev,
							     struct_size(sensor_gpios, table,
									 MAX_SER_GPIO_NUM + 1),
							     GFP_KERNEL);
					if (!sensor_gpios)
						return -ENOMEM;

					sensor_gpios->dev_id = dev_id;

					int line = 0;
					for (int i = 0; i < MAX_SER_GPIO_NUM; i++) {
						if (subdev_pdata->gpio &&
						    subdev_pdata->gpio[i].con_id != NULL) {
							sensor_gpios->table[line] =
								subdev_pdata->gpio[i];
							sensor_gpios->table[line++].key =
								common->gpio_chip.label;
							dev_dbg(dev, " Adding line %d as %s", i,
								subdev_pdata->gpio[i].con_id);
						}
					}

					gpiod_add_lookup_table(sensor_gpios);

					struct v4l2_subdev *subdev =
						v4l2_i2c_new_subdev_board(sd->v4l2_dev,
									  common->muxc->adapter[link_id],
									  &subdev_pdata->board_info, NULL);

					gpiod_remove_lookup_table(sensor_gpios);
					devm_kfree(dev, sensor_gpios);

					if (IS_ERR_OR_NULL(subdev)) {
						dev_err(dev,
							"Failure registering sensor %s (0x%02x)",
							subdev_pdata->board_info.type,
							subdev_pdata->board_info.addr);
						return PTR_ERR(subdev);
					}

					dev_dbg(dev, "Registered sensor %s (0x%02x)",
						subdev_pdata->board_info.type,
						subdev_pdata->board_info.addr);

					int remote_pad = media_get_pad_index(&subdev->entity,
									     MEDIA_PAD_FL_SOURCE,
									     PAD_SIGNAL_DEFAULT);
					int local_pad = max9x_csi_link_to_pad(common, 0);

					dev_dbg(dev, "Create link from sen pad %d -> ser link %d (pad %d)",
						remote_pad, link_id,
						local_pad);

					ret = media_create_pad_link(&subdev->entity, remote_pad,
								    &sd->entity, local_pad,
								    MEDIA_LNK_FL_IMMUTABLE |
								    MEDIA_LNK_FL_ENABLED);
					if (ret) {
						dev_err(dev, "Failed creating pad link to serializer");
						return ret;
					}

				}
			}
		}
	}

	return 0;
}

static struct v4l2_subdev_internal_ops max9x_sd_internal_ops = {
	.registered = max9x_registered,
	.init_state = max9x_init_state,
};

static int max9x_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct max9x_serdes_v4l *v4l = container_of(ctrl->handler, struct max9x_serdes_v4l, ctrl_handler);
	struct max9x_common *common = container_of(v4l, struct max9x_common, v4l);
	struct device *dev = common->dev;

	dev_dbg(dev, "s_ctrl");

	switch (ctrl->id) {
	case V4L2_CID_LINK_FREQ: {
		if (ctrl->p_new.p_u8) {
			if (*ctrl->p_new.p_u8 <= (ARRAY_SIZE(max9x_op_sys_clock) - 1)) {
				unsigned int csi_link_id;
				struct v4l2_ctrl *link_freq = v4l->link_freq;

				dev_info(dev, "user-modified %s index val=%d to user-val=%d",
					 ctrl->name, (unsigned int)link_freq->val,
					 (unsigned int)*ctrl->p_new.p_u8);

				link_freq->val = (s32) *ctrl->p_new.p_u8;

				for (csi_link_id = 0; csi_link_id < common->num_csi_links; csi_link_id++) {
					if (!common->csi_link[csi_link_id].enabled)
						continue;

					common->csi_link[csi_link_id].config.freq_mhz =
						MAX9X_LINK_FREQ_HZ_TO_MBPS(max9x_op_sys_clock[link_freq->val]);

					if (common->csi_link[csi_link_id].config.freq_mhz > 1500) {
						common->csi_link[csi_link_id].config.auto_init_deskew_enabled = true;
						common->csi_link[csi_link_id].config.initial_deskew_width = 7;
					}
				}
			}
		}
	}
	break;

	default:
		dev_info(dev, "unknown control id: 0x%X", ctrl->id);
	}

	return 0;
}

static const struct v4l2_ctrl_ops max9x_ctrl_ops = {
	.s_ctrl = max9x_s_ctrl,
};

static struct v4l2_ctrl_config max9x_v4l2_controls[] = {
	{
		.ops = &max9x_ctrl_ops,
		.id = V4L2_CID_LINK_FREQ,
		.name = "V4L2_CID_LINK_FREQ",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.min = 0,
		.max = ARRAY_SIZE(max9x_op_sys_clock) - 1,
		.def = 2,
		.menu_skip_mask = 0,
		.qmenu_int = max9x_op_sys_clock,
	},
};

static int max9x_csi_link_to_pad(struct max9x_common *common, int csi_id)
{
	return (csi_id < common->num_csi_links) ? csi_id : -EINVAL;
}

static int max9x_serial_link_to_pad(struct max9x_common *common, int link_id)
{
	dev_dbg(common->dev, "link_id %d < num_serial_links %d ? num_csi_links %d + link_id %d : -EINVAL",
		link_id, common->num_serial_links, common->num_csi_links, link_id);
	return (link_id < common->num_serial_links) ? common->num_csi_links + link_id : -EINVAL;
}

static int max9x_register_v4l_subdev(struct max9x_common *common)
{
	struct i2c_client *client = common->client;
	struct device *dev = common->dev;
	struct max9x_serdes_v4l *v4l = &common->v4l;
	struct v4l2_subdev *sd = &v4l->sd;
	struct v4l2_ctrl_handler *ctrl_handler = &v4l->ctrl_handler;
	struct max9x_pdata *pdata = dev->platform_data;
	int ret;

	mutex_init(&v4l->lock);

	v4l2_i2c_subdev_init(sd, client, &max9x_sd_ops);
	snprintf(sd->name, sizeof(sd->name), "%s %s", client->name, pdata->suffix);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_STREAMS;
	sd->internal_ops = &max9x_sd_internal_ops;
	sd->entity.function = MEDIA_ENT_F_VID_MUX;

	v4l->num_pads = common->num_csi_links + common->num_serial_links;
	v4l->pads = devm_kzalloc(dev, v4l->num_pads * sizeof(*v4l->pads), GFP_KERNEL);
	v4l->ffmts = devm_kzalloc(dev, v4l->num_pads * sizeof(*v4l->ffmts), GFP_KERNEL);
	if (!v4l->pads || !v4l->ffmts)
		return -ENOMEM;

	/* change sink/source turn */
	for (unsigned int p = 0; p < v4l->num_pads; p++) {
		struct media_pad *pad = &v4l->pads[p];

		if (p < common->num_csi_links) {
			unsigned int c = p;

			dev_dbg(dev, "pad %d/%d -> csi %d", p, v4l->num_pads, c);

			pad->flags = (common->type == MAX9X_SERIALIZER) ? MEDIA_PAD_FL_SINK : MEDIA_PAD_FL_SOURCE;
		} else if (p >= common->num_csi_links) {
			unsigned int s = p - common->num_csi_links;

			dev_dbg(dev, "pad %d/%d -> serial %d", p, v4l->num_pads, s);

			pad->flags = (common->type == MAX9X_SERIALIZER) ? MEDIA_PAD_FL_SOURCE : MEDIA_PAD_FL_SINK;
		}
	}

	ret = v4l2_ctrl_handler_init(ctrl_handler, ARRAY_SIZE(max9x_v4l2_controls));
	if (ret) {
		dev_err(dev, "Failed to init V4L2 controls: %d", ret);
		return ret;
	}

	for (int i = 0; i < ARRAY_SIZE(max9x_v4l2_controls); i++) {
		struct v4l2_ctrl_config *ctrl_config = &max9x_v4l2_controls[i];
		struct v4l2_ctrl *ctrl;

		if (ctrl_config->id == V4L2_CID_LINK_FREQ) {
			unsigned int link_freq_n;
			unsigned int csi_link_id;

			for (link_freq_n = 0; link_freq_n < ARRAY_SIZE(max9x_op_sys_clock); link_freq_n++) {
				unsigned int link_freq = MAX9X_LINK_FREQ_HZ_TO_MBPS(max9x_op_sys_clock[link_freq_n]);

				for (csi_link_id = 0; csi_link_id < common->num_csi_links; csi_link_id++) {
					if (!common->csi_link[csi_link_id].enabled)
						continue;
					if (common->csi_link[csi_link_id].config.freq_mhz == link_freq) {
						ctrl_config->def = link_freq_n;
						break;
					}
				}
			}
		}

		ctrl = v4l2_ctrl_new_custom(ctrl_handler, ctrl_config, common);
		if (!ctrl) {
			ret = ctrl_handler->error;
			dev_err(dev, "Failed to create V4L2 control %s: %d", ctrl_config->name, ret);
			goto probe_error_v4l2_ctrl_handler_free;
		}
	}

	v4l->link_freq = v4l2_ctrl_find(ctrl_handler, V4L2_CID_LINK_FREQ);
	v4l->sd.ctrl_handler = ctrl_handler;

	ret = media_entity_pads_init(&v4l->sd.entity, v4l->num_pads, v4l->pads);
	if (ret) {
		dev_err(dev, "Failed to init media entity: %d", ret);
		goto probe_error_v4l2_ctrl_handler_free;
	}

	ret = v4l2_subdev_init_finalize(&v4l->sd);
	if (ret) {
		dev_err(dev, "failed to init v4l2 subdev: %d\n", ret);
		media_entity_cleanup(&v4l->sd.entity);
		goto probe_error_media_entity_cleanup;
	}

	ret = v4l2_async_register_subdev(&v4l->sd);
	if (ret) {
		dev_err(dev, "v4l register failed: %d", ret);
		goto probe_error_media_entity_cleanup;
	}

	return 0;

probe_error_media_entity_cleanup:
	media_entity_cleanup(&v4l->sd.entity);
probe_error_v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(ctrl_handler);
	return ret;
}

/*
 * max9x_enable_serial_link() - Enable power and logic to link.
 */
int max9x_enable_serial_link(struct max9x_common *common, unsigned int link_id)
{
	struct max9x_serdes_serial_link *serial_link;
	struct device *dev = common->dev;
	int ret;

	serial_link = &common->serial_link[link_id];

	if (!serial_link->regulator_enabled) {
		if (!IS_ERR_OR_NULL(serial_link->poc_regulator)) {
			ret = regulator_enable(serial_link->poc_regulator);
			if (ret) {
				dev_err(dev, "Failed to enable %s", MAX9X_POC_REGULATOR_NAME);
				return ret;
			}
		}

		serial_link->regulator_enabled = true;
	}

	if (common->serial_link_ops && common->serial_link_ops->enable)
		return common->serial_link_ops->enable(common, link_id);

	return 0;
}

/*
 * max9x_disable_serial_link() - Disable power and logic to link.
 */
int max9x_disable_serial_link(struct max9x_common *common, unsigned int link_id)
{
	struct max9x_serdes_serial_link *serial_link;
	struct device *dev = common->dev;
	int ret;

	if (unlikely(link_id >= common->num_serial_links))
		return 0;

	serial_link = &common->serial_link[link_id];

	if (serial_link->regulator_enabled) {
		if (common->serial_link_ops && common->serial_link_ops->disable)
			common->serial_link_ops->disable(common, link_id);

		serial_link->regulator_enabled = false;

		if (!IS_ERR_OR_NULL(serial_link->poc_regulator)) {
			ret = regulator_disable(serial_link->poc_regulator);

			if (ret) {
				dev_err(dev, "Failed to disable %s", MAX9X_POC_REGULATOR_NAME);
				return ret;
			}
		}
	}

	return 0;
}

/*
 *  max9x_sysfs_create_get_link() - Creates a sysfs virtual file to check link lock status
 */
int max9x_sysfs_create_get_link(struct max9x_common *common, unsigned int link_id)
{
	struct device *dev = common->dev;
	int ret;
	char *attr_name;

	if (common->serial_link_ops && common->serial_link_ops->get_locked) {
		struct device_attribute *link_lock_status =
			devm_kzalloc(dev, sizeof(struct device_attribute), GFP_KERNEL);

		if (!link_lock_status) {
			dev_err(dev, "Failed to allocate memory for link lock status");
			return -ENOMEM;
		}

		attr_name = (char *)devm_kzalloc(dev, sizeof(char) * ATTR_NAME_LEN, GFP_KERNEL);
		if (!attr_name) {
			dev_err(dev, "Failed to allocate memory link lock attribute name");
			return -ENOMEM;
		}

		ret = snprintf(attr_name, ATTR_NAME_LEN, "link-lock_%d", link_id);
		if (ret < 0)
			return ret;

		link_lock_status->attr.name = attr_name;
		link_lock_status->attr.mode = ATTR_READ_ONLY;
		link_lock_status->show = max9x_link_status_show;

		ret = device_create_file(dev, link_lock_status);
		if (ret < 0)
			return ret;

		common->serial_link[link_id].link_lock_status = link_lock_status;
	}

	return 0;
}

/*
 *  max9x_sysfs_destroy_get_link() - Destroys the sysfs device attribute for link lock status
 */
static void max9x_sysfs_destroy_get_link(struct max9x_common *common, unsigned int link_id)
{
	struct device *dev = common->dev;

	if (common->serial_link[link_id].link_lock_status)
		device_remove_file(dev, common->serial_link[link_id].link_lock_status);
}

/*
 * max9x_enable_line_faults() - Enables all the line fault monitors using the device tree
 */
int max9x_enable_line_faults(struct max9x_common *common)
{
	return 0;
}

/*
 * max9x_disable_line_faults() - Disables all currently stored line fault monitors
 */
int max9x_disable_line_faults(struct max9x_common *common)
{
	struct device *dev = common->dev;
	int ret;
	int line;
	int final_ret = 0;

	if (common->line_fault &&
		common->line_fault_ops &&
		common->line_fault_ops->disable) {

		for (line = 0; line < common->num_line_faults; line++) {
			ret = common->line_fault_ops->disable(common, line);

			if (ret) {
				dev_err(dev, "Failed to disable line fault %d", line);
				final_ret = ret;
			}

			common->line_fault[line].enabled = false;
		}
	}

	return final_ret;
}

static void max9x_sysfs_destroy_line_fault_status(struct max9x_common *common, unsigned int line)
{
	struct device *dev = common->dev;

	if (common->line_fault[line].line_fault_status)
		device_remove_file(dev, common->line_fault[line].line_fault_status);
}

static int max9x_parse_pdata(struct max9x_common *common, struct max9x_pdata *pdata)
{
	int ret;

	for (int i = 0; i < pdata->num_serial_links; i++) {
		ret = max9x_parse_serial_link_pdata(common, &pdata->serial_links[i]);
		if (ret)
			goto err;
	}

	for (int i = 0; i < pdata->num_video_pipes; i++) {
		ret = max9x_parse_video_pipe_pdata(common, &pdata->video_pipes[i]);
		if (ret)
			goto err;
	}

	for (int i = 0; i < pdata->num_csi_links; i++) {
		ret = max9x_parse_csi_link_pdata(common, &pdata->csi_links[i]);
		if (ret)
			goto err;
	}

	for (int i = 0; i < pdata->num_subdevs; i++) {
		ret = max9x_parse_subdev_pdata(common, &pdata->subdevs[i]);
		if (ret)
			goto err;
	}

	common->external_refclk_enable = pdata->external_refclk_enable;

err:
	return ret;
}

static int max9x_parse_serial_link_pdata(struct max9x_common *common,
					 struct max9x_serial_link_pdata *serial_link_pdata)
{
	struct device *dev = common->dev;
	unsigned int serial_link_id = serial_link_pdata->link_id;

	if (serial_link_id >= common->num_serial_links) {
		dev_err(dev, "Serial link pdata: Invalid link id");
		return -EINVAL;
	}

	struct max9x_serdes_serial_link *serial_link = &common->serial_link[serial_link_id];

	serial_link->enabled = true;

	serial_link->config.link_type = serial_link_pdata->link_type;
	serial_link->config.rx_freq_mhz = serial_link_pdata->rx_freq_mhz;
	serial_link->config.tx_freq_mhz = serial_link_pdata->tx_freq_mhz;

	if (serial_link_pdata->poc_regulator[0] != 0) {
		serial_link->poc_regulator = devm_regulator_get_optional(dev, serial_link_pdata->poc_regulator);

		if (PTR_ERR(serial_link->poc_regulator) == -EPROBE_DEFER) {
			dev_dbg(dev, "POC regulator not ready deferring...");
			return -EPROBE_DEFER;
		}
		if (IS_ERR_OR_NULL(serial_link->poc_regulator))
			dev_dbg(dev, "Missing POC regulator");
	}

	return 0;
}

static int max9x_parse_video_pipe_pdata(struct max9x_common *common,
					struct max9x_video_pipe_pdata *video_pipe_pdata)
{
	struct device *dev = common->dev;
	unsigned int serial_link_id = video_pipe_pdata->serial_link_id;
	unsigned int pipe_id = video_pipe_pdata->pipe_id;
	unsigned int max_maps;
	unsigned int max_data_types;

	if (serial_link_id >= common->num_serial_links) {
		dev_err(dev, "Video pdata: Invalid serial link id");
		return -EINVAL;
	}
	if (pipe_id >= common->num_video_pipes) {
		dev_err(dev, "Video pdata: Invalid video pipe id");
		return -EINVAL;
	}

	struct max9x_serdes_video_pipe *pipe = &common->video_pipe[pipe_id];

	pipe->enabled = true;

	max_maps = 0;
	max_data_types = 0;
	if (common->common_ops && common->common_ops->max_elements) {
		max_maps = common->common_ops->max_elements(common, MAX9X_MIPI_MAP);
		max_data_types = common->common_ops->max_elements(common, MAX9X_DATA_TYPES);
	}

	if (common->type == MAX9X_DESERIALIZER) {
		if (video_pipe_pdata->num_maps > max_maps) {
			dev_err(dev, "Video pdata: Too many maps");
			return -EINVAL;
		}

		pipe->config.map = devm_kzalloc(dev,
						video_pipe_pdata->num_maps * sizeof(*pipe->config.map),
						GFP_KERNEL);
		if (!pipe->config.map) {
			dev_err(dev, "Video pdata: Failed t oallocate mmeory for maps");
			return -ENOMEM;
		}

		pipe->config.src_link = video_pipe_pdata->serial_link_id;
		pipe->config.src_pipe = video_pipe_pdata->src_pipe_id;

		for (unsigned int i = 0; i < video_pipe_pdata->num_maps; i++) {
			struct max9x_serdes_mipi_map *map = &pipe->config.map[i];
			struct max9x_serdes_mipi_map *map_pdata = &video_pipe_pdata->maps[i];

			map->src_vc = map_pdata->src_vc;
			map->src_dt = map_pdata->src_dt;
			map->dst_vc = map_pdata->dst_vc;
			map->dst_dt = map_pdata->dst_dt;
			map->dst_csi = map_pdata->dst_csi;
		}
		pipe->config.num_maps = video_pipe_pdata->num_maps;
	} else if (common->type == MAX9X_SERIALIZER) {
		if (video_pipe_pdata->num_data_types > max_data_types) {
			dev_err(dev, "Video pdata: Too many maps");
			return -EINVAL;
		}

		pipe->config.data_type = devm_kzalloc(dev,
						      video_pipe_pdata->num_data_types * sizeof(*pipe->config.map),
						      GFP_KERNEL);
		if (!pipe->config.data_type) {
			dev_err(dev, "Video pdata: Failed t oallocate mmeory for data types");
			return -ENOMEM;
		}

		pipe->config.src_csi = video_pipe_pdata->src_csi_id;

		for (unsigned int i = 0; i < video_pipe_pdata->num_data_types; i++) {
			pipe->config.data_type[i] = video_pipe_pdata->data_types[i];
		}
		pipe->config.num_data_types = video_pipe_pdata->num_data_types;
	}

	return 0;
}

static int max9x_parse_csi_link_pdata(struct max9x_common *common,
				      struct max9x_csi_link_pdata *csi_link_pdata)
{
	unsigned int csi_link_id = csi_link_pdata->link_id;

	if (csi_link_id >= common->num_csi_links) {
		dev_err(common->dev, "CSI link pdata: Invalid link id");
		return -EINVAL;
	}

	struct max9x_serdes_csi_link *csi_link = &common->csi_link[csi_link_id];

	mutex_init(&csi_link->csi_mutex);
	csi_link->enabled = true;

	csi_link->config.num_maps = csi_link_pdata->num_maps;
	csi_link->config.map =
		devm_kzalloc(common->dev,
			     csi_link->config.num_maps * sizeof(*csi_link->config.map),
			     GFP_KERNEL);
	if (!csi_link->config.map)
		return -ENOMEM;
	memcpy(csi_link->config.map, csi_link_pdata->maps,
		   csi_link->config.num_maps * sizeof(*csi_link->config.map));
	csi_link->config.bus_type = csi_link_pdata->bus_type;
	csi_link->config.num_lanes = csi_link_pdata->num_lanes;
	csi_link->config.freq_mhz = csi_link_pdata->tx_rate_mbps;
	csi_link->config.auto_init_deskew_enabled = csi_link_pdata->auto_initial_deskew;

	if (csi_link->config.auto_init_deskew_enabled)
		csi_link->config.initial_deskew_width = csi_link_pdata->initial_deskew_width;
	csi_link->config.auto_start = csi_link_pdata->auto_start;

	return 0;
}

static int max9x_parse_subdev_pdata(struct max9x_common *common,
				    struct max9x_subdev_pdata *subdev_pdata)
{
	unsigned int serial_link_id = subdev_pdata->serial_link_id;
	struct max9x_serdes_serial_link *serial_link;

	if (serial_link_id >= common->num_serial_links) {
		dev_err(common->dev, "Subdev pdata: Invalid serial link id");
		return -EINVAL;
	}

	serial_link = &common->serial_link[serial_link_id];

	if (!serial_link->enabled)
		return 0;

	serial_link->remote.pdata = subdev_pdata;

	return 0;
}

int max9x_select_i2c_chan(struct i2c_mux_core *muxc, u32 chan_id)
{
	struct max9x_common *common = i2c_mux_priv(muxc);
	int ret = 0;
	unsigned long timeout = jiffies + msecs_to_jiffies(10000);

	if (unlikely(chan_id > common->num_serial_links))
		return -EINVAL;

	do {
		mutex_lock(&common->isolate_mutex);
		if (common->selected_link < 0 || chan_id == common->selected_link)
			break;

		mutex_unlock(&common->isolate_mutex);

		usleep_range(1000, 1050);

		if (time_is_before_jiffies(timeout)) {
			dev_warn(common->dev, "select %d TIMEOUT", chan_id);
			return -ETIMEDOUT;
		}
	} while (1);

	common->selected_link = chan_id;

	if (common->serial_link_ops && common->serial_link_ops->select)
		ret = common->serial_link_ops->select(common, chan_id);

	mutex_unlock(&common->isolate_mutex);

	return ret;
}

int max9x_deselect_i2c_chan(struct i2c_mux_core *muxc, u32 chan_id)
{
	struct max9x_common *common = i2c_mux_priv(muxc);
	int ret = 0;

	if (unlikely(chan_id > common->num_serial_links))
		return -EINVAL;

	mutex_lock(&common->isolate_mutex);
	if (common->serial_link_ops && common->serial_link_ops->deselect)
		ret = common->serial_link_ops->deselect(common, chan_id);

	common->selected_link = -1;
	mutex_unlock(&common->isolate_mutex);

	return ret;
}

int max9x_des_isolate_serial_link(struct max9x_common *common, unsigned int link_id)
{
	int ret = 0;
	unsigned long timeout = jiffies + msecs_to_jiffies(10000);

	if (link_id >= common->num_serial_links) {
		dev_err(common->dev, "link_id %d outside of num_serial_links %d", link_id, common->num_serial_links);
		return -EINVAL;
	}

	dev_info(common->dev, "Isolate %d", link_id);

	do {
		mutex_lock(&common->isolate_mutex);
		if ((common->isolated_link < 0) && (common->selected_link < 0 || link_id == common->selected_link))
			break;

		if (common->isolated_link == link_id) {
			dev_warn(common->dev, "Link %d is already isolated", link_id);
			mutex_unlock(&common->isolate_mutex);
			return -EINVAL;
		}
		mutex_unlock(&common->isolate_mutex);

		usleep_range(1000, 1050);

		if (time_is_before_jiffies(timeout))
			return -ETIMEDOUT;
	} while (1);

	common->isolated_link = link_id;
	if (common->serial_link_ops && common->serial_link_ops->isolate)
		ret = common->serial_link_ops->isolate(common, link_id);

	mutex_unlock(&common->isolate_mutex);
	dev_info(common->dev, "Isolate %d complete", link_id);

	return ret;
}

int max9x_des_deisolate_serial_link(struct max9x_common *common, unsigned int link_id)
{
	int ret = 0;

	if (link_id >= common->num_serial_links)
		return -EINVAL;

	dev_info(common->dev, "Deisolate %d", link_id);

	mutex_lock(&common->isolate_mutex);
	if (common->serial_link_ops && common->serial_link_ops->deisolate)
		ret = common->serial_link_ops->deisolate(common, link_id);

	common->isolated_link = -1;
	dev_info(common->dev, "Deisolate %d complete", link_id);
	mutex_unlock(&common->isolate_mutex);

	return ret;
}

ssize_t max9x_link_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max9x_common *common = dev_get_drvdata(dev);
	int link;
	int ret;
	bool locked;

	ret = sscanf(attr->attr.name, "link-lock_%d", &link);
	if (ret < 0)
		return ret;

	if (common->serial_link_ops && common->serial_link_ops->get_locked) {
		ret = common->serial_link_ops->get_locked(common, link, &locked);
		if (ret < 0)
			return ret;

		return sysfs_emit(buf, "%d", !!locked);
	}

	dev_err(dev, "get_locked not defined");
	return -EINVAL;
}

int max9x_setup_translations(struct max9x_common *common)
{
	int err = 0;

	if (!common->translation_ops)
		return 0;

	/*
	 * Translation is currently only supported on serializer side
	 * (translating requests from SOC to remote sensor module)
	 */
	if (common->type != MAX9X_SERIALIZER)
		return 0;

	struct max9x_pdata *pdata = common->dev->platform_data;

	for (unsigned int i = 0; i < pdata->num_subdevs; i++) {
		struct max9x_subdev_pdata *subdev_pdata = &pdata->subdevs[i];
		unsigned int virt_addr = subdev_pdata->board_info.addr;
		unsigned int phys_addr = subdev_pdata->phys_addr ? subdev_pdata->phys_addr : virt_addr;

		if (virt_addr == phys_addr || common->translation_ops->add == NULL)
			continue;

		/* Only I2C 1 is supported at this time */
		err = common->translation_ops->add(common, 0, virt_addr, phys_addr);
		if (err)
			dev_warn(common->dev, "Failed to add translation for i2c address 0x%02x -> 0x%02x: %d",
				 virt_addr, phys_addr, err);
		break;
	}

	return err;
}

int max9x_disable_translations(struct max9x_common *common)
{
	int err = 0;

	if (!common->translation_ops)
		return 0;

	/*
	 * Translation is currently only supported on serializer side
	 * (translating requests from SOC to remote sensor module)
	 */
	if (common->type != MAX9X_SERIALIZER)
		return 0;

	struct max9x_pdata *pdata = common->dev->platform_data;

	for (unsigned int i = 0; i < pdata->num_subdevs; i++) {
		struct max9x_subdev_pdata *subdev_pdata = &pdata->subdevs[i];
		unsigned int virt_addr = subdev_pdata->board_info.addr;
		unsigned int phys_addr = subdev_pdata->phys_addr ? subdev_pdata->phys_addr : virt_addr;

		if (virt_addr != phys_addr) {
			if (common->translation_ops->remove) {
				/* Only I2C 1 is supported at this time */
				err = common->translation_ops->remove(common, 0, virt_addr, phys_addr);
				if (err)
					dev_err(common->dev, "Failed to remove translation for i2c address 0x%02x -> 0x%02x: %d",
							virt_addr, phys_addr, err);
				break;
			}
		}
	}

	return err;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0)
static int max9x_des_probe(struct i2c_client *client, const struct i2c_device_id *id)
#else
static int max9x_des_probe(struct i2c_client *client)
#endif
{
	struct device *dev = &client->dev;
	struct max9x_common *des = NULL;
	int ret = 0;

	dev_info(dev, "Probing");

	des = devm_kzalloc(dev, sizeof(*des), GFP_KERNEL);
	if (!des) {
		dev_err(dev, "Failed to allocate memory.");
		return -ENOMEM;
	}

	TRY(ret, max9x_common_init_i2c_client(
		des,
		client,
		&max9x_regmap_config,
		NULL, NULL, NULL, NULL));

	dev_info(dev, "probe successful");
	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
static int max9x_des_remove(struct i2c_client *client)
#else
static void max9x_des_remove(struct i2c_client *client)
#endif
{
	struct device *dev = &client->dev;
	struct max9x_common *des = NULL;

	dev_info(dev, "Removing");

	des = max9x_client_to_common(client);

	max9x_destroy(des);

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	return 0;
#endif
}

static int max9x_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max9x_common *common = max9x_client_to_common(client);

	return max9x_common_resume(common);
}

static int max9x_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max9x_common *common = max9x_client_to_common(client);

	return max9x_common_suspend(common);
}

static const struct dev_pm_ops max9x_des_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(max9x_suspend, max9x_resume)
};

static struct i2c_driver max9x_driver = {
	.driver		= {
		.name	= "max9x",
		.of_match_table = max9x_of_match,
		.pm = &max9x_des_pm_ops,
	},
	.probe		= max9x_des_probe,
	.remove		= max9x_des_remove,
	.id_table	= max9x_id,
};

module_i2c_driver(max9x_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Josh Watts <jwatts@d3embedded.com>");
MODULE_AUTHOR("Yan, Dongcheng <dongcheng.yan@intel.com>");
MODULE_DESCRIPTION("Common logic for Maxim GMSL serializers & deserializers");
