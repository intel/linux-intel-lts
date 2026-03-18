/*
 * max96717_main.c - Maxim MAX96717 CSI-2 to GMSL2/GMSL1 Serializer
 *
 * Copyright (c) 2022, D3 Engineering.  All rights reserved.
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
// Copyright (c) 2025 Intel Corporation.

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/regmap.h>
#include <linux/sysfs.h>
#include <linux/slab.h>

#include "max96717.h"

static const struct regmap_config max96717_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
};

// Declarations
static int max96717_set_pipe_csi_enabled(struct max9x_common *common, unsigned int pipe_id,
					 unsigned int csi_id, bool enable);
static int max96717_video_pipe_double_pixel(struct max9x_common *common, unsigned int pipe_id, unsigned int bpp);
static int max96717_max_elements(struct max9x_common *common, enum max9x_element_type element);
static int max96717_enable_serial_link(struct max9x_common *common, unsigned int link);
static int max96717_disable_serial_link(struct max9x_common *common, unsigned int link);
static int max96717_enable(struct max9x_common *common);
static int max96717_disable(struct max9x_common *common);
static int max96717_pixel_mode(struct max9x_common *common, bool pixel);

static struct max9x_common *from_gpio_chip(struct gpio_chip *chip);
static int max96717_gpio_get_direction(struct gpio_chip *chip, unsigned int offset);
static int max96717_gpio_direction_input(struct gpio_chip *chip, unsigned int offset);
static int max96717_gpio_direction_output(struct gpio_chip *chip, unsigned int offset, int value);
static int max96717_gpio_get(struct gpio_chip *chip, unsigned int offset);
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 17, 0)
static void max96717_gpio_set(struct gpio_chip *chip, unsigned int offset, int value);
#else
static int max96717_gpio_set(struct gpio_chip *chip, unsigned int offset, int value);
#endif
static int max96717_setup_gpio(struct max9x_common *common);

static struct max9x_common_ops max96717_common_ops = {
	.max_elements = max96717_max_elements,
	.enable = max96717_enable,
	.disable = max96717_disable,
};

static struct max9x_serial_link_ops max96717_serial_link_ops = {
	.enable = max96717_enable_serial_link,
	.disable = max96717_disable_serial_link,
};

static struct max9x_translation_ops max96717_translation_ops;

static struct max9x_common *from_gpio_chip(struct gpio_chip *chip)
{
	return container_of(chip, struct max9x_common, gpio_chip);
}

static int max96717_gpio_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	struct max9x_common *common = from_gpio_chip(chip);
	struct regmap *map = common->map;
	unsigned int val;
	int ret;

	ret = regmap_read(map, MAX96717_GPIO_A(offset), &val);
	if (ret)
		return ret;

	return (FIELD_GET(MAX96717_GPIO_A_OUT_DIS_FIELD, val) == 0U ?
		GPIOD_OUT_LOW : GPIOD_IN);
}

static int max96717_gpio_direction_input(struct gpio_chip *chip, unsigned int offset)
{
	struct max9x_common *common = from_gpio_chip(chip);
	struct regmap *map = common->map;

	return regmap_update_bits(map, MAX96717_GPIO_A(offset),
		MAX96717_GPIO_A_OUT_DIS_FIELD,
		MAX9X_FIELD_PREP(MAX96717_GPIO_A_OUT_DIS_FIELD, 1U));
}

static int max96717_gpio_direction_output(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct max9x_common *common = from_gpio_chip(chip);
	struct regmap *map = common->map;
	unsigned int mask = 0;
	unsigned int val;

	mask = MAX96717_GPIO_A_OUT_DIS_FIELD | MAX96717_GPIO_A_OUT_FIELD |
		MAX96717_GPIO_A_RX_EN_FIELD;

	// Enable the GPIO as an output
	val = MAX9X_FIELD_PREP(MAX96717_GPIO_A_OUT_DIS_FIELD, 0U);
	// Write out the initial value to the GPIO
	val |= MAX9X_FIELD_PREP(MAX96717_GPIO_A_OUT_FIELD, (value == 0 ? 0U : 1U));
	// Disable remote control over SerDes link
	val |= MAX9X_FIELD_PREP(MAX96717_GPIO_A_RX_EN_FIELD, 0U);

	return regmap_update_bits(map, MAX96717_GPIO_A(offset), mask, val);
}

static int max96717_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct max9x_common *common = from_gpio_chip(chip);
	struct regmap *map = common->map;
	unsigned int val;
	int ret;

	ret = regmap_read(map, MAX96717_GPIO_A(offset), &val);
	if (ret)
		return ret;

	return FIELD_GET(MAX96717_GPIO_A_IN_FIELD, val);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 17, 0)
static void max96717_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct max9x_common *common = from_gpio_chip(chip);
	struct regmap *map = common->map;

	regmap_update_bits(map, MAX96717_GPIO_A(offset),
		MAX96717_GPIO_A_OUT_FIELD,
		MAX9X_FIELD_PREP(MAX96717_GPIO_A_OUT_FIELD, (value == 0 ? 0U : 1U)));
}
#else
static int max96717_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct max9x_common *common = from_gpio_chip(chip);
	struct regmap *map = common->map;

	return regmap_update_bits(map, MAX96717_GPIO_A(offset),
		MAX96717_GPIO_A_OUT_FIELD,
		MAX9X_FIELD_PREP(MAX96717_GPIO_A_OUT_FIELD, (value == 0 ? 0U : 1U)));
}
#endif

static int max96717_setup_gpio(struct max9x_common *common)
{
	struct device *dev = common->dev;
	int ret;
	struct max9x_gpio_pdata *gpio_pdata = NULL;

	if (common->dev->platform_data) {
		struct max9x_pdata *pdata = common->dev->platform_data;
		gpio_pdata = &pdata->gpio;
	}

	// Functions
	if (gpio_pdata && gpio_pdata->label)
		common->gpio_chip.label = gpio_pdata->label;
	else
		common->gpio_chip.label = dev_name(dev);

	dev_dbg(dev, "gpio_chip label is %s, dev_name is %s",
		common->gpio_chip.label, dev_name(dev));

	// Functions
	common->gpio_chip.label = MAX96717_NAME;
	common->gpio_chip.parent = dev;
	common->gpio_chip.get_direction = max96717_gpio_get_direction;
	common->gpio_chip.direction_input = max96717_gpio_direction_input;
	common->gpio_chip.direction_output = max96717_gpio_direction_output;
	common->gpio_chip.get = max96717_gpio_get;
	common->gpio_chip.set = max96717_gpio_set;
	common->gpio_chip.ngpio = MAX96717_NUM_GPIO;
	common->gpio_chip.can_sleep = 1;
	common->gpio_chip.base = -1;
	if (gpio_pdata && gpio_pdata->names)
		common->gpio_chip.names = gpio_pdata->names;

	ret = devm_gpiochip_add_data(dev, &common->gpio_chip, common);
	if (ret < 0) {
		dev_err(dev, "gpio_init: Failed to add max96717_gpio\n");
		return ret;
	}

	return 0;
}

static int max96717_set_pipe_csi_enabled(struct max9x_common *common,
					 unsigned int pipe_id,
					 unsigned int csi_id, bool enable)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	int ret;

	dev_dbg(dev, "Video-pipe %d, csi %d: %s, %d lanes", \
		pipe_id, csi_id, (enable ? "enable" : "disable"), \
		common->csi_link[csi_id].config.num_lanes);

	// Select number of lanes for CSI port csi_id
	ret = regmap_update_bits(map, MAX96717_MIPI_RX_1,
		MAX96717_MIPI_RX_1_SEL_CSI_LANES_FIELD(csi_id),
		MAX9X_FIELD_PREP(MAX96717_MIPI_RX_1_SEL_CSI_LANES_FIELD(csi_id),
			common->csi_link[csi_id].config.num_lanes - 1));
	if (ret)
		return ret;

	// Select CSI port csi_id for video pipe pipe_id
	ret = regmap_update_bits(map, MAX96717_FRONTTOP_0,
		MAX96717_FRONTTOP_0_SEL_CSI_FIELD(pipe_id)
		| MAX96717_FRONTTOP_0_START_CSI_FIELD(csi_id),
		MAX9X_FIELD_PREP(MAX96717_FRONTTOP_0_SEL_CSI_FIELD(pipe_id), csi_id)
		| MAX9X_FIELD_PREP(MAX96717_FRONTTOP_0_START_CSI_FIELD(csi_id), enable ? 1U : 0U));
	if (ret)
		return ret;

	// Start video pipe pipe_id from CSI port csi_id
	ret =  regmap_update_bits(map, MAX96717_FRONTTOP_9,
		MAX96717_FRONTTOP_9_START_VIDEO_FIELD(pipe_id, csi_id),
		MAX9X_FIELD_PREP(MAX96717_FRONTTOP_9_START_VIDEO_FIELD(pipe_id, csi_id), enable ? 1U : 0U));
	if (ret)
		return ret;

	return 0;
}

/**
 * max96717_video_pipe_double_pixel() - Configure Double Loading Mode on a video pipe
 * @common: max9x_common
 * @pipe_id: Target pipe's ID
 * @bpp: Original BPP to double. This can be 0 (disables), 8, 10, or 12.
 *
 * Double loading mode squeezes two input pixels together such that they are
 * treated as a single pixel by the video pipe. Using this method increases
 * bandwidth efficiency.
 *
 * See: GMSL2 Customers User Guide Section 30.5.1.1.1.2 "Double Loading Mode"
 * See: GMSL2 Customers User Guide Section 43.3.4.5.1 "Double Mode (Serializer)"
 */
static int max96717_video_pipe_double_pixel(struct max9x_common *common,
					    unsigned int pipe_id, unsigned int bpp)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	int ret;

	unsigned int reg;
	unsigned int fields;
	unsigned int vals;
	unsigned int dbl_bpp;

	// Clear all the double pixel mode fields
	ret = regmap_update_bits(map,
		MAX96717_FRONTTOP_10,
		MAX96717_FRONTTOP_10_DBL8_FIELD(pipe_id),
		0x0);
	if (ret)
		return ret;

	ret = regmap_update_bits(map,
		MAX96717_FRONTTOP_11,
		MAX96717_FRONTTOP_11_DBL10_FIELD(pipe_id) |
		MAX96717_FRONTTOP_11_DBL12_FIELD(pipe_id),
		0x0);
	if (ret)
		return ret;

	// Enable/disable double pixel mode for pipe
	switch (bpp) {
	case 0:
		//bpp not used on this pipe, but still valid input
		break;
	case 8:
		reg = MAX96717_FRONTTOP_10;
		fields = MAX96717_FRONTTOP_10_DBL8_FIELD(pipe_id);
		break;
	case 10:
		reg = MAX96717_FRONTTOP_11;
		fields = MAX96717_FRONTTOP_11_DBL10_FIELD(pipe_id);
		break;
	case 12:
		reg = MAX96717_FRONTTOP_11;
		fields = MAX96717_FRONTTOP_11_DBL12_FIELD(pipe_id);
		break;
	default:
		dev_err(dev, "Unsupported BPP for pixel doubling: %u", bpp);
		return -EINVAL;
	}

	// Enable pixel doubling for specified pipe
	if (bpp != 0) {
		dev_info(dev, "Configure double loading mode for pipe %u (%ubpp -> %ubpp)",
			 pipe_id, bpp, (bpp * 2));
		ret = regmap_update_bits(map, reg, fields, 0xFF);
		if (ret)
			return ret;
	}

	// Enable software BPP override and set BPP value
	dbl_bpp = bpp * 2;

	// Override output bpp
	reg = MAX96717_FRONTTOP_2X(pipe_id);

	fields = MAX96717_FRONTTOP_2X_BPP_EN_FIELD;
	vals = MAX9X_FIELD_PREP(MAX96717_FRONTTOP_2X_BPP_EN_FIELD, bpp == 0 ? 0U : 1U);

	fields |= MAX96717_FRONTTOP_2X_BPP_FIELD;
	vals |= MAX9X_FIELD_PREP(MAX96717_FRONTTOP_2X_BPP_FIELD, dbl_bpp);

	return regmap_update_bits(map, reg, fields, vals);
}

static int max96717_max_elements(struct max9x_common *common,
				 enum max9x_element_type element)
{
	switch (element) {
	case MAX9X_SERIAL_LINK:
		return MAX96717_NUM_SERIAL_LINKS;
	case MAX9X_VIDEO_PIPE:
		return MAX96717_NUM_VIDEO_PIPES;
	case MAX9X_MIPI_MAP:
		return MAX96717_NUM_MIPI_MAPS;
	case MAX9X_CSI_LINK:
		return MAX96717_NUM_CSI_LINKS;
	default:
		break;
	}

	return 0;
}

static int max96717_enable_serial_link(struct max9x_common *common,
				       unsigned int link_id)
{
	struct device *dev = common->dev;
	unsigned int pipe_id;
	int ret;

	dev_info(dev, "Link %d: Enable", link_id);
	for (pipe_id = 0; pipe_id < common->num_video_pipes; pipe_id++) {
		struct max9x_serdes_pipe_config *config;

		if (common->video_pipe[pipe_id].enabled == false)
			continue;

		config = &common->video_pipe[pipe_id].config;
		ret = max96717_set_pipe_csi_enabled(common, pipe_id,
						    config->src_csi, true);
		if (ret)
			return ret;
		ret = max96717_video_pipe_double_pixel(common, pipe_id,
						       config->dbl_pixel_bpp);
		if (ret)
			return ret;
	}

	return 0;
}

static int max96717_disable_serial_link(struct max9x_common *common,
					unsigned int link_id)
{
	struct device *dev = common->dev;
	unsigned int pipe_id;
	int ret;

	dev_info(dev, "Link %d: Disable", link_id);

	for (pipe_id = 0; pipe_id < common->num_video_pipes; pipe_id++) {
		struct max9x_serdes_pipe_config *config;

		if (common->video_pipe[pipe_id].enabled == false)
			continue;

		config = &common->video_pipe[pipe_id].config;

		ret = max96717_set_pipe_csi_enabled(common, pipe_id,
						    config->src_csi, false);
		if (ret)
			return ret;
	}

	return 0;
}

static int max96717_disable(struct max9x_common *common)
{
	struct device *dev = common->dev;

	dev_dbg(dev, "Disable");

	return 0;
}

static int max96717_pixel_mode(struct max9x_common *common, bool pixel)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;

	dev_dbg(dev, "Pixel mode: %d", pixel);

	// Register actually enables tunnel mode. Clearing the bit "enables" pixel mode.
	return regmap_write(map, MAX96717_EXT11,
			    MAX9X_FIELD_PREP(MAX96717_TUNNEL_MODE, !pixel));
}

/**
 * Enable the MAX96717 to replicate a frame sync signal from the deserializer.
 * NOTE: Currently the MAX96717 driver supports frame sync across its
 * GPIO8.
 */
static int max96717_enable_frame_sync(struct max9x_common *common)
{
	struct device_node *node = common->dev->of_node;
	struct device *dev = common->dev;
	struct regmap *map = common->map;

	int ret;
	int deserializer_tx_id;

	ret = of_property_read_u32(node, "fsync-tx-id", &deserializer_tx_id);
	// Not necessarily problematic, no frame sync tx found
	if (ret == -ENODATA || ret == -EINVAL) {
		dev_info(dev, "Frame sync GPIO tx id not found");
		return 0;
	}
	// Other errors are problematic
	else if (ret < 0) {
		dev_err(dev, "Failed to read frame sync tx id with err %d",
			ret);
		return ret;
	}

	ret = regmap_write(map, MAX96717_GPIO_C(8), deserializer_tx_id);
	if (ret) {
		dev_err(dev, "Failed to write des frame sync id with err: %d",
			ret);
		return ret;
	}

	return 0;
}

static int max96717_get_datatype(struct max9x_common *common)
{
	struct device_node *node = common->dev->of_node;
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	int ret, datatype;

	ret = of_property_read_u32(node, "data-type", &datatype);
	if (ret == -ENODATA || ret == -EINVAL) {
		dev_dbg(dev, "Data-type not found not filtering");
		return regmap_write(map, MAX96717_FRONTTOP_16, 0);
	}
	// Other errors are problematic
	else if (ret < 0) {
		dev_err(dev, "Problem reading in data-type with err %d", ret);
		return ret;
	}

	dev_dbg(dev, "Setting image data type to %x", datatype);
	/* Filter out metadata, only use the image datatype.
	 * MAX96717_FRONTTOP_16: mem_dt1_selz.
	 * Bit 6 is enable
	 */
	return regmap_write(map, MAX96717_FRONTTOP_16,
			   (datatype & MAX96717_FRONTTOP_16_FIELD) | MAX96717_FRONTTOP_16_ENABLE);
}

static int max96717_enable(struct max9x_common *common)
{
	struct device *dev = common->dev;
	int ret;

	dev_dbg(dev, "setup gpio");
	ret = max96717_setup_gpio(common);
	if (ret)
		return ret;

	dev_dbg(dev, "setup datatype");
	ret = max96717_get_datatype(common);
	if (ret)
		return ret;

	dev_dbg(dev, "setup pixel mode");
	// this driver MUST be in pixel mode as that is what our driver architecture supports
	ret = max96717_pixel_mode(common, true);
	if (ret)
		return ret;

	dev_dbg(dev, "setup frame sync");
	ret = max96717_enable_frame_sync(common);
	if (ret)
		return ret;

	return 0;
}

int max96717_get_ops(struct max9x_common_ops **common_ops, struct max9x_serial_link_ops **serial_ops,
		    struct max9x_csi_link_ops **csi_ops, struct max9x_line_fault_ops **lf_ops,
			struct max9x_translation_ops **trans_ops)
{
	*common_ops = &max96717_common_ops;
	*serial_ops = &max96717_serial_link_ops;
	*csi_ops = NULL;
	*lf_ops = NULL;
	*trans_ops = &max96717_translation_ops;
	return 0;
}

#if 0
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0)
static int max96717_probe(struct i2c_client *client, const struct i2c_device_id *id)
#else
static int max96717_probe(struct i2c_client *client)
#endif
{
	struct device *dev = &client->dev;
	struct max9x_common *ser = NULL;
	int ret = 0;

	dev_info(dev, "Probing");

	ser = devm_kzalloc(dev, sizeof(*ser), GFP_KERNEL);
	if (!ser) {
		dev_err(dev, "Failed to allocate memory.");
		return -ENOMEM;
	}

	ret = max9x_common_init_i2c_client(
		ser,
		client,
		&max96717_regmap_config,
		&max96717_common_ops,
		&max96717_serial_link_ops,
		NULL, /* csi_link_os */
		NULL);
	if (ret)
		return ret;

	return 0;

err:
	max9x_destroy(ser);
	return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
static int max96717_remove(struct i2c_client *client)
#else
static void max96717_remove(struct i2c_client *client)
#endif
{
	struct device *dev = &client->dev;
	struct max9x_common *ser = NULL;

	dev_info(dev, "Removing");

	ser = max9x_client_to_common(client);

	max9x_destroy(ser);

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	return 0;
#endif
}

static struct i2c_device_id max96717_idtable[] = {
	{MAX96717_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, max96717_idtable);

static struct of_device_id max96717_of_match[] = {
	{ .compatible = MAX96717_NAME},
	{},
};
MODULE_DEVICE_TABLE(of, max96717_of_match);

static struct i2c_driver max96717_driver = {
	.driver = {
		.name = MAX96717_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(max96717_of_match),
	},
	.probe = max96717_probe,
	.remove = max96717_remove,
	.id_table = max96717_idtable,
};

module_i2c_driver(max96717_driver);
#endif
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Cody Burrows <cburrows@d3embedded.com>");
MODULE_AUTHOR("Josh Watts <jwatts@d3embedded.com>");
MODULE_AUTHOR("He Jiabin <jiabin.he@intel.com>");
MODULE_DESCRIPTION("Maxim MAX96717 CSI-2 to GMSL2 Serializer driver");
