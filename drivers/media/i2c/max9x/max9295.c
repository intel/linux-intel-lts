/*
 * max9295_main.c - Maxim MAX9295 CSI-2 to GMSL2/GMSL1 Serializer
 *
 * Copyright (c) 2020, D3 Engineering.  All rights reserved.
 * Copyright (c) 2023-2024, Define Design Deploy Corp.  All rights reserved.
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
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include "max9295.h"
#include "regmap-retry.h"

static const char *const max9295_gpio_chip_names[] = {
	"MFP0",
	"MFP1",
	"MFP2",
	"MFP3",
	"MFP4",
	"MFP5",
	"MFP6",
	"MFP7",
	"MFP8",
	"MFP9",
	"MFP10",
};

/* Declarations */
static int max9295_set_pipe_csi_enabled(struct max9x_common *common,
					unsigned int pipe_id, unsigned int csi_id, bool enable);
static int max9295_set_pipe_data_types_enabled(struct max9x_common *common, unsigned int pipe_id, bool enable);
static int max9295_set_video_pipe_double_loading(struct max9x_common *common, unsigned int pipe_id, unsigned int bpp);
static int max9295_set_video_pipe_pixel_padding(struct max9x_common *common,
						unsigned int pipe_id, unsigned int min_bpp, unsigned int max_bpp);
static int max9295_max_elements(struct max9x_common *common, enum max9x_element_type element);
static int max9295_enable_serial_link(struct max9x_common *common, unsigned int link);
static int max9295_disable_serial_link(struct max9x_common *common, unsigned int link);
static int max9295_set_local_control_channel_enabled(struct max9x_common *common, bool enabled);
static int max9295_select_serial_link(struct max9x_common *common, unsigned int link);
static int max9295_deselect_serial_link(struct max9x_common *common, unsigned int link);
static int max9295_verify_devid(struct max9x_common *common);
static int max9295_enable(struct max9x_common *common);
static int max9295_disable(struct max9x_common *common);
static int max9295_remap_addr(struct max9x_common *common);
static int max9295_remap_reset(struct max9x_common *common);
static int max9295_add_translate_addr(struct max9x_common *common,
				      unsigned int i2c_id, unsigned int virt_addr, unsigned int phys_addr);
static int max9295_remove_translate_addr(struct max9x_common *common,
					 unsigned int i2c_id, unsigned int virt_addr, unsigned int phys_addr);
static int max9295_reset(struct max9x_common *common);

/* max9295 gpio */
static struct max9x_common *from_gpio_chip(struct gpio_chip *chip);
static int max9295_gpio_get_direction(struct gpio_chip *chip, unsigned int offset);
static int max9295_gpio_direction_input(struct gpio_chip *chip, unsigned int offset);
static int max9295_gpio_direction_output(struct gpio_chip *chip, unsigned int offset, int value);
static int max9295_gpio_get(struct gpio_chip *chip, unsigned int offset);
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 17, 0)
static void max9295_gpio_set(struct gpio_chip *chip, unsigned int offset, int value);
#else
static int max9295_gpio_set(struct gpio_chip *chip, unsigned int offset, int value);
#endif
static int max9295_gpio_set_config(struct gpio_chip *chip, unsigned int offset, unsigned long config);

static int max9295_setup_gpio(struct max9x_common *common);
/* max9295 gpio */

static struct max9x_common_ops max9295_common_ops = {
	.max_elements = max9295_max_elements,
	.soft_reset = max9295_reset,
	.enable = max9295_enable,
	.disable = max9295_disable,
	.verify_devid = max9295_verify_devid,
	.remap_addr = max9295_remap_addr,
	.remap_reset = max9295_remap_reset,
	.setup_gpio = max9295_setup_gpio,
};

static struct max9x_serial_link_ops max9295_serial_link_ops = {
	.enable = max9295_enable_serial_link,
	.disable = max9295_disable_serial_link,
	.select = max9295_select_serial_link,
	.deselect = max9295_deselect_serial_link,
};

static struct max9x_translation_ops max9295_translation_ops = {
	.add = max9295_add_translate_addr,
	.remove = max9295_remove_translate_addr,
};

static struct max9x_common *from_gpio_chip(struct gpio_chip *chip)
{
	return container_of(chip, struct max9x_common, gpio_chip);
}

static int max9295_gpio_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	struct max9x_common *common = from_gpio_chip(chip);
	struct regmap *map = common->map;
	unsigned int val;
	int ret;

	TRY(ret, regmap_read(map, MAX9295_GPIO_A(offset), &val));

	return (FIELD_GET(MAX9295_GPIO_A_OUT_DIS_FIELD, val) == 0U ?
		GPIOD_OUT_LOW : GPIOD_IN);
}

static int max9295_gpio_direction_input(struct gpio_chip *chip, unsigned int offset)
{
	struct max9x_common *common = from_gpio_chip(chip);
	struct regmap *map = common->map;

	return regmap_update_bits(map, MAX9295_GPIO_A(offset),
			MAX9295_GPIO_A_OUT_DIS_FIELD,
			MAX9X_FIELD_PREP(MAX9295_GPIO_A_OUT_DIS_FIELD, 1U));
}

static int max9295_gpio_direction_output(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct max9x_common *common = from_gpio_chip(chip);
	struct regmap *map = common->map;
	unsigned int mask = 0;
	unsigned int val;

	mask = MAX9295_GPIO_A_OUT_DIS_FIELD | MAX9295_GPIO_A_OUT_FIELD |
	       MAX9295_GPIO_A_RX_EN_FIELD;

	// Enable the GPIO as an output
	val = MAX9X_FIELD_PREP(MAX9295_GPIO_A_OUT_DIS_FIELD, 0U);
	// Write out the initial value to the GPIO
	val |= MAX9X_FIELD_PREP(MAX9295_GPIO_A_OUT_FIELD, (value == 0 ? 0U : 1U));
	// Disable remote control over SerDes link
	val |= MAX9X_FIELD_PREP(MAX9295_GPIO_A_RX_EN_FIELD, 0U);

	return regmap_update_bits(map, MAX9295_GPIO_A(offset), mask, val);
}

static int max9295_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct max9x_common *common = from_gpio_chip(chip);
	struct regmap *map = common->map;
	unsigned int val;
	int ret;

	TRY(ret, regmap_read(map, MAX9295_GPIO_A(offset), &val));

	return FIELD_GET(MAX9295_GPIO_A_IN_FIELD, val);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 17, 0)
static void max9295_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct max9x_common *common = from_gpio_chip(chip);
	struct regmap *map = common->map;

	regmap_update_bits(map, MAX9295_GPIO_A(offset),
		MAX9295_GPIO_A_OUT_FIELD,
		MAX9X_FIELD_PREP(MAX9295_GPIO_A_OUT_FIELD, (value == 0 ? 0U : 1U)));
}
#else
static int max9295_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct max9x_common *common = from_gpio_chip(chip);
	struct regmap *map = common->map;

	return regmap_update_bits(map, MAX9295_GPIO_A(offset),
		MAX9295_GPIO_A_OUT_FIELD,
		MAX9X_FIELD_PREP(MAX9295_GPIO_A_OUT_FIELD, (value == 0 ? 0U : 1U)));
}
#endif

static int max9295_gpio_set_config(struct gpio_chip *chip, unsigned int offset, unsigned long config)
{
	struct max9x_common *common = from_gpio_chip(chip);
	struct regmap *map = common->map;
	unsigned int out_type_mask = MAX9295_GPIO_B_OUT_TYPE_FIELD;
	unsigned int out_type_val;

	// TODO: Add pull-up and pull-down support

	// open drain is NOT requested, configure to push pull
	if ((config & GPIO_OPEN_DRAIN) == 0)
		out_type_val = MAX9X_FIELD_PREP(MAX9295_GPIO_B_OUT_TYPE_FIELD, 1U);
	else
		out_type_val = MAX9X_FIELD_PREP(MAX9295_GPIO_B_OUT_TYPE_FIELD, 0U);

	return regmap_update_bits(map, MAX9295_GPIO_B(offset), out_type_mask, out_type_val);
}

static int max9295_setup_gpio(struct max9x_common *common)
{
	struct device *dev = common->dev;
	int ret;
	struct max9x_gpio_pdata *gpio_pdata = NULL;

	if (dev->platform_data) {
		struct max9x_pdata *pdata = dev->platform_data;
		gpio_pdata = &pdata->gpio;
	}

	// Functions
	if (gpio_pdata && gpio_pdata->label)
		common->gpio_chip.label = gpio_pdata->label;
	else
		common->gpio_chip.label = dev_name(dev);

	dev_dbg(dev, "gpio_chip label is %s, dev_name is %s",
		common->gpio_chip.label, dev_name(dev));

	common->gpio_chip.parent = dev;
	common->gpio_chip.get_direction = max9295_gpio_get_direction;
	common->gpio_chip.direction_input = max9295_gpio_direction_input;
	common->gpio_chip.direction_output = max9295_gpio_direction_output;
	common->gpio_chip.get = max9295_gpio_get;
	common->gpio_chip.set = max9295_gpio_set;
	common->gpio_chip.set_config = max9295_gpio_set_config;
	common->gpio_chip.ngpio = MAX9295_NUM_GPIO;
	common->gpio_chip.can_sleep = 1;
	common->gpio_chip.base = -1;
	if (gpio_pdata && gpio_pdata->names)
		common->gpio_chip.names = gpio_pdata->names;
	else
		common->gpio_chip.names = max9295_gpio_chip_names;

	ret = devm_gpiochip_add_data(dev, &common->gpio_chip, common);
	if (ret < 0) {
		dev_err(dev, "gpio_init: Failed to add max9295_gpio\n");
		return ret;
	}

	return 0;
}

static int max9295_set_pipe_csi_enabled(struct max9x_common *common,
					unsigned int pipe_id, unsigned int csi_id, bool enable)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	int ret;

	dev_dbg(dev, "Video-pipe %d, csi %d: %s, %d lanes", pipe_id, csi_id,
		(enable ? "enable" : "disable"), common->csi_link[csi_id].config.num_lanes);

	// Select number of lanes for CSI port csi_id
	TRY(ret, regmap_update_bits(map, MAX9295_MIPI_RX_1,
			MAX9295_MIPI_RX_1_SEL_CSI_LANES_FIELD(csi_id),
			MAX9X_FIELD_PREP(MAX9295_MIPI_RX_1_SEL_CSI_LANES_FIELD(csi_id),
			common->csi_link[csi_id].config.num_lanes - 1))
	);

	// Select CSI port csi_id for video pipe pipe_id
	// Enable CSI port csi_id (9295A only has port B, 9295B has both ports)
	TRY(ret, regmap_update_bits(map, MAX9295_FRONTTOP_0,
			MAX9295_FRONTTOP_0_SEL_CSI_FIELD(pipe_id) | MAX9295_FRONTTOP_0_START_CSI_FIELD(csi_id),
			MAX9X_FIELD_PREP(MAX9295_FRONTTOP_0_SEL_CSI_FIELD(pipe_id), csi_id) |
			MAX9X_FIELD_PREP(MAX9295_FRONTTOP_0_START_CSI_FIELD(csi_id), enable ? 1U : 0U))
	);

	// Start video pipe pipe_id from CSI port csi_id
	TRY(ret, regmap_update_bits(map, MAX9295_FRONTTOP_9,
			MAX9295_FRONTTOP_9_START_VIDEO_FIELD(pipe_id, csi_id),
			MAX9X_FIELD_PREP(MAX9295_FRONTTOP_9_START_VIDEO_FIELD(pipe_id, csi_id), enable ? 1U : 0U))
	);

	// Enable video transmit for pipe
	TRY(ret, regmap_update_bits(map, MAX9295_REG2,
			MAX9295_REG2_VID_TX_EN_FIELD(pipe_id),
			MAX9X_FIELD_PREP(MAX9295_REG2_VID_TX_EN_FIELD(pipe_id), enable ? 1U : 0U))
	);

	return 0;
}

static int max9295_set_pipe_data_types_enabled(struct max9x_common *common,
					       unsigned int pipe_id, bool enable)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	int data_type_slot, dt;
	int ret = 0;

	for (data_type_slot = 0; data_type_slot < common->video_pipe[pipe_id].config.num_data_types; data_type_slot++) {
		dt = common->video_pipe[pipe_id].config.data_type[data_type_slot];
		dev_dbg(dev, "Video-pipe %d, data type %d: (%#.2x: %s)",
			pipe_id, data_type_slot, dt, (enable ? "enable" : "disable"));

		TRY(ret, regmap_update_bits_retry(map, MAX9295_MEM_DT_SEL(pipe_id, data_type_slot),
				MAX9295_MEM_DT_SEL_DT_FIELD | MAX9295_MEM_DT_SEL_EN_FIELD,
				MAX9X_FIELD_PREP(MAX9295_MEM_DT_SEL_DT_FIELD, dt) |
				MAX9X_FIELD_PREP(MAX9295_MEM_DT_SEL_EN_FIELD, enable ? 1U : 0U))
		);
	}

	return 0;
}

/**
 * max9295_set_video_pipe_double_loading() - Configure Double Loading Mode on a video pipe
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
static int max9295_set_video_pipe_double_loading(struct max9x_common *common,
						 unsigned int pipe_id, unsigned int bpp)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	struct max9x_serdes_pipe_config *config = &common->video_pipe[pipe_id].config;
	unsigned int reg;
	unsigned int fields;
	int ret;

	if (pipe_id >= MAX9295_NUM_VIDEO_PIPES)
		return -EINVAL;

	// Reset double loading registers to defaults
	TRY(ret, regmap_update_bits(map, MAX9295_FRONTTOP_10,
				    MAX9295_FRONTTOP_10_DBL8_FIELD(pipe_id),
				    0x0));

	TRY(ret, regmap_update_bits(map, MAX9295_FRONTTOP_11,
				    MAX9295_FRONTTOP_11_DBL10_FIELD(pipe_id) |
				    MAX9295_FRONTTOP_11_DBL12_FIELD(pipe_id),
				    0x0));

	// Enable double pixel mode for pipe
	switch (bpp) {
	case 0:
		//bpp not used on this pipe, but still valid input
		break;
	case 8:
		reg = MAX9295_FRONTTOP_10;
		fields = MAX9295_FRONTTOP_10_DBL8_FIELD(pipe_id);
		break;
	case 10:
		reg = MAX9295_FRONTTOP_11;
		fields = MAX9295_FRONTTOP_11_DBL10_FIELD(pipe_id);
		break;
	case 12:
		reg = MAX9295_FRONTTOP_11;
		fields = MAX9295_FRONTTOP_11_DBL12_FIELD(pipe_id);
		break;
	default:
		dev_err(dev, "Unsupported BPP for pixel doubling: %u", bpp);
		return -EINVAL;
	}

	// Enable pixel doubling for specified pipe
	if (bpp != 0) {
		dev_info(dev, "Configuring double loading mode for pipe %u (%ubpp -> %ubpp)",
			 pipe_id, bpp, (bpp * 2));
		TRY(ret, regmap_update_bits(map, reg, fields, 0xFF));
	}

	// We share MAX9295_SOFT_BPP_FIELD/MAX9295_SOFT_BPP_EN_FIELD with
	// max9295_set_video_pipe_pixel_padding(), only write to it if zero
	// padding is disabled and double loading mode is enabled. Zero padding
	// takes precedence and handles the 'both are disabled' case.
	if (config->soft_min_pixel_bpp == 0 && config->soft_max_pixel_bpp == 0) {
		// Override output bpp
		TRY(ret, regmap_update_bits(map, MAX9295_SOFT_BPP(pipe_id),
				MAX9295_SOFT_BPP_EN_FIELD | MAX9295_SOFT_BPP_FIELD,
				MAX9X_FIELD_PREP(MAX9295_SOFT_BPP_EN_FIELD, bpp == 0 ? 0U : 1U) |
				MAX9X_FIELD_PREP(MAX9295_SOFT_BPP_FIELD, bpp * 2))
		);
	}

	return 0;
}

/**
 * max9295_set_video_pipe_pixel_padding() - Configure zero padding on a video pipe
 * @common: max9x_common
 * @pipe_id: Target pipe's ID
 * @min_bpp: Smallest BPP value of data in the pipe. Must be >= 8.
 * @max_bpp: Largest BPP value of data in the pipe. Must be <= 16.
 *
 * Normally, video pipes can only transmit data with the same BPP value. Zero
 * padding is a method to allow data with multiple BPP values to be transmitted
 * over the same video pipe by padding the smaller BPP data to be the same BPP
 * as the largest BPP data. The deserializer will automatically recover the
 * data's original BPP based on datatype information transmitted alongside the
 * padded data.
 *
 * See: GMSL2 Customers User Guide Section 43.3.4.5.2 "Zero Padding"
 */
static int max9295_set_video_pipe_pixel_padding(struct max9x_common *common,
						unsigned int pipe_id, unsigned int min_bpp, unsigned int max_bpp)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	struct max9x_serdes_pipe_config *config = &common->video_pipe[pipe_id].config;
	int ret;
	bool enable = (min_bpp != 0) && (max_bpp != 0);

	if (enable)
		dev_dbg(dev, "Configuring zero padding for pipe %u (%u <= bpp <= %u)", pipe_id, min_bpp, max_bpp);
	else
		dev_dbg(dev, "%s: pipe %u, min_bpp: %u, max_bpp: %u not enabled", __func__, pipe_id, min_bpp, max_bpp);

	if (pipe_id >= MAX9295_NUM_VIDEO_PIPES)
		return -EINVAL;

	/* Auto bpp should be disabled to override bpp */
	TRY(ret, regmap_update_bits(map, MAX9295_VIDEO_TX0(pipe_id),
			MAX9295_VIDEO_TX0_AUTO_BPP_EN_FIELD,
			MAX9X_FIELD_PREP(MAX9295_VIDEO_TX0_AUTO_BPP_EN_FIELD, enable ? 0U : 1U))
	);

	if (enable)
		TRY(ret, regmap_update_bits(map, MAX9295_VIDEO_TX1(pipe_id),
				MAX9295_VIDEO_TX1_BPP_FIELD,
				MAX9X_FIELD_PREP(MAX9295_VIDEO_TX1_BPP_FIELD, max_bpp))
		);

	// We share MAX9295_SOFT_BPP_FIELD/MAX9295_SOFT_BPP_EN_FIELD with
	// max9295_set_video_pipe_double_loading(), only write to it if zero
	// padding is enabled (this function takes precedence) or if both zero
	// pading is disabled and double loading is disabled.
	if (enable || (!enable && config->dbl_pixel_bpp == 0)) {
		TRY(ret, regmap_update_bits(map, MAX9295_SOFT_BPP(pipe_id),
				MAX9295_SOFT_BPP_EN_FIELD | MAX9295_SOFT_BPP_FIELD,
				MAX9X_FIELD_PREP(MAX9295_SOFT_BPP_EN_FIELD, enable ? 1U : 0U) |
				MAX9X_FIELD_PREP(MAX9295_SOFT_BPP_FIELD, min_bpp))
		);
	}

	return 0;
}

static int max9295_max_elements(struct max9x_common *common, enum max9x_element_type element)
{
	switch (element) {
	case MAX9X_SERIAL_LINK:
		return MAX9295_NUM_SERIAL_LINKS;
	case MAX9X_VIDEO_PIPE:
		return MAX9295_NUM_VIDEO_PIPES;
	case MAX9X_MIPI_MAP:
		return MAX9295_NUM_MIPI_MAPS;
	case MAX9X_CSI_LINK:
		return MAX9295_NUM_CSI_LINKS;
	case MAX9X_DATA_TYPES:
		return MAX9295_NUM_DATA_TYPES;
	default:
		break;
	}

	return 0;
}

static int max9295_enable_serial_link(struct max9x_common *common, unsigned int link_id)
{
	struct device *dev = common->dev;
	unsigned int pipe_id;
	int ret;

	dev_dbg(dev, "Link %d: Enable", link_id);

	for (pipe_id = 0; pipe_id < common->num_video_pipes; pipe_id++) {
		struct max9x_serdes_pipe_config *config;

		if (common->video_pipe[pipe_id].enabled == false)
			continue;

		config = &common->video_pipe[pipe_id].config;

		TRY(ret, max9295_set_pipe_data_types_enabled(common, pipe_id, true));

		TRY(ret, max9295_set_video_pipe_double_loading(common, pipe_id, config->dbl_pixel_bpp));

		TRY(ret, max9295_set_video_pipe_pixel_padding(common, pipe_id,
					config->soft_min_pixel_bpp, config->soft_max_pixel_bpp));

		TRY(ret, max9295_set_pipe_csi_enabled(common, pipe_id, config->src_csi, true));
	}

	return 0;
}

static int max9295_disable_serial_link(struct max9x_common *common, unsigned int link_id)
{
	struct device *dev = common->dev;
	unsigned int pipe_id;
	int ret;

	dev_dbg(dev, "Link %d: Disable", link_id);

	for (pipe_id = 0; pipe_id < common->num_video_pipes; pipe_id++) {
		struct max9x_serdes_pipe_config *config;

		if (common->video_pipe[pipe_id].enabled == false)
			continue;

		config = &common->video_pipe[pipe_id].config;

		TRY(ret, max9295_set_pipe_data_types_enabled(common, pipe_id, false));

		TRY(ret, max9295_set_pipe_csi_enabled(common, pipe_id, config->src_csi, false));
	}

	return 0;
}

/*
 * Enable RCLK (27MHz) output on MFP4 pin. This pin is routed on some imager boards
 * to the imager instead of an on-board oscillator.
 */
static int max9295_enable_rclk(struct max9x_common *common)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	int ret;

	if (!common->external_refclk_enable) {
		dev_dbg(dev, "Enable RCLK: external-refclk-enable not present, not enabling external refclk");
		return 0;
	}

	dev_info(dev, "Enable RCLK: 27MHz");

	// Configure pre-defined 27MHz frequency (0b01 = 27MHz)
	TRY(ret, regmap_update_bits(map, MAX9295_REF_VTG0,
			MAX9295_REFGEN_PREDEF_FREQ_FIELD,
			MAX9X_FIELD_PREP(MAX9295_REFGEN_PREDEF_FREQ_FIELD, 1U))
	);

	// Enable reference generation
	TRY(ret, regmap_update_bits(map, MAX9295_REF_VTG0,
			MAX9295_REFGEN_EN_FIELD,
			MAX9X_FIELD_PREP(MAX9295_REFGEN_EN_FIELD, 1U))
	);

	// Configure reference generation output on MFP4
	TRY(ret, regmap_update_bits_retry(map, MAX9295_REF_VTG1,
			MAX9295_PCLK_GPIO_FIELD,
			MAX9X_FIELD_PREP(MAX9295_PCLK_GPIO_FIELD, 4U))
	);

	// Enable output
	TRY(ret, regmap_update_bits_retry(map, MAX9295_REF_VTG1,
			MAX9295_PCLK_EN_FIELD,
			MAX9X_FIELD_PREP(MAX9295_PCLK_EN_FIELD, 1U))
	);

	TRY(ret, regmap_update_bits(map, MAX9295_REG3,
			MAX9295_RCLK_SEL_FIELD,
			MAX9X_FIELD_PREP(MAX9295_RCLK_SEL_FIELD, 3U))
	);

	TRY(ret, regmap_update_bits(map, MAX9295_REG6,
			MAX9295_RCLK_EN_FIELD,
			MAX9X_FIELD_PREP(MAX9295_RCLK_EN_FIELD, 1U))
	);

	return 0;
}

static int max9295_set_local_control_channel_enabled(struct max9x_common *common, bool enabled)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;

	dev_dbg(dev, "set rem cc %s", (enabled ? "enable" : "disable"));

	return regmap_update_bits_retry(map, MAX9295_PHY_REM_CTRL, MAX9295_PHY_LOCAL_CTRL_DIS_FIELD,
				  MAX9X_FIELD_PREP(MAX9295_PHY_LOCAL_CTRL_DIS_FIELD,  (enabled ? 0U : 1U)));
}

static int max9295_select_serial_link(struct max9x_common *common, unsigned int link)
{
	return max9295_set_local_control_channel_enabled(common, true);
}

static int max9295_deselect_serial_link(struct max9x_common *common, unsigned int link)
{
	return max9295_set_local_control_channel_enabled(common, false);
}

static int max9295_verify_devid(struct max9x_common *common)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	unsigned int dev_id, dev_rev;
	int ret;

	// Fetch and output chip name + revision
	TRY(ret, regmap_read_retry(map, MAX9295_DEV_ID, &dev_id));
	TRY(ret, regmap_read_retry(map, MAX9295_DEV_REV, &dev_rev));

	switch (dev_id) {
	case MAX9295A:
		dev_info(dev, "Detected MAX9295A revision %ld",
			 FIELD_GET(MAX9295_DEV_REV_FIELD, dev_rev));
		break;
	case MAX9295B:
		dev_info(dev, "Detected MAX9295B revision %ld",
			 FIELD_GET(MAX9295_DEV_REV_FIELD, dev_rev));
		break;
	case MAX9295E:
		dev_info(dev, "Detected MAX9295E revision %ld",
			 FIELD_GET(MAX9295_DEV_REV_FIELD, dev_rev));
		break;
	default:
		dev_err(dev, "Unknown device ID %d revision %ld", dev_id,
			FIELD_GET(MAX9295_DEV_REV_FIELD, dev_rev));
		return -EINVAL;
	}

	return 0;
}

static int max9295_enable(struct max9x_common *common)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	int ret;

	TRY(ret, max9295_verify_devid(common));

	// Turn on RCLK/PCLK
	ret = max9295_enable_rclk(common);
	if (ret)
		dev_warn(dev, "Failed to enable RCLK output");

	// Initialize local CC to off
	TRY(ret, max9295_set_local_control_channel_enabled(common, false));

	/* Clear the pipe maps */
	TRY(ret, regmap_write_retry(map, MAX9295_FRONTTOP_9, 0));

	/* Clear the csi port selections */
	TRY(ret, regmap_write_retry(map, MAX9295_FRONTTOP_0, MAX9295_FRONTTOP_0_LINE_INFO));

	return 0;
}

static int max9295_disable(struct max9x_common *common)
{
	struct device *dev = common->dev;

	dev_dbg(dev, "Disable");

	return 0;
}

static int max9295_remap_addr(struct max9x_common *common)
{
	int ret;
	struct device *dev = common->dev;
	const unsigned int RETRY_MS_MIN = 32;
	const unsigned int RETRY_MS_MAX = 512;
	unsigned int ms;

	if (!common->phys_client)
		return 0;

	if (!common->phys_map)
		return 0;

	dev_info(dev, "Remap address from 0x%02x to 0x%02x",
		 common->phys_client->addr, common->client->addr);

	TRY(ret, regmap_update_bits(common->phys_map, MAX9295_REG0,
			MAX9295_REG0_DEV_ADDR_FIELD,
			FIELD_PREP(MAX9295_REG0_DEV_ADDR_FIELD, common->client->addr))
	);

	/*
	 * Use lower bits of I2C address as unique TX_SRC_ID to prevent
	 * conflicts for info frames, SPI, etc. (Leave video pipes alone)
	 */
	TRY_DEV_HERE(ret, regmap_update_bits(common->map, MAX9295_CFGI_INFOFR_TR3, MAX9295_TR3_TX_SRC_ID,
		MAX9X_FIELD_PREP(MAX9295_TR3_TX_SRC_ID, common->client->addr)), dev);
	TRY_DEV_HERE(ret, regmap_update_bits(common->map, MAX9295_CFGL_SPI_TR3, MAX9295_TR3_TX_SRC_ID,
		MAX9X_FIELD_PREP(MAX9295_TR3_TX_SRC_ID, common->client->addr)), dev);
	/*
	 * This exponential retry works around for MAX96724X.
	 */
	for (ms = RETRY_MS_MIN; ms <= RETRY_MS_MAX; ms <<= 1) {
		ret = regmap_update_bits(common->map, MAX9295_CFGC_CC_TR3, MAX9295_TR3_TX_SRC_ID,
			MAX9X_FIELD_PREP(MAX9295_TR3_TX_SRC_ID, common->client->addr));
		if (ret) {
			dev_warn(dev,
				"configure MAX9295_CFGC_CC_TR3 failed, trying again (waiting %d ms)", ms);
			msleep(ms);
		} else
			break;
	}

	TRY_DEV_HERE(ret, regmap_update_bits(common->map, MAX9295_CFGL_GPIO_TR3, MAX9295_TR3_TX_SRC_ID,
		MAX9X_FIELD_PREP(MAX9295_TR3_TX_SRC_ID, common->client->addr)), dev);
	TRY_DEV_HERE(ret, regmap_update_bits(common->map, MAX9295_CFGL_IIC_X, MAX9295_TR3_TX_SRC_ID,
		MAX9X_FIELD_PREP(MAX9295_TR3_TX_SRC_ID, common->client->addr)), dev);
	TRY_DEV_HERE(ret, regmap_update_bits(common->map, MAX9295_CFGL_IIC_Y, MAX9295_TR3_TX_SRC_ID,
		MAX9X_FIELD_PREP(MAX9295_TR3_TX_SRC_ID, common->client->addr)), dev);

	return 0;
}

static int max9295_remap_reset(struct max9x_common *common)
{
	int ret;
	struct device *dev = common->dev;
	struct max9x_pdata *pdata = dev->platform_data;
	u32 phys_addr = pdata->phys_addr ? pdata->phys_addr :
			common->client->addr;
	u32 virt_addr = common->client->addr;

	dev_info(dev, "Remap reset address from 0x%02x to 0x%02x", virt_addr,
		 phys_addr);

	TRY(ret, regmap_update_bits(common->map, MAX9295_REG0, MAX9295_REG0_DEV_ADDR_FIELD,
				    FIELD_PREP(MAX9295_REG0_DEV_ADDR_FIELD, phys_addr)));

	return 0;
}

static int max9295_add_translate_addr(struct max9x_common *common,
				      unsigned int i2c_id, unsigned int virt_addr, unsigned int phys_addr)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	unsigned int alias;
	unsigned int src;
	int ret;

	for (alias = 0; alias < MAX9295_NUM_ALIASES; alias++) {
		TRY(ret, regmap_read_retry(map, MAX9295_I2C_SRC(i2c_id, alias), &src));

		src = FIELD_GET(MAX9295_I2C_SRC_FIELD, src);
		if (src == virt_addr || src == 0) {
			dev_dbg(dev, "SRC %02x = %02x, DST %02x = %02x",
				MAX9295_I2C_SRC(i2c_id, alias), virt_addr,
				MAX9295_I2C_DST(i2c_id, alias), phys_addr);
			TRY(ret, regmap_write_retry(map, MAX9295_I2C_DST(i2c_id, alias),
					      MAX9X_FIELD_PREP(MAX9295_I2C_DST_FIELD, phys_addr))
			);

			TRY(ret, regmap_write_retry(map, MAX9295_I2C_SRC(i2c_id, alias),
					      MAX9X_FIELD_PREP(MAX9295_I2C_SRC_FIELD, virt_addr))
			);
		}
	}

	return 0;
}

static int max9295_remove_translate_addr(struct max9x_common *common,
					 unsigned int i2c_id, unsigned int virt_addr, unsigned int phys_addr)
{
	struct regmap *map = common->map;
	unsigned int alias;
	unsigned int src;
	int ret;

	for (alias = 0; alias < MAX9295_NUM_ALIASES; alias++) {
		TRY(ret, regmap_read_retry(map, MAX9295_I2C_SRC(i2c_id, alias), &src));
		src = FIELD_GET(MAX9295_I2C_SRC_FIELD, src);
		if (src == virt_addr) {
			return regmap_write_retry(map, MAX9295_I2C_DST(i2c_id, alias),
					    MAX9X_FIELD_PREP(MAX9295_I2C_DST_FIELD, 0));
		}
	}

	return 0;
}

static int max9295_reset(struct max9x_common *common)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	int ret;

	dev_dbg(dev, "Reset");

	/* Reset entire chip by CTRL0_RST_ALL: 0x10[7]*/
	TRY(ret, regmap_write(map, MAX9295_CTRL0, MAX9295_CTRL0_RST_ALL));
	usleep_range(45000, 45050);

	return 0;
}

int max9295_get_ops(struct max9x_common_ops **common_ops, struct max9x_serial_link_ops **serial_ops,
		    struct max9x_csi_link_ops **csi_ops, struct max9x_line_fault_ops **lf_ops,
			struct max9x_translation_ops **trans_ops)
{
	(*common_ops) = &max9295_common_ops;
	(*serial_ops) = &max9295_serial_link_ops;
	(*csi_ops) = NULL;
	(*lf_ops) = NULL;
	(*trans_ops) = &max9295_translation_ops;
	return 0;
}

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Josh Watts <jwatts@d3embedded.com>");
MODULE_AUTHOR("Yan, Dongcheng <dongcheng.yan@intel.com>");
MODULE_DESCRIPTION("Maxim MAX9295 CSI-2/parallel to GMSL2 Serializer driver");
