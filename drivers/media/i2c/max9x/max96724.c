/*
 * max96724.c - Maxim MAX96724 GMSL2/GMSL1 to CSI-2 Deserializer
 *
 * Copyright (c) 2023-2024, Define Design Deploy Corp. All rights reserved.
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
// Copyright (c) 2025-2026 Intel Corporation.

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

#include "max96724.h"
#include "regmap-retry.h"

// Params
int max96724_serial_link_timeout_ms = MAX96724_DEFAULT_SERIAL_LINK_TIMEOUT_MS;
module_param(max96724_serial_link_timeout_ms, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(max96724_serial_link_timeout_ms, "Timeout for serial link in milliseconds");

// Declarations
static int max96724_set_phy_mode(struct max9x_common *common, unsigned int phy_mode, unsigned int phy_clk);
static int max96724_set_phy_enabled(struct max9x_common *common, unsigned int csi_id, bool enable);
static int max96724_set_phy_lane_map(struct max9x_common *common, unsigned int csi_id, unsigned int phy_lane_map);
static int max96724_set_phy_dpll_freq(struct max9x_common *common, unsigned int csi_id, unsigned int freq_mhz);
static int max96724_set_mipi_lane_cnt(struct max9x_common *common, unsigned int csi_id, int num_lanes);
static int max96724_set_initial_deskew(struct max9x_common *common, unsigned int csi_id, bool enable);
static int max96724_configure_csi_dphy(struct max9x_common *common);
static int max96724_enable(struct max9x_common *common);
static int max96724_max_elements(struct max9x_common *common, enum max9x_element_type element);
static int max96724_get_serial_link_lock(struct max9x_common *common, unsigned int link_id, bool *locked);
static int max96724_set_serial_link_state(struct max9x_common *common, unsigned int link_id, bool enable);
static int max96724_serial_link_reset(struct max9x_common *common, unsigned int link_id);
static int max96724_set_serial_link_rate(struct max9x_common *common, unsigned int link_id);
static int max96724_set_video_pipe_src(struct max9x_common *common, unsigned int pipe_id,
				       unsigned int link_id, unsigned int src_pipe);
static int max96724_set_video_pipe_maps_enabled(struct max9x_common *common, unsigned int pipe_id, int num_maps);
static int max96724_set_video_pipe_map(struct max9x_common *common, unsigned int pipe_id, unsigned int map_id,
				       struct max9x_serdes_mipi_map *mipi_map);
static int max96724_set_csi_link_enabled(struct max9x_common *common, unsigned int csi_id, bool enable);
static int max96724_csi_double_pixel(struct max9x_common *common, unsigned int csi_id, unsigned int bpp);
static int max96724_set_video_pipe_enabled(struct max9x_common *common, unsigned int pipe_id, bool enable);
static int max96724_set_serial_link_routing(struct max9x_common *common, unsigned int link_id);
static int max96724_disable_serial_link(struct max9x_common *common, unsigned int link_id);
static int max96724_enable_serial_link(struct max9x_common *common, unsigned int link_id);
static int max96724_set_remote_control_channel_enabled(struct max9x_common *common, unsigned int link_id, bool enabled);
static int max96724_select_serial_link(struct max9x_common *common, unsigned int link);
static int max96724_deselect_serial_link(struct max9x_common *common, unsigned int link);
static int max96724_disable_line_fault(struct max9x_common *common, unsigned int line);
static int max96724_enable_line_fault(struct max9x_common *common, unsigned int line);
static int max96724_set_line_fault(struct max9x_common *common, unsigned int line, bool enable);
static int max96724_get_line_fault(struct max9x_common *common, unsigned int line);

// Functions
static int max96724_set_phy_mode(struct max9x_common *common, unsigned int phy_mode, unsigned int phy_clk)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;

	dev_dbg(dev, "CSI: phy_mode=%d, phy_clk=0x%02x", phy_mode, phy_clk);

	return regmap_update_bits(map, MAX96724_MIPI_PHY0,
		MAX96724_MIPI_PHY0_MODE_FIELD |
		MAX96724_MIPI_PHY0_CLK_PHY0 | MAX96724_MIPI_PHY0_CLK_PHY3,
		MAX9X_FIELD_PREP(MAX96724_MIPI_PHY0_MODE_FIELD, phy_mode) |
		(phy_clk & (MAX96724_MIPI_PHY0_CLK_PHY0 | MAX96724_MIPI_PHY0_CLK_PHY3)));
}

static int max96724_set_phy_enabled(struct max9x_common *common,
				    unsigned int csi_id, bool enable)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	int ret;
	struct mutex *lock = &common->link_mutex;

	mutex_lock(lock);
	dev_dbg(dev, "CSI link %d: %s", csi_id, (enable ? "enable" : "disable"));

	ret = regmap_update_bits(map, MAX96724_MIPI_PHY_ENABLE,
		MAX96724_MIPI_PHY_ENABLE_FIELD(csi_id),
		MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_ENABLE_FIELD(csi_id), enable ? 1U : 0U));
	mutex_unlock(lock);
	return ret;
}

static int max96724_set_phy_lane_map(struct max9x_common *common,
				     unsigned int csi_id, unsigned int phy_lane_map)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;

	dev_dbg(dev, "CSI link %d: phy_lane_map=0x%04x", csi_id, phy_lane_map);

	return regmap_update_bits(map, MAX96724_MIPI_PHY_LANE_MAP(csi_id),
		MAX96724_MIPI_PHY_LANE_MAP_FIELD(csi_id, 0)
		| MAX96724_MIPI_PHY_LANE_MAP_FIELD(csi_id, 1),
		phy_lane_map);
}

static int max96724_set_phy_dpll_freq(struct max9x_common *common,
				      unsigned int csi_id, unsigned int freq_mhz)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;

	dev_dbg(dev, "CSI link %d: freq %u MHz, %u mult", csi_id, freq_mhz,
		freq_mhz / MAX96724_DPLL_FREQ_MHZ_MULTIPLE);

	return regmap_update_bits(map, MAX96724_DPLL_FREQ(csi_id),
		MAX96724_DPLL_FREQ_FIELD,
		MAX9X_FIELD_PREP(MAX96724_DPLL_FREQ_FIELD, freq_mhz / MAX96724_DPLL_FREQ_MHZ_MULTIPLE));
}

static int max96724_set_initial_deskew(struct max9x_common *common, unsigned int csi_id, bool enable)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	unsigned int val;
	unsigned int width;

	dev_dbg(dev, "CSI link %d: Initial deskew %s", csi_id, enable ? "enabled" : "disabled");

	val = MAX9X_FIELD_PREP(MAX96724_MIPI_TX_DESKEW_INIT_AUTO_EN, enable);

	if (enable) {
		width = common->csi_link[csi_id].config.initial_deskew_width;
		dev_dbg(dev, "Initial deskew width: 0x%03x", width);

		if (width > MAX96724_INIT_DESKEW_WIDTH_MAX) {
			dev_err(dev, "Unsupported initial deskew width!");
			return -EINVAL;
		}

		val |= MAX9X_FIELD_PREP(MAX96724_MIPI_TX_DESKEW_WIDTH_FIELD, width);
	}

	return regmap_write(map, MAX96724_MIPI_TX_DESKEW_INIT(csi_id), val);
}

static int max96724_set_periodic_deskew(struct max9x_common *common, unsigned int csi_id, bool enable, unsigned int width, unsigned int interval)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	unsigned int val;

	val = MAX9X_FIELD_PREP(MAX96724_MIPI_TX_DESKEW_PERIODIC_EN, enable);

	if (enable) {
		if (width > MAX96724_PERIODIC_DESKEW_WIDTH_MAX) {
			dev_err(dev, "Unsupported periodic deskew width!");
			return -EINVAL;
		}
		val |= MAX9X_FIELD_PREP(MAX96724_MIPI_TX_DESKEW_PERIODIC_WIDTH_FIELD, width);

		if (interval > MAX96724_PERIODIC_DESKEW_MAXFRAME) {
			dev_err(dev, "Unsupported periodic deskew interval!");
			return -EINVAL;
		}
		val |= MAX9X_FIELD_PREP(MAX96724_MIPI_TX_DESKEW_PERIODIC_INTERVAL_FIELD, interval);

		dev_dbg(dev, "CSI link %d: Periodic deskew width: 0x%x, interval: 0x%x", csi_id, width, interval);
	}

	return regmap_write(map, MAX96724_MIPI_TX_DESKEW_PERIODIC(csi_id), val);
}

static int max96724_set_mipi_lane_cnt(struct max9x_common *common,
				      unsigned int csi_id, int num_lanes)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	int ret;

	dev_dbg(dev, "CSI link %d: %d lanes", csi_id, num_lanes);

	ret = regmap_update_bits(map, MAX96724_MIPI_TX_LANE_CNT(csi_id),
		MAX96724_MIPI_TX_VCX_EN_FIELD,
		MAX9X_FIELD_PREP(MAX96724_MIPI_TX_VCX_EN_FIELD, MAX96724_VC_2_BITS));

	if (ret) {
		dev_err(dev, "Failed to configure virtual channel extension!");
		return ret;
	}

	ret = regmap_update_bits(
		map, MAX96724_MIPI_TX_LANE_CNT(csi_id),
		MAX96724_MIPI_TX_CPHY_EN_FIELD,
		MAX9X_FIELD_PREP(MAX96724_MIPI_TX_CPHY_EN_FIELD,
				 common->csi_link[csi_id].config.bus_type ==
						 V4L2_MBUS_CSI2_CPHY ?
					 1 :
					 0));

	if (ret) {
		dev_err(dev, "Failed to enable CSI2 %d to CPHY mode!", csi_id);
		return ret;
	}

	if (num_lanes > 0) {
		ret = regmap_update_bits(
			map, MAX96724_MIPI_TX_LANE_CNT(csi_id),
			MAX96724_MIPI_TX_LANE_CNT_FIELD,
			MAX9X_FIELD_PREP(MAX96724_MIPI_TX_LANE_CNT_FIELD,
					 (num_lanes - 1)));

		if (ret) {
			dev_err(dev,
				" Failed to set CSI2 controller %d with %d lanes !",
				csi_id, num_lanes);
			return ret;
		}
	}

	return ret;
}

static int max96724_configure_csi_dphy(struct max9x_common *common)
{
	struct device *dev = common->dev;
	unsigned int phy_mode;
	unsigned int phy_lane_map[MAX96724_NUM_CSI_LINKS];
	unsigned int phy_clk = 0;
	unsigned int csi_id;
	int ret;

	for (csi_id = 0; csi_id < MAX96724_NUM_CSI_LINKS; csi_id++) {
		dev_dbg(dev, "CSI link %d: enabled=%d, num_lanes=%d, freq_mhz=%d, init_deskew=%d",
			csi_id,
			common->csi_link[csi_id].enabled,
			common->csi_link[csi_id].config.num_lanes,
			common->csi_link[csi_id].config.freq_mhz,
			common->csi_link[csi_id].config.auto_init_deskew_enabled);
	}

	//TODO: Allow DT to override lane mapping?

	// Determine correct phy_mode and associate lane mapping
	if (common->csi_link[0].config.num_lanes <= 2
	    && common->csi_link[1].config.num_lanes <= 2
	    && common->csi_link[2].config.num_lanes <= 2
	    && common->csi_link[3].config.num_lanes <= 2) {

		phy_mode = MAX96724_MIPI_PHY_4X2;

		phy_lane_map[0] = MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(0, 0), 0)
				| MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(0, 1), 1);
		phy_lane_map[1] = MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(1, 0), 0)
				| MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(1, 1), 1);
		phy_lane_map[2] = MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(2, 0), 0)
				| MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(2, 1), 1);
		phy_lane_map[3] = MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(3, 0), 0)
				| MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(3, 1), 1);

	} else if (common->csi_link[0].config.num_lanes == 0
		   && common->csi_link[1].config.num_lanes >= 3
		   && common->csi_link[2].config.num_lanes >= 3
		   && common->csi_link[3].config.num_lanes == 0) {

		phy_mode = MAX96724_MIPI_PHY_2X4;

		phy_lane_map[0] = MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(0, 0), 0)
				| MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(0, 1), 1);
		phy_lane_map[1] = MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(1, 0), 2)
				| MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(1, 1), 3);
		phy_lane_map[2] = MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(2, 0), 0)
				| MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(2, 1), 1);
		phy_lane_map[3] = MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(3, 0), 2)
				| MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(3, 1), 3);

	} else if (common->csi_link[0].config.num_lanes == 0
		   && common->csi_link[1].config.num_lanes >= 3
		   && common->csi_link[2].config.num_lanes <= 2
		   && common->csi_link[3].config.num_lanes <= 2) {

		phy_mode = MAX96724_MIPI_PHY_1X4A_22;

		phy_lane_map[0] = MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(0, 0), 0)
				| MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(0, 1), 1);
		phy_lane_map[1] = MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(1, 0), 2)
				| MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(1, 1), 3);
		phy_lane_map[2] = MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(2, 0), 0)
				| MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(2, 1), 1);
		phy_lane_map[3] = MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(3, 0), 0)
				| MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(3, 1), 1);

		/* Use CKCP/N alternate CLK on PHY0 */
		phy_clk = MAX96724_MIPI_PHY0_CLK_PHY0;

	} else if (common->csi_link[0].config.num_lanes <= 2
		   && common->csi_link[1].config.num_lanes <= 2
		   && common->csi_link[2].config.num_lanes >= 3
		   && common->csi_link[3].config.num_lanes == 0) {

		phy_mode = MAX96724_MIPI_PHY_1X4B_22;

		phy_lane_map[0] = MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(0, 0), 0)
				| MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(0, 1), 1);
		phy_lane_map[1] = MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(1, 0), 0)
				| MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(1, 1), 1);
		phy_lane_map[2] = MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(2, 0), 0)
				| MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(2, 1), 1);
		phy_lane_map[3] = MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(3, 0), 2)
				| MAX9X_FIELD_PREP(MAX96724_MIPI_PHY_LANE_MAP_FIELD(3, 1), 3);

	} else {
		dev_err(dev, "Invalid CSI configuration! Supported modes: 4x2, 2x4, 1x4+2x2, 2x2+1x4");
		return -EINVAL;
	}

	ret = max96724_set_phy_mode(common, phy_mode, phy_clk);
	if (ret)
		return ret;

	for (csi_id = 0; csi_id < MAX96724_NUM_CSI_LINKS; csi_id++) {
		struct max9x_serdes_csi_config *config = &common->csi_link[csi_id].config;

		ret = max96724_set_phy_enabled(common, csi_id, false);
		if (ret)
			return ret;

		ret = max96724_set_phy_lane_map(common, csi_id, phy_lane_map[csi_id]);
		if (ret)
			return ret;

		ret = max96724_set_mipi_lane_cnt(common, csi_id,
						 common->csi_link[csi_id].config.num_lanes);
		if (ret)
			return ret;

		ret = max96724_set_initial_deskew(common, csi_id,
						  common->csi_link[csi_id].config.auto_init_deskew_enabled);
		if (ret)
			return ret;

		ret = max96724_set_periodic_deskew(
				 common, csi_id,
				 common->csi_link[csi_id]
					 .config.auto_init_deskew_enabled,
				 MAX96724_PERIODIC_DESKEW_8x1KUI,
				 MAX96724_PERIODIC_DESKEW_1FRAME);
		if (ret)
			return ret;

		if (WARN_ONCE(config->freq_mhz > 0 && config->freq_mhz < MAX96724_DPLL_FREQ_MHZ_MULTIPLE,
			      "CSI frequency must be greater than %d MHz", MAX96724_DPLL_FREQ_MHZ_MULTIPLE))
			return -EINVAL;

		ret = max96724_set_phy_dpll_freq(common, csi_id, config->freq_mhz);
		if (ret)
			return ret;
	}

	return 0;
}

static int max96724_set_all_reset(struct max9x_common *common, bool enable)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;

	dev_dbg(dev, "Reset %s", (enable ? "enable" : "disable"));

	return regmap_update_bits_retry(map, MAX96724_RESET_ALL,
		MAX96724_RESET_ALL_FIELD,
		MAX9X_FIELD_PREP(MAX96724_RESET_ALL_FIELD, enable ? 1U : 0U));
}

static int max96724_soft_reset(struct max9x_common *common)
{
	struct device *dev = common->dev;
	int ret;

	dev_dbg(dev, "Soft reset");

	ret = max96724_set_all_reset(common, 1);
	if (ret)
		return ret;

	/* Wait for hardware available after soft reset */
	/* TODO: Optimize sleep time 20 ms */
	msleep(20);

	ret = max96724_set_all_reset(common, 0);
	if (ret)
		return ret;

	return 0;
}

static int max96724_max_elements(struct max9x_common *common, enum max9x_element_type element)
{
	switch (element) {
	case MAX9X_SERIAL_LINK:
		return MAX96724_NUM_SERIAL_LINKS;
	case MAX9X_VIDEO_PIPE:
		return MAX96724_NUM_VIDEO_PIPES;
	case MAX9X_MIPI_MAP:
		return MAX96724_NUM_MIPI_MAPS;
	case MAX9X_CSI_LINK:
		return MAX96724_NUM_CSI_LINKS;
	case MAX9X_LINE_FAULT:
		return MAX96724_NUM_LINE_FAULTS;
	default:
		break;
	}

	return 0;
}

static struct max9x_common_ops max96724_common_ops = {
	.enable = max96724_enable,
	.soft_reset = max96724_soft_reset,
	.max_elements = max96724_max_elements,
};

static int max96724_get_serial_link_lock(struct max9x_common *common, unsigned int link_id, bool *locked)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	unsigned int val;
	int ret;

	ret = regmap_read(map, MAX96724_PHY_LOCKED(link_id), &val);
	if (ret) {
		dev_err(dev, "failed to read link lock with err %d", ret);
		return ret;
	}

	if (FIELD_GET(MAX96724_PHY_LOCKED_FIELD, val) != 0)
		*locked = true;
	else
		*locked = false;

	return 0;
}

static int max96724_set_serial_link_state(struct max9x_common *common, unsigned int link_id, bool enable)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	enum max9x_serdes_link_type link_type = common->serial_link[link_id].config.link_type;
	int ret;
	struct mutex *lock = &common->link_mutex;

	mutex_lock(lock);
	dev_dbg(dev, "Serial-link %d: %s", link_id, (enable ? "up" : "down"));

	ret = regmap_update_bits(map, MAX96724_LINK_CTRL,
		MAX96724_LINK_CTRL_EN_FIELD(link_id)
		| MAX96724_LINK_CTRL_GMSL_FIELD(link_id),
		MAX9X_FIELD_PREP(MAX96724_LINK_CTRL_EN_FIELD(link_id), enable ? 1U : 0U)
		| MAX9X_FIELD_PREP(MAX96724_LINK_CTRL_GMSL_FIELD(link_id), link_type == MAX9X_LINK_TYPE_GMSL2 ? 1U : 0U));
	mutex_unlock(lock);
	return ret;
}

static int max96724_serial_link_reset(struct max9x_common *common, unsigned int link_id)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	int ret;
	struct mutex *lock = &common->link_mutex;

	mutex_lock(lock);
	dev_dbg(dev, "Serial-link %d reset", link_id);

	ret = regmap_update_bits(map, MAX96724_RESET_CTRL,
		MAX96724_RESET_CTRL_FIELD(link_id),
		MAX9X_FIELD_PREP(MAX96724_RESET_CTRL_FIELD(link_id), 1U));
	mutex_unlock(lock);

	return ret;
}

static int max96724_set_serial_link_rate(struct max9x_common *common, unsigned int link_id)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	struct max9x_serdes_serial_config *config = &common->serial_link[link_id].config;
	int tx_rate, rx_rate;

	tx_rate = max9x_serdes_mhz_to_rate(max96724_tx_rates, ARRAY_SIZE(max96724_tx_rates), config->tx_freq_mhz);
	if (tx_rate < 0)
		return tx_rate;

	rx_rate = max9x_serdes_mhz_to_rate(max96724_rx_rates, ARRAY_SIZE(max96724_rx_rates), config->rx_freq_mhz);
	if (rx_rate < 0)
		return rx_rate;

	dev_dbg(dev, "Serial-link %d: TX=%d MHz RX=%d MHz", link_id, config->tx_freq_mhz, config->rx_freq_mhz);

	return regmap_update_bits(map, MAX96724_PHY_RATE_CTRL(link_id),
		MAX96724_PHY_RATE_CTRL_TX_FIELD(link_id)
		| MAX96724_PHY_RATE_CTRL_RX_FIELD(link_id),
		MAX9X_FIELD_PREP(MAX96724_PHY_RATE_CTRL_TX_FIELD(link_id), tx_rate)
		| MAX9X_FIELD_PREP(MAX96724_PHY_RATE_CTRL_RX_FIELD(link_id), rx_rate));
}

static int max96724_set_video_pipe_src(struct max9x_common *common,
				       unsigned int pipe_id, unsigned int link_id, unsigned int src_pipe)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;

	dev_dbg(dev, "Video-pipe %d: src_link=%u, src_pipe=%u", pipe_id, link_id, src_pipe);

	return regmap_update_bits(map, MAX96724_VIDEO_PIPE_SEL(pipe_id),
		MAX96724_VIDEO_PIPE_SEL_LINK_FIELD(pipe_id)
		| MAX96724_VIDEO_PIPE_SEL_INPUT_FIELD(pipe_id),
		MAX9X_FIELD_PREP(MAX96724_VIDEO_PIPE_SEL_LINK_FIELD(pipe_id), link_id)
		| MAX9X_FIELD_PREP(MAX96724_VIDEO_PIPE_SEL_INPUT_FIELD(pipe_id), src_pipe));
}

static int max96724_set_video_pipe_maps_enabled(struct max9x_common *common,
						unsigned int pipe_id, int num_maps)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	unsigned int val = 0;
	int ret = 0;
	struct mutex *lock = &common->link_mutex;

	if (num_maps > 0)
		val = GENMASK(num_maps - 1, 0);

	mutex_lock(lock);
	dev_dbg(dev, "Video-pipe %d: num_maps=%u", pipe_id, num_maps);

	ret = regmap_write(map, MAX96724_MAP_EN_L(pipe_id),
		MAX9X_FIELD_PREP(MAX96724_MAP_EN_FIELD, val));
	if (ret)
		goto unlock_exit;

	ret = regmap_write(map, MAX96724_MAP_EN_H(pipe_id),
		MAX9X_FIELD_PREP(MAX96724_MAP_EN_FIELD, val >> 8));
	if (ret)
		goto unlock_exit;

unlock_exit:
	mutex_unlock(lock);
	return ret;
}

static int max96724_set_video_pipe_map(struct max9x_common *common,
				       unsigned int pipe_id, unsigned int map_id,
				       struct max9x_serdes_mipi_map *mipi_map)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	int ret = 0;
	struct mutex *lock = &common->link_mutex;

	mutex_lock(lock);
	dev_dbg(dev, "Video-pipe %d, map %d: VC%d:DT%02x->VC%d:DT%02x, dst_csi=%d ",
		pipe_id, map_id,
		mipi_map->src_vc,
		mipi_map->src_dt,
		mipi_map->dst_vc,
		mipi_map->dst_dt,
		mipi_map->dst_csi);

	ret = regmap_write(map, MAX96724_MAP_SRC_L(pipe_id, map_id),
		MAX9X_FIELD_PREP(MAX96724_MAP_SRC_L_VC_FIELD, mipi_map->src_vc)
		| MAX9X_FIELD_PREP(MAX96724_MAP_SRC_L_DT_FIELD, mipi_map->src_dt));
	if (ret)
		goto unlock_exit;

	ret = regmap_write(map, MAX96724_MAP_DST_L(pipe_id, map_id),
		MAX9X_FIELD_PREP(MAX96724_MAP_DST_L_VC_FIELD, mipi_map->dst_vc)
		| MAX9X_FIELD_PREP(MAX96724_MAP_DST_L_DT_FIELD, mipi_map->dst_dt));
	if (ret)
		goto unlock_exit;

	ret = regmap_write(map, MAX96724_MAP_SRCDST_H(pipe_id, map_id),
		MAX9X_FIELD_PREP(MAX96724_MAP_SRCDST_H_SRC_VC_FIELD, mipi_map->src_vc)
		| MAX9X_FIELD_PREP(MAX96724_MAP_SRCDST_H_DST_VC_FIELD, mipi_map->dst_vc));
	if (ret)
		goto unlock_exit;

	ret = regmap_update_bits(map, MAX96724_MAP_DPHY_DEST(pipe_id, map_id),
		MAX96724_MAP_DPHY_DEST_FIELD(map_id),
		MAX9X_FIELD_PREP(MAX96724_MAP_DPHY_DEST_FIELD(map_id), mipi_map->dst_csi));
	if (ret)
		goto unlock_exit;

unlock_exit:
	mutex_unlock(lock);
	return ret;
}

static int max96724_set_csi_link_enabled(struct max9x_common *common,
					 unsigned int csi_id, bool enable)
{
	struct device *dev = common->dev;
	struct max9x_serdes_csi_link *csi_link;
	int ret = 0;

	if (csi_id >= common->num_csi_links)
		return -EINVAL;

	csi_link = &common->csi_link[csi_id];

	if (WARN_ONCE(enable && csi_link->enabled == false,
		     "Tried to enable a disabled CSI port???"))
		return -EINVAL;

	if (WARN_ONCE(enable && csi_link->config.num_lanes == 0,
		     "Tried to enable CSI port with no lanes???"))
		return -EINVAL;

	mutex_lock(&csi_link->csi_mutex);

	dev_dbg(dev, "CSI link %d: %s (%d users)", csi_id,
		(enable ? "enable" : "disable"), csi_link->usecount);

	if (enable && csi_link->usecount == 0) {
		// Enable && first user

		ret = max96724_set_phy_enabled(common, csi_id, true);
		if (ret)
			goto err_unlock;

	} else if (!enable && csi_link->usecount == 1) {
		// Disable && no more users

		ret = max96724_set_phy_enabled(common, csi_id, false);
		if (ret)
			goto err_unlock;

	}

	// Keep track of number of enabled maps using this CSI link
	if (enable)
		csi_link->usecount++;
	else if (csi_link->usecount > 0)
		csi_link->usecount--;

err_unlock:
	mutex_unlock(&csi_link->csi_mutex);

	return ret;
}

static int max96724_csi_double_pixel(struct max9x_common *common,
				     unsigned int csi_id,
				     unsigned int bpp)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	unsigned int value;
	int ret;

	dev_info(dev, "CSI ALT Mem mapping for %u bpp on csi %i", bpp, csi_id);

	switch (bpp) {
	case 0:
		value = 0;
		break;
	case 8:
		value =	FIELD_PREP(MAX96724_MIPI_TX_ALT_MEM_8BPP, 1U);
		dev_err(dev, "8 BPP currently unsupported for pixel doubling");
		return -EINVAL;
	case 10:
		value =	FIELD_PREP(MAX96724_MIPI_TX_ALT_MEM_10BPP, 1U);
		break;
	case 12:
		value =	FIELD_PREP(MAX96724_MIPI_TX_ALT_MEM_12BPP, 1U);
		break;
	default:
		dev_err(dev, "Unsupported BPP for pixel doubling: %u", bpp);
		return -EINVAL;
	}

	// Enable alt mem mapping
	ret = regmap_update_bits(
		map, MAX96724_MIPI_TX_ALT_MEM(csi_id),
		MAX96724_MIPI_TX_ALT_MEM_FIELD, value);

	return ret;

}

static int max96724_set_video_pipe_enabled(struct max9x_common *common,
					   unsigned int pipe_id, bool enable)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	int ret = 0;
	struct mutex *lock = &common->link_mutex;

	mutex_lock(lock);
	dev_dbg(dev, "Video-pipe %d: %s", pipe_id, (enable ? "enable" : "disable"));

	// Enable max96712 "legacy" mode
	// Non "legacy" mode ignores pipe mapping, and selects all streams for pipe
	// 0. The Jetson doesn't know what to do with that and throws spurious data
	// stream errors.
	ret = regmap_update_bits(map, MAX96724_VIDEO_PIPE_EN,
				MAX96724_VIDEO_PIPE_STREAM_SEL_ALL_FIELD,
				MAX9X_FIELD_PREP(MAX96724_VIDEO_PIPE_STREAM_SEL_ALL_FIELD,
				MAX96724_VIDEO_PIPE_LEGACY_MODE));
	if (ret) {
		dev_err(dev, "Failed to clear select all streams bit");
		goto unlock_exit;
	}

	ret = regmap_update_bits(map, MAX96724_VIDEO_PIPE_EN,
		MAX96724_VIDEO_PIPE_EN_FIELD(pipe_id),
		MAX9X_FIELD_PREP(MAX96724_VIDEO_PIPE_EN_FIELD(pipe_id), enable ? 1U : 0U));

unlock_exit:
	mutex_unlock(lock);
	return ret;
}

static int max96724_set_serial_link_routing(struct max9x_common *common,
					    unsigned int link_id)
{
	unsigned int pipe_id;
	unsigned int map_id;
	int ret;

	for (pipe_id = 0; pipe_id < common->num_video_pipes; pipe_id++) {
		struct max9x_serdes_pipe_config *config;

		if (common->video_pipe[pipe_id].enabled == false)
			continue;

		config = &common->video_pipe[pipe_id].config;
		if (config->src_link != link_id)
			continue;

		ret = max96724_set_video_pipe_src(common, pipe_id,
						  config->src_link,
						  config->src_pipe);
		if (ret)
			return ret;

		ret = max96724_set_video_pipe_maps_enabled(common, pipe_id,
							   config->num_maps);
		if (ret)
			return ret;

		for (map_id = 0; map_id < config->num_maps; map_id++) {
			ret = max96724_set_video_pipe_map(common, pipe_id, map_id,
							  &config->map[map_id]);
			if (ret)
				return ret;

			if (!config->map[map_id].is_csi_enabled) {
				ret = max96724_set_csi_link_enabled(common,
							    config->map[map_id].dst_csi,
							    true);
				if (ret)
					return ret;
				config->map[map_id].is_csi_enabled = true;
			}

			ret = max96724_csi_double_pixel(common,
							config->map[map_id].dst_csi,
							config->dbl_pixel_bpp);
			if (ret)
				return ret;
		}

		ret = max96724_set_video_pipe_enabled(common, pipe_id, true);
		if (ret)
			return ret;
	}

	return 0;
}

static int max96724_disable_serial_link(struct max9x_common *common,
					unsigned int link_id)
{
	unsigned int pipe_id;
	unsigned int map_id;
	int ret;

	for (pipe_id = 0; pipe_id < common->num_video_pipes; pipe_id++) {
		struct max9x_serdes_pipe_config *config;

		if (common->video_pipe[pipe_id].enabled == false)
			continue;

		config = &common->video_pipe[pipe_id].config;
		if (config->src_link != link_id)
			continue;

		ret = max96724_set_video_pipe_enabled(common, pipe_id, false);
		if (ret)
			return ret;

		ret = max96724_set_video_pipe_maps_enabled(common, pipe_id, 0);
		if (ret)
			return ret;

		for (map_id = 0; map_id < config->num_maps; map_id++) {
			if (!config->map[map_id].is_csi_enabled)
				continue;
			ret = max96724_set_csi_link_enabled(common, config->map[map_id].dst_csi, false);
			if (ret)
				return ret;
			config->map[map_id].is_csi_enabled = false;
		}
	}

	ret = max96724_set_serial_link_state(common, link_id, false);
	if (ret)
		return ret;

	return 0;
}

static int max96724_enable_serial_link(struct max9x_common *common,
				       unsigned int link_id)
{
	struct device *dev = common->dev;
	int ret;
	bool locked;
	unsigned long timeout;

	if (WARN_ON_ONCE(link_id >= common->num_serial_links))
		return -EINVAL;

	if (WARN_ONCE(common->serial_link[link_id].config.link_type != MAX9X_LINK_TYPE_GMSL2,
		      "Only GMSL2 is supported!"))
		return -EINVAL;

	// GMSL2
	ret = max96724_set_remote_control_channel_enabled(common, link_id, false);
	if (ret)
		return ret;

	ret = max96724_set_serial_link_state(common, link_id, true);
	if (ret)
		return ret;

	ret = max96724_set_serial_link_rate(common, link_id);
	if (ret)
		return ret;

	ret = max96724_serial_link_reset(common, link_id);
	if (ret)
		return ret;

	ret = max96724_set_serial_link_routing(common, link_id);
	if (ret)
		return ret;

	timeout = jiffies + msecs_to_jiffies(max96724_serial_link_timeout_ms);
	do {
		usleep_range(10 * 1000, 25 * 1000); // 10 to 25 ms
		ret = max96724_get_serial_link_lock(common, link_id, &locked);
		if (ret)
			return ret;
	} while (!locked && time_before(jiffies, timeout));

	if (!locked) {
		dev_err(dev, "Serial-link %d: Failed to lock!", link_id);
		return -ETIMEDOUT;
	}

	return 0;
}

static int max96724_set_remote_control_channel_enabled(struct max9x_common *common,
						       unsigned int link_id,
						       bool enabled)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	int ret;
	struct mutex *lock = &common->link_mutex;
	//TODO: Allow DT to choose which port gets enabled: bit 0 disables port 0, bit 1 disables port 1
	//See also 0x0E REG14 DIS_REM_CC_P2[3:0]

	// Note that the register value 1 *disables* port-to-remote command & control
	mutex_lock(lock);
	dev_dbg(dev, "set rem cc %s", (enabled ? "enable" : "disable"));
	if (enabled) {
		ret = regmap_write(map, MAX96724_REM_CC,
			~(MAX9X_FIELD_PREP(MAX96724_REM_CC_DIS_PORT_FIELD(link_id, 0), enabled ? 1 : 0)));
	} else {
		ret = regmap_write(map, MAX96724_REM_CC, ~(0));
	}

	mutex_unlock(lock);
	return ret;
}

static int max96724_select_serial_link(struct max9x_common *common, unsigned int link)
{
	return max96724_set_remote_control_channel_enabled(common, link, true);
}

static int max96724_deselect_serial_link(struct max9x_common *common, unsigned int link)
{
	return max96724_set_remote_control_channel_enabled(common, link, false);
}

static struct max9x_serial_link_ops max96724_serial_link_ops = {
	.enable = max96724_enable_serial_link,
	.disable = max96724_disable_serial_link,
	.select = max96724_select_serial_link,
	.deselect = max96724_deselect_serial_link,
	.get_locked = max96724_get_serial_link_lock,
};

static int max96724_disable_line_fault(struct max9x_common *common, unsigned int line)
{
	return max96724_set_line_fault(common, line, false);
}

static int max96724_enable_line_fault(struct max9x_common *common, unsigned int line)
{
	return max96724_set_line_fault(common, line, true);
}

static int max96724_set_line_fault(struct max9x_common *common, unsigned int line, bool enable)
{
	int ret;

	ret = regmap_update_bits(common->map,
		MAX96724_PU_LF,
		MAX96724_PU_LF_FIELD(line),
		enable ? 0xFF : 0x00);

	return ret;
}

static int max96724_get_line_fault(struct max9x_common *common, unsigned int line)
{
	unsigned int val = 0;
	int ret = regmap_read(common->map, MAX96724_LF(line), &val);

	if (ret < 0)
		return ret;

	return MAX96724_LF_VAL(val, line);
}

static struct max9x_line_fault_ops max96724_line_fault_ops = {
	.enable = max96724_enable_line_fault,
	.disable = max96724_disable_line_fault,
	.get_status = max96724_get_line_fault,
};

static int max96724_enable(struct max9x_common *common)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	int link_id;
	int ret;

	dev_dbg(dev, "Enable");

	for (link_id = 0; link_id < common->num_serial_links; link_id++) {
		ret = max96724_disable_serial_link(common, link_id);
		if (ret)
			return ret;
	}

	// Apply errata tuning
	ret = regmap_multi_reg_write(map, max96724_errata_rev1, ARRAY_SIZE(max96724_errata_rev1));
	if (ret)
		return ret;

	ret = max96724_configure_csi_dphy(common);
	if (ret)
		return ret;

	return 0;
}

static int max96724_enable_csi_link(struct max9x_common *common,
				    unsigned int csi_link_id)
{
	return max96724_set_csi_link_enabled(common, csi_link_id, true);
}

static int max96724_disable_csi_link(struct max9x_common *common,
				     unsigned int csi_link_id)
{
	return max96724_set_csi_link_enabled(common, csi_link_id, false);
}

static struct max9x_csi_link_ops max96724_csi_link_ops = {
	.enable = max96724_enable_csi_link,
	.disable = max96724_disable_csi_link,
};

int max96724_get_ops(struct max9x_common_ops **common_ops, struct max9x_serial_link_ops **serial_ops,
		    struct max9x_csi_link_ops **csi_ops, struct max9x_line_fault_ops **lf_ops,
			struct max9x_translation_ops **trans_ops)
{
	(*common_ops) = &max96724_common_ops;
	(*serial_ops) = &max96724_serial_link_ops;
	(*csi_ops) = &max96724_csi_link_ops;
	(*lf_ops) = &max96724_line_fault_ops;
	(*trans_ops) = NULL;
	return 0;
}

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Cody Burrows <cburrows@d3embedded.com>");
MODULE_AUTHOR("Josh Watts <jwatts@d3embedded.com>");
MODULE_AUTHOR("He Jiabin <jiabin.he@intel.com>");
MODULE_DESCRIPTION("Maxim MAX96724 Quad GMSL2/GMSL1 to CSI-2 Deserializer driver");
