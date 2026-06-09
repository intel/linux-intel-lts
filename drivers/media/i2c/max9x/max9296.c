/*
 * max9296.c - Maxim MAX9296 GMSL2/GMSL1 to CSI-2 Deserializer
 *
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

#include "max9296.h"

// Params
int max9296_serial_link_timeout_ms = MAX9296_DEFAULT_SERIAL_LINK_TIMEOUT_MS;
module_param(max9296_serial_link_timeout_ms, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(max9296_serial_link_timeout_ms, "Timeout for serial link in milliseconds");

// Declarations
static int max9296_set_phy_mode(struct max9x_common *common, unsigned int phy_mode);
static int max9296_set_phy_enabled(struct max9x_common *common, unsigned int csi_id, bool enable);
static int max9296_set_phy_lane_map(struct max9x_common *common, unsigned int csi_id, unsigned int phy_lane_map);
static int max9296_set_phy_dpll_enabled(struct max9x_common *common, unsigned int csi_id, bool enable);
static int max9296_set_phy_dpll_freq(struct max9x_common *common, unsigned int csi_id, unsigned int freq_mhz);
static int max9296_set_mipi_lane_cnt(struct max9x_common *common, unsigned int csi_id, int num_lanes);
static int max9296_set_initial_deskew(struct max9x_common *common, unsigned int csi_id,
				      bool enable, unsigned int width);
static int max9296_configure_csi_dphy(struct max9x_common *common);
static int max9296_enable(struct max9x_common *common);
static int max9296_max_elements(struct max9x_common *common, enum max9x_element_type element);
static int max9296_get_serial_link_lock(struct max9x_common *common, unsigned int link_id, bool *locked);
static int max9296_serial_link_reset(struct max9x_common *common, unsigned int link_id);
static int max9296_set_serial_link_rate(struct max9x_common *common, unsigned int link_id);
static int max9296_set_video_pipe_src(struct max9x_common *common, unsigned int pipe_id,
				      unsigned int link_id, unsigned int src_pipe);
static int max9296_set_video_pipe_maps_enabled(struct max9x_common *common, unsigned int pipe_id, int num_maps);
static int max9296_set_video_pipe_map(struct max9x_common *common, unsigned int pipe_id,
				      unsigned int map_id, struct max9x_serdes_mipi_map *mipi_map);
static int max9296_set_csi_double_loading_mode(struct max9x_common *common, unsigned int csi_id, unsigned int bpp);
static int max9296_set_csi_link_enabled(struct max9x_common *common, unsigned int csi_id, bool enable);
static int max9296_set_video_pipe_enabled(struct max9x_common *common, unsigned int pipe_id, bool enable);
static int max9296_set_serial_link_routing(struct max9x_common *common, unsigned int link_id);
static int max9296_disable_serial_link(struct max9x_common *common, unsigned int link_id);
static int max9296_enable_serial_link(struct max9x_common *common, unsigned int link_id);
static int max9296_isolate_serial_link(struct max9x_common *common, unsigned int link);
static int max9296_deisolate_serial_link(struct max9x_common *common, unsigned int link);
static int max9296_wait_link_lock(struct max9x_common *common, int link);
static int max9296_enable_csi_link(struct max9x_common *common, unsigned int csi_link_id);
static int max9296_disable_csi_link(struct max9x_common *common, unsigned int csi_link_id);

/* Currently unused */
static int max9296_conf_phy_maps(struct max9x_common *common, unsigned int csi_id, unsigned int *phy_lane_map);

// Functions
static int max9296_set_phy_mode(struct max9x_common *common, unsigned int phy_mode)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;

	dev_dbg(dev, "CSI: phy_mode=%d", phy_mode);

	return regmap_update_bits(map, MAX9296_MIPI_PHY0,
		MAX9296_MIPI_PHY0_MODE_FIELD,
		MAX9X_FIELD_PREP(MAX9296_MIPI_PHY0_MODE_FIELD, phy_mode));
}

static int max9296_set_phy_enabled(struct max9x_common *common, unsigned int csi_id, bool enable)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;

	dev_dbg(dev, "CSI link %d: %s", csi_id, (enable ? "enable" : "disable"));

	return regmap_update_bits(map, MAX9296_MIPI_PHY_ENABLE,
		MAX9296_MIPI_PHY_ENABLE_FIELD(csi_id),
		MAX9X_FIELD_PREP(MAX9296_MIPI_PHY_ENABLE_FIELD(csi_id), enable ? 1U : 0U));
}

static int max9296_set_phy_lane_map(struct max9x_common *common, unsigned int csi_id, unsigned int phy_lane_map)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;

	dev_dbg(dev, "CSI link %d: phy_lane_map=0x%04x", csi_id, phy_lane_map);

	return regmap_update_bits(map, MAX9296_MIPI_PHY_LANE_MAP(csi_id),
		MAX9296_MIPI_PHY_LANE_MAP_FIELD(csi_id, 0)
		| MAX9296_MIPI_PHY_LANE_MAP_FIELD(csi_id, 1),
		phy_lane_map);
}

static int max9296_set_phy_dpll_enabled(struct max9x_common *common, unsigned int csi_id, bool enable)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;

	dev_dbg(dev, "CSI link %d: DPLL %s", csi_id, (enable ? "on" : "off"));

	return regmap_update_bits(map, MAX9296_DPLL_RESET(csi_id),
		MAX9296_DPLL_RESET_SOFT_RST_FIELD,
		MAX9X_FIELD_PREP(MAX9296_DPLL_RESET_SOFT_RST_FIELD, enable ? 1U : 0U));
}

static int max9296_set_phy_dpll_freq(struct max9x_common *common, unsigned int csi_id, unsigned int freq_mhz)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;

	if (WARN_ONCE(freq_mhz > 0 && freq_mhz < MAX9296_DPLL_FREQ_MHZ_MULTIPLE,
				"CSI frequency must be greater than %d MHz", MAX9296_DPLL_FREQ_MHZ_MULTIPLE))
		return -EINVAL;

	dev_dbg(dev, "CSI link %d: freq %u MHz, %u mult", csi_id, freq_mhz, freq_mhz / MAX9296_DPLL_FREQ_MHZ_MULTIPLE);

	return regmap_update_bits(map, MAX9296_DPLL_FREQ(csi_id),
		MAX9296_DPLL_FREQ_FIELD,
		MAX9X_FIELD_PREP(MAX9296_DPLL_FREQ_FIELD, freq_mhz / MAX9296_DPLL_FREQ_MHZ_MULTIPLE));
}

static int max9296_set_mipi_lane_cnt(struct max9x_common *common, unsigned int csi_id, int num_lanes)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;

	dev_dbg(dev, "CSI link %d: %d lanes", csi_id, num_lanes);

	return regmap_update_bits(map, MAX9296_MIPI_TX_LANE_CNT(csi_id),
		MAX9296_MIPI_TX_LANE_CNT_FIELD,
		MAX9X_FIELD_PREP(MAX9296_MIPI_TX_LANE_CNT_FIELD,
			(common->csi_link[csi_id].config.num_lanes - 1)));
}

static int max9296_set_initial_deskew(struct max9x_common *common, unsigned int csi_id,
				      bool enable, unsigned int width)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;

	dev_dbg(dev, "CSI link %d: Initial deskew %s", csi_id, enable ? "enabled" : "disabled");

	/* clamp initial deskew width to 7 which is 8*32k UI*/
	if (width > 7)
		width = 7;

	return regmap_write(map, MAX9296_MIPI_TX_DESKEW_INIT(csi_id),
			    MAX9X_FIELD_PREP(MAX9296_MIPI_TX_DESKEW_INIT_AUTO_EN, enable) |
			    MAX9X_FIELD_PREP(MAX9296_MIPI_TX_DESKEW_INIT_WIDTH, width));
}

static int max9296_conf_phy_maps(struct max9x_common *common, unsigned int csi_id,
				 unsigned int *phy_lane_map)
{
	struct device *dev = common->dev;
	int i;
	const unsigned int phy_count = 2;
	const unsigned int controller = csi_id / phy_count;

	if (common->csi_link[csi_id].config.num_lanes != common->csi_link[csi_id].config.num_maps) {
		dev_err(dev, "CSI%u number of maps %u must match number of lanes %u.", csi_id,
			common->csi_link[csi_id].config.num_maps,
			common->csi_link[csi_id].config.num_lanes);
		return -EINVAL;
	}

	for (i = 0; i < common->csi_link[csi_id].config.num_maps; i++) {
		const struct max9x_serdes_phy_map *map = &common->csi_link[csi_id].config.map[i];

		if (map->int_csi >= 4) {
			dev_err(dev, "CSI%u does not have %u Lanes can not map.", csi_id,
				map->int_csi + 1);
			return -EINVAL;
		}
		if (map->phy_lane >= 2) {
			dev_err(dev, "Each PHY has 2 lanes can not map %u.", map->phy_ind);
			return -EINVAL;
		}
		if (map->phy_ind < (controller * phy_count) ||
		    map->phy_ind >= ((controller + 1) * phy_count)) {
			dev_err(dev, "CSI%u does not have PHYs %u can not map.", csi_id,
				map->phy_ind);
			return -EINVAL;
		}
		phy_lane_map[map->phy_ind] |= MAX9X_FIELD_PREP(
			MAX9296_MIPI_PHY_LANE_MAP_FIELD(map->phy_ind, map->phy_lane), map->int_csi);
	}
	return 0;
}

/*
 * MAX9296A has four PHYs, but does not support single-PHY configurations,
 * only double-PHY configurations, even when only using two lanes.
 * For PHY 0 + PHY 1, PHY 1 is the master PHY.
 * For PHY 2 + PHY 3, PHY 2 is the master PHY.
 * Clock is always on the master PHY.
 * For first pair of PHYs, first lanes are on the master PHY.
 * For second pair of PHYs, first lanes are on the master PHY too.
 *
 * PHY 0 + 1
 * CLK is on PHY 1
 * PHY1 Lane 0 = D0
 * PHY1 Lane 1 = D1
 * PHY0 Lane 0 = D2
 * PHY0 Lane 1 = D3
 *
 * PHY 2 + 3
 * CLK is on PHY 2
 * PHY2 Lane 0 = D0
 * PHY2 Lane 1 = D1
 * PHY3 Lane 0 = D2
 * PHY3 Lane 1 = D3
 */
static int max9296_configure_csi_dphy(struct max9x_common *common)
{
	struct device *dev = common->dev;
	unsigned int phy_mode;
	unsigned int phy_lane_map[MAX9296_NUM_CSI_LINKS] = {0};
	unsigned int csi_id;
	unsigned int PHY1, PHY2;
	int ret;

	for (csi_id = 0; csi_id < MAX9296_NUM_CSI_LINKS; csi_id++) {
		dev_dbg(dev, "CSI link %d: enabled=%d, num_lanes=%d, freq_mhz=%d init_deskew=%d",
			csi_id, common->csi_link[csi_id].enabled,
			common->csi_link[csi_id].config.num_lanes,
			common->csi_link[csi_id].config.freq_mhz,
			common->csi_link[csi_id].config.auto_init_deskew_enabled);
	}

	PHY1 = common->csi_link[0].enabled ? common->csi_link[0].config.num_lanes : 0;
	PHY2 = common->csi_link[1].enabled ? common->csi_link[1].config.num_lanes : 0;
	/* Each csi controller has 4 lanes. each phy has 2 lanes*/
	if ((PHY1 + PHY2) > 4) {
		dev_err(dev, "CSI controller 1 has more than %u lanes. PHY0: %u, PHY1: %u", 4, PHY1,
			PHY1);
		return -EINVAL;
	}

	PHY1 = common->csi_link[2].enabled ? common->csi_link[2].config.num_lanes : 0;
	PHY2 = common->csi_link[3].enabled ? common->csi_link[3].config.num_lanes : 0;
	/* Each csi controller has 4 lanes. each phy has 2 lanes */
	if ((PHY1 + PHY2) > 4) {
		dev_err(dev, "CSI controller 2 has more than %u lanes. PHY2: %u, PHY3: %u", 4, PHY1,
			PHY1);
		return -EINVAL;
	}

	if (common->csi_link[1].enabled && common->csi_link[1].config.num_lanes == 4) {
		if (common->csi_link[0].enabled)
			dev_warn(dev, "CSI link 0 enabled, but CSI link 1 is 4 lanes.");

		/* lane 1 is master for CSI controller 1*/
		max9296_conf_phy_maps(common, 1, phy_lane_map);
		if (common->csi_link[2].enabled && common->csi_link[2].config.num_lanes == 4) {
			if (common->csi_link[3].enabled)
				dev_warn(dev, "CSI link 3 enabled, but CSI link 2 is 4 lanes.");
			dev_dbg(dev, "CSI phy mode is set to 2X4Lanes");
			/* lane 2 is master for CSI controller 2*/
			max9296_conf_phy_maps(common, 2, phy_lane_map);
			phy_mode = MAX9296_MIPI_PHY_2X4; // 2x 4lane
		} else {
			dev_dbg(dev, "CSI phy mode is set to A: 4Lanes B: 2X2Lanes");
			max9296_conf_phy_maps(common, 2, phy_lane_map);
			max9296_conf_phy_maps(common, 3, phy_lane_map);
			phy_mode = MAX9296_MIPI_PHY_1X4A_22; // A: 1x 4lane B: 2x 2lane
		}
	} else {
		max9296_conf_phy_maps(common, 1, phy_lane_map);
		max9296_conf_phy_maps(common, 2, phy_lane_map);
		if (common->csi_link[2].enabled && common->csi_link[2].config.num_lanes == 4) {
			if (common->csi_link[3].enabled)
				dev_warn(dev, "CSI link 3 enabled, but CSI link 2 is 4 lanes.");
			dev_dbg(dev, "CSI phy mode is set to A: 2X2Lanes B: 4Lanes");
			/* lane 2 is master for CSI controller 2*/
			max9296_conf_phy_maps(common, 2, phy_lane_map);
			phy_mode = MAX9296_MIPI_PHY_1X4B_22; // B: 1x 4lane A: 2x 2lane
		} else {
			dev_dbg(dev, "CSI phy mode is set to 4X2Lanes");
			max9296_conf_phy_maps(common, 2, phy_lane_map);
			max9296_conf_phy_maps(common, 3, phy_lane_map);
			phy_mode = MAX9296_MIPI_PHY_4X2; // 4x 2lane
		}
	}

	TRY(ret, max9296_set_phy_mode(common, phy_mode));

	for (csi_id = 0; csi_id < MAX9296_NUM_CSI_LINKS; csi_id++) {
		struct max9x_serdes_csi_config *config =
			&common->csi_link[csi_id].config;

		TRY(ret, max9296_set_phy_enabled(common, csi_id, false));

		TRY(ret, max9296_set_phy_dpll_enabled(common, csi_id, false));

		TRY(ret, max9296_set_phy_lane_map(common, csi_id,
				phy_lane_map[csi_id])
		);

		TRY(ret, max9296_set_mipi_lane_cnt(common, csi_id,
				config->num_lanes)
		);
	}

	return 0;
}

static int max9296_set_all_reset(struct max9x_common *common, bool enable)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;

	dev_dbg(dev, "Reset ALL %s", (enable ? "enable" : "disable"));

	return regmap_update_bits(map, MAX9296_CTRL0,
			MAX9296_CTRL0_RESET_ALL_FIELD,
			MAX9X_FIELD_PREP(MAX9296_CTRL0_RESET_ALL_FIELD, enable ? 1U : 0U));
}

static int max9296_soft_reset(struct max9x_common *common)
{
	struct device *dev = common->dev;
	int ret;

	dev_dbg(dev, "Soft reset");

	TRY(ret, max9296_set_all_reset(common, 1));
	/* lock time for reset_all / PWDNB in spec, I2C Wake time is 2.25ms */
	usleep_range(45000, 45050);
	TRY(ret, max9296_set_all_reset(common, 0));

	return 0;
}

static int max9296_max_elements(struct max9x_common *common, enum max9x_element_type element)
{
	switch (element) {
	case MAX9X_SERIAL_LINK:
		return MAX9296_NUM_SERIAL_LINKS;
	case MAX9X_VIDEO_PIPE:
		return MAX9296_NUM_VIDEO_PIPES;
	case MAX9X_MIPI_MAP:
		return MAX9296_NUM_MIPI_MAPS;
	case MAX9X_CSI_LINK:
		return MAX9296_NUM_CSI_LINKS;
	default:
		break;
	}

	return 0;
}

static struct max9x_common_ops max9296_common_ops = {
	.enable = max9296_enable,
	.soft_reset = max9296_soft_reset,
	.max_elements = max9296_max_elements,
};

static int max9296_set_video_pipe_src(struct max9x_common *common, unsigned int pipe_id,
				      unsigned int link_id, unsigned int src_pipe)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;

	/*
	 * link_id ignored for max9296, pipe routing done through stream-ids,
	 * which must be unique across links
	 */
	dev_dbg(dev, "Video-pipe %d: src_pipe=%u", pipe_id, src_pipe);

	return regmap_update_bits(map, MAX9296_VIDEO_PIPE_SEL(pipe_id),
			MAX9296_VIDEO_PIPE_STR_SEL_FIELD,
			MAX9X_FIELD_PREP(MAX9296_VIDEO_PIPE_STR_SEL_FIELD, src_pipe));
}

static int max9296_set_video_pipe_maps_enabled(struct max9x_common *common, unsigned int pipe_id, int num_maps)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	unsigned int val = 0;
	int ret;

	if (num_maps > 0)
		val = GENMASK(num_maps - 1, 0);

	dev_dbg(dev, "Video-pipe %d: num_maps=%u", pipe_id, num_maps);

	TRY(ret, regmap_write(map, MAX9296_MAP_EN_L(pipe_id),
			MAX9X_FIELD_PREP(MAX9296_MAP_EN_FIELD, val))
	);

	TRY(ret, regmap_write(map, MAX9296_MAP_EN_H(pipe_id),
			MAX9X_FIELD_PREP(MAX9296_MAP_EN_FIELD, val >> 8))
	);

	return 0;
}

static int max9296_set_video_pipe_map(struct max9x_common *common, unsigned int pipe_id,
				      unsigned int map_id, struct max9x_serdes_mipi_map *mipi_map)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	int ret;

	dev_dbg(dev, "Video-pipe %d, map %d: VC%d:DT%02x->VC%d:DT%02x, dst_csi=%d ",
		pipe_id, map_id, mipi_map->src_vc, mipi_map->src_dt,
		mipi_map->dst_vc, mipi_map->dst_dt, mipi_map->dst_csi);

	TRY(ret, regmap_write(map, MAX9296_MAP_SRC_L(pipe_id, map_id),
			MAX9X_FIELD_PREP(MAX9296_MAP_SRC_L_VC_FIELD, mipi_map->src_vc) |
			MAX9X_FIELD_PREP(MAX9296_MAP_SRC_L_DT_FIELD, mipi_map->src_dt))
	);

	TRY(ret, regmap_write(map, MAX9296_MAP_DST_L(pipe_id, map_id),
			MAX9X_FIELD_PREP(MAX9296_MAP_DST_L_VC_FIELD, mipi_map->dst_vc) |
			MAX9X_FIELD_PREP(MAX9296_MAP_DST_L_DT_FIELD, mipi_map->dst_dt))
	);

	TRY(ret, regmap_write(map, MAX9296_MAP_SRCDST_H(pipe_id, map_id),
			MAX9X_FIELD_PREP(MAX9296_MAP_SRCDST_H_SRC_VC_FIELD, mipi_map->src_vc) |
			MAX9X_FIELD_PREP(MAX9296_MAP_SRCDST_H_DST_VC_FIELD, mipi_map->dst_vc))
	);

	TRY(ret, regmap_update_bits(map, MAX9296_MAP_DPHY_DEST(pipe_id, map_id),
			MAX9296_MAP_DPHY_DEST_FIELD(map_id),
			MAX9X_FIELD_PREP(MAX9296_MAP_DPHY_DEST_FIELD(map_id), mipi_map->dst_csi))
	);

	return 0;
}

/**
 * max9296_set_csi_double_loading_mode() - Configure Double Loading Mode on a CSI controller
 * @common: max9x_common
 * @csi_id: Target CSI controller's ID
 * @bpp: Original BPP to double. This can be 0 (disables), 8, 10, or 12.
 *
 * Double loading mode squeezes two input pixels together such that they are
 * treated as a single pixel by the video pipe. Using this method increases
 * bandwidth efficiency.
 *
 * See: GMSL2 Customers User Guide Section 30.5.1.1.1.2 "Double Loading Mode"
 * See: GMSL2 Customers User Guide Section 43.4.2.3 "Double Mode (Deserializer)"
 */
static int max9296_set_csi_double_loading_mode(struct max9x_common *common, unsigned int csi_id, unsigned int bpp)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	unsigned int value;

	switch (bpp) {
	case 0:
		value = 0;
		break;
	case 8:
		value =	FIELD_PREP(MAX9296_MIPI_TX_ALT_MEM_8BPP, 1U);
		// To fully support 8bpp, additional register writes are
		// needed for 'bpp8dbl' and 'bpp8dbl_mode' fields on each pipe.
		dev_err(dev, "8 BPP currently unsupported for pixel doubling");
		return -EINVAL;
	case 10:
		value =	FIELD_PREP(MAX9296_MIPI_TX_ALT_MEM_10BPP, 1U);
		break;
	case 12:
		value =	FIELD_PREP(MAX9296_MIPI_TX_ALT_MEM_12BPP, 1U);
		break;
	default:
		dev_err(dev, "Unsupported BPP for pixel doubling: %u", bpp);
		return -EINVAL;
	}

	if (bpp > 0)
		dev_info(dev, "Configuring double loading mode on CSI %d: %u bpp -> %u bpp",
			csi_id, bpp, (bpp * 2));

	// Enable alt mem mapping
	return regmap_update_bits(map, MAX9296_MIPI_TX_ALT_MEM(csi_id),
			MAX9296_MIPI_TX_ALT_MEM_FIELD, value);
}

static int max9296_set_csi_link_enabled(struct max9x_common *common, unsigned int csi_id, bool enable)
{
	struct device *dev = common->dev;
	struct max9x_serdes_csi_link *csi_link;
	int ret = 0;

	if (csi_id >= common->num_csi_links)
		return -EINVAL;

	csi_link = &common->csi_link[csi_id];

	if (WARN_ONCE(enable && csi_link->enabled == false, "Tried to enable a disabled CSI port???"))
		return -EINVAL;

	if (WARN_ONCE(enable && csi_link->config.num_lanes == 0, "Tried to enable CSI port with no lanes???"))
		return -EINVAL;

	mutex_lock(&csi_link->csi_mutex);

	dev_dbg(dev, "CSI link %d: %s (%d users)", csi_id, (enable ? "enable" : "disable"), csi_link->usecount);

	if (enable && csi_link->usecount == 0) {
		// Enable && first user
		ret = max9296_set_initial_deskew(common, csi_id, csi_link->config.auto_init_deskew_enabled,
						 csi_link->config.initial_deskew_width);
		if (ret)
			goto err_unlock;

		ret = max9296_set_phy_dpll_freq(common, csi_id, csi_link->config.freq_mhz);
		if (ret)
			goto err_unlock;

		ret = max9296_set_phy_dpll_enabled(common, csi_id, true);
		if (ret)
			goto err_unlock;

		ret = max9296_set_phy_enabled(common, csi_id, true);
		if (ret)
			goto err_unlock;

	} else if (!enable && csi_link->usecount == 1) {
		// Disable && no more users
		ret = max9296_set_phy_enabled(common, csi_id, false);
		if (ret)
			goto err_unlock;

		ret = max9296_set_phy_dpll_enabled(common, csi_id, false);
		if (ret)
			goto err_unlock;
	}

	if (enable)
		csi_link->usecount++;
	else if (csi_link->usecount > 0)
		csi_link->usecount--;

err_unlock:
	mutex_unlock(&csi_link->csi_mutex);

	return ret;
}

static int max9296_set_video_pipe_enabled(struct max9x_common *common, unsigned int pipe_id, bool enable)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;

	dev_dbg(dev, "Video-pipe %d: %s", pipe_id, (enable ? "enable" : "disable"));

	return regmap_update_bits(map, MAX9296_VIDEO_PIPE_EN(pipe_id),
			MAX9296_VIDEO_PIPE_EN_FIELD(pipe_id),
			MAX9X_FIELD_PREP(MAX9296_VIDEO_PIPE_EN_FIELD(pipe_id), enable ? 1U : 0U));
}

/***** max9296_serial_link_ops auxiliary functions *****/
static int max9296_set_serial_link_rate(struct max9x_common *common, unsigned int link_id)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	struct max9x_serdes_serial_config *config = &common->serial_link[link_id].config;
	int tx_rate, rx_rate;

	tx_rate = max9x_serdes_mhz_to_rate(max9296_tx_rates, ARRAY_SIZE(max9296_tx_rates), config->tx_freq_mhz);
	if (tx_rate < 0)
		return tx_rate;

	rx_rate = max9x_serdes_mhz_to_rate(max9296_rx_rates, ARRAY_SIZE(max9296_rx_rates), config->rx_freq_mhz);
	if (rx_rate < 0)
		return rx_rate;

	dev_dbg(dev, "Serial-link %d: TX=%d MHz RX=%d MHz", link_id, config->tx_freq_mhz, config->rx_freq_mhz);

	return regmap_update_bits(map, MAX9296_PHY_REM_CTRL,
			MAX9296_PHY_REM_CTRL_TX_FIELD | MAX9296_PHY_REM_CTRL_RX_FIELD,
			MAX9X_FIELD_PREP(MAX9296_PHY_REM_CTRL_TX_FIELD, tx_rate) |
			MAX9X_FIELD_PREP(MAX9296_PHY_REM_CTRL_RX_FIELD, rx_rate));
}

static int max9296_set_serial_link_routing(struct max9x_common *common, unsigned int link_id)
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

		ret = max9296_set_video_pipe_src(common, pipe_id, config->src_link, config->src_pipe);
		if (ret)
			return ret;

		ret = max9296_set_video_pipe_maps_enabled(common, pipe_id, config->num_maps);
		if (ret)
			return ret;

		for (map_id = 0; map_id < config->num_maps; map_id++) {
			ret = max9296_set_video_pipe_map(common, pipe_id, map_id, &config->map[map_id]);
			if (ret)
				return ret;

			ret = max9296_set_csi_double_loading_mode(common,
								  config->map[map_id].dst_csi, config->dbl_pixel_bpp);
			if (ret)
				return ret;

			if (!config->map[map_id].is_csi_enabled && common->csi_link[config->map[map_id].dst_csi].config.auto_start) {
				ret = max9296_set_csi_link_enabled(common, config->map[map_id].dst_csi, true);
				if (ret)
					return ret;
				config->map[map_id].is_csi_enabled = true;
			}
		}

		ret = max9296_set_video_pipe_enabled(common, pipe_id, true);
		if (ret)
			return ret;
	}

	return 0;
}

static int max9296_serial_link_reset(struct max9x_common *common, unsigned int link_id)
{
	struct regmap *map = common->map;

	/* GMSL RX rate must be the same as the SER. This is set in REG1(0x1)[1:0] */
	return regmap_update_bits(map, MAX9296_CTRL0, MAX9296_CTRL0_RESET_ONESHOT_FIELD,
				  MAX9X_FIELD_PREP(MAX9296_CTRL0_RESET_ONESHOT_FIELD, 1U));
}

static int max9296_get_serial_link_lock(struct max9x_common *common, unsigned int link_id, bool *locked)
{
	struct regmap *map = common->map;
	unsigned int val;
	int ret;

	/* Only looks at link A refer to header file */
	ret = regmap_read(map, MAX9296_PHY_LOCKED(link_id), &val);
	if (ret)
		return ret;

	if (FIELD_GET(MAX9296_PHY_LOCKED_FIELD, val) != 0)
		*locked = true;
	else
		*locked = false;

	return 0;
}

static int max9296_wait_link_lock(struct max9x_common *common, int link)
{
	bool locked;
	int ret;
	ulong timeout = jiffies + msecs_to_jiffies(max9296_serial_link_timeout_ms);

	do {
		ret = max9296_get_serial_link_lock(common, link, &locked);
		if (ret == 0 && locked)
			return 0;

		usleep_range(1000, 2000);
	} while (time_is_after_jiffies(timeout));

	return -ETIMEDOUT;
}
/***** max9296_serial_link_ops auxiliary functions *****/

/***** max9296_serial_link_ops *****/
static int max9296_isolate_serial_link(struct max9x_common *common, unsigned int link)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	unsigned int link_cfg;
	unsigned int auto_link;
	int ret;

	dev_dbg(dev, "Isolate link %d", link);

	auto_link = 0;
	link_cfg = (link == 0) ? MAX9296_LINK_A : MAX9296_LINK_B;

	TRY_DEV_HERE(ret, regmap_update_bits(map, MAX9296_CTRL0,
		MAX9296_CTRL0_AUTO_CFG_FIELD | MAX9296_CTRL0_LINK_CFG_FIELD,
		FIELD_PREP(MAX9296_CTRL0_AUTO_CFG_FIELD, auto_link)
		| FIELD_PREP(MAX9296_CTRL0_LINK_CFG_FIELD, link_cfg)),
		dev);

	TRY_DEV_HERE(ret, max9296_serial_link_reset(common, link), dev);

	TRY_DEV_HERE(ret, max9296_wait_link_lock(common, link), dev);

	return 0;
}

static int max9296_deisolate_serial_link(struct max9x_common *common, unsigned int link)
{
	struct device *dev = common->dev;
	struct regmap *map = common->map;
	unsigned int link_cfg;
	unsigned int auto_link = 0;
	int ret;
	bool link_a = common->serial_link[0].detected;
	bool link_b = common->serial_link[1].detected;

	if (link_a && link_b)
		link_cfg = MAX9296_LINK_SPLIT;
	else if (link_a)
		link_cfg = MAX9296_LINK_A;
	else if (link_b)
		link_cfg = MAX9296_LINK_B;
	else {
		dev_err(dev, "No link was detected");
		return -EINVAL;
	}

	dev_dbg(dev, "Deisolate link %d (link_cfg=%d)", link, link_cfg);

	TRY_DEV_HERE(ret, regmap_update_bits(
			map,
			MAX9296_CTRL0,
			MAX9296_CTRL0_AUTO_CFG_FIELD
			|MAX9296_CTRL0_LINK_CFG_FIELD,
			FIELD_PREP(MAX9296_CTRL0_AUTO_CFG_FIELD, auto_link)
			|FIELD_PREP(MAX9296_CTRL0_LINK_CFG_FIELD, link_cfg)),
		dev);

	TRY_DEV_HERE(ret, max9296_serial_link_reset(common, link), dev);

	TRY_DEV_HERE(ret, max9296_wait_link_lock(common, link), dev);

	return 0;
}

static int max9296_enable_serial_link(struct max9x_common *common, unsigned int link_id)
{
	int ret;

	if (WARN_ON_ONCE(link_id >= common->num_serial_links))
		return -EINVAL;

	if (WARN_ONCE(common->serial_link[link_id].config.link_type
				!= MAX9X_LINK_TYPE_GMSL2, "Only GMSL2 is supported!"))
		return -EINVAL;

	// GMSL2
	ret = max9296_set_serial_link_rate(common, link_id);
	if (ret)
		return ret;

	ret = max9296_isolate_serial_link(common, link_id);
	if (ret)
		return ret;

	common->serial_link[link_id].detected = true;

	ret = max9296_set_serial_link_routing(common, link_id);
	if (ret)
		return ret;

	max9296_deisolate_serial_link(common, link_id);

	return 0;
}

static int max9296_disable_serial_link(struct max9x_common *common, unsigned int link_id)
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

		ret = max9296_set_video_pipe_enabled(common, pipe_id, false);
		if (ret)
			return ret;

		ret = max9296_set_video_pipe_maps_enabled(common, pipe_id, 0);
		if (ret)
			return ret;

		for (map_id = 0; map_id < config->num_maps; map_id++) {
			if (!config->map[map_id].is_csi_enabled)
				continue;
			ret = max9296_set_csi_link_enabled(common, config->map[map_id].dst_csi, false);
			if (ret)
				return ret;
			config->map[map_id].is_csi_enabled = false;
		}
	}

	return 0;
}

static struct max9x_serial_link_ops max9296_serial_link_ops = {
	.enable = max9296_enable_serial_link,
	.disable = max9296_disable_serial_link,
	.isolate = max9296_isolate_serial_link,
	.deisolate = max9296_deisolate_serial_link,
};
/***** max9296_serial_link_ops *****/

static int max9296_enable_csi_link(struct max9x_common *common, unsigned int csi_link_id)
{
	return max9296_set_csi_link_enabled(common, csi_link_id, true);
}

static int max9296_disable_csi_link(struct max9x_common *common, unsigned int csi_link_id)
{
	return max9296_set_csi_link_enabled(common, csi_link_id, false);
}

static struct max9x_csi_link_ops max9296_csi_link_ops = {
	.enable = max9296_enable_csi_link,
	.disable = max9296_disable_csi_link,
};

static int max9296_enable(struct max9x_common *common)
{
	struct device *dev = common->dev;
	int link_id;
	int ret;

	dev_dbg(dev, "Enable");

	for (link_id = 0; link_id < common->num_serial_links; link_id++) {
		ret = max9296_disable_serial_link(common, link_id);
		if (ret)
			return ret;
	}

	ret = max9296_configure_csi_dphy(common);

	if (ret)
		return ret;

	return 0;
}

int max9296_get_ops(struct max9x_common_ops **common_ops, struct max9x_serial_link_ops **serial_ops,
		    struct max9x_csi_link_ops **csi_ops, struct max9x_line_fault_ops **lf_ops,
			struct max9x_translation_ops **trans_ops)
{
	*common_ops = &max9296_common_ops;
	*serial_ops = &max9296_serial_link_ops;
	*csi_ops = &max9296_csi_link_ops;
	*lf_ops = NULL;
	*trans_ops = NULL;

	return 0;
}

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Josh Watts <jwatts@d3embedded.com>");
MODULE_AUTHOR("Jacob Kiggins <jkiggins@d3embedded.com>");
MODULE_AUTHOR("Yan Dongcheng <dongcheng.yan@intel.com>");
MODULE_DESCRIPTION("Maxim MAX9296 Dual GMSL2/GMSL1 to CSI-2 Deserializer driver");
