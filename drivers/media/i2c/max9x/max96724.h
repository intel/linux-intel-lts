/*
 * max96724.h - Maxim MAX96724 registers and constants.
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
// Copyright (c) 2025 Intel Corporation.

#ifndef _MAX96724_H_
#define _MAX96724_H_

#include <linux/bitops.h>
#include <linux/regmap.h>
#include "serdes.h"

enum max96724_phy_mode {
	MAX96724_MIPI_PHY_4X2 = BIT(0), // four 2-lane ports
	MAX96724_MIPI_PHY_2X4 = BIT(2), // two 4-lane ports (CSI1 is master for port A, CSI2 for port B)
	MAX96724_MIPI_PHY_1X4A_22 = BIT(3), // one 4-lane (PHY0+PHY1, CSI1 for port A) port and two 2-lane ports
	MAX96724_MIPI_PHY_1X4B_22 = BIT(4), // one 4-lane (PHY2+PHY3, CSI2 for port B) port and two 2-lane ports
};

// described in detail on page 340 of the datasheet & reg doc
enum max96724_initial_deskew_width {
	MAX96724_INIT_DESKEW_1x32UI = 0,
	MAX96724_INIT_DESKEW_2x32UI,
	MAX96724_INIT_DESKEW_3x32UI,
	MAX96724_INIT_DESKEW_4x32UI,
	MAX96724_INIT_DESKEW_5x32UI,
	MAX96724_INIT_DESKEW_6x32UI,
	MAX96724_INIT_DESKEW_7x32UI,
	MAX96724_INIT_DESKEW_8x32UI,
	MAX96724_INIT_DESKEW_WIDTH_MAX = MAX96724_INIT_DESKEW_8x32UI,
};

enum max96724_periodic_deskew_width {
	MAX96724_PERIODIC_DESKEW_1x1KUI = 0,
	MAX96724_PERIODIC_DESKEW_2x1KUI,
	MAX96724_PERIODIC_DESKEW_3x1KUI,
	MAX96724_PERIODIC_DESKEW_4x1KUI,
	MAX96724_PERIODIC_DESKEW_5x1KUI,
	MAX96724_PERIODIC_DESKEW_6x1KUI,
	MAX96724_PERIODIC_DESKEW_7x1KUI,
	MAX96724_PERIODIC_DESKEW_8x1KUI,
	MAX96724_PERIODIC_DESKEW_WIDTH_MAX = MAX96724_PERIODIC_DESKEW_8x1KUI,
};

enum max96724_periodic_deskew_interval {
	MAX96724_PERIODIC_DESKEW_1FRAME = 0,
	MAX96724_PERIODIC_DESKEW_2FRAME,
	MAX96724_PERIODIC_DESKEW_4FRAME,
	MAX96724_PERIODIC_DESKEW_8FRAME,
	MAX96724_PERIODIC_DESKEW_16FRAME,
	MAX96724_PERIODIC_DESKEW_32FRAME,
	MAX96724_PERIODIC_DESKEW_64FRAME,
	MAX96724_PERIODIC_DESKEW_128FRAME,
	MAX96724_PERIODIC_DESKEW_MAXFRAME = MAX96724_PERIODIC_DESKEW_128FRAME,
};

enum max96724_fsync_pin {
	MAX96724_FSYNC_PIN_MFP0 = 0,
	MAX96724_FSYNC_PIN_MFP7 = 1,
};

enum max96724_fsync_mode {
	MAX96724_FSYNC_GEN_ON_GPIO_OFF = 0,
	MAX96724_FSYNC_GEN_ON_GPIO_OUTPUT,
	MAX96724_FSYNC_GEN_OFF_GPIO_INPUT,
	MAX96724_FSYNC_GEN_OFF_GPIO_OFF,
};

enum max96724_fsync_method {
	MAX96724_FSYNC_METHOD_MANUAL = 0,
	MAX96724_FSYNC_METHOD_SEMI_AUTO,
	MAX96724_FSYNC_METHOD_AUTO,
};

enum max96724_fsync_gpio_type {
	MAX96724_FS_GPIO_TYPE_GMSL1 = 0,
	MAX96724_FS_GPIO_TYPE_GMSL2,
};

enum max96724_virtual_channel_size {
	MAX96724_VC_2_BITS = 0,
	MAX96724_VC_4_BITS = 1,
};

#define MAX96724_NUM_SERIAL_LINKS 4
#define MAX96724_NUM_VIDEO_PIPES 4
#define MAX96724_NUM_MIPI_MAPS 16
#define MAX96724_NUM_CSI_LINKS 4
#define MAX96724_NUM_LINE_FAULTS 4
// There are 16 GPIO registers and 11 GPIO TX/RX forwarding registers, but only 9 pins exposed
#define MAX96724_NUM_GPIOS 9

#define MAX96724_DEFAULT_SERIAL_LINK_TIMEOUT_MS 200

#define MAX96724_DPLL_FREQ_MHZ_MULTIPLE 100

#define MAX96724_FLD_OFS(n, bits_per_field, count) (((n) % (count)) * (bits_per_field))
#define MAX96724_OFFSET_GENMASK(offset, h, l) GENMASK(offset + h, offset + l)

#define MAX96724_REM_CC 0x03
#define MAX96724_REM_CC_DIS_PORT_FIELD(link, port) BIT(MAX96724_FLD_OFS(link, 2, 4) + (port % 2))
#define MAX96724_LINK_CTRL 0x06
#define MAX96724_LINK_CTRL_GMSL_FIELD(link) (BIT(link) << 4)
#define MAX96724_LINK_CTRL_EN_FIELD(link) BIT(link)
#define MAX96724_PHY_RATE_CTRL(link) (0x10 + ((link) / 2))
#define MAX96724_PHY_RATE_CTRL_TX_FIELD(link) (GENMASK(1, 0) << (MAX96724_FLD_OFS(link, 4, 2) + 2))
#define MAX96724_PHY_RATE_CTRL_RX_FIELD(link) (GENMASK(1, 0) << (MAX96724_FLD_OFS(link, 4, 2) + 0))

// Link lock registers
#define MAX96724_PHY_LOCKED(link) ((link) == 0 ? 0x1A : 0x0A + ((link) - 1))
#define MAX96724_PHY_LOCKED_FIELD BIT(3)
#define MAX96724_RESET_ALL 0x13
#define MAX96724_RESET_ALL_FIELD BIT(6)
#define MAX96724_RESET_CTRL 0x18
#define MAX96724_RESET_CTRL_FIELD(link) BIT(link)
#define MAX96724_DEV_REV 0x4C
#define MAX96724_DEV_REV_FIELD GENMASK(3, 0)

#define MAX96724_VIDEO_PIPE_SEL(pipe) (0xF0 + ((pipe) / 2))
#define MAX96724_VIDEO_PIPE_SEL_LINK_FIELD(link) \
	(GENMASK(1, 0) << (MAX96724_FLD_OFS(link, 4, 2) + 2))
#define MAX96724_VIDEO_PIPE_SEL_INPUT_FIELD(link) \
	(GENMASK(1, 0) << (MAX96724_FLD_OFS(link, 4, 2) + 0))

#define MAX96724_VIDEO_PIPE_EN 0xF4
#define MAX96724_VIDEO_PIPE_EN_FIELD(pipe) BIT(pipe)
#define MAX96724_VIDEO_PIPE_STREAM_SEL_ALL_FIELD BIT(4)
#define MAX96724_VIDEO_PIPE_LEGACY_MODE 0

#define MAX96724_DPLL_FREQ(phy) (0x415 + ((phy) * 3))
#define MAX96724_DPLL_FREQ_FIELD GENMASK(4, 0)

#define MAX96724_MIPI_TX_EXT(pipe) (0x800 + ((pipe) * 0x10))

#define MAX96724_MIPI_PHY0 0x8A0
#define MAX96724_MIPI_PHY0_MODE_FIELD GENMASK(4, 0)
#define MAX96724_MIPI_PHY0_CLK_PHY3 BIT(6)
#define MAX96724_MIPI_PHY0_CLK_PHY0 BIT(5)

#define MAX96724_MIPI_PHY_ENABLE 0x8A2
#define MAX96724_MIPI_PHY_ENABLE_FIELD(csi) BIT((csi) + 4)
#define MAX96724_MIPI_PHY_LANE_MAP(csi) (0x8A3 + (csi) / 2)
#define MAX96724_MIPI_PHY_LANE_MAP_FIELD(csi, lane) \
	(GENMASK(1, 0) << (MAX96724_FLD_OFS(csi, 4, 2) + MAX96724_FLD_OFS(lane, 2, 2)))

// Note that CSIs and pipes overlap:
// 0x901 through 0x90A and 0x933 through 0x934 are CSIs, repeated every 0x40 up to 4 times
// 0x90B through 0x932 are pipes, repeated every 0x40 up to 4 times
#define MAX96724_MIPI_TX(pipe) (0x900 + ((pipe) * 0x40))
#define MAX96724_MIPI_TX_LANE_CNT(csi) (MAX96724_MIPI_TX(csi) + 0x0A)
#define MAX96724_MIPI_TX_LANE_CNT_FIELD GENMASK(7, 6)
#define MAX96724_MIPI_TX_CPHY_EN_FIELD BIT(5)
#define MAX96724_MIPI_TX_VCX_EN_FIELD BIT(4)
#define MAX96724_MIPI_TX_DESKEW_INIT(csi) (MAX96724_MIPI_TX(csi) + 0x03)
#define MAX96724_MIPI_TX_DESKEW_INIT_AUTO_EN BIT(7)
#define MAX96724_MIPI_TX_DESKEW_WIDTH_FIELD GENMASK(2, 0)
#define MAX96724_MIPI_TX_DESKEW_PERIODIC(csi) (MAX96724_MIPI_TX(csi) + 0x04)
#define MAX96724_MIPI_TX_DESKEW_PERIODIC_EN BIT(7)
#define MAX96724_MIPI_TX_DESKEW_PERIODIC_INTERVAL_FIELD GENMASK(5, 3)
#define MAX96724_MIPI_TX_DESKEW_PERIODIC_WIDTH_FIELD GENMASK(2, 0)
#define MAX96724_MAP_EN_L(pipe) (MAX96724_MIPI_TX(pipe) + 0x0B)
#define MAX96724_MAP_EN_H(pipe) (MAX96724_MIPI_TX(pipe) + 0x0C)
#define MAX96724_MAP_EN_FIELD GENMASK(7, 0)
#define MAX96724_MAP_SRC_L(pipe, map) (MAX96724_MIPI_TX(pipe) + 0x0D + ((map) * 2))
#define MAX96724_MAP_SRC_L_VC_FIELD GENMASK(7, 6)
#define MAX96724_MAP_SRC_L_DT_FIELD GENMASK(5, 0)
#define MAX96724_MAP_DST_L(pipe, map) (MAX96724_MIPI_TX(pipe) + 0x0D + ((map) * 2) + 1)
#define MAX96724_MAP_DST_L_VC_FIELD GENMASK(7, 6)
#define MAX96724_MAP_DST_L_DT_FIELD GENMASK(5, 0)
#define MAX96724_MAP_SRCDST_H(pipe, map) (MAX96724_MIPI_TX_EXT(pipe) + (map))
#define MAX96724_MAP_SRCDST_H_SRC_VC_FIELD GENMASK(7, 5)
#define MAX96724_MAP_SRCDST_H_DST_VC_FIELD GENMASK(4, 2)
#define MAX96724_MAP_DPHY_DEST(pipe, map) (MAX96724_MIPI_TX(pipe) + 0x2D + ((map) / 4))
#define MAX96724_MAP_DPHY_DEST_FIELD(map) (GENMASK(1, 0) << MAX96724_FLD_OFS(map, 2, 4))

#define MAX96724_MIPI_TX_ALT_MEM(csi) (MAX96724_MIPI_TX(csi) + 0x33)
#define MAX96724_MIPI_TX_ALT_MEM_FIELD GENMASK(2, 0)
#define MAX96724_MIPI_TX_ALT_MEM_8BPP BIT(1)
#define MAX96724_MIPI_TX_ALT_MEM_10BPP BIT(2)
#define MAX96724_MIPI_TX_ALT_MEM_12BPP BIT(0)

#define MAX96724_VID_TX(pipe) (0x100 + (0x12 * (pipe)))

// GPIO Registers
#define MAX96724_GPIO_REG(n) (0x300 + 3 * n)
#define MAX96724_GPIO_RES_CFG BIT(7)
#define MAX96724_GPIO_TX_ENABLE BIT(1)
#define MAX96724_GPIO_OUTDRV_DISABLE BIT(0)

#define MAX96724_GPIO_A_REG(n) (MAX96724_GPIO_REG(n) + 1)

#define MAX96724_GPIO_B_REG(n) \
	((n <= 2) ? (0x337 + 3 * n) : (n <= 7) ? (0x341 + 3 * (n - 3)) \
	: (0x351 + 3 * (n - 8)))

#define MAX96724_GPIO_C_REG(n) \
	((n == 0) ? (0x36D) : (n <= 5) ? (0x371 + 3 * (n - 1)) \
	: (0x381 + 3 * (n - 6)))

#define MAX96724_GPIO_D_REG(n) \
	((n <= 3) ? (0x3A4 + 3 * n) : (n <= 8) ? (0x3B1 + 3 * (n - 4)) \
	: (0x3C1 + 3 * (n - 9)))

#define MAX96724_GPIO_PUSH_PULL BIT(5)

// Native frame sync registers
#define MAX96724_FSYNC_0 (0x4A0)
#define MAX96724_FSYNC_OUT_PIN_FIELD BIT(5)
#define MAX96724_EN_VS_GEN_FIELD BIT(6)
#define MAX96724_FSYNC_MODE_FIELD GENMASK(3, 2)
#define MAX96724_FSYNC_METH_FIELD GENMASK(1, 0)

#define MAX96724_FSYNC_5 (0x4A5)
#define MAX96724_FSYNC_PERIOD_L_FIELD GENMASK(7, 0)
#define MAX96724_FSYNC_6 (0x4A6)
#define MAX96724_FSYNC_PERIOD_M_FIELD GENMASK(7, 0)
#define MAX96724_FSYNC_7 (0x4A7)
#define MAX96724_FSYNC_PERIOD_H_FIELD GENMASK(7, 0)

#define MAX96724_FSYNC_15 (0x4AF)
#define MAX96724_FS_GPIO_TYPE_FIELD BIT(7)
#define MAX96724_FS_USE_XTAL_FIELD BIT(6)
#define MAX96724_AUTO_FS_LINKS_FIELD BIT(4)
#define MAX96724_FS_LINK_FIELD(link) BIT(link)

#define MAX96724_FSYNC_17 (0x4B1)
#define MAX96724_FSYNC_TX_ID_FIELD GENMASK(7, 3)
#define MAX96724_FSYNC_ERR_THR_FIELD GENMASK(2, 0)

// Line fault status registers
#define MAX96724_LF(link) (0xE1 + ((link) / 2))
#define MAX96724_LF_VAL(val, link) ((val >> ((link % 2) * 4)) & GENMASK(2, 0))

#define MAX96724_PU_LF (0xE0)
#define MAX96724_PU_LF_FIELD(line) BIT(line)

static struct max9x_serdes_rate_table max96724_rx_rates[] = {
	{ .val = 1, .freq_mhz = 3000 }, // 3 Gbps
	{ .val = 2, .freq_mhz = 6000 }, // 6 Gbps
};

static struct max9x_serdes_rate_table max96724_tx_rates[] = {
	{ .val = 0, .freq_mhz = 187 }, // 187.5 Mbps
};

// See: Errata for MAX96724/MAX96724F/MAX96724R (Rev. 1) document
//      https://www.analog.com/media/en/technical-documentation/data-sheets/max96724-f-r-rev1-b-0a-errata.pdf
static const struct reg_sequence max96724_errata_rev1[] = {
	// Errata #5 - GMSL2 Link requires register writes for robust 6 Gbps operation
	{ 0x1449, 0x75, },
	{ 0x1549, 0x75, },
	{ 0x1649, 0x75, },
	{ 0x1749, 0x75, },
};

#endif /* _MAX96724_H_ */

