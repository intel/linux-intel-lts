/* SPDX-License-Identifier: MIT
 *
 * Copyright © 2019 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *
 */
#ifndef __KMB_DSI_H__
#define __KMB_DSI_H__

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include "kmb_drv.h"

/* MIPI TX CFG*/
#define MIPI_TX_LANE_DATA_RATE_MBPS 891
#define MIPI_TX_REF_CLK_KHZ         24000
#define MIPI_TX_CFG_CLK_KHZ         24000
#define MIPI_TX_BPP		    24

/* DPHY Tx test codes*/
#define TEST_CODE_FSM_CONTROL				0x03
#define TEST_CODE_MULTIPLE_PHY_CTRL			0x0C
#define TEST_CODE_PLL_PROPORTIONAL_CHARGE_PUMP_CTRL	0x0E
#define TEST_CODE_PLL_INTEGRAL_CHARGE_PUMP_CTRL		0x0F
#define TEST_CODE_PLL_VCO_CTRL				0x12
#define TEST_CODE_PLL_GMP_CTRL				0x13
#define TEST_CODE_PLL_PHASE_ERR_CTRL			0x14
#define TEST_CODE_PLL_LOCK_FILTER			0x15
#define TEST_CODE_PLL_UNLOCK_FILTER			0x16
#define TEST_CODE_PLL_INPUT_DIVIDER			0x17
#define TEST_CODE_PLL_FEEDBACK_DIVIDER			0x18
#define   PLL_FEEDBACK_DIVIDER_HIGH			(1 << 7)
#define TEST_CODE_PLL_OUTPUT_CLK_SEL			0x19
#define   PLL_N_OVR_EN					(1 << 4)
#define   PLL_M_OVR_EN					(1 << 5)
#define TEST_CODE_VOD_LEVEL				0x24
#define TEST_CODE_PLL_CHARGE_PUMP_BIAS			0x1C
#define TEST_CODE_PLL_LOCK_DETECTOR			0x1D
#define TEST_CODE_HS_FREQ_RANGE_CFG			0x44
#define TEST_CODE_PLL_ANALOG_PROG			0x1F
#define TEST_CODE_SLEW_RATE_OVERRIDE_CTRL		0xA0
#define TEST_CODE_SLEW_RATE_DDL_LOOP_CTRL		0xA3
#define TEST_CODE_SLEW_RATE_DDL_CYCLES			0xA4

/* DPHY params */
#define PLL_N_MIN	0
#define PLL_N_MAX	15
#define PLL_M_MIN	62
#define PLL_M_MAX	623
#define PLL_FVCO_MAX	1250

#define TIMEOUT		600

#define MIPI_TX_FRAME_GEN				4
#define MIPI_TX_FRAME_GEN_SECTIONS			4
#define MIPI_CTRL_VIRTUAL_CHANNELS			4
#define MIPI_D_LANES_PER_DPHY				2
#define MIPI_CTRL_2LANE_MAX_MC_FIFO_LOC			255
#define MIPI_CTRL_4LANE_MAX_MC_FIFO_LOC			511
/* 2 Data Lanes per D-PHY */
#define MIPI_DPHY_D_LANES				2
#define MIPI_DPHY_DEFAULT_BIT_RATES			63

#define to_kmb_connector(x) container_of(x, struct kmb_connector, base)
#define to_kmb_host(x) container_of(x, struct kmb_dsi_host, base)
#define to_kmb_dsi(x) container_of(x, struct kmb_dsi, base)

struct kmb_connector;
struct kmb_dsi_host;

struct kmb_dsi {
	struct drm_encoder base;
	struct kmb_connector *attached_connector;
	struct kmb_dsi_host *dsi_host;
	struct drm_bridge *bridge;
};

struct kmb_dsi_host {
	struct mipi_dsi_host *base;
	struct kmb_dsi *kmb_dsi;
	struct mipi_dsi_device *device;
};

struct kmb_connector {
	struct drm_connector base;
	struct drm_encoder *encoder;
	struct drm_display_mode *fixed_mode;
};

/* DPHY Tx test codes */

enum mipi_ctrl_num {
	MIPI_CTRL0 = 0,
	MIPI_CTRL1,
	MIPI_CTRL2,
	MIPI_CTRL3,
	MIPI_CTRL4,
	MIPI_CTRL5,
	MIPI_CTRL6,
	MIPI_CTRL7,
	MIPI_CTRL8,
	MIPI_CTRL9,
	MIPI_CTRL_NA
};

enum mipi_dphy_num {
	MIPI_DPHY0 = 0,
	MIPI_DPHY1,
	MIPI_DPHY2,
	MIPI_DPHY3,
	MIPI_DPHY4,
	MIPI_DPHY5,
	MIPI_DPHY6,
	MIPI_DPHY7,
	MIPI_DPHY8,
	MIPI_DPHY9,
	MIPI_DPHY_NA
};

enum mipi_dir {
	MIPI_RX,
	MIPI_TX
};

enum mipi_ctrl_type {
	MIPI_DSI,
	MIPI_CSI
};

enum mipi_data_if {
	MIPI_IF_DMA,
	MIPI_IF_PARALLEL
};

enum mipi_data_mode {
	MIPI_DATA_MODE0,
	MIPI_DATA_MODE1,
	MIPI_DATA_MODE2,
	MIPI_DATA_MODE3
};

enum mipi_dsi_video_mode {
	DSI_VIDEO_MODE_NO_BURST_PULSE,
	DSI_VIDEO_MODE_NO_BURST_EVENT,
	DSI_VIDEO_MODE_BURST
};

enum mipi_dsi_blanking_mode {
	TRANSITION_TO_LOW_POWER,
	SEND_BLANK_PACKET
};

enum mipi_dsi_eotp {
	DSI_EOTP_DISABLED,
	DSI_EOTP_ENABLES
};

enum mipi_dsi_data_type {
	DSI_SP_DT_RESERVED_00 = 0x00,
	DSI_SP_DT_VSYNC_START = 0x01,
	DSI_SP_DT_COLOR_MODE_OFF = 0x02,
	DSI_SP_DT_GENERIC_SHORT_WR = 0x03,
	DSI_SP_DT_GENERIC_RD = 0x04,
	DSI_SP_DT_DCS_SHORT_WR = 0x05,
	DSI_SP_DT_DCS_RD = 0x06,
	DSI_SP_DT_EOTP = 0x08,
	DSI_LP_DT_NULL = 0x09,
	DSI_LP_DT_RESERVED_0A = 0x0a,
	DSI_LP_DT_RESERVED_0B = 0x0b,
	DSI_LP_DT_LPPS_YCBCR422_20B = 0x0c,
	DSI_LP_DT_PPS_RGB101010_30B = 0x0d,
	DSI_LP_DT_PPS_RGB565_16B = 0x0e,
	DSI_LP_DT_RESERVED_0F = 0x0f,

	DSI_SP_DT_RESERVED_10 = 0x10,
	DSI_SP_DT_VSYNC_END = 0x11,
	DSI_SP_DT_COLOR_MODE_ON = 0x12,
	DSI_SP_DT_GENERIC_SHORT_WR_1PAR = 0x13,
	DSI_SP_DT_GENERIC_RD_1PAR = 0x14,
	DSI_SP_DT_DCS_SHORT_WR_1PAR = 0x15,
	DSI_SP_DT_RESERVED_16 = 0x16,
	DSI_SP_DT_RESERVED_17 = 0x17,
	DSI_SP_DT_RESERVED_18 = 0x18,
	DSI_LP_DT_BLANK = 0x19,
	DSI_LP_DT_RESERVED_1A = 0x1a,
	DSI_LP_DT_RESERVED_1B = 0x1b,
	DSI_LP_DT_PPS_YCBCR422_24B = 0x1c,
	DSI_LP_DT_PPS_RGB121212_36B = 0x1d,
	DSI_LP_DT_PPS_RGB666_18B = 0x1e,
	DSI_LP_DT_RESERVED_1F = 0x1f,

	DSI_SP_DT_RESERVED_20 = 0x20,
	DSI_SP_DT_HSYNC_START = 0x21,
	DSI_SP_DT_SHUT_DOWN_PERIPH_CMD = 0x22,
	DSI_SP_DT_GENERIC_SHORT_WR_2PAR = 0x23,
	DSI_SP_DT_GENERIC_RD_2PAR = 0x24,
	DSI_SP_DT_RESERVED_25 = 0x25,
	DSI_SP_DT_RESERVED_26 = 0x26,
	DSI_SP_DT_RESERVED_27 = 0x27,
	DSI_SP_DT_RESERVED_28 = 0x28,
	DSI_LP_DT_GENERIC_LONG_WR = 0x29,
	DSI_LP_DT_RESERVED_2A = 0x2a,
	DSI_LP_DT_RESERVED_2B = 0x2b,
	DSI_LP_DT_PPS_YCBCR422_16B = 0x2c,
	DSI_LP_DT_RESERVED_2D = 0x2d,
	DSI_LP_DT_LPPS_RGB666_18B = 0x2e,
	DSI_LP_DT_RESERVED_2F = 0x2f,

	DSI_SP_DT_RESERVED_30 = 0x30,
	DSI_SP_DT_HSYNC_END = 0x31,
	DSI_SP_DT_TURN_ON_PERIPH_CMD = 0x32,
	DSI_SP_DT_RESERVED_33 = 0x33,
	DSI_SP_DT_RESERVED_34 = 0x34,
	DSI_SP_DT_RESERVED_35 = 0x35,
	DSI_SP_DT_RESERVED_36 = 0x36,
	DSI_SP_DT_SET_MAX_RETURN_PKT_SIZE = 0x37,
	DSI_SP_DT_RESERVED_38 = 0x38,
	DSI_LP_DT_DSC_LONG_WR = 0x39,
	DSI_LP_DT_RESERVED_3A = 0x3a,
	DSI_LP_DT_RESERVED_3B = 0x3b,
	DSI_LP_DT_RESERVED_3C = 0x3c,
	DSI_LP_DT_PPS_YCBCR420_12B = 0x3d,
	DSI_LP_DT_PPS_RGB888_24B = 0x3e,
	DSI_LP_DT_RESERVED_3F = 0x3f
};

enum mipi_tx_hs_tp_sel {
	MIPI_TX_HS_TP_WHOLE_FRAME_COLOR0 = 0,
	MIPI_TX_HS_TP_WHOLE_FRAME_COLOR1,
	MIPI_TX_HS_TP_V_STRIPES,
	MIPI_TX_HS_TP_H_STRIPES,
};

enum dphy_mode {
	MIPI_DPHY_SLAVE = 0,
	MIPI_DPHY_MASTER
};

enum dphy_tx_fsm {
	DPHY_TX_POWERDWN = 0,
	DPHY_TX_BGPON,
	DPHY_TX_TERMCAL,
	DPHY_TX_TERMCALUP,
	DPHY_TX_OFFSETCAL,
	DPHY_TX_LOCK,
	DPHY_TX_SRCAL,
	DPHY_TX_IDLE,
	DPHY_TX_ULP,
	DPHY_TX_LANESTART,
	DPHY_TX_CLKALIGN,
	DPHY_TX_DDLTUNNING,
	DPHY_TX_ULP_FORCE_PLL,
	DPHY_TX_LOCK_LOSS
};

struct mipi_data_type_params {
	uint8_t size_constraint_pixels;
	uint8_t size_constraint_bytes;
	uint8_t pixels_per_pclk;
	uint8_t bits_per_pclk;
};

struct mipi_tx_dsi_cfg {
	uint8_t hfp_blank_en;	/*horizontal front porch blanking enable */
	uint8_t eotp_en;	/*End of transmission packet enable */
	/*last vertical front porch blanking mode */
	uint8_t lpm_last_vfp_line;
	/*first vertical sync active blanking mode */
	uint8_t lpm_first_vsa_line;
	uint8_t sync_pulse_eventn;	/*sync type */
	uint8_t hfp_blanking;	/*horizontal front porch blanking mode */
	uint8_t hbp_blanking;	/*horizontal back porch blanking mode */
	uint8_t hsa_blanking;	/*horizontal sync active blanking mode */
	uint8_t v_blanking;	/*vertical timing blanking mode */
};

struct mipi_tx_frame_section_cfg {
	uint32_t dma_v_stride;
	uint16_t dma_v_scale_cfg;
	uint16_t width_pixels;
	uint16_t height_lines;
	uint8_t dma_packed;
	uint8_t bpp;
	uint8_t bpp_unpacked;
	uint8_t dma_h_stride;
	uint8_t data_type;
	uint8_t data_mode;
	uint8_t dma_flip_rotate_sel;
};

struct mipi_tx_frame_timing_cfg {
	uint32_t bpp;
	uint32_t lane_rate_mbps;
	uint32_t hsync_width;
	uint32_t h_backporch;
	uint32_t h_frontporch;
	uint32_t h_active;
	uint16_t vsync_width;
	uint16_t v_backporch;
	uint16_t v_frontporch;
	uint16_t v_active;
	uint8_t active_lanes;
};

struct mipi_tx_frame_sect_phcfg {
	uint32_t wc;
	enum mipi_data_mode data_mode;
	enum mipi_dsi_data_type data_type;
	uint8_t vchannel;
	uint8_t dma_packed;
};

struct mipi_tx_frame_cfg {
	struct mipi_tx_frame_section_cfg *sections[MIPI_TX_FRAME_GEN_SECTIONS];
	uint32_t hsync_width;	/*in pixels */
	uint32_t h_backporch;	/*in pixels */
	uint32_t h_frontporch;	/*in pixels */
	uint16_t vsync_width;	/*in lines */
	uint16_t v_backporch;	/*in lines */
	uint16_t v_frontporch;	/*in lines */
};

struct mipi_tx_ctrl_cfg {
	struct mipi_tx_frame_cfg *frames[MIPI_TX_FRAME_GEN];
	struct mipi_tx_dsi_cfg *tx_dsi_cfg;
	uint8_t line_sync_pkt_en;
	uint8_t line_counter_active;
	uint8_t frame_counter_active;
	uint8_t tx_hsclkkidle_cnt;
	uint8_t tx_hsexit_cnt;
	uint8_t tx_crc_en;
	uint8_t tx_hact_wait_stop;
	uint8_t tx_always_use_hact;
	uint8_t tx_wait_trig;
	uint8_t tx_wait_all_sect;
};

/*configuration structure for MIPI control */
struct mipi_ctrl_cfg {
	/* controller index : CTRL6 for connecting to LCD */
	uint8_t index;
	uint8_t type;		/* controller type : MIPI_DSI */
	uint8_t dir;		/* controller direction : MIPI_TX */
	uint8_t active_lanes;	/* # active lanes per controller 2/4 */
	uint32_t lane_rate_mbps;	/*MBPS */
	uint32_t ref_clk_khz;
	uint32_t cfg_clk_khz;
	uint32_t data_if;	/*MIPI_IF_DMA or MIPI_IF_PARALLEL */
	struct mipi_tx_ctrl_cfg tx_ctrl_cfg;
};

/* Structure for storing user specified interrupts that are enabled */
union mipi_irq_cfg {
	uint8_t value;
	struct {
		uint8_t line_compare:1;
		uint8_t dma_event:1;
		uint8_t frame_done:1;
		uint8_t ctrl_error:1;
		uint8_t dphy_error:1;
	} irq_cfg;
};

struct drm_bridge *kmb_dsi_host_bridge_init(struct device *dev);
int kmb_dsi_init(struct drm_device *dev, struct drm_bridge *bridge);
void kmb_plane_destroy(struct drm_plane *plane);
void mipi_tx_handle_irqs(struct kmb_drm_private *dev_p);
void kmb_dsi_host_unregister(void);
int kmb_dsi_hw_init(struct drm_device *dev, struct drm_display_mode *mode);
#endif /* __KMB_DSI_H__ */
