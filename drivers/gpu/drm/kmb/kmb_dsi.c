// SPDX-License-Identifier: GPL-2.0
/*
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
#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_edid.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_bridge.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/gpio/consumer.h>
#include "kmb_drv.h"
#include "kmb_regs.h"
#include "kmb_dsi.h"
#include <linux/fs.h>
#include <linux/buffer_head.h>

static int hw_initialized;
#define IMAGE_PATH "/home/root/1280x720.pnm"
#define MIPI_TX_TEST_PATTERN_GENERATION

#define IMG_HEIGHT_LINES  720
#define IMG_WIDTH_PX      1280
#define LCD_BYTESPP       1

/*MIPI TX CFG*/
#define MIPI_TX_ACTIVE_LANES  2
//#define MIPI_TX_LANE_DATA_RATE_MBPS 1782
#define MIPI_TX_LANE_DATA_RATE_MBPS 891
//#define MIPI_TX_LANE_DATA_RATE_MBPS 80
#define MIPI_TX_REF_CLK_KHZ         24000
#define MIPI_TX_CFG_CLK_KHZ         24000

/*DPHY Tx test codes*/
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

/* D-Phy params  */
#define PLL_N_MIN	0
#define PLL_N_MAX	15
#define PLL_M_MIN	62
#define PLL_M_MAX	623
#define PLL_FVCO_MAX	1250

#define TIMEOUT		600
static struct mipi_dsi_host *dsi_host;
static struct mipi_dsi_device *dsi_device;

/*
 * These are added here only temporarily for testing,
 * these will eventually go to the device tree sections,
 * and can be used as a refernce later for device tree additions
 */
struct mipi_tx_frame_section_cfg mipi_tx_frame0_sect_cfg = {
	.width_pixels = IMG_WIDTH_PX,
	.height_lines = IMG_HEIGHT_LINES,
	.data_type = DSI_LP_DT_PPS_RGB888_24B,
	//.data_mode = MIPI_DATA_MODE1,
	.data_mode = MIPI_DATA_MODE0,
	.dma_packed = 1
};

struct mipi_tx_frame_cfg mipitx_frame0_cfg = {
	.sections[0] = &mipi_tx_frame0_sect_cfg,
	.sections[1] = NULL,
	.sections[2] = NULL,
	.sections[3] = NULL,
	.vsync_width = 5,
	.v_backporch = 20,
	.v_frontporch = 5,
	.hsync_width = 40,
	.h_backporch = 220,
	.h_frontporch = 110,
};

struct mipi_tx_dsi_cfg mipitx_dsi_cfg = {
	.hfp_blank_en = 0,
	.eotp_en = 0,
	.lpm_last_vfp_line = 0,
	.lpm_first_vsa_line = 0,
	.sync_pulse_eventn = DSI_VIDEO_MODE_NO_BURST_EVENT,
	.hfp_blanking = SEND_BLANK_PACKET,
	.hbp_blanking = SEND_BLANK_PACKET,
	.hsa_blanking = SEND_BLANK_PACKET,
	.v_blanking = SEND_BLANK_PACKET,
};

struct mipi_ctrl_cfg mipi_tx_init_cfg = {
	.index = MIPI_CTRL6,
	.type = MIPI_DSI,
	.dir = MIPI_TX,
	.active_lanes = MIPI_TX_ACTIVE_LANES,
	.lane_rate_mbps = MIPI_TX_LANE_DATA_RATE_MBPS,
	.ref_clk_khz = MIPI_TX_REF_CLK_KHZ,
	.cfg_clk_khz = MIPI_TX_CFG_CLK_KHZ,
//      .data_if = MIPI_IF_PARALLEL,
	.data_if = MIPI_IF_DMA,
	.tx_ctrl_cfg = {
			.frames[0] = &mipitx_frame0_cfg,
			.frames[1] = NULL,
			.frames[2] = NULL,
			.frames[3] = NULL,
			.tx_dsi_cfg = &mipitx_dsi_cfg,
			.line_sync_pkt_en = 0,
			.line_counter_active = 0,
			.frame_counter_active = 0,
			.tx_always_use_hact = 1,
			.tx_hact_wait_stop = 1,
			}

};

u8 *iBuf;

struct mipi_hs_freq_range_cfg {
	uint16_t default_bit_rate_mbps;
	uint8_t hsfreqrange_code;
};

static struct mipi_hs_freq_range_cfg
	mipi_hs_freq_range[MIPI_DPHY_DEFAULT_BIT_RATES] = {
	{.default_bit_rate_mbps = 80, .hsfreqrange_code = 0x00},
	{.default_bit_rate_mbps = 90, .hsfreqrange_code = 0x10},
	{.default_bit_rate_mbps = 100, .hsfreqrange_code = 0x20},
	{.default_bit_rate_mbps = 110, .hsfreqrange_code = 0x30},
	{.default_bit_rate_mbps = 120, .hsfreqrange_code = 0x01},
	{.default_bit_rate_mbps = 130, .hsfreqrange_code = 0x11},
	{.default_bit_rate_mbps = 140, .hsfreqrange_code = 0x21},
	{.default_bit_rate_mbps = 150, .hsfreqrange_code = 0x31},
	{.default_bit_rate_mbps = 160, .hsfreqrange_code = 0x02},
	{.default_bit_rate_mbps = 170, .hsfreqrange_code = 0x12},
	{.default_bit_rate_mbps = 180, .hsfreqrange_code = 0x22},
	{.default_bit_rate_mbps = 190, .hsfreqrange_code = 0x32},
	{.default_bit_rate_mbps = 205, .hsfreqrange_code = 0x03},
	{.default_bit_rate_mbps = 220, .hsfreqrange_code = 0x13},
	{.default_bit_rate_mbps = 235, .hsfreqrange_code = 0x23},
	{.default_bit_rate_mbps = 250, .hsfreqrange_code = 0x33},
	{.default_bit_rate_mbps = 275, .hsfreqrange_code = 0x04},
	{.default_bit_rate_mbps = 300, .hsfreqrange_code = 0x14},
	{.default_bit_rate_mbps = 325, .hsfreqrange_code = 0x25},
	{.default_bit_rate_mbps = 350, .hsfreqrange_code = 0x35},
	{.default_bit_rate_mbps = 400, .hsfreqrange_code = 0x05},
	{.default_bit_rate_mbps = 450, .hsfreqrange_code = 0x16},
	{.default_bit_rate_mbps = 500, .hsfreqrange_code = 0x26},
	{.default_bit_rate_mbps = 550, .hsfreqrange_code = 0x37},
	{.default_bit_rate_mbps = 600, .hsfreqrange_code = 0x07},
	{.default_bit_rate_mbps = 650, .hsfreqrange_code = 0x18},
	{.default_bit_rate_mbps = 700, .hsfreqrange_code = 0x28},
	{.default_bit_rate_mbps = 750, .hsfreqrange_code = 0x39},
	{.default_bit_rate_mbps = 800, .hsfreqrange_code = 0x09},
	{.default_bit_rate_mbps = 850, .hsfreqrange_code = 0x19},
	{.default_bit_rate_mbps = 900, .hsfreqrange_code = 0x29},
	{.default_bit_rate_mbps = 1000, .hsfreqrange_code = 0x0A},
	{.default_bit_rate_mbps = 1050, .hsfreqrange_code = 0x1A},
	{.default_bit_rate_mbps = 1100, .hsfreqrange_code = 0x2A},
	{.default_bit_rate_mbps = 1150, .hsfreqrange_code = 0x3B},
	{.default_bit_rate_mbps = 1200, .hsfreqrange_code = 0x0B},
	{.default_bit_rate_mbps = 1250, .hsfreqrange_code = 0x1B},
	{.default_bit_rate_mbps = 1300, .hsfreqrange_code = 0x2B},
	{.default_bit_rate_mbps = 1350, .hsfreqrange_code = 0x3C},
	{.default_bit_rate_mbps = 1400, .hsfreqrange_code = 0x0C},
	{.default_bit_rate_mbps = 1450, .hsfreqrange_code = 0x1C},
	{.default_bit_rate_mbps = 1500, .hsfreqrange_code = 0x2C},
	{.default_bit_rate_mbps = 1550, .hsfreqrange_code = 0x3D},
	{.default_bit_rate_mbps = 1600, .hsfreqrange_code = 0x0D},
	{.default_bit_rate_mbps = 1650, .hsfreqrange_code = 0x1D},
	{.default_bit_rate_mbps = 1700, .hsfreqrange_code = 0x2E},
	{.default_bit_rate_mbps = 1750, .hsfreqrange_code = 0x3E},
	{.default_bit_rate_mbps = 1800, .hsfreqrange_code = 0x0E},
	{.default_bit_rate_mbps = 1850, .hsfreqrange_code = 0x1E},
	{.default_bit_rate_mbps = 1900, .hsfreqrange_code = 0x2F},
	{.default_bit_rate_mbps = 1950, .hsfreqrange_code = 0x3F},
	{.default_bit_rate_mbps = 2000, .hsfreqrange_code = 0x0F},
	{.default_bit_rate_mbps = 2050, .hsfreqrange_code = 0x40},
	{.default_bit_rate_mbps = 2100, .hsfreqrange_code = 0x41},
	{.default_bit_rate_mbps = 2150, .hsfreqrange_code = 0x42},
	{.default_bit_rate_mbps = 2200, .hsfreqrange_code = 0x43},
	{.default_bit_rate_mbps = 2250, .hsfreqrange_code = 0x44},
	{.default_bit_rate_mbps = 2300, .hsfreqrange_code = 0x45},
	{.default_bit_rate_mbps = 2350, .hsfreqrange_code = 0x46},
	{.default_bit_rate_mbps = 2400, .hsfreqrange_code = 0x47},
	{.default_bit_rate_mbps = 2450, .hsfreqrange_code = 0x48},
	{.default_bit_rate_mbps = 2500, .hsfreqrange_code = 0x49}
};

union mipi_irq_cfg int_cfg = {
	.irq_cfg.frame_done = 1,
	.irq_cfg.ctrl_error = 1,
};

static enum drm_mode_status
kmb_dsi_mode_valid(struct drm_connector *connector,
		   struct drm_display_mode *mode)
{
	return MODE_OK;
}

static int kmb_dsi_get_modes(struct drm_connector *connector)
{
	int num_modes = 0;

	num_modes = drm_add_modes_noedid(connector,
				 connector->dev->mode_config.max_width,
				 connector->dev->mode_config.max_height);
	return num_modes;
}

static void kmb_dsi_connector_destroy(struct drm_connector *connector)
{
	struct kmb_connector *kmb_connector = to_kmb_connector(connector);

	drm_connector_cleanup(connector);
	kfree(kmb_connector);
}

static void kmb_dsi_encoder_destroy(struct drm_encoder *encoder)
{
	struct kmb_dsi *kmb_dsi = to_kmb_dsi(encoder);

	drm_encoder_cleanup(encoder);
	kfree(kmb_dsi);
}

static const struct drm_encoder_funcs kmb_dsi_funcs = {
	.destroy = kmb_dsi_encoder_destroy,
};

static const struct
drm_connector_helper_funcs kmb_dsi_connector_helper_funcs = {
	.get_modes = kmb_dsi_get_modes,
	.mode_valid = kmb_dsi_mode_valid,
};

static const struct drm_connector_funcs kmb_dsi_connector_funcs = {
	.destroy = kmb_dsi_connector_destroy,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
};

static ssize_t kmb_dsi_host_transfer(struct mipi_dsi_host *host,
				     const struct mipi_dsi_msg *msg)
{
	return 0;
}

static int kmb_dsi_host_attach(struct mipi_dsi_host *host,
			       struct mipi_dsi_device *dev)
{
	return 0;
}

static int kmb_dsi_host_detach(struct mipi_dsi_host *host,
			       struct mipi_dsi_device *dev)
{
	return 0;
}

static const struct mipi_dsi_host_ops kmb_dsi_host_ops = {
	.attach = kmb_dsi_host_attach,
	.detach = kmb_dsi_host_detach,
	.transfer = kmb_dsi_host_transfer,
};

static struct kmb_dsi_host *kmb_dsi_host_init(struct drm_device *drm,
					      struct kmb_dsi *kmb_dsi)
{
	struct kmb_dsi_host *host;

	DRM_INFO("%s : %d\n", __func__, __LINE__);
	host = kzalloc(sizeof(*host), GFP_KERNEL);
	if (!host)
		return NULL;

	host->base = dsi_host;
	host->base->ops = &kmb_dsi_host_ops;
	host->kmb_dsi = kmb_dsi;

	host->base->dev = drm->dev;

	dsi_device->host = host->base;
	host->device = dsi_device;
	return host;
}

struct drm_bridge *kmb_dsi_host_bridge_init(struct device *dev)
{
	struct device_node *encoder_node;
	struct drm_bridge *bridge;

	/* Create and register MIPI DSI host */
	if (!dsi_host) {
		dsi_host = kzalloc(sizeof(*dsi_host), GFP_KERNEL);
		if (!dsi_host)
			return ERR_PTR(-ENOMEM);

		dsi_host->ops = &kmb_dsi_host_ops;

		if (!dsi_device) {
			dsi_device = kzalloc(sizeof(*dsi_device), GFP_KERNEL);
			if (!dsi_device) {
				kfree(dsi_host);
				return ERR_PTR(-ENOMEM);
			}
		}

		dsi_host->dev = dev;
		mipi_dsi_host_register(dsi_host);
	}
#ifndef FCCTEST
	/* find ADV7535 node and initialize it */
	DRM_INFO("trying to get bridge info %pOF\n", dev->of_node);
	encoder_node = of_parse_phandle(dev->of_node, "encoder-slave", 0);
	DRM_INFO("encoder node =  %pOF\n", encoder_node);
	if (!encoder_node) {
		DRM_ERROR("failed to get bridge info from DT\n");
		return ERR_PTR(-EINVAL);
	}

	/* Locate drm bridge from the hdmi encoder DT node */
	bridge = of_drm_find_bridge(encoder_node);
	of_node_put(encoder_node);
	if (!bridge) {
		DRM_INFO("wait for external bridge driver DT\n");
		return ERR_PTR(-EPROBE_DEFER);
	}
#endif
	return bridge;
}

void dsi_host_unregister(void)
{
	mipi_dsi_host_unregister(dsi_host);
}

u32 mipi_get_datatype_params(u32 data_type, u32 data_mode,
			     struct mipi_data_type_params *params)
{
	struct mipi_data_type_params data_type_parameters;

	switch (data_type) {
	case DSI_LP_DT_PPS_YCBCR420_12B:
		data_type_parameters.size_constraint_pixels = 2;
		data_type_parameters.size_constraint_bytes = 3;
		switch (data_mode) {
			/* case 0 not supported according to MDK */
		case 1:
		case 2:
		case 3:
			data_type_parameters.pixels_per_pclk = 2;
			data_type_parameters.bits_per_pclk = 24;
			break;
		default:
			DRM_ERROR("DSI: Invalid data_mode %d\n", data_mode);
			return -EINVAL;
		};
		break;
	case DSI_LP_DT_PPS_YCBCR422_16B:
		data_type_parameters.size_constraint_pixels = 2;
		data_type_parameters.size_constraint_bytes = 4;
		switch (data_mode) {
			/* case 0 and 1 not supported according to MDK */
		case 2:
			data_type_parameters.pixels_per_pclk = 1;
			data_type_parameters.bits_per_pclk = 16;
			break;
		case 3:
			data_type_parameters.pixels_per_pclk = 2;
			data_type_parameters.bits_per_pclk = 32;
			break;
		default:
			DRM_ERROR("DSI: Invalid data_mode %d\n", data_mode);
			return -EINVAL;
		};
		break;
	case DSI_LP_DT_LPPS_YCBCR422_20B:
	case DSI_LP_DT_PPS_YCBCR422_24B:
		data_type_parameters.size_constraint_pixels = 2;
		data_type_parameters.size_constraint_bytes = 6;
		switch (data_mode) {
			/* case 0 not supported according to MDK */
		case 1:
		case 2:
		case 3:
			data_type_parameters.pixels_per_pclk = 1;
			data_type_parameters.bits_per_pclk = 24;
			break;
		default:
			DRM_ERROR("DSI: Invalid data_mode %d\n", data_mode);
			return -EINVAL;
		};
		break;
	case DSI_LP_DT_PPS_RGB565_16B:
		data_type_parameters.size_constraint_pixels = 1;
		data_type_parameters.size_constraint_bytes = 2;
		switch (data_mode) {
		case 0:
		case 1:
			data_type_parameters.pixels_per_pclk = 1;
			data_type_parameters.bits_per_pclk = 16;
			break;
		case 2:
		case 3:
			data_type_parameters.pixels_per_pclk = 2;
			data_type_parameters.bits_per_pclk = 32;
			break;
		default:
			DRM_ERROR("DSI: Invalid data_mode %d\n", data_mode);
			return -EINVAL;
		};
		break;
	case DSI_LP_DT_PPS_RGB666_18B:
		data_type_parameters.size_constraint_pixels = 4;
		data_type_parameters.size_constraint_bytes = 9;
		data_type_parameters.bits_per_pclk = 18;
		data_type_parameters.pixels_per_pclk = 1;
		break;
	case DSI_LP_DT_LPPS_RGB666_18B:
	case DSI_LP_DT_PPS_RGB888_24B:
		data_type_parameters.size_constraint_pixels = 1;
		data_type_parameters.size_constraint_bytes = 3;
		data_type_parameters.bits_per_pclk = 24;
		data_type_parameters.pixels_per_pclk = 1;
		break;
	case DSI_LP_DT_PPS_RGB101010_30B:
		data_type_parameters.size_constraint_pixels = 4;
		data_type_parameters.size_constraint_bytes = 15;
		data_type_parameters.bits_per_pclk = 30;
		data_type_parameters.pixels_per_pclk = 1;
		break;

	default:
		DRM_ERROR("DSI: Invalid data_type %d\n", data_type);
		return -EINVAL;
	}

	*params = data_type_parameters;
	return 0;
}

static u32 compute_wc(u32 width_px, u8 size_constr_p, u8 size_constr_b)
{
	/* calculate the word count for each long packet */
	return (((width_px / size_constr_p) * size_constr_b) & 0xffff);
}

static u32 compute_unpacked_bytes(u32 wc, u8 bits_per_pclk)
{
	/*number of PCLK cycles needed to transfer a line */
	/* with each PCLK cycle, 4 Bytes are sent through the PPL module */
	return ((wc * 8) / bits_per_pclk) * 4;
}

static u32 mipi_tx_fg_section_cfg_regs(struct kmb_drm_private *dev_p,
				       u8 frame_id,
				       u8 section, u32 height_lines,
				       u32 unpacked_bytes,
				       struct mipi_tx_frame_sect_phcfg *ph_cfg)
{
	u32 cfg = 0;
	u32 ctrl_no = MIPI_CTRL6;
	u32 reg_adr;

	/*frame section packet header */
	/*word count */
	cfg = (ph_cfg->wc & MIPI_TX_SECT_WC_MASK) << 0;	/* bits [15:0] */
	/*data type */
	cfg |= ((ph_cfg->data_type & MIPI_TX_SECT_DT_MASK)
		<< MIPI_TX_SECT_DT_SHIFT);	/* bits [21:16] */
	/* virtual channel */
	cfg |= ((ph_cfg->vchannel & MIPI_TX_SECT_VC_MASK)
		<< MIPI_TX_SECT_VC_SHIFT);	/* bits [23:22] */
	/* data mode */
	cfg |= ((ph_cfg->data_mode & MIPI_TX_SECT_DM_MASK)
		<< MIPI_TX_SECT_DM_SHIFT);	/* bits [24:25] */
	cfg |= MIPI_TX_SECT_DMA_PACKED;
	DRM_INFO("%s : %d ctrl=%d frame_id=%d section=%d cfg=%x\n",
		 __func__, __LINE__, ctrl_no, frame_id, section, cfg);
	kmb_write_mipi(dev_p, (MIPI_TXm_HS_FGn_SECTo_PH(ctrl_no, frame_id,
							section)), cfg);
	/*unpacked bytes */
	/*there are 4 frame generators and each fg has 4 sections
	 *there are 2 registers for unpacked bytes -
	 *# bytes each section occupies in memory
	 *REG_UNPACKED_BYTES0: [15:0]-BYTES0, [31:16]-BYTES1
	 *REG_UNPACKED_BYTES1: [15:0]-BYTES2, [31:16]-BYTES3
	 */
	reg_adr = MIPI_TXm_HS_FGn_SECT_UNPACKED_BYTES0(ctrl_no, frame_id)
	    + (section / 2) * 4;
	kmb_write_bits_mipi(dev_p, reg_adr, (section % 2) * 16, 16,
			    unpacked_bytes);

	/* line config */
	reg_adr = MIPI_TXm_HS_FGn_SECTo_LINE_CFG(ctrl_no, frame_id, section);
	kmb_write_mipi(dev_p, reg_adr, height_lines);
	return 0;
}

static u32 mipi_tx_fg_section_cfg(struct kmb_drm_private *dev_p, u8 frame_id,
				  u8 section,
				  struct mipi_tx_frame_section_cfg *frame_scfg,
				  u32 *bits_per_pclk, u32 *wc)
{
	u32 ret = 0;
	u32 unpacked_bytes;
	struct mipi_data_type_params data_type_parameters;
	struct mipi_tx_frame_sect_phcfg ph_cfg;

	ret = mipi_get_datatype_params(frame_scfg->data_type,
				       frame_scfg->data_mode,
				       &data_type_parameters);
	if (ret)
		return ret;
	/*
	 * packet width has to be a multiple of the minimum packet width
	 * (in pixels) set for each data type
	 */
	if (frame_scfg->width_pixels %
	    data_type_parameters.size_constraint_pixels != 0)
		return -EINVAL;

	*wc = compute_wc(frame_scfg->width_pixels,
			 data_type_parameters.size_constraint_pixels,
			 data_type_parameters.size_constraint_bytes);

	unpacked_bytes =
	    compute_unpacked_bytes(*wc, data_type_parameters.bits_per_pclk);

	ph_cfg.wc = *wc;
	ph_cfg.data_mode = frame_scfg->data_mode;
	ph_cfg.data_type = frame_scfg->data_type;
	ph_cfg.vchannel = frame_id;

	mipi_tx_fg_section_cfg_regs(dev_p, frame_id, section,
				    frame_scfg->height_lines,
				    unpacked_bytes, &ph_cfg);

	/*caller needs bits_per_clk for additional caluclations */
	*bits_per_pclk = data_type_parameters.bits_per_pclk;
	return 0;
}

static void mipi_tx_fg_cfg_regs(struct kmb_drm_private *dev_p,
				u8 frame_gen,
				struct mipi_tx_frame_timing_cfg *fg_cfg)
{
	u32 sysclk;
	/*float ppl_llp_ratio; */
	u32 ppl_llp_ratio;
	u32 ctrl_no = MIPI_CTRL6, reg_adr, val, offset;

	/*Get system clock for blanking period cnfigurations */
	/*TODO need to get system clock from clock driver */
	/* Assume 700 Mhz system clock for now */
	sysclk = 500;

	/*ppl-pixel packing layer, llp-low level protocol
	 * frame genartor timing parameters are clocked on the system clock
	 * whereas as the equivalent parameters in the LLP blocks are clocked
	 * on LLP Tx clock from the D-PHY - BYTE clock
	 */

	/*multiply by 1000 to keep the precision */
	ppl_llp_ratio = ((fg_cfg->bpp / 8) * sysclk * 1000) /
	    ((fg_cfg->lane_rate_mbps / 8) * fg_cfg->active_lanes);

	/*frame generator number of lines */
	reg_adr = MIPI_TXm_HS_FGn_NUM_LINES(ctrl_no, frame_gen);
	kmb_write_mipi(dev_p, reg_adr, fg_cfg->v_active);

	/*vsync width */
	/*
	 *there are 2 registers for vsync width -VSA in lines for channels 0-3
	 *REG_VSYNC_WIDTH0: [15:0]-VSA for channel0, [31:16]-VSA for channel1
	 *REG_VSYNC_WIDTH1: [15:0]-VSA for channel2, [31:16]-VSA for channel3
	 */
	offset = (frame_gen % 2) * 16;
	reg_adr = MIPI_TXm_HS_VSYNC_WIDTHn(ctrl_no, frame_gen / 2);
	kmb_write_bits_mipi(dev_p, reg_adr, offset, 16, fg_cfg->vsync_width);

	/*v backporch - same register config like vsync width */
	reg_adr = MIPI_TXm_HS_V_BACKPORCHESn(ctrl_no, frame_gen / 2);
	kmb_write_bits_mipi(dev_p, reg_adr, offset, 16, fg_cfg->v_backporch);

	/*v frontporch - same register config like vsync width */
	reg_adr = MIPI_TXm_HS_V_FRONTPORCHESn(ctrl_no, frame_gen / 2);
	kmb_write_bits_mipi(dev_p, reg_adr, offset, 16, fg_cfg->v_frontporch);

	/*v active - same register config like vsync width */
	reg_adr = MIPI_TXm_HS_V_ACTIVEn(ctrl_no, frame_gen / 2);
	kmb_write_bits_mipi(dev_p, reg_adr, offset, 16, fg_cfg->v_active);

	/*hsyc width */
	reg_adr = MIPI_TXm_HS_HSYNC_WIDTHn(ctrl_no, frame_gen);
	kmb_write_mipi(dev_p, reg_adr,
		       (fg_cfg->hsync_width * ppl_llp_ratio) / 1000);

	/*h backporch */
	reg_adr = MIPI_TXm_HS_H_BACKPORCHn(ctrl_no, frame_gen);
	kmb_write_mipi(dev_p, reg_adr,
		       (fg_cfg->h_backporch * ppl_llp_ratio) / 1000);

	/*h frontporch */
	reg_adr = MIPI_TXm_HS_H_FRONTPORCHn(ctrl_no, frame_gen);
	kmb_write_mipi(dev_p, reg_adr,
		       (fg_cfg->h_frontporch * ppl_llp_ratio) / 1000);

	/*h active */
	reg_adr = MIPI_TXm_HS_H_ACTIVEn(ctrl_no, frame_gen);
	/*convert h_active which is wc in bytes to cycles */
	val = (fg_cfg->h_active * sysclk * 1000) /
	    ((fg_cfg->lane_rate_mbps / 8) * fg_cfg->active_lanes);
	val /= 1000;
	kmb_write_mipi(dev_p, reg_adr, val);

	/* llp hsync width */
	reg_adr = MIPI_TXm_HS_LLP_HSYNC_WIDTHn(ctrl_no, frame_gen);
	kmb_write_mipi(dev_p, reg_adr, fg_cfg->hsync_width * (fg_cfg->bpp / 8));

	/* llp h backporch */
	reg_adr = MIPI_TXm_HS_LLP_H_BACKPORCHn(ctrl_no, frame_gen);
	kmb_write_mipi(dev_p, reg_adr, fg_cfg->h_backporch * (fg_cfg->bpp / 8));

	/* llp h frontporch */
	reg_adr = MIPI_TXm_HS_LLP_H_FRONTPORCHn(ctrl_no, frame_gen);
	kmb_write_mipi(dev_p, reg_adr,
		       fg_cfg->h_frontporch * (fg_cfg->bpp / 8));
}

static void mipi_tx_fg_cfg(struct kmb_drm_private *dev_p, u8 frame_gen,
			   u8 active_lanes, u32 bpp, u32 wc,
			   u32 lane_rate_mbps, struct mipi_tx_frame_cfg *fg_cfg)
{
	u32 i, fg_num_lines = 0;
	struct mipi_tx_frame_timing_cfg fg_t_cfg;

	/*calculate the total frame generator number of lines based on it's
	 * active sections
	 */
	for (i = 0; i < MIPI_TX_FRAME_GEN_SECTIONS; i++) {
		if (fg_cfg->sections[i] != NULL)
			fg_num_lines += fg_cfg->sections[i]->height_lines;
	}

	fg_t_cfg.bpp = bpp;
	fg_t_cfg.lane_rate_mbps = lane_rate_mbps;
	fg_t_cfg.hsync_width = fg_cfg->hsync_width;
	fg_t_cfg.h_backporch = fg_cfg->h_backporch;
	fg_t_cfg.h_frontporch = fg_cfg->h_frontporch;
	fg_t_cfg.h_active = wc;
	fg_t_cfg.vsync_width = fg_cfg->vsync_width;
	fg_t_cfg.v_backporch = fg_cfg->v_backporch;
	fg_t_cfg.v_frontporch = fg_cfg->v_frontporch;
	fg_t_cfg.v_active = fg_num_lines;
	fg_t_cfg.active_lanes = active_lanes;

	/*apply frame generator timing setting */
	mipi_tx_fg_cfg_regs(dev_p, frame_gen, &fg_t_cfg);
}

static void mipi_tx_multichannel_fifo_cfg(struct kmb_drm_private *dev_p,
					  u8 active_lanes, u8 vchannel_id)
{
	u32 fifo_size, fifo_rthreshold;
	u32 ctrl_no = MIPI_CTRL6;

	/*clear all mc fifo channel sizes and thresholds */
	kmb_write_mipi(dev_p, MIPI_TX_HS_MC_FIFO_CTRL_EN, 0);
	kmb_write_mipi(dev_p, MIPI_TX_HS_MC_FIFO_CHAN_ALLOC0, 0);
	kmb_write_mipi(dev_p, MIPI_TX_HS_MC_FIFO_CHAN_ALLOC1, 0);
	kmb_write_mipi(dev_p, MIPI_TX_HS_MC_FIFO_RTHRESHOLD0, 0);
	kmb_write_mipi(dev_p, MIPI_TX_HS_MC_FIFO_RTHRESHOLD1, 0);

	fifo_size = ((active_lanes > MIPI_D_LANES_PER_DPHY) ?
		     MIPI_CTRL_4LANE_MAX_MC_FIFO_LOC :
		     MIPI_CTRL_2LANE_MAX_MC_FIFO_LOC) - 1;
	/*MC fifo size for virtual channels 0-3 */
	/*
	 *REG_MC_FIFO_CHAN_ALLOC0: [8:0]-channel0, [24:16]-channel1
	 *REG_MC_FIFO_CHAN_ALLOC1: [8:0]-2, [24:16]-channel3
	 */
	SET_MC_FIFO_CHAN_ALLOC(dev_p, ctrl_no, vchannel_id, fifo_size);

	/*set threshold to half the fifo size, actual size=size*16 */
	fifo_rthreshold = ((fifo_size) * 8) & BIT_MASK_16;
	SET_MC_FIFO_RTHRESHOLD(dev_p, ctrl_no, vchannel_id, fifo_rthreshold);

	/*enable the MC FIFO channel corresponding to the Virtual Channel */
	kmb_set_bit_mipi(dev_p, MIPI_TXm_HS_MC_FIFO_CTRL_EN(ctrl_no),
			 vchannel_id);
}

static void mipi_tx_ctrl_cfg(struct kmb_drm_private *dev_p, u8 fg_id,
			     struct mipi_ctrl_cfg *ctrl_cfg)
{
	u32 sync_cfg = 0, ctrl = 0, fg_en;
	u32 ctrl_no = MIPI_CTRL6;

	/*MIPI_TX_HS_SYNC_CFG */
	if (ctrl_cfg->tx_ctrl_cfg.line_sync_pkt_en)
		sync_cfg |= LINE_SYNC_PKT_ENABLE;
	if (ctrl_cfg->tx_ctrl_cfg.frame_counter_active)
		sync_cfg |= FRAME_COUNTER_ACTIVE;
	if (ctrl_cfg->tx_ctrl_cfg.line_counter_active)
		sync_cfg |= LINE_COUNTER_ACTIVE;
	if (ctrl_cfg->tx_ctrl_cfg.tx_dsi_cfg->v_blanking)
		sync_cfg |= DSI_V_BLANKING;
	if (ctrl_cfg->tx_ctrl_cfg.tx_dsi_cfg->hsa_blanking)
		sync_cfg |= DSI_HSA_BLANKING;
	if (ctrl_cfg->tx_ctrl_cfg.tx_dsi_cfg->hbp_blanking)
		sync_cfg |= DSI_HBP_BLANKING;
	if (ctrl_cfg->tx_ctrl_cfg.tx_dsi_cfg->hfp_blanking)
		sync_cfg |= DSI_HFP_BLANKING;
	if (ctrl_cfg->tx_ctrl_cfg.tx_dsi_cfg->sync_pulse_eventn)
		sync_cfg |= DSI_SYNC_PULSE_EVENTN;
	if (ctrl_cfg->tx_ctrl_cfg.tx_dsi_cfg->lpm_first_vsa_line)
		sync_cfg |= DSI_LPM_FIRST_VSA_LINE;
	if (ctrl_cfg->tx_ctrl_cfg.tx_dsi_cfg->lpm_last_vfp_line)
		sync_cfg |= DSI_LPM_LAST_VFP_LINE;
	/* enable frame generator */
	fg_en = 1 << fg_id;
	sync_cfg |= FRAME_GEN_EN(fg_en);
	if (ctrl_cfg->tx_ctrl_cfg.tx_always_use_hact)
		sync_cfg |= ALWAYS_USE_HACT(fg_en);
	if (ctrl_cfg->tx_ctrl_cfg.tx_hact_wait_stop)
		sync_cfg |= HACT_WAIT_STOP(fg_en);

	/* MIPI_TX_HS_CTRL */
	ctrl = HS_CTRL_EN;	/* type:DSI,source:LCD */
	if (ctrl_cfg->tx_ctrl_cfg.tx_dsi_cfg->eotp_en)
		ctrl |= DSI_EOTP_EN;
	if (ctrl_cfg->tx_ctrl_cfg.tx_dsi_cfg->hfp_blank_en)
		ctrl |= DSI_CMD_HFP_EN;
	ctrl |= LCD_VC(fg_id);
	ctrl |= ACTIVE_LANES(ctrl_cfg->active_lanes - 1);
	/*67 ns stop time */
	ctrl |= HSEXIT_CNT(0x43);

	kmb_write_mipi(dev_p, MIPI_TXm_HS_SYNC_CFG(ctrl_no), sync_cfg);
	kmb_write_mipi(dev_p, MIPI_TXm_HS_CTRL(ctrl_no), ctrl);
}

#ifdef MIPI_TX_TEST_PATTERN_GENERATION
static void mipi_tx_hs_tp_gen(struct kmb_drm_private *dev_p, int vc,
			      int tp_sel, u32 stripe_width, u32 color0,
			      u32 color1, u32 ctrl_no)
{
	int val = 0;
	/* Select test pattern mode on the virtual channel */
	val = TP_SEL_VCm(vc, tp_sel);

	if (tp_sel == MIPI_TX_HS_TP_V_STRIPES ||
	    tp_sel == MIPI_TX_HS_TP_H_STRIPES) {
		val |= TP_STRIPE_WIDTH(stripe_width);
	}

	/* Configure test pattern colors */
	kmb_write_mipi(dev_p, MIPI_TXm_HS_TEST_PAT_COLOR0(ctrl_no), color0);
	kmb_write_mipi(dev_p, MIPI_TXm_HS_TEST_PAT_COLOR1(ctrl_no), color1);

	/* Enable test pattern generation on the virtual channel */
	val |= TP_EN_VCm(vc);
	kmb_write_mipi(dev_p, MIPI_TXm_HS_TEST_PAT_CTRL(ctrl_no), val);
}
#endif

static u32 mipi_tx_init_cntrl(struct kmb_drm_private *dev_p,
			      struct mipi_ctrl_cfg *ctrl_cfg)
{
	u32 ret;
	u8 active_vchannels = 0;
	u8 frame_id, sect;
	u32 bits_per_pclk = 0;
	u32 word_count = 0;

	/*This is the order in which mipi tx needs to be initialized
	 * set frame section parameters
	 * set frame specific parameters
	 * connect lcd to mipi
	 * multi channel fifo cfg
	 * set mipitxcctrlcfg
	 */

	for (frame_id = 0; frame_id < 4; frame_id++) {
		/* find valid frame, assume only one valid frame */
		if (ctrl_cfg->tx_ctrl_cfg.frames[frame_id] == NULL)
			continue;

		/* Frame Section configuration */
		/*TODO - assume there is only one valid section in a frame, so
		 * bits_per_pclk and word_count are only set once
		 */
		for (sect = 0; sect < MIPI_CTRL_VIRTUAL_CHANNELS; sect++) {
			if (ctrl_cfg->tx_ctrl_cfg.frames[frame_id]->sections[sect]
			    == NULL)
				continue;

			ret = mipi_tx_fg_section_cfg(dev_p, frame_id, sect,
						     ctrl_cfg->tx_ctrl_cfg.frames[frame_id]->sections[sect],
						     &bits_per_pclk,
						     &word_count);
			if (ret)
				return ret;

		}

		/* set frame specific parameters */
		mipi_tx_fg_cfg(dev_p, frame_id, ctrl_cfg->active_lanes,
			       bits_per_pclk,
			       word_count, ctrl_cfg->lane_rate_mbps,
			       ctrl_cfg->tx_ctrl_cfg.frames[frame_id]);

		active_vchannels++;

		/*connect lcd to mipi */

		/*stop iterating as only one virtual channel shall be used for
		 * LCD connection
		 */
		break;
	}

	if (active_vchannels == 0)
		return -EINVAL;
	/*Multi-Channel FIFO Configuration */
	mipi_tx_multichannel_fifo_cfg(dev_p, ctrl_cfg->active_lanes, frame_id);

	/*Frame Generator Enable */
	mipi_tx_ctrl_cfg(dev_p, frame_id, ctrl_cfg);
	return ret;
}

int dphy_read_testcode(struct kmb_drm_private *dev_p, int dphy_sel,
		       int test_code)
{
	u32 reg_wr_data;
	u32 reg_rd_data;
	int data;

	reg_wr_data = dphy_sel;
	kmb_write_mipi(dev_p, DPHY_TEST_CTRL1, reg_wr_data);

	data = 0;
	reg_wr_data = 0;
	reg_rd_data = 0;

	if (((dphy_sel >> 0 & 0x1) == 1) | ((dphy_sel >> 4 & 0x1) ==
					    1) | ((dphy_sel >> 8 & 0x1) == 1))
		reg_wr_data |= data << 0;
	if (((dphy_sel >> 1 & 0x1) == 1) | ((dphy_sel >> 5 & 0x1) ==
					    1) | ((dphy_sel >> 9 & 0x1) == 1))
		reg_wr_data |= data << 8;
	if (((dphy_sel >> 2 & 0x1) == 1) | ((dphy_sel >> 6 & 0x1) ==
					    1) | ((dphy_sel >> 10 & 0x1) == 1))
		reg_wr_data |= data << 16;
	if (((dphy_sel >> 3 & 0x1) == 1) | ((dphy_sel >> 7 & 0x1) ==
					    1) | ((dphy_sel >> 11 & 0x1) == 1))
		reg_wr_data |= data << 24;

	if ((dphy_sel >> 0 & 0xf) > 0)
		kmb_write_mipi(dev_p, DPHY_TEST_DIN0_3, reg_wr_data);
	if ((dphy_sel >> 4 & 0xf) > 0)
		kmb_write_mipi(dev_p, DPHY_TEST_DIN4_7, reg_wr_data);
	if ((dphy_sel >> 8 & 0x3) > 0)
		kmb_write_mipi(dev_p, DPHY_TEST_DIN8_9, reg_wr_data);

	reg_wr_data = 0;
	reg_wr_data = (dphy_sel | dphy_sel << 12);
	kmb_write_mipi(dev_p, DPHY_TEST_CTRL1, reg_wr_data);

	reg_wr_data = 0;
	reg_wr_data = dphy_sel << 12;
	kmb_write_mipi(dev_p, DPHY_TEST_CTRL1, reg_wr_data);

	reg_wr_data = 0;
	kmb_write_mipi(dev_p, DPHY_TEST_CTRL1, reg_wr_data);

	data = test_code >> 8 & 0xf;
	reg_wr_data = 0;
	if (((dphy_sel >> 0 & 0x1) == 1) | ((dphy_sel >> 4 & 0x1) ==
					    1) | ((dphy_sel >> 8 & 0x1) == 1))
		reg_wr_data |= data << 0;
	if (((dphy_sel >> 1 & 0x1) == 1) | ((dphy_sel >> 5 & 0x1) ==
					    1) | ((dphy_sel >> 9 & 0x1) == 1))
		reg_wr_data |= data << 8;
	if (((dphy_sel >> 2 & 0x1) == 1) | ((dphy_sel >> 6 & 0x1) ==
					    1) | ((dphy_sel >> 10 & 0x1) == 1))
		reg_wr_data |= data << 16;
	if (((dphy_sel >> 3 & 0x1) == 1) | ((dphy_sel >> 7 & 0x1) ==
					    1) | ((dphy_sel >> 11 & 0x1) == 1))
		reg_wr_data |= data << 24;

	if ((dphy_sel >> 0 & 0xf) > 0)
		kmb_write_mipi(dev_p, DPHY_TEST_DIN0_3, reg_wr_data);
	if ((dphy_sel >> 4 & 0xf) > 0)
		kmb_write_mipi(dev_p, DPHY_TEST_DIN4_7, reg_wr_data);
	if ((dphy_sel >> 8 & 0x3) > 0)
		kmb_write_mipi(dev_p, DPHY_TEST_DIN8_9, reg_wr_data);

	reg_wr_data = 0;
	reg_wr_data = dphy_sel;
	kmb_write_mipi(dev_p, DPHY_TEST_CTRL1, reg_wr_data);

	data = test_code & 0xff;
	reg_wr_data = 0;
	if (((dphy_sel >> 0 & 0x1) == 1) | ((dphy_sel >> 4 & 0x1) ==
					    1) | ((dphy_sel >> 8 & 0x1) == 1))
		reg_wr_data |= data << 0;
	if (((dphy_sel >> 1 & 0x1) == 1) | ((dphy_sel >> 5 & 0x1) ==
					    1) | ((dphy_sel >> 9 & 0x1) == 1))
		reg_wr_data |= data << 8;
	if (((dphy_sel >> 2 & 0x1) == 1) | ((dphy_sel >> 6 & 0x1) ==
					    1) | ((dphy_sel >> 10 & 0x1) == 1))
		reg_wr_data |= data << 16;
	if (((dphy_sel >> 3 & 0x1) == 1) | ((dphy_sel >> 7 & 0x1) ==
					    1) | ((dphy_sel >> 11 & 0x1) == 1))
		reg_wr_data |= data << 24;

	if ((dphy_sel >> 0 & 0xf) > 0)
		kmb_write_mipi(dev_p, DPHY_TEST_DIN0_3, reg_wr_data);
	if ((dphy_sel >> 4 & 0xf) > 0)
		kmb_write_mipi(dev_p, DPHY_TEST_DIN4_7, reg_wr_data);
	if ((dphy_sel >> 8 & 0x3) > 0)
		kmb_write_mipi(dev_p, DPHY_TEST_DIN8_9, reg_wr_data);

	reg_wr_data = 0;
	reg_wr_data = (dphy_sel | dphy_sel << 12);
	kmb_write_mipi(dev_p, DPHY_TEST_CTRL1, reg_wr_data);

	reg_wr_data = 0;
	reg_wr_data = dphy_sel << 12;
	kmb_write_mipi(dev_p, DPHY_TEST_CTRL1, reg_wr_data);

	reg_wr_data = 0;
	kmb_write_mipi(dev_p, DPHY_TEST_CTRL1, reg_wr_data);

	if ((dphy_sel >> 0 & 0xf) > 0)
		reg_rd_data = kmb_read_mipi(dev_p, DPHY_TEST_DOUT0_3);
	if ((dphy_sel >> 4 & 0xf) > 0)
		reg_rd_data = kmb_read_mipi(dev_p, DPHY_TEST_DOUT4_7);
	if ((dphy_sel >> 8 & 0x3) > 0)
		reg_rd_data = kmb_read_mipi(dev_p, DPHY_TEST_DOUT8_9);

	if (((dphy_sel >> 0 & 0x1) == 1) | ((dphy_sel >> 4 & 0x1) == 1) |
	    ((dphy_sel >> 8 & 0x1) == 1))
		data = reg_rd_data >> 0;
	if (((dphy_sel >> 1 & 0x1) == 1) | ((dphy_sel >> 5 & 0x1) == 1) |
	    ((dphy_sel >> 9 & 0x1) == 1))
		data = reg_rd_data >> 8;
	if (((dphy_sel >> 2 & 0x1) == 1) | ((dphy_sel >> 6 & 0x1) == 1) |
	    ((dphy_sel >> 10 & 0x1) == 1))
		data = reg_rd_data >> 16;
	if (((dphy_sel >> 3 & 0x1) == 1) | ((dphy_sel >> 7 & 0x1) == 1) |
	    ((dphy_sel >> 11 & 0x1) == 1))
		data = reg_rd_data >> 24;

	return data;

}

static void test_mode_send(struct kmb_drm_private *dev_p, u32 dphy_no,
			   u32 test_code, u32 test_data)
{
#ifdef DEBUG
	if (test_code != TEST_CODE_FSM_CONTROL)
		DRM_INFO("test_code = %02x, test_data = %08x\n", test_code,
			 test_data);
#endif

	/* send the test code first */
	/*  Steps for code:
	 * - set testclk HIGH
	 * - set testdin with test code
	 * - set testen HIGH
	 * - set testclk LOW
	 * - set testen LOW
	 */

	/* Set testclk high */
	SET_DPHY_TEST_CTRL1_CLK(dev_p, dphy_no);

	/* Set testdin */
	SET_TEST_DIN0_3(dev_p, dphy_no, test_code);

	/* Set testen high */
	SET_DPHY_TEST_CTRL1_EN(dev_p, dphy_no);

	/* Set testclk low */
	CLR_DPHY_TEST_CTRL1_CLK(dev_p, dphy_no);

	/* Set testen low */
	CLR_DPHY_TEST_CTRL1_EN(dev_p, dphy_no);

	/* Send the test data next */
	/*  Steps for data:
	 * - set testen LOW
	 * - set testclk LOW
	 * - set testdin with data
	 * - set testclk HIGH
	 */
	if (test_code) {
		/* Set testen low */
		CLR_DPHY_TEST_CTRL1_EN(dev_p, dphy_no);

		/* Set testclk low */
		CLR_DPHY_TEST_CTRL1_CLK(dev_p, dphy_no);

		/* Set data in testdin */
		kmb_write_mipi(dev_p,
			       DPHY_TEST_DIN0_3 + ((dphy_no / 0x4) * 0x4),
			       test_data << ((dphy_no % 4) * 8));

		/* Set testclk high */
		SET_DPHY_TEST_CTRL1_CLK(dev_p, dphy_no);
	}
}

static inline void
	set_test_mode_src_osc_freq_target_low_bits(struct kmb_drm_private
							      *dev_p,
							      u32 dphy_no,
							      u32 freq)
{
	/*typical rise/fall time=166,
	 * refer Table 1207 databook,sr_osc_freq_target[7:0
	 */
	test_mode_send(dev_p, dphy_no,
		       TEST_CODE_SLEW_RATE_DDL_CYCLES, (freq & 0x7f));
}

static inline void
set_test_mode_slew_rate_calib_en(struct kmb_drm_private *dev_p, u32 dphy_no)
{
	/*do not bypass slew rate calibration algorithm */
	/*bits[1:0}=srcal_en_ovr_en, srcal_en_ovr, bit[6]=sr_range */
	test_mode_send(dev_p, dphy_no, TEST_CODE_SLEW_RATE_OVERRIDE_CTRL,
		       (0x03 | (1 << 6)));
}

static inline void
set_test_mode_src_osc_freq_target_hi_bits(struct kmb_drm_private *dev_p,
					  u32 dphy_no, u32 freq)
{
	u32 data;
	/*typical rise/fall time=166, refer Table 1207 databook,
	 * sr_osc_freq_target[11:7
	 */
	data = ((freq >> 6) & 0x1f) | (1 << 7);	/*flag this as high nibble */
	test_mode_send(dev_p, dphy_no, TEST_CODE_SLEW_RATE_DDL_CYCLES, data);
}

struct vco_params {
	u32 freq;
	u32 range;
	u32 divider;
};

static struct vco_params vco_table[] = {
	{52, 0x3f, 8},
	{80, 0x39, 8},
	{105, 0x2f, 4},
	{160, 0x29, 4},
	{210, 0x1f, 2},
	{320, 0x19, 2},
	{420, 0x0f, 1},
	{630, 0x09, 1},
	{1100, 0x03, 1},
	{0xffff, 0x01, 1},
};

static void mipi_tx_get_vco_params(struct vco_params *vco)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vco_table); i++) {
		if (vco->freq < vco_table[i].freq) {
			*vco = vco_table[i];
			return;
		}
	}
	WARN_ONCE(1, "Invalid vco freq = %u for PLL setup\n", vco->freq);
}

static void mipi_tx_pll_setup(struct kmb_drm_private *dev_p, u32 dphy_no,
			      u32 ref_clk_mhz, u32 target_freq_mhz)
{
	/* pll_ref_clk: - valid range: 2~64 MHz; Typically 24 MHz
	 * Fvco: - valid range: 320~1250 MHz (Gen3 D-PHY)
	 * Fout: - valid range: 40~1250 MHz (Gen3 D-PHY)
	 * n: - valid range [0 15]
	 * N: - N = n + 1
	 *      -valid range: [1 16]
	 *      -conditions: - (pll_ref_clk / N) >= 2 MHz
	 *              -(pll_ref_clk / N) <= 8 MHz
	 * m: valid range [62 623]
	 * M: - M = m + 2
	 *      -valid range [64 625]
	 *      -Fvco = (M/N) * pll_ref_clk
	 */
	struct vco_params vco_p = {
		.range = 0,
		.divider = 1,
	};
	u32 best_n = 0, best_m = 0;
	u32 n = 0, m = 0, div = 0, delta, freq = 0, t_freq;
	u32 best_freq_delta = 3000;

	vco_p.freq = target_freq_mhz;
	mipi_tx_get_vco_params(&vco_p);
	/*search pll n parameter */
	for (n = PLL_N_MIN; n <= PLL_N_MAX; n++) {
		/*calculate the pll input frequency division ratio
		 * multiply by 1000 for precision -
		 * no floating point, add n for rounding
		 */
		div = ((ref_clk_mhz * 1000) + n) / (n + 1);
		/*found a valid n parameter */
		if ((div < 2000 || div > 8000))
			continue;
		/*search pll m parameter */
		for (m = PLL_M_MIN; m <= PLL_M_MAX; m++) {
			/*calculate the Fvco(DPHY PLL output frequency)
			 * using the current n,m params
			 */
			freq = div * (m + 2);
			freq /= 1000;
			/* trim the potential pll freq to max supported */
			if (freq > PLL_FVCO_MAX)
				continue;

			delta = abs(freq - target_freq_mhz);
			/*select the best (closest to target pll freq)
			 * n,m parameters so far
			 */
			if (delta < best_freq_delta) {
				best_n = n;
				best_m = m;
				best_freq_delta = delta;
			}
		}
	}

	/*Program vco_cntrl parameter
	 *PLL_VCO_Control[5:0] = pll_vco_cntrl_ovr,
	 * PLL_VCO_Control[6]   = pll_vco_cntrl_ovr_en
	 */
	test_mode_send(dev_p, dphy_no, TEST_CODE_PLL_VCO_CTRL, (vco_p.range
								| (1 << 6)));

	/*Program m, n pll parameters */

	DRM_INFO("%s : %d m = %d n = %d\n", __func__, __LINE__, best_m, best_n);

	/*PLL_Input_Divider_Ratio[3:0] = pll_n_ovr */
	test_mode_send(dev_p, dphy_no, TEST_CODE_PLL_INPUT_DIVIDER,
		       (best_n & 0x0f));

	/* m - low nibble PLL_Loop_Divider_Ratio[4:0] = pll_m_ovr[4:0] */
	test_mode_send(dev_p, dphy_no, TEST_CODE_PLL_FEEDBACK_DIVIDER,
		       (best_m & 0x1f));

	/*m -high nibble PLL_Loop_Divider_Ratio[4:0] = pll_m_ovr[9:5] */
	test_mode_send(dev_p, dphy_no, TEST_CODE_PLL_FEEDBACK_DIVIDER,
		       ((best_m >> 5) & 0x1f) | PLL_FEEDBACK_DIVIDER_HIGH);

	/*enable overwrite of n,m parameters :pll_n_ovr_en, pll_m_ovr_en */
	test_mode_send(dev_p, dphy_no, TEST_CODE_PLL_OUTPUT_CLK_SEL,
		       (PLL_N_OVR_EN | PLL_M_OVR_EN));

	/*Program Charge-Pump parameters */

	/*pll_prop_cntrl-fixed values for prop_cntrl from DPHY doc */
	t_freq = target_freq_mhz * vco_p.divider;
	test_mode_send(dev_p, dphy_no,
		       TEST_CODE_PLL_PROPORTIONAL_CHARGE_PUMP_CTRL,
		       ((t_freq > 1150) ? 0x0C : 0x0B));

	/*pll_int_cntrl-fixed value for int_cntrl from DPHY doc */
	test_mode_send(dev_p, dphy_no, TEST_CODE_PLL_INTEGRAL_CHARGE_PUMP_CTRL,
		       0x00);

	/*pll_gmp_cntrl-fixed value for gmp_cntrl from DPHY doci */
	test_mode_send(dev_p, dphy_no, TEST_CODE_PLL_GMP_CTRL, 0x10);

	/*pll_cpbias_cntrl-fixed value for cpbias_cntrl from DPHY doc */
	test_mode_send(dev_p, dphy_no, TEST_CODE_PLL_CHARGE_PUMP_BIAS, 0x10);

	/*pll_th1 -Lock Detector Phase error threshold, document gives fixed
	 * value
	 */
	test_mode_send(dev_p, dphy_no, TEST_CODE_PLL_PHASE_ERR_CTRL, 0x02);

	/*PLL Lock Configuration */

	/*pll_th2 - Lock Filter length, document gives fixed value */
	test_mode_send(dev_p, dphy_no, TEST_CODE_PLL_LOCK_FILTER, 0x60);

	/*pll_th3- PLL Unlocking filter, document gives fixed value */
	test_mode_send(dev_p, dphy_no, TEST_CODE_PLL_UNLOCK_FILTER, 0x03);

	/*pll_lock_sel-PLL Lock Detector Selection, document gives
	 * fixed value
	 */
	test_mode_send(dev_p, dphy_no, TEST_CODE_PLL_LOCK_DETECTOR, 0x02);
}

static void dphy_init_sequence(struct kmb_drm_private *dev_p,
			       struct mipi_ctrl_cfg *cfg, u32 dphy_no,
			       int active_lanes, enum dphy_mode mode)
{
	u32 test_code = 0;
	u32 test_data = 0, val;
	int i = 0;

	/* deassert SHUTDOWNZ signal */
	DRM_INFO("%s : %d  MIPI_DPHY_STAT0_4_7 = 0x%x)\n", __func__, __LINE__,
		 kmb_read_mipi(dev_p, MIPI_DPHY_STAT4_7));
	/*Set D-PHY in shutdown mode */
	/*assert RSTZ signal */
	CLR_DPHY_INIT_CTRL0(dev_p, dphy_no, RESETZ);
	/* assert SHUTDOWNZ signal */
	CLR_DPHY_INIT_CTRL0(dev_p, dphy_no, SHUTDOWNZ);
	val = kmb_read_mipi(dev_p, DPHY_INIT_CTRL0);
	DRM_INFO("%s : %d DPHY_INIT_CTRL0 = 0x%x\n", __func__, __LINE__, val);

	/*Init D-PHY_n */
	/*Pulse testclear signal to make sure the d-phy configuration starts
	 * from a clean base
	 */
	CLR_DPHY_TEST_CTRL0(dev_p, dphy_no);
	ndelay(15);
	SET_DPHY_TEST_CTRL0(dev_p, dphy_no);
	/*TODO may need to add 15ns delay here */
	ndelay(15);
	CLR_DPHY_TEST_CTRL0(dev_p, dphy_no);
	val = kmb_read_mipi(dev_p, DPHY_TEST_CTRL0);
	DRM_INFO("%s : %d DPHY_TEST_CTRL0 = 0x%x\n", __func__, __LINE__, val);
	ndelay(15);

	/*Set mastermacro bit - Master or slave mode */
	test_code = TEST_CODE_MULTIPLE_PHY_CTRL;
	/*DPHY has its own clock lane enabled (master) */
	if (mode == MIPI_DPHY_MASTER)
		test_data = 0x01;
	else
		test_data = 0x00;

	/*send the test code and data */
	test_mode_send(dev_p, dphy_no, test_code, test_data);
	/*Set the lane data rate */
	for (i = 0; i < MIPI_DPHY_DEFAULT_BIT_RATES; i++) {
		if (mipi_hs_freq_range[i].default_bit_rate_mbps <
		    cfg->lane_rate_mbps)
			continue;
		/* send the test code and data */
		/*bit[6:0] = hsfreqrange_ovr bit[7] = hsfreqrange_ovr_en */
		test_code = TEST_CODE_HS_FREQ_RANGE_CFG;
		test_data =
		    (mipi_hs_freq_range[i].hsfreqrange_code & 0x7f) | (1 << 7);
		test_mode_send(dev_p, dphy_no, test_code, test_data);
		break;
	}
	/*
	 * High-Speed Tx Slew Rate Calibration
	 * BitRate: > 1.5 Gbps && <= 2.5 Gbps: slew rate control OFF
	 */
	if (cfg->lane_rate_mbps > 1500) {
		/*bypass slew rate calibration algorithm */
		/*bits[1:0} srcal_en_ovr_en, srcal_en_ovr */
		test_code = TEST_CODE_SLEW_RATE_OVERRIDE_CTRL;
		test_data = 0x02;
		test_mode_send(dev_p, dphy_no, test_code, test_data);

		/* disable slew rate calibration */
		test_code = TEST_CODE_SLEW_RATE_DDL_LOOP_CTRL;
		test_data = 0x00;
		test_mode_send(dev_p, dphy_no, test_code, test_data);
	} else if (cfg->lane_rate_mbps > 1000) {
		/*BitRate: > 1 Gbps && <= 1.5 Gbps: - slew rate control ON
		 * typical rise/fall times: 166 ps
		 */

		/*do not bypass slew rate calibration algorithm */
		/*bits[1:0}=srcal_en_ovr_en, srcal_en_ovr, bit[6]=sr_range */
		test_code = TEST_CODE_SLEW_RATE_OVERRIDE_CTRL;
		test_data = (0x03 | (1 << 6));
		test_mode_send(dev_p, dphy_no, test_code, test_data);

//              set_test_mode_slew_rate_calib_en(dev_p, dphy_no);

		/* enable slew rate calibration */
		test_code = TEST_CODE_SLEW_RATE_DDL_LOOP_CTRL;
		test_data = 0x01;
		test_mode_send(dev_p, dphy_no, test_code, test_data);

		/*set sr_osc_freq_target[6:0] */
		/*typical rise/fall time=166, refer Table 1207 databook */
		/*typical rise/fall time=166, refer Table 1207 databook,
		 * sr_osc_freq_target[7:0
		 */
		test_code = TEST_CODE_SLEW_RATE_DDL_CYCLES;
		test_data = (0x72f & 0x7f);
		test_mode_send(dev_p, dphy_no, test_code, test_data);
		/*set sr_osc_freq_target[11:7] */
		/*typical rise/fall time=166, refer Table 1207 databook,
		 * sr_osc_freq_target[11:7
		 */
		test_code = TEST_CODE_SLEW_RATE_DDL_CYCLES;
		/*flag this as high nibble */
		test_data = ((0x72f >> 6) & 0x1f) | (1 << 7);
		test_mode_send(dev_p, dphy_no, test_code, test_data);
	} else {
		/*lane_rate_mbps <= 1000 Mbps */
		/*BitRate:  <= 1 Gbps:
		 * - slew rate control ON
		 * - typical rise/fall times: 225 ps
		 */

		/*do not bypass slew rate calibration algorithm */
		test_code = TEST_CODE_SLEW_RATE_OVERRIDE_CTRL;
		test_data = (0x03 | (1 << 6));
		test_mode_send(dev_p, dphy_no, test_code, test_data);

		/* enable slew rate calibration */
		test_code = TEST_CODE_SLEW_RATE_DDL_LOOP_CTRL;
		test_data = 0x01;
		test_mode_send(dev_p, dphy_no, test_code, test_data);

		/*typical rise/fall time=255, refer Table 1207 databook */
		test_code = TEST_CODE_SLEW_RATE_DDL_CYCLES;
		test_data = (0x523 & 0x7f);
		test_mode_send(dev_p, dphy_no, test_code, test_data);

		/*set sr_osc_freq_target[11:7] */
		test_code = TEST_CODE_SLEW_RATE_DDL_CYCLES;
		/*flag this as high nibble */
		test_data = ((0x523 >> 6) & 0x1f) | (1 << 7);
		test_mode_send(dev_p, dphy_no, test_code, test_data);

	}
	/*Set cfgclkfreqrange */
	val = (((cfg->cfg_clk_khz / 1000) - 17) * 4) & 0x3f;
	SET_DPHY_FREQ_CTRL0_3(dev_p, dphy_no, val);
	val = kmb_read_mipi(dev_p, DPHY_FREQ_CTRL0_3 + 4);

	/*Enable config clk for the corresponding d-phy */
	kmb_set_bit_mipi(dev_p, DPHY_CFG_CLK_EN, dphy_no);
	val = kmb_read_mipi(dev_p, DPHY_CFG_CLK_EN);
	/* PLL setup */
	if (mode == MIPI_DPHY_MASTER) {
		/*Set PLL regulator in bypass */
		test_code = TEST_CODE_PLL_ANALOG_PROG;
		test_data = 0x01;
		test_mode_send(dev_p, dphy_no, test_code, test_data);

		/* PLL Parameters Setup */
		mipi_tx_pll_setup(dev_p, dphy_no, cfg->ref_clk_khz / 1000,
				  cfg->lane_rate_mbps / 2);

		/*Set clksel */
		kmb_write_bits_mipi(dev_p, DPHY_INIT_CTRL1, PLL_CLKSEL_0,
				    2, 0x01);
		val = kmb_read_mipi(dev_p, DPHY_INIT_CTRL1);

		/*Set pll_shadow_control */
		kmb_set_bit_mipi(dev_p, DPHY_INIT_CTRL1, PLL_SHADOW_CTRL);
		val = kmb_read_mipi(dev_p, DPHY_INIT_CTRL1);
	}
#define MIPI_TX_FORCE_VOD
#ifdef MIPI_TX_FORCE_VOD
#define MIPI_TX_VOD_LVL	450
#define TEST_CODE_BANDGAP 0x24
	/* Set bandgap/VOD level */
	switch (MIPI_TX_VOD_LVL) {
	case 200:
		test_data = 0x00;
		break;
	case 300:
		test_data = 0x20;
		break;
	case 350:
		test_data = 0x40;
		break;
	case 450:
		test_data = 0x60;
		break;
	case 400:
	default:
		test_data = 0x70;
		break;
	}
	test_mode_send(dev_p, dphy_no, TEST_CODE_BANDGAP, test_data);
#endif

	/*Send NORMAL OPERATION test code */
	test_code = 0x0;
	test_data = 0x0;
	test_mode_send(dev_p, dphy_no, test_code, test_data);

	/* Configure BASEDIR for data lanes
	 * NOTE: basedir only applies to LANE_0 of each D-PHY.
	 * The other lanes keep their direction based on the D-PHY type,
	 * either Rx or Tx.
	 * bits[5:0]  - BaseDir: 1 = Rx
	 * bits[9:6] - BaseDir: 0 = Tx
	 */
	kmb_write_bits_mipi(dev_p, DPHY_INIT_CTRL2, 0, 9, 0x03f);
	val = kmb_read_mipi(dev_p, DPHY_INIT_CTRL2);
	ndelay(15);

	/* Enable CLOCK LANE - */
	/*clock lane should be enabled regardless of the direction set for
	 * the D-PHY (Rx/Tx)
	 */
	kmb_set_bit_mipi(dev_p, DPHY_INIT_CTRL2, 12 + dphy_no);
	val = kmb_read_mipi(dev_p, DPHY_INIT_CTRL2);

	/* enable DATA LANES */
	kmb_write_bits_mipi(dev_p, DPHY_ENABLE, dphy_no * 2, 2,
			    ((1 << active_lanes) - 1));

	val = kmb_read_mipi(dev_p, DPHY_ENABLE);
	ndelay(15);
	/*Take D-PHY out of shutdown mode */
	/* deassert SHUTDOWNZ signal */
	SET_DPHY_INIT_CTRL0(dev_p, dphy_no, SHUTDOWNZ);
	ndelay(15);

	/*deassert RSTZ signal */
	SET_DPHY_INIT_CTRL0(dev_p, dphy_no, RESETZ);
	val = kmb_read_mipi(dev_p, DPHY_INIT_CTRL0);
}

static void dphy_wait_fsm(struct kmb_drm_private *dev_p, u32 dphy_no,
			  enum dphy_tx_fsm fsm_state)
{
	enum dphy_tx_fsm val = DPHY_TX_POWERDWN;
	int i = 0;
	int status = 1;

	do {
		test_mode_send(dev_p, dphy_no, TEST_CODE_FSM_CONTROL, 0x80);
		/*TODO-need to add a time out and return failure */
		val = GET_TEST_DOUT4_7(dev_p, dphy_no);
		i++;
		if (i > TIMEOUT) {
			status = 0;
			DRM_INFO("%s: timing out fsm_state = %x GET_TEST_DOUT4_7 = %x",
			     __func__, fsm_state, kmb_read_mipi(dev_p,
						      DPHY_TEST_DOUT4_7));
			break;
		}
	} while (val != fsm_state);
	DRM_INFO("%s: dphy %d val = %x\n", __func__, dphy_no, val);

	DRM_INFO("%s: dphy %d val = %x\n", __func__, dphy_no, val);
	DRM_INFO("********** DPHY %d WAIT_FSM %s **********\n",
		 dphy_no, status ? "SUCCESS" : "FAILED");
}

static u32 wait_init_done(struct kmb_drm_private *dev_p, u32 dphy_no,
			  u32 active_lanes)
{
	u32 stopstatedata = 0;
	u32 data_lanes = (1 << active_lanes) - 1;
	int i = 0, val;
	int status = 1;

	DRM_INFO("%s : %d dphy = %d active_lanes=%d data_lanes=%d\n",
		 __func__, __LINE__, dphy_no, active_lanes, data_lanes);

	do {
		val = kmb_read_mipi(dev_p, MIPI_DPHY_STAT4_7);
		stopstatedata = GET_STOPSTATE_DATA(dev_p, dphy_no) & data_lanes;
		i++;
		if (i > TIMEOUT) {
			status = 0;
			DRM_INFO("!WAIT_INIT_DONE: TIMING OUT! (err_stat=%d)n",
				 kmb_read_mipi(dev_p, MIPI_DPHY_ERR_STAT6_7));
			break;
		}
		udelay(1);
	} while (stopstatedata != data_lanes);

	DRM_INFO("********** DPHY %d INIT - %s **********\n",
		 dphy_no, status ? "SUCCESS" : "FAILED");

	return 0;
}

static u32 wait_pll_lock(struct kmb_drm_private *dev_p, u32 dphy_no)
{
	int i = 0;
	int status = 1;

	do {
		;
		/*TODO-need to add a time out and return failure */
		i++;
		udelay(1);
		if (i > TIMEOUT) {
			status = 0;
			DRM_INFO("%s: timing out", __func__);
			DRM_INFO("%s : PLL_LOCK = 0x%x\n", __func__,
				 kmb_read_mipi(dev_p, DPHY_PLL_LOCK));
			break;
		}

	} while (!GET_PLL_LOCK(dev_p, dphy_no));

	DRM_INFO("********** PLL Locked for DPHY %d - %s **********\n",
		 dphy_no, status ? "SUCCESS" : "FAILED");
	return 0;
}

static u32 mipi_tx_init_dphy(struct kmb_drm_private *dev_p,
			     struct mipi_ctrl_cfg *cfg)
{
	u32 dphy_no = MIPI_DPHY6;

	DRM_INFO("%s : %d active_lanes=%d lane_rate=%d\n",
		 __func__, __LINE__, cfg->active_lanes,
		 MIPI_TX_LANE_DATA_RATE_MBPS);
	/*multiple D-PHYs needed */
	if (cfg->active_lanes > MIPI_DPHY_D_LANES) {
		/*
		 *Initialization for Tx aggregation mode is done according to
		 *http://dub30.ir.intel.com/bugzilla/show_bug.cgi?id=27785#c12:
		 *a. start init PHY1
		 *b. poll for PHY1 FSM state LOCK
		 *   b1. reg addr 0x03[3:0] - state_main[3:0] == 5 (LOCK)
		 *c. poll for PHY1 calibrations done :
		 *   c1. termination calibration lower section: addr 0x22[5]
		 *   - rescal_done
		 *   c2. slewrate calibration (if data rate < = 1500 Mbps):
		 *     addr 0xA7[3:2] - srcal_done, sr_finished
		 *d. start init PHY0
		 *e. poll for PHY0 stopstate
		 *f. poll for PHY1 stopstate
		 */
		/*PHY #N+1 ('slave') */

		dphy_init_sequence(dev_p, cfg, dphy_no + 1,
				   (cfg->active_lanes - MIPI_DPHY_D_LANES),
				   MIPI_DPHY_SLAVE);
		dphy_wait_fsm(dev_p, dphy_no + 1, DPHY_TX_LOCK);

		/*PHY #N master */
		dphy_init_sequence(dev_p, cfg, dphy_no, MIPI_DPHY_D_LANES,
				   MIPI_DPHY_MASTER);
		/* wait for DPHY init to complete */
		wait_init_done(dev_p, dphy_no, MIPI_DPHY_D_LANES);
		wait_init_done(dev_p, dphy_no + 1,
			       cfg->active_lanes - MIPI_DPHY_D_LANES);
		wait_pll_lock(dev_p, dphy_no);
		wait_pll_lock(dev_p, dphy_no + 1);
		udelay(1000);
		dphy_wait_fsm(dev_p, dphy_no, DPHY_TX_IDLE);
	} else {		/* Single DPHY */
		dphy_init_sequence(dev_p, cfg, dphy_no, cfg->active_lanes,
				   MIPI_DPHY_MASTER);
		dphy_wait_fsm(dev_p, dphy_no, DPHY_TX_IDLE);
		wait_init_done(dev_p, dphy_no, cfg->active_lanes);
		wait_pll_lock(dev_p, dphy_no);
	}

	return 0;
}

void mipi_tx_handle_irqs(struct kmb_drm_private *dev_p)
{
	uint32_t irq_ctrl_stat_0, hs_stat, hs_enable;
	uint32_t irq_ctrl_enabled_0;

	irq_ctrl_stat_0 = MIPI_GET_IRQ_STAT0(dev_p);
	irq_ctrl_enabled_0 = MIPI_GET_IRQ_ENABLED0(dev_p);
	/*only service enabled interrupts */
	irq_ctrl_stat_0 &= irq_ctrl_enabled_0;

	if (irq_ctrl_stat_0 & MIPI_DPHY_ERR_MASK) {
		if (irq_ctrl_stat_0 & ((1 << (MIPI_DPHY6 + 1))))
			SET_MIPI_CTRL_IRQ_CLEAR0(dev_p, MIPI_CTRL6,
						 MIPI_DPHY_ERR_IRQ);
	} else if (irq_ctrl_stat_0 & MIPI_HS_IRQ_MASK) {
		hs_stat = GET_MIPI_TX_HS_IRQ_STATUS(dev_p, MIPI_CTRL6);
		hs_enable = GET_HS_IRQ_ENABLE(dev_p, MIPI_CTRL6);
		hs_stat &= hs_enable;
		/*look for errors */
		if (hs_stat & MIPI_TX_HS_IRQ_ERROR) {
			CLR_HS_IRQ_ENABLE(dev_p, MIPI_CTRL6,
					  (hs_stat & MIPI_TX_HS_IRQ_ERROR) |
					  MIPI_TX_HS_IRQ_DMA_DONE |
					  MIPI_TX_HS_IRQ_DMA_IDLE);
		}
		/* clear local, then global */
		SET_MIPI_TX_HS_IRQ_CLEAR(dev_p, MIPI_CTRL6, hs_stat);
		SET_MIPI_CTRL_IRQ_CLEAR0(dev_p, MIPI_CTRL6, MIPI_HS_IRQ);
	}

}

void dma_transfer(struct kmb_drm_private *dev_p, int mipi_number,
		  u64 dma_start_address, int data_length)
{
	u64 dma_cfg_adr_offset;
	u64 dma_start_adr_offset;
	u64 dma_length_adr_offset;
	u32 reg_wr_data;
	int axi_burst_length;
	int mipi_fifo_flush;
	int dma_pipelined_axi_en;
	int dma_en;
	int dma_autorestart_mode_0;
	int tx_rx;

	DRM_INFO("%s: starting a new DMA transfer for mipi %d ", __func__,
		 mipi_number);

	if (mipi_number < 6)
		tx_rx = 0;
	else
		tx_rx = 1;

	dma_cfg_adr_offset =
		MIPI_TX_HS_DMA_CFG + HS_OFFSET(mipi_number);
	dma_start_adr_offset =
		MIPI_TX_HS_DMA_START_ADR_CHAN0 + HS_OFFSET(mipi_number);
	dma_length_adr_offset =
		MIPI_TX_HS_DMA_LEN_CHAN0 + HS_OFFSET(mipi_number);

	reg_wr_data = 0;
	reg_wr_data = dma_start_address;
	kmb_write_mipi(dev_p, dma_start_adr_offset, reg_wr_data);

	reg_wr_data = 0;
	reg_wr_data = data_length;
	kmb_write_mipi(dev_p, dma_length_adr_offset, reg_wr_data);

	axi_burst_length = 16;
	mipi_fifo_flush = 0;
	dma_pipelined_axi_en = 1;
	dma_en = 1;
	dma_autorestart_mode_0 = 0;

	reg_wr_data = 0;
	reg_wr_data =
	    ((axi_burst_length & 0x1ffff) << 0 | (mipi_fifo_flush & 0xf) << 9 |
	     (dma_pipelined_axi_en & 0x1) << 13 | (dma_en & 0xf) << 16 |
	     (dma_autorestart_mode_0 & 0x3) << 24);

	kmb_write_mipi(dev_p, dma_cfg_adr_offset, reg_wr_data);
}

/**
 * Reads specified number of bytes from the file.
 *
 * @param file         - file structure.
 * @param offset       - offset in the file.
 * @param addr         - address of the buffer.
 * @param count        - size of the buffer .
 *
 * @return 0 if success or error code.
 */
int kmb_kernel_read(struct file *file, loff_t offset,
		    char *addr, unsigned long count)
{
	char __user *buf = (char __user *)addr;
	ssize_t ret;

	if (!(file->f_mode & FMODE_READ))
		return -EBADF;

	ret = kernel_read(file, buf, count, &offset);

	return ret;
}

int kmb_dsi_hw_init(struct drm_device *dev)
{
	struct kmb_drm_private *dev_p = dev->dev_private;
	int i;

	if (hw_initialized)
		return 0;
	udelay(1000);
	kmb_write_mipi(dev_p, DPHY_ENABLE, 0);
	kmb_write_mipi(dev_p, DPHY_INIT_CTRL0, 0);
	kmb_write_mipi(dev_p, DPHY_INIT_CTRL1, 0);
	kmb_write_mipi(dev_p, DPHY_INIT_CTRL2, 0);

	/* initialize mipi controller */
	mipi_tx_init_cntrl(dev_p, &mipi_tx_init_cfg);
	/* irq initialization */
	//mipi_tx_init_irqs(dev_p, &int_cfg, &mipi_tx_init_cfg.tx_ctrl_cfg);
	/*d-phy initialization */
	mipi_tx_init_dphy(dev_p, &mipi_tx_init_cfg);
#ifdef MIPI_TX_TEST_PATTERN_GENERATION
	for (i = MIPI_CTRL6; i < MIPI_CTRL6 + 1; i++) {
		mipi_tx_hs_tp_gen(dev_p, 0, MIPI_TX_HS_TP_V_STRIPES,
				  0x05, 0xffffff, 0xff00, i);
	}
	DRM_INFO("%s : %d MIPI_TXm_HS_CTRL = 0x%x\n", __func__,
		 __LINE__, kmb_read_mipi(dev_p, MIPI_TXm_HS_CTRL(6)));
#else
	dma_data_length = image_height * image_width * unpacked_bytes;
	file = filp_open(IMAGE_PATH, O_RDWR, 0);
	if (IS_ERR(file)) {
		DRM_ERROR("filp_open failed\n");
		return -EBADF;
	}

	file_buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!file_buf) {
		DRM_ERROR("file_buf alloc failed\n");
		return -ENOMEM;
	}

	i_size = i_size_read(file_inode(file));
	while (offset < i_size) {

		file_buf_len = kmb_kernel_read(file, offset,
					       file_buf, PAGE_SIZE);
		if (file_buf_len < 0) {
			rc = file_buf_len;
			break;
		}
		if (file_buf_len == 0)
			break;
		offset += file_buf_len;
		count++;
		dma_tx_start_address = file_buf;
		dma_transfer(dev_p, MIPI_CTRL6, dma_tx_start_address,
			     PAGE_SIZE);

	}
	DRM_INFO("count = %d\n", count);
	kfree(file_buf);
	filp_close(file, NULL);

#endif //MIPI_TX_TEST_PATTERN_GENERATION

	hw_initialized = true;
	DRM_INFO("%s : %d mipi hw_initialized = %d\n", __func__, __LINE__,
		 hw_initialized);
	return 0;
}

int kmb_dsi_init(struct drm_device *dev, struct drm_bridge *bridge)
{
	struct kmb_dsi *kmb_dsi;
	struct drm_encoder *encoder;
	struct kmb_connector *kmb_connector;
	struct drm_connector *connector;
	struct kmb_dsi_host *host;
	int ret = 0;

	kmb_dsi = kzalloc(sizeof(*kmb_dsi), GFP_KERNEL);
	if (!kmb_dsi) {
		DRM_ERROR("failed to allocate kmb_dsi\n");
		return -ENOMEM;
	}

	DRM_INFO("%s : %d\n", __func__, __LINE__);
	kmb_connector = kzalloc(sizeof(*kmb_connector), GFP_KERNEL);
	if (!kmb_connector) {
		kfree(kmb_dsi);
		DRM_ERROR("failed to allocate kmb_connector\n");
		return -ENOMEM;
	}

	kmb_dsi->attached_connector = kmb_connector;

	host = kmb_dsi_host_init(dev, kmb_dsi);
	if (!host) {
		DRM_ERROR("Faile to allocate host\n");
//              drm_encoder_cleanup(encoder);
		kfree(kmb_dsi);
		kfree(kmb_connector);
		return -ENOMEM;
	}

	connector = &kmb_connector->base;
	encoder = &kmb_dsi->base;
	encoder->possible_crtcs = 1;
	encoder->possible_clones = 0;
	drm_encoder_init(dev, encoder, &kmb_dsi_funcs, DRM_MODE_ENCODER_DSI,
			 "MIPI-DSI");

	drm_connector_init(dev, connector, &kmb_dsi_connector_funcs,
			   DRM_MODE_CONNECTOR_DSI);
	drm_connector_helper_add(connector, &kmb_dsi_connector_helper_funcs);

	DRM_INFO("%s : %d connector = %s encoder = %s\n", __func__,
		 __LINE__, connector->name, encoder->name);

	DRM_INFO("%s : %d\n", __func__, __LINE__);
	ret = drm_connector_attach_encoder(connector, encoder);

	/* Link drm_bridge to encoder */
#ifndef FCCTEST
	ret = drm_bridge_attach(encoder, bridge, NULL);
	if (ret) {
		DRM_ERROR("failed to attach bridge to MIPI\n");
		drm_encoder_cleanup(encoder);
		return ret;
	}
#endif
#ifndef FCCTEST
	DRM_INFO("%s : %d Bridge attached : SUCCESS\n", __func__, __LINE__);
#endif

#ifdef FCCTEST
	kmb_dsi_hw_init(dev);
#endif

	return 0;
}
