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
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include "kmb_drv.h"
#include "kmb_regs.h"
#include "kmb_dsi.h"

#define IMG_WIDTH_PX      1920
#define IMG_HEIGHT_LINES  1080
#define LCD_BYTESPP       1

/*MIPI TX CFG*/
#define MIPI_TX_ACTIVE_LANES        4
#define MIPI_TX_LANE_DATA_RATE_MBPS 888
#define MIPI_TX_REF_CLK_KHZ         24000
#define MIPI_TX_CFG_CLK_KHZ         24000

/*
 * These are added here only temporarily for testing,
 * these will eventually go to the device tree sections,
 * and can be used as a refernce later for device tree additions
 */
struct mipi_tx_frame_section_cfg mipi_tx_frame0_sect_cfg = {
	.width_pixels = IMG_WIDTH_PX,
	.height_lines = IMG_HEIGHT_LINES,
	.data_type = DSI_LP_DT_PPS_RGB888_24B,
	.data_mode = MIPI_DATA_MODE1,
	.dma_packed = 0
};

struct mipi_tx_frame_cfg mipitx_frame0_cfg = {
	.sections[0] = &mipi_tx_frame0_sect_cfg,
	.sections[1] = NULL,
	.sections[2] = NULL,
	.sections[3] = NULL,
	.vsync_width = 5,
	.v_backporch = 36,
	.v_frontporch = 4,
	.hsync_width = 44,
	.h_backporch = 148,
	.h_frontporch = 88
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
	.data_if = MIPI_IF_PARALLEL,
	.tx_ctrl_cfg = {
			.frames[0] = &mipitx_frame0_cfg,
			.frames[1] = NULL,
			.frames[2] = NULL,
			.frames[3] = NULL,
			.tx_dsi_cfg = &mipitx_dsi_cfg,
			.line_sync_pkt_en = 0,
			.line_counter_active = 0,
			.frame_counter_active = 0,
			}

};

static enum drm_mode_status
kmb_dsi_mode_valid(struct drm_connector *connector,
		   struct drm_display_mode *mode)
{
	return MODE_OK;
}

static int kmb_dsi_get_modes(struct drm_connector *connector)
{
	struct drm_display_mode *mode;
	struct kmb_connector *kmb_connector = to_kmb_connector(connector);

	mode = drm_mode_duplicate(connector->dev, kmb_connector->fixed_mode);
	drm_mode_probed_add(connector, mode);
	return 1;
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

static struct kmb_dsi_host *kmb_dsi_host_init(struct kmb_dsi *kmb_dsi)
{
	struct kmb_dsi_host *host;
	struct mipi_dsi_device *device;

	host = kzalloc(sizeof(*host), GFP_KERNEL);
	if (!host)
		return NULL;

	host->base.ops = &kmb_dsi_host_ops;
	host->kmb_dsi = kmb_dsi;

	device = kzalloc(sizeof(*device), GFP_KERNEL);
	if (!device) {
		kfree(host);
		return NULL;
	}
	device->host = &host->base;
	host->device = device;
	return host;
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

static u32 mipi_tx_fg_section_cfg_regs(struct kmb_drm_private *dev_priv,
				       u8 frame_id, u8 section,
				       u32 height_lines, u32 unpacked_bytes,
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
	kmb_write(dev_priv,
		  (MIPI_TXm_HS_FGn_SECTo_PH(ctrl_no, frame_id, section)), cfg);

	/*unpacked bytes */
	/*there are 4 frame generators and each fg has 4 sections
	 *there are 2 registers for unpacked bytes -
	 *# bytes each section occupies in memory
	 *REG_UNPACKED_BYTES0: [15:0]-BYTES0, [31:16]-BYTES1
	 *REG_UNPACKED_BYTES1: [15:0]-BYTES2, [31:16]-BYTES3
	 */
	reg_adr =
	    MIPI_TXm_HS_FGn_SECT_UNPACKED_BYTES0(ctrl_no,
						 frame_id) + (section / 2) * 4;
	kmb_write_bits(dev_priv, reg_adr, (section % 2) * 16, 16,
		       unpacked_bytes);

	/* line config */
	reg_adr = MIPI_TXm_HS_FGn_SECTo_LINE_CFG(ctrl_no, frame_id, section);
	kmb_write(dev_priv, reg_adr, height_lines);
	return 0;
}

static u32 mipi_tx_fg_section_cfg(struct kmb_drm_private *dev_priv,
				  u8 frame_id,
				  u8 section,
				  struct mipi_tx_frame_section_cfg *frame_scfg,
				  u32 *bits_per_pclk, u32 *wc)
{
	u32 ret = 0;
	u32 unpacked_bytes;
	struct mipi_data_type_params data_type_parameters;
	struct mipi_tx_frame_sect_phcfg ph_cfg;

	ret =
	    mipi_get_datatype_params(frame_scfg->data_type,
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

	mipi_tx_fg_section_cfg_regs(dev_priv, frame_id, section,
				    frame_scfg->height_lines, unpacked_bytes,
				    &ph_cfg);

	/*caller needs bits_per_clk for additional caluclations */
	*bits_per_pclk = data_type_parameters.bits_per_pclk;
	return 0;
}

static void mipi_tx_fg_cfg_regs(struct kmb_drm_private *dev_priv,
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
	sysclk = 700;

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
	kmb_write(dev_priv, reg_adr, fg_cfg->v_active);

	/*vsync width */
	/*
	 *there are 2 registers for vsync width -VSA in lines for channels 0-3
	 *REG_VSYNC_WIDTH0: [15:0]-VSA for channel0, [31:16]-VSA for channel1
	 *REG_VSYNC_WIDTH1: [15:0]-VSA for channel2, [31:16]-VSA for channel3
	 */
	offset = (frame_gen % 2) * 16;
	reg_adr = MIPI_TXm_HS_VSYNC_WIDTHn(ctrl_no, frame_gen);
	kmb_write_bits(dev_priv, reg_adr, offset, 16, fg_cfg->vsync_width);

	/*v backporch - same register config like vsync width */
	reg_adr = MIPI_TXm_HS_V_BACKPORCHESn(ctrl_no, frame_gen);
	kmb_write_bits(dev_priv, reg_adr, offset, 16, fg_cfg->v_backporch);

	/*v frontporch - same register config like vsync width */
	reg_adr = MIPI_TXm_HS_V_FRONTPORCHESn(ctrl_no, frame_gen);
	kmb_write_bits(dev_priv, reg_adr, offset, 16, fg_cfg->v_frontporch);

	/*v active - same register config like vsync width */
	reg_adr = MIPI_TXm_HS_V_ACTIVEn(ctrl_no, frame_gen);
	kmb_write_bits(dev_priv, reg_adr, offset, 16, fg_cfg->v_active);

	/*hsyc width */
	reg_adr = MIPI_TXm_HS_HSYNC_WIDTHn(ctrl_no, frame_gen);
	kmb_write(dev_priv, reg_adr,
		  (fg_cfg->hsync_width * ppl_llp_ratio) / 1000);

	/*h backporch */
	reg_adr = MIPI_TXm_HS_H_BACKPORCHn(ctrl_no, frame_gen);
	kmb_write(dev_priv, reg_adr,
		  (fg_cfg->h_backporch * ppl_llp_ratio) / 1000);

	/*h frontporch */
	reg_adr = MIPI_TXm_HS_H_FRONTPORCHn(ctrl_no, frame_gen);
	kmb_write(dev_priv, reg_adr,
		  (fg_cfg->h_frontporch * ppl_llp_ratio) / 1000);

	/*h active */
	reg_adr = MIPI_TXm_HS_H_ACTIVEn(ctrl_no, frame_gen);
	/*convert h_active which is wc in bytes to cycles */
	val = (fg_cfg->h_active * sysclk * 1000) /
	    ((fg_cfg->lane_rate_mbps / 8) * fg_cfg->active_lanes);
	val /= 1000;
	kmb_write(dev_priv, reg_adr, val);

	/* llp hsync width */
	reg_adr = MIPI_TXm_HS_LLP_HSYNC_WIDTHn(ctrl_no, frame_gen);
	kmb_write(dev_priv, reg_adr, fg_cfg->hsync_width * (fg_cfg->bpp / 8));

	/* llp h backporch */
	reg_adr = MIPI_TXm_HS_LLP_H_BACKPORCHn(ctrl_no, frame_gen);
	kmb_write(dev_priv, reg_adr, fg_cfg->h_backporch * (fg_cfg->bpp / 8));

	/* llp h frontporch */
	reg_adr = MIPI_TXm_HS_LLP_H_FRONTPORCHn(ctrl_no, frame_gen);
	kmb_write(dev_priv, reg_adr, fg_cfg->h_frontporch * (fg_cfg->bpp / 8));
}

static void mipi_tx_fg_cfg(struct kmb_drm_private *dev_priv, u8 frame_gen,
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
	mipi_tx_fg_cfg_regs(dev_priv, frame_gen, &fg_t_cfg);
}

static u32 mipi_tx_init_cntrl(struct kmb_drm_private *dev_priv,
			      struct mipi_ctrl_cfg *ctrl_cfg)
{
	u32 ret;
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

			ret = mipi_tx_fg_section_cfg(dev_priv, frame_id, sect,
						     ctrl_cfg->tx_ctrl_cfg.frames[frame_id]->sections[sect],
						     &bits_per_pclk,
						     &word_count);
			if (ret)
				return ret;

		}

		/* set frame specific parameters */
		mipi_tx_fg_cfg(dev_priv, frame_id, ctrl_cfg->active_lanes,
			       bits_per_pclk,
			       word_count,
			       ctrl_cfg->lane_rate_mbps,
			       ctrl_cfg->tx_ctrl_cfg.frames[frame_id]);
		/*function for setting frame sepecific parameters will be
		 * called here
		 */
		/*bits_per_pclk and word_count will be passed in to this
		 * function
		 */

	}
	return ret;
}

void kmb_dsi_init(struct drm_device *dev)
{
	struct kmb_dsi *kmb_dsi;
	struct drm_encoder *encoder;
	struct kmb_connector *kmb_connector;
	struct drm_connector *connector;
	struct kmb_dsi_host *host;
	struct kmb_drm_private *dev_priv = dev->dev_private;

	kmb_dsi = kzalloc(sizeof(*kmb_dsi), GFP_KERNEL);
	if (!kmb_dsi)
		return;

	kmb_connector = kzalloc(sizeof(*kmb_connector), GFP_KERNEL);
	if (!kmb_connector) {
		kfree(kmb_dsi);
		return;
	}

	kmb_dsi->attached_connector = kmb_connector;

	connector = &kmb_connector->base;
	encoder = &kmb_dsi->base;
	drm_encoder_init(dev, encoder, &kmb_dsi_funcs, DRM_MODE_ENCODER_DSI,
			 "MIPI-DSI");

	host = kmb_dsi_host_init(kmb_dsi);
	if (!host) {
		drm_encoder_cleanup(encoder);
		kfree(kmb_dsi);
		kfree(kmb_connector);
	}

	drm_connector_init(dev, connector, &kmb_dsi_connector_funcs,
			   DRM_MODE_CONNECTOR_DSI);
	drm_connector_helper_add(connector, &kmb_dsi_connector_helper_funcs);

	connector->encoder = encoder;
	drm_connector_attach_encoder(connector, encoder);

	/* initialize mipi controller */
	mipi_tx_init_cntrl(dev_priv, &mipi_tx_init_cfg);
}
