/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2016 - 2018 Intel Corporation
 *
 * Author: Wang, Zaikuo <zaikuo.wang@intel.com>
 *
 */

#ifndef __CRLMODULE_PIXTER_STUB_CONFIGURATION_H_
#define __CRLMODULE_PIXTER_STUB_CONFIGURATION_H_

#include "crlmodule-sensor-ds.h"

static struct crl_pll_configuration pixter_stub_pll_configurations[] = {
	{
		.input_clk = 24000000,
		.op_sys_clk = 400000000,
		.bitsperpixel = 8,
		.pixel_rate_csi = 800000000,
		.pixel_rate_pa = 800000000,
		.comp_items = 0,
		.ctrl_data = 0,
		.pll_regs_items = 0,
		.pll_regs = NULL,
	},
	{
		.input_clk = 24000000,
		.op_sys_clk = 400000000,
		.bitsperpixel = 10,
		.pixel_rate_csi = 800000000,
		.pixel_rate_pa = 800000000,
		.comp_items = 0,
		.ctrl_data = 0,
		.pll_regs_items = 0,
		.pll_regs = NULL,
	},
	{
		.input_clk = 24000000,
		.op_sys_clk = 400000000,
		.bitsperpixel = 12,
		.pixel_rate_csi = 800000000,
		.pixel_rate_pa = 800000000,
		.comp_items = 0,
		.ctrl_data = 0,
		.pll_regs_items = 0,
		.pll_regs = NULL,
	},
	{
		.input_clk = 24000000,
		.op_sys_clk = 400000000,
		.bitsperpixel = 16,
		.pixel_rate_csi = 800000000,
		.pixel_rate_pa = 800000000,
		.comp_items = 0,
		.ctrl_data = 0,
		.pll_regs_items = 0,
		.pll_regs = NULL,
	},
	{
		.input_clk = 24000000,
		.op_sys_clk = 400000000,
		.bitsperpixel = 20,
		.pixel_rate_csi = 800000000,
		.pixel_rate_pa = 800000000,
		.comp_items = 0,
		.ctrl_data = 0,
		.pll_regs_items = 0,
		.pll_regs = NULL,
	},
	{
		.input_clk = 24000000,
		.op_sys_clk = 400000000,
		.bitsperpixel = 24,
		.pixel_rate_csi = 800000000,
		.pixel_rate_pa = 800000000,
		.comp_items = 0,
		.ctrl_data = 0,
		.pll_regs_items = 0,
		.pll_regs = NULL,
	},
};

/* resolutions for linux pss with yuv/rgb pass-through */
static struct crl_subdev_rect_rep pixter_stub_vga_rects[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 4096, 3072 },
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_SCALER,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 640, 480 },
	},
};

static struct crl_subdev_rect_rep pixter_stub_720p_rects[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 4096, 3072 },
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_SCALER,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 1280, 720 },
	},
};

static struct crl_subdev_rect_rep pixter_stub_1080p_rects[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 4096, 3072 },
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_SCALER,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 1920, 1080 },
	},
};

static struct crl_subdev_rect_rep pixter_stub_4p5_rects[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 4096, 3072 },
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_SCALER,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 2816, 1600 },
	},
};

static struct crl_subdev_rect_rep pixter_stub_4k_rects[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 4096, 3072 },
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_SCALER,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 3840, 2160 },
	},
};

static struct crl_subdev_rect_rep pixter_stub_480i_rects[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 4096, 3072 },
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_SCALER,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 720, 240 },
	},
};

static struct crl_subdev_rect_rep pixter_stub_576i_rects[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 4096, 3072 },
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_SCALER,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 720, 288 },
	},
};

static struct crl_subdev_rect_rep pixter_stub_1080i_rects[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 4096, 3072 },
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_SCALER,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 1920, 540 },
	},
};

/* vga for linux pss with imx135/imx477 input simulation */
static struct crl_subdev_rect_rep pixter_stub_vga_pad1_rects[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 4096, 3072 },
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_SCALER,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 672, 512 },
	},
};

/* vga for linux pss with imx135/imx477 input simulation */
static struct crl_subdev_rect_rep pixter_stub_vga_pad2_rects[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 4096, 3072 },
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_SCALER,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 768, 576 },
	},
};


/* 720p for linux pss with imx135/imx477 input simulation */
static struct crl_subdev_rect_rep pixter_stub_720p_pad1_rects[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 4096, 3072 },
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_SCALER,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 1312, 768 },
	},
};

/* 720p for linux pss with imx135/imx477 input simulation */
static struct crl_subdev_rect_rep pixter_stub_720p_pad2_rects[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 4096, 3072 },
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_SCALER,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 1312, 984 },
	},
};


/* 1080p for linux pss with imx135 input simulation */
static struct crl_subdev_rect_rep pixter_stub_1080p_pad1_rects[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 4096, 3072 },
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_SCALER,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 1952, 1120 },
	},
};

/* 1080p for linux pss with imx477 input simulation */
static struct crl_subdev_rect_rep pixter_stub_1080p_pad2_rects[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 4096, 3072 },
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_SCALER,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 2048, 1128 },
	},
};


/* 1080p for linux pss with imx477 input simulation */
static struct crl_subdev_rect_rep pixter_stub_1080p_pad3_rects[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 4096, 3072 },
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_SCALER,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 2048, 1536 },
	},
};


/* 2816x1600 for linux pss with imx135/imx477 input simulation */
static struct crl_subdev_rect_rep pixter_stub_4p5_pad1_rects[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 4096, 3072 },
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_SCALER,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 2848, 1632 },
	},
};
/* 2816x1600 for linux pss with imx135/imx477 input simulation */
static struct crl_subdev_rect_rep pixter_stub_4p5_pad2_rects[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 4096, 3072 },
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_SCALER,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 2848, 2136 },
	},
};


/* 4k for linux pss with imx135 input simulation */
static struct crl_subdev_rect_rep pixter_stub_4k_pad1_rects[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 4096, 3072 },
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_SCALER,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 3872, 2208 },
	},
};

/* 4k for linux pss with imx477 input simulation */
static struct crl_subdev_rect_rep pixter_stub_4k_pad2_rects[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 4096, 3072 },
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_SCALER,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 4064, 2288 },
	},
};

/* 4k for linux pss with imx477 input simulation */
static struct crl_subdev_rect_rep pixter_stub_4k_pad3_rects[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 4096, 3072 },
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_SCALER,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 4032, 2288 },
	},
};

/* 4096x3072 for linux pss with imx135/imx477 full input simulation */
static struct crl_subdev_rect_rep pixter_stub_full_rects[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 4096, 3072 },
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_SCALER,
		.in_rect = { 0, 0, 4096, 3072 },
		.out_rect = { 0, 0, 4096, 3072 },
	},
};


static struct crl_mode_rep pixter_stub_modes[] = {
	{
		.sd_rects_items = ARRAY_SIZE(pixter_stub_vga_rects),
		.sd_rects = pixter_stub_vga_rects,
		.scale_m = 1,
		.width = 640,
		.height = 480,
		.min_llp = 6024,
		.min_fll = 4096,
		.comp_items = 0,
		.ctrl_data = 0,
		.mode_regs_items = 0,
		.mode_regs = NULL,
	},
	{
		.sd_rects_items = ARRAY_SIZE(pixter_stub_720p_rects),
		.sd_rects = pixter_stub_720p_rects,
		.scale_m = 1,
		.width = 1280,
		.height = 720,
		.min_llp = 6024,
		.min_fll = 4096,
		.comp_items = 0,
		.ctrl_data = 0,
		.mode_regs_items = 0,
		.mode_regs = NULL,
	},
	{
		.sd_rects_items = ARRAY_SIZE(pixter_stub_1080p_rects),
		.sd_rects = pixter_stub_1080p_rects,
		.scale_m = 1,
		.width = 1920,
		.height = 1080,
		.min_llp = 6024,
		.min_fll = 4096,
		.comp_items = 0,
		.ctrl_data = 0,
		.mode_regs_items = 0,
		.mode_regs = NULL,
	},
	{
		.sd_rects_items = ARRAY_SIZE(pixter_stub_4p5_rects),
		.sd_rects = pixter_stub_4p5_rects,
		.scale_m = 1,
		.width = 2816,
		.height = 1600,
		.min_llp = 6024,
		.min_fll = 4096,
		.comp_items = 0,
		.ctrl_data = 0,
		.mode_regs_items = 0,
		.mode_regs = NULL,
	},
	{
		.sd_rects_items = ARRAY_SIZE(pixter_stub_4k_rects),
		.sd_rects = pixter_stub_4k_rects,
		.scale_m = 1,
		.width = 3840,
		.height = 2160,
		.min_llp = 6024,
		.min_fll = 4096,
		.comp_items = 0,
		.ctrl_data = 0,
		.mode_regs_items = 0,
		.mode_regs = NULL,
	},
	{
		.sd_rects_items = ARRAY_SIZE(pixter_stub_vga_pad1_rects),
		.sd_rects = pixter_stub_vga_pad1_rects,
		.scale_m = 1,
		.width = 672,
		.height = 512,
		.min_llp = 6024,
		.min_fll = 4096,
		.comp_items = 0,
		.ctrl_data = 0,
		.mode_regs_items = 0,
		.mode_regs = NULL,
	},
	{
		.sd_rects_items = ARRAY_SIZE(pixter_stub_vga_pad2_rects),
		.sd_rects = pixter_stub_vga_pad2_rects,
		.scale_m = 1,
		.width = 768,
		.height = 576,
		.min_llp = 6024,
		.min_fll = 4096,
		.comp_items = 0,
		.ctrl_data = 0,
		.mode_regs_items = 0,
		.mode_regs = NULL,
	},
	{
		.sd_rects_items = ARRAY_SIZE(pixter_stub_720p_pad1_rects),
		.sd_rects = pixter_stub_720p_pad1_rects,
		.scale_m = 1,
		.width = 1312,
		.height = 768,
		.min_llp = 6024,
		.min_fll = 4096,
		.comp_items = 0,
		.ctrl_data = 0,
		.mode_regs_items = 0,
		.mode_regs = NULL,
	},
	{
		.sd_rects_items = ARRAY_SIZE(pixter_stub_720p_pad2_rects),
		.sd_rects = pixter_stub_720p_pad2_rects,
		.scale_m = 1,
		.width = 1312,
		.height = 984,
		.min_llp = 6024,
		.min_fll = 4096,
		.comp_items = 0,
		.ctrl_data = 0,
		.mode_regs_items = 0,
		.mode_regs = NULL,
	},
	{
		.sd_rects_items = ARRAY_SIZE(pixter_stub_1080p_pad1_rects),
		.sd_rects = pixter_stub_1080p_pad1_rects,
		.scale_m = 1,
		.width = 1952,
		.height = 1120,
		.min_llp = 6024,
		.min_fll = 4096,
		.comp_items = 0,
		.ctrl_data = 0,
		.mode_regs_items = 0,
		.mode_regs = NULL,
	},
	{
		.sd_rects_items = ARRAY_SIZE(pixter_stub_1080p_pad2_rects),
		.sd_rects = pixter_stub_1080p_pad2_rects,
		.scale_m = 1,
		.width = 2048,
		.height = 1128,
		.min_llp = 6024,
		.min_fll = 4096,
		.comp_items = 0,
		.ctrl_data = 0,
		.mode_regs_items = 0,
		.mode_regs = NULL,
	},
	{
		.sd_rects_items = ARRAY_SIZE(pixter_stub_1080p_pad3_rects),
		.sd_rects = pixter_stub_1080p_pad3_rects,
		.scale_m = 1,
		.width = 2048,
		.height = 1536,
		.min_llp = 6024,
		.min_fll = 4096,
		.comp_items = 0,
		.ctrl_data = 0,
		.mode_regs_items = 0,
		.mode_regs = NULL,
	},

	{
		.sd_rects_items = ARRAY_SIZE(pixter_stub_4p5_pad1_rects),
		.sd_rects = pixter_stub_4p5_pad1_rects,
		.scale_m = 1,
		.width = 2848,
		.height = 1632,
		.min_llp = 6024,
		.min_fll = 4096,
		.comp_items = 0,
		.ctrl_data = 0,
		.mode_regs_items = 0,
		.mode_regs = NULL,
	},
	{
		.sd_rects_items = ARRAY_SIZE(pixter_stub_4p5_pad2_rects),
		.sd_rects = pixter_stub_4p5_pad2_rects,
		.scale_m = 1,
		.width = 2848,
		.height = 2136,
		.min_llp = 6024,
		.min_fll = 4096,
		.comp_items = 0,
		.ctrl_data = 0,
		.mode_regs_items = 0,
		.mode_regs = NULL,
	},
	{
		.sd_rects_items = ARRAY_SIZE(pixter_stub_4k_pad1_rects),
		.sd_rects = pixter_stub_4k_pad1_rects,
		.scale_m = 1,
		.width = 3872,
		.height = 2208,
		.min_llp = 6024,
		.min_fll = 4096,
		.comp_items = 0,
		.ctrl_data = 0,
		.mode_regs_items = 0,
		.mode_regs = NULL,
	},
	{
		.sd_rects_items = ARRAY_SIZE(pixter_stub_4k_pad2_rects),
		.sd_rects = pixter_stub_4k_pad2_rects,
		.scale_m = 1,
		.width = 4064,
		.height = 2288,
		.min_llp = 6024,
		.min_fll = 4096,
		.comp_items = 0,
		.ctrl_data = 0,
		.mode_regs_items = 0,
		.mode_regs = NULL,
	},
	{
		.sd_rects_items = ARRAY_SIZE(pixter_stub_4k_pad3_rects),
		.sd_rects = pixter_stub_4k_pad3_rects,
		.scale_m = 1,
		.width = 4032,
		.height = 2288,
		.min_llp = 6024,
		.min_fll = 4096,
		.comp_items = 0,
		.ctrl_data = 0,
		.mode_regs_items = 0,
		.mode_regs = NULL,
	},
	{
		.sd_rects_items = ARRAY_SIZE(pixter_stub_full_rects),
		.sd_rects = pixter_stub_full_rects,
		.scale_m = 1,
		.width = 4096,
		.height = 3072,
		.min_llp = 6024,
		.min_fll = 4096,
		.comp_items = 0,
		.ctrl_data = 0,
		.mode_regs_items = 0,
		.mode_regs = NULL,
	},
	{
		.sd_rects_items = ARRAY_SIZE(pixter_stub_480i_rects),
		.sd_rects = pixter_stub_480i_rects,
		.scale_m = 1,
		.width = 720,
		.height = 240,
		.min_llp = 6024,
		.min_fll = 4096,
		.comp_items = 0,
		.ctrl_data = 0,
		.mode_regs_items = 0,
		.mode_regs = NULL,
	},
	{
		.sd_rects_items = ARRAY_SIZE(pixter_stub_576i_rects),
		.sd_rects = pixter_stub_576i_rects,
		.scale_m = 1,
		.width = 720,
		.height = 288,
		.min_llp = 6024,
		.min_fll = 4096,
		.comp_items = 0,
		.ctrl_data = 0,
				.mode_regs_items = 0,
		.mode_regs = NULL,
	},
	{
		.sd_rects_items = ARRAY_SIZE(pixter_stub_1080i_rects),
		.sd_rects = pixter_stub_1080i_rects,
		.scale_m = 1,
		.width = 1920,
		.height = 540,
		.min_llp = 6024,
		.min_fll = 4096,
		.comp_items = 0,
		.ctrl_data = 0,
		.mode_regs_items = 0,
		.mode_regs = NULL,
	},
};

static struct crl_sensor_subdev_config pixter_stub_sensor_subdevs[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_SCALER,
		.name = "pixter_stub scaler",
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.name = "pixter_stub pixel array",
	},
};

static struct crl_sensor_subdev_config pixter_stub_b_sensor_subdevs[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_SCALER,
		.name = "pixter_stubB scaler",
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.name = "pixter_stubB pixel array",
	},
};

static struct crl_sensor_subdev_config pixter_stub_c_sensor_subdevs[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_SCALER,
		.name = "pixter_stubC scaler",
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.name = "pixter_stubC pixel array",
	},
};

static struct crl_sensor_subdev_config pixter_stub_d_sensor_subdevs[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_SCALER,
		.name = "pixter_stubD scaler",
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.name = "pixter_stubD pixel array",
	},
};

static struct crl_sensor_subdev_config pixter_stub_e_sensor_subdevs[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_SCALER,
		.name = "pixter_stubE scaler",
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.name = "pixter_stubE pixel array",
	},
};

static struct crl_sensor_subdev_config pixter_stub_f_sensor_subdevs[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_SCALER,
		.name = "pixter_stubF scaler",
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.name = "pixter_stubF pixel array",
	},
};

static struct crl_sensor_subdev_config pixter_stub_g_sensor_subdevs[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_SCALER,
		.name = "pixter_stubG scaler",
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.name = "pixter_stubG pixel array",
	},
};

static struct crl_sensor_subdev_config pixter_stub_h_sensor_subdevs[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_SCALER,
		.name = "pixter_stubH scaler",
	},
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.name = "pixter_stubH pixel array",
	},
};

static struct crl_sensor_limits pixter_stub_sensor_limits = {
	.x_addr_min = 0,
	.y_addr_min = 0,
	.x_addr_max = 4096,
	.y_addr_max = 3072,
	.min_frame_length_lines = 160,
	.max_frame_length_lines = 8192,
	.min_line_length_pixels = 6024,
	.max_line_length_pixels = 8192,
	.scaler_m_min = 1,
	.scaler_m_max = 1,
	.scaler_n_min = 1,
	.scaler_n_max = 1,
	.min_even_inc = 1,
	.max_even_inc = 1,
	.min_odd_inc = 1,
	.max_odd_inc = 1,
};

/* no flip for pixter stub as no real sensor HW */
static struct crl_flip_data pixter_stub_flip_configurations[] = {
	{
		.flip = CRL_FLIP_DEFAULT_NONE,
		.pixel_order = CRL_PIXEL_ORDER_IGNORE,
	},
	{
		.flip = CRL_FLIP_VFLIP,
		.pixel_order = CRL_PIXEL_ORDER_IGNORE,
	},
	{
		.flip = CRL_FLIP_HFLIP,
		.pixel_order = CRL_PIXEL_ORDER_IGNORE,
	},
	{
		.flip = CRL_FLIP_HFLIP_VFLIP,
		.pixel_order = CRL_PIXEL_ORDER_IGNORE,
	},
};

static struct crl_csi_data_fmt pixter_stub_crl_csi_data_fmt[] = {
	{
		.code = MEDIA_BUS_FMT_SGRBG8_1X8,
		.pixel_order = CRL_PIXEL_ORDER_GRBG,
		.bits_per_pixel = 8,
	},
	{
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.pixel_order = CRL_PIXEL_ORDER_RGGB,
		.bits_per_pixel = 8,
	},
	{
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.pixel_order = CRL_PIXEL_ORDER_BGGR,
		.bits_per_pixel = 8,
	},
	{
		.code = MEDIA_BUS_FMT_SGBRG8_1X8,
		.pixel_order = CRL_PIXEL_ORDER_GBRG,
		.bits_per_pixel = 8,
	},
	{
		.code = MEDIA_BUS_FMT_UYVY8_1X16,
		.pixel_order = CRL_PIXEL_ORDER_IGNORE,
		.bits_per_pixel = 16,
	},
	{
		.code = MEDIA_BUS_FMT_YUYV8_1X16,
		.pixel_order = CRL_PIXEL_ORDER_IGNORE,
		.bits_per_pixel = 16,
	},
	{
		.code = MEDIA_BUS_FMT_RGB565_1X16,
		.pixel_order = CRL_PIXEL_ORDER_IGNORE,
		.bits_per_pixel = 16,
	},
	{
		.code = MEDIA_BUS_FMT_RGB888_1X24,
		.pixel_order = CRL_PIXEL_ORDER_IGNORE,
		.bits_per_pixel = 24,
	},
	{
		.code = MEDIA_BUS_FMT_YUYV10_1X20,
		.pixel_order = CRL_PIXEL_ORDER_IGNORE,
		.bits_per_pixel = 20,
	},
	{
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.pixel_order = CRL_PIXEL_ORDER_GRBG,
		.bits_per_pixel = 10,
	},
	{
		.code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.pixel_order = CRL_PIXEL_ORDER_RGGB,
		.bits_per_pixel = 10,
	},
	{
		.code = MEDIA_BUS_FMT_SBGGR10_1X10,
		.pixel_order = CRL_PIXEL_ORDER_BGGR,
		.bits_per_pixel = 10,
	},
	{
		.code = MEDIA_BUS_FMT_SGBRG10_1X10,
		.pixel_order = CRL_PIXEL_ORDER_GBRG,
		.bits_per_pixel = 10,
	},
	{
		.code = MEDIA_BUS_FMT_SGRBG12_1X12,
		.pixel_order = CRL_PIXEL_ORDER_GRBG,
		.bits_per_pixel = 12,
	},
	{
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.pixel_order = CRL_PIXEL_ORDER_RGGB,
		.bits_per_pixel = 12,
	},
	{
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.pixel_order = CRL_PIXEL_ORDER_BGGR,
		.bits_per_pixel = 12,
	},
	{
		.code = MEDIA_BUS_FMT_SGBRG12_1X12,
		.pixel_order = CRL_PIXEL_ORDER_GBRG,
		.bits_per_pixel = 12,
	},
};

static struct crl_v4l2_ctrl pixter_stub_v4l2_ctrls[] = {
	{
		.sd_type = CRL_SUBDEV_TYPE_SCALER,
		.op_type = CRL_V4L2_CTRL_SET_OP,
		.context = SENSOR_IDLE,
		.ctrl_id = V4L2_CID_LINK_FREQ,
		.name = "V4L2_CID_LINK_FREQ",
		.type = CRL_V4L2_CTRL_TYPE_MENU_INT,
		.data.v4l2_int_menu.def = 0,
		.data.v4l2_int_menu.max = 0,
		.data.v4l2_int_menu.menu = 0,
		.flags = 0,
		.impact = CRL_IMPACTS_NO_IMPACT,
		.ctrl = 0,
		.regs_items = 0,
		.regs = 0,
		.dep_items = 0,
		.dep_ctrls = 0,
	},
	{
		.sd_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.op_type = CRL_V4L2_CTRL_GET_OP,
		.context = SENSOR_POWERED_ON,
		.ctrl_id = V4L2_CID_PIXEL_RATE,
		.name = "V4L2_CID_PIXEL_RATE_PA",
		.type = CRL_V4L2_CTRL_TYPE_INTEGER,
		.data.std_data.min = 0,
		.data.std_data.max = 800000000,
		.data.std_data.step = 1,
		.data.std_data.def = 0,
		.flags = 0,
		.impact = CRL_IMPACTS_NO_IMPACT,
		.ctrl = 0,
		.regs_items = 0,
		.regs = 0,
		.dep_items = 0,
		.dep_ctrls = 0,
	},
	{
		.sd_type = CRL_SUBDEV_TYPE_SCALER,
		.op_type = CRL_V4L2_CTRL_GET_OP,
		.context = SENSOR_POWERED_ON,
		.ctrl_id = V4L2_CID_PIXEL_RATE,
		.name = "V4L2_CID_PIXEL_RATE_CSI",
		.type = CRL_V4L2_CTRL_TYPE_INTEGER,
		.data.std_data.min = 0,
		.data.std_data.max = 800000000,
		.data.std_data.step = 1,
		.data.std_data.def = 0,
		.flags = 0,
		.impact = CRL_IMPACTS_NO_IMPACT,
		.ctrl = 0,
		.regs_items = 0,
		.regs = 0,
		.dep_items = 0,
		.dep_ctrls = 0,
	},
	{
		.sd_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.op_type = CRL_V4L2_CTRL_SET_OP,
		.context = SENSOR_POWERED_ON,
		.ctrl_id = V4L2_CID_ANALOGUE_GAIN,
		.name = "V4L2_CID_ANALOGUE_GAIN",
		.type = CRL_V4L2_CTRL_TYPE_INTEGER,
		.data.std_data.min = 0,
		.data.std_data.max = 4096,
		.data.std_data.step = 1,
		.data.std_data.def = 128,
		.flags = 0,
		.impact = CRL_IMPACTS_NO_IMPACT,
		.ctrl = 0,
		.regs_items = 0,
		.regs = 0,
		.dep_items = 0,
		.dep_ctrls = 0,
	},
	{
		.sd_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.op_type = CRL_V4L2_CTRL_SET_OP,
		.context = SENSOR_POWERED_ON,
		.ctrl_id = V4L2_CID_EXPOSURE,
		.name = "V4L2_CID_EXPOSURE",
		.type = CRL_V4L2_CTRL_TYPE_INTEGER,
		.data.std_data.min = 0,
		.data.std_data.max = 65500,
		.data.std_data.step = 1,
		.data.std_data.def = 0,
		.flags = 0,
		.impact = CRL_IMPACTS_NO_IMPACT,
		.ctrl = 0,
		.regs_items = 0,
		.regs = 0,
		.dep_items = 0,
		.dep_ctrls = 0,
	},
	{
		.sd_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.op_type = CRL_V4L2_CTRL_SET_OP,
		.context = SENSOR_POWERED_ON,
		.ctrl_id = CRL_CID_EXPOSURE_SHS1,
		.type = CRL_V4L2_CTRL_TYPE_CUSTOM,
		.name = "CRL_CID_EXPOSURE_SHS1",
		.data.std_data.min = 4,
		.data.std_data.max = 65500,
		.data.std_data.step = 1,
		.data.std_data.def = 0x5500,
		.flags = V4L2_CTRL_FLAG_UPDATE,
		.impact = CRL_IMPACTS_NO_IMPACT,
		.ctrl = 0,
		.regs_items = 0,
		.regs = 0,
		.dep_items = 0,
		.dep_ctrls = 0,
		.v4l2_type = V4L2_CTRL_TYPE_INTEGER
	},
	{
		.sd_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.op_type = CRL_V4L2_CTRL_SET_OP,
		.context = SENSOR_POWERED_ON,
		.ctrl_id = CRL_CID_EXPOSURE_SHS2,
		.type = CRL_V4L2_CTRL_TYPE_CUSTOM,
		.name = "CRL_CID_EXPOSURE_SHS2",
		.data.std_data.min = 4,
		.data.std_data.max = 65500,
		.data.std_data.step = 1,
		.data.std_data.def = 0x500,
		.flags = V4L2_CTRL_FLAG_UPDATE,
		.impact = CRL_IMPACTS_NO_IMPACT,
		.ctrl = 0,
		.regs_items = 0,
		.regs = 0,
		.dep_items = 0,
		.dep_ctrls = 0,
		.v4l2_type = V4L2_CTRL_TYPE_INTEGER
	},
	{
		.sd_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.op_type = CRL_V4L2_CTRL_SET_OP,
		.context = SENSOR_POWERED_ON,
		.ctrl_id = CRL_CID_EXPOSURE_SHS3,
		.type = CRL_V4L2_CTRL_TYPE_CUSTOM,
		.name = "CRL_CID_EXPOSURE_SHS3",
		.data.std_data.min = 4,
		.data.std_data.max = 65500,
		.data.std_data.step = 1,
		.data.std_data.def = 0x1000,
		.flags = V4L2_CTRL_FLAG_UPDATE,
		.impact = CRL_IMPACTS_NO_IMPACT,
		.ctrl = 0,
		.regs_items = 0,
		.regs = 0,
		.dep_items = 0,
		.dep_ctrls = 0,
		.v4l2_type = V4L2_CTRL_TYPE_INTEGER
	},
	{
		.sd_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.op_type = CRL_V4L2_CTRL_SET_OP,
		.context = SENSOR_POWERED_ON,
		.ctrl_id = V4L2_CID_HFLIP,
		.name = "V4L2_CID_HFLIP",
		.type = CRL_V4L2_CTRL_TYPE_INTEGER,
		.data.std_data.min = 0,
		.data.std_data.max = 1,
		.data.std_data.step = 1,
		.data.std_data.def = 0,
		.flags = 0,
		.impact = CRL_IMPACTS_NO_IMPACT,
		.ctrl = 0,
		.regs_items = 0,
		.regs = 0,
		.dep_items = 0,
		.dep_ctrls = 0,
	},
	{
		.sd_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.op_type = CRL_V4L2_CTRL_SET_OP,
		.context = SENSOR_POWERED_ON,
		.ctrl_id = V4L2_CID_VFLIP,
		.name = "V4L2_CID_VFLIP",
		.type = CRL_V4L2_CTRL_TYPE_INTEGER,
		.data.std_data.min = 0,
		.data.std_data.max = 1,
		.data.std_data.step = 1,
		.data.std_data.def = 0,
		.flags = 0,
		.impact = CRL_IMPACTS_NO_IMPACT,
		.ctrl = 0,
		.regs_items = 0,
		.regs = 0,
		.dep_items = 0,
		.dep_ctrls = 0,
	},
	{
		.sd_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.op_type = CRL_V4L2_CTRL_SET_OP,
		.context = SENSOR_POWERED_ON,
		.ctrl_id = V4L2_CID_FRAME_LENGTH_LINES,
		.name = "Frame length lines",
		.type = CRL_V4L2_CTRL_TYPE_CUSTOM,
		.data.std_data.min = 160,
		.data.std_data.max = 65535,
		.data.std_data.step = 1,
		.data.std_data.def = 4130,
		.flags = V4L2_CTRL_FLAG_UPDATE,
		.impact = CRL_IMPACTS_NO_IMPACT,
		.ctrl = 0,
		.regs_items = 0,
		.regs = 0,
		.dep_items = 0,
		.dep_ctrls = 0,
		.v4l2_type = V4L2_CTRL_TYPE_INTEGER,
	},
	{
		.sd_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.op_type = CRL_V4L2_CTRL_SET_OP,
		.context = SENSOR_POWERED_ON,
		.ctrl_id = V4L2_CID_LINE_LENGTH_PIXELS,
		.name = "Line Length Pixels",
		.type = CRL_V4L2_CTRL_TYPE_CUSTOM,
		.data.std_data.min = 6024,
		.data.std_data.max = 65520,
		.data.std_data.step = 1,
		.data.std_data.def = 6024,
		.flags = V4L2_CTRL_FLAG_UPDATE,
		.impact = CRL_IMPACTS_NO_IMPACT,
		.ctrl = 0,
		.regs_items = 0,
		.regs = 0,
		.dep_items = 0,
		.dep_ctrls = 0,
		.v4l2_type = V4L2_CTRL_TYPE_INTEGER,
	},
	{
		.sd_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.op_type = CRL_V4L2_CTRL_SET_OP,
		.context = SENSOR_POWERED_ON,
		.ctrl_id = V4L2_CID_GAIN,
		.name = "Digital Gain",
		.type = CRL_V4L2_CTRL_TYPE_INTEGER,
		.data.std_data.min = 0,
		.data.std_data.max = 4095,
		.data.std_data.step = 1,
		.data.std_data.def = 1024,
		.flags = 0,
		.impact = CRL_IMPACTS_NO_IMPACT,
		.ctrl = 0,
		.regs_items = 0,
		.regs = 0,
		.dep_items = 0,
		.dep_ctrls = 0,
	},
	{
		.sd_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.op_type = CRL_V4L2_CTRL_SET_OP,
		.context = SENSOR_POWERED_ON,
		.ctrl_id = CRL_CID_ANALOG_GAIN_L,
		.name = "CRL_CID_ANALOG_GAIN_L",
		.type = CRL_V4L2_CTRL_TYPE_CUSTOM,
		.data.std_data.min = 0,
		.data.std_data.max = 0x978,
		.data.std_data.step = 1,
		.data.std_data.def = 0,
		.flags = 0,
		.impact = CRL_IMPACTS_NO_IMPACT,
		.ctrl = 0,
		.regs_items = 0,
		.regs = 0,
		.dep_items = 0,
		.dep_ctrls = 0,
		.v4l2_type = V4L2_CTRL_TYPE_INTEGER,
	},
	{
		.sd_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.op_type = CRL_V4L2_CTRL_SET_OP,
		.context = SENSOR_POWERED_ON,
		.ctrl_id = CRL_CID_ANALOG_GAIN_S,
		.name = "CRL_CID_ANALOG_GAIN_S",
		.type = CRL_V4L2_CTRL_TYPE_CUSTOM,
		.data.std_data.min = 0,
		.data.std_data.max = 0x978,
		.data.std_data.step = 1,
		.data.std_data.def = 0,
		.flags = V4L2_CTRL_FLAG_UPDATE,
		.impact = CRL_IMPACTS_NO_IMPACT,
		.ctrl = 0,
		.regs_items = 0,
		.regs = 0,
		.dep_items = 0,
		.dep_ctrls = 0,
		.v4l2_type = V4L2_CTRL_TYPE_INTEGER,
	},
	{
		.sd_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.op_type = CRL_V4L2_CTRL_SET_OP,
		.context = SENSOR_POWERED_ON,
		.ctrl_id = CRL_CID_ANALOG_GAIN_VS,
		.name = "CRL_CID_ANALOG_GAIN_VS",
		.type = CRL_V4L2_CTRL_TYPE_CUSTOM,
		.data.std_data.min = 0,
		.data.std_data.max = 0x978,
		.data.std_data.step = 1,
		.data.std_data.def = 0,
		.flags = V4L2_CTRL_FLAG_UPDATE,
		.impact = CRL_IMPACTS_NO_IMPACT,
		.ctrl = 0,
		.regs_items = 0,
		.regs = 0,
		.dep_items = 0,
		.dep_ctrls = 0,
		.v4l2_type = V4L2_CTRL_TYPE_INTEGER,
	},
	{
		.sd_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.op_type = CRL_V4L2_CTRL_SET_OP,
		.context = SENSOR_POWERED_ON,
		.ctrl_id = CRL_CID_DIGITAL_GAIN_L,
		.name = "CRL_CID_DIGITAL_GAIN_L",
		.type = CRL_V4L2_CTRL_TYPE_CUSTOM,
		.data.std_data.min = 0,
		.data.std_data.max = 0x978,
		.data.std_data.step = 1,
		.data.std_data.def = 64,
		.flags = V4L2_CTRL_FLAG_UPDATE,
		.impact = CRL_IMPACTS_NO_IMPACT,
		.ctrl = 0,
		.regs_items = 0,
		.regs = 0,
		.dep_items = 0,
		.dep_ctrls = 0,
		.v4l2_type = V4L2_CTRL_TYPE_INTEGER,
	},
	{
		.sd_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.op_type = CRL_V4L2_CTRL_SET_OP,
		.context = SENSOR_POWERED_ON,
		.ctrl_id = CRL_CID_DIGITAL_GAIN_S,
		.name = "CRL_CID_DIGITAL_GAIN_S",
		.type = CRL_V4L2_CTRL_TYPE_CUSTOM,
		.data.std_data.min = 0,
		.data.std_data.max = 0x978,
		.data.std_data.step = 1,
		.data.std_data.def = 64,
		.flags = V4L2_CTRL_FLAG_UPDATE,
		.impact = CRL_IMPACTS_NO_IMPACT,
		.ctrl = 0,
		.regs_items = 0,
		.regs = 0,
		.dep_items = 0,
		.dep_ctrls = 0,
		.v4l2_type = V4L2_CTRL_TYPE_INTEGER,
	},
	{
		.sd_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.op_type = CRL_V4L2_CTRL_SET_OP,
		.context = SENSOR_POWERED_ON,
		.ctrl_id = CRL_CID_DIGITAL_GAIN_VS,
		.name = "CRL_CID_DIGITAL_GAIN_VS",
		.type = CRL_V4L2_CTRL_TYPE_CUSTOM,
		.data.std_data.min = 0,
		.data.std_data.max = 0x978,
		.data.std_data.step = 1,
		.data.std_data.def = 64,
		.flags = V4L2_CTRL_FLAG_UPDATE,
		.impact = CRL_IMPACTS_NO_IMPACT,
		.ctrl = 0,
		.regs_items = 0,
		.regs = 0,
		.dep_items = 0,
		.dep_ctrls = 0,
		.v4l2_type = V4L2_CTRL_TYPE_INTEGER,
	},
};

static struct crl_arithmetic_ops pixter_stub_frame_desc_width_ops[] = {
	{
		.op = CRL_ASSIGNMENT,
		.operand.entity_type = CRL_DYNAMIC_VAL_OPERAND_TYPE_VAR_REF,
		.operand.entity_val = CRL_VAR_REF_OUTPUT_WIDTH,
	},
};

static struct crl_arithmetic_ops pixter_stub_frame_desc_height_ops[] = {
	{
		.op = CRL_ASSIGNMENT,
		.operand.entity_type = CRL_DYNAMIC_VAL_OPERAND_TYPE_CONST,
		.operand.entity_val = 1,
	},
};

static struct crl_frame_desc pixter_stub_frame_desc[] = {
	{
		.flags.entity_val = 0,
		.bpp.entity_type = CRL_DYNAMIC_VAL_OPERAND_TYPE_VAR_REF,
		.bpp.entity_val = CRL_VAR_REF_BITSPERPIXEL,
		.pixelcode.entity_val = MEDIA_BUS_FMT_FIXED,
		.length.entity_val = 0,
		.start_line.entity_val = 0,
		.start_pixel.entity_val = 0,
		.width = {
			.ops_items = ARRAY_SIZE(pixter_stub_frame_desc_width_ops),
			.ops = pixter_stub_frame_desc_width_ops,
		},
		.height = {
			.ops_items = ARRAY_SIZE(pixter_stub_frame_desc_height_ops),
			.ops = pixter_stub_frame_desc_height_ops,
		},
		.csi2_channel.entity_val = 0,
		.csi2_data_type.entity_val = 0x12,
	},
	{
		.flags.entity_val = 0,
		.bpp.entity_type = CRL_DYNAMIC_VAL_OPERAND_TYPE_VAR_REF,
		.bpp.entity_val = CRL_VAR_REF_BITSPERPIXEL,
		.pixelcode.entity_val = MEDIA_BUS_FMT_FIXED,
		.length.entity_val = 0,
		.start_line.entity_val = 0,
		.start_pixel.entity_val = 0,
		.width = {
			.ops_items = ARRAY_SIZE(pixter_stub_frame_desc_width_ops),
			.ops = pixter_stub_frame_desc_width_ops,
		},
		.height = {
			.ops_items = ARRAY_SIZE(pixter_stub_frame_desc_height_ops),
			.ops = pixter_stub_frame_desc_height_ops,
		},
		.csi2_channel.entity_val = 1,
		.csi2_data_type.entity_val = 0x12,
	},
	{
		.flags.entity_val = 0,
		.bpp.entity_type = CRL_DYNAMIC_VAL_OPERAND_TYPE_VAR_REF,
		.bpp.entity_val = CRL_VAR_REF_BITSPERPIXEL,
		.pixelcode.entity_val = MEDIA_BUS_FMT_FIXED,
		.length.entity_val = 0,
		.start_line.entity_val = 0,
		.start_pixel.entity_val = 0,
		.width = {
			.ops_items = ARRAY_SIZE(pixter_stub_frame_desc_width_ops),
			.ops = pixter_stub_frame_desc_width_ops,
		},
		.height = {
			.ops_items = ARRAY_SIZE(pixter_stub_frame_desc_height_ops),
			.ops = pixter_stub_frame_desc_height_ops,
		},
		.csi2_channel.entity_val = 2,
		.csi2_data_type.entity_val = 0x12,
	},
	{
		.flags.entity_val = 0,
		.bpp.entity_type = CRL_DYNAMIC_VAL_OPERAND_TYPE_VAR_REF,
		.bpp.entity_val = CRL_VAR_REF_BITSPERPIXEL,
		.pixelcode.entity_val = MEDIA_BUS_FMT_FIXED,
		.length.entity_val = 0,
		.start_line.entity_val = 0,
		.start_pixel.entity_val = 0,
		.width = {
			.ops_items = ARRAY_SIZE(pixter_stub_frame_desc_width_ops),
			.ops = pixter_stub_frame_desc_width_ops,
		},
		.height = {
			.ops_items = ARRAY_SIZE(pixter_stub_frame_desc_height_ops),
			.ops = pixter_stub_frame_desc_height_ops,
		},
		.csi2_channel.entity_val = 3,
		.csi2_data_type.entity_val = 0x12,
	},
};

#define DEFINE_PIXTER_CRL_CONFIGURATION(port) \
static struct crl_sensor_configuration pixter_##port##_crl_configuration = { \
	.powerup_regs_items = 0, \
	.powerup_regs = NULL, \
\
	.poweroff_regs_items = 0, \
	.poweroff_regs = NULL, \
\
	.id_reg_items = 0, \
	.id_regs = NULL, \
\
	.subdev_items = ARRAY_SIZE(pixter_##port##_sensor_subdevs), \
	.subdevs = pixter_##port##_sensor_subdevs, \
\
	.sensor_limits = &pixter_stub_sensor_limits, \
\
	.pll_config_items = ARRAY_SIZE(pixter_stub_pll_configurations), \
	.pll_configs = pixter_stub_pll_configurations, \
\
	.modes_items = ARRAY_SIZE(pixter_stub_modes), \
	.modes = pixter_stub_modes, \
\
	.streamon_regs_items = 0, \
	.streamon_regs = NULL, \
\
	.streamoff_regs_items = 0, \
	.streamoff_regs = NULL, \
\
	.v4l2_ctrls_items = ARRAY_SIZE(pixter_stub_v4l2_ctrls), \
	.v4l2_ctrl_bank = pixter_stub_v4l2_ctrls, \
\
	.flip_items = ARRAY_SIZE(pixter_stub_flip_configurations), \
	.flip_data = pixter_stub_flip_configurations, \
\
	.frame_desc_entries = ARRAY_SIZE(pixter_stub_frame_desc), \
	.frame_desc_type = CRL_V4L2_MBUS_FRAME_DESC_TYPE_CSI2, \
	.frame_desc = pixter_stub_frame_desc, \
\
	.csi_fmts_items = ARRAY_SIZE(pixter_stub_crl_csi_data_fmt), \
	.csi_fmts = pixter_stub_crl_csi_data_fmt, \
}
DEFINE_PIXTER_CRL_CONFIGURATION(stub);
DEFINE_PIXTER_CRL_CONFIGURATION(stub_b);
DEFINE_PIXTER_CRL_CONFIGURATION(stub_c);
DEFINE_PIXTER_CRL_CONFIGURATION(stub_d);
DEFINE_PIXTER_CRL_CONFIGURATION(stub_e);
DEFINE_PIXTER_CRL_CONFIGURATION(stub_f);
DEFINE_PIXTER_CRL_CONFIGURATION(stub_g);
DEFINE_PIXTER_CRL_CONFIGURATION(stub_h);


#endif  /* __CRLMODULE_PIXTER_STUB_CONFIGURATION_H_ */
