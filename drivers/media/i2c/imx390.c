// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 Intel Corporation.

#include <asm/unaligned.h>
#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <linux/version.h>
#include <media/imx390.h>

#define IMX390_LINK_FREQ_360MHZ		360000000ULL
#define IMX390_LINK_FREQ_300MHZ		300000000ULL
#define IMX390_LINK_FREQ_288MHZ		288000000ULL
#define IMX390_LINK_FREQ_240MHZ		240000000ULL

#define FIXED_POINT_SCALING_FACTOR (1ULL << 22)

#define IMX390_REG_VALUE_08BIT		1
#define IMX390_REG_VALUE_16BIT		2

#define IMX390_REG_CHIP_ID		0x0330
#define IMX390_CHIP_ID			0x0

/* vertical-timings from sensor */
#define IMX390_REG_VTS			0x300A
#define IMX390_VTS_MAX			0xffff

#define IMX390_CLK_FREQ			(1485000)

/* Exposure controls from sensor */
#define IMX390_REG_EXPOSURE		0x3012
#define	IMX390_EXPOSURE_MIN		(30 * FIXED_POINT_SCALING_FACTOR / 1000000)
#define	IMX390_EXPOSURE_MAX		(33333 * FIXED_POINT_SCALING_FACTOR / 1000000)
#define	IMX390_EXPOSURE_DEF		(11000 * FIXED_POINT_SCALING_FACTOR / 1000000)
#define	IMX390_EXPOSURE_STEP		(1 * FIXED_POINT_SCALING_FACTOR / 1000000)

/* Analog gain controls from sensor */
#define	IMX390_ANAL_GAIN_MIN		0
#define	IMX390_ANAL_GAIN_MAX		0x64
#define	IMX390_ANAL_GAIN_STEP		1
#define	IMX390_ANAL_GAIN_DEFAULT	0x1c

/* Digital gain controls from sensor */
#define IMX390_DGTL_GAIN_MIN		0
#define IMX390_DGTL_GAIN_MAX		0x7ff
#define IMX390_DGTL_GAIN_STEP		1
#define IMX390_DGTL_GAIN_DEFAULT	0x80

#define IMX390_GAIN_MIN			0
#define IMX390_GAIN_DEFAULT		0x80

#define IMX390_RED_BALANCE_MIN		0
#define IMX390_RED_BALANCE_MAX		0xfff
#define IMX390_RED_BALANCE_STEP		1
#define IMX390_RED_BALANCE_DEF		0x100

#define IMX390_BLUE_BALANCE_MIN		0
#define IMX390_BLUE_BALANCE_MAX		0xfff
#define IMX390_BLUE_BALANCE_STEP	1
#define IMX390_BLUE_BALANCE_DEF		0x100

#define MAX(a, b)		(((a) > (b)) ? (a) : (b))

#define IMX390_CID_CSI_PORT         (V4L2_CID_USER_BASE | 0x1001)
#define IMX390_CID_I2C_BUS         (V4L2_CID_USER_BASE | 0x1002)
#define IMX390_CID_I2C_ID         (V4L2_CID_USER_BASE | 0x1003)
#define IMX390_CID_I2C_SLAVE_ADDRESS         (V4L2_CID_USER_BASE | 0x1004)
#define IMX390_CID_FPS         (V4L2_CID_USER_BASE | 0x1005)
#define IMX390_CID_FRAME_INTERVAL	(V4L2_CID_USER_BASE | 0x1006)

/*
 * dummy CID
 */
#define V4L2_CID_IMX390_BASE		(V4L2_CID_USER_BASE + 0x2050)

#define V4L2_CID_FRAME_LENGTH_LINES (V4L2_CID_IMX390_BASE + 1)
#define V4L2_CID_LINE_LENGTH_PIXELS (V4L2_CID_IMX390_BASE + 2)
#define IMX390_CID_SENSOR_THERMAL_DATA (V4L2_CID_IMX390_BASE + 3)

/*
 * Select sensor mode directly, driver programs media pad
 * formats as in configuration file
 */
#define IMX390_CID_SENSOR_MODE (V4L2_CID_IMX390_BASE + 4)

/* IMX230 HDR specific controls */
#define IMX390_CID_IMX230_HDR_MODE		(V4L2_CID_IMX390_BASE + 5)
#define IMX390_CID_IMX230_HDR_ET_RATIO	(V4L2_CID_IMX390_BASE + 6)

/* Set multi-exposure frame in HDR with different exposure value */
#define IMX390_CID_EXPOSURE_SHS1		(V4L2_CID_IMX390_BASE + 8)
#define IMX390_CID_EXPOSURE_SHS2		(V4L2_CID_IMX390_BASE + 9)
#define IMX390_CID_EXPOSURE_SHS3		(V4L2_CID_IMX390_BASE + 10)
#define IMX390_CID_EXPOSURE_RHS1		(V4L2_CID_IMX390_BASE + 11)
#define IMX390_CID_EXPOSURE_RHS2		(V4L2_CID_IMX390_BASE + 12)

/* Switch to enable/disable PDAF settings */
#define IMX390_CID_SENSOR_PDAF		(V4L2_CID_IMX390_BASE + 13)

/* Set multi-digital gain */
#define IMX390_CID_DIGITAL_GAIN_L		(V4L2_CID_IMX390_BASE + 14)
#define IMX390_CID_DIGITAL_GAIN_S		(V4L2_CID_IMX390_BASE + 15)
#define IMX390_CID_DIGITAL_GAIN_VS		(V4L2_CID_IMX390_BASE + 16)

/* Get sensor bit linear */
#define IMX390_CID_SENSOR_BIT_LINEAR	(V4L2_CID_IMX390_BASE + 17)

/* set sensor msb align*/
#define IMX390_CID_MSB_ALIGN		(V4L2_CID_IMX390_BASE + 18)

/* enable/disable auto exposure */
#define IMX390_CID_AUTO_EXPOSURE_DEBUG	(V4L2_CID_IMX390_BASE + 19)

/* set analog gain for HDR frames */
#define IMX390_CID_ANALOG_GAIN_L		(V4L2_CID_IMX390_BASE + 20)
#define IMX390_CID_ANALOG_GAIN_S		(V4L2_CID_IMX390_BASE + 21)
#define IMX390_CID_ANALOG_GAIN_VS		(V4L2_CID_IMX390_BASE + 22)

/* Set exposure mode: Linear mode or 2-/3-/4-HDR mode */
#define IMX390_CID_EXPOSURE_MODE		(V4L2_CID_IMX390_BASE + 23)

/* Set HDR mode exposure ratio */
#define IMX390_CID_EXPOSURE_HDR_RATIO	(V4L2_CID_IMX390_BASE + 24)

/* choose hcg/lcg for linear analog */
#define IMX390_CID_ANALOG_LINEAR_CG	(V4L2_CID_IMX390_BASE + 25)

/* Digital gain controls from sensor */
#define IMX390_DUMMY_MIN		0
#define IMX390_DUMMY_MAX		0x7ff
#define IMX390_DUMMY_STEP		1
#define IMX390_DUMMY_DEF		0x1

/*
 * end of dummy CID
 */

#define to_imx390(_sd)			container_of(_sd, struct imx390, sd)

/**
 * Register addresses (see data sheet/register map)
 */
enum {
	IMX390_REG_STANDBY = 0x0000,
	IMX390_REG_REG_HOLD = 0x0008,
	IMX390_REG_SHS1 = 0x000c,
	IMX390_REG_SHS2 = 0x0010,
	IMX390_REG_AGAIN_SP1H = 0x0018,
	IMX390_REG_AGAIN_SP1L = 0x001a,

	/* default bayer order RGGB */
	IMX390_REG_WBGAIN_R = 0x0030,
	IMX390_REG_WBGAIN_GR = 0x0032,
	IMX390_REG_WBGAIN_GB = 0x0034,
	IMX390_REG_WBGAIN_B = 0x0036,

	IMX390_REG_OBB_CLAMP_CTRL_SEL = 0x0083,
	IMX390_REG_REAR_EMBDATA_LINE = 0x2E18,
	IMX390_REG_REV1 = 0x3060,
	IMX390_REG_REV2 = 0x3064,
	IMX390_REG_REV3 = 0x3067,
	IMX390_REG_WBGAIN_FORCE_X1 = 0x36A8,
};

enum {
	IMX390_LINK_FREQ_360MBPS,
	IMX390_LINK_FREQ_300MBPS,
	IMX390_LINK_FREQ_288MBPS,
	IMX390_LINK_FREQ_240MBPS,
};

struct imx390_reg {
	u16 address;
	u16 val;
};

struct imx390_reg_list {
	u32 num_of_regs;
	const struct imx390_reg *regs;
};

struct imx390_link_freq_config {
	const struct imx390_reg_list reg_list;
};

struct imx390_mode {
	/* Frame width in pixels */
	u32 width;

	/* Frame height in pixels */
	u32 height;

	bool hdr_en;

	/* Horizontal timining size */
	u32 hts;

	/* Default vertical timining size */
	u32 vts_def;

	/* Min vertical timining size */
	u32 vts_min;

	/* Link frequency needed for this resolution */
	u32 link_freq_index;

	/* MEDIA_BUS_FMT */
	u32 code;

	/* MIPI_LANES */
	s32 lanes;

	/* MODE_FPS*/
	u32 fps;

	/* bit per pixel */
	u32 bpp;

	/* Sensor register settings for this resolution */
	const struct imx390_reg_list reg_list;
};

struct imx390 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;

	/* V4L2 Controls */
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *mipi_lanes;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *analogue_gain;
	struct v4l2_ctrl *digital_gain;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *strobe_source;
	struct v4l2_ctrl *strobe;
	struct v4l2_ctrl *strobe_stop;
	struct v4l2_ctrl *timeout;
	struct v4l2_ctrl *csi_port;
	struct v4l2_ctrl *i2c_bus;
	struct v4l2_ctrl *i2c_id;
	struct v4l2_ctrl *i2c_slave_address;
	struct v4l2_ctrl *fps;
	struct v4l2_ctrl *frame_interval;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *red_balance;
	struct v4l2_ctrl *blue_balance;
	struct v4l2_ctrl *dummy_exp_shs1;
	struct v4l2_ctrl *dummy_exp_shs2;
	struct v4l2_ctrl *dummy_exp_shs3;
	struct v4l2_ctrl *dummy_exp_rhs1;
	struct v4l2_ctrl *dummy_exp_rhs2;
	struct v4l2_ctrl *dummy_dg_l;
	struct v4l2_ctrl *dummy_dg_s;
	struct v4l2_ctrl *dummy_dg_vs;
	struct v4l2_ctrl *dummy_ag_l;
	struct v4l2_ctrl *dummy_ag_s;
	struct v4l2_ctrl *dummy_ag_vs;
	struct v4l2_ctrl *lsc_pattern;

	/* Current mode */
	const struct imx390_mode *cur_mode;
	/* Previous mode */
	const struct imx390_mode *pre_mode;

	/* To serialize asynchronus callbacks */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;

	struct imx390_platform_data *platform_data;
};

#include "imx390-mode-1280x960-CROP.h"
#include "imx390_mode_1920x1200HDR3_CUST_PWL12.h"

static int imx390_group_hold_enable(struct imx390 *imx390, s32 val);

enum {
	LSC_PATTERN_DISABLED = 0,
	LSC_PATTERN_UNITY,
	LSC_PATTERN_TABLE,
};

static const char * const lsc_qmenu[] = {
	"Disabled",
	"all 0x80",
	"table",
};

static const struct imx390_reg imx390_lsc_pattern_unity[] = {
	{0x4000, 0x80},
	{0x4001, 0x80},
	{0x4002, 0x80},
	{0x4003, 0x80},
	{0x4004, 0x80},
	{0x4005, 0x80},
	{0x4006, 0x80},
	{0x4007, 0x80},
	{0x4008, 0x80},
	{0x4009, 0x80},
	{0x400A, 0x80},
	{0x400B, 0x80},
	{0x400C, 0x80},
	{0x400D, 0x80},
	{0x400E, 0x80},
	{0x400F, 0x80},
	{0x4010, 0x80},
	{0x4011, 0x80},
	{0x4012, 0x80},
	{0x4013, 0x80},
	{0x4014, 0x80},
	{0x4015, 0x80},
	{0x4016, 0x80},
	{0x4017, 0x80},
	{0x4018, 0x80},
	{0x4019, 0x80},
	{0x401A, 0x80},
	{0x401B, 0x80},
	{0x401C, 0x80},
	{0x401D, 0x80},
	{0x401E, 0x80},
	{0x401F, 0x80},
	{0x4020, 0x80},
	{0x4021, 0x80},
	{0x4022, 0x80},
	{0x4023, 0x80},
	{0x4024, 0x80},
	{0x4025, 0x80},
	{0x4026, 0x80},
	{0x4027, 0x80},
	{0x4028, 0x80},
	{0x4029, 0x80},
	{0x402A, 0x80},
	{0x402B, 0x80},
	{0x402C, 0x80},
	{0x402D, 0x80},
	{0x402E, 0x80},
	{0x402F, 0x80},
	{0x4030, 0x80},
	{0x4031, 0x80},
	{0x4032, 0x80},
	{0x4033, 0x80},
	{0x4034, 0x80},
	{0x4035, 0x80},
	{0x4036, 0x80},
	{0x4037, 0x80},
	{0x4038, 0x80},
	{0x4039, 0x80},
	{0x403A, 0x80},
	{0x403B, 0x80},
	{0x403C, 0x80},
	{0x403D, 0x80},
	{0x403E, 0x80},
	{0x403F, 0x80},
	{0x4040, 0x80},
	{0x4041, 0x80},
	{0x4042, 0x80},
	{0x4043, 0x80},
	{0x4044, 0x80},
	{0x4045, 0x80},
	{0x4046, 0x80},
	{0x4047, 0x80},
	{0x4048, 0x80},
	{0x4049, 0x80},
	{0x404A, 0x80},
	{0x404B, 0x80},
	{0x404C, 0x80},
	{0x404D, 0x80},
	{0x404E, 0x80},
	{0x404F, 0x80},
	{0x4050, 0x80},
	{0x4051, 0x80},
	{0x4052, 0x80},
	{0x4053, 0x80},
	{0x4054, 0x80},
	{0x4055, 0x80},
	{0x4056, 0x80},
	{0x4057, 0x80},
	{0x4058, 0x80},
	{0x4059, 0x80},
	{0x405A, 0x80},
	{0x405B, 0x80},
	{0x405C, 0x80},
	{0x405D, 0x80},
	{0x405E, 0x80},
	{0x405F, 0x80},
	{0x4060, 0x80},
	{0x4061, 0x80},
	{0x4062, 0x80},
	{0x4063, 0x80},
	{0x4064, 0x80},
	{0x4065, 0x80},
	{0x4066, 0x80},
	{0x4067, 0x80},
	{0x4068, 0x80},
	{0x4069, 0x80},
	{0x406A, 0x80},
	{0x406B, 0x80},
	{0x406C, 0x80},
	{0x406D, 0x80},
	{0x406E, 0x80},
	{0x406F, 0x80},
	{0x4070, 0x80},
	{0x4071, 0x80},
	{0x4072, 0x80},
	{0x4073, 0x80},
	{0x4074, 0x80},
	{0x4075, 0x80},
	{0x4076, 0x80},
	{0x4077, 0x80},
	{0x4078, 0x80},
	{0x4079, 0x80},
	{0x407A, 0x80},
	{0x407B, 0x80},
	{0x407C, 0x80},
	{0x407D, 0x80},
	{0x407E, 0x80},
	{0x407F, 0x80},
	{0x4080, 0x80},
	{0x4081, 0x80},
	{0x4082, 0x80},
	{0x4083, 0x80},
	{0x4084, 0x80},
	{0x4085, 0x80},
	{0x4086, 0x80},
	{0x4087, 0x80},
	{0x4088, 0x80},
	{0x4089, 0x80},
	{0x408A, 0x80},
	{0x408B, 0x80},
	{0x408C, 0x80},
	{0x408D, 0x80},
	{0x408E, 0x80},
	{0x408F, 0x80},
	{0x4090, 0x80},
	{0x4091, 0x80},
	{0x4092, 0x80},
	{0x4093, 0x80},
	{0x4094, 0x80},
	{0x4095, 0x80},
	{0x4096, 0x80},
	{0x4097, 0x80},
	{0x4098, 0x80},
	{0x4099, 0x80},
	{0x409A, 0x80},
	{0x409B, 0x80},
	{0x409C, 0x80},
	{0x409D, 0x80},
	{0x409E, 0x80},
	{0x409F, 0x80},
	{0x40A0, 0x80},
	{0x40A1, 0x80},
	{0x40A2, 0x80},
	{0x40A3, 0x80},
	{0x40A4, 0x80},
	{0x40A5, 0x80},
	{0x40A6, 0x80},
	{0x40A7, 0x80},
	{0x40A8, 0x80},
	{0x40A9, 0x80},
	{0x40AA, 0x80},
	{0x40AB, 0x80},
	{0x40AC, 0x80},
	{0x40AD, 0x80},
	{0x40AE, 0x80},
	{0x40AF, 0x80},
	{0x40B0, 0x80},
	{0x40B1, 0x80},
	{0x40B2, 0x80},
	{0x40B3, 0x80},
	{0x40B4, 0x80},
	{0x40B5, 0x80},
	{0x40B6, 0x80},
	{0x40B7, 0x80},
	{0x40B8, 0x80},
	{0x40B9, 0x80},
	{0x40BA, 0x80},
	{0x40BB, 0x80},
	{0x40BC, 0x80},
	{0x40BD, 0x80},
	{0x40BE, 0x80},
	{0x40BF, 0x80},
	{0x40C0, 0x80},
	{0x40C1, 0x80},
	{0x40C2, 0x80},
	{0x40C3, 0x80},
	{0x40C4, 0x80},
	{0x40C5, 0x80},
	{0x40C6, 0x80},
	{0x40C7, 0x80},
	{0x40C8, 0x80},
	{0x40C9, 0x80},
	{0x40CA, 0x80},
	{0x40CB, 0x80},
	{0x40CC, 0x80},
	{0x40CD, 0x80},
	{0x40CE, 0x80},
	{0x40CF, 0x80},
	{0x40D0, 0x80},
	{0x40D1, 0x80},
	{0x40D2, 0x80},
	{0x40D3, 0x80},
	{0x40D4, 0x80},
	{0x40D5, 0x80},
	{0x40D6, 0x80},
	{0x40D7, 0x80},
};

static const struct imx390_reg_list lsc_unity_list = {
	.num_of_regs = ARRAY_SIZE(imx390_lsc_pattern_unity),
	.regs = imx390_lsc_pattern_unity,
};

static const struct imx390_reg imx390_lsc_pattern_vendor_def[] = {
	{0x01D0, 0x01}, /* SHD_ON                    */
	{0x3AF6, 0x00}, /* SHD_DIFF_ACCURACY         */
	{0x4000, 0x64}, /* SHD_KNOT_1_R0             */
	{0x4001, 0x5D}, /* SHD_KNOT_1_GR0            */
	{0x4002, 0x5D}, /* SHD_KNOT_1_GB0            */
	{0x4003, 0x5B}, /* SHD_KNOT_1_B0             */
	{0x4004, 0x6D}, /* SHD_KNOT_1_R1             */
	{0x4005, 0x69}, /* SHD_KNOT_1_GR1            */
	{0x4006, 0x69}, /* SHD_KNOT_1_GB1            */
	{0x4007, 0x67}, /* SHD_KNOT_1_B1             */
	{0x4008, 0x74}, /* SHD_KNOT_1_R2             */
	{0x4009, 0x71}, /* SHD_KNOT_1_GR2            */
	{0x400A, 0x71}, /* SHD_KNOT_1_GB2            */
	{0x400B, 0x70}, /* SHD_KNOT_1_B2             */
	{0x400C, 0x77}, /* SHD_KNOT_1_R3             */
	{0x400D, 0x76}, /* SHD_KNOT_1_GR3            */
	{0x400E, 0x76}, /* SHD_KNOT_1_GB3            */
	{0x400F, 0x75}, /* SHD_KNOT_1_B3             */
	{0x4010, 0x78}, /* SHD_KNOT_1_R4             */
	{0x4011, 0x77}, /* SHD_KNOT_1_GR4            */
	{0x4012, 0x77}, /* SHD_KNOT_1_GB4            */
	{0x4013, 0x76}, /* SHD_KNOT_1_B4             */
	{0x4014, 0x77}, /* SHD_KNOT_1_R5             */
	{0x4015, 0x75}, /* SHD_KNOT_1_GR5            */
	{0x4016, 0x75}, /* SHD_KNOT_1_GB5            */
	{0x4017, 0x74}, /* SHD_KNOT_1_B5             */
	{0x4018, 0x72}, /* SHD_KNOT_1_R6             */
	{0x4019, 0x70}, /* SHD_KNOT_1_GR6            */
	{0x401A, 0x70}, /* SHD_KNOT_1_GB6            */
	{0x401B, 0x6F}, /* SHD_KNOT_1_B6             */
	{0x401C, 0x6B}, /* SHD_KNOT_1_R7             */
	{0x401D, 0x67}, /* SHD_KNOT_1_GR7            */
	{0x401E, 0x67}, /* SHD_KNOT_1_GB7            */
	{0x401F, 0x66}, /* SHD_KNOT_1_B7             */
	{0x4020, 0x61}, /* SHD_KNOT_1_R8             */
	{0x4021, 0x5B}, /* SHD_KNOT_1_GR8            */
	{0x4022, 0x5B}, /* SHD_KNOT_1_GB8            */
	{0x4023, 0x59}, /* SHD_KNOT_1_B8             */
	{0x4024, 0x69}, /* SHD_KNOT_1_R9             */
	{0x4025, 0x63}, /* SHD_KNOT_1_GR9            */
	{0x4026, 0x63}, /* SHD_KNOT_1_GB9            */
	{0x4027, 0x62}, /* SHD_KNOT_1_B9             */
	{0x4028, 0x71}, /* SHD_KNOT_1_R10            */
	{0x4029, 0x6E}, /* SHD_KNOT_1_GR10           */
	{0x402A, 0x6E}, /* SHD_KNOT_1_GB10           */
	{0x402B, 0x6D}, /* SHD_KNOT_1_B10            */
	{0x402C, 0x78}, /* SHD_KNOT_1_R11            */
	{0x402D, 0x76}, /* SHD_KNOT_1_GR11           */
	{0x402E, 0x76}, /* SHD_KNOT_1_GB11           */
	{0x402F, 0x76}, /* SHD_KNOT_1_B11            */
	{0x4030, 0x7C}, /* SHD_KNOT_1_R12            */
	{0x4031, 0x7B}, /* SHD_KNOT_1_GR12           */
	{0x4032, 0x7B}, /* SHD_KNOT_1_GB12           */
	{0x4033, 0x7B}, /* SHD_KNOT_1_B12            */
	{0x4034, 0x7D}, /* SHD_KNOT_1_R13            */
	{0x4035, 0x7D}, /* SHD_KNOT_1_GR13           */
	{0x4036, 0x7D}, /* SHD_KNOT_1_GB13           */
	{0x4037, 0x7C}, /* SHD_KNOT_1_B13            */
	{0x4038, 0x7B}, /* SHD_KNOT_1_R14            */
	{0x4039, 0x7B}, /* SHD_KNOT_1_GR14           */
	{0x403A, 0x7B}, /* SHD_KNOT_1_GB14           */
	{0x403B, 0x7A}, /* SHD_KNOT_1_B14            */
	{0x403C, 0x77}, /* SHD_KNOT_1_R15            */
	{0x403D, 0x75}, /* SHD_KNOT_1_GR15           */
	{0x403E, 0x76}, /* SHD_KNOT_1_GB15           */
	{0x403F, 0x75}, /* SHD_KNOT_1_B15            */
	{0x4040, 0x70}, /* SHD_KNOT_1_R16            */
	{0x4041, 0x6D}, /* SHD_KNOT_1_GR16           */
	{0x4042, 0x6D}, /* SHD_KNOT_1_GB16           */
	{0x4043, 0x6C}, /* SHD_KNOT_1_B16            */
	{0x4044, 0x67}, /* SHD_KNOT_1_R17            */
	{0x4045, 0x61}, /* SHD_KNOT_1_GR17           */
	{0x4046, 0x61}, /* SHD_KNOT_1_GB17           */
	{0x4047, 0x60}, /* SHD_KNOT_1_B17            */
	{0x4048, 0x6B}, /* SHD_KNOT_1_R18            */
	{0x4049, 0x66}, /* SHD_KNOT_1_GR18           */
	{0x404A, 0x66}, /* SHD_KNOT_1_GB18           */
	{0x404B, 0x65}, /* SHD_KNOT_1_B18            */
	{0x404C, 0x74}, /* SHD_KNOT_1_R19            */
	{0x404D, 0x71}, /* SHD_KNOT_1_GR19           */
	{0x404E, 0x71}, /* SHD_KNOT_1_GB19           */
	{0x404F, 0x70}, /* SHD_KNOT_1_B19            */
	{0x4050, 0x7B}, /* SHD_KNOT_1_R20            */
	{0x4051, 0x7A}, /* SHD_KNOT_1_GR20           */
	{0x4052, 0x7A}, /* SHD_KNOT_1_GB20           */
	{0x4053, 0x79}, /* SHD_KNOT_1_B20            */
	{0x4054, 0x7F}, /* SHD_KNOT_1_R21            */
	{0x4055, 0x7E}, /* SHD_KNOT_1_GR21           */
	{0x4056, 0x7E}, /* SHD_KNOT_1_GB21           */
	{0x4057, 0x7E}, /* SHD_KNOT_1_B21            */
	{0x4058, 0x80}, /* SHD_KNOT_1_R22            */
	{0x4059, 0x80}, /* SHD_KNOT_1_GR22           */
	{0x405A, 0x80}, /* SHD_KNOT_1_GB22           */
	{0x405B, 0x80}, /* SHD_KNOT_1_B22            */
	{0x405C, 0x7E}, /* SHD_KNOT_1_R23            */
	{0x405D, 0x7E}, /* SHD_KNOT_1_GR23           */
	{0x405E, 0x7E}, /* SHD_KNOT_1_GB23           */
	{0x405F, 0x7D}, /* SHD_KNOT_1_B23            */
	{0x4060, 0x7A}, /* SHD_KNOT_1_R24            */
	{0x4061, 0x79}, /* SHD_KNOT_1_GR24           */
	{0x4062, 0x79}, /* SHD_KNOT_1_GB24           */
	{0x4063, 0x78}, /* SHD_KNOT_1_B24            */
	{0x4064, 0x73}, /* SHD_KNOT_1_R25            */
	{0x4065, 0x70}, /* SHD_KNOT_1_GR25           */
	{0x4066, 0x70}, /* SHD_KNOT_1_GB25           */
	{0x4067, 0x6F}, /* SHD_KNOT_1_B25            */
	{0x4068, 0x69}, /* SHD_KNOT_1_R26            */
	{0x4069, 0x64}, /* SHD_KNOT_1_GR26           */
	{0x406A, 0x64}, /* SHD_KNOT_1_GB26           */
	{0x406B, 0x63}, /* SHD_KNOT_1_B26            */
	{0x406C, 0x6B}, /* SHD_KNOT_1_R27            */
	{0x406D, 0x66}, /* SHD_KNOT_1_GR27           */
	{0x406E, 0x66}, /* SHD_KNOT_1_GB27           */
	{0x406F, 0x65}, /* SHD_KNOT_1_B27            */
	{0x4070, 0x74}, /* SHD_KNOT_1_R28            */
	{0x4071, 0x71}, /* SHD_KNOT_1_GR28           */
	{0x4072, 0x71}, /* SHD_KNOT_1_GB28           */
	{0x4073, 0x70}, /* SHD_KNOT_1_B28            */
	{0x4074, 0x7B}, /* SHD_KNOT_1_R29            */
	{0x4075, 0x79}, /* SHD_KNOT_1_GR29           */
	{0x4076, 0x79}, /* SHD_KNOT_1_GB29           */
	{0x4077, 0x79}, /* SHD_KNOT_1_B29            */
	{0x4078, 0x7E}, /* SHD_KNOT_1_R30            */
	{0x4079, 0x7D}, /* SHD_KNOT_1_GR30           */
	{0x407A, 0x7D}, /* SHD_KNOT_1_GB30           */
	{0x407B, 0x7D}, /* SHD_KNOT_1_B30            */
	{0x407C, 0x7F}, /* SHD_KNOT_1_R31            */
	{0x407D, 0x7F}, /* SHD_KNOT_1_GR31           */
	{0x407E, 0x7F}, /* SHD_KNOT_1_GB31           */
	{0x407F, 0x7F}, /* SHD_KNOT_1_B31            */
	{0x4080, 0x7E}, /* SHD_KNOT_1_R32            */
	{0x4081, 0x7D}, /* SHD_KNOT_1_GR32           */
	{0x4082, 0x7D}, /* SHD_KNOT_1_GB32           */
	{0x4083, 0x7D}, /* SHD_KNOT_1_B32            */
	{0x4084, 0x7A}, /* SHD_KNOT_1_R33            */
	{0x4085, 0x78}, /* SHD_KNOT_1_GR33           */
	{0x4086, 0x78}, /* SHD_KNOT_1_GB33           */
	{0x4087, 0x78}, /* SHD_KNOT_1_B33            */
	{0x4088, 0x73}, /* SHD_KNOT_1_R34            */
	{0x4089, 0x70}, /* SHD_KNOT_1_GR34           */
	{0x408A, 0x70}, /* SHD_KNOT_1_GB34           */
	{0x408B, 0x6F}, /* SHD_KNOT_1_B34            */
	{0x408C, 0x69}, /* SHD_KNOT_1_R35            */
	{0x408D, 0x64}, /* SHD_KNOT_1_GR35           */
	{0x408E, 0x64}, /* SHD_KNOT_1_GB35           */
	{0x408F, 0x63}, /* SHD_KNOT_1_B35            */
	{0x4090, 0x68}, /* SHD_KNOT_1_R36            */
	{0x4091, 0x62}, /* SHD_KNOT_1_GR36           */
	{0x4092, 0x62}, /* SHD_KNOT_1_GB36           */
	{0x4093, 0x61}, /* SHD_KNOT_1_B36            */
	{0x4094, 0x71}, /* SHD_KNOT_1_R37            */
	{0x4095, 0x6D}, /* SHD_KNOT_1_GR37           */
	{0x4096, 0x6D}, /* SHD_KNOT_1_GB37           */
	{0x4097, 0x6D}, /* SHD_KNOT_1_B37            */
	{0x4098, 0x77}, /* SHD_KNOT_1_R38            */
	{0x4099, 0x75}, /* SHD_KNOT_1_GR38           */
	{0x409A, 0x75}, /* SHD_KNOT_1_GB38           */
	{0x409B, 0x75}, /* SHD_KNOT_1_B38            */
	{0x409C, 0x7B}, /* SHD_KNOT_1_R39            */
	{0x409D, 0x7A}, /* SHD_KNOT_1_GR39           */
	{0x409E, 0x7A}, /* SHD_KNOT_1_GB39           */
	{0x409F, 0x7A}, /* SHD_KNOT_1_B39            */
	{0x40A0, 0x7C}, /* SHD_KNOT_1_R40            */
	{0x40A1, 0x7B}, /* SHD_KNOT_1_GR40           */
	{0x40A2, 0x7B}, /* SHD_KNOT_1_GB40           */
	{0x40A3, 0x7B}, /* SHD_KNOT_1_B40            */
	{0x40A4, 0x7B}, /* SHD_KNOT_1_R41            */
	{0x40A5, 0x79}, /* SHD_KNOT_1_GR41           */
	{0x40A6, 0x79}, /* SHD_KNOT_1_GB41           */
	{0x40A7, 0x79}, /* SHD_KNOT_1_B41            */
	{0x40A8, 0x77}, /* SHD_KNOT_1_R42            */
	{0x40A9, 0x74}, /* SHD_KNOT_1_GR42           */
	{0x40AA, 0x74}, /* SHD_KNOT_1_GB42           */
	{0x40AB, 0x74}, /* SHD_KNOT_1_B42            */
	{0x40AC, 0x70}, /* SHD_KNOT_1_R43            */
	{0x40AD, 0x6C}, /* SHD_KNOT_1_GR43           */
	{0x40AE, 0x6C}, /* SHD_KNOT_1_GB43           */
	{0x40AF, 0x6C}, /* SHD_KNOT_1_B43            */
	{0x40B0, 0x66}, /* SHD_KNOT_1_R44            */
	{0x40B1, 0x60}, /* SHD_KNOT_1_GR44           */
	{0x40B2, 0x60}, /* SHD_KNOT_1_GB44           */
	{0x40B3, 0x5F}, /* SHD_KNOT_1_B44            */
	{0x40B4, 0x62}, /* SHD_KNOT_1_R45            */
	{0x40B5, 0x5B}, /* SHD_KNOT_1_GR45           */
	{0x40B6, 0x5B}, /* SHD_KNOT_1_GB45           */
	{0x40B7, 0x5A}, /* SHD_KNOT_1_B45            */
	{0x40B8, 0x6B}, /* SHD_KNOT_1_R46            */
	{0x40B9, 0x66}, /* SHD_KNOT_1_GR46           */
	{0x40BA, 0x66}, /* SHD_KNOT_1_GB46           */
	{0x40BB, 0x65}, /* SHD_KNOT_1_B46            */
	{0x40BC, 0x71}, /* SHD_KNOT_1_R47            */
	{0x40BD, 0x6E}, /* SHD_KNOT_1_GR47           */
	{0x40BE, 0x6E}, /* SHD_KNOT_1_GB47           */
	{0x40BF, 0x6D}, /* SHD_KNOT_1_B47            */
	{0x40C0, 0x75}, /* SHD_KNOT_1_R48            */
	{0x40C1, 0x72}, /* SHD_KNOT_1_GR48           */
	{0x40C2, 0x72}, /* SHD_KNOT_1_GB48           */
	{0x40C3, 0x72}, /* SHD_KNOT_1_B48            */
	{0x40C4, 0x76}, /* SHD_KNOT_1_R49            */
	{0x40C5, 0x74}, /* SHD_KNOT_1_GR49           */
	{0x40C6, 0x74}, /* SHD_KNOT_1_GB49           */
	{0x40C7, 0x74}, /* SHD_KNOT_1_B49            */
	{0x40C8, 0x75}, /* SHD_KNOT_1_R50            */
	{0x40C9, 0x72}, /* SHD_KNOT_1_GR50           */
	{0x40CA, 0x72}, /* SHD_KNOT_1_GB50           */
	{0x40CB, 0x72}, /* SHD_KNOT_1_B50            */
	{0x40CC, 0x71}, /* SHD_KNOT_1_R51            */
	{0x40CD, 0x6D}, /* SHD_KNOT_1_GR51           */
	{0x40CE, 0x6D}, /* SHD_KNOT_1_GB51           */
	{0x40CF, 0x6D}, /* SHD_KNOT_1_B51            */
	{0x40D0, 0x6A}, /* SHD_KNOT_1_R52            */
	{0x40D1, 0x64}, /* SHD_KNOT_1_GR52           */
	{0x40D2, 0x64}, /* SHD_KNOT_1_GB52           */
	{0x40D3, 0x64}, /* SHD_KNOT_1_B52            */
	{0x40D4, 0x61}, /* SHD_KNOT_1_R53            */
	{0x40D5, 0x59}, /* SHD_KNOT_1_GR53           */
	{0x40D6, 0x59}, /* SHD_KNOT_1_GB53           */
	{0x40D7, 0x58}, /* SHD_KNOT_1_B53            */
	{0x4300, 0x8E}, /* SHD_KNOT_DIFF_1_R0        */
	{0x4301, 0x8B}, /* SHD_KNOT_DIFF_1_GR0       */
	{0x4302, 0x8B}, /* SHD_KNOT_DIFF_1_GB0       */
	{0x4303, 0x8C}, /* SHD_KNOT_DIFF_1_B0        */
	{0x4304, 0x88}, /* SHD_KNOT_DIFF_1_R1        */
	{0x4305, 0x88}, /* SHD_KNOT_DIFF_1_GR1       */
	{0x4306, 0x88}, /* SHD_KNOT_DIFF_1_GB1       */
	{0x4307, 0x88}, /* SHD_KNOT_DIFF_1_B1        */
	{0x4308, 0x88}, /* SHD_KNOT_DIFF_1_R2        */
	{0x4309, 0x84}, /* SHD_KNOT_DIFF_1_GR2       */
	{0x430A, 0x84}, /* SHD_KNOT_DIFF_1_GB2       */
	{0x430B, 0x85}, /* SHD_KNOT_DIFF_1_B2        */
	{0x430C, 0x83}, /* SHD_KNOT_DIFF_1_R3        */
	{0x430D, 0x82}, /* SHD_KNOT_DIFF_1_GR3       */
	{0x430E, 0x82}, /* SHD_KNOT_DIFF_1_GB3       */
	{0x430F, 0x82}, /* SHD_KNOT_DIFF_1_B3        */
	{0x4310, 0x84}, /* SHD_KNOT_DIFF_1_R4        */
	{0x4311, 0x83}, /* SHD_KNOT_DIFF_1_GR4       */
	{0x4312, 0x83}, /* SHD_KNOT_DIFF_1_GB4       */
	{0x4313, 0x83}, /* SHD_KNOT_DIFF_1_B4        */
	{0x4314, 0x84}, /* SHD_KNOT_DIFF_1_R5        */
	{0x4315, 0x85}, /* SHD_KNOT_DIFF_1_GR5       */
	{0x4316, 0x85}, /* SHD_KNOT_DIFF_1_GB5       */
	{0x4317, 0x84}, /* SHD_KNOT_DIFF_1_B5        */
	{0x4318, 0x86}, /* SHD_KNOT_DIFF_1_R6        */
	{0x4319, 0x87}, /* SHD_KNOT_DIFF_1_GR6       */
	{0x431A, 0x87}, /* SHD_KNOT_DIFF_1_GB6       */
	{0x431B, 0x87}, /* SHD_KNOT_DIFF_1_B6        */
	{0x431C, 0x8D}, /* SHD_KNOT_DIFF_1_R7        */
	{0x431D, 0x8B}, /* SHD_KNOT_DIFF_1_GR7       */
	{0x431E, 0x8B}, /* SHD_KNOT_DIFF_1_GB7       */
	{0x431F, 0x8C}, /* SHD_KNOT_DIFF_1_B7        */
	{0x4320, 0x93}, /* SHD_KNOT_DIFF_1_R8        */
	{0x4321, 0x91}, /* SHD_KNOT_DIFF_1_GR8       */
	{0x4322, 0x90}, /* SHD_KNOT_DIFF_1_GB8       */
	{0x4323, 0x91}, /* SHD_KNOT_DIFF_1_B8        */
	{0x4324, 0x8A}, /* SHD_KNOT_DIFF_1_R9        */
	{0x4325, 0x89}, /* SHD_KNOT_DIFF_1_GR9       */
	{0x4326, 0x89}, /* SHD_KNOT_DIFF_1_GB9       */
	{0x4327, 0x8A}, /* SHD_KNOT_DIFF_1_B9        */
	{0x4328, 0x87}, /* SHD_KNOT_DIFF_1_R10       */
	{0x4329, 0x85}, /* SHD_KNOT_DIFF_1_GR10      */
	{0x432A, 0x85}, /* SHD_KNOT_DIFF_1_GB10      */
	{0x432B, 0x86}, /* SHD_KNOT_DIFF_1_B10       */
	{0x432C, 0x84}, /* SHD_KNOT_DIFF_1_R11       */
	{0x432D, 0x82}, /* SHD_KNOT_DIFF_1_GR11      */
	{0x432E, 0x82}, /* SHD_KNOT_DIFF_1_GB11      */
	{0x432F, 0x83}, /* SHD_KNOT_DIFF_1_B11       */
	{0x4330, 0x82}, /* SHD_KNOT_DIFF_1_R12       */
	{0x4331, 0x81}, /* SHD_KNOT_DIFF_1_GR12      */
	{0x4332, 0x81}, /* SHD_KNOT_DIFF_1_GB12      */
	{0x4333, 0x81}, /* SHD_KNOT_DIFF_1_B12       */
	{0x4334, 0x81}, /* SHD_KNOT_DIFF_1_R13       */
	{0x4335, 0x81}, /* SHD_KNOT_DIFF_1_GR13      */
	{0x4336, 0x81}, /* SHD_KNOT_DIFF_1_GB13      */
	{0x4337, 0x81}, /* SHD_KNOT_DIFF_1_B13       */
	{0x4338, 0x82}, /* SHD_KNOT_DIFF_1_R14       */
	{0x4339, 0x82}, /* SHD_KNOT_DIFF_1_GR14      */
	{0x433A, 0x82}, /* SHD_KNOT_DIFF_1_GB14      */
	{0x433B, 0x82}, /* SHD_KNOT_DIFF_1_B14       */
	{0x433C, 0x85}, /* SHD_KNOT_DIFF_1_R15       */
	{0x433D, 0x85}, /* SHD_KNOT_DIFF_1_GR15      */
	{0x433E, 0x85}, /* SHD_KNOT_DIFF_1_GB15      */
	{0x433F, 0x85}, /* SHD_KNOT_DIFF_1_B15       */
	{0x4340, 0x8A}, /* SHD_KNOT_DIFF_1_R16       */
	{0x4341, 0x89}, /* SHD_KNOT_DIFF_1_GR16      */
	{0x4342, 0x89}, /* SHD_KNOT_DIFF_1_GB16      */
	{0x4343, 0x89}, /* SHD_KNOT_DIFF_1_B16       */
	{0x4344, 0x91}, /* SHD_KNOT_DIFF_1_R17       */
	{0x4345, 0x8E}, /* SHD_KNOT_DIFF_1_GR17      */
	{0x4346, 0x8E}, /* SHD_KNOT_DIFF_1_GB17      */
	{0x4347, 0x8F}, /* SHD_KNOT_DIFF_1_B17       */
	{0x4348, 0x8A}, /* SHD_KNOT_DIFF_1_R18       */
	{0x4349, 0x87}, /* SHD_KNOT_DIFF_1_GR18      */
	{0x434A, 0x87}, /* SHD_KNOT_DIFF_1_GB18      */
	{0x434B, 0x89}, /* SHD_KNOT_DIFF_1_B18       */
	{0x434C, 0x85}, /* SHD_KNOT_DIFF_1_R19       */
	{0x434D, 0x84}, /* SHD_KNOT_DIFF_1_GR19      */
	{0x434E, 0x84}, /* SHD_KNOT_DIFF_1_GB19      */
	{0x434F, 0x85}, /* SHD_KNOT_DIFF_1_B19       */
	{0x4350, 0x82}, /* SHD_KNOT_DIFF_1_R20       */
	{0x4351, 0x82}, /* SHD_KNOT_DIFF_1_GR20      */
	{0x4352, 0x82}, /* SHD_KNOT_DIFF_1_GB20      */
	{0x4353, 0x82}, /* SHD_KNOT_DIFF_1_B20       */
	{0x4354, 0x81}, /* SHD_KNOT_DIFF_1_R21       */
	{0x4355, 0x81}, /* SHD_KNOT_DIFF_1_GR21      */
	{0x4356, 0x81}, /* SHD_KNOT_DIFF_1_GB21      */
	{0x4357, 0x81}, /* SHD_KNOT_DIFF_1_B21       */
	{0x4358, 0x80}, /* SHD_KNOT_DIFF_1_R22       */
	{0x4359, 0x80}, /* SHD_KNOT_DIFF_1_GR22      */
	{0x435A, 0x80}, /* SHD_KNOT_DIFF_1_GB22      */
	{0x435B, 0x80}, /* SHD_KNOT_DIFF_1_B22       */
	{0x435C, 0x81}, /* SHD_KNOT_DIFF_1_R23       */
	{0x435D, 0x81}, /* SHD_KNOT_DIFF_1_GR23      */
	{0x435E, 0x81}, /* SHD_KNOT_DIFF_1_GB23      */
	{0x435F, 0x81}, /* SHD_KNOT_DIFF_1_B23       */
	{0x4360, 0x84}, /* SHD_KNOT_DIFF_1_R24       */
	{0x4361, 0x84}, /* SHD_KNOT_DIFF_1_GR24      */
	{0x4362, 0x84}, /* SHD_KNOT_DIFF_1_GB24      */
	{0x4363, 0x83}, /* SHD_KNOT_DIFF_1_B24       */
	{0x4364, 0x88}, /* SHD_KNOT_DIFF_1_R25       */
	{0x4365, 0x88}, /* SHD_KNOT_DIFF_1_GR25      */
	{0x4366, 0x88}, /* SHD_KNOT_DIFF_1_GB25      */
	{0x4367, 0x87}, /* SHD_KNOT_DIFF_1_B25       */
	{0x4368, 0x8E}, /* SHD_KNOT_DIFF_1_R26       */
	{0x4369, 0x8D}, /* SHD_KNOT_DIFF_1_GR26      */
	{0x436A, 0x8D}, /* SHD_KNOT_DIFF_1_GB26      */
	{0x436B, 0x8C}, /* SHD_KNOT_DIFF_1_B26       */
	{0x436C, 0x87}, /* SHD_KNOT_DIFF_1_R27       */
	{0x436D, 0x86}, /* SHD_KNOT_DIFF_1_GR27      */
	{0x436E, 0x86}, /* SHD_KNOT_DIFF_1_GB27      */
	{0x436F, 0x88}, /* SHD_KNOT_DIFF_1_B27       */
	{0x4370, 0x85}, /* SHD_KNOT_DIFF_1_R28       */
	{0x4371, 0x84}, /* SHD_KNOT_DIFF_1_GR28      */
	{0x4372, 0x84}, /* SHD_KNOT_DIFF_1_GB28      */
	{0x4373, 0x85}, /* SHD_KNOT_DIFF_1_B28       */
	{0x4374, 0x83}, /* SHD_KNOT_DIFF_1_R29       */
	{0x4375, 0x82}, /* SHD_KNOT_DIFF_1_GR29      */
	{0x4376, 0x82}, /* SHD_KNOT_DIFF_1_GB29      */
	{0x4377, 0x82}, /* SHD_KNOT_DIFF_1_B29       */
	{0x4378, 0x81}, /* SHD_KNOT_DIFF_1_R30       */
	{0x4379, 0x80}, /* SHD_KNOT_DIFF_1_GR30      */
	{0x437A, 0x80}, /* SHD_KNOT_DIFF_1_GB30      */
	{0x437B, 0x80}, /* SHD_KNOT_DIFF_1_B30       */
	{0x437C, 0x80}, /* SHD_KNOT_DIFF_1_R31       */
	{0x437D, 0x80}, /* SHD_KNOT_DIFF_1_GR31      */
	{0x437E, 0x80}, /* SHD_KNOT_DIFF_1_GB31      */
	{0x437F, 0x80}, /* SHD_KNOT_DIFF_1_B31       */
	{0x4380, 0x81}, /* SHD_KNOT_DIFF_1_R32       */
	{0x4381, 0x81}, /* SHD_KNOT_DIFF_1_GR32      */
	{0x4382, 0x81}, /* SHD_KNOT_DIFF_1_GB32      */
	{0x4383, 0x80}, /* SHD_KNOT_DIFF_1_B32       */
	{0x4384, 0x84}, /* SHD_KNOT_DIFF_1_R33       */
	{0x4385, 0x84}, /* SHD_KNOT_DIFF_1_GR33      */
	{0x4386, 0x84}, /* SHD_KNOT_DIFF_1_GB33      */
	{0x4387, 0x83}, /* SHD_KNOT_DIFF_1_B33       */
	{0x4388, 0x88}, /* SHD_KNOT_DIFF_1_R34       */
	{0x4389, 0x88}, /* SHD_KNOT_DIFF_1_GR34      */
	{0x438A, 0x88}, /* SHD_KNOT_DIFF_1_GB34      */
	{0x438B, 0x87}, /* SHD_KNOT_DIFF_1_B34       */
	{0x438C, 0x8E}, /* SHD_KNOT_DIFF_1_R35       */
	{0x438D, 0x8E}, /* SHD_KNOT_DIFF_1_GR35      */
	{0x438E, 0x8E}, /* SHD_KNOT_DIFF_1_GB35      */
	{0x438F, 0x8C}, /* SHD_KNOT_DIFF_1_B35       */
	{0x4390, 0x89}, /* SHD_KNOT_DIFF_1_R36       */
	{0x4391, 0x88}, /* SHD_KNOT_DIFF_1_GR36      */
	{0x4392, 0x88}, /* SHD_KNOT_DIFF_1_GB36      */
	{0x4393, 0x8A}, /* SHD_KNOT_DIFF_1_B36       */
	{0x4394, 0x86}, /* SHD_KNOT_DIFF_1_R37       */
	{0x4395, 0x85}, /* SHD_KNOT_DIFF_1_GR37      */
	{0x4396, 0x85}, /* SHD_KNOT_DIFF_1_GB37      */
	{0x4397, 0x86}, /* SHD_KNOT_DIFF_1_B37       */
	{0x4398, 0x83}, /* SHD_KNOT_DIFF_1_R38       */
	{0x4399, 0x83}, /* SHD_KNOT_DIFF_1_GR38      */
	{0x439A, 0x83}, /* SHD_KNOT_DIFF_1_GB38      */
	{0x439B, 0x83}, /* SHD_KNOT_DIFF_1_B38       */
	{0x439C, 0x81}, /* SHD_KNOT_DIFF_1_R39       */
	{0x439D, 0x81}, /* SHD_KNOT_DIFF_1_GR39      */
	{0x439E, 0x81}, /* SHD_KNOT_DIFF_1_GB39      */
	{0x439F, 0x81}, /* SHD_KNOT_DIFF_1_B39       */
	{0x43A0, 0x81}, /* SHD_KNOT_DIFF_1_R40       */
	{0x43A1, 0x81}, /* SHD_KNOT_DIFF_1_GR40      */
	{0x43A2, 0x81}, /* SHD_KNOT_DIFF_1_GB40      */
	{0x43A3, 0x81}, /* SHD_KNOT_DIFF_1_B40       */
	{0x43A4, 0x82}, /* SHD_KNOT_DIFF_1_R41       */
	{0x43A5, 0x82}, /* SHD_KNOT_DIFF_1_GR41      */
	{0x43A6, 0x82}, /* SHD_KNOT_DIFF_1_GB41      */
	{0x43A7, 0x81}, /* SHD_KNOT_DIFF_1_B41       */
	{0x43A8, 0x84}, /* SHD_KNOT_DIFF_1_R42       */
	{0x43A9, 0x85}, /* SHD_KNOT_DIFF_1_GR42      */
	{0x43AA, 0x85}, /* SHD_KNOT_DIFF_1_GB42      */
	{0x43AB, 0x83}, /* SHD_KNOT_DIFF_1_B42       */
	{0x43AC, 0x8A}, /* SHD_KNOT_DIFF_1_R43       */
	{0x43AD, 0x8A}, /* SHD_KNOT_DIFF_1_GR43      */
	{0x43AE, 0x8A}, /* SHD_KNOT_DIFF_1_GB43      */
	{0x43AF, 0x88}, /* SHD_KNOT_DIFF_1_B43       */
	{0x43B0, 0x93}, /* SHD_KNOT_DIFF_1_R44       */
	{0x43B1, 0x91}, /* SHD_KNOT_DIFF_1_GR44      */
	{0x43B2, 0x91}, /* SHD_KNOT_DIFF_1_GB44      */
	{0x43B3, 0x8F}, /* SHD_KNOT_DIFF_1_B44       */
	{0x43B4, 0x8B}, /* SHD_KNOT_DIFF_1_R45       */
	{0x43B5, 0x8A}, /* SHD_KNOT_DIFF_1_GR45      */
	{0x43B6, 0x8A}, /* SHD_KNOT_DIFF_1_GB45      */
	{0x43B7, 0x8C}, /* SHD_KNOT_DIFF_1_B45       */
	{0x43B8, 0x89}, /* SHD_KNOT_DIFF_1_R46       */
	{0x43B9, 0x87}, /* SHD_KNOT_DIFF_1_GR46      */
	{0x43BA, 0x87}, /* SHD_KNOT_DIFF_1_GB46      */
	{0x43BB, 0x89}, /* SHD_KNOT_DIFF_1_B46       */
	{0x43BC, 0x84}, /* SHD_KNOT_DIFF_1_R47       */
	{0x43BD, 0x84}, /* SHD_KNOT_DIFF_1_GR47      */
	{0x43BE, 0x84}, /* SHD_KNOT_DIFF_1_GB47      */
	{0x43BF, 0x84}, /* SHD_KNOT_DIFF_1_B47       */
	{0x43C0, 0x83}, /* SHD_KNOT_DIFF_1_R48       */
	{0x43C1, 0x83}, /* SHD_KNOT_DIFF_1_GR48      */
	{0x43C2, 0x83}, /* SHD_KNOT_DIFF_1_GB48      */
	{0x43C3, 0x83}, /* SHD_KNOT_DIFF_1_B48       */
	{0x43C4, 0x82}, /* SHD_KNOT_DIFF_1_R49       */
	{0x43C5, 0x83}, /* SHD_KNOT_DIFF_1_GR49      */
	{0x43C6, 0x83}, /* SHD_KNOT_DIFF_1_GB49      */
	{0x43C7, 0x82}, /* SHD_KNOT_DIFF_1_B49       */
	{0x43C8, 0x83}, /* SHD_KNOT_DIFF_1_R50       */
	{0x43C9, 0x84}, /* SHD_KNOT_DIFF_1_GR50      */
	{0x43CA, 0x84}, /* SHD_KNOT_DIFF_1_GB50      */
	{0x43CB, 0x83}, /* SHD_KNOT_DIFF_1_B50       */
	{0x43CC, 0x86}, /* SHD_KNOT_DIFF_1_R51       */
	{0x43CD, 0x86}, /* SHD_KNOT_DIFF_1_GR51      */
	{0x43CE, 0x86}, /* SHD_KNOT_DIFF_1_GB51      */
	{0x43CF, 0x85}, /* SHD_KNOT_DIFF_1_B51       */
	{0x43D0, 0x8E}, /* SHD_KNOT_DIFF_1_R52       */
	{0x43D1, 0x8D}, /* SHD_KNOT_DIFF_1_GR52      */
	{0x43D2, 0x8D}, /* SHD_KNOT_DIFF_1_GB52      */
	{0x43D3, 0x8B}, /* SHD_KNOT_DIFF_1_B52       */
	{0x43D4, 0x94}, /* SHD_KNOT_DIFF_1_R53       */
	{0x43D5, 0x93}, /* SHD_KNOT_DIFF_1_GR53      */
	{0x43D6, 0x93}, /* SHD_KNOT_DIFF_1_GB53      */
	{0x43D7, 0x91}, /* SHD_KNOT_DIFF_1_B53       */
	{0x34C0, 0xDD}, /* AE_SENSRATIO_SP1H_SP1L_R  */
	{0x34C1, 0x00}, /* AE_SENSRATIO_SP1H_SP1L_R  */
	{0x34C2, 0xF1}, /* AE_SENSRATIO_SP1H_SP1L_GR */
	{0x34C3, 0x00}, /* AE_SENSRATIO_SP1H_SP1L_GR */
	{0x34C4, 0xEC}, /* AE_SENSRATIO_SP1H_SP1L_GB */
	{0x34C5, 0x00}, /* AE_SENSRATIO_SP1H_SP1L_GB */
	{0x34C6, 0xE5}, /* AE_SENSRATIO_SP1H_SP1L_B  */
	{0x34C7, 0x00}, /* AE_SENSRATIO_SP1H_SP1L_B  */
	{0x34C8, 0xE7}, /* AE_SENSRATIO_SP1H_SP2_R   */
	{0x34C9, 0x1B}, /* AE_SENSRATIO_SP1H_SP2_R   */
	{0x34CA, 0x02}, /* AE_SENSRATIO_SP1H_SP2_GR  */
	{0x34CB, 0x1D}, /* AE_SENSRATIO_SP1H_SP2_GR  */
	{0x34CC, 0x5E}, /* AE_SENSRATIO_SP1H_SP2_GB  */
	{0x34CD, 0x1C}, /* AE_SENSRATIO_SP1H_SP2_GB  */
	{0x34CE, 0x4B}, /* AE_SENSRATIO_SP1H_SP2_B   */
	{0x34CF, 0x1B}, /* AE_SENSRATIO_SP1H_SP2_B   */
	{0x3053, 0x00}, /* "OTP_SENSRATIOEN		"    */
	{0x3630, 0x40}, /* WDC_SGAIN_ADJ_SP1_R       */
	{0x3631, 0x00}, /* WDC_SGAIN_ADJ_SP1_R       */
	{0x3632, 0x40}, /* WDC_SGAIN_ADJ_SP1_GR      */
	{0x3633, 0x00}, /* WDC_SGAIN_ADJ_SP1_GR      */
	{0x3634, 0x40}, /* WDC_SGAIN_ADJ_SP1_GB      */
	{0x3635, 0x00}, /* WDC_SGAIN_ADJ_SP1_GB      */
	{0x3636, 0x40}, /* WDC_SGAIN_ADJ_SP1_B       */
	{0x3637, 0x00}, /* WDC_SGAIN_ADJ_SP1_B       */
	{0x3638, 0x40}, /* WDC_SGAIN_ADJ_SP2_R       */
	{0x3639, 0x00}, /* WDC_SGAIN_ADJ_SP2_R       */
	{0x363A, 0x40}, /* WDC_SGAIN_ADJ_SP2_GR      */
	{0x363B, 0x00}, /* WDC_SGAIN_ADJ_SP2_GR      */
	{0x363C, 0x40}, /* WDC_SGAIN_ADJ_SP2_GB      */
	{0x363D, 0x00}, /* WDC_SGAIN_ADJ_SP2_GB      */
	{0x363E, 0x40}, /* WDC_SGAIN_ADJ_SP2_B       */
	{0x363F, 0x00}, /* WDC_SGAIN_ADJ_SP2_B       */
	{0x3838, 0xB3}, /* OBB_CLAMP_OFFSET_R_SP1H   */
	{0x3839, 0xFF}, /* OBB_CLAMP_OFFSET_R_SP1H   */
	{0x383A, 0xB5}, /* OBB_CLAMP_OFFSET_GR_SP1H  */
	{0x383B, 0xFF}, /* OBB_CLAMP_OFFSET_GR_SP1H  */
	{0x383C, 0xB4}, /* OBB_CLAMP_OFFSET_GB_SP1H  */
	{0x383D, 0xFF}, /* OBB_CLAMP_OFFSET_GB_SP1H  */
	{0x383E, 0xB7}, /* OBB_CLAMP_OFFSET_B_SP1H   */
	{0x383F, 0xFF}, /* OBB_CLAMP_OFFSET_B_SP1H   */
	{0x3840, 0xE3}, /* OBB_CLAMP_OFFSET_R_SP1L   */
	{0x3841, 0xFF}, /* OBB_CLAMP_OFFSET_R_SP1L   */
	{0x3842, 0xE6}, /* OBB_CLAMP_OFFSET_GR_SP1L  */
	{0x3843, 0xFF}, /* OBB_CLAMP_OFFSET_GR_SP1L  */
	{0x3844, 0xF9}, /* OBB_CLAMP_OFFSET_GB_SP1L  */
	{0x3845, 0xFF}, /* OBB_CLAMP_OFFSET_GB_SP1L  */
	{0x3846, 0xFE}, /* OBB_CLAMP_OFFSET_B_SP1L   */
	{0x3847, 0xFF}, /* OBB_CLAMP_OFFSET_B_SP1L   */
	{0x3848, 0xC9}, /* OBB_CLAMP_OFFSET_R_SP2    */
	{0x3849, 0xFF}, /* OBB_CLAMP_OFFSET_R_SP2    */
	{0x384A, 0xCA}, /* OBB_CLAMP_OFFSET_GR_SP2   */
	{0x384B, 0xFF}, /* OBB_CLAMP_OFFSET_GR_SP2   */
	{0x384C, 0xE5}, /* OBB_CLAMP_OFFSET_GB_SP2   */
	{0x384D, 0xFF}, /* OBB_CLAMP_OFFSET_GB_SP2   */
	{0x384E, 0xF0}, /* OBB_CLAMP_OFFSET_B_SP2    */
	{0x384F, 0xFF}, /* OBB_CLAMP_OFFSET_B_SP2    */
};

static const struct imx390_reg_list lsc_vendor_def_list = {
	.num_of_regs = ARRAY_SIZE(imx390_lsc_pattern_vendor_def),
	.regs = imx390_lsc_pattern_vendor_def,
};

static const s64 link_freq_menu_items[] = {
	IMX390_LINK_FREQ_360MHZ,
	IMX390_LINK_FREQ_300MHZ,
	IMX390_LINK_FREQ_288MHZ,
	IMX390_LINK_FREQ_240MHZ,
};

static const struct imx390_mode supported_modes[] = {
	{
		.width = 1280,
		.height = 960,
		.hdr_en = false,
		.hts = 2464,
		.vts_def = 2435,
		.vts_min = 2435,
		.code = MEDIA_BUS_FMT_SGRBG12_1X12,
		.lanes = 4,
		.fps = 30,
		.bpp = 12,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx390_mode_1280x960CROP),
			.regs = imx390_mode_1280x960CROP,
		},
		.link_freq_index = -1,
	},
	{
		.width = 1920,
		.height = 1200,
		.hdr_en = true,
		.hts = 2464,
		.vts_def = 2435,
		.vts_min = 2435,
		.code = MEDIA_BUS_FMT_SGRBG12_1X12,
		.lanes = 4,
		.fps = 30,
		.bpp = 12,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx390_mode_1920x1200HDR3_CUST_PWL12),
			.regs = imx390_mode_1920x1200HDR3_CUST_PWL12,
		},
		.link_freq_index = -1,
	},
};

static u32 supported_formats[] = {
	MEDIA_BUS_FMT_SGRBG12_1X12,
};

static int imx390_read_reg(struct imx390 *imx390, u16 reg, u16 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx390->sd);
	struct i2c_msg msgs[2];
	u8 addr_buf[2];
	u8 data_buf[4] = {0};
	int ret;

	if (len > 4)
		return -EINVAL;

	put_unaligned_be16(reg, addr_buf);
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = sizeof(addr_buf);
	msgs[0].buf = addr_buf;
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_buf[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = get_unaligned_be32(data_buf);

	return 0;
}

static int imx390_write_reg(struct imx390 *imx390, u16 reg, u16 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx390->sd);
	u8 buf[6];

	if (len > 4)
		return -EINVAL;

	dev_dbg(&client->dev, "%s, reg %x len %x, val %x\n", __func__, reg, len, val);
	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val << 8 * (4 - len), buf + 2);
	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

static int imx390_write_reg_list(struct imx390 *imx390,
				 const struct imx390_reg_list *r_list)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx390->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < r_list->num_of_regs; i++) {
		ret = imx390_write_reg(imx390, r_list->regs[i].address,
				IMX390_REG_VALUE_08BIT,
				       r_list->regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
				    "failed to write reg 0x%4.4x. error = %d",
				    r_list->regs[i].address, ret);
			return ret;
		}
	}

	return 0;
}

static int imx390_is_hdr(struct imx390 *imx390)
{
	// int mode_ix = self->s_data->sensor_mode_id;
	// return imx390_modes_formats[mode_ix].hdr_en;

	if (imx390->cur_mode->hdr_en)
		return 1;

	return 0;
}

static int imx390_group_hold_enable(struct imx390 *imx390, s32 val)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(&imx390->sd);

	dev_dbg(&client->dev, "group hold: %d", val);

	ret = imx390_write_reg(imx390, IMX390_REG_REG_HOLD, IMX390_REG_VALUE_08BIT, val ? 1:0);

	if (ret)
		dev_err(&client->dev, "failed to set group hold");

	return ret;
}

/**
 * imx390 gain is 0 to 30 in .3db steps.
 *
 * @param self driver instance
 * @param val gain
 *
 * @return 0 on success
 */
static int imx390_gain_set(struct imx390 *imx390, s64 val)
{
	u16 gain = 0;
	u32 prevgain = 0;

	gain = (u16)val; // * 10 / 3 / FIXED_POINT_SCALING_FACTOR;

	if (gain > 100)
		gain = 100;

	if (gain < 0)
		gain = 0;

	imx390_read_reg(imx390, IMX390_REG_AGAIN_SP1H, IMX390_REG_VALUE_08BIT, &prevgain);

	imx390_group_hold_enable(imx390, 1);

	imx390_write_reg(imx390, IMX390_REG_AGAIN_SP1H, IMX390_REG_VALUE_08BIT,  gain & 0xff);
	imx390_write_reg(imx390, IMX390_REG_AGAIN_SP1H + 1, IMX390_REG_VALUE_08BIT, (gain >> 8) & 0xff);

	imx390_group_hold_enable(imx390, 0);

	imx390_read_reg(imx390, IMX390_REG_AGAIN_SP1H, IMX390_REG_VALUE_08BIT, &prevgain);

	return 0;
}

static u64 get_pixel_rate(struct imx390 *imx390)
{
	u64 pixel_rate = 72000000;

	return pixel_rate;
}

/*
 * from table 1, AND9820-D.pdf.
 * for context A, hblank = LLP(0x300C) - active data time.
 */
static u64 get_hblank(struct imx390 *imx390)
{
	u64 hblank = 0;

	return hblank;
}

static int imx390_exposure_raw_set(struct imx390 *self, u32 exp)
{
	/* This should never be called in HDR mode but we'll put check
	 * in to be safe.
	 */
	if (imx390_is_hdr(self))
		return 0;

	imx390_group_hold_enable(self, 1);

	struct imx390_reg exposure_array[] = {
		/* 20 bit value 0xc, 0xd, 0xe */
		{IMX390_REG_SHS1, exp & 0xff},
		{IMX390_REG_SHS1 + 1, (exp & 0xff00) >> 8},
		{IMX390_REG_SHS1 + 2, (exp & 0xf0000) >> 16},

		/* 20 bit value  0x10, 0x11, 0x12 */
		{IMX390_REG_SHS2, exp & 0xff},
		{IMX390_REG_SHS2 + 1, (exp & 0xff00) >> 8},
		{IMX390_REG_SHS2 + 2, (exp & 0xf0000) >> 16},
	};

	const struct imx390_reg_list exp_list = {
		.num_of_regs = ARRAY_SIZE(exposure_array),
		.regs = exposure_array,
	};

	/* True means to print the register values. This is a small
	 * table so it's OK.
	 */
	imx390_write_reg_list(self, &exp_list);
	imx390_group_hold_enable(self, 0);
	return 0;
}

static int imx390_exposure_set(struct imx390 *self, s64 val)
{
	u32 coarse_time;
	u32 reg;
	u32 pixclk = 72000000;
	u32 linelen = self->cur_mode->width;

	/* This is figuring out how many lines are output for the
	 * desired exposure time.
	 */
	/* pixel clock * TIME / line_length */
	coarse_time = pixclk * val / linelen / FIXED_POINT_SCALING_FACTOR;

	/* The 390 is configured such that the SHS registers are the
	 * difference between VMAX and the exposure time expressed as
	 * lines.
	 */
	/* FRAME_LENGTH is VMAX */
	/* VMAX=1125 */
	reg =  1125 - coarse_time;
	/* The data sheet says values of 0 and 1 are prohibited...and
	 * also says that the default value is 1...
	 */
	if (reg < 2)
		reg = 2;
	else if (reg >= 0x100000)
		reg = 0x100000 - 1;

	return imx390_exposure_raw_set(self, reg);
}

static int imx390_white_balance_set(struct imx390 *self)
{
	u16 cf00, cf01, cf10, cf11;
	u16 red, blue;
	u16 max_chroma, r, g, b;
	struct i2c_client *client = v4l2_get_subdevdata(&self->sd);

	dev_dbg(&client->dev, "%s\n", __func__);
	red = *self->red_balance->p_new.p_s32;
	blue = *self->blue_balance->p_new.p_s32;

	max_chroma = MAX(MAX(red, 0x100), blue);
	r = (max_chroma * 0x100) / red;
	g = max_chroma;
	b = (max_chroma * 0x100) / blue;

	cf00 = IMX390_REG_WBGAIN_R;
	cf01 = IMX390_REG_WBGAIN_GR;
	cf10 = IMX390_REG_WBGAIN_GB;
	cf11 = IMX390_REG_WBGAIN_B;

	imx390_group_hold_enable(self, 1);
	dev_dbg(&client->dev, "self->cur_mode->code[%x] MEDIA_BUS_FMT_SGRBG12_1X12[%x]\n", self->cur_mode->code, MEDIA_BUS_FMT_SGRBG12_1X12);
	if (self->cur_mode->code == MEDIA_BUS_FMT_SGRBG12_1X12) {
		imx390_write_reg(self, IMX390_REG_WBGAIN_FORCE_X1, IMX390_REG_VALUE_08BIT, 0);

		imx390_write_reg(self, cf00, IMX390_REG_VALUE_08BIT, r & 0xff);
		imx390_write_reg(self, cf00 + 1, IMX390_REG_VALUE_08BIT, (r & 0xff00) >> 8);
		imx390_write_reg(self, cf01, IMX390_REG_VALUE_08BIT, g & 0xff);
		imx390_write_reg(self, cf01 + 1, IMX390_REG_VALUE_08BIT, (g & 0xff00) >> 8);
		imx390_write_reg(self, cf10, IMX390_REG_VALUE_08BIT, g & 0xff);
		imx390_write_reg(self, cf10 + 1, IMX390_REG_VALUE_08BIT, (g & 0xff00) >> 8);
		imx390_write_reg(self, cf11, IMX390_REG_VALUE_08BIT, b & 0xff);
		imx390_write_reg(self, cf11 + 1, IMX390_REG_VALUE_08BIT, (b & 0xff00) >> 8);
	}
	imx390_group_hold_enable(self, 0);

	return 0;
}

/*
 * imx390_set_lsc_pattern
 * len shading correction pattern
 */
static int imx390_set_lsc_pattern(struct imx390 *self, int val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&self->sd);
	int ret = 0;

	if (val == LSC_PATTERN_UNITY)
		ret = imx390_write_reg_list(self, &lsc_unity_list);

	if (val == LSC_PATTERN_TABLE)
		ret = imx390_write_reg_list(self, &lsc_vendor_def_list);

	if (!ret)
		dev_dbg(&client->dev,
			"%s : LSC PATTERN control success\n", __func__);
	else
		dev_err(&client->dev, "%s ret = %d\n", __func__, ret);

	return ret;
}

static int imx390_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx390 *imx390 = container_of(ctrl->handler,
					     struct imx390, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&imx390->sd);
	int ret = 0;

	switch (ctrl->id) {
	case IMX390_CID_EXPOSURE_SHS1:
	case IMX390_CID_EXPOSURE_SHS2:
	case IMX390_CID_EXPOSURE_SHS3:
	case IMX390_CID_EXPOSURE_RHS1:
	case IMX390_CID_EXPOSURE_RHS2:
	case IMX390_CID_DIGITAL_GAIN_L:
	case IMX390_CID_DIGITAL_GAIN_S:
	case IMX390_CID_DIGITAL_GAIN_VS:
	case IMX390_CID_ANALOG_GAIN_L:
	case IMX390_CID_ANALOG_GAIN_S:
	case IMX390_CID_ANALOG_GAIN_VS:
	case V4L2_CID_DIGITAL_GAIN:
	case V4L2_CID_GAIN:
		ret = 0;
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = imx390_gain_set(imx390, *ctrl->p_new.p_s64);
		break;
	case V4L2_CID_EXPOSURE:
		ret = imx390_exposure_set(imx390, *ctrl->p_new.p_s64);
		break;
	case V4L2_CID_VBLANK:
		ret = imx390_write_reg(imx390, IMX390_REG_VTS,
				IMX390_REG_VALUE_16BIT,
				imx390->cur_mode->height + ctrl->val);
		break;
	case V4L2_CID_RED_BALANCE:
	case V4L2_CID_BLUE_BALANCE:
		ret = imx390_white_balance_set(imx390);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = imx390_set_lsc_pattern(imx390, ctrl->val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx390_ctrl_ops = {
	.s_ctrl = imx390_set_ctrl,
};

static int imx390_init_controls(struct imx390 *imx390)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	s64 hblank;
	int ret;
	struct v4l2_ctrl_config cfg = { 0 };

	ctrl_hdlr = &imx390->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 8);
	if (ret)
		return ret;

	ctrl_hdlr->lock = &imx390->mutex;
	imx390->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr, &imx390_ctrl_ops,
					   V4L2_CID_LINK_FREQ,
					   ARRAY_SIZE(link_freq_menu_items) - 1,
					   0, link_freq_menu_items);
	if (imx390->link_freq)
		imx390->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx390->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx390_ctrl_ops,
			  V4L2_CID_VBLANK,
			  0,
			  IMX390_VTS_MAX - imx390->cur_mode->height, 1,
			  imx390->cur_mode->vts_def - imx390->cur_mode->height);

	imx390->gain = v4l2_ctrl_new_std(
			ctrl_hdlr,
			&imx390_ctrl_ops,
			V4L2_CID_GAIN, IMX390_GAIN_MIN,
			IMX390_DGTL_GAIN_MAX * IMX390_ANAL_GAIN_MAX, 1,
			IMX390_GAIN_DEFAULT);

	imx390->analogue_gain = v4l2_ctrl_new_std(ctrl_hdlr, &imx390_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX390_ANAL_GAIN_MIN, IMX390_ANAL_GAIN_MAX,
			  IMX390_ANAL_GAIN_STEP, IMX390_ANAL_GAIN_DEFAULT);

	imx390->digital_gain = v4l2_ctrl_new_std(ctrl_hdlr, &imx390_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			IMX390_DGTL_GAIN_MIN, IMX390_DGTL_GAIN_MAX,
			IMX390_DGTL_GAIN_STEP, IMX390_DGTL_GAIN_DEFAULT);

	imx390->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx390_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX390_EXPOSURE_MIN,
					     IMX390_EXPOSURE_MAX,
					     IMX390_EXPOSURE_STEP,
					     IMX390_EXPOSURE_DEF);

	imx390->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx390_ctrl_ops,
			V4L2_CID_PIXEL_RATE, get_pixel_rate(imx390), get_pixel_rate(imx390),
			1, get_pixel_rate(imx390));

	if (imx390->pixel_rate)
		imx390->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	hblank = get_hblank(imx390);
	imx390->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx390_ctrl_ops, V4L2_CID_HBLANK,
					hblank, hblank, 1, hblank);
	if (imx390->hblank)
		imx390->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx390->red_balance = v4l2_ctrl_new_std(ctrl_hdlr, &imx390_ctrl_ops,
					     V4L2_CID_RED_BALANCE,
					     IMX390_RED_BALANCE_MIN,
					     IMX390_RED_BALANCE_MAX,
					     IMX390_RED_BALANCE_STEP,
					     IMX390_RED_BALANCE_DEF);

	imx390->blue_balance = v4l2_ctrl_new_std(ctrl_hdlr, &imx390_ctrl_ops,
					     V4L2_CID_BLUE_BALANCE,
					     IMX390_BLUE_BALANCE_MIN,
					     IMX390_BLUE_BALANCE_MAX,
					     IMX390_BLUE_BALANCE_STEP,
					     IMX390_BLUE_BALANCE_DEF);

	cfg.ops = &imx390_ctrl_ops;
	cfg.id = IMX390_CID_EXPOSURE_SHS1;
	cfg.name = "IMX390_CID_EXPOSURE_SHS1";
	cfg.type = V4L2_CTRL_TYPE_INTEGER;
	cfg.max = IMX390_DUMMY_MAX;
	cfg.min = IMX390_DUMMY_MIN;
	cfg.step = IMX390_DUMMY_STEP;
	cfg.def = IMX390_DUMMY_DEF;
	cfg.qmenu = 0; cfg.elem_size = 0;
	imx390->dummy_exp_shs1 = v4l2_ctrl_new_custom(ctrl_hdlr, &cfg, NULL);

	cfg.id = IMX390_CID_EXPOSURE_SHS2;
	cfg.name = "IMX390_CID_EXPOSURE_SHS2";
	imx390->dummy_exp_shs2 = v4l2_ctrl_new_custom(ctrl_hdlr, &cfg, NULL);

	cfg.id = IMX390_CID_EXPOSURE_SHS3;
	cfg.name = "IMX390_CID_EXPOSURE_SHS3";
	imx390->dummy_exp_shs3 = v4l2_ctrl_new_custom(ctrl_hdlr, &cfg, NULL);

	cfg.id = IMX390_CID_EXPOSURE_RHS1;
	cfg.name = "IMX390_CID_EXPOSURE_RHS1";
	imx390->dummy_exp_rhs1 = v4l2_ctrl_new_custom(ctrl_hdlr, &cfg, NULL);

	cfg.id = IMX390_CID_EXPOSURE_RHS2;
	cfg.name = "IMX390_CID_EXPOSURE_RHS2";
	imx390->dummy_exp_rhs2 = v4l2_ctrl_new_custom(ctrl_hdlr, &cfg, NULL);

	cfg.id = IMX390_CID_DIGITAL_GAIN_L;
	cfg.name = "IMX390_CID_DIGITAL_GAIN_L";
	imx390->dummy_dg_l = v4l2_ctrl_new_custom(ctrl_hdlr, &cfg, NULL);

	cfg.id = IMX390_CID_DIGITAL_GAIN_S;
	cfg.name = "IMX390_CID_DIGITAL_GAIN_S";
	imx390->dummy_dg_s = v4l2_ctrl_new_custom(ctrl_hdlr, &cfg, NULL);

	cfg.id = IMX390_CID_DIGITAL_GAIN_VS;
	cfg.name = "IMX390_CID_DIGITAL_GAIN_VS";
	imx390->dummy_dg_vs = v4l2_ctrl_new_custom(ctrl_hdlr, &cfg, NULL);

	cfg.id = IMX390_CID_ANALOG_GAIN_L;
	cfg.name = "IMX390_CID_ANALOG_GAIN_L";
	imx390->dummy_ag_l = v4l2_ctrl_new_custom(ctrl_hdlr, &cfg, NULL);

	cfg.id = IMX390_CID_ANALOG_GAIN_S;
	cfg.name = "IMX390_CID_ANALOG_GAIN_S";
	imx390->dummy_ag_s = v4l2_ctrl_new_custom(ctrl_hdlr, &cfg, NULL);

	cfg.id = IMX390_CID_ANALOG_GAIN_VS;
	cfg.name = "IMX390_CID_ANALOG_GAIN_VS";
	imx390->dummy_ag_vs = v4l2_ctrl_new_custom(ctrl_hdlr, &cfg, NULL);

	imx390->lsc_pattern = v4l2_ctrl_new_std_menu_items(
		ctrl_hdlr, &imx390_ctrl_ops,
		V4L2_CID_TEST_PATTERN,
		ARRAY_SIZE(lsc_qmenu) - 1, 0, 0, lsc_qmenu);

	if (ctrl_hdlr->error)
		return ctrl_hdlr->error;

	imx390->sd.ctrl_handler = ctrl_hdlr;

	return 0;
}

static void imx390_update_pad_format(const struct imx390_mode *mode,
				     struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = mode->code;
	fmt->field = V4L2_FIELD_NONE;
}

static int imx390_start_streaming(struct imx390 *imx390)
{
	int retries, ret;
	struct i2c_client *client = v4l2_get_subdevdata(&imx390->sd);
	const struct imx390_reg_list *reg_list;

	if (imx390->cur_mode != imx390->pre_mode) {
		reg_list = &imx390->cur_mode->reg_list;
		ret = imx390_write_reg_list(imx390, reg_list);
		if (ret) {
			dev_err(&client->dev, "failed to set stream mode");
			return ret;
		}
		imx390->pre_mode = imx390->cur_mode;
	} else
		dev_dbg(&client->dev, "same mode, skip write reg list");

	/*
	 * WA: i2c write to IMX390_REG_STANDBY no response randomly,
	 * pipeline fails to start.
	 * retries 1000 times, wait for i2c recover, pipeline started
	 * with extra delay, instead of fails.
	 */
	retries = 1000;
	do {
		ret = imx390_write_reg(imx390, IMX390_REG_STANDBY,
			IMX390_REG_VALUE_08BIT, 0);
		if (ret)
			dev_err(&client->dev, "retry to write STANDBY");
	} while (ret && retries--);

	if (ret) {
		dev_err(&client->dev, "failed to set stream");
		return ret;
	}

	return 0;
}

static void imx390_stop_streaming(struct imx390 *imx390)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx390->sd);

	if (imx390_write_reg(imx390, IMX390_REG_STANDBY,
			       IMX390_REG_VALUE_08BIT, 1))
		dev_err(&client->dev, "failed to set stream");
}

static int imx390_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx390 *imx390 = to_imx390(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	if (imx390->streaming == enable)
		return 0;

	mutex_lock(&imx390->mutex);
	if (enable) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			mutex_unlock(&imx390->mutex);
			return ret;
		}

		ret = imx390_start_streaming(imx390);
		if (ret) {
			enable = 0;
			imx390_stop_streaming(imx390);
			pm_runtime_put(&client->dev);
		}
	} else {
		imx390_stop_streaming(imx390);
		pm_runtime_put(&client->dev);
	}

	imx390->streaming = enable;

	mutex_unlock(&imx390->mutex);

	return ret;
}

static int imx390_g_frame_interval(struct v4l2_subdev *sd,
		struct v4l2_subdev_frame_interval *fival)
{
	struct imx390 *imx390 = to_imx390(sd);

	fival->pad = 0;
	fival->interval.numerator = 1;
	fival->interval.denominator = imx390->cur_mode->fps;

	return 0;
}

static int __maybe_unused imx390_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx390 *imx390 = to_imx390(sd);

	mutex_lock(&imx390->mutex);
	if (imx390->streaming)
		imx390_stop_streaming(imx390);

	mutex_unlock(&imx390->mutex);

	return 0;
}

static int __maybe_unused imx390_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx390 *imx390 = to_imx390(sd);
	int ret;

	mutex_lock(&imx390->mutex);
	if (imx390->streaming) {
		ret = imx390_start_streaming(imx390);
		if (ret) {
			imx390->streaming = false;
			imx390_stop_streaming(imx390);
			mutex_unlock(&imx390->mutex);
			return ret;
		}
	}

	mutex_unlock(&imx390->mutex);

	return 0;
}

static int imx390_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *sd_state,
			     struct v4l2_subdev_format *fmt)
{
	struct imx390 *imx390 = to_imx390(sd);
	const struct imx390_mode *mode;
	int ret = 0;
	s32 vblank_def;
	s64 hblank;
	int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++)
		if (supported_modes[i].code == fmt->format.code) {
			if (supported_modes[i].width == fmt->format.width
					&& supported_modes[i].height == fmt->format.height) {
				mode = &supported_modes[i];
				break;

			}
		}

	if (i >= ARRAY_SIZE(supported_modes))
		mode = &supported_modes[0];

	mutex_lock(&imx390->mutex);

	fmt->format.code = supported_formats[0];

	imx390_update_pad_format(mode, &fmt->format);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		*v4l2_subdev_get_try_format(sd, sd_state, fmt->pad) = fmt->format;
	} else {
		imx390->cur_mode = mode;
		__v4l2_ctrl_s_ctrl(imx390->link_freq, mode->link_freq_index);
		__v4l2_ctrl_modify_range(imx390->pixel_rate,
					get_pixel_rate(imx390),
					get_pixel_rate(imx390),
					1,
					get_pixel_rate(imx390));

		hblank = get_hblank(imx390);
		__v4l2_ctrl_modify_range(imx390->hblank,
					hblank,
					hblank,
					1,
					hblank);

		/* Update limits and set FPS to default */
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(imx390->vblank,
					 0,
					 IMX390_VTS_MAX - mode->height, 1,
					 vblank_def);
		//__v4l2_ctrl_s_ctrl(imx390->vblank, vblank_def);
	}

	mutex_unlock(&imx390->mutex);

	return 0;
}

static int imx390_get_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *sd_state,
			     struct v4l2_subdev_format *fmt)
{
	struct imx390 *imx390 = to_imx390(sd);

	mutex_lock(&imx390->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt->format = *v4l2_subdev_get_try_format(&imx390->sd, sd_state,
							  fmt->pad);
	else
		imx390_update_pad_format(imx390->cur_mode, &fmt->format);

	mutex_unlock(&imx390->mutex);

	return 0;
}

static int imx390_get_frame_desc(struct v4l2_subdev *sd,
	unsigned int pad, struct v4l2_mbus_frame_desc *desc)
{
	unsigned int i;

	desc->num_entries = V4L2_FRAME_DESC_ENTRY_MAX;

	for (i = 0; i < desc->num_entries; i++) {
		desc->entry[i].flags = 0;
		desc->entry[i].pixelcode = MEDIA_BUS_FMT_FIXED;
		desc->entry[i].length = 0;
	}

	return 0;
}

static int imx390_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(supported_formats))
		return -EINVAL;

	code->code = supported_formats[code->index];

	return 0;
}

static int imx390_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int imx390_frame_rate[] = { 40, 20 };

static int imx390_enum_frame_interval(struct v4l2_subdev *subdev,
		struct v4l2_subdev_state *sd_state,
		struct v4l2_subdev_frame_interval_enum *fie)
{
	int mode_size = ARRAY_SIZE(supported_modes);
	int i;

	if (fie->index >= ARRAY_SIZE(imx390_frame_rate))
		return -EINVAL;

	for (i = 0; i < mode_size; i++)
		if (fie->code == supported_modes[i].code
			&& fie->width == supported_modes[i].width
			&& fie->height == supported_modes[i].height)
			break;

	if (i == mode_size)
		return -EINVAL;

	fie->interval.numerator = 1;
	fie->interval.denominator = imx390_frame_rate[fie->index];

	return 0;
}

static int imx390_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx390 *imx390 = to_imx390(sd);

	mutex_lock(&imx390->mutex);
	imx390_update_pad_format(&supported_modes[0],
				 v4l2_subdev_get_try_format(sd, fh->state, 0));
	mutex_unlock(&imx390->mutex);

	return 0;
}

static const struct v4l2_subdev_video_ops imx390_video_ops = {
	.s_stream = imx390_set_stream,
	.g_frame_interval = imx390_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops imx390_pad_ops = {
	.set_fmt = imx390_set_format,
	.get_fmt = imx390_get_format,
	.get_frame_desc = imx390_get_frame_desc,
	.enum_mbus_code = imx390_enum_mbus_code,
	.enum_frame_size = imx390_enum_frame_size,
	.enum_frame_interval = imx390_enum_frame_interval,
};

static const struct v4l2_subdev_ops imx390_subdev_ops = {
	.video = &imx390_video_ops,
	.pad = &imx390_pad_ops,
};

static const struct media_entity_operations imx390_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops imx390_internal_ops = {
	.open = imx390_open,
};

static int imx390_identify_module(struct imx390 *imx390)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx390->sd);
	int ret;
	u32 val;

	ret = imx390_read_reg(imx390, IMX390_REG_CHIP_ID,
			      IMX390_REG_VALUE_08BIT, &val);
	if (ret)
		return ret;

	return 0;

	/* chip id not known yet */
	if (val != IMX390_CHIP_ID) {
		dev_err(&client->dev, "chip id mismatch: %x!=%x",
			IMX390_CHIP_ID, val);
		return -ENXIO;
	}

	return 0;
}

static int imx390_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx390 *imx390 = to_imx390(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	pm_runtime_disable(&client->dev);
	mutex_destroy(&imx390->mutex);

	return 0;
}

irqreturn_t imx390_threaded_irq_fn(int irq, void *dev_id)
{
	struct imx390 *imx390 = dev_id;

	mutex_lock(&imx390->mutex);
	if (imx390->streaming == false) {
		gpio_set_value(imx390->platform_data->gpios[0], 0);
		goto imx390_irq_handled;
	}
	if (imx390->strobe_source->val == V4L2_FLASH_STROBE_SOURCE_EXTERNAL) {

		gpio_set_value(imx390->platform_data->gpios[0],
				gpio_get_value(imx390->platform_data->irq_pin));
	}

imx390_irq_handled:
	mutex_unlock(&imx390->mutex);
	return IRQ_HANDLED;
}

static int imx390_probe(struct i2c_client *client)
{
	struct v4l2_subdev *sd;
	struct imx390 *imx390;
	const struct imx390_reg_list *reg_list;
	int ret;

	imx390 = devm_kzalloc(&client->dev, sizeof(*imx390), GFP_KERNEL);
	if (!imx390)
		return -ENOMEM;

	imx390->platform_data = client->dev.platform_data;
	if (imx390->platform_data == NULL) {
		dev_err(&client->dev, "no platform data provided\n");
		return -EINVAL;
	}

	/* initialize subdevice */
	sd = &imx390->sd;
	v4l2_i2c_subdev_init(sd, client, &imx390_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	sd->internal_ops = &imx390_internal_ops;
	sd->entity.ops = &imx390_subdev_entity_ops;

	/* initialize subdev media pad */
	imx390->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &imx390->pad);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s : media entity init Failed %d\n", __func__, ret);
		return ret;
	}

	ret = imx390_identify_module(imx390);
	if (ret) {
		dev_err(&client->dev, "failed to find sensor: %d", ret);
		return ret;
	}

	if (imx390->platform_data->suffix)
		snprintf(imx390->sd.name,
				sizeof(imx390->sd.name), "imx390 %c",
				imx390->platform_data->suffix);

	mutex_init(&imx390->mutex);

	/* 1920x1200 default */
	imx390->cur_mode = &supported_modes[1];
	imx390->pre_mode = imx390->cur_mode;

	reg_list = &imx390->cur_mode->reg_list;
	ret = imx390_write_reg_list(imx390, reg_list);
	if (ret) {
		dev_err(&client->dev, "failed to apply preset mode");
		return ret;
	}

	ret = imx390_init_controls(imx390);
	if (ret) {
		dev_err(&client->dev, "failed to init controls: %d", ret);
		goto probe_error_v4l2_ctrl_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&imx390->sd);
	if (ret < 0) {
		dev_err(&client->dev, "failed to register V4L2 subdev: %d",
			ret);
		goto probe_error_media_entity_cleanup;
	}

	/*
	 * Device is already turned on by i2c-core with ACPI domain PM.
	 * Enable runtime PM and turn off the device.
	 */
	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	return 0;

probe_error_media_entity_cleanup:
	media_entity_cleanup(&imx390->sd.entity);

probe_error_v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(imx390->sd.ctrl_handler);
	mutex_destroy(&imx390->mutex);

	return ret;
}

static const struct dev_pm_ops imx390_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(imx390_suspend, imx390_resume)
};

static const struct i2c_device_id imx390_id_table[] = {
	{ "imx390", 0 },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(i2c, imx390_id_table);

static struct i2c_driver imx390_i2c_driver = {
	.driver = {
		.name = "imx390",
		.pm = &imx390_pm_ops,
	},
	.probe_new = imx390_probe,
	.remove = imx390_remove,
	.id_table = imx390_id_table,
};

module_i2c_driver(imx390_i2c_driver);

MODULE_AUTHOR("Chang, Ying <ying.chang@intel.com>");
MODULE_DESCRIPTION("imx390 sensor driver");
MODULE_LICENSE("GPL v2");
