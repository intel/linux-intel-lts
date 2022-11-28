// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 Intel Corporation.

#include <asm/unaligned.h>
#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/gpio.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/ar0234.h>
#include <linux/version.h>

#include <linux/ipu-isys.h>

#define AR0234_REG_VALUE_08BIT		1
#define AR0234_REG_VALUE_16BIT		2

#define AR0234_LINK_FREQ_360MHZ		360000000ULL
#define AR0234_LINK_FREQ_300MHZ		300000000ULL
#define AR0234_LINK_FREQ_288MHZ		288000000ULL
#define AR0234_LINK_FREQ_240MHZ		240000000ULL
#define AR0234_LINK_FREQ_22_5MHZ	22500000ULL

#define FSERIAL_CLK_4_LANE		240000000ULL
#define FSERIAL_CLK_2_LANE		180000000ULL /* use as pixel rate */

#define PIX_CLK_4_LANE			60000000ULL
#define PIX_CLK_2_LANE			45000000ULL

#define AR0234_REG_CHIP_ID		0x3000
#define AR0234_CHIP_ID			0x0A56

#define AR0234_REG_MODE_SELECT		0x301A
#define AR0234_MODE_STANDBY		0x2058
#define AR0234_MODE_STREAMING		0x205C

/* vertical-timings from sensor */
#define AR0234_REG_VTS			0x300A
#define AR0234_VTS_MAX			0xffff

/* Exposure controls from sensor */
#define AR0234_REG_EXPOSURE		0x3012
#define	AR0234_EXPOSURE_MIN		0
#define AR0234_EXPOSURE_MAX_MARGIN	80
#define	AR0234_EXPOSURE_STEP		1

/* Analog gain controls from sensor */
#define AR0234_REG_ANALOG_GAIN		0x3060
#define	AR0234_ANAL_GAIN_MIN		0
#define	AR0234_ANAL_GAIN_MAX		0x7f
#define	AR0234_ANAL_GAIN_STEP		1
#define	AR0234_ANAL_GAIN_DEFAULT	0xe

/* Digital gain controls from sensor */
#define AR0234_REG_GLOBAL_GAIN		0x305E
#define AR0234_DGTL_GAIN_MIN		0
#define AR0234_DGTL_GAIN_MAX		0x7ff
#define AR0234_DGTL_GAIN_STEP		1
#define AR0234_DGTL_GAIN_DEFAULT	0x80

#define AR0234_REG_LED_FLASH_CONTROL	0x3270
#define AR0234_LED_FLASH_EN		0x100
#define AR0234_LED_DELAY		0xff

#define AR0234_REG_IMAGE_ORIENTATION	0x301D
#define AR0234_HFLIP_BIT		0x0
#define AR0234_VFLIP_BIT		0x1

#define WIN_WIDTH	1280
#define WIN_HEIGHT	960

#define AR0234_CID_CSI_PORT         (V4L2_CID_USER_BASE | 0x1001)
#define AR0234_CID_I2C_BUS         (V4L2_CID_USER_BASE | 0x1002)
#define AR0234_CID_I2C_ID         (V4L2_CID_USER_BASE | 0x1003)
#define AR0234_CID_I2C_SLAVE_ADDRESS         (V4L2_CID_USER_BASE | 0x1004)
#define AR0234_CID_FPS         (V4L2_CID_USER_BASE | 0x1005)
#define AR0234_CID_FRAME_INTERVAL	(V4L2_CID_USER_BASE | 0x1006)

#define to_ar0234(_sd)			container_of(_sd, struct ar0234, sd)

enum {
	AR0234_LINK_FREQ_360MBPS,
	AR0234_LINK_FREQ_300MBPS,
	AR0234_LINK_FREQ_288MBPS,
	AR0234_LINK_FREQ_240MBPS,
	AR0234_LINK_FREQ_22_5MBPS,
};

struct ar0234_reg {
	u16 address;
	u16 val;
};

struct ar0234_reg_list {
	u32 num_of_regs;
	const struct ar0234_reg *regs;
};

struct ar0234_link_freq_config {
	const struct ar0234_reg_list reg_list;
};

struct ar0234_mode {
	/* Frame width in pixels */
	u32 width;

	/* Frame height in pixels */
	u32 height;

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
	const struct ar0234_reg_list reg_list;
};

static const struct ar0234_reg freq_1280x960_10bit_4lane_300M[] = {
	{0x302A, 0x0005},
	{0x302C, 0x0002},
	{0x302E, 0x0009},
	{0x3030, 0x00C8},
	{0x3036, 0x000A},
	{0x3038, 0x0002},
	{0x31B0, 0x005C},
	{0x31B2, 0x0046},
	{0x31B4, 0x31C6},
	{0x31B6, 0x2190},
	{0x31B8, 0x6049},
	{0x31BA, 0x0208},
	{0x31BC, 0x8A06},
	{0x31AE, 0x0204},
	{0x3002, 0x0080},
	{0x3004, 0x0148},
	{0x3006, 0x043F},
	{0x3008, 0x0647},
	{0x300A, 0x0983},
	{0x300C, 0x0268},
	{0x3012, 0x093E},
	{0x31AC, 0x0A0A},
	{0x306E, 0x9010},
	{0x30A2, 0x0001},
	{0x30A6, 0x0001},
	{0x3082, 0x0003},
	{0x3040, 0x0000},
	{0x31D0, 0x0000},
};

static const struct ar0234_reg freq_1280x960_8bit_4lane_240M[] = {
	{0x302A, 0x0008},
	{0x302C, 0x0001},
	{0x302E, 0x0009},
	{0x3030, 0x00A0},
	{0x3036, 0x0008},
	{0x3038, 0x0002},
	{0x31B0, 0x005B},
	{0x31B2, 0x0046},
	{0x31B4, 0x1206},
	{0x31B6, 0x2193},
	{0x31B8, 0x604B},
	{0x31BA, 0x0188},
	{0x31BC, 0x8A06},
	{0x31AE, 0x0204},
	{0x3002, 0x0080},
	{0x3004, 0x0148},
	{0x3006, 0x043F},
	{0x3008, 0x0647},
	{0x300A, 0x0983},
	{0x300C, 0x0268},
	{0x3012, 0x093E},
	{0x31AC, 0x0A08},
	{0x306E, 0x9010},
	{0x30A2, 0x0001},
	{0x30A6, 0x0001},
	{0x3082, 0x0003},
	{0x3040, 0x0000},
	{0x31D0, 0x0000},
	{0x3354, 0x002a},
	{0x3354, 0x002a},
	{0x3354, 0x002a},
};

static const struct ar0234_reg freq_1280x960_10bit_2lane_360M[] = {
	{0x302A, 0x0005},
	{0x302C, 0x0004},
	{0x302E, 0x0003},
	{0x3030, 0x0050},
	{0x3036, 0x000A},
	{0x3038, 0x0002},
	{0x31B0, 0x006E},
	{0x31B2, 0x0050},
	{0x31B4, 0x4207},
	{0x31B6, 0x2213},
	{0x31B8, 0x704A},
	{0x31BA, 0x0289},
	{0x31BC, 0x8C08},
	{0x31AE, 0x0202},
	{0x3002, 0x0080},
	{0x3004, 0x0148},
	{0x3006, 0x043F},
	{0x3008, 0x0647},
	{0x300A, 0x05B5},
	{0x300C, 0x0268},
	{0x3012, 0x058C},
	{0x31AC, 0x0A0A},
	{0x306E, 0x9010},
	{0x30A2, 0x0001},
	{0x30A6, 0x0001},
	{0x3082, 0x0003},
	{0x3040, 0x0000},
	{0x31D0, 0x0000},
};

static const struct ar0234_reg freq_1280x960_8bit_2lane_288M[] = {
	{0x302A, 0x0008},
	{0x302C, 0x0002},
	{0x302E, 0x0003},
	{0x3030, 0x0040},
	{0x3036, 0x0008},
	{0x3038, 0x0002},
	{0x31B0, 0x006A},
	{0x31B2, 0x004F},
	{0x31B4, 0x1207},
	{0x31B6, 0x2216},
	{0x31B8, 0x704B},
	{0x31BA, 0x0209},
	{0x31BC, 0x8C08},
	{0x31AE, 0x0202},
	{0x3002, 0x0080},
	{0x3004, 0x0148},
	{0x3006, 0x043F},
	{0x3008, 0x0647},
	{0x300A, 0x05B5},
	{0x300C, 0x0268},
	{0x3012, 0x058C},
	{0x31AC, 0x0A08},
	{0x306E, 0x9010},
	{0x30A2, 0x0001},
	{0x30A6, 0x0001},
	{0x3082, 0x0003},
	{0x3040, 0x0000},
	{0x31D0, 0x0000},
	{0x3354, 0x002a},
	{0x3354, 0x002a},
	{0x3354, 0x002a},
};

static const struct ar0234_reg freq_1920x1200_10bit_2lane_22_5M[] = {
{0x302A, 0x0005},		//VT_PIX_CLK_DIV = 5
{0x302C, 0x0002},		//VT_SYS_CLK_DIV = 2
{0x302E, 0x0003},		//PRE_PLL_CLK_DIV = 3
{0x3030, 0x0032},		//PLL_MULTIPLIER = 50
{0x3036, 0x000A},		//OP_PIX_CLK_DIV = 10
{0x3038, 0x0002},		//OP_SYS_CLK_DIV = 2
{0x31B0, 0x004C},		//FRAME_PREAMBLE = 76
{0x31B2, 0x003C},		//LINE_PREAMBLE = 60
{0x31B4, 0x31C5},		//MIPI_TIMING_0 = 12741
{0x31B6, 0x214C},		//MIPI_TIMING_1 = 8524
{0x31B8, 0x5048},		//MIPI_TIMING_2 = 20552
{0x31BA, 0x0186},		//MIPI_TIMING_3 = 390
{0x31BC, 0x0805},		//MIPI_TIMING_4 = 2053
{0x3354, 0x002B},		//MIPI_CNTRL = 43

{0x301A, 0x2058},		//RESET_REGISTER = 8280
{0x31AE, 0x0202},		//SERIAL_FORMAT = 514
{0x3002, 0x0008},		//Y_ADDR_START = 8
{0x3004, 0x0008},		//X_ADDR_START = 8
{0x3006, 0x04B7},		//Y_ADDR_END = 1207
{0x3008, 0x0787},		//X_ADDR_END = 1927
{0x300A, 0x04C4},		//FRAME_LENGTH_LINES = 1220
{0x300C, 0x0264},		//LINE_LENGTH_PCK = 612
{0x3012, 0x0024},		//COARSE_INTEGRATION_TIME = 36
{0x31AC, 0x0A0A},		//DATA_FORMAT_BITS = 2570
{0x306E, 0x9010},		//DATAPATH_SELECT = 36880
{0x30A2, 0x0001},		//X_ODD_INC = 1
{0x30A6, 0x0001},		//Y_ODD_INC = 1
{0x3082, 0x0003},		//OPERATION_MODE_CTRL = 3
{0x3040, 0x0000},		//READ_MODE = 0
{0x31D0, 0x0000},		//COMPANDING = 0
{0x301A, 0x205C},		//RESET_REGISTER = 8284
};

static const struct ar0234_reg mode_1280x960_8bit_4lane[] = {
	// Reset {
	{0x301a, 0x00d9},
	{0x0000, 0x00c8},
	//		1D-DDC_Parameters {
	{0x3F4C, 0x121F},
	{0x3F4E, 0x121F},
	{0x3F50, 0x0B81},
	{0x0000, 0x00c8},
	//		1D-DDC_Parameters }
	// Reset }
	// AR0234CS REV2 Sequencer-brt_spots_ablo_gnd_11feb19 {
	{0x3088, 0x8000},
	{0x3086, 0xC1AE},
	{0x3086, 0x327F},
	{0x3086, 0x5780},
	{0x3086, 0x272F},
	{0x3086, 0x7416},
	{0x3086, 0x7E13},
	{0x3086, 0x8000},
	{0x3086, 0x307E},
	{0x3086, 0xFF80},
	{0x3086, 0x20C3},
	{0x3086, 0xB00E},
	{0x3086, 0x8190},
	{0x3086, 0x1643},
	{0x3086, 0x1651},
	{0x3086, 0x9D3E},
	{0x3086, 0x9545},
	{0x3086, 0x2209},
	{0x3086, 0x3781},
	{0x3086, 0x9016},
	{0x3086, 0x4316},
	{0x3086, 0x7F90},
	{0x3086, 0x8000},
	{0x3086, 0x387F},
	{0x3086, 0x1380},
	{0x3086, 0x233B},
	{0x3086, 0x7F93},
	{0x3086, 0x4502},
	{0x3086, 0x8000},
	{0x3086, 0x7FB0},
	{0x3086, 0x8D66},
	{0x3086, 0x7F90},
	{0x3086, 0x8192},
	{0x3086, 0x3C16},
	{0x3086, 0x357F},
	{0x3086, 0x9345},
	{0x3086, 0x0280},
	{0x3086, 0x007F},
	{0x3086, 0xB08D},
	{0x3086, 0x667F},
	{0x3086, 0x9081},
	{0x3086, 0x8237},
	{0x3086, 0x4502},
	{0x3086, 0x3681},
	{0x3086, 0x8044},
	{0x3086, 0x1631},
	{0x3086, 0x4374},
	{0x3086, 0x1678},
	{0x3086, 0x7B7D},
	{0x3086, 0x4502},
	{0x3086, 0x450A},
	{0x3086, 0x7E12},
	{0x3086, 0x8180},
	{0x3086, 0x377F},
	{0x3086, 0x1045},
	{0x3086, 0x0A0E},
	{0x3086, 0x7FD4},
	{0x3086, 0x8024},
	{0x3086, 0x0E82},
	{0x3086, 0x9CC2},
	{0x3086, 0xAFA8},
	{0x3086, 0xAA03},
	{0x3086, 0x430D},
	{0x3086, 0x2D46},
	{0x3086, 0x4316},
	{0x3086, 0x5F16},
	{0x3086, 0x530D},
	{0x3086, 0x1660},
	{0x3086, 0x401E},
	{0x3086, 0x2904},
	{0x3086, 0x2984},
	{0x3086, 0x81E7},
	{0x3086, 0x816F},
	{0x3086, 0x1706},
	{0x3086, 0x81E7},
	{0x3086, 0x7F81},
	{0x3086, 0x5C0D},
	{0x3086, 0x5754},
	{0x3086, 0x495F},
	{0x3086, 0x5305},
	{0x3086, 0x5307},
	{0x3086, 0x4D2B},
	{0x3086, 0xF810},
	{0x3086, 0x164C},
	{0x3086, 0x0755},
	{0x3086, 0x562B},
	{0x3086, 0xB82B},
	{0x3086, 0x984E},
	{0x3086, 0x1129},
	{0x3086, 0x9460},
	{0x3086, 0x5C09},
	{0x3086, 0x5C1B},
	{0x3086, 0x4002},
	{0x3086, 0x4500},
	{0x3086, 0x4580},
	{0x3086, 0x29B6},
	{0x3086, 0x7F80},
	{0x3086, 0x4004},
	{0x3086, 0x7F88},
	{0x3086, 0x4109},
	{0x3086, 0x5C0B},
	{0x3086, 0x29B2},
	{0x3086, 0x4115},
	{0x3086, 0x5C03},
	{0x3086, 0x4105},
	{0x3086, 0x5F2B},
	{0x3086, 0x902B},
	{0x3086, 0x8081},
	{0x3086, 0x6F40},
	{0x3086, 0x1041},
	{0x3086, 0x0160},
	{0x3086, 0x29A2},
	{0x3086, 0x29A3},
	{0x3086, 0x5F4D},
	{0x3086, 0x1C17},
	{0x3086, 0x0281},
	{0x3086, 0xE729},
	{0x3086, 0x8345},
	{0x3086, 0x8840},
	{0x3086, 0x0F7F},
	{0x3086, 0x8A40},
	{0x3086, 0x2345},
	{0x3086, 0x8024},
	{0x3086, 0x4008},
	{0x3086, 0x7F88},
	{0x3086, 0x5D29},
	{0x3086, 0x9288},
	{0x3086, 0x102B},
	{0x3086, 0x0489},
	{0x3086, 0x165C},
	{0x3086, 0x4386},
	{0x3086, 0x170B},
	{0x3086, 0x5C03},
	{0x3086, 0x8A48},
	{0x3086, 0x4D4E},
	{0x3086, 0x2B80},
	{0x3086, 0x4C09},
	{0x3086, 0x4119},
	{0x3086, 0x816F},
	{0x3086, 0x4110},
	{0x3086, 0x4001},
	{0x3086, 0x6029},
	{0x3086, 0x8229},
	{0x3086, 0x8329},
	{0x3086, 0x435C},
	{0x3086, 0x055F},
	{0x3086, 0x4D1C},
	{0x3086, 0x81E7},
	{0x3086, 0x4502},
	{0x3086, 0x8180},
	{0x3086, 0x7F80},
	{0x3086, 0x410A},
	{0x3086, 0x9144},
	{0x3086, 0x1609},
	{0x3086, 0x2FC3},
	{0x3086, 0xB130},
	{0x3086, 0xC3B1},
	{0x3086, 0x0343},
	{0x3086, 0x164A},
	{0x3086, 0x0A43},
	{0x3086, 0x160B},
	{0x3086, 0x4316},
	{0x3086, 0x8F43},
	{0x3086, 0x1690},
	{0x3086, 0x4316},
	{0x3086, 0x7F81},
	{0x3086, 0x450A},
	{0x3086, 0x410F},
	{0x3086, 0x7F83},
	{0x3086, 0x5D29},
	{0x3086, 0x4488},
	{0x3086, 0x102B},
	{0x3086, 0x0453},
	{0x3086, 0x0D40},
	{0x3086, 0x2345},
	{0x3086, 0x0240},
	{0x3086, 0x087F},
	{0x3086, 0x8053},
	{0x3086, 0x0D89},
	{0x3086, 0x165C},
	{0x3086, 0x4586},
	{0x3086, 0x170B},
	{0x3086, 0x5C05},
	{0x3086, 0x8A60},
	{0x3086, 0x4B91},
	{0x3086, 0x4416},
	{0x3086, 0x09C1},
	{0x3086, 0x2CA9},
	{0x3086, 0xAB30},
	{0x3086, 0x51B3},
	{0x3086, 0x3D5A},
	{0x3086, 0x7E3D},
	{0x3086, 0x7E19},
	{0x3086, 0x8000},
	{0x3086, 0x8B1F},
	{0x3086, 0x2A1F},
	{0x3086, 0x83A2},
	{0x3086, 0x7516},
	{0x3086, 0xAD33},
	{0x3086, 0x450A},
	{0x3086, 0x7F53},
	{0x3086, 0x8023},
	{0x3086, 0x8C66},
	{0x3086, 0x7F13},
	{0x3086, 0x8184},
	{0x3086, 0x1481},
	{0x3086, 0x8031},
	{0x3086, 0x3D64},
	{0x3086, 0x452A},
	{0x3086, 0x9451},
	{0x3086, 0x9E96},
	{0x3086, 0x3D2B},
	{0x3086, 0x3D1B},
	{0x3086, 0x529F},
	{0x3086, 0x0E3D},
	{0x3086, 0x083D},
	{0x3086, 0x167E},
	{0x3086, 0x307E},
	{0x3086, 0x1175},
	{0x3086, 0x163E},
	{0x3086, 0x970E},
	{0x3086, 0x82B2},
	{0x3086, 0x3D7F},
	{0x3086, 0xAC3E},
	{0x3086, 0x4502},
	{0x3086, 0x7E11},
	{0x3086, 0x7FD0},
	{0x3086, 0x8000},
	{0x3086, 0x8C66},
	{0x3086, 0x7F90},
	{0x3086, 0x8194},
	{0x3086, 0x3F44},
	{0x3086, 0x1681},
	{0x3086, 0x8416},
	{0x3086, 0x2C2C},
	{0x3086, 0x2C2C},
	// AR0234CS REV2 Sequencer-brt_spots_ablo_gnd_11feb19 }
	// RECOMMENDED_SETTINGS-Pxlclk90MHz {
	{0x3044, 0x0410},
	{0x3094, 0x03D4},
	{0x3096, 0x0480},
	{0x30BA, 0x7602},
	{0x30FE, 0x002A},
	{0x31DE, 0x0410},
	{0x3ED6, 0x1435},
	{0x3ED8, 0x9865},
	{0x3EDA, 0x7698},
	{0x3EDC, 0x99FF},
	{0x3EE2, 0xBB88},
	{0x3EE4, 0x8836},
	{0x3EF0, 0x1CF0},
	{0x3EF2, 0x0000},
	{0x3EF8, 0x6166},
	{0x3EFA, 0x3333},
	{0x3EFC, 0x6634},
	{0x3276, 0x05DC},
	{0x3F00, 0x9D05},
	{0x3EEE, 0xA4FE},
	{0x30BA, 0x7602},
	//		Rowbanding_Settings {
	{0x3EEC, 0x0C0C},
	{0x3EE8, 0xAAE4},
	//		Rowbanding_Settings }
	//		Gain Table 90MHz {
	//		Gain Table 90MHz }
	{0x3102, 0x5000},
	//		AutoExposure Maximum Analog Gain 16x {
	{0x3EEE, 0xA4AA},
	//		AutoExposure Maximum Analog Gain 16x }
	// RECOMMENDED_SETTINGS-Pxlclk90MHz }
	{0x3270, 0x0100},
	{0x3270, 0x0100},
	{0x3270, 0x0100},
};

static const struct ar0234_reg mode_1280x960_10bit_2lane[] = {
	{0x0000, 0x00C8},
	{0x301A, 0x00D9},
	{0x0000, 0x00C8},
	{0x3F4C, 0x121F},
	{0x3F4E, 0x121F},
	{0x3F50, 0x0B81},
	{0x31E0, 0x0003},
	{0x31E0, 0x0003},
	{0x30B0, 0x0028},
	{0x0000, 0x00C8},
	{0x3088, 0x8000},
	{0x3086, 0xC1AE},
	{0x3086, 0x327F},
	{0x3086, 0x5780},
	{0x3086, 0x272F},
	{0x3086, 0x7416},
	{0x3086, 0x7E13},
	{0x3086, 0x8000},
	{0x3086, 0x307E},
	{0x3086, 0xFF80},
	{0x3086, 0x20C3},
	{0x3086, 0xB00E},
	{0x3086, 0x8190},
	{0x3086, 0x1643},
	{0x3086, 0x1651},
	{0x3086, 0x9D3E},
	{0x3086, 0x9545},
	{0x3086, 0x2209},
	{0x3086, 0x3781},
	{0x3086, 0x9016},
	{0x3086, 0x4316},
	{0x3086, 0x7F90},
	{0x3086, 0x8000},
	{0x3086, 0x387F},
	{0x3086, 0x1380},
	{0x3086, 0x233B},
	{0x3086, 0x7F93},
	{0x3086, 0x4502},
	{0x3086, 0x8000},
	{0x3086, 0x7FB0},
	{0x3086, 0x8D66},
	{0x3086, 0x7F90},
	{0x3086, 0x8192},
	{0x3086, 0x3C16},
	{0x3086, 0x357F},
	{0x3086, 0x9345},
	{0x3086, 0x0280},
	{0x3086, 0x007F},
	{0x3086, 0xB08D},
	{0x3086, 0x667F},
	{0x3086, 0x9081},
	{0x3086, 0x8237},
	{0x3086, 0x4502},
	{0x3086, 0x3681},
	{0x3086, 0x8044},
	{0x3086, 0x1631},
	{0x3086, 0x4374},
	{0x3086, 0x1678},
	{0x3086, 0x7B7D},
	{0x3086, 0x4502},
	{0x3086, 0x450A},
	{0x3086, 0x7E12},
	{0x3086, 0x8180},
	{0x3086, 0x377F},
	{0x3086, 0x1045},
	{0x3086, 0x0A0E},
	{0x3086, 0x7FD4},
	{0x3086, 0x8024},
	{0x3086, 0x0E82},
	{0x3086, 0x9CC2},
	{0x3086, 0xAFA8},
	{0x3086, 0xAA03},
	{0x3086, 0x430D},
	{0x3086, 0x2D46},
	{0x3086, 0x4316},
	{0x3086, 0x5F16},
	{0x3086, 0x530D},
	{0x3086, 0x1660},
	{0x3086, 0x401E},
	{0x3086, 0x2904},
	{0x3086, 0x2984},
	{0x3086, 0x81E7},
	{0x3086, 0x816F},
	{0x3086, 0x1706},
	{0x3086, 0x81E7},
	{0x3086, 0x7F81},
	{0x3086, 0x5C0D},
	{0x3086, 0x5754},
	{0x3086, 0x495F},
	{0x3086, 0x5305},
	{0x3086, 0x5307},
	{0x3086, 0x4D2B},
	{0x3086, 0xF810},
	{0x3086, 0x164C},
	{0x3086, 0x0755},
	{0x3086, 0x562B},
	{0x3086, 0xB82B},
	{0x3086, 0x984E},
	{0x3086, 0x1129},
	{0x3086, 0x9460},
	{0x3086, 0x5C09},
	{0x3086, 0x5C1B},
	{0x3086, 0x4002},
	{0x3086, 0x4500},
	{0x3086, 0x4580},
	{0x3086, 0x29B6},
	{0x3086, 0x7F80},
	{0x3086, 0x4004},
	{0x3086, 0x7F88},
	{0x3086, 0x4109},
	{0x3086, 0x5C0B},
	{0x3086, 0x29B2},
	{0x3086, 0x4115},
	{0x3086, 0x5C03},
	{0x3086, 0x4105},
	{0x3086, 0x5F2B},
	{0x3086, 0x902B},
	{0x3086, 0x8081},
	{0x3086, 0x6F40},
	{0x3086, 0x1041},
	{0x3086, 0x0160},
	{0x3086, 0x29A2},
	{0x3086, 0x29A3},
	{0x3086, 0x5F4D},
	{0x3086, 0x1C17},
	{0x3086, 0x0281},
	{0x3086, 0xE729},
	{0x3086, 0x8345},
	{0x3086, 0x8840},
	{0x3086, 0x0F7F},
	{0x3086, 0x8A40},
	{0x3086, 0x2345},
	{0x3086, 0x8024},
	{0x3086, 0x4008},
	{0x3086, 0x7F88},
	{0x3086, 0x5D29},
	{0x3086, 0x9288},
	{0x3086, 0x102B},
	{0x3086, 0x0489},
	{0x3086, 0x165C},
	{0x3086, 0x4386},
	{0x3086, 0x170B},
	{0x3086, 0x5C03},
	{0x3086, 0x8A48},
	{0x3086, 0x4D4E},
	{0x3086, 0x2B80},
	{0x3086, 0x4C09},
	{0x3086, 0x4119},
	{0x3086, 0x816F},
	{0x3086, 0x4110},
	{0x3086, 0x4001},
	{0x3086, 0x6029},
	{0x3086, 0x8229},
	{0x3086, 0x8329},
	{0x3086, 0x435C},
	{0x3086, 0x055F},
	{0x3086, 0x4D1C},
	{0x3086, 0x81E7},
	{0x3086, 0x4502},
	{0x3086, 0x8180},
	{0x3086, 0x7F80},
	{0x3086, 0x410A},
	{0x3086, 0x9144},
	{0x3086, 0x1609},
	{0x3086, 0x2FC3},
	{0x3086, 0xB130},
	{0x3086, 0xC3B1},
	{0x3086, 0x0343},
	{0x3086, 0x164A},
	{0x3086, 0x0A43},
	{0x3086, 0x160B},
	{0x3086, 0x4316},
	{0x3086, 0x8F43},
	{0x3086, 0x1690},
	{0x3086, 0x4316},
	{0x3086, 0x7F81},
	{0x3086, 0x450A},
	{0x3086, 0x410F},
	{0x3086, 0x7F83},
	{0x3086, 0x5D29},
	{0x3086, 0x4488},
	{0x3086, 0x102B},
	{0x3086, 0x0453},
	{0x3086, 0x0D40},
	{0x3086, 0x2345},
	{0x3086, 0x0240},
	{0x3086, 0x087F},
	{0x3086, 0x8053},
	{0x3086, 0x0D89},
	{0x3086, 0x165C},
	{0x3086, 0x4586},
	{0x3086, 0x170B},
	{0x3086, 0x5C05},
	{0x3086, 0x8A60},
	{0x3086, 0x4B91},
	{0x3086, 0x4416},
	{0x3086, 0x09C1},
	{0x3086, 0x2CA9},
	{0x3086, 0xAB30},
	{0x3086, 0x51B3},
	{0x3086, 0x3D5A},
	{0x3086, 0x7E3D},
	{0x3086, 0x7E19},
	{0x3086, 0x8000},
	{0x3086, 0x8B1F},
	{0x3086, 0x2A1F},
	{0x3086, 0x83A2},
	{0x3086, 0x7516},
	{0x3086, 0xAD33},
	{0x3086, 0x450A},
	{0x3086, 0x7F53},
	{0x3086, 0x8023},
	{0x3086, 0x8C66},
	{0x3086, 0x7F13},
	{0x3086, 0x8184},
	{0x3086, 0x1481},
	{0x3086, 0x8031},
	{0x3086, 0x3D64},
	{0x3086, 0x452A},
	{0x3086, 0x9451},
	{0x3086, 0x9E96},
	{0x3086, 0x3D2B},
	{0x3086, 0x3D1B},
	{0x3086, 0x529F},
	{0x3086, 0x0E3D},
	{0x3086, 0x083D},
	{0x3086, 0x167E},
	{0x3086, 0x307E},
	{0x3086, 0x1175},
	{0x3086, 0x163E},
	{0x3086, 0x970E},
	{0x3086, 0x82B2},
	{0x3086, 0x3D7F},
	{0x3086, 0xAC3E},
	{0x3086, 0x4502},
	{0x3086, 0x7E11},
	{0x3086, 0x7FD0},
	{0x3086, 0x8000},
	{0x3086, 0x8C66},
	{0x3086, 0x7F90},
	{0x3086, 0x8194},
	{0x3086, 0x3F44},
	{0x3086, 0x1681},
	{0x3086, 0x8416},
	{0x3086, 0x2C2C},
	{0x3086, 0x2C2C},
	{0x302A, 0x0005},
	{0x302C, 0x0001},
	{0x302E, 0x0003},
	{0x3030, 0x0032},
	{0x3036, 0x000A},
	{0x3038, 0x0001},
	{0x30B0, 0x0028},
	{0x31B0, 0x0082},
	{0x31B2, 0x005C},
	{0x31B4, 0x5248},
	{0x31B6, 0x3257},
	{0x31B8, 0x904B},
	{0x31BA, 0x030B},
	{0x31BC, 0x8E09},
	{0x3354, 0x002B},
	{0x31D0, 0x0000},
	{0x31AE, 0x0204},
	{0x3002, 0x00D0},
	{0x3004, 0x0148},
	{0x3006, 0x048F},
	{0x3008, 0x0647},
	{0x3064, 0x1802},
	{0x300A, 0x04C4},
	{0x300C, 0x04C4},
	{0x30A2, 0x0001},
	{0x30A6, 0x0001},
	{0x3012, 0x010C},
	{0x3786, 0x0006},
	{0x31AE, 0x0202},
	{0x3088, 0x8050},
	{0x3086, 0x9237},
	{0x3044, 0x0410},
	{0x3094, 0x03D4},
	{0x3096, 0x0280},
	{0x30BA, 0x7606},
	{0x30B0, 0x0028},
	{0x30BA, 0x7600},
	{0x30FE, 0x002A},
	{0x31DE, 0x0410},
	{0x3ED6, 0x1435},
	{0x3ED8, 0x9865},
	{0x3EDA, 0x7698},
	{0x3EDC, 0x99FF},
	{0x3EE2, 0xBB88},
	{0x3EE4, 0x8836},
	{0x3EF0, 0x1CF0},
	{0x3EF2, 0x0000},
	{0x3EF8, 0x6166},
	{0x3EFA, 0x3333},
	{0x3EFC, 0x6634},
	{0x3088, 0x81BA},
	{0x3086, 0x3D02},
	{0x3276, 0x05DC},
	{0x3F00, 0x9D05},
	{0x3ED2, 0xFA86},
	{0x3EEE, 0xA4FE},
	{0x3ECC, 0x6E42},
	{0x3ECC, 0x0E42},
	{0x3EEC, 0x0C0C},
	{0x3EE8, 0xAAE4},
	{0x3EE6, 0x3363},
	{0x3EE6, 0x3363},
	{0x3EE8, 0xAAE4},
	{0x3EE8, 0xAAE4},
	{0x3180, 0xC24F},
	{0x3102, 0x5000},
	{0x3060, 0x000D},
	{0x3ED0, 0xFF44},
	{0x3ED2, 0xAA86},
	{0x3ED4, 0x031F},
	{0x3EEE, 0xA4AA},
};

static const struct ar0234_reg mode_1920x1200_10bit_2lane[] = {
	{0x0000, 0x00c8},
	{0x301A, 0x00D9}, // RESET_REGISTER
	{0x0000, 0x00c8},
	{0x3F4C, 0x121F}, // RESERVED_MFR_3F4C
	{0x3F4E, 0x121F}, // RESERVED_MFR_3F4E
	{0x3F50, 0x0B81}, // RESERVED_MFR_3F50
	{0x31E0, 0x0003}, // PIX_DEF_ID
	{0x31E0, 0x0003}, // PIX_DEF_ID
	{0x30B0, 0x0028}, // DIGITAL_TEST
	{0x0000, 0x00c8},
	{0x3088, 0x8000}, // SEQ_CTRL_PORT
	{0x3086, 0xC1AE}, // SEQ_DATA_PORT
	{0x3086, 0x327F}, // SEQ_DATA_PORT
	{0x3086, 0x5780}, // SEQ_DATA_PORT
	{0x3086, 0x272F}, // SEQ_DATA_PORT
	{0x3086, 0x7416}, // SEQ_DATA_PORT
	{0x3086, 0x7E13}, // SEQ_DATA_PORT
	{0x3086, 0x8000}, // SEQ_DATA_PORT
	{0x3086, 0x307E}, // SEQ_DATA_PORT
	{0x3086, 0xFF80}, // SEQ_DATA_PORT
	{0x3086, 0x20C3}, // SEQ_DATA_PORT
	{0x3086, 0xB00E}, // SEQ_DATA_PORT
	{0x3086, 0x8190}, // SEQ_DATA_PORT
	{0x3086, 0x1643}, // SEQ_DATA_PORT
	{0x3086, 0x1651}, // SEQ_DATA_PORT
	{0x3086, 0x9D3E}, // SEQ_DATA_PORT
	{0x3086, 0x9545}, // SEQ_DATA_PORT
	{0x3086, 0x2209}, // SEQ_DATA_PORT
	{0x3086, 0x3781}, // SEQ_DATA_PORT
	{0x3086, 0x9016}, // SEQ_DATA_PORT
	{0x3086, 0x4316}, // SEQ_DATA_PORT
	{0x3086, 0x7F90}, // SEQ_DATA_PORT
	{0x3086, 0x8000}, // SEQ_DATA_PORT
	{0x3086, 0x387F}, // SEQ_DATA_PORT
	{0x3086, 0x1380}, // SEQ_DATA_PORT
	{0x3086, 0x233B}, // SEQ_DATA_PORT
	{0x3086, 0x7F93}, // SEQ_DATA_PORT
	{0x3086, 0x4502}, // SEQ_DATA_PORT
	{0x3086, 0x8000}, // SEQ_DATA_PORT
	{0x3086, 0x7FB0}, // SEQ_DATA_PORT
	{0x3086, 0x8D66}, // SEQ_DATA_PORT
	{0x3086, 0x7F90}, // SEQ_DATA_PORT
	{0x3086, 0x8192}, // SEQ_DATA_PORT
	{0x3086, 0x3C16}, // SEQ_DATA_PORT
	{0x3086, 0x357F}, // SEQ_DATA_PORT
	{0x3086, 0x9345}, // SEQ_DATA_PORT
	{0x3086, 0x0280}, // SEQ_DATA_PORT
	{0x3086, 0x007F}, // SEQ_DATA_PORT
	{0x3086, 0xB08D}, // SEQ_DATA_PORT
	{0x3086, 0x667F}, // SEQ_DATA_PORT
	{0x3086, 0x9081}, // SEQ_DATA_PORT
	{0x3086, 0x8237}, // SEQ_DATA_PORT
	{0x3086, 0x4502}, // SEQ_DATA_PORT
	{0x3086, 0x3681}, // SEQ_DATA_PORT
	{0x3086, 0x8044}, // SEQ_DATA_PORT
	{0x3086, 0x1631}, // SEQ_DATA_PORT
	{0x3086, 0x4374}, // SEQ_DATA_PORT
	{0x3086, 0x1678}, // SEQ_DATA_PORT
	{0x3086, 0x7B7D}, // SEQ_DATA_PORT
	{0x3086, 0x4502}, // SEQ_DATA_PORT
	{0x3086, 0x450A}, // SEQ_DATA_PORT
	{0x3086, 0x7E12}, // SEQ_DATA_PORT
	{0x3086, 0x8180}, // SEQ_DATA_PORT
	{0x3086, 0x377F}, // SEQ_DATA_PORT
	{0x3086, 0x1045}, // SEQ_DATA_PORT
	{0x3086, 0x0A0E}, // SEQ_DATA_PORT
	{0x3086, 0x7FD4}, // SEQ_DATA_PORT
	{0x3086, 0x8024}, // SEQ_DATA_PORT
	{0x3086, 0x0E82}, // SEQ_DATA_PORT
	{0x3086, 0x9CC2}, // SEQ_DATA_PORT
	{0x3086, 0xAFA8}, // SEQ_DATA_PORT
	{0x3086, 0xAA03}, // SEQ_DATA_PORT
	{0x3086, 0x430D}, // SEQ_DATA_PORT
	{0x3086, 0x2D46}, // SEQ_DATA_PORT
	{0x3086, 0x4316}, // SEQ_DATA_PORT
	{0x3086, 0x5F16}, // SEQ_DATA_PORT
	{0x3086, 0x530D}, // SEQ_DATA_PORT
	{0x3086, 0x1660}, // SEQ_DATA_PORT
	{0x3086, 0x401E}, // SEQ_DATA_PORT
	{0x3086, 0x2904}, // SEQ_DATA_PORT
	{0x3086, 0x2984}, // SEQ_DATA_PORT
	{0x3086, 0x81E7}, // SEQ_DATA_PORT
	{0x3086, 0x816F}, // SEQ_DATA_PORT
	{0x3086, 0x1706}, // SEQ_DATA_PORT
	{0x3086, 0x81E7}, // SEQ_DATA_PORT
	{0x3086, 0x7F81}, // SEQ_DATA_PORT
	{0x3086, 0x5C0D}, // SEQ_DATA_PORT
	{0x3086, 0x5754}, // SEQ_DATA_PORT
	{0x3086, 0x495F}, // SEQ_DATA_PORT
	{0x3086, 0x5305}, // SEQ_DATA_PORT
	{0x3086, 0x5307}, // SEQ_DATA_PORT
	{0x3086, 0x4D2B}, // SEQ_DATA_PORT
	{0x3086, 0xF810}, // SEQ_DATA_PORT
	{0x3086, 0x164C}, // SEQ_DATA_PORT
	{0x3086, 0x0755}, // SEQ_DATA_PORT
	{0x3086, 0x562B}, // SEQ_DATA_PORT
	{0x3086, 0xB82B}, // SEQ_DATA_PORT
	{0x3086, 0x984E}, // SEQ_DATA_PORT
	{0x3086, 0x1129}, // SEQ_DATA_PORT
	{0x3086, 0x9460}, // SEQ_DATA_PORT
	{0x3086, 0x5C09}, // SEQ_DATA_PORT
	{0x3086, 0x5C1B}, // SEQ_DATA_PORT
	{0x3086, 0x4002}, // SEQ_DATA_PORT
	{0x3086, 0x4500}, // SEQ_DATA_PORT
	{0x3086, 0x4580}, // SEQ_DATA_PORT
	{0x3086, 0x29B6}, // SEQ_DATA_PORT
	{0x3086, 0x7F80}, // SEQ_DATA_PORT
	{0x3086, 0x4004}, // SEQ_DATA_PORT
	{0x3086, 0x7F88}, // SEQ_DATA_PORT
	{0x3086, 0x4109}, // SEQ_DATA_PORT
	{0x3086, 0x5C0B}, // SEQ_DATA_PORT
	{0x3086, 0x29B2}, // SEQ_DATA_PORT
	{0x3086, 0x4115}, // SEQ_DATA_PORT
	{0x3086, 0x5C03}, // SEQ_DATA_PORT
	{0x3086, 0x4105}, // SEQ_DATA_PORT
	{0x3086, 0x5F2B}, // SEQ_DATA_PORT
	{0x3086, 0x902B}, // SEQ_DATA_PORT
	{0x3086, 0x8081}, // SEQ_DATA_PORT
	{0x3086, 0x6F40}, // SEQ_DATA_PORT
	{0x3086, 0x1041}, // SEQ_DATA_PORT
	{0x3086, 0x0160}, // SEQ_DATA_PORT
	{0x3086, 0x29A2}, // SEQ_DATA_PORT
	{0x3086, 0x29A3}, // SEQ_DATA_PORT
	{0x3086, 0x5F4D}, // SEQ_DATA_PORT
	{0x3086, 0x1C17}, // SEQ_DATA_PORT
	{0x3086, 0x0281}, // SEQ_DATA_PORT
	{0x3086, 0xE729}, // SEQ_DATA_PORT
	{0x3086, 0x8345}, // SEQ_DATA_PORT
	{0x3086, 0x8840}, // SEQ_DATA_PORT
	{0x3086, 0x0F7F}, // SEQ_DATA_PORT
	{0x3086, 0x8A40}, // SEQ_DATA_PORT
	{0x3086, 0x2345}, // SEQ_DATA_PORT
	{0x3086, 0x8024}, // SEQ_DATA_PORT
	{0x3086, 0x4008}, // SEQ_DATA_PORT
	{0x3086, 0x7F88}, // SEQ_DATA_PORT
	{0x3086, 0x5D29}, // SEQ_DATA_PORT
	{0x3086, 0x9288}, // SEQ_DATA_PORT
	{0x3086, 0x102B}, // SEQ_DATA_PORT
	{0x3086, 0x0489}, // SEQ_DATA_PORT
	{0x3086, 0x165C}, // SEQ_DATA_PORT
	{0x3086, 0x4386}, // SEQ_DATA_PORT
	{0x3086, 0x170B}, // SEQ_DATA_PORT
	{0x3086, 0x5C03}, // SEQ_DATA_PORT
	{0x3086, 0x8A48}, // SEQ_DATA_PORT
	{0x3086, 0x4D4E}, // SEQ_DATA_PORT
	{0x3086, 0x2B80}, // SEQ_DATA_PORT
	{0x3086, 0x4C09}, // SEQ_DATA_PORT
	{0x3086, 0x4119}, // SEQ_DATA_PORT
	{0x3086, 0x816F}, // SEQ_DATA_PORT
	{0x3086, 0x4110}, // SEQ_DATA_PORT
	{0x3086, 0x4001}, // SEQ_DATA_PORT
	{0x3086, 0x6029}, // SEQ_DATA_PORT
	{0x3086, 0x8229}, // SEQ_DATA_PORT
	{0x3086, 0x8329}, // SEQ_DATA_PORT
	{0x3086, 0x435C}, // SEQ_DATA_PORT
	{0x3086, 0x055F}, // SEQ_DATA_PORT
	{0x3086, 0x4D1C}, // SEQ_DATA_PORT
	{0x3086, 0x81E7}, // SEQ_DATA_PORT
	{0x3086, 0x4502}, // SEQ_DATA_PORT
	{0x3086, 0x8180}, // SEQ_DATA_PORT
	{0x3086, 0x7F80}, // SEQ_DATA_PORT
	{0x3086, 0x410A}, // SEQ_DATA_PORT
	{0x3086, 0x9144}, // SEQ_DATA_PORT
	{0x3086, 0x1609}, // SEQ_DATA_PORT
	{0x3086, 0x2FC3}, // SEQ_DATA_PORT
	{0x3086, 0xB130}, // SEQ_DATA_PORT
	{0x3086, 0xC3B1}, // SEQ_DATA_PORT
	{0x3086, 0x0343}, // SEQ_DATA_PORT
	{0x3086, 0x164A}, // SEQ_DATA_PORT
	{0x3086, 0x0A43}, // SEQ_DATA_PORT
	{0x3086, 0x160B}, // SEQ_DATA_PORT
	{0x3086, 0x4316}, // SEQ_DATA_PORT
	{0x3086, 0x8F43}, // SEQ_DATA_PORT
	{0x3086, 0x1690}, // SEQ_DATA_PORT
	{0x3086, 0x4316}, // SEQ_DATA_PORT
	{0x3086, 0x7F81}, // SEQ_DATA_PORT
	{0x3086, 0x450A}, // SEQ_DATA_PORT
	{0x3086, 0x410F}, // SEQ_DATA_PORT
	{0x3086, 0x7F83}, // SEQ_DATA_PORT
	{0x3086, 0x5D29}, // SEQ_DATA_PORT
	{0x3086, 0x4488}, // SEQ_DATA_PORT
	{0x3086, 0x102B}, // SEQ_DATA_PORT
	{0x3086, 0x0453}, // SEQ_DATA_PORT
	{0x3086, 0x0D40}, // SEQ_DATA_PORT
	{0x3086, 0x2345}, // SEQ_DATA_PORT
	{0x3086, 0x0240}, // SEQ_DATA_PORT
	{0x3086, 0x087F}, // SEQ_DATA_PORT
	{0x3086, 0x8053}, // SEQ_DATA_PORT
	{0x3086, 0x0D89}, // SEQ_DATA_PORT
	{0x3086, 0x165C}, // SEQ_DATA_PORT
	{0x3086, 0x4586}, // SEQ_DATA_PORT
	{0x3086, 0x170B}, // SEQ_DATA_PORT
	{0x3086, 0x5C05}, // SEQ_DATA_PORT
	{0x3086, 0x8A60}, // SEQ_DATA_PORT
	{0x3086, 0x4B91}, // SEQ_DATA_PORT
	{0x3086, 0x4416}, // SEQ_DATA_PORT
	{0x3086, 0x09C1}, // SEQ_DATA_PORT
	{0x3086, 0x2CA9}, // SEQ_DATA_PORT
	{0x3086, 0xAB30}, // SEQ_DATA_PORT
	{0x3086, 0x51B3}, // SEQ_DATA_PORT
	{0x3086, 0x3D5A}, // SEQ_DATA_PORT
	{0x3086, 0x7E3D}, // SEQ_DATA_PORT
	{0x3086, 0x7E19}, // SEQ_DATA_PORT
	{0x3086, 0x8000}, // SEQ_DATA_PORT
	{0x3086, 0x8B1F}, // SEQ_DATA_PORT
	{0x3086, 0x2A1F}, // SEQ_DATA_PORT
	{0x3086, 0x83A2}, // SEQ_DATA_PORT
	{0x3086, 0x7516}, // SEQ_DATA_PORT
	{0x3086, 0xAD33}, // SEQ_DATA_PORT
	{0x3086, 0x450A}, // SEQ_DATA_PORT
	{0x3086, 0x7F53}, // SEQ_DATA_PORT
	{0x3086, 0x8023}, // SEQ_DATA_PORT
	{0x3086, 0x8C66}, // SEQ_DATA_PORT
	{0x3086, 0x7F13}, // SEQ_DATA_PORT
	{0x3086, 0x8184}, // SEQ_DATA_PORT
	{0x3086, 0x1481}, // SEQ_DATA_PORT
	{0x3086, 0x8031}, // SEQ_DATA_PORT
	{0x3086, 0x3D64}, // SEQ_DATA_PORT
	{0x3086, 0x452A}, // SEQ_DATA_PORT
	{0x3086, 0x9451}, // SEQ_DATA_PORT
	{0x3086, 0x9E96}, // SEQ_DATA_PORT
	{0x3086, 0x3D2B}, // SEQ_DATA_PORT
	{0x3086, 0x3D1B}, // SEQ_DATA_PORT
	{0x3086, 0x529F}, // SEQ_DATA_PORT
	{0x3086, 0x0E3D}, // SEQ_DATA_PORT
	{0x3086, 0x083D}, // SEQ_DATA_PORT
	{0x3086, 0x167E}, // SEQ_DATA_PORT
	{0x3086, 0x307E}, // SEQ_DATA_PORT
	{0x3086, 0x1175}, // SEQ_DATA_PORT
	{0x3086, 0x163E}, // SEQ_DATA_PORT
	{0x3086, 0x970E}, // SEQ_DATA_PORT
	{0x3086, 0x82B2}, // SEQ_DATA_PORT
	{0x3086, 0x3D7F}, // SEQ_DATA_PORT
	{0x3086, 0xAC3E}, // SEQ_DATA_PORT
	{0x3086, 0x4502}, // SEQ_DATA_PORT
	{0x3086, 0x7E11}, // SEQ_DATA_PORT
	{0x3086, 0x7FD0}, // SEQ_DATA_PORT
	{0x3086, 0x8000}, // SEQ_DATA_PORT
	{0x3086, 0x8C66}, // SEQ_DATA_PORT
	{0x3086, 0x7F90}, // SEQ_DATA_PORT
	{0x3086, 0x8194}, // SEQ_DATA_PORT
	{0x3086, 0x3F44}, // SEQ_DATA_PORT
	{0x3086, 0x1681}, // SEQ_DATA_PORT
	{0x3086, 0x8416}, // SEQ_DATA_PORT
	{0x3086, 0x2C2C}, // SEQ_DATA_PORT
	{0x3086, 0x2C2C}, // SEQ_DATA_PORT
	{0x302A, 0x0005}, // VT_PIX_CLK_DIV
	{0x302C, 0x0002}, // VT_SYS_CLK_DIV
	{0x302E, 0x0003}, // PRE_PLL_CLK_DIV
	{0x3030, 0x0032}, // PLL_MULTIPLIER
	{0x3036, 0x000A}, // OP_PIX_CLK_DIV
	{0x3038, 0x0002}, // OP_SYS_CLK_DIV
	{0x30B0, 0x0028}, // DIGITAL_TEST
	{0x31B0, 0x0082}, // FRAME_PREAMBLE
	{0x31B2, 0x005C}, // LINE_PREAMBLE
	{0x31B4, 0x5248}, // MIPI_TIMING_0
	{0x31B6, 0x3257}, // MIPI_TIMING_1
	{0x31B8, 0x904B}, // MIPI_TIMING_2
	{0x31BA, 0x030B}, // MIPI_TIMING_3
	{0x31BC, 0x8E09}, // MIPI_TIMING_4
	{0x3354, 0x002B}, // MIPI_CNTRL
	{0x31D0, 0x0000}, // COMPANDING
	{0x31AE, 0x0204}, // SERIAL_FORMAT
	{0x3002, 0x0008}, // Y_ADDR_START
	{0x3004, 0x0008}, // X_ADDR_START
	{0x3006, 0x04B7}, // Y_ADDR_END
	{0x3008, 0x0787}, // X_ADDR_END
	{0x3064, 0x1802}, // SMIA_TEST
	{0x300A, 0x04C4}, // FRAME_LENGTH_LINES
	{0x300C, 0x0264}, // LINE_LENGTH_PCK
	{0x30A2, 0x0001}, // X_ODD_INC
	{0x30A6, 0x0001}, // Y_ODD_INC
	{0x3012, 0x02DC}, // COARSE_INTEGRATION_TIME
	{0x3786, 0x0006}, // DIGITAL_CTRL_1
	{0x31AE, 0x0202}, // SERIAL_FORMAT
	{0x3044, 0x0410}, // RESERVED_MFR_3044
	{0x3094, 0x03D4}, // RESERVED_MFR_3094
	{0x3096, 0x0480}, // RESERVED_MFR_3096
	{0x30BA, 0x7606}, // RESERVED_MFR_30BA
	{0x30B0, 0x0028}, // DIGITAL_TEST
	{0x30BA, 0x7600}, // RESERVED_MFR_30BA
	{0x30FE, 0x002A}, // NOISE_PEDESTAL
	{0x31DE, 0x0410}, // RESERVED_MFR_31DE
	{0x3ED6, 0x1435}, // RESERVED_MFR_3ED6
	{0x3ED8, 0x9865}, // RESERVED_MFR_3ED8
	{0x3EDA, 0x7698}, // RESERVED_MFR_3EDA
	{0x3EDC, 0x99FF}, // RESERVED_MFR_3EDC
	{0x3EE2, 0xBB88}, // RESERVED_MFR_3EE2
	{0x3EE4, 0x8836}, // RESERVED_MFR_3EE4
	{0x3EF0, 0x1CF0}, // RESERVED_MFR_3EF0
	{0x3EF2, 0x0000}, // RESERVED_MFR_3EF2
	{0x3EF8, 0x6166}, // RESERVED_MFR_3EF8
	{0x3EFA, 0x3333}, // RESERVED_MFR_3EFA
	{0x3EFC, 0x6634}, // RESERVED_MFR_3EFC
	{0x3088, 0x81BA}, // SEQ_CTRL_PORT
	{0x3086, 0x3D02}, // SEQ_DATA_PORT
	{0x3276, 0x05DC}, // RESERVED_MFR_3276
	{0x3F00, 0x9D05}, // RESERVED_MFR_3F00
	{0x3ED2, 0xFA86}, // RESERVED_MFR_3ED2
	{0x3EEE, 0xA4FE}, // RESERVED_MFR_3EEE
	{0x3ECC, 0x6D42}, // RESERVED_MFR_3ECC
	{0x3ECC, 0x0D42}, // RESERVED_MFR_3ECC
	{0x3EEC, 0x0C0C}, // RESERVED_MFR_3EEC
	{0x3EE8, 0xAAE4}, // RESERVED_MFR_3EE8
	{0x3EE6, 0x3363}, // RESERVED_MFR_3EE6
	{0x3EE6, 0x3363}, // RESERVED_MFR_3EE6
	{0x3EE8, 0xAAE4}, // RESERVED_MFR_3EE8
	{0x3EE8, 0xAAE4}, // RESERVED_MFR_3EE8
	{0x3180, 0xC24F}, // DELTA_DK_CONTROL
	{0x3102, 0x5000}, // AE_LUMA_TARGET_REG
	{0x3060, 0x000D}, // ANALOG_GAIN
	{0x3ED0, 0xFF44}, // RESERVED_MFR_3ED0
	{0x3ED2, 0xAA86}, // RESERVED_MFR_3ED2
	{0x3ED4, 0x031F}, // RESERVED_MFR_3ED4
	{0x3EEE, 0xA4AA}, // RESERVED_MFR_3EEE
};

static const s64 link_freq_menu_items[] = {
	AR0234_LINK_FREQ_360MHZ,
	AR0234_LINK_FREQ_300MHZ,
	AR0234_LINK_FREQ_288MHZ,
	AR0234_LINK_FREQ_240MHZ,
	AR0234_LINK_FREQ_22_5MHZ,
};

static const struct ar0234_link_freq_config link_freq_configs[] = {
	[AR0234_LINK_FREQ_360MBPS] = {
		.reg_list = {
			.num_of_regs =
				ARRAY_SIZE(freq_1280x960_10bit_2lane_360M),
			.regs = freq_1280x960_10bit_2lane_360M,
		}
	},
	[AR0234_LINK_FREQ_300MBPS] = {
		.reg_list = {
			.num_of_regs =
				ARRAY_SIZE(freq_1280x960_10bit_4lane_300M),
			.regs = freq_1280x960_10bit_4lane_300M,
		}
	},
	[AR0234_LINK_FREQ_288MBPS] = {
		.reg_list = {
			.num_of_regs =
				ARRAY_SIZE(freq_1280x960_8bit_2lane_288M),
			.regs = freq_1280x960_8bit_2lane_288M,
		}
	},
	[AR0234_LINK_FREQ_240MBPS] = {
		.reg_list = {
			.num_of_regs =
				ARRAY_SIZE(freq_1280x960_8bit_4lane_240M),
			.regs = freq_1280x960_8bit_4lane_240M,
		}
	},
	[AR0234_LINK_FREQ_22_5MBPS] = {
		.reg_list = {
			.num_of_regs =
				ARRAY_SIZE(freq_1920x1200_10bit_2lane_22_5M),
			.regs = freq_1920x1200_10bit_2lane_22_5M,
		}
	},
};

static const struct ar0234_mode supported_modes[] = {
	{
		.width = WIN_WIDTH,
		.height = WIN_HEIGHT,
		.hts = 2464,
		.vts_def = 2435,
		.vts_min = 2435,
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.lanes = 2,
		.fps = 30,
		.bpp = 10,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1280x960_10bit_2lane),
			.regs = mode_1280x960_10bit_2lane,
		},
		.link_freq_index = -1,
	},
	{
		.width = WIN_WIDTH,
		.height = WIN_HEIGHT,
		.hts = 2464,
		.vts_def = 2435,
		.vts_min = 2435,
		.code = MEDIA_BUS_FMT_SGRBG8_1X8,
		.lanes = 4,
		.fps = 40,
		.bpp = 8,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1280x960_8bit_4lane),
			.regs = mode_1280x960_8bit_4lane,
		},
		.link_freq_index = AR0234_LINK_FREQ_240MBPS,
	},
	{
		.width = 1920,
		.height = 1200,
		.hts = 2464,
		.vts_def = 2435,
		.vts_min = 2435,
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.lanes = 2,
		.fps = 30,
		.bpp = 10,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1920x1200_10bit_2lane),
			.regs = mode_1920x1200_10bit_2lane,
		},
		.link_freq_index = AR0234_LINK_FREQ_22_5MBPS,
	},
};

static u32 supported_formats[] = {
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGRBG8_1X8,
};

struct ar0234 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;

	/* V4L2 Controls */
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *analogue_gain;
	struct v4l2_ctrl *digital_gain;
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
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *query_sub_stream;
	struct v4l2_ctrl *set_sub_stream;

	/* Current mode */
	const struct ar0234_mode *cur_mode;

	/* To serialize asynchronus callbacks */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;

	struct ar0234_platform_data *platform_data;
};

static int ar0234_read_reg(struct ar0234 *ar0234, u16 reg, u16 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar0234->sd);
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

static int ar0234_write_reg(struct ar0234 *ar0234, u16 reg, u16 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar0234->sd);
	u8 buf[6];

	if (reg == 0) {
		msleep(val);
		return 0;
	}

	if (len > 4)
		return -EINVAL;

	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val << 8 * (4 - len), buf + 2);
	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

static int ar0234_write_reg_list(struct ar0234 *ar0234,
				 const struct ar0234_reg_list *r_list)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar0234->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < r_list->num_of_regs; i++) {
		ret = ar0234_write_reg(ar0234, r_list->regs[i].address,
				AR0234_REG_VALUE_16BIT,
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

static int ar0234_update_digital_gain(struct ar0234 *ar0234, u32 d_gain)
{
	return ar0234_write_reg(ar0234, AR0234_REG_GLOBAL_GAIN,
				AR0234_REG_VALUE_16BIT, d_gain);
}

static u64 get_pixel_rate(struct ar0234 *ar0234)
{
	u64 pixel_rate;

	if (ar0234->cur_mode->lanes == 4)
		pixel_rate = FSERIAL_CLK_4_LANE;
	else if (ar0234->cur_mode->lanes == 2)
		pixel_rate = FSERIAL_CLK_2_LANE;
	else
		pixel_rate = FSERIAL_CLK_4_LANE;

	return pixel_rate;
}

/*
 * from table 1, AND9820-D.pdf.
 * for context A, hblank = LLP(0x300C) - active data time.
 */
static u64 get_hblank(struct ar0234 *ar0234)
{
	u64 hblank;
	u64 pixel_rate;
	u64 pixel_clk;

	if (ar0234->cur_mode->lanes == 4) {
		pixel_rate = FSERIAL_CLK_4_LANE;
		pixel_clk = PIX_CLK_4_LANE;
	} else if (ar0234->cur_mode->lanes == 2) {
		pixel_rate = FSERIAL_CLK_2_LANE;
		pixel_clk = PIX_CLK_2_LANE;
	} else {
		pixel_rate = FSERIAL_CLK_4_LANE;
		pixel_clk = PIX_CLK_4_LANE;
	}

	/*
	 * for pixel clock is ar0234 internal,
	 * return hblank in the numbers of pixel rate.
	 */
	hblank = 0x384 * (pixel_rate / pixel_clk);

	return hblank;
}

static int ar0234_set_stream(struct v4l2_subdev *sd, int enable);

static int ar0234_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ar0234 *ar0234 = container_of(ctrl->handler,
					     struct ar0234, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&ar0234->sd);
	s64 exposure_max;
	int ret = 0;
	u32 val;

	if (ctrl->id == V4L2_CID_IPU_SET_SUB_STREAM) {
		val = (*ctrl->p_new.p_s64 & 0xFFFF);
		dev_info(&client->dev, "V4L2_CID_IPU_SET_SUB_STREAM %x\n", val);
		mutex_unlock(&ar0234->mutex);
		ret = ar0234_set_stream(&ar0234->sd, val & 0x00FF);
		mutex_lock(&ar0234->mutex);
		return ret;
	}

	/* Propagate change of current control to all related controls */
	if (ctrl->id == V4L2_CID_VBLANK) {
		/* Update max exposure while meeting expected vblanking */
		exposure_max = ar0234->cur_mode->height + ctrl->val -
			       AR0234_EXPOSURE_MAX_MARGIN;
		__v4l2_ctrl_modify_range(ar0234->exposure,
					 ar0234->exposure->minimum,
					 exposure_max, ar0234->exposure->step,
					 ar0234->cur_mode->height -
					 AR0234_EXPOSURE_MAX_MARGIN);
	}

	/* V4L2 controls values will be applied only when power is already up */
	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = ar0234_write_reg(ar0234, AR0234_REG_ANALOG_GAIN,
				       AR0234_REG_VALUE_16BIT, ctrl->val);
		break;

	case V4L2_CID_DIGITAL_GAIN:
		ret = ar0234_update_digital_gain(ar0234, ctrl->val);
		break;

	case V4L2_CID_EXPOSURE:
		/* 4 least significant bits of expsoure are fractional part */
		ret = ar0234_write_reg(ar0234, AR0234_REG_EXPOSURE,
				       AR0234_REG_VALUE_16BIT, ctrl->val);
		break;

	case V4L2_CID_VBLANK:
		ret = ar0234_write_reg(ar0234, AR0234_REG_VTS,
				       AR0234_REG_VALUE_16BIT,
				       ar0234->cur_mode->height + ctrl->val);
		dev_dbg(&client->dev, "set vblank %d\n", ar0234->cur_mode->height + ctrl->val);
		break;
	case V4L2_CID_FLASH_STROBE_SOURCE:
		dev_dbg(&client->dev, "set led flash source %d\n", ctrl->val);
		break;

	case V4L2_CID_FLASH_STROBE:
		if (ar0234->platform_data->gpios[0] != -1) {
			if (ar0234->strobe_source->val ==
					V4L2_FLASH_STROBE_SOURCE_SOFTWARE)
				gpio_set_value(ar0234->platform_data->gpios[0], 1);
		}
		dev_info(&client->dev, "turn on led %d\n", ctrl->val);

		break;

	case V4L2_CID_FLASH_STROBE_STOP:
		if (ar0234->platform_data->gpios[0] != -1) {
			if (ar0234->strobe_source->val ==
					V4L2_FLASH_STROBE_SOURCE_SOFTWARE)
				gpio_set_value(ar0234->platform_data->gpios[0], 0);
		}
		dev_info(&client->dev, "turn off led %d\n", ctrl->val);
		break;

	case V4L2_CID_FLASH_TIMEOUT:
		ret = ar0234_read_reg(ar0234, AR0234_REG_LED_FLASH_CONTROL,
				AR0234_REG_VALUE_16BIT, &val);

		ret = ar0234_write_reg(ar0234, AR0234_REG_LED_FLASH_CONTROL,
				AR0234_REG_VALUE_16BIT,
				(AR0234_LED_DELAY & ctrl->val) | ((~AR0234_LED_DELAY) & val));
		dev_info(&client->dev, "set led delay %d\n",
				AR0234_LED_DELAY & ctrl->val);
		break;
	case V4L2_CID_VFLIP:
		ret = ar0234_read_reg(ar0234, AR0234_REG_IMAGE_ORIENTATION,
				AR0234_REG_VALUE_08BIT, &val);

		val &= ~(0x1 << AR0234_VFLIP_BIT);
		val |= ctrl->val << AR0234_VFLIP_BIT;
		ret = ar0234_write_reg(ar0234, AR0234_REG_IMAGE_ORIENTATION,
				AR0234_REG_VALUE_08BIT,
				val);
		dev_info(&client->dev, "set vflip %d\n", ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = ar0234_read_reg(ar0234, AR0234_REG_IMAGE_ORIENTATION,
				AR0234_REG_VALUE_08BIT, &val);

		val &= ~(0x1 << AR0234_HFLIP_BIT);
		val |= ctrl->val << AR0234_HFLIP_BIT;
		ret = ar0234_write_reg(ar0234, AR0234_REG_IMAGE_ORIENTATION,
				AR0234_REG_VALUE_08BIT,
				val);
		dev_info(&client->dev, "set hflip %d\n", ctrl->val);
		break;
	case V4L2_CID_IPU_QUERY_SUB_STREAM:
		dev_dbg(&client->dev, "query stream\n");
		break;
	default:
		dev_err(&client->dev, "unexpected ctrl id 0x%08x\n", ctrl->id);
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ar0234_ctrl_ops = {
	.s_ctrl = ar0234_set_ctrl,
};

static struct v4l2_ctrl_config ar0234_csi_port = {
	.ops	= &ar0234_ctrl_ops,
	.id	= AR0234_CID_CSI_PORT,
	.type	= V4L2_CTRL_TYPE_INTEGER,
	.name	= "CSI port",
	.min	= 1,
	.max	= 5,
	.def	= 1,
	.step	= 1,
	.flags	= V4L2_CTRL_FLAG_READ_ONLY,
};

static struct v4l2_ctrl_config ar0234_i2c_bus = {
	.ops	= &ar0234_ctrl_ops,
	.id	= AR0234_CID_I2C_BUS,
	.type	= V4L2_CTRL_TYPE_INTEGER,
	.name	= "I2C bus",
	.min	= 0,
	.max	= MINORMASK,
	.def	= 0,
	.step	= 1,
	.flags	= V4L2_CTRL_FLAG_READ_ONLY,
};

static struct v4l2_ctrl_config ar0234_i2c_id = {
	.ops	= &ar0234_ctrl_ops,
	.id	= AR0234_CID_I2C_ID,
	.type	= V4L2_CTRL_TYPE_INTEGER,
	.name	= "I2C id",
	.min	= 0x10,
	.max	= 0x77,
	.def	= 0x10,
	.step	= 1,
	.flags	= V4L2_CTRL_FLAG_READ_ONLY,
};

static struct v4l2_ctrl_config ar0234_i2c_slave_address = {
	.ops	= &ar0234_ctrl_ops,
	.id	= AR0234_CID_I2C_SLAVE_ADDRESS,
	.type	= V4L2_CTRL_TYPE_INTEGER,
	.name	= "I2C slave address",
	.min	= 0x0,
	.max	= 0x7f,
	.def	= 0x10,
	.step	= 1,
	.flags	= V4L2_CTRL_FLAG_READ_ONLY,
};

static struct v4l2_ctrl_config ar0234_fps = {
	.ops	= &ar0234_ctrl_ops,
	.id	= AR0234_CID_FPS,
	.type	= V4L2_CTRL_TYPE_INTEGER,
	.name	= "fps",
	.min	= 10,
	.max	= 120,
	.def	= 30,
	.step	= 1,
	.flags	= V4L2_CTRL_FLAG_READ_ONLY,
};

static struct v4l2_ctrl_config ar0234_frame_interval = {
	.ops	= &ar0234_ctrl_ops,
	.id	= AR0234_CID_FRAME_INTERVAL,
	.type	= V4L2_CTRL_TYPE_INTEGER,
	.name	= "frame interval",
	.min	= 0,
	.max	= 1000,
	.def	= 25,
	.step	= 1,
	.flags	= V4L2_CTRL_FLAG_READ_ONLY,
};

static struct v4l2_ctrl_config ar0234_q_sub_stream = {
	.ops = &ar0234_ctrl_ops,
	.id = V4L2_CID_IPU_QUERY_SUB_STREAM,
	.name = "query virtual channel",
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.max = 1,
	.min = 0,
	.def = 0,
	.menu_skip_mask = 0,
	.qmenu_int = NULL,
};

static const struct v4l2_ctrl_config ar0234_s_sub_stream = {
	.ops = &ar0234_ctrl_ops,
	.id = V4L2_CID_IPU_SET_SUB_STREAM,
	.name = "set virtual channel",
	.type = V4L2_CTRL_TYPE_INTEGER64,
	.max = 0xFFFF,
	.min = 0,
	.def = 0,
	.step = 1,
};

#define MIPI_CSI2_TYPE_RAW8    0x2a
#define MIPI_CSI2_TYPE_RAW10   0x2b

static unsigned int mbus_code_to_mipi(u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_SGRBG10_1X10:
		return MIPI_CSI2_TYPE_RAW10;
	case MEDIA_BUS_FMT_SGRBG8_1X8:
		return MIPI_CSI2_TYPE_RAW8;
	default:
		WARN_ON(1);
		return -EINVAL;
	}
}

static void set_sub_stream_fmt(s64 *sub_stream, u32 code)
{
       *sub_stream &= 0xFFFFFFFFFFFF0000;
       *sub_stream |= code;
}

static void set_sub_stream_h(s64 *sub_stream, u32 height)
{
       s64 val = height;
       val &= 0xFFFF;
       *sub_stream &= 0xFFFFFFFF0000FFFF;
       *sub_stream |= val << 16;
}

static void set_sub_stream_w(s64 *sub_stream, u32 width)
{
       s64 val = width;
       val &= 0xFFFF;
       *sub_stream &= 0xFFFF0000FFFFFFFF;
       *sub_stream |= val << 32;
}

static void set_sub_stream_dt(s64 *sub_stream, u32 dt)
{
       s64 val = dt;
       val &= 0xFF;
       *sub_stream &= 0xFF00FFFFFFFFFFFF;
       *sub_stream |= val << 48;
}

static void set_sub_stream_vc_id(s64 *sub_stream, u32 vc_id)
{
       s64 val = vc_id;
       val &= 0xFF;
       *sub_stream &= 0x00FFFFFFFFFFFFFF;
       *sub_stream |= val << 56;
}

static int ar0234_init_controls(struct ar0234 *ar0234)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar0234->sd);
	struct v4l2_ctrl_handler *ctrl_hdlr;
	s64 exposure_max;
	s64 hblank;
	struct v4l2_ctrl_config cfg = { 0 };
	int ret;

	ctrl_hdlr = &ar0234->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 8);
	if (ret)
		return ret;

	ctrl_hdlr->lock = &ar0234->mutex;
	ar0234->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr, &ar0234_ctrl_ops,
					   V4L2_CID_LINK_FREQ,
					   ARRAY_SIZE(link_freq_menu_items) - 1,
					   0, link_freq_menu_items);
	if (ar0234->link_freq)
		ar0234->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ar0234->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &ar0234_ctrl_ops,
			  V4L2_CID_VBLANK,
			  0,
			  AR0234_VTS_MAX - ar0234->cur_mode->height, 1,
			  ar0234->cur_mode->vts_def - ar0234->cur_mode->height);
	v4l2_ctrl_new_std(ctrl_hdlr, &ar0234_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  AR0234_ANAL_GAIN_MIN, AR0234_ANAL_GAIN_MAX,
			  AR0234_ANAL_GAIN_STEP, AR0234_ANAL_GAIN_DEFAULT);
	v4l2_ctrl_new_std(ctrl_hdlr, &ar0234_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  AR0234_DGTL_GAIN_MIN, AR0234_DGTL_GAIN_MAX,
			  AR0234_DGTL_GAIN_STEP, AR0234_DGTL_GAIN_DEFAULT);
	exposure_max = ar0234->cur_mode->vts_def - AR0234_EXPOSURE_MAX_MARGIN;
	ar0234->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &ar0234_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     AR0234_EXPOSURE_MIN, exposure_max,
					     AR0234_EXPOSURE_STEP,
					     exposure_max);
	ar0234->strobe_source = v4l2_ctrl_new_std_menu(
			ctrl_hdlr, &ar0234_ctrl_ops,
			V4L2_CID_FLASH_STROBE_SOURCE,
			1, 0, 1);
	ar0234->strobe = v4l2_ctrl_new_std(ctrl_hdlr, &ar0234_ctrl_ops,
			V4L2_CID_FLASH_STROBE, 0, 0, 0, 0);
	ar0234->strobe_stop = v4l2_ctrl_new_std(ctrl_hdlr, &ar0234_ctrl_ops,
			V4L2_CID_FLASH_STROBE_STOP, 0, 0, 0, 0);
	ar0234->timeout = v4l2_ctrl_new_std(ctrl_hdlr, &ar0234_ctrl_ops,
			V4L2_CID_FLASH_TIMEOUT, -128, 127, 1, 0);

	ar0234_csi_port.def = ar0234->platform_data->port;
	ar0234->csi_port =
		v4l2_ctrl_new_custom(ctrl_hdlr, &ar0234_csi_port, NULL);
	ar0234_i2c_bus.def = i2c_adapter_id(client->adapter);
	ar0234->i2c_bus =
		v4l2_ctrl_new_custom(ctrl_hdlr, &ar0234_i2c_bus, NULL);
	ar0234_i2c_id.def = client->addr;
	ar0234->i2c_id = v4l2_ctrl_new_custom(ctrl_hdlr, &ar0234_i2c_id, NULL);
	ar0234_i2c_slave_address.def = ar0234->platform_data->i2c_slave_address;
	ar0234->i2c_slave_address = v4l2_ctrl_new_custom(ctrl_hdlr, &ar0234_i2c_slave_address, NULL);
	ar0234_fps.def = ar0234->cur_mode->fps;
	ar0234->fps = v4l2_ctrl_new_custom(ctrl_hdlr, &ar0234_fps, NULL);
	ar0234_frame_interval.def = 1000 / ar0234->cur_mode->fps;
	ar0234->frame_interval = v4l2_ctrl_new_custom(ctrl_hdlr, &ar0234_frame_interval, NULL);

	ar0234->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &ar0234_ctrl_ops,
			V4L2_CID_PIXEL_RATE, get_pixel_rate(ar0234), get_pixel_rate(ar0234),
			1, get_pixel_rate(ar0234));
	if (ar0234->pixel_rate)
		ar0234->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	hblank = get_hblank(ar0234);
	ar0234->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &ar0234_ctrl_ops, V4L2_CID_HBLANK,
					hblank, hblank, 1, hblank);
	if (ar0234->hblank)
		ar0234->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ar0234->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &ar0234_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 1);
	ar0234->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &ar0234_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);

	ar0234_q_sub_stream.qmenu_int = devm_kzalloc(&client->dev, sizeof(s64), GFP_KERNEL);
	if (!ar0234_q_sub_stream.qmenu_int) {
		dev_dbg(&client->dev, "failed alloc mem for query sub stream.\n");
		return -ENOMEM;
	}
	ar0234->query_sub_stream = v4l2_ctrl_new_custom(ctrl_hdlr, &ar0234_q_sub_stream, NULL);
	if (ctrl_hdlr->error) {
		dev_dbg(&client->dev, "new query sub stream ctrl, error = %d.\n",
			ctrl_hdlr->error);
		return ctrl_hdlr->error;
	}

	ar0234->set_sub_stream = v4l2_ctrl_new_custom(ctrl_hdlr, &ar0234_s_sub_stream, NULL);
	if (ctrl_hdlr->error) {
		dev_dbg(&client->dev, "new set sub stream ctrl, error = %d.\n",
			ctrl_hdlr->error);
		return ctrl_hdlr->error;
	}

	if (ctrl_hdlr->error)
		return ctrl_hdlr->error;

	ar0234->sd.ctrl_handler = ctrl_hdlr;

	return ret;
}

static void ar0234_update_pad_format(const struct ar0234_mode *mode,
				     struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = mode->code;
	fmt->field = V4L2_FIELD_NONE;
}

static int ar0234_start_streaming(struct ar0234 *ar0234)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar0234->sd);
	const struct ar0234_reg_list *reg_list;
	int link_freq_index, ret;

	reg_list = &ar0234->cur_mode->reg_list;
	ret = ar0234_write_reg_list(ar0234, reg_list);
	if (ret) {
		dev_err(&client->dev, "failed to set mode");
		return ret;
	}

	link_freq_index = ar0234->cur_mode->link_freq_index;
	if (link_freq_index >= 0) {
		reg_list = &link_freq_configs[link_freq_index].reg_list;
		ret = ar0234_write_reg_list(ar0234, reg_list);
		if (ret) {
			dev_err(&client->dev, "failed to set plls");
			return ret;
		}
	}

	ar0234->set_sub_stream->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	ret = __v4l2_ctrl_handler_setup(ar0234->sd.ctrl_handler);
	ar0234->set_sub_stream->flags &= ~V4L2_CTRL_FLAG_READ_ONLY;
	if (ret)
		return ret;

	ret = ar0234_write_reg(ar0234, AR0234_REG_MODE_SELECT,
			       AR0234_REG_VALUE_16BIT, AR0234_MODE_STREAMING);
	if (ret) {
		dev_err(&client->dev, "failed to set stream");
		return ret;
	}

	return 0;
}

static void ar0234_stop_streaming(struct ar0234 *ar0234)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar0234->sd);

	if (ar0234_write_reg(ar0234, AR0234_REG_MODE_SELECT,
			     AR0234_REG_VALUE_16BIT, AR0234_MODE_STANDBY))
		dev_err(&client->dev, "failed to set stream");
	/*
	 * turn off flash, clear possible noise.
	 */
	if (ar0234->platform_data->gpios[0] != -1)
		gpio_set_value(ar0234->platform_data->gpios[0], 0);
}

static int ar0234_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct ar0234 *ar0234 = to_ar0234(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	if (ar0234->streaming == enable)
		return 0;

	mutex_lock(&ar0234->mutex);
	if (enable) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			mutex_unlock(&ar0234->mutex);
			return ret;
		}

		ret = ar0234_start_streaming(ar0234);
		if (ret) {
			enable = 0;
			ar0234_stop_streaming(ar0234);
			pm_runtime_put(&client->dev);
		}
	} else {
		ar0234_stop_streaming(ar0234);
		pm_runtime_put(&client->dev);
	}

	ar0234->streaming = enable;
	mutex_unlock(&ar0234->mutex);

	return ret;
}

static int ar0234_g_frame_interval(struct v4l2_subdev *sd,
		struct v4l2_subdev_frame_interval *fival)
{
	struct ar0234 *ar0234 = to_ar0234(sd);

	fival->pad = 0;
	fival->interval.numerator = 1;
	fival->interval.denominator = ar0234->cur_mode->fps;

	return 0;
}

static int __maybe_unused ar0234_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0234 *ar0234 = to_ar0234(sd);

	mutex_lock(&ar0234->mutex);
	if (ar0234->streaming)
		ar0234_stop_streaming(ar0234);

	mutex_unlock(&ar0234->mutex);

	return 0;
}

static int __maybe_unused ar0234_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0234 *ar0234 = to_ar0234(sd);
	int ret;

	mutex_lock(&ar0234->mutex);
	if (ar0234->streaming) {
		ret = ar0234_start_streaming(ar0234);
		if (ret) {
			ar0234->streaming = false;
			ar0234_stop_streaming(ar0234);
			mutex_unlock(&ar0234->mutex);
			return ret;
		}
	}

	mutex_unlock(&ar0234->mutex);

	return 0;
}

static int ar0234_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *sd_state,
			     struct v4l2_subdev_format *fmt)
{
	struct ar0234 *ar0234 = to_ar0234(sd);
	struct i2c_client *client = v4l2_get_subdevdata(&ar0234->sd);
	const struct ar0234_mode *mode;
	s32 vblank_def;
	s64 hblank;
	int i;
	s64 *sub_stream;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		if (supported_modes[i].width != fmt->format.width
			|| supported_modes[i].height != fmt->format.height) {
			dev_dbg(&client->dev, "resolution doesn't match\n");
			continue;
		}
		if (supported_modes[i].code != fmt->format.code) {
			dev_dbg(&client->dev, "pixel format doesn't match\n");
			continue;
		}
		mode = &supported_modes[i];
		break;
	}

	if (i >= ARRAY_SIZE(supported_modes))
		mode = &supported_modes[0];

	mutex_lock(&ar0234->mutex);
	ar0234_update_pad_format(mode, &fmt->format);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		*v4l2_subdev_get_try_format(sd, sd_state, fmt->pad) = fmt->format;
	} else {
		ar0234->cur_mode = mode;
		__v4l2_ctrl_s_ctrl(ar0234->link_freq, mode->link_freq_index);
		__v4l2_ctrl_modify_range(ar0234->pixel_rate,
					get_pixel_rate(ar0234),
					get_pixel_rate(ar0234),
					1,
					get_pixel_rate(ar0234));

		hblank = get_hblank(ar0234);
		__v4l2_ctrl_modify_range(ar0234->hblank,
					hblank,
					hblank,
					1,
					hblank);

		/* Update limits and set FPS to default */
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(ar0234->vblank,
					 0,
					 AR0234_VTS_MAX - mode->height, 1,
					 vblank_def);
		__v4l2_ctrl_s_ctrl(ar0234->vblank, vblank_def);

		__v4l2_ctrl_s_ctrl(ar0234->fps, mode->fps);

		__v4l2_ctrl_s_ctrl(ar0234->frame_interval, 1000 / mode->fps);

		sub_stream = ar0234->query_sub_stream->qmenu_int;
		set_sub_stream_fmt(sub_stream, mode->code);
		set_sub_stream_h(sub_stream, mode->height);
		set_sub_stream_w(sub_stream, mode->width);
		set_sub_stream_dt(sub_stream, mbus_code_to_mipi(mode->code));
		set_sub_stream_vc_id(sub_stream, 0);
	}

	mutex_unlock(&ar0234->mutex);

	return 0;
}

static int ar0234_get_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *sd_state,
			     struct v4l2_subdev_format *fmt)
{
	struct ar0234 *ar0234 = to_ar0234(sd);

	mutex_lock(&ar0234->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt->format = *v4l2_subdev_get_try_format(&ar0234->sd,
							  sd_state, fmt->pad);
	else
		ar0234_update_pad_format(ar0234->cur_mode, &fmt->format);

	mutex_unlock(&ar0234->mutex);

	return 0;
}

static int ar0234_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(supported_formats))
		return -EINVAL;

	code->code = supported_formats[code->index];

	return 0;
}

static int ar0234_enum_frame_size(struct v4l2_subdev *sd,
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

static int ar0234_frame_rate[] = { 40, 20 };

static int ar0234_enum_frame_interval(struct v4l2_subdev *subdev,
		struct v4l2_subdev_state *sd_state,
		struct v4l2_subdev_frame_interval_enum *fie)
{
	int mode_size = ARRAY_SIZE(supported_modes);
	int i;

	if (fie->index >= ARRAY_SIZE(ar0234_frame_rate))
		return -EINVAL;

	for (i = 0; i < mode_size; i++)
		if (fie->code == supported_modes[i].code
			&& fie->width == supported_modes[i].width
			&& fie->height == supported_modes[i].height)
			break;

	if (i == mode_size)
		return -EINVAL;

	fie->interval.numerator = 1;
	fie->interval.denominator = ar0234_frame_rate[fie->index];

	return 0;
}

static int ar0234_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct ar0234 *ar0234 = to_ar0234(sd);

	mutex_lock(&ar0234->mutex);
	ar0234_update_pad_format(&supported_modes[0],
				 v4l2_subdev_get_try_format(sd, fh->state, 0));
	mutex_unlock(&ar0234->mutex);

	return 0;
}

static const struct v4l2_subdev_video_ops ar0234_video_ops = {
	.s_stream = ar0234_set_stream,
	.g_frame_interval = ar0234_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops ar0234_pad_ops = {
	.set_fmt = ar0234_set_format,
	.get_fmt = ar0234_get_format,
	.enum_mbus_code = ar0234_enum_mbus_code,
	.enum_frame_size = ar0234_enum_frame_size,
	.enum_frame_interval = ar0234_enum_frame_interval,
};

static const struct v4l2_subdev_ops ar0234_subdev_ops = {
	.video = &ar0234_video_ops,
	.pad = &ar0234_pad_ops,
};

static const struct media_entity_operations ar0234_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops ar0234_internal_ops = {
	.open = ar0234_open,
};

static int ar0234_identify_module(struct ar0234 *ar0234)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar0234->sd);
	int ret;
	u32 val;

	ret = ar0234_read_reg(ar0234, AR0234_REG_CHIP_ID,
			      AR0234_REG_VALUE_16BIT, &val);
	if (ret)
		return ret;

	if (val != AR0234_CHIP_ID) {
		dev_err(&client->dev, "chip id mismatch: %x!=%x",
			AR0234_CHIP_ID, val);
		return -ENXIO;
	}

	return 0;
}

static int ar0234_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0234 *ar0234 = to_ar0234(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	pm_runtime_disable(&client->dev);
	mutex_destroy(&ar0234->mutex);

	return 0;
}

irqreturn_t ar0234_threaded_irq_fn(int irq, void *dev_id)
{
	struct ar0234 *ar0234 = dev_id;

	if ((ar0234->platform_data->gpios[0] != -1) && (ar0234->platform_data->irq_pin != -1)) {
		mutex_lock(&ar0234->mutex);
		if (ar0234->streaming == false) {
			gpio_set_value(ar0234->platform_data->gpios[0], 0);
			goto ar0234_irq_handled;
		}

		if (ar0234->strobe_source->val == V4L2_FLASH_STROBE_SOURCE_EXTERNAL) {
			gpio_set_value(ar0234->platform_data->gpios[0],
					gpio_get_value(ar0234->platform_data->irq_pin));
		}

ar0234_irq_handled:
		mutex_unlock(&ar0234->mutex);
	}

	return IRQ_HANDLED;
}

static int ar0234_probe(struct i2c_client *client)
{
	struct ar0234 *ar0234;
	int ret;

	ar0234 = devm_kzalloc(&client->dev, sizeof(*ar0234), GFP_KERNEL);
	if (!ar0234)
		return -ENOMEM;

	ar0234->platform_data = client->dev.platform_data;
	if (ar0234->platform_data == NULL) {
		dev_err(&client->dev, "no platform data provided\n");
		return -EINVAL;
	}
	v4l2_i2c_subdev_init(&ar0234->sd, client, &ar0234_subdev_ops);
	ret = ar0234_identify_module(ar0234);
	if (ret) {
		dev_err(&client->dev, "failed to find sensor: %d", ret);
		return ret;
	}

	if (ar0234->platform_data->suffix)
		snprintf(ar0234->sd.name,
				sizeof(ar0234->sd.name), "ar0234 %c",
				ar0234->platform_data->suffix);

	mutex_init(&ar0234->mutex);
	ar0234->cur_mode = &supported_modes[0];
	ret = ar0234_init_controls(ar0234);
	if (ret) {
		dev_err(&client->dev, "failed to init controls: %d", ret);
		goto probe_error_v4l2_ctrl_handler_free;
	}

	ar0234->sd.internal_ops = &ar0234_internal_ops;
	ar0234->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ar0234->sd.entity.ops = &ar0234_subdev_entity_ops;
	ar0234->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ar0234->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&ar0234->sd.entity, 1, &ar0234->pad);
	if (ret) {
		dev_err(&client->dev, "failed to init entity pads: %d", ret);
		goto probe_error_v4l2_ctrl_handler_free;
	}

	if ((ar0234->platform_data->gpios[0] != -1) && (ar0234->platform_data->irq_pin != -1)) {
		ret = devm_gpio_request(&client->dev,
				ar0234->platform_data->irq_pin,
				ar0234->platform_data->irq_pin_name);
		if (ret) {
			dev_err(&client->dev, "IRQ pin request failed!\n");
			goto probe_error_v4l2_ctrl_handler_free;
		}
		gpio_direction_input(ar0234->platform_data->irq_pin);
		ret = devm_request_threaded_irq(&client->dev,
				gpio_to_irq(ar0234->platform_data->irq_pin),
				NULL, ar0234_threaded_irq_fn,
				ar0234->platform_data->irq_pin_flags,
				ar0234->platform_data->irq_pin_name,
				ar0234);
		if (ret) {
			dev_err(&client->dev, "IRQ request failed!\n");
			goto probe_error_v4l2_ctrl_handler_free;
		}

		ret = devm_gpio_request_one(&client->dev,
				ar0234->platform_data->gpios[0],
				GPIOF_OUT_INIT_LOW, "LED");
		if (ret) {
			dev_err(&client->dev, "LED GPIO pin request failed!\n");
			goto probe_error_v4l2_ctrl_handler_free;
		}
	}

	ret = v4l2_async_register_subdev_sensor(&ar0234->sd);
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
	media_entity_cleanup(&ar0234->sd.entity);

probe_error_v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(ar0234->sd.ctrl_handler);
	mutex_destroy(&ar0234->mutex);

	return ret;
}

static const struct dev_pm_ops ar0234_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ar0234_suspend, ar0234_resume)
};

static const struct i2c_device_id ar0234_id_table[] = {
	{ "ar0234", 0 },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(i2c, ar0234_id_table);

static struct i2c_driver ar0234_i2c_driver = {
	.driver = {
		.name = "ar0234",
		.pm = &ar0234_pm_ops,
	},
	.probe_new = ar0234_probe,
	.remove = ar0234_remove,
	.id_table = ar0234_id_table,
};

module_i2c_driver(ar0234_i2c_driver);

MODULE_AUTHOR("Chang, Ying <ying.chang@intel.com>");
MODULE_DESCRIPTION("ar0234 sensor driver");
MODULE_LICENSE("GPL v2");
