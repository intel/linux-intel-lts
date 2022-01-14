// SPDX-License-Identifier: GPL-2.0-only
/*
 * kmb-imx412.c - KeemBay Camera imx412 Sensor Driver.
 *
 * Copyright (C) 2020 Intel Corporation
 */

#include <asm/unaligned.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/kmb-isp-ctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define KMB_IMX412_DRV_NAME	"kmb-imx412-sensor"

/* Streaming Mode */
#define KMB_IMX412_REG_MODE_SELECT	0x0100
#define KMB_IMX412_MODE_STANDBY		0x00
#define KMB_IMX412_MODE_STREAMING	0x01

/* Lines per frame */
#define KMB_IMX412_REG_LPFR		0x0340
#define KMB_IMX412_LPFR_MAX		0xFFFF

/* Chip ID */
#define KMB_IMX412_REG_ID	0x0016
#define KMB_IMX412_ID		0x577

/* Exposure control */
#define KMB_IMX412_REG_EXPOSURE_FIT		0x0200
#define KMB_IMX412_FIT_DEFAULT			0x0790
#define KMB_IMX412_REG_EXPOSURE_CIT		0x0202
#define KMB_IMX412_EXPOSURE_MIN			8
#define KMB_IMX412_EXPOSURE_STEP		1
#define KMB_IMX412_EXPOSURE_DEFAULT		0x0648
#define KMB_IMX412_EXPOSURE_MIN_OFFSET_TO_LPFR	22

#define KMB_IMX412_REG_DOL_EXPOSURE_CIT_1ST_FRM			0x00EA
#define KMB_IMX412_DOL_EXPOSURE_CIT_1ST_FRM_MIN			8
#define KMB_IMX412_DOL_EXPOSURE_CIT_1ST_FRM_STEP		2
#define KMB_IMX412_DOL_EXPOSURE_CIT_1ST_FRM_DEFAULT		0x0648
#define KMB_IMX412_DOL_EXPOSURE_CIT_1ST_FRM_MIN_OFFSET		22

#define KMB_IMX412_REG_DOL_OFFSET_2ND_FRM			0x00E6
#define KMB_IMX412_REG_DOL_EXPOSURE_CIT_2ND_FRM			0x00EC
#define KMB_IMX412_DOL_EXPOSURE_CIT_2ND_FRM_MIN			8
#define KMB_IMX412_DOL_EXPOSURE_CIT_2ND_FRM_STEP		2
#define KMB_IMX412_DOL_EXPOSURE_CIT_2ND_FRM_DEFAULT		0x00E0
#define KMB_IMX412_DOL_EXPOSURE_CIT_2ND_FRM_MIN_OFFSET		22

#define KMB_IMX412_REG_DOL_OFFSET_3RD_FRM			0x00E8
#define KMB_IMX412_REG_DOL_EXPOSURE_CIT_3RD_FRM			0x00EE
#define KMB_IMX412_DOL_EXPOSURE_CIT_3RD_FRM_MIN			8
#define KMB_IMX412_DOL_EXPOSURE_CIT_3RD_FRM_STEP		2
#define KMB_IMX412_DOL_EXPOSURE_CIT_3RD_FRM_DEFAULT		0x000E
#define KMB_IMX412_DOL_EXPOSURE_CIT_3RD_FRM_MIN_OFFSET		22

/* Analog gain control */
#define KMB_IMX412_REG_AGAIN		0x0204
#define KMB_IMX412_AGAIN_MIN		0
#define KMB_IMX412_AGAIN_MAX		978
#define KMB_IMX412_AGAIN_STEP		1
#define KMB_IMX412_AGAIN_DEFAULT	0

#define KMB_IMX412_REG_DOL_ANA_GAIN_1ST_FRM		0x00F0
#define KMB_IMX412_DOL_ANA_GAIN_1ST_FRM_MIN		0
#define KMB_IMX412_DOL_ANA_GAIN_1ST_FRM_MAX		978
#define KMB_IMX412_DOL_ANA_GAIN_1ST_FRM_STEP		1
#define KMB_IMX412_DOL_ANA_GAIN_1ST_FRM_DEFAULT		0

#define KMB_IMX412_REG_DOL_ANA_GAIN_2ND_FRM		0x00F2
#define KMB_IMX412_REG_DOL_ANA_GAIN_2ND_FRM_MIN		0
#define KMB_IMX412_REG_DOL_ANA_GAIN_2ND_FRM_MAX		978
#define KMB_IMX412_REG_DOL_ANA_GAIN_2ND_FRM_STEP	1
#define KMB_IMX412_REG_DOL_ANA_GAIN_2ND_FRM_DEFAULT	0

#define KMB_IMX412_REG_DOL_ANA_GAIN_3RD_FRM		0x00F4
#define KMB_IMX412_REG_DOL_ANA_GAIN_3RD_FRM_MIN		0
#define KMB_IMX412_REG_DOL_ANA_GAIN_3RD_FRM_MAX		978
#define KMB_IMX412_REG_DOL_ANA_GAIN_3RD_FRM_STEP	1
#define KMB_IMX412_REG_DOL_ANA_GAIN_3RD_FRM_DEFAULT	0

/* Digital gain controls */
#define KMB_IMX412_REG_DGAIN		0x020E
#define KMB_IMX412_DGAIN_MIN		256
#define KMB_IMX412_DGAIN_MAX		4095
#define KMB_IMX412_DGAIN_STEP		1
#define KMB_IMX412_DGAIN_DEFAULT	256

#define KMB_IMX412_REG_DOL_DIG_GAIN_1ST_FRM		0x00F6
#define KMB_IMX412_REG_DOL_DIG_GAIN_1ST_FRM_MIN		256
#define KMB_IMX412_REG_DOL_DIG_GAIN_1ST_FRM_MAX		4095
#define KMB_IMX412_REG_DOL_DIG_GAIN_1ST_FRM_STEP	1
#define KMB_IMX412_REG_DOL_DIG_GAIN_1ST_FRM_DEFAULT	256

#define KMB_IMX412_REG_DOL_DIG_GAIN_2ND_FRM		0x00F8
#define KMB_IMX412_REG_DOL_DIG_GAIN_2ND_FRM_MIN		256
#define KMB_IMX412_REG_DOL_DIG_GAIN_2ND_FRM_MAX		4095
#define KMB_IMX412_REG_DOL_DIG_GAIN_2ND_FRM_STEP	1
#define KMB_IMX412_REG_DOL_DIG_GAIN_2ND_FRM_DEFAULT	256

#define KMB_IMX412_REG_DOL_DIG_GAIN_3RD_FRM		0x00FA
#define KMB_IMX412_REG_DOL_DIG_GAIN_3RD_FRM_MIN		256
#define KMB_IMX412_REG_DOL_DIG_GAIN_3RD_FRM_MAX		4095
#define KMB_IMX412_REG_DOL_DIG_GAIN_3RD_FRM_STEP	1
#define KMB_IMX412_REG_DOL_DIG_GAIN_3RD_FRM_DEFAULT	256

/* Group hold register */
#define KMB_IMX412_REG_HOLD	0x0104

/* Input clock rate */
#define KMB_IMX412_INCLK_RATE	24000000

#define KMB_IMX412_REG_MIN	0x00
#define KMB_IMX412_REG_MAX	0xFFFF

/* Link frequency */
#define KMB_IMX412_LINK_FREQ_2100MHz	2100000000
#define KMB_IMX412_LINK_FREQ_1200MHz	1200000000
#define KMB_IMX412_LINK_FREQ_1452MHz	1452000000
#define KMB_IMX412_LINK_FREQ_1260MHz	1260000000

#define KMB_IMX412_FPS_NUMERATOR	(1 << 8)
#define KMB_IMX412_2DOL_GET_1ST_MAX_EXP(x)	\
	(x - (2 * 22) - 2)

#define KMB_IMX412_3DOL_GET_1ST_MAX_EXP(x)	\
	(((x * 4) / 3) - (3 * 22) - 4)

#define KMB_IMX412_MAIN_I2C		0x10
#define KMB_IMX412_AUX_ADD_EN_2ND	0x3010
#define KMB_IMX412_AUX_ADD_ACK_2ND	0x3011

/**
 * struct kmb_imx412_reg - KMB imx412 Sensor register
 * @address: Register address
 * @val: Register value
 */
struct kmb_imx412_reg {
	u16 address;
	u8 val;
};

/**
 * struct kmb_imx412_reg_list - KMB imx412 Sensor register list
 * @num_of_regs: Number of registers in the list
 * @regs: Pointer to register list
 */
struct kmb_imx412_reg_list {
	u32 num_of_regs;
	const struct kmb_imx412_reg *regs;
};

/**
 * enum kmb_imx412_streaming_mode - Streaming mode
 *
 * @KMB_IMX412_ULL: Normal mode
 * @KMB_IMX412_2DOL_HDR: HDR mode 2 frames
 * @KMB_IMX412_3DOL_HDR: HDR mode 3 frames
 */
enum kmb_imx412_streaming_mode {
	KMB_IMX412_ULL = 0,
	KMB_IMX412_2DOL_HDR = 1,
	KMB_IMX412_3DOL_HDR = 2,
};

/**
 * struct kmb_imx412_mode - KMB imx412 Sensor mode structure
 * @width: Frame width
 * @height: Frame height
 * @code: Format code
 * @ppln: Pixels per line
 * @lpfr: Lines per frame
 * @skip_lines: Top lines to be skipped
 * @pclk: Sensor pixel clock
 * @row_time: Row time in ns
 * @num_lanes: Data lanes number
 * @link_freq_idx: Link frequency index
 * @def: Default frames per second
 * @min: Min frames per second
 * @max: Max frames per second
 * @step: Frame rate step
 * @reg_list: Register list for sensor mode
 */
struct kmb_imx412_mode {
	u32 width;
	u32 height;
	u32 code;
	u32 ppln;
	u32 lpfr[3];
	u32 skip_lines;
	u64 pclk;
	u32 row_time;
	u32 num_lanes;
	u32 link_freq_idx;
	struct {
		u32 def;
		u32 min;
		u32 max;
		u32 step;
	} fps;
	struct kmb_imx412_reg_list reg_list;
};

/**
 * struct kmb_imx412 - KMB imx412 Sensor device structure
 * @dev: pointer to generic device
 * @client: pointer to i2c client
 * @broadcast_client: pointer to i2c auxiliary client
 * @sd: V4L2 sub-device
 * @pad: Media pad. Only one pad supported
 * @reset_gpio: Sensor reset gpio
 * @inclk: Sensor input clock
 * @ctrl_handler: V4L2 control handler
 * @pclk_ctrl: Pointer to pixel clock control
 * @link_freq_ctrl: Pointer to link frequency control
 * @hblank_ctrl: Pointer to horizontal blanking control
 * @vblank_ctrl: Pointer to vertical blanking control
 * @row_time_ctrl: Pointer to row time control
 * @exp_ctrl: Pointer to exposure control
 * @again_ctrl: Pointer to analog gain control
 * @dgain_ctrl: Pointer to digital gain control
 * @exp_ctrl1: Pointer to short exposure control
 * @again_ctrl1: Pointer to short analog gain control
 * @dgain_ctrl1: Pointer to short digital gain control
 * @exp_ctrl2: Pointer to very short exposure control
 * @again_ctrl2: Pointer to very short analog gain control
 * @dgain_ctrl2: Pointer to very short digital gain control
 * @sync_mode_ctrl: Pointer to sync mode control
 * @sync_enable_ctrl: Pointer to hardware sync enable control
 * @sync_start_ctrl: Pointer to sync start control
 * @sync_type_ctrl: Pointer to sync type control
 * @num_lanes: Number of data lanes
 * @fps: FPS to be applied on next stream on
 * @lpfr: Lines per frame for long exposure frame
 * @cur_mode: Current selected sensor mode
 * @supported_modes: Pointer to supported sensor modes
 * @num_supported_modes: Number of supported sensor modes
 * @mutex: Mutex for serializing sensor controls
 * @streaming_mode: Current selected streaming mode
 * @streaming: Flag indicating streaming state
 * @sync_mode: Sensor sync mode
 * @sync_type: Sensor sync type - sw or hw
 */
struct kmb_imx412 {
	struct device *dev;
	struct i2c_client *client;
	struct i2c_client *broadcast_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct gpio_desc *reset_gpio;
	struct clk *inclk;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pclk_ctrl;
	struct v4l2_ctrl *link_freq_ctrl;
	struct v4l2_ctrl *hblank_ctrl;
	struct v4l2_ctrl *vblank_ctrl;
	struct v4l2_ctrl *row_time_ctrl;
	struct {
		struct v4l2_ctrl *exp_ctrl;
		struct v4l2_ctrl *again_ctrl;
		struct v4l2_ctrl *dgain_ctrl;
		struct v4l2_ctrl *exp_ctrl1;
		struct v4l2_ctrl *again_ctrl1;
		struct v4l2_ctrl *dgain_ctrl1;
		struct v4l2_ctrl *exp_ctrl2;
		struct v4l2_ctrl *again_ctrl2;
		struct v4l2_ctrl *dgain_ctrl2;
	};
	struct v4l2_ctrl *sync_mode_ctrl;
	struct v4l2_ctrl *sync_start_ctrl;
	struct v4l2_ctrl *sync_type_ctrl;
	u32 num_lanes;
	u32 fps;
	u32 lpfr;
	const struct kmb_imx412_mode *cur_mode;
	const struct kmb_imx412_mode *supported_modes;
	u32 num_supported_modes;
	struct mutex mutex;
	enum kmb_imx412_streaming_mode streaming_mode;
	bool streaming;
	enum kmb_sync_mode sync_mode;
	enum kmb_sync_type sync_type;
};

static const s64 link_freq[] = {
	KMB_IMX412_LINK_FREQ_2100MHz,
	KMB_IMX412_LINK_FREQ_1200MHz,
	KMB_IMX412_LINK_FREQ_1452MHz,
	KMB_IMX412_LINK_FREQ_1260MHz
};

/* Sensor mode registers */
static const struct kmb_imx412_reg mode_4056x3040_regs[] = {
	{0x0136, 0x18},
	{0x0137, 0x00},
	{0x3C7E, 0x08},
	{0x3C7F, 0x02},
	{0x38A8, 0x1F},
	{0x38A9, 0xFF},
	{0x38AA, 0x1F},
	{0x38AB, 0xFF},
	{0x55D4, 0x00},
	{0x55D5, 0x00},
	{0x55D6, 0x07},
	{0x55D7, 0xFF},
	{0x55E8, 0x07},
	{0x55E9, 0xFF},
	{0x55EA, 0x00},
	{0x55EB, 0x00},
	{0x575C, 0x07},
	{0x575D, 0xFF},
	{0x575E, 0x00},
	{0x575F, 0x00},
	{0x5764, 0x00},
	{0x5765, 0x00},
	{0x5766, 0x07},
	{0x5767, 0xFF},
	{0x5974, 0x04},
	{0x5975, 0x01},
	{0x5F10, 0x09},
	{0x5F11, 0x92},
	{0x5F12, 0x32},
	{0x5F13, 0x72},
	{0x5F14, 0x16},
	{0x5F15, 0xBA},
	{0x5F17, 0x13},
	{0x5F18, 0x24},
	{0x5F19, 0x60},
	{0x5F1A, 0xE3},
	{0x5F1B, 0xAD},
	{0x5F1C, 0x74},
	{0x5F2D, 0x25},
	{0x5F5C, 0xD0},
	{0x6A22, 0x00},
	{0x6A23, 0x1D},
	{0x7BA8, 0x00},
	{0x7BA9, 0x00},
	{0x886B, 0x00},
	{0x9002, 0x0A},
	{0x9004, 0x1A},
	{0x9214, 0x93},
	{0x9215, 0x69},
	{0x9216, 0x93},
	{0x9217, 0x6B},
	{0x9218, 0x93},
	{0x9219, 0x6D},
	{0x921A, 0x57},
	{0x921B, 0x58},
	{0x921C, 0x57},
	{0x921D, 0x59},
	{0x921E, 0x57},
	{0x921F, 0x5A},
	{0x9220, 0x57},
	{0x9221, 0x5B},
	{0x9222, 0x93},
	{0x9223, 0x02},
	{0x9224, 0x93},
	{0x9225, 0x03},
	{0x9226, 0x93},
	{0x9227, 0x04},
	{0x9228, 0x93},
	{0x9229, 0x05},
	{0x922A, 0x98},
	{0x922B, 0x21},
	{0x922C, 0xB2},
	{0x922D, 0xDB},
	{0x922E, 0xB2},
	{0x922F, 0xDC},
	{0x9230, 0xB2},
	{0x9231, 0xDD},
	{0x9232, 0xB2},
	{0x9233, 0xE1},
	{0x9234, 0xB2},
	{0x9235, 0xE2},
	{0x9236, 0xB2},
	{0x9237, 0xE3},
	{0x9238, 0xB7},
	{0x9239, 0xB9},
	{0x923A, 0xB7},
	{0x923B, 0xBB},
	{0x923C, 0xB7},
	{0x923D, 0xBC},
	{0x923E, 0xB7},
	{0x923F, 0xC5},
	{0x9240, 0xB7},
	{0x9241, 0xC7},
	{0x9242, 0xB7},
	{0x9243, 0xC9},
	{0x9244, 0x98},
	{0x9245, 0x56},
	{0x9246, 0x98},
	{0x9247, 0x55},
	{0x9380, 0x00},
	{0x9381, 0x62},
	{0x9382, 0x00},
	{0x9383, 0x56},
	{0x9384, 0x00},
	{0x9385, 0x52},
	{0x9388, 0x00},
	{0x9389, 0x55},
	{0x938A, 0x00},
	{0x938B, 0x55},
	{0x938C, 0x00},
	{0x938D, 0x41},
	{0x5078, 0x01},
	{0x0112, 0x0A},
	{0x0113, 0x0A},
	{0x0114, 0x03},
	{0x0342, 0x11},
	{0x0343, 0xA0},
	{0x0340, 0x0D},
	{0x0341, 0xDA},
	{0x3210, 0x00},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x0F},
	{0x0349, 0xD7},
	{0x034A, 0x0B},
	{0x034B, 0xDF},
	{0x00E3, 0x00},
	{0x00E4, 0x00},
	{0x00E5, 0x01},
	{0x00FC, 0x0A},
	{0x00FD, 0x0A},
	{0x00FE, 0x0A},
	{0x00FF, 0x0A},
	{0xE013, 0x00},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x00},
	{0x0901, 0x11},
	{0x0902, 0x00},
	{0x3140, 0x02},
	{0x3241, 0x11},
	{0x3250, 0x03},
	{0x3E10, 0x00},
	{0x3E11, 0x00},
	{0x3F0D, 0x00},
	{0x3F42, 0x00},
	{0x3F43, 0x00},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x10},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040A, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x0F},
	{0x040D, 0xD8},
	{0x040E, 0x0B},
	{0x040F, 0xE0},
	{0x034C, 0x0F},
	{0x034D, 0xD8},
	{0x034E, 0x0B},
	{0x034F, 0xE0},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x04},
	{0x0306, 0x00},
	{0x0307, 0xC8},
	{0x0309, 0x0A},
	{0x030B, 0x01},
	{0x030D, 0x02},
	{0x030E, 0x01},
	{0x030F, 0x5E},
	{0x0310, 0x00},
	{0x0820, 0x12},
	{0x0821, 0xC0},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x3E20, 0x01},
	{0x3E37, 0x00},
	{0x3F50, 0x00},
	{0x3F56, 0x00},
	{0x3F57, 0xE2},
	{0x3C0A, 0x5A},
	{0x3C0B, 0x55},
	{0x3C0C, 0x28},
	{0x3C0D, 0x07},
	{0x3C0E, 0xFF},
	{0x3C0F, 0x00},
	{0x3C10, 0x00},
	{0x3C11, 0x02},
	{0x3C12, 0x00},
	{0x3C13, 0x03},
	{0x3C14, 0x00},
	{0x3C15, 0x00},
	{0x3C16, 0x0C},
	{0x3C17, 0x0C},
	{0x3C18, 0x0C},
	{0x3C19, 0x0A},
	{0x3C1A, 0x0A},
	{0x3C1B, 0x0A},
	{0x3C1C, 0x00},
	{0x3C1D, 0x00},
	{0x3C1E, 0x00},
	{0x3C1F, 0x00},
	{0x3C20, 0x00},
	{0x3C21, 0x00},
	{0x3C22, 0x3F},
	{0x3C23, 0x0A},
	{0x3E35, 0x01},
	{0x3F4A, 0x03},
	{0x3F4B, 0xBF},
	{0x3F26, 0x00},
	{0x0202, 0x0D},
	{0x0203, 0xC4},
	{0x0204, 0x00},
	{0x0205, 0x00},
	{0x020E, 0x01},
	{0x020F, 0x00},
	{0x0210, 0x01},
	{0x0211, 0x00},
	{0x0212, 0x01},
	{0x0213, 0x00},
	{0x0214, 0x01},
	{0x0215, 0x00},
	{0xBCF1, 0x00},
};

static const struct kmb_imx412_reg mode_4056x3040_2L_regs[] = {
	{0x0136, 0x18},
	{0x0137, 0x00},
	{0x3C7E, 0x04},
	{0x3C7F, 0x02},
	{0x38A8, 0x1F},
	{0x38A9, 0xFF},
	{0x38AA, 0x1F},
	{0x38AB, 0xFF},
	{0x55D4, 0x00},
	{0x55D5, 0x00},
	{0x55D6, 0x07},
	{0x55D7, 0xFF},
	{0x55E8, 0x07},
	{0x55E9, 0xFF},
	{0x55EA, 0x00},
	{0x55EB, 0x00},
	{0x575C, 0x07},
	{0x575D, 0xFF},
	{0x575E, 0x00},
	{0x575F, 0x00},
	{0x5764, 0x00},
	{0x5765, 0x00},
	{0x5766, 0x07},
	{0x5767, 0xFF},
	{0x5974, 0x04},
	{0x5975, 0x01},
	{0x5F10, 0x09},
	{0x5F11, 0x92},
	{0x5F12, 0x32},
	{0x5F13, 0x72},
	{0x5F14, 0x16},
	{0x5F15, 0xBA},
	{0x5F17, 0x13},
	{0x5F18, 0x24},
	{0x5F19, 0x60},
	{0x5F1A, 0xE3},
	{0x5F1B, 0xAD},
	{0x5F1C, 0x74},
	{0x5F2D, 0x25},
	{0x5F5C, 0xD0},
	{0x6A22, 0x00},
	{0x6A23, 0x1D},
	{0x7BA8, 0x00},
	{0x7BA9, 0x00},
	{0x886B, 0x00},
	{0x9002, 0x0A},
	{0x9004, 0x1A},
	{0x9214, 0x93},
	{0x9215, 0x69},
	{0x9216, 0x93},
	{0x9217, 0x6B},
	{0x9218, 0x93},
	{0x9219, 0x6D},
	{0x921A, 0x57},
	{0x921B, 0x58},
	{0x921C, 0x57},
	{0x921D, 0x59},
	{0x921E, 0x57},
	{0x921F, 0x5A},
	{0x9220, 0x57},
	{0x9221, 0x5B},
	{0x9222, 0x93},
	{0x9223, 0x02},
	{0x9224, 0x93},
	{0x9225, 0x03},
	{0x9226, 0x93},
	{0x9227, 0x04},
	{0x9228, 0x93},
	{0x9229, 0x05},
	{0x922A, 0x98},
	{0x922B, 0x21},
	{0x922C, 0xB2},
	{0x922D, 0xDB},
	{0x922E, 0xB2},
	{0x922F, 0xDC},
	{0x9230, 0xB2},
	{0x9231, 0xDD},
	{0x9232, 0xB2},
	{0x9233, 0xE1},
	{0x9234, 0xB2},
	{0x9235, 0xE2},
	{0x9236, 0xB2},
	{0x9237, 0xE3},
	{0x9238, 0xB7},
	{0x9239, 0xB9},
	{0x923A, 0xB7},
	{0x923B, 0xBB},
	{0x923C, 0xB7},
	{0x923D, 0xBC},
	{0x923E, 0xB7},
	{0x923F, 0xC5},
	{0x9240, 0xB7},
	{0x9241, 0xC7},
	{0x9242, 0xB7},
	{0x9243, 0xC9},
	{0x9244, 0x98},
	{0x9245, 0x56},
	{0x9246, 0x98},
	{0x9247, 0x55},
	{0x9380, 0x00},
	{0x9381, 0x62},
	{0x9382, 0x00},
	{0x9383, 0x56},
	{0x9384, 0x00},
	{0x9385, 0x52},
	{0x9388, 0x00},
	{0x9389, 0x55},
	{0x938A, 0x00},
	{0x938B, 0x55},
	{0x938C, 0x00},
	{0x938D, 0x41},
	{0x5078, 0x01},
	{0x9827, 0x20},
	{0x9830, 0x0A},
	{0x9833, 0x0A},
	{0x9834, 0x32},
	{0x9837, 0x22},
	{0x983C, 0x04},
	{0x983F, 0x0A},
	{0x994F, 0x00},
	{0x9A48, 0x06},
	{0x9A49, 0x06},
	{0x9A4A, 0x06},
	{0x9A4B, 0x06},
	{0x9A4E, 0x03},
	{0x9A4F, 0x03},
	{0x9A54, 0x03},
	{0x9A66, 0x03},
	{0x9A67, 0x03},
	{0xA2C9, 0x02},
	{0xA2CB, 0x02},
	{0xA2CD, 0x02},
	{0xB249, 0x3F},
	{0xB24F, 0x3F},
	{0xB290, 0x3F},
	{0xB293, 0x3F},
	{0xB296, 0x3F},
	{0xB299, 0x3F},
	{0xB2A2, 0x3F},
	{0xB2A8, 0x3F},
	{0xB2A9, 0x0D},
	{0xB2AA, 0x0D},
	{0xB2AB, 0x3F},
	{0xB2BA, 0x2F},
	{0xB2BB, 0x2F},
	{0xB2BC, 0x2F},
	{0xB2BD, 0x10},
	{0xB2C0, 0x3F},
	{0xB2C3, 0x3F},
	{0xB2D2, 0x3F},
	{0xB2DE, 0x20},
	{0xB2DF, 0x20},
	{0xB2E0, 0x20},
	{0xB2EA, 0x3F},
	{0xB2ED, 0x3F},
	{0xB2EE, 0x3F},
	{0xB2EF, 0x3F},
	{0xB2F0, 0x2F},
	{0xB2F1, 0x2F},
	{0xB2F2, 0x2F},
	{0xB2F9, 0x0E},
	{0xB2FA, 0x0E},
	{0xB2FB, 0x0E},
	{0xB759, 0x01},
	{0xB765, 0x3F},
	{0xB76B, 0x3F},
	{0xB7B3, 0x03},
	{0xB7B5, 0x03},
	{0xB7B7, 0x03},
	{0xB7BF, 0x03},
	{0xB7C1, 0x03},
	{0xB7C3, 0x03},
	{0xB7EF, 0x02},
	{0xB7F5, 0x1F},
	{0xB7F7, 0x1F},
	{0xB7F9, 0x1F},
	{0x0112, 0x0A},
	{0x0113, 0x0A},
	{0x0114, 0x01},
	{0x0342, 0x23},
	{0x0343, 0x40},
	{0x0340, 0x0C},
	{0x0341, 0x1E},
	{0x3210, 0x00},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x0F},
	{0x0349, 0xD7},
	{0x034A, 0x0B},
	{0x034B, 0xDF},
	{0x00E3, 0x00},
	{0x00E4, 0x00},
	{0x00E5, 0x01},
	{0x00FC, 0x0A},
	{0x00FD, 0x0A},
	{0x00FE, 0x0A},
	{0x00FF, 0x0A},
	{0xE013, 0x00},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x00},
	{0x0901, 0x11},
	{0x0902, 0x00},
	{0x3140, 0x02},
	{0x3241, 0x11},
	{0x3250, 0x03},
	{0x3E10, 0x00},
	{0x3E11, 0x00},
	{0x3F0D, 0x00},
	{0x3F42, 0x00},
	{0x3F43, 0x00},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x10},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040A, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x0F},
	{0x040D, 0xD8},
	{0x040E, 0x0B},
	{0x040F, 0xE0},
	{0x034C, 0x0F},
	{0x034D, 0xD8},
	{0x034E, 0x0B},
	{0x034F, 0xE0},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x04},
	{0x0306, 0x01},
	{0x0307, 0x5E},
	{0x0309, 0x0A},
	{0x030B, 0x01},
	{0x030D, 0x02},
	{0x030E, 0x01},
	{0x030F, 0x5E},
	{0x0310, 0x00},
	{0x0820, 0x10},
	{0x0821, 0x68},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x3100, 0x00},
	{0x3E20, 0x01},
	{0x3E37, 0x00},
	{0x3F50, 0x00},
	{0x3F56, 0x01},
	{0x3F57, 0x02},
	{0x3F4C, 0x01},
	{0xE000, 0x01},
	{0x3C0A, 0x5A},
	{0x3C0B, 0x55},
	{0x3C0C, 0x28},
	{0x3C0D, 0x07},
	{0x3C0E, 0xFF},
	{0x3C0F, 0x00},
	{0x3C10, 0x00},
	{0x3C11, 0x02},
	{0x3C12, 0x00},
	{0x3C13, 0x03},
	{0x3C14, 0x00},
	{0x3C15, 0x00},
	{0x3C16, 0x0C},
	{0x3C17, 0x0C},
	{0x3C18, 0x0C},
	{0x3C19, 0x0A},
	{0x3C1A, 0x0A},
	{0x3C1B, 0x0A},
	{0x3C1C, 0x00},
	{0x3C1D, 0x00},
	{0x3C1E, 0x00},
	{0x3C1F, 0x00},
	{0x3C20, 0x00},
	{0x3C21, 0x00},
	{0x3C22, 0x3F},
	{0x3C23, 0x0A},
	{0x3E35, 0x01},
	{0x3F4A, 0x03},
	{0x3F4B, 0xBF},
	{0x3F26, 0x00},
	{0x0202, 0x0C},
	{0x0203, 0x08},
	{0x0204, 0x00},
	{0x0205, 0x00},
	{0x020E, 0x01},
	{0x020F, 0x00},
	{0x0210, 0x01},
	{0x0211, 0x00},
	{0x0212, 0x01},
	{0x0213, 0x00},
	{0x0214, 0x01},
	{0x0215, 0x00},
	{0xBCF1, 0x00},
};

static const struct kmb_imx412_reg mode_3200x1800_2L_10b_regs[] = {
	{0x0136, 0x18},
	{0x0137, 0x00},
	{0x3C7E, 0x0C},
	{0x3C7F, 0x02},
	{0x38A8, 0x1F},
	{0x38A9, 0xFF},
	{0x38AA, 0x1F},
	{0x38AB, 0xFF},
	{0x55D4, 0x00},
	{0x55D5, 0x00},
	{0x55D6, 0x07},
	{0x55D7, 0xFF},
	{0x55E8, 0x07},
	{0x55E9, 0xFF},
	{0x55EA, 0x00},
	{0x55EB, 0x00},
	{0x575C, 0x07},
	{0x575D, 0xFF},
	{0x575E, 0x00},
	{0x575F, 0x00},
	{0x5764, 0x00},
	{0x5765, 0x00},
	{0x5766, 0x07},
	{0x5767, 0xFF},
	{0x5974, 0x04},
	{0x5975, 0x01},
	{0x5F10, 0x09},
	{0x5F11, 0x92},
	{0x5F12, 0x32},
	{0x5F13, 0x72},
	{0x5F14, 0x16},
	{0x5F15, 0xBA},
	{0x5F17, 0x13},
	{0x5F18, 0x24},
	{0x5F19, 0x60},
	{0x5F1A, 0xE3},
	{0x5F1B, 0xAD},
	{0x5F1C, 0x74},
	{0x5F2D, 0x25},
	{0x5F5C, 0xD0},
	{0x6A22, 0x00},
	{0x6A23, 0x1D},
	{0x7BA8, 0x00},
	{0x7BA9, 0x00},
	{0x886B, 0x00},
	{0x9002, 0x0A},
	{0x9004, 0x1A},
	{0x9214, 0x93},
	{0x9215, 0x69},
	{0x9216, 0x93},
	{0x9217, 0x6B},
	{0x9218, 0x93},
	{0x9219, 0x6D},
	{0x921A, 0x57},
	{0x921B, 0x58},
	{0x921C, 0x57},
	{0x921D, 0x59},
	{0x921E, 0x57},
	{0x921F, 0x5A},
	{0x9220, 0x57},
	{0x9221, 0x5B},
	{0x9222, 0x93},
	{0x9223, 0x02},
	{0x9224, 0x93},
	{0x9225, 0x03},
	{0x9226, 0x93},
	{0x9227, 0x04},
	{0x9228, 0x93},
	{0x9229, 0x05},
	{0x922A, 0x98},
	{0x922B, 0x21},
	{0x922C, 0xB2},
	{0x922D, 0xDB},
	{0x922E, 0xB2},
	{0x922F, 0xDC},
	{0x9230, 0xB2},
	{0x9231, 0xDD},
	{0x9232, 0xB2},
	{0x9233, 0xE1},
	{0x9234, 0xB2},
	{0x9235, 0xE2},
	{0x9236, 0xB2},
	{0x9237, 0xE3},
	{0x9238, 0xB7},
	{0x9239, 0xB9},
	{0x923A, 0xB7},
	{0x923B, 0xBB},
	{0x923C, 0xB7},
	{0x923D, 0xBC},
	{0x923E, 0xB7},
	{0x923F, 0xC5},
	{0x9240, 0xB7},
	{0x9241, 0xC7},
	{0x9242, 0xB7},
	{0x9243, 0xC9},
	{0x9244, 0x98},
	{0x9245, 0x56},
	{0x9246, 0x98},
	{0x9247, 0x55},
	{0x9380, 0x00},
	{0x9381, 0x62},
	{0x9382, 0x00},
	{0x9383, 0x56},
	{0x9384, 0x00},
	{0x9385, 0x52},
	{0x9388, 0x00},
	{0x9389, 0x55},
	{0x938A, 0x00},
	{0x938B, 0x55},
	{0x938C, 0x00},
	{0x938D, 0x41},
	{0x5078, 0x01},
	{0x9827, 0x20},
	{0x9830, 0x0A},
	{0x9833, 0x0A},
	{0x9834, 0x32},
	{0x9837, 0x22},
	{0x983C, 0x04},
	{0x983F, 0x0A},
	{0x994F, 0x00},
	{0x9A48, 0x06},
	{0x9A49, 0x06},
	{0x9A4A, 0x06},
	{0x9A4B, 0x06},
	{0x9A4E, 0x03},
	{0x9A4F, 0x03},
	{0x9A54, 0x03},
	{0x9A66, 0x03},
	{0x9A67, 0x03},
	{0xA2C9, 0x02},
	{0xA2CB, 0x02},
	{0xA2CD, 0x02},
	{0xB249, 0x3F},
	{0xB24F, 0x3F},
	{0xB290, 0x3F},
	{0xB293, 0x3F},
	{0xB296, 0x3F},
	{0xB299, 0x3F},
	{0xB2A2, 0x3F},
	{0xB2A8, 0x3F},
	{0xB2A9, 0x0D},
	{0xB2AA, 0x0D},
	{0xB2AB, 0x3F},
	{0xB2BA, 0x2F},
	{0xB2BB, 0x2F},
	{0xB2BC, 0x2F},
	{0xB2BD, 0x10},
	{0xB2C0, 0x3F},
	{0xB2C3, 0x3F},
	{0xB2D2, 0x3F},
	{0xB2DE, 0x20},
	{0xB2DF, 0x20},
	{0xB2E0, 0x20},
	{0xB2EA, 0x3F},
	{0xB2ED, 0x3F},
	{0xB2EE, 0x3F},
	{0xB2EF, 0x3F},
	{0xB2F0, 0x2F},
	{0xB2F1, 0x2F},
	{0xB2F2, 0x2F},
	{0xB2F9, 0x0E},
	{0xB2FA, 0x0E},
	{0xB2FB, 0x0E},
	{0xB759, 0x01},
	{0xB765, 0x3F},
	{0xB76B, 0x3F},
	{0xB7B3, 0x03},
	{0xB7B5, 0x03},
	{0xB7B7, 0x03},
	{0xB7BF, 0x03},
	{0xB7C1, 0x03},
	{0xB7C3, 0x03},
	{0xB7EF, 0x02},
	{0xB7F5, 0x1F},
	{0xB7F7, 0x1F},
	{0xB7F9, 0x1F},
	{0x0112, 0x0A},
	{0x0113, 0x0A},
	{0x0114, 0x01},
	{0x0342, 0x23},
	{0x0343, 0x40},
	{0x0340, 0x07},
	{0x0341, 0x45},
	{0x3210, 0x00},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x02},
	{0x0347, 0x6C},
	{0x0348, 0x0F},
	{0x0349, 0xD7},
	{0x034A, 0x09},
	{0x034B, 0x73},
	{0x00E3, 0x00},
	{0x00E4, 0x00},
	{0x00E5, 0x01},
	{0x00FC, 0x0A},
	{0x00FD, 0x0A},
	{0x00FE, 0x0A},
	{0x00FF, 0x0A},
	{0xE013, 0x00},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x00},
	{0x0901, 0x11},
	{0x0902, 0x00},
	{0x3140, 0x02},
	{0x3241, 0x11},
	{0x3250, 0x03},
	{0x3E10, 0x00},
	{0x3E11, 0x00},
	{0x3F0D, 0x00},
	{0x3F42, 0x00},
	{0x3F43, 0x00},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x10},
	{0x0408, 0x01},
	{0x0409, 0xAC},
	{0x040A, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x0C},
	{0x040D, 0x80},
	{0x040E, 0x07},
	{0x040F, 0x08},
	{0x034C, 0x0C},
	{0x034D, 0x80},
	{0x034E, 0x07},
	{0x034F, 0x08},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x04},
	{0x0306, 0x00},
	{0x0307, 0xD2},
	{0x0309, 0x0A},
	{0x030B, 0x01},
	{0x030D, 0x02},
	{0x030E, 0x01},
	{0x030F, 0x5E},
	{0x0310, 0x00},
	{0x0820, 0x09},
	{0x0821, 0xD8},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x3E20, 0x01},
	{0x3E37, 0x00},
	{0x3F50, 0x00},
	{0x3F56, 0x01},
	{0x3F57, 0xAE},
	{0x3C0A, 0x5A},
	{0x3C0B, 0x55},
	{0x3C0C, 0x28},
	{0x3C0D, 0x07},
	{0x3C0E, 0xFF},
	{0x3C0F, 0x00},
	{0x3C10, 0x00},
	{0x3C11, 0x02},
	{0x3C12, 0x00},
	{0x3C13, 0x03},
	{0x3C14, 0x00},
	{0x3C15, 0x00},
	{0x3C16, 0x0C},
	{0x3C17, 0x0C},
	{0x3C18, 0x0C},
	{0x3C19, 0x0A},
	{0x3C1A, 0x0A},
	{0x3C1B, 0x0A},
	{0x3C1C, 0x00},
	{0x3C1D, 0x00},
	{0x3C1E, 0x00},
	{0x3C1F, 0x00},
	{0x3C20, 0x00},
	{0x3C21, 0x00},
	{0x3C22, 0x3F},
	{0x3C23, 0x0A},
	{0x3E35, 0x01},
	{0x3F4A, 0x03},
	{0x3F4B, 0xBF},
	{0x3F26, 0x00},
	{0x0202, 0x07},
	{0x0203, 0x2F},
	{0x0204, 0x00},
	{0x0205, 0x00},
	{0x020E, 0x01},
	{0x020F, 0x00},
	{0x0210, 0x01},
	{0x0211, 0x00},
	{0x0212, 0x01},
	{0x0213, 0x00},
	{0x0214, 0x01},
	{0x0215, 0x00},
	{0xBCF1, 0x00},
};

static const struct kmb_imx412_reg mode_1920x1080_2L_12b_regs[] = {
	{0x0136, 0x18},
	{0x0137, 0x00},
	{0x3C7E, 0x06},
	{0x3C7F, 0x02},
	{0x38A8, 0x1F},
	{0x38A9, 0xFF},
	{0x38AA, 0x1F},
	{0x38AB, 0xFF},
	{0x55D4, 0x00},
	{0x55D5, 0x00},
	{0x55D6, 0x07},
	{0x55D7, 0xFF},
	{0x55E8, 0x07},
	{0x55E9, 0xFF},
	{0x55EA, 0x00},
	{0x55EB, 0x00},
	{0x575C, 0x07},
	{0x575D, 0xFF},
	{0x575E, 0x00},
	{0x575F, 0x00},
	{0x5764, 0x00},
	{0x5765, 0x00},
	{0x5766, 0x07},
	{0x5767, 0xFF},
	{0x5974, 0x04},
	{0x5975, 0x01},
	{0x5F10, 0x09},
	{0x5F11, 0x92},
	{0x5F12, 0x32},
	{0x5F13, 0x72},
	{0x5F14, 0x16},
	{0x5F15, 0xBA},
	{0x5F17, 0x13},
	{0x5F18, 0x24},
	{0x5F19, 0x60},
	{0x5F1A, 0xE3},
	{0x5F1B, 0xAD},
	{0x5F1C, 0x74},
	{0x5F2D, 0x25},
	{0x5F5C, 0xD0},
	{0x6A22, 0x00},
	{0x6A23, 0x1D},
	{0x7BA8, 0x00},
	{0x7BA9, 0x00},
	{0x886B, 0x00},
	{0x9002, 0x0A},
	{0x9004, 0x1A},
	{0x9214, 0x93},
	{0x9215, 0x69},
	{0x9216, 0x93},
	{0x9217, 0x6B},
	{0x9218, 0x93},
	{0x9219, 0x6D},
	{0x921A, 0x57},
	{0x921B, 0x58},
	{0x921C, 0x57},
	{0x921D, 0x59},
	{0x921E, 0x57},
	{0x921F, 0x5A},
	{0x9220, 0x57},
	{0x9221, 0x5B},
	{0x9222, 0x93},
	{0x9223, 0x02},
	{0x9224, 0x93},
	{0x9225, 0x03},
	{0x9226, 0x93},
	{0x9227, 0x04},
	{0x9228, 0x93},
	{0x9229, 0x05},
	{0x922A, 0x98},
	{0x922B, 0x21},
	{0x922C, 0xB2},
	{0x922D, 0xDB},
	{0x922E, 0xB2},
	{0x922F, 0xDC},
	{0x9230, 0xB2},
	{0x9231, 0xDD},
	{0x9232, 0xB2},
	{0x9233, 0xE1},
	{0x9234, 0xB2},
	{0x9235, 0xE2},
	{0x9236, 0xB2},
	{0x9237, 0xE3},
	{0x9238, 0xB7},
	{0x9239, 0xB9},
	{0x923A, 0xB7},
	{0x923B, 0xBB},
	{0x923C, 0xB7},
	{0x923D, 0xBC},
	{0x923E, 0xB7},
	{0x923F, 0xC5},
	{0x9240, 0xB7},
	{0x9241, 0xC7},
	{0x9242, 0xB7},
	{0x9243, 0xC9},
	{0x9244, 0x98},
	{0x9245, 0x56},
	{0x9246, 0x98},
	{0x9247, 0x55},
	{0x9380, 0x00},
	{0x9381, 0x62},
	{0x9382, 0x00},
	{0x9383, 0x56},
	{0x9384, 0x00},
	{0x9385, 0x52},
	{0x9388, 0x00},
	{0x9389, 0x55},
	{0x938A, 0x00},
	{0x938B, 0x55},
	{0x938C, 0x00},
	{0x938D, 0x41},
	{0x5078, 0x01},
	{0x9827, 0x20},
	{0x9830, 0x0A},
	{0x9833, 0x0A},
	{0x9834, 0x32},
	{0x9837, 0x22},
	{0x983C, 0x04},
	{0x983F, 0x0A},
	{0x994F, 0x00},
	{0x9A48, 0x06},
	{0x9A49, 0x06},
	{0x9A4A, 0x06},
	{0x9A4B, 0x06},
	{0x9A4E, 0x03},
	{0x9A4F, 0x03},
	{0x9A54, 0x03},
	{0x9A66, 0x03},
	{0x9A67, 0x03},
	{0xA2C9, 0x02},
	{0xA2CB, 0x02},
	{0xA2CD, 0x02},
	{0xB249, 0x3F},
	{0xB24F, 0x3F},
	{0xB290, 0x3F},
	{0xB293, 0x3F},
	{0xB296, 0x3F},
	{0xB299, 0x3F},
	{0xB2A2, 0x3F},
	{0xB2A8, 0x3F},
	{0xB2A9, 0x0D},
	{0xB2AA, 0x0D},
	{0xB2AB, 0x3F},
	{0xB2BA, 0x2F},
	{0xB2BB, 0x2F},
	{0xB2BC, 0x2F},
	{0xB2BD, 0x10},
	{0xB2C0, 0x3F},
	{0xB2C3, 0x3F},
	{0xB2D2, 0x3F},
	{0xB2DE, 0x20},
	{0xB2DF, 0x20},
	{0xB2E0, 0x20},
	{0xB2EA, 0x3F},
	{0xB2ED, 0x3F},
	{0xB2EE, 0x3F},
	{0xB2EF, 0x3F},
	{0xB2F0, 0x2F},
	{0xB2F1, 0x2F},
	{0xB2F2, 0x2F},
	{0xB2F9, 0x0E},
	{0xB2FA, 0x0E},
	{0xB2FB, 0x0E},
	{0xB759, 0x01},
	{0xB765, 0x3F},
	{0xB76B, 0x3F},
	{0xB7B3, 0x03},
	{0xB7B5, 0x03},
	{0xB7B7, 0x03},
	{0xB7BF, 0x03},
	{0xB7C1, 0x03},
	{0xB7C3, 0x03},
	{0xB7EF, 0x02},
	{0xB7F5, 0x1F},
	{0xB7F7, 0x1F},
	{0xB7F9, 0x1F},
	{0x0112, 0x0C},
	{0x0113, 0x0C},
	{0x0114, 0x01},
	{0x0342, 0x18},
	{0x0343, 0x50},
	{0x0340, 0x0A},
	{0x0341, 0x0A},
	{0x3210, 0x00},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x01},
	{0x0347, 0xB8},
	{0x0348, 0x0F},
	{0x0349, 0xD7},
	{0x034A, 0x0A},
	{0x034B, 0x27},
	{0x00E3, 0x00},
	{0x00E4, 0x00},
	{0x00E5, 0x01},
	{0x00FC, 0x0A},
	{0x00FD, 0x0A},
	{0x00FE, 0x0A},
	{0x00FF, 0x0A},
	{0xE013, 0x00},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x00},
	{0x0901, 0x11},
	{0x0902, 0x00},
	{0x3140, 0x02},
	{0x3241, 0x11},
	{0x3250, 0x03},
	{0x3E10, 0x00},
	{0x3E11, 0x00},
	{0x3F0D, 0x01},
	{0x3F42, 0x00},
	{0x3F43, 0x00},
	{0x0401, 0x02},
	{0x0404, 0x00},
	{0x0405, 0x20},
	{0x0408, 0x00},
	{0x0409, 0x6C},
	{0x040A, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x0F},
	{0x040D, 0x00},
	{0x040E, 0x08},
	{0x040F, 0x70},
	{0x034C, 0x07},
	{0x034D, 0x80},
	{0x034E, 0x04},
	{0x034F, 0x38},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x04},
	{0x0306, 0x00},
	{0x0307, 0xC8},
	{0x0309, 0x0C},
	{0x030B, 0x01},
	{0x030D, 0x02},
	{0x030E, 0x01},
	{0x030F, 0x5E},
	{0x0310, 0x00},
	{0x0820, 0x09},
	{0x0821, 0x60},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x3E20, 0x01},
	{0x3E37, 0x00},
	{0x3F50, 0x00},
	{0x3F56, 0x01},
	{0x3F57, 0x38},
	{0xE000, 0x01},
	{0x3C0A, 0x5A},
	{0x3C0B, 0x55},
	{0x3C0C, 0x28},
	{0x3C0D, 0x07},
	{0x3C0E, 0xFF},
	{0x3C0F, 0x00},
	{0x3C10, 0x00},
	{0x3C11, 0x02},
	{0x3C12, 0x00},
	{0x3C13, 0x03},
	{0x3C14, 0x00},
	{0x3C15, 0x00},
	{0x3C16, 0x0C},
	{0x3C17, 0x0C},
	{0x3C18, 0x0C},
	{0x3C19, 0x0A},
	{0x3C1A, 0x0A},
	{0x3C1B, 0x0A},
	{0x3C1C, 0x00},
	{0x3C1D, 0x00},
	{0x3C1E, 0x00},
	{0x3C1F, 0x00},
	{0x3C20, 0x00},
	{0x3C21, 0x00},
	{0x3C22, 0x3F},
	{0x3C23, 0x0A},
	{0x3E35, 0x01},
	{0x3F4A, 0x03},
	{0x3F4B, 0x85},
	{0x3F26, 0x00},
	{0x0202, 0x09},
	{0x0203, 0xF4},
	{0x0204, 0x00},
	{0x0205, 0x00},
	{0x020E, 0x01},
	{0x020F, 0x00},
	{0x0210, 0x01},
	{0x0211, 0x00},
	{0x0212, 0x01},
	{0x0213, 0x00},
	{0x0214, 0x01},
	{0x0215, 0x00},
	{0xBCF1, 0x00},
};

static const struct kmb_imx412_reg mode_4056x3040_2DOL_1452_regs[] = {
	{0x0100, 0x00},
	{0x0136, 0x18},
	{0x0137, 0x00},
	{0x3C7E, 0x0B},
	{0x3C7F, 0x02},
	{0x38A8, 0x1F},
	{0x38A9, 0xFF},
	{0x38AA, 0x1F},
	{0x38AB, 0xFF},
	{0x55D4, 0x00},
	{0x55D5, 0x00},
	{0x55D6, 0x07},
	{0x55D7, 0xFF},
	{0x55E8, 0x07},
	{0x55E9, 0xFF},
	{0x55EA, 0x00},
	{0x55EB, 0x00},
	{0x575C, 0x07},
	{0x575D, 0xFF},
	{0x575E, 0x00},
	{0x575F, 0x00},
	{0x5764, 0x00},
	{0x5765, 0x00},
	{0x5766, 0x07},
	{0x5767, 0xFF},
	{0x5974, 0x04},
	{0x5975, 0x01},
	{0x5F10, 0x09},
	{0x5F11, 0x92},
	{0x5F12, 0x32},
	{0x5F13, 0x72},
	{0x5F14, 0x16},
	{0x5F15, 0xBA},
	{0x5F17, 0x13},
	{0x5F18, 0x24},
	{0x5F19, 0x60},
	{0x5F1A, 0xE3},
	{0x5F1B, 0xAD},
	{0x5F1C, 0x74},
	{0x5F2D, 0x25},
	{0x5F5C, 0xD0},
	{0x6A22, 0x00},
	{0x6A23, 0x1D},
	{0x7BA8, 0x00},
	{0x7BA9, 0x00},
	{0x886B, 0x00},
	{0x9002, 0x0A},
	{0x9004, 0x1A},
	{0x9214, 0x93},
	{0x9215, 0x69},
	{0x9216, 0x93},
	{0x9217, 0x6B},
	{0x9218, 0x93},
	{0x9219, 0x6D},
	{0x921A, 0x57},
	{0x921B, 0x58},
	{0x921C, 0x57},
	{0x921D, 0x59},
	{0x921E, 0x57},
	{0x921F, 0x5A},
	{0x9220, 0x57},
	{0x9221, 0x5B},
	{0x9222, 0x93},
	{0x9223, 0x02},
	{0x9224, 0x93},
	{0x9225, 0x03},
	{0x9226, 0x93},
	{0x9227, 0x04},
	{0x9228, 0x93},
	{0x9229, 0x05},
	{0x922A, 0x98},
	{0x922B, 0x21},
	{0x922C, 0xB2},
	{0x922D, 0xDB},
	{0x922E, 0xB2},
	{0x922F, 0xDC},
	{0x9230, 0xB2},
	{0x9231, 0xDD},
	{0x9232, 0xB2},
	{0x9233, 0xE1},
	{0x9234, 0xB2},
	{0x9235, 0xE2},
	{0x9236, 0xB2},
	{0x9237, 0xE3},
	{0x9238, 0xB7},
	{0x9239, 0xB9},
	{0x923A, 0xB7},
	{0x923B, 0xBB},
	{0x923C, 0xB7},
	{0x923D, 0xBC},
	{0x923E, 0xB7},
	{0x923F, 0xC5},
	{0x9240, 0xB7},
	{0x9241, 0xC7},
	{0x9242, 0xB7},
	{0x9243, 0xC9},
	{0x9244, 0x98},
	{0x9245, 0x56},
	{0x9246, 0x98},
	{0x9247, 0x55},
	{0x9380, 0x00},
	{0x9381, 0x62},
	{0x9382, 0x00},
	{0x9383, 0x56},
	{0x9384, 0x00},
	{0x9385, 0x52},
	{0x9388, 0x00},
	{0x9389, 0x55},
	{0x938A, 0x00},
	{0x938B, 0x55},
	{0x938C, 0x00},
	{0x938D, 0x41},
	{0x5078, 0x01},
	{0x0112, 0x0C},
	{0x0113, 0x0C},
	{0x0114, 0x03},
	{0x0342, 0x18},
	{0x0343, 0x50},
	{0x0340, 0x0C},
	{0x0341, 0x26},
	{0x3210, 0x00},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x0F},
	{0x0349, 0xD7},
	{0x034A, 0x0B},
	{0x034B, 0xDF},
	{0x00E3, 0x01},
	{0x00E4, 0x01},
	{0x00E5, 0x00},
	{0x00FC, 0x0C},
	{0x00FD, 0x0C},
	{0x00FE, 0x0C},
	{0x00FF, 0x0C},
	{0xE013, 0x01},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x00},
	{0x0901, 0x11},
	{0x0902, 0x00},
	{0x3140, 0x02},
	{0x3241, 0x11},
	{0x3250, 0x03},
	{0x3E10, 0x01},
	{0x3E11, 0x02},
	{0x3F0D, 0x01},
	{0x3F42, 0x00},
	{0x3F43, 0x00},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x10},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040A, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x0F},
	{0x040D, 0xD8},
	{0x040E, 0x0B},
	{0x040F, 0xE0},
	{0x034C, 0x0F},
	{0x034D, 0xD8},
	{0x034E, 0x0B},
	{0x034F, 0xE0},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x04},
	{0x0306, 0x00},
	{0x0307, 0xF2},
	{0x0309, 0x0C},
	{0x030B, 0x01},
	{0x030D, 0x02},
	{0x030E, 0x01},
	{0x030F, 0x5E},
	{0x0310, 0x00},
	{0x0820, 0x16},
	{0x0821, 0xB0},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x3E20, 0x01},
	{0x3E37, 0x00},
	{0x3F50, 0x00},
	{0x3F56, 0x01},
	{0x3F57, 0x02},
	{0x3C0A, 0x5A},
	{0x3C0B, 0x55},
	{0x3C0C, 0x28},
	{0x3C0D, 0x07},
	{0x3C0E, 0xFF},
	{0x3C0F, 0x00},
	{0x3C10, 0x00},
	{0x3C11, 0x02},
	{0x3C12, 0x00},
	{0x3C13, 0x03},
	{0x3C14, 0x00},
	{0x3C15, 0x00},
	{0x3C16, 0x0C},
	{0x3C17, 0x0C},
	{0x3C18, 0x0C},
	{0x3C19, 0x0A},
	{0x3C1A, 0x0A},
	{0x3C1B, 0x0A},
	{0x3C1C, 0x00},
	{0x3C1D, 0x00},
	{0x3C1E, 0x00},
	{0x3C1F, 0x00},
	{0x3C20, 0x00},
	{0x3C21, 0x00},
	{0x3C22, 0x3F},
	{0x3C23, 0x0A},
	{0x3E35, 0x01},
	{0x3F4A, 0x03},
	{0x3F4B, 0x85},
	{0x3F26, 0x00},
	{0x0202, 0x0C},
	{0x0203, 0x10},
	{0x0204, 0x00},
	{0x0205, 0x00},
	{0x020E, 0x01},
	{0x020F, 0x00},
	{0x0210, 0x01},
	{0x0211, 0x00},
	{0x0212, 0x01},
	{0x0213, 0x00},
	{0x0214, 0x01},
	{0x0215, 0x00},
	{0xBCF1, 0x00},
	{0x00EC, 0xB4},
	{0xE000, 0x01},
};

static const struct kmb_imx412_reg mode_4056x3040_3DOL_1452_regs[] = {
	{0x0100, 0x00},
	{0x0136, 0x18},
	{0x0137, 0x00},
	{0x3C7E, 0x0B},
	{0x3C7F, 0x02},
	{0x38A8, 0x1F},
	{0x38A9, 0xFF},
	{0x38AA, 0x1F},
	{0x38AB, 0xFF},
	{0x55D4, 0x00},
	{0x55D5, 0x00},
	{0x55D6, 0x07},
	{0x55D7, 0xFF},
	{0x55E8, 0x07},
	{0x55E9, 0xFF},
	{0x55EA, 0x00},
	{0x55EB, 0x00},
	{0x575C, 0x07},
	{0x575D, 0xFF},
	{0x575E, 0x00},
	{0x575F, 0x00},
	{0x5764, 0x00},
	{0x5765, 0x00},
	{0x5766, 0x07},
	{0x5767, 0xFF},
	{0x5974, 0x04},
	{0x5975, 0x01},
	{0x5F10, 0x09},
	{0x5F11, 0x92},
	{0x5F12, 0x32},
	{0x5F13, 0x72},
	{0x5F14, 0x16},
	{0x5F15, 0xBA},
	{0x5F17, 0x13},
	{0x5F18, 0x24},
	{0x5F19, 0x60},
	{0x5F1A, 0xE3},
	{0x5F1B, 0xAD},
	{0x5F1C, 0x74},
	{0x5F2D, 0x25},
	{0x5F5C, 0xD0},
	{0x6A22, 0x00},
	{0x6A23, 0x1D},
	{0x7BA8, 0x00},
	{0x7BA9, 0x00},
	{0x886B, 0x00},
	{0x9002, 0x0A},
	{0x9004, 0x1A},
	{0x9214, 0x93},
	{0x9215, 0x69},
	{0x9216, 0x93},
	{0x9217, 0x6B},
	{0x9218, 0x93},
	{0x9219, 0x6D},
	{0x921A, 0x57},
	{0x921B, 0x58},
	{0x921C, 0x57},
	{0x921D, 0x59},
	{0x921E, 0x57},
	{0x921F, 0x5A},
	{0x9220, 0x57},
	{0x9221, 0x5B},
	{0x9222, 0x93},
	{0x9223, 0x02},
	{0x9224, 0x93},
	{0x9225, 0x03},
	{0x9226, 0x93},
	{0x9227, 0x04},
	{0x9228, 0x93},
	{0x9229, 0x05},
	{0x922A, 0x98},
	{0x922B, 0x21},
	{0x922C, 0xB2},
	{0x922D, 0xDB},
	{0x922E, 0xB2},
	{0x922F, 0xDC},
	{0x9230, 0xB2},
	{0x9231, 0xDD},
	{0x9232, 0xB2},
	{0x9233, 0xE1},
	{0x9234, 0xB2},
	{0x9235, 0xE2},
	{0x9236, 0xB2},
	{0x9237, 0xE3},
	{0x9238, 0xB7},
	{0x9239, 0xB9},
	{0x923A, 0xB7},
	{0x923B, 0xBB},
	{0x923C, 0xB7},
	{0x923D, 0xBC},
	{0x923E, 0xB7},
	{0x923F, 0xC5},
	{0x9240, 0xB7},
	{0x9241, 0xC7},
	{0x9242, 0xB7},
	{0x9243, 0xC9},
	{0x9244, 0x98},
	{0x9245, 0x56},
	{0x9246, 0x98},
	{0x9247, 0x55},
	{0x9380, 0x00},
	{0x9381, 0x62},
	{0x9382, 0x00},
	{0x9383, 0x56},
	{0x9384, 0x00},
	{0x9385, 0x52},
	{0x9388, 0x00},
	{0x9389, 0x55},
	{0x938A, 0x00},
	{0x938B, 0x55},
	{0x938C, 0x00},
	{0x938D, 0x41},
	{0x5078, 0x01},
	{0x0112, 0x0C},
	{0x0113, 0x0C},
	{0x0114, 0x03},
	{0x0342, 0x18},
	{0x0343, 0x50},
	{0x0340, 0x0C},
	{0x0341, 0x26},
	{0x3210, 0x00},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x0F},
	{0x0349, 0xD7},
	{0x034A, 0x0B},
	{0x034B, 0xDF},
	{0x00E3, 0x01},
	{0x00E4, 0x02},
	{0x00E5, 0x00},
	{0x00FC, 0x0C},
	{0x00FD, 0x0C},
	{0x00FE, 0x0C},
	{0x00FF, 0x0C},
	{0xE013, 0x01},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x00},
	{0x0901, 0x11},
	{0x0902, 0x00},
	{0x3140, 0x02},
	{0x3241, 0x11},
	{0x3250, 0x03},
	{0x3E10, 0x01},
	{0x3E11, 0x02},
	{0x3F0D, 0x01},
	{0x3F42, 0x00},
	{0x3F43, 0x00},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x10},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040A, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x0F},
	{0x040D, 0xD8},
	{0x040E, 0x0B},
	{0x040F, 0xE0},
	{0x034C, 0x0F},
	{0x034D, 0xD8},
	{0x034E, 0x0B},
	{0x034F, 0xE0},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x04},
	{0x0306, 0x00},
	{0x0307, 0xF2},
	{0x0309, 0x0C},
	{0x030B, 0x01},
	{0x030D, 0x02},
	{0x030E, 0x01},
	{0x030F, 0x5E},
	{0x0310, 0x00},
	{0x0820, 0x16},
	{0x0821, 0xB0},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x3E20, 0x01},
	{0x3E37, 0x00},
	{0x3F50, 0x00},
	{0x3F56, 0x01},
	{0x3F57, 0x02},
	{0x3C0A, 0x5A},
	{0x3C0B, 0x55},
	{0x3C0C, 0x28},
	{0x3C0D, 0x07},
	{0x3C0E, 0xFF},
	{0x3C0F, 0x00},
	{0x3C10, 0x00},
	{0x3C11, 0x02},
	{0x3C12, 0x00},
	{0x3C13, 0x03},
	{0x3C14, 0x00},
	{0x3C15, 0x00},
	{0x3C16, 0x0C},
	{0x3C17, 0x0C},
	{0x3C18, 0x0C},
	{0x3C19, 0x0A},
	{0x3C1A, 0x0A},
	{0x3C1B, 0x0A},
	{0x3C1C, 0x00},
	{0x3C1D, 0x00},
	{0x3C1E, 0x00},
	{0x3C1F, 0x00},
	{0x3C20, 0x00},
	{0x3C21, 0x00},
	{0x3C22, 0x3F},
	{0x3C23, 0x0A},
	{0x3E35, 0x01},
	{0x3F4A, 0x03},
	{0x3F4B, 0x85},
	{0x3F26, 0x00},
	{0x0202, 0x0C},
	{0x0203, 0x10},
	{0x0204, 0x00},
	{0x0205, 0x00},
	{0x020E, 0x01},
	{0x020F, 0x00},
	{0x0210, 0x01},
	{0x0211, 0x00},
	{0x0212, 0x01},
	{0x0213, 0x00},
	{0x0214, 0x01},
	{0x0215, 0x00},
	{0xBCF1, 0x00},
	{0x00EC, 0xB4},
	{0xE000, 0x01},
};

/* Supported sensor mode configurations */
static const struct kmb_imx412_mode supported_modes[] = {
	{
		.width = 4056,
		.height = 3040,
		.ppln = 4512,
		.lpfr[0] = 3546,
		.skip_lines = 0,
		.pclk = 480000000,
		.code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.num_lanes = 4,
		.link_freq_idx = 1,
		.fps = {
			.def = 30 * KMB_IMX412_FPS_NUMERATOR,
			.max = 30 * KMB_IMX412_FPS_NUMERATOR,
			.min = 3 * KMB_IMX412_FPS_NUMERATOR,
			.step = 5 * KMB_IMX412_FPS_NUMERATOR,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_4056x3040_regs),
			.regs = mode_4056x3040_regs,
		},
	},
	{
		.width = 4056,
		.height = 3040,
		.ppln = 4512,
		.lpfr[0] = 3102,
		.skip_lines = 0,
		.pclk = 420000000,
		.code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.num_lanes = 2,
		.link_freq_idx = 0,
		.fps = {
			.def = 30 * KMB_IMX412_FPS_NUMERATOR,
			.max = 30 * KMB_IMX412_FPS_NUMERATOR,
			.min = 2 * KMB_IMX412_FPS_NUMERATOR,
			.step = 5 * KMB_IMX412_FPS_NUMERATOR,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_4056x3040_2L_regs),
			.regs = mode_4056x3040_2L_regs,
		},
	},
	{
		.width = 3200,
		.height = 1800,
		.ppln = 4512,
		.lpfr[0] = 1861,
		.skip_lines = 0,
		.pclk = 252000000,
		.code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.num_lanes = 2,
		.link_freq_idx = 3,
		.fps = {
			.def = 30 * KMB_IMX412_FPS_NUMERATOR,
			.max = 30 * KMB_IMX412_FPS_NUMERATOR,
			.min = 2 * KMB_IMX412_FPS_NUMERATOR,
			.step = 5 * KMB_IMX412_FPS_NUMERATOR,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_3200x1800_2L_10b_regs),
			.regs = mode_3200x1800_2L_10b_regs,
		},
	},
	{
		.width = 1920,
		.height = 1080,
		.ppln = 2841,
		.lpfr[0] = 2570,
		.skip_lines = 0,
		.pclk = 200000000,
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.num_lanes = 2,
		.link_freq_idx = 1,
		.fps = {
			.def = 30 * KMB_IMX412_FPS_NUMERATOR,
			.max = 30 * KMB_IMX412_FPS_NUMERATOR,
			.min = 2 * KMB_IMX412_FPS_NUMERATOR,
			.step = 5 * KMB_IMX412_FPS_NUMERATOR,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1920x1080_2L_12b_regs),
			.regs = mode_1920x1080_2L_12b_regs,
		},
	}
};

static const struct kmb_imx412_mode hdr_2dol_supported_modes[] = {
	{
		.width = 4056,
		.height = 3040,
		.ppln = 4498,
		.lpfr[0] = 3410,
		.lpfr[1] = 200,
		.skip_lines = 0,
		.pclk = 484000000,
		.row_time = (12860 * 2),
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.num_lanes = 4,
		.link_freq_idx = 2,
		.fps = {
			.def = 15 * KMB_IMX412_FPS_NUMERATOR,
			.max = 15 * KMB_IMX412_FPS_NUMERATOR,
			.min = 2 * KMB_IMX412_FPS_NUMERATOR,
			.step = 5 * KMB_IMX412_FPS_NUMERATOR,
		},
		.reg_list = {
			.num_of_regs
				= ARRAY_SIZE(mode_4056x3040_2DOL_1452_regs),
			.regs = mode_4056x3040_2DOL_1452_regs,
		},
	}
};

static const struct kmb_imx412_mode hdr_3dol_supported_modes[] = {
	{
		.width = 4056,
		.height = 3040,
		.ppln = 4498,
		.lpfr[0] = 3410,
		.lpfr[1] = 246,
		.lpfr[2] = 282,
		.skip_lines = 0,
		.pclk = 484000000,
		.row_time = (12860 * 4),
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.num_lanes = 4,
		.link_freq_idx = 2,
		.fps = {
			.def = 7.5 * KMB_IMX412_FPS_NUMERATOR,
			.max = 7.5 * KMB_IMX412_FPS_NUMERATOR,
			.min = 7.5 * KMB_IMX412_FPS_NUMERATOR,
			.step = 1 * KMB_IMX412_FPS_NUMERATOR,
		},
		.reg_list = {
			.num_of_regs
				= ARRAY_SIZE(mode_4056x3040_3DOL_1452_regs),
			.regs = mode_4056x3040_3DOL_1452_regs,
		},
	}
};

/**
 * to_kmb_imx412 - imv412 V4L2 sub-device to kmb_imx412 device.
 * @subdev: pointer to imx412 V4L2 sub-device device
 *
 * Return: Pointer to kmb_imx412 device
 */
static inline struct kmb_imx412 *to_kmb_imx412(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct kmb_imx412, sd);
}

/**
 * kmb_imx412_read_reg - Read registers.
 * @kmb_imx412: pointer to imx412 device
 * @reg: Register address
 * @len: Length of bytes to read. Max supported bytes is 4
 * @val: Pointer to register value to be filled.
 *
 * Return: 0 if successful
 */
static int
kmb_imx412_read_reg(struct kmb_imx412 *kmb_imx412, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&kmb_imx412->sd);
	struct i2c_msg msgs[2] = { 0 };
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;

	if (len > 4)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

/**
 * kmb_imx412_i2c_write - Write register to i2c client
 * @client: i2c client to write to
 * @reg: Register address
 * @len: Length of bytes. Max supported bytes is 4
 * @val: Register value
 *
 * Return: 0 if successful
 */
static int
kmb_imx412_i2c_write(struct i2c_client *client, u16 reg, u32 len, u32 val)
{
	int buf_i, val_i;
	u8 buf[6], *val_p;
	__be32 val_be;
	int ret;

	if (WARN_ON(!client))
		return -ENODEV;

	if (len > 4) {
		dev_err_ratelimited(&client->dev,
				    "write reg 0x%4.4x invalid len %d",
				    reg, len);
		return -EINVAL;
	}

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	ret = i2c_master_send(client, buf, len + 2);
	if (ret != len + 2) {
		dev_err_ratelimited(&client->dev,
				    "write reg 0x%4.4x return err %d",
				    reg, ret);
		return -EIO;
	}

	return 0;
}

/**
 * kmb_imx412_write_reg - Write register to primary i2c client
 * @kmb_imx412: pointer to imx412 device
 * @reg: Register address
 * @len: Length of bytes. Max supported bytes is 4
 * @val: Register value
 *
 * Return: 0 if successful
 */
static int
kmb_imx412_write_reg(struct kmb_imx412 *kmb_imx412, u16 reg, u32 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&kmb_imx412->sd);

	return kmb_imx412_i2c_write(client, reg, len, val);
}

/**
 * kmb_imx412_i2c_broadcast - Broadcast on auxiliary i2c client. This is done
 *                            when dual camera on a shared i2c bus without XVS
 *                            is enabled and the Streaming command to both
 *                            sensors is sent at the same time.
 * @kmb_imx412: pointer to imx412 device
 * @reg: Register address
 * @len: Length of bytes. Max supported bytes is 4
 * @val: Register value
 *
 * Return: 0 if successful
 */
static int
kmb_imx412_i2c_broadcast(struct kmb_imx412 *kmb_imx412, u16 reg, u32 len,
			 u32 val)
{
	return kmb_imx412_i2c_write(kmb_imx412->broadcast_client,
				    reg, len, val);
}

/**
 * kmb_imx412_write_regs - Write a list of registers
 * @kmb_imx412: pointer to imx412 device
 * @regs: List of registers to be written
 * @len: Length of registers array
 *
 * Return: 0 if successful
 */
static int kmb_imx412_write_regs(struct kmb_imx412 *kmb_imx412,
			     const struct kmb_imx412_reg *regs, u32 len)
{
	int ret;
	u32 i;

	for (i = 0; i < len; i++) {
		ret = kmb_imx412_write_reg(kmb_imx412,
					   regs[i].address,
					   1, regs[i].val);
		if (ret)
			return ret;
	}

	return 0;
}

/**
 * kmb_imx412_update_fps - Update current sensor mode to match the selected FPS
 * @kmb_imx412: pointer to imx412 device
 * @mode: pointer to kmb_imx412_mode sensor mode
 *
 * Return: none
 */
static void kmb_imx412_update_fps(struct kmb_imx412 *kmb_imx412,
				  const struct kmb_imx412_mode *mode)
{
	u32 lpfr = (mode->lpfr[0] *
		(mode->fps.def / KMB_IMX412_FPS_NUMERATOR)) / kmb_imx412->fps;

	if (lpfr > KMB_IMX412_LPFR_MAX)
		lpfr = KMB_IMX412_LPFR_MAX;

	if (lpfr < mode->height + 22)
		lpfr = mode->height + 22;

	kmb_imx412->lpfr = lpfr;

	dev_dbg(kmb_imx412->dev, "Selected FPS %d lpfr %d",
		kmb_imx412->fps, lpfr);
}

/**
 * kmb_imx412_open - Open imx412 subdevice
 * @sd: pointer to imx412 V4L2 sub-device structure
 * @fh: pointer to imx412 V4L2 sub-device file handle
 *
 * Return: 0 if successful
 */
static int kmb_imx412_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct kmb_imx412 *kmb_imx412 = to_kmb_imx412(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->state, 0);

	mutex_lock(&kmb_imx412->mutex);

	/* Initialize try_fmt */
	try_fmt->width = kmb_imx412->cur_mode->width;
	try_fmt->height = kmb_imx412->cur_mode->height;
	try_fmt->code = kmb_imx412->cur_mode->code;
	try_fmt->field = V4L2_FIELD_NONE;
	kmb_imx412->sync_type = KMB_SYNC_TYPE_HW;

	mutex_unlock(&kmb_imx412->mutex);

	return 0;
}

/**
 * kmb_imx412_open - Handle synchronous starting of more than one sensors
 *
 * @kmb_imx412: pointer to kmb_imx412 device
 *
 * Return: 0 if successful
 */
static int kmb_imx412_sync_start(struct kmb_imx412 *kmb_imx412)
{
	int ret = 0;

	dev_dbg(kmb_imx412->dev, "V4L2_CID_SYNC_START");

	switch (kmb_imx412->sync_type) {
	case KMB_SYNC_TYPE_HW:
		ret = kmb_imx412_write_reg(kmb_imx412,
					   KMB_IMX412_REG_MODE_SELECT,
					   1, KMB_IMX412_MODE_STREAMING);
		if (ret)
			dev_err(kmb_imx412->dev, "Failed to start streaming!");
		break;
	case KMB_SYNC_TYPE_SW:
		if (kmb_imx412->sync_mode == KMB_SYNC_MAIN) {
			ret =
			kmb_imx412_i2c_broadcast(kmb_imx412,
						 KMB_IMX412_REG_MODE_SELECT,
						 1, KMB_IMX412_MODE_STREAMING);
			if (ret)
				dev_err(kmb_imx412->dev,
					"Failed to broadcast start streaming!");
		}
		break;
	default:
		ret = -EINVAL;
		dev_err(kmb_imx412->dev, "Invalid sync type!");
	}

	return ret;
}

/**
 * kmb_imx412_set_ctrl - Set subdevice control. Supported controls:
 *                       V4L2_CID_ANALOGUE_GAIN
 *                       V4L2_CID_DIGITAL_GAIN
 *                       V4L2_CID_EXPOSURE
 *                       All three controls are in one cluster.
 *
 * @ctrl: pointer to v4l2_ctrl structure
 *
 * Return: 0 if successful
 */
static int kmb_imx412_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct kmb_imx412 *kmb_imx412 =
		container_of(ctrl->handler, struct kmb_imx412, ctrl_handler);
	u32 exposure[3] = {0};
	u32 analog_gain[3] = {0};
	u32 digital_gain[3] = {0};
	u32 lpfr = 0;
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_SYNC_MODE:
		kmb_imx412->sync_mode = ctrl->val;
		dev_dbg(kmb_imx412->dev, "V4L2_CID_SYNC_MODE %d", ctrl->val);
		return 0;
	case V4L2_CID_SYNC_START:
		if (kmb_imx412->sync_mode == KMB_SYNC_NONE)
			return -EINVAL;

		ret = kmb_imx412_sync_start(kmb_imx412);
		return ret;
	case V4L2_CID_SYNC_TYPE:
		kmb_imx412->sync_type = ctrl->val;

		dev_dbg(kmb_imx412->dev, "V4L2_CID_SYNC_TYPE %d", ctrl->val);
		return 0;
	}

	/* Set exposure and gain only if sensor is in power on state */
	if (!pm_runtime_get_if_in_use(kmb_imx412->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		/* Handle the cluster for both controls */
		exposure[0] = ctrl->val;
		analog_gain[0] = kmb_imx412->again_ctrl->val;
		digital_gain[0] = kmb_imx412->dgain_ctrl->val;

		exposure[1] = kmb_imx412->exp_ctrl1->val;
		analog_gain[1] = kmb_imx412->again_ctrl1->val;
		digital_gain[1] = kmb_imx412->dgain_ctrl1->val;

		exposure[2] = kmb_imx412->exp_ctrl2->val;
		analog_gain[2] = kmb_imx412->again_ctrl2->val;
		digital_gain[2] = kmb_imx412->dgain_ctrl2->val;

		break;
	default:
		dev_err(kmb_imx412->dev, "Invalid control %x", ctrl->id);
		ret = -EINVAL;
		goto error_pm_runtime_put;
	}

	lpfr = kmb_imx412->lpfr;

	switch (kmb_imx412->streaming_mode) {
	case KMB_IMX412_ULL:
		if (exposure[0] > lpfr - KMB_IMX412_EXPOSURE_MIN_OFFSET_TO_LPFR)
			exposure[0] = lpfr -
				KMB_IMX412_EXPOSURE_MIN_OFFSET_TO_LPFR;
		else if (exposure[0] < KMB_IMX412_EXPOSURE_MIN)
			exposure[0] = KMB_IMX412_EXPOSURE_MIN;

		dev_dbg(kmb_imx412->dev,
			"Set long exp %u again %u dgain %u lpfr %u",
			exposure[0], analog_gain[0], digital_gain[0], lpfr);
		break;
	case KMB_IMX412_2DOL_HDR:
		if (exposure[0] > KMB_IMX412_2DOL_GET_1ST_MAX_EXP(lpfr))
			exposure[0] = KMB_IMX412_2DOL_GET_1ST_MAX_EXP(lpfr);
		else if (exposure[0] < KMB_IMX412_DOL_EXPOSURE_CIT_1ST_FRM_MIN)
			exposure[0] = KMB_IMX412_DOL_EXPOSURE_CIT_1ST_FRM_MIN;
		exposure[0] = ALIGN(exposure[0], 2);

		if (exposure[1] > kmb_imx412->cur_mode->lpfr[1] -
				KMB_IMX412_DOL_EXPOSURE_CIT_2ND_FRM_MIN_OFFSET)
			exposure[1] = kmb_imx412->cur_mode->lpfr[1] -
				KMB_IMX412_DOL_EXPOSURE_CIT_2ND_FRM_MIN_OFFSET;
		else if (exposure[1] < KMB_IMX412_DOL_EXPOSURE_CIT_2ND_FRM_MIN)
			exposure[1] = KMB_IMX412_DOL_EXPOSURE_CIT_2ND_FRM_MIN;
		exposure[1] = ALIGN(exposure[1], 2);

		dev_dbg(kmb_imx412->dev,
			"Set long exp %u again %u dgain %u lpfr %u",
			exposure[0], analog_gain[0], digital_gain[0], lpfr);

		dev_dbg(kmb_imx412->dev,
			"Set short exp %u again %u dgain %u",
			exposure[1], analog_gain[1], digital_gain[1]);
		break;
	case KMB_IMX412_3DOL_HDR:
		if (exposure[0] > KMB_IMX412_3DOL_GET_1ST_MAX_EXP(lpfr))
			exposure[0] = KMB_IMX412_3DOL_GET_1ST_MAX_EXP(lpfr);
		else if (exposure[0] < KMB_IMX412_DOL_EXPOSURE_CIT_1ST_FRM_MIN)
			exposure[0] = KMB_IMX412_DOL_EXPOSURE_CIT_1ST_FRM_MIN;
		exposure[0] = ALIGN(exposure[0], 2);

		if (exposure[1] > kmb_imx412->cur_mode->lpfr[1] -
				KMB_IMX412_DOL_EXPOSURE_CIT_2ND_FRM_MIN_OFFSET)
			exposure[1] = kmb_imx412->cur_mode->lpfr[1] -
				KMB_IMX412_DOL_EXPOSURE_CIT_2ND_FRM_MIN_OFFSET;
		else if (exposure[1] < KMB_IMX412_DOL_EXPOSURE_CIT_2ND_FRM_MIN)
			exposure[1] = KMB_IMX412_DOL_EXPOSURE_CIT_2ND_FRM_MIN;
		exposure[1] = ALIGN(exposure[1], 2);

		if (exposure[2] > kmb_imx412->cur_mode->lpfr[2] -
				kmb_imx412->cur_mode->lpfr[1] -
				KMB_IMX412_DOL_EXPOSURE_CIT_3RD_FRM_MIN_OFFSET)
			exposure[2] = kmb_imx412->cur_mode->lpfr[2] -
				kmb_imx412->cur_mode->lpfr[1] -
				KMB_IMX412_DOL_EXPOSURE_CIT_3RD_FRM_MIN_OFFSET;
		else if (exposure[2] < KMB_IMX412_DOL_EXPOSURE_CIT_3RD_FRM_MIN)
			exposure[2] = KMB_IMX412_DOL_EXPOSURE_CIT_3RD_FRM_MIN;
		exposure[2] = ALIGN(exposure[2], 2);

		dev_dbg(kmb_imx412->dev,
			"Set long exp %u again %u dgain %u lpfr %u",
			exposure[0], analog_gain[0], digital_gain[0], lpfr);

		dev_dbg(kmb_imx412->dev,
			"Set middle exp %u again %u dgain %u",
			exposure[1], analog_gain[1], digital_gain[1]);

		dev_dbg(kmb_imx412->dev,
			"Set short exp %u again %u dgain %u",
			exposure[2], analog_gain[2], digital_gain[2]);
		break;
	default:
		dev_err(kmb_imx412->dev, "Invalid sensor mode %d",
			kmb_imx412->streaming_mode);
		ret = -EINVAL;
		goto error_pm_runtime_put;
	}

	ret = kmb_imx412_write_reg(kmb_imx412, KMB_IMX412_REG_HOLD, 1, 1);
	if (ret)
		goto error_pm_runtime_put;

	ret = kmb_imx412_write_reg(kmb_imx412, KMB_IMX412_REG_LPFR,
				   2, lpfr);
	if (ret)
		goto error_release_group_hold;

	switch (kmb_imx412->streaming_mode) {
	case KMB_IMX412_ULL:
		ret = kmb_imx412_write_reg(kmb_imx412,
					KMB_IMX412_REG_EXPOSURE_CIT,
					2,
					exposure[0]);
		if (ret)
			goto error_release_group_hold;


		ret = kmb_imx412_write_reg(kmb_imx412, KMB_IMX412_REG_AGAIN,
					   2, analog_gain[0]);
		if (ret)
			goto error_release_group_hold;

		ret = kmb_imx412_write_reg(kmb_imx412, KMB_IMX412_REG_DGAIN,
					   2, digital_gain[0]);
		if (ret)
			goto error_release_group_hold;

		break;
	case KMB_IMX412_2DOL_HDR:
		ret = kmb_imx412_write_reg(kmb_imx412,
					KMB_IMX412_REG_DOL_EXPOSURE_CIT_1ST_FRM,
					2,
					exposure[0]);
		if (ret)
			goto error_release_group_hold;

		ret = kmb_imx412_write_reg(kmb_imx412,
					KMB_IMX412_REG_DOL_EXPOSURE_CIT_2ND_FRM,
					2,
					exposure[1]);
		if (ret)
			goto error_release_group_hold;

		ret = kmb_imx412_write_reg(kmb_imx412,
				KMB_IMX412_REG_DOL_OFFSET_2ND_FRM,
				2,
				kmb_imx412->cur_mode->lpfr[1]);
		if (ret)
			goto error_release_group_hold;

		ret = kmb_imx412_write_reg(kmb_imx412,
					KMB_IMX412_REG_DOL_ANA_GAIN_1ST_FRM,
					2,
					analog_gain[0]);
		if (ret)
			goto error_release_group_hold;

		ret = kmb_imx412_write_reg(kmb_imx412,
					KMB_IMX412_REG_DOL_ANA_GAIN_2ND_FRM,
					2,
					analog_gain[1]);
		if (ret)
			goto error_release_group_hold;

		ret = kmb_imx412_write_reg(kmb_imx412,
					   KMB_IMX412_REG_DOL_DIG_GAIN_1ST_FRM,
					   2, digital_gain[0]);
		if (ret)
			goto error_release_group_hold;

		ret = kmb_imx412_write_reg(kmb_imx412,
					   KMB_IMX412_REG_DOL_DIG_GAIN_2ND_FRM,
					   2, digital_gain[1]);
		if (ret)
			goto error_release_group_hold;
		break;
	case KMB_IMX412_3DOL_HDR:
		ret = kmb_imx412_write_reg(kmb_imx412,
					KMB_IMX412_REG_DOL_EXPOSURE_CIT_1ST_FRM,
					2,
					exposure[0]);
		if (ret)
			goto error_release_group_hold;

		ret = kmb_imx412_write_reg(kmb_imx412,
					KMB_IMX412_REG_DOL_EXPOSURE_CIT_2ND_FRM,
					2,
					exposure[1]);
		if (ret)
			goto error_release_group_hold;

		ret = kmb_imx412_write_reg(kmb_imx412,
					KMB_IMX412_REG_DOL_EXPOSURE_CIT_3RD_FRM,
					2,
					exposure[2]);
		if (ret)
			goto error_release_group_hold;

		ret = kmb_imx412_write_reg(kmb_imx412,
				KMB_IMX412_REG_DOL_OFFSET_2ND_FRM,
				2,
				kmb_imx412->cur_mode->lpfr[1]);
		if (ret)
			goto error_release_group_hold;

		ret = kmb_imx412_write_reg(kmb_imx412,
				KMB_IMX412_REG_DOL_OFFSET_3RD_FRM,
				2,
				kmb_imx412->cur_mode->lpfr[2]);
		if (ret)
			goto error_release_group_hold;

		ret = kmb_imx412_write_reg(kmb_imx412,
					KMB_IMX412_REG_DOL_ANA_GAIN_1ST_FRM,
					2,
					analog_gain[0]);
		if (ret)
			goto error_release_group_hold;

		ret = kmb_imx412_write_reg(kmb_imx412,
					KMB_IMX412_REG_DOL_ANA_GAIN_2ND_FRM,
					2,
					analog_gain[1]);
		if (ret)
			goto error_release_group_hold;

		ret = kmb_imx412_write_reg(kmb_imx412,
					KMB_IMX412_REG_DOL_ANA_GAIN_3RD_FRM,
					2,
					analog_gain[2]);
		if (ret)
			goto error_release_group_hold;

		ret = kmb_imx412_write_reg(kmb_imx412,
					   KMB_IMX412_REG_DOL_DIG_GAIN_1ST_FRM,
					   2, digital_gain[0]);
		if (ret)
			goto error_release_group_hold;

		ret = kmb_imx412_write_reg(kmb_imx412,
					   KMB_IMX412_REG_DOL_DIG_GAIN_2ND_FRM,
					   2, digital_gain[1]);
		if (ret)
			goto error_release_group_hold;

		ret = kmb_imx412_write_reg(kmb_imx412,
					   KMB_IMX412_REG_DOL_DIG_GAIN_3RD_FRM,
					   2, digital_gain[2]);
		if (ret)
			goto error_release_group_hold;
		break;
	}
	kmb_imx412_write_reg(kmb_imx412, KMB_IMX412_REG_HOLD, 1, 0);

	pm_runtime_put(kmb_imx412->dev);

	return 0;

error_release_group_hold:
	kmb_imx412_write_reg(kmb_imx412, KMB_IMX412_REG_HOLD, 1, 0);
error_pm_runtime_put:
	pm_runtime_put(kmb_imx412->dev);
	return ret;
}

/* V4l2 subdevice control ops*/
static const struct v4l2_ctrl_ops kmb_imx412_ctrl_ops = {
	.s_ctrl = kmb_imx412_set_ctrl,
};

static const struct v4l2_ctrl_config sync_mode = {
	.ops = &kmb_imx412_ctrl_ops,
	.id = V4L2_CID_SYNC_MODE,
	.name = "V4L2_CID_SYNC_MODE",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = KMB_SYNC_NONE,
	.max = KMB_SYNC_AUX,
	.def = KMB_SYNC_NONE,
	.step = 1,
};

static const struct v4l2_ctrl_config sync_start = {
	.ops = &kmb_imx412_ctrl_ops,
	.id = V4L2_CID_SYNC_START,
	.name = "V4L2_CID_SYNC_START",
	.type = V4L2_CTRL_TYPE_BUTTON,
};

static const struct v4l2_ctrl_config sync_type = {
	.ops = &kmb_imx412_ctrl_ops,
	.id = V4L2_CID_SYNC_TYPE,
	.name = "V4L2_CID_SYNC_TYPE",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = KMB_SYNC_TYPE_HW,
	.max = KMB_SYNC_TYPE_SW,
	.def = KMB_SYNC_TYPE_HW,
	.step = 1,
};

/**
 * kmb_imx412_parse_mipi_lanes - Parse mipi number of data lanes from dt
 * @kmb_imx412: pointer to kmb camera device
 *
 * Return: 0 if successful
 */
static inline int kmb_imx412_parse_mipi_lanes(struct kmb_imx412 *kmb_imx412)
{
	struct fwnode_handle *fwnode = NULL;
	struct v4l2_fwnode_endpoint ep_data;
	struct fwnode_handle *cam_fwnode = dev_fwnode(kmb_imx412->dev);
	int ret;

	fwnode = fwnode_graph_get_next_endpoint(cam_fwnode, fwnode);

	memset(&ep_data, 0, sizeof(ep_data));
	ep_data.bus_type = V4L2_MBUS_CSI2_DPHY;
	ret = v4l2_fwnode_endpoint_parse(fwnode, &ep_data);
	if (ret) {
		dev_err(kmb_imx412->dev, "No endpoint to parse in this fwnode");
		return -ENOENT;
	}

	kmb_imx412->num_lanes = ep_data.bus.mipi_csi2.num_data_lanes;
	dev_dbg(kmb_imx412->dev, "num_data_lanes %d\n", kmb_imx412->num_lanes);

	return 0;
}

static const struct v4l2_ctrl_config again_short = {
	.ops = &kmb_imx412_ctrl_ops,
	.id = V4L2_CID_ANALOGUE_GAIN_SHORT,
	.name = "V4L2_CID_ANALOGUE_GAIN_SHORT",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = KMB_IMX412_REG_DOL_ANA_GAIN_2ND_FRM_MIN,
	.max = KMB_IMX412_REG_DOL_ANA_GAIN_2ND_FRM_MAX,
	.def = KMB_IMX412_REG_DOL_ANA_GAIN_2ND_FRM_DEFAULT,
	.step = KMB_IMX412_REG_DOL_ANA_GAIN_2ND_FRM_STEP,
	.menu_skip_mask = 0,
};

static const struct v4l2_ctrl_config dgain_short = {
	.ops = &kmb_imx412_ctrl_ops,
	.id = V4L2_CID_DIGITAL_GAIN_SHORT,
	.name = "V4L2_CID_DIGITAL_GAIN_SHORT",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = KMB_IMX412_REG_DOL_DIG_GAIN_2ND_FRM_MIN,
	.max = KMB_IMX412_REG_DOL_DIG_GAIN_2ND_FRM_MAX,
	.def = KMB_IMX412_REG_DOL_DIG_GAIN_2ND_FRM_DEFAULT,
	.step = KMB_IMX412_REG_DOL_DIG_GAIN_2ND_FRM_STEP,
	.menu_skip_mask = 0,
};

static const struct v4l2_ctrl_config exposure_short = {
	.ops = &kmb_imx412_ctrl_ops,
	.id = V4L2_CID_EXPOSURE_SHORT,
	.name = "V4L2_CID_EXPOSURE_SHORT",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = KMB_IMX412_DOL_EXPOSURE_CIT_1ST_FRM_MIN,
	.max = KMB_IMX412_REG_MAX,
	.def = KMB_IMX412_DOL_EXPOSURE_CIT_1ST_FRM_DEFAULT,
	.step = KMB_IMX412_DOL_EXPOSURE_CIT_1ST_FRM_STEP,
	.menu_skip_mask = 0,
};

static const struct v4l2_ctrl_config again_very_short = {
	.ops = &kmb_imx412_ctrl_ops,
	.id = V4L2_CID_ANALOGUE_GAIN_VERY_SHORT,
	.name = "V4L2_CID_ANALOGUE_GAIN_VERY_SHORT",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = KMB_IMX412_REG_DOL_ANA_GAIN_3RD_FRM_MIN,
	.max = KMB_IMX412_REG_DOL_ANA_GAIN_3RD_FRM_MAX,
	.def = KMB_IMX412_REG_DOL_ANA_GAIN_3RD_FRM_DEFAULT,
	.step = KMB_IMX412_REG_DOL_ANA_GAIN_3RD_FRM_STEP,
	.menu_skip_mask = 0,
};

static const struct v4l2_ctrl_config dgain_very_short = {
	.ops = &kmb_imx412_ctrl_ops,
	.id = V4L2_CID_DIGITAL_GAIN_VERY_SHORT,
	.name = "V4L2_CID_DIGITAL_GAIN_VERY_SHORT",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = KMB_IMX412_REG_DOL_DIG_GAIN_3RD_FRM_MIN,
	.max = KMB_IMX412_REG_DOL_DIG_GAIN_3RD_FRM_MAX,
	.def = KMB_IMX412_REG_DOL_DIG_GAIN_3RD_FRM_DEFAULT,
	.step = KMB_IMX412_REG_DOL_DIG_GAIN_3RD_FRM_STEP,
	.menu_skip_mask = 0,
};

static const struct v4l2_ctrl_config exposure_very_short = {
	.ops = &kmb_imx412_ctrl_ops,
	.id = V4L2_CID_EXPOSURE_VERY_SHORT,
	.name = "V4L2_CID_EXPOSURE_VERY_SHORT",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = KMB_IMX412_DOL_EXPOSURE_CIT_3RD_FRM_MIN,
	.max = KMB_IMX412_REG_MAX,
	.def = KMB_IMX412_DOL_EXPOSURE_CIT_3RD_FRM_DEFAULT,
	.step = KMB_IMX412_DOL_EXPOSURE_CIT_3RD_FRM_STEP,
	.menu_skip_mask = 0,
};

static const struct v4l2_ctrl_config row_time = {
	.ops = &kmb_imx412_ctrl_ops,
	.id = V4L2_CID_ROW_TIME_NS,
	.name = "V4L2_CID_ROW_TIME_NS",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = KMB_IMX412_REG_MIN,
	.max = KMB_IMX412_REG_MAX,
	.def = KMB_IMX412_REG_MIN,
	.step = 1,
	.menu_skip_mask = 0,
};

/**
 * kmb_imx412_get_camera_mode_by_fmt - Get the most appropriate camera
 *         mode that meets the code and resolution criteria
 * @kmb_imx412: pointer to kmb_imx412 device
 * @code: media bus format code
 * @width: frame width
 * @height: frame height
 *
 * Return: pointer to the most appropriate camera mode
 */
static const struct kmb_imx412_mode *
kmb_imx412_get_camera_mode_by_fmt(struct kmb_imx412 *kmb_imx412, u32 code,
				  u32 width, u32 height)
{
	const struct kmb_imx412_mode *mode = kmb_imx412->supported_modes;
	int i;

	for (i = 0; i < kmb_imx412->num_supported_modes; i++) {
		if (mode[i].code == code && mode[i].width == width &&
		    mode[i].height == height)
			return &mode[i];
	}

	return NULL;
}

/**
 * kmb_imx412_filter_supported_modes - Filter supported sensor modes
 * @kmb_imx412: pointer to kmb_imx412 device
 *
 * Filter supported sensor modes based on number of CSI-2 lanes detected
 *
 * Return: 0 if successful
 */
static int kmb_imx412_filter_supported_modes(struct kmb_imx412 *kmb_imx412)
{
	static const struct kmb_imx412_mode *modes;
	int num_modes;
	int i;

	kmb_imx412->supported_modes = NULL;

	switch (kmb_imx412->streaming_mode) {
	case KMB_IMX412_2DOL_HDR:
		modes = hdr_2dol_supported_modes;
		num_modes = ARRAY_SIZE(hdr_2dol_supported_modes);
		break;
	case KMB_IMX412_3DOL_HDR:
		modes = hdr_3dol_supported_modes;
		num_modes = ARRAY_SIZE(hdr_3dol_supported_modes);
		break;
	default:
		modes = supported_modes;
		num_modes = ARRAY_SIZE(supported_modes);
		break;
	}

	for (i = 0; i < num_modes; i++) {
		if (kmb_imx412->num_lanes >= modes[i].num_lanes) {
			kmb_imx412->supported_modes = &modes[i];
			break;
		}
	}

	kmb_imx412->num_supported_modes = num_modes - i;

	if (!kmb_imx412->supported_modes || !kmb_imx412->num_supported_modes)
		return -EINVAL;

	return 0;
}

/**
 * kmb_imx412_select_camera_mode - Select the most appropriate camera mode
 * @kmb_imx412: pointer to kmb_imx412 device
 * @fmt: V4L2 sub-device format need to be set
 *
 * Select camera mode with the nearest resolution
 *
 * Return: pointer to the most appropriate camera mode
 */
static const struct kmb_imx412_mode *
kmb_imx412_select_camera_mode(struct kmb_imx412 *kmb_imx412,
			      struct v4l2_subdev_format *fmt)
{
	int ret;

	switch (fmt->format.reserved[0]) {
	case 3:
		kmb_imx412->streaming_mode = KMB_IMX412_3DOL_HDR;
		break;
	case 2:
		kmb_imx412->streaming_mode = KMB_IMX412_2DOL_HDR;
		break;
	default:
		kmb_imx412->streaming_mode = KMB_IMX412_ULL;
		break;
	}

	ret = kmb_imx412_filter_supported_modes(kmb_imx412);
	if (ret < 0)
		return NULL;

	return v4l2_find_nearest_size(kmb_imx412->supported_modes,
		kmb_imx412->num_supported_modes,
		width, height, fmt->format.width,
		fmt->format.height);
}

/**
 * kmb_imx412_enum_mbus_code - Enumerate V4L2 sub-device mbus codes
 * @sd: pointer to imx412 V4L2 sub-device structure
 * @cfg: V4L2 sub-device pad configuration
 * @fsize: V4L2 sub-device size enumeration need to be filled
 *
 * Return: 0 if successful
 */
static int kmb_imx412_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct kmb_imx412 *kmb_imx412 = to_kmb_imx412(sd);

	if (code->index >= kmb_imx412->num_supported_modes)
		return -EINVAL;

	mutex_lock(&kmb_imx412->mutex);
	code->code = kmb_imx412->supported_modes[code->index].code;
	mutex_unlock(&kmb_imx412->mutex);

	return 0;
}

/**
 * kmb_imx412_get_mode_from_code - Looks for sensor mode with mbus code
 *                                 being idx-th with that code
 * @kmb_imx412: pointer to kmb_imx412 device
 * @code: mbus code
 * @idx: n-th in order sensor mode with mbus code
 *
 * Return: pointer to sensor mode
 */
static const struct kmb_imx412_mode *
kmb_imx412_get_mode_from_code(const struct kmb_imx412 *kmb_imx412,
			      const u32 code, u32 idx)
{
	const struct kmb_imx412_mode *modes = kmb_imx412->supported_modes;
	u32 mode_cnt = 0;
	u32 i;

	for (i = 0; i < kmb_imx412->num_supported_modes; ++i) {
		if (code != modes[i].code)
			continue;

		if (mode_cnt == idx)
			return &modes[i];

		mode_cnt++;
	}

	return NULL;
}

/**
 * kmb_imx412_enum_frame_size - Enumerate V4L2 sub-device frame sizes
 * @sd: pointer to imx412 V4L2 sub-device structure
 * @cfg: V4L2 sub-device pad configuration
 * @fsize: V4L2 sub-device size enumeration need to be filled
 *
 * Return: 0 if successful
 */
static int kmb_imx412_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *cfg,
				  struct v4l2_subdev_frame_size_enum *fsize)
{
	struct kmb_imx412 *kmb_imx412 = to_kmb_imx412(sd);
	const struct kmb_imx412_mode *mode;

	if (fsize->index >= kmb_imx412->num_supported_modes)
		return -EINVAL;

	mutex_lock(&kmb_imx412->mutex);

	mode = kmb_imx412_get_mode_from_code(kmb_imx412, fsize->code,
					     fsize->index);
	if (!mode) {
		mutex_unlock(&kmb_imx412->mutex);
		return -EINVAL;
	}

	fsize->min_width = mode->width;
	fsize->max_width = fsize->min_width;
	fsize->min_height = mode->height;
	fsize->max_height = fsize->min_height;

	mutex_unlock(&kmb_imx412->mutex);
	return 0;
}

/**
 * kmb_imx412_enum_frame_interval - Enumerate V4L2 sub-device frame intervals
 * @sd: pointer to imx412 V4L2 sub-device structure
 * @cfg: V4L2 sub-device pad configuration
 * @fsize: V4L2 sub-device size enumeration need to be filled
 *
 * callback for VIDIOC_SUBDEV_ENUM_FRAME_INTERVAL()
 *
 * Return: 0 if successful
 */
static int
kmb_imx412_enum_frame_interval(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *cfg,
			       struct v4l2_subdev_frame_interval_enum *fie)
{
	struct kmb_imx412 *kmb_imx412 = to_kmb_imx412(sd);
	const struct kmb_imx412_mode *mode;
	int fps;
	int ret = 0;

	if (fie->pad)
		return -EINVAL;

	mutex_lock(&kmb_imx412->mutex);

	if (fie->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		if (fie->code != kmb_imx412->cur_mode->code ||
		    fie->width != kmb_imx412->cur_mode->width ||
		    fie->height != kmb_imx412->cur_mode->height) {
			ret = -EINVAL;
			goto exit_unlock;
		}

		mode = kmb_imx412->cur_mode;
	} else {
		mode = kmb_imx412_get_camera_mode_by_fmt(kmb_imx412,
				fie->code, fie->width, fie->height);
		if (!mode) {
			ret = -EINVAL;
			goto exit_unlock;
		}
	}

	fps = mode->fps.step * fie->index;
	fie->interval.numerator = KMB_IMX412_FPS_NUMERATOR;
	fie->interval.denominator = fps;

	if (fps < mode->fps.min) {
		fie->interval.denominator = mode->fps.min;
	} else if (fps > mode->fps.max) {
		ret = -EINVAL;
		goto exit_unlock;
	}

	dev_dbg(kmb_imx412->dev, "Enum FPS %d/%d",
		fie->interval.numerator,
		fie->interval.denominator);

exit_unlock:
	mutex_unlock(&kmb_imx412->mutex);
	return ret;
}

/**
 * kmb_imx412_fill_pad_format - Fill subdevice pad format
 *                              from selected sensor mode
 * @kmb_imx412: pointer to kmb_imx412 device
 * @mode: Pointer to kmb_imx412_mode sensor mode
 * @fmt: V4L2 sub-device format need to be filled
 *
 * Return: none
 */
static void kmb_imx412_fill_pad_format(struct kmb_imx412 *kmb_imx412,
				       const struct kmb_imx412_mode *mode,
				       struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.code = mode->code;
	fmt->format.field = V4L2_FIELD_NONE;
	if (kmb_imx412->streaming_mode == KMB_IMX412_2DOL_HDR)
		fmt->format.reserved[0] = 2;
	else if (kmb_imx412->streaming_mode == KMB_IMX412_3DOL_HDR)
		fmt->format.reserved[0] = 3;
}

/**
 * kmb_imx412_skip_top_lines - Skip top lines containing metadata
 * @sd: pointer to imx412 V4L2 sub-device structure
 * @lines: number of lines to be skipped
 *
 * Return: 0 if successful
 */
static int kmb_imx412_skip_top_lines(struct v4l2_subdev *sd, u32 *lines)
{
	struct kmb_imx412 *kmb_imx412 = to_kmb_imx412(sd);

	*lines = kmb_imx412->cur_mode->skip_lines;

	return 0;
}

/**
 * kmb_imx412_update_controls - Update control ranges based on streaming mode
 * @kmb_imx412: pointer to kmb_imx412 device
 * @mode: pointer to kmb_imx412_mode sensor mode
 *
 * Return: none
 */
static void kmb_imx412_update_controls(struct kmb_imx412 *kmb_imx412,
					const struct kmb_imx412_mode *mode)
{
	switch (kmb_imx412->streaming_mode) {
	case KMB_IMX412_ULL:
		__v4l2_ctrl_s_ctrl(kmb_imx412->link_freq_ctrl,
				   mode->link_freq_idx);
		__v4l2_ctrl_s_ctrl(kmb_imx412->vblank_ctrl,
				   kmb_imx412->lpfr - mode->height);
		__v4l2_ctrl_s_ctrl(kmb_imx412->hblank_ctrl,
				   mode->ppln - mode->width);
		__v4l2_ctrl_s_ctrl(kmb_imx412->row_time_ctrl,
				   mode->row_time);
		__v4l2_ctrl_modify_range(kmb_imx412->pclk_ctrl,
					mode->pclk, mode->pclk,
					1, mode->pclk);
		__v4l2_ctrl_modify_range(kmb_imx412->exp_ctrl,
					KMB_IMX412_EXPOSURE_MIN,
					kmb_imx412->lpfr -
					KMB_IMX412_EXPOSURE_MIN_OFFSET_TO_LPFR,
					1, KMB_IMX412_EXPOSURE_DEFAULT);
		break;
	case KMB_IMX412_2DOL_HDR:
		__v4l2_ctrl_s_ctrl(kmb_imx412->link_freq_ctrl,
				   mode->link_freq_idx);
		__v4l2_ctrl_s_ctrl(kmb_imx412->vblank_ctrl,
				   kmb_imx412->lpfr - mode->height);
		__v4l2_ctrl_s_ctrl(kmb_imx412->hblank_ctrl,
				   mode->ppln - mode->width);
		__v4l2_ctrl_s_ctrl(kmb_imx412->row_time_ctrl,
				   mode->row_time);
		__v4l2_ctrl_modify_range(kmb_imx412->pclk_ctrl,
					mode->pclk,
					mode->pclk,
					1,
					mode->pclk);
		__v4l2_ctrl_modify_range(kmb_imx412->exp_ctrl,
				KMB_IMX412_DOL_EXPOSURE_CIT_1ST_FRM_MIN,
				mode->lpfr[0] -
				KMB_IMX412_DOL_EXPOSURE_CIT_1ST_FRM_MIN_OFFSET,
				KMB_IMX412_DOL_EXPOSURE_CIT_1ST_FRM_STEP,
				KMB_IMX412_DOL_EXPOSURE_CIT_1ST_FRM_DEFAULT);
		__v4l2_ctrl_modify_range(kmb_imx412->exp_ctrl1,
				KMB_IMX412_DOL_EXPOSURE_CIT_2ND_FRM_MIN,
				mode->lpfr[1] -
				KMB_IMX412_DOL_EXPOSURE_CIT_2ND_FRM_MIN_OFFSET,
				KMB_IMX412_DOL_EXPOSURE_CIT_2ND_FRM_STEP,
				KMB_IMX412_DOL_EXPOSURE_CIT_2ND_FRM_DEFAULT);
		break;
	case KMB_IMX412_3DOL_HDR:
		__v4l2_ctrl_s_ctrl(kmb_imx412->link_freq_ctrl,
				   mode->link_freq_idx);
		__v4l2_ctrl_s_ctrl(kmb_imx412->vblank_ctrl,
				   kmb_imx412->lpfr - mode->height);
		__v4l2_ctrl_s_ctrl(kmb_imx412->hblank_ctrl,
				   mode->ppln - mode->width);
		__v4l2_ctrl_s_ctrl(kmb_imx412->row_time_ctrl,
				   mode->row_time);
		__v4l2_ctrl_modify_range(kmb_imx412->pclk_ctrl,
					mode->pclk,
					mode->pclk,
					1,
					mode->pclk);
		__v4l2_ctrl_modify_range(kmb_imx412->exp_ctrl,
				KMB_IMX412_DOL_EXPOSURE_CIT_1ST_FRM_MIN,
				mode->lpfr[0] -
				KMB_IMX412_DOL_EXPOSURE_CIT_1ST_FRM_MIN_OFFSET,
				KMB_IMX412_DOL_EXPOSURE_CIT_1ST_FRM_STEP,
				KMB_IMX412_DOL_EXPOSURE_CIT_1ST_FRM_DEFAULT);
		__v4l2_ctrl_modify_range(kmb_imx412->exp_ctrl1,
				KMB_IMX412_DOL_EXPOSURE_CIT_2ND_FRM_MIN,
				mode->lpfr[1]-
				KMB_IMX412_DOL_EXPOSURE_CIT_2ND_FRM_MIN_OFFSET,
				KMB_IMX412_DOL_EXPOSURE_CIT_2ND_FRM_STEP,
				KMB_IMX412_DOL_EXPOSURE_CIT_2ND_FRM_DEFAULT);
		__v4l2_ctrl_modify_range(kmb_imx412->exp_ctrl2,
				KMB_IMX412_DOL_EXPOSURE_CIT_3RD_FRM_MIN,
				mode->lpfr[2] - mode->lpfr[1] -
				KMB_IMX412_DOL_EXPOSURE_CIT_3RD_FRM_MIN_OFFSET,
				KMB_IMX412_DOL_EXPOSURE_CIT_3RD_FRM_STEP,
				KMB_IMX412_DOL_EXPOSURE_CIT_3RD_FRM_DEFAULT);
		break;
	default:
		pr_err("No such streaming mode");
		break;
	}
}

/**
 * kmb_imx412_get_pad_format - Get subdevice pad format
 * @sd: pointer to imx412 V4L2 sub-device structure
 * @cfg: V4L2 sub-device pad configuration
 * @fmt: V4L2 sub-device format need to be set
 *
 * Return: 0 if successful
 */
static int kmb_imx412_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *cfg,
				 struct v4l2_subdev_format *fmt)
{
	struct kmb_imx412 *kmb_imx412 = to_kmb_imx412(sd);

	mutex_lock(&kmb_imx412->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *framefmt;

		framefmt = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		fmt->format = *framefmt;
	} else {
		kmb_imx412_fill_pad_format(kmb_imx412,
					   kmb_imx412->cur_mode,
					   fmt);
	}

	mutex_unlock(&kmb_imx412->mutex);

	return 0;
}

/**
 * kmb_imx412_set_pad_format - Set subdevice pad format
 * @sd: pointer to imx412 V4L2 sub-device structure
 * @cfg: V4L2 sub-device pad configuration
 * @fmt: V4L2 sub-device format need to be set
 *
 * Return: 0 if successful
 */
static int
kmb_imx412_set_pad_format(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct kmb_imx412 *kmb_imx412 = to_kmb_imx412(sd);
	const struct kmb_imx412_mode *mode;
	struct v4l2_mbus_framefmt *framefmt;

	mutex_lock(&kmb_imx412->mutex);

	mode = kmb_imx412_select_camera_mode(kmb_imx412, fmt);
	if (!mode) {
		dev_err(sd->dev, "No camera mode was selected!");
		mutex_unlock(&kmb_imx412->mutex);
		return -EINVAL;
	}

	kmb_imx412->fps = kmb_imx412->fps ? : mode->fps.def;
	if (kmb_imx412->fps > mode->fps.max / KMB_IMX412_FPS_NUMERATOR)
		kmb_imx412->fps = mode->fps.max / KMB_IMX412_FPS_NUMERATOR;
	else if (kmb_imx412->fps < mode->fps.min / KMB_IMX412_FPS_NUMERATOR)
		kmb_imx412->fps = mode->fps.min / KMB_IMX412_FPS_NUMERATOR;

	kmb_imx412_update_fps(kmb_imx412, mode);

	kmb_imx412_fill_pad_format(kmb_imx412, mode, fmt);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		framefmt = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		*framefmt = fmt->format;
	} else {
		kmb_imx412_update_controls(kmb_imx412, mode);
		kmb_imx412->cur_mode = mode;
	}

	mutex_unlock(&kmb_imx412->mutex);

	return 0;
}

/**
 * kmb_imx412_prepare_for_broadcast - Prepare sensor registers specific for
 *                                    starting streaming through i2c broadcast
 *                                    command
 * @kmb_imx412: pointer to kmb_imx412 device
 *
 * Return: 0 if successful
 */
static int kmb_imx412_prepare_for_broadcast(struct kmb_imx412 *kmb_imx412)
{
	int ret = 0;

	ret = kmb_imx412_write_reg(kmb_imx412,
				   KMB_IMX412_AUX_ADD_EN_2ND, 1, 1);
	if (ret)
		return ret;

	if (kmb_imx412->sync_mode == KMB_SYNC_MAIN) {
		ret = kmb_imx412_write_reg(kmb_imx412,
					   KMB_IMX412_AUX_ADD_ACK_2ND, 1, 1);
		if (ret)
			return ret;
	}

	return ret;
}

/**
 * kmb_imx412_start_streaming - Start sensor stream
 * @kmb_imx412: pointer to kmb_imx412 device
 *
 * Return: 0 if successful
 */
static int kmb_imx412_start_streaming(struct kmb_imx412 *kmb_imx412)
{
	const struct kmb_imx412_reg_list *reg_list;
	int ret;

	ret = kmb_imx412_write_reg(kmb_imx412, KMB_IMX412_REG_MODE_SELECT,
				   1, KMB_IMX412_MODE_STANDBY);
	if (ret) {
		dev_err(kmb_imx412->dev, "fail to stop streaming");
		return ret;
	}

	/* Write sensor mode registers */
	reg_list = &kmb_imx412->cur_mode->reg_list;
	ret = kmb_imx412_write_regs(kmb_imx412, reg_list->regs,
				    reg_list->num_of_regs);
	if (ret) {
		dev_err(kmb_imx412->dev, "fail to write initial registers");
		return ret;
	}

	/* Setup handler will write actual exposure and gain */
	ret =  __v4l2_ctrl_handler_setup(kmb_imx412->sd.ctrl_handler);
	if (ret) {
		dev_err(kmb_imx412->dev, "fail to setup handler");
		return ret;
	}

	if (kmb_imx412->sync_type == KMB_SYNC_TYPE_SW) {
		ret = kmb_imx412_prepare_for_broadcast(kmb_imx412);
		if (ret)
			return ret;
	}

	/* Start streaming only when hw sync is not enabled  */
	if (kmb_imx412->sync_mode == KMB_SYNC_NONE) {
		ret = kmb_imx412_write_reg(kmb_imx412,
					KMB_IMX412_REG_MODE_SELECT,
					1, KMB_IMX412_MODE_STREAMING);
		if (ret) {
			dev_err(kmb_imx412->dev, "fail to start streaming");
			return ret;
		}
	}

	return 0;
}

/**
 * kmb_imx412_stop_streaming - Stop sensor stream
 * @kmb_imx412: pointer to kmb_imx412 device
 *
 * Return: 0 if successful
 */
static int kmb_imx412_stop_streaming(struct kmb_imx412 *kmb_imx412)
{
	return kmb_imx412_write_reg(kmb_imx412, KMB_IMX412_REG_MODE_SELECT,
				1, KMB_IMX412_MODE_STANDBY);
}

/**
 * kmb_imx412_set_stream - Enable sensor streaming
 * @sd: pointer to imx412 subdevice
 * @enable: Set to enable sensor streaming
 *
 * Return: 0 if successful
 */
static int kmb_imx412_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct kmb_imx412 *kmb_imx412 = to_kmb_imx412(sd);
	int ret;

	mutex_lock(&kmb_imx412->mutex);

	if (kmb_imx412->streaming == enable) {
		mutex_unlock(&kmb_imx412->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_get_sync(kmb_imx412->dev);
		if (ret)
			goto error_unlock;

		ret = kmb_imx412_start_streaming(kmb_imx412);
		if (ret)
			goto error_power_off;
	} else {
		kmb_imx412_stop_streaming(kmb_imx412);
		pm_runtime_put(kmb_imx412->dev);
	}

	kmb_imx412->streaming = enable;

	mutex_unlock(&kmb_imx412->mutex);

	return 0;

error_power_off:
	pm_runtime_put(kmb_imx412->dev);
error_unlock:
	mutex_unlock(&kmb_imx412->mutex);
	return ret;
}

/**
 * kmb_imx412_get_frame_interval - Get subdevice frame interval
 * @sd: pointer to imx412 V4L2 sub-device structure
 * @interval: V4L2 sub-device current farme interval
 *
 * Return: 0 if successful
 */
static int kmb_imx412_get_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *interval)
{
	struct kmb_imx412 *kmb_imx412 = to_kmb_imx412(sd);

	if (interval->pad)
		return -EINVAL;

	mutex_lock(&kmb_imx412->mutex);
	interval->interval.numerator = KMB_IMX412_FPS_NUMERATOR;
	interval->interval.denominator = kmb_imx412->fps;
	mutex_unlock(&kmb_imx412->mutex);

	dev_dbg(kmb_imx412->dev, "Get frame interval %d/%d",
		interval->interval.numerator,
		interval->interval.denominator);

	return 0;
}

/**
 * kmb_imx412_set_frame_interval - Set subdevice frame interval
 * @sd: pointer to imx412 V4L2 sub-device structure
 * @interval: V4L2 sub-device farme interval to be set
 *
 * Return: 0 if successful
 */
static int kmb_imx412_set_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *interval)
{
	struct kmb_imx412 *kmb_imx412 = to_kmb_imx412(sd);
	u32 fps;
	u32 fps_min = kmb_imx412->cur_mode->fps.min / KMB_IMX412_FPS_NUMERATOR;
	u32 fps_max = kmb_imx412->cur_mode->fps.max / KMB_IMX412_FPS_NUMERATOR;

	if (interval->pad)
		return -EINVAL;

	mutex_lock(&kmb_imx412->mutex);

	fps = (u32)(interval->interval.denominator /
		interval->interval.numerator);
	if (fps < fps_min) {
		interval->interval.numerator = KMB_IMX412_FPS_NUMERATOR;
		interval->interval.denominator = fps_min;
	} else if (fps > fps_max) {
		interval->interval.numerator = KMB_IMX412_FPS_NUMERATOR;
		interval->interval.denominator = fps_max;
	}

	kmb_imx412->fps = (u32) (interval->interval.denominator /
				     interval->interval.numerator);

	kmb_imx412_update_fps(kmb_imx412, kmb_imx412->cur_mode);

	kmb_imx412_update_controls(kmb_imx412, kmb_imx412->cur_mode);

	mutex_unlock(&kmb_imx412->mutex);

	dev_dbg(kmb_imx412->dev, "Set frame interval %d/%d",
		interval->interval.numerator,
		interval->interval.denominator);

	return 0;
}

/**
 * kmb_imx412_power_on - Sensor power on sequence
 * @kmb_imx412: imb_imx412 device
 *
 * Return: 0 if successful
 */
static int kmb_imx412_power_on(struct kmb_imx412 *kmb_imx412)
{
	int ret;

	/* request optional reset pin */
	kmb_imx412->reset_gpio =
		gpiod_get_optional(kmb_imx412->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(kmb_imx412->reset_gpio)) {
		ret = PTR_ERR(kmb_imx412->reset_gpio);
		dev_err(kmb_imx412->dev, "failed to get reset gpio %d", ret);
		return ret;
	}

	/* Ignore the call if reset gpio is not present */
	if (kmb_imx412->reset_gpio)
		gpiod_set_value_cansleep(kmb_imx412->reset_gpio, 1);

	ret = clk_prepare_enable(kmb_imx412->inclk);
	if (ret) {
		dev_err(kmb_imx412->dev, "fail to enable inclk\n");
		goto error_reset;
	}

	usleep_range(25000, 30000);

	return 0;

error_reset:
	if (kmb_imx412->reset_gpio) {
		gpiod_set_value_cansleep(kmb_imx412->reset_gpio, 0);
		gpiod_put(kmb_imx412->reset_gpio);
		kmb_imx412->reset_gpio = NULL;
	}

	return ret;
}

/**
 * kmb_imx412_power_off - Sensor power off sequence
 * @kmb_imx412: imb_imx412 device
 */
static void kmb_imx412_power_off(struct kmb_imx412 *kmb_imx412)
{
	/* Ignore the call if reset gpio is not present */
	if (kmb_imx412->reset_gpio)
		gpiod_set_value_cansleep(kmb_imx412->reset_gpio, 0);

	clk_disable_unprepare(kmb_imx412->inclk);

	if (kmb_imx412->reset_gpio) {
		gpiod_put(kmb_imx412->reset_gpio);
		kmb_imx412->reset_gpio = NULL;
	}
}

/**
 * kmb_imx412_detect - Detect imx412 sensor
 * @kmb_imx412: pointer to kmb_imx412 device
 *
 * Return: 0 if successful, -EIO if sensor id does not match
 */
static int kmb_imx412_detect(struct kmb_imx412 *kmb_imx412)
{
	int ret;
	u32 val;

	ret = kmb_imx412_read_reg(kmb_imx412, KMB_IMX412_REG_ID, 2, &val);
	if (ret)
		return ret;

	if (val != KMB_IMX412_ID) {
		dev_err(kmb_imx412->dev, "chip id mismatch: %x!=%x",
			KMB_IMX412_ID, val);
		return -EIO;
	}
	return 0;
}

/**
 * kmb_imx412_s_power - Set power core operation. Actual power is enabled on
 *                      set stream, this callback is used only for reset formats
 *                      and fps to default values.
 * @sd: pointer to kmb_imx412 sub-device.
 * @on: power on/off flag.
 *
 * Return: 0 if successful
 */
static int kmb_imx412_s_power(struct v4l2_subdev *sd, int on)
{
	struct kmb_imx412 *kmb_imx412 = to_kmb_imx412(sd);

	if (!on)
		return 0;

	mutex_lock(&kmb_imx412->mutex);

	/* Set default sensor mode */
	kmb_imx412->fps = 0;
	kmb_imx412->cur_mode = &supported_modes[0];
	kmb_imx412->lpfr = kmb_imx412->cur_mode->lpfr[0];

	mutex_unlock(&kmb_imx412->mutex);

	return 0;

}

/* V4l2 subdevice ops */
static struct v4l2_subdev_core_ops kmb_imx412_subdev_core_ops = {
	.s_power = kmb_imx412_s_power,
};

static const struct v4l2_subdev_video_ops kmb_imx412_video_ops = {
	.s_stream = kmb_imx412_set_stream,
	.g_frame_interval = kmb_imx412_get_frame_interval,
	.s_frame_interval = kmb_imx412_set_frame_interval,
};

static const struct v4l2_subdev_pad_ops kmb_imx412_pad_ops = {
	.enum_mbus_code = kmb_imx412_enum_mbus_code,
	.get_fmt = kmb_imx412_get_pad_format,
	.set_fmt = kmb_imx412_set_pad_format,
	.enum_frame_size = kmb_imx412_enum_frame_size,
	.enum_frame_interval = kmb_imx412_enum_frame_interval,
};

static const struct v4l2_subdev_sensor_ops kmb_imx412_sensor_ops = {
	.g_skip_top_lines = kmb_imx412_skip_top_lines,
};

static const struct v4l2_subdev_ops kmb_imx412_subdev_ops = {
	.core = &kmb_imx412_subdev_core_ops,
	.video = &kmb_imx412_video_ops,
	.pad = &kmb_imx412_pad_ops,
	.sensor = &kmb_imx412_sensor_ops,
};

static const struct media_entity_operations kmb_imx412_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops kmb_imx412_internal_ops = {
	.open = kmb_imx412_open,
};

/**
 * kmb_imx412_init_controls - Initialize sensor subdevice controls
 * @kmb_imx412: pointer to kmb_imx412 device
 *
 * Return: 0 if successful
 */
static int kmb_imx412_init_controls(struct kmb_imx412 *kmb_imx412)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &kmb_imx412->ctrl_handler;
	const struct kmb_imx412_mode *mode = kmb_imx412->cur_mode;
	u32 hblank;
	u32 vblank;
	int ret;

	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 16);
	if (ret)
		return ret;

	/* Serialize controls with sensor device */
	ctrl_hdlr->lock = &kmb_imx412->mutex;

	/* Initialize exposure and gain */
	kmb_imx412->exp_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						 &kmb_imx412_ctrl_ops,
						 V4L2_CID_EXPOSURE,
						 KMB_IMX412_EXPOSURE_MIN,
						 mode->lpfr[0],
						 KMB_IMX412_EXPOSURE_STEP,
						 KMB_IMX412_EXPOSURE_DEFAULT);

	kmb_imx412->again_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						   &kmb_imx412_ctrl_ops,
						   V4L2_CID_ANALOGUE_GAIN,
						   KMB_IMX412_AGAIN_MIN,
						   KMB_IMX412_AGAIN_MAX,
						   KMB_IMX412_AGAIN_STEP,
						   KMB_IMX412_AGAIN_DEFAULT);

	kmb_imx412->dgain_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						   &kmb_imx412_ctrl_ops,
						   V4L2_CID_DIGITAL_GAIN,
						   KMB_IMX412_DGAIN_MIN,
						   KMB_IMX412_DGAIN_MAX,
						   KMB_IMX412_DGAIN_STEP,
						   KMB_IMX412_DGAIN_DEFAULT);

	kmb_imx412->exp_ctrl1 = v4l2_ctrl_new_custom(ctrl_hdlr,
						     &exposure_short,
						     NULL);

	kmb_imx412->again_ctrl1 = v4l2_ctrl_new_custom(ctrl_hdlr,
						       &again_short,
						       NULL);

	kmb_imx412->dgain_ctrl1 = v4l2_ctrl_new_custom(ctrl_hdlr,
						       &dgain_short,
						       NULL);

	kmb_imx412->exp_ctrl2 = v4l2_ctrl_new_custom(ctrl_hdlr,
						     &exposure_very_short,
						     NULL);

	kmb_imx412->again_ctrl2 = v4l2_ctrl_new_custom(ctrl_hdlr,
						       &again_very_short,
						       NULL);

	kmb_imx412->dgain_ctrl2 = v4l2_ctrl_new_custom(ctrl_hdlr,
						       &dgain_very_short,
						       NULL);

	v4l2_ctrl_cluster(9, &kmb_imx412->exp_ctrl);

	/* Read only controls */
	kmb_imx412->pclk_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						  &kmb_imx412_ctrl_ops,
						  V4L2_CID_PIXEL_RATE,
						  mode->pclk, mode->pclk,
						  1, mode->pclk);
	if (kmb_imx412->pclk_ctrl)
		kmb_imx412->pclk_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	kmb_imx412->link_freq_ctrl = v4l2_ctrl_new_int_menu(ctrl_hdlr,
						&kmb_imx412_ctrl_ops,
						V4L2_CID_LINK_FREQ,
						ARRAY_SIZE(link_freq) - 1,
						mode->link_freq_idx, link_freq);
	if (kmb_imx412->link_freq_ctrl)
		kmb_imx412->link_freq_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank = mode->lpfr[0] - mode->height;
	kmb_imx412->vblank_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						    &kmb_imx412_ctrl_ops,
						    V4L2_CID_VBLANK,
						    KMB_IMX412_REG_MIN,
						    KMB_IMX412_REG_MAX,
						    1, vblank);
	if (kmb_imx412->vblank_ctrl)
		kmb_imx412->vblank_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	hblank = mode->ppln - mode->width;
	kmb_imx412->hblank_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						    &kmb_imx412_ctrl_ops,
						    V4L2_CID_HBLANK,
						    KMB_IMX412_REG_MIN,
						    KMB_IMX412_REG_MAX,
						    1, hblank);
	if (kmb_imx412->hblank_ctrl)
		kmb_imx412->hblank_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	kmb_imx412->row_time_ctrl = v4l2_ctrl_new_custom(ctrl_hdlr,
							 &row_time,
							 NULL);
	if (kmb_imx412->row_time_ctrl)
		kmb_imx412->row_time_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* HW sync controls */
	kmb_imx412->sync_mode_ctrl = v4l2_ctrl_new_custom(ctrl_hdlr,
							  &sync_mode,
							  NULL);
	kmb_imx412->sync_start_ctrl = v4l2_ctrl_new_custom(ctrl_hdlr,
							   &sync_start,
							   NULL);
	kmb_imx412->sync_type_ctrl = v4l2_ctrl_new_custom(ctrl_hdlr,
							  &sync_type,
							  NULL);
	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(kmb_imx412->dev, "control init failed: %d", ret);
		goto error;
	}

	kmb_imx412->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

/* --------------- probe as i2c device -------------------- */

/**
 * kmb_imx412_i2c_resume - PM resume callback
 * @dev: i2c device
 *
 * Return: 0 if successful
 */
static int kmb_imx412_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct kmb_imx412 *kmb_imx412 = to_kmb_imx412(sd);

	return kmb_imx412_power_on(kmb_imx412);
}

/**
 * kmb_imx412_i2c_suspend - PM suspend callback
 * @dev: i2c device
 *
 * Return: 0 if successful
 */
static int kmb_imx412_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct kmb_imx412 *kmb_imx412 = to_kmb_imx412(sd);

	kmb_imx412_power_off(kmb_imx412);

	return 0;
}

/**
 * kmb_imx412_i2c_probe - I2C client device binding
 * @client: pointer to i2c client device
 * @id: pointer to i2c device id
 *
 * Return: 0 if successful
 */
static int kmb_imx412_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct kmb_imx412 *kmb_imx412;
	int ret;

	kmb_imx412 = devm_kzalloc(&client->dev, sizeof(*kmb_imx412),
				  GFP_KERNEL);
	if (!kmb_imx412)
		return -ENOMEM;

	mutex_init(&kmb_imx412->mutex);

	kmb_imx412->dev = &client->dev;

	/* Initialize subdev */
	v4l2_i2c_subdev_init(&kmb_imx412->sd, client, &kmb_imx412_subdev_ops);

	/* Get sensor input clock */
	kmb_imx412->inclk = devm_clk_get(&client->dev, "inclk");
	if (IS_ERR(kmb_imx412->inclk)) {
		ret = PTR_ERR(kmb_imx412->inclk);
		dev_err(&client->dev, "could not get inclk");
		goto error_mutex_destroy;
	}

	ret = clk_set_rate(kmb_imx412->inclk, KMB_IMX412_INCLK_RATE);
	if (ret) {
		dev_err(&client->dev, "could not set inclk frequency\n");
		goto error_mutex_destroy;
	}

	ret = kmb_imx412_power_on(kmb_imx412);
	if (ret) {
		dev_err(&client->dev, "failed to power-on the sensor\n");
		goto error_mutex_destroy;
	}

	/* Check module identity */
	ret = kmb_imx412_detect(kmb_imx412);
	if (ret) {
		dev_err(&client->dev, "failed to find sensor: %d", ret);
		goto error_sensor_power_off;
	}

	/* Set default mode to max resolution */
	kmb_imx412->cur_mode = &supported_modes[0];
	kmb_imx412->fps = kmb_imx412->cur_mode->fps.def;
	kmb_imx412->lpfr = kmb_imx412->cur_mode->lpfr[0];

	ret = kmb_imx412_init_controls(kmb_imx412);
	if (ret) {
		dev_err(&client->dev, "failed to init controls: %d", ret);
		goto error_sensor_power_off;
	}

	/* Initialize subdev */
	kmb_imx412->sd.internal_ops = &kmb_imx412_internal_ops;
	kmb_imx412->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	kmb_imx412->sd.entity.ops = &kmb_imx412_subdev_entity_ops;
	kmb_imx412->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
	kmb_imx412->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&kmb_imx412->sd.entity, 1,
				     &kmb_imx412->pad);
	if (ret) {
		dev_err(&client->dev, "failed to init entity pads: %d", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&kmb_imx412->sd);
	if (ret < 0) {
		dev_err(&client->dev,
				"failed to register async subdev: %d", ret);
		goto error_media_entity;
	}

	ret = kmb_imx412_parse_mipi_lanes(kmb_imx412);
	if (ret < 0) {
		dev_err(kmb_imx412->dev, "Fail to parse device tree\n");
		goto error_media_entity;
	}

	ret = kmb_imx412_filter_supported_modes(kmb_imx412);
	if (ret < 0) {
		dev_err(kmb_imx412->dev,
				"failed to find supported mode: %d", ret);
		goto error_media_entity;
	}

	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	return 0;

error_media_entity:
	media_entity_cleanup(&kmb_imx412->sd.entity);
error_handler_free:
	v4l2_ctrl_handler_free(kmb_imx412->sd.ctrl_handler);
error_sensor_power_off:
	kmb_imx412_power_off(kmb_imx412);
error_mutex_destroy:
	mutex_destroy(&kmb_imx412->mutex);

	return ret;
}

/**
 * kmb_imx412_i2c_remove - I2C client device unbinding
 * @client: pointer to I2C client device
 *
 * Return: 0 if successful
 */
static int kmb_imx412_i2c_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct kmb_imx412 *kmb_imx412 = to_kmb_imx412(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(&client->dev);
	pm_runtime_suspended(&client->dev);

	mutex_destroy(&kmb_imx412->mutex);

	return 0;
}

static const struct dev_pm_ops kmb_imx412_i2c_pm_ops = {
	SET_RUNTIME_PM_OPS(kmb_imx412_i2c_suspend, kmb_imx412_i2c_resume, NULL)
};

static const struct i2c_device_id kmb_imx412_i2c_id_table[] = {
	{KMB_IMX412_DRV_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, kmb_imx412_i2c_id_table);

static struct i2c_driver kmb_imx412_i2c_driver = {
	.probe = kmb_imx412_i2c_probe,
	.remove = kmb_imx412_i2c_remove,
	.id_table = kmb_imx412_i2c_id_table,
	.driver = {
		.owner = THIS_MODULE,
		.pm = &kmb_imx412_i2c_pm_ops,
		.name = KMB_IMX412_DRV_NAME,
	},
};

/* --------------- probe as platform device ----------------- */

/**
 * kmb_imx412_platform_resume - PM resume callback
 * @dev: platform device
 *
 * Return: 0 if successful
 */
static int kmb_imx412_platform_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct kmb_imx412 *kmb_imx412 = platform_get_drvdata(pdev);

	return kmb_imx412_power_on(kmb_imx412);
}

/**
 * kmb_imx412_platform_suspend - PM suspend callback
 * @dev: platform device
 *
 * Return: 0 if successful
 */
static int kmb_imx412_platform_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct kmb_imx412 *kmb_imx412 = platform_get_drvdata(pdev);

	kmb_imx412_power_off(kmb_imx412);

	return 0;
}

/**
 * kmb_imx412_get_i2c_client - Get I2C client
 * @kmb_imx412: pointer to kmb_imx412 device
 * @info: i2c board info
 * @addr_list: i2c address list
 *
 * Return: handle to new i2c client if successful, NULL otherwise
 */
static struct i2c_client *
kmb_imx412_get_i2c_client(struct kmb_imx412 *kmb_imx412,
			  struct i2c_board_info *info,
			  const unsigned short *addr_list)
{
	struct i2c_adapter *i2c_adp;
	struct device_node *phandle;
	struct i2c_client *new_client;

	phandle = of_parse_phandle(kmb_imx412->dev->of_node, "i2c-bus", 0);
	if (!phandle)
		return NULL;

	i2c_adp = of_get_i2c_adapter_by_node(phandle);
	of_node_put(phandle);
	if (!i2c_adp)
		return NULL;

	new_client = i2c_new_scanned_device(i2c_adp, info, addr_list, NULL);
	i2c_put_adapter(i2c_adp);

	return IS_ERR(new_client) ? NULL : new_client;
}

/**
 * kmb_imx412_get_main_i2c_client - Get main I2C client
 * @kmb_imx412: pointer to kmb_imx412 device
 * @main_addr: pointer or address on which device was detected
 *
 * Return: none
 */
static void kmb_imx412_get_main_i2c_client(struct kmb_imx412 *kmb_imx412,
					   unsigned short *main_addr)
{
	struct i2c_board_info info = {
		I2C_BOARD_INFO("kmb-imx412-sensor-p", KMB_IMX412_MAIN_I2C)};
	const unsigned short addr_list[] = {KMB_IMX412_MAIN_I2C, 0x1A,
					    I2C_CLIENT_END};

	kmb_imx412->client = kmb_imx412_get_i2c_client(kmb_imx412, &info,
						       addr_list);

	if (kmb_imx412->client) {
		dev_dbg(kmb_imx412->dev, "Detected on i2c address %x",
			info.addr);
		*main_addr = info.addr;
	}
}

/**
 * kmb_imx412_get_broadcast_i2c_client - Get broadcast I2C client
 * @kmb_imx412: pointer to kmb_imx412 device
 *
 * Return: none
 */
static void kmb_imx412_get_broadcast_i2c_client(struct kmb_imx412 *kmb_imx412)
{
	struct i2c_board_info info = {
		I2C_BOARD_INFO("kmb-imx412-sensor-p", 0x36)};
	const unsigned short addr_list[] = {0x36, I2C_CLIENT_END};
	int ret;

	ret = kmb_imx412_write_reg(kmb_imx412,
				   KMB_IMX412_AUX_ADD_EN_2ND, 1, 1);
	if (ret)
		return;

	ret = kmb_imx412_write_reg(kmb_imx412,
				   KMB_IMX412_AUX_ADD_ACK_2ND, 1, 1);
	if (ret) {
		dev_err(kmb_imx412->dev, "failed to write add ack 2nd!");
		return;
	}

	kmb_imx412->broadcast_client = kmb_imx412_get_i2c_client(kmb_imx412,
								 &info,
								 addr_list);
}

/**
 * kmb_imx412_pdev_probe - Platform device binding
 * @pdev: pointer to platform device
 *
 * Return: 0 if successful
 */
static int kmb_imx412_pdev_probe(struct platform_device *pdev)
{
	struct kmb_imx412 *kmb_imx412;
	struct gpio_descs *detect_gpios;
	unsigned short main_addr;
	int ret;

	kmb_imx412 = devm_kzalloc(&pdev->dev, sizeof(*kmb_imx412), GFP_KERNEL);
	if (!kmb_imx412)
		return -ENOMEM;

	platform_set_drvdata(pdev, kmb_imx412);

	mutex_init(&kmb_imx412->mutex);

	kmb_imx412->dev = &pdev->dev;

	/* Initialize subdev */
	v4l2_subdev_init(&kmb_imx412->sd, &kmb_imx412_subdev_ops);
	kmb_imx412->sd.owner = pdev->dev.driver->owner;
	kmb_imx412->sd.dev = &pdev->dev;

	/* request optional detect pins */
	detect_gpios =
		gpiod_get_array_optional(&pdev->dev, "detect", GPIOD_OUT_LOW);
	if (IS_ERR(detect_gpios)) {
		ret = PTR_ERR(detect_gpios);
		dev_info(&pdev->dev, "failed to get detect gpios %d", ret);
		/* Defer the probe if detect gpios are busy */
		ret = (ret == -EBUSY) ? -EPROBE_DEFER : ret;
		goto error_mutex_destroy;
	}

	/* Get sensor input clock */
	kmb_imx412->inclk = devm_clk_get(&pdev->dev, "inclk");
	if (IS_ERR(kmb_imx412->inclk)) {
		ret = PTR_ERR(kmb_imx412->inclk);
		dev_err(&pdev->dev, "could not get inclk");
		goto error_put_detect_gpios;
	}

	ret = clk_set_rate(kmb_imx412->inclk, KMB_IMX412_INCLK_RATE);
	if (ret) {
		dev_err(&pdev->dev, "could not set inclk frequency\n");
		goto error_put_detect_gpios;
	}

	ret = kmb_imx412_power_on(kmb_imx412);
	if (ret) {
		dev_dbg(&pdev->dev, "failed to power-on the sensor %d", ret);
		/* Defer the probe if resourcess are busy */
		ret = (ret == -EBUSY) ? -EPROBE_DEFER : ret;
		goto error_put_detect_gpios;
	}

	kmb_imx412_get_main_i2c_client(kmb_imx412, &main_addr);
	if (!kmb_imx412->client) {
		dev_dbg(&pdev->dev, "failed to get i2c!");
		goto error_sensor_power_off;
	}

	v4l2_set_subdevdata(&kmb_imx412->sd, kmb_imx412->client);
	i2c_set_clientdata(kmb_imx412->client, &kmb_imx412->sd);
	v4l2_i2c_subdev_set_name(&kmb_imx412->sd, kmb_imx412->client,
				 KMB_IMX412_DRV_NAME, pdev->name);

	if (main_addr == KMB_IMX412_MAIN_I2C)
		kmb_imx412_get_broadcast_i2c_client(kmb_imx412);

	/* Check module identity */
	ret = kmb_imx412_detect(kmb_imx412);
	if (ret) {
		dev_err(&pdev->dev, "failed to find sensor: %d", ret);
		goto error_unregister_i2c_dev;
	}

	/* Set default mode to max resolution */
	kmb_imx412->cur_mode = &supported_modes[0];
	kmb_imx412->fps = kmb_imx412->cur_mode->fps.def;
	kmb_imx412->lpfr = kmb_imx412->cur_mode->lpfr[0];

	ret = kmb_imx412_init_controls(kmb_imx412);
	if (ret) {
		dev_err(&pdev->dev, "failed to init controls: %d", ret);
		goto error_unregister_i2c_dev;
	}

	/* Initialize subdev */
	kmb_imx412->sd.internal_ops = &kmb_imx412_internal_ops;
	kmb_imx412->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	kmb_imx412->sd.entity.ops = &kmb_imx412_subdev_entity_ops;
	kmb_imx412->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
	kmb_imx412->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&kmb_imx412->sd.entity, 1,
				     &kmb_imx412->pad);
	if (ret) {
		dev_err(&pdev->dev, "failed to init entity pads: %d", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&kmb_imx412->sd);
	if (ret < 0) {
		dev_err(&pdev->dev,
				"failed to register async subdev: %d", ret);
		goto error_media_entity;
	}

	ret = kmb_imx412_parse_mipi_lanes(kmb_imx412);
	if (ret < 0) {
		dev_err(kmb_imx412->dev, "Fail to parse device tree\n");
		goto error_media_entity;
	}

	ret = kmb_imx412_filter_supported_modes(kmb_imx412);
	if (ret < 0) {
		dev_err(&pdev->dev,
				"failed to find supported mode: %d", ret);
		goto error_media_entity;
	}

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_idle(&pdev->dev);

	if (detect_gpios)
		gpiod_put_array(detect_gpios);

	dev_info(&pdev->dev, "Probe success!");
	return 0;

error_media_entity:
	media_entity_cleanup(&kmb_imx412->sd.entity);
error_handler_free:
	v4l2_ctrl_handler_free(kmb_imx412->sd.ctrl_handler);
error_unregister_i2c_dev:
	if (kmb_imx412->broadcast_client)
		i2c_unregister_device(kmb_imx412->broadcast_client);
	if (kmb_imx412->client)
		i2c_unregister_device(kmb_imx412->client);
error_sensor_power_off:
	kmb_imx412_power_off(kmb_imx412);
error_put_detect_gpios:
	if (detect_gpios)
		gpiod_put_array(detect_gpios);
error_mutex_destroy:
	mutex_destroy(&kmb_imx412->mutex);

	return ret;
}

/**
 * kmb_imx412_pdev_remove - Platform device unbinding
 * @pdev: pointer to platform device
 *
 * Return: 0 if successful
 */
static int kmb_imx412_pdev_remove(struct platform_device *pdev)
{
	struct kmb_imx412 *kmb_imx412 = platform_get_drvdata(pdev);
	struct v4l2_subdev *sd = &kmb_imx412->sd;

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(&pdev->dev);
	pm_runtime_suspended(&pdev->dev);

	if (kmb_imx412->client)
		i2c_unregister_device(kmb_imx412->client);

	if (kmb_imx412->broadcast_client) {
		i2c_unregister_device(kmb_imx412->broadcast_client);
		kmb_imx412->broadcast_client = NULL;
	}

	mutex_destroy(&kmb_imx412->mutex);

	return 0;
}

static const struct dev_pm_ops kmb_imx412_platform_pm_ops = {
	SET_RUNTIME_PM_OPS(kmb_imx412_platform_suspend,
			   kmb_imx412_platform_resume, NULL)
};

static const struct of_device_id kmb_imx412_id_table[] = {
	{.compatible = "intel,kmb-imx412-sensor-p"},
	{}
};
MODULE_DEVICE_TABLE(of, kmb_imx412_id_table);

static struct platform_driver kmb_imx412_platform_driver = {
	.probe	= kmb_imx412_pdev_probe,
	.remove = kmb_imx412_pdev_remove,
	.driver = {
		.name = KMB_IMX412_DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &kmb_imx412_platform_pm_ops,
		.of_match_table = kmb_imx412_id_table,
	}
};

static int __init kmb_imx412_init(void)
{
	int ret;

	ret = i2c_add_driver(&kmb_imx412_i2c_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&kmb_imx412_platform_driver);
	if (ret)
		i2c_del_driver(&kmb_imx412_i2c_driver);

	return ret;
}

static void __exit kmb_imx412_exit(void)
{
	i2c_del_driver(&kmb_imx412_i2c_driver);
	platform_driver_unregister(&kmb_imx412_platform_driver);
}

module_init(kmb_imx412_init);
module_exit(kmb_imx412_exit);

MODULE_DESCRIPTION("KeemBay imx412 Sensor driver");
MODULE_LICENSE("GPL v2");
