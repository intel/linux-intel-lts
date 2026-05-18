// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022-2026 Intel Corporation.

#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <linux/regmap.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0)
#include <asm/unaligned.h>
#else
#include <linux/unaligned.h>
#endif
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/i2c/d4xx.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 10, 0)
#include <media/mipi-csi2.h>
#endif

/* DS5 device usage */
#define DS5_DRIVER_NAME 		"d4xx"
#define DS5_DRIVER_NAME_AWG 	"d4xx-awg"
#define DS5_DRIVER_NAME_ASR 	"d4xx-asr"
#define DS5_DRIVER_NAME_CLASS 	"d4xx-class"
#define DS5_DRIVER_NAME_DFU 	"d4xx-dfu"
#define DS5_FW_VERSION			0x030C
#define DS5_FW_BUILD			0x030E
#define DS5_DEVICE_TYPE			0x0310
#define DS5_DEVICE_TYPE_D45X		6
#define DS5_DEVICE_TYPE_D43X		5
#define DS5_DEVICE_TYPE_D46X		4

/* GMSL CSI data types */
#define GMSL_CSI_DT_YUV422_8 0x1E
#define GMSL_CSI_DT_RGB_888 0x24
#define GMSL_CSI_DT_RAW_8 0x2A
#define GMSL_CSI_DT_EMBED 0x12

/* DS5 Registers */
#define DS5_MIPI_LANE_NUMS		0x0400
#define DS5_MIPI_LANE_DATARATE	0x0402
#define DS5_MIPI_CONF_STATUS	0x0500
#define DS5_START_STOP_STREAM	0x1000
#define DS5_DEPTH_STREAM_DT		0x4000
#define DS5_DEPTH_STREAM_MD		0x4002
#define DS5_DEPTH_RES_WIDTH		0x4004
#define DS5_DEPTH_RES_HEIGHT	0x4008
#define DS5_DEPTH_FPS			0x400C
#define DS5_DEPTH_OVERRIDE		0x401C

#define DS5_RGB_STREAM_DT		0x4020
#define DS5_RGB_STREAM_MD		0x4022
#define DS5_RGB_RES_WIDTH		0x4024
#define DS5_RGB_RES_HEIGHT		0x4028
#define DS5_RGB_FPS				0x402C

#define DS5_IMU_STREAM_DT		0x4040
#define DS5_IMU_STREAM_MD		0x4042
#define DS5_IMU_RES_WIDTH		0x4044
#define DS5_IMU_RES_HEIGHT		0x4048
#define DS5_IMU_FPS				0x404C

#define DS5_IR_STREAM_DT		0x4080
#define DS5_IR_STREAM_MD		0x4082
#define DS5_IR_RES_WIDTH		0x4084
#define DS5_IR_RES_HEIGHT		0x4088
#define DS5_IR_FPS				0x408C
#define DS5_IR_OVERRIDE			0x409C

#define DS5_DEPTH_CONTROL_BASE		0x4100
#define DS5_RGB_CONTROL_BASE		0x4200
#define DS5_MANUAL_EXPOSURE_LSB		0x0000
#define DS5_MANUAL_EXPOSURE_MSB		0x0002
#define DS5_MANUAL_GAIN			0x0004
#define DS5_LASER_POWER			0x0008
#define DS5_AUTO_EXPOSURE_MODE		0x000C
#define DS5_EXPOSURE_ROI_TOP		0x0010
#define DS5_EXPOSURE_ROI_LEFT		0x0014
#define DS5_EXPOSURE_ROI_BOTTOM		0x0018
#define DS5_EXPOSURE_ROI_RIGHT		0x001C
#define DS5_MANUAL_LASER_POWER		0x0024
#define DS5_PWM_FREQUENCY		0x0028

#define DS5_CONFIG_STATUS			0x4800
#define DS5_DEPTH_CONFIG_STATUS		0x4800
#define DS5_RGB_CONFIG_STATUS		0x4802
#define DS5_IMU_CONFIG_STATUS		0x4804
#define DS5_IR_CONFIG_STATUS		0x4808

#define DS5_MIPI_SUPPORT_LINES		0x0300
#define DS5_MIPI_SUPPORT_PHY		0x0304
#define DS5_MIPI_DATARATE_MIN		0x0308
#define DS5_MIPI_DATARATE_MAX		0x030A

/* DS5 Register Values */
#define DS5_STREAM_STATUS	0x1004
#define DS5_DEPTH_STREAM_STATUS	0x1004
#define DS5_RGB_STREAM_STATUS	0x1008
#define DS5_IMU_STREAM_STATUS	0x100C
#define DS5_IR_STREAM_STATUS	0x1014

// stream status
#define DS5_STREAM_STOP			0x100
#define DS5_STREAM_START		0x200
#define DS5_STREAM_IDLE			0x1
#define DS5_STREAM_STREAMING	0x2

// config status
#define DS5_STATUS_STREAMING		0x1
#define DS5_STATUS_INVALID_DT		0x2
#define DS5_STATUS_INVALID_RES		0x4
#define DS5_STATUS_INVALID_FPS		0x8

/* DS5 Values*/
#define DS5_STREAM_DEPTH		0x0
#define DS5_STREAM_RGB			0x1
#define DS5_STREAM_IMU			0x2
#define DS5_STREAM_IR			0x4

/* const */
#define MIPI_LANE_RATE			1000
#define MAX_DEPTH_EXP			200000
#define MAX_RGB_EXP				10000
#define DEF_DEPTH_EXP			33000
#define DEF_RGB_EXP				1660

/* DFU const */
#define DFU_WAIT_RET_LEN 6
#define DS5_START_POLL_TIME	10
#define DS5_START_MAX_TIME	1000
#define DS5_START_MAX_COUNT	(DS5_START_MAX_TIME / DS5_START_POLL_TIME)
#define DFU_MAGIC_NUMBER "/0x01/0x02/0x03/0x04"
#define DFU_BLOCK_SIZE 1024
#define DS5_FRAMERATE_DEFAULT_IDX 1

/* DFU state */
#define DFU_STATE_RECOVERY 0x201

#define D4XX_LINK_FREQ_750MHZ		750000000ULL
#define D4XX_LINK_FREQ_720MHZ		720000000ULL
#define D4XX_LINK_FREQ_600MHZ		600000000ULL
#define D4XX_LINK_FREQ_576MHZ		576000000ULL
#define D4XX_LINK_FREQ_480MHZ		480000000ULL
#define D4XX_LINK_FREQ_450MHZ		450000000ULL
#define D4XX_LINK_FREQ_360MHZ		360000000ULL
#define D4XX_LINK_FREQ_300MHZ		300000000ULL
#define D4XX_LINK_FREQ_288MHZ		288000000ULL
#define D4XX_LINK_FREQ_240MHZ		240000000ULL
#define D4XX_LINK_FREQ_225MHZ		22500000ULL

/* Custom CID */
// TODO: why to use DS5_DEPTH_Y_STREAMS_DT?
#define DS5_CAMERA_CID_BASE	(V4L2_CTRL_CLASS_CAMERA | DS5_DEPTH_STREAM_DT)
#define DS5_CAMERA_CID_LOG			(DS5_CAMERA_CID_BASE+0)
#define DS5_CAMERA_CID_LASER_POWER		(DS5_CAMERA_CID_BASE+1)
#define DS5_CAMERA_CID_MANUAL_LASER_POWER	(DS5_CAMERA_CID_BASE+2)
#define DS5_CAMERA_DEPTH_CALIBRATION_TABLE_GET	(DS5_CAMERA_CID_BASE+3)
#define DS5_CAMERA_DEPTH_CALIBRATION_TABLE_SET	(DS5_CAMERA_CID_BASE+4)
#define DS5_CAMERA_COEFF_CALIBRATION_TABLE_GET	(DS5_CAMERA_CID_BASE+5)
#define DS5_CAMERA_COEFF_CALIBRATION_TABLE_SET	(DS5_CAMERA_CID_BASE+6)
#define DS5_CAMERA_CID_FW_VERSION		(DS5_CAMERA_CID_BASE+7)
#define DS5_CAMERA_CID_GVD			(DS5_CAMERA_CID_BASE+8)
#define DS5_CAMERA_CID_AE_ROI_GET		(DS5_CAMERA_CID_BASE+9)
#define DS5_CAMERA_CID_AE_ROI_SET		(DS5_CAMERA_CID_BASE+10)
#define DS5_CAMERA_CID_AE_SETPOINT_GET		(DS5_CAMERA_CID_BASE+11)
#define DS5_CAMERA_CID_AE_SETPOINT_SET		(DS5_CAMERA_CID_BASE+12)
#define DS5_CAMERA_CID_ERB			(DS5_CAMERA_CID_BASE+13)
#define DS5_CAMERA_CID_EWB			(DS5_CAMERA_CID_BASE+14)
#define DS5_CAMERA_CID_HWMC			(DS5_CAMERA_CID_BASE+15)
#define DS5_CAMERA_CID_PWM			(DS5_CAMERA_CID_BASE+22)
/* the HWMC will remain for legacy tools compatibility,
 * HWMC_RW used for UVC compatibility
 */
#define DS5_CAMERA_CID_HWMC_RW		(DS5_CAMERA_CID_BASE+32)

/* HWMC registers */
#define DS5_HWMC_DATA			0x4900
#define DS5_HWMC_STATUS			0x4904
#define DS5_HWMC_RESP_LEN		0x4908
#define DS5_HWMC_EXEC			0x490C

/* HWMC status */
#define DS5_HWMC_STATUS_OK		0
#define DS5_HWMC_STATUS_ERR		1
#define DS5_HWMC_STATUS_WIP		2
#define DS5_HWMC_BUFFER_SIZE	1024

/* HWMC const */
#define DS5_MAX_LOG_WAIT 200
#define DS5_MAX_LOG_SLEEP 10
#define DS5_MAX_LOG_POLL (DS5_MAX_LOG_WAIT / DS5_MAX_LOG_SLEEP)
#define DS5_N_CONTROLS			8

/* helper function */
#define to_ds5(_sd)			container_of(_sd, struct ds5, sd)

#define ds5_read_with_check(ds5, reg, val) \
	do { \
		if (ds5_read(ds5, reg, val)) \
			return -EINVAL; \
	} while (0)
#define ds5_raw_read_with_check(ds5, reg, val, size) \
	do { \
		if (ds5_raw_read(ds5, reg, val, size)) \
			return -EINVAL; \
	} while (0)
#define ds5_write_with_check(ds5, reg, val) \
	do { \
		if (ds5_write(ds5, reg, val)) \
			return -EINVAL; \
	} while (0)
#define ds5_raw_write_with_check(ds5, reg, val, size) \
	do { \
		if (ds5_raw_write(ds5, reg, val, size)) \
			return -EINVAL; \
	} while (0)

#define DS5_REG_SENSOR_STATE	0x5020

#define NR_OF_DS5_PADS 7
#define NR_OF_DS5_STREAMS 4

#define USE_Y

struct v4l2_mbus_framefmt ds5_ffmts[NR_OF_DS5_PADS];

enum DS5_HWMC_ERR {
	DS5_HWMC_ERR_SUCCESS = 0,
	DS5_HWMC_ERR_CMD     = -1,
	DS5_HWMC_ERR_PARAM   = -6,
	DS5_HWMC_ERR_NODATA  = -21,
};

struct ds5_reg {
	enum {
		DS5_REG_LEN_DELAY = 0,
		DS5_REG_LEN_08BIT = 1,
		DS5_REG_LEN_16BIT = 2,
	} mode;
	u16 address;
	u16 val;
};

struct ds5_reg_list {
	u32 num_of_regs;
	const struct ds5_reg *regs;
};

static const struct regmap_config ds5_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.reg_format_endian = REGMAP_ENDIAN_NATIVE,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
};
static const s64 link_freq_menu_items[] = {
	D4XX_LINK_FREQ_750MHZ,
	D4XX_LINK_FREQ_720MHZ,
	D4XX_LINK_FREQ_600MHZ,
	D4XX_LINK_FREQ_576MHZ,
	D4XX_LINK_FREQ_480MHZ,
	D4XX_LINK_FREQ_450MHZ,
	D4XX_LINK_FREQ_360MHZ,
	D4XX_LINK_FREQ_300MHZ,
	D4XX_LINK_FREQ_288MHZ,
	D4XX_LINK_FREQ_240MHZ,
	D4XX_LINK_FREQ_225MHZ,
};

struct ds5_ctrls {
	struct v4l2_ctrl_handler handler;
	struct v4l2_ctrl_handler handler_depth;
	struct v4l2_ctrl_handler handler_rgb;
	struct v4l2_ctrl_handler handler_y8;
	struct v4l2_ctrl_handler handler_imu;
	struct {
		struct v4l2_ctrl *log;
		struct v4l2_ctrl *fw_version;
		struct v4l2_ctrl *gvd;
		struct v4l2_ctrl *get_depth_calib;
		struct v4l2_ctrl *set_depth_calib;
		struct v4l2_ctrl *get_coeff_calib;
		struct v4l2_ctrl *set_coeff_calib;
		struct v4l2_ctrl *ae_roi_get;
		struct v4l2_ctrl *ae_roi_set;
		struct v4l2_ctrl *ae_setpoint_get;
		struct v4l2_ctrl *ae_setpoint_set;
		struct v4l2_ctrl *erb;
		struct v4l2_ctrl *ewb;
		struct v4l2_ctrl *hwmc;
		struct v4l2_ctrl *laser_power;
		struct v4l2_ctrl *manual_laser_power;
		struct v4l2_ctrl *auto_exp;
		struct v4l2_ctrl *exposure;
		/* in DS5 manual gain only works with manual exposure */
		struct v4l2_ctrl *gain;
		struct v4l2_ctrl *link_freq;
		struct v4l2_ctrl *query_sub_stream;
		struct v4l2_ctrl *set_sub_stream;
	};
};

enum ds5_mux_pad {
	DS5_MUX_PAD_EXTERNAL,
	DS5_MUX_PAD_DEPTH,
	DS5_MUX_PAD_RGB,
	DS5_MUX_PAD_IR,
	DS5_MUX_PAD_IMU,
	DS5_MUX_PAD_COUNT,
};
enum state_sid {
	DEPTH_SID = 0,
	RGB_SID,
	IR_SID,
	IMU_SID,
	MUX_SID = -1
};

struct ds5_link_freq_config {
	const struct ds5_reg_list reg_list;
};

struct ds5_resolution {
	/* Frame width in pixels */
	u16 width;

	/* Frame height in pixels */
	u16 height;

	/* MEDIA_BUS_FMT */
	u16 code;

	/* MODE_FPS*/
	const u16 *framerates;
	u8 n_framerates;
};

enum {
	DS5_DS5U,
	DS5_ASR,
	DS5_AWG,
};

struct ds5_variant {
	const struct ds5_format *formats;
	unsigned int n_formats;
};

struct ds5_format {
	unsigned int n_resolutions;
	const struct ds5_resolution *resolutions;
	u32 mbus_code;
	u8 data_type;
};
struct ds5_stream_config {
	u16 config_status_base;
	u16 stream_status_base;
	u16 stream_id;
	u16 vc_id;

	u16 dt_addr;
	u16 md_addr;
	u16 override_addr;
	u16 fps_addr;
	u16 width_addr;
	u16 height_addr;
	u16 md_fmt;
};

struct ds5_sensor {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;
	u16 mux_pad;
	struct {
		const struct ds5_format *format;
		const struct ds5_resolution *resolution;
		u16 framerate;
	} config;
	struct ds5_stream_config stream_cfg;
	/* Streaming on/off */
	bool streaming;

	const struct ds5_format *formats;
	unsigned int n_formats;
	int pipe_id;
	int initialized;

	struct gpio_desc *reset_gpio;
};

struct ds5_mux_subdev {
	struct v4l2_subdev subdev;
};

struct ds5 {
	struct { struct ds5_sensor sensor; } depth;
	struct { struct ds5_sensor sensor; } ir;
	struct { struct ds5_sensor sensor; } rgb;
	struct { struct ds5_sensor sensor; } imu;
	struct {
		struct ds5_mux_subdev sd;
		struct media_pad pads[DS5_MUX_PAD_COUNT];
		struct ds5_sensor *last_set;
	} mux;

	struct ds5_ctrls ctrls;
	struct ds5_dfu_dev dfu_dev;
	bool power;
	const struct ds5_variant *variant;
	int is_depth, is_y8, is_rgb, is_imu;
	int aggregated;
	int routing_initialized;
	u16 fw_version;
	u16 fw_build;

	/* i2c client */
	struct i2c_client *client;

	struct ds5_platform_data *platform_data;
	struct gpio_desc *reset_gpio;

	struct regmap *regmap;
	struct mutex mutex;

	int pad_to_vc[DS5_MUX_PAD_COUNT];
	char suffix[5];
};

struct ds5_counters {
	unsigned int n_res;
	unsigned int n_fmt;
	unsigned int n_ctrl;
};

static inline void msleep_range(unsigned int delay_base)
{
	usleep_range(delay_base * 1000, delay_base * 1000 + 500);
}
static const u16 ds5_framerates[] = {5, 30};
static const u16 ds5_framerate_30 = 30;
static const u16 ds5_framerate_15_30[] = {15, 30};
static const u16 ds5_framerate_25 = 25;
static const u16 ds5_depth_framerate_to_30[] = {5, 15, 30};
static const u16 ds5_framerate_to_30[] = {5, 10, 15, 30};
static const u16 ds5_framerate_to_60[] = {5, 15, 30, 60};
static const u16 ds5_framerate_to_90[] = {5, 15, 30, 60, 90};
static const u16 ds5_framerate_100[] = {100};
static const u16 ds5_framerate_90[] = {90};
static const u16 ds5_imu_framerates[] = {50, 100, 200, 400};

static const struct ds5_reg ds5_init_reg[] = {

};

static const struct ds5_reg ds5_framesync_reg[] = {

};

static const struct ds5_reg ds5_1920_1536_30fps_reg[] = {

};

static const struct ds5_reg ds5_1920_1080_30fps_reg[] = {

};

static const struct ds5_reg ds5_1280_720_30fps_reg[] = {

};

static const struct ds5_reg_list ds5_init_reg_list = {
	.num_of_regs = ARRAY_SIZE(ds5_init_reg),
	.regs = ds5_init_reg,
};

static const struct ds5_reg_list ds5_framesync_reg_list = {
	.num_of_regs = ARRAY_SIZE(ds5_framesync_reg),
	.regs = ds5_framesync_reg,
};

static const struct ds5_reg_list ds5_1920_1536_30fps_reg_list = {
	.num_of_regs = ARRAY_SIZE(ds5_1920_1536_30fps_reg),
	.regs = ds5_1920_1536_30fps_reg,
};

static const struct ds5_reg_list ds5_1920_1080_30fps_reg_list = {
	.num_of_regs = ARRAY_SIZE(ds5_1920_1080_30fps_reg),
	.regs = ds5_1920_1080_30fps_reg,
};

static const struct ds5_reg_list ds5_1280_720_30fps_reg_list = {
	.num_of_regs = ARRAY_SIZE(ds5_1280_720_30fps_reg),
	.regs = ds5_1280_720_30fps_reg,
};

static const struct ds5_resolution d43x_depth_sizes[] = {
	{
		.width = 1280,
		.height = 720,
		.framerates = ds5_depth_framerate_to_30,
		.n_framerates = ARRAY_SIZE(ds5_depth_framerate_to_30),
	},
	{
		.width = 848,
		.height = 480,
		.framerates = ds5_framerate_to_90,
		.n_framerates = ARRAY_SIZE(ds5_framerate_to_90),
	},
	{
		.width = 848,
		.height = 100,
		.framerates = ds5_framerate_100,
		.n_framerates = ARRAY_SIZE(ds5_framerate_100),
	},
	{
		.width =  640,
		.height = 480,
		.framerates = ds5_framerate_to_90,
		.n_framerates = ARRAY_SIZE(ds5_framerate_to_90),
	},
	{
		.width =  640,
		.height = 360,
		.framerates = ds5_framerate_to_90,
		.n_framerates = ARRAY_SIZE(ds5_framerate_to_90),
	},
	{
		.width =  480,
		.height = 270,
		.framerates = ds5_framerate_to_90,
		.n_framerates = ARRAY_SIZE(ds5_framerate_to_90),
	},
	{
		.width =  424,
		.height = 240,
		.framerates = ds5_framerate_to_90,
		.n_framerates = ARRAY_SIZE(ds5_framerate_to_90),
	},
	{
		.width =  256,
		.height = 144,
		.framerates = ds5_framerate_90,
		.n_framerates = ARRAY_SIZE(ds5_framerate_90),
	},
};
static const struct ds5_resolution d46x_depth_sizes[] = {
	{
		.width = 1280,
		.height = 960,
		.framerates = ds5_framerates,
		.n_framerates = ARRAY_SIZE(ds5_framerates),
	}, {
		.width =  640,
		.height = 480,
		.framerates = ds5_framerates,
		.n_framerates = ARRAY_SIZE(ds5_framerates),
	},
};

static const struct ds5_resolution y8_sizes[] = {
	{
		.width = 1280,
		.height = 720,
		.framerates = ds5_depth_framerate_to_30,
		.n_framerates = ARRAY_SIZE(ds5_depth_framerate_to_30),
	}, {
		.width =  848,
		.height = 480,
		.framerates = ds5_framerate_to_90,
		.n_framerates = ARRAY_SIZE(ds5_framerate_to_90),
	}, {
		.width =  640,
		.height = 480,
		.framerates = ds5_framerate_to_90,
		.n_framerates = ARRAY_SIZE(ds5_framerate_to_90),
	}, {
		.width =  640,
		.height = 360,
		.framerates = ds5_framerate_to_90,
		.n_framerates = ARRAY_SIZE(ds5_framerate_to_90),
	}, {
		.width =  480,
		.height = 270,
		.framerates = ds5_framerate_to_90,
		.n_framerates = ARRAY_SIZE(ds5_framerate_to_90),
	}, {
		.width =  424,
		.height = 240,
		.framerates = ds5_framerate_to_90,
		.n_framerates = ARRAY_SIZE(ds5_framerate_to_90),
	}
};

static const struct ds5_resolution ds5_rlt_rgb_sizes[] = {
	{
		.width = 1280,
		.height = 800,
		.framerates = ds5_framerate_to_30,
		.n_framerates = ARRAY_SIZE(ds5_framerate_to_30),
	}, {
		.width = 1280,
		.height = 720,
		.framerates = ds5_framerate_to_30,
		.n_framerates = ARRAY_SIZE(ds5_framerate_to_30),
	}, {
		.width = 848,
		.height = 480,
		.framerates = ds5_framerate_to_60,
		.n_framerates = ARRAY_SIZE(ds5_framerate_to_60),
	}, {
		.width = 640,
		.height = 480,
		.framerates = ds5_framerate_to_60,
		.n_framerates = ARRAY_SIZE(ds5_framerate_to_60),
	}, {
		.width = 640,
		.height = 360,
		.framerates = ds5_framerate_to_90,
		.n_framerates = ARRAY_SIZE(ds5_framerate_to_90),
	}, {
		.width = 480,
		.height = 270,
		.framerates = ds5_framerate_to_90,
		.n_framerates = ARRAY_SIZE(ds5_framerate_to_90),
	}, {
		.width = 424,
		.height = 240,
		.framerates = ds5_framerate_to_90,
		.n_framerates = ARRAY_SIZE(ds5_framerate_to_90),
	},
};

static const struct ds5_resolution ds5_onsemi_rgb_sizes[] = {
	{
		.width = 640,
		.height = 480,
		.framerates = ds5_framerate_to_90,
		.n_framerates = ARRAY_SIZE(ds5_framerate_to_90),
	}, {
		.width = 960,
		.height = 720,
		.framerates = ds5_framerate_to_60,
		.n_framerates = ARRAY_SIZE(ds5_framerate_to_60),
	}, {
		.width = 1280,
		.height = 720,
		.framerates = ds5_framerates,
		.n_framerates = ARRAY_SIZE(ds5_framerates),
	}, {
		.width = 1920,
		.height = 1080,
		.framerates = ds5_framerates,
		.n_framerates = ARRAY_SIZE(ds5_framerates),
	}, {
		.width = 2048,
		.height = 1536,
		.framerates = ds5_framerates,
		.n_framerates = ARRAY_SIZE(ds5_framerates),
	},
};

static const struct ds5_resolution ds5_size_w10 = {
	.width =  1920,
	.height = 1080,
	.framerates = &ds5_framerate_30,
	.n_framerates = 1,
};

static const struct ds5_resolution d43x_calibration_sizes[] = {
	{
		.width =  1280,
		.height = 800,
		.framerates = ds5_framerate_15_30,
		.n_framerates = ARRAY_SIZE(ds5_framerate_15_30),
	},
};

static const struct ds5_resolution d46x_calibration_sizes[] = {
	{
		.width =  1600,
		.height = 1300,
		.framerates = ds5_framerate_15_30,
		.n_framerates = ARRAY_SIZE(ds5_framerate_15_30),
	},
};

static const struct ds5_resolution ds5_size_imu[] = {
	{
	.width = 32,
	.height = 1,
	.framerates = ds5_imu_framerates,
	.n_framerates = ARRAY_SIZE(ds5_imu_framerates),
	},
};

// 32 bit IMU introduced with IMU sensitivity attribute Firmware
static const struct ds5_resolution ds5_size_imu_extended[] = {
	{
	.width = 38,
	.height = 1,
	.framerates = ds5_imu_framerates,
	.n_framerates = ARRAY_SIZE(ds5_imu_framerates),
	},
};

static const struct ds5_format ds5_depth_formats_d43x[] = {
	{
		.data_type = GMSL_CSI_DT_YUV422_8,	/* Z16 */
		.mbus_code = MEDIA_BUS_FMT_FIXED,
		.n_resolutions = ARRAY_SIZE(d43x_depth_sizes),
		.resolutions = d43x_depth_sizes,
	}, {
		// TODO: 0x31 is replaced with 0x1e since it caused low FPS in Jetson.
		.data_type = GMSL_CSI_DT_YUV422_8,	/* Z16 */
		.mbus_code = MEDIA_BUS_FMT_UYVY8_1X16,
		.n_resolutions = ARRAY_SIZE(d43x_depth_sizes),
		.resolutions = d43x_depth_sizes,
	}, {
		.data_type = GMSL_CSI_DT_RAW_8,	/* Y8 */
		.mbus_code = MEDIA_BUS_FMT_Y8_1X8,
		.n_resolutions = ARRAY_SIZE(d43x_depth_sizes),
		.resolutions = d43x_depth_sizes,
	}, {
		.data_type = GMSL_CSI_DT_RGB_888,	/* 24-bit Calibration */
		.mbus_code = MEDIA_BUS_FMT_RGB888_1X24,	/* FIXME */
		.n_resolutions = ARRAY_SIZE(d43x_calibration_sizes),
		.resolutions = d43x_calibration_sizes,
	},
};

static const struct ds5_format ds5_depth_formats_d46x[] = {
	{
		// TODO: 0x31 is replaced with 0x1e since it caused low FPS in Jetson.
		.data_type = GMSL_CSI_DT_YUV422_8,	/* Z16 */
		.mbus_code = MEDIA_BUS_FMT_UYVY8_1X16,
		.n_resolutions = ARRAY_SIZE(d46x_depth_sizes),
		.resolutions = d46x_depth_sizes,
	}, {
		/* First format: default */
		.data_type = GMSL_CSI_DT_RAW_8,	/* Y8 */
		.mbus_code = MEDIA_BUS_FMT_Y8_1X8,
		.n_resolutions = ARRAY_SIZE(d46x_depth_sizes),
		.resolutions = d46x_depth_sizes,
	}, {
		.data_type = GMSL_CSI_DT_RGB_888,	/* 24-bit Calibration */
		.mbus_code = MEDIA_BUS_FMT_RGB888_1X24,	/* FIXME */
		.n_resolutions = ARRAY_SIZE(d46x_calibration_sizes),
		.resolutions = d46x_calibration_sizes,
	},
};

#define DS5_DEPTH_N_FORMATS 1

static const struct ds5_format ds5_y_formats_ds5u[] = {
	{
		/* First format: default */
		.data_type = GMSL_CSI_DT_RAW_8,	/* Y8 */
		.mbus_code = MEDIA_BUS_FMT_Y8_1X8,
		.n_resolutions = ARRAY_SIZE(y8_sizes),
		.resolutions = y8_sizes,
	}, {
		.data_type = GMSL_CSI_DT_YUV422_8,	/* Y8I */
		.mbus_code = MEDIA_BUS_FMT_VYUY8_1X16,
		.n_resolutions = ARRAY_SIZE(y8_sizes),
		.resolutions = y8_sizes,
	}, {
		.data_type = GMSL_CSI_DT_RGB_888,	/* 24-bit Calibration */
		.mbus_code = MEDIA_BUS_FMT_RGB888_1X24,	/* FIXME */
		.n_resolutions = ARRAY_SIZE(d43x_calibration_sizes),
		.resolutions = d43x_calibration_sizes,
	},
};

static const struct ds5_format ds5_rlt_rgb_format = {
	.data_type = GMSL_CSI_DT_YUV422_8,	/* UYVY */
	.mbus_code = MEDIA_BUS_FMT_YUYV8_1X16,
	.n_resolutions = ARRAY_SIZE(ds5_rlt_rgb_sizes),
	.resolutions = ds5_rlt_rgb_sizes,
};
#define DS5_RLT_RGB_N_FORMATS 1

static const struct ds5_format ds5_onsemi_rgb_format = {
	.data_type = GMSL_CSI_DT_YUV422_8,	/* UYVY */
	.mbus_code = MEDIA_BUS_FMT_YUYV8_1X16,
	.n_resolutions = ARRAY_SIZE(ds5_onsemi_rgb_sizes),
	.resolutions = ds5_onsemi_rgb_sizes,
};
#define DS5_ONSEMI_RGB_N_FORMATS 1

static const struct ds5_variant ds5_variants[] = {
	[DS5_DS5U] = {
		.formats = ds5_y_formats_ds5u,
		.n_formats = ARRAY_SIZE(ds5_y_formats_ds5u),
	},
};

static const struct ds5_format ds5_imu_formats[] = {
	{
		/* First format: default */
		.data_type = GMSL_CSI_DT_RAW_8,	/* IMU DT */
		.mbus_code = MEDIA_BUS_FMT_Y8_1X8,
		.n_resolutions = ARRAY_SIZE(ds5_size_imu),
		.resolutions = ds5_size_imu,
	},
};

static const struct ds5_format ds5_imu_formats_extended[] = {
	{
		/* First format: default */
		.data_type = GMSL_CSI_DT_RAW_8,	/* IMU DT */
		.mbus_code = MEDIA_BUS_FMT_Y8_1X8,
		.n_resolutions = ARRAY_SIZE(ds5_size_imu_extended),
		.resolutions = ds5_size_imu_extended,
	},
};

static const struct v4l2_mbus_framefmt ds5_mbus_framefmt_template = {
	.width = 0,
	.height = 0,
	.code = MEDIA_BUS_FMT_FIXED,
	.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT,
	.quantization = V4L2_QUANTIZATION_DEFAULT,
	.xfer_func = V4L2_XFER_FUNC_DEFAULT,
};
/// basic i2c read/write functions
static int ds5_raw_write(struct ds5 *ds5, u16 reg, const void *val, size_t val_len)
{
	int ret;

	dev_dbg(&ds5->client->dev, "%s, reg %x len %x, val %x\n", __func__, reg, (int)val_len, *(u16 *)val);

	ret = regmap_raw_write(ds5->regmap, reg, val, val_len);
	if (ret < 0)
		dev_err(&ds5->client->dev, "regmap raw write failed: %d\n", ret);
	else
		if (ds5->dfu_dev.dfu_state_flag == DS5_DFU_IDLE)
			dev_dbg(&ds5->client->dev, "regmap raw write: reg=0x%04x val=0x%04x\n",
				reg, *(u16 *)val);
	return ret;
}
static int ds5_write(struct ds5 *ds5, u16 reg, u16 val)
{
	int ret;
	u8 value[2];

	value[1] = val >> 8;
	value[0] = val & 0x00FF;

	dev_dbg(&ds5->client->dev, "%s, reg %x, val %x\n", __func__, reg, val);

	ret = regmap_raw_write(ds5->regmap, reg, value, sizeof(value));
	if (ret < 0)
		dev_err(&ds5->client->dev, "regmap write failed: %d\n", ret);
	else
		if (ds5->dfu_dev.dfu_state_flag == DS5_DFU_IDLE)
			dev_dbg(&ds5->client->dev, "regmap write: reg=0x%04x val=0x%04x\n",
				reg, val);

	return ret;
}
static int ds5_raw_read(struct ds5 *ds5, u16 reg, void *val, size_t val_len)
{
	int ret = regmap_raw_read(ds5->regmap, reg, val, val_len);

	if (ret < 0)
		dev_err(&ds5->client->dev, "regmap raw read failed: %d\n", ret);
	else
		dev_dbg(&ds5->client->dev, "regmap raw read: reg=0x%04x val=0x%04x\n",
			reg, *(u16 *)val);

	return ret;
}
static int ds5_read(struct ds5 *ds5, u16 reg, u16 *val)
{
	int ret;

	ret = regmap_raw_read(ds5->regmap, reg, val, 2);
	if (ret < 0)
		dev_err(&ds5->client->dev, "regmap raw read failed: %d\n", ret);
	else
		if (ds5->dfu_dev.dfu_state_flag == DS5_DFU_IDLE)
			dev_dbg(&ds5->client->dev, "regmap raw read: reg=0x%04x val=0x%04x\n",
				reg, *val);

	dev_dbg(&ds5->client->dev, "%s, reg %x, val %x\n", __func__, reg, *val);
	return ret;
}

/* Get readable sensor name */
static const char *ds5_get_sensor_name(struct ds5 *ds5)
{
	static const char *sensor_name[] = {"unknown", "RGB", "DEPTH", "Y8", "IMU"};
	int sensor_id = ds5->is_rgb * 1 + ds5->is_depth * 2 + \
			ds5->is_y8 * 3 + ds5->is_imu * 4;
	if (sensor_id >= (sizeof(sensor_name)/sizeof(*sensor_name)))
		sensor_id = 0;

	return sensor_name[sensor_id];
}

static struct ds5_sensor *vc_to_sensor(struct ds5 *ds5, int vc_id)
{
	struct ds5_sensor *sensor = NULL;

	switch (vc_id) {
	case 0:
			sensor = &ds5->depth.sensor;
			break;
	case 1:
			sensor = &ds5->rgb.sensor;
			break;
	case 2:
			sensor = &ds5->ir.sensor;
			break;
	case 3:
			sensor = &ds5->imu.sensor;
			break;
	default:
			sensor = NULL;
			break;
	}
	return sensor;
}

static int ds5_sub_configure(struct ds5 *state, u16 vc_id)
{
	struct ds5_sensor *sensor;
	struct ds5_stream_config *stream_cfg;
	u16 fmt, md_fmt;
	u16 data_type1, data_type2;
	u16 dt_addr, md_addr, override_addr, fps_addr, width_addr, height_addr;
	int ret;

	sensor = vc_to_sensor(state, vc_id);

	dev_dbg(&state->client->dev, "%s: Configuring %s stream (vc_id=%d)\n",
		__func__, ds5_get_sensor_name(state), vc_id);

	stream_cfg = &sensor->stream_cfg;
	dt_addr = stream_cfg->dt_addr;
	md_addr = stream_cfg->md_addr;
	override_addr = stream_cfg->override_addr;
	fps_addr = stream_cfg->fps_addr;
	width_addr = stream_cfg->width_addr;
	height_addr = stream_cfg->height_addr;
	md_fmt = stream_cfg->md_fmt;

	fmt = sensor->streaming ? sensor->config.format->data_type : 0;

	/*
	 * Set depth stream Z16 data type as 0x31
	 * Set IR stream Y8I data type as 0x32
	 */
	if (vc_id == 0 && fmt != 0) {
		dev_dbg(&state->client->dev, "Configuring ds5 DT reg=%x val=0x31\n", dt_addr);
		ret = ds5_write(state, dt_addr, 0x31);
	} else if (state->is_y8 && fmt != 0 &&
		 sensor->config.format->data_type == GMSL_CSI_DT_YUV422_8) {
		dev_dbg(&state->client->dev, "Configuring ds5 DT reg=%x val=0x32\n", dt_addr);
		ret = ds5_write(state, dt_addr, 0x32);
	} else {
		dev_dbg(&state->client->dev, "Configuring ds5 DT reg=%x val=%x\n", dt_addr, fmt);
		ret = ds5_write(state, dt_addr, fmt);
	}
	if (ret < 0) {
		dev_err(&state->client->dev, "Configuring ds5 DT failed\n", __func__);
		return ret;
	}

	dev_dbg(&state->client->dev, "Configuring ds5 MD reg=%x val=%x (vc_id<<8 %x|md_fmt %x)\n", md_addr, ((vc_id<<8)|md_fmt), vc_id, md_fmt);
	ret = ds5_write(state, md_addr, (vc_id << 8) | md_fmt);
	if (ret < 0)
		return ret;

	// stream stop ends here
	if (!sensor->streaming)
		return ret;

	if (override_addr != 0) {
		dev_dbg(&state->client->dev, "Configuring ds5 Override reg=%x val=%x\n", override_addr, fmt);
		ret = ds5_write(state, override_addr, fmt);
		if (ret < 0)
			return ret;
	}

	dev_dbg(&state->client->dev, "Configuring ds5 FPS reg=%x val=%x\n", fps_addr, sensor->config.framerate);
	ret = ds5_write(state, fps_addr, sensor->config.framerate);
	if (ret < 0)
		return ret;

	dev_dbg(&state->client->dev, "Configuring ds5 Width reg=%x val=%x\n", width_addr, sensor->config.resolution->width);
	ret = ds5_write(state, width_addr, sensor->config.resolution->width);
	if (ret < 0)
		return ret;

	dev_dbg(&state->client->dev, "Configuring ds5 Height reg=%x val=%x\n", height_addr, sensor->config.resolution->height);
	ret = ds5_write(state, height_addr, sensor->config.resolution->height);
	if (ret < 0)
		return ret;

	msleep(50);

	dev_dbg(&state->client->dev, "%s: Sensor %s stream configured\n",
		__func__, ds5_get_sensor_name(state));

	return 0;
}

static int ds5_configure(struct ds5 *state)
{
	struct ds5_sensor *sensor;
	u16 fmt, md_fmt, vc_id;
#ifdef CONFIG_VIDEO_D4XX_SERDES
	u16 data_type1, data_type2;
#endif
	u16 dt_addr, md_addr, override_addr, fps_addr, width_addr, height_addr;
	int ret;

	if (state->is_depth) {
		sensor = &state->depth.sensor;
		dt_addr = DS5_DEPTH_STREAM_DT;
		md_addr = DS5_DEPTH_STREAM_MD;
		override_addr = DS5_DEPTH_OVERRIDE;
		fps_addr = DS5_DEPTH_FPS;
		width_addr = DS5_DEPTH_RES_WIDTH;
		height_addr = DS5_DEPTH_RES_HEIGHT;
		md_fmt = GMSL_CSI_DT_EMBED;
		vc_id = 0;
	} else if (state->is_rgb) {
		sensor = &state->rgb.sensor;
		dt_addr = DS5_RGB_STREAM_DT;
		md_addr = DS5_RGB_STREAM_MD;
		override_addr = 0;
		fps_addr = DS5_RGB_FPS;
		width_addr = DS5_RGB_RES_WIDTH;
		height_addr = DS5_RGB_RES_HEIGHT;
		md_fmt = GMSL_CSI_DT_EMBED;
		vc_id = 1;
	} else if (state->is_y8) {
		sensor = &state->ir.sensor;
		dt_addr = DS5_IR_STREAM_DT;
		md_addr = DS5_IR_STREAM_MD;
		override_addr = DS5_IR_OVERRIDE;
		fps_addr = DS5_IR_FPS;
		width_addr = DS5_IR_RES_WIDTH;
		height_addr = DS5_IR_RES_HEIGHT;
		md_fmt = GMSL_CSI_DT_EMBED;
		vc_id = 2;
	} else if (state->is_imu) {
		sensor = &state->imu.sensor;
		dt_addr = DS5_IMU_STREAM_DT;
		md_addr = DS5_IMU_STREAM_MD;
		override_addr = 0;
		fps_addr = DS5_IMU_FPS;
		width_addr = DS5_IMU_RES_WIDTH;
		height_addr = DS5_IMU_RES_HEIGHT;
		md_fmt = 0x0;
		vc_id = 3;
	} else {
		return -EINVAL;
	}

	fmt = sensor->streaming ? sensor->config.format->data_type : 0;

	/*
	 * Set depth stream Z16 data type as 0x31
	 * Set IR stream Y8I data type as 0x32
	 */
	if (state->is_depth && fmt != 0) {
		ret = ds5_write(state, dt_addr, 0x31);
	} else if (state->is_y8 && fmt != 0 &&
		 sensor->config.format->data_type == GMSL_CSI_DT_YUV422_8) {
		ret = ds5_write(state, dt_addr, 0x32);
	} else {
		ret = ds5_write(state, dt_addr, fmt);
	}
	if (ret < 0)
		return ret;

	ret = ds5_write(state, md_addr, (vc_id << 8) | md_fmt);
	if (ret < 0)
		return ret;

	if (!sensor->streaming)
		return ret;

	if (override_addr != 0) {
		ret = ds5_write(state, override_addr, fmt);
		if (ret < 0)
			return ret;
	}

	ret = ds5_write(state, fps_addr, sensor->config.framerate);
	if (ret < 0)
		return ret;

	ret = ds5_write(state, width_addr, sensor->config.resolution->width);
	if (ret < 0)
		return ret;

	ret = ds5_write(state, height_addr, sensor->config.resolution->height);
	if (ret < 0)
		return ret;

	return 0;
}

static void ds5_set_state_last_set(struct ds5 *ds5)
{
	 dev_dbg(&ds5->client->dev, "%s(): %s\n",
		__func__, ds5_get_sensor_name(ds5));

	if (ds5->is_depth)
		ds5->mux.last_set = &ds5->depth.sensor;
	else if (ds5->is_rgb)
		ds5->mux.last_set = &ds5->rgb.sensor;
	else if (ds5->is_y8)
		ds5->mux.last_set = &ds5->ir.sensor;
	else
		ds5->mux.last_set = &ds5->imu.sensor;
}
static int ds5_s_state_pad(struct ds5 *ds5, int pad)
{
	int ret = 0;

	dev_dbg(&ds5->client->dev, "%s(): set state for pad: %d\n", __func__, pad);

	switch (pad) {
	case DS5_MUX_PAD_DEPTH:
		ds5->is_depth = 1;
		ds5->is_rgb = 0;
		ds5->is_y8 = 0;
		ds5->is_imu = 0;
		break;
	case DS5_MUX_PAD_RGB:
		ds5->is_depth = 0;
		ds5->is_rgb = 1;
		ds5->is_y8 = 0;
		ds5->is_imu = 0;
		break;
	case DS5_MUX_PAD_IR:
		ds5->is_depth = 0;
		ds5->is_rgb = 0;
		ds5->is_y8 = 1;
		ds5->is_imu = 0;
		break;
	case DS5_MUX_PAD_IMU:
		ds5->is_depth = 0;
		ds5->is_rgb = 0;
		ds5->is_y8 = 0;
		ds5->is_imu = 1;
		break;
	default:
		dev_warn(&ds5->client->dev, "%s(): unknown pad: %d\n", __func__, pad);
		ret = -EINVAL;
		break;
	}
	ds5_set_state_last_set(ds5);
	return ret;
}

static int ds5_sensor_register(struct ds5 *ds5, struct ds5_sensor *sensor)
{
	struct v4l2_subdev *sd = &sensor->sd;
	struct media_entity *entity = &sensor->sd.entity;
	int ret = -1;

	ret = v4l2_subdev_init_finalize(sd);
	if (ret < 0) {
		dev_err(sd->dev, "%s(): %d: %d\n", __func__, __LINE__, ret);
		return ret;
	}
	ret = v4l2_device_register_subdev(ds5->mux.sd.subdev.v4l2_dev, sd);
	if (ret < 0) {
		dev_err(sd->dev, "%s(): %d: %d\n", __func__, __LINE__, ret);
		return ret;
	}

	ret = media_create_pad_link(entity, 0,
			&ds5->mux.sd.subdev.entity, sensor->mux_pad,
			MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
	if (ret < 0) {
		dev_err(sd->dev, "%s(): %d: %d\n", __func__, __LINE__, ret);
		goto e_sd;
	}

	sensor->initialized = true;

	dev_dbg(sd->dev, "%s(): 0 -> %d\n", __func__, sensor->mux_pad);

	return 0;

e_sd:
	v4l2_device_unregister_subdev(sd);

	return ret;
}

static void ds5_sensor_remove(struct ds5_sensor *sensor)
{
	v4l2_device_unregister_subdev(&sensor->sd);

	media_entity_cleanup(&sensor->sd.entity);
}
// v4l2 fn for all
static int ds5_sensor_s_stream(struct v4l2_subdev *sd, int on)
{
	struct ds5_sensor *sensor = container_of(sd, struct ds5_sensor, sd);

	dev_dbg(sensor->sd.dev, "%s(): sensor: name=%s state=%d\n",
		__func__, sensor->sd.name, on);

	sensor->streaming = on;

	return 0;
}
static u16 __ds5_probe_framerate(const struct ds5_resolution *res, u16 target)
{
	int i;
	u16 framerate;

	for (i = 0; i < res->n_framerates; i++) {
		framerate = res->framerates[i];
		if (target <= framerate)
			return framerate;
	}

	return res->framerates[res->n_framerates - 1];
}
/* No locking needed for enumeration methods */
static int ds5_sensor_enum_mbus_code(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 10)
				     struct v4l2_subdev_pad_config *cfg,
#else
				     struct v4l2_subdev_state *v4l2_state,
#endif
				     struct v4l2_subdev_mbus_code_enum *mce)
{
	struct ds5_sensor *sensor = container_of(sd, struct ds5_sensor, sd);

	dev_dbg(sensor->sd.dev, "%s(): sensor %s pad: %d index: %d\n",
		__func__, sensor->sd.name, mce->pad, mce->index);
	if (mce->pad)
		return -EINVAL;

	if (mce->index >= sensor->n_formats)
		return -EINVAL;

	mce->code = sensor->formats[mce->index].mbus_code;

	return 0;
}

static int ds5_sensor_enum_frame_size(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 10)
				     struct v4l2_subdev_pad_config *cfg,
#else
				     struct v4l2_subdev_state *v4l2_state,
#endif
				      struct v4l2_subdev_frame_size_enum *fse)
{
	struct ds5_sensor *sensor = container_of(sd, struct ds5_sensor, sd);
	struct ds5 *state = v4l2_get_subdevdata(sd);
	const struct ds5_format *fmt;
	unsigned int i;

	dev_dbg(sensor->sd.dev, "%s(): sensor %s is %s\n",
		__func__, sensor->sd.name, ds5_get_sensor_name(state));

	for (i = 0, fmt = sensor->formats; i < sensor->n_formats; i++, fmt++)
		if (fse->code == fmt->mbus_code)
			break;

	if (i == sensor->n_formats)
		return -EINVAL;

	if (fse->index >= fmt->n_resolutions)
		return -EINVAL;

	fse->min_width = fse->max_width = fmt->resolutions[fse->index].width;
	fse->min_height = fse->max_height = fmt->resolutions[fse->index].height;

	return 0;
}

static int ds5_sensor_enum_frame_interval(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 10)
				     struct v4l2_subdev_pad_config *cfg,
#else
				     struct v4l2_subdev_state *v4l2_state,
#endif
					  struct v4l2_subdev_frame_interval_enum *fie)
{
	struct ds5_sensor *sensor = container_of(sd, struct ds5_sensor, sd);
	const struct ds5_format *fmt;
	const struct ds5_resolution *res;
	unsigned int i;

	for (i = 0, fmt = sensor->formats; i < sensor->n_formats; i++, fmt++)
		if (fie->code == fmt->mbus_code)
			break;

	if (i == sensor->n_formats)
		return -EINVAL;

	for (i = 0, res = fmt->resolutions; i < fmt->n_resolutions; i++, res++)
		if (res->width == fie->width && res->height == fie->height)
			break;

	if (i == fmt->n_resolutions)
		return -EINVAL;

	if (fie->index >= res->n_framerates)
		return -EINVAL;

	fie->interval.numerator = 1;
	fie->interval.denominator = res->framerates[fie->index];

	return 0;
}

static int ds5_sensor_get_fmt(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 10)
				     struct v4l2_subdev_pad_config *cfg,
#else
				     struct v4l2_subdev_state *v4l2_state,
#endif
			      struct v4l2_subdev_format *fmt)
{
	struct ds5_sensor *sensor = container_of(sd, struct ds5_sensor, sd);
	struct ds5 *state = v4l2_get_subdevdata(sd);

	if (fmt->pad)
		return -EINVAL;

	mutex_lock(&state->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 10)
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(6, 8, 0)
		fmt->format = *v4l2_subdev_get_try_format(sd, v4l2_state, fmt->pad);
#else
		fmt->format = *v4l2_subdev_state_get_format(v4l2_state, fmt->pad);
#endif
	else
		fmt->format = sensor->format;

	mutex_unlock(&state->mutex);

	dev_dbg(sd->dev, "%s(): pad %x, code %x, res %ux%u\n",
			__func__, fmt->pad, fmt->format.code,
			fmt->format.width, fmt->format.height);

	return 0;
}

/* Called with lock held */
static const struct ds5_format *ds5_sensor_find_format(
		struct ds5_sensor *sensor,
		struct v4l2_mbus_framefmt *ffmt,
		const struct ds5_resolution **best)
{
	const struct ds5_resolution *res;
	const struct ds5_format *fmt;
	unsigned long best_delta = ~0;
	unsigned int i;

	for (i = 0, fmt = sensor->formats; i < sensor->n_formats; i++, fmt++) {
		if (fmt->mbus_code == ffmt->code)
			break;
	}
	dev_dbg(sensor->sd.dev, "%s(): mbus_code = %x, code = %x\n",
		__func__, fmt->mbus_code, ffmt->code);

	if (i == sensor->n_formats) {
		/* Not found, use default */
		dev_dbg(sensor->sd.dev, "%s:%d Not found, use default\n",
			__func__, __LINE__);
		fmt = sensor->formats;
	}
	for (i = 0, res = fmt->resolutions; i < fmt->n_resolutions; i++, res++) {
		unsigned long delta = abs(ffmt->width * ffmt->height -
				res->width * res->height);
		if (delta < best_delta) {
			best_delta = delta;
			*best = res;
		}
	}

	ffmt->code = fmt->mbus_code;
	ffmt->width = (*best)->width;
	ffmt->height = (*best)->height;

	//ffmt->field = V4L2_FIELD_NONE;
	/* Should we use V4L2_COLORSPACE_RAW for Y12I? */
	//ffmt->colorspace = V4L2_COLORSPACE_SRGB;

	return fmt;
}
static int __ds5_sensor_set_fmt(struct ds5 *state, struct ds5_sensor *sensor,
				struct v4l2_subdev_state *v4l2_state,
				struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *mf;

	int substream = -1;

	dev_dbg(sensor->sd.dev, "%s(): state %p, sensor %p, fmt %p, fmt->format %p\n",
		__func__, state, sensor, fmt,  &fmt->format);

	mf = &fmt->format;

	if (fmt->pad)
		return -EINVAL;

	mutex_lock(&state->mutex);

	sensor->config.format = ds5_sensor_find_format(sensor, mf,
						&sensor->config.resolution);
	//r = DS5_FRAMERATE_DEFAULT_IDX < sensor->config.resolution->n_framerates ?
	//	DS5_FRAMERATE_DEFAULT_IDX : 0;
	/* FIXME: check if a framerate has been set */
	//sensor->config.framerate = sensor->config.resolution->framerates[r];

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 10)
	if (cfg && fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		*v4l2_subdev_get_try_format(&sensor->sd, cfg, fmt->pad) = *mf;
#elif LINUX_VERSION_CODE < KERNEL_VERSION(6, 8, 0)
	if (v4l2_state && fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		*v4l2_subdev_get_try_format(&sensor->sd, v4l2_state, fmt->pad) = *mf;
#else
	if (v4l2_state && fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		*v4l2_subdev_state_get_format(v4l2_state, fmt->pad) = *mf;
#endif

	else
// FIXME: use this format in .s_stream()
		sensor->format = *mf;

	state->mux.last_set = sensor;

	mutex_unlock(&state->mutex);

	dev_dbg(sensor->sd.dev, "%s(): fmt->pad: %d, sensor->mux_pad: %d, code: 0x%x, %ux%u\n", __func__,
		fmt->pad, sensor->mux_pad, fmt->format.code,
		fmt->format.width, fmt->format.height);

	return 0;
}

static int ds5_sensor_set_fmt(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 10)
				     struct v4l2_subdev_pad_config *cfg,
#else
				     struct v4l2_subdev_state *v4l2_state,
#endif
			      struct v4l2_subdev_format *fmt)
{
	struct ds5_sensor *sensor = container_of(sd, struct ds5_sensor, sd);
	struct ds5 *state = v4l2_get_subdevdata(sd);

	/* set state by vc */
	ds5_s_state_pad(state, sensor->mux_pad);

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 10)
	return __ds5_sensor_set_fmt(state, sensor, cfg, fmt);
#else
	return __ds5_sensor_set_fmt(state, sensor, v4l2_state, fmt);
#endif
}

static int ds5_sensor_get_frame_interval(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
/* pad ops */
		struct v4l2_subdev_state *sd_state,
		struct v4l2_subdev_frame_interval *fi)
#else
/* Video ops */
		struct v4l2_subdev_frame_interval *fi)
#endif
{
	struct ds5_sensor *sensor = container_of(sd, struct ds5_sensor, sd);

	if (NULL == sd || NULL == fi)
		return -EINVAL;

	fi->interval.numerator = 1;
	fi->interval.denominator = sensor->config.framerate;

	dev_dbg(sd->dev, "%s(): %s %u\n", __func__, sd->name,
			fi->interval.denominator);

	return 0;
}

static int ds5_sensor_set_frame_interval(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
/* pad ops */
		struct v4l2_subdev_state *sd_state,
		struct v4l2_subdev_frame_interval *fi)
#else
/* Video ops */
		struct v4l2_subdev_frame_interval *fi)
#endif
{
	struct ds5_sensor *sensor = container_of(sd, struct ds5_sensor, sd);
	u16 framerate = 1;

	if (NULL == sd || NULL == fi || fi->interval.numerator == 0)
		return -EINVAL;

	framerate = fi->interval.denominator / fi->interval.numerator;
	framerate = __ds5_probe_framerate(sensor->config.resolution, framerate);
	sensor->config.framerate = framerate;
	fi->interval.numerator = 1;
	fi->interval.denominator = framerate;

	dev_dbg(sd->dev, "%s(): %s %u\n", __func__, sd->name, framerate);

	return 0;
}
// v4l2 fn for mux
static int ds5_mux_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct ds5 *state = v4l2_get_subdevdata(sd);

	dev_dbg(sd->dev, "%s(): %s (%p)\n", __func__, sd->name, fh);
	if (state->dfu_dev.dfu_state_flag)
		return -EBUSY;
	state->dfu_dev.device_open_count++;

	return 0;
};
static int ds5_mux_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct ds5 *state = v4l2_get_subdevdata(sd);

	dev_dbg(sd->dev, "%s(): %s (%p)\n", __func__, sd->name, fh);
	state->dfu_dev.device_open_count--;
	return 0;
};
static int ds5_state_to_pad(struct ds5 *state)
{
	int pad = -1;

	if (state->is_depth)
		pad = DS5_MUX_PAD_DEPTH;
	if (state->is_y8)
		pad = DS5_MUX_PAD_IR;
	if (state->is_rgb)
		pad = DS5_MUX_PAD_RGB;
	if (state->is_imu)
		pad = DS5_MUX_PAD_IMU;
	return pad;
}

static int ds5_mux_set_stream(struct v4l2_subdev *sd, int on)
{
	struct ds5 *state = container_of(sd, struct ds5, mux.sd.subdev);
	u16 streaming, status;
	int ret = 0;
	unsigned int i = 0;
	int restore_val = 0;
	u16 config_status_base, stream_status_base, stream_id, vc_id;
	struct ds5_sensor *sensor = state->mux.last_set;

	// spare duplicate calls
	if (sensor->streaming == on)
		return 0;
	if (state->is_depth) {
		config_status_base = DS5_DEPTH_CONFIG_STATUS;
		stream_status_base = DS5_DEPTH_STREAM_STATUS;
		stream_id = DS5_STREAM_DEPTH;
		vc_id = 0;
	} else if (state->is_rgb) {
		config_status_base = DS5_RGB_CONFIG_STATUS;
		stream_status_base = DS5_RGB_STREAM_STATUS;
		stream_id = DS5_STREAM_RGB;
		vc_id = 1;
	} else if (state->is_y8) {
		config_status_base = DS5_IR_CONFIG_STATUS;
		stream_status_base = DS5_IR_STREAM_STATUS;
		stream_id = DS5_STREAM_IR;
		vc_id = 2;
	} else if (state->is_imu) {
		config_status_base = DS5_IMU_CONFIG_STATUS;
		stream_status_base = DS5_IMU_STREAM_STATUS;
		stream_id = DS5_STREAM_IMU;
		vc_id = 3;
	} else {
		return -EINVAL;
	}

	dev_dbg(&state->client->dev, "%s s_stream for stream %s, vc:%d, SENSOR=%s on = %d\n",
			__func__, sensor->sd.name, vc_id, ds5_get_sensor_name(state), on);

	restore_val = sensor->streaming;
	sensor->streaming = on;

	if (on) {
		ret = ds5_configure(state);
		if (ret)
			goto restore_s_state;

		ret = ds5_write(state, DS5_START_STOP_STREAM,
				DS5_STREAM_START | stream_id);
		if (ret < 0) {
			goto restore_s_state;
		}

		// check streaming status from FW
		for (i = 0; i < DS5_START_MAX_COUNT; i++) {
			ds5_read(state, stream_status_base, &streaming);
			ds5_read(state, config_status_base, &status);
			if ((status & DS5_STATUS_STREAMING) &&
					streaming == DS5_STREAM_STREAMING)
				break;

			msleep_range(DS5_START_POLL_TIME);
		}

		if (i == DS5_START_MAX_COUNT) {
			dev_err(&state->client->dev,
				"%s start streaming failed, exit on timeout\n", __func__);
			/* notify fw */
			ret = ds5_write(state, DS5_START_STOP_STREAM,
					DS5_STREAM_STOP | stream_id);
			ret = -EAGAIN;
			goto restore_s_state;
		} else {
			dev_dbg(&state->client->dev, "%s started after %dms\n",
				__func__, i * DS5_START_POLL_TIME);
		}
	} else { // off
		ret = ds5_write(state, DS5_START_STOP_STREAM,
				DS5_STREAM_STOP | stream_id);
		if (ret < 0) {
			goto restore_s_state;
		}
	}

	ds5_read(state, config_status_base, &status);
	ds5_read(state, stream_status_base, &streaming);
	dev_dbg(&state->client->dev,
			"%s: %s %s, stream_status 0x%x:%x, config_status 0x%x:%x ret=%d\n",
			__func__, ds5_get_sensor_name(state),
			(on)?"START":"STOP",
			stream_status_base, streaming,
			config_status_base, status, ret);

	return ret;

restore_s_state:
#ifdef CONFIG_VIDEO_D4XX_SERDES
	if (on && sensor->pipe_id >= 0) {
		//if (max9296_release_pipe(state->dser_dev, sensor->pipe_id) < 0)
		//	dev_warn(&state->client->dev, "release pipe failed\n");
		sensor->pipe_id = -1;
	}
#endif

	ds5_read(state, config_status_base, &status);
	dev_err(&state->client->dev,
			"%s stream toggle failed! %x status 0x%04x\n",
			ds5_get_sensor_name(state), restore_val, status);

	sensor->streaming = restore_val;

	return ret;
}
static int _mux_set_routing(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_krouting *routing)
{
	static const struct v4l2_mbus_framefmt format = {
		.width = 640,
		.height = 480,
		.code = MEDIA_BUS_FMT_UYVY8_1X16,
	};

	int ret;
	int completed = 0;
	struct ds5 *ds5 = container_of(sd, struct ds5, mux.sd.subdev);

	/*
	 * Note: we can only support up to V4L2_FRAME_DESC_ENTRY_MAX, until
	 * frame desc is made dynamically allocated.
	 */
	if (routing->num_routes > V4L2_FRAME_DESC_ENTRY_MAX)
		return -E2BIG;

	ret = v4l2_subdev_routing_validate(sd, routing,
					   V4L2_SUBDEV_ROUTING_ONLY_1_TO_1 |
					   V4L2_SUBDEV_ROUTING_NO_SINK_STREAM_MIX);
	if (ret)
		return ret;

	ret = v4l2_subdev_set_routing_with_fmt(sd, state, routing, &format);
	if (ret)
		return ret;

	ds5->routing_initialized = 1;
	return 0;

	/* get format from sink pads*/
    for (unsigned int i = 0; i < routing->num_routes; i++) {
	const struct v4l2_subdev_route *route = &routing->routes[i];
	struct v4l2_mbus_framefmt *sink_fmt, *source_fmt;
	struct ds5_sensor *sensor = NULL;
		struct ds5 *ds5 = container_of(sd, struct ds5, mux.sd.subdev);

	/* Get the sensor corresponding to this sink pad */
	switch (route->sink_pad) {
	case DS5_MUX_PAD_DEPTH:
	    sensor = &ds5->depth.sensor;
	    break;
	case DS5_MUX_PAD_RGB:
	    sensor = &ds5->rgb.sensor;
	    break;
	case DS5_MUX_PAD_IR:
	    sensor = &ds5->ir.sensor;
	    break;
	case DS5_MUX_PAD_IMU:
	    sensor = &ds5->imu.sensor;
	    break;
	default:
	    dev_err(sd->dev, "Invalid sink pad %u\n", route->sink_pad);
	    return -EINVAL;
	}
		if (sensor->initialized != true)
			continue;
	/* Get sink pad format (use sensor's active format as base) */
	sink_fmt = v4l2_subdev_state_get_format(state, route->sink_pad);
	if (!sink_fmt) {
	    dev_err(sd->dev, "Failed to get format for sink pad %u\n",
		route->sink_pad);
	    return -EINVAL;
	}

	/* Initialize sink format from sensor's current format if not set */
	if (sink_fmt->width == 0 || sink_fmt->height == 0) {
	    *sink_fmt = sensor->format;
	    dev_dbg(sd->dev, "Initialized sink pad %u format from sensor: %ux%u, code 0x%x\n",
		route->sink_pad, sink_fmt->width, sink_fmt->height, sink_fmt->code);
	}

	/* Get source pad format */
	source_fmt = v4l2_subdev_state_get_format(state, route->source_pad);
	if (!source_fmt) {
	    dev_err(sd->dev, "Failed to get format for source pad %u\n",
		route->source_pad);
	    return -EINVAL;
	}

	/* Copy format from sink to source pad */
	*source_fmt = *sink_fmt;

	dev_dbg(sd->dev, "Route %u: sink pad %u -> source pad %u, format %ux%u, code 0x%x\n",
	    i, route->sink_pad, route->source_pad,
	    source_fmt->width, source_fmt->height, source_fmt->code);

		completed++;
    }

	if (completed)
		ret = v4l2_subdev_set_routing_with_fmt(sd, state, routing, &format);
	else
		ret = v4l2_subdev_set_routing(sd, state, routing);
	if (ret)
		return ret;

	return 0;
}

/* No locking needed */
static int ds5_mux_enum_mbus_code(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 10)
				     struct v4l2_subdev_pad_config *cfg,
#else
				     struct v4l2_subdev_state *v4l2_state,
#endif
				  struct v4l2_subdev_mbus_code_enum *mce)
{
	struct ds5 *state = container_of(sd, struct ds5, mux.sd.subdev);
	struct v4l2_subdev_mbus_code_enum tmp = *mce;
	struct v4l2_subdev *remote_sd;
	int ret = -1;

	dev_dbg(&state->client->dev, "%s(): %s\n", __func__, sd->name);
	switch (mce->pad) {
	case DS5_MUX_PAD_IR:
		remote_sd = &state->ir.sensor.sd;
		break;
	case DS5_MUX_PAD_DEPTH:
		remote_sd = &state->depth.sensor.sd;
		break;
	case DS5_MUX_PAD_RGB:
		remote_sd = &state->rgb.sensor.sd;
		break;
	case DS5_MUX_PAD_IMU:
		remote_sd = &state->imu.sensor.sd;
		break;
	case DS5_MUX_PAD_EXTERNAL:
		if (mce->index >= state->ir.sensor.n_formats +
				state->depth.sensor.n_formats)
			return -EINVAL;

		/*
		 * First list Left node / Motion Tracker formats, then depth.
		 * This should also help because D16 doesn't have a direct
		 * analog in MIPI CSI-2.
		 */
		if (mce->index < state->ir.sensor.n_formats) {
			remote_sd = &state->ir.sensor.sd;
		} else {
			tmp.index = mce->index - state->ir.sensor.n_formats;
			remote_sd = &state->depth.sensor.sd;
		}

		break;
	default:
		return -EINVAL;
	}

	tmp.pad = 0;
	if (state->is_rgb)
		remote_sd = &state->rgb.sensor.sd;
	if (state->is_depth)
		remote_sd = &state->depth.sensor.sd;
	if (state->is_y8)
		remote_sd = &state->ir.sensor.sd;
	if (state->is_imu)
		remote_sd = &state->imu.sensor.sd;
	/* Locks internally */
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 10)
	ret = ds5_sensor_enum_mbus_code(remote_sd, cfg, &tmp);
#else
	ret = ds5_sensor_enum_mbus_code(remote_sd, v4l2_state, &tmp);
#endif
	if (!ret)
		mce->code = tmp.code;

	return ret;
}
/* No locking needed */
static int ds5_mux_enum_frame_size(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 10)
				     struct v4l2_subdev_pad_config *cfg,
#else
				     struct v4l2_subdev_state *v4l2_state,
#endif
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct ds5 *state = container_of(sd, struct ds5, mux.sd.subdev);
	struct v4l2_subdev_frame_size_enum tmp = *fse;
	struct v4l2_subdev *remote_sd;
	u32 pad = fse->pad;
	int ret = -1;

	tmp.pad = 0;
	pad = ds5_state_to_pad(state);

	switch (pad) {
	case DS5_MUX_PAD_IR:
		remote_sd = &state->ir.sensor.sd;
		break;
	case DS5_MUX_PAD_DEPTH:
		remote_sd = &state->depth.sensor.sd;
		break;
	case DS5_MUX_PAD_RGB:
		remote_sd = &state->rgb.sensor.sd;
		break;
	case DS5_MUX_PAD_IMU:
		remote_sd = &state->imu.sensor.sd;
		break;
	case DS5_MUX_PAD_EXTERNAL:
		/*
		 * Assume, that different sensors don't support the same formats
		 * Try the Depth sensor first, then the Motion Tracker
		 */
		remote_sd = &state->depth.sensor.sd;
		ret = ds5_sensor_enum_frame_size(remote_sd, NULL, &tmp);
		if (!ret) {
			*fse = tmp;
			fse->pad = pad;
			return 0;
		}

		remote_sd = &state->ir.sensor.sd;
		break;
	default:
		return -EINVAL;
	}

	/* Locks internally */
	ret = ds5_sensor_enum_frame_size(remote_sd, NULL, &tmp);
	if (!ret) {
		*fse = tmp;
		fse->pad = pad;
	}

	return ret;
}

/* No locking needed */
static int ds5_mux_enum_frame_interval(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 10)
				     struct v4l2_subdev_pad_config *cfg,
#else
				     struct v4l2_subdev_state *v4l2_state,
#endif
				     struct v4l2_subdev_frame_interval_enum *fie)
{
	struct ds5 *state = container_of(sd, struct ds5, mux.sd.subdev);
	struct v4l2_subdev_frame_interval_enum tmp = *fie;
	struct v4l2_subdev *remote_sd;
	u32 pad = fie->pad;
	int ret = -1;

	tmp.pad = 0;

	dev_dbg(state->depth.sensor.sd.dev,
			"%s(): pad %d code %x width %d height %d\n",
			__func__, pad, tmp.code, tmp.width, tmp.height);

	pad = ds5_state_to_pad(state);

	switch (pad) {
	case DS5_MUX_PAD_IR:
		remote_sd = &state->ir.sensor.sd;
		break;
	case DS5_MUX_PAD_DEPTH:
		remote_sd = &state->depth.sensor.sd;
		break;
	case DS5_MUX_PAD_RGB:
		remote_sd = &state->rgb.sensor.sd;
		break;
	case DS5_MUX_PAD_IMU:
		remote_sd = &state->imu.sensor.sd;
		break;
	case DS5_MUX_PAD_EXTERNAL:
		/* Similar to ds5_mux_enum_frame_size() above */
		if (state->is_rgb)
			remote_sd = &state->rgb.sensor.sd;
		else
			remote_sd = &state->ir.sensor.sd;
		ret = ds5_sensor_enum_frame_interval(remote_sd, NULL, &tmp);
		if (!ret) {
			*fie = tmp;
			fie->pad = pad;
			return 0;
		}

		remote_sd = &state->ir.sensor.sd;
		break;
	default:
		return -EINVAL;
	}

	/* Locks internally */
	ret = ds5_sensor_enum_frame_interval(remote_sd, NULL, &tmp);
	if (!ret) {
		*fie = tmp;
		fie->pad = pad;
	}

	return ret;
}
static int ds5_mux_set_fmt(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 10)
		struct v4l2_subdev_pad_config *cfg,
#else
		struct v4l2_subdev_state *v4l2_state,
#endif
		struct v4l2_subdev_format *fmt)
{
	struct ds5 *state = container_of(sd, struct ds5, mux.sd.subdev);
	struct v4l2_mbus_framefmt *ffmt;

	struct ds5_sensor *sensor = state->mux.last_set;
	u32 pad = sensor->mux_pad;
	int ret = 0;

	int substream = -1;

	dev_dbg(sd->dev, "%s: fmt->pad:%d, sensor->mux_pad: %d, for sensor: %s\n",
			__func__, fmt->pad, pad, sensor->sd.name);

	if (pad != DS5_MUX_PAD_EXTERNAL)
		ds5_s_state_pad(state, pad);
	sensor = state->mux.last_set;
	switch (pad) {
	case DS5_MUX_PAD_DEPTH:
	case DS5_MUX_PAD_IR:
	case DS5_MUX_PAD_RGB:
	case DS5_MUX_PAD_IMU:
		ffmt = &sensor->format;
		break;
	case DS5_MUX_PAD_EXTERNAL:
		ffmt = &ds5_ffmts[pad];
		break;
	default:
		return -EINVAL;
	}

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		ffmt->width = fmt->format.width;
		ffmt->height = fmt->format.height;
		ffmt->code = fmt->format.code;
	}
	fmt->format = *ffmt;
	dev_dbg(sd->dev, "%s(): set fmt->pad:%d, sensor->mux_pad: %d, \
		code: 0x%x: %ux%u for sensor: %s\n",
		__func__,
		fmt->pad, pad, fmt->format.code,
		fmt->format.width, fmt->format.height,
		sensor->sd.name);

	return ret;
}
/* No locking needed */
static int ds5_mux_get_fmt(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 10)
				     struct v4l2_subdev_pad_config *cfg,
#else
				     struct v4l2_subdev_state *v4l2_state,
#endif
			   struct v4l2_subdev_format *fmt)
{
	struct ds5 *ds5 = container_of(sd, struct ds5, mux.sd.subdev);
	u32 pad = fmt->pad;
	int ret = 0;
	struct ds5_sensor *sensor = NULL;
	struct v4l2_mbus_framefmt *ffmt;

	mutex_lock(&ds5->mutex);

	// Source Pad
	if (pad == DS5_MUX_PAD_EXTERNAL) {
	// For external pad, we need to find which sink pad this stream maps to
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
	    // Get format from state for try format
	    ffmt = v4l2_subdev_state_get_format(v4l2_state, fmt->pad, fmt->stream);
	    if (!ffmt) {
		dev_err(sd->dev, "Failed to get try format for pad %u stream %u\n",
		    pad, fmt->stream);
		ret = -EINVAL;
		goto unlock;
	    }
	    fmt->format = *ffmt;
	} else {
	    // For active format, find the sensor based on routing
	    u32 sink_pad = 0;
	    bool route_found = false;

	    // Look up routing table to find which sink pad this stream maps to
	    if (v4l2_state) {
		struct v4l2_subdev_krouting *routing =
		    &v4l2_state->routing;

		for (unsigned int i = 0; i < routing->num_routes; i++) {
		    const struct v4l2_subdev_route *route = &routing->routes[i];

		    if (route->source_pad == fmt->pad &&
			route->source_stream == fmt->stream &&
			(route->flags & V4L2_SUBDEV_ROUTE_FL_ACTIVE)) {
			sink_pad = route->sink_pad;
			route_found = true;
			break;
		    }
		}
	    }

	    if (!route_found) {
		// Fallback to last_set if no routing found
		sensor = ds5->mux.last_set;
		dev_warn(sd->dev, "No route found for external pad stream %u, using last_set sensor %s\n",
		     fmt->stream, sensor ? sensor->sd.name : "none");
	    } else {
		// Get sensor based on sink pad from routing
		switch (sink_pad) {
		case DS5_MUX_PAD_DEPTH:
		    sensor = &ds5->depth.sensor;
		    break;
		case DS5_MUX_PAD_IR:
		    sensor = &ds5->ir.sensor;
		    break;
		case DS5_MUX_PAD_RGB:
		    sensor = &ds5->rgb.sensor;
		    break;
		case DS5_MUX_PAD_IMU:
		    sensor = &ds5->imu.sensor;
		    break;
		default:
		    dev_err(sd->dev, "Invalid sink pad %u for stream %u\n",
			sink_pad, fmt->stream);
		    ret = -EINVAL;
		    goto unlock;
		}

		dev_dbg(sd->dev, "Stream %u maps to sink pad %u, sensor %s\n",
		    fmt->stream, sink_pad, sensor->sd.name);
	    }

	    if (!sensor) {
		dev_err(sd->dev, "No sensor found for stream %u\n", fmt->stream);
		ret = -EINVAL;
		goto unlock;
	    }

	    fmt->format = sensor->format;
	}
    } else {
		// Sink pads
		switch (fmt->pad) {
		case DS5_MUX_PAD_DEPTH:
			sensor = &ds5->depth.sensor;
			break;
		case DS5_MUX_PAD_IR:
			sensor = &ds5->ir.sensor;
			break;
		case DS5_MUX_PAD_RGB:
			sensor = &ds5->rgb.sensor;
			break;
		case DS5_MUX_PAD_IMU:
			sensor = &ds5->imu.sensor;
			break;
		default:
			return -EINVAL;
		}

		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			ffmt = v4l2_subdev_state_get_format(v4l2_state, fmt->pad, fmt->stream);
			fmt->format = *ffmt;
		} else
			fmt->format = sensor->format;
	}
	ds5_s_state_pad(ds5, sensor->mux_pad);
unlock:
	mutex_unlock(&ds5->mutex);

	dev_dbg(sd->dev, "%s: fmt->pad:%d, stream:%u, width:%d, height:%d\n",
		__func__, fmt->pad, fmt->stream,
		fmt->format.width, fmt->format.height);

	return 0;
}
static unsigned int ds5_mbus_code_to_mipi(u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_RGB565_1X16:
		return MIPI_CSI2_DT_RGB565;
	case MEDIA_BUS_FMT_RGB888_1X24:
		return MIPI_CSI2_DT_RGB888;
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_YUYV8_1X16:
		return MIPI_CSI2_DT_YUV422_8B;
	case MEDIA_BUS_FMT_SBGGR16_1X16:
	case MEDIA_BUS_FMT_SGBRG16_1X16:
	case MEDIA_BUS_FMT_SGRBG16_1X16:
	case MEDIA_BUS_FMT_SRGGB16_1X16:
		return MIPI_CSI2_DT_RAW16;
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		return MIPI_CSI2_DT_RAW12;
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		return MIPI_CSI2_DT_RAW10;
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
		return MIPI_CSI2_DT_RAW8;
	default:
		/* return unavailable MIPI data type - 0x3f */
		WARN_ON(1);
		return 0x3f;
	}
}
static int ds5_mux_get_frame_desc(struct v4l2_subdev *sd,
	unsigned int pad, struct v4l2_mbus_frame_desc *desc)
{
	// return actual stream supported by sensor based on pad
    struct ds5 *ds5 = container_of(sd, struct ds5, mux.sd.subdev);
    struct v4l2_subdev_state *state;
    struct v4l2_subdev_krouting *routing;
    unsigned int i;
    int ret = 0;

    dev_dbg(sd->dev, "%s: pad %u\n", __func__, pad);

    // Only source pad (pad 0) should be queried for frame descriptor
    if (pad != DS5_MUX_PAD_EXTERNAL)
	return -EINVAL;

    // Get the current subdev state to access routing
    state = v4l2_subdev_get_locked_active_state(sd);
    if (!state) {
	dev_err(sd->dev, "Failed to get subdev state\n");
	return -EINVAL;
    }
    routing = &state->routing;
    if (!routing->num_routes) {
	dev_dbg(sd->dev, "No routes configured\n");
	desc->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	desc->num_entries = 0;
	return 0;
    }
    desc->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
    desc->num_entries = 0;

    // Iterate through all active routes for the source pad
    for (i = 0; i < routing->num_routes; i++) {
	const struct v4l2_subdev_route *route = &routing->routes[i];
	struct ds5_sensor *sensor = NULL;
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_mbus_frame_desc_entry *entry;

	// Skip if route is not active or not for our source pad
	if (!(route->flags & V4L2_SUBDEV_ROUTE_FL_ACTIVE) ||
	    route->source_pad != pad)
	    continue;

	// Find the sensor corresponding to the sink pad
	switch (route->sink_pad) {
	case DS5_MUX_PAD_DEPTH:
	    sensor = &ds5->depth.sensor;
	    break;
	case DS5_MUX_PAD_RGB:
	    sensor = &ds5->rgb.sensor;
	    break;
	case DS5_MUX_PAD_IR:
	    sensor = &ds5->ir.sensor;
	    break;
	case DS5_MUX_PAD_IMU:
	    sensor = &ds5->imu.sensor;
	    break;
	default:
	    dev_warn(sd->dev, "Invalid sink pad %u in route %u\n",
		route->sink_pad, i);
	    continue;
	}

	// Skip if sensor is not initialized
	if (!sensor->initialized)
	    continue;

	// Get format for this route
	fmt = v4l2_subdev_state_get_format(state, route->sink_pad, route->sink_stream);
	if (!fmt) {
	    dev_warn(sd->dev, "No format for pad %u stream %u\n",
		route->sink_pad, route->sink_stream);
	    continue;
	}

	// Add entry to frame descriptor
	if (desc->num_entries >= V4L2_FRAME_DESC_ENTRY_MAX) {
	    dev_warn(sd->dev, "Too many frame desc entries, truncating\n");
	    break;
	}

	       if (route->source_stream != sensor->stream_cfg.vc_id) {
		       dev_warn(sd->dev, "Expected Routing for sensor %s is stream %u\n",
			       sensor->sd.name, sensor->stream_cfg.vc_id);
	       }

	entry = &desc->entry[desc->num_entries];
	entry->flags = 0;
	entry->stream = route->source_stream;
	entry->pixelcode = fmt->code;

	// Calculate length based on format
	entry->length = fmt->width * fmt->height;
	switch (fmt->code) {
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_YUYV8_1X16:
	    entry->length *= 2; // 16 bits per pixel
	    break;
	case MEDIA_BUS_FMT_Y8_1X8:
	    entry->length *= 1; // 8 bits per pixel
	    break;
	case MEDIA_BUS_FMT_RGB888_1X24:
	    entry->length *= 3; // 24 bits per pixel
	    break;
	default:
	    entry->length *= 2; // Default to 16 bits per pixel
	    break;
	}

	// Set CSI-2 specific fields
	entry->bus.csi2.vc = sensor->stream_cfg.vc_id;
	// Map media bus format to CSI-2 data type
	if (sensor->config.format && sensor->config.format->data_type) {
	    entry->bus.csi2.dt = sensor->config.format->data_type;
	} else {
		entry->bus.csi2.dt = MIPI_CSI2_DT_YUV422_8B;
	}

	desc->num_entries++;

	dev_dbg(sd->dev, "Entry %u: stream %u, vc %u, dt 0x%02x, pixelcode 0x%04x, length %u\n",
	    desc->num_entries - 1, entry->stream,
	    entry->bus.csi2.vc, entry->bus.csi2.dt,
	    entry->pixelcode, entry->length);
    }

    dev_dbg(sd->dev, "%s: returning %u entries\n", __func__, desc->num_entries);

    return ret;
}

static int ds5_mux_get_frame_interval(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
/* pad ops */
		struct v4l2_subdev_state *sd_state,
		struct v4l2_subdev_frame_interval *fi)
#else
/* Video ops */
		struct v4l2_subdev_frame_interval *fi)
#endif
{
	struct ds5 *state = container_of(sd, struct ds5, mux.sd.subdev);
	struct ds5_sensor *sensor = NULL;

	if (NULL == sd || NULL == fi)
		return -EINVAL;

	sensor = state->mux.last_set;

	fi->interval.numerator = 1;
	fi->interval.denominator = sensor->config.framerate;

	dev_dbg(sd->dev, "%s(): %s %u\n", __func__, sd->name,
			fi->interval.denominator);

	return 0;
}

static int ds5_mux_set_frame_interval(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
/* pad ops */
		struct v4l2_subdev_state *sd_state,
		struct v4l2_subdev_frame_interval *fi)
#else
/* Video ops */
		struct v4l2_subdev_frame_interval *fi)
#endif
{
	struct ds5 *state = container_of(sd, struct ds5, mux.sd.subdev);
	struct ds5_sensor *sensor = NULL;
	u16 framerate = 1;

	if (NULL == sd || NULL == fi || fi->interval.numerator == 0)
		return -EINVAL;

	sensor = state->mux.last_set;

	framerate = fi->interval.denominator / fi->interval.numerator;
	framerate = __ds5_probe_framerate(sensor->config.resolution, framerate);
	sensor->config.framerate = framerate;
	fi->interval.numerator = 1;
	fi->interval.denominator = framerate;

	dev_dbg(sd->dev, "%s(): %s %u\n", __func__, sd->name, framerate);

	return 0;
}
static int ds5_mux_registered(struct v4l2_subdev *sd)
{
	struct ds5 *ds5 = v4l2_get_subdevdata(sd);
	int ret = ds5_sensor_register(ds5, &ds5->depth.sensor);

	if (ret < 0)
		return ret;

	ret = ds5_sensor_register(ds5, &ds5->ir.sensor);
	if (ret < 0)
		goto e_depth;

	ret = ds5_sensor_register(ds5, &ds5->rgb.sensor);
	if (ret < 0)
		goto e_rgb;

	ret = ds5_sensor_register(ds5, &ds5->imu.sensor);
	if (ret < 0)
		goto e_imu;

	return 0;

e_imu:
	v4l2_device_unregister_subdev(&ds5->rgb.sensor.sd);

e_rgb:
	v4l2_device_unregister_subdev(&ds5->ir.sensor.sd);

e_depth:
	v4l2_device_unregister_subdev(&ds5->depth.sensor.sd);

	return ret;
}

static void ds5_mux_unregistered(struct v4l2_subdev *sd)
{
	struct ds5 *ds5 = v4l2_get_subdevdata(sd);

	ds5_sensor_remove(&ds5->imu.sensor);
	ds5_sensor_remove(&ds5->rgb.sensor);
	ds5_sensor_remove(&ds5->ir.sensor);
	ds5_sensor_remove(&ds5->depth.sensor);
}
static int ds5_mux_set_routing(struct v4l2_subdev *sd,
	struct v4l2_subdev_state *state,
	enum v4l2_subdev_format_whence which,
	struct v4l2_subdev_krouting *routing)
{
	_mux_set_routing(sd, state, routing);

	return 0;
}

static int ds5_mux_init_state(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *state)
{
//

	struct ds5 *ds5 = container_of(sd, struct ds5, mux.sd.subdev);

	if (ds5->routing_initialized)
		return 0;

	struct v4l2_subdev_route mux_routes[] = {
		{
			.sink_pad = 1,
			.sink_stream = 0,
			.source_pad = 0,
			.source_stream = 0,
			.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		},
	};
	struct v4l2_subdev_krouting mux_routing = {
		.num_routes = ARRAY_SIZE(mux_routes),
		.routes = mux_routes,
	};

	return _mux_set_routing(sd, state, &mux_routing);
}
/// init configuration

static int ds5_sensor_config_init(struct i2c_client *client, struct ds5 *ds5)
{
	struct ds5_sensor *sensor;
	u16 cfg0 = 0, cfg0_md = 0, cfg1 = 0, cfg1_md = 0;
	u16 dw = 0, dh = 0, yw = 0, yh = 0, dev_type = 0;
	int ret;

	/* read back depth configuration */
	ret = ds5_read(ds5, DS5_DEPTH_STREAM_DT, &cfg0);
	if (!ret)
		ret = ds5_read(ds5, DS5_DEPTH_STREAM_MD, &cfg0_md);
	if (!ret)
		ret = ds5_read(ds5, DS5_DEPTH_RES_WIDTH, &dw);
	if (!ret)
		ret = ds5_read(ds5, DS5_DEPTH_RES_HEIGHT, &dh);

	/* read back IR configuration */
	if (!ret)
		ret = ds5_read(ds5, DS5_IR_STREAM_DT, &cfg1);
	if (!ret)
		ret = ds5_read(ds5, DS5_IR_STREAM_MD, &cfg1_md);
	if (!ret)
		ret = ds5_read(ds5, DS5_IR_RES_WIDTH, &yw);
	if (!ret)
		ret = ds5_read(ds5, DS5_IR_RES_HEIGHT, &yh);

	/* read back device type */
	if (!ret)
		ret = ds5_read(ds5, DS5_DEVICE_TYPE, &dev_type);
	if (ret < 0)
		return ret;

	dev_dbg(&client->dev, "%s: Depth cfg0 %x %ux%u cfg0_md %x %ux%u\n", __func__,
		 cfg0, dw, dh, cfg0_md, yw, yh);

	dev_dbg(&client->dev, "%s: IR cfg1 %x %ux%u cfg1_md %x %ux%u\n", __func__,
		 cfg1, dw, dh, cfg1_md, yw, yh);


	/* initialize ir sensor formats */
	sensor = &ds5->ir.sensor;

	sensor->formats = ds5->variant->formats;
	sensor->n_formats = ds5->variant->n_formats;
	sensor->mux_pad = DS5_MUX_PAD_IR;

	/* initialize rgb sensor formats */
	sensor = &ds5->rgb.sensor;
	switch (dev_type) {
	case DS5_DEVICE_TYPE_D43X:
	case DS5_DEVICE_TYPE_D46X:
		sensor->formats = &ds5_onsemi_rgb_format;
		sensor->n_formats = DS5_ONSEMI_RGB_N_FORMATS;
		break;
	case DS5_DEVICE_TYPE_D45X:
		sensor->formats = &ds5_rlt_rgb_format;
		sensor->n_formats = DS5_RLT_RGB_N_FORMATS;
		break;
	default:
		sensor->formats = &ds5_onsemi_rgb_format;
		sensor->n_formats = DS5_ONSEMI_RGB_N_FORMATS;
	}
	sensor->mux_pad = DS5_MUX_PAD_RGB;

	/* initialize imu sensor formats */
	sensor = &ds5->imu.sensor;

	/* For fimware version starting from: 5.16,
	   IMU will have 32bit axis values.
	   5.16.x.y = firmware version: 0x0510 */
	if (ds5->fw_version >= 0x510)
		sensor->formats = ds5_imu_formats_extended;
	else
		sensor->formats = ds5_imu_formats;
	sensor->n_formats = 1;
	sensor->mux_pad = DS5_MUX_PAD_IMU;

	/* Development: set a configuration during probing */
	if ((cfg0 & 0xff00) == 0x1800) {
		/* MIPI CSI-2 YUV420 isn't supported by V4L, reconfigure to Y8 */
		struct v4l2_subdev_format fmt = {
			.which = V4L2_SUBDEV_FORMAT_ACTIVE,
			.pad = 0,
			/* Use template to fill in .field, .colorspace etc. */
			.format = ds5_mbus_framefmt_template,
		};
#ifdef USE_Y
		/* Override .width, .height, .code */
		fmt.format.width = yw;
		fmt.format.height = yh;
		fmt.format.code = MEDIA_BUS_FMT_UYVY8_2X8;

		ds5->ir.sensor.streaming = true;
		ds5->depth.sensor.streaming = true;
		ret = __ds5_sensor_set_fmt(ds5, &ds5->ir.sensor, NULL, &fmt);
#else
		fmt.format.width = dw;
		fmt.format.height = dh;
		fmt.format.code = MEDIA_BUS_FMT_UYVY8_1X16;

		ds5->ir.sensor.streaming = false;
		ds5->depth.sensor.streaming = true;
		ret = __ds5_sensor_set_fmt(ds5, &ds5->depth.sensor, NULL, &fmt);
#endif
		if (ret < 0)
			return ret;
	}

	/* initialize depth sensor formats */
	sensor = &ds5->depth.sensor;
	switch (dev_type) {
	case DS5_DEVICE_TYPE_D43X:
	case DS5_DEVICE_TYPE_D45X:
		sensor->formats = ds5_depth_formats_d43x;
		break;
	case DS5_DEVICE_TYPE_D46X:
		sensor->formats = ds5_depth_formats_d46x;
		break;
	default:
		sensor->formats = ds5_depth_formats_d46x;
	}
	sensor->n_formats = 2;
	sensor->mux_pad = DS5_MUX_PAD_DEPTH;

	return 0;
}

static int ds5_sensor_set_stream(struct v4l2_subdev *sd, u64 streams_mask, int enable)
{
	struct ds5 *state = container_of(sd, struct ds5, mux.sd.subdev);
	struct ds5_sensor *sensor = container_of(sd, struct ds5_sensor, sd);
	int ret, vc_id, i;
	int restore_val;
	u16 streaming, status;
	u16 stream_status_base;
	u16 config_status_base;

	dev_dbg(sensor->sd.dev, "%s(): sensor: name=%s state=%d\n",
		__func__, sensor->sd.name, enable);

	for (vc_id = 0; vc_id < 8; vc_id++) {
		if (streams_mask & (1 << vc_id)) {
			break;
		}
	}
	if (vc_id == 8) {
		dev_err(&state->client->dev, "%s invalid streams_mask %llx\n", __func__, streams_mask);
		return -EINVAL;
	}

	sensor = vc_to_sensor(state, vc_id);
	if (sensor == NULL) {
		dev_err(&state->client->dev, "%s invalid vc_id %d\n", __func__, vc_id);
		return -EINVAL;
	}
	stream_status_base = sensor->stream_cfg.stream_status_base;
	config_status_base = sensor->stream_cfg.config_status_base;
	restore_val = sensor->streaming;

	sensor->streaming = enable;

	ds5_s_state_pad(state, sensor->mux_pad);

	if (enable) {
		ret = ds5_sub_configure(state, vc_id);

		if (ret)
			goto restore_s_state;

		dev_dbg(&state->client->dev, "%s: starting stream with VC %d. reg %x val %x\n",
			__func__, vc_id, DS5_START_STOP_STREAM, DS5_STREAM_START | sensor->stream_cfg.stream_id);

		ret = ds5_write(state, DS5_START_STOP_STREAM, DS5_STREAM_START | sensor->stream_cfg.stream_id);
		if (ret < 0) {
			dev_err(&state->client->dev, "%s: ds5 write DS5_STREAM_START failed\n", __func__);
			goto restore_s_state;
		}

		msleep(200);

		// check streaming status from FW
		for (i = 0; i < DS5_START_MAX_COUNT; i++) {
			ds5_read(state, stream_status_base, &streaming);
			ds5_read(state, config_status_base, &status);
			if ((status & DS5_STATUS_STREAMING) &&
					streaming == DS5_STREAM_STREAMING)
				break;

			msleep_range(DS5_START_POLL_TIME, DS5_START_POLL_TIME + 10);
		}

		if (i == DS5_START_MAX_COUNT) {
			dev_err(&state->client->dev,
				"%s start streaming failed, exit on timeout\n", __func__);
			/* notify fw */
			ret = ds5_write(state, DS5_START_STOP_STREAM,
					DS5_STREAM_STOP | sensor->stream_cfg.stream_id);
			ret = -EAGAIN;
			goto restore_s_state;
		} else {
			dev_dbg(&state->client->dev, "%s started after %dms\n",
				__func__, i * DS5_START_POLL_TIME);
		}
	} else {
		dev_dbg(&state->client->dev, "%s: stopping stream. reg %x val %x\n",
			__func__, DS5_START_STOP_STREAM, DS5_STREAM_STOP | sensor->stream_cfg.stream_id);

		ret = ds5_write(state, DS5_START_STOP_STREAM, DS5_STREAM_STOP | sensor->stream_cfg.stream_id);
		if (ret < 0) {
			dev_err(&state->client->dev, "%s: ds5 write DS5_STREAM_STOP failed\n", __func__);
			goto restore_s_state;
		}

		u16 depth_status, rgb_status, ir_status, imu_status;
	ds5_read(state, DS5_DEPTH_STREAM_STATUS, &depth_status);
	ds5_read(state, DS5_RGB_STREAM_STATUS, &rgb_status);
	ds5_read(state, DS5_IR_STREAM_STATUS, &ir_status);
	ds5_read(state, DS5_IMU_STREAM_STATUS, &imu_status);
		dev_dbg(&state->client->dev, "%s: after stop stream: depth status %x, rgb status %x, ir status %x, imu status %x\n",
			__func__, depth_status, rgb_status, ir_status, imu_status);

		u16 depth_config, rgb_config, ir_config, imu_config;

		ds5_read(state, DS5_DEPTH_CONFIG_STATUS, &depth_config);
	ds5_read(state, DS5_RGB_CONFIG_STATUS, &rgb_config);
	ds5_read(state, DS5_IR_CONFIG_STATUS, &ir_config);
	ds5_read(state, DS5_IMU_CONFIG_STATUS, &imu_config);
		dev_dbg(&state->client->dev, "%s: after stop stream: depth config %x, rgb config %x, ir config %x, imu config %x\n",
			__func__, depth_config, rgb_config, ir_config, imu_config);
	}

	ds5_read(state, config_status_base, &status);
	ds5_read(state, stream_status_base, &streaming);
	dev_dbg(&state->client->dev,
			"%s: %s %s, stream_status 0x%x:%x, config_status 0x%x:%x ret=%d\n",
			__func__, ds5_get_sensor_name(state),
			(enable)?"START":"STOP",
			stream_status_base, streaming,
			config_status_base, status, ret);

	return 0;

restore_s_state:
	ds5_read(state, config_status_base, &status);
	dev_err(&state->client->dev,
			"%s stream toggle failed! %x status 0x%04x\n",
			ds5_get_sensor_name(state), restore_val, status);

	sensor->streaming = restore_val;

	return ret;
}
static int ds5_enable_streams(struct v4l2_subdev *subdev,
	struct v4l2_subdev_state *state,
	u32 pad, u64 streams_mask)
{
	struct ds5 *ds5 = container_of(subdev, struct ds5, mux.sd.subdev);

	ds5_sensor_set_stream(subdev, streams_mask, true);

	return 0;
}
static int ds5_disable_streams(struct v4l2_subdev *subdev,
	 struct v4l2_subdev_state *state,
	 u32 pad, u64 streams_mask)
{
	ds5_sensor_set_stream(subdev, streams_mask, false);

	return 0;
}

// v4l2 ops for all
static const struct v4l2_subdev_video_ops ds5_sensor_video_ops = {
	.s_stream		= ds5_sensor_s_stream,
};

static const struct v4l2_subdev_internal_ops ds5_sensor_internal_ops = {
	.open = ds5_mux_open,
	.close = ds5_mux_close,
};

// v4l2 ops for depth
static const struct v4l2_subdev_pad_ops ds5_depth_pad_ops = {
	.enum_mbus_code	= ds5_sensor_enum_mbus_code,
	.enum_frame_size = ds5_sensor_enum_frame_size,
	.enum_frame_interval = ds5_sensor_enum_frame_interval,
	.get_fmt		= ds5_sensor_get_fmt,
	.set_fmt		= ds5_sensor_set_fmt,
	.enable_streams = ds5_enable_streams,
	.disable_streams = ds5_disable_streams,
	.get_frame_interval = ds5_sensor_get_frame_interval,
	.set_frame_interval = ds5_sensor_set_frame_interval,
};
static const struct v4l2_subdev_ops ds5_depth_subdev_ops = {
	.pad = &ds5_depth_pad_ops,
	.video = &ds5_sensor_video_ops,
};
// v4l2 ops for ir
static const struct v4l2_subdev_pad_ops ds5_ir_pad_ops = {
	.enum_mbus_code	= ds5_sensor_enum_mbus_code,
	.enum_frame_size = ds5_sensor_enum_frame_size,
	.enum_frame_interval = ds5_sensor_enum_frame_interval,
	.get_fmt		= ds5_sensor_get_fmt,
	.set_fmt		= ds5_sensor_set_fmt,
	.enable_streams = ds5_enable_streams,
	.disable_streams = ds5_disable_streams,
	.get_frame_interval = ds5_sensor_get_frame_interval,
	.set_frame_interval = ds5_sensor_set_frame_interval,
};
static const struct v4l2_subdev_ops ds5_ir_subdev_ops = {
	.pad = &ds5_ir_pad_ops,
	.video = &ds5_sensor_video_ops,
};
// v4l2 ops for rgb
static const struct v4l2_subdev_pad_ops ds5_rgb_pad_ops = {
	.enum_mbus_code	= ds5_sensor_enum_mbus_code,
	.enum_frame_size = ds5_sensor_enum_frame_size,
	.enum_frame_interval = ds5_sensor_enum_frame_interval,
	.get_fmt		= ds5_sensor_get_fmt,
	.set_fmt		= ds5_sensor_set_fmt,
	.enable_streams = ds5_enable_streams,
	.disable_streams = ds5_disable_streams,
	.get_frame_interval = ds5_sensor_get_frame_interval,
	.set_frame_interval = ds5_sensor_set_frame_interval,
};
static const struct v4l2_subdev_ops ds5_rgb_subdev_ops = {
	.pad = &ds5_rgb_pad_ops,
	.video = &ds5_sensor_video_ops,
};
// v4l2 ops for imu
static const struct v4l2_subdev_pad_ops ds5_imu_pad_ops = {
	.enum_mbus_code	= ds5_sensor_enum_mbus_code,
	.enum_frame_size = ds5_sensor_enum_frame_size,
	.enum_frame_interval = ds5_sensor_enum_frame_interval,
	.get_fmt		= ds5_sensor_get_fmt,
	.set_fmt		= ds5_sensor_set_fmt,
	.enable_streams = ds5_enable_streams,
	.disable_streams = ds5_disable_streams,
	.get_frame_interval = ds5_sensor_get_frame_interval,
	.set_frame_interval = ds5_sensor_set_frame_interval,
};
static const struct v4l2_subdev_ops ds5_imu_subdev_ops = {
	.pad = &ds5_imu_pad_ops,
	.video = &ds5_sensor_video_ops,
};

// v4l2 ops for mux
static const struct v4l2_subdev_internal_ops ds5_mux_internal_ops = {
	.open = ds5_mux_open,
	.close = ds5_mux_close,
	.registered = ds5_mux_registered,
	.unregistered = ds5_mux_unregistered,
	.init_state = ds5_mux_init_state,
};

static const struct v4l2_subdev_pad_ops ds5_mux_pad_ops = {
	.enum_mbus_code	= ds5_mux_enum_mbus_code,
	.set_fmt		= ds5_mux_set_fmt,
	.get_fmt		= ds5_mux_get_fmt,
	.get_frame_desc		= ds5_mux_get_frame_desc,
	.set_routing 	= ds5_mux_set_routing,
	.enable_streams = ds5_enable_streams,
	.disable_streams = ds5_disable_streams,
};

static const struct v4l2_subdev_core_ops ds5_mux_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
};
static const struct v4l2_subdev_ops ds5_mux_subdev_ops = {
	.core = &ds5_mux_core_ops,
	.pad = &ds5_mux_pad_ops,
};

/* This is needed for .get_fmt()
 * and if streaming is started without .set_fmt()
 */
static void ds5_sensor_format_init(struct ds5_sensor *sensor)
{
	const struct ds5_format *fmt;
	struct v4l2_mbus_framefmt *ffmt;
	unsigned int i;

	if (sensor->config.format)
		return;

	dev_dbg(sensor->sd.dev, "%s(): on pad %u\n", __func__, sensor->mux_pad);

	ffmt = &sensor->format;
	*ffmt = ds5_mbus_framefmt_template;
	/* Use the first format */
	fmt = sensor->formats;
	ffmt->code = fmt->mbus_code;
	/* and the first resolution */
	ffmt->width = fmt->resolutions->width;
	ffmt->height = fmt->resolutions->height;

	sensor->config.format = fmt;
	sensor->config.resolution = fmt->resolutions;
	/* Set default framerate to 30, or to 1st one if not supported */
	for (i = 0; i < fmt->resolutions->n_framerates; i++) {
		if (fmt->resolutions->framerates[i] == ds5_framerate_30 /* fps */) {
			sensor->config.framerate = ds5_framerate_30;
			return;
		}
	}
	sensor->config.framerate = fmt->resolutions->framerates[0];
}
static int ds5_hw_init(struct i2c_client *client, struct ds5 *ds5)
{
	struct v4l2_subdev *sd = &ds5->mux.sd.subdev;
	u16 mipi_status, n_lanes, phy, drate_min, drate_max;
	int ret;


	ds5_read_with_check(ds5, DS5_MIPI_SUPPORT_LINES, &n_lanes);

	ds5_read_with_check(ds5, DS5_MIPI_SUPPORT_PHY, &phy);

	ds5_read_with_check(ds5, DS5_MIPI_DATARATE_MIN, &drate_min);

	ds5_read_with_check(ds5, DS5_MIPI_DATARATE_MAX, &drate_max);

	dev_info(sd->dev, "%s: supported lanes: %u, supported phy: %x, data rate: %u-%u\n",
		__func__, n_lanes, phy, drate_min, drate_max);

	// fixed number of lane to 2
	n_lanes = 2;

	// configure mipi lane
	ret = ds5_write(ds5, DS5_MIPI_LANE_NUMS, n_lanes - 1);
	if (ret) {
		dev_err(sd->dev, "%s: set lane nums %u failed: %d\n",
			__func__, n_lanes, ret);
		return ret;
	}
	// configure mipi data rate
	ret = ds5_write(ds5, DS5_MIPI_LANE_DATARATE, MIPI_LANE_RATE);
	if (ret) {
		dev_err(sd->dev, "%s: set lane rate %u failed: %d\n",
			__func__, MIPI_LANE_RATE, ret);
		return ret;
	}

	// check mipi status
	ret = ds5_read(ds5, DS5_MIPI_CONF_STATUS, &mipi_status);
	if (ret) {
		dev_err(sd->dev, "%s: read mipi status failed: %d\n",
			__func__, ret);
		return ret;
	}

	dev_dbg(sd->dev, "%s: mipi status: 0x%x\n", __func__, mipi_status);

	u16 streaming, status, config_status_base, stream_status_base;

	// reset all streams to original state // val 0x107 (0x100 | 0x0 | 0x2 | 0x1 | 0x4)
	dev_dbg(&client->dev, "%s: reset all streams to original state\n", __func__);
	ret = ds5_write(ds5, DS5_START_STOP_STREAM, DS5_STREAM_STOP | DS5_STREAM_DEPTH);
	if (ret) {
		dev_err(&client->dev, "%s: failed to reset depth stream: %d\n", __func__, ret);
		return ret;
	}
	msleep(50);
	ret = ds5_write(ds5, DS5_START_STOP_STREAM, DS5_STREAM_STOP | DS5_STREAM_IR);
	if (ret) {
		dev_err(&client->dev, "%s: failed to reset IR stream: %d\n", __func__, ret);
		return ret;
	}
	msleep(50);
	ret = ds5_write(ds5, DS5_START_STOP_STREAM, DS5_STREAM_STOP | DS5_STREAM_RGB);
	if (ret) {
		dev_err(&client->dev, "%s: failed to reset RGB stream: %d\n", __func__, ret);
		return ret;
	}
	msleep(50);
	ret = ds5_write(ds5, DS5_START_STOP_STREAM, DS5_STREAM_STOP | DS5_STREAM_IMU);
	if (ret) {
		dev_err(&client->dev, "%s: failed to reset all streams: %d\n", __func__, ret);
		return ret;
	}
	msleep(50);

	stream_status_base = DS5_DEPTH_STREAM_STATUS;
	config_status_base = DS5_DEPTH_CONFIG_STATUS;

	// check streaming status from FW
	for (int i = 0; i < DS5_START_MAX_COUNT; i++) {
		ds5_read(ds5, stream_status_base, &streaming);
		ds5_read(ds5, config_status_base, &status);
		if ((status & DS5_STATUS_STREAMING) &&
				streaming == DS5_STREAM_STREAMING)
			break;

		msleep_range(DS5_START_POLL_TIME);
	}

	dev_dbg(&client->dev, "%s: DEPTH stream_status 0x%x:%x, config_status 0x%x:%x ret=%d\n",
		__func__, stream_status_base, streaming,
		config_status_base, status, ret);

	stream_status_base = DS5_IR_STREAM_STATUS;
	config_status_base = DS5_IR_CONFIG_STATUS;
	ds5_read(ds5, config_status_base, &status);
	ds5_read(ds5, stream_status_base, &streaming);

	dev_dbg(&client->dev, "%s: IR stream_status 0x%x:%x, config_status 0x%x:%x ret=%d\n",
		__func__, stream_status_base, streaming,
		config_status_base, status, ret);

	stream_status_base = DS5_RGB_STREAM_STATUS;
	config_status_base = DS5_RGB_CONFIG_STATUS;
	ds5_read(ds5, config_status_base, &status);
	ds5_read(ds5, stream_status_base, &streaming);

	dev_dbg(&client->dev, "%s: RGB stream_status 0x%x:%x, config_status 0x%x:%x ret=%d\n",
		__func__, stream_status_base, streaming,
		config_status_base, status, ret);

	stream_status_base = DS5_IMU_STREAM_STATUS;
	config_status_base = DS5_IMU_CONFIG_STATUS;
	ds5_read(ds5, config_status_base, &status);
	ds5_read(ds5, stream_status_base, &streaming);

	dev_dbg(&client->dev, "%s: IMU stream_status 0x%x:%x, config_status 0x%x:%x ret=%d\n",
		__func__, stream_status_base, streaming,
		config_status_base, status, ret);

	return ret;
}

static int ds5_hw_set_auto_exposure(struct ds5 *state, u32 base, s32 val)
{
	dev_dbg(&state->client->dev, "%s: entry: %d\n", __func__, val);

	if (val != V4L2_EXPOSURE_APERTURE_PRIORITY &&
		val != V4L2_EXPOSURE_MANUAL)
		return -EINVAL;

	/*
	 * In firmware color auto exposure setting follow the uvc_menu_info
	 * exposure_auto_controls numbers, in drivers/media/usb/uvc/uvc_ctrl.c.
	 */
	if (state->is_rgb && val == V4L2_EXPOSURE_APERTURE_PRIORITY)
		val = 8;

	/*
	 * In firmware depth auto exposure on: 1, off: 0.
	 */
	if (!state->is_rgb) {
		if (val == V4L2_EXPOSURE_APERTURE_PRIORITY)
			val = 1;
		else if (val == V4L2_EXPOSURE_MANUAL)
			val = 0;
	}

	return ds5_write(state, base | DS5_AUTO_EXPOSURE_MODE, (u16)val);
}

/*
 * Manual exposure in us
 * Depth/Y8: between 100 and 200000 (200ms)
 * Color: between 100 and 1000000 (1s)
 */
static int ds5_hw_set_exposure(struct ds5 *state, u32 base, s32 val)
{
	int ret = -1;

	dev_dbg(&state->client->dev, "%s: entry: %d\n", __func__, val);

	if (val < 1)
		val = 1;
	if ((state->is_depth || state->is_y8) && val > MAX_DEPTH_EXP)
		val = MAX_DEPTH_EXP;
	if (state->is_rgb && val > MAX_RGB_EXP)
		val = MAX_RGB_EXP;

	/*
	 * Color and depth uses different unit:
	 *	Color: 1 is 100 us
	 *	Depth: 1 is 1 us
	 */

	ret = ds5_write(state, base | DS5_MANUAL_EXPOSURE_MSB, (u16)(val >> 16));
	if (!ret)
		ret = ds5_write(state, base | DS5_MANUAL_EXPOSURE_LSB,
				(u16)(val & 0xffff));


	return ret;
}
static int ds5_set_calibration_data(struct ds5 *state,
		struct hwm_cmd *cmd, u16 length)
{
	int ret = -1;
	int retries = 10;
	u16 status = 2;


	ds5_raw_write_with_check(state, DS5_HWMC_DATA, cmd, length);

	ds5_write_with_check(state, DS5_HWMC_EXEC, 0x01); /* execute cmd */
	do {
		if (retries != 10)
			msleep_range(200);
		ret = ds5_read(state, DS5_HWMC_STATUS, &status);
	} while (retries-- && status != 0);

	if (ret || status != 0) {
		dev_err(&state->client->dev,
				"%s(): Failed to set calibration table %d,ret: %d, fw error: %x\n",
				__func__, cmd->param1, ret, status);
	}

	return ret;
}
static int ds5_get_calibration_data(struct ds5 *state, enum table_id id,
		unsigned char *table, unsigned int length)
{
	struct hwm_cmd *cmd;
	int ret = -1;
	int retries = 3;
	u16 status = 2;
	u16 table_length;

	dev_dbg(&state->client->dev, "%s: get calibration table %d, length %d\n", __func__, id, length);

	cmd = devm_kzalloc(&state->client->dev,
			sizeof(struct hwm_cmd) + length + 4, GFP_KERNEL);
	if (!cmd) {
		dev_err(&state->client->dev, "%s: Can't allocate memory\n", __func__);
		return -ENOMEM;
	}

	memcpy(cmd, &get_calib_data, sizeof(get_calib_data));
	cmd->param1 = id;
	ds5_raw_write_with_check(state, 0x4900, cmd, sizeof(struct hwm_cmd));
	ds5_write_with_check(state, 0x490c, 0x01); /* execute cmd */
	do {
		if (retries != 3)
			msleep_range(10);
		ret = ds5_read(state, 0x4904, &status);
	} while (ret && retries-- && status != 0);

	if (ret || status != 0) {
		dev_err(&state->client->dev,
				"%s(): Failed to get calibration table %d, fw error: %x\n",
				__func__, id, status);
		devm_kfree(&state->client->dev, cmd);
		return status;
	}

	// get table length from fw
	ret = regmap_raw_read(state->regmap, 0x4908,
			&table_length, sizeof(table_length));

	// read table
	ds5_raw_read_with_check(state, 0x4900, cmd->Data, table_length);

	// first 4 bytes are opcode HWM, not part of calibration table
	memcpy(table, cmd->Data + 4, length);
	devm_kfree(&state->client->dev, cmd);


	return 0;
}
/* HWMC functions */
static int ds5_get_hwmc_status(struct ds5 *state)
{
	int ret = 0;
	u16 status = DS5_HWMC_STATUS_WIP;
	int retries = 100;
	int errorCode;


	do {
		if (retries != 100)
			msleep_range(1);
		ret = ds5_read(state, DS5_HWMC_STATUS, &status);
	} while (!ret && retries-- && status == DS5_HWMC_STATUS_WIP);

	dev_info(&state->client->dev, "%s(): ret: 0x%x, status: 0x%x\n", __func__, ret, status);

	if (ret || status != DS5_HWMC_STATUS_OK) {
		if (status == DS5_HWMC_STATUS_ERR) {
			ds5_raw_read(state, DS5_HWMC_DATA, &errorCode, sizeof(errorCode));
			switch (errorCode) {
			case (DS5_HWMC_ERR_CMD):
			case (DS5_HWMC_ERR_PARAM):
				ret = -EBADMSG;
			break;
			case (DS5_HWMC_ERR_NODATA):
				ret = -ENODATA;
			break;

			default:
				ret = -EBADMSG;
				break;
			}

			dev_err(&state->client->dev,
				"%s: HWMC failed, ret: %d, status: %x, error code: %d\n",
				__func__, ret, status, errorCode);
		}
	}
	if (!ret && (status != DS5_HWMC_STATUS_OK)) {
		dev_err(&state->client->dev, "%s: HWMC status not OK: %x\n", __func__, status);
		ret = -EBUSY;
	}


	return ret;
}

static int ds5_get_hwmc(struct ds5 *state, unsigned char *data,
		u16 cmdDataLen, u16 *dataLen)
{
	int ret = 0;
	u16 tmp_len = 0;


	if (!data)
		return -ENOBUFS;

	memset(data, 0, cmdDataLen);
	ret = ds5_get_hwmc_status(state);
	if (ret) {
		dev_dbg(&state->client->dev,
			"%s: HWMC status not clear, ret: %d\n",
			__func__, ret);
			return ret;
	}

	ret = regmap_raw_read(state->regmap, DS5_HWMC_RESP_LEN,
			&tmp_len, sizeof(tmp_len));
	if (ret) {
		dev_err(&state->client->dev,
			"%s: Can't read HWMC response length, ret: %d\n",
			__func__, ret);
		return -EBADMSG;
	}

	if (tmp_len > cmdDataLen) {
		dev_err(&state->client->dev,
			"%s: HWMC response length too big: %d > %d\n",
			__func__, tmp_len, cmdDataLen);
		return -ENOBUFS;
	}

	dev_dbg(&state->client->dev,
			"%s: HWMC read len: %d, lrs_len: %d\n",
			__func__, tmp_len, tmp_len - 4);

	ds5_raw_read_with_check(state, DS5_HWMC_DATA, data, tmp_len);
	if (dataLen)
		*dataLen = tmp_len;

	return ret;
}
static int ds5_send_hwmc(struct ds5 *state,
			u16 cmdLen,
			struct hwm_cmd *cmd)
{
	dev_dbg(&state->client->dev,
			"%s(): HWMC header: 0x%x, magic: 0x%x, opcode: 0x%x, cmdLen: %d, param1: %d, param2: %d, param3: %d, param4: %d\n",
			__func__, cmd->header, cmd->magic_word, cmd->opcode,
			cmdLen,	cmd->param1, cmd->param2, cmd->param3, cmd->param4);

	ds5_raw_write_with_check(state, DS5_HWMC_DATA, cmd, cmdLen);

	ds5_write_with_check(state, DS5_HWMC_EXEC, 0x01); /* execute cmd */

	return 0;
}

static int ds5_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ds5 *state = container_of(ctrl->handler, struct ds5,
					 ctrls.handler);
	struct v4l2_subdev *sd = &state->mux.sd.subdev;
	struct ds5_sensor *sensor = (struct ds5_sensor *)ctrl->priv;
	int ret = -EINVAL;
	u16 base = DS5_DEPTH_CONTROL_BASE;

	if (sensor) {
		switch (sensor->mux_pad) {
		case DS5_MUX_PAD_DEPTH:
			state = container_of(ctrl->handler, struct ds5, ctrls.handler_depth);
			state->is_rgb = 0;
			state->is_depth = 1;
			state->is_y8 = 0;
			state->is_imu = 0;
		break;
		case DS5_MUX_PAD_RGB:
			state = container_of(ctrl->handler, struct ds5, ctrls.handler_rgb);
			state->is_rgb = 1;
			state->is_depth = 0;
			state->is_y8 = 0;
			state->is_imu = 0;
		break;
		case DS5_MUX_PAD_IR:
			state = container_of(ctrl->handler, struct ds5, ctrls.handler_y8);
			state->is_rgb = 0;
			state->is_depth = 0;
			state->is_y8 = 1;
			state->is_imu = 0;
		break;
		case DS5_MUX_PAD_IMU:
			state = container_of(ctrl->handler, struct ds5, ctrls.handler_imu);
			state->is_rgb = 0;
			state->is_depth = 0;
			state->is_y8 = 0;
			state->is_imu = 1;
		break;
		default:
			state->is_rgb = 0;
			state->is_depth = 0;
			state->is_y8 = 0;
			state->is_imu = 1;
		break;

		}
	}

	if (state->is_rgb)
		base = DS5_RGB_CONTROL_BASE;
	else if (state->is_imu)
		return -EINVAL;

	v4l2_dbg(3, 1, sd, "ctrl: %s, value: %d\n", ctrl->name, ctrl->val);
	dev_dbg(&state->client->dev, "%s(): %s - ctrl: %s, value: %d\n",
		__func__, ds5_get_sensor_name(state), ctrl->name, ctrl->val);

	mutex_lock(&state->mutex);

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = ds5_write(state, base | DS5_MANUAL_GAIN, ctrl->val);
		break;

	case V4L2_CID_EXPOSURE_AUTO:
		ret = ds5_hw_set_auto_exposure(state, base, ctrl->val);
		break;

	case V4L2_CID_EXPOSURE_ABSOLUTE:
		ret = ds5_hw_set_exposure(state, base, ctrl->val);
		break;
	case DS5_CAMERA_CID_LASER_POWER:
		if (!state->is_rgb)
			ret = ds5_write(state, base | DS5_LASER_POWER,
					ctrl->val);
		break;
	case DS5_CAMERA_CID_MANUAL_LASER_POWER:
		if (!state->is_rgb)
			ret = ds5_write(state, base | DS5_MANUAL_LASER_POWER,
					ctrl->val);
		break;
	case DS5_CAMERA_DEPTH_CALIBRATION_TABLE_SET:
		dev_dbg(&state->client->dev,
			"%s(): DS5_CAMERA_DEPTH_CALIBRATION_TABLE_SET\n",	__func__);
		if (ctrl->p_new.p) {
			struct hwm_cmd *calib_cmd;

			dev_dbg(&state->client->dev,
				"%s(): table id: 0x%x\n",
				__func__, *((u8 *)ctrl->p_new.p + 2));
			if (DEPTH_CALIBRATION_ID == *((u8 *)ctrl->p_new.p + 2)) {
				calib_cmd = devm_kzalloc(&state->client->dev,
					sizeof(struct hwm_cmd) + 256, GFP_KERNEL);
				if (!calib_cmd) {
					dev_err(&state->client->dev,
						"%s(): Can't allocate memory for 0x%x\n",
						__func__, ctrl->id);
					ret = -ENOMEM;
					break;
				}
				memcpy(calib_cmd, &set_calib_data, sizeof(set_calib_data));
				calib_cmd->header = 276;
				calib_cmd->param1 = DEPTH_CALIBRATION_ID;
				memcpy(calib_cmd->Data, (u8 *)ctrl->p_new.p, 256);
				ret = ds5_set_calibration_data(state, calib_cmd,
					sizeof(struct hwm_cmd) + 256);
				devm_kfree(&state->client->dev, calib_cmd);
			}
		}
		break;
	case DS5_CAMERA_COEFF_CALIBRATION_TABLE_SET:
			dev_dbg(&state->client->dev,
				"%s(): DS5_CAMERA_COEFF_CALIBRATION_TABLE_SET\n",
				__func__);
			if (ctrl->p_new.p) {
				struct hwm_cmd *calib_cmd;

				dev_dbg(&state->client->dev,
					"%s(): table id %d\n",
					__func__, *((u8 *)ctrl->p_new.p + 2));
				if (COEF_CALIBRATION_ID == *((u8 *)ctrl->p_new.p + 2)) {
					calib_cmd = devm_kzalloc(&state->client->dev,
						sizeof(struct hwm_cmd) + 512, GFP_KERNEL);
					if (!calib_cmd) {
						dev_err(&state->client->dev,
							"%s(): Can't allocate memory for 0x%x\n",
							__func__, ctrl->id);
						ret = -ENOMEM;
						break;
					}
				memcpy(calib_cmd, &set_calib_data, sizeof(set_calib_data));
				calib_cmd->header = 532;
				calib_cmd->param1 = COEF_CALIBRATION_ID;
				memcpy(calib_cmd->Data, (u8 *)ctrl->p_new.p, 512);
				ret = ds5_set_calibration_data(state, calib_cmd,
						sizeof(struct hwm_cmd) + 512);
				devm_kfree(&state->client->dev, calib_cmd);
			}
		}
		break;
	case DS5_CAMERA_CID_AE_ROI_SET:
		if (ctrl->p_new.p_u16) {
			struct hwm_cmd ae_roi_cmd;

			memcpy(&ae_roi_cmd, &set_ae_roi, sizeof(ae_roi_cmd));
			ae_roi_cmd.param1 = *((u16 *)ctrl->p_new.p_u16);
			ae_roi_cmd.param2 = *((u16 *)ctrl->p_new.p_u16 + 1);
			ae_roi_cmd.param3 = *((u16 *)ctrl->p_new.p_u16 + 2);
			ae_roi_cmd.param4 = *((u16 *)ctrl->p_new.p_u16 + 3);
			ret = ds5_send_hwmc(state, sizeof(struct hwm_cmd),
				&ae_roi_cmd);
			if (!ret)
				ret = ds5_get_hwmc_status(state);
		}
		break;
	case DS5_CAMERA_CID_AE_SETPOINT_SET:
		if (ctrl->p_new.p_s32) {
			struct hwm_cmd *ae_setpoint_cmd;

			dev_dbg(&state->client->dev, "%s():0x%x\n",
				__func__, *(ctrl->p_new.p_s32));
			ae_setpoint_cmd = devm_kzalloc(&state->client->dev,
					sizeof(struct hwm_cmd) + 4, GFP_KERNEL);
			if (!ae_setpoint_cmd) {
				dev_err(&state->client->dev,
					"%s(): Can't allocate memory for 0x%x\n",
					__func__, ctrl->id);
				ret = -ENOMEM;
				break;
			}
			memcpy(ae_setpoint_cmd, &set_ae_setpoint, sizeof(set_ae_setpoint));
			memcpy(ae_setpoint_cmd->Data, (u8 *)ctrl->p_new.p_s32, 4);
			ret = ds5_send_hwmc(state, sizeof(struct hwm_cmd) + 4,
					ae_setpoint_cmd);
			if (!ret)
				ret = ds5_get_hwmc_status(state);
			devm_kfree(&state->client->dev, ae_setpoint_cmd);
		}
		break;
	case DS5_CAMERA_CID_ERB:
		if (ctrl->p_new.p_u8) {
			u16 offset = 0;
			u16 size = 0;
			u16 len = 0;
			struct hwm_cmd *erb_cmd;

			offset = *(ctrl->p_new.p_u8) << 8;
			offset |= *(ctrl->p_new.p_u8 + 1);
			size = *(ctrl->p_new.p_u8 + 2) << 8;
			size |= *(ctrl->p_new.p_u8 + 3);

			dev_dbg(&state->client->dev, "%s(): offset %x, size: %x\n",
							__func__, offset, size);
			len = sizeof(struct hwm_cmd) + size;
			erb_cmd = devm_kzalloc(&state->client->dev,	len, GFP_KERNEL);
			if (!erb_cmd) {
				dev_err(&state->client->dev,
					"%s(): Can't allocate memory for 0x%x\n",
					__func__, ctrl->id);
				ret = -ENOMEM;
				break;
			}
			memcpy(erb_cmd, &erb, sizeof(struct hwm_cmd));
			erb_cmd->param1 = offset;
			erb_cmd->param2 = size;
			ret = ds5_send_hwmc(state, sizeof(struct hwm_cmd), erb_cmd);
			if (!ret)
				ret = ds5_get_hwmc(state, erb_cmd->Data, len, &size);
			if (ret) {
				dev_err(&state->client->dev,
					"%s(): ERB cmd failed, ret: %d,requested size: %d, actual size: %d\n",
					__func__, ret, erb_cmd->param2, size);
				devm_kfree(&state->client->dev, erb_cmd);
				return -EAGAIN;
			}

			// Actual size returned from FW
			*(ctrl->p_new.p_u8 + 2) = (size & 0xFF00) >> 8;
			*(ctrl->p_new.p_u8 + 3) = (size & 0x00FF);

			memcpy(ctrl->p_new.p_u8 + 4, erb_cmd->Data + 4, size - 4);
			dev_dbg(&state->client->dev, "%s(): 0x%x 0x%x 0x%x 0x%x\n",
				__func__,
				*(ctrl->p_new.p_u8),
				*(ctrl->p_new.p_u8+1),
				*(ctrl->p_new.p_u8+2),
				*(ctrl->p_new.p_u8+3));
			devm_kfree(&state->client->dev, erb_cmd);
		}
		break;
	case DS5_CAMERA_CID_EWB:
		if (ctrl->p_new.p_u8) {
			u16 offset = 0;
			u16 size = 0;
			struct hwm_cmd *ewb_cmd;

			offset = *((u8 *)ctrl->p_new.p_u8) << 8;
			offset |= *((u8 *)ctrl->p_new.p_u8 + 1);
			size = *((u8 *)ctrl->p_new.p_u8 + 2) << 8;
			size |= *((u8 *)ctrl->p_new.p_u8 + 3);

			dev_dbg(&state->client->dev, "%s():0x%x 0x%x 0x%x 0x%x\n",
					__func__,
					*((u8 *)ctrl->p_new.p_u8),
					*((u8 *)ctrl->p_new.p_u8 + 1),
					*((u8 *)ctrl->p_new.p_u8 + 2),
					*((u8 *)ctrl->p_new.p_u8 + 3));

			ewb_cmd = devm_kzalloc(&state->client->dev,
					sizeof(struct hwm_cmd) + size,
					GFP_KERNEL);
			if (!ewb_cmd) {
				dev_err(&state->client->dev,
					"%s(): Can't allocate memory for 0x%x\n",
					__func__, ctrl->id);
				ret = -ENOMEM;
				break;
			}
			memcpy(ewb_cmd, &ewb, sizeof(ewb));
			ewb_cmd->header = 0x14 + size;
			ewb_cmd->param1 = offset; // start index
			ewb_cmd->param2 = size; // size
			memcpy(ewb_cmd->Data, (u8 *)ctrl->p_new.p_u8 + 4, size);
			ret = ds5_send_hwmc(state, sizeof(struct hwm_cmd) + size, ewb_cmd);
			if (!ret)
				ret = ds5_get_hwmc_status(state);
			if (ret) {
				dev_err(&state->client->dev,
					"%s(): EWB cmd failed, ret: %d,requested size: %d, actual size: %d\n",
					__func__, ret, ewb_cmd->param2, size);
				devm_kfree(&state->client->dev, ewb_cmd);
				return -EAGAIN;
			}

			devm_kfree(&state->client->dev, ewb_cmd);
		}
		break;
	case DS5_CAMERA_CID_HWMC:
		if (ctrl->p_new.p_u8) {
			u16 size = 0;
			struct hwm_cmd *cmd = (struct hwm_cmd *)ctrl->p_new.p_u8;

			size = *((u8 *)ctrl->p_new.p_u8 + 1) << 8;
			size |= *((u8 *)ctrl->p_new.p_u8 + 0);
			ret = ds5_send_hwmc(state, size + 4, cmd);
			ret = ds5_get_hwmc(state, cmd->Data, ctrl->dims[0], &size);
			if (ctrl->dims[0] < DS5_HWMC_BUFFER_SIZE) {
				ret = -ENODATA;
				break;
			}
			/*This is needed for legacy hwmc */
			size += 4; // SIZE_OF_HW_MONITOR_HEADER
			cmd->Data[1000] = (unsigned char)((size) & 0x00FF);
			cmd->Data[1001] = (unsigned char)(((size) & 0xFF00) >> 8);
		}
		break;
	case DS5_CAMERA_CID_HWMC_RW:
		if (ctrl->p_new.p_u8) {
			u16 size = *((u8 *)ctrl->p_new.p_u8 + 1) << 8;

			size |= *((u8 *)ctrl->p_new.p_u8 + 0);
			ret = ds5_send_hwmc(state, size + 4,
					(struct hwm_cmd *)ctrl->p_new.p_u8);
		}
		break;
	case DS5_CAMERA_CID_PWM:
		if (state->is_depth)
			ret = ds5_write(state, base | DS5_PWM_FREQUENCY, ctrl->val);
		break;
	case V4L2_CID_LINK_FREQ: {
		if (!sensor && ctrl->p_new.p_u8) {
		  /* MTL and RPL/ADL IPU6 CSI-DPHY do NOT share
		   *  the same default link_freq.
		   * V4L2_CID_LINK_FREQ DS5 mux must be R/W for udev ot set DPHY platform specific link_freq
		   * via systemd-udevd rules.
		   */
		if (*ctrl->p_new.p_u8 <= (ARRAY_SIZE(link_freq_menu_items) - 1)) {
			struct v4l2_ctrl *link_freq = state->ctrls.link_freq;

			dev_info(&state->client->dev,
				"user-modified %s index val=%d to user-val=%d",
				 ctrl->name,
				 (unsigned int) link_freq->val,
				 (unsigned int) *ctrl->p_new.p_u8);
			link_freq->val = (s32) *ctrl->p_new.p_u8;
			ret = 0;
		}
		}

	}
	break;
	}

	mutex_unlock(&state->mutex);

	return ret;
}
static int ds5_gvd(struct ds5 *state, unsigned char *data)
{
	struct hwm_cmd cmd;
	int ret = -1;
	u16 length = 0;
	u16 status = 2;
	u8 retries = 3;

	memcpy(&cmd, &gvd, sizeof(gvd));
	ds5_raw_write_with_check(state, 0x4900, &cmd, sizeof(cmd));
	ds5_write_with_check(state, 0x490c, 0x01); /* execute cmd */
	do {
		if (retries != 3)
			msleep_range(10);

		ret = ds5_read(state, 0x4904, &status);
	} while (ret && retries-- && status != 0);

	if (ret || status != 0) {
		dev_err(&state->client->dev,
				"%s(): Failed to read GVD, HWM cmd status: %x\n",
				__func__, status);
		return status;
	}

	ret = regmap_raw_read(state->regmap, 0x4908, &length, sizeof(length));
	ds5_raw_read_with_check(state, 0x4900, data, length);

	return ret;
}

// ds5 read from register
static int ds5_get_state(struct ds5 *ds5, u16 *rval)
{
	struct i2c_client *client = ds5->client;
	int ret;
	int retry = 5;

	do {
		ret = ds5_read(ds5, DS5_REG_SENSOR_STATE, rval);
	} while (retry-- && ret < 0);

	if (ret) {
		dev_err(&client->dev, "failed to communicate with D4XX: %d on addr 0x%04x", ret, client->addr);
		return ret;
	}
	return 0;
}
static int ds5_get_fw_info(struct ds5 *ds5, u16 *fw_version, u16 *fw_build)
{
	struct i2c_client *client = ds5->client;
	int ret;
	int retry = 5;
	u16 version, build;


	ds5_read_with_check(ds5, DS5_FW_VERSION, &version);
	ds5_read_with_check(ds5, DS5_FW_BUILD, &build);

	dev_info(&client->dev, "D4XX Sensor: %s, firmware build: %d.%d.%d.%d\n",
		 ds5_get_sensor_name(ds5),
		 (version >> 8) & 0xFF,
		 version & 0xFF,
		 (build >> 8) & 0xFF,
		 build & 0xFF);

	*fw_version = version;
	*fw_build = build;

	return 0;
}
static int ds5_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ds5 *state = container_of(ctrl->handler, struct ds5,
			ctrls.handler);
	u16 log_prepare[] = {0x0014, 0xcdab, 0x000f, 0x0000, 0x0400, 0x0000,
			0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
	u16 execute_cmd = 0x0001;
	unsigned int i;
	u32 data;
	int ret = 0;
	struct ds5_sensor *sensor = (struct ds5_sensor *)ctrl->priv;
	u16 base = (state->is_rgb) ? DS5_RGB_CONTROL_BASE : DS5_DEPTH_CONTROL_BASE;
	u16 reg;

	if (sensor) {
		switch (sensor->mux_pad) {
		case DS5_MUX_PAD_DEPTH:
			state = container_of(ctrl->handler, struct ds5, ctrls.handler_depth);
			state->is_rgb = 0;
			state->is_depth = 1;
			state->is_y8 = 0;
			state->is_imu = 0;
		break;
		case DS5_MUX_PAD_RGB:
			state = container_of(ctrl->handler, struct ds5, ctrls.handler_rgb);
			state->is_rgb = 1;
			state->is_depth = 0;
			state->is_y8 = 0;
			state->is_imu = 0;
		break;
		case DS5_MUX_PAD_IR:
			state = container_of(ctrl->handler, struct ds5, ctrls.handler_y8);
			state->is_rgb = 0;
			state->is_depth = 0;
			state->is_y8 = 1;
			state->is_imu = 0;
		break;
		case DS5_MUX_PAD_IMU:
			state = container_of(ctrl->handler, struct ds5, ctrls.handler_imu);
			state->is_rgb = 0;
			state->is_depth = 0;
			state->is_y8 = 0;
			state->is_imu = 1;
		break;
		default:
			state->is_rgb = 0;
			state->is_depth = 0;
			state->is_y8 = 0;
			state->is_imu = 1;
		break;

		}
	}
	base = (state->is_rgb) ? DS5_RGB_CONTROL_BASE : DS5_DEPTH_CONTROL_BASE;

	dev_dbg(&state->client->dev, "%s(): %s - ctrl: %s\n",
		__func__, ds5_get_sensor_name(state), ctrl->name);

	switch (ctrl->id) {

	case V4L2_CID_ANALOGUE_GAIN:
		if (state->is_imu)
			return -EINVAL;
		ret = ds5_read(state, base | DS5_MANUAL_GAIN, ctrl->p_new.p_u16);
		break;

	case V4L2_CID_EXPOSURE_AUTO:
		if (state->is_imu)
			return -EINVAL;
		ds5_read(state, base | DS5_AUTO_EXPOSURE_MODE, &reg);
		*ctrl->p_new.p_u16 = reg;
		/* see ds5_hw_set_auto_exposure */
		if (!state->is_rgb) {
			if (reg == 1)
				*ctrl->p_new.p_u16 = V4L2_EXPOSURE_APERTURE_PRIORITY;
			else if (reg == 0)
				*ctrl->p_new.p_u16 = V4L2_EXPOSURE_MANUAL;
		}

		if (state->is_rgb && reg == 8)
			*ctrl->p_new.p_u16 = V4L2_EXPOSURE_APERTURE_PRIORITY;

		break;

	case V4L2_CID_EXPOSURE_ABSOLUTE:
		if (state->is_imu)
			return -EINVAL;
		/* see ds5_hw_set_exposure */
		ds5_read(state, base | DS5_MANUAL_EXPOSURE_MSB, &reg);
		data = ((u32)reg << 16) & 0xffff0000;
		ds5_read(state, base | DS5_MANUAL_EXPOSURE_LSB, &reg);
		data |= reg;
		*ctrl->p_new.p_u32 = data;
		break;

	case DS5_CAMERA_CID_LASER_POWER:
		if (!state->is_rgb)
			ds5_read(state, base | DS5_LASER_POWER, ctrl->p_new.p_u16);
		break;

	case DS5_CAMERA_CID_MANUAL_LASER_POWER:
		if (!state->is_rgb)
			ds5_read(state, base | DS5_MANUAL_LASER_POWER, ctrl->p_new.p_u16);
		break;

	case DS5_CAMERA_CID_LOG:
		// TODO: wrap HWMonitor command
		//       1. prepare and send command
		//       2. send command
		//       3. execute command
		//       4. wait for ccompletion
		ret = regmap_raw_write(state->regmap, 0x4900,
				log_prepare, sizeof(log_prepare));
		if (ret < 0)
			return ret;

		ret = regmap_raw_write(state->regmap, 0x490C,
				&execute_cmd, sizeof(execute_cmd));
		if (ret < 0)
			return ret;

		for (i = 0; i < DS5_MAX_LOG_POLL; i++) {
			ret = regmap_raw_read(state->regmap, 0x4904,
					&data, sizeof(data));
			dev_dbg(&state->client->dev, "%s(): log ready 0x%x\n",
				 __func__, data);
			if (ret < 0)
				return ret;
			if (!data)
				break;
			msleep_range(5);
		}

		ret = regmap_raw_read(state->regmap, 0x4908, &data, sizeof(data));
		dev_dbg(&state->client->dev, "%s(): log size 0x%x\n", __func__, data);
		if (ret < 0)
			return ret;
		if (!data)
			return 0;
		if (data > 1024)
			return -ENOBUFS;
		ret = regmap_raw_read(state->regmap, 0x4900,
				ctrl->p_new.p_u8, data);
		break;
	case DS5_CAMERA_DEPTH_CALIBRATION_TABLE_GET:
		ret = ds5_get_calibration_data(state, DEPTH_CALIBRATION_ID,
				ctrl->p_new.p_u8, 256);
		break;
	case DS5_CAMERA_COEFF_CALIBRATION_TABLE_GET:
		ret = ds5_get_calibration_data(state, COEF_CALIBRATION_ID,
				ctrl->p_new.p_u8, 512);
		break;
	case DS5_CAMERA_CID_FW_VERSION:
		ret = ds5_get_fw_info(state, &state->fw_version, &state->fw_build);
		*ctrl->p_new.p_u32 = state->fw_version << 16;
		*ctrl->p_new.p_u32 |= state->fw_build;
		break;
	case DS5_CAMERA_CID_GVD:
		ret = ds5_gvd(state, ctrl->p_new.p_u8);
		break;
	case DS5_CAMERA_CID_AE_ROI_GET:
		if (ctrl->p_new.p_u16) {
			u16 len = sizeof(struct hwm_cmd) + 12;
			u16 dataLen = 0;
			struct hwm_cmd *ae_roi_cmd;

			ae_roi_cmd = devm_kzalloc(&state->client->dev, len, GFP_KERNEL);
			if (!ae_roi_cmd) {
				dev_err(&state->client->dev,
					"%s(): Can't allocate memory for 0x%x\n",
					__func__, ctrl->id);
				ret = -ENOMEM;
				break;
			}
			memcpy(ae_roi_cmd, &get_ae_roi, sizeof(struct hwm_cmd));
			ret = ds5_send_hwmc(state, sizeof(struct hwm_cmd), ae_roi_cmd);
			if (ret) {
				devm_kfree(&state->client->dev, ae_roi_cmd);
				return ret;
			}
			ret = ds5_get_hwmc(state, ae_roi_cmd->Data, len, &dataLen);
			if (!ret && dataLen <= ctrl->dims[0])
				memcpy(ctrl->p_new.p_u16, ae_roi_cmd->Data + 4, 8);
			devm_kfree(&state->client->dev, ae_roi_cmd);
		}
		break;
	case DS5_CAMERA_CID_AE_SETPOINT_GET:
	if (ctrl->p_new.p_s32) {
		u16 len = sizeof(struct hwm_cmd) + 8;
		u16 dataLen = 0;
		struct hwm_cmd *ae_setpoint_cmd;

		ae_setpoint_cmd = devm_kzalloc(&state->client->dev,	len, GFP_KERNEL);
		if (!ae_setpoint_cmd) {
			dev_err(&state->client->dev,
					"%s(): Can't allocate memory for 0x%x\n",
					__func__, ctrl->id);
			ret = -ENOMEM;
			break;
		}
		memcpy(ae_setpoint_cmd, &get_ae_setpoint, sizeof(struct hwm_cmd));
		ret = ds5_send_hwmc(state, sizeof(struct hwm_cmd), ae_setpoint_cmd);
		if (ret) {
			devm_kfree(&state->client->dev, ae_setpoint_cmd);
			return ret;
		}
		ret = ds5_get_hwmc(state, ae_setpoint_cmd->Data, len, &dataLen);
		memcpy(ctrl->p_new.p_s32, ae_setpoint_cmd->Data + 4, 4);
		dev_dbg(&state->client->dev, "%s(): len: %d, 0x%x\n",
			__func__, dataLen, *(ctrl->p_new.p_s32));
		devm_kfree(&state->client->dev, ae_setpoint_cmd);
		}
		break;
	case DS5_CAMERA_CID_HWMC_RW:
		if (ctrl->p_new.p_u8) {
			unsigned char *data = (unsigned char *)ctrl->p_new.p_u8;
			u16 dataLen = 0;
			u16 bufLen = ctrl->dims[0];

			ret = ds5_get_hwmc(state, data,	bufLen, &dataLen);
			/* ignore empty data calls */
			if (!dataLen) {
				dev_dbg(sensor->sd.dev,
				  "%s(): DS5_CAMERA_CID_HWMC_RW empty call  Len: %d (ret: %x)\n",
				  __func__, dataLen, ret);
				ret = 0;
				break;
			}
			if (ret)
				break;
			/* This is needed for librealsense, to align there code with UVC,
			 * last word is length - 4 bytes header length */
			dataLen -= 4;
			data[bufLen - 4] = (unsigned char)(dataLen & 0x00FF);
			data[bufLen - 3] = (unsigned char)((dataLen & 0xFF00) >> 8);
			data[bufLen - 2] = 0;
			data[bufLen - 1] = 0;
		}
		break;
	case DS5_CAMERA_CID_PWM:
		if (state->is_depth)
			ds5_read(state, base | DS5_PWM_FREQUENCY, ctrl->p_new.p_u16);
		break;
	}
	return ret;
}

static const struct v4l2_ctrl_ops ds5_ctrl_ops = {
	.s_ctrl	= ds5_s_ctrl,
	.g_volatile_ctrl = ds5_g_volatile_ctrl,
};

// v4l2 ctrl
static const struct v4l2_ctrl_config ds5_ctrl_log = {
	.ops = &ds5_ctrl_ops,
	.id = DS5_CAMERA_CID_LOG,
	.name = "Logger",
	.type = V4L2_CTRL_TYPE_U8,
	.dims = {1024},
	.elem_size = sizeof(u8),
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	.step = 1,
};

static const struct v4l2_ctrl_config ds5_ctrl_laser_power = {
	.ops = &ds5_ctrl_ops,
	.id = DS5_CAMERA_CID_LASER_POWER,
	.name = "Laser power on/off",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 1,
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
};

static const struct v4l2_ctrl_config ds5_ctrl_manual_laser_power = {
	.ops = &ds5_ctrl_ops,
	.id = DS5_CAMERA_CID_MANUAL_LASER_POWER,
	.name = "Manual laser power",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 360,
	.step = 30,
	.def = 150,
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
};

static const struct v4l2_ctrl_config ds5_ctrl_fw_version = {
	.ops = &ds5_ctrl_ops,
	.id = DS5_CAMERA_CID_FW_VERSION,
	.name = "fw version",
	.type = V4L2_CTRL_TYPE_U32,
	.dims = {1},
	.elem_size = sizeof(u32),
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	.step = 1,
};

static const struct v4l2_ctrl_config ds5_ctrl_gvd = {
	.ops = &ds5_ctrl_ops,
	.id = DS5_CAMERA_CID_GVD,
	.name = "GVD",
	.type = V4L2_CTRL_TYPE_U8,
	.dims = {239},
	.elem_size = sizeof(u8),
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	.step = 1,
};

static const struct v4l2_ctrl_config ds5_ctrl_get_depth_calib = {
	.ops = &ds5_ctrl_ops,
	.id = DS5_CAMERA_DEPTH_CALIBRATION_TABLE_GET,
	.name = "get depth calib",
	.type = V4L2_CTRL_TYPE_U8,
	.dims = {256},
	.elem_size = sizeof(u8),
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	.step = 1,
};

static const struct v4l2_ctrl_config ds5_ctrl_set_depth_calib = {
	.ops = &ds5_ctrl_ops,
	.id = DS5_CAMERA_DEPTH_CALIBRATION_TABLE_SET,
	.name = "set depth calib",
	.type = V4L2_CTRL_TYPE_U8,
	.dims = {256},
	.elem_size = sizeof(u8),
	.min = 0,
	.max = 0xFFFFFFFF,
	.def = 240,
	.step = 1,
};

static const struct v4l2_ctrl_config ds5_ctrl_get_coeff_calib = {
	.ops = &ds5_ctrl_ops,
	.id = DS5_CAMERA_COEFF_CALIBRATION_TABLE_GET,
	.name = "get coeff calib",
	.type = V4L2_CTRL_TYPE_U8,
	.dims = {512},
	.elem_size = sizeof(u8),
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	.step = 1,
};

static const struct v4l2_ctrl_config ds5_ctrl_set_coeff_calib = {
	.ops = &ds5_ctrl_ops,
	.id = DS5_CAMERA_COEFF_CALIBRATION_TABLE_SET,
	.name = "set coeff calib",
	.type = V4L2_CTRL_TYPE_U8,
	.dims = {512},
	.elem_size = sizeof(u8),
	.min = 0,
	.max = 0xFFFFFFFF,
	.def = 240,
	.step = 1,
};

static const struct v4l2_ctrl_config ds5_ctrl_ae_roi_get = {
	.ops = &ds5_ctrl_ops,
	.id = DS5_CAMERA_CID_AE_ROI_GET,
	.name = "ae roi get",
	.type = V4L2_CTRL_TYPE_U8,
	.dims = {8},
	.elem_size = sizeof(u16),
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	.step = 1,
};

static const struct v4l2_ctrl_config ds5_ctrl_ae_roi_set = {
	.ops = &ds5_ctrl_ops,
	.id = DS5_CAMERA_CID_AE_ROI_SET,
	.name = "ae roi set",
	.type = V4L2_CTRL_TYPE_U8,
	.dims = {8},
	.elem_size = sizeof(u16),
	.min = 0,
	.max = 0xFFFFFFFF,
	.def = 240,
	.step = 1,
};

static const struct v4l2_ctrl_config ds5_ctrl_ae_setpoint_get = {
	.ops = &ds5_ctrl_ops,
	.id = DS5_CAMERA_CID_AE_SETPOINT_GET,
	.name = "ae setpoint get",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	.step = 1,
};

static const struct v4l2_ctrl_config ds5_ctrl_ae_setpoint_set = {
	.ops = &ds5_ctrl_ops,
	.id = DS5_CAMERA_CID_AE_SETPOINT_SET,
	.name = "ae setpoint set",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 4095,
	.step = 1,
	.def = 0,
};

static const struct v4l2_ctrl_config ds5_ctrl_erb = {
	.ops = &ds5_ctrl_ops,
	.id = DS5_CAMERA_CID_ERB,
	.name = "ERB eeprom read",
	.type = V4L2_CTRL_TYPE_U8,
	.dims = {1020},
	.elem_size = sizeof(u8),
	.min = 0,
	.max = 0xFFFFFFFF,
	.def = 240,
	.step = 1,
	.step = 1,
};

static const struct v4l2_ctrl_config ds5_ctrl_ewb = {
	.ops = &ds5_ctrl_ops,
	.id = DS5_CAMERA_CID_EWB,
	.name = "EWB eeprom write",
	.type = V4L2_CTRL_TYPE_U8,
	.dims = {1020},
	.elem_size = sizeof(u8),
	.min = 0,
	.max = 0xFFFFFFFF,
	.def = 240,
	.step = 1,
	.step = 1,
};

static const struct v4l2_ctrl_config ds5_ctrl_hwmc = {
	.ops = &ds5_ctrl_ops,
	.id = DS5_CAMERA_CID_HWMC,
	.name = "HWMC",
	.type = V4L2_CTRL_TYPE_U8,
	.dims = {DS5_HWMC_BUFFER_SIZE + 4},
	.elem_size = sizeof(u8),
	.min = 0,
	.max = 0xFFFFFFFF,
	.def = 240,
	.step = 1,
	.step = 1,
};

static const struct v4l2_ctrl_config ds5_ctrl_hwmc_rw = {
	.ops = &ds5_ctrl_ops,
	.id = DS5_CAMERA_CID_HWMC_RW,
	.name = "HWMC_RW",
	.type = V4L2_CTRL_TYPE_U8,
	.dims = {DS5_HWMC_BUFFER_SIZE},
	.elem_size = sizeof(u8),
	.min = 0,
	.max = 0xFFFFFFFF,
	.def = 240,
	.step = 1,
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
};

static const struct v4l2_ctrl_config ds5_ctrl_pwm = {
	.ops = &ds5_ctrl_ops,
	.id = DS5_CAMERA_CID_PWM,
	.name = "PWM Frequency Selector",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 1,
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
};

static const struct v4l2_ctrl_config d4xx_controls_link_freq = {
	.ops = &ds5_ctrl_ops,
	.id = V4L2_CID_LINK_FREQ,
	.name = "V4L2_CID_LINK_FREQ",
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.max = ARRAY_SIZE(link_freq_menu_items) - 1,
	.min =  0,
	.step  = 0,
	.def = 7,    // default D4XX_LINK_FREQ_300MHZ
	.qmenu_int = link_freq_menu_items,
};

static int ds5_ctrl_init(struct ds5 *state, int sid)
{
	const struct v4l2_ctrl_ops *ops = &ds5_ctrl_ops;
	struct ds5_ctrls *ctrls = &state->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;
	struct v4l2_subdev *sd = &state->mux.sd.subdev;
	int ret = -1;
	struct ds5_sensor *sensor = NULL;

	switch (sid) {
	case DEPTH_SID:
		hdl = &ctrls->handler_depth;
		sensor = &state->depth.sensor;
		break;
	case RGB_SID:
		hdl = &ctrls->handler_rgb;
		sensor = &state->rgb.sensor;
		break;
	case IR_SID:
		hdl = &ctrls->handler_y8;
		sensor = &state->ir.sensor;
		break;
	case IMU_SID:
		hdl = &ctrls->handler_imu;
		sensor = &state->imu.sensor;
		break;
	default:
		/* control for MUX */
		hdl = &ctrls->handler;
		sensor = NULL;
		break;
	}

	dev_dbg(NULL, "%s():%d sid: %d\n", __func__, __LINE__, sid);
	ret = v4l2_ctrl_handler_init(hdl, DS5_N_CONTROLS);
	if (ret < 0) {
		v4l2_err(sd, "cannot init ctrl handler (%d)\n", ret);
		return ret;
	}

	if (sid == DEPTH_SID || sid == IR_SID) {
		ctrls->laser_power = v4l2_ctrl_new_custom(hdl,
						&ds5_ctrl_laser_power,
						sensor);
		ctrls->manual_laser_power = v4l2_ctrl_new_custom(hdl,
						&ds5_ctrl_manual_laser_power,
						sensor);
	}

	/* Total gain */
	if (sid == DEPTH_SID || sid == IR_SID) {
		ctrls->gain = v4l2_ctrl_new_std(hdl, ops,
						V4L2_CID_ANALOGUE_GAIN,
						16, 248, 1, 16);
	} else if (sid == RGB_SID) {
		ctrls->gain = v4l2_ctrl_new_std(hdl, ops,
						V4L2_CID_ANALOGUE_GAIN,
						0, 128, 1, 64);
	}

	if ((ctrls->gain) && (sid >= DEPTH_SID && sid < IMU_SID)) {
		ctrls->gain->priv = sensor;
		ctrls->gain->flags =
				V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
	}
	if (sid >= DEPTH_SID && sid < IMU_SID) {

		ctrls->auto_exp = v4l2_ctrl_new_std_menu(hdl, ops,
				V4L2_CID_EXPOSURE_AUTO,
				V4L2_EXPOSURE_APERTURE_PRIORITY,
				~((1 << V4L2_EXPOSURE_MANUAL) |
						(1 << V4L2_EXPOSURE_APERTURE_PRIORITY)),
						V4L2_EXPOSURE_APERTURE_PRIORITY);

		if (ctrls->auto_exp) {
			ctrls->auto_exp->flags |=
					V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
			ctrls->auto_exp->priv = sensor;
		}
	}

	/* Exposure time: V4L2_CID_EXPOSURE_ABSOLUTE default unit: 100 us. */
	if (sid == DEPTH_SID || sid == IR_SID) {
		ctrls->exposure = v4l2_ctrl_new_std(hdl, ops,
					V4L2_CID_EXPOSURE_ABSOLUTE,
					1, MAX_DEPTH_EXP, 1, DEF_DEPTH_EXP);
	} else if (sid == RGB_SID) {
		ctrls->exposure = v4l2_ctrl_new_std(hdl, ops,
					V4L2_CID_EXPOSURE_ABSOLUTE,
					1, MAX_RGB_EXP, 1, DEF_RGB_EXP);
	}

	if ((ctrls->exposure) && (sid >= DEPTH_SID && sid < IMU_SID)) {
		ctrls->exposure->priv = sensor;
		ctrls->exposure->flags |=
				V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
		/* override default int type to u32 to match SKU & UVC */
		ctrls->exposure->type = V4L2_CTRL_TYPE_U32;
	}

	ctrls->link_freq = v4l2_ctrl_new_custom(hdl, &d4xx_controls_link_freq, sensor);
	/* MTL and RPL/ADL IPU6 CSI-DPHY do NOT share
	 *  the same default link_freq.
	 * V4L2_CID_LINK_FREQ DS5 mux must be R/W for udev ot set DPHY platform specific link_freq
	 * via systemd-udevd rules.
	*/
	if (sensor && ctrls->link_freq)
		ctrls->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	if (hdl->error) {
		v4l2_err(sd, "error creating controls (%d)\n", hdl->error);
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	// Add these after v4l2_ctrl_handler_setup so they won't be set up
	if (sid >= DEPTH_SID && sid < IMU_SID) {
		ctrls->log = v4l2_ctrl_new_custom(hdl, &ds5_ctrl_log, sensor);
		ctrls->fw_version = v4l2_ctrl_new_custom(hdl, &ds5_ctrl_fw_version, sensor);
		ctrls->gvd = v4l2_ctrl_new_custom(hdl, &ds5_ctrl_gvd, sensor);
		ctrls->get_depth_calib =
				v4l2_ctrl_new_custom(hdl, &ds5_ctrl_get_depth_calib, sensor);
		ctrls->set_depth_calib =
				v4l2_ctrl_new_custom(hdl, &ds5_ctrl_set_depth_calib, sensor);
		ctrls->get_coeff_calib =
				v4l2_ctrl_new_custom(hdl, &ds5_ctrl_get_coeff_calib, sensor);
		ctrls->set_coeff_calib =
				v4l2_ctrl_new_custom(hdl, &ds5_ctrl_set_coeff_calib, sensor);
		ctrls->ae_roi_get = v4l2_ctrl_new_custom(hdl, &ds5_ctrl_ae_roi_get, sensor);
		ctrls->ae_roi_set = v4l2_ctrl_new_custom(hdl, &ds5_ctrl_ae_roi_set, sensor);
		ctrls->ae_setpoint_get =
				v4l2_ctrl_new_custom(hdl, &ds5_ctrl_ae_setpoint_get, sensor);
		ctrls->ae_setpoint_set =
				v4l2_ctrl_new_custom(hdl, &ds5_ctrl_ae_setpoint_set, sensor);
		ctrls->erb = v4l2_ctrl_new_custom(hdl, &ds5_ctrl_erb, sensor);
		ctrls->ewb = v4l2_ctrl_new_custom(hdl, &ds5_ctrl_ewb, sensor);
		ctrls->hwmc = v4l2_ctrl_new_custom(hdl, &ds5_ctrl_hwmc, sensor);
		v4l2_ctrl_new_custom(hdl, &ds5_ctrl_hwmc_rw, sensor);
	}
	// DEPTH specific
	if (sid == DEPTH_SID)
		v4l2_ctrl_new_custom(hdl, &ds5_ctrl_pwm, sensor);
	// IMU specific
	if (sid == IMU_SID)
		ctrls->fw_version = v4l2_ctrl_new_custom(hdl, &ds5_ctrl_fw_version, sensor);

	switch (sid) {
	case DEPTH_SID:
		state->depth.sensor.sd.ctrl_handler = hdl;
		dev_info(state->depth.sensor.sd.dev,
			"%s():%d set ctrl_handler pad:%d, %p, %s\n",
			 __func__, __LINE__,
			 state->depth.sensor.mux_pad,
			 state->depth.sensor.sd.ctrl_handler,
			 state->depth.sensor.sd.name);
		break;
	case RGB_SID:
		state->rgb.sensor.sd.ctrl_handler = hdl;
		dev_info(state->rgb.sensor.sd.dev,
			"%s():%d set ctrl_handler pad:%d, %p, %s\n",
			 __func__, __LINE__,
			 state->rgb.sensor.mux_pad,
			 state->rgb.sensor.sd.ctrl_handler,
			 state->rgb.sensor.sd.name);
		break;
	case IR_SID:
		state->ir.sensor.sd.ctrl_handler = hdl;
		dev_info(state->ir.sensor.sd.dev,
			"%s():%d set ctrl_handler pad:%d, %p, %s\n",
			 __func__, __LINE__,
			 state->ir.sensor.mux_pad,
			 state->ir.sensor.sd.ctrl_handler,
			 state->ir.sensor.sd.name);
		break;
	case IMU_SID:
		state->imu.sensor.sd.ctrl_handler = hdl;
		dev_info(state->imu.sensor.sd.dev,
			"%s():%d set ctrl_handler pad:%d, %p, %s\n",
			 __func__, __LINE__,
			 state->imu.sensor.mux_pad,
			 state->imu.sensor.sd.ctrl_handler,
			 state->imu.sensor.sd.name);
		break;
	default:
		state->mux.sd.subdev.ctrl_handler = hdl;
		dev_info(state->mux.sd.subdev.dev,
			"%s():%d set ctrl_handler for MUX: %p, %s\n",
			 __func__, __LINE__,
			 state->mux.sd.subdev.ctrl_handler,
			 state->mux.sd.subdev.name);
		break;
	}

	return 0;
}

/// v4l subdev init

static int ds5_sensor_v4l_init(struct i2c_client *client, struct ds5 *ds5,
		struct ds5_sensor *sensor, const struct v4l2_subdev_ops *ops,
		const char *name)
{
	struct v4l2_subdev *sd = &sensor->sd;
	struct media_entity *entity = &sensor->sd.entity;
	struct media_pad *pad = &sensor->pad;
	dev_t *dev_num = &ds5->client->dev.devt;

	v4l2_i2c_subdev_init(sd, client, ops);
	// Set owner to NULL so we can unload the driver module
	sd->owner = NULL;
	sd->internal_ops = &ds5_sensor_internal_ops;
	sd->grp_id = *dev_num;
	v4l2_set_subdevdata(sd, ds5);

	snprintf(sd->name, sizeof(sd->name), "D4XX %s %d-%04x",
		 name, i2c_adapter_id(client->adapter), client->addr);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	pad->flags = MEDIA_PAD_FL_SOURCE;
	entity->obj_type = MEDIA_ENTITY_TYPE_V4L2_SUBDEV;
	entity->function = MEDIA_ENT_F_CAM_SENSOR;

	dev_info(&client->dev,
		 "%s: init media_entity %s, type:%x, func:%x\n",
		 __func__,
		 sd->name,
		 entity->obj_type,
		 entity->function);

	return media_entity_pads_init(entity, 1, pad);
}

static int ds5_depth_init(struct i2c_client *client, struct ds5 *ds5)
{
	struct ds5_sensor *sensor = &ds5->depth.sensor;
	struct ds5_stream_config *stream_cfg = &sensor->stream_cfg;

	sensor->mux_pad = DS5_MUX_PAD_DEPTH;

	stream_cfg->config_status_base = DS5_DEPTH_CONFIG_STATUS;
	stream_cfg->stream_status_base = DS5_DEPTH_STREAM_STATUS;
	stream_cfg->stream_id = DS5_STREAM_DEPTH;
	stream_cfg->vc_id = 0;

	stream_cfg->dt_addr = DS5_DEPTH_STREAM_DT;
	stream_cfg->md_addr = DS5_DEPTH_STREAM_MD;
	stream_cfg->override_addr = DS5_DEPTH_OVERRIDE;
	stream_cfg->fps_addr = DS5_DEPTH_FPS;
	stream_cfg->width_addr = DS5_DEPTH_RES_WIDTH;
	stream_cfg->height_addr = DS5_DEPTH_RES_HEIGHT;
	stream_cfg->md_fmt = GMSL_CSI_DT_EMBED;

	return ds5_sensor_v4l_init(client, ds5, &ds5->depth.sensor,
		       &ds5_depth_subdev_ops, "depth");
}
static int ds5_rgb_init(struct i2c_client *client, struct ds5 *ds5)
{
	struct ds5_sensor *sensor = &ds5->rgb.sensor;
	struct ds5_stream_config *stream_cfg = &sensor->stream_cfg;

	sensor->mux_pad = DS5_MUX_PAD_RGB;

	sensor->stream_cfg.config_status_base = DS5_RGB_CONFIG_STATUS;
	sensor->stream_cfg.stream_status_base = DS5_RGB_STREAM_STATUS;
	sensor->stream_cfg.stream_id = DS5_STREAM_RGB;
	sensor->stream_cfg.vc_id = 1;

	stream_cfg->dt_addr = DS5_RGB_STREAM_DT;
	stream_cfg->md_addr = DS5_RGB_STREAM_MD;
	stream_cfg->override_addr = 0;
	stream_cfg->fps_addr = DS5_RGB_FPS;
	stream_cfg->width_addr = DS5_RGB_RES_WIDTH;
	stream_cfg->height_addr = DS5_RGB_RES_HEIGHT;
	stream_cfg->md_fmt = GMSL_CSI_DT_EMBED;

	return ds5_sensor_v4l_init(client, ds5, &ds5->rgb.sensor,
		       &ds5_rgb_subdev_ops, "rgb");
}

static int ds5_ir_init(struct i2c_client *client, struct ds5 *ds5)
{
	struct ds5_sensor *sensor = &ds5->ir.sensor;
	struct ds5_stream_config *stream_cfg = &sensor->stream_cfg;

	sensor->mux_pad = DS5_MUX_PAD_IR;

	sensor->stream_cfg.config_status_base = DS5_IR_CONFIG_STATUS;
	sensor->stream_cfg.stream_status_base = DS5_IR_STREAM_STATUS;
	sensor->stream_cfg.stream_id = DS5_STREAM_IR;
	sensor->stream_cfg.vc_id = 2;

	stream_cfg->dt_addr = DS5_IR_STREAM_DT;
	stream_cfg->md_addr = DS5_IR_STREAM_MD;
	stream_cfg->override_addr = DS5_IR_OVERRIDE;
	stream_cfg->fps_addr = DS5_IR_FPS;
	stream_cfg->width_addr = DS5_IR_RES_WIDTH;
	stream_cfg->height_addr = DS5_IR_RES_HEIGHT;
	stream_cfg->md_fmt = GMSL_CSI_DT_EMBED;

	return ds5_sensor_v4l_init(client, ds5, &ds5->ir.sensor,
		       &ds5_ir_subdev_ops, "ir");
}
static int ds5_imu_init(struct i2c_client *client, struct ds5 *ds5)
{
	struct ds5_sensor *sensor = &ds5->imu.sensor;
	struct ds5_stream_config *stream_cfg = &sensor->stream_cfg;

	sensor->mux_pad = DS5_MUX_PAD_IMU;

	sensor->stream_cfg.config_status_base = DS5_IMU_CONFIG_STATUS;
	sensor->stream_cfg.stream_status_base = DS5_IMU_STREAM_STATUS;
	sensor->stream_cfg.stream_id = DS5_STREAM_IMU;
	sensor->stream_cfg.vc_id = 3;

	stream_cfg->dt_addr = DS5_IMU_STREAM_DT;
	stream_cfg->md_addr = DS5_IMU_STREAM_MD;
	stream_cfg->override_addr = 0;
	stream_cfg->fps_addr = DS5_IMU_FPS;
	stream_cfg->width_addr = DS5_IMU_RES_WIDTH;
	stream_cfg->height_addr = DS5_IMU_RES_HEIGHT;
	stream_cfg->md_fmt = 0x0;
	return ds5_sensor_v4l_init(client, ds5, &ds5->imu.sensor,
		       &ds5_imu_subdev_ops, "imu");
}
static int ds5_mux_init(struct i2c_client *client, struct ds5 *ds5)
{
	struct v4l2_subdev *sd = &ds5->mux.sd.subdev;
	struct media_entity *entity = &ds5->mux.sd.subdev.entity;
	struct media_pad *pads = ds5->mux.pads, *pad;
	unsigned int i;
	int ret;
	char suffix[5];

	v4l2_i2c_subdev_init(sd, client, &ds5_mux_subdev_ops);
	// Set owner to NULL so we can unload the driver module
	sd->owner = NULL;
	sd->internal_ops = &ds5_mux_internal_ops;
	v4l2_set_subdevdata(sd, ds5);

	snprintf(sd->name, sizeof(sd->name), "DS5 mux %d-%04x",
		 i2c_adapter_id(client->adapter), client->addr);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_STREAMS;
	entity->obj_type = MEDIA_ENTITY_TYPE_V4L2_SUBDEV;
	entity->function = MEDIA_ENT_F_VID_IF_BRIDGE;

	dev_info(&client->dev,
		 "%s():%d init media_entity %s, type:%x, func:%x\n",
		 __func__, __LINE__,
		 sd->name,
		 entity->obj_type,
		 entity->function);

	pads[0].flags = MEDIA_PAD_FL_SOURCE;
	for (i = 1, pad = pads + 1; i < ARRAY_SIZE(ds5->mux.pads); i++, pad++)
		pad->flags = MEDIA_PAD_FL_SINK;

	ret = media_entity_pads_init(entity, ARRAY_SIZE(ds5->mux.pads), pads);
	if (ret < 0)
		return ret;

	/* initialize v4l2 ctrl for mux */
	ret = ds5_ctrl_init(ds5, MUX_SID);
	if (ret < 0)
		goto e_entity;

	/* initialize v4l2 ctrl for depth */
	ret = ds5_ctrl_init(ds5, DEPTH_SID);
	if (ret < 0)
		return ret;

	/* initialize v4l2 ctrl for rgb */
	ret = ds5_ctrl_init(ds5, RGB_SID);
	if (ret < 0)
		return ret;

	/* initialize v4l2 ctrl for ir */
	ret = ds5_ctrl_init(ds5, IR_SID);
	if (ret < 0)
		return ret;

	/* initialize v4l2 ctrl for imu */
	ret = ds5_ctrl_init(ds5, IMU_SID);
	if (ret < 0)
		return ret;

	ds5_set_state_last_set(ds5);

	return 0;

e_entity:
	media_entity_cleanup(entity);

	return ret;
}
static int ds5_mux_register(struct i2c_client *client, struct ds5 *ds5)
{
	int ret = v4l2_subdev_init_finalize(&ds5->mux.sd.subdev);

	if (ret < 0)
		return ret;
	return v4l2_async_register_subdev(&ds5->mux.sd.subdev);
}
static void ds5_mux_remove(struct ds5 *ds5)
{
	v4l2_async_unregister_subdev(&ds5->mux.sd.subdev);
	v4l2_ctrl_handler_free(ds5->mux.sd.subdev.ctrl_handler);
	media_entity_cleanup(&ds5->mux.sd.subdev.entity);
}
static int ds5_sensor_init(struct i2c_client *client, struct ds5 *ds5)
{
	int ret;

	ret = ds5_sensor_config_init(client, ds5);
	if (ret < 0)
		return ret;

	ds5_sensor_format_init(&ds5->depth.sensor);
	ds5_sensor_format_init(&ds5->ir.sensor);
	ds5_sensor_format_init(&ds5->rgb.sensor);
	ds5_sensor_format_init(&ds5->imu.sensor);

	return 0;
}
static int ds5_v4l_init(struct i2c_client *client, struct ds5 *ds5)
{
	int ret;

	/* initialize configuration */
	ret = ds5_sensor_init(client, ds5);
	if (ret < 0)
		return ret;

	/* create depth v4l2 subdev */
	ret = ds5_depth_init(client, ds5);
	if (ret < 0)
		return ret;

	/* create ir v4l2 subdev */
	ret = ds5_ir_init(client, ds5);
	if (ret < 0)
		goto e_depth;

	/* create rgb v4l2 subdev */
	ret = ds5_rgb_init(client, ds5);
	if (ret < 0)
		goto e_ir;

	/* create imu v4l2 subdev */
	ret = ds5_imu_init(client, ds5);
	if (ret < 0)
		goto e_rgb;

	/* create mux v4l2 subdev */
	ret = ds5_mux_init(client, ds5);
	if (ret < 0)
		goto e_imu;

	/* register mux subdev*/
	ret = ds5_mux_register(client, ds5);
	if (ret < 0)
		goto e_mux;

	return 0;
e_mux:
	ds5_mux_remove(ds5);
e_imu:
	media_entity_cleanup(&ds5->imu.sensor.sd.entity);
e_rgb:
	media_entity_cleanup(&ds5->rgb.sensor.sd.entity);
e_ir:
	media_entity_cleanup(&ds5->ir.sensor.sd.entity);
e_depth:
	media_entity_cleanup(&ds5->depth.sensor.sd.entity);
	return ret;
}
/// DFU device handling

static int ds5_dfu_wait_for_status(struct ds5 *state)
{
	int i, ret = 0;
	u16 status;

	for (i = 0; i < DS5_START_MAX_COUNT; i++) {
		ds5_read(state, 0x5000, &status);
		if (status == 0x0001 || status == 0x0002) {
			dev_err(&state->client->dev,
					"%s(): dfu failed status(0x%4x)\n",
					__func__, status);
			ret = -EREMOTEIO;
			break;
		}
		if (!status)
			break;
		msleep_range(DS5_START_POLL_TIME);
	}

	return ret;
};

static int ds5_dfu_switch_to_dfu(struct ds5 *state)
{
	int ret;
	int i = DS5_START_MAX_COUNT;
	u16 status;

	ds5_raw_write_with_check(state, 0x4900,
			&cmd_switch_to_dfu, sizeof(cmd_switch_to_dfu));
	ds5_write_with_check(state, 0x490c, 0x01); /* execute cmd */
	/*Wait for DFU fw to boot*/
	do {
		msleep_range(DS5_START_POLL_TIME*10);
		ret = ds5_read(state, 0x5000, &status);
	} while (ret && i--);
	return ret;
};

static int ds5_dfu_wait_for_get_dfu_status(struct ds5 *state,
		enum dfu_fw_state exp_state)
{
	int ret = 0;
	u16 status, dfu_state_len = 0x0000;
	unsigned char dfu_asw_buf[DFU_WAIT_RET_LEN];
	unsigned int dfu_wr_wait_msec = 0;

	do {
		ds5_write_with_check(state, 0x5008, 0x0003); // Get Write state
		do {
			ds5_read_with_check(state, 0x5000, &status);
			if (status == 0x0001) {
				dev_err(&state->client->dev,
						"%s(): Write status error I2C_STATUS_ERROR(1)\n",
						__func__);
				return -EINVAL;
			} else
				if (status == 0x0002 && dfu_wr_wait_msec)
					msleep_range(dfu_wr_wait_msec);

		} while (status);

		ds5_read_with_check(state, 0x5004, &dfu_state_len);
		if (dfu_state_len != DFU_WAIT_RET_LEN) {
			dev_err(&state->client->dev,
					"%s(): Wrong answer len (%d)\n", __func__, dfu_state_len);
			return -EINVAL;
		}
		ds5_raw_read_with_check(state, 0x4e00, &dfu_asw_buf, DFU_WAIT_RET_LEN);
		if (dfu_asw_buf[0]) {
			dev_err(&state->client->dev,
					"%s(): Wrong dfu_status (%d)\n", __func__, dfu_asw_buf[0]);
			return -EINVAL;
		}
		dfu_wr_wait_msec = (((unsigned int)dfu_asw_buf[3]) << 16)
						| (((unsigned int)dfu_asw_buf[2]) << 8)
						| dfu_asw_buf[1];
	} while (dfu_asw_buf[4] == dfuDNBUSY && exp_state == dfuDNLOAD_IDLE);

	if (dfu_asw_buf[4] != exp_state) {
		dev_notice(&state->client->dev,
				"%s(): Wrong dfu_state (%d) while expected(%d)\n",
				__func__, dfu_asw_buf[4], exp_state);
		ret = -EINVAL;
	}
	return ret;
};
static int ds5_dfu_get_dev_info(struct ds5 *state, struct __fw_status *buf)
{
	int ret = 0;
	u16 len = 0;

	ret = ds5_write(state, 0x5008, 0x0002); //Upload DFU cmd
	if (!ret)
		ret = ds5_dfu_wait_for_status(state);
	if (!ret)
		ds5_read_with_check(state, 0x5004, &len);
	/*Sanity check*/
	if (len == sizeof(struct __fw_status)) {
		ds5_raw_read_with_check(state, 0x4e00, buf, len);
	} else {
		dev_err(&state->client->dev,
				"%s(): Wrong state size (%d)\n",
				__func__, len);
		ret = -EINVAL;
	}
	return ret;
};

static int ds5_dfu_detach(struct ds5 *state)
{
	int ret;
	struct __fw_status buf = {0};

	ds5_write_with_check(state, 0x500c, 0x00);
	ret = ds5_dfu_wait_for_get_dfu_status(state, dfuIDLE);
	if (!ret)
		ret = ds5_dfu_get_dev_info(state, &buf);
	dev_notice(&state->client->dev, "%s():DFU ver (0x%x) received\n",
			__func__, buf.DFU_version);
	dev_notice(&state->client->dev, "%s():FW last version (0x%x) received\n",
			__func__, buf.FW_lastVersion);
	dev_notice(&state->client->dev, "%s():FW status (%s)\n",
			__func__, buf.DFU_isLocked ? "locked" : "unlocked");
	return ret;
};

/* When a process reads from our device, this gets called. */
static ssize_t ds5_dfu_device_read(struct file *flip,
		char __user *buffer, size_t len, loff_t *offset)
{
	struct ds5 *state = flip->private_data;
	u16 fw_ver, fw_build;
	char msg[32];
	int ret = 0;

	if (mutex_lock_interruptible(&state->mutex))
		return -ERESTARTSYS;

	ret |= ds5_get_fw_info(state, &fw_ver, &fw_build);
	if (ret < 0)
		goto e_dfu_read_failed;
	snprintf(msg, sizeof(msg), "DFU info: \tver:  %d.%d.%d.%d\n",
			(fw_ver >> 8) & 0xff, fw_ver & 0xff,
			(fw_build >> 8) & 0xff, fw_build & 0xff);

	if (copy_to_user(buffer, msg, strlen(msg)))
		ret = -EFAULT;
	else {
		state->dfu_dev.msg_write_once = ~state->dfu_dev.msg_write_once;
		ret = strlen(msg) & state->dfu_dev.msg_write_once;
	}

e_dfu_read_failed:
	mutex_unlock(&state->mutex);
	return ret;
};

static ssize_t ds5_dfu_device_write(struct file *flip,
		const char __user *buffer, size_t len, loff_t *offset)
{
	struct ds5 *state = flip->private_data;
	int ret = 0;
	(void)offset;

	if (mutex_lock_interruptible(&state->mutex))
		return -ERESTARTSYS;
	switch (state->dfu_dev.dfu_state_flag) {

	case DS5_DFU_OPEN:
		ret = ds5_dfu_switch_to_dfu(state);
		if (ret < 0) {
			dev_err(&state->client->dev, "%s(): Switch to dfu failed (%d)\n",
					__func__, ret);
			goto dfu_write_error;
		}
	/* fallthrough - procceed to recovery */
	__attribute__((__fallthrough__));
	case DS5_DFU_RECOVERY:
		ret = ds5_dfu_detach(state);
		if (ret < 0) {
			dev_err(&state->client->dev, "%s(): Detach failed (%d)\n",
					__func__, ret);
			goto dfu_write_error;
		}
		state->dfu_dev.dfu_state_flag = DS5_DFU_IN_PROGRESS;
		state->dfu_dev.init_v4l_f = 1;
	/* fallthrough - procceed to download */
	__attribute__((__fallthrough__));
	case DS5_DFU_IN_PROGRESS: {
		unsigned int dfu_full_blocks = len / DFU_BLOCK_SIZE;
		unsigned int dfu_part_blocks = len % DFU_BLOCK_SIZE;

		while (dfu_full_blocks--) {
			if (copy_from_user(state->dfu_dev.dfu_msg, buffer, DFU_BLOCK_SIZE)) {
				ret = -EFAULT;
				goto dfu_write_error;
			}
			ret = ds5_raw_write(state, 0x4a00,
					state->dfu_dev.dfu_msg, DFU_BLOCK_SIZE);
			if (ret < 0)
				goto dfu_write_error;
			ret = ds5_dfu_wait_for_get_dfu_status(state, dfuDNLOAD_IDLE);
			if (ret < 0)
				goto dfu_write_error;
			buffer += DFU_BLOCK_SIZE;
		}
		if (copy_from_user(state->dfu_dev.dfu_msg, buffer, dfu_part_blocks)) {
				ret = -EFAULT;
				goto dfu_write_error;
		}
		if (dfu_part_blocks) {
			ret = ds5_raw_write(state, 0x4a00,
					state->dfu_dev.dfu_msg, dfu_part_blocks);
			if (!ret)
				ret = ds5_dfu_wait_for_get_dfu_status(state, dfuDNLOAD_IDLE);
			if (!ret)
				ret = ds5_write(state, 0x4a04, 0x00); /*Download complete */
			if (!ret)
				ret = ds5_dfu_wait_for_get_dfu_status(state, dfuMANIFEST);
			if (ret < 0)
				goto dfu_write_error;
			state->dfu_dev.dfu_state_flag = DS5_DFU_DONE;
		}
		dev_notice(&state->client->dev, "%s(): DFU block (%d) bytes written\n",
				__func__, (int)len);
		break;
	}
	default:
		dev_err(&state->client->dev, "%s(): Wrong state (%d)\n",
				__func__, state->dfu_dev.dfu_state_flag);
		ret = -EINVAL;
		goto dfu_write_error;

	};
	mutex_unlock(&state->mutex);
	return len;

dfu_write_error:
	state->dfu_dev.dfu_state_flag = DS5_DFU_ERROR;
	// Reset DFU device to IDLE states
	if (!ds5_write(state, 0x5010, 0x0))
		state->dfu_dev.dfu_state_flag = DS5_DFU_IDLE;
	mutex_unlock(&state->mutex);
	return ret;
};

static int ds5_dfu_device_open(struct inode *inode, struct file *file)
{
	struct ds5 *state = container_of(inode->i_cdev, struct ds5,
			dfu_dev.ds5_cdev);

	if (state->dfu_dev.device_open_count)
		return -EBUSY;
	state->dfu_dev.device_open_count++;
	if (state->dfu_dev.dfu_state_flag != DS5_DFU_RECOVERY)
		state->dfu_dev.dfu_state_flag = DS5_DFU_OPEN;
	state->dfu_dev.dfu_msg = devm_kzalloc(&state->client->dev,
			DFU_BLOCK_SIZE, GFP_KERNEL);
	if (!state->dfu_dev.dfu_msg)
		return -ENOMEM;

	file->private_data = state;

	return 0;
};

static int ds5_dfu_device_release(struct inode *inode, struct file *file)
{
	struct ds5 *ds5 = container_of(inode->i_cdev, struct ds5, dfu_dev.ds5_cdev);

	int ret = 0, retry = 10;
	u16 rval;

	ds5->dfu_dev.device_open_count--;
	if (ds5->dfu_dev.dfu_state_flag != DS5_DFU_RECOVERY)
		ds5->dfu_dev.dfu_state_flag = DS5_DFU_IDLE;
	if (ds5->dfu_dev.dfu_state_flag == DS5_DFU_DONE
			&& ds5->dfu_dev.init_v4l_f)
		ds5_v4l_init(ds5->client, ds5);
	ds5->dfu_dev.init_v4l_f = 0;
	if (ds5->dfu_dev.dfu_msg)
		devm_kfree(&ds5->client->dev, ds5->dfu_dev.dfu_msg);
	ds5->dfu_dev.dfu_msg = NULL;

	/* Verify communication */
	ret = ds5_get_state(ds5, &rval);
	if (ret < 0) {
		dev_warn(&ds5->client->dev,
			"%s(): no communication with d4xx\n", __func__);
		return ret;
	}

	return ret;
};

static const struct file_operations ds5_device_file_ops = {
	.owner = THIS_MODULE,
	.read = &ds5_dfu_device_read,
	.write = &ds5_dfu_device_write,
	.open = &ds5_dfu_device_open,
	.release = &ds5_dfu_device_release
};

struct class *g_ds5_class;
atomic_t primary_chardev = ATOMIC_INIT(0);

static int ds5_chrdev_init(struct i2c_client *client, struct ds5 *ds5)
{
	struct cdev *ds5_cdev = &ds5->dfu_dev.ds5_cdev;
	struct class **ds5_class = &ds5->dfu_dev.ds5_class;
	char suffix[5];
	struct device *chr_dev;
	char dev_name[sizeof(DS5_DRIVER_NAME_DFU) + 8];
	dev_t *dev_num = &client->dev.devt;
	int ret;

	dev_dbg(&client->dev, "%s()\n", __func__);
	/* Request the kernel for N_MINOR devices */
	ret = alloc_chrdev_region(dev_num, 0, 1, DS5_DRIVER_NAME_DFU);
	if (ret < 0)
		return ret;

	if (!atomic_read(&primary_chardev)) {
		dev_dbg(&client->dev, "%s(): <Major, Minor>: <%d, %d>\n",
				__func__, MAJOR(*dev_num), MINOR(*dev_num));
		/* Create a class : appears at /sys/class */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 2, 0)
		*ds5_class = class_create(THIS_MODULE, DS5_DRIVER_NAME_CLASS);
#else
		*ds5_class = class_create(DS5_DRIVER_NAME_CLASS);
#endif
		dev_warn(&ds5->client->dev, "%s() class_create\n", __func__);
		if (IS_ERR(*ds5_class)) {
			dev_err(&client->dev, "Could not create class device\n");
			unregister_chrdev_region(0, 1);
			ret = PTR_ERR(*ds5_class);
			return ret;
		}
		g_ds5_class = *ds5_class;
	} else
		*ds5_class = g_ds5_class;
	/* Associate the cdev with a set of file_operations */
	cdev_init(ds5_cdev, &ds5_device_file_ops);
	/* Build up the current device number. To be used further */
	*dev_num = MKDEV(MAJOR(*dev_num), MINOR(*dev_num));
	/* Create a device node for this device. */
#ifndef CONFIG_OF
	bool proceed = false;
	int count = 0;

	do {
		char temp_name[sizeof(DS5_DRIVER_NAME_DFU) + 8];
		snprintf (dev_name, sizeof(dev_name), "%s-%d-%04x",
			DS5_DRIVER_NAME_DFU, i2c_adapter_id(client->adapter), client->addr);
		struct device *exist = class_find_device_by_name(*ds5_class, temp_name);

		if (exist) {
			put_device(exist);
			count++;
			if (count > 100) {
				strcpy(suffix, "fail");
				proceed = true;
			}
		} else
			proceed = true;
		} while (!proceed);
#else
#endif
	snprintf (dev_name, sizeof(dev_name), "%s-%d-%04x",
			DS5_DRIVER_NAME_DFU, i2c_adapter_id(client->adapter), client->addr);

	chr_dev = device_create(*ds5_class, NULL, *dev_num, NULL, dev_name);
	if (IS_ERR(chr_dev)) {
		ret = PTR_ERR(chr_dev);
		dev_err(&client->dev, "Could not create device\n");
		class_destroy(*ds5_class);
		unregister_chrdev_region(0, 1);
		return ret;
	}
	cdev_add(ds5_cdev, *dev_num, 1);
	atomic_inc(&primary_chardev);
	return 0;
};

static int ds5_chrdev_remove(struct ds5 *ds5)
{
	struct class **ds5_class = &ds5->dfu_dev.ds5_class;
	dev_t *dev_num = &ds5->client->dev.devt;

	if (!ds5_class) {
		return 0;
	}
	unregister_chrdev_region(*dev_num, 1);
	device_destroy(*ds5_class, *dev_num);
	if (atomic_dec_and_test(&primary_chardev)) {
		dev_warn(&ds5->client->dev, "%s() class_destroy\n", __func__);
		class_destroy(*ds5_class);
	}
	return 0;
}

/* SYSFS attributes */

#ifdef CONFIG_SYSFS
static ssize_t ds5_fw_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *c = to_i2c_client(dev);
	struct ds5 *state = container_of(i2c_get_clientdata(c),
			struct ds5, mux.sd.subdev);

	int ret = ds5_get_fw_info(state, &state->fw_version, &state->fw_build);

	if (ret < 0) {
		dev_warn(&state->client->dev,
			"%s(): no communication with d4xx\n", __func__);
	}
	return snprintf(buf, PAGE_SIZE, "D4XX Sensor: %s, Version: %d.%d.%d.%d\n",
			ds5_get_sensor_name(state),
			(state->fw_version >> 8) & 0xff, state->fw_version & 0xff,
			(state->fw_build >> 8) & 0xff, state->fw_build & 0xff);
}

static DEVICE_ATTR_RO(ds5_fw_ver);

/* Derive 'device_attribute' structure for a read register's attribute */
struct dev_ds5_reg_attribute {
	struct device_attribute attr;
	u16 reg;	// register
	u8 valid;	// validity of above data
};

/** Read DS5 register.
 * ds5_read_reg_show will actually read register from ds5 while
 * ds5_read_reg_store will store register to read
 * Example:
 * echo -n "0xc03c" >ds5_read_reg
 * Read register result:
 * cat ds5_read_reg
 * Expected:
 * reg:0xc93c, result:0x11
 */
static ssize_t ds5_read_reg_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u16 rbuf;
	int n;
	struct i2c_client *c = to_i2c_client(dev);
	struct ds5 *state = container_of(i2c_get_clientdata(c),
			struct ds5, mux.sd.subdev);
	struct dev_ds5_reg_attribute *ds5_rw_attr = container_of(attr,
			struct dev_ds5_reg_attribute, attr);
	if (ds5_rw_attr->valid != 1)
		return -EINVAL;
	ds5_read(state, ds5_rw_attr->reg, &rbuf);

	n = snprintf(buf, PAGE_SIZE, "register:0x%4x, value:0x%02x\n",
			ds5_rw_attr->reg, rbuf);

	return n;
}

/** Read DS5 register - Store reg to attr struct pointer
 * ds5_read_reg_show will actually read register from ds5 while
 * ds5_read_reg_store will store module, offset and length
 */
static ssize_t ds5_read_reg_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct dev_ds5_reg_attribute *ds5_rw_attr = container_of(attr,
			struct dev_ds5_reg_attribute, attr);
	int rc = -1;
	u32 reg;

	ds5_rw_attr->valid = 0;
	/* Decode input */
	rc = sscanf(buf, "0x%04x", &reg);
	if (rc != 1)
		return -EINVAL;
	ds5_rw_attr->reg = reg;
	ds5_rw_attr->valid = 1;
	return count;
}

#define DS5_RW_REG_ATTR(_name) \
		struct dev_ds5_reg_attribute dev_attr_##_name = { \
			__ATTR(_name, 0644, \
			ds5_read_reg_show, ds5_read_reg_store), \
			0, 0 }

static DS5_RW_REG_ATTR(ds5_read_reg);

static ssize_t ds5_write_reg_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *c = to_i2c_client(dev);
	struct ds5 *state = container_of(i2c_get_clientdata(c),
			struct ds5, mux.sd.subdev);

	int rc = -1;
	u32 reg, w_val = 0;
	u16 val = -1;
	/* Decode input */
	rc = sscanf(buf, "0x%04x 0x%04x", &reg, &w_val);
	if (rc != 2)
		return -EINVAL;
	val = w_val & 0xffff;
	mutex_lock(&state->mutex);
	ds5_write(state, reg, val);
	mutex_unlock(&state->mutex);
	return count;
}

static DEVICE_ATTR_WO(ds5_write_reg);

static struct attribute *ds5_attributes[] = {
		&dev_attr_ds5_fw_ver.attr,
		&dev_attr_ds5_read_reg.attr.attr,
		&dev_attr_ds5_write_reg.attr,
		NULL
};

static const struct attribute_group ds5_attr_group = {
	.attrs = ds5_attributes,
};
#endif
static int ds5_reset(struct gpio_desc *reset_gpio)
{
	if (!IS_ERR_OR_NULL(reset_gpio)) {
		gpiod_set_value_cansleep(reset_gpio, 0);
		usleep_range(500, 1000);
		gpiod_set_value_cansleep(reset_gpio, 1);
		/*Needs to sleep for quite a while before register writes*/
		usleep_range(200 * 1000, 200 * 1000 + 500);

		return 0;
	}

	return -EINVAL;
}

// v4l2 ops

static void ds5_update_pad_format(const struct ds5_resolution *resolutions,
				     struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = resolutions->width;
	fmt->height = resolutions->height;
	fmt->code = resolutions->code;
	//fmt->field = V4L2_FIELD_NONE;
}

static int __maybe_unused ds5_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ds5 *ds5 = to_ds5(sd);

	mutex_lock(&ds5->mutex);
	if (ds5->streaming)
		ds5_stop_streaming(ds5);

	mutex_unlock(&ds5->mutex);

	return 0;
}

static int __maybe_unused ds5_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ds5 *ds5 = to_ds5(sd);
	const struct ds5_reg_list *reg_list;
	int ret;
	u16 rval;

	if (ds5->reset_gpio != NULL)
		ds5_reset(ds5->reset_gpio);

	ret = ds5_get_state(ds5, &rval);
	if (ret == 0) {
		reg_list = &ds5->cur_mode->reg_list;
		ret = ds5_write_reg_list(ds5, reg_list);
		if (ret) {
			dev_err(&client->dev, "resume: failed to apply cur mode");
			return ret;
		}
	} else {
		dev_err(&client->dev, "ds5 resume failed");
		return ret;
	}

	mutex_lock(&ds5->mutex);
	if (ds5->streaming) {
		ret = ds5_start_streaming(ds5);
		if (ret) {
			ds5->streaming = false;
			ds5_stop_streaming(ds5);
			mutex_unlock(&ds5->mutex);
			return ret;
		}
	}

	mutex_unlock(&ds5->mutex);

	return 0;
}

static void ds5_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ds5 *ds5 = container_of(sd, struct ds5, mux.sd.subdev);

	dev_info(&client->dev, "D4XX remove %s\n", ds5_get_sensor_name(ds5));
	if (ds5->dfu_dev.dfu_state_flag != DS5_DFU_RECOVERY) {
#ifdef CONFIG_SYSFS
		sysfs_remove_group(&client->dev.kobj, &ds5_attr_group);
#endif
		ds5_mux_remove(ds5);
		if (ds5->is_depth) {
			ds5_chrdev_remove(ds5);
		}
	}
}

static int ds5_probe(struct i2c_client *client)
{
	struct v4l2_subdev *sd;
	struct ds5 *ds5;
	const struct ds5_reg_list *reg_list;
	int ret;
	u16 rval;

	ds5 = devm_kzalloc(&client->dev, sizeof(*ds5), GFP_KERNEL);
	if (!ds5)
		return -ENOMEM;

	ds5->client = client;
	ds5->platform_data = client->dev.platform_data;
	if (ds5->platform_data == NULL) {
		dev_warn(&client->dev, "no platform data provided\n");
	} else {
		dev_info(&client->dev, "platform data provided\n");
	}

	ds5->regmap = devm_regmap_init_i2c(client, &ds5_regmap_config);
	if (IS_ERR(ds5->regmap)) {
		ret = PTR_ERR(ds5->regmap);
		dev_err(&client->dev, "failed to init regmap: %d\n", ret);
		return ret;
	}

	ds5->variant = ds5_variants;

	ds5->reset_gpio = devm_gpiod_get_optional(&client->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ds5->reset_gpio))
		return -EPROBE_DEFER;
	else if (ds5->reset_gpio == NULL)
		dev_warn(&client->dev, "no reset gpio provided\n");
	else {
		dev_info(&client->dev, "found reset gpio\n");

		ds5_reset(ds5->reset_gpio);
	}
	/* ds5 communication */
	ret = ds5_get_state(ds5, &rval);
	if (ret) {
		dev_err(&client->dev, "failed to find sensor: %d", ret);
		return ret;
	}

	ds5->is_depth = 1;
	ds5->is_rgb = 0;
	ds5->is_imu = 0;
	ds5->is_y8 = 0;

	/* create DFU chardev */
	if (ds5->is_depth) {
		ret = ds5_chrdev_init(client, ds5);
		if (ret) {
			dev_err(&client->dev, "failed to init char dev: %d", ret);
			return ret;
		}
	}

	/* ds5 communication */
	ret = ds5_get_state(ds5, &rval);
	if (ret) {
		dev_err(&client->dev, "failed to find sensor: %d", ret);
		goto probe_error_chrdev_remove;
	}

	if (rval == DFU_STATE_RECOVERY) {
		dev_info(&client->dev, "D4XX recovery state\n", __func__);
		ds5->dfu_dev.dfu_state_flag = DS5_DFU_RECOVERY;
		return 0;
	}

	/* get ds5 firmware info */
	ret = ds5_get_fw_info(ds5, &ds5->fw_version, &ds5->fw_build);
	if (ret) {
		dev_err(&client->dev, "failed to get fw info: %d", ret);
		goto probe_error_chrdev_remove;
	}

	/* register v4l2 subdev */
	ret = ds5_v4l_init(client, ds5);
	if (ret) {
		dev_err(&client->dev, "failed to init v4l2: %d", ret);
		goto probe_error_media_entity_cleanup;
	}

	/* configure ds5 hardware */
	ret = ds5_hw_init(client, ds5);
	if (ret) {
		dev_err(&client->dev, "failed to init hw: %d", ret);
		goto probe_error_media_entity_cleanup;
	}

	mutex_init(&ds5->mutex);

#ifdef CONFIG_SYSFS
	/* Custom sysfs attributes */
	/* create the sysfs file group */
	int err = sysfs_create_group(&ds5->client->dev.kobj, &ds5_attr_group);
#endif

	/*
	 * Device is already turned on by i2c-core with ACPI domain PM.
	 * Enable runtime PM and turn off the device.
	 */
	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	return 0;

probe_error_media_entity_cleanup:
	//media_entity_cleanup(&ds5->sd.entity);
	pm_runtime_disable(&client->dev);
	mutex_destroy(&ds5->mutex);

probe_error_chrdev_remove:
	ds5_chrdev_remove(ds5);

	return ret;
}

static const struct dev_pm_ops ds5_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ds5_suspend, ds5_resume)
};

static const struct acpi_device_id ds5_acpi_ids[] = {
	{ "INTC10CD" },
	{}
};

MODULE_DEVICE_TABLE(acpi, ds5_acpi_ids);

static const struct i2c_device_id ds5_id_table[] = {
	{ "d4xx", 0 },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(i2c, ds5_id_table);

static struct i2c_driver ds5_i2c_driver = {
	.driver = {
		.name = "d4xx",
		.acpi_match_table = ACPI_PTR(ds5_acpi_ids),
		.pm = &ds5_pm_ops,
	},
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 6, 0)
	.probe_new = ds5_probe,
#else
	.probe = ds5_probe,
#endif
	.remove = ds5_remove,
	.id_table = ds5_id_table,
};

module_i2c_driver(ds5_i2c_driver);

MODULE_AUTHOR("Hao Yao <hao.yao@intel.com>");
MODULE_AUTHOR("Khai Wen, Ng <khai.wen.ng@intel.com>");
MODULE_DESCRIPTION("ds5 sensor driver");
MODULE_LICENSE("GPL v2");
