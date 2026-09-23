/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2014 - 2022 Intel Corporation */

#ifndef __DS5_H
#define __DS5_H

#include <linux/types.h>

#define DS5_NAME "ds5"

#define DS5_I2C_ADDRESS 0x10

struct ds5_platform_data {
	unsigned int port;
	unsigned int lanes;
	uint32_t i2c_slave_address;
	int irq_pin;
	unsigned int irq_pin_flags;
	char irq_pin_name[16];
	char suffix[5];
	int gpios[4];
};

enum dfu_fw_state {
	appIDLE                = 0x0000,
	appDETACH              = 0x0001,
	dfuIDLE                = 0x0002,
	dfuDNLOAD_SYNC         = 0x0003,
	dfuDNBUSY              = 0x0004,
	dfuDNLOAD_IDLE         = 0x0005,
	dfuMANIFEST_SYNC       = 0x0006,
	dfuMANIFEST            = 0x0007,
	dfuMANIFEST_WAIT_RESET = 0x0008,
	dfuUPLOAD_IDLE         = 0x0009,
	dfuERROR               = 0x000a
};

enum dfu_state {
	DS5_DFU_IDLE = 0,
	DS5_DFU_RECOVERY,
	DS5_DFU_OPEN,
	DS5_DFU_IN_PROGRESS,
	DS5_DFU_DONE,
	DS5_DFU_ERROR
} dfu_state_t;

struct hwm_cmd {
	u16 header;
	u16 magic_word;
	u32 opcode;
	u32 param1;
	u32 param2;
	u32 param3;
	u32 param4;
	unsigned char Data[0];
};

static const struct hwm_cmd cmd_switch_to_dfu = {
	.header = 0x14,
	.magic_word = 0xCDAB,
	.opcode = 0x1e,
	.param1 = 0x01,
};

enum table_id {
	COEF_CALIBRATION_ID = 0x19,
	DEPTH_CALIBRATION_ID = 0x1f,
	RGB_CALIBRATION_ID = 0x20,
	IMU_CALIBRATION_ID = 0x22
} table_id_t;

static const struct hwm_cmd get_calib_data = {
	.header = 0x14,
	.magic_word = 0xCDAB,
	.opcode = 0x15,
	.param1 = 0x00,	//table_id
};

static const struct hwm_cmd set_calib_data = {
	.header = 0x0114,
	.magic_word = 0xCDAB,
	.opcode = 0x62,
	.param1 = 0x00,	//table_id
	.param2 = 0x02,	//region
};

static const struct hwm_cmd gvd = {
	.header = 0x14,
	.magic_word = 0xCDAB,
	.opcode = 0x10,
};

static const struct hwm_cmd set_ae_roi = {
	.header = 0x14,
	.magic_word = 0xCDAB,
	.opcode = 0x44,
};

static const struct hwm_cmd get_ae_roi = {
	.header = 0x014,
	.magic_word = 0xCDAB,
	.opcode = 0x45,
};

static const struct hwm_cmd set_ae_setpoint = {
	.header = 0x18,
	.magic_word = 0xCDAB,
	.opcode = 0x2B,
	.param1 = 0xa, // AE control
};

static const struct hwm_cmd get_ae_setpoint = {
	.header = 0x014,
	.magic_word = 0xCDAB,
	.opcode = 0x2C,
	.param1 = 0xa, // AE control
	.param2 = 0, // get current
};

static const struct hwm_cmd erb = {
	.header = 0x14,
	.magic_word = 0xCDAB,
	.opcode = 0x17,
};

static const struct hwm_cmd ewb = {
	.header = 0x14,
	.magic_word = 0xCDAB,
	.opcode = 0x18,
};
struct __fw_status {
	uint32_t	spare1;
	uint32_t	FW_lastVersion;
	uint32_t	FW_highestVersion;
	uint16_t	FW_DownloadStatus;
	uint16_t	DFU_isLocked;
	uint16_t	DFU_version;
	uint8_t		ivcamSerialNum[8];
	uint8_t		spare2[42];
};

struct ds5_dfu_dev {
	struct cdev ds5_cdev;
	struct class *ds5_class;
	int device_open_count;
	enum dfu_state dfu_state_flag;
	unsigned char *dfu_msg;
	u16 msg_write_once;
	unsigned char init_v4l_f;
	u32 bus_clk_rate;
};

#endif /* __DS5_H  */
