// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2020 Intel Corporation

#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/version.h>

#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/ti960.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-core.h>

#include "ti960-reg.h"
#include "ti953.h"

int ti953_reg_write(struct v4l2_subdev *sd, unsigned short rx_port,
	unsigned short ser_alias, unsigned char reg, unsigned char val)
{
	int ret;
	int retry, timeout = 10;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned short addr_backup;

	dev_dbg(sd->dev, "%s port %d, ser_alias %x, reg %x, val %x",
		__func__, rx_port, ser_alias, reg, val);
	addr_backup = client->addr;
	client->addr = ser_alias;
	for (retry = 0; retry < timeout; retry++) {
		ret = i2c_smbus_write_byte_data(client, reg, val);
		if (ret < 0)
			usleep_range(5000, 6000);
		else
			break;
	}

	client->addr = addr_backup;
	if (retry >= timeout) {
		dev_err(sd->dev,
			"%s:write reg failed: port=%2x, addr=%2x, reg=%2x\n",
			__func__, rx_port, ser_alias, reg);
		return -EREMOTEIO;
	}

	return 0;
}

int ti953_reg_read(struct v4l2_subdev *sd, unsigned short rx_port,
	unsigned short ser_alias, unsigned char reg, unsigned char *val)
{
	int ret, retry, timeout = 10;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned short addr_backup;

	addr_backup = client->addr;
	client->addr = ser_alias;
	for (retry = 0; retry < timeout; retry++) {
		ret = i2c_smbus_read_byte_data(client, reg);
		if (ret < 0)
			usleep_range(5000, 6000);
		else {
			*val = ret & 0xFF;
			break;
		}
	}

	client->addr = addr_backup;
	if (retry >= timeout) {
		dev_err(sd->dev,
			"%s:read reg failed: port=%2x, addr=%2x, reg=%2x\n",
			__func__, rx_port, ser_alias, reg);
		return -EREMOTEIO;
	}

	return 0;
}

bool ti953_detect(struct v4l2_subdev *sd, unsigned short rx_port, unsigned short ser_alias)
{
	bool ret = false;
	int i;
	int rval;
	unsigned char val;

	for (i = 0; i < ARRAY_SIZE(ti953_FPD3_RX_ID); i++) {
		rval = ti953_reg_read(sd, rx_port, ser_alias,
			ti953_FPD3_RX_ID[i].reg, &val);
		if (rval) {
			dev_err(sd->dev, "port %d, ti953 write timeout %d\n", rx_port, rval);
			break;
		}
		if (val != ti953_FPD3_RX_ID[i].val_expected)
			break;
	}

	if (i == ARRAY_SIZE(ti953_FPD3_RX_ID))
		ret = true;
	else
		dev_err(sd->dev, "TI953 Probe Failed");

	return ret;
}

int ti953_init(struct v4l2_subdev *sd, unsigned short rx_port, unsigned short ser_alias)
{
	int i, rval;

	for (i = 0; i < ARRAY_SIZE(ti953_init_settings); i++) {
		rval = ti953_reg_write(sd, rx_port, ser_alias,
			ti953_init_settings[i].reg,
			ti953_init_settings[i].val);
		if (rval) {
			dev_err(sd->dev, "port %d, ti953 write timeout %d\n", 0, rval);
			break;
		}
	}

	ti953_init_clk(sd, rx_port, ser_alias);

	return 0;
}

int ti953_init_clk(struct v4l2_subdev *sd, unsigned short rx_port, unsigned short ser_alias)
{
	int i, rval;

	for (i = 0; i < ARRAY_SIZE(ti953_init_settings_clk); i++) {
		rval = ti953_reg_write(sd, rx_port, ser_alias,
			ti953_init_settings_clk[i].reg,
			ti953_init_settings_clk[i].val);
		if (rval) {
			dev_err(sd->dev, "port %d, ti953 write timeout %d\n", 0, rval);
			break;
		}
	}

	return 0;
}

int32_t ti953_bus_speed(struct v4l2_subdev *sd, uint16_t rx_port, uint16_t ser_alias, uint8_t i2c_speed)
{
	struct ti953_register_write scl_high_reg;
	struct ti953_register_write scl_low_reg;
	int32_t ret = 0;

	scl_high_reg.reg = TI953_SCL_HIGH_TIME;
	scl_low_reg.reg = TI953_SCL_LOW_TIME;
	switch (i2c_speed) {
	case TI953_I2C_SPEED_STANDARD:
		scl_high_reg.val = TI953_I2C_SCL_HIGH_TIME_STANDARD;
		scl_low_reg.val = TI953_I2C_SCL_LOW_TIME_STANDARD;
		break;
	case TI953_I2C_SPEED_FAST:
		scl_high_reg.val = TI953_I2C_SCL_HIGH_TIME_FAST;
		scl_low_reg.val = TI953_I2C_SCL_LOW_TIME_FAST;
		break;
	case TI953_I2C_SPEED_FAST_PLUS:
		scl_high_reg.val = TI953_I2C_SCL_HIGH_TIME_FAST_PLUS;
		scl_low_reg.val = TI953_I2C_SCL_LOW_TIME_FAST_PLUS;
		break;
	case TI953_I2C_SPEED_HIGH:
	default:
		dev_err(sd->dev, "port %u, ti953 unsupported I2C speed mode %u",
			rx_port, i2c_speed);
		scl_high_reg.val = TI953_I2C_SCL_HIGH_TIME_STANDARD;
		scl_low_reg.val = TI953_I2C_SCL_LOW_TIME_STANDARD;
		ret = -EINVAL;
		break;
	}
	if (ret != 0)
		return ret;
	ret = ti953_reg_write(sd, rx_port, ser_alias,
			      scl_high_reg.reg, scl_high_reg.val);
	if (ret != 0) {
		dev_err(sd->dev, "port %u, ti953 write SCL_HIGH_TIME failed %d",
			rx_port, ret);
		return ret;
	}
	ret = ti953_reg_write(sd, rx_port, ser_alias,
			      scl_low_reg.reg, scl_low_reg.val);
	if (ret != 0) {
		dev_err(sd->dev, "port %u, ti953 write SCL_LOW_TIME failed %d",
			rx_port, ret);
		return ret;
	}

	return 0;
}
