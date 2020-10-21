// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 * Intel tsens I2C thermal Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 */

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include "intel_tsens_thermal.h"

#define TSENS_BYTE_INDEX_SHIFT	0x6
#define TSENS_BYTE_INDEX_MASK	0x3
#define TSENS_SENSOR_TYPE_MASK	0x3F

struct intel_tsens_i2c {
	int sensor_type;
	u16 buffer_idx;
	bool read_only;
	u8 idx_write_cnt;
	struct intel_tsens_i2c_plat_data *plat_data;
};

static int intel_i2c_tsens_slave_cb(struct i2c_client *client,
				    enum i2c_slave_event event, u8 *val)
{
	struct intel_tsens_i2c *tsens_i2c = i2c_get_clientdata(client);
	struct intel_tsens_i2c_plat_data *plat_data = tsens_i2c->plat_data;
	int ret = 0;

	switch (event) {
	case I2C_SLAVE_WRITE_RECEIVED:
		tsens_i2c->sensor_type = *val;
		break;

	case I2C_SLAVE_READ_PROCESSED:
	case I2C_SLAVE_READ_REQUESTED:
		if (plat_data->get_temp) {
			int temp;
			int sensor_type = tsens_i2c->sensor_type &
						TSENS_SENSOR_TYPE_MASK;

			if (!plat_data->get_temp(sensor_type, &temp,
						 plat_data->pdata)) {
				u8 offset = (tsens_i2c->sensor_type >>
						TSENS_BYTE_INDEX_SHIFT) &
						TSENS_BYTE_INDEX_MASK;
				u8 *ptr_temp = (u8 *)&temp;

				*val = ptr_temp[offset];
				tsens_i2c->buffer_idx++;
				ret = 0;
			} else {
				ret = -EINVAL;
			}
		} else {
			ret = -EINVAL;
		}
		break;

	case I2C_SLAVE_STOP:
	case I2C_SLAVE_WRITE_REQUESTED:
		tsens_i2c->idx_write_cnt = 0;
		tsens_i2c->buffer_idx = 0;
		break;

	default:
		break;
	}
	return ret;
}

static int intel_i2c_tsens_slave_probe(struct i2c_client *client,
				       const struct i2c_device_id *id)
{	struct intel_tsens_i2c *priv;
	int ret;

	if (!id->driver_data) {
		dev_err(&client->dev, "No platform data");
		return -EINVAL;
	}
	priv = devm_kzalloc(&client->dev,
			    sizeof(struct intel_tsens_i2c),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->plat_data = (struct intel_tsens_i2c_plat_data *)id->driver_data;
	i2c_set_clientdata(client, priv);
	ret = i2c_slave_register(client, intel_i2c_tsens_slave_cb);
	if (ret)
		dev_err(&client->dev, "i2c slave register failed\n");

	return ret;
};

static struct i2c_device_id intel_i2c_tsens_slave_id[] = {
	{ "intel_tsens", (kernel_ulong_t)&i2c_plat_data},
	{}
};
MODULE_DEVICE_TABLE(i2c, intel_i2c_tsens_slave_id);

static struct i2c_driver intel_i2c_tsens_slave_driver = {
	.driver = {
		.name = "intel_tsens",
	},
	.probe = intel_i2c_tsens_slave_probe,
	.remove = i2c_slave_unregister,
	.id_table = intel_i2c_tsens_slave_id,
};

module_i2c_driver(intel_i2c_tsens_slave_driver);

MODULE_AUTHOR("Udhayakumar C <udhayakumar.c@intel.com>");
MODULE_DESCRIPTION("tsens i2c slave driver");
MODULE_LICENSE("GPL");
