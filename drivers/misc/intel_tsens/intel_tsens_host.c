// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 * Intel tsens I2C thermal Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 */

#include <asm/page.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/hddl_device.h>
#include <linux/i2c.h>
#include <linux/ioctl.h>
#include <linux/intel_tsens_host.h>
#include <linux/kernel.h>
#include <linux/kmod.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/printk.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/sched/mm.h>
#include <linux/sched/task.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <uapi/linux/stat.h>

#define TSENS_BINDING_NAME	"intel_tsens"
#define TSENS_BYTE_INDEX_SHIFT  0x6
#define TSENS_READ_BYTE0	(0x0 << TSENS_BYTE_INDEX_SHIFT)
#define TSENS_READ_BYTE1	(0x1 << TSENS_BYTE_INDEX_SHIFT)
#define TSENS_READ_BYTE2	(0x2 << TSENS_BYTE_INDEX_SHIFT)
#define TSENS_READ_BYTE3	(0x3 << TSENS_BYTE_INDEX_SHIFT)

static int tsens_i2c_smbus_read_byte_data(struct i2c_client *i2c, u8 command,
					  u8 *i2c_val)
{
	union i2c_smbus_data data;
	int status;

	status = i2c_smbus_xfer(i2c->adapter, i2c->addr, i2c->flags,
				I2C_SMBUS_READ, command,
				I2C_SMBUS_BYTE_DATA, &data);
	*i2c_val = data.byte;
	return status;
}

/**
 * intel_tsens_get_temp - get updated temperatue
 * @zone: Thermal zone device
 * @temp: updated temperature value.
 *
 * Temperature value read from sensors ranging from -40000 (-40 degree Celsius)
 * to 126000 (126 degree Celsius). if there is a failure while reading update
 * temperature, -255 would be returned as temperature to indicate failure.
 */
static int intel_tsens_get_temp(struct thermal_zone_device *zone,
				int *temp)
{
	struct intel_tsens_host *tsens =
		(struct intel_tsens_host *)zone->devdata;
	struct mutex *sync_unregister_mutex;
	struct i2c_client *i2c_c;
	int status, sensor_type;
	u8 i2c_val;
	s32 val;

	if (strstr(zone->type, "smb")) {
		sync_unregister_mutex = &tsens->sync_smb_unregister;
		mutex_lock(sync_unregister_mutex);
		i2c_c = tsens->i2c_smbus;
	} else {
		sync_unregister_mutex = &tsens->sync_xlk_unregister;
		mutex_lock(sync_unregister_mutex);
		i2c_c = tsens->i2c_xlk;
	}
	*temp = -255;
	if (!i2c_c) {
		mutex_unlock(sync_unregister_mutex);
		return -EINVAL;
	}
	sensor_type = tsens->t_data->sensor_type | TSENS_READ_BYTE0;
	status = tsens_i2c_smbus_read_byte_data(i2c_c,
						sensor_type,
						&i2c_val);
	if (status < 0)
		goto unlock_and_exit;
	val = i2c_val;
	sensor_type = tsens->t_data->sensor_type | TSENS_READ_BYTE1;
	status = tsens_i2c_smbus_read_byte_data(i2c_c,
						sensor_type,
						&i2c_val);
	if (status < 0)
		goto unlock_and_exit;
	val |= (i2c_val << 8);
	sensor_type = tsens->t_data->sensor_type | TSENS_READ_BYTE2;
	status = tsens_i2c_smbus_read_byte_data(i2c_c,
						sensor_type,
						&i2c_val);
	if (status < 0)
		goto unlock_and_exit;
	val |= (i2c_val << 16);
	sensor_type = tsens->t_data->sensor_type | TSENS_READ_BYTE3;
	status = tsens_i2c_smbus_read_byte_data(i2c_c,
						sensor_type,
						&i2c_val);
	if (status < 0)
		goto unlock_and_exit;
	val |= (i2c_val << 24);
	*temp = val;
	mutex_unlock(sync_unregister_mutex);
	return 0;

unlock_and_exit:
	mutex_unlock(sync_unregister_mutex);
	return status;
}

static int intel_tsens_thermal_get_trip_type(struct thermal_zone_device *zone,
					     int trip,
					     enum thermal_trip_type *type)
{
	struct intel_tsens_host *tsens =
		(struct intel_tsens_host *)zone->devdata;

	if (tsens->trip_info[trip]->trip_type != THERMAL_TRIP_CRITICAL)
		*type = tsens->trip_info[trip]->trip_type;
	return 0;
}

static int intel_tsens_thermal_get_trip_temp(struct thermal_zone_device *zone,
					     int trip, int *temp)
{
	struct intel_tsens_host *tsens =
		(struct intel_tsens_host *)zone->devdata;

	if (tsens->trip_info[trip]->trip_type != THERMAL_TRIP_CRITICAL)
		*temp = tsens->trip_info[trip]->temp;
	return 0;
}

static int intel_tsens_thermal_notify(struct thermal_zone_device *tz,
				      int trip, enum thermal_trip_type type)
{
	int ret = 0;

	switch (type) {
	case THERMAL_TRIP_ACTIVE:
		dev_warn(&tz->device,
			 "zone %s reached to active temperature %d\n",
			 tz->type, tz->temperature);
		ret = 1;
		break;
	case THERMAL_TRIP_CRITICAL:
		dev_warn(&tz->device,
			 "zone %s reached to critical temperature %d\n",
			 tz->type, tz->temperature);
		ret = 1;
		break;
	default:
		break;
	}
	return ret;
}

static int intel_tsens_bind(struct thermal_zone_device *tz,
			    struct thermal_cooling_device *cdev)
{
	int ret;

	/*
	 * Check here thermal device zone name and cdev name to match,
	 * then call the bind device
	 */
	if (strncmp(TSENS_BINDING_NAME, cdev->type,
		    strlen(TSENS_BINDING_NAME)) == 0) {
		ret = thermal_zone_bind_cooling_device
				(tz,
				THERMAL_TRIP_ACTIVE,
				cdev,
				THERMAL_NO_LIMIT,
				THERMAL_NO_LIMIT,
				THERMAL_WEIGHT_DEFAULT);
		if (ret) {
			dev_err(&tz->device,
				"binding zone %s with cdev %s failed:%d\n",
				tz->type, cdev->type, ret);
			return ret;
		}
	}
	return 0;
}

static int intel_tsens_unbind(struct thermal_zone_device *tz,
			      struct thermal_cooling_device *cdev)
{
	int ret;

	if (strncmp(TSENS_BINDING_NAME, cdev->type,
		    strlen(TSENS_BINDING_NAME)) == 0) {
		ret = thermal_zone_unbind_cooling_device(tz, 0, cdev);
		if (ret) {
			dev_err(&tz->device,
				"unbinding zone %s with cdev %s failed:%d\n",
				tz->type, cdev->type, ret);
			return ret;
		}
	}
	return 0;
}

static struct thermal_zone_device_ops tsens_thermal_ops = {
	.bind = intel_tsens_bind,
	.unbind = intel_tsens_unbind,
	.get_temp = intel_tsens_get_temp,
	.get_trip_type	= intel_tsens_thermal_get_trip_type,
	.get_trip_temp	= intel_tsens_thermal_get_trip_temp,
	.notify		= intel_tsens_thermal_notify,
};

static int intel_tsens_add_tz(struct intel_tsens_host *tsens,
			      struct thermal_zone_device **tz,
			      const char *name,
			      struct device *dev,
			      int i)
{
	int ret;

	*tz =  thermal_zone_device_register(name,
					    tsens->t_data->n_trips,
					    0, tsens,
					    &tsens_thermal_ops,
					    NULL,
					    tsens->t_data->passive_delay,
					    tsens->t_data->polling_delay);
	if (IS_ERR(*tz)) {
		ret = PTR_ERR(*tz);
		dev_err(dev,
			"failed to register thermal zone device %s\n",
			tsens->t_data->name);
		return ret;
	}
	return 0;
}

static void intel_tsens_remove_tz(struct intel_hddl_clients *d)
{
	int i;

	for (i = 0; i < d->nsens; i++) {
		struct intel_tsens_host *tsens = d->tsens[i];

		if (tsens->tz_smbus) {
			mutex_lock(&tsens->sync_smb_unregister);
			tsens->i2c_smbus = NULL;
			thermal_zone_device_unregister(tsens->tz_smbus);
			mutex_unlock(&tsens->sync_smb_unregister);
		}
		if (tsens->tz_xlk) {
			mutex_lock(&tsens->sync_xlk_unregister);
			tsens->tz_xlk = NULL;
			thermal_zone_device_unregister(tsens->tz_xlk);
			mutex_unlock(&tsens->sync_xlk_unregister);
		}
	}
}

static int intel_tsens_tj_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct intel_hddl_clients *d = client->dev.platform_data;
	u32 device_id = tsens_get_device_id(d);
	char *i2c_str;
	int ret, i;

	if (strstr(client->adapter->name, "SMBus I801")) {
		i2c_str = "smb";
		for (i = 0; i < d->nsens; i++) {
			struct intel_tsens_host *tsens = d->tsens[i];

			tsens->sensor_name_smbus =
				kasprintf(GFP_KERNEL,
					  "%s_%s-%x",
					  tsens->t_data->name,
					  i2c_str, device_id);
			tsens->i2c_smbus = client;
			mutex_init(&tsens->sync_smb_unregister);
			ret = intel_tsens_add_tz(tsens,
						 &tsens->tz_smbus,
						 tsens->sensor_name_smbus,
						 &client->dev,
						 i);
			if (ret) {
				dev_err(&client->dev,
					"thermal zone configuration failed\n");
				intel_tsens_remove_tz(d);
				return ret;
			}
		}
	} else {
		i2c_str = "xlk";
		for (i = 0; i < d->nsens; i++) {
			struct intel_tsens_host *tsens = d->tsens[i];

			tsens->sensor_name_xlk =
				kasprintf(GFP_KERNEL,
					  "%s_%s-%x",
					  tsens->t_data->name,
					  i2c_str, device_id);
			tsens->i2c_xlk = client;
			mutex_init(&tsens->sync_xlk_unregister);
			ret = intel_tsens_add_tz(tsens,
						 &tsens->tz_xlk,
						 tsens->sensor_name_xlk,
						 &client->dev,
						 i);
			if (ret) {
				dev_err(&client->dev,
					"thermal zone configuration failed\n");
				intel_tsens_remove_tz(d);
				return ret;
			}
		}
	}

	i2c_set_clientdata(client, d);

	return 0;
}

static int intel_tsens_tj_exit(struct i2c_client *client)
{
	struct intel_hddl_clients *d = client->dev.platform_data;

	if (!d) {
		dev_err(&client->dev,
			"Unable to get private data\n");
		return -EINVAL;
	}
	intel_tsens_remove_tz(d);
	return 0;
}

static const struct i2c_device_id i2c_intel_tsens_id[] = {
	{ "intel_tsens", (kernel_ulong_t)NULL },
	{}
};
MODULE_DEVICE_TABLE(i2c, i2c_intel_tsens_id);

static struct i2c_driver i2c_intel_tsens_driver = {
	.driver = {
		.name = "intel_tsens",
	},
	.probe = intel_tsens_tj_probe,
	.remove = intel_tsens_tj_exit,
	.id_table = i2c_intel_tsens_id,
};
module_i2c_driver(i2c_intel_tsens_driver);

MODULE_DESCRIPTION("Intel tsens host Device driver");
MODULE_AUTHOR("Sandeep Singh <sandeep1.singh@intel.com>");
MODULE_AUTHOR("Vaidya, Mahesh R <mahesh.r.vaidya@intel.com>");
MODULE_AUTHOR("Udhayakumar C <udhayakumar.c@intel.com>");
MODULE_LICENSE("GPL v2");
