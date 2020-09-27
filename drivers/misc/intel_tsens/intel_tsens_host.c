// SPDX-License-Identifier: GPL-2.0-only
/*
 * HDDL Device Kernel module.
 *
 * Copyright (C) 2019-2020 Intel Corporation
 */

#include <asm/page.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/printk.h>
#include <uapi/linux/stat.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/xlink.h>
#include <linux/time.h>
#include <linux/kmod.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/sched/mm.h>
#include <linux/sched/task.h>
#include <linux/hddl_device.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/thermal.h>
#include <linux/hddl_device.h>
#include <linux/intel_tsens_host.h>

static int intel_tsens_get_temp(struct thermal_zone_device *zone,
							int *temp)
{
	struct intel_tsens_host *tsens =
		(struct intel_tsens_host *)zone->devdata;
	struct i2c_client *i2c_c;

	if (strstr(zone->type, "smb") != NULL)
		i2c_c = tsens->i2c_smbus;
	else
		i2c_c = tsens->i2c_xlk;

	*temp = i2c_smbus_read_word_data(i2c_c,
				tsens->t_data->sensor_type);
	return 0;
}

static int intel_tsens_thermal_get_trip_type(struct thermal_zone_device *zone,
			int trip, enum thermal_trip_type *type)
{
	struct intel_tsens_host *tsens =
		(struct intel_tsens_host *)zone->devdata;

	*type = tsens->trip_info[trip]->trip_type;
	return 0;
}

static int intel_tsens_thermal_get_trip_temp(struct thermal_zone_device *zone,
				int trip, int *temp)
{
	struct intel_tsens_host *tsens =
		(struct intel_tsens_host *)zone->devdata;

	*temp = tsens->trip_info[trip]->temp;
	return 0;
}

static int intel_tsens_thermal_notify(struct thermal_zone_device *zone,
			       int trip, enum thermal_trip_type type)
{

	switch (type) {
	case THERMAL_TRIP_ACTIVE:
		pr_warn("Thermal reached to active temperature\n");
		break;
	case THERMAL_TRIP_CRITICAL:
		pr_warn("Thermal reached to critical temperature\n");
		break;
	default:
		pr_warn("Thermal not reached to active temperature\n");
		break;
	}
	thermal_generate_netlink_event(zone, type);
	return 0;
}

static int intel_tsens_bind(struct thermal_zone_device *tz,
		    struct thermal_cooling_device *cdev)
{
	int ret;

	/*Check here thermal device zone name and*/
	/*cdev name to match, then call the bind device */
	if (strncmp("intel_tsens_thermal", cdev->type,
				THERMAL_NAME_LENGTH) == 0) {
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

	if (strncmp("intel_tsens_thermal", cdev->type,
				THERMAL_NAME_LENGTH) == 0) {
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

static int intel_tsens_add_thermal_zone(struct intel_tsens_host *tsens,
					struct thermal_zone_device **tz,
					const char *name,
					struct device *dev,
					int i)
{
	int ret;

	*tz =  thermal_zone_device_register(
			name,
			tsens->t_data->n_trips,
			0,
			tsens,
			&tsens_thermal_ops,
			NULL,
			tsens->t_data->passive_delay,
			tsens->t_data->polling_delay
			);
	if (IS_ERR(*tz)) {
		ret = PTR_ERR(*tz);
		dev_err(dev,
			"failed to register thermal zone device %s\n",
			tsens->t_data->name);
		goto remove_thermal_zone;
	}
	return 0;

remove_thermal_zone:
	/*TODO: unregister thermal zone */
	return ret;
}

static int intel_tsens_tj_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct intel_hddl_clients *d = client->dev.platform_data;
	uint32_t *device_id = &d->xlink_dev.sw_device_id;
	char *i2c_str;
	int ret, i;

	if (strstr(client->adapter->name, "SMBus I801") != NULL) {
		i2c_str = "smb";
		for (i = 0; i < d->nsens; i++) {
			struct intel_tsens_host *tsens = d->tsens[i];

			tsens->sensor_name_smbus = kasprintf(GFP_KERNEL,
				"%s_%s-%x", tsens->t_data->name,
				i2c_str, *device_id);
			tsens->i2c_smbus = client;
			ret = intel_tsens_add_thermal_zone(tsens,
					&tsens->tz_smbus,
					tsens->sensor_name_smbus,
					&client->dev,
					i);
			if (ret) {
				dev_err(&client->dev, "thermal zone configuration failed\n");
				return ret;
			}
		}
	} else {
		i2c_str = "xlk";
		for (i = 0; i < d->nsens; i++) {
			struct intel_tsens_host *tsens = d->tsens[i];

			tsens->sensor_name_xlk = kasprintf(GFP_KERNEL,
				"%s_%s-%x", tsens->t_data->name,
				i2c_str, *device_id);
			tsens->i2c_xlk = client;
			ret = intel_tsens_add_thermal_zone(tsens,
					&tsens->tz_xlk,
					tsens->sensor_name_xlk,
					&client->dev,
					i);
			if (ret) {
				dev_err(&client->dev, "thermal zone configuration failed\n");
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
	int i;

	if (strstr(client->adapter->name, "SMBus I801") != NULL) {
		for (i = 0; i < d->nsens; i++) {
			struct intel_tsens_host *tsens = d->tsens[i];

			thermal_zone_device_unregister(
				tsens->tz_smbus);
		}
	} else {
		for (i = 0; i < d->nsens; i++) {
			struct intel_tsens_host *tsens = d->tsens[i];

			thermal_zone_device_unregister(
				tsens->tz_xlk);
		}
	}
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

MODULE_DESCRIPTION("KeemBay HDDL Device driver");
MODULE_AUTHOR("Raja Subramanian, Lakshmi Bai"
	      "<lakshmi.bai.raja.subramanian@intel.com>");
MODULE_AUTHOR("Sandeep Singh <sandeep1.singh@intel.com>");
MODULE_AUTHOR("Vaidya, Mahesh R <mahesh.r.vaidya@intel.com>");
MODULE_AUTHOR("Udhayakumar C <udhayakumar.c@intel.com>");
MODULE_LICENSE("GPL v2");
