// SPDX-License-Identifier: GPL-2.0-only
/*
 * host_kmb_tj.c - Host KeemBay Thermal Driver.
 *
 * Copyright (C) 2019-2020 Intel Corporation
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/delay.h>
#include "keembay_tsens_host.h"
#include <linux/slab.h>

#include <linux/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

static int keembay_get_temp_host(struct thermal_zone_device *thermal,
							int *temp)
{
	struct kmb_trip_point_info *kmb_zone_info = thermal->devdata;
	struct keembay_therm_info *ktherm = kmb_zone_info->thermal_info;
	unsigned char rx_buf[4] = {0, 0, 0, 0};

	spin_lock(&ktherm->lock);
	*temp = 0;
	switch (kmb_zone_info->sensor_type) {
	case KEEMBAY_SENSOR_MSS_HOST:
		rx_buf[0] = i2c_smbus_read_byte_data(ktherm->i2c_c, 8);
		mdelay(100);
		rx_buf[1] = i2c_smbus_read_byte_data(ktherm->i2c_c, 9);
		*temp = (rx_buf[1] << (8) | rx_buf[0]);
		break;

	case KEEMBAY_SENSOR_CSS_HOST:
		rx_buf[0] = i2c_smbus_read_byte_data(ktherm->i2c_c, 12);
		mdelay(100);
		rx_buf[1] = i2c_smbus_read_byte_data(ktherm->i2c_c, 13);
		*temp = (rx_buf[1] << (8) | rx_buf[0]);
		break;

	case KEEMBAY_SENSOR_NCE_HOST:
		rx_buf[0] = i2c_smbus_read_byte_data(ktherm->i2c_c, 4);
		mdelay(100);
		rx_buf[1] = i2c_smbus_read_byte_data(ktherm->i2c_c, 5);
		*temp = (rx_buf[1] << (8) | rx_buf[0]);
		break;

	case KEEMBAY_SENSOR_SOC_HOST:
		rx_buf[0] = i2c_smbus_read_byte_data(ktherm->i2c_c, 0);
		mdelay(100);
		rx_buf[1] = i2c_smbus_read_byte_data(ktherm->i2c_c, 1);
		*temp = (rx_buf[1] << (8) | rx_buf[0]);
		break;

	default:
		break;
	}
	spin_unlock(&ktherm->lock);

	/* TODO: How to do error handling here */
	return 0;
}

static int keembay_thermal_get_trip_type_host(struct thermal_zone_device *zone,
			int trip, enum thermal_trip_type *type)
{

	struct kmb_trip_point_info *kmb_zone_info = zone->devdata;

	*type = kmb_zone_info->trip_info[trip].trip_type;
	return 0;
}


static int keembay_thermal_get_trip_temp_host(struct thermal_zone_device *zone,
				int trip, int *temp)
{

	struct kmb_trip_point_info *kmb_zone_info = zone->devdata;

	*temp = kmb_zone_info->trip_info[trip].temperature;
	return 0;
}



static int keembay_thermal_notify_host(struct thermal_zone_device *zone,
			       int trip, enum thermal_trip_type type)
{

	switch (type) {
	case THERMAL_TRIP_ACTIVE:
		printk(KERN_WARNING "Thermal reached to active temperature\n");
		break;
	case THERMAL_TRIP_CRITICAL:
		printk(KERN_WARNING "Thermal reached to critical temperature\n");
		break;
	default:
		printk(KERN_WARNING "Thermal not reached to active temperature\n");
		break;
	}
	thermal_generate_netlink_event(zone, type);
	return 0;
}

static int keembay_bind_host(struct thermal_zone_device *tz,
		    struct thermal_cooling_device *cdev)
{
	int ret;

	/*Check here thermal device zone name and*/
	/*cdev name to match, then call the bind device */
	if (strncmp("keembay_thermal", cdev->type, THERMAL_NAME_LENGTH) == 0) {
		ret = thermal_zone_bind_cooling_device
				(tz,
				KEEMBAY_TRIP_ACTIVE,
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

static int keembay_unbind_host(struct thermal_zone_device *tz,
		      struct thermal_cooling_device *cdev)
{
	int ret;

	ret = thermal_zone_unbind_cooling_device(tz, 0, cdev);
	if (ret) {
		dev_err(&tz->device,
			"unbinding zone %s with cdev %s failed:%d\n",
			tz->type, cdev->type, ret);
		return ret;
	}
	return 0;
}


static struct thermal_zone_device_ops ops_host = {
	.bind = keembay_bind_host,
	.unbind = keembay_unbind_host,
	.get_temp = keembay_get_temp_host,
	.get_trip_type	= keembay_thermal_get_trip_type_host,
	.get_trip_temp	= keembay_thermal_get_trip_temp_host,
	.notify		= keembay_thermal_notify_host,

};

int keembay_thermal_zone_register_host(
		struct kmb_trip_point_info *zone_trip_info)
{
	int ret;

	zone_trip_info->tz =  thermal_zone_device_register(
		zone_trip_info->sensor_name,
		zone_trip_info->n_trips,
		0,
		zone_trip_info,
		&ops_host,
		NULL,
		zone_trip_info->passive_delay,
		zone_trip_info->polling_delay
		);
	if (IS_ERR(zone_trip_info->tz)) {
		ret = PTR_ERR(zone_trip_info->tz);
		printk(KERN_WARNING "failed to"
				"register thermal zone device %d\n", ret);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(keembay_thermal_zone_register_host);

int keembay_thermal_zone_unregister_host(
			struct kmb_trip_point_info *zone_trip_info)
{
	thermal_zone_device_unregister(zone_trip_info->tz);
	if (zone_trip_info->thermal_info != NULL)
		kfree(zone_trip_info->thermal_info);
	return 0;
}


struct kmb_trip_point_info mss_zone_trip_info_host = {
	.sensor_type = KEEMBAY_SENSOR_MSS_HOST,
	.sensor_name = NULL,
	.passive_delay = 1000,
	.polling_delay = 2000,
	.n_trips = 2,
	.trip_info = {
		{ THERMAL_TRIP_ACTIVE, 40000 },
		{ THERMAL_TRIP_ACTIVE, 80000 },
	},
	NULL,
	NULL,
};

struct kmb_trip_point_info css_zone_trip_info_host = {
	.sensor_type = KEEMBAY_SENSOR_CSS_HOST,
	.sensor_name = NULL,
	.passive_delay = 1000,
	.polling_delay = 2000,
	.n_trips = 2,
	.trip_info = {
		{ THERMAL_TRIP_ACTIVE, 40000 },
		{ THERMAL_TRIP_ACTIVE, 80000 },
	},
	NULL,
	NULL,
};

struct kmb_trip_point_info nce_max_zone_trip_info_host = {
	.sensor_type = KEEMBAY_SENSOR_NCE_HOST,
	.sensor_name = NULL,
	.passive_delay = 1000,
	.polling_delay = 2000,
	.n_trips = 2,
	.trip_info = {
		{ THERMAL_TRIP_ACTIVE, 40000 },
		{ THERMAL_TRIP_ACTIVE, 80000 },
	},
	NULL,
	NULL,
};

struct kmb_trip_point_info soc_max_zone_trip_info_host = {
	.sensor_type = KEEMBAY_SENSOR_SOC_HOST,
	.sensor_name = NULL,
	.passive_delay = 1000,
	.polling_delay = 2000,
	.n_trips = 2,
	.trip_info = {
		{ THERMAL_TRIP_ACTIVE, 40000 },
		{ THERMAL_TRIP_ACTIVE, 80000 },
	},
	NULL,
	NULL,
};

static int host_kmb_tj_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct kmb_trip_point_info *host_kmb_trip_info1 = NULL;
	struct kmb_trip_point_info *host_kmb_trip_info2 = NULL;
	struct kmb_trip_point_info *host_kmb_trip_info3 = NULL;
	struct kmb_trip_point_info *host_kmb_trip_info4 = NULL;
	char *i2c_str;
	uint32_t *device_id = client->dev.platform_data;

	if (strstr(client->adapter->name, "SMBus I801") != NULL)
		i2c_str = "smb";
	else
		i2c_str = "xlk";

	host_kmb_trip_info1 = kzalloc(
			sizeof(struct kmb_trip_point_info), GFP_KERNEL);
	memcpy(host_kmb_trip_info1, &mss_zone_trip_info_host,
				sizeof(struct kmb_trip_point_info));
	host_kmb_trip_info1->sensor_name = kasprintf(GFP_KERNEL,
				"mss_%s-%x", i2c_str, *device_id);
	host_kmb_trip_info1->thermal_info =  kzalloc(
			sizeof(struct keembay_therm_info), GFP_KERNEL);
	host_kmb_trip_info1->thermal_info->i2c_c = client;

	host_kmb_trip_info2 = kzalloc(
			sizeof(struct kmb_trip_point_info), GFP_KERNEL);
	memcpy(host_kmb_trip_info2, &css_zone_trip_info_host,
				sizeof(struct kmb_trip_point_info));
	host_kmb_trip_info2->sensor_name = kasprintf(GFP_KERNEL,
				"css_%s-%x", i2c_str, *device_id);
	host_kmb_trip_info2->thermal_info =  kzalloc(
			sizeof(struct keembay_therm_info), GFP_KERNEL);
	host_kmb_trip_info2->thermal_info->i2c_c = client;

	host_kmb_trip_info3 = kzalloc(
			sizeof(struct kmb_trip_point_info), GFP_KERNEL);
	memcpy(host_kmb_trip_info3, &nce_max_zone_trip_info_host,
				sizeof(struct kmb_trip_point_info));
	host_kmb_trip_info3->sensor_name = kasprintf(GFP_KERNEL,
				"nce_%s-%x", i2c_str, *device_id);
	host_kmb_trip_info3->thermal_info =
		kzalloc(sizeof(struct keembay_therm_info), GFP_KERNEL);
	host_kmb_trip_info3->thermal_info->i2c_c = client;

	host_kmb_trip_info4 = kzalloc(
			sizeof(struct kmb_trip_point_info), GFP_KERNEL);
	memcpy(host_kmb_trip_info4, &soc_max_zone_trip_info_host,
				sizeof(struct kmb_trip_point_info));
	host_kmb_trip_info4->sensor_name = kasprintf(
			GFP_KERNEL, "soc_%s-%x", i2c_str, *device_id);
	host_kmb_trip_info4->thermal_info =  kzalloc(
			sizeof(struct keembay_therm_info), GFP_KERNEL);
	host_kmb_trip_info4->thermal_info->i2c_c = client;

	/* i2c_set_clientdata(client, host_kmb_trip_info); */
	keembay_thermal_zone_register_host(host_kmb_trip_info1);
	keembay_thermal_zone_register_host(host_kmb_trip_info2);
	keembay_thermal_zone_register_host(host_kmb_trip_info3);
	keembay_thermal_zone_register_host(host_kmb_trip_info4);

	printk(KERN_INFO "host_kmb_tj: probe success\n");

	return 0;
}

static int host_kmb_tj_exit(struct i2c_client *client)
{
	struct kmb_trip_point_info *host_kmb_trip_info =
					i2c_get_clientdata(client);

	kfree(host_kmb_trip_info->thermal_info);
	kfree(host_kmb_trip_info);

	return 0;
}


static const struct i2c_device_id i2c_host_kmb_tj_id[] = {
	{ "host_kmb_tj", 16 },
	{}
};
MODULE_DEVICE_TABLE(i2c, i2c_host_kmb_tj_id);

static struct i2c_driver i2c_host_kmb_tj_driver = {
	.driver = {
		.name = "host_kmb_tj",
	},
	.probe = host_kmb_tj_probe,
	.remove = host_kmb_tj_exit,
	.id_table = i2c_host_kmb_tj_id,
};
module_i2c_driver(i2c_host_kmb_tj_driver);


MODULE_DESCRIPTION("Host KeemBay Thermal Driver");
MODULE_AUTHOR("Sandeep Singh <sandeep1.singh@intel.com>");
MODULE_AUTHOR("Vaidya, Mahesh R <mahesh.r.vaidya@intel.com>");
MODULE_LICENSE("GPL v2");
