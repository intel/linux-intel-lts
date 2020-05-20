// SPDX-License-Identifier: GPL-2.0-only
/*
 * Keembay Temperature sensor module.
 *
 * Copyright (C) 2019-2020 Intel Corporation.
 */

#ifndef _LINUX_KEEMBAY_TSENS_HOST_H
#define _LINUX_KEEMBAY_TSENS_HOST_H

#include <linux/thermal.h>


enum keembay_thermal_trip {
	KEEMBAY_TRIP_ACTIVE,
	KEEMBAY_TRIP_PASSIVE,
	KEEMBAY_TRIP_CRITICAL,
	KEEMBAY_TRIP_NUM,
};

enum keembay_thermal_sensor_en {
	KEEMBAY_SENSOR_MSS,
	KEEMBAY_SENSOR_CSS,
	KEEMBAY_SENSOR_NCE,
	KEEMBAY_SENSOR_SOC,
	KEEMBAY_SENSOR_MSS_HOST,
	KEEMBAY_SENSOR_CSS_HOST,
	KEEMBAY_SENSOR_NCE_HOST,
	KEEMBAY_SENSOR_SOC_HOST
};

struct kmb_trip_info_st {
	enum thermal_trip_type trip_type;
	int temperature;
};


struct keembay_therm_info {
	struct i2c_client *i2c_c;
	struct i2c_adapter *host_kmb_tj_adap;
	spinlock_t lock;
	struct device *dev;
	int temperature;
};


#define HOST_KEEMBAY_MAX_TRIP_INFO 10
struct kmb_trip_point_info {
	enum keembay_thermal_sensor_en sensor_type;
	const char *sensor_name;
	int n_trips;
	int passive_delay;
	int polling_delay;
	struct kmb_trip_info_st trip_info[HOST_KEEMBAY_MAX_TRIP_INFO];
	struct keembay_therm_info *thermal_info;
	struct thermal_zone_device *tz;
	int kmb_address;
};

#endif /* _LINUX_KEEMBAY_TSENS_HOST_H */
