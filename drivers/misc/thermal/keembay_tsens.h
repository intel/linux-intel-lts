/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Keembay Temperature sensor local-host module.
 *
 * Copyright (C) 2019-2020 Intel Corporation.
 */

#ifndef _KEEMBAY_TSENS_H
#define _KEEMBAY_TSENS_H


#include <linux/thermal.h>

enum keembay_thermal_trip {
	KEEMBAY_TRIP_PASSIVE,
	KEEMBAY_TRIP_CRITICAL,
	KEEMBAY_TRIP_NUM,
};

enum keembay_thermal_sensor_en {
	KEEMBAY_SENSOR_MSS,
	KEEMBAY_SENSOR_CSS,
	KEEMBAY_SENSOR_NCE,
	KEEMBAY_SENSOR_SOC
};

struct kmb_trip_info_st {
	enum thermal_trip_type trip_type;
	int temperature;
};



struct keembay_therm_info {
	int current_temp;
	int alarm_temp;
	void __iomem *regs_val;
	struct clk *thermal_clk;
	spinlock_t lock;
	int mss;
	int css;
	int nce;
	int nce0;
	int nce1;
	struct device *dev;
	const struct thermal_soc_data *socdata;
};

#define KEEMBAY_MAX_TRIP_INFO 10
struct kmb_trip_point_info {
	enum keembay_thermal_sensor_en sensor_type;
	const char *sensor_name;
	int n_trips;
	int passive_delay;
	int polling_delay;
	struct kmb_trip_info_st trip_info[KEEMBAY_MAX_TRIP_INFO];
	struct keembay_therm_info *thermal_info;
	struct thermal_zone_device *tz;
};

int *kmb_tj_get_temp_base(void);

#endif /* _KEEMBAY_TSENS_H */
