// SPDX-License-Identifier: GPL-2.0-only
/*
 * HDDL Device HELPER module.
 *
 * Copyright (C) 2019-2020 Intel Corporation
 */

#ifndef _LINUX_INTEL_TSENS_HOST_DEVICE_H
#define _LINUX_INTEL_TSENS_HOST_DEVICE_H

__packed __aligned(4) struct intel_tsens_host_trip_info {
	enum thermal_trip_type trip_type;
	int temp;
};

struct intel_tsens_host {
	const char *sensor_name_smbus;
	const char *sensor_name_xlk;
	struct intel_tsens_data *t_data;
	struct intel_tsens_host_trip_info **trip_info;
	uint32_t device_id;
	struct i2c_client *i2c_xlk;
	struct i2c_client *i2c_smbus;
	struct thermal_zone_device *tz_xlk;
	struct thermal_zone_device *tz_smbus;
};

struct intel_tsens_host_plat_data {
	int nsens;
	struct intel_tsens_host **tsens;
};

#endif /*_LINUX_INTEL_TSENS_HOST_DEVICE_H*/
