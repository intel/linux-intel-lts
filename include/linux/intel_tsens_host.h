/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *
 * Intel tsens host I2C thermal Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 */

#ifndef _LINUX_INTEL_TSENS_HOST_DEVICE_H
#define _LINUX_INTEL_TSENS_HOST_DEVICE_H

struct intel_tsens_host_trip_info {
	enum thermal_trip_type trip_type;
	int temp;
} __packed __aligned(4);

struct intel_tsens_host {
	const char *sensor_name_smbus;
	const char *sensor_name_xlk;
	struct intel_tsens_data *t_data;
	struct intel_tsens_host_trip_info **trip_info;
	u32 device_id;
	struct i2c_client *i2c_xlk;
	struct i2c_client *i2c_smbus;
	struct thermal_zone_device *tz_xlk;
	struct thermal_zone_device *tz_smbus;
	struct mutex sync_smb_unregister;
	struct mutex sync_xlk_unregister;
};

struct intel_tsens_host_plat_data {
	int nsens;
	struct intel_tsens_host **tsens;
};
#endif /*_LINUX_INTEL_TSENS_HOST_DEVICE_H*/
