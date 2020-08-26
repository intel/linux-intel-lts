// SPDX-License-Identifier: GPL-2.0-only
/*
 * HDDL Device HELPER module.
 *
 * Copyright (C) 2019-2020 Intel Corporation
 */

#ifndef _LINUX_INTEL_TSENS_HOST_DEVICE_H
#define _LINUX_INTEL_TSENS_HOST_DEVICE_H

#include "../hddl_device/hddl_device.h"

__packed __aligned(size) struct intel_tsens_trip_info {
	enum thermal_trip_type trip_type;
	int temp;
};

struct intel_tsens {
	struct intel_tsens_data *t_data;
	struct intel_tsens_trip_info **trip_info;
	struct thermal_zone_device *tz;
};

struct intel_tsens_plat_data {
	int nsens;
	struct intel_tsens **tsens;
};

#endif /*_LINUX_INTEL_TSENS_HOST_DEVICE_H*/
