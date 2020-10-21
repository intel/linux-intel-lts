/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *
 * High Density Deep Learning utils.
 *
 * Copyright (C) 2020 Intel Corporation
 *
 */

#ifndef _LINUX_HDDL_DEVICE_UTIL_H
#define _LINUX_HDDL_DEVICE_UTIL_H

#include <linux/xlink_drv_inf.h>
#include "../xlink-core/xlink-defs.h"

#define HDDL_NEW_DEV_POLL_TIME 2000

typedef int (*intel_hddl_connect_task)(void *);

struct intel_hddl_clients **
	intel_hddl_setup_device(struct device *dev,
				intel_hddl_connect_task task, u32 *n_devs,
				struct intel_hddl_clients **hddl_clients,
				void *pdata);

int intel_hddl_xlink_remove_i2c_adap(struct device *dev,
				     struct intel_hddl_clients *c);

void intel_hddl_add_xlink_i2c_clients(struct device *dev,
				      struct intel_hddl_clients *c,
				      struct intel_hddl_i2c_devs **i2c_devs,
				      int n_clients, int remote);

int intel_hddl_register_xlink_i2c_adap(struct device *dev,
				       struct intel_hddl_clients *c);

struct intel_hddl_clients **intel_hddl_get_clients(int *n_devs);

void intel_hddl_device_remove(struct intel_hddl_clients *d);

void intel_hddl_unregister_pdev(struct intel_hddl_clients *c);

void intel_hddl_close_xlink_device(struct device *dev,
				   struct intel_hddl_clients *d);

int intel_hddl_open_xlink_device(struct device *dev,
				 struct intel_hddl_clients *d);

void intel_hddl_free_i2c_client(struct intel_hddl_clients *d,
				struct intel_hddl_i2c_devs *i2c_dev);

#endif /* _LINUX_HDDL_DEVICE_UTIL_H */
