/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *
 * High Density Deep Learning Kernel module.
 *
 * Copyright (C) 2020 Intel Corporation
 *
 */

#ifndef __HDDL_DEVICE_H
#define __HDDL_DEVICE_H

#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/types.h>
#if IS_ENABLED(CONFIG_XLINK_CORE)
#include <linux/xlink.h>
#include <linux/xlink_drv_inf.h>
#endif /* XLINK_CORE */

#define HDDL_ALIGN 4

#define HDDL_MAGIC 'x'
#define HDDL_READ_SW_ID_DATA		_IOW(HDDL_MAGIC, 'a', void*)
#define HDDL_SOFT_RESET		_IOW(HDDL_MAGIC, 'b', void*)

struct sw_id_hddl_data {
	u32 board_id;
	u32 soc_id;
	u32 soc_adaptor_no[2];
	u32 sw_id;
	u32 return_id;
};

struct sw_id_soft_reset {
	u32 sw_id;
	u32 return_id;
};

enum hddl_xlink_adapter {
	HDDL_XLINK_I2C_MASTER,
	HDDL_XLINK_I2C_SLAVE,
	HDDL_XLINK_I2C_END,
};

enum hddl_device {
	HDDL_I2C_CLIENT		= (1 << 0),
	HDDL_XLINK_CLIENT	= (1 << 1),
	HDDL_XLINK_SMBUS_CLIENT	= (1 << 2),
};

enum hddl_device_status {
	HDDL_DEV_STATUS_START,
	HDDL_DEV_STATUS_CONNECTED,
	HDDL_DEV_STATUS_DISCONNECTED,
	HDDL_DEV_STATUS_END,
};

enum hddl_msg_type {
	HDDL_GET_NSENS		= 0x10,
	HDDL_GET_SENS_NAME	= 0x11,
	HDDL_GET_SENS_DETAILS	= 0x12,
	HDDL_GET_SENS_TRIP_INFO	= 0x13,
	HDDL_GET_N_I2C_DEVS	= 0x14,
	HDDL_GET_I2C_DEVS	= 0x15,
	HDDL_GET_I2C_DEV_ADDR	= 0x16,
	HDDL_GET_SENS_COMPLETE	= 0x20,
};

struct intel_hddl_tsens_msg {
	int msg_type;
	u32 sensor_type;
	u32 trip_info_idx;
} __packed __aligned(HDDL_ALIGN);

struct intel_hddl_board_info {
	int board_id;
	int soc_id;
} __packed __aligned(HDDL_ALIGN);

struct intel_tsens_data {
	char name[20];
	u32 n_trips;
	u32 passive_delay;
	u32 polling_delay;
	u32 sensor_type;
} __packed __aligned(HDDL_ALIGN);

struct intel_hddl_i2c_devs_data {
	char name[20];
	u32 addr;
	u32 bus;
	int enabled;
	int local_host;
	int remote_host;
} __packed __aligned(HDDL_ALIGN);

struct intel_hddl_i2c_devs {
	char name[20];
	u32 addr;
	u32 bus;
	int enabled;
	int local_host;
	int remote_host;
	struct i2c_board_info board_info;
	struct i2c_client *xlk_client;
	struct i2c_client *i2c_client;
	struct i2c_client *smbus_client;
};

struct intel_hddl_clients {
#if IS_ENABLED(CONFIG_XLINK_CORE)
	struct xlink_handle xlink_dev;
#endif /* XLINK_CORE */
	struct task_struct *hddl_dev_connect_task;
	void *task;
	u32 chan_num;
	void *pdata;
	struct intel_hddl_board_info board_info;
	u32 xlink_i2c_ch[HDDL_XLINK_I2C_END];
	u32 i2c_chan_num;
	void **tsens;
	u32 nsens;
	struct platform_device *xlink_i2c_plt_dev[HDDL_XLINK_I2C_END];
	struct platform_device *pdev;
	struct i2c_adapter *adap[HDDL_XLINK_I2C_END];
	struct i2c_adapter *smbus_adap;
	struct intel_hddl_i2c_devs **i2c_devs;
	int n_clients;
	enum hddl_device_status status;
	/* hddl device lock */
	struct mutex lock;
};

struct intel_tsens_trip_info {
	enum thermal_trip_type trip_type;
	int temp;
} __packed __aligned(HDDL_ALIGN);

#if IS_ENABLED(CONFIG_XLINK_CORE)
static inline u32 tsens_get_device_id(struct intel_hddl_clients *d)
{
	return d->xlink_dev.sw_device_id;
}
#else
static inline u32 tsens_get_device_id(struct intel_hddl_clients *d)
{
	return -EINVAL;
}
#endif /* XLINK_CORE */

#endif /* __HDDL_DEVICE_H */
