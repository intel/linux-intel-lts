// SPDX-License-Identifier: GPL-2.0-only
/*
 * Keembay HDDL module.
 *
 * Copyright (C) 2019-2020 Intel Corporation.
 */

#ifndef __HDDL_DEVICE_H
#define __HDDL_DEVICE_H

#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/thermal.h>
#include <linux/xlink.h>
#include <linux/xlink_drv_inf.h>

#define HDDL_MAGIC 'x'
#define HDDL_READ_SW_ID_DATA    _IOW(HDDL_MAGIC, 'a', void*)
#define HDDL_SOFT_RESET		_IOW(HDDL_MAGIC, 'b', void*)

typedef struct sw_id_hddl_data {
	uint32_t board_id;
	uint32_t soc_id;
	uint32_t soc_adaptor_no[2];
	uint32_t sw_id;
	uint32_t return_id;
} T_SW_ID_HDDL_DATA;

typedef struct sw_id_soft_reset {
	uint32_t sw_id;
	uint32_t return_id;
} T_SW_ID_SOFT_RESET;

enum hddl_device {
	HDDL_I2C_SLAVE = (1<<0),
	HDDL_XLINK_SLAVE = (1<<1),
	HDDL_XLINK_SMBUS_SLAVE = (1<<2),
};

enum hddl_device_status {
	HDDL_DEV_STATUS_START,
	HDDL_DEV_STATUS_CONNECTED,
	HDDL_DEV_STATUS_DISCONNECTED,
	HDDL_DEV_STATUS_END,
};

enum hddl_msg_type {
	HDDL_GET_NSENS = 0x10,
	HDDL_GET_SENS_NAME = 0x11,
	HDDL_GET_SENS_DETAILS = 0x12,
	HDDL_GET_SENS_TRIP_INFO = 0x13,
	HDDL_GET_N_I2C_DEVS = 0x14,
	HDDL_GET_I2C_DEVS = 0x15,
	HDDL_GET_I2C_DEV_ADDR = 0x16,
	HDDL_GET_SENS_COMPLETE = 0x20,
};

__packed __aligned(4) struct intel_hddl_tsens_msg {
	int msg_type;
	uint32_t index;
	uint32_t index2;
};

__packed __aligned(4) struct intel_hddl_board_info {
	int board_id;
	int soc_id;
};

__packed __aligned(4) struct intel_tsens_data {
	char name[20];
	u32 n_trips;
	u32 passive_delay;
	u32 polling_delay;
	u32 sensor_type;
};

__packed __aligned(4) struct intel_hddl_i2c_devs_data {
	char name[20];
	uint32_t addr;
	uint32_t bus;
	int enabled;
	int local_host;
	int remote_host;
};

struct intel_hddl_i2c_devs {
	char name[20];
	uint32_t addr;
	uint32_t bus;
	int enabled;
	int local_host;
	int remote_host;
	struct i2c_board_info board_info;
	struct i2c_client *xlk_client;
	struct i2c_client *i2c_client;
	struct i2c_client *smbus_client;
};

struct intel_hddl_clients {
	struct xlink_handle xlink_dev;
	struct task_struct *hddl_dev_connect_task;
	void *task;
	uint32_t chan_num;
	void *pdata;
	struct intel_hddl_board_info board_info;
	void **tsens;
	int nsens;
	uint32_t xlink_i2c_ch[2];
	uint32_t i2c_chan_num;
	struct platform_device *xlink_i2c_plt_dev[2];
	struct platform_device *pdev;
	struct i2c_adapter *adap[2];
	struct intel_hddl_i2c_devs **i2c_devs;
	int n_slaves;
	enum hddl_device_status status;
	struct mutex lock;
};

struct intel_tsens_plat_data {
	const char *name;
	void __iomem *base_addr;
	int (*get_temp)(struct platform_device *pdev, int type, int *temp);
	void *pdata;
};

__packed __aligned(4) struct intel_tsens_trip_info {
	enum thermal_trip_type trip_type;
	int temp;
};

struct intel_tsens_plat_info {
	const char *plat_name;
	struct platform_device *pdev;
	void __iomem *base_addr;
};

struct intel_tsens {
	char name[20];
	u32 n_trips;
	u32 passive_delay;
	u32 polling_delay;
	u32 sensor_type;
	u64 addr;
	u64 size;
	u32 curr_temp;
	void __iomem *base_addr;
	struct intel_tsens_trip_info **trip_info;
	struct thermal_zone_device *tz;
	void *pdata;
	struct intel_tsens_plat_info plat_info;
};

struct intel_tsens **intel_tsens_hddl_register(int *nsens);

#endif /* __HDDL_DEVICE_H */

