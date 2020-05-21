// SPDX-License-Identifier: GPL-2.0-only
/*
 * Keembay HDDL module.
 *
 * Copyright (C) 2019-2020 Intel Corporation.
 */

#ifndef __HDDL_DEVICE_H
#define __HDDL_DEVICE_H

#define HDDL_MAGIC 'x'
#define HDDL_READ_SW_ID_DATA    _IOW(HDDL_MAGIC, 'a', void*)
#define HDDL_SOFT_RESET		      _IOW(HDDL_MAGIC, 'b', void*)

typedef struct hddl_device_kmb_st {
	uint32_t board_id;
	struct kmb {
		uint32_t  id;
		struct xlink_handle devH;
		struct platform_device_info host_kmb_tj_info;
		struct platform_device *host_kmb_tj_plt_dev;
		uint32_t xlink_i2c_ch[2];
		struct i2c_adapter adap[2];
		struct platform_device *xlink_i2c_plt_dev[2];
	} soc[3];
} T_HDDL_DEVICE_KMB_NODE;

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

#endif /* __HDDL_DEVICE_H */

