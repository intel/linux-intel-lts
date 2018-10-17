/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2018-2019 Intel Corporation. */

#ifndef __RPMB_KEY_SBL__
#define __RPMB_KEY_SBL__

int rpmb_key_sbl_get(ulong params_addr, size_t max_partition_num,
		     u8 rpmb_key[][RPMB_KEY_LENGTH]);

#endif /* __RPMB_KEY_SBL__ */
