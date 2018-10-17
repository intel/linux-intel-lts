/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2018-2019 Intel Corporation. */

#ifndef __RPMB_KEY_ABL__
#define __RPMB_KEY_ABL__

int rpmb_key_abl_get(ulong params_addr, const u8 *dev_id, size_t dev_id_len,
		     size_t max_partition_num, u8 rpmb_key[][RPMB_KEY_LENGTH]);

#endif /* !__RPMB_KEY_ABL__ */
