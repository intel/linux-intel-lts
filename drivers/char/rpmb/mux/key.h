/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0 */
/*
 * RPMB Key management: retrieve and distribute
 *
 * Copyright (c) 2018 Intel Corporation. All rights reserved.
 */

#ifndef __RPMB_KEY_H__
#define __RPMB_KEY_H__

/*
 * Storage may support multiple rpmb partitions, but the specification
 * does not specify the max number of rpmb partitions.
 * Here we use 6 for now. In future, this may need to be expanded
 * dynamically.
 */
#define RPMB_MAX_PARTITION_NUMBER 6U

#define RPMB_KEY_LENGTH 64U

#ifdef CONFIG_RPMB_MUX_KEY
int rpmb_key_get(const u8 *dev_id, size_t dev_id_len,
		 size_t max_partition_num, u8 rpmb_key[][RPMB_KEY_LENGTH]);
#else
static inline
int rpmb_key_get(const u8 *dev_id, size_t dev_id_len,
		 size_t max_partition_num, u8 rpmb_key[][RPMB_KEY_LENGTH])
{
	return -EOPNOTSUPP;
}
#endif

#endif /* !__RPMB_KEY_H__ */
