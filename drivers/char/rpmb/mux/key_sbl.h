/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0 */

#ifndef __RPMB_KEY_SBL__
#define __RPMB_KEY_SBL__

int rpmb_key_sbl_get(ulong params_addr, size_t max_partition_num,
		     u8 rpmb_key[][RPMB_KEY_LENGTH]);

#endif /* __RPMB_KEY_SBL__ */
