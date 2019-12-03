// SPDX-License-Identifier: GPL-2.0
/*
 * Parse legacy seed from ABL(Automotive Bootloader). Derive a rpmb key
 * with the legacy seed.
 *
 * Copyright (c) 2018-2019 Intel Corporation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ":%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <crypto/hash.h>

#include "key.h"
#include "key_abl.h"
#include "mux_hkdf.h"

#define ABL_SEED_LEN 32U
#define ABL_SEED_LIST_MAX 4U
#define EMMC_SERIAL_LEN 15U

struct abl_seed_info {
	u8 svn;
	u8 reserved[3];
	u8 seed[ABL_SEED_LEN];
};

struct dev_sec_info {
	u32 size_of_this_struct;
	u32 version;
	u32 num_seeds;
	struct abl_seed_info seed_list[ABL_SEED_LIST_MAX];
};

/*
 * The output serial is concatenation of mmc product name with a string
 * representation of PSN.
 */
static int rpmb_key_abl_build_serial(const u8 *cid, u8 *serial)
{
	u32 psn;

	if (!cid || !serial)
		return -EFAULT;

	psn = (cid[9] << 24) | (cid[8] << 16) | (cid[15] << 8) | cid[14];

	serial[0] = cid[0];
	serial[1] = cid[7];
	serial[2] = cid[6];
	serial[3] = cid[5];
	serial[4] = cid[4];
	serial[5] = cid[11];

	snprintf(&serial[6], 9, "%08x", psn);

	return 0;
}

int rpmb_key_abl_get(ulong params_addr, const u8 *dev_id, size_t dev_id_len,
		     size_t max_partition_num, u8 rpmb_key[][RPMB_KEY_LENGTH])
{
	u32 i, legacy_seed_index = 0;
	struct dev_sec_info *sec_info;
	struct abl_seed_info *seed_list;
	u8 serial[EMMC_SERIAL_LEN] = {0};
	int ret;

	if (!params_addr || !dev_id || !dev_id_len || !max_partition_num) {
		pr_err("Invalid input params!\n");
		return -EFAULT;
	}

	ret = rpmb_key_abl_build_serial(dev_id, serial);
	if (ret) {
		pr_err("Failed to build serial from cid\n");
		return -EFAULT;
	}

	sec_info = memremap(params_addr, sizeof(*sec_info), MEMREMAP_WB);
	if (!sec_info) {
		pr_err("Remap params_addr failed!\n");
		return -EFAULT;
	}
	seed_list = &sec_info->seed_list[0];

	/*
	 * The seed_list must contain at least 2 seeds: 1 is legacy
	 * seed and others are SVN based seed.
	 */
	if (sec_info->num_seeds < 2U ||
	    sec_info->num_seeds > ABL_SEED_LIST_MAX) {
		pr_err("Invalid seed number!\n");
		memunmap(sec_info);
		return -EFAULT;
	}

	/*
	 * The seed_list from ABL contains several seeds which based on SVN
	 * and one legacy seed which is not based on SVN. The legacy seed's
	 * svn value is minimum in the seed list. And CSE ensures at least two
	 * seeds will be generated which will contain the legacy seed.
	 * Here find the legacy seed index first.
	 */
	for (i = 1; i < sec_info->num_seeds; i++) {
		if (seed_list[i].svn < seed_list[legacy_seed_index].svn)
			legacy_seed_index = i;
	}

	/*
	 * The eMMC Field Firmware Update would impact below fields of
	 * CID(Card Identification):
	 *     CID[6]:PRV (Product Revision)
	 *     CID[0]:CRC (CRC7 checksum)
	 * Mapping relation between CID and eMMC serial:
	 *     serial[0] = CID[0]
	 *     serial[2] = CID[6]
	 * So mask off serial[0]/serial[2] fields when using eMMC serial
	 * to derive rpmb key.
	 */
	serial[0] ^= serial[0];
	serial[2] ^= serial[2];

	/*
	 * Derive RPMB key from legacy seed with storage serial number.
	 * Currently, only support eMMC storage device, UFS storage device is
	 * not supported.
	 */
	ret = mux_hkdf_sha256(&rpmb_key[0][0], SHA256_HASH_SIZE,
			      (const u8 *)&seed_list[legacy_seed_index].seed[0],
			      ABL_SEED_LEN,
			      NULL, 0,
			      (const u8 *)serial, sizeof(serial));

	memset(&seed_list[legacy_seed_index], 0, sizeof(struct abl_seed_info));
	memunmap(sec_info);

	return ret;
}
