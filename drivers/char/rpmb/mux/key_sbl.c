// SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0
/*
 * Parse RPMB key from SBL(SlimBootloader).
 *
 * Copyright (c) 2018 Intel Corporation. All rights reserved.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ":%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/io.h>

#include "key.h"
#include "key_sbl.h"

#define SEED_ENTRY_TYPE_SVNSEED             0x1U
#define SEED_ENTRY_TYPE_RPMBSEED            0x2U

#define SEED_ENTRY_USAGE_BASE_ON_SERIAL     0x1U
#define SEED_ENTRY_USAGE_NOT_BASE_ON_SERIAL 0x2U

struct image_boot_params {
	u32 size_of_this_struct;
	u32 version;
	u64 p_seed_list;
	u64 p_platform_info;
	u64 reserved;
};

struct seed_entry {
	/* SVN based seed or RPMB seed or attestation key_box */
	u8 type;
	/* For SVN seed: useed or dseed
	 * For RPMB seed: serial number based or not
	 */
	u8 usage;
	/* index for the same type and usage seed */
	u8 index;
	u8 reserved;
	/* reserved for future use */
	u16 flags;
	/* Total size of this seed entry */
	u16 seed_entry_size;
	/* SVN seed: struct seed_info
	 * RPMB seed: u8 rpmb_seed[key_len]
	 */
	u8 seed[0];
};

struct seed_list_hob {
	u8 revision;
	u8 rsvd0[3];
	u32 buffer_size;
	u8 total_seed_count;
	u8 rsvd1[3];
	struct seed_entry entry[0];
};

static int rpmb_key_sbl_parse_seed_list(struct seed_list_hob *seed_hob,
					size_t max_partition_num,
					u8 rpmb_seed[][RPMB_KEY_LENGTH])
{
	u8 i;
	u8 index = 0U;
	struct seed_entry *entry;

	if (!seed_hob || !max_partition_num) {
		pr_warn("Invalid input parameters!\n");
		goto fail;
	}

	if (seed_hob->total_seed_count == 0U) {
		pr_warn("Total seed count is 0.\n");
		goto fail;
	}

	entry = seed_hob->entry;

	for (i = 0U; i < seed_hob->total_seed_count; i++) {
		if ((u8 *)entry >= (u8 *)seed_hob + seed_hob->buffer_size) {
			pr_warn("Exceed memory boundray!\n");
			goto fail;
		}

		/* retrieve rpmb seed */
		if (entry->type == SEED_ENTRY_TYPE_RPMBSEED) {
			if (entry->index != 0) {
				pr_warn("RPMB usage mismatch!\n");
				goto fail;
			}

			/* The seed_entry with same type/usage are always
			 * arranged by index in order of 0~3.
			 */
			if (entry->index != index) {
				pr_warn("Index mismatch.\n");
				goto fail;
			}

			if (entry->index > max_partition_num) {
				pr_warn("Index exceed max number!\n");
				goto fail;
			}

			memcpy(&rpmb_seed[index], entry->seed, RPMB_KEY_LENGTH);
			index++;

			/* erase original seed in seed entry */
			memset(entry->seed, 0U, RPMB_KEY_LENGTH);
		}

		entry = (struct seed_entry *)((u8 *)entry +
						entry->seed_entry_size);
	}

	return 0;

fail:
	return -EFAULT;
}

int rpmb_key_sbl_get(ulong params_addr, size_t max_partition_num,
		     u8 rpmb_key[][RPMB_KEY_LENGTH])
{
	struct image_boot_params *boot_params = NULL;
	struct seed_list_hob *seed_list = NULL;
	u32 remap_buffer_size = 0;

	if (!params_addr || !max_partition_num) {
		pr_err("Invalid input params!\n");
		goto fail;
	}

	boot_params = memremap(params_addr, sizeof(*boot_params), MEMREMAP_WB);
	if (!boot_params) {
		pr_err("Remap params_addr failed!\n");
		goto fail;
	}

	seed_list = memremap(boot_params->p_seed_list,
			     sizeof(*seed_list), MEMREMAP_WB);
	if (!seed_list) {
		pr_err("Remap seed_list failed!\n");
		goto fail;
	}

	remap_buffer_size = seed_list->buffer_size;
	memunmap(seed_list);

	/* Remap with actual buffer size */
	seed_list = memremap(boot_params->p_seed_list,
			     remap_buffer_size, MEMREMAP_WB);

	return rpmb_key_sbl_parse_seed_list(seed_list, max_partition_num,
					    rpmb_key);

fail:
	if (seed_list)
		memunmap(seed_list);
	if (boot_params)
		memunmap(boot_params);
	return -EFAULT;
}
