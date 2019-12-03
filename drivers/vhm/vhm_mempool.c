// SPDX-License-Identifier: GPL-2.0+ OR BSD-Clause
/*
 * virtio and hyperviosr service module (VHM): VHM mempool
 */

#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mempool.h>
#include <linux/printk.h>
#include <linux/slab.h>

#include <linux/vhm/acrn_vhm_mm.h>

static mempool_t *acrn_mempool;

int acrn_mempool_init(int min_nr, int buf_size)
{
	if (acrn_mempool)
		return 0;

	acrn_mempool = mempool_create_kmalloc_pool(min_nr, buf_size);

	if (acrn_mempool == NULL) {
		pr_err("Failed to initialize the memory poll\n");
		return -ENOMEM;
	}
	return 0;
}

void acrn_mempool_deinit(void)
{
	mempool_destroy(acrn_mempool);
	acrn_mempool = NULL;
}

void *acrn_mempool_alloc(gfp_t gfp_flag)
{
	if (acrn_mempool == NULL)
		return NULL;

	return mempool_alloc(acrn_mempool, gfp_flag);
}
EXPORT_SYMBOL_GPL(acrn_mempool_alloc);

void acrn_mempool_free(void *buf_ptr)
{
	if ((buf_ptr == NULL) || (acrn_mempool == NULL))
		return;

	mempool_free(buf_ptr, acrn_mempool);
}
EXPORT_SYMBOL_GPL(acrn_mempool_free);
