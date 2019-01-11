/* SPDX-License-Identifier: GPL-2.0 */
/* Intel Keystore Linux driver, Copyright (c) 2018, Intel Corporatio. */
#ifndef _MANIFEST_CACHE_H_
#define _MANIFEST_CACHE_H_

#include <linux/types.h>
#include <linux/list.h>
#include <linux/ktime.h>

#define MAX_APP_NAME_SIZE             255
#define MAX_CACHE_MANIFEST_ENTRIES    1000

/**
 * struct mf_cache_entry - The manifest cache entry structure.
 *
 * @list: Kernel list head for linked list.
 * @app_name: Application name.
 * @expiry: Expiration time.
 */
struct mf_cache_entry {
	struct list_head list;
	char app_name[MAX_APP_NAME_SIZE];
	ktime_t expiry;
};

/**
 * Find non-expired app in cache or remove the expired one.
 *
 * @param app_name Application name.
 *
 * @return remaining time if found or negative error code (see errno).
 */
int mf_find_in_cache(const char *app_name);

/**
 * Add app to cache or update time to live if app already
 * exists in cache.
 *
 * @param app_name Application name.
 * @param time_to_live Time to live in cache (in seconds).
 *
 * @return 0 if OK or negative error code (see errno).
 */
int mf_add_to_cache(const char *app_name, uint64_t ttl);

/**
 * Delete all cache entries.
 */
void mf_clear_cache(void);

/**
 * Dump cache contents
 */
void mf_dump_cache(void);

#endif /* _MANIFEST_CACHE_H_ */
