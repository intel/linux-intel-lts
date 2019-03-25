// SPDX-License-Identifier: GPL-2.0
// Intel Keystore Linux driver, Copyright (c) 2018, Intel Corporation.
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/slab.h>

#include "manifest_cache.h"

/* The linked list of cache entries. */
static struct list_head mf_cache = LIST_HEAD_INIT(mf_cache);

/**
 * Allocate a cache entry.
 *
 * @return Cache entry structure pointer or NULL if out of memory.
 */
static struct mf_cache_entry *mf_allocate_cache_entry(void)
{
	struct mf_cache_entry *item = NULL;
	struct list_head *pos, *q;
	unsigned int cnt = 0;

	/* calculate number of cache entries in use */
	list_for_each_safe(pos, q, &mf_cache) {
		item = list_entry(pos, struct mf_cache_entry, list);
		if (ktime_ms_delta(ktime_get(), item->expiry) >= 0) {
			list_del(pos);
			kzfree(item);
		} else
			cnt++;
	}

	/* check for maximum number of clients */
	if (cnt < MAX_CACHE_MANIFEST_ENTRIES) {
		/* allocate memory */
		item = kzalloc(sizeof(struct mf_cache_entry),
				GFP_KERNEL);
		if (item)
			list_add(&(item->list), &mf_cache);
	}

	return item;
}

/**
 * Find non-expired app in cache or remove the expired one.
 *
 * @param app_name Application name.
 *
 * @return remaining time if found or negative error code (see errno).
 */
int mf_find_in_cache(const char *app_name)
{
	struct list_head *pos, *q;
	struct mf_cache_entry *item;

	if (!app_name)
		return -EFAULT;

	list_for_each_safe(pos, q, &mf_cache) {
		item = list_entry(pos, struct mf_cache_entry, list);
		if (!strcmp(app_name, item->app_name)) {
			long long delta = ktime_ms_delta(ktime_get(),
					item->expiry);

			if (delta >= 0) {
				list_del(pos);
				kzfree(item);
				return -ESRCH;
			} else
				return (int) ((-delta + 999) / 1000);
		}
	}

	return -ESRCH;
}

/**
 * Add app to cache or update time to live if app already
 * exists in cache.
 *
 * @param app_name Application name.
 * @param time_to_live Time to live in cache (in seconds).
 *
 * @return 0 if OK or negative error code (see errno).
 */
int mf_add_to_cache(const char *app_name, uint64_t ttl)
{
	struct list_head *pos, *q;
	struct mf_cache_entry *item;

	if (!app_name)
		return -EFAULT;

	if (!*app_name || strlen(app_name) >= MAX_APP_NAME_SIZE)
		return -EINVAL;

	list_for_each_safe(pos, q, &mf_cache) {
		item = list_entry(pos, struct mf_cache_entry, list);
		if (!strcmp(app_name, item->app_name))
			goto found;
	}

	item = mf_allocate_cache_entry();
	if (!item)
		return -ENOMEM;

	strcpy(item->app_name, app_name);

found:
	item->expiry = ktime_add_ms(ktime_get(), ttl * 1000);
	return 0;
}

/**
 * Delete all cache entries.
 */
void mf_clear_cache(void)
{
	struct list_head *pos, *q;
	struct mf_cache_entry *item;

	list_for_each_safe(pos, q, &mf_cache) {
		item = list_entry(pos, struct mf_cache_entry, list);
		list_del(pos);
		kzfree(item);
	}
}

/**
 * Dump cache contents
 */
void mf_dump_cache(void)
{
	struct list_head *pos;
	struct mf_cache_entry *item;
	long long delta;

	list_for_each(pos, &mf_cache) {
		item = list_entry(pos, struct mf_cache_entry, list);
		delta = ktime_ms_delta(ktime_get(), item->expiry);
		pr_info(KBUILD_MODNAME ": cache: app=%s ttl=%d sec\n",
				item->app_name, (int) ((-delta + 999) / 1000));
	}
}
/* end of file */
