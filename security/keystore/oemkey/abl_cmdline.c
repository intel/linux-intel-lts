// SPDX-License-Identifier: GPL-2.0
// Intel Keystore Linux driver, Copyright (c) 2018, Intel Corporatio.
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/linkage.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/io.h>

#include "abl_cmdline.h"

static inline ssize_t size_inside_page(unsigned long start,
					     unsigned long size)
{
	unsigned long sz = PAGE_SIZE - (start & (PAGE_SIZE - 1));

	return min(sz, size);
}

int memcpy_from_ph(void *dest, phys_addr_t p, size_t count)
{
	if (!valid_phys_addr_range(p, count))
		return -EFAULT;

	while (count > 0) {
		ssize_t sz = size_inside_page(p, count);

		memcpy(dest, __va(p), sz);
		dest += sz;
		p += sz;
		count -= sz;
	}

	return 0;
}
EXPORT_SYMBOL(memcpy_from_ph);

int memset_ph(phys_addr_t p, int val, size_t count)
{
	if (!valid_phys_addr_range(p, count))
		return -EFAULT;

	while (count > 0) {
		ssize_t sz = size_inside_page(p, count);

		memset(__va(p), val, sz);
		p += sz;
		count -= sz;
	}

	return 0;
}
EXPORT_SYMBOL(memset_ph);

/* end of file */
