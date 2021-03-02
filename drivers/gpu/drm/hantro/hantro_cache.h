/* SPDX-License-Identifier: GPL-2.0 */
/*
 *    Hantro cache controller header file.
 *
 *    Copyright (c) 2017 - 2020, VeriSilicon Inc.
 *    Copyright (c) 2020 - 2021, Intel Corporation
 */

#ifndef __HANTRO_CACHE_H__
#define __HANTRO_CACHE_H__

#undef PDEBUG /* undef it, just in case */
#ifdef CACHE_DEBUG
#ifdef __KERNEL__
/* This one if debugging is on, and kernel space */
#define PDEBUG(fmt, arg...)                                                    \
	do {                                                                   \
		if (verbose)                                                   \
			pr_info(fmt, ##arg);                                   \
	} while (0)

#else
/* This one for user space */
#define PDEBUG(fmt, args...) printf(__FILE__ ":%d: " fmt, __LINE__, ##args)
#endif
#else
#define PDEBUG(fmt, args...) /* not debugging: nothing */
#endif

long hantrocache_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
int hantrocache_init(void);
int hantrocache_cleanup(void);
int cache_probe(dtbnode *pnode);
void hantrocache_remove(struct device_info *pdevice);

int cache_open(struct inode *inode, struct file *filp);
int cache_release(struct file *filp);

#endif /* __HANTRO_CACHE_H__ */
