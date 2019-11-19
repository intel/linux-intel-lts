/*
 *    Hantro cache controller header file.
 *
 *    Copyright (c) 2017, VeriSilicon Inc.
 *
 *    This program is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU General Public License
 *    as published by the Free Software Foundation; either version 2
 *    of the License, or (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You may obtain a copy of the GNU General Public License
 *    Version 2 or later at the following locations:
 *    http://www.opensource.org/licenses/gpl-license.html
 *    http://www.gnu.org/copyleft/gpl.html
 */

#ifndef _CACHE_H_
#define _CACHE_H_
#include <linux/ioctl.h>
#include "hantro_priv.h"
#include "hantro.h"

#undef PDEBUG   /* undef it, just in case */
#ifdef CACHE_DEBUG
#ifdef __KERNEL__
/* This one if debugging is on, and kernel space */
#define PDEBUG printk
#else
/* This one for user space */
#define PDEBUG(fmt, args...) printf(__FILE__ ":%d: " fmt, __LINE__, ## args)
#endif
#else
#define PDEBUG(fmt, args...)  /* not debugging: nothing */
#endif

long hantrocache_ioctl(
	struct file *filp,
	unsigned int cmd,
	unsigned long arg);

int cache_init(void);
int cache_probe(struct platform_device *pdev, struct hantro_core_info *prc, int corenum);
void cache_cleanup(void);
int cache_open(struct inode *inode, struct file *filp);
int cache_release(void);

#endif /* !_CACHE_H_ */

