/* SPDX-License-Identifier: GPL-2.0 */
/*
 *    Hantro cache controller header file.
 *
 *    Copyright (c) 2017 - 2020, VeriSilicon Inc.
 *    Copyright (c) 2020 - 2021, Intel Corporation
 */

#ifndef __HANTRO_CACHE_H__
#define __HANTRO_CACHE_H__

#define L2CACHE_E_OFF 0x4
#define SHAPER_E_OFF 0x0
#define SHAPER_INT_OFF 0xC

long hantrocache_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
int hantrocache_probe(struct dtbnode *pnode);
void hantrocache_remove(struct device_info *pdevinfo);
int hantrocache_open(struct inode *inode, struct file *file);
void hantrocache_release(struct file *file);

#endif /* __HANTRO_CACHE_H__ */
