/*
 *    Hantro decoder hardware driver header file.
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

#ifndef _HANTRODEC_H_
#define _HANTRODEC_H_
#include <linux/ioctl.h>
#include <linux/types.h>
#include "hantro_priv.h"
#include "hantro.h"

#undef PDEBUG
#ifdef HANTRODEC_DEBUG
#  ifdef __KERNEL__
#    define PDEBUG(fmt, args...) pr_info("hantrodec: " fmt, ## args)
#  else
#    define PDEBUG(fmt, args...) fprintf(stderr, fmt, ## args)
#  endif
#else
#  define PDEBUG(fmt, args...)
#endif

int hantrodec_init(void);
int hantrodec_probe(struct platform_device *pdev, struct hantro_core_info *prc, int corenum);
void hantrodec_cleanup(void);
long hantrodec_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
u32 *hantrodec_getRegAddr(u32 coreid, u32 regid);
int hantrodec_open(struct inode *inode, struct file *filp);
u32 hantrodec_readbandwidth(int isreadBW);

#endif /* !_HANTRODEC_H_ */
