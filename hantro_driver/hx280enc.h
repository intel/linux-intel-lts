/* SPDX-License-Identifier: GPL-2.0 */
/*
 *    Hantro encoder hardware driver header file.
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

#ifndef __HX280ENC_H__
#define __HX280ENC_H__

#include "hantro_priv.h"
#include "hantro.h"
/*
 * Macros to help debugging
 */

#undef PDEBUG /* undef it, just in case */
#ifdef HX280ENC_DEBUG
#ifdef __KERNEL__
/* This one if debugging is on, and kernel space */
#define PDEBUG(fmt, args...) pr_info("hmp4e: " fmt, ##args)
#else
/* This one for user space */
#define PDEBUG(fmt, args...) printf(__FILE__ ":%d: " fmt, __LINE__, ##args)
#endif
#else
#define PDEBUG(fmt, args...) /* not debugging: nothing */
#endif

#define ENC_HW_ID1		0x48320100
#define ENC_HW_ID2		0x80006000
#define CORE_INFO_AMOUNT_OFFSET	28

long hantroenc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
int hantroenc_init(void);
int hantroenc_probe(dtbnode *pnode);
void hantroenc_cleanup(void);
u32 *hantroenc_getRegAddr(u32 coreid, u32 regid);
u32 hantroenc_readbandwidth(int sliceidx, int isreadBW);
int hantroenc_release(void);

#endif /* __HX280ENC_H__ */
