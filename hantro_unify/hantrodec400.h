/* SPDX-License-Identifier: GPL-2.0 */
/*
 *    Hantro dec400 controller hardware driver header file.
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

#ifndef __HANTRO_DEC400_H__
#define __HANTRO_DEC400_H__

#include "hantro_priv.h"
#include "hantro.h"

int hantrodec400_init(void);
int hantro_dec400_probe(dtbnode *pnode);
void hantro_dec400_cleanup(void);
long hantrodec400_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

#endif /* __HANTRO_DEC400_H__ */
