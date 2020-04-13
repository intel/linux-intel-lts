/*
 *    Hantro dec400 controller hardware driver header file.
 *
 *    Copyright (c) 2017, VeriSilicon Inc.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License, version 2, as
 *    published by the Free Software Foundation.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License version 2 for more details.
 *
 *    You may obtain a copy of the GNU General Public License
 *    Version 2 at the following locations:
 *    https://opensource.org/licenses/gpl-2.0.php
 */

#ifndef _HANTRO_DEC400_H_
#define _HANTRO_DEC400_H_

#include "hantro_priv.h"
#include "hantro.h"

int hantro_dec400_probe(dtbnode *pnode);
void hantro_dec400_cleanup(void);
long hantrodec400_ioctl(
	struct file *filp,
	unsigned int cmd,
	unsigned long arg);
int hantrodec400_init(void);

#endif	//_HANTRO_DEC400_H_


