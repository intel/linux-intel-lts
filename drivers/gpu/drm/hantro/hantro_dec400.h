/* SPDX-License-Identifier: GPL-2.0 */
/*
 *    Hantro dec400 controller hardware driver header file.
 *
 *    Copyright (c) 2017 - 2020, VeriSilicon Inc.
 *    Copyright (c) 2020 - 2021, Intel Corporation
 */

#ifndef __HANTRO_DEC400_H__
#define __HANTRO_DEC400_H__

int hantrodec400_probe(struct dtbnode *pnode);
void hantrodec400_remove(struct device_info *pdevinfo);
long hantrodec400_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

#endif /* __HANTRO_DEC400_H__ */
