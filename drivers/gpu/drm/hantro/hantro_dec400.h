/* SPDX-License-Identifier: GPL-2.0 */
/*
 *    Hantro dec400 controller hardware driver header file.
 *
 *    Copyright (c) 2017 - 2020, VeriSilicon Inc.
 *    Copyright (c) 2020 - 2021, Intel Corporation
 */

#ifndef __HANTRO_DEC400_H__
#define __HANTRO_DEC400_H__

int hantrodec400_init(void);
int hantrodec400_cleanup(void);
int hantro_dec400_probe(dtbnode *pnode);
void hantrodec400_remove(struct device_info *pdevice);

long hantrodec400_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

#endif /* __HANTRO_DEC400_H__ */
