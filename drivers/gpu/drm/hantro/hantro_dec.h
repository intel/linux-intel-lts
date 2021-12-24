/* SPDX-License-Identifier: GPL-2.0 */
/*
 *    Hantro decoder hardware driver header file.
 *
 *    Copyright (c) 2017 - 2020, VeriSilicon Inc.
 *    Copyright (c) 2020 - 2021, Intel Corporation
 */

#ifndef __HANTRO_DEC_H__
#define __HANTRO_DEC_H__

int hantrodec_release(struct file *file);
int hantrodec_init(void);
int hantrodec_cleanup(void);
int hantrodec_probe(struct dtbnode *pnode);
void hantrodec_remove(struct device_info *pdevinfo);
long hantrodec_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
int hantrodec_open(struct inode *inode, struct file *file);
u32 hantrodec_read_bandwidth(struct device_info *pdevinfo, int is_read_bw);

#endif /* __HANTRO_DEC_H__ */
