/* SPDX-License-Identifier: GPL-2.0 */
/*
 *    Hantro decoder hardware driver header file.
 *
 *    Copyright (c) 2017 - 2020, VeriSilicon Inc.
 *    Copyright (c) 2020 - 2021, Intel Corporation
 */

#ifndef __HANTRO_DEC_H__
#define __HANTRO_DEC_H__

#undef PDEBUG
#ifdef HANTRODEC_DEBUG
#ifdef __KERNEL__
#define PDEBUG(fmt, args...) pr_info("hantrodec: " fmt, ##args)
#else
#define PDEBUG(fmt, args...) fprintf(stderr, fmt, ##args)
#endif
#else
#define PDEBUG(fmt, args...)
#endif

int hantrodec_release(struct file *filp);
int hantrodec_init(void);
int hantrodec_cleanup(void);
int hantrodec_probe(dtbnode *pnode);
void hantrodec_remove(struct device_info *pdevice);
long hantrodec_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
int hantrodec_open(struct inode *inode, struct file *filp);
u32 hantrodec_readbandwidth(struct device_info *pdevice, int is_read_bw);

#endif /* __HANTRO_DEC_H__ */
