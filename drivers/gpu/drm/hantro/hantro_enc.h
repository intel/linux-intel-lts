/* SPDX-License-Identifier: GPL-2.0 */
/*
 *    Hantro encoder hardware driver header file.
 *
 *    Copyright (c) 2017 - 2020, VeriSilicon Inc.
 *    Copyright (c) 2020 - 2021, Intel Corporation
 */

#ifndef __HX280ENC_H__
#define __HX280ENC_H__

#define ENC_HW_ID1		0x48320100
#define ENC_HW_ID2		0x80006000
#define CORE_INFO_AMOUNT_OFFSET	28

long hantroenc_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
int hantroenc_init(void);
int hantroenc_cleanup(void);
int hantroenc_probe(struct dtbnode *pnode);
void hantroenc_remove(struct device_info *pdevinfo);
u32 hantroenc_read_bandwidth(struct device_info *pdevinfo, int is_read_bw);
int hantroenc_release(void);

#endif /* __HX280ENC_H__ */
