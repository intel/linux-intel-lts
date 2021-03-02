/* SPDX-License-Identifier: GPL-2.0 */
/*
 *    Hantro encoder hardware driver header file.
 *
 *    Copyright (c) 2017 - 2020, VeriSilicon Inc.
 *    Copyright (c) 2020 - 2021, Intel Corporation
 */

#ifndef __HX280ENC_H__
#define __HX280ENC_H__

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
int hantroenc_cleanup(void);
int hantroenc_probe(dtbnode *pnode);
void hantroenc_remove(struct device_info *pdevice);
u32 hantroenc_readbandwidth(struct device_info *pdevice, int is_read_bw);
int hantroenc_release(void);

#endif /* __HX280ENC_H__ */
