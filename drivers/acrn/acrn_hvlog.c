/*
 * ACRN Hypervisor logmsg
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright (c) 2017 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Contact Information: Li Fei <fei1.li@intel.com>
 *
 * BSD LICENSE
 *
 * Copyright (C) 2017 Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Li Fei <fei1.li@intel.com>
 *
 */
#define pr_fmt(fmt) "ACRN HVLog: " fmt

#include <linux/memblock.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/major.h>
#include <linux/miscdevice.h>
#include <linux/vhm/vhm_hypercall.h>
#include <linux/vhm/acrn_hv_defs.h>

#include <asm/hypervisor.h>

#include "sbuf.h"

#define LOG_ENTRY_SIZE		80
#define DEFAULT_PCPU_NR		4

#define foreach_cpu(cpu, cpu_num)					\
	for ((cpu) = 0; (cpu) < (cpu_num); (cpu)++)

#define foreach_hvlog_type(idx, hvlog_type)				\
	for ((idx) = 0; (idx) < (hvlog_type); (idx)++)

enum sbuf_hvlog_index {
	SBUF_CUR_HVLOG = 0,
	SBUF_LAST_HVLOG,
	SBUF_HVLOG_TYPES
};

struct acrn_hvlog {
	struct miscdevice miscdev;
	char name[24];
	shared_buf_t *sbuf;
	atomic_t open_cnt;
	int pcpu_num;
};

static struct acrn_hvlog *acrn_hvlog_devs[SBUF_HVLOG_TYPES];
static uint16_t pcpu_nr = DEFAULT_PCPU_NR;
static unsigned long long hvlog_buf_size;
static unsigned long long hvlog_buf_base;

static int __init early_hvlog(char *p)
{
	int ret;

	pr_debug("%s(%s)\n", __func__, p);
	hvlog_buf_size = memparse(p, &p);
	if (*p != '@')
		return 0;
	hvlog_buf_base = memparse(p + 1, &p);

	if (!!hvlog_buf_base && !!hvlog_buf_size) {
		ret = memblock_reserve(hvlog_buf_base, hvlog_buf_size);
		if (ret) {
			pr_err("%s: Error reserving hvlog memblock\n",
				__func__);
			hvlog_buf_base = 0;
			hvlog_buf_size = 0;
			return ret;
		}
	}

	return 0;
}
early_param("hvlog", early_hvlog);


static inline shared_buf_t *hvlog_mark_unread(shared_buf_t *sbuf)
{
	/* sbuf must point to valid data.
	 * clear the lowest bit in the magic to indicate that the sbuf point
	 * to the last boot valid data. We will read all of valid data in the
	 * sbuf later from 0 offset to sbuf->tail.
	 */
	if (sbuf != NULL) {
		sbuf->magic &= ~1;
		sbuf->head = 0;
	}

	return sbuf;
}

static int acrn_hvlog_open(struct inode *inode, struct file *filp)
{
	struct acrn_hvlog *acrn_hvlog;

	acrn_hvlog = container_of(filp->private_data,
				struct acrn_hvlog, miscdev);
	pr_debug("%s, %s\n", __func__, acrn_hvlog->miscdev.name);

	if (acrn_hvlog->pcpu_num >= pcpu_nr) {
		pr_err("%s, invalid pcpu_num: %d\n",
				__func__, acrn_hvlog->pcpu_num);
		return -EIO;
	}

	/* More than one reader at the same time could get data messed up */
	if (atomic_cmpxchg(&acrn_hvlog->open_cnt, 0, 1) != 0)
		return -EBUSY;

	filp->private_data = acrn_hvlog;

	return 0;
}

static int acrn_hvlog_release(struct inode *inode, struct file *filp)
{
	struct acrn_hvlog *acrn_hvlog;

	acrn_hvlog = filp->private_data;

	pr_debug("%s, %s\n", __func__, acrn_hvlog->miscdev.name);

	if (acrn_hvlog->pcpu_num >= pcpu_nr) {
		pr_err("%s, invalid pcpu_num: %d\n",
				__func__, acrn_hvlog->pcpu_num);
		return -EIO;
	}

	atomic_dec(&acrn_hvlog->open_cnt);
	filp->private_data = NULL;

	return 0;
}

static ssize_t acrn_hvlog_read(struct file *filp, char __user *buf,
				size_t count, loff_t *offset)
{
	char data[LOG_ENTRY_SIZE];
	struct acrn_hvlog *acrn_hvlog;
	int ret;

	acrn_hvlog = (struct acrn_hvlog *)filp->private_data;

	pr_debug("%s, %s\n", __func__, acrn_hvlog->miscdev.name);

	if (acrn_hvlog->pcpu_num >= pcpu_nr) {
		pr_err("%s, invalid pcpu_num: %d\n",
				__func__, acrn_hvlog->pcpu_num);
		return -EIO;
	}

	if (acrn_hvlog->sbuf != NULL) {
		ret = sbuf_get(acrn_hvlog->sbuf, (uint8_t *)&data);
		if (ret > 0) {
			if (copy_to_user(buf, &data, ret))
				return -EFAULT;
		}

		return ret;
	}

	return 0;
}

static const struct file_operations acrn_hvlog_fops = {
	.owner  = THIS_MODULE,
	.open   = acrn_hvlog_open,
	.release = acrn_hvlog_release,
	.read = acrn_hvlog_read,
};

/**
 * base0 = hvlog_buf_base;
 * base1 = hvlog_buf_base + (hvlog_buf_size >> 1)
 * if there is valid data in base0, cur_logbuf = base1, last_logbuf = base0.
 * if there is valid data in base1, cur_logbuf = base0, last_logbuf = base1.
 * if there is no valid data both in base0 and base1, cur_logbuf = base0,
 * last_logbuf = 0.
 */
static void assign_hvlog_buf_base(uint64_t *cur_logbuf, uint64_t *last_logbuf)
{
	uint64_t base0, base1;
	uint32_t ele_num, size;
	uint16_t pcpu_id;

	base0 = hvlog_buf_base;
	base1 = hvlog_buf_base + (hvlog_buf_size >> 1);
	size = (hvlog_buf_size >> 1) / pcpu_nr;
	ele_num = (size - SBUF_HEAD_SIZE) / LOG_ENTRY_SIZE;

	foreach_cpu(pcpu_id, pcpu_nr) {
		if (sbuf_check_valid(ele_num, LOG_ENTRY_SIZE,
					base0 + (size * pcpu_id))) {
			*last_logbuf = base0;
			*cur_logbuf = base1;
			return;
		}
	}

	foreach_cpu(pcpu_id, pcpu_nr) {
		if (sbuf_check_valid(ele_num, LOG_ENTRY_SIZE,
					base1 + (size * pcpu_id))) {
			*last_logbuf = base1;
			*cur_logbuf = base0;
			return;
		}
	}

	/* No last logbuf found */
	*last_logbuf = 0;
	*cur_logbuf = base0;
}

static int init_hvlog_dev(uint64_t base, uint32_t hvlog_type)
{
	int err = 0;
	uint16_t idx, i;
	shared_buf_t *sbuf;
	struct acrn_hvlog *hvlog;
	uint32_t ele_size, ele_num, size;

	if (!base)
		return -ENODEV;

	size = (hvlog_buf_size >> 1) / pcpu_nr;
	ele_size = LOG_ENTRY_SIZE;
	ele_num = (size - SBUF_HEAD_SIZE) / ele_size;

	foreach_cpu(idx, pcpu_nr) {
		hvlog = &acrn_hvlog_devs[hvlog_type][idx];

		switch (hvlog_type) {
		case SBUF_CUR_HVLOG:
			snprintf(hvlog->name, sizeof(hvlog->name),
						"acrn_hvlog_cur_%hu", idx);
			sbuf = sbuf_construct(ele_num, ele_size,
						base + (size * idx));
			sbuf_share_setup(idx, ACRN_HVLOG, sbuf);
			break;
		case SBUF_LAST_HVLOG:
			snprintf(hvlog->name, sizeof(hvlog->name),
						"acrn_hvlog_last_%hu", idx);
			sbuf = sbuf_check_valid(ele_num, ele_size,
						base + (size * idx));
			hvlog_mark_unread(sbuf);
			break;
		default:
			return -EINVAL;
		}

		hvlog->miscdev.name = hvlog->name;
		hvlog->miscdev.minor = MISC_DYNAMIC_MINOR;
		hvlog->miscdev.fops = &acrn_hvlog_fops;
		hvlog->pcpu_num = idx;
		hvlog->sbuf = sbuf;

		err = misc_register(&(hvlog->miscdev));
		if (err < 0) {
			pr_err("Failed to register %s, errno %d\n",
							hvlog->name, err);
			goto err_reg;
		}
	}

	return 0;

err_reg:
	for (i = --idx; i >= 0; i--)
		misc_deregister(&acrn_hvlog_devs[hvlog_type][i].miscdev);

	return err;
}

static void deinit_hvlog_dev(uint32_t hvlog_type)
{
	uint16_t idx;
	struct acrn_hvlog *hvlog;

	foreach_cpu(idx, pcpu_nr) {
		hvlog = &acrn_hvlog_devs[hvlog_type][idx];
		switch (hvlog_type) {
		case SBUF_CUR_HVLOG:
			sbuf_share_setup(idx, ACRN_HVLOG, 0);
			sbuf_deconstruct(hvlog->sbuf);
			break;
		case SBUF_LAST_HVLOG:
			break;
		default:
			break;
		}

		misc_deregister(&(hvlog->miscdev));
	}

	kfree(acrn_hvlog_devs[hvlog_type]);
}

static int __init acrn_hvlog_init(void)
{
	int idx, ret = 0;
	struct acrn_hw_info hw_info;
	uint64_t cur_logbuf, last_logbuf;

	if (x86_hyper_type != X86_HYPER_ACRN) {
		pr_err("acrn_hvlog: not running under acrn hypervisor!\n");
		return -EINVAL;
	}

	if (!hvlog_buf_base || !hvlog_buf_size) {
		pr_warn("no fixed memory reserve for hvlog.\n");
		return 0;
	}

	memset(&hw_info, 0, sizeof(struct acrn_hw_info));
	ret = hcall_get_hw_info(virt_to_phys(&hw_info));
	if (!ret)
		pcpu_nr = hw_info.cpu_num;

	foreach_hvlog_type(idx, SBUF_HVLOG_TYPES) {
		acrn_hvlog_devs[idx] = kcalloc(pcpu_nr,
			sizeof(struct acrn_hvlog), GFP_KERNEL);
		if (!acrn_hvlog_devs[idx])
			return -ENOMEM;
	}

	assign_hvlog_buf_base(&cur_logbuf, &last_logbuf);
	ret = init_hvlog_dev(cur_logbuf, SBUF_CUR_HVLOG);
	if (ret) {
		pr_err("Failed to init cur hvlog devs, errno %d\n", ret);
		return ret;
	}

	/* If error happens for last hvlog devs setup, just print out an warn */
	ret = init_hvlog_dev(last_logbuf, SBUF_LAST_HVLOG);
	if (ret)
		pr_warn("Failed to init last hvlog devs, errno %d\n", ret);

	pr_info("Initialized hvlog module with %u cpu\n", pcpu_nr);
	return 0;
}

static void __exit acrn_hvlog_exit(void)
{
	int i;

	foreach_hvlog_type(i, SBUF_HVLOG_TYPES)
		deinit_hvlog_dev(i);

	pr_info("Exit hvlog module\n");
}

module_init(acrn_hvlog_init);
module_exit(acrn_hvlog_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Intel Corp., http://www.intel.com");
MODULE_DESCRIPTION("Driver for the Intel ACRN Hypervisor Logmsg");
MODULE_VERSION("0.1");
