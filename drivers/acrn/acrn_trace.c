/*
*
* ACRN Trace module
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
* Contact Information: Yan, Like <like.yan@intel.com>
*
* BSD LICENSE
*
* Copyright (c) 2017 Intel Corporation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*   * Neither the name of Intel Corporation nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
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
* Like Yan <like.yan@intel.com>
*
*/

#define pr_fmt(fmt) "ACRNTrace: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/major.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/vhm/vhm_hypercall.h>
#include <linux/vhm/acrn_hv_defs.h>

#include <asm/hypervisor.h>

#include "sbuf.h"


#define TRACE_SBUF_SIZE		(4 * 1024 * 1024)
#define TRACE_ELEMENT_SIZE	32 /* byte */
#define TRACE_ELEMENT_NUM	((TRACE_SBUF_SIZE - SBUF_HEAD_SIZE) /	\
				TRACE_ELEMENT_SIZE)

#define foreach_cpu(cpu, cpu_num)					\
	for ((cpu) = 0; (cpu) < (cpu_num); (cpu)++)

#define DEFAULT_NR_CPUS	4
/* actual physical cpu number, initialized by module init */
static int pcpu_num = DEFAULT_NR_CPUS;

struct acrn_trace {
	struct miscdevice miscdev;
	char name[24];
	shared_buf_t *sbuf;
	atomic_t open_cnt;
	uint16_t pcpu_id;
};

static struct acrn_trace *acrn_trace_devs;

/************************************************************************
 *
 * file_operations functions
 *
 ***********************************************************************/
static int acrn_trace_open(struct inode *inode, struct file *filep)
{
	struct acrn_trace *dev;

	dev = container_of(filep->private_data, struct acrn_trace, miscdev);
	if (!dev) {
		pr_err("No such dev\n");
		return -ENODEV;
	}
	pr_debug("%s, cpu %d\n", __func__, dev->pcpu_id);

	/* More than one reader at the same time could get data messed up */
	if (atomic_read(&dev->open_cnt))
		return -EBUSY;

	atomic_inc(&dev->open_cnt);

	return 0;
}

static int acrn_trace_release(struct inode *inode, struct file *filep)
{
	struct acrn_trace *dev;

	dev = container_of(filep->private_data, struct acrn_trace, miscdev);
	if (!dev) {
		pr_err("No such dev\n");
		return -ENODEV;
	}

	pr_debug("%s, cpu %d\n", __func__, dev->pcpu_id);

	atomic_dec(&dev->open_cnt);

	return 0;
}

static int acrn_trace_mmap(struct file *filep, struct vm_area_struct *vma)
{
	phys_addr_t paddr;
	struct acrn_trace *dev;

	dev = container_of(filep->private_data, struct acrn_trace, miscdev);
	if (!dev) {
		pr_err("No such dev\n");
		return -ENODEV;
	}

	pr_debug("%s, cpu %d\n", __func__, dev->pcpu_id);

	WARN_ON(!virt_addr_valid(dev->sbuf));
	paddr = virt_to_phys(dev->sbuf);

	if (remap_pfn_range(vma, vma->vm_start,
				paddr >> PAGE_SHIFT,
				vma->vm_end - vma->vm_start,
				vma->vm_page_prot)) {
		pr_err("Failed to mmap sbuf for cpu%d\n", dev->pcpu_id);
		return -EAGAIN;
	}

	return 0;
}

static const struct file_operations acrn_trace_fops = {
	.owner  = THIS_MODULE,
	.open   = acrn_trace_open,
	.release = acrn_trace_release,
	.mmap   = acrn_trace_mmap,
};

/*
 * acrn_trace_init()
 */
static int __init acrn_trace_init(void)
{
	int ret = 0;
	int i, cpu;
	shared_buf_t *sbuf;
	struct miscdevice *miscdev;
	struct acrn_hw_info hw_info;

	if (x86_hyper_type != X86_HYPER_ACRN) {
		pr_err("acrn_trace: not support acrn hypervisor!\n");
		return -EINVAL;
	}

	memset(&hw_info, 0, sizeof(struct acrn_hw_info));
	ret = hcall_get_hw_info(virt_to_phys(&hw_info));
	if (!ret)
		pcpu_num = hw_info.cpu_num;

	acrn_trace_devs = kcalloc(pcpu_num, sizeof(struct acrn_trace),
				GFP_KERNEL);
	if (!acrn_trace_devs)
		return -ENOMEM;

	foreach_cpu(cpu, pcpu_num) {
		/* allocate shared_buf */
		sbuf = sbuf_allocate(TRACE_ELEMENT_NUM, TRACE_ELEMENT_SIZE);
		if (!sbuf) {
			ret = -ENOMEM;
			goto out_free;
		}
		acrn_trace_devs[cpu].sbuf = sbuf;
	}

	foreach_cpu(cpu, pcpu_num) {
		sbuf = acrn_trace_devs[cpu].sbuf;
		ret = sbuf_share_setup(cpu, ACRN_TRACE, sbuf);
		if (ret < 0) {
			pr_err("Failed to setup SBuf, cpuid %d\n", cpu);
			goto out_sbuf;
		}
	}

	foreach_cpu(cpu, pcpu_num) {
		acrn_trace_devs[cpu].pcpu_id = cpu;

		miscdev = &acrn_trace_devs[cpu].miscdev;
		snprintf(acrn_trace_devs[cpu].name,
				sizeof(acrn_trace_devs[cpu].name),
				"acrn_trace_%d", cpu);
		miscdev->name = acrn_trace_devs[cpu].name;
		miscdev->minor = MISC_DYNAMIC_MINOR;
		miscdev->fops = &acrn_trace_fops;

		ret = misc_register(&acrn_trace_devs[cpu].miscdev);
		if (ret < 0) {
			pr_err("Failed to register acrn_trace_%d, errno %d\n",
				cpu, ret);
			goto out_dereg;
		}
	}

	pr_info("Initialized acrn trace module with %u cpu\n", pcpu_num);
	return ret;

out_dereg:
	for (i = --cpu; i >= 0; i--)
		misc_deregister(&acrn_trace_devs[i].miscdev);
	cpu = pcpu_num;

out_sbuf:
	for (i = --cpu; i >= 0; i--)
		sbuf_share_setup(i, ACRN_TRACE, NULL);
	cpu = pcpu_num;

out_free:
	for (i = --cpu; i >= 0; i--)
		sbuf_free(acrn_trace_devs[i].sbuf);
	kfree(acrn_trace_devs);

	return ret;
}

/*
 * acrn_trace_exit()
 */
static void __exit acrn_trace_exit(void)
{
	int cpu;

	pr_info("%s, cpu_num %d\n", __func__, pcpu_num);

	foreach_cpu(cpu, pcpu_num) {
		/* deregister devices */
		misc_deregister(&acrn_trace_devs[cpu].miscdev);

		/* set sbuf pointer to NULL in HV */
		sbuf_share_setup(cpu, ACRN_TRACE, NULL);

		/* free sbuf, per-cpu sbuf should be set NULL */
		sbuf_free(acrn_trace_devs[cpu].sbuf);
	}

	kfree(acrn_trace_devs);
}

module_init(acrn_trace_init);
module_exit(acrn_trace_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Intel Corp., http://www.intel.com");
MODULE_DESCRIPTION("Driver for the Intel ACRN Hypervisor Trace");
MODULE_VERSION("0.1");
