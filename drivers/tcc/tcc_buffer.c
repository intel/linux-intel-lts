// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 *  This file is provided under a dual BSD/GPLv2 license.  When using or
 *  redistributing this file, you may do so under either license.
 *
 *  GPL LICENSE SUMMARY
 *
 *  Time Coordinated Compute (TCC)
 *  Pseudo SRAM interface support on top of Cache Allocation Technology
 *  Copyright (C) 2020 Intel Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of version 2 of the GNU General Public License as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  BSD LICENSE
 *
 *  Time Coordinated Compute (TCC)
 *  Pseudo SRAM interface support on top of Cache Allocation Technology
 *  Copyright (C) 2020 Intel Corporation
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in
 *      the documentation and/or other materials provided with the
 *      distribution.
 *
 *    * Neither the name of Intel Corporation nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define pr_fmt(fmt) "TCC Buffer: " fmt

#include <linux/acpi.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/version.h>

#include "tcc_buffer.h"

#define PTCT_ENTRY_OFFSET_VERSION 0
#define PTCT_ENTRY_OFFSET_SIZE    0
#define PTCT_ENTRY_OFFSET_TYPE    1

#define PSRAM_OFFSET_CACHELEVEL   (PTCT_ENTRY_OFFSET_TYPE + 1)
#define PSRAM_OFFSET_PADDR_LO     (PSRAM_OFFSET_CACHELEVEL + 1)
#define PSRAM_OFFSET_PADDR_HI     (PSRAM_OFFSET_PADDR_LO + 1)
#define PSRAM_OFFSET_WAY          (PSRAM_OFFSET_PADDR_HI + 1)
#define PSRAM_OFFSET_SIZE         (PSRAM_OFFSET_WAY + 1)
#define PSRAM_OFFSET_APIC         (PSRAM_OFFSET_SIZE + 1)

#define MHL_OFFSET_HIERARCHY      (PTCT_ENTRY_OFFSET_TYPE + 1)
#define MHL_OFFSET_CLOCKCYCLES    (MHL_OFFSET_HIERARCHY + 1)
#define MHL_OFFSET_APIC           (MHL_OFFSET_CLOCKCYCLES + 1)

enum PTCT_ENTRY_TYPE {
	PTCT_PTCD_LIMITS          = 1,
	PTCT_PTCM_BINARY          = 2,
	PTCT_WRC_L3_WAYMASK       = 3,
	PTCT_GT_L3_WAYMASK        = 4,
	PTCT_PESUDO_SRAM          = 5,
	PTCT_STREAM_DATAPATH      = 6,
	PTCT_TIMEAWARE_SUBSYSTEMS = 7,
	PTCT_REALTIME_IOMMU       = 8,
	PTCT_MEMORY_HIERARCHY_LATENCY = 9,
	PTCT_ENTRY_TYPE_NUMS
};

#define ENTRY_HEADER_SIZE (sizeof(struct tcc_ptct_entry_header) / sizeof(u32))
#define ACPI_HEADER_SIZE (sizeof(struct acpi_table_header) / sizeof(u32))

#define MEM_FREE 0
#define MEM_BUSY 1

struct tcc_ptct_entry_header {
	u16 size;
	u16 format;
	u32 type;
};

struct tcc_ptct_psram {
	u32 cache_level;
	u32 phyaddr_lo;
	u32 phyaddr_hi;
	u32 cache_ways;
	u32 size;
	u32 apic_id;
};

struct tcc_ptct_mhlatency {
	u32 cache_level;
	u32 latency;
	u32 *apicids;
};

struct memory_slot_info {
	u64 paddr;
	size_t size;
	u32 status;
	u32 minor;
	u32 open_count;
	struct list_head node;
};

struct psram {
	struct tcc_buf_mem_config_s config;
	u64 paddr;
	void *vaddr;
	cpumask_t cpumask;
	struct list_head memslots;
	struct list_head node;
};

struct tcc_config {
	u32 l2_latency;
	u32 l2_num_of_threads_share;
	u32 l3_latency;
	u32 l3_num_of_threads_share;
	u32 ptct_size;
	u32 num_of_psram;
	u32 minor;
	struct list_head psrams;
};

static unsigned int tcc_buffer_device_major;
static unsigned long tcc_buffer_device_minor_avail = GENMASK(MINORBITS, 0);
static struct class *tcc_buffer_class;
static struct acpi_table_header *acpi_ptct_tbl;
static struct tcc_config *p_tcc_config;
static u32 tcc_init;
DEFINE_MUTEX(tccbuffer_mutex);

static int tcc_buffer_minor_get(u32 *minor)
{
	unsigned long first_bit;

	first_bit = find_first_bit(&tcc_buffer_device_minor_avail, MINORBITS);
	if (first_bit == MINORBITS)
		return -ENOSPC;
	__clear_bit(first_bit, &tcc_buffer_device_minor_avail);
	*minor = first_bit;
	return 0;
}

static struct memory_slot_info *tcc_get_memslot(u32 minor)
{
	struct psram *p_psram;
	struct memory_slot_info *p_slot;

	list_for_each_entry(p_psram, &p_tcc_config->psrams, node) {
		if (p_psram->config.size > 0) {
			list_for_each_entry(p_slot, &p_psram->memslots, node) {
				if (p_slot->minor == minor)
					return p_slot;
			}
		}
	}
	return NULL;
}

static void tcc_get_cache_info(void)
{
	u32 eax, edx, ebx, ecx;
	int i, lvl;

	i = 0;
	do {
		cpuid_count(0x00000004, i, &eax, &ebx, &ecx, &edx);

		lvl = ((eax & 0xE0) >> 5);
		if ((lvl == 2) || (lvl == 3)) {
			if (lvl == 2)
				p_tcc_config->l2_num_of_threads_share = ((eax >> 14) & 0xFFF) + 1;
			else
				p_tcc_config->l3_num_of_threads_share = ((eax >> 14) & 0xFFF) + 1;
		}

		i++;
	} while ((eax & 0x1F) != 0);
}

static void tcc_get_psram_cpumask(u32 apicid, u32 num_threads_sharing, cpumask_t *mask)
{
	u32 i = 0;
	u32 apicid_start = 0, apicid_end = 0;

	apicid_start = apicid & (~(num_threads_sharing - 1));
	apicid_end = apicid_start + num_threads_sharing;

	for_each_online_cpu(i) {
		if ((cpu_data(i).apicid >= apicid_start) &&
			(cpu_data(i).apicid < apicid_end))
			cpumask_set_cpu(i, mask);
	}
}

static int tcc_parse_ptct(void)
{
	u32 *tbl_swap;
	u32 offset = 0, entry_size, entry_type;
	struct tcc_ptct_entry_header *entry_header;
	struct tcc_ptct_mhlatency *entry_mhl;
	struct tcc_ptct_psram *entry_psram;
	static struct psram *p_new_psram;
	static struct memory_slot_info *p_memslot;
	struct psram *p_tmp_psram;
	u64 l2_start, l2_end, l3_start, l3_end;

	tbl_swap = (u32 *)acpi_ptct_tbl;

	tcc_get_cache_info();

	p_tcc_config->ptct_size = acpi_ptct_tbl->length;
	p_tcc_config->num_of_psram = 0;
	/* offset in INTEGER size*/
	offset = ACPI_HEADER_SIZE;
	tbl_swap = tbl_swap + offset;

	do {
		entry_header = (struct tcc_ptct_entry_header *)tbl_swap;

		entry_size = entry_header->size;
		entry_type = entry_header->type;

		if (entry_type == PTCT_MEMORY_HIERARCHY_LATENCY) {
			entry_mhl = (struct tcc_ptct_mhlatency *)(tbl_swap + ENTRY_HEADER_SIZE);
			if (entry_mhl->cache_level == RGN_L2)
				p_tcc_config->l2_latency = entry_mhl->latency;
			else if (entry_mhl->cache_level == RGN_L3)
				p_tcc_config->l3_latency = entry_mhl->latency;
		}

		offset += entry_size / sizeof(u32);
		tbl_swap = tbl_swap + entry_size / sizeof(u32);
	} while ((offset < (acpi_ptct_tbl->length) / sizeof(u32)) && entry_size);

	tbl_swap = (u32 *)acpi_ptct_tbl;
	offset = ACPI_HEADER_SIZE;
	tbl_swap = tbl_swap + offset;

	do {
		entry_header = (struct tcc_ptct_entry_header *)tbl_swap;

		entry_size = entry_header->size;
		entry_type = entry_header->type;

		switch (entry_type) {
		case PTCT_PESUDO_SRAM:
			entry_psram = (struct tcc_ptct_psram *)(tbl_swap + ENTRY_HEADER_SIZE);
			if (entry_psram->cache_level != RGN_L2 && entry_psram->cache_level != RGN_L3)
				break;

			p_new_psram = kzalloc(sizeof(struct psram), GFP_KERNEL);
			if (!p_new_psram)
				return -ENOMEM;

			p_new_psram->config.id = p_tcc_config->num_of_psram++;
			p_new_psram->config.type = entry_psram->cache_level;
			p_new_psram->paddr = ((u64)(entry_psram->phyaddr_hi) << 32) | entry_psram->phyaddr_lo;
			p_new_psram->config.size = entry_psram->size;
			p_new_psram->config.ways = entry_psram->cache_ways;

			if (entry_psram->cache_level == RGN_L2) {
				p_new_psram->config.latency = p_tcc_config->l2_latency;
				tcc_get_psram_cpumask(entry_psram->apic_id, p_tcc_config->l2_num_of_threads_share, &p_new_psram->cpumask);
				p_new_psram->vaddr = memremap(p_new_psram->paddr, p_new_psram->config.size, MEMREMAP_WB);
				INIT_LIST_HEAD(&p_new_psram->memslots);

				p_memslot = kzalloc(sizeof(struct memory_slot_info), GFP_KERNEL);
				if (!p_memslot)
					return -ENOMEM;

				p_memslot->paddr = p_new_psram->paddr;
				p_memslot->size = p_new_psram->config.size;
				p_memslot->status = MEM_FREE;
				p_memslot->minor = UNDEFINED_DEVNODE;
				list_add_tail(&p_memslot->node, &p_new_psram->memslots);
			} else if (entry_psram->cache_level == RGN_L3) {
				p_new_psram->config.latency = p_tcc_config->l3_latency;
				tcc_get_psram_cpumask(entry_psram->apic_id, p_tcc_config->l3_num_of_threads_share, &p_new_psram->cpumask);
			}

			p_new_psram->config.cpu_mask_p = (void *)&p_new_psram->cpumask;
			list_add_tail(&p_new_psram->node, &p_tcc_config->psrams);
			break;

		default:
			break;
		}
		/* move to next entry*/
		offset += entry_size / sizeof(u32);
		tbl_swap = tbl_swap + entry_size / sizeof(u32);
	} while ((offset < (acpi_ptct_tbl->length) / sizeof(u32)) && entry_size);

	l2_start = 0;
	l2_end = 0;

	list_for_each_entry(p_tmp_psram, &p_tcc_config->psrams, node) {
		if (p_tmp_psram->config.type == RGN_L2) {
			if (l2_start == 0 && l2_end == 0) {
				l2_start = p_tmp_psram->paddr;
				l2_end = p_tmp_psram->paddr + p_tmp_psram->config.size;
			} else {
				if (p_tmp_psram->paddr < l2_start)
					l2_start = p_tmp_psram->paddr;

				if (p_tmp_psram->paddr + p_tmp_psram->config.size > l2_end)
					l2_end = p_tmp_psram->paddr + p_tmp_psram->config.size;
			}
		}
	}

	list_for_each_entry(p_tmp_psram, &p_tcc_config->psrams, node) {
		if (p_tmp_psram->config.type == RGN_L3) {
			l3_start = p_tmp_psram->paddr;
			l3_end = p_tmp_psram->paddr + p_tmp_psram->config.size;

			if (l2_start <= l3_start) {
				if (l2_end < l3_end) {
					if (l2_end > l3_start)
						l3_start = l2_end;
				} else
					l3_start = l3_end;
			} else if (l2_start <= l3_end)
				l3_end = l2_start;

			p_tmp_psram->paddr = l3_start;
			p_tmp_psram->config.size = l3_end - l3_start;

			if (p_tmp_psram->config.size > 0) {
				p_tmp_psram->vaddr = memremap(p_tmp_psram->paddr, p_tmp_psram->config.size, MEMREMAP_WB);
				INIT_LIST_HEAD(&p_tmp_psram->memslots);

				p_memslot = kzalloc(sizeof(struct memory_slot_info), GFP_KERNEL);
				if (!p_memslot)
					return -ENOMEM;

				p_memslot->paddr = p_tmp_psram->paddr;
				p_memslot->size = p_tmp_psram->config.size;
				p_memslot->status = MEM_FREE;
				p_memslot->minor = UNDEFINED_DEVNODE;
				list_add_tail(&p_memslot->node, &p_tmp_psram->memslots);
			}
		}
	}

	return 0;
}

static u32 tcc_allocate_memslot(u32 id, size_t size)
{
	int ret;
	struct device *dev_ret;
	u32 new_minor = UNDEFINED_DEVNODE;
	struct psram *p_psram;
	static struct memory_slot_info *p_memslot;
	struct memory_slot_info *p_slot;
	u32 found = 0;
	u32 new_size = 0;

	mutex_lock(&tccbuffer_mutex);

	list_for_each_entry(p_psram, &p_tcc_config->psrams, node) {
		if ((p_psram->config.id == id) && (p_psram->config.size > 0)) {
			list_for_each_entry(p_slot, &p_psram->memslots, node) {
				if (p_slot->status == MEM_FREE && p_slot->size >= size) {
					found = 1;
					break;
				}
			}
		}
	}

	if (!found)
		goto fail;

	new_size = p_slot->size - size;

	if (new_size > 0) {
		p_memslot = kzalloc(sizeof(struct memory_slot_info), GFP_KERNEL);
		if (p_memslot == NULL)
			goto fail;
		p_memslot->paddr = p_slot->paddr + size;
		p_memslot->size = new_size;
		p_memslot->status = MEM_FREE;
		list_add(&p_memslot->node, &p_slot->node);
	}

	p_slot->size = size;
	p_slot->status = MEM_BUSY;
	p_slot->open_count = 0;

	ret = tcc_buffer_minor_get(&new_minor);
	if (ret < 0) {
		pr_err("Unable to obtain a new minor number\n");
		goto fail;
	}

	dev_ret = device_create(tcc_buffer_class, NULL,
				MKDEV(tcc_buffer_device_major, new_minor),
				NULL, TCC_BUFFER_NAME "%d", new_minor);
	if (IS_ERR(dev_ret)) {
		ret = PTR_ERR(dev_ret);
		pr_err("Failed to create character device\n");
		goto fail_create_device;
	}

	p_slot->minor = new_minor;

	mutex_unlock(&tccbuffer_mutex);
	return new_minor;
fail_create_device:
	__set_bit(new_minor, &tcc_buffer_device_minor_avail);
fail:
	mutex_unlock(&tccbuffer_mutex);
	return UNDEFINED_DEVNODE;
}

static void tcc_free_memslot(struct memory_slot_info *p_memslot)
{
	struct memory_slot_info *pre_slot;
	struct memory_slot_info *next_slot;
	void *vaddr;

	mutex_lock(&tccbuffer_mutex);
	device_destroy(tcc_buffer_class, MKDEV(tcc_buffer_device_major, p_memslot->minor));
	__set_bit(p_memslot->minor, &tcc_buffer_device_minor_avail);
	p_memslot->status = MEM_FREE;
	p_memslot->minor = UNDEFINED_DEVNODE;
	vaddr = memremap(p_memslot->paddr, p_memslot->size, MEMREMAP_WB);
	if (vaddr != NULL) {
		memset(vaddr, 0, p_memslot->size);
		memunmap(vaddr);
	}
	pre_slot = list_prev_entry(p_memslot, node);
	next_slot = list_next_entry(p_memslot, node);

	if (pre_slot->status == MEM_FREE) {
		pre_slot->size += p_memslot->size;
		if (next_slot->status == MEM_FREE) {
			pre_slot->size += next_slot->size;
			list_del(&next_slot->node);
			kfree(next_slot);
		}
		list_del(&p_memslot->node);
		kfree(p_memslot);
	}
	mutex_unlock(&tccbuffer_mutex);
}

static int tcc_buffer_open(struct inode *i, struct file *f)
{
	struct memory_slot_info *p_memslot;

	if (p_tcc_config->minor == MINOR(i->i_rdev))
		return 0;
	p_memslot = tcc_get_memslot(MINOR(i->i_rdev));
	if (p_memslot == NULL) {
		pr_err("OPEN(): No device node %u.\n", MINOR(i->i_rdev));
		return -ECHRNG;
	}
	if (p_memslot->open_count > 0) {
		pr_err("OPEN(): This device is already open.\n");
		return -EBUSY;
	}
	p_memslot->open_count++;
	f->private_data = p_memslot;
	return 0;
}

static int tcc_buffer_close(struct inode *i, struct file *f)
{
	struct memory_slot_info *p_memslot;

	if (p_tcc_config->minor == MINOR(i->i_rdev))
		return 0;
	p_memslot = (struct memory_slot_info *)(f->private_data);
	p_memslot->open_count--;
	if (p_memslot->open_count == 0)
		tcc_free_memslot(p_memslot);

	return 0;
}

static int tcc_buffer_mmap(struct file *f, struct vm_area_struct *vma)
{
	struct memory_slot_info *p_memslot = (struct memory_slot_info *)(f->private_data);
	u64 len = vma->vm_end - vma->vm_start;
	u64 pfn = 0;
	int ret = 0;

	if (len & (PAGE_SIZE - 1)) {
		pr_err("length must be page-aligned!");
		return -EINVAL;
	}

	if (!(vma->vm_flags & VM_SHARED))
		return -EINVAL;

	pfn = (p_memslot->paddr) >> PAGE_SHIFT;
	ret = remap_pfn_range(vma, vma->vm_start, pfn, len, vma->vm_page_prot);

	if (ret < 0)
		return -EAGAIN;
	else
		return 0;
}

static long tcc_buffer_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct psram *p_psram;
	struct tcc_buf_mem_config_s memconfig;
	struct tcc_buf_mem_req_s req_mem;

	switch (cmd) {
	case TCC_GET_REGION_COUNT:
		if (NULL == (int *)arg) {
			pr_err("arg from user is nullptr!");
			return -EINVAL;
		}
		ret = copy_to_user((int *)arg, &p_tcc_config->num_of_psram, sizeof(p_tcc_config->num_of_psram));
		if (ret != 0)
			return -EFAULT;

		break;
	case TCC_GET_MEMORY_CONFIG:
		if (NULL == (struct tcc_buf_mem_config_s *)arg) {
			pr_err("arg from user is nullptr!");
			return -EINVAL;
		}

		ret = copy_from_user(&memconfig, (struct tcc_buf_mem_config_s *)arg,
							 sizeof(struct tcc_buf_mem_config_s));
		if (ret != 0)
			return -EFAULT;

		if (memconfig.cpu_mask_p == NULL) {
			pr_err("cpu_mask_p from user is nullptr");
			return -EINVAL;
		}

		list_for_each_entry(p_psram, &p_tcc_config->psrams, node) {
			if (p_psram->config.id == memconfig.id) {
				ret = copy_to_user((struct tcc_buf_mem_config_s *)arg, &(p_psram->config),
					sizeof(struct tcc_buf_mem_config_s) - sizeof(void *));
				if (ret != 0)
					return -EFAULT;

				ret = copy_to_user((struct cpumask *)memconfig.cpu_mask_p,
					(struct cpumask *)(p_psram->config.cpu_mask_p), sizeof(struct cpumask));
				if (ret != 0)
					return -EFAULT;

				return 0;
			}
		}

		ret = -EFAULT;
		break;
	case TCC_QUERY_PTCT_SIZE:
		if (NULL == (u32 *)arg) {
			pr_err("arg from user is nullptr!");
			return -EINVAL;
		}
		ret = copy_to_user((u32 *)arg, &p_tcc_config->ptct_size, sizeof(p_tcc_config->ptct_size));
		if (ret != 0)
			return -EFAULT;

		break;
	case TCC_GET_PTCT:
		if (NULL == (u32 *)arg) {
			pr_err("arg from user is nullptr!");
			return -EINVAL;
		}
		ret = copy_to_user((u32 *)arg, acpi_ptct_tbl, p_tcc_config->ptct_size);
		if (ret != 0)
			return -EFAULT;

		break;
	case TCC_REQ_BUFFER:
		/* Given id, size from user library, return devnode */

		if (NULL == (struct tcc_buf_mem_req_s *)arg) {
			pr_err("arg from user is nullptr!");
			return -EINVAL;
		}
		ret = copy_from_user(&req_mem, (struct tcc_buf_mem_req_s *)arg, sizeof(req_mem));
		if (ret != 0)
			return -EFAULT;

		req_mem.devnode = tcc_allocate_memslot(req_mem.id, req_mem.size);

		if (req_mem.devnode == UNDEFINED_DEVNODE)
			return -ENOMEM;

		ret = copy_to_user((struct tcc_buf_mem_req_s *)arg, &req_mem, sizeof(req_mem));
		if (ret != 0)
			return -EFAULT;

		break;
	default:
		return -ENOIOCTLCMD;
	}

	return ret;
}

static const struct file_operations tcc_buffer_fops = {
	.owner = THIS_MODULE,
	.mmap = tcc_buffer_mmap,
	.unlocked_ioctl = tcc_buffer_ioctl,
	.open = tcc_buffer_open,
	.release = tcc_buffer_close,
};

static void tcc_cleanup(void)
{
	struct psram *p_psram, *p_temp_psram;
	struct memory_slot_info *p_slot, *p_temp_slot;

	list_for_each_entry_safe(p_psram, p_temp_psram, &p_tcc_config->psrams, node) {
		if (p_psram->config.size > 0) {
			list_for_each_entry_safe(p_slot, p_temp_slot, &p_psram->memslots, node) {
				if (p_slot->status != MEM_FREE)
					device_destroy(tcc_buffer_class, MKDEV(tcc_buffer_device_major, p_slot->minor));

				list_del(&p_slot->node);
				kfree(p_slot);
			}
			if ((p_psram->vaddr) && (p_psram->config.size > 0))
				memunmap(p_psram->vaddr);
		}
		list_del(&p_psram->node);
		kfree(p_psram);
	}
}

static int __init tcc_buffer_init(void)
{
	int ret;
	struct device *dev_ret;
	u32 new_minor = UNDEFINED_DEVNODE;
	acpi_status status = AE_OK;

	status = acpi_get_table(ACPI_SIG_PTCT, 0, &acpi_ptct_tbl);
	if (ACPI_FAILURE(status) || !acpi_ptct_tbl) {
		pr_err("Stop! ACPI doesn't provide PTCT.");
		return -1;
	}

	p_tcc_config = kzalloc(sizeof(struct tcc_config), GFP_KERNEL);
	if (!p_tcc_config)
		return -1;
	INIT_LIST_HEAD(&p_tcc_config->psrams);

	ret = tcc_parse_ptct();
	if (ret != 0)
		goto err_exit;

	ret = register_chrdev(0, TCC_BUFFER_NAME, &tcc_buffer_fops);
	if (ret < 0) {
		pr_err("Couldn't regiter_chrdev, returen error=%d\n", ret);
		goto err_exit;
	}
	tcc_buffer_device_major = ret;

	ret = tcc_buffer_minor_get(&new_minor);
	if (ret < 0) {
		pr_err("Unable to obtain a new minor number\n");
		goto err_class;
	}

	tcc_buffer_class = class_create(THIS_MODULE, TCC_BUFFER_NAME);
	if (IS_ERR(tcc_buffer_class)) {
		ret = PTR_ERR(tcc_buffer_class);
		pr_err("Create class failed!\n");
		goto err_class;
	}

	dev_ret = device_create(tcc_buffer_class, NULL,
				MKDEV(tcc_buffer_device_major, new_minor), NULL,
				TCC_BUFFER_NAME);

	if (IS_ERR(dev_ret)) {
		ret = PTR_ERR(dev_ret);
		pr_err("Failed to create character device\n");
		goto err_device;
	}
	tcc_init = 1;
	p_tcc_config->minor = new_minor;

	pr_err("TCC buffer init successfully\n");
	return 0;

err_device:
	class_destroy(tcc_buffer_class);
err_class:
	unregister_chrdev(tcc_buffer_device_major, TCC_BUFFER_NAME);
err_exit:
	tcc_cleanup();
	kfree(p_tcc_config);
	return ret;
}

static void __exit tcc_buffer_exit(void)
{
	if (tcc_init) {
		tcc_cleanup();
		device_destroy(tcc_buffer_class, MKDEV(tcc_buffer_device_major, p_tcc_config->minor));
		class_destroy(tcc_buffer_class);
		unregister_chrdev(tcc_buffer_device_major, TCC_BUFFER_NAME);
		kfree(p_tcc_config);
	}
}

module_init(tcc_buffer_init);
module_exit(tcc_buffer_exit);

MODULE_LICENSE("Dual BSD/GPL");
