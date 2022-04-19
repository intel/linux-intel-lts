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

#include <asm/cacheflush.h>
#include <asm/intel-family.h>
#include <asm/nops.h>
#include <asm/perf_event.h>
#include <asm/div64.h>
#include <linux/acpi.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/smp.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include "tcc_buffer.h"

/*
 * This driver supports two versions of configuration tables.
 *
 * Some terminologies between these two versions are different.
 * a) In first version, the sram with cache locking is named as pseodu sram (PSRAM);
 *    In second version, the sram with cache locking is named as software sram (SSRAM);
 * b) In first version, configuration table is named as PTCT;
 *    In second version, configuration table is named as RTCT;
 *
 * This driver continues using the terminologies defined in first version.
 */

static int tccdbg;
module_param(tccdbg, int, 0644);
MODULE_PARM_DESC(tccdbg, "Turn on/off trace");
static int strict_affinity_check = 1;
module_param(strict_affinity_check, int, 0644);
MODULE_PARM_DESC(strict_affinity_check, "Check affinity of buffer request");
#define dprintk(fmt, arg...) \
	do { if (tccdbg) pr_info(fmt, ##arg); } while (0)

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

#define FORMAT_V1                 1
#define FORMAT_V2                 2

enum PTCT_ENTRY_TYPE {
	PTCT_PTCD_LIMITS = 1,
	PTCT_PTCM_BINARY,
	PTCT_WRC_L3_WAYMASK,
	PTCT_GT_L3_WAYMASK,
	PTCT_PESUDO_SRAM,
	PTCT_STREAM_DATAPATH,
	PTCT_TIMEAWARE_SUBSYSTEMS,
	PTCT_REALTIME_IOMMU,
	PTCT_MEMORY_HIERARCHY_LATENCY,
	PTCT_ENTRY_TYPE_NUMS
};

enum PTCT_V2_ENTRY_TYPE {
	PTCT_V2_COMPATIBILITY = 0,
	PTCT_V2_RTCD_LIMIT,
	PTCT_V2_CRL_BINARY,
	PTCT_V2_IA_WAYMASK,
	PTCT_V2_WRC_WAYMASK,
	PTCT_V2_GT_WAYMASK,
	PTCT_V2_SSRAM_WAYMASK,
	PTCT_V2_SSRAM,
	PTCT_V2_MEMORY_HIERARCHY_LATENCY,
	PTCT_V2_ERROR_LOG_ADDRESS,
	PTCT_V2_ENTRY_TYPE_NUMS
};

#define ENTRY_HEADER_SIZE (sizeof(struct tcc_ptct_entry_header) / sizeof(u32))
#define ACPI_HEADER_SIZE (sizeof(struct acpi_table_header) / sizeof(u32))

#define MEM_FREE     0
#define MEM_BUSY     1
#define MAXCACHEID   64
static u32 ssram_waymask_v2[2][MAXCACHEID] = {{0}};

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

struct tcc_ptct_crl {
	u32 crladdr_lo;
	u32 crladdr_hi;
	u32 crlsize;
};

struct tcc_ptct_psram_v2 {
	u32 cache_level;
	u32 cache_id;
	u32 phyaddr_lo;
	u32 phyaddr_hi;
	u32 size;
	u32 shared;
};

struct tcc_ptct_mhlatency_v2 {
	u32 cache_level;
	u32 latency;
	u32 *cacheids;
};

struct tcc_ptct_sram_waymask_v2 {
	u32 cache_level;
	u32 cache_id;
	u32 waymask;
};

struct tcc_ptct_compatibility {
	u32 rtct_version;
	u32 rtct_version_minor;
	u32 rtcd_version;
	u32 rtcd_version_minor;
};

struct tcc_ptct_errlog_v2 {
	u32 erraddr_lo;
	u32 erraddr_hi;
	u32 errsize;
};

struct tcc_ptct_crl_v2 {
	u32 crladdr_lo;
	u32 crladdr_hi;
	u32 crlsize;
};

struct memory_slot_info {
	u64 paddr;
	void *vaddr;
	size_t size;
	u32 status;
	u32 minor;
	u32 open_count;
	u32 psramid;
	cpumask_t cpumask;
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

/* White-listed registers */
static const struct tcc_registers_wl_s tcc_registers_wl_ehl[] = {
	{0xFEDA0000, 0x0A78},
};

static const struct tcc_registers_wl_s tcc_registers_wl_tglu[] = {
	{ 0xFEDA0000, 0x0A78},
	{ 0xFEDC0000, 0x6F08},
	{ 0xFEDC0000, 0x6F10},
	{ 0xFEDC0000, 0x6F00},
};

static const struct tcc_registers_wl_s tcc_registers_wl_tglh[] = {
	{ 0xFEDA0000, 0x0A78},
};

#define MAXDEVICENODE 250
static unsigned int tcc_buffer_device_major;
DECLARE_BITMAP(tcc_buffer_device_minor_avail, MAXDEVICENODE);
static struct class *tcc_buffer_class;
static struct acpi_table_header *acpi_ptct_tbl;
static struct tcc_config *p_tcc_config;
static u64 erraddr, crladdr;
static u32 errsize, crlsize;
static u32 tcc_init;
static u32 ptct_format = FORMAT_V1;
DEFINE_MUTEX(tccbuffer_mutex);
static int tcc_errlog_show(struct seq_file *m, void *v);
static int tcc_crlver_show(struct seq_file *m, void *v);
static void *cache_info_k_virt_addr;
struct cache_info_s {
	u64 phy_addr;
	u32 cache_level;
	u32 cache_size;
	u32 cacheline_size;
	u32 testcase;
	u64 l1_hits;
	u64 l1_miss;
	u64 l2_hits;
	u64 l2_miss;
	u64 l3_hits;
	u64 l3_miss;
};
static struct cache_info_s cache_info_k = {0,};
#define BUFSIZE  sizeof(struct cache_info_s)
static struct proc_dir_entry *ent;
static u64 hardware_prefetcher_disable_bits;
static u64 get_hardware_prefetcher_disable_bits(void);
static inline void tcc_perf_wrmsrl(u32 msr, u64 val);
static int start_measure(void);
static int tcc_perf_fn(void);

#define MSR_MISC_FEATURE_CONTROL    0x000001a4
#define READ_BYTE_SIZE              64
#define MISC_MSR_BITS_COMMON        ((0x52ULL << 16) | 0xd1)
#define MISC_MSR_BITS_GENERIC_L3    ((0x52ULL << 16) | 0x2e)
#define MISC_MSR_BITS_ALL_CLEAR     0x0
#define PERFMON_EVENTSEL_BITMASK    (~(0x40ULL << 16))

static u64 get_hardware_prefetcher_disable_bits(void)
{
	if (boot_cpu_data.x86_vendor != X86_VENDOR_INTEL || boot_cpu_data.x86 != 6)
		return 0;

	dprintk("x86_model 0x%02X\n", (u32)(boot_cpu_data.x86_model));
	switch (boot_cpu_data.x86_model) {
	case INTEL_FAM6_ATOM_GOLDMONT:
	case INTEL_FAM6_ATOM_GOLDMONT_PLUS:
	return 0x5;
	case INTEL_FAM6_ATOM_TREMONT:
	return 0x1F;
	case INTEL_FAM6_ALDERLAKE:
	case INTEL_FAM6_ALDERLAKE_N:
	case INTEL_FAM6_RAPTORLAKE:
	return 0x2F;
	default:
	return 0xF;
	}
}

static inline void tcc_perf_wrmsrl(u32 reg, u64 data)
{
	__wrmsr(reg, (u32)(data & 0xffffffffULL), (u32)(data >> 32));
}

static int tcc_perf_fn(void)
{
	u64 perf_l1h = 0, perf_l1m = 0, msr_bits_l1h = 0, msr_bits_l1m = 0;
	u64 perf_l2h = 0, perf_l2m = 0, msr_bits_l2h = 0, msr_bits_l2m = 0;
	u64 perf_l3h = 0, perf_l3m = 0, msr_bits_l3h = 0, msr_bits_l3m = 0;
	u64 i = 0, start = 0, end = 0, tsc_delta = 0, tsc_us = 0;
	u32 cacheline_len = 0, cacheread_size = 0;
	void *cachemem_k;

	pr_err("In %s\n", __func__);
	if (cache_info_k.cache_level == RGN_L2) {
		msr_bits_l2h = (MISC_MSR_BITS_COMMON) | (0x2  << 8);
		msr_bits_l2m = (MISC_MSR_BITS_COMMON) | (0x10 << 8);
		msr_bits_l1h = (MISC_MSR_BITS_COMMON) | (0x1  << 8);
		msr_bits_l1m = (MISC_MSR_BITS_COMMON) | (0x08 << 8);
	} else if (cache_info_k.cache_level == RGN_L3) {
		msr_bits_l2h = (MISC_MSR_BITS_COMMON) | (0x2  << 8);
		msr_bits_l2m = (MISC_MSR_BITS_COMMON) | (0x10 << 8);
		msr_bits_l3h = (MISC_MSR_BITS_COMMON) | (0x4  << 8);
		if (boot_cpu_data.x86_model == INTEL_FAM6_ALDERLAKE_N)
			msr_bits_l3m = (MISC_MSR_BITS_GENERIC_L3) | (0x41 << 8);
		else
			msr_bits_l3m = (MISC_MSR_BITS_COMMON) | (0x20 << 8);
	}
	asm volatile (" cli ");
	__wrmsr(MSR_MISC_FEATURE_CONTROL, hardware_prefetcher_disable_bits, 0x0);

	cachemem_k     = cache_info_k_virt_addr;
	cacheread_size = cache_info_k.cache_size;
	cacheline_len  = cache_info_k.cacheline_size;
	if ((cacheline_len == 0) || (cachemem_k == NULL))
		return -1;

	/* Disable events and reset counters. 4 pairs. */
	tcc_perf_wrmsrl(MSR_ARCH_PERFMON_EVENTSEL0, MISC_MSR_BITS_ALL_CLEAR);
	tcc_perf_wrmsrl(MSR_ARCH_PERFMON_EVENTSEL0 + 1, MISC_MSR_BITS_ALL_CLEAR);
	tcc_perf_wrmsrl(MSR_ARCH_PERFMON_EVENTSEL0 + 2, MISC_MSR_BITS_ALL_CLEAR);
	tcc_perf_wrmsrl(MSR_ARCH_PERFMON_EVENTSEL0 + 3, MISC_MSR_BITS_ALL_CLEAR);

	tcc_perf_wrmsrl(MSR_ARCH_PERFMON_PERFCTR0, MISC_MSR_BITS_ALL_CLEAR);
	tcc_perf_wrmsrl(MSR_ARCH_PERFMON_PERFCTR0 + 1, MISC_MSR_BITS_ALL_CLEAR);
	tcc_perf_wrmsrl(MSR_ARCH_PERFMON_PERFCTR0 + 2, MISC_MSR_BITS_ALL_CLEAR);
	tcc_perf_wrmsrl(MSR_ARCH_PERFMON_PERFCTR0 + 3, MISC_MSR_BITS_ALL_CLEAR);

	/* Set and enable L3 counters if msr_bits_l3h is prepared */
	if (msr_bits_l3h > 0) {
		tcc_perf_wrmsrl(MSR_ARCH_PERFMON_EVENTSEL0 + 2, msr_bits_l3h);
		tcc_perf_wrmsrl(MSR_ARCH_PERFMON_EVENTSEL0 + 3, msr_bits_l3m);
	}

	/* Set and enable L1 counters if msr_bits_l1h is prepared */
	if (msr_bits_l1h > 0) {
		tcc_perf_wrmsrl(MSR_ARCH_PERFMON_EVENTSEL0 + 2, msr_bits_l1h);
		tcc_perf_wrmsrl(MSR_ARCH_PERFMON_EVENTSEL0 + 3, msr_bits_l1m);
	}

	/* Set and enable the L2 counters, which is always preapred */
	tcc_perf_wrmsrl(MSR_ARCH_PERFMON_EVENTSEL0, msr_bits_l2h);
	tcc_perf_wrmsrl(MSR_ARCH_PERFMON_EVENTSEL0 + 1, msr_bits_l2m);

	/* capture the timestamp at the meantime while hitting buffer */
	start = rdtsc_ordered();
	for (i = 0; i < cacheread_size; i += cacheline_len) {
		/* Add a barrier to prevent reading beyond the end of the buffer */
		rmb();
		asm volatile("mov (%0,%1,1), %%eax\n\t"
				:
				: "r" (cachemem_k), "r" (i)
				: "%eax", "memory");
	}
	end = rdtsc_ordered();
	tcc_perf_wrmsrl(MSR_ARCH_PERFMON_EVENTSEL0, msr_bits_l2h & PERFMON_EVENTSEL_BITMASK);
	tcc_perf_wrmsrl(MSR_ARCH_PERFMON_EVENTSEL0 + 1, msr_bits_l2m & PERFMON_EVENTSEL_BITMASK);

	if (msr_bits_l3h > 0) {
		tcc_perf_wrmsrl(MSR_ARCH_PERFMON_EVENTSEL0 + 2, msr_bits_l3h & PERFMON_EVENTSEL_BITMASK);
		tcc_perf_wrmsrl(MSR_ARCH_PERFMON_EVENTSEL0 + 3, msr_bits_l3m & PERFMON_EVENTSEL_BITMASK);
	}

	if (msr_bits_l1h > 0) {
		tcc_perf_wrmsrl(MSR_ARCH_PERFMON_EVENTSEL0 + 2, msr_bits_l1h & PERFMON_EVENTSEL_BITMASK);
		tcc_perf_wrmsrl(MSR_ARCH_PERFMON_EVENTSEL0 + 3, msr_bits_l1m & PERFMON_EVENTSEL_BITMASK);
	}

	perf_l2h = native_read_pmc(0);
	perf_l2m = native_read_pmc(1);

	if (msr_bits_l3h > 0) {
		perf_l3h = native_read_pmc(2);
		perf_l3m = native_read_pmc(3);
	}

	if (msr_bits_l1h > 0) {
		perf_l1h = native_read_pmc(2);
		perf_l1m = native_read_pmc(3);
	}
	/* Add a barrier to ensure all previous instructions are retired before proceeding */
	rmb();
	wrmsr(MSR_MISC_FEATURE_CONTROL, 0x0, 0x0);
	asm volatile (" sti ");

	if (cache_info_k.cache_level == RGN_L2) {
		pr_err("PERFMARK perf_l2h=%-10llu perf_l2m=%-10llu", perf_l2h, perf_l2m);
		pr_err("PERFMARK perf_l1h=%-10llu perf_l1m=%-10llu", perf_l1h, perf_l1m);
		cache_info_k.l1_hits = perf_l1h;
		cache_info_k.l1_miss = perf_l1m;
		cache_info_k.l2_hits = perf_l2h;
		cache_info_k.l2_miss = perf_l2m;
		cache_info_k.l3_hits = 0;
		cache_info_k.l3_miss = 0;
	} else if (cache_info_k.cache_level == RGN_L3) {
		pr_err("PERFMARK perf_l2h=%-10llu perf_l2m=%-10llu", perf_l2h, perf_l2m);
		pr_err("PERFMARK perf_l3h=%-10llu perf_l3m=%-10llu", perf_l3h, perf_l3m);
		cache_info_k.l1_hits = 0;
		cache_info_k.l1_miss = 0;
		cache_info_k.l2_hits = perf_l2h;
		cache_info_k.l2_miss = perf_l2m;
		cache_info_k.l3_hits = perf_l3h;
		cache_info_k.l3_miss = perf_l3m;
	}
	tsc_delta = end-start;
	tsc_us = (end-start)*1000;
	pr_err("start: %lld\n", start);
	pr_err("end:   %lld\n", end);
	pr_err("delta: %lld\n", tsc_delta);
	pr_err("tsc:   %d kHz\n", tsc_khz);
	pr_err("With integer truncation:\n");
	do_div(tsc_delta, cacheread_size/cacheline_len);
	pr_err("Average each cacheline read takes: %lld tsc ticks\n", tsc_delta);
	do_div(tsc_us, tsc_khz);
	pr_err("Total cache read takes:      %lld us\n", tsc_us);

	return 0;
}

static int start_measure(void)
{
	int ret = -1;

	hardware_prefetcher_disable_bits = get_hardware_prefetcher_disable_bits();

	switch (cache_info_k.testcase) {
	case TEST_CACHE_PERF:
		ret = tcc_perf_fn();
	break;
	default:
		goto out;
	}

	if (ret < 0)
		goto out;
	ret = 0;
out:
	return ret;
}

static ssize_t set_test_setup(struct file *file, const char __user *ubuf, size_t count, loff_t *ppos)
{
	if (count > BUFSIZE)
		return -EFAULT;

	if (copy_from_user((char *)(&cache_info_k), ubuf, BUFSIZE))
		return -EFAULT;

	switch (cache_info_k.testcase) {
	case TEST_CACHE_PERF:
	{
		pr_err("cache_info_k.phy_addr        0x%016llx\n", cache_info_k.phy_addr);
		pr_err("cache_info_k.cache_level     %d\n", cache_info_k.cache_level);
		pr_err("cache_info_k.cache_size      0x%08x\n", cache_info_k.cache_size);
		pr_err("cache_info_k.cacheline_size  %d\n", cache_info_k.cacheline_size);
		pr_err("cache_info_k.testcase        %d\n", cache_info_k.testcase);

		cache_info_k_virt_addr = memremap(cache_info_k.phy_addr, cache_info_k.cache_size, MEMREMAP_WB);

		if (cache_info_k_virt_addr == NULL)
			pr_err("cache_info_k_virt_addr == NULL\n");
		else
			pr_err("cache_info_k_virt_addr       0x%px\n", cache_info_k_virt_addr);

		if (start_measure() != 0)
			pr_err("Something wrong with the cache performance measurement!");

		memunmap(cache_info_k_virt_addr);
		cache_info_k_virt_addr = NULL;
	}
	break;
	default:
		pr_err("This testcase is not handled yet.\n");
	return -EFAULT;
	}

	return BUFSIZE;
}

static ssize_t get_test_result(struct file *file, char __user *ubuf, size_t count, loff_t *ppos)
{
	if (count < BUFSIZE)
		return 0;

	switch (cache_info_k.testcase) {
	case TEST_CACHE_PERF:
		if (copy_to_user(ubuf, (char *)(&cache_info_k), BUFSIZE))
			return -EFAULT;
	break;
	default:
		pr_err("This testcase is not handled yet.\n");
	return -EFAULT;
	}

	return BUFSIZE;
}

#if KERNEL_VERSION(5, 6, 1) > LINUX_VERSION_CODE
static const struct file_operations testops = {
	.owner = THIS_MODULE,
	.read = get_test_result,
	.write = set_test_setup,
};
#else
static const struct proc_ops testops = {
	.proc_read = get_test_result,
	.proc_write = set_test_setup,
};
#endif

/****************************************************************************/

#define CPUID_LEAF_EXT_TOPO_ENUM 0x0B
#define CPUID_0B_SUBLEAF_SMT     0
#define CPUID_0B_SUBLEAF_CORE    1

static u32 utils_apicid(void)
{
	u32 eax = 0, ebx = 0, ecx = 0, edx = 0;

	cpuid_count(CPUID_LEAF_EXT_TOPO_ENUM, CPUID_0B_SUBLEAF_SMT, &eax, &ebx, &ecx, &edx);
	return edx;
}

static int curr_process_cpu(void)
{
	u32 apicid = 0;
	int i = 0, cpu = 0xFFFF;

	apicid = utils_apicid();
	for_each_online_cpu(i) {
		dprintk("cpu_data(%d).apicid = %08x. Check for %08x.\n", i, cpu_data(i).apicid, apicid);
		/* each cpu could corresponding to one apicid, even with HT */
		if (cpu_data(i).apicid == apicid)
			cpu = i;
	}
	if (cpu == 0xFFFF) {
		pr_err("Failed to find  cpu for apicid\n");
		return -EFAULT;
	}
	return cpu;
}

static int tcc_buffer_minor_get(u32 *minor)
{
	unsigned long first_bit;

	first_bit = find_first_bit(&tcc_buffer_device_minor_avail[0], MAXDEVICENODE);
	if (first_bit == MAXDEVICENODE)
		return -ENOSPC;
	__clear_bit(first_bit, &tcc_buffer_device_minor_avail[0]);
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

static void tcc_get_psram_cpumask(u32 coreid, u32 num_threads_sharing, cpumask_t *mask)
{
	u32 i = 0;
	u32 apicid_start = 0, apicid_end = 0;
	u32 index_msb, id;
	u32 apicid = 0;

	index_msb = get_count_order(num_threads_sharing);
	if (ptct_format == FORMAT_V2)
		apicid = coreid << index_msb;
	else
		apicid = coreid;
	dprintk("apicid is %d from cacheid %d\n", apicid, coreid);
	apicid_start = apicid & (~(num_threads_sharing - 1));
	apicid_end = apicid_start + num_threads_sharing;
	dprintk("apicid_start %d  apicid_end %d\n", apicid_start, apicid_end);
	for_each_online_cpu(i) {
		if ((cpu_data(i).apicid >= apicid_start) &&
			(cpu_data(i).apicid < apicid_end))
			cpumask_set_cpu(i, mask);
		id = cpu_data(i).apicid >> index_msb;
		dprintk("cpu_data(%d).apicid %2d\tnum_threads_sharing %d\tmsb %d\tcache_id %d\n", i, cpu_data(i).apicid, num_threads_sharing, index_msb, id);
	}
	dprintk("Cachel level dependent! apicid  %d  num_threads_sharing %d ==> cpumask %lx\n", apicid, num_threads_sharing, *(unsigned long *)mask);
}

static int tcc_errlog_show(struct seq_file *m, void *v)
{
	void *errlog_buff = NULL;
	u32 i = 0;
	int ret = 0;

	if (errsize > 0) {
		errlog_buff = memremap(erraddr, errsize, MEMREMAP_WB);
		if (!errlog_buff) {
			seq_puts(m, "System error. Fail to map this errlog kernel address.\n");
			ret = -ENOMEM;
		} else {
			seq_printf(m, "errlog_addr   @ %016llx\n", erraddr);
			seq_printf(m, "errlog_size   @ %08x\n", errsize);
			for (i = 0; i < errsize; i += sizeof(int))
				seq_printf(m, "%08x\n", ((u32 *)errlog_buff)[i/sizeof(int)]);
			memunmap(errlog_buff);
		}
	} else
		seq_puts(m, "No TCC Error Log Buffer.\n");

	return ret;
}

static int tcc_crlver_show(struct seq_file *m, void *v)
{
	void *crl_buff = NULL;
	int ret = 0;

	if (crlsize > 0) {
		crl_buff = memremap(crladdr, 8, MEMREMAP_WB);
		if (!crl_buff) {
			seq_puts(m, "System error. Fail to map this CRL address.\n");
			ret = -ENOMEM;
		} else {
			seq_printf(m, "%08x\n", ((u32 *)crl_buff)[0]);
			seq_printf(m, "%08x\n", ((u32 *)crl_buff)[1]);
			memunmap(crl_buff);
		}
	} else
		seq_puts(m, "No CRL.\n");

	return ret;
}

static int tcc_parse_ptct(void)
{
	u32 *tbl_swap;
	u32 offset = 0, entry_size = 0, entry_type = 0;
	u32 cache_level = 0, cache_id = 0;
	struct tcc_ptct_entry_header *entry_header;
	struct tcc_ptct_mhlatency *entry_mhl;
	struct tcc_ptct_psram *entry_psram;
	struct tcc_ptct_mhlatency_v2 *entry_mhl_v2;
	struct tcc_ptct_psram_v2 *entry_psram_v2;
	struct tcc_ptct_sram_waymask_v2 *entry_sram_waymask_v2;
	struct tcc_ptct_errlog_v2 *entry_errlog_v2;
	struct tcc_ptct_crl_v2 *entry_crl_v2;
	struct tcc_ptct_crl *entry_crl;
	static struct psram *p_new_psram;
	static struct memory_slot_info *p_memslot;
	struct psram *p_tmp_psram;
	struct tcc_ptct_compatibility *compatibility;
	u64 l2_start, l2_end, l3_start, l3_end;

	tbl_swap = (u32 *)acpi_ptct_tbl;

	tcc_get_cache_info();

	p_tcc_config->ptct_size = acpi_ptct_tbl->length;
	p_tcc_config->num_of_psram = 0;
	/* offset in INTEGER size*/
	offset = ACPI_HEADER_SIZE;
	tbl_swap = tbl_swap + offset;

	/* Check PTCT format */
	do {
		entry_header = (struct tcc_ptct_entry_header *)tbl_swap;

		entry_size = entry_header->size;
		entry_type = entry_header->type;

		if (entry_type == PTCT_V2_COMPATIBILITY) {
			compatibility = (struct tcc_ptct_compatibility *)(tbl_swap + ENTRY_HEADER_SIZE);
			ptct_format = compatibility->rtct_version;
			if ((ptct_format != FORMAT_V1) && (ptct_format != FORMAT_V2)) {
				pr_err("TCC PTCT cannot support this version %d.\n", ptct_format);
				return -EINVAL;
			}
		}

		offset += entry_size / sizeof(u32);
		tbl_swap = tbl_swap + entry_size / sizeof(u32);
	} while ((offset < (acpi_ptct_tbl->length) / sizeof(u32)) && entry_size);

	dprintk("ptct_format = %s\n", (ptct_format == FORMAT_V1) ? "FORMAT_V1":"FORMAT_V2");

	/* Parse and save memory latency and errer log buffer address and crl version */
	tbl_swap = (u32 *)acpi_ptct_tbl;
	offset = ACPI_HEADER_SIZE;
	tbl_swap = tbl_swap + offset;

	do {
		entry_header = (struct tcc_ptct_entry_header *)tbl_swap;

		entry_size = entry_header->size;
		entry_type = entry_header->type;

		if ((ptct_format == FORMAT_V1) && (entry_type == PTCT_MEMORY_HIERARCHY_LATENCY)) {
			entry_mhl = (struct tcc_ptct_mhlatency *)(tbl_swap + ENTRY_HEADER_SIZE);
			if (entry_mhl->cache_level == RGN_L2)
				p_tcc_config->l2_latency = entry_mhl->latency;
			else if (entry_mhl->cache_level == RGN_L3)
				p_tcc_config->l3_latency = entry_mhl->latency;
		} else if ((ptct_format == FORMAT_V2) && (entry_type == PTCT_V2_MEMORY_HIERARCHY_LATENCY)) {
			entry_mhl_v2 = (struct tcc_ptct_mhlatency_v2 *)(tbl_swap + ENTRY_HEADER_SIZE);
			if (entry_mhl_v2->cache_level == RGN_L2)
				p_tcc_config->l2_latency = entry_mhl_v2->latency;
			else if (entry_mhl_v2->cache_level == RGN_L3)
				p_tcc_config->l3_latency = entry_mhl_v2->latency;
		} else if ((ptct_format == FORMAT_V2) && (entry_type == PTCT_V2_ERROR_LOG_ADDRESS)) {
			entry_errlog_v2 = (struct tcc_ptct_errlog_v2 *)(tbl_swap + ENTRY_HEADER_SIZE);
			erraddr = ((u64)(entry_errlog_v2->erraddr_hi) << 32) | entry_errlog_v2->erraddr_lo;
			errsize = entry_errlog_v2->errsize;
		} else if ((ptct_format == FORMAT_V2) && (entry_type == PTCT_V2_CRL_BINARY)) {
			entry_crl_v2 = (struct tcc_ptct_crl_v2 *)(tbl_swap + ENTRY_HEADER_SIZE);
			crladdr = ((u64)(entry_crl_v2->crladdr_hi) << 32) | entry_crl_v2->crladdr_lo;
			crlsize = entry_crl_v2->crlsize;
		} else if ((ptct_format == FORMAT_V1) && (entry_type == PTCT_PTCM_BINARY)) {
			entry_crl = (struct tcc_ptct_crl *)(tbl_swap + ENTRY_HEADER_SIZE);
			crladdr = ((u64)(entry_crl->crladdr_hi) << 32) | entry_crl->crladdr_lo;
			crlsize = entry_crl->crlsize;
		}

		offset += entry_size / sizeof(u32);
		tbl_swap = tbl_swap + entry_size / sizeof(u32);
	} while ((offset < (acpi_ptct_tbl->length) / sizeof(u32)) && entry_size);

	/* Parse and save ssram waymask in version 2 format*/
	if (ptct_format == FORMAT_V2) {
		tbl_swap = (u32 *)acpi_ptct_tbl;
		offset = ACPI_HEADER_SIZE;
		tbl_swap = tbl_swap + offset;

		do {
			entry_header = (struct tcc_ptct_entry_header *)tbl_swap;
			entry_size = entry_header->size;
			entry_type = entry_header->type;

			if (entry_type == PTCT_V2_SSRAM_WAYMASK) {
				entry_sram_waymask_v2 = (struct tcc_ptct_sram_waymask_v2 *)(tbl_swap + ENTRY_HEADER_SIZE);
				cache_level = entry_sram_waymask_v2->cache_level;
				cache_id =  entry_sram_waymask_v2->cache_id;
				if (((cache_level == RGN_L2) || (cache_level == RGN_L3)) && (cache_id < MAXCACHEID)) {
					ssram_waymask_v2[cache_level - RGN_L2][cache_id] = entry_sram_waymask_v2->waymask;
					dprintk("ssram_waymask_v2[cache_level %d][cache_id %d]  = 0x%08x\n", cache_level, cache_id, ssram_waymask_v2[cache_level - RGN_L2][cache_id]);
				}
			}

			offset += entry_size / sizeof(u32);
			tbl_swap = tbl_swap + entry_size / sizeof(u32);
		} while ((offset < (acpi_ptct_tbl->length) / sizeof(u32)) && entry_size);
	}

	/* Parse and save ssram regions */
	tbl_swap = (u32 *)acpi_ptct_tbl;
	offset = ACPI_HEADER_SIZE;
	tbl_swap = tbl_swap + offset;

	do {
		entry_header = (struct tcc_ptct_entry_header *)tbl_swap;

		entry_size = entry_header->size;
		entry_type = entry_header->type;

		if (ptct_format == FORMAT_V1) {
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
					p_memslot->vaddr = p_new_psram->vaddr;
					p_memslot->size = p_new_psram->config.size;
					p_memslot->status = MEM_FREE;
					p_memslot->minor = UNDEFINED_DEVNODE;
					p_memslot->psramid = p_new_psram->config.id;
					p_memslot->cpumask = p_new_psram->cpumask;
					dprintk("%s\n", "RGN_L2");
					dprintk("p_new_psram->paddr @ %016llx\n", (u64)(p_new_psram->paddr));
					dprintk("p_memslot->paddr   @ %016llx\n", (u64)(p_memslot->paddr));
					dprintk("p_memslot->psramid @ %d\n", p_memslot->psramid);
					dprintk("p_memslot->cpumask @ %llx\n", *((u64 *)&(p_memslot->cpumask)));
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
		} else {
			/* RTCT version 2 format */
			switch (entry_type) {
			case PTCT_V2_SSRAM:
				dprintk("%s\n", "Find new ssram entry");
				entry_psram_v2 = (struct tcc_ptct_psram_v2 *)(tbl_swap + ENTRY_HEADER_SIZE);
				if (entry_psram_v2->cache_level != RGN_L2 && entry_psram_v2->cache_level != RGN_L3)
					break;
				dprintk("%s\n", "PTCT_V2_SSRAM");
				dprintk("entry_psram_v2->cache_level= %08x\n", entry_psram_v2->cache_level);
				dprintk("entry_psram_v2->cache_id   = %08x\n", entry_psram_v2->cache_id);
				dprintk("entry_psram_v2->phyaddr_lo = %08x\n", entry_psram_v2->phyaddr_lo);
				dprintk("entry_psram_v2->phyaddr_hi = %08x\n", entry_psram_v2->phyaddr_hi);
				dprintk("entry_psram_v2->size       = %08x\n", entry_psram_v2->size);

				p_new_psram = kzalloc(sizeof(struct psram), GFP_KERNEL);
				if (!p_new_psram)
					return -ENOMEM;

				p_new_psram->config.id = p_tcc_config->num_of_psram++;
				p_new_psram->config.type = entry_psram_v2->cache_level;
				p_new_psram->paddr = ((u64)(entry_psram_v2->phyaddr_hi) << 32) | entry_psram_v2->phyaddr_lo;
				p_new_psram->config.size = entry_psram_v2->size;
				if (((entry_psram_v2->cache_level == RGN_L2) || (entry_psram_v2->cache_level == RGN_L3)) && (entry_psram_v2->cache_id < MAXCACHEID))
					p_new_psram->config.ways = ssram_waymask_v2[entry_psram_v2->cache_level - RGN_L2][entry_psram_v2->cache_id];

				if (entry_psram_v2->cache_level == RGN_L2) {
					p_new_psram->config.latency = p_tcc_config->l2_latency;
					tcc_get_psram_cpumask(entry_psram_v2->cache_id, p_tcc_config->l2_num_of_threads_share, &p_new_psram->cpumask);
					p_new_psram->vaddr = memremap(p_new_psram->paddr, p_new_psram->config.size, MEMREMAP_WB);
					INIT_LIST_HEAD(&p_new_psram->memslots);

					p_memslot = kzalloc(sizeof(struct memory_slot_info), GFP_KERNEL);
					if (!p_memslot)
						return -ENOMEM;

					p_memslot->paddr = p_new_psram->paddr;
					p_memslot->vaddr = p_new_psram->vaddr;
					dprintk("%s\n", "RGN_L2");
					dprintk("p_new_psram->paddr @ %016llx\n", (u64)(p_new_psram->paddr));
					dprintk("p_memslot->paddr   @ %016llx\n", (u64)(p_memslot->paddr));
					p_memslot->size = p_new_psram->config.size;
					p_memslot->status = MEM_FREE;
					p_memslot->minor = UNDEFINED_DEVNODE;
					p_memslot->psramid = p_new_psram->config.id;
					p_memslot->cpumask = p_new_psram->cpumask;
					dprintk("p_memslot->size    @ %016llx\n", (u64)(p_memslot->size));
					dprintk("p_memslot->psramid @ %d\n", p_memslot->psramid);
					dprintk("p_memslot->cpumask @ %llx\n", *((u64 *)&(p_memslot->cpumask)));
					list_add_tail(&p_memslot->node, &p_new_psram->memslots);
				} else if (entry_psram_v2->cache_level == RGN_L3) {
					dprintk("%s\n", "RGN_L3 first stage");
					p_new_psram->config.latency = p_tcc_config->l3_latency;
					tcc_get_psram_cpumask(entry_psram_v2->cache_id, p_tcc_config->l3_num_of_threads_share, &p_new_psram->cpumask);
				}

				p_new_psram->config.cpu_mask_p = (void *)&p_new_psram->cpumask;
				list_add_tail(&p_new_psram->node, &p_tcc_config->psrams);
				break;

			default:
				break;
			}
		}
		/* move to next entry*/
		offset += entry_size / sizeof(u32);
		tbl_swap = tbl_swap + entry_size / sizeof(u32);
	} while ((offset < (acpi_ptct_tbl->length) / sizeof(u32)) && entry_size);
	dprintk("%s\n", "Process possible overlay l2/l3 region");
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
			dprintk("l2_start = 0x%016llx\n", l2_start);
			dprintk("l2_end   = 0x%016llx\n", l2_end);
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

			dprintk("l3_start = 0x%016llx\n", l3_start);
			dprintk("l3_end   = 0x%016llx\n", l3_end);

			p_tmp_psram->paddr = l3_start;
			p_tmp_psram->config.size = l3_end - l3_start;

			if (p_tmp_psram->config.size > 0) {
				p_tmp_psram->vaddr = memremap(p_tmp_psram->paddr, p_tmp_psram->config.size, MEMREMAP_WB);
				INIT_LIST_HEAD(&p_tmp_psram->memslots);

				p_memslot = kzalloc(sizeof(struct memory_slot_info), GFP_KERNEL);
				if (!p_memslot)
					return -ENOMEM;

				p_memslot->paddr = p_tmp_psram->paddr;
				p_memslot->vaddr = p_tmp_psram->vaddr;
				dprintk("%s\n", "RGN_L3 second stage");
				dprintk("p_tmp_psram->paddr @ %016llx\n", (u64)(p_tmp_psram->paddr));
				dprintk("p_memslot->paddr   @ %016llx\n", (u64)(p_memslot->paddr));
				p_memslot->size = p_tmp_psram->config.size;
				p_memslot->status = MEM_FREE;
				p_memslot->minor = UNDEFINED_DEVNODE;
				p_memslot->psramid = p_tmp_psram->config.id;
				p_memslot->cpumask = p_tmp_psram->cpumask;
				dprintk("p_memslot->size    @ %016llx\n", (u64)(p_memslot->size));
				dprintk("p_memslot->psramid @ %d\n", p_memslot->psramid);
				dprintk("p_memslot->cpumask @ %llx\n", *((u64 *)&(p_memslot->cpumask)));
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

	if (!found) {
		pr_err("No enough memory\n");
		goto fail;
	}

	ret = tcc_buffer_minor_get(&new_minor);
	if (ret < 0) {
		pr_err("Unable to obtain a new minor number\n");
		goto fail;
	}
	dprintk("%s\n", "tcc_allocate_memslot()");
	new_size = p_slot->size - size;

	if (new_size > 0) {
		p_memslot = kzalloc(sizeof(struct memory_slot_info), GFP_KERNEL);
		if (p_memslot == NULL)
			goto fail_create_device;
		p_memslot->paddr = p_slot->paddr + size;
		p_memslot->vaddr = p_slot->vaddr + size;
		p_memslot->size = new_size;
		p_memslot->status = MEM_FREE;
		p_memslot->psramid = p_slot->psramid;
		p_memslot->cpumask = p_slot->cpumask;
		dprintk("%s\n", "memslot for remained memory.");
		dprintk("p_memslot->paddr   @ %016llx\n", (u64)(p_memslot->paddr));
		dprintk("p_memslot->size    @ %016llx\n", (u64)(p_memslot->size));
		list_add(&p_memslot->node, &p_slot->node);
	}

	p_slot->size = size;
	p_slot->status = MEM_BUSY;
	p_slot->open_count = 0;

	dev_ret = device_create(tcc_buffer_class, NULL,
				MKDEV(tcc_buffer_device_major, new_minor),
				NULL, TCC_BUFFER_NAME "%d", new_minor);
	if (IS_ERR(dev_ret)) {
		ret = PTR_ERR(dev_ret);
		pr_err("SystemOS failed to create character device. Please reload driver.\n");
		goto fail_create_device;
	}

	p_slot->minor = new_minor;
	dprintk("%s\n", "memslot for allocated memory.");
	dprintk("p_slot->paddr    @ %016llx\n", (u64)(p_slot->paddr));
	dprintk("p_slot->size     @ %016llx\n", (u64)(p_slot->size));
	dprintk("p_slot->minor    @ %d\n", p_slot->minor);
	dprintk("p_slot->psramid  @ %d\n", p_slot->psramid);
	dprintk("p_slot->cpumask  @ %08llx\n", *((u64 *)&(p_slot->cpumask)));
	mutex_unlock(&tccbuffer_mutex);
	return new_minor;
fail_create_device:
	__set_bit(new_minor, &tcc_buffer_device_minor_avail[0]);
fail:
	mutex_unlock(&tccbuffer_mutex);
	return UNDEFINED_DEVNODE;
}

/*
 * To use this funcionint:
 *    smp_call_function_any(const struct cpumask *mask, smp_call_func_t func, void *info, int wait)
 */
struct mem_s {
	void *vaddr;
	size_t size;
};
static void clear_mem(void *info);

static void clear_mem(void *info)
{
	struct mem_s *mem = (struct mem_s *) info;

	memset(mem->vaddr, 0, mem->size);
}

static void tcc_free_memslot(struct memory_slot_info *p_memslot)
{
	struct memory_slot_info *pre_slot;
	struct memory_slot_info *next_slot;
	struct mem_s mem_info;
	struct psram *p_psram;
	u32 is_first = 0, is_last = 0;

	mutex_lock(&tccbuffer_mutex);
	device_destroy(tcc_buffer_class, MKDEV(tcc_buffer_device_major, p_memslot->minor));
	__set_bit(p_memslot->minor, &tcc_buffer_device_minor_avail[0]);
	p_memslot->status = MEM_FREE;
	p_memslot->minor = UNDEFINED_DEVNODE;
	dprintk("%s\n", "tcc_free_memslot()");
	dprintk("p_memslot->paddr    @ %016llx\n", (u64)(p_memslot->paddr));
	dprintk("p_memslot->vaddr    @ %016llx\n", (u64)(p_memslot->vaddr));
	dprintk("p_memslot->size     @ %016llx\n", (u64)(p_memslot->size));
	mem_info.vaddr = p_memslot->vaddr;
	mem_info.size  = p_memslot->size;
	smp_call_function_any(&(p_memslot->cpumask), clear_mem, &mem_info, 1);

	list_for_each_entry(p_psram, &p_tcc_config->psrams, node) {
		if (p_psram->config.size > 0) {
			if (list_is_first(&p_memslot->node, &p_psram->memslots))
				is_first = 1;
			if (list_is_last(&p_memslot->node, &p_psram->memslots))
				is_last = 1;
		}
	}

	if (!is_first)
		pre_slot = list_prev_entry(p_memslot, node);

	if (!is_last)
		next_slot = list_next_entry(p_memslot, node);

	if ((!is_first) && (pre_slot->status == MEM_FREE)) {
		dprintk("%s\n", "This is not FIRST slot, and pre_slot is FREE to merge");
		pre_slot->size += p_memslot->size;
		if ((!is_last) && (next_slot->status == MEM_FREE)) {
			dprintk("%s\n", "AND this is not LAST slot, and next_slot is also FREE to merge");
			pre_slot->size += next_slot->size;
			list_del(&next_slot->node);
			kfree(next_slot);
		}
		dprintk("paddr	             @ %016llx\n", (u64)(pre_slot->paddr));
		dprintk("size	             @ %016llx (extended)\n", (u64)(pre_slot->size));
		list_del(&p_memslot->node);
		kfree(p_memslot);
	} else if ((!is_last) && (next_slot->status == MEM_FREE)) {
		dprintk("%s\n", "This is not LAST slot, and next_slot is FREE to merge");
		p_memslot->size += next_slot->size;
		dprintk("paddr               @ %016llx\n", (u64)(p_memslot->paddr));
		dprintk("size	             @ %016llx (extended)\n", (u64)(p_memslot->size));
		list_del(&next_slot->node);
		kfree(next_slot);
	} else {
		dprintk("%s\n", "No other attributes need to set.");
	}

	mutex_unlock(&tccbuffer_mutex);
}

static int tcc_buffer_open(struct inode *i, struct file *f)
{
	struct memory_slot_info *p_memslot;
	int cpu, testmask = 0;

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
	cpu = curr_process_cpu();
	if (cpu < 0) {
		pr_err("OPEN(): No cpu found.\n");
		return -EINVAL;
	}
	testmask = cpumask_test_cpu(cpu, &(p_memslot->cpumask));
	if (testmask == 0) {
		pr_err("OPEN(): psram device is open from non-affinity cpu.\n");
		if (strict_affinity_check == 1)
			return -EINVAL;
	}

	p_memslot->open_count++;
	dprintk("%s\n", "open()");
	dprintk("p_memslot->paddr      @ %016llx\n", (u64)(p_memslot->paddr));
	dprintk("p_memslot->size       @ %016llx\n", (u64)(p_memslot->size));

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
	dprintk("%s\n", "close()");
	dprintk("p_memslot->paddr      @ %016llx\n", (u64)(p_memslot->paddr));
	dprintk("p_memslot->size       @ %016llx\n", (u64)(p_memslot->size));
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

	if (!(vma->vm_flags & VM_SHARED)) {
		pr_err("mmap() should specify VM_SHARED flag!");
		return -EINVAL;
	}

	if (p_memslot == NULL)
		return -EINVAL;

	dprintk("%s\n", "mmap()");
	dprintk("p_memslot->paddr      @ %016llx\n", (u64)(p_memslot->paddr));
	dprintk("p_memslot->size       @ %016llx\n", (u64)(p_memslot->size));

	pfn = (p_memslot->paddr) >> PAGE_SHIFT;
	ret = remap_pfn_range(vma, vma->vm_start, pfn, len, vma->vm_page_prot);

	if (ret < 0)
		return -EAGAIN;
	else
		return 0;
}

static int register_address_is_allowed(u64 base, u64 offset)
{
	/* check if phyaddr is within the whitelist range */
	u32 len = 0;

	if (offset >= TCC_REG_MAX_OFFSET)
		return 0;

	switch (boot_cpu_data.x86_model) {
	case INTEL_FAM6_TIGERLAKE:
		/* TGL-H */
		for (len = 0; len < ARRAY_SIZE(tcc_registers_wl_tglh); len++) {
			if ((tcc_registers_wl_tglh[len].base == base) && (tcc_registers_wl_tglh[len].offset == offset)) {
				dprintk("{base %016llx, offset %016llx} is in the tglh whitelist.\n", base, offset);
				return 1;
			}
		}
	break;
	case INTEL_FAM6_TIGERLAKE_L:
		/* TGL-U */
		for (len = 0; len < ARRAY_SIZE(tcc_registers_wl_tglu); len++) {
			if ((tcc_registers_wl_tglu[len].base == base) && (tcc_registers_wl_tglu[len].offset == offset)) {
				dprintk("{base %016llx, offset %016llx} is in the tglu whitelist.\n", base, offset);
				return 1;
			}
		}
	break;
	case INTEL_FAM6_ATOM_TREMONT:
		/* EHL */
		for (len = 0; len < ARRAY_SIZE(tcc_registers_wl_ehl); len++) {
			if ((tcc_registers_wl_ehl[len].base == base) && (tcc_registers_wl_ehl[len].offset == offset)) {
				dprintk("{base %016llx, offset %016llx} is in the ehl whitelist.\n", base, offset);
				return 1;
			}
		}
	break;
	default:
		pr_err("No whitelisted register.");
	break;
	}
	return 0;
}

static long tcc_buffer_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct psram *p_psram;
	struct tcc_buf_mem_config_s memconfig;
	struct tcc_buf_mem_req_s req_mem;
	struct tcc_register_s tcc_register;
	u64 register_phyaddr;
	void *register_data = NULL;
	void *errlog_buff = NULL;

	int cpu, testmask = 0;

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

		cpu = curr_process_cpu();
		if (cpu < 0) {
			pr_err("No cpu found.\n");
			return -EINVAL;
		}
		/* check the request psram region affinity with this process */
		list_for_each_entry(p_psram, &p_tcc_config->psrams, node) {
			if (p_psram->config.size > 0) {
				dprintk("p_psram %d  cpumask %08lx\n", p_psram->config.id, *((long *)&(p_psram->cpumask)));
				if (p_psram->config.id == req_mem.id)
					testmask = cpumask_test_cpu(cpu, &(p_psram->cpumask));
			}
		}
		if (testmask == 0) {
			pr_err("psram is requested from non-affinity cpu.\n");
			if (strict_affinity_check == 1)
				return -EFAULT;
		}

		if (req_mem.size & (PAGE_SIZE - 1)) {
			pr_err("size must be page-aligned!");
			return -EINVAL;
		}

		/* KW */
		if ((req_mem.size > 0) && (req_mem.size < 0xFFFFFFFF))
			req_mem.devnode = tcc_allocate_memslot(req_mem.id, req_mem.size);
		else {
			pr_err("size requested is either Zero or is too huge.\n");
			return -EINVAL;
		}

		if (req_mem.devnode == UNDEFINED_DEVNODE)
			return -ENOMEM;

		ret = copy_to_user((struct tcc_buf_mem_req_s *)arg, &req_mem, sizeof(req_mem));
		if (ret != 0)
			return -EFAULT;

		break;
	case TCC_GET_REGISTER:
		if (NULL == (struct tcc_register_s *)arg) {
			pr_err("arg from user is nullptr!");
			return -EINVAL;
		}

		ret = copy_from_user(&tcc_register, (struct tcc_register_s *)arg,
							 sizeof(struct tcc_register_s));
		if (ret != 0)
			return -EFAULT;

		if (tcc_register.e_format == TCC_MMIO32) {
			if ((tcc_register.info.mmio32.base <= (UINT_MAX - TCC_REG_MAX_OFFSET)) && (tcc_register.info.mmio32.addr <= TCC_REG_MAX_OFFSET)) {
				if (register_address_is_allowed((u64)(tcc_register.info.mmio32.base), (u64)(tcc_register.info.mmio32.addr)) == 0) {
					pr_err("this register is not allowed to read!");
					return -EINVAL;
				}
			} else {
				pr_err("Invalid mmio32 register base/addr provided.");
				return -EINVAL;
			}
			register_phyaddr = (u64)(tcc_register.info.mmio32.base + tcc_register.info.mmio32.addr);
			dprintk("register_phyaddr        0x%016llx\n", register_phyaddr);
			register_data = memremap(register_phyaddr, sizeof(int), MEMREMAP_WB);
			if (!register_data) {
				pr_err("cannot map this address");
				return -ENOMEM;
			}
			tcc_register.info.mmio32.data = (*(int *)(register_data)) & tcc_register.info.mmio32.mask;
			dprintk("mmio32.data   u32 value 0x%08x\n", tcc_register.info.mmio32.data);
			memunmap(register_data);
		} else if (tcc_register.e_format == TCC_MMIO64) {
			if ((tcc_register.info.mmio64.base <= (ULONG_MAX - TCC_REG_MAX_OFFSET)) && (tcc_register.info.mmio64.addr <= TCC_REG_MAX_OFFSET)) {
				if (register_address_is_allowed(tcc_register.info.mmio64.base, tcc_register.info.mmio64.addr) == 0) {
					pr_err("this register is not allowed to read!");
					return -EINVAL;
				}
			} else {
				pr_err("Invalid mmio64 register base/addr provided.");
				return -EINVAL;
			}
			register_phyaddr = (u64)(tcc_register.info.mmio64.base + tcc_register.info.mmio64.addr);
			dprintk("register_phyaddr        0x%016llx\n", register_phyaddr);
			register_data = memremap(register_phyaddr, sizeof(long), MEMREMAP_WB);
			if (!register_data) {
				pr_err("cannot map this address");
				return -ENOMEM;
			}
			tcc_register.info.mmio64.data = (*(long *)(register_data)) & tcc_register.info.mmio64.mask;
			dprintk("mmio64.data   u64 value 0x%016llx\n", tcc_register.info.mmio64.data);
			memunmap(register_data);
		} else {
			pr_err("Invalid or unsupported format!");
			return -EINVAL;
		}

		ret = copy_to_user((struct tcc_register_s *)arg, &tcc_register, sizeof(struct tcc_register_s));
		if (ret != 0)
			return -EFAULT;

		break;
	case TCC_GET_ERRLOG:
		if ((ptct_format == FORMAT_V1) || (errsize == 0)) {
			pr_err("ErrLog entry in this version is not valid!");
			return -EFAULT;
		}
		if (NULL == (u32 *)arg) {
			pr_err("arg from user is nullptr!");
			return -EINVAL;
		}
		errlog_buff = memremap(erraddr, errsize, MEMREMAP_WB);
		if (!errlog_buff) {
			pr_err("cannot map this errlog address");
			return -ENOMEM;
		}
		ret = copy_to_user((u32 *)arg, errlog_buff, errsize);
		memunmap(errlog_buff);
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
				if (p_slot->status != MEM_FREE) {
					device_destroy(tcc_buffer_class, MKDEV(tcc_buffer_device_major, p_slot->minor));
					__set_bit(p_slot->minor, &tcc_buffer_device_minor_avail[0]);
				}

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
		status = acpi_get_table(ACPI_SIG_RTCT, 0, &acpi_ptct_tbl);
		if (ACPI_FAILURE(status) || !acpi_ptct_tbl) {
			pr_err("Stop! ACPI doesn't provide PTCT/RTCT.");
			return -EFAULT;
		}
		dprintk("%s\n", "RTCT found.");
	}

	p_tcc_config = kzalloc(sizeof(struct tcc_config), GFP_KERNEL);
	if (!p_tcc_config)
		return -ENOMEM;
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

	bitmap_set(&tcc_buffer_device_minor_avail[0], 0, MAXDEVICENODE);

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

	ent = proc_create("tcc_cache_test", 0660, NULL, &testops);
	proc_create_single("tcc_errlog", 0, NULL, tcc_errlog_show);
	proc_create_single("tcc_crlver", 0, NULL, tcc_crlver_show);
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
		__set_bit(p_tcc_config->minor, &tcc_buffer_device_minor_avail[0]);
		class_destroy(tcc_buffer_class);
		unregister_chrdev(tcc_buffer_device_major, TCC_BUFFER_NAME);
		kfree(p_tcc_config);

		proc_remove(ent);
		remove_proc_entry("tcc_errlog", NULL);
		remove_proc_entry("tcc_crlver", NULL);
	}
	pr_err("exit().\n");
}

module_init(tcc_buffer_init);
module_exit(tcc_buffer_exit);

MODULE_LICENSE("Dual BSD/GPL");
