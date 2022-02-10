/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */
/*
 *
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
#include <linux/types.h>
#include <linux/ioctl.h>

/* TCC Device Interface */
#define TCC_BUFFER_NAME  "/tcc/tcc_buffer"
#define UNDEFINED_DEVNODE 256
#define TCC_REG_MAX_OFFSET 0x100000
#define IOCTL_TCC_MAGIC   'T'
enum tcc_buf_region_type {
	RGN_UNKNOWN = 0,
	RGN_L1,
	RGN_L2,
	RGN_L3,
	RGN_EDRAM,
	RGN_MALLOC, /* DRAM */
	RGN_TOTAL_TYPES
};

/*
 * id: pseudo-SRAM region id from which user request for attribute.
 * latency: delay in clockcycles
 * type: the type of the memory pSRAM region
 * size: total size in byte
 * ways: the cache ways used to create the pSRAM region.
 * cpu_mask_p: affinity bitmask of the logical cores available for access to the pSRAM region
 */
struct tcc_buf_mem_config_s {
	unsigned int id;
	unsigned int latency;
	size_t size;
	enum tcc_buf_region_type type;
	unsigned int ways;
	void *cpu_mask_p;
};

/*
 * id: pseudo-SRAM region id, from which user request for buffer
 * size: buffer size (byte).
 * devnode: driver returns device node to user
 */
struct tcc_buf_mem_req_s {
	unsigned int id;
	size_t size;
	unsigned int devnode;
};

/* This structure defines the whitelisted register for mmio read */
struct tcc_registers_wl_s {
	u64 base;
	u64 offset;
};

/* enum type for tcc_register structure */
enum TCC_REG_PHASE {
	TCC_PRE_MEM = 0,
	TCC_POST_MEM,
	TCC_LATE_INIT,
	TCC_INVALID_PHASE  = 0xFFFFFFFF
};

enum TCC_REG_FORMAT {
	TCC_MMIO32 = 0,
	TCC_MMIO64,
	TCC_MSR,
	TCC_IOSFSB,
	TCC_MAILBOX,
	TCC_INVALID_FORMAT = 0xFFFFFFFF
};

enum TCC_IOSFSB_NETWORK {
	TCC_IOSFSB_CPU = 0,
	TCC_IOSFSB_PCH
};

enum TCC_MAILBOX_TYPE {
	TCC_MAILBOX_SA = 0,
	TCC_MAILBOX_MSR
};

/* Support mmio32 and mmio64 formats only. */
struct tcc_register_s {
	enum TCC_REG_PHASE e_phase;
	enum TCC_REG_FORMAT e_format;
	union {
		struct {
			u32 base;
			u32 addr;
			u32 mask;
			u32 data;
		} mmio32;
		struct {
			u64 base;
			u64 addr;
			u64 mask;
			u64 data;
		} mmio64;
		struct {
			u32 apic_id;
			u32 addr;
			u64 mask;
			u64 data;
		} msr;
		struct {
			enum TCC_IOSFSB_NETWORK e_iosfsb_network;
			u8 port;
			u8 type;
			u32 addr;
			u32 mask;
			u32 data;
		} iosfsb;
		struct {
			enum TCC_MAILBOX_TYPE e_type;
			u32 addr;
			u32 mask;
			u32 data;
		} mailbox;
	} info;
};

enum {
	TEST_CACHE_PERF = 1,
	TEST_CACHE_TOTAL_CASES
};

enum ioctl_index {
	IOCTL_TCC_GET_REGION_COUNT = 1,
	IOCTL_TCC_GET_MEMORY_CONFIG,
	IOCTL_TCC_REQ_BUFFER,
	IOCTL_TCC_QUERY_PTCT_SIZE,
	IOCTL_TCC_GET_PTCT,
	IOCTL_TCC_GET_REGISTER,
	IOCTL_TCC_GET_ERRLOG,
	IOCTL_TCC_MEASURE_CACHE = 10,
	IOCTL_TCC_MEASURE_USER_START,
	IOCTL_TCC_MEASURE_USER_END
};

#define TCC_GET_REGION_COUNT       _IOR(IOCTL_TCC_MAGIC, IOCTL_TCC_GET_REGION_COUNT, unsigned int *)
#define TCC_GET_MEMORY_CONFIG      _IOWR(IOCTL_TCC_MAGIC, IOCTL_TCC_GET_MEMORY_CONFIG, struct tcc_buf_mem_config_s *)
#define TCC_QUERY_PTCT_SIZE        _IOR(IOCTL_TCC_MAGIC, IOCTL_TCC_QUERY_PTCT_SIZE, unsigned int *)
#define TCC_GET_PTCT               _IOR(IOCTL_TCC_MAGIC, IOCTL_TCC_GET_PTCT, unsigned int *)
#define TCC_REQ_BUFFER             _IOWR(IOCTL_TCC_MAGIC, IOCTL_TCC_REQ_BUFFER, struct tcc_buf_mem_req_s *)
#define TCC_GET_REGISTER           _IOWR(IOCTL_TCC_MAGIC, IOCTL_TCC_GET_REGISTER, struct tcc_register_s *)
#define TCC_GET_ERRLOG             _IOWR(IOCTL_TCC_MAGIC, IOCTL_TCC_GET_ERRLOG, unsigned int *)
#define TCC_MEASURE_CACHE          _IOW(IOCTL_TCC_MAGIC, IOCTL_TCC_MEASURE_CACHE, unsigned int)
#define TCC_MEASURE_USER_START     _IO(IOCTL_TCC_MAGIC, IOCTL_TCC_MEASURE_USER_START)
#define TCC_MEASURE_USER_END       _IO(IOCTL_TCC_MAGIC, IOCTL_TCC_MEASURE_USER_END)
