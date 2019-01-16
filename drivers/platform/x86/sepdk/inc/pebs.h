/* ****************************************************************************
 *  Copyright(C) 2009-2018 Intel Corporation.  All Rights Reserved.
 *
 *  This file is part of SEP Development Kit
 *
 *  SEP Development Kit is free software; you can redistribute it
 *  and/or modify it under the terms of the GNU General Public License
 *  version 2 as published by the Free Software Foundation.
 *
 *  SEP Development Kit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  As a special exception, you may use this file as part of a free software
 *  library without restriction.  Specifically, if other files instantiate
 *  templates or use macros or inline functions from this file, or you
 *  compile this file and link it with other files to produce an executable
 *  this file does not by itself cause the resulting executable to be
 *  covered by the GNU General Public License.  This exception does not
 *  however invalidate any other reasons why the executable file might be
 *  covered by the GNU General Public License.
 * ****************************************************************************
 */

#ifndef _PEBS_H_
#define _PEBS_H_


typedef struct PEBS_REC_NODE_S PEBS_REC_NODE;

struct PEBS_REC_NODE_S {
	U64 r_flags; // Offset 0x00
	U64 linear_ip; // Offset 0x08
	U64 rax; // Offset 0x10
	U64 rbx; // Offset 0x18
	U64 rcx; // Offset 0x20
	U64 rdx; // Offset 0x28
	U64 rsi; // Offset 0x30
	U64 rdi; // Offset 0x38
	U64 rbp; // Offset 0x40
	U64 rsp; // Offset 0x48
	U64 r8; // Offset 0x50
	U64 r9; // Offset 0x58
	U64 r10; // Offset 0x60
	U64 r11; // Offset 0x68
	U64 r12; // Offset 0x70
	U64 r13; // Offset 0x78
	U64 r14; // Offset 0x80
	U64 r15; // Offset 0x88
};

typedef struct PEBS_REC_EXT_NODE_S PEBS_REC_EXT_NODE;
typedef PEBS_REC_EXT_NODE * PEBS_REC_EXT;
struct PEBS_REC_EXT_NODE_S {
	PEBS_REC_NODE pebs_basic; // Offset 0x00 to 0x88
	U64 glob_perf_overflow; // Offset 0x90
	U64 data_linear_address; // Offset 0x98
	U64 data_source; // Offset 0xA0
	U64 latency; // Offset 0xA8
};

#define PEBS_REC_EXT_r_flags(x) ((x)->pebs_basic.r_flags)
#define PEBS_REC_EXT_linear_ip(x) ((x)->pebs_basic.linear_ip)
#define PEBS_REC_EXT_rax(x) 	((x)->pebs_basic.rax)
#define PEBS_REC_EXT_rbx(x) 	((x)->pebs_basic.rbx)
#define PEBS_REC_EXT_rcx(x) 	((x)->pebs_basic.rcx)
#define PEBS_REC_EXT_rdx(x) 	((x)->pebs_basic.rdx)
#define PEBS_REC_EXT_rsi(x)		((x)->pebs_basic.rsi)
#define PEBS_REC_EXT_rdi(x) 	((x)->pebs_basic.rdi)
#define PEBS_REC_EXT_rbp(x) 	((x)->pebs_basic.rbp)
#define PEBS_REC_EXT_rsp(x) 	((x)->pebs_basic.rsp)
#define PEBS_REC_EXT_r8(x) 		((x)->pebs_basic.r8)
#define PEBS_REC_EXT_r9(x) 		((x)->pebs_basic.r9)
#define PEBS_REC_EXT_r10(x) 	((x)->pebs_basic.r10)
#define PEBS_REC_EXT_r11(x) 	((x)->pebs_basic.r11)
#define PEBS_REC_EXT_r12(x) 	((x)->pebs_basic.r12)
#define PEBS_REC_EXT_r13(x) 	((x)->pebs_basic.r13)
#define PEBS_REC_EXT_r14(x) 	((x)->pebs_basic.r14)
#define PEBS_REC_EXT_r15(x) 	((x)->pebs_basic.r15)
#define PEBS_REC_EXT_glob_perf_overflow(x) ((x)->glob_perf_overflow)
#define PEBS_REC_EXT_data_linear_address(x) ((x)->data_linear_address)
#define PEBS_REC_EXT_data_source(x) ((x)->data_source)
#define PEBS_REC_EXT_latency(x) ((x)->latency)

typedef struct PEBS_REC_EXT1_NODE_S PEBS_REC_EXT1_NODE;
typedef PEBS_REC_EXT1_NODE * PEBS_REC_EXT1;
struct PEBS_REC_EXT1_NODE_S {
	PEBS_REC_EXT_NODE pebs_ext;
	U64 eventing_ip; //Offset 0xB0
	U64 hle_info; //Offset 0xB8
};

#define PEBS_REC_EXT1_r_flags(x) ((x)->pebs_ext.pebs_basic.r_flags)
#define PEBS_REC_EXT1_linear_ip(x) ((x)->pebs_ext.pebs_basic.linear_ip)
#define PEBS_REC_EXT1_rax(x) ((x)->pebs_ext.pebs_basic.rax)
#define PEBS_REC_EXT1_rbx(x) ((x)->pebs_ext.pebs_basic.rbx)
#define PEBS_REC_EXT1_rcx(x) ((x)->pebs_ext.pebs_basic.rcx)
#define PEBS_REC_EXT1_rdx(x) ((x)->pebs_ext.pebs_basic.rdx)
#define PEBS_REC_EXT1_rsi(x) ((x)->pebs_ext.pebs_basic.rsi)
#define PEBS_REC_EXT1_rdi(x) ((x)->pebs_ext.pebs_basic.rdi)
#define PEBS_REC_EXT1_rbp(x) ((x)->pebs_ext.pebs_basic.rbp)
#define PEBS_REC_EXT1_rsp(x) ((x)->pebs_ext.pebs_basic.rsp)
#define PEBS_REC_EXT1_r8(x) ((x)->pebs_ext.pebs_basic.r8)
#define PEBS_REC_EXT1_r9(x) ((x)->pebs_ext.pebs_basic.r9)
#define PEBS_REC_EXT1_r10(x) ((x)->pebs_ext.pebs_basic.r10)
#define PEBS_REC_EXT1_r11(x) ((x)->pebs_ext.pebs_basic.r11)
#define PEBS_REC_EXT1_r12(x) ((x)->pebs_ext.pebs_basic.r12)
#define PEBS_REC_EXT1_r13(x) ((x)->pebs_ext.pebs_basic.r13)
#define PEBS_REC_EXT1_r14(x) ((x)->pebs_ext.pebs_basic.r14)
#define PEBS_REC_EXT1_r15(x) ((x)->pebs_ext.pebs_basic.r15)
#define PEBS_REC_EXT1_glob_perf_overflow(x) ((x)->pebs_ext.glob_perf_overflow)
#define PEBS_REC_EXT1_data_linear_address(x)                      \
	((x)->pebs_ext.data_linear_address)
#define PEBS_REC_EXT1_data_source(x) ((x)->pebs_ext.data_source)
#define PEBS_REC_EXT1_latency(x) ((x)->pebs_ext.latency)
#define PEBS_REC_EXT1_eventing_ip(x) ((x)->eventing_ip)
#define PEBS_REC_EXT1_hle_info(x) ((x)->hle_info)

typedef struct PEBS_REC_EXT2_NODE_S PEBS_REC_EXT2_NODE;
typedef PEBS_REC_EXT2_NODE * PEBS_REC_EXT2;
struct PEBS_REC_EXT2_NODE_S {
	PEBS_REC_EXT1_NODE pebs_ext1;
	U64 tsc; //Offset 0xC0
};

#define PEBS_REC_EXT2_r_flags(x) ((x)->pebs_ext1->pebs_ext.pebs_basic.r_flags)
#define PEBS_REC_EXT2_linear_ip(x)                                            \
	((x)->pebs_ext1->pebs_ext.pebs_basic.linear_ip)
#define PEBS_REC_EXT2_rax(x) ((x)->pebs_ext1->pebs_ext.pebs_basic.rax)
#define PEBS_REC_EXT2_rbx(x) ((x)->pebs_ext1->pebs_ext.pebs_basic.rbx)
#define PEBS_REC_EXT2_rcx(x) ((x)->pebs_ext1->pebs_ext.pebs_basic.rcx)
#define PEBS_REC_EXT2_rdx(x) ((x)->pebs_ext1->pebs_ext.pebs_basic.rdx)
#define PEBS_REC_EXT2_rsi(x) ((x)->pebs_ext1->pebs_ext.pebs_basic.rsi)
#define PEBS_REC_EXT2_rdi(x) ((x)->pebs_ext1->pebs_ext.pebs_basic.rdi)
#define PEBS_REC_EXT2_rbp(x) ((x)->pebs_ext1->pebs_ext.pebs_basic.rbp)
#define PEBS_REC_EXT2_rsp(x) ((x)->pebs_ext1->pebs_ext.pebs_basic.rsp)
#define PEBS_REC_EXT2_r8(x) ((x)->pebs_ext1->pebs_ext.pebs_basic.r8)
#define PEBS_REC_EXT2_r9(x) ((x)->pebs_ext1->pebs_ext.pebs_basic.r9)
#define PEBS_REC_EXT2_r10(x) ((x)->pebs_ext1->pebs_ext.pebs_basic.r10)
#define PEBS_REC_EXT2_r11(x) ((x)->pebs_ext1->pebs_ext.pebs_basic.r11)
#define PEBS_REC_EXT2_r12(x) ((x)->pebs_ext1->pebs_ext.pebs_basic.r12)
#define PEBS_REC_EXT2_r13(x) ((x)->pebs_ext1->pebs_ext.pebs_basic.r13)
#define PEBS_REC_EXT2_r14(x) ((x)->pebs_ext1->pebs_ext.pebs_basic.r14)
#define PEBS_REC_EXT2_r15(x) ((x)->pebs_ext1->pebs_ext.pebs_basic.r15)
#define PEBS_REC_EXT2_glob_perf_overflow(x)                                   \
	((x)->pebs_ext1->pebs_ext.glob_perf_overflow)
#define PEBS_REC_EXT2_data_linear_address(x)                                  \
	((x)->pebs_ext1->pebs_ext.data_linear_address)
#define PEBS_REC_EXT2_data_source(x) ((x)->pebs_ext1->pebs_ext.data_source)
#define PEBS_REC_EXT2_latency(x) ((x)->pebs_ext1->pebs_ext.latency)
#define PEBS_REC_EXT2_eventing_ip(x) ((x)->pebs_ext1->eventing_ip)
#define PEBS_REC_EXT2_hle_info(x) ((x)->pebs_ext1->hle_info)
#define PEBS_REC_EXT2_tsc(x) ((x)->tsc)

typedef struct APEBS_CONFIG_NODE_S APEBS_CONFIG_NODE;
typedef APEBS_CONFIG_NODE * APEBS_CONFIG;

struct APEBS_CONFIG_NODE_S {
	U8 apebs_enabled;
	U8 collect_mem;
	U8 collect_gpr;
	U8 collect_xmm;
	U8 collect_lbrs;
	U8 precise_ip_lbrs;
	U8 num_lbr_entries;
	U16 basic_offset;
	U16 mem_offset;
	U16 gpr_offset;
	U16 xmm_offset;
	U16 lbr_offset;
};

#define APEBS_CONFIG_apebs_enabled(x) ((x)->apebs_enabled)
#define APEBS_CONFIG_collect_mem(x) ((x)->collect_mem)
#define APEBS_CONFIG_collect_gpr(x) ((x)->collect_gpr)
#define APEBS_CONFIG_collect_xmm(x) ((x)->collect_xmm)
#define APEBS_CONFIG_collect_lbrs(x) ((x)->collect_lbrs)
#define APEBS_CONFIG_precise_ip_lbrs(x) ((x)->precise_ip_lbrs)
#define APEBS_CONFIG_num_lbr_entries(x) ((x)->num_lbr_entries)
#define APEBS_CONFIG_basic_offset(x) ((x)->basic_offset)
#define APEBS_CONFIG_mem_offset(x) ((x)->mem_offset)
#define APEBS_CONFIG_gpr_offset(x) ((x)->gpr_offset)
#define APEBS_CONFIG_xmm_offset(x) ((x)->xmm_offset)
#define APEBS_CONFIG_lbr_offset(x) ((x)->lbr_offset)

typedef struct ADAPTIVE_PEBS_BASIC_INFO_NODE_S ADAPTIVE_PEBS_BASIC_INFO_NODE;
typedef ADAPTIVE_PEBS_BASIC_INFO_NODE * ADAPTIVE_PEBS_BASIC_INFO;

struct ADAPTIVE_PEBS_BASIC_INFO_NODE_S {
	U64 record_info; // Offset 0x0
		// [47:0] - record format, [63:48] - record size
	U64 eventing_ip; // Offset 0x8
	U64 applicable_counters; // Offset 0x10
	U64 tsc; // Offset 0x18
};

#define ADAPTIVE_PEBS_BASIC_INFO_record_info(x) ((x)->record_info)
#define ADAPTIVE_PEBS_BASIC_INFO_eventing_ip(x) ((x)->eventing_ip)
#define ADAPTIVE_PEBS_BASIC_INFO_tsc(x) ((x)->tsc)
#define ADAPTIVE_PEBS_BASIC_INFO_applicable_counters(x)                       \
	((x)->applicable_counters)

typedef struct ADAPTIVE_PEBS_MEM_INFO_NODE_S ADAPTIVE_PEBS_MEM_INFO_NODE;
typedef ADAPTIVE_PEBS_MEM_INFO_NODE * ADAPTIVE_PEBS_MEM_INFO;

struct ADAPTIVE_PEBS_MEM_INFO_NODE_S {
	U64 data_linear_address; // Offset 0x20
	U64 data_source; // Offset 0x28
	U64 latency; // Offset 0x30
	U64 hle_info; // Offset 0x38
};

#define ADAPTIVE_PEBS_MEM_INFO_data_linear_address(x) ((x)->data_linear_address)
#define ADAPTIVE_PEBS_MEM_INFO_data_source(x) ((x)->data_source)
#define ADAPTIVE_PEBS_MEM_INFO_latency(x) ((x)->latency)
#define ADAPTIVE_PEBS_MEM_INFO_hle_info(x) ((x)->hle_info)

typedef struct ADAPTIVE_PEBS_GPR_INFO_NODE_S ADAPTIVE_PEBS_GPR_INFO_NODE;
typedef ADAPTIVE_PEBS_GPR_INFO_NODE * ADAPTIVE_PEBS_GPR_INFO;

struct ADAPTIVE_PEBS_GPR_INFO_NODE_S {
	U64 rflags; // Offset 0x40
	U64 rip; // Offset 0x48
	U64 rax; // Offset 0x50
	U64 rcx; // Offset 0x58
	U64 rdx; // Offset 0x60
	U64 rbx; // Offset 0x68
	U64 rsp; // Offset 0x70
	U64 rbp; // Offset 0x78
	U64 rsi; // Offset 0x80
	U64 rdi; // Offset 0x88
	U64 r8; // Offset 0x90
	U64 r9; // Offset 0x98
	U64 r10; // Offset 0xA0
	U64 r11; // Offset 0xA8
	U64 r12; // Offset 0xB0
	U64 r13; // Offset 0xB8
	U64 r14; // Offset 0xC0
	U64 r15; // Offset 0xC8
};

#define ADAPTIVE_PEBS_GPR_INFO_rflags(x) ((x)->rflags)
#define ADAPTIVE_PEBS_GPR_INFO_rip(x) ((x)->rip)
#define ADAPTIVE_PEBS_GPR_INFO_rax(x) ((x)->rax)
#define ADAPTIVE_PEBS_GPR_INFO_rcx(x) ((x)->rcx)
#define ADAPTIVE_PEBS_GPR_INFO_rdx(x) ((x)->rdx)
#define ADAPTIVE_PEBS_GPR_INFO_rbx(x) ((x)->rbx)
#define ADAPTIVE_PEBS_GPR_INFO_rsp(x) ((x)->rsp)
#define ADAPTIVE_PEBS_GPR_INFO_rbp(x) ((x)->rbp)
#define ADAPTIVE_PEBS_GPR_INFO_rsi(x) ((x)->rsi)
#define ADAPTIVE_PEBS_GPR_INFO_rdi(x) ((x)->rdi)
#define ADAPTIVE_PEBS_GPR_INFO_r8(x) ((x)->r8)
#define ADAPTIVE_PEBS_GPR_INFO_r9(x) ((x)->r9)
#define ADAPTIVE_PEBS_GPR_INFO_r10(x) ((x)->r10)
#define ADAPTIVE_PEBS_GPR_INFO_r11(x) ((x)->r11)
#define ADAPTIVE_PEBS_GPR_INFO_r12(x) ((x)->r12)
#define ADAPTIVE_PEBS_GPR_INFO_r13(x) ((x)->r13)
#define ADAPTIVE_PEBS_GPR_INFO_r14(x) ((x)->r14)
#define ADAPTIVE_PEBS_GPR_INFO_r15(x) ((x)->r15)

typedef struct ADAPTIVE_PEBS_XMM_INFO_NODE_S ADAPTIVE_PEBS_XMM_INFO_NODE;
typedef ADAPTIVE_PEBS_XMM_INFO_NODE * ADAPTIVE_PEBS_XMM_INFO;

struct ADAPTIVE_PEBS_XMM_INFO_NODE_S {
	U64 xmm0_l; // Offset 0xD0
	U64 xmm0_h; // Offset 0xD8
	U64 xmm1_l; // Offset 0xE0
	U64 xmm1_h; // Offset 0xE8
	U64 xmm2_l; // Offset 0xF0
	U64 xmm2_h; // Offset 0xF8
	U64 xmm3_l; // Offset 0x100
	U64 xmm3_h; // Offset 0x108
	U64 xmm4_l; // Offset 0x110
	U64 xmm4_h; // Offset 0x118
	U64 xmm5_l; // Offset 0x120
	U64 xmm5_h; // Offset 0x128
	U64 xmm6_l; // Offset 0x130
	U64 xmm6_h; // Offset 0x138
	U64 xmm7_l; // Offset 0x140
	U64 xmm7_h; // Offset 0x148
	U64 xmm8_l; // Offset 0x150
	U64 xmm8_h; // Offset 0x158
	U64 xmm9_l; // Offset 0x160
	U64 xmm9_h; // Offset 0x168
	U64 xmm10_l; // Offset 0x170
	U64 xmm10_h; // Offset 0x178
	U64 xmm11_l; // Offset 0x180
	U64 xmm11_h; // Offset 0x188
	U64 xmm12_l; // Offset 0x190
	U64 xmm12_h; // Offset 0x198
	U64 xmm13_l; // Offset 0x1A0
	U64 xmm13_h; // Offset 0x1A8
	U64 xmm14_l; // Offset 0x1B0
	U64 xmm14_h; // Offset 0x1B8
	U64 xmm15_l; // Offset 0x1C0
	U64 xmm15_h; // Offset 0x1C8
};

#define ADAPTIVE_PEBS_XMM_INFO_xmm0_l(x) ((x)->xmm0_l)
#define ADAPTIVE_PEBS_XMM_INFO_xmm0_h(x) ((x)->xmm0_h)
#define ADAPTIVE_PEBS_XMM_INFO_xmm1_l(x) ((x)->xmm1_l)
#define ADAPTIVE_PEBS_XMM_INFO_xmm1_h(x) ((x)->xmm1_h)
#define ADAPTIVE_PEBS_XMM_INFO_xmm2_l(x) ((x)->xmm2_l)
#define ADAPTIVE_PEBS_XMM_INFO_xmm2_h(x) ((x)->xmm2_h)
#define ADAPTIVE_PEBS_XMM_INFO_xmm3_l(x) ((x)->xmm3_l)
#define ADAPTIVE_PEBS_XMM_INFO_xmm3_h(x) ((x)->xmm3_h)
#define ADAPTIVE_PEBS_XMM_INFO_xmm4_l(x) ((x)->xmm4_l)
#define ADAPTIVE_PEBS_XMM_INFO_xmm4_h(x) ((x)->xmm4_h)
#define ADAPTIVE_PEBS_XMM_INFO_xmm5_l(x) ((x)->xmm5_l)
#define ADAPTIVE_PEBS_XMM_INFO_xmm5_h(x) ((x)->xmm5_h)
#define ADAPTIVE_PEBS_XMM_INFO_xmm6_l(x) ((x)->xmm6_l)
#define ADAPTIVE_PEBS_XMM_INFO_xmm6_h(x) ((x)->xmm6_h)
#define ADAPTIVE_PEBS_XMM_INFO_xmm7_l(x) ((x)->xmm7_l)
#define ADAPTIVE_PEBS_XMM_INFO_xmm7_h(x) ((x)->xmm7_h)
#define ADAPTIVE_PEBS_XMM_INFO_xmm8_l(x) ((x)->xmm8_l)
#define ADAPTIVE_PEBS_XMM_INFO_xmm8_h(x) ((x)->xmm8_h)
#define ADAPTIVE_PEBS_XMM_INFO_xmm9_l(x) ((x)->xmm9_l)
#define ADAPTIVE_PEBS_XMM_INFO_xmm9_h(x) ((x)->xmm9_h)
#define ADAPTIVE_PEBS_XMM_INFO_xmm10_l(x) ((x)->xmm10_l)
#define ADAPTIVE_PEBS_XMM_INFO_xmm10_h(x) ((x)->xmm10_h)
#define ADAPTIVE_PEBS_XMM_INFO_xmm11_l(x) ((x)->xmm11_l)
#define ADAPTIVE_PEBS_XMM_INFO_xmm11_h(x) ((x)->xmm11_h)
#define ADAPTIVE_PEBS_XMM_INFO_xmm12_l(x) ((x)->xmm12_l)
#define ADAPTIVE_PEBS_XMM_INFO_xmm12_h(x) ((x)->xmm12_h)
#define ADAPTIVE_PEBS_XMM_INFO_xmm13_l(x) ((x)->xmm13_l)
#define ADAPTIVE_PEBS_XMM_INFO_xmm13_h(x) ((x)->xmm13_h)
#define ADAPTIVE_PEBS_XMM_INFO_xmm14_l(x) ((x)->xmm14_l)
#define ADAPTIVE_PEBS_XMM_INFO_xmm14_h(x) ((x)->xmm14_h)
#define ADAPTIVE_PEBS_XMM_INFO_xmm15_l(x) ((x)->xmm15_l)
#define ADAPTIVE_PEBS_XMM_INFO_xmm15_h(x) ((x)->xmm15_h)

typedef struct ADAPTIVE_PEBS_LBR_INFO_NODE_S ADAPTIVE_PEBS_LBR_INFO_NODE;
typedef ADAPTIVE_PEBS_LBR_INFO_NODE * ADAPTIVE_PEBS_LBR_INFO;

struct ADAPTIVE_PEBS_LBR_INFO_NODE_S {
	U64 lbr_from; // Offset 0x1D0
	U64 lbr_to; // Offset 0x1D8
	U64 lbr_info; // Offset 0x1E0
};

#define ADAPTIVE_PEBS_LBR_INFO_lbr_from(x) ((x)->lbr_from)
#define ADAPTIVE_PEBS_LBR_INFO_lbr_to(x) ((x)->lbr_to)
#define ADAPTIVE_PEBS_LBR_INFO_lbr_info(x) ((x)->lbr_info)

typedef struct LATENCY_INFO_NODE_S LATENCY_INFO_NODE;
typedef LATENCY_INFO_NODE * LATENCY_INFO;

struct LATENCY_INFO_NODE_S {
	U64 linear_address;
	U64 data_source;
	U64 latency;
	U64 stack_pointer;
	U64 phys_addr;
};

#define LATENCY_INFO_linear_address(x) ((x)->linear_address)
#define LATENCY_INFO_data_source(x) ((x)->data_source)
#define LATENCY_INFO_latency(x) ((x)->latency)
#define LATENCY_INFO_stack_pointer(x) ((x)->stack_pointer)
#define LATENCY_INFO_phys_addr(x) ((x)->phys_addr)

typedef struct DTS_BUFFER_EXT_NODE_S DTS_BUFFER_EXT_NODE;
typedef DTS_BUFFER_EXT_NODE * DTS_BUFFER_EXT;
struct DTS_BUFFER_EXT_NODE_S {
	U64 base; // Offset 0x00
	U64 index; // Offset 0x08
	U64 max; // Offset 0x10
	U64 threshold; // Offset 0x18
	U64 pebs_base; // Offset 0x20
	U64 pebs_index; // Offset 0x28
	U64 pebs_max; // Offset 0x30
	U64 pebs_threshold; // Offset 0x38
	U64 counter_reset0; // Offset 0x40
	U64 counter_reset1; // Offset 0x48
	U64 counter_reset2; // Offset 0x50
	U64 counter_reset3;
};

#define DTS_BUFFER_EXT_base(x) ((x)->base)
#define DTS_BUFFER_EXT_index(x) ((x)->index)
#define DTS_BUFFER_EXT_max(x) ((x)->max)
#define DTS_BUFFER_EXT_threshold(x) ((x)->threshold)
#define DTS_BUFFER_EXT_pebs_base(x) ((x)->pebs_base)
#define DTS_BUFFER_EXT_pebs_index(x) ((x)->pebs_index)
#define DTS_BUFFER_EXT_pebs_max(x) ((x)->pebs_max)
#define DTS_BUFFER_EXT_pebs_threshold(x) ((x)->pebs_threshold)
#define DTS_BUFFER_EXT_counter_reset0(x) ((x)->counter_reset0)
#define DTS_BUFFER_EXT_counter_reset1(x) ((x)->counter_reset1)
#define DTS_BUFFER_EXT_counter_reset2(x) ((x)->counter_reset2)
#define DTS_BUFFER_EXT_counter_reset3(x) ((x)->counter_reset3)

typedef struct DTS_BUFFER_EXT1_NODE_S DTS_BUFFER_EXT1_NODE;
typedef DTS_BUFFER_EXT1_NODE * DTS_BUFFER_EXT1;
struct DTS_BUFFER_EXT1_NODE_S {
	DTS_BUFFER_EXT_NODE dts_buffer;
	U64 counter_reset4; // Offset 0x60
	U64 counter_reset5; // Offset 0x68
	U64 counter_reset6; // Offset 0x70
	U64 counter_reset7; // Offset 0x78
	U64 fixed_counter_reset0; // Offset 0x80
	U64 fixed_counter_reset1; // Offset 0x88
	U64 fixed_counter_reset2; // Offset 0x90
	U64 fixed_counter_reset3; // Offset 0x98
};

#define DTS_BUFFER_EXT1_base(x) ((x)->dts_buffer.base)
#define DTS_BUFFER_EXT1_index(x) ((x)->dts_buffer.index)
#define DTS_BUFFER_EXT1_max(x) ((x)->dts_buffer.max)
#define DTS_BUFFER_EXT1_threshold(x) ((x)->dts_buffer.threshold)
#define DTS_BUFFER_EXT1_pebs_base(x) ((x)->dts_buffer.pebs_base)
#define DTS_BUFFER_EXT1_pebs_index(x) ((x)->dts_buffer.pebs_index)
#define DTS_BUFFER_EXT1_pebs_max(x) ((x)->dts_buffer.pebs_max)
#define DTS_BUFFER_EXT1_pebs_threshold(x) ((x)->dts_buffer.pebs_threshold)
#define DTS_BUFFER_EXT1_counter_reset0(x) ((x)->dts_buffer.counter_reset0)
#define DTS_BUFFER_EXT1_counter_reset1(x) ((x)->dts_buffer.counter_reset1)
#define DTS_BUFFER_EXT1_counter_reset2(x) ((x)->dts_buffer.counter_reset2)
#define DTS_BUFFER_EXT1_counter_reset3(x) ((x)->dts_buffer.counter_reset3)
#define DTS_BUFFER_EXT1_counter_reset4(x) ((x)->counter_reset4)
#define DTS_BUFFER_EXT1_counter_reset5(x) ((x)->counter_reset5)
#define DTS_BUFFER_EXT1_counter_reset6(x) ((x)->counter_reset6)
#define DTS_BUFFER_EXT1_counter_reset7(x) ((x)->counter_reset7)
#define DTS_BUFFER_EXT1_fixed_counter_reset0(x) ((x)->fixed_counter_reset0)
#define DTS_BUFFER_EXT1_fixed_counter_reset1(x) ((x)->fixed_counter_reset1)
#define DTS_BUFFER_EXT1_fixed_counter_reset2(x) ((x)->fixed_counter_reset2)
#define DTS_BUFFER_EXT1_fixed_counter_reset3(x) ((x)->fixed_counter_reset3)

extern OS_STATUS PEBS_Initialize(U32 dev_idx);

extern OS_STATUS PEBS_Allocate(void);

extern VOID PEBS_Destroy(void);

extern VOID PEBS_Flush_Buffer(void *);

extern VOID PEBS_Reset_Counter(S32 this_cpu, U32 index, U64 value);

extern VOID PEBS_Reset_Index(S32 this_cpu);

extern VOID PEBS_Modify_IP(void *sample, DRV_BOOL is_64bit_addr, U32 rec_index);

extern VOID PEBS_Modify_TSC(void *sample, U32 rec_index);

extern U32 PEBS_Get_Num_Records_Filled(void);

extern U64 PEBS_Fill_Buffer(S8 *buffer, EVENT_DESC evt_desc, U32 rec_index);

extern U64 APEBS_Fill_Buffer(S8 *buffer, EVENT_DESC evt_desc, U32 rec_index);

extern U64 PEBS_Overflowed(S32 this_cpu, U64 overflow_status, U32 rec_index);

/*
 *  Dispatch table for virtualized functions.
 *  Used to enable common functionality for different
 *  processor microarchitectures
 */
typedef struct PEBS_DISPATCH_NODE_S PEBS_DISPATCH_NODE;
typedef PEBS_DISPATCH_NODE * PEBS_DISPATCH;
struct PEBS_DISPATCH_NODE_S {
	VOID (*initialize_threshold)(DTS_BUFFER_EXT);
	U64 (*overflow)(S32, U64, U32);
	VOID (*modify_ip)(void *, DRV_BOOL, U32);
	VOID (*modify_tsc)(void *, U32);
	U32 (*get_num_records_filled)(void);
};

typedef struct PEBS_INFO_NODE_S PEBS_INFO_NODE;
typedef PEBS_INFO_NODE *PEBS_INFO;
struct PEBS_INFO_NODE_S {
	PEBS_DISPATCH pebs_dispatch;
	U32 pebs_record_size;
	U16 apebs_basic_offset;
	U16 apebs_mem_offset;
	U16 apebs_gpr_offset;
	U16 apebs_xmm_offset;
	U16 apebs_lbr_offset;
};

#define APEBS_RECORD_SIZE_MASK 0xFFFF000000000000ULL //[63:48]
#define APEBS_RECORD_FORMAT_MASK 0xFFFFFFFFFFFFULL //[47:0]
#define APEBS_MEM_RECORD_FORMAT_MASK 0x1ULL
#define APEBS_GPR_RECORD_FORMAT_MASK 0x2ULL
#define APEBS_XMM_RECORD_FORMAT_MASK 0x4ULL
#define APEBS_LBR_RECORD_FORMAT_MASK 0x8ULL


extern PEBS_DISPATCH_NODE core2_pebs;
extern PEBS_DISPATCH_NODE core2p_pebs;
extern PEBS_DISPATCH_NODE corei7_pebs;
extern PEBS_DISPATCH_NODE haswell_pebs;
extern PEBS_DISPATCH_NODE perfver4_pebs;
extern PEBS_DISPATCH_NODE perfver4_apebs;

#endif
