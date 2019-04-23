/* SPDX-License-Identifier: GPL-2.0 AND BSD-3-Clause
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2014 - 2019 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Contact Information:
 * SoC Watch Developer Team <socwatchdevelopers@intel.com>
 * Intel Corporation,
 * 1300 S Mopac Expwy,
 * Austin, TX 78746
 *
 * BSD LICENSE
 *
 * Copyright(c) 2014 - 2019 Intel Corporation.
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
 */

#ifndef _SWHV_DRIVER_H_
#define _SWHV_DRIVER_H_ 1

#include <linux/version.h> /* LINUX_VERSION_CODE */
#include <linux/vmalloc.h> /* vmalloc */
#include "swhv_defines.h"
#include "sw_kernel_defines.h"
#include "pw_version.h"

#define MAX_CORE_COUNT 8

#define MOBILEVISOR 1
#define ACRN 2

/* define this flag to have IDT entry programmed for SoCWatch IRQ handler */
#define SOCWATCH_IDT_IRQ 1

extern void SYS_Perfvec_Handler(void);
extern short SYS_Get_cs(void);

#if defined(SWDRV_IA32) && (SOCWATCH_IDT_IRQ)
extern void *SYS_Get_IDT_Base_HWR(void);   /* / IDT base from hardware IDTR */

#define SYS_Get_IDT_Base SYS_Get_IDT_Base_HWR
#endif /* defined(SWDRV_IA32) && (SOCWATCH_IDT_IRQ) */

#if defined(SWDRV_EM64T) && (SOCWATCH_IDT_IRQ)
extern void SYS_Get_IDT_Base(void **);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 25)
typedef struct gate_struct gate_struct_t;
#else
typedef struct gate_struct64 gate_struct_t;
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,25) */
#endif /* defined(SWDRV_EM64T) && (SOCWATCH_IDT_IRQ) */

/* miscellaneous defines */
#define CPU() (raw_smp_processor_id())
#define GET_BOOL_STRING(b) ((b) ? "TRUE" : "FALSE")

#define _STRINGIFY(x)     #x
#define STRINGIFY(x)      _STRINGIFY(x)
#define _STRINGIFY_W(x)   L#x
#define STRINGIFY_W(x)    _STRINGIFY_W(x)

/*
 * 64bit Compare-and-swap.
 */
#define CAS64(p, o, n) (cmpxchg64((p), (o), (n)) == (o))

typedef struct PWCollector_msg PWCollector_msg_t;

#endif /* _SWHV_DRIVER_H_ */
