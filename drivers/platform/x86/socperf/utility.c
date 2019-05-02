/************************************************************************
/* ***********************************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(C) 2005-2019 Intel Corporation. All rights reserved.
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
 * BSD LICENSE
 *
 * Copyright(C) 2005-2019 Intel Corporation. All rights reserved.
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
 * ***********************************************************************************************
 */


#include "lwpmudrv_defines.h"
#include <linux/version.h>
#include <linux/fs.h>
#include <asm/msr.h>
#include <linux/ptrace.h>

#include "lwpmudrv_types.h"
#include "rise_errors.h"
#include "lwpmudrv_ecb.h"
#include "socperfdrv.h"
#include "utility.h"
#if defined(DRV_SOFIA)
#include "noc_uncore.h"
#elif defined(DRV_BUTTER)
#include "axi_uncore.h"
#else
#include "soc_uncore.h"
#include "haswellunc_sa.h"
#include "npk_uncore.h"
#endif

volatile int config_done;

VOID SOCPERF_UTILITY_Read_TSC(U64 *pTsc)
{
	*pTsc = rdtsc_ordered();
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn       VOID SOCPERF_UTILITY_Read_Cpuid
 *
 * @brief    executes the cpuid_function of cpuid and returns values
 *
 * @param  IN   cpuid_function
 *         OUT  rax  - results of the cpuid instruction in the
 *         OUT  rbx  - corresponding registers
 *         OUT  rcx
 *         OUT  rdx
 *
 * @return   none
 *
 * <I>Special Notes:</I>
 *              <NONE>
 *
 */
VOID SOCPERF_UTILITY_Read_Cpuid(U64 cpuid_function, U64 *rax_value,
				       U64 *rbx_value, U64 *rcx_value,
				       U64 *rdx_value)
{
	U32 function = (U32)cpuid_function;
	U32 *eax = (U32 *)rax_value;
	U32 *ebx = (U32 *)rbx_value;
	U32 *ecx = (U32 *)rcx_value;
	U32 *edx = (U32 *)rdx_value;

	*eax = function;

	__asm__("cpuid"
		: "=a"(*eax), "=b"(*ebx), "=c"(*ecx), "=d"(*edx)
		: "a"(function), "b"(*ebx), "c"(*ecx), "d"(*edx));
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn       VOID SOCPERF_UTILITY_Configure_CPU
 *
 * @brief    Reads the CPU information from the hardware
 *
 * @param    param   dispatch_id -  The id of the dispatch table.
 *
 * @return   Pointer to the correct dispatch table for the CPU architecture
 *
 * <I>Special Notes:</I>
 *              <NONE>
 */
DISPATCH SOCPERF_UTILITY_Configure_CPU(U32 dispatch_id)
{
	DISPATCH dispatch = NULL;

	switch (dispatch_id) {
#if defined(DRV_SOFIA)
	case 1000:
		SOCPERF_PRINT_DEBUG(
			"Set up the SoC Uncore NOC dispatch table\n");
		dispatch = &noc_dispatch;
		break;
#elif defined(DRV_BUTTER)
	case 1100:
		SOCPERF_PRINT_DEBUG(
			"Set up the SoC Uncore AXI dispatch table\n");
		dispatch = &axi_dispatch;
		break;
#else
	case 230:
		SOCPERF_PRINT_DEBUG("Set up the Haswell SA dispatch table\n");
		dispatch = &socperf_hswunc_sa_dispatch;
		break;
	case 700:
		SOCPERF_PRINT_DEBUG("Set up the SOC Uncore dispatch table\n");
		dispatch = &soc_uncore_dispatch;
		break;
	case 701:
		SOCPERF_PRINT_DEBUG(
			"Set up the SoC Uncore NPK dispatch table\n");
		dispatch = &npk_dispatch;
		break;
#endif
	default:
		dispatch = NULL;
		SOCPERF_PRINT_ERROR(
			"Architecture not supported (dispatch_id=%d)\n",
			dispatch_id);
		break;
	}

	return dispatch;
}
