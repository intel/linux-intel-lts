/*

  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY

  Copyright(c) 2014 - 2018 Intel Corporation.

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  Contact Information:
  SoC Watch Developer Team <socwatchdevelopers@intel.com>
  Intel Corporation,
  1300 S Mopac Expwy,
  Austin, TX 78746

  BSD LICENSE

  Copyright(c) 2014 - 2018 Intel Corporation.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <linux/version.h>

#include "control.h"
#include <linux/sched.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
#define SMP_CALL_FUNCTION(func, ctx, retry, wait)                              \
	smp_call_function((func), (ctx), (wait))
#define SMP_CALL_FUNCTION_SINGLE(cpuid, func, ctx, retry, wait)                \
	smp_call_function_single((cpuid), (func), (ctx), (wait))
#define ON_EACH_CPU(func, ctx, retry, wait) on_each_cpu((func), (ctx), (wait))
#else
#define SMP_CALL_FUNCTION(func, ctx, retry, wait)                              \
	smp_call_function((func), (ctx), (retry), (wait))
#define SMP_CALL_FUNCTION_SINGLE(cpuid, func, ctx, retry, wait)                \
	smp_call_function_single((cpuid), (func), (ctx), (retry), (wait))
#define ON_EACH_CPU(func, ctx, retry, wait)                                    \
	on_each_cpu((func), (ctx), (retry), (wait))
#endif

extern int num_CPUs;
/* ------------------------------------------------------------------------- */
/*!
 * @fn       VOID CONTROL_Invoke_Cpu (func, ctx, arg)
 *
 * @brief    Set up a DPC call and insert it into the queue
 *
 * @param    IN cpu_idx  - the core id to dispatch this function to
 *           IN func     - function to be invoked by the specified core(s)
 *           IN ctx      - pointer to the parameter block for each function
 *                         invocation
 *
 * @return   None
 *
 * <I>Special Notes:</I>
 *
 */
extern void CONTROL_Invoke_Cpu(int cpu_idx, void (*func)(pvoid), pvoid ctx)
{
	SMP_CALL_FUNCTION_SINGLE(cpu_idx, func, ctx, 0, 1);

	return;
}

/* ------------------------------------------------------------------------- */
/*
 * @fn VOID CONTROL_Invoke_Parallel_Service(func, ctx, blocking, exclude)
 *
 * @param    func     - function to be invoked by each core in the system
 * @param    ctx      - pointer to the parameter block for each function invocation
 * @param    blocking - Wait for invoked function to complete
 * @param    exclude  - exclude the current core from executing the code
 *
 * @returns  None
 *
 * @brief    Service routine to handle all kinds of parallel invoke on all CPU calls
 *
 * <I>Special Notes:</I>
 *           Invoke the function provided in parallel in either a blocking or
 *           non-blocking mode.  The current core may be excluded if desired.
 *           NOTE - Do not call this function directly from source code.
 *           Use the aliases CONTROL_Invoke_Parallel(), CONTROL_Invoke_Parallel_NB(),
 *           or CONTROL_Invoke_Parallel_XS().
 *
 */
extern void CONTROL_Invoke_Parallel_Service(void (*func)(pvoid), pvoid ctx,
					    int blocking, int exclude)
{
	if (num_CPUs == 1) {
		if (!exclude) {
			func(ctx);
		}
		return;
	}
	if (!exclude) {
		ON_EACH_CPU(func, ctx, 0, blocking);
		return;
	}

	preempt_disable();
	SMP_CALL_FUNCTION(func, ctx, 0, blocking);
	preempt_enable();

	return;
}
