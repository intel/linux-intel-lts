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

#ifndef _CONTROL_H_
#define _CONTROL_H_

#include <linux/smp.h>
#include <linux/timer.h>
#include <asm/io.h>
#include <asm/atomic.h>

#include "swhv_driver.h"
/****************************************************************************
 **  Handy Short cuts
 ***************************************************************************/

typedef void *pvoid;
#define TRUE 1
#define FALSE 0
/*
 *  These routines have macros defined in asm/system.h
 */
#define SYS_Local_Irq_Enable() local_irq_enable()
#define SYS_Local_Irq_Disable() local_irq_disable()
#define SYS_Local_Irq_Save(flags) local_irq_save(flags)
#define SYS_Local_Irq_Restore(flags) local_irq_restore(flags)

/*
 * CONTROL_THIS_CPU()
 *     Parameters
 *         None
 *     Returns
 *         CPU number of the processor being executed on
 *
 */
#define CONTROL_THIS_CPU() smp_processor_id()

/****************************************************************************
 **  Interface definitions
 ***************************************************************************/

/*
 *  Execution Control Functions
 */

extern void CONTROL_Invoke_Cpu(s32 cpuid, void (*func)(pvoid), pvoid ctx);

/*
 * @fn VOID CONTROL_Invoke_Parallel_Service(func, ctx, blocking, exclude)
 *
 * @param    func     - function to be invoked by each core in the system
 * @param    ctx      - pointer to the parameter block for each function
 *                      invocation
 * @param    blocking - Wait for invoked function to complete
 * @param    exclude  - exclude the current core from executing the code
 *
 * @returns  none
 *
 * @brief    Service routine to handle all kinds of parallel invoke on
 *           all CPU calls
 *
 * <I>Special Notes:</I>
 *         Invoke the function provided in parallel in either a
 *         blocking/non-blocking mode.
 *         The current core may be excluded if desired.
 *         NOTE - Do not call this function directly from source code.
 *         Use the aliases
 *         CONTROL_Invoke_Parallel(), CONTROL_Invoke_Parallel_NB(),
 *         CONTROL_Invoke_Parallel_XS().
 *
 */
extern void CONTROL_Invoke_Parallel_Service(void (*func)(pvoid), pvoid ctx,
					    s32 blocking, s32 exclude);

/*
 * @fn VOID CONTROL_Invoke_Parallel(func, ctx)
 *
 * @param    func     - function to be invoked by each core in the system
 * @param    ctx      - pointer to the parameter block for each function
 *                      invocation
 *
 * @returns  none
 *
 * @brief    Invoke the named function in parallel. Wait for all the
 *           functions to complete.
 *
 * <I>Special Notes:</I>
 *        Invoke the function named in parallel, including the CPU
 *        that the control is being invoked on
 *
 *        Macro built on the service routine
 *
 */
#define CONTROL_Invoke_Parallel(a, b)                                          \
	CONTROL_Invoke_Parallel_Service((a), (b), TRUE, FALSE)

/*
 * @fn VOID CONTROL_Invoke_Parallel_NB(func, ctx)
 *
 * @param    func     - function to be invoked by each core in the system
 * @param    ctx      - pointer to the parameter block for each function
 *                      invocation
 *
 * @returns  none
 *
 * @brief    Invoke the named function in parallel. DO NOT Wait for all
 *           the functions to complete.
 *
 * <I>Special Notes:</I>
 *        Invoke the function named in parallel, including the CPU
 *        that the control is being invoked on
 *
 *        Macro built on the service routine
 *
 */
#define CONTROL_Invoke_Parallel_NB(a, b)                                       \
	CONTROL_Invoke_Parallel_Service((a), (b), FALSE, FALSE)

/*
 * @fn VOID CONTROL_Invoke_Parallel_XS(func, ctx)
 *
 * @param    func     - function to be invoked by each core in the system
 * @param    ctx      - pointer to the parameter block for each function
 *                      invocation
 *
 * @returns  none
 *
 * @brief    Invoke the named function in parallel. Wait for all
 *           the functions to complete.
 *
 * <I>Special Notes:</I>
 *        Invoke the function named in parallel, excluding the CPU
 *        that the control is being invoked on
 *
 *        Macro built on the service routine
 *
 */
#define CONTROL_Invoke_Parallel_XS(a, b)                                       \
	CONTROL_Invoke_Parallel_Service((a), (b), TRUE, TRUE)

#endif
