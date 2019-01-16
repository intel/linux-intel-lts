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

#include "lwpmudrv_defines.h"
#include <linux/version.h>
#include <linux/jiffies.h>
#include <linux/time.h>
#include <linux/percpu.h>
#include "lwpmudrv_types.h"
#include "lwpmudrv_ecb.h"
#include "lwpmudrv_struct.h"
#include "lwpmudrv.h"
#include "control.h"
#include "utility.h"
#include "eventmux.h"

static PVOID em_tables;
static size_t em_tables_size;

/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID eventmux_Allocate_Groups (
 *                         VOID  *params
 *                        )
 *
 * @brief       Allocate memory need to support event multiplexing
 *
 * @param       params - pointer to a S32 that holds the size of buffer to allocate
 *
 * @return      NONE
 *
 * <I>Special Notes:</I>
 *              Allocate the memory needed to save different group counters
 *              Called via the parallel control mechanism
 */
static VOID eventmux_Allocate_Groups(PVOID params)
{
	U32 this_cpu;
	CPU_STATE cpu_state;
	U32 dev_idx;
	EVENT_CONFIG ec;

	SEP_DRV_LOG_TRACE_IN("");

	preempt_disable();
	this_cpu = CONTROL_THIS_CPU();
	cpu_state = &pcb[this_cpu];
	dev_idx = core_to_dev_map[this_cpu];
	ec = LWPMU_DEVICE_ec(&devices[dev_idx]);
	preempt_enable();

	if (EVENT_CONFIG_mode(ec) == EM_DISABLED ||
	    EVENT_CONFIG_num_groups(ec) == 1) {
		return;
	}

	CPU_STATE_em_tables(cpu_state) =
		em_tables + CPU_STATE_em_table_offset(cpu_state);

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID eventmux_Deallocate_Groups (
 *                         VOID  *params
 *                        )
 *
 * @brief       Free the scratch memory need to support event multiplexing
 *
 * @param       params - pointer to NULL
 *
 * @return      NONE
 *
 * <I>Special Notes:</I>
 *              Free the memory needed to save different group counters
 *              Called via the parallel control mechanism
 */
static VOID eventmux_Deallocate_Groups(PVOID params)
{
	U32 this_cpu;
	CPU_STATE cpu_state;
	U32 dev_idx;
	EVENT_CONFIG ec;

	SEP_DRV_LOG_TRACE_IN("");

	preempt_disable();
	this_cpu = CONTROL_THIS_CPU();
	cpu_state = &pcb[this_cpu];
	dev_idx = core_to_dev_map[this_cpu];
	ec = LWPMU_DEVICE_ec(&devices[dev_idx]);
	preempt_enable();

	if (EVENT_CONFIG_mode(ec) == EM_DISABLED ||
	    EVENT_CONFIG_num_groups(ec) == 1) {
		return;
	}

	CPU_STATE_em_tables(cpu_state) = NULL;

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID eventmux_Timer_Callback_Thread (
 *                        )
 *
 * @brief       Stop all the timer threads and terminate them
 *
 * @param       none
 *
 * @return      NONE
 *
 * <I>Special Notes:</I>
 *              timer routine - The event multiplexing happens here.
 */
static VOID eventmux_Timer_Callback_Thread(
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0)
	struct timer_list *tl
#else
	unsigned long arg
#endif
)
{
	U32 this_cpu;
	CPU_STATE pcpu;
	U32 dev_idx;
	DISPATCH dispatch;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0)
	SEP_DRV_LOG_TRACE_IN("");
#else
	SEP_DRV_LOG_TRACE_IN("Arg: %u.", (U32)arg);
#endif

	preempt_disable();
	this_cpu = CONTROL_THIS_CPU();
	pcpu = &pcb[this_cpu];
	dev_idx = core_to_dev_map[this_cpu];
	dispatch = LWPMU_DEVICE_dispatch(&devices[dev_idx]);
	preempt_enable();

	if (CPU_STATE_em_tables(pcpu) == NULL) {
		SEP_DRV_LOG_ERROR_TRACE_OUT("Em_tables is NULL!");
		return;
	}

	dispatch->swap_group(TRUE);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0)
	mod_timer(CPU_STATE_em_timer(pcpu),
		  jiffies + CPU_STATE_em_timer_delay(pcpu));
#else
	CPU_STATE_em_timer(pcpu)->expires = jiffies + arg;
	add_timer(CPU_STATE_em_timer(pcpu));
#endif

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID eventmux_Prepare_Timer_Threads (
 *                         VOID
 *                        )
 *
 * @brief       Stop all the timer threads and terminate them
 *
 * @param       NONE
 *
 * @return      NONE
 *
 * <I>Special Notes:</I>
 *              Set up the timer threads to prepare for event multiplexing.
 *              Do not start the threads as yet
 */
static VOID eventmux_Prepare_Timer_Threads(PVOID arg)
{
	U32 this_cpu;
	CPU_STATE pcpu;
	U32 dev_idx;
	EVENT_CONFIG ec;

	SEP_DRV_LOG_TRACE_IN("");

	// initialize and set up the timer for all cpus
	// Do not start the timer as yet.
	preempt_disable();
	this_cpu = CONTROL_THIS_CPU();
	pcpu = &pcb[this_cpu];
	dev_idx = core_to_dev_map[this_cpu];
	ec = LWPMU_DEVICE_ec(&devices[dev_idx]);
	preempt_enable();

	if (EVENT_CONFIG_mode(ec) != EM_TIMER_BASED) {
		return;
	}

	CPU_STATE_em_timer(pcpu) = (struct timer_list *)CONTROL_Allocate_Memory(
		sizeof(struct timer_list));

	if (CPU_STATE_em_timer(pcpu) == NULL) {
		SEP_DRV_LOG_ERROR_TRACE_OUT("Pcpu = NULL!");
		return;
	}

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID eventmux_Cancel_Timers (
 *                         VOID
 *                        )
 *
 * @brief       Stop all the timer threads and terminate them
 *
 * @param       NONE
 *
 * @return      NONE
 *
 * <I>Special Notes:</I>
 *              Cancel all the timer threads that have been started
 */
static VOID eventmux_Cancel_Timers(void)
{
	CPU_STATE pcpu;
	S32 i;
	U32 dev_idx;
	EVENT_CONFIG ec;

	SEP_DRV_LOG_TRACE_IN("");

	/*
	 *  Cancel the timer for all active CPUs
	 */
	for (i = 0; i < GLOBAL_STATE_active_cpus(driver_state); i++) {
		pcpu = &pcb[i];
		dev_idx = core_to_dev_map[i];
		ec = LWPMU_DEVICE_ec(&devices[dev_idx]);
		if (EVENT_CONFIG_mode(ec) != EM_TIMER_BASED) {
			continue;
		}
		del_timer_sync(CPU_STATE_em_timer(pcpu));
		CPU_STATE_em_timer(pcpu) =
			(struct timer_list *)CONTROL_Free_Memory(
				CPU_STATE_em_timer(pcpu));
	}

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID eventmux_Start_Timers (
 *                         long unsigned arg
 *                        )
 *
 * @brief       Start the timer on a single cpu
 *
 * @param       delay   interval time in jiffies
 *
 * @return      NONE
 *
 * <I>Special Notes:</I>
 *              start the timer on a single cpu
 *              Call from each cpu to get cpu affinity for Timer_Callback_Thread
 */
static VOID eventmux_Start_Timers(PVOID arg)
{
	U32 this_cpu;
	CPU_STATE pcpu;
	U32 dev_idx;
	EVENT_CONFIG ec;
	unsigned long delay;

	SEP_DRV_LOG_TRACE_IN("");

	preempt_disable();
	this_cpu = CONTROL_THIS_CPU();
	pcpu = &pcb[this_cpu];
	dev_idx = core_to_dev_map[this_cpu];
	ec = LWPMU_DEVICE_ec(&devices[dev_idx]);
	preempt_enable();

	if (EVENT_CONFIG_mode(ec) != EM_TIMER_BASED ||
	    EVENT_CONFIG_num_groups(ec) == 1) {
		return;
	}

	/*
	 * notice we want to use group 0's time slice for the initial timer
	 */
	delay = msecs_to_jiffies(EVENT_CONFIG_em_factor(ec));

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0)
	CPU_STATE_em_timer_delay(pcpu) = delay;
	timer_setup(CPU_STATE_em_timer(pcpu), eventmux_Timer_Callback_Thread,
		    0);
	mod_timer(CPU_STATE_em_timer(pcpu),
		  jiffies + CPU_STATE_em_timer_delay(pcpu));
#else
	init_timer(CPU_STATE_em_timer(pcpu));
	CPU_STATE_em_timer(pcpu)->function = eventmux_Timer_Callback_Thread;
	CPU_STATE_em_timer(pcpu)->data = delay;
	CPU_STATE_em_timer(pcpu)->expires = jiffies + delay;
	add_timer(CPU_STATE_em_timer(pcpu));
#endif

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID EVENTMUX_Start (
 *                         VOID
 *                        )
 *
 * @brief       Start the timers and enable all the threads
 *
 * @param       NONE
 *
 * @return      NONE
 *
 * <I>Special Notes:</I>
 *              if event multiplexing has been enabled, set up the time slices and
 *              start the timer threads for all the timers
 */
VOID EVENTMUX_Start(void)
{
	SEP_DRV_LOG_TRACE_IN("");

	/*
	 * Start the timer for all cpus
	 */
	CONTROL_Invoke_Parallel(eventmux_Start_Timers, NULL);

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID EVENTMUX_Initialize (
 *                         VOID
 *                        )
 *
 * @brief       Initialize the event multiplexing module
 *
 * @param       NONE
 *
 * @return      NONE
 *
 * <I>Special Notes:</I>
 *              if event multiplexing has been enabled,
 *              then allocate the memory needed to save and restore all the counter data
 *              set up the timers needed, but do not start them
 */
VOID EVENTMUX_Initialize(void)
{
	S32 size_of_vector;
	S32 cpu_num;
	CPU_STATE pcpu;
	U32 dev_idx;
	EVENT_CONFIG ec;

	SEP_DRV_LOG_TRACE_IN("");

	for (cpu_num = 0; cpu_num < GLOBAL_STATE_num_cpus(driver_state);
	     cpu_num++) {
		pcpu = &pcb[cpu_num];
		dev_idx = core_to_dev_map[cpu_num];
		ec = LWPMU_DEVICE_ec(&devices[dev_idx]);
		if (EVENT_CONFIG_mode(ec) == EM_DISABLED ||
		    EVENT_CONFIG_num_groups(ec) == 1) {
			continue;
		}
		size_of_vector = EVENT_CONFIG_num_groups(ec) *
				 EVENT_CONFIG_max_gp_events(ec) * sizeof(S64);
		CPU_STATE_em_table_offset(pcpu) = em_tables_size;
		em_tables_size += size_of_vector;
	}

	if (em_tables_size) {
		em_tables = CONTROL_Allocate_Memory(em_tables_size);
	}
	CONTROL_Invoke_Parallel(eventmux_Allocate_Groups, NULL);

	CONTROL_Invoke_Parallel(eventmux_Prepare_Timer_Threads,
				(VOID *)(size_t)0);

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID EVENTMUX_Destroy (
 *                         VOID
 *                        )
 *
 * @brief       Clean up the event multiplexing threads
 *
 * @param       NONE
 *
 * @return      NONE
 *
 * <I>Special Notes:</I>
 *              if event multiplexing has been enabled, then stop and cancel all the timers
 *              free up all the memory that is associated with EM
 */
VOID EVENTMUX_Destroy(void)
{
	SEP_DRV_LOG_TRACE_IN("");

	eventmux_Cancel_Timers();

	if (em_tables) {
		em_tables = CONTROL_Free_Memory(em_tables);
		em_tables_size = 0;
	}
	CONTROL_Invoke_Parallel(eventmux_Deallocate_Groups, (VOID *)(size_t)0);

	SEP_DRV_LOG_TRACE_OUT("");
}
