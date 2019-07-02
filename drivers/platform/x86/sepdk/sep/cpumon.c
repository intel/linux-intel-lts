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

/*
 *  CVS_Id="$Id$"
 */

#include "lwpmudrv_defines.h"
#include <linux/version.h>
#include <linux/interrupt.h>
#if defined(DRV_EM64T)
#include <asm/desc.h>
#endif

#include "lwpmudrv_types.h"
#include "lwpmudrv_ecb.h"
#include "apic.h"
#include "lwpmudrv.h"
#include "control.h"
#include "utility.h"
#include "cpumon.h"
#include "pmi.h"
#include "sys_info.h"

#include <linux/ptrace.h>
#include <asm/nmi.h>

#if !defined(DRV_SEP_ACRN_ON)
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0))
#include <linux/notifier.h>
static int cpumon_NMI_Handler(unsigned int cmd, struct pt_regs *regs)
{
	U32 captured_state = GET_DRIVER_STATE();

	if (DRIVER_STATE_IN(captured_state, STATE_BIT_RUNNING |
						    STATE_BIT_PAUSING |
						    STATE_BIT_PREPARE_STOP |
						    STATE_BIT_TERMINATING)) {
		if (captured_state != DRV_STATE_TERMINATING) {
			PMI_Interrupt_Handler(regs);
		}
		return NMI_HANDLED;
	} else {
		return NMI_DONE;
	}
}

#define EBS_NMI_CALLBACK cpumon_NMI_Handler

#else
#include <linux/kdebug.h>
static int cpumon_NMI_Handler(struct notifier_block *self, unsigned long val,
			      void *data)
{
	struct die_args *args = (struct die_args *)data;
	U32 captured_state = GET_DRIVER_STATE();

	if (args) {
		switch (val) {
		case DIE_NMI:
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 38))
		case DIE_NMI_IPI:
#endif
			if (DRIVER_STATE_IN(captured_state,
					    STATE_BIT_RUNNING |
						    STATE_BIT_PAUSING |
						    STATE_BIT_PREPARE_STOP |
						    STATE_BIT_TERMINATING)) {
				if (captured_state != DRV_STATE_TERMINATING) {
					PMI_Interrupt_Handler(args->regs);
				}
				return NOTIFY_STOP;
			}
		}
	}
	return NOTIFY_DONE;
}

static struct notifier_block cpumon_notifier = { .notifier_call =
							 cpumon_NMI_Handler,
						 .next = NULL,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 38))
						 .priority = 2
#else
		.priority = NMI_LOCAL_LOW_PRIOR,
#endif
};
#endif
#endif

static volatile S32 cpuhook_installed;

/*
 * CPU Monitoring Functionality
 */

/*
 * General per-processor initialization
 */
#if defined(DRV_CPU_HOTPLUG)
/* ------------------------------------------------------------------------- */
/*!
 * @fn       DRV_BOOL CPUMON_is_Online_Allowed()
 *
 * @param    None
 *
 * @return   DRV_BOOL TRUE if cpu is allowed to go Online, else FALSE
 *
 * @brief    Checks if the cpu is allowed to go online during the
 * @brief    current driver state
 *
 */
DRV_BOOL CPUMON_is_Online_Allowed(void)
{
	DRV_BOOL is_allowed = FALSE;
#if !defined(DRV_SEP_ACRN_ON)
	U32 cur_driver_state;

	SEP_DRV_LOG_TRACE_IN("");

	cur_driver_state = GET_DRIVER_STATE();

	switch (cur_driver_state) {
	case DRV_STATE_IDLE:
	case DRV_STATE_PAUSED:
	case DRV_STATE_RUNNING:
	case DRV_STATE_PAUSING:
		is_allowed = TRUE;
		break;
	default:
		SEP_DRV_LOG_TRACE(
			"CPU is prohibited to online in driver state %d.",
			cur_driver_state);
		break;
	}
#endif

	SEP_DRV_LOG_TRACE_OUT("Res: %u.", is_allowed);
	return is_allowed;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn       DRV_BOOL CPUMON_is_Offline_Allowed()
 *
 * @param    None
 *
 * @return   DRV_BOOL TRUE if cpu is allowed to go Offline, else FALSE
 *
 * @brief    Checks if the cpu is allowed to go offline during the
 * @brief    current driver state
 *
 */
DRV_BOOL CPUMON_is_Offline_Allowed(void)
{
	DRV_BOOL is_allowed = FALSE;
#if !defined(DRV_SEP_ACRN_ON)
	U32 cur_driver_state;

	SEP_DRV_LOG_TRACE_IN("");

	cur_driver_state = GET_DRIVER_STATE();

	switch (cur_driver_state) {
	case DRV_STATE_PAUSED:
	case DRV_STATE_RUNNING:
	case DRV_STATE_PAUSING:
		is_allowed = TRUE;
		break;
	default:
		SEP_DRV_LOG_TRACE(
			"CPU is prohibited to offline in driver state %d.",
			cur_driver_state);
		break;
	}
#endif

	SEP_DRV_LOG_TRACE_OUT("Res: %u.", is_allowed);
	return is_allowed;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn       VOID CPUMON_Online_Cpu(
 *               PVOID param)
 *
 * @param    PVOID param
 *
 * @return   None
 *
 * @brief    Sets a cpu online, initialize APIC on it,
 * @brief    Build the sys_info for this cpu
 *
 */
VOID CPUMON_Online_Cpu(PVOID param)
{
	S32 this_cpu;
	CPU_STATE pcpu;

	SEP_DRV_LOG_TRACE_IN("Dummy param: %p.", param);

	if (param == NULL) {
		preempt_disable();
		this_cpu = CONTROL_THIS_CPU();
		preempt_enable();
	} else {
		this_cpu = *(S32 *)param;
	}
	pcpu = &pcb[this_cpu];
	if (pcpu == NULL) {
		SEP_DRV_LOG_WARNING_TRACE_OUT("Unable to set CPU %d online!",
					      this_cpu);
		return;
	}
	SEP_DRV_LOG_INIT("Setting CPU %d online, PCPU = %p.", this_cpu, pcpu);
	CPU_STATE_offlined(pcpu) = FALSE;
	CPU_STATE_accept_interrupt(pcpu) = 1;
	CPU_STATE_initial_mask(pcpu) = 1;
	CPU_STATE_group_swap(pcpu) = 1;
	APIC_Init(NULL);
	APIC_Install_Interrupt_Handler(NULL);

	SYS_INFO_Build_Cpu(NULL);

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn       VOID CPUMON_Offline_Cpu(
 *               PVOID param)
 *
 * @param    PVOID parm
 *
 * @return   None
 *
 * @brief    Sets a cpu offline
 *
 */
VOID CPUMON_Offline_Cpu(PVOID param)
{
	S32 this_cpu;
	CPU_STATE pcpu;

	SEP_DRV_LOG_TRACE_IN("Dummy parm: %p.", parm);

	if (param == NULL) {
		preempt_disable();
		this_cpu = CONTROL_THIS_CPU();
		preempt_enable();
	} else {
		this_cpu = *(S32 *)param;
	}
	pcpu = &pcb[this_cpu];

	if (pcpu == NULL) {
		SEP_DRV_LOG_WARNING_TRACE_OUT("Unable to set CPU %d offline.",
					      this_cpu);
		return;
	}
	SEP_DRV_LOG_INIT("Setting CPU %d offline.", this_cpu);
	CPU_STATE_offlined(pcpu) = TRUE;

	SEP_DRV_LOG_TRACE_OUT("");
}
#endif

/* ------------------------------------------------------------------------- */
/*!
 * @fn extern void CPUMON_Install_Cpuhooks(void)
 *
 * @param    None
 *
 * @return   None     No return needed
 *
 * @brief  set up the interrupt handler (on a per-processor basis)
 * @brief  Initialize the APIC in two phases (current CPU, then others)
 *
 */
VOID CPUMON_Install_Cpuhooks(void)
{
#if !defined(DRV_SEP_ACRN_ON)
	S32 me = 0;

	SEP_DRV_LOG_TRACE_IN("");

	if (cpuhook_installed) {
		SEP_DRV_LOG_WARNING_TRACE_OUT("Cpuhook already installed.");
		return;
	}

	CONTROL_Invoke_Parallel(APIC_Init, NULL);
	CONTROL_Invoke_Parallel(APIC_Install_Interrupt_Handler,
				(PVOID)(size_t)me);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0))
	register_nmi_handler(NMI_LOCAL, EBS_NMI_CALLBACK, 0, "sep_pmi");
#else
	register_die_notifier(&cpumon_notifier);
#endif

	cpuhook_installed = 1;

	SEP_DRV_LOG_TRACE_OUT("");
#endif
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn extern void CPUMON_Remove_Cpuhools(void)
 *
 * @param    None
 *
 * @return   None     No return needed
 *
 * @brief  De-Initialize the APIC in phases
 * @brief  clean up the interrupt handler (on a per-processor basis)
 *
 */
VOID CPUMON_Remove_Cpuhooks(void)
{
	SEP_DRV_LOG_TRACE_IN("");

#if !defined(DRV_SEP_ACRN_ON)
	CONTROL_Invoke_Parallel(APIC_Restore_LVTPC, NULL);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0))
	unregister_nmi_handler(NMI_LOCAL, "sep_pmi");
#else
	unregister_die_notifier(&cpumon_notifier);
#endif

	cpuhook_installed = 0;
#endif

	SEP_DRV_LOG_TRACE_OUT("");
}
