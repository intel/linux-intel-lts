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
#include <linux/wait.h>
#include <linux/fs.h>

#include "lwpmudrv_types.h"
#include "lwpmudrv_ecb.h"
#include "lwpmudrv_struct.h"

#include "lwpmudrv.h"
#include "utility.h"
#include "control.h"
#include "output.h"
#include "perfver4.h"
#include "ecb_iterators.h"
#include "pebs.h"
#include "apic.h"

extern U64 *read_counter_info;
extern DRV_CONFIG drv_cfg;
extern U64 *interrupt_counts;
extern DRV_SETUP_INFO_NODE req_drv_setup_info;
extern EMON_BUFFER_DRIVER_HELPER emon_buffer_driver_helper;
static U64 perf_metrics_counter_reload_value;

typedef struct SADDR_S {
	S64 addr : PERFVER4_LBR_DATA_BITS;
} SADDR;

static U32 restore_reg_addr[3];

#define SADDR_addr(x) ((x).addr)
#define MSR_ENERGY_MULTIPLIER 0x606 // Energy Multiplier MSR

#define IS_FIXED_CTR_ENABLED(ia32_perf_global_ctrl_reg_val)                    \
	((ia32_perf_global_ctrl_reg_val)&0x700000000ULL)
#define IS_FOUR_FIXED_CTR_ENABLED(ia32_perf_global_ctrl_reg_val)               \
	((ia32_perf_global_ctrl_reg_val)&0xF00000000ULL)
#define IS_PMC_PEBS_ENABLED_GP(ia32_perf_global_ctrl_reg_val,                  \
			       ia32_pebs_enable_reg_val)                       \
	(((ia32_perf_global_ctrl_reg_val)&0xfULL) ==                           \
	 ((ia32_pebs_enable_reg_val)&0xfULL))
#define IS_PMC_PEBS_ENABLED_FP_AND_GP(ia32_perf_global_ctrl_reg_val,           \
				      ia32_pebs_enable_reg_val)                \
	(((ia32_perf_global_ctrl_reg_val)&0xf000000ffULL) ==                   \
	 ((ia32_pebs_enable_reg_val)&0xf000000ffULL))

#define DISABLE_FRZ_ON_PMI(ia32_debug_ctrl_reg_val)                            \
	(0xefff & (ia32_debug_ctrl_reg_val))
/* ------------------------------------------------------------------------- */
/*!
 * @fn void perfver4_Write_PMU(param)
 *
 * @param    param    dummy parameter which is not used
 *
 * @return   None     No return needed
 *
 * @brief    Initial set up of the PMU registers
 *
 * <I>Special Notes</I>
 *         Initial write of PMU registers.
 *         Walk through the enties and write the value of the register accordingly.
 *         Assumption:  For CCCR registers the enable bit is set to value 0.
 *         When current_group = 0, then this is the first time this routine is called,
 *         initialize the locks and set up EM tables.
 */
static VOID perfver4_Write_PMU(VOID *param)
{
	U32 this_cpu;
	CPU_STATE pcpu;
	ECB pecb;
	U32 dev_idx;
	U32 cur_grp;
	EVENT_CONFIG ec;
	DISPATCH dispatch;
	DEV_CONFIG pcfg;
#if defined(DRV_SEP_ACRN_ON)
	struct profiling_pmi_config *pmi_config;
	U32 index;
	S32 msr_idx;
#else
	U32 counter_index;
#endif

	SEP_DRV_LOG_TRACE_IN("Dummy param: %p.", param);

	if (param == NULL) {
		preempt_disable();
		this_cpu = CONTROL_THIS_CPU();
		preempt_enable();
	} else {
		this_cpu = *(S32 *)param;
	}

	pcpu = &pcb[this_cpu];
	dev_idx = core_to_dev_map[this_cpu];
	cur_grp = CPU_STATE_current_group(pcpu);
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[dev_idx])[cur_grp];
	ec = LWPMU_DEVICE_ec(&devices[dev_idx]);
	dispatch = LWPMU_DEVICE_dispatch(&devices[dev_idx]);
	pcfg = LWPMU_DEVICE_pcfg(&devices[dev_idx]);

	if (!pecb) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!pecb).");
		return;
	}

#if !defined(DRV_SEP_ACRN_ON)
	counter_index = 0;
	if (CPU_STATE_current_group(pcpu) == 0) {
		if (EVENT_CONFIG_mode(ec) != EM_DISABLED) {
			U32 index;
			U32 st_index;
			U32 j;

			/* Save all the initialization values away into an array for Event Multiplexing. */
			for (j = 0; j < EVENT_CONFIG_num_groups(ec); j++) {
				CPU_STATE_current_group(pcpu) = j;
				st_index = CPU_STATE_current_group(pcpu) *
					   EVENT_CONFIG_max_gp_events(ec);
				FOR_EACH_REG_CORE_OPERATION(
					pecb, i, PMU_OPERATION_DATA_GP)
				{
					index = st_index + i -
						ECB_operations_register_start(
							pecb,
							PMU_OPERATION_DATA_GP);
					CPU_STATE_em_tables(pcpu)[index] =
						ECB_entries_reg_value(pecb, i);
				}
				END_FOR_EACH_REG_CORE_OPERATION;
			}
			/* Reset the current group to the very first one. */
			CPU_STATE_current_group(pcpu) =
				this_cpu % EVENT_CONFIG_num_groups(ec);
		}
	}

	if (dispatch->hw_errata) {
		dispatch->hw_errata();
	}

	/* Clear outstanding frozen bits */
	SYS_Write_MSR(IA32_PERF_GLOBAL_OVF_CTRL, PERFVER4_FROZEN_BIT_MASK);

	FOR_EACH_REG_CORE_OPERATION(pecb, i, PMU_OPERATION_ALL_REG)
	{
		/*
		 * Writing the GLOBAL Control register enables the PMU to start counting.
		 * So write 0 into the register to prevent any counting from starting.
		 */
		if (i == ECB_SECTION_REG_INDEX(pecb, GLOBAL_CTRL_REG_INDEX,
					       PMU_OPERATION_GLOBAL_REGS)) {
			SYS_Write_MSR(ECB_entries_reg_id(pecb, i), 0LL);
			continue;
		}
		/*
		 *  PEBS is enabled for this collection session
		 */
		if (DRV_SETUP_INFO_pebs_accessible(&req_drv_setup_info) &&
		    i == ECB_SECTION_REG_INDEX(pecb, PEBS_ENABLE_REG_INDEX,
					       PMU_OPERATION_GLOBAL_REGS) &&
		    ECB_entries_reg_value(pecb, i)) {
			SYS_Write_MSR(ECB_entries_reg_id(pecb, i), 0LL);
			continue;
		}

		if (DEV_CONFIG_pebs_mode(pcfg) &&
		    (ECB_entries_precise_get(pecb, i) == 1)) {
			if (ECB_entries_fixed_reg_get(pecb, i)) {
				counter_index = (ECB_entries_reg_id(pecb, i) -
						 IA32_FIXED_CTR0 + 8);
			} else {
				counter_index = (ECB_entries_reg_id(pecb, i) -
						 IA32_PMC0);
			}
			PEBS_Reset_Counter(this_cpu, counter_index,
					   ECB_entries_reg_value(pecb, i));
		}

		SYS_Write_MSR(ECB_entries_reg_id(pecb, i),
			      ECB_entries_reg_value(pecb, i));
#if defined(MYDEBUG)
		{
			U64 val = SYS_Read_MSR(ECB_entries_reg_id(pecb, i));
			SEP_DRV_LOG_TRACE(
				"Write reg 0x%x --- value 0x%llx -- read 0x%llx.",
				ECB_entries_reg_id(pecb, i),
				ECB_entries_reg_value(pecb, i), val);
		}
#endif
	}
	END_FOR_EACH_REG_CORE_OPERATION;
#else
	pmi_config = (struct profiling_pmi_config *)CONTROL_Allocate_Memory(
		sizeof(struct profiling_pmi_config));
	if (pmi_config == NULL) {
		SEP_PRINT_ERROR("pmi_config memory allocation failed\n");
		return;
	}
	memset(pmi_config, 0, sizeof(struct profiling_pmi_config));

	msr_idx = 0;
	pmi_config->num_groups = 1;

	pmi_config->initial_list[0][msr_idx].msr_id = IA32_PERF_GLOBAL_CTRL;
	pmi_config->initial_list[0][msr_idx].op_type = MSR_OP_WRITE;
	pmi_config->initial_list[0][msr_idx].reg_type = PMU_MSR_CCCR;
	pmi_config->initial_list[0][msr_idx].value = 0x0;
	pmi_config->initial_list[0][msr_idx].param = 0x0;
	msr_idx++;

	FOR_EACH_CCCR_REG_CPU(pecb, i, this_cpu)
	{
		if ((ECB_entries_reg_id(pecb, i) == IA32_PERF_GLOBAL_CTRL) ||
		    (ECB_entries_reg_id(pecb, i) == IA32_PEBS_ENABLE)) {
			continue;
		}

		pmi_config->initial_list[0][msr_idx].msr_id =
			ECB_entries_reg_id(pecb, i);
		pmi_config->initial_list[0][msr_idx].op_type = MSR_OP_WRITE;
		pmi_config->initial_list[0][msr_idx].reg_type = PMU_MSR_CCCR;
		pmi_config->initial_list[0][msr_idx].value =
			ECB_entries_reg_value(pecb, i);
		pmi_config->initial_list[0][msr_idx].param = 0x0;
		msr_idx++;
		BUG_ON(msr_idx >= MAX_MSR_LIST_NUM);
	}
	END_FOR_EACH_CCCR_REG_CPU;

	FOR_EACH_ESCR_REG_CPU(pecb, i, this_cpu)
	{
		pmi_config->initial_list[0][msr_idx].msr_id =
			ECB_entries_reg_id(pecb, i);
		pmi_config->initial_list[0][msr_idx].op_type = MSR_OP_WRITE;
		pmi_config->initial_list[0][msr_idx].reg_type = PMU_MSR_ESCR;
		pmi_config->initial_list[0][msr_idx].value =
			ECB_entries_reg_value(pecb, i);
		pmi_config->initial_list[0][msr_idx].param = 0x0;
		msr_idx++;
		BUG_ON(msr_idx >= MAX_MSR_LIST_NUM);
	}
	END_FOR_EACH_ESCR_REG_CPU;

	FOR_EACH_DATA_REG_CPU(pecb, i, this_cpu)
	{
		if (ECB_entries_fixed_reg_get(pecb, i)) {
			index = ECB_entries_reg_id(pecb, i) - IA32_FIXED_CTR0 +
				0x20;
		} else if (ECB_entries_is_gp_reg_get(pecb, i)) {
			index = ECB_entries_reg_id(pecb, i) - IA32_PMC0;
		} else {
			continue;
		}
		pmi_config->initial_list[0][msr_idx].msr_id =
			ECB_entries_reg_id(pecb, i);
		pmi_config->initial_list[0][msr_idx].op_type = MSR_OP_WRITE;
		pmi_config->initial_list[0][msr_idx].reg_type = PMU_MSR_DATA;
		pmi_config->initial_list[0][msr_idx].value =
			ECB_entries_reg_value(pecb, i);
		pmi_config->initial_list[0][msr_idx].param = index;
		msr_idx++;
		BUG_ON(msr_idx >= MAX_MSR_LIST_NUM);
	}
	END_FOR_EACH_DATA_REG_CPU;
	pmi_config->initial_list[0][msr_idx].msr_id = -1;

	FOR_EACH_CCCR_REG_CPU(pecb, i, this_cpu)
	{
		if (ECB_entries_reg_id(pecb, i) == IA32_PERF_GLOBAL_CTRL) {
			pmi_config->start_list[0][0].msr_id =
				IA32_PERF_GLOBAL_CTRL;
			pmi_config->start_list[0][0].op_type = MSR_OP_WRITE;
			pmi_config->start_list[0][0].reg_type = PMU_MSR_CCCR;
			pmi_config->start_list[0][0].value =
				ECB_entries_reg_value(pecb, i);
			pmi_config->start_list[0][0].param = 0x0;
			pmi_config->start_list[0][1].msr_id = -1;
			break;
		}
	}
	END_FOR_EACH_CCCR_REG_CPU;

	pmi_config->stop_list[0][0].msr_id = IA32_PERF_GLOBAL_CTRL;
	pmi_config->stop_list[0][0].op_type = MSR_OP_WRITE;
	pmi_config->stop_list[0][0].reg_type = PMU_MSR_CCCR;
	pmi_config->stop_list[0][0].value = 0x0;
	pmi_config->stop_list[0][0].param = 0x0;
	pmi_config->stop_list[0][1].msr_id = -1;

	if (DRV_CONFIG_counting_mode(drv_cfg) == FALSE) {
		pmi_config->entry_list[0][0].msr_id = IA32_PERF_GLOBAL_CTRL;
		pmi_config->entry_list[0][0].op_type = MSR_OP_WRITE;
		pmi_config->entry_list[0][0].reg_type = PMU_MSR_CCCR;
		pmi_config->entry_list[0][0].value = 0x0;
		pmi_config->entry_list[0][0].param = 0x0;
		pmi_config->entry_list[0][1].msr_id = -1;

		msr_idx = 0;
		FOR_EACH_CCCR_REG_CPU(pecb, i, this_cpu)
		{
			if ((ECB_entries_reg_id(pecb, i) ==
			     IA32_PERF_GLOBAL_CTRL) ||
			    (ECB_entries_reg_id(pecb, i) == IA32_PEBS_ENABLE)) {
				continue;
			}

			pmi_config->exit_list[0][msr_idx].msr_id =
				ECB_entries_reg_id(pecb, i);
			pmi_config->exit_list[0][msr_idx].op_type =
				MSR_OP_WRITE;
			pmi_config->exit_list[0][msr_idx].reg_type =
				PMU_MSR_CCCR;
			pmi_config->exit_list[0][msr_idx].value =
				ECB_entries_reg_value(pecb, i);
			pmi_config->exit_list[0][msr_idx].param = 0x0;
			msr_idx++;
			BUG_ON(msr_idx >= MAX_MSR_LIST_NUM);
		}
		END_FOR_EACH_CCCR_REG_CPU;

		FOR_EACH_ESCR_REG_CPU(pecb, i, this_cpu)
		{
			pmi_config->exit_list[0][msr_idx].msr_id =
				ECB_entries_reg_id(pecb, i);
			pmi_config->exit_list[0][msr_idx].op_type =
				MSR_OP_WRITE;
			pmi_config->exit_list[0][msr_idx].reg_type =
				PMU_MSR_ESCR;
			pmi_config->exit_list[0][msr_idx].value =
				ECB_entries_reg_value(pecb, i);
			pmi_config->exit_list[0][msr_idx].param = 0x0;
			msr_idx++;
			BUG_ON(msr_idx >= MAX_MSR_LIST_NUM);
		}
		END_FOR_EACH_ESCR_REG_CPU;

		FOR_EACH_DATA_REG_CPU(pecb, i, this_cpu)
		{
			if (ECB_entries_fixed_reg_get(pecb, i)) {
				index = ECB_entries_reg_id(pecb, i) -
					IA32_FIXED_CTR0 + 0x20;
			} else if (ECB_entries_is_gp_reg_get(pecb, i)) {
				index = ECB_entries_reg_id(pecb, i) - IA32_PMC0;
			} else {
				continue;
			}
			pmi_config->exit_list[0][msr_idx].msr_id =
				ECB_entries_reg_id(pecb, i);
			pmi_config->exit_list[0][msr_idx].op_type =
				MSR_OP_WRITE;
			pmi_config->exit_list[0][msr_idx].reg_type =
				PMU_MSR_DATA;
			pmi_config->exit_list[0][msr_idx].value =
				ECB_entries_reg_value(pecb, i);
			pmi_config->exit_list[0][msr_idx].param = index;
			msr_idx++;
			BUG_ON(msr_idx >= MAX_MSR_LIST_NUM);
		}
		END_FOR_EACH_DATA_REG_CPU;

		FOR_EACH_CCCR_REG_CPU(pecb, i, this_cpu)
		{
			if (ECB_entries_reg_id(pecb, i) ==
			    IA32_PERF_GLOBAL_CTRL) {
				pmi_config->exit_list[0][msr_idx].msr_id =
					IA32_PERF_GLOBAL_CTRL;
				pmi_config->exit_list[0][msr_idx].op_type =
					MSR_OP_WRITE;
				pmi_config->exit_list[0][msr_idx].reg_type =
					PMU_MSR_CCCR;
				pmi_config->exit_list[0][msr_idx].value =
					ECB_entries_reg_value(pecb, i);
				pmi_config->exit_list[0][msr_idx].param = 0x0;
				msr_idx++;
				BUG_ON(msr_idx >= MAX_MSR_LIST_NUM);
				break;
			}
		}
		END_FOR_EACH_CCCR_REG_CPU;
		pmi_config->exit_list[0][msr_idx].msr_id = -1;
	}

	BUG_ON(!virt_addr_valid(pmi_config));

	if (acrn_hypercall2(HC_PROFILING_OPS, PROFILING_CONFIG_PMI,
			virt_to_phys(pmi_config)) != OS_SUCCESS) {
		SEP_DRV_LOG_ERROR(
			"[ACRN][HC:CONFIG_PMI][%s]: Failed to write PMI config info",
			__func__);
	}
	pmi_config = CONTROL_Free_Memory(pmi_config);
#endif

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn void perfver4_Disable_PMU(param)
 *
 * @param    param    dummy parameter which is not used
 *
 * @return   None     No return needed
 *
 * @brief    Zero out the global control register.  This automatically disables the PMU counters.
 *
 */
static VOID perfver4_Disable_PMU(PVOID param)
{
#if !defined(DRV_SEP_ACRN_ON)
	U32 this_cpu;
	CPU_STATE pcpu;
	ECB pecb;
	U32 dev_idx;
	U32 cur_grp;
	DEV_CONFIG pcfg;

	SEP_DRV_LOG_TRACE_IN("Dummy param: %p.", param);

	this_cpu = CONTROL_THIS_CPU();
	pcpu = &pcb[this_cpu];
	dev_idx = core_to_dev_map[this_cpu];
	cur_grp = CPU_STATE_current_group(pcpu);
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[dev_idx])[cur_grp];
	pcfg = LWPMU_DEVICE_pcfg(&devices[dev_idx]);

	if (!pecb) {
		// no programming for this device for this group
		SEP_DRV_LOG_TRACE_OUT("Early exit (!pecb).");
		return;
	}

	if (GET_DRIVER_STATE() != DRV_STATE_RUNNING) {
		SEP_DRV_LOG_TRACE("Driver state = %d.", GET_DRIVER_STATE());
		SYS_Write_MSR(ECB_entries_reg_id(
				      pecb, ECB_SECTION_REG_INDEX(
						    pecb, GLOBAL_CTRL_REG_INDEX,
						    PMU_OPERATION_GLOBAL_REGS)),
			      0LL);
		if (DEV_CONFIG_pebs_mode(pcfg)) {
			SYS_Write_MSR(
				ECB_entries_reg_id(
					pecb,
					ECB_SECTION_REG_INDEX(
						pecb, PEBS_ENABLE_REG_INDEX,
						PMU_OPERATION_GLOBAL_REGS)),
				0LL);
		}
	}

	SEP_DRV_LOG_TRACE_OUT("");
#endif
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn void perfver4_Enable_PMU(param)
 *
 * @param    param    dummy parameter which is not used
 *
 * @return   None     No return needed
 *
 * @brief    Set the enable bit for all the Control registers
 *
 */
static VOID perfver4_Enable_PMU(PVOID param)
{
#if !defined(DRV_SEP_ACRN_ON)
	/*
	 * Get the value from the event block
	 *   0 == location of the global control reg for this block.
	 *   Generalize this location awareness when possible
	 */
	U32 this_cpu;
	CPU_STATE pcpu;
	ECB pecb;
	U32 dev_idx;
	U32 cur_grp;
	DEV_CONFIG pcfg;
	U64 global_control_val;
	U64 pebs_enable_val;
	DRV_BOOL multi_pebs_enabled;

	SEP_DRV_LOG_TRACE_IN("Dummy param: %p.", param);

	this_cpu = CONTROL_THIS_CPU();
	pcpu = &pcb[this_cpu];
	dev_idx = core_to_dev_map[this_cpu];
	cur_grp = CPU_STATE_current_group(pcpu);
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[dev_idx])[cur_grp];
	pcfg = LWPMU_DEVICE_pcfg(&devices[dev_idx]);

	if (!pecb) {
		// no programming for this device for this group
		SEP_DRV_LOG_TRACE_OUT("Early exit (!pecb).");
		return;
	}

	if (KVM_guest_mode) {
		SYS_Write_MSR(ECB_entries_reg_id(
				      pecb, ECB_SECTION_REG_INDEX(
						    pecb, GLOBAL_CTRL_REG_INDEX,
						    PMU_OPERATION_GLOBAL_REGS)),
			      0LL);
	}
	if (GET_DRIVER_STATE() == DRV_STATE_RUNNING) {
		APIC_Enable_Pmi();

		/* Clear outstanding frozen bits */
		SYS_Write_MSR(ECB_entries_reg_id(
				      pecb,
				      ECB_SECTION_REG_INDEX(
					      pecb, GLOBAL_OVF_CTRL_REG_INDEX,
					      PMU_OPERATION_GLOBAL_REGS)),
			      PERFVER4_FROZEN_BIT_MASK);

		if (CPU_STATE_reset_mask(pcpu)) {
			SEP_DRV_LOG_TRACE("Overflow reset mask %llx.",
					  CPU_STATE_reset_mask(pcpu));
			// Reinitialize the global overflow control register
			SYS_Write_MSR(
				ECB_entries_reg_id(
					pecb,
					ECB_SECTION_REG_INDEX(
						pecb, GLOBAL_CTRL_REG_INDEX,
						PMU_OPERATION_GLOBAL_REGS)),
				ECB_entries_reg_value(
					pecb,
					ECB_SECTION_REG_INDEX(
						pecb, GLOBAL_CTRL_REG_INDEX,
						PMU_OPERATION_GLOBAL_REGS)));
			SYS_Write_MSR(
				ECB_entries_reg_id(
					pecb,
					ECB_SECTION_REG_INDEX(
						pecb, DEBUG_CTRL_REG_INDEX,
						PMU_OPERATION_GLOBAL_REGS)),
				ECB_entries_reg_value(
					pecb,
					ECB_SECTION_REG_INDEX(
						pecb, DEBUG_CTRL_REG_INDEX,
						PMU_OPERATION_GLOBAL_REGS)));
			CPU_STATE_reset_mask(pcpu) = 0LL;
		}
		if (CPU_STATE_group_swap(pcpu)) {
			CPU_STATE_group_swap(pcpu) = 0;
			SYS_Write_MSR(
				ECB_entries_reg_id(
					pecb,
					ECB_SECTION_REG_INDEX(
						pecb, GLOBAL_CTRL_REG_INDEX,
						PMU_OPERATION_GLOBAL_REGS)),
				ECB_entries_reg_value(
					pecb,
					ECB_SECTION_REG_INDEX(
						pecb, GLOBAL_CTRL_REG_INDEX,
						PMU_OPERATION_GLOBAL_REGS)));
			if (DEV_CONFIG_pebs_mode(pcfg) ||
			    DEV_CONFIG_latency_capture(pcfg)) {
				SYS_Write_MSR(
					ECB_entries_reg_id(
						pecb,
						ECB_SECTION_REG_INDEX(
							pecb,
							PEBS_ENABLE_REG_INDEX,
							PMU_OPERATION_GLOBAL_REGS)),
					ECB_entries_reg_value(
						pecb,
						ECB_SECTION_REG_INDEX(
							pecb,
							PEBS_ENABLE_REG_INDEX,
							PMU_OPERATION_GLOBAL_REGS)));
			}
			SYS_Write_MSR(
				ECB_entries_reg_id(
					pecb,
					ECB_SECTION_REG_INDEX(
						pecb, DEBUG_CTRL_REG_INDEX,
						PMU_OPERATION_GLOBAL_REGS)),
				ECB_entries_reg_value(
					pecb,
					ECB_SECTION_REG_INDEX(
						pecb, DEBUG_CTRL_REG_INDEX,
						PMU_OPERATION_GLOBAL_REGS)));
#if defined(MYDEBUG)
			{
				U64 val;
				val = SYS_Read_MSR(ECB_entries_reg_id(
					pecb,
					ECB_SECTION_REG_INDEX(
						pecb, GLOBAL_CTRL_REG_INDEX,
						PMU_OPERATION_GLOBAL_REGS)));
				SEP_DRV_LOG_TRACE(
					"Write reg 0x%x--- read 0x%llx.",
					ECB_entries_reg_id(pecb, 0), val);
			}
#endif
		}

		multi_pebs_enabled = (DEV_CONFIG_pebs_mode(pcfg) &&
				      (DEV_CONFIG_pebs_record_num(pcfg) > 1) &&
				      (DRV_SETUP_INFO_page_table_isolation(
					       &req_drv_setup_info) ==
				       DRV_SETUP_INFO_PTI_DISABLED));

		// FIXME: workaround for sampling both pebs event and non-pebs event
		//        with pebs buffer size > 1
		if (multi_pebs_enabled) {
			global_control_val = SYS_Read_MSR(ECB_entries_reg_id(
				pecb, ECB_SECTION_REG_INDEX(
					      pecb, GLOBAL_CTRL_REG_INDEX,
					      PMU_OPERATION_GLOBAL_REGS)));
			pebs_enable_val = SYS_Read_MSR(ECB_entries_reg_id(
				pecb, ECB_SECTION_REG_INDEX(
					      pecb, PEBS_ENABLE_REG_INDEX,
					      PMU_OPERATION_GLOBAL_REGS)));
			if (IS_FIXED_CTR_ENABLED(global_control_val) ||
			    !IS_PMC_PEBS_ENABLED_GP(global_control_val,
						    pebs_enable_val)) {
				SEP_DRV_LOG_TRACE(
					"Global_control_val = 0x%llx pebs_enable_val = 0x%llx.",
					global_control_val, pebs_enable_val);
				SYS_Write_MSR(
					ECB_entries_reg_id(
						pecb,
						ECB_SECTION_REG_INDEX(
							pecb,
							DEBUG_CTRL_REG_INDEX,
							PMU_OPERATION_GLOBAL_REGS)),
					DISABLE_FRZ_ON_PMI(ECB_entries_reg_value(
						pecb,
						ECB_SECTION_REG_INDEX(
							pecb,
							DEBUG_CTRL_REG_INDEX,
							PMU_OPERATION_GLOBAL_REGS))));
			}
		}
	}
	SEP_DRV_LOG_TRACE("Reenabled PMU with value 0x%llx.",
			  ECB_entries_reg_value(pecb, 0));

	SEP_DRV_LOG_TRACE_OUT("");
#endif
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn perfver4_Read_PMU_Data(param)
 *
 * @param    param    dummy parameter which is not used
 *
 * @return   None     No return needed
 *
 * @brief    Read all the data MSR's into a buffer.  Called by the interrupt handler.
 *
 */
static void perfver4_Read_PMU_Data(PVOID param)
{
	U32 j;
	U64 *buffer = read_counter_info;
	U32 this_cpu;
	CPU_STATE pcpu;
	ECB pecb;
	U32 dev_idx;
	U32 cur_grp;
#if defined(DRV_SEP_ACRN_ON)
	S32 start_index, cpu_idx, msr_idx;
	struct profiling_msr_ops_list *msr_list;
#endif

	SEP_DRV_LOG_TRACE_IN("Dummy param: %p.", param);

	if (param == NULL) {
		preempt_disable();
		this_cpu = CONTROL_THIS_CPU();
		preempt_enable();
	} else {
		this_cpu = *(S32 *)param;
	}

#if !defined(DRV_SEP_ACRN_ON)
	pcpu = &pcb[this_cpu];
	dev_idx = core_to_dev_map[this_cpu];
	cur_grp = CPU_STATE_current_group(pcpu);
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[dev_idx])[cur_grp];

	if (!pecb) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!pecb).");
		return;
	}

	SEP_DRV_LOG_TRACE("PMU control_data 0x%p, buffer 0x%p.",
			  LWPMU_DEVICE_PMU_register_data(&devices[dev_idx]),
			  buffer);
	FOR_EACH_REG_CORE_OPERATION(pecb, i, PMU_OPERATION_DATA_ALL)
	{
		j = EMON_BUFFER_CORE_EVENT_OFFSET(
			EMON_BUFFER_DRIVER_HELPER_core_index_to_thread_offset_map(
				emon_buffer_driver_helper)[this_cpu],
			ECB_entries_core_event_id(pecb, i));

		buffer[j] = SYS_Read_MSR(ECB_entries_reg_id(pecb, i));
		SEP_DRV_LOG_TRACE("j=%u, value=%llu, cpu=%u, event_id=%u", j,
				  buffer[j], this_cpu,
				  ECB_entries_core_event_id(pecb, i));
	}
	END_FOR_EACH_REG_CORE_OPERATION;
#else
	if (DRV_CONFIG_counting_mode(drv_cfg) == TRUE) {
		msr_list = (struct profiling_msr_ops_list *)
			CONTROL_Allocate_Memory(
				GLOBAL_STATE_num_cpus(driver_state) *
				sizeof(struct profiling_msr_ops_list));
		memset(msr_list, 0,
		       GLOBAL_STATE_num_cpus(driver_state) *
			       sizeof(struct profiling_msr_ops_list));
		for (cpu_idx = 0; cpu_idx < GLOBAL_STATE_num_cpus(driver_state);
		     cpu_idx++) {
			pcpu = &pcb[cpu_idx];
			dev_idx = core_to_dev_map[cpu_idx];
			cur_grp = CPU_STATE_current_group(pcpu);
			pecb = LWPMU_DEVICE_PMU_register_data(
				&devices[dev_idx])[cur_grp];

			if (!pecb) {
				continue;
			}

			msr_idx = 0;
			FOR_EACH_DATA_REG_CPU(pecb, i, cpu_idx)
			{
				msr_list[cpu_idx].entries[msr_idx].msr_id =
					ECB_entries_reg_id(pecb, i);
				msr_list[cpu_idx].entries[msr_idx].op_type =
					MSR_OP_READ_CLEAR;
				msr_list[cpu_idx].entries[msr_idx].value = 0LL;
				msr_idx++;
			}
			END_FOR_EACH_DATA_REG_CPU;
			msr_list[cpu_idx].num_entries = msr_idx;
			msr_list[cpu_idx].msr_op_state = MSR_OP_REQUESTED;
		}

		BUG_ON(!virt_addr_valid(msr_list));

		if (acrn_hypercall2(HC_PROFILING_OPS, PROFILING_MSR_OPS,
				virt_to_phys(msr_list)) != OS_SUCCESS) {
			msr_list = CONTROL_Free_Memory(msr_list);
			SEP_DRV_LOG_ERROR_FLOW_OUT(
			"[ACRN][HC:MSR_OPS][%s]: MSR operation failed",
			__func__);
			return;
		}
		for (cpu_idx = 0; cpu_idx < GLOBAL_STATE_num_cpus(driver_state);
		     cpu_idx++) {
			pcpu = &pcb[cpu_idx];
			dev_idx = core_to_dev_map[cpu_idx];
			cur_grp = CPU_STATE_current_group(pcpu);
			pecb = LWPMU_DEVICE_PMU_register_data(
				&devices[dev_idx])[cur_grp];

			if (!pecb) {
				continue;
			}

			start_index = ECB_num_events(pecb) * cpu_idx;
			msr_idx = 0;
			FOR_EACH_DATA_REG_CPU(pecb, i, cpu_idx)
			{
				j = start_index +
				    ECB_entries_event_id_index(pecb, i);
				buffer[j] =
					msr_list[cpu_idx].entries[msr_idx].value;
				msr_idx++;
			}
			END_FOR_EACH_DATA_REG_CPU;
		}

		msr_list = CONTROL_Free_Memory(msr_list);
	}
#endif

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn void perfver4_Check_Overflow(masks)
 *
 * @param    masks    the mask structure to populate
 *
 * @return   None     No return needed
 *
 * @brief  Called by the data processing method to figure out which registers have overflowed.
 *
 */
static void perfver4_Check_Overflow(DRV_MASKS masks)
{
	U32 index;
	U64 overflow_status = 0;
	U32 this_cpu;
	BUFFER_DESC bd;
	CPU_STATE pcpu;
	ECB pecb;
	U32 dev_idx;
	U32 cur_grp;
	DEV_CONFIG pcfg;
	DISPATCH dispatch;
	U64 overflow_status_clr = 0;
	DRV_EVENT_MASK_NODE event_flag;

	SEP_DRV_LOG_TRACE_IN("Masks: %p.", masks);

	this_cpu = CONTROL_THIS_CPU();
	bd = &cpu_buf[this_cpu];
	pcpu = &pcb[this_cpu];
	dev_idx = core_to_dev_map[this_cpu];
	cur_grp = CPU_STATE_current_group(pcpu);
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[dev_idx])[cur_grp];
	pcfg = LWPMU_DEVICE_pcfg(&devices[dev_idx]);
	dispatch = LWPMU_DEVICE_dispatch(&devices[dev_idx]);

	if (!pecb) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!pecb).");
		return;
	}

	// initialize masks
	DRV_MASKS_masks_num(masks) = 0;

	overflow_status = SYS_Read_MSR(ECB_entries_reg_id(
		pecb, ECB_SECTION_REG_INDEX(pecb, GLOBAL_STATUS_REG_INDEX,
					    PMU_OPERATION_GLOBAL_STATUS)));

	if (DEV_CONFIG_pebs_mode(pcfg) &&
	    (DEV_CONFIG_pebs_record_num(pcfg) == 1)) {
		overflow_status = PEBS_Overflowed(this_cpu, overflow_status, 0);
	}
	overflow_status_clr = overflow_status;

	if (dispatch->check_overflow_gp_errata) {
		overflow_status = dispatch->check_overflow_gp_errata(
			pecb, &overflow_status_clr);
	}
	SEP_DRV_LOG_TRACE("Overflow: cpu: %d, status 0x%llx.", this_cpu,
			  overflow_status);
	index = 0;
	BUFFER_DESC_sample_count(bd) = 0;
	FOR_EACH_REG_CORE_OPERATION(pecb, i, PMU_OPERATION_DATA_ALL)
	{
		if (ECB_entries_fixed_reg_get(pecb, i)) {
			index = i -
				ECB_operations_register_start(
					pecb, PMU_OPERATION_DATA_FIXED) +
				0x20;
			if (dispatch->check_overflow_errata) {
				overflow_status =
					dispatch->check_overflow_errata(
						pecb, i, overflow_status);
			}
		} else if (ECB_entries_is_gp_reg_get(pecb, i)) {
			index = i - ECB_operations_register_start(
					    pecb, PMU_OPERATION_DATA_GP);
		} else {
			continue;
		}
		if (overflow_status & ((U64)1 << index)) {
			SEP_DRV_LOG_TRACE("Overflow: cpu: %d, index %d.",
					  this_cpu, index);
			SEP_DRV_LOG_TRACE(
				"Register 0x%x --- val 0%llx.",
				ECB_entries_reg_id(pecb, i),
				SYS_Read_MSR(ECB_entries_reg_id(pecb, i)));
			SYS_Write_MSR(ECB_entries_reg_id(pecb, i),
				      ECB_entries_reg_value(pecb, i));

			if (DRV_CONFIG_enable_cp_mode(drv_cfg)) {
				/* Increment the interrupt count. */
				if (interrupt_counts) {
					interrupt_counts
						[this_cpu *
							 DRV_CONFIG_num_events(
								 drv_cfg) +
						 ECB_entries_event_id_index(
							 pecb, i)] += 1;
				}
			}

			DRV_EVENT_MASK_bitFields1(&event_flag) = (U8)0;
			if (ECB_entries_precise_get(pecb, i)) {
				DRV_EVENT_MASK_precise(&event_flag) = 1;
			}
			if (ECB_entries_lbr_value_get(pecb, i)) {
				DRV_EVENT_MASK_lbr_capture(&event_flag) = 1;
			}
			if (ECB_entries_uncore_get(pecb, i)) {
				DRV_EVENT_MASK_uncore_capture(&event_flag) = 1;
			}
			if (ECB_entries_branch_evt_get(pecb, i)) {
				DRV_EVENT_MASK_branch(&event_flag) = 1;
			}

			if (DRV_MASKS_masks_num(masks) < MAX_OVERFLOW_EVENTS) {
				DRV_EVENT_MASK_bitFields1(
					DRV_MASKS_eventmasks(masks) +
					DRV_MASKS_masks_num(masks)) =
					DRV_EVENT_MASK_bitFields1(&event_flag);
				DRV_EVENT_MASK_event_idx(
					DRV_MASKS_eventmasks(masks) +
					DRV_MASKS_masks_num(masks)) =
					ECB_entries_event_id_index(pecb, i);
				DRV_MASKS_masks_num(masks)++;
			} else {
				SEP_DRV_LOG_ERROR(
					"The array for event masks is full.");
			}

			SEP_DRV_LOG_TRACE("Overflow -- 0x%llx, index 0x%llx.",
					  overflow_status, (U64)1 << index);
			SEP_DRV_LOG_TRACE("Slot# %d, reg_id 0x%x, index %d.", i,
					  ECB_entries_reg_id(pecb, i), index);
			if (ECB_entries_event_id_index(pecb, i) ==
			    CPU_STATE_trigger_event_num(pcpu)) {
				CPU_STATE_trigger_count(pcpu)--;
			}
		}
	}
	END_FOR_EACH_REG_CORE_OPERATION;

	CPU_STATE_reset_mask(pcpu) = overflow_status_clr;
	/* Clear outstanding overflow bits */
	SYS_Write_MSR(ECB_entries_reg_id(
			      pecb, ECB_SECTION_REG_INDEX(
					    pecb, GLOBAL_OVF_CTRL_REG_INDEX,
					    PMU_OPERATION_GLOBAL_REGS)),
		      overflow_status_clr & PERFVER4_OVERFLOW_BIT_MASK_HT_ON);

	SEP_DRV_LOG_TRACE("Check overflow completed %d.", this_cpu);

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn perfver4_Swap_Group(restart)
 *
 * @param    restart    dummy parameter which is not used
 *
 * @return   None     No return needed
 *
 * @brief    Perform the mechanics of swapping the event groups for event mux operations
 *
 * <I>Special Notes</I>
 *         Swap function for event multiplexing.
 *         Freeze the counting.
 *         Swap the groups.
 *         Enable the counting.
 *         Reset the event trigger count
 *
 */
static VOID perfver4_Swap_Group(DRV_BOOL restart)
{
	U32 index;
	U32 next_group;
	U32 st_index;
	U32 this_cpu;
	CPU_STATE pcpu;
	U32 dev_idx;
	DISPATCH dispatch;
	DEV_CONFIG pcfg;
	EVENT_CONFIG ec;
	U32 counter_index;

	SEP_DRV_LOG_TRACE_IN("Dummy restart: %u.", restart);

	this_cpu = CONTROL_THIS_CPU();
	pcpu = &pcb[this_cpu];
	dev_idx = core_to_dev_map[this_cpu];
	dispatch = LWPMU_DEVICE_dispatch(&devices[dev_idx]);
	pcfg = LWPMU_DEVICE_pcfg(&devices[dev_idx]);
	ec = LWPMU_DEVICE_ec(&devices[dev_idx]);
	counter_index = 0;

	st_index =
		CPU_STATE_current_group(pcpu) * EVENT_CONFIG_max_gp_events(ec);
	next_group = (CPU_STATE_current_group(pcpu) + 1);
	if (next_group >= EVENT_CONFIG_num_groups(ec)) {
		next_group = 0;
	}

	SEP_DRV_LOG_TRACE("Current group : 0x%x.",
			  CPU_STATE_current_group(pcpu));
	SEP_DRV_LOG_TRACE("Next group : 0x%x.", next_group);

	// Save the counters for the current group
	if (!DRV_CONFIG_event_based_counts(drv_cfg)) {
		FOR_EACH_REG_CORE_OPERATION(pecb, i, PMU_OPERATION_DATA_GP)
		{
			index = st_index + i -
				ECB_operations_register_start(
					pecb, PMU_OPERATION_DATA_GP);
			CPU_STATE_em_tables(pcpu)[index] =
				SYS_Read_MSR(ECB_entries_reg_id(pecb, i));
			SEP_DRV_LOG_TRACE("Saved value for reg 0x%x : 0x%llx.",
					  ECB_entries_reg_id(pecb, i),
					  CPU_STATE_em_tables(pcpu)[index]);
		}
		END_FOR_EACH_REG_CORE_OPERATION;
	}

	CPU_STATE_current_group(pcpu) = next_group;

	if (dispatch->hw_errata) {
		dispatch->hw_errata();
	}

	// First write the GP control registers (eventsel)
	FOR_EACH_REG_CORE_OPERATION(pecb, i, PMU_OPERATION_CTRL_GP)
	{
		SYS_Write_MSR(ECB_entries_reg_id(pecb, i),
			      ECB_entries_reg_value(pecb, i));
	}
	END_FOR_EACH_REG_CORE_OPERATION;

	if (DRV_CONFIG_event_based_counts(drv_cfg)) {
		// In EBC mode, reset the counts for all events except for trigger event
		FOR_EACH_REG_CORE_OPERATION(pecb, i, PMU_OPERATION_DATA_ALL)
		{
			if (ECB_entries_event_id_index(pecb, i) !=
			    CPU_STATE_trigger_event_num(pcpu)) {
				SYS_Write_MSR(ECB_entries_reg_id(pecb, i), 0LL);
			}
		}
		END_FOR_EACH_REG_CORE_OPERATION;
	} else {
		// Then write the gp count registers
		st_index = CPU_STATE_current_group(pcpu) *
			   EVENT_CONFIG_max_gp_events(ec);
		FOR_EACH_REG_CORE_OPERATION(pecb, i, PMU_OPERATION_DATA_GP)
		{
			index = st_index + i -
				ECB_operations_register_start(
					pecb, PMU_OPERATION_DATA_GP);
			SYS_Write_MSR(ECB_entries_reg_id(pecb, i),
				      CPU_STATE_em_tables(pcpu)[index]);
			SEP_DRV_LOG_TRACE(
				"Restore value for reg 0x%x : 0x%llx.",
				ECB_entries_reg_id(pecb, i),
				CPU_STATE_em_tables(pcpu)[index]);
		}
		END_FOR_EACH_REG_CORE_OPERATION;
	}

	FOR_EACH_REG_CORE_OPERATION(pecb, i, PMU_OPERATION_OCR)
	{
		SYS_Write_MSR(ECB_entries_reg_id(pecb, i),
			      ECB_entries_reg_value(pecb, i));
	}
	END_FOR_EACH_REG_CORE_OPERATION;

	if (DEV_CONFIG_pebs_record_num(pcfg)) {
		FOR_EACH_REG_CORE_OPERATION(pecb, i, PMU_OPERATION_DATA_ALL)
		{
			if (ECB_entries_precise_get(pecb, i) == 1) {
				if (ECB_entries_fixed_reg_get(pecb, i)) {
					counter_index =
						i -
						ECB_operations_register_start(
							pecb,
							PMU_OPERATION_DATA_FIXED) +
						8;
				} else {
					counter_index =
						i -
						ECB_operations_register_start(
							pecb,
							PMU_OPERATION_DATA_GP);
				}
				PEBS_Reset_Counter(this_cpu, counter_index,
						   ECB_entries_reg_value(pecb,
									 i));
			}
		}
		END_FOR_EACH_REG_CORE_OPERATION;
	}

	/*
	 *  reset the em factor when a group is swapped
	 */
	CPU_STATE_trigger_count(pcpu) = EVENT_CONFIG_em_factor(ec);

	/*
	 * The enable routine needs to rewrite the control registers
	 */
	CPU_STATE_reset_mask(pcpu) = 0LL;
	CPU_STATE_group_swap(pcpu) = 1;

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn perfver4_Initialize(params)
 *
 * @param    params    dummy parameter which is not used
 *
 * @return   None     No return needed
 *
 * @brief    Initialize the PMU setting up for collection
 *
 * <I>Special Notes</I>
 *         Saves the relevant PMU state (minimal set of MSRs required
 *         to avoid conflicts with other Linux tools, such as Oprofile).
 *         This function should be called in parallel across all CPUs
 *         prior to the start of sampling, before PMU state is changed.
 *
 */
static VOID perfver4_Initialize(VOID *param)
{
	U32 this_cpu;
	CPU_STATE pcpu;
	U32 dev_idx;
	DEV_CONFIG pcfg;
	U32 cur_grp;
	ECB pecb = NULL;

	SEP_DRV_LOG_TRACE_IN("Dummy param: %p.", param);

	if (pcb == NULL) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!pcb).");
		return;
	}

	this_cpu = CONTROL_THIS_CPU();
	pcpu = &pcb[this_cpu];
	dev_idx = core_to_dev_map[this_cpu];
	pcfg = LWPMU_DEVICE_pcfg(&devices[dev_idx]);
	CPU_STATE_pmu_state(pcpu) = pmu_state + (this_cpu * 3);
	if (CPU_STATE_pmu_state(pcpu) == NULL) {
		SEP_DRV_LOG_WARNING_TRACE_OUT(
			"Unable to save PMU state on CPU %d.", this_cpu);
		return;
	}

	cur_grp = CPU_STATE_current_group(pcpu);
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[dev_idx])[cur_grp];
	restore_reg_addr[0] = ECB_entries_reg_id(
		pecb, ECB_SECTION_REG_INDEX(pecb, DEBUG_CTRL_REG_INDEX,
					    PMU_OPERATION_GLOBAL_REGS));
	restore_reg_addr[1] = ECB_entries_reg_id(
		pecb, ECB_SECTION_REG_INDEX(pecb, GLOBAL_CTRL_REG_INDEX,
					    PMU_OPERATION_GLOBAL_REGS));
	restore_reg_addr[2] = ECB_entries_reg_id(
		pecb, ECB_SECTION_REG_INDEX(pecb, FIXED_CTRL_REG_INDEX,
					    PMU_OPERATION_GLOBAL_REGS));

	// save the original PMU state on this CPU (NOTE: must only be called ONCE per collection)
	CPU_STATE_pmu_state(pcpu)[0] = SYS_Read_MSR(restore_reg_addr[0]);
	CPU_STATE_pmu_state(pcpu)[1] = SYS_Read_MSR(restore_reg_addr[1]);
	CPU_STATE_pmu_state(pcpu)[2] = SYS_Read_MSR(restore_reg_addr[2]);

	if (DRV_CONFIG_ds_area_available(drv_cfg) &&
	    DEV_CONFIG_pebs_mode(pcfg)) {
		SYS_Write_MSR(ECB_entries_reg_id(
				      pecb, ECB_SECTION_REG_INDEX(
						    pecb, PEBS_ENABLE_REG_INDEX,
						    PMU_OPERATION_GLOBAL_REGS)),
			      0LL);
	}

	SEP_DRV_LOG_TRACE("Saving PMU state on CPU %d:", this_cpu);
	SEP_DRV_LOG_TRACE("    msr_val(IA32_DEBUG_CTRL)=0x%llx.",
			  CPU_STATE_pmu_state(pcpu)[0]);
	SEP_DRV_LOG_TRACE("    msr_val(IA32_PERF_GLOBAL_CTRL)=0x%llx.",
			  CPU_STATE_pmu_state(pcpu)[1]);
	SEP_DRV_LOG_TRACE("    msr_val(IA32_FIXED_CTRL)=0x%llx.",
			  CPU_STATE_pmu_state(pcpu)[2]);

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn perfver4_Destroy(params)
 *
 * @param    params    dummy parameter which is not used
 *
 * @return   None     No return needed
 *
 * @brief    Reset the PMU setting up after collection
 *
 * <I>Special Notes</I>
 *         Restores the previously saved PMU state done in pmv_v4_Initialize.
 *         This function should be called in parallel across all CPUs
 *         after sampling collection ends/terminates.
 *
 */
static VOID perfver4_Destroy(VOID *param)
{
	U32 this_cpu;
	CPU_STATE pcpu;

	SEP_DRV_LOG_TRACE_IN("Dummy param: %p.", param);

	if (pcb == NULL) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!pcb).");
		return;
	}

	preempt_disable();
	this_cpu = CONTROL_THIS_CPU();
	preempt_enable();
	pcpu = &pcb[this_cpu];

	if (CPU_STATE_pmu_state(pcpu) == NULL) {
		SEP_DRV_LOG_WARNING_TRACE_OUT(
			"Unable to restore PMU state on CPU %d.", this_cpu);
		return;
	}

	SEP_DRV_LOG_TRACE("Clearing PMU state on CPU %d:", this_cpu);
	SEP_DRV_LOG_TRACE("    msr_val(IA32_DEBUG_CTRL)=0x0.");
	SEP_DRV_LOG_TRACE("    msr_val(IA32_PERF_GLOBAL_CTRL)=0x0.");
	SEP_DRV_LOG_TRACE("    msr_val(IA32_FIXED_CTRL)=0x0.");

	SYS_Write_MSR(restore_reg_addr[0], 0);
	SYS_Write_MSR(restore_reg_addr[1], 0);
	SYS_Write_MSR(restore_reg_addr[2], 0);

	CPU_STATE_pmu_state(pcpu) = NULL;

	SEP_DRV_LOG_TRACE_OUT("");
}

/*
 * @fn perfver4_Read_LBRs(buffer)
 *
 * @param   IN buffer - pointer to the buffer to write the data into
 * @return  Last branch source IP address
 *
 * @brief   Read all the LBR registers into the buffer provided and return
 *
 */
static U64 perfver4_Read_LBRs(VOID *buffer, PVOID data)
{
	U32 i, count = 0;
	U64 *lbr_buf = NULL;
	U64 value = 0;
	U64 tos_ip_addr = 0;
	U64 tos_ptr = 0;
	SADDR saddr;
	U32 pairs = 0;
	U32 this_cpu;
	U32 dev_idx;
	LBR lbr;
	DEV_CONFIG pcfg;
#if defined(DRV_SEP_ACRN_ON)
	struct lbr_pmu_sample *lbr_data = NULL;
#endif

	SEP_DRV_LOG_TRACE_IN("Buffer: %p.", buffer);

	preempt_disable();
	this_cpu = CONTROL_THIS_CPU();
	preempt_enable();
	dev_idx = core_to_dev_map[this_cpu];
	pcfg = LWPMU_DEVICE_pcfg(&devices[dev_idx]);
	lbr = LWPMU_DEVICE_lbr(&devices[dev_idx]);

	if (lbr == NULL) {
		return 0;
	}

#if defined(DRV_SEP_ACRN_ON)
	if (data == NULL) {
		return 0;
	}
	lbr_data = (struct lbr_pmu_sample *)data;
#endif

	if (buffer && DEV_CONFIG_store_lbrs(pcfg)) {
		lbr_buf = (U64 *)buffer;
	}

	if (LBR_num_entries(lbr) > 0) {
		pairs = (LBR_num_entries(lbr) - 1) / 3;
	}
	for (i = 0; i < LBR_num_entries(lbr); i++) {
#if !defined(DRV_SEP_ACRN_ON)
		value = SYS_Read_MSR(LBR_entries_reg_id(lbr, i));
#else
		if (i == 0) {
			value = lbr_data->lbr_tos;
		} else {
			if (LBR_entries_etype(lbr, i) == LBR_ENTRY_FROM_IP) {
				value = lbr_data->lbr_from_ip[i - 1];
			} else if (LBR_entries_etype(lbr, i) ==
				   LBR_ENTRY_TO_IP) {
				value = lbr_data->lbr_to_ip[i - pairs - 1];
			} else {
				value = lbr_data->lbr_info[i - 2 * pairs - 1];
			}
		}
#endif
		if (buffer && DEV_CONFIG_store_lbrs(pcfg)) {
			*lbr_buf = value;
		}
		if (DEV_CONFIG_collect_callstacks(pcfg)) {
			if ((LBR_entries_etype(lbr, i) == LBR_ENTRY_FROM_IP &&
			     i > tos_ptr + 1) ||
			    (LBR_entries_etype(lbr, i) == LBR_ENTRY_TO_IP &&
			     i > tos_ptr + pairs + 1) ||
			    (LBR_entries_etype(lbr, i) == LBR_ENTRY_INFO &&
			     i > tos_ptr + 2 * pairs + 1)) {
				if (buffer && DEV_CONFIG_store_lbrs(pcfg)) {
					*lbr_buf = 0x0ULL;
					lbr_buf++;
				}
				continue;
			}
		}
		SEP_DRV_LOG_TRACE("LBR %u, 0x%llx.", i, value);
		if (i == 0) {
			tos_ptr = value;
		} else {
			if (LBR_entries_etype(lbr, i) ==
			    LBR_ENTRY_FROM_IP) { // LBR from register
				if (tos_ptr == count) {
					SADDR_addr(saddr) =
						value & PERFVER4_LBR_BITMASK;
					tos_ip_addr = (U64)SADDR_addr(
						saddr); // Add signed extension
					SEP_DRV_LOG_TRACE(
						"Tos_ip_addr %llu, 0x%llx.",
						tos_ptr, value);
				}
				count++;
			}
		}
		if (buffer && DEV_CONFIG_store_lbrs(pcfg)) {
			lbr_buf++;
		}
	}

	SEP_DRV_LOG_TRACE_OUT("Res: %llu.", tos_ip_addr);
	return tos_ip_addr;
}

/*
 * @fn perfver4_Clean_Up(param)
 *
 * @param   IN param - currently not used
 *
 * @brief   Clean up registers in ECB
 *
 */
static VOID perfver4_Clean_Up(VOID *param)
{
	U32 this_cpu;
	CPU_STATE pcpu;
	ECB pecb = NULL;
	U32 dev_idx;
	U32 cur_grp;

	SEP_DRV_LOG_TRACE_IN("Dummy param: %p.", param);

	this_cpu = CONTROL_THIS_CPU();
	pcpu = &pcb[this_cpu];
	dev_idx = core_to_dev_map[this_cpu];
	cur_grp = CPU_STATE_current_group(pcpu);
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[dev_idx])[cur_grp];

	if (!pecb) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!pecb).");
		return;
	}

	FOR_EACH_REG_CORE_OPERATION(pecb, i, PMU_OPERATION_ALL_REG)
	{
		if (ECB_entries_clean_up_get(pecb, i)) {
			SEP_DRV_LOG_TRACE("Clean up set --- RegId --- %x.",
					  ECB_entries_reg_id(pecb, i));
			SYS_Write_MSR(ECB_entries_reg_id(pecb, i), 0LL);
		}
	}
	END_FOR_EACH_REG_CORE_OPERATION;

	/* Clear outstanding frozen bits */
	if (pecb) {
		SYS_Write_MSR(ECB_entries_reg_id(
				      pecb,
				      ECB_SECTION_REG_INDEX(
					      pecb, GLOBAL_OVF_CTRL_REG_INDEX,
					      PMU_OPERATION_GLOBAL_REGS)),
			      PERFVER4_FROZEN_BIT_MASK);
	}

	SEP_DRV_LOG_TRACE_OUT("");
	return;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn void perfver4_Check_Overflow_Htoff_Mode(masks)
 *
 * @param    masks    the mask structure to populate
 *
 * @return   None     No return needed
 *
 * @brief  Called by the data processing method to figure out which registers have overflowed.
 *
 */
static void perfver4_Check_Overflow_Htoff_Mode(DRV_MASKS masks)
{
	U32 index;
	U64 value = 0;
	U64 overflow_status = 0;
	U32 this_cpu;
	BUFFER_DESC bd;
	CPU_STATE pcpu;
	ECB pecb;
	U32 dev_idx;
	U32 cur_grp;
	DISPATCH dispatch;
	DEV_CONFIG pcfg;
	U64 overflow_status_clr = 0;
	DRV_EVENT_MASK_NODE event_flag;

	SEP_DRV_LOG_TRACE_IN("Masks: %p.", masks);

	this_cpu = CONTROL_THIS_CPU();
	bd = &cpu_buf[this_cpu];
	pcpu = &pcb[this_cpu];
	dev_idx = core_to_dev_map[this_cpu];
	cur_grp = CPU_STATE_current_group(pcpu);
	dispatch = LWPMU_DEVICE_dispatch(&devices[dev_idx]);
	pcfg = LWPMU_DEVICE_pcfg(&devices[dev_idx]);
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[dev_idx])[cur_grp];

	if (!pecb) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!pecb).");
		return;
	}

	// initialize masks
	DRV_MASKS_masks_num(masks) = 0;

	overflow_status = SYS_Read_MSR(ECB_entries_reg_id(
		pecb, ECB_SECTION_REG_INDEX(pecb, GLOBAL_STATUS_REG_INDEX,
					    PMU_OPERATION_GLOBAL_STATUS)));

	if (DEV_CONFIG_pebs_mode(pcfg) &&
	    (DEV_CONFIG_pebs_record_num(pcfg) == 1)) {
		overflow_status = PEBS_Overflowed(this_cpu, overflow_status, 0);
	}
	overflow_status_clr = overflow_status;
	SEP_DRV_LOG_TRACE("Overflow: cpu: %d, status 0x%llx.", this_cpu,
			  overflow_status);
	index = 0;
	BUFFER_DESC_sample_count(bd) = 0;

	if (dispatch->check_overflow_gp_errata) {
		overflow_status = dispatch->check_overflow_gp_errata(
			pecb, &overflow_status_clr);
	}

	FOR_EACH_REG_CORE_OPERATION(pecb, i, PMU_OPERATION_DATA_ALL)
	{
		if (ECB_entries_fixed_reg_get(pecb, i)) {
			index = i -
				ECB_operations_register_start(
					pecb, PMU_OPERATION_DATA_FIXED) +
				0x20;
		} else if (ECB_entries_is_gp_reg_get(pecb, i) &&
			   ECB_entries_reg_value(pecb, i) != 0) {
			index = i - ECB_operations_register_start(
					    pecb, PMU_OPERATION_DATA_GP);
			if (index >= 4 && index <= 7) {
				value = SYS_Read_MSR(
					ECB_entries_reg_id(pecb, i));
				if (value > 0 && value <= 0x100000000LL) {
					overflow_status |= ((U64)1 << index);
				}
			}
		} else {
			continue;
		}
		if (overflow_status & ((U64)1 << index)) {
			SEP_DRV_LOG_TRACE("Overflow:  cpu: %d, index %d.",
					  this_cpu, index);
			SEP_DRV_LOG_TRACE(
				"Register 0x%x --- val 0%llx.",
				ECB_entries_reg_id(pecb, i),
				SYS_Read_MSR(ECB_entries_reg_id(pecb, i)));
			SYS_Write_MSR(ECB_entries_reg_id(pecb, i),
				      ECB_entries_reg_value(pecb, i));

			if (DRV_CONFIG_enable_cp_mode(drv_cfg)) {
				/* Increment the interrupt count. */
				if (interrupt_counts) {
					interrupt_counts
						[this_cpu *
							 DRV_CONFIG_num_events(
								 drv_cfg) +
						 ECB_entries_event_id_index(
							 pecb, i)] += 1;
				}
			}

			DRV_EVENT_MASK_bitFields1(&event_flag) = (U8)0;
			if (ECB_entries_precise_get(pecb, i)) {
				DRV_EVENT_MASK_precise(&event_flag) = 1;
			}
			if (ECB_entries_lbr_value_get(pecb, i)) {
				DRV_EVENT_MASK_lbr_capture(&event_flag) = 1;
			}

			if (DRV_MASKS_masks_num(masks) < MAX_OVERFLOW_EVENTS) {
				DRV_EVENT_MASK_bitFields1(
					DRV_MASKS_eventmasks(masks) +
					DRV_MASKS_masks_num(masks)) =
					DRV_EVENT_MASK_bitFields1(&event_flag);
				DRV_EVENT_MASK_event_idx(
					DRV_MASKS_eventmasks(masks) +
					DRV_MASKS_masks_num(masks)) =
					ECB_entries_event_id_index(pecb, i);
				DRV_MASKS_masks_num(masks)++;
			} else {
				SEP_DRV_LOG_ERROR(
					"The array for event masks is full.");
			}

			SEP_DRV_LOG_TRACE("Overflow -- 0x%llx, index 0x%llx.",
					  overflow_status, (U64)1 << index);
			SEP_DRV_LOG_TRACE("Slot# %d, reg_id 0x%x, index %d.", i,
					  ECB_entries_reg_id(pecb, i), index);
			if (ECB_entries_event_id_index(pecb, i) ==
			    CPU_STATE_trigger_event_num(pcpu)) {
				CPU_STATE_trigger_count(pcpu)--;
			}
		}
	}
	END_FOR_EACH_REG_CORE_OPERATION;

	CPU_STATE_reset_mask(pcpu) = overflow_status_clr;
	/* Clear outstanding overflow bits */
	SYS_Write_MSR(ECB_entries_reg_id(
			      pecb, ECB_SECTION_REG_INDEX(
					    pecb, GLOBAL_OVF_CTRL_REG_INDEX,
					    PMU_OPERATION_GLOBAL_REGS)),
		      overflow_status_clr & PERFVER4_OVERFLOW_BIT_MASK_HT_OFF);

	SEP_DRV_LOG_TRACE_OUT("");
}

#define MAX_COUNTER 0xFFFFFFFFFFFFLLU
#define FIXED_CTR3_BIT_INDEX 35

/* ------------------------------------------------------------------------- */
/*!
 * @fn void perfver4_Check_Overflow_Nonht_Mode(masks)
 *
 * @param    masks    the mask structure to populate
 *
 * @return   None     No return needed
 *
 * @brief  Called by the data processing method to figure out which registers have overflowed.
 *
 */
static VOID perfver4_Check_Overflow_Nonht_Mode(DRV_MASKS masks)
{
	U32 index;
	U64 overflow_status = 0;
	U32 this_cpu = CONTROL_THIS_CPU();
	BUFFER_DESC bd = &cpu_buf[this_cpu];
	CPU_STATE pcpu = &pcb[this_cpu];
	U32 dev_idx = core_to_dev_map[this_cpu];
	U32 cur_grp = CPU_STATE_current_group(pcpu);
	DEV_CONFIG pcfg = LWPMU_DEVICE_pcfg(&devices[dev_idx]);
	ECB pecb = LWPMU_DEVICE_PMU_register_data(&devices[dev_idx])[cur_grp];
	U64 overflow_status_clr = 0;
	DRV_EVENT_MASK_NODE event_flag;

	SEP_DRV_LOG_TRACE_IN("");

	if (!pecb) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!pecb).");
		return;
	}

	// initialize masks
	DRV_MASKS_masks_num(masks) = 0;

	overflow_status = SYS_Read_MSR(ECB_entries_reg_id(
		pecb, ECB_SECTION_REG_INDEX(pecb, GLOBAL_STATUS_REG_INDEX,
					    PMU_OPERATION_GLOBAL_STATUS)));

	if (DEV_CONFIG_pebs_mode(pcfg) &&
	    (DEV_CONFIG_pebs_record_num(pcfg) == 1)) {
		overflow_status = PEBS_Overflowed(this_cpu, overflow_status, 0);
	}
	overflow_status_clr = overflow_status;
	SEP_DRV_LOG_TRACE("Overflow:  cpu: %d, status 0x%llx.", this_cpu,
			  overflow_status);
	index = 0;
	BUFFER_DESC_sample_count(bd) = 0;

	FOR_EACH_REG_CORE_OPERATION(pecb, i, PMU_OPERATION_DATA_ALL)
	{
		if (ECB_entries_fixed_reg_get(pecb, i)) {
			index = i -
				ECB_operations_register_start(
					pecb, PMU_OPERATION_DATA_FIXED) +
				0x20;
		} else if (ECB_entries_is_gp_reg_get(pecb, i) &&
			   ECB_entries_reg_value(pecb, i) != 0) {
			index = i - ECB_operations_register_start(
					    pecb, PMU_OPERATION_DATA_GP);
		} else {
			continue;
		}
		if (overflow_status & ((U64)1 << index)) {
			SEP_DRV_LOG_TRACE("Overflow:  cpu: %d, index %d.",
					  this_cpu, index);
			SEP_DRV_LOG_TRACE(
				"register 0x%x --- val 0%llx.",
				ECB_entries_reg_id(pecb, i),
				SYS_Read_MSR(ECB_entries_reg_id(pecb, i)));

			DRV_EVENT_MASK_bitFields1(&event_flag) = (U8)0;
			if (DEV_CONFIG_enable_perf_metrics(pcfg) &&
			    index == FIXED_CTR3_BIT_INDEX) {
				perf_metrics_counter_reload_value =
					ECB_entries_reg_value(
						pecb, i); // saving reload value
				// Writing positive SAV into data register before reading metrics
				SYS_Write_MSR(
					ECB_entries_reg_id(pecb, i),
					((~(ECB_entries_reg_value(pecb, i)) +
					  1) &
					 MAX_COUNTER));
				DRV_EVENT_MASK_perf_metrics_capture(
					&event_flag) = 1;
			} else {
				SYS_Write_MSR(ECB_entries_reg_id(pecb, i),
					      ECB_entries_reg_value(pecb, i));
			}
			if (DRV_CONFIG_enable_cp_mode(drv_cfg)) {
				/* Increment the interrupt count. */
				if (interrupt_counts) {
					interrupt_counts
						[this_cpu *
							 DRV_CONFIG_num_events(
								 drv_cfg) +
						 ECB_entries_event_id_index(
							 pecb, i)] += 1;
				}
			}

			if (ECB_entries_precise_get(pecb, i)) {
				DRV_EVENT_MASK_precise(&event_flag) = 1;
			}
			if (ECB_entries_lbr_value_get(pecb, i)) {
				DRV_EVENT_MASK_lbr_capture(&event_flag) = 1;
			}
			if (ECB_entries_uncore_get(pecb, i)) {
				DRV_EVENT_MASK_uncore_capture(&event_flag) = 1;
			}
			if (ECB_entries_branch_evt_get(pecb, i)) {
				DRV_EVENT_MASK_branch(&event_flag) = 1;
			}

			if (DRV_MASKS_masks_num(masks) < MAX_OVERFLOW_EVENTS) {
				DRV_EVENT_MASK_bitFields1(
					DRV_MASKS_eventmasks(masks) +
					DRV_MASKS_masks_num(masks)) =
					DRV_EVENT_MASK_bitFields1(&event_flag);
				DRV_EVENT_MASK_event_idx(
					DRV_MASKS_eventmasks(masks) +
					DRV_MASKS_masks_num(masks)) =
					ECB_entries_event_id_index(pecb, i);
				DRV_MASKS_masks_num(masks)++;
			} else {
				SEP_DRV_LOG_ERROR(
					"The array for event masks is full.");
			}

			SEP_DRV_LOG_TRACE("Overflow -- 0x%llx, index 0x%llx.",
					  overflow_status, (U64)1 << index);
			SEP_DRV_LOG_TRACE("Slot# %d, reg_id 0x%x, index %d.", i,
					  ECB_entries_reg_id(pecb, i), index);
			if (ECB_entries_event_id_index(pecb, i) ==
			    CPU_STATE_trigger_event_num(pcpu)) {
				CPU_STATE_trigger_count(pcpu)--;
			}
		}
	}
	END_FOR_EACH_REG_CORE_OPERATION;

	CPU_STATE_reset_mask(pcpu) = overflow_status_clr;
	/* Clear outstanding overflow bits */
	SYS_Write_MSR(ECB_entries_reg_id(
			      pecb, ECB_SECTION_REG_INDEX(
					    pecb, GLOBAL_OVF_CTRL_REG_INDEX,
					    PMU_OPERATION_GLOBAL_REGS)),
		      overflow_status_clr & PERFVER4_OVERFLOW_BIT_MASK_NON_HT);

	SEP_DRV_LOG_TRACE("Check Overflow completed %d.", this_cpu);
}
/* ------------------------------------------------------------------------- */
/*!
 * @fn void perfver4_Read_Power(buffer)
 *
 * @param    buffer   - pointer to the buffer to write the data into
 *
 * @return   None     No return needed
 *
 * @brief  Read all the power MSRs into the buffer provided and return.
 *
 */
static VOID perfver4_Read_Power(VOID *buffer)
{
	U32 i;
	U64 *pwr_buf = (U64 *)buffer;
	U32 this_cpu;
	U32 dev_idx;
	PWR pwr;

	SEP_DRV_LOG_TRACE_IN("Buffer: %p.", buffer);

	this_cpu = CONTROL_THIS_CPU();
	dev_idx = core_to_dev_map[this_cpu];
	pwr = LWPMU_DEVICE_pwr(&devices[dev_idx]);

	for (i = 0; i < PWR_num_entries(pwr); i++) {
		*pwr_buf = SYS_Read_MSR(PWR_entries_reg_id(pwr, i));
		pwr_buf++;
	}

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn perfver4_Read_Counts(param, id)
 *
 * @param    param    The read thread node to process
 * @param    id       The event id for the which the sample is generated
 *
 * @return   None     No return needed
 *
 * @brief    Read CPU event based counts data and store into the buffer param;
 *           For the case of the trigger event, store the SAV value.
 */
static VOID perfver4_Read_Counts(PVOID param, U32 id)
{
	U64 *data;
	U32 this_cpu;
	CPU_STATE pcpu;
	U32 dev_idx;
	DEV_CONFIG pcfg;
	U32 event_id = 0;

	SEP_DRV_LOG_TRACE_IN("Param: %p, id: %u.", param, id);

	this_cpu = CONTROL_THIS_CPU();
	pcpu = &pcb[this_cpu];
	dev_idx = core_to_dev_map[this_cpu];
	pcfg = LWPMU_DEVICE_pcfg(&devices[dev_idx]);

	if (DEV_CONFIG_ebc_group_id_offset(pcfg)) {
		// Write GroupID
		data = (U64 *)((S8 *)param +
			       DEV_CONFIG_ebc_group_id_offset(pcfg));
		*data = CPU_STATE_current_group(pcpu) + 1;
	}

	FOR_EACH_REG_CORE_OPERATION(pecb, i, PMU_OPERATION_DATA_ALL)
	{
		if (ECB_entries_counter_event_offset(pecb, i) == 0) {
			continue;
		}
		data = (U64 *)((S8 *)param +
			       ECB_entries_counter_event_offset(pecb, i));
		event_id = ECB_entries_event_id_index(pecb, i);
		if (event_id == id) {
			*data = ~(ECB_entries_reg_value(pecb, i) - 1) &
				ECB_entries_max_bits(pecb, i);
			;
		} else {
			*data = SYS_Read_MSR(ECB_entries_reg_id(pecb, i));
			SYS_Write_MSR(ECB_entries_reg_id(pecb, i), 0LL);
		}
	}
	END_FOR_EACH_REG_CORE_OPERATION;

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn perfver4_Read_Metrics(buffer, id)
 *
 * @param    param        buffer to write metrics into
 *
 * @return   None     No return needed
 *
 * @brief    Read hardware metrics from IA32_PERF_METRICS MSR
 */
static VOID perfver4_Read_Metrics(PVOID buffer)
{
	U64 *data, metrics = 0;
	U32 j;
	U32 this_cpu = CONTROL_THIS_CPU();
	U32 dev_idx = core_to_dev_map[this_cpu];
	DEV_CONFIG pcfg = LWPMU_DEVICE_pcfg(&devices[dev_idx]);

	data = (U64 *)buffer;
	FOR_EACH_NONEVENT_REG(pecb, i)
	{
		metrics = SYS_Read_MSR(ECB_entries_reg_id(pecb, i));
		for (j = 0; j < DEV_CONFIG_num_perf_metrics(pcfg); j++) {
			*data = (metrics & (0xFFULL << 8 * j)) >> 8 * j;
			data++;
		}
	}
	END_FOR_EACH_NONEVENT_REG;

	if (DRV_CONFIG_emon_mode(drv_cfg)) {
		SYS_Write_MSR(IA32_FIXED_CTR3, 0LL);
	} else {
		SYS_Write_MSR(IA32_FIXED_CTR3,
			      perf_metrics_counter_reload_value);
		perf_metrics_counter_reload_value = 0;
	}

	SYS_Write_MSR(IA32_PERF_METRICS, 0LL);
}
/* ------------------------------------------------------------------------- */
/*!
 * @fn          U64 perfver4_Platform_Info
 *
 * @brief       Reads the MSR_PLATFORM_INFO register if present
 *
 * @param       void
 *
 * @return      value read from the register
 *
 * <I>Special Notes:</I>
 *              <NONE>
 */
static VOID perfver4_Platform_Info(PVOID data)
{
	DRV_PLATFORM_INFO platform_data = (DRV_PLATFORM_INFO)data;
	U64 value = 0;
	U64 energy_multiplier;

	SEP_DRV_LOG_TRACE_IN("Data: %p.", data);

	if (!platform_data) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!platform_data).");
		return;
	}

#define IA32_MSR_PLATFORM_INFO 0xCE
	value = SYS_Read_MSR(IA32_MSR_PLATFORM_INFO);

	DRV_PLATFORM_INFO_info(platform_data) = value;
	DRV_PLATFORM_INFO_ddr_freq_index(platform_data) = 0;

#define IA32_MSR_MISC_ENABLE 0x1A4
	DRV_PLATFORM_INFO_misc_valid(platform_data) = 1;
	value = SYS_Read_MSR(IA32_MSR_MISC_ENABLE);
	DRV_PLATFORM_INFO_misc_info(platform_data) = value;
#undef IA32_MSR_MISC_ENABLE

	energy_multiplier = SYS_Read_MSR(MSR_ENERGY_MULTIPLIER);
	SEP_DRV_LOG_TRACE("MSR_ENERGY_MULTIPLIER: %llx.", energy_multiplier);
	DRV_PLATFORM_INFO_energy_multiplier(platform_data) =
		(U32)(energy_multiplier & 0x00001F00) >> 8;

	SEP_DRV_LOG_TRACE_OUT("");
}

/*
 * Initialize the dispatch table
 */
DISPATCH_NODE perfver4_dispatch = { .init = perfver4_Initialize,
				    .fini = perfver4_Destroy,
				    .write = perfver4_Write_PMU,
				    .freeze = perfver4_Disable_PMU,
				    .restart = perfver4_Enable_PMU,
				    .read_data = perfver4_Read_PMU_Data,
				    .check_overflow = perfver4_Check_Overflow,
				    .swap_group = perfver4_Swap_Group,
				    .read_lbrs = perfver4_Read_LBRs,
				    .cleanup = perfver4_Clean_Up,
				    .hw_errata = NULL,
				    .read_power = perfver4_Read_Power,
				    .check_overflow_errata = NULL,
				    .read_counts = perfver4_Read_Counts,
				    .check_overflow_gp_errata = NULL,
				    .read_ro = NULL,
				    .platform_info = perfver4_Platform_Info,
				    .trigger_read = NULL,
				    .scan_for_uncore = NULL,
				    .read_metrics = NULL };

DISPATCH_NODE perfver4_dispatch_htoff_mode = {
	.init = perfver4_Initialize,
	.fini = perfver4_Destroy,
	.write = perfver4_Write_PMU,
	.freeze = perfver4_Disable_PMU,
	.restart = perfver4_Enable_PMU,
	.read_data = perfver4_Read_PMU_Data,
	.check_overflow = perfver4_Check_Overflow_Htoff_Mode,
	.swap_group = perfver4_Swap_Group,
	.read_lbrs = perfver4_Read_LBRs,
	.cleanup = perfver4_Clean_Up,
	.hw_errata = NULL,
	.read_power = perfver4_Read_Power,
	.check_overflow_errata = NULL,
	.read_counts = perfver4_Read_Counts,
	.check_overflow_gp_errata = NULL,
	.read_ro = NULL,
	.platform_info = perfver4_Platform_Info,
	.trigger_read = NULL,
	.scan_for_uncore = NULL,
	.read_metrics = NULL
};

DISPATCH_NODE perfver4_dispatch_nonht_mode = {
	.init = perfver4_Initialize,
	.fini = perfver4_Destroy,
	.write = perfver4_Write_PMU,
	.freeze = perfver4_Disable_PMU,
	.restart = perfver4_Enable_PMU,
	.read_data = perfver4_Read_PMU_Data,
	.check_overflow = perfver4_Check_Overflow_Nonht_Mode,
	.swap_group = perfver4_Swap_Group,
	.read_lbrs = perfver4_Read_LBRs,
	.cleanup = perfver4_Clean_Up,
	.hw_errata = NULL,
	.read_power = perfver4_Read_Power,
	.check_overflow_errata = NULL,
	.read_counts = perfver4_Read_Counts,
	.check_overflow_gp_errata = NULL,
	.read_ro = NULL,
	.platform_info = perfver4_Platform_Info,
	.trigger_read = NULL,
	.scan_for_uncore = NULL,
	.read_metrics = perfver4_Read_Metrics
};
