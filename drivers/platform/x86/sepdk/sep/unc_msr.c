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
#include "lwpmudrv_types.h"
#include "lwpmudrv_ecb.h"
#include "lwpmudrv_struct.h"

#include "inc/ecb_iterators.h"
#include "inc/control.h"
#include "inc/unc_common.h"
#include "inc/utility.h"

extern U64 *read_counter_info;
extern EMON_BUFFER_DRIVER_HELPER emon_buffer_driver_helper;
extern DRV_CONFIG drv_cfg;

/*!
 * @fn          static VOID UNC_COMMON_MSR_Write_PMU(VOID*)
 *
 * @brief       Initial write of PMU registers
 * Walk through the enties and write the value of the register accordingly.
 * When current_group = 0, then this is the first time this routine is called,
 *
 * @param       None
 *
 * @return      None
 *
 * <I>Special Notes:</I>
 */
static VOID UNC_MSR_Write_PMU(PVOID param)
{
	U32 dev_idx;
	U32 this_cpu;
	CPU_STATE pcpu;

	SEP_DRV_LOG_TRACE_IN("Param: %p.", param);

	dev_idx = *((U32 *)param);
	this_cpu = CONTROL_THIS_CPU();
	pcpu = &pcb[this_cpu];

	if (!CPU_STATE_socket_master(pcpu)) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!CPU_STATE_socket_master).");
		return;
	}

	FOR_EACH_REG_UNC_OPERATION(pecb, dev_idx, idx, PMU_OPERATION_WRITE)
	{
		SYS_Write_MSR(ECB_entries_reg_id(pecb, idx),
			      ECB_entries_reg_value(pecb, idx));
	}
	END_FOR_EACH_REG_UNC_OPERATION;

	FOR_EACH_REG_UNC_OPERATION(pecb, dev_idx, idx, PMU_OPERATION_READ)
	{
		SYS_Write_MSR(ECB_entries_reg_id(pecb, idx), 0ULL);
		if (LWPMU_DEVICE_counter_mask(&devices[dev_idx]) == 0) {
			LWPMU_DEVICE_counter_mask(&devices[dev_idx]) =
				(U64)ECB_entries_max_bits(pecb, idx);
		}
	}
	END_FOR_EACH_REG_UNC_OPERATION;

	SEP_DRV_LOG_TRACE_OUT("");
}

/*!
 * @fn         static VOID UNC_MSR_Enable_PMU(PVOID)
 *
 * @brief      Set the enable bit for all the evsel registers
 *
 * @param      None
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
static VOID UNC_MSR_Enable_PMU(PVOID param)
{
	U32 dev_idx;
	U32 this_cpu;
	CPU_STATE pcpu;
	U64 reg_val = 0;

	SEP_DRV_LOG_TRACE_IN("Param: %p.", param);

	dev_idx = *((U32 *)param);
	this_cpu = CONTROL_THIS_CPU();
	pcpu = &pcb[this_cpu];

	if (!CPU_STATE_socket_master(pcpu)) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!CPU_STATE_socket_master).");
		return;
	}

	FOR_EACH_REG_UNC_OPERATION(pecb, dev_idx, idx, PMU_OPERATION_ENABLE)
	{
		reg_val = ECB_entries_reg_value(pecb, idx);
		if (ECB_entries_reg_rw_type(pecb, idx) ==
		    PMU_REG_RW_READ_WRITE) {
			reg_val = SYS_Read_MSR(ECB_entries_reg_id(pecb, idx));
			if (ECB_entries_reg_type(pecb, idx) ==
			    PMU_REG_UNIT_CTRL) {
				reg_val &= ECB_entries_reg_value(pecb, idx);
			} else {
				reg_val |= ECB_entries_reg_value(pecb, idx);
			}
		}
		SYS_Write_MSR(ECB_entries_reg_id(pecb, idx), reg_val);
	}
	END_FOR_EACH_REG_UNC_OPERATION;

	SEP_DRV_LOG_TRACE_OUT("");
}

/*!
 * @fn         static VOID UNC_MSR_Disable_PMU(PVOID)
 *
 * @brief      Set the enable bit for all the evsel registers
 *
 * @param      None
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
static VOID UNC_MSR_Disable_PMU(PVOID param)
{
	U32 dev_idx;
	U32 this_cpu;
	CPU_STATE pcpu;
	U64 reg_val = 0;

	SEP_DRV_LOG_TRACE_IN("Param: %p.", param);

	dev_idx = *((U32 *)param);
	this_cpu = CONTROL_THIS_CPU();
	pcpu = &pcb[this_cpu];

	if (!CPU_STATE_socket_master(pcpu)) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!CPU_STATE_socket_master).");
		return;
	}

	FOR_EACH_REG_UNC_OPERATION(pecb, dev_idx, idx, PMU_OPERATION_DISABLE)
	{
		reg_val = ECB_entries_reg_value(pecb, idx);
		if (ECB_entries_reg_rw_type(pecb, idx) ==
		    PMU_REG_RW_READ_WRITE) {
			reg_val = SYS_Read_MSR(ECB_entries_reg_id(pecb, idx));
			if (ECB_entries_reg_type(pecb, idx) ==
			    PMU_REG_UNIT_CTRL) {
				reg_val |= ECB_entries_reg_value(pecb, idx);
			} else {
				reg_val &= ECB_entries_reg_value(pecb, idx);
			}
		}
		SYS_Write_MSR(ECB_entries_reg_id(pecb, idx), reg_val);
	}
	END_FOR_EACH_REG_UNC_OPERATION;

	SEP_DRV_LOG_TRACE_OUT("");
}

/*!
 * @fn static VOID UNC_MSR_Read_PMU_Data(param)
 *
 * @param    param    The read thread node to process
 * @param    id       The id refers to the device index
 *
 * @return   None     No return needed
 *
 * @brief    Read the Uncore count data and store into the buffer
 * Let us say we have 2 core events in a dual socket JKTN;
 * The start_index will be at 32 as it will 2 events in 16 CPU per socket
 * The position for first event of QPI will be computed based on its event
 *
 */
static VOID UNC_MSR_Read_PMU_Data(PVOID param)
{
	U32 j = 0;
	U32 dev_idx;
	U32 this_cpu;
	U32 package_num = 0;
	U64 *buffer;
	CPU_STATE pcpu;
	U32 cur_grp;
	ECB pecb;

	SEP_DRV_LOG_TRACE_IN("Param: %p.", param);

	dev_idx = *((U32 *)param);
	this_cpu = CONTROL_THIS_CPU();
	buffer = read_counter_info;
	pcpu = &pcb[this_cpu];
	package_num = core_to_package_map[this_cpu];
	cur_grp = LWPMU_DEVICE_cur_group(&devices[(dev_idx)])[package_num];
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[(dev_idx)])[cur_grp];

	// NOTE THAT the read_pmu function on for EMON collection.
	if (!DRV_CONFIG_emon_mode(drv_cfg)) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!emon_mode).");
		return;
	}
	if (!CPU_STATE_socket_master(pcpu)) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!CPU_STATE_socket_master).");
		return;
	}
	if (!pecb) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!pecb).");
		return;
	}

	//Read in the counts into temporary buffer
	FOR_EACH_REG_UNC_OPERATION(pecb, dev_idx, idx, PMU_OPERATION_READ)
	{
		if (ECB_entries_event_scope(pecb, idx) == SYSTEM_EVENT) {
			j = ECB_entries_uncore_buffer_offset_in_system(pecb,
								       idx);
		} else {
			j = EMON_BUFFER_UNCORE_PACKAGE_EVENT_OFFSET(
				package_num,
				EMON_BUFFER_DRIVER_HELPER_num_entries_per_package(
					emon_buffer_driver_helper),
				ECB_entries_uncore_buffer_offset_in_package(
					pecb, idx));
		}

		buffer[j] = SYS_Read_MSR(ECB_entries_reg_id(pecb, idx));
		SEP_DRV_LOG_TRACE("j=%u, value=%llu, cpu=%u, event_id=%u", j,
				  buffer[j], this_cpu,
				  ECB_entries_core_event_id(pecb, idx));
	}
	END_FOR_EACH_REG_UNC_OPERATION;

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn       static VOID UNC_MSR_Trigger_Read(id)
 *
 * @param    id       Device index
 *
 * @return   None     No return needed
 *
 * @brief    Read the Uncore data from counters and store into buffer
 */
static VOID UNC_MSR_Trigger_Read(PVOID param, U32 id)
{
	U32 this_cpu;
	U32 package_num;
	U32 cur_grp;
	ECB pecb;
	U32 index = 0;
	U64 diff = 0;
	U64 value;
	U64 *data;

	SEP_DRV_LOG_TRACE_IN("Param: %p, id: %u.", param, id);

	this_cpu = CONTROL_THIS_CPU();
	package_num = core_to_package_map[this_cpu];
	cur_grp = LWPMU_DEVICE_cur_group(&devices[id])[package_num];
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[id])[cur_grp];

	// Write GroupID
	data = (U64 *)((S8 *)param + ECB_group_offset(pecb));
	*data = cur_grp + 1;
	//Read in the counts into uncore buffer
	FOR_EACH_REG_UNC_OPERATION(pecb, id, idx, PMU_OPERATION_READ)
	{
		value = SYS_Read_MSR(ECB_entries_reg_id(pecb, idx));
		//check for overflow
		if (value <
		    LWPMU_DEVICE_prev_value(&devices[id])[package_num][index]) {
			diff = LWPMU_DEVICE_counter_mask(&devices[id]) -
			       LWPMU_DEVICE_prev_value(
				       &devices[id])[package_num][index];
			diff += value;
		} else {
			diff = value -
			       LWPMU_DEVICE_prev_value(
				       &devices[id])[package_num][index];
		}
		LWPMU_DEVICE_acc_value(
			&devices[id])[package_num][cur_grp][index] += diff;
		LWPMU_DEVICE_prev_value(&devices[id])[package_num][index] =
			value;
		data = (U64 *)((S8 *)param +
			       ECB_entries_counter_event_offset(pecb, idx));
		*data = LWPMU_DEVICE_acc_value(
			&devices[id])[package_num][cur_grp][index];
		index++;
	}
	END_FOR_EACH_REG_UNC_OPERATION;

	SEP_DRV_LOG_TRACE_OUT("");
}

/*
 * Initialize the dispatch table
 */

DISPATCH_NODE unc_msr_dispatch = { .init = NULL,
				   .fini = NULL,
				   .write = UNC_MSR_Write_PMU,
				   .freeze = UNC_MSR_Disable_PMU,
				   .restart = UNC_MSR_Enable_PMU,
				   .read_data = UNC_MSR_Read_PMU_Data,
				   .check_overflow = NULL,
				   .swap_group = NULL,
				   .read_lbrs = NULL,
				   .cleanup = UNC_COMMON_MSR_Clean_Up,
				   .hw_errata = NULL,
				   .read_power = NULL,
				   .check_overflow_errata = NULL,
				   .read_counts = NULL,
				   .check_overflow_gp_errata = NULL,
				   .read_ro = NULL,
				   .platform_info = NULL,
				   .trigger_read = UNC_MSR_Trigger_Read,
				   .scan_for_uncore = NULL,
				   .read_metrics = NULL };
