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
#include "inc/utility.h"
#include "inc/valleyview_sochap.h"

static U64 *uncore_current_data;
static U64 *uncore_to_read_data;
extern DRV_CONFIG drv_cfg;

extern U64 *read_counter_info;
extern VOID SOCPERF_Read_Data3(PVOID data_buffer);

/*!
 * @fn         static VOID valleyview_VISA_Initialize(PVOID)
 *
 * @brief      Initialize any registers or addresses
 *
 * @param      param
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
static VOID valleyview_VISA_Initialize(VOID *param)
{
	SEP_DRV_LOG_TRACE_IN("Param: %p.", param);

	// Allocate memory for reading GMCH counter values + the group id
	if (!uncore_current_data) {
		uncore_current_data = CONTROL_Allocate_Memory(
			(VLV_CHAP_MAX_COUNTERS + 1) * sizeof(U64));
		if (!uncore_current_data) {
			SEP_DRV_LOG_ERROR_TRACE_OUT(
				"Early exit (uncore_current_data is NULL!).");
			return;
		}
	}
	if (!uncore_to_read_data) {
		uncore_to_read_data = CONTROL_Allocate_Memory(
			(VLV_CHAP_MAX_COUNTERS + 1) * sizeof(U64));
		if (!uncore_to_read_data) {
			SEP_DRV_LOG_ERROR_TRACE_OUT(
				"Early exit (uncore_to_read_data is NULL!).");
			return;
		}
	}

	SEP_DRV_LOG_TRACE_OUT("");
}

/*!
 * @fn         static VOID valleyview_VISA_Enable_PMU(PVOID)
 *
 * @brief      Start counting
 *
 * @param      param - device index
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
static VOID valleyview_VISA_Enable_PMU(PVOID param)
{
	U32 this_cpu;
	CPU_STATE pcpu;

	SEP_DRV_LOG_TRACE_IN("Param: %p.", param);

	this_cpu = CONTROL_THIS_CPU();
	pcpu = &pcb[this_cpu];

	if (!CPU_STATE_system_master(pcpu)) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!system_master).");
		return;
	}

	SEP_DRV_LOG_TRACE("Starting the counters...");
	if (uncore_current_data) {
		memset(uncore_current_data, 0,
		       (VLV_CHAP_MAX_COUNTERS + 1) * sizeof(U64));
	}
	if (uncore_to_read_data) {
		memset(uncore_to_read_data, 0,
		       (VLV_CHAP_MAX_COUNTERS + 1) * sizeof(U64));
	}

	SEP_DRV_LOG_TRACE_OUT("");
}

/*!
 * @fn         static VOID valleyview_VISA_Disable_PMU(PVOID)
 *
 * @brief      Unmap the virtual address when sampling/driver stops
 *
 * @param      param - device index
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
static VOID valleyview_VISA_Disable_PMU(PVOID param)
{
	U32 this_cpu;
	CPU_STATE pcpu;
	U32 cur_driver_state;

	SEP_DRV_LOG_TRACE_IN("Param: %p.", param);

	this_cpu = CONTROL_THIS_CPU();
	pcpu = &pcb[this_cpu];
	cur_driver_state = GET_DRIVER_STATE();

	if (!CPU_STATE_system_master(pcpu)) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!system_master).");
		return;
	}
	SEP_DRV_LOG_TRACE("Stopping the counters...");
	if (cur_driver_state == DRV_STATE_PREPARE_STOP ||
	    cur_driver_state == DRV_STATE_STOPPED ||
	    cur_driver_state == DRV_STATE_TERMINATING) {
		uncore_current_data = CONTROL_Free_Memory(uncore_current_data);
		uncore_to_read_data = CONTROL_Free_Memory(uncore_to_read_data);
	}

	SEP_DRV_LOG_TRACE_OUT("");
}

/*!
 * @fn         static VOID valleyview_VISA_Clean_Up(PVOID)
 *
 * @brief      Reset any registers or addresses
 *
 * @param      param
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
static VOID valleyview_VISA_Clean_Up(VOID *param)
{
	SEP_DRV_LOG_TRACE_IN("Param: %p.", param);
	SEP_DRV_LOG_TRACE_OUT("Empty function.");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn valleyview_VISA_Read_PMU_Data(param)
 *
 * @param    param    The device index
 *
 * @return   None     No return needed
 *
 * @brief    Read the Uncore count data and store into the buffer param;
 *
 */
static VOID valleyview_VISA_Read_PMU_Data(PVOID param)
{
	U32 j;
	U64 *buffer = read_counter_info;
	U32 dev_idx;
	U32 this_cpu;
	CPU_STATE pcpu;
	U32 package_num;
	U32 event_index = 0;
	U32 cur_grp;
	ECB pecb;
	U64 counter_buffer[VLV_CHAP_MAX_COUNTERS + 1];

	SEP_DRV_LOG_TRACE_IN("Param: %p.", param);

	dev_idx = *((U32 *)param);
	this_cpu = CONTROL_THIS_CPU();
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
		SEP_DRV_LOG_TRACE_OUT("Early exit (!socket_master).");
		return;
	}
	if (!pecb) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!pecb).");
		return;
	}

	SOCPERF_Read_Data3((void*)counter_buffer);

	FOR_EACH_REG_UNC_OPERATION(pecb, dev_idx, idx, PMU_OPERATION_READ)
	{
		//the buffer index for this PMU needs to account for each event
		j = ECB_entries_uncore_buffer_offset_in_system(pecb, idx);
		buffer[j] = counter_buffer[event_index + 1];
		event_index++;
		SEP_DRV_LOG_TRACE("j=%u, value=%llu, cpu=%u", j, buffer[j],
				  this_cpu);
	}
	END_FOR_EACH_REG_UNC_OPERATION;

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn valleyview_Trigger_Read()
 *
 * @param    None
 *
 * @return   None     No return needed
 *
 * @brief    Read the SoCHAP counters when timer is triggered
 *
 */
static VOID valleyview_Trigger_Read(PVOID param, U32 id)
{
	U64 *data = (U64 *)param;
	U32 cur_grp;
	ECB pecb;
	U32 this_cpu;
	U32 package_num;

	SEP_DRV_LOG_TRACE_IN("Param: %p, , id: %u.", param, id);

	this_cpu = CONTROL_THIS_CPU();
	package_num = core_to_package_map[this_cpu];
	cur_grp = LWPMU_DEVICE_cur_group(&devices[id])[package_num];
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[id])[cur_grp];

	// group id
	data = (U64 *)((S8 *)data + ECB_group_offset(pecb));
	SOCPERF_Read_Data3((void*)data);

	SEP_DRV_LOG_TRACE_OUT("");
}

/*
 * Initialize the dispatch table
 */
DISPATCH_NODE valleyview_visa_dispatch = {
	.init = valleyview_VISA_Initialize,
	.fini = NULL,
	.write = NULL,
	.freeze = valleyview_VISA_Disable_PMU,
	.restart = valleyview_VISA_Enable_PMU,
	.read_data = valleyview_VISA_Read_PMU_Data,
	.check_overflow = NULL,
	.swap_group = NULL,
	.read_lbrs = NULL,
	.cleanup = valleyview_VISA_Clean_Up,
	.hw_errata = NULL,
	.read_power = NULL,
	.check_overflow_errata = NULL,
	.read_counts = NULL,
	.check_overflow_gp_errata = NULL,
	.read_ro = NULL,
	.platform_info = NULL,
	.trigger_read = valleyview_Trigger_Read,
	.scan_for_uncore = NULL,
	.read_metrics = NULL
};
