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

#include <linux/version.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/pci.h>
#include "lwpmudrv_defines.h"
#include "lwpmudrv_types.h"

#if defined(PCI_HELPERS_API)
#include <asm/intel_scu_ipc.h>
#include <asm/intel-mid.h>
#endif

#include "rise_errors.h"
#include "lwpmudrv_ecb.h"
#include "lwpmudrv_struct.h"
#include "lwpmudrv_chipset.h"
#include "inc/lwpmudrv.h"
#include "inc/control.h"
#include "inc/utility.h"
#include "inc/gmch.h"
#include "inc/pci.h"

// global variables for determining which register offsets to use
static U32 gmch_register_read; // value=0 indicates invalid read register
static U32 gmch_register_write; // value=0 indicates invalid write register
static U32 number_of_events;

//global variable for reading GMCH counter values
static U64 *gmch_current_data;
static U64 *gmch_to_read_data;

// global variable for tracking number of overflows per GMCH counter
static U32 gmch_overflow[MAX_CHIPSET_COUNTERS];
static U64 last_gmch_count[MAX_CHIPSET_COUNTERS];

extern DRV_CONFIG drv_cfg;
extern CHIPSET_CONFIG pma;
extern CPU_STATE pcb;

/*
 * @fn        gmch_PCI_Read32(address)
 *
 * @brief     Read the 32bit value specified by the address
 *
 * @return    the read value
 *
 */
#if defined(PCI_HELPERS_API)
#define gmch_PCI_Read32 intel_mid_msgbus_read32_raw
#else
static U32 gmch_PCI_Read32(unsigned long address)
{
	U32 read_value = 0;

	SEP_DRV_LOG_TRACE_IN("Address: %lx.", address);

	PCI_Write_U32(0, 0, 0, GMCH_MSG_CTRL_REG, (U32)address);
	read_value = PCI_Read_U32(0, 0, 0, GMCH_MSG_DATA_REG);

	SEP_DRV_LOG_TRACE_OUT("Res: %x.", read_value);
	return read_value;
}
#endif

/*
 * @fn        gmch_PCI_Write32(address, data)
 *
 * @brief     Write the 32bit value into the address specified
 *
 * @return    None
 *
 */
#if defined(PCI_HELPERS_API)
#define gmch_PCI_Write32 intel_mid_msgbus_write32_raw
#else
static void gmch_PCI_Write32(unsigned long address, unsigned long data)
{
	SEP_DRV_LOG_TRACE_IN("Address: %lx, data: %lx.", address, data);

	PCI_Write_U32(0, 0, 0, GMCH_MSG_DATA_REG, data);
	PCI_Write_U32(0, 0, 0, GMCH_MSG_CTRL_REG, address);

	SEP_DRV_LOG_TRACE_OUT("");
}
#endif

/*
 * @fn        gmch_Check_Enabled()
 *
 * @brief     Read GMCH PMON capabilities
 *
 * @param     None
 *
 * @return    GMCH enable bits
 *
 */
static ULONG gmch_Check_Enabled(void)
{
	ULONG enabled_value;

	SEP_DRV_LOG_TRACE_IN("");

	enabled_value =
		gmch_PCI_Read32(GMCH_PMON_CAPABILITIES + gmch_register_read);

	SEP_DRV_LOG_TRACE_OUT("Res: %lx.", enabled_value);
	return enabled_value;
}

/*
 * @fn        gmch_Init_Chipset()
 *
 * @brief     Initialize GMCH Counters.  See note below.
 *
 * @param     None
 *
 * @note      This function must be called BEFORE any other function
 *            in this file!
 *
 * @return    VT_SUCCESS if successful, error otherwise
 *
 */
static U32 gmch_Init_Chipset(void)
{
	int i;
	CHIPSET_SEGMENT cs;
	CHIPSET_SEGMENT gmch_chipset_seg;

	SEP_DRV_LOG_TRACE_IN("");

	cs = &CHIPSET_CONFIG_gmch(pma);
	gmch_chipset_seg = &CHIPSET_CONFIG_gmch(pma);

	// configure read/write registers offsets according to usermode setting
	if (cs) {
		gmch_register_read = CHIPSET_SEGMENT_read_register(cs);
		gmch_register_write = CHIPSET_SEGMENT_write_register(cs);
		;
	}
	if (gmch_register_read == 0 || gmch_register_write == 0) {
		SEP_DRV_LOG_ERROR_TRACE_OUT(
			"VT_CHIPSET_CONFIG_FAILED(Invalid GMCH read/write registers!)");
		return VT_CHIPSET_CONFIG_FAILED;
	}

	number_of_events = CHIPSET_SEGMENT_total_events(gmch_chipset_seg);
	SEP_DRV_LOG_INIT("Number of chipset events %d.", number_of_events);

	// Allocate memory for reading GMCH counter values + the group id
	gmch_current_data =
		CONTROL_Allocate_Memory((number_of_events + 1) * sizeof(U64));
	if (!gmch_current_data) {
		SEP_DRV_LOG_ERROR_TRACE_OUT("OS_NO_MEM (!gmch_current_data).");
		return OS_NO_MEM;
	}
	gmch_to_read_data =
		CONTROL_Allocate_Memory((number_of_events + 1) * sizeof(U64));
	if (!gmch_to_read_data) {
		SEP_DRV_LOG_ERROR_TRACE_OUT("OS_NO_MEM (!gmch_to_read_data).");
		return OS_NO_MEM;
	}

	if (!DRV_CONFIG_enable_chipset(drv_cfg)) {
		SEP_DRV_LOG_TRACE_OUT(
			"VT_SUCCESS (!DRV_CONFIG_enable_chipset(drv_cfg)).");
		return VT_SUCCESS;
	}

	if (!CHIPSET_CONFIG_gmch_chipset(pma)) {
		SEP_DRV_LOG_TRACE_OUT(
			"VT_SUCCESS (!CHIPSET_CONFIG_gmch_chipset(drv_cfg)).");
		return VT_SUCCESS;
	}
	// initialize the GMCH per-counter overflow numbers
	for (i = 0; i < MAX_CHIPSET_COUNTERS; i++) {
		gmch_overflow[i] = 0;
		last_gmch_count[i] = 0;
	}

	// disable fixed and GP counters
	gmch_PCI_Write32(GMCH_PMON_GLOBAL_CTRL + gmch_register_write,
			 0x00000000);
	// clear fixed counter filter
	gmch_PCI_Write32(GMCH_PMON_FIXED_CTR_CTRL + gmch_register_write,
			 0x00000000);

	SEP_DRV_LOG_TRACE_OUT("VT_SUCCESS.");
	return VT_SUCCESS;
}

/*
 * @fn        gmch_Start_Counters()
 *
 * @brief     Start the GMCH Counters.
 *
 * @param     None
 *
 * @return    None
 *
 */
static VOID gmch_Start_Counters(void)
{
	SEP_DRV_LOG_TRACE_IN("");

	// reset and start chipset counters
	if (pma == NULL) {
		SEP_DRV_LOG_ERROR("gmch_Start_Counters: ERROR pma=NULL.");
	}

	// enable fixed and GP counters
	gmch_PCI_Write32(GMCH_PMON_GLOBAL_CTRL + gmch_register_write,
			 0x0001000F);
	// enable fixed counter filter
	gmch_PCI_Write32(GMCH_PMON_FIXED_CTR_CTRL + gmch_register_write,
			 0x00000001);

	SEP_DRV_LOG_TRACE_OUT("");
}

/*
 * @fn        gmch_Trigger_Read()
 *
 * @brief     Read the GMCH counters through PCI Config space
 *
 * @return    None
 *
 */
static VOID gmch_Trigger_Read(void)
{
	U64 *data;
	int i, data_index;
	U64 val;
	U64 *gmch_data;
	U32 counter_data_low;
	U32 counter_data_high;
	U64 counter_data;
	U64 cmd_register_low_read;
	U64 cmd_register_high_read;
	U32 gp_counter_index = 0;
	U64 overflow;
	U32 cur_driver_state;

	CHIPSET_SEGMENT gmch_chipset_seg;
	CHIPSET_EVENT chipset_events;
	U64 *temp;

	SEP_DRV_LOG_TRACE_IN("");

	cur_driver_state = GET_DRIVER_STATE();

	if (!IS_COLLECTING_STATE(cur_driver_state)) {
		SEP_DRV_LOG_ERROR_TRACE_OUT("Invalid driver state!");
		return;
	}

	if (pma == NULL) {
		SEP_DRV_LOG_ERROR_TRACE_OUT("pma is NULL!");
		return;
	}

	if (gmch_current_data == NULL) {
		SEP_DRV_LOG_ERROR_TRACE_OUT("gmch_current_data is NULL!");
		return;
	}

	if (CHIPSET_CONFIG_gmch_chipset(pma) == 0) {
		SEP_DRV_LOG_ERROR_TRACE_OUT(
			"CHIPSET_CONFIG_gmch_chipset(pma) is NULL!");
		return;
	}

	data = gmch_current_data;
	data_index = 0;

	preempt_disable();
	SYS_Local_Irq_Disable();
	gmch_chipset_seg = &CHIPSET_CONFIG_gmch(pma);
	chipset_events = CHIPSET_SEGMENT_events(gmch_chipset_seg);

	// Write GroupID
	data[data_index] = 1;
	// Increment the data index as the event id starts from zero
	data_index++;

	// GMCH data will be written as gmch_data[0], gmch_data[1], ...
	gmch_data = data + data_index;

	// read the GMCH counters and add them into the sample record

	// iterate through GMCH counters configured to collect on events
	for (i = 0; i < CHIPSET_SEGMENT_total_events(gmch_chipset_seg); i++) {
		U32 event_id = CHIPSET_EVENT_event_id(&chipset_events[i]);
		// read count for fixed GMCH counter event
		if (event_id == 0) {
			cmd_register_low_read =
				GMCH_PMON_FIXED_CTR0 + gmch_register_read;
			data[data_index++] =
				(U64)gmch_PCI_Read32(cmd_register_low_read);
			overflow = GMCH_PMON_FIXED_CTR_OVF_VAL;
		} else {
			// read count for general GMCH counter event
			switch (gp_counter_index) {
			case 0:
			default:
				cmd_register_low_read = GMCH_PMON_GP_CTR0_L +
							gmch_register_read;
				cmd_register_high_read = GMCH_PMON_GP_CTR0_H +
							 gmch_register_read;
				break;

			case 1:
				cmd_register_low_read = GMCH_PMON_GP_CTR1_L +
							gmch_register_read;
				cmd_register_high_read = GMCH_PMON_GP_CTR1_H +
							 gmch_register_read;
				break;

			case 2:
				cmd_register_low_read = GMCH_PMON_GP_CTR2_L +
							gmch_register_read;
				cmd_register_high_read = GMCH_PMON_GP_CTR2_H +
							 gmch_register_read;
				break;

			case 3:
				cmd_register_low_read = GMCH_PMON_GP_CTR3_L +
							gmch_register_read;
				cmd_register_high_read = GMCH_PMON_GP_CTR3_H +
							 gmch_register_read;
				break;
			}
			counter_data_low =
				gmch_PCI_Read32(cmd_register_low_read);
			counter_data_high =
				gmch_PCI_Read32(cmd_register_high_read);
			counter_data = (U64)counter_data_high;
			data[data_index++] =
				(counter_data << 32) + counter_data_low;
			overflow = GMCH_PMON_GP_CTR_OVF_VAL;
			gp_counter_index++;
		}

		/* Compute the running count of the event. */
		gmch_data[i] &= overflow;
		val = gmch_data[i];
		if (gmch_data[i] < last_gmch_count[i]) {
			gmch_overflow[i]++;
		}
		gmch_data[i] = gmch_data[i] + gmch_overflow[i] * overflow;
		last_gmch_count[i] = val;
	}

	temp = gmch_to_read_data;
	gmch_to_read_data = gmch_current_data;
	gmch_current_data = temp;
	SYS_Local_Irq_Enable();
	preempt_enable();

	SEP_DRV_LOG_TRACE_OUT("");
}

/*
 * @fn        gmch_Read_Counters()
 *
 * @brief     Copy the GMCH data to the sampling data stream.
 *
 * @param     param - pointer to data stream where samples are to be written
 *
 * @return    None
 *
 */
static VOID gmch_Read_Counters(PVOID param)
{
	U64 *data;
	int i;
	U32 cur_driver_state;

	SEP_DRV_LOG_TRACE_IN("Param: %p.", param);

	cur_driver_state = GET_DRIVER_STATE();

	if (!IS_COLLECTING_STATE(cur_driver_state)) {
		SEP_DRV_LOG_ERROR_TRACE_OUT("Invalid driver state!");
		return;
	}

	if (pma == NULL) {
		SEP_DRV_LOG_ERROR_TRACE_OUT("pma is NULL!");
		return;
	}

	if (param == NULL) {
		SEP_DRV_LOG_ERROR_TRACE_OUT("param is NULL!");
		return;
	}

	if (gmch_to_read_data == NULL) {
		SEP_DRV_LOG_ERROR_TRACE_OUT("gmch_to_read_data is NULL!");
		return;
	}

	/*
	 * Account for the group id that is placed at start of chipset array
	 * Number of data elements to be transferred is number_of_events + 1.
	 */
	data = param;
	for (i = 0; i < number_of_events + 1; i++) {
		data[i] = gmch_to_read_data[i];
		SEP_DRV_LOG_TRACE(
			"Interrupt gmch read counters data %d is: 0x%llx.", i,
			data[i]);
	}

	SEP_DRV_LOG_TRACE_OUT("");
}

/*
 * @fn        gmch_Stop_Counters()
 *
 * @brief     Stop the GMCH counters
 *
 * @param     None
 *
 * @return    None
 *
 */
static VOID gmch_Stop_Counters(void)
{
	SEP_DRV_LOG_TRACE_IN("");

	// stop and reset the chipset counters
	number_of_events = 0;
	if (pma == NULL) {
		SEP_DRV_LOG_ERROR("gmch_Stop_Counters: pma=NULL.");
	}

	// disable fixed and GP counters
	gmch_PCI_Write32(GMCH_PMON_GLOBAL_CTRL + gmch_register_write,
			 0x00000000);
	gmch_PCI_Write32(GMCH_PMON_FIXED_CTR_CTRL + gmch_register_write,
			 0x00000000);

	SEP_DRV_LOG_TRACE_OUT("");
}

/*
 * @fn        gmch_Fini_Chipset()
 *
 * @brief     Reset GMCH to state where it can be used again.
 *            Called at cleanup phase.
 *
 * @param     None
 *
 * @return    None
 *
 */
static VOID gmch_Fini_Chipset(void)
{
	SEP_DRV_LOG_TRACE_IN("");

	if (!gmch_Check_Enabled()) {
		SEP_DRV_LOG_WARNING("GMCH is not enabled!");
	}

	gmch_current_data = CONTROL_Free_Memory(gmch_current_data);
	gmch_to_read_data = CONTROL_Free_Memory(gmch_to_read_data);

	SEP_DRV_LOG_TRACE_OUT("");
}

//
// Initialize the GMCH chipset dispatch table
//

CS_DISPATCH_NODE gmch_dispatch = { .init_chipset = gmch_Init_Chipset,
				   .start_chipset = gmch_Start_Counters,
				   .read_counters = gmch_Read_Counters,
				   .stop_chipset = gmch_Stop_Counters,
				   .fini_chipset = gmch_Fini_Chipset,
				   .Trigger_Read = gmch_Trigger_Read };
