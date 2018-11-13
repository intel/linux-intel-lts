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
#include "inc/pci.h"
#include "inc/unc_gt.h"

extern U64 *read_counter_info;
extern EMON_BUFFER_DRIVER_HELPER emon_buffer_driver_helper;

static U64 unc_gt_virtual_address;
static SEP_MMIO_NODE unc_gt_map;
static U32 unc_gt_rc6_reg1;
static U32 unc_gt_rc6_reg2;
static U32 unc_gt_clk_gt_reg1;
static U32 unc_gt_clk_gt_reg2;
static U32 unc_gt_clk_gt_reg3;
static U32 unc_gt_clk_gt_reg4;

/*!
 * @fn          static VOID unc_gt_Write_PMU(VOID*)
 *
 * @brief       Initial write of PMU registers
 *              Walk through the enties and write the value of the register accordingly.
 *
 * @param       device id
 *
 * @return      None
 *
 * <I>Special Notes:</I>
 */
static VOID unc_gt_Write_PMU(VOID *param)
{
	U32 dev_idx;
	ECB pecb;
	DRV_PCI_DEVICE_ENTRY_NODE dpden;
	U64 device_id;
	U32 vendor_id;
	U64 bar_lo;
	U32 offset_delta;
	U32 tmp_value;
	U32 this_cpu;
	U32 value;
	CPU_STATE pcpu;

	SEP_DRV_LOG_TRACE_IN("Param: %p.", param);

	dev_idx = *((U32 *)param);
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[dev_idx])[0];
	this_cpu = CONTROL_THIS_CPU();
	pcpu = &pcb[this_cpu];

	if (!CPU_STATE_system_master(pcpu)) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!system_master).");
		return;
	}

	dpden = ECB_pcidev_entry_node(pecb);
	value = PCI_Read_U32(DRV_PCI_DEVICE_ENTRY_bus_no(&dpden),
			     DRV_PCI_DEVICE_ENTRY_dev_no(&dpden),
			     DRV_PCI_DEVICE_ENTRY_func_no(&dpden), 0);
	vendor_id = DRV_GET_PCI_VENDOR_ID(value);
	device_id = DRV_GET_PCI_DEVICE_ID(value);

	if (DRV_IS_INTEL_VENDOR_ID(vendor_id) &&
	    DRV_IS_GT_DEVICE_ID(device_id)) {
		SEP_DRV_LOG_TRACE("Found Desktop GT.");
	}

	bar_lo = PCI_Read_U32(DRV_PCI_DEVICE_ENTRY_bus_no(&dpden),
			      DRV_PCI_DEVICE_ENTRY_dev_no(&dpden),
			      DRV_PCI_DEVICE_ENTRY_func_no(&dpden),
			      DRV_PCI_DEVICE_ENTRY_bar_offset(&dpden));
	bar_lo &= UNC_GT_BAR_MASK;

	PCI_Map_Memory(&unc_gt_map, bar_lo, GT_MMIO_SIZE);
	unc_gt_virtual_address = SEP_MMIO_NODE_virtual_address(&unc_gt_map);

	FOR_EACH_PCI_DATA_REG_RAW(pecb, i, dev_idx)
	{
		offset_delta = ECB_entries_reg_offset(pecb, i);
		// this is needed for overflow detection of the accumulators.
		if (LWPMU_DEVICE_counter_mask(&devices[dev_idx]) == 0) {
			LWPMU_DEVICE_counter_mask(&devices[dev_idx]) =
				(U64)ECB_entries_max_bits(pecb, i);
		}
	}
	END_FOR_EACH_PCI_CCCR_REG_RAW;

	//enable the global control to clear the counter first
	SYS_Write_MSR(PERF_GLOBAL_CTRL, ECB_entries_reg_value(pecb, 0));
	FOR_EACH_PCI_CCCR_REG_RAW(pecb, i, dev_idx)
	{
		offset_delta = ECB_entries_reg_offset(pecb, i);
		if (offset_delta == PERF_GLOBAL_CTRL) {
			continue;
		}
		PCI_MMIO_Write_U32(unc_gt_virtual_address, offset_delta,
				   GT_CLEAR_COUNTERS);

		SEP_DRV_LOG_TRACE("CCCR offset delta is 0x%x W is clear ctrs.",
				  offset_delta);
	}
	END_FOR_EACH_PCI_CCCR_REG_RAW;

	//disable the counters
	SYS_Write_MSR(PERF_GLOBAL_CTRL, 0LL);

	FOR_EACH_PCI_CCCR_REG_RAW(pecb, i, dev_idx)
	{
		offset_delta = ECB_entries_reg_offset(pecb, i);
		if (offset_delta == PERF_GLOBAL_CTRL) {
			continue;
		}
		PCI_MMIO_Write_U32(unc_gt_virtual_address, offset_delta,
				   ((U32)ECB_entries_reg_value(pecb, i)));
		tmp_value =
			PCI_MMIO_Read_U32(unc_gt_virtual_address, offset_delta);

		// remove compiler warning on unused variables
		if (tmp_value) {
		}

		SEP_DRV_LOG_TRACE(
			"CCCR offset delta is 0x%x R is 0x%x W is 0x%llx.",
			offset_delta, tmp_value,
			ECB_entries_reg_value(pecb, i));
	}
	END_FOR_EACH_PCI_CCCR_REG_RAW;

	SEP_DRV_LOG_TRACE_OUT("");
}

/*!
 * @fn          static VOID unc_gt_Disable_RC6_Clock_Gating(void)
 *
 * @brief       This snippet of code allows GT events to count by
 *              disabling settings related to clock gating/power
 * @param       none
 *
 * @return      None
 *
 * <I>Special Notes:</I>
 */
static VOID unc_gt_Disable_RC6_Clock_Gating(void)
{
	U32 tmp;

	SEP_DRV_LOG_TRACE_IN("");

	// Disable RC6
	unc_gt_rc6_reg1 =
		PCI_MMIO_Read_U32(unc_gt_virtual_address, UNC_GT_RC6_REG1);
	tmp = unc_gt_rc6_reg1 | UNC_GT_RC6_REG1_OR_VALUE;
	unc_gt_rc6_reg2 =
		PCI_MMIO_Read_U32(unc_gt_virtual_address, UNC_GT_RC6_REG2);

	PCI_MMIO_Write_U32(unc_gt_virtual_address, UNC_GT_RC6_REG2,
			   UNC_GT_RC6_REG2_VALUE);
	PCI_MMIO_Write_U32(unc_gt_virtual_address, UNC_GT_RC6_REG1, tmp);

	SEP_DRV_LOG_TRACE("Original value of RC6 rc6_1 = 0x%x, rc6_2 = 0x%x.",
			  unc_gt_rc6_reg1, unc_gt_rc6_reg2);

	// Disable clock gating
	// Save
	unc_gt_clk_gt_reg1 =
		PCI_MMIO_Read_U32(unc_gt_virtual_address, UNC_GT_GCPUNIT_REG1);
	unc_gt_clk_gt_reg2 =
		PCI_MMIO_Read_U32(unc_gt_virtual_address, UNC_GT_GCPUNIT_REG2);
	unc_gt_clk_gt_reg3 =
		PCI_MMIO_Read_U32(unc_gt_virtual_address, UNC_GT_GCPUNIT_REG3);
	unc_gt_clk_gt_reg4 =
		PCI_MMIO_Read_U32(unc_gt_virtual_address, UNC_GT_GCPUNIT_REG4);

	SEP_DRV_LOG_TRACE("Original value of RC6 ck_1 = 0x%x, ck_2 = 0x%x.",
			  unc_gt_clk_gt_reg1, unc_gt_clk_gt_reg2);
	SEP_DRV_LOG_TRACE("Original value of RC6 ck_3 = 0x%x, ck_4 = 0x%x.",
			  unc_gt_clk_gt_reg3, unc_gt_clk_gt_reg4);

	// Disable
	PCI_MMIO_Write_U32(unc_gt_virtual_address, UNC_GT_GCPUNIT_REG1,
			   UNC_GT_GCPUNIT_REG1_VALUE);
	PCI_MMIO_Write_U32(unc_gt_virtual_address, UNC_GT_GCPUNIT_REG2,
			   UNC_GT_GCPUNIT_REG2_VALUE);
	PCI_MMIO_Write_U32(unc_gt_virtual_address, UNC_GT_GCPUNIT_REG3,
			   UNC_GT_GCPUNIT_REG3_VALUE);
	PCI_MMIO_Write_U32(unc_gt_virtual_address, UNC_GT_GCPUNIT_REG4,
			   UNC_GT_GCPUNIT_REG4_VALUE);

	SEP_DRV_LOG_TRACE_OUT("");
}

/*!
 * @fn          static VOID unc_gt_Restore_RC6_Clock_Gating(void)
 *
 * @brief       This snippet of code restores the system settings
 *              for clock gating/power
 * @param       none
 *
 * @return      None
 *
 * <I>Special Notes:</I>
 */
static VOID unc_gt_Restore_RC6_Clock_Gating(void)
{
	SEP_DRV_LOG_TRACE_IN("");

	PCI_MMIO_Write_U32(unc_gt_virtual_address, UNC_GT_RC6_REG2,
			   unc_gt_rc6_reg2);
	PCI_MMIO_Write_U32(unc_gt_virtual_address, UNC_GT_RC6_REG1,
			   unc_gt_rc6_reg1);

	PCI_MMIO_Write_U32(unc_gt_virtual_address, UNC_GT_GCPUNIT_REG1,
			   unc_gt_clk_gt_reg1);
	PCI_MMIO_Write_U32(unc_gt_virtual_address, UNC_GT_GCPUNIT_REG2,
			   unc_gt_clk_gt_reg2);
	PCI_MMIO_Write_U32(unc_gt_virtual_address, UNC_GT_GCPUNIT_REG3,
			   unc_gt_clk_gt_reg3);
	PCI_MMIO_Write_U32(unc_gt_virtual_address, UNC_GT_GCPUNIT_REG4,
			   unc_gt_clk_gt_reg4);

	SEP_DRV_LOG_TRACE_OUT("");
}

/*!
 * @fn         static VOID unc_gt_Enable_PMU(PVOID)
 *
 * @brief      Disable the clock gating and Set the global enable
 *
 * @param      device_id
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
static VOID unc_gt_Enable_PMU(PVOID param)
{
	U32 dev_idx;
	ECB pecb;
	U32 this_cpu;
	CPU_STATE pcpu;

	SEP_DRV_LOG_TRACE_IN("Param: %p.", param);

	dev_idx = *((U32 *)param);
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[dev_idx])[0];
	this_cpu = CONTROL_THIS_CPU();
	pcpu = &pcb[this_cpu];

	if (!CPU_STATE_system_master(pcpu)) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!system_master).");
		return;
	}

	unc_gt_Disable_RC6_Clock_Gating();

	if (pecb && GET_DRIVER_STATE() == DRV_STATE_RUNNING) {
		SYS_Write_MSR(PERF_GLOBAL_CTRL, ECB_entries_reg_value(pecb, 0));
		SEP_DRV_LOG_TRACE("Enabling GT Global control = 0x%llx.",
				  ECB_entries_reg_value(pecb, 0));
	}

	SEP_DRV_LOG_TRACE_OUT("");
}
/*!
 * @fn         static VOID unc_gt_Disable_PMU(PVOID)
 *
 * @brief      Unmap the virtual address when sampling/driver stops
 *             and restore system values for clock gating settings
 *
 * @param      None
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
static VOID unc_gt_Disable_PMU(PVOID param)
{
	U32 this_cpu;
	CPU_STATE pcpu;
	U32 cur_driver_state;

	SEP_DRV_LOG_TRACE_IN("Dummy param: %p.", param);

	this_cpu = CONTROL_THIS_CPU();
	pcpu = &pcb[this_cpu];
	cur_driver_state = GET_DRIVER_STATE();

	if (!CPU_STATE_system_master(pcpu)) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!system_master).");
		return;
	}
	unc_gt_Restore_RC6_Clock_Gating();

	if (unc_gt_virtual_address &&
	    (cur_driver_state == DRV_STATE_STOPPED ||
	     cur_driver_state == DRV_STATE_PREPARE_STOP ||
	     cur_driver_state == DRV_STATE_TERMINATING)) {
		SYS_Write_MSR(PERF_GLOBAL_CTRL, 0LL);
		PCI_Unmap_Memory(&unc_gt_map);
		unc_gt_virtual_address = 0;
	}

	SEP_DRV_LOG_TRACE_OUT("");
}

/*!
 * @fn unc_gt_Read_Counts(param, id)
 *
 * @param    param    The read thread node to process
 * @param    id       The id refers to the device index
 *
 * @return   None     No return needed
 *
 * @brief    Read the Uncore count data and store into the buffer param;
 *
 */
static VOID unc_gt_Read_Counts(PVOID param, U32 id)
{
	U64 *data = (U64 *)param;
	U32 cur_grp;
	ECB pecb;
	U32 offset_delta;
	U32 tmp_value_lo = 0;
	U32 tmp_value_hi = 0;
	GT_CTR_NODE gt_ctr_value;
	U32 this_cpu;
	U32 package_num;

	SEP_DRV_LOG_TRACE_IN("Param: %p, id: %u.", param, id);

	this_cpu = CONTROL_THIS_CPU();
	package_num = core_to_package_map[this_cpu];
	cur_grp = LWPMU_DEVICE_cur_group(&devices[id])[package_num];
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[id])[cur_grp];

	// Write GroupID
	data = (U64 *)((S8 *)data + ECB_group_offset(pecb));
	*data = cur_grp + 1;
	GT_CTR_NODE_value_reset(gt_ctr_value);

	//Read in the counts into temporary buffe
	FOR_EACH_PCI_DATA_REG_RAW(pecb, i, id)
	{
		offset_delta = ECB_entries_reg_offset(pecb, i);
		tmp_value_lo =
			PCI_MMIO_Read_U32(unc_gt_virtual_address, offset_delta);
		offset_delta = offset_delta + NEXT_ADDR_OFFSET;
		tmp_value_hi =
			PCI_MMIO_Read_U32(unc_gt_virtual_address, offset_delta);
		data = (U64 *)((S8 *)param +
			       ECB_entries_counter_event_offset(pecb, i));
		GT_CTR_NODE_low(gt_ctr_value) = tmp_value_lo;
		GT_CTR_NODE_high(gt_ctr_value) = tmp_value_hi;
		*data = GT_CTR_NODE_value(gt_ctr_value);
		SEP_DRV_LOG_TRACE("DATA offset delta is 0x%x R is 0x%llx.",
				  offset_delta,
				  GT_CTR_NODE_value(gt_ctr_value));
	}
	END_FOR_EACH_PCI_DATA_REG_RAW;

	SEP_DRV_LOG_TRACE_OUT("");
}

static VOID unc_gt_Read_PMU_Data(PVOID param)
{
	U32 j;
	U64 *buffer = read_counter_info;
	U32 dev_idx;
	U32 this_cpu;
	CPU_STATE pcpu;
	// U32 cur_grp;
	U32 offset_delta;
	U32 tmp_value_lo = 0;
	U32 tmp_value_hi = 0;
	GT_CTR_NODE gt_ctr_value;
	U32 package_num = 0;

	SEP_DRV_LOG_DEBUG_IN("Param: %p.", param);

	dev_idx = *((U32 *)param);
	this_cpu = CONTROL_THIS_CPU();
	pcpu = &pcb[this_cpu];

	if (!CPU_STATE_system_master(pcpu)) {
		SEP_DRV_LOG_DEBUG_OUT("Early exit (!system_master).");
		return;
	}

	package_num = core_to_package_map[this_cpu];
	// cur_grp = LWPMU_DEVICE_cur_group(&devices[(dev_idx)])[package_num];

	FOR_EACH_PCI_DATA_REG_RAW(pecb, i, dev_idx)
	{
		j = EMON_BUFFER_UNCORE_PACKAGE_EVENT_OFFSET(
			package_num,
			EMON_BUFFER_DRIVER_HELPER_num_entries_per_package(
				emon_buffer_driver_helper),
			ECB_entries_uncore_buffer_offset_in_package(pecb, i));
		offset_delta = ECB_entries_reg_offset(pecb, i);
		tmp_value_lo =
			PCI_MMIO_Read_U32(unc_gt_virtual_address, offset_delta);
		offset_delta = offset_delta + NEXT_ADDR_OFFSET;
		tmp_value_hi =
			PCI_MMIO_Read_U32(unc_gt_virtual_address, offset_delta);
		GT_CTR_NODE_low(gt_ctr_value) = tmp_value_lo;
		GT_CTR_NODE_high(gt_ctr_value) = tmp_value_hi;
		buffer[j] = GT_CTR_NODE_value(gt_ctr_value);
		SEP_DRV_LOG_TRACE("j=%u, value=%llu, cpu=%u", j, buffer[j],
				  this_cpu);
	}
	END_FOR_EACH_PCI_DATA_REG_RAW;

	SEP_DRV_LOG_DEBUG_OUT("");
}

/*
 * Initialize the dispatch table
 */

DISPATCH_NODE unc_gt_dispatch = { .init = NULL,
				  .fini = NULL,
				  .write = unc_gt_Write_PMU,
				  .freeze = unc_gt_Disable_PMU,
				  .restart = unc_gt_Enable_PMU,
				  .read_data = unc_gt_Read_PMU_Data,
				  .check_overflow = NULL,
				  .swap_group = NULL,
				  .read_lbrs = NULL,
				  .cleanup = NULL,
				  .hw_errata = NULL,
				  .read_power = NULL,
				  .check_overflow_errata = NULL,
				  .read_counts = unc_gt_Read_Counts,
				  .check_overflow_gp_errata = NULL,
				  .read_ro = NULL,
				  .platform_info = NULL,
				  .trigger_read = unc_gt_Read_Counts,
				  .scan_for_uncore = NULL,
				  .read_metrics = NULL };
