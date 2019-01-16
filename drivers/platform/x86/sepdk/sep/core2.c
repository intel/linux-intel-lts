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
#include "core2.h"
#include "ecb_iterators.h"
#include "pebs.h"
#include "apic.h"

#if !defined(DRV_ANDROID)
#include "jkt_unc_ha.h"
#include "jkt_unc_qpill.h"
#include "pci.h"
#endif

extern EVENT_CONFIG global_ec;
extern U64 *read_counter_info;
extern LBR lbr;
extern DRV_CONFIG drv_cfg;
extern DEV_CONFIG pcfg;
extern PWR pwr;
extern U64 *interrupt_counts;
extern DRV_SETUP_INFO_NODE req_drv_setup_info;
extern EMON_BUFFER_DRIVER_HELPER emon_buffer_driver_helper;

#if !defined(DRV_ANDROID)
static U32 direct2core_data_saved;
static U32 bl_bypass_data_saved;
#endif

static U32 restore_reg_addr[3];

typedef struct SADDR_S {
	S64 addr : CORE2_LBR_DATA_BITS;
} SADDR;

#define SADDR_addr(x) ((x).addr)
#define MSR_ENERGY_MULTIPLIER 0x606 // Energy Multiplier MSR

#if !defined(DRV_ANDROID)
/* ------------------------------------------------------------------------- */
/*!
 * @fn void core2_Disable_Direct2core(ECB)
 *
 * @param    pecb     ECB of group being scheduled
 *
 * @return   None     No return needed
 *
 * @brief    program the QPILL and HA register for disabling of direct2core
 *
 * <I>Special Notes</I>
 */
static VOID core2_Disable_Direct2core(ECB pecb)
{
	U32 busno = 0;
	U32 dev_idx = 0;
	U32 base_idx = 0;
	U32 device_id = 0;
	U32 value = 0;
	U32 vendor_id = 0;
	U32 core2_qpill_dev_no[2] = { 8, 9 };
	U32 this_cpu;

	SEP_DRV_LOG_TRACE_IN("PECB: %p.", pecb);

	this_cpu = CONTROL_THIS_CPU();

	// Discover the bus # for HA
	for (busno = 0; busno < MAX_BUSNO; busno++) {
		value = PCI_Read_U32(busno, JKTUNC_HA_DEVICE_NO,
				     JKTUNC_HA_D2C_FUNC_NO, 0);
		vendor_id = value & VENDOR_ID_MASK;
		device_id = (value & DEVICE_ID_MASK) >> DEVICE_ID_BITSHIFT;

		if (vendor_id != DRV_IS_PCI_VENDOR_ID_INTEL) {
			continue;
		}
		if (device_id != JKTUNC_HA_D2C_DID) {
			continue;
		}
		value = 0;
		// now program at the offset
		value = PCI_Read_U32(busno, JKTUNC_HA_DEVICE_NO,
				     JKTUNC_HA_D2C_FUNC_NO,
				     JKTUNC_HA_D2C_OFFSET);
		restore_ha_direct2core[this_cpu][busno] = 0;
		restore_ha_direct2core[this_cpu][busno] = value;
	}
	for (busno = 0; busno < MAX_BUSNO; busno++) {
		value = PCI_Read_U32(busno, JKTUNC_HA_DEVICE_NO,
				     JKTUNC_HA_D2C_FUNC_NO, 0);
		vendor_id = value & VENDOR_ID_MASK;
		device_id = (value & DEVICE_ID_MASK) >> DEVICE_ID_BITSHIFT;

		if (vendor_id != DRV_IS_PCI_VENDOR_ID_INTEL) {
			continue;
		}
		if (device_id != JKTUNC_HA_D2C_DID) {
			continue;
		}

		// now program at the offset
		value = PCI_Read_U32(busno, JKTUNC_HA_DEVICE_NO,
				     JKTUNC_HA_D2C_FUNC_NO,
				     JKTUNC_HA_D2C_OFFSET);
		value |= value | JKTUNC_HA_D2C_BITMASK;
		PCI_Write_U32(busno, JKTUNC_HA_DEVICE_NO, JKTUNC_HA_D2C_FUNC_NO,
			      JKTUNC_HA_D2C_OFFSET, value);
	}

	// Discover the bus # for QPI
	for (dev_idx = 0; dev_idx < 2; dev_idx++) {
		base_idx = dev_idx * MAX_BUSNO;
		for (busno = 0; busno < MAX_BUSNO; busno++) {
			value = PCI_Read_U32(busno, core2_qpill_dev_no[dev_idx],
					     JKTUNC_QPILL_D2C_FUNC_NO, 0);
			vendor_id = value & VENDOR_ID_MASK;
			device_id =
				(value & DEVICE_ID_MASK) >> DEVICE_ID_BITSHIFT;

			if (vendor_id != DRV_IS_PCI_VENDOR_ID_INTEL) {
				continue;
			}
			if ((device_id != JKTUNC_QPILL0_D2C_DID) &&
			    (device_id != JKTUNC_QPILL1_D2C_DID)) {
				continue;
			}
			// now program at the corresponding offset
			value = PCI_Read_U32(busno, core2_qpill_dev_no[dev_idx],
					     JKTUNC_QPILL_D2C_FUNC_NO,
					     JKTUNC_QPILL_D2C_OFFSET);
			restore_qpi_direct2core[this_cpu][base_idx + busno] = 0;
			restore_qpi_direct2core[this_cpu][base_idx + busno] =
				value;
		}
	}
	for (dev_idx = 0; dev_idx < 2; dev_idx++) {
		for (busno = 0; busno < MAX_BUSNO; busno++) {
			value = PCI_Read_U32(busno, core2_qpill_dev_no[dev_idx],
					     JKTUNC_QPILL_D2C_FUNC_NO, 0);
			vendor_id = value & VENDOR_ID_MASK;
			device_id =
				(value & DEVICE_ID_MASK) >> DEVICE_ID_BITSHIFT;

			if (vendor_id != DRV_IS_PCI_VENDOR_ID_INTEL) {
				continue;
			}
			if ((device_id != JKTUNC_QPILL0_D2C_DID) &&
			    (device_id != JKTUNC_QPILL1_D2C_DID)) {
				continue;
			}
			// now program at the corresponding offset
			value = PCI_Read_U32(busno, core2_qpill_dev_no[dev_idx],
					     JKTUNC_QPILL_D2C_FUNC_NO,
					     JKTUNC_QPILL_D2C_OFFSET);
			value |= value | JKTUNC_QPILL_D2C_BITMASK;
			PCI_Write_U32(busno, core2_qpill_dev_no[dev_idx],
				      JKTUNC_QPILL_D2C_FUNC_NO,
				      JKTUNC_QPILL_D2C_OFFSET, value);
		}
	}

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn void core2_Disable_BL_Bypass(ECB)
 *
 * @param    pecb     ECB of group being scheduled
 *
 * @return   None     No return needed
 *
 * @brief    Disable the BL Bypass
 *
 * <I>Special Notes</I>
 */
static VOID core2_Disable_BL_Bypass(ECB pecb)
{
	U64 value;
	U32 this_cpu;

	SEP_DRV_LOG_TRACE_IN("PECB: %p.", pecb);

	this_cpu = CONTROL_THIS_CPU();

	value = SYS_Read_MSR(CORE2UNC_DISABLE_BL_BYPASS_MSR);
	restore_bl_bypass[this_cpu] = 0;
	restore_bl_bypass[this_cpu] = value;
	value |= CORE2UNC_BLBYPASS_BITMASK;
	SYS_Write_MSR(CORE2UNC_DISABLE_BL_BYPASS_MSR, value);

	SEP_DRV_LOG_TRACE_OUT("");
}
#endif

/* ------------------------------------------------------------------------- */
/*!
 * @fn void core2_Write_PMU(param)
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
static VOID core2_Write_PMU(VOID *param)
{
	U32 this_cpu;
	CPU_STATE pcpu;
	ECB pecb;
	U32 dev_idx;
	U32 cur_grp;
	EVENT_CONFIG ec;
	DISPATCH dispatch;

	SEP_DRV_LOG_TRACE_IN("");

	this_cpu = CONTROL_THIS_CPU();
	pcpu = &pcb[this_cpu];
	dev_idx = core_to_dev_map[this_cpu];
	cur_grp = CPU_STATE_current_group(pcpu);
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[dev_idx])[cur_grp];
	ec = LWPMU_DEVICE_ec(&devices[dev_idx]);
	dispatch = LWPMU_DEVICE_dispatch(&devices[dev_idx]);

	if (!pecb) {
		SEP_DRV_LOG_TRACE_OUT(
			"No programming for this device in this group.");
		return;
	}

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
		SYS_Write_MSR(ECB_entries_reg_id(pecb, i),
			      ECB_entries_reg_value(pecb, i));
#if defined(MYDEBUG)
		{
			U64 val = SYS_Read_MSR(ECB_entries_reg_id(pecb, i));
			SEP_DRV_LOG_TRACE(
				"Register 0x%x: wrvalue 0x%llx, rdvalue 0x%llx.",
				ECB_entries_reg_id(pecb, i),
				ECB_entries_reg_value(pecb, i), val);
		}
#endif
	}
	END_FOR_EACH_REG_CORE_OPERATION;

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn void core2_Disable_PMU(param)
 *
 * @param    param    dummy parameter which is not used
 *
 * @return   None     No return needed
 *
 * @brief    Zero out the global control register.  This automatically disables the PMU counters.
 *
 */
static VOID core2_Disable_PMU(PVOID param)
{
	U32 this_cpu;
	CPU_STATE pcpu;
	ECB pecb;
	U32 dev_idx;
	U32 cur_grp;
	DEV_CONFIG pcfg;

	SEP_DRV_LOG_TRACE_IN("");

	this_cpu = CONTROL_THIS_CPU();
	pcpu = &pcb[this_cpu];
	dev_idx = core_to_dev_map[this_cpu];
	cur_grp = CPU_STATE_current_group(pcpu);
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[dev_idx])[cur_grp];
	pcfg = LWPMU_DEVICE_pcfg(&devices[dev_idx]);

	if (!pecb) {
		SEP_DRV_LOG_TRACE_OUT(
			"No programming for this device in this group.");
		return;
	}

	if (GET_DRIVER_STATE() != DRV_STATE_RUNNING) {
		SEP_DRV_LOG_TRACE("Driver state is not RUNNING.");
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
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn void core2_Enable_PMU(param)
 *
 * @param    param    dummy parameter which is not used
 *
 * @return   None     No return needed
 *
 * @brief    Set the enable bit for all the Control registers
 *
 */
static VOID core2_Enable_PMU(PVOID param)
{
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

	SEP_DRV_LOG_TRACE_IN("");

	this_cpu = CONTROL_THIS_CPU();
	pcpu = &pcb[this_cpu];
	dev_idx = core_to_dev_map[this_cpu];
	cur_grp = CPU_STATE_current_group(pcpu);
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[dev_idx])[cur_grp];
	pcfg = LWPMU_DEVICE_pcfg(&devices[dev_idx]);

	if (!pecb) {
		SEP_DRV_LOG_TRACE_OUT(
			"No programming for this device in this group.");
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
			if (DEV_CONFIG_pebs_mode(pcfg)) {
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
					ECB_entries_reg_id(
						pecb,
						ECB_SECTION_REG_INDEX(
							pecb,
							GLOBAL_CTRL_REG_INDEX,
							PMU_OPERATION_GLOBAL_REGS)),
					val);
			}
#endif
		}
	}
	SEP_DRV_LOG_TRACE("Reenabled PMU with value 0x%llx.",
			  ECB_entries_reg_value(pecb, 0));

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn void corei7_Enable_PMU_2(param)
 *
 * @param    param    dummy parameter which is not used
 *
 * @return   None     No return needed
 *
 * @brief    Set the enable bit for all the Control registers
 *
 */
static VOID corei7_Enable_PMU_2(PVOID param)
{
	/*
	 * Get the value from the event block
	 *   0 == location of the global control reg for this block.
	 */
	U32 this_cpu;
	CPU_STATE pcpu;
	ECB pecb;
	U64 pebs_val = 0;
	U32 dev_idx;
	U32 cur_grp;
	DEV_CONFIG pcfg;

	SEP_DRV_LOG_TRACE_IN("");

	this_cpu = CONTROL_THIS_CPU();
	pcpu = &pcb[this_cpu];
	dev_idx = core_to_dev_map[this_cpu];
	cur_grp = CPU_STATE_current_group(pcpu);
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[dev_idx])[cur_grp];
	pcfg = LWPMU_DEVICE_pcfg(&devices[dev_idx]);

	if (!pecb) {
		SEP_DRV_LOG_TRACE_OUT(
			"No programming for this device in this group.");
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
		if (CPU_STATE_group_swap(pcpu)) {
			CPU_STATE_group_swap(pcpu) = 0;
			if (DEV_CONFIG_pebs_mode(pcfg)) {
				pebs_val = SYS_Read_MSR(ECB_entries_reg_id(
					pecb,
					ECB_SECTION_REG_INDEX(
						pecb, PEBS_ENABLE_REG_INDEX,
						PMU_OPERATION_GLOBAL_REGS)));
				if (ECB_entries_reg_value(
					    pecb,
					    ECB_SECTION_REG_INDEX(
						    pecb, PEBS_ENABLE_REG_INDEX,
						    PMU_OPERATION_GLOBAL_REGS)) !=
				    0) {
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
				} else if (pebs_val != 0) {
					SYS_Write_MSR(
						ECB_entries_reg_id(
							pecb,
							ECB_SECTION_REG_INDEX(
								pecb,
								PEBS_ENABLE_REG_INDEX,
								PMU_OPERATION_GLOBAL_REGS)),
						0LL);
				}
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
#if defined(MYDEBUG)
			SEP_DRV_LOG_TRACE("Reenabled PMU with value 0x%llx.",
					  ECB_entries_reg_value(pecb, 0));
#endif
		}
		if (CPU_STATE_reset_mask(pcpu)) {
#if defined(MYDEBUG)
			SEP_DRV_LOG_TRACE("Overflow reset mask %llx.",
					  CPU_STATE_reset_mask(pcpu));
#endif
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
	}

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn core2_Read_PMU_Data(param)
 *
 * @param    param    dummy parameter which is not used
 *
 * @return   None     No return needed
 *
 * @brief    Read all the data MSR's into a buffer.  Called by the interrupt handler.
 *
 */
static void core2_Read_PMU_Data(PVOID param)
{
	U32 j;
	U64 *buffer = read_counter_info;
	U32 this_cpu;
	CPU_STATE pcpu;
	ECB pecb;
	U32 dev_idx;
	U32 cur_grp;

	SEP_DRV_LOG_TRACE_IN("");

	preempt_disable();
	this_cpu = CONTROL_THIS_CPU();
	preempt_enable();
	pcpu = &pcb[this_cpu];
	dev_idx = core_to_dev_map[this_cpu];
	cur_grp = CPU_STATE_current_group(pcpu);
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[dev_idx])[cur_grp];

	if (!pecb) {
		SEP_DRV_LOG_TRACE_OUT(
			"No programming for this device in this group.");
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

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn core2_Check_Overflow_Errata(pecb, index, overflow_status)
 *
 * @param  pecb:            The current event control block
 * @param  index:           index of the register to process
 * @param  overflow_status: current overflow mask
 *
 * @return Updated Event mask of the overflowed registers.
 *
 * @brief  Go through the overflow errata for the architecture and set the mask
 *
 * <I>Special Notes</I>
 *         fixed_counter1 on some architectures gets interfered by
 *         other event counts.  Overcome this problem by reading the
 *         counter value and resetting the overflow mask.
 *
 */
static U64 core2_Check_Overflow_Errata(ECB pecb, U32 index, U64 overflow_status)
{
	SEP_DRV_LOG_TRACE_IN("");

	if (DRV_CONFIG_num_events(drv_cfg) == 1) {
		SEP_DRV_LOG_TRACE_OUT("Res: %llu. (num_events = 1)",
				      overflow_status);
		return overflow_status;
	}
	if (ECB_entries_reg_id(pecb, index) == IA32_FIXED_CTR1 &&
	    (overflow_status & 0x200000000LL) == 0LL) {
		U64 val = SYS_Read_MSR(IA32_FIXED_CTR1);
		val &= ECB_entries_max_bits(pecb, index);
		if (val < ECB_entries_reg_value(pecb, index)) {
			overflow_status |= 0x200000000LL;
			SEP_DRV_LOG_TRACE(
				"Reset -- clk count %llx, status %llx.", val,
				overflow_status);
		}
	}

	SEP_DRV_LOG_TRACE_OUT("Res: %llu.", overflow_status);
	return overflow_status;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn void core2_Check_Overflow(masks)
 *
 * @param    masks    the mask structure to populate
 *
 * @return   None     No return needed
 *
 * @brief  Called by the data processing method to figure out which registers have overflowed.
 *
 */
static void core2_Check_Overflow(DRV_MASKS masks)
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

	SEP_DRV_LOG_TRACE_IN("");

	this_cpu = CONTROL_THIS_CPU();
	bd = &cpu_buf[this_cpu];
	pcpu = &pcb[this_cpu];
	dev_idx = core_to_dev_map[this_cpu];
	cur_grp = CPU_STATE_current_group(pcpu);
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[dev_idx])[cur_grp];
	pcfg = LWPMU_DEVICE_pcfg(&devices[dev_idx]);
	dispatch = LWPMU_DEVICE_dispatch(&devices[dev_idx]);

	if (!pecb) {
		SEP_DRV_LOG_TRACE_OUT(
			"No programming for this device in this group.");
		return;
	}

	// initialize masks
	DRV_MASKS_masks_num(masks) = 0;

	overflow_status = SYS_Read_MSR(ECB_entries_reg_id(
		pecb, ECB_SECTION_REG_INDEX(pecb, GLOBAL_STATUS_REG_INDEX,
					    PMU_OPERATION_GLOBAL_STATUS)));

	if (DEV_CONFIG_pebs_mode(pcfg)) {
		overflow_status = PEBS_Overflowed(this_cpu, overflow_status, 0);
	}
	overflow_status_clr = overflow_status;

	if (dispatch->check_overflow_gp_errata) {
		overflow_status = dispatch->check_overflow_gp_errata(
			pecb, &overflow_status_clr);
	}
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
			SEP_DRV_LOG_TRACE("Overflow:  cpu: %d, index %d.",
					  this_cpu, index);
			SEP_DRV_LOG_TRACE(
				"register 0x%x --- val 0%llx.",
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
			if (ECB_entries_fixed_reg_get(pecb, i)) {
				CPU_STATE_p_state_counting(pcpu) = 1;
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

			SEP_DRV_LOG_TRACE("overflow -- 0x%llx, index 0x%llx.",
					  overflow_status, (U64)1 << index);
			SEP_DRV_LOG_TRACE("slot# %d, reg_id 0x%x, index %d.", i,
					  ECB_entries_reg_id(pecb, i), index);
			if (ECB_entries_event_id_index(pecb, i) ==
			    CPU_STATE_trigger_event_num(pcpu)) {
				CPU_STATE_trigger_count(pcpu)--;
			}
		}
	}
	END_FOR_EACH_REG_CORE_OPERATION;

	CPU_STATE_reset_mask(pcpu) = overflow_status_clr;
	// Reinitialize the global overflow control register
	SYS_Write_MSR(ECB_entries_reg_id(
			      pecb, ECB_SECTION_REG_INDEX(
					    pecb, GLOBAL_OVF_CTRL_REG_INDEX,
					    PMU_OPERATION_GLOBAL_REGS)),
		      overflow_status_clr);

	SEP_DRV_LOG_TRACE("Check Overflow completed %d.", this_cpu);
	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn core2_Swap_Group(restart)
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
static VOID core2_Swap_Group(DRV_BOOL restart)
{
	U32 index;
	U32 next_group;
	U32 st_index;
	U32 this_cpu;
	CPU_STATE pcpu;
	U32 dev_idx;
	DISPATCH dispatch;
	EVENT_CONFIG ec;

	SEP_DRV_LOG_TRACE_IN("");

	this_cpu = CONTROL_THIS_CPU();
	pcpu = &pcb[this_cpu];
	dev_idx = core_to_dev_map[this_cpu];
	dispatch = LWPMU_DEVICE_dispatch(&devices[dev_idx]);
	ec = LWPMU_DEVICE_ec(&devices[dev_idx]);

	st_index =
		CPU_STATE_current_group(pcpu) * EVENT_CONFIG_max_gp_events(ec);
	next_group = (CPU_STATE_current_group(pcpu) + 1);
	if (next_group >= EVENT_CONFIG_num_groups(ec)) {
		next_group = 0;
	}

	SEP_DRV_LOG_TRACE("current group : 0x%x.",
			  CPU_STATE_current_group(pcpu));
	SEP_DRV_LOG_TRACE("next group : 0x%x.", next_group);

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
 * @fn core2_Initialize(params)
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
static VOID core2_Initialize(VOID *param)
{
	U32 this_cpu;
	CPU_STATE pcpu;
	U32 dev_idx;
	DEV_CONFIG pcfg;
	U32 i = 0;
	ECB pecb = NULL;
	U32 cur_grp;

	SEP_DRV_LOG_TRACE_IN("");

	this_cpu = CONTROL_THIS_CPU();
	dev_idx = core_to_dev_map[this_cpu];
	pcfg = LWPMU_DEVICE_pcfg(&devices[dev_idx]);

	if (pcb == NULL) {
		SEP_DRV_LOG_TRACE_OUT(
			"No programming for this device in this group.");
		return;
	}

	pcpu = &pcb[this_cpu];
	cur_grp = CPU_STATE_current_group(pcpu);
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[dev_idx])[cur_grp];
	CPU_STATE_pmu_state(pcpu) = pmu_state + (this_cpu * 3);
	if (CPU_STATE_pmu_state(pcpu) == NULL) {
		SEP_DRV_LOG_WARNING_TRACE_OUT(
			"Unable to save PMU state on CPU %d.", this_cpu);
		return;
	}

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

#if !defined(DRV_ANDROID)
	if (!CPU_STATE_socket_master(pcpu)) {
		SEP_DRV_LOG_TRACE_OUT("Not socket master.");
		return;
	}

	direct2core_data_saved = 0;
	bl_bypass_data_saved = 0;
	cur_grp = CPU_STATE_current_group(pcpu);

	if (restore_ha_direct2core && restore_qpi_direct2core) {
		for (i = 0; i < GLOBAL_STATE_num_em_groups(driver_state); i++) {
			pecb = LWPMU_DEVICE_PMU_register_data(
				&devices[dev_idx])[i];
			if (pecb && (ECB_flags(pecb) & ECB_direct2core_bit)) {
				core2_Disable_Direct2core(
					LWPMU_DEVICE_PMU_register_data(
						&devices[dev_idx])[cur_grp]);
				direct2core_data_saved = 1;
				break;
			}
		}
	}
	if (restore_bl_bypass) {
		for (i = 0; i < GLOBAL_STATE_num_em_groups(driver_state); i++) {
			pecb = LWPMU_DEVICE_PMU_register_data(
				&devices[dev_idx])[i];
			if (pecb && (ECB_flags(pecb) & ECB_bl_bypass_bit)) {
				core2_Disable_BL_Bypass(
					LWPMU_DEVICE_PMU_register_data(
						&devices[dev_idx])[cur_grp]);
				bl_bypass_data_saved = 1;
				break;
			}
		}
	}
#endif

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn core2_Destroy(params)
 *
 * @param    params    dummy parameter which is not used
 *
 * @return   None     No return needed
 *
 * @brief    Reset the PMU setting up after collection
 *
 * <I>Special Notes</I>
 *         Restores the previously saved PMU state done in core2_Initialize.
 *         This function should be called in parallel across all CPUs
 *         after sampling collection ends/terminates.
 *
 */
static VOID core2_Destroy(VOID *param)
{
	U32 this_cpu;
	CPU_STATE pcpu;

	SEP_DRV_LOG_TRACE_IN("");

	if (pcb == NULL) {
		SEP_DRV_LOG_TRACE_OUT(
			"No programming for this device in this group.");
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

	// Tentative code below (trying to avoid race conditions with the NMI watchdog). Should be evaluated in the coming few days. (2018/05/21)
	SYS_Write_MSR(restore_reg_addr[0], 0);
	SYS_Write_MSR(restore_reg_addr[1], 0);
	SYS_Write_MSR(restore_reg_addr[2], 0);

	CPU_STATE_pmu_state(pcpu) = NULL;

	SEP_DRV_LOG_TRACE_OUT("");
}

/*
 * @fn core2_Read_LBRs(buffer)
 *
 * @param   IN buffer - pointer to the buffer to write the data into
 * @return  Last branch source IP address
 *
 * @brief   Read all the LBR registers into the buffer provided and return
 *
 */
static U64 core2_Read_LBRs(VOID *buffer, PVOID data)
{
	U32 i, count = 0;
	U64 *lbr_buf = NULL;
	U64 value = 0;
	U64 tos_ip_addr = 0;
	U64 tos_ptr = 0;
	SADDR saddr;
	U32 this_cpu;
	U32 dev_idx;
	LBR lbr;
	DEV_CONFIG pcfg;

	SEP_DRV_LOG_TRACE_IN("");

	this_cpu = CONTROL_THIS_CPU();
	dev_idx = core_to_dev_map[this_cpu];
	pcfg = LWPMU_DEVICE_pcfg(&devices[dev_idx]);
	lbr = LWPMU_DEVICE_lbr(&devices[dev_idx]);

	if (buffer && DEV_CONFIG_store_lbrs(pcfg)) {
		lbr_buf = (U64 *)buffer;
	}

	for (i = 0; i < LBR_num_entries(lbr); i++) {
		value = SYS_Read_MSR(LBR_entries_reg_id(lbr, i));
		if (buffer && DEV_CONFIG_store_lbrs(pcfg)) {
			*lbr_buf = value;
		}
		SEP_DRV_LOG_TRACE("core2_Read_LBRs %u, 0x%llx.", i, value);
		if (i == 0) {
			tos_ptr = value;
		} else {
			if (LBR_entries_etype(lbr, i) ==
			    LBR_ENTRY_FROM_IP) { // LBR from register
				if (tos_ptr == count) {
					SADDR_addr(saddr) =
						value & CORE2_LBR_BITMASK;
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
 * @fn corei7_Read_LBRs(buffer)
 *
 * @param   IN buffer - pointer to the buffer to write the data into
 * @return  Last branch source IP address
 *
 * @brief   Read all the LBR registers into the buffer provided and return
 *
 */
static U64 corei7_Read_LBRs(VOID *buffer, PVOID data)
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

	SEP_DRV_LOG_TRACE_IN("");

	this_cpu = CONTROL_THIS_CPU();
	dev_idx = core_to_dev_map[this_cpu];
	pcfg = LWPMU_DEVICE_pcfg(&devices[dev_idx]);
	lbr = LWPMU_DEVICE_lbr(&devices[dev_idx]);

	if (buffer && DEV_CONFIG_store_lbrs(pcfg)) {
		lbr_buf = (U64 *)buffer;
	}

	if (LBR_num_entries(lbr) > 0) {
		pairs = (LBR_num_entries(lbr) - 1) / 2;
	}
	for (i = 0; i < LBR_num_entries(lbr); i++) {
		value = SYS_Read_MSR(LBR_entries_reg_id(lbr, i));
		if (buffer && DEV_CONFIG_store_lbrs(pcfg)) {
			*lbr_buf = value;
		}
		if (DEV_CONFIG_collect_callstacks(pcfg)) {
			if ((LBR_entries_etype(lbr, i) == LBR_ENTRY_FROM_IP &&
			     i > tos_ptr + 1) ||
			    (LBR_entries_etype(lbr, i) == LBR_ENTRY_TO_IP &&
			     i > tos_ptr + pairs + 1)) {
				if (buffer && DEV_CONFIG_store_lbrs(pcfg)) {
					*lbr_buf = 0x0ULL;
					lbr_buf++;
				}
				continue;
			}
		}
#if defined(DRV_SEP_ACRN_ON)
		if (DEV_CONFIG_collect_callstacks(pcfg)) {
			if ((LBR_entries_etype(lbr, i) == LBR_ENTRY_FROM_IP &&
			     i > tos_ptr + 1) ||
			    (LBR_entries_etype(lbr, i) == LBR_ENTRY_TO_IP &&
			     i > tos_ptr + pairs + 1)) {
				if (buffer && DEV_CONFIG_store_lbrs(pcfg)) {
					*lbr_buf = 0x0ULL;
					lbr_buf++;
				}
				continue;
			}
		}
#endif
		SEP_DRV_LOG_TRACE("I: %u, value: 0x%llx.", i, value);
		if (i == 0) {
			tos_ptr = value;
		} else {
			if (LBR_entries_etype(lbr, i) ==
			    LBR_ENTRY_FROM_IP) { // LBR from register
				if (tos_ptr == count) {
					SADDR_addr(saddr) =
						value & CORE2_LBR_BITMASK;
					tos_ip_addr = (U64)SADDR_addr(
						saddr); // Add signed extension
					SEP_DRV_LOG_TRACE(
						"tos_ip_addr %llu, 0x%llx.",
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

static VOID core2_Clean_Up(VOID *param)
{
#if !defined(DRV_ANDROID)
	U32 this_cpu;
	CPU_STATE pcpu;
	U32 busno = 0;
	U32 dev_idx = 0;
	U32 base_idx = 0;
	U32 device_id = 0;
	U32 value = 0;
	U32 vendor_id = 0;
	U32 core2_qpill_dev_no[2] = { 8, 9 };
#endif

	SEP_DRV_LOG_TRACE_IN("");

#if !defined(DRV_ANDROID)
	this_cpu = CONTROL_THIS_CPU();
	pcpu = &pcb[this_cpu];
#endif

	FOR_EACH_REG_CORE_OPERATION(pecb, i, PMU_OPERATION_ALL_REG)
	{
		if (ECB_entries_clean_up_get(pecb, i)) {
			SEP_DRV_LOG_TRACE("clean up set --- RegId --- %x.",
					  ECB_entries_reg_id(pecb, i));
			SYS_Write_MSR(ECB_entries_reg_id(pecb, i), 0LL);
		}
	}
	END_FOR_EACH_REG_CORE_OPERATION;

#if !defined(DRV_ANDROID)
	if (!CPU_STATE_socket_master(pcpu)) {
		SEP_DRV_LOG_TRACE_OUT("Not socket master.");
		return;
	}

	if (restore_ha_direct2core && restore_qpi_direct2core &&
	    direct2core_data_saved) {
		// Discover the bus # for HA
		for (busno = 0; busno < MAX_BUSNO; busno++) {
			value = PCI_Read_U32(busno, JKTUNC_HA_DEVICE_NO,
					     JKTUNC_HA_D2C_FUNC_NO, 0);
			vendor_id = value & VENDOR_ID_MASK;
			device_id =
				(value & DEVICE_ID_MASK) >> DEVICE_ID_BITSHIFT;

			if (vendor_id != DRV_IS_PCI_VENDOR_ID_INTEL) {
				continue;
			}
			if (device_id != JKTUNC_HA_D2C_DID) {
				continue;
			}

			// now program at the offset
			PCI_Write_U32(busno, JKTUNC_HA_DEVICE_NO,
				      JKTUNC_HA_D2C_FUNC_NO,
				      JKTUNC_HA_D2C_OFFSET,
				      restore_ha_direct2core[this_cpu][busno]);
		}

		// Discover the bus # for QPI
		for (dev_idx = 0; dev_idx < 2; dev_idx++) {
			base_idx = dev_idx * MAX_BUSNO;
			for (busno = 0; busno < MAX_BUSNO; busno++) {
				value = PCI_Read_U32(
					busno, core2_qpill_dev_no[dev_idx],
					JKTUNC_QPILL_D2C_FUNC_NO, 0);
				vendor_id = value & VENDOR_ID_MASK;
				device_id = (value & DEVICE_ID_MASK) >>
					    DEVICE_ID_BITSHIFT;

				if (vendor_id != DRV_IS_PCI_VENDOR_ID_INTEL) {
					continue;
				}
				if ((device_id != JKTUNC_QPILL0_D2C_DID) &&
				    (device_id != JKTUNC_QPILL1_D2C_DID)) {
					continue;
				}
				// now program at the corresponding offset
				PCI_Write_U32(busno,
					      core2_qpill_dev_no[dev_idx],
					      JKTUNC_QPILL_D2C_FUNC_NO,
					      JKTUNC_QPILL_D2C_OFFSET,
					      restore_qpi_direct2core[this_cpu]
								     [base_idx +
								      busno]);
			}
		}
	}
	if (restore_bl_bypass && bl_bypass_data_saved) {
		SYS_Write_MSR(CORE2UNC_DISABLE_BL_BYPASS_MSR,
			      restore_bl_bypass[this_cpu]);
	}
#endif

	SEP_DRV_LOG_TRACE_OUT("");
}

static VOID corei7_Errata_Fix(void)
{
	U32 this_cpu = CONTROL_THIS_CPU();
	CPU_STATE pcpu = &pcb[this_cpu];
	ECB(pecb) = NULL;
	U32 dev_idx, cur_grp;
	DEV_CONFIG pcfg;

	SEP_DRV_LOG_TRACE_IN("");

	this_cpu = CONTROL_THIS_CPU();
	dev_idx = core_to_dev_map[this_cpu];
	pcfg = LWPMU_DEVICE_pcfg(&devices[dev_idx]);
	cur_grp = CPU_STATE_current_group(pcpu);
	pcfg = LWPMU_DEVICE_pcfg(&devices[dev_idx]);
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[dev_idx])[cur_grp];

	if (DEV_CONFIG_pebs_mode(pcfg)) {
		SYS_Write_MSR(ECB_entries_reg_id(
				      pecb, ECB_SECTION_REG_INDEX(
						    pecb, PEBS_ENABLE_REG_INDEX,
						    PMU_OPERATION_GLOBAL_REGS)),
			      0LL);
	}

	FOR_EACH_REG_CORE_OPERATION(pecb, i, PMU_OPERATION_HW_ERRATA)
	{
		SYS_Write_MSR(ECB_entries_reg_id(pecb, i),
			      ECB_entries_reg_value(pecb, i));
	}
	END_FOR_EACH_REG_CORE_OPERATION;

	SEP_DRV_LOG_TRACE_OUT("");
}

static VOID corei7_Errata_Fix_2(void)
{
	SEP_DRV_LOG_TRACE_IN("");

	FOR_EACH_REG_CORE_OPERATION(pecb, i, PMU_OPERATION_HW_ERRATA)
	{
		SYS_Write_MSR(ECB_entries_reg_id(pecb, i),
			      ECB_entries_reg_value(pecb, i));
	}
	END_FOR_EACH_REG_CORE_OPERATION;

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn void core2_Check_Overflow_Htoff_Mode(masks)
 *
 * @param    masks    the mask structure to populate
 *
 * @return   None     No return needed
 *
 * @brief  Called by the data processing method to figure out which registers have overflowed.
 *
 */
static void core2_Check_Overflow_Htoff_Mode(DRV_MASKS masks)
{
	U32 index;
	U64 value = 0;
	U64 overflow_status = 0;
	U32 this_cpu;
	BUFFER_DESC bd;
	CPU_STATE pcpu;
	U32 dev_idx;
	U32 cur_grp;
	DISPATCH dispatch;
	DEV_CONFIG pcfg;
	ECB pecb;
	U64 overflow_status_clr = 0;
	DRV_EVENT_MASK_NODE event_flag;

	SEP_DRV_LOG_TRACE_IN("");

	this_cpu = CONTROL_THIS_CPU();
	bd = &cpu_buf[this_cpu];
	pcpu = &pcb[this_cpu];
	dev_idx = core_to_dev_map[this_cpu];
	cur_grp = CPU_STATE_current_group(pcpu);
	dispatch = LWPMU_DEVICE_dispatch(&devices[dev_idx]);
	pcfg = LWPMU_DEVICE_pcfg(&devices[dev_idx]);
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[dev_idx])[cur_grp];

	SEP_DRV_LOG_TRACE("");

	if (!pecb) {
		SEP_DRV_LOG_TRACE_OUT(
			"No programming for this device in this group.");
		return;
	}

	// initialize masks
	DRV_MASKS_masks_num(masks) = 0;

	overflow_status = SYS_Read_MSR(ECB_entries_reg_id(
		pecb, ECB_SECTION_REG_INDEX(pecb, GLOBAL_STATUS_REG_INDEX,
					    PMU_OPERATION_GLOBAL_STATUS)));

	if (DEV_CONFIG_pebs_mode(pcfg)) {
		overflow_status = PEBS_Overflowed(this_cpu, overflow_status, 0);
	}
	overflow_status_clr = overflow_status;
	SEP_DRV_LOG_TRACE("Overflow:  cpu: %d, status 0x%llx.", this_cpu,
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
			if (i >= (ECB_operations_register_start(
					  pecb, PMU_OPERATION_DATA_GP) +
				  4) &&
			    i <= (ECB_operations_register_start(
					  pecb, PMU_OPERATION_DATA_GP) +
				  7)) {
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
				"register 0x%x --- val 0%llx.",
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
			if (ECB_entries_fixed_reg_get(pecb, i)) {
				CPU_STATE_p_state_counting(pcpu) = 1;
			}
			if (ECB_entries_precise_get(pecb, i)) {
				DRV_EVENT_MASK_precise(&event_flag) = 1;
			}
			if (ECB_entries_lbr_value_get(pecb, i)) {
				DRV_EVENT_MASK_lbr_capture(&event_flag) = 1;
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

			SEP_DRV_LOG_TRACE("overflow -- 0x%llx, index 0x%llx.",
					  overflow_status, (U64)1 << index);
			SEP_DRV_LOG_TRACE("slot# %d, reg_id 0x%x, index %d.", i,
					  ECB_entries_reg_id(pecb, i), index);
			if (ECB_entries_event_id_index(pecb, i) ==
			    CPU_STATE_trigger_event_num(pcpu)) {
				CPU_STATE_trigger_count(pcpu)--;
			}
		}
	}
	END_FOR_EACH_REG_CORE_OPERATION;

	CPU_STATE_reset_mask(pcpu) = overflow_status_clr;
	// Reinitialize the global overflow control register
	SYS_Write_MSR(ECB_entries_reg_id(
			      pecb, ECB_SECTION_REG_INDEX(
					    pecb, GLOBAL_OVF_CTRL_REG_INDEX,
					    PMU_OPERATION_GLOBAL_REGS)),
		      overflow_status_clr);

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn void core2_Read_Power(buffer)
 *
 * @param    buffer   - pointer to the buffer to write the data into
 *
 * @return   None     No return needed
 *
 * @brief  Read all the power MSRs into the buffer provided and return.
 *
 */
static VOID corei7_Read_Power(VOID *buffer)
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
 * @fn core2_Read_Counts(param, id)
 *
 * @param    param      The read thread node to process
 * @param    id         The event id for the which the sample is generated
 *
 * @return   None     No return needed
 *
 * @brief    Read CPU event based counts for the events with reg value=0 and store into the buffer param;
 *
 */
static VOID core2_Read_Counts(PVOID param, U32 id)
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

	if (DRV_CONFIG_enable_p_state(drv_cfg)) {
		CPU_STATE_p_state_counting(pcpu) = 0;
	}

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn corei7_Check_Overflow_Errata(pecb)
 *
 * @param pecb:            The current event control block
 * @param overflow_status: current overflow mask
 *
 * @return   Updated Event mask of the overflowed registers.
 *
 * @brief    There is a bug where highly correlated precise events do
 *           not raise an indication on overflows in Core i7 and SNB.
 */
static U64 corei7_Check_Overflow_Errata(ECB pecb__, U64 *overflow_status_clr)
{
	U64 index = 0, value = 0, overflow_status = 0;

	SEP_DRV_LOG_TRACE_IN("PECB: %p, overflow_status_clr: %p.", pecb__,
			     overflow_status_clr);

	overflow_status = *overflow_status_clr;

	if (DRV_CONFIG_num_events(drv_cfg) == 1) {
		SEP_DRV_LOG_TRACE_OUT("Res = %llu (num_events = 1).",
				      overflow_status);
		return overflow_status;
	}

	FOR_EACH_REG_CORE_OPERATION(pecb, i, PMU_OPERATION_DATA_ALL)
	{
		if (ECB_entries_reg_value(pecb, i) == 0) {
			continue;
		}
		if (ECB_entries_is_gp_reg_get(pecb, i)) {
			index = i - ECB_operations_register_start(
					    pecb, PMU_OPERATION_DATA_GP);
			value = SYS_Read_MSR(ECB_entries_reg_id(pecb, i));
			if (value > 0LL && value <= 0x100000000LL) {
				overflow_status |= ((U64)1 << index);
				*overflow_status_clr |= ((U64)1 << index);
				SEP_DRV_LOG_TRACE("Counter 0x%x value 0x%llx.",
						  ECB_entries_reg_id(pecb, i),
						  value);
			}
			continue;
		}
		if (ECB_entries_fixed_reg_get(pecb, i)) {
			index = i -
				ECB_operations_register_start(
					pecb, PMU_OPERATION_DATA_FIXED) +
				0x20;
			if (!(overflow_status & ((U64)1 << index))) {
				value = SYS_Read_MSR(
					ECB_entries_reg_id(pecb, i));
				if (ECB_entries_reg_id(pecb, i) ==
				    ECB_entries_reg_id(
					    pecb,
					    ECB_SECTION_REG_INDEX(
						    pecb, 0,
						    PMU_OPERATION_CHECK_OVERFLOW_GP_ERRATA))) {
					if (!(value > 0LL &&
					      value <= 0x1000000LL) &&
					    (*overflow_status_clr &
					     ((U64)1 << index))) {
						//Clear it only for overflow_status so that we do not create sample records
						//Please do not remove the check for MSR index
						overflow_status =
							overflow_status &
							~((U64)1 << index);
						continue;
					}
				}
				if (value > 0LL && value <= 0x100000000LL) {
					overflow_status |= ((U64)1 << index);
					*overflow_status_clr |=
						((U64)1 << index);
					SEP_DRV_LOG_TRACE(
						"counter 0x%x value 0x%llx\n",
						ECB_entries_reg_id(pecb, i),
						value);
				}
			}
		}
	}
	END_FOR_EACH_REG_CORE_OPERATION;

	SEP_DRV_LOG_TRACE_OUT("Res = %llu.", overflow_status);
	return overflow_status;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          U64 corei7_Read_Platform_Info
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
static VOID corei7_Platform_Info(PVOID data)
{
	DRV_PLATFORM_INFO platform_data = (DRV_PLATFORM_INFO)data;
	U64 value = 0;

	SEP_DRV_LOG_TRACE_IN("Data: %p.", data);

	if (!platform_data) {
		SEP_DRV_LOG_TRACE_OUT("Platform_data is NULL!");
		return;
	}

	DRV_PLATFORM_INFO_energy_multiplier(platform_data) = 0;

#define IA32_MSR_PLATFORM_INFO 0xCE
	value = SYS_Read_MSR(IA32_MSR_PLATFORM_INFO);

	DRV_PLATFORM_INFO_info(platform_data) = value;
	DRV_PLATFORM_INFO_ddr_freq_index(platform_data) = 0;
#undef IA32_MSR_PLATFORM_INFO
#define IA32_MSR_MISC_ENABLE 0x1A4
	DRV_PLATFORM_INFO_misc_valid(platform_data) = 1;
	value = SYS_Read_MSR(IA32_MSR_MISC_ENABLE);
	DRV_PLATFORM_INFO_misc_info(platform_data) = value;
#undef IA32_MSR_MISC_ENABLE
	SEP_DRV_LOG_TRACE("Read from MSR_ENERGY_MULTIPLIER reg is %llu.",
			  SYS_Read_MSR(MSR_ENERGY_MULTIPLIER));
	DRV_PLATFORM_INFO_energy_multiplier(platform_data) =
		(U32)(SYS_Read_MSR(MSR_ENERGY_MULTIPLIER) & 0x00001F00) >> 8;

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          U64 corei7_Platform_Info_Nehalem
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
static VOID corei7_Platform_Info_Nehalem(PVOID data)
{
	DRV_PLATFORM_INFO platform_data = (DRV_PLATFORM_INFO)data;
	U64 value = 0;

	SEP_DRV_LOG_TRACE_IN("Data: %p.", data);

	if (!platform_data) {
		SEP_DRV_LOG_TRACE_OUT("Platform_data is NULL!");
		return;
	}

#define IA32_MSR_PLATFORM_INFO 0xCE
	value = SYS_Read_MSR(IA32_MSR_PLATFORM_INFO);

	DRV_PLATFORM_INFO_info(platform_data) = value;
	DRV_PLATFORM_INFO_ddr_freq_index(platform_data) = 0;
#undef IA32_MSR_PLATFORM_INFO
#define IA32_MSR_MISC_ENABLE 0x1A4
	DRV_PLATFORM_INFO_misc_valid(platform_data) = 1;
	value = SYS_Read_MSR(IA32_MSR_MISC_ENABLE);
	DRV_PLATFORM_INFO_misc_info(platform_data) = value;
#undef IA32_MSR_MISC_ENABLE
	DRV_PLATFORM_INFO_energy_multiplier(platform_data) = 0;

	SEP_DRV_LOG_TRACE_OUT("");
}

/*
 * Initialize the dispatch table
 */
DISPATCH_NODE core2_dispatch = { .init = core2_Initialize,
				 .fini = core2_Destroy,
				 .write = core2_Write_PMU,
				 .freeze = core2_Disable_PMU,
				 .restart = core2_Enable_PMU,
				 .read_data = core2_Read_PMU_Data,
				 .check_overflow = core2_Check_Overflow,
				 .swap_group = core2_Swap_Group,
				 .read_lbrs = core2_Read_LBRs,
				 .cleanup = core2_Clean_Up,
				 .hw_errata = NULL,
				 .read_power = NULL,
				 .check_overflow_errata =
					 core2_Check_Overflow_Errata,
				 .read_counts = core2_Read_Counts,
				 .check_overflow_gp_errata = NULL,
				 .read_ro = NULL,
				 .platform_info = NULL,
				 .trigger_read = NULL,
				 .scan_for_uncore = NULL,
				 .read_metrics = NULL };

DISPATCH_NODE corei7_dispatch = { .init = core2_Initialize,
				  .fini = core2_Destroy,
				  .write = core2_Write_PMU,
				  .freeze = core2_Disable_PMU,
				  .restart = core2_Enable_PMU,
				  .read_data = core2_Read_PMU_Data,
				  .check_overflow = core2_Check_Overflow,
				  .swap_group = core2_Swap_Group,
				  .read_lbrs = corei7_Read_LBRs,
				  .cleanup = core2_Clean_Up,
				  .hw_errata = corei7_Errata_Fix,
				  .read_power = corei7_Read_Power,
				  .check_overflow_errata = NULL,
				  .read_counts = core2_Read_Counts,
				  .check_overflow_gp_errata =
					  corei7_Check_Overflow_Errata,
				  .read_ro = NULL,
				  .platform_info = corei7_Platform_Info,
				  .trigger_read = NULL,
				  .scan_for_uncore = NULL,
				  .read_metrics = NULL };

DISPATCH_NODE corei7_dispatch_2 = { .init = core2_Initialize,
				    .fini = core2_Destroy,
				    .write = core2_Write_PMU,
				    .freeze = core2_Disable_PMU,
				    .restart = corei7_Enable_PMU_2,
				    .read_data = core2_Read_PMU_Data,
				    .check_overflow = core2_Check_Overflow,
				    .swap_group = core2_Swap_Group,
				    .read_lbrs = corei7_Read_LBRs,
				    .cleanup = core2_Clean_Up,
				    .hw_errata = corei7_Errata_Fix_2,
				    .read_power = corei7_Read_Power,
				    .check_overflow_errata = NULL,
				    .read_counts = core2_Read_Counts,
				    .check_overflow_gp_errata =
					    corei7_Check_Overflow_Errata,
				    .read_ro = NULL,
				    .platform_info = corei7_Platform_Info,
				    .trigger_read = NULL,
				    .scan_for_uncore = NULL,
				    .read_metrics = NULL };

DISPATCH_NODE corei7_dispatch_nehalem = {
	.init = core2_Initialize,
	.fini = core2_Destroy,
	.write = core2_Write_PMU,
	.freeze = core2_Disable_PMU,
	.restart = core2_Enable_PMU,
	.read_data = core2_Read_PMU_Data,
	.check_overflow = core2_Check_Overflow,
	.swap_group = core2_Swap_Group,
	.read_lbrs = corei7_Read_LBRs,
	.cleanup = core2_Clean_Up,
	.hw_errata = corei7_Errata_Fix,
	.read_power = corei7_Read_Power,
	.check_overflow_errata = NULL,
	.read_counts = core2_Read_Counts,
	.check_overflow_gp_errata = corei7_Check_Overflow_Errata,
	.read_ro = NULL,
	.platform_info = corei7_Platform_Info_Nehalem,
	.trigger_read = NULL,
	.scan_for_uncore = NULL,
	.read_metrics = NULL
};

DISPATCH_NODE corei7_dispatch_htoff_mode = {
	.init = core2_Initialize,
	.fini = core2_Destroy,
	.write = core2_Write_PMU,
	.freeze = core2_Disable_PMU,
	.restart = core2_Enable_PMU,
	.read_data = core2_Read_PMU_Data,
	.check_overflow = core2_Check_Overflow_Htoff_Mode,
	.swap_group = core2_Swap_Group,
	.read_lbrs = corei7_Read_LBRs,
	.cleanup = core2_Clean_Up,
	.hw_errata = corei7_Errata_Fix,
	.read_power = corei7_Read_Power,
	.check_overflow_errata = NULL,
	.read_counts = core2_Read_Counts,
	.check_overflow_gp_errata = corei7_Check_Overflow_Errata,
	.read_ro = NULL,
	.platform_info = corei7_Platform_Info,
	.trigger_read = NULL,
	.scan_for_uncore = NULL,
	.read_metrics = NULL
};

DISPATCH_NODE corei7_dispatch_htoff_mode_2 = {
	.init = core2_Initialize,
	.fini = core2_Destroy,
	.write = core2_Write_PMU,
	.freeze = core2_Disable_PMU,
	.restart = corei7_Enable_PMU_2,
	.read_data = core2_Read_PMU_Data,
	.check_overflow = core2_Check_Overflow_Htoff_Mode,
	.swap_group = core2_Swap_Group,
	.read_lbrs = corei7_Read_LBRs,
	.cleanup = core2_Clean_Up,
	.hw_errata = corei7_Errata_Fix_2,
	.read_power = corei7_Read_Power,
	.check_overflow_errata = NULL,
	.read_counts = core2_Read_Counts,
	.check_overflow_gp_errata = corei7_Check_Overflow_Errata,
	.read_ro = NULL,
	.platform_info = corei7_Platform_Info,
	.trigger_read = NULL,
	.scan_for_uncore = NULL,
	.read_metrics = NULL
};
