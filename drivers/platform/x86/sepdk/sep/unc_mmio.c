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

#include <linux/pci.h>

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
#include "unc_common.h"
#include "ecb_iterators.h"
#include "pebs.h"
#include "inc/pci.h"

extern U64 *read_counter_info;
extern U64 *prev_counter_data;
extern DRV_CONFIG drv_cfg;
extern EMON_BUFFER_DRIVER_HELPER emon_buffer_driver_helper;

#define MASK_32BIT 0xffffffff
#define MASK_64BIT 0xffffffff00000000ULL

#define IS_MASTER(device_type, cpu)                                            \
	(((device_type) == DRV_SINGLE_INSTANCE) ?                              \
		 CPU_STATE_system_master(&pcb[cpu]) :                          \
		 CPU_STATE_socket_master(&pcb[(cpu)]))
#define GET_PACKAGE_NUM(device_type, cpu)                                      \
	(((device_type) == DRV_SINGLE_INSTANCE) ? 0 : core_to_package_map[cpu])
#define IS_64BIT(mask) (((mask) >> 32) != 0)

#define EVENT_COUNTER_MAX_TRY 30

struct FPGA_CONTROL_NODE_S {
	union {
		struct {
			U64 rst_ctrs : 1;
			U64 rsvd1 : 7;
			U64 frz : 1;
			U64 rsvd2 : 7;
			U64 event_select : 4;
			U64 port_id : 2;
			U64 rsvd3 : 1;
			U64 port_enable : 1;
			U64 rsvd4 : 40;
		} bits;
		U64 bit_field;
	} u;
};

static struct FPGA_CONTROL_NODE_S control_node;

/*!
 * @fn          static VOID unc_mmio_Write_PMU(VOID*)
 *
 * @brief       Initial write of PMU registers
 *              Walk through the enties and write the value of the register accordingly.
 *              When current_group = 0, then this is the first time this routine is called,
 *
 * @param       None
 *
 * @return      None
 *
 * <I>Special Notes:</I>
 */
static VOID unc_mmio_Write_PMU(VOID *param)
{
	U32 dev_idx;
	U32 offset_delta = 0;
	DEV_UNC_CONFIG pcfg_unc;
	U32 event_id = 0;
	U64 tmp_value = 0;
	U32 this_cpu;
	U32 package_num = 0;
	U32 cur_grp;
	ECB pecb;
	U64 virtual_addr = 0;
	U32 idx_w = 0;
	U32 event_code = 0;
	U32 counter = 0;
	U32 entry = 0;
	U32 dev_node = 0;

	SEP_DRV_LOG_TRACE_IN("Param: %p.", param);

	dev_idx = *((U32 *)param);
	this_cpu = CONTROL_THIS_CPU();
	pcfg_unc = (DEV_UNC_CONFIG)LWPMU_DEVICE_pcfg(&devices[dev_idx]);
	if (!IS_MASTER(DEV_UNC_CONFIG_device_type(pcfg_unc), this_cpu)) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!is_master).");
		return;
	}

	package_num =
		GET_PACKAGE_NUM(DEV_UNC_CONFIG_device_type(pcfg_unc), this_cpu);
	cur_grp = LWPMU_DEVICE_cur_group(&devices[(dev_idx)])[package_num];
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[(dev_idx)])[(cur_grp)];
	if (!pecb) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!pecb).");
		return;
	}

	dev_node = ECB_dev_node(pecb);
	entry = package_num;
	if (!IS_MMIO_MAP_VALID(dev_node, entry)) {
		SEP_DRV_LOG_ERROR_TRACE_OUT("Early exit (!IS_MMIO_MAP_VALID).");
		return;
	}

	virtual_addr = virtual_address_table(dev_node, entry);

	FOR_EACH_REG_UNC_OPERATION(pecb, dev_idx, idx, PMU_OPERATION_WRITE)
	{
		PCI_MMIO_Write_U64(virtual_addr, ECB_entries_reg_id(pecb, idx),
				   ECB_entries_reg_value(pecb, idx));
	}
	END_FOR_EACH_REG_UNC_OPERATION;

	if (DRV_CONFIG_emon_mode(drv_cfg)) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!event_based_counts).");
		return;
	}

	idx_w = ECB_operations_register_start(pecb, PMU_OPERATION_WRITE);
	FOR_EACH_REG_UNC_OPERATION(pecb, dev_idx, idx, PMU_OPERATION_READ)
	{
		if (ECB_entries_reg_offset(pecb, idx) >
		    DRV_PCI_DEVICE_ENTRY_base_offset_for_mmio(
			    &ECB_pcidev_entry_node(pecb))) {
			offset_delta =
				ECB_entries_reg_offset(pecb, idx) -
				DRV_PCI_DEVICE_ENTRY_base_offset_for_mmio(
					&ECB_pcidev_entry_node(pecb));
		} else {
			offset_delta = ECB_entries_reg_offset(pecb, idx);
		}

		if ((DEV_UNC_CONFIG_device_type(pcfg_unc) ==
		     DRV_SINGLE_INSTANCE) &&
		    (GET_NUM_MAP_ENTRIES(dev_node) > 1)) {
			// multiple MMIO mapping per <dev_no, func_no> device, find virtual_addr per mapping.
			entry = ECB_entries_unit_id(pecb, idx);
			virtual_addr = virtual_address_table(dev_node, entry);
		}

		if ((ECB_entries_counter_type(pecb, idx) ==
		     PROG_FREERUN_COUNTER) &&
		    (ECB_entries_unit_id(pecb, idx) == 0)) {
			//Write event code before reading
			PCI_MMIO_Write_U64(virtual_addr,
					   ECB_entries_reg_id(pecb, idx_w),
					   ECB_entries_reg_value(pecb, idx_w));
			event_code = (U32)control_node.u.bits.event_select;
			idx_w++;
		}

		// this is needed for overflow detection of the accumulators.
		if (IS_64BIT((U64)(ECB_entries_max_bits(pecb, idx)))) {
			if (ECB_entries_counter_type(pecb, idx) ==
			    PROG_FREERUN_COUNTER) {
				do {
					if (counter > EVENT_COUNTER_MAX_TRY) {
						break;
					}
					tmp_value = SYS_MMIO_Read64(
						virtual_addr, offset_delta);
					counter++;
				} while (event_code != (tmp_value >> 60));
			}
			tmp_value = SYS_MMIO_Read64(virtual_addr, offset_delta);
		} else {
			tmp_value = SYS_MMIO_Read32(virtual_addr, offset_delta);
		}
		tmp_value &= (U64)ECB_entries_max_bits(pecb, idx);

		LWPMU_DEVICE_prev_value(
			&devices[dev_idx])[package_num][event_id] = tmp_value;
		SEP_DRV_LOG_TRACE(
			"unc_mmio_Write_PMU: cpu[%d], device[%d], package[%d], entry %d, event_id %d, value %llu\n",
			this_cpu, dev_idx, package_num, entry, event_id,
			tmp_value);
		event_id++;

		if (LWPMU_DEVICE_counter_mask(&devices[dev_idx]) == 0) {
			LWPMU_DEVICE_counter_mask(&devices[dev_idx]) =
				(U64)ECB_entries_max_bits(pecb, idx);
		}
	}
	END_FOR_EACH_REG_UNC_OPERATION;
	SEP_DRV_LOG_TRACE(
		"BAR address is 0x%llx and virt is 0x%llx.",
		DRV_PCI_DEVICE_ENTRY_bar_address(&ECB_pcidev_entry_node(pecb)),
		virtual_addr);

	SEP_DRV_LOG_TRACE_OUT("");
}

/*!
 * @fn         static VOID unc_mmio_Enable_PMU(PVOID)
 *
 * @brief      Capture the previous values to calculate delta later.
 *
 * @param      None
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
static void unc_mmio_Enable_PMU(PVOID param)
{
	U32 j;
	U64 *buffer = prev_counter_data;
	U32 this_cpu;
	U32 dev_idx;
	DEV_UNC_CONFIG pcfg_unc;
	U32 package_num;
	U32 offset_delta;
	U32 cur_grp;
	ECB pecb;
	U64 virtual_addr = 0;
	U64 reg_val = 0;
	U32 idx_w = 0;
	U32 event_code = 0;
	U32 counter = 0;
	// U32 num_events = 0;
	U32 entry = 0;
	// U32 num_pkgs = num_packages;
	U32 dev_node = 0;

	SEP_DRV_LOG_TRACE_IN("Param: %p.", param);

	dev_idx = *((U32 *)param);
	this_cpu = CONTROL_THIS_CPU();
	pcfg_unc = (DEV_UNC_CONFIG)LWPMU_DEVICE_pcfg(&devices[dev_idx]);
	if (!IS_MASTER(DEV_UNC_CONFIG_device_type(pcfg_unc), this_cpu)) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!IS_MASTER).");
		return;
	}

	package_num =
		GET_PACKAGE_NUM(DEV_UNC_CONFIG_device_type(pcfg_unc), this_cpu);
	cur_grp = LWPMU_DEVICE_cur_group(&devices[(dev_idx)])[package_num];
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[(dev_idx)])[(cur_grp)];
	if (!pecb) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!pecb).");
		return;
	}

	dev_node = ECB_dev_node(pecb);
	entry = package_num;
	if (!IS_MMIO_MAP_VALID(dev_node, entry)) {
		SEP_DRV_LOG_ERROR_TRACE_OUT("Early exit (!IS_MMIO_MAP_VALID).");
		return;
	}

	// if (DEV_UNC_CONFIG_device_type(pcfg_unc) == DRV_SINGLE_INSTANCE) {
	// 	num_pkgs = 1;
	// }

	virtual_addr = virtual_address_table(dev_node, entry);

	// NOTE THAT the enable function currently captures previous values
	// for EMON collection to avoid unnecessary memory copy.
	if (DRV_CONFIG_emon_mode(drv_cfg)) {
		// num_events = ECB_num_events(pecb);
		idx_w = ECB_operations_register_start(pecb,
						      PMU_OPERATION_WRITE);
		FOR_EACH_REG_UNC_OPERATION(pecb, dev_idx, idx,
					   PMU_OPERATION_READ)
		{
			if (ECB_entries_reg_offset(pecb, idx) >
			    DRV_PCI_DEVICE_ENTRY_base_offset_for_mmio(
				    &ECB_pcidev_entry_node(pecb))) {
				offset_delta =
					ECB_entries_reg_offset(pecb, idx) -
					DRV_PCI_DEVICE_ENTRY_base_offset_for_mmio(
						&ECB_pcidev_entry_node(pecb));
			} else {
				offset_delta =
					ECB_entries_reg_offset(pecb, idx);
			}

			if ((DEV_UNC_CONFIG_device_type(pcfg_unc) ==
			     DRV_SINGLE_INSTANCE) &&
			    (GET_NUM_MAP_ENTRIES(dev_node) > 1)) {
				// multiple MMIO mapping per <dev_no, func_no> device, find virtual_addr per mapping.
				entry = ECB_entries_unit_id(pecb, idx);
				virtual_addr =
					virtual_address_table(dev_node, entry);
			}

			if ((ECB_entries_counter_type(pecb, idx) ==
			     PROG_FREERUN_COUNTER) &&
			    (ECB_entries_unit_id(pecb, idx) == 0)) {
				PCI_MMIO_Write_U64(
					virtual_addr,
					ECB_entries_reg_id(pecb, idx_w),
					ECB_entries_reg_value(pecb, idx_w));
				control_node.u.bit_field =
					ECB_entries_reg_value(pecb, idx_w);
				event_code =
					(U32)control_node.u.bits.event_select;
				idx_w++;
			}

			if ((ECB_entries_event_scope(pecb, idx) ==
			     PACKAGE_EVENT) ||
			    (ECB_entries_event_scope(pecb, idx) ==
			     SYSTEM_EVENT)) {
				if (ECB_entries_event_scope(pecb, idx) ==
				    SYSTEM_EVENT) {
					j = ECB_entries_uncore_buffer_offset_in_system(
						pecb, idx);
				} else {
					j = EMON_BUFFER_UNCORE_PACKAGE_EVENT_OFFSET(
						package_num,
						EMON_BUFFER_DRIVER_HELPER_num_entries_per_package(
							emon_buffer_driver_helper),
						ECB_entries_uncore_buffer_offset_in_package(
							pecb, idx));
				}

				if (IS_64BIT((U64)(
					    ECB_entries_max_bits(pecb, idx)))) {
					if (ECB_entries_counter_type(pecb,
								     idx) ==
					    PROG_FREERUN_COUNTER) {
						do {
							if (counter >
							    EVENT_COUNTER_MAX_TRY) {
								break;
							}
							buffer[j] = SYS_MMIO_Read64(
								virtual_addr,
								offset_delta);
							counter++;
						} while (event_code !=
							 (buffer[j] >> 60));
					}
					buffer[j] = SYS_MMIO_Read64(
						virtual_addr, offset_delta);
				} else {
					buffer[j] = SYS_MMIO_Read32(
						virtual_addr, offset_delta);
				}
				buffer[j] &=
					(U64)ECB_entries_max_bits(pecb, idx);
				SEP_DRV_LOG_TRACE(
					"j=%u, value=%llu, cpu=%u, MSR=0x%x", j,
					buffer[j], this_cpu,
					ECB_entries_reg_id(pecb, idx));
			}
		}
		END_FOR_EACH_REG_UNC_OPERATION;
	}
	virtual_addr = virtual_address_table(dev_node, entry);
	FOR_EACH_REG_UNC_OPERATION(pecb, dev_idx, idx, PMU_OPERATION_ENABLE)
	{
		if (ECB_entries_reg_rw_type(pecb, idx) ==
		    PMU_REG_RW_READ_WRITE) {
			reg_val = PCI_MMIO_Read_U64(
				virtual_addr, ECB_entries_reg_id(pecb, idx));
			reg_val &= ECB_entries_reg_value(pecb, idx);
			PCI_MMIO_Write_U64(virtual_addr,
					   ECB_entries_reg_id(pecb, idx),
					   reg_val);
		}
	}
	END_FOR_EACH_REG_UNC_OPERATION;

	SEP_DRV_LOG_TRACE_OUT("");
}

/*!
 * @fn         static VOID unc_mmio_Disable_PMU(PVOID)
 *
 * @brief      Unmap the virtual address when you stop sampling.
 *
 * @param      None
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
static void unc_mmio_Disable_PMU(PVOID param)
{
	U32 dev_idx;
	U32 this_cpu;
	U64 virtual_addr = 0;
	U64 reg_val = 0;
	DEV_UNC_CONFIG pcfg_unc;
	U32 package_num;
	U32 dev_node = 0;
	U32 cur_grp = 0;
	ECB pecb;
	U32 entry = 0;

	SEP_DRV_LOG_TRACE_IN("Param: %p.", param);

	dev_idx = *((U32 *)param);
	this_cpu = CONTROL_THIS_CPU();
	pcfg_unc = (DEV_UNC_CONFIG)LWPMU_DEVICE_pcfg(&devices[dev_idx]);
	if (!IS_MASTER(DEV_UNC_CONFIG_device_type(pcfg_unc), this_cpu)) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!IS_MASTER).");
		return;
	}

	package_num =
		GET_PACKAGE_NUM(DEV_UNC_CONFIG_device_type(pcfg_unc), this_cpu);
	cur_grp = LWPMU_DEVICE_cur_group(&devices[dev_idx])[package_num];
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[dev_idx])[(cur_grp)];
	if (!pecb) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!pecb).");
		return;
	}

	dev_node = ECB_dev_node(pecb);
	entry = package_num;
	if (!IS_MMIO_MAP_VALID(dev_node, entry)) {
		SEP_DRV_LOG_ERROR_TRACE_OUT("Early exit (!IS_MMIO_MAP_VALID).");
		return;
	}

	virtual_addr = virtual_address_table(dev_node, entry);

	FOR_EACH_REG_UNC_OPERATION(pecb, dev_idx, idx, PMU_OPERATION_DISABLE)
	{
		if (ECB_entries_reg_rw_type(pecb, idx) ==
		    PMU_REG_RW_READ_WRITE) {
			reg_val = PCI_MMIO_Read_U64(
				virtual_addr, ECB_entries_reg_id(pecb, idx));
			reg_val |= ECB_entries_reg_value(pecb, idx);
			PCI_MMIO_Write_U64(virtual_addr,
					   ECB_entries_reg_id(pecb, idx),
					   reg_val);
		}
	}
	END_FOR_EACH_REG_UNC_OPERATION;

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn       void unc_mmio_Trigger_Read(id)
 *
 * @param    id       Device index
 *
 * @return   None     No return needed
 *
 * @brief    Read the Uncore data from counters and store into buffer
 */
static VOID unc_mmio_Trigger_Read(PVOID param, U32 id)
{
	U32 this_cpu;
	U32 cur_grp;
	ECB pecb;
	U32 index = 0;
	U64 diff = 0;
	U32 offset_delta = 0;
	U64 value = 0ULL;
	U64 *data;
	U64 virtual_addr = 0;
	DEV_UNC_CONFIG pcfg_unc;
	U32 package_num;
	U32 idx_w = 0;
	U32 event_code = 0;
	U32 counter = 0;
	U32 entry = 0;
	U32 dev_node = 0;

	SEP_DRV_LOG_TRACE_IN("Param: %p, id: %u.", param, id);

	this_cpu = CONTROL_THIS_CPU();
	pcfg_unc = (DEV_UNC_CONFIG)LWPMU_DEVICE_pcfg(&devices[id]);
	if (!IS_MASTER(DEV_UNC_CONFIG_device_type(pcfg_unc), this_cpu)) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!IS_MASTER).");
		return;
	}

	package_num =
		GET_PACKAGE_NUM(DEV_UNC_CONFIG_device_type(pcfg_unc), this_cpu);
	cur_grp = LWPMU_DEVICE_cur_group(&devices[id])[package_num];
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[id])[(cur_grp)];
	if (!pecb) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!pecb).");
		return;
	}

	dev_node = ECB_dev_node(pecb);
	entry = package_num;
	if (!IS_MMIO_MAP_VALID(dev_node, entry)) {
		SEP_DRV_LOG_ERROR_TRACE_OUT("Early exit (!IS_MMIO_MAP_VALID).");
		return;
	}

	virtual_addr = virtual_address_table(dev_node, entry);

	// Write GroupID
	data = (U64 *)((S8 *)param + ECB_group_offset(pecb));
	*data = cur_grp + 1;
	//Read in the counts into temporary buffer
	idx_w = ECB_operations_register_start(pecb, PMU_OPERATION_WRITE);
	FOR_EACH_REG_UNC_OPERATION(pecb, id, idx, PMU_OPERATION_READ)
	{
		if (ECB_entries_reg_offset(pecb, idx) >
		    DRV_PCI_DEVICE_ENTRY_base_offset_for_mmio(
			    &ECB_pcidev_entry_node(pecb))) {
			offset_delta =
				ECB_entries_reg_offset(pecb, idx) -
				DRV_PCI_DEVICE_ENTRY_base_offset_for_mmio(
					&ECB_pcidev_entry_node(pecb));
		} else {
			offset_delta = ECB_entries_reg_offset(pecb, idx);
		}

		if ((DEV_UNC_CONFIG_device_type(pcfg_unc) ==
		     DRV_SINGLE_INSTANCE) &&
		    (GET_NUM_MAP_ENTRIES(dev_node) > 1)) {
			// multiple MMIO mapping per <dev_no, func_no> device
			entry = ECB_entries_unit_id(pecb, idx);
			virtual_addr = virtual_address_table(dev_node, entry);
		}

		if ((ECB_entries_counter_type(pecb, idx) ==
		     PROG_FREERUN_COUNTER) &&
		    (ECB_entries_unit_id(pecb, idx) == 0)) {
			PCI_MMIO_Write_U64(virtual_addr,
					   ECB_entries_reg_id(pecb, idx_w),
					   ECB_entries_reg_value(pecb, idx_w));
			control_node.u.bit_field =
				ECB_entries_reg_value(pecb, idx_w);
			event_code = (U32)control_node.u.bits.event_select;
			idx_w++;
		}

		if (IS_64BIT((U64)(ECB_entries_max_bits(pecb, idx)))) {
			if (ECB_entries_counter_type(pecb, idx) ==
			    PROG_FREERUN_COUNTER) {
				do {
					if (counter > EVENT_COUNTER_MAX_TRY) {
						break;
					}
					value = SYS_MMIO_Read64(virtual_addr,
								offset_delta);
					counter++;
				} while (event_code != (value >> 60));
			}
			value = SYS_MMIO_Read64(virtual_addr, offset_delta);
		} else {
			value = SYS_MMIO_Read32((volatile unsigned int *)virtual_addr, offset_delta);
		}
		value &= (U64)ECB_entries_max_bits(pecb, idx);

		data = (U64 *)((S8 *)param +
			       ECB_entries_counter_event_offset(pecb, idx));
		//check for overflow if not a static counter
		if (ECB_entries_counter_type(pecb, idx) == STATIC_COUNTER) {
			*data = value;
		} else {
			if (value < LWPMU_DEVICE_prev_value(
					    &devices[id])[package_num][index]) {
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
				&devices[id])[package_num][cur_grp][index] +=
				diff;
			LWPMU_DEVICE_prev_value(
				&devices[id])[package_num][index] = value;
			*data = LWPMU_DEVICE_acc_value(
				&devices[id])[package_num][cur_grp][index];
		}
		index++;
	}
	END_FOR_EACH_REG_UNC_OPERATION;

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn unc_mmio_Read_PMU_Data(param)
 *
 * @param    param    dummy parameter which is not used
 *
 * @return   None     No return needed
 *
 * @brief    Read all the data MSR's into a buffer.  Called by the interrupt handler.
 *
 */
static VOID unc_mmio_Read_PMU_Data(PVOID param)
{
	U32 j;
	U64 *buffer = read_counter_info;
	U64 *prev_buffer = prev_counter_data;
	U32 this_cpu;
	U32 dev_idx;
	DEV_UNC_CONFIG pcfg_unc;
	U32 offset_delta;
	U32 cur_grp;
	ECB pecb;
	U64 tmp_value = 0ULL;
	U64 virtual_addr = 0;
	U32 idx_w = 0;
	U32 event_code = 0;
	U32 counter = 0;
	// U32 num_events = 0;
	U32 package_num;
	U32 entry = 0;
	U32 dev_node = 0;

	SEP_DRV_LOG_TRACE_IN("Param: %p.", param);

	dev_idx = *((U32 *)param);
	this_cpu = CONTROL_THIS_CPU();
	pcfg_unc = (DEV_UNC_CONFIG)LWPMU_DEVICE_pcfg(&devices[dev_idx]);
	if (!IS_MASTER(DEV_UNC_CONFIG_device_type(pcfg_unc), this_cpu)) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!IS_MASTER).");
		return;
	}

	package_num =
		GET_PACKAGE_NUM(DEV_UNC_CONFIG_device_type(pcfg_unc), this_cpu);
	cur_grp = LWPMU_DEVICE_cur_group(&devices[(dev_idx)])[package_num];
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[(dev_idx)])[(cur_grp)];
	if (!pecb) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!pecb).");
		return;
	}

	dev_node = ECB_dev_node(pecb);
	entry = package_num;
	if (!IS_MMIO_MAP_VALID(dev_node, entry)) {
		SEP_DRV_LOG_ERROR_TRACE_OUT("Early exit (!IS_MMIO_MAP_VALID).");
		return;
	}

	virtual_addr = virtual_address_table(dev_node, entry);

	// num_events = ECB_num_events(pecb);

	idx_w = ECB_operations_register_start(pecb, PMU_OPERATION_WRITE);

	FOR_EACH_REG_UNC_OPERATION(pecb, dev_idx, idx, PMU_OPERATION_READ)
	{
		if (ECB_entries_reg_offset(pecb, idx) >
		    DRV_PCI_DEVICE_ENTRY_base_offset_for_mmio(
			    &ECB_pcidev_entry_node(pecb))) {
			offset_delta =
				ECB_entries_reg_offset(pecb, idx) -
				DRV_PCI_DEVICE_ENTRY_base_offset_for_mmio(
					&ECB_pcidev_entry_node(pecb));
		} else {
			offset_delta = ECB_entries_reg_offset(pecb, idx);
		}

		if ((DEV_UNC_CONFIG_device_type(pcfg_unc) ==
		     DRV_SINGLE_INSTANCE) &&
		    (GET_NUM_MAP_ENTRIES(dev_node) > 1)) {
			// multiple MMIO mapping per <dev_no, func_no> device, find virtual_addr per mapping.
			entry = ECB_entries_unit_id(pecb, idx);
			virtual_addr = virtual_address_table(dev_node, entry);
		}

		if ((ECB_entries_counter_type(pecb, idx) ==
		     PROG_FREERUN_COUNTER) &&
		    (ECB_entries_unit_id(pecb, idx) == 0)) {
			PCI_MMIO_Write_U64(virtual_addr,
					   ECB_entries_reg_id(pecb, idx_w),
					   ECB_entries_reg_value(pecb, idx_w));
			control_node.u.bit_field =
				ECB_entries_reg_value(pecb, idx_w);
			event_code = (U32)control_node.u.bits.event_select;
			idx_w++;
		}

		if ((ECB_entries_event_scope(pecb, idx) == PACKAGE_EVENT) ||
		    (ECB_entries_event_scope(pecb, idx) == SYSTEM_EVENT)) {
			if (ECB_entries_event_scope(pecb, idx) ==
			    SYSTEM_EVENT) {
				j = ECB_entries_uncore_buffer_offset_in_system(
					pecb, idx);
			} else {
				j = EMON_BUFFER_UNCORE_PACKAGE_EVENT_OFFSET(
					package_num,
					EMON_BUFFER_DRIVER_HELPER_num_entries_per_package(
						emon_buffer_driver_helper),
					ECB_entries_uncore_buffer_offset_in_package(
						pecb, idx));
			}

			if (IS_64BIT((U64)(ECB_entries_max_bits(pecb, idx)))) {
				if (ECB_entries_counter_type(pecb, idx) ==
				    PROG_FREERUN_COUNTER) {
					do {
						if (counter >
						    EVENT_COUNTER_MAX_TRY) {
							break;
						}
						tmp_value = SYS_MMIO_Read64(
							virtual_addr,
							offset_delta);
						counter++;
					} while (event_code !=
						 (tmp_value >> 60));
				}
				tmp_value = SYS_MMIO_Read64(virtual_addr,
							    offset_delta);
			} else {
				tmp_value = SYS_MMIO_Read32(virtual_addr,
							    offset_delta);
			}
			tmp_value &= (U64)ECB_entries_max_bits(pecb, idx);
			if (ECB_entries_counter_type(pecb, idx) ==
			    STATIC_COUNTER) {
				buffer[j] = tmp_value;
			} else {
				if (tmp_value >= prev_buffer[j]) {
					buffer[j] = tmp_value - prev_buffer[j];
				} else {
					buffer[j] = tmp_value +
						    (ECB_entries_max_bits(pecb,
									  idx) -
						     prev_buffer[j]);
				}
			}
			SEP_DRV_LOG_TRACE("j=%u, value=%llu, cpu=%u, MSR=0x%x",
					  j, buffer[j], this_cpu,
					  ECB_entries_reg_id(pecb, idx));
		}
	}
	END_FOR_EACH_REG_UNC_OPERATION;

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn unc_mmio_Initialize(param)
 *
 * @param    param    dummy parameter which is not used
 *
 * @return   None     No return needed
 *
 * @brief    Do the mapping of the physical address (to do the invalidates in the TLB)
 *           NOTE: this should never be done with SMP call
 *
 */
static VOID unc_mmio_Initialize(PVOID param)
{
	DRV_PCI_DEVICE_ENTRY_NODE dpden;

	U64 bar;

	U64 physical_address;
	U32 dev_idx = 0;
	U32 cur_grp = 0;
	ECB pecb = NULL;
	U32 dev_node;
	U32 i = 0;
	U32 page_len = 4096; // 4K

	U32 use_default_busno = 0;
	U32 entries = 0;

	SEP_DRV_LOG_TRACE_IN("Param: %p.", param);

	dev_idx = *((U32 *)param);
	cur_grp = LWPMU_DEVICE_cur_group(&devices[(dev_idx)])[0];
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[dev_idx])[cur_grp];

	if (!pecb) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!pecb).");
		return;
	}
	dev_node = ECB_dev_node(pecb);

	if (IS_MMIO_MAP_VALID(dev_node, 0)) {
		SEP_DRV_LOG_INIT_TRACE_OUT(
			"Early exit (device[%d] node %d already mapped).",
			dev_idx, dev_node);
		return;
	}

	dpden = ECB_pcidev_entry_node(pecb);

	// use busno found from topology scan if available
	// otherwise use the default one
	entries = GET_NUM_MAP_ENTRIES(dev_node);
	if (entries == 0) {
		use_default_busno = 1;
		entries = 1; // this could the client, does not through the scan
		UNC_PCIDEV_num_entries(&(unc_pcidev_map[dev_node])) = 1;
		UNC_PCIDEV_max_entries(&(unc_pcidev_map[dev_node])) = 1;
	}
	if (!UNC_PCIDEV_mmio_map(&(unc_pcidev_map[dev_node]))) {
		// it is better to allocate space in the beginning
		UNC_PCIDEV_mmio_map(&(unc_pcidev_map[dev_node])) =
			CONTROL_Allocate_Memory(entries *
						sizeof(SEP_MMIO_NODE));
		if (UNC_PCIDEV_mmio_map(&(unc_pcidev_map[dev_node])) == NULL) {
			SEP_DRV_LOG_ERROR_TRACE_OUT("Early exit (No Memory).");
			return;
		}
		memset(UNC_PCIDEV_mmio_map(&(unc_pcidev_map[dev_node])), 0,
		       entries * sizeof(U64));
	}
	for (i = 0; i < entries; i++) {
		if (!use_default_busno) {
			if (IS_BUS_MAP_VALID(dev_node, i)) {
				DRV_PCI_DEVICE_ENTRY_bus_no(&dpden) =
					UNC_PCIDEV_busno_entry(
						&(unc_pcidev_map[dev_node]), i);
			}
		}

		bar = PCI_Read_U64(DRV_PCI_DEVICE_ENTRY_bus_no(&dpden),
				   DRV_PCI_DEVICE_ENTRY_dev_no(&dpden),
				   DRV_PCI_DEVICE_ENTRY_func_no(&dpden),
				   DRV_PCI_DEVICE_ENTRY_bar_offset(&dpden));

		bar &= DRV_PCI_DEVICE_ENTRY_bar_mask(&dpden);

		DRV_PCI_DEVICE_ENTRY_bar_address(&ECB_pcidev_entry_node(pecb)) =
			bar;
		physical_address = DRV_PCI_DEVICE_ENTRY_bar_address(
					   &ECB_pcidev_entry_node(pecb)) +
				   DRV_PCI_DEVICE_ENTRY_base_offset_for_mmio(
					   &ECB_pcidev_entry_node(pecb));

		PCI_Map_Memory(&UNC_PCIDEV_mmio_map_entry(
				       &(unc_pcidev_map[dev_node]), i),
			       physical_address, page_len);
	}
	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn unc_mmio_fpga_Initialize(param)
 *
 * @param    param    dummy parameter which is not used
 *
 * @return   None     No return needed
 *
 * @brief    Do the mapping of the physical address (to do the invalidates in the TLB)
 *           NOTE: this should never be done with SMP call
 *
 */
static VOID unc_mmio_fpga_Initialize(PVOID param)
{
#if defined(DRV_EM64T)
	U64 phys_addr;
	SEP_MMIO_NODE tmp_map = { 0 };
	U64 virt_addr;
	U64 dfh;
	U32 id;
	U32 offset = 0;
	S32 next_offset = -1;
	U32 dev_idx;
	U32 cur_grp;
	ECB pecb;
	U32 bus_list[2] = { 0x5e, 0xbe };
	U32 busno;
	U32 page_len = 4096;
	U32 package_num = 0;
	U32 dev_node = 0;
	U32 entries = 0;
	DRV_PCI_DEVICE_ENTRY_NODE dpden;

	SEP_DRV_LOG_TRACE_IN("Param: %p.", param);

	dev_idx = *((U32 *)param);
	cur_grp = LWPMU_DEVICE_cur_group(&devices[(dev_idx)])[0];
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[dev_idx])[cur_grp];

	if (!pecb) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!pecb).");
		return;
	}

	dev_node = ECB_dev_node(pecb);

	entries = GET_NUM_MAP_ENTRIES(dev_node);
	if (entries == 0) {
		entries = num_packages;
	}

	if (!UNC_PCIDEV_mmio_map(&(unc_pcidev_map[dev_node]))) {
		// it is better to allocate space in the beginning
		UNC_PCIDEV_mmio_map(&(unc_pcidev_map[dev_node])) =
			CONTROL_Allocate_Memory(entries *
						sizeof(SEP_MMIO_NODE));
		if (UNC_PCIDEV_mmio_map(&(unc_pcidev_map[dev_node])) == NULL) {
			SEP_DRV_LOG_ERROR_TRACE_OUT("Early exit (No Memory).");
			return;
		}
		memset(UNC_PCIDEV_mmio_map(&(unc_pcidev_map[dev_node])), 0,
		       (entries * sizeof(SEP_MMIO_NODE)));
		UNC_PCIDEV_num_entries(&(unc_pcidev_map[dev_node])) = 0;
		UNC_PCIDEV_max_entries(&(unc_pcidev_map[dev_node])) = entries;
	} else {
		if (virtual_address_table(dev_node, 0) != 0) {
			SEP_DRV_LOG_INIT_TRACE_OUT(
				"Early exit (device[%d] node %d already mapped).",
				dev_idx, dev_node);
			return;
		}
	}

	dpden = ECB_pcidev_entry_node(pecb);

	for (package_num = 0; package_num < num_packages; package_num++) {
		if (package_num < 2) {
			busno = bus_list[package_num];
		} else {
			busno = 0;
		}
		phys_addr =
			PCI_Read_U64(busno, DRV_PCI_DEVICE_ENTRY_dev_no(&dpden),
				     DRV_PCI_DEVICE_ENTRY_func_no(&dpden),
				     DRV_PCI_DEVICE_ENTRY_bar_offset(&dpden));
		phys_addr &= DRV_PCI_DEVICE_ENTRY_bar_mask(&dpden);
		if (package_num == 0) {
			PCI_Map_Memory(&tmp_map, phys_addr, 8 * page_len);
			virt_addr = SEP_MMIO_NODE_virtual_address(&tmp_map);
			while (next_offset != 0) {
				dfh = SYS_MMIO_Read64((U64)virt_addr, offset);
				next_offset = (U32)((dfh >> 16) & 0xffffff);
				id = (U32)(dfh & 0xfff);
				if (offset &&
				    (id ==
				     DRV_PCI_DEVICE_ENTRY_feature_id(&dpden))) {
					break;
				}
				offset += next_offset;
			}
			PCI_Unmap_Memory(&tmp_map);
		}
		phys_addr += offset;
		PCI_Map_Memory(
			&UNC_PCIDEV_mmio_map_entry(&(unc_pcidev_map[dev_node]),
						   package_num),
			phys_addr, 8 * page_len);
		UNC_PCIDEV_num_entries(&(unc_pcidev_map[dev_node]))++;
	}

	SEP_DRV_LOG_TRACE_OUT("");
#endif
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn unc_mmio_Destroy(param)
 *
 * @param    param    dummy parameter which is not used
 *
 * @return   None     No return needed
 *
 * @brief    Invalidate the entry in TLB of the physical address
 *           NOTE: this should never be done with SMP call
 *
 */
static VOID unc_mmio_Destroy(PVOID param)
{
	U32 dev_idx;
	U32 i;
	U64 addr = 0;
	U32 cur_grp = 0;
	U32 dev_node = 0;
	U32 entries = 0;
	ECB pecb;

	SEP_DRV_LOG_TRACE_IN("Param: %p.", param);

	dev_idx = *((U32 *)param);
	cur_grp = LWPMU_DEVICE_cur_group(&devices[(dev_idx)])[0];
	pecb = LWPMU_DEVICE_PMU_register_data(&devices[dev_idx])[cur_grp];

	if (!pecb) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!pecb).");
		return;
	}
	dev_node = ECB_dev_node(pecb);

	if (!UNC_PCIDEV_mmio_map(&(unc_pcidev_map[dev_node]))) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (no mapping).");
		return;
	}

	entries = GET_NUM_MAP_ENTRIES(dev_node);

	for (i = 0; i < entries; i++) {
		addr = virtual_address_table(dev_node, i);
		if (addr) {
			PCI_Unmap_Memory(&UNC_PCIDEV_mmio_map_entry(
				&(unc_pcidev_map[dev_node]), i));
		}
	}

	SEP_DRV_LOG_TRACE_OUT("");
}

/*
 * Initialize the dispatch table
 */
DISPATCH_NODE unc_mmio_dispatch = { .init = unc_mmio_Initialize,
				    .fini = unc_mmio_Destroy,
				    .write = unc_mmio_Write_PMU,
				    .freeze = unc_mmio_Disable_PMU,
				    .restart = unc_mmio_Enable_PMU,
				    .read_data = unc_mmio_Read_PMU_Data,
				    .check_overflow = NULL,
				    .swap_group = NULL,
				    .read_lbrs = NULL,
				    .cleanup = UNC_COMMON_Dummy_Func,
				    .hw_errata = NULL,
				    .read_power = NULL,
				    .check_overflow_errata = NULL,
				    .read_counts = NULL,
				    .check_overflow_gp_errata = NULL,
				    .read_ro = NULL,
				    .platform_info = NULL,
				    .trigger_read = unc_mmio_Trigger_Read,
				    .scan_for_uncore = NULL,
				    .read_metrics = NULL };

DISPATCH_NODE unc_mmio_fpga_dispatch = { .init = unc_mmio_fpga_Initialize,
					 .fini = unc_mmio_Destroy,
					 .write = unc_mmio_Write_PMU,
					 .freeze = unc_mmio_Disable_PMU,
					 .restart = unc_mmio_Enable_PMU,
					 .read_data = unc_mmio_Read_PMU_Data,
					 .check_overflow = NULL,
					 .swap_group = NULL,
					 .read_lbrs = NULL,
					 .cleanup = UNC_COMMON_Dummy_Func,
					 .hw_errata = NULL,
					 .read_power = NULL,
					 .check_overflow_errata = NULL,
					 .read_counts = NULL,
					 .check_overflow_gp_errata = NULL,
					 .read_ro = NULL,
					 .platform_info = NULL,
					 .trigger_read = unc_mmio_Trigger_Read,
					 .scan_for_uncore = NULL,
					 .read_metrics = NULL };
