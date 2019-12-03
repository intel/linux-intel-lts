/* ***********************************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(C) 2013-2019 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * BSD LICENSE
 *
 * Copyright(C) 2013-2019 Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ***********************************************************************************************
 */


#include "lwpmudrv_defines.h"
#include <linux/version.h>
#include <linux/wait.h>
#include <linux/fs.h>

#include "lwpmudrv_types.h"
#include "lwpmudrv_ecb.h"
#include "lwpmudrv_struct.h"

#include "inc/socperfdrv.h"
#include "inc/ecb_iterators.h"
#include "inc/pci.h"
#include "inc/control.h"
#include "inc/npk_uncore.h"

extern LWPMU_DEVICE device_uncore;
static U32 counter_overflow[SOC_NPK_COUNTER_MAX_COUNTERS];
static U64 counter_virtual_address;
static U64 mchbar_virtual_address;
static U64 mchbar_offset;

/*!
 * @fn          static ULONG read_From_Register(U64  bar_virtual_address,
												U64  mmio_offset,
												U32 *data_val)
 *
 * @brief       Reads register programming info
 *
 * @param       bar_virtual_address - memory address
 *              mmio_offset         - offset of the register
 *              data_val            - register value read
 *
 * @return      data from the counter register
 *
 * <I>Special Notes:</I>
 */
static void read_From_Register(U64 bar_virtual_address, U64 mmio_offset,
			       U32 *data_val)
{
	if (data_val) {
		*data_val = readl((void __iomem *)((char *)(UIOP)(bar_virtual_address) +
					  mmio_offset));
	}
}

/*!
 * @fn          static ULONG write_To_Register(U64  bar_virtual_address,
											   U64  mmio_offset,
											   U32  value)
 *
 * @brief       Write register programming info
 *
 * @param       bar_virtual_address - memory address
 *              mmio_offset         - offset of the register
 *              value               - register value to be written
 *
 * @return      none
 *
 * <I>Special Notes:</I>
 */
static void write_To_Register(U64 bar_virtual_address, U64 mmio_offset,
			      ULONG value)
{
	U32 read_reg = 0;

	writel(value,
	       (void __iomem *)(((char *)(UIOP)bar_virtual_address) + mmio_offset));
	read_From_Register(bar_virtual_address, mmio_offset, &read_reg);
}

/*!
 * @fn          static VOID uncore_Reset_Counters(U32 dev_idx)
 *
 * @brief       Reset counters
 *
 * @param       dev_idx - device index
 *
 * @return      None
 *
 * <I>Special Notes:</I>
 */
static VOID uncore_Reset_Counters(U32 dev_idx)
{
	U32 data_reg = 0;

	if (counter_virtual_address) {
		FOR_EACH_PCI_REG_RAW(pecb, i, dev_idx)
		{
			if (ECB_entries_reg_type(pecb, i) ==
			    PMU_REG_EVENT_SELECT) {
				data_reg =
					i + ECB_operations_register_len(
						    pecb, PMU_OPERATION_WRITE);
				if (ECB_entries_reg_type(pecb, data_reg) ==
				    PMU_REG_DATA) {
					write_To_Register(
						counter_virtual_address,
						ECB_entries_reg_offset(
							pecb, data_reg),
						(ULONG)0);
				}
				write_To_Register(counter_virtual_address,
						  ECB_entries_reg_offset(pecb,
									 i),
						  (ULONG)SOC_NPK_UNCORE_STOP);
			}
		}
		END_FOR_EACH_PCI_REG_RAW;
	}
}

/*!
 * @fn          static VOID uncore_Write_PMU(VOID*)
 *
 * @brief       Initial write of PMU registers
 *              Walk through the entries and write the value of the register accordingly.
 *              When current_group = 0, then this is the first time this routine is called,
 *
 * @param       param - device index
 *
 * @return      None
 *
 * <I>Special Notes:</I>
 */
static VOID uncore_Write_PMU(VOID *param)
{
	U32 dev_idx = *((U32 *)param);
	ECB pecb;
	DRV_PCI_DEVICE_ENTRY dpden;
	U32 pci_address;
	U32 bar_lo;
	U64 bar_hi;
	U64 final_bar;
	U64 physical_address;
	U32 dev_index = 0;
	S32 bar_list[SOC_NPK_UNCORE_MAX_PCI_DEVICES];
	U32 bar_index = 0;
	U64 virtual_address = 0;
	U32 bar_name = 0;
	DRV_PCI_DEVICE_ENTRY curr_pci_entry = NULL;
	U32 next_bar_offset = 0;
	U64 mmio_offset = 0;
	U32 i = 0;
	U32 map_size = 0;
	U32 cur_grp;

	if (device_uncore == NULL) {
		SOCPERF_PRINT_ERROR("ERROR: NULL device_uncore!\n");
		return;
	}
	cur_grp = LWPMU_DEVICE_cur_group(device_uncore);

	pecb = (ECB)LWPMU_DEVICE_PMU_register_data(device_uncore)[cur_grp];
	if (pecb == NULL) {
		SOCPERF_PRINT_ERROR("ERROR: null pecb!\n");
		return;
	}

	for (dev_index = 0; dev_index < SOC_NPK_UNCORE_MAX_PCI_DEVICES;
	     dev_index++) {
		bar_list[dev_index] = -1;
	}

	// initialize the per-counter overflow numbers
	for (i = 0; i < SOC_NPK_COUNTER_MAX_COUNTERS; i++) {
		counter_overflow[i] = 0;
		socperf_pcb[0].last_uncore_count[i] = 0;
	}

	ECB_pcidev_entry_list(pecb) = (DRV_PCI_DEVICE_ENTRY)(
		(S8 *)pecb + ECB_pcidev_list_offset(pecb));
	dpden = ECB_pcidev_entry_list(pecb);

	uncore_Reset_Counters(dev_idx);

	SOCPERF_PRINT_DEBUG(
		"Inside VISA Driver Write PMU: Number of entries=%d\n",
		ECB_num_pci_devices(pecb));
	for (dev_index = 0; dev_index < ECB_num_pci_devices(pecb);
	     dev_index++) {
		curr_pci_entry = &dpden[dev_index];
		bar_name = DRV_PCI_DEVICE_ENTRY_bar_name(curr_pci_entry);
		mmio_offset = DRV_PCI_DEVICE_ENTRY_base_offset_for_mmio(
			curr_pci_entry);

		// UNC_MMIO programming
		if (bar_list[bar_name] != -1) {
			bar_index = bar_list[bar_name];
			virtual_address = DRV_PCI_DEVICE_ENTRY_virtual_address(
				&dpden[bar_index]);
			DRV_PCI_DEVICE_ENTRY_virtual_address(curr_pci_entry) =
				DRV_PCI_DEVICE_ENTRY_virtual_address(
					&dpden[bar_index]);
			write_To_Register(virtual_address, mmio_offset,
					  (U32)DRV_PCI_DEVICE_ENTRY_value(
						  curr_pci_entry));
			continue;
		}

		pci_address = FORM_PCI_ADDR(
			DRV_PCI_DEVICE_ENTRY_bus_no(curr_pci_entry),
			DRV_PCI_DEVICE_ENTRY_dev_no(curr_pci_entry),
			DRV_PCI_DEVICE_ENTRY_func_no(curr_pci_entry),
			DRV_PCI_DEVICE_ENTRY_bar_offset(curr_pci_entry));
		bar_lo = SOCPERF_PCI_Read_Ulong(pci_address);
		SOCPERF_PRINT_DEBUG(
			"The bus=%x device=%x function=%x offset=%x\n",
			DRV_PCI_DEVICE_ENTRY_bus_no(curr_pci_entry),
			DRV_PCI_DEVICE_ENTRY_dev_no(curr_pci_entry),
			DRV_PCI_DEVICE_ENTRY_func_no(curr_pci_entry),
			DRV_PCI_DEVICE_ENTRY_bar_offset(curr_pci_entry));
		next_bar_offset =
			DRV_PCI_DEVICE_ENTRY_bar_offset(curr_pci_entry) +
			SOC_NPK_UNCORE_NEXT_ADDR_OFFSET;
		pci_address = FORM_PCI_ADDR(
			DRV_PCI_DEVICE_ENTRY_bus_no(curr_pci_entry),
			DRV_PCI_DEVICE_ENTRY_dev_no(curr_pci_entry),
			DRV_PCI_DEVICE_ENTRY_func_no(curr_pci_entry),
			next_bar_offset);
		bar_hi = SOCPERF_PCI_Read_Ulong(pci_address);
		SOCPERF_PRINT_DEBUG(
			"The bus=%x device=%x function=%x offset=%x\n",
			DRV_PCI_DEVICE_ENTRY_bus_no(curr_pci_entry),
			DRV_PCI_DEVICE_ENTRY_dev_no(curr_pci_entry),
			DRV_PCI_DEVICE_ENTRY_func_no(curr_pci_entry),
			next_bar_offset);
		final_bar = (bar_hi << SOC_NPK_UNCORE_BAR_ADDR_SHIFT) | bar_lo;
		if (bar_name == UNC_MCHBAR) {
			final_bar &= SOC_NPK_UNCORE_MCHBAR_ADDR_MASK;
			map_size = SOC_NPK_UNCORE_MCHBAR_MMIO_PAGE_SIZE;
		} else {
			final_bar &= SOC_NPK_UNCORE_BAR_ADDR_MASK;
			map_size = SOC_NPK_UNCORE_NPK_BAR_MMIO_PAGE_SIZE;
		}
		DRV_PCI_DEVICE_ENTRY_bar_address(curr_pci_entry) = final_bar;
		physical_address =
			DRV_PCI_DEVICE_ENTRY_bar_address(curr_pci_entry);

		if (physical_address) {
			DRV_PCI_DEVICE_ENTRY_virtual_address(curr_pci_entry) =
				(U64)(UIOP)ioremap_nocache(physical_address,
							   map_size);
			virtual_address = DRV_PCI_DEVICE_ENTRY_virtual_address(
				curr_pci_entry);

			write_To_Register(virtual_address, mmio_offset,
					  (U32)DRV_PCI_DEVICE_ENTRY_value(
						  curr_pci_entry));
			bar_list[bar_name] = dev_index;
			if (counter_virtual_address == 0) {
				counter_virtual_address = virtual_address;
			}
			if (mchbar_virtual_address == 0 &&
			    bar_name == UNC_MCHBAR) {
				mchbar_virtual_address = virtual_address;
				mchbar_offset = mmio_offset;
			}
		}
	}
}

/*!
 * @fn         static VOID uncore_Disable_PMU(PVOID)
 *
 * @brief      Unmap the virtual address when sampling/driver stops
 *
 * @param      param - device index
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
static VOID uncore_Disable_PMU(PVOID param)
{
	U32 dev_idx = *((U32 *)param);

	if (GLOBAL_STATE_current_phase(socperf_driver_state) ==
	    DRV_STATE_PREPARE_STOP) {
		uncore_Reset_Counters(dev_idx);
		if (mchbar_virtual_address) {
			write_To_Register(mchbar_virtual_address, mchbar_offset,
					  0x0);
			iounmap((void __iomem *)(UIOP)(mchbar_virtual_address));
			SOCPERF_PRINT_DEBUG("Unmapping MCHBAR address=%x\n",
					    mchbar_virtual_address);
		}
		if (counter_virtual_address) {
			iounmap((void __iomem *)(UIOP)(counter_virtual_address));
			SOCPERF_PRINT_DEBUG("Unmapping NPKBAR address=%x\n",
					    counter_virtual_address);
		}
		counter_virtual_address = 0;
		mchbar_virtual_address = 0;
		mchbar_offset = 0;
	}
}

/*!
 * @fn         static VOID uncore_Initialize(PVOID)
 *
 * @brief      Initialize any registers or addresses
 *
 * @param      param
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
static VOID uncore_Initialize(VOID *param)
{
	counter_virtual_address = 0;
	mchbar_virtual_address = 0;
	mchbar_offset = 0;
}

/*!
 * @fn         static VOID uncore_Clean_Up(PVOID)
 *
 * @brief      Reset any registers or addresses
 *
 * @param      param
 *
 * @return     None
 *
 * <I>Special Notes:</I>
 */
static VOID uncore_Clean_Up(VOID *param)
{
	counter_virtual_address = 0;
	mchbar_virtual_address = 0;
	mchbar_offset = 0;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn uncore_Read_Data()
 *
 * @param    None
 *
 * @return   None     No return needed
 *
 * @brief    Read the counters
 *
 */
static VOID uncore_Read_Data(PVOID data_buffer)
{
	U32 event_id = 0;
	U64 *data;
	int data_index;
	U32 data_val = 0;
	U32 data_reg = 0;
	U64 total_count = 0;
	U32 event_index = 0;
	U32 cur_grp;

	if (device_uncore == NULL) {
		SOCPERF_PRINT_ERROR("ERROR: NULL device_uncore!\n");
		return;
	}
	cur_grp = LWPMU_DEVICE_cur_group(device_uncore);

	if (GLOBAL_STATE_current_phase(socperf_driver_state) ==
		    DRV_STATE_UNINITIALIZED ||
	    GLOBAL_STATE_current_phase(socperf_driver_state) ==
		    DRV_STATE_IDLE ||
	    GLOBAL_STATE_current_phase(socperf_driver_state) ==
		    DRV_STATE_RESERVED ||
	    GLOBAL_STATE_current_phase(socperf_driver_state) ==
		    DRV_STATE_PREPARE_STOP ||
	    GLOBAL_STATE_current_phase(socperf_driver_state) ==
		    DRV_STATE_STOPPED) {
		SOCPERF_PRINT_ERROR("ERROR: RETURING EARLY from Read_Data\n");
		return;
	}

	if (data_buffer == NULL) {
		return;
	}

	data = (U64 *)data_buffer;
	data_index = 0;

	// Write GroupID
	data[data_index] = cur_grp + 1;
	// Increment the data index as the event id starts from zero
	data_index++;

	FOR_EACH_PCI_REG_RAW(pecb, i, dev_idx)
	{
		if (ECB_entries_reg_type(pecb, i) == PMU_REG_EVENT_SELECT) {
			write_To_Register(counter_virtual_address,
					  ECB_entries_reg_offset(pecb, i),
					  (ULONG)SOC_NPK_UNCORE_SAMPLE_DATA);

			data_reg = i + ECB_operations_register_len(
					       pecb, PMU_OPERATION_WRITE);
			if (ECB_entries_reg_type(pecb, data_reg) ==
			    PMU_REG_DATA) {
				read_From_Register(
					counter_virtual_address,
					ECB_entries_reg_offset(pecb, data_reg),
					&data_val);
				if (data_val <
				    socperf_pcb[0]
					    .last_uncore_count[event_index]) {
					counter_overflow[event_index]++;
				}
				socperf_pcb[0].last_uncore_count[event_index] =
					data_val;
				total_count = data_val +
					      counter_overflow[event_index] *
						      SOC_NPK_COUNTER_MAX_COUNT;
				event_index++;
				data[data_index + event_id] = total_count;
				SOCPERF_PRINT_DEBUG("DATA[%d]=%llu\n", event_id,
						    total_count);
				event_id++;
			}
		}
	}
	END_FOR_EACH_PCI_REG_RAW;
}

/*
 * Initialize the dispatch table
 */
DISPATCH_NODE npk_dispatch = {
	.init = uncore_Initialize, // initialize
	.fini = NULL, // destroy
	.write = uncore_Write_PMU, // write
	.freeze = uncore_Disable_PMU, // freeze
	.restart = NULL, // restart
	.read_data = NULL, // read
	.check_overflow = NULL, // check for overflow
	.swap_group = NULL,
	.read_lbrs = NULL,
	.clean_up = uncore_Clean_Up,
	.hw_errata = NULL,
	.read_power = NULL,
	.check_overflow_errata = NULL,
	.read_counts = NULL, //read_counts
	.check_overflow_gp_errata = NULL,
	.read_power = NULL,
	.platform_info = NULL,
	.trigger_read = NULL,
	.read_current_data = uncore_Read_Data,
	.create_mem = NULL,
	.check_status = NULL,
	.read_mem = NULL,
	.stop_mem = NULL
};
