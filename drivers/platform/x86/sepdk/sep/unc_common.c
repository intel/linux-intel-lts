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
#include "inc/pci.h"
#include "inc/unc_common.h"
#include "inc/utility.h"

extern UNCORE_TOPOLOGY_INFO_NODE uncore_topology;
extern PLATFORM_TOPOLOGY_PROG_NODE platform_topology_prog_node;
extern U64 *read_counter_info;

/* this is the table to keep pci_bus structure for PCI devices
 * for both pci config access and mmio access
 */
UNC_PCIDEV_NODE unc_pcidev_map[MAX_DEVICES];

#define GET_PACKAGE_NUM(device_type, cpu)                                      \
	(((device_type) == DRV_SINGLE_INSTANCE) ? 0 : core_to_package_map[cpu])

/************************************************************/
/*
 * unc common Dispatch functions
 *
 ************************************************************/
void UNC_COMMON_Dummy_Func(PVOID param)
{
	SEP_DRV_LOG_TRACE_IN("Dummy param: %p.", param);
	SEP_DRV_LOG_TRACE_OUT("Empty function.");
}

/************************************************************/
/*
 * UNC common PCI  based API
 *
 ************************************************************/

/*!
 * @fn          OS_STATUS UNC_COMMON_Add_Bus_Map
 *
 * @brief       This code discovers which package's data is read off of which bus.
 *
 * @param       None
 *
 * @return      OS_STATUS
 *
 * <I>Special Notes:</I>
 *     This probably will move to the UBOX once that is programmed.
 */
OS_STATUS
UNC_COMMON_Add_Bus_Map(U32 uncore_did, U32 dev_node, U32 bus_no)
{
	U32 i = 0;
	U32 entries = 0;

	if (!UNC_PCIDEV_busno_list(&(unc_pcidev_map[dev_node]))) {
		// allocate array for holding bus mapping
		// package based device: an entry per package, all units in the same package are in the same bus.
		// system based device:  an entry per unit if in different bus
		entries = GET_MAX_PCIDEV_ENTRIES(num_packages);
		UNC_PCIDEV_busno_list(&(unc_pcidev_map[dev_node])) =
			CONTROL_Allocate_Memory(entries * sizeof(S32));
		if (UNC_PCIDEV_busno_list(&(unc_pcidev_map[dev_node])) ==
		    NULL) {
			SEP_DRV_LOG_ERROR("Memory allocation failure!");
			return OS_NO_MEM;
		}
		UNC_PCIDEV_num_entries(&(unc_pcidev_map[dev_node])) = 0;
		UNC_PCIDEV_max_entries(&(unc_pcidev_map[dev_node])) = entries;
		for (i = 0; i < entries; i++) {
			UNC_PCIDEV_busno_entry(&(unc_pcidev_map[dev_node]), i) =
				INVALID_BUS_NUMBER;
		}
	} else {
		entries = UNC_PCIDEV_max_entries(&(unc_pcidev_map[dev_node]));
	}

	for (i = 0; i < UNC_PCIDEV_num_entries(&(unc_pcidev_map[dev_node]));
	     i++) {
		if (UNC_PCIDEV_busno_entry(&(unc_pcidev_map[dev_node]), i) ==
		    (S32)bus_no) {
			SEP_DRV_LOG_TRACE(
				"Already in the map,  another unit, no add.");
			return OS_SUCCESS;
		}
	}
	if (i < entries) {
		UNC_PCIDEV_busno_entry(&(unc_pcidev_map[dev_node]), i) =
			(S32)bus_no;
		UNC_PCIDEV_num_entries(&(unc_pcidev_map[dev_node]))++;
		SEP_DRV_LOG_TRACE("Add numpackages=%d busno=%x  devnode=%d.",
				  num_packages, bus_no, dev_node);
		return OS_SUCCESS;
	}
	SEP_DRV_LOG_ERROR_TRACE_OUT(
		"Exceed max map entries, drop this bus map!");
	return OS_NO_MEM;
}

OS_STATUS UNC_COMMON_Init(void)
{
	U32 i = 0;

	for (i = 0; i < MAX_DEVICES; i++) {
		memset(&(unc_pcidev_map[i]), 0, sizeof(UNC_PCIDEV_NODE));
	}

	memset((char *)&uncore_topology, 0, sizeof(UNCORE_TOPOLOGY_INFO_NODE));
	memset((char *)&platform_topology_prog_node, 0,
	       sizeof(PLATFORM_TOPOLOGY_PROG_NODE));

	return OS_SUCCESS;
}

/*!
 * @fn         extern VOID UNC_COMMON_Clean_Up(PVOID)
 *
 * @brief      clear out out programming
 *
 * @param      None
 *
 * @return     None
 */
void UNC_COMMON_Clean_Up(void)
{
	U32 i = 0;
	for (i = 0; i < MAX_DEVICES; i++) {
		if (UNC_PCIDEV_busno_list(&(unc_pcidev_map[i]))) {
			UNC_PCIDEV_busno_list(&(unc_pcidev_map[i])) =
				CONTROL_Free_Memory(UNC_PCIDEV_busno_list(
					&(unc_pcidev_map[i])));
		}
		if (UNC_PCIDEV_mmio_map(&(unc_pcidev_map[i]))) {
			UNC_PCIDEV_mmio_map(&(unc_pcidev_map[i])) =
				CONTROL_Free_Memory(UNC_PCIDEV_mmio_map(
					&(unc_pcidev_map[i])));
		}
		memset(&(unc_pcidev_map[i]), 0, sizeof(UNC_PCIDEV_NODE));
	}
}

/*!
 * @fn          static VOID UNC_COMMON_PCI_Scan_For_Uncore(VOID*)
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

VOID UNC_COMMON_PCI_Scan_For_Uncore(PVOID param, U32 dev_node,
					   DEVICE_CALLBACK callback)
{
	U32 device_id;
	U32 value;
	U32 vendor_id;
	U32 busno;
	U32 j, k, l;
	U32 device_found = 0;

	SEP_DRV_LOG_TRACE_IN("Dummy param: %p, dev_node: %u, callback: %p.",
			     param, dev_node, callback);

	for (busno = 0; busno < 256; busno++) {
		for (j = 0; j < MAX_PCI_DEVNO; j++) {
			if (!(UNCORE_TOPOLOGY_INFO_pcidev_valid(
				    &uncore_topology, dev_node, j))) {
				continue;
			}
			for (k = 0; k < MAX_PCI_FUNCNO; k++) {
				if (!(UNCORE_TOPOLOGY_INFO_pcidev_is_devno_funcno_valid(
					    &uncore_topology, dev_node, j,
					    k))) {
					continue;
				}
				device_found = 0;
				value = PCI_Read_U32_Valid(busno, j, k, 0, 0);
				CONTINUE_IF_NOT_GENUINE_INTEL_DEVICE(
					value, vendor_id, device_id);
				SEP_DRV_LOG_TRACE("Uncore device ID = 0x%x.",
						  device_id);

				for (l = 0;
				     l <
				     UNCORE_TOPOLOGY_INFO_num_deviceid_entries(
					     &uncore_topology, dev_node);
				     l++) {
					if (UNCORE_TOPOLOGY_INFO_deviceid(
						    &uncore_topology, dev_node,
						    l) == device_id) {
						device_found = 1;
						break;
					}
				}
				if (device_found) {
					if (UNC_COMMON_Add_Bus_Map(
						    device_id, dev_node,
						    busno) == OS_SUCCESS) {
						UNCORE_TOPOLOGY_INFO_pcidev_num_entries_found(
							&uncore_topology,
							dev_node, j, k)++;
						SEP_DRV_LOG_DETECTION(
							"Found device 0x%x at BDF(%x:%x:%x) [%u unit(s) so far].",
							device_id, busno, j, k,
							UNCORE_TOPOLOGY_INFO_pcidev_num_entries_found(
								&uncore_topology,
								dev_node, j,
								k));
					}
				}
			}
		}
	}

	SEP_DRV_LOG_TRACE_OUT("");
}

/*!
 * @fn          extern VOID UNC_COMMON_Get_Platform_Topology()
 *
 * @brief       This function will walk through the platform registers to retrieve information and calculate the bus no.
 *              Reads appropriate pci_config regs and populates the PLATFORM_TOPOLOGY_PROG_NODE structure with the reg value.
 *
 * @param       U32             dev_node - Device no.
 *
 * @return      None
 *
 * <I>Special Notes:</I>
 *                   device_num corresponds to Memory controller
 *                   func_num  corresponds to Channel number
 *                   reg_offset corresponds to dimm slot
 */
VOID UNC_COMMON_Get_Platform_Topology(U32 dev_node)
{
	U32 num_registers = 0;
	// U32 device_index = 0;
	U32 bus_num = 0;
	U32 i = 0;
	U32 func_num = 0;
	U32 num_pkgs = num_packages;
	U32 device_num = 0;
	U32 reg_offset = 0;
	U32 len = 0;
	U64 reg_value = 0;
	U32 device_value = 0;
	U64 reg_mask = 0;
	U32 vendor_id;
	U32 device_id;
	U32 valid;

	PLATFORM_TOPOLOGY_REG topology_regs = NULL;

	SEP_DRV_LOG_TRACE_IN("Dev_node: %u.", dev_node);
	PLATFORM_TOPOLOGY_PROG_topology_device_prog_valid(
		&platform_topology_prog_node, dev_node) = 1;

	if (num_packages > MAX_PACKAGES) {
		SEP_DRV_LOG_ERROR(
			"Num_packages %d > MAX_PACKAGE, getting for only %d packages.",
			num_packages, MAX_PACKAGES);
		num_pkgs = MAX_PACKAGES;
	}

	num_registers = PLATFORM_TOPOLOGY_PROG_topology_device_num_registers(
		&platform_topology_prog_node, dev_node);
	topology_regs = PLATFORM_TOPOLOGY_PROG_topology_topology_regs(
		&platform_topology_prog_node, dev_node);
	// device_index = PLATFORM_TOPOLOGY_PROG_topology_device_device_index(
		// &platform_topology_prog_node, dev_node);

	for (i = 0; i < num_pkgs; i++) {
		for (len = 0; len < num_registers; len++) {
			if (PLATFORM_TOPOLOGY_REG_reg_type(
				    topology_regs, len) == PMU_REG_PROG_MSR) {
				reg_value = SYS_Read_MSR(
					PLATFORM_TOPOLOGY_REG_reg_id(
						topology_regs, len));
				reg_mask = PLATFORM_TOPOLOGY_REG_reg_mask(
					topology_regs, len);
				PLATFORM_TOPOLOGY_REG_reg_value(topology_regs,
								len, i) =
					reg_value & reg_mask;
				SEP_DRV_LOG_TRACE(
					"Read UNCORE_MSR_FREQUENCY 0x%x\n",
					PLATFORM_TOPOLOGY_REG_reg_id(
						topology_regs, len));
			} else {
				if (!IS_BUS_MAP_VALID(dev_node, i)) {
					continue;
				}
				bus_num = GET_BUS_MAP(dev_node, i);
				device_num = PLATFORM_TOPOLOGY_REG_device(
					topology_regs, len);
				func_num = PLATFORM_TOPOLOGY_REG_function(
					topology_regs, len);
				reg_offset = PLATFORM_TOPOLOGY_REG_reg_id(
					topology_regs, len);
				device_value = PCI_Read_U32_Valid(
					bus_num, device_num, func_num, 0, 0);
				CHECK_IF_GENUINE_INTEL_DEVICE(device_value,
							      vendor_id,
							      device_id, valid);
				SEP_DRV_LOG_TRACE("Uncore device ID = 0x%x.",
						  device_id);
				if (!valid) {
					PLATFORM_TOPOLOGY_REG_device_valid(
						topology_regs, len) = 0;
				}
				PLATFORM_TOPOLOGY_REG_reg_value(topology_regs,
								len, i) =
					PCI_Read_U32_Valid(bus_num, device_num,
							   func_num, reg_offset,
							   PCI_INVALID_VALUE);
			}
		}
		if (PLATFORM_TOPOLOGY_PROG_topology_device_scope(
			    &platform_topology_prog_node, dev_node) ==
		    SYSTEM_EVENT) {
			break;
		}
	}
	SEP_DRV_LOG_TRACE_OUT("");
}

/************************************************************/
/*
 * UNC common MSR  based API
 *
 ************************************************************/

/*!
 * @fn         VOID UNC_COMMON_MSR_Clean_Up(PVOID)
 *
 * @brief      clear out out programming
 *
 * @param      None
 *
 * @return     None
 */
VOID UNC_COMMON_MSR_Clean_Up(VOID *param)
{
	U32 dev_idx;

	SEP_DRV_LOG_TRACE_IN("Param: %p.", param);
	dev_idx = *((U32 *)param);
	FOR_EACH_REG_ENTRY_UNC(pecb, dev_idx, i)
	{
		if (ECB_entries_clean_up_get(pecb, i)) {
			SYS_Write_MSR(ECB_entries_reg_id(pecb, i), 0LL);
		}
	}
	END_FOR_EACH_REG_ENTRY_UNC;

	SEP_DRV_LOG_TRACE_OUT("");
}
