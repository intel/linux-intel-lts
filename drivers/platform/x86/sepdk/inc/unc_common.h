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

#ifndef _UNC_COMMON_H_INC_
#define _UNC_COMMON_H_INC_

#include "pci.h"

#define DRV_IS_PCI_VENDOR_ID_INTEL 0x8086
#define VENDOR_ID_MASK 0x0000FFFF
#define DEVICE_ID_MASK 0xFFFF0000
#define DEVICE_ID_BITSHIFT 16

#define UNCORE_SOCKETID_UBOX_LNID_OFFSET 0x40
#define UNCORE_SOCKETID_UBOX_GID_OFFSET 0x54

#define INVALID_BUS_NUMBER -1
#define PCI_INVALID_VALUE 0xFFFFFFFF

typedef struct DEVICE_CALLBACK_NODE_S DEVICE_CALLBACK_NODE;
typedef DEVICE_CALLBACK_NODE * DEVICE_CALLBACK;

struct DEVICE_CALLBACK_NODE_S {
	DRV_BOOL (*is_Valid_Device)(U32);
	DRV_BOOL (*is_Valid_For_Write)(U32, U32);
	DRV_BOOL (*is_Unit_Ctl)(U32);
	DRV_BOOL (*is_PMON_Ctl)(U32);
};

#define MAX_PCIDEV_UNITS 16
#define GET_MAX_PCIDEV_ENTRIES(num_pkg)                                        \
	((num_pkg > MAX_PCIDEV_UNITS) ? num_pkg : MAX_PCIDEV_UNITS)

typedef struct UNC_PCIDEV_NODE_S UNC_PCIDEV_NODE;

struct UNC_PCIDEV_NODE_S {
	U32 num_entries;
	U32 max_entries;
	S32 *busno_list; // array for pcibus mapping
	SEP_MMIO_NODE *mmio_map; // virtual memory mapping entries
};

#define UNC_PCIDEV_max_entries(x) ((x)->max_entries)
#define UNC_PCIDEV_num_entries(x) ((x)->num_entries)
#define UNC_PCIDEV_busno_list(x) ((x)->busno_list)
#define UNC_PCIDEV_busno_entry(x, entry) ((x)->busno_list[entry])
#define UNC_PCIDEV_mmio_map(x) ((x)->mmio_map)
#define UNC_PCIDEV_mmio_map_entry(x, entry) ((x)->mmio_map[entry])
#define UNC_PCIDEV_virtual_addr_entry(x, entry)                                \
	(SEP_MMIO_NODE_virtual_address(&UNC_PCIDEV_mmio_map_entry(x, entry)))

#define UNC_PCIDEV_is_busno_valid(x, entry)                                    \
	(((x)->busno_list) && ((x)->num_entries > (entry)) &&                  \
	((x)->busno_list[(entry)] != INVALID_BUS_NUMBER))
#define UNC_PCIDEV_is_vaddr_valid(x, entry)                                    \
	(((x)->mmio_map) && ((x)->num_entries > (entry)) &&                    \
	((x)->mmio_map[(entry)].virtual_address))

extern UNC_PCIDEV_NODE unc_pcidev_map[];

#define GET_BUS_MAP(dev_node, entry)                                           \
	(UNC_PCIDEV_busno_entry((&(unc_pcidev_map[dev_node])), entry))
#define GET_NUM_MAP_ENTRIES(dev_node)                                          \
	(UNC_PCIDEV_num_entries(&(unc_pcidev_map[dev_node])))
#define IS_MMIO_MAP_VALID(dev_node, entry)                                     \
	(UNC_PCIDEV_is_vaddr_valid((&(unc_pcidev_map[dev_node])), entry))
#define IS_BUS_MAP_VALID(dev_node, entry)                                      \
	(UNC_PCIDEV_is_busno_valid((&(unc_pcidev_map[dev_node])), entry))
#define virtual_address_table(dev_node, entry)                                 \
	(UNC_PCIDEV_virtual_addr_entry(&(unc_pcidev_map[dev_node]), entry))

extern OS_STATUS UNC_COMMON_Do_Bus_to_Socket_Map(U32 uncore_did, U32 dev_node,
						U32 bus_no, U32 device_no,
						U32 function_no);

extern VOID UNC_COMMON_Dummy_Func(PVOID param);

extern VOID UNC_COMMON_Read_Counts(PVOID param, U32 id);

/************************************************************/
/*
 * UNC common PCI  based API
 *
 ************************************************************/

extern VOID UNC_COMMON_PCI_Write_PMU(PVOID param, U32 ubox_did, U32 control_msr,
				U32 ctl_val, U32 pci_dev_index,
				DEVICE_CALLBACK callback);

extern VOID UNC_COMMON_PCI_Enable_PMU(PVOID param, U32 control_msr,
				U32 enable_val, U32 disable_val,
				DEVICE_CALLBACK callback);

extern VOID UNC_COMMON_PCI_Disable_PMU(PVOID param, U32 control_msr,
				U32 enable_val, U32 disable_val,
				DEVICE_CALLBACK callback);

extern OS_STATUS UNC_COMMON_Add_Bus_Map(U32 uncore_did, U32 dev_node,
					U32 bus_no);

extern OS_STATUS UNC_COMMON_Init(void);

extern VOID UNC_COMMON_Clean_Up(void);

extern VOID UNC_COMMON_PCI_Trigger_Read(U32 id);

extern VOID UNC_COMMON_PCI_Read_PMU_Data(PVOID param);

extern VOID UNC_COMMON_PCI_Scan_For_Uncore(PVOID param, U32 dev_info_node,
					DEVICE_CALLBACK callback);

extern VOID UNC_COMMON_Get_Platform_Topology(U32 dev_info_node);

/************************************************************/
/*
 * UNC common MSR  based API
 *
 ************************************************************/

extern VOID UNC_COMMON_MSR_Write_PMU(PVOID param, U32 control_msr,
				U64 control_val, U64 reset_val,
				DEVICE_CALLBACK callback);

extern VOID UNC_COMMON_MSR_Enable_PMU(PVOID param, U32 control_msr,
				U64 control_val, U64 unit_ctl_val,
				U64 pmon_ctl_val,
				DEVICE_CALLBACK callback);

extern VOID UNC_COMMON_MSR_Disable_PMU(PVOID param, U32 control_msr,
				U64 unit_ctl_val, U64 pmon_ctl_val,
				DEVICE_CALLBACK callback);

extern VOID UNC_COMMON_MSR_Trigger_Read(U32 id);

extern VOID UNC_COMMON_MSR_Read_PMU_Data(PVOID param);

extern VOID UNC_COMMON_MSR_Clean_Up(PVOID param);

#endif
