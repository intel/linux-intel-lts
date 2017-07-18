/**
 * Support for Intel Camera Imaging ISP subsystem.
* Copyright (c) 2010 - 2017, Intel Corporation.
*
* This program is free software; you can redistribute it and/or modify it
* under the terms and conditions of the GNU General Public License,
* version 2, as published by the Free Software Foundation.
*
* This program is distributed in the hope it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
 */

#ifndef __IPU_DEVICE_CELL_PROPERTIES_IMPL_H
#define __IPU_DEVICE_CELL_PROPERTIES_IMPL_H

#include "ipu_device_sp2600_control_properties_impl.h"
#include "ipu_device_sp2600_proxy_properties_impl.h"
#include "ipu_device_idsp_properties_impl.h"
#include "ipu_device_cell_properties_defs.h"
#include "ipu_device_cell_devices.h"

static const unsigned int
ipu_device_spc0_mem_address[IPU_DEVICE_SP2600_CONTROL_NUM_MEMORIES] = {
	SPC0_REGS_CBUS_ADDRESS,
	IPU_DEVICE_INVALID_MEM_ADDRESS, /* no pmem */
	SPC0_DMEM_CBUS_ADDRESS
};

static const unsigned int
ipu_device_spp0_mem_address[IPU_DEVICE_SP2600_PROXY_NUM_MEMORIES] = {
	SPP0_REGS_CBUS_ADDRESS,
	IPU_DEVICE_INVALID_MEM_ADDRESS, /* no pmem */
	SPP0_DMEM_CBUS_ADDRESS
};

static const unsigned int
ipu_device_spp1_mem_address[IPU_DEVICE_SP2600_PROXY_NUM_MEMORIES] = {
	SPP1_REGS_CBUS_ADDRESS,
	IPU_DEVICE_INVALID_MEM_ADDRESS, /* no pmem */
	SPP1_DMEM_CBUS_ADDRESS
};

static const unsigned int
ipu_device_isp0_mem_address[IPU_DEVICE_IDSP_NUM_MEMORIES] = {
	ISP0_REGS_CBUS_ADDRESS, /* reg addr */
	ISP0_PMEM_CBUS_ADDRESS, /* pmem addr */
	ISP0_DMEM_CBUS_ADDRESS, /* dmem addr */
	ISP0_BMEM_CBUS_ADDRESS, /* bamem addr */
	ISP0_VMEM_CBUS_ADDRESS  /* vmem addr */
};

static const unsigned int
ipu_device_isp1_mem_address[IPU_DEVICE_IDSP_NUM_MEMORIES] = {
	ISP1_REGS_CBUS_ADDRESS, /* reg addr */
	ISP1_PMEM_CBUS_ADDRESS, /* pmem addr */
	ISP1_DMEM_CBUS_ADDRESS, /* dmem addr */
	ISP1_BMEM_CBUS_ADDRESS, /* bamem addr */
	ISP1_VMEM_CBUS_ADDRESS  /* vmem addr */
};

static const unsigned int
ipu_device_isp2_mem_address[IPU_DEVICE_IDSP_NUM_MEMORIES] = {
	ISP2_REGS_CBUS_ADDRESS, /* reg addr */
	ISP2_PMEM_CBUS_ADDRESS, /* pmem addr */
	ISP2_DMEM_CBUS_ADDRESS, /* dmem addr */
	ISP2_BMEM_CBUS_ADDRESS, /* bamem addr */
	ISP2_VMEM_CBUS_ADDRESS  /* vmem addr */
};

static const unsigned int
ipu_device_spc0_mem_databus_address[IPU_DEVICE_SP2600_CONTROL_NUM_MEMORIES] = {
	IPU_DEVICE_INVALID_MEM_ADDRESS, /* no reg addr */
	IPU_DEVICE_INVALID_MEM_ADDRESS, /* no pmem */
	SPC0_DMEM_DBUS_ADDRESS
};

static const unsigned int
ipu_device_spp0_mem_databus_address[IPU_DEVICE_SP2600_PROXY_NUM_MEMORIES] = {
	IPU_DEVICE_INVALID_MEM_ADDRESS, /* no reg addr */
	IPU_DEVICE_INVALID_MEM_ADDRESS, /* no pmem */
	SPP0_DMEM_DBUS_ADDRESS
};

static const unsigned int
ipu_device_spp1_mem_databus_address[IPU_DEVICE_SP2600_PROXY_NUM_MEMORIES] = {
	IPU_DEVICE_INVALID_MEM_ADDRESS, /* no reg addr */
	IPU_DEVICE_INVALID_MEM_ADDRESS, /* no pmem */
	SPP1_DMEM_DBUS_ADDRESS
};

static const unsigned int
ipu_device_isp0_mem_databus_address[IPU_DEVICE_IDSP_NUM_MEMORIES] = {
	IPU_DEVICE_INVALID_MEM_ADDRESS, /* no reg addr */
	ISP0_PMEM_DBUS_ADDRESS,         /* pmem addr */
	ISP0_DMEM_DBUS_ADDRESS,         /* dmem addr */
	ISP0_BMEM_DBUS_ADDRESS,         /* bamem addr */
	ISP0_VMEM_DBUS_ADDRESS          /* vmem addr */
};

static const unsigned int
ipu_device_isp1_mem_databus_address[IPU_DEVICE_IDSP_NUM_MEMORIES] = {
	IPU_DEVICE_INVALID_MEM_ADDRESS, /* no reg addr */
	ISP1_PMEM_DBUS_ADDRESS,         /* pmem addr */
	ISP1_DMEM_DBUS_ADDRESS,         /* dmem addr */
	ISP1_BMEM_DBUS_ADDRESS,         /* bamem addr */
	ISP1_VMEM_DBUS_ADDRESS          /* vmem addr */
};

static const unsigned int
ipu_device_isp2_mem_databus_address[IPU_DEVICE_IDSP_NUM_MEMORIES] = {
	IPU_DEVICE_INVALID_MEM_ADDRESS, /* no reg addr */
	ISP2_PMEM_DBUS_ADDRESS,         /* pmem addr */
	ISP2_DMEM_DBUS_ADDRESS,         /* dmem addr */
	ISP2_BMEM_DBUS_ADDRESS,         /* bamem addr */
	ISP2_VMEM_DBUS_ADDRESS          /* vmem addr */
};

static const struct ipu_device_cell_properties_s
ipu_device_cell_properties[NUM_CELLS] = {
	{
		&ipu_device_sp2600_control_properties,
		ipu_device_spc0_mem_address,
		ipu_device_spc0_mem_databus_address
	},
	{
		&ipu_device_sp2600_proxy_properties,
		ipu_device_spp0_mem_address,
		ipu_device_spp0_mem_databus_address
	},
	{
		&ipu_device_sp2600_proxy_properties,
		ipu_device_spp1_mem_address,
		ipu_device_spp1_mem_databus_address
	},
	{
		&ipu_device_idsp_properties,
		ipu_device_isp0_mem_address,
		ipu_device_isp0_mem_databus_address
	},
	{
		&ipu_device_idsp_properties,
		ipu_device_isp1_mem_address,
		ipu_device_isp1_mem_databus_address
	},
	{
		&ipu_device_idsp_properties,
		ipu_device_isp2_mem_address,
		ipu_device_isp2_mem_databus_address
	}
};

#ifdef C_RUN

/* Mapping between hrt_hive_processors enum and cell_id's used in FW
 * The mapping of CRUN cell ids can be checked in SDK at
 * systems/ipu_system/hrt/include/ipu_system_hrtx.h
 */
static const int ipu_device_map_cell_id_to_crun_proc_id[NUM_CELLS] = {
	4, /* SPC0 */
	5, /* SPP0 */
	6, /* SPP1 */
	0, /* ISP0 */
	1, /* ISP1 */
	2  /* ISP2 */
};

#endif

#endif /* __IPU_DEVICE_CELL_PROPERTIES_IMPL_H */

