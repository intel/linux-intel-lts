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

#include <asm/page.h>
#include <asm/io.h>

#include "lwpmudrv_defines.h"
#include "lwpmudrv_types.h"
#include "lwpmudrv_ecb.h"
#include "lwpmudrv_gfx.h"
#include "lwpmudrv.h"
#include "inc/pci.h"
#include "gfx.h"
#include "utility.h"

static char *gfx_virtual_addr;
static SEP_MMIO_NODE gfx_map;
static U32 gfx_code = GFX_CTRL_DISABLE;
static U32 gfx_counter[GFX_NUM_COUNTERS];
static U32 gfx_overflow[GFX_NUM_COUNTERS];

/*!
 * @fn     OS_STATUS GFX_Read
 *
 * @brief  Reads the counters into the buffer provided for the purpose
 *
 * @param  buffer  - buffer to read the counts into
 *
 * @return STATUS_SUCCESS if read succeeded, otherwise error
 *
 * @note
 */
OS_STATUS GFX_Read(S8 *buffer)
{
	U64 *samp = (U64 *)buffer;
	U32 i;
	U32 val;
	char *reg_addr;

	SEP_DRV_LOG_TRACE_IN("Buffer: %p.", buffer);

	// GFX counting was not specified
	if (gfx_virtual_addr == NULL || gfx_code == GFX_CTRL_DISABLE) {
		SEP_DRV_LOG_ERROR_TRACE_OUT(
			"OS_INVALID (!gfx_virtual_addr || gfx_code==GFX_CTRL_DISABLE)");
		return OS_INVALID;
	}

	// check for sampling buffer
	if (!samp) {
		SEP_DRV_LOG_ERROR_TRACE_OUT("OS_INVALID (!samp).");
		return OS_INVALID;
	}

	// set the GFX register address
	reg_addr = gfx_virtual_addr + GFX_PERF_REG;

	// for all counters - save the information to the sampling stream
	for (i = 0; i < GFX_NUM_COUNTERS; i++) {
		// read the ith GFX event count
		reg_addr += 4;
		val = *(U32 *)(reg_addr);
#if defined(GFX_COMPUTE_DELTAS)
		// if the current count is bigger than the previous one,
		// then the counter overflowed
		// so make sure the delta gets adjusted to account for it
		if (val < gfx_counter[i]) {
			samp[i] = val + (GFX_CTR_OVF_VAL - gfx_counter[i]);
		} else {
			samp[i] = val - gfx_counter[i];
		}
#else
		// just keep track of raw count for this counter
		// if the current count is bigger than the previous one,
		// then the counter overflowed
		if (val < gfx_counter[i]) {
			gfx_overflow[i]++;
		}
		samp[i] = val + (U64)gfx_overflow[i] * GFX_CTR_OVF_VAL;
#endif
		// save the current count
		gfx_counter[i] = val;
	}

	SEP_DRV_LOG_TRACE_OUT("OS_SUCCESS.");
	return OS_SUCCESS;
}

/*!
 * @fn     OS_STATUS GFX_Set_Event_Code
 *
 * @brief  Programs the Graphics PMU with the right event code
 *
 * @param  arg - buffer containing graphics event code
 *
 * @return STATUS_SUCCESS if success, otherwise error
 *
 * @note
 */
OS_STATUS GFX_Set_Event_Code(IOCTL_ARGS arg)
{
	U32 i;
	char *reg_addr;
	U32 reg_value;

	SEP_DRV_LOG_FLOW_IN("Arg: %p.", arg);

	// extract the graphics event code from usermode
	if (get_user(gfx_code, (int __user *)arg->buf_usr_to_drv)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"OS_FAULT (Unable to obtain gfx_code from usermode!).");
		return OS_FAULT;
	}
	SEP_DRV_LOG_TRACE("Got gfx_code=0x%x.", gfx_code);

	// memory map the address to GFX counters, if not already done
	if (gfx_virtual_addr == NULL) {
		PCI_Map_Memory(&gfx_map, GFX_BASE_ADDRESS + GFX_BASE_NEW_OFFSET,
			       PAGE_SIZE);
		gfx_virtual_addr =
			(char *)(UIOP)SEP_MMIO_NODE_virtual_address(&gfx_map);
	}

	// initialize the GFX counts
	for (i = 0; i < GFX_NUM_COUNTERS; i++) {
		gfx_counter[i] = 0;
		gfx_overflow[i] = 0;
		// only used if storing raw counts
		// (i.e., GFX_COMPUTE_DELTAS is undefined)
	}

	// get current GFX event code
	if (gfx_virtual_addr == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"OS_INVALID (Invalid gfx_virtual_addr=0x%p!).",
			gfx_virtual_addr);
		return OS_INVALID;
	}

	reg_addr = gfx_virtual_addr + GFX_PERF_REG;
	reg_value = *(U32 *)(reg_addr);
	SEP_DRV_LOG_TRACE("Read reg_value=0x%x from reg_addr=0x%p.", reg_value,
			reg_addr);

	/* Update the GFX counter group */
	// write the GFX counter group with reset = 1 for all counters
	reg_value = (gfx_code | GFX_REG_CTR_CTRL);
	*(U32 *)(reg_addr) = reg_value;
	SEP_DRV_LOG_TRACE("Wrote reg_value=0x%x to reg_addr=0x%p.", reg_value,
			reg_addr);

	SEP_DRV_LOG_FLOW_OUT("OS_SUCCESS.");
	return OS_SUCCESS;
}

/*!
 * @fn     OS_STATUS GFX_Start
 *
 * @brief  Starts the count of the Graphics PMU
 *
 * @param  NONE
 *
 * @return OS_SUCCESS if success, otherwise error
 *
 * @note
 */
OS_STATUS GFX_Start(void)
{
	U32 reg_value;
	char *reg_addr;

	SEP_DRV_LOG_TRACE_IN("");

	// GFX counting was not specified
	if (gfx_virtual_addr == NULL || gfx_code == GFX_CTRL_DISABLE) {
		SEP_DRV_LOG_ERROR(
			"Invalid gfx_virtual_addr=0x%p or gfx_code=0x%x.",
			gfx_virtual_addr, gfx_code);
		SEP_DRV_LOG_TRACE_OUT("OS_INVALID.");
		return OS_INVALID;
	}

	// turn on GFX counters as per event code
	reg_addr = gfx_virtual_addr + GFX_PERF_REG;
	*(U32 *)(reg_addr) = gfx_code;

	// verify event code was written properly
	reg_value = *(U32 *)reg_addr;
	if (reg_value != gfx_code) {
		SEP_DRV_LOG_ERROR("Got register value 0x%x, expected 0x%x.",
				  reg_value, gfx_code);
		SEP_DRV_LOG_TRACE_OUT("OS_INVALID.");
		return OS_INVALID;
	}

	SEP_DRV_LOG_TRACE_OUT("OS_SUCCESS.");
	return OS_SUCCESS;
}

/*!
 * @fn     OS_STATUS GFX_Stop
 *
 * @brief  Stops the count of the Graphics PMU
 *
 * @param  NONE
 *
 * @return OS_SUCCESS if success, otherwise error
 *
 * @note
 */
OS_STATUS GFX_Stop(void)
{
	char *reg_addr;

	SEP_DRV_LOG_TRACE_IN("");

	// GFX counting was not specified
	if (gfx_virtual_addr == NULL || gfx_code == GFX_CTRL_DISABLE) {
		SEP_DRV_LOG_ERROR(
			"Invalid gfx_virtual_addr=0x%p or gfx_code=0x%x.",
			gfx_virtual_addr, gfx_code);
		SEP_DRV_LOG_TRACE_OUT("OS_INVALID.");
		return OS_INVALID;
	}

	// turn off GFX counters
	reg_addr = gfx_virtual_addr + GFX_PERF_REG;
	*(U32 *)(reg_addr) = GFX_CTRL_DISABLE;

	// unmap the memory mapped virtual address
	PCI_Unmap_Memory(&gfx_map);
	gfx_virtual_addr = NULL;

	// reset the GFX global variables
	gfx_code = GFX_CTRL_DISABLE;

	SEP_DRV_LOG_TRACE_OUT("OS_SUCCESS.");
	return OS_SUCCESS;
}
