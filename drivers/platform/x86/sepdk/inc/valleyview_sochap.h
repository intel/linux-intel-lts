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

#ifndef _VALLEYVIEW_SOCHAP_H_INC_
#define _VALLEYVIEW_SOCHAP_H_INC_

/*
 * Local to this architecture: Valleyview uncore SA unit
 *
 */
#define VLV_VISA_DESKTOP_DID 0x000C04
#define VLV_VISA_NEXT_ADDR_OFFSET 4
#define VLV_VISA_BAR_ADDR_SHIFT 32
#define VLV_VISA_BAR_ADDR_MASK 0x000FFFC00000LL
#define VLV_VISA_MAX_PCI_DEVICES 16
#define VLV_VISA_MCR_REG_OFFSET 0xD0
#define VLV_VISA_MDR_REG_OFFSET 0xD4
#define VLV_VISA_MCRX_REG_OFFSET 0xD8
#define VLV_VISA_BYTE_ENABLES 0xF
#define VLV_VISA_OP_CODE_SHIFT 24
#define VLV_VISA_PORT_ID_SHIFT 16
#define VLV_VISA_OFFSET_HI_MASK 0xFF
#define VLV_VISA_OFFSET_LO_MASK 0xFF
#define VLV_CHAP_SIDEBAND_PORT_ID 23
#define VLV_CHAP_SIDEBAND_WRITE_OP_CODE 1
#define VLV_CHAP_SIDEBAND_READ_OP_CODE 0
#define VLV_CHAP_MAX_COUNTERS 8
#define VLV_CHAP_MAX_COUNT 0x00000000FFFFFFFFLL

#define VLV_VISA_OTHER_BAR_MMIO_PAGE_SIZE 4096
#define VLV_VISA_CHAP_SAMPLE_DATA 0x00020000
#define VLV_VISA_CHAP_STOP 0x00040000
#define VLV_VISA_CHAP_START 0x00110000
#define VLV_VISA_CHAP_CTRL_REG_OFFSET 0x0

extern DISPATCH_NODE valleyview_visa_dispatch;

#endif
