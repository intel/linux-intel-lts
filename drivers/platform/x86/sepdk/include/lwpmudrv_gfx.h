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

#ifndef _LWPMUDRV_GFX_H_
#define _LWPMUDRV_GFX_H_

#if defined(__cplusplus)
extern "C" {
#endif

#define GFX_BASE_ADDRESS 0xFF200000
#define GFX_BASE_NEW_OFFSET 0x00080000
#define GFX_PERF_REG 0x040 // location of GFX counter relative to base
#define GFX_NUM_COUNTERS 9 // max number of GFX counters per counter group
#define GFX_CTR_OVF_VAL 0xFFFFFFFF // overflow value for GFX counters

#define GFX_REG_CTR_CTRL 0x01FF
#define GFX_CTRL_DISABLE 0x1E00


#if defined(__cplusplus)
}
#endif

#endif
