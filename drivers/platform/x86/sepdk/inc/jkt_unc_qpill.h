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


#ifndef _JKTUNC_QPILL_H_INC_
#define _JKTUNC_QPILL_H_INC_

/*
 * Local to this architecture: JKT uncore QPILL unit
 *
 */
#define JKTUNC_QPILL0_DID       0x3C41
	// --- QPILL0 PerfMon DID --- B:D 1:8:2
#define JKTUNC_QPILL_MM0_DID    0x3C86
	// --- QPILL0 PerfMon MM Config DID --- B:D 1:8:6
#define JKTUNC_QPILL1_DID       0x3C42
	// --- QPILL1 PerfMon DID --- B:D 1:9:2
#define JKTUNC_QPILL2_DID       0x3C44
	// --- QPILL0 PerfMon DID --- B:D 1:8:2
#define JKTUNC_QPILL3_DID       0x3C45
	// --- QPILL0 PerfMon DID --- B:D 1:8:2
#define JKTUNC_QPILL_MM1_DID    0x3C96
	// --- QPILL1 PerfMon MM Config DID --- B:D 1:9:6
#define JKTUNC_QPILL_MCFG_DID   0x3C28
	// --- QPILL1 PerfMon MCFG DID --- B:D 0:5:0
#define JKTUNC_QPILL0_D2C_DID   0x3C80
	// --- D2C QPILL Port 1 config DID B:D:F X:8:0
#define JKTUNC_QPILL1_D2C_DID   0x3C90
	// --- D2C QPILL Port 2 config DID B:D:F X:9:0

#define JKTUNC_QPILL_PERF_GLOBAL_CTRL 0x391

#define IA32_DEBUG_CTRL             0x1D9

#define JKTUNC_QPILL_D2C_OFFSET     0x80
#define JKTUNC_QPILL_D2C_BITMASK    0x00000002
#define JKTUNC_QPILL_FUNC_NO        2
#define JKTUNC_QPILL_D2C_FUNC_NO    0

extern DISPATCH_NODE jktunc_qpill_dispatch;

#endif
