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

#ifndef _UNC_GT_H_INC_
#define _UNC_GT_H_INC_

/*
 * Local to this architecture: SNB uncore GT unit
 *
 */
#define GT_MMIO_SIZE 0x200000
#define NEXT_ADDR_OFFSET 4
#define UNC_GT_BAR_MASK 0xFFF00000
#define PERF_GLOBAL_CTRL 0x391
#define GT_CLEAR_COUNTERS 0xFFFF0000

#define IA32_DEBUG_CTRL 0x1D9
#define MAX_FREE_RUNNING_EVENTS 6
#define GT_DID_1 0x102
#define INTEL_VENDOR_ID 0x8086
#define DRV_GET_PCI_VENDOR_ID(value) (value & 0x0000FFFF)
#define DRV_GET_PCI_DEVICE_ID(value) ((value & 0xFFFF0000) >> 16)
#define DRV_IS_INTEL_VENDOR_ID(value) (value == INTEL_VENDOR_ID)
#define DRV_IS_GT_DEVICE_ID(value) (value == GT_DID_1)

//clock gating disable values
#define UNC_GT_GCPUNIT_REG1 0x9400
#define UNC_GT_GCPUNIT_REG2 0x9404
#define UNC_GT_GCPUNIT_REG3 0x9408
#define UNC_GT_GCPUNIT_REG4 0x940c
#define UNC_GT_GCPUNIT_REG1_VALUE 0xffffffff
#define UNC_GT_GCPUNIT_REG2_VALUE 0xffffffff
#define UNC_GT_GCPUNIT_REG3_VALUE 0xffe3ffff
#define UNC_GT_GCPUNIT_REG4_VALUE 0x00000003
//RC6 disable
#define UNC_GT_RC6_REG1 0xa090
#define UNC_GT_RC6_REG2 0xa094
#define UNC_GT_RC6_REG1_OR_VALUE 0x80000000
#define UNC_GT_RC6_REG2_VALUE 0x00000000
extern DISPATCH_NODE unc_gt_dispatch;

typedef struct GT_CTR_NODE_S GT_CTR_NODE;
typedef GT_CTR_NODE * GT_CTR;
struct GT_CTR_NODE_S {
	union {
		struct {
			U32 low : 32;
			U32 high : 12;
		} bits;
		U64 value;
	} u;
};

#define GT_CTR_NODE_value(x) (x.u.value)
#define GT_CTR_NODE_low(x) (x.u.bits.low)
#define GT_CTR_NODE_high(x) (x.u.bits.high)
#define GT_CTR_NODE_value_reset(x) x.u.value = 0

#define DRV_WRITE_PCI_REG_ULONG(va, offset_delta, value)                       \
	writel(value, (void __iomem *)((char *)(va + offset_delta)))
#define DRV_READ_PCI_REG_ULONG(va, offset_delta)                               \
	readl((void __iomem *)(char *)(va + offset_delta))

#endif
