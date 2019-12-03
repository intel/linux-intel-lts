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

#ifndef _HSWUNC_SA_H_INC_
#define _HSWUNC_SA_H_INC_

/*
 * Local to this architecture: Haswell uncore SA unit
 *
 */
#define HSWUNC_SA_DESKTOP_DID               0x000C04
#define HSWUNC_SA_NEXT_ADDR_OFFSET          4
#define HSWUNC_SA_BAR_ADDR_SHIFT            32
#define HSWUNC_SA_BAR_ADDR_MASK             0x0007FFFFFF000LL
#define HSWUNC_SA_MAX_PCI_DEVICES           16
#define HSWUNC_SA_MAX_COUNT                 0x00000000FFFFFFFFLL
#define HSWUNC_SA_MAX_COUNTERS              8

#define HSWUNC_SA_MCHBAR_MMIO_PAGE_SIZE     (8 * 4096)
#define HSWUNC_SA_PCIEXBAR_MMIO_PAGE_SIZE   (57 * 4096)
#define HSWUNC_SA_OTHER_BAR_MMIO_PAGE_SIZE  4096
#define HSWUNC_SA_GDXCBAR_OFFSET_LO         0x5420
#define HSWUNC_SA_GDXCBAR_OFFSET_HI         0x5424
#define HSWUNC_SA_GDXCBAR_MASK              0x7FFFFFF000LL
#define HSWUNC_SA_CHAP_SAMPLE_DATA          0x00020000
#define HSWUNC_SA_CHAP_STOP                 0x00040000
#define HSWUNC_SA_CHAP_CTRL_REG_OFFSET      0x0

#define HSWUNC_SA_PAGE_MASK                 0xfffffffffffff000
#define HSWUNC_SA_PAGE_OFFSET_MASK          0xfff
#define HSWUNC_SA_PAGE_SIZE                 0x1000

extern DISPATCH_NODE hswunc_sa_dispatch;

#endif
