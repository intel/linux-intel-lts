/*
 * Copyright (c) 2017 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef INTEL_IPU5_FPGA_H
#define INTEL_IPU5_FPGA_H

#include "../intel-ipu5-buttress-regs.h"

#define IPU5_BAR_FOR_BRIDGE 1

#define IPU5_FPGA_PCI_BRIDGE_RESET_START 0x9200000
#define IPU5_FPGA_PCI_BRIDGE_RESET_END \
	(IPU5_FPGA_PCI_BRIDGE_RESET_START + PAGE_SIZE - 1)

#define IPU5_FPGA_RESET_REG 0x20
#define IPU5_FPGA_RESET_ACTIVE 0x0
#define IPU5_FPGA_RESET_RELEASE 0x1

#define IPU5_BTR_IS_ON 0x80000006
#define IPU5_BTR_PS_ON 0x80070880

#endif

