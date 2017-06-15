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
#ifndef INTEL_IPU_FPGA_H
#define INTEL_IPU_FPGA_H

#include "../intel-ipu4.h"
#include "../intel-ipu4-buttress.h"
#include "../intel-ipu4-buttress-regs.h"

#include "intel-ipu5-fpga.h"

#define BUTTRESS_REG_SECURE_TOUCH	0x318
#define BUTTRESS_SECURE_TOUCH_SECURE_TOUCH_SHIFT	31

#define BUTTRESS_POWER_TIMEOUT                 200

#endif
