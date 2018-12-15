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

#ifndef _PMI_H_
#define _PMI_H_

#include "lwpmudrv_defines.h"
#include <linux/ptrace.h>
#include <linux/version.h>

#if defined(DRV_IA32)
#if KERNEL_VERSION(2, 6, 25) > LINUX_VERSION_CODE
#define REGS_xcs(regs) (regs->xcs)
#define REGS_eip(regs) (regs->eip)
#define REGS_eflags(regs) (regs->eflags)
#else
#define REGS_xcs(regs) (regs->cs)
#define REGS_eip(regs) (regs->ip)
#define REGS_eflags(regs) (regs->flags)
#endif
#endif

#if defined(DRV_EM64T)
#define REGS_cs(regs) (regs->cs)

#if KERNEL_VERSION(2, 6, 25) > LINUX_VERSION_CODE
#define REGS_rip(regs) (regs->rip)
#define REGS_eflags(regs) (regs->eflags)
#else
#define REGS_rip(regs) (regs->ip)
#define REGS_eflags(regs) (regs->flags)
#endif
#endif

asmlinkage VOID PMI_Interrupt_Handler(struct pt_regs *regs);

#if defined(DRV_SEP_ACRN_ON)
extern VOID PMI_Buffer_Handler(PVOID);
#endif

extern U32 pmi_Get_CSD(U32, U32 *, U32 *);

#endif
