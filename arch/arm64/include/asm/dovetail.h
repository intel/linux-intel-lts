/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (C) 2018 Philippe Gerum  <rpm@xenomai.org>.
 */
#ifndef _ASM_ARM64_DOVETAIL_H
#define _ASM_ARM64_DOVETAIL_H

#include <asm/fpsimd.h>

/* ARM64 traps */
#define ARM64_TRAP_ACCESS	0	/* Data or instruction access exception */
#define ARM64_TRAP_ALIGN	1	/* SP/PC alignment abort */
#define ARM64_TRAP_SEA		2	/* Synchronous external abort */
#define ARM64_TRAP_DEBUG	3	/* Debug trap */
#define ARM64_TRAP_UNDI		4	/* Undefined instruction */
#define ARM64_TRAP_UNDSE	5	/* Undefined synchronous exception */
#define ARM64_TRAP_FPE		6	/* FPSIMD exception */
#define ARM64_TRAP_SVE		7	/* SVE access trap */
#define ARM64_TRAP_BTI		8	/* Branch target identification */

#ifdef CONFIG_DOVETAIL

static inline void arch_dovetail_exec_prepare(void)
{ }

static inline void arch_dovetail_switch_prepare(bool leave_inband)
{ }

static inline void arch_dovetail_switch_finish(bool enter_inband)
{
	fpsimd_restore_current_oob();
}

/*
 * 172 is __NR_prctl from unistd32 in ARM32 mode, without #inclusion
 * hell. At the end of the day, this number is written in stone to
 * honor the ABI stability promise anyway.
 */
#define arch_dovetail_is_syscall(__nr)	\
	((is_compat_task() && (__nr) == 172) || __nr == __NR_prctl)

#endif

#endif /* _ASM_ARM64_DOVETAIL_H */
