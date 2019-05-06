/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (C) 2019 Philippe Gerum.
 */
#ifndef _ASM_X86_DOVETAIL_H
#define _ASM_X86_DOVETAIL_H

#if !defined(__ASSEMBLY__) && defined(CONFIG_DOVETAIL)

static inline void arch_dovetail_exec_prepare(void)
{ }

static inline
void arch_dovetail_switch_prepare(bool leave_inband)
{ }

static inline
void arch_dovetail_switch_finish(bool enter_inband)
{ }

#endif

#endif /* _ASM_X86_DOVETAIL_H */
