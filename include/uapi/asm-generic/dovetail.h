/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef __ASM_GENERIC_DOVETAIL_H
#define __ASM_GENERIC_DOVETAIL_H

/*
 * Legacy out-of-band syscall marker. Will be dropped at some point.
 */
#define __OOB_SYSCALL_BIT	0x10000000

/* Dovetail out-of-band syscall via prctl(2) */
#define PR_OOB_SYSCALL		0x4f4f4243

#endif /* !__ASM_GENERIC_DOVETAIL_H */
