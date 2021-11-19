/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_ASM_X86_MMAN_H
#define _UAPI_ASM_X86_MMAN_H

#define MAP_32BIT	0x40		/* only give out 32bit addresses */

#define PROT_SHADOW_STACK	0x10	/* shadow stack pages */

#include <asm-generic/mman.h>

#endif /* _UAPI_ASM_X86_MMAN_H */
