/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef _ASM_KEYLOCKER_H
#define _ASM_KEYLOCKER_H

#ifndef __ASSEMBLY__

#include <linux/bits.h>

#define KL_CPUID                0x019
#define KL_CPUID_EAX_SUPERVISOR BIT(0)
#define KL_CPUID_EBX_AESKLE     BIT(0)
#define KL_CPUID_EBX_WIDE       BIT(2)
#define KL_CPUID_EBX_BACKUP     BIT(4)
#define KL_CPUID_ECX_RAND       BIT(1)

#endif /*__ASSEMBLY__ */
#endif /* _ASM_KEYLOCKER_H */
