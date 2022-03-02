/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef _ASM_KEYLOCKER_H
#define _ASM_KEYLOCKER_H

#ifndef __ASSEMBLY__

#include <linux/bits.h>

#include <asm/msr.h>

#define KL_CPUID                0x019
#define KL_CPUID_EAX_SUPERVISOR BIT(0)
#define KL_CPUID_EBX_AESKLE     BIT(0)
#define KL_CPUID_EBX_WIDE       BIT(2)
#define KL_CPUID_EBX_BACKUP     BIT(4)
#define KL_CPUID_ECX_RAND       BIT(1)

bool check_keylocker_readiness(void);

static inline u64 read_iwkeybackup_status(void)
{
	u64 status;

	rdmsrl(MSR_IA32_IWKEYBACKUP_STATUS, status);
	return status;
}

void backup_iwkey(void);
bool load_iwkey(void);
bool copy_iwkey(void);

void make_iwkeydata(bool use_hwrandom);
#ifdef CONFIG_X86_KL
void invalidate_iwkeydata(void);
#else
#define invalidate_iwkeydata() do { } while (0)
#endif

#endif /*__ASSEMBLY__ */
#endif /* _ASM_KEYLOCKER_H */
