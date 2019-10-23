// SPDX-License-Identifier: GPL-2.0-only

/*
 * Key Locker feature check and support Internal Wrapping Key (IWKey)
 */

#include <linux/random.h>
#include <linux/bits.h>
#include <linux/acpi.h>
#include <linux/delay.h>

#include <asm/keylocker.h>
#include <asm/fpu/types.h>

static bool iwkeybackup_available;

bool check_keylocker_readiness(void)
{
	u32 eax, ebx, ecx, edx;

	cpuid_count(KL_CPUID, 0, &eax, &ebx, &ecx, &edx);
	/* BIOS may not enable it on some systems. */
	if (!(ebx & KL_CPUID_EBX_AESKLE)) {
		pr_debug("x86/keylocker: not fully enabled\n");
		return false;
	}

	iwkeybackup_available = (ebx & KL_CPUID_EBX_BACKUP);
	/* IWKey backup is essential with S3/4 states */
	if (!iwkeybackup_available &&
	    (acpi_sleep_state_supported(ACPI_STATE_S3) ||
	     acpi_sleep_state_supported(ACPI_STATE_S4))) {
		pr_debug("x86/keylocker: no key backup support with possible S3/4\n");
		return false;
	}
	return true;
}

#define LOADIWKEY		".byte 0xf3,0x0f,0x38,0xdc,0xd1"
#define LOADIWKEY_NUM_OPERANDS	3

static struct iwkey {
	bool valid;
	struct reg_128_bit value[LOADIWKEY_NUM_OPERANDS];
} iwkeydata;

void make_iwkeydata(void)
{
	int i;

	for (i = 0; i < LOADIWKEY_NUM_OPERANDS; i++)
		get_random_bytes(&iwkeydata.value[i], sizeof(struct reg_128_bit));

	iwkeydata.valid = true;
}

void invalidate_iwkeydata(void)
{
	if (!iwkeydata.valid)
		return;

	memset(&iwkeydata, 0, sizeof(iwkeydata));
}

#define IWKEY_SW_PROVIDED	0

bool load_iwkey(void)
{
	u32 keysource = IWKEY_SW_PROVIDED;
	struct reg_128_bit zeros = { 0 };
	bool err = true;

	if (!iwkeydata.valid)
		return false;

	asm ("movdqu %0, %%xmm0; movdqu %1, %%xmm1; movdqu %2, %%xmm2;"
	     :: "m"(iwkeydata.value[0]), "m"(iwkeydata.value[1]), "m"(iwkeydata.value[2]));

	asm volatile (LOADIWKEY CC_SET(z)
		      : CC_OUT(z) (err)
		      : "a"(keysource));

	asm ("movdqu %0, %%xmm0; movdqu %0, %%xmm1; movdqu %0, %%xmm2;"
	     :: "m"(zeros));

	return err ? false : true;
}

void backup_iwkey(void)
{
	if (iwkeybackup_available)
		wrmsrl(MSR_IA32_COPY_LOCAL_TO_PLATFORM, 1);
}

#define IWKEY_RESTORE_RETRY	1

bool copy_iwkey(void)
{
	bool copied = false;
	int i;

	/* Use valid key data when available */
	if (iwkeydata.valid)
		return load_iwkey();

	if (!iwkeybackup_available)
		return copied;

	wrmsrl(MSR_IA32_COPY_PLATFORM_TO_LOCAL, 1);

	for (i = 0; (i <= IWKEY_RESTORE_RETRY) && !copied; i++) {
		u64 status;

		if (i)
			udelay(1);
		rdmsrl(MSR_IA32_COPY_STATUS, status);
		copied = status & BIT(0) ? true : false;
	}

	return copied;
}
