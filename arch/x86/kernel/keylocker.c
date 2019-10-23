// SPDX-License-Identifier: GPL-2.0-only

/*
 * Key Locker feature check and support Internal Wrapping Key (IWKey)
 */

#include <linux/random.h>
#include <linux/bits.h>

#include <asm/keylocker.h>
#include <asm/fpu/types.h>

bool check_keylocker_readiness(void)
{
	u32 eax, ebx, ecx, edx;

	cpuid_count(KL_CPUID, 0, &eax, &ebx, &ecx, &edx);
	/* BIOS may not enable it on some systems. */
	if (!(ebx & KL_CPUID_EBX_AESKLE)) {
		pr_debug("x86/keylocker: not fully enabled\n");
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
