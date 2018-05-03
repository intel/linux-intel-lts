// SPDX-License-Identifier: GPL-2.0
/*
 * ibt.c - Intel Indirect Branch Tracking support
 *
 * Copyright (c) 2021, Intel Corporation.
 * Yu-cheng Yu <yu-cheng.yu@intel.com>
 */

#include <linux/user.h>
#include <asm/fpu/internal.h>
#include <asm/fpu/xstate.h>
#include <asm/fpu/types.h>
#include <asm/msr.h>
#include <asm/cet.h>

static int ibt_set_clear_msr_bits(u64 set, u64 clear)
{
	u64 msr;
	int r;

	fpregs_lock();

	if (test_thread_flag(TIF_NEED_FPU_LOAD))
		__fpregs_load_activate();

	r = rdmsrl_safe(MSR_IA32_U_CET, &msr);
	if (!r) {
		msr = (msr & ~clear) | set;
		r = wrmsrl_safe(MSR_IA32_U_CET, msr);
	}

	fpregs_unlock();

	return r;
}

int ibt_setup(void)
{
	int r;

	if (!cpu_feature_enabled(X86_FEATURE_IBT))
		return -EOPNOTSUPP;

	r = ibt_set_clear_msr_bits(CET_ENDBR_EN | CET_NO_TRACK_EN, 0);
	if (!r)
		current->thread.shstk.ibt = 1;

	return r;
}

void ibt_disable(void)
{
	if (!current->thread.shstk.ibt)
		return;

	ibt_set_clear_msr_bits(0, CET_ENDBR_EN);
	current->thread.shstk.ibt = 0;
}
