// SPDX-License-Identifier: GPL-2.0
/*
 * shstk.c - Intel shadow stack support
 *
 * Copyright (c) 2021, Intel Corporation.
 * Yu-cheng Yu <yu-cheng.yu@intel.com>
 */

#include <linux/types.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/sched/signal.h>
#include <linux/compat.h>
#include <linux/sizes.h>
#include <linux/user.h>
#include <asm/msr.h>
#include <asm/fpu/internal.h>
#include <asm/fpu/xstate.h>
#include <asm/fpu/types.h>
#include <asm/cet.h>

static void start_update_msrs(void)
{
	fpregs_lock();
	if (test_thread_flag(TIF_NEED_FPU_LOAD))
		__fpregs_load_activate();
}

static void end_update_msrs(void)
{
	fpregs_unlock();
}

static unsigned long alloc_shstk(unsigned long size)
{
	int flags = MAP_ANONYMOUS | MAP_PRIVATE;
	struct mm_struct *mm = current->mm;
	unsigned long addr, populate;

	mmap_write_lock(mm);
	addr = do_mmap(NULL, 0, size, PROT_READ, flags, VM_SHADOW_STACK, 0,
		       &populate, NULL);
	mmap_write_unlock(mm);

	return addr;
}

int shstk_setup(void)
{
	struct thread_shstk *shstk = &current->thread.shstk;
	unsigned long addr, size;

	if (!cpu_feature_enabled(X86_FEATURE_SHSTK))
		return -EOPNOTSUPP;

	size = round_up(min_t(unsigned long long, rlimit(RLIMIT_STACK), SZ_4G), PAGE_SIZE);
	addr = alloc_shstk(size);
	if (IS_ERR_VALUE(addr))
		return PTR_ERR((void *)addr);

	shstk->base = addr;
	shstk->size = size;

	start_update_msrs();
	wrmsrl(MSR_IA32_PL3_SSP, addr + size);
	wrmsrl(MSR_IA32_U_CET, CET_SHSTK_EN);
	end_update_msrs();

	return 0;
}

void shstk_free(struct task_struct *tsk)
{
	struct thread_shstk *shstk = &tsk->thread.shstk;

	if (!cpu_feature_enabled(X86_FEATURE_SHSTK) ||
	    !shstk->size ||
	    !shstk->base)
		return;

	if (!tsk->mm)
		return;

	while (1) {
		int r;

		r = vm_munmap(shstk->base, shstk->size);

		/*
		 * vm_munmap() returns -EINTR when mmap_lock is held by
		 * something else, and that lock should not be held for a
		 * long time.  Retry it for the case.
		 */
		if (r == -EINTR) {
			cond_resched();
			continue;
		}

		/*
		 * For all other types of vm_munmap() failure, either the
		 * system is out of memory or there is bug.
		 */
		WARN_ON_ONCE(r);
		break;
	}

	shstk->base = 0;
	shstk->size = 0;
}

void shstk_disable(void)
{
	struct thread_shstk *shstk = &current->thread.shstk;
	u64 msr_val;

	if (!cpu_feature_enabled(X86_FEATURE_SHSTK) ||
	    !shstk->size ||
	    !shstk->base)
		return;

	start_update_msrs();
	rdmsrl(MSR_IA32_U_CET, msr_val);
	wrmsrl(MSR_IA32_U_CET, msr_val & ~CET_SHSTK_EN);
	wrmsrl(MSR_IA32_PL3_SSP, 0);
	end_update_msrs();

	shstk_free(current);
}
