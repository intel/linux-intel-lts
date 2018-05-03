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

int shstk_alloc_thread_stack(struct task_struct *tsk, unsigned long clone_flags,
			     unsigned long stack_size)
{
	struct thread_shstk *shstk = &tsk->thread.shstk;
	struct cet_user_state *state;
	unsigned long addr;

	/*
	 * clone2 does not pass stack_size.  Use RLIMIT_STACK and cap to 4GB.
	 */
	if (!stack_size)
		stack_size = min_t(unsigned long long, rlimit(RLIMIT_STACK), SZ_4G);

	if (!shstk->size)
		return 0;

	/*
	 * For CLONE_VM, except vfork, the child needs a separate shadow
	 * stack.
	 */
	if ((clone_flags & (CLONE_VFORK | CLONE_VM)) != CLONE_VM)
		return 0;

	/*
	 * This is in clone() syscall and fpu__copy() already copies xstates
	 * from the parent.  If get_xsave_addr() returns null, then XFEATURE_
	 * CET_USER is still in init state, which certainly is an error.
	 */
	state = get_xsave_addr(&tsk->thread.fpu.state.xsave, XFEATURE_CET_USER);
	if (!state)
		return -EINVAL;

	/*
	 * Compat-mode pthreads share a limited address space.
	 * If each function call takes an average of four slots
	 * stack space, allocate 1/4 of stack size for shadow stack.
	 */
	if (in_compat_syscall())
		stack_size /= 4;

	stack_size = round_up(stack_size, PAGE_SIZE);
	addr = alloc_shstk(stack_size);
	if (IS_ERR_VALUE(addr)) {
		shstk->base = 0;
		shstk->size = 0;
		return PTR_ERR((void *)addr);
	}

	fpu__prepare_write(&tsk->thread.fpu);
	state->user_ssp = (u64)(addr + stack_size);
	shstk->base = addr;
	shstk->size = stack_size;
	return 0;
}

void shstk_free(struct task_struct *tsk)
{
	struct thread_shstk *shstk = &tsk->thread.shstk;

	if (!cpu_feature_enabled(X86_FEATURE_SHSTK) ||
	    !shstk->size ||
	    !shstk->base)
		return;

	/*
	 * When fork() with CLONE_VM fails, the child (tsk) already has a
	 * shadow stack allocated, and exit_thread() calls this function to
	 * free it.  In this case the parent (current) and the child share
	 * the same mm struct.
	 */
	if (!tsk->mm || tsk->mm != current->mm)
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
