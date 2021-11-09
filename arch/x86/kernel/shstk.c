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
#include <asm/special_insns.h>

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

static unsigned long get_user_shstk_addr(void)
{
	struct fpu *fpu = &current->thread.fpu;
	unsigned long ssp = 0;

	fpregs_lock();

	if (fpregs_state_valid(fpu, smp_processor_id())) {
		rdmsrl(MSR_IA32_PL3_SSP, ssp);
	} else {
		struct cet_user_state *p;

		/*
		 * When !fpregs_state_valid() and get_xsave_addr() returns
		 * null, XFEAUTRE_CET_USER is in init state.  Shadow stack
		 * pointer is null in this case, so return zero.
		 */
		p = get_xsave_addr(&fpu->state.xsave, XFEATURE_CET_USER);
		if (p)
			ssp = p->user_ssp;
	}

	fpregs_unlock();

	return ssp;
}

/*
 * Create a restore token on the shadow stack.  A token is always 8-byte
 * and aligned to 8.
 */
static int create_rstor_token(bool ia32, unsigned long ssp,
			       unsigned long *token_addr)
{
	unsigned long addr;

	/* Aligned to 8 is aligned to 4, so test 8 first */
	if ((!ia32 && !IS_ALIGNED(ssp, 8)) || !IS_ALIGNED(ssp, 4))
		return -EINVAL;

	addr = ALIGN_DOWN(ssp, 8) - 8;

	/* Is the token for 64-bit? */
	if (!ia32)
		ssp |= BIT(0);

	if (write_user_shstk_64((u64 __user *)addr, (u64)ssp))
		return -EFAULT;

	*token_addr = addr;

	return 0;
}

/*
 * Create a restore token on shadow stack, and then push the user-mode
 * function return address.
 */
int shstk_setup_rstor_token(bool ia32, unsigned long ret_addr,
			    unsigned long *new_ssp)
{
	struct thread_shstk *shstk = &current->thread.shstk;
	unsigned long ssp, token_addr;
	int err;

	if (!shstk->size)
		return 0;

	if (!ret_addr)
		return -EINVAL;

	ssp = get_user_shstk_addr();
	if (!ssp)
		return -EINVAL;

	err = create_rstor_token(ia32, ssp, &token_addr);
	if (err)
		return err;

	if (ia32) {
		ssp = token_addr - sizeof(u32);
		err = write_user_shstk_32((u32 __user *)ssp, (u32)ret_addr);
	} else {
		ssp = token_addr - sizeof(u64);
		err = write_user_shstk_64((u64 __user *)ssp, (u64)ret_addr);
	}

	if (!err)
		*new_ssp = ssp;

	return err;
}

/*
 * Verify token_addr points to a valid token, and then set *new_ssp
 * according to the token.
 */
int shstk_check_rstor_token(bool proc32, unsigned long *new_ssp)
{
	unsigned long token_addr;
	unsigned long token;
	bool shstk32;

	token_addr = get_user_shstk_addr();

	if (get_user(token, (unsigned long __user *)token_addr))
		return -EFAULT;

	/* Is mode flag correct? */
	shstk32 = !(token & BIT(0));
	if (proc32 ^ shstk32)
		return -EINVAL;

	/* Is busy flag set? */
	if (token & BIT(1))
		return -EINVAL;

	/* Mask out flags */
	token &= ~3UL;

	/*
	 * Restore address aligned?
	 */
	if ((!proc32 && !IS_ALIGNED(token, 8)) || !IS_ALIGNED(token, 4))
		return -EINVAL;

	/*
	 * Token placed properly?
	 */
	if (((ALIGN_DOWN(token, 8) - 8) != token_addr) || token >= TASK_SIZE_MAX)
		return -EINVAL;

	*new_ssp = token;

	return 0;
}

int setup_signal_shadow_stack(int ia32, void __user *restorer)
{
	struct thread_shstk *shstk = &current->thread.shstk;
	unsigned long new_ssp;
	int err;

	if (!cpu_feature_enabled(X86_FEATURE_SHSTK) || !shstk->size)
		return 0;

	err = shstk_setup_rstor_token(ia32, (unsigned long)restorer,
				      &new_ssp);
	if (err)
		return err;

	start_update_msrs();
	err = wrmsrl_safe(MSR_IA32_PL3_SSP, new_ssp);
	end_update_msrs();

	return err;
}

int restore_signal_shadow_stack(void)
{
	struct thread_shstk *shstk = &current->thread.shstk;
	int ia32 = in_ia32_syscall();
	unsigned long new_ssp;
	int err;

	if (!cpu_feature_enabled(X86_FEATURE_SHSTK) || !shstk->size)
		return 0;

	err = shstk_check_rstor_token(ia32, &new_ssp);
	if (err)
		return err;

	start_update_msrs();
	err = wrmsrl_safe(MSR_IA32_PL3_SSP, new_ssp);
	end_update_msrs();

	return err;
}

unsigned long cet_alloc_shstk(unsigned long len)
{
	unsigned long token;
	unsigned long addr, ssp;

	addr = alloc_shstk(round_up(len, PAGE_SIZE));

	if (IS_ERR_VALUE(addr))
		return addr;

	/* Restore token is 8 bytes and aligned to 8 bytes */
	ssp = addr + len;
	token = ssp;

	if (!in_ia32_syscall())
		token |= BIT(0);
	ssp -= 8;

	if (write_user_shstk_64((u64 __user *)ssp, (u64)token)) {
		vm_munmap(addr, len);
		return -EINVAL;
	}

	return addr;
}
