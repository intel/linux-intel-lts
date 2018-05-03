// SPDX-License-Identifier: GPL-2.0

#include <linux/errno.h>
#include <linux/uaccess.h>
#include <linux/prctl.h>
#include <linux/compat.h>
#include <linux/mman.h>
#include <linux/elfcore.h>
#include <linux/processor.h>
#include <asm/prctl.h>
#include <asm/cet.h>

/* See Documentation/x86/intel_cet.rst. */

static int cet_copy_status_to_user(struct thread_shstk *shstk, u64 __user *ubuf)
{
	u64 buf[3] = {};

	if (shstk->size) {
		buf[0] |= GNU_PROPERTY_X86_FEATURE_1_SHSTK;
		buf[1] = shstk->base;
		buf[2] = shstk->size;
	}

	return copy_to_user(ubuf, buf, sizeof(buf));
}

int prctl_cet(int option, u64 arg2)
{
	struct thread_shstk *shstk;

	if (!cpu_feature_enabled(X86_FEATURE_SHSTK))
		return -ENOTSUPP;

	shstk = &current->thread.shstk;

	if (option == ARCH_X86_CET_STATUS)
		return cet_copy_status_to_user(shstk, (u64 __user *)arg2);

	switch (option) {
	case ARCH_X86_CET_DISABLE:
		if (shstk->locked)
			return -EPERM;

		if (arg2 & ~GNU_PROPERTY_X86_FEATURE_1_VALID)
			return -EINVAL;
		if (arg2 & GNU_PROPERTY_X86_FEATURE_1_SHSTK)
			shstk_disable();
		return 0;

	case ARCH_X86_CET_LOCK:
		if (arg2)
			return -EINVAL;
		shstk->locked = 1;
		return 0;

	default:
		return -ENOSYS;
	}
}
