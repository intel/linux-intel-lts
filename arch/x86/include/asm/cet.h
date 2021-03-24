/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_X86_CET_H
#define _ASM_X86_CET_H

#ifndef __ASSEMBLY__
#include <linux/types.h>

struct task_struct;

/*
 * Per-thread CET status
 */
struct thread_shstk {
	u64	base;
	u64	size;
};

#ifdef CONFIG_X86_SHADOW_STACK
int shstk_setup(void);
int shstk_alloc_thread_stack(struct task_struct *p, unsigned long clone_flags,
			     unsigned long stack_size);
void shstk_free(struct task_struct *p);
void shstk_disable(void);
int shstk_setup_rstor_token(bool ia32, unsigned long restorer,
			    unsigned long *new_ssp);
int shstk_check_rstor_token(bool ia32, unsigned long *new_ssp);
#else
static inline int shstk_setup(void) { return 0; }
static inline int shstk_alloc_thread_stack(struct task_struct *p,
					   unsigned long clone_flags,
					   unsigned long stack_size) { return 0; }
static inline void shstk_free(struct task_struct *p) {}
static inline void shstk_disable(void) {}
static inline int shstk_setup_rstor_token(bool ia32, unsigned long restorer,
					  unsigned long *new_ssp) { return 0; }
static inline int shstk_check_rstor_token(bool ia32,
					  unsigned long *new_ssp) { return 0; }
#endif

#endif /* __ASSEMBLY__ */

#endif /* _ASM_X86_CET_H */
