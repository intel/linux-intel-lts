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
	u64	locked:1;
	u64	ibt:1;
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
int setup_signal_shadow_stack(int ia32, void __user *restorer);
int restore_signal_shadow_stack(void);
unsigned long cet_alloc_shstk(unsigned long size);
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
static inline int setup_signal_shadow_stack(int ia32, void __user *restorer) { return 0; }
static inline int restore_signal_shadow_stack(void) { return 0; }
#endif

#ifdef CONFIG_X86_IBT
int ibt_setup(void);
void ibt_disable(void);
int ibt_get_clear_wait_endbr(void);
int ibt_set_wait_endbr(void);
#else
static inline int ibt_setup(void) { return 0; }
static inline void ibt_disable(void) {}
static inline int ibt_get_clear_wait_endbr(void) { return 0; }
static inline int ibt_set_wait_endbr(void) { return 0; }
#endif

#ifdef CONFIG_X86_SHADOW_STACK
int prctl_cet(int option, u64 arg2);
#else
static inline int prctl_cet(int option, u64 arg2) { return -EINVAL; }
#endif

#endif /* __ASSEMBLY__ */

#endif /* _ASM_X86_CET_H */
