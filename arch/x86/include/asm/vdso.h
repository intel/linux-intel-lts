/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_X86_VDSO_H
#define _ASM_X86_VDSO_H

#include <asm/page_types.h>
#include <linux/linkage.h>
#include <linux/init.h>

#ifndef __ASSEMBLER__

#include <linux/mm_types.h>

struct vdso_image {
	void *data;
	unsigned long size;   /* Always a multiple of PAGE_SIZE */

	unsigned long alt, alt_len;

	long sym_vvar_start;  /* Negative offset to the vvar area */

	long sym_vvar_page;
	long sym_pvclock_page;
	long sym_hvclock_page;
	long sym_timens_page;
	long sym_VDSO32_NOTE_MASK;
	long sym___kernel_sigreturn;
	long sym___kernel_rt_sigreturn;
	long sym___kernel_vsyscall;
	long sym_int80_landing_pad;
};

#ifdef CONFIG_X86_64
extern const struct vdso_image vdso_image_64;
#endif

#ifdef CONFIG_X86_X32
extern const struct vdso_image vdso_image_x32;
#endif

#if defined CONFIG_X86_32 || defined CONFIG_COMPAT
extern const struct vdso_image vdso_image_32;
#endif

extern void __init init_vdso_image(const struct vdso_image *image);

extern int map_vdso_once(const struct vdso_image *image, unsigned long addr);

#else /* __ASSEMBLER__ */

/*
 * ENDBR is an instruction for the Indirect Branch Tracking (IBT) component
 * of CET.  IBT prevents attacks by ensuring that (most) indirect branches
 * function calls may only land at ENDBR instructions.  Branches that don't
 * follow the rules will result in control flow (#CF) exceptions.
 * ENDBR is a noop when IBT is unsupported or disabled.  Most ENDBR
 * instructions are inserted automatically by the compiler, but branch
 * targets written in assembly must have ENDBR added manually.
 */
#ifdef CONFIG_X86_IBT
#define ENDBR64 endbr64
#define ENDBR32 endbr32
#else
#define ENDBR64
#define ENDBR32
#endif

#endif /* __ASSEMBLER__ */
#endif /* _ASM_X86_VDSO_H */
