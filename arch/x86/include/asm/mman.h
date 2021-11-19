/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_X86_MMAN_H
#define _ASM_X86_MMAN_H

#include <linux/mm.h>
#include <uapi/asm/mman.h>

#ifdef CONFIG_X86_INTEL_MEMORY_PROTECTION_KEYS
/*
 * Take the 4 protection key bits out of the vma->vm_flags
 * value and turn them in to the bits that we can put in
 * to a pte.
 *
 * Only override these if Protection Keys are available
 * (which is only on 64-bit).
 */
#define arch_vm_get_page_prot(vm_flags)	__pgprot(	\
		((vm_flags) & VM_PKEY_BIT0 ? _PAGE_PKEY_BIT0 : 0) |	\
		((vm_flags) & VM_PKEY_BIT1 ? _PAGE_PKEY_BIT1 : 0) |	\
		((vm_flags) & VM_PKEY_BIT2 ? _PAGE_PKEY_BIT2 : 0) |	\
		((vm_flags) & VM_PKEY_BIT3 ? _PAGE_PKEY_BIT3 : 0))

#define pkey_vm_prot_bits(prot, key) (			\
		((key) & 0x1 ? VM_PKEY_BIT0 : 0) |      \
		((key) & 0x2 ? VM_PKEY_BIT1 : 0) |      \
		((key) & 0x4 ? VM_PKEY_BIT2 : 0) |      \
		((key) & 0x8 ? VM_PKEY_BIT3 : 0))
#else
#define pkey_vm_prot_bits(prot, key) (0)
#endif

static inline unsigned long arch_calc_vm_prot_bits(unsigned long prot,
						   unsigned long pkey)
{
	unsigned long vm_prot_bits = pkey_vm_prot_bits(prot, pkey);

	if (prot & PROT_SHADOW_STACK)
		vm_prot_bits |= VM_SHADOW_STACK;

	return vm_prot_bits;
}

#define arch_calc_vm_prot_bits(prot, pkey) arch_calc_vm_prot_bits(prot, pkey)

#ifdef CONFIG_X86_SHADOW_STACK
static inline bool arch_validate_prot(unsigned long prot, unsigned long addr)
{
	unsigned long valid = PROT_READ | PROT_WRITE | PROT_EXEC | PROT_SEM |
			      PROT_SHADOW_STACK;

	if (prot & ~valid)
		return false;

	if (prot & PROT_SHADOW_STACK) {
		if (!current->thread.shstk.size)
			return false;

		/*
		 * A shadow stack mapping is indirectly writable by only
		 * the CALL and WRUSS instructions, but not other write
		 * instructions).  PROT_SHADOW_STACK and PROT_WRITE are
		 * mutually exclusive.
		 */
		if (prot & PROT_WRITE)
			return false;
	}

	return true;
}

#define arch_validate_prot arch_validate_prot

static inline bool arch_validate_flags(struct vm_area_struct *vma, unsigned long vm_flags)
{
	/*
	 * Shadow stack must be anonymous and not shared.
	 */
	if ((vm_flags & VM_SHADOW_STACK) && !vma_is_anonymous(vma))
		return false;

	return true;
}

#define arch_validate_flags(vma, vm_flags) arch_validate_flags(vma, vm_flags)

#endif /* CONFIG_X86_SHADOW_STACK */

#endif /* _ASM_X86_MMAN_H */
