/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_X86_ACRNHYPER_H
#define _ASM_X86_ACRNHYPER_H

#include <linux/types.h>
#include <linux/atomic.h>
#include <linux/nmi.h>
#include <asm/io.h>

#ifdef CONFIG_ACRN_GUEST
/* ACRN Hypervisor callback */
void acrn_hv_callback_vector(void);

void acrn_setup_intr_irq(void (*handler)(void));
void acrn_remove_intr_irq(void);
#endif

#endif
