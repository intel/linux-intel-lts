// SPDX-License-Identifier: GPL-2.0
/*
 * ACRN detection support
 *
 * Copyright (C) 2019 Intel Corporation. All rights reserved.
 *
 * Jason Chen CJ <jason.cj.chen@intel.com>
 * Zhao Yakui <yakui.zhao@intel.com>
 *
 */

#include <linux/interrupt.h>
#include <asm/acrn.h>
#include <asm/apic.h>
#include <asm/desc.h>
#include <asm/hypervisor.h>
#include <asm/acrnhyper.h>
#include <asm/irq_vectors.h>
#include <asm/irq_regs.h>
#include <asm/desc.h>
#include <asm/apic.h>
#include <asm/cpufeature.h>

static uint32_t __init acrn_detect(void)
{
	return hypervisor_cpuid_base("ACRNACRNACRN\0\0", 0);
}

static void __init acrn_init_platform(void)
{
	/* Setup the IDT for ACRN hypervisor callback */
	alloc_intr_gate(HYPERVISOR_CALLBACK_VECTOR, acrn_hv_callback_vector);
}

static bool acrn_x2apic_available(void)
{
	return boot_cpu_has(X86_FEATURE_X2APIC);
}

static void (*acrn_intr_handler)(void);
/*
 * Handler for ACRN_HV_CALLBACK.
 */
__visible void __irq_entry acrn_hv_vector_handler(struct pt_regs *regs)
{
	struct pt_regs *old_regs = set_irq_regs(regs);

	/*
	 * The hypervisor requires that the APIC EOI should be acked.
	 * If the APIC EOI is not acked, the APIC ISR bit for the
	 * HYPERVISOR_CALLBACK_VECTOR will not be cleared and then it
	 * will block the interrupt whose vector is lower than
	 * HYPERVISOR_CALLBACK_VECTOR.
	 */
	entering_ack_irq();
#ifdef CONFIG_X86
	inc_irq_stat(irq_hv_callback_count);
#endif

	if (acrn_intr_handler)
		acrn_intr_handler();

	exiting_irq();
	set_irq_regs(old_regs);
}

void acrn_setup_intr_irq(void (*handler)(void))
{
	acrn_intr_handler = handler;
}

void acrn_remove_intr_irq(void)
{
	acrn_intr_handler = NULL;
}

const __initconst struct hypervisor_x86 x86_hyper_acrn = {
	.name                   = "ACRN",
	.detect                 = acrn_detect,
	.type			= X86_HYPER_ACRN,
	.init.init_platform     = acrn_init_platform,
	.init.x2apic_available  = acrn_x2apic_available,
};
EXPORT_SYMBOL(acrn_setup_intr_irq);
EXPORT_SYMBOL(acrn_remove_intr_irq);
