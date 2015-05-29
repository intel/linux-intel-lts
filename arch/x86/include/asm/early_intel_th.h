/*
 * early_intel_th.h: Intel Trace Hub early printk
 *
 * (C) Copyright 2015 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _ASM_X86_EARLY_INTEL_TH_H
#define _ASM_X86_EARLY_INTEL_TH_H

#ifdef CONFIG_INTEL_TH_EARLY_PRINTK
extern struct console intel_th_early_console;
extern void early_intel_th_init(const char *);
#endif /* CONFIG_INTEL_TH_EARLY_PRINTK */

#endif /* _ASM_X86_EARLY_INTEL_TH_H */

