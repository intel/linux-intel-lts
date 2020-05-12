/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright(c) 2017-2020 Intel Corporation */

#ifndef __GNA_IRQ_H__
#define __GNA_IRQ_H__

#include <linux/interrupt.h>

extern irqreturn_t gna_interrupt(int irq, void *ctx);

extern irqreturn_t gna_irq_thread(int irq, void *ctx);

#endif // __GNA_IRQ_H__
