// SPDX-License-Identifier: GPL-2.0-only
// Copyright(c) 2017-2020 Intel Corporation

#include "gna.h"

#include "gna_drv.h"
#include "gna_irq.h"

irqreturn_t gna_interrupt(int irq, void *priv)
{
	struct gna_private *gna_priv;
	void __iomem *addr;
	irqreturn_t ret;
	u32 hw_status;

	gna_priv = (struct gna_private *)priv;

	addr = gna_priv->bar0.mem_addr;
	ret = IRQ_HANDLED;

	hw_status = gna_reg_read(addr, GNASTS);
	dev_dbg(&gna_priv->dev,
		"received interrupt, device status: 0x%x\n", hw_status);

	/* check if interrupt originated from gna device */
	if (hw_status & GNA_INTERRUPT) {
		dev_dbg(&gna_priv->dev, "interrupt originated from gna device\n");

		spin_lock(&gna_priv->hw_lock);
		gna_priv->hw_status = hw_status;
		ret = IRQ_WAKE_THREAD;
		spin_unlock(&gna_priv->hw_lock);
	}

	return ret;
}

irqreturn_t gna_irq_thread(int irq, void *priv)
{
	struct gna_private *gna_priv;

	gna_priv = (struct gna_private *)priv;
	atomic_inc(&gna_priv->isr_count);
	tasklet_schedule(&gna_priv->request_tasklet);

	return IRQ_HANDLED;
}
