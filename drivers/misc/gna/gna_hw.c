// SPDX-License-Identifier: GPL-2.0-only
// Copyright(c) 2017-2020 Intel Corporation

#include "gna.h"

#include "gna_drv.h"
#include "gna_hw.h"

void gna_start_scoring(struct gna_private *gna_priv, void __iomem *addr,
			      struct gna_compute_cfg *compute_cfg)
{
	unsigned long recovery_jiffies;
	union gna_ctrl_reg ctrl;

	ctrl.val = gna_reg_read(addr, GNACTRL);

	ctrl.ctrl.start_accel    = 1;
	ctrl.ctrl.compl_int_en   = 1;
	ctrl.ctrl.err_int_en     = 1;
	ctrl.ctrl.comp_stats_en  = compute_cfg->hw_perf_encoding & 0xF;
	ctrl.ctrl.active_list_en = compute_cfg->active_list_on & 0x1;
	ctrl.ctrl.gna_mode       = compute_cfg->gna_mode & 0x3;

	recovery_jiffies = msecs_to_jiffies(recovery_timeout * 1000);
	gna_priv->isr_timer.expires = jiffies + recovery_jiffies;
	add_timer(&gna_priv->isr_timer);

	gna_reg_write(addr, GNACTRL, ctrl.val);

	dev_dbg(&gna_priv->dev, "scoring started...\n");
}

static void gna_clear_saturation(
	struct gna_private *gna_priv, void __iomem *addr)
{
	u32 val;

	val = gna_reg_read(addr, GNASTS);
	if (val & GNA_STS_SATURATE) {
		dev_dbg(&gna_priv->dev, "saturation reached\n");
		dev_dbg(&gna_priv->dev, "gna status: %#x\n", val);

		val = val & GNA_STS_SATURATE;
		gna_reg_write(addr, GNASTS, val);

		val = gna_reg_read(addr, GNASTS);
		dev_dbg(&gna_priv->dev, "gna modified status: %#x\n", val);
	}
}

void gna_debug_isi(struct gna_private *gna_priv, void __iomem *addr)
{
	u32 isv_lo, isv_hi;

	gna_reg_write(addr, GNAISI, 0x80);
	isv_lo = gna_reg_read(addr, GNAPISV);
	isv_hi = gna_reg_read(addr, GNAPISV + sizeof(__u32));

	dev_dbg(&gna_priv->dev, "labase: %#x\n", isv_lo);
	dev_dbg(&gna_priv->dev, "lacnt: %#x\n", isv_hi);

	gna_reg_write(addr, GNAISI, 0x82);
	isv_lo = gna_reg_read(addr, GNAPISV);
	isv_hi = gna_reg_read(addr, GNAPISV + sizeof(__u32));

	dev_dbg(&gna_priv->dev, "{n_inputs,nnFlags,nnop}: %#x\n", isv_lo);
	dev_dbg(&gna_priv->dev,
	"{inputIteration/nInputConvStride,n_groups,n_outputs}: %#x\n", isv_hi);

	gna_reg_write(addr, GNAISI, 0x83);
	isv_lo = gna_reg_read(addr, GNAPISV);
	isv_hi = gna_reg_read(addr, GNAPISV + sizeof(__u32));

	dev_dbg(&gna_priv->dev,
	"{res,outFbIter,inputInLastIter/nConvFilterSize}: %#x\n", isv_lo);
	dev_dbg(&gna_priv->dev,
	"{outFbInLastIter/poolStride,outFbInFirstIter/nConvFlts: %#x\n", isv_hi);

	gna_reg_write(addr, GNAISI, 0x84);
	isv_lo = gna_reg_read(addr, GNAPISV);
	isv_hi = gna_reg_read(addr, GNAPISV + sizeof(__u32));

	dev_dbg(&gna_priv->dev,
	"{nActListElems/nCopyElems,res,nActSegs/poolSize}: %#x\n", isv_lo);
	dev_dbg(&gna_priv->dev, "reserved: %#x\n", isv_hi);

	gna_reg_write(addr, GNAISI, 0x86);
	isv_lo = gna_reg_read(addr, GNAPISV);
	isv_hi = gna_reg_read(addr, GNAPISV + sizeof(__u32));

	dev_dbg(&gna_priv->dev, "in_buffer: %#x\n", isv_lo);
	dev_dbg(&gna_priv->dev, "out_act_fn_buffer: %#x\n", isv_hi);

	gna_reg_write(addr, GNAISI, 0x87);
	isv_lo = gna_reg_read(addr, GNAPISV);
	isv_hi = gna_reg_read(addr, GNAPISV + sizeof(__u32));

	dev_dbg(&gna_priv->dev, "out_sum_buffer: %#x\n", isv_lo);
	dev_dbg(&gna_priv->dev, "out_fb_buffer: %#x\n", isv_hi);

	gna_reg_write(addr, GNAISI, 0x88);
	isv_lo = gna_reg_read(addr, GNAPISV);
	isv_hi = gna_reg_read(addr, GNAPISV + sizeof(__u32));

	dev_dbg(&gna_priv->dev, "weight/filter buffer: %#x\n", isv_lo);
	dev_dbg(&gna_priv->dev, "bias buffer: %#x\n", isv_hi);

	gna_reg_write(addr, GNAISI, 0x89);
	isv_lo = gna_reg_read(addr, GNAPISV);
	isv_hi = gna_reg_read(addr, GNAPISV + sizeof(__u32));

	dev_dbg(&gna_priv->dev, "indices buffer: %#x\n", isv_lo);
	dev_dbg(&gna_priv->dev, "pwl segments buffer: %#x\n", isv_hi);
}

void gna_abort_hw(struct gna_private *gna_priv, void __iomem *addr)
{
	u32 val;
	int i;

	/* saturation bit in the GNA status register needs
	 * to be expicitly cleared
	 */
	gna_clear_saturation(gna_priv, addr);

	val = gna_reg_read(addr, GNASTS);
	dev_dbg(&gna_priv->dev, "status before abort: %#x\n", val);

	val = gna_reg_read(addr, GNACTRL);
	val |= GNA_CTRL_ABORT_BIT;
	gna_reg_write(addr, GNACTRL, val);

	i = 100;
	do {
		val = gna_reg_read(addr, GNASTS);
		if ((val & 0x1) == 0)
			break;
	} while (--i);

	if (i == 0)
		dev_err(&gna_priv->dev, "abort did not complete\n");
}
