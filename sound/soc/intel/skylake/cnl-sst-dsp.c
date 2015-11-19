/*
 * cnl-sst-dsp.c - CNL SST library generic function
 *
 * Copyright (C) 2015-16, Intel Corporation.
 * Author: Guneshwor Singh <guneshwor.o.singh@intel.com>
 *
 * Modified from:
 *	SKL SST library generic function
 *	Copyright (C) 2014-15, Intel Corporation.
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#include <sound/pcm.h>

#include "../common/sst-dsp.h"
#include "../common/sst-ipc.h"
#include "../common/sst-dsp-priv.h"
#include "cnl-sst-dsp.h"

/* various timeout values */
#define CNL_DSP_PU_TO		50
#define CNL_DSP_PD_TO		50
#define CNL_DSP_RESET_TO	50

static int cnl_dsp_core_set_reset_state(struct sst_dsp  *ctx)
{
	int ret;

	/* update bits */
	sst_dsp_shim_update_bits_unlocked(ctx,
			CNL_ADSP_REG_ADSPCS, CNL_ADSPCS_CRST_MASK,
			CNL_ADSPCS_CRST(CNL_DSP_CORES_MASK));

	/* poll with timeout to check if operation successful */
	ret = sst_dsp_register_poll(ctx,
			CNL_ADSP_REG_ADSPCS,
			CNL_ADSPCS_CRST_MASK,
			CNL_ADSPCS_CRST(CNL_DSP_CORES_MASK),
			CNL_DSP_RESET_TO,
			"Set reset");
	if ((sst_dsp_shim_read_unlocked(ctx, CNL_ADSP_REG_ADSPCS) &
				CNL_ADSPCS_CRST(CNL_DSP_CORES_MASK)) !=
				CNL_ADSPCS_CRST(CNL_DSP_CORES_MASK)) {
		dev_err(ctx->dev, "Set reset state failed\n");
		ret = -EIO;
	}

	return ret;
}

static int cnl_dsp_core_unset_reset_state(struct sst_dsp  *ctx)
{
	int ret;

	/* update bits */
	sst_dsp_shim_update_bits_unlocked(ctx, CNL_ADSP_REG_ADSPCS,
					CNL_ADSPCS_CRST_MASK1, 0);

	/* poll with timeout to check if operation successful */
	ret = sst_dsp_register_poll(ctx,
			CNL_ADSP_REG_ADSPCS,
			CNL_ADSPCS_CRST_MASK1,
			0,
			CNL_DSP_RESET_TO,
			"Unset reset");

	if ((sst_dsp_shim_read_unlocked(ctx, CNL_ADSP_REG_ADSPCS) &
				 CNL_ADSPCS_CRST1(1)) != 0) {
		dev_err(ctx->dev, "Unset reset state failed\n");
		ret = -EIO;
	}

	return ret;
}

static bool is_cnl_dsp_core_enable(struct sst_dsp *ctx)
{
	int val;
	bool is_enable;

	val = sst_dsp_shim_read_unlocked(ctx, CNL_ADSP_REG_ADSPCS);

	is_enable = ((val & CNL_ADSPCS_CPA(CNL_DSP_CORES_MASK)) &&
			(val & CNL_ADSPCS_SPA(CNL_DSP_CORES_MASK)) &&
			!(val & CNL_ADSPCS_CRST(CNL_DSP_CORES_MASK)) &&
			!(val & CNL_ADSPCS_CSTALL(CNL_DSP_CORES_MASK)));

	dev_dbg(ctx->dev, "DSP core is enabled=%d\n", is_enable);
	return is_enable;
}

static int cnl_dsp_reset_core(struct sst_dsp *ctx)
{
	/* stall core */
	sst_dsp_shim_write_unlocked(ctx, CNL_ADSP_REG_ADSPCS,
			sst_dsp_shim_read_unlocked(ctx, CNL_ADSP_REG_ADSPCS) &
			CNL_ADSPCS_CSTALL(CNL_DSP_CORES_MASK));

	/* set reset state */
	return cnl_dsp_core_set_reset_state(ctx);
}

static int cnl_dsp_start_core(struct sst_dsp *ctx)
{
	int ret;

	/* unset reset state */
	ret = cnl_dsp_core_unset_reset_state(ctx);
	if (ret < 0) {
		dev_dbg(ctx->dev, "dsp unset reset failed\n");
		return ret;
	}

	/* run core */
	dev_dbg(ctx->dev, "Unstalling core...\n");
	/* FIXME Unstalling only one core out of 4 cores for CNL */
	sst_dsp_shim_write_unlocked(ctx, CNL_ADSP_REG_ADSPCS,
			 sst_dsp_shim_read_unlocked(ctx, CNL_ADSP_REG_ADSPCS) &
				~CNL_ADSPCS_CSTALL1(1));
	dev_dbg(ctx->dev, "FW Poll Status: reg=0x%#x\n",
		sst_dsp_shim_read(ctx, CNL_ADSP_REG_ADSPCS));

	/* FIXME Disabling this check since we unstalled only one core */

	/* if (!is_cnl_dsp_core_enable(ctx)) {
		cnl_dsp_reset_core(ctx);
		dev_err(ctx->dev, "DSP core enable failed\n");
		ret = -EIO;
	} */

	return ret;
}

static int cnl_dsp_core_power_up(struct sst_dsp  *ctx)
{
	int ret;

	/* update bits */
	sst_dsp_shim_update_bits_unlocked(ctx, CNL_ADSP_REG_ADSPCS,
		CNL_ADSPCS_SPA_MASK, CNL_ADSPCS_SPA(CNL_DSP_CORES_MASK));

	/* poll with timeout to check if operation successful */
	ret = sst_dsp_register_poll(ctx,
			CNL_ADSP_REG_ADSPCS,
			CNL_ADSPCS_CPA_MASK,
			CNL_ADSPCS_CPA(CNL_DSP_CORES_MASK),
			CNL_DSP_PU_TO,
			"Power up");

	if ((sst_dsp_shim_read_unlocked(ctx, CNL_ADSP_REG_ADSPCS) &
			CNL_ADSPCS_CPA(CNL_DSP_CORES_MASK)) !=
			CNL_ADSPCS_CPA(CNL_DSP_CORES_MASK)) {
		dev_err(ctx->dev, "DSP core power up failed\n");
		ret = -EIO;
	}

	return ret;
}

static int cnl_dsp_core_power_down(struct sst_dsp  *ctx)
{
	/* update bits */
	sst_dsp_shim_update_bits_unlocked(ctx, CNL_ADSP_REG_ADSPCS,
					CNL_ADSPCS_SPA_MASK, 0);

	/* poll with timeout to check if operation successful */
	return sst_dsp_register_poll(ctx,
			CNL_ADSP_REG_ADSPCS,
			CNL_ADSPCS_CPA_MASK,
			0,
			CNL_DSP_PD_TO,
			"Power down");
}

int cnl_dsp_enable_core(struct sst_dsp *ctx)
{
	int ret;

	/* power up */
	ret = cnl_dsp_core_power_up(ctx);
	if (ret < 0) {
		dev_dbg(ctx->dev, "dsp core power up failed\n");
		return ret;
	}

	return cnl_dsp_start_core(ctx);
}

int cnl_dsp_disable_core(struct sst_dsp *ctx)
{
	int ret;

	ret = cnl_dsp_reset_core(ctx);
	if (ret < 0) {
		dev_err(ctx->dev, "dsp core reset failed\n");
		return ret;
	}

	/* power down core*/
	ret = cnl_dsp_core_power_down(ctx);
	if (ret < 0) {
		dev_err(ctx->dev, "dsp core power down failed\n");
		return ret;
	}

	if (is_cnl_dsp_core_enable(ctx)) {
		dev_err(ctx->dev, "DSP core disable failed\n");
		ret = -EIO;
	}

	return ret;
}

irqreturn_t cnl_dsp_sst_interrupt(int irq, void *dev_id)
{
	struct sst_dsp *ctx = dev_id;
	u32 val;
	irqreturn_t result = IRQ_NONE;

	spin_lock(&ctx->spinlock);

	val = sst_dsp_shim_read_unlocked(ctx, CNL_ADSP_REG_ADSPIS);
	ctx->intr_status = val;

	if (val == 0xffffffff) {
		spin_unlock(&ctx->spinlock);
		return IRQ_NONE;
	}

	if (val & CNL_ADSPIS_IPC) {
		cnl_ipc_int_disable(ctx);
		result = IRQ_WAKE_THREAD;
	}

	spin_unlock(&ctx->spinlock);

	return result;
}

void cnl_dsp_free(struct sst_dsp *dsp)
{
	cnl_ipc_int_disable(dsp);

	free_irq(dsp->irq, dsp);
	cnl_dsp_disable_core(dsp);
}
EXPORT_SYMBOL_GPL(cnl_dsp_free);
