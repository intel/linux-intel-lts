// SPDX-License-Identifier: MIT
/*
 * Copyright(c) 2019 - 2021 Intel Corporation.
 *
 */

#include <linux/delay.h>
#include <linux/module.h>

#include "csr.h"
#include "iaf_drv.h"
#include "ops.h"
#include "parallel.h"
#include "routing_pause.h"
#include "routing_topology.h"

static bool routing_pause_enable = true;
static unsigned long routing_pause_timeout = 60; /* seconds */

static void write_pause(struct fsubdev *sd)
{
	/* only pause bridge requests; let the rest flush */
	u64 value = FIELD_PREP(MASK_BT_PAC_MDFI_REQ, 1);
	u32 offset = CSR_BT_PAUSE_CTRL;
	int err;
	u8 lpn;

	for_each_bridge_lpn(lpn, sd) {
		err = ops_linkmgr_port_csr_wr(sd, lpn, offset, &value,
					      sizeof(value), false);
		if (err) {
			sd_err(sd, "unable to pause bridge port %d: %d\n",
			       lpn, err);
			routing_sd_transition_error(sd);
			return;
		}
	}
}

static void write_unpause(struct fsubdev *sd)
{
	u64 value = 0; /* all pause flags clear */
	u32 offset = CSR_BT_PAUSE_CTRL;
	int err;
	u8 lpn;

	for_each_bridge_lpn(lpn, sd) {
		err = ops_linkmgr_port_csr_wr(sd, lpn, offset, &value,
					      sizeof(value), false);
		if (err) {
			sd_err(sd, "unable to unpause bridge port %d: %d\n",
			       lpn, err);
			routing_sd_transition_error(sd);
			return;
		}
	}
}

static int read_outstanding_tx(struct fsubdev *sd, u64 *count)
{
	u64 value = 0;
	u64 total = 0;
	u32 offset = CSR_BT_OUTSTANDING_TX;
	int err;
	u8 lpn;

	for_each_bridge_lpn(lpn, sd) {
		err = ops_linkmgr_port_csr_rd(sd, lpn, offset, sizeof(value),
					      &value);
		if (err) {
			sd_err(sd,
			       "unable to read outstanding tx for bridge port %d: %d\n",
			       lpn, err);
			routing_sd_transition_error(sd);
			return err;
		}

		total += FIELD_GET(MASK_BT_OT_FBRC, value);
		total += FIELD_GET(MASK_BT_OT_MDFI, value);
	}

	*count = total;
	return 0;
}

/**
 * struct pause_ctx_sd - Tracks per-sd pause state.
 * @sd: the sd being tracked
 * @alive: true if the sd has not failed along the way
 * @quiet: true once the sd has been observed to be paused and flushed
 */
struct pause_ctx_sd {
	struct fsubdev *sd;
	bool quiet;
};

/**
 * struct routing_pause_ctx - Tracks overall pause state.
 * @sd_count: count of sds in the sd array
 * @sd: array of per-sd contexts
 */
struct routing_pause_ctx {
	int sd_count;
	struct pause_ctx_sd sd[];
};

static void pause_fn(void *ctx)
{
	struct pause_ctx_sd *sd_ctx = ctx;

	write_pause(sd_ctx->sd);
}

static void pause_all(struct routing_pause_ctx *ctx)
{
	struct par_group group;
	int i;

	par_start(&group);

	for (i = 0; i < ctx->sd_count; ++i)
		par_work_queue(&group, pause_fn, ctx->sd + i);

	par_wait(&group);
}

static void unpause_fn(void *ctx)
{
	struct pause_ctx_sd *sd_ctx = ctx;

	write_unpause(sd_ctx->sd);
}

static void unpause_all(struct routing_pause_ctx *ctx)
{
	struct par_group group;
	int i;

	par_start(&group);

	for (i = 0; i < ctx->sd_count; ++i)
		if (!routing_sd_is_error(ctx->sd[i].sd))
			par_work_queue(&group, unpause_fn, ctx->sd + i);

	par_wait(&group);
}

static void poll_fn(void *ctx)
{
	struct pause_ctx_sd *sd = ctx;
	u64 count;
	int err;

	err = read_outstanding_tx(sd->sd, &count);
	if (err)
		return;

	if (!count)
		sd->quiet = 1;
}

/**
 * poll_all - Polls all devices in the context for outstanding transactions.
 * @ctx: the context to operate on
 *
 * Return: true if all devices are quiesced; false otherwise
 */
static bool poll_all(struct routing_pause_ctx *ctx)
{
	struct par_group group;
	struct pause_ctx_sd *sd;
	int i;

	par_start(&group);

	for (i = 0; i < ctx->sd_count; ++i) {
		sd = ctx->sd + i;

		if (routing_sd_is_error(sd->sd) || sd->quiet)
			continue;

		par_work_queue(&group, poll_fn, sd);
	}

	par_wait(&group);

	for (i = 0; i < ctx->sd_count; ++i) {
		sd = ctx->sd + i;

		if (!routing_sd_is_error(sd->sd) && !sd->quiet)
			return false;
	}

	return true;
}

/**
 * routing_pause_init - Allocs a new pause context.
 *
 * Return: pointer to context, or NULL on failure.
 */
struct routing_pause_ctx *routing_pause_init(void)
{
	struct routing_pause_ctx *ctx;
	struct pause_ctx_sd *sd_ctx;
	struct fsubdev *sd;
	int count = 0;

	/*
	 * if pause is not enabled, just return an empty context; calling code
	 * can continue as normal and the pause api calls will naturally
	 * degrade to no-ops
	 */
	if (!routing_pause_enable)
		return kzalloc(sizeof(*ctx), GFP_KERNEL);

	for (sd = routing_sd_iter(0); sd; sd = routing_sd_next(sd, 0))
		++count;

	ctx = kzalloc(struct_size(ctx, sd, count), GFP_KERNEL);
	if (!ctx)
		return NULL;

	for (sd = routing_sd_iter(0); sd; sd = routing_sd_next(sd, 0)) {
		sd_ctx = ctx->sd + ctx->sd_count++;
		sd_ctx->sd = sd;
		sd_ctx->quiet = false;
	}

	return ctx;
}

/**
 * routing_pause_start - Starts a pause period by pausing bridge
 * requests and waiting for the fabric to flush.
 * @ctx: context to operate on
 */
void routing_pause_start(struct routing_pause_ctx *ctx)
{
	unsigned long interval;
	unsigned long remaining;
	unsigned long end_jif;

	pause_all(ctx);

	end_jif = jiffies + msecs_to_jiffies(routing_pause_timeout * 1000);

	for (interval = 1; time_before(jiffies, end_jif); interval *= 10) {
		bool quiet = poll_all(ctx);

		if (quiet)
			break;

		/*
		 * skip the short wait on the first pass.  note that in
		 * general we expect the fabric to quiesce much faster than
		 * we spend polling for it.
		 */
		if (interval == 1)
			continue;

		remaining = jiffies_delta_to_msecs(end_jif - jiffies);
		msleep(min(interval, remaining));
	}
}

/**
 * routing_pause_end - Ends a pause period by unpausing devices
 * and frees the context.
 * @ctx: context to operate on
 */
void routing_pause_end(struct routing_pause_ctx *ctx)
{
	unpause_all(ctx);
	kfree(ctx);
}

module_param(routing_pause_enable, bool, 0400);
MODULE_PARM_DESC(routing_pause_enable, "Determines whether routing will attempt to pause the fabric when making programming changes (default: Y)");

module_param(routing_pause_timeout, ulong, 0400);
MODULE_PARM_DESC(routing_pause_timeout, "Maximum time to wait before declaring a timeout when waiting for fabric to quiesce in seconds (default: 60s)");
