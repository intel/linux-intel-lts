/* SPDX-License-Identifier: MIT */
/*
 * Copyright(c) 2020 - 2021 Intel Corporation.
 *
 */

#ifndef PARALLEL_H_INCLUDED
#define PARALLEL_H_INCLUDED

#include <linux/completion.h>
#include <linux/workqueue.h>

/**
 * struct par_group - Parallel execution context for a group of functions that
 * can be collectively waited on.
 * @outstanding: the number of outstanding functions
 * @done: signalled when all parallel work completes
 *
 * This schedules functions onto the system workqueue, and associates them with
 * a shared context.  An API call is provided to wait on this group of
 * functions for completion.
 */
struct par_group {
	atomic_t outstanding;
	struct completion done;
};

void par_start(struct par_group *ctx);
void par_wait(struct par_group *ctx);
int par_work_queue(struct par_group *ctx,
		   void (*fn)(void *), void *fn_ctx);

#endif /* PARALLEL_H_INCLUDED */
