// SPDX-License-Identifier: MIT
/*
 * Copyright(c) 2020 - 2021 Intel Corporation.
 *
 */

#include <linux/slab.h>

#include "iaf_drv.h"
#include "parallel.h"

/**
 * par_start - Initializes a parallel execution context.
 * @group: group instance to operate on
 */
void par_start(struct par_group *group)
{
	/*
	 * hold an additional ref to prevent work transitioning the ref to
	 * zero while queueing is in progress
	 */
	atomic_set(&group->outstanding, 1);
	init_completion(&group->done);
}

/**
 * par_wait - Waits for all parallel execution tied to the context to complete.
 * @group: group instance to operate on
 */
void par_wait(struct par_group *group)
{
	/*
	 * remove the "queueing in progress" ref, and wait only if other refs
	 * remain
	 */
	if (!atomic_dec_and_test(&group->outstanding))
		wait_for_completion(&group->done);
}

/**
 * struct par_work - The per-work-item execution context.
 * @work: the kernel workqueue item
 * @group: the parent parallel execution group
 * @fn: the function to execute
 * @fn_ctx: arbitrary context passed to the execution function
 */
struct par_work {
	struct work_struct work;
	struct par_group *group;
	void (*fn)(void *ctx);
	void *fn_ctx;
};

/**
 * par_work_fn - The workqueue function which passes control to the user
 * supplied execution function.
 * @work: the kernel workqueue item associated with the work
 */
static void par_work_fn(struct work_struct *work)
{
	struct par_work *pw = container_of(work, struct par_work, work);

	pw->fn(pw->fn_ctx);

	if (atomic_dec_and_test(&pw->group->outstanding))
		complete(&pw->group->done);

	kfree(pw);
}

/**
 * par_work_queue - Queues a new function to run in parallel.
 * @group: the parallel execution group
 * @fn: the function to execute
 * @fn_ctx: the context to pass to the execution function
 *
 * Return: zero on success; negative errno otherwise.
 */
int par_work_queue(struct par_group *group,
		   void (*fn)(void *), void *fn_ctx)
{
	struct par_work *pw;

	pw = kmalloc(sizeof(*pw), GFP_KERNEL);
	if (!pw)
		return -ENOMEM;

	INIT_WORK(&pw->work, par_work_fn);
	pw->group = group;
	pw->fn = fn;
	pw->fn_ctx = fn_ctx;
	atomic_inc(&group->outstanding);
	queue_work(iaf_unbound_wq, &pw->work);

	return 0;
}
