// SPDX-License-Identifier: MIT
/*
 * Copyright(c) 2019 - 2022 Intel Corporation.
 *
 */

#include <linux/module.h>
#include <linux/debugfs.h>

#include "debugfs.h"
#include "iaf_drv.h"
#include "routing_event.h"
#include "routing_engine.h"

static bool routing_shutting_down;
static bool routing_disable;
static unsigned long routing_min_delay_ms = 1000;
static unsigned long routing_max_delay_ms = 10000;

enum rem_state {
	REM_STATE_DISABLED = 0,
	REM_STATE_IDLE,
	REM_STATE_QUEUED,
	REM_STATE_RUNNING
};

static DEFINE_MUTEX(rem_lock);
static enum rem_state rem_state;
static u64 rem_pending_requests;
static u64 rem_serviced_requests;
static unsigned long rem_last_route_jiffies;
static struct delayed_work rem_routing_work;

/**
 * rem_schedule - Schedules routing work.
 *
 * Assumes caller holds the lock and has done the state checks required to
 * validate a schedule attempt.
 */
static void rem_schedule(void)
{
	unsigned long now = jiffies;
	unsigned long min_delay = msecs_to_jiffies(routing_min_delay_ms);
	unsigned long max_delay = msecs_to_jiffies(routing_max_delay_ms);
	unsigned long next = rem_last_route_jiffies + max_delay;
	unsigned long delay;

	if (time_after(next, now) && !routing_shutting_down) {
		delay = next - now;
		if (delay < min_delay)
			delay = min_delay;
	} else {
		delay = min_delay;
	}

	queue_delayed_work(iaf_unbound_wq, &rem_routing_work, delay);

	rem_state = REM_STATE_QUEUED;
}

static int rem_disable_change_callback(void)
{
	int ret = 0;

	mutex_lock(&rem_lock);

	switch (rem_state) {
	case REM_STATE_DISABLED:
		if (!routing_disable) {
			rem_state = REM_STATE_IDLE;
			if (rem_pending_requests != rem_serviced_requests)
				rem_schedule();
		}

		break;
	case REM_STATE_IDLE:
		if (routing_disable)
			rem_state = REM_STATE_DISABLED;
		break;
	default:
		ret = -EBUSY;
		break;
	}

	mutex_unlock(&rem_lock);

	return ret;
}

static void routing_work_fn(struct work_struct *work)
{
	u64 serviced_requests;

	if (rem_route_start(&serviced_requests))
		return;

	routing_sweep(serviced_requests);

	rem_route_finish();
}

void rem_init(void)
{
	struct dentry *root_node = get_debugfs_root_node();

	rem_state = routing_disable ? REM_STATE_DISABLED : REM_STATE_IDLE;
	rem_last_route_jiffies = jiffies;
	INIT_DELAYED_WORK(&rem_routing_work, routing_work_fn);

	debugfs_create_u32("rem_state", 0400, root_node, &rem_state);
	debugfs_create_u64("rem_pending_requests", 0400, root_node,
			   &rem_pending_requests);
	debugfs_create_u64("rem_serviced_requests", 0400, root_node,
			   &rem_serviced_requests);
	debugfs_create_ulong("rem_last_route_jiffies", 0400, root_node,
			     &rem_last_route_jiffies);
}

void rem_shutting_down(void)
{
	routing_shutting_down = 1;
}

void rem_stop(void)
{
	routing_disable = 1;
	rem_disable_change_callback();
	cancel_delayed_work_sync(&rem_routing_work);
}

/**
 * rem_request - Requests a routing sweep to be initiated.
 *
 * Return: true if routing was requested; false otherwise.
 */
bool rem_request(void)
{
	bool first;
	bool requested = true;

	mutex_lock(&rem_lock);

	first = rem_pending_requests == rem_serviced_requests;
	++rem_pending_requests;

	switch (rem_state) {
	case REM_STATE_DISABLED:
		requested = false;
		break;
	case REM_STATE_IDLE:
		if (first)
			rem_schedule();
		break;
	default:
		break;
	}

	mutex_unlock(&rem_lock);

	return requested;
}

/**
 * rem_route_start - Returns non-zero when starting was unsuccessful and the
 * routing engine should not proceed; zero otherwise.
 * @serviced_requests: number of rem requests serviced at the time of route
 * start (output parameter)
 *
 * Return: Zero on success, non-zero otherwise.
 */
int rem_route_start(u64 *serviced_requests)
{
	u64 events;

	mutex_lock(&rem_lock);

	switch (rem_state) {
	case REM_STATE_QUEUED:
		rem_state = REM_STATE_RUNNING;
		events = rem_pending_requests - rem_serviced_requests;
		rem_serviced_requests = rem_pending_requests;
		rem_last_route_jiffies = jiffies;
		*serviced_requests = rem_serviced_requests;
		pr_debug("routing fabric due to %llu events\n", events);
		break;
	default:
		mutex_unlock(&rem_lock);
		return -1;
	}

	mutex_unlock(&rem_lock);

	return 0;
}

void rem_route_finish(void)
{
	u64 events;

	mutex_lock(&rem_lock);

	switch (rem_state) {
	case REM_STATE_RUNNING:
		events = rem_pending_requests - rem_serviced_requests;
		if (routing_disable)
			rem_state = REM_STATE_DISABLED;
		else if (!events)
			rem_state = REM_STATE_IDLE;
		else
			rem_schedule();
		break;
	default:
		pr_warn("unexpected route finish in state %u\n", rem_state);
		break;
	}

	mutex_unlock(&rem_lock);
}

static int rem_routing_disable_set(const char *val,
				   const struct kernel_param *kp)
{
	bool cur = routing_disable;
	int err;

	err = param_set_bool(val, kp);
	if (err)
		return err;

	err = rem_disable_change_callback();
	if (err)
		routing_disable = cur;

	return err;
}

static const struct kernel_param_ops disable_ops = {
	.set = rem_routing_disable_set,
	.get = param_get_bool,
};

module_param_cb(routing_disable, &disable_ops, &routing_disable, 0600);
MODULE_PARM_DESC(routing_disable, "Disables routing engine from automatically running (default: N) (bool)");

module_param(routing_min_delay_ms, ulong, 0600);
MODULE_PARM_DESC(routing_min_delay_ms, "Minimum delay for scheduling the routing engine in milliseconds (default: 1000)");

module_param(routing_max_delay_ms, ulong, 0600);
MODULE_PARM_DESC(routing_max_delay_ms, "Maximum delay for scheduling the routing engine in milliseconds (default: 10000)");
