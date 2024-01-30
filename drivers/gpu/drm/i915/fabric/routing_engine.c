// SPDX-License-Identifier: MIT
/*
 * Copyright(c) 2019 - 2022 Intel Corporation.
 *
 */

#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/sched/clock.h>
#include <linux/wait.h>

#include "debugfs.h"
#include "port.h"
#include "routing_engine.h"
#include "routing_event.h"
#include "routing_logic.h"
#include "routing_io.h"
#include "routing_p2p.h"
#include "trace.h"

#define DEBUGFS_ROUTING_ROOT_NAME "routing"
#define DEBUGFS_ROUTING_STATUS_NAME "status"

/**
 * struct routing_context - tracks routing engine state
 * @topo: the topology structure tracking per-sweep state
 * @shutdown: true if the routing engine has been stopped
 * @gen_start: the generation counter incremented at sweep start
 * @gen_end: the generation counter incremented at sweep end
 * @gen_wait: the wait queue associated with sweep sync
 * @status: snapshot of routing status for query
 *
 * The start generation counter is incremented at the beginning of every
 * sweep attempt, regardless of whether any previous sweep passed or failed.
 *
 * The end generation counter is incremented to match start at the end of a
 * successful sweep attempt.
 *
 * For clients, if they make changes and then observe start == N, their changes
 * may have raced with sweep N.  To confirm that their changes have taken
 * effect, they want to wait until end > N, i.e. "at least one new sweep was
 * scheduled and completed successfully after sweep N."
 */
struct routing_context {
	struct routing_topology topo;
	bool shutdown;
	atomic_t gen_start;
	atomic_t gen_end;
	wait_queue_head_t gen_wait;
	struct routing_status {
		u64 serviced_requests;
		u64 usec_logic;
		u64 usec_io;
		u64 usec_p2p;
		int err;
		u16 num_devs;
		u16 num_sds_used;
		u16 num_sds_total;
		u16 num_ports_routed;
		u16 num_ports_routable;
		u16 num_ports_linkable;
		u16 num_ports_total;
		u16 num_planes;
	} status;
};

static struct routing_context routing_context;

static int debugfs_routing_status_open(struct inode *inode, struct file *file)
{
	struct routing_status *status = &routing_context.status;
	struct routing_status_info {
		struct debugfs_blob_wrapper blob;
		char buf[256];
	} *info;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	down_write(&routable_lock); /* exclusive lock */

	info->blob.size = scnprintf(info->buf, ARRAY_SIZE(info->buf),
				    "gs %d ge %d err %d d %u s %u %u p %u %u %u %u pl %u req %llu tl %llu ti %llu tp %llu\n",
				    atomic_read(&routing_context.gen_start),
				    atomic_read(&routing_context.gen_end),
				    status->err, status->num_devs,
				    status->num_sds_used, status->num_sds_total,
				    status->num_ports_routed, status->num_ports_routable,
				    status->num_ports_linkable, status->num_ports_total,
				    status->num_planes, status->serviced_requests,
				    status->usec_logic, status->usec_io, status->usec_p2p);

	up_write(&routable_lock);

	info->blob.data = info->buf;
	file->private_data = info;

	return 0;
}

static const struct file_operations debugfs_routing_status_ops = {
	.owner = THIS_MODULE,
	.open = debugfs_routing_status_open,
	.read = blob_read,
	.release = blob_release,
	.llseek = default_llseek,
};

static int routing_policy_param;

module_param_named(routing_policy, routing_policy_param, int, 0400);
MODULE_PARM_DESC(routing_policy, "Determine the routing policy (0=default 1=resiliency 2=route-through)");

static void topology_destroy(struct routing_topology *topo)
{
	struct routing_plane *plane, *plane_tmp;
	struct fsubdev *sd;

	list_for_each_entry(sd, &routable_list, routable_link)
		routing_sd_destroy(sd);

	list_for_each_entry_safe(plane, plane_tmp, &topo->plane_list, topo_link)
		routing_plane_destroy(plane);
}

/**
 * routing_init - Initializes the routing engine.
 */
void routing_init(void)
{
	struct routing_topology *topo = &routing_context.topo;
	struct dentry *root;

	switch (routing_policy_param) {
	case ROUTING_POLICY_DEFAULT:
		pr_info("routing policy: default\n");
		break;
	case ROUTING_POLICY_RESILIENCY:
		pr_info("routing policy: resiliency\n");
		break;
	case ROUTING_POLICY_ROUTE_THROUGH:
		pr_info("routing policy: route through\n");
		break;
	default:
		pr_err("routing policy: invalid; using default\n");
		routing_policy_param = ROUTING_POLICY_DEFAULT;
		break;
	}

	init_waitqueue_head(&routing_context.gen_wait);

	root = debugfs_create_dir(DEBUGFS_ROUTING_ROOT_NAME,
				  get_debugfs_root_node());

	debugfs_create_file(DEBUGFS_ROUTING_STATUS_NAME, 0400, root, NULL,
			    &debugfs_routing_status_ops);

	INIT_LIST_HEAD(&topo->plane_list);
	topo->policy = routing_policy_param;

	routing_p2p_init(root);
}

static void print_status(int err, u64 serviced_requests,
			 u64 logic_nsecs, u64 io_nsecs, u64 p2p_nsecs)
{
	DECLARE_BITMAP(devs_seen, (ROUTING_MAX_DEVICES + 1) / 2) = {};
	struct routing_status *status = &routing_context.status;
	struct fsubdev *sd;
	struct fport *port;
	bool complete;
	u8 lpn;

	memset(status, 0, sizeof(*status));
	status->num_planes = routing_context.topo.num_planes;
	status->serviced_requests = serviced_requests;
	status->usec_logic = logic_nsecs / 1000;
	status->usec_io = io_nsecs / 1000;
	status->usec_p2p = p2p_nsecs / 1000;
	status->err = err;

	list_for_each_entry(sd, &routable_list, routable_link) {
		pr_debug("routing status: device %u sd %u state %u\n",
			 sd->fdev->pd->index, sd_index(sd),
			 sd->routing.state);

		++status->num_sds_total;
		if (!test_bit(sd->fdev->pd->index, devs_seen)) {
			__set_bit(sd->fdev->pd->index, devs_seen);
			++status->num_devs;
		}

		if (routing_sd_is_error(sd))
			continue;

		++status->num_sds_used;

		for_each_fabric_port(port, lpn, sd) {
			bool routable = port->routing.routable;
			bool routed = port->routing.neighbor;

			switch (port->port_type) {
			case IAF_FW_PORT_TYPE_UNKNOWN:
			case IAF_FW_PORT_TYPE_DISCONNECTED:
				// disregard ports that are disabled for the sku
				break;
			case IAF_FW_PORT_TYPE_FIXED:
			case IAF_FW_PORT_TYPE_VARIABLE:
			case IAF_FW_PORT_TYPE_STANDARD:
			case IAF_FW_PORT_TYPE_SI_PHOTONICS:
				++status->num_ports_total;
				break;
			}

			switch (port->state) {
			case PM_PORT_STATE_INACTIVE:
			case PM_PORT_STATE_ISOLATED:
				// disregard ports that can't achieve link
				break;
			case PM_PORT_STATE_REQUESTED:
			case PM_PORT_STATE_ARMED:
			case PM_PORT_STATE_ACTIVE:
				++status->num_ports_linkable;
				break;
			}

			if (routable)
				++status->num_ports_routable;
			if (routed)
				++status->num_ports_routed;

			pr_debug("routing status: device %u sd %u port %u type %u state %u routable %u routed %u\n",
				 sd->fdev->pd->index, sd_index(sd), port->lpn,
				 port->port_type, port->state, routable, routed);
		}
	}

	complete = status->num_ports_routed == status->num_ports_linkable;

	pr_info("routing status: gen %d %s %s: devs %u sds %u/%u ports %u/%u/%u/%u planes %u\n",
		atomic_read(&routing_context.gen_end), err ? "fail" : "pass",
		complete ? "complete" : "partial", status->num_devs,
		status->num_sds_used, status->num_sds_total,
		status->num_ports_routed, status->num_ports_routable,
		status->num_ports_linkable, status->num_ports_total,
		status->num_planes);

	pr_debug("routing status debug: reqs %llu logic %llu io %llu p2p %llu\n",
		 status->serviced_requests, status->usec_logic, status->usec_io, status->usec_p2p);
}

/**
 * cleanup_next - Cleanup the "next" versions of data structures generated
 * during a sweep (generally when we're on an error path and won't be
 * activating them).
 */
static void cleanup_next(void)
{
	struct fsubdev *sd;

	for (sd = routing_sd_iter(0); sd; sd = routing_sd_next(sd, 0)) {
		routing_uft_destroy(sd->routing.uft_next);
		sd->routing.uft_next = NULL;

		kfree(sd->routing.dfid_map_next);
		sd->routing.dfid_map_next = NULL;
	}
}

/*
 * update_routed_status - Updates the "routed" field of all ports to reflect
 * whether they are in use.
 *
 * Assumes sweep success.
 *
 * If a device is in the error state, its "routed" status will be forced to
 * false here.  We don't do this at the moment of error transition to prevent
 * the previous status from changing until a sweep has succeeded, ensuring the
 * status correctly reflects that the fabric-wide routing is no longer using
 * it.
 */
static void update_routed_status(void)
{
	struct fsubdev *sd;
	struct fport *port;
	u8 lpn;

	/* enumerate ALL devices to handle forcing failed devices to false */
	for (sd = routing_sd_iter(1); sd; sd = routing_sd_next(sd, 1)) {
		for_each_fabric_port(port, lpn, sd) {
			bool routed = !routing_sd_is_error(sd) &&
				      port->routing.neighbor;

			if (!routed)
				list_del_init(&port->unroute_link);

			atomic_set(&port->routed, routed);
		}
	}
}

/**
 * routing_sweep - Perform routing calculation and programming.
 * @serviced_requests: number of rem requests serviced at the time of route
 * start
 */
void routing_sweep(u64 serviced_requests)
{
	u64 ref;
	u64 logic_nsecs = 0;
	u64 io_nsecs = 0;
	u64 p2p_nsecs = 0;
	int gen_start;
	int err;

	down_write(&routable_lock); /* exclusive lock */

	gen_start = atomic_inc_return(&routing_context.gen_start);

	pr_debug("routing start: gen %u\n", gen_start);

	routing_topo_reset_sd_error(&routing_context.topo);

	ref = local_clock();
	err = routing_logic_run(&routing_context.topo);
	logic_nsecs = local_clock() - ref;

	downgrade_write(&routable_lock); /* downgrade to shared/read lock */

	if (err) {
		pr_err("failed to route fabric: %d\n", err);
		cleanup_next();
		goto finalize;
	}

	ref = local_clock();
	err = routing_io_run();
	io_nsecs = local_clock() - ref;

	if (err) {
		pr_err("failed to program fabric: %d\n", err);
		cleanup_next();
		goto finalize;
	}

	ref = local_clock();
	routing_p2p_cache();
	p2p_nsecs = local_clock() - ref;

finalize:
	/*
	 * If any device transitioned to error, we likely have an inconsistent
	 * fabric programmed, or at least may see a recovery on a resweep
	 * due to the device being excluded.  Explicitly reschedule routing.
	 *
	 * Otherwise, device-agnostic routing errors with no change in device
	 * error states are unlikely to be recoverable; skip explicit resweep
	 * and just wait for normal event driven sweep signals.
	 */
	if (routing_topo_check_sd_error(&routing_context.topo)) {
		pr_warn("device error during routing; scheduling resweep\n");
		rem_request();
	} else if (!err) {
		update_routed_status();
		atomic_set(&routing_context.gen_end,
			   atomic_read(&routing_context.gen_start));
		wake_up(&routing_context.gen_wait);
	}

	print_status(err, serviced_requests, logic_nsecs, io_nsecs, p2p_nsecs);
	up_read(&routable_lock);
}

/**
 * routing_stop - Stops the routing engine, but doesn't invalidate it.
 */
void routing_stop(void)
{
	routing_context.shutdown = true;
	wake_up(&routing_context.gen_wait);
}

/**
 * routing_destroy - Tears down the routing engine.
 */
void routing_destroy(void)
{
	topology_destroy(&routing_context.topo);
}

#define DPA_BITS order_base_2(ROUTING_MIN_DPA_PER_SD)

/**
 * fidgen_init - Fills out the bridge FIDGEN registers for DFID lookup.
 * @fidgen: The fidgen struct to operate on.
 *
 * Assumes the incoming structure is zeroed.
 *
 * Mask A selects tile DPA ranges as indexes into the LUT.
 * Mask B is unused.
 * Mask D selects all hashable bits as hash input.
 * Mask H selects all hashed path bits as modulo input.
 * Modulo selects the maximum number of alternate paths from H.
 * Net result is that we map each DPA range into 64 DFIDs.
 */
static void fidgen_init(struct routing_fidgen *fidgen)
{
	fidgen->mask_a = GENMASK_ULL(CSR_FIDGEN_LUT_INDEX_WIDTH, 0) << DPA_BITS;
	fidgen->shift_a = DPA_BITS | FIELD_PREP(MASK_FIDGEN_SHIFT_RIGHT, 1);
	fidgen->mask_h = GENMASK_ULL(63, 0);
	fidgen->mask_d = GENMASK_ULL(DPA_BITS - 1, 0);
	fidgen->modulo = 7;
}

/**
 * routing_sd_init - Initializes the routing-specific data within a sd.
 * @sd: The sd to initialize.
 */
void routing_sd_init(struct fsubdev *sd)
{
	/* shift from 1GB space in platform device to routing block size */
	static const int shift = order_base_2(ROUTING_MIN_DPA_PER_SD) - 30;

	u16 sd_size = sd->fdev->pd->dpa.pkg_size / sd->fdev->pd->sd_cnt;

	WARN(sd_size & ((1u << shift) - 1),
	     "package size not a multiple of the minimum DPA block size");

	sd->routing.topo = &routing_context.topo;
	sd->routing.state = TILE_ROUTING_STATE_VALID;
	INIT_LIST_HEAD(&sd->routing.plane_link);

	sd->routing.dpa_idx_range = sd_size >> shift;
	sd->routing.dpa_idx_base = (sd->fdev->pd->dpa.pkg_offset >> shift) +
				   (sd_index(sd) * sd->routing.dpa_idx_range);

	sd->routing.fid_group = sd->fdev->pd->index * IAF_MAX_SUB_DEVS +
				sd_index(sd);
	sd->routing.fid_mgmt = ROUTING_FID_CPORT_BASE + sd->routing.fid_group;
	sd->routing.fid_base = ROUTING_FID_BLOCK_BASE +
			       ROUTING_FID_BLOCK_SIZE * sd->routing.fid_group;

	fidgen_init(&sd->routing.fidgen);

	pr_debug("device %u sd %u fid_mgmt 0x%04x fid_base 0x%04x\n",
		 sd->fdev->pd->index, sd_index(sd),
		 sd->routing.fid_mgmt, sd->routing.fid_base);

	trace_rt_assign_fids(sd);
}

/**
 * routing_sd_once - Performs one-time programming of the static subset of
 * routing data.
 * @sd: the subdevice to initialize
 *
 * Return: 0 on success, non-zero otherwise.
 */
int routing_sd_once(struct fsubdev *sd)
{
	int err;

	err = routing_io_sd_once(sd);
	if (err)
		return err;

	down_write(&routable_lock); /* exclusive lock */
	list_add(&sd->routable_link, &routable_list);
	up_write(&routable_lock);

	return 0;
}

/**
 * sweep_sync - Synchronously waits for a successful routing sweep.
 *
 * Note: will return without waiting during driver unload.
 */
static void sweep_sync(void)
{
	int target = atomic_read(&routing_context.gen_start) + 1;

	if (rem_request())
		wait_event_killable
			(routing_context.gen_wait,
			 routing_context.shutdown ||
			 atomic_read(&routing_context.gen_end) >= target);
}

/**
 * routing_dev_unroute - Synchronously ensures the device is no longer actively
 * routed.
 * @dev: fabric device
 *
 * Clears the routability of all ports on all subdevices of the device, then
 * waits for a sweep to complete.
 *
 * Note that if the routing event manager has been disabled (the common case
 * during driver unload), this will return without waiting for routing.
 */
void routing_dev_unroute(struct fdev *dev)
{
	struct fsubdev *sd;
	struct fport *p;
	bool changed = false;
	u8 lpn;
	u8 i;

	for (i = 0; i < dev->pd->sd_cnt; ++i) {
		sd = dev->sd + i;

		/* don't unroute that which can't be routed */
		if (list_empty(&sd->routable_link))
			continue;

		for_each_fabric_port(p, lpn, sd)
			changed |= test_and_clear_bit(PORT_CONTROL_ROUTABLE, p->controls);
	}

	if (changed)
		sweep_sync();
}

/**
 * routing_sd_destroy - Uninitializes the routing-specific data within a sd.
 * @sd: The sd to destroy.
 */
void routing_sd_destroy(struct fsubdev *sd)
{
	down_write(&routable_lock); /* exclusive lock */

	list_del_init(&sd->routable_link);
	routing_sd_transition_error(sd);
	memset(&sd->routing, 0, sizeof(sd->routing));

	up_write(&routable_lock);
}

/**
 * routing_port_routed_query - Returns port usage status.
 * @sd: sd to operate on
 * @port_mask: mask of ports to query
 * @usage_mask: mask of usage by port
 *
 * The bits in both masks are logical port numbers, inclusive of port 0, and
 * must be long enough to include the maximum fabric port number.
 */
void routing_port_routed_query(struct fsubdev *sd, unsigned long *port_mask,
			       unsigned long *usage_mask)
{
	struct fport *port;
	u8 lpn;

	for_each_masked_port(port, lpn, sd->port, port_mask, PORT_COUNT)
		if (port && atomic_read(&port->routed))
			__set_bit(port->lpn, usage_mask);
}

void routing_generation_read(u32 *counter_start, u32 *counter_end)
{
	*counter_start = (u32)atomic_read(&routing_context.gen_start);
	*counter_end = (u32)atomic_read(&routing_context.gen_end);
}
