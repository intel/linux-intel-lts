// SPDX-License-Identifier: MIT
/*
 * Copyright(c) 2020 - 2021 Intel Corporation.
 *
 */

#include <linux/seq_file.h>
#include <linux/debugfs.h>

#include "routing_logic.h"
#include "routing_p2p.h"

/**
 * struct routing_p2p_entry - The peer connectivity data from a single source
 * device (NOT subdevice) to all destination devices.
 * @devs: mapping from dev->pd->index to struct query_info
 */
struct routing_p2p_entry {
	struct xarray devs;
};

/**
 * bw_from_speed - Returns the bandwidth as a function of link speed only.
 * @link_speed: The STL-encoded link speed.
 *
 * Return: - The bandwidth in units of tenths of Gbps, or 0 otherwise.
 */
static u32 bw_from_speed(u32 link_speed)
{
	switch (link_speed) {
	case LINK_SPEED_12G: return 125;
	case LINK_SPEED_25G: return 258;
	case LINK_SPEED_53G: return 531;
	case LINK_SPEED_90G: return 900;
	}

	return 0;
}

/**
 * bw_from_width - Returns the bandwidth represented by a link width only.
 * @link_width: The STL-encoded link width.
 *
 * Return: The bandwidth in units of tenths of Gbps.
 */
static u32 bw_from_width(u32 link_width)
{
	/* order_base_2() is always positive */
	return (u32)(order_base_2(link_width) + 1);
}

/**
 * get_bandwidth_port - Returns the bandwidth of a port.
 * @port: The port to operate on.
 *
 * Return: The bandwidth in tenths of Gbps.
 */
static u32 get_bandwidth_port(struct fport *port)
{
	u32 speed = bw_from_speed(port->portinfo->link_speed_active);
	u32 rx = bw_from_width(port->portinfo->link_width_downgrade_rx_active);
	u32 tx = bw_from_width(port->portinfo->link_width_downgrade_tx_active);

	return speed * min(rx, tx);
}

/*
 * get_bandwidth_link - Returns the bandwidth of a link, factoring in the
 * port on either end.
 * @port: A port on either end of the link.
 *
 * Return: The bandwidth in tenths of Gbps.
 */
static u32 get_bandwidth_link(struct fport *port)
{
	u32 bw_a = get_bandwidth_port(port);
	u32 bw_b = get_bandwidth_port(port->routing.neighbor);

	return min(bw_a, bw_b);
}

/**
 * struct trace_path - Represents the tracing state for a single path, which
 * is a collection of identically routed DFIDs.
 * @sd_curr: The "current" sd (at the tail of the intermediate path)
 * @dfid_mask: The mask of the FID offsets (from the base FID) that follow
 * this path
 * @hops: The length of the path in hops.
 * @done: True if this path has reached the ingress sd for the destination
 * device, or is otherwise invalid (i.e. no further processing required)
 */
struct trace_path {
	struct fsubdev *sd_curr;
	u64 dfid_mask;
	u8 hops;
	bool done;
};

/*
 * struct trace - Tracks the tracing state for an entire trace of all paths
 * between a single subdevice pair.
 * @sd_src: The source subdevice.
 * @sd_dst: The destination subdevice.
 * @port_mask_ingress: The mask of port offsets used to ingress into the fabric
 * from the source subdevice.
 * @port_mask_egress: The mask of port offsets used to egress from the fabric
 * into the egress subdevice.
 * @num_paths: The number of paths in the paths array.
 * @paths: The set of traced paths, each one defined as a path through the
 * fabric shared by a subset of the DFIDs.
 *
 * A trace will begin with one trace path, representing start of the
 * path of the entire block of DFIDs.
 */
struct trace {
	struct fsubdev *sd_src;
	struct fsubdev *sd_dst;
	DECLARE_BITMAP(port_mask_ingress, PORT_COUNT);
	DECLARE_BITMAP(port_mask_egress, PORT_COUNT);
	u8 num_paths;
	struct trace_path paths[ROUTING_FID_BLOCK_SIZE];
};

/*
 * extend_path_one - Extends a single path trace by one hop.
 * @trace: The trace to operate on.
 * @path: The path to operate on.
 * @port: The egress port to extend this path along.
 *
 * Return: 0 on success, non-zero otherwise.
 */
static int extend_path_one(struct trace *trace,
			   struct trace_path *path,
			   struct fport *port)
{
	struct fsubdev *sd_neighbor;
	struct fport *port_neighbor;
	u16 cost_before, cost_after;
	u8 lpn_neighbor = port->portinfo->neighbor_port_number;

	sd_neighbor = find_routable_sd(port->portinfo->neighbor_guid);
	if (!sd_neighbor)
		goto fail;

	/* watch for trivial cycles */
	if (path->sd_curr == sd_neighbor)
		goto fail;

	/* ensure we're at least not regressing (NOTE: won't prevent loops) */
	cost_before = routing_cost_lookup(path->sd_curr, trace->sd_dst);
	cost_after = routing_cost_lookup(sd_neighbor, trace->sd_dst);
	if (cost_after > cost_before)
		goto fail;

	port_neighbor = get_fport_handle(sd_neighbor, lpn_neighbor);
	if (!port_neighbor)
		goto fail;

	if (path->sd_curr == trace->sd_src) {
		/*
		 * beginning of trace:
		 * cache the ingress port for the bw calculation
		 */
		port->routing.neighbor = port_neighbor;
	}

	path->sd_curr = sd_neighbor;
	path->hops++;

	/* if we've reached the destination, we're done */
	if (path->sd_curr == trace->sd_dst) {
		path->done = true;
		__set_bit(lpn_neighbor, trace->port_mask_egress);

		/*
		 * end of trace:
		 * cache the egress port for the bw calculation
		 */
		port_neighbor->routing.neighbor = port;
	}

	return 0;

fail:
	path->hops = 0;
	path->done = true;
	return -EINVAL;
}

/**
 * extend_path - Extends the trace of a specific path by one hop.
 * @trace: The trace to operate on.
 * @path: The specific path within the trace being operated on.
 *
 * Possibly forks additional trace paths if the path diverges at this hop
 * via multiple egress ports.
 */
static void extend_path(struct trace *trace, struct trace_path *path)
{
	struct trace_path *path_curr = path;
	struct trace_path *path_next;
	struct fport *port;
	u8 *block;
	/* the mask of FIDs remaining to be assigned to a port */
	unsigned long mask_remaining = path->dfid_mask;
	/* the mask of FIDs assigned to the current port */
	u64 mask_port;
	u16 fid_offset;
	u8 lpn;
	u8 uft_src;

	BUILD_BUG_ON(ROUTING_FID_BLOCK_SIZE > BITS_PER_LONG);

	if (routing_sd_is_error(path_curr->sd_curr)) {
		path_curr->hops = 0;
		path_curr->done = true;
		return;
	}

	block = routing_uft_bridge_get(path_curr->sd_curr->routing.uft,
				       trace->sd_dst);
	if (!block) {
		path_curr->hops = 0;
		path_curr->done = true;
		return;
	}

	/*
	 * determine what subset of the DFID set applies to each port.
	 * if the entire set is not exhausted on a given port, the remaining
	 * is split into a new path
	 */
	for_each_fabric_port(port, lpn, path_curr->sd_curr) {
		/* the path origin sources traffic from the bridge */
		uft_src = path_curr->sd_curr == trace->sd_src ? UFT_SEL_ORIGIN : UFT_SEL_HOP;

		/* determine the set of FIDs that use this port */
		mask_port = 0;
		for_each_set_bit(fid_offset, &mask_remaining, ROUTING_FID_BLOCK_SIZE) {
			if (routing_uft_entry_get(block, uft_src, fid_offset) == lpn)
				mask_port |= 1ull << fid_offset;
		}

		/* if no FIDs use this port: skip */
		if (!mask_port)
			continue;

		if (path_curr->sd_curr == trace->sd_src)
			__set_bit(lpn, trace->port_mask_ingress);

		/* this path is now local to this port; update the dfid mask */
		path_curr->dfid_mask = mask_port;

		/*
		 * if all FIDs have been assigned to a port: done.
		 * otherwise, split off a new path for the remaining FIDs.
		 */
		mask_remaining &= ~mask_port;
		if (mask_remaining) {
			path_next = &trace->paths[trace->num_paths];
			trace->num_paths++;

			/* we should never generate more paths than we have FIDs */
			WARN(trace->num_paths > ROUTING_FID_BLOCK_SIZE,
			     "traced paths exceeds FID block size");

			*path_next = *path_curr;
			path_next->dfid_mask = mask_remaining;
		}

		/*
		 * extending the path mutates the curr pointer.
		 * do so after cloning the next path
		 */
		extend_path_one(trace, path_curr, port);
		if (!mask_remaining)
			break;

		path_curr = path_next;
	}

	if (mask_remaining) {
		/* remaining FIDs were not assigned to a port; fail the path */
		path_curr->hops = 0;
		path_curr->done = true;
		return;
	}
}

/**
 * union_sd2sd - Returns the union of two link connectivities.
 * @dst: The first connectivity, which will contain the result.
 * @other: The second connectivity.
 *
 * Return: The unified link connectivity.
 */
static void union_sd2sd(struct sd2sd_info *dst, struct sd2sd_info *other)
{
	dst->bandwidth = min(dst->bandwidth, other->bandwidth);
	dst->latency = max(dst->latency, other->latency);
}

/**
 * find_egress - Determines the subdevice within the destination device that
 * traffic will arrive on (fabric egress) for a given src/dst subdevice pair.
 * @src: The source subdevice.
 * @dst: The destination subdevice.
 *
 * Return: The egress subdevice within the destination device, or NULL if not
 * found.
 */
static struct fsubdev *find_egress(struct fsubdev *src, struct fsubdev *dst)
{
	struct fsubdev *ingress;
	u16 dpa = dst->routing.dpa_idx_base;
	u16 dfid;
	int i;

	/* driver only ever sets bits 9:0 of the DFID */
	WARN_ON(src->routing.dfid_map->dfid[dpa] > 0x3ff);

	dfid = (u16)(src->routing.dfid_map->dfid[dpa] << ROUTING_DPA_DFID_MAP_SHIFT);

	for (i = 0; i < dst->fdev->pd->sd_cnt; ++i) {
		ingress = &dst->fdev->sd[i];
		if (ingress->routing.fid_base == dfid)
			return ingress;
	}

	return NULL;
}

/**
 * reduce_bandwidth - Reduces the bandwidth of a set of paths down to a
 * single bandwidth.
 * @trace: The trace to operate on.
 *
 * Return: The bandwidth in Gbps.
 *
 * Note that this only considers the bandwidth of the links that ingress
 * traffic into the fabric from the source subdevice, and the links that
 * egress traffic from the fabric at the destination subdevice.  If paths
 * are longer than two hops, the intermediate hops are ignored.
 */
static u16 reduce_bandwidth(struct trace *trace)
{
	struct fport *port;
	u32 bw_ingress = 0;
	u32 bw_egress = 0;
	u8 lpn;

	for_each_set_bit(lpn, trace->port_mask_ingress, PORT_COUNT) {
		port = get_fport_handle(trace->sd_src, lpn);
		if (!port)
			continue;
		bw_ingress += get_bandwidth_link(port);
	}

	for_each_set_bit(lpn, trace->port_mask_egress, PORT_COUNT) {
		port = get_fport_handle(trace->sd_dst, lpn);
		if (!port)
			continue;
		bw_egress += get_bandwidth_link(port);
	}

	/* 90 Gps * 4x * 8 fabric ports == 2880 max, which fits a u16 */
	return (u16)(min(bw_ingress, bw_egress) / 10);
}

/**
 * reduce_latency - Reduces the latency of a set of paths down to a single
 * average latency.
 * @trace: The trace to operate on.
 *
 * Return: The latency in tenths of hops.
 */
static u16 reduce_latency(struct trace *trace)
{
	struct trace_path *path;
	u32 l = 0;
	u8 i;

	/* cannot be more than 32 paths per trace */
	WARN_ON(trace->num_paths > 32);
	for (i = 0; i < trace->num_paths; ++i) {
		path = &trace->paths[i];
		if (!path->hops)
			return 0;

		/* weighted by DFIDs per path */
		l += path->hops * hweight64(path->dfid_mask);
	}

	/* dfid masks are disjoint per path */
	WARN_ON(l > 255 * 64);

	/* convert latency to tenths and divide out the DFID weight */
	return (u16)(l * 10 / ROUTING_VALID_FIDS_PER_BLOCK);
}

/**
 * reduce - Reduces all traced paths in a trace to a single output result.
 * @trace: The trace to operate on.
 * @dst: The output in which to store results.
 */
static void reduce(struct trace *trace, struct sd2sd_info *dst)
{
	dst->latency = reduce_latency(trace);
	dst->bandwidth = dst->latency ? reduce_bandwidth(trace) : 0;
}

/**
 * trace - Traces all routes between endpoints to determine the
 * available bandwidth/latency.
 * @sd_src: The source subdevice.
 * @sd_dst: The destination subdevice.
 * @si: The output populate with results.
 *
 * Return: 0 on success, non-zero otherwise.
 *
 * Assumes sd_src != sd_dst.
 */
static int trace(struct fsubdev *sd_src, struct fsubdev *sd_dst,
		 struct sd2sd_info *si)
{
	struct trace_path *path;
	struct trace *trace;
	u8 num_paths;
	bool done;
	int limit;
	int pass;
	u8 i;

	if (routing_cost_lookup(sd_src, sd_dst) == ROUTING_COST_INFINITE)
		return -EINVAL;

	trace = kzalloc(sizeof(*trace), GFP_KERNEL);
	if (!trace)
		return -ENOMEM;

	trace->sd_src = sd_src;
	trace->sd_dst = sd_dst;
	trace->num_paths = 1;
	trace->paths[0].sd_curr = sd_src;
	trace->paths[0].dfid_mask = ROUTING_FID_BLOCK_MASK;

	/*
	 * limit iterations arbitrarily to prevent a broken fabric routing
	 * with cycles from inducing a runaway trace
	 */
	limit = ROUTING_FID_BLOCK_SIZE * ROUTING_HOP_LIMIT;
	for (pass = 0; pass < limit; ++pass) {
		/*
		 * although each pass may add paths, they will already have
		 * been processed; save off the initial count
		 */
		num_paths = trace->num_paths;

		done = true;

		for (i = 0; i < num_paths; ++i) {
			path = &trace->paths[i];

			if (path->done)
				continue;

			extend_path(trace, path);

			if (!path->done)
				done = false;
		}

		if (done)
			break;
	}

	if (pass >= limit) {
		pr_err("%s: iterations exceeded (possible cyclic fabric routing)",
		       __func__);
		kfree(trace);
		return -EINVAL;
	}

	reduce(trace, si);

	kfree(trace);

	return 0;
}

/**
 * p2p_sd - calculates the connectivity between subdevices
 * @src: the source subdevice
 * @dst: the destination subdevice
 * @si: the output connectivity info structure
 *
 * This first determines the EGRESS subdev (i.e. the subdev at which traffic
 * egresses the fabric), and traces in both directions between SRC and EGRESS.
 *
 * If SRC and DST are on the same plane, then EGRESS == DST, as in:
 *
 * -------------                      --------------------
 * | 0.0 (SRC) |---(fabric plane 0)---| 1.0 (EGRESS/DST) |
 * -------------                      --------------------
 * | 0.1       |---(fabric plane 1)---| 1.1              |
 * -------------                      --------------------
 *
 * If SRC and DST they're on different planes, EGRESS will be the tile at the
 * destination that shares a plane with SRC, as in:
 *
 * -------------                      ----------------
 * | 0.0 (SRC) |---(fabric plane 0)---| 1.0 (EGRESS) |
 * -------------                      ----------------
 * | 0.1       |---(fabric plane 1)---| 1.1 (DST)    |
 * -------------                      ----------------
 *
 * The reason we trace SRC/EGRESS rather than SRC/DST is that if we're routing
 * from SRC to DST in the second diagram, a request takes:
 *   0.0 -> plane 0 -> 1.0 -> 1.1
 * and a response will retrace the request path, and thus go:
 *   1.1 -> 1.0 -> plane 0 -> 0.0
 *
 * Responses might not travel through plane 0 the same way requests did (hence
 * needing to trace the paths in the driver), but the point is they WILL NOT
 * take the path:
 *   1.1 -> plane 1 -> 0.1 -> 0.0
 */
static void p2p_sd(struct fsubdev *src, struct fsubdev *dst,
		   struct sd2sd_info *si)
{
	struct fsubdev *egress;
	struct sd2sd_info si_reverse;

	if (src->fdev == dst->fdev)
		goto zero;

	if (list_empty(&src->routable_link) || list_empty(&dst->routable_link))
		goto zero;

	if (routing_sd_is_error(src) || routing_sd_is_error(dst))
		goto zero;

	egress = find_egress(src, dst);
	if (!egress)
		goto zero;

	if (trace(src, egress, si))
		goto zero;

	if (trace(egress, src, &si_reverse))
		goto zero;

	union_sd2sd(si, &si_reverse);
	return;

zero:
	si->bandwidth = 0;
	si->latency = 0;
}

static void p2p_dev(struct fdev *src, struct fdev *dst,
		    struct query_info *qi)
{
	struct fsubdev *sd_src, *sd_dst;
	struct sd2sd_info *si;
	u8 i, j;

	for (i = 0; i < qi->src_cnt; ++i) {
		sd_src = &src->sd[i];
		for (j = 0; j < qi->dst_cnt; ++j) {
			sd_dst = &dst->sd[j];
			si = &qi->sd2sd[i * qi->dst_cnt + j];
			p2p_sd(sd_src, sd_dst, si);
		}
	}
}

/*
 * this may be used on the "current" rcu entry, or the "next" non-rcu entry.
 * use __force to strip __rcu-ness to appease the checker in both cases,
 * given that this will never be invoked on the rcu entry except on device
 * teardown.
 */
static void p2p_entry_clear(__force struct routing_p2p_entry **entry)
{
	struct query_info *qi;
	unsigned long i;

	xa_for_each(&(*entry)->devs, i, qi)
		kfree(qi);
	xa_destroy(&(*entry)->devs);
	kfree(*entry);
	*entry = NULL;
}

static void cache_pair(struct fdev *src, struct fdev *dst)
{
	struct query_info *qi;

	qi = kmalloc(struct_size(qi, sd2sd, src->pd->sd_cnt * dst->pd->sd_cnt),
		     GFP_KERNEL);
	if (!qi)
		return;

	qi->src_cnt = src->pd->sd_cnt;
	qi->dst_cnt = dst->pd->sd_cnt;

	p2p_dev(src, dst, qi);

	xa_store(&src->p2p_next->devs, dst->pd->index, qi, GFP_KERNEL);
}

static int cache_src_cb(struct fdev *dst, void *src)
{
	cache_pair((struct fdev *)src, dst);

	return 0;
}

static void cache_src(struct fdev *src)
{
	fdev_process_each(cache_src_cb, src);
}

static int p2p_entry_alloc_cb(struct fdev *dev, void *args)
{
	dev->p2p_next = kmalloc(sizeof(*dev->p2p_next), GFP_KERNEL);
	if (dev->p2p_next) {
		xa_init(&dev->p2p_next->devs);
		cache_src(dev);

		dev->p2p_next = rcu_replace_pointer(dev->p2p, dev->p2p_next,
						    true);
	}

	return 0;
}

static int p2p_entry_clear_cb(struct fdev *dev, void *args)
{
	if (dev->p2p_next)
		p2p_entry_clear(&dev->p2p_next);

	return 0;
}

/**
 * routing_p2p_cache - Updates cached peer-to-peer connectivity queries
 * for all devices.
 *
 * Invoked with the routing lock held, preserving the lifetime of the devices
 * in the xarray.
 *
 * Note that devices may contain subdevices that are not routable (i.e. not
 * on the routable list) or in an error state, which the connectivity query
 * logic must account for.
 */
void routing_p2p_cache(void)
{
	fdev_process_each(p2p_entry_alloc_cb, NULL);

	synchronize_rcu();

	fdev_process_each(p2p_entry_clear_cb, NULL);
}

static void p2p_zero(struct fdev *src, struct fdev *dst, struct query_info *qi)
{
	u16 i, n;

	qi->src_cnt = src->pd->sd_cnt;
	qi->dst_cnt = dst->pd->sd_cnt;

	/* i,n (u16) can hold UINT8_MAX*UINT8_MAX+1 */
	for (i = 0, n = qi->src_cnt * qi->dst_cnt; i < n; ++i) {
		qi->sd2sd[i].bandwidth = 0;
		qi->sd2sd[i].latency = 0;
	}
}

void routing_p2p_lookup(struct fdev *src, struct fdev *dst,
			struct query_info *qi)
{
	struct routing_p2p_entry *src_entry;
	struct query_info *qi_cached;

	rcu_read_lock();

	src_entry = rcu_dereference(src->p2p);
	if (!src_entry) {
		rcu_read_unlock();
		p2p_zero(src, dst, qi);
		return;
	}

	qi_cached = xa_load(&src_entry->devs, dst->pd->index);
	if (!qi_cached) {
		p2p_zero(src, dst, qi);
	} else if (qi->src_cnt != qi_cached->src_cnt ||
		   qi->dst_cnt != qi_cached->dst_cnt) {
		p2p_zero(src, dst, qi);
	} else {
		memcpy(qi->sd2sd, qi_cached->sd2sd,
		       sizeof(*qi->sd2sd) * qi->src_cnt * qi->dst_cnt);
	}

	rcu_read_unlock();
}

void routing_p2p_clear(struct fdev *dev)
{
	if (dev->p2p)
		p2p_entry_clear(&dev->p2p);

	if (dev->p2p_next)
		p2p_entry_clear(&dev->p2p_next);
}

/**
 * struct debugfs_p2p_iter - Represents a pairwise iteration of all base
 * tile pairs (tile 0).
 * @src: the current source tile
 * @dst: the current destination tile
 *
 * Given N devices, this produces N^2 entries, one for every pair of source
 * and destination subdevice with index 0.
 */
struct debugfs_p2p_iter {
	struct fsubdev *src, *dst;
};

/**
 * debugfs_p2p_iter_end - Destroys the iteration context.
 * @iter: the seq iterator
 *
 * The routing lock was taken when the iteration began to prevent the routing
 * list from being modified across multiple userspace reads of the debugfs
 * interface.  All paths out of the seq interface end in this call, which
 * releases the lock.
 */
static void debugfs_p2p_iter_end(struct debugfs_p2p_iter *iter)
{
	up_read(&routable_lock);
	kfree(iter);
}

/**
 * debugfs_p2p_seq_start - Initialize an iterator for the seq interface.
 * @s: the seq interface
 * @pos: the read position being requested
 *
 * Return: a new iterator, or NULL on failure
 *
 * For simplicity, we require the read begin at offset 0.
 *
 * We also require the routing lock be held during the seq lifetime to
 * prevent invalidation of the iteration over the routable device list.
 */
static void *debugfs_p2p_seq_start(struct seq_file *s, loff_t *pos)
{
	struct debugfs_p2p_iter *iter;

	if (*pos)
		return NULL;

	iter = kmalloc(sizeof(*iter), GFP_KERNEL);
	if (!iter)
		return NULL;

	down_read(&routable_lock); /* shared lock */

	iter->src = routing_sd_iter(0);
	if (!iter->src) {
		debugfs_p2p_iter_end(iter);
		return NULL;
	}

	iter->dst = iter->src;

	return iter;
}

/**
 * debugfs_p2p_iter_inc - Increment the p2p debugfs seq file iterator.
 * @iter: The iterator to operate on.
 *
 * Return: The incremented iterator, or NULL if terminated.
 *
 * This effectively enumerates devices, as it skips over subdevices that
 * are not index 0.
 */
static void *debugfs_p2p_iter_inc(struct debugfs_p2p_iter *iter)
{
	do {
		iter->dst = routing_sd_next(iter->dst, 0);
		if (!iter->dst) {
			iter->src = routing_sd_next(iter->src, 0);
			if (!iter->src) {
				debugfs_p2p_iter_end(iter);
				return NULL;
			}

			iter->dst = routing_sd_iter(0);
			if (!iter->dst) {
				debugfs_p2p_iter_end(iter);
				return NULL;
			}
		}
	} while (sd_index(iter->src) || sd_index(iter->dst));

	return iter;
}

static void *debugfs_p2p_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	struct debugfs_p2p_iter *iter = v;

	(*pos)++;
	return debugfs_p2p_iter_inc(iter);
}

static void debugfs_p2p_seq_stop(struct seq_file *s, void *v)
{
	if (v)
		debugfs_p2p_iter_end(v);
}

static int debugfs_p2p_seq_show(struct seq_file *s, void *v)
{
	struct debugfs_p2p_iter *iter = v;
	struct query_info *qi;
	struct sd2sd_info *si;
	u8 src_cnt = iter->src->fdev->pd->sd_cnt;
	u8 dst_cnt = iter->dst->fdev->pd->sd_cnt;
	u8 i, j;

	qi = kmalloc(struct_size(qi, sd2sd, src_cnt * dst_cnt), GFP_KERNEL);
	if (!qi)
		return -ENOMEM;

	qi->src_cnt = src_cnt;
	qi->dst_cnt = dst_cnt;
	routing_p2p_lookup(iter->src->fdev, iter->dst->fdev, qi);

	for (i = 0; i < src_cnt; ++i) {
		for (j = 0; j < dst_cnt; ++j) {
			si = &qi->sd2sd[i * dst_cnt + j];
			seq_printf(s, "%u %u %u %u %u %u\n",
				   iter->src->fdev->pd->index, i,
				   iter->dst->fdev->pd->index, j,
				   si->bandwidth, si->latency);
		}
	}

	kfree(qi);
	return 0;
}

static const struct seq_operations debugfs_p2p_seq_ops = {
	.start = debugfs_p2p_seq_start,
	.next = debugfs_p2p_seq_next,
	.stop = debugfs_p2p_seq_stop,
	.show = debugfs_p2p_seq_show,
};

static int debugfs_p2p_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &debugfs_p2p_seq_ops);
}

static const struct file_operations debugfs_ops = {
	.open = debugfs_p2p_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

#define DEBUGFS_NAME_P2P "p2p"

void routing_p2p_init(struct dentry *root)
{
	debugfs_create_file(DEBUGFS_NAME_P2P, 0400, root, NULL, &debugfs_ops);
}
