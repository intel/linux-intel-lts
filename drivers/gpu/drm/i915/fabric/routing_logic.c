// SPDX-License-Identifier: MIT
/*
 * Copyright(c) 2019 - 2021 Intel Corporation.
 *
 */

#include <linux/bitmap.h>
#include <linux/bitfield.h>
#include <linux/version.h>

#include "csr.h"
#include "routing_logic.h"
#include "trace.h"

/*
 * Returns the bridge endpoint port index corresponding to the FID.
 */
static u8 fid_to_bridge_offset(u16 fid)
{
	return FIELD_GET(ROUTING_FID_MDFI_MASK, fid);
}

/*
 * Returns the cost between the specified source and destination subdevices.
 *
 * Returns ROUTING_COST_INFINITE if the two do not share a plane.
 */
u16 routing_cost_lookup(struct fsubdev *sd_src, struct fsubdev *sd_dst)
{
	u16 i, j, n;

	if (!sd_src->routing.plane ||
	    sd_src->routing.plane != sd_dst->routing.plane)
		return ROUTING_COST_INFINITE;

	i = sd_src->routing.plane_index;
	j = sd_dst->routing.plane_index;
	n = sd_src->routing.plane->num_subdevs;

	return sd_src->routing.plane->cost[i * n + j];
}

static void validate_routing_policy(struct routing_topology *topo,
				    struct fsubdev *sd)
{
	switch (topo->policy) {
	case ROUTING_POLICY_ROUTE_THROUGH:
	case ROUTING_POLICY_RESILIENCY:
		if (IS_ANR_STEP(sd, ANR_ARI_STEP_A0, ANR_ARI_STEP_A_LAST)) {
			pr_err(DEV_SD_FMT
			       "ASIC A-series rev is incompatible with the selected routing policy (%d)\n",
			       sd->fdev->pd->index, sd_index(sd), topo->policy);
			routing_sd_transition_error(sd);
		}
		break;
	case ROUTING_POLICY_DEFAULT:
		break;
	}
}

/**
 * init - Perform top-level scan of subdevices and initialize any necessary
 * state.
 * @topo: The topology to operate on.
 */
static void init(struct routing_topology *topo)
{
	struct fsubdev *sd;
	struct fport *port;
	u16 sd_max;
	u8 lpn;

	topo->max_dpa_index = 0;

	for (sd = routing_sd_iter(0); sd; sd = routing_sd_next(sd, 0)) {
		/* routing should never be running against a debug device */
		if (WARN_ON(dev_is_runtime_debug(sd->fdev)))
			continue;

		pr_debug("device %u sd %u dpa_base 0x%04x dpa_range 0x%04x\n",
			 sd->fdev->pd->index, sd_index(sd),
			 sd->routing.dpa_idx_base, sd->routing.dpa_idx_range);

		sd_max = sd->routing.dpa_idx_base +
			 sd->routing.dpa_idx_range - 1;

		if (topo->max_dpa_index < sd_max)
			topo->max_dpa_index = sd_max;

		for_each_fabric_port(port, lpn, sd)
			port->routing.routable = port_is_routable(port);

		validate_routing_policy(topo, sd);
	}
}

/*
 * Establishes the cached neighbor pointer for a single fabric port.
 */
static void process_neighbor(struct fsubdev *sd_src,
			     struct fport *port_src)
{
	struct fsubdev *sd_dst;
	struct fport *port_dst;
	u64 nguid;

	port_src->routing.neighbor = NULL;

	if (!port_is_routable(port_src))
		return;

	if (!port_src->portinfo)
		return;

	nguid = port_src->portinfo->neighbor_guid;
	if (!nguid || sd_src->guid == nguid)
		return;

	sd_dst = find_routable_sd(nguid);
	if (!sd_dst)
		return;

	port_dst = get_fport_handle(sd_dst,
				    port_src->portinfo->neighbor_port_number);
	if (!port_dst)
		return;

	if (!port_is_routable(port_dst))
		return;

	if (!port_dst->portinfo)
		return;
	if (port_dst->portinfo->neighbor_guid != sd_src->guid)
		return;
	if (port_dst->portinfo->neighbor_port_number != port_src->lpn)
		return;

	port_src->routing.neighbor = port_dst;
}

/*
 * Establishes the cached neighbor pointers across all sd fabric ports.
 */
static void process_neighbors(void)
{
	struct fsubdev *sd;
	struct fport *port;
	u8 lpn;

	for (sd = routing_sd_iter(0); sd; sd = routing_sd_next(sd, 0))
		for_each_fabric_port(port, lpn, sd)
			process_neighbor(sd, port);
}

static struct routing_plane *plane_alloc(struct routing_topology *topo)
{
	struct routing_plane *plane = kzalloc(sizeof(*plane), GFP_KERNEL);

	if (!plane)
		return NULL;

	INIT_LIST_HEAD(&plane->topo_link);
	INIT_LIST_HEAD(&plane->sd_list);
	plane->index = topo->num_planes;
	plane->topo = topo;

	list_add_tail(&plane->topo_link, &topo->plane_list);
	topo->num_planes++;

	return plane;
}

/**
 * routing_plane_destroy - Destroys and deallocs the specified plane.
 * @plane: The plane to destroy.
 *
 * Tiles that were members of the plane have their plane assignment cleared,
 * but otherwise remain in their present state.
 */
void routing_plane_destroy(struct routing_plane *plane)
{
	struct fsubdev *sd, *sd_next;

	list_for_each_entry_safe(sd, sd_next, &plane->sd_list,
				 routing.plane_link) {
		list_del_init(&sd->routing.plane_link);
		sd->routing.plane = NULL;
		sd->routing.plane_index = 0;
	}

	list_del(&plane->topo_link);
	plane->topo->num_planes--;

	kfree(plane->cost);
	kfree(plane);
}

/**
 * plane_fail - Takes all subdevices in a plane to their error state, then
 * destroys the plane.
 * @plane: The plane to operate on.
 */
static void plane_fail(struct routing_plane *plane)
{
	struct fsubdev *sd, *sd_next;

	/* _safe since the error trasitions remove subdevices from the plane */
	list_for_each_entry_safe(sd, sd_next, &plane->sd_list,
				 routing.plane_link)
		routing_sd_transition_error(sd);

	routing_plane_destroy(plane);
}

static void propagate_plane_entry(struct list_head *queue,
				  struct routing_plane *plane,
				  struct fsubdev *sd)
{
	struct fsubdev *sd_neighbor;
	struct fport *port, *port_neighbor;
	u8 lpn;

	list_add_tail(&sd->routing.plane_link, &plane->sd_list);
	sd->routing.plane_index = sd->routing.plane->num_subdevs++;

	for_each_fabric_port(port, lpn, sd) {
		port_neighbor = neighbor_of(port);
		if (!port_neighbor)
			continue;

		sd_neighbor = port_neighbor->sd;
		if (routing_sd_is_error(sd_neighbor) ||
		    sd_neighbor->routing.plane)
			continue;

		sd_neighbor->routing.plane = plane;
		list_add_tail(&sd_neighbor->routing.plane_link, queue);
	}
}

/**
 * propagate_plane - Propagates a plane to the specified sd's neighbors in a
 * breadth-first manner.
 * @plane: The current plane being searched.
 * @root: The root sd of the search.
 *
 * Uses whether a given sd has a plane assigned as an indicator of whether it
 * has been visited already.
 */
static void propagate_plane(struct routing_plane *plane,
			    struct fsubdev *root)
{
	struct fsubdev *sd;
	LIST_HEAD(queue);

	trace_rt_plane(root);

	list_add_tail(&root->routing.plane_link, &queue);

	while (!list_empty(&queue)) {
		sd = list_first_entry(&queue, struct fsubdev,
				      routing.plane_link);
		list_del_init(&sd->routing.plane_link);

		pr_debug("plane %u device %u sd %u\n",
			 sd->routing.plane->index, sd->fdev->pd->index,
			 sd_index(sd));

		propagate_plane_entry(&queue, plane, sd);
	}
}

/*
 * build_planes - Dynamically calculates the planes that exist on the fabric.
 *
 * It does this by a single linear top-level pass over all subdevices.  If the
 * subdevice is unassigned to a plane, it allocates a new plane, and then
 * propagates that plane to all of that subdevice's neighbors transitively by a
 * breadth-first traversal.
 *
 * After propagating a plane, all subdevices on that plane have been visited
 * and assigned, and all subdevices still unassigned correspond to new planes.
 */
static int build_planes(struct routing_topology *topo)
{
	struct routing_plane *plane, *plane_next;
	struct fsubdev *sd;

	list_for_each_entry_safe(plane, plane_next, &topo->plane_list,
				 topo_link)
		routing_plane_destroy(plane);

	if (WARN_ON(topo->num_planes))
		return -EINVAL;

	for (sd = routing_sd_iter(0); sd; sd = routing_sd_next(sd, 0)) {
		if (sd->routing.plane)
			continue;

		sd->routing.plane = plane_alloc(topo);
		if (!sd->routing.plane) {
			routing_sd_transition_error(sd);
			return -ENOMEM;
		}

		propagate_plane(sd->routing.plane, sd);
	}

	return 0;
}

/**
 * compute_cost_for_plane - Computes a cost matrix for the given plane.
 * @plane: The plane to operate on.
 *
 * This is an N^2 matrix that encodes the cost between any source and
 * destination sd by "plane index", or the per-plane index assigned to
 * each sd during plane creation.
 *
 * Cost is currently hop distance, but could in the future be weighted
 * by link width/speed, LQI, or other factors.
 *
 * Return: Zero on success, non-zero otherwise.
 */
static int compute_cost_for_plane(struct routing_plane *plane)
{
	struct fsubdev *sd;
	struct fport *port, *port_neighbor;
	u16 n = plane->num_subdevs;
	size_t cost_size = sizeof(*plane->cost) * n * n;
	u16 i, j, k;
	u32 ij, ik, kj;
	u16 value;
	u8 lpn;

	plane->cost = kmalloc(cost_size, GFP_KERNEL);
	if (!plane->cost)
		return -ENOMEM;

	memset(plane->cost, 0xff, cost_size);

	/* generate a single-hop cost matrix */
	list_for_each_entry(sd, &plane->sd_list, routing.plane_link) {
		i = sd->routing.plane_index;
		plane->cost[i * n + i] = 0;
		for_each_fabric_port(port, lpn, sd) {
			port_neighbor = neighbor_of(port);
			if (!port_neighbor)
				continue;

			j = port_neighbor->sd->routing.plane_index;
			plane->cost[i * n + j] = 1;
		}
	}

	/*
	 * relax single-hop to multi-hop all-pairs shortest-path via
	 * floyd-warshall
	 */
	for (k = 0; k < n; ++k) {
		for (i = 0; i < n; ++i) {
			if (i == k)
				continue;
			/* ik (u32) can hold UINT16_MAX*UINT16_MAX+UINT16_MAX */
			ik = i * n + k;
			if (plane->cost[ik] == ROUTING_COST_INFINITE)
				continue;
			for (j = 0; j < n; ++j) {
				if (j == k || i == j)
					continue;
				/* kj (u32) can hold UINT16_MAX*UINT16_MAX+UINT16_MAX */
				kj = k * n + j;
				if (plane->cost[kj] == ROUTING_COST_INFINITE)
					continue;
				/* ij (u32) can hold UINT16_MAX*UINT16_MAX+UINT16_MAX */
				ij = i * n + j;
				value = plane->cost[ik] + plane->cost[kj];
				if (plane->cost[ij] > value)
					plane->cost[ij] = value;
			}
		}
	}

	return 0;
}

static void compute_costs(struct routing_topology *topo)
{
	struct routing_plane *plane, *plane_next;
	int err;

	list_for_each_entry_safe(plane, plane_next, &topo->plane_list,
				 topo_link) {
		err = compute_cost_for_plane(plane);
		if (err) {
			pr_err("%s: failed to compute cost matrix for plane %u: %d\n",
			       __func__, plane->index, err);
			plane_fail(plane);
		}
	}
}

static void set_dpa_lut(struct fsubdev *sd_src, struct fsubdev *sd_dst,
			u16 dfid)
{
	u16 start = sd_dst->routing.dpa_idx_base;
	/* end (u32) can hold UINT16_MAX+UINT16_MAX */
	u32 end = start + sd_dst->routing.dpa_idx_range;
	u32 i;

	/* i (u32) can hold UINT16_MAX+UINT16_MAX+1 */
	for (i = start; i < end; ++i) {
		sd_src->routing.dfid_map_next->dfid[i] =
			dfid >> ROUTING_DPA_DFID_MAP_SHIFT;
		pr_debug("device %u sd %u to device %u sd %u: dpa_idx %d dfid 0x%04x\n",
			 sd_src->fdev->pd->index, sd_index(sd_src),
			 sd_dst->fdev->pd->index, sd_index(sd_dst), i, dfid);
	}
}

static void map_addresses_for_sd_dst(struct fsubdev *sd_src,
				     struct fsubdev *sd_dst)
{
	struct fdev *dev_dst;
	struct fsubdev *neighbor;
	u8 i;

	/*
	 * same-package addresses are invalid.
	 * hw has no support for bridge->bridge data path, and we do not
	 * support routing tiles within a package.
	 */
	if (sd_src->fdev == sd_dst->fdev) {
		set_dpa_lut(sd_src, sd_dst, 0);
		return;
	}

	/* directly accessibile addresses route normally */
	if (routing_cost_lookup(sd_src, sd_dst) != ROUTING_COST_INFINITE) {
		set_dpa_lut(sd_src, sd_dst, sd_dst->routing.fid_base);
		return;
	}

	/*
	 * indirectly accessibile dpas route via directly acessible
	 * subdevices on the same package
	 */
	dev_dst = sd_dst->fdev;
	for (i = 0; i < dev_dst->pd->sd_cnt; ++i) {
		neighbor = &dev_dst->sd[i];
		if (sd_dst == neighbor)
			continue;
		if (routing_sd_is_error(neighbor))
			continue;
		if (routing_cost_lookup(sd_src, neighbor) == ROUTING_COST_INFINITE)
			continue;

		set_dpa_lut(sd_src, sd_dst, neighbor->routing.fid_base);
		return;
	}
}

/**
 * dfid_map_alloc - Allocates and initializes the DPA->DFID map.
 * @topo: The topology to operate on.
 *
 * Return: A pointer to the dfid map struct, or NULL otherwise.
 */
static struct routing_dfid_map *dfid_map_alloc(struct routing_topology *topo)
{
	struct routing_dfid_map *dfid_map;
	u16 dfid_count = topo->max_dpa_index + 1;

	dfid_map = kzalloc(struct_size(dfid_map, dfid, dfid_count),
			   GFP_KERNEL);
	if (!dfid_map)
		return NULL;

	dfid_map->size = dfid_count;

	return dfid_map;
}

/**
 * map_addresses_for_sd - Build the DPA-to-DFID map.
 * @topo: The topology to operate on.
 * @sd_src: The source sd.
 *
 * For each source/dest pair, if the destination is accessible according to
 * the routing rules, map the destination's address to the destination's DFID.
 * If the destination is not accessible, attempt to find the first sd on the
 * destination device that is, and use it instead.
 *
 * Currently specific to point-to-point without route-through, where
 * "accessible" is defined as reachable in a single hop.
 */
static void map_addresses_for_sd(struct routing_topology *topo,
				 struct fsubdev *sd_src)
{
	struct fsubdev *sd_dst;
	struct routing_dfid_map *dfid_map;

	dfid_map = dfid_map_alloc(topo);
	if (!dfid_map) {
		routing_sd_transition_error(sd_src);
		return;
	}

	sd_src->routing.dfid_map_next = dfid_map;

	for (sd_dst = routing_sd_iter(0); sd_dst;
	     sd_dst = routing_sd_next(sd_dst, 0))
		map_addresses_for_sd_dst(sd_src, sd_dst);
}

/**
 * map_addresses - Build the DPA-to-DFID maps for all devices.
 * @topo: The topology to operate on.
 */
static void map_addresses(struct routing_topology *topo)
{
	struct fsubdev *sd;

	for (sd = routing_sd_iter(0); sd; sd = routing_sd_next(sd, 0))
		map_addresses_for_sd(topo, sd);
}

/**
 * find_next_port - Finds the next applicable output port for routing
 * table generation.
 * @sd_src: source subdevice
 * @sd_dst: destination subdevice
 * @sel: UFT source selector
 * @ports: array of ports to select from
 * @num_ports: length of the ports array
 * @curr_port: input/output param indicating the current port array index
 * @reuse_port: whether to resuse the last used port (A series modulo WA)
 *
 * Return: The next available port to use for routing table generation.
 *
 * Port selection assumes the incoming port list already eliminates
 * routes not valid for the policy.
 *
 * Port selection by policy:
 *   DEFAULT:
 *     round robin
 *   RESILIENCY:
 *     route fabric source via direct only
 *     route bridge source:
 *       direct if possible
 *       else round robin
 *   ROUTE_THROUGH:
 *     route fabric source via direct only
 *     route bridge source:
 *       round robin
 */
static u8 find_next_port(struct fsubdev *sd_src, const struct fsubdev *sd_dst, enum uft_sel sel,
			 const u8 *ports, u8 num_ports, u8 *curr_port, bool reuse_port)
{
	struct fport *port;
	u8 first_port = *curr_port;
	u8 lpn;

	if (reuse_port)
		return ports[*curr_port];

	do {
		lpn = ports[*curr_port];
		*curr_port = (*curr_port + 1) % num_ports;
		switch (sel) {
		case UFT_SEL_ORIGIN:
			/* from bridge: allow to any port in the list */
			return lpn;
		case UFT_SEL_HOP:
			/* from fabric: allow only to direct destinations */
			if (test_bit(lpn, sd_src->bport_lpns))
				return lpn;
			port = get_fport_handle(sd_src, lpn);
			if (!port || !port->routing.neighbor)
				continue;
			if (port->routing.neighbor->sd == sd_dst)
				return lpn;
			break;
		case UFT_SEL_BOTH:
		default:
			/* no support for routing BOTH using port lists */
			return ROUTING_UFT_INVALID_PORT4;
		}
	} while (*curr_port != first_port);

	return ROUTING_UFT_INVALID_PORT4;
}

/**
 * unicast_route_pair_once - Programs a specific DFID offset using the
 *   specified context, and updates the index state accordingly.
 * @sd_src: the source subdevice
 * @sd_dst: the destination subdevice
 * @block: the UFT block being routed
 * @ports: the array of sd_src ports being routed over
 * @num_ports: number of valid ports in the ports array
 * @idx_origin: the current index into the ports list for the origin uft
 * @idx_hop: the current index into the ports list for the hop uft
 * @dfid_base: the dfid corresponding to block offset 0
 * @dfid_offset: the uft block offset being programmed
 * @reuse_port: whether to resuse the last used port (A series modulo WA)
 */
static void unicast_route_pair_once(struct fsubdev *sd_src, struct fsubdev *sd_dst, u8 *block,
				    u8 *ports, u8 num_ports, u8 *idx_origin, u8 *idx_hop,
				    u16 dfid_base, u16 dfid_offset, bool reuse_port)
{
	u8 lpn_origin;
	u8 lpn_hop;

	lpn_origin = find_next_port(sd_src, sd_dst, UFT_SEL_ORIGIN,
				    ports, num_ports, idx_origin, reuse_port);
	if (lpn_origin == ROUTING_UFT_INVALID_PORT4)
		return;

	routing_uft_entry_set(block, UFT_SEL_ORIGIN, dfid_offset, lpn_origin);

	lpn_hop = find_next_port(sd_src, sd_dst, UFT_SEL_HOP,
				 ports, num_ports, idx_hop, reuse_port);
	if (lpn_hop == ROUTING_UFT_INVALID_PORT4)
		return;

	routing_uft_entry_set(block, UFT_SEL_HOP, dfid_offset, lpn_hop);

	trace_rt_pair(sd_src->fdev->pd->index, sd_index(sd_src),
		      dfid_base + dfid_offset, lpn_origin, lpn_hop);
}

/**
 * unicast_route_pair_via - Route all remote DFIDs of the dest device via a
 * specific list of egress ports.
 * @uft: The unicast forwarding table to update.
 * @sd_src: The source sd being routed.
 * @sd_dst: The destination sd being routed.
 * @ports: The array of outbound ports on the source that point to the
 *         destination.
 * @num_ports: The size of the ports array.
 *
 * Return: Zero on success; negative otherwise.
 */
static int unicast_route_pair_via(struct routing_uft *uft,
				  struct fsubdev *sd_src,
				  struct fsubdev *sd_dst,
				  u8 *ports, u8 num_ports)
{
	bool is_astep = IS_ANR_STEP(sd_src, ANR_ARI_STEP_A0, ANR_ARI_STEP_A_LAST);
	bool reuse_port;
	u8 *block;
	u16 fid;
	u8 mdfi;
	u8 path;
	u8 idx_origin = 0;
	u8 idx_hop = 0;

	if (WARN_ON(!num_ports))
		return -EINVAL;

	/* route the cport/mgmt fid */
	unicast_route_pair_once(sd_src, sd_dst, uft->mgmt, ports, num_ports,
				&idx_origin, &idx_hop, sd_dst->routing.fid_mgmt,
				sd_dst->routing.fid_mgmt - ROUTING_FID_CPORT_BASE, false);

	block = routing_uft_bridge_get(uft, sd_dst);
	if (!block)
		return -ENOMEM;

	/*
	 * reset the indices after routing the cport/mgmt fid, as we don't
	 * consider management traffic when balancing data traffic
	 */
	idx_origin = 0;
	idx_hop = 0;

	/*
	 * stripe FIDs over available ports
	 *
	 * assign the FIDs in channel order to prevent any one channel
	 * from not spreading out over the port space efficiently
	 */
	for (mdfi = 0; mdfi < ROUTING_FID_NUM_CHANNELS; ++mdfi) {
		for (path = 0; path < ROUTING_FID_NUM_PATHS; ++path) {
			/* fid as an offset from the base fid */
			fid = FIELD_PREP(ROUTING_FID_PATH_MASK, path) |
			      FIELD_PREP(ROUTING_FID_MDFI_MASK, mdfi);

			/*
			 * on ANR A0, FIDs will never use the highest path due
			 * to a HW modulus bug.
			 *
			 * we'll assign these FIDs, but skip factoring them
			 * into the accounting by simply not incrementing the
			 * index (i.e. the next fid will overlap it)
			 */
			reuse_port = is_astep && path == ROUTING_FID_NUM_PATHS - 1;

			unicast_route_pair_once(sd_src, sd_dst, block, ports,
						num_ports, &idx_origin, &idx_hop,
						sd_dst->routing.fid_base, fid, reuse_port);
		}
	}

	return 0;
}

/**
 * unicast_route_pair - Route all remote DFIDs of the dest device over fabric
 * ports of the source device.
 * @uft: The unicast forwarding table to update.
 * @sd_src: The source sd being routed.
 * @sd_dst: The destination sd being routed.
 *
 * Return: Zero on success; negative otherwise.
 */
static int unicast_route_pair(struct routing_uft *uft,
			      struct fsubdev *sd_src,
			      struct fsubdev *sd_dst)
{
	struct fport *port_src;
	struct fport *port_next;
	enum routing_policy policy = sd_src->routing.topo->policy;
	u8 ports[PORT_FABRIC_COUNT] = {};
	u8 indirect_ports[PORT_FABRIC_COUNT] = {};
	u8 num_ports = 0;
	u8 num_indirect = 0;
	u8 lpn;

	/* determine candidate ports to destination */
	for_each_fabric_port(port_src, lpn, sd_src) {
		port_next = neighbor_of(port_src);
		if (!port_next)
			continue;

		if (port_next->sd == sd_dst)
			ports[num_ports++] = lpn;
		else if (routing_cost_lookup(port_next->sd, sd_dst) == 1)
			indirect_ports[num_indirect++] = lpn;
	}

	/* leave unreachable destinations with invalid routes */
	if (!num_ports && !num_indirect) {
		pr_debug("no egress ports found: device %u sd %u to device %u sd %u\n",
			 sd_src->fdev->pd->index, sd_index(sd_src),
			 sd_dst->fdev->pd->index, sd_index(sd_dst));
		return 0;
	}

	/* a port should end up on one of the two lists; never both */
	if (WARN_ON(num_ports + num_indirect > PORT_FABRIC_COUNT))
		return -EINVAL;

	/* select/validate final port list based on routing policy */
	switch (policy) {
	case ROUTING_POLICY_ROUTE_THROUGH:
		/* route through uses direct and indirect paths freely */
		memcpy(ports + num_ports, indirect_ports, num_indirect);
		num_ports += num_indirect;
		break;
	case ROUTING_POLICY_RESILIENCY:
		/* resiliency only uses indirect if direct not available */
		if (!num_ports && num_indirect) {
			memcpy(ports, indirect_ports, num_indirect);
			num_ports = num_indirect;
		}
		break;
	case ROUTING_POLICY_DEFAULT:
	default:
		/* default relies on only direct paths */
		if (!num_ports) {
			pr_debug("no egress ports found: device %u sd %u to device %u sd %u\n",
				 sd_src->fdev->pd->index, sd_index(sd_src),
				 sd_dst->fdev->pd->index, sd_index(sd_dst));
			return 0;
		}
		break;
	}

	/* submit final non-empty port list to uft writer */
	return unicast_route_pair_via(uft, sd_src, sd_dst, ports, num_ports);
}

#if KERNEL_VERSION(6, 1, 0) > LINUX_VERSION_CODE
static int find_nth_bit(unsigned long *mask, unsigned long len, unsigned long n)
{
	int bit = find_first_bit(mask, len);

	if (bit == len)
		return len;

	while (n--) {
		bit = find_next_bit(mask, len, bit + 1);
		if (bit == len)
			return len;
	}

	return bit;
}
#endif

/**
 * unicast_route_local - Route the FIDs local to the sd via the bridge ports.
 * @uft: The unicast forwarding table to update.
 * @sd_src: The source sd being routed.
 *
 * Return: Zero on success; negative otherwise.
 */
static int unicast_route_local(struct routing_uft *uft, struct fsubdev *sd_src)
{
	u8 *block;
	u16 fid;
	u16 base;
	u8 bridge_offset;
	u8 port;

	routing_uft_entry_set(uft->mgmt, UFT_SEL_BOTH,
			      sd_src->routing.fid_mgmt -
			      ROUTING_FID_CPORT_BASE, 0);

	trace_rt_local(sd_src->fdev->pd->index, sd_index(sd_src),
		       sd_src->routing.fid_mgmt, 0, 0);

	block = routing_uft_bridge_get(uft, sd_src);
	if (!block)
		return -ENOMEM;

	for (fid = 0; fid < ROUTING_FID_BLOCK_SIZE; ++fid) {
		if (!is_valid_fid(fid)) {
			routing_uft_entry_set(block, UFT_SEL_BOTH, fid, ROUTING_UFT_INVALID_PORT4);
			continue;
		}

		base = sd_src->routing.fid_base;
		bridge_offset = fid_to_bridge_offset(base + fid);
		/* find_nth_bit() result will always be in range 0..PORT_COUNT */
		port = (u8)find_nth_bit(sd_src->bport_lpns, PORT_COUNT, bridge_offset);
		if (unlikely(port == PORT_COUNT))
			continue;

		routing_uft_entry_set(block, UFT_SEL_BOTH, fid, port);

		trace_rt_local(sd_src->fdev->pd->index,
			       sd_index(sd_src), fid, port, port);
	}

	return 0;
}

/**
 * unicast_route_src - Route all FIDs accessible from a given sd.
 * @sd_src: The source sd being routed.
 */
static void unicast_route_src(struct fsubdev *sd_src)
{
	struct fsubdev *sd_dst;
	int err;

	sd_src->routing.uft_next = routing_uft_alloc();
	if (!sd_src->routing.uft_next)
		goto err;

	for (sd_dst = routing_sd_iter(0); sd_dst;
	     sd_dst = routing_sd_next(sd_dst, 0)) {
		if (sd_src == sd_dst)
			continue;
		if (routing_cost_lookup(sd_src, sd_dst) ==
						ROUTING_COST_INFINITE)
			continue;
		err = unicast_route_pair(sd_src->routing.uft_next,
					 sd_src, sd_dst);
		if (err)
			goto err;
	}

	err = unicast_route_local(sd_src->routing.uft_next, sd_src);
	if (err)
		goto err;

	return;

err:
	routing_sd_transition_error(sd_src);
}

/**
 * unicast_routing - Route all FIDs across all devices.
 */
static void unicast_routing(void)
{
	struct fsubdev *sd;

	for (sd = routing_sd_iter(0); sd; sd = routing_sd_next(sd, 0))
		unicast_route_src(sd);
}

/**
 * routing_logic_run - Fully programs/routes the specified topology in the
 * manner of a one-time programming.
 * @topo: The topology to program/route.
 *
 * Return: Zero on success, non-zero otherwise.
 */
int routing_logic_run(struct routing_topology *topo)
{
	int err;

	pr_debug("initialize sweep\n");
	init(topo);

	pr_debug("process neighbors\n");
	process_neighbors();

	pr_debug("build planes\n");
	err = build_planes(topo);
	if (err) {
		pr_err("%s: failed to build planes: %d\n", __func__, err);
		return err;
	}

	pr_debug("compute costs\n");
	compute_costs(topo);

	pr_debug("unicast routing\n");
	unicast_routing();

	pr_debug("map addresses\n");
	map_addresses(topo);

	return 0;
}
