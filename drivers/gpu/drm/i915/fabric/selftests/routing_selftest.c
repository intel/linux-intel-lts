// SPDX-License-Identifier: MIT
/*
 * Copyright(c) 2019 - 2021 Intel Corporation.
 *
 */

#include "../routing_logic.h"
#include "../routing_p2p.h"
#include "selftest.h"
#include "routing_selftest.h"
#include "routing_mock.h"

static int validate_single_dpa(struct routing_topology *topo,
			       struct fsubdev *sd_src)
{
	int i;
	int start = sd_src->routing.dpa_idx_base;
	int end = start + sd_src->routing.dpa_idx_range;
	u16 dfid;

	for (i = start; i < end; ++i) {
		TEST(i <= topo->max_dpa_index,
		     "topology has an accurate max dpa index");

		dfid = (u16)(sd_src->routing.dfid_map_next->dfid[i] <<
			     ROUTING_DPA_DFID_MAP_SHIFT);
		TEST(dfid == 0,
		     "sds configure their local memory range to invalid fids: device %u sd %u dpa_idx %d lut_value 0x%04x",
		     sd_src->fdev->pd->index, sd_index(sd_src), i,
		     (u16)sd_src->routing.dfid_map_next->dfid[i]);
	}

	return 0;
}

static int validate_single(struct routing_topology *topo,
			   struct fsubdev *sd_src)
{
	struct fport *port;
	u8 *block;
	int err;
	u16 dfid;
	u16 i, j, n;
	u8 fid_channel, port_channel;
	u8 lpn, lpn_neighbor;

	TEST(sd_src->routing.state != TILE_ROUTING_STATE_ERROR,
	     "the sd has not errored");

	TEST(sd_src->routing.plane, "the sd is assigned to a plane");

	TEST(sd_src->routing.uft_next->mgmt,
	     "the sd received a unicast forwarding table for mgmt fids");

	TEST(!xa_empty(&sd_src->routing.uft_next->bridges),
	     "the sd received a unicast forwarding table for bridge fids");

	TEST(sd_src->routing.dfid_map_next, "the sd received a dfid map");

	err = validate_single_dpa(topo, sd_src);
	if (err)
		return err;

	lpn = routing_uft_entry_get(sd_src->routing.uft_next->mgmt,
				    UFT_SEL_ORIGIN, sd_src->routing.fid_mgmt -
				    ROUTING_FID_CPORT_BASE);

	TEST(lpn == 0,
	     "subdevs route their own mgmt fid to port 0: device %u sd %u fid 0x%04x port %u",
	     sd_src->fdev->pd->index, sd_index(sd_src),
	     sd_src->routing.fid_mgmt, lpn);

	block = routing_uft_bridge_get(sd_src->routing.uft_next, sd_src);

	TEST(block, "sd uft block to self is valid");

	for (dfid = 0; dfid < ROUTING_FID_BLOCK_SIZE; ++dfid) {
		lpn = routing_uft_entry_get(block, UFT_SEL_ORIGIN, dfid);

		if (!is_valid_fid(dfid)) {
			TEST(lpn == ROUTING_UFT_INVALID_PORT4,
			     "routes to invalid fids are invalid: device %u sd %u dfid 0x%04x port %u",
			     sd_src->fdev->pd->index, sd_index(sd_src),
			     dfid, lpn);

			continue;
		}
		fid_channel = FIELD_GET(ROUTING_FID_MDFI_MASK, dfid);
		if (fid_channel == ROUTING_FID_NUM_CHANNELS) {
			TEST(lpn == ROUTING_UFT_INVALID_PORT4,
			     "fids to the fourth bridge are not routed: device %u sd %u dfid 0x%04x port %u",
			     sd_src->fdev->pd->index, sd_index(sd_src),
			     dfid, lpn);
			continue;
		}

		TEST(lpn >= PORT_BRIDGE_START && lpn <= PORT_BRIDGE_END,
		     "local fids map to the local bridge ports: device %u sd %u fid 0x%04x port %u",
		     sd_src->fdev->pd->index, sd_index(sd_src),
		     sd_src->routing.fid_base + dfid, lpn);

		port_channel = lpn - PORT_BRIDGE_START;
		TEST(fid_channel == port_channel,
		     "fid mdfi channel is consistent with bridge port: device %u sd %u fid 0x%04x port %u fid_channel %u port_channel %u",
		     sd_src->fdev->pd->index, sd_index(sd_src),
		     sd_src->routing.fid_base + dfid, lpn,
		     fid_channel, port_channel);
	}

	for_each_fabric_port(port, lpn, sd_src) {
		TEST(!port->portinfo->neighbor_guid || neighbor_of(port),
		     "ports with neighbor guids have neighbors: device %u sd %u port %u neighbor_guid 0x%016llx neighbor_valid %u",
		     sd_src->fdev->pd->index, sd_index(sd_src),
		     port->lpn, port->portinfo->neighbor_guid,
		     !!neighbor_of(port));

		if (!port->portinfo->neighbor_guid)
			continue;

		lpn_neighbor = port->portinfo->neighbor_port_number;

		TEST(!port->portinfo->neighbor_port_number ||
		     test_bit(lpn_neighbor, sd_src->fport_lpns),
		     "neighbors of fabric ports are fabric ports");

		n = sd_src->routing.plane->num_subdevs;
		i = sd_src->routing.plane_index;
		j = neighbor_of(port)->sd->routing.plane_index;
		TEST(sd_src->routing.plane->cost[i * n + j] == 1,
		     "the cost to a neighbor is exactly 1");
	}

	return 0;
}

static int validate_pair_dfid(struct routing_topology *topo,
			      struct fsubdev *sd_src,
			      struct fsubdev *sd_dst,
			      u16 dfid)
{
	u8 *block;
	struct fport *port, *nport;
	u8 lpn;

	TEST(sd_index(sd_src) != sd_index(sd_dst) ||
	     sd_src->routing.plane == sd_dst->routing.plane,
	     "subdevices with the same offset share a plane");

	block = routing_uft_bridge_get(sd_src->routing.uft_next, sd_dst);

	TEST(block, "sd uft block to destination is valid");

	lpn = routing_uft_entry_get(block, UFT_SEL_ORIGIN,
				    dfid - sd_dst->routing.fid_base);

	if (!is_valid_fid(dfid)) {
		TEST(lpn == ROUTING_UFT_INVALID_PORT4,
		     "routes to invalid fids are invalid: device %u sd %u dfid 0x%04x port %u",
		     sd_src->fdev->pd->index, sd_index(sd_src),
		     dfid, lpn);
	} else if (sd_src->routing.plane == sd_dst->routing.plane) {
		TEST(lpn < PORT_COUNT,
		     "uft entries exist for every on-plane dfid: source devce %u sd %u; dest device %u sd %u; dfid 0x%04x port %u",
		     sd_src->fdev->pd->index, sd_index(sd_src),
		     sd_dst->fdev->pd->index, sd_index(sd_dst),
		     dfid, lpn);

		port = get_fport_handle(sd_src, lpn);
		TEST(port,
		     "fabric port numbers map to valid ports: port %u", lpn);

		nport = neighbor_of(port);
		TEST(nport && nport->sd == sd_dst,
		     "uft entries point to the correct neighbor: source device %u sd %u; dest device %u sd %u; dfid 0x%04x port %u neighbor valid %u device %u sd %u",
		     sd_src->fdev->pd->index, sd_index(sd_src),
		     sd_dst->fdev->pd->index, sd_index(sd_dst),
		     dfid, lpn, nport ? 1 : 0,
		     nport ? nport->sd->fdev->pd->index : 0,
		     nport ? sd_index(nport->sd) : 0);
	} else {
		TEST(lpn == ROUTING_UFT_INVALID_PORT4,
		     "routes to off-plane fids are invalid");
	}

	return 0;
}

static int validate_pair_dpa_entry(struct routing_topology *topo,
				   struct fsubdev *sd_src,
				   struct fsubdev *sd_dst, u16 dpa_idx)
{
	struct fsubdev *neighbor;
	u16 dfid = (u16)(sd_src->routing.dfid_map_next->dfid[dpa_idx] <<
			 ROUTING_DPA_DFID_MAP_SHIFT);
	int i;

	for (i = 0; i < sd_dst->fdev->pd->sd_cnt; ++i) {
		neighbor = &sd_dst->fdev->sd[i];
		if (neighbor->routing.fid_base != dfid)
			continue;
		if (routing_cost_lookup(sd_src, neighbor) ==
		    ROUTING_COST_INFINITE)
			continue;
		return 0;
	}

	FAIL("dfid does not map to a reachable subdevice: src device %u sd %u;"
	     " dst device %u sd %u; dpa_idx %u dfid 0x%04x\n",
	     sd_src->fdev->pd->index, sd_index(sd_src),
	     sd_dst->fdev->pd->index, sd_index(sd_dst), dpa_idx, dfid);
}

static int validate_pair_dpa(struct routing_topology *topo,
			     struct fsubdev *sd_src,
			     struct fsubdev *sd_dst)
{
	u16 i;
	u16 dst_start = sd_dst->routing.dpa_idx_base;
	u16 dst_end = dst_start + sd_dst->routing.dpa_idx_range;
	u16 src_start = sd_src->routing.dpa_idx_base;
	u16 src_end = src_start + sd_src->routing.dpa_idx_range;
	int err;

	for (i = dst_start; i < dst_end; ++i) {
		err = validate_pair_dpa_entry(topo, sd_src, sd_dst, i);
		if (err)
			return err;

		TEST(src_start >= dst_end || dst_start >= src_end,
		     "addresses are unique across all sds, regardless of plane");
	}

	return 0;
}

static int validate_p2p_test(struct fsubdev *sd_src, struct fsubdev *sd_dst,
			     struct query_info *qi, u16 bw, u16 lat)
{
	int i;

	for (i = 0; i < qi->src_cnt * qi->dst_cnt; ++i) {
		TEST(qi->sd2sd[i].bandwidth == bw,
		     "bandwidth is valid: src %u dst %u index %u: actual %u expected %u",
		     sd_src->fdev->pd->index, sd_dst->fdev->pd->index, i,
		     qi->sd2sd[i].bandwidth, bw);
		TEST(qi->sd2sd[i].latency == lat,
		     "latency is valid: src %u dst %u index %u: actual %u expected %u",
		     sd_src->fdev->pd->index, sd_dst->fdev->pd->index, i,
		     qi->sd2sd[i].latency, lat);
	}

	return 0;
}

static int validate_p2p(struct fsubdev *sd_src, struct fsubdev *sd_dst)
{
	struct query_info *qi;
	int n;
	u16 bw, lat;
	int err;

	/* only do p2p test once per device */
	if (sd_index(sd_src) || sd_index(sd_dst))
		return 0;

	n = sd_src->fdev->pd->sd_cnt * sd_dst->fdev->pd->sd_cnt;
	qi = kzalloc(struct_size(qi, sd2sd, n), GFP_KERNEL);
	if (!qi)
		return -ENOMEM;

	qi->src_cnt = sd_src->fdev->pd->sd_cnt;
	qi->dst_cnt = sd_dst->fdev->pd->sd_cnt;

	routing_p2p_lookup(sd_src->fdev, sd_dst->fdev, qi);

	if (sd_src->fdev == sd_dst->fdev) {
		/* intra-device is reported as no fabric connectivity */
		bw = 0;
		lat = 0;
	} else {
		/* inter-device is one hop at 90g/4x link config */
		bw = 360;
		lat = 10;
	}

	err = validate_p2p_test(sd_src, sd_dst, qi, bw, lat);

	kfree(qi);
	return err;
}

static int validate_pair(struct routing_topology *topo,
			 struct fsubdev *sd_src,
			 struct fsubdev *sd_dst)
{
	int err;
	u16 dfid;
	u8 lpn;

	TEST(sd_src->routing.fid_base != sd_dst->routing.fid_base,
	     "fids are unique across all subdevs, regardless of plane");

	err = validate_pair_dpa(topo, sd_src, sd_dst);
	if (err)
		return err;

	lpn = routing_uft_entry_get(sd_src->routing.uft_next->mgmt,
				    UFT_SEL_ORIGIN, sd_dst->routing.fid_mgmt -
				    ROUTING_FID_CPORT_BASE);

	if (sd_src->routing.plane == sd_dst->routing.plane) {
		TEST(lpn >= PORT_FABRIC_START && lpn <= PORT_FABRIC_END,
		     "routes to on-plane remote mgmt fids go via fabric ports: device %u sd %u dfid 0x%04x port %u",
		     sd_src->fdev->pd->index, sd_index(sd_src),
		     sd_dst->routing.fid_mgmt, lpn);
	} else {
		TEST(lpn == ROUTING_UFT_INVALID_PORT4,
		     "routes to off-plane remote mgmt fids are invalid: device %u sd %u dfid 0x%04x port %u",
		     sd_src->fdev->pd->index, sd_index(sd_src),
		     sd_dst->routing.fid_mgmt, lpn);
	}

	for (dfid = sd_dst->routing.fid_base;
	     dfid < sd_dst->routing.fid_base + ROUTING_FID_BLOCK_SIZE; ++dfid) {
		err = validate_pair_dfid(topo, sd_src, sd_dst, dfid);
		if (err)
			return err;
	}

	return 0;
}

/*
 * Validate the routing calculations, assuming a healthy point-to-point
 * topology.
 */
static int validate(struct routing_topology *topo)
{
	struct fsubdev *sd_src, *sd_dst;
	int err;

	for (sd_src = routing_sd_iter(1); sd_src;
	     sd_src = routing_sd_next(sd_src, 1)) {
		err = validate_single(topo, sd_src);
		if (err)
			return err;

		for (sd_dst = routing_sd_iter(1); sd_dst;
		     sd_dst = routing_sd_next(sd_dst, 1)) {
			if (sd_src == sd_dst ||
			    sd_src->fdev == sd_dst->fdev)
				continue;

			TEST(sd_src->fdev->pd->index !=
			     sd_dst->fdev->pd->index,
			     "devices do not share indices");

			err = validate_pair(topo, sd_src, sd_dst);
			if (err)
				return err;
		}
	}

	return 0;
}

static void mock_io(struct routing_topology *topo)
{
	struct fsubdev *sd;

	/* just update the pending structures as if they were written */
	for (sd = routing_sd_iter(1); sd; sd = routing_sd_next(sd, 1))
		routing_update(sd);

	routing_p2p_cache();
}

static int validate_post_io(void)
{
	struct fsubdev *sd_src, *sd_dst;
	int err;

	for (sd_src = routing_sd_iter(1); sd_src;
	     sd_src = routing_sd_next(sd_src, 1)) {
		for (sd_dst = routing_sd_iter(1); sd_dst;
		     sd_dst = routing_sd_next(sd_dst, 1)) {
			err = validate_p2p(sd_src, sd_dst);
			if (err)
				return err;
		}
	}

	return 0;
}

int routing_selftest(void)
{
	struct routing_topology topo;
	int err;

	err = routing_mock_create_topology(&topo);
	if (err) {
		pr_err("%s: failed to create topology: %d\n", __func__, err);
		goto exit;
	}

	err = routing_logic_run(&topo);
	if (err) {
		pr_err("%s: failed to route topology: %d\n", __func__, err);
		goto exit;
	}

	err = validate(&topo);
	if (err)
		goto exit;

	mock_io(&topo);

	err = validate_post_io();

exit:
	routing_mock_destroy(&topo);
	return err;
}
