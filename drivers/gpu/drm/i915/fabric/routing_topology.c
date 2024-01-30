// SPDX-License-Identifier: MIT
/*
 * Copyright(c) 2019 - 2021 Intel Corporation.
 *
 */

#include "routing_topology.h"

u8 *routing_uft_block_alloc(u16 num_fids)
{
	u8 *block = kmalloc(num_fids, GFP_KERNEL);

	if (!block)
		return NULL;

	memset(block, 0xff, num_fids);
	return block;
}

/**
 * routing_uft_bridge_get - Returns the unicast fowarding table (UFT) block of
 * the source sd corresponding to the DFIDs of the destination sd's bridge.
 * @uft: The unicast forwarding table being queried (owned by source sd).
 * @sd_dst: The destination sd for which the block is being requested.
 *
 * Return: The requested block if found, otherwise NULL.
 */
u8 *routing_uft_bridge_get(struct routing_uft *uft, struct fsubdev *sd_dst)
{
	u8 *block;

	block = xa_load(&uft->bridges, sd_dst->routing.fid_group);
	if (!block) {
		block = routing_uft_block_alloc(ROUTING_FID_BLOCK_SIZE);
		if (!block)
			return NULL;
		xa_store(&uft->bridges, sd_dst->routing.fid_group, block,
			 GFP_KERNEL);
	}

	return block;
}

/**
 * routing_uft_alloc - Returns an newly allocated UFT with empty destination
 * bridge blocks.
 *
 * Return: The allocated unicast forwarding table, else NULL.
 */
struct routing_uft *routing_uft_alloc(void)
{
	struct routing_uft *uft = kzalloc(sizeof(*uft), GFP_KERNEL);

	if (!uft)
		return NULL;

	uft->mgmt = routing_uft_block_alloc(ROUTING_MAX_DEVICES);
	if (!uft->mgmt) {
		kfree(uft);
		return NULL;
	}

	xa_init(&uft->bridges);

	return uft;
}

/**
 * routing_uft_destroy - Destroys the specified UFT, deallocating all UFT
 * blocks within.
 * @uft: The unicast fowarding table to destroy.
 */
void routing_uft_destroy(struct routing_uft *uft)
{
	u8 *block;
	unsigned long i;

	if (!uft)
		return;

	kfree(uft->mgmt);

	xa_for_each(&uft->bridges, i, block) {
		xa_erase(&uft->bridges, i);
		kfree(block);
	}

	xa_destroy(&uft->bridges);

	kfree(uft);
}

void routing_uft_update(struct fsubdev *sd)
{
	struct routing_uft *prev = sd->routing.uft;

	sd->routing.uft = sd->routing.uft_next;
	sd->routing.uft_next = NULL;
	routing_uft_destroy(prev);
}

void routing_dfid_map_update(struct fsubdev *sd)
{
	struct routing_dfid_map *prev = sd->routing.dfid_map;

	sd->routing.dfid_map = sd->routing.dfid_map_next;
	sd->routing.dfid_map_next = NULL;
	kfree(prev);
}

/**
 * routing_sd_transition_error - Transitions the specified sd into an error
 * state and frees resources.
 * @sd: The sd to transition.
 */
void routing_sd_transition_error(struct fsubdev *sd)
{
	if (sd->routing.state == TILE_ROUTING_STATE_ERROR)
		return;

	routing_topo_signal_sd_error(sd->routing.topo);

	sd->routing.state = TILE_ROUTING_STATE_ERROR;

	if (sd->routing.fid_mgmt) {
		sd->routing.fid_group = 0;
		sd->routing.fid_mgmt = 0;
		sd->routing.fid_base = 0;
	}

	if (sd->routing.plane) {
		sd->routing.plane->num_subdevs--;
		list_del_init(&sd->routing.plane_link);
		sd->routing.plane = NULL;
		sd->routing.plane_index = 0;
	}

	routing_uft_destroy(sd->routing.uft);
	sd->routing.uft = NULL;

	routing_uft_destroy(sd->routing.uft_next);
	sd->routing.uft_next = NULL;

	kfree(sd->routing.dfid_map);
	sd->routing.dfid_map = NULL;

	kfree(sd->routing.dfid_map_next);
	sd->routing.dfid_map_next = NULL;
}

static struct fsubdev *next(struct fsubdev *sd)
{
	if (sd) {
		sd = list_next_entry(sd, routable_link);
		if (&sd->routable_link == &routable_list)
			sd = NULL;
	}

	return sd;
}

/**
 * routing_sd_iter - Initializes an iterator to traverse subdevices.
 * @all: non-zero if all subdevices should be iterated; otherwise only
 * subdevices not in an error state are iterated
 *
 * Return: The first sd in the iteration.
 */
struct fsubdev *routing_sd_iter(int all)
{
	struct fsubdev *sd;

	sd = list_first_entry_or_null(&routable_list, struct fsubdev,
				      routable_link);

	if (!all)
		while (sd && routing_sd_is_error(sd))
			sd = next(sd);

	return sd;
}

/**
 * routing_sd_next - Returns the next sd in the iteration.
 * @sd: The current sd, relative to which the next sd will be fetched.
 * @all: non-zero if all subdevices should be iterated; otherwise only
 * subdevices not in an error state are iterated
 *
 * Return: The next sd in the iteration.
 */
struct fsubdev *routing_sd_next(struct fsubdev *sd, int all)
{
	sd = next(sd);

	if (!all)
		while (sd && routing_sd_is_error(sd))
			sd = next(sd);

	return sd;
}
