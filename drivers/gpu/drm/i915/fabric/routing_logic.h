/* SPDX-License-Identifier: MIT */
/*
 * Copyright(c) 2019 - 2021 Intel Corporation.
 *
 */

#ifndef ROUTING_LOGIC_H_INCLUDED
#define ROUTING_LOGIC_H_INCLUDED

#include "routing_topology.h"

u16 routing_cost_lookup(struct fsubdev *sd_src, struct fsubdev *sd_dst);
void routing_plane_destroy(struct routing_plane *plane);
int routing_logic_run(struct routing_topology *topo);

#endif /* ROUTING_LOGIC_H_INCLUDED */
