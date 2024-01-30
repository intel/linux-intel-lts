/* SPDX-License-Identifier: MIT */
/*
 * Copyright(c) 2020 - 2021 Intel Corporation.
 *
 */

#ifndef ROUTING_P2P_H_INCLUDED
#define ROUTING_P2P_H_INCLUDED

#include "routing_topology.h"

struct routing_p2p_entry;

void routing_p2p_init(struct dentry *root);

void routing_p2p_cache(void);
void routing_p2p_lookup(struct fdev *src, struct fdev *dst,
			struct query_info *qi);

void routing_p2p_clear(struct fdev *dev);

#endif /* ROUTING_P2P_H_INCLUDED */
