/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2020, Intel Corporation. */

#ifndef _IGC_XDP_H_
#define _IGC_XDP_H_

#include <asm-generic/int-ll64.h>

struct igc_md_desc {
	u64 timestamp;
};

int igc_xdp_set_prog(struct igc_adapter *adapter, struct bpf_prog *prog,
		     struct netlink_ext_ack *extack);
int igc_xdp_setup_pool(struct igc_adapter *adapter, struct xsk_buff_pool *pool,
		       u16 queue_id);

static inline bool igc_xdp_is_enabled(struct igc_adapter *adapter)
{
	return !!adapter->xdp_prog;
}

int igc_xdp_register_rxq_info(struct igc_ring *ring);
void igc_xdp_unregister_rxq_info(struct igc_ring *ring);
int igc_xdp_query_btf(struct net_device *dev, u8 *enabled);
int igc_xdp_set_btf_md(struct net_device *dev, u8 enable);

#endif /* _IGC_XDP_H_ */
