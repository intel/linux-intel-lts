/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright(c) 2019 Intel Corporation. */

#ifndef _STMMAC_XSK_H_
#define _STMMAC_XSK_H_

struct xdp_umem;
struct zero_copy_allocator;

int stmmac_xsk_umem_setup(struct stmmac_priv *priv, struct xdp_umem *umem,
			  u16 qid);
void stmmac_zca_free(struct zero_copy_allocator *alloc, unsigned long handle);
bool stmmac_alloc_rx_buffers_zc(struct stmmac_rx_queue *rx_q, u16 count);
int stmmac_rx_zc(struct stmmac_priv *priv, int limit, u32 queue);
int stmmac_xdp_tx_clean(struct stmmac_priv *priv, int budget, u32 queue);
int stmmac_xsk_wakeup(struct net_device *dev, u32 queue, u32 flags);
void stmmac_xsk_clean_rx_queue(struct stmmac_rx_queue *rx_q);
void stmmac_xsk_clean_tx_queue(struct stmmac_tx_queue *tx_q);
bool stmmac_xsk_any_rx_ring_enabled(struct net_device *dev);

#endif /* _STMMAC_XSK_H_ */
