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

#endif /* _STMMAC_XSK_H_ */
