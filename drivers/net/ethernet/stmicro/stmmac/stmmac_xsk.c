// SPDX-License-Identifier: GPL-2.0
/* Copyright(c) 2019 Intel Corporation. */

#include <linux/bpf_trace.h>
#include <net/xdp_sock.h>
#include <net/xdp.h>

#include "stmmac.h"

/**
 * stmmac_xsk_umem_dma_map - DMA maps all UMEM memory for the netdev
 * @priv: driver private structure
 * @umem: UMEM to DMA map
 *
 * Returns 0 on success, <0 on failure
 **/
static int stmmac_xsk_umem_dma_map(struct stmmac_priv *priv,
				   struct xdp_umem *umem)
{
	struct device *dev;
	unsigned int i, j;
	dma_addr_t dma;

	dev = priv->device;
	for (i = 0; i < umem->npgs; i++) {
		dma = dma_map_page_attrs(dev, umem->pgs[i], 0, PAGE_SIZE,
					 DMA_BIDIRECTIONAL,
					 STMMAC_RX_DMA_ATTR);
		if (dma_mapping_error(dev, dma))
			goto out_unmap;

		umem->pages[i].dma = dma;
	}

	return 0;

out_unmap:
	for (j = 0; j < i; j++) {
		dma_unmap_page_attrs(dev, umem->pages[i].dma, PAGE_SIZE,
				     DMA_BIDIRECTIONAL, STMMAC_RX_DMA_ATTR);
		umem->pages[i].dma = 0;
	}

	return -1;
}

/**
 * stmmac_xsk_umem_dma_unmap - DMA unmaps all UMEM memory for the netdev
 * @priv: driver private structure
 * @umem: UMEM to DMA map
 **/
static void stmmac_xsk_umem_dma_unmap(struct stmmac_priv *priv,
				      struct xdp_umem *umem)
{
	struct device *dev;
	unsigned int i;

	dev = priv->device;

	for (i = 0; i < umem->npgs; i++) {
		dma_unmap_page_attrs(dev, umem->pages[i].dma, PAGE_SIZE,
				     DMA_BIDIRECTIONAL, STMMAC_RX_DMA_ATTR);

		umem->pages[i].dma = 0;
	}
}

/**
 * stmmac_xsk_umem_enable - Enable/associate a UMEM to a certain ring/qid
 * @priv: driver private structure
 * @umem: UMEM
 * @qid: Rx queue to associate UMEM to
 *
 * Returns 0 on success, <0 on failure
 **/
static int stmmac_xsk_umem_enable(struct stmmac_priv *priv,
				  struct xdp_umem *umem, u16 qid)
{
	struct net_device *netdev = priv->dev;
	struct xdp_umem_fq_reuse *reuseq;
	bool if_running;
	int err;

	if (qid >= priv->plat->num_queue_pairs)
		return -EINVAL;

	if (qid >= netdev->real_num_rx_queues ||
	    qid >= netdev->real_num_tx_queues)
		return -EINVAL;

	reuseq = xsk_reuseq_prepare(priv->dma_rx_size);
	if (!reuseq)
		return -ENOMEM;

	xsk_reuseq_free(xsk_reuseq_swap(umem, reuseq));

	err = stmmac_xsk_umem_dma_map(priv, umem);
	if (err)
		return err;

	set_bit(qid, &priv->af_xdp_zc_qps);

	if_running = netif_running(priv->dev) && stmmac_enabled_xdp(priv);

	if (if_running) {
		err = stmmac_queue_pair_disable(priv, qid);
		if (err)
			return err;

		err = stmmac_queue_pair_enable(priv, qid);
		if (err)
			return err;
	}

	return 0;
}

/**
 * stmmac_xsk_umem_disable - Disassociate a UMEM from a certain ring/qid
 * @priv: driver private structure
 * @qid: Rx queue to associate UMEM to
 *
 * Returns 0 on success, <0 on failure
 **/
static int stmmac_xsk_umem_disable(struct stmmac_priv *priv, u16 qid)
{
	struct net_device *netdev = priv->dev;
	struct xdp_umem *umem;
	bool if_running;
	int err;

	umem = xdp_get_umem_from_qid(netdev, qid);
	if (!umem)
		return -EINVAL;

	if_running = netif_running(priv->dev) && stmmac_enabled_xdp(priv);

	if (if_running) {
		err = stmmac_queue_pair_disable(priv, qid);
		if (err)
			return err;
	}

	clear_bit(qid, &priv->af_xdp_zc_qps);
	stmmac_xsk_umem_dma_unmap(priv, umem);

	if (if_running) {
		err = stmmac_queue_pair_enable(priv, qid);
		if (err)
			return err;
	}

	return 0;
}

/**
 * stmmac_xsk_umem_setup - Enable/disassociate a UMEM to/from a ring/qid
 * @priv: driver private structure
 * @umem: UMEM to enable/associate to a ring, or NULL to disable
 * @qid: Rx queue to (dis)associate UMEM (from)to
 *
 * This function enables or disables a UMEM to a certain queue.
 *
 * Returns 0 on success, <0 on failure
 **/
int stmmac_xsk_umem_setup(struct stmmac_priv *priv, struct xdp_umem *umem,
			  u16 qid)
{
	return umem ? stmmac_xsk_umem_enable(priv, umem, qid) :
	       stmmac_xsk_umem_disable(priv, qid);
}

/**
 * stmmac_run_xdp_zc - Executes an XDP program on an xdp_buff
 * @rx_q: Rx queue structure
 * @xdp: xdp_buff used as input to the XDP program
 *
 * This function enables or disables a UMEM to a certain ring.
 *
 * Returns any of I_XDP_{PASS, CONSUMED, TX, REDIR}
 **/
static int stmmac_run_xdp_zc(struct stmmac_rx_queue *rx_q, struct xdp_buff *xdp)
{
	struct stmmac_priv *priv = rx_q->priv_data;
	int err, result = STMMAC_XDP_PASS;
	struct stmmac_tx_queue *xdp_q;
	struct bpf_prog *xdp_prog;
	u32 act;

	rcu_read_lock();
	/* NB! xdp_prog will always be !NULL, due to the fact that
	 * this path is enabled by setting an XDP program.
	 */
	xdp_prog = READ_ONCE(rx_q->xdp_prog);
	act = bpf_prog_run_xdp(xdp_prog, xdp);
	xdp->handle += xdp->data - xdp->data_hard_start;

	switch (act) {
	case XDP_PASS:
		break;
	case XDP_TX:
		xdp_q = &priv->xdp_queue[rx_q->queue_index];
		result = stmmac_xmit_xdp_tx_queue(xdp, xdp_q);
		break;
	case XDP_REDIRECT:
		err = xdp_do_redirect(priv->dev, xdp, xdp_prog);
		result = !err ? STMMAC_XDP_REDIR : STMMAC_XDP_CONSUMED;
		break;
	default:
		bpf_warn_invalid_xdp_action(act);
		/* fall through */
	case XDP_ABORTED:
		trace_xdp_exception(priv->dev, xdp_prog, act);
		/* fall through -- handle aborts by dropping packet */
	case XDP_DROP:
		result = STMMAC_XDP_CONSUMED;
		break;
	}
	rcu_read_unlock();
	return result;
}

/**
 * stmmac_alloc_buffer_zc - Allocates an Rx Buffer from XDP ZC
 * @rx_q: RX queue structure
 * @bi: Rx buffer to populate
 *
 * This function allocates an Rx buffer. The buffer can come from fill
 * queue, or via the recycle queue (next_to_alloc).
 *
 * Returns true for a successful allocation, false otherwise
 **/
static bool stmmac_alloc_buffer_zc(struct stmmac_rx_queue *rx_q,
				   struct stmmac_rx_buffer *buf)
{
	struct xdp_umem *umem = rx_q->xsk_umem;
	void *addr = buf->umem_addr;
	u64 handle, hr;

	if (addr)
		return true;

	if (!xsk_umem_peek_addr(umem, &handle))
		return false;

	hr = umem->headroom + XDP_PACKET_HEADROOM;

	buf->addr = xdp_umem_get_dma(umem, handle);
	buf->addr += hr;

	buf->umem_addr = xdp_umem_get_data(umem, handle);
	buf->umem_addr += hr;

	buf->umem_handle = handle + umem->headroom;

	xsk_umem_discard_addr(umem);

	return true;
}

/**
 * stmmac_alloc_buffer_slow_zc - Allocates an stmmac_rx_buffer
 * @rx_q: Rx queue
 * @buf: Rx buffer to populate
 *
 * This function allocates an Rx buffer. The buffer can come from fill
 * queue, or via the reuse queue.
 *
 * Returns true for a successful allocation, false otherwise
 **/
static bool stmmac_alloc_buffer_slow_zc(struct stmmac_rx_queue *rx_q,
					struct stmmac_rx_buffer *buf)
{
	struct xdp_umem *umem = rx_q->xsk_umem;
	u64 handle, hr;

	if (!xsk_umem_peek_addr_rq(umem, &handle))
		return false;

	handle &= rx_q->xsk_umem->chunk_mask;

	hr = umem->headroom + XDP_PACKET_HEADROOM;

	buf->addr = xdp_umem_get_dma(umem, handle);
	buf->addr += hr;

	buf->umem_addr = xdp_umem_get_data(umem, handle);
	buf->umem_addr += hr;

	buf->umem_handle = handle + umem->headroom;

	xsk_umem_discard_addr_rq(umem);

	return true;
}

static __always_inline bool
__stmmac_alloc_rx_buffers_zc(struct stmmac_rx_queue *rx_q, u16 count,
			     bool alloc(struct stmmac_rx_queue *rx_q,
					struct stmmac_rx_buffer *buf))
{
	struct stmmac_priv *priv = rx_q->priv_data;
	u16 entry = rx_q->dirty_rx;
	bool ok = true;
	struct stmmac_rx_buffer *buf;
	struct dma_desc *rx_desc;
	unsigned int last_refill = entry;

	do {
		bool use_rx_wd;

		if (priv->extend_desc)
			rx_desc = (struct dma_desc *)(rx_q->dma_erx + entry);
		else
			rx_desc = rx_q->dma_rx + entry;

		buf = &rx_q->buf_pool[entry];
		if (!alloc(rx_q, buf)) {
			ok = false;
			goto no_buffers;
		}

		dma_sync_single_range_for_device(priv->device, buf->addr, 0,
						 rx_q->dma_buf_sz,
						 DMA_BIDIRECTIONAL);

		stmmac_set_desc_addr(priv, rx_desc, buf->addr);
		stmmac_refill_desc3(priv, rx_q, rx_desc);
		use_rx_wd = priv->use_riwt && rx_q->rx_count_frames;

		stmmac_set_rx_owner(priv, rx_desc, use_rx_wd);
		last_refill = entry;
		entry = STMMAC_GET_ENTRY(entry, priv->dma_rx_size);

		count--;
	} while (count);

no_buffers:
	if (rx_q->dirty_rx != entry) {
		rx_q->dirty_rx = entry;
		rx_q->next_to_alloc = entry;

		wmb();
		rx_q->rx_tail_addr = rx_q->dma_rx_phy + (last_refill *
				     sizeof(struct dma_desc));
		stmmac_set_rx_tail_ptr(priv, priv->ioaddr,
				       rx_q->rx_tail_addr, rx_q->queue_index);
	}

	return ok;
}

/**
 * stmmac_alloc_rx_buffers_zc - Allocates a number of Rx buffers
 * @rx_q: Rx queue structure
 * @count: The number of buffers to allocate
 *
 * This function allocates a number of Rx buffers from the reuse queue
 * or fill ring and places them on the Rx queue.
 *
 * Returns true for a successful allocation, false otherwise
 **/
bool stmmac_alloc_rx_buffers_zc(struct stmmac_rx_queue *rx_q, u16 count)
{
	rx_q->cur_rx = 0;
	rx_q->dirty_rx = 0;
	rx_q->next_to_alloc = 0;

	return __stmmac_alloc_rx_buffers_zc(rx_q, count,
					    stmmac_alloc_buffer_slow_zc);
}

/**
 * stmmac_alloc_rx_buffers_fast_zc - Allocates a number of Rx buffers
 * @rx_q: Rx queue
 * @count: The number of buffers to allocate
 *
 * This function allocates a number of Rx buffers from the fill ring
 * or the internal recycle mechanism and places them on the Rx ring.
 *
 * Returns true for a successful allocation, false otherwise
 **/
static bool stmmac_alloc_rx_buffers_fast_zc(struct stmmac_rx_queue *rx_q,
					    u16 count)
{
	return __stmmac_alloc_rx_buffers_zc(rx_q, count,
					   stmmac_alloc_buffer_zc);
}

/**
 * stmmac_get_rx_buffer_zc - Return the current Rx buffer
 * @rx_q: Rx queue structure
 * @size: The size of the rx buffer (read from descriptor)
 *
 * This function returns the current, received Rx buffer, and also
 * does DMA synchronization for the Rx queue.
 *
 * Returns the received Rx buffer
 **/
static struct stmmac_rx_buffer *stmmac_get_rx_buffer_zc(struct stmmac_rx_queue *rx_q,
							const unsigned int size)
{
	struct stmmac_rx_buffer *buf;
	struct stmmac_priv *priv;

	buf = &rx_q->buf_pool[rx_q->cur_rx];
	priv = rx_q->priv_data;

	/* we are reusing so sync this buffer for CPU use */
	dma_sync_single_range_for_cpu(priv->device,
				      buf->addr, 0,
				      size,
				      DMA_BIDIRECTIONAL);

	return buf;
}

/**
 * stmmac_reuse_rx_buffer_zc - Recycle an Rx buffer
 * @rx_q: Rx queue
 * @old_buf: The Rx buffer to recycle
 *
 * This function recycles a finished Rx buffer, and places it on the
 * recycle queue (next_to_alloc).
 **/
static void stmmac_reuse_rx_buffer_zc(struct stmmac_rx_queue *rx_q,
				      struct stmmac_rx_buffer *old_buf)
{
	struct stmmac_rx_buffer *new_buf = &rx_q->buf_pool[rx_q->next_to_alloc];
	unsigned long mask = (unsigned long)rx_q->xsk_umem->chunk_mask;
	u64 hr = rx_q->xsk_umem->headroom + XDP_PACKET_HEADROOM;
	struct stmmac_priv *priv = rx_q->priv_data;
	u16 nta = rx_q->next_to_alloc;

	/* update, and store next to alloc */
	nta++;
	rx_q->next_to_alloc = (nta < priv->dma_rx_size) ? nta : 0;

	/* transfer page from old buffer to new buffer */
	new_buf->addr = old_buf->addr & mask;
	new_buf->addr += hr;

	new_buf->umem_addr = (void *)((unsigned long)old_buf->umem_addr & mask);
	new_buf->umem_addr += hr;

	new_buf->umem_handle = old_buf->umem_handle & mask;
	new_buf->umem_handle += rx_q->xsk_umem->headroom;

	old_buf->umem_addr = NULL;
}

/**
 * stmmac_zca_free - Free callback for MEM_TYPE_ZERO_COPY allocations
 * @alloc: Zero-copy allocator
 * @handle: Buffer handle
 **/
void stmmac_zca_free(struct zero_copy_allocator *alloc, unsigned long handle)
{
	struct stmmac_rx_buffer *buf;
	struct stmmac_rx_queue *rx_q;
	struct stmmac_priv *priv;
	u64 hr, mask;
	u16 nta;

	rx_q = container_of(alloc, struct stmmac_rx_queue, zca);
	hr = rx_q->xsk_umem->headroom + XDP_PACKET_HEADROOM;
	mask = rx_q->xsk_umem->chunk_mask;

	nta = rx_q->next_to_alloc;
	buf = &rx_q->buf_pool[nta];
	priv = rx_q->priv_data;

	nta++;
	rx_q->next_to_alloc = (nta < priv->dma_rx_size) ? nta : 0;

	handle &= mask;

	buf->addr = xdp_umem_get_dma(rx_q->xsk_umem, handle);
	buf->addr += hr;

	buf->umem_addr = xdp_umem_get_data(rx_q->xsk_umem, handle);
	buf->umem_addr += hr;

	buf->umem_handle = (u64)handle + rx_q->xsk_umem->headroom;
}

/**
 * stmmac_construct_skb_zc - Create skbufff from zero-copy Rx buffer
 * @rx_q: Rx queue structure
 * @bi: Rx buffer
 * @xdp: xdp_buff
 *
 * This functions allocates a new skb from a zero-copy Rx buffer.
 *
 * Returns the skb, or NULL on failure.
 **/
static struct sk_buff *stmmac_construct_skb_zc(struct stmmac_rx_queue *rx_q,
					       struct stmmac_rx_buffer *buf,
					       struct xdp_buff *xdp)
{
	struct stmmac_priv *priv = rx_q->priv_data;
	unsigned int metasize = xdp->data - xdp->data_meta;
	unsigned int datasize = xdp->data_end - xdp->data;
	struct stmmac_channel *ch;
	struct sk_buff *skb;

	ch = &priv->channel[rx_q->queue_index];

	/* allocate a skb to store the frags */
	skb = __napi_alloc_skb(&ch->rx_napi,
			       xdp->data_end - xdp->data_hard_start,
			       GFP_ATOMIC | __GFP_NOWARN);
	if (unlikely(!skb))
		return NULL;

	skb_reserve(skb, xdp->data - xdp->data_hard_start);
	memcpy(__skb_put(skb, datasize), xdp->data, datasize);
	if (metasize)
		skb_metadata_set(skb, metasize);

	stmmac_reuse_rx_buffer_zc(rx_q, buf);

	return skb;
}

/**
 * stmmac_inc_ntc: Advance the next_to_clean index
 * @rx_q: Rx queue
 **/
static void stmmac_inc_ntc(struct stmmac_rx_queue *rx_q)
{
	struct stmmac_priv *priv = rx_q->priv_data;
	struct dma_desc *rx_desc;
	u32 ntc;

	ntc = rx_q->cur_rx + 1;
	ntc = (ntc < priv->dma_rx_size) ? ntc : 0;
	rx_q->cur_rx = ntc;

	if (priv->extend_desc)
		rx_desc = (struct dma_desc *)(rx_q->dma_erx + ntc);
	else
		rx_desc = rx_q->dma_rx + ntc;

	prefetch(rx_desc);
}

/**
 * stmmac_rx_zc - Consumes Rx packets from the hardware queue
 * @rx_q: Rx queue structure
 * @budget: NAPI budget
 *
 * Returns amount of work completed
 **/
int stmmac_rx_zc(struct stmmac_priv *priv, int budget, u32 queue)
{
	unsigned int total_rx_bytes = 0, total_rx_packets = 0;
	struct stmmac_rx_queue *rx_q = &priv->rx_queue[queue];
	struct stmmac_channel *ch = &priv->channel[queue];
	u16 fill_count = STMMAC_RX_DESC_UNUSED(rx_q);
	unsigned int xdp_res, xdp_xmit = 0;
	int coe = priv->hw->rx_csum;
	bool failure = false;
	struct sk_buff *skb;
	struct xdp_buff xdp;

	xdp.rxq = &rx_q->xdp_rxq;

	while (likely(total_rx_packets < (unsigned int)budget)) {
		struct stmmac_rx_buffer *buf;
		struct dma_desc *rx_desc;
		unsigned int size;
		int status;

		if (fill_count >= STMMAC_RX_BUFFER_WRITE) {
			failure = failure ||
				  !stmmac_alloc_rx_buffers_fast_zc(rx_q,
								   fill_count);
			fill_count = 0;
		}

		if (priv->extend_desc)
			rx_desc = (struct dma_desc *)(rx_q->dma_erx +
						      rx_q->cur_rx);
		else
			rx_desc = rx_q->dma_rx + rx_q->cur_rx;

		/* This memory barrier is needed to keep us from reading
		 * any other fields out of the rx_desc until we have
		 * verified the descriptor has been written back.
		 */
		dma_rmb();

		/* read the status of the incoming frame */
		status = stmmac_rx_status(priv, &priv->dev->stats,
					  &priv->xstats, rx_desc);

		if (unlikely(status & dma_own))
			break;

		size = stmmac_get_rx_frame_len(priv, rx_desc,
					       coe);
		if (!size)
			break;

		buf = stmmac_get_rx_buffer_zc(rx_q, size);

		if (unlikely(status == discard_frame)) {
			stmmac_reuse_rx_buffer_zc(rx_q, buf);
			priv->dev->stats.rx_errors++;
			fill_count++;
			continue;
		}

		xdp.data = buf->umem_addr;
		xdp.data_meta = xdp.data;
		xdp.data_hard_start = xdp.data - XDP_PACKET_HEADROOM;
		xdp.data_end = xdp.data + size;
		xdp.handle = buf->umem_handle;

		xdp_res = stmmac_run_xdp_zc(rx_q, &xdp);
		if (xdp_res) {
			if (xdp_res & (STMMAC_XDP_TX | STMMAC_XDP_REDIR)) {
				xdp_xmit |= xdp_res;
				buf->umem_addr = NULL;
			} else {
				stmmac_reuse_rx_buffer_zc(rx_q, buf);
			}

			total_rx_bytes += size;
			total_rx_packets++;

			fill_count++;
			stmmac_inc_ntc(rx_q);
			continue;
		}

		/* XDP_PASS path */
		skb = stmmac_construct_skb_zc(rx_q, buf, &xdp);
		if (unlikely(!skb)) {
			priv->dev->stats.rx_dropped++;
			break;
		}

		fill_count++;

		stmmac_inc_ntc(rx_q);

		if (eth_skb_pad(skb))
			continue;

		total_rx_bytes += skb->len;
		total_rx_packets++;

		/* Use HW to strip VLAN header before fallback
		 * to SW.
		 */
		status = stmmac_rx_hw_vlan(priv, priv->dev,
					   priv->hw, rx_desc, skb);
		if (status == -EINVAL)
			stmmac_rx_vlan(priv->dev, skb);

		skb->protocol = eth_type_trans(skb, priv->dev);

		if (unlikely(!coe))
			skb_checksum_none_assert(skb);
		else
			skb->ip_summed = CHECKSUM_UNNECESSARY;

		napi_gro_receive(&ch->rx_napi, skb);
	}

	stmmac_finalize_xdp_rx(rx_q, xdp_xmit);

	priv->dev->stats.rx_packets += total_rx_packets;
	priv->dev->stats.rx_bytes += total_rx_bytes;

	return failure ? budget : (int)total_rx_packets;
}
