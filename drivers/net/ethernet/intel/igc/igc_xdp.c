// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2020, Intel Corporation. */

#include <linux/if_vlan.h>
#include <net/xdp_sock_drv.h>
#include <linux/btf.h>

#include "igc.h"
#include "igc_xdp.h"

#define BTF_INFO_ENC(kind, kind_flag, vlen)			\
	((!!(kind_flag) << 31) | ((kind) << 24) | ((vlen) & BTF_MAX_VLEN))

#define BTF_TYPE_ENC(name, info, size_or_type)	\
	(name), (info), (size_or_type)

#define BTF_INT_ENC(encoding, bits_offset, nr_bits)	\
	((encoding) << 24 | (bits_offset) << 16 | (nr_bits))

#define BTF_TYPE_INT_ENC(name, encoding, bits_offset, bits, sz)	\
	BTF_TYPE_ENC(name, BTF_INFO_ENC(BTF_KIND_INT, 0, 0), sz),	\
	BTF_INT_ENC(encoding, bits_offset, bits)

#define BTF_STRUCT_ENC(name, nr_elems, sz)	\
	BTF_TYPE_ENC(name, BTF_INFO_ENC(BTF_KIND_STRUCT, 1, nr_elems), sz)

#define BTF_MEMBER_ENC(name, type, bits_offset)	\
	(name), (type), (bits_offset)

/* struct xdp_md_desc {
 *	u64 timestamp;
 * };
 */
#define IGC_MD_NUM_MMBRS 1
static const char names_str[] = "\0xdp_md_desc\0timestamp\0";

/* Must match struct xdp_md_desc */
static const u32 igc_md_raw_types[] = {
	/* #define u64 */
	BTF_TYPE_INT_ENC(0, 0, 0, 64, 8),         /* type [1] */
	/* struct xdp_md_desc { */
	BTF_STRUCT_ENC(1, IGC_MD_NUM_MMBRS, 8),
		BTF_MEMBER_ENC(13, 1, 0),    /* u64 timestamp;    */
	/* } */
};

static int igc_xdp_register_btf(struct igc_adapter *priv)
{
	unsigned int type_sec_sz, str_sec_sz;
	char *types_sec, *str_sec;
	struct btf_header *hdr;
	unsigned int btf_size;
	void *raw_btf = NULL;
	int err = 0;

	type_sec_sz = sizeof(igc_md_raw_types);
	str_sec_sz  = sizeof(names_str);

	btf_size = sizeof(*hdr) + type_sec_sz + str_sec_sz;
	raw_btf = kzalloc(btf_size, GFP_KERNEL);
	if (!raw_btf)
		return -ENOMEM;

	hdr = raw_btf;
	hdr->magic    = BTF_MAGIC;
	hdr->version  = BTF_VERSION;
	hdr->hdr_len  = sizeof(*hdr);
	hdr->type_off = 0;
	hdr->type_len = type_sec_sz;
	hdr->str_off  = type_sec_sz;
	hdr->str_len  = str_sec_sz;

	types_sec = raw_btf   + sizeof(*hdr);
	str_sec   = types_sec + type_sec_sz;
	memcpy(types_sec, igc_md_raw_types, type_sec_sz);
	memcpy(str_sec, names_str, str_sec_sz);

	priv->btf = btf_register(priv->netdev->name, raw_btf, btf_size);
	if (IS_ERR(priv->btf)) {
		err = PTR_ERR(priv->btf);
		priv->btf = NULL;
		netdev_err(priv->netdev, "failed to register BTF MD, err (%d)\n", err);
	}

	kfree(raw_btf);
	return err;
}

int igc_xdp_query_btf(struct net_device *dev, u8 *enabled)
{
	struct igc_adapter *priv = netdev_priv(dev);
	u32 md_btf_id = 0;

	if (!IS_ENABLED(CONFIG_BPF_SYSCALL))
		return md_btf_id;

	if (!priv->btf)
		igc_xdp_register_btf(priv);

	*enabled = !!priv->btf_enabled;
	md_btf_id = priv->btf ? btf_obj_id(priv->btf) : 0;

	return md_btf_id;
}

int igc_xdp_set_btf_md(struct net_device *dev, u8 enable)
{
	struct igc_adapter *priv = netdev_priv(dev);
	int err = 0;

	if (enable && !priv->btf) {
		igc_xdp_register_btf(priv);
		if (!priv->btf) {
			err = -EINVAL;
			goto unlock;
		}
	}

	priv->btf_enabled = enable;
unlock:
	return err;
}

int igc_xdp_set_prog(struct igc_adapter *adapter, struct bpf_prog *prog,
		     struct netlink_ext_ack *extack)
{
	struct net_device *dev = adapter->netdev;
	bool if_running = netif_running(dev);
	struct bpf_prog *old_prog;

	if (dev->mtu > ETH_DATA_LEN) {
		/* For now, the driver doesn't support XDP functionality with
		 * jumbo frames so we return error.
		 */
		NL_SET_ERR_MSG_MOD(extack, "Jumbo frames not supported");
		return -EOPNOTSUPP;
	}

	if (if_running)
		igc_close(dev);

	old_prog = xchg(&adapter->xdp_prog, prog);
	if (old_prog)
		bpf_prog_put(old_prog);

	if (if_running)
		igc_open(dev);

	return 0;
}

static int igc_xdp_enable_pool(struct igc_adapter *adapter,
			       struct xsk_buff_pool *pool, u16 queue_id)
{
	struct net_device *ndev = adapter->netdev;
	struct device *dev = &adapter->pdev->dev;
	struct igc_ring *rx_ring, *tx_ring;
	struct napi_struct *napi;
	bool needs_reset;
	u32 frame_size;
	int err;

	if (queue_id >= adapter->num_rx_queues ||
	    queue_id >= adapter->num_tx_queues)
		return -EINVAL;

	frame_size = xsk_pool_get_rx_frame_size(pool);
	if (frame_size < ETH_FRAME_LEN + VLAN_HLEN * 2) {
		/* When XDP is enabled, the driver doesn't support frames that
		 * span over multiple buffers. To avoid that, we check if xsk
		 * frame size is big enough to fit the max ethernet frame size
		 * + vlan double tagging.
		 */
		return -EOPNOTSUPP;
	}

	err = xsk_pool_dma_map(pool, dev, IGC_RX_DMA_ATTR);
	if (err) {
		netdev_err(ndev, "Failed to map xsk pool\n");
		return err;
	}

	needs_reset = netif_running(adapter->netdev) && igc_xdp_is_enabled(adapter);

	rx_ring = adapter->rx_ring[queue_id];
	tx_ring = adapter->tx_ring[queue_id];
	/* Rx and Tx rings share the same napi context. */
	napi = &rx_ring->q_vector->napi;

	if (needs_reset) {
		igc_disable_rx_ring(rx_ring);
		igc_disable_tx_ring(tx_ring);
		napi_disable(napi);
	}

	set_bit(IGC_RING_FLAG_AF_XDP_ZC, &rx_ring->flags);
	set_bit(IGC_RING_FLAG_AF_XDP_ZC, &tx_ring->flags);

	if (needs_reset) {
		napi_enable(napi);
		igc_enable_rx_ring(rx_ring);
		igc_enable_tx_ring(tx_ring);

		err = igc_xsk_wakeup(ndev, queue_id, XDP_WAKEUP_RX);
		if (err) {
			xsk_pool_dma_unmap(pool, IGC_RX_DMA_ATTR);
			return err;
		}
	}

	return 0;
}

static int igc_xdp_disable_pool(struct igc_adapter *adapter, u16 queue_id)
{
	struct igc_ring *rx_ring, *tx_ring;
	struct xsk_buff_pool *pool;
	struct napi_struct *napi;
	bool needs_reset;

	if (queue_id >= adapter->num_rx_queues ||
	    queue_id >= adapter->num_tx_queues)
		return -EINVAL;

	pool = xsk_get_pool_from_qid(adapter->netdev, queue_id);
	if (!pool)
		return -EINVAL;

	needs_reset = netif_running(adapter->netdev) && igc_xdp_is_enabled(adapter);

	rx_ring = adapter->rx_ring[queue_id];
	tx_ring = adapter->tx_ring[queue_id];
	/* Rx and Tx rings share the same napi context. */
	napi = &rx_ring->q_vector->napi;

	if (needs_reset) {
		igc_disable_rx_ring(rx_ring);
		igc_disable_tx_ring(tx_ring);
		napi_disable(napi);
	}

	xsk_pool_dma_unmap(pool, IGC_RX_DMA_ATTR);
	clear_bit(IGC_RING_FLAG_AF_XDP_ZC, &rx_ring->flags);
	clear_bit(IGC_RING_FLAG_AF_XDP_ZC, &tx_ring->flags);

	if (needs_reset) {
		napi_enable(napi);
		igc_enable_rx_ring(rx_ring);
		igc_enable_tx_ring(tx_ring);
	}

	return 0;
}

int igc_xdp_setup_pool(struct igc_adapter *adapter, struct xsk_buff_pool *pool,
		       u16 queue_id)
{
	return pool ? igc_xdp_enable_pool(adapter, pool, queue_id) :
		      igc_xdp_disable_pool(adapter, queue_id);
}
