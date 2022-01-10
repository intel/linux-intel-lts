// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2022, Intel Corporation. */

#include <linux/netdevice.h>
#include <linux/network_proxy.h>
#include <linux/semaphore.h>
#include <linux/stmmac.h>
#include <linux/workqueue.h>
#include "stmmac.h"
#include "stmmac_netproxy.h"
#include "stmmac_platform.h"

/* network device context attached to network proxy framework */
static struct np_netdev np_netdev = { 0 };
static struct np_shm np_shm = { NULL };

/*  netproxy_isr - Network Proxy interrupt service routine
 *  @irq: interrupt number.
 *  @dev_id: to pass the net device pointer.
 *  Description: ISR to service Network Proxy interrupt.
 */
irqreturn_t netproxy_isr(int irq, void *dev_id)
{
	struct net_device *ndev = (struct net_device *)dev_id;
	struct stmmac_priv *priv = netdev_priv(ndev);
	u32 value;

	value = readl(priv->ioaddr + GBE_PROXYMODE_EXIT_STS_REG);
	writel(value, priv->ioaddr + GBE_PROXYMODE_EXIT_STS_REG);

	if (!netif_running(ndev)) {
		netdev_err(priv->dev,
			   "Netprox exit failed: netdev is not running\n");
		return IRQ_HANDLED;
	}

	return IRQ_WAKE_THREAD;
}

/*  netproxy_isr_thread - Network Proxy ISR thread
 *  @irq: interrupt number.
 *  @dev_id: to pass the net device pointer.
 *  Description: Thread to service Network Proxy interrupt.
 */
irqreturn_t netproxy_isr_thread(int irq, void *dev_id)
{
	int a2h_pkt_hdr_len = sizeof(struct np_a2h_packet_header);
	struct net_device *ndev = (struct net_device *)dev_id;
	int a2h_hdr_len = sizeof(struct np_a2h_pool_header);
	struct stmmac_priv *priv = netdev_priv(ndev);
	struct np_a2h_packet_header a2h_pkt_hdr;
	struct np_a2h_pool_header a2h_hdr;
	struct stmmac_channel *ch;
	void __iomem *a2h_mem_ptr;
	void __iomem *pkt_content;
	struct sk_buff *skb;
	unsigned long flags;
	int i;

	a2h_mem_ptr = priv->ioaddr + NETWORK_PROXY_SHMEM_OFFSET;
	ch = &priv->channel[0];

	/* Get A2H memory pool header */
	memcpy_fromio((void *)&a2h_hdr, a2h_mem_ptr, a2h_hdr_len);
	a2h_mem_ptr += a2h_hdr_len;

	/* TODO: Create workqueue and mutex for each packet */
	/* Attach A2H Rx buffer to sk_buff then use napi_gro_receive() */
	for (i = 0; i < a2h_hdr.total_packets; i++) {
		/* Get A2H packet header */
		memcpy_fromio((void *)&a2h_pkt_hdr, a2h_mem_ptr,
			      a2h_pkt_hdr_len);
		pkt_content = a2h_mem_ptr + a2h_pkt_hdr_len;

		/* Once the length of A2H packet is found to be equal to zero,
		 * all the remaining A2H packets will be ignored.
		 */
		if (!a2h_pkt_hdr.pkt_len) {
			netdev_err(priv->dev,
				   "Netprox failed to submit a2h packets.\n");
			goto err_skb;
		}

		skb = napi_alloc_skb(&ch->rx_napi, a2h_pkt_hdr.pkt_len);
		if (!skb)
			goto err_skb;

		/* Get A2H packet content */
		skb_copy_to_linear_data(skb, pkt_content, a2h_pkt_hdr.pkt_len);

		skb_put(skb, a2h_pkt_hdr.pkt_len);
		skb->protocol = eth_type_trans(skb, priv->dev);
		skb->ip_summed = CHECKSUM_UNNECESSARY;

		/* Submit skbuf to queue 0 */
		skb_record_rx_queue(skb, 0);
		napi_gro_receive(&ch->rx_napi, skb);
		skb = NULL;

		/* Move the pointer the next A2H packet header */
		a2h_mem_ptr += NP_A2H_PKT_MAX + a2h_pkt_hdr_len;
	}

err_skb:
	priv->networkproxy_exit = 1;
	stmmac_resume_common(priv, ndev);
	priv->networkproxy_exit = 0;
	netif_device_attach(ndev);

	if (napi_schedule_prep(&ch->rx_napi)) {
		spin_lock_irqsave(&ch->lock, flags);
		stmmac_disable_dma_irq(priv, priv->ioaddr, 0, 1, 0);
		spin_unlock_irqrestore(&ch->lock, flags);
		__napi_schedule_irqoff(&ch->rx_napi);
	}

	return IRQ_HANDLED;
}

/**
 * stmmac_netprox_suspend - stmmac suspend function for
 * ECMA-393 Network Proxy technology
 * @priv: driver private structure
 * @ndev: net device structure
 * Description: The common function entry to trigger stmmac driver to
 * enter Network Proxy mode. This function can be called from below:-
 * a) Linux PM :- echo mem > /sys/power/state
 * b) User-space Network Proxy library
 */
static int stmmac_netprox_suspend(struct stmmac_priv *priv,
				  struct net_device *ndev)
{
	int result = 1;
	int retry = 5;

	if (!ndev || !netif_running(ndev))
		return 0;

	/* do generic suspend if Network Proxy Agent is not ready */
	if (!netprox_agent_is_ready()) {
		netdev_err(priv->dev, "Netprox is not ready\n");
		return stmmac_suspend_main(priv, ndev);
	}

	/* Check MAC is not WIP in frame transmission from MTL Tx */
	do {
		result = stmmac_mtl_tx_completed(priv, priv->ioaddr,
						 priv->plat->tx_queues_to_use);
		usleep_range(1000, 2000);
	} while (retry-- > 0 && result);

	/* Message Network Proxy Agent to enter proxy mode */
	netprox_host_proxy_enter();

	stmmac_suspend_common(priv, ndev);

	/* Change the destination of MAC controller interrupt and DMA transfer
	 * from Network Proxy Host to Agent.
	 */
	writel(GBE_PROXYMODE_ENTER, priv->ioaddr + GBE_PROXYMODE_REG);

	return 0;
}

/**
 * stmmac_netprox_resume - stmmac resume function for
 * ECMA-393 Network Proxy technology
 * @priv: driver private structure
 * @ndev: net device structure
 * Description: The common function entry to trigger stmmac driver to
 * exit Network Proxy mode. This function can be called from below:-
 * a) Network Proxy Host message Agent to exit proxy mode
 *
 */
static int stmmac_netprox_resume(struct stmmac_priv *priv,
				 struct net_device *ndev)
{
	/* do generic resume if Network Proxy Agent is not ready */
	if (!netprox_agent_is_ready()) {
		netdev_err(priv->dev, "Netprox is not ready\n");
		return stmmac_resume_main(priv, ndev);
	}

	/* Message Network Proxy Agent to exit Proxy mode */
	netprox_host_proxy_exit();

	return 0;
}

/**
 * stmmac_netproxy_wakeup_enable - stmmac network proxy wakeup enable function
 * @ndev: net device structure
 * @enable: 1: enable; 0: disable
 * Description: Enable/disable Network Proxy to wake up system
 */
static void stmmac_netproxy_wakeup_enable(struct net_device *ndev, bool enable)
{
	struct stmmac_priv *priv = netdev_priv(ndev);

	device_set_wakeup_enable(priv->device, enable);
}

/**
 * stmmac_netproxy_register - register to network proxy framework
 * @ndev: net device structure
 * Description: register to network proxy framework after stmmac_open() success
 */
int stmmac_netproxy_register(struct net_device *ndev)
{
	struct stmmac_priv *priv = netdev_priv(ndev);

	/* Allocate workqueue */
	priv->netprox_wq = create_singlethread_workqueue("netprox_wq");
	if (!priv->netprox_wq) {
		dev_err(priv->device, "failed to create netprox workqueue\n");
		return -1;
	}

	np_netdev.netdev = ndev;
	np_netdev.proxy_wakeup_enable = &stmmac_netproxy_wakeup_enable;

	/* TODO: check registration is success */
	netprox_register_netdev(&np_netdev, NULL, 0);

	np_shm.shm_ptr = (char *)priv->ioaddr + NETWORK_PROXY_SHMEM_OFFSET;
	np_shm.shm_max_len = NETWORK_PROXY_SHMEM_LEN;
	netprox_register_shm(&np_shm);

	return 0;
}
EXPORT_SYMBOL(stmmac_netproxy_register);

int stmmac_netproxy_deregister(struct net_device *ndev)
{
	struct stmmac_priv *priv = netdev_priv(ndev);

	if (priv->netprox_wq)
		destroy_workqueue(priv->netprox_wq);

	return 0;
}
EXPORT_SYMBOL(stmmac_netproxy_deregister);

const struct stmmac_pm_ops dwmac_netprox_pm_ops = {
	.suspend = stmmac_netprox_suspend,
	.resume = stmmac_netprox_resume,
};