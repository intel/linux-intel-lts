// SPDX-License-Identifier: GPL-2.0-only
/* Intel virtual Network Interface Card driver (vNIC driver) for
 * ethernet communication over  Intel Integrated Sensor Hub
 * (ISH) using ISH Trasport protocol (ISHTP)
 *
 * Copyright (c) 2019, Intel Corporation.
 */

#include <linux/etherdevice.h>
#include <linux/if_arp.h>
#include <linux/intel-ish-client-if.h>
#include <linux/module.h>
#include <linux/uuid.h>

#define ETH_ISHTP_DRV_NAME		"eth_ishtp"
#define ETH_ISHTP_DRV_DESC		"Intel Ethernet ISHTP Driver"
#define ETH_ISHTP_TX_QUE_LEN		100
#define WAIT_FOR_SEND_SLICE_MS		100
#define WAIT_FOR_SEND_COUNT		10
#define ETH_ISHTP_CL_RX_RING_SIZE	4
#define ETH_ISHTP_CL_TX_RING_SIZE	4
#define ETH_ISHTP_DATA_BUFLEN		984
#define ETH_ISHTP_RX_MAX_LEN		984
#define ETH_ISHTP_TX_MAX_LEN		984
#define ETH_ISHTP_TX_MIN_LEN            14

/* Default timeout period */
#define ETH_ISHTP_TIMEOUT	500 /* In jiffies */
#define ETH_ISHTP_MIN_MTU (ETH_ZLEN - ETH_HLEN)
#define ETH_ISHTP_MAX_MTU  ETH_ISHTP_DATA_BUFLEN

static const guid_t eth_ishtp_uuid =
	GUID_INIT(0xeb83e1fb, 0x4c61, 0x4829,
		  0x98, 0x4c, 0x3, 0x23, 0xab, 0x4b, 0x41, 0x65);

struct eth_ishtp_prv {
	struct device *dev;
	struct net_device *net_dev;
	struct net_device_stats stats;
	unsigned char *tx_buff;
	int tx_len;
	struct ishtp_cl *eth_ishtp_cl;
	struct ishtp_cl_device *cl_device;
	struct ishtp_fw_client *fw_client;
	struct ishtp_cl_rb *rb;
	struct work_struct reset_work;
};

static int eth_ishtp_cl_write(struct eth_ishtp_prv *prv, int *written)
{
	struct ishtp_cl *cl = prv->eth_ishtp_cl;
	unsigned char *buf = prv->tx_buff;
	int len = prv->tx_len;
	int ret;

	ret = ishtp_cl_send(cl, buf, len);
	if (ret)
		goto err_exit;

	*written = len;
	return 0;

err_exit:
	*written = 0;
	dev_err(prv->dev, "[WRITE] Data send error: %d\n", ret);
	return ret;
}

static void eth_ishtp_rx(struct net_device *dev)
{
	struct eth_ishtp_prv *prv = netdev_priv(dev);
	struct ishtp_cl_rb *rb = prv->rb;
	struct sk_buff *skb;


	/*
	 * The packet has been retrieved from the transmission
	 * medium. Build an skb around it, so upper layers can handle it
	 */
	skb = netdev_alloc_skb_ip_align(dev, rb->buffer.size);
	if (!skb) {
		dev_err(prv->dev, "eth_ishtp rx: packet dropped\n");
		dev->stats.rx_dropped++;
		return;
	}

	skb_put(skb, rb->buffer.size);
	skb_copy_to_linear_data(skb, rb->buffer.data, rb->buffer.size);

	/* Write metadata, and then pass to the receive level */
	skb->protocol = eth_type_trans(skb, dev);
	skb->ip_summed = CHECKSUM_NONE;
	dev->stats.rx_packets++;
	dev->stats.rx_bytes += rb->buffer.size;
	netif_rx(skb);
}

static void eth_ishtp_cl_event_cb(struct ishtp_cl_device *cl_device)
{
	struct eth_ishtp_prv *prv = ishtp_get_drvdata(cl_device);
	struct net_device *net_dev = prv->net_dev;
	struct ishtp_cl_rb *rb;

	while ((rb = ishtp_cl_rx_get_rb(prv->eth_ishtp_cl)) != NULL) {
		prv->rb = rb;
		eth_ishtp_rx(net_dev);
		prv->rb = NULL;

		ishtp_cl_io_rb_recycle(rb);
	}


	if (ishtp_cl_tx_empty(prv->eth_ishtp_cl) && netif_queue_stopped(prv->net_dev))
		netif_wake_queue(prv->net_dev);

}

static int eth_ishtp_open(struct net_device *dev)
{

	netif_carrier_on(dev);
	netif_start_queue(dev);
	return 0;
}

static int eth_ishtp_stop(struct net_device *dev)
{
	netif_carrier_off(dev);
	netif_stop_queue(dev);
	return 0;
}

 /* Configuration changes by ifconfig */
static int eth_ishtp_config(struct net_device *dev, struct ifmap *map)
{
	if (dev->flags & IFF_UP) /* can't act on a running interface */
		return -EBUSY;

	return 0;
}

/* Transmit a packet (called by the kernel) */
static int eth_ishtp_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct eth_ishtp_prv *prv = netdev_priv(dev);
	int written;
	int status;

	if (skb->len < ETH_ISHTP_TX_MIN_LEN || skb->len > ETH_ISHTP_TX_MAX_LEN) {
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	if (!netif_running(dev))  {
		dev_info(prv->dev, "xmit: iface is down\n");
		dev_kfree_skb_any(skb);
		netif_stop_queue(dev);
		return NETDEV_TX_OK;
	}

	prv->tx_len = skb->len;
	prv->tx_buff = skb->data;
	status = eth_ishtp_cl_write(prv, &written);

	/* Tx failed */
	if (status) {
		dev_err(prv->dev, "xmit: ishtp_cl_write failed\n");
		netif_stop_queue(dev);
		return NETDEV_TX_BUSY;
	}
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

static void eth_ishtp_tx_timeout(struct net_device *dev)
{
	struct eth_ishtp_prv *prv = netdev_priv(dev);

	dev_err(prv->dev, "eth_ishtp tx timeout\n");
	dev_err(prv->dev, "Transmit timeout at %ld, latency %ld\n",
		 jiffies, dev_trans_start(dev));

	dev->stats.tx_errors++;
	dev->stats.tx_dropped++;

	netif_wake_queue(dev);
}

static int eth_ishtp_dev_init(struct net_device *dev)
{
	struct eth_ishtp_prv *prv = netdev_priv(dev);
	struct ishtp_fw_client *fw_client;
	struct ishtp_device *ishtp_dev;
	struct ishtp_cl *eth_ishtp_cl;
	int ret, ishtp_fw_client_id;

	dev->mtu = ETH_ISHTP_MAX_MTU;
	dev->type = ARPHRD_ETHER;

	eth_ishtp_cl = ishtp_cl_allocate(prv->cl_device);
	if (!eth_ishtp_cl) {
		dev_err(prv->dev, "ishtp cl allocate failed\n");
		ret = -ENOMEM;
		return ret;
	}

	prv->eth_ishtp_cl = eth_ishtp_cl;

	ret = ishtp_cl_link(eth_ishtp_cl);
	if (ret) {
		dev_err(prv->dev, "ishtp_cl_link failed\n");
		ret = -ENOENT;
		goto err_free_cl;
	}

	if (!ishtp_get_ishtp_device(eth_ishtp_cl)) {
		dev_err(prv->dev, "ishtp_get_ishtp_device failed");
		ret = -ENODEV;
		goto err_cl_unlink;
	}

	/* Connect to FW client */
	ishtp_set_tx_ring_size(eth_ishtp_cl, ETH_ISHTP_CL_TX_RING_SIZE);
	ishtp_set_rx_ring_size(eth_ishtp_cl, ETH_ISHTP_CL_RX_RING_SIZE);

	ishtp_dev = ishtp_get_ishtp_device(eth_ishtp_cl);
	fw_client = ishtp_fw_cl_get_client(ishtp_dev, &eth_ishtp_uuid);
	if (!fw_client) {
		dev_err(prv->dev, "FW client not found\n");
		ret = -ENOENT;
		goto err_cl_unlink;
	}

	ishtp_fw_client_id = ishtp_get_fw_client_id(fw_client);
	ishtp_cl_set_fw_client_id(eth_ishtp_cl, ishtp_fw_client_id);
	ishtp_set_connection_state(eth_ishtp_cl, ISHTP_CL_CONNECTING);

	ret = ishtp_register_event_cb(prv->cl_device, eth_ishtp_cl_event_cb);
	if (ret) {
		dev_err(ishtp_device(prv->cl_device),
			"ishtp register callback failed\n");
		free_netdev(prv->net_dev);
		goto err_cl_unlink;
	}

	ret = ishtp_cl_connect(eth_ishtp_cl);
	if (ret) {
		dev_err(prv->dev, "Client connect failed\n");
		goto err_cl_unlink;
	}

	dev_info(prv->dev, "Connected to fw client = %d\n", ishtp_fw_client_id);

	return 0;

err_cl_unlink:
	ishtp_cl_unlink(eth_ishtp_cl);

err_free_cl:
	ishtp_put_device(prv->cl_device);
	ishtp_cl_free(eth_ishtp_cl);
	prv->eth_ishtp_cl = NULL;
	return ret;
}

static void eth_ishtp_dev_uninit(struct net_device *dev)
{
	struct eth_ishtp_prv *prv = netdev_priv(dev);
	int try = WAIT_FOR_SEND_COUNT;
	struct ishtp_cl_rb *rb;
	struct ishtp_cl *cl;
	int ret;

	cancel_work_sync(&prv->reset_work);

	cl = prv->eth_ishtp_cl;
	if (cl) {
		do {
			if (ishtp_cl_tx_empty(cl))
				break;

			msleep_interruptible(WAIT_FOR_SEND_SLICE_MS);
		} while (--try);

		ishtp_set_connection_state(cl, ISHTP_CL_DISCONNECTING);
		ret = ishtp_cl_disconnect(cl);
		ishtp_cl_unlink(cl);
		ishtp_cl_flush_queues(cl);
		ishtp_cl_free(cl);

		prv->eth_ishtp_cl = NULL;
	}

	rb = prv->rb;
	if (rb) {
		ishtp_cl_io_rb_recycle(rb);
		prv->rb = NULL;
	}
}

static const struct net_device_ops eth_ishtp_netdev_ops = {
	.ndo_init            = eth_ishtp_dev_init,
	.ndo_uninit          = eth_ishtp_dev_uninit,
	.ndo_open            = eth_ishtp_open,
	.ndo_stop            = eth_ishtp_stop,
	.ndo_start_xmit      = eth_ishtp_xmit,
	.ndo_set_mac_address = eth_mac_addr,
	.ndo_set_config      = eth_ishtp_config,
	.ndo_tx_timeout      = eth_ishtp_tx_timeout,
	.ndo_validate_addr   = eth_validate_addr
};

static void eth_ishtp_setup(struct net_device *dev)
{
	struct eth_ishtp_prv *prv;

	ether_setup(dev);
	dev->watchdog_timeo = ETH_ISHTP_TIMEOUT;
	dev->netdev_ops = &eth_ishtp_netdev_ops;
	dev->flags           |= IFF_NOARP;
	dev->flags           &= ~(IFF_MULTICAST);
	dev->priv_flags &= ~IFF_TX_SKB_SHARING;
	dev->tx_queue_len = ETH_ISHTP_TX_QUE_LEN;
	dev->min_mtu = ETH_ISHTP_MIN_MTU;
	dev->max_mtu = ETH_ISHTP_MAX_MTU;

	prv = netdev_priv(dev);
}

static void eth_ishtp_reset(struct work_struct *work)
{
	struct ishtp_cl_device *cl_device;
	struct ishtp_fw_client *fw_client;
	struct ishtp_device *ishtp_dev;
	struct ishtp_cl *eth_ishtp_cl;
	struct eth_ishtp_prv *prv;
	int ret, fw_client_id;

	prv = container_of(work, struct eth_ishtp_prv, reset_work);

	cl_device = prv->cl_device;
	eth_ishtp_cl = prv->eth_ishtp_cl;

	if (!ishtp_get_ishtp_device(eth_ishtp_cl)) {
		dev_err(ishtp_device(prv->cl_device),
			"Allocate ishtp_cl failed\n");
		return;
	}

	if (eth_ishtp_cl) {
		ishtp_cl_unlink(eth_ishtp_cl);
		ishtp_cl_flush_queues(eth_ishtp_cl);
		ishtp_cl_free(eth_ishtp_cl);
		eth_ishtp_cl = NULL;
		eth_ishtp_cl = ishtp_cl_allocate(cl_device);
		if (!eth_ishtp_cl) {
			dev_err(ishtp_device(prv->cl_device),
				"Allocate ishtp_cl failed\n");
			return;
		}

		prv->eth_ishtp_cl = eth_ishtp_cl;
		ret = ishtp_cl_link(eth_ishtp_cl);
		if (ret) {
			dev_err(ishtp_device(prv->cl_device),
				"Can not link to ishtp\n");
			goto out_free;
		}

		ishtp_dev = ishtp_get_ishtp_device(prv->eth_ishtp_cl);

		/* Connect to FW client */
		ishtp_set_tx_ring_size(eth_ishtp_cl, ETH_ISHTP_CL_TX_RING_SIZE);
		ishtp_set_rx_ring_size(eth_ishtp_cl, ETH_ISHTP_CL_RX_RING_SIZE);

		fw_client = ishtp_fw_cl_get_client(ishtp_dev, &eth_ishtp_uuid);
		if (!fw_client) {
			dev_err(ishtp_device(prv->cl_device),
				"Don't find related fw client\n");
			ret = -ENOENT;
			goto out_unlink_free;
		}

		fw_client_id = ishtp_get_fw_client_id(fw_client);
		ishtp_cl_set_fw_client_id(eth_ishtp_cl, fw_client_id);
		ishtp_set_connection_state(eth_ishtp_cl, ISHTP_CL_CONNECTING);

		ret = ishtp_cl_connect(eth_ishtp_cl);
		if (ret) {
			dev_err(ishtp_device(prv->cl_device),
				"Connect to fw failed %d\n", ret);
			goto out_unlink_free;
		}

		prv->eth_ishtp_cl = eth_ishtp_cl;
	}

	/* After reset, must register event callback again */
	ishtp_register_event_cb(cl_device, eth_ishtp_cl_event_cb);

	return;

out_unlink_free:
	ishtp_cl_unlink(eth_ishtp_cl);

out_free:
	ishtp_cl_free(eth_ishtp_cl);
	prv->eth_ishtp_cl = NULL;
	dev_err(ishtp_device(prv->cl_device), "Reset failed\n");
}

static int eth_ishtp_cl_reset(struct ishtp_cl_device *cl_device)
{
	struct eth_ishtp_prv *prv;

	prv = (struct eth_ishtp_prv *)ishtp_get_drvdata(cl_device);
	if (!prv)
		return -ENODEV;

	schedule_work(&prv->reset_work);
	return 0;
}

static int eth_ishtp_cl_remove(struct ishtp_cl_device *cl_device)
{
	struct eth_ishtp_prv *prv;

	prv = ishtp_get_drvdata(cl_device);
	if (!prv)
		return -ENODEV;

	unregister_netdev(prv->net_dev);
	free_netdev(prv->net_dev);
	ishtp_put_device(cl_device);
	return 0;
}

static int eth_ishtp_cl_probe(struct ishtp_cl_device *cl_device)
{
	struct eth_ishtp_prv *eth_ishtp_prv;
	struct net_device *net_dev;
	int ret;

	/* Allocate the devices */
	net_dev = alloc_netdev(sizeof(struct eth_ishtp_prv), "ehi%d",
			       NET_NAME_UNKNOWN, eth_ishtp_setup);
	if (!net_dev) {
		ret = -ENOMEM;
		return ret;
	}

	eth_ishtp_prv = netdev_priv(net_dev);
	eth_ishtp_prv->net_dev = net_dev;
	eth_ishtp_prv->dev = &net_dev->dev;
	eth_ishtp_prv->cl_device = cl_device;

	ishtp_set_drvdata(cl_device, eth_ishtp_prv);
	ishtp_get_device(cl_device);

	eth_hw_addr_random(eth_ishtp_prv->net_dev);
	netdev_info(net_dev, "Using random MAC address: %p\n",
		    eth_ishtp_prv->net_dev->dev_addr);

	INIT_WORK(&eth_ishtp_prv->reset_work, eth_ishtp_reset);

	ret = register_netdev(net_dev);
	if (ret) {
		netdev_err(net_dev, "Unable to register net device");
		free_netdev(net_dev);
		return ret;
	}

	return ret;
}

static struct ishtp_cl_driver eth_ishtp_cl_driver = {
	.name = ETH_ISHTP_DRV_NAME,
	.guid = &eth_ishtp_uuid,
	.probe = eth_ishtp_cl_probe,
	.remove = eth_ishtp_cl_remove,
	.reset = eth_ishtp_cl_reset,
};

static int __init eth_ishtp_init(void)
{
	return ishtp_cl_driver_register(&eth_ishtp_cl_driver, THIS_MODULE);
}

static void __exit eth_ishtp_exit(void)
{
	return ishtp_cl_driver_unregister(&eth_ishtp_cl_driver);
}

module_init(eth_ishtp_init);
module_exit(eth_ishtp_exit);

MODULE_DESCRIPTION(ETH_ISHTP_DRV_DESC);

MODULE_AUTHOR("Nachiketa Kumar <nachiketa.kumar@intel.com>");
MODULE_AUTHOR("Sing Nallasellan <singaravelan.nallasellan@intel.com>");

MODULE_LICENSE("GPL");
