/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2022, Intel Corporation. */

#ifndef __STMMAC_NETWORK_PROXY_H__
#define __STMMAC_NETWORK_PROXY_H__

/* Proxy mode register in GBE MISC for enter Proxy Mode */
#define GBE_PROXYMODE_REG		0x10600
#define GBE_PROXYMODE_ENTER		BIT(0)

/* Proxy mode exit interrupt register. */
#define GBE_PROXYMODE_EXIT_STS_REG	0x10604
#define GBE_PROXYMODE_EXIT_STS_TRUE	BIT(0)

/* Shared memory for A2H Packets */
#define NETWORK_PROXY_SHMEM_OFFSET	(128 * 1024)
#define NETWORK_PROXY_SHMEM_LEN		(128 * 1024)

int stmmac_netproxy_register(struct net_device *netdev);
int stmmac_netproxy_deregister(struct net_device *netdev);
irqreturn_t netproxy_isr(int irq, void *dev_id);
irqreturn_t netproxy_isr_thread(int irq, void *dev_id);

#endif /* __STMMAC_NETPROXY_H__ */