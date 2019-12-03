/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2018, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2,
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Lay, Kuan Loon <kuan.loon.lay@intel.com>
 */

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
irqreturn_t netproxy_irq(int irq, void *dev_id);

#endif /* __STMMAC_NETWORK_PROXY_H__ */
