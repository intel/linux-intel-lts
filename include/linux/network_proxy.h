/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2022, Intel Corporation. */

#ifndef __NETWORK_PROXY_H__
#define __NETWORK_PROXY_H__

#include <linux/errno.h>
#include <linux/network_proxy_common.h>
#include <linux/printk.h>
#include <linux/types.h>

/* Network Proxy Host States */
enum np_host_state {
	NP_HOST_PROXY_EXIT = 0,
	NP_HOST_PROXY_ENTER,
};

/* Network Proxy Agent States */
enum np_agent_state {
	NP_AGENT_UNKNOWN = 0,
	NP_AGENT_READY,
};

/* IPC Device for Network Proxy Agent */
struct np_ipcdev {
	void *ipc_cl;
	int (*ipc_send)(void *ipc_cl, void *msg, int size, bool async);
};

/* Network Device for Network Proxy Agent */
struct np_netdev {
	struct net_device *netdev;
	int (*proxy_enter)(struct net_device *netdev);
	void (*proxy_wakeup_enable)(struct net_device *netdev, bool enable);
};

/* Network Device for Network Proxy Agent */
struct np_configfs {
	void (*agent_info)(struct np_agent_info *info);
};

/* Network Proxy Context */
struct np_context {
	enum np_host_state host_state;
	enum np_agent_state agent_state;
	struct np_ipcdev *np_ipcdev;
	struct np_netdev *np_netdev;
	struct np_configfs *np_configfs;
	struct np_agent_info np_agent_info;
};

int netprox_agent_is_ready(void);
int netprox_host_proxy_enter(void);
int netprox_host_proxy_exit(void);
int netprox_send_ipc_msg(int cmd, const char *msg, int size);
int netprox_read_rule(struct np_rules *rule, void *ptr, int *size);
int netprox_write_rule(struct np_rules *rule, int size);
int netprox_ipc_recv(int cmd, unsigned char *payload, int size);
int netprox_register_ipcdev(struct np_ipcdev *np_ipcdev);
int netprox_deregister_ipcdev(struct np_ipcdev *np_ipcdev);
int netprox_register_netdev(struct np_netdev *np_netdev, void *config,
			    int size);
int netprox_deregister_netdev(struct np_netdev *np_netdev);
int netprox_register_configfs(struct np_configfs *np_cfs);
int netprox_deregister_configfs(struct np_configfs *np_cfs);

static inline int np_cfs_agent_info(struct np_context *ctx,
				    struct np_agent_info np_agent_info)
{
	if (ctx->np_configfs && np_agent_info.major) {
		ctx->np_configfs->agent_info(&np_agent_info);

		return 0;
	}

	pr_err("Network Proxy Configfs registration fail.\n");

	return -ENODEV;
}

#endif /* __NETWORK_PROXY_H__ */
