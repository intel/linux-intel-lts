// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2022, Intel Corporation. */

#include <linux/module.h>
#include <linux/network_proxy.h>
#include <linux/slab.h>

static struct np_context *np_ctx;

int netprox_send_ipc_msg(int cmd, const char *msg, int size)
{
	struct np_ipc_msg ipc_msg = { {0} };
	struct np_ipcdev *ipcd = np_ctx->np_ipcdev;
	struct np_netdev *nd = np_ctx->np_netdev;

	ipc_msg.ipc_hdr.command = cmd;

	if (size >= NP_IPC_PYLD_MAX) {
		pr_err("netprox send msg over size: %d\n", size);
		return -ENOMEM;
	}

	if (ipcd && nd) {
		if (msg && size) {
			memcpy(ipc_msg.ipc_pyld, msg, size);
			ipc_msg.ipc_hdr.size = size;
		}

		ipcd->ipc_send(np_ctx->np_ipcdev->ipc_cl, (void *)&ipc_msg,
			       ipc_msg.ipc_hdr.size + sizeof(struct np_ipc_hdr),
			       false);
	} else {
		pr_err("Netprox send IPC message failure: incomplete device registration.\n");
		return -ENODEV;
	}

	return 0;
}
EXPORT_SYMBOL(netprox_send_ipc_msg);

int netprox_agent_is_ready(void)
{
	if (!np_ctx->np_ipcdev || !np_ctx->np_netdev)
		return 0;

	return (np_ctx->agent_state == NP_AGENT_READY);
}
EXPORT_SYMBOL(netprox_agent_is_ready);

int netprox_host_proxy_enter(void)
{
	if (np_ctx->agent_state != NP_AGENT_READY) {
		pr_err("Network Proxy Agent is not ready.\n");
		return -EPERM;
	} else if (np_ctx->host_state == NP_HOST_PROXY_ENTER) {
		pr_err("Network Proxy Host is in proxy mode.\n");
		return -EPERM;
	}

	np_ctx->host_state = NP_HOST_PROXY_ENTER;

	return 0;
}
EXPORT_SYMBOL(netprox_host_proxy_enter);

int netprox_host_proxy_exit(void)
{
	if (np_ctx->host_state != NP_HOST_PROXY_ENTER) {
		pr_err("Host is not in proxy mode.\n");
		return -EPERM;
	}

	return netprox_send_ipc_msg(NP_H2A_CMD_PROXY_EXIT, NULL, 0);
}
EXPORT_SYMBOL(netprox_host_proxy_exit);

int netprox_ipc_recv(int cmd, unsigned char *payload, int size)
{
	struct net_device *netdev = np_ctx->np_netdev->netdev;

	/* Process IPC message from Network Proxy Agent */
	switch (cmd) {
	case NP_A2H_CMD_AGENT_READY:
		np_ctx->agent_state = NP_AGENT_READY;
		np_ctx->np_netdev->proxy_wakeup_enable(netdev, 1);
		break;
	case NP_A2H_CMD_HOST_IS_AWAKE:
		/* Ethernet driver's resume function will eventually
		 * call into netprox_host_proxy_exit() function. Thus,
		 * do nothing here.
		 */
		break;
	case NP_A2H_CMD_HOST_IS_EXITED:
		np_ctx->host_state = NP_HOST_PROXY_EXIT;
		break;
	default:
		pr_err("%s unknown command %d\n", __func__, cmd);
		break;
	};
	return 0;
}
EXPORT_SYMBOL(netprox_ipc_recv);

int netprox_register_ipcdev(struct np_ipcdev *np_ipcdev)
{
	np_ctx->np_ipcdev = np_ipcdev;

	return 0;
}
EXPORT_SYMBOL(netprox_register_ipcdev);

int netprox_deregister_ipcdev(struct np_ipcdev *np_ipcdev)
{
	np_ctx->np_ipcdev = NULL;

	return 0;
}
EXPORT_SYMBOL(netprox_deregister_ipcdev);

int netprox_register_netdev(struct np_netdev *np_netdev,
			    void *config, int size)
{
	np_ctx->np_netdev = np_netdev;

	return netprox_send_ipc_msg(NP_H2A_CMD_NETDEV_READY, config, size);
}
EXPORT_SYMBOL(netprox_register_netdev);

int netprox_deregister_netdev(struct np_netdev *np_netdev)
{
	np_ctx->np_netdev = NULL;

	return 0;
}
EXPORT_SYMBOL(netprox_deregister_netdev);

static int __init network_proxy_init(void)
{
	np_ctx = kzalloc(sizeof(*np_ctx), GFP_KERNEL);
	if (!np_ctx)
		return -ENOMEM;

	np_ctx->host_state = NP_HOST_PROXY_EXIT;
	np_ctx->agent_state = NP_AGENT_UNKNOWN;

	return 0;
}

static void __exit network_proxy_deinit(void)
{
	kfree(np_ctx);
}

module_init(network_proxy_init);
module_exit(network_proxy_deinit);

MODULE_DESCRIPTION("Network Proxy");
MODULE_AUTHOR("Song Yoong Siang <yoong.siang.song@intel.com>");
MODULE_AUTHOR("Ong Boon Leong <boon.leong.ong@intel.com>");

MODULE_LICENSE("GPL");
MODULE_ALIAS("networkproxy:*");
