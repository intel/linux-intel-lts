// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2022, Intel Corporation. */

#include <linux/delay.h>
#include <linux/inetdevice.h>
#include <linux/ipv6.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/network_proxy.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <net/addrconf.h>
#include <net/if_inet6.h>

/* timeout 1 second */
#define NP_TIMEOUT_MS	1000

static struct np_context *np_ctx;
static struct task_struct *agent_access_task;
static unsigned char *ipc_ptr;

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

static int netprox_read_from_agent(struct np_rules *rule, void *content,
				   int *size)
{
	struct np_rules *ipc_resp;
	void *ipc_res;
	long time;

	netprox_send_ipc_msg(NP_H2A_CMD_READ_CLS_RULE, (const char *)rule,
			     sizeof(struct np_rules));

	agent_access_task = current;
	set_current_state(TASK_INTERRUPTIBLE);
	time = schedule_timeout(msecs_to_jiffies(NP_TIMEOUT_MS));
	if (time > 0) {
		/* If time > 0 mean Agent response
		 * NP_A2H_CMD_READ_CLS_RULE_RESULT, check the response
		 * is same as query
		 */
		ipc_resp = (struct np_rules *)ipc_ptr;
		if (rule->group == ipc_resp->group &&
		    rule->type == ipc_resp->type &&
		    rule->offset == ipc_resp->offset &&
		    rule->size == ipc_resp->size) {
			/* only copy the content without struct np_rules */
			ipc_res = ipc_resp + 1;
			memcpy(content, ipc_res, ipc_resp->size);
		}
	} else {
		pr_err("netprox read rule timeout\n");
		return -EPERM;
	}

	return 0;
}

static int netprox_process_classifier_rule_read(struct np_rules *rule,
						void *content, int *size)
{
	int ret;

	switch (rule->type) {
	case NP_RL_T_IPV4:
	case NP_RL_T_IPV6:
	case NP_RL_T_SNMP:
	case NP_RL_T_TCP_WAKE_PORT:
	case NP_RL_T_MDNS:
		ret = netprox_read_from_agent(rule, content, size);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int netprox_process_mib_rule_read(struct np_rules *rule, void *content,
					 int *size)
{
	int ret;

	switch (rule->type) {
	case NP_RL_T_MAC_ADDR:
	case NP_RL_T_IPV4:
	case NP_RL_T_IPV6:
	case NP_RL_T_SNMP_COMMUNITY_STR:
	case NP_RL_T_TCP_WAKE_PORT:
	case NP_RL_T_IPV4_SUBNET:
		ret = netprox_read_from_agent(rule, content, size);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

int netprox_read_rule(struct np_rules *rule, void *content, int *size)
{
	int ret;

	switch (rule->group) {
	case NP_RL_G_CLS:
		ret = netprox_process_classifier_rule_read(rule, content, size);
		break;
	case NP_RL_G_MIB:
		ret = netprox_process_mib_rule_read(rule, content, size);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}
EXPORT_SYMBOL(netprox_read_rule);

static int netprox_send_netdev_mib(int rule_type)
{
	struct inet6_ifaddr *ifp, *tmp;
	struct in_device *indevice;
	struct in_ifaddr *inifaddr;
	struct net_device *ndev;
	struct inet6_dev *idev;
	struct np_rules *rule;
	int size, i;
	int ret = 0;
	char *ptr;

	if (np_ctx->np_netdev) {
		ndev = np_ctx->np_netdev->netdev;
	} else {
		pr_err("netprox net device not register\n");
		return -ENODEV;
	}

	size = 0;

	switch (rule_type) {
	case NP_RL_T_MAC_ADDR:
		size = sizeof(struct np_rules) + NP_MAC_ADDR_BYTES;
		break;
	case NP_RL_T_IPV4_SUBNET:
	case NP_RL_T_IPV4:
		indevice = ndev->ip_ptr;
		if (indevice) {
			inifaddr = indevice->ifa_list;
			if (inifaddr) {
				/* only set size if ipv4 info is valid */
				size = sizeof(struct np_rules)
				       + NP_IPV4_ADDR_BYTES;
			} else {
				pr_err("IPV4 MIB ifa_list is null\n");
			}
		} else {
			pr_err("IPV4 MIB ip_ptr is null\n");
		}
		break;
	case NP_RL_T_IPV6:
		idev = ndev->ip6_ptr;
		if (idev) {
			/* only set size if ipv6 info is valid */
			size = sizeof(struct np_rules) +
				(NP_IPV6_ADDR_ARRAY * NP_IPV6_ADDR_BYTES);
		} else {
			pr_err("IPV6 MIB ip6_ptr is null\n");
		}
		break;
	default:
		pr_err("unknown MIB type\n");
		break;
	}

	if (size == 0)
		return -EPERM;

	rule = kzalloc(size, GFP_KERNEL);
	if (!rule)
		return -ENOMEM;

	/* Set rule of MIB */
	rule->group = NP_RL_G_MIB;
	rule->type = rule_type;
	rule->offset = 0;
	ptr = (char *)rule->value;

	switch (rule_type) {
	case NP_RL_T_MAC_ADDR:
		memcpy(ptr, ndev->dev_addr, NP_MAC_ADDR_BYTES);
		break;
	case NP_RL_T_IPV4:
		indevice = ndev->ip_ptr;
		inifaddr = indevice->ifa_list;
		memcpy(ptr, (void *)&inifaddr->ifa_address, NP_IPV4_ADDR_BYTES);
		break;
	case NP_RL_T_IPV6:
		i = 0;
		idev = ndev->ip6_ptr;
		size = sizeof(struct np_rules);
		list_for_each_entry_safe(ifp, tmp, &idev->addr_list,
					 if_list) {
			if (i++ == NP_IPV6_ADDR_ARRAY) {
				pr_err("more than %d IPV6 addr\n",
				       NP_IPV6_ADDR_ARRAY);
				break;
			}
			size += NP_IPV6_ADDR_BYTES;
			memcpy(ptr, (void *)&ifp->addr, NP_IPV6_ADDR_BYTES);
			ptr += NP_IPV6_ADDR_BYTES;
		}
		break;
	case NP_RL_T_IPV4_SUBNET:
		indevice = ndev->ip_ptr;
		inifaddr = indevice->ifa_list;
		memcpy(ptr, (void *)&inifaddr->ifa_mask, NP_IPV4_ADDR_BYTES);
		break;
	default:
		ret = -EPERM;
		pr_err("unknown MIB type\n");
		break;
	}

	if (ret) {
		kfree(rule);
		return ret;
	}

	rule->size = size - sizeof(struct np_rules);
	ret = netprox_send_ipc_msg(NP_H2A_CMD_WRITE_CLS_RULE,
				   (const char *)rule,
				   size);
	kfree(rule);

	return ret;
}

static int netprox_process_classifier_rule_write(struct np_rules *rule,
						 int size)
{
	unsigned short *value;
	int ret = 0;

	switch (rule->type) {
	case NP_RL_T_IPV4:
	case NP_RL_T_IPV6:
		/* Check if IPv4/IPv6 rule is enable then send IPv4/IPv6 info */
		value = (unsigned short *)(rule->value);
		if (*value & NP_RL_CLS_ENABLE) {
			ret = netprox_send_netdev_mib(NP_RL_T_MAC_ADDR);
			if (ret == 0)
				ret = netprox_send_netdev_mib(rule->type);
			if (ret == 0)
				ret =
				netprox_send_netdev_mib(NP_RL_T_IPV4_SUBNET);
		}
		break;
	default:
		break;
	}

	ret |= netprox_send_ipc_msg(NP_H2A_CMD_WRITE_CLS_RULE,
				    (const char *)rule,
				    size);
	return ret;
}

int netprox_write_rule(struct np_rules *rule, int size)
{
	int ret;

	switch (rule->group) {
	case NP_RL_G_CLS:
		ret = netprox_process_classifier_rule_write(rule, size);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}
EXPORT_SYMBOL(netprox_write_rule);

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
	case NP_A2H_CMD_AGENT_INFO:
		if (size == sizeof(struct np_agent_info))
			memcpy(&np_ctx->np_agent_info, payload, size);
		break;
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
	case NP_A2H_CMD_READ_CLS_RULE_RESULT:
		ipc_ptr = payload;
		if (agent_access_task)
			wake_up_process(agent_access_task);
		else
			pr_err("Received cls_rule_result after timeout.\n");
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

int netprox_register_configfs(struct np_configfs *np_configfs)
{
	np_ctx->np_configfs = np_configfs;

	return np_cfs_agent_info(np_ctx, np_ctx->np_agent_info);
}
EXPORT_SYMBOL(netprox_register_configfs);

int netprox_deregister_configfs(struct np_configfs *np_configfs)
{
	np_ctx->np_configfs = NULL;

	return 0;
}
EXPORT_SYMBOL(netprox_deregister_configfs);

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
