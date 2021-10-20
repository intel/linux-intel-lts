// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2022, Intel Corporation. */

#include <linux/configfs.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/network_proxy.h>
#include <linux/slab.h>
#include <stddef.h>

#define cfs_agent(item) to_np_cfs(item)->agent_info
#define cfs_rules(item) to_np_cfs(item)->rules

struct np_cfs {
	struct configfs_subsystem subsys;
	struct np_agent_info agent_info;
	struct np_rules rules;
};

static inline struct np_cfs *to_np_cfs(struct config_item *item)
{
	return item ? container_of(to_configfs_subsystem(to_config_group(item)),
				   struct np_cfs, subsys) : NULL;
}

static ssize_t np_cfs_agent_version_show(struct config_item *item, char *page)
{
	return sysfs_emit(page, "%d.%d.%d\n", cfs_agent(item).major,
			  cfs_agent(item).minor,
			  cfs_agent(item).revision);
}

static ssize_t np_cfs_max_cls_rules_show(struct config_item *item, char *page)
{
	return sysfs_emit(page, "%d\n", cfs_agent(item).max_cls_rules);
}

static ssize_t np_cfs_max_resp_rules_show(struct config_item *item, char *page)
{
	return sysfs_emit(page, "%d\n", cfs_agent(item).max_resp_rules);
}

static ssize_t np_cfs_rules_group_show(struct config_item *item, char *page)
{
	return sysfs_emit(page, "%d\n", cfs_rules(item).group);
}

static ssize_t np_cfs_rules_group_store(struct config_item *item,
					const char *page, size_t count)
{
	unsigned long tmp;
	int ret;

	ret = kstrtoul(page, 0, &tmp);
	if (ret)
		return ret;

	cfs_rules(item).group = tmp;

	return count;
}

static ssize_t np_cfs_rules_type_show(struct config_item *item, char *page)
{
	return sysfs_emit(page, "%d\n", cfs_rules(item).type);
}

static ssize_t np_cfs_rules_type_store(struct config_item *item,
				       const char *page, size_t count)
{
	unsigned long tmp;
	int ret;

	ret = kstrtoul(page, 0, &tmp);
	if (ret)
		return ret;

	cfs_rules(item).type = tmp;

	return count;
}

static ssize_t np_cfs_rules_offset_show(struct config_item *item, char *page)
{
	return sysfs_emit(page, "%d\n", cfs_rules(item).offset);
}

static ssize_t np_cfs_rules_offset_store(struct config_item *item,
					 const char *page, size_t count)
{
	unsigned long tmp;
	int ret;

	ret = kstrtoul(page, 0, &tmp);
	if (ret)
		return ret;

	if (tmp < 0) {
		pr_err("Invalid offset\n");
		return -EINVAL;
	}

	cfs_rules(item).offset = tmp;

	return count;
}

static ssize_t np_cfs_rules_size_show(struct config_item *item, char *page)
{
	return sysfs_emit(page, "%d\n", cfs_rules(item).size);
}

static ssize_t np_cfs_rules_size_store(struct config_item *item,
				       const char *page, size_t count)
{
	unsigned long tmp;
	int ret;

	ret = kstrtoul(page, 0, &tmp);
	if (ret)
		return ret;

	if (tmp <= 0) {
		pr_err("Invalid size\n");
		return -EINVAL;
	}
	cfs_rules(item).size = tmp;

	return count;
}

static ssize_t np_cfs_rules_value_show(struct config_item *item, char *page)
{
	struct np_rules *rule;
	void *content;
	int size;
	int ret;

	rule = &(cfs_rules(item));
	size = rule->size;

	/* allocate buffer for data copy */
	content = kzalloc(size, GFP_KERNEL);
	if (!content)
		return -ENOMEM;

	/* actual read size store in size */
	ret = netprox_read_rule(rule, content, &size);
	if (ret == 0 && size > 0) {
		memcpy(page, content, size);
		ret = size;
	}
	kfree(content);

	return ret;
}

static ssize_t np_cfs_rules_value_store(struct config_item *item,
					const char *page, size_t count)
{
	struct np_rules *rule;
	void *content;
	int size;
	int ret;

	/* allocate rule, value is dynamic size according to count */
	size = sizeof(struct np_rules) + count;
	rule = kzalloc(size, GFP_KERNEL);
	if (!rule)
		return -ENOMEM;

	/* copy the configfs np_rules group, type, offset, size */
	memcpy(rule, &(cfs_rules(item)), sizeof(struct np_rules));

	/* copy the content */
	content = rule + 1;
	memcpy(content, page, count);

	ret = netprox_write_rule(rule, size);
	if (!ret)
		ret = count;

	kfree(rule);

	return ret;
}

static ssize_t np_cfs_ipc_send_show(struct config_item *item, char *page)
{
	return -EPERM;
}

static ssize_t np_cfs_ipc_send_store(struct config_item *item,
				     const char *page, size_t count)
{
	struct np_ipc_hdr *ipc_hdr = (struct np_ipc_hdr *)page;
	int ret;

	ret = netprox_send_ipc_msg(ipc_hdr->command,
				   (page + sizeof(struct np_ipc_hdr)),
				   count - sizeof(struct np_ipc_hdr));
	if (ret != 0)
		return ret;

	return count;
}

CONFIGFS_ATTR_RO(np_cfs_, agent_version);
CONFIGFS_ATTR_RO(np_cfs_, max_cls_rules);
CONFIGFS_ATTR_RO(np_cfs_, max_resp_rules);
CONFIGFS_ATTR(np_cfs_, rules_group);
CONFIGFS_ATTR(np_cfs_, rules_type);
CONFIGFS_ATTR(np_cfs_, rules_offset);
CONFIGFS_ATTR(np_cfs_, rules_size);
CONFIGFS_ATTR(np_cfs_, rules_value);
CONFIGFS_ATTR(np_cfs_, ipc_send);

static struct configfs_attribute *np_cfs_attrs[] = {
	&np_cfs_attr_agent_version,
	&np_cfs_attr_max_cls_rules,
	&np_cfs_attr_max_resp_rules,
	&np_cfs_attr_rules_group,
	&np_cfs_attr_rules_type,
	&np_cfs_attr_rules_offset,
	&np_cfs_attr_rules_size,
	&np_cfs_attr_rules_value,
	&np_cfs_attr_ipc_send,
	NULL,
};

static const struct config_item_type np_cfs_type = {
	.ct_attrs	= np_cfs_attrs,
	.ct_owner	= THIS_MODULE,
};

static struct np_cfs np_subsys = {
	.subsys = {
		.su_group = {
			.cg_item = {
				.ci_namebuf = "networkproxy",
				.ci_type = &np_cfs_type,
			},
		},
	},
};

void netprox_configfs_agent_info(struct np_agent_info *info)
{
	np_subsys.agent_info.major = info->major;
	np_subsys.agent_info.minor = info->minor;
	np_subsys.agent_info.revision = info->revision;
	np_subsys.agent_info.max_cls_rules = info->max_cls_rules;
	np_subsys.agent_info.max_resp_rules = info->max_resp_rules;
}

static struct np_configfs np_configfs = {
	.agent_info = &netprox_configfs_agent_info,
};

int netprox_configfs_init(void)
{
	struct configfs_subsystem *subsys = &np_subsys.subsys;
	int ret;

	ret = netprox_register_configfs(&np_configfs);
	if (ret) {
		pr_err("Error %d while registering Netprox configfs.\n",
		       ret);
		return ret;
	}

	config_group_init(&subsys->su_group);
	mutex_init(&subsys->su_mutex);
	ret = configfs_register_subsystem(subsys);
	if (ret) {
		pr_err("Error %d while registering subsystem %s\n",
		       ret,
		       subsys->su_group.cg_item.ci_namebuf);
		goto out_unregister;
	}

	return 0;

out_unregister:
	configfs_unregister_subsystem(subsys);

	return ret;
}
EXPORT_SYMBOL(netprox_configfs_init);

void netprox_configfs_deinit(void)
{
	struct configfs_subsystem *subsys = &np_subsys.subsys;

	configfs_unregister_subsystem(subsys);

	netprox_deregister_configfs(&np_configfs);
}
EXPORT_SYMBOL(netprox_configfs_deinit);

module_init(netprox_configfs_init);
module_exit(netprox_configfs_deinit);

MODULE_DESCRIPTION("Network Proxy Configfs");
MODULE_AUTHOR("Song Yoong Siang <yoong.siang.song@intel.com>");
MODULE_AUTHOR("Ong Boon Leong <boon.leong.ong@intel.com>");

MODULE_LICENSE("GPL");
MODULE_ALIAS("networkproxy:*");
