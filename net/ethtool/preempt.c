// SPDX-License-Identifier: GPL-2.0-only

#include "netlink.h"
#include "common.h"

struct preempt_req_info {
	struct ethnl_req_info		base;
};

struct preempt_reply_data {
	struct ethnl_reply_data		base;
	struct ethtool_fp		fp;
};

#define PREEMPT_REPDATA(__reply_base) \
	container_of(__reply_base, struct preempt_reply_data, base)

const struct nla_policy
ethnl_preempt_get_policy[] = {
	[ETHTOOL_A_PREEMPT_HEADER]		= NLA_POLICY_NESTED(ethnl_header_policy),
};

static int preempt_prepare_data(const struct ethnl_req_info *req_base,
				struct ethnl_reply_data *reply_base,
				struct genl_info *info)
{
	struct preempt_reply_data *data = PREEMPT_REPDATA(reply_base);
	struct net_device *dev = reply_base->dev;
	int ret;

	if (!dev->ethtool_ops->get_preempt)
		return -EOPNOTSUPP;

	ret = ethnl_ops_begin(dev);
	if (ret < 0)
		return ret;

	ret = dev->ethtool_ops->get_preempt(dev, &data->fp);
	ethnl_ops_complete(dev);

	return ret;
}

static int preempt_reply_size(const struct ethnl_req_info *req_base,
			      const struct ethnl_reply_data *reply_base)
{
	int len = 0;

	len += nla_total_size(sizeof(u8)); /* _PREEMPT_ENABLED */
	len += nla_total_size(sizeof(u32)); /* _PREEMPT_ADD_FRAG_SIZE */
	len += nla_total_size(sizeof(u8)); /* _PREEMPT_DISABLE_VERIFY */
	len += nla_total_size(sizeof(u8)); /* _PREEMPT_VERIFIED */

	return len;
}

static int preempt_fill_reply(struct sk_buff *skb,
			      const struct ethnl_req_info *req_base,
			      const struct ethnl_reply_data *reply_base)
{
	const struct preempt_reply_data *data = PREEMPT_REPDATA(reply_base);
	const struct ethtool_fp *preempt = &data->fp;

	if (nla_put_u8(skb, ETHTOOL_A_PREEMPT_ENABLED, preempt->enabled))
		return -EMSGSIZE;

	if (nla_put_u32(skb, ETHTOOL_A_PREEMPT_ADD_FRAG_SIZE,
			preempt->add_frag_size))
		return -EMSGSIZE;

	if (nla_put_u8(skb, ETHTOOL_A_PREEMPT_DISABLE_VERIFY, preempt->disable_verify))
		return -EMSGSIZE;

	if (nla_put_u8(skb, ETHTOOL_A_PREEMPT_VERIFIED, preempt->verified))
		return -EMSGSIZE;

	return 0;
}

const struct ethnl_request_ops ethnl_preempt_request_ops = {
	.request_cmd		= ETHTOOL_MSG_PREEMPT_GET,
	.reply_cmd		= ETHTOOL_MSG_PREEMPT_GET_REPLY,
	.hdr_attr		= ETHTOOL_A_PREEMPT_HEADER,
	.req_info_size		= sizeof(struct preempt_req_info),
	.reply_data_size	= sizeof(struct preempt_reply_data),

	.prepare_data		= preempt_prepare_data,
	.reply_size		= preempt_reply_size,
	.fill_reply		= preempt_fill_reply,
};

const struct nla_policy
ethnl_preempt_set_policy[ETHTOOL_A_PREEMPT_MAX + 1] = {
	[ETHTOOL_A_PREEMPT_HEADER]			= NLA_POLICY_NESTED(ethnl_header_policy),
	[ETHTOOL_A_PREEMPT_ENABLED]			= NLA_POLICY_RANGE(NLA_U8, 0, 1),
	[ETHTOOL_A_PREEMPT_ADD_FRAG_SIZE]		= { .type = NLA_U32 },
	[ETHTOOL_A_PREEMPT_DISABLE_VERIFY]		= NLA_POLICY_RANGE(NLA_U8, 0, 1),
};

int ethnl_set_preempt(struct sk_buff *skb, struct genl_info *info)
{
	struct ethnl_req_info req_info = {};
	struct nlattr **tb = info->attrs;
	struct ethtool_fp preempt = {};
	struct net_device *dev;
	bool mod = false;
	int ret;

	ret = ethnl_parse_header_dev_get(&req_info,
					 tb[ETHTOOL_A_PREEMPT_HEADER],
					 genl_info_net(info), info->extack,
					 true);
	if (ret < 0)
		return ret;
	dev = req_info.dev;
	ret = -EOPNOTSUPP;
	if (!dev->ethtool_ops->get_preempt ||
	    !dev->ethtool_ops->set_preempt)
		goto out_dev;

	rtnl_lock();
	ret = ethnl_ops_begin(dev);
	if (ret < 0)
		goto out_rtnl;

	ret = dev->ethtool_ops->get_preempt(dev, &preempt);
	if (ret < 0) {
		GENL_SET_ERR_MSG(info, "failed to retrieve frame preemption settings");
		goto out_ops;
	}

	ethnl_update_u8(&preempt.enabled,
			tb[ETHTOOL_A_PREEMPT_ENABLED], &mod);
	ethnl_update_u32(&preempt.add_frag_size,
			 tb[ETHTOOL_A_PREEMPT_ADD_FRAG_SIZE], &mod);
	ethnl_update_u8(&preempt.disable_verify,
			tb[ETHTOOL_A_PREEMPT_DISABLE_VERIFY], &mod);
	ret = 0;
	if (!mod)
		goto out_ops;

	ret = dev->ethtool_ops->set_preempt(dev, &preempt, info->extack);
	if (ret < 0) {
		GENL_SET_ERR_MSG(info, "frame preemption settings update failed");
		goto out_ops;
	}

	ethtool_notify(dev, ETHTOOL_MSG_PREEMPT_NTF, NULL);

out_ops:
	ethnl_ops_complete(dev);
out_rtnl:
	rtnl_unlock();
out_dev:
	dev_put(dev);
	return ret;
}
