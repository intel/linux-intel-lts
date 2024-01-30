// SPDX-License-Identifier: MIT
/*
 * Copyright(c) 2019 - 2023 Intel Corporation.
 *
 */

#include <linux/types.h>
#include <linux/netlink.h>
#include <net/genetlink.h>
#include <linux/skbuff.h>
#include <net/sock.h>

#include "csr.h"
#include "iaf_drv.h"
#include "io.h"
#include "mbdb.h"
#include "netlink.h"
#include "ops.h"
#include "port.h"
#include "routing_engine.h"
#include "routing_event.h"
#include "trace.h"

#define IAF_ATTR_MAX (_IAF_ATTR_COUNT - 1)
#define MSG_ENLARGE_RETRIES 4

/*
 * Prefix definitions:
 * nla_ - netlink attribute
 * genl_ - generic netlink
 * nl_ - this modules names
 */

typedef enum cmd_rsp (*nl_process_op_cb_t)(struct sk_buff *msg,
		      struct genl_info *info);

struct nl_device_enum_cb_args {
	struct sk_buff *msg;
	u16 entries;
};

static struct genl_family nl_iaf_family;

static DEFINE_MUTEX(nl_lock);

static enum cmd_rsp nl_add_rsp_attrs(struct sk_buff *msg, struct nlattr *context, enum cmd_rsp rsp)
{
	if (nla_put_u8(msg, IAF_ATTR_CMD_OP_RESULT, rsp) ||
	    nla_put_u8(msg, IAF_ATTR_CMD_OP_MSG_TYPE, IAF_CMD_MSG_RESPONSE)) {
		return IAF_CMD_RSP_MSGSIZE;
	}

	/* Client need not specify a context attribute */
	if (context)
		if (nla_put_u64_64bit(msg, IAF_ATTR_CMD_OP_CONTEXT,
				      nla_get_u64(context), IAF_ATTR_PAD))
			return IAF_CMD_RSP_MSGSIZE;

	return IAF_CMD_RSP_SUCCESS;
}

static enum cmd_rsp nl_alloc_msg(struct genl_info *info, size_t msg_sz,
				 struct sk_buff **msg, void **usrhdr)
{
	*msg = genlmsg_new(msg_sz, GFP_KERNEL);
	if (!*msg)
		return IAF_CMD_RSP_NOMEM;

	*usrhdr = genlmsg_put_reply(*msg, info, &nl_iaf_family, 0, info->genlhdr->cmd);
	if (!*usrhdr) {
		nlmsg_free(*msg);
		return IAF_CMD_RSP_NOMEM;
	}

	return IAF_CMD_RSP_SUCCESS;
}

static enum cmd_rsp nl_send_reply(struct genl_info *info, struct sk_buff *msg, void *usrhdr,
				  enum cmd_rsp rsp)
{
	enum cmd_rsp ret;

	ret = nl_add_rsp_attrs(msg, info->attrs[IAF_ATTR_CMD_OP_CONTEXT], rsp);
	if (ret != IAF_CMD_RSP_SUCCESS) {
		nlmsg_free(msg);
		return IAF_CMD_RSP_MSGSIZE;
	}

	genlmsg_end(msg, usrhdr);

	if (genlmsg_reply(msg, info)) {
		ret = IAF_CMD_RSP_FAILURE;
		pr_err("Unable to send reply msg.\n");
		nlmsg_free(msg);
	}

	return ret;
}

static enum cmd_rsp nl_alloc_process_and_send_reply(struct genl_info *info, size_t msg_sz,
						    nl_process_op_cb_t op_cb)
{
	struct sk_buff *msg;
	void *usrhdr;
	enum cmd_rsp ret;

	if (nl_alloc_msg(info, msg_sz, &msg, &usrhdr) != IAF_CMD_RSP_SUCCESS)
		return IAF_CMD_RSP_NOMEM;

	ret = op_cb(msg, info);
	if (ret != IAF_CMD_RSP_SUCCESS) {
		nlmsg_free(msg);
		return ret;
	}

	return nl_send_reply(info, msg, usrhdr, IAF_CMD_RSP_SUCCESS);
}

static enum cmd_rsp nl_send_error_reply(struct genl_info *info, enum cmd_rsp rsp)
{
	struct sk_buff *msg;
	void *usrhdr;

	if (nl_alloc_msg(info, NLMSG_GOODSIZE, &msg, &usrhdr) != IAF_CMD_RSP_SUCCESS)
		return IAF_CMD_RSP_NOMEM;

	return nl_send_reply(info, msg, usrhdr, rsp);
}

static enum cmd_rsp nl_process_op_req(struct genl_info *info, nl_process_op_cb_t op_cb)
{
	enum cmd_rsp ret = IAF_CMD_RSP_FAILURE;
	size_t msg_sz;
	int retries;

	trace_nl_rsp(info->genlhdr->cmd, NLMSG_GOODSIZE, info->snd_seq);

	/* If we run out of space in the message, try to enlarge it */
	for (msg_sz = NLMSG_GOODSIZE, retries = MSG_ENLARGE_RETRIES; retries; retries--,
	     msg_sz += NLMSG_GOODSIZE) {
		pr_debug("netlink response message buffer size is %lu\n", msg_sz);
		ret = nl_alloc_process_and_send_reply(info, msg_sz, op_cb);
		if (ret == IAF_CMD_RSP_SUCCESS)
			return ret;

		/* Only enlarge if msg was too small for the response */
		if (ret != IAF_CMD_RSP_MSGSIZE)
			break;
	}

	return nl_send_error_reply(info, ret);
}

static int nl_process_query(struct sk_buff *msg, struct genl_info *info, nl_process_op_cb_t op_cb)
{
	enum cmd_rsp ret;

	mutex_lock(&nl_lock);

	trace_nl_req(info->genlhdr->cmd, info->nlhdr->nlmsg_len, info->snd_seq);

	if (info->genlhdr->version != INTERFACE_VERSION) {
		ret = nl_send_error_reply(info, IAF_CMD_RSP_INVALID_INTERFACE_VERSION);
		goto unlock;
	}

	if (!info->attrs[IAF_ATTR_CMD_OP_MSG_TYPE]) {
		ret = nl_send_error_reply(info, IAF_CMD_RSP_MISSING_MSG_TYPE);
		goto unlock;
	}

	if (nla_get_u8(info->attrs[IAF_ATTR_CMD_OP_MSG_TYPE]) != IAF_CMD_MSG_REQUEST) {
		ret = nl_send_error_reply(info, IAF_CMD_RSP_MSG_TYPE_NOT_REQUEST);
		goto unlock;
	}

	ret = nl_process_op_req(info, op_cb);

unlock:
	mutex_unlock(&nl_lock);
	return ret == IAF_CMD_RSP_SUCCESS ? 0 : -EIO;
}

static enum cmd_rsp nl_get_sd(struct genl_info *info, struct fsubdev **sd)
{
	struct nlattr *fabric_id_attr = info->attrs[IAF_ATTR_FABRIC_ID];
	struct nlattr *sd_idx_attr = info->attrs[IAF_ATTR_SD_INDEX];

	if (!fabric_id_attr)
		return IAF_CMD_RSP_MISSING_FABRIC_ID;

	if (!sd_idx_attr)
		return IAF_CMD_RSP_MISSING_SD_INDEX;

	*sd = find_sd_id(nla_get_u32(fabric_id_attr), nla_get_u8(sd_idx_attr));
	if (IS_ERR(*sd)) {
		if (PTR_ERR(*sd) == -ENODEV)
			return IAF_CMD_RSP_UNKNOWN_FABRIC_ID;

		return IAF_CMD_RSP_SD_INDEX_OUT_OF_RANGE;
	}

	if (dev_is_runtime_debug((*sd)->fdev))
		return IAF_CMD_RSP_UNKNOWN_FABRIC_ID;

	/* Check that all sd's for this fabric device have been initialized */
	if (!smp_load_acquire(&(*sd)->fdev->all_sds_inited)) {
		fdev_put((*sd)->fdev);
		return IAF_CMD_RSP_AGAIN;
	}

	return IAF_CMD_RSP_SUCCESS;
}

static enum cmd_rsp nl_get_fdev(struct genl_info *info, struct fdev **dev)
{
	struct nlattr *fabric_id = info->attrs[IAF_ATTR_FABRIC_ID];

	if (!fabric_id)
		return IAF_CMD_RSP_MISSING_FABRIC_ID;

	*dev = fdev_find(nla_get_u32(fabric_id));
	if (!(*dev))
		return IAF_CMD_RSP_UNKNOWN_FABRIC_ID;

	if (dev_is_runtime_debug(*dev)) {
		fdev_put(*dev);
		return IAF_CMD_RSP_UNKNOWN_FABRIC_ID;
	}

	return IAF_CMD_RSP_SUCCESS;
}

static enum cmd_rsp nl_add_fabric_device_attrs(struct sk_buff *msg,
					       struct fdev *dev)
{
	const struct iaf_pdata *pd = dev->pd;

	if (nla_put_string(msg, IAF_ATTR_DEV_NAME,
			   dev_name(&dev->pdev->dev)) ||
	    nla_put_string(msg, IAF_ATTR_PARENT_DEV_NAME,
			   dev_name(dev->pdev->dev.parent)) ||

	    nla_put_u8(msg, IAF_ATTR_PCI_SLOT_NUM, pd->slot) ||
	    nla_put_u8(msg, IAF_ATTR_SOCKET_ID, pd->socket_id) ||
	    nla_put_u8(msg, IAF_ATTR_VERSION, pd->version) ||
	    nla_put_u8(msg, IAF_ATTR_PRODUCT_TYPE, pd->product) ||
	    nla_put_u8(msg, IAF_ATTR_SUBDEVICE_COUNT, pd->sd_cnt))
		return IAF_CMD_RSP_MSGSIZE;

	return IAF_CMD_RSP_SUCCESS;
}

static enum cmd_rsp nl_add_fabric_ports(struct sk_buff *msg, struct fsubdev *sd)
{
	struct fport *port;
	u8 lpn;

	for_each_fabric_port(port, lpn, sd) {
		struct nlattr *nested_attr;

		nested_attr = nla_nest_start(msg, IAF_ATTR_FABRIC_PORT);
		if (!nested_attr)
			return IAF_CMD_RSP_MSGSIZE;

		if (nla_put_u8(msg, IAF_ATTR_FABRIC_PORT_NUMBER, lpn) ||
		    nla_put_u8(msg, IAF_ATTR_FABRIC_PORT_TYPE, port->port_type)) {
			nla_nest_cancel(msg, nested_attr);
			return IAF_CMD_RSP_MSGSIZE;
		}

		nla_nest_end(msg, nested_attr);
	}

	return IAF_CMD_RSP_SUCCESS;
}

static enum cmd_rsp nl_add_bridge_ports(struct sk_buff *msg, struct fsubdev *sd)
{
	u8 lpn;

	for_each_bridge_lpn(lpn, sd)
		if (nla_put_u8(msg, IAF_ATTR_BRIDGE_PORT_NUMBER, lpn))
			return IAF_CMD_RSP_MSGSIZE;

	return IAF_CMD_RSP_SUCCESS;
}

static enum cmd_rsp nl_add_sub_device_attrs(struct sk_buff *msg,
					    struct fsubdev *sd)
{
	if (nla_put_u64_64bit(msg, IAF_ATTR_GUID, sd->guid, IAF_ATTR_PAD) ||
	    nla_put_u8(msg, IAF_ATTR_EXTENDED_PORT_COUNT,
		       sd->extended_port_cnt) ||
	    nla_put_u8(msg, IAF_ATTR_FABRIC_PORT_COUNT, sd->port_cnt) ||
	    nla_put_u8(msg, IAF_ATTR_SWITCH_LIFETIME,
		       FIELD_GET(SLT_PSC_EP0_SWITCH_LIFETIME,
				 sd->switchinfo.slt_psc_ep0)) ||
	    nla_put_u8(msg, IAF_ATTR_ROUTING_MODE_SUPPORTED,
		       sd->switchinfo.routing_mode_supported) ||
	    nla_put_u8(msg, IAF_ATTR_ROUTING_MODE_ENABLED,
		       sd->switchinfo.routing_mode_enabled) ||
	    nla_put_u8(msg, IAF_ATTR_EHHANCED_PORT_0_PRESENT,
		       FIELD_GET(SLT_PSC_EP0_ENHANCED_PORT_0,
				 sd->switchinfo.slt_psc_ep0)))
		return IAF_CMD_RSP_MSGSIZE;

	if (nl_add_fabric_ports(msg, sd))
		return IAF_CMD_RSP_MSGSIZE;

	return nl_add_bridge_ports(msg, sd);
}

static int nl_process_device_enum_cb(struct fdev *dev, void *args)
{
	struct nl_device_enum_cb_args *device_enum_args = args;
	struct sk_buff *msg = device_enum_args->msg;

	struct nlattr *nested_attr =
		nla_nest_start(msg, IAF_ATTR_FABRIC_DEVICE);

	if (dev_is_runtime_debug(dev))
		return -ENODEV;

	if (!nested_attr)
		return -EMSGSIZE;

	if (nla_put_u32(msg, IAF_ATTR_FABRIC_ID, dev->fabric_id)) {
		nla_nest_cancel(msg, nested_attr);
		return -EMSGSIZE;
	}

	if (nl_add_fabric_device_attrs(msg, dev)) {
		nla_nest_cancel(msg, nested_attr);
		return -EMSGSIZE;
	}

	nla_nest_end(msg, nested_attr);

	device_enum_args->entries++;

	return 0;
}

static enum cmd_rsp nl_process_device_enum(struct sk_buff *msg,
					   struct genl_info *info)
{
	struct nl_device_enum_cb_args device_enum_args;

	device_enum_args.msg = msg;
	device_enum_args.entries = 0;

	/* Get each fdev from xarray and fill in nested information */
	if (fdev_process_each(nl_process_device_enum_cb, &device_enum_args))
		return IAF_CMD_RSP_MSGSIZE;

	if (nla_put_u16(msg, IAF_ATTR_ENTRIES, device_enum_args.entries))
		return IAF_CMD_RSP_MSGSIZE;

	return IAF_CMD_RSP_SUCCESS;
}

static enum cmd_rsp nl_port_bitmap(struct fsubdev *sd, struct genl_info *info,
				   unsigned long *port_mask)
{
	struct nlattr *nla;
	int remaining;

	nlmsg_for_each_attr(nla, info->nlhdr, GENL_HDRLEN, remaining)
		if (nla_type(nla) == IAF_ATTR_FABRIC_PORT_NUMBER) {
			u8 lpn = nla_get_u8(nla);

			if (!get_fport_handle(sd, lpn))
				return IAF_CMD_RSP_PORT_RANGE_ERROR;

			set_bit(lpn, port_mask);
		}

	return IAF_CMD_RSP_SUCCESS;
}

static enum cmd_rsp nl_process_set_port_state(struct sk_buff *msg,
					      struct genl_info *info)
{
	struct fsubdev *sd;
	DECLARE_BITMAP(port_mask, PORT_COUNT) = {};
	enum cmd_rsp ret;

	/* Get the sd to access */
	ret = nl_get_sd(info, &sd);
	if (ret != IAF_CMD_RSP_SUCCESS)
		return ret;

	ret = nl_port_bitmap(sd, info, port_mask);
	if (ret == IAF_CMD_RSP_SUCCESS) {
		switch (info->genlhdr->cmd) {
		case IAF_CMD_OP_PORT_ENABLE:
			if (enable_fports(sd, port_mask, PORT_COUNT) == -EAGAIN)
				ret = IAF_CMD_RSP_AGAIN;
			break;
		case IAF_CMD_OP_PORT_DISABLE:
			if (disable_fports(sd, port_mask, PORT_COUNT) == -EAGAIN)
				ret = IAF_CMD_RSP_AGAIN;
			break;
		case IAF_CMD_OP_PORT_USAGE_ENABLE:
			ret = enable_usage_fports(sd, port_mask, PORT_COUNT);
			break;
		case IAF_CMD_OP_PORT_USAGE_DISABLE:
			if (disable_usage_fports(sd, port_mask, PORT_COUNT) == -EAGAIN)
				ret = IAF_CMD_RSP_AGAIN;
			break;
		default:
			ret = IAF_CMD_RSP_FAILURE;
			break;
		}
	}

	fdev_put(sd->fdev);
	return ret;
}

static enum cmd_rsp nl_set_port_beacon(struct sk_buff *msg, struct fsubdev *sd,
				       u8 lpn, bool enable)
{
	struct fport *port = get_fport_handle(sd, lpn);
	bool beacon_enabled;
	enum cmd_rsp err;

	if (!port)
		return IAF_CMD_RSP_PORT_RANGE_ERROR;

	beacon_enabled = test_bit(PORT_CONTROL_BEACONING, port->controls);

	if ((enable && beacon_enabled) || (!enable && !beacon_enabled))
		return IAF_CMD_RSP_SUCCESS;

	err = ops_linkmgr_port_beacon_set(sd, lpn, enable);
	if (err)
		return IAF_CMD_RSP_MAIL_BOX_ERROR;

	change_bit(PORT_CONTROL_BEACONING, port->controls);

	return IAF_CMD_RSP_SUCCESS;
}

static enum cmd_rsp nl_process_set_port_beacon_state(struct sk_buff *msg,
						     struct genl_info *info)
{
	bool enable = info->genlhdr->cmd == IAF_CMD_OP_PORT_BEACON_ENABLE;
	struct fsubdev *sd;
	DECLARE_BITMAP(port_mask, PORT_COUNT) = {};
	enum cmd_rsp ret;
	u8 lpn;

	/* Get the sd to access */
	ret = nl_get_sd(info, &sd);
	if (ret != IAF_CMD_RSP_SUCCESS)
		return ret;

	ret = nl_port_bitmap(sd, info, port_mask);
	if (ret == IAF_CMD_RSP_SUCCESS) {
		/*
		 * When the caller does not specify any ports, all lpns are
		 * walked and it's corresponding control_bit setting is set to
		 * "enable" for that port's lpn.
		 * Otherwise, only lpns specified in the message are examined
		 * and set to "enable".
		 */
		if (bitmap_empty(port_mask, PORT_COUNT)) {
			for_each_fabric_lpn(lpn, sd) {
				ret = nl_set_port_beacon(msg, sd, lpn, enable);
				if (ret != IAF_CMD_RSP_SUCCESS)
					break;
			}
		} else {
			for_each_set_bit(lpn, port_mask, PORT_COUNT) {
				ret = nl_set_port_beacon(msg, sd, lpn, enable);
				if (ret != IAF_CMD_RSP_SUCCESS)
					break;
			}
		}
	}

	fdev_put(sd->fdev);
	return ret;
}

static enum cmd_rsp nl_add_enabled_state(struct sk_buff *msg, u8 lpn,
					 bool enabled)
{
	struct nlattr *nested_attr;

	/* The caller guarantees the lpn is a valid fport and won't be NULL */

	nested_attr = nla_nest_start(msg, IAF_ATTR_FABRIC_PORT);
	if (!nested_attr)
		return IAF_CMD_RSP_MSGSIZE;

	if (nla_put_u8(msg, IAF_ATTR_FABRIC_PORT_NUMBER, lpn) ||
	    nla_put_u8(msg, IAF_ATTR_ENABLED_STATE, enabled)) {
		nla_nest_cancel(msg, nested_attr);
		return IAF_CMD_RSP_MSGSIZE;
	}

	nla_nest_end(msg, nested_attr);

	return IAF_CMD_RSP_SUCCESS;
}

static enum cmd_rsp nl_process_query_port_control_state(struct sk_buff *msg,
							struct genl_info *info)
{
	struct fsubdev *sd;
	struct fport *port;
	int bit;
	DECLARE_BITMAP(port_mask, PORT_COUNT) = {};
	unsigned long *mask;
	enum cmd_rsp ret;
	u8 lpn;

	/* Get the sd to access */
	ret = nl_get_sd(info, &sd);
	if (ret != IAF_CMD_RSP_SUCCESS)
		return ret;

	ret = nl_port_bitmap(sd, info, port_mask);
	if (ret != IAF_CMD_RSP_SUCCESS)
		goto exit;

	switch (info->genlhdr->cmd) {
	case IAF_CMD_OP_PORT_STATE_QUERY:
		bit = PORT_CONTROL_ENABLED;
		break;
	case IAF_CMD_OP_PORT_USAGE_STATE_QUERY:
		bit = PORT_CONTROL_ROUTABLE;
		break;
	case IAF_CMD_OP_PORT_BEACON_STATE_QUERY:
		bit = PORT_CONTROL_BEACONING;
		break;
	default:
		ret = IAF_CMD_RSP_FAILURE;
		goto exit;
	}

	/*
	 * When the caller does not specify any ports, all lpns are
	 * walked and it's corresponding control_bit setting is added to
	 * the response for that port's lpn.
	 * Otherwise, only lpns specified in the message are examined
	 * and added to the response.
	 */
	if (bitmap_empty(port_mask, PORT_COUNT)) {
		for_each_fabric_port(port, lpn, sd) {
			mask = port->controls;

			ret = nl_add_enabled_state(msg, lpn, test_bit(bit, mask));
			if (ret != IAF_CMD_RSP_SUCCESS)
				break;
		}
	} else {
		for_each_masked_port(port, lpn, sd->port, port_mask, PORT_COUNT) {
			mask = port->controls;

			ret = nl_add_enabled_state(msg, lpn, test_bit(bit, mask));
			if (ret != IAF_CMD_RSP_SUCCESS)
				break;
		}
	}

exit:
	fdev_put(sd->fdev);
	return ret;
}

static enum cmd_rsp nl_process_port_routed_query(struct sk_buff *msg,
						 struct genl_info *info)
{
	struct fsubdev *sd;
	DECLARE_BITMAP(port_mask, PORT_COUNT) = {};
	DECLARE_BITMAP(usage_mask, PORT_COUNT) = {};
	u8 lpn;
	enum cmd_rsp ret;

	/* Get the sd to access */
	ret = nl_get_sd(info, &sd);
	if (ret != IAF_CMD_RSP_SUCCESS)
		return ret;

	ret = nl_port_bitmap(sd, info, port_mask);
	if (ret == IAF_CMD_RSP_SUCCESS) {
		routing_port_routed_query(sd, port_mask, usage_mask);

		for_each_set_bit(lpn, port_mask, PORT_COUNT) {
			ret = nl_add_enabled_state(msg, lpn,
						   test_bit(lpn, usage_mask));
			if (ret != IAF_CMD_RSP_SUCCESS)
				break;
		}
	}

	fdev_put(sd->fdev);
	return ret;
}

static enum cmd_rsp nl_process_rem_request(struct sk_buff *msg,
					   struct genl_info *info)
{
	rem_request();

	return IAF_CMD_RSP_SUCCESS;
}

static enum cmd_rsp nl_process_routing_gen_query(struct sk_buff *msg,
						 struct genl_info *info)
{
	u32 counter_start, counter_end;

	routing_generation_read(&counter_start, &counter_end);

	if (nla_put_u32(msg, IAF_ATTR_ROUTING_GEN_START, counter_start) ||
	    nla_put_u32(msg, IAF_ATTR_ROUTING_GEN_END, counter_end))
		return IAF_CMD_RSP_MSGSIZE;

	return IAF_CMD_RSP_SUCCESS;
}

static enum cmd_rsp nl_process_fabric_device_properties(struct sk_buff *msg,
							struct genl_info *info)
{
	struct fdev *dev = NULL;
	enum cmd_rsp ret;

	ret = nl_get_fdev(info, &dev);
	if (ret)
		return ret;

	ret = nl_add_fabric_device_attrs(msg, dev);

	fdev_put(dev);
	return ret;
}

static enum cmd_rsp nl_process_sub_device_properties_get(struct sk_buff *msg,
							 struct genl_info *info)
{
	struct fsubdev *sd;
	enum cmd_rsp ret;

	/* Get the sd to access */
	ret = nl_get_sd(info, &sd);
	if (ret != IAF_CMD_RSP_SUCCESS)
		return ret;

	ret = nl_add_sub_device_attrs(msg, sd);

	fdev_put(sd->fdev);
	return ret;
}

static enum cmd_rsp nl_add_fport_status(struct sk_buff *msg, u8 lpn,
					struct fsubdev *sd)
{
	struct fport_status status;
	struct nlattr *nested_attr;
	enum cmd_rsp err;

	err = get_fport_status(sd, lpn, &status);
	if (err)
		return IAF_CMD_RSP_PORT_RANGE_ERROR;

	/* The caller guarantees the lpn is a valid fport and won't be NULL */

	nested_attr = nla_nest_start(msg, IAF_ATTR_FABRIC_PORT);
	if (!nested_attr)
		return IAF_CMD_RSP_MSGSIZE;

	if (nla_put_u8(msg, IAF_ATTR_FABRIC_PORT_NUMBER, lpn) ||
	    nla_put_u8(msg, IAF_ATTR_FPORT_HEALTH, status.health)) {
		nla_nest_cancel(msg, nested_attr);
		return IAF_CMD_RSP_MSGSIZE;
	}

	switch (status.health) {
	case FPORT_HEALTH_OFF:
	case FPORT_HEALTH_HEALTHY:
		break;

	case FPORT_HEALTH_FAILED:
		if (test_bit(FPORT_ERROR_FAILED, status.errors))
			if (nla_put_u8(msg, IAF_ATTR_FPORT_ERROR_FAILED, 1))
				err = IAF_CMD_RSP_MSGSIZE;

		if (test_bit(FPORT_ERROR_ISOLATED, status.errors))
			if (nla_put_u8(msg, IAF_ATTR_FPORT_ERROR_ISOLATED, 1))
				err = IAF_CMD_RSP_MSGSIZE;

		if (test_bit(FPORT_ERROR_FLAPPING, status.errors))
			if (nla_put_u8(msg, IAF_ATTR_FPORT_ERROR_FLAPPING, 1))
				err = IAF_CMD_RSP_MSGSIZE;

		if (test_bit(FPORT_ERROR_LINK_DOWN, status.errors))
			if (nla_put_u8(msg, IAF_ATTR_FPORT_ERROR_LINK_DOWN, 1))
				err = IAF_CMD_RSP_MSGSIZE;

		if (test_bit(FPORT_ERROR_DID_NOT_TRAIN, status.errors))
			if (nla_put_u8(msg, IAF_ATTR_FPORT_ERROR_DID_NOT_TRAIN,
				       1))
				err = IAF_CMD_RSP_MSGSIZE;
		break;

	case FPORT_HEALTH_DEGRADED:
		if (test_bit(FPORT_ISSUE_LQI, status.issues))
			if (nla_put_u8(msg, IAF_ATTR_FPORT_ISSUE_LQI, 1))
				err = IAF_CMD_RSP_MSGSIZE;

		if (test_bit(FPORT_ISSUE_LWD, status.issues))
			if (nla_put_u8(msg, IAF_ATTR_FPORT_ISSUE_LWD, 1))
				err = IAF_CMD_RSP_MSGSIZE;

		if (test_bit(FPORT_ISSUE_RATE, status.issues))
			if (nla_put_u8(msg, IAF_ATTR_FPORT_ISSUE_RATE, 1))
				err = IAF_CMD_RSP_MSGSIZE;
		break;
	}

	if (err) {
		nla_nest_cancel(msg, nested_attr);
		return err;
	}

	nla_nest_end(msg, nested_attr);

	return IAF_CMD_RSP_SUCCESS;
}

static enum cmd_rsp nl_process_fport_status_query(struct sk_buff *msg,
						  struct genl_info *info)
{
	struct fsubdev *sd;
	DECLARE_BITMAP(port_mask, PORT_COUNT) = {};
	enum cmd_rsp ret;
	u8 lpn;

	/* Get the sd to access */
	ret = nl_get_sd(info, &sd);
	if (ret != IAF_CMD_RSP_SUCCESS)
		return ret;

	ret = nl_port_bitmap(sd, info, port_mask);
	if (ret == IAF_CMD_RSP_SUCCESS) {
		/*
		 * When the caller does not specify any ports, all lpns are
		 * walked and it's corresponding nested status setting is added
		 * to the response for that port's lpn.
		 * Otherwise, only lpns specified in the message are examined
		 * and added to the response.
		 */

		if (bitmap_empty(port_mask, PORT_COUNT)) {
			for_each_fabric_lpn(lpn, sd) {
				ret = nl_add_fport_status(msg, lpn, sd);
				if (ret != IAF_CMD_RSP_SUCCESS)
					break;
			}
		} else {
			for_each_set_bit(lpn, port_mask, PORT_COUNT) {
				ret = nl_add_fport_status(msg, lpn, sd);
				if (ret != IAF_CMD_RSP_SUCCESS)
					break;
			}
		}
	}

	fdev_put(sd->fdev);
	return ret;
}

static enum cmd_rsp nl_process_sub_device_trap_count_query(struct sk_buff *msg,
							   struct genl_info *info)
{
	struct fsubdev *sd;
	enum cmd_rsp ret;
	u64 psc_trap_count;
	u64 lwd_trap_count;
	u64 lqi_trap_count;
	u64 qsfp_fault_trap_count;
	u64 qsfp_present_trap_count;

	/* Get the sd to access */
	ret = nl_get_sd(info, &sd);
	if (ret != IAF_CMD_RSP_SUCCESS)
		return ret;

	psc_trap_count = atomic64_read(&sd->psc_trap_count);
	lwd_trap_count = atomic64_read(&sd->lwd_trap_count);
	lqi_trap_count = atomic64_read(&sd->lqi_trap_count);
	qsfp_fault_trap_count = atomic64_read(&sd->qsfp_fault_trap_count);
	qsfp_present_trap_count = atomic64_read(&sd->qsfp_present_trap_count);

	if (nla_put_u64_64bit(msg, IAF_ATTR_SUB_DEVICE_TRAP_COUNT,
			      psc_trap_count + lwd_trap_count + lqi_trap_count +
			      qsfp_fault_trap_count + qsfp_present_trap_count, IAF_ATTR_PAD) ||
	    nla_put_u64_64bit(msg, IAF_ATTR_SUB_DEVICE_PORT_STATE_CHANGE_TRAP_COUNT,
			      psc_trap_count, IAF_ATTR_PAD) ||
	    nla_put_u64_64bit(msg, IAF_ATTR_SUB_DEVICE_PORT_LWD_TRAP_COUNT,
			      lwd_trap_count, IAF_ATTR_PAD) ||
	    nla_put_u64_64bit(msg, IAF_ATTR_SUB_DEVICE_PORT_LQI_TRAP_COUNT,
			      lqi_trap_count, IAF_ATTR_PAD) ||
	    nla_put_u64_64bit(msg, IAF_ATTR_SUB_DEVICE_QSFP_MGR_FAULT_TRAP_COUNT,
			      qsfp_fault_trap_count, IAF_ATTR_PAD) ||
	    nla_put_u64_64bit(msg, IAF_ATTR_SUB_DEVICE_QSFP_MGR_PORT_PRESENT_TRAP_COUNT,
			      qsfp_present_trap_count, IAF_ATTR_PAD))
		ret = IAF_CMD_RSP_MSGSIZE;

	fdev_put(sd->fdev);
	return ret;
}

static enum cmd_rsp nl_add_fport_properties(struct sk_buff *msg, u8 lpn,
					    struct fsubdev *sd)
{
	struct nlattr *nested_attr;
	struct fport *fabric_port = get_fport_handle(sd, lpn);
	struct portinfo *port_info;

	/*
	 * The caller guarantees the lpn is a valid fport and won't be NULL
	 * but guard against the condition anyway.
	 * There should be no reason to include logging an error.
	 */
	if (!fabric_port)
		return IAF_CMD_RSP_PORT_RANGE_ERROR;

	port_info = fabric_port->portinfo;

	nested_attr = nla_nest_start(msg, IAF_ATTR_FABRIC_PORT);
	if (!nested_attr)
		return IAF_CMD_RSP_MSGSIZE;

	if (nla_put_u8(msg, IAF_ATTR_FABRIC_PORT_NUMBER, lpn) ||
	    nla_put_u8(msg, IAF_ATTR_FABRIC_PORT_TYPE, fabric_port->port_type) ||
	    nla_put_u8(msg, IAF_ATTR_FPORT_PM_PORT_STATE, fabric_port->state) ||
	    nla_put_u8(msg, IAF_ATTR_FPORT_ROUTED,
		       atomic_read(&fabric_port->routed)) ||
	    nla_put_u8(msg, IAF_ATTR_FPORT_LOGICAL_STATE,
		       fabric_port->log_state) ||
	    nla_put_u8(msg, IAF_ATTR_FPORT_PHYSICAL_STATE,
		       fabric_port->phys_state) ||
	    nla_put_u32(msg, IAF_ATTR_FPORT_FID, port_info->fid) ||
	    nla_put_u32(msg, IAF_ATTR_FPORT_LINK_DOWN_COUNT,
			port_info->link_down_count) ||
	    nla_put_u64_64bit(msg, IAF_ATTR_FPORT_NEIGHBOR_GUID,
			      port_info->neighbor_guid, IAF_ATTR_PAD) ||
	    nla_put_u32(msg, IAF_ATTR_FPORT_PORT_ERROR_ACTION,
			port_info->port_error_action) ||
	    nla_put_u8(msg, IAF_ATTR_FPORT_NEIGHBOR_PORT_NUMBER,
		       port_info->neighbor_port_number) ||
	    nla_put_u8(msg, IAF_ATTR_FPORT_PORT_LINK_MODE_ACTIVE,
		       port_info->port_link_mode_active) ||
	    nla_put_u8(msg, IAF_ATTR_FPORT_NEIGHBOR_LINK_DOWN_REASON,
		       port_info->neighbor_link_down_reason) ||
	    nla_put_u8(msg, IAF_ATTR_FPORT_H_O_Q_LIFETIME,
		       port_info->h_o_q_lifetime) ||
	    nla_put_u8(msg, IAF_ATTR_FPORT_VL_CAP, port_info->vl_cap) ||
	    nla_put_u8(msg, IAF_ATTR_FPORT_OPERATIONAL_VLS,
		       port_info->operational_vls) ||
	    nla_put_u8(msg, IAF_ATTR_FPORT_NEIGHBOR_MTU,
		       port_info->neighbor_mtu) ||
	    nla_put_u8(msg, IAF_ATTR_FPORT_LTP_CRC_MODE_SUPPORTED,
		       port_info->crc_mode_supported) ||
	    nla_put_u8(msg, IAF_ATTR_FPORT_LTP_CRC_MODE_ENABLED,
		       port_info->crc_mode_enabled) ||
	    nla_put_u8(msg, IAF_ATTR_FPORT_LTP_CRC_MODE_ACTIVE,
		       port_info->crc_mode_active) ||
	    nla_put_u8(msg, IAF_ATTR_FPORT_LINK_WIDTH_SUPPORTED,
		       port_info->link_width_supported) ||
	    nla_put_u8(msg, IAF_ATTR_FPORT_LINK_WIDTH_ENABLED,
		       port_info->link_width_enabled) ||
	    nla_put_u8(msg, IAF_ATTR_FPORT_LINK_WIDTH_ACTIVE,
		       port_info->link_width_active) ||
	    nla_put_u8(msg, IAF_ATTR_FPORT_LINK_SPEED_SUPPORTED,
		       port_info->link_speed_supported) ||
	    nla_put_u8(msg, IAF_ATTR_FPORT_LINK_SPEED_ENABLED,
		       port_info->link_speed_enabled) ||
	    nla_put_u8(msg, IAF_ATTR_FPORT_LINK_SPEED_ACTIVE,
		       port_info->link_speed_active) ||
	    nla_put_u8(msg, IAF_ATTR_FPORT_LINK_WIDTH_DOWNGRADE_RX_ACTIVE,
		       port_info->link_width_downgrade_rx_active) ||
	    nla_put_u8(msg, IAF_ATTR_FPORT_LINK_WIDTH_DOWNGRADE_TX_ACTIVE,
		       port_info->link_width_downgrade_tx_active) ||
	    nla_put_u8(msg, IAF_ATTR_FPORT_LINK_INIT_REASON,
		       port_info->link_init_reason) ||
	    nla_put_u8(msg, IAF_ATTR_FPORT_LINK_DOWN_REASON,
		       port_info->link_down_reason) ||
	    nla_put_u8(msg, IAF_ATTR_FPORT_LQI_OFFLINE_DISABLED_REASON,
		       FIELD_GET(OLDR_NN_LQI_OFFLINE_DISABLED_REASON,
				 port_info->oldr_nn_lqi)) ||
	    nla_put_u8(msg, IAF_ATTR_FPORT_LQI_NEIGHBOR_NORMAL,
		       FIELD_GET(OLDR_NN_LQI_NEIGHBOR_NORMAL,
				 port_info->oldr_nn_lqi)) ||
	    nla_put_u8(msg, IAF_ATTR_FPORT_LINK_QUALITY_INDICATOR,
		       FIELD_GET(OLDR_NN_LQI_LINK_QUALITY_INDICATOR,
				 port_info->oldr_nn_lqi)) ||
	    nla_put_u64_64bit(msg, IAF_ATTR_FPORT_BPS_LINK_SPEED_MAX,
			      bps_link_speed(port_info->link_speed_enabled),
			      IAF_ATTR_PAD) ||
	    nla_put_u64_64bit(msg, IAF_ATTR_FPORT_BPS_LINK_SPEED_ACTIVE,
			      bps_link_speed(port_info->link_speed_active),
			      IAF_ATTR_PAD) ||
	    nla_put_u32(msg, IAF_ATTR_FPORT_LQI_CHANGE_COUNT,
			port_info->lqi_change_count)) {
		nla_nest_cancel(msg, nested_attr);
		return IAF_CMD_RSP_MSGSIZE;
	}

	nla_nest_end(msg, nested_attr);

	return IAF_CMD_RSP_SUCCESS;
}

static enum cmd_rsp nl_process_fport_properties_query(struct sk_buff *msg,
						      struct genl_info *info)
{
	struct fsubdev *sd;
	DECLARE_BITMAP(port_mask, PORT_COUNT) = {};
	enum cmd_rsp ret;
	u8 lpn;

	/* Get the sd to access */
	ret = nl_get_sd(info, &sd);
	if (ret != IAF_CMD_RSP_SUCCESS)
		return ret;

	ret = nl_port_bitmap(sd, info, port_mask);
	if (ret == IAF_CMD_RSP_SUCCESS) {
		/*
		 * When the caller does not specify any ports, all lpns are
		 * walked and it's corresponding nested status setting is added
		 * to the response for that port's lpn.
		 * Otherwise, only lpns specified in the message are examined
		 * and added to the response.
		 */
		down_write(&routable_lock); /* exclusive lock */

		if (bitmap_empty(port_mask, PORT_COUNT)) {
			for_each_fabric_lpn(lpn, sd) {
				ret = nl_add_fport_properties(msg, lpn, sd);
				if (ret != IAF_CMD_RSP_SUCCESS)
					break;
			}
		} else {
			for_each_set_bit(lpn, port_mask, PORT_COUNT) {
				ret = nl_add_fport_properties(msg, lpn, sd);
				if (ret != IAF_CMD_RSP_SUCCESS)
					break;
			}
		}

		up_write(&routable_lock);
	}

	fdev_put(sd->fdev);
	return ret;
}

static enum cmd_rsp nl_get_throughput(struct sk_buff *msg, struct fsubdev *sd, u8 lpn)
{
	struct fport *port;
	u64 rsp_area[3];
	struct mbdb_op_port_status_get_rsp *rsp = (struct mbdb_op_port_status_get_rsp *)rsp_area;
	static struct mbdb_op_csr_range csr_ranges[] = {
		{ .offset = O_FPC_PORTRCV_DATA_CNT,
		  .num_csrs = 1
		},
		{ .offset = TP_PRF_XMIT_DATA_OFFSET,
		  .num_csrs = 1
		},
	};

	port = get_fport_handle(sd, lpn);
	if (!port)
		return IAF_CMD_RSP_PORT_RANGE_ERROR;

	if (ops_port_status_get(sd, lpn, ARRAY_SIZE(csr_ranges),
				csr_ranges, rsp))
		return IAF_CMD_RSP_MAIL_BOX_ERROR;

	if (nla_put_u64_64bit(msg, IAF_ATTR_TIMESTAMP, rsp->cp_free_run_rtc,
			      IAF_ATTR_PAD) ||
	    nla_put_u64_64bit(msg, IAF_ATTR_FPORT_RX_BYTES,
			      flits_to_bytes(rsp->regs[0]), IAF_ATTR_PAD) ||
	    nla_put_u64_64bit(msg, IAF_ATTR_FPORT_TX_BYTES,
			      flits_to_bytes(rsp->regs[1]), IAF_ATTR_PAD))
		return IAF_CMD_RSP_MSGSIZE;

	return IAF_CMD_RSP_SUCCESS;
}

static enum cmd_rsp nl_process_fport_xmit_recv_counts(struct sk_buff *msg,
						      struct genl_info *info)
{
	struct fsubdev *sd;
	enum cmd_rsp ret;

	if (!info->attrs[IAF_ATTR_FABRIC_PORT_NUMBER])
		return IAF_CMD_RSP_MISSING_PORT_NUMBER;

	/* Get the sd to access */
	ret = nl_get_sd(info, &sd);
	if (ret != IAF_CMD_RSP_SUCCESS)
		return ret;

	ret = nl_get_throughput(msg, sd, nla_get_u8(info->attrs[IAF_ATTR_FABRIC_PORT_NUMBER]));

	fdev_put(sd->fdev);

	return ret;
}

static enum cmd_rsp nl_process_fport_throughput(struct sk_buff *msg, struct genl_info *info)
{
	struct nlattr *nla;
	int remaining;
	int remaining_nested;
	enum cmd_rsp ret = IAF_CMD_RSP_SUCCESS;
	struct fsubdev *sd;

	nlmsg_for_each_attr(nla, info->nlhdr, GENL_HDRLEN, remaining) {
		if (nla_type(nla) == IAF_ATTR_FABRIC_PORT) {
			struct nlattr *fabric_id_attr;
			struct nlattr *sd_idx_attr;
			struct nlattr *lpn_attr;
			struct nlattr *cur;
			struct nlattr *nested_attr;

			fabric_id_attr = NULL;
			sd_idx_attr = NULL;
			lpn_attr = NULL;

			nla_for_each_nested(cur, nla, remaining_nested) {
				switch (nla_type(cur)) {
				case IAF_ATTR_FABRIC_ID:
					fabric_id_attr = cur;
					break;
				case IAF_ATTR_SD_INDEX:
					sd_idx_attr = cur;
					break;
				case IAF_ATTR_FABRIC_PORT_NUMBER:
					lpn_attr = cur;
					break;
				default:
					break;
				}
			}

			if (!fabric_id_attr)
				return IAF_CMD_RSP_MISSING_FABRIC_ID;

			if (!sd_idx_attr)
				return IAF_CMD_RSP_MISSING_SD_INDEX;

			if (!lpn_attr)
				return IAF_CMD_RSP_MISSING_PORT_NUMBER;

			sd = find_sd_id(nla_get_u32(fabric_id_attr), nla_get_u8(sd_idx_attr));
			if (IS_ERR(sd)) {
				if (PTR_ERR(sd) == -ENODEV)
					return IAF_CMD_RSP_UNKNOWN_FABRIC_ID;

				return IAF_CMD_RSP_SD_INDEX_OUT_OF_RANGE;
			}

			nested_attr = nla_nest_start(msg, IAF_ATTR_FABRIC_PORT_THROUGHPUT);
			if (!nested_attr) {
				fdev_put(sd->fdev);
				return IAF_CMD_RSP_MSGSIZE;
			}

			if (nla_put_u32(msg, IAF_ATTR_FABRIC_ID,
					nla_get_u32(fabric_id_attr)) ||
				nla_put_u8(msg, IAF_ATTR_SD_INDEX, nla_get_u8(sd_idx_attr)) ||
				nla_put_u8(msg, IAF_ATTR_FABRIC_PORT_NUMBER,
					   nla_get_u8(lpn_attr))) {
				fdev_put(sd->fdev);
				nla_nest_cancel(msg, nested_attr);
				return IAF_CMD_RSP_MSGSIZE;
			}

			ret = nl_get_throughput(msg, sd, nla_get_u8(lpn_attr));

			fdev_put(sd->fdev);

			if (ret) {
				nla_nest_cancel(msg, nested_attr);
				return ret;
			}

			nla_nest_end(msg, nested_attr);
		}
	}

	return IAF_CMD_RSP_SUCCESS;
}

static const struct nla_policy nl_iaf_policy_basic[IAF_ATTR_MAX + 1] = {
	[IAF_ATTR_CMD_OP_MSG_TYPE] = { .type = NLA_U8 },
	[IAF_ATTR_CMD_OP_CONTEXT] = { .type = NLA_U64 },
};

static const struct nla_policy nl_iaf_policy_fabric_id[IAF_ATTR_MAX + 1] = {
	[IAF_ATTR_CMD_OP_MSG_TYPE] = { .type = NLA_U8 },
	[IAF_ATTR_CMD_OP_CONTEXT] = { .type = NLA_U64 },
	[IAF_ATTR_FABRIC_ID] = { .type = NLA_U32 },
};

static const struct nla_policy nl_iaf_policy_fabric_id_sd_index[IAF_ATTR_MAX + 1] = {
	[IAF_ATTR_CMD_OP_MSG_TYPE] = { .type = NLA_U8 },
	[IAF_ATTR_CMD_OP_CONTEXT] = { .type = NLA_U64 },
	[IAF_ATTR_FABRIC_ID] = { .type = NLA_U32 },
	[IAF_ATTR_SD_INDEX] = { .type = NLA_U8 },
};

static const struct nla_policy nl_iaf_policy_fabric_id_sd_index_port[IAF_ATTR_MAX + 1] = {
	[IAF_ATTR_CMD_OP_MSG_TYPE] = { .type = NLA_U8 },
	[IAF_ATTR_CMD_OP_CONTEXT] = { .type = NLA_U64 },
	[IAF_ATTR_FABRIC_ID] = { .type = NLA_U32 },
	[IAF_ATTR_SD_INDEX] = { .type = NLA_U8 },
	[IAF_ATTR_FABRIC_PORT_NUMBER] = { .type = NLA_U8 },
};

static const struct nla_policy nl_iaf_policy_fabric_id_sd_index_nested_port[IAF_ATTR_MAX + 1] = {
	[IAF_ATTR_CMD_OP_MSG_TYPE] = { .type = NLA_U8 },
	[IAF_ATTR_CMD_OP_CONTEXT] = { .type = NLA_U64 },
	[IAF_ATTR_FABRIC_PORT] = { .type = NLA_NESTED},
	[IAF_ATTR_FABRIC_ID] = { .type = NLA_U32 },
	[IAF_ATTR_SD_INDEX] = { .type = NLA_U8 },
	[IAF_ATTR_FABRIC_PORT_NUMBER] = { .type = NLA_U8 },
};

static int nl_device_enum_op(struct sk_buff *msg, struct genl_info *info)
{
	return nl_process_query(msg, info, nl_process_device_enum);
}

static int nl_port_enable_op(struct sk_buff *msg, struct genl_info *info)
{
	return nl_process_query(msg, info, nl_process_set_port_state);
}

static int nl_port_disable_op(struct sk_buff *msg, struct genl_info *info)
{
	return nl_process_query(msg, info, nl_process_set_port_state);
}

static int nl_port_state_query_op(struct sk_buff *msg, struct genl_info *info)
{
	return nl_process_query(msg, info, nl_process_query_port_control_state);
}

static int nl_port_usage_enable_op(struct sk_buff *msg, struct genl_info *info)
{
	return nl_process_query(msg, info, nl_process_set_port_state);
}

static int nl_port_usage_disable_op(struct sk_buff *msg, struct genl_info *info)
{
	return nl_process_query(msg, info, nl_process_set_port_state);
}

static int nl_port_usage_state_query_op(struct sk_buff *msg, struct genl_info *info)
{
	return nl_process_query(msg, info, nl_process_query_port_control_state);
}

static int nl_port_beacon_enable_op(struct sk_buff *msg, struct genl_info *info)
{
	return nl_process_query(msg, info, nl_process_set_port_beacon_state);
}

static int nl_port_beacon_disable_op(struct sk_buff *msg, struct genl_info *info)
{
	return nl_process_query(msg, info, nl_process_set_port_beacon_state);
}

static int nl_port_beacon_state_query_op(struct sk_buff *msg, struct genl_info *info)
{
	return nl_process_query(msg, info, nl_process_query_port_control_state);
}

static int nl_port_routed_query_op(struct sk_buff *msg, struct genl_info *info)
{
	return nl_process_query(msg, info, nl_process_port_routed_query);
}

static int nl_rem_request_op(struct sk_buff *msg, struct genl_info *info)
{
	return nl_process_query(msg, info, nl_process_rem_request);
}

static int nl_routing_gen_query_op(struct sk_buff *msg, struct genl_info *info)
{
	return nl_process_query(msg, info, nl_process_routing_gen_query);
}

static int nl_fabric_device_properties_op(struct sk_buff *msg, struct genl_info *info)
{
	return nl_process_query(msg, info, nl_process_fabric_device_properties);
}

static int nl_fabric_sub_device_properties_get_op(struct sk_buff *msg, struct genl_info *info)
{
	return nl_process_query(msg, info, nl_process_sub_device_properties_get);
}

static int nl_fport_status_query_op(struct sk_buff *msg, struct genl_info *info)
{
	return nl_process_query(msg, info, nl_process_fport_status_query);
}

static int nl_sub_device_trap_count_query_op(struct sk_buff *msg, struct genl_info *info)
{
	return nl_process_query(msg, info, nl_process_sub_device_trap_count_query);
}

static int nl_fport_properties_op(struct sk_buff *msg, struct genl_info *info)
{
	return nl_process_query(msg, info, nl_process_fport_properties_query);
}

static int nl_fport_xmit_recv_counts_op(struct sk_buff *msg, struct genl_info *info)
{
	return nl_process_query(msg, info, nl_process_fport_xmit_recv_counts);
}

static int nl_fport_throughput_op(struct sk_buff *msg, struct genl_info *info)
{
	return nl_process_query(msg, info, nl_process_fport_throughput);
}

static const struct genl_ops nl_iaf_cmds[] = {
	{ .cmd = IAF_CMD_OP_DEVICE_ENUM,
	  .doit = nl_device_enum_op,
	  .policy = nl_iaf_policy_basic,
	},

	{ .cmd = IAF_CMD_OP_PORT_ENABLE,
	  .doit = nl_port_enable_op,
	  .policy = nl_iaf_policy_fabric_id_sd_index_port,
	  .flags = GENL_UNS_ADMIN_PERM,
	},

	{ .cmd = IAF_CMD_OP_PORT_DISABLE,
	  .doit = nl_port_disable_op,
	  .policy = nl_iaf_policy_fabric_id_sd_index_port,
	  .flags = GENL_UNS_ADMIN_PERM,
	},

	{ .cmd = IAF_CMD_OP_PORT_STATE_QUERY,
	  .doit = nl_port_state_query_op,
	  .policy = nl_iaf_policy_fabric_id_sd_index_port,
	},

	{ .cmd = IAF_CMD_OP_PORT_USAGE_ENABLE,
	  .doit = nl_port_usage_enable_op,
	  .policy = nl_iaf_policy_fabric_id_sd_index_port,
	  .flags = GENL_UNS_ADMIN_PERM,
	},

	{ .cmd = IAF_CMD_OP_PORT_USAGE_DISABLE,
	  .doit = nl_port_usage_disable_op,
	  .policy = nl_iaf_policy_fabric_id_sd_index_port,
	  .flags = GENL_UNS_ADMIN_PERM,
	},

	{ .cmd = IAF_CMD_OP_PORT_USAGE_STATE_QUERY,
	  .doit = nl_port_usage_state_query_op,
	  .policy = nl_iaf_policy_fabric_id_sd_index_port,
	},

	{ .cmd = IAF_CMD_OP_PORT_BEACON_ENABLE,
	  .doit = nl_port_beacon_enable_op,
	  .policy = nl_iaf_policy_fabric_id_sd_index_port,
	  .flags = GENL_UNS_ADMIN_PERM,
	},

	{ .cmd = IAF_CMD_OP_PORT_BEACON_DISABLE,
	  .doit = nl_port_beacon_disable_op,
	  .policy = nl_iaf_policy_fabric_id_sd_index_port,
	  .flags = GENL_UNS_ADMIN_PERM,
	},

	{ .cmd = IAF_CMD_OP_PORT_BEACON_STATE_QUERY,
	  .doit = nl_port_beacon_state_query_op,
	  .policy = nl_iaf_policy_fabric_id_sd_index_port,
	},

	{ .cmd = IAF_CMD_OP_PORT_ROUTED_QUERY,
	  .doit = nl_port_routed_query_op,
	  .policy = nl_iaf_policy_fabric_id_sd_index_port,
	},

	{ .cmd = IAF_CMD_OP_REM_REQUEST,
	  .doit = nl_rem_request_op,
	  .policy = nl_iaf_policy_basic,
	  .flags = GENL_UNS_ADMIN_PERM,
	},

	{ .cmd = IAF_CMD_OP_ROUTING_GEN_QUERY,
	  .doit = nl_routing_gen_query_op,
	  .policy = nl_iaf_policy_basic,
	},

	{ .cmd = IAF_CMD_OP_FABRIC_DEVICE_PROPERTIES,
	  .doit = nl_fabric_device_properties_op,
	  .policy = nl_iaf_policy_fabric_id,
	},

	{ .cmd = IAF_CMD_OP_SUB_DEVICE_PROPERTIES_GET,
	  .doit = nl_fabric_sub_device_properties_get_op,
	  .policy = nl_iaf_policy_fabric_id_sd_index,
	},

	{ .cmd = IAF_CMD_OP_FPORT_STATUS_QUERY,
	  .doit = nl_fport_status_query_op,
	  .policy = nl_iaf_policy_fabric_id_sd_index_port,
	},

	{ .cmd = IAF_CMD_OP_SUB_DEVICE_TRAP_COUNT_QUERY,
	  .doit = nl_sub_device_trap_count_query_op,
	  .policy = nl_iaf_policy_fabric_id_sd_index,
	},

	{ .cmd = IAF_CMD_OP_FPORT_PROPERTIES,
	  .doit = nl_fport_properties_op,
	  .policy = nl_iaf_policy_fabric_id_sd_index_port,
	},

	{ .cmd = IAF_CMD_OP_FPORT_XMIT_RECV_COUNTS,
	  .doit = nl_fport_xmit_recv_counts_op,
	  .policy = nl_iaf_policy_fabric_id_sd_index_port,
	},

	{ .cmd = IAF_CMD_OP_FPORT_THROUGHPUT,
	  .doit = nl_fport_throughput_op,
	  .policy = nl_iaf_policy_fabric_id_sd_index_nested_port,
	},
};

static struct genl_family nl_iaf_family = {
	.name = "iaf_ze",
	.version = INTERFACE_VERSION,
	.maxattr = IAF_ATTR_MAX,
	.ops = nl_iaf_cmds,
	.n_ops = ARRAY_SIZE(nl_iaf_cmds),
	.parallel_ops = true,
};

static bool nl_iaf_family_registered;

/**
 * nl_term - Unregisters the interface between kernel and user space.
 */
void nl_term(void)
{
	if (!nl_iaf_family_registered)
		return;

	if (genl_unregister_family(&nl_iaf_family))
		pr_err("Unable to unregister netlink family %s\n", nl_iaf_family.name);
	else
		nl_iaf_family_registered = false;
}

/**
 * nl_init - Registers the interface between kernel and user space.
 *
 * Return: genl_register_family result
 */
int nl_init(void)
{
	int err;

	/* IAF_CMD_OP_UNSPEC is not included so-1 */
	BUILD_BUG_ON(ARRAY_SIZE(nl_iaf_cmds) != _IAF_CMD_OP_COUNT - 1);

	err = genl_register_family(&nl_iaf_family);
	if (err)
		pr_err("Cannot register iaf family %s\n", nl_iaf_family.name);
	else
		nl_iaf_family_registered = true;

	return err;
}
