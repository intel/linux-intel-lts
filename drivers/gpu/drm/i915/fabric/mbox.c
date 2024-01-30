// SPDX-License-Identifier: MIT
/*
 * Copyright(c) 2020 - 2023 Intel Corporation.
 *
 */

#include <linux/types.h>
#include <linux/netlink.h>
#include <net/genetlink.h>
#include <linux/skbuff.h>
#include <net/sock.h>

#include "iaf_drv.h"
#include "io.h"
#include "mbdb.h"
#include "ops.h"
#include "mbox.h"

#define INTERFACE_VERSION 1

enum mbox_cmd {
	CMD_MB_UNSPEC,
	CMD_MB,
	_CMD_MAX,
};

enum mbox_attr {
	ATTR_UNSPEC,

	/* MB operation */
	ATTR_MB,
	_ATTR_MAX,
};

#define ATTR_MAX (_ATTR_MAX - 1)
#define CMD_MAX (_CMD_MAX - 1)

enum mbox_ops {
	OP_MBOX_SEND,
	OP_MBOX_RECV,
	OP_CSR_READ_SEND,
	OP_CSR_READ_RECV,
	OP_CSR_WRITE_SEND,
	OP_CSR_WRITE_RECV,
};

struct mbox_send_msg {
	/* Request Info */
	u32 msg_len;
	u32 reserved;
	struct mbox_msg msg;
} __packed;

struct mbox_recv_msg {
	/* Response Info */
	struct mbox_msg msg;
};

struct csr_read_send_msg {
	/* Request Info */
	u32 bar_offset;
};

struct csr_read_recv_msg {
	/* Response Info */
	u32 bar_offset;
	u32 reserved;
	u64 csr_data;
} __packed;

struct csr_write_send_msg {
	/* Request Info */
	u32 bar_offset;
	u32 reserved;
	u64 csr_data;
} __packed;

struct csr_write_recv_msg {
	/* Response Info */
	u32 bar_offset;
};

struct sd_id {
	u32 fabric_id;
	u8 sd_index;
	u8 port;
	u16 reserved;
} __packed;

struct mbox_op_msg {
	u64 context;
	u32 op;
	struct sd_id sd_hdl;
	u32 reserved1;
	union {
		struct mbox_send_msg mbox_send;
		struct mbox_recv_msg mbox_recv;
		struct csr_read_send_msg csr_read_send;
		struct csr_read_recv_msg csr_read_recv;
		struct csr_write_send_msg csr_write_send;
		struct csr_write_recv_msg csr_write_recv;
	};
	/* Response Info */
	s32 result;
	u32 reserved2;
} __packed;

static struct genl_family mbox_iaf_family;

static DEFINE_MUTEX(mbnl_lock);

static struct mbox_op_msg *mbox_alloc_rsp(struct mbox_op_msg *req,
					  enum mbox_ops op)
{
	struct mbox_op_msg *rsp;

	rsp = kmalloc(sizeof(*rsp), GFP_KERNEL);
	if (!rsp)
		return ERR_PTR(-ENOMEM);

	rsp->context = req->context;
	rsp->sd_hdl = req->sd_hdl;
	rsp->op = op;
	rsp->result = 0;

	return rsp;
}

static int mbox_send_reply(struct genl_info *info, struct mbox_op_msg *op_msg)
{
	struct sk_buff *msg;
	void *usrhdr;
	int ret;

	msg = genlmsg_new(NLMSG_GOODSIZE, GFP_NOWAIT);
	if (!msg)
		return -ENOMEM;

	usrhdr = genlmsg_put_reply(msg, info, &mbox_iaf_family, 0,
				   info->genlhdr->cmd);
	if (!usrhdr) {
		nlmsg_free(msg);
		return -ENOMEM;
	}

	ret = nla_put(msg, ATTR_MB, sizeof(*op_msg), op_msg);
	if (ret) {
		nlmsg_free(msg);
		return ret;
	}

	genlmsg_end(msg, usrhdr);

	ret = genlmsg_reply(msg, info);
	if (ret) {
		pr_err("Unable to send response msg.\n");
		nlmsg_free(msg);
	}

	return ret;
}

static int mbox_ack_op_req(struct mbox_op_msg *req, struct genl_info *info)
{
	return mbox_send_reply(info, req);
}

static int mbox_nak_op_req(struct mbox_op_msg *req, struct genl_info *info,
			   int result)
{
	req->result = result;
	return mbox_ack_op_req(req, info);
}

static u64 mbox_build_rsp_cw(u64 cw, u64 req_cw, int rsp_result)
{
	u64 rsp_cw;

	rsp_cw = build_cw(mbdb_mbox_op_code(cw), mbdb_mbox_msg_type(cw),
			  mbdb_mbox_is_posted(cw), mbdb_mbox_seq_no(cw),
			  mbdb_mbox_params_len(cw), mbdb_mbox_tid(req_cw));

	if (rsp_result) {
		rsp_result = rsp_result < 0 ? MBOX_RSP_STATUS_LOGICAL_ERROR
					    : rsp_result;
		rsp_cw |= FIELD_PREP(MBOX_CW_RSP_STATUS, rsp_result);
	}

	return rsp_cw;
}

static struct mbox_op_msg *mbox_execute_op(struct fsubdev *sd,
					   struct mbox_op_msg *req,
					   u64 *cw,
					   int *op_result)
{
	u64 req_cw = req->mbox_send.msg.cw;
	bool posted = mbdb_mbox_is_posted(req_cw) == MBOX_NO_RESPONSE_REQUESTED;
	struct mbox_op_msg *rsp = NULL;
	void *recv_data = NULL;
	u32 recv_len = 0;
	struct mbdb_ibox *ibox;
	struct mbdb_op_param op_params;
	int ret;

	if (!posted) {
		rsp = mbox_alloc_rsp(req, OP_MBOX_RECV);
		if (IS_ERR(rsp))
			return rsp;

		recv_data = rsp->mbox_recv.msg.param_area;
		recv_len = sizeof(rsp->mbox_recv.msg.param_area);
	}

	ibox = mbdb_op_build_cw_and_acquire_ibox_set_hdlr(sd, mbdb_mbox_op_code(req_cw),
							  mbdb_mbox_params_len(req_cw), recv_data,
							  recv_len, cw, POSTED(posted),
							  ops_rsp_handler_relaxed);
	if (IS_ERR(ibox)) {
		ret = PTR_ERR(ibox);
		kfree(rsp);
		return ERR_PTR(ret);
	}

	if (req->mbox_send.msg_len < sizeof(*cw)) {
		mbdb_ibox_release_pre_wait(ibox);
		kfree(rsp);
		return ERR_PTR(-EINVAL);
	}

	op_params.len = req->mbox_send.msg_len - sizeof(*cw);

	if (op_params.len) {
		op_params.data = req->mbox_send.msg.param_area;
		ret = ops_execute(sd, cw, 1, &op_params, ibox);
	} else {
		ret = ops_execute(sd, cw, 0, NULL, ibox);
	}

	if (ret < 0) {
		/* ops_execute failed to communicate with the mailbox */
		kfree(rsp);
		return ERR_PTR(ret);
	}

	/* Set the response result returned by the mailbox operation */
	*op_result = ret;

	return rsp;
}

static int mbox_send_op(struct fsubdev *sd, struct mbox_op_msg *req,
			struct genl_info *info)
{
	struct mbox_op_msg *rsp;
	u64 cw;
	int op_result = MBOX_RSP_STATUS_OK;
	int ret;

	req->result = 0;

	rsp = mbox_execute_op(sd, req, &cw, &op_result);
	if (IS_ERR(rsp))
		return mbox_nak_op_req(req, info, PTR_ERR(rsp));

	ret = mbox_ack_op_req(req, info);
	if (!ret && rsp) {
		rsp->result = op_result;
		rsp->mbox_recv.msg.cw =
			mbox_build_rsp_cw(cw, req->mbox_send.msg.cw, op_result);

		ret = mbox_send_reply(info, rsp);
	}

	kfree(rsp);

	return ret;
}

static int mbox_csr_read_op(struct fsubdev *sd, struct mbox_op_msg *req,
			    struct genl_info *info)
{
	struct mbox_op_msg *rsp;
	u64 __iomem *addr;
	int ret;

	req->result = 0;

	rsp = mbox_alloc_rsp(req, OP_CSR_READ_RECV);
	if (IS_ERR(rsp))
		return mbox_nak_op_req(req, info, PTR_ERR(rsp));

	addr = (u64 __iomem *)(sd->csr_base + req->csr_read_send.bar_offset);

	rsp->csr_read_recv.bar_offset = req->csr_read_send.bar_offset;
	rsp->csr_read_recv.csr_data = readq(addr);

	ret = mbox_ack_op_req(req, info);
	if (!ret)
		ret = mbox_send_reply(info, rsp);

	kfree(rsp);

	return ret;
}

static int mbox_csr_write_op(struct fsubdev *sd, struct mbox_op_msg *req,
			     struct genl_info *info)
{
	struct mbox_op_msg *rsp;
	u64 __iomem *addr;
	int ret;

	req->result = 0;

	rsp = mbox_alloc_rsp(req, OP_CSR_WRITE_RECV);
	if (IS_ERR(rsp))
		return mbox_nak_op_req(req, info, PTR_ERR(rsp));

	addr = (u64 __iomem *)(sd->csr_base + req->csr_write_send.bar_offset);
	writeq(req->csr_write_send.csr_data, addr);

	rsp->csr_write_recv.bar_offset = req->csr_write_send.bar_offset;

	ret = mbox_ack_op_req(req, info);
	if (!ret)
		ret = mbox_send_reply(info, rsp);

	kfree(rsp);

	return ret;
}

static int mbox_process_op(struct sk_buff *msg, struct genl_info *info)
{
	struct nlattr *attr = info->attrs[ATTR_MB];
	struct mbox_op_msg *req;
	struct fsubdev *sd;
	int ret;

	mutex_lock(&mbnl_lock);

	if (info->genlhdr->version != INTERFACE_VERSION) {
		ret = -EINVAL;
		goto unlock_mbnl;
	}

	if (!attr) {
		ret = -ENODATA;
		goto unlock_mbnl;
	}

	req = nla_data(attr);

	sd = find_sd_id(req->sd_hdl.fabric_id, req->sd_hdl.sd_index);
	if (IS_ERR(sd)) {
		ret = mbox_nak_op_req(req, info, PTR_ERR(sd));
		goto unlock_mbnl;
	}

	switch (req->op) {
	case OP_MBOX_SEND:
		ret = mbox_send_op(sd, req, info);
		break;

	case OP_CSR_READ_SEND:
		ret = mbox_csr_read_op(sd, req, info);
		break;

	case OP_CSR_WRITE_SEND:
		ret = mbox_csr_write_op(sd, req, info);
		break;

	case OP_MBOX_RECV:
	case OP_CSR_READ_RECV:
	case OP_CSR_WRITE_RECV:
	default:
		ret = mbox_nak_op_req(req, info, -EINVAL);
		break;
	}

	fdev_put(sd->fdev);

unlock_mbnl:
	mutex_unlock(&mbnl_lock);
	return ret;
}

static struct nla_policy mbox_iaf_policy[ATTR_MAX + 1] = {
	[ATTR_MB] = NLA_POLICY_EXACT_LEN(sizeof(struct mbox_op_msg)),
};

static struct genl_ops mbox_iaf_cmds[CMD_MAX] = {
	{
		.cmd = CMD_MB,
		.doit = mbox_process_op,
		.flags = GENL_UNS_ADMIN_PERM,

	},
};

/* Change name, need flag day with L8sim */
static struct genl_family mbox_iaf_family = {
	.name = "intel_cd",
	.version = INTERFACE_VERSION,
	.maxattr = ATTR_MAX,
	.ops = mbox_iaf_cmds,
	.n_ops = ARRAY_SIZE(mbox_iaf_cmds),
	.policy = mbox_iaf_policy,
	.parallel_ops = true,
};

static bool mbox_iaf_family_registered;

/**
 * mbox_term_module - Unregisters the interface between kernel and user space.
 */
void mbox_term_module(void)
{
	if (!mbox_iaf_family_registered)
		return;

	if (genl_unregister_family(&mbox_iaf_family))
		pr_err("Unable to unregister netlink family %s\n", mbox_iaf_family.name);
	else
		mbox_iaf_family_registered = false;
}

/**
 * mbox_init_module - Registers the interface between kernel and user space.
 */
void mbox_init_module(void)
{
	if (genl_register_family(&mbox_iaf_family)) {
		pr_err("Cannot register mbox_iaf family %s\n", mbox_iaf_family.name);
		pr_info("Green Link interface is not available\n");
	} else {
		mbox_iaf_family_registered = true;
	}
}
