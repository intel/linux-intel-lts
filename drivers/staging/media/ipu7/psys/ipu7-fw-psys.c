// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2016 - 2024 Intel Corporation

#include <linux/delay.h>
#include <linux/dma-mapping.h>

#include <uapi/linux/ipu7-psys.h>

#include "abi/ipu7_fw_common_abi.h"
#include "abi/ipu7_fw_msg_abi.h"
#include "abi/ipu7_fw_psys_config_abi.h"

#include "ipu7-boot.h"
#include "ipu7-bus.h"
#include "ipu7-dma.h"
#include "ipu7-fw-psys.h"
#include "ipu7-syscom.h"
#include "ipu7-psys.h"

#define TLV_TYPE(type) ((u32)(type) & 0x3FU)
#define TLV_SIZE(buf_size) (((buf_size) / TLV_ITEM_ALIGNMENT) & 0xFFFFU)

/*
 * Node resource ID of INSYS, required when there is a link from INSYS to PSYS.
 */
#define IPU_PSYS_NODE_RSRC_ID_IS	(0xFEU)

/*
 * Special node resource ID to identify a generic external node.
 * Required when there is a link to/from IPU and that node.
 */
#define IPU_PSYS_NODE_RSRC_ID_EXT_IP	(0xFFU)

int ipu7_fw_psys_init(struct ipu7_psys *psys)
{
	struct ipu7_bus_device *adev = psys->adev;
	struct device *dev = &adev->auxdev.dev;
	struct ipu7_syscom_context *syscom;
	struct ipu7_psys_config *psys_config;
	struct syscom_queue_config *queue_configs;
	dma_addr_t psys_config_dma_addr;
	u32 freq;
	int i, num_queues, ret;

	/* Allocate and init syscom context. */
	syscom = devm_kzalloc(dev, sizeof(struct ipu7_syscom_context),
			      GFP_KERNEL);
	if (!syscom)
		return -ENOMEM;

	adev->syscom = syscom;
	syscom->num_input_queues = FWPS_MSG_ABI_MAX_INPUT_QUEUES;
	syscom->num_output_queues = FWPS_MSG_ABI_MAX_OUTPUT_QUEUES;
	num_queues = syscom->num_input_queues + syscom->num_output_queues;
	queue_configs = devm_kzalloc(dev, FW_QUEUE_CONFIG_SIZE(num_queues),
				     GFP_KERNEL);
	if (!queue_configs) {
		ipu7_fw_psys_release(psys);
		return -ENOMEM;
	}
	syscom->queue_configs = queue_configs;
	queue_configs[FWPS_MSG_ABI_OUT_ACK_QUEUE_ID].max_capacity =
		IPU_PSYS_ACK_QUEUE_SIZE;
	queue_configs[FWPS_MSG_ABI_OUT_ACK_QUEUE_ID].token_size_in_bytes =
		IPU_PSYS_OUT_MSG_SIZE;
	queue_configs[FWPS_MSG_ABI_OUT_LOG_QUEUE_ID].max_capacity =
		IPU_PSYS_LOG_QUEUE_SIZE;
	queue_configs[FWPS_MSG_ABI_OUT_LOG_QUEUE_ID].token_size_in_bytes =
		IPU_PSYS_OUT_MSG_SIZE;
	queue_configs[FWPS_MSG_ABI_IN_DEV_QUEUE_ID].max_capacity =
		IPU_PSYS_CMD_QUEUE_SIZE;
	queue_configs[FWPS_MSG_ABI_IN_DEV_QUEUE_ID].token_size_in_bytes =
		FWPS_MSG_HOST2FW_MAX_SIZE;
	queue_configs[FWPS_MSG_ABI_IN_RESERVED_QUEUE_ID].max_capacity = 0;
	queue_configs[FWPS_MSG_ABI_IN_RESERVED_QUEUE_ID].token_size_in_bytes =
		0;

	for (i = FWPS_MSG_ABI_IN_FIRST_TASK_QUEUE_ID; i < num_queues; i++) {
		queue_configs[i].max_capacity = IPU_PSYS_TASK_QUEUE_SIZE;
		queue_configs[i].token_size_in_bytes =
			sizeof(struct ia_gofo_msg_indirect);
	}

	/* Allocate PSYS subsys config. */
	psys_config = ipu7_dma_alloc(adev, sizeof(struct ipu7_psys_config),
				     &psys_config_dma_addr, GFP_KERNEL, 0);
	if (!psys_config) {
		dev_err(dev, "Failed to allocate psys subsys config.\n");
		ipu7_fw_psys_release(psys);
		return -ENOMEM;
	}
	psys->subsys_config = psys_config;
	psys->subsys_config_dma_addr = psys_config_dma_addr;
	memset(psys_config, 0, sizeof(struct ipu7_psys_config));
	ret = ipu_buttress_get_psys_freq(adev->isp, &freq);
	if (ret) {
		dev_err(dev, "Failed to get PSYS frequency.\n");
		ipu7_fw_psys_release(psys);
		return ret;
	}

	ret = ipu7_boot_init_boot_config(adev, queue_configs, num_queues,
					 freq, psys_config_dma_addr, 1U);
	if (ret)
		ipu7_fw_psys_release(psys);
	return ret;
}

void ipu7_fw_psys_release(struct ipu7_psys *psys)
{
	struct ipu7_bus_device *adev = psys->adev;

	ipu7_boot_release_boot_config(adev);
	if (psys->subsys_config) {
		ipu7_dma_free(adev, sizeof(struct ipu7_psys_config),
			      psys->subsys_config,
			      psys->subsys_config_dma_addr, 0);
		psys->subsys_config = NULL;
		psys->subsys_config_dma_addr = 0;
	}
}

static int ipu7_fw_dev_ready(struct ipu7_psys *psys, u16 type)
{
	const struct ia_gofo_msg_header_ack *ack_header;
	u8 buffer[FWPS_MSG_FW2HOST_MAX_SIZE];
	int ret;

	ret = ipu7_fw_psys_event_handle(psys, buffer);
	if (ret)
		return ret;

	ack_header = (const struct ia_gofo_msg_header_ack *)buffer;

	if (ack_header->header.tlv_header.tlv_type == type)
		return 0;

	return -EAGAIN;
}

static int ipu7_fw_dev_open(struct ipu7_psys *psys)
{
	struct ipu7_syscom_context *ctx = psys->adev->syscom;
	struct ipu7_msg_dev_open *token;

	dev_dbg(&psys->dev, "send_token: fw psys open\n");

	token = ipu7_syscom_get_token(ctx, FWPS_MSG_ABI_IN_DEV_QUEUE_ID);
	if (!token)
		return -ENODATA;

	token->header.tlv_header.tlv_type = TLV_TYPE(IPU_MSG_TYPE_DEV_OPEN);
	token->header.tlv_header.tlv_len32 = TLV_SIZE(sizeof(*token));
	token->header.user_token = 0;

	token->max_graphs = IPU_PSYS_MAX_GRAPH_NUMS;
	token->dev_msg_map = (u8)(IPU_MSG_DEVICE_OPEN_SEND_RESP |
				  IPU_MSG_DEVICE_OPEN_SEND_IRQ);
	token->enable_power_gating = 0;

	ipu7_syscom_put_token(ctx, FWPS_MSG_ABI_IN_DEV_QUEUE_ID);

	ipu_buttress_wakeup_ps_uc(psys->adev->isp);

	return 0;
}

int ipu7_fw_psys_open(struct ipu7_psys *psys)
{
	u32 retry = IPU_PSYS_OPEN_CLOSE_RETRY;
	int ret;

	ret = ipu7_fw_dev_open(psys);
	if (ret) {
		dev_err(&psys->dev, "failed to open PSYS dev.\n");
		return ret;
	}
	psys->dev_state = IPU_MSG_DEV_STATE_OPEN_WAIT;

	do {
		usleep_range(IPU_PSYS_OPEN_CLOSE_TIMEOUT_US,
			     IPU_PSYS_OPEN_CLOSE_TIMEOUT_US + 10);
		ret = ipu7_fw_dev_ready(psys, IPU_MSG_TYPE_DEV_OPEN_ACK);
		if (!ret) {
			dev_dbg(&psys->dev, "dev open done.\n");
			psys->dev_state = IPU_MSG_DEV_STATE_OPEN;
			return 0;
		}
	} while (retry--);

	if (!retry)
		dev_err(&psys->dev, "wait dev open timeout!\n");

	return ret;
}

static int ipu7_fw_dev_close(struct ipu7_psys *psys)
{
	struct ipu7_syscom_context *ctx = psys->adev->syscom;
	struct ipu7_msg_dev_close *token;

	dev_dbg(&psys->dev, "send_token: fw psys close\n");
	token = ipu7_syscom_get_token(ctx, FWPS_MSG_ABI_IN_DEV_QUEUE_ID);
	if (!token)
		return -ENODATA;

	token->header.tlv_header.tlv_type = TLV_TYPE(IPU_MSG_TYPE_DEV_CLOSE);
	token->header.tlv_header.tlv_len32 = TLV_SIZE(sizeof(*token));
	token->header.user_token = 0;

	token->dev_msg_map = (u8)(IPU_MSG_DEVICE_CLOSE_SEND_RESP |
				  IPU_MSG_DEVICE_CLOSE_SEND_IRQ);

	ipu7_syscom_put_token(ctx, FWPS_MSG_ABI_IN_DEV_QUEUE_ID);

	ipu_buttress_wakeup_ps_uc(psys->adev->isp);

	return 0;
}

void ipu7_fw_psys_close(struct ipu7_psys *psys)
{
	u32 retry = IPU_PSYS_OPEN_CLOSE_RETRY;
	int ret;

	ret = ipu7_fw_dev_close(psys);
	if (ret) {
		dev_err(&psys->dev, "failed to close PSYS dev.\n");
		return;
	}

	psys->dev_state = IPU_MSG_DEV_STATE_CLOSE_WAIT;

	do {
		usleep_range(IPU_PSYS_OPEN_CLOSE_TIMEOUT_US,
			     IPU_PSYS_OPEN_CLOSE_TIMEOUT_US + 10);
		ret = ipu7_fw_dev_ready(psys, IPU_MSG_TYPE_DEV_CLOSE_ACK);
		if (!ret) {
			dev_dbg(&psys->dev, "dev close done.\n");
			psys->dev_state = IPU_MSG_DEV_STATE_CLOSED;
			return;
		}
	} while (retry--);

	if (!retry)
		dev_err(&psys->dev, "wait dev close timeout!\n");
}

static void
ipu7_fw_psys_build_node_profile(const struct node_profile *profile,
				void **buf_ptr_ptr)
{
	struct ipu7_msg_cb_profile *cb_profile =
		(struct ipu7_msg_cb_profile *)*buf_ptr_ptr;
	u16 buf_size = sizeof(*cb_profile);

	memcpy(cb_profile->profile_base.teb, profile->teb,
	       sizeof(cb_profile->profile_base.teb));

	memcpy(cb_profile->rbm, profile->rbm, sizeof(cb_profile->rbm));
	memcpy(cb_profile->deb, profile->deb, sizeof(cb_profile->deb));
	memcpy(cb_profile->reb, profile->reb, sizeof(cb_profile->reb));

	cb_profile->profile_base.tlv_header.tlv_type =
		TLV_TYPE(IPU_MSG_NODE_PROFILE_TYPE_CB);
	cb_profile->profile_base.tlv_header.tlv_len32 = TLV_SIZE(buf_size);

	*buf_ptr_ptr += buf_size;
}

/* skip term, return false */
static bool ipu7_fw_psys_build_node_term(const struct node_ternimal *term,
					 void **buf_ptr_ptr)
{
	struct ipu7_msg_term *msg_term = (struct ipu7_msg_term *)*buf_ptr_ptr;
	u16 buf_size = sizeof(*msg_term);

	if (!term->term_id && !term->buf_size)
		return false;

	memset(msg_term, 0, sizeof(*msg_term));
	msg_term->term_id = term->term_id;
	/* Disable progress message on connect terminals */
	msg_term->event_req_bm = 0U;
	msg_term->payload_size = term->buf_size;

	msg_term->tlv_header.tlv_type = TLV_TYPE(IPU_MSG_TERM_TYPE_BASE);
	msg_term->tlv_header.tlv_len32 = TLV_SIZE(buf_size);

	*buf_ptr_ptr += buf_size;
	return true;
}

/* When skip processing node, just return false */
static bool ipu7_fw_psys_build_node(const struct graph_node *node,
				    void **buf_ptr_ptr)
{
	struct ipu7_msg_node *msg_node = (struct ipu7_msg_node *)*buf_ptr_ptr;
	u16 buf_size = sizeof(*msg_node);
	bool ret = false;
	u8 i = 0;
	u8 max_terms = 0;

	memset(msg_node, 0, sizeof(*msg_node));
	/**
	 * Pass node info to FW, do not check for external IP and ISYS
	 * As FW expects a external node
	 */
	if (node->node_rsrc_id != IPU_PSYS_NODE_RSRC_ID_IS &&
	    node->node_rsrc_id != IPU_PSYS_NODE_RSRC_ID_EXT_IP) {
		if (node->profiles[0].teb[0] == 0U)
			return false;
	}

	/**
	 * Sanity check for dummy node, TEB should set to required one
	 */
	if (node->node_rsrc_id == IPU_PSYS_NODE_RSRC_ID_IS ||
	    node->node_rsrc_id == IPU_PSYS_NODE_RSRC_ID_EXT_IP) {
		if (node->profiles[0].teb[0] != IPU_MSG_NODE_DONT_CARE_TEB_LO ||
		    node->profiles[0].teb[1] != IPU_MSG_NODE_DONT_CARE_TEB_HI)
			return false;
	}

	msg_node->node_rsrc_id = node->node_rsrc_id;
	msg_node->node_ctx_id = node->node_ctx_id;
	msg_node->num_frags = 1;

	*buf_ptr_ptr += buf_size;

	msg_node->profiles_list.head_offset =
		(u16)((uintptr_t)*buf_ptr_ptr
		      - (uintptr_t)&msg_node->profiles_list);
	for (i = 0; i < ARRAY_SIZE(node->profiles); i++) {
		ipu7_fw_psys_build_node_profile(&node->profiles[i],
						buf_ptr_ptr);
		msg_node->profiles_list.num_elems++;
	}

	msg_node->terms_list.head_offset =
		(u16)((uintptr_t)*buf_ptr_ptr -
		      (uintptr_t)&msg_node->terms_list);
	max_terms = ARRAY_SIZE(node->terminals);
	for (i = 0; i < max_terms && i < node->num_terms; i++) {
		ret = ipu7_fw_psys_build_node_term(&node->terminals[i],
						   buf_ptr_ptr);
		if (ret)
			msg_node->terms_list.num_elems++;
	}

	buf_size = (u32)(uintptr_t)*buf_ptr_ptr - (uintptr_t)msg_node;
	msg_node->tlv_header.tlv_type = TLV_TYPE(IPU_MSG_NODE_TYPE_BASE);
	msg_node->tlv_header.tlv_len32 = TLV_SIZE(buf_size);

	return true;
}

static bool ipu7_fw_psys_build_link(const struct graph_link *link,
				    void **buf_ptr_ptr)
{
	struct ipu7_msg_link *msg_link = (struct ipu7_msg_link *)*buf_ptr_ptr;

	if (!link->ep_src.node_ctx_id && !link->ep_dst.node_ctx_id &&
	    !link->ep_src.term_id && !link->ep_dst.term_id)
		return false;

	msg_link->endpoints.ep_src.node_ctx_id = link->ep_src.node_ctx_id;
	msg_link->endpoints.ep_src.term_id = link->ep_src.term_id;

	msg_link->endpoints.ep_dst.node_ctx_id = link->ep_dst.node_ctx_id;
	msg_link->endpoints.ep_dst.term_id = link->ep_dst.term_id;

	msg_link->foreign_key = link->foreign_key;
	msg_link->streaming_mode = link->streaming_mode;
	msg_link->pbk_id = link->pbk_id;
	msg_link->pbk_slot_id = link->pbk_slot_id;
	msg_link->delayed_link = link->delayed_link;

	*buf_ptr_ptr += sizeof(*msg_link);

	msg_link->link_options.num_elems = 0;
	msg_link->link_options.head_offset =
		(u16)((uintptr_t)*buf_ptr_ptr -
		      (uintptr_t)&msg_link->link_options);
	msg_link->tlv_header.tlv_type = TLV_TYPE(IPU_MSG_LINK_TYPE_GENERIC);
	msg_link->tlv_header.tlv_len32 = TLV_SIZE(sizeof(*msg_link));

	return true;
}

int ipu7_fw_psys_graph_open(const struct ipu_psys_graph_info *graph,
			    struct ipu7_psys *psys,
			    struct ipu7_psys_stream *ip)
{
	struct ipu7_syscom_context *ctx = psys->adev->syscom;
	void *buf_ptr;
	struct ipu7_msg_graph_open *graph_open;
	u32 buf_size = 0;
	bool ret = false;
	u8 i = 0;

	dev_dbg(&psys->dev, "send_token: fw psys graph open\n");
	buf_ptr = ipu7_syscom_get_token(ctx, FWPS_MSG_ABI_IN_DEV_QUEUE_ID);
	if (!buf_ptr)
		return -ENODATA;

	graph_open = (struct ipu7_msg_graph_open *)buf_ptr;

	memset(graph_open, 0, sizeof(*graph_open));
	graph_open->graph_id = ip->graph_id;
	graph_open->graph_msg_map = (u8)(IPU_MSG_GRAPH_OPEN_SEND_RESP
					 | IPU_MSG_GRAPH_OPEN_SEND_IRQ);

	buf_ptr += sizeof(*graph_open);
	graph_open->nodes.head_offset = (u16)((uintptr_t)buf_ptr
					      - (uintptr_t)&graph_open->nodes);
	for (i = 0; i < ARRAY_SIZE(ip->nodes); i++) {
		ret = ipu7_fw_psys_build_node(&ip->nodes[i], &buf_ptr);
		if (ret)
			graph_open->nodes.num_elems++;
	}

	graph_open->links.head_offset = (u16)((uintptr_t)buf_ptr
					      - (uintptr_t)&graph_open->links);
	for (i = 0; i < ARRAY_SIZE(graph->links); i++) {
		ret = ipu7_fw_psys_build_link(&graph->links[i], &buf_ptr);
		if (ret)
			graph_open->links.num_elems++;
	}

	buf_size = (u32)((uintptr_t)buf_ptr - (uintptr_t)graph_open);
	graph_open->header.tlv_header.tlv_type =
		TLV_TYPE(IPU_MSG_TYPE_GRAPH_OPEN);
	graph_open->header.tlv_header.tlv_len32 = TLV_SIZE(buf_size);
	graph_open->header.user_token = 0;

	ipu7_syscom_put_token(ctx, FWPS_MSG_ABI_IN_DEV_QUEUE_ID);

	ipu_buttress_wakeup_ps_uc(psys->adev->isp);

	return 0;
}

int ipu7_fw_psys_graph_close(u8 graph_id, struct ipu7_psys *psys)
{
	struct ipu7_syscom_context *ctx = psys->adev->syscom;
	struct ipu7_msg_graph_close *token;

	dev_dbg(&psys->dev, "send_token: fw psys graph close\n");
	token = ipu7_syscom_get_token(ctx, FWPS_MSG_ABI_IN_DEV_QUEUE_ID);
	if (!token)
		return -ENODATA;

	token->header.tlv_header.tlv_type = TLV_TYPE(IPU_MSG_TYPE_GRAPH_CLOSE);
	token->header.tlv_header.tlv_len32 = TLV_SIZE(sizeof(*token));
	token->header.user_token = 0;

	token->graph_id = graph_id;
	token->graph_msg_map = (u8)(IPU_MSG_DEVICE_CLOSE_SEND_RESP
				    | IPU_MSG_DEVICE_CLOSE_SEND_IRQ);

	ipu7_syscom_put_token(ctx, FWPS_MSG_ABI_IN_DEV_QUEUE_ID);

	ipu_buttress_wakeup_ps_uc(psys->adev->isp);

	return 0;
}

int ipu7_fw_psys_task_request(const struct ipu_psys_task_request *task,
			      struct ipu7_psys_stream *ip,
			      struct ipu_psys_task_queue *tq,
			      struct ipu7_psys *psys)
{
	struct ipu7_syscom_context *ctx = psys->adev->syscom;
	struct ipu7_msg_task *msg = tq->msg_task;
	struct ia_gofo_msg_indirect *ind;
	u8 node_ctx_id = task->node_ctx_id;
	u32 node_q_id;
	u32 teb_hi, teb_lo;
	u64 teb;
	u8 i, term_id;
	u8 num_terms;
	u8 max_terms;

	if (node_ctx_id >= ip->num_nodes ||
	    node_ctx_id >= ARRAY_SIZE(ip->nodes) ||
	    node_ctx_id >= ARRAY_SIZE(ip->q_id))
		return -EINVAL;

	node_q_id = ip->q_id[node_ctx_id];

	ind = ipu7_syscom_get_token(ctx, node_q_id);
	if (!ind)
		return -ENODATA;

	memset(msg, 0, sizeof(*msg));
	msg->graph_id = task->graph_id;
	msg->node_ctx_id = node_ctx_id;
	msg->profile_idx = 0U; /* Only one profile on HKR */
	msg->frame_id = task->frame_id;
	msg->frag_id = 0U; /* No frag, set to 0 */
	/*
	 * Each task has a flag indicating if ack needed, it may be used to
	 * reduce interrupts if multiple CBs supported.
	 */
	msg->req_done_msg = 1;
	msg->req_done_irq = 1;

	memcpy(msg->payload_reuse_bm, task->payload_reuse_bm,
	       sizeof(task->payload_reuse_bm));

	teb_hi = ip->nodes[node_ctx_id].profiles[0].teb[1];
	teb_lo = ip->nodes[node_ctx_id].profiles[0].teb[0];
	teb = (teb_lo | (((u64)teb_hi) << 32));

	num_terms = ip->nodes[node_ctx_id].num_terms;
	max_terms = min_t(u8, num_terms, (u8)ARRAY_SIZE(tq->task_buffers));
	for (i = 0U; i < max_terms; i++) {
		term_id = tq->task_buffers[i].term_id;
		if (term_id < ARRAY_SIZE(msg->term_buffers) &&
		    (teb & BIT_ULL(term_id)))
			msg->term_buffers[term_id] = tq->ipu7_addr[i];
	}

	msg->header.tlv_header.tlv_type = TLV_TYPE(IPU_MSG_TYPE_TASK_REQ);
	msg->header.tlv_header.tlv_len32 = TLV_SIZE(sizeof(*msg));
	msg->header.user_token = (uintptr_t)tq;

	ind->header.tlv_header.tlv_type = TLV_TYPE(IPU_MSG_TYPE_INDIRECT);
	ind->header.tlv_header.tlv_len32 = TLV_SIZE(sizeof(*ind));
	ind->header.msg_options.num_elems = 0;
	ind->header.msg_options.head_offset = 0;
	ind->ref_header = msg->header.tlv_header;
	ind->ref_msg_ptr = tq->task_dma_addr;

	ipu7_syscom_put_token(ctx, node_q_id);

	ipu_buttress_wakeup_ps_uc(psys->adev->isp);

	return 0;
}

int ipu7_fw_psys_event_handle(struct ipu7_psys *psys, u8 *buf_ptr)
{
	struct ipu7_syscom_context *ctx = psys->adev->syscom;
	void *token;

	token = ipu7_syscom_get_token(ctx, FWPS_MSG_ABI_OUT_ACK_QUEUE_ID);
	if (!token)
		return -ENODATA;

	memcpy(buf_ptr, token, sizeof(u8) * FWPS_MSG_FW2HOST_MAX_SIZE);

	ipu7_syscom_put_token(ctx, FWPS_MSG_ABI_OUT_ACK_QUEUE_ID);
	return 0;
}

int ipu7_fw_psys_get_log(struct ipu7_psys *psys)
{
	void *token;
	struct ia_gofo_msg_log *log_msg;
	u8 msg_type, msg_len;
	u32 count, fmt_id;
	struct device *dev = &psys->adev->auxdev.dev;
	struct psys_fw_log *fw_log = psys->fw_log;
	u32 log_size = sizeof(struct ia_gofo_msg_log_info_ts);

	token = ipu7_syscom_get_token(psys->adev->syscom,
				      FWPS_MSG_ABI_OUT_LOG_QUEUE_ID);
	if (!token)
		return -ENODATA;

	while (token) {
		log_msg = (struct ia_gofo_msg_log *)token;

		msg_type = log_msg->header.tlv_header.tlv_type;
		msg_len = log_msg->header.tlv_header.tlv_len32;
		if (msg_type != IPU_MSG_TYPE_DEV_LOG || !msg_len)
			dev_warn(dev, "Invalid msg data from Log queue!\n");

		count = log_msg->log_info_ts.log_info.log_counter;
		fmt_id = log_msg->log_info_ts.log_info.fmt_id;
		if (count > fw_log->count + 1)
			dev_warn(dev, "log msg lost, count %u+1 != %u!\n",
				 count, fw_log->count);

		if (fmt_id == IA_GOFO_MSG_LOG_FMT_ID_INVALID) {
			dev_err(dev, "invalid log msg fmt_id 0x%x!\n", fmt_id);
			ipu7_syscom_put_token(psys->adev->syscom,
					      FWPS_MSG_ABI_OUT_LOG_QUEUE_ID);
			return -EIO;
		}

		if (log_size + fw_log->head - fw_log->addr >
		    FW_LOG_BUF_SIZE)
			fw_log->head = fw_log->addr;

		memcpy(fw_log->head, (void *)&log_msg->log_info_ts,
		       sizeof(struct ia_gofo_msg_log_info_ts));

		fw_log->count = count;
		fw_log->head += log_size;
		fw_log->size += log_size;

		ipu7_syscom_put_token(psys->adev->syscom,
				      FWPS_MSG_ABI_OUT_LOG_QUEUE_ID);

		token = ipu7_syscom_get_token(psys->adev->syscom,
					      FWPS_MSG_ABI_OUT_LOG_QUEUE_ID);
	};

	return 0;
}

