// SPDX-License-Identifier: MIT
/*
 * Copyright(c) 2019 - 2023 Intel Corporation.
 *
 */

#include <linux/delay.h>

#include "csr.h"
#include "fw.h"
#include "iaf_drv.h"
#include "io.h"
#include "mbdb.h"
#include "ops.h"
#include "trace.h"

#define MAX_OP_RETRIES 16

enum lock_state {
	LOCK,
	NOLOCK,
};

void ops_rsp_handler_relaxed(struct mbdb_ibox *ibox)
{
	u16 rsp_len = mbdb_mbox_params_len(ibox->cw);

	if (unlikely(rsp_len != ibox->rsp_len))
		/* Adjust response returned by firmware */
		rsp_len = min(rsp_len, ibox->rsp_len);

	io_readq_aligned(mbdb_ibox_gp_inbox_param_addr(ibox), ibox->response, rsp_len);
}

void ops_rsp_handler_default(struct mbdb_ibox *ibox)
{
	if (unlikely(mbdb_mbox_params_len(ibox->cw) != ibox->rsp_len)) {
		ibox->rsp_status = -EINVAL;
		sd_err(mbdb_ibox_sd(ibox), "mbdb inbox rsp_len mismatch: cw 0x%016llx actual %u expected %u\n",
		       ibox->cw, mbdb_mbox_params_len(ibox->cw), ibox->rsp_len);
	} else {
		ops_rsp_handler_relaxed(ibox);
	}
}

/**
 * ops_element_copy - Copy array element data from src to dst even when they are of different sizes
 * @src: src to be copied from
 * @dst: dst to be copied into
 * @src_sz: size of src element
 * @dst_sz: size of dst element
 * @elements: array element count
 *
 * Note: It is the responsibility of the caller to recognize the possibility that if dst > src
 *       extra space at the tail of dst will not be altered
 */
void ops_element_copy(void __iomem *src, void *dst, size_t src_sz, size_t dst_sz, size_t elements)
{
	u8 __iomem *src_ptr;
	u8 *dst_ptr;
	size_t copy_sz;

	for (src_ptr = src, dst_ptr = dst, copy_sz = min(src_sz, dst_sz); elements;
	     elements--, src_ptr += src_sz, dst_ptr += dst_sz)
		io_readq_unaligned(src_ptr, dst_ptr, copy_sz);
}

struct mbdb_ibox *
mbdb_op_build_cw_and_acquire_ibox_set_hdlr(struct fsubdev *sd, const u8 op_code, size_t parm_len,
					   void *response, u32 rsp_len, u64 *cw,
					   unsigned long flags, op_response_handler op_rsp_handler)
{
	bool posted = FIELD_GET(OP_CODE_POSTED, flags);
	struct mbdb_ibox *ibox;

	if (FIELD_GET(OP_CODE_CHECK_IS_VALID, flags))
		if (!is_opcode_valid(sd, op_code))
			return ERR_PTR(-EPERM);

	ibox = mbdb_ibox_acquire(sd, op_code, response, rsp_len, posted, op_rsp_handler);
	if (IS_ERR(ibox)) {
		sd_err(sd, "Unable to acquire ibox\n");
		return ibox;
	}

	*cw = build_cw(op_code, MBOX_REQUEST,
		       posted ? MBOX_NO_RESPONSE_REQUESTED
			      : MBOX_RESPONSE_REQUESTED,
		       0, parm_len, mbdb_ibox_tid(ibox));

	return ibox;
}

struct mbdb_ibox *
mbdb_op_build_cw_and_acquire_ibox(struct fsubdev *sd, const u8 op_code, size_t parm_len,
				  void *response, u32 rsp_len, u64 *cw, unsigned long flags)
{
	return mbdb_op_build_cw_and_acquire_ibox_set_hdlr(sd, op_code, parm_len, response, rsp_len,
							  cw, flags, ops_rsp_handler_default);
}

int ops_tx_dcc_interp_override_enable_set(struct fsubdev *sd, u32 port, u32 lane, u32 enable,
					  bool posted)
{
	const u8 op_code = MBOX_OP_CODE_SERDES_TX_DCC_MARGIN;
	struct mbdb_ibox *ibox;
	struct dcc_interp_override_enable_set {
		u32 sub_op;
		u32 port;
		u32 lane;
		u32 enable;
	} __packed req = {};
	struct tx_dcc_margin_error_rsp rsp;
	struct mbdb_op_param op_param;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, sizeof(req), posted ? NULL : &rsp,
						 sizeof(rsp), &cw, POSTED_CHECK_OP(posted));
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	req.sub_op = SERDES_MARGIN_SUB_OP_CODE_TX_DCC_INTERP_OV_EN_SET;
	req.port = port;
	req.lane = lane;
	req.enable = enable;

	op_param.len = sizeof(req);
	op_param.data = &req;

	return ops_execute(sd, &cw, 1, &op_param, ibox);
}

int ops_tx_dcc_interp_override_enable_get(struct fsubdev *sd, u32 port, u32 lane,
					  struct tx_dcc_interp_override_enable_get_rsp *rsp)
{
	const u8 op_code = MBOX_OP_CODE_SERDES_TX_DCC_MARGIN;
	struct mbdb_ibox *ibox;
	struct dcc_interp_override_enable_get {
		u32 sub_op;
		u32 port;
		u32 lane;
	} __packed req = {};
	struct mbdb_op_param op_param;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, sizeof(req), rsp, sizeof(*rsp), &cw,
						 NON_POSTED_CHECK_OP);
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	req.sub_op = SERDES_MARGIN_SUB_OP_CODE_TX_DCC_INTERP_OV_EN_GET;
	req.port = port;
	req.lane = lane;

	op_param.len = sizeof(req);
	op_param.data = &req;

	return ops_execute(sd, &cw, 1, &op_param, ibox);
}

int ops_tx_dcc_interp_override_set(struct fsubdev *sd, u32 port, u32 lane, u32 value, bool posted)
{
	const u8 op_code = MBOX_OP_CODE_SERDES_TX_DCC_MARGIN;
	struct mbdb_ibox *ibox;
	struct dcc_interp_override_set {
		u32 sub_op;
		u32 port;
		u32 lane;
		u32 value;
	} __packed req = {};
	struct tx_dcc_margin_error_rsp rsp;
	struct mbdb_op_param op_param;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, sizeof(req), posted ? NULL : &rsp,
						 posted ? 0 : sizeof(rsp), &cw,
						 POSTED_CHECK_OP(posted));
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	req.sub_op = SERDES_MARGIN_SUB_OP_CODE_TX_DCC_INTERP_OVVL_SET;
	req.port = port;
	req.lane = lane;
	req.value = value;

	op_param.len = sizeof(req);
	op_param.data = &req;

	return ops_execute(sd, &cw, 1, &op_param, ibox);
}

int ops_tx_dcc_interp_override_get(struct fsubdev *sd, u32 port, u32 lane,
				   struct tx_dcc_interp_override_get_rsp *rsp)
{
	const u8 op_code = MBOX_OP_CODE_SERDES_TX_DCC_MARGIN;
	struct mbdb_ibox *ibox;
	struct dcc_interp_override_get {
		u32 sub_op;
		u32 port;
		u32 lane;
	} __packed req = {};
	struct mbdb_op_param op_param;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, sizeof(req), rsp, sizeof(*rsp), &cw,
						 NON_POSTED_CHECK_OP);
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	req.sub_op = SERDES_MARGIN_SUB_OP_CODE_TX_DCC_INTERP_OVVL_GET;
	req.port = port;
	req.lane = lane;

	op_param.len = sizeof(req);
	op_param.data = &req;

	return ops_execute(sd, &cw, 1, &op_param, ibox);
}

int ops_maint_mode_set(struct fsubdev *sd, u32 port, u32 mode, bool posted)
{
	const u8 op_code = MBOX_OP_CODE_SERDES_TX_DCC_MARGIN;
	struct mbdb_ibox *ibox;
	struct maint_mode_set_req {
		u32 sub_op;
		u32 port;
		u32 mode;
	} __packed req = {};
	struct tx_dcc_margin_error_rsp rsp;
	struct mbdb_op_param op_param;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, sizeof(req), posted ? NULL : &rsp,
						 posted ? 0 : sizeof(rsp), &cw,
						 POSTED_CHECK_OP(posted));
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	req.sub_op = SERDES_MARGIN_SUB_OP_CODE_MAINT_MODE_SET;
	req.port = port;
	req.mode = mode;

	op_param.len = sizeof(req);
	op_param.data = &req;

	return ops_execute(sd, &cw, 1, &op_param, ibox);
}

int ops_maint_mode_get(struct fsubdev *sd, u32 port, struct maint_mode_get_rsp *rsp)
{
	const u8 op_code = MBOX_OP_CODE_SERDES_TX_DCC_MARGIN;
	struct mbdb_ibox *ibox;
	struct maint_mode_get_req {
		u32 sub_op;
		u32 port;
	} __packed req = {};
	struct mbdb_op_param op_param;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, sizeof(req), rsp, sizeof(*rsp), &cw,
						 NON_POSTED_CHECK_OP);
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	req.sub_op = SERDES_MARGIN_SUB_OP_CODE_MAINT_MODE_GET;
	req.port = port;

	op_param.len = sizeof(req);
	op_param.data = &req;

	return ops_execute(sd, &cw, 1, &op_param, ibox);
}

int ops_tx_dcc_index_set(struct fsubdev *sd, u32 port, u32 lane, u32 index, bool posted)
{
	const u8 op_code = MBOX_OP_CODE_SERDES_TX_DCC_MARGIN;
	struct mbdb_ibox *ibox;
	struct serdes_margin_param_set_req {
		u32 sub_op;
		u32 port;
		u32 lane;
		u32 index;
	} __packed req = {};
	struct tx_dcc_margin_error_rsp rsp;
	struct mbdb_op_param op_param;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, sizeof(req), posted ? NULL : &rsp,
						 posted ? 0 : sizeof(rsp), &cw,
						 POSTED_CHECK_OP(posted));
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	req.sub_op = SERDES_MARGIN_SUB_OP_CODE_TX_DCC_INDEX_SET;
	req.port = port;
	req.lane = lane;
	req.index = index;

	op_param.len = sizeof(req);
	op_param.data = &req;

	return ops_execute(sd, &cw, 1, &op_param, ibox);
}

int ops_tx_dcc_interp_get(struct fsubdev *sd, u32 port, u32 lane, struct tx_dcc_interp_get_rsp *rsp)
{
	const u8 op_code = MBOX_OP_CODE_SERDES_TX_DCC_MARGIN;
	struct mbdb_ibox *ibox;
	struct dcc_interp_get_req {
		u32 sub_op;
		u32 port;
		u32 lane;
	} __packed req = {};
	struct mbdb_op_param op_param;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, sizeof(req), rsp, sizeof(*rsp), &cw,
						 NON_POSTED_CHECK_OP);
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	req.sub_op = SERDES_MARGIN_SUB_OP_CODE_TX_DCC_INTERP_GET;
	req.port = port;
	req.lane = lane;

	op_param.len = sizeof(req);
	op_param.data = &req;

	return ops_execute(sd, &cw, 1, &op_param, ibox);
}

int ops_tx_dcc_margin_param_set(struct fsubdev *sd, u32 port, u16 value, bool posted)
{
	const u8 op_code = MBOX_OP_CODE_SERDES_TX_DCC_MARGIN;
	struct mbdb_ibox *ibox;
	struct serdes_margin_param_set_req {
		u32 sub_op;
		u32 port;
		u16 value;
	} __packed req = {};
	struct tx_dcc_margin_error_rsp rsp;
	struct mbdb_op_param op_param;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, sizeof(req), posted ? NULL : &rsp,
						 posted ? 0 : sizeof(rsp), &cw,
						 POSTED_CHECK_OP(posted));
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	req.sub_op = SERDES_MARGIN_SUB_OP_CODE_TX_DCC_MARGIN_PARAM_SET;
	req.port = port;
	req.value = value;

	op_param.len = sizeof(req);
	op_param.data = &req;

	return ops_execute(sd, &cw, 1, &op_param, ibox);
}

int ops_tx_dcc_margin_param_get(struct fsubdev *sd, u32 port,
				struct tx_dcc_margin_param_get_rsp *rsp)
{
	const u8 op_code = MBOX_OP_CODE_SERDES_TX_DCC_MARGIN;
	struct mbdb_ibox *ibox;
	struct serdes_margin_param_get_req {
		u32 sub_op;
		u32 port;
	} __packed req = {};
	struct mbdb_op_param op_param;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, sizeof(req), rsp, sizeof(*rsp), &cw,
						 NON_POSTED_CHECK_OP);
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	req.sub_op = SERDES_MARGIN_SUB_OP_CODE_TX_DCC_MARGIN_PARAM_GET;
	req.port = port;

	op_param.len = sizeof(req);
	op_param.data = &req;

	return ops_execute(sd, &cw, 1, &op_param, ibox);
}

int ops_serdes_channel_estimate_get(struct fsubdev *sd, u32 port, u32 lane,
				    struct mbdb_serdes_ch_est_rsp *rsp)
{
	const u8 op_code = MBOX_OP_CODE_SERDES_CHEST_GET;
	struct mbdb_ibox *ibox;
	struct mbdb_serdes_ch_est_req {
		u32 port;
		u32 lane;
		u32 ilv;
		u32 range_min;
		u32 range_max;
	} __packed req = {};
	struct mbdb_op_param op_param;
	u64 cw;
	int ret;

	ibox = mbdb_op_build_cw_and_acquire_ibox_set_hdlr(sd, op_code, sizeof(req), rsp->data,
							  sizeof(rsp->data), &cw,
							  NON_POSTED_CHECK_OP,
							  ops_rsp_handler_relaxed);
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	req.port = port;
	req.lane = lane;

	op_param.len = sizeof(req);
	op_param.data = &req;

	ret = ops_execute(sd, &cw, 1, &op_param, ibox);
	if (!ret)
		rsp->elements = mbdb_mbox_params_len(cw) / sizeof(rsp->data[0]);

	return ret;
}

int ops_port_var_table_write(struct fsubdev *sd, u32 port, u32 link_speed,
			     struct port_var_data *data, bool posted)
{
	const u8 op_code = MBOX_OP_CODE_VARIABLE_TABLE_WRITE;
	struct mbdb_ibox *ibox;
	struct mbdb_var_table_write_req {
		u32 port;
		u32 link_speed;
		struct port_var_data data;
	} __packed req;
	struct mbdb_op_param op_param;
	u64 cw;

	/* must write a valid port/speed setting, can write multiple speeds */
	if (!port || !(link_speed & (LINK_SPEED_SLOW | LINK_SPEED_FAST)))
		return -EINVAL;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, sizeof(req), NULL, 0, &cw,
						 POSTED_CHECK_OP(posted));
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	req.port = port;
	req.link_speed = link_speed;
	memcpy(&req.data, data, sizeof(*data));

	op_param.len = sizeof(req);
	op_param.data = &req;

	return ops_execute(sd, &cw, 1, &op_param, ibox);
}

int ops_port_var_table_read(struct fsubdev *sd, u32 port, u32 link_speed, struct port_var_data *rsp)
{
	const u8 op_code = MBOX_OP_CODE_VARIABLE_TABLE_READ;
	struct mbdb_ibox *ibox;
	struct mbdb_var_table_read_req {
		u32 port;
		u32 link_speed;
	} __packed req;
	struct mbdb_op_param op_param;
	u64 cw;

	/* must read a single valid port/speed setting */
	if (!port || (bool)(link_speed & LINK_SPEED_SLOW) == (bool)(link_speed & LINK_SPEED_FAST))
		return -EINVAL;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, sizeof(req), rsp, sizeof(*rsp), &cw,
						 NON_POSTED_CHECK_OP);
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	req.port = port;
	req.link_speed = link_speed;

	op_param.len = sizeof(req);
	op_param.data = &req;

	return ops_execute(sd, &cw, 1, &op_param, ibox);
}

static void ops_serdes_eqinfo_rsp_handler(struct mbdb_ibox *ibox)
{
	u16 rsp_len = mbdb_mbox_params_len(ibox->cw);
	struct mbdb_serdes_eq_info_get_rsp *rsp = ibox->response;
	u64 __iomem *gp_inbox_param_addr = mbdb_ibox_gp_inbox_param_addr(ibox);

	/* B0 response received */
	if (rsp_len == sizeof(struct mbdb_serdes_eq_info_get_rsp)) {
		ops_element_copy(gp_inbox_param_addr, rsp, rsp_len / LANES,
				 sizeof(*rsp) / LANES, LANES);
	} else {
		ibox->rsp_status = -EINVAL;
		sd_err(mbdb_ibox_sd(ibox), "mbdb inbox rsp_len mismatch: cw 0x%016llx actual %u expected %u\n",
		       ibox->cw, rsp_len, ibox->rsp_len);
	}
}

int ops_linkmgr_trace_mask_set(struct fsubdev *sd, u64 mask)
{
	const u8 op_code = MBOX_OP_CODE_LINK_MGR_TRACE_MASK_SET;
	struct mbdb_ibox *ibox;
	struct mbdb_linkmgr_trace_mask_set_req {
		u64 mask;
	} __packed req;
	struct mbdb_op_param op_param;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, sizeof(req), NULL, 0, &cw,
						 NON_POSTED_CHECK_OP);
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	req.mask = mask;

	op_param.len = sizeof(req);
	op_param.data = &req;

	return ops_execute(sd, &cw, 1, &op_param, ibox);
}

int ops_linkmgr_trace_mask_get(struct fsubdev *sd, u64 *mask)
{
	const u8 op_code = MBOX_OP_CODE_LINK_MGR_TRACE_MASK_GET;
	struct mbdb_ibox *ibox;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, 0, mask, sizeof(*mask), &cw,
						 NON_POSTED_CHECK_OP);
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	return ops_execute(sd, &cw, 0, NULL, ibox);
}

int ops_linkmgr_trace_dump(struct fsubdev *sd, u32 cnt, bool first,
			   struct mbdb_op_linkmgr_trace_dump_rsp *rsp)
{
	const u8 op_code = MBOX_OP_CODE_LINK_MGR_TRACE_DUMP;
	struct mbdb_ibox *ibox;
	struct mbdb_linkmgr_trace_dump_req {
		u32 cnt;
		u32 first;
	} __packed req;
	struct mbdb_op_param op_param;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, sizeof(req), rsp, sizeof(*rsp), &cw,
						 NON_POSTED_CHECK_OP);
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	req.cnt = cnt;
	req.first = first;

	op_param.len = sizeof(req);
	op_param.data = &req;

	return ops_execute(sd, &cw, 1, &op_param, ibox);
}

int ops_serdes_eqinfo_get(struct fsubdev *sd, u32 port, struct mbdb_serdes_eq_info_get_rsp *rsp)
{
	const u8 op_code = MBOX_OP_CODE_SERDES_EQINFO_GET;
	struct mbdb_ibox *ibox;
	struct mbdb_serdes_eqinfo_req {
		u32 port;
	} __packed req;
	struct mbdb_op_param op_param;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox_set_hdlr(sd, op_code, sizeof(req), rsp,
							  sizeof(*rsp), &cw, NON_POSTED_CHECK_OP,
							  ops_serdes_eqinfo_rsp_handler);
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	req.port = port;

	op_param.len = sizeof(req);
	op_param.data = &req;

	return ops_execute(sd, &cw, 1, &op_param, ibox);
}

int ops_serdes_histogram_get(struct fsubdev *sd, u32 port, struct mbdb_serdes_histogram_rsp *rsp)
{
	const u8 op_code = MBOX_OP_CODE_SERDES_HISTOGRAM_GET;
	struct mbdb_ibox *ibox;
	struct mbdb_serdes_histogram_req {
		u32 port;
	} __packed req;
	struct mbdb_op_param op_param;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, sizeof(req), rsp, sizeof(*rsp), &cw,
						 NON_POSTED_CHECK_OP);
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	req.port = port;

	op_param.len = sizeof(req);
	op_param.data = &req;

	return ops_execute(sd, &cw, 1, &op_param, ibox);
}

int ops_statedump(struct fsubdev *sd, u32 offset, struct mbdb_statedump_rsp *rsp)
{
	const u8 op_code = MBOX_OP_CODE_STATE_DUMP;
	struct mbdb_ibox *ibox;
	struct mbdb_statedump_req {
		u32 offset;
	} __packed req;
	struct mbdb_op_param op_param;
	u64 cw;
	int ret;

	ibox = mbdb_op_build_cw_and_acquire_ibox_set_hdlr(sd, op_code, sizeof(req), rsp->descs,
							  sizeof(rsp->descs), &cw,
							  NON_POSTED_CHECK_OP,
							  ops_rsp_handler_relaxed);
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	req.offset = offset;

	op_param.len = sizeof(req);
	op_param.data = &req;

	ret = ops_execute_nolock(sd, &cw, 1, &op_param, ibox);
	if (!ret)
		rsp->desc_count = mbdb_mbox_params_len(cw) / sizeof(rsp->descs[0]);

	return ret;
}

int ops_port_status_get(struct fsubdev *sd, u32 port, u8 num_csr_ranges,
			const struct mbdb_op_csr_range *csr_ranges,
			struct mbdb_op_port_status_get_rsp *rsp)
{
	const u8 op_code = MBOX_OP_CODE_PORT_STATUS_GET;
	struct mbdb_ibox *ibox;
	struct mbdb_op_port_status_get_req {
		u32 port;
		u32 reserved;
	} __packed req;
	struct mbdb_op_param op_param[2];
	u64 cw;
	int csr_count = 0;
	int i;
	size_t req_len;
	size_t rsp_len;

	for (i = 0; i < num_csr_ranges; i++)
		csr_count += csr_ranges[i].num_csrs;

	if (!csr_count || csr_count > MAX_CSRS) {
		sd_err(sd, "Wrong number of csrs(%d) specified\n", csr_count);
		return -EINVAL;
	}

	req_len = sizeof(req) + sizeof(*csr_ranges) * num_csr_ranges;
	rsp_len = sizeof(*rsp) + CSR_SIZE * csr_count;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, req_len, rsp, rsp_len, &cw,
						 NON_POSTED_CHECK_OP);
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	req.port = port;
	req.reserved = 0;

	op_param[0].len = sizeof(req);
	op_param[0].data = &req;
	/*
	 * The req struct which only identifies the port on which to act is
	 * immediately followed by a list of csr ranges to be obtained.
	 */
	op_param[1].len = sizeof(*csr_ranges) * num_csr_ranges;
	op_param[1].data = csr_ranges;

	return ops_execute(sd, &cw, ARRAY_SIZE(op_param), op_param, ibox);
}

int ops_linkmgr_port_lqi_trap_ack(struct fsubdev *sd, bool posted)
{
	const u8 op_code = MBOX_OP_CODE_LINK_MGR_PORT_LQI_TRAP_ACKNOWLEDGE;
	struct mbdb_ibox *ibox;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, 0, NULL, 0, &cw,
						 POSTED_CHECK_OP(posted));
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	return ops_execute(sd, &cw, 0, NULL, ibox);
}

int ops_linkmgr_port_lqi_trap_ena_set(struct fsubdev *sd, bool enabled,
				      bool posted)
{
	const u8 op_code = MBOX_OP_CODE_LINK_MGR_PORT_LQI_TRAP_ENABLE_SET;
	struct mbdb_ibox *ibox;
	struct mbdb_op_enable_param req;
	struct mbdb_op_param op_param;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, sizeof(req), NULL, 0, &cw,
						 POSTED_CHECK_OP(posted));
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	mbdb_op_enable_param_set(&req, enabled);

	op_param.len = sizeof(req);
	op_param.data = &req;

	return ops_execute(sd, &cw, 1, &op_param, ibox);
}

int ops_linkmgr_port_lwd_trap_ack(struct fsubdev *sd, bool posted)
{
	const u8 op_code = MBOX_OP_CODE_LINK_MGR_PORT_LWD_TRAP_ACKNOWLEDGE;
	struct mbdb_ibox *ibox;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, 0, NULL, 0, &cw,
						 POSTED_CHECK_OP(posted));
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	return ops_execute(sd, &cw, 0, NULL, ibox);
}

int ops_linkmgr_port_lwd_trap_ena_set(struct fsubdev *sd, bool enabled,
				      bool posted)
{
	const u8 op_code = MBOX_OP_CODE_LINK_MGR_PORT_LWD_TRAP_ENABLE_SET;
	struct mbdb_ibox *ibox;
	struct mbdb_op_enable_param req;
	struct mbdb_op_param op_param;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, sizeof(req), NULL, 0, &cw,
						 POSTED_CHECK_OP(posted));
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	mbdb_op_enable_param_set(&req, enabled);

	op_param.len = sizeof(req);
	op_param.data = &req;

	return ops_execute(sd, &cw, 1, &op_param, ibox);
}

int ops_rpipe_get(struct fsubdev *sd, struct mbdb_op_rpipe_get *req,
		  struct mbdb_op_rpipe_get *rsp)
{
	const u8 op_code = MBOX_OP_CODE_RPIPE_GET;
	struct mbdb_ibox *ibox;
	struct mbdb_op_param op_param;
	u64 cw;
	size_t rsp_len;

	rsp_len = sizeof(*rsp) + (req->num_entries * sizeof(u8));
	if (rsp_len > MBOX_PARAM_AREA_IN_BYTES) {
		sd_err(sd, "Too many entries specified.\n");
		return -EINVAL;
	}

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, sizeof(*req), rsp, rsp_len, &cw,
						 NON_POSTED_CHECK_OP);
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	op_param.len = sizeof(*req);
	op_param.data = req;

	return ops_execute(sd, &cw, 1, &op_param, ibox);
}

int ops_rpipe_set(struct fsubdev *sd, struct mbdb_op_rpipe_set *req,
		  struct mbdb_op_rpipe_set *rsp, bool posted)
{
	const u8 op_code = MBOX_OP_CODE_RPIPE_SET;
	struct mbdb_ibox *ibox;
	struct mbdb_op_param op_param;
	u64 cw;
	size_t req_len;
	size_t rsp_len;

	if (WARN_ON(!posted && !rsp))
		return -EINVAL;

	req_len = sizeof(*req) + (req->num_entries * sizeof(u8));
	rsp_len = posted ? 0 : sizeof(*rsp) + (req->num_entries * sizeof(u8));
	if (req_len > MBOX_PARAM_AREA_IN_BYTES ||
	    rsp_len > MBOX_PARAM_AREA_IN_BYTES) {
		sd_err(sd, "Too many entries specified\n");
		return -EINVAL;
	}

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, req_len, rsp, rsp_len, &cw,
						 POSTED_CHECK_OP(posted));
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	op_param.len = req_len;
	op_param.data = req;

	return ops_execute(sd, &cw, 1, &op_param, ibox);
}

void ops_portinfo_rsp_handler(struct mbdb_ibox *ibox)
{
	u16 rsp_len = mbdb_mbox_params_len(ibox->cw);
	struct mbdb_op_portinfo *rsp;
	struct portinfo *per_portinfo;
	u8 port_count;
	size_t fw_portinfo_sz;
	u8 __iomem *gp_inbox_param_addr;

	if (likely(rsp_len == ibox->rsp_len)) {
		io_readq_aligned(mbdb_ibox_gp_inbox_param_addr(ibox), ibox->response, rsp_len);
		return;
	}

	sd_warn(mbdb_ibox_sd(ibox), "mbdb inbox rsp_len mismatch: cw 0x%016llx actual %u expected %u\n",
		ibox->cw, rsp_len, ibox->rsp_len);

	gp_inbox_param_addr = (u8 __iomem *)mbdb_ibox_gp_inbox_param_addr(ibox);

	rsp = ibox->response;
	per_portinfo = rsp->per_portinfo;

	io_readq_aligned((u64 __iomem *)gp_inbox_param_addr, rsp, sizeof(*rsp));

	port_count = hweight32(rsp->port_mask);

	fw_portinfo_sz = (rsp_len - sizeof(*rsp)) / port_count;

	ops_element_copy((void __iomem *)(gp_inbox_param_addr + sizeof(*rsp)), per_portinfo,
			 fw_portinfo_sz, sizeof(struct portinfo), port_count);
}

int ops_portinfo_get(struct fsubdev *sd, u32 port_mask,
		     struct mbdb_op_portinfo *rsp)
{
	const u8 op_code = MBOX_OP_CODE_PORTINFO_GET;
	struct mbdb_ibox *ibox;
	struct mbdb_op_param op_param;
	u64 cw;
	size_t rsp_len;

	rsp_len = sizeof(*rsp) + (hweight32(port_mask) * sizeof(struct portinfo));
	if (rsp_len > MBOX_PARAM_AREA_IN_BYTES) {
		sd_err(sd, "Too many ports specified.\n");
		return -EINVAL;
	}

	ibox = mbdb_op_build_cw_and_acquire_ibox_set_hdlr(sd, op_code, sizeof(port_mask), rsp,
							  rsp_len, &cw, NON_POSTED_CHECK_OP,
							  ops_portinfo_rsp_handler);
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	op_param.len = sizeof(port_mask);
	op_param.data = &port_mask;

	return ops_execute(sd, &cw, 1, &op_param, ibox);
}

int ops_switchinfo_set(struct fsubdev *sd, struct mbdb_op_switchinfo *req,
		       struct mbdb_op_switchinfo *rsp, bool posted)
{
	const u8 op_code = MBOX_OP_CODE_SWITCHINFO_SET;
	struct mbdb_ibox *ibox;
	struct mbdb_op_param op_param;
	u64 cw;

	if (WARN_ON(!posted && !rsp))
		return -EINVAL;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, sizeof(*req), rsp,
						 posted ? 0 : sizeof(*rsp), &cw,
						 POSTED_CHECK_OP(posted));
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	op_param.len = sizeof(*req);
	op_param.data = req;

	return ops_execute(sd, &cw, 1, &op_param, ibox);
}

int ops_switchinfo_get(struct fsubdev *sd, struct mbdb_op_switchinfo *rsp)
{
	const u8 op_code = MBOX_OP_CODE_SWITCHINFO_GET;
	struct mbdb_ibox *ibox;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, 0, rsp, sizeof(*rsp), &cw,
						 NON_POSTED_CHECK_OP);
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	return ops_execute(sd, &cw, 0, NULL, ibox);
}

int ops_linkmgr_psc_trap_ack(struct fsubdev *sd, bool posted)
{
	const u8 op_code = MBOX_OP_CODE_LINK_MGR_PORT_STATE_CHANGE_TRAP_ACKNOWLEDGE;
	struct mbdb_ibox *ibox;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, 0, NULL, 0, &cw,
						 POSTED_CHECK_OP(posted));
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	return ops_execute(sd, &cw, 0, NULL, ibox);
}

int ops_linkmgr_psc_trap_ena_set(struct fsubdev *sd, bool enabled, bool posted)
{
	const u8 op_code = MBOX_OP_CODE_LINK_MGR_PORT_STATE_CHANGE_TRAP_ENABLE_SET;
	struct mbdb_ibox *ibox;
	struct mbdb_op_enable_param req;
	struct mbdb_op_param op_param;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, sizeof(req), NULL, 0, &cw,
						 POSTED_CHECK_OP(posted));
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	mbdb_op_enable_param_set(&req, enabled);

	op_param.len = sizeof(req);
	op_param.data = &req;

	return ops_execute(sd, &cw, 1, &op_param, ibox);
}

int ops_linkmgr_port_beacon_set(struct fsubdev *sd, u32 port, bool enable)
{
	const u8 op_code = MBOX_OP_CODE_LINK_MGR_PORT_BEACON_SET;
	struct mbdb_ibox *ibox;
	struct linkmgr_port_beacon_set_req {
		u32 port;
		struct mbdb_op_enable_param enable;
	} __packed req;
	struct mbdb_op_param op_param;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, sizeof(req), NULL, 0, &cw,
						 NON_POSTED_CHECK_OP);
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	req.port = port;
	mbdb_op_enable_param_set(&req.enable, enable);

	op_param.len = sizeof(req);
	op_param.data = &req;

	return ops_execute(sd, &cw, 1, &op_param, ibox);
}

int ops_linkmgr_ps_get(struct fsubdev *sd, u32 port, u32 *result)
{
	const u8 op_code = MBOX_OP_CODE_LINK_MGR_PORT_STATES_GET;
	struct mbdb_ibox *ibox;
	struct mbdb_op_param op_param;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, sizeof(port), result, sizeof(*result),
						 &cw, NON_POSTED_CHECK_OP);
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	op_param.len = sizeof(port);
	op_param.data = &port;

	return ops_execute(sd, &cw, 1, &op_param, ibox);
}

int ops_qsfpmgr_fault_trap_ack(struct fsubdev *sd, bool posted)
{
	const u8 op_code = MBOX_OP_CODE_QSFP_MGR_FAULT_TRAP_ACKNOWLEDGE;
	struct mbdb_ibox *ibox;
	u64 cw;

	/*
	 * This opcode is optional and may not be supported in firmware.
	 * Return EIO so the caller logs a message indicating we cannot issue the ack opcode.
	 */
	if (!test_bit(op_code, sd->fw_version.supported_opcodes))
		return -EIO;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, 0, NULL, 0, &cw,
						 POSTED_CHECK_OP(posted));
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	return ops_execute(sd, &cw, 0, NULL, ibox);
}

int ops_reset(struct fsubdev *sd, bool posted)
{
	const u8 op_code = MBOX_OP_CODE_RESET;
	struct mbdb_ibox *ibox;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, 0, NULL, 0, &cw,
						 POSTED_CHECK_OP(posted));
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	return ops_execute(sd, &cw, 0, NULL, ibox);
}

int ops_linkmgr_port_csr_wr(struct fsubdev *sd, u32 lpn, u32 offset,
			    const void *data, u16 len, bool posted)
{
	const u8 op_code = MBOX_OP_CODE_LINK_MGR_PORT_CSR_WR;
	struct mbdb_ibox *ibox;
	struct linkmgr_port_csr_wr_req {
		u32 lpn;
		u32 offset;
	} __packed req;
	struct mbdb_op_param op_param[2];
	u64 cw;

	if (WARN_ON(!FIELD_FIT(MBOX_CW_PARAMS_LEN, len)))
		return -EINVAL;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, len, NULL, 0, &cw,
						 POSTED_CHECK_OP(posted));
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	req.lpn = lpn;
	req.offset = offset;

	op_param[0].len = sizeof(req);
	op_param[0].data = &req;
	op_param[1].len = len;
	op_param[1].data = data;

	return ops_execute(sd, &cw, ARRAY_SIZE(op_param), op_param, ibox);
}

int ops_linkmgr_port_csr_rd(struct fsubdev *sd, u32 lpn, u32 offset, u16 len,
			    u64 *result)
{
	const u8 op_code = MBOX_OP_CODE_LINK_MGR_PORT_CSR_RD;
	struct mbdb_ibox *ibox;
	struct linkmgr_port_csr_rd_req {
		u32 lpn;
		u32 offset;
	} __packed req;
	struct mbdb_op_param op_param;
	u64 cw;

	if (WARN_ON(!FIELD_FIT(MBOX_CW_PARAMS_LEN, len)))
		return -EINVAL;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, len, result, len, &cw,
						 NON_POSTED_CHECK_OP);
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	req.lpn = lpn;
	req.offset = offset;

	op_param.len = sizeof(req);
	op_param.data = &req;

	return ops_execute(sd, &cw, 1, &op_param, ibox);
}

int ops_linkmgr_port_maj_phystate_set(struct fsubdev *sd, u32 port,
				      u32 new_state, u32 *result)
{
	const u8 op_code = MBOX_OP_CODE_LINK_MGR_PORT_MAJOR_PHYSICAL_STATE_SET;
	struct mbdb_ibox *ibox;
	struct linkmgr_port_maj_phystate_set_req {
		u32 port;
		u32 new_state;
	} __packed req;
	struct mbdb_op_param op_param;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, sizeof(req), result, sizeof(*result),
						 &cw, NON_POSTED_CHECK_OP);
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	req.port = port;
	req.new_state = new_state;

	op_param.len = sizeof(req);
	op_param.data = &req;

	return ops_execute(sd, &cw, 1, &op_param, ibox);
}

int ops_linkmgr_port_link_state_set(struct fsubdev *sd, u32 port, u32 new_state,
				    u32 *result)
{
	const u8 op_code = MBOX_OP_CODE_LINK_MGR_PORT_LINK_STATE_SET;
	struct mbdb_ibox *ibox;
	struct linkmgr_port_link_state_set_req {
		u32 port;
		u32 new_state;
	} __packed req;
	struct mbdb_op_param op_param;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, sizeof(req), result, sizeof(*result),
						 &cw, NON_POSTED_CHECK_OP);
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	req.port = port;
	req.new_state = new_state;

	op_param.len = sizeof(req);
	op_param.data = &req;

	return ops_execute(sd, &cw, 1, &op_param, ibox);
}

int ops_ini_loaded_set(struct fsubdev *sd, bool posted)
{
	const u8 op_code = MBOX_OP_CODE_INI_LOADED_SET;
	struct mbdb_ibox *ibox;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, 0, NULL, 0, &cw,
						 POSTED_CHECK_OP(posted));
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	return ops_execute(sd, &cw, 0, NULL, ibox);
}

int ops_ini_table_load(struct fsubdev *sd,
		       struct mbdb_op_ini_table_load_req *info, u32 *result)
{
	const u8 op_code = MBOX_OP_CODE_INI_TABLE_LOAD;
	struct mbdb_ibox *ibox;
	struct mbdb_op_param op_param;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, sizeof(*info), result,
						 sizeof(*result), &cw, NON_POSTED_CHECK_OP);
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	op_param.len = sizeof(*info);
	op_param.data = info;

	return ops_execute(sd, &cw, 1, &op_param, ibox);
}

int ops_walloc(struct fsubdev *sd, u16 dwords, u32 *addr)
{
	const u8 op_code = MBOX_OP_CODE_WALLOC;
	struct mbdb_ibox *ibox;
	struct mbdb_op_param op_param;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, sizeof(dwords), addr, sizeof(*addr),
						 &cw, NON_POSTED_CHECK_OP);
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	op_param.len = sizeof(dwords);
	op_param.data = &dwords;

	return ops_execute(sd, &cw, 1, &op_param, ibox);
}

int ops_fw_start(struct fsubdev *sd)
{
	const u8 op_code = MBOX_OP_CODE_FW_START;
	struct mbdb_ibox *ibox;
	u32 result = 0;
	u64 cw;
	int err;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, 0, &result, sizeof(result), &cw,
						 NON_POSTED_CHECK_OP);
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	err = ops_execute(sd, &cw, 0, NULL, ibox);
	if (err)
		return err;

	/* map fw start failures to an op status error */
	return result ? MBOX_RSP_STATUS_LOGICAL_ERROR : 0;
}

int ops_csr_raw_write(struct fsubdev *sd, u32 addr, const void *data, u32 len,
		      bool posted)
{
	const u8 op_code = MBOX_OP_CODE_CSR_RAW_WR;
	struct mbdb_ibox *ibox = NULL;
	struct csr_raw_write_req {
		u32 addr;
		u32 reserved;
	} __packed req;
	struct mbdb_op_param op_param[2];
	u64 cw;

	if (len > MBOX_WRITE_DATA_SIZE_IN_BYTES) {
		sd_err(sd, "Write of raw buffer to mailbox, max size was exceeded. len %d\n",
		       len);
		return -EINVAL;
	}

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, len, NULL, 0, &cw,
						 POSTED_CHECK_OP(posted));
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	req.addr = addr;
	req.reserved = 0;

	op_param[0].len = sizeof(req);
	op_param[0].data = &req;
	op_param[1].len = len;
	op_param[1].data = data;

	return ops_execute(sd, &cw, ARRAY_SIZE(op_param), op_param, ibox);
}

static int ops_csr_raw_read_exec(struct fsubdev *sd, u32 addr, size_t read_len, void *data,
				 enum lock_state lockit)
{
	const u8 op_code = MBOX_OP_CODE_CSR_RAW_RD;
	struct mbdb_ibox *ibox;
	struct csr_raw_write_req {
		u32 addr;
	} __packed req;
	struct mbdb_op_param op_param;
	u64 cw;

	if (read_len > MBOX_READ_DATA_SIZE_IN_BYTES) {
		sd_err(sd, "Read of raw buffer from mailbox, max size was exceeded. read_len %ld\n",
		       read_len);
		return -EINVAL;
	}

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, read_len, data, read_len, &cw,
						 NON_POSTED_CHECK_OP);
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	req.addr = addr;

	op_param.len = sizeof(req);
	op_param.data = &req;

	if (lockit == LOCK)
		return ops_execute(sd, &cw, 1, &op_param, ibox);

	return ops_execute_nolock(sd, &cw, 1, &op_param, ibox);
}

int ops_csr_raw_read_nolock(struct fsubdev *sd, u32 addr, size_t read_len, void *data)
{
	return ops_csr_raw_read_exec(sd, addr, read_len, data, NOLOCK);
}

int ops_csr_raw_read(struct fsubdev *sd, u32 addr, size_t read_len, void *data)
{
	return ops_csr_raw_read_exec(sd, addr, read_len, data, LOCK);
}

int ops_fw_version(struct fsubdev *sd,
		   struct mbdb_op_fw_version_rsp *fw_version)
{
	const u8 op_code = MBOX_OP_CODE_FW_VERSION;
	struct mbdb_ibox *ibox;
	u64 cw;

	ibox = mbdb_op_build_cw_and_acquire_ibox(sd, op_code, 0, fw_version, sizeof(*fw_version),
						 &cw, NON_POSTED);
	if (IS_ERR(ibox))
		return PTR_ERR(ibox);

	return ops_execute(sd, &cw, 0, NULL, ibox);
}

int ops_mem_posted_wr(struct fsubdev *sd, u32 addr, const u8 *data, u32 len)
{
	int ret = 0;

	if (!len)
		return -EINVAL;

	while (len) {
		int xfer_len = len <= MBOX_WRITE_DATA_SIZE_IN_BYTES
			       ? len : MBOX_WRITE_DATA_SIZE_IN_BYTES;
		ret = ops_csr_raw_write(sd, addr, data, xfer_len, true);
		if (ret)
			break;

		len -= xfer_len;
		addr += xfer_len;
		data += xfer_len;
	}

	return ret;
}

static int ops_execute_exec(struct fsubdev *sd, u64 *cw, u8 op_param_count,
			    struct mbdb_op_param *op_param, struct mbdb_ibox *ibox);

static int ops_retry(struct fsubdev *sd, u64 *cw, u8 op_param_count, struct mbdb_op_param *op_param,
		     struct mbdb_ibox *ibox)
{
	if (ibox->retries < MAX_OP_RETRIES) {
		struct mbdb_ibox *new_ibox =
			mbdb_op_build_cw_and_acquire_ibox_set_hdlr(sd, mbdb_mbox_op_code(*cw),
								   mbdb_mbox_params_len(*cw),
								   ibox->response, ibox->rsp_len,
								   cw, NON_POSTED,
								   ibox->op_rsp_handler);
		if (IS_ERR(new_ibox))
			return PTR_ERR(new_ibox);

		if (WARN(!new_ibox, " ibox acquire must be NON_POSTED only\n"))
			return -EINVAL;

		new_ibox->retries = ibox->retries + 1;

		msleep(100 * new_ibox->retries);

		sd_dbg(sd, "Retry %u for op code %u\n", new_ibox->retries, mbdb_mbox_op_code(*cw));

		return ops_execute_exec(sd, cw, op_param_count, op_param, new_ibox);
	}

	return -EBUSY;
}

/**
 * ops_execute_exec - Executes a mailbox operation
 *
 * @sd: subdevice to be actioned
 * @cw: mailbox control word
 * @op_param_count: count of parameters in op_param
 * @op_param: array of parameters for the mailbox operation
 * @ibox: when present, the response area for non-posted operations
 *
 * Return: Zero on success, Negative on resource error.
 * Positive on mailbox response status indicating an error from the mailbox
 *
 * Note: The ibox is consumed when present
 *
 */
static int ops_execute_exec(struct fsubdev *sd, u64 *cw, u8 op_param_count,
			    struct mbdb_op_param *op_param, struct mbdb_ibox *ibox)
{
	struct mbox_msg __iomem *mbox_msg;
	u64 __iomem *mbox_msg_param_area;
	int ret = 0;

	mbox_msg = mbdb_outbox_acquire(sd, cw);
	if (IS_ERR(mbox_msg)) {
		mbdb_ibox_release_pre_wait(ibox);
		return PTR_ERR(mbox_msg);
	}

	if (unlikely(READ_ONCE(sd->fdev->dev_disabled)))
		return -EIO;

	writeq(*cw, &mbox_msg->cw);

	mbox_msg_param_area = mbox_msg->param_area;

	if (op_param) {
		struct mbdb_op_param *cur_op_param = op_param;
		u8 i;

		for (i = 0; i < op_param_count; cur_op_param++, i++) {
			io_write(mbox_msg_param_area, cur_op_param->data, cur_op_param->len);
			mbox_msg_param_area += cur_op_param->len >> 3;
		}
	}

	trace_iaf_mbx_exec(sd->fdev->pd->index, sd_index(sd),
			   mbdb_mbox_op_code(*cw), mbdb_mbox_seq_no(*cw),
			   mbdb_mbox_tid(*cw), *cw);

	mbdb_outbox_release(sd);

	if (ibox) {
		ret = mbdb_ibox_wait(ibox);
		if (!ret) {
			if (ibox->rsp_status == MBOX_RSP_STATUS_RETRY) {
				ret = ops_retry(sd, cw, op_param_count, op_param, ibox);
			} else {
				*cw = ibox->cw;
				ret = ibox->rsp_status;
			}
		}

		mbdb_ibox_release(ibox);
	}

	return ret;
}

/**
 * ops_execute_nolock - Executes a mailbox operation with no locking
 *
 * @sd: subdevice to be actioned
 * @cw: mailbox control word
 * @op_param_count: count of parameters in op_param
 * @op_param: array of parameters for the mailbox operation
 * @ibox: when present, the response area for non-posted operations
 *
 * Return: Zero on success, Negative on resource error.
 * Positive on mailbox response status indicating an error from the mailbox
 *
 * Note: The ibox is consumed when present
 *
 */
int ops_execute_nolock(struct fsubdev *sd, u64 *cw, u8 op_param_count,
		       struct mbdb_op_param *op_param, struct mbdb_ibox *ibox)
{
	return ops_execute_exec(sd, cw, op_param_count, op_param, ibox);
}

/**
 * ops_execute - Executes a mailbox operation obtaining a read lock
 *
 * @sd: subdevice to be actioned
 * @cw: mailbox control word
 * @op_param_count: count of parameters in op_param
 * @op_param: array of parameters for the mailbox operation
 * @ibox: when present, the response area for non-posted operations
 *
 * Return: Zero on success, Negative on resource error.
 * Positive on mailbox response status indicating an error from the mailbox
 *
 * Note: The ibox is consumed when present
 *
 */
int ops_execute(struct fsubdev *sd, u64 *cw, u8 op_param_count,
		struct mbdb_op_param *op_param, struct mbdb_ibox *ibox)
{
	int ret;

	ret = down_read_killable(&sd->statedump.state_dump_mbdb_sem);
	if (ret) {
		mbdb_ibox_release_pre_wait(ibox);
		return ret;
	}

	ret = ops_execute_exec(sd, cw, op_param_count, op_param, ibox);
	up_read(&sd->statedump.state_dump_mbdb_sem);

	return ret;
}
