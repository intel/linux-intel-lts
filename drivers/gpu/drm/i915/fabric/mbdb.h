/* SPDX-License-Identifier: MIT */
/*
 * Copyright(c) 2019 - 2023 Intel Corporation.
 *
 */

#ifndef MBDB_H_INCLUDED
#define MBDB_H_INCLUDED

#include <linux/types.h>
#include <linux/bits.h>
#include <linux/bitfield.h>

#include "iaf_drv.h"

#define MBOX_CW_OPCODE GENMASK_ULL(7, 0)
#define MBOX_CW_IS_REQ BIT_ULL_MASK(8)
#define MBOX_CW_POSTED BIT_ULL_MASK(9)
#define MBOX_CW_SEQ_NO GENMASK_ULL(15, 10)
#define MBOX_CW_PARAMS_LEN GENMASK_ULL(27, 16)
#define MBOX_CW_RSP_STATUS GENMASK_ULL(31, 28)
#define MBOX_CW_TID GENMASK_ULL(63, 32)

#define MBOX_CW_SEQ_NO_MASK 0x3F

#define CP_ADDR_MBDB_BASE 0x6000

enum mbdb_counters {
	MBDB_COUNTERS_FIRST,
	POSTED_REQUESTS = MBDB_COUNTERS_FIRST,
	NON_POSTED_REQUESTS,
	TIMEDOUT_REQUESTS,
	HANDLED_RECEIVED_REQUESTS,
	UNHANDLED_RECEIVED_REQUESTS,
	NON_ERROR_RESPONSES,
	ERROR_RESPONSES,
	UNMATCHED_RESPONSES,
	TIMEDOUT_RESPONSES,
	RETRY_RESPONSES,
	OUTBOUND_SEQNUM_MISMATCHES,
	INBOUND_SEQNUM_MISMATCHES,
	MBDB_COUNTERS_MAX,
};

enum mbdb_msg_type {
	MBOX_RESPONSE = 0,
	MBOX_REQUEST  = 1
};

enum posted {
	MBOX_RESPONSE_REQUESTED    = 0,
	MBOX_NO_RESPONSE_REQUESTED = 1
};

struct mbdb_ibox;

typedef void (*op_response_handler)(struct mbdb_ibox *ibox);

struct mbdb_ibox {
	struct list_head ibox_list_link;
	struct mbdb *mbdb;
	struct completion ibox_full;
	u64 cw;
	u32 tid;
	u16 rsp_len;
	void *response;
	int rsp_status; /* MBDB_RSP_STATUS value from the cw or errno from a response handler */
	op_response_handler op_rsp_handler;
	u8 op_code;
	u8 retries;
};

/* Outbox related */
u8 mbdb_outbox_seqno(struct fsubdev *sd);
struct mbox_msg __iomem *mbdb_outbox_acquire(struct fsubdev *sd, u64 *cw);
void mbdb_outbox_release(struct fsubdev *sd);

/* Communicate miscellaneous info to firmware via MISC_SHARED registers */
void mbdb_tile_number_set(struct fsubdev *sd);

/* ibox related (i.e. virtual inbox) */
struct mbdb_ibox *mbdb_ibox_acquire(struct fsubdev *sd, u8 op_code,
				    void *response, u32 rsp_len, bool posted,
				    op_response_handler op_rsp_handler);

void mbdb_ibox_release_pre_wait(struct mbdb_ibox *ibox);
void mbdb_ibox_release(struct mbdb_ibox *ibox);
int mbdb_ibox_wait(struct mbdb_ibox *ibox);

void destroy_mbdb(struct fsubdev *sd);
int create_mbdb(struct fsubdev *sd);

irqreturn_t mbdb_handle_irq(struct fsubdev *sd);

u64 mbdb_get_mbox_comm_errors(struct fsubdev *sd);

u64 *mbdb_get_mailbox_counters(struct fsubdev *sd);

static inline u64 build_cw(u8 op_code, enum mbdb_msg_type req_rsp,
			   enum posted is_posted, u16 seq_no, u16 length,
			   u32 tid)
{
	return (FIELD_PREP(MBOX_CW_OPCODE, op_code) |
		FIELD_PREP(MBOX_CW_IS_REQ, req_rsp) |
		FIELD_PREP(MBOX_CW_POSTED, is_posted) |
		FIELD_PREP(MBOX_CW_SEQ_NO, seq_no) |
		FIELD_PREP(MBOX_CW_PARAMS_LEN, length) |
		FIELD_PREP(MBOX_CW_TID, tid));
}

static inline u8 mbdb_mbox_op_code(u64 cw)
{
	return FIELD_GET(MBOX_CW_OPCODE, cw);
}

static inline enum mbdb_msg_type mbdb_mbox_msg_type(u64 cw)
{
	return FIELD_GET(MBOX_CW_IS_REQ, cw);
}

static inline enum posted mbdb_mbox_is_posted(u64 cw)
{
	return FIELD_GET(MBOX_CW_POSTED, cw);
}

static inline u8 mbdb_mbox_seq_no(u64 cw)
{
	return FIELD_GET(MBOX_CW_SEQ_NO, cw);
}

static inline u8 mbdb_mbox_seqno_next(u8 seqno)
{
	return (seqno + 1) & MBOX_CW_SEQ_NO_MASK;
}

static inline bool mbdb_mbox_seqno_error(u8 seqno, u8 expected_seqno)
{
	return seqno != expected_seqno;
}

static inline u16 mbdb_mbox_params_len(u64 cw)
{
	return FIELD_GET(MBOX_CW_PARAMS_LEN, cw);
}

static inline u32 mbdb_mbox_tid(u64 cw)
{
	return FIELD_GET(MBOX_CW_TID, cw);
}

static inline u32 mbdb_ibox_tid(struct mbdb_ibox *ibox)
{
	return ibox ? ibox->tid : 0;
}

static inline u8 mbdb_mbox_rsp_status(u64 cw)
{
	return FIELD_GET(MBOX_CW_RSP_STATUS, cw);
}

struct fsubdev *mbdb_ibox_sd(struct mbdb_ibox *ibox);

u64 __iomem *mbdb_ibox_gp_inbox_param_addr(struct mbdb_ibox *ibox);

int mbdb_ibox_waiters(struct fsubdev *sd);

void mbdb_reinit(struct fsubdev *sd);

void mbdb_init_module(void);

#endif
