/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright(c) 2016-2019, Intel Corporation.
 */

#ifndef __BH_INTERNAL_H
#define __BH_INTERNAL_H

#include <linux/list.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/uuid.h>

#include "bh_cmd_defs.h"

/**
 * struct bh_session_record - session record
 *
 * @link: link in dal_dev_session_list of dal fw client
 * @host_id: message/session host id
 * @ta_session_id: session id
 */
struct bh_session_record {
	struct list_head link;
	u64 host_id;
	u64 ta_session_id;
};

/* command buffer size */
#define CMD_BUF_SIZE(cmd) (sizeof(struct bh_command_header) + sizeof(cmd))

/**
 * enum bh_connection_index - connection index to dal fw clients
 *
 * @BH_CONN_IDX_START: start idx
 *
 * @BH_CONN_IDX_IVM: Intel/Issuer Virtual Machine
 * @BH_CONN_IDX_SDM: Security Domain Manager
 * @BH_CONN_IDX_LAUNCHER: Run Time Manager (Launcher)
 *
 * @BH_CONN_MAX : max connection idx
 */
enum bh_connection_index {
	BH_CONN_IDX_START = 0,

	BH_CONN_IDX_IVM = 0,
	BH_CONN_IDX_SDM = 1,
	BH_CONN_IDX_LAUNCHER = 2,

	BH_CONN_MAX
};

u64 bh_get_msg_host_id(void);

struct bh_session_record *bh_session_find(u64 host_id);
void bh_session_add(struct bh_session_record *session);
void bh_session_remove(u64 host_id);

int bh_request(unsigned int conn_idx,
	       void *hdr, unsigned int hdr_len,
	       const void *data, unsigned int data_len,
	       u64 host_id, void **response);

int bh_proxy_check_svl_jta_blocked_state(uuid_t *ta_id);

int bh_proxy_list_jta_packages(unsigned int *count, uuid_t **ta_ids);

int bh_proxy_dnload_jta(uuid_t *ta_id,
			const char *ta_pkg, unsigned int pkg_len);

int bh_proxy_open_jta_session(uuid_t *ta_id,
			      const char *init_buffer, unsigned int init_len,
			      u64 *host_id, const char *ta_pkg,
			      unsigned int pkg_len);

void bh_prep_session_close_cmd(void *cmdbuf, u64 ta_session_id);
#endif /* __BH_INTERNAL_H */
