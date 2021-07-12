/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright(c) 2016-2019, Intel Corporation.
 */

#ifndef __BH_EXTERNAL_H
#define __BH_EXTERNAL_H

#include <linux/kernel.h>
#include "bh_cmd_defs.h"

# define MSG_SEQ_START_NUMBER BIT_ULL(32)

bool bh_is_initialized(void);
void bh_init_internal(void);
void bh_deinit_internal(void);

int bh_ta_session_open(u64 *host_id, const char *ta_id, const u8 *ta_pkg,
		       size_t pkg_len, const u8 *init_param, size_t init_len);

int bh_ta_session_close(u64 host_id);

int bh_ta_session_command(u64 host_id, int command_id, const void *input,
			  size_t length, void **output, size_t *output_length,
			  int *response_code);

const struct bh_command_header *bh_msg_cmd_hdr(const void *msg, size_t len);

typedef int (*bh_filter_func)(const struct bh_command_header *hdr,
			      size_t count, void *ctx);

int bh_filter_hdr(const struct bh_command_header *hdr, size_t count, void *ctx,
		  const bh_filter_func tbl[]);

bool bh_msg_is_cmd_open_session(const struct bh_command_header *hdr);

const uuid_t *bh_open_session_ta_id(const struct bh_command_header *hdr,
				    size_t count);

void bh_prep_access_denied_response(const char *cmd,
				    struct bh_response_header *res);

bool bh_msg_is_cmd(const void *msg, size_t len);
bool bh_msg_is_response(const void *msg, size_t len);

#endif /* __BH_EXTERNAL_H */
