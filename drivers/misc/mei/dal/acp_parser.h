/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright(c) 2016-2019, Intel Corporation.
 */
#ifndef _ACP_PARSER_H
#define _ACP_PARSER_H

#include "acp_format.h"

/**
 * struct ac_ins_jta_pack_ext - parsed ta pack from acp file
 *
 * @head: acp pack header
 * @cmd_pack: ta installation information pack
 * @ta_pack: raw ta pack
 */
struct ac_ins_jta_pack_ext {
	struct ac_pack_header *head;
	struct ac_ins_jta_pack cmd_pack;
	char *ta_pack;
} __packed;

/**
 * struct ac_ins_jta_prop_ext - parsed ta properties information
 *                              from acp file
 *
 * @cmd_pack: ta installation properties pack
 * @jeff_pack: ta jeff pack
 */
struct ac_ins_jta_prop_ext {
	struct ac_ins_jta_prop cmd_pack;
	char *jeff_pack;
} __packed;

int acp_pload_ins_jta(const void *raw_data, unsigned int size,
		      struct ac_ins_jta_pack_ext *pack);

#endif /* _ACP_PARSER_H */
