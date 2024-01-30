/* SPDX-License-Identifier: MIT */
/*
 * Copyright(c) 2020 - 2023 Intel Corporation.
 *
 */

#ifndef _FW_H_
#define _FW_H_

#include "iaf_drv.h"

#define FW_VERSION_INIT_BIT	BIT(1)
#define FW_VERSION_ENV_BIT	BIT(0)

void fw_init_module(void);
void fw_init_dev(struct fdev *dev);
int load_and_init_fw(struct fdev *dev);
void flush_any_outstanding_fw_initializations(struct fdev *dev);
bool is_opcode_valid(struct fsubdev *sd, u8 op_code);

#endif
