/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2013 - 2018 Intel Corporation */

#ifndef ICI_FW_ISYS_H
#define ICI_FW_ISYS_H

#include "ipu-fw-com.h"

struct ici_isys;
int ici_fw_isys_init(struct ici_isys *isys, unsigned int num_streams);
int ici_fw_isys_close(struct ici_isys *isys);
int ici_fw_isys_simple_cmd(struct ici_isys *isys,
                           const unsigned int stream_handle,
                           enum ipu_fw_isys_send_type send_type);
int ici_fw_isys_complex_cmd(struct ici_isys *isys,
                            const unsigned int stream_handle,
                            void *cpu_mapped_buf,
                            dma_addr_t dma_mapped_buf,
                            size_t size, enum ipu_fw_isys_send_type send_type);
int ici_fw_isys_send_proxy_token(struct ici_isys *isys,
                                 unsigned int req_id,
                                 unsigned int index,
                                 unsigned int offset, u32 value);
void ici_fw_isys_cleanup(struct ici_isys *isys);
#endif
