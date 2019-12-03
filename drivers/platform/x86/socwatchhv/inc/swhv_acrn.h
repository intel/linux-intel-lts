/* SPDX-License-Identifier: GPL-2.0 AND BSD-3-Clause
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2014 - 2019 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Contact Information:
 * SoC Watch Developer Team <socwatchdevelopers@intel.com>
 * Intel Corporation,
 * 1300 S Mopac Expwy,
 * Austin, TX 78746
 *
 * BSD LICENSE
 *
 * Copyright(c) 2014 - 2019 Intel Corporation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _SWHV_ACRN_H_
#define _SWHV_ACRN_H_ 1

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <asm/io.h>
#include <linux/version.h> /* LINUX_VERSION_CODE */
#include <linux/list.h> /* for struct list_head */

#include "swhv_defines.h"
#include "pw_version.h"

#define SW_DEFINE_LIST_HEAD(name, dummy)            struct list_head name
#define SW_DECLARE_LIST_HEAD(name, dummy)           extern struct list_head name
#define SW_LIST_ENTRY(name, dummy)                  struct list_head name
#define SW_LIST_HEAD_VAR(dummy)                     struct list_head
#define SW_LIST_HEAD_INIT(head)                     INIT_LIST_HEAD(head)
#define SW_LIST_ENTRY_INIT(node, field)             INIT_LIST_HEAD(&node->field)
#define SW_LIST_ADD(head, node, field)              list_add_tail(&node->field, head)
#define SW_LIST_GET_HEAD_ENTRY(head, type, field)   list_first_entry(head, struct type, field)
#define SW_LIST_UNLINK(node, field)                 list_del(&node->field)
#define SW_LIST_FOR_EACH_ENTRY(node, head, field)   list_for_each_entry(node, head, field)
#define SW_LIST_EMPTY(head)                         list_empty(head)
#define SW_LIST_HEAD_INITIALIZER(head)              LIST_HEAD_INIT(head)

int device_open_i(struct inode *inode, struct file *file);

ssize_t device_read_i(struct file *file, /* see include/linux/fs.h   */
	char __user *buffer, /* buffer to be filled with data */
	size_t length, /* length of the buffer */
	loff_t *offset);

long swhv_configure(struct swhv_driver_interface_msg __user *remote_msg, int local_len);
long swhv_start(void);
long swhv_stop(void);
long swhv_get_cpu_count(u32 __user *remote_args);
long swhv_get_clock(u32 __user *remote_in_args, u64 __user *remote_args);
long swhv_get_topology(u64 __user *remote_args);
long swhv_get_hypervisor_type(u32 __user *remote_args);
int swhv_load_driver_i(void);
void swhv_unload_driver_i(void);
void cleanup_error_i(void);
long swhv_msr_read(u32 __user *remote_in_args, u64 __user *remote_args);
long swhv_collection_poll(void);

enum MSR_CMD_TYPE {
    MSR_OP_NONE = 0,
    MSR_OP_READ,
    MSR_OP_WRITE,
    MSR_OP_READ_CLEAR
};

enum MSR_CMD_STATUS {
    MSR_OP_READY = 0,
    MSR_OP_REQUESTED,
    MSR_OP_HANDLED
};

struct profiling_msr_op {
    /* value to write or location to write into */
    uint64_t value;
    /* MSR address to read/write; last entry will have value of -1 */
    uint32_t msr_id;
    /* parameter; usage depends on operation */
    uint16_t param;
    uint8_t msr_op_type;
    uint8_t reg_type;
};

#define MAX_MSR_LIST_NUM 15
struct profiling_msr_ops_list {
    int32_t collector_id;
    uint32_t num_entries;
    int32_t msr_op_state;   /* enum value from 'MSR_CMD_STATUS' */
    struct profiling_msr_op entries[MAX_MSR_LIST_NUM];
};

#define COLLECTOR_SOCWATCH 1

struct profiling_control {
    int32_t collector_id;
    int32_t reserved;
    uint64_t switches;
};

/**
 * struct - swhv_acrn_msr_collector_data
 * Information about the MSR collector to be invoked at collection time.
 *
 * @list:                   List/link implementation
 * @cpu_mask:               Collect if cpu matches mask
 * @sample_id:              ID of the metric requesting these operations
 * @msr_ops_list:           Ptr to list of MSR read/write operations
 * @per_msg_payload_size:   Data size
 */
typedef struct swhv_acrn_msr_collector_data {
    SW_LIST_ENTRY(list, swhv_acrn_msr_collector_data);
    pw_s16_t                        cpu_mask;
    pw_s16_t                        sample_id;
    struct profiling_msr_ops_list *msr_ops_list;
    size_t                          per_msg_payload_size;
} swhv_acrn_msr_collector_data_t;
#endif /* _SWHV_ACRN_H_ */
