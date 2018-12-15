/*

  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY

  Copyright(c) 2014 - 2018 Intel Corporation.

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  Contact Information:
  SoC Watch Developer Team <socwatchdevelopers@intel.com>
  Intel Corporation,
  1300 S Mopac Expwy,
  Austin, TX 78746

  BSD LICENSE

  Copyright(c) 2014 - 2018 Intel Corporation.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#ifndef __SW_COLLECTOR_H__

#include "sw_internal.h"

/*
 * Forward declaration
 */
struct sw_hw_ops;

/* TODO: convert from 'list_head' to 'hlist_head' */
/**
 * struct - sw_collector_data
 * Information about the collector to be invoked at collection time.
 *
 * The collector_lists array holds linked lists of collectors to
 * be exercised at specific points in time during the collection
 * (e.g. begin, poll, end, etc.).  At a trigger time, the driver walks
 * that time's list of nodes, and exercises the collectors on that list.
 *
 * @list:                   List/link implementation
 * @cpumask:                Collect if cpu matches mask
 * @info:                   Ptr to metric info
 * @ops:                    Ptr to collector's operations
 * @last_update_jiffies:    Indicates when this node was last exercised.
 * @per_msg_payload_size:   Data size
 * @msg:                    Ptr to collected data
 */
typedef struct sw_collector_data {
	SW_LIST_ENTRY(list, sw_collector_data);
	struct cpumask cpumask;
	struct sw_driver_interface_info *info;
	const struct sw_hw_ops **ops;
	size_t per_msg_payload_size;
	u64 last_update_jiffies;
	struct sw_driver_msg *msg;
} sw_collector_data_t;
#define GET_MSG_SLOT_FOR_CPU(msgs, cpu, size)                                  \
	((struct sw_driver_msg *)&(                                            \
		((char *)(msgs))[(cpu) *                                       \
				 (sizeof(struct sw_driver_msg) + (size))]))

struct sw_collector_data *sw_alloc_collector_node(void);
void sw_free_collector_node(struct sw_collector_data *node);
int sw_handle_collector_node(struct sw_collector_data *data);
int sw_handle_collector_node_on_cpu(struct sw_collector_data *data, int cpu);
int sw_write_collector_node(struct sw_collector_data *data);

void sw_init_collector_list(void *list_head);
void sw_destroy_collector_list(void *list_head);
int sw_handle_collector_list(void *list_head,
			     int (*func)(struct sw_collector_data *data));
int sw_handle_collector_list_on_cpu(void *list_head,
				    int (*func)(struct sw_collector_data *data,
						int cpu),
				    int cpu);

int sw_handle_driver_io_descriptor(
	char *dst_vals, int cpu,
	const struct sw_driver_io_descriptor *descriptor,
	const struct sw_hw_ops *hw_ops);
int sw_init_driver_io_descriptor(struct sw_driver_io_descriptor *descriptor);
int sw_reset_driver_io_descriptor(struct sw_driver_io_descriptor *descriptor);

int sw_add_driver_info(void *list_head,
		       const struct sw_driver_interface_info *info);

void sw_handle_per_cpu_msg(void *info);
void sw_handle_per_cpu_msg_no_sched(void *info);
void sw_handle_per_cpu_msg_on_cpu(int cpu, void *info);

void sw_set_collector_ops(const struct sw_hw_ops *hw_ops);

/**
 * Process all messages for the given time.
 * @param[in]   when    The time period e.g. 'BEGIN' or 'END'
 *
 * @returns     0   on success, non-zero on error
 */
extern int sw_process_snapshot(enum sw_when_type when);
extern int sw_process_snapshot_on_cpu(enum sw_when_type when, int cpu);
#endif /* __SW_COLLECTOR_H__ */
