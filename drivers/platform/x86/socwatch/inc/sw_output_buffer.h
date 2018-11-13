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

#ifndef _SW_OUTPUT_BUFFER_H_
#define _SW_OUTPUT_BUFFER_H_ 1
/*
 * Special mask for the case where all buffers have been flushed.
 */
// #define sw_ALL_WRITES_DONE_MASK 0xffffffff
#define SW_ALL_WRITES_DONE_MASK ((u32)-1)
/*
 * Special mask for the case where no data is available to be read.
 */
#define SW_NO_DATA_AVAIL_MASK ((u32)-2)

/*
 * Forward declarations.
 */
struct sw_driver_msg;

/*
 * Data structures.
 */
enum sw_wakeup_action {
	SW_WAKEUP_ACTION_DIRECT,
	SW_WAKEUP_ACTION_TIMER,
	SW_WAKEUP_ACTION_NONE,
};

/*
 * Variable declarations.
 */
extern u64 sw_num_samples_produced, sw_num_samples_dropped;
extern unsigned long sw_buffer_alloc_size;
extern int sw_max_num_cpus;
extern wait_queue_head_t sw_reader_queue;

/*
 * Public API.
 */
int sw_init_per_cpu_buffers(void);
void sw_destroy_per_cpu_buffers(void);
void sw_reset_per_cpu_buffers(void);

void sw_count_samples_produced_dropped(void);

int sw_produce_polled_msg(struct sw_driver_msg *, enum sw_wakeup_action);
int sw_produce_generic_msg(struct sw_driver_msg *, enum sw_wakeup_action);

bool sw_any_seg_full(u32 *val, bool is_flush_mode);
size_t sw_consume_data(u32 mask, void __user *buffer, size_t bytes_to_read);

unsigned int sw_get_output_buffer_size(void);

void sw_wait_once(void);
void sw_wakeup(void);

void sw_print_output_buffer_overheads(void);

/*
 * Init reader queue.
 */
int sw_init_reader_queue(void);
/*
 * Destroy reader queue.
 */
void sw_destroy_reader_queue(void);
/*
 * Wakeup client waiting for a full buffer.
 */
void sw_wakeup_reader(enum sw_wakeup_action);
/*
 * Wakeup client waiting for a full buffer, and
 * cancel any timers initialized by the reader
 * subsys.
 */
void sw_cancel_reader(void);
/*
 * Print some stats about the reader subsys.
 */
void sw_print_reader_stats(void);

#endif // _SW_OUTPUT_BUFFER_H_
