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
#ifndef __SW_HARDWARE_IO_H__
#define __SW_HARDWARE_IO_H__

#include "sw_structs.h"

typedef int (*sw_io_desc_init_func_t)
	(struct sw_driver_io_descriptor *descriptor);
typedef void (*sw_hardware_op_func_t)
	(char *dst_vals,
	int cpu, const struct sw_driver_io_descriptor *descriptor,
	u16 counter_size_in_bytes);
typedef int (*sw_io_desc_print_func_t)
	(const struct sw_driver_io_descriptor *descriptor);
typedef int (*sw_io_desc_reset_func_t)
	(const struct sw_driver_io_descriptor *descriptor);
typedef bool (*sw_io_desc_available_func_t)(void);
typedef bool (*sw_hw_op_post_config_func_t)(void);

/**
 * struct sw_hw_ops - Operations for each of the HW collection mechanisms
 *                    in swkernelcollector.
 * @name:           A descriptive name used to identify this particular
 *                  operation.
 * @init:           Initialize a metric's collection.
 * @read:           Read a metric's data.
 * @write:          Write to the HW for the metric(?).
 * @print:          Print out the data.
 * @reset:          Opposite of init--called after we're done collecting.
 * @available:      Decide whether this H/W op is available on the current
 *                  platform.
 * @post_config:    Perform any post-configuration steps.
 */
struct sw_hw_ops {
	const char *name;
	sw_io_desc_init_func_t       init;
	sw_hardware_op_func_t        read;
	sw_hardware_op_func_t        write;
	sw_io_desc_print_func_t      print;
	sw_io_desc_reset_func_t      reset;
	sw_io_desc_available_func_t  available;
	sw_hw_op_post_config_func_t  post_config;
};

bool sw_is_valid_hw_op_id(int id);
int sw_get_hw_op_id(const struct sw_hw_ops *op);
const struct sw_hw_ops *sw_get_hw_ops_for(int id);
const char *sw_get_hw_op_abstract_name(const struct sw_hw_ops *op);

int sw_for_each_hw_op(int (*func)(const struct sw_hw_ops *op, void *priv),
			void *priv, bool return_on_error);

/**
 * Add an operation to the list of providers.
 */
int sw_register_hw_op(const struct sw_hw_ops *ops);
/**
 * Register all H/W operations.
 */
int sw_register_hw_ops(void);
/**
 * Unregister previously registered H/W operations.
 */
void sw_free_hw_ops(void);

#endif /* __SW_HARDWARE_IO_H__ */
