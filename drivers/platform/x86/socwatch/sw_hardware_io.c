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

#include "sw_types.h"
#include "sw_kernel_defines.h"
#include "sw_ops_provider.h"
#include "sw_mem.h"
#include "sw_internal.h"
#include "sw_hardware_io.h"

struct sw_ops_node {
	const struct sw_hw_ops *op;
	int id;

	SW_LIST_ENTRY(list, sw_ops_node);
};

static SW_DEFINE_LIST_HEAD(s_ops,
sw_in			   sw_ops_node) = SW_LIST_HEAD_INITIALIZER(s_ops);

static int s_op_idx = -1;

/*
 * Function definitions.
 */
int sw_get_hw_op_id(const struct sw_hw_ops *ops)
{
	if (ops && ops->name) {
		struct sw_ops_node *node = NULL;

		SW_LIST_FOR_EACH_ENTRY(node, &s_ops, list)
		{
			if (node->op->name &&
			    !strcmp(node->op->name, ops->name)) {
				return node->id;
			}
		}
	}
	return -1;
}

const struct sw_hw_ops *sw_get_hw_ops_for(int id)
{
	struct sw_ops_node *node = NULL;

	SW_LIST_FOR_EACH_ENTRY(node, &s_ops, list)
	{
		if (node->id == id) {
			return node->op;
		}
	}
	return NULL;
}

bool sw_is_valid_hw_op_id(int id)
{
	struct sw_ops_node *node = NULL;

	SW_LIST_FOR_EACH_ENTRY(node, &s_ops, list)
	{
		if (node->id == id) {
			return true;
		}
	}
	return false;
}

const char *sw_get_hw_op_abstract_name(const struct sw_hw_ops *op)
{
	if (op) {
		return op->name;
	}
	return NULL;
}

int sw_for_each_hw_op(int (*func)(const struct sw_hw_ops *op, void *priv),
		      void *priv, bool return_on_error) {
	int retval = PW_SUCCESS;
	struct sw_ops_node *node = NULL;

	if (func) {
		SW_LIST_FOR_EACH_ENTRY(node, &s_ops, list)
		{
			if ((*func)(node->op, priv)) {
				retval = -EIO;
				if (return_on_error) {
					break;
				}
			}
		}
	}
	return retval;
}

int sw_register_hw_op(const struct sw_hw_ops *op)
{
	struct sw_ops_node *node = NULL;

	if (!op) {
		pw_pr_error("NULL input node in \"sw_register_hw_op\"");
		return -EIO;
	}
	node = sw_kmalloc(sizeof(struct sw_ops_node), GFP_KERNEL);
	if (!node) {
		pw_pr_error("sw_kmalloc error in \"sw_register_hw_op\"");
		return -ENOMEM;
	}
	node->op = op;
	node->id = ++s_op_idx;
	SW_LIST_ENTRY_INIT(node, list);
	SW_LIST_ADD(&s_ops, node, list);
	return PW_SUCCESS;
}

int sw_register_hw_ops(void)
{
	return sw_register_ops_providers();
}

void sw_free_hw_ops(void)
{
	/*
	 * Free all nodes.
	 */
	while (!SW_LIST_EMPTY(&s_ops)) {
		struct sw_ops_node *node =
			SW_LIST_GET_HEAD_ENTRY(&s_ops, sw_ops_node, list);
		SW_LIST_UNLINK(node, list);
		sw_kfree(node);
	}
	/*
	 * Call our providers to deallocate resources.
	 */
	sw_free_ops_providers();
}
