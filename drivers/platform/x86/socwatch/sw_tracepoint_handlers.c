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
#include "sw_structs.h"
#include "sw_kernel_defines.h"
#include "sw_types.h"
#include "sw_tracepoint_handlers.h"
#include "sw_trace_notifier_provider.h"
#include "sw_mem.h"

/* -------------------------------------------------
 * Data structures and variable definitions.
 * -------------------------------------------------
 */
struct sw_trace_list_node {
	struct sw_trace_notifier_data *data;
	int id;

	SW_LIST_ENTRY(list, sw_trace_list_node);
};
static SW_DEFINE_LIST_HEAD(s_trace_list, sw_trace_list_node) =
				SW_LIST_HEAD_INITIALIZER(s_trace_list);
static SW_DEFINE_LIST_HEAD(s_notifier_list, sw_trace_list_node) =
				SW_LIST_HEAD_INITIALIZER(s_notifier_list);
static int s_trace_idx = -1, s_notifier_idx = -1;

SW_DEFINE_LIST_HEAD(sw_topology_list, sw_topology_node) =
				SW_LIST_HEAD_INITIALIZER(sw_topology_list);
size_t sw_num_topology_entries;


/* -------------------------------------------------
 * Function definitions.
 * -------------------------------------------------
 */
int sw_extract_tracepoints(void)
{
	return sw_extract_trace_notifier_providers();
}

void sw_reset_trace_notifier_lists(void)
{
	sw_reset_trace_notifier_providers();
}

void sw_print_trace_notifier_overheads(void)
{
	sw_print_trace_notifier_provider_overheads();
}

static int sw_for_each_node_i(
	void *list_head,
	int (*func)(struct sw_trace_notifier_data *node, void *priv),
	void *priv, bool return_on_error)
{
	SW_LIST_HEAD_VAR(sw_trace_list_node) * head = list_head;
	int retval = PW_SUCCESS;
	struct sw_trace_list_node *lnode = NULL;

	SW_LIST_FOR_EACH_ENTRY(lnode, head, list) {
		if ((*func)(lnode->data, priv)) {
			retval = -EIO;
			if (return_on_error)
				break;
		}
	}
	return retval;
}

int sw_for_each_tracepoint_node(
	int (*func)(struct sw_trace_notifier_data *node, void *priv),
	void *priv, bool return_on_error)
{
	if (func)
		return sw_for_each_node_i(&s_trace_list,
			func, priv, return_on_error);

	return PW_SUCCESS;
}

int sw_for_each_notifier_node(
	int (*func)(struct sw_trace_notifier_data *node, void *priv),
	void *priv, bool return_on_error)
{

	if (func)
		return sw_for_each_node_i(&s_notifier_list,
						func, priv, return_on_error);

	return PW_SUCCESS;
}

/*
 * Retrieve the ID for the corresponding tracepoint/notifier.
 */
int sw_get_trace_notifier_id(struct sw_trace_notifier_data *tnode)
{
	struct sw_trace_list_node *lnode = NULL;

	SW_LIST_HEAD_VAR(sw_trace_list_node) * head = (void *)&s_trace_list;
	if (!tnode) {
		pw_pr_error(
			"ERROR: cannot get ID for NULL trace/notifier data!\n");
		return -EIO;
	}
	if (!(tnode->type == SW_TRACE_COLLECTOR_TRACEPOINT ||
			tnode->type == SW_TRACE_COLLECTOR_NOTIFIER)) {
		pw_pr_error(
			"ERROR: cannot get ID for invalid trace/notifier data!\n");
		return -EIO;
	}
	if (!tnode->name || !tnode->name->abstract_name) {
		pw_pr_error(
			"ERROR: cannot get ID for trace/notifier data without valid name!\n");
		return -EIO;
	}

#if defined(LINUX_VERSION_CODE)
#if KERNEL_VERSION(3, 15, 0) <= LINUX_VERSION_CODE &&	\
	defined(CONFIG_TRACEPOINTS)

	if (tnode->type == SW_TRACE_COLLECTOR_TRACEPOINT &&
		tnode->name->kernel_name && !tnode->tp) {
		/* No tracepoint structure found so no ID possible */
		return -EIO;
	}
#endif
#endif
	if (tnode->type == SW_TRACE_COLLECTOR_NOTIFIER)
		head = (void *)&s_notifier_list;

	SW_LIST_FOR_EACH_ENTRY(lnode, head, list) {
		struct sw_trace_notifier_data *data = lnode->data;

		if (!strcmp(
			data->name->abstract_name, tnode->name->abstract_name))
			return lnode->id;
	}
	return -1;
}
/*
 * Retrieve the "kernel" name for this tracepoint/notifier.
 */
const char *sw_get_trace_notifier_kernel_name(
		struct sw_trace_notifier_data *node)
{
	return node->name->kernel_name;
};
/*
 * Retrieve the "abstract" name for this tracepoint/notifier.
 */
const char *sw_get_trace_notifier_abstract_name(
			struct sw_trace_notifier_data *node)
{
	return node->name->abstract_name;
};

/*
 * Add a single TRACE/NOTIFY provider.
 */
int sw_register_trace_notify_provider(struct sw_trace_notifier_data *data)
{
	struct sw_trace_list_node *lnode = NULL;

	if (!data) {
		pw_pr_error(
			"ERROR: cannot add NULL trace/notifier provider!\n");
		return -EIO;
	}
	if (!(data->type == SW_TRACE_COLLECTOR_TRACEPOINT ||
			data->type == SW_TRACE_COLLECTOR_NOTIFIER)) {
		pw_pr_error(
			"ERROR: cannot add invalid trace/notifier data!\n");
		return -EIO;
	}
	/*
	 * Kernel name is allowed to be NULL, but abstract name
	 * MUST be present!
	 */
	if (!data->name || !data->name->abstract_name) {
		pw_pr_error(
			"ERROR: cannot add trace/notifier provider without an abstract name!\n");
		pw_pr_error("ERROR: data->name = %p\n", data->name);
		return -EIO;
	}
	lnode = sw_kmalloc(sizeof(*lnode), GFP_KERNEL);
	if (!lnode) {
		pw_pr_error(
			"ERROR: couldn't allocate a list node when adding a trace/notifier provider!\n");
		return -ENOMEM;
	}
	lnode->data = data;
	SW_LIST_ENTRY_INIT(lnode, list);
	if (data->type == SW_TRACE_COLLECTOR_TRACEPOINT) {
		lnode->id = ++s_trace_idx;
		SW_LIST_ADD(&s_trace_list, lnode, list);
	} else {
		lnode->id = ++s_notifier_idx;
		SW_LIST_ADD(&s_notifier_list, lnode, list);
	}
	return PW_SUCCESS;
}
/*
 * Add all TRACE/NOTIFY providers.
 */
int sw_add_trace_notify(void)
{
	return sw_add_trace_notifier_providers();
}

static void sw_free_trace_notifier_list_i(void *list_head)
{
	SW_LIST_HEAD_VAR(sw_trace_list_node) * head = list_head;
	while (!SW_LIST_EMPTY(head)) {
		struct sw_trace_list_node *lnode =
			SW_LIST_GET_HEAD_ENTRY(head, sw_trace_list_node, list);

		SW_LIST_UNLINK(lnode, list);
		sw_kfree(lnode);
	}
}
/*
 * Remove TRACE/NOTIFY providers.
 */
void sw_remove_trace_notify(void)
{
	/*
	 * Free all nodes.
	 */
	sw_free_trace_notifier_list_i(&s_trace_list);
	sw_free_trace_notifier_list_i(&s_notifier_list);
	/*
	 * Call our providers to deallocate resources.
	 */
	sw_remove_trace_notifier_providers();
	/*
	 * Clear out the topology list
	 */
	sw_clear_topology_list();
}

#define REG_FLAG (void *)1
#define UNREG_FLAG (void *)2
static int sw_reg_unreg_node_i(struct sw_trace_notifier_data *node,
				void *is_reg)
{
	if (is_reg == REG_FLAG) {
		/*
		 * Do we have anything to collect?
		 * Update: or were we asked to always register?
		 */
		if (SW_LIST_EMPTY(&node->list) && !node->always_register)
			return PW_SUCCESS;

		/*
		 * Sanity: ensure we have a register AND an unregister function
		 * before proceeding!
		 */
		if (node->probe_register == NULL ||
				node->probe_unregister == NULL) {
			pw_pr_debug(
				"WARNING: invalid trace/notifier register/unregister function for %s\n",
				 sw_get_trace_notifier_kernel_name(node));
			/*
			 * Don't flag this as an error -- some socwatch
			 * trace providers don't have a register/unregister
			 * function
			 */
			return PW_SUCCESS;
		}
		if ((*node->probe_register)(node))
			return -EIO;

		node->was_registered = true;
		return PW_SUCCESS;
	} else if (is_reg == UNREG_FLAG) {
		if (node->was_registered) {
			/*
			 * No need to check for validity of probe unregister
			 * function -- 'sw_register_notifiers_i()'
			 * would already have done so!
			 */
			WARN_ON((*node->probe_unregister)(node));
			node->was_registered = false;
			pw_pr_debug("OK, unregistered trace/notifier for %s\n",
				sw_get_trace_notifier_kernel_name(node));
		}
		return PW_SUCCESS;
	}
	pw_pr_error("ERROR: invalid reg/unreg flag value 0x%lx\n",
		(unsigned long)is_reg);
	return -EIO;
}
/*
 * Register all required tracepoints and notifiers.
 */
int sw_register_trace_notifiers(void)
{
	/*
	 * First, the tracepoints.
	 */
	if (sw_for_each_tracepoint_node(&sw_reg_unreg_node_i,
			REG_FLAG, true /* return on error */)) {
		pw_pr_error("ERROR registering some tracepoints\n");
		return -EIO;
	}
	/*
	 * And then the notifiers.
	 */
	if (sw_for_each_notifier_node(&sw_reg_unreg_node_i,
			REG_FLAG, true /* return on error */)) {
		pw_pr_error("ERROR registering some tracepoints\n");
		return -EIO;
	}
	return PW_SUCCESS;
};
/*
 * Unregister all previously registered tracepoints and notifiers.
 */
int sw_unregister_trace_notifiers(void)
{
	/*
	 * First, the notifiers.
	 */
	if (sw_for_each_notifier_node(&sw_reg_unreg_node_i, UNREG_FLAG,
			true /* return on error */)) {
		pw_pr_error("ERROR registering some tracepoints\n");
		return -EIO;
	}
	/*
	 * And then the tracepoints.
	 */
	if (sw_for_each_tracepoint_node(
			&sw_reg_unreg_node_i,
			UNREG_FLAG, true /* return on error */)) {
		pw_pr_error("ERROR registering some tracepoints\n");
		return -EIO;
	}
	return PW_SUCCESS;
};

void sw_clear_topology_list(void)
{
	SW_LIST_HEAD_VAR(sw_topology_node) * head = &sw_topology_list;
	while (!SW_LIST_EMPTY(head)) {
		struct sw_topology_node *lnode =
			SW_LIST_GET_HEAD_ENTRY(head, sw_topology_node, list);

		pw_pr_debug("Clearing topology node for cpu %d\n",
			lnode->change.cpu);
		SW_LIST_UNLINK(lnode, list);
		sw_kfree(lnode);
	}
	sw_num_topology_entries  = 0;
}
