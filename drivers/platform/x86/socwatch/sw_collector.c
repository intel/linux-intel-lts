/*

  This file is provided under a dual BSD/GPLv2 license.	 When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY

  Copyright(c) 2014 - 2018 Intel Corporation.

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
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
  notice, this list of condiions and the following disclaimer.
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
#include "sw_internal.h"
#include "sw_structs.h"
#include "sw_collector.h"
#include "sw_kernel_defines.h"
#include "sw_mem.h"
#include "sw_types.h"
#include "sw_hardware_io.h"
#include "sw_output_buffer.h"

/* -------------------------------------------------
 * Local function declarations.
 * -------------------------------------------------
 */
void sw_free_driver_interface_info_i(struct sw_driver_interface_info *info);
const struct sw_hw_ops **sw_alloc_ops_i(pw_u16_t num_io_descriptors);
void sw_free_ops_i(const struct sw_hw_ops **ops);
struct sw_driver_interface_info *
sw_copy_driver_interface_info_i(const struct sw_driver_interface_info *info);
int sw_init_driver_interface_info_i(struct sw_driver_interface_info *info);
int sw_reset_driver_interface_info_i(struct sw_driver_interface_info *info);
int sw_init_ops_i(const struct sw_hw_ops **ops,
		  const struct sw_driver_interface_info *info);
sw_driver_msg_t *
sw_alloc_collector_msg_i(const struct sw_driver_interface_info *info,
			 size_t per_msg_payload_size);
void sw_free_collector_msg_i(sw_driver_msg_t *msg);
size_t sw_get_payload_size_i(const struct sw_driver_interface_info *info);
void sw_handle_per_cpu_msg_i(void *info, enum sw_wakeup_action action);
/* -------------------------------------------------
 * Variables.
 * -------------------------------------------------
 */
const static struct sw_hw_ops *s_hw_ops;
/* -------------------------------------------------
 * Function definitions.
 * -------------------------------------------------
 */
/*
 * Driver interface info functions.
 */

/**
 * sw_add_driver_info() - Add a collector node to the list called at this
 *			"when type".
 * @head:   The collector node list to add the new node to.
 * @info:   Driver information to add to the list.
 *
 *  This function allocates and links in a "collector node" for each
 *  collector based on the collector info in the info parameter.
 *  The function allocates the new node, and links it to a local copy
 *  of the passed-in driver interface info.  If the collector has an
 *  init function among its operations, it iterates through the
 *  descriptors in info, passing each one to the init function.
 *
 *  Finally, it allocates and initializes the "collector message" which
 *  buffers a data sample that this collector gathers during the run.
 *
 * Returns:  -PW_ERROR on failure, PW_SUCCESS on success.
 */
int sw_add_driver_info(void *list_head,
		       const struct sw_driver_interface_info *info)
{
	SW_LIST_HEAD_VAR(sw_collector_data) * head = list_head;
	struct sw_collector_data *node = sw_alloc_collector_node();

	if (!node) {
		pw_pr_error("ERROR allocating collector node!\n");
		return -PW_ERROR;
	}

	node->info = sw_copy_driver_interface_info_i(info);
	if (!node->info) {
		pw_pr_error(
			"ERROR allocating or copying driver_interface_info!\n");
		sw_free_collector_node(node);
		return -PW_ERROR;
	}
	/*
	 * Initialize the collectors in the node's descriptors.
	 */
	if (sw_init_driver_interface_info_i(node->info)) {
		pw_pr_error(
			"ERROR initializing a driver_interface_info node!\n");
		sw_free_collector_node(node);
		return -PW_ERROR;
	}
	/*
	 * Allocate the ops array. We do this one time as an optimization
	 * (we could always just repeatedly call 'sw_get_hw_ops_for()'
	 * during the collection but we want to avoid that overhead)
	 */
	node->ops = sw_alloc_ops_i(info->num_io_descriptors);
	if (!node->ops || sw_init_ops_i(node->ops, info)) {
		pw_pr_error("ERROR initializing the ops array!\n");
		sw_free_collector_node(node);
		return -PW_ERROR;
	}
	/*
	 * Allocate and initialize the "collector message".
	 */
	node->per_msg_payload_size = sw_get_payload_size_i(info);
	pw_pr_debug("Debug: Per msg payload size = %u\n",
		    (unsigned int)node->per_msg_payload_size);
	node->msg = sw_alloc_collector_msg_i(info, node->per_msg_payload_size);
	if (!node->msg) {
		pw_pr_error("ERROR allocating space for a collector msg!\n");
		sw_free_collector_node(node);
		return -PW_ERROR;
	}
	pw_pr_debug("NODE = %p, NODE->MSG = %p\n", node, node->msg);
	cpumask_clear(&node->cpumask);
	{
		/*
		 * For now, use following protocol:
		 * cpu_mask == -2 ==> Collect on ALL CPUs
		 * cpu_mask == -1 ==> Collect on ANY CPU
		 * cpu_mask >= 0 ==> Collect on a specific CPU
		 */
		if (node->info->cpu_mask >= 0) {
			/*
			 * Collect data on 'node->info->cpu_mask'
			 */
			cpumask_set_cpu(node->info->cpu_mask, &node->cpumask);
			pw_pr_debug("OK: set CPU = %d\n", node->info->cpu_mask);
		} else if (node->info->cpu_mask == -1) {
			/*
			 * Collect data on ANY CPU. Leave empty as a flag
			 * to signify user wishes to collect data on 'ANY' cpu.
			 */
			pw_pr_debug("OK: set ANY CPU\n");
		} else {
			/*
			 * Collect data on ALL cpus.
			 */
			cpumask_copy(&node->cpumask, cpu_present_mask);
			pw_pr_debug("OK: set ALL CPUs\n");
		}
	}
	SW_LIST_ADD(head, node, list);
	return PW_SUCCESS;
}

const struct sw_hw_ops **sw_alloc_ops_i(pw_u16_t num_io_descriptors)
{
	size_t size = num_io_descriptors * sizeof(struct sw_hw_ops *);
	const struct sw_hw_ops **ops = sw_kmalloc(size, GFP_KERNEL);

	if (ops) {
		memset(ops, 0, size);
	}
	return ops;
}

void sw_free_driver_interface_info_i(struct sw_driver_interface_info *info)
{
	if (info) {
		sw_kfree(info);
	}
}

void sw_free_ops_i(const struct sw_hw_ops **ops)
{
	if (ops) {
		sw_kfree(ops);
	}
}

/**
 * sw_copy_driver_interface_info_i - Allocate and copy the passed-in "info".
 *
 * @info: Information about the metric and collection properties
 *
 * Returns: a pointer to the newly allocated sw_driver_interface_info,
 *	    which is a copy of the version passed in via the info pointer.
 */
struct sw_driver_interface_info *
sw_copy_driver_interface_info_i(const struct sw_driver_interface_info *info)
{
	size_t size;
	struct sw_driver_interface_info *node = NULL;

	if (!info) {
		pw_pr_error("ERROR: NULL sw_driver_interface_info in alloc!\n");
		return node;
	}

	size = SW_DRIVER_INTERFACE_INFO_HEADER_SIZE() +
	       (info->num_io_descriptors *
		sizeof(struct sw_driver_io_descriptor));
	node = (struct sw_driver_interface_info *)sw_kmalloc(size, GFP_KERNEL);
	if (!node) {
		pw_pr_error("ERROR allocating driver interface info!\n");
		return node;
	}
	memcpy((char *)node, (const char *)info, size);

	/*
	 * Do debug dump.
	 */
	pw_pr_debug("DRIVER info has plugin_ID = %d, metric_ID = %d, "
		    "msg_ID = %d\n",
		    node->plugin_id, node->metric_id, node->msg_id);

	return node;
}
int sw_init_driver_interface_info_i(struct sw_driver_interface_info *info)
{
	/*
	 * Do any initialization here.
	 * For now, only IPC/MMIO descriptors need to be initialized.
	 */
	int i = 0;
	struct sw_driver_io_descriptor *descriptor = NULL;

	if (!info) {
		pw_pr_error("ERROR: no info!\n");
		return -PW_ERROR;
	}
	for (i = 0,
	    descriptor = (struct sw_driver_io_descriptor *)info->descriptors;
	     i < info->num_io_descriptors; ++i, ++descriptor) {
		if (sw_init_driver_io_descriptor(descriptor)) {
			return -PW_ERROR;
		}
	}
	return PW_SUCCESS;
}

int sw_reset_driver_interface_info_i(struct sw_driver_interface_info *info)
{
	/*
	 * Do any finalization here.
	 * For now, only IPC/MMIO descriptors need to be finalized.
	 */
	int i = 0;
	struct sw_driver_io_descriptor *descriptor = NULL;

	if (!info) {
		pw_pr_error("ERROR: no info!\n");
		return -PW_ERROR;
	}
	for (i = 0,
	    descriptor = (struct sw_driver_io_descriptor *)info->descriptors;
	     i < info->num_io_descriptors; ++i, ++descriptor) {
		if (sw_reset_driver_io_descriptor(descriptor)) {
			return -PW_ERROR;
		}
	}
	return PW_SUCCESS;
}
int sw_init_ops_i(const struct sw_hw_ops **ops,
		  const struct sw_driver_interface_info *info)
{
	int i = 0;
	struct sw_driver_io_descriptor *descriptor = NULL;

	if (!ops || !info) {
		return -PW_ERROR;
	}
	for (i = 0,
	    descriptor = (struct sw_driver_io_descriptor *)info->descriptors;
	     i < info->num_io_descriptors; ++i, ++descriptor) {
		ops[i] = sw_get_hw_ops_for(descriptor->collection_type);
		if (ops[i] == NULL) {
			return -PW_ERROR;
		}
	}
	return PW_SUCCESS;
}

/*
 * If this descriptor's collector has an init function, call it passing in
 * this descriptor.  That allows the collector to perform any initialization
 * or registration specific to this metric.
 */
int sw_init_driver_io_descriptor(struct sw_driver_io_descriptor *descriptor)
{
	sw_io_desc_init_func_t init_func = NULL;
	const struct sw_hw_ops *ops =
		sw_get_hw_ops_for(descriptor->collection_type);
	if (ops == NULL) {
		pw_pr_error("NULL ops found in init_driver_io_desc: type %d\n",
			    descriptor->collection_type);
		return -PW_ERROR;
	}
	init_func = ops->init;

	if (init_func) {
		int retval = (*init_func)(descriptor);

		if (retval) {
			pw_pr_error("(*init) return value for type %d: %d\n",
				    descriptor->collection_type, retval);
		}
		return retval;
	}
	return PW_SUCCESS;
}

/*
 * If this descriptor's collector has a finalize function, call it passing in
 * this descriptor. This allows the collector to perform any finalization
 * specific to this metric.
 */
int sw_reset_driver_io_descriptor(struct sw_driver_io_descriptor *descriptor)
{
	sw_io_desc_reset_func_t reset_func = NULL;
	const struct sw_hw_ops *ops =
		sw_get_hw_ops_for(descriptor->collection_type);
	if (ops == NULL) {
		pw_pr_error("NULL ops found in reset_driver_io_desc: type %d\n",
			    descriptor->collection_type);
		return -PW_ERROR;
	}
	pw_pr_debug("calling reset on descriptor of type %d\n",
		    descriptor->collection_type);
	reset_func = ops->reset;

	if (reset_func) {
		int retval = (*reset_func)(descriptor);

		if (retval) {
			pw_pr_error("(*reset) return value for type %d: %d\n",
				    descriptor->collection_type, retval);
		}
		return retval;
	}
	return PW_SUCCESS;
}

int sw_handle_driver_io_descriptor(
	char *dst_vals, int cpu,
	const struct sw_driver_io_descriptor *descriptor,
	const struct sw_hw_ops *hw_ops)
{
	typedef void (*sw_hardware_io_func_t)(
		char *, int, const struct sw_driver_io_descriptor *, u16);
	sw_hardware_io_func_t hardware_io_func = NULL;

	if (descriptor->collection_command < SW_IO_CMD_READ ||
	    descriptor->collection_command > SW_IO_CMD_WRITE) {
		return -PW_ERROR;
	}
	switch (descriptor->collection_command) {
	case SW_IO_CMD_READ:
		hardware_io_func = hw_ops->read;
		break;
	case SW_IO_CMD_WRITE:
		hardware_io_func = hw_ops->write;
		break;
	default:
		break;
	}
	if (hardware_io_func) {
		(*hardware_io_func)(dst_vals, cpu, descriptor,
				    descriptor->counter_size_in_bytes);
	} else {
		pw_pr_debug(
			"NO ops to satisfy %u operation for collection type %u!\n",
			descriptor->collection_command,
			descriptor->collection_type);
	}
	return PW_SUCCESS;
}

sw_driver_msg_t *
sw_alloc_collector_msg_i(const struct sw_driver_interface_info *info,
			 size_t per_msg_payload_size)
{
	size_t per_msg_size = 0, total_size = 0;
	sw_driver_msg_t *msg = NULL;

	if (!info) {
		return NULL;
	}
	per_msg_size = sizeof(struct sw_driver_msg) + per_msg_payload_size;
	total_size = per_msg_size * num_possible_cpus();
	msg = (sw_driver_msg_t *)sw_kmalloc(total_size, GFP_KERNEL);
	if (msg) {
		int cpu = -1;

		memset(msg, 0, total_size);
		for_each_possible_cpu(cpu) {
			sw_driver_msg_t *__msg = GET_MSG_SLOT_FOR_CPU(
				msg, cpu, per_msg_payload_size);
			char *__payload =
				(char *)__msg + sizeof(struct sw_driver_msg);
			__msg->cpuidx = (pw_u16_t)cpu;
			__msg->plugin_id = (pw_u8_t)info->plugin_id;
			__msg->metric_id = (pw_u8_t)info->metric_id;
			__msg->msg_id = (pw_u8_t)info->msg_id;
			__msg->payload_len = per_msg_payload_size;
			__msg->p_payload = __payload;
			pw_pr_debug(
				"[%d]: per_msg_payload_size = %zx, msg = %p, payload = %p\n",
				cpu, per_msg_payload_size, __msg, __payload);
		}
	}
	return msg;
}

void sw_free_collector_msg_i(sw_driver_msg_t *msg)
{
	if (msg) {
		sw_kfree(msg);
	}
}

size_t sw_get_payload_size_i(const struct sw_driver_interface_info *info)
{
	size_t size = 0;
	int i = 0;

	if (info) {
		for (i = 0; i < info->num_io_descriptors;
		     size +=
		     ((struct sw_driver_io_descriptor *)info->descriptors)[i]
			     .counter_size_in_bytes,
		    ++i)
			;
	}
	return size;
}

void sw_handle_per_cpu_msg_i(void *info, enum sw_wakeup_action action)
{
	/*
	 * Basic algo:
	 * For each descriptor in 'node->info->descriptors'; do:
	 * 1. Perform H/W read; use 'descriptor->collection_type'
	 * to determine type of read; use 'descriptor->counter_size_in_bytes'
	 * for read size. Use msg->p_payload[dst_idx] as dst address
	 * 2. Increment dst idx by 'descriptor->counter_size_in_bytes'
	 */
	struct sw_collector_data *node = (struct sw_collector_data *)info;
	int cpu = RAW_CPU();
	u16 num_descriptors = node->info->num_io_descriptors, i = 0;
	struct sw_driver_io_descriptor *descriptors =
		(struct sw_driver_io_descriptor *)node->info->descriptors;
	sw_driver_msg_t *msg = GET_MSG_SLOT_FOR_CPU(node->msg, cpu,
						    node->per_msg_payload_size);
	char *dst_vals = msg->p_payload;
	const struct sw_hw_ops **ops = node->ops;
	bool wasAnyWrite = false;

	/* msg TSC assigned when msg is written to buffer */
	msg->cpuidx = cpu;

	for (i = 0; i < num_descriptors; ++i,
	    dst_vals += descriptors->counter_size_in_bytes, ++descriptors) {
		if (unlikely(ops[i] == NULL)) {
			pw_pr_debug("NULL OPS!\n");
			continue;
		}
		if (descriptors->collection_command == SW_IO_CMD_WRITE) {
			wasAnyWrite = true;
		}
		if (sw_handle_driver_io_descriptor(dst_vals, cpu, descriptors,
						   ops[i])) {
			pw_pr_error("ERROR reading descriptor with type %d\n",
				    descriptors->collection_type);
		}
	}

	/*
	 * We produce messages only on READs. Note that SWA prohibits
	 * messages that contain both READ and WRITE descriptors, so it
	 * is enough to check if there was ANY WRITE descriptor in this
	 * message.
	 */
	if (likely(wasAnyWrite == false)) {
		if (sw_produce_generic_msg(msg, action)) {
			pw_pr_warn("WARNING: could NOT produce message!\n");
		}
	}

	return;
}

/*
 * Collector list and node functions.
 */
struct sw_collector_data *sw_alloc_collector_node(void)
{
	struct sw_collector_data *node = (struct sw_collector_data *)sw_kmalloc(
		sizeof(struct sw_collector_data), GFP_KERNEL);
	if (node) {
		node->per_msg_payload_size = 0x0;
		node->last_update_jiffies = 0x0;
		node->info = NULL;
		node->ops = NULL;
		node->msg = NULL;
		SW_LIST_ENTRY_INIT(node, list);
	}
	return node;
}

void sw_free_collector_node(struct sw_collector_data *node)
{
	if (!node) {
		return;
	}
	if (node->info) {
		sw_reset_driver_interface_info_i(node->info);
		sw_free_driver_interface_info_i(node->info);
		node->info = NULL;
	}
	if (node->ops) {
		sw_free_ops_i(node->ops);
		node->ops = NULL;
	}
	if (node->msg) {
		sw_free_collector_msg_i(node->msg);
		node->msg = NULL;
	}
	sw_kfree(node);
	return;
}

int sw_handle_collector_node(struct sw_collector_data *node)
{
	if (!node || !node->info || !node->ops || !node->msg) {
		return -PW_ERROR;
	}
	pw_pr_debug("Calling SMP_CALL_FUNCTION_MANY!\n");
	sw_schedule_work(&node->cpumask, &sw_handle_per_cpu_msg, node);
	return PW_SUCCESS;
}

int sw_handle_collector_node_on_cpu(struct sw_collector_data *node, int cpu)
{
	if (!node || !node->info || !node->ops || !node->msg) {
		return -PW_ERROR;
	}
	/*
	 * Check if this node indicates it should be scheduled
	 * on the given cpu. If so, clear all other CPUs from the
	 * mask and schedule the node.
	 */
	if (cpumask_test_cpu(cpu, &node->cpumask)) {
		struct cpumask tmp_mask;

		cpumask_clear(&tmp_mask);
		cpumask_set_cpu(cpu, &tmp_mask);
		pw_pr_debug("Calling SMP_CALL_FUNCTION_MANY!\n");
		sw_schedule_work(&tmp_mask, &sw_handle_per_cpu_msg, node);
	}
	return PW_SUCCESS;
}

void sw_init_collector_list(void *list_head)
{
	SW_LIST_HEAD_VAR(sw_collector_data) * head = list_head;
	SW_LIST_HEAD_INIT(head);
}

void sw_destroy_collector_list(void *list_head)
{
	SW_LIST_HEAD_VAR(sw_collector_data) * head = list_head;
	while (!SW_LIST_EMPTY(head)) {
		struct sw_collector_data *curr =
			SW_LIST_GET_HEAD_ENTRY(head, sw_collector_data, list);
		BUG_ON(!curr->info);
		SW_LIST_UNLINK(curr, list);
		sw_free_collector_node(curr);
	}
}

/**
 * sw_handle_collector_list - Iterate through the collector list, calling
 *			      func() upon each element.
 * @list_head:	The collector list head.
 * @func:  The function to call for each collector.
 *
 * This function is called when one of the "when types" fires, since the
 * passed-in collector node list is the list of collections to do at that time.
 *
 * Returns: PW_SUCCESS on success, -PW_ERROR on error.
 */
int sw_handle_collector_list(void *list_head,
			     int (*func)(struct sw_collector_data *data))
{
	SW_LIST_HEAD_VAR(sw_collector_data) * head = list_head;
	int retVal = PW_SUCCESS;
	struct sw_collector_data *curr = NULL;

	if (!head || !func) {
		return -PW_ERROR;
	}
	SW_LIST_FOR_EACH_ENTRY(curr, head, list)
	{
		pw_pr_debug("HANDLING\n");
		if ((*func)(curr)) {
			retVal = -PW_ERROR;
		}
	}
	return retVal;
}

int sw_handle_collector_list_on_cpu(void *list_head,
				    int (*func)(struct sw_collector_data *data,
						int cpu),
				    int cpu)
{
	SW_LIST_HEAD_VAR(sw_collector_data) * head = list_head;
	int retVal = PW_SUCCESS;
	struct sw_collector_data *curr = NULL;
	if (!head || !func) {
		return -PW_ERROR;
	}
	SW_LIST_FOR_EACH_ENTRY(curr, head, list)
	{
		pw_pr_debug("HANDLING\n");
		if ((*func)(curr, cpu)) {
			retVal = -PW_ERROR;
		}
	}
	return retVal;
}

void sw_handle_per_cpu_msg(void *info)
{
	sw_handle_per_cpu_msg_i(info, SW_WAKEUP_ACTION_DIRECT);
}

void sw_handle_per_cpu_msg_no_sched(void *info)
{
	sw_handle_per_cpu_msg_i(info, SW_WAKEUP_ACTION_TIMER);
}

void sw_handle_per_cpu_msg_on_cpu(int cpu, void *info)
{
	if (unlikely(cpu == RAW_CPU())) {
		sw_handle_per_cpu_msg_no_sched(info);
	} else {
		pw_pr_debug("[%d] is handling for %d\n", RAW_CPU(), cpu);
		/*
		 * No need to disable preemption -- 'smp_call_function_single'
		 * does that for us.
		 */
		smp_call_function_single(
			cpu, &sw_handle_per_cpu_msg_no_sched, info,
			false /* false ==> do NOT wait for function
			       * completion
			       */);
	}
}

void sw_set_collector_ops(const struct sw_hw_ops *hw_ops)
{
	s_hw_ops = hw_ops;
}
