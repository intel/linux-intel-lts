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
#define MOD_AUTHOR "Gautam Upadhyaya <gautam.upadhyaya@intel.com>"
#define MOD_DESC "SoC Watch kernel module"

#include "sw_internal.h"
#include "sw_structs.h"
#include "sw_kernel_defines.h"
#include "sw_types.h"
#include "sw_mem.h"
#include "sw_ioctl.h"
#include "sw_output_buffer.h"
#include "sw_hardware_io.h"
#include "sw_overhead_measurements.h"
#include "sw_tracepoint_handlers.h"
#include "sw_collector.h"
#include "sw_file_ops.h"

/* -------------------------------------------------
 * Compile time constants.
 * -------------------------------------------------
 */
/*
 * Number of entries in the 'sw_collector_lists' array
 */
#define NUM_COLLECTOR_MODES (SW_WHEN_TYPE_END - SW_WHEN_TYPE_BEGIN + 1)
#define PW_OUTPUT_BUFFER_SIZE                                                  \
	256 /* Number of output messages in each per-cpu buffer */
/*
 * Check if tracepoint/notifier ID is in (user-supplied) mask
 */
#define IS_TRACE_NOTIFIER_ID_IN_MASK(id, mask)                                 \
	((id) >= 0 && (((mask) >> (id)) & 0x1))

/* -------------------------------------------------
 *  Local function declarations.
 * -------------------------------------------------
 */
int sw_load_driver_i(void);
void sw_unload_driver_i(void);
int sw_init_collector_lists_i(void);
void sw_destroy_collector_lists_i(void);
int sw_init_data_structures_i(void);
void sw_destroy_data_structures_i(void);
int sw_get_arch_details_i(void);
void sw_iterate_driver_info_lists_i(void);
void sw_handle_immediate_request_i(void *request);
int sw_print_collector_node_i(struct sw_collector_data *data);
int sw_collection_start_i(void);
int sw_collection_stop_i(void);
int sw_collection_poll_i(void);
size_t sw_get_payload_size_i(const struct sw_driver_interface_info *info);
sw_driver_msg_t *
sw_alloc_collector_msg_i(const struct sw_driver_interface_info *info,
			 size_t per_msg_payload_size);
static long sw_unlocked_handle_ioctl_i(unsigned int ioctl_num,
				       void *p_local_args);
static long
sw_set_driver_infos_i(struct sw_driver_interface_msg __user *remote_msg,
		      int local_len);
static long sw_handle_cmd_i(sw_driver_collection_cmd_t cmd,
			    u64 __user *remote_out_args);
static void sw_do_extract_scu_fw_version(void);
static long
sw_get_available_name_id_mappings_i(enum sw_name_id_type type,
				    struct sw_name_info_msg __user *remote_info,
				    size_t local_len);
static enum sw_driver_collection_cmd sw_get_collection_cmd_i(void);
static bool sw_should_flush_buffer_i(void);

/* -------------------------------------------------
 * Data structures.
 * -------------------------------------------------
 */
/*
 * Structure to hold current CMD state
 * of the device driver. Constantly evolving, but
 * that's OK -- this is internal to the driver
 * and is NOT exported.
 */
struct swa_internal_state {
	sw_driver_collection_cmd_t
		cmd; // indicates which command was specified
		     // last e.g. START, STOP etc.
	/*
	 * Should we write to our per-cpu output buffers?
	 * YES if we're actively collecting.
	 * NO if we're not.
	 */
	bool write_to_buffers;
	/*
	 * Should we "drain/flush" the per-cpu output buffers?
	 * (See "device_read" for an explanation)
	 */
	bool drain_buffers;
	// Others...
};

/* -------------------------------------------------
 * Variables.
 * -------------------------------------------------
 */
static bool do_force_module_scope_for_cpu_frequencies;
module_param(do_force_module_scope_for_cpu_frequencies, bool, S_IRUSR);
MODULE_PARM_DESC(
	do_force_module_scope_for_cpu_frequencies,
	"Toggle module scope for cpu frequencies. Sets \"affected_cpus\" and \"related_cpus\" of cpufreq_policy.");

static unsigned short sw_buffer_num_pages = 16;
module_param(sw_buffer_num_pages, ushort, S_IRUSR);
MODULE_PARM_DESC(
	sw_buffer_num_pages,
	"Specify number of 4kB pages to use for each per-cpu buffer. MUST be a power of 2! Default value = 16 (64 kB)");

/* TODO: convert from 'list_head' to 'hlist_head' */
/*
 * sw_collector_lists is an array of linked lists of "collector nodes"
 * (sw_collector_data structs).  It is indexed by the sw_when_type_t's.
 * Each list holds the collectors to "execute" at a specific time,
 * e.g. the beginning of the run, at a poll interval, tracepoint, etc.
 */
static SW_DEFINE_LIST_HEAD(sw_collector_lists,
			   sw_collector_data)[NUM_COLLECTOR_MODES];
static __read_mostly u16 sw_scu_fw_major_minor;

static struct swa_internal_state s_internal_state;
static struct sw_file_ops s_ops = {
	.ioctl_handler = &sw_unlocked_handle_ioctl_i,
	.stop_handler = &sw_collection_stop_i,
	.get_current_cmd = &sw_get_collection_cmd_i,
	.should_flush = &sw_should_flush_buffer_i,
};

/*
 * For each function that you want to profile,
 * do the following (e.g. function 'foo'):
 * **************************************************
 * DECLARE_OVERHEAD_VARS(foo);
 * **************************************************
 * This will declare the two variables required
 * to keep track of overheads incurred in
 * calling/servicing 'foo'. Note that the name
 * that you declare here *MUST* match the function name!
 */

DECLARE_OVERHEAD_VARS(sw_collection_poll_i); // for POLL
DECLARE_OVERHEAD_VARS(sw_any_seg_full);

/*
 * String representation of the various 'SW_WHEN_TYPE_XYZ' enum values.
 * Debugging ONLY!
 */
#if DO_DEBUG_OUTPUT
static const char *s_when_type_names[] = { "BEGIN", "POLL", "NOTIFIER",
					   "TRACEPOINT", "END" };
#endif // DO_DEBUG_OUTPUT

/* -------------------------------------------------
 * Function definitions.
 * -------------------------------------------------
 */
/*
 * External functions.
 */
int sw_process_snapshot(enum sw_when_type when)
{
	if (when > SW_WHEN_TYPE_END) {
		pw_pr_error("invalid snapshot time %d specified!\n", when);
		return -EINVAL;
	}
	if (sw_handle_collector_list(&sw_collector_lists[when],
				     &sw_handle_collector_node)) {
		pw_pr_error("ERROR: could NOT handle snapshot for time %d!\n",
			    when);
		return -EIO;
	}
	return 0;
}

int sw_process_snapshot_on_cpu(enum sw_when_type when, int cpu)
{
	if (when > SW_WHEN_TYPE_END) {
		pw_pr_error("invalid snapshot time %d specified!\n", when);
		return -EINVAL;
	}
	if (sw_handle_collector_list_on_cpu(&sw_collector_lists[when],
					    &sw_handle_collector_node_on_cpu,
					    cpu)) {
		pw_pr_error("ERROR: could NOT handle snapshot for time %d!\n",
			    when);
		return -EIO;
	}
	return 0;
}

/*
 * Driver interface info and collector list functions.
 */
int sw_print_collector_node_i(struct sw_collector_data *curr)
{
	pw_u16_t num_descriptors = 0;
	sw_io_desc_print_func_t print_func = NULL;
	struct sw_driver_io_descriptor *descriptor = NULL;
	struct sw_driver_interface_info *info = NULL;

	if (!curr) {
		return -PW_ERROR;
	}
	info = curr->info;
	descriptor = (struct sw_driver_io_descriptor *)info->descriptors;
	pw_pr_debug(
		"cpu-mask = %d, Plugin-ID = %d, Metric-ID = %d, MSG-ID = %d\n",
		info->cpu_mask, info->plugin_id, info->metric_id, info->msg_id);
	for (num_descriptors = info->num_io_descriptors; num_descriptors > 0;
	     --num_descriptors, ++descriptor) {
		const struct sw_hw_ops *ops =
			sw_get_hw_ops_for(descriptor->collection_type);
		if (ops == NULL) {
			return -PW_ERROR;
		}
		print_func = ops->print;
		if (print_func && (*print_func)(descriptor)) {
			return -PW_ERROR;
		}
	}
	return PW_SUCCESS;
}

/*
 * Driver interface info and collector list functions.
 */

/**
 * sw_reset_collector_node_i - Call the reset op on all of the descriptors
 *                             in coll that have one.
 * @coll: The data structure containing an array of collector descriptors.
 *
 * Return: PW_SUCCESS if all of the resets succeeded, -PW_ERROR if any failed.
 */
static int sw_reset_collector_node_i(struct sw_collector_data *coll)
{
	struct sw_driver_io_descriptor *descriptor = NULL;
	struct sw_driver_interface_info *info = NULL;
	int num_descriptors;
	int retcode = PW_SUCCESS;

	if (!coll) {
		return -PW_ERROR;
	}
	info = coll->info;

	descriptor = (struct sw_driver_io_descriptor *)info->descriptors;
	pw_pr_debug(
		"cpu-mask = %d, Plugin-ID = %d, Metric-ID = %d, MSG-ID = %d\n",
		info->cpu_mask, info->plugin_id, info->metric_id, info->msg_id);
	for (num_descriptors = info->num_io_descriptors; num_descriptors > 0;
	     --num_descriptors, ++descriptor) {
		const struct sw_hw_ops *ops =
			sw_get_hw_ops_for(descriptor->collection_type);
		if (ops && ops->reset && (*ops->reset)(descriptor)) {
			retcode = -PW_ERROR;
		}
	}
	return retcode;
}

static int sw_iterate_trace_notifier_list_i(struct sw_trace_notifier_data *node,
				     void *dummy)
{
	return sw_handle_collector_list(&node->list,
					&sw_print_collector_node_i);
}

void sw_iterate_driver_info_lists_i(void)
{
	sw_when_type_t which;

	for (which = SW_WHEN_TYPE_BEGIN; which <= SW_WHEN_TYPE_END; ++which) {
		pw_pr_debug("ITERATING list %s\n", s_when_type_names[which]);
		if (sw_handle_collector_list(
			    &sw_collector_lists[which],
			    &sw_print_collector_node_i)) { // Should NEVER happen!
			pw_pr_error(
				"WARNING: error occurred while printing values!\n");
		}
	}

	if (sw_for_each_tracepoint_node(&sw_iterate_trace_notifier_list_i, NULL,
					false /*return-on-error*/)) {
		pw_pr_error(
			"WARNING: error occurred while printing tracepoint values!\n");
	}
	if (sw_for_each_notifier_node(&sw_iterate_trace_notifier_list_i, NULL,
				      false /*return-on-error*/)) {
		pw_pr_error(
			"WARNING: error occurred while printing notifier values!\n");
	}
}

static void sw_reset_collectors_i(void)
{
	sw_when_type_t which;

	for (which = SW_WHEN_TYPE_BEGIN; which <= SW_WHEN_TYPE_END; ++which) {
		pw_pr_debug("ITERATING list %s\n", s_when_type_names[which]);
		if (sw_handle_collector_list(&sw_collector_lists[which],
					     &sw_reset_collector_node_i)) {
			pw_pr_error(
				"WARNING: error occurred while resetting a collector!\n");
		}
	}
}

int sw_init_data_structures_i(void)
{
	/*
	 * Find the # CPUs in this system.
	 * Update: use 'num_possible' instead of 'num_present' in case
	 * the cpus aren't numbered contiguously
	 */
	sw_max_num_cpus = num_possible_cpus();

	/*
	 * Initialize our trace subsys: MUST be called
	 * BEFORE 'sw_init_collector_lists_i()!
	 */
	if (sw_add_trace_notify()) {
		sw_destroy_data_structures_i();
		return -PW_ERROR;
	}
	if (sw_init_collector_lists_i()) {
		sw_destroy_data_structures_i();
		return -PW_ERROR;
	}
	if (sw_init_per_cpu_buffers()) {
		sw_destroy_data_structures_i();
		return -PW_ERROR;
	}
	if (sw_register_hw_ops()) {
		sw_destroy_data_structures_i();
		return -PW_ERROR;
	}
	return PW_SUCCESS;
}

void sw_destroy_data_structures_i(void)
{
	sw_free_hw_ops();
	sw_destroy_per_cpu_buffers();
	sw_destroy_collector_lists_i();
	sw_remove_trace_notify();
}

int sw_get_arch_details_i(void)
{
	/*
	 * SCU F/W version (if applicable)
	 */
	sw_do_extract_scu_fw_version();
	return PW_SUCCESS;
}

#define INIT_FLAG ((void *)0)
#define DESTROY_FLAG ((void *)1)

static int
sw_init_destroy_trace_notifier_lists_i(struct sw_trace_notifier_data *node,
				       void *is_init)
{
	if (is_init == INIT_FLAG) {
		sw_init_collector_list(&node->list);
	} else {
		sw_destroy_collector_list(&node->list);
	}
	node->was_registered = false;

	return PW_SUCCESS;
}

int sw_init_collector_lists_i(void)
{
	int i = 0;

	for (i = 0; i < NUM_COLLECTOR_MODES; ++i) {
		sw_init_collector_list(&sw_collector_lists[i]);
	}
	sw_for_each_tracepoint_node(&sw_init_destroy_trace_notifier_lists_i,
				    INIT_FLAG, false /*return-on-error*/);
	sw_for_each_notifier_node(&sw_init_destroy_trace_notifier_lists_i,
				  INIT_FLAG, false /*return-on-error*/);

	return PW_SUCCESS;
}

void sw_destroy_collector_lists_i(void)
{
	int i = 0;

	for (i = 0; i < NUM_COLLECTOR_MODES; ++i) {
		sw_destroy_collector_list(&sw_collector_lists[i]);
	}
	sw_for_each_tracepoint_node(&sw_init_destroy_trace_notifier_lists_i,
				    DESTROY_FLAG, false /*return-on-error*/);
	sw_for_each_notifier_node(&sw_init_destroy_trace_notifier_lists_i,
				  DESTROY_FLAG, false /*return-on-error*/);
}

/*
 * Used for {READ,WRITE}_IMMEDIATE requests.
 */
typedef struct sw_immediate_request_info sw_immediate_request_info_t;
struct sw_immediate_request_info {
	struct sw_driver_io_descriptor *local_descriptor;
	char *dst_vals;
	int *retVal;
};
void sw_handle_immediate_request_i(void *request)
{
	struct sw_immediate_request_info *info =
		(struct sw_immediate_request_info *)request;
	struct sw_driver_io_descriptor *descriptor = info->local_descriptor;
	char *dst_vals = info->dst_vals;
	const struct sw_hw_ops *ops =
		sw_get_hw_ops_for(descriptor->collection_type);
	if (likely(ops != NULL)) {
		*(info->retVal) = sw_handle_driver_io_descriptor(
			dst_vals, RAW_CPU(), descriptor, ops);
	} else {
		pw_pr_error(
			"No operations found to satisfy collection type %u!\n",
			descriptor->collection_type);
	}
	return;
}

static int num_times_polled;

int sw_collection_start_i(void)
{
	/*
	 * Reset the poll tick counter.
	 */
	num_times_polled = 0;
	/*
	 * Update the output buffers.
	 */
	sw_reset_per_cpu_buffers();
	/*
	 * Ensure clients don't think we're in 'flush' mode.
	 */
	s_internal_state.drain_buffers = false;
	/*
	 * Set the 'command'
	 */
	s_internal_state.cmd = SW_DRIVER_START_COLLECTION;
	/*
	 * Clear out the topology list
	 */
	sw_clear_topology_list();
	/*
	 * Handle 'START' snapshots, if any.
	 */
	{
		if (sw_handle_collector_list(
			    &sw_collector_lists[SW_WHEN_TYPE_BEGIN],
			    &sw_handle_collector_node)) {
			pw_pr_error(
				"ERROR: could NOT handle START collector list!\n");
			return -PW_ERROR;
		}
	}
	/*
	 * Register any required tracepoints and notifiers.
	 */
	{
		if (sw_register_trace_notifiers()) {
			pw_pr_error("ERROR registering trace_notifiers!\n");
			sw_unregister_trace_notifiers();
			return -PW_ERROR;
		}
	}
	pw_pr_debug("OK, STARTED collection!\n");
	return PW_SUCCESS;
}

int sw_collection_stop_i(void)
{
	/*
	 * Unregister any registered tracepoints and notifiers.
	 */
	if (sw_unregister_trace_notifiers()) {
		pw_pr_warn(
			"Warning: some trace_notifier probe functions could NOT be unregistered!\n");
	}
	/*
	 * Handle 'STOP' snapshots, if any.
	 */
	if (sw_handle_collector_list(&sw_collector_lists[SW_WHEN_TYPE_END],
				     &sw_handle_collector_node)) {
		pw_pr_error("ERROR: could NOT handle STOP collector list!\n");
		return -PW_ERROR;
	}
	/*
	 * Set the 'command'
	 */
	s_internal_state.cmd = SW_DRIVER_STOP_COLLECTION;
	/*
	 * Tell consumers to 'flush' all buffers. We need to
	 * defer this as long as possible because it needs to be
	 * close to the 'wake_up_interruptible', below.
	 */
	s_internal_state.drain_buffers = true;
	smp_mb();
	/*
	 * Wakeup any sleeping readers, and cleanup any
	 * timers in the reader subsys.
	 */
	sw_cancel_reader();
	/*
	 * Collect stats on samples produced and dropped.
	 * TODO: call from 'device_read()' instead?
	 */
	sw_count_samples_produced_dropped();
#if DO_OVERHEAD_MEASUREMENTS
	pw_pr_force(
		"DEBUG: there were %llu samples produced and %llu samples dropped in buffer v5!\n",
		sw_num_samples_produced, sw_num_samples_dropped);
#endif // DO_OVERHEAD_MEASUREMENTS
	/*
	 * DEBUG: iterate over collection lists.
	 */
	sw_iterate_driver_info_lists_i();
	/*
	 * Shut down any collectors that need shutting down.
	 */
	sw_reset_collectors_i();
	/*
	 * Clear out the collector lists.
	 */
	sw_destroy_collector_lists_i();
	pw_pr_debug("OK, STOPPED collection!\n");
#if DO_OVERHEAD_MEASUREMENTS
	pw_pr_force("There were %d poll ticks!\n", num_times_polled);
#endif // DO_OVERHEAD_MEASUREMENTS
	return PW_SUCCESS;
}

int sw_collection_poll_i(void)
{
	/*
	 * Handle 'POLL' timer expirations.
	 */
	if (SW_LIST_EMPTY(&sw_collector_lists[SW_WHEN_TYPE_POLL])) {
		pw_pr_debug("DEBUG: EMPTY POLL LIST\n");
	}
	++num_times_polled;
	return sw_handle_collector_list(&sw_collector_lists[SW_WHEN_TYPE_POLL],
					&sw_handle_collector_node);
}

/*
 * Private data for the 'sw_add_trace_notifier_driver_info_i' function.
 */
struct tn_data {
	struct sw_driver_interface_info *info;
	u64 mask;
};

static int
sw_add_trace_notifier_driver_info_i(struct sw_trace_notifier_data *node,
				    void *priv)
{
	struct tn_data *data = (struct tn_data *)priv;
	struct sw_driver_interface_info *local_info = data->info;
	u64 mask = data->mask;
	int id = sw_get_trace_notifier_id(node);

	if (IS_TRACE_NOTIFIER_ID_IN_MASK(id, mask)) {
		pw_pr_debug("TRACEPOINT ID = %d is IN mask 0x%llx\n", id, mask);
		if (sw_add_driver_info(&node->list, local_info)) {
			pw_pr_error(
				"WARNING: could NOT add driver info to list!\n");
			return -PW_ERROR;
		}
	}
	return PW_SUCCESS;
}

static int sw_post_config_i(const struct sw_hw_ops *op, void *priv)
{
	if (!op->available || !(*op->available)()) {
		/* op not available */
		return 0;
	}
	if (!op->post_config || (*op->post_config)()) {
		return 0;
	}
	return -EIO;
}

/**
 * sw_set_driver_infos_i - Process the collection config data passed down
 *                         from the client.
 * @remote_msg: The user space address of our ioctl data.
 * @local_len:  The number of bytes of remote_msg we should copy.
 *
 * This function copies the ioctl data from user space to kernel
 * space.  That data is an array of sw_driver_interface_info structs,
 * which hold information about tracepoints, notifiers, and collector
 * configuration info for this collection run..  For each driver_info
 * struct, it calls the appropriate "add info" (registration/
 * configuration) function for each of the "when types" (begin, poll,
 * notifier, tracepoint, end) which should trigger a collection
 * operation for that collector.
 *
 * When this function is done, the data structures corresponding to
 * collection should be configured and initialized.
 *
 *
 * Returns: PW_SUCCESS on success, or a non-zero on an error.
 */
static long
sw_set_driver_infos_i(struct sw_driver_interface_msg __user *remote_msg,
		      int local_len)
{
	struct sw_driver_interface_info *local_info = NULL;
	struct sw_driver_interface_msg *local_msg = vmalloc(local_len);
	pw_u8_t read_triggers = 0x0;
	pw_u16_t num_infos = 0;
	sw_when_type_t i = SW_WHEN_TYPE_BEGIN;
	char *__data = (char *)local_msg->infos;
	size_t dst_idx = 0;

	if (!local_msg) {
		pw_pr_error("ERROR allocating space for local message!\n");
		return -EFAULT;
	}
	if (copy_from_user(local_msg, (struct sw_driver_interface_msg __user *)
			   remote_msg, local_len)) {
		pw_pr_error("ERROR copying message from user space!\n");
		vfree(local_msg);
		return -EFAULT;
	}
	/*
	 * We aren't allowed to config the driver multiple times between
	 * collections. Clear out any previous config values.
	 */
	sw_destroy_collector_lists_i();

	/*
	 * Did the user specify a min polling interval?
	 */
	sw_min_polling_interval_msecs = local_msg->min_polling_interval_msecs;
	pw_pr_debug("min_polling_interval_msecs = %u\n",
		    sw_min_polling_interval_msecs);

	num_infos = local_msg->num_infos;
	pw_pr_debug("LOCAL NUM INFOS = %u\n", num_infos);
	for (; num_infos > 0; --num_infos) {
		local_info =
			(struct sw_driver_interface_info *)&__data[dst_idx];
		dst_idx += (SW_DRIVER_INTERFACE_INFO_HEADER_SIZE() +
			    local_info->num_io_descriptors *
				    sizeof(struct sw_driver_io_descriptor));
		read_triggers = local_info->trigger_bits;
		pw_pr_debug(
			"read_triggers = %u, # msrs = %u, new dst_idx = %u\n",
			(unsigned int)read_triggers,
			(unsigned int)local_info->num_io_descriptors,
			(unsigned int)dst_idx);
		for (i = SW_WHEN_TYPE_BEGIN; i <= SW_WHEN_TYPE_END;
		     ++i, read_triggers >>= 1) {
			if (read_triggers & 0x1) { // Bit 'i' is set
				pw_pr_debug("BIT %d is SET!\n", i);
				if (i == SW_WHEN_TYPE_TRACEPOINT) {
					struct tn_data tn_data = {
						local_info,
						local_info->tracepoint_id_mask
					};
					pw_pr_debug(
						"TRACEPOINT, MASK = 0x%llx\n",
						local_info->tracepoint_id_mask);
					sw_for_each_tracepoint_node(
						&sw_add_trace_notifier_driver_info_i,
						&tn_data,
						false /*return-on-error*/);
				} else if (i == SW_WHEN_TYPE_NOTIFIER) {
					struct tn_data tn_data = {
						local_info,
						local_info->notifier_id_mask
					};
					pw_pr_debug(
						"NOTIFIER, MASK = 0x%llx\n",
						local_info->notifier_id_mask);
					sw_for_each_notifier_node(
						&sw_add_trace_notifier_driver_info_i,
						&tn_data,
						false /*return-on-error*/);
				} else {
					if (sw_add_driver_info(
						    &sw_collector_lists[i],
						    local_info)) {
						pw_pr_error(
							"WARNING: could NOT add driver info to list for 'when type' %d!\n",
							i);
					}
				}
			}
		}
	}
	if (sw_for_each_hw_op(&sw_post_config_i, NULL,
			      false /*return-on-error*/)) {
		pw_pr_error("POST-CONFIG error!\n");
	}
	vfree(local_msg);
	memset(&s_internal_state, 0, sizeof(s_internal_state));
	/*
	 * DEBUG: iterate over collection lists.
	 */
	sw_iterate_driver_info_lists_i();
	return PW_SUCCESS;
}

static long sw_handle_cmd_i(sw_driver_collection_cmd_t cmd,
			    u64 __user *remote_out_args)
{
	/*
	 * First, handle the command.
	 */
	if (cmd < SW_DRIVER_START_COLLECTION ||
	    cmd > SW_DRIVER_CANCEL_COLLECTION) {
		pw_pr_error("ERROR: invalid cmd = %d\n", cmd);
		return -PW_ERROR;
	}
	switch (cmd) {
	case SW_DRIVER_START_COLLECTION:
		if (sw_collection_start_i()) {
			return -PW_ERROR;
		}
		break;
	case SW_DRIVER_STOP_COLLECTION:
		if (sw_collection_stop_i()) {
			return -PW_ERROR;
		}
		break;
	default:
		pw_pr_error("WARNING: unsupported command %d\n", cmd);
		break;
	}
	/*
	 * Then retrieve sample stats.
	 */
#if DO_COUNT_DROPPED_SAMPLES
	if (cmd == SW_DRIVER_STOP_COLLECTION) {
		u64 local_args[2] = { sw_num_samples_produced,
				      sw_num_samples_dropped };
		if (copy_to_user(remote_out_args, local_args,
				 sizeof(local_args))) {
			pw_pr_error(
				"couldn't copy collection stats to user space!\n");
			return -PW_ERROR;
		}
	}
#endif // DO_COUNT_DROPPED_SAMPLES
	return PW_SUCCESS;
}

#ifdef SFI_SIG_OEMB
static int sw_do_parse_sfi_oemb_table(struct sfi_table_header *header)
{
#ifdef CONFIG_X86_WANT_INTEL_MID
	struct sfi_table_oemb *oemb = (struct sfi_table_oemb *)
		header; // 'struct sfi_table_oemb' defined in 'intel-mid.h'
	if (!oemb) {
		pw_pr_error("ERROR: NULL sfi table header!\n");
		return -PW_ERROR;
	}
	sw_scu_fw_major_minor = (oemb->scu_runtime_major_version << 8) |
				(oemb->scu_runtime_minor_version);
	pw_pr_debug("DEBUG: major = %u, minor = %u\n",
		    oemb->scu_runtime_major_version,
		    oemb->scu_runtime_minor_version);
#endif // CONFIG_X86_WANT_INTEL_MID
	return PW_SUCCESS;
}
#endif // SFI_SIG_OEMB

static void sw_do_extract_scu_fw_version(void)
{
	sw_scu_fw_major_minor = 0x0;
#ifdef SFI_SIG_OEMB
	if (sfi_table_parse(SFI_SIG_OEMB, NULL, NULL,
			    &sw_do_parse_sfi_oemb_table)) {
		pw_pr_force("WARNING: NO SFI information!\n");
	}
#endif // SFI_SIG_OEMB
}

static int sw_gather_trace_notifier_i(struct sw_trace_notifier_data *node,
				      struct sw_name_info_msg *msg,
				      enum sw_name_id_type type)
{
	pw_u16_t *idx = &msg->payload_len;
	char *buffer = (char *)&msg->pairs[*idx];
	struct sw_name_id_pair *pair = (struct sw_name_id_pair *)buffer;
	int id = sw_get_trace_notifier_id(node);
	struct sw_string_type *str = &pair->name;
	const char *abstract_name = sw_get_trace_notifier_abstract_name(node);

	if (likely(abstract_name && id >= 0)) {
		++msg->num_name_id_pairs;
		pair->type = type;
		pair->id = (u16)id;
		str->len = strlen(abstract_name) + 1; // "+1" for trailing '\0'
		memcpy(&str->data[0], abstract_name, str->len);

		pw_pr_debug("TP[%d] = %s (%u)\n",
			    sw_get_trace_notifier_id(node), abstract_name,
			    (unsigned int)strlen(abstract_name));

		*idx += SW_NAME_ID_HEADER_SIZE() +
			SW_STRING_TYPE_HEADER_SIZE() + str->len;
	}

	return PW_SUCCESS;
}

static int sw_gather_tracepoint_i(struct sw_trace_notifier_data *node,
				  void *priv)
{
	return sw_gather_trace_notifier_i(node, (struct sw_name_info_msg *)priv,
					  SW_NAME_TYPE_TRACEPOINT);
}

static int sw_gather_notifier_i(struct sw_trace_notifier_data *node, void *priv)
{
	return sw_gather_trace_notifier_i(node, (struct sw_name_info_msg *)priv,
					  SW_NAME_TYPE_NOTIFIER);
}

static long
sw_get_available_trace_notifiers_i(enum sw_name_id_type type,
				   struct sw_name_info_msg *local_info)
{
	long retVal = PW_SUCCESS;

	if (type == SW_NAME_TYPE_TRACEPOINT) {
		retVal = sw_for_each_tracepoint_node(&sw_gather_tracepoint_i,
						     local_info,
						     false /*return-on-error*/);
	} else {
		retVal = sw_for_each_notifier_node(&sw_gather_notifier_i,
						   local_info,
						   false /*return-on-error*/);
	}
	pw_pr_debug(
		"There are %u extracted traces/notifiers for a total of %u bytes!\n",
		local_info->num_name_id_pairs, local_info->payload_len);
	return retVal;
}

static int sw_gather_hw_op_i(const struct sw_hw_ops *op, void *priv)
{
	struct sw_name_info_msg *msg = (struct sw_name_info_msg *)priv;
	pw_u16_t *idx = &msg->payload_len;
	char *buffer = (char *)&msg->pairs[*idx];
	struct sw_name_id_pair *pair = (struct sw_name_id_pair *)buffer;
	struct sw_string_type *str = &pair->name;
	const char *abstract_name = sw_get_hw_op_abstract_name(op);
	int id = sw_get_hw_op_id(op);

	pw_pr_debug("Gather Collector[%d] = %s\n", id, abstract_name);
	if (likely(abstract_name && id >= 0)) {
		/*
		 * Final check: is this operation available on the
		 * target platform? If 'available' function doesn't
		 * exist then YES. Else call 'available'
		 * function to decide.
		 */
		pw_pr_debug("%s has available = %p\n", abstract_name,
			    op->available);
		if (!op->available || (*op->available)()) {
			++msg->num_name_id_pairs;
			pair->type = SW_NAME_TYPE_COLLECTOR;
			pair->id = (u16)id;
			str->len = strlen(abstract_name) +
				   1; // "+1" for trailing '\0'
			memcpy(&str->data[0], abstract_name, str->len);

			*idx += SW_NAME_ID_HEADER_SIZE() +
				SW_STRING_TYPE_HEADER_SIZE() + str->len;
		}
	}

	return PW_SUCCESS;
}

static long sw_get_available_collectors_i(struct sw_name_info_msg *local_info)
{
	return sw_for_each_hw_op(&sw_gather_hw_op_i, local_info,
				 false /*return-on-error*/);
}

static long
sw_get_available_name_id_mappings_i(enum sw_name_id_type type,
				    struct sw_name_info_msg __user *remote_info,
				    size_t local_len)
{
	char *buffer = vmalloc(local_len);
	struct sw_name_info_msg *local_info = NULL;
	long retVal = PW_SUCCESS;

	if (!buffer) {
		pw_pr_error("ERROR: couldn't alloc temp buffer!\n");
		return -PW_ERROR;
	}
	memset(buffer, 0, local_len);
	local_info = (struct sw_name_info_msg *)buffer;

	if (type == SW_NAME_TYPE_COLLECTOR) {
		retVal = sw_get_available_collectors_i(local_info);
	} else {
		retVal = sw_get_available_trace_notifiers_i(type, local_info);
	}
	if (retVal == PW_SUCCESS) {
		retVal = copy_to_user(remote_info, local_info, local_len);
		if (retVal) {
			pw_pr_error(
				"ERROR: couldn't copy tracepoint info to user space!\n");
		}
	}
	vfree(buffer);
	return retVal;
}

static long
sw_get_topology_changes_i(struct sw_driver_topology_msg __user *remote_msg,
			  size_t local_len)
{
	char *buffer = NULL;
	struct sw_driver_topology_msg *local_msg = NULL;
	size_t buffer_len = sizeof(struct sw_driver_topology_msg) +
			    sw_num_topology_entries *
				    sizeof(struct sw_driver_topology_change);
	long retVal = PW_SUCCESS;
	struct sw_driver_topology_change *dst = NULL;
	size_t dst_idx = 0;

	SW_LIST_HEAD_VAR(sw_topology_node) *head = (void *)&sw_topology_list;
	struct sw_topology_node *tnode = NULL;

	if (local_len < buffer_len) {
		pw_pr_error(
			"ERROR: insufficient buffer space to encode topology changes! Requires %zu, output space = %zu\n",
			buffer_len, local_len);
		return -EIO;
	}

	buffer = vmalloc(buffer_len);
	if (!buffer) {
		pw_pr_error(
			"ERROR: couldn't allocate buffer for topology transfer!\n");
		return -EIO;
	}
	memset(buffer, 0, buffer_len);

	local_msg = (struct sw_driver_topology_msg *)buffer;
	local_msg->num_entries = sw_num_topology_entries;
	dst = (struct sw_driver_topology_change *)&local_msg
		      ->topology_entries[0];
	SW_LIST_FOR_EACH_ENTRY(tnode, head, list)
	{
		struct sw_driver_topology_change *change = &tnode->change;

		memcpy(&dst[dst_idx++], change, sizeof(*change));
	}
	retVal = copy_to_user(remote_msg, local_msg, buffer_len);
	if (retVal) {
		pw_pr_error(
			"ERROR: couldn't copy topology changes to user space!\n");
	}
	vfree(buffer);
	return retVal;
}

#if defined(CONFIG_COMPAT) && defined(CONFIG_X86_64)
#define MATCH_IOCTL(num, pred) ((num) == (pred) || (num) == (pred##32))
#else
#define MATCH_IOCTL(num, pred) ((num) == (pred))
#endif

static long sw_unlocked_handle_ioctl_i(unsigned int ioctl_num,
				       void *p_local_args)
{
	struct sw_driver_ioctl_arg local_args;
	int local_in_len, local_out_len;

	if (!p_local_args) {
		pw_pr_error("ERROR: NULL p_local_args value?!\n");
		return -PW_ERROR;
	}

	/*
	 * (1) Sanity check:
	 * Before doing anything, double check to
	 * make sure this IOCTL was really intended
	 * for us!
	 */
	if (_IOC_TYPE(ioctl_num) != APWR_IOCTL_MAGIC_NUM) {
		pw_pr_error(
			"ERROR: requested IOCTL TYPE (%d) != APWR_IOCTL_MAGIC_NUM (%d)\n",
			_IOC_TYPE(ioctl_num), APWR_IOCTL_MAGIC_NUM);
		return -PW_ERROR;
	}
	/*
	 * (2) Extract arg lengths.
	 */
	local_args = *((struct sw_driver_ioctl_arg *)p_local_args);

	local_in_len = local_args.in_len;
	local_out_len = local_args.out_len;
	pw_pr_debug("GU: local_in_len = %d, local_out_len = %d\n", local_in_len,
		    local_out_len);
	/*
	 * (3) Service individual IOCTL requests.
	 */
	if (MATCH_IOCTL(ioctl_num, PW_IOCTL_CONFIG)) {
		pw_pr_debug("PW_IOCTL_CONFIG\n");
		return sw_set_driver_infos_i(
			(struct sw_driver_interface_msg __user *)
				local_args.in_arg,
			local_in_len);
	} else if (MATCH_IOCTL(ioctl_num, PW_IOCTL_CMD)) {
		sw_driver_collection_cmd_t local_cmd;

		pw_pr_debug("PW_IOCTL_CMD\n");
		if (get_user(local_cmd, (sw_driver_collection_cmd_t __user *)
						local_args.in_arg)) {
			pw_pr_error("ERROR: could NOT extract cmd value!\n");
			return -PW_ERROR;
		}
		return sw_handle_cmd_i(local_cmd,
				       (u64 __user *)local_args.out_arg);
	} else if (MATCH_IOCTL(ioctl_num, PW_IOCTL_POLL)) {
		pw_pr_debug("PW_IOCTL_POLL\n");
		return DO_PER_CPU_OVERHEAD_FUNC_RET(int, sw_collection_poll_i);
	} else if (MATCH_IOCTL(ioctl_num, PW_IOCTL_IMMEDIATE_IO)) {
		struct sw_driver_interface_info *local_info;
		struct sw_driver_io_descriptor *local_descriptor = NULL;
		int retVal = PW_SUCCESS;
		char *src_vals = NULL;
		char *dst_vals = NULL;

		pw_pr_debug("PW_IOCTL_IMMEDIATE_IO\n");
		pw_pr_debug("local_in_len = %u\n", local_in_len);

		src_vals = vmalloc(local_in_len);
		if (!src_vals) {
			pw_pr_error(
				"ERROR allocating space for immediate IO\n");
			return -PW_ERROR;
		}
		if (local_out_len) {
			dst_vals = vmalloc(local_out_len);
			if (!dst_vals) {
				vfree(src_vals);
				pw_pr_error(
					"ERROR allocating space for immediate IO\n");
				return -PW_ERROR;
			}
		}
		if (copy_from_user(src_vals, (char __user *)local_args.in_arg,
				   local_in_len)) {
			pw_pr_error(
				"ERROR copying in immediate IO descriptor\n");
			retVal = -PW_ERROR;
			goto ret_immediate_io;
		}
		local_info = (struct sw_driver_interface_info *)src_vals;
		pw_pr_debug(
			"OK, asked to perform immediate IO on cpu(s) %d, # descriptors = %d\n",
			local_info->cpu_mask, local_info->num_io_descriptors);
		/*
		 * For now, require only a single descriptor.
		 */
		if (local_info->num_io_descriptors != 1) {
			pw_pr_error(
				"ERROR: told to perform immediate IO with %d descriptors -- MAX of 1 descriptor allowed!\n",
				local_info->num_io_descriptors);
			retVal = -PW_ERROR;
			goto ret_immediate_io;
		}
		local_descriptor = ((struct sw_driver_io_descriptor *)
					    local_info->descriptors);
		pw_pr_debug("Collection type after %d\n",
			    local_descriptor->collection_type);
		/*
		 * Check cpu mask for correctness here. For now, we do NOT allow
		 * reading on ALL cpus.
		 */
		if ((int)local_info->cpu_mask < -1 ||
		    (int)local_info->cpu_mask >= (int)sw_max_num_cpus) {
			pw_pr_error(
				"ERROR: invalid cpu mask %d specified in immediate IO; valid values are: -1, [0 -- %d]!\n",
				local_info->cpu_mask, sw_max_num_cpus - 1);
			retVal = -PW_ERROR;
			goto ret_immediate_io;
		}
		/*
		 * Check collection type for correctness here
		 */
		pw_pr_debug(
			"Asked to perform immediate IO with descriptor with type = %d, on cpu = %d\n",
			local_descriptor->collection_type,
			local_info->cpu_mask);
		if (sw_is_valid_hw_op_id(local_descriptor->collection_type) ==
		    false) {
			pw_pr_error(
				"ERROR: invalid collection type %d specified for immediate IO\n",
				(int)local_descriptor->collection_type);
			retVal = -PW_ERROR;
			goto ret_immediate_io;
		}
		/*
		 * Check collection cmd for correctness here
		 */
		if (local_descriptor->collection_command < SW_IO_CMD_READ ||
		    local_descriptor->collection_command > SW_IO_CMD_WRITE) {
			pw_pr_error(
				"ERROR: invalid collection command %d specified for immediate IO\n",
				local_descriptor->collection_command);
			retVal = -PW_ERROR;
			goto ret_immediate_io;
		}
		/*
		 * Initialize the descriptor -- 'MMIO' and 'IPC' reads may need
		 * an "ioremap_nocache"
		 */
		if (sw_init_driver_io_descriptor(local_descriptor)) {
			pw_pr_error(
				"ERROR initializing immediate IO descriptor\n");
			retVal = -PW_ERROR;
			goto ret_immediate_io;
		}
		/*
		 * OK, perform the actual IO.
		 */
		{
			struct sw_immediate_request_info request_info = {
				local_descriptor, dst_vals, &retVal
			};
			struct cpumask cpumask;

			cpumask_clear(&cpumask);
			switch (local_info->cpu_mask) {
			case -1: // IO on ANY CPU (assume current CPU)
				cpumask_set_cpu(RAW_CPU(), &cpumask);
				pw_pr_debug("ANY CPU\n");
				break;
			default: // IO on a particular CPU
				cpumask_set_cpu(local_info->cpu_mask, &cpumask);
				pw_pr_debug("[%d] setting for %d\n", RAW_CPU(),
					    local_info->cpu_mask);
				break;
			}
			sw_schedule_work(&cpumask,
					 &sw_handle_immediate_request_i,
					 &request_info);
		}
		if (retVal != PW_SUCCESS) {
			pw_pr_error(
				"ERROR performing immediate IO on one (or more) CPUs!\n");
			goto ret_immediate_io_reset;
		}
		/*
		 * OK, all done.
		 */
		if (local_descriptor->collection_command == SW_IO_CMD_READ) {
			if (copy_to_user(local_args.out_arg, dst_vals,
					 local_out_len)) {
				pw_pr_error(
					"ERROR copying %u bytes of value to userspace!\n",
					local_out_len);
				retVal = -PW_ERROR;
				goto ret_immediate_io_reset;
			}
			pw_pr_debug(
				"OK, copied %u bytes of value to userspace addr %p!\n",
				local_out_len, local_args.out_arg);
		}
ret_immediate_io_reset:
		/*
		 * Reset the descriptor -- 'MMIO' and 'IPC' reads may have
		 * performed an "ioremap_nocache" which now needs to be
		 * unmapped.
		 */
		if (sw_reset_driver_io_descriptor(local_descriptor)) {
			pw_pr_error(
				"ERROR resetting immediate IO descriptor\n");
			retVal = -PW_ERROR;
			goto ret_immediate_io;
		}
ret_immediate_io:
		vfree(src_vals);
		if (dst_vals) {
			vfree(dst_vals);
		}
		return retVal;
	} else if (MATCH_IOCTL(ioctl_num, PW_IOCTL_GET_SCU_FW_VERSION)) {
		u32 local_data = (u32)sw_scu_fw_major_minor;

		if (put_user(local_data, (u32 __user *)local_args.out_arg)) {
			pw_pr_error(
				"ERROR copying scu fw version to userspace!\n");
			return -PW_ERROR;
		}
		return PW_SUCCESS;
	} else if (MATCH_IOCTL(ioctl_num, PW_IOCTL_GET_DRIVER_VERSION)) {
		pw_u64_t local_version =
			(pw_u64_t)SW_DRIVER_VERSION_MAJOR << 32 |
			(pw_u64_t)SW_DRIVER_VERSION_MINOR << 16 |
			(pw_u64_t)SW_DRIVER_VERSION_OTHER;
		if (put_user(local_version, (u64 __user *)local_args.out_arg)) {
			pw_pr_error(
				"ERROR copying driver version to userspace!\n");
			return -PW_ERROR;
		}
		return PW_SUCCESS;
	} else if (MATCH_IOCTL(ioctl_num, PW_IOCTL_GET_AVAILABLE_TRACEPOINTS)) {
		pw_pr_debug("DEBUG: AVAIL tracepoints! local_out_len = %u\n",
			    local_out_len);
		return sw_get_available_name_id_mappings_i(
			SW_NAME_TYPE_TRACEPOINT,
			(struct sw_name_info_msg __user *)local_args.out_arg,
			local_out_len);
	} else if (MATCH_IOCTL(ioctl_num, PW_IOCTL_GET_AVAILABLE_NOTIFIERS)) {
		pw_pr_debug("DEBUG: AVAIL tracepoints! local_out_len = %u\n",
			    local_out_len);
		return sw_get_available_name_id_mappings_i(
			SW_NAME_TYPE_NOTIFIER,
			(struct sw_name_info_msg __user *)local_args.out_arg,
			local_out_len);
	} else if (MATCH_IOCTL(ioctl_num, PW_IOCTL_GET_AVAILABLE_COLLECTORS)) {
		pw_pr_debug("DEBUG: AVAIL tracepoints! local_out_len = %u\n",
			    local_out_len);
		return sw_get_available_name_id_mappings_i(
			SW_NAME_TYPE_COLLECTOR,
			(struct sw_name_info_msg __user *)local_args.out_arg,
			local_out_len);
	} else if (MATCH_IOCTL(ioctl_num, PW_IOCTL_GET_TOPOLOGY_CHANGES)) {
		pw_pr_debug("DEBUG: TOPOLOGY changes! local_out_len = %u\n",
			    local_out_len);
		return sw_get_topology_changes_i(
			(struct sw_driver_topology_msg __user *)
				local_args.out_arg,
			local_out_len);
	} else {
		pw_pr_error("ERROR: invalid ioctl num: %u\n",
			    _IOC_NR(ioctl_num));
	}
	return -PW_ERROR;
}

static enum sw_driver_collection_cmd sw_get_collection_cmd_i(void)
{
	return s_internal_state.cmd;
};

static bool sw_should_flush_buffer_i(void)
{
	return s_internal_state.drain_buffers;
};

int sw_load_driver_i(void)
{
	/*
	 * Set per-cpu buffer size.
	 * First, Perform sanity checking of per-cpu buffer size.
	 */
	/*
	 * 1. Num pages MUST be pow-of-2.
	 */
	{
		if (sw_buffer_num_pages & (sw_buffer_num_pages - 1)) {
			pw_pr_error(
				"Invalid value (%u) for number of pages in each per-cpu buffer; MUST be a power of 2!\n",
				sw_buffer_num_pages);
			return -PW_ERROR;
		}
	}
	/*
	 * 2. Num pages MUST be <= 16 (i.e. per-cpu buffer size
	 * MUST be <= 64 kB)
	 */
	{
		if (sw_buffer_num_pages > 16) {
			pw_pr_error(
				"Invalid value (%u) for number of pages in each per-cpu buffer; MUST be <= 16!\n",
				sw_buffer_num_pages);
			return -PW_ERROR;
		}
	}
	sw_buffer_alloc_size = sw_buffer_num_pages * PAGE_SIZE;
	/*
	 * Retrieve any arch details here.
	 */
	if (sw_get_arch_details_i()) {
		pw_pr_error("ERROR retrieving arch details!\n");
		return -PW_ERROR;
	}
	/*
	 * Check to see if the user wants us to force
	 * software coordination of CPU frequencies.
	 */
	if (do_force_module_scope_for_cpu_frequencies) {
		pw_pr_force(
			"DEBUG: FORCING MODULE SCOPE FOR CPU FREQUENCIES!\n");
		if (sw_set_module_scope_for_cpus()) {
			pw_pr_force("ERROR setting affected cpus\n");
			return -PW_ERROR;
		} else {
			pw_pr_debug("OK, setting worked\n");
		}
	}
	if (sw_init_data_structures_i()) {
		pw_pr_error("ERROR initializing data structures!\n");
		goto err_ret_init_data;
	}
	if (sw_register_dev(&s_ops)) {
		goto err_ret_register_dev;
	}
	/*
	 * Retrieve a list of tracepoint structs to use when
	 * registering probe functions.
	 */
	{
		if (sw_extract_tracepoints()) {
			pw_pr_error(
				"ERROR: could NOT retrieve a complete list of valid tracepoint structs!\n");
			goto err_ret_tracepoint;
		}
	}
	pw_pr_force("-----------------------------------------\n");
	pw_pr_force("OK: LOADED SoC Watch Driver\n");
#ifdef CONFIG_X86_WANT_INTEL_MID
	pw_pr_force("SOC Identifier = %u, Stepping = %u\n",
		    intel_mid_identify_cpu(), intel_mid_soc_stepping());
#endif // CONFIG_X86_WANT_INTEL_MID
	pw_pr_force("-----------------------------------------\n");
	return PW_SUCCESS;

err_ret_tracepoint:
	sw_unregister_dev();
err_ret_register_dev:
	sw_destroy_data_structures_i();
err_ret_init_data:
	if (do_force_module_scope_for_cpu_frequencies) {
		if (sw_reset_module_scope_for_cpus()) {
			pw_pr_force("ERROR resetting affected cpus\n");
		} else {
			pw_pr_debug("OK, resetting worked\n");
		}
	}
	return -PW_ERROR;
}

void sw_unload_driver_i(void)
{
	sw_iterate_driver_info_lists_i();

	sw_unregister_dev();

	sw_destroy_data_structures_i();

	if (do_force_module_scope_for_cpu_frequencies) {
		if (sw_reset_module_scope_for_cpus()) {
			pw_pr_force("ERROR resetting affected cpus\n");
		} else {
			pw_pr_debug("OK, resetting worked\n");
		}
	}

	pw_pr_force("-----------------------------------------\n");
	pw_pr_force("OK: UNLOADED SoC Watch Driver\n");

	sw_print_trace_notifier_overheads();
	sw_print_output_buffer_overheads();
	PRINT_CUMULATIVE_OVERHEAD_PARAMS(sw_collection_poll_i, "POLL");
	PRINT_CUMULATIVE_OVERHEAD_PARAMS(sw_any_seg_full, "ANY_SEG_FULL");
#if DO_TRACK_MEMORY_USAGE
	{
		/*
		 * Dump memory stats.
		 */
		pw_pr_force(
			"TOTAL # BYTES ALLOCED = %llu, CURR # BYTES ALLOCED = %llu, MAX # BYTES ALLOCED = %llu\n",
			sw_get_total_bytes_alloced(),
			sw_get_curr_bytes_alloced(),
			sw_get_max_bytes_alloced());
		if (unlikely(sw_get_curr_bytes_alloced())) {
			pw_pr_force(
				"***********************************************************************\n");
			pw_pr_force(
				"WARNING: possible memory leak: there are %llu bytes still allocated!\n",
				sw_get_curr_bytes_alloced());
			pw_pr_force(
				"***********************************************************************\n");
		}
	}
#endif // DO_TRACK_MEMORY_USAGE
	pw_pr_force("-----------------------------------------\n");
}

module_init(sw_load_driver_i);
module_exit(sw_unload_driver_i);

MODULE_LICENSE("GPL");
MODULE_AUTHOR(MOD_AUTHOR);
MODULE_DESCRIPTION(MOD_DESC);
