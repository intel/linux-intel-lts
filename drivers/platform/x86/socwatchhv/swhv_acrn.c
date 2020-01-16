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
#include <linux/version.h>
#include <asm/io.h>
#include <linux/uaccess.h>

#include <asm/hypervisor.h>
#include <linux/vhm/acrn_hv_defs.h>
#include <linux/vhm/vhm_hypercall.h>

#include "swhv_defines.h"
#include "swhv_driver.h"
#include "swhv_ioctl.h"
#include "swhv_structs.h"
#include "control.h"
#include "swhv_acrn.h"
#include "swhv_acrn_sbuf.h"

/* *******************************************
 * Compile-time constants
 * *******************************************
 */
#define foreach_cpu(cpu, cpu_num) for ((cpu) = 0; (cpu) < (cpu_num); (cpu)++)

/* actual physical cpu number, initialized by module init */
static int pcpu_num;
bool flush_mode;

/* TODO is this needed? */
/* module_param(nr_cpus, int, S_IRUSR | S_IWUSR); */

static struct shared_buf **sbuf_per_cpu;

static pw_u64_t global_collection_switch;
static SW_DEFINE_LIST_HEAD(swhv_msr_collector, swhv_acrn_msr_collector_data);

/* used by the MSR read IOCTL */
struct profiling_msr_ops_list *msr_read_ops_list;

bool buffer_not_ready(int *cpu);

struct swhv_acrn_msr_collector_data *swhv_alloc_msr_collector_node(void)
{
	struct swhv_acrn_msr_collector_data *node =
		(struct swhv_acrn_msr_collector_data *)
		kmalloc(sizeof(struct swhv_acrn_msr_collector_data),
				GFP_KERNEL);

	if (node) {
		node->per_msg_payload_size = 0x0;
		node->sample_id = 0x0;
		node->msr_ops_list =
			kmalloc_array(pcpu_num,
				sizeof(struct profiling_msr_ops_list),
				GFP_KERNEL);
		memset(node->msr_ops_list, 0, pcpu_num *
			sizeof(struct profiling_msr_ops_list));
		SW_LIST_ENTRY_INIT(node, list);
	}
	return node;
}
struct swhv_acrn_msr_collector_data *swhv_add_driver_msr_info(void *list_head,
			   const struct swhv_driver_interface_info *info)
{
	int cpu;

	SW_LIST_HEAD_VAR(swhv_acrn_msr_collector_data) * head = list_head;

	struct swhv_acrn_msr_collector_data *node = swhv_alloc_msr_collector_node();

	if (!node) {
		pw_pr_error("ERROR allocating MSR collector node!\n");
		return NULL;
	}

	node->sample_id = info->sample_id;
	node->cpu_mask = info->cpu_mask;
	foreach_cpu(cpu, pcpu_num) {
		node->msr_ops_list[cpu].collector_id = COLLECTOR_SOCWATCH;
		node->msr_ops_list[cpu].msr_op_state = MSR_OP_REQUESTED;
	}

	SW_LIST_ADD(head, node, list);
	return node;
}


int swhv_add_driver_msr_io_desc(struct swhv_acrn_msr_collector_data *node,
				struct swhv_driver_io_descriptor *info)
{
	int idx, cpu;
	pw_u16_t num_entries;
	struct profiling_msr_op *msr_op = NULL;

	/* Confirm this is an MSR IO descriptor */
	if (info->collection_type != SWHV_COLLECTOR_TYPE_MSR) {
		pw_pr_error("ERROR trying to configure MSR collector with other data!\n");
		return -EINVAL;
	}

	foreach_cpu(cpu, pcpu_num) {
		num_entries = node->msr_ops_list[cpu].num_entries;
		if (num_entries >= MAX_MSR_LIST_NUM) {
			pw_pr_error("ERROR trying to add too many MSRs to collect!\n");
			return -PW_ERROR;
		}

		idx = num_entries;

		msr_op = &(node->msr_ops_list[cpu].entries[idx]);

		msr_op->msr_id = info->msr_descriptor.address;
		if (info->collection_command == SWHV_IO_CMD_READ)
			msr_op->msr_op_type = MSR_OP_READ;
		else if (info->collection_command == SWHV_IO_CMD_WRITE)
			msr_op->msr_op_type = MSR_OP_WRITE;


		/*
		 * Use the param field to set sample id.
		 * This'll be used in the hypervisor to set the id in the samples
		 */
		msr_op->param = (uint16_t)node->sample_id;

		num_entries++;

		if (num_entries < MAX_MSR_LIST_NUM)
			node->msr_ops_list[cpu].entries[num_entries].msr_id = -1;

		node->msr_ops_list[cpu].num_entries = num_entries;
	}
	return PW_SUCCESS;
}

int swhv_init_per_cpu_buffers(void)
{
	int i, ret, cpu;

	sbuf_per_cpu = vmalloc(pcpu_num * sizeof(struct shared_buf *));

	foreach_cpu(cpu, pcpu_num) {
		/* allocate shared_buf */
		sbuf_per_cpu[cpu] = sbuf_allocate(ACRN_BUF_ELEMENT_NUM,
					ACRN_BUF_ELEMENT_SIZE);
		if (!sbuf_per_cpu[cpu]) {
			pw_pr_error("Failed  to allocate buffer for cpu %d\n", cpu);
			ret = -ENOMEM;
			goto out_free;
		}
	}

	/* TODO understand the use of this API */
	foreach_cpu(cpu, pcpu_num) {
		ret = sbuf_share_setup(cpu, ACRN_SOCWATCH, sbuf_per_cpu[cpu]);
		if (ret < 0) {
			pw_pr_error("Failed to setup buffer for cpu %d\n", cpu);
			goto out_sbuf;
		}
	}

	return PW_SUCCESS;
out_sbuf:
	for (i = --cpu; i >= 0; i--)
		sbuf_share_setup(i, ACRN_SOCWATCH, NULL);

	cpu = pcpu_num;

out_free:
	for (i = --cpu; i >= 0; i--)
		sbuf_free(sbuf_per_cpu[i]);


	vfree(sbuf_per_cpu);
	return ret;
}

void swhv_destroy_per_cpu_buffers(void)
{
	int cpu;

	pw_pr_debug("%s, pcpu_num: %d\n", __func__, pcpu_num);

	foreach_cpu(cpu, pcpu_num) {
		/* TODO anything else to de-register? */
		/* deregister devices */

		/* set sbuf pointer to NULL in HV */
		sbuf_share_setup(cpu, ACRN_SOCWATCH, NULL);

		/* free sbuf, sbuf_per_cpu[cpu] should be set NULL */
		sbuf_free(sbuf_per_cpu[cpu]);
	}
	vfree(sbuf_per_cpu);
}

void swhv_free_msr_collector_node(struct swhv_acrn_msr_collector_data *node)
{
	if (node) {
		kfree(node->msr_ops_list);
		kfree(node);
	}
}

void swhv_init_msr_collector_list(void)
{
	void *list_head = &swhv_msr_collector;

	SW_LIST_HEAD_VAR(swhv_acrn_msr_collector_data) * head = list_head;
	SW_LIST_HEAD_INIT(head);
}

void swhv_destroy_msr_collector_list(void)
{
	void *list_head = &swhv_msr_collector;

	SW_LIST_HEAD_VAR(swhv_acrn_msr_collector_data) * head = list_head;
	while (!SW_LIST_EMPTY(head)) {
		struct swhv_acrn_msr_collector_data *curr =
			SW_LIST_GET_HEAD_ENTRY(head,
				swhv_acrn_msr_collector_data, list);

		SW_LIST_UNLINK(curr, list);
		swhv_free_msr_collector_node(curr);
	}
}

void swhv_handle_hypervisor_collector(uint32_t control_cmd)
{
	struct profiling_control *acrn_profiling_control;

	acrn_profiling_control =
		kmalloc(sizeof(struct profiling_control), GFP_KERNEL);
	memset(acrn_profiling_control, 0, sizeof(struct profiling_control));

	acrn_profiling_control->collector_id = COLLECTOR_SOCWATCH;

	if (control_cmd == 1) {
		/* start collection + send switch bitmask */
		pw_pr_debug("STARTING ACRN PROFILING SERVICE\n");
		/* first bit controls start/stop of collection */
		global_collection_switch |= control_cmd;
	} else if (control_cmd == 0) {
		/* stop collection + reset switch bitmask */
		pw_pr_debug("STOPPING ACRN PROFILING SERVICE\n");
		global_collection_switch = control_cmd;
	}
	acrn_profiling_control->switches = global_collection_switch;

	/* send collection command + switch bitmask */
	acrn_hypercall2(HC_PROFILING_OPS, PROFILING_SET_CONTROL_SWITCH,
		virt_to_phys(acrn_profiling_control));
	kfree(acrn_profiling_control);
}

int swhv_handle_msr_collector_list(void)
{
	void *list_head = &swhv_msr_collector;

	SW_LIST_HEAD_VAR(swhv_acrn_msr_collector_data) * head = list_head;
	int retVal = PW_SUCCESS;
	struct swhv_acrn_msr_collector_data *curr = NULL;

	if (SW_LIST_EMPTY(&swhv_msr_collector)) {
		pw_pr_debug("DEBUG: EMPTY MSR COLLECTOR LIST\n");
		return retVal;
	}

	if (!head)
		return -PW_ERROR;

	SW_LIST_FOR_EACH_ENTRY(curr, head, list) {
		pw_pr_debug("HANDLING MSR NODE\n");

		/* hypervisor call to do immediate MSR read */
		acrn_hypercall2(HC_PROFILING_OPS, PROFILING_MSR_OPS,
				virt_to_phys(curr->msr_ops_list));
	}
	return retVal;
}

long swhv_configure(struct swhv_driver_interface_msg __user *remote_msg, int local_len)
{
	struct swhv_driver_interface_info *local_info = NULL;
	struct swhv_driver_io_descriptor *local_io_desc = NULL;
	struct swhv_driver_interface_msg *local_msg = vmalloc(local_len);
	pw_u16_t num_infos = 0, num_io_desc = 0;
	pw_u32_t local_config_bitmap = 0;
	int done = 0;
	bool driver_info_added = false;

	char *__data = (char *)local_msg->infos;
	size_t dst_idx = 0, desc_idx = 0;
	struct swhv_acrn_msr_collector_data *msr_collector_node = NULL;

	if (!local_msg) {
		pw_pr_error("ERROR allocating space for local message!\n");
		return -EFAULT;
	}
	if (copy_from_user(local_msg, remote_msg, local_len)) {
		pw_pr_error("ERROR copying message from user space!\n");
		vfree(local_msg);
		return -EFAULT;
	}

	flush_mode = false;

	pw_pr_debug("local_len: %d\n", local_len);
	/*
	 * We aren't allowed to config the driver multiple times between
	 * collections. Clear out any previous config values.
	 */
	swhv_destroy_msr_collector_list();

	/* clear the collection bitmask */
	global_collection_switch = 0;

	num_infos = local_msg->num_infos;
	pw_pr_debug("LOCAL NUM INFOS = %u\n", num_infos);
	for (; num_infos > 0 && !done; --num_infos) {
		local_info = (struct swhv_driver_interface_info *)&__data[dst_idx];
		desc_idx = dst_idx + SWHV_DRIVER_INTERFACE_INFO_HEADER_SIZE();
		dst_idx += (SWHV_DRIVER_INTERFACE_INFO_HEADER_SIZE() +
				local_info->num_io_descriptors *
				sizeof(struct swhv_driver_io_descriptor));
		pw_pr_debug("# msrs = %u\n",
			(unsigned)local_info->num_io_descriptors);

		num_io_desc = local_info->num_io_descriptors;
		pw_pr_debug("LOCAL NUM IO DESC = %u\n", num_io_desc);

		driver_info_added = false;
		for (; num_io_desc > 0; --num_io_desc) {
			local_io_desc = (struct swhv_driver_io_descriptor *)
					&__data[desc_idx];
			desc_idx += sizeof(struct swhv_driver_io_descriptor);
			if (local_io_desc->collection_type ==
					SWHV_COLLECTOR_TYPE_MSR) {

				if (!driver_info_added) {
					msr_collector_node =
						swhv_add_driver_msr_info(
							&swhv_msr_collector,
							local_info);
					if (msr_collector_node == NULL)
						return -PW_ERROR;

					driver_info_added = true;
				}

				pw_pr_debug(
					"MSR - addr: 0x%llx, type: %u, read/write: %u\n",
					local_io_desc->msr_descriptor.address,
					local_io_desc->msr_descriptor.type,
					local_io_desc->collection_command);
				swhv_add_driver_msr_io_desc(msr_collector_node, local_io_desc);
			} else if (local_io_desc->collection_type ==
					SWHV_COLLECTOR_TYPE_SWITCH) {

				local_config_bitmap = local_io_desc->switch_descriptor.switch_bitmask;
				pw_pr_debug("local bitmask = %u\n", local_config_bitmap);

				global_collection_switch = local_config_bitmap;

				/* only one set of collection switches are
				 * expected, we are done configuring
				 */
				done = 1;
				break;
			} else
				pw_pr_error(
					"WARNING: unknown collector configuration requested, collector id: %u!\n",
					local_io_desc->collection_type);

		}
		driver_info_added = false;
	}
	vfree(local_msg);
	return PW_SUCCESS;
}

long swhv_stop(void)
{
	uint32_t control = 0; /* stop collection command */

	pw_pr_debug("socwatch: stop called\n");

	/* If MSR ops are present, perform them to get begin snapshot data. */
	swhv_handle_msr_collector_list();

	/* stop collection + reset switch bitmask */
	swhv_handle_hypervisor_collector(control);

	/* flush partially filled hypervisor buffers */
	flush_mode = true;

	/*
	 * Clear out the MSR collector list.
	 */
	swhv_destroy_msr_collector_list();

	return PW_SUCCESS;
}

long swhv_start(void)
{
	uint32_t control = 1; /* start collection command */
#if 0
	struct profiling_vm_info_list *vm_info_list = NULL;
	int i;
#endif
	pw_pr_debug("socwatch: start called\n");

	flush_mode = false;

	/* start collection + send switch bitmask */
	swhv_handle_hypervisor_collector(control);

	/* If MSR ops are present, perform them to get begin snapshot data. */
	swhv_handle_msr_collector_list();

#if 0
	/* Expand this eventually to retrive VM-realted info from the hypervisor */
	/* Leaving it here for now. */
	vm_info_list = kmalloc(sizeof(struct profiling_vm_info_list), GFP_KERNEL);
	memset(vm_info_list, 0, sizeof(struct profiling_vm_info_list));

	acrn_hypercall2(HC_PROFILING_OPS, PROFILING_GET_VMINFO, virt_to_phys(vm_info_list));

	pw_pr_debug("Number of VMs: %d\n", vm_info_list->num_vms);
	for (i = 0; i < vm_info_list->num_vms; ++i) {
		pw_pr_debug("VM id: %d\n", vm_info_list->vm_list[i].vm_id_num);
		pw_pr_debug("VM name: %s\n", vm_info_list->vm_list[i].vm_name);
	}
#endif
	return PW_SUCCESS;
}

long swhv_get_cpu_count(u32 __user *remote_args)
{
	uint32_t num_CPUs = pcpu_num;

	return copy_to_user(remote_args, &num_CPUs, sizeof(num_CPUs));
};

int device_open_i(struct inode *inode, struct file *file)
{
	pw_pr_debug("socwatch: device_open_i() called\n");
	return PW_SUCCESS;
}

long swhv_get_clock(u32 __user *remote_in_args, u64 __user *remote_args)
{
	return -1;
}

long swhv_get_topology(u64 __user *remote_args)
{
	return -1;
}

long swhv_get_hypervisor_type(u32 __user *remote_args)
{
	uint32_t hypervisor_type = swhv_hypervisor_acrn;

	return copy_to_user(remote_args, &hypervisor_type,
			sizeof(hypervisor_type));
}

long swhv_msr_read(u32 __user *remote_in_args, u64 __user *remote_args)
{
	int cpu;
	uint64_t msr_addr = 0, value;
	int ret = PW_SUCCESS;

	if (get_user(msr_addr, remote_in_args)) {
	pw_pr_error("ERROR: couldn't copy remote args for read MSR IOCTL!\n");
	return -1;
	}

	if (!msr_read_ops_list) {
	msr_read_ops_list = kmalloc_array(pcpu_num,
			sizeof(struct profiling_msr_ops_list), GFP_KERNEL);
	if (!msr_read_ops_list) {
		pw_pr_error("couldn't allocate memory for doing an MSR read!\n");
		return -1;
	}
	memset(msr_read_ops_list, 0, pcpu_num * sizeof(struct profiling_msr_ops_list));
	}

	/*
	 * The hypercall is set in such a way that the MSR read will occur on
	 * all CPUs and as a result we have to set up structures for each CPU.
	 */
	foreach_cpu(cpu, pcpu_num) {
		msr_read_ops_list[cpu].collector_id = COLLECTOR_SOCWATCH;
		msr_read_ops_list[cpu].msr_op_state = MSR_OP_REQUESTED;
		msr_read_ops_list[cpu].num_entries = 1;
		msr_read_ops_list[cpu].entries[0].msr_id = msr_addr;
		msr_read_ops_list[cpu].entries[0].msr_op_type = MSR_OP_READ;
		/* the next entry is expected to be set to -1 */
		msr_read_ops_list[cpu].entries[1].msr_id = -1;
		/* set to 0 to not generate sample in hypervisor */
		msr_read_ops_list[cpu].entries[1].param = 0;
	}

	/* hypervisor call to do immediate MSR read */
	acrn_hypercall2(HC_PROFILING_OPS, PROFILING_MSR_OPS,
		virt_to_phys(msr_read_ops_list));

	/* copy value to remote args, pick from any CPU */
	value = msr_read_ops_list[0].entries[0].value;

	if (copy_to_user(remote_args, &value, sizeof(value))) {
		pw_pr_error("ERROR: unable to copy MSR value to userspace!\n");
		ret = -PW_ERROR;
	}

	return ret;
}

long swhv_collection_poll(void)
{
	int ret = PW_SUCCESS;
	/*
	 * Handle 'POLL' timer expirations.
	 */
	if (SW_LIST_EMPTY(&swhv_msr_collector))
		pw_pr_debug("DEBUG: EMPTY MSR COLLECTOR POLL LIST\n");


	if (swhv_handle_msr_collector_list()) {
		pw_pr_error("ERROR: unable to copy MSR value to userspace!\n");
		ret = -PW_ERROR;
	}
	return ret;
}

ssize_t swhv_transfer_data(void *user_buffer, struct shared_buf *sbuf_to_copy, size_t bytes_to_read)
{
	unsigned long bytes_not_copied;
	ssize_t bytes_read;
	ssize_t ret = 0;
	void *data_read = NULL;

	if (bytes_to_read == 0) {
		pw_pr_debug("%s - 0 bytes requested to transfer! Returning...\n", __func__);
		return bytes_to_read;
	}

	data_read = vmalloc(bytes_to_read);
	if (!data_read) {
		pw_pr_error("couldn't allocate memory when trying to transfer data to userspace!\n");
		return 0;
	}

	pw_pr_debug("%s - bytes to transfer %zu\n", __func__, bytes_to_read);

	if (sbuf_to_copy) {
		bytes_read = sbuf_get_variable(sbuf_to_copy, &data_read, bytes_to_read);

		if (bytes_read != bytes_to_read)
			pw_pr_warn(
				"%s - bytes read (%zu bytes) are not equal to expected bytes (%zu bytes) to be read!",
				__func__, bytes_read, bytes_to_read);


		if (bytes_read < 0) {
			pw_pr_error("Error reading this buffer\n");
			ret = -PW_ERROR;
			goto ret_free;
		}
		if (bytes_read) {
			/* copy data to device file */
			if (bytes_read > bytes_to_read) {
				pw_pr_error("user buffer is too small\n");
				ret = -PW_ERROR;
				goto ret_free;
			}

			bytes_not_copied =
				copy_to_user(user_buffer, data_read, bytes_read);
			/* TODO check if this is meaningful enough to have */
			/* *offset += bytes_read - bytes_not_copied; */

			if (bytes_not_copied) {
				pw_pr_error(
					"transferring data to user mode failed, bytes %ld\n",
					bytes_not_copied);
				/* copy_to_user returns an unsigned */
				ret = -EIO;
				goto ret_free;
			}
			ret = bytes_read;
			goto ret_free;
		} else
			pw_pr_debug(
				"Buffer empty! nothing more to read from this buffer\n");

	}

ret_free:
	vfree(data_read);
	return ret;
}

bool buffer_not_ready(int *cpu)
{
	/* cycle through and confirm buffers on all CPUs are less than
	 * ACRN_BUF_TRANSFER_SIZE
	 * as well as flush mode has not been requested
	 */
	int i = 0;
	bool not_enough_data = true;

	pw_pr_debug(
		"checking if a buffer is ready to be copied to the device file\n");
	/*
	 * It's possible that the buffer from cpu0 may always have data to
	 * transfer and can potentially prevent buffers from other cpus from
	 * ever being serviced.
	 * TODO Consider adding an optimization to check for last cpu read.
	 */
	for (i = 0; i < pcpu_num; ++i) {
		if (ACRN_BUF_FILLED_SIZE(sbuf_per_cpu[i]) >= ACRN_BUF_TRANSFER_SIZE ||
				(flush_mode && ACRN_BUF_FILLED_SIZE(sbuf_per_cpu[i]))) {
			not_enough_data = false;
			*cpu = i;
			pw_pr_debug(
				"buffer ready (flush_mode=%d) on cpu %d, waking up read queue\n",
				flush_mode, *cpu);
			break;
		}
	}
	return not_enough_data && !flush_mode;
}

ssize_t device_read_i(struct file *file, char __user *user_buffer,
		size_t length, loff_t *offset)
{
	ssize_t bytes_read = 0;
	int cpu = 0;

	pw_pr_debug("%s - usermode attempting to read device file\n", __func__);

	if (buffer_not_ready(&cpu)) {
		pw_pr_debug("%s - no buffer ready to be read\n", __func__);
		return bytes_read;
	}

	if (flush_mode)
		pw_pr_debug("flush mode on, ready to flush a buffer\n");

	length = ACRN_BUF_FILLED_SIZE(sbuf_per_cpu[cpu]);
	pw_pr_debug("on cpu %d, buffer size is %zu bytes\n", cpu, length);

	bytes_read = swhv_transfer_data(user_buffer, sbuf_per_cpu[cpu], length);

	return bytes_read;
}

void cleanup_error_i(void)
{
	/* NOP for acrn */
}

int swhv_load_driver_i(void)
{
	int ret = PW_SUCCESS;

	if (x86_hyper_type != X86_HYPER_ACRN) {
		pw_pr_error("Non-ACRN hypervisor not supported!\n");
		return -EINVAL;
	}

	/* TODO: we could get the cpu count by querying the hypervisor later */
	pcpu_num = num_present_cpus();
	pw_pr_debug("%s, pcpu_num: %d\n", __func__, pcpu_num);

	ret = swhv_init_per_cpu_buffers();
	if (ret < 0)
		return ret;


	swhv_init_msr_collector_list();

	return ret;
}

void swhv_unload_driver_i(void)
{
	swhv_destroy_per_cpu_buffers();

	/* used by the MSR read IOCTL */
	kfree(msr_read_ops_list);
}
