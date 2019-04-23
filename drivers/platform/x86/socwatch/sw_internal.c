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

#include "sw_hardware_io.h"
#include "sw_mem.h"
#include "sw_kernel_defines.h"
#include "sw_telem.h"
#include "sw_internal.h"

bool sw_check_output_buffer_params(
	void __user *buffer, size_t bytes_to_read, size_t buff_size)
{
	if (!buffer) {
		pw_pr_error("ERROR: NULL ptr in sw_consume_data!\n");
		return false;
	}
	if (bytes_to_read != buff_size) {
		pw_pr_error("Error: bytes_to_read = %zu, required to be %zu\n",
			bytes_to_read, buff_size);
		return false;
	}
	return true;
}

unsigned long sw_copy_to_user(char __user *dst, char *src, size_t bytes_to_copy)
{
	return copy_to_user(dst, src, bytes_to_copy);
}

void sw_schedule_work(
	const struct cpumask *mask, void (*work)(void *), void *data)
{
	/*
	 * Did the user ask us to run on 'ANY' CPU?
	 */
	if (cpumask_empty(mask))
		(*work)(data); /* Call on current CPU */
	else {
		preempt_disable();
		{
			/*
			 * Did the user ask to run on this CPU?
			 */
			if (cpumask_test_cpu(RAW_CPU(), mask))
				(*work)(data); /* Call on current CPU */

			/*
			 * OK, now check other CPUs.
			 */
			smp_call_function_many(mask, work, data, true);
		}
		preempt_enable();
	}
}

int sw_get_cpu(unsigned long *flags)
{
	local_irq_save(*flags);
	return get_cpu();
}

void sw_put_cpu(unsigned long flags)
{
	put_cpu();
	local_irq_restore(flags);
}

#ifndef CONFIG_NR_CPUS_PER_MODULE
	#define CONFIG_NR_CPUS_PER_MODULE 2
#endif /* CONFIG_NR_CPUS_PER_MODULE */

static void sw_get_cpu_sibling_mask(int cpu, struct cpumask *sibling_mask)
{
	unsigned int base =
		(cpu/CONFIG_NR_CPUS_PER_MODULE) * CONFIG_NR_CPUS_PER_MODULE;
	unsigned int i;

	cpumask_clear(sibling_mask);
	for (i = base; i < (base+CONFIG_NR_CPUS_PER_MODULE); ++i)
		cpumask_set_cpu(i, sibling_mask);

}

struct pw_cpufreq_node {
	int cpu;
	struct cpumask cpus, related_cpus;
	unsigned int shared_type;
	struct list_head list;
};
static struct list_head pw_cpufreq_policy_lists;

int sw_set_module_scope_for_cpus(void)
{
	/*
	 * Warning: no support for cpu hotplugging!
	 */
	int cpu = 0;

	INIT_LIST_HEAD(&pw_cpufreq_policy_lists);
	for_each_online_cpu(cpu) {
		struct cpumask sibling_mask;
		struct pw_cpufreq_node *node = NULL;
		struct cpufreq_policy *policy = cpufreq_cpu_get(cpu);

		if (!policy)
			continue;

		/*
		 * Get siblings for this cpu.
		 */
		sw_get_cpu_sibling_mask(cpu, &sibling_mask);
		/*
		 * Check if affected_cpus already contains sibling_mask
		 */
		if (cpumask_subset(&sibling_mask, policy->cpus)) {
			/*
			 * 'sibling_mask' is already a subset of
			 * affected_cpus -- nothing to do on this CPU.
			 */
			cpufreq_cpu_put(policy);
			continue;
		}

		node = sw_kmalloc(sizeof(*node), GFP_ATOMIC);
		if (node) {
			cpumask_clear(&node->cpus);
			cpumask_clear(&node->related_cpus);

			node->cpu = cpu;
			cpumask_copy(&node->cpus, policy->cpus);
			cpumask_copy(&node->related_cpus, policy->related_cpus);
			node->shared_type = policy->shared_type;
		}

		policy->shared_type = CPUFREQ_SHARED_TYPE_ALL;
		/*
		 * Set siblings. Don't worry about online/offline, that's
		 * handled below.
		 */
		cpumask_copy(policy->cpus, &sibling_mask);
		/*
		 * Ensure 'related_cpus' is a superset of 'cpus'
		 */
		cpumask_or(policy->related_cpus,
			policy->related_cpus, policy->cpus);
		/*
		 * Ensure 'cpus' only contains online cpus.
		 */
		cpumask_and(policy->cpus, policy->cpus, cpu_online_mask);

		cpufreq_cpu_put(policy);

		if (node) {
			INIT_LIST_HEAD(&node->list);
			list_add_tail(&node->list, &pw_cpufreq_policy_lists);
		}
	}
	return PW_SUCCESS;
}

int sw_reset_module_scope_for_cpus(void)
{
	struct list_head *head = &pw_cpufreq_policy_lists;

	while (!list_empty(head)) {
		struct pw_cpufreq_node *node =
			list_first_entry(head, struct pw_cpufreq_node, list);
		int cpu = node->cpu;
		struct cpufreq_policy *policy = cpufreq_cpu_get(cpu);

		if (!policy)
			continue;

		policy->shared_type = node->shared_type;
		cpumask_copy(policy->related_cpus, &node->related_cpus);
		cpumask_copy(policy->cpus, &node->cpus);

		cpufreq_cpu_put(policy);

		pw_pr_debug("OK, reset cpufreq_policy for cpu %d\n", cpu);
		list_del(&node->list);
		sw_kfree(node);
	}
	return PW_SUCCESS;
}

int sw_setup_telem(u64 addrs[3])
{
	return setup_telem(addrs);
}

void sw_destroy_telem(void)
{
	destroy_telem();
}
