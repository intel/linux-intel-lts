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

#include <linux/version.h> /* "LINUX_VERSION_CODE" */
#include <linux/hrtimer.h>
#if KERNEL_VERSION(4, 11, 0) > LINUX_VERSION_CODE
	#include <asm/cputime.h>
#else
	#include <linux/sched/cputime.h>
#endif
#include <asm/hardirq.h>
#include <asm/local.h>

#include <trace/events/power.h>
#include <trace/events/irq.h>
#include <trace/events/timer.h>
#include <trace/events/power.h>
#include <trace/events/sched.h>
#if KERNEL_VERSION(3, 14, 0) <= LINUX_VERSION_CODE
#include <asm/trace/irq_vectors.h> /* for the various APIC vector tracepoints
				    *  (e.g. "thermal_apic",
				    *  "local_timer" etc.)
				    */
#endif /* KERNEL_VERSION(3, 14, 0) <= LINUX_VERSION_CODE */
struct pool_workqueue;
struct cpu_workqueue_struct;
#include <trace/events/workqueue.h>
#include <linux/suspend.h> /* for 'pm_notifier' */
#include <linux/cpufreq.h> /* for "cpufreq_notifier" */
#include <linux/cpu.h> /* for 'CPU_UP_PREPARE' etc */

#include "sw_kernel_defines.h"
#include "sw_collector.h"
#include "sw_overhead_measurements.h"
#include "sw_tracepoint_handlers.h"
#include "sw_output_buffer.h"
#include "sw_mem.h"
#include "sw_trace_notifier_provider.h"

/* -------------------------------------------------
 * Compile time constants and useful macros.
 * -------------------------------------------------
 */
#ifndef __get_cpu_var
/*
 * Kernels >= 3.19 don't include a definition
 * of '__get_cpu_var'. Create one now.
 */
#define __get_cpu_var(var) (*this_cpu_ptr(&var))
#endif /* __get_cpu_var */

#define BEGIN_LOCAL_IRQ_STATS_READ(p)                                          \
	do {                                                                   \
		p = &__get_cpu_var(irq_stat);

#define END_LOCAL_IRQ_STATS_READ(p)                                            \
	}                                                                      \
	while (0)
/*
 * CAS{32,64}
 */
#define CAS32(p, o, n) (cmpxchg((p), (o), (n)) == (o))
#define CAS64(p, o, n) (cmpxchg64((p), (o), (n)) == (o))
/*
 * Timer start pid accessor macros
 */
#ifdef CONFIG_TIMER_STATS
#define GET_TIMER_THREAD_ID(t)                                                 \
	((t)->start_pid) /* 'start_pid' is actually the thread ID
			  * of the thread that initialized the timer
			  */
#else
#define GET_TIMER_THREAD_ID(t) (-1)
#endif /* CONFIG_TIMER_STATS */
/*
 * Tracepoint probe register/unregister functions and
 * helper macros.
 */
#if IS_ENABLED(CONFIG_TRACEPOINTS)
#if KERNEL_VERSION(2, 6, 35) > LINUX_VERSION_CODE
#define DO_REGISTER_SW_TRACEPOINT_PROBE(node, name, probe)                     \
	WARN_ON(register_trace_##name(probe))
#define DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, name, probe)                   \
	unregister_trace_##name(probe)
#elif KERNEL_VERSION(3, 15, 0) > LINUX_VERSION_CODE
#define DO_REGISTER_SW_TRACEPOINT_PROBE(node, name, probe)                     \
	WARN_ON(register_trace_##name(probe, NULL))
#define DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, name, probe)                   \
	unregister_trace_##name(probe, NULL)
#else
#define DO_REGISTER_SW_TRACEPOINT_PROBE(node, name, probe)                     \
	WARN_ON(tracepoint_probe_register(node->tp, probe, NULL))
#define DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, name, probe)                   \
	tracepoint_probe_unregister(node->tp, probe, NULL)
#endif
#else /* CONFIG_TRACEPOINTS */
#define DO_REGISTER_SW_TRACEPOINT_PROBE(...) /* NOP */
#define DO_UNREGISTER_SW_TRACEPOINT_PROBE(...) /* NOP */
#endif /* CONFIG_TRACEPOINTS */
#if KERNEL_VERSION(2, 6, 35) > LINUX_VERSION_CODE
#define _DEFINE_PROBE_FUNCTION(name, ...) static void name(__VA_ARGS__)
#else
#define _DEFINE_PROBE_FUNCTION(name, ...)                                      \
	static void name(void *ignore, __VA_ARGS__)
#endif
#define DEFINE_PROBE_FUNCTION(x) _DEFINE_PROBE_FUNCTION(x)

/*
 * Tracepoint probe function parameters.
 * These tracepoint signatures depend on kernel version.
 */
#if KERNEL_VERSION(2, 6, 36) > LINUX_VERSION_CODE
#define PROBE_TPS_PARAMS                                                       \
	sw_probe_power_start_i, unsigned int type, unsigned int state
#elif KERNEL_VERSION(2, 6, 38) > LINUX_VERSION_CODE
#define PROBE_TPS_PARAMS                                                       \
	sw_probe_power_start_i, unsigned int type, unsigned int state,         \
		unsigned int cpu_id
#else
#define PROBE_TPS_PARAMS                                                       \
	sw_probe_cpu_idle_i, unsigned int state, unsigned int cpu_id
#endif

#if KERNEL_VERSION(2, 6, 38) > LINUX_VERSION_CODE
#define PROBE_TPF_PARAMS                                                       \
	sw_probe_power_frequency_i, unsigned int type, unsigned int state
#else
#define PROBE_TPF_PARAMS                                                       \
	sw_probe_cpu_frequency_i, unsigned int new_freq, unsigned int cpu
#endif

#if KERNEL_VERSION(2, 6, 35) > LINUX_VERSION_CODE
#define PROBE_SCHED_WAKEUP_PARAMS                                              \
	sw_probe_sched_wakeup_i, struct rq *rq, struct task_struct *task,      \
		int success
#else
#define PROBE_SCHED_WAKEUP_PARAMS                                              \
	sw_probe_sched_wakeup_i, struct task_struct *task, int success
#endif

#if IS_ENABLED(CONFIG_ANDROID)
#if KERNEL_VERSION(3, 4, 0) > LINUX_VERSION_CODE
#define PROBE_WAKE_LOCK_PARAMS sw_probe_wake_lock_i, struct wake_lock *lock
#define PROBE_WAKE_UNLOCK_PARAMS                                               \
	sw_probe_wake_unlock_i, struct wake_unlock *unlock
#else
#define PROBE_WAKE_LOCK_PARAMS                                                 \
	sw_probe_wakeup_source_activate_i, const char *name, unsigned int state
#define PROBE_WAKE_UNLOCK_PARAMS                                               \
	sw_probe_wakeup_source_deactivate_i, const char *name,                 \
		unsigned int state
#endif /* version */
#endif /* CONFIG_ANDROID */

#if KERNEL_VERSION(2, 6, 35) >= LINUX_VERSION_CODE
#define PROBE_WORKQUEUE_PARAMS                                                 \
	sw_probe_workqueue_execution_i, struct task_struct *wq_thread,         \
		struct work_struct *work
#else
#define PROBE_WORKQUEUE_PARAMS                                                 \
	sw_probe_workqueue_execute_start_i, struct work_struct *work
#endif

#define PROBE_SCHED_SWITCH_PARAMS                                              \
	sw_probe_sched_switch_i, struct task_struct *prev,                     \
		struct task_struct *next
/*
 * These tracepoint signatures are independent of kernel version.
 */
#define PROBE_IRQ_PARAMS                                                       \
	sw_probe_irq_handler_entry_i, int irq, struct irqaction *action
#define PROBE_TIMER_ARGS sw_probe_timer_expire_entry_i, struct timer_list *t
#define PROBE_HRTIMER_PARAMS                                                   \
	sw_probe_hrtimer_expire_entry_i, struct hrtimer *hrt, ktime_t *now
#define PROBE_PROCESS_FORK_PARAMS                                              \
	sw_probe_sched_process_fork_i, struct task_struct *parent,             \
		struct task_struct *child
#define PROBE_SCHED_PROCESS_EXIT_PARAMS                                        \
	sw_probe_sched_process_exit_i, struct task_struct *task
#define PROBE_THERMAL_APIC_ENTRY_PARAMS                                        \
	sw_probe_thermal_apic_entry_i, int vector
#define PROBE_THERMAL_APIC_EXIT_PARAMS sw_probe_thermal_apic_exit_i, int vector

#define IS_VALID_WAKEUP_EVENT(cpu)                                             \
	({                                                                     \
		bool *per_cpu_event =                                          \
			&per_cpu(sw_is_valid_wakeup_event, (cpu));             \
		bool old_value =                                               \
			CAS32(per_cpu_event, true, sw_wakeup_event_flag);      \
		old_value;                                                     \
	})
#define SHOULD_PRODUCE_WAKEUP_SAMPLE(cpu) (IS_VALID_WAKEUP_EVENT(cpu))
#define RESET_VALID_WAKEUP_EVENT_COUNTER(cpu)                                  \
	(per_cpu(sw_is_valid_wakeup_event, (cpu)) = true)

#define NUM_TRACEPOINT_NODES SW_ARRAY_SIZE(s_trace_collector_lists)
#define NUM_VALID_TRACEPOINTS (NUM_TRACEPOINT_NODES - 1) /* "-1" for IPI */
#define FOR_EACH_TRACEPOINT_NODE(idx, node)                                    \
	for (idx = 0; idx < NUM_TRACEPOINT_NODES &&                            \
		      (node = &s_trace_collector_lists[idx]);                  \
	     ++idx)

#define FOR_EACH_NOTIFIER_NODE(idx, node)                                      \
	for (idx = 0; idx < SW_ARRAY_SIZE(s_notifier_collector_lists) &&       \
		      (node = &s_notifier_collector_lists[idx]);               \
	     ++idx)
/*
 * Use these macros if all tracepoint ID numbers
 * ARE contiguous from 0 -- max tracepoint ID #
 */
/* #if 0
#define IS_VALID_TRACE_NOTIFIER_ID(id)                                         \
	((id) >= 0 && (id) < SW_ARRAY_SIZE(s_trace_collector_lists))
#define GET_COLLECTOR_TRACE_NODE(id) (&s_trace_collector_lists[id])
#define FOR_EACH_trace_notifier_id(idx)                                        \
	for (idx = 0; idx < SW_ARRAY_SIZE(s_trace_collector_lists); ++idx)
#endif */
/*
 * Use these macros if all tracepoint ID numbers
 * are NOT contiguous from 0 -- max tracepoint ID #
 */
#define GET_COLLECTOR_TRACE_NODE(idx)                                          \
	({                                                                     \
		int __idx = 0;                                                 \
		struct sw_trace_notifier_data *__node = NULL,                  \
					      *__retVal = NULL;                \
		FOR_EACH_TRACEPOINT_NODE(__idx, __node)                        \
		{                                                              \
			if ((idx) == GET_TRACE_NOTIFIER_ID(__node)) {          \
				__retVal = __node;                             \
				break;                                         \
			}                                                      \
		}                                                              \
		__retVal;                                                      \
	})
#define IS_VALID_TRACE_NOTIFIER_ID(idx) (GET_COLLECTOR_TRACE_NODE(idx) != NULL)

#define GET_COLLECTOR_NOTIFIER_NODE(idx)                                       \
	({                                                                     \
		int __idx = 0;                                                 \
		struct sw_trace_notifier_data *__node = NULL,                  \
					      *__retVal = NULL;                \
		FOR_EACH_NOTIFIER_NODE(__idx, __node)                          \
		{                                                              \
			if ((idx) == GET_TRACE_NOTIFIER_ID(__node)) {          \
				__retVal = __node;                             \
				break;                                         \
			}                                                      \
		}                                                              \
		__retVal;                                                      \
	})
#define IS_VALID_NOTIFIER_ID(idx) (GET_COLLECTOR_NOTIFIER_NODE(idx) != NULL)

/* -------------------------------------------------
 * Local function declarations.
 * -------------------------------------------------
 */
/*
 * The tracepoint registration functions.
 */
int sw_register_trace_cpu_idle_i(struct sw_trace_notifier_data *node);
int sw_unregister_trace_cpu_idle_i(struct sw_trace_notifier_data *node);
int sw_register_trace_cpu_frequency_i(struct sw_trace_notifier_data *node);
int sw_unregister_trace_cpu_frequency_i(struct sw_trace_notifier_data *node);
int sw_register_trace_irq_handler_entry_i(struct sw_trace_notifier_data *node);
int sw_unregister_trace_irq_handler_entry_i(struct sw_trace_notifier_data *node);
int sw_register_trace_timer_expire_entry_i(struct sw_trace_notifier_data *node);
int sw_unregister_trace_timer_expire_entry_i(struct sw_trace_notifier_data *node);
int sw_register_trace_hrtimer_expire_entry_i(struct sw_trace_notifier_data *node);
int sw_unregister_trace_hrtimer_expire_entry_i(struct sw_trace_notifier_data *node);
int sw_register_trace_sched_wakeup_i(struct sw_trace_notifier_data *node);
int sw_unregister_trace_sched_wakeup_i(struct sw_trace_notifier_data *node);
int sw_register_trace_sched_process_fork_i(struct sw_trace_notifier_data *node);
int sw_unregister_trace_sched_process_fork_i(struct sw_trace_notifier_data *node);
int sw_register_trace_sched_process_exit_i(struct sw_trace_notifier_data *node);
int sw_unregister_trace_sched_process_exit_i(struct sw_trace_notifier_data *node);
#if KERNEL_VERSION(3,14,0) <= LINUX_VERSION_CODE
    int sw_register_trace_thermal_apic_entry_i(struct sw_trace_notifier_data *node);
    int sw_unregister_trace_thermal_apic_entry_i(struct sw_trace_notifier_data *node);
    int sw_register_trace_thermal_apic_exit_i(struct sw_trace_notifier_data *node);
    int sw_unregister_trace_thermal_apic_exit_i(struct sw_trace_notifier_data *node);
#endif // KERNEL_VERSION(3,14,0) <= LINUX_VERSION_CODE
#if IS_ENABLED(CONFIG_ANDROID)
    #if KERNEL_VERSION(3,4,0) > LINUX_VERSION_CODE
        int sw_register_trace_wake_lock_i(struct sw_trace_notifier_data *node);
        int sw_unregister_trace_wake_lock_i(struct sw_trace_notifier_data *node);
        int sw_register_trace_wake_unlock_i(struct sw_trace_notifier_data *node);
        int sw_unregister_trace_wake_unlock_i(struct sw_trace_notifier_data *node);
    #else // KERNEL_VERSION(3,4,0) > LINUX_VERSION_CODE
        int sw_register_trace_wakeup_source_activate_i(struct sw_trace_notifier_data *node);
        int sw_unregister_trace_wakeup_source_activate_i(struct sw_trace_notifier_data *node);
        int sw_register_trace_wakeup_source_deactivate_i(struct sw_trace_notifier_data *node);
        int sw_unregister_trace_wakeup_source_deactivate_i(struct sw_trace_notifier_data *node);
    #endif // KERNEL_VERSION(3,4,0) > LINUX_VERSION_CODE
#endif // IS_ENABLED(CONFIG_ANDROID)
int sw_register_trace_workqueue_execution_i(struct sw_trace_notifier_data *node);
int sw_unregister_trace_workqueue_execution_i(struct sw_trace_notifier_data *node);
int sw_register_trace_sched_switch_i(struct sw_trace_notifier_data *node);
int sw_unregister_trace_sched_switch_i(struct sw_trace_notifier_data *node);
int sw_register_pm_notifier_i(struct sw_trace_notifier_data *node);
int sw_unregister_pm_notifier_i(struct sw_trace_notifier_data *node);
int sw_register_cpufreq_notifier_i(struct sw_trace_notifier_data *node);
int sw_unregister_cpufreq_notifier_i(struct sw_trace_notifier_data *node);
int sw_register_hotcpu_notifier_i(struct sw_trace_notifier_data *node);
int sw_unregister_hotcpu_notifier_i(struct sw_trace_notifier_data *node);
void sw_handle_sched_wakeup_i(struct sw_collector_data *node, int source_cpu, int target_cpu);
void sw_handle_timer_wakeup_helper_i(struct sw_collector_data *curr, struct sw_trace_notifier_data *node,
                                     pid_t tid);
void sw_handle_apic_timer_wakeup_i(struct sw_collector_data *node);
void sw_handle_workqueue_wakeup_helper_i(int cpu, struct sw_collector_data *node);
void sw_handle_sched_switch_helper_i(void);
void sw_tps_apic_i(int cpu);
void sw_tps_tps_i(int cpu);
void sw_tps_wakeup_i(int cpu);
void sw_tps_i(void);
void sw_tpf_i(int cpu, struct sw_trace_notifier_data *node);
void sw_process_fork_exit_helper_i(struct sw_collector_data *node, struct task_struct *task, bool is_fork);
void sw_produce_wakelock_msg_i(int cpu, struct sw_collector_data *node, const char *name,
                               int type, u64 timeout, int pid, int tid, const char *proc_name);
u64 sw_my_local_arch_irq_stats_cpu_i(void);

/*
 * The tracepoint probes.
 */
/*
 * The tracepoint handlers.
 */
void sw_handle_trace_notifier_i(struct sw_trace_notifier_data *node);
void sw_handle_trace_notifier_on_cpu_i(int cpu, struct sw_trace_notifier_data *node);
void sw_handle_reset_messages_i(struct sw_trace_notifier_data *node);

/* -------------------------------------------------
 * Variable definitions.
 * -------------------------------------------------
 */
/*
 * For overhead measurements.
 */
DECLARE_OVERHEAD_VARS(
	sw_handle_timer_wakeup_helper_i); /* for the "timer_expire"
					   *   family of probes
					   */
DECLARE_OVERHEAD_VARS(sw_handle_irq_wakeup_i); /* for IRQ wakeups */
DECLARE_OVERHEAD_VARS(sw_handle_sched_wakeup_i); /* for SCHED */
DECLARE_OVERHEAD_VARS(sw_tps_i); /* for TPS */
DECLARE_OVERHEAD_VARS(sw_tpf_i); /* for TPF */
DECLARE_OVERHEAD_VARS(sw_process_fork_exit_helper_i);
#if IS_ENABLED(CONFIG_ANDROID)
DECLARE_OVERHEAD_VARS(sw_handle_wakelock_i); /* for wake lock/unlock */
#endif /* CONFIG_ANDROID */
DECLARE_OVERHEAD_VARS(sw_handle_workqueue_wakeup_helper_i);
DECLARE_OVERHEAD_VARS(sw_handle_sched_switch_helper_i);
/*
 * Per-cpu wakeup counters.
 * Used to decide which wakeup event is the first to occur after a
 * core wakes up from a C-state.
 * Set to 'true' in TPS probe
 */
static DEFINE_PER_CPU(bool, sw_is_valid_wakeup_event) = { true };
/*
 * Per-cpu counts of the number of times the local APIC fired.
 * We need a separate count because some apic timer fires don't seem
 * to result in hrtimer/timer expires
 */
static DEFINE_PER_CPU(u64, sw_num_local_apic_timer_inters);
/*
 * Flag value to use to decide if the event is a valid wakeup event.
 * Set to 'false' in TPS probe.
 */
static bool sw_wakeup_event_flag = true;

#if IS_ENABLED(CONFIG_TRACEPOINTS)
/*
 * Scheduler-based polling emulation.
 */
static DEFINE_PER_CPU(unsigned long, sw_pcpu_polling_jiff);
#endif /* CONFIG_TRACEPOINTS */

pw_u16_t sw_min_polling_interval_msecs;

/*
 * IDs for supported tracepoints.
 */
enum sw_trace_id {
	SW_TRACE_ID_CPU_IDLE,
	SW_TRACE_ID_CPU_FREQUENCY,
	SW_TRACE_ID_IRQ_HANDLER_ENTRY,
	SW_TRACE_ID_TIMER_EXPIRE_ENTRY,
	SW_TRACE_ID_HRTIMER_EXPIRE_ENTRY,
	SW_TRACE_ID_SCHED_WAKEUP,
	SW_TRACE_ID_IPI,
	SW_TRACE_ID_SCHED_PROCESS_FORK,
	SW_TRACE_ID_SCHED_PROCESS_EXIT,
	SW_TRACE_ID_THERMAL_APIC_ENTRY,
	SW_TRACE_ID_THERMAL_APIC_EXIT,
	SW_TRACE_ID_WAKE_LOCK,
	SW_TRACE_ID_WAKE_UNLOCK,
	SW_TRACE_ID_WORKQUEUE_EXECUTE_START,
	SW_TRACE_ID_SCHED_SWITCH,
};

/*
 * IDs for supported notifiers.
 */
enum sw_notifier_id {
	SW_NOTIFIER_ID_SUSPEND, /* TODO: change name? */
	SW_NOTIFIER_ID_SUSPEND_ENTER,
	SW_NOTIFIER_ID_SUSPEND_EXIT,
	SW_NOTIFIER_ID_HIBERNATE,
	SW_NOTIFIER_ID_HIBERNATE_ENTER,
	SW_NOTIFIER_ID_HIBERNATE_EXIT,
	SW_NOTIFIER_ID_COUNTER_RESET,
	SW_NOTIFIER_ID_CPUFREQ,
	SW_NOTIFIER_ID_HOTCPU,
};

/*
 * Names for supported tracepoints. A tracepoint
 * 'name' consists of two strings: a "kernel" string
 * that is used to locate the tracepoint within the kernel
 * and an "abstract" string, that is used by Ring-3 to
 * specify which tracepoints to use during a collection.
 */
static const struct sw_trace_notifier_name s_trace_names[] = {
	[SW_TRACE_ID_CPU_IDLE] = { "cpu_idle", "CPU-IDLE" },
	[SW_TRACE_ID_CPU_FREQUENCY] = { "cpu_frequency", "CPU-FREQUENCY" },
	[SW_TRACE_ID_IRQ_HANDLER_ENTRY] = { "irq_handler_entry", "IRQ-ENTRY" },
	[SW_TRACE_ID_TIMER_EXPIRE_ENTRY] = { "timer_expire_entry",
					     "TIMER-ENTRY" },
	[SW_TRACE_ID_HRTIMER_EXPIRE_ENTRY] = { "hrtimer_expire_entry",
					       "HRTIMER-ENTRY" },
	[SW_TRACE_ID_SCHED_WAKEUP] = { "sched_wakeup", "SCHED-WAKEUP" },
	[SW_TRACE_ID_IPI] = { NULL, "IPI" },
	[SW_TRACE_ID_SCHED_PROCESS_FORK] = { "sched_process_fork",
					     "PROCESS-FORK" },
	[SW_TRACE_ID_SCHED_PROCESS_EXIT] = { "sched_process_exit",
					     "PROCESS-EXIT" },
#if KERNEL_VERSION(3, 14, 0) <= LINUX_VERSION_CODE
	[SW_TRACE_ID_THERMAL_APIC_ENTRY] = { "thermal_apic_entry",
					     "THERMAL-THROTTLE-ENTRY" },
	[SW_TRACE_ID_THERMAL_APIC_EXIT] = { "thermal_apic_exit",
					    "THERMAL-THROTTLE-EXIT" },
#endif /* KERNEL_VERSION(3, 14, 0) <= LINUX_VERSION_CODE  */
#if IS_ENABLED(CONFIG_ANDROID)
#if KERNEL_VERSION(3, 4, 0) > LINUX_VERSION_CODE
	[SW_TRACE_ID_WAKE_LOCK] = { "wake_lock", "WAKE-LOCK" },
	[SW_TRACE_ID_WAKE_UNLOCK] = { "wake_unlock", "WAKE-UNLOCK" },
#else /* KERNEL_VERSION(3, 4, 0) <= LINUX_VERSION_CODE */
	[SW_TRACE_ID_WAKE_LOCK] = { "wakeup_source_activate", "WAKE-LOCK" },
	[SW_TRACE_ID_WAKE_UNLOCK] = { "wakeup_source_deactivate",
				      "WAKE-UNLOCK" },
#endif
#endif
	[SW_TRACE_ID_WORKQUEUE_EXECUTE_START] = { "workqueue_execute_start",
						  "WORKQUEUE-START" },
	[SW_TRACE_ID_SCHED_SWITCH] = { "sched_switch", "CONTEXT-SWITCH" },
};

/*
 * Names for supported notifiers. A notifier
 * 'name' consists of two strings: an unused "kernel" string
 * and an "abstract" string, that is used by Ring-3 to
 * specify which notifiers to use during a collection.
 */
static const struct sw_trace_notifier_name s_notifier_names[] = {
	[SW_NOTIFIER_ID_SUSPEND] = { "suspend_notifier" /* don't care */,
				     "SUSPEND-NOTIFIER" },
	[SW_NOTIFIER_ID_SUSPEND_ENTER] = { NULL, "SUSPEND-ENTER" },
	[SW_NOTIFIER_ID_SUSPEND_EXIT] = { NULL, "SUSPEND-EXIT" },
	[SW_NOTIFIER_ID_HIBERNATE] = { "hibernate_notifier" /* don't care */,
				       "HIBERNATE-NOTIFIER" },
	[SW_NOTIFIER_ID_HIBERNATE_ENTER] = { NULL, "HIBERNATE-ENTER" },
	[SW_NOTIFIER_ID_HIBERNATE_EXIT] = { NULL, "HIBERNATE-EXIT" },
	[SW_NOTIFIER_ID_COUNTER_RESET] = { NULL, "COUNTER-RESET" },
	[SW_NOTIFIER_ID_CPUFREQ] = { "cpufreq_notifier" /* don't care */,
				     "CPUFREQ-NOTIFIER" },
	[SW_NOTIFIER_ID_HOTCPU] = { "hotcpu_notifier" /* don't care */,
				    "HOTCPU-NOTIFIER" },
};

#if IS_ENABLED(CONFIG_TRACEPOINTS)
/*
 * A list of supported tracepoints.
 */
static struct sw_trace_notifier_data s_trace_collector_lists[] = {
	{ SW_TRACE_COLLECTOR_TRACEPOINT, &s_trace_names[SW_TRACE_ID_CPU_IDLE],
	  &sw_register_trace_cpu_idle_i, &sw_unregister_trace_cpu_idle_i,
	  NULL },
	{ SW_TRACE_COLLECTOR_TRACEPOINT,
	  &s_trace_names[SW_TRACE_ID_CPU_FREQUENCY],
	  &sw_register_trace_cpu_frequency_i,
	  &sw_unregister_trace_cpu_frequency_i, NULL },
	{ SW_TRACE_COLLECTOR_TRACEPOINT,
	  &s_trace_names[SW_TRACE_ID_IRQ_HANDLER_ENTRY],
	  &sw_register_trace_irq_handler_entry_i,
	  &sw_unregister_trace_irq_handler_entry_i, NULL },
	{ SW_TRACE_COLLECTOR_TRACEPOINT,
	  &s_trace_names[SW_TRACE_ID_TIMER_EXPIRE_ENTRY],
	  &sw_register_trace_timer_expire_entry_i,
	  &sw_unregister_trace_timer_expire_entry_i, NULL },
	{ SW_TRACE_COLLECTOR_TRACEPOINT,
	  &s_trace_names[SW_TRACE_ID_HRTIMER_EXPIRE_ENTRY],
	  &sw_register_trace_hrtimer_expire_entry_i,
	  &sw_unregister_trace_hrtimer_expire_entry_i, NULL },
	{ SW_TRACE_COLLECTOR_TRACEPOINT,
	  &s_trace_names[SW_TRACE_ID_SCHED_WAKEUP],
	  &sw_register_trace_sched_wakeup_i,
	  &sw_unregister_trace_sched_wakeup_i, NULL },
	/* Placeholder for IPI -- no tracepoints associated with it! */
	{ SW_TRACE_COLLECTOR_TRACEPOINT, &s_trace_names[SW_TRACE_ID_IPI], NULL,
	  NULL, NULL },
	{ SW_TRACE_COLLECTOR_TRACEPOINT,
	  &s_trace_names[SW_TRACE_ID_SCHED_PROCESS_FORK],
	  &sw_register_trace_sched_process_fork_i,
	  &sw_unregister_trace_sched_process_fork_i, NULL },
	{ SW_TRACE_COLLECTOR_TRACEPOINT,
	  &s_trace_names[SW_TRACE_ID_SCHED_PROCESS_EXIT],
	  &sw_register_trace_sched_process_exit_i,
	  &sw_unregister_trace_sched_process_exit_i, NULL },
#if KERNEL_VERSION(3, 14, 0) <= LINUX_VERSION_CODE
	/*
	 * For thermal throttling.
	 * We probably only need one of either 'entry' or 'exit'. Use
	 * both, until we decide which one to keep. Note that
	 * tracepoint IDs for these, and subsequent tracepoints
	 * (e.g. 'wake_lock') will change once we've picked which
	 * one to use.
	 */
	{ SW_TRACE_COLLECTOR_TRACEPOINT,
	  &s_trace_names[SW_TRACE_ID_THERMAL_APIC_ENTRY],
	  &sw_register_trace_thermal_apic_entry_i,
	  &sw_unregister_trace_thermal_apic_entry_i, NULL },
	{ SW_TRACE_COLLECTOR_TRACEPOINT,
	  &s_trace_names[SW_TRACE_ID_THERMAL_APIC_EXIT],
	  &sw_register_trace_thermal_apic_exit_i,
	  &sw_unregister_trace_thermal_apic_exit_i, NULL },
#endif /* KERNEL_VERSION(3, 14, 0) <= LINUX_VERSION_CODE */
/* Wakelocks have multiple tracepoints, depending on kernel version */
#if IS_ENABLED(CONFIG_ANDROID)
#if KERNEL_VERSION(3, 4, 0) > LINUX_VERSION_CODE
	{ SW_TRACE_COLLECTOR_TRACEPOINT, &s_trace_names[SW_TRACE_ID_WAKE_LOCK],
	  &sw_register_trace_wake_lock_i, &sw_unregister_trace_wake_lock_i,
	  NULL },
	{ SW_TRACE_COLLECTOR_TRACEPOINT,
	  &s_trace_names[SW_TRACE_ID_WAKE_UNLOCK],
	  &sw_register_trace_wake_unlock_i, &sw_unregister_trace_wake_unlock_i,
	  NULL },
#else /* KERNEL_VERSION(3, 4, 0) <= LINUX_VERSION_CODE  */
	{ SW_TRACE_COLLECTOR_TRACEPOINT, &s_trace_names[SW_TRACE_ID_WAKE_LOCK],
	  &sw_register_trace_wakeup_source_activate_i,
	  &sw_unregister_trace_wakeup_source_activate_i, NULL },
	{ SW_TRACE_COLLECTOR_TRACEPOINT,
	  &s_trace_names[SW_TRACE_ID_WAKE_UNLOCK],
	  &sw_register_trace_wakeup_source_deactivate_i,
	  &sw_unregister_trace_wakeup_source_deactivate_i, NULL },
#endif /* KERNEL_VERSION(3, 4, 0) > LINUX_VERSION_CODE */
#endif /* CONFIG_ANDROID */
	{ SW_TRACE_COLLECTOR_TRACEPOINT,
	  &s_trace_names[SW_TRACE_ID_WORKQUEUE_EXECUTE_START],
	  &sw_register_trace_workqueue_execution_i,
	  &sw_unregister_trace_workqueue_execution_i, NULL },
	{ SW_TRACE_COLLECTOR_TRACEPOINT,
	  &s_trace_names[SW_TRACE_ID_SCHED_SWITCH],
	  &sw_register_trace_sched_switch_i,
	  &sw_unregister_trace_sched_switch_i, NULL },
};

/*
 * List of supported notifiers.
 */
static struct sw_trace_notifier_data s_notifier_collector_lists[] = {
	{ SW_TRACE_COLLECTOR_NOTIFIER,
	  &s_notifier_names[SW_NOTIFIER_ID_SUSPEND], &sw_register_pm_notifier_i,
	  &sw_unregister_pm_notifier_i, NULL, true /* always register */ },
	/* Placeholder for suspend enter/exit -- these will be called
	 * from within the pm notifier
	 */
	{ SW_TRACE_COLLECTOR_NOTIFIER,
	  &s_notifier_names[SW_NOTIFIER_ID_SUSPEND_ENTER], NULL, NULL, NULL },
	{ SW_TRACE_COLLECTOR_NOTIFIER,
	  &s_notifier_names[SW_NOTIFIER_ID_SUSPEND_EXIT], NULL, NULL, NULL },
	/* Placeholder for hibernate enter/exit -- these will be called
	 * from within the pm notifier
	 */
	{ SW_TRACE_COLLECTOR_NOTIFIER,
	  &s_notifier_names[SW_NOTIFIER_ID_HIBERNATE], NULL, NULL, NULL },
	{ SW_TRACE_COLLECTOR_NOTIFIER,
	  &s_notifier_names[SW_NOTIFIER_ID_HIBERNATE_ENTER], NULL, NULL, NULL },
	{ SW_TRACE_COLLECTOR_NOTIFIER,
	  &s_notifier_names[SW_NOTIFIER_ID_HIBERNATE_EXIT], NULL, NULL, NULL },
	{ SW_TRACE_COLLECTOR_NOTIFIER,
	  &s_notifier_names[SW_NOTIFIER_ID_COUNTER_RESET], NULL, NULL, NULL },
	{ SW_TRACE_COLLECTOR_NOTIFIER,
	  &s_notifier_names[SW_NOTIFIER_ID_CPUFREQ],
	  &sw_register_cpufreq_notifier_i, &sw_unregister_cpufreq_notifier_i },
};

/*
 * Special entry for CPU notifier (i.e. "hotplug" notifier)
 * We don't want these to be visible to the user.
 */
static struct sw_trace_notifier_data s_hotplug_notifier_data = {
	SW_TRACE_COLLECTOR_NOTIFIER,
	&s_notifier_names[SW_NOTIFIER_ID_HOTCPU],
	&sw_register_hotcpu_notifier_i,
	&sw_unregister_hotcpu_notifier_i,
	NULL,
	true /* always register */
};
#else /* !CONFIG_TRACEPOINTS */
/*
 * A list of supported tracepoints.
 */
static struct sw_trace_notifier_data s_trace_collector_lists[] = {
	/* EMPTY */};
/*
 * List of supported notifiers.
 */
static struct sw_trace_notifier_data s_notifier_collector_lists[] = {
	/* EMPTY */ };

#endif /* CONFIG_TRACEPOINTS */

/*
 * Macros to retrieve tracepoint and notifier IDs.
 */
#define GET_TRACE_ID_FROM_NODE(node) ((node)->name - s_trace_names)
#define GET_NOTIFIER_ID_FROM_NODE(node) ((node)->name - s_notifier_names)

#define GET_TRACE_NOTIFIER_ID(node)                                            \
	(int)(((node)->type == SW_TRACE_COLLECTOR_TRACEPOINT) ?                \
		      GET_TRACE_ID_FROM_NODE(node) :                           \
		      GET_NOTIFIER_ID_FROM_NODE(node))

/* -------------------------------------------------
 * Function definitions.
 * -------------------------------------------------
 */

/*
 * Retrieve a TSC value
 */
static inline u64 sw_tscval(void)
{
	unsigned int low, high;

	asm volatile("rdtsc" : "=a"(low), "=d"(high));
	return low | ((unsigned long long)high) << 32;
};

u64 sw_timestamp(void)
{
	struct timespec ts;

	getnstimeofday(&ts);
	return (ts.tv_sec * 1000000000ULL + ts.tv_nsec);
}

/*
 * Basically the same as arch/x86/kernel/irq.c --> "arch_irq_stat_cpu(cpu)"
 */
u64 sw_my_local_arch_irq_stats_cpu_i(void)
{
	u64 sum = 0;
	irq_cpustat_t *stats;
#ifdef __arm__
	int i = 0;
#endif
	BEGIN_LOCAL_IRQ_STATS_READ(stats);
	{
#ifndef __arm__
		sum += stats->__nmi_count;
#if IS_ENABLED(CONFIG_X86_LOCAL_APIC)
		sum += stats->apic_timer_irqs;
		sum += stats->irq_spurious_count;
#endif
#if KERNEL_VERSION(2, 6, 34) <= LINUX_VERSION_CODE
		sum += stats->x86_platform_ipis;
#endif /* 2,6,34 */
		sum += stats->apic_perf_irqs;
#if KERNEL_VERSION(3, 5, 0) <= LINUX_VERSION_CODE
		sum += stats->apic_irq_work_irqs;
#endif /* 3,5,0 */
#ifdef CONFIG_SMP
		sum += stats->irq_call_count;
		sum += stats->irq_resched_count;
		sum += stats->irq_tlb_count;
#endif
#ifdef CONFIG_X86_THERMAL_VECTOR
		sum += stats->irq_thermal_count;
#endif

#else
		sum += stats->__softirq_pending;
#ifdef CONFIG_SMP
		for (i = 0; i < NR_IPI; ++i)
			sum += stats->ipi_irqs[i];

#endif
#ifdef CONFIG_X86_MCE
		sum += stats->mce_exception_count;
		sum += stats->mce_poll_count;
#endif
#endif
	}
	END_LOCAL_IRQ_STATS_READ(stats);
	return sum;
};

/*
 * Generic tracepoint/notifier handling function.
 */
void sw_handle_trace_notifier_i(struct sw_trace_notifier_data *node)
{
	struct sw_collector_data *curr = NULL;

	if (!node)
		return;

	list_for_each_entry(curr, &node->list, list) {
		pw_pr_debug("DEBUG: handling message\n");
		sw_handle_per_cpu_msg(curr);
	}
};

/*
 * Generic tracepoint/notifier handling function.
 */
void sw_handle_trace_notifier_on_cpu_i(int cpu,
				       struct sw_trace_notifier_data *node)
{
	struct sw_collector_data *curr = NULL;

	if (!node)
		return;

	list_for_each_entry(curr, &node->list, list)
		sw_handle_per_cpu_msg_on_cpu(cpu, curr);

};

void sw_handle_reset_messages_i(struct sw_trace_notifier_data *node)
{
	struct sw_collector_data *curr = NULL;

	if (!node)
		return;

	list_for_each_entry(curr, &node->list, list) {
		pw_pr_debug("Handling message of unknown cpumask on cpu %d\n",
			    RAW_CPU());
		sw_schedule_work(&curr->cpumask, &sw_handle_per_cpu_msg, curr);
	}
}

/*
 * Tracepoint helpers.
 */

/*
 * TIMER wakeup handling function.
 */
static void sw_handle_timer_wakeup_i(struct sw_collector_data *node, pid_t pid,
			      pid_t tid)
{
	int cpu = RAW_CPU();
	sw_driver_msg_t *msg = GET_MSG_SLOT_FOR_CPU(node->msg, cpu,
						    node->per_msg_payload_size);
	/* char *dst_vals = (char *)(unsigned long)msg->p_payload; */
	char *dst_vals = msg->p_payload;

	/* msg->tsc = sw_timestamp(); */
	/* msg TSC assigned when msg is written to buffer */
	msg->cpuidx = cpu;

	/*
	 * TIMER handling ==> only return the pid, tid
	 */
	*((int *)dst_vals) = pid;
	dst_vals += sizeof(pid);
	*((int *)dst_vals) = tid;

	if (sw_produce_generic_msg(msg, SW_WAKEUP_ACTION_DIRECT))
		pw_pr_warn("WARNING: could NOT produce message!\n");

	pw_pr_debug("HANDLED timer expire for %d, %d\n", pid, tid);
};

/*
 * Helper function for {hr}timer expires. Required for overhead tracking.
 */
void sw_handle_timer_wakeup_helper_i(struct sw_collector_data *curr,
				     struct sw_trace_notifier_data *node,
				     pid_t tid)
{
	pid_t pid = -1;

	if (tid == 0)
		pid = 0;
	else {
		struct task_struct *task =
			pid_task(find_pid_ns(tid, &init_pid_ns), PIDTYPE_PID);
		if (likely(task))
			pid = task->tgid;
	}
	list_for_each_entry(curr, &node->list, list)
		sw_handle_timer_wakeup_i(curr, pid, tid);
};

/*
 * SCHED wakeup handling function.
 */
void sw_handle_sched_wakeup_i(struct sw_collector_data *node, int source_cpu,
			      int target_cpu)
{
	int cpu = source_cpu;
	sw_driver_msg_t *msg = GET_MSG_SLOT_FOR_CPU(node->msg, cpu,
						    node->per_msg_payload_size);
	/* char *dst_vals = (char *)(unsigned long)msg->p_payload; */
	char *dst_vals = msg->p_payload;

	/* msg->tsc = sw_timestamp(); */
	/* msg TSC assigned when msg is written to buffer */
	msg->cpuidx = source_cpu;

	/*
	 * sched handling ==> only return the source, target CPUs
	 */
	*((int *)dst_vals) = source_cpu;
	dst_vals += sizeof(source_cpu);
	*((int *)dst_vals) = target_cpu;

	if (sw_produce_generic_msg(msg, SW_WAKEUP_ACTION_NONE))
		pw_pr_warn("WARNING: could NOT produce message!\n");

};

/*
 * APIC timer wakeup
 */
void sw_handle_apic_timer_wakeup_i(struct sw_collector_data *node)
{
	/*
	 * Send an empty message back to Ring-3
	 */
	int cpu = RAW_CPU();
	sw_driver_msg_t *msg = GET_MSG_SLOT_FOR_CPU(node->msg, cpu,
						    node->per_msg_payload_size);
	/* char *dst_vals = (char *)(unsigned long)msg->p_payload; */

	/* msg->tsc = sw_timestamp(); */
	/* msg TSC assigned when msg is written to buffer */
	msg->cpuidx = cpu;

	if (sw_produce_generic_msg(msg, SW_WAKEUP_ACTION_DIRECT))
		pw_pr_warn("WARNING: could NOT produce message!\n");

	pw_pr_debug("HANDLED APIC timer wakeup for cpu = %d\n", cpu);
};

/*
 * Helper function for workqueue executions. Required for overhead tracking.
 */
void sw_handle_workqueue_wakeup_helper_i(int cpu,
					 struct sw_collector_data *node)
{
	sw_driver_msg_t *msg = GET_MSG_SLOT_FOR_CPU(node->msg, cpu,
						    node->per_msg_payload_size);

	/* msg->tsc = sw_timestamp(); */
	/* msg TSC assigned when msg is written to buffer */
	msg->cpuidx = cpu;

	/*
	 * Workqueue wakeup ==> empty message.
	 */
	if (sw_produce_generic_msg(msg, SW_WAKEUP_ACTION_DIRECT))
		pw_pr_error("WARNING: could NOT produce message!\n");
};

/*
 * Helper function for sched_switch. Required for overhead tracking.
 */
void sw_handle_sched_switch_helper_i(void)
{
	static struct sw_trace_notifier_data *node;

	if (unlikely(node == NULL)) {
		node = GET_COLLECTOR_TRACE_NODE(SW_TRACE_ID_SCHED_SWITCH);
		pw_pr_debug("SCHED SWITCH NODE = %p\n", node);
	}
	if (!node)
		return;

	preempt_disable();
	{
		struct sw_collector_data *curr;

		list_for_each_entry(curr, &node->list, list) {
			unsigned long curr_jiff = jiffies,
				      prev_jiff = curr->last_update_jiffies;
			unsigned long delta_msecs =
				jiffies_to_msecs(curr_jiff) -
				jiffies_to_msecs(prev_jiff);
			struct cpumask *mask = &curr->cpumask;
			u16 timeout = curr->info->sampling_interval_msec;

			if (!timeout)
				timeout = sw_min_polling_interval_msecs;

			/* Has there been enough time since the last
			 * collection point?
			 */
			if (delta_msecs < timeout)
				continue;

			/* Update timestamp and handle message */
			if (cpumask_test_cpu(
				    RAW_CPU(),
				    mask) /* This msg must be handled on
					   * the current CPU
					   */
			    ||
			    cpumask_empty(
				    mask) /* This msg may be handled by
					   * any CPU
					   */) {
				if (!CAS64(&curr->last_update_jiffies,
					   prev_jiff, curr_jiff)) {
					/*
					 * CAS failure should only be possible
					 * for messages that can be handled
					 * on any CPU, in which case it
					 * indicates a different CPU already
					 * handled this message.
					 */
					continue;
				}
				sw_handle_per_cpu_msg_no_sched(curr);
			}
		}
	}
	preempt_enable();
};

/*
 * Probe functions.
 */

/*
 * 1. TPS
 */

/*
 * Check IPI wakeups within the cpu_idle tracepoint.
 */
void sw_tps_apic_i(int cpu)
{
	static struct sw_trace_notifier_data *apic_timer_node;

	if (unlikely(apic_timer_node == NULL)) {
		apic_timer_node = GET_COLLECTOR_TRACE_NODE(SW_TRACE_ID_IPI);
		pw_pr_debug("apic NODE = %p\n", apic_timer_node);
	}
	if (apic_timer_node) {
		bool local_apic_timer_fired = false;
		u64 curr_num_local_apic = sw_my_local_arch_irq_stats_cpu_i();
		u64 *old_num_local_apic =
			&__get_cpu_var(sw_num_local_apic_timer_inters);

		if (*old_num_local_apic &&
		    (*old_num_local_apic != curr_num_local_apic)) {
			local_apic_timer_fired = true;
		}
		*old_num_local_apic = curr_num_local_apic;

		if (local_apic_timer_fired &&
		    SHOULD_PRODUCE_WAKEUP_SAMPLE(cpu)) {
			struct sw_collector_data *curr = NULL;

			list_for_each_entry(curr, &apic_timer_node->list,
					     list) {
				sw_handle_apic_timer_wakeup_i(curr);
			}
		}
	}
};

/*
 * Perform any user-defined tasks within the
 * cpu_idle tracepoint.
 */
void sw_tps_tps_i(int cpu)
{
	static struct sw_trace_notifier_data *tps_node;

	if (unlikely(tps_node == NULL)) {
		tps_node = GET_COLLECTOR_TRACE_NODE(SW_TRACE_ID_CPU_IDLE);
		pw_pr_debug("TPS NODE = %p\n", tps_node);
	}
	sw_handle_trace_notifier_i(tps_node);
};

/*
 * Perform any wakeup-related tasks within the
 * cpu_idle tracepoint.
 */
void sw_tps_wakeup_i(int cpu)
{
	/*
	 * For now, assume we will always have to
	 * do some wakeup book keeping. Later, we'll
	 * need to detect if the user requested wakeups.
	 */
	sw_wakeup_event_flag = false;
	RESET_VALID_WAKEUP_EVENT_COUNTER(cpu);
};

void sw_tps_i(void)
{
	/*
	 * Update: FIRST handle IPI wakeups
	 * THEN handle TPS
	 */
	int cpu = RAW_CPU();

	sw_tps_apic_i(cpu);
	sw_tps_tps_i(cpu);
	sw_tps_wakeup_i(cpu);
};

/*
 * 2. TPF
 */

/*
 * Helper function for overhead measurements.
 */
void sw_tpf_i(int cpu, struct sw_trace_notifier_data *node)
{
	sw_handle_trace_notifier_on_cpu_i((int)cpu, node);
};

#if IS_ENABLED(CONFIG_TRACEPOINTS)
DEFINE_PROBE_FUNCTION(PROBE_TPS_PARAMS)
{
#if KERNEL_VERSION(2, 6, 38) <= LINUX_VERSION_CODE
	if (state == PWR_EVENT_EXIT)
		return;
#endif
	DO_PER_CPU_OVERHEAD_FUNC(sw_tps_i);
};

DEFINE_PROBE_FUNCTION(PROBE_TPF_PARAMS)
{
#if KERNEL_VERSION(2, 6, 38) > LINUX_VERSION_CODE
	int cpu = RAW_CPU();
#endif /* version < 2.6.38 */
	static struct sw_trace_notifier_data *node;

	if (unlikely(node == NULL)) {
		node = GET_COLLECTOR_TRACE_NODE(SW_TRACE_ID_CPU_FREQUENCY);
		pw_pr_debug("NODE = %p\n", node);
	}
	DO_PER_CPU_OVERHEAD_FUNC(sw_tpf_i, (int)cpu, node);
};

/*
 * IRQ wakeup handling function.
 */
static void sw_handle_irq_wakeup_i(struct sw_collector_data *node, int irq)
{
	int cpu = RAW_CPU();
	sw_driver_msg_t *msg = GET_MSG_SLOT_FOR_CPU(node->msg, cpu,
						    node->per_msg_payload_size);
	/* char *dst_vals = (char *)(unsigned long)msg->p_payload; */
	char *dst_vals = msg->p_payload;

	/* msg->tsc = sw_timestamp(); */
	/* msg TSC assigned when msg is written to buffer */
	msg->cpuidx = cpu;

	/*
	 * IRQ handling ==> only return the irq number
	 */
	*((int *)dst_vals) = irq;

	if (sw_produce_generic_msg(msg, SW_WAKEUP_ACTION_DIRECT))
		pw_pr_warn("WARNING: could NOT produce message!\n");

};

/*
 * 3. IRQ handler entry
 */
DEFINE_PROBE_FUNCTION(PROBE_IRQ_PARAMS)
{
	int cpu = RAW_CPU();
	static struct sw_trace_notifier_data *node;

	struct sw_collector_data *curr = NULL;

	if (unlikely(node == NULL)) {
		node = GET_COLLECTOR_TRACE_NODE(SW_TRACE_ID_IRQ_HANDLER_ENTRY);
		pw_pr_debug("NODE = %p\n", node);
	}
	if (!node || !SHOULD_PRODUCE_WAKEUP_SAMPLE(cpu))
		return;

	list_for_each_entry(curr, &node->list, list)
		DO_PER_CPU_OVERHEAD_FUNC(sw_handle_irq_wakeup_i, curr, irq);

};

/*
 * 4. TIMER expire
 */
DEFINE_PROBE_FUNCTION(PROBE_TIMER_ARGS)
{
	int cpu = RAW_CPU();
	static struct sw_trace_notifier_data *node;

	struct sw_collector_data *curr = NULL;
	pid_t tid = GET_TIMER_THREAD_ID(t);

	if (unlikely(node == NULL)) {
		node = GET_COLLECTOR_TRACE_NODE(SW_TRACE_ID_TIMER_EXPIRE_ENTRY);
		pw_pr_debug("NODE = %p\n", node);
	}

	if (!node || !SHOULD_PRODUCE_WAKEUP_SAMPLE(cpu))
		return;

	DO_PER_CPU_OVERHEAD_FUNC(sw_handle_timer_wakeup_helper_i, curr, node,
				 tid);
};

/*
 * 5. HRTIMER expire
 */
DEFINE_PROBE_FUNCTION(PROBE_HRTIMER_PARAMS)
{
	int cpu = RAW_CPU();
	static struct sw_trace_notifier_data *node;
	struct sw_collector_data *curr = NULL;
	pid_t tid = GET_TIMER_THREAD_ID(hrt);

	if (unlikely(node == NULL)) {
		node = GET_COLLECTOR_TRACE_NODE(
			SW_TRACE_ID_HRTIMER_EXPIRE_ENTRY);
		pw_pr_debug("NODE = %p\n", node);
	}

	if (!node || !SHOULD_PRODUCE_WAKEUP_SAMPLE(cpu))
		return;

	DO_PER_CPU_OVERHEAD_FUNC(sw_handle_timer_wakeup_helper_i, curr, node,
				 tid);
};

/*
 * 6. SCHED wakeup
 */
DEFINE_PROBE_FUNCTION(PROBE_SCHED_WAKEUP_PARAMS)
{
	static struct sw_trace_notifier_data *node;
	struct sw_collector_data *curr = NULL;
	int target_cpu = task_cpu(task), source_cpu = RAW_CPU();
	/*
	 * "Self-sched" samples are "don't care".
	 */
	if (target_cpu == source_cpu)
		return;

	if (unlikely(node == NULL)) {
		node = GET_COLLECTOR_TRACE_NODE(SW_TRACE_ID_SCHED_WAKEUP);
		pw_pr_debug("NODE = %p\n", node);
	}
	/*
	 * Unlike other wakeup sources, we check the per-cpu flag
	 * of the TARGET cpu to decide if we should produce a sample.
	 */
	if (!node || !SHOULD_PRODUCE_WAKEUP_SAMPLE(target_cpu))
		return;

	list_for_each_entry(curr, &node->list, list) {
		/* sw_handle_sched_wakeup_i(curr, source_cpu, target_cpu); */
		DO_PER_CPU_OVERHEAD_FUNC(sw_handle_sched_wakeup_i, curr,
					 source_cpu, target_cpu);
	}
};

/*
 * 8. PROCESS fork
 */

/*
 * Helper for PROCESS fork, PROCESS exit
 */
void sw_process_fork_exit_helper_i(struct sw_collector_data *node,
				   struct task_struct *task, bool is_fork)
{
	int cpu = RAW_CPU();
	pid_t pid = task->tgid, tid = task->pid;
	const char *name = task->comm;
	sw_driver_msg_t *msg = GET_MSG_SLOT_FOR_CPU(node->msg, cpu,
						    node->per_msg_payload_size);
	char *dst_vals = msg->p_payload;

	msg->cpuidx = cpu;

	/*
	 * Fork/Exit ==> return pid, tid
	 * Fork ==> also return name
	 */
	*((int *)dst_vals) = pid;
	dst_vals += sizeof(pid);
	*((int *)dst_vals) = tid;
	dst_vals += sizeof(tid);
	if (is_fork)
		memcpy(dst_vals, name, SW_MAX_PROC_NAME_SIZE);


	if (sw_produce_generic_msg(msg, SW_WAKEUP_ACTION_DIRECT))
		pw_pr_warn("WARNING: could NOT produce message!\n");

	pw_pr_debug(
		"HANDLED process %s event for task: pid = %d, tid = %d, name = %s\n",
		is_fork ? "FORK" : "EXIT", pid, tid, name);
};

DEFINE_PROBE_FUNCTION(PROBE_PROCESS_FORK_PARAMS)
{
	static struct sw_trace_notifier_data *node;
	struct sw_collector_data *curr = NULL;

	if (unlikely(node == NULL)) {
		node = GET_COLLECTOR_TRACE_NODE(SW_TRACE_ID_SCHED_PROCESS_FORK);
		pw_pr_debug("NODE = %p\n", node);
	}
	if (!node)
		return;

	list_for_each_entry(curr, &node->list, list) {
		DO_PER_CPU_OVERHEAD_FUNC(sw_process_fork_exit_helper_i, curr,
					 child, true /* true ==> fork */);
	}
};

/*
 * 9. PROCESS exit
 */
DEFINE_PROBE_FUNCTION(PROBE_SCHED_PROCESS_EXIT_PARAMS)
{
	static struct sw_trace_notifier_data *node;
	struct sw_collector_data *curr = NULL;

	if (unlikely(node == NULL)) {
		node = GET_COLLECTOR_TRACE_NODE(SW_TRACE_ID_SCHED_PROCESS_EXIT);
		pw_pr_debug("NODE = %p\n", node);
	}
	if (!node)
		return;

	list_for_each_entry(curr, &node->list, list) {
		DO_PER_CPU_OVERHEAD_FUNC(sw_process_fork_exit_helper_i, curr,
					 task, false /* false ==> exit */);
	}
};

#if KERNEL_VERSION(3, 14, 0) <= LINUX_VERSION_CODE
/*
 * 10. THERMAL_APIC entry
 */
DEFINE_PROBE_FUNCTION(PROBE_THERMAL_APIC_ENTRY_PARAMS)
{
	int cpu = RAW_CPU();
	static struct sw_trace_notifier_data *node;

	if (unlikely(node == NULL)) {
		node = GET_COLLECTOR_TRACE_NODE(SW_TRACE_ID_THERMAL_APIC_ENTRY);
		pw_pr_debug("NODE = %p\n", node);
	}
	DO_PER_CPU_OVERHEAD_FUNC(sw_tpf_i, (int)cpu, node);
};

/*
 * 10. THERMAL_APIC exit
 */
DEFINE_PROBE_FUNCTION(PROBE_THERMAL_APIC_EXIT_PARAMS)
{
	int cpu = RAW_CPU();
	static struct sw_trace_notifier_data *node;

	if (unlikely(node == NULL)) {
		node = GET_COLLECTOR_TRACE_NODE(SW_TRACE_ID_THERMAL_APIC_EXIT);
		pw_pr_debug("NODE = %p\n", node);
	}
	DO_PER_CPU_OVERHEAD_FUNC(sw_tpf_i, (int)cpu, node);
};
#endif /* KERNEL_VERSION(3, 14, 0) <= LINUX_VERSION_CODE */

#if IS_ENABLED(CONFIG_ANDROID)
/*
 * 11. WAKE lock / WAKEUP source activate.
 */

/*
 * Helper function to produce wake lock/unlock messages.
 */
void sw_produce_wakelock_msg_i(int cpu, struct sw_collector_data *node,
			       const char *name, int type, u64 timeout, int pid,
			       int tid, const char *proc_name)
{
	sw_driver_msg_t *msg = GET_MSG_SLOT_FOR_CPU(node->msg, cpu,
						    node->per_msg_payload_size);
	char *dst_vals = msg->p_payload;

	msg->cpuidx = cpu;

	/*
	 * Protocol:
	 * wakelock_timeout, wakelock_type, wakelock_name,
	 * proc_pid, proc_tid, proc_name
	 */
	*((u64 *)dst_vals) = timeout;
	dst_vals += sizeof(timeout);
	*((int *)dst_vals) = type;
	dst_vals += sizeof(type);
	strncpy(dst_vals, name, SW_MAX_KERNEL_WAKELOCK_NAME_SIZE);
	dst_vals += SW_MAX_KERNEL_WAKELOCK_NAME_SIZE;

	*((int *)dst_vals) = pid;
	dst_vals += sizeof(pid);
	*((int *)dst_vals) = tid;
	dst_vals += sizeof(tid);
	strncpy(dst_vals, proc_name, SW_MAX_PROC_NAME_SIZE);
	dst_vals += SW_MAX_PROC_NAME_SIZE;

	if (sw_produce_generic_msg(msg, SW_WAKEUP_ACTION_DIRECT))
		pw_pr_warn("WARNING: could NOT produce message!\n");

};

/*
 * Helper function to handle wake lock/unlock callbacks.
 */
void sw_handle_wakelock_i(int cpu, struct sw_trace_notifier_data *node,
			  const char *name, int type, u64 timeout)
{
	int pid = PID(), tid = TID();
	const char *proc_name = NAME();
	struct sw_collector_data *curr = NULL;

	if (!node)
		return;


	list_for_each_entry(curr, &node->list, list) {
		sw_produce_wakelock_msg_i(cpu, curr, name, type, timeout, pid,
					  tid, proc_name);
	}
};

DEFINE_PROBE_FUNCTION(PROBE_WAKE_LOCK_PARAMS)
{
	int cpu = RAW_CPU();
	static struct sw_trace_notifier_data *node;
	enum sw_kernel_wakelock_type type = SW_WAKE_LOCK;
	u64 timeout = 0;
#if KERNEL_VERSION(3, 4, 0) > LINUX_VERSION_CODE
	const char *name = lock->name;
#endif

	if (unlikely(node == NULL)) {
		node = GET_COLLECTOR_TRACE_NODE(SW_TRACE_ID_WAKE_LOCK);
		pw_pr_debug("NODE = %p\n", node);
	}
#if KERNEL_VERSION(3, 4, 0) > LINUX_VERSION_CODE
	/*
	 * Was this wakelock acquired with a timeout i.e.
	 * is this an auto expire wakelock?
	 */
	if (lock->flags & (1U << 10)) {
		type = SW_WAKE_LOCK_TIMEOUT;
		timeout = jiffies_to_msecs(lock->expires - jiffies);
	}
#endif /* KERNEL_VERSION(3, 4, 0) > LINUX_VERSION_CODE */
	DO_PER_CPU_OVERHEAD_FUNC(sw_handle_wakelock_i, cpu, node, name,
				 (int)type, timeout);
};

/*
 * 11. WAKE unlock / WAKEUP source deactivate.
 */
DEFINE_PROBE_FUNCTION(PROBE_WAKE_UNLOCK_PARAMS)
{
	int cpu = RAW_CPU();
	static struct sw_trace_notifier_data *node;
	enum sw_kernel_wakelock_type type = SW_WAKE_UNLOCK;
#if KERNEL_VERSION(3, 4, 0) > LINUX_VERSION_CODE
	const char *name = lock->name;
#endif

	if (unlikely(node == NULL)) {
		node = GET_COLLECTOR_TRACE_NODE(SW_TRACE_ID_WAKE_UNLOCK);
		pw_pr_debug("NODE = %p\n", node);
	}
	DO_PER_CPU_OVERHEAD_FUNC(sw_handle_wakelock_i, cpu, node, name,
				 (int)type, 0 /*timeout*/);
};
#endif /* CONFIG_ANDROID */

/*
 * 12. WORKQUEUE
 */
DEFINE_PROBE_FUNCTION(PROBE_WORKQUEUE_PARAMS)
{
	int cpu = RAW_CPU();
	static struct sw_trace_notifier_data *node;
	struct sw_collector_data *curr = NULL;

	if (unlikely(node == NULL)) {
		node = GET_COLLECTOR_TRACE_NODE(
			SW_TRACE_ID_WORKQUEUE_EXECUTE_START);
		pw_pr_debug("NODE = %p\n", node);
	}

	if (!node || !SHOULD_PRODUCE_WAKEUP_SAMPLE(cpu))
		return;

	list_for_each_entry(curr, &node->list, list)
		DO_PER_CPU_OVERHEAD_FUNC(sw_handle_workqueue_wakeup_helper_i,
					 cpu, curr);

};

/*
 * 13. SCHED switch
 */
DEFINE_PROBE_FUNCTION(PROBE_SCHED_SWITCH_PARAMS)
{
	DO_PER_CPU_OVERHEAD_FUNC(sw_handle_sched_switch_helper_i);
};

/*
 * 1. SUSPEND notifier
 */
static void sw_send_pm_notification_i(int value)
{
	struct sw_driver_msg *msg = NULL;
	size_t buffer_len = sizeof(*msg) + sizeof(value);
	char *buffer = vmalloc(buffer_len);

	if (!buffer) {
		pw_pr_error(
			"couldn't allocate memory when sending suspend notification!\n");
		return;
	}
	msg = (struct sw_driver_msg *)buffer;
	msg->tsc = sw_timestamp();
	msg->cpuidx = RAW_CPU();
	msg->plugin_id = 0; /* "0" indicates a system message */
	msg->metric_id = 1; /* "1" indicates a suspend/resume message (TODO) */
	msg->msg_id = 0;
	/* don't care; TODO: use the 'msg_id' to encode the 'value'? */
	msg->payload_len = sizeof(value);
	msg->p_payload = buffer + sizeof(*msg);
	*((int *)msg->p_payload) = value;
	if (sw_produce_generic_msg(msg, SW_WAKEUP_ACTION_DIRECT))
		pw_pr_error("couldn't produce generic message!\n");

	vfree(buffer);
}

static u64 sw_pm_enter_tsc;
static bool sw_is_reset_i(void)
{
	/*
	 * TODO: rely on checking the IA32_FIXED_CTR2 instead?
	 */
	u64 curr_tsc = sw_tscval();
	bool is_reset = sw_pm_enter_tsc > curr_tsc;

	pw_pr_force("DEBUG: curr tsc = %llu, prev tsc = %llu, is reset = %s\n",
		    curr_tsc, sw_pm_enter_tsc, is_reset ? "true" : "false");

	return is_reset;
}

static void sw_probe_pm_helper_i(int id, int both_id, bool is_enter,
				 enum sw_pm_action action, enum sw_pm_mode mode)
{
	struct sw_trace_notifier_data *node = GET_COLLECTOR_NOTIFIER_NODE(id);
	struct sw_trace_notifier_data *both_node =
		GET_COLLECTOR_NOTIFIER_NODE(both_id);
	struct sw_trace_notifier_data *reset_node =
		GET_COLLECTOR_NOTIFIER_NODE(SW_NOTIFIER_ID_COUNTER_RESET);
	if (is_enter) {
		/*
		 * Entering HIBERNATION/SUSPEND
		 */
		sw_pm_enter_tsc = sw_tscval();
	} else {
		/*
		 * Exitting HIBERNATION/SUSPEND
		 */
		if (sw_is_reset_i() && reset_node)
			sw_handle_reset_messages_i(reset_node);

	}
	if (node)
		sw_handle_trace_notifier_i(node);

	if (both_node)
		sw_handle_trace_notifier_i(both_node);

	/* Send the suspend-resume notification */
	sw_send_pm_notification_i(SW_PM_VALUE(mode, action));
}

static bool sw_is_suspend_via_firmware(void)
{
#if KERNEL_VERSION(4, 4, 0) <= LINUX_VERSION_CODE
	/* 'pm_suspend_via_firmware' only available in kernel >= 4.4 */
	return pm_suspend_via_firmware();
#endif
	return true;
}

static int sw_probe_pm_notifier_i(struct notifier_block *block,
				  unsigned long state,
				  void *dummy)
{
	static const struct {
		enum sw_pm_action action;
		int node_id;
		int both_id;
		bool is_enter;
	} pm_data[PM_POST_RESTORE] = {
		[PM_HIBERNATION_PREPARE] = { SW_PM_ACTION_HIBERNATE_ENTER,
					     SW_NOTIFIER_ID_HIBERNATE_ENTER,
					     SW_NOTIFIER_ID_HIBERNATE, true },
		[PM_POST_HIBERNATION] = { SW_PM_ACTION_HIBERNATE_EXIT,
					  SW_NOTIFIER_ID_HIBERNATE_EXIT,
					  SW_NOTIFIER_ID_HIBERNATE, false },
		[PM_SUSPEND_PREPARE] = { SW_PM_ACTION_SUSPEND_ENTER,
					 SW_NOTIFIER_ID_SUSPEND_ENTER,
					 SW_NOTIFIER_ID_SUSPEND, true },
		[PM_POST_SUSPEND] = { SW_PM_ACTION_SUSPEND_EXIT,
				      SW_NOTIFIER_ID_SUSPEND_EXIT,
				      SW_NOTIFIER_ID_SUSPEND, false },
	};
	enum sw_pm_action action = pm_data[state].action;
	enum sw_pm_mode mode = sw_is_suspend_via_firmware() ?
				       SW_PM_MODE_FIRMWARE :
				       SW_PM_MODE_NONE;
	if (action != SW_PM_ACTION_NONE) {
		int node_id = pm_data[state].node_id,
		    both_id = pm_data[state].both_id;
		bool is_enter = pm_data[state].is_enter;

		sw_probe_pm_helper_i(node_id, both_id, is_enter, action, mode);
	} else {
		/* Not supported */
		pw_pr_error(
			"ERROR: unknown state %lu passed to SWA pm notifier!\n",
			state);
	}
	return NOTIFY_DONE;
}

static void sw_store_topology_change_i(enum cpu_action type,
				       int cpu, int core_id,
				       int pkg_id)
{
	struct sw_topology_node *node = sw_kmalloc(sizeof(*node), GFP_ATOMIC);

	if (!node) {
		pw_pr_error(
			"couldn't allocate a node for topology change tracking!\n");
		return;
	}
	node->change.timestamp = sw_timestamp();
	node->change.type = type;
	node->change.cpu = cpu;
	node->change.core = core_id;
	node->change.pkg = pkg_id;

	SW_LIST_ADD(&sw_topology_list, node, list);
	++sw_num_topology_entries;
}

#if KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE
int sw_probe_hotplug_notifier_i(struct notifier_block *block,
				unsigned long action, void *pcpu)
{
	unsigned int cpu = (unsigned long)pcpu;
	unsigned int pkg_id = topology_physical_package_id(cpu);
	unsigned int core_id = topology_core_id(cpu);

	switch (action) {
	case CPU_UP_PREPARE:
	case CPU_UP_PREPARE_FROZEN:
		/* CPU is coming online -- store top change */
		sw_store_topology_change_i(SW_CPU_ACTION_ONLINE_PREPARE, cpu,
					   core_id, pkg_id);
		pw_pr_debug(
			"DEBUG: SoC Watch has cpu %d (phys = %d, core = %d) preparing to come online at tsc = %llu! Current cpu = %d\n",
			cpu, pkg_id, core_id, sw_timestamp(), RAW_CPU());
		break;
	case CPU_ONLINE:
	case CPU_ONLINE_FROZEN:
		/* CPU is online -- first store top change
		 * then take BEGIN snapshot
		 */
		sw_store_topology_change_i(SW_CPU_ACTION_ONLINE, cpu, core_id,
					   pkg_id);
		sw_process_snapshot_on_cpu(SW_WHEN_TYPE_BEGIN, cpu);
		pw_pr_debug(
			"DEBUG: SoC Watch has cpu %d (phys = %d, core = %d) online at tsc = %llu! Current cpu = %d\n",
			cpu, pkg_id, core_id, sw_timestamp(), RAW_CPU());
		break;
	case CPU_DOWN_PREPARE:
	case CPU_DOWN_PREPARE_FROZEN:
		/* CPU is going offline -- take END snapshot */
		sw_process_snapshot_on_cpu(SW_WHEN_TYPE_END, cpu);
		pw_pr_debug(
			"DEBUG: SoC Watch has cpu %d preparing to go offline at tsc = %llu! Current cpu = %d\n",
			cpu, sw_timestamp(), RAW_CPU());
		break;
	case CPU_DEAD:
	case CPU_DEAD_FROZEN:
		/* CPU is offline -- store top change */
		sw_store_topology_change_i(SW_CPU_ACTION_OFFLINE, cpu, core_id,
					   pkg_id);
		pw_pr_debug(
			"DEBUG: SoC Watch has cpu %d offlined at tsc = %llu! Current cpu = %d\n",
			cpu, sw_timestamp(), RAW_CPU());
		break;
	default:
		break;
	}
	return NOTIFY_OK;
};
#else
static void sw_probe_cpuhp_helper_i(unsigned int cpu, enum cpu_action action)
{
	unsigned int pkg_id = topology_physical_package_id(cpu);
	unsigned int core_id = topology_core_id(cpu);

	switch (action) {
	case SW_CPU_ACTION_ONLINE_PREPARE:
		/* CPU is coming online -- store top change */
		sw_store_topology_change_i(action, cpu, core_id, pkg_id);
		break;
	case SW_CPU_ACTION_ONLINE:
		/* CPU is online -- first store top change
		 * then take BEGIN snapshot
		 */
		sw_store_topology_change_i(action, cpu, core_id, pkg_id);
		sw_process_snapshot_on_cpu(SW_WHEN_TYPE_BEGIN, cpu);
		break;
	case SW_CPU_ACTION_OFFLINE:
		/* CPU is preparing to go offline -- take
		 * END snapshot then store top change
		 */
		sw_process_snapshot_on_cpu(SW_WHEN_TYPE_END, cpu);
		sw_store_topology_change_i(action, cpu, core_id, pkg_id);
		break;
	default:
		break;
	}
}

static int sw_probe_cpu_offline_i(unsigned int cpu)
{
	pw_pr_debug("DEBUG: offline notification for cpu %u at %llu\n",
	       cpu, sw_tscval());
	sw_probe_cpuhp_helper_i(cpu, SW_CPU_ACTION_OFFLINE);
	return 0;
}

static int sw_probe_cpu_online_i(unsigned int cpu)
{
	pw_pr_debug("DEBUG: online notification for cpu %u at %llu\n", cpu,
	       sw_tscval());
	sw_probe_cpuhp_helper_i(cpu, SW_CPU_ACTION_ONLINE_PREPARE);
	sw_probe_cpuhp_helper_i(cpu, SW_CPU_ACTION_ONLINE);
	return 0;
}
#endif /* KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE  */

/*
 * 2. CPUFREQ notifier
 */
static int sw_probe_cpufreq_notifier_i(struct notifier_block *block,
				unsigned long state, void *data)
{
	struct cpufreq_freqs *freqs = data;
	static struct sw_trace_notifier_data *node;
	int cpu = freqs->cpu;

	if (state == CPUFREQ_PRECHANGE) {
		pw_pr_debug(
			"CPU %d reports a CPUFREQ_PRECHANGE for target CPU %d at TSC = %llu\n",
			RAW_CPU(), cpu, sw_timestamp());
		if (unlikely(node == NULL)) {
			node = GET_COLLECTOR_NOTIFIER_NODE(
				SW_NOTIFIER_ID_CPUFREQ);
			pw_pr_debug("NODE = %p\n", node);
		}
		/* Force an atomic context by disabling preemption */
		get_cpu();
		DO_PER_CPU_OVERHEAD_FUNC(sw_tpf_i, cpu, node);
		put_cpu();
	}
	return NOTIFY_DONE;
}

/*
 * 1. TPS.
 */
int sw_register_trace_cpu_idle_i(struct sw_trace_notifier_data *node)
{
#if KERNEL_VERSION(2, 6, 38) > LINUX_VERSION_CODE
	DO_REGISTER_SW_TRACEPOINT_PROBE(node, power_start,
					sw_probe_power_start_i);
#else /* kernel version >= 2.6.38 */
	DO_REGISTER_SW_TRACEPOINT_PROBE(node, cpu_idle, sw_probe_cpu_idle_i);
#endif /* KERNEL_VERSION(2, 6, 38) > LINUX_VERSION_CODE */
	return PW_SUCCESS;
};

int sw_unregister_trace_cpu_idle_i(struct sw_trace_notifier_data *node)
{
#if KERNEL_VERSION(2, 6, 38) > LINUX_VERSION_CODE
	DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, power_start,
					  sw_probe_power_start_i);
#else /* kernel version >= 2.6.38 */
	DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, cpu_idle, sw_probe_cpu_idle_i);
#endif /* KERNEL_VERSION(2, 6, 38) > LINUX_VERSION_CODE */
	return PW_SUCCESS;
};

/*
 * 2. TPF
 */
int sw_register_trace_cpu_frequency_i(struct sw_trace_notifier_data *node)
{
#if KERNEL_VERSION(2, 6, 38) > LINUX_VERSION_CODE
	DO_REGISTER_SW_TRACEPOINT_PROBE(node, power_frequency,
					sw_probe_power_frequency_i);
#else /* kernel version >= 2.6.38 */
	DO_REGISTER_SW_TRACEPOINT_PROBE(node, cpu_frequency,
					sw_probe_cpu_frequency_i);
#endif /* KERNEL_VERSION(2, 6, 38) > LINUX_VERSION_CODE */
	return PW_SUCCESS;
};

int sw_unregister_trace_cpu_frequency_i(struct sw_trace_notifier_data *node)
{
#if KERNEL_VERSION(2, 6, 38) > LINUX_VERSION_CODE
	DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, power_frequency,
					  sw_probe_power_frequency_i);
#else /* kernel version >= 2.6.38 */
	DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, cpu_frequency,
					  sw_probe_cpu_frequency_i);
#endif /* KERNEL_VERSION(2, 6, 38) > LINUX_VERSION_CODE */
	return PW_SUCCESS;
};

/*
 * 3. IRQ handler entry
 */
int sw_register_trace_irq_handler_entry_i(struct sw_trace_notifier_data *node)
{
	DO_REGISTER_SW_TRACEPOINT_PROBE(node, irq_handler_entry,
					sw_probe_irq_handler_entry_i);
	return PW_SUCCESS;
};

int sw_unregister_trace_irq_handler_entry_i(struct sw_trace_notifier_data *node)
{
	DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, irq_handler_entry,
					  sw_probe_irq_handler_entry_i);
	return PW_SUCCESS;
};

/*
 * 4. TIMER expire.
 */
int sw_register_trace_timer_expire_entry_i(struct sw_trace_notifier_data *node)
{
	DO_REGISTER_SW_TRACEPOINT_PROBE(node, timer_expire_entry,
					sw_probe_timer_expire_entry_i);
	return PW_SUCCESS;
};

int sw_unregister_trace_timer_expire_entry_i(struct sw_trace_notifier_data
					     *node)
{
	DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, timer_expire_entry,
					  sw_probe_timer_expire_entry_i);
	return PW_SUCCESS;
};

/*
 * 5. HRTIMER expire.
 */
int sw_register_trace_hrtimer_expire_entry_i(struct sw_trace_notifier_data
					     *node)
{
	DO_REGISTER_SW_TRACEPOINT_PROBE(node, hrtimer_expire_entry,
					sw_probe_hrtimer_expire_entry_i);
	return PW_SUCCESS;
};

int sw_unregister_trace_hrtimer_expire_entry_i(
	struct sw_trace_notifier_data *node)
{
	DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, hrtimer_expire_entry,
					  sw_probe_hrtimer_expire_entry_i);
	return PW_SUCCESS;
};

/*
 * 6. SCHED wakeup
 */
int sw_register_trace_sched_wakeup_i(struct sw_trace_notifier_data *node)
{
	DO_REGISTER_SW_TRACEPOINT_PROBE(node, sched_wakeup,
					sw_probe_sched_wakeup_i);
	return PW_SUCCESS;
};

int sw_unregister_trace_sched_wakeup_i(struct sw_trace_notifier_data *node)
{
	DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, sched_wakeup,
					  sw_probe_sched_wakeup_i);
	return PW_SUCCESS;
};

/*
 * 8. PROCESS fork
 */
int sw_register_trace_sched_process_fork_i(struct sw_trace_notifier_data *node)
{
	DO_REGISTER_SW_TRACEPOINT_PROBE(node, sched_process_fork,
					sw_probe_sched_process_fork_i);
	return PW_SUCCESS;
};

int sw_unregister_trace_sched_process_fork_i(struct sw_trace_notifier_data
					     *node)
{
	DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, sched_process_fork,
					  sw_probe_sched_process_fork_i);
	return PW_SUCCESS;
};

/*
 * 9. PROCESS exit
 */
int sw_register_trace_sched_process_exit_i(struct sw_trace_notifier_data *node)
{
	DO_REGISTER_SW_TRACEPOINT_PROBE(node, sched_process_exit,
					sw_probe_sched_process_exit_i);
	return PW_SUCCESS;
};

int sw_unregister_trace_sched_process_exit_i(struct sw_trace_notifier_data
					     *node)
{
	DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, sched_process_exit,
					  sw_probe_sched_process_exit_i);
	return PW_SUCCESS;
};

/*
 * 10. THERMAL_APIC entry
 */
#if KERNEL_VERSION(3, 14, 0) <= LINUX_VERSION_CODE
int sw_register_trace_thermal_apic_entry_i(struct sw_trace_notifier_data *node)
{
	DO_REGISTER_SW_TRACEPOINT_PROBE(node, thermal_apic_entry,
					sw_probe_thermal_apic_entry_i);
	return PW_SUCCESS;
};

int sw_unregister_trace_thermal_apic_entry_i(struct sw_trace_notifier_data
					     *node)
{
	DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, thermal_apic_entry,
					  sw_probe_thermal_apic_entry_i);
	return PW_SUCCESS;
};

/*
 * 10. THERMAL_APIC exit
 */
int sw_register_trace_thermal_apic_exit_i(struct sw_trace_notifier_data *node)
{
	DO_REGISTER_SW_TRACEPOINT_PROBE(node, thermal_apic_exit,
					sw_probe_thermal_apic_exit_i);
	return PW_SUCCESS;
};

int sw_unregister_trace_thermal_apic_exit_i(struct sw_trace_notifier_data *node)
{
	DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, thermal_apic_exit,
					  sw_probe_thermal_apic_exit_i);
	return PW_SUCCESS;
};
#endif /* KERNEL_VERSION(3, 14, 0) <= LINUX_VERSION_CODE */

/*
 * 11. WAKE lock / WAKEUP source activate.
 */
#if IS_ENABLED(CONFIG_ANDROID)
#if KERNEL_VERSION(3, 4, 0) > LINUX_VERSION_CODE
int sw_register_trace_wake_lock_i(struct sw_trace_notifier_data *node)
{
	DO_REGISTER_SW_TRACEPOINT_PROBE(node, wake_lock, sw_probe_wake_lock_i);
	return PW_SUCCESS;
};

int sw_unregister_trace_wake_lock_i(struct sw_trace_notifier_data *node)
{
	DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, wake_lock,
					  sw_probe_wake_lock_i);
	return PW_SUCCESS;
};
#else /* KERNEL_VERSION(3, 4, 0) > LINUX_VERSION_CODE */
int sw_register_trace_wakeup_source_activate_i(
	struct sw_trace_notifier_data *node)
{
	DO_REGISTER_SW_TRACEPOINT_PROBE(node, wakeup_source_activate,
					sw_probe_wakeup_source_activate_i);
	return PW_SUCCESS;
};

int sw_unregister_trace_wakeup_source_activate_i(
	struct sw_trace_notifier_data *node)
{
	DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, wakeup_source_activate,
					  sw_probe_wakeup_source_activate_i);
	return PW_SUCCESS;
};
#endif /* KERNEL_VERSION(3, 4, 0) > LINUX_VERSION_CODE */

/*
 * 11. WAKE unlock / WAKEUP source deactivate.
 */
#if KERNEL_VERSION(3, 4, 0) > LINUX_VERSION_CODE
int sw_register_trace_wake_unlock_i(struct sw_trace_notifier_data *node)
{
	DO_REGISTER_SW_TRACEPOINT_PROBE(node, wake_unlock,
					sw_probe_wake_unlock_i);
	return PW_SUCCESS;
};

int sw_unregister_trace_wake_unlock_i(struct sw_trace_notifier_data *node)
{
	DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, wake_unlock,
					  sw_probe_wake_unlock_i);
	return PW_SUCCESS;
};

#else /* KERNEL_VERSION(3, 4, 0) > LINUX_VERSION_CODE */
int sw_register_trace_wakeup_source_deactivate_i(
	struct sw_trace_notifier_data *node)
{
	DO_REGISTER_SW_TRACEPOINT_PROBE(node, wakeup_source_deactivate,
					sw_probe_wakeup_source_deactivate_i);
	return PW_SUCCESS;
};

int sw_unregister_trace_wakeup_source_deactivate_i(
	struct sw_trace_notifier_data *node)
{
	DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, wakeup_source_deactivate,
					  sw_probe_wakeup_source_deactivate_i);
	return PW_SUCCESS;
};
#endif /*  KERNEL_VERSION(3, 4, 0) > LINUX_VERSION_CODE */
#endif /* CONFIG_ANDROID */

/*
 * 12. WORKQUEUE execution.
 */
int sw_register_trace_workqueue_execution_i(struct sw_trace_notifier_data *node)
{
#if KERNEL_VERSION(2, 6, 35) >= LINUX_VERSION_CODE
	DO_REGISTER_SW_TRACEPOINT_PROBE(node, workqueue_execution,
					sw_probe_workqueue_execution_i);
#else
	DO_REGISTER_SW_TRACEPOINT_PROBE(node, workqueue_execute_start,
					sw_probe_workqueue_execute_start_i);
#endif
	return PW_SUCCESS;
};

int sw_unregister_trace_workqueue_execution_i(
	struct sw_trace_notifier_data *node)
{
#if KERNEL_VERSION(2, 6, 35) >= LINUX_VERSION_CODE
	DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, workqueue_execution,
					  sw_probe_workqueue_execution_i);
#else
	DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, workqueue_execute_start,
					  sw_probe_workqueue_execute_start_i);
#endif
	return PW_SUCCESS;
};

/*
 * 13. SCHED switch
 */
int sw_register_trace_sched_switch_i(struct sw_trace_notifier_data *node)
{
	/*
	 * Set polling tick time, in jiffies.
	 * Used by the context switch tracepoint to decide
	 * if enough time has elapsed since the last
	 * collection point to read resources again.
	 */
	{
		int cpu = 0;

		for_each_present_cpu(cpu)
			*(&per_cpu(sw_pcpu_polling_jiff, cpu)) = jiffies;

	}
	DO_REGISTER_SW_TRACEPOINT_PROBE(node, sched_switch,
					sw_probe_sched_switch_i);
	return PW_SUCCESS;
};

int sw_unregister_trace_sched_switch_i(struct sw_trace_notifier_data *node)
{
	DO_UNREGISTER_SW_TRACEPOINT_PROBE(node, sched_switch,
					  sw_probe_sched_switch_i);
	return PW_SUCCESS;
};

/*
 * Notifier register/unregister functions.
 */

/*
 * 1. SUSPEND notifier.
 */
static struct notifier_block sw_pm_notifier = {
	.notifier_call = &sw_probe_pm_notifier_i,
};

int sw_register_pm_notifier_i(struct sw_trace_notifier_data *node)
{
	register_pm_notifier(&sw_pm_notifier);
	return PW_SUCCESS;
};

int sw_unregister_pm_notifier_i(struct sw_trace_notifier_data *node)
{
	unregister_pm_notifier(&sw_pm_notifier);
	return PW_SUCCESS;
};

/*
 * 2. CPUFREQ notifier.
 */
static struct notifier_block sw_cpufreq_notifier = {
	.notifier_call = &sw_probe_cpufreq_notifier_i,
};

int sw_register_cpufreq_notifier_i(struct sw_trace_notifier_data *node)
{
	cpufreq_register_notifier(&sw_cpufreq_notifier,
				  CPUFREQ_TRANSITION_NOTIFIER);
	return PW_SUCCESS;
};

int sw_unregister_cpufreq_notifier_i(struct sw_trace_notifier_data *node)
{
	cpufreq_unregister_notifier(&sw_cpufreq_notifier,
				    CPUFREQ_TRANSITION_NOTIFIER);
	return PW_SUCCESS;
};

#if KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE
/*
 * 3. CPU hot plug notifier.
 */
struct notifier_block sw_cpu_hotplug_notifier = {
	.notifier_call = &sw_probe_hotplug_notifier_i,
};

int sw_register_hotcpu_notifier_i(struct sw_trace_notifier_data *node)
{
	register_hotcpu_notifier(&sw_cpu_hotplug_notifier);
	return PW_SUCCESS;
};

int sw_unregister_hotcpu_notifier_i(struct sw_trace_notifier_data *node)
{
	unregister_hotcpu_notifier(&sw_cpu_hotplug_notifier);
	return PW_SUCCESS;
};

#else /* KERNEL_VERSION(4, 10, 0) <= LINUX_VERSION_CODE */
static int sw_cpuhp_state = -1;
int sw_register_hotcpu_notifier_i(struct sw_trace_notifier_data *node)
{
	sw_cpuhp_state = cpuhp_setup_state_nocalls(CPUHP_AP_ONLINE_DYN,
						   "socwatch:online",
						   &sw_probe_cpu_online_i,
						   &sw_probe_cpu_offline_i);
	if (sw_cpuhp_state < 0) {
		pw_pr_error("couldn't register socwatch hotplug callbacks!\n");
		return -EIO;
	}
	return 0;
};

int sw_unregister_hotcpu_notifier_i(struct sw_trace_notifier_data *node)
{
	if (sw_cpuhp_state >= 0)
		cpuhp_remove_state_nocalls((enum cpuhp_state)sw_cpuhp_state);

	return 0;
};
#endif /* KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE */

/*
 * Tracepoint extraction routines.
 * Required for newer kernels (>=3.15)
 */
#if KERNEL_VERSION(3, 15, 0) <= LINUX_VERSION_CODE
static void sw_extract_tracepoint_callback(struct tracepoint *tp, void *priv)
{
	struct sw_trace_notifier_data *node = NULL;
	int i = 0;
	int *numStructsFound = (int *)priv;

	if (*numStructsFound == NUM_VALID_TRACEPOINTS) {
		/*
		 * We've found all the tracepoints we need.
		 */
		return;
	}
	if (tp) {
		FOR_EACH_TRACEPOINT_NODE(i, node)
		{
			if (node->tp == NULL && node->name) {
				const char *name =
					sw_get_trace_notifier_kernel_name(node);
				if (name && !strcmp(tp->name, name)) {
					node->tp = tp;
					++*numStructsFound;
					pw_pr_debug("OK, found TP %s\n",
						    tp->name);
				}
			}
		}
	}
};
#endif /* KERNEL_VERSION(3, 15, 0) <= LINUX_VERSION_CODE */
#endif /* CONFIG_TRACEPOINTS */

/*
 * Retrieve the list of tracepoint structs to use
 * when registering and unregistering tracepoint handlers.
 */
int sw_extract_trace_notifier_providers(void)
{
#if KERNEL_VERSION(3, 15, 0) <= LINUX_VERSION_CODE  &&			\
	IS_ENABLED(CONFIG_TRACEPOINTS)
	int numCallbacks = 0;

	for_each_kernel_tracepoint(&sw_extract_tracepoint_callback,
				   &numCallbacks);
	/*
	 * Did we get the complete list?
	 */
	if (numCallbacks != NUM_VALID_TRACEPOINTS)
		pw_pr_warn(
		       "WARNING : Could NOT find tracepoint structs for some tracepoints !\n");
#endif /* KERNEL_VERSION(3, 15, 0) <= LINUX_VERSION_CODE */
	return PW_SUCCESS;
};

void sw_reset_trace_notifier_providers(void)
{
	/*
	 * Reset the wakeup flag. Not strictly required if we aren't probing
	 * any of the wakeup tracepoints.
	 */
	{
		int cpu = 0;

		for_each_online_cpu(cpu)
			RESET_VALID_WAKEUP_EVENT_COUNTER(cpu);
	}
	/*
	 * Reset the wakeup event flag. Not strictly required if we
	 * aren't probing any of the wakeup tracepoints. Will be reset
	 * in the power_start tracepoint if user requested a c-state
	 * collection.
	 */
	sw_wakeup_event_flag = true;
};

void sw_print_trace_notifier_provider_overheads(void)
{
	PRINT_CUMULATIVE_OVERHEAD_PARAMS(sw_tps_i, "TPS");
	PRINT_CUMULATIVE_OVERHEAD_PARAMS(sw_tpf_i, "TPF");
	PRINT_CUMULATIVE_OVERHEAD_PARAMS(sw_handle_irq_wakeup_i, "IRQ");
	PRINT_CUMULATIVE_OVERHEAD_PARAMS(sw_handle_timer_wakeup_helper_i,
					 "TIMER_EXPIRE");
	PRINT_CUMULATIVE_OVERHEAD_PARAMS(sw_handle_sched_wakeup_i,
					 "SCHED WAKEUP");
	PRINT_CUMULATIVE_OVERHEAD_PARAMS(sw_process_fork_exit_helper_i,
					 "PROCESS FORK/EXIT");
#if IS_ENABLED(CONFIG_ANDROID)
	PRINT_CUMULATIVE_OVERHEAD_PARAMS(sw_handle_wakelock_i,
					 "WAKE LOCK/UNLOCK");
#endif /* CONFIG_ANDROID */
	PRINT_CUMULATIVE_OVERHEAD_PARAMS(sw_handle_workqueue_wakeup_helper_i,
					 "WORKQUEUE");
	PRINT_CUMULATIVE_OVERHEAD_PARAMS(sw_handle_sched_switch_helper_i,
					 "SCHED SWITCH");
};

/*
 * Add all trace/notifier providers.
 */
int sw_add_trace_notifier_providers(void)
{
	struct sw_trace_notifier_data *node = NULL;
	int i = 0;

	FOR_EACH_TRACEPOINT_NODE(i, node)
	{
		if (sw_register_trace_notify_provider(node)) {
			pw_pr_error("ERROR : couldn't add a trace provider!\n");
			return -EIO;
		}
	}
	FOR_EACH_NOTIFIER_NODE(i, node)
	{
		if (sw_register_trace_notify_provider(node)) {
			pw_pr_error(
				"ERROR: couldn't add a notifier provider !\n");
			return -EIO;
		}
	}
	/*
	 * Add the cpu hot plug notifier.
	 */
	{
		if (sw_register_trace_notify_provider(
			    &s_hotplug_notifier_data)) {
			pw_pr_error(
				"ERROR : couldn't add cpu notifier provider!\n");
			return -EIO;
		}
	}
	return PW_SUCCESS;
}

/*
 * Remove previously added providers.
 */
void sw_remove_trace_notifier_providers(void)
{ /* NOP */
}
