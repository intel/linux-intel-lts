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

#ifndef __SW_DATA_STRUCTS_H__
#define __SW_DATA_STRUCTS_H__

/*
 * Taken from 'sw_driver'
 * TODO: move to separate file?
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/cpumask.h>
#include <linux/hrtimer.h>
#include <linux/fs.h>      /* inode */
#include <linux/device.h>  /* class_create */
#include <linux/cdev.h>    /* cdev_alloc */
#include <linux/vmalloc.h> /* vmalloc */
#include <linux/sched.h>   /* TASK_INTERRUPTIBLE */
#include <linux/wait.h>    /* wait_event_interruptible */
#include <linux/pci.h>     /* pci_get_bus_and_slot */
#include <linux/version.h> /* LINUX_VERSION_CODE */
#include <linux/sfi.h>     /* For SFI F/W version */
#include <asm/hardirq.h>
#include <linux/cpufreq.h>
#include <asm/local.h>     /* local_t */
#include <linux/hardirq.h> /* "in_atomic" */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 12, 0)
    #include <asm/uaccess.h>   /* copy_to_user */
#else
    #include <linux/uaccess.h>   /* copy_to_user */
#endif /* LINUX_VERSION_CODE */

#ifdef CONFIG_X86_WANT_INTEL_MID
    #include <asm/intel-mid.h>
#endif /* CONFIG_X86_WANT_INTEL_MID */
/*
 * End taken from sw_driver
 */

#include "sw_structs.h"
#include "sw_ioctl.h"
#include "sw_list.h"

/* ******************************************
 * Compile time constants
 * ******************************************
 */
#define GET_POLLED_CPU() (sw_max_num_cpus)
#define CAS32(p, o, n) (cmpxchg((p), (o), (n)) == (o))

/* ******************************************
 * Function declarations.
 * ******************************************
 */
/*
 * Output to user.
 */
unsigned long sw_copy_to_user(char __user *dst, char *src, size_t bytes_to_copy);
bool sw_check_output_buffer_params(void __user *buffer, size_t bytes_to_read,
	size_t buff_size);
/*
 * smp call function.
 */
void sw_schedule_work(const struct cpumask *mask, void (*work)(void *),
	void *data);
/*
 * Save IRQ flags and retrieve cpu number.
 */
int sw_get_cpu(unsigned long *flags);
/*
 * Restore IRQ flags.
 */
void sw_put_cpu(unsigned long flags);
/*
 * Set module scope for cpu frequencies.
 */
int sw_set_module_scope_for_cpus(void);
/*
 * reset module scope for cpu frequencies.
 */
int sw_reset_module_scope_for_cpus(void);
/*
 * Setup p-unit/pmc telemetry
 */
int sw_setup_telem(u64 addrs[3]);
/*
 * Tear down p-unit/pmc telemetry
 */
void sw_destroy_telem(void);

#endif /* __SW_DATA_STRUCTS_H__ */
