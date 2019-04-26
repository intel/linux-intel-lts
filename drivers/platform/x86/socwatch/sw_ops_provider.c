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
#include <linux/errno.h>
#include <linux/pci.h> /* "pci_get_domain_bus_and_slot" */
#include <linux/delay.h> /* "udelay" */
#include <asm/msr.h>
#ifdef CONFIG_RPMSG_IPC
	#include <asm/intel_mid_rpmsg.h>
#endif /* CONFIG_RPMSG_IPC */

#include "sw_types.h"
#include "sw_kernel_defines.h"
#include "sw_hardware_io.h"
#include "sw_telem.h"
#include "sw_ops_provider.h"

/*
 * Compile time constants.
 */
/*
 * Should we be doing 'direct' PCI reads and writes?
 * '1' ==> YES, call "pci_{read,write}_config_dword()" directly
 * '0' ==> NO, Use the "intel_mid_msgbus_{read32,write32}_raw()" API
 *		(defined in 'intel_mid_pcihelpers.c')
 */
#define DO_DIRECT_PCI_READ_WRITE 0
#if !IS_ENABLED(CONFIG_ANDROID) || !defined(CONFIG_X86_WANT_INTEL_MID)
    /*
     * 'intel_mid_pcihelpers.h' is probably not present -- force
     * direct PCI calls in this case.
     */
	#undef DO_DIRECT_PCI_READ_WRITE
	#define DO_DIRECT_PCI_READ_WRITE  1
#endif
#if !DO_DIRECT_PCI_READ_WRITE
	#include <asm/intel_mid_pcihelpers.h>
#endif

#define SW_PCI_MSG_CTRL_REG 0x000000D0
#define SW_PCI_MSG_DATA_REG 0x000000D4

/*
 *  NUM_RETRY & USEC_DELAY are used in PCH Mailbox (sw_read_pch_mailbox_info_i).
 *  Tested on KBL + SPT-LP. May need to revisit.
 */
#define NUM_RETRY  100
#define USEC_DELAY 100

#define EXTCNF_CTRL 0xF00 /* offset for hw semaphore. */
#define FWSM_CTRL 0x5B54 /* offset for fw semaphore */
#define GBE_CTRL_OFFSET 0x34 /* GBE LPM offset */

#define IS_HW_SEMAPHORE_SET(data) (data & (pw_u64_t)(0x1 << 6))
#define IS_FW_SEMAPHORE_SET(data) (data & (pw_u64_t)0x1)
/*
 * Number of retries for mailbox configuration
 */
#define MAX_MAILBOX_ITERS 100

/*
 * Local data structures.
 */
/*
 * TODO: separate into H/W and S/W IO?
 */
enum sw_io_type {
	SW_IO_MSR		= 0,
	SW_IO_IPC		= 1,
	SW_IO_MMIO		= 2,
	SW_IO_PCI		= 3,
	SW_IO_CONFIGDB		= 4,
	SW_IO_TRACE_ARGS	= 5,
	SW_IO_WAKEUP		= 6,
	SW_IO_SOCPERF		= 7,
	SW_IO_PROC_NAME		= 8,
	SW_IO_IRQ_NAME		= 9,
	SW_IO_WAKELOCK		= 10,
	SW_IO_TELEM		= 11,
	SW_IO_PCH_MAILBOX	= 12,
	SW_IO_MAILBOX		= 13,
	SW_IO_MAX		= 14,
};

/*
 * "io_remapped" values for HW and FW semaphores
 */
static struct {
	volatile void __iomem *hw_semaphore;
	volatile void __iomem *fw_semaphore;
} s_gbe_semaphore = {NULL, NULL};

/*
 * Function declarations.
 */
/*
 * Exported by the SOCPERF driver.
 */
extern void __weak SOCPERF_Read_Data3(void *data_buffer);

/*
 * Init functions.
 */
int sw_ipc_mmio_descriptor_init_func_i(struct sw_driver_io_descriptor *descriptor);
int sw_pch_mailbox_descriptor_init_func_i(struct sw_driver_io_descriptor *descriptor);
int sw_mailbox_descriptor_init_func_i(struct sw_driver_io_descriptor *descriptor);

/*
 * Read functions.
 */
void sw_read_msr_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);
void sw_read_ipc_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);
void sw_read_mmio_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);
void sw_read_pch_mailbox_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);
void sw_read_mailbox_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);
void sw_read_pci_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);
void sw_read_configdb_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);
void sw_read_socperf_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);

/*
 * Write functions.
 */
void sw_write_msr_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);
void sw_write_ipc_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);
void sw_write_mmio_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);
void sw_write_mailbox_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);
void sw_write_pci_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);
void sw_write_configdb_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);
void sw_write_trace_args_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);
void sw_write_wakeup_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);
void sw_write_socperf_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);

/*
 * Print functions.
 */
int sw_print_msr_io_descriptor(const struct sw_driver_io_descriptor *descriptor);

/*
 * Reset functions -- equal but opposite of init.
 */
int sw_ipc_mmio_descriptor_reset_func_i(const struct sw_driver_io_descriptor *descriptor);
int sw_pch_mailbox_descriptor_reset_func_i(const struct sw_driver_io_descriptor *descriptor);
int sw_mailbox_descriptor_reset_func_i(const struct sw_driver_io_descriptor *descriptor);

/*
 * Available functions.
 */
bool sw_socperf_available_i(void);

/*
 * Helper functions.
 */
u32 sw_platform_configdb_read32(u32 address);
u32 sw_platform_pci_read32(u32 bus, u32 device, u32 function, u32 ctrl_offset, u32 ctrl_value, u32 data_offset);
u64 sw_platform_pci_read64(u32 bus, u32 device, u32 function, u32 ctrl_offset, u32 ctrl_value, u32 data_offset);
bool sw_platform_pci_write32(u32 bus, u32 device, u32 function, u32 write_offset, u32 data_value);

/*
 * Table of collector operations.
 */
static const struct sw_hw_ops s_hw_ops[] = {
	[SW_IO_MSR] = {
		.name = "MSR",
		.init = NULL,
		.read = &sw_read_msr_info_i,
		.write = &sw_write_msr_info_i,
		.print = &sw_print_msr_io_descriptor,
		.reset = NULL,
		.available = NULL
	},
	[SW_IO_IPC] = {
		.name = "IPC",
		.init = &sw_ipc_mmio_descriptor_init_func_i,
		.read = &sw_read_ipc_info_i,
		.reset = &sw_ipc_mmio_descriptor_reset_func_i,
		/* Other fields are don't care (will be set to NULL) */
	},
	[SW_IO_MMIO] = {
		.name = "MMIO",
		.init = &sw_ipc_mmio_descriptor_init_func_i,
		.read = &sw_read_mmio_info_i,
		.write = &sw_write_mmio_info_i,
		.reset = &sw_ipc_mmio_descriptor_reset_func_i,
		/* Other fields are don't care (will be set to NULL) */
	},
	[SW_IO_PCI] = {
		.name = "PCI",
		.read = &sw_read_pci_info_i,
		.write = &sw_write_pci_info_i,
		/* Other fields are don't care (will be set to NULL) */
	},
	[SW_IO_CONFIGDB] = {
		.name = "CONFIGDB",
		.read = &sw_read_configdb_info_i,
		/* Other fields are don't care (will be set to NULL) */
	},
	[SW_IO_WAKEUP] = {
		.name = "WAKEUP",
		/* Other fields are don't care (will be set to NULL) */
	},
	[SW_IO_SOCPERF] = {
		.name = "SOCPERF",
		.read = &sw_read_socperf_info_i,
		.available = &sw_socperf_available_i,
		/* Other fields are don't care (will be set to NULL) */
	},
	[SW_IO_PROC_NAME] = {
		.name = "PROC-NAME",
		/* Other fields are don't care (will be set to NULL) */
	},
	[SW_IO_IRQ_NAME] = {
		.name = "IRQ-NAME",
		/* Other fields are don't care (will be set to NULL) */
	},
	[SW_IO_WAKELOCK] = {
		.name = "WAKELOCK",
		/* Other fields are don't care (will be set to NULL) */
	},
	[SW_IO_TELEM] = {
		.name = "TELEM",
		.init = &sw_telem_init_func,
		.read = &sw_read_telem_info,
		.reset = &sw_reset_telem,
		.available = &sw_telem_available,
		.post_config = &sw_telem_post_config,
		/* Other fields are don't care (will be set to NULL) */
	},
	[SW_IO_PCH_MAILBOX] = {
		.name = "PCH-MAILBOX",
		.init = &sw_pch_mailbox_descriptor_init_func_i,
		.read = &sw_read_pch_mailbox_info_i,
		.reset = &sw_pch_mailbox_descriptor_reset_func_i,
		/* Other fields are don't care (will be set to NULL) */
	},
	[SW_IO_MAILBOX] = {
		.name = "MAILBOX",
		.init = &sw_mailbox_descriptor_init_func_i,
		.read = &sw_read_mailbox_info_i,
		.write = &sw_write_mailbox_info_i,
		.reset = &sw_mailbox_descriptor_reset_func_i,
		/* Other fields are don't care (will be set to NULL) */
	},
	[SW_IO_MAX] = {
		.name = NULL,
		/* Other fields are don't care (will be set to NULL) */
	}
};

/*
 * Function definitions.
 */
int sw_ipc_mmio_descriptor_init_func_i(
	struct sw_driver_io_descriptor *descriptor)
{
	/* Perform any required 'io_remap' calls here */
	struct sw_driver_ipc_mmio_io_descriptor *__ipc_mmio = NULL;
	u64 data_address = 0;

	if (!descriptor) /* Should NEVER happen */
		return -PW_ERROR;

	if (descriptor->collection_type == SW_IO_IPC)
		__ipc_mmio = &descriptor->ipc_descriptor;
	else
		__ipc_mmio = &descriptor->mmio_descriptor;

	pw_pr_debug("cmd = %u, sub-cmd = %u, data_addr = 0x%llx\n"
		__ipc_mmio->command, __ipc_mmio->sub_command,
		__ipc_mmio->data_address);
	data_address = __ipc_mmio->data_address;

	if (!data_address)
		return PW_SUCCESS;

	__ipc_mmio->data_remapped_address =
		(pw_u64_t)(unsigned long)ioremap_nocache(
			(unsigned long)data_address,
			descriptor->counter_size_in_bytes);
	if ((void *)(unsigned long)__ipc_mmio->data_remapped_address == NULL)
		return -EIO;

	pw_pr_debug("mapped addr 0x%llx\n", __ipc_mmio->data_remapped_address);
	if ((__ipc_mmio->is_gbe) &&
		(!s_gbe_semaphore.hw_semaphore ||
			!s_gbe_semaphore.fw_semaphore) &&
				(data_address >= GBE_CTRL_OFFSET)) {

		u64 hw_addr = (data_address - GBE_CTRL_OFFSET) + EXTCNF_CTRL;
		u64 fw_addr = (data_address - GBE_CTRL_OFFSET) + FWSM_CTRL;
		pw_pr_debug("Initializing GBE semaphore\n");

		s_gbe_semaphore.hw_semaphore =
			ioremap_nocache(
				(unsigned long)hw_addr,
				descriptor->counter_size_in_bytes);
		s_gbe_semaphore.fw_semaphore =
			ioremap_nocache(
				(unsigned long)fw_addr,
				descriptor->counter_size_in_bytes);
		if (s_gbe_semaphore.hw_semaphore == NULL ||
			s_gbe_semaphore.fw_semaphore == NULL) {
			pw_pr_error(
				"couldn't mmap hw/fw semaphores for GBE MMIO op!\n");
			return -EIO;
		}
		pw_pr_debug(
			"GBE has hw_sem = 0x%llx, fw_sem = 0x%llx, size = %u\n",
			(unsigned long long)s_gbe_semaphore.hw_semaphore,
			(unsigned long long)s_gbe_semaphore.fw_semaphore,
			descriptor->counter_size_in_bytes);
	}

	return PW_SUCCESS;
}

int sw_pch_mailbox_descriptor_init_func_i(
	struct sw_driver_io_descriptor *descriptor)
{
	/* Perform any required 'io_remap' calls here */
	struct sw_driver_pch_mailbox_io_descriptor *__pch_mailbox = NULL;

	if (!descriptor) /* Should NEVER happen */
		return -PW_ERROR;

	__pch_mailbox = &descriptor->pch_mailbox_descriptor;
	pw_pr_debug("pch_mailbox data_addr = 0x%llx\n",
		(unsigned long long)__pch_mailbox->data_address);
	if (__pch_mailbox->mtpmc_address) {
		__pch_mailbox->mtpmc_remapped_address =
			(pw_u64_t)(unsigned long)ioremap_nocache(
				(unsigned long)__pch_mailbox->mtpmc_address,
				descriptor->counter_size_in_bytes);
		if ((void *)(unsigned long)
			__pch_mailbox->mtpmc_remapped_address == NULL)
			return -PW_ERROR;

		pw_pr_debug("mtpmc_mapped addr 0x%llx\n",
			__pch_mailbox->mtpmc_remapped_address);
	}
	if (__pch_mailbox->msg_full_sts_address) {
		__pch_mailbox->msg_full_sts_remapped_address =
			(pw_u64_t)(unsigned long)ioremap_nocache(
				(unsigned long)
					__pch_mailbox->msg_full_sts_address,
				descriptor->counter_size_in_bytes);
		if ((void *)(unsigned long)
			__pch_mailbox->msg_full_sts_remapped_address == NULL)
			return -PW_ERROR;

		pw_pr_debug("msg_full_sts_mapped addr 0x%llx\n",
			__pch_mailbox->msg_full_sts_address);
	}
	if (__pch_mailbox->mfpmc_address) {
		__pch_mailbox->mfpmc_remapped_address =
			(pw_u64_t)(unsigned long)ioremap_nocache(
				(unsigned long)__pch_mailbox->mfpmc_address,
				descriptor->counter_size_in_bytes);
		if ((void *)(unsigned long)
			__pch_mailbox->mfpmc_remapped_address == NULL)
			return -PW_ERROR;

		pw_pr_debug("mfpmc_mapped addr 0x%llx\n",
			__pch_mailbox->mfpmc_remapped_address);
	}
	return PW_SUCCESS;
}

int sw_mailbox_descriptor_init_func_i(
	struct sw_driver_io_descriptor *descriptor)
{
	/* Perform any required 'io_remap' calls here */
	struct sw_driver_mailbox_io_descriptor *__mailbox = NULL;

	if (!descriptor) /* Should NEVER happen */
		return -PW_ERROR;

	__mailbox = &descriptor->mailbox_descriptor;

	pw_pr_debug(
		"type = %u, interface_address = 0x%llx, data_address = 0x%llx\n",
		__mailbox->is_msr_type, __mailbox->interface_address,
		__mailbox->data_address);

	if (!__mailbox->is_msr_type) {
		if (__mailbox->interface_address) {
			__mailbox->interface_remapped_address =
				(pw_u64_t)(unsigned long)ioremap_nocache(
					(unsigned long)__mailbox->interface_address,
					descriptor->counter_size_in_bytes);
			if ((void *)(unsigned long)
				__mailbox->interface_remapped_address == NULL) {
				pw_pr_error(
					"Couldn't iomap interface_address = 0x%llx\n",
					__mailbox->interface_address);
				return -PW_ERROR;
			}
		}
		if (__mailbox->data_address) {
			__mailbox->data_remapped_address =
				(pw_u64_t)(unsigned long)ioremap_nocache(
					(unsigned long)__mailbox->data_address,
					descriptor->counter_size_in_bytes);
			if ((void *)(unsigned long)
				__mailbox->data_remapped_address == NULL) {
				pw_pr_error(
					"Couldn't iomap data_address = 0x%llx\n",
					__mailbox->data_address);
				return -PW_ERROR;
			}
		}
		pw_pr_debug("OK, mapped addr 0x%llx, 0x%llx\n",
			__mailbox->interface_remapped_address,
			__mailbox->data_remapped_address);
	}
	return PW_SUCCESS;
}

void sw_read_msr_info_i(
	char *dst_vals, int cpu,
	const struct sw_driver_io_descriptor *descriptors,
	u16 counter_size_in_bytes)
{
	u64 address = descriptors->msr_descriptor.address;
	u32 l = 0, h = 0;

	if (likely(cpu == RAW_CPU()))
		rdmsr_safe((unsigned long)address, &l, &h);
	else {
		if (rdmsr_safe_on_cpu(
			cpu, (unsigned long)address, &l, &h)) {
			pw_pr_warn(
				"Failed to read MSR address = 0x%llx\n",
				address);
				l = 0; h = 0;
		}
	}
	switch (counter_size_in_bytes) {
	case 4:
		*((u32 *)dst_vals) = l;
		break;
	case 8:
		*((u64 *)dst_vals) = ((u64)h << 32) | l;
		pw_pr_debug(
			"read MSR value = %llu\n", *((u64 *)dst_vals));
		break;
	default:
		break;
	}
}

#ifdef CONFIG_RPMSG_IPC
	#define SW_DO_IPC(cmd, sub_cmd) rpmsg_send_generic_simple_command(cmd, sub_cmd)
#else
	#define SW_DO_IPC(cmd, sub_cmd) (-ENODEV)
#endif // CONFIG_RPMSG_IPC

void sw_read_ipc_info_i(
	char *dst_vals, int cpu,
	const struct sw_driver_io_descriptor *descriptors,
	u16 counter_size_in_bytes)
{
	u16 cmd = descriptors->ipc_descriptor.command;
	u16 sub_cmd = descriptors->ipc_descriptor.sub_command;
	unsigned long remapped_address = (unsigned long)
		descriptors->ipc_descriptor.data_remapped_address;

	if (cmd || sub_cmd) {
		pw_pr_debug("EXECUTING IPC Cmd = %u, %u\n", cmd, sub_cmd);
		if (SW_DO_IPC(cmd, sub_cmd)) {
			pw_pr_error("ERROR running IPC command(s)\n");
			return;
		}
	}

	if (remapped_address) {
		pw_pr_debug("COPYING MMIO size %u\n", counter_size_in_bytes);
		memcpy(dst_vals, (void *)remapped_address,
				counter_size_in_bytes);
	}
	pw_pr_debug("Value = %llu\n", *((u64 *)dst_vals));
}

static void sw_read_gbe_mmio_info_i(
	char *dst_vals,
	const struct sw_driver_io_descriptor *descriptors,
	u16 counter_size_in_bytes)
{
	u32 hw_val = 0, fw_val = 0;
	unsigned long remapped_address = (unsigned long)
		descriptors->mmio_descriptor.data_remapped_address;
	u64 write_value = descriptors->write_value;

	memset(dst_vals, 0, counter_size_in_bytes);

	pw_pr_debug(
		"hw_sem = 0x%llx, fw_sem = 0x%llx, addr = 0x%lx, dst_vals = 0x%lx, size = %u\n",
		(unsigned long long)s_gbe_semaphore.hw_semaphore,
		(unsigned long long)s_gbe_semaphore.fw_semaphore,
		remapped_address,
		(unsigned long)dst_vals,
		counter_size_in_bytes);
	if (!s_gbe_semaphore.hw_semaphore || !s_gbe_semaphore.fw_semaphore ||
		!remapped_address)
		return;

	memcpy_fromio(&hw_val, s_gbe_semaphore.hw_semaphore, sizeof(hw_val));
	memcpy_fromio(&fw_val, s_gbe_semaphore.fw_semaphore, sizeof(fw_val));
	pw_pr_debug("HW_VAL = 0x%lx, FW_VAL = 0x%lx\n",
		(unsigned long)hw_val, (unsigned long)fw_val);
	if (!IS_HW_SEMAPHORE_SET(hw_val) && !IS_FW_SEMAPHORE_SET(fw_val)) {
		memcpy_toio((volatile void __iomem *)remapped_address,
				&write_value, 4 /* counter_size_in_bytes*/);
		memcpy_fromio(dst_vals, (volatile void __iomem *)remapped_address,
				counter_size_in_bytes);
	}
}
void sw_read_mmio_info_i(char *dst_vals, int cpu,
	const struct sw_driver_io_descriptor *descriptors,
	u16 counter_size_in_bytes)
{
	unsigned long remapped_address =
	(unsigned long)descriptors->mmio_descriptor.data_remapped_address;

	/* MMIO for GBE requires a mailbox-like operation */
	if (descriptors->mmio_descriptor.is_gbe)
		sw_read_gbe_mmio_info_i(dst_vals, descriptors, counter_size_in_bytes);
	else {
		if (remapped_address)
			memcpy_fromio(dst_vals, (volatile void __iomem *)remapped_address,
				counter_size_in_bytes);
	}
	pw_pr_debug("Value = %llu\n", *((u64 *)dst_vals));
}

void sw_read_pch_mailbox_info_i(
	char *dst_vals, int cpu,
	const struct sw_driver_io_descriptor *descriptor,
	u16 counter_size_in_bytes)
{
	/*
	 * TODO: spinlock?
	 */
	const struct sw_driver_pch_mailbox_io_descriptor *pch_mailbox =
					 &descriptor->pch_mailbox_descriptor;
	u32 address = pch_mailbox->data_address;
	u64 mtpmc_remapped_address = pch_mailbox->mtpmc_remapped_address;
	u64 msg_full_sts_remapped_address =
		pch_mailbox->msg_full_sts_remapped_address;
	u64 mfpmc_remapped_address = pch_mailbox->mfpmc_remapped_address;

	/*
	 * write address of desired device counter to request from PMC
	 * (shift and add 2 to format device offset)
	 */
	if (mtpmc_remapped_address) {
		int iter = 0;
		u32 written_val = 0;
		/* shift and add 2 to format device offset */
		u32 write_value = (address << 16) + 2;

		memcpy_toio(
			(volatile void __iomem *)(unsigned long)mtpmc_remapped_address,
			 &write_value, 4 /*counter_size_in_bytes*/);
		/*
		 * Check if address has been written using a while loop in
		 * order to wait for the PMC to consume that address and to
		 * introduce sufficient delay so that the message full
		 * status bit has time to flip. This should ensure all is
		 * ready when begin the wait loop for it to turn 0, which
		 * indicates the value is available to be read.
		 * (This fixes problem where values being read were huge.)
		 */
		do {
			memcpy_fromio(&written_val,
				(volatile void __iomem *)(unsigned long)mtpmc_remapped_address,
				4 /*counter_size_in_bytes*/);
			pw_pr_debug(
				"DEBUG: written_val = 0x%x, address = 0x%x\n",
				written_val, address);
			udelay(USEC_DELAY);
		} while ((written_val >> 16) != address && ++iter < NUM_RETRY);
	}


	/*
	 * wait for PMC to set status indicating that device
	 * counter is available for read.
	 */
	if (msg_full_sts_remapped_address) {
		u32 status_wait = 0;
		int iter = 0;

		do {
			memcpy_fromio(&status_wait,
				(volatile void __iomem*)(unsigned long)
					msg_full_sts_remapped_address,
				4 /*counter_size_in_bytes*/);
			pw_pr_debug("DEBUG: status_wait = 0x%x\n",
				status_wait);
			udelay(USEC_DELAY);
		} while ((status_wait & 0x01000000) >> 24 &&
			++iter < NUM_RETRY);
	}

	/*
	 * read device counter
	 */
	if (mfpmc_remapped_address) {
		memcpy_fromio(dst_vals,
			(volatile void __iomem*)(unsigned long)mfpmc_remapped_address,
			4 /*counter_size_in_bytes*/);
		pw_pr_debug("DEBUG: read value = 0x%x\n",
			*((pw_u32_t *)dst_vals));
	}
}

void sw_read_mailbox_info_i(char *dst_vals, int cpu,
		const struct sw_driver_io_descriptor *descriptor,
		u16 counter_size_in_bytes)
{
	/*
	 * TODO: spinlock?
	 */
	const struct sw_driver_mailbox_io_descriptor *mailbox =
		&descriptor->mailbox_descriptor;
	unsigned long interface_address = mailbox->interface_address;
	unsigned long interface_remapped_address = mailbox->interface_remapped_address;
	unsigned long data_address = mailbox->data_address;
	size_t iter = 0;

	if (mailbox->is_msr_type) {
		u64 command = 0;

		rdmsrl_safe(interface_address, &command);
		command &= mailbox->command_mask;
		command |= mailbox->command | (u64)0x1 << mailbox->run_busy_bit;
		wrmsrl_safe(interface_address, command);
		do {
			udelay(1);
			rdmsrl_safe(interface_address, &command);
		} while ((command & ((u64)0x1 << mailbox->run_busy_bit)) &&
				++iter < MAX_MAILBOX_ITERS);
		if (iter >= MAX_MAILBOX_ITERS) {
			pw_pr_error("Couldn't write to BIOS mailbox\n");
			command = MAX_UNSIGNED_64_BIT_VALUE;
		} else
			rdmsrl_safe(data_address, &command);
		switch (counter_size_in_bytes) {
		case 4:
			*((u32 *)dst_vals) = (u32)command;
			break;
		case 8:
			*((u64 *)dst_vals) = command;
			break;
		default:
			pw_pr_error("Invalid counter size %u, assuming 4 bytes!\n", counter_size_in_bytes);
			*((u32 *)dst_vals) = (u32)command;
			break;
		}
	}  else {
		u32 command = 0;
		/* Always use 4 bytes, regardless of 'counter_size_in_bytes' */
		const size_t counter_size = 4;

		memcpy_fromio(&command,
			(volatile void __iomem *)(unsigned long)interface_remapped_address,
			sizeof(command));
		command &= mailbox->command_mask;
		command |= (u32)mailbox->command |
				(u32)0x1 << mailbox->run_busy_bit;
		memcpy_toio((volatile void __iomem *)(unsigned long)interface_remapped_address,
			&command, sizeof(command));
		do {
			udelay(1);
			memcpy_fromio(&command,
				(volatile void __iomem *)(unsigned long)interface_remapped_address,
				sizeof(command));
		} while ((command & ((u32)0x1 << mailbox->run_busy_bit)) &&
				++iter < MAX_MAILBOX_ITERS);
		if (iter >= MAX_MAILBOX_ITERS) {
			pw_pr_error("Couldn't write to BIOS mailbox\n");
			command = MAX_UNSIGNED_32_BIT_VALUE;
		} else
			memcpy_fromio(&command,
				(volatile void __iomem *)(unsigned long)mailbox->data_remapped_address,
				counter_size);

		*((u32 *)dst_vals) = command;
	}
}

void sw_read_pci_info_i(char *dst_vals, int cpu,
	const struct sw_driver_io_descriptor *descriptors,
	u16 counter_size_in_bytes)
{
	u32 bus = descriptors->pci_descriptor.bus;
	u32 device = descriptors->pci_descriptor.device;
	u32 function = descriptors->pci_descriptor.function;
	u32 offset = descriptors->pci_descriptor.offset;
	u32 data32 = 0;
	u64 data64 = 0;

	switch (counter_size_in_bytes) {
	case 4:
		data32 = sw_platform_pci_read32(bus, device, function,
			0 /* CTRL-OFFSET */, 0 /* CTRL-DATA, don't care */,
			offset /* DATA-OFFSET */);
		*((u32 *)dst_vals) = data32;
		break;
	case 8:
		data64 = sw_platform_pci_read64(bus, device, function,
			0 /* CTRL-OFFSET */, 0 /* CTRL-DATA, don't care */,
			offset /* DATA-OFFSET */);
		*((u64 *)dst_vals) = data64;
		break;
	default:
		pw_pr_error("ERROR: invalid read size = %u\n",
				counter_size_in_bytes);
	}
}
void sw_read_configdb_info_i(char *dst_vals, int cpu,
	const struct sw_driver_io_descriptor *descriptors,
	u16 counter_size_in_bytes)
{
	pw_u32_t address = descriptors->configdb_descriptor.address;
	u32 data = sw_platform_configdb_read32(address);

	pw_pr_debug(
		"ADDRESS = 0x%x, CPU = %d, dst_vals = %p, counter size = %u, data = %u\n",
		address, cpu, dst_vals, counter_size_in_bytes, data);
	/*
	 * 'counter_size_in_bytes' is ignored, for now.
	 */
	*((u32 *)dst_vals) = data;
}
void sw_read_socperf_info_i(char *dst_vals, int cpu,
	const struct sw_driver_io_descriptor *descriptors,
	u16 counter_size_in_bytes)
{
	u64 *socperf_buffer = (u64 *)dst_vals;

	memset(socperf_buffer, 0, counter_size_in_bytes);
	SOCPERF_Read_Data3(socperf_buffer);

}

/**
 * Decide if the socperf interface is available for use
 * @returns	 true if available
 */
bool sw_socperf_available_i(void)
{
	bool retVal = false;

	/* The symbol below is weak.  We return 1 if we have a definition
	 * for this socperf-driver-supplied symbol, or 0 if only the
	 * weak definition exists. This test will suffice to detect if
	 * the socperf driver is loaded.
	 */
	if (SOCPERF_Read_Data3) {
		pw_pr_debug("INFO: SoCPerf support in ON!\n");
		retVal = true;
	} else
		pw_pr_debug("INFO: SoCPerf support is OFF!\n");

	return retVal;
}


/**
 * sw_platform_configdb_read32 - for reading PCI space through config registers
 *							   of the platform.
 * @address: An address in the PCI space
 *
 * Returns: the value read from address.
 */
u32 sw_platform_configdb_read32(u32 address)
{
	u32 read_value = 0;
#if DO_DIRECT_PCI_READ_WRITE
	read_value = sw_platform_pci_read32(
		0/*bus*/, 0/*device*/, 0/*function*/,
		SW_PCI_MSG_CTRL_REG/*ctrl-offset*/, address/*ctrl-value*/,
		SW_PCI_MSG_DATA_REG/*data-offset*/);
#else /* !DO_DIRECT_PCI_READ_WRITE */
	read_value = intel_mid_msgbus_read32_raw(address);
#endif /* if DO_DIRECT_PCI_READ_WRITE */
	pw_pr_debug("address = %u, value = %u\n", address, read_value);
	return read_value;
}

u32 sw_platform_pci_read32(u32 bus, u32 device, u32 function,
		u32 write_offset, u32 write_value, u32 read_offset)
{
	u32 read_value = 0;
	struct pci_dev *pci_root =
		pci_get_domain_bus_and_slot(0, bus,
			/* 0, PCI_DEVFN(0, 0)); */
			PCI_DEVFN(device, function));

	if (!pci_root)
		return 0; /* Application will verify the data */

	if (write_offset)
		pci_write_config_dword(pci_root,
			/* SW_PCI_MSG_CTRL_REG, address); */
			write_offset, write_value);

	pci_read_config_dword(pci_root,
		/* SW_PCI_MSG_DATA_REG, &read_value); */
		read_offset, &read_value);
	return read_value;
}

u64 sw_platform_pci_read64(u32 bus, u32 device, u32 function, u32 write_offset,
	u32 write_value, u32 read_offset)
{
	u32 lo = sw_platform_pci_read32(
		bus, device, function, 0 /* CTRL-OFFSET */,
		0 /* CTRL-DATA, don't care */,
		read_offset /* DATA-OFFSET */);
	u32 hi = sw_platform_pci_read32(
		bus, device, function, 0 /* CTRL-OFFSET */,
		0 /* CTRL-DATA, don't care */,
		read_offset + 4 /* DATA-OFFSET */);

	return ((u64)hi << 32) | lo;
}

void sw_write_msr_info_i(char *dst_vals, int cpu,
	const struct sw_driver_io_descriptor *descriptor,
	u16 counter_size_in_bytes)
{
	u64 write_value = descriptor->write_value;
	u64 address = descriptor->msr_descriptor.address;

	pw_pr_debug(
		"ADDRESS = 0x%llx, CPU = %d, counter size = %u, value = %llu\n",
		address, cpu, counter_size_in_bytes, write_value);
	if (likely(cpu == RAW_CPU()))
		wrmsrl_safe((unsigned long)address, write_value);
	else {
		u32 l = write_value & 0xffffffff;
		u32 h = (write_value >> 32) & 0xffffffff;

		wrmsr_safe_on_cpu(cpu, (u32)address, l, h);
	}
};

void sw_write_mmio_info_i(char *dst_vals, int cpu,
	const struct sw_driver_io_descriptor *descriptor,
	u16 counter_size_in_bytes)
{
	unsigned long remapped_address = (unsigned long)
		descriptor->mmio_descriptor.data_remapped_address;
	u64 write_value = descriptor->write_value;

	if (remapped_address)
		memcpy_toio((volatile void __iomem *)remapped_address, &write_value,
			counter_size_in_bytes);

	pw_pr_debug("Value = %llu\n", *((u64 *)dst_vals));
};

void sw_write_mailbox_info_i(char *dst_vals, int cpu,
	const struct sw_driver_io_descriptor *descriptor,
	u16 counter_size_in_bytes)
{
	/*
	 * TODO: spinlock?
	 */
	const struct sw_driver_mailbox_io_descriptor *mailbox =
					&descriptor->mailbox_descriptor;
	unsigned long interface_address = mailbox->interface_address;
	unsigned long interface_remapped_address =
					mailbox->interface_remapped_address;
	unsigned long data_address = mailbox->data_address;
	u64 data = descriptor->write_value;
	size_t iter = 0;

	if (mailbox->is_msr_type) {
		u64 command = 0;

		rdmsrl_safe(interface_address, &command);
		command &= mailbox->command_mask;
		command |= mailbox->command |
				(u64)0x1 << mailbox->run_busy_bit;
		wrmsrl_safe(data_address, data);
		wrmsrl_safe(interface_address, command);
		do {
			rdmsrl_safe(interface_address, &command);
		} while ((command & ((u64)0x1 << mailbox->run_busy_bit)) &&
				++iter < MAX_MAILBOX_ITERS);
	} else {
		u32 command = 0;

		memcpy_fromio(&command,
			(volatile void __iomem *)(unsigned long)interface_remapped_address,
			sizeof(command));
		command &= mailbox->command_mask;
		command |= (u32)mailbox->command |
			(u32)0x1 << mailbox->run_busy_bit;
		memcpy_toio((volatile void __iomem *)(unsigned long)
			mailbox->data_remapped_address,
			&data, sizeof(data));
		memcpy_toio((volatile void __iomem *)(unsigned long)interface_remapped_address,
			&command, sizeof(command));
		do {
			memcpy_fromio(&command, (volatile void __iomem *)(unsigned long)
				interface_remapped_address, sizeof(command));
		} while ((command & ((u32)0x1 << mailbox->run_busy_bit)) &&
				++iter < MAX_MAILBOX_ITERS);
	}
}

void sw_write_pci_info_i(char *dst_vals, int cpu,
	const struct sw_driver_io_descriptor *descriptor,
	u16 counter_size_in_bytes)
{
	u32 bus = descriptor->pci_descriptor.bus;
	u32 device = descriptor->pci_descriptor.device;
	u32 function = descriptor->pci_descriptor.function;
	u32 offset = descriptor->pci_descriptor.offset;
	u32 write_value = (u32)descriptor->write_value;
	/*
	 * 'counter_size_in_bytes' is ignored for now.
	 */
	if (!sw_platform_pci_write32(bus, device, function, offset,
			write_value))
		pw_pr_error("ERROR writing to PCI B/D/F/O %u/%u/%u/%u\n",
			bus, device, function, offset);
	else
		pw_pr_debug("OK, successfully wrote to PCI B/D/F/O %u/%u/%u/%u\n",
			bus, device, function, offset);

};

/*
 * Write to PCI space via config registers.
 */
bool sw_platform_pci_write32(u32 bus, u32 device, u32 function,
	u32 write_offset, u32 data_value)
{
	struct pci_dev *pci_root =
		pci_get_domain_bus_and_slot(0, bus,
			PCI_DEVFN(device, function));/* 0, PCI_DEVFN(0, 0)); */

	if (!pci_root)
		return false;


	pci_write_config_dword(pci_root, write_offset, data_value);

	return true;
};

int sw_print_msr_io_descriptor(const struct sw_driver_io_descriptor *descriptor)
{
	if (!descriptor)
		return -PW_ERROR;

	pw_pr_debug("MSR address = 0x%llx\n",
		descriptor->msr_descriptor.address);
	return PW_SUCCESS;
}

int sw_ipc_mmio_descriptor_reset_func_i(
	const struct sw_driver_io_descriptor *descriptor)
{
	/* Unmap previously mapped memory here */
	struct sw_driver_ipc_mmio_io_descriptor *__ipc_mmio = NULL;

	if (!descriptor) /* Should NEVER happen */
		return -PW_ERROR;

	if (descriptor->collection_type == SW_IO_IPC)
		__ipc_mmio = (struct sw_driver_ipc_mmio_io_descriptor *)
			&descriptor->ipc_descriptor;
	else
		__ipc_mmio = (struct sw_driver_ipc_mmio_io_descriptor *)
			&descriptor->mmio_descriptor;

	if (__ipc_mmio->data_remapped_address) {
		pw_pr_debug("unmapping addr 0x%llx\n",
			__ipc_mmio->data_remapped_address);
		iounmap((volatile void __iomem *)(unsigned long)
			__ipc_mmio->data_remapped_address);
		__ipc_mmio->data_remapped_address = 0;
	}
	/* Uninitialize the GBE, if it wasn't already done */
	if (s_gbe_semaphore.hw_semaphore ||
			s_gbe_semaphore.fw_semaphore) {
		pw_pr_debug("Uninitializing gbe!\n");
		if (s_gbe_semaphore.hw_semaphore)
			iounmap(s_gbe_semaphore.hw_semaphore);

		if (s_gbe_semaphore.fw_semaphore)
			iounmap(s_gbe_semaphore.fw_semaphore);

		memset(&s_gbe_semaphore, 0, sizeof(s_gbe_semaphore));
	}
	return PW_SUCCESS;
}

int sw_pch_mailbox_descriptor_reset_func_i(
	const struct sw_driver_io_descriptor *descriptor)
{
	/* Unmap previously mapped memory here */
	struct sw_driver_pch_mailbox_io_descriptor *__pch_mailbox = NULL;

	if (!descriptor) /* Should NEVER happen */
		return -PW_ERROR;

	__pch_mailbox = (struct sw_driver_pch_mailbox_io_descriptor *)
			&descriptor->pch_mailbox_descriptor;
	if (__pch_mailbox->mtpmc_remapped_address) {
		pw_pr_debug("unmapping addr 0x%llx\n",
			__pch_mailbox->mtpmc_remapped_address);
		iounmap((volatile void __iomem *)(unsigned long)
			__pch_mailbox->mtpmc_remapped_address);
		__pch_mailbox->mtpmc_remapped_address = 0;
	}
	if (__pch_mailbox->msg_full_sts_remapped_address) {
		pw_pr_debug("unmapping addr 0x%llx\n",
			__pch_mailbox->msg_full_sts_remapped_address);
		iounmap((volatile void __iomem *)(unsigned long)
			__pch_mailbox->msg_full_sts_remapped_address);
		__pch_mailbox->msg_full_sts_remapped_address = 0;
	}
	if (__pch_mailbox->mfpmc_remapped_address) {
		pw_pr_debug("unmapping addr 0x%llx\n",
			__pch_mailbox->mfpmc_remapped_address);
		iounmap((volatile void __iomem *)(unsigned long)
			__pch_mailbox->mfpmc_remapped_address);
		__pch_mailbox->mfpmc_remapped_address = 0;
	}
	return PW_SUCCESS;
}

int sw_mailbox_descriptor_reset_func_i(
	const struct sw_driver_io_descriptor *descriptor)
{
	/* Unmap previously mapped memory here */
	struct sw_driver_mailbox_io_descriptor *__mailbox = NULL;

	if (!descriptor) /* Should NEVER happen */
		return -PW_ERROR;

	__mailbox = (struct sw_driver_mailbox_io_descriptor *)
			&descriptor->mailbox_descriptor;
	if (!__mailbox->is_msr_type) {
		if (__mailbox->interface_remapped_address) {
			pw_pr_debug("unmapping addr 0x%llx\n",
				__mailbox->interface_remapped_address);
			iounmap((volatile void __iomem *)(unsigned long)
				__mailbox->interface_remapped_address);
			__mailbox->interface_remapped_address = 0;
		}
		if (__mailbox->data_remapped_address) {
			pw_pr_debug("unmapping addr 0x%llx\n",
				__mailbox->data_remapped_address);
			iounmap((volatile void __iomem *)(unsigned long)
				__mailbox->data_remapped_address);
			__mailbox->data_remapped_address = 0;
		}
	}
	return PW_SUCCESS;
}

#define NUM_HW_OPS SW_ARRAY_SIZE(s_hw_ops)
#define FOR_EACH_HW_OP(idx, op)					\
	for (idx = 0; idx < NUM_HW_OPS && (op =  &s_hw_ops[idx]); ++idx)

int sw_register_ops_providers(void)
{
	size_t idx = 0;
	const struct sw_hw_ops *op = NULL;

	FOR_EACH_HW_OP(idx, op) {
		if (op->name && sw_register_hw_op(op)) {
			pw_pr_error(
				"ERROR registering provider %s\n", op->name);
			return -EIO;
		}
	}
	return PW_SUCCESS;
}

void sw_free_ops_providers(void)
{
	/* NOP */
}
