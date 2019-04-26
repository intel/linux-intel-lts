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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/compiler.h>     /* Definition of __weak */
#include <linux/version.h>      /* LINUX_VERSION_CODE */
#include <linux/delay.h>        /* 'udelay' */
#include <linux/io.h>           /* Definition of ioremap_nocache and iounmap */
#include "sw_kernel_defines.h"  /* pw_pr_debug */
#include "sw_mem.h"             /* sw_kmalloc/free */
#include "sw_lock_defs.h"       /* Various lock-related definitions */
#include "sw_telem.h"           /* Signatures of fn's exported from here. */

/*
 * These functions and data structures are exported by the Telemetry
 * driver.  However, that file may not be available in the kernel for
 * which this driver is being built, so we re-define many of the same
 * things here.
 */
/**
 * struct telemetry_evtlog - The "event log" returned by the kernel's
 *                           full-read telemetry driver.
 * @telem_evtid:   The 16-bit event ID.
 * @telem_evtlog:  The actual telemetry data.
 */
struct telemetry_evtlog {
	u32 telem_evtid;	/* Event ID of a data item. */
	u64 telem_evtlog;   /* Counter data */
};

struct telemetry_evtconfig {
	u32 *evtmap;	/* Array of Event-IDs to Enable */
	u8 num_evts;	/* Number of Events (<29) in evtmap */
	u8 period;	  /* Sampling period */
};

#define MAX_TELEM_EVENTS 28  /* Max telem events per unit */

/* The enable bit is set when programming events, but is returned
 * cleared for queried events requests.
 */
#define TELEM_EVENT_ENABLE 0x8000 /* Enabled when Event ID HIGH bit */

/*
 * Sampling Period values.
 * The sampling period is encoded in an 7-bit value, where
 *	Period = (Value * 16^Exponent) usec where:
 *		bits[6:3] -> Value;
 *		bits [0:2]-> Exponent;
 * Here are some of the calculated possible values:
 * | Value  Val+Exp  | Value | Exponent | Period (usec) | Period (msec) |
 * |-----------------+-------+----------+---------------+---------------|
 * | 0xA = 000 1+010 |   1   |     2    |           256 |         0.256 |
 * | 0x12= 001 0+010 |   2   |     2    |           512 |         0.512 |
 * | 0x22= 010 0+010 |   4   |     2    |          1024 |         1.024 |
 * | 0xB = 000 1+011 |   1   |     3    |          4096 |         4.096 |
 * | 0x13= 001 0+011 |   2   |     3    |          8192 |         8.192 |
 * | 0x1B= 001 1+011 |   3   |     3    |         12288 |        12.288 |
 * | 0x0C= 000 1+100 |   1   |     4    |         65536 |        65.536 |
 * | 0x0D= 000 1+101 |   1   |     5    |       1048576 |      1048.576 |
 */
#define TELEM_SAMPLING_1MS 0x22  /* Approximately 1 ms */
#define TELEM_SAMPLING_1S  0x0D  /* Approximately 1 s */

/* These functions make up the main APIs of the telemetry driver.  We
 * define all of them with weak linkage so that we can still compile
 * and load into kernels which don't have a telemetry driver.
 */
extern int __weak telemetry_get_eventconfig(
	struct telemetry_evtconfig *punit_config,
	struct telemetry_evtconfig *pmc_config,
	int  punit_len,
	int  pmc_len);

extern int __weak telemetry_reset_events(void);

extern int __weak telemetry_set_sampling_period(
	u8 punit_period,
	u8 pmc_period);
/*
 * Older kernels didn't have the p-unit/pmc ipc command interface
 */
extern int __weak intel_punit_ipc_command(
	u32 cmd, u32 para1, u32 para2, u32 *in, u32 *out);

extern int __weak intel_pmc_ipc_command(
	u32 cmd, u32 sub, u8 *in, u32 inlen, u32 *out, u32 outlen);
/*
 * Spinlock to guard updates to the 'iters' values.
 */
static SW_DEFINE_SPINLOCK(sw_telem_lock);


/* ************************************************
 * Constants for P-unit/PMC telemetry interface
 *  ***********************************************
 */

#define PUNIT_MAILBOX_INTERFACE_OFFSET		0x7084
#define PUNIT_MAILBOX_DATA_OFFSET		0x7080

#define PSS_TELEM_SSRAM_OFFSET			0x1A00
#define IOSS_TELEM_SSRAM_OFFSET			0x1B00
#define TELEM_SSRAM_SIZE			240

#define PMC_IPC_CMD				0x0

#define PMC_IPC_STATUS				0x04

#define PMC_IPC_WRITE_BUFFER			0x80
#define PMC_IPC_READ_BUFFER			0x90

#define PMC_IPC_PMC_TELEMETRY_COMMAND		0xEB


#define TELEM_READ_TIMEOUT_TRIAL		10
#define TELEM_MAILBOX_STATUS_TIMEOUT		1000

#define IPC_BIOS_PUNIT_CMD_BASE			0x00

#define IPC_BIOS_PUNIT_CMD_READ_TELE_INFO				\
					(IPC_BIOS_PUNIT_CMD_BASE + 0x09)
#define IPC_BIOS_PUNIT_CMD_READ_TELE_EVENT_CTRL				\
					(IPC_BIOS_PUNIT_CMD_BASE + 0x0c)
#define IPC_BIOS_PUNIT_CMD_WRITE_TELE_EVENT_CTRL			\
					(IPC_BIOS_PUNIT_CMD_BASE + 0x0d)
#define IPC_BIOS_PUNIT_CMD_WRITE_TELE_EVENT				\
					(IPC_BIOS_PUNIT_CMD_BASE + 0x11)

#define IOSS_TELEM_EVENT_WRITE			0x1
#define IOSS_TELEM_INFO_READ			0x2
#define IOSS_TELEM_EVENT_CTL_READ		0x7
#define IOSS_TELEM_EVENT_CTL_WRITE		0x8

#define IOSS_TELEM_EVT_CTRL_WRITE_SIZE		0x4
#define IOSS_TELEM_READ_WORD			0x1
#define IOSS_TELEM_EVT_WRITE_SIZE		0x3

#ifndef BIT
	#define BIT(x)				(1<<x)
#endif /* BIT */

#define TELEM_DISABLE(x)			((x) &= ~(BIT(31)))
#define TELEM_ENABLE_SSRAM_EVT_TRACE(x)		((x) &= ~(BIT(30) | BIT(24)))
#define TELEM_ENABLE_PERIODIC(x)	((x) |= (BIT(23) | BIT(31) | BIT(7)))
#define TELEM_IOSS_EVTID_SHIFT			8

#define TELEM_INFO_SSRAMEVTS_MASK		0xFF00
#define TELEM_INFO_SSRAMEVTS_SHIFT		0x8

#define TELEM_MIN_PERIOD(x)			((x) & 0x7F0000)
#define TELEM_MAX_PERIOD(x)			((x) & 0x7F000000)
#define TELEM_CLEAR_SAMPLE_PERIOD(x)		((x) &= ~0x7F)
#define TELEM_DEFAULT_SAMPLING_PERIOD		TELEM_SAMPLING_1MS

#define IS_TELEM_CONFIGURED()			\
	(s_telemEventInfo[TELEM_PUNIT].idx > 0	\
	|| s_telemEventInfo[TELEM_PMC].idx > 0)

static u64 s_mchBarAddrs[3] = {0, 0, 0};

static struct {
	volatile u64 *ssram_virt_addr;
	int idx, iters;
	u32 events[MAX_TELEM_EVENTS];
	u64 data_buffer[MAX_TELEM_EVENTS];
} s_telemEventInfo[TELEM_UNIT_NONE] = {
	[TELEM_PUNIT] = {NULL, 0, 0},
	[TELEM_PMC] = {NULL, 0, 0},
};

static volatile u64 *s_punitInterfaceAddr;
static volatile u64 *s_punitDataAddr;
static volatile u64 *s_pmcIPCCmdAddr;
static volatile u64 *s_pmcIPCStsAddr;
static volatile u64 *s_pmcIPCWBufAddr;
static volatile u64 *s_pmcIPCRBufAddr;

/**
 * setup_punit_mbox -- Setup P-Unit virtual mappings
 *
 * Returns: true if setup successfully
 */
static bool setup_punit_mbox(void)
{
	s_punitInterfaceAddr = (u64 *)ioremap_nocache(
				(unsigned long)s_mchBarAddrs[TELEM_MCHBAR_CFG] +
				PUNIT_MAILBOX_INTERFACE_OFFSET, 0x4);
	s_punitDataAddr = (u64 *)ioremap_nocache(
				(unsigned long)s_mchBarAddrs[TELEM_MCHBAR_CFG] +
				PUNIT_MAILBOX_DATA_OFFSET, 0x4);
	s_telemEventInfo[TELEM_PUNIT].ssram_virt_addr = (u64 *)ioremap_nocache(
				(unsigned long)
					s_mchBarAddrs[TELEM_SSRAMBAR_CFG] +
				PSS_TELEM_SSRAM_OFFSET, TELEM_SSRAM_SIZE);

	return (s_punitInterfaceAddr && s_punitDataAddr &&
		s_telemEventInfo[TELEM_PUNIT].ssram_virt_addr);
}

/**
 * destroy_punit_mbox -- Unmap p-unit virtual addresses
 */
static void destroy_punit_mbox(void)
{
	if (s_punitInterfaceAddr) {
		iounmap(s_punitInterfaceAddr);
		s_punitInterfaceAddr = NULL;
	}
	if (s_punitDataAddr) {
		iounmap(s_punitDataAddr);
		s_punitDataAddr = NULL;
	}
	if (s_telemEventInfo[TELEM_PUNIT].ssram_virt_addr) {
		iounmap(s_telemEventInfo[TELEM_PUNIT].ssram_virt_addr);
		s_telemEventInfo[TELEM_PUNIT].ssram_virt_addr = NULL;
	}
}

/**
 * setup_pmc_mbox -- Setup PMC virtual mappings
 *
 * Returns: true if setup successfully
 */
static bool setup_pmc_mbox(void)
{
	s_pmcIPCCmdAddr = (u64 *)ioremap_nocache(
			(unsigned long)s_mchBarAddrs[TELEM_IPC1BAR_CFG] +
			PMC_IPC_CMD, 0x4);
	s_pmcIPCStsAddr = (u64 *)ioremap_nocache(
			(unsigned long)s_mchBarAddrs[TELEM_IPC1BAR_CFG] +
			PMC_IPC_STATUS, 0x4);
	s_pmcIPCWBufAddr = (u64 *)ioremap_nocache(
			(unsigned long)s_mchBarAddrs[TELEM_IPC1BAR_CFG] +
			PMC_IPC_WRITE_BUFFER, 0x4);
	s_pmcIPCRBufAddr = (u64 *)ioremap_nocache(
			(unsigned long)s_mchBarAddrs[TELEM_IPC1BAR_CFG] +
			PMC_IPC_READ_BUFFER, 0x4);
	s_telemEventInfo[TELEM_PMC].ssram_virt_addr = (u64 *)ioremap_nocache(
			(unsigned long)s_mchBarAddrs[TELEM_SSRAMBAR_CFG] +
			IOSS_TELEM_SSRAM_OFFSET, TELEM_SSRAM_SIZE);

	return (s_pmcIPCCmdAddr && s_pmcIPCStsAddr &&
		s_pmcIPCWBufAddr && s_pmcIPCRBufAddr &&
		s_telemEventInfo[TELEM_PMC].ssram_virt_addr);
}

/**
 * destroy_pmc_mbox -- Unmap PMC virtual addresses
 */
static void destroy_pmc_mbox(void)
{
	if (s_pmcIPCCmdAddr) {
		iounmap(s_pmcIPCCmdAddr);
		s_pmcIPCCmdAddr = NULL;
	}
	if (s_pmcIPCStsAddr) {
		iounmap(s_pmcIPCStsAddr);
		s_pmcIPCStsAddr = NULL;
	}
	if (s_pmcIPCWBufAddr) {
		iounmap(s_pmcIPCWBufAddr);
		s_pmcIPCWBufAddr = NULL;
	}
	if (s_pmcIPCRBufAddr) {
		iounmap(s_pmcIPCRBufAddr);
		s_pmcIPCRBufAddr = NULL;
	}
	if (s_telemEventInfo[TELEM_PMC].ssram_virt_addr) {
		iounmap(s_telemEventInfo[TELEM_PMC].ssram_virt_addr);
		s_telemEventInfo[TELEM_PMC].ssram_virt_addr = NULL;
	}
}

/**
 * setup_telem - Setup telemetry interface
 *
 * Returns: 0 if setup successfully, 1 otherwise
 */
int setup_telem(u64 addrs[3])
{
	/*
	 * Don't setup if already done so
	 */
	if (s_mchBarAddrs[TELEM_MCHBAR_CFG])
		return 0;

	memcpy(s_mchBarAddrs, addrs, sizeof(s_mchBarAddrs));
	/*
	 * Setup Punit
	 */
	if (!setup_punit_mbox()) {
		pw_pr_error("Couldn't setup PUNIT mbox\n");
		return -1;
	}
	/*
	 * Setup PMC
	 */
	if (!setup_pmc_mbox()) {
		pw_pr_error("Couldn't setup PMC mbox\n");
		return -1;
	}
	return 0;
}

/**
 * destroy_telem - Destroy telemetry interface
 */
void destroy_telem(void)
{
	destroy_punit_mbox();
	destroy_pmc_mbox();

	memset(s_mchBarAddrs, 0, sizeof(s_mchBarAddrs));
}

/**
 * get_or_set_id - Add ID to list of events if not previously added
 *
 * Returns: 0 if setup successfully, 1 otherwise
 */
static int get_or_set_id(u32 *events, u32 *unit_idx, u32 id)
{
	u32 i = 0;

	if (*unit_idx >= MAX_TELEM_EVENTS)
		return -1;

	for (i = 0; i <  *unit_idx; ++i) {
		if (events[i] == id)
			return i;
	}
	events[*unit_idx] = id;
	return (*unit_idx)++;
}

static int add_telem_id(enum telemetry_unit unit, u32 id)
{
	return get_or_set_id(
		s_telemEventInfo[unit].events,
		&s_telemEventInfo[unit].idx, id);
}

static void remove_telem_ids(void)
{
	memset(s_telemEventInfo, 0, sizeof(s_telemEventInfo));
}


static u64 read_telem_data(u64 *dst, volatile void *src, size_t num_events)
{
	u32 i, timeout = 0;
	u64 prev_timestamp = 0, next_timestamp = 0, start_time = 0, event_data;

	if (!dst)
		return 0;

	do {
		u64 *_src = (u64 *)src;

		prev_timestamp = *_src;
		if (!prev_timestamp)
			return 0;

		start_time = *(_src + 1);

		for (i = 0; i < num_events; ++i) {
			event_data = *(_src + 2 + i);
			dst[i] = event_data;
		}
		next_timestamp = *_src;

		if (!next_timestamp)
			return 0;

		if (++timeout == TELEM_READ_TIMEOUT_TRIAL)
			break;

	} while (prev_timestamp != next_timestamp);
	return prev_timestamp == next_timestamp ? start_time : 0;
}

/**
 * @returns timestamp (1st entry of SSRAM)
 */
static u64 flush_telem_to_buffer(enum telemetry_unit unit)
{
	return read_telem_data(s_telemEventInfo[unit].data_buffer,
			   s_telemEventInfo[unit].ssram_virt_addr,
			   s_telemEventInfo[unit].idx);
}

static void read_telem_from_buffer(u64 *dst, enum telemetry_unit unit)
{
	memcpy(dst, s_telemEventInfo[unit].data_buffer,
		s_telemEventInfo[unit].idx * sizeof(*dst));
}

static u64 read_event_from_buffer(enum telemetry_unit unit, int idx)
{
	if (idx < 0 || idx >= MAX_TELEM_EVENTS)
		return SW_TELEM_READ_FAIL_VALUE;

	return s_telemEventInfo[unit].data_buffer[idx];
}

static bool punit_start_telem(void)
{
	u32 telem_info = 0, telem_ctrl = 0, i;

	/* Reset data buffer */
	memset(s_telemEventInfo[TELEM_PUNIT].data_buffer, 0,
		sizeof(s_telemEventInfo[TELEM_PUNIT].data_buffer));

	/* Read basic config */
	if (intel_punit_ipc_command(IPC_BIOS_PUNIT_CMD_READ_TELE_INFO, 0, 0,
			NULL, &telem_info))
		pw_pr_warn("Could not execute P-unit IPC command to read telem info\n");

	/* Debug info */
	pw_pr_debug("DEBUG: Read P-Unit telem_info = 0x%x\n", telem_info);
	pw_pr_debug("## SOCWATCHDRV ## PUNIT Telemetry info has events = %u\n",
		(telem_info & TELEM_INFO_SSRAMEVTS_MASK) >>
			TELEM_INFO_SSRAMEVTS_SHIFT);
	pw_pr_debug(
		"## SOCWATCHDRV ## PUNIT Telemetry info has event_regs = %u\n",
		telem_info & TELEM_INFO_SSRAMEVTS_MASK);
	pw_pr_debug(
		"## SOCWATCHDRV ## PUNIT Telemetry info has min_period = %u\n",
		TELEM_MIN_PERIOD(telem_info));
	pw_pr_debug(
		"## SOCWATCHDRV ## PUNIT Telemetry info has max_period = %u\n",
		TELEM_MAX_PERIOD(telem_info));

	/*TODO: check if #events or #event_regs is less than 28; exit if so */

	/* Read control structure */
	if (intel_punit_ipc_command(IPC_BIOS_PUNIT_CMD_READ_TELE_EVENT_CTRL,
			0, 0, NULL, &telem_ctrl))
		pw_pr_warn("Could not execute P-unit IPC command to read telem ctrl structure\n");

	/* Disable telem */
	TELEM_DISABLE(telem_ctrl);
	if (intel_punit_ipc_command(IPC_BIOS_PUNIT_CMD_WRITE_TELE_EVENT_CTRL,
			0, 0, &telem_ctrl, NULL))
		pw_pr_warn("Could not execute P-unit IPC command to write telem ctrl structure\n");

	/* Each event added requires a separate command */
	for (i = 0; i < s_telemEventInfo[TELEM_PUNIT].idx; ++i) {
		u32 event = s_telemEventInfo[TELEM_PUNIT].events[i] |
			TELEM_EVENT_ENABLE;

		pw_pr_debug("DEBUG: enabling PUNIT event 0x%x\n",
		s_telemEventInfo[TELEM_PUNIT].events[i]);
		if (intel_punit_ipc_command(
				IPC_BIOS_PUNIT_CMD_WRITE_TELE_EVENT, i, 0,
				&event, NULL))
			pw_pr_warn("Could not execute P-unit IPC command to write telem event\n");

	}

	TELEM_CLEAR_SAMPLE_PERIOD(telem_ctrl);
	TELEM_ENABLE_SSRAM_EVT_TRACE(telem_ctrl);
	TELEM_ENABLE_PERIODIC(telem_ctrl);
	telem_ctrl |= TELEM_DEFAULT_SAMPLING_PERIOD;

	/* Enable telemetry via control structure */
	if (intel_punit_ipc_command(IPC_BIOS_PUNIT_CMD_WRITE_TELE_EVENT_CTRL,
			0, 0, &telem_ctrl, NULL))
		pw_pr_warn("Could not execute P-unit IPC command to write telem ctrl structure\n");

	return true;
}

static void punit_stop_telem(void)
{
	u32 telem_ctrl = 0;

	if (intel_punit_ipc_command(
			IPC_BIOS_PUNIT_CMD_READ_TELE_EVENT_CTRL, 0, 0,
			NULL, &telem_ctrl))
		pw_pr_warn("Could not execute P-unit IPC command to read telem ctrl structure\n");

	/* Disable telem */
	TELEM_DISABLE(telem_ctrl);
	if (intel_punit_ipc_command(
			IPC_BIOS_PUNIT_CMD_WRITE_TELE_EVENT_CTRL, 0, 0,
			&telem_ctrl, NULL))
		pw_pr_warn("Could not execute P-unit IPC command to write telem ctrl structure\n");
}

static bool pmc_start_telem(void)
{
	u32 telem_info = 0, telem_ctrl = 0, i;

	/* Reset data buffer */
	memset(s_telemEventInfo[TELEM_PMC].data_buffer,
		0, sizeof(s_telemEventInfo[TELEM_PMC].data_buffer));

	/* Read basic config */
	if (intel_pmc_ipc_command(PMC_IPC_PMC_TELEMETRY_COMMAND,
			IOSS_TELEM_INFO_READ, NULL, 0, &telem_info,
			IOSS_TELEM_READ_WORD))
		pw_pr_warn("Could not execute PMC IPC command to read telemetry info\n");

	pw_pr_debug("DEBUG: Read PMC telem_info = 0x%x\n", telem_info);
	pw_pr_debug("## SOCWATCHDRV ## PMC Telemetry info has events = %u\n",
		(telem_info & TELEM_INFO_SSRAMEVTS_MASK) >>
			TELEM_INFO_SSRAMEVTS_SHIFT);
	pw_pr_debug("## SOCWATCHDRV ## PMC Telemetry info has event_regs = %u\n",
		telem_info & TELEM_INFO_SSRAMEVTS_MASK);
	pw_pr_debug("## SOCWATCHDRV ## PMC Telemetry info has min_period = %u\n",
		TELEM_MIN_PERIOD(telem_info));
	pw_pr_debug("## SOCWATCHDRV ## PMC Telemetry info has max_period = %u\n",
		TELEM_MAX_PERIOD(telem_info));

	/*TODO: check if #events or #event_regs is less than 28; exit if so */

	/* Read control structure */
	if (intel_pmc_ipc_command(PMC_IPC_PMC_TELEMETRY_COMMAND,
			IOSS_TELEM_EVENT_CTL_READ, NULL, 0, &telem_ctrl,
			IOSS_TELEM_READ_WORD))
		pw_pr_warn("Could not execute PMC IPC command to read telem control info\n");

	/* Disable telemetry */
	TELEM_DISABLE(telem_ctrl);
	if (intel_pmc_ipc_command(PMC_IPC_PMC_TELEMETRY_COMMAND,
			IOSS_TELEM_EVENT_CTL_WRITE, (u8 *)&telem_ctrl,
			IOSS_TELEM_EVT_CTRL_WRITE_SIZE, NULL, 0))
		pw_pr_warn("Could not execute PMC IPC command to read telem control info\n");


	/* Each event added requires a separate command */
	for (i = 0; i < s_telemEventInfo[TELEM_PMC].idx; ++i) {
		u32 event =
			s_telemEventInfo[TELEM_PMC].events[i] |
			TELEM_EVENT_ENABLE;

		event <<= TELEM_IOSS_EVTID_SHIFT;
		event |= i; /* Set the index register */
		pw_pr_debug("DEBUG: enabling PMC event 0x%x\n",
			s_telemEventInfo[TELEM_PMC].events[i]);
		if (intel_pmc_ipc_command(PMC_IPC_PMC_TELEMETRY_COMMAND,
				IOSS_TELEM_EVENT_WRITE, (u8 *)&event,
				IOSS_TELEM_EVT_WRITE_SIZE, NULL, 0))
			pw_pr_warn("Could not execute PMC IPC command to read telem control info\n");
	}

	TELEM_CLEAR_SAMPLE_PERIOD(telem_ctrl);
	TELEM_ENABLE_SSRAM_EVT_TRACE(telem_ctrl);
	TELEM_ENABLE_PERIODIC(telem_ctrl);
	telem_ctrl |= TELEM_DEFAULT_SAMPLING_PERIOD;

	/* Enable telemetry via control structure */
	if (intel_pmc_ipc_command(PMC_IPC_PMC_TELEMETRY_COMMAND,
			IOSS_TELEM_EVENT_CTL_WRITE, (u8 *)&telem_ctrl,
			IOSS_TELEM_EVT_CTRL_WRITE_SIZE, NULL, 0))
		pw_pr_warn("Could not execute PMC IPC command to read telem control info\n");

	return true;
}

static void pmc_stop_telem(void)
{
	u32 telem_ctrl = 0;

	/* Read control structure */
	if (intel_pmc_ipc_command(PMC_IPC_PMC_TELEMETRY_COMMAND,
			IOSS_TELEM_EVENT_CTL_READ, NULL, 0, &telem_ctrl,
			IOSS_TELEM_READ_WORD))
		pw_pr_warn("Could not execute PMC IPC command to read telem control info\n");

	/* Disable telemetry */
	TELEM_DISABLE(telem_ctrl);
	if (intel_pmc_ipc_command(PMC_IPC_PMC_TELEMETRY_COMMAND,
			IOSS_TELEM_EVENT_CTL_WRITE, (u8 *)&telem_ctrl,
			IOSS_TELEM_EVT_CTRL_WRITE_SIZE, NULL, 0))
		pw_pr_warn("Could not execute PMC IPC command to read telem control info\n");
}

/**
 * Configurs events + starts counters
 * @returns  0 on success
 */
static int start_telem(void)
{
	if (s_telemEventInfo[TELEM_PUNIT].idx) {
		if (punit_start_telem() == false)
			return -1;

		/* Return value is don't care */
		flush_telem_to_buffer(TELEM_PUNIT);
	}

	if (s_telemEventInfo[TELEM_PMC].idx) {
		if (pmc_start_telem() == false)
			return -1;

		flush_telem_to_buffer(TELEM_PMC);
	}
	pw_pr_debug("OK, bypass telem started\n");
	return 0;
}

static void stop_telem(void)
{
	if (s_telemEventInfo[TELEM_PUNIT].idx) {
		punit_stop_telem();
		s_telemEventInfo[TELEM_PUNIT].idx = 0;
	}
	if (s_telemEventInfo[TELEM_PMC].idx) {
		pmc_stop_telem();
		s_telemEventInfo[TELEM_PMC].idx = 0;
	}
	pw_pr_debug("OK, bypass telem stopped\n");
}

int read_telem(u64 *dst, enum telemetry_unit unit, bool should_retry)
{
	size_t num_iters = should_retry ? 10 : 0;
	u64 timestamp = 0;

	do {
		timestamp = flush_telem_to_buffer(unit);
	} while (!timestamp && should_retry && num_iters--);

	if (timestamp) {
		read_telem_from_buffer(dst, unit);
		return 0;
	}
	return -1;
}

/**
 * builtin_telemetry_available - Determine if telemetry driver is present
 *
 * Returns: 1 if telemetry driver is present, 0 if not.
 */
static int builtin_telemetry_available(void)
{
	int retval = 0;
	struct telemetry_evtconfig punit_evtconfig;
	struct telemetry_evtconfig pmc_evtconfig;
	u32 punit_event_map[MAX_TELEM_EVENTS];
	u32 pmc_event_map[MAX_TELEM_EVENTS];


	/* The symbol below is weak.  We return 1 if we have a definition
	 * for this telemetry-driver-supplied symbol, or 0 if only the
	 * weak definition exists. This test will suffice to detect if
	 * the telemetry driver is loaded.
	 */
	if (telemetry_get_eventconfig) {
		/* OK, the telemetry driver is loaded. But it's possible it
		 * hasn't been configured properly. To check that, retrieve
		 * the number of events currently configured. This should never
		 * be zero since the telemetry driver reserves some SSRAM slots
		 * for its own use
		 */
		memset(&punit_evtconfig, 0, sizeof(punit_evtconfig));
		memset(&pmc_evtconfig, 0, sizeof(pmc_evtconfig));

		punit_evtconfig.evtmap = (u32 *) &punit_event_map;
		pmc_evtconfig.evtmap = (u32 *) &pmc_event_map;

		retval = telemetry_get_eventconfig(&punit_evtconfig, &pmc_evtconfig,
						MAX_TELEM_EVENTS, MAX_TELEM_EVENTS);
		return (retval == 0 && punit_evtconfig.num_evts > 0 &&
			pmc_evtconfig.num_evts > 0);
	}
	return 0;
}

/**
 * was_telemetry_setup - Check if the P-unit and PMC addresses have been mapped
 *
 * Returns: true if successfully mapped
 */
static bool was_telemetry_setup(void)
{
	return s_punitInterfaceAddr && s_punitDataAddr &&
		s_telemEventInfo[TELEM_PUNIT].ssram_virt_addr /* P-unit */ &&
		s_pmcIPCCmdAddr && s_pmcIPCStsAddr && s_pmcIPCWBufAddr &&
		s_pmcIPCRBufAddr && s_telemEventInfo[TELEM_PMC].ssram_virt_addr;
}


/**
 * sw_telem_init_func - Set up the telemetry unit to retrieve a data item
 *						(e.g. counter).
 * @descriptor:  The IO descriptor containing the unit and ID
 *						of the telemetry info to gather.
 *
 * Because we don't (currently) control all of the counters, we
 * economize by seeing if it's already being collected before allocate
 * a slot for it.
 *
 * Returns: PW_SUCCESS  if the telem collector can collect the requested data.
 *		 -PW_ERROR   if the the addition of that item fails.
 */
int sw_telem_init_func(struct sw_driver_io_descriptor *descriptor)
{
	struct sw_driver_telem_io_descriptor *td =
		&(descriptor->telem_descriptor);
	u8  unit = td->unit;  /* Telemetry unit to use. */
	u32 id; /* Event ID we want telemetry to track. */

	if (!was_telemetry_setup())
		return -ENXIO;

	id = (u32)(td->id);

	td->idx = add_telem_id(unit, id);
	if (td->idx < 0) {
		pw_pr_error("ERROR adding id 0x%x to unit %d\n", id, unit);
		return -1;
	}
	pw_pr_debug("OK, added id 0x%x to unit %d at pos %d\n",
			id, unit, td->idx);

	return 0;
}


/**
 * sw_read_telem_info - Read a metric's data from the telemetry driver.
 * @dest:		Destination (storage for the read data)
 * @cpu:		Which CPU to read from (not used)
 * @descriptor:		The descriptor containing the data ID to read
 * @data_size_in_bytes: The # of bytes in the result (always 8)
 *
 * Returns: Nothing, but stores SW_TELEM_READ_FAIL_VALUE to dest if
 * the read fails.
 */
void sw_read_telem_info(char *dest, int cpu,
			  const sw_driver_io_descriptor_t *descriptor,
			  u16 data_size_in_bytes)
{
	u64 *data_dest = (u64 *)dest;
	const struct sw_driver_telem_io_descriptor *td =
		&(descriptor->telem_descriptor);
	u8 unit = td->unit;
	bool needs_refresh = false;

	/*
	 * Check if we need to refresh the list of values
	 */
	LOCK(sw_telem_lock);
	{
		if (s_telemEventInfo[unit].iters == 0)
			needs_refresh = true;

		if (++s_telemEventInfo[unit].iters ==
				s_telemEventInfo[unit].idx)
			s_telemEventInfo[unit].iters = 0;
	}

	UNLOCK(sw_telem_lock);

	if (needs_refresh) {
		u64 timestamp = flush_telem_to_buffer(unit);

		pw_pr_debug("DEBUG: unit %d refreshed, timestamp = %llu\n",
			unit, timestamp);
		if (!timestamp) { /* failure */
			*data_dest = SW_TELEM_READ_FAIL_VALUE;
			return;
		}
	} else
		pw_pr_debug("DEBUG: unit %d NOT refreshed\n", unit);

	*data_dest = read_event_from_buffer(unit, td->idx);
}

/**
 * sw_reset_telem - Stop collecting telemetry info.
 * @descriptor: Unused in this function
 *
 * Stop collecting anything extra, and give the driver back to
 * debugfs.  Because this driver increases the sampling rate, the
 * kernel's telemetry driver can't succesfully reset the driver unless
 * we first drop the rate back down to a much slower rate.  This is a
 * temporary measure, since the reset operation will then reset the
 * sampling interval to whatever the GMIN driver wants.
 *
 * Returns: 0
 */
int sw_reset_telem(const struct sw_driver_io_descriptor *descriptor)
{
	if (IS_TELEM_CONFIGURED()) {
		stop_telem();
		remove_telem_ids();
		/* Return control to 'builtin' telemetry driver */
		telemetry_set_sampling_period(TELEM_SAMPLING_1S,
					  TELEM_SAMPLING_1S);
		telemetry_reset_events();
	}
	return 0;
}

/**
 * sw_available_telem -- Decide if the telemetry subsystem is available for use
 */
bool sw_telem_available(void)
{
	/*
	 * Telemetry driver MUST be loaded; we perform this check because
	 * on some systems an error with the p-unit/pmc IPC interface causes
	 * kernel panics.
	 */
	return builtin_telemetry_available();
};

bool sw_telem_post_config(void)
{
	if (start_telem())
		return false;

	return true;
}
