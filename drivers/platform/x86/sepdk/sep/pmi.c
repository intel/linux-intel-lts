/* ****************************************************************************
 *  Copyright(C) 2009-2018 Intel Corporation.  All Rights Reserved.
 *
 *  This file is part of SEP Development Kit
 *
 *  SEP Development Kit is free software; you can redistribute it
 *  and/or modify it under the terms of the GNU General Public License
 *  version 2 as published by the Free Software Foundation.
 *
 *  SEP Development Kit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  As a special exception, you may use this file as part of a free software
 *  library without restriction.  Specifically, if other files instantiate
 *  templates or use macros or inline functions from this file, or you
 *  compile this file and link it with other files to produce an executable
 *  this file does not by itself cause the resulting executable to be
 *  covered by the GNU General Public License.  This exception does not
 *  however invalidate any other reasons why the executable file might be
 *  covered by the GNU General Public License.
 * ****************************************************************************
 */

#include "lwpmudrv_defines.h"
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/ptrace.h>
#if defined(DRV_EM64T)
#include <asm/desc.h>
#endif
#include <asm/apic.h>
#include <asm/nmi.h>

#include "lwpmudrv_types.h"
#include "lwpmudrv_ecb.h"
#include "lwpmudrv_struct.h"
#include "apic.h"
#include "lwpmudrv.h"
#include "output.h"
#include "control.h"
#include "pmi.h"
#include "utility.h"
#include "pebs.h"
#include "ecb_iterators.h"
#include "msrdefs.h"

#if defined(BUILD_CHIPSET)
#include "lwpmudrv_chipset.h"
#endif
#include "sepdrv_p_state.h"

// Desc id #0 is used for module records
#define COMPUTE_DESC_ID(index) ((index))

extern DRV_CONFIG drv_cfg;
extern uid_t uid;
extern DRV_SETUP_INFO_NODE req_drv_setup_info;
#define EFLAGS_V86_MASK 0x00020000L

/*********************************************************************
 * Global Variables / State
 *********************************************************************/

/*********************************************************************
 * Interrupt Handler
 *********************************************************************/

/*
 *  PMI_Interrupt_Handler
 *      Arguments
 *          IntFrame - Pointer to the Interrupt Frame
 *
 *      Returns
 *          None
 *
 *      Description
 *  Grab the data that is needed to populate the sample records
 */
#if defined(DRV_EM64T)
#define IS_LDT_BIT 0x4
#define SEGMENT_SHIFT 3
IDTGDT_DESC gdt_desc;

U32 pmi_Get_CSD(U32 seg, U32 *low, U32 *high)
{
	PVOID gdt_max_addr;
	struct desc_struct *gdt;
	CodeDescriptor *csd;

	SEP_DRV_LOG_TRACE_IN("Seg: %u, low: %p, high: %p.", seg, low, high);

	gdt_max_addr =
		(PVOID)(((U64)gdt_desc.idtgdt_base) + gdt_desc.idtgdt_limit);
	gdt = gdt_desc.idtgdt_base;

	if (seg & IS_LDT_BIT) {
		*low = 0;
		*high = 0;
		SEP_DRV_LOG_TRACE_OUT("FALSE [%u, %u] (IS_LDT_BIT).", *low,
				      *high);
		return FALSE;
	}

	// segment offset is based on dropping the bottom 3 bits...
	csd = (CodeDescriptor *)&(gdt[seg >> SEGMENT_SHIFT]);

	if (((PVOID)csd) >= gdt_max_addr) {
		SEP_DRV_LOG_WARNING_TRACE_OUT(
			"FALSE (segment too big in get_CSD(0x%x)!).", seg);
		return FALSE;
	}

	*low = csd->u1.lowWord;
	*high = csd->u2.highWord;

	SEP_DRV_LOG_TRACE("Seg 0x%x, low %08x, high %08x, reserved_0: %d.", seg,
			  *low, *high, csd->u2.s2.reserved_0);
	SEP_DRV_LOG_TRACE_OUT("TRUE [%u, %u].", *low, *high);

	return TRUE;
}
#endif

asmlinkage VOID PMI_Interrupt_Handler(struct pt_regs *regs)
{
	SampleRecordPC *psamp;
	CPU_STATE pcpu;
	BUFFER_DESC bd;
#if defined(DRV_IA32)
	U32 csdlo; // low  half code seg descriptor
	U32 csdhi; // high half code seg descriptor
	U32 seg_cs; // code seg selector
#endif
	DRV_MASKS_NODE event_mask;
	U32 this_cpu;
	U32 dev_idx;
	DISPATCH dispatch;
	DEV_CONFIG pcfg;
	U32 i;
	U32 is_64bit_addr = FALSE;
	U32 pid;
	U32 tid;
	U64 tsc;
	U32 desc_id;
	EVENT_DESC evt_desc;
	U32 accept_interrupt = 1;
#if defined(SECURE_SEP)
	uid_t l_uid;
#endif
	U64 lbr_tos_from_ip = 0;
	DRV_BOOL multi_pebs_enabled;

	SEP_DRV_LOG_INTERRUPT_IN(
		"PID: %d, TID: %d.", current->pid,
		GET_CURRENT_TGID()); // needs to be before function calls for the tracing to make sense
	// may later want to separate the INTERRUPT_IN from the PID/TID logging

	this_cpu = CONTROL_THIS_CPU();
	pcpu = &pcb[this_cpu];
	bd = &cpu_buf[this_cpu];
	dev_idx = core_to_dev_map[this_cpu];
	dispatch = LWPMU_DEVICE_dispatch(&devices[dev_idx]);
	pcfg = LWPMU_DEVICE_pcfg(&devices[dev_idx]);
	multi_pebs_enabled =
		(DEV_CONFIG_pebs_mode(pcfg) &&
		 (DEV_CONFIG_pebs_record_num(pcfg) > 1) &&
		 (DRV_SETUP_INFO_page_table_isolation(&req_drv_setup_info) ==
		  DRV_SETUP_INFO_PTI_DISABLED));
	SYS_Locked_Inc(&CPU_STATE_in_interrupt(
		pcpu)); // needs to be before dispatch->freeze to ensure printk is never called from an interrupt

	// Disable the counter control
	dispatch->freeze(NULL);

	CPU_STATE_nmi_handled(&pcb[this_cpu])++;

#if defined(SECURE_SEP)
	l_uid = DRV_GET_UID(current);
	accept_interrupt = (l_uid == uid);
#endif
	dispatch->check_overflow(&event_mask);
	if (GET_DRIVER_STATE() != DRV_STATE_RUNNING ||
	    CPU_STATE_accept_interrupt(&pcb[this_cpu]) != 1) {
		goto pmi_cleanup;
	}

	pid = GET_CURRENT_TGID();
	tid = current->pid;

	if (DRV_CONFIG_target_pid(drv_cfg) > 0 &&
	    pid != DRV_CONFIG_target_pid(drv_cfg)) {
		accept_interrupt = 0;
	}

	if (accept_interrupt == 0) {
		goto pmi_cleanup;
	}
	UTILITY_Read_TSC(&tsc);
	if (multi_pebs_enabled && PEBS_Get_Num_Records_Filled() > 0) {
		PEBS_Flush_Buffer(NULL);
	}

	SEP_DRV_LOG_TRACE("Nb overflowed events: %d.", event_mask.masks_num);
	for (i = 0; i < event_mask.masks_num; i++) {
		if (multi_pebs_enabled &&
		    (DRV_EVENT_MASK_precise(&event_mask.eventmasks[i]))) {
			continue;
		}
		if (DRV_CONFIG_event_based_counts(drv_cfg) == 0) {
			desc_id = COMPUTE_DESC_ID(DRV_EVENT_MASK_event_idx(
				&event_mask.eventmasks[i]));
		} else {
			desc_id = CPU_STATE_current_group(pcpu);
		}
		evt_desc = desc_data[desc_id];
		psamp = (SampleRecordPC *)OUTPUT_Reserve_Buffer_Space(
			bd, EVENT_DESC_sample_size(evt_desc),
			(NMI_mode) ? TRUE : FALSE, !SEP_IN_NOTIFICATION,
			(S32)this_cpu);

		if (!psamp) {
			continue;
		}
		lbr_tos_from_ip = 0;
		CPU_STATE_num_samples(pcpu) += 1;
		SAMPLE_RECORD_descriptor_id(psamp) = desc_id;
		SAMPLE_RECORD_tsc(psamp) = tsc;
		SAMPLE_RECORD_pid_rec_index_raw(psamp) = 1;
		SAMPLE_RECORD_pid_rec_index(psamp) = pid;
		SAMPLE_RECORD_tid(psamp) = tid;
		SAMPLE_RECORD_cpu_num(psamp) = (U16)this_cpu;
#if defined(DRV_IA32)
		SAMPLE_RECORD_eip(psamp) = REGS_eip(regs);
		SAMPLE_RECORD_eflags(psamp) = REGS_eflags(regs);
		SAMPLE_RECORD_cs(psamp) = (U16)REGS_xcs(regs);

		if (SAMPLE_RECORD_eflags(psamp) & EFLAGS_V86_MASK) {
			csdlo = 0;
			csdhi = 0;
		} else {
			seg_cs = SAMPLE_RECORD_cs(psamp);
			SYS_Get_CSD(seg_cs, &csdlo, &csdhi);
		}
		SAMPLE_RECORD_csd(psamp).u1.lowWord = csdlo;
		SAMPLE_RECORD_csd(psamp).u2.highWord = csdhi;
#elif defined(DRV_EM64T)
		SAMPLE_RECORD_cs(psamp) = (U16)REGS_cs(regs);

		pmi_Get_CSD(SAMPLE_RECORD_cs(psamp),
			    &SAMPLE_RECORD_csd(psamp).u1.lowWord,
			    &SAMPLE_RECORD_csd(psamp).u2.highWord);
#endif
		SEP_DRV_LOG_TRACE("SAMPLE_RECORD_pid_rec_index(psamp) %x.",
				  SAMPLE_RECORD_pid_rec_index(psamp));
		SEP_DRV_LOG_TRACE("SAMPLE_RECORD_tid(psamp) %x.",
				  SAMPLE_RECORD_tid(psamp));
#if defined(DRV_IA32)
		SEP_DRV_LOG_TRACE("SAMPLE_RECORD_eip(psamp) %x.",
				  SAMPLE_RECORD_eip(psamp));
		SEP_DRV_LOG_TRACE("SAMPLE_RECORD_eflags(psamp) %x.",
				  SAMPLE_RECORD_eflags(psamp));
#endif
		SEP_DRV_LOG_TRACE("SAMPLE_RECORD_cpu_num(psamp) %x.",
				  SAMPLE_RECORD_cpu_num(psamp));
		SEP_DRV_LOG_TRACE("SAMPLE_RECORD_cs(psamp) %x.",
				  SAMPLE_RECORD_cs(psamp));
		SEP_DRV_LOG_TRACE("SAMPLE_RECORD_csd(psamp).lowWord %x.",
				  SAMPLE_RECORD_csd(psamp).u1.lowWord);
		SEP_DRV_LOG_TRACE("SAMPLE_RECORD_csd(psamp).highWord %x.",
				  SAMPLE_RECORD_csd(psamp).u2.highWord);

#if defined(DRV_EM64T)
		is_64bit_addr =
			(SAMPLE_RECORD_csd(psamp).u2.s2.reserved_0 == 1);
		if (is_64bit_addr) {
			SAMPLE_RECORD_iip(psamp) = REGS_rip(regs);
			SAMPLE_RECORD_ipsr(psamp) =
				(REGS_eflags(regs) & 0xffffffff) |
				(((U64)SAMPLE_RECORD_csd(psamp).u2.s2.dpl)
				 << 32);
			SAMPLE_RECORD_ia64_pc(psamp) = TRUE;
		} else {
			SAMPLE_RECORD_eip(psamp) = REGS_rip(regs);
			SAMPLE_RECORD_eflags(psamp) = REGS_eflags(regs);
			SAMPLE_RECORD_ia64_pc(psamp) = FALSE;

			SEP_DRV_LOG_TRACE("SAMPLE_RECORD_eip(psamp) 0x%x.",
					  SAMPLE_RECORD_eip(psamp));
			SEP_DRV_LOG_TRACE("SAMPLE_RECORD_eflags(psamp) %x.",
					  SAMPLE_RECORD_eflags(psamp));
		}
#endif

		SAMPLE_RECORD_event_index(psamp) =
			DRV_EVENT_MASK_event_idx(&event_mask.eventmasks[i]);
		if (DEV_CONFIG_pebs_mode(pcfg) &&
		    DRV_EVENT_MASK_precise(&event_mask.eventmasks[i])) {
			if (EVENT_DESC_pebs_offset(evt_desc) ||
			    EVENT_DESC_latency_offset_in_sample(evt_desc)) {
				lbr_tos_from_ip = PEBS_Fill_Buffer((S8 *)psamp,
								   evt_desc, 0);
			}
			PEBS_Modify_IP((S8 *)psamp, is_64bit_addr, 0);
			PEBS_Modify_TSC((S8 *)psamp, 0);
		}
		if (DEV_CONFIG_collect_lbrs(pcfg) &&
		    DRV_EVENT_MASK_lbr_capture(&event_mask.eventmasks[i]) &&
		    !DEV_CONFIG_apebs_collect_lbrs(pcfg)) {
			lbr_tos_from_ip = dispatch->read_lbrs(
				!DEV_CONFIG_store_lbrs(pcfg) ?
					NULL :
					((S8 *)(psamp) +
					 EVENT_DESC_lbr_offset(evt_desc)),
				NULL);
		}
		if (DRV_EVENT_MASK_branch(&event_mask.eventmasks[i]) &&
		    DEV_CONFIG_precise_ip_lbrs(pcfg) && lbr_tos_from_ip) {
			if (is_64bit_addr) {
				SAMPLE_RECORD_iip(psamp) = lbr_tos_from_ip;
				SEP_DRV_LOG_TRACE(
					"UPDATED SAMPLE_RECORD_iip(psamp) 0x%llx.",
					SAMPLE_RECORD_iip(psamp));
			} else {
				SAMPLE_RECORD_eip(psamp) = (U32)lbr_tos_from_ip;
				SEP_DRV_LOG_TRACE(
					"UPDATED SAMPLE_RECORD_eip(psamp) 0x%x.",
					SAMPLE_RECORD_eip(psamp));
			}
		}
		if (DEV_CONFIG_power_capture(pcfg)) {
			dispatch->read_power(
				((S8 *)(psamp) +
				 EVENT_DESC_power_offset_in_sample(evt_desc)));
		}

#if defined(BUILD_CHIPSET)
		if (DRV_CONFIG_enable_chipset(drv_cfg)) {
			cs_dispatch->read_counters(
				((S8 *)(psamp) +
				 DRV_CONFIG_chipset_offset(drv_cfg)));
		}
#endif
		if (DRV_CONFIG_event_based_counts(drv_cfg)) {
			dispatch->read_counts(
				(S8 *)psamp,
				DRV_EVENT_MASK_event_idx(
					&event_mask.eventmasks[i]));
		}
		if (DEV_CONFIG_enable_perf_metrics(pcfg) &&
		    DRV_EVENT_MASK_perf_metrics_capture(
			    &event_mask.eventmasks[i])) {
			dispatch->read_metrics(
				(S8 *)(psamp) +
				EVENT_DESC_perfmetrics_offset(evt_desc));
		}
		if (DRV_CONFIG_enable_p_state(drv_cfg)) {
			if (DRV_CONFIG_read_pstate_msrs(drv_cfg) &&
			    (DRV_CONFIG_p_state_trigger_index(drv_cfg) == -1 ||
			     SAMPLE_RECORD_event_index(psamp) ==
				     DRV_CONFIG_p_state_trigger_index(
					     drv_cfg))) {
				SEPDRV_P_STATE_Read(
					(S8 *)(psamp) +
						EVENT_DESC_p_state_offset(
							evt_desc),
					pcpu);
			}
			if (!DRV_CONFIG_event_based_counts(drv_cfg) &&
			    CPU_STATE_p_state_counting(pcpu)) {
				dispatch->read_counts(
					(S8 *)psamp,
					DRV_EVENT_MASK_event_idx(
						&event_mask.eventmasks[i]));
			}
		}
	}

pmi_cleanup:
	if (DEV_CONFIG_pebs_mode(pcfg)) {
		if (!multi_pebs_enabled) {
			PEBS_Reset_Index(this_cpu);
		} else {
			if (cpu_sideband_buf) {
				OUTPUT outbuf = &BUFFER_DESC_outbuf(
					&cpu_sideband_buf[this_cpu]);
				if (OUTPUT_signal_full(outbuf) &&
				    !OUTPUT_tasklet_queued(outbuf)) {
					SEP_DRV_LOG_TRACE(
						"Interrupt-driven sideband buffer flush tasklet scheduling.");
					OUTPUT_tasklet_queued(outbuf) = TRUE;
					tasklet_schedule(&CPU_STATE_nmi_tasklet(
						&pcb[this_cpu]));
				}
			}
		}
	}

	// Reset the data counters
	if (CPU_STATE_trigger_count(&pcb[this_cpu]) == 0) {
		dispatch->swap_group(FALSE);
	}
	// Re-enable the counter control
	dispatch->restart(NULL);
	SYS_Locked_Dec(&CPU_STATE_in_interrupt(
		&pcb[this_cpu])); // do not use SEP_DRV_LOG_X (where X != INTERRUPT) below this

	SEP_DRV_LOG_INTERRUPT_OUT("");
}

#if defined(DRV_SEP_ACRN_ON)
/* ------------------------------------------------------------------------- */
/*!
 * @fn  S32 PMI_Buffer_Handler(PVOID data)
 *
 * @param data - Pointer to data
 *
 * @return S32
 *
 * @brief Handle the PMI sample data in buffer
 *
 * <I>Special Notes</I>
 */
S32 PMI_Buffer_Handler(PVOID data)
{
	SampleRecordPC *psamp;
	CPU_STATE pcpu;
	BUFFER_DESC bd;
	S32 cpu_id, j;
	U32 desc_id;
	EVENT_DESC evt_desc;
	U64 lbr_tos_from_ip = 0;
	ECB pecb;
	U32 dev_idx;
	DISPATCH dispatch;
	DEV_CONFIG pcfg;

	struct data_header header;
	struct pmu_sample psample;
	S32 data_size, payload_size, expected_payload_size, index;
	U64 overflow_status = 0;

	if (!pcb || !cpu_buf || !devices) {
		return 0;
	}
	cpu_id = (S32)(size_t)data;

	pcpu = &pcb[cpu_id];
	bd = &cpu_buf[cpu_id];
	dev_idx = core_to_dev_map[cpu_id];
	dispatch = LWPMU_DEVICE_dispatch(&devices[dev_idx]);
	pcfg = LWPMU_DEVICE_pcfg(&devices[dev_idx]);
	pecb = LWPMU_DEVICE_PMU_register_data(
		&devices[dev_idx])[CPU_STATE_current_group(pcpu)];

	while (1) {
		if ((GLOBAL_STATE_current_phase(driver_state) ==
		     DRV_STATE_PREPARE_STOP) ||
		    (GLOBAL_STATE_current_phase(driver_state) ==
		     DRV_STATE_TERMINATING) ||
		    (GLOBAL_STATE_current_phase(driver_state) ==
		     DRV_STATE_STOPPED)) {
			goto handler_cleanup;
		}

		data_size =
			sbuf_get(samp_buf_per_cpu[cpu_id], (uint8_t *)&header);
		if (data_size <= 0) {
			continue;
		}
		payload_size = 0;
		if ((header.data_type == (1 << CORE_PMU_SAMPLING)) ||
		    (header.data_type == (1 << LBR_PMU_SAMPLING))) {
			if (header.data_type == (1 << CORE_PMU_SAMPLING)) {
				expected_payload_size = CORE_PMU_SAMPLE_SIZE;
			} else if (header.data_type ==
				   (1 << LBR_PMU_SAMPLING)) {
				expected_payload_size = CORE_PMU_SAMPLE_SIZE +
							LBR_PMU_SAMPLE_SIZE;
			} else {
				expected_payload_size = 0;
			}
			for (j = 0; j < (expected_payload_size - 1) /
							TRACE_ELEMENT_SIZE +
						1;
			     j++) {
				while (1) {
					data_size = sbuf_get(
						samp_buf_per_cpu[cpu_id],
						(uint8_t *)&psample +
							j * TRACE_ELEMENT_SIZE);
					if (data_size <= 0) {
						if ((GLOBAL_STATE_current_phase(
							     driver_state) ==
						     DRV_STATE_PREPARE_STOP) ||
						    (GLOBAL_STATE_current_phase(
							     driver_state) ==
						     DRV_STATE_TERMINATING) ||
						    (GLOBAL_STATE_current_phase(
							     driver_state) ==
						     DRV_STATE_STOPPED)) {
							goto handler_cleanup;
						}
					} else {
						break;
					}
				}

				payload_size += data_size;
			}
			if (header.payload_size > payload_size) {
				// Mismatch in payload size in header info
				SEP_PRINT_ERROR(
					"Mismatch in data size: header=%llu, payload_size=%d\n",
					header.payload_size, payload_size);
				break;
			}
			if (header.cpu_id != cpu_id) {
				// Mismatch in cpu index in header info
				SEP_PRINT_ERROR(
					"Mismatch in cpu idx: header=%u, buffer=%d\n",
					header.cpu_id, cpu_id);
				break;
			}

			// Now, handle the sample data in buffer
			overflow_status = psample.csample.overflow_status;
			SEP_PRINT_DEBUG("overflow_status cpu%d, value=0x%llx\n",
					cpu_id, overflow_status);

			FOR_EACH_DATA_REG_CPU(pecb, i, cpu_id)
			{
				if (ECB_entries_is_gp_reg_get(pecb, i)) {
					index = ECB_entries_reg_id(pecb, i) -
						IA32_PMC0;
				} else if (ECB_entries_fixed_reg_get(pecb, i)) {
					index = ECB_entries_reg_id(pecb, i) -
						IA32_FIXED_CTR0 + 0x20;
				} else {
					continue;
				}

				if (overflow_status & ((U64)1 << index)) {
					desc_id = COMPUTE_DESC_ID(
						ECB_entries_event_id_index(pecb,
									   i));
					evt_desc = desc_data[desc_id];
					SEP_PRINT_DEBUG(
						"In Interrupt handler: event_id_index=%u, desc_id=%u\n",
						ECB_entries_event_id_index(pecb,
									   i),
						desc_id);

					psamp = (SampleRecordPC *)
						OUTPUT_Reserve_Buffer_Space(
							bd,
							EVENT_DESC_sample_size(
								evt_desc),
							TRUE,
							!SEP_IN_NOTIFICATION,
							cpu_id);
					if (!psamp) {
						SEP_PRINT_DEBUG(
							"In Interrupt handler: psamp is NULL. No output buffer allocated\n");
						continue;
					}

					CPU_STATE_num_samples(pcpu) += 1;
					SAMPLE_RECORD_descriptor_id(psamp) =
						desc_id;
					SAMPLE_RECORD_event_index(psamp) =
						ECB_entries_event_id_index(pecb,
									   i);
					SAMPLE_RECORD_osid(psamp) =
						psample.csample.os_id;
					SAMPLE_RECORD_tsc(psamp) = header.tsc;
					SAMPLE_RECORD_pid_rec_index_raw(psamp) =
						1;
					SAMPLE_RECORD_pid_rec_index(psamp) = 0;
					SAMPLE_RECORD_pid_rec_index(psamp) = 0;
					SAMPLE_RECORD_tid(psamp) = 0;
					SAMPLE_RECORD_cpu_num(psamp) =
						(U16)header.cpu_id;
					SAMPLE_RECORD_cs(psamp) =
						(U16)psample.csample.cs;

					SAMPLE_RECORD_iip(psamp) =
						psample.csample.rip;
					SAMPLE_RECORD_ipsr(psamp) =
						(psample.csample.rflags &
						 0xffffffff) |
						(((U64)SAMPLE_RECORD_csd(psamp)
							  .u2.s2.dpl)
						 << 32);
					SAMPLE_RECORD_ia64_pc(psamp) = TRUE;

					if (DEV_CONFIG_collect_lbrs(pcfg) &&

					    !DEV_CONFIG_apebs_collect_lbrs(
						    pcfg) &&
					    header.data_type ==
						    (1 << LBR_PMU_SAMPLING)) {
						lbr_tos_from_ip = dispatch->read_lbrs(
							!DEV_CONFIG_store_lbrs(
								pcfg) ?
								NULL :
								((S8 *)(psamp) +
								 EVENT_DESC_lbr_offset(
									 evt_desc)),
							&psample.lsample);
					}

					SEP_PRINT_DEBUG(
						"SAMPLE_RECORD_cpu_num(psamp) %x\n",
						SAMPLE_RECORD_cpu_num(psamp));
					SEP_PRINT_DEBUG(
						"SAMPLE_RECORD_iip(psamp) %x\n",
						SAMPLE_RECORD_iip(psamp));
					SEP_PRINT_DEBUG(
						"SAMPLE_RECORD_cs(psamp) %x\n",
						SAMPLE_RECORD_cs(psamp));
					SEP_PRINT_DEBUG(
						"SAMPLE_RECORD_csd(psamp).lowWord %x\n",
						SAMPLE_RECORD_csd(psamp)
							.u1.lowWord);
					SEP_PRINT_DEBUG(
						"SAMPLE_RECORD_csd(psamp).highWord %x\n",
						SAMPLE_RECORD_csd(psamp)
							.u2.highWord);
				}
			}
			END_FOR_EACH_DATA_REG_CPU;
		}
	}

handler_cleanup:
	return 0;
}
#endif
