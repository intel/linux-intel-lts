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

#ifndef _LWPMUDRV_STRUCT_UTILS_H_
#define _LWPMUDRV_STRUCT_UTILS_H_

#if defined(__cplusplus)
extern "C" {
#endif

// processor execution modes
#define MODE_UNKNOWN 99
// the following defines must start at 0
#define MODE_64BIT 3
#define MODE_32BIT 2
#define MODE_16BIT 1
#define MODE_V86 0

// sampling methods
#define SM_RTC 2020 // real time clock
#define SM_VTD 2021 // OS Virtual Timer Device
#define SM_NMI 2022 // non-maskable interrupt time based
#define SM_EBS 2023 // event based sampling
#define SM_EBC 2024 // event based counting

// sampling mechanism bitmap definitions
#define INTERRUPT_RTC 0x1
#define INTERRUPT_VTD 0x2
#define INTERRUPT_NMI 0x4
#define INTERRUPT_EBS 0x8

// Device types
#define DEV_CORE 0x01
#define DEV_UNC 0x02

// eflags defines
#define EFLAGS_VM 0x00020000 // V86 mode
#define EFLAGS_IOPL0 0
#define EFLAGS_IOPL1 0x00001000
#define EFLAGS_IOPL2 0x00002000
#define EFLAGS_IOPL3 0x00003000
#define MAX_EMON_GROUPS 1000
#define MAX_PCI_BUSNO 256
#define MAX_DEVICES 30
#define MAX_REGS 64
#define MAX_EMON_GROUPS 1000
#define MAX_PCI_DEVNO 32
#define MAX_PCI_FUNCNO 8
#define MAX_PCI_DEVUNIT 16
#define MAX_TURBO_VALUES 32
#define REG_BIT_MASK 0xFFFFFFFFFFFFFFFFULL

extern float freq_multiplier;

// Enumeration for invoking dispatch on multiple cpus or not
typedef enum { DRV_MULTIPLE_INSTANCE = 0, DRV_SINGLE_INSTANCE } DRV_PROG_TYPE;

typedef struct DRV_CONFIG_NODE_S DRV_CONFIG_NODE;
typedef DRV_CONFIG_NODE * DRV_CONFIG;

struct DRV_CONFIG_NODE_S {
	U32 size;
	U16 version;
	U16 reserved1;
	U32 num_events;
	U32 num_chipset_events;
	U32 chipset_offset;
	S32 seed_name_len;
	union {
		S8 *seed_name;
		U64 dummy1;
	} u1;
	union {
		S8 *cpu_mask;
		U64 dummy2;
	} u2;
	union {
		U64 collection_config;
		struct {
			U64 start_paused : 1;
			U64 counting_mode : 1;
			U64 enable_chipset : 1;
			U64 enable_gfx : 1;
			U64 enable_pwr : 1;
			U64 emon_mode : 1;
			U64 debug_inject : 1;
			U64 virt_phys_translation : 1;
			U64 enable_p_state : 1;
			U64 enable_cp_mode : 1;
			U64 read_pstate_msrs : 1;
			U64 use_pcl : 1;
			U64 enable_ebc : 1;
			U64 enable_tbc : 1;
			U64 ds_area_available : 1;
			U64 per_cpu_tsc : 1;
			U64 reserved_field1 : 48;
		} s1;
	} u3;
	U64 target_pid;
	U32 os_of_interest;
	U16 unc_timer_interval;
	U16 unc_em_factor;
	S32 p_state_trigger_index;
	DRV_BOOL multi_pebs_enabled;
	U32 reserved2;
	U32 reserved3;
	U64 reserved4;
	U64 reserved5;
	U64 reserved6;
};

#define DRV_CONFIG_size(cfg) ((cfg)->size)
#define DRV_CONFIG_version(cfg) ((cfg)->version)
#define DRV_CONFIG_num_events(cfg) ((cfg)->num_events)
#define DRV_CONFIG_num_chipset_events(cfg) ((cfg)->num_chipset_events)
#define DRV_CONFIG_chipset_offset(cfg) ((cfg)->chipset_offset)

#define DRV_CONFIG_seed_name(cfg) ((cfg)->u1.seed_name)
#define DRV_CONFIG_seed_name_len(cfg) ((cfg)->seed_name_len)
#define DRV_CONFIG_cpu_mask(cfg) ((cfg)->u2.cpu_mask)
#define DRV_CONFIG_start_paused(cfg) ((cfg)->u3.s1.start_paused)
#define DRV_CONFIG_counting_mode(cfg) ((cfg)->u3.s1.counting_mode)
#define DRV_CONFIG_enable_chipset(cfg) ((cfg)->u3.s1.enable_chipset)
#define DRV_CONFIG_enable_gfx(cfg) ((cfg)->u3.s1.enable_gfx)
#define DRV_CONFIG_enable_pwr(cfg) ((cfg)->u3.s1.enable_pwr)
#define DRV_CONFIG_emon_mode(cfg) ((cfg)->u3.s1.emon_mode)
#define DRV_CONFIG_debug_inject(cfg) ((cfg)->u3.s1.debug_inject)
#define DRV_CONFIG_virt_phys_translation(cfg)                                  \
	((cfg)->u3.s1.virt_phys_translation)
#define DRV_CONFIG_enable_p_state(cfg) ((cfg)->u3.s1.enable_p_state)
#define DRV_CONFIG_enable_cp_mode(cfg) ((cfg)->u3.s1.enable_cp_mode)
#define DRV_CONFIG_read_pstate_msrs(cfg) ((cfg)->u3.s1.read_pstate_msrs)
#define DRV_CONFIG_use_pcl(cfg) ((cfg)->u3.s1.use_pcl)
#define DRV_CONFIG_event_based_counts(cfg) ((cfg)->u3.s1.enable_ebc)
#define DRV_CONFIG_timer_based_counts(cfg) ((cfg)->u3.s1.enable_tbc)
#define DRV_CONFIG_ds_area_available(cfg) ((cfg)->u3.s1.ds_area_available)
#define DRV_CONFIG_per_cpu_tsc(cfg) ((cfg)->u3.s1.per_cpu_tsc)
#define DRV_CONFIG_target_pid(cfg) ((cfg)->target_pid)
#define DRV_CONFIG_os_of_interest(cfg) ((cfg)->os_of_interest)
#define DRV_CONFIG_unc_timer_interval(cfg) ((cfg)->unc_timer_interval)
#define DRV_CONFIG_unc_em_factor(cfg) ((cfg)->unc_em_factor)
#define DRV_CONFIG_p_state_trigger_index(cfg) ((cfg)->p_state_trigger_index)
#define DRV_CONFIG_multi_pebs_enabled(cfg) ((cfg)->multi_pebs_enabled)

#define DRV_CONFIG_VERSION 1

typedef struct DEV_CONFIG_NODE_S DEV_CONFIG_NODE;
typedef DEV_CONFIG_NODE * DEV_CONFIG;

struct DEV_CONFIG_NODE_S {
	U16 size;
	U16 version;
	U32 dispatch_id;
	U32 pebs_mode;
	U32 pebs_record_num;
	U32 results_offset; // to store the offset for this device's results
	U32 max_gp_counters;
	U32 device_type;
	U32 core_type;
	union {
		U64 enable_bit_fields;
		struct {
			U64 pebs_capture : 1;
			U64 collect_lbrs : 1;
			U64 collect_callstacks : 1;
			U64 collect_kernel_callstacks : 1;
			U64 latency_capture : 1;
			U64 power_capture : 1;
			U64 htoff_mode : 1;
			U64 eventing_ip_capture : 1;
			U64 hle_capture : 1;
			U64 precise_ip_lbrs : 1;
			U64 store_lbrs : 1;
			U64 tsc_capture : 1;
			U64 enable_perf_metrics : 1;
			U64 enable_adaptive_pebs : 1;
			U64 apebs_collect_mem_info : 1;
			U64 apebs_collect_gpr : 1;
			U64 apebs_collect_xmm : 1;
			U64 apebs_collect_lbrs : 1;
			U64 collect_fixed_counter_pebs : 1;
			U64 collect_os_callstacks : 1;
			U64 reserved_field1 : 44;
		} s1;
	} u1;
	U32 emon_unc_offset[MAX_EMON_GROUPS];
	U32 ebc_group_id_offset;
	U8 num_perf_metrics;
	U8 apebs_num_lbr_entries;
	U16 emon_perf_metrics_offset;
	U32 device_scope;
	U32 reserved1;
	U64 reserved2;
	U64 reserved3;
	U64 reserved4;
};

#define DEV_CONFIG_dispatch_id(cfg) ((cfg)->dispatch_id)
#define DEV_CONFIG_pebs_mode(cfg) ((cfg)->pebs_mode)
#define DEV_CONFIG_pebs_record_num(cfg) ((cfg)->pebs_record_num)
#define DEV_CONFIG_results_offset(cfg) ((cfg)->results_offset)
#define DEV_CONFIG_max_gp_counters(cfg) ((cfg)->max_gp_counters)

#define DEV_CONFIG_device_type(cfg) ((cfg)->device_type)
#define DEV_CONFIG_core_type(cfg) ((cfg)->core_type)

#define DEV_CONFIG_pebs_capture(cfg) ((cfg)->u1.s1.pebs_capture)
#define DEV_CONFIG_collect_lbrs(cfg) ((cfg)->u1.s1.collect_lbrs)
#define DEV_CONFIG_collect_callstacks(cfg) ((cfg)->u1.s1.collect_callstacks)
#define DEV_CONFIG_collect_kernel_callstacks(cfg)                              \
	((cfg)->u1.s1.collect_kernel_callstacks)
#define DEV_CONFIG_latency_capture(cfg) ((cfg)->u1.s1.latency_capture)
#define DEV_CONFIG_power_capture(cfg) ((cfg)->u1.s1.power_capture)
#define DEV_CONFIG_htoff_mode(cfg) ((cfg)->u1.s1.htoff_mode)
#define DEV_CONFIG_eventing_ip_capture(cfg) ((cfg)->u1.s1.eventing_ip_capture)
#define DEV_CONFIG_hle_capture(cfg) ((cfg)->u1.s1.hle_capture)
#define DEV_CONFIG_precise_ip_lbrs(cfg) ((cfg)->u1.s1.precise_ip_lbrs)
#define DEV_CONFIG_store_lbrs(cfg) ((cfg)->u1.s1.store_lbrs)
#define DEV_CONFIG_tsc_capture(cfg) ((cfg)->u1.s1.tsc_capture)
#define DEV_CONFIG_enable_perf_metrics(cfg) ((cfg)->u1.s1.enable_perf_metrics)
#define DEV_CONFIG_enable_adaptive_pebs(cfg) ((cfg)->u1.s1.enable_adaptive_pebs)
#define DEV_CONFIG_apebs_collect_mem_info(cfg)                                 \
	((cfg)->u1.s1.apebs_collect_mem_info)
#define DEV_CONFIG_apebs_collect_gpr(cfg) ((cfg)->u1.s1.apebs_collect_gpr)
#define DEV_CONFIG_apebs_collect_xmm(cfg) ((cfg)->u1.s1.apebs_collect_xmm)
#define DEV_CONFIG_apebs_collect_lbrs(cfg) ((cfg)->u1.s1.apebs_collect_lbrs)
#define DEV_CONFIG_collect_fixed_counter_pebs(cfg)                             \
	((cfg)->u1.s1.collect_fixed_counter_pebs)
#define DEV_CONFIG_collect_os_callstacks(cfg)                                  \
	((cfg)->u1.s1.collect_os_callstacks)
#define DEV_CONFIG_enable_bit_fields(cfg) ((cfg)->u1.enable_bit_fields)
#define DEV_CONFIG_emon_unc_offset(cfg, grp_num)                               \
	((cfg)->emon_unc_offset[grp_num])
#define DEV_CONFIG_ebc_group_id_offset(cfg) ((cfg)->ebc_group_id_offset)
#define DEV_CONFIG_num_perf_metrics(cfg) ((cfg)->num_perf_metrics)
#define DEV_CONFIG_apebs_num_lbr_entries(cfg) ((cfg)->apebs_num_lbr_entries)
#define DEV_CONFIG_emon_perf_metrics_offset(cfg)                               \
	((cfg)->emon_perf_metrics_offset)
#define DEV_CONFIG_device_scope(cfg) ((cfg)->device_scope)

typedef struct DEV_UNC_CONFIG_NODE_S DEV_UNC_CONFIG_NODE;
typedef DEV_UNC_CONFIG_NODE * DEV_UNC_CONFIG;

struct DEV_UNC_CONFIG_NODE_S {
	U16 size;
	U16 version;
	U32 dispatch_id;
	U32 results_offset;
	U32 device_type;
	U32 device_scope;
	U32 reserved1;
	U32 emon_unc_offset[MAX_EMON_GROUPS];
	U64 reserved2;
	U64 reserved3;
	U64 reserved4;
};

#define DEV_UNC_CONFIG_dispatch_id(cfg) ((cfg)->dispatch_id)
#define DEV_UNC_CONFIG_results_offset(cfg) ((cfg)->results_offset)
#define DEV_UNC_CONFIG_emon_unc_offset(cfg, grp_num)                           \
	((cfg)->emon_unc_offset[grp_num])
#define DEV_UNC_CONFIG_device_type(cfg) ((cfg)->device_type)
#define DEV_UNC_CONFIG_device_scope(cfg) ((cfg)->device_scope)

/*
 *    X86 processor code descriptor
 */
typedef struct CodeDescriptor_s {
	union {
		U32 lowWord; // low dword of descriptor
		struct { // low broken out by fields
			U16 limitLow; // segment limit 15:00
			U16 baseLow; // segment base 15:00
		} s1;
	} u1;
	union {
		U32 highWord; // high word of descriptor
		struct { // high broken out by bit fields
			U32 baseMid : 8; // base 23:16
			U32 accessed : 1; // accessed
			U32 readable : 1; // readable
			U32 conforming : 1; // conforming code segment
			U32 oneOne : 2; // always 11
			U32 dpl : 2; // Dpl
			U32 pres : 1; // present bit
			U32 limitHi : 4; // limit 19:16
			U32 sys : 1; // available for use by system
			U32 reserved_0 : 1; // reserved, always 0
			U32 default_size : 1;
			// default operation size (1=32bit, 0=16bit)
			U32 granularity : 1; // granularity (1=32 bit, 0=20 bit)
			U32 baseHi : 8; // base hi 31:24
		} s2;
	} u2;
} CodeDescriptor;

/*
 *  Module record. These are emitted whenever a DLL/EXE is loaded or unloaded.
 *  The filename fields may be 0 on an unload.  The records reperesent a module
 *  for a certain span of time, delineated by the load / unload samplecounts.
 *  Note:
 *  The structure contains 64 bit fields which may cause the compiler to pad the
 *  length of the structure to an 8 byte boundary.
 */
typedef struct ModuleRecord_s {
	U16 recLength; // total length of this record (including this length,
		// always U32 multiple)  output from sampler is variable
		// length (pathname at end of record) sampfile builder moves
		// path names to a separate "literal pool" area
		// so that these records become fixed length, and can be treated
		// as an array see modrecFixedLen in header

	U16 segmentType : 2;
	// V86, 16, 32, 64 (see MODE_ defines), maybe inaccurate for Win95
	// .. a 16 bit module may become a 32 bit module, inferred by
	// ..looking at 1st sample record that matches the module selector
	U16 loadEvent : 1; // 0 for load, 1 for unload
	U16 processed : 1; // 0 for load, 1 for unload
	U16 reserved0 : 12;

	U16 selector; // code selector or V86 segment
	U16 segmentNameLength;
	// length of the segment name if the segmentNameSet bit is set
	U32 segmentNumber;
	// segment number, Win95 can have multiple pieces for one module
	union {
		U32 flags; // all the flags as one dword
		struct {
			U32 exe : 1; // this module is an exe
			U32 globalModule : 1;
			// globally loaded module.  There may be multiple
			// module records for a global module, but the samples
			// will only point to the 1st one, the others will be
			// ignored.  NT's Kernel32 is an example of this.
			// REVISIT this??
			U32 bogusWin95 : 1;
			// "bogus" win95 module.  By bogus, we mean a
			// module that has a pid of 0, no length and no base.
			// Selector actually used as a 32 bit module.
			U32 pidRecIndexRaw : 1; // pidRecIndex is raw OS pid
			U32 sampleFound : 1;
			// at least one sample referenced this module
			U32 tscUsed : 1; // tsc set when record written
			U32 duplicate : 1;
			// 1st pass analysis has determined this is a
			// duplicate load
			U32 globalModuleTB5 : 1;
			// module mapped into all processes on system
			U32 segmentNameSet : 1;
			// set if the segment name was collected
			// (initially done for xbox collections)
			U32 firstModuleRecInProcess : 1;
			// if the pidCreatesTrackedInModuleRecs flag is set
			//  in the SampleHeaderEx struct and this flag
			//  is set, the associated module indicates
			//  the beginning of a new process
			U32 source : 1;
			// 0 for path in target system,
			// 1 for path in host system
			U32 unknownLoadAddress : 1;
			// for 0 valid loadAddr64 value,
			// 1 for invalid loadAddr64 value
			U32 reserved1 : 20;
		} s1;
	} u2;
	U64 length64; // module length
	U64 loadAddr64; // load address
	U32 pidRecIndex;
	// process ID rec index (index into  start of pid record section)
	// .. (see pidRecIndexRaw).  If pidRecIndex == 0 and pidRecIndexRaw == 1
	// ..then this is a kernel or global module.  Can validly
	// ..be 0 if not raw (array index).  Use ReturnPid() to access this
	// ..field
	U32 osid; // OS identifier
	U64 unloadTsc; // TSC collected on an unload event
	U32 path; // module path name (section offset on disk)
		// ..when initally written by sampler name is at end of this
		// ..struct, when merged with main file names are pooled at end
		// ..of ModuleRecord Section so ModulesRecords can be
		// ..fixed length
	U16 pathLength; // path name length (inludes terminating \0)
	U16 filenameOffset; // offset into path name of base filename
	U32 segmentName; // offset to the segmentName from the beginning of the
		//  module section in a processed module section
		//  (s/b 0 in a raw module record)
		// in a raw module record, the segment name will follow the
		//  module name and the module name's terminating NULL char
	U32 page_offset_high;
	U64 tsc; // time stamp counter module event occurred
	U32 parent_pid; // Parent PID of the process
	U32 page_offset_low;
} ModuleRecord;

#define MR_unloadTscSet(x, y) { (x)->unloadTsc = (y); }
#define MR_unloadTscGet(x) ((x)->unloadTsc)

#define MR_page_offset_Set(x, y)                                    \
	{                                                           \
		(x)->page_offset_low = (y)&0xFFFFFFFF;              \
		(x)->page_offset_high = ((y) >> 32) & 0xFFFFFFFF;   \
	}

#define MR_page_offset_Get(x)                                                  \
	((((U64)(x)->page_offset_high) << 32) | (x)->page_offset_low)

// Accessor macros for ModuleRecord
#define MODULE_RECORD_rec_length(x) ((x)->recLength)
#define MODULE_RECORD_segment_type(x) ((x)->segmentType)
#define MODULE_RECORD_load_event(x) ((x)->loadEvent)
#define MODULE_RECORD_processed(x) ((x)->processed)
#define MODULE_RECORD_selector(x) ((x)->selector)
#define MODULE_RECORD_segment_name_length(x) ((x)->segmentNameLength)
#define MODULE_RECORD_segment_number(x) ((x)->segmentNumber)
#define MODULE_RECORD_flags(x) ((x)->u2.flags)
#define MODULE_RECORD_exe(x) ((x)->u2.s1.exe)
#define MODULE_RECORD_global_module(x) ((x)->u2.s1.globalModule)
#define MODULE_RECORD_bogus_win95(x) ((x)->u2.s1.bogusWin95)
#define MODULE_RECORD_pid_rec_index_raw(x) ((x)->u2.s1.pidRecIndexRaw)
#define MODULE_RECORD_sample_found(x) ((x)->u2.s1.sampleFound)
#define MODULE_RECORD_tsc_used(x) ((x)->u2.s1.tscUsed)
#define MODULE_RECORD_duplicate(x) ((x)->u2.s1.duplicate)
#define MODULE_RECORD_global_module_tb5(x) ((x)->u2.s1.globalModuleTB5)
#define MODULE_RECORD_segment_name_set(x) ((x)->u2.s1.segmentNameSet)
#define MODULE_RECORD_first_module_rec_in_process(x)                           \
	((x)->u2.s1.firstModuleRecInProcess)
#define MODULE_RECORD_source(x) ((x)->u2.s1.source)
#define MODULE_RECORD_unknown_load_address(x) ((x)->u2.s1.unknownLoadAddress)
#define MODULE_RECORD_length64(x) ((x)->length64)
#define MODULE_RECORD_load_addr64(x) ((x)->loadAddr64)
#define MODULE_RECORD_pid_rec_index(x) ((x)->pidRecIndex)
#define MODULE_RECORD_load_sample_count(x) ((x)->u5.s2.loadSampleCount)
#define MODULE_RECORD_unload_sample_count(x) ((x)->u5.s2.unloadSampleCount)
#define MODULE_RECORD_unload_tsc(x) ((x)->unloadTsc)
#define MODULE_RECORD_path(x) ((x)->path)
#define MODULE_RECORD_path_length(x) ((x)->pathLength)
#define MODULE_RECORD_filename_offset(x) ((x)->filenameOffset)
#define MODULE_RECORD_segment_name(x) ((x)->segmentName)
#define MODULE_RECORD_tsc(x) ((x)->tsc)
#define MODULE_RECORD_parent_pid(x) ((x)->parent_pid)
#define MODULE_RECORD_osid(x) ((x)->osid)

/*
 *  Sample record.  Size can be determined by looking at the header record.
 *  There can be up to 3 sections.  The SampleFileHeader defines the presence
 *  of sections and their offsets. Within a sample file, all of the sample
 *  records have the same number of sections and the same size.  However,
 *  different sample record sections and sizes can exist in different
 *  sample files.  Since recording counters and the time stamp counter for
 *  each sample can be space consuming, the user can determine whether or not
 *  this information is kept at sample collection time.
 */

typedef struct SampleRecordPC_s { // Program Counter section
	U32 descriptor_id;
	U32 osid; // OS identifier
	union {
		struct {
			U64 iip; // IA64 interrupt instruction pointer
			U64 ipsr; // IA64 interrupt processor status register
		} s1;
		struct {
			U32 eip; // IA32 instruction pointer
			U32 eflags; // IA32 eflags
			CodeDescriptor csd; // IA32 code seg descriptor(8 bytes)
		} s2;
	} u1;
	U16 cs; // IA32 cs (0 for IA64)
	union {
		U16 cpuAndOS; // cpu and OS info as one word
		struct { // cpu and OS info broken out
			U16 cpuNum : 12; // cpu number (0 - 4096)
			U16 notVmid0 : 1;
			// win95, vmid0 flag(1 means NOT vmid 0)
			U16 codeMode : 2; // processor mode, see MODE_ defines
			U16 uncore_valid : 1;
			// identifies if the uncore count is valid
		} s3;
	} u2;
	U32 tid; // OS thread ID  (may get reused, see tidIsRaw)
	U32 pidRecIndex; // process ID rec index (index into start of pid
		// record section) .. can validly be 0 if not raw
		// (array index).  Use ReturnPid() to
		// ..access this field .. (see pidRecIndexRaw)
	union {
		U32 bitFields2;
		struct {
			U32 mrIndex : 20;
			// module record index (index into start of
			// module rec section) .. (see mrIndexNone)
			U32 eventIndex : 8; // index into the Events section
			U32 tidIsRaw : 1; // tid is raw OS tid
			U32 IA64PC : 1; // TRUE=this is a IA64 PC sample record
			U32 pidRecIndexRaw : 1; // pidRecIndex is raw OS pid
			U32 mrIndexNone : 1; // no mrIndex (unknown module)
		} s4;
	} u3;
	U64 tsc; // processor timestamp counter
} SampleRecordPC, *PSampleRecordPC;

#define SAMPLE_RECORD_descriptor_id(x) ((x)->descriptor_id)
#define SAMPLE_RECORD_osid(x) ((x)->osid)
#define SAMPLE_RECORD_iip(x) ((x)->u1.s1.iip)
#define SAMPLE_RECORD_ipsr(x) ((x)->u1.s1.ipsr)
#define SAMPLE_RECORD_eip(x) ((x)->u1.s2.eip)
#define SAMPLE_RECORD_eflags(x) ((x)->u1.s2.eflags)
#define SAMPLE_RECORD_csd(x) ((x)->u1.s2.csd)
#define SAMPLE_RECORD_cs(x) ((x)->cs)
#define SAMPLE_RECORD_cpu_and_os(x) ((x)->u2.cpuAndOS)
#define SAMPLE_RECORD_cpu_num(x) ((x)->u2.s3.cpuNum)
#define SAMPLE_RECORD_uncore_valid(x) ((x)->u2.s3.uncore_valid)
#define SAMPLE_RECORD_not_vmid0(x) ((x)->u2.s3.notVmid0)
#define SAMPLE_RECORD_code_mode(x) ((x)->u2.s3.codeMode)
#define SAMPLE_RECORD_tid(x) ((x)->tid)
#define SAMPLE_RECORD_pid_rec_index(x) ((x)->pidRecIndex)
#define SAMPLE_RECORD_bit_fields2(x) ((x)->u3.bitFields2)
#define SAMPLE_RECORD_mr_index(x) ((x)->u3.s4.mrIndex)
#define SAMPLE_RECORD_event_index(x) ((x)->u3.s4.eventIndex)
#define SAMPLE_RECORD_tid_is_raw(x) ((x)->u3.s4.tidIsRaw)
#define SAMPLE_RECORD_ia64_pc(x) ((x)->u3.s4.IA64PC)
#define SAMPLE_RECORD_pid_rec_index_raw(x) ((x)->u3.s4.pidRecIndexRaw)
#define SAMPLE_RECORD_mr_index_none(x) ((x)->u3.s4.mrIndexNone)
#define SAMPLE_RECORD_tsc(x) ((x)->tsc)

// end of SampleRecord sections

/* Uncore Sample Record definition. This is a skinny sample record used by
 * uncore boxes to record samples.
 * The sample record consists of a descriptor id, cpu info and timestamp.
 */

typedef struct UncoreSampleRecordPC_s {
	U32 descriptor_id;
	U32 osid;
	U16 cpuNum;
	U16 pkgNum;
	union {
		U32 flags;
		struct {
			U32 uncore_valid : 1;
			// identifies if the uncore count is valid
			U32 reserved1 : 31;
		} s1;
	} u1;
	U64 reserved2;
	U64 tsc; // processor timestamp counter
} UncoreSampleRecordPC, *PUnocreSampleRecordPC;

#define UNCORE_SAMPLE_RECORD_descriptor_id(x) ((x)->descriptor_id)
#define UNCORE_SAMPLE_RECORD_osid(x) ((x)->osid)
#define UNCORE_SAMPLE_RECORD_cpu_num(x) ((x)->cpuNum)
#define UNCORE_SAMPLE_RECORD_pkg_num(x) ((x)->pkgNum)
#define UNCORE_SAMPLE_RECORD_uncore_valid(x) ((x)->u1.s1.uncore_valid)
#define UNCORE_SAMPLE_RECORD_tsc(x) ((x)->tsc)

// end of UncoreSampleRecord section

// Definitions for user markers data
// The instances of these structures will be written to user markers temp file
#define MARKER_DEFAULT_TYPE "Default_Marker"
#define MARKER_DEFAULT_ID 0
#define MAX_MARKER_LENGTH 136

#define MARK_ID 4
#define MARK_DATA 2
#define THREAD_INFO 8

/*
 *  Common Register descriptions
 */

/*
 *  Bits used in the debug control register
 */
#define DEBUG_CTL_LBR 0x0000001
#define DEBUG_CTL_BTF 0x0000002
#define DEBUG_CTL_TR 0x0000040
#define DEBUG_CTL_BTS 0x0000080
#define DEBUG_CTL_BTINT 0x0000100
#define DEBUG_CTL_BT_OFF_OS 0x0000200
#define DEBUG_CTL_BTS_OFF_USR 0x0000400
#define DEBUG_CTL_FRZ_LBR_ON_PMI 0x0000800
#define DEBUG_CTL_FRZ_PMON_ON_PMI 0x0001000
#define DEBUG_CTL_ENABLE_UNCORE_PMI_BIT 0x0002000

#define DEBUG_CTL_NODE_lbr_get(reg) ((reg)&DEBUG_CTL_LBR)
#define DEBUG_CTL_NODE_lbr_set(reg) ((reg) |= DEBUG_CTL_LBR)
#define DEBUG_CTL_NODE_lbr_clear(reg) ((reg) &= ~DEBUG_CTL_LBR)

#define DEBUG_CTL_NODE_btf_get(reg) ((reg)&DEBUG_CTL_BTF)
#define DEBUG_CTL_NODE_btf_set(reg) ((reg) |= DEBUG_CTL_BTF)
#define DEBUG_CTL_NODE_btf_clear(reg) ((reg) &= ~DEBUG_CTL_BTF)

#define DEBUG_CTL_NODE_tr_get(reg) ((reg)&DEBUG_CTL_TR)
#define DEBUG_CTL_NODE_tr_set(reg) ((reg) |= DEBUG_CTL_TR)
#define DEBUG_CTL_NODE_tr_clear(reg) ((reg) &= ~DEBUG_CTL_TR)

#define DEBUG_CTL_NODE_bts_get(reg) ((reg)&DEBUG_CTL_BTS)
#define DEBUG_CTL_NODE_bts_set(reg) ((reg) |= DEBUG_CTL_BTS)
#define DEBUG_CTL_NODE_bts_clear(reg) ((reg) &= ~DEBUG_CTL_BTS)

#define DEBUG_CTL_NODE_btint_get(reg) ((reg)&DEBUG_CTL_BTINT)
#define DEBUG_CTL_NODE_btint_set(reg) ((reg) |= DEBUG_CTL_BTINT)
#define DEBUG_CTL_NODE_btint_clear(reg) ((reg) &= ~DEBUG_CTL_BTINT)

#define DEBUG_CTL_NODE_bts_off_os_get(reg) ((reg)&DEBUG_CTL_BTS_OFF_OS)
#define DEBUG_CTL_NODE_bts_off_os_set(reg) ((reg) |= DEBUG_CTL_BTS_OFF_OS)
#define DEBUG_CTL_NODE_bts_off_os_clear(reg) ((reg) &= ~DEBUG_CTL_BTS_OFF_OS)

#define DEBUG_CTL_NODE_bts_off_usr_get(reg) ((reg)&DEBUG_CTL_BTS_OFF_USR)
#define DEBUG_CTL_NODE_bts_off_usr_set(reg) ((reg) |= DEBUG_CTL_BTS_OFF_USR)
#define DEBUG_CTL_NODE_bts_off_usr_clear(reg) ((reg) &= ~DEBUG_CTL_BTS_OFF_USR)

#define DEBUG_CTL_NODE_frz_lbr_on_pmi_get(reg) ((reg)&DEBUG_CTL_FRZ_LBR_ON_PMI)
#define DEBUG_CTL_NODE_frz_lbr_on_pmi_set(reg)                                 \
	((reg) |= DEBUG_CTL_FRZ_LBR_ON_PMI)
#define DEBUG_CTL_NODE_frz_lbr_on_pmi_clear(reg)                               \
	((reg) &= ~DEBUG_CTL_FRZ_LBR_ON_PMI)

#define DEBUG_CTL_NODE_frz_pmon_on_pmi_get(reg)                                \
	((reg)&DEBUG_CTL_FRZ_PMON_ON_PMI)
#define DEBUG_CTL_NODE_frz_pmon_on_pmi_set(reg)                                \
	((reg) |= DEBUG_CTL_FRZ_PMON_ON_PMI)
#define DEBUG_CTL_NODE_frz_pmon_on_pmi_clear(reg)                              \
	((reg) &= ~DEBUG_CTL_FRZ_PMON_ON_PMI)

#define DEBUG_CTL_NODE_enable_uncore_pmi_get(reg)                              \
	((reg)&DEBUG_CTL_ENABLE_UNCORE_PMI)
#define DEBUG_CTL_NODE_enable_uncore_pmi_set(reg)                              \
	((reg) |= DEBUG_CTL_ENABLE_UNCORE_PMI)
#define DEBUG_CTL_NODE_enable_uncore_pmi_clear(reg)                            \
	((reg) &= ~DEBUG_CTL_ENABLE_UNCORE_PMI)

/*
 * @macro SEP_VERSION_NODE_S
 * @brief
 * This structure supports versioning in Sep. The field major indicates major,
 * version minor indicates the minor version and api indicates the api version
 * for the current sep build. This structure is initialized at the time when
 * the driver is loaded.
 */

typedef struct SEP_VERSION_NODE_S SEP_VERSION_NODE;
typedef SEP_VERSION_NODE * SEP_VERSION;

struct SEP_VERSION_NODE_S {
	union {
		U32 sep_version;
		struct {
			S32 major : 8;
			S32 minor : 8;
			S32 api : 8;
			S32 update : 8;
		} s1;
	} u1;
};

#define SEP_VERSION_NODE_sep_version(version) ((version)->u1.sep_version)
#define SEP_VERSION_NODE_major(version) ((version)->u1.s1.major)
#define SEP_VERSION_NODE_minor(version) ((version)->u1.s1.minor)
#define SEP_VERSION_NODE_api(version) ((version)->u1.s1.api)
#define SEP_VERSION_NODE_update(version) ((version)->u1.s1.update)

/*
 *  The VTSA_SYS_INFO_STRUCT information that is shared across kernel mode
 *  and user mode code, very specifically for tb5 file generation
 */

typedef enum {
	GT_UNK = 0,
	GT_PER_CPU,
	GT_PER_CHIPSET,
	GT_CPUID,
	GT_NODE,
	GT_SYSTEM,
	GT_SAMPLE_RECORD_INFO
} GEN_ENTRY_TYPES;

typedef enum {
	GST_UNK = 0,
	GST_X86,
	GST_ITANIUM,
	GST_SA, //strong arm
	GST_XSC,
	GST_EM64T,
	GST_CS860
} GEN_ENTRY_SUBTYPES;

typedef struct __fixed_size_pointer {
	union {
		U64 fs_force_alignment;
		struct {
			U32 fs_unused;
			U32 is_ptr : 1;
		} s1;
	} u1;
	union {
		U64 fs_offset;
		void *fs_ptr;
	} u2;
} VTSA_FIXED_SIZE_PTR;

#define VTSA_FIXED_SIZE_PTR_is_ptr(fsp) ((fsp)->u1.s1.is_ptr)
#define VTSA_FIXED_SIZE_PTR_fs_offset(fsp) ((fsp)->u2.fs_offset)
#define VTSA_FIXED_SIZE_PTR_fs_ptr(fsp) ((fsp)->u2.fs_ptr)

typedef struct __generic_array_header {
	//
	// Information realted to the generic header
	//
	U32 hdr_size; // size of this generic header
		// (for versioning and real data starts
		//  after the header)

	U32 next_field_hdr_padding; // make sure next field is 8-byte aligned

	//
	// VTSA_FIXED_SIZE_PTR should always be on an 8-byte boundary...
	//
	// pointer to the next generic header if there is one
	//
	VTSA_FIXED_SIZE_PTR hdr_next_gen_hdr;

	U32 hdr_reserved[7]; // padding for future use - force to 64 bytes...

	//
	// Information related to the array this header is describing
	//
	U32 array_num_entries;
	U32 array_entry_size;
	U16 array_type; // from the GEN_ENTRY_TYPES enumeration
	U16 array_subtype; // from the GEN_ENTRY_SUBTYPES enumeration
} VTSA_GEN_ARRAY_HDR;

#define VTSA_GEN_ARRAY_HDR_hdr_size(gah) ((gah)->hdr_size)
#define VTSA_GEN_ARRAY_HDR_hdr_next_gen_hdr(gah) ((gah)->hdr_next_gen_hdr)
#define VTSA_GEN_ARRAY_HDR_array_num_entries(gah) ((gah)->array_num_entries)
#define VTSA_GEN_ARRAY_HDR_array_entry_size(gah) ((gah)->array_entry_size)
#define VTSA_GEN_ARRAY_HDR_array_type(gah) ((gah)->array_type)
#define VTSA_GEN_ARRAY_HDR_array_subtype(gah) ((gah)->array_subtype)

typedef struct __cpuid_x86 {
	U32 cpuid_eax_input;
	U32 cpuid_eax;
	U32 cpuid_ebx;
	U32 cpuid_ecx;
	U32 cpuid_edx;
} VTSA_CPUID_X86;

#define VTSA_CPUID_X86_cpuid_eax_input(cid) ((cid)->cpuid_eax_input)
#define VTSA_CPUID_X86_cpuid_eax(cid) ((cid)->cpuid_eax)
#define VTSA_CPUID_X86_cpuid_ebx(cid) ((cid)->cpuid_ebx)
#define VTSA_CPUID_X86_cpuid_ecx(cid) ((cid)->cpuid_ecx)
#define VTSA_CPUID_X86_cpuid_edx(cid) ((cid)->cpuid_edx)

typedef struct __cpuid_ipf {
	U64 cpuid_select;
	U64 cpuid_val;
} VTSA_CPUID_IPF;

#define VTSA_CPUID_IPF_cpuid_select(cid) ((cid)->cpuid_select)
#define VTSA_CPUID_IPF_cpuid_val(cid) ((cid)->cpuid_val)

typedef struct __generic_per_cpu {
	//
	// per cpu information
	//
	U32 cpu_number; // cpu number (as defined by the OS)
	U32 cpu_speed_mhz; // cpu speed (in Mhz)
	U32 cpu_fsb_mhz; // Front Side Bus speed (in Mhz) (if known)
	U32 cpu_cache_L2;
	// ??? USER: cpu L2 (marketing definition) cache size (if known)

	//
	// And pointer to other structures. Keep this on an 8-byte boundary
	//
	// "pointer" to generic array header that should contain
	// cpuid information for this cpu
	//
	VTSA_FIXED_SIZE_PTR cpu_cpuid_array;

	S64 cpu_tsc_offset;
	// TSC offset from CPU 0 computed as (TSC CPU N - TSC CPU 0)
	//
	// intel processor number (from mkting).
	// Currently 3 decimal digits (3xx, 5xx and 7xx)
	//
	U32 cpu_intel_processor_number;

	U32 cpu_cache_L3;
	// ??? USER: cpu L3 (marketing definition) cache size (if known)

	U64 platform_id;

	//
	// package/mapping information
	//
	// The hierarchy for uniquely identifying a logical processor
	// in a system is node number/id (from the node structure),
	// package number, core number, and thread number.
	// Core number is for identifying a core within a package.
	//
	// Actually, on Itanium getting all this information is
	// pretty involved with complicated algorithm using PAL calls.
	// I don't know how important all this stuff is to the user.
	// Maybe we can just have the place holder now and figure out
	// how to fill them later.
	//
	U16 cpu_package_num; // package number for this cpu (if known)
	U16 cpu_core_num; // core number (if known)
	U16 cpu_hw_thread_num; // hw thread number inside the core (if known)

	U16 cpu_threads_per_core; // total number of h/w threads per core
	U16 cpu_module_id; // Processor module number
	U16 cpu_num_modules; // Number of processor modules
	U32 cpu_core_type; // Core type for hetero
	U32 arch_perfmon_ver;
	U32 num_gp_counters;
	U32 num_fixed_counters;
	U32 reserved1;
	U64 reserved2;
	U64 reserved3;

} VTSA_GEN_PER_CPU;

#define VTSA_GEN_PER_CPU_cpu_number(p_cpu) ((p_cpu)->cpu_number)
#define VTSA_GEN_PER_CPU_cpu_speed_mhz(p_cpu) ((p_cpu)->cpu_speed_mhz)
#define VTSA_GEN_PER_CPU_cpu_fsb_mhz(p_cpu) ((p_cpu)->cpu_fsb_mhz)
#define VTSA_GEN_PER_CPU_cpu_cache_L2(p_cpu) ((p_cpu)->cpu_cache_L2)
#define VTSA_GEN_PER_CPU_cpu_cpuid_array(p_cpu) ((p_cpu)->cpu_cpuid_array)
#define VTSA_GEN_PER_CPU_cpu_tsc_offset(p_cpu) ((p_cpu)->cpu_tsc_offset)
#define VTSA_GEN_PER_CPU_cpu_intel_processor_number(p_cpu)                     \
	((p_cpu)->cpu_intel_processor_number)
#define VTSA_GEN_PER_CPU_cpu_cache_L3(p_cpu) ((p_cpu)->cpu_cache_L3)
#define VTSA_GEN_PER_CPU_platform_id(p_cpu) ((p_cpu)->platform_id)
#define VTSA_GEN_PER_CPU_cpu_package_num(p_cpu) ((p_cpu)->cpu_package_num)
#define VTSA_GEN_PER_CPU_cpu_core_num(p_cpu) ((p_cpu)->cpu_core_num)
#define VTSA_GEN_PER_CPU_cpu_hw_thread_num(p_cpu) ((p_cpu)->cpu_hw_thread_num)
#define VTSA_GEN_PER_CPU_cpu_threads_per_core(p_cpu)                           \
	((p_cpu)->cpu_threads_per_core)
#define VTSA_GEN_PER_CPU_cpu_module_num(p_cpu) ((p_cpu)->cpu_module_id)
#define VTSA_GEN_PER_CPU_cpu_num_modules(p_cpu) ((p_cpu)->cpu_num_modules)
#define VTSA_GEN_PER_CPU_cpu_core_type(p_cpu) ((p_cpu)->cpu_core_type)
#define VTSA_GEN_PER_CPU_arch_perfmon_ver(p_cpu) ((p_cpu)->arch_perfmon_ver)
#define VTSA_GEN_PER_CPU_num_gp_counters(p_cpu) ((p_cpu)->num_gp_counters)
#define VTSA_GEN_PER_CPU_num_fixed_counters(p_cpu) ((p_cpu)->num_fixed_counters)

typedef struct __node_info {
	U32 node_type_from_shell;
	U32 node_id; // The node number/id (if known)

	U32 node_num_available; // total number cpus on this node
	U32 node_num_used; // USER: number used based on cpu mask at time of run

	U64 node_physical_memory;
	// amount of physical memory (bytes) on this node

	//
	// pointer to the first generic header that
	// contains the per-cpu information
	//
	// Keep the VTSA_FIXED_SIZE_PTR on an 8-byte boundary...
	//
	VTSA_FIXED_SIZE_PTR node_percpu_array;

	U32 node_reserved[2]; // leave some space

} VTSA_NODE_INFO;

#define VTSA_NODE_INFO_node_type_from_shell(vni) ((vni)->node_type_from_shell)
#define VTSA_NODE_INFO_node_id(vni) ((vni)->node_id)
#define VTSA_NODE_INFO_node_num_available(vni) ((vni)->node_num_available)
#define VTSA_NODE_INFO_node_num_used(vni) ((vni)->node_num_used)
#define VTSA_NODE_INFO_node_physical_memory(vni) ((vni)->node_physical_memory)
#define VTSA_NODE_INFO_node_percpu_array(vni) ((vni)->node_percpu_array)

typedef struct __sys_info {
	//
	// Keep this on an 8-byte boundary
	//
	VTSA_FIXED_SIZE_PTR node_array; // the per-node information

	U64 min_app_address;
	// USER: lower allowed user space address (if known)
	U64 max_app_address;
	// USER: upper allowed user space address (if known)
	U32 page_size; // Current page size
	U32 allocation_granularity;
	// USER: Granularity of allocation requests (if known)
	U32 reserved1; // added for future fields
	U32 reserved2; // alignment purpose
	U64 reserved3[3]; // added for future fields

} VTSA_SYS_INFO;

#define VTSA_SYS_INFO_node_array(sys_info) ((sys_info)->node_array)
#define VTSA_SYS_INFO_min_app_address(sys_info) ((sys_info)->min_app_address)
#define VTSA_SYS_INFO_max_app_address(sys_info) ((sys_info)->max_app_address)
#define VTSA_SYS_INFO_page_size(sys_info) ((sys_info)->page_size)
#define VTSA_SYS_INFO_allocation_granularity(sys_info)                         \
	((sys_info)->allocation_granularity)

typedef struct DRV_TOPOLOGY_INFO_NODE_S DRV_TOPOLOGY_INFO_NODE;
typedef DRV_TOPOLOGY_INFO_NODE * DRV_TOPOLOGY_INFO;

struct DRV_TOPOLOGY_INFO_NODE_S {
	U32 cpu_number; // cpu number (as defined by the OS)
	U16 cpu_package_num; // package number for this cpu (if known)
	U16 cpu_core_num; // core number (if known)
	U16 cpu_hw_thread_num; // T0 or T1 if HT enabled
	U16 reserved1;
	S32 socket_master;
	S32 core_master;
	S32 thr_master;
	U32 cpu_module_num;
	U32 cpu_module_master;
	U32 cpu_num_modules;
	U32 cpu_core_type;
	U32 arch_perfmon_ver;
	U32 num_gp_counters;
	U32 num_fixed_counters;
	U32 reserved2;
	U64 reserved3;
	U64 reserved4;
};

#define DRV_TOPOLOGY_INFO_cpu_number(dti) ((dti)->cpu_number)
#define DRV_TOPOLOGY_INFO_cpu_package_num(dti) ((dti)->cpu_package_num)
#define DRV_TOPOLOGY_INFO_cpu_core_num(dti) ((dti)->cpu_core_num)
#define DRV_TOPOLOGY_INFO_socket_master(dti) ((dti)->socket_master)
#define DRV_TOPOLOGY_INFO_core_master(dti) ((dti)->core_master)
#define DRV_TOPOLOGY_INFO_thr_master(dti) ((dti)->thr_master)
#define DRV_TOPOLOGY_INFO_cpu_hw_thread_num(dti) ((dti)->cpu_hw_thread_num)
#define DRV_TOPOLOGY_INFO_cpu_module_num(dti) ((dti)->cpu_module_num)
#define DRV_TOPOLOGY_INFO_cpu_module_master(dti) ((dti)->cpu_module_master)
#define DRV_TOPOLOGY_INFO_cpu_num_modules(dti) ((dti)->cpu_num_modules)
#define DRV_TOPOLOGY_INFO_cpu_core_type(dti) ((dti)->cpu_core_type)
#define DRV_TOPOLOGY_INFO_arch_perfmon_ver(dti) ((dti)->arch_perfmon_ver)
#define DRV_TOPOLOGY_INFO_num_gp_counters(dti) ((dti)->num_gp_counters)
#define DRV_TOPOLOGY_INFO_num_fixed_counters(dti) ((dti)->num_fixed_counters)

#define VALUE_TO_BE_DISCOVERED 0

// dimm information
typedef struct DRV_DIMM_INFO_NODE_S DRV_DIMM_INFO_NODE;
typedef DRV_DIMM_INFO_NODE * DRV_DIMM_INFO;

struct DRV_DIMM_INFO_NODE_S {
	U32 platform_id;
	U32 channel_num;
	U32 rank_num;
	U32 value;
	U8 mc_num;
	U8 dimm_valid;
	U8 valid_value;
	U8 rank_value;
	U8 density_value;
	U8 width_value;
	U16 socket_num;
	U64 reserved1;
	U64 reserved2;
};

#define DRV_DIMM_INFO_platform_id(di) ((di)->platform_id)
#define DRV_DIMM_INFO_channel_num(di) ((di)->channel_num)
#define DRV_DIMM_INFO_rank_num(di) ((di)->rank_num)
#define DRV_DIMM_INFO_value(di) ((di)->value)
#define DRV_DIMM_INFO_mc_num(di) ((di)->mc_num)
#define DRV_DIMM_INFO_dimm_valid(di) ((di)->dimm_valid)
#define DRV_DIMM_INFO_valid_value(di) ((di)->valid_value)
#define DRV_DIMM_INFO_rank_value(di) ((di)->rank_value)
#define DRV_DIMM_INFO_density_value(di) ((di)->density_value)
#define DRV_DIMM_INFO_width_value(di) ((di)->width_value)
#define DRV_DIMM_INFO_socket_num(di) ((di)->socket_num)

//platform information. need to get from driver
#define MAX_PACKAGES 16
#define MAX_CHANNELS 8
#define MAX_RANKS 3

typedef struct DRV_PLATFORM_INFO_NODE_S DRV_PLATFORM_INFO_NODE;
typedef DRV_PLATFORM_INFO_NODE * DRV_PLATFORM_INFO;

struct DRV_PLATFORM_INFO_NODE_S {
	U64 info; // platform info
	U64 ddr_freq_index; // freq table index
	U8 misc_valid; // misc enabled valid bit
	U8 reserved1; // added for alignment purpose
	U16 reserved2;
	U32 vmm_timer_freq; // timer frequency from VMM on SoFIA (in HZ)
	U64 misc_info; // misc enabled info
	U64 ufs_freq; // ufs frequency (HSX only)
	DRV_DIMM_INFO_NODE dimm_info[MAX_PACKAGES * MAX_CHANNELS * MAX_RANKS];
	U64 energy_multiplier; // Value of energy multiplier
	U64 reserved3;
	U64 reserved4;
	U64 reserved5;
	U64 reserved6;
};

#define DRV_PLATFORM_INFO_info(data) ((data)->info)
#define DRV_PLATFORM_INFO_ddr_freq_index(data) ((data)->ddr_freq_index)
#define DRV_PLATFORM_INFO_misc_valid(data) ((data)->misc_valid)
#define DRV_PLATFORM_INFO_misc_info(data) ((data)->misc_info)
#define DRV_PLATFORM_INFO_ufs_freq(data) ((data)->ufs_freq)
#define DRV_PLATFORM_INFO_dimm_info(data) ((data)->dimm_info)
#define DRV_PLATFORM_INFO_energy_multiplier(data) ((data)->energy_multiplier)
#define DRV_PLATFORM_INFO_vmm_timer_freq(data) ((data)->vmm_timer_freq)

//platform information. need to get from Platform picker
typedef struct PLATFORM_FREQ_INFO_NODE_S PLATFORM_FREQ_INFO_NODE;
typedef PLATFORM_FREQ_INFO_NODE * PLATFORM_FREQ_INFO;

struct PLATFORM_FREQ_INFO_NODE_S {
	float multiplier; // freq multiplier
	double *table; // freq table
	U32 table_size; // freq table size
	U64 reserved1;
	U64 reserved2;
	U64 reserved3;
	U64 reserved4;
};
#define PLATFORM_FREQ_INFO_multiplier(data) ((data)->multiplier)
#define PLATFORM_FREQ_INFO_table(data) ((data)->table)
#define PLATFORM_FREQ_INFO_table_size(data) ((data)->table_size)

typedef struct DEVICE_INFO_NODE_S DEVICE_INFO_NODE;
typedef DEVICE_INFO_NODE * DEVICE_INFO; //NEEDED in PP

struct DEVICE_INFO_NODE_S {
	S8 *dll_name;
	PVOID dll_handle;
	S8 *cpu_name;
	S8 *pmu_name;
	DRV_STCHAR *event_db_file_name;
	//PLATFORM_IDENTITY plat_identity;
	// is undefined right now. Please take this as structure containing U64
	U32 plat_type;
	// device type (e.g., DEVICE_INFO_CORE, etc. ... see enum below)
	U32 plat_sub_type;
	// cti_type (e.g., CTI_Sandybridge, etc., ... see env_info_types.h)
	S32 dispatch_id;
	// this will be set in user mode dlls and will be unique across all
	// IPF, IA32 (including MIDS).
	ECB *ecb;
	EVENT_CONFIG ec;
	DEV_CONFIG pcfg;
	DEV_UNC_CONFIG pcfg_unc;
	U32 num_of_groups;
	U32 size_of_alloc; // size of each event control block
	PVOID drv_event;
	U32 num_events;
	U32 event_id_index;
	// event id index of device
	// (basically how many events processed before this device)
	U32 num_counters;
	U32 group_index;
	U32 num_packages;
	U32 num_units;
	U32 device_type;
	U32 core_type;
	U32 pmu_clone_id; // cti_type of platform to impersonate in device DLLs
	U32 device_scope;
	U32 reserved1;
	U64 reserved2;
	U64 reserved3;
};

#define MAX_EVENT_NAME_LENGTH 256

#define DEVICE_INFO_dll_name(pdev) ((pdev)->dll_name)
#define DEVICE_INFO_dll_handle(pdev) ((pdev)->dll_handle)
#define DEVICE_INFO_cpu_name(pdev) ((pdev)->cpu_name)
#define DEVICE_INFO_pmu_name(pdev) ((pdev)->pmu_name)
#define DEVICE_INFO_event_db_file_name(pdev) ((pdev)->event_db_file_name)
#define DEVICE_INFO_plat_type(pdev) ((pdev)->plat_type)
#define DEVICE_INFO_plat_sub_type(pdev) ((pdev)->plat_sub_type)
#define DEVICE_INFO_pmu_clone_id(pdev) ((pdev)->pmu_clone_id)
#define DEVICE_INFO_dispatch_id(pdev) ((pdev)->dispatch_id)
#define DEVICE_INFO_ecb(pdev) ((pdev)->ecb)
#define DEVICE_INFO_ec(pdev) ((pdev)->ec)
#define DEVICE_INFO_pcfg(pdev) ((pdev)->pcfg)
#define DEVICE_INFO_pcfg_unc(pdev) ((pdev)->pcfg_unc)
#define DEVICE_INFO_num_groups(pdev) ((pdev)->num_of_groups)
#define DEVICE_INFO_size_of_alloc(pdev) ((pdev)->size_of_alloc)
#define DEVICE_INFO_drv_event(pdev) ((pdev)->drv_event)
#define DEVICE_INFO_num_events(pdev) ((pdev)->num_events)
#define DEVICE_INFO_event_id_index(pdev) ((pdev)->event_id_index)
#define DEVICE_INFO_num_counters(pdev) ((pdev)->num_counters)
#define DEVICE_INFO_group_index(pdev) ((pdev)->group_index)
#define DEVICE_INFO_num_packages(pdev) ((pdev)->num_packages)
#define DEVICE_INFO_num_units(pdev) ((pdev)->num_units)
#define DEVICE_INFO_device_type(pdev) ((pdev)->device_type)
#define DEVICE_INFO_core_type(pdev) ((pdev)->core_type)
#define DEVICE_INFO_device_scope(pdev) ((pdev)->device_scope)

typedef struct DEVICE_INFO_DATA_NODE_S DEVICE_INFO_DATA_NODE;
typedef DEVICE_INFO_DATA_NODE * DEVICE_INFO_DATA; //NEEDED in PP

struct DEVICE_INFO_DATA_NODE_S {
	DEVICE_INFO pdev_info;
	U32 num_elements;
	U32 num_allocated;
	U64 reserved1;
	U64 reserved2;
	U64 reserved3;
	U64 reserved4;
};

#define DEVICE_INFO_DATA_pdev_info(d) ((d)->pdev_info)
#define DEVICE_INFO_DATA_num_elements(d) ((d)->num_elements)
#define DEVICE_INFO_DATA_num_allocated(d) ((d)->num_allocated)

typedef enum {
	DEVICE_INFO_CORE = 0,
	DEVICE_INFO_UNCORE = 1,
	DEVICE_INFO_CHIPSET = 2,
	DEVICE_INFO_GFX = 3,
	DEVICE_INFO_PWR = 4,
	DEVICE_INFO_TELEMETRY = 5
} DEVICE_INFO_TYPE;

typedef enum {
	INVALID_TERMINATE_TYPE = 0,
	STOP_TERMINATE,
	CANCEL_TERMINATE
} ABNORMAL_TERMINATE_TYPE;

typedef enum {
	DEVICE_SCOPE_PACKAGE = 0,
	DEVICE_SCOPE_SYSTEM = 1
} DEVICE_SCOPE_TYPE;

typedef struct PCIFUNC_INFO_NODE_S PCIFUNC_INFO_NODE;
typedef PCIFUNC_INFO_NODE * PCIFUNC_INFO;

struct PCIFUNC_INFO_NODE_S {
	U32 valid;
	U32 num_entries;
	// the number of entries found with same <dev_no, func_no>
	// but difference bus_no.
	U64 deviceId;
	U64 reserved1;
	U64 reserved2;
};

#define PCIFUNC_INFO_NODE_funcno(x) ((x)->funcno)
#define PCIFUNC_INFO_NODE_valid(x) ((x)->valid)
#define PCIFUNC_INFO_NODE_deviceId(x) ((x)->deviceId)
#define PCIFUNC_INFO_NODE_num_entries(x) ((x)->num_entries)

typedef struct PCIDEV_INFO_NODE_S PCIDEV_INFO_NODE;
typedef PCIDEV_INFO_NODE * PCIDEV_INFO;

struct PCIDEV_INFO_NODE_S {
	PCIFUNC_INFO_NODE func_info[MAX_PCI_FUNCNO];
	U32 valid;
	U32 dispatch_id;
	U64 reserved1;
	U64 reserved2;
};

#define PCIDEV_INFO_NODE_func_info(x, i) ((x).func_info[i])
#define PCIDEV_INFO_NODE_valid(x) ((x).valid)

typedef struct UNCORE_PCIDEV_NODE_S UNCORE_PCIDEV_NODE;

struct UNCORE_PCIDEV_NODE_S {
	PCIDEV_INFO_NODE pcidev[MAX_PCI_DEVNO];
	U32 dispatch_id;
	U32 scan;
	U32 num_uncore_units;
	U32 num_deviceid_entries;
	U8 dimm_device1;
	U8 dimm_device2;
	U16 reserved1;
	U32 reserved2;
	U64 reserved3;
	U64 reserved4;
	U32 deviceid_list[MAX_PCI_DEVNO];
};

// Structure used to perform uncore device discovery

typedef struct UNCORE_TOPOLOGY_INFO_NODE_S UNCORE_TOPOLOGY_INFO_NODE;
typedef UNCORE_TOPOLOGY_INFO_NODE * UNCORE_TOPOLOGY_INFO;

struct UNCORE_TOPOLOGY_INFO_NODE_S {
	UNCORE_PCIDEV_NODE device[MAX_DEVICES];
};

#define UNCORE_TOPOLOGY_INFO_device(x, dev_index) ((x)->device[dev_index])
#define UNCORE_TOPOLOGY_INFO_device_dispatch_id(x, dev_index)                  \
	((x)->device[dev_index].dispatch_id)
#define UNCORE_TOPOLOGY_INFO_device_scan(x, dev_index)                         \
	((x)->device[dev_index].scan)
#define UNCORE_TOPOLOGY_INFO_pcidev_valid(x, dev_index, devno)                 \
	((x)->device[dev_index].pcidev[devno].valid)
#define UNCORE_TOPOLOGY_INFO_pcidev_dispatch_id(x, dev_index, devno)           \
	((x)->device[dev_index].pcidev[devno].dispatch_id)
#define UNCORE_TOPOLOGY_INFO_pcidev(x, dev_index, devno)                       \
	((x)->device[dev_index].pcidev[devno])
#define UNCORE_TOPOLOGY_INFO_num_uncore_units(x, dev_index)                    \
	((x)->device[dev_index].num_uncore_units)
#define UNCORE_TOPOLOGY_INFO_num_deviceid_entries(x, dev_index)                \
	((x)->device[dev_index].num_deviceid_entries)
#define UNCORE_TOPOLOGY_INFO_dimm_device1(x, dev_index)                        \
	((x)->device[dev_index].dimm_device1)
#define UNCORE_TOPOLOGY_INFO_dimm_device2(x, dev_index)                        \
	((x)->device[dev_index].dimm_device2)
#define UNCORE_TOPOLOGY_INFO_deviceid(x, dev_index, deviceid_idx)              \
	((x)->device[dev_index].deviceid_list[deviceid_idx])
#define UNCORE_TOPOLOGY_INFO_pcidev_set_funcno_valid(x, dev_index, devno,      \
						     funcno)                   \
	((x)->device[dev_index].pcidev[devno].func_info[funcno].valid = 1)
#define UNCORE_TOPOLOGY_INFO_pcidev_is_found_in_platform(x, dev_index, devno,  \
							 funcno)               \
	((x)->device[dev_index].pcidev[devno].func_info[funcno].num_entries)
#define UNCORE_TOPOLOGY_INFO_pcidev_is_devno_funcno_valid(x, dev_index, devno, \
							  funcno)              \
	((x)->device[dev_index].pcidev[devno].func_info[funcno].valid ? TRUE : \
									FALSE)
#define UNCORE_TOPOLOGY_INFO_pcidev_is_device_found(x, dev_index, devno,       \
						    funcno)                    \
	((x)->device[dev_index].pcidev[devno].func_info[funcno].num_entries > 0)

#define UNCORE_TOPOLOGY_INFO_pcidev_num_entries_found(x, dev_index, devno,     \
						      funcno)                  \
	((x)->device[dev_index].pcidev[devno].func_info[funcno].num_entries)

typedef enum {
	CORE_TOPOLOGY_NODE = 0,
	UNCORE_TOPOLOGY_NODE_IMC = 1,
	UNCORE_TOPOLOGY_NODE_UBOX = 2,
	UNCORE_TOPOLOGY_NODE_QPI = 3,
	MAX_TOPOLOGY_DEV = 4,
	// When you adding new topo node to this enum,
	// make sue MAX_TOPOLOGY_DEV is always the last one.
} UNCORE_TOPOLOGY_NODE_INDEX_TYPE;

typedef struct PLATFORM_TOPOLOGY_REG_NODE_S PLATFORM_TOPOLOGY_REG_NODE;
typedef PLATFORM_TOPOLOGY_REG_NODE * PLATFORM_TOPOLOGY_REG;

struct PLATFORM_TOPOLOGY_REG_NODE_S {
	U32 bus;
	U32 device;
	U32 function;
	U32 reg_id;
	U64 reg_mask;
	U64 reg_value[MAX_PACKAGES];
	U8 reg_type;
	U8 device_valid;
	U16 reserved1;
	U32 reserved2;
	U64 reserved3;
	U64 reserved4;
};

#define PLATFORM_TOPOLOGY_REG_bus(x, i) ((x)[(i)].bus)
#define PLATFORM_TOPOLOGY_REG_device(x, i) ((x)[(i)].device)
#define PLATFORM_TOPOLOGY_REG_function(x, i) ((x)[(i)].function)
#define PLATFORM_TOPOLOGY_REG_reg_id(x, i) ((x)[(i)].reg_id)
#define PLATFORM_TOPOLOGY_REG_reg_mask(x, i) ((x)[(i)].reg_mask)
#define PLATFORM_TOPOLOGY_REG_reg_type(x, i) ((x)[(i)].reg_type)
#define PLATFORM_TOPOLOGY_REG_device_valid(x, i) ((x)[(i)].device_valid)
#define PLATFORM_TOPOLOGY_REG_reg_value(x, i, package_no)                      \
	((x)[(i)].reg_value[package_no])

typedef struct PLATFORM_TOPOLOGY_DISCOVERY_NODE_S
	PLATFORM_TOPOLOGY_DISCOVERY_NODE;
typedef PLATFORM_TOPOLOGY_DISCOVERY_NODE * PLATFORM_TOPOLOGY_DISCOVERY;

struct PLATFORM_TOPOLOGY_DISCOVERY_NODE_S {
	U32 device_index;
	U32 device_id;
	U32 num_registers;
	U8 scope;
	U8 prog_valid;
	U16 reserved2;
	U64 reserved3;
	U64 reserved4;
	U64 reserved5;
	PLATFORM_TOPOLOGY_REG_NODE topology_regs[MAX_REGS];
};

//Structure used to discover the uncore device topology_device

typedef struct PLATFORM_TOPOLOGY_PROG_NODE_S PLATFORM_TOPOLOGY_PROG_NODE;
typedef PLATFORM_TOPOLOGY_PROG_NODE * PLATFORM_TOPOLOGY_PROG;

struct PLATFORM_TOPOLOGY_PROG_NODE_S {
	U32 num_devices;
	PLATFORM_TOPOLOGY_DISCOVERY_NODE topology_device[MAX_TOPOLOGY_DEV];
};

#define PLATFORM_TOPOLOGY_PROG_num_devices(x) ((x)->num_devices)
#define PLATFORM_TOPOLOGY_PROG_topology_device(x, dev_index)                   \
	((x)->topology_device[dev_index])
#define PLATFORM_TOPOLOGY_PROG_topology_device_device_index(x, dev_index)      \
	((x)->topology_device[dev_index].device_index)
#define PLATFORM_TOPOLOGY_PROG_topology_device_device_id(x, dev_index)         \
	((x)->topology_device[dev_index].device_id)
#define PLATFORM_TOPOLOGY_PROG_topology_device_scope(x, dev_index)             \
	((x)->topology_device[dev_index].scope)
#define PLATFORM_TOPOLOGY_PROG_topology_device_num_registers(x, dev_index)     \
	((x)->topology_device[dev_index].num_registers)
#define PLATFORM_TOPOLOGY_PROG_topology_device_prog_valid(x, dev_index)        \
	((x)->topology_device[dev_index].prog_valid)
#define PLATFORM_TOPOLOGY_PROG_topology_topology_regs(x, dev_index)            \
	((x)->topology_device[dev_index].topology_regs)

typedef struct FPGA_GB_DISCOVERY_NODE_S FPGA_GB_DISCOVERY_NODE;

struct FPGA_GB_DISCOVERY_NODE_S {
	U16 bar_num;
	U16 feature_id;
	U32 device_id;
	U64 afu_id_l;
	U64 afu_id_h;
	U32 feature_offset;
	U32 feature_len;
	U8 scan;
	U8 valid;
	U16 reserved1;
	U32 reserved2;
};

typedef struct FPGA_GB_DEV_NODE_S FPGA_GB_DEV_NODE;
typedef FPGA_GB_DEV_NODE * FPGA_GB_DEV;

struct FPGA_GB_DEV_NODE_S {
	U32 num_devices;
	FPGA_GB_DISCOVERY_NODE fpga_gb_device[MAX_DEVICES];
};

#define FPGA_GB_DEV_num_devices(x) ((x)->num_devices)
#define FPGA_GB_DEV_device(x, dev_index) ((x)->fpga_gb_device[dev_index])
#define FPGA_GB_DEV_bar_num(x, dev_index)                                      \
	((x)->fpga_gb_device[dev_index].bar_num)
#define FPGA_GB_DEV_feature_id(x, dev_index)                                   \
	((x)->fpga_gb_device[dev_index].feature_id)
#define FPGA_GB_DEV_device_id(x, dev_index)                                    \
	((x)->fpga_gb_device[dev_index].device_id)
#define FPGA_GB_DEV_afu_id_low(x, dev_index)                                   \
	((x)->fpga_gb_device[dev_index].afu_id_l)
#define FPGA_GB_DEV_afu_id_high(x, dev_index)                                  \
	((x)->fpga_gb_device[dev_index].afu_id_h)
#define FPGA_GB_DEV_feature_offset(x, dev_index)                               \
	((x)->fpga_gb_device[dev_index].feature_offset)
#define FPGA_GB_DEV_feature_len(x, dev_index)                                  \
	((x)->fpga_gb_device[dev_index].feature_len)
#define FPGA_GB_DEV_scan(x, dev_index) ((x)->fpga_gb_device[dev_index].scan)
#define FPGA_GB_DEV_valid(x, dev_index) ((x)->fpga_gb_device[dev_index].valid)

typedef enum {
	UNCORE_TOPOLOGY_INFO_NODE_IMC = 0,
	UNCORE_TOPOLOGY_INFO_NODE_QPILL = 1,
	UNCORE_TOPOLOGY_INFO_NODE_HA = 2,
	UNCORE_TOPOLOGY_INFO_NODE_R3 = 3,
	UNCORE_TOPOLOGY_INFO_NODE_R2 = 4,
	UNCORE_TOPOLOGY_INFO_NODE_IRP = 5,
	UNCORE_TOPOLOGY_INFO_NODE_IMC_UCLK = 6,
	UNCORE_TOPOLOGY_INFO_NODE_EDC_ECLK = 7,
	UNCORE_TOPOLOGY_INFO_NODE_EDC_UCLK = 8,
	UNCORE_TOPOLOGY_INFO_NODE_M2M = 9,
	UNCORE_TOPOLOGY_INFO_NODE_HFI_RXE = 10,
	UNCORE_TOPOLOGY_INFO_NODE_HFI_TXE = 11,
	UNCORE_TOPOLOGY_INFO_NODE_FPGA_CACHE = 12,
	UNCORE_TOPOLOGY_INFO_NODE_FPGA_FAB = 13,
	UNCORE_TOPOLOGY_INFO_NODE_FPGA_THERMAL = 14,
	UNCORE_TOPOLOGY_INFO_NODE_FPGA_POWER = 15,
} UNCORE_TOPOLOGY_INFO_NODE_INDEX_TYPE;

typedef struct SIDEBAND_INFO_NODE_S SIDEBAND_INFO_NODE;
typedef SIDEBAND_INFO_NODE * SIDEBAND_INFO;

struct SIDEBAND_INFO_NODE_S {
	U32 tid;
	U32 pid;
	U64 tsc;
};

#define SIDEBAND_INFO_pid(x) ((x)->pid)
#define SIDEBAND_INFO_tid(x) ((x)->tid)
#define SIDEBAND_INFO_tsc(x) ((x)->tsc)

typedef struct SAMPLE_DROP_NODE_S SAMPLE_DROP_NODE;
typedef SAMPLE_DROP_NODE * SAMPLE_DROP;

struct SAMPLE_DROP_NODE_S {
	U32 os_id;
	U32 cpu_id;
	U32 sampled;
	U32 dropped;
};

#define SAMPLE_DROP_os_id(x) ((x)->os_id)
#define SAMPLE_DROP_cpu_id(x) ((x)->cpu_id)
#define SAMPLE_DROP_sampled(x) ((x)->sampled)
#define SAMPLE_DROP_dropped(x) ((x)->dropped)

#define MAX_SAMPLE_DROP_NODES 20

typedef struct SAMPLE_DROP_INFO_NODE_S SAMPLE_DROP_INFO_NODE;
typedef SAMPLE_DROP_INFO_NODE * SAMPLE_DROP_INFO;

struct SAMPLE_DROP_INFO_NODE_S {
	U32 size;
	SAMPLE_DROP_NODE drop_info[MAX_SAMPLE_DROP_NODES];
};

#define SAMPLE_DROP_INFO_size(x) ((x)->size)
#define SAMPLE_DROP_INFO_drop_info(x, index) ((x)->drop_info[index])

#define IS_PEBS_SAMPLE_RECORD(sample_record)                                   \
	((SAMPLE_RECORD_pid_rec_index(sample_record) == (U32)-1) &&            \
	 (SAMPLE_RECORD_tid(sample_record) == (U32)-1))

/*
 *  VMM vendor information
 */
#define KVM_SIGNATURE "KVMKVMKVM\0\0\0"
#define XEN_SIGNATURE "XenVMMXenVMM"
#define VMWARE_SIGNATURE "VMwareVMware"
#define HYPERV_SIGNATURE "Microsoft Hv"

#define DRV_VMM_UNKNOWN 0
#define DRV_VMM_MOBILEVISOR 1
#define DRV_VMM_KVM 2
#define DRV_VMM_XEN 3
#define DRV_VMM_HYPERV 4
#define DRV_VMM_VMWARE 5
#define DRV_VMM_ACRN 6

/*
 * @macro DRV_SETUP_INFO_NODE_S
 * @brief
 * This structure supports driver information such as NMI profiling mode.
 */

typedef struct DRV_SETUP_INFO_NODE_S DRV_SETUP_INFO_NODE;
typedef DRV_SETUP_INFO_NODE * DRV_SETUP_INFO;

struct DRV_SETUP_INFO_NODE_S {
	union {
		U64 modes;
		struct {
			U64 nmi_mode : 1;
			U64 vmm_mode : 1;
			U64 vmm_vendor : 8;
			U64 vmm_guest_vm : 1;
			U64 pebs_accessible : 1;
			U64 cpu_hotplug_mode : 1;
			U64 matrix_inaccessible : 1;
			U64 page_table_isolation : 2;
			U64 pebs_ignored_by_pti : 1;
			U64 reserved1 : 47;
		} s1;
	} u1;
	U64 reserved2;
	U64 reserved3;
	U64 reserved4;
};

#define DRV_SETUP_INFO_nmi_mode(info) ((info)->u1.s1.nmi_mode)
#define DRV_SETUP_INFO_vmm_mode(info) ((info)->u1.s1.vmm_mode)
#define DRV_SETUP_INFO_vmm_vendor(info) ((info)->u1.s1.vmm_vendor)
#define DRV_SETUP_INFO_vmm_guest_vm(info) ((info)->u1.s1.vmm_guest_vm)
#define DRV_SETUP_INFO_pebs_accessible(info) ((info)->u1.s1.pebs_accessible)
#define DRV_SETUP_INFO_cpu_hotplug_mode(info) ((info)->u1.s1.cpu_hotplug_mode)
#define DRV_SETUP_INFO_matrix_inaccessible(info)                               \
	((info)->u1.s1.matrix_inaccessible)
#define DRV_SETUP_INFO_page_table_isolation(info)                              \
	((info)->u1.s1.page_table_isolation)
#define DRV_SETUP_INFO_pebs_ignored_by_pti(info)                               \
	((info)->u1.s1.pebs_ignored_by_pti)

#define DRV_SETUP_INFO_PTI_DISABLED 0
#define DRV_SETUP_INFO_PTI_KPTI 1
#define DRV_SETUP_INFO_PTI_KAISER 2
#define DRV_SETUP_INFO_PTI_VA_SHADOW 3
#define DRV_SETUP_INFO_PTI_UNKNOWN 4

/*
  Type: task_info_t
  Description:
	  Represents the equivalent of a Linux Thread.
  Fields:
	  o  id: A unique identifier. May be `NULL_TASK_ID`.
	  o  name: Human-readable name for this task
	  o  executable_name: Literal path to the binary elf that this task's
			  entry point is executing from.
	  o  address_space_id: The unique ID for the address space this task is
			  running in.
  */
struct task_info_node_s {
	U64 id;
	char name[32];
	U64 address_space_id;
};

/*
  Type: REMOTE_SWITCH
  Description:
	  Collection switch set on target
*/
typedef struct REMOTE_SWITCH_NODE_S REMOTE_SWITCH_NODE;
typedef REMOTE_SWITCH_NODE * REMOTE_SWITCH;

struct REMOTE_SWITCH_NODE_S {
	U32 auto_mode : 1;
	U32 adv_hotspot : 1;
	U32 lbr_callstack : 2;
	U32 full_pebs : 1;
	U32 uncore_supported : 1;
	U32 agent_mode : 2;
	U32 sched_switch_enabled : 1;
	U32 data_transfer_mode : 1;
	U32 reserved1 : 22;
	U32 reserved2;
};

#define REMOTE_SWITCH_auto_mode(x) ((x).auto_mode)
#define REMOTE_SWITCH_adv_hotspot(x) ((x).adv_hotspot)
#define REMOTE_SWITCH_lbr_callstack(x) ((x).lbr_callstack)
#define REMOTE_SWITCH_full_pebs(x) ((x).full_pebs)
#define REMOTE_SWITCH_uncore_supported(x) ((x).uncore_supported)
#define REMOTE_SWITCH_agent_mode(x) ((x).agent_mode)
#define REMOTE_SWITCH_sched_switch_enabled(x) ((x).sched_switch_enabled)
#define REMOTE_SWITCH_data_transfer_mode(x) ((x).data_transfer_mode)

/*
  Type: REMOTE_OS_INFO
  Description:
	  Remote target OS system information
*/
#define OSINFOLEN 64
typedef struct REMOTE_OS_INFO_NODE_S REMOTE_OS_INFO_NODE;
typedef REMOTE_OS_INFO_NODE * REMOTE_OS_INFO;

struct REMOTE_OS_INFO_NODE_S {
	U32 os_family;
	U32 reserved1;
	S8 sysname[OSINFOLEN];
	S8 release[OSINFOLEN];
	S8 version[OSINFOLEN];
};

#define REMOTE_OS_INFO_os_family(x) ((x).os_family)
#define REMOTE_OS_INFO_sysname(x) ((x).sysname)
#define REMOTE_OS_INFO_release(x) ((x).release)
#define REMOTE_OS_INFO_version(x) ((x).version)

/*
  Type: REMOTE_HARDWARE_INFO
  Description:
	  Remote target hardware information
*/
typedef struct REMOTE_HARDWARE_INFO_NODE_S REMOTE_HARDWARE_INFO_NODE;
typedef REMOTE_HARDWARE_INFO_NODE * REMOTE_HARDWARE_INFO;

struct REMOTE_HARDWARE_INFO_NODE_S {
	U32 num_cpus;
	U32 family;
	U32 model;
	U32 stepping;
	U64 tsc_freq;
	U64 reserved2;
	U64 reserved3;
};

#define REMOTE_HARDWARE_INFO_num_cpus(x) ((x).num_cpus)
#define REMOTE_HARDWARE_INFO_family(x) ((x).family)
#define REMOTE_HARDWARE_INFO_model(x) ((x).model)
#define REMOTE_HARDWARE_INFO_stepping(x) ((x).stepping)
#define REMOTE_HARDWARE_INFO_tsc_frequency(x) ((x).tsc_freq)

/*
  Type: SEP_AGENT_MODE
  Description:
	  SEP mode on target agent
*/
typedef enum {
	NATIVE_AGENT = 0,
	HOST_VM_AGENT, // Service OS in ACRN
	GUEST_VM_AGENT // User OS in ACRN
} SEP_AGENT_MODE;

/*
  Type: DATA_TRANSFER_MODE
  Description:
	 Data transfer mode from target agent to remote host
*/
typedef enum {
	IMMEDIATE_TRANSFER = 0,
	DELAYED_TRANSFER // Send after collection is done
} DATA_TRANSFER_MODE;

#define MAX_NUM_OS_ALLOWED 6
#define TARGET_IP_NAMELEN 64

typedef struct TARGET_INFO_NODE_S TARGET_INFO_NODE;
typedef TARGET_INFO_NODE * TARGET_INFO;

struct TARGET_INFO_NODE_S {
	U32 num_of_agents;
	U32 reserved;
	U32 os_id[MAX_NUM_OS_ALLOWED];
	S8 ip_address[MAX_NUM_OS_ALLOWED][TARGET_IP_NAMELEN];
	REMOTE_OS_INFO_NODE os_info[MAX_NUM_OS_ALLOWED];
	REMOTE_HARDWARE_INFO_NODE hardware_info[MAX_NUM_OS_ALLOWED];
	REMOTE_SWITCH_NODE remote_switch[MAX_NUM_OS_ALLOWED];
};

#define TARGET_INFO_num_of_agents(x) 	((x)->num_of_agents)
#define TARGET_INFO_os_id(x, i) 		((x)->os_id[i])
#define TARGET_INFO_os_info(x, i) 		((x)->os_info[i])
#define TARGET_INFO_ip_address(x, i) 	((x)->ip_address[i])
#define TARGET_INFO_hardware_info(x, i) ((x)->hardware_info[i])
#define TARGET_INFO_remote_switch(x, i) ((x)->remote_switch[i])

typedef struct CPU_MAP_TRACE_NODE_S CPU_MAP_TRACE_NODE;
typedef CPU_MAP_TRACE_NODE * CPU_MAP_TRACE;

struct CPU_MAP_TRACE_NODE_S {
	U64 tsc;
	U32 os_id;
	U32 vcpu_id;
	U32 pcpu_id;
	U8 is_static : 1;
	U8 initial : 1;
	U8 reserved1 : 6;
	U8 reserved2;
	U16 reserved3;
	U64 tsc_offset;
};

#define CPU_MAP_TRACE_tsc(x) ((x)->tsc)
#define CPU_MAP_TRACE_os_id(x) ((x)->os_id)
#define CPU_MAP_TRACE_vcpu_id(x) ((x)->vcpu_id)
#define CPU_MAP_TRACE_pcpu_id(x) ((x)->pcpu_id)
#define CPU_MAP_TRACE_is_static(x) ((x)->is_static)
#define CPU_MAP_TRACE_initial(x) ((x)->initial)

#define MAX_NUM_VCPU 64
#define MAX_NUM_VM 16

typedef struct CPU_MAP_TRACE_LIST_NODE_S   CPU_MAP_TRACE_LIST_NODE;
typedef        CPU_MAP_TRACE_LIST_NODE   * CPU_MAP_TRACE_LIST;

struct CPU_MAP_TRACE_LIST_NODE_S {
	U32 osid;
	U8  num_entries;
	U8  reserved1;
	U16 reserved2;
	CPU_MAP_TRACE_NODE  entries[MAX_NUM_VCPU];
};

typedef struct VM_OSID_MAP_NODE_S   VM_OSID_MAP_NODE;
typedef        VM_OSID_MAP_NODE   * VM_OSID_MAP;
struct VM_OSID_MAP_NODE_S {
	U32 num_vms;
	U32 reserved1;
	U32 osid[MAX_NUM_VM];
};

typedef struct VM_SWITCH_TRACE_NODE_S VM_SWITCH_TRACE_NODE;
typedef VM_SWITCH_TRACE_NODE * VM_SWITCH_TRACE;

struct VM_SWITCH_TRACE_NODE_S {
	U64 tsc;
	U32 from_os_id;
	U32 to_os_id;
	U64 reason;
	U64 reserved1;
	U64 reserved2;
};

#define VM_SWITCH_TRACE_tsc(x)          ((x)->tsc)
#define VM_SWITCH_TRACE_from_os_id(x)   ((x)->from_os_id)
#define VM_SWITCH_TRACE_to_os_id(x)     ((x)->to_os_id)
#define VM_SWITCH_TRACE_reason(x)       ((x)->reason)

typedef struct EMON_BUFFER_DRIVER_HELPER_NODE_S EMON_BUFFER_DRIVER_HELPER_NODE;
typedef EMON_BUFFER_DRIVER_HELPER_NODE * EMON_BUFFER_DRIVER_HELPER;

struct EMON_BUFFER_DRIVER_HELPER_NODE_S {
	U32 num_entries_per_package;
	U32 num_cpu;
	U32 power_num_package_events;
	U32 power_num_module_events;
	U32 power_num_thread_events;
	U32 power_device_offset_in_package;
	U32 core_num_events;
	U32 core_index_to_thread_offset_map[];
};

#define EMON_BUFFER_DRIVER_HELPER_num_entries_per_package(x)                   \
	((x)->num_entries_per_package)
#define EMON_BUFFER_DRIVER_HELPER_num_cpu(x) ((x)->num_cpu)
#define EMON_BUFFER_DRIVER_HELPER_power_num_package_events(x)                  \
	((x)->power_num_package_events)
#define EMON_BUFFER_DRIVER_HELPER_power_num_module_events(x)                   \
	((x)->power_num_module_events)
#define EMON_BUFFER_DRIVER_HELPER_power_num_thread_events(x)                   \
	((x)->power_num_thread_events)
#define EMON_BUFFER_DRIVER_HELPER_power_device_offset_in_package(x)            \
	((x)->power_device_offset_in_package)
#define EMON_BUFFER_DRIVER_HELPER_core_num_events(x) ((x)->core_num_events)
#define EMON_BUFFER_DRIVER_HELPER_core_index_to_thread_offset_map(x)           \
	((x)->core_index_to_thread_offset_map)

// EMON counts buffer follow this hardware topology:
// package -> device -> unit/thread -> event

// Calculate the CORE thread offset
// Using for initialization: calculate the cpu_index_to_thread_offset_map
// in emon_Create_Emon_Buffer_Descriptor()
// EMON_BUFFER_CORE_THREAD_OFFSET =
// package_id * num_entries_per_package  +  //package offset
// device_offset_in_package              +  //device base offset
// (core_id * threads_per_core + thread_id) * num_core_events + //thread offset
#define EMON_BUFFER_CORE_THREAD_OFFSET(package_id, num_entries_per_package,    \
				       device_offset_in_package, core_id,      \
				       threads_per_core, thread_id,            \
				       num_core_events)                        \
	(package_id * num_entries_per_package + device_offset_in_package +       \
		(core_id * threads_per_core + thread_id) * num_core_events)

// Take cpu_index and cpu_index_to_thread_offset_map to get thread_offset,
// and calculate the CORE event offset
// Using for kernel and emon_output.c printing function
// EMON_BUFFER_CORE_EVENT_OFFSET =
//      cpu_index_to_thread_offset +  //thread offset
//      core_event_id                 //event_offset
#define EMON_BUFFER_CORE_EVENT_OFFSET(cpu_index_to_thread_offset,              \
				      core_event_id)                           \
	(cpu_index_to_thread_offset + core_event_id)

// Calculate the device level to UNCORE event offset
// Using for kernel and emon_output.c printing function
// EMON_BUFFER_UNCORE_PACKAGE_EVENT_OFFSET_IN_PACKAGE =
//      device_offset_in_package         +  //device_offset_in_package
//      device_unit_id * num_unit_events +  //unit_offset
//      device_event_id                     //event_offset
#define EMON_BUFFER_UNCORE_PACKAGE_EVENT_OFFSET_IN_PACKAGE(                    \
	device_offset_in_package, device_unit_id, num_unit_events,             \
	device_event_id)                                                       \
	(device_offset_in_package + device_unit_id * num_unit_events +           \
		device_event_id)

// Take 'device level to UNCORE event offset' and package_id,
// calculate the UNCORE package level event offset
// Using for emon_output.c printing function
// EMON_BUFFER_UNCORE_PACKAGE_EVENT_OFFSET =
//      package_id * num_entries_per_package +  //package_offset
//      uncore_offset_in_package;               //offset_in_package
#define EMON_BUFFER_UNCORE_PACKAGE_EVENT_OFFSET(                               \
	package_id, num_entries_per_package, uncore_offset_in_package)         \
	(package_id * num_entries_per_package + uncore_offset_in_package)

// Take 'device level to UNCORE event offset',
// calculate the UNCORE system level event offset
// Using for emon_output.c printing function
// EMON_BUFFER_UNCORE_SYSTEM_EVENT_OFFSET =
//      device_offset_in_system            +  //device_offset_in_system
//      device_unit_id * num_system_events +  //device_unit_offset
//      device_event_id                       //event_offset
#define EMON_BUFFER_UNCORE_SYSTEM_EVENT_OFFSET(device_offset_in_system,        \
					       device_unit_id,                 \
					       num_system_events,              \
					       device_event_id)                \
	(device_offset_in_system + device_unit_id * num_system_events +        \
		device_event_id)

// Calculate the package level power event offset
// Using for kernel and emon_output.c printing function
// EMON_BUFFER_UNCORE_PACKAGE_POWER_EVENT_OFFSET =
//      package_id * num_entries_per_package + //package offset
//      device_offset_in_package             + //device offset
//      package_event_offset                   //power package event offset
#define EMON_BUFFER_UNCORE_PACKAGE_POWER_EVENT_OFFSET(                         \
	package_id, num_entries_per_package, device_offset_in_package,         \
	device_event_offset)                                                   \
	(package_id * num_entries_per_package + device_offset_in_package +     \
		device_event_offset)

// Calculate the module level power event offset
// Using for kernel and emon_output.c printing function
// EMON_BUFFER_UNCORE_MODULE_POWER_EVENT_OFFSET =
//      package_id * num_entries_per_package + //package offset
//      device_offset_in_package             + //device offset
//      num_package_events                   + //package event offset
//      module_id * num_module_events        + //module offset
//      module_event_offset                    //power module event offset
#define EMON_BUFFER_UNCORE_MODULE_POWER_EVENT_OFFSET(                          \
	package_id, num_entries_per_package, device_offset_in_package,         \
	num_package_events, module_id, num_module_events, device_event_offset) \
	(package_id * num_entries_per_package + device_offset_in_package +     \
		num_package_events + module_id * num_module_events +           \
		device_event_offset)

// Calculate the package level power event offset
// Using for kernel and emon_output.c printing function
// EMON_BUFFER_UNCORE_THREAD_POWER_EVENT_OFFSET =
//  package_id * num_entries_per_package                  + //package offset
//  device_offset_in_package                              + //device offset
//  num_package_events                                    + //package offset
//  num_modules_per_package * num_module_events           + //module offset
//  (core_id*threads_per_core+thread_id)*num_thread_events  + //thread offset
//  thread_event_offset                           //power thread event offset
#define EMON_BUFFER_UNCORE_THREAD_POWER_EVENT_OFFSET(                          \
	package_id, num_entries_per_package, device_offset_in_package,         \
	num_package_events, num_modules_per_package, num_module_events,        \
	core_id, threads_per_core, thread_id, num_unit_events,                 \
	device_event_offset)                                                   \
	(package_id * num_entries_per_package + device_offset_in_package +     \
		num_package_events +                                           \
		num_modules_per_package * num_module_events +                  \
		(core_id * threads_per_core + thread_id) * num_unit_events +   \
		device_event_offset)

/*
 ************************************
 *  DRIVER LOG BUFFER DECLARATIONS  *
 ************************************
 */

#define DRV_MAX_NB_LOG_CATEGORIES 256 // Must be a multiple of 8
#define DRV_NB_LOG_CATEGORIES 14
#define DRV_LOG_CATEGORY_LOAD 0
#define DRV_LOG_CATEGORY_INIT 1
#define DRV_LOG_CATEGORY_DETECTION 2
#define DRV_LOG_CATEGORY_ERROR 3
#define DRV_LOG_CATEGORY_STATE_CHANGE 4
#define DRV_LOG_CATEGORY_MARK 5
#define DRV_LOG_CATEGORY_DEBUG 6
#define DRV_LOG_CATEGORY_FLOW 7
#define DRV_LOG_CATEGORY_ALLOC 8
#define DRV_LOG_CATEGORY_INTERRUPT 9
#define DRV_LOG_CATEGORY_TRACE 10
#define DRV_LOG_CATEGORY_REGISTER 11
#define DRV_LOG_CATEGORY_NOTIFICATION 12
#define DRV_LOG_CATEGORY_WARNING 13

#define LOG_VERBOSITY_UNSET 0xFF
#define LOG_VERBOSITY_DEFAULT 0xFE
#define LOG_VERBOSITY_NONE 0

#define LOG_CHANNEL_MEMLOG 0x1
#define LOG_CHANNEL_AUXMEMLOG 0x2
#define LOG_CHANNEL_PRINTK 0x4
#define LOG_CHANNEL_TRACEK 0x8
#define LOG_CHANNEL_MOSTWHERE                                                  \
	(LOG_CHANNEL_MEMLOG | LOG_CHANNEL_AUXMEMLOG | LOG_CHANNEL_PRINTK)
#define LOG_CHANNEL_EVERYWHERE                                                 \
	(LOG_CHANNEL_MEMLOG | LOG_CHANNEL_AUXMEMLOG | LOG_CHANNEL_PRINTK |     \
	 LOG_CHANNEL_TRACEK)
#define LOG_CHANNEL_MASK LOG_CATEGORY_VERBOSITY_EVERYWHERE

#define LOG_CONTEXT_REGULAR 0x10
#define LOG_CONTEXT_INTERRUPT 0x20
#define LOG_CONTEXT_NOTIFICATION 0x40
#define LOG_CONTEXT_ALL                                                        \
	(LOG_CONTEXT_REGULAR | LOG_CONTEXT_INTERRUPT | LOG_CONTEXT_NOTIFICATION)
#define LOG_CONTEXT_MASK LOG_CONTEXT_ALL
#define LOG_CONTEXT_SHIFT 4

#define DRV_LOG_NOTHING 0
#define DRV_LOG_FLOW_IN 1
#define DRV_LOG_FLOW_OUT 2

/*
 * @macro DRV_LOG_ENTRY_NODE_S
 * @brief
 * This structure is used to store a log message from the driver.
 */

#define DRV_LOG_MESSAGE_LENGTH 64
#define DRV_LOG_FUNCTION_NAME_LENGTH 32

typedef struct DRV_LOG_ENTRY_NODE_S DRV_LOG_ENTRY_NODE;
typedef DRV_LOG_ENTRY_NODE * DRV_LOG_ENTRY;
struct DRV_LOG_ENTRY_NODE_S {
	char function_name[DRV_LOG_FUNCTION_NAME_LENGTH];
	char message[DRV_LOG_MESSAGE_LENGTH];

	U16 temporal_tag;
	U16 integrity_tag;

	U8 category;
	U8 secondary_info; // Secondary attribute:
		// former driver state for STATE category
		// 'ENTER' or 'LEAVE' for FLOW and TRACE categories
	U16 processor_id;
	// NB: not guaranteed to be accurate (due to preemption/core migration)

	U64 tsc;

	U16 nb_active_interrupts; // never 100% accurate, merely indicative
	U8 active_drv_operation; // only 100% accurate IOCTL-called functions
	U8 driver_state;

	U16 line_number; // as per the __LINE__ macro

	U16 nb_active_notifications;

	U64 reserved; // need padding to reach 128 bytes
}; // this structure should be exactly 128-byte long

#define DRV_LOG_ENTRY_temporal_tag(ent) ((ent)->temporal_tag)
#define DRV_LOG_ENTRY_integrity_tag(ent) ((ent)->integrity_tag)
#define DRV_LOG_ENTRY_category(ent) ((ent)->category)
#define DRV_LOG_ENTRY_secondary_info(ent) ((ent)->secondary_info)
#define DRV_LOG_ENTRY_processor_id(ent) ((ent)->processor_id)
#define DRV_LOG_ENTRY_tsc(ent) ((ent)->tsc)
#define DRV_LOG_ENTRY_driver_state(ent) ((ent)->driver_state)
#define DRV_LOG_ENTRY_active_drv_operation(ent) ((ent)->active_drv_operation)
#define DRV_LOG_ENTRY_nb_active_interrupts(ent) ((ent)->nb_active_interrupts)
#define DRV_LOG_ENTRY_nb_active_notifications(ent)                             \
	((ent)->nb_active_notifications)
#define DRV_LOG_ENTRY_line_number(ent) ((ent)->line_number)
#define DRV_LOG_ENTRY_message(ent) ((ent)->message)
#define DRV_LOG_ENTRY_function_name(ent) ((ent)->function_name)

/*
 * @macro DRV_LOG_BUFFER_NODE_S
 * @brief Circular buffer structure storing the latest
 *        DRV_LOG_MAX_NB_ENTRIES driver messages
 */

#define DRV_LOG_SIGNATURE_SIZE 8 // Must be a multiple of 8
#define DRV_LOG_SIGNATURE_0 'S'
#define DRV_LOG_SIGNATURE_1 'e'
#define DRV_LOG_SIGNATURE_2 'P'
#define DRV_LOG_SIGNATURE_3 'd'
#define DRV_LOG_SIGNATURE_4 'R'
#define DRV_LOG_SIGNATURE_5 'v'
#define DRV_LOG_SIGNATURE_6 '5'
#define DRV_LOG_SIGNATURE_7 '\0'
// The signature is "SePdRv4"; not declared as string on purpose to avoid
// false positives when trying to identify the log buffer in a crash dump

#define DRV_LOG_VERSION 1
#define DRV_LOG_FILLER_BYTE 1

#define DRV_LOG_DRIVER_VERSION_SIZE 64 // Must be a multiple of 8
#define DRV_LOG_MAX_NB_PRI_ENTRIES 	(8192 * 2)
// 2MB buffer [*HAS TO BE* a power of 2!] [8192 entries = 1 MB]
#define DRV_LOG_MAX_NB_AUX_ENTRIES (8192)
// 1MB buffer [*HAS TO BE* a power of 2!]
#define DRV_LOG_MAX_NB_ENTRIES                                                 \
	(DRV_LOG_MAX_NB_PRI_ENTRIES + DRV_LOG_MAX_NB_AUX_ENTRIES)

typedef struct DRV_LOG_BUFFER_NODE_S DRV_LOG_BUFFER_NODE;
typedef DRV_LOG_BUFFER_NODE * DRV_LOG_BUFFER;
struct DRV_LOG_BUFFER_NODE_S {
	char header_signature[DRV_LOG_SIGNATURE_SIZE];
	// some signature to be able to locate the log even without -g; ASCII
	// would help should we change the signature for each log's version
	// instead of keeping it in a dedicated field?

	U32 log_size; // filled with sizeof(this structure) at init.
	U32 max_nb_pri_entries;
	// filled with the driver's "DRV_LOG_MAX_NB_PRIM_ENTRIES" at init.

	U32 max_nb_aux_entries;
	// filled with the driver's "DRV_LOG_MAX_NB_AUX_ENTRIES" at init.
	U32 reserved1;

	U64 init_time; // primary log disambiguator

	U32 disambiguator;
	// used to differentiate the driver's version of the log when a
	// full memory dump can contain some from userland
	U32 log_version; // 0 at first, increase when format changes?

	U32 pri_entry_index;
	// should be incremented *atomically* as a means to (re)allocate
	// the next primary log entry.
	U32 aux_entry_index;
	// should be incremented *atomically* as a means to (re)allocate
	// the next auxiliary log entry.

	char driver_version[DRV_LOG_DRIVER_VERSION_SIZE];

	U8 driver_state;
	U8 active_drv_operation;
	U16 reserved2;
	U32 nb_drv_operations;

	U32 nb_interrupts;
	U16 nb_active_interrupts;
	U16 nb_active_notifications;

	U32 nb_notifications;
	U32 nb_driver_state_transitions;

	U8 contiguous_physical_memory;
	U8 reserved3;
	U16 reserved4;
	U32 reserved5;

	U8 verbosities[DRV_MAX_NB_LOG_CATEGORIES];

	DRV_LOG_ENTRY_NODE entries[DRV_LOG_MAX_NB_ENTRIES];

	char footer_signature[DRV_LOG_SIGNATURE_SIZE];
};

#define DRV_LOG_BUFFER_pri_entry_index(log) ((log)->pri_entry_index)
#define DRV_LOG_BUFFER_aux_entry_index(log) ((log)->aux_entry_index)
#define DRV_LOG_BUFFER_header_signature(log) ((log)->header_signature)
#define DRV_LOG_BUFFER_footer_signature(log) ((log)->footer_signature)
#define DRV_LOG_BUFFER_log_size(log) ((log)->log_size)
#define DRV_LOG_BUFFER_driver_version(log) ((log)->driver_version)
#define DRV_LOG_BUFFER_driver_state(log) ((log)->driver_state)
#define DRV_LOG_BUFFER_active_drv_operation(log) ((log)->active_drv_operation)
#define DRV_LOG_BUFFER_nb_interrupts(log) ((log)->nb_interrupts)
#define DRV_LOG_BUFFER_nb_active_interrupts(log) ((log)->nb_active_interrupts)
#define DRV_LOG_BUFFER_nb_notifications(log) ((log)->nb_notifications)
#define DRV_LOG_BUFFER_nb_active_notifications(log)                            \
	((log)->nb_active_notifications)
#define DRV_LOG_BUFFER_nb_driver_state_transitions(log)                        \
	((log)->nb_driver_state_transitions)
#define DRV_LOG_BUFFER_nb_drv_operations(log) ((log)->nb_drv_operations)
#define DRV_LOG_BUFFER_max_nb_pri_entries(log) ((log)->max_nb_pri_entries)
#define DRV_LOG_BUFFER_max_nb_aux_entries(log) ((log)->max_nb_aux_entries)
#define DRV_LOG_BUFFER_init_time(log) ((log)->init_time)
#define DRV_LOG_BUFFER_disambiguator(log) ((log)->disambiguator)
#define DRV_LOG_BUFFER_log_version(log) ((log)->log_version)
#define DRV_LOG_BUFFER_entries(log) ((log)->entries)
#define DRV_LOG_BUFFER_contiguous_physical_memory(log)                         \
	((log)->contiguous_physical_memory)
#define DRV_LOG_BUFFER_verbosities(log) ((log)->verbosities)

#define DRV_LOG_CONTROL_MAX_DATA_SIZE                                          \
	DRV_MAX_NB_LOG_CATEGORIES // Must be a multiple of 8

typedef struct DRV_LOG_CONTROL_NODE_S DRV_LOG_CONTROL_NODE;
typedef DRV_LOG_CONTROL_NODE * DRV_LOG_CONTROL;

struct DRV_LOG_CONTROL_NODE_S {
	U32 command;
	U32 reserved1;
	U8 data[DRV_LOG_CONTROL_MAX_DATA_SIZE];
	// only DRV_NB_LOG_CATEGORIES elements will be used, but let's plan for
	// backwards compatibility if LOG_CATEGORY_UNSET, READ instead of WRITE

	U64 reserved2;
	// may later want to add support for resizing the buffer,
	// or only log 100 first interrupts, etc.
	U64 reserved3;
	U64 reserved4;
	U64 reserved5;
};

#define DRV_LOG_CONTROL_command(x) ((x)->command)
#define DRV_LOG_CONTROL_verbosities(x) ((x)->data)
#define DRV_LOG_CONTROL_message(x)                                             \
	((x)->data) // Userland 'MARK' messages use the 'data' field too.
#define DRV_LOG_CONTROL_log_size(x) (*((U32 *)((x)->data)))

#define DRV_LOG_CONTROL_COMMAND_NONE 0
#define DRV_LOG_CONTROL_COMMAND_ADJUST_VERBOSITY 1
#define DRV_LOG_CONTROL_COMMAND_MARK 2
#define DRV_LOG_CONTROL_COMMAND_QUERY_SIZE 3
#define DRV_LOG_CONTROL_COMMAND_BENCHMARK 4

#if defined(__cplusplus)
}
#endif

#endif
