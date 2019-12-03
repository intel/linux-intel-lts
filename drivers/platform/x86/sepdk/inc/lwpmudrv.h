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

#ifndef _LWPMUDRV_H_
#define _LWPMUDRV_H_

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/compat.h>
#if KERNEL_VERSION(4, 12, 0) > LINUX_VERSION_CODE
#include <asm/uaccess.h>
#else
#include <linux/uaccess.h>
#endif
#include <asm/cpufeature.h>

#include "lwpmudrv_defines.h"
#include "lwpmudrv_types.h"
#include "lwpmudrv_ecb.h"
#include "lwpmudrv_version.h"
#include "lwpmudrv_struct.h"
#include "pebs.h"
#if defined(BUILD_CHIPSET)
#include "lwpmudrv_chipset.h"
#endif

#if defined(DRV_SEP_ACRN_ON)
#include <linux/vhm/acrn_hv_defs.h>
#include <linux/vhm/vhm_hypercall.h>
#endif

#if defined(X86_FEATURE_KAISER) || defined(CONFIG_KAISER) ||                   \
	defined(KAISER_HEADER_PRESENT)
#define DRV_USE_KAISER
#elif defined(X86_FEATURE_PTI)
#define DRV_USE_PTI
#endif

/*
 * Print macros for driver messages
 */

#if defined(MYDEBUG)
#define SEP_PRINT_DEBUG(fmt, args...)                                \
	{                                                                \
		printk(KERN_INFO SEP_MSG_PREFIX " [DEBUG] " fmt, ##args);    \
	}
#else
#define SEP_PRINT_DEBUG(fmt, args...)                                \
	{                                                                \
		;                                                            \
	}
#endif

#define SEP_PRINT(fmt, args...)                                      \
	{                                                                \
		printk(KERN_INFO SEP_MSG_PREFIX " " fmt, ##args);            \
	}

#define SEP_PRINT_WARNING(fmt, args...)                              \
	{                                                                \
		printk(KERN_ALERT SEP_MSG_PREFIX " [Warning] " fmt, ##args); \
	}

#define SEP_PRINT_ERROR(fmt, args...)                                \
	{                                                                \
		printk(KERN_CRIT SEP_MSG_PREFIX " [ERROR] " fmt, ##args);    \
	}

// Macro to return the thread group id
#define GET_CURRENT_TGID() (current->tgid)

#define OVERFLOW_ARGS U64 *, U64 *

typedef struct DRV_EVENT_MASK_NODE_S DRV_EVENT_MASK_NODE;
typedef DRV_EVENT_MASK_NODE * DRV_EVENT_MASK;

struct DRV_EVENT_MASK_NODE_S {
	U16 event_idx; // 0 <= index < MAX_EVENTS
	U16 reserved1;
	union {
		U32 bitFields1;
		struct {
			U32 precise : 1;
			U32 lbr_capture : 1;
			U32 dear_capture : 1;
			// Indicates which events need to have additional
			// registers read because they are DEAR events.
			U32 iear_capture : 1;
			// Indicates which events need to have additional
			// registers read because they are IEAR events.
			U32 btb_capture : 1;
			// Indicates which events need to have additional
			// registers read because they are BTB events.
			U32 ipear_capture : 1;
			// Indicates which events need to have additional
			// registers read because they are IPEAR events.
			U32 uncore_capture : 1;
			U32 branch : 1;
			// Whether event is related to branch operation or not
			U32 perf_metrics_capture : 1;
			// Whether the event is related to perf_metrics or not
			U32 reserved : 23;
		} s1;
	} u1;
};

#define DRV_EVENT_MASK_event_idx(d)           ((d)->event_idx)
#define DRV_EVENT_MASK_bitFields1(d)          ((d)->u1.bitFields1)
#define DRV_EVENT_MASK_precise(d)             ((d)->u1.s1.precise)
#define DRV_EVENT_MASK_lbr_capture(d)         ((d)->u1.s1.lbr_capture)
#define DRV_EVENT_MASK_dear_capture(d)        ((d)->u1.s1.dear_capture)
#define DRV_EVENT_MASK_iear_capture(d)        ((d)->u1.s1.iear_capture)
#define DRV_EVENT_MASK_btb_capture(d)         ((d)->u1.s1.btb_capture)
#define DRV_EVENT_MASK_ipear_capture(d)       ((d)->u1.s1.ipear_capture)
#define DRV_EVENT_MASK_uncore_capture(d)      ((d)->u1.s1.uncore_capture)
#define DRV_EVENT_MASK_branch(d)              ((d)->u1.s1.branch)
#define DRV_EVENT_MASK_perf_metrics_capture(d)         \
		((d)->u1.s1.perf_metrics_capture)

#define MAX_OVERFLOW_EVENTS   16
/* This defines the maximum number of overflow events per interrupt. \
 * In order to reduce memory footprint, the value should be at least \
 * the number of fixed and general PMU registers.                    \
 * Sandybridge with HT off has 11 PMUs(3 fixed and 8 generic)
 */

typedef struct DRV_MASKS_NODE_S DRV_MASKS_NODE;
typedef DRV_MASKS_NODE * DRV_MASKS;

/*
 * @macro DRV_EVENT_MASK_NODE_S
 * @brief
 * The structure is used to store overflow events when handling PMU interrupt.
 * This approach should be more efficient than checking all event masks
 * if there are many events to be monitored
 * and only a few events among them have overflow per interrupt.
 */
struct DRV_MASKS_NODE_S {
	DRV_EVENT_MASK_NODE eventmasks[MAX_OVERFLOW_EVENTS];
	U8 masks_num; // 0 <= mask_num <= MAX_OVERFLOW_EVENTS
};

#define DRV_MASKS_masks_num(d) ((d)->masks_num)
#define DRV_MASKS_eventmasks(d) ((d)->eventmasks)

/*
 *  Dispatch table for virtualized functions.
 *  Used to enable common functionality for different
 *  processor microarchitectures
 */
typedef struct DISPATCH_NODE_S DISPATCH_NODE;
typedef DISPATCH_NODE *DISPATCH;

struct DISPATCH_NODE_S {
	VOID (*init)(PVOID);
	VOID (*fini)(PVOID);
	VOID (*write)(PVOID);
	VOID (*freeze)(PVOID);
	VOID (*restart)(PVOID);
	VOID (*read_data)(PVOID);
	VOID (*check_overflow)(DRV_MASKS);
	VOID (*swap_group)(DRV_BOOL);
	U64 (*read_lbrs)(PVOID, PVOID);
	VOID (*cleanup)(PVOID);
	VOID (*hw_errata)(void);
	VOID (*read_power)(PVOID);
	U64 (*check_overflow_errata)(ECB, U32, U64);
	VOID (*read_counts)(PVOID, U32);
	U64 (*check_overflow_gp_errata)(ECB, U64 *);
	VOID (*read_ro)(PVOID, U32, U32);
	VOID (*platform_info)(PVOID);
	VOID (*trigger_read)(PVOID, U32);
		// Counter reads triggered/initiated by User mode timer
	VOID (*scan_for_uncore)(PVOID);
	VOID (*read_metrics)(PVOID);
};

#if defined(BUILD_CHIPSET)
/*
 *  Dispatch table for virtualized functions.
 *  Used to enable common functionality for different
 *  chipset types
 */
typedef struct CS_DISPATCH_NODE_S CS_DISPATCH_NODE;
typedef CS_DISPATCH_NODE *CS_DISPATCH;
struct CS_DISPATCH_NODE_S {
	U32  (*init_chipset)(void);
		// initialize chipset (must be called before the others!)
	VOID (*start_chipset)(void); // start the chipset counters
	VOID (*read_counters)(PVOID);
		// at interrupt time, read out the chipset counters
	VOID (*stop_chipset)(void); // stop the chipset counters
	VOID (*fini_chipset)(void);
		// clean up resources and reset chipset state (called last)
	VOID (*Trigger_Read)(void);
		// GMCH counter reads triggered/initiated by User mode timer
};
extern CS_DISPATCH cs_dispatch;
#endif

/*
 * global declarations
 */

extern VOID **PMU_register_data;
extern VOID **desc_data;
extern U64 *prev_counter_data;
extern U64 *read_counter_info;
extern U64 total_ram;
extern U32 output_buffer_size;
extern U32 saved_buffer_size;
extern uid_t uid;
extern DRV_CONFIG drv_cfg;
extern volatile pid_t control_pid;
extern U64 *interrupt_counts;
extern EMON_BUFFER_DRIVER_HELPER emon_buffer_driver_helper;

extern DRV_BOOL multi_pebs_enabled;
extern DRV_BOOL unc_buf_init;

extern DRV_SETUP_INFO_NODE req_drv_setup_info;


/* needed for target agent support */
extern U32 osid;
extern DRV_BOOL sched_switch_enabled;

#if defined(BUILD_CHIPSET)
extern CHIPSET_CONFIG pma;
#endif

extern UNCORE_TOPOLOGY_INFO_NODE uncore_topology;
extern PLATFORM_TOPOLOGY_PROG_NODE platform_topology_prog_node;
extern wait_queue_head_t wait_exit;
/*
 * end of declarations
 */

/*!
 * @struct LWPMU_DEVICE_NODE_S
 * @brief  Struct to hold fields per device
 *           PMU_register_data_unc - MSR info
 *           dispatch_unc          - dispatch table
 *           em_groups_counts_unc  - # groups
 *           pcfg_unc              - config struct
 */
typedef struct LWPMU_DEVICE_NODE_S LWPMU_DEVICE_NODE;
typedef LWPMU_DEVICE_NODE * LWPMU_DEVICE;

struct LWPMU_DEVICE_NODE_S {
	VOID **PMU_register_data;
	DISPATCH dispatch;
	S32 em_groups_count;
	VOID *pcfg;
	U64 **unc_prev_value;
	U64 ***unc_acc_value;
	U64 counter_mask;
	U64 num_events;
	U32 num_units;
	VOID *ec;
	S32 *cur_group;
	S32 pci_dev_node_index;
	U32 device_type;
	LBR lbr;
	PWR pwr;
	PEBS_INFO_NODE pebs_info_node;
};

#define LWPMU_DEVICE_PMU_register_data(dev)  ((dev)->PMU_register_data)
#define LWPMU_DEVICE_dispatch(dev)           ((dev)->dispatch)
#define LWPMU_DEVICE_em_groups_count(dev)    ((dev)->em_groups_count)
#define LWPMU_DEVICE_pcfg(dev)               ((dev)->pcfg)
#define LWPMU_DEVICE_prev_value(dev)         ((dev)->unc_prev_value)
#define LWPMU_DEVICE_acc_value(dev)          ((dev)->unc_acc_value)
#define LWPMU_DEVICE_counter_mask(dev)       ((dev)->counter_mask)
#define LWPMU_DEVICE_num_events(dev)         ((dev)->num_events)
#define LWPMU_DEVICE_num_units(dev)          ((dev)->num_units)
#define LWPMU_DEVICE_ec(dev)                 ((dev)->ec)
#define LWPMU_DEVICE_cur_group(dev)          ((dev)->cur_group)
#define LWPMU_DEVICE_pci_dev_node_index(dev) ((dev)->pci_dev_node_index)
#define LWPMU_DEVICE_device_type(dev)        ((dev)->device_type)
#define LWPMU_DEVICE_lbr(dev)                ((dev)->lbr)
#define LWPMU_DEVICE_pwr(dev)                ((dev)->pwr)
#define LWPMU_DEVICE_pebs_dispatch(dev)   ((dev)->pebs_info_node.pebs_dispatch)

#define LWPMU_DEVICE_pebs_record_size(dev)                                     \
	((dev)->pebs_info_node.pebs_record_size)
#define LWPMU_DEVICE_apebs_basic_offset(dev)                                   \
	((dev)->pebs_info_node.apebs_basic_offset)
#define LWPMU_DEVICE_apebs_mem_offset(dev)                                     \
	((dev)->pebs_info_node.apebs_mem_offset)
#define LWPMU_DEVICE_apebs_gpr_offset(dev)                                     \
	((dev)->pebs_info_node.apebs_gpr_offset)
#define LWPMU_DEVICE_apebs_xmm_offset(dev)                                     \
	((dev)->pebs_info_node.apebs_xmm_offset)
#define LWPMU_DEVICE_apebs_lbr_offset(dev)                                     \
	((dev)->pebs_info_node.apebs_lbr_offset)

extern U32 num_devices;
extern U32 cur_device;
extern LWPMU_DEVICE devices;
extern U64 *pmu_state;

// Handy macro
#define TSC_SKEW(this_cpu) (cpu_tsc[this_cpu] - cpu_tsc[0])

/*
 *  The IDT / GDT descriptor for use in identifying code segments
 */
#if defined(DRV_EM64T)
#pragma pack(push, 1)
typedef struct _idtgdtDesc {
	U16 idtgdt_limit;
	PVOID idtgdt_base;
} IDTGDT_DESC;
#pragma pack(pop)

extern IDTGDT_DESC gdt_desc;
#endif

extern DRV_BOOL NMI_mode;
extern DRV_BOOL KVM_guest_mode;

#if defined(DRV_SEP_ACRN_ON)
#define SBUF_MAX_SIZE (1ULL << 22)
#define SBUF_HEAD_SIZE 64

#define TRACE_SBUF_SIZE (4 * 1024 * 1024)
#define TRACE_ELEMENT_SIZE 32 /* byte */
#define TRACE_ELEMENT_NUM                                                     \
	((TRACE_SBUF_SIZE - SBUF_HEAD_SIZE) / TRACE_ELEMENT_SIZE)

#define COLLECTOR_SEP 0
#define COLLECTOR_SOCWATCH 1

enum PROFILING_FEATURE {
	CORE_PMU_SAMPLING = 0,
	CORE_PMU_COUNTING,
	PEBS_PMU_SAMPLING,
	LBR_PMU_SAMPLING,
	UNCORE_PMU_SAMPLING,
	VM_SWITCH_TRACING,
	// Add socwatch feature
};

enum sbuf_type {
	ACRN_TRACE,
	ACRN_HVLOG,
	ACRN_SEP,
	ACRN_SOCWATCH,
	ACRN_SBUF_TYPE_MAX,
};

struct data_header {
	int32_t collector_id;
	uint16_t cpu_id;
	uint16_t data_type;
	uint64_t tsc; /* TSC */
	uint64_t payload_size;
	uint64_t reserved;
} __aligned(32);

#define PROFILING_DATA_HEADER_SIZE (sizeof(struct data_header))

struct core_pmu_sample {
	/** context where PMI is triggered */
	uint32_t os_id;
	/** the task id */
	uint32_t task_id;
	/** instruction pointer */
	uint64_t rip;
	/** the task name */
	char task[16];
	/** physical core ID */
	uint32_t cpu_id;
	/** the process id */
	uint32_t process_id;
	/** perf global status msr value (for overflow status) */
	uint64_t overflow_status;
	/** rflags */
	uint32_t rflags;
	/** code segment */
	uint32_t cs;
} __aligned(32);

#define CORE_PMU_SAMPLE_SIZE (sizeof(struct core_pmu_sample))

#define NUM_LBR_ENTRY 32

struct lbr_pmu_sample {
	/* LBR TOS */
	uint64_t lbr_tos;
	/* LBR FROM IP */
	uint64_t lbr_from_ip[NUM_LBR_ENTRY];
	/* LBR TO IP */
	uint64_t lbr_to_ip[NUM_LBR_ENTRY];
	/* LBR info */
	uint64_t lbr_info[NUM_LBR_ENTRY];
} __aligned(32);

#define LBR_PMU_SAMPLE_SIZE (sizeof(struct lbr_pmu_sample))

struct pmu_sample {
	/* core pmu sample */
	struct core_pmu_sample csample;
	/* lbr pmu sample */
	struct lbr_pmu_sample lsample;
} __aligned(32);

#define PMU_SAMPLE_SIZE (sizeof(struct pmu_sample))

struct vm_switch_trace {
	uint64_t vmenter_tsc;
	uint64_t vmexit_tsc;
	uint64_t vmexit_reason;
	int32_t os_id;
} __aligned(32);

#define VM_SWITCH_TRACE_SIZE (sizeof(struct vm_switch_trace))

typedef struct shared_buf shared_buf_t;
typedef struct profiling_control profiling_control_t;
typedef struct data_header data_header_t;
typedef struct core_pmu_sample core_pmu_sample_t;
typedef struct vm_switch_trace vm_switch_trace_t;

shared_buf_t *sbuf_allocate(uint32_t ele_num, uint32_t ele_size);
void sbuf_free(shared_buf_t *sbuf);
int sbuf_get(shared_buf_t *sbuf, uint8_t *data);
int sbuf_share_setup(uint32_t pcpu_id, uint32_t sbuf_id, shared_buf_t *sbuf);

extern shared_buf_t **samp_buf_per_cpu;

#define MAX_NR_VCPUS 4
#define MAX_NR_VMS  4
#define MAX_MSR_LIST_NUM 15
#define MAX_GROUP_NUM 1

enum MSR_OP_STATUS { MSR_OP_READY = 0, MSR_OP_REQUESTED, MSR_OP_HANDLED };

enum MSR_OP_TYPE {
	MSR_OP_NONE = 0,
	MSR_OP_READ,
	MSR_OP_WRITE,
	MSR_OP_READ_CLEAR
};

enum PMU_MSR_TYPE { PMU_MSR_CCCR = 0, PMU_MSR_ESCR, PMU_MSR_DATA };

struct profiling_msr_op {
	/* value to write or location to write into */
	uint64_t value;
	/* MSR address to read/write; last entry will have value of -1 */
	uint32_t msr_id;
	/* parameter; usage depends on operation */
	uint16_t param;
	uint8_t op_type;
	uint8_t reg_type;
};

struct profiling_msr_ops_list {
	int32_t collector_id;
	uint32_t num_entries;
	int32_t msr_op_state;
	struct profiling_msr_op entries[MAX_MSR_LIST_NUM];
};

struct profiling_vcpu_pcpu_map {
	int16_t vcpu_id;
	int16_t pcpu_id;
	uint32_t apic_id;
};

struct profiling_vm_info {
	uint16_t vm_id;
	u_char guid[16];
	char vm_name[16];
	uint16_t num_vcpus;
	struct profiling_vcpu_pcpu_map cpu_map[MAX_NR_VCPUS];
};

struct profiling_vm_info_list {
	uint16_t num_vms;
	struct profiling_vm_info vm_list[MAX_NR_VMS+1];
};

struct profiling_version_info {
	int32_t major;
	int32_t minor;
	int64_t supported_features;
	int64_t reserved;
};

struct profiling_control {
	int32_t collector_id;
	int32_t reserved;
	uint64_t switches;
};

struct profiling_pmi_config {
	uint32_t num_groups;
	uint32_t trigger_count;
	struct profiling_msr_op initial_list[MAX_GROUP_NUM][MAX_MSR_LIST_NUM];
	struct profiling_msr_op start_list[MAX_GROUP_NUM][MAX_MSR_LIST_NUM];
	struct profiling_msr_op stop_list[MAX_GROUP_NUM][MAX_MSR_LIST_NUM];
	struct profiling_msr_op entry_list[MAX_GROUP_NUM][MAX_MSR_LIST_NUM];
	struct profiling_msr_op exit_list[MAX_GROUP_NUM][MAX_MSR_LIST_NUM];
};

struct profiling_vmsw_config {
	int32_t collector_id;
	struct profiling_msr_op initial_list[MAX_MSR_LIST_NUM];
	struct profiling_msr_op entry_list[MAX_MSR_LIST_NUM];
	struct profiling_msr_op exit_list[MAX_MSR_LIST_NUM];
};

struct profiling_pcpuid {
	uint32_t leaf;
	uint32_t subleaf;
	uint32_t eax;
	uint32_t ebx;
	uint32_t ecx;
	uint32_t edx;
};

struct profiling_status {
	uint32_t samples_logged;
	uint32_t samples_dropped;
};

#endif

#endif
