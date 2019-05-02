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
#include "lwpmudrv_version.h"

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <asm/page.h>
#include <linux/cdev.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>
#include <linux/device.h>
#include <linux/ptrace.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0)
#include <linux/sched/clock.h>
#else
#include <linux/sched.h>
#endif
#include <linux/syscalls.h>
#include <asm/unistd.h>
#include <linux/compat.h>
#include <linux/vmalloc.h>
#include <linux/kthread.h>
#if defined(CONFIG_HYPERVISOR_GUEST)
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 34)
#include <asm/hypervisor.h>
#endif
#endif
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 32)
#include <xen/xen.h>
#endif

#if defined(CONFIG_XEN_HAVE_VPMU)
#include <asm/xen/hypercall.h>
#include <asm/xen/page.h>
#include <xen/interface/xenpmu.h>
#endif

#include "lwpmudrv_types.h"
#include "lwpmudrv_ecb.h"
#include "lwpmudrv_ioctl.h"
#include "lwpmudrv_struct.h"
#include "inc/ecb_iterators.h"
#include "inc/unc_common.h"

#if defined(BUILD_GFX)
#include "gfx.h"
#endif

#if defined(BUILD_CHIPSET)
#include "lwpmudrv_chipset.h"
#endif
#include "pci.h"

#include "apic.h"
#include "cpumon.h"
#include "lwpmudrv.h"
#include "utility.h"
#include "control.h"
#include "core2.h"
#include "pmi.h"

#include "output.h"
#include "linuxos.h"
#include "sys_info.h"
#include "eventmux.h"
#include "pebs.h"

MODULE_AUTHOR("Copyright(C) 2007-2018 Intel Corporation");
MODULE_VERSION(SEP_NAME "_" SEP_VERSION_STR);
MODULE_LICENSE("Dual BSD/GPL");

static struct task_struct *abnormal_handler;

typedef struct LWPMU_DEV_NODE_S LWPMU_DEV_NODE;
typedef LWPMU_DEV_NODE * LWPMU_DEV;

struct LWPMU_DEV_NODE_S {
	long buffer;
	struct semaphore sem;
	struct cdev cdev;
};

#define LWPMU_DEV_buffer(dev) ((dev)->buffer)
#define LWPMU_DEV_sem(dev) ((dev)->sem)
#define LWPMU_DEV_cdev(dev) ((dev)->cdev)

/* Global variables of the driver */
static SEP_VERSION_NODE drv_version;
U64 *read_counter_info;
U64 *prev_counter_data;
static U64 prev_counter_size;
VOID **desc_data;
U64 total_ram;
U32 output_buffer_size = OUTPUT_LARGE_BUFFER;
U32 saved_buffer_size;
static S32 desc_count;
uid_t uid;
DRV_CONFIG drv_cfg;
static DEV_CONFIG cur_pcfg;
volatile pid_t control_pid;
U64 *interrupt_counts;
static LWPMU_DEV lwpmu_control;
static LWPMU_DEV lwmod_control;
static LWPMU_DEV lwsamp_control;
static LWPMU_DEV lwsampunc_control;
static LWPMU_DEV lwsideband_control;
EMON_BUFFER_DRIVER_HELPER emon_buffer_driver_helper;

/* needed for multiple devices (core/uncore) */
U32 num_devices;
static U32 num_core_devs;
U32 cur_device;
LWPMU_DEVICE devices;
static U32 uncore_em_factor;
static unsigned long unc_timer_interval;
static struct timer_list *unc_read_timer;
static S32 max_groups_unc;
DRV_BOOL multi_pebs_enabled = FALSE;
DRV_BOOL unc_buf_init = FALSE;
DRV_BOOL NMI_mode = TRUE;
DRV_BOOL KVM_guest_mode = FALSE;
DRV_SETUP_INFO_NODE req_drv_setup_info;

/* needed for target agent support */
U32 osid = OS_ID_NATIVE;
DRV_BOOL sched_switch_enabled = FALSE;

#if defined(DRV_SEP_ACRN_ON)
struct profiling_vm_info_list  *vm_info_list;
static struct timer_list       *buffer_read_timer;
static unsigned long            buffer_timer_interval;
shared_buf_t                  **samp_buf_per_cpu;
#endif

#define UNCORE_EM_GROUP_SWAP_FACTOR 100
#define PMU_DEVICES 2 // pmu, mod

extern U32 *cpu_built_sysinfo;

#define DRV_DEVICE_DELIMITER "!"

#if defined(DRV_USE_UNLOCKED_IOCTL)
static struct mutex ioctl_lock;
#endif

#if defined(BUILD_CHIPSET)
CHIPSET_CONFIG pma;
CS_DISPATCH cs_dispatch;
#endif
static S8 *cpu_mask_bits;

/*
 *  Global data: Buffer control structure
 */
BUFFER_DESC cpu_buf;
BUFFER_DESC unc_buf;
BUFFER_DESC module_buf;
BUFFER_DESC cpu_sideband_buf;

static dev_t lwpmu_DevNum; /* the major and minor parts for SEP3 base */
static dev_t lwsamp_DevNum; /* the major and minor parts for SEP3 percpu */
static dev_t lwsampunc_DevNum;
/* the major and minor parts for SEP3 per package */
static dev_t lwsideband_DevNum;

static struct class *pmu_class;

//extern volatile int config_done;

CPU_STATE pcb;
static size_t pcb_size;
U32 *core_to_package_map;
U32 *core_to_phys_core_map;
U32 *core_to_thread_map;
U32 *core_to_dev_map;
U32 *threads_per_core;
U32 num_packages;
U64 *pmu_state;
U64 *cpu_tsc;
static U64 *prev_cpu_tsc;
static U64 *diff_cpu_tsc;
U64 *restore_bl_bypass;
U32 **restore_ha_direct2core;
U32 **restore_qpi_direct2core;
U32 *occupied_core_ids;
UNCORE_TOPOLOGY_INFO_NODE uncore_topology;
PLATFORM_TOPOLOGY_PROG_NODE platform_topology_prog_node;
static PLATFORM_TOPOLOGY_PROG_NODE req_platform_topology_prog_node;

#if !defined(DRV_SEP_ACRN_ON)
static U8 *prev_set_CR4;
#endif

wait_queue_head_t wait_exit;

extern OS_STATUS SOCPERF_Switch_Group3(void);

#if !defined(DRV_USE_UNLOCKED_IOCTL)
#define MUTEX_INIT(lock)
#define MUTEX_LOCK(lock)
#define MUTEX_UNLOCK(lock)
#else
#define MUTEX_INIT(lock) mutex_init(&(lock));
#define MUTEX_LOCK(lock) mutex_lock(&(lock))
#define MUTEX_UNLOCK(lock) mutex_unlock(&(lock))
#endif

#if defined(CONFIG_XEN_HAVE_VPMU)
typedef struct xen_pmu_params xen_pmu_params_t;
typedef struct xen_pmu_data xen_pmu_data_t;

static DEFINE_PER_CPU(xen_pmu_data_t *, sep_xenpmu_shared);
#endif

/* ------------------------------------------------------------------------- */
/*!
 * @fn  void lwpmudrv_PWR_Info(IOCTL_ARGS arg)
 *
 * @param arg - Pointer to the IOCTL structure
 *
 * @return OS_STATUS
 *
 * @brief Make a copy of the Power control information that has been passed in.
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_PWR_Info(IOCTL_ARGS arg)
{
	SEP_DRV_LOG_FLOW_IN("");
	if (DEV_CONFIG_power_capture(cur_pcfg) == FALSE) {
		SEP_DRV_LOG_WARNING_FLOW_OUT(
			"'Success' (Power capture is disabled!).");
		return OS_SUCCESS;
	}

	// make sure size of incoming arg is correct
	if ((arg->len_usr_to_drv != sizeof(PWR_NODE)) ||
	    (arg->buf_usr_to_drv == NULL)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"OS_FAULT (PWR capture has not been configured!).");
		return OS_FAULT;
	}

	//
	// First things first: Make a copy of the data for global use.
	//
	LWPMU_DEVICE_pwr(&devices[cur_device]) =
		CONTROL_Allocate_Memory((int)arg->len_usr_to_drv);
	if (!LWPMU_DEVICE_pwr(&devices[cur_device])) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Memory allocation failure!");
		return OS_NO_MEM;
	}

	if (copy_from_user(LWPMU_DEVICE_pwr(&devices[cur_device]),
			   (void __user *)arg->buf_usr_to_drv, arg->len_usr_to_drv)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Memory copy failure!");
		return OS_FAULT;
	}

	SEP_DRV_LOG_FLOW_OUT("Success");
	return OS_SUCCESS;
}

/*
 * @fn void lwpmudrv_Allocate_Restore_Buffer
 *
 * @param
 *
 * @return   OS_STATUS
 *
 * @brief    allocate buffer space to save/restore the data (for JKT, QPILL and HA register) before collection
 */
static OS_STATUS lwpmudrv_Allocate_Restore_Buffer(void)
{
	int i = 0;
	SEP_DRV_LOG_TRACE_IN("");

	if (!restore_ha_direct2core) {
		restore_ha_direct2core = CONTROL_Allocate_Memory(
			GLOBAL_STATE_num_cpus(driver_state) * sizeof(U32 *));
		if (!restore_ha_direct2core) {
			SEP_DRV_LOG_ERROR_TRACE_OUT(
				"Memory allocation failure for restore_ha_direct2core!");
			return OS_NO_MEM;
		}
		for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
			restore_ha_direct2core[i] = CONTROL_Allocate_Memory(
				MAX_BUSNO * sizeof(U32));
		}
	}
	if (!restore_qpi_direct2core) {
		restore_qpi_direct2core = CONTROL_Allocate_Memory(
			GLOBAL_STATE_num_cpus(driver_state) * sizeof(U32 *));
		if (!restore_qpi_direct2core) {
			SEP_DRV_LOG_ERROR_TRACE_OUT(
				"Memory allocation failure for restore_qpi_direct2core!");
			return OS_NO_MEM;
		}
		for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
			restore_qpi_direct2core[i] = CONTROL_Allocate_Memory(
				2 * MAX_BUSNO * sizeof(U32));
		}
	}
	if (!restore_bl_bypass) {
		restore_bl_bypass = CONTROL_Allocate_Memory(
			GLOBAL_STATE_num_cpus(driver_state) * sizeof(U64));
		if (!restore_bl_bypass) {
			SEP_DRV_LOG_ERROR_TRACE_OUT(
				"Memory allocation failure for restore_bl_bypass!");
			return OS_NO_MEM;
		}
	}

	SEP_DRV_LOG_TRACE_OUT("Success");
	return OS_SUCCESS;
}

/*
 * @fn void lwpmudrv_Allocate_Uncore_Buffer
 *
 * @param
 *
 * @return   OS_STATUS
 *
 * @brief    allocate buffer space for writing/reading uncore data
 */
static OS_STATUS lwpmudrv_Allocate_Uncore_Buffer(void)
{
	U32 i, j, k, l;
	U32 max_entries = 0;
	U32 num_entries;
	ECB ecb;

	SEP_DRV_LOG_TRACE_IN(
		""); // this function is not checking memory allocations properly

	for (i = num_core_devs; i < num_devices; i++) {
		if (!LWPMU_DEVICE_pcfg(&devices[i])) {
			continue;
		}
		LWPMU_DEVICE_acc_value(&devices[i]) =
			CONTROL_Allocate_Memory(num_packages * sizeof(U64 **));
		LWPMU_DEVICE_prev_value(&devices[i]) =
			CONTROL_Allocate_Memory(num_packages * sizeof(U64 *));
		for (j = 0; j < num_packages; j++) {
			// Allocate memory and zero out accumulator array (one per group)
			LWPMU_DEVICE_acc_value(&devices[i])[j] =
				CONTROL_Allocate_Memory(
					LWPMU_DEVICE_em_groups_count(
						&devices[i]) *
					sizeof(U64 *));
			for (k = 0;
			     k < LWPMU_DEVICE_em_groups_count(&devices[i]);
			     k++) {
				ecb = LWPMU_DEVICE_PMU_register_data(
					&devices[i])[k];
				num_entries =
					ECB_num_events(ecb) *
					LWPMU_DEVICE_num_units(&devices[i]);
				LWPMU_DEVICE_acc_value(&devices[i])[j][k] =
					CONTROL_Allocate_Memory(num_entries *
								sizeof(U64));
				for (l = 0; l < num_entries; l++) {
					LWPMU_DEVICE_acc_value(
						&devices[i])[j][k][l] = 0LL;
				}
				if (max_entries < num_entries) {
					max_entries = num_entries;
				}
			}
			// Allocate memory and zero out prev_value array (one across groups)
			LWPMU_DEVICE_prev_value(&devices[i])[j] =
				CONTROL_Allocate_Memory(max_entries *
							sizeof(U64));
			for (k = 0; k < max_entries; k++) {
				LWPMU_DEVICE_prev_value(&devices[i])[j][k] =
					0LL;
			}
		}
		max_entries = 0;
	}

	SEP_DRV_LOG_TRACE_OUT("Success");
	return OS_SUCCESS;
}

/*
 * @fn void lwpmudrv_Free_Uncore_Buffer
 *
 * @param
 *
 * @return   OS_STATUS
 *
 * @brief    Free uncore data buffers
 */
static OS_STATUS lwpmudrv_Free_Uncore_Buffer(U32 i)
{
	U32 j, k;

	SEP_DRV_LOG_TRACE_IN("");

	if (LWPMU_DEVICE_prev_value(&devices[i])) {
		for (j = 0; j < num_packages; j++) {
			LWPMU_DEVICE_prev_value(&devices[i])[j] =
				CONTROL_Free_Memory(LWPMU_DEVICE_prev_value(
					&devices[i])[j]);
		}
		LWPMU_DEVICE_prev_value(&devices[i]) = CONTROL_Free_Memory(
			LWPMU_DEVICE_prev_value(&devices[i]));
	}
	if (LWPMU_DEVICE_acc_value(&devices[i])) {
		for (j = 0; j < num_packages; j++) {
			if (LWPMU_DEVICE_acc_value(&devices[i])[j]) {
				for (k = 0; k < LWPMU_DEVICE_em_groups_count(
							&devices[i]);
				     k++) {
					LWPMU_DEVICE_acc_value(
						&devices[i])[j][k] =
						CONTROL_Free_Memory(
							LWPMU_DEVICE_acc_value(
								&devices[i])[j]
									    [k]);
				}
				LWPMU_DEVICE_acc_value(&devices[i])[j] =
					CONTROL_Free_Memory(
						LWPMU_DEVICE_acc_value(
							&devices[i])[j]);
			}
		}
		LWPMU_DEVICE_acc_value(&devices[i]) = CONTROL_Free_Memory(
			LWPMU_DEVICE_acc_value(&devices[i]));
	}

	SEP_DRV_LOG_TRACE_OUT("Success");
	return OS_SUCCESS;
}

/*
 * @fn void lwpmudrv_Free_Restore_Buffer
 *
 * @param
 *
 * @return   OS_STATUS
 *
 * @brief    allocate buffer space to save/restore the data (for JKT, QPILL and HA register) before collection
 */
static OS_STATUS lwpmudrv_Free_Restore_Buffer(void)
{
	U32 i = 0;

	SEP_DRV_LOG_TRACE_IN("");

	if (restore_ha_direct2core) {
		for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
			restore_ha_direct2core[i] =
				CONTROL_Free_Memory(restore_ha_direct2core[i]);
		}
		restore_ha_direct2core =
			CONTROL_Free_Memory(restore_ha_direct2core);
	}
	if (restore_qpi_direct2core) {
		for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
			restore_qpi_direct2core[i] =
				CONTROL_Free_Memory(restore_qpi_direct2core[i]);
		}
		restore_qpi_direct2core =
			CONTROL_Free_Memory(restore_qpi_direct2core);
	}
	if (restore_bl_bypass) {
		restore_bl_bypass = CONTROL_Free_Memory(restore_bl_bypass);
	}

	SEP_DRV_LOG_TRACE_OUT("Success");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwpmudrv_Initialize_State(void)
 *
 * @param none
 *
 * @return OS_STATUS
 *
 * @brief  Allocates the memory needed at load time.  Initializes all the
 * @brief  necessary state variables with the default values.
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Initialize_State(void)
{
	S32 i, max_cpu_id = 0;

	SEP_DRV_LOG_INIT_IN("");

	for_each_possible_cpu (i) {
		if (cpu_present(i)) {
			if (i > max_cpu_id) {
				max_cpu_id = i;
			}
		}
	}
	max_cpu_id++;

	/*
	 *  Machine Initializations
	 *  Abstract this information away into a separate entry point
	 *
	 *  Question:  Should we allow for the use of Hot-cpu
	 *    add/subtract functionality while the driver is executing?
	 */
	if (max_cpu_id > num_present_cpus()) {
		GLOBAL_STATE_num_cpus(driver_state) = max_cpu_id;
	} else {
		GLOBAL_STATE_num_cpus(driver_state) = num_present_cpus();
	}
	GLOBAL_STATE_active_cpus(driver_state) = num_online_cpus();
	GLOBAL_STATE_cpu_count(driver_state) = 0;
	GLOBAL_STATE_dpc_count(driver_state) = 0;
	GLOBAL_STATE_num_em_groups(driver_state) = 0;
	CHANGE_DRIVER_STATE(STATE_BIT_ANY, DRV_STATE_UNINITIALIZED);

	SEP_DRV_LOG_INIT_OUT("Success: num_cpus=%d, active_cpus=%d.",
			     GLOBAL_STATE_num_cpus(driver_state),
			     GLOBAL_STATE_active_cpus(driver_state));
	return OS_SUCCESS;
}

#if !defined(CONFIG_PREEMPT_COUNT) && !defined(DRV_SEP_ACRN_ON)
/* ------------------------------------------------------------------------- */
/*!
 * @fn  static void lwpmudrv_Fill_TSC_Info (PVOID param)
 *
 * @param param - pointer the buffer to fill in.
 *
 * @return none
 *
 * @brief  Read the TSC and write into the correct array slot.
 *
 * <I>Special Notes</I>
 */
atomic_t read_now;
static wait_queue_head_t read_tsc_now;
static VOID lwpmudrv_Fill_TSC_Info(PVOID param)
{
	U32 this_cpu;

	SEP_DRV_LOG_TRACE_IN("");

	preempt_disable();
	this_cpu = CONTROL_THIS_CPU();
	preempt_enable();
	//
	// Wait until all CPU's are ready to proceed
	// This will serve as a synchronization point to compute tsc skews.
	//

	if (atomic_read(&read_now) >= 1) {
		if (atomic_dec_and_test(&read_now) == FALSE) {
			wait_event_interruptible(read_tsc_now,
						 (atomic_read(&read_now) >= 1));
		}
	} else {
		wake_up_interruptible_all(&read_tsc_now);
	}
	UTILITY_Read_TSC(&cpu_tsc[this_cpu]);
	SEP_DRV_LOG_TRACE("This cpu %d --- tsc --- 0x%llx.", this_cpu,
			  cpu_tsc[this_cpu]);

	SEP_DRV_LOG_TRACE_OUT("Success");
}
#endif

/*********************************************************************
 *  Internal Driver functions
 *     Should be called only from the lwpmudrv_DeviceControl routine
 *********************************************************************/

/* ------------------------------------------------------------------------- */
/*!
 * @fn static void lwpmudrv_Dump_Tracer(const char *)
 *
 * @param Name of the tracer
 *
 * @return void
 *
 * @brief  Function that handles the generation of markers into the ftrace stream
 *
 * <I>Special Notes</I>
 */
static void lwpmudrv_Dump_Tracer(const char *name, U64 tsc)
{
	SEP_DRV_LOG_TRACE_IN("");
	if (tsc == 0) {
		preempt_disable();
		UTILITY_Read_TSC(&tsc);
		tsc -= TSC_SKEW(CONTROL_THIS_CPU());
		preempt_enable();
	}
	SEP_DRV_LOG_TRACE_OUT("Success");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwpmudrv_Version(IOCTL_ARGS arg)
 *
 * @param arg - pointer to the IOCTL_ARGS structure
 *
 * @return OS_STATUS
 *
 * @brief  Local function that handles the LWPMU_IOCTL_VERSION call.
 * @brief  Returns the version number of the kernel mode sampling.
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Version(IOCTL_ARGS arg)
{
	OS_STATUS status;

	SEP_DRV_LOG_FLOW_IN("");

	// Check if enough space is provided for collecting the data
	if ((arg->len_drv_to_usr != sizeof(U32)) ||
	    (arg->buf_drv_to_usr == NULL)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Invalid arguments.");
		return OS_FAULT;
	}

	status = put_user(SEP_VERSION_NODE_sep_version(&drv_version),
			  (U32 __user *)arg->buf_drv_to_usr);

	SEP_DRV_LOG_FLOW_OUT("Return value: %d", status);
	return status;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwpmudrv_Reserve(IOCTL_ARGS arg)
 *
 * @param arg - pointer to the IOCTL_ARGS structure
 *
 * @return OS_STATUS
 *
 * @brief
 * @brief  Local function that handles the LWPMU_IOCTL_RESERVE call.
 * @brief  Sets the state to RESERVED if possible.  Returns BUSY if unable
 * @brief  to reserve the PMU.
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Reserve(IOCTL_ARGS arg)
{
	OS_STATUS status = OS_SUCCESS;

	SEP_DRV_LOG_FLOW_IN("");

	// Check if enough space is provided for collecting the data
	if ((arg->len_drv_to_usr != sizeof(S32)) ||
	    (arg->buf_drv_to_usr == NULL)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Invalid arguments.");
		return OS_FAULT;
	}

	status = put_user(!CHANGE_DRIVER_STATE(STATE_BIT_UNINITIALIZED,
					       DRV_STATE_RESERVED),
			  (int __user*)arg->buf_drv_to_usr);

	SEP_DRV_LOG_FLOW_OUT("Return value: %d", status);
	return status;
}

#if !defined(DRV_SEP_ACRN_ON)
/* ------------------------------------------------------------------------- */
/*!
 * @fn static OS_STATUS lwpmudrv_Finish_Op(void)
 *
 * @param - none
 *
 * @return OS_STATUS
 *
 * @brief Finalize PMU after collection
 *
 * <I>Special Notes</I>
 */
static VOID lwpmudrv_Finish_Op(PVOID param)
{
	U32 this_cpu = CONTROL_THIS_CPU();
	U32 dev_idx = core_to_dev_map[this_cpu];
	DISPATCH dispatch = LWPMU_DEVICE_dispatch(&devices[dev_idx]);

	SEP_DRV_LOG_TRACE_IN("");

	if (dispatch != NULL && dispatch->fini != NULL) {
		dispatch->fini(&dev_idx);
	}

	SEP_DRV_LOG_TRACE_OUT("");
}
#endif

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static VOID lwpmudrv_Clean_Up(DRV_BOOL)
 *
 * @param  DRV_BOOL finish - Flag to call finish
 *
 * @return VOID
 *
 * @brief  Cleans up the memory allocation.
 *
 * <I>Special Notes</I>
 */
static VOID lwpmudrv_Clean_Up(DRV_BOOL finish)
{
	U32 i;

	SEP_DRV_LOG_FLOW_IN("");

	if (DRV_CONFIG_use_pcl(drv_cfg) == TRUE) {
		drv_cfg = CONTROL_Free_Memory(drv_cfg);
		goto signal_end;
	}

	if (devices) {
		U32 id;
		U32 num_groups = 0;
		EVENT_CONFIG ec;
		DISPATCH dispatch_unc = NULL;

		for (id = 0; id < num_devices; id++) {
			if (LWPMU_DEVICE_pcfg(&devices[id])) {
				if (LWPMU_DEVICE_device_type(&devices[id]) ==
				    DEVICE_INFO_UNCORE) {
					dispatch_unc = LWPMU_DEVICE_dispatch(
						&devices[id]);
					if (dispatch_unc &&
					    dispatch_unc->fini) {
						SEP_DRV_LOG_TRACE(
							"LWP: calling UNC Init.");
						dispatch_unc->fini(
							(PVOID *)&id);
					}
					lwpmudrv_Free_Uncore_Buffer(id);
				} else if (finish) {
#if !defined(DRV_SEP_ACRN_ON)
					CONTROL_Invoke_Parallel(
						lwpmudrv_Finish_Op, NULL);
#endif
				}
			}

			if (LWPMU_DEVICE_PMU_register_data(&devices[id])) {
				ec = LWPMU_DEVICE_ec(&devices[id]);
				if (LWPMU_DEVICE_device_type(&devices[id]) ==
				    DEVICE_INFO_CORE) {
					num_groups =
						EVENT_CONFIG_num_groups(ec);
				} else {
					num_groups =
						EVENT_CONFIG_num_groups_unc(ec);
				}
				for (i = 0; i < num_groups; i++) {
					LWPMU_DEVICE_PMU_register_data(
						&devices[id])[i] =
						CONTROL_Free_Memory(
							LWPMU_DEVICE_PMU_register_data(
								&devices[id])[i]);
				}
				LWPMU_DEVICE_PMU_register_data(&devices[id]) =
					CONTROL_Free_Memory(
						LWPMU_DEVICE_PMU_register_data(
							&devices[id]));
			}
			LWPMU_DEVICE_pcfg(&devices[id]) = CONTROL_Free_Memory(
				LWPMU_DEVICE_pcfg(&devices[id]));
			LWPMU_DEVICE_ec(&devices[id]) = CONTROL_Free_Memory(
				LWPMU_DEVICE_ec(&devices[id]));
			if (LWPMU_DEVICE_lbr(&devices[id])) {
				LWPMU_DEVICE_lbr(&devices[id]) =
					CONTROL_Free_Memory(
						LWPMU_DEVICE_lbr(&devices[id]));
			}
			if (LWPMU_DEVICE_pwr(&devices[id])) {
				LWPMU_DEVICE_pwr(&devices[id]) =
					CONTROL_Free_Memory(
						LWPMU_DEVICE_pwr(&devices[id]));
			}
			if (LWPMU_DEVICE_cur_group(&devices[id])) {
				LWPMU_DEVICE_cur_group(&devices[id]) =
					CONTROL_Free_Memory(
						LWPMU_DEVICE_cur_group(
							&devices[id]));
			}
		}
		devices = CONTROL_Free_Memory(devices);
	}

	if (desc_data) {
		for (i = 0; i < GLOBAL_STATE_num_descriptors(driver_state);
		     i++) {
			desc_data[i] = CONTROL_Free_Memory(desc_data[i]);
		}
		desc_data = CONTROL_Free_Memory(desc_data);
	}

	if (restore_bl_bypass) {
		restore_bl_bypass = CONTROL_Free_Memory(restore_bl_bypass);
	}

	if (restore_qpi_direct2core) {
		for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
			restore_qpi_direct2core[i] =
				CONTROL_Free_Memory(restore_qpi_direct2core[i]);
		}
		restore_qpi_direct2core =
			CONTROL_Free_Memory(restore_qpi_direct2core);
	}

	if (restore_ha_direct2core) {
		for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
			restore_ha_direct2core[i] =
				CONTROL_Free_Memory(restore_ha_direct2core[i]);
		}
		restore_ha_direct2core =
			CONTROL_Free_Memory(restore_ha_direct2core);
	}

	drv_cfg = CONTROL_Free_Memory(drv_cfg);
	pmu_state = CONTROL_Free_Memory(pmu_state);
	cpu_mask_bits = CONTROL_Free_Memory(cpu_mask_bits);
	core_to_dev_map = CONTROL_Free_Memory(core_to_dev_map);
#if defined(BUILD_CHIPSET)
	pma = CONTROL_Free_Memory(pma);
#endif

signal_end:
	GLOBAL_STATE_num_em_groups(driver_state) = 0;
	GLOBAL_STATE_num_descriptors(driver_state) = 0;
	num_devices = 0;
	num_core_devs = 0;
	max_groups_unc = 0;
	control_pid = 0;
	unc_buf_init = FALSE;

	OUTPUT_Cleanup();
	memset(pcb, 0, pcb_size);

	SEP_DRV_LOG_FLOW_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static NTSTATUS lwpmudrv_Initialize_Driver (PVOID buf_usr_to_drv, size_t len_usr_to_drv)
 *
 * @param  buf_usr_to_drv   - pointer to the input buffer
 * @param  len_usr_to_drv   - size of the input buffer
 *
 * @return NTSTATUS
 *
 * @brief  Local function that handles the LWPMU_IOCTL_INIT_DRIVER call.
 * @brief  Sets up the interrupt handler.
 * @brief  Set up the output buffers/files needed to make the driver
 * @brief  operational.
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Initialize_Driver(PVOID buf_usr_to_drv,
					size_t len_usr_to_drv)
{
	S32 cpu_num;
	int status = OS_SUCCESS;

	SEP_DRV_LOG_FLOW_IN("");

	if (buf_usr_to_drv == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Invalid arguments.");
		return OS_FAULT;
	}

	if (!CHANGE_DRIVER_STATE(STATE_BIT_RESERVED, DRV_STATE_IDLE)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Unexpected driver state!");
		return OS_FAULT;
	}

	interrupt_counts = NULL;
	pmu_state = NULL;

	drv_cfg = CONTROL_Allocate_Memory(len_usr_to_drv);
	if (!drv_cfg) {
		status = OS_NO_MEM;
		SEP_DRV_LOG_ERROR("Memory allocation failure for drv_cfg!");
		goto clean_return;
	}

	if (copy_from_user(drv_cfg, (void __user *)buf_usr_to_drv,
			len_usr_to_drv)) {
		SEP_DRV_LOG_ERROR("Memory copy failure for drv_cfg!");
		status = OS_FAULT;
		goto clean_return;
	}

	if (DRV_CONFIG_enable_cp_mode(drv_cfg)) {
#if (defined(DRV_EM64T))
		if (output_buffer_size == OUTPUT_LARGE_BUFFER) {
			output_buffer_size = OUTPUT_CP_BUFFER;
		}
#endif
		interrupt_counts = CONTROL_Allocate_Memory(
			GLOBAL_STATE_num_cpus(driver_state) *
			DRV_CONFIG_num_events(drv_cfg) * sizeof(U64));
		if (interrupt_counts == NULL) {
			SEP_DRV_LOG_ERROR(
				"Memory allocation failure for interrupt_counts!");
			status = OS_NO_MEM;
			goto clean_return;
		}
	} else if (output_buffer_size == OUTPUT_CP_BUFFER) {
		output_buffer_size = OUTPUT_LARGE_BUFFER;
	}

	if (DRV_CONFIG_use_pcl(drv_cfg) == TRUE) {
		SEP_DRV_LOG_FLOW_OUT("Success, using PCL.");
		return OS_SUCCESS;
	}

	pmu_state = CONTROL_Allocate_KMemory(
		GLOBAL_STATE_num_cpus(driver_state) * sizeof(U64) * 3);
	if (!pmu_state) {
		SEP_DRV_LOG_ERROR("Memory allocation failure for pmu_state!");
		status = OS_NO_MEM;
		goto clean_return;
	}
	uncore_em_factor = 0;
	for (cpu_num = 0; cpu_num < GLOBAL_STATE_num_cpus(driver_state);
	     cpu_num++) {
		CPU_STATE_accept_interrupt(&pcb[cpu_num]) = 1;
		CPU_STATE_initial_mask(&pcb[cpu_num]) = 1;
		CPU_STATE_group_swap(&pcb[cpu_num]) = 1;
		CPU_STATE_reset_mask(&pcb[cpu_num]) = 0;
		CPU_STATE_num_samples(&pcb[cpu_num]) = 0;
		CPU_STATE_last_p_state_valid(&pcb[cpu_num]) = FALSE;
#if defined(DRV_CPU_HOTPLUG)
		CPU_STATE_offlined(&pcb[cpu_num]) = TRUE;
#else
		CPU_STATE_offlined(&pcb[cpu_num]) = FALSE;
#endif
		CPU_STATE_nmi_handled(&pcb[cpu_num]) = 0;
	}

	DRV_CONFIG_seed_name(drv_cfg) = NULL;
	DRV_CONFIG_seed_name_len(drv_cfg) = 0;

	SEP_DRV_LOG_TRACE("Config : size = %d.", DRV_CONFIG_size(drv_cfg));
	SEP_DRV_LOG_TRACE("Config : counting_mode = %d.",
			  DRV_CONFIG_counting_mode(drv_cfg));

	control_pid = current->pid;
	SEP_DRV_LOG_TRACE("Control PID = %d.", control_pid);

	if (core_to_dev_map == NULL) {
		core_to_dev_map = CONTROL_Allocate_Memory(
			GLOBAL_STATE_num_cpus(driver_state) * sizeof(U32));
	}

	if (DRV_CONFIG_counting_mode(drv_cfg) == FALSE) {
		if (cpu_buf == NULL) {
			cpu_buf = CONTROL_Allocate_Memory(
				GLOBAL_STATE_num_cpus(driver_state) *
				sizeof(BUFFER_DESC_NODE));
			if (!cpu_buf) {
				SEP_DRV_LOG_ERROR(
					"Memory allocation failure for cpu_buf!");
				status = OS_NO_MEM;
				goto clean_return;
			}
		}

		if (module_buf == NULL) {
			module_buf = CONTROL_Allocate_Memory(
				sizeof(BUFFER_DESC_NODE));
			if (!module_buf) {
				status = OS_NO_MEM;
				goto clean_return;
			}
		}

#if defined(CONFIG_TRACEPOINTS)
		multi_pebs_enabled = (DRV_CONFIG_multi_pebs_enabled(drv_cfg) &&
				      (DRV_SETUP_INFO_page_table_isolation(
					       &req_drv_setup_info) ==
				       DRV_SETUP_INFO_PTI_DISABLED));
#endif
		if (multi_pebs_enabled || sched_switch_enabled) {
			if (cpu_sideband_buf == NULL) {
				cpu_sideband_buf = CONTROL_Allocate_Memory(
					GLOBAL_STATE_num_cpus(driver_state) *
					sizeof(BUFFER_DESC_NODE));
				if (!cpu_sideband_buf) {
					SEP_DRV_LOG_ERROR(
						"Memory allocation failure for cpu_sideband_buf!");
					status = OS_NO_MEM;
					goto clean_return;
				}
			}
		}

#if defined(DRV_SEP_ACRN_ON)
		if (samp_buf_per_cpu == NULL) {
			samp_buf_per_cpu =
				(shared_buf_t **)CONTROL_Allocate_Memory(
					GLOBAL_STATE_num_cpus(driver_state) *
					sizeof(shared_buf_t *));
			if (!samp_buf_per_cpu) {
				SEP_PRINT_ERROR(
					"lwpmudrv_Initialize: unable to allocate memory for samp_buf_per_cpu\n");
				goto clean_return;
			}
		}

		for (cpu_num = 0; cpu_num < GLOBAL_STATE_num_cpus(driver_state);
		     cpu_num++) {
			samp_buf_per_cpu[cpu_num] = sbuf_allocate(
				TRACE_ELEMENT_NUM, TRACE_ELEMENT_SIZE);
			if (!samp_buf_per_cpu[cpu_num]) {
				pr_err("Failed to allocate sampbuf on cpu%d\n",
				       cpu_num);
				goto clean_return;
			}

			status = sbuf_share_setup(cpu_num, ACRN_SEP,
						  samp_buf_per_cpu[cpu_num]);
			if (status < 0) {
				status = OS_FAULT;
				pr_err("Failed to set up sampbuf on cpu%d\n",
				       cpu_num);
				goto clean_return;
			}
		}
#endif

		/*
	 * Allocate the output and control buffers for each CPU in the system
	 * Allocate and set up the temp output files for each CPU in the system
	 * Allocate and set up the temp outout file for detailing the Modules in the system
	 */
		status = OUTPUT_Initialize();
		if (status != OS_SUCCESS) {
			SEP_DRV_LOG_ERROR("OUTPUT_Initialize failed!");
			goto clean_return;
		}

		/*
	 * Program the APIC and set up the interrupt handler
	 */
#if !defined(DRV_SEP_ACRN_ON)
		CPUMON_Install_Cpuhooks();
#endif
		SEP_DRV_LOG_TRACE("Finished Installing cpu hooks.");
#if defined(DRV_CPU_HOTPLUG)
		for (cpu_num = 0; cpu_num < GLOBAL_STATE_num_cpus(driver_state);
		     cpu_num++) {
			if (cpu_built_sysinfo &&
			    cpu_built_sysinfo[cpu_num] == 0) {
				cpu_tsc[cpu_num] = cpu_tsc[0];
				CONTROL_Invoke_Cpu(cpu_num, SYS_INFO_Build_Cpu,
						   NULL);
			}
		}
#endif

#if defined(DRV_EM64T)
		SYS_Get_GDT_Base((PVOID *)&gdt_desc);
#endif
		SEP_DRV_LOG_TRACE("About to install module notification.");
		LINUXOS_Install_Hooks();
	}

clean_return:
	if (status != OS_SUCCESS) {
		drv_cfg = CONTROL_Free_Memory(drv_cfg);
		interrupt_counts = CONTROL_Free_Memory(interrupt_counts);
		pmu_state = CONTROL_Free_Memory(pmu_state);
		CHANGE_DRIVER_STATE(STATE_BIT_ANY, DRV_STATE_TERMINATING);
	}

	SEP_DRV_LOG_FLOW_OUT("Return value: %d.", status);
	return status;
}
/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwpmudrv_Initialize (PVOID buf_usr_to_drv, size_t len_usr_to_drv)
 *
 * @param  buf_usr_to_drv  - pointer to the input buffer
 * @param  len_usr_to_drv  - size of the input buffer
 *
 * @return OS_STATUS
 *
 * @brief  Local function that handles the LWPMU_IOCTL_INIT call.
 * @brief  Sets up the interrupt handler.
 * @brief  Set up the output buffers/files needed to make the driver
 * @brief  operational.
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Initialize(PVOID buf_usr_to_drv,
				     size_t len_usr_to_drv)
{
	int status = OS_SUCCESS;
	S32 cpu_num;

	SEP_DRV_LOG_FLOW_IN("");

	if (buf_usr_to_drv == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Invalid arguments.");
		return OS_FAULT;
	}

	if (cur_device >= num_devices) {
		CHANGE_DRIVER_STATE(STATE_BIT_ANY, DRV_STATE_TERMINATING);
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"No more devices to allocate! Wrong lwpmudrv_Init_Num_Devices.");
		return OS_FAULT;
	}

	/*
	 *   Program State Initializations
	 */
	LWPMU_DEVICE_pcfg(&devices[cur_device]) =
		CONTROL_Allocate_Memory(len_usr_to_drv);
	if (!LWPMU_DEVICE_pcfg(&devices[cur_device])) {
		status = OS_NO_MEM;
		SEP_DRV_LOG_ERROR("Memory allocation failure for pcfg!");
		goto clean_return;
	}

	if (copy_from_user(LWPMU_DEVICE_pcfg(&devices[cur_device]),
			   (void __user *)buf_usr_to_drv, len_usr_to_drv)) {
		SEP_DRV_LOG_ERROR("Memory copy failure for pcfg!");
		status = OS_FAULT;
		goto clean_return;
	}
	cur_pcfg = (DEV_CONFIG)LWPMU_DEVICE_pcfg(&devices[cur_device]);

	if (DRV_CONFIG_use_pcl(drv_cfg) == TRUE) {
		SEP_DRV_LOG_FLOW_OUT("Success, using PCL.");
		return OS_SUCCESS;
	}

	LWPMU_DEVICE_dispatch(&devices[cur_device]) =
		UTILITY_Configure_CPU(DEV_CONFIG_dispatch_id(cur_pcfg));
	if (LWPMU_DEVICE_dispatch(&devices[cur_device]) == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Dispatch pointer is NULL!");
		status = OS_INVALID;
		goto clean_return;
	}

#if !defined(DRV_SEP_ACRN_ON)
	if (DRV_CONFIG_counting_mode(drv_cfg) == FALSE) {
		status = PEBS_Initialize(cur_device);
		if (status != OS_SUCCESS) {
			SEP_DRV_LOG_ERROR("PEBS_Initialize failed!");
			goto clean_return;
		}
	}
#endif

	/* Create core to device ID map */
	for (cpu_num = 0; cpu_num < GLOBAL_STATE_num_cpus(driver_state);
	     cpu_num++) {
		if (CPU_STATE_core_type(&pcb[cpu_num]) ==
		    DEV_CONFIG_core_type(cur_pcfg)) {
			core_to_dev_map[cpu_num] = cur_device;
		}
	}
	num_core_devs++; //New core device
	LWPMU_DEVICE_device_type(&devices[cur_device]) = DEVICE_INFO_CORE;

clean_return:
	if (status != OS_SUCCESS) {
		// release all memory allocated in this function:
		lwpmudrv_Clean_Up(FALSE);
#if !defined(DRV_SEP_ACRN_ON)
		PEBS_Destroy();
#endif
		CHANGE_DRIVER_STATE(STATE_BIT_ANY, DRV_STATE_TERMINATING);
	}

	SEP_DRV_LOG_FLOW_OUT("Return value: %d.", status);
	return status;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwpmudrv_Initialize_Num_Devices(IOCTL_ARGS arg)
 *
 * @param arg - pointer to the IOCTL_ARGS structure
 *
 * @return OS_STATUS
 *
 * @brief
 * @brief  Local function that handles the LWPMU_IOCTL_INIT_NUM_DEV call.
 * @brief  Init # uncore devices.
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Initialize_Num_Devices(IOCTL_ARGS arg)
{
	SEP_DRV_LOG_FLOW_IN("");

	// Check if enough space is provided for collecting the data
	if ((arg->len_usr_to_drv != sizeof(U32)) ||
	    (arg->buf_usr_to_drv == NULL)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Invalid arguments.");
		return OS_FAULT;
	}

	if (copy_from_user(&num_devices, (void __user *)arg->buf_usr_to_drv,
			   arg->len_usr_to_drv)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Memory copy failure");
		return OS_FAULT;
	}
	/*
	 *   Allocate memory for number of devices
	 */
	if (num_devices != 0) {
		devices = CONTROL_Allocate_Memory(num_devices *
						  sizeof(LWPMU_DEVICE_NODE));
		if (!devices) {
			SEP_DRV_LOG_ERROR_FLOW_OUT(
				"Unable to allocate memory for devices!");
			return OS_NO_MEM;
		}
	}
	cur_device = 0;

	SEP_DRV_LOG_FLOW_OUT("Success: num_devices=%d, devices=0x%p.",
			     num_devices, devices);
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwpmudrv_Initialize_UNC(PVOID buf_usr_to_drv, U32 len_usr_to_drv)
 *
 * @param  buf_usr_to_drv   - pointer to the input buffer
 * @param  len_usr_to_drv   - size of the input buffer
 *
 * @return OS_STATUS
 *
 * @brief  Local function that handles the LWPMU_IOCTL_INIT call.
 * @brief  Sets up the interrupt handler.
 * @brief  Set up the output buffers/files needed to make the driver
 * @brief  operational.
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Initialize_UNC(PVOID buf_usr_to_drv,
					 U32 len_usr_to_drv)
{
	DEV_UNC_CONFIG pcfg_unc;
	U32 i;
	int status = OS_SUCCESS;

	SEP_DRV_LOG_FLOW_IN("");

	if (GET_DRIVER_STATE() != DRV_STATE_IDLE) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Skipped: current state is not IDLE.");
		return OS_IN_PROGRESS;
	}

	if (!devices) {
		CHANGE_DRIVER_STATE(STATE_BIT_ANY, DRV_STATE_TERMINATING);
		SEP_DRV_LOG_ERROR_FLOW_OUT("No devices allocated!");
		return OS_INVALID;
	}

	/*
	 *   Program State Initializations:
	 *   Foreach device, copy over pcfg and configure dispatch table
	 */
	if (cur_device >= num_devices) {
		CHANGE_DRIVER_STATE(STATE_BIT_ANY, DRV_STATE_TERMINATING);
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"No more devices to allocate! Wrong lwpmudrv_Init_Num_Devices.");
		return OS_FAULT;
	}
	if (buf_usr_to_drv == NULL) {
		CHANGE_DRIVER_STATE(STATE_BIT_ANY, DRV_STATE_TERMINATING);
		SEP_DRV_LOG_ERROR_FLOW_OUT("Invalid arguments.");
		return OS_FAULT;
	}
	if (len_usr_to_drv != sizeof(DEV_UNC_CONFIG_NODE)) {
		CHANGE_DRIVER_STATE(STATE_BIT_ANY, DRV_STATE_TERMINATING);
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Got len_usr_to_drv=%d, expecting size=%d",
			len_usr_to_drv, (int)sizeof(DEV_UNC_CONFIG_NODE));
		return OS_FAULT;
	}
	// allocate memory
	LWPMU_DEVICE_pcfg(&devices[cur_device]) =
		CONTROL_Allocate_Memory(sizeof(DEV_UNC_CONFIG_NODE));
	// copy over pcfg
	if (copy_from_user(LWPMU_DEVICE_pcfg(&devices[cur_device]),
			   (void __user *)buf_usr_to_drv, len_usr_to_drv)) {
		CHANGE_DRIVER_STATE(STATE_BIT_ANY, DRV_STATE_TERMINATING);
		SEP_DRV_LOG_ERROR_FLOW_OUT("Failed to copy from user!");
		return OS_FAULT;
	}
	// configure dispatch from dispatch_id
	pcfg_unc = (DEV_UNC_CONFIG)LWPMU_DEVICE_pcfg(&devices[cur_device]);
	if (!pcfg_unc) {
		CHANGE_DRIVER_STATE(STATE_BIT_ANY, DRV_STATE_TERMINATING);
		SEP_DRV_LOG_ERROR_FLOW_OUT("Invalid pcfg_unc.");
		return OS_INVALID;
	}

	LWPMU_DEVICE_dispatch(&devices[cur_device]) =
		UTILITY_Configure_CPU(DEV_UNC_CONFIG_dispatch_id(pcfg_unc));
	if (LWPMU_DEVICE_dispatch(&devices[cur_device]) == NULL) {
		CHANGE_DRIVER_STATE(STATE_BIT_ANY, DRV_STATE_TERMINATING);
		SEP_DRV_LOG_ERROR_FLOW_OUT("Unable to configure CPU!");
		return OS_FAULT;
	}

	LWPMU_DEVICE_cur_group(&devices[cur_device]) =
		CONTROL_Allocate_Memory(num_packages * sizeof(S32));
	if (LWPMU_DEVICE_cur_group(&devices[cur_device]) == NULL) {
		CHANGE_DRIVER_STATE(STATE_BIT_ANY, DRV_STATE_TERMINATING);
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Cur_grp allocation failed for device %u!", cur_device);
		return OS_NO_MEM;
	}
	for (i = 0; i < num_packages; i++) {
		LWPMU_DEVICE_cur_group(&devices[cur_device])[i] = 0;
	}

	LWPMU_DEVICE_em_groups_count(&devices[cur_device]) = 0;
	LWPMU_DEVICE_num_units(&devices[cur_device]) = 0;
	LWPMU_DEVICE_device_type(&devices[cur_device]) = DEVICE_INFO_UNCORE;

	if (DRV_CONFIG_counting_mode(drv_cfg) == FALSE) {
		if (unc_buf == NULL) {
			unc_buf = CONTROL_Allocate_Memory(
				num_packages * sizeof(BUFFER_DESC_NODE));
			if (!unc_buf) {
				CHANGE_DRIVER_STATE(STATE_BIT_ANY,
						    DRV_STATE_TERMINATING);
				SEP_DRV_LOG_ERROR_FLOW_OUT(
					"Memory allocation failure.");
				return OS_NO_MEM;
			}
		}

		if (!unc_buf_init) {
			status = OUTPUT_Initialize_UNC();
			if (status != OS_SUCCESS) {
				CHANGE_DRIVER_STATE(STATE_BIT_ANY,
						    DRV_STATE_TERMINATING);
				SEP_DRV_LOG_ERROR_FLOW_OUT(
					"OUTPUT_Initialize failed!");
				return status;
			}
			unc_buf_init = TRUE;
		}
	}

	SEP_DRV_LOG_FLOW_OUT("unc dispatch id = %d.",
			     DEV_UNC_CONFIG_dispatch_id(pcfg_unc));

	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwpmudrv_Terminate(void)
 *
 * @param  none
 *
 * @return OS_STATUS
 *
 * @brief  Local function that handles the DRV_OPERATION_TERMINATE call.
 * @brief  Cleans up the interrupt handler and resets the PMU state.
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Terminate(void)
{
	SEP_DRV_LOG_FLOW_IN("");

	if (GET_DRIVER_STATE() == DRV_STATE_UNINITIALIZED) {
		SEP_DRV_LOG_FLOW_OUT("Success (already uninitialized).");
		return OS_SUCCESS;
	}

	if (!CHANGE_DRIVER_STATE(STATE_BIT_STOPPED | STATE_BIT_TERMINATING,
				 DRV_STATE_UNINITIALIZED)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Unexpected state!");
		return OS_FAULT;
	}

	if (drv_cfg && DRV_CONFIG_counting_mode(drv_cfg) == FALSE) {
		LINUXOS_Uninstall_Hooks();
	}

	lwpmudrv_Clean_Up(TRUE);

	SEP_DRV_LOG_FLOW_OUT("Success");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn static void lwpmudrv_Switch_To_Next_Group(param)
 *
 * @param  none
 *
 * @return none
 *
 * @brief  Switch to the next event group for both core and uncore.
 * @brief  This function assumes an active collection is frozen
 * @brief  or no collection is active.
 *
 * <I>Special Notes</I>
 */
static VOID lwpmudrv_Switch_To_Next_Group(void)
{
	S32 cpuid;
	U32 i, j;
	CPU_STATE pcpu;
	EVENT_CONFIG ec;
	DEV_UNC_CONFIG pcfg_unc;
	DISPATCH dispatch_unc;
	ECB pecb_unc = NULL;
	U32 cur_grp = 0;

	SEP_DRV_LOG_FLOW_IN("");

	for (cpuid = 0; cpuid < GLOBAL_STATE_num_cpus(driver_state); cpuid++) {
		pcpu = &pcb[cpuid];
		ec = (EVENT_CONFIG)LWPMU_DEVICE_ec(
			&devices[core_to_dev_map[cpuid]]);
		CPU_STATE_current_group(pcpu)++;
		// make the event group list circular
		CPU_STATE_current_group(pcpu) %= EVENT_CONFIG_num_groups(ec);
	}

	if (num_devices) {
		for (i = num_core_devs; i < num_devices; i++) {
			pcfg_unc = LWPMU_DEVICE_pcfg(&devices[i]);
			dispatch_unc = LWPMU_DEVICE_dispatch(&devices[i]);
			if (LWPMU_DEVICE_em_groups_count(&devices[i]) > 1) {
				if (pcb && pcfg_unc && dispatch_unc &&
				    DRV_CONFIG_emon_mode(drv_cfg)) {
					for (j = 0; j < num_packages; j++) {
						cur_grp =
							LWPMU_DEVICE_cur_group(
								&devices[i])[j];
						pecb_unc =
							LWPMU_DEVICE_PMU_register_data(
								&devices[i])
								[cur_grp];
						LWPMU_DEVICE_cur_group(
							&devices[i])[j]++;
						if (CPU_STATE_current_group(
							    &pcb[0]) == 0) {
							LWPMU_DEVICE_cur_group(
								&devices[i])[j] =
								0;
						}
						LWPMU_DEVICE_cur_group(
							&devices[i])[j] %=
							LWPMU_DEVICE_em_groups_count(
								&devices[i]);
					}
					SEP_DRV_LOG_TRACE(
						"Swap Group to %d for device %d.",
						cur_grp, i);
					if (pecb_unc &&
					    ECB_device_type(pecb_unc) ==
						    DEVICE_UNC_SOCPERF) {
						SOCPERF_Switch_Group3();
					}
				}
			}
		}
	}

	SEP_DRV_LOG_FLOW_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwmpudrv_Get_Driver_State(IOCTL_ARGS arg)
 *
 * @param arg - pointer to the IOCTL_ARGS structure
 *
 * @return OS_STATUS
 *
 * @brief  Local function that handles the LWPMU_IOCTL_GET_Driver_State call.
 * @brief  Returns the current driver state.
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Get_Driver_State(IOCTL_ARGS arg)
{
	OS_STATUS status = OS_SUCCESS;

	SEP_DRV_LOG_TRACE_IN("");

	// Check if enough space is provided for collecting the data
	if ((arg->len_drv_to_usr != sizeof(U32)) ||
	    (arg->buf_drv_to_usr == NULL)) {
		SEP_DRV_LOG_ERROR_TRACE_OUT("Invalid arguments!");
		return OS_FAULT;
	}

	status = put_user(GET_DRIVER_STATE(), (U32 __user*)arg->buf_drv_to_usr);

	SEP_DRV_LOG_TRACE_OUT("Return value: %d.", status);
	return status;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn static OS_STATUS lwpmudrv_Pause_Uncore(void)
 *
 * @param - 1 if switching group, 0 otherwise
 *
 * @return OS_STATUS
 *
 * @brief Pause the uncore collection
 *
 * <I>Special Notes</I>
 */
static VOID lwpmudrv_Pause_Uncore(PVOID param)
{
	U32 i;
	U32 switch_grp;
	DEV_UNC_CONFIG pcfg_unc = NULL;
	DISPATCH dispatch_unc = NULL;

	SEP_DRV_LOG_TRACE_IN("");

	switch_grp = *((U32 *)param);

	for (i = num_core_devs; i < num_devices; i++) {
		pcfg_unc = (DEV_UNC_CONFIG)LWPMU_DEVICE_pcfg(&devices[i]);
		dispatch_unc = LWPMU_DEVICE_dispatch(&devices[i]);

		if (pcfg_unc && dispatch_unc && dispatch_unc->freeze) {
			SEP_DRV_LOG_TRACE("LWP: calling UNC Pause.");
			if (switch_grp) {
				if (LWPMU_DEVICE_em_groups_count(&devices[i]) >
				    1) {
					dispatch_unc->freeze(&i);
				}
			} else {
				dispatch_unc->freeze(&i);
			}
		}
	}

	SEP_DRV_LOG_TRACE_OUT("");
}

#if !defined(DRV_SEP_ACRN_ON)
/* ------------------------------------------------------------------------- */
/*!
 * @fn static OS_STATUS lwpmudrv_Pause_Op(void)
 *
 * @param - none
 *
 * @return OS_STATUS
 *
 * @brief Pause the core/uncore collection
 *
 * <I>Special Notes</I>
 */
static VOID lwpmudrv_Pause_Op(PVOID param)
{
	U32 dev_idx;
	DISPATCH dispatch;
	U32 switch_grp = 0;
	U32 this_cpu = CONTROL_THIS_CPU();

	dev_idx = core_to_dev_map[this_cpu];
	dispatch = LWPMU_DEVICE_dispatch(&devices[dev_idx]);

	SEP_DRV_LOG_TRACE_IN("");

	if (dispatch != NULL && dispatch->freeze != NULL &&
	    DRV_CONFIG_use_pcl(drv_cfg) == FALSE) {
		dispatch->freeze(param);
	}

	lwpmudrv_Pause_Uncore((PVOID)&switch_grp);

	SEP_DRV_LOG_TRACE_OUT("");
}
#endif

/* ------------------------------------------------------------------------- */
/*!
 * @fn static OS_STATUS lwpmudrv_Pause(void)
 *
 * @param - none
 *
 * @return OS_STATUS
 *
 * @brief Pause the collection
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Pause(void)
{
	int i;
	int done = FALSE;

	SEP_DRV_LOG_FLOW_IN("");

	if (!pcb || !drv_cfg) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Pcb or drv_cfg pointer is NULL!");
		return OS_INVALID;
	}

	if (CHANGE_DRIVER_STATE(STATE_BIT_RUNNING, DRV_STATE_PAUSING)) {
		if (DRV_CONFIG_use_pcl(drv_cfg) == FALSE) {
			for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state);
			     i++) {
				CPU_STATE_accept_interrupt(&pcb[i]) = 0;
			}
			while (!done) {
				done = TRUE;
				for (i = 0;
				     i < GLOBAL_STATE_num_cpus(driver_state);
				     i++) {
					if (atomic_read(&CPU_STATE_in_interrupt(
						    &pcb[i]))) {
						done = FALSE;
					}
				}
			}
		}
#if !defined(DRV_SEP_ACRN_ON)
		CONTROL_Invoke_Parallel(lwpmudrv_Pause_Op, NULL);
#endif
	/*
	 * This means that the PAUSE state has been reached.
	 */
		CHANGE_DRIVER_STATE(STATE_BIT_PAUSING, DRV_STATE_PAUSED);
	}

	SEP_DRV_LOG_FLOW_OUT("Success");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn static OS_STATUS lwpmudrv_Resume_Uncore(void)
 *
 * @param - 1 if switching group, 0 otherwise
 *
 * @return OS_STATUS
 *
 * @brief Resume the uncore collection
 *
 * <I>Special Notes</I>
 */
static VOID lwpmudrv_Resume_Uncore(PVOID param)
{
	U32 i;
	U32 switch_grp;
	DEV_UNC_CONFIG pcfg_unc = NULL;
	DISPATCH dispatch_unc = NULL;

	SEP_DRV_LOG_TRACE_IN("");

	switch_grp = *((U32 *)param);

	for (i = num_core_devs; i < num_devices; i++) {
		pcfg_unc = (DEV_UNC_CONFIG)LWPMU_DEVICE_pcfg(&devices[i]);
		dispatch_unc = LWPMU_DEVICE_dispatch(&devices[i]);

		if (pcfg_unc && dispatch_unc && dispatch_unc->restart) {
			SEP_DRV_LOG_TRACE("LWP: calling UNC Resume.");
			if (switch_grp) {
				if (LWPMU_DEVICE_em_groups_count(&devices[i]) >
				    1) {
					dispatch_unc->restart(&i);
				}
			} else {
				dispatch_unc->restart(&i);
			}
		}
	}

	SEP_DRV_LOG_TRACE_OUT("");
}

#if !defined(DRV_SEP_ACRN_ON)
/* ------------------------------------------------------------------------- */
/*!
 * @fn static OS_STATUS lwpmudrv_Resume_Op(void)
 *
 * @param - none
 *
 * @return OS_STATUS
 *
 * @brief Resume the core/uncore collection
 *
 * <I>Special Notes</I>
 */
static VOID lwpmudrv_Resume_Op(PVOID param)
{
	U32 this_cpu = CONTROL_THIS_CPU();
	U32 dev_idx = core_to_dev_map[this_cpu];
	DISPATCH dispatch = LWPMU_DEVICE_dispatch(&devices[dev_idx]);
	U32 switch_grp = 0;

	SEP_DRV_LOG_TRACE_IN("");

	if (dispatch != NULL && dispatch->restart != NULL &&
	    DRV_CONFIG_use_pcl(drv_cfg) == FALSE) {
		dispatch->restart((VOID *)(size_t)0);
	}

	lwpmudrv_Resume_Uncore((PVOID)&switch_grp);

	SEP_DRV_LOG_TRACE_OUT("");
}
#endif

/* ------------------------------------------------------------------------- */
/*!
 * @fn static OS_STATUS lwpmudrv_Resume(void)
 *
 * @param - none
 *
 * @return OS_STATUS
 *
 * @brief Resume the collection
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Resume(void)
{
	int i;

	SEP_DRV_LOG_FLOW_IN("");

	if (!pcb || !drv_cfg) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Pcb or drv_cfg pointer is NULL!");
		return OS_INVALID;
	}

	/*
	 * If we are in the process of pausing sampling, wait until the pause has been
	 * completed.  Then start the Resume process.
	 */
	while (GET_DRIVER_STATE() == DRV_STATE_PAUSING) {
		/*
	 *  This delay probably needs to be expanded a little bit more for large systems.
	 *  For now, it is probably sufficient.
	 */
		SYS_IO_Delay();
		SYS_IO_Delay();
	}

	if (CHANGE_DRIVER_STATE(STATE_BIT_PAUSED, DRV_STATE_RUNNING)) {
		for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
			if (cpu_mask_bits) {
				CPU_STATE_accept_interrupt(&pcb[i]) =
					cpu_mask_bits[i] ? 1 : 0;
				CPU_STATE_group_swap(&pcb[i]) = 1;
			} else {
				CPU_STATE_accept_interrupt(&pcb[i]) = 1;
				CPU_STATE_group_swap(&pcb[i]) = 1;
			}
		}
#if !defined(DRV_SEP_ACRN_ON)
		CONTROL_Invoke_Parallel(lwpmudrv_Resume_Op, NULL);
#endif
	}

	SEP_DRV_LOG_FLOW_OUT("Success");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn static OS_STATUS lwpmudrv_Write_Uncore(void)
 *
 * @param - 1 if switching group, 0 otherwise
 *
 * @return OS_STATUS
 *
 * @brief Program the uncore collection
 *
 * <I>Special Notes</I>
 */
static VOID lwpmudrv_Write_Uncore(PVOID param)
{
	U32 i;
	U32 switch_grp;
	DEV_UNC_CONFIG pcfg_unc = NULL;
	DISPATCH dispatch_unc = NULL;

	SEP_DRV_LOG_TRACE_IN("");

	switch_grp = *((U32 *)param);

	for (i = num_core_devs; i < num_devices; i++) {
		pcfg_unc = (DEV_UNC_CONFIG)LWPMU_DEVICE_pcfg(&devices[i]);
		dispatch_unc = LWPMU_DEVICE_dispatch(&devices[i]);

		if (pcfg_unc && dispatch_unc && dispatch_unc->write) {
			SEP_DRV_LOG_TRACE("LWP: calling UNC Write.");
			if (switch_grp) {
				if (LWPMU_DEVICE_em_groups_count(&devices[i]) >
				    1) {
					dispatch_unc->write(&i);
				}
			} else {
				dispatch_unc->write(&i);
			}
		}
	}

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn static OS_STATUS lwpmudrv_Write_Op(void)
 *
 * @param - Do operation for Core only
 *
 * @return OS_STATUS
 *
 * @brief Program the core/uncore collection
 *
 * <I>Special Notes</I>
 */
static VOID lwpmudrv_Write_Op(PVOID param)
{
	U32 this_cpu;
	U32 dev_idx;
	DISPATCH dispatch;
	U32 switch_grp = 0;

	SEP_DRV_LOG_TRACE_IN("");

	this_cpu = CONTROL_THIS_CPU();
	dev_idx = core_to_dev_map[this_cpu];
	dispatch = LWPMU_DEVICE_dispatch(&devices[dev_idx]);

	if (dispatch != NULL && dispatch->write != NULL) {
		dispatch->write((VOID *)(size_t)0);
	}

	if (param == NULL) {
		lwpmudrv_Write_Uncore((PVOID)&switch_grp);
	}

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwpmudrv_Switch_Group(void)
 *
 * @param none
 *
 * @return OS_STATUS
 *
 * @brief Switch the current group that is being collected.
 *
 * <I>Special Notes</I>
 *     This routine is called from the user mode code to handle the multiple group
 *     situation.  4 distinct steps are taken:
 *     Step 1: Pause the sampling
 *     Step 2: Increment the current group count
 *     Step 3: Write the new group to the PMU
 *     Step 4: Resume sampling
 */
static OS_STATUS lwpmudrv_Switch_Group(void)
{
	S32 idx;
	CPU_STATE pcpu;
	EVENT_CONFIG ec;
	OS_STATUS status = OS_SUCCESS;
	U32 current_state = GET_DRIVER_STATE();

	SEP_DRV_LOG_FLOW_IN("");

	if (!pcb || !drv_cfg) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Pcb or drv_cfg pointer is NULL!");
		return OS_INVALID;
	}

	if (current_state != DRV_STATE_RUNNING &&
	    current_state != DRV_STATE_PAUSED) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Return value: %d (invalid driver state!).", status);
		return status;
	}

	status = lwpmudrv_Pause();

	for (idx = 0; idx < GLOBAL_STATE_num_cpus(driver_state); idx++) {
		pcpu = &pcb[idx];
		ec = (EVENT_CONFIG)LWPMU_DEVICE_ec(
			&devices[core_to_dev_map[idx]]);
		CPU_STATE_current_group(pcpu)++;
		// make the event group list circular
		CPU_STATE_current_group(pcpu) %= EVENT_CONFIG_num_groups(ec);
	}
#if !defined(DRV_SEP_ACRN_ON)
	CONTROL_Invoke_Parallel(lwpmudrv_Write_Op,
				(VOID *)(size_t)CONTROL_THIS_CPU());
#else
	lwpmudrv_Write_Op((VOID *)(size_t)CONTROL_THIS_CPU());
#endif
	if (drv_cfg && DRV_CONFIG_start_paused(drv_cfg) == FALSE) {
		lwpmudrv_Resume();
	}

	SEP_DRV_LOG_FLOW_OUT("Return value: %d", status);
	return status;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn static OS_STATUS lwpmudrv_Trigger_Read_Op(void)
 *
 * @param - none
 *
 * @return OS_STATUS
 *
 * @brief Read uncore data
 *
 * <I>Special Notes</I>
 */
static VOID lwpmudrv_Trigger_Read_Op(PVOID param)
{
	DEV_UNC_CONFIG pcfg_unc = NULL;
	DISPATCH dispatch_unc = NULL;
	U32 this_cpu;
	CPU_STATE pcpu;
	U32 package_num;
	U64 tsc;
	BUFFER_DESC bd;
	EVENT_DESC evt_desc;
	U32 cur_grp;
	ECB pecb;
	U32 sample_size = 0;
	U32 offset = 0;
	PVOID buf;
	UncoreSampleRecordPC *psamp;
	U32 i;

	SEP_DRV_LOG_TRACE_IN("");

	this_cpu = CONTROL_THIS_CPU();
	pcpu = &pcb[this_cpu];
	package_num = core_to_package_map[this_cpu];

	if (!DRIVER_STATE_IN(GET_DRIVER_STATE(),
			     STATE_BIT_RUNNING | STATE_BIT_PAUSED)) {
		SEP_DRV_LOG_ERROR_TRACE_OUT("State is not RUNNING or PAUSED!");
		return;
	}

	if (!CPU_STATE_socket_master(pcpu)) {
		SEP_DRV_LOG_TRACE_OUT("Not socket master.");
		return;
	}

	UTILITY_Read_TSC(&tsc);
	bd = &unc_buf[package_num];

	for (i = num_core_devs; i < num_devices; i++) {
		pcfg_unc = (DEV_UNC_CONFIG)LWPMU_DEVICE_pcfg(&devices[i]);
		if (pcfg_unc) {
			cur_grp = LWPMU_DEVICE_cur_group(
				&devices[i])[package_num];
			pecb = LWPMU_DEVICE_PMU_register_data(
				&devices[i])[cur_grp];
			evt_desc = desc_data[ECB_descriptor_id(pecb)];
			sample_size += EVENT_DESC_sample_size(evt_desc);
		}
	}

	buf = OUTPUT_Reserve_Buffer_Space(bd, sample_size, FALSE,
					  !SEP_IN_NOTIFICATION, -1);

	if (buf) {
		for (i = num_core_devs; i < num_devices; i++) {
			pcfg_unc =
				(DEV_UNC_CONFIG)LWPMU_DEVICE_pcfg(&devices[i]);
			dispatch_unc = LWPMU_DEVICE_dispatch(&devices[i]);
			if (pcfg_unc && dispatch_unc &&
			    dispatch_unc->trigger_read) {
				cur_grp = LWPMU_DEVICE_cur_group(
					&devices[i])[package_num];
				pecb = LWPMU_DEVICE_PMU_register_data(
					&devices[i])[cur_grp];
				evt_desc = desc_data[ECB_descriptor_id(pecb)];

				psamp = (UncoreSampleRecordPC *)(((S8 *)buf) +
								 offset);
				UNCORE_SAMPLE_RECORD_descriptor_id(psamp) =
					ECB_descriptor_id(pecb);
				UNCORE_SAMPLE_RECORD_tsc(psamp) = tsc;
				UNCORE_SAMPLE_RECORD_uncore_valid(psamp) = 1;
				UNCORE_SAMPLE_RECORD_cpu_num(psamp) =
					(U16)this_cpu;
				UNCORE_SAMPLE_RECORD_pkg_num(psamp) =
					(U16)package_num;

				dispatch_unc->trigger_read(psamp, i);
				offset += EVENT_DESC_sample_size(evt_desc);
			}
		}
	} else {
		SEP_DRV_LOG_WARNING(
			"Buffer space reservation failed; some samples will be dropped.");
	}

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwpmudrv_Uncore_Switch_Group(void)
 *
 * @param none
 *
 * @return OS_STATUS
 *
 * @brief Switch the current group that is being collected.
 *
 * <I>Special Notes</I>
 *     This routine is called from the user mode code to handle the multiple group
 *     situation.  4 distinct steps are taken:
 *     Step 1: Pause the sampling
 *     Step 2: Increment the current group count
 *     Step 3: Write the new group to the PMU
 *     Step 4: Resume sampling
 */
static OS_STATUS lwpmudrv_Uncore_Switch_Group(void)
{
	OS_STATUS status = OS_SUCCESS;
	U32 current_state = GET_DRIVER_STATE();
	U32 i = 0;
	U32 j, k;
	DEV_UNC_CONFIG pcfg_unc;
	DISPATCH dispatch_unc;
	ECB ecb_unc;
	U32 cur_grp;
	U32 num_units;
	U32 switch_grp = 1;

	SEP_DRV_LOG_FLOW_IN("");

	if (!devices || !drv_cfg) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Devices or drv_cfg pointer is NULL!");
		return OS_INVALID;
	}

	if (current_state != DRV_STATE_RUNNING &&
	    current_state != DRV_STATE_PAUSED) {
		SEP_DRV_LOG_FLOW_OUT("Driver state is not RUNNING or PAUSED!");
		return OS_INVALID;
	}

	if (max_groups_unc > 1) {
		CONTROL_Invoke_Parallel(lwpmudrv_Pause_Uncore,
					(PVOID)&switch_grp);
		for (i = num_core_devs; i < num_devices; i++) {
			pcfg_unc = LWPMU_DEVICE_pcfg(&devices[i]);
			dispatch_unc = LWPMU_DEVICE_dispatch(&devices[i]);
			num_units = LWPMU_DEVICE_num_units(&devices[i]);
			if (!pcfg_unc || !dispatch_unc) {
				continue;
			}
			if (LWPMU_DEVICE_em_groups_count(&devices[i]) > 1) {
				for (j = 0; j < num_packages; j++) {
					cur_grp = LWPMU_DEVICE_cur_group(
						&devices[i])[j];
					ecb_unc =
						LWPMU_DEVICE_PMU_register_data(
							&devices[i])[cur_grp];
					// Switch group
					LWPMU_DEVICE_cur_group(
						&devices[i])[j]++;
					LWPMU_DEVICE_cur_group(
						&devices[i])[j] %=
						LWPMU_DEVICE_em_groups_count(
							&devices[i]);
					if (ecb_unc &&
					    (ECB_device_type(ecb_unc) ==
					     DEVICE_UNC_SOCPERF) &&
					    (j == 0)) {
						SOCPERF_Switch_Group3();
					}
					// Post group switch
					cur_grp = LWPMU_DEVICE_cur_group(
						&devices[i])[j];
					ecb_unc =
						LWPMU_DEVICE_PMU_register_data(
							&devices[i])[cur_grp];
					for (k = 0;
					     k < (ECB_num_events(ecb_unc) *
						  num_units);
					     k++) {
						LWPMU_DEVICE_prev_value(
							&devices[i])[j][k] =
							0LL; //zero out prev_value for new collection
					}
				}
			}
		}
		CONTROL_Invoke_Parallel(lwpmudrv_Write_Uncore,
					(PVOID)&switch_grp);
		CONTROL_Invoke_Parallel(lwpmudrv_Resume_Uncore,
					(PVOID)&switch_grp);
	}

	SEP_DRV_LOG_FLOW_OUT("Return value: %d", status);
	return status;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn static VOID lwpmudrv_Trigger_Read(void)
 *
 * @param - none
 *
 * @return - OS_STATUS
 *
 * @brief Read the Counter Data.
 *
 * <I>Special Notes</I>
 */
static VOID lwpmudrv_Trigger_Read(
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0)
	struct timer_list *tl
#else
	unsigned long arg
#endif
)
{
	SEP_DRV_LOG_TRACE_IN("");

	if (GET_DRIVER_STATE() != DRV_STATE_RUNNING) {
		SEP_DRV_LOG_TRACE_OUT("Success: driver state is not RUNNING");
		return;
	}
#if defined(BUILD_CHIPSET)
	if (cs_dispatch && cs_dispatch->Trigger_Read) {
		cs_dispatch->Trigger_Read();
	}
#endif

	if (drv_cfg && DRV_CONFIG_use_pcl(drv_cfg) == TRUE) {
		SEP_DRV_LOG_TRACE_OUT("Success: Using PCL");
		return;
	}

	CONTROL_Invoke_Parallel(lwpmudrv_Trigger_Read_Op, NULL);

	uncore_em_factor++;
	if (uncore_em_factor == DRV_CONFIG_unc_em_factor(drv_cfg)) {
		SEP_DRV_LOG_TRACE("Switching Uncore Group...");
		lwpmudrv_Uncore_Switch_Group();
		uncore_em_factor = 0;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0)
	mod_timer(unc_read_timer, jiffies + unc_timer_interval);
#else
	unc_read_timer->expires = jiffies + unc_timer_interval;
	add_timer(unc_read_timer);
#endif

	SEP_DRV_LOG_TRACE_OUT("Success.");
}


#if defined(DRV_SEP_ACRN_ON)
/* ------------------------------------------------------------------------- */
/*!
 * @fn static VOID lwpmudrv_ACRN_Buffer_Read(void)
 *
 * @param - none
 *
 * @return - OS_STATUS
 *
 * @brief Read the ACRN Buffer Data.
 *
 * <I>Special Notes</I>
 */
static VOID lwpmudrv_ACRN_Buffer_Read(
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0)
	struct timer_list *tl
#else
	unsigned long arg
#endif
)
{
	S32 i;

	SEP_DRV_LOG_TRACE_IN("");

	if (GET_DRIVER_STATE() != DRV_STATE_RUNNING) {
		SEP_DRV_LOG_TRACE_OUT("Success: driver state is not RUNNING");
		return;
	}

	for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
		PMI_Buffer_Handler(&i);
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0)
	mod_timer(buffer_read_timer, jiffies + buffer_timer_interval);
#else
	buffer_read_timer->expires = jiffies + buffer_timer_interval;
	add_timer(buffer_read_timer);
#endif

	SEP_DRV_LOG_TRACE_OUT("Success.");
}
#endif


/* ------------------------------------------------------------------------- */
/*!
 * @fn static void lwmudrv_Read_Specific_TSC (PVOID param)
 *
 * @param param - pointer to the result
 *
 * @return none
 *
 * @brief  Read the tsc value in the current processor and
 * @brief  write the result into param.
 *
 * <I>Special Notes</I>
 */
static VOID lwpmudrv_Read_Specific_TSC(PVOID param)
{
	U32 this_cpu;

	SEP_DRV_LOG_TRACE_IN("");

	preempt_disable();
	this_cpu = CONTROL_THIS_CPU();
	if (this_cpu == 0) {
		UTILITY_Read_TSC((U64 *)param);
	}
	preempt_enable();

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID lwpmudrv_Uncore_Stop_Timer (void)
 *
 * @brief       Stop the uncore read timer
 *
 * @param       none
 *
 * @return      none
 *
 * <I>Special Notes:</I>
 */
static VOID lwpmudrv_Uncore_Stop_Timer(void)
{
	SEP_DRV_LOG_FLOW_IN("");

	if (unc_read_timer == NULL) {
		return;
	}

	del_timer_sync(unc_read_timer);
	unc_read_timer = CONTROL_Free_Memory(unc_read_timer);

	SEP_DRV_LOG_FLOW_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          OS_STATUS lwpmudrv_Uncore_Start_Timer (void)
 *
 * @brief       Start the uncore read timer
 *
 * @param       none
 *
 * @return      OS_STATUS
 *
 * <I>Special Notes:</I>
 */
static VOID lwpmudrv_Uncore_Start_Timer(void)
{
	SEP_DRV_LOG_FLOW_IN("");

	unc_timer_interval =
		msecs_to_jiffies(DRV_CONFIG_unc_timer_interval(drv_cfg));
	unc_read_timer = CONTROL_Allocate_Memory(sizeof(struct timer_list));
	if (unc_read_timer == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory allocation failure for unc_read_timer!");
		return;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0)
	timer_setup(unc_read_timer, lwpmudrv_Trigger_Read, 0);
	mod_timer(unc_read_timer, jiffies + unc_timer_interval);
#else
	init_timer(unc_read_timer);
	unc_read_timer->function = lwpmudrv_Trigger_Read;
	unc_read_timer->expires = jiffies + unc_timer_interval;
	add_timer(unc_read_timer);
#endif

	SEP_DRV_LOG_FLOW_OUT("");
}


#if defined(DRV_SEP_ACRN_ON)
/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID lwpmudrv_ACRN_Flush_Stop_Timer (void)
 *
 * @brief       Stop the ACRN buffer read timer
 *
 * @param       none
 *
 * @return      none
 *
 * <I>Special Notes:</I>
 */
static VOID lwpmudrv_ACRN_Flush_Stop_Timer(void)
{
	SEP_DRV_LOG_FLOW_IN("");

	if (buffer_read_timer == NULL) {
		return;
	}

	del_timer_sync(buffer_read_timer);
	buffer_read_timer = CONTROL_Free_Memory(buffer_read_timer);

	SEP_DRV_LOG_FLOW_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          OS_STATUS lwpmudrv_ACRN_Flush_Start_Timer (void)
 *
 * @brief       Start the ACRN buffer read timer
 *
 * @param       none
 *
 * @return      OS_STATUS
 *
 * <I>Special Notes:</I>
 */
static VOID lwpmudrv_ACRN_Flush_Start_Timer(void)
{
	SEP_DRV_LOG_FLOW_IN("");

	buffer_timer_interval = msecs_to_jiffies(10);
	buffer_read_timer = CONTROL_Allocate_Memory(sizeof(struct timer_list));
	if (buffer_read_timer == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory allocation failure for buffer_read_timer!");
		return;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0)
	timer_setup(buffer_read_timer, lwpmudrv_ACRN_Buffer_Read, 0);
	mod_timer(buffer_read_timer, jiffies + buffer_timer_interval);
#else
	init_timer(buffer_read_timer);
	buffer_read_timer->function = lwpmudrv_ACRN_Buffer_Read;
	buffer_read_timer->expires = jiffies + buffer_timer_interval;
	add_timer(buffer_read_timer);
#endif

	SEP_DRV_LOG_FLOW_OUT("");
}
#endif


/* ------------------------------------------------------------------------- */
/*!
 * @fn static OS_STATUS lwpmudrv_Init_Op(void)
 *
 * @param - none
 *
 * @return OS_STATUS
 *
 * @brief Initialize PMU before collection
 *
 * <I>Special Notes</I>
 */
static VOID lwpmudrv_Init_Op(PVOID param)
{
	U32 this_cpu;
	U32 dev_idx;
	DISPATCH dispatch;

	preempt_disable();
	this_cpu = CONTROL_THIS_CPU();
	preempt_enable();
	dev_idx = core_to_dev_map[this_cpu];
	dispatch = LWPMU_DEVICE_dispatch(&devices[dev_idx]);

	SEP_DRV_LOG_TRACE_IN("");

	if (dispatch != NULL && dispatch->init != NULL) {
		dispatch->init(&dev_idx);
	}

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn static OS_STATUS lwpmudrv_Init_PMU(void)
 *
 * @param - none
 *
 * @return - OS_STATUS
 *
 * @brief Initialize the PMU and the driver state in preparation for data collection.
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Init_PMU(IOCTL_ARGS args)
{
	DEV_UNC_CONFIG pcfg_unc = NULL;
	DISPATCH dispatch_unc = NULL;
	EVENT_CONFIG ec;
	U32 i;
	U32 emon_buffer_size = 0;
	OS_STATUS status = OS_SUCCESS;

	SEP_DRV_LOG_FLOW_IN("");

	if (args->len_usr_to_drv == 0 || args->buf_usr_to_drv == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Invalid arguments.");
		return OS_INVALID;
	}

	if (copy_from_user(&emon_buffer_size, (void __user *)args->buf_usr_to_drv,
			   sizeof(U32))) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Memory copy failure");
		return OS_FAULT;
	}
	prev_counter_size = emon_buffer_size;

	if (!drv_cfg) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("drv_cfg not set!");
		return OS_FAULT;
	}
	if (DRV_CONFIG_use_pcl(drv_cfg) == TRUE) {
		SEP_DRV_LOG_FLOW_OUT("Success: using PCL.");
		return OS_SUCCESS;
	}

	if (GET_DRIVER_STATE() != DRV_STATE_IDLE) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Discarded: driver state is not IDLE!");
		return OS_IN_PROGRESS;
	}

	for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
		ec = (EVENT_CONFIG)LWPMU_DEVICE_ec(
			&devices[core_to_dev_map[i]]);
		CPU_STATE_trigger_count(&pcb[i]) = EVENT_CONFIG_em_factor(ec);
		CPU_STATE_trigger_event_num(&pcb[i]) =
			EVENT_CONFIG_em_event_num(ec);
	}

	// set cur_device's total groups to max groups of all devices
	max_groups_unc = 0;
	for (i = num_core_devs; i < num_devices; i++) {
		if (max_groups_unc <
		    LWPMU_DEVICE_em_groups_count(&devices[i])) {
			max_groups_unc =
				LWPMU_DEVICE_em_groups_count(&devices[i]);
		}
	}
	// now go back and up total groups for all devices
	if (DRV_CONFIG_emon_mode(drv_cfg) == TRUE) {
		for (i = num_core_devs; i < num_devices; i++) {
			if (LWPMU_DEVICE_em_groups_count(&devices[i]) <
			    max_groups_unc) {
				LWPMU_DEVICE_em_groups_count(&devices[i]) =
					max_groups_unc;
			}
		}
	}

	// allocate save/restore space before program the PMU
	lwpmudrv_Allocate_Restore_Buffer();

	// allocate uncore read buffers for SEP
	if (unc_buf_init && !DRV_CONFIG_emon_mode(drv_cfg)) {
		lwpmudrv_Allocate_Uncore_Buffer();
	}

	// must be done after pcb is created and before PMU is first written to
#if !defined(DRV_SEP_ACRN_ON)
	CONTROL_Invoke_Parallel(lwpmudrv_Init_Op, NULL);
#else
	lwpmudrv_Init_Op(NULL);
#endif

	for (i = num_core_devs; i < num_devices; i++) {
		pcfg_unc = (DEV_UNC_CONFIG)LWPMU_DEVICE_pcfg(&devices[i]);
		dispatch_unc = LWPMU_DEVICE_dispatch(&devices[i]);
		if (pcfg_unc && dispatch_unc && dispatch_unc->init) {
			dispatch_unc->init((VOID *)&i);
		}
	}

	// Allocate PEBS buffers
	if (DRV_CONFIG_counting_mode(drv_cfg) == FALSE) {
		PEBS_Allocate();
	}

	//
	// Transfer the data into the PMU registers
	//
#if !defined(DRV_SEP_ACRN_ON)
	CONTROL_Invoke_Parallel(lwpmudrv_Write_Op, NULL);
#else
	lwpmudrv_Write_Op(NULL);
#endif

	SEP_DRV_LOG_TRACE("IOCTL_Init_PMU - finished initial Write.");

	if (DRV_CONFIG_counting_mode(drv_cfg) == TRUE ||
	    DRV_CONFIG_emon_mode(drv_cfg) == TRUE) {
		if (!read_counter_info) {
			read_counter_info =
				CONTROL_Allocate_Memory(emon_buffer_size);
			if (!read_counter_info) {
				SEP_DRV_LOG_ERROR_FLOW_OUT(
					"Memory allocation failure!");
				return OS_NO_MEM;
			}
		}
		if (!prev_counter_data) {
			prev_counter_data =
				CONTROL_Allocate_Memory(emon_buffer_size);
			if (!prev_counter_data) {
				read_counter_info =
					CONTROL_Free_Memory(read_counter_info);
				SEP_DRV_LOG_ERROR_FLOW_OUT(
					"Memory allocation failure!");
				return OS_NO_MEM;
			}
		}
		if (!emon_buffer_driver_helper) {
			// allocate size = size of EMON_BUFFER_DRIVER_HELPER_NODE + the number of entries in core_index_to_thread_offset_map, which is num of cpu
			emon_buffer_driver_helper = CONTROL_Allocate_Memory(
				sizeof(EMON_BUFFER_DRIVER_HELPER_NODE) +
				sizeof(U32) *
					GLOBAL_STATE_num_cpus(driver_state));
			if (!emon_buffer_driver_helper) {
				SEP_DRV_LOG_ERROR_FLOW_OUT(
					"Memory allocation failure!");
				return OS_NO_MEM;
			}
		}
	}

	SEP_DRV_LOG_FLOW_OUT("Return value: %d", status);
	return status;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn static void lwpmudrv_Read_MSR(pvoid param)
 *
 * @param param - pointer to the buffer to store the MSR counts
 *
 * @return none
 *
 * @brief
 * @brief  Read the U64 value at address in buf_drv_to_usr and
 * @brief  write the result into buf_usr_to_drv.
 *
 * <I>Special Notes</I>
 */
static VOID lwpmudrv_Read_MSR(PVOID param)
{
	S32 cpu_idx;
	MSR_DATA this_node;
#if !defined(DRV_SEP_ACRN_ON)
	S64 reg_num;
#else
	struct profiling_msr_ops_list *msr_list;
#endif

	SEP_DRV_LOG_TRACE_IN("");

	if (param == NULL) {
		preempt_disable();
		cpu_idx = (S32)CONTROL_THIS_CPU();
		preempt_enable();
	} else {
		cpu_idx = *(S32 *)param;
	}
#if !defined(DRV_SEP_ACRN_ON)
	this_node = &msr_data[cpu_idx];
	reg_num = MSR_DATA_addr(this_node);

	if (reg_num != 0) {
		MSR_DATA_value(this_node) =
			(U64)SYS_Read_MSR((U32)MSR_DATA_addr(this_node));
	}
#else
	msr_list = (struct profiling_msr_ops_list *)CONTROL_Allocate_Memory(
		GLOBAL_STATE_num_cpus(driver_state) *
		sizeof(struct profiling_msr_ops_list));
	memset(msr_list, 0,
	       GLOBAL_STATE_num_cpus(driver_state) *
		       sizeof(struct profiling_msr_ops_list));
	for (cpu_idx = 0; cpu_idx < GLOBAL_STATE_num_cpus(driver_state);
	     cpu_idx++) {
		this_node = &msr_data[cpu_idx];
		msr_list[cpu_idx].collector_id = COLLECTOR_SEP;
		msr_list[cpu_idx].entries[0].msr_id = MSR_DATA_addr(this_node);
		msr_list[cpu_idx].entries[0].op_type = MSR_OP_READ;
		msr_list[cpu_idx].entries[0].value = 0LL;
		msr_list[cpu_idx].num_entries = 1;
		msr_list[cpu_idx].msr_op_state = MSR_OP_REQUESTED;
	}

	BUG_ON(!virt_addr_valid(msr_list));

	acrn_hypercall2(HC_PROFILING_OPS, PROFILING_MSR_OPS,
			virt_to_phys(msr_list));

	for (cpu_idx = 0; cpu_idx < GLOBAL_STATE_num_cpus(driver_state);
	     cpu_idx++) {
		this_node = &msr_data[cpu_idx];
		MSR_DATA_value(this_node) = msr_list[cpu_idx].entries[0].value;
	}

	msr_list = CONTROL_Free_Memory(msr_list);
#endif

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn static OS_STATUS lwpmudrv_Read_MSR_All_Cores(IOCTL_ARGS arg)
 *
 * @param arg - pointer to the IOCTL_ARGS structure
 *
 * @return OS_STATUS
 *
 * @brief  Read the U64 value at address into buf_drv_to_usr and write
 * @brief  the result into buf_usr_to_drv.
 * @brief  Returns OS_SUCCESS if the read across all cores succeed,
 * @brief  otherwise OS_FAULT.
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Read_MSR_All_Cores(IOCTL_ARGS arg)
{
	U64 *val;
	S32 reg_num;
	S32 i;
	MSR_DATA node;
#if defined(DRV_SEP_ACRN_ON)
	S32 this_cpu = 0;
#endif

	SEP_DRV_LOG_FLOW_IN("");

	if ((arg->len_usr_to_drv != sizeof(U32)) ||
	    (arg->buf_usr_to_drv == NULL) || (arg->buf_drv_to_usr == NULL)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Invalid arguments!");
		return OS_FAULT;
	}

	val = (U64 *)arg->buf_drv_to_usr;
	if (val == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("NULL buf_usr_to_drv!");
		return OS_FAULT;
	}

	if (copy_from_user(&reg_num, (void __user *)arg->buf_usr_to_drv, sizeof(U32))) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Memory copy failure!");
		return OS_FAULT;
	}

	msr_data = CONTROL_Allocate_Memory(GLOBAL_STATE_num_cpus(driver_state) *
					   sizeof(MSR_DATA_NODE));
	if (!msr_data) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Memory allocation failure!");
		return OS_NO_MEM;
	}

	for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
		node = &msr_data[i];
		MSR_DATA_addr(node) = reg_num;
	}

#if !defined(DRV_SEP_ACRN_ON)
	CONTROL_Invoke_Parallel(lwpmudrv_Read_MSR, NULL);
#else
	lwpmudrv_Read_MSR(&this_cpu);
#endif

	/* copy values to arg array? */
	if (arg->len_drv_to_usr < GLOBAL_STATE_num_cpus(driver_state)) {
		msr_data = CONTROL_Free_Memory(msr_data);
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Not enough memory allocated in output buffer!");
		return OS_FAULT;
	}
	for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
		node = &msr_data[i];
		if (copy_to_user((void __user *)&val[i], (U64 *)&MSR_DATA_value(node),
				 sizeof(U64))) {
			SEP_DRV_LOG_ERROR_FLOW_OUT("Memory copy failure!");
			return OS_FAULT;
		}
	}

	msr_data = CONTROL_Free_Memory(msr_data);

	SEP_DRV_LOG_FLOW_OUT("Success");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn static void lwpmudrv_Write_MSR(pvoid iaram)
 *
 * @param param - pointer to array containing the MSR address and the value to be written
 *
 * @return none
 *
 * @brief
 * @brief  Read the U64 value at address in buf_drv_to_usr and
 * @brief  write the result into buf_usr_to_drv.
 *
 * <I>Special Notes</I>
 */
static VOID lwpmudrv_Write_MSR(PVOID param)
{
	S32 cpu_idx;
	MSR_DATA this_node;
#if !defined(DRV_SEP_ACRN_ON)
	U32 reg_num;
	U64 val;
#else
	struct profiling_msr_ops_list *msr_list;
#endif

	SEP_DRV_LOG_TRACE_IN("");

	if (param == NULL) {
		preempt_disable();
		cpu_idx = (S32)CONTROL_THIS_CPU();
		preempt_enable();
	} else {
		cpu_idx = *(S32 *)param;
	}

#if !defined(DRV_SEP_ACRN_ON)
	this_node = &msr_data[cpu_idx];
	reg_num = (U32)MSR_DATA_addr(this_node);
	val = (U64)MSR_DATA_value(this_node);
	// don't attempt to write MSR 0
	if (reg_num == 0) {
		preempt_enable();
		SEP_DRV_LOG_ERROR_TRACE_OUT("Error: tried to write MSR 0!");
		return;
	}

	SYS_Write_MSR(reg_num, val);
	preempt_enable();

#else
	msr_list = (struct profiling_msr_ops_list *)CONTROL_Allocate_Memory(
		GLOBAL_STATE_num_cpus(driver_state) *
		sizeof(struct profiling_msr_ops_list));
	memset(msr_list, 0,
	       GLOBAL_STATE_num_cpus(driver_state) *
		       sizeof(struct profiling_msr_ops_list));
	for (cpu_idx = 0; cpu_idx < GLOBAL_STATE_num_cpus(driver_state);
	     cpu_idx++) {
		this_node = &msr_data[cpu_idx];
		msr_list[cpu_idx].collector_id = COLLECTOR_SEP;
		msr_list[cpu_idx].entries[0].msr_id = MSR_DATA_addr(this_node);
		msr_list[cpu_idx].entries[0].op_type = MSR_OP_WRITE;
		msr_list[cpu_idx].entries[0].value = MSR_DATA_value(this_node);
		msr_list[cpu_idx].num_entries = 1;
		msr_list[cpu_idx].msr_op_state = MSR_OP_REQUESTED;
	}

	BUG_ON(!virt_addr_valid(msr_list));

	acrn_hypercall2(HC_PROFILING_OPS, PROFILING_MSR_OPS,
			virt_to_phys(msr_list));

	msr_list = CONTROL_Free_Memory(msr_list);
#endif

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn static OS_STATUS lwpmudrv_Write_MSR_All_Cores(IOCTL_ARGS arg)
 *
 * @param arg - pointer to the IOCTL_ARGS structure
 *
 * @return OS_STATUS
 *
 * @brief  Read the U64 value at address into buf_usr_to_drv and write
 * @brief  the result into buf_usr_to_drv.
 * @brief  Returns OS_SUCCESS if the write across all cores succeed,
 * @brief  otherwise OS_FAULT.
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Write_MSR_All_Cores(IOCTL_ARGS arg)
{
	EVENT_REG_NODE buf;
	EVENT_REG buf_usr_to_drv = &buf;
	U32 reg_num;
	U64 val;
	S32 i;
	MSR_DATA node;
#if defined(DRV_SEP_ACRN_ON)
	S32 this_cpu = 0;
#endif

	SEP_DRV_LOG_FLOW_IN("");

	if (arg->len_usr_to_drv < sizeof(EVENT_REG_NODE) ||
	    arg->buf_usr_to_drv == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Invalid arguments!");
		return OS_FAULT;
	}

	if (copy_from_user(buf_usr_to_drv, (void __user *)arg->buf_usr_to_drv,
			   sizeof(EVENT_REG_NODE))) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Memory copy failure!");
		return OS_FAULT;
	}
	reg_num = (U32)EVENT_REG_reg_id(buf_usr_to_drv, 0);
	val = (U64)EVENT_REG_reg_value(buf_usr_to_drv, 0);

	msr_data = CONTROL_Allocate_Memory(GLOBAL_STATE_num_cpus(driver_state) *
					   sizeof(MSR_DATA_NODE));
	if (!msr_data) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Memory allocation failure");
		return OS_NO_MEM;
	}

	for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
		node = &msr_data[i];
		MSR_DATA_addr(node) = reg_num;
		MSR_DATA_value(node) = val;
	}

#if !defined(DRV_SEP_ACRN_ON)
	CONTROL_Invoke_Parallel(lwpmudrv_Write_MSR, NULL);
#else
	lwpmudrv_Write_MSR(&this_cpu);
#endif

	msr_data = CONTROL_Free_Memory(msr_data);

	SEP_DRV_LOG_FLOW_OUT("Success");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn static void lwpmudrv_Read_Data_Op(PVOID param)
 *
 * @param param   - dummy
 *
 * @return void
 *
 * @brief  Read all the core/uncore data counters at one shot
 *
 * <I>Special Notes</I>
 */
static void lwpmudrv_Read_Data_Op(VOID *param)
{
	U32 this_cpu;
	DISPATCH dispatch;
	U32 dev_idx;
	DEV_UNC_CONFIG pcfg_unc;

	preempt_disable();
	this_cpu = CONTROL_THIS_CPU();
	preempt_enable();

	SEP_DRV_LOG_TRACE_IN("");

	if (devices == NULL) {
		SEP_DRV_LOG_ERROR_TRACE_OUT("Devices is null!");
		return;
	}
	dev_idx = core_to_dev_map[this_cpu];
	dispatch = LWPMU_DEVICE_dispatch(&devices[dev_idx]);
	if (dispatch != NULL && dispatch->read_data != NULL) {
		dispatch->read_data(param);
	}
	for (dev_idx = num_core_devs; dev_idx < num_devices; dev_idx++) {
		pcfg_unc = (DEV_UNC_CONFIG)LWPMU_DEVICE_pcfg(&devices[dev_idx]);
		if (pcfg_unc == NULL) {
			continue;
		}
		if (!(DRV_CONFIG_emon_mode(drv_cfg) ||
		      DRV_CONFIG_counting_mode(drv_cfg))) {
			continue;
		}
		dispatch = LWPMU_DEVICE_dispatch(&devices[dev_idx]);
		if (dispatch == NULL) {
			continue;
		}
		if (dispatch->read_data == NULL) {
			continue;
		}
		dispatch->read_data((VOID *)&dev_idx);
	}

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn static OS_STATUS lwpmudrv_Read_MSRs(IOCTL_ARG arg)
 *
 * @param arg - pointer to the IOCTL_ARGS structure
 *
 * @return OS_STATUS
 *
 * @brief  Read all the programmed data counters and accumulate them
 * @brief  into a single buffer.
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Read_MSRs(IOCTL_ARGS arg)
{
#if defined(DRV_SEP_ACRN_ON)
	S32 this_cpu = 0;
#endif

	SEP_DRV_LOG_FLOW_IN("");

	if (arg->len_drv_to_usr == 0 || arg->buf_drv_to_usr == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Invalid arguments.");
		return OS_SUCCESS;
	}
	//
	// Transfer the data in the PMU registers to the output buffer
	//
	if (!read_counter_info) {
		read_counter_info =
			CONTROL_Allocate_Memory(arg->len_drv_to_usr);
		if (!read_counter_info) {
			SEP_DRV_LOG_ERROR_FLOW_OUT("Memory allocation failure");
			return OS_NO_MEM;
		}
	}
	if (!prev_counter_data) {
		prev_counter_data =
			CONTROL_Allocate_Memory(arg->len_drv_to_usr);
		if (!prev_counter_data) {
			read_counter_info =
				CONTROL_Free_Memory(read_counter_info);
			SEP_DRV_LOG_ERROR_FLOW_OUT("Memory allocation failure");
			return OS_NO_MEM;
		}
	}
	memset(read_counter_info, 0, arg->len_drv_to_usr);

#if !defined(DRV_SEP_ACRN_ON)
	CONTROL_Invoke_Parallel(lwpmudrv_Read_Data_Op, NULL);
#else
	lwpmudrv_Read_Data_Op(&this_cpu);
#endif

	if (copy_to_user((void __user *)arg->buf_drv_to_usr, read_counter_info,
			 arg->len_drv_to_usr)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Memory copy failure");
		return OS_FAULT;
	}

	SEP_DRV_LOG_FLOW_OUT("Success");
	return OS_SUCCESS;
}

#if defined(DRV_SEP_ACRN_ON)
/* ------------------------------------------------------------------------- */
/*!
 * @fn static void lwpmudrv_Read_Metrics_Op(PVOID param)
 *
 * @param param   - dummy
 *
 * @return void
 *
 * @brief  Read metrics register IA32_PERF_METRICS to collect topdown metrics
 *         from PMU
 *
 * <I>Special Notes</I>
 */
static VOID lwpmudrv_Read_Metrics_Op(PVOID param)
{
	U32 this_cpu;
	CPU_STATE pcpu;
	U32 offset;
	U32 dev_idx;
	DEV_CONFIG pcfg;
	DISPATCH dispatch;

	SEP_DRV_LOG_TRACE_IN("Dummy param: %p.", param);

	this_cpu = CONTROL_THIS_CPU();
	pcpu = &pcb[this_cpu];
	dev_idx = core_to_dev_map[this_cpu];
	pcfg = LWPMU_DEVICE_pcfg(&devices[dev_idx]);
	dispatch = LWPMU_DEVICE_dispatch(&devices[dev_idx]);
	// The pmu metric will be append after core event at thread level (basically treat them as extra core events).
	// Move the pointer to the end of the core event for this cpu index.
	offset = EMON_BUFFER_CORE_EVENT_OFFSET(
		EMON_BUFFER_DRIVER_HELPER_core_index_to_thread_offset_map(
			emon_buffer_driver_helper)[this_cpu],
		EMON_BUFFER_DRIVER_HELPER_core_num_events(
			emon_buffer_driver_helper));

	if (!DEV_CONFIG_enable_perf_metrics(pcfg) ||
	    !DEV_CONFIG_emon_perf_metrics_offset(pcfg) ||
	    (CPU_STATE_current_group(pcpu) != 0)) {
		return;
	}

	if (dispatch != NULL && dispatch->read_metrics != NULL) {
		dispatch->read_metrics(read_counter_info + offset);
		SEP_DRV_LOG_TRACE("Data read = %llu.",
				  *(read_counter_info + offset));
	}

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn static OS_STATUS lwpmudrv_Read_Metrics(IOCTL_ARGS args)
 *
 * @param args   - pointer to IOCTL_ARGS_NODE structure
 *
 * @return OS_STATUS
 *
 * @brief  Read metrics register on all cpus and accumulate them into the output
 *         buffer
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Read_Metrics(IOCTL_ARGS args)
{
	U32 this_cpu;
	CPU_STATE pcpu;
	U32 offset;
	U64 *p_buffer;
	DEV_CONFIG pcfg;
	U32 idx;

	SEP_DRV_LOG_FLOW_IN("Args: %p.", args);

	this_cpu = CONTROL_THIS_CPU();
	pcpu = &pcb[this_cpu];
	p_buffer = (U64 *)(args->buf_drv_to_usr);

	if (args->len_drv_to_usr == 0 || args->buf_drv_to_usr == NULL) {
		SEP_DRV_LOG_FLOW_OUT("Invalid parameters!");
		return OS_SUCCESS;
	}

	if (!read_counter_info) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Read_counter_info is NULL!");
		return OS_FAULT;
	}

	CONTROL_Invoke_Parallel(lwpmudrv_Read_Metrics_Op, NULL);
	for (idx = 0; idx < num_core_devs; idx++) {
		pcfg = LWPMU_DEVICE_pcfg(&devices[idx]);
		offset = DEV_CONFIG_emon_perf_metrics_offset(pcfg);
		if (!DEV_CONFIG_enable_perf_metrics(pcfg) || !offset ||
		    (CPU_STATE_current_group(pcpu) != 0)) {
			continue;
		}
		p_buffer += offset;
		if (copy_to_user((void __user *)p_buffer, read_counter_info + offset,
				 (sizeof(U64) * num_packages *
				  GLOBAL_STATE_num_cpus(driver_state) *
				  DEV_CONFIG_num_perf_metrics(pcfg)))) {
			SEP_DRV_LOG_ERROR_FLOW_OUT(
				"Failed copy_to_user for read_counter_info!");
			return OS_FAULT;
		}
	}

	SEP_DRV_LOG_FLOW_OUT("Success.");
	return OS_SUCCESS;
}

#endif

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwpmudrv_Read_Counters_And_Switch_Group(IOCTL_ARGS arg)
 *
 * @param arg - pointer to the IOCTL_ARGS structure
 *
 * @return OS_STATUS
 *
 * @brief  Read / Store the counters and switch to the next valid group.
 *
 * <I>Special Notes</I>
 *     This routine is called from the user mode code to handle the multiple group
 *     situation.  10 distinct steps are taken:
 *     Step 1:  Save the previous cpu's tsc
 *     Step 2:  Read the current cpu's tsc
 *     Step 3:  Pause the counting PMUs
 *     Step 4:  Calculate the difference between the current and previous cpu's tsc
 *     Step 5:  Save original buffer ptr and copy cpu's tsc into the output buffer
 *              Increment the buffer position by number of CPU
 *     Step 6:  Read the currently programmed data PMUs and copy the data into the output buffer
 *              Restore the original buffer ptr.
 *     Step 7:  Write the new group to the PMU
 *     Step 8:  Write the new group to the PMU
 *     Step 9:  Read the current cpu's tsc for next collection (so read MSRs time not included in report)
 *     Step 10: Resume the counting PMUs
 */
static OS_STATUS lwpmudrv_Read_Counters_And_Switch_Group(IOCTL_ARGS arg)
{
	U64 *p_buffer = NULL;
	char *orig_r_buf_ptr = NULL;
	U64 orig_r_buf_len = 0;
	OS_STATUS status = OS_SUCCESS;
	DRV_BOOL enter_in_pause_state = 0;
	U32 i = 0;
#if !defined(CONFIG_PREEMPT_COUNT)
	U64 *tmp = NULL;
#endif

	SEP_DRV_LOG_FLOW_IN("");

	if (arg->buf_drv_to_usr == NULL || arg->len_drv_to_usr == 0) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Invalid arguments.");
		return OS_FAULT;
	}

	if (!DRIVER_STATE_IN(GET_DRIVER_STATE(),
			     STATE_BIT_RUNNING | STATE_BIT_PAUSED)) {
		SEP_DRV_LOG_FLOW_OUT(
			"'Success'/error: driver state is not RUNNING or PAUSED!");
		return OS_SUCCESS;
	}

	if (GET_DRIVER_STATE() == DRV_STATE_PAUSED) {
		enter_in_pause_state = 1;
	}

	// step 1
#if !defined(CONFIG_PREEMPT_COUNT)
	if (DRV_CONFIG_per_cpu_tsc(drv_cfg)) {
		// swap cpu_tsc and prev_cpu_tsc, so that cpu_tsc is saved in prev_cpu_tsc.
		tmp = prev_cpu_tsc;
		prev_cpu_tsc = cpu_tsc;
		cpu_tsc = tmp;
	} else
#endif
		prev_cpu_tsc[0] = cpu_tsc[0];

		// step 2
		// if per_cpu_tsc is not defined, read cpu0's tsc and save in var cpu_tsc[0]
		// if per_cpu_tsc is defined, read all cpu's tsc and save in var cpu_tsc by lwpmudrv_Fill_TSC_Info
#if !defined(CONFIG_PREEMPT_COUNT)
	if (DRV_CONFIG_per_cpu_tsc(drv_cfg)) {
		atomic_set(&read_now, GLOBAL_STATE_num_cpus(driver_state));
		init_waitqueue_head(&read_tsc_now);
		CONTROL_Invoke_Parallel(lwpmudrv_Fill_TSC_Info,
					(PVOID)(size_t)0);
	} else
#endif
		CONTROL_Invoke_Cpu(0, lwpmudrv_Read_Specific_TSC, &cpu_tsc[0]);

	// step 3
	// Counters should be frozen right after time stamped.
	if (!enter_in_pause_state) {
		status = lwpmudrv_Pause();
	}

	// step 4
	if (DRV_CONFIG_per_cpu_tsc(drv_cfg)) {
		for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
#if !defined(CONFIG_PREEMPT_COUNT)
			diff_cpu_tsc[i] = cpu_tsc[i] - prev_cpu_tsc[i];
#else
			// if CONFIG_PREEMPT_COUNT is defined, means lwpmudrv_Fill_TSC_Info can not be run.
			// return all cpu's tsc difference with cpu0's tsc difference instead
			diff_cpu_tsc[i] = cpu_tsc[0] - prev_cpu_tsc[0];
#endif
		}
	} else {
		diff_cpu_tsc[0] = cpu_tsc[0] - prev_cpu_tsc[0];
	}

	// step 5
	orig_r_buf_ptr = arg->buf_drv_to_usr;
	orig_r_buf_len = arg->len_drv_to_usr;

	if (copy_to_user((void __user *)arg->buf_drv_to_usr, diff_cpu_tsc,
			 GLOBAL_STATE_num_cpus(driver_state) * sizeof(U64))) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Memory copy failure!");
		return OS_FAULT;
	}

	p_buffer = (U64 *)(arg->buf_drv_to_usr);
	p_buffer += GLOBAL_STATE_num_cpus(driver_state);
	arg->buf_drv_to_usr = (char *)p_buffer;
	arg->len_drv_to_usr -=
		GLOBAL_STATE_num_cpus(driver_state) * sizeof(U64);

	// step 6
	status = lwpmudrv_Read_MSRs(arg);

#if defined(DRV_SEP_ACRN_ON)
	status = lwpmudrv_Read_Metrics(arg);
#endif
	arg->buf_drv_to_usr = orig_r_buf_ptr;
	arg->len_drv_to_usr = orig_r_buf_len;

	// step 7
	// for each processor, increment its current group number
	lwpmudrv_Switch_To_Next_Group();

	// step 8
#if !defined(DRV_SEP_ACRN_ON)
	CONTROL_Invoke_Parallel(lwpmudrv_Write_Op, NULL);
#else
	lwpmudrv_Write_Op(NULL);
#endif

	// step 9
	// if per_cpu_tsc is defined, read all cpu's tsc and save in cpu_tsc for next run
#if !defined(CONFIG_PREEMPT_COUNT)
	if (DRV_CONFIG_per_cpu_tsc(drv_cfg)) {
		atomic_set(&read_now, GLOBAL_STATE_num_cpus(driver_state));
		init_waitqueue_head(&read_tsc_now);
		CONTROL_Invoke_Parallel(lwpmudrv_Fill_TSC_Info,
					(PVOID)(size_t)0);
	} else
#endif
		CONTROL_Invoke_Cpu(0, lwpmudrv_Read_Specific_TSC, &cpu_tsc[0]);

	// step 10
	if (!enter_in_pause_state) {
		status = lwpmudrv_Resume();
	}

	SEP_DRV_LOG_FLOW_OUT("Return value: %d", status);
	return status;
}

/*
 * @fn  static OS_STATUS lwpmudrv_Read_And_Reset_Counters(IOCTL_ARGS arg)
 *
 * @param arg - pointer to the IOCTL_ARGS structure
 *
 * @return OS_STATUS
 *
 * @brief  Read the current value of the counters, and reset them all to 0.
 *
 * <I>Special Notes</I>
 *     This routine is called from the user mode code to handle the multiple group
 *     situation. 9 distinct steps are taken:
 *     Step 1: Save the previous cpu's tsc
 *     Step 2: Read the current cpu's tsc
 *     Step 3: Pause the counting PMUs
 *     Step 4: Calculate the difference between the current and previous cpu's tsc
 *     Step 5: Save original buffer ptr and copy cpu's tsc into the output buffer
 *             Increment the buffer position by number of CPU
 *     Step 6: Read the currently programmed data PMUs and copy the data into the output buffer
 *             Restore the original buffer ptr.
 *     Step 7: Write the new group to the PMU
 *     Step 8: Read the current cpu's tsc for next collection (so read MSRs time not included in report)
 *     Step 9: Resume the counting PMUs
 */
static OS_STATUS lwpmudrv_Read_And_Reset_Counters(IOCTL_ARGS arg)
{
	U64 *p_buffer = NULL;
	char *orig_r_buf_ptr = NULL;
	U64 orig_r_buf_len = 0;
	OS_STATUS status = OS_SUCCESS;
	DRV_BOOL enter_in_pause_state = 0;
	U32 i = 0;
#if !defined(CONFIG_PREEMPT_COUNT)
	U64 *tmp = NULL;
#endif

	SEP_DRV_LOG_FLOW_IN("");

	if (arg->buf_drv_to_usr == NULL || arg->len_drv_to_usr == 0) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Invalid arguments.");
		return OS_FAULT;
	}

	if (!DRIVER_STATE_IN(GET_DRIVER_STATE(),
			     STATE_BIT_RUNNING | STATE_BIT_PAUSED)) {
		SEP_DRV_LOG_FLOW_OUT(
			"'Success'/error: driver state is not RUNNING or PAUSED!");
		return OS_SUCCESS;
	}

	if (GET_DRIVER_STATE() == DRV_STATE_PAUSED) {
		enter_in_pause_state = 1;
	}

	// step 1
#if !defined(CONFIG_PREEMPT_COUNT)
	if (DRV_CONFIG_per_cpu_tsc(drv_cfg)) {
		// swap cpu_tsc and prev_cpu_tsc, so that cpu_tsc is saved in prev_cpu_tsc.
		tmp = prev_cpu_tsc;
		prev_cpu_tsc = cpu_tsc;
		cpu_tsc = tmp;
	} else
#endif
		prev_cpu_tsc[0] = cpu_tsc[0];

		// step 2
		// if per_cpu_tsc is not defined, read cpu0's tsc into var cpu_tsc[0]
		// if per_cpu_tsc is defined, read all cpu's tsc into var cpu_tsc by lwpmudrv_Fill_TSC_Info
#if !defined(CONFIG_PREEMPT_COUNT)
	if (DRV_CONFIG_per_cpu_tsc(drv_cfg)) {
		atomic_set(&read_now, GLOBAL_STATE_num_cpus(driver_state));
		init_waitqueue_head(&read_tsc_now);
		CONTROL_Invoke_Parallel(lwpmudrv_Fill_TSC_Info,
					(PVOID)(size_t)0);
	} else
#endif
		CONTROL_Invoke_Cpu(0, lwpmudrv_Read_Specific_TSC, &cpu_tsc[0]);

	// step 3
	// Counters should be frozen right after time stamped.
	if (!enter_in_pause_state) {
		status = lwpmudrv_Pause();
		if (status != OS_INVALID) {
			return status;
		}
	}

	// step 4
	if (DRV_CONFIG_per_cpu_tsc(drv_cfg)) {
		for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
#if !defined(CONFIG_PREEMPT_COUNT)
			diff_cpu_tsc[i] = cpu_tsc[i] - prev_cpu_tsc[i];
#else
			// if CONFIG_PREEMPT_COUNT is defined, means lwpmudrv_Fill_TSC_Info can not be run.
			// return all cpu's tsc difference with cpu0's tsc difference instead
			diff_cpu_tsc[i] = cpu_tsc[0] - prev_cpu_tsc[0];
#endif
		}
	} else {
		diff_cpu_tsc[0] = cpu_tsc[0] - prev_cpu_tsc[0];
	}

	// step 5
	orig_r_buf_ptr = arg->buf_drv_to_usr;
	orig_r_buf_len = arg->len_drv_to_usr;

	if (copy_to_user((void __user *)arg->buf_drv_to_usr, diff_cpu_tsc,
			 GLOBAL_STATE_num_cpus(driver_state) * sizeof(U64))) {
		return OS_FAULT;
	}

	p_buffer = (U64 *)(arg->buf_drv_to_usr);
	p_buffer += GLOBAL_STATE_num_cpus(driver_state);
	arg->buf_drv_to_usr = (char *)p_buffer;
	arg->len_drv_to_usr -=
		GLOBAL_STATE_num_cpus(driver_state) * sizeof(U64);

	// step 6
	status = lwpmudrv_Read_MSRs(arg);
#if defined(DRV_SEP_ACRN_ON)
	status = lwpmudrv_Read_Metrics(arg);
#endif
	arg->buf_drv_to_usr = orig_r_buf_ptr;
	arg->len_drv_to_usr = orig_r_buf_len;

	// step 7
#if !defined(DRV_SEP_ACRN_ON)
	CONTROL_Invoke_Parallel(lwpmudrv_Write_Op, NULL);
#else
	lwpmudrv_Write_Op(NULL);
#endif

	// step 8
	// if per_cpu_tsc is defined, read all cpu's tsc and save in cpu_tsc for next run
#if !defined(CONFIG_PREEMPT_COUNT)
	if (DRV_CONFIG_per_cpu_tsc(drv_cfg)) {
		atomic_set(&read_now, GLOBAL_STATE_num_cpus(driver_state));
		init_waitqueue_head(&read_tsc_now);
		CONTROL_Invoke_Parallel(lwpmudrv_Fill_TSC_Info,
					(PVOID)(size_t)0);
	} else
#endif
		CONTROL_Invoke_Cpu(0, lwpmudrv_Read_Specific_TSC, &cpu_tsc[0]);

	// step 9
	if (!enter_in_pause_state) {
		status = lwpmudrv_Resume();
		if (status !=  OS_SUCCESS)
			return status;
	}

	SEP_DRV_LOG_FLOW_OUT("Return value: %d", status);
	return status;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn static OS_STATUS lwpmudrv_Set_Num_EM_Groups(IOCTL_ARGS arg)
 *
 * @param arg - pointer to the IOCTL_ARGS structure
 *
 * @return OS_STATUS
 *
 * @brief Configure the event multiplexing group.
 *
 * <I>Special Notes</I>
 *     None
 */
static OS_STATUS lwpmudrv_Set_EM_Config(IOCTL_ARGS arg)
{
	EVENT_CONFIG ec;

	SEP_DRV_LOG_FLOW_IN("");

	if (GET_DRIVER_STATE() != DRV_STATE_IDLE) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Skipped: Driver state is not IDLE!");
		return OS_IN_PROGRESS;
	}

	if (arg->buf_usr_to_drv == NULL ||
	    arg->len_usr_to_drv != sizeof(EVENT_CONFIG_NODE)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Invalid arguments.");
		return OS_INVALID;
	}

	LWPMU_DEVICE_ec(&devices[cur_device]) =
		CONTROL_Allocate_Memory(sizeof(EVENT_CONFIG_NODE));
	if (!LWPMU_DEVICE_ec(&devices[cur_device])) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Memory allocation failure for ec!");
		return OS_NO_MEM;
	}

	if (copy_from_user(LWPMU_DEVICE_ec(&devices[cur_device]),
			   (void __user *)arg->buf_usr_to_drv, sizeof(EVENT_CONFIG_NODE))) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory copy failure (event config)!");
		return OS_FAULT;
	}

	ec = (EVENT_CONFIG)LWPMU_DEVICE_ec(&devices[cur_device]);
	LWPMU_DEVICE_PMU_register_data(&devices[cur_device]) =
		CONTROL_Allocate_Memory(EVENT_CONFIG_num_groups(ec) *
					sizeof(VOID *));
	if (!LWPMU_DEVICE_PMU_register_data(&devices[cur_device])) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory allocation failure for PMU_register_data!");
		return OS_NO_MEM;
	}

	EVENTMUX_Initialize();

	SEP_DRV_LOG_FLOW_OUT("OS_SUCCESS.");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn static OS_STATUS lwpmudrv_Set_EM_Config_UNC(IOCTL_ARGS arg)
 *
 * @param arg - pointer to the IOCTL_ARGS structure
 *
 * @return OS_STATUS
 *
 * @brief  Set the number of em groups in the global state node.
 * @brief  Also, copy the EVENT_CONFIG struct that has been passed in,
 * @brief  into a global location for now.
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Set_EM_Config_UNC(IOCTL_ARGS arg)
{
	EVENT_CONFIG ec;
	SEP_DRV_LOG_FLOW_IN("");

	if (GET_DRIVER_STATE() != DRV_STATE_IDLE) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Skipped: driver state is not IDLE!");
		return OS_IN_PROGRESS;
	}

	// allocate memory
	LWPMU_DEVICE_ec(&devices[cur_device]) =
		CONTROL_Allocate_Memory(sizeof(EVENT_CONFIG_NODE));
	if (copy_from_user(LWPMU_DEVICE_ec(&devices[cur_device]),
			   (void __user *)arg->buf_usr_to_drv, arg->len_usr_to_drv)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory copy failure for LWPMU_device_ec!");
		return OS_FAULT;
	}
	// configure num_groups from ec of the specific device
	ec = (EVENT_CONFIG)LWPMU_DEVICE_ec(&devices[cur_device]);
	SEP_DRV_LOG_TRACE("Num Groups UNCORE: %d.",
			  EVENT_CONFIG_num_groups_unc(ec));
	LWPMU_DEVICE_PMU_register_data(&devices[cur_device]) =
		CONTROL_Allocate_Memory(EVENT_CONFIG_num_groups_unc(ec) *
					sizeof(VOID *));
	if (!LWPMU_DEVICE_PMU_register_data(&devices[cur_device])) {
		LWPMU_DEVICE_ec(&devices[cur_device]) = CONTROL_Free_Memory(
			LWPMU_DEVICE_ec(&devices[cur_device]));
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory allocation failure for LWPMU_DEVICE_PMU_register_data");
		return OS_NO_MEM;
	}
	LWPMU_DEVICE_em_groups_count(&devices[cur_device]) = 0;

	SEP_DRV_LOG_FLOW_OUT("Success");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn static OS_STATUS lwpmudrv_Configure_events(IOCTL_ARGS arg)
 *
 * @param arg - pointer to the IOCTL_ARGS structure
 *
 * @return OS_STATUS
 *
 * @brief  Copies one group of events into kernel space at
 * @brief  PMU_register_data[em_groups_count].
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Configure_Events(IOCTL_ARGS arg)
{
	U32 group_id;
	ECB ecb;
	U32 em_groups_count;
	EVENT_CONFIG ec;

	SEP_DRV_LOG_FLOW_IN("");

	if (GET_DRIVER_STATE() != DRV_STATE_IDLE) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Skipped: driver state is not IDLE!");
		return OS_IN_PROGRESS;
	}

	ec = (EVENT_CONFIG)LWPMU_DEVICE_ec(&devices[cur_device]);
	em_groups_count = LWPMU_DEVICE_em_groups_count(&devices[cur_device]);

	if (em_groups_count >= EVENT_CONFIG_num_groups(ec)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Error: EM groups number exceeded initial configuration!");
		return OS_INVALID;
	}
	if (arg->buf_usr_to_drv == NULL ||
	    arg->len_usr_to_drv < sizeof(ECB_NODE)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Invalid arguments.");
		return OS_INVALID;
	}

	ecb = CONTROL_Allocate_Memory(arg->len_usr_to_drv);
	if (!ecb) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory allocation failure for ecb!");
		return OS_NO_MEM;
	}
	if (copy_from_user(ecb, (void __user *)arg->buf_usr_to_drv, arg->len_usr_to_drv)) {
		CONTROL_Free_Memory(ecb);
		SEP_DRV_LOG_ERROR_FLOW_OUT("Memory copy failure for ecb data!");
		return OS_FAULT;
	}
	group_id = ECB_group_id(ecb);

	if (group_id >= EVENT_CONFIG_num_groups(ec)) {
		CONTROL_Free_Memory(ecb);
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Group_id is larger than total number of groups!");
		return OS_INVALID;
	}

	LWPMU_DEVICE_PMU_register_data(&devices[cur_device])[group_id] = ecb;
	LWPMU_DEVICE_em_groups_count(&devices[cur_device]) = group_id + 1;

	SEP_DRV_LOG_FLOW_OUT("Success");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn static OS_STATUS lwpmudrv_Configure_events_UNC(IOCTL_ARGS arg)
 *
 * @param arg - pointer to the IOCTL_ARGS structure
 *
 * @return OS_STATUS
 *
 * @brief  Make a copy of the uncore registers that need to be programmed
 * @brief  for the next event set used for event multiplexing
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Configure_Events_UNC(IOCTL_ARGS arg)
{
	VOID **PMU_register_data_unc;
	S32 em_groups_count_unc;
	ECB ecb;
	EVENT_CONFIG ec_unc;
	DEV_UNC_CONFIG pcfg_unc;
	U32 group_id = 0;
	ECB in_ecb = NULL;

	SEP_DRV_LOG_FLOW_IN("");

	if (GET_DRIVER_STATE() != DRV_STATE_IDLE) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Skipped: driver state is not IDLE!");
		return OS_IN_PROGRESS;
	}

	em_groups_count_unc =
		LWPMU_DEVICE_em_groups_count(&devices[cur_device]);
	PMU_register_data_unc =
		LWPMU_DEVICE_PMU_register_data(&devices[cur_device]);
	ec_unc = LWPMU_DEVICE_ec(&devices[cur_device]);
	pcfg_unc = LWPMU_DEVICE_pcfg(&devices[cur_device]);

	if (pcfg_unc == NULL || ec_unc == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Pcfg_unc or ec_unc NULL!");
		return OS_INVALID;
	}

	if (em_groups_count_unc >= (S32)EVENT_CONFIG_num_groups_unc(ec_unc)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Uncore EM groups number exceeded initial configuration!");
		return OS_INVALID;
	}
	if (arg->buf_usr_to_drv == NULL ||
	    arg->len_usr_to_drv < sizeof(ECB_NODE)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Invalid arguments.");
		return OS_INVALID;
	}

	in_ecb = CONTROL_Allocate_Memory(arg->len_usr_to_drv);
	if (!in_ecb) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory allocation failure for uncore ecb!");
		return OS_NO_MEM;
	}
	if (copy_from_user(in_ecb, (void __user *)arg->buf_usr_to_drv, arg->len_usr_to_drv)) {
		CONTROL_Free_Memory(in_ecb);
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory copy failure for uncore ecb data!");
		return OS_FAULT;
	}
	group_id = ECB_group_id(in_ecb);

	if (group_id >= EVENT_CONFIG_num_groups_unc(ec_unc)) {
		CONTROL_Free_Memory(in_ecb);
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Group_id is larger than total number of groups!");
		return OS_INVALID;
	}

	PMU_register_data_unc[group_id] = in_ecb;
	// at this point, we know the number of uncore events for this device,
	// so allocate the results buffer per thread for uncore only for SEP event based uncore counting
	ecb = PMU_register_data_unc[group_id];
	if (ecb == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Encountered NULL ECB!");
		return OS_INVALID;
	}
	LWPMU_DEVICE_num_events(&devices[cur_device]) = ECB_num_events(ecb);
	LWPMU_DEVICE_em_groups_count(&devices[cur_device]) = group_id + 1;

	SEP_DRV_LOG_FLOW_OUT("Success");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwpmudrv_Set_Sample_Descriptors(IOCTL_ARGS arg)
 *
 * @param arg - pointer to the IOCTL_ARGS structure
 *
 * @return OS_STATUS
 *
 * @brief  Set the number of descriptor groups in the global state node.
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Set_Sample_Descriptors(IOCTL_ARGS arg)
{
	SEP_DRV_LOG_FLOW_IN("");

	if (GET_DRIVER_STATE() != DRV_STATE_IDLE) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Skipped: driver state is not IDLE!");
		return OS_IN_PROGRESS;
	}
	if (arg->len_usr_to_drv != sizeof(U32) || arg->buf_usr_to_drv == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Invalid arguments (Unknown size of Sample Descriptors!).");
		return OS_INVALID;
	}

	desc_count = 0;
	if (copy_from_user(&GLOBAL_STATE_num_descriptors(driver_state),
			(void __user *)arg->buf_usr_to_drv, sizeof(U32))) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Memory copy failure");
		return OS_FAULT;
	}

	desc_data = CONTROL_Allocate_Memory(
		(size_t)GLOBAL_STATE_num_descriptors(driver_state) *
		sizeof(VOID *));
	if (desc_data == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory allocation failure for desc_data!");
		return OS_NO_MEM;
	}

	SEP_DRV_LOG_FLOW_OUT("Success");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn static OS_STATUS lwpmudrv_Configure_Descriptors(IOCTL_ARGS arg)
 *
 * @param arg - pointer to the IOCTL_ARGS structure
 * @return OS_STATUS
 *
 * @brief Make a copy of the descriptors that need to be read in order
 * @brief to configure a sample record.
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Configure_Descriptors(IOCTL_ARGS arg)
{
	U32 uncopied;

	SEP_DRV_LOG_FLOW_IN("");

	if (GET_DRIVER_STATE() != DRV_STATE_IDLE) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Skipped: driver state is not IDLE!");
		return OS_IN_PROGRESS;
	}

	if (desc_count >= GLOBAL_STATE_num_descriptors(driver_state)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Descriptor groups number exceeded initial configuration!");
		return OS_INVALID;
	}

	if (arg->len_usr_to_drv == 0 || arg->buf_usr_to_drv == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Invalid arg value!");
		return OS_INVALID;
	}
	if (desc_data == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("NULL desc_data!");
		return OS_INVALID;
	}
	//
	// First things first: Make a copy of the data for global use.
	//
	desc_data[desc_count] = CONTROL_Allocate_Memory(arg->len_usr_to_drv);
	uncopied = copy_from_user(desc_data[desc_count], (void __user *)arg->buf_usr_to_drv,
				  arg->len_usr_to_drv);
	if (uncopied > 0) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Unable to copy desc_data from user!");
		return OS_NO_MEM;
	}
	SEP_DRV_LOG_TRACE("Added descriptor # %d.", desc_count);
	desc_count++;

	SEP_DRV_LOG_FLOW_OUT("Success");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn static OS_STATUS lwpmudrv_LBR_Info(IOCTL_ARGS arg)
 *
 *
 * @param arg - pointer to the IOCTL_ARGS structure
 * @return OS_STATUS
 *
 * @brief Make a copy of the LBR information that is passed in.
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_LBR_Info(IOCTL_ARGS arg)
{
	SEP_DRV_LOG_FLOW_IN("");

	if (GET_DRIVER_STATE() != DRV_STATE_IDLE) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Skipped: driver state is not IDLE!");
		return OS_IN_PROGRESS;
	}

	if (cur_pcfg == NULL || DEV_CONFIG_collect_lbrs(cur_pcfg) == FALSE) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"LBR capture has not been configured!");
		return OS_INVALID;
	}

	if (arg->len_usr_to_drv == 0 || arg->buf_usr_to_drv == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Invalid arguments!");
		return OS_INVALID;
	}

	//
	// First things first: Make a copy of the data for global use.
	//

	LWPMU_DEVICE_lbr(&devices[cur_device]) =
		CONTROL_Allocate_Memory((int)arg->len_usr_to_drv);
	if (!LWPMU_DEVICE_lbr(&devices[cur_device])) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Error: Memory allocation failure for lbr!");
		return OS_NO_MEM;
	}

	if (copy_from_user(LWPMU_DEVICE_lbr(&devices[cur_device]),
			   (void __user *)arg->buf_usr_to_drv, arg->len_usr_to_drv)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory copy failure for lbr struct!");
		return OS_FAULT;
	}

	SEP_DRV_LOG_FLOW_OUT("Success");
	return OS_SUCCESS;
}

#if !defined(DRV_SEP_ACRN_ON)
#define CR4_PCE 0x00000100 //Performance-monitoring counter enable RDPMC
/* ------------------------------------------------------------------------- */
/*!
 * @fn static void lwpmudrv_Set_CR4_PCE_Bit(PVOID param)
 *
 * @param param - dummy parameter
 *
 * @return NONE
 *
 * @brief Set CR4's PCE bit on the logical processor
 *
 * <I>Special Notes</I>
 */
static VOID lwpmudrv_Set_CR4_PCE_Bit(PVOID param)
{
	U32 this_cpu;
#if defined(DRV_IA32)
	U32 prev_CR4_value = 0;

	SEP_DRV_LOG_TRACE_IN("");

	// remember if RDPMC bit previously set
	// and then enabled it
	__asm__("movl %%cr4, %%eax\n\t"
		"movl %%eax, %0\n\t"
		"orl  %1, %%eax\n\t"
		"movl %%eax, %%cr4\n\t"
		: "=irg"(prev_CR4_value)
		: "irg"(CR4_PCE)
		: "eax");
#endif
#if defined(DRV_EM64T)
	U64 prev_CR4_value = 0;

	SEP_DRV_LOG_TRACE_IN("");

	// remember if RDPMC bit previously set
	// and then enabled it
	__asm__("movq %%cr4, %%rax\n\t"
		"movq %%rax, %0\n\t"
		"orq  %1, %%rax\n\t"
		"movq %%rax, %%cr4"
		: "=irg"(prev_CR4_value)
		: "irg"(CR4_PCE)
		: "rax");
#endif
	preempt_disable();
	this_cpu = CONTROL_THIS_CPU();
	preempt_enable();

	// if bit RDPMC bit was set before,
	// set flag for when we clear it
	if (prev_CR4_value & CR4_PCE) {
		prev_set_CR4[this_cpu] = 1;
	}

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn static void lwpmudrv_Clear_CR4_PCE_Bit(PVOID param)
 *
 * @param param - dummy parameter
 *
 * @return NONE
 *
 * @brief ClearSet CR4's PCE bit on the logical processor
 *
 * <I>Special Notes</I>
 */
static VOID lwpmudrv_Clear_CR4_PCE_Bit(PVOID param)
{
	U32 this_cpu;

	SEP_DRV_LOG_TRACE_IN("");

	preempt_disable();
	this_cpu = CONTROL_THIS_CPU();
	preempt_enable();

	// only clear the CR4 bit if it wasn't set
	// before we started
	if (prev_set_CR4 && !prev_set_CR4[this_cpu]) {
#if defined(DRV_IA32)
		__asm__("movl %%cr4, %%eax\n\t"
			"andl %0, %%eax\n\t"
			"movl %%eax, %%cr4\n"
			:
			: "irg"(~CR4_PCE)
			: "eax");
#endif
#if defined(DRV_EM64T)
		__asm__("movq %%cr4, %%rax\n\t"
			"andq %0, %%rax\n\t"
			"movq %%rax, %%cr4\n"
			:
			: "irg"(~CR4_PCE)
			: "rax");
#endif
	}

	SEP_DRV_LOG_TRACE_OUT("");
}

#endif

/* ------------------------------------------------------------------------- */
/*!
 * @fn static OS_STATUS lwpmudrv_Start(void)
 *
 * @param none
 *
 * @return OS_STATUS
 *
 * @brief  Local function that handles the LWPMU_IOCTL_START call.
 * @brief  Set up the OS hooks for process/thread/load notifications.
 * @brief  Write the initial set of MSRs.
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Start(void)
{
	OS_STATUS status = OS_SUCCESS;
#if !defined(CONFIG_PREEMPT_COUNT) && !defined(DRV_SEP_ACRN_ON)
	U32 cpu_num;
#endif
#if defined(DRV_SEP_ACRN_ON)
	struct profiling_control *control = NULL;
#endif

	SEP_DRV_LOG_FLOW_IN("");

	if (!CHANGE_DRIVER_STATE(STATE_BIT_IDLE, DRV_STATE_RUNNING)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Skipped: driver state is not IDLE!");
		return OS_IN_PROGRESS;
	}

	if (drv_cfg == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("NULL drv_cfg!");
		return OS_INVALID;
	}

	if (DRV_CONFIG_use_pcl(drv_cfg) == TRUE) {
		if (DRV_CONFIG_start_paused(drv_cfg)) {
			CHANGE_DRIVER_STATE(STATE_BIT_RUNNING,
					    DRV_STATE_PAUSED);
		}
		SEP_DRV_LOG_FLOW_OUT("[PCL enabled] Early return value: %d",
				     status);
		return status;
	}

#if !defined(DRV_SEP_ACRN_ON)
	prev_set_CR4 = CONTROL_Allocate_Memory(
		GLOBAL_STATE_num_cpus(driver_state) * sizeof(U8));
	CONTROL_Invoke_Parallel(lwpmudrv_Set_CR4_PCE_Bit, (PVOID)(size_t)0);
#endif

#if !defined(CONFIG_PREEMPT_COUNT) && !defined(DRV_SEP_ACRN_ON)
	atomic_set(&read_now, GLOBAL_STATE_num_cpus(driver_state));
	init_waitqueue_head(&read_tsc_now);
	CONTROL_Invoke_Parallel(lwpmudrv_Fill_TSC_Info, (PVOID)(size_t)0);
#endif

#if !defined(CONFIG_PREEMPT_COUNT) && !defined(DRV_SEP_ACRN_ON)
	for (cpu_num = 0; cpu_num < GLOBAL_STATE_num_cpus(driver_state);
	     cpu_num++) {
		if (CPU_STATE_offlined(&pcb[cpu_num])) {
			cpu_tsc[cpu_num] = cpu_tsc[0];
		}
	}
#else
	UTILITY_Read_TSC(&cpu_tsc[0]);
#endif

	if (DRV_CONFIG_start_paused(drv_cfg)) {
		CHANGE_DRIVER_STATE(STATE_BIT_RUNNING, DRV_STATE_PAUSED);
	} else {
#if !defined(DRV_SEP_ACRN_ON)
		CONTROL_Invoke_Parallel(lwpmudrv_Resume_Op, NULL);
#else
		control = (struct profiling_control *)CONTROL_Allocate_Memory(
			sizeof(struct profiling_control));
		if (control == NULL) {
			SEP_PRINT_ERROR(
				"lwpmudrv_Start: Unable to allocate memory\n");
			return OS_NO_MEM;
		}
		memset(control, 0, sizeof(struct profiling_control));

		BUG_ON(!virt_addr_valid(control));
		control->collector_id = COLLECTOR_SEP;

		acrn_hypercall2(HC_PROFILING_OPS, PROFILING_GET_CONTROL_SWITCH,
				virt_to_phys(control));

		SEP_DRV_LOG_TRACE("ACRN profiling collection running 0x%llx\n",
				control->switches);

		if (DRV_CONFIG_counting_mode(drv_cfg) == FALSE) {
			control->switches |= (1 << CORE_PMU_SAMPLING);
			if (DEV_CONFIG_collect_lbrs(cur_pcfg)) {
				control->switches |= (1 << LBR_PMU_SAMPLING);
			}
		} else {
			control->switches |= (1 << CORE_PMU_COUNTING);
		}

		acrn_hypercall2(HC_PROFILING_OPS, PROFILING_SET_CONTROL_SWITCH,
				virt_to_phys(control));
		control = CONTROL_Free_Memory(control);

		lwpmudrv_ACRN_Flush_Start_Timer();
#endif

#if defined(BUILD_CHIPSET)
		if (DRV_CONFIG_enable_chipset(drv_cfg) && cs_dispatch != NULL &&
		    cs_dispatch->start_chipset != NULL) {
			cs_dispatch->start_chipset();
		}
#endif

		EVENTMUX_Start();
		lwpmudrv_Dump_Tracer("start", 0);

#if defined(BUILD_GFX)
		SEP_DRV_LOG_TRACE("Enable_gfx=%d.",
				  (int)DRV_CONFIG_enable_gfx(drv_cfg));
		if (DRV_CONFIG_enable_gfx(drv_cfg)) {
			GFX_Start();
		}
#endif
		if (unc_buf_init) {
			lwpmudrv_Uncore_Start_Timer();
		}
	}

	SEP_DRV_LOG_FLOW_OUT("Return value: %d", status);
	return status;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn static OS_STATUS lwpmudrv_Cleanup_Op(void)
 *
 * @param - none
 *
 * @return OS_STATUS
 *
 * @brief Clean up registers after collection
 *
 * <I>Special Notes</I>
 */
static VOID lwpmudrv_Cleanup_Op(PVOID param)
{
	U32 this_cpu;
	U32 dev_idx;
	DISPATCH dispatch;

	preempt_disable();
	this_cpu = CONTROL_THIS_CPU();
	preempt_enable();
	dev_idx = core_to_dev_map[this_cpu];
	dispatch = LWPMU_DEVICE_dispatch(&devices[dev_idx]);

	SEP_DRV_LOG_TRACE_IN("");

	if (dispatch != NULL && dispatch->cleanup != NULL) {
		dispatch->cleanup(&dev_idx);
	}

	SEP_DRV_LOG_TRACE_OUT("");
}

/*
 * @fn lwpmudrv_Prepare_Stop();
 *
 * @param        NONE
 * @return       OS_STATUS
 *
 * @brief  Local function that handles the DRV_OPERATION_STOP call.
 * @brief  Cleans up the interrupt handler.
 */
static OS_STATUS lwpmudrv_Prepare_Stop(void)
{
	S32 i;
	S32 done = FALSE;
	S32 cpu_num;
#if defined(DRV_SEP_ACRN_ON)
	struct profiling_control *control = NULL;
#endif

	SEP_DRV_LOG_FLOW_IN("");

	if (GET_DRIVER_STATE() != DRV_STATE_TERMINATING) {
		if (!CHANGE_DRIVER_STATE(STATE_BIT_RUNNING | STATE_BIT_PAUSED,
					 DRV_STATE_PREPARE_STOP)) {
			SEP_DRV_LOG_ERROR_FLOW_OUT("Unexpected driver state.");
			return OS_INVALID;
		}
	} else {
		SEP_DRV_LOG_WARNING("Abnormal termination path.");
	}

	if (drv_cfg == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("drv_cfg is NULL!");
		return OS_INVALID;
	}

	if (DRV_CONFIG_use_pcl(drv_cfg) == TRUE) {
		SEP_DRV_LOG_FLOW_OUT("Success: using PCL");
		return OS_SUCCESS;
	}

	for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
		CPU_STATE_accept_interrupt(&pcb[i]) = 0;
	}
	while (!done) {
		done = TRUE;
		for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
			if (atomic_read(&CPU_STATE_in_interrupt(&pcb[i]))) {
				done = FALSE;
			}
		}
	}
#if !defined(DRV_SEP_ACRN_ON)
	CONTROL_Invoke_Parallel(lwpmudrv_Pause_Op, NULL);
#else
	control = (struct profiling_control *)CONTROL_Allocate_Memory(
		sizeof(struct profiling_control));
	if (control == NULL) {
		SEP_PRINT_ERROR("lwpmudrv_Start: Unable to allocate memory\n");
		return OS_NO_MEM;
	}
	memset(control, 0, sizeof(struct profiling_control));

	BUG_ON(!virt_addr_valid(control));
	control->collector_id = COLLECTOR_SEP;

	acrn_hypercall2(HC_PROFILING_OPS, PROFILING_GET_CONTROL_SWITCH,
			virt_to_phys(control));

	SEP_DRV_LOG_TRACE("ACRN profiling collection running 0x%llx\n",
			control->switches);

	if (DRV_CONFIG_counting_mode(drv_cfg) == FALSE) {
		control->switches &=
			~(1 << CORE_PMU_SAMPLING);
	} else {
		control->switches &= ~(1 << CORE_PMU_COUNTING);
	}

	acrn_hypercall2(HC_PROFILING_OPS, PROFILING_SET_CONTROL_SWITCH,
			virt_to_phys(control));
	control = CONTROL_Free_Memory(control);

	lwpmudrv_ACRN_Flush_Stop_Timer();
        SEP_DRV_LOG_TRACE("Calling final PMI_Buffer_Handler\n");

	for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
		PMI_Buffer_Handler(&i);
	}
#endif

	SEP_DRV_LOG_TRACE("Outside of all interrupts.");

#if defined(BUILD_CHIPSET)
	if (DRV_CONFIG_enable_chipset(drv_cfg) && cs_dispatch != NULL &&
	    cs_dispatch->stop_chipset != NULL) {
		cs_dispatch->stop_chipset();
	}
#endif

#if defined(BUILD_GFX)
	SEP_DRV_LOG_TRACE("Enable_gfx=%d.",
			  (int)DRV_CONFIG_enable_gfx(drv_cfg));
	if (DRV_CONFIG_enable_gfx(drv_cfg)) {
		GFX_Stop();
	}
#endif

	if (unc_buf_init) {
		lwpmudrv_Uncore_Stop_Timer();
	}

	if (drv_cfg == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("drv_cfg is NULL!");
		return OS_INVALID;
	}

	/*
	 * Clean up all the control registers
	 */
#if !defined(DRV_SEP_ACRN_ON)
	CONTROL_Invoke_Parallel(lwpmudrv_Cleanup_Op, (VOID *)NULL);
#else
	lwpmudrv_Cleanup_Op(NULL);
#endif

	SEP_DRV_LOG_TRACE("Cleanup finished.");
	lwpmudrv_Free_Restore_Buffer();

#if !defined(DRV_SEP_ACRN_ON)
	if (prev_set_CR4) {
		CONTROL_Invoke_Parallel(lwpmudrv_Clear_CR4_PCE_Bit,
					(VOID *)(size_t)0);
		prev_set_CR4 = CONTROL_Free_Memory(prev_set_CR4);
	}
#endif

#if defined(BUILD_CHIPSET)
	if (DRV_CONFIG_enable_chipset(drv_cfg) && cs_dispatch &&
	    cs_dispatch->fini_chipset) {
		cs_dispatch->fini_chipset();
	}
#endif

	for (cpu_num = 0; cpu_num < GLOBAL_STATE_num_cpus(driver_state);
	     cpu_num++) {
		SEP_DRV_LOG_TRACE(
			"# of PMU interrupts via NMI triggered on cpu%d: %u.",
			cpu_num, CPU_STATE_nmi_handled(&pcb[cpu_num]));
	}

	SEP_DRV_LOG_FLOW_OUT("Success.");
	return OS_SUCCESS;
}

/*
 * @fn lwpmudrv_Finish_Stop();
 *
 * @param  NONE
 * @return OS_STATUS
 *
 * @brief  Local function that handles the DRV_OPERATION_STOP call.
 * @brief  Cleans up the interrupt handler.
 */
static OS_STATUS lwpmudrv_Finish_Stop(void)
{
	OS_STATUS status = OS_SUCCESS;
	S32 idx, cpu;

	SEP_DRV_LOG_FLOW_IN("");

	if (GET_DRIVER_STATE() != DRV_STATE_TERMINATING) {
		if (!CHANGE_DRIVER_STATE(STATE_BIT_PREPARE_STOP,
					 DRV_STATE_STOPPED)) {
			SEP_DRV_LOG_ERROR_FLOW_OUT("Unexpected driver state!");
			return OS_FAULT;
		}
	} else {
		SEP_DRV_LOG_WARNING("Abnormal termination path.");
	}

	if (drv_cfg == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("drv_cfg is NULL!");
		return OS_INVALID;
	}

	if (DRV_CONFIG_counting_mode(drv_cfg) == FALSE) {
		if (GET_DRIVER_STATE() != DRV_STATE_TERMINATING) {
#if !defined(DRV_SEP_ACRN_ON)
			CONTROL_Invoke_Parallel(PEBS_Flush_Buffer, NULL);
#endif
			/*
		 *  Make sure that the module buffers are not deallocated and that the module flush
		 *  thread has not been terminated.
		 */
			if (GET_DRIVER_STATE() != DRV_STATE_TERMINATING) {
				status = LINUXOS_Enum_Process_Modules(TRUE);
			}
			OUTPUT_Flush();
		}
		/*
	 * Clean up the interrupt handler via the IDT
	 */
#if !defined(DRV_SEP_ACRN_ON)
		CPUMON_Remove_Cpuhooks();
		PEBS_Destroy();
#else
		for (cpu = 0; cpu < GLOBAL_STATE_num_cpus(driver_state);
		     cpu++) {
			sbuf_share_setup(cpu, ACRN_SEP, NULL);
			sbuf_free(samp_buf_per_cpu[cpu]);
		}
		samp_buf_per_cpu = CONTROL_Free_Memory(samp_buf_per_cpu);
#endif
		EVENTMUX_Destroy();
	}

	if (DRV_CONFIG_enable_cp_mode(drv_cfg)) {
		if (interrupt_counts) {
			for (cpu = 0; cpu < GLOBAL_STATE_num_cpus(driver_state);
			     cpu++) {
				for (idx = 0;
				     idx < DRV_CONFIG_num_events(drv_cfg);
				     idx++) {
					SEP_DRV_LOG_TRACE(
						"Interrupt count: CPU %d, event %d = %lld.",
						cpu, idx,
						interrupt_counts
							[cpu * DRV_CONFIG_num_events(
								       drv_cfg) +
							 idx]);
				}
			}
		}
	}

	read_counter_info = CONTROL_Free_Memory(read_counter_info);
	prev_counter_data = CONTROL_Free_Memory(prev_counter_data);
	emon_buffer_driver_helper =
		CONTROL_Free_Memory(emon_buffer_driver_helper);
	lwpmudrv_Dump_Tracer("stop", 0);

	SEP_DRV_LOG_FLOW_OUT("Return value: %d", status);
	return status;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwpmudrv_Get_Normalized_TSC(IOCTL_ARGS arg)
 *
 * @param arg - Pointer to the IOCTL structure
 *
 * @return OS_STATUS
 *
 * @brief  Return the current value of the normalized TSC.
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Get_Normalized_TSC(IOCTL_ARGS arg)
{
	U64 tsc = 0;
	U64 this_cpu = 0;
	size_t size_to_copy = sizeof(U64);

	SEP_DRV_LOG_TRACE_IN("");

	if (arg->len_drv_to_usr != size_to_copy ||
	    arg->buf_drv_to_usr == NULL) {
		SEP_DRV_LOG_ERROR_TRACE_OUT("Invalid arguments!");
		return OS_INVALID;
	}

	preempt_disable();
	UTILITY_Read_TSC(&tsc);
	this_cpu = CONTROL_THIS_CPU();
	tsc -= TSC_SKEW(CONTROL_THIS_CPU());
	preempt_enable();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	if (drv_cfg && DRV_CONFIG_use_pcl(drv_cfg) == TRUE) {
		preempt_disable();
		tsc = cpu_clock(this_cpu);
		preempt_enable();
	} else {
#endif
		tsc -= TSC_SKEW(this_cpu);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	}
#endif
	if (copy_to_user((void __user *)arg->buf_drv_to_usr, (VOID *)&tsc, size_to_copy)) {
		SEP_DRV_LOG_ERROR_TRACE_OUT("Memory copy failure!");
		return OS_FAULT;
	}
	lwpmudrv_Dump_Tracer("marker", tsc);

	SEP_DRV_LOG_TRACE_OUT("Success");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwpmudrv_Get_Num_Cores(IOCTL_ARGS arg)
 *
 * @param arg - Pointer to the IOCTL structure
 *
 * @return OS_STATUS
 *
 * @brief  Quickly return the (total) number of cpus in the system.
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Get_Num_Cores(IOCTL_ARGS arg)
{
	OS_STATUS status = OS_SUCCESS;
	S32 num = GLOBAL_STATE_num_cpus(driver_state);

	SEP_DRV_LOG_FLOW_IN("");

	if (arg->len_drv_to_usr != sizeof(S32) || arg->buf_drv_to_usr == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Error: Invalid arguments.");
		return OS_INVALID;
	}

	SEP_DRV_LOG_TRACE("Num_Cores is %d, buf_usr_to_drv is 0x%p.", num,
			  arg->buf_drv_to_usr);
	status = put_user(num, (S32 __user*)arg->buf_drv_to_usr);

	SEP_DRV_LOG_FLOW_OUT("Return value: %d", status);
	return status;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwpmudrv_Set_CPU_Mask(PVOID buf_usr_to_drv, U32 len_usr_to_drv)
 *
 * @param buf_usr_to_drv   - pointer to the CPU mask buffer
 * @param len_usr_to_drv   - size of the CPU mask buffer
 *
 * @return OS_STATUS
 *
 * @brief  process the CPU mask as requested by the user
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Set_CPU_Mask(PVOID buf_usr_to_drv,
				       size_t len_usr_to_drv)
{
	U32 cpu_count = 0;

	SEP_DRV_LOG_FLOW_IN("");

	if (GET_DRIVER_STATE() != DRV_STATE_IDLE) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Skipped: driver state is not IDLE!");
		return OS_IN_PROGRESS;
	}

	if (len_usr_to_drv == 0 || buf_usr_to_drv == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"len_usr_to_drv == 0 or buf_usr_to_drv is NULL!");
		return OS_INVALID;
	}

	cpu_mask_bits = CONTROL_Allocate_Memory((int)len_usr_to_drv);
	if (!cpu_mask_bits) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory allocation failure for cpu_mask_bits!");
		return OS_NO_MEM;
	}

	if (copy_from_user(cpu_mask_bits, (void __user *)buf_usr_to_drv,
			   (int)len_usr_to_drv)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Memory copy failure!");
		return OS_FAULT;
	}

	for (cpu_count = 0;
	     cpu_count < (U32)GLOBAL_STATE_num_cpus(driver_state);
	     cpu_count++) {
		CPU_STATE_accept_interrupt(&pcb[cpu_count]) =
			cpu_mask_bits[cpu_count] ? 1 : 0;
		CPU_STATE_initial_mask(&pcb[cpu_count]) =
			cpu_mask_bits[cpu_count] ? 1 : 0;
	}

	SEP_DRV_LOG_FLOW_OUT("Success");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwpmudrv_Get_KERNEL_CS(IOCTL_ARGS arg)
 *
 * @param arg - Pointer to the IOCTL structure
 *
 * @return OS_STATUS
 *
 * @brief  Return the value of the Kernel symbol KERNEL_CS.
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Get_KERNEL_CS(IOCTL_ARGS arg)
{
	OS_STATUS status = OS_SUCCESS;
	S32 num = __KERNEL_CS;

	SEP_DRV_LOG_FLOW_IN("");

	if (arg->len_drv_to_usr != sizeof(S32) || arg->buf_drv_to_usr == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Error: Invalid arguments.");
		return OS_INVALID;
	}

	SEP_DRV_LOG_TRACE("__KERNEL_CS is %d, buf_usr_to_drv is 0x%p.", num,
			  arg->buf_drv_to_usr);
	status = put_user(num, (S32 __user *)arg->buf_drv_to_usr);

	SEP_DRV_LOG_FLOW_OUT("Return value: %d.", status);
	return status;
}

/*
 * @fn lwpmudrv_Set_UID
 *
 * @param     IN   arg      - pointer to the output buffer
 * @return   OS_STATUS
 *
 * @brief  Receive the value of the UID of the collector process.
 */
static OS_STATUS lwpmudrv_Set_UID(IOCTL_ARGS arg)
{
	OS_STATUS status = OS_SUCCESS;

	SEP_DRV_LOG_FLOW_IN("");

	if (arg->len_usr_to_drv != sizeof(uid_t) ||
	    arg->buf_usr_to_drv == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Error: Invalid arguments.");
		return OS_INVALID;
	}

	if (GET_DRIVER_STATE() != DRV_STATE_IDLE) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Skipped: driver state is not IDLE!");
		return OS_IN_PROGRESS;
	}

	status = get_user(uid, (S32 __user *)arg->buf_usr_to_drv);
	SEP_DRV_LOG_TRACE("Uid is %d.", uid);

	SEP_DRV_LOG_FLOW_OUT("Return value: %d.", status);
	return status;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwpmudrv_Get_TSC_Skew_Info(IOCTL_ARGS arg)
 *
 * @param arg - Pointer to the IOCTL structure
 *
 * @return OS_STATUS
 * @brief  Return the current value of the TSC skew data
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Get_TSC_Skew_Info(IOCTL_ARGS arg)
{
	S64 *skew_array;
	size_t skew_array_len;
	S32 i;

	SEP_DRV_LOG_FLOW_IN("");

	skew_array_len = GLOBAL_STATE_num_cpus(driver_state) * sizeof(U64);
	if (arg->len_drv_to_usr < skew_array_len ||
	    arg->buf_drv_to_usr == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Input buffer too small or NULL!");
		return OS_INVALID;
	}

	if (!DRV_CONFIG_enable_cp_mode(drv_cfg) &&
	    GET_DRIVER_STATE() != DRV_STATE_STOPPED) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Skipped: cp_mode not enabled and driver is not STOPPED!");
		return OS_IN_PROGRESS;
	}

	SEP_DRV_LOG_TRACE("Dispatched with len_drv_to_usr=%lld.",
			  arg->len_drv_to_usr);

	skew_array = CONTROL_Allocate_Memory(skew_array_len);
	if (skew_array == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory allocation failure for skew_array!");
		return OS_NO_MEM;
	}

	for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
		skew_array[i] = TSC_SKEW(i);
	}

	if (copy_to_user((void __user *)arg->buf_drv_to_usr, skew_array, skew_array_len)) {
		skew_array = CONTROL_Free_Memory(skew_array);
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory copy failure for skew_array!");
		return OS_FAULT;
	}

	skew_array = CONTROL_Free_Memory(skew_array);

	SEP_DRV_LOG_FLOW_OUT("Success");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwpmudrv_Collect_Sys_Config(IOCTL_ARGS arg)
 *
 * @param arg - Pointer to the IOCTL structure
 *
 * @return OS_STATUS
 *
 * @brief  Local function that handles the COLLECT_SYS_CONFIG call.
 * @brief  Builds and collects the SYS_INFO data needed.
 * @brief  Writes the result into the argument.
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Collect_Sys_Config(IOCTL_ARGS arg)
{
	OS_STATUS status = OS_SUCCESS;
	U32 num;

	SEP_DRV_LOG_FLOW_IN("");

	num = SYS_INFO_Build();

	if (arg->len_drv_to_usr < sizeof(S32) || arg->buf_drv_to_usr == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Error: Invalid arguments.");
		return OS_INVALID;
	}

	SEP_DRV_LOG_TRACE("Size of sys info is %d.", num);
	status = put_user(num, (S32 __user *)arg->buf_drv_to_usr);

	SEP_DRV_LOG_FLOW_OUT("Return value: %d", status);
	return status;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwpmudrv_Sys_Config(IOCTL_ARGS arg)
 *
 * @param arg - Pointer to the IOCTL structure
 *
 * @return OS_STATUS
 *
 * @brief  Return the current value of the normalized TSC.
 *
 * @brief  Transfers the VTSA_SYS_INFO data back to the abstraction layer.
 * @brief  The buf_usr_to_drv should have enough space to handle the transfer.
 */
static OS_STATUS lwpmudrv_Sys_Config(IOCTL_ARGS arg)
{
	SEP_DRV_LOG_FLOW_IN("");

	if (arg->len_drv_to_usr == 0 || arg->buf_drv_to_usr == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Error: Invalid arguments.");
		return OS_INVALID;
	}

	SYS_INFO_Transfer(arg->buf_drv_to_usr, arg->len_drv_to_usr);

	SEP_DRV_LOG_FLOW_OUT("Success");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwpmudrv_Samp_Read_Num_Of_Core_Counters(IOCTL_ARGS arg)
 *
 * @param arg - Pointer to the IOCTL structure
 *
 * @return OS_STATUS
 *
 * @brief  Read memory mapped i/o physical location
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Samp_Read_Num_Of_Core_Counters(IOCTL_ARGS arg)
{
	U64 rax, rbx, rcx, rdx, num_basic_functions;
	U32 val = 0;
	OS_STATUS status = OS_SUCCESS;

	SEP_DRV_LOG_FLOW_IN("");

	if (arg->len_drv_to_usr == 0 || arg->buf_drv_to_usr == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Error: Invalid arguments.");
		return OS_INVALID;
	}

	UTILITY_Read_Cpuid(0x0, &num_basic_functions, &rbx, &rcx, &rdx);

	if (num_basic_functions >= 0xA) {
		UTILITY_Read_Cpuid(0xA, &rax, &rbx, &rcx, &rdx);
		val = ((U32)(rax >> 8)) & 0xFF;
	}
	status = put_user(val, (U32 __user *)arg->buf_drv_to_usr);
	SEP_DRV_LOG_TRACE("Num of counter is %d.", val);

	SEP_DRV_LOG_FLOW_OUT("Return value: %d.", status);
	return status;
}

#if defined(BUILD_CHIPSET)

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static DRV_BOOL lwpmudrv_Is_Physical_Address_Free(U32 physical_addrss)
 *
 * @param physical_address - physical address
 *
 * @return DRV_BOOL
 *
 * @brief  Check if physical address is available
 *
 * <I>Special Notes</I>
 */
static DRV_BOOL lwpmudrv_Is_Physical_Address_Free(U32 physical_address)
{
	U32 new_value;
	U32 test_value = 0;
	U32 value = 0;

	SEP_DRV_LOG_TRACE_IN("");

	if (GET_DRIVER_STATE() != DRV_STATE_IDLE) {
		SEP_DRV_LOG_WARNING_TRACE_OUT(
			"FALSE: driver state is not IDLE!");
		return FALSE;
	}
	if (physical_address == 0) {
		SEP_DRV_LOG_WARNING_TRACE_OUT("FALSE: is NULL!");
		return FALSE;
	}

	// First attempt read
	//
	PCI_Read_From_Memory_Address(physical_address, &value);

	// Value must be 0xFFFFFFFFF or there is NO chance
	// that this memory location is available.
	//
	if (value != 0xFFFFFFFF) {
		SEP_DRV_LOG_TRACE_OUT("FALSE: value is not 0xFFFFFFFF!");
		return FALSE;
	}

	//
	// Try to write a bit to a zero (this probably
	// isn't too safe, but this is just for testing)
	//
	new_value = 0xFFFFFFFE;
	PCI_Write_To_Memory_Address(physical_address, new_value);
	PCI_Read_From_Memory_Address(physical_address, &test_value);

	// Write back original
	PCI_Write_To_Memory_Address(physical_address, value);

	if (new_value == test_value) {
		// The write appeared to change the
		// memory, it must be mapped already
		//
		SEP_DRV_LOG_TRACE_OUT("FALSE: appears to be mapped already!");
		return FALSE;
	}

	if (test_value == 0xFFFFFFFF) {
		// The write did not change the bit, so
		// apparently, this memory must not be mapped
		// to anything.
		//
		SEP_DRV_LOG_TRACE_OUT("TRUE: appears not to be mapped!");
		return TRUE;
	}

	SEP_DRV_LOG_TRACE_OUT("FALSE: Odd case!");
	return FALSE;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwpmudrv_Samp_Find_Physical_Address(IOCTL_ARGS arg)
 *
 * @param arg - Pointer to the IOCTL structure
 *
 * @return OS_STATUS
 *
 * @brief  Find a free physical address
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Samp_Find_Physical_Address(IOCTL_ARGS arg)
{
	CHIPSET_PCI_SEARCH_ADDR_NODE user_addr;
	CHIPSET_PCI_SEARCH_ADDR search_addr;
	U32 addr;

	SEP_DRV_LOG_FLOW_IN("");

	search_addr = (CHIPSET_PCI_SEARCH_ADDR)arg->buf_usr_to_drv;

	if (GET_DRIVER_STATE() != DRV_STATE_IDLE) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Skipped: driver state is not IDLE.");
		return OS_IN_PROGRESS;
	}

	if (arg->len_drv_to_usr == 0 || arg->buf_drv_to_usr == NULL ||
	    arg->len_usr_to_drv == 0 || arg->buf_usr_to_drv == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Invalid arguments!");
		return OS_INVALID;
	}

	if (!search_addr) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Search_addr is NULL!");
		return OS_FAULT;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 0, 0)
	/* 'type' field has been removed */
	if (!access_ok((void __user *)search_addr,
		       sizeof(CHIPSET_PCI_SEARCH_ADDR_NODE))) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Access not OK!");
		return OS_FAULT;
	}
#else
	if (!access_ok(VERIFY_WRITE, (void __user *)search_addr,
		       sizeof(CHIPSET_PCI_SEARCH_ADDR_NODE))) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Access not OK!");
		return OS_FAULT;
	}
#endif

	if (copy_from_user(&user_addr, (void __user *)search_addr,
			   sizeof(CHIPSET_PCI_SEARCH_ADDR_NODE))) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Error: Memory copy failure for search_addr!");
		return OS_FAULT;
	}

	if (CHIPSET_PCI_SEARCH_ADDR_start(&user_addr) >
	    CHIPSET_PCI_SEARCH_ADDR_stop(&user_addr)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"SEARCH_ADDR_start > SEARCH_ADDR_stop!");
		return OS_INVALID;
	}

	CHIPSET_PCI_SEARCH_ADDR_address(&user_addr) = 0;

	for (addr = CHIPSET_PCI_SEARCH_ADDR_start(&user_addr);
	     addr <= CHIPSET_PCI_SEARCH_ADDR_stop(&user_addr);
	     addr += CHIPSET_PCI_SEARCH_ADDR_increment(&user_addr)) {
		SEP_DRV_LOG_TRACE("Addr=%x:", addr);
		if (lwpmudrv_Is_Physical_Address_Free(addr)) {
			CHIPSET_PCI_SEARCH_ADDR_address(&user_addr) = addr;
			break;
		}
	}

	if (copy_to_user((void __user *)arg->buf_drv_to_usr, (VOID *)&user_addr,
			 sizeof(CHIPSET_PCI_SEARCH_ADDR_NODE))) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Error: Memory copy failure for user_addr!");
		return OS_FAULT;
	}

	SEP_DRV_LOG_FLOW_OUT("Success");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwpmudrv_Samp_Read_PCI_Config(IOCTL_ARGS arg)
 *
 * @param arg - Pointer to the IOCTL structure
 *
 * @return OS_STATUS
 *
 * @brief  Read the PCI Configuration Space
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Samp_Read_PCI_Config(IOCTL_ARGS arg)
{
	CHIPSET_PCI_CONFIG rd_pci = NULL;

	SEP_DRV_LOG_FLOW_IN("");

	if (arg->len_drv_to_usr == 0 || arg->buf_drv_to_usr == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Invalid arguments!");
		return OS_FAULT;
	}

	rd_pci = CONTROL_Allocate_Memory(arg->len_drv_to_usr);
	if (rd_pci == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory allocation failure for rd_pci!");
		return OS_NO_MEM;
	}

	if (copy_from_user(rd_pci, (void __user *)arg->buf_usr_to_drv,
			   sizeof(CHIPSET_PCI_CONFIG_NODE))) {
		CONTROL_Free_Memory(rd_pci);
		SEP_DRV_LOG_ERROR_FLOW_OUT("Memory copy failure for rd_pci!");
		return OS_FAULT;
	}

	CHIPSET_PCI_CONFIG_value(rd_pci) =
		PCI_Read_U32(CHIPSET_PCI_CONFIG_bus(rd_pci),
			     CHIPSET_PCI_CONFIG_device(rd_pci),
			     CHIPSET_PCI_CONFIG_function(rd_pci),
			     CHIPSET_PCI_CONFIG_offset(rd_pci));

	if (copy_to_user((void __user *)arg->buf_drv_to_usr, (VOID *)rd_pci,
			 sizeof(CHIPSET_PCI_CONFIG_NODE))) {
		CONTROL_Free_Memory(rd_pci);
		SEP_DRV_LOG_ERROR_FLOW_OUT("Memory copy failure for rd_pci!");
		return OS_FAULT;
	}

	SEP_DRV_LOG_TRACE("Value at this PCI address:0x%x.",
			  CHIPSET_PCI_CONFIG_value(rd_pci));

	CONTROL_Free_Memory(rd_pci);

	SEP_DRV_LOG_FLOW_OUT("Success.");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwpmudrv_Samp_Write_PCI_Config(IOCTL_ARGS arg)
 *
 * @param arg - Pointer to the IOCTL structure
 *
 * @return OS_STATUS
 *
 * @brief  Write to the PCI Configuration Space
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Samp_Write_PCI_Config(IOCTL_ARGS arg)
{
	CHIPSET_PCI_CONFIG wr_pci = NULL;

	SEP_DRV_LOG_FLOW_IN("");

	// the following allows "sep -el -pc" to work, since the command must access the
	// the driver ioctls before driver is used for a collection

	if (!DRIVER_STATE_IN(GET_DRIVER_STATE(),
			     STATE_BIT_UNINITIALIZED | STATE_BIT_IDLE)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Skipped: Driver state is not IDLE or UNINITIALIZED!");
		return OS_IN_PROGRESS;
	}

	if (arg->len_usr_to_drv == 0 || arg->buf_usr_to_drv == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Invalid arguments!");
		return OS_INVALID;
	}

	wr_pci = CONTROL_Allocate_Memory(arg->len_usr_to_drv);
	if (wr_pci == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory allocation failure for wr_pci!");
		return OS_NO_MEM;
	}
	if (copy_from_user(wr_pci, (void __user *)arg->buf_usr_to_drv,
			   sizeof(CHIPSET_PCI_CONFIG_NODE))) {
		CONTROL_Free_Memory(wr_pci);
		SEP_DRV_LOG_ERROR_FLOW_OUT("Memory copy failure for wr_pci!");
		return OS_FAULT;
	}

	PCI_Write_U32(CHIPSET_PCI_CONFIG_bus(wr_pci),
		      CHIPSET_PCI_CONFIG_device(wr_pci),
		      CHIPSET_PCI_CONFIG_function(wr_pci),
		      CHIPSET_PCI_CONFIG_offset(wr_pci),
		      CHIPSET_PCI_CONFIG_value(wr_pci));

	CONTROL_Free_Memory(wr_pci);

	SEP_DRV_LOG_FLOW_OUT("Success");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwpmudrv_Samp_Chipset_Init(IOCTL_ARGS arg)
 *
 * @param arg - Pointer to the IOCTL structure
 *
 * @return OS_STATUS
 *
 * @brief  Initialize the chipset cnfiguration
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Samp_Chipset_Init(IOCTL_ARGS arg)
{
	PVOID buf_usr_to_drv;
	U32 len_usr_to_drv;

	SEP_DRV_LOG_FLOW_IN("");

	buf_usr_to_drv = arg->buf_usr_to_drv;
	len_usr_to_drv = arg->len_usr_to_drv;

	if (GET_DRIVER_STATE() != DRV_STATE_IDLE) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Skipped: driver state is not IDLE!");
		return OS_IN_PROGRESS;
	}

	if (buf_usr_to_drv == NULL || len_usr_to_drv == 0) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Error: Invalid arguments!");
		return OS_INVALID;
	}

	// First things first: Make a copy of the data for global use.
	pma = CONTROL_Allocate_Memory(len_usr_to_drv);

	if (pma == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory allocation failure for pma!");
		return OS_NO_MEM;
	}

	if (copy_from_user(pma, (void __user *)buf_usr_to_drv, len_usr_to_drv)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Memory copy failure for pma!");
		return OS_FAULT;
	}

#if defined(MY_DEBUG)

	SEP_DRV_LOG_TRACE("Chipset Configuration follows...");
	SEP_DRV_LOG_TRACE("pma->length=%d.", CHIPSET_CONFIG_length(pma));
	SEP_DRV_LOG_TRACE("pma->version=%d.",
			  CHIPSET_CONFIG_major_version(pma));
	SEP_DRV_LOG_TRACE("pma->processor=%d.", CHIPSET_CONFIG_processor(pma));
	SEP_DRV_LOG_TRACE("pma->mch_chipset=%d.",
			  CHIPSET_CONFIG_mch_chipset(pma));
	SEP_DRV_LOG_TRACE("pma->ich_chipset=%d.",
			  CHIPSET_CONFIG_ich_chipset(pma));
	SEP_DRV_LOG_TRACE("pma->gmch_chipset=%d.",
			  CHIPSET_CONFIG_gmch_chipset(pma));
	SEP_DRV_LOG_TRACE("pma->mother_board_time=%d.",
			  CHIPSET_CONFIG_motherboard_time(pma));
	SEP_DRV_LOG_TRACE("pma->host_proc_run=%d.",
			  CHIPSET_CONFIG_host_proc_run(pma));
	SEP_DRV_LOG_TRACE("pma->noa_chipset=%d.",
			  CHIPSET_CONFIG_noa_chipset(pma));
	SEP_DRV_LOG_TRACE("pma->bnb_chipset=%d.",
			  CHIPSET_CONFIG_bnb_chipset(pma));

	if (CHIPSET_CONFIG_mch_chipset(pma)) {
		SEP_DRV_LOG_TRACE("pma->mch->phys_add=0x%llx.",
				  CHIPSET_SEGMENT_physical_address(
					  &CHIPSET_CONFIG_mch(pma)));
		SEP_DRV_LOG_TRACE(
			"pma->mch->size=%d.",
			CHIPSET_SEGMENT_size(&CHIPSET_CONFIG_mch(pma)));
		SEP_DRV_LOG_TRACE(
			"pma->mch->num_counters=%d.",
			CHIPSET_SEGMENT_num_counters(&CHIPSET_CONFIG_mch(pma)));
		SEP_DRV_LOG_TRACE(
			"pma->mch->total_events=%d.",
			CHIPSET_SEGMENT_total_events(&CHIPSET_CONFIG_mch(pma)));
	}

	if (CHIPSET_CONFIG_ich_chipset(pma)) {
		SEP_DRV_LOG_TRACE("pma->ich->phys_add=0x%llx.",
				  CHIPSET_SEGMENT_physical_address(
					  &CHIPSET_CONFIG_ich(pma)));
		SEP_DRV_LOG_TRACE(
			"pma->ich->size=%d.",
			CHIPSET_SEGMENT_size(&CHIPSET_CONFIG_ich(pma)));
		SEP_DRV_LOG_TRACE(
			"pma->ich->num_counters=%d.",
			CHIPSET_SEGMENT_num_counters(&CHIPSET_CONFIG_ich(pma)));
		SEP_DRV_LOG_TRACE(
			"pma->ich->total_events=%d.",
			CHIPSET_SEGMENT_total_events(&CHIPSET_CONFIG_ich(pma)));
	}

	if (CHIPSET_CONFIG_gmch_chipset(pma)) {
		SEP_DRV_LOG_TRACE("pma->gmch->phys_add=0x%llx.",
				  CHIPSET_SEGMENT_physical_address(
					  &CHIPSET_CONFIG_gmch(pma)));
		SEP_DRV_LOG_TRACE(
			"pma->gmch->size=%d.",
			CHIPSET_SEGMENT_size(&CHIPSET_CONFIG_gmch(pma)));
		SEP_DRV_LOG_TRACE("pma->gmch->num_counters=%d.",
				  CHIPSET_SEGMENT_num_counters(
					  &CHIPSET_CONFIG_gmch(pma)));
		SEP_DRV_LOG_TRACE("pma->gmch->total_events=%d.",
				  CHIPSET_SEGMENT_total_events(
					  &CHIPSET_CONFIG_gmch(pma)));
		SEP_DRV_LOG_TRACE("pma->gmch->read_register=0x%x.",
				  CHIPSET_SEGMENT_read_register(
					  &CHIPSET_CONFIG_gmch(pma)));
		SEP_DRV_LOG_TRACE("pma->gmch->write_register=0x%x.",
				  CHIPSET_SEGMENT_write_register(
					  &CHIPSET_CONFIG_gmch(pma)));
	}

#endif

	// Set up the global cs_dispatch table
	cs_dispatch = UTILITY_Configure_Chipset();
	if (cs_dispatch == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Unknown chipset family!");
		return OS_INVALID;
	}

	// Initialize chipset configuration
	if (cs_dispatch->init_chipset()) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Failed to initialize the chipset!");
		return OS_INVALID;
	}
	SEP_DRV_LOG_FLOW_OUT("Success");
	return OS_SUCCESS;
}

#endif

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwpmudrv_Get_Platform_Info(IOCTL_ARGS arg)
 *
 * @param arg - Pointer to the IOCTL structure
 *
 * @return OS_STATUS
 *
 * @brief       Reads the MSR_PLATFORM_INFO register if present
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Get_Platform_Info(IOCTL_ARGS args)
{
	U32 size = sizeof(DRV_PLATFORM_INFO_NODE);
	OS_STATUS status = OS_SUCCESS;
	DRV_PLATFORM_INFO platform_data = NULL;
	U32 *dispatch_ids = NULL;
	DISPATCH dispatch_ptr = NULL;
	U32 i = 0;
	U32 num_entries; // # dispatch ids to process

	SEP_DRV_LOG_FLOW_IN("");

	num_entries =
		args->len_usr_to_drv / sizeof(U32); // # dispatch ids to process

	platform_data = CONTROL_Allocate_Memory(sizeof(DRV_PLATFORM_INFO_NODE));
	if (!platform_data) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory allocation failure for platform_data!");
		return OS_NO_MEM;
	}

	memset(platform_data, 0, sizeof(DRV_PLATFORM_INFO_NODE));
	if (args->len_usr_to_drv > 0 && args->buf_usr_to_drv != NULL) {
		dispatch_ids = CONTROL_Allocate_Memory(args->len_usr_to_drv);
		if (!dispatch_ids) {
			platform_data = CONTROL_Free_Memory(platform_data);
			SEP_DRV_LOG_ERROR_FLOW_OUT(
				"Memory allocation failure for dispatch_ids!");
			return OS_NO_MEM;
		}

		status = copy_from_user(dispatch_ids, (void __user *)args->buf_usr_to_drv,
					args->len_usr_to_drv);
		if (status) {
			platform_data = CONTROL_Free_Memory(platform_data);
			dispatch_ids = CONTROL_Free_Memory(dispatch_ids);
			SEP_DRV_LOG_ERROR_FLOW_OUT(
				"Memory copy failure for dispatch_ids!");
			return status;
		}
		for (i = 0; i < num_entries; i++) {
			if (dispatch_ids[i] > 0) {
				dispatch_ptr =
					UTILITY_Configure_CPU(dispatch_ids[i]);
				if (dispatch_ptr &&
				    dispatch_ptr->platform_info) {
					dispatch_ptr->platform_info(
						(PVOID)platform_data);
				}
			}
		}
		dispatch_ids = CONTROL_Free_Memory(dispatch_ids);
	} else if (devices) {
		dispatch_ptr = LWPMU_DEVICE_dispatch(
			&devices[0]); //placeholder, needs to be fixed
		if (dispatch_ptr && dispatch_ptr->platform_info) {
			dispatch_ptr->platform_info((PVOID)platform_data);
		}
	}

	if (args->len_drv_to_usr < size || args->buf_drv_to_usr == NULL) {
		platform_data = CONTROL_Free_Memory(platform_data);
		SEP_DRV_LOG_ERROR_FLOW_OUT("Error: Invalid arguments!");
		return OS_FAULT;
	}

	status = copy_to_user((void __user *)args->buf_drv_to_usr, platform_data, size);
	platform_data = CONTROL_Free_Memory(platform_data);

	SEP_DRV_LOG_FLOW_OUT("Return value: %d", status);
	return status;
}
/* ------------------------------------------------------------------------- */
/*!
 * @fn          void lwpmudrv_Setup_Cpu_Topology (value)
 *
 * @brief       Sets up the per CPU state structures
 *
 * @param       IOCTL_ARGS args
 *
 * @return      OS_STATUS
 *
 * <I>Special Notes:</I>
 *              This function was added to support abstract dll creation.
 */
static OS_STATUS lwpmudrv_Setup_Cpu_Topology(IOCTL_ARGS args)
{
	S32 cpu_num;
	S32 iter;
	DRV_TOPOLOGY_INFO drv_topology, dt;

	SEP_DRV_LOG_FLOW_IN("");

	if (GET_DRIVER_STATE() != DRV_STATE_IDLE) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Skipped: driver state is not IDLE!");
		return OS_IN_PROGRESS;
	}
	if (args->len_usr_to_drv == 0 || args->buf_usr_to_drv == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Topology information has been misconfigured!");
		return OS_INVALID;
	}

	drv_topology = CONTROL_Allocate_Memory(args->len_usr_to_drv);
	if (drv_topology == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory allocation failure for drv_topology!");
		return OS_NO_MEM;
	}

	if (copy_from_user(drv_topology,
			   (void __user *)(args->buf_usr_to_drv),
			   args->len_usr_to_drv)) {
		drv_topology = CONTROL_Free_Memory(drv_topology);
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory copy failure for drv_topology!");
		return OS_FAULT;
	}
	/*
	 *   Topology Initializations
	 */
	num_packages = 0;
	for (iter = 0; iter < GLOBAL_STATE_num_cpus(driver_state); iter++) {
		dt = &drv_topology[iter];
		cpu_num = DRV_TOPOLOGY_INFO_cpu_number(dt);
		CPU_STATE_socket_master(&pcb[cpu_num]) =
			DRV_TOPOLOGY_INFO_socket_master(dt);
		num_packages += CPU_STATE_socket_master(&pcb[cpu_num]);
		CPU_STATE_core_master(&pcb[cpu_num]) =
			DRV_TOPOLOGY_INFO_core_master(dt);
		CPU_STATE_thr_master(&pcb[cpu_num]) =
			DRV_TOPOLOGY_INFO_thr_master(dt);
		CPU_STATE_core_type(&pcb[cpu_num]) =
			DRV_TOPOLOGY_INFO_cpu_core_type(dt);
		CPU_STATE_cpu_module_num(&pcb[cpu_num]) =
			(U16)DRV_TOPOLOGY_INFO_cpu_module_num(
				&drv_topology[iter]);
		CPU_STATE_cpu_module_master(&pcb[cpu_num]) =
			(U16)DRV_TOPOLOGY_INFO_cpu_module_master(
				&drv_topology[iter]);
		CPU_STATE_system_master(&pcb[cpu_num]) = (iter) ? 0 : 1;
		SEP_DRV_LOG_TRACE("Cpu %d sm = %d cm = %d tm = %d.", cpu_num,
				  CPU_STATE_socket_master(&pcb[cpu_num]),
				  CPU_STATE_core_master(&pcb[cpu_num]),
				  CPU_STATE_thr_master(&pcb[cpu_num]));
	}
	drv_topology = CONTROL_Free_Memory(drv_topology);

	SEP_DRV_LOG_FLOW_OUT("Success");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwpmudrv_Get_Num_Samples(IOCTL_ARGS arg)
 *
 * @param arg - Pointer to the IOCTL structure
 *
 * @return OS_STATUS
 *
 * @brief       Returns the number of samples collected during the current
 * @brief       sampling run
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Get_Num_Samples(IOCTL_ARGS args)
{
	S32 cpu_num;
	U64 samples = 0;
	OS_STATUS status;

	SEP_DRV_LOG_FLOW_IN("");

	if (pcb == NULL) {
		SEP_DRV_LOG_ERROR("PCB was not initialized.");
		return OS_FAULT;
	}

	if (args->len_drv_to_usr == 0 || args->buf_drv_to_usr == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Topology information has been misconfigured!");
		return OS_INVALID;
	}

	for (cpu_num = 0; cpu_num < GLOBAL_STATE_num_cpus(driver_state);
	     cpu_num++) {
		samples += CPU_STATE_num_samples(&pcb[cpu_num]);

		SEP_DRV_LOG_TRACE("Samples for cpu %d = %lld.", cpu_num,
				  CPU_STATE_num_samples(&pcb[cpu_num]));
	}
	SEP_DRV_LOG_TRACE("Total number of samples %lld.", samples);
	status = put_user(samples, (U64 __user *)args->buf_drv_to_usr);

	SEP_DRV_LOG_FLOW_OUT("Return value: %d", status);
	return status;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwpmudrv_Set_Device_Num_Units(IOCTL_ARGS arg)
 *
 * @param arg - Pointer to the IOCTL structure
 *
 * @return OS_STATUS
 *
 * @brief       Set the number of devices for the sampling run
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Set_Device_Num_Units(IOCTL_ARGS args)
{
	SEP_DRV_LOG_FLOW_IN("");

	if (GET_DRIVER_STATE() != DRV_STATE_IDLE) {
		SEP_DRV_LOG_FLOW_OUT(
			"'Success'/Skipped: driver state is not IDLE!");
		return OS_SUCCESS;
	}

	if (args->len_usr_to_drv == 0 || args->buf_usr_to_drv == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Error: Invalid arguments.");
		return OS_INVALID;
	}

	if (copy_from_user(&(LWPMU_DEVICE_num_units(&devices[cur_device])),
			   (void __user *)args->buf_usr_to_drv, sizeof(U32))) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory copy failure for device num units!");
		return OS_FAULT;
	}
	SEP_DRV_LOG_TRACE("LWP: num_units = %d cur_device = %d.",
			  LWPMU_DEVICE_num_units(&devices[cur_device]),
			  cur_device);
	// on to the next device.
	cur_device++;

	SEP_DRV_LOG_FLOW_OUT("Success");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwpmudrv_Get_Interval_Counts(IOCTL_ARGS arg)
 *
 * @param arg - Pointer to the IOCTL structure
 *
 * @return OS_STATUS
 *
 * @brief       Returns the number of samples collected during the current
 * @brief       sampling run
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Get_Interval_Counts(IOCTL_ARGS args)
{
	SEP_DRV_LOG_FLOW_IN("");

	if (!DRV_CONFIG_enable_cp_mode(drv_cfg)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Not in CP mode!");
		return OS_INVALID;
	}
	if (args->len_drv_to_usr == 0 || args->buf_drv_to_usr == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Interval Counts information has been misconfigured!");
		return OS_INVALID;
	}
	if (!interrupt_counts) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Interrupt_counts is NULL!");
		return OS_INVALID;
	}

	if (copy_to_user((void __user *)args->buf_drv_to_usr, interrupt_counts,
			 args->len_drv_to_usr)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Memory copy failure!");
		return OS_FAULT;
	}

	SEP_DRV_LOG_FLOW_OUT("Success");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          U64 lwpmudrv_Set_Uncore_Topology_Info_And_Scan
 *
 * @brief       Reads the MSR_PLATFORM_INFO register if present
 *
 * @param arg   Pointer to the IOCTL structure
 *
 * @return      status
 *
 * <I>Special Notes:</I>
 *              <NONE>
 */
static OS_STATUS lwpmudrv_Set_Uncore_Topology_Info_And_Scan(IOCTL_ARGS args)
{
	SEP_DRV_LOG_FLOW_IN("");
	SEP_DRV_LOG_FLOW_OUT("Success [but did not do anything]");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          U64 lwpmudrv_Get_Uncore_Topology
 *
 * @brief       Reads the MSR_PLATFORM_INFO register if present
 *
 * @param arg   Pointer to the IOCTL structure
 *
 * @return      status
 *
 * <I>Special Notes:</I>
 *              <NONE>
 */
static OS_STATUS lwpmudrv_Get_Uncore_Topology(IOCTL_ARGS args)
{
	U32 dev;
	static UNCORE_TOPOLOGY_INFO_NODE req_uncore_topology;

	SEP_DRV_LOG_FLOW_IN("");

	if (args->buf_usr_to_drv == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Invalid arguments (buf_usr_to_drv is NULL)!");
		return OS_INVALID;
	}
	if (args->len_usr_to_drv != sizeof(UNCORE_TOPOLOGY_INFO_NODE)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Invalid arguments (unexpected len_usr_to_drv value)!");
		return OS_INVALID;
	}
	if (args->buf_drv_to_usr == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Invalid arguments (buf_drv_to_usr is NULL)!");
		return OS_INVALID;
	}
	if (args->len_drv_to_usr != sizeof(UNCORE_TOPOLOGY_INFO_NODE)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Invalid arguments (unexpected len_drv_to_usr value)!");
		return OS_INVALID;
	}

	memset((char *)&req_uncore_topology, 0,
	       sizeof(UNCORE_TOPOLOGY_INFO_NODE));
	if (copy_from_user(&req_uncore_topology, (void __user *)args->buf_usr_to_drv,
			   args->len_usr_to_drv)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Memory copy failure!");
		return OS_FAULT;
	}

	for (dev = 0; dev < MAX_DEVICES; dev++) {
		// skip if user does not require to scan this device
		if (!UNCORE_TOPOLOGY_INFO_device_scan(&req_uncore_topology,
						      dev)) {
			continue;
		}
		// skip if this device has been discovered
		if (UNCORE_TOPOLOGY_INFO_device_scan(&uncore_topology, dev)) {
			continue;
		}
		memcpy((U8 *)&(UNCORE_TOPOLOGY_INFO_device(&uncore_topology,
							   dev)),
		       (U8 *)&(UNCORE_TOPOLOGY_INFO_device(&req_uncore_topology,
							   dev)),
		       sizeof(UNCORE_PCIDEV_NODE));
		UNC_COMMON_PCI_Scan_For_Uncore((VOID *)&dev, dev, NULL);
	}

	if (copy_to_user((void __user *)args->buf_drv_to_usr, &uncore_topology,
			 args->len_drv_to_usr)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Memory copy failure!");
		return OS_FAULT;
	}

	SEP_DRV_LOG_FLOW_OUT("Success");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          U64 lwpmudrv_Get_Platform_Topology
 *
 * @brief       Reads the MSR or PCI PLATFORM_INFO register if present
 *
 * @param arg   Pointer to the IOCTL structure
 *
 * @return      status
 *
 * <I>Special Notes:</I>
 *              <NONE>
 */
static OS_STATUS lwpmudrv_Get_Platform_Topology(IOCTL_ARGS args)
{
	U32 dev;
	U32 num_topology_devices = 0;

	SEP_DRV_LOG_FLOW_IN("");

	if (args->buf_usr_to_drv == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Invalid arguments (buf_usr_to_drv is NULL)!");
		return OS_INVALID;
	}
	if (args->len_usr_to_drv != sizeof(PLATFORM_TOPOLOGY_PROG_NODE)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Invalid arguments (unexpected len_usr_to_drv value)!");
		return OS_INVALID;
	}
	if (args->buf_drv_to_usr == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Invalid arguments (buf_drv_to_usr is NULL)!");
		return OS_INVALID;
	}
	if (args->len_drv_to_usr != sizeof(PLATFORM_TOPOLOGY_PROG_NODE)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Invalid arguments (unexpected len_drv_to_usr value)!");
		return OS_INVALID;
	}

	memset((char *)&req_platform_topology_prog_node, 0,
	       sizeof(PLATFORM_TOPOLOGY_PROG_NODE));
	if (copy_from_user(&req_platform_topology_prog_node,
			   (void __user *)args->buf_usr_to_drv, args->len_usr_to_drv)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory copy failure for req_platform_topology_prog_node!");
		return OS_FAULT;
	}

	num_topology_devices = PLATFORM_TOPOLOGY_PROG_num_devices(
		&req_platform_topology_prog_node);
	for (dev = 0; dev < num_topology_devices; dev++) {
		//skip if we have populated the register values already
		if (PLATFORM_TOPOLOGY_PROG_topology_device_prog_valid(
			    &platform_topology_prog_node, dev)) {
			continue;
		}
		memcpy((U8 *)&(PLATFORM_TOPOLOGY_PROG_topology_device(
			       &platform_topology_prog_node, dev)),
		       (U8 *)&(PLATFORM_TOPOLOGY_PROG_topology_device(
			       &req_platform_topology_prog_node, dev)),
		       sizeof(PLATFORM_TOPOLOGY_DISCOVERY_NODE));
		UNC_COMMON_Get_Platform_Topology(dev);
	}

	if (copy_to_user((void __user *)args->buf_drv_to_usr, &platform_topology_prog_node,
			 args->len_drv_to_usr)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory copy failure for platform_topology_prog_node!");
		return OS_FAULT;
	}

	SEP_DRV_LOG_FLOW_OUT("Success");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          OS_STATUS lwpmudrv_Flush(void)
 *
 * @brief       Flushes the current contents of sampling buffers
 *
 * @param     - none
 *
 * @return      status
 *
 * <I>Special Notes:</I>
 */
static OS_STATUS lwpmudrv_Flush(void)
{
	OS_STATUS status = OS_FAULT;
	SEP_DRV_LOG_FLOW_IN("");

	if (!DRV_CONFIG_enable_cp_mode(drv_cfg)) {
		SEP_DRV_LOG_ERROR(
			"The flush failed. Continuous profiling, -cp, is not enabled!");
		goto clean_return;
	}

	if (!DRIVER_STATE_IN(GET_DRIVER_STATE(), STATE_BIT_PAUSED)) {
		SEP_DRV_LOG_ERROR(
			"The flush failed. The driver should be paused!");
		goto clean_return;
	}

	if (multi_pebs_enabled || sched_switch_enabled) {
#if !defined(DRV_SEP_ACRN_ON)
		CONTROL_Invoke_Parallel(PEBS_Flush_Buffer, NULL);
#endif
	}

	LINUXOS_Uninstall_Hooks();
	LINUXOS_Enum_Process_Modules(TRUE);
	status = OUTPUT_Flush();
	LINUXOS_Install_Hooks();

clean_return:
	SEP_DRV_LOG_FLOW_OUT("Status: %d.", status);
	return status;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          U64 lwpmudrv_Get_Driver_log
 *
 * @brief       Dumps the driver log
 *
 * @param arg   Pointer to the IOCTL structure
 *
 * @return      status
 *
 * <I>Special Notes:</I>
 *              <NONE>
 */
static OS_STATUS lwpmudrv_Get_Driver_Log(IOCTL_ARGS args)
{
	SEP_DRV_LOG_FLOW_IN("");

	if (args->buf_drv_to_usr == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Invalid arguments (buf_drv_to_usr is NULL)!");
		return OS_INVALID;
	}
	if (args->len_drv_to_usr < sizeof(*DRV_LOG())) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Invalid arguments (unexpected len_drv_to_usr value)!");
		return OS_INVALID;
	}

	if (copy_to_user((void __user *)args->buf_drv_to_usr, DRV_LOG(), sizeof(*DRV_LOG()))) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Memory copy failure!");
		return OS_FAULT;
	}

	SEP_DRV_LOG_DISAMBIGUATE(); // keeps the driver log's footprint unique (has the highest disambiguator field)

	SEP_DRV_LOG_FLOW_OUT("Success");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          U64 lwpmudrv_Control_Driver_log
 *
 * @brief       Sets or/and gets the driver log's configuration
 *
 * @param arg   Pointer to the IOCTL structure
 *
 * @return      status
 *
 * <I>Special Notes:</I>
 *              <NONE>
 */
static OS_STATUS lwpmudrv_Control_Driver_Log(IOCTL_ARGS args)
{
	DRV_LOG_CONTROL_NODE log_control;
	U32 i;

	SEP_DRV_LOG_FLOW_IN("");

	if (args->buf_usr_to_drv == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Invalid arguments (buf_usr_to_drv is NULL)!");
		return OS_INVALID;
	}
	if (args->len_usr_to_drv < sizeof(log_control)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Invalid arguments (unexpected len_usr_to_drv value)!");
		return OS_INVALID;
	}

	if (copy_from_user(&log_control, (void __user *)args->buf_usr_to_drv,
			   sizeof(log_control))) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Memory copy failure!");
		return OS_FAULT;
	}

	if (DRV_LOG_CONTROL_command(&log_control) ==
	    DRV_LOG_CONTROL_COMMAND_ADJUST_VERBOSITY) {
		for (i = 0; i < DRV_NB_LOG_CATEGORIES; i++) {
			if (DRV_LOG_CONTROL_verbosities(&log_control)[i] ==
			    LOG_VERBOSITY_UNSET) {
				SEP_DRV_LOG_TRACE(
					"Current verbosity mask for '%s' is 0x%x",
					(UTILITY_Log_Category_Strings()[i]),
					((U32)DRV_LOG_VERBOSITY(i)));
				DRV_LOG_CONTROL_verbosities(&log_control)[i] =
					DRV_LOG_VERBOSITY(i);
			} else if (DRV_LOG_CONTROL_verbosities(
					   &log_control)[i] ==
				   LOG_VERBOSITY_DEFAULT) {
				U32 verbosity;
				switch (i) {
				case DRV_LOG_CATEGORY_LOAD:
					verbosity =
						DRV_LOG_DEFAULT_LOAD_VERBOSITY;
					break;
				case DRV_LOG_CATEGORY_INIT:
					verbosity =
						DRV_LOG_DEFAULT_INIT_VERBOSITY;
					break;
				case DRV_LOG_CATEGORY_DETECTION:
					verbosity =
						DRV_LOG_DEFAULT_DETECTION_VERBOSITY;
					break;
				case DRV_LOG_CATEGORY_ERROR:
					verbosity =
						DRV_LOG_DEFAULT_ERROR_VERBOSITY;
					break;
				case DRV_LOG_CATEGORY_STATE_CHANGE:
					verbosity =
						DRV_LOG_DEFAULT_STATE_CHANGE_VERBOSITY;
					break;
				case DRV_LOG_CATEGORY_MARK:
					verbosity =
						DRV_LOG_DEFAULT_MARK_VERBOSITY;
					break;
				case DRV_LOG_CATEGORY_DEBUG:
					verbosity =
						DRV_LOG_DEFAULT_DEBUG_VERBOSITY;
					break;
				case DRV_LOG_CATEGORY_FLOW:
					verbosity =
						DRV_LOG_DEFAULT_FLOW_VERBOSITY;
					break;
				case DRV_LOG_CATEGORY_ALLOC:
					verbosity =
						DRV_LOG_DEFAULT_ALLOC_VERBOSITY;
					break;
				case DRV_LOG_CATEGORY_INTERRUPT:
					verbosity =
						DRV_LOG_DEFAULT_INTERRUPT_VERBOSITY;
					break;
				case DRV_LOG_CATEGORY_TRACE:
					verbosity =
						DRV_LOG_DEFAULT_TRACE_VERBOSITY;
					break;
				case DRV_LOG_CATEGORY_REGISTER:
					verbosity =
						DRV_LOG_DEFAULT_REGISTER_VERBOSITY;
					break;
				case DRV_LOG_CATEGORY_NOTIFICATION:
					verbosity =
						DRV_LOG_DEFAULT_NOTIFICATION_VERBOSITY;
					break;
				case DRV_LOG_CATEGORY_WARNING:
					verbosity =
						DRV_LOG_DEFAULT_WARNING_VERBOSITY;
					break;
				// default:
				// 	SEP_DRV_LOG_ERROR(
				// 		"Unspecified category '%s' when resetting to default!",
				// 		UTILITY_Log_Category_Strings()
				// 			[i]);
				// 	verbosity = LOG_VERBOSITY_NONE;
				// 	break;
				}
				SEP_DRV_LOG_INIT(
					"Resetting verbosity mask for '%s' from 0x%x to 0x%x.",
					UTILITY_Log_Category_Strings()[i],
					(U32)DRV_LOG_VERBOSITY(i), verbosity);
				DRV_LOG_VERBOSITY(i) = verbosity;
				DRV_LOG_CONTROL_verbosities(&log_control)[i] =
					verbosity;
			} else {
				SEP_DRV_LOG_INIT(
					"Changing verbosity mask for '%s' from 0x%x to 0x%x.",
					UTILITY_Log_Category_Strings()[i],
					(U32)DRV_LOG_VERBOSITY(i),
					(U32)DRV_LOG_CONTROL_verbosities(
						&log_control)[i]);
				DRV_LOG_VERBOSITY(i) =
					DRV_LOG_CONTROL_verbosities(
						&log_control)[i];
			}
		}

		for (; i < DRV_MAX_NB_LOG_CATEGORIES; i++) {
			DRV_LOG_CONTROL_verbosities(&log_control)[i] =
				LOG_VERBOSITY_UNSET;
		}

		if (copy_to_user((void __user *)args->buf_drv_to_usr, &log_control,
				 sizeof(log_control))) {
			SEP_DRV_LOG_ERROR_FLOW_OUT("Memory copy failure!");
			return OS_FAULT;
		}
	} else if (DRV_LOG_CONTROL_command(&log_control) ==
		   DRV_LOG_CONTROL_COMMAND_MARK) {
		DRV_LOG_CONTROL_message(
			&log_control)[DRV_LOG_CONTROL_MAX_DATA_SIZE - 1] = 0;
		SEP_DRV_LOG_MARK("Mark: '%s'.",
				 DRV_LOG_CONTROL_message(&log_control));
	} else if (DRV_LOG_CONTROL_command(&log_control) ==
		   DRV_LOG_CONTROL_COMMAND_QUERY_SIZE) {
		DRV_LOG_CONTROL_log_size(&log_control) = sizeof(*DRV_LOG());
		SEP_DRV_LOG_TRACE("Driver log size is %u bytes.",
				  DRV_LOG_CONTROL_log_size(&log_control));
		if (copy_to_user((void __user *)args->buf_drv_to_usr, &log_control,
				 sizeof(log_control))) {
			SEP_DRV_LOG_ERROR_FLOW_OUT("Memory copy failure!");
			return OS_FAULT;
		}
	} else if (DRV_LOG_CONTROL_command(&log_control) ==
		   DRV_LOG_CONTROL_COMMAND_BENCHMARK) {
		U32 nb_iterations =
			*(U32 *)&DRV_LOG_CONTROL_message(&log_control);

		SEP_DRV_LOG_INIT_IN("Starting benchmark (%u iterations)...",
				    nb_iterations);
		for (i = 0; i < nb_iterations; i++) {
			(void)i;
		}
		SEP_DRV_LOG_INIT_OUT("Benchmark complete (%u/%u iterations).",
				     i, nb_iterations);
	}

	SEP_DRV_LOG_FLOW_OUT("Success");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          U64 lwpmudrv_Get_Drv_Setup_Info
 *
 * @brief       Get numerous information of driver
 *
 * @param arg   Pointer to the IOCTL structure
 *
 * @return      status
 *
 * <I>Special Notes:</I>
 *              <NONE>
 */
static OS_STATUS lwpmudrv_Get_Drv_Setup_Info(IOCTL_ARGS args)
{
#define VMM_VENDOR_STR_LEN 12
	U32 pebs_unavailable = 0;
	U64 rbx, rcx, rdx, num_basic_functions;
	S8 vmm_vendor_name[VMM_VENDOR_STR_LEN + 1];
	S8 *vmm_vmware_str = "VMwareVMware";
	S8 *vmm_kvm_str = "KVMKVMKVM\0\0\0";
	S8 *vmm_mshyperv_str = "Microsoft Hv";
	S8 *vmm_acrn_str = "ACRNACRNACRN";
#if defined(DRV_USE_KAISER)
	int *kaiser_enabled_ptr;
	int *kaiser_pti_option;
#endif
	bool is_hypervisor = FALSE;

	SEP_DRV_LOG_FLOW_IN("Args: %p.", args);

	if (args->buf_drv_to_usr == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Invalid arguments (buf_drv_to_usr is NULL)!");
		return OS_INVALID;
	}
	if (args->len_drv_to_usr != sizeof(DRV_SETUP_INFO_NODE)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Invalid arguments (unexpected len_drv_to_usr value)!");
		return OS_INVALID;
	}

	memset((char *)&req_drv_setup_info, 0, sizeof(DRV_SETUP_INFO_NODE));

	DRV_SETUP_INFO_nmi_mode(&req_drv_setup_info) = 1;

#if defined(DRV_SEP_ACRN_ON)
	is_hypervisor = TRUE;
#endif
	if (boot_cpu_has(X86_FEATURE_HYPERVISOR) || is_hypervisor) {
		UTILITY_Read_Cpuid(0x40000000, &num_basic_functions, &rbx, &rcx,
				   &rdx);
		memcpy(vmm_vendor_name, &rbx, 4);
		memcpy(vmm_vendor_name + 4, &rcx, 4);
		memcpy(vmm_vendor_name + 8, &rdx, 4);
		memcpy(vmm_vendor_name + 12, "\0", 1);

		if (!strncmp(vmm_vendor_name, vmm_vmware_str,
			     VMM_VENDOR_STR_LEN)) {
			DRV_SETUP_INFO_vmm_mode(&req_drv_setup_info) = 1;
			DRV_SETUP_INFO_vmm_vendor(&req_drv_setup_info) =
				DRV_VMM_VMWARE;
		} else if (!strncmp(vmm_vendor_name, vmm_kvm_str,
				    VMM_VENDOR_STR_LEN)) {
			DRV_SETUP_INFO_vmm_mode(&req_drv_setup_info) = 1;
			DRV_SETUP_INFO_vmm_vendor(&req_drv_setup_info) =
				DRV_VMM_KVM;
		} else if (!strncmp(vmm_vendor_name, vmm_acrn_str,
				    VMM_VENDOR_STR_LEN)) {
			DRV_SETUP_INFO_vmm_mode(&req_drv_setup_info) = 1;
			DRV_SETUP_INFO_vmm_vendor(&req_drv_setup_info) =
				DRV_VMM_ACRN;
		} else if (!strncmp(vmm_vendor_name, vmm_mshyperv_str,
				    VMM_VENDOR_STR_LEN)) {
			DRV_SETUP_INFO_vmm_mode(&req_drv_setup_info) = 1;
			DRV_SETUP_INFO_vmm_vendor(&req_drv_setup_info) =
				DRV_VMM_HYPERV;
			if (num_basic_functions >= 0x40000003) {
				UTILITY_Read_Cpuid(0x40000003,
						   &num_basic_functions, &rbx,
						   &rcx, &rdx);
				if (rbx & 0x1) {
					DRV_SETUP_INFO_vmm_guest_vm(
						&req_drv_setup_info) = 0;
				} else {
					DRV_SETUP_INFO_vmm_guest_vm(
						&req_drv_setup_info) = 1;
				}
			}
		}
	}
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 32)
	else if (xen_domain()) {
		DRV_SETUP_INFO_vmm_mode(&req_drv_setup_info) = 1;
		DRV_SETUP_INFO_vmm_vendor(&req_drv_setup_info) = DRV_VMM_XEN;

		if (xen_initial_domain()) {
			DRV_SETUP_INFO_vmm_guest_vm(&req_drv_setup_info) = 0;
		} else {
			DRV_SETUP_INFO_vmm_guest_vm(&req_drv_setup_info) = 1;
		}
	}
#endif
	else {
		if (LINUXOS_Check_KVM_Guest_Process()) {
			DRV_SETUP_INFO_vmm_mode(&req_drv_setup_info) = 1;
			DRV_SETUP_INFO_vmm_vendor(&req_drv_setup_info) =
				DRV_VMM_KVM;
		}
	}

	pebs_unavailable = (SYS_Read_MSR(IA32_MISC_ENABLE) >> 12) & 0x1;
	if (!pebs_unavailable) {
		if (!wrmsr_safe(IA32_PEBS_ENABLE, 0, 0)) {
			DRV_SETUP_INFO_pebs_accessible(&req_drv_setup_info) = 1;
		}
	}

#if defined(DRV_USE_KAISER)
	kaiser_enabled_ptr = (int *)UTILITY_Find_Symbol("kaiser_enabled");
	if (kaiser_enabled_ptr && *kaiser_enabled_ptr) {
		SEP_DRV_LOG_INIT(
			"KAISER is enabled! (&kaiser_enable=%p, val: %d).",
			kaiser_enabled_ptr, *kaiser_enabled_ptr);
		DRV_SETUP_INFO_page_table_isolation(&req_drv_setup_info) =
			DRV_SETUP_INFO_PTI_KAISER;
	} else {
		kaiser_pti_option = (int *)UTILITY_Find_Symbol("pti_option");
		if (kaiser_pti_option) {
			SEP_DRV_LOG_INIT(
				"KAISER pti_option=%p pti_option val=%d",
				kaiser_pti_option, *kaiser_pti_option);
#if defined(X86_FEATURE_PTI)
			if (static_cpu_has(X86_FEATURE_PTI)) {
				SEP_DRV_LOG_INIT(
					"KAISER is Enabled or in Auto Enable!\n");
				DRV_SETUP_INFO_page_table_isolation(
					&req_drv_setup_info) =
					DRV_SETUP_INFO_PTI_KAISER;
			} else {
				SEP_DRV_LOG_INIT(
					"KAISER is present but disabled!");
			}
#endif
		}
	}
	if (!kaiser_enabled_ptr && !kaiser_pti_option) {
		SEP_DRV_LOG_ERROR(
			"Could not find KAISER information. Assuming no KAISER!");
	}
#elif defined(DRV_USE_PTI)
	if (static_cpu_has(X86_FEATURE_PTI)) {
		SEP_DRV_LOG_INIT("Kernel Page Table Isolation is enabled!");
		DRV_SETUP_INFO_page_table_isolation(&req_drv_setup_info) =
			DRV_SETUP_INFO_PTI_KPTI;
	}
#endif

	SEP_DRV_LOG_TRACE("DRV_SETUP_INFO nmi_mode %d.",
			  DRV_SETUP_INFO_nmi_mode(&req_drv_setup_info));
	SEP_DRV_LOG_TRACE("DRV_SETUP_INFO vmm_mode %d.",
			  DRV_SETUP_INFO_vmm_mode(&req_drv_setup_info));
	SEP_DRV_LOG_TRACE("DRV_SETUP_INFO vmm_vendor %d.",
			  DRV_SETUP_INFO_vmm_vendor(&req_drv_setup_info));
	SEP_DRV_LOG_TRACE("DRV_SETUP_INFO vmm_guest_vm %d.",
			  DRV_SETUP_INFO_vmm_guest_vm(&req_drv_setup_info));
	SEP_DRV_LOG_TRACE("DRV_SETUP_INFO pebs_accessible %d.",
			  DRV_SETUP_INFO_pebs_accessible(&req_drv_setup_info));
	SEP_DRV_LOG_TRACE(
		"DRV_SETUP_INFO page_table_isolation %d.",
		DRV_SETUP_INFO_page_table_isolation(&req_drv_setup_info));

#if defined(DRV_CPU_HOTPLUG)
	DRV_SETUP_INFO_cpu_hotplug_mode(&req_drv_setup_info) = 1;
#endif

	if (copy_to_user((void __user *)args->buf_drv_to_usr, &req_drv_setup_info,
			 args->len_drv_to_usr)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Memory allocation failure!");
		return OS_FAULT;
	}

	SEP_DRV_LOG_FLOW_OUT("Success.");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          U64 lwpmudrv_Set_Emon_Buffer_Driver_Helper
 *
 * @brief       Setup EMON buffer driver helper
 *
 * @param arg   Pointer to the IOCTL structure
 *
 * @return      status
 *
 * <I>Special Notes:</I>
 *              <NONE>
 */
static OS_STATUS lwpmudrv_Set_Emon_Buffer_Driver_Helper(IOCTL_ARGS args)
{
	SEP_DRV_LOG_FLOW_IN("");

	if (args->len_usr_to_drv == 0 || args->buf_usr_to_drv == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Error: Invalid arguments.");
		return OS_INVALID;
	}

	if (!emon_buffer_driver_helper) {
		emon_buffer_driver_helper =
			CONTROL_Allocate_Memory(args->len_usr_to_drv);
		if (emon_buffer_driver_helper == NULL) {
			SEP_DRV_LOG_ERROR_FLOW_OUT(
				"Memory allocation failure for emon_buffer_driver_helper!");
			return OS_NO_MEM;
		}
	}

	if (copy_from_user(emon_buffer_driver_helper, (void __user *)args->buf_usr_to_drv,
			   args->len_usr_to_drv)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory copy failure for device num units!");
		return OS_FAULT;
	}

	SEP_DRV_LOG_FLOW_OUT("Success");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          U64 lwpmudrv_Set_OSID
 *
 * @brief       Set OSID with specified value
 *
 * @param arg   Pointer to the IOCTL structure
 *
 * @return      status
 *
 * <I>Special Notes:</I>
 *              <NONE>
 */

static OS_STATUS lwpmudrv_Set_OSID(IOCTL_ARGS args)
{
	OS_STATUS status = OS_SUCCESS;

	if (args->buf_usr_to_drv == NULL) {
		SEP_PRINT_ERROR("Invalid arguments (buf_usr_to_drv is NULL)!");
		return OS_INVALID;
	}
	if (args->len_usr_to_drv != sizeof(U32)) {
		SEP_PRINT_ERROR(
			"Invalid arguments (unexpected len_usr_to_drv value)!");
		return OS_INVALID;
	}

	status = get_user(osid, (U32  __user *)args->buf_usr_to_drv);
	return status;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwpmudrv_Get_Agent_Mode(IOCTL_ARGS arg)
 *
 * @param arg - pointer to the IOCTL_ARGS structure
 *
 * @return OS_STATUS
 *
 * @brief  Local function that copies agent mode from drv to usr code
 * @brief  Returns status.
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Get_Agent_Mode(IOCTL_ARGS args)
{
	OS_STATUS status;

	if (args->buf_drv_to_usr == NULL) {
		SEP_PRINT_ERROR("Invalid arguments (buf_drv_to_usr is NULL)!");
		return OS_INVALID;
	}
	if (args->len_drv_to_usr != sizeof(U32)) {
		SEP_PRINT_ERROR(
			"Invalid arguments (unexpected len_drv_to_usr value)!");
		return OS_INVALID;
	}

#if defined(DRV_SEP_ACRN_ON)
	status = put_user(HOST_VM_AGENT, (U32 __user *)args->buf_drv_to_usr);
	sched_switch_enabled = TRUE;
#else
	status = put_user(-1, (U32 __user *)args->buf_drv_to_usr);
	SEP_PRINT_ERROR("Invalid agent mode..!");
	status = OS_INVALID;
#endif

	return status;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwpmudrv_Get_Num_Of_Vms(IOCTL_ARGS arg)
 *
 * @param arg - pointer to the IOCTL_ARGS structure
 *
 * @return OS_STATUS
 *
 * @brief  Local function to get number of VMS available
 * @brief  Returns status.
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Get_Num_Of_Vms(IOCTL_ARGS args)

{
	VM_OSID_MAP_NODE vm_map;
#if defined(DRV_SEP_ACRN_ON)
	U32 i;
#endif
	if (args->buf_drv_to_usr == NULL) {
		SEP_PRINT_ERROR("Invalid arguments (buf_drv_to_usr is NULL)!");
		return OS_INVALID;
	}

	if (args->len_drv_to_usr != sizeof(VM_OSID_MAP_NODE)) {
		SEP_PRINT_ERROR(
			"Invalid arguments (unexpected len_drv_to_usr value)!");
		return OS_INVALID;
	}

	memset(&vm_map, 0, sizeof(VM_OSID_MAP_NODE));

#if defined(DRV_SEP_ACRN_ON)
	if (vm_info_list == NULL) {
		vm_info_list =
		CONTROL_Allocate_Memory(sizeof(struct profiling_vm_info_list));
	}
	memset(vm_info_list, 0, sizeof(struct profiling_vm_info_list));

	BUG_ON(!virt_addr_valid(vm_info_list));

	acrn_hypercall2(HC_PROFILING_OPS, PROFILING_GET_VMINFO,
			virt_to_phys(vm_info_list));

	vm_map.num_vms = 0;
	for (i = 0; i < vm_info_list->num_vms; i++) {
		if (vm_info_list->vm_list[i].num_vcpus != 0) {
			vm_map.osid[i] = (U32)vm_info_list->vm_list[i].vm_id;
			vm_map.num_vms++;
		}
	}

#endif
	if (copy_to_user((void __user *)args->buf_drv_to_usr,
		&vm_map, args->len_drv_to_usr)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Memory copy failure!");
		return OS_FAULT;
	}

	return OS_SUCCESS;

}
/* ------------------------------------------------------------------------- */
/*!
 * @fn  static OS_STATUS lwpmudrv_Get_Cpu_Map_Info(IOCTL_ARGS arg)
 *
 * @param arg - pointer to the IOCTL_ARGS structure
 *
 * @return OS_STATUS
 *
 * @brief  Local function to get pcpu-vcpu mapping info
 * @brief  Returns status.
 *
 * <I>Special Notes</I>
 */
static OS_STATUS lwpmudrv_Get_Cpu_Map_Info(IOCTL_ARGS args)
{
	CPU_MAP_TRACE_LIST cpumap;
	DRV_STATUS status = OS_SUCCESS;
#if defined(DRV_SEP_ACRN_ON)
	U32 i, j;
#endif

	if ((args->buf_drv_to_usr == NULL) ||
		(args->len_drv_to_usr != sizeof(CPU_MAP_TRACE_LIST_NODE))) {
		SEP_PRINT_ERROR("Invalid drv_to_usr arguments!");
		return OS_INVALID;
	}

	if ((args->buf_usr_to_drv == NULL) ||
		(args->len_usr_to_drv != sizeof(CPU_MAP_TRACE_LIST_NODE))) {
		SEP_PRINT_ERROR("Invalid usr_to_drv arguments!");
		return OS_INVALID;
	}

	cpumap = (CPU_MAP_TRACE_LIST)
		CONTROL_Allocate_Memory(sizeof(CPU_MAP_TRACE_LIST_NODE));
	if (cpumap == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Memory allocation failure");
		return OS_NO_MEM;
	}

	if (copy_from_user(cpumap, (void __user *)args->buf_usr_to_drv,
			sizeof(CPU_MAP_TRACE_LIST_NODE))) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Memory copy failure");
		status = OS_FAULT;
		goto cleanup;
	}

#if defined(DRV_SEP_ACRN_ON)
	if (vm_info_list == NULL) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("vm_info_list is NULL!");
		status = OS_INVALID;
		goto cleanup;
	}

	SEP_DRV_LOG_TRACE("CPU mapping for osid %d ", cpumap->osid);
	for (i = 0; i < vm_info_list->num_vms; i++) {
		if (vm_info_list->vm_list[i].vm_id == cpumap->osid) {
			for (j = 0;
			     j < vm_info_list->vm_list[i].num_vcpus; j++) {
				UTILITY_Read_TSC(&(cpumap->entries[j].tsc));
				cpumap->entries[j].is_static = 1;
				cpumap->entries[j].vcpu_id =
				vm_info_list->vm_list[i].cpu_map[j].vcpu_id;
				cpumap->entries[j].pcpu_id =
				vm_info_list->vm_list[i].cpu_map[j].pcpu_id;
				cpumap->entries[j].os_id =
				vm_info_list->vm_list[i].vm_id;
				cpumap->num_entries++;
			}
		}
	}
#endif
	if (copy_to_user((void __user *)args->buf_drv_to_usr,
		cpumap, args->len_drv_to_usr)) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Memory copy failure!");
		status = OS_FAULT;
		goto cleanup;
	}

cleanup:
	cpumap = CONTROL_Free_Memory(cpumap);
	return status;
}


/*******************************************************************************
 *  External Driver functions - Open
 *      This function is common to all drivers
 *******************************************************************************/

static int lwpmu_Open(struct inode *inode, struct file *filp)
{
	SEP_DRV_LOG_TRACE_IN("Maj:%d, min:%d", imajor(inode), iminor(inode));

	filp->private_data = container_of(inode->i_cdev, LWPMU_DEV_NODE, cdev);

	SEP_DRV_LOG_TRACE_OUT("");
	return 0;
}

/*******************************************************************************
 *  External Driver functions
 *      These functions are registered into the file operations table that
 *      controls this device.
 *      Open, Close, Read, Write, Release
 *******************************************************************************/

static ssize_t lwpmu_Read(struct file *filp, char __user *buf, size_t count,
			  loff_t *f_pos)
{
	unsigned long retval;

	SEP_DRV_LOG_TRACE_IN("");

	/* Transfering data to user space */
	SEP_DRV_LOG_TRACE("Dispatched with count=%d.", (S32)count);
	if (copy_to_user((void __user *)buf, &LWPMU_DEV_buffer(lwpmu_control), 1)) {
		retval = OS_FAULT;
		SEP_DRV_LOG_ERROR_TRACE_OUT("Memory copy failure!");
		return retval;
	}
	/* Changing reading position as best suits */
	if (*f_pos == 0) {
		*f_pos += 1;
		SEP_DRV_LOG_TRACE_OUT("Return value: 1.");
		return 1;
	}

	SEP_DRV_LOG_TRACE_OUT("Return value: 0.");
	return 0;
}

static ssize_t lwpmu_Write(struct file *filp, const char __user *buf, size_t count,
			   loff_t *f_pos)
{
	unsigned long retval;

	SEP_DRV_LOG_TRACE_IN("");

	SEP_DRV_LOG_TRACE("Dispatched with count=%d.", (S32)count);
	if (copy_from_user(&LWPMU_DEV_buffer(lwpmu_control), (void __user *)(buf + count - 1),
			   1)) {
		retval = OS_FAULT;
		SEP_DRV_LOG_ERROR_TRACE_OUT("Memory copy failure!");
		return retval;
	}

	SEP_DRV_LOG_TRACE_OUT("Return value: 1.");
	return 1;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  extern IOCTL_OP_TYPE lwpmu_Service_IOCTL(IOCTL_USE_NODE, filp, cmd, arg)
 *
 * @param   IOCTL_USE_INODE       - Used for pre 2.6.32 kernels
 * @param   struct   file   *filp - file pointer
 * @param   unsigned int     cmd  - IOCTL command
 * @param   unsigned long    arg  - args to the IOCTL command
 *
 * @return OS_STATUS
 *
 * @brief  SEP Worker function that handles IOCTL requests from the user mode.
 *
 * <I>Special Notes</I>
 */
static IOCTL_OP_TYPE lwpmu_Service_IOCTL(IOCTL_USE_INODE struct file *filp,
					 unsigned int cmd,
					 IOCTL_ARGS_NODE local_args)
{
	int status = OS_SUCCESS;

	SEP_DRV_LOG_TRACE_IN("Command: %d.", cmd);

	if (cmd == DRV_OPERATION_GET_DRIVER_STATE) {
		SEP_DRV_LOG_TRACE("DRV_OPERATION_GET_DRIVER_STATE.");
		status = lwpmudrv_Get_Driver_State(&local_args);
		SEP_DRV_LOG_TRACE_OUT("Return value for command %d: %d", cmd,
				      status);
		return status;
	}
	if (cmd == DRV_OPERATION_GET_DRIVER_LOG) {
		SEP_DRV_LOG_TRACE("DRV_OPERATION_GET_DRIVER_LOG.");
		status = lwpmudrv_Get_Driver_Log(&local_args);
		SEP_DRV_LOG_TRACE_OUT("Return value for command %d: %d", cmd,
				      status);
		return status;
	}
	if (cmd == DRV_OPERATION_CONTROL_DRIVER_LOG) {
		SEP_DRV_LOG_TRACE("DRV_OPERATION_CONTROL_DRIVER_LOG.");
		status = lwpmudrv_Control_Driver_Log(&local_args);
		SEP_DRV_LOG_TRACE_OUT("Return value for command %d: %d", cmd,
				      status);
		return status;
	}
	if (GET_DRIVER_STATE() == DRV_STATE_PREPARE_STOP) {
		SEP_DRV_LOG_TRACE("skipping ioctl -- processing stop.");
		SEP_DRV_LOG_TRACE_OUT("Return value for command %d: %d", cmd,
				      status);
		return status;
	}

	MUTEX_LOCK(ioctl_lock);
	UTILITY_Driver_Set_Active_Ioctl(cmd);

	switch (cmd) {
	/*
	* Common IOCTL commands
	*/

	case DRV_OPERATION_VERSION:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_VERSION.");
		status = lwpmudrv_Version(&local_args);
		break;

	case DRV_OPERATION_RESERVE:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_RESERVE.");
		status = lwpmudrv_Reserve(&local_args);
		break;

	case DRV_OPERATION_INIT_DRIVER:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_INIT_DRIVER.");
		status = lwpmudrv_Initialize_Driver(local_args.buf_usr_to_drv,
						    local_args.len_usr_to_drv);
		break;

	case DRV_OPERATION_INIT:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_INIT.");
		status = lwpmudrv_Initialize(local_args.buf_usr_to_drv,
					     local_args.len_usr_to_drv);
		break;

	case DRV_OPERATION_INIT_PMU:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_INIT_PMU.");
		status = lwpmudrv_Init_PMU(&local_args);
		break;

	case DRV_OPERATION_SET_CPU_MASK:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_SET_CPU_MASK.");
		status = lwpmudrv_Set_CPU_Mask(local_args.buf_usr_to_drv,
					       local_args.len_usr_to_drv);
		break;

	case DRV_OPERATION_START:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_START.");
		status = lwpmudrv_Start();
		break;

	case DRV_OPERATION_STOP:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_STOP.");
		status = lwpmudrv_Prepare_Stop();
		UTILITY_Driver_Set_Active_Ioctl(0);
		MUTEX_UNLOCK(ioctl_lock);

		MUTEX_LOCK(ioctl_lock);
		UTILITY_Driver_Set_Active_Ioctl(cmd);
		if (GET_DRIVER_STATE() == DRV_STATE_PREPARE_STOP) {
			status = lwpmudrv_Finish_Stop();
			if (status == OS_SUCCESS) {
				// if stop was successful, relevant memory should have been freed,
				// so try to compact the memory tracker
				CONTROL_Memory_Tracker_Compaction();
			}
		}
		break;

	case DRV_OPERATION_PAUSE:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_PAUSE.");
		status = lwpmudrv_Pause();
		break;

	case DRV_OPERATION_RESUME:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_RESUME.");
		status = lwpmudrv_Resume();
		break;

	case DRV_OPERATION_EM_GROUPS:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_EM_GROUPS.");
		status = lwpmudrv_Set_EM_Config(&local_args);
		break;

	case DRV_OPERATION_EM_CONFIG_NEXT:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_EM_CONFIG_NEXT.");
		status = lwpmudrv_Configure_Events(&local_args);
		break;

	case DRV_OPERATION_NUM_DESCRIPTOR:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_NUM_DESCRIPTOR.");
		status = lwpmudrv_Set_Sample_Descriptors(&local_args);
		break;

	case DRV_OPERATION_DESC_NEXT:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_DESC_NEXT.");
		status = lwpmudrv_Configure_Descriptors(&local_args);
		break;

	case DRV_OPERATION_GET_NORMALIZED_TSC:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_GET_NORMALIZED_TSC.");
		status = lwpmudrv_Get_Normalized_TSC(&local_args);
		break;

	case DRV_OPERATION_GET_NORMALIZED_TSC_STANDALONE:
		SEP_DRV_LOG_TRACE(
			"DRV_OPERATION_GET_NORMALIZED_TSC_STANDALONE.");
		status = lwpmudrv_Get_Normalized_TSC(&local_args);
		break;

	case DRV_OPERATION_NUM_CORES:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_NUM_CORES.");
		status = lwpmudrv_Get_Num_Cores(&local_args);
		break;

	case DRV_OPERATION_KERNEL_CS:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_KERNEL_CS.");
		status = lwpmudrv_Get_KERNEL_CS(&local_args);
		break;

	case DRV_OPERATION_SET_UID:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_SET_UID.");
		status = lwpmudrv_Set_UID(&local_args);
		break;

	case DRV_OPERATION_TSC_SKEW_INFO:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_TSC_SKEW_INFO.");
		status = lwpmudrv_Get_TSC_Skew_Info(&local_args);
		break;

	case DRV_OPERATION_COLLECT_SYS_CONFIG:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_COLLECT_SYS_CONFIG.");
		status = lwpmudrv_Collect_Sys_Config(&local_args);
		break;

	case DRV_OPERATION_GET_SYS_CONFIG:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_GET_SYS_CONFIG.");
		status = lwpmudrv_Sys_Config(&local_args);
		break;

	case DRV_OPERATION_TERMINATE:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_TERMINATE.");
		status = lwpmudrv_Terminate();
		break;

	case DRV_OPERATION_SET_CPU_TOPOLOGY:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_SET_CPU_TOPOLOGY.");
		status = lwpmudrv_Setup_Cpu_Topology(&local_args);
		break;

	case DRV_OPERATION_GET_NUM_CORE_CTRS:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_GET_NUM_CORE_CTRS.");
		status = lwpmudrv_Samp_Read_Num_Of_Core_Counters(&local_args);
		break;

	case DRV_OPERATION_GET_PLATFORM_INFO:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_GET_PLATFORM_INFO.");
		status = lwpmudrv_Get_Platform_Info(&local_args);
		break;

	case DRV_OPERATION_READ_MSRS:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_READ_MSRs.");
		status = lwpmudrv_Read_MSRs(&local_args);
		break;

	case DRV_OPERATION_SWITCH_GROUP:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_SWITCH_GROUP.");
		status = lwpmudrv_Switch_Group();
		break;

	case DRV_OPERATION_SET_OSID:
		SEP_DRV_LOG_TRACE("LWPMUDRV_IOCTL_SET_OSID\n");
		status = lwpmudrv_Set_OSID(&local_args);
		break;

	case DRV_OPERATION_GET_AGENT_MODE:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_GET_AGENT_MODE\n");
		status = lwpmudrv_Get_Agent_Mode(&local_args);
		break;

	case DRV_OPERATION_GET_VCPU_MAP:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_GET_CPU_MAP\n");
		status = lwpmudrv_Get_Cpu_Map_Info(&local_args);
		break;

	case DRV_OPERATION_GET_NUM_VM:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_GET_NUM_VM\n");
		status = lwpmudrv_Get_Num_Of_Vms(&local_args);
		break;

		/*
		 * EMON-specific IOCTL commands
		 */
	case DRV_OPERATION_READ_MSR:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_READ_MSR.");
		status = lwpmudrv_Read_MSR_All_Cores(&local_args);
		break;

	case DRV_OPERATION_WRITE_MSR:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_WRITE_MSR.");
		status = lwpmudrv_Write_MSR_All_Cores(&local_args);
		break;

	case DRV_OPERATION_READ_SWITCH_GROUP:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_READ_SWITCH_GROUP.");
		status = lwpmudrv_Read_Counters_And_Switch_Group(&local_args);
		break;

	case DRV_OPERATION_READ_AND_RESET:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_READ_AND_RESET.");
		status = lwpmudrv_Read_And_Reset_Counters(&local_args);
		break;

		/*
		 * Platform-specific IOCTL commands (IA32 and Intel64)
		 */

	case DRV_OPERATION_INIT_UNC:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_INIT_UNC.");
		status = lwpmudrv_Initialize_UNC(local_args.buf_usr_to_drv,
						 local_args.len_usr_to_drv);
		break;

	case DRV_OPERATION_EM_GROUPS_UNC:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_EM_GROUPS_UNC.");
		status = lwpmudrv_Set_EM_Config_UNC(&local_args);
		break;

	case DRV_OPERATION_EM_CONFIG_NEXT_UNC:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_EM_CONFIG_NEXT_UNC.");
		status = lwpmudrv_Configure_Events_UNC(&local_args);
		break;

	case DRV_OPERATION_LBR_INFO:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_LBR_INFO.");
		status = lwpmudrv_LBR_Info(&local_args);
		break;

	case DRV_OPERATION_PWR_INFO:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_PWR_INFO.");
		status = lwpmudrv_PWR_Info(&local_args);
		break;

	case DRV_OPERATION_INIT_NUM_DEV:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_INIT_NUM_DEV.");
		status = lwpmudrv_Initialize_Num_Devices(&local_args);
		break;
	case DRV_OPERATION_GET_NUM_SAMPLES:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_GET_NUM_SAMPLES.");
		status = lwpmudrv_Get_Num_Samples(&local_args);
		break;

	case DRV_OPERATION_SET_DEVICE_NUM_UNITS:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_SET_DEVICE_NUM_UNITS.");
		status = lwpmudrv_Set_Device_Num_Units(&local_args);
		break;

	case DRV_OPERATION_GET_INTERVAL_COUNTS:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_GET_INTERVAL_COUNTS.");
		lwpmudrv_Get_Interval_Counts(&local_args);
		break;

	case DRV_OPERATION_SET_SCAN_UNCORE_TOPOLOGY_INFO:
		SEP_DRV_LOG_TRACE(
			"DRV_OPERATION_SET_SCAN_UNCORE_TOPOLOGY_INFO.");
		status =
			lwpmudrv_Set_Uncore_Topology_Info_And_Scan(&local_args);
		break;

	case DRV_OPERATION_GET_UNCORE_TOPOLOGY:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_GET_UNCORE_TOPOLOGY.");
		status = lwpmudrv_Get_Uncore_Topology(&local_args);
		break;

	case DRV_OPERATION_GET_PLATFORM_TOPOLOGY:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_GET_PLATFORM_TOPOLOGY.");
		status = lwpmudrv_Get_Platform_Topology(&local_args);
		break;

	case DRV_OPERATION_FLUSH:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_FLUSH.");
		status = lwpmudrv_Flush();
		break;

	case DRV_OPERATION_SET_EMON_BUFFER_DRIVER_HELPER:
		SEP_DRV_LOG_TRACE(
			"DRV_OPERATION_SET_EMON_BUFFER_DRIVER_HELPER.");
		status = lwpmudrv_Set_Emon_Buffer_Driver_Helper(&local_args);
		break;

		/*
		 * Graphics IOCTL commands
		 */

#if defined(BUILD_GFX)
	case DRV_OPERATION_SET_GFX_EVENT:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_SET_GFX_EVENT.");
		SEP_DRV_LOG_TRACE("lwpmudrv_Device_Control: enable_gfx=%d.",
				  (int)DRV_CONFIG_enable_gfx(drv_cfg));
		status = GFX_Set_Event_Code(&local_args);
		break;
#endif

		/*
		 * Chipset IOCTL commands
		 */

#if defined(BUILD_CHIPSET)
	case DRV_OPERATION_PCI_READ: {
		CHIPSET_PCI_ARG_NODE pci_data;

		SEP_DRV_LOG_TRACE("DRV_OPERATION_PCI_READ.");

		if (local_args.buf_usr_to_drv == NULL ||
		    local_args.len_usr_to_drv != sizeof(CHIPSET_PCI_ARG_NODE) ||
		    local_args.buf_drv_to_usr == NULL ||
		    local_args.len_drv_to_usr != sizeof(CHIPSET_PCI_ARG_NODE)) {
			status = OS_FAULT;
			goto cleanup;
		}

		if (copy_from_user(&pci_data,
				   (void __user *)local_args.buf_usr_to_drv,
				   sizeof(CHIPSET_PCI_ARG_NODE))) {
			status = OS_FAULT;
			goto cleanup;
		}

		status = PCI_Read_From_Memory_Address(
			CHIPSET_PCI_ARG_address(&pci_data),
			&CHIPSET_PCI_ARG_value(&pci_data));

		if (copy_to_user((void __user *)local_args.buf_drv_to_usr, &pci_data,
				 sizeof(CHIPSET_PCI_ARG_NODE))) {
			status = OS_FAULT;
			goto cleanup;
		}

		break;
	}

	case DRV_OPERATION_PCI_WRITE: {
		CHIPSET_PCI_ARG_NODE pci_data;

		SEP_DRV_LOG_TRACE("DRV_OPERATION_PCI_WRITE.");

		if (local_args.buf_usr_to_drv == NULL ||
		    local_args.len_usr_to_drv != sizeof(CHIPSET_PCI_ARG_NODE)) {
			status = OS_FAULT;
			goto cleanup;
		}

		if (copy_from_user(&pci_data,
				   (void __user *)local_args.buf_usr_to_drv,
				   sizeof(CHIPSET_PCI_ARG_NODE))) {
			status = OS_FAULT;
			goto cleanup;
		}

		status = PCI_Write_To_Memory_Address(
			CHIPSET_PCI_ARG_address(&pci_data),
			CHIPSET_PCI_ARG_value(&pci_data));
		break;
	}

	case DRV_OPERATION_FD_PHYS:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_FD_PHYS.");
		status = lwpmudrv_Samp_Find_Physical_Address(&local_args);
		break;

	case DRV_OPERATION_READ_PCI_CONFIG:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_READ_PCI_CONFIG.");
		status = lwpmudrv_Samp_Read_PCI_Config(&local_args);
		break;

	case DRV_OPERATION_WRITE_PCI_CONFIG:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_WRITE_PCI_CONFIG.");
		status = lwpmudrv_Samp_Write_PCI_Config(&local_args);
		break;

	case DRV_OPERATION_CHIPSET_INIT:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_CHIPSET_INIT.");
		SEP_DRV_LOG_TRACE("Enable_chipset=%d.",
				  (int)DRV_CONFIG_enable_chipset(drv_cfg));
		status = lwpmudrv_Samp_Chipset_Init(&local_args);
		break;

	case DRV_OPERATION_GET_CHIPSET_DEVICE_ID:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_GET_CHIPSET_DEVICE_ID.");
		status = lwpmudrv_Samp_Read_PCI_Config(&local_args);
		break;
#endif

	case DRV_OPERATION_GET_DRV_SETUP_INFO:
		SEP_DRV_LOG_TRACE("DRV_OPERATION_GET_DRV_SETUP_INFO.");
		status = lwpmudrv_Get_Drv_Setup_Info(&local_args);
		break;

		/*
		 * if none of the above, treat as unknown/illegal IOCTL command
		 */

	default:
		SEP_DRV_LOG_ERROR("Unknown IOCTL number: %d!", cmd);
		status = OS_ILLEGAL_IOCTL;
		break;
	}
#if defined(BUILD_CHIPSET)
cleanup:
#endif
	UTILITY_Driver_Set_Active_Ioctl(0);
	MUTEX_UNLOCK(ioctl_lock);

	SEP_DRV_LOG_TRACE_OUT("Return value for command %d: %d.", cmd, status);
	return status;
}

static long lwpmu_Device_Control(IOCTL_USE_INODE struct file *filp,
				 unsigned int cmd, unsigned long arg)
{
	int status = OS_SUCCESS;
	IOCTL_ARGS_NODE local_args;

	SEP_DRV_LOG_TRACE_IN("Cmd type: %d, subcommand: %d.", _IOC_TYPE(cmd),
			     _IOC_NR(cmd));

#if !defined(DRV_USE_UNLOCKED_IOCTL)
	SEP_DRV_LOG_TRACE("Cmd: 0x%x, called on inode maj:%d, min:%d.", cmd,
			  imajor(inode), iminor(inode));
#endif
	SEP_DRV_LOG_TRACE("Type: %d, subcommand: %d.", _IOC_TYPE(cmd),
			  _IOC_NR(cmd));

	if (_IOC_TYPE(cmd) != LWPMU_IOC_MAGIC) {
		SEP_DRV_LOG_ERROR_TRACE_OUT("Unknown IOCTL magic: %d!",
					    _IOC_TYPE(cmd));
		return OS_ILLEGAL_IOCTL;
	}

	memset(&local_args, 0, sizeof(IOCTL_ARGS_NODE));

	if (arg) {
		status = copy_from_user(&local_args, (void __user *)arg,
					sizeof(IOCTL_ARGS_NODE));
	}

	status = lwpmu_Service_IOCTL(IOCTL_USE_INODE filp, _IOC_NR(cmd),
				     local_args);

	SEP_DRV_LOG_TRACE_OUT("Return value: %d.", status);
	return status;
}

#if defined(CONFIG_COMPAT) && defined(DRV_EM64T)
static long lwpmu_Device_Control_Compat(struct file *filp, unsigned int cmd,
					unsigned long arg)
{
	int status = OS_SUCCESS;
	IOCTL_COMPAT_ARGS_NODE local_args_compat;
	IOCTL_ARGS_NODE local_args;

	SEP_DRV_LOG_TRACE_IN("Compat: type: %d, subcommand: %d.",
			     _IOC_TYPE(cmd), _IOC_NR(cmd));

	memset(&local_args_compat, 0, sizeof(IOCTL_COMPAT_ARGS_NODE));
	SEP_DRV_LOG_TRACE("Compat: type: %d, subcommand: %d.", _IOC_TYPE(cmd),
			  _IOC_NR(cmd));

	if (_IOC_TYPE(cmd) != LWPMU_IOC_MAGIC) {
		SEP_DRV_LOG_ERROR_TRACE_OUT("Unknown IOCTL magic: %d!",
					    _IOC_TYPE(cmd));
		return OS_ILLEGAL_IOCTL;
	}

	if (arg) {
		status = copy_from_user(&local_args_compat,
					(void __user *)arg,
					sizeof(IOCTL_COMPAT_ARGS_NODE));
	} // NB: status defined above is not being used...
	local_args.len_drv_to_usr = local_args_compat.len_drv_to_usr;
	local_args.len_usr_to_drv = local_args_compat.len_usr_to_drv;
	local_args.buf_drv_to_usr =
		(char *)compat_ptr(local_args_compat.buf_drv_to_usr);
	local_args.buf_usr_to_drv =
		(char *)compat_ptr(local_args_compat.buf_usr_to_drv);
	local_args.command = _IOC_NR(cmd);

	status = lwpmu_Service_IOCTL(filp, _IOC_NR(cmd), local_args);

	SEP_DRV_LOG_TRACE_OUT("Return value: %d", status);
	return status;
}
#endif

/*
 * @fn        LWPMUDRV_Abnormal_Terminate(void)
 *
 * @brief     This routine is called from linuxos_Exit_Task_Notify if the user process has
 *            been killed by an uncatchable signal (example kill -9).  The state variable
 *            abormal_terminate is set to 1 and the clean up routines are called.  In this
 *            code path the OS notifier hooks should not be unloaded.
 *
 * @param     None
 *
 * @return    OS_STATUS
 *
 * <I>Special Notes:</I>
 *     <none>
 */
static int LWPMUDRV_Abnormal_Terminate(void)
{
	int status;
	SEP_DRV_LOG_FLOW_IN("");

	SEP_DRV_LOG_TRACE("Calling lwpmudrv_Prepare_Stop.");
	status = lwpmudrv_Prepare_Stop();
	if (status != OS_SUCCESS)
		return status;

	SEP_DRV_LOG_TRACE("Calling lwpmudrv_Finish_Stop.");
	status = lwpmudrv_Finish_Stop();
	if (status != OS_SUCCESS)
		return status;

	SEP_DRV_LOG_TRACE("Calling lwpmudrv_Terminate.");
	status = lwpmudrv_Terminate();

	SEP_DRV_LOG_FLOW_OUT("Return value: %d.", status);
	return status;
}

static int lwpmudrv_Abnormal_Handler(void *data)
{
	SEP_DRV_LOG_FLOW_IN("");

	while (!kthread_should_stop()) {
		if (wait_event_interruptible_timeout(
			    wait_exit,
			    GET_DRIVER_STATE() == DRV_STATE_TERMINATING,
			    msecs_to_jiffies(350))) {
			SEP_DRV_LOG_WARNING(
				"Processing abnormal termination...");
			MUTEX_LOCK(ioctl_lock);
			SEP_DRV_LOG_TRACE("Locked ioctl_lock...");
			LWPMUDRV_Abnormal_Terminate();
			SEP_DRV_LOG_TRACE("Unlocking ioctl_lock...");
			MUTEX_UNLOCK(ioctl_lock);
		}
	}

	SEP_DRV_LOG_FLOW_OUT("End of thread.");
	return 0;
}

/*****************************************************************************************
 *
 *   Driver Entry / Exit functions that will be called on when the driver is loaded and
 *   unloaded
 *
 ****************************************************************************************/

/*
 * Structure that declares the usual file access functions
 * First one is for lwpmu_c, the control functions
 */
static struct file_operations lwpmu_Fops = {
	.owner = THIS_MODULE,
	IOCTL_OP = lwpmu_Device_Control,
#if defined(CONFIG_COMPAT) && defined(DRV_EM64T)
	.compat_ioctl = lwpmu_Device_Control_Compat,
#endif
	.read = lwpmu_Read,
	.write = lwpmu_Write,
	.open = lwpmu_Open,
	.release = NULL,
	.llseek = NULL,
};

/*
 * Second one is for lwpmu_m, the module notification functions
 */
static struct file_operations lwmod_Fops = {
	.owner = THIS_MODULE,
	IOCTL_OP = NULL, //None needed
	.read = OUTPUT_Module_Read,
	.write = NULL, //No writing accepted
	.open = lwpmu_Open,
	.release = NULL,
	.llseek = NULL,
};

/*
 * Third one is for lwsamp_nn, the sampling functions
 */
static struct file_operations lwsamp_Fops = {
	.owner = THIS_MODULE,
	IOCTL_OP = NULL, //None needed
	.read = OUTPUT_Sample_Read,
	.write = NULL, //No writing accepted
	.open = lwpmu_Open,
	.release = NULL,
	.llseek = NULL,
};

/*
 * Fourth one is for lwsamp_sideband, the pebs process info functions
 */
static struct file_operations lwsideband_Fops = {
	.owner = THIS_MODULE,
	IOCTL_OP = NULL, //None needed
	.read = OUTPUT_SidebandInfo_Read,
	.write = NULL, //No writing accepted
	.open = lwpmu_Open,
	.release = NULL,
	.llseek = NULL,
};

/*
 * Fifth one is for lwsampunc_nn, the uncore sampling functions
 */
static struct file_operations lwsampunc_Fops = {
	.owner = THIS_MODULE,
	IOCTL_OP = NULL, //None needed
	.read = OUTPUT_UncSample_Read,
	.write = NULL, //No writing accepted
	.open = lwpmu_Open,
	.release = NULL,
	.llseek = NULL,
};

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static int lwpmudrv_setup_cdev(dev, fops, dev_number)
 *
 * @param LWPMU_DEV               dev  - pointer to the device object
 * @param struct file_operations *fops - pointer to the file operations struct
 * @param dev_t                   dev_number - major/monor device number
 *
 * @return OS_STATUS
 *
 * @brief  Set up the device object.
 *
 * <I>Special Notes</I>
 */
static int lwpmu_setup_cdev(LWPMU_DEV dev, struct file_operations *fops,
			    dev_t dev_number)
{
	int res;
	SEP_DRV_LOG_TRACE_IN("");

	cdev_init(&LWPMU_DEV_cdev(dev), fops);
	LWPMU_DEV_cdev(dev).owner = THIS_MODULE;
	LWPMU_DEV_cdev(dev).ops = fops;

	res = cdev_add(&LWPMU_DEV_cdev(dev), dev_number, 1);

	SEP_DRV_LOG_TRACE_OUT("Return value: %d", res);
	return res;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static int lwpmu_Load(void)
 *
 * @param none
 *
 * @return STATUS
 *
 * @brief  Load the driver module into the kernel.  Set up the driver object.
 * @brief  Set up the initial state of the driver and allocate the memory
 * @brief  needed to keep basic state information.
 */
static int lwpmu_Load(void)
{
	int i, j, num_cpus;
	dev_t lwmod_DevNum;
	OS_STATUS status = OS_INVALID;
	char dev_name[MAXNAMELEN];
	struct device *sep_device;
#if defined(CONFIG_XEN_HAVE_VPMU)
	xen_pmu_params_t xenpmu_param;
	xen_pmu_data_t *xenpmu_data;
	unsigned long pfn;
#endif

	SEP_DRV_LOG_LOAD("Driver loading...");
	if (UTILITY_Driver_Log_Init() !=
	    OS_SUCCESS) { // Do not use SEP_DRV_LOG_X (where X != LOAD) before this, or if this fails
		SEP_DRV_LOG_LOAD("Error: could not allocate log buffer.");
		return OS_NO_MEM;
	}
	SEP_DRV_LOG_FLOW_IN("Starting internal log monitoring.");

	CONTROL_Memory_Tracker_Init();

#if defined(DRV_SEP_ACRN_ON)
	SEP_DRV_LOG_FLOW_IN("Starting internal log monitoring.");
	vm_info_list =
		CONTROL_Allocate_Memory(sizeof(struct profiling_vm_info_list));
	memset(vm_info_list, 0, sizeof(struct profiling_vm_info_list));

	BUG_ON(!virt_addr_valid(vm_info_list));

	acrn_hypercall2(HC_PROFILING_OPS, PROFILING_GET_VMINFO,
			virt_to_phys(vm_info_list));
#endif

#if !defined(CONFIG_XEN_HAVE_VPMU)
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 32)
	if (xen_initial_domain()) {
		SEP_DRV_LOG_LOAD(
			"PMU virtualization is not enabled on XEN dom0!");
	}
#endif
#endif

	/* Get one major device number and two minor numbers. */
	/*   The result is formatted as major+minor(0) */
	/*   One minor number is for control (lwpmu_c), */
	/*   the other (lwpmu_m) is for modules */
	SEP_DRV_LOG_INIT("About to register chrdev...");

	lwpmu_DevNum = MKDEV(0, 0);
	status = alloc_chrdev_region(&lwpmu_DevNum, 0, PMU_DEVICES,
				     SEP_DRIVER_NAME);
	SEP_DRV_LOG_INIT("Result of alloc_chrdev_region is %d.", status);
	if (status < 0) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Error: Failed to alloc chrdev_region (return = %d).",
			status);
		return status;
	}
	SEP_DRV_LOG_LOAD("Major number is %d", MAJOR(lwpmu_DevNum));
	status = lwpmudrv_Initialize_State();
	if (status < 0) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Failed to initialize state (return = %d)!", status);
		return status;
	}
	num_cpus = GLOBAL_STATE_num_cpus(driver_state);
	SEP_DRV_LOG_LOAD("Detected %d total CPUs and %d active CPUs.", num_cpus,
			 GLOBAL_STATE_active_cpus(driver_state));

#if defined(CONFIG_XEN_HAVE_VPMU)
	if (xen_initial_domain()) {
		xenpmu_param.version.maj = XENPMU_VER_MAJ;
		xenpmu_param.version.min = XENPMU_VER_MIN;

		for (i = 0; i < num_cpus; i++) {
			xenpmu_data =
				(xen_pmu_data_t *)get_zeroed_page(GFP_KERNEL);
			;
			if (!xenpmu_data) {
				SEP_DRV_LOG_ERROR_FLOW_OUT(
					"Memory allocation failure for xenpmu_data!");
				return OS_NO_MEM;
			}
			pfn = vmalloc_to_pfn((char *)xenpmu_data);

			xenpmu_param.val = pfn_to_mfn(pfn);
			xenpmu_param.vcpu = i;
			status = HYPERVISOR_xenpmu_op(XENPMU_init,
						      (PVOID)&xenpmu_param);

			per_cpu(sep_xenpmu_shared, i) = xenpmu_data;
		}
		SEP_DRV_LOG_LOAD("VPMU is initialized on XEN Dom0.");
	}
#endif

	PCI_Initialize();

	/* Allocate memory for the control structures */
	lwpmu_control = CONTROL_Allocate_Memory(sizeof(LWPMU_DEV_NODE));
	lwmod_control = CONTROL_Allocate_Memory(sizeof(LWPMU_DEV_NODE));
	lwsamp_control =
		CONTROL_Allocate_Memory(num_cpus * sizeof(LWPMU_DEV_NODE));
	lwsideband_control =
		CONTROL_Allocate_Memory(num_cpus * sizeof(LWPMU_DEV_NODE));

	if (!lwsideband_control || !lwsamp_control || !lwpmu_control ||
	    !lwmod_control) {
		CONTROL_Free_Memory(lwpmu_control);
		CONTROL_Free_Memory(lwmod_control);
		CONTROL_Free_Memory(lwsamp_control);
		CONTROL_Free_Memory(lwsideband_control);

		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory allocation failure for control structures!");
		return OS_NO_MEM;
	}

	/* Register the file operations with the OS */

	pmu_class = class_create(THIS_MODULE, SEP_DRIVER_NAME);
	if (IS_ERR(pmu_class)) {
		SEP_DRV_LOG_ERROR("Error registering SEP control class!");
	}
	sep_device = device_create(pmu_class, NULL, lwpmu_DevNum, NULL,
		      SEP_DRIVER_NAME DRV_DEVICE_DELIMITER "c");
	if (IS_ERR(sep_device)) {
		SEP_DRV_LOG_ERROR("Error creating SEP PMU device!");
	}

	status = lwpmu_setup_cdev(lwpmu_control, &lwpmu_Fops, lwpmu_DevNum);
	if (status) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Error %d when adding lwpmu as char device!", status);
		unregister_chrdev(MAJOR(lwpmu_DevNum), SEP_DRIVER_NAME);
		device_destroy(pmu_class, lwpmu_DevNum);
		unregister_chrdev_region(lwpmu_DevNum, PMU_DEVICES);
		class_destroy(pmu_class);
		return status;
	}
	/* _c init was fine, now try _m */
	lwmod_DevNum = MKDEV(MAJOR(lwpmu_DevNum), MINOR(lwpmu_DevNum) + 1);

	sep_device = device_create(pmu_class, NULL, lwmod_DevNum, NULL,
		      SEP_DRIVER_NAME DRV_DEVICE_DELIMITER "m");
	if (IS_ERR(sep_device)) {
		SEP_DRV_LOG_ERROR("Error creating SEP PMU device!");
	}

	status = lwpmu_setup_cdev(lwmod_control, &lwmod_Fops, lwmod_DevNum);
	if (status) {
		cdev_del(&LWPMU_DEV_cdev(lwpmu_control));
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Error %d when adding lwpmu as char device!", status);
		unregister_chrdev(MAJOR(lwpmu_DevNum), SEP_DRIVER_NAME);
		device_destroy(pmu_class, lwpmu_DevNum);
		device_destroy(pmu_class, lwpmu_DevNum + 1);
		cdev_del(&LWPMU_DEV_cdev(lwpmu_control));
		unregister_chrdev_region(lwpmu_DevNum, PMU_DEVICES);
		class_destroy(pmu_class);
		return status;
	}

	/* allocate one sampling device per cpu */
	lwsamp_DevNum = MKDEV(0, 0);
	status = alloc_chrdev_region(&lwsamp_DevNum, 0, num_cpus,
				     SEP_SAMPLES_NAME);

	if (status < 0) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Error: Failed to alloc chrdev_region (return = %d).",
			status);
		unregister_chrdev(MAJOR(lwpmu_DevNum), SEP_DRIVER_NAME);
		device_destroy(pmu_class, lwpmu_DevNum);
		device_destroy(pmu_class, lwpmu_DevNum + 1);
		cdev_del(&LWPMU_DEV_cdev(lwpmu_control));
		cdev_del(&LWPMU_DEV_cdev(lwmod_control));
		unregister_chrdev_region(lwpmu_DevNum, PMU_DEVICES);
		class_destroy(pmu_class);
		return status;
	}

	/* Register the file operations with the OS */
	for (i = 0; i < num_cpus; i++) {
		snprintf(dev_name, MAXNAMELEN, "%s%ss%d", SEP_DRIVER_NAME,
			 DRV_DEVICE_DELIMITER, i);

		sep_device = device_create(pmu_class, NULL, lwsamp_DevNum + i, NULL,
			      dev_name);
		if (IS_ERR(sep_device)) {
			SEP_DRV_LOG_ERROR("Error creating SEP PMU device !");
		}
		status = lwpmu_setup_cdev(lwsamp_control + i, &lwsamp_Fops,
					  lwsamp_DevNum + i);
		if (status) {
			SEP_DRV_LOG_ERROR_FLOW_OUT(
				"Error %d when adding lwpmu as char device!",
				status);
			unregister_chrdev(MAJOR(lwpmu_DevNum), SEP_DRIVER_NAME);
			device_destroy(pmu_class, lwpmu_DevNum);
			device_destroy(pmu_class, lwpmu_DevNum + 1);
			cdev_del(&LWPMU_DEV_cdev(lwpmu_control));
			cdev_del(&LWPMU_DEV_cdev(lwmod_control));
			unregister_chrdev_region(lwpmu_DevNum, PMU_DEVICES);
			unregister_chrdev(MAJOR(lwsamp_DevNum), SEP_SAMPLES_NAME);
			for (j = i; j > 0; j--) {
				device_destroy(pmu_class, lwsamp_DevNum + j);
				cdev_del(&LWPMU_DEV_cdev(&lwsamp_control[j]));
			}

			class_destroy(pmu_class);
			return status;
		} else {
			SEP_DRV_LOG_INIT("Added sampling device %d.", i);
		}
	}

	lwsideband_DevNum = MKDEV(0, 0);
	status = alloc_chrdev_region(&lwsideband_DevNum, 0, num_cpus,
				     SEP_SIDEBAND_NAME);

	if (status < 0) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory allocation failure for chrdev_region for sideband!");
		unregister_chrdev(MAJOR(lwpmu_DevNum), SEP_DRIVER_NAME);
		device_destroy(pmu_class, lwpmu_DevNum);
		device_destroy(pmu_class, lwpmu_DevNum + 1);
		cdev_del(&LWPMU_DEV_cdev(lwpmu_control));
		cdev_del(&LWPMU_DEV_cdev(lwmod_control));
		unregister_chrdev_region(lwpmu_DevNum, PMU_DEVICES);
		unregister_chrdev(MAJOR(lwsamp_DevNum), SEP_SAMPLES_NAME);
		for (j = 0; j < num_cpus; j++) {
			device_destroy(pmu_class, lwsamp_DevNum + j);
			cdev_del(&LWPMU_DEV_cdev(&lwsamp_control[j]));
		}

		class_destroy(pmu_class);
		return status;
	}

	for (i = 0; i < num_cpus; i++) {
		snprintf(dev_name, MAXNAMELEN, "%s%sb%d", SEP_DRIVER_NAME,
			 DRV_DEVICE_DELIMITER, i);
		sep_device = device_create(pmu_class, NULL, lwsideband_DevNum + i, NULL,
			      dev_name);
		if (IS_ERR(sep_device)) {
			SEP_DRV_LOG_ERROR("Error creating SEP PMU device!");
		}
		status = lwpmu_setup_cdev(lwsideband_control + i,
					  &lwsideband_Fops,
					  lwsideband_DevNum + i);
		if (status) {
			SEP_DRV_LOG_ERROR_FLOW_OUT(
				"Error %d when adding lwsideband as char device!",
				status);
			unregister_chrdev(MAJOR(lwpmu_DevNum), SEP_DRIVER_NAME);
			device_destroy(pmu_class, lwpmu_DevNum);
			device_destroy(pmu_class, lwpmu_DevNum + 1);
			cdev_del(&LWPMU_DEV_cdev(lwpmu_control));
			cdev_del(&LWPMU_DEV_cdev(lwmod_control));
			unregister_chrdev_region(lwpmu_DevNum, PMU_DEVICES);
			unregister_chrdev(MAJOR(lwsamp_DevNum), SEP_SAMPLES_NAME);
			unregister_chrdev(MAJOR(lwsideband_DevNum), SEP_SIDEBAND_NAME);
			for (j = 0; j < num_cpus; j++) {
				device_destroy(pmu_class, lwsamp_DevNum + j);
				cdev_del(&LWPMU_DEV_cdev(&lwsamp_control[j]));
			}
			for (j = i; j > 0; j--) {
				device_destroy(pmu_class, lwsideband_DevNum + j);
				cdev_del(&LWPMU_DEV_cdev(&lwsideband_control[j]));
			}

			class_destroy(pmu_class);
			return status;
		} else {
			SEP_DRV_LOG_INIT("Added sampling sideband device %d.",
					 i);
		}
	}

	cpu_tsc = (U64 *)CONTROL_Allocate_Memory(
		GLOBAL_STATE_num_cpus(driver_state) * sizeof(U64));
	prev_cpu_tsc = (U64 *)CONTROL_Allocate_Memory(
		GLOBAL_STATE_num_cpus(driver_state) * sizeof(U64));
	diff_cpu_tsc = (U64 *)CONTROL_Allocate_Memory(
		GLOBAL_STATE_num_cpus(driver_state) * sizeof(U64));

#if !defined(CONFIG_PREEMPT_COUNT) && !defined(DRV_SEP_ACRN_ON)
	atomic_set(&read_now, GLOBAL_STATE_num_cpus(driver_state));
	init_waitqueue_head(&read_tsc_now);
	CONTROL_Invoke_Parallel(lwpmudrv_Fill_TSC_Info, (PVOID)(size_t)0);
#endif

	pcb_size = GLOBAL_STATE_num_cpus(driver_state) * sizeof(CPU_STATE_NODE);
	pcb = CONTROL_Allocate_Memory(pcb_size);
	if (!pcb) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory allocation failure for PCB!");
		return OS_NO_MEM;
	}

	core_to_package_map = CONTROL_Allocate_Memory(
		GLOBAL_STATE_num_cpus(driver_state) * sizeof(U32));
	if (!core_to_package_map) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory allocation failure for core_to_package_map!");
		return OS_NO_MEM;
	}

	core_to_phys_core_map = CONTROL_Allocate_Memory(
		GLOBAL_STATE_num_cpus(driver_state) * sizeof(U32));
	if (!core_to_phys_core_map) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory allocation failure for core_to_phys_core_map!");
		return OS_NO_MEM;
	}

	core_to_thread_map = CONTROL_Allocate_Memory(
		GLOBAL_STATE_num_cpus(driver_state) * sizeof(U32));
	if (!core_to_thread_map) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory allocation failure for core_to_thread_map!");
		return OS_NO_MEM;
	}

	threads_per_core = CONTROL_Allocate_Memory(
		GLOBAL_STATE_num_cpus(driver_state) * sizeof(U32));
	if (!threads_per_core) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory allocation failure for threads_per_core!");
		return OS_NO_MEM;
	}

	occupied_core_ids = CONTROL_Allocate_Memory(
		GLOBAL_STATE_num_cpus(driver_state) * sizeof(U32));
	if (!occupied_core_ids) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Memory allocation failure for occupied_core_ids!");
		return OS_NO_MEM;
	}
	SYS_INFO_Build();
	memset(pcb, 0, pcb_size);

	if (total_ram <= OUTPUT_MEMORY_THRESHOLD) {
		output_buffer_size = OUTPUT_SMALL_BUFFER;
	}

	MUTEX_INIT(ioctl_lock);

	status = UNC_COMMON_Init();
	if (status) {
		SEP_DRV_LOG_ERROR_FLOW_OUT("Error %d when init uncore struct!",
					   status);
		return status;
	}

	/* allocate one sampling device per package (for uncore)*/
	lwsampunc_control =
		CONTROL_Allocate_Memory(num_packages * sizeof(LWPMU_DEV_NODE));
	if (!lwsampunc_control) {
		CONTROL_Free_Memory(lwsampunc_control);
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"lwpmu driver failed to alloc space!\n");
		return OS_NO_MEM;
	}

	lwsampunc_DevNum = MKDEV(0, 0);
	status = alloc_chrdev_region(&lwsampunc_DevNum, 0, num_packages,
				     SEP_UNCORE_NAME);

	if (status < 0) {
		SEP_DRV_LOG_ERROR_FLOW_OUT(
			"Error: Failed to alloc chrdev_region (return = %d).",
			status);
		unregister_chrdev(MAJOR(lwpmu_DevNum), SEP_DRIVER_NAME);
		device_destroy(pmu_class, lwpmu_DevNum);
		device_destroy(pmu_class, lwpmu_DevNum + 1);
		cdev_del(&LWPMU_DEV_cdev(lwpmu_control));
		cdev_del(&LWPMU_DEV_cdev(lwmod_control));
		unregister_chrdev_region(lwpmu_DevNum, PMU_DEVICES);
		unregister_chrdev(MAJOR(lwsamp_DevNum), SEP_SAMPLES_NAME);
		unregister_chrdev(MAJOR(lwsampunc_DevNum), SEP_UNCORE_NAME);
		unregister_chrdev(MAJOR(lwsideband_DevNum), SEP_SIDEBAND_NAME);
		for (j = 0; j < num_cpus; j++) {
			device_destroy(pmu_class, lwsamp_DevNum + j);
			device_destroy(pmu_class, lwsideband_DevNum + j);
			cdev_del(&LWPMU_DEV_cdev(&lwsideband_control[j]));
			cdev_del(&LWPMU_DEV_cdev(&lwsamp_control[j]));
		}

		class_destroy(pmu_class);
		unregister_chrdev_region(lwsamp_DevNum, num_cpus);
		unregister_chrdev_region(lwsideband_DevNum, num_cpus);
		return status;
	}

	/* Register the file operations with the OS */
	for (i = 0; i < num_packages; i++) {
		snprintf(dev_name, MAXNAMELEN, "%s%su%d", SEP_DRIVER_NAME,
			 DRV_DEVICE_DELIMITER, i);
		sep_device = device_create(pmu_class, NULL,
			lwsampunc_DevNum + i, NULL, dev_name);
		if (IS_ERR(sep_device)) {
			SEP_DRV_LOG_ERROR("Error creating SEP PMU device!");
		}
		status = lwpmu_setup_cdev(lwsampunc_control + i,
					&lwsampunc_Fops,
					lwsampunc_DevNum + i);
		if (status) {
			SEP_DRV_LOG_ERROR_FLOW_OUT(
				"Error %d when adding lwpmu as char device!",
				status);
			unregister_chrdev(MAJOR(lwpmu_DevNum), SEP_DRIVER_NAME);
			device_destroy(pmu_class, lwpmu_DevNum);
			device_destroy(pmu_class, lwpmu_DevNum + 1);
			cdev_del(&LWPMU_DEV_cdev(lwpmu_control));
			cdev_del(&LWPMU_DEV_cdev(lwmod_control));
			unregister_chrdev_region(lwpmu_DevNum, PMU_DEVICES);
			unregister_chrdev(MAJOR(lwsamp_DevNum), SEP_SAMPLES_NAME);
			unregister_chrdev(MAJOR(lwsampunc_DevNum), SEP_UNCORE_NAME);
			unregister_chrdev(MAJOR(lwsideband_DevNum), SEP_SIDEBAND_NAME);
			for (j = 0; j < num_cpus; j++) {
				device_destroy(pmu_class, lwsamp_DevNum + j);
				device_destroy(pmu_class, lwsideband_DevNum + j);
				cdev_del(&LWPMU_DEV_cdev(&lwsideband_control[j]));
				cdev_del(&LWPMU_DEV_cdev(&lwsamp_control[j]));
			}

			for (j = i; j > 0; j--) {
				device_destroy(pmu_class, lwsampunc_DevNum + i);
				cdev_del(&LWPMU_DEV_cdev(&lwsampunc_control[j]));
			}

			class_destroy(pmu_class);
			unregister_chrdev_region(lwsamp_DevNum, num_cpus);
			unregister_chrdev_region(lwsampunc_DevNum, num_packages);
			unregister_chrdev_region(lwsideband_DevNum, num_cpus);

			return status;
		} else {
			SEP_DRV_LOG_INIT("Added sampling device %d.", i);
		}
	}

	init_waitqueue_head(&wait_exit);
	abnormal_handler = kthread_create(lwpmudrv_Abnormal_Handler, NULL,
					  "SEPDRV_ABNORMAL_HANDLER");
	if (abnormal_handler) {
		wake_up_process(abnormal_handler);
	}

#if defined(DRV_CPU_HOTPLUG)
	/* Register CPU hotplug notifier */
	LINUXOS_Register_Hotplug();
#endif
	/*
	 *  Initialize the SEP driver version (done once at driver load time)
	 */
	SEP_VERSION_NODE_major(&drv_version) = SEP_MAJOR_VERSION;
	SEP_VERSION_NODE_minor(&drv_version) = SEP_MINOR_VERSION;
	SEP_VERSION_NODE_api(&drv_version) = SEP_API_VERSION;
	SEP_VERSION_NODE_update(&drv_version) = SEP_UPDATE_VERSION;

	//
	// Display driver version information
	//
	SEP_DRV_LOG_LOAD("PMU collection driver v%d.%d.%d %s has been loaded.",
			 SEP_VERSION_NODE_major(&drv_version),
			 SEP_VERSION_NODE_minor(&drv_version),
			 SEP_VERSION_NODE_api(&drv_version),
			 SEP_RELEASE_STRING);

#if defined(BUILD_CHIPSET)
	SEP_DRV_LOG_LOAD("Chipset support is enabled.");
#endif

#if defined(BUILD_GFX)
	SEP_DRV_LOG_LOAD("Graphics support is enabled.");
#endif

	SEP_DRV_LOG_LOAD("NMI will be used for handling PMU interrupts.");

	SEP_DRV_LOG_FLOW_OUT("Return value: %d.", status);
	return status;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  static int lwpmu_Unload(void)
 *
 * @param none
 *
 * @return none
 *
 * @brief  Remove the driver module from the kernel.
 */
static VOID lwpmu_Unload(void)
{
	int i = 0;
	int num_cpus;
#if defined(CONFIG_XEN_HAVE_VPMU)
	xen_pmu_params_t xenpmu_param;
#endif
	PVOID tmp_pcb;

	SEP_DRV_LOG_FLOW_IN("");

	SEP_DRV_LOG_LOAD("Driver unloading.");

	num_cpus = GLOBAL_STATE_num_cpus(driver_state);

	if (abnormal_handler) {
		if (GET_DRIVER_STATE() != DRV_STATE_UNINITIALIZED) {
			CHANGE_DRIVER_STATE(STATE_BIT_ANY,
					    DRV_STATE_TERMINATING);
		}
		wake_up_interruptible_all(&wait_exit);
		kthread_stop(abnormal_handler);
		abnormal_handler = NULL;
	}

#if defined(CONFIG_XEN_HAVE_VPMU)
	if (xen_initial_domain()) {
		xenpmu_param.version.maj = XENPMU_VER_MAJ;
		xenpmu_param.version.min = XENPMU_VER_MIN;

		for (i = 0; i < num_cpus; i++) {
			xenpmu_param.vcpu = i;
			HYPERVISOR_xenpmu_op(XENPMU_finish, &xenpmu_param);

			vfree(per_cpu(sep_xenpmu_shared, i));
			per_cpu(sep_xenpmu_shared, i) = NULL;
		}
		SEP_DRV_LOG_LOAD("VPMU was disabled on XEN Dom0.");
	}
#endif

	LINUXOS_Uninstall_Hooks();
	SYS_INFO_Destroy();
	OUTPUT_Destroy();
	cpu_buf = CONTROL_Free_Memory(cpu_buf);
	unc_buf = CONTROL_Free_Memory(unc_buf);
	cpu_sideband_buf = CONTROL_Free_Memory(cpu_sideband_buf);
	module_buf = CONTROL_Free_Memory(module_buf);
	cpu_tsc = CONTROL_Free_Memory(cpu_tsc);
	prev_cpu_tsc = CONTROL_Free_Memory(prev_cpu_tsc);
	diff_cpu_tsc = CONTROL_Free_Memory(diff_cpu_tsc);
	core_to_package_map = CONTROL_Free_Memory(core_to_package_map);
	core_to_phys_core_map = CONTROL_Free_Memory(core_to_phys_core_map);
	core_to_thread_map = CONTROL_Free_Memory(core_to_thread_map);
	threads_per_core = CONTROL_Free_Memory(threads_per_core);
	occupied_core_ids = CONTROL_Free_Memory(occupied_core_ids);
#if defined(DRV_SEP_ACRN_ON)
	vm_info_list = CONTROL_Free_Memory(vm_info_list);
#endif

	tmp_pcb = pcb;
	// Ensures there is no log message written (ERROR, ALLOC, ...)
	pcb = NULL; // between pcb being freed and pcb being NULL.
	CONTROL_Free_Memory(tmp_pcb);
	pcb_size = 0;

	UNC_COMMON_Clean_Up();

	unregister_chrdev(MAJOR(lwpmu_DevNum), SEP_DRIVER_NAME);
	device_destroy(pmu_class, lwpmu_DevNum);
	device_destroy(pmu_class, lwpmu_DevNum + 1);

	cdev_del(&LWPMU_DEV_cdev(lwpmu_control));
	cdev_del(&LWPMU_DEV_cdev(lwmod_control));
	unregister_chrdev_region(lwpmu_DevNum, PMU_DEVICES);

	unregister_chrdev(MAJOR(lwsamp_DevNum), SEP_SAMPLES_NAME);
	unregister_chrdev(MAJOR(lwsampunc_DevNum), SEP_UNCORE_NAME);
	unregister_chrdev(MAJOR(lwsideband_DevNum), SEP_SIDEBAND_NAME);

	for (i = 0; i < num_cpus; i++) {
		device_destroy(pmu_class, lwsamp_DevNum + i);
		device_destroy(pmu_class, lwsideband_DevNum + i);
		cdev_del(&LWPMU_DEV_cdev(&lwsamp_control[i]));
		cdev_del(&LWPMU_DEV_cdev(&lwsideband_control[i]));
	}

	for (i = 0; i < num_packages; i++) {
		device_destroy(pmu_class, lwsampunc_DevNum + i);
		cdev_del(&LWPMU_DEV_cdev(&lwsampunc_control[i]));
	}

	class_destroy(pmu_class);

	unregister_chrdev_region(lwsamp_DevNum, num_cpus);
	unregister_chrdev_region(lwsampunc_DevNum, num_packages);
	unregister_chrdev_region(lwsideband_DevNum, num_cpus);
	lwpmu_control = CONTROL_Free_Memory(lwpmu_control);
	lwmod_control = CONTROL_Free_Memory(lwmod_control);
	lwsamp_control = CONTROL_Free_Memory(lwsamp_control);
	lwsampunc_control = CONTROL_Free_Memory(lwsampunc_control);
	lwsideband_control = CONTROL_Free_Memory(lwsideband_control);

	CONTROL_Memory_Tracker_Free();

#if defined(DRV_CPU_HOTPLUG)
	/* Unregister CPU hotplug notifier */
	LINUXOS_Unregister_Hotplug();
#endif

	SEP_DRV_LOG_FLOW_OUT(
		"Log deallocation. Cannot track further in internal log.");
	UTILITY_Driver_Log_Free(); // Do not use SEP_DRV_LOG_X (where X != LOAD) after this

	SEP_DRV_LOG_LOAD(
		"PMU collection driver v%d.%d.%d %s has been unloaded.",
		SEP_VERSION_NODE_major(&drv_version),
		SEP_VERSION_NODE_minor(&drv_version),
		SEP_VERSION_NODE_api(&drv_version), SEP_RELEASE_STRING);
}

/* Declaration of the init and exit functions */
module_init(lwpmu_Load);
module_exit(lwpmu_Unload);
