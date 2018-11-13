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

#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/profile.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/delay.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0)
#include <linux/sched/mm.h>
#include <linux/sched/signal.h>
#else
#include <linux/sched.h>
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
#include <linux/cpuhotplug.h>
#endif
#include <linux/fs.h>
#include <linux/cpu.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 28)
#include <trace/events/sched.h>
#endif

#include "lwpmudrv_types.h"
#include "lwpmudrv_ecb.h"
#include "lwpmudrv_struct.h"
#include "inc/lwpmudrv.h"
#include "inc/control.h"
#include "inc/utility.h"
#include "inc/cpumon.h"
#include "inc/output.h"
#include "inc/pebs.h"

#include "inc/linuxos.h"
#include "inc/apic.h"

#include <asm/apic.h>
#include <asm/io_apic.h>


extern DRV_BOOL multi_pebs_enabled;
extern DRV_BOOL sched_switch_enabled;
extern uid_t uid;
extern volatile pid_t control_pid;
static volatile S32 hooks_installed;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 15, 0)
static struct tracepoint *tp_sched_switch;
#endif

#define HOOK_FREE 0
#define HOOK_UNINSTALL -10000
static atomic_t hook_state = ATOMIC_INIT(HOOK_UNINSTALL);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0) && defined(DRV_CPU_HOTPLUG)
static enum cpuhp_state cpuhp_sepdrv_state;
#endif
extern wait_queue_head_t wait_exit;

static PVOID local_tasklist_lock;

#define MY_TASK PROFILE_TASK_EXIT
#define MY_UNMAP PROFILE_MUNMAP
#ifdef CONFIG_X86_64
#define MR_SEG_NUM 0
#else
#define MR_SEG_NUM 2
#endif

#if !defined(KERNEL_IMAGE_SIZE)
#define KERNEL_IMAGE_SIZE (512 * 1024 * 1024)
#endif

#if defined(DRV_IA32)
static U16 linuxos_Get_Exec_Mode(struct task_struct *p)
{
	return (unsigned short)MODE_32BIT;
}
#endif

#if defined(DRV_EM64T)
static U16 linuxos_Get_Exec_Mode(struct task_struct *p)
{
	SEP_DRV_LOG_TRACE_IN("P: %p.", p);

	if (!p) {
		SEP_DRV_LOG_ERROR_TRACE_OUT("MODE_UNKNOWN (p is NULL!).");
		return MODE_UNKNOWN;
	}

	if (test_tsk_thread_flag(p, TIF_IA32)) {
		SEP_DRV_LOG_TRACE_OUT(
			"Res: %u (test_tsk_thread_flag TIF_IA32).",
			(U16)(unsigned short)MODE_32BIT);
		return (unsigned short)MODE_32BIT;
	}

	SEP_DRV_LOG_TRACE_OUT("Res: %u.", (U16)(unsigned short)MODE_64BIT);
	return (unsigned short)MODE_64BIT;
}
#endif

static S32 linuxos_Load_Image_Notify_Routine(char *name, U64 base, U32 size,
					     U64 page_offset, U32 pid,
					     U32 parent_pid, U32 options,
					     unsigned short mode,
					     S32 load_event, U32 segment_num,
					     U32 kernel_modules, U32 osid)
{
	char *raw_path;
	ModuleRecord *mra;
	char buf[sizeof(ModuleRecord) + MAXNAMELEN + 32];
	U64 tsc_read;
	S32 local_load_event = (load_event == -1) ? 0 : load_event;
	U64 page_offset_shift;

	SEP_DRV_LOG_NOTIFICATION_TRACE_IN(load_event == 1,
					  "Name: '%s', pid: %d.", name, pid);

	mra = (ModuleRecord *)buf;
	memset(mra, '\0', sizeof(buf));
	raw_path = (char *)mra + sizeof(ModuleRecord);

	page_offset_shift = page_offset << PAGE_SHIFT;
	MR_page_offset_Set(mra, page_offset_shift);
	MODULE_RECORD_segment_type(mra) = mode;
	MODULE_RECORD_load_addr64(mra) = (U64)(size_t)base;
	MODULE_RECORD_length64(mra) = size;
	MODULE_RECORD_tsc_used(mra) = 1;
	MODULE_RECORD_first_module_rec_in_process(mra) =
		options & LOPTS_1ST_MODREC;
	MODULE_RECORD_segment_number(mra) = segment_num;
	MODULE_RECORD_exe(mra) = (LOPTS_EXE & options) ? 1 : 0;
	MODULE_RECORD_global_module_tb5(mra) =
		(options & LOPTS_GLOBAL_MODULE) ? 1 : 0;
	MODULE_RECORD_global_module(mra) =
		(options & LOPTS_GLOBAL_MODULE) ? 1 : 0;
	MODULE_RECORD_processed(mra) = 0;
	MODULE_RECORD_parent_pid(mra) = parent_pid;
	MODULE_RECORD_osid(mra) = osid;
	MODULE_RECORD_pid_rec_index(mra) = pid;

	if (kernel_modules) {
		MODULE_RECORD_tsc(mra) = 0;
		MR_unloadTscSet(mra, (U64)(0xffffffffffffffffLL));
	} else {
		UTILITY_Read_TSC(&tsc_read);
		preempt_disable();
		tsc_read -= TSC_SKEW(CONTROL_THIS_CPU());
		preempt_enable();

		if (local_load_event) {
			MR_unloadTscSet(mra, tsc_read);
		} else {
			MR_unloadTscSet(mra, (U64)(-1));
		}
	}

	MODULE_RECORD_pid_rec_index_raw(mra) = 1; // raw pid
#if defined(DEBUG)
	if (total_loads_init) {
		SEP_DRV_LOG_NOTIFICATION_TRACE(
			load_event == 1,
			"Setting pid_rec_index_raw pid 0x%x %s.", pid, name);
	}
#endif

	strncpy(raw_path, name, MAXNAMELEN);
	raw_path[MAXNAMELEN] = 0;
	MODULE_RECORD_path_length(mra) = (U16)strlen(raw_path) + 1;
	MODULE_RECORD_rec_length(mra) = (U16)ALIGN_8(
		sizeof(ModuleRecord) + MODULE_RECORD_path_length(mra));

#if defined(DRV_IA32)
	MODULE_RECORD_selector(mra) = (pid == 0) ? __KERNEL_CS : __USER_CS;
#endif
#if defined(DRV_EM64T)
	if (mode == MODE_64BIT) {
		MODULE_RECORD_selector(mra) =
			(pid == 0) ? __KERNEL_CS : __USER_CS;
	} else if (mode == MODE_32BIT) {
		MODULE_RECORD_selector(mra) =
			(pid == 0) ? __KERNEL32_CS : __USER32_CS;
	}
#endif

	OUTPUT_Module_Fill((PVOID)mra, MODULE_RECORD_rec_length(mra),
			   load_event == 1);

	SEP_DRV_LOG_NOTIFICATION_TRACE_OUT(load_event == 1, "OS_SUCCESS");
	return OS_SUCCESS;
}

#ifdef DRV_MM_EXE_FILE_PRESENT
static DRV_BOOL linuxos_Equal_VM_Exe_File(struct vm_area_struct *vma)
{
	S8 name_vm_file[MAXNAMELEN];
	S8 name_exe_file[MAXNAMELEN];
	S8 *pname_vm_file = NULL;
	S8 *pname_exe_file = NULL;
	DRV_BOOL res;

	SEP_DRV_LOG_TRACE_IN("FMA: %p.", vma);

	if (vma == NULL) {
		SEP_DRV_LOG_TRACE_OUT("FALSE (!vma).");
		return FALSE;
	}

	if (vma->vm_file == NULL) {
		SEP_DRV_LOG_TRACE_OUT("FALSE (!vma->vm_file).");
		return FALSE;
	}

	if (vma->vm_mm->exe_file == NULL) {
		SEP_DRV_LOG_TRACE_OUT("FALSE (!vma->vm_mm->exe_file).");
		return FALSE;
	}

	pname_vm_file = D_PATH(vma->vm_file,
			name_vm_file, MAXNAMELEN);
	pname_exe_file = D_PATH(vma->vm_mm->exe_file,
			name_exe_file, MAXNAMELEN);
	res = strcmp(pname_vm_file, pname_exe_file) == 0;

	SEP_DRV_LOG_TRACE_OUT("Res: %u.", res);
	return res;
}
#endif

/* ------------------------------------------------------------------------- */
/*!
 * @fn          linuxos_Map_Kernel_Modules (void)
 *
 * @brief       Obtain kernel module details from modules list
 *              and map the details to the module record.
 *
 * @return      S32       VT_SUCCESS on success
 */
static S32 linuxos_Map_Kernel_Modules(void)
{
#if defined(CONFIG_MODULES)
	struct module *current_module;
	struct list_head *modules;
	U16 exec_mode;
	unsigned long long addr;
	unsigned long long size;
#if defined(CONFIG_RANDOMIZE_BASE)
	unsigned long dyn_addr = 0;
#endif

	SEP_DRV_LOG_TRACE_IN("");


	mutex_lock(&module_mutex);

#if defined(DRV_EM64T)
	addr = (unsigned long)__START_KERNEL_map;
	exec_mode = MODE_64BIT;
#elif defined(DRV_IA32)
	addr = (unsigned long)PAGE_OFFSET;
	exec_mode = MODE_32BIT;
#else
	exec_mode = MODE_UNKNOWN;
#endif

	SEP_DRV_LOG_TRACE(
		"     kernel module            address           size");
	SEP_DRV_LOG_TRACE(
		"  -------------------    ------------------    -------");

	addr += (CONFIG_PHYSICAL_START + (CONFIG_PHYSICAL_ALIGN - 1)) &
		~(CONFIG_PHYSICAL_ALIGN - 1);
	size = (unsigned long)KERNEL_IMAGE_SIZE -
	       ((CONFIG_PHYSICAL_START + (CONFIG_PHYSICAL_ALIGN - 1)) &
		~(CONFIG_PHYSICAL_ALIGN - 1)) -
	       1;

#if defined(CONFIG_RANDOMIZE_BASE)
	if (!dyn_addr) {
		dyn_addr = (unsigned long)UTILITY_Find_Symbol("_text");
		if (!dyn_addr) {
			dyn_addr = (unsigned long)UTILITY_Find_Symbol("_stext");
		}

		if (dyn_addr && dyn_addr > addr) {
			dyn_addr &= ~(PAGE_SIZE - 1);
			size -= (dyn_addr - addr);
			addr = dyn_addr;
		} else {
			SEP_DRV_LOG_WARNING_TRACE_OUT(
				"Could not find the kernel start address!");
		}
	}
#endif

	linuxos_Load_Image_Notify_Routine(
		"vmlinux", addr, size, 0, 0, 0,
		LOPTS_1ST_MODREC | LOPTS_GLOBAL_MODULE | LOPTS_EXE, exec_mode,
		-1, MR_SEG_NUM, 1, OS_ID_NATIVE);

	SEP_DRV_LOG_TRACE("kmodule: %20s    0x%llx    0x%llx.", "vmlinux", addr,
			  size);

#if defined(DRV_SEP_ACRN_ON)
	linuxos_Load_Image_Notify_Routine(
		"VMM", 0x0, (U32)0xffffffffffffffffLL, 0, 0, 0,
		LOPTS_1ST_MODREC | LOPTS_GLOBAL_MODULE | LOPTS_EXE, exec_mode,
		-1, MR_SEG_NUM, 1, OS_ID_ACORN);
#endif

	for (modules = (struct list_head *)(THIS_MODULE->list.prev);
	     (unsigned long)modules > MODULES_VADDR; modules = modules->prev)
		;
	list_for_each_entry (current_module, modules, list) {
		char *name = current_module->name;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0) ||                           \
	defined(SEP_CONFIG_MODULE_LAYOUT)
		addr = (unsigned long)current_module->core_layout.base;
		size = current_module->core_layout.size;
#else
		addr = (unsigned long)current_module->module_core;
		size = current_module->core_size;
#endif

		if (module_is_live(current_module)) {
			SEP_DRV_LOG_TRACE("kmodule: %20s    0x%llx    0x%llx.",
					  name, addr, size);
			linuxos_Load_Image_Notify_Routine(
				name, addr, size, 0, 0, 0, LOPTS_GLOBAL_MODULE,
				exec_mode, -1, 0, 1, OS_ID_NATIVE);
		}
	}

	mutex_unlock(&module_mutex);
#endif
	SEP_DRV_LOG_TRACE_OUT("OS_SUCCESS");
	return OS_SUCCESS;
}

//
// Register the module for a process.  The task_struct and mm
// should be locked if necessary to make sure they don't change while we're
// iterating...
// Used as a service routine
//
static S32 linuxos_VMA_For_Process(struct task_struct *p,
				   struct vm_area_struct *vma, S32 load_event,
				   U32 *first)
{
	U32 options = 0;
	S8 name[MAXNAMELEN];
	S8 *pname = NULL;
	U32 ppid = 0;
	U16 exec_mode;
	U64 page_offset = 0;

#if defined(DRV_ANDROID)
	char andr_app[TASK_COMM_LEN];
#endif

	SEP_DRV_LOG_NOTIFICATION_TRACE_IN(
		load_event == 1, "P = %p, vma = %p, load_event: %d, first: %p.",
		p, vma, load_event, first);

	if (p == NULL) {
		SEP_DRV_LOG_NOTIFICATION_ERROR(load_event == 1,
					       "Skipped p=NULL.");
		SEP_DRV_LOG_NOTIFICATION_TRACE_OUT(load_event == 1,
						   "OS_SUCCESS (!p).");
		return OS_SUCCESS;
	}

	if (vma->vm_file)
		pname = D_PATH(vma->vm_file, name, MAXNAMELEN);

	page_offset = vma->vm_pgoff;

	if (!IS_ERR(pname) && pname != NULL) {
		SEP_DRV_LOG_NOTIFICATION_TRACE(load_event == 1,
					       "enum: %s, %d, %lx, %lx %llu.",
					       pname, p->pid, vma->vm_start,
					       (vma->vm_end - vma->vm_start),
					       page_offset);

		// if the VM_EXECUTABLE flag is set then this is the module
		// that is being used to name the module
		if (DRV_VM_MOD_EXECUTABLE(vma)) {
			options |= LOPTS_EXE;
#if defined(DRV_ANDROID)
			if (!strcmp(pname, "/system/bin/app_process") ||
			    !strcmp(pname, "/system/bin/app_process32") ||
			    !strcmp(pname, "/system/bin/app_process64")) {
				memset(andr_app, '\0', TASK_COMM_LEN);
				strncpy(andr_app, p->comm, TASK_COMM_LEN);
				pname = andr_app;
			}
#endif
		}
		// mark the first of the bunch...
		if (*first == 1) {
			options |= LOPTS_1ST_MODREC;
			*first = 0;
		}
	}
#if defined(DRV_ALLOW_VDSO)
	else if ((vma->vm_mm  != NULL) &&
		 vma->vm_start == (long)vma->vm_mm->context.vdso) {
		pname = "[vdso]";
	}
#endif
#if defined(DRV_ALLOW_SYSCALL)
	else if (vma->vm_start == VSYSCALL_START) {
		pname = "[vsyscall]";
	}
#endif

	if (pname != NULL) {
		options = 0;
		if (DRV_VM_MOD_EXECUTABLE(vma)) {
			options |= LOPTS_EXE;
		}

		if (p && p->parent) {
			ppid = p->parent->tgid;
		}
		exec_mode = linuxos_Get_Exec_Mode(p);
		// record this module
		linuxos_Load_Image_Notify_Routine(
			pname, vma->vm_start, (vma->vm_end - vma->vm_start),
			page_offset, p->pid, ppid, options, exec_mode,
			load_event, 1, 0, OS_ID_NATIVE);
	}

	SEP_DRV_LOG_NOTIFICATION_TRACE_OUT(load_event == 1, "OS_SUCCESS.");
	return OS_SUCCESS;
}

//
// Common loop to enumerate all modules for a process.  The task_struct and mm
// should be locked if necessary to make sure they don't change while we're
// iterating...
//
static S32 linuxos_Enum_Modules_For_Process(struct task_struct *p,
					    struct mm_struct *mm,
					    S32 load_event)
{
	struct vm_area_struct *mmap;
	U32 first = 1;

#if defined(SECURE_SEP)
	uid_t l_uid;
#endif

	SEP_DRV_LOG_NOTIFICATION_TRACE_IN(load_event == 1,
					  "P: %p, mm: %p, load_event: %d.", p,
					  mm, load_event);

#if defined(SECURE_SEP)
	l_uid = DRV_GET_UID(p);
	/*
	 * Check for:  same uid, or root uid
	 */
	if (l_uid != uid && l_uid != 0) {
		SEP_DRV_LOG_NOTIFICATION_TRACE_OUT(
			load_event == 1,
			"OS_SUCCESS (secure_sep && l_uid != uid && l_uid != 0).");
		return OS_SUCCESS;
	}
#endif
	for (mmap = mm->mmap; mmap; mmap = mmap->vm_next) {
		/* We have 3 distinct conditions here.
		 * 1) Is the page executable?
		 * 2) Is is a part of the vdso area?
		 * 3) Is it the vsyscall area?
		 */
		if (((mmap->vm_flags & VM_EXEC) && mmap->vm_file &&
		     mmap->vm_file->DRV_F_DENTRY)
#if defined(DRV_ALLOW_VDSO)
		    || (mmap->vm_mm &&
			mmap->vm_start == (long)mmap->vm_mm->context.vdso)
#endif
#if defined(DRV_ALLOW_VSYSCALL)
		    || (mmap->vm_start == VSYSCALL_START)
#endif
		) {

			linuxos_VMA_For_Process(p, mmap, load_event, &first);
		}
	}

	SEP_DRV_LOG_NOTIFICATION_TRACE_OUT(load_event == 1, "OS_SUCCESS");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          static int linuxos_Exec_Unmap_Notify(
 *                  struct notifier_block  *self,
 *                  unsigned long           val,
 *                  VOID                   *data)
 *
 * @brief       this function is called whenever a task exits
 *
 * @param       self IN  - not used
 *              val  IN  - not used
 *              data IN  - this is cast in the mm_struct of the task that is call unmap
 *
 * @return      none
 *
 * <I>Special Notes:</I>
 *
 * This notification is called from do_munmap(mm/mmap.c). This is called when ever
 * a module is loaded or unloaded. It looks like it is called right after a module is
 * loaded or before its unloaded (if using dlopen, dlclose).
 * However it is not called when a process is exiting instead exit_mmap is called
 * (resulting in an EXIT_MMAP notification).
 */
static int linuxos_Exec_Unmap_Notify(struct notifier_block *self,
				     unsigned long val, PVOID data)
{
	struct mm_struct *mm;
	struct vm_area_struct *mmap = NULL;
	U32 first = 1;
	U32 cur_driver_state;

#if defined(SECURE_SEP)
	uid_t l_uid;
#endif

	SEP_DRV_LOG_NOTIFICATION_IN("Self: %p, val: %lu, data: %p.", self, val,
				    data);
	SEP_DRV_LOG_NOTIFICATION_TRACE(SEP_IN_NOTIFICATION,
				       "enter: unmap: hook_state %d.",
				       atomic_read(&hook_state));

	cur_driver_state = GET_DRIVER_STATE();

#if defined(SECURE_SEP)
	l_uid = DRV_GET_UID(current);
	/*
	 * Check for:  same uid, or root uid
	 */
	if (l_uid != uid && l_uid != 0) {
		SEP_DRV_LOG_NOTIFICATION_OUT(
			"Returns 0 (secure_sep && l_uid != uid && l_uid != 0).");
		return 0;
	}
#endif

	if (!IS_COLLECTING_STATE(cur_driver_state)) {
		SEP_DRV_LOG_NOTIFICATION_OUT("Early exit (driver state).");
		return 0;
	}
	if (!atomic_add_negative(1, &hook_state)) {
		SEP_DRV_LOG_NOTIFICATION_TRACE(SEP_IN_NOTIFICATION,
					       "unmap: hook_state %d.",
					       atomic_read(&hook_state));
		mm = get_task_mm(current);
		if (mm) {
			UTILITY_down_read_mm(mm);
			mmap = FIND_VMA(mm, data);
			if (mmap && mmap->vm_file &&
			    (mmap->vm_flags & VM_EXEC)) {
				linuxos_VMA_For_Process(current, mmap, TRUE,
							&first);
			}
			UTILITY_up_read_mm(mm);
			mmput(mm);
		}
	}
	atomic_dec(&hook_state);
	SEP_DRV_LOG_NOTIFICATION_TRACE(SEP_IN_NOTIFICATION,
				       "exit: unmap done: hook_state %d.",
				       atomic_read(&hook_state));

	SEP_DRV_LOG_NOTIFICATION_OUT("Returns 0.");
	return 0;
}

#if defined(DRV_CPU_HOTPLUG)
/* ------------------------------------------------------------------------- */
/*!
 * @fn       VOID linuxos_Handle_Online_cpu(
 *               PVOID param)
 *
 * @param    PVOID param
 *
 * @return   None
 *
 * @brief    Callback function to set the cpu online
 * @brief    and begin collection on it
 */
static VOID linuxos_Handle_Online_cpu(PVOID param)
{
	U32 this_cpu;
	U32 dev_idx;
	DISPATCH dispatch;

	SEP_DRV_LOG_NOTIFICATION_TRACE_IN(SEP_IN_NOTIFICATION,
					  "Dummy param: %p.", param);

	preempt_disable();
	this_cpu = CONTROL_THIS_CPU();
	dev_idx = core_to_dev_map[this_cpu];
	dispatch = LWPMU_DEVICE_dispatch(&devices[dev_idx]);
	preempt_enable();
	CPUMON_Online_Cpu((PVOID)&this_cpu);
	if (CPU_STATE_pmu_state(&pcb[this_cpu]) == NULL) {
		if (dispatch && dispatch->init) {
			dispatch->init(NULL);
		}
	}
	if (dispatch && dispatch->write) {
		dispatch->write(NULL);
	}
	CPU_STATE_group_swap(&pcb[this_cpu]) = 1;
	if (GET_DRIVER_STATE() == DRV_STATE_RUNNING) {
		// possible race conditions with notifications.
		// cleanup should wait until all notifications are done,
		// and new notifications should not proceed
		if (dispatch && dispatch->restart) {
			dispatch->restart(NULL);
		}
	}

	SEP_DRV_LOG_NOTIFICATION_TRACE_OUT(SEP_IN_NOTIFICATION, "");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn       VOID linuxos_Handle_Offline_cpu(
 *               PVOID param)
 *
 * @param    PVOID param
 *
 * @return   None
 *
 * @brief    Callback function to set the cpu offline
 * @brief    and stop collection on it
 */
static VOID linuxos_Handle_Offline_cpu(PVOID param)
{
	U32 this_cpu;
	U32 apic_lvterr;
	U32 dev_idx;
	DISPATCH dispatch;
	SEP_DRV_LOG_NOTIFICATION_TRACE_IN(SEP_IN_NOTIFICATION,
					  "Dummy param: %p.", param);

	preempt_disable();
	this_cpu = CONTROL_THIS_CPU();
	dev_idx = core_to_dev_map[this_cpu];
	dispatch = LWPMU_DEVICE_dispatch(&devices[dev_idx]);
	preempt_enable();
	CPUMON_Offline_Cpu((PVOID)&this_cpu);
	if (dispatch && dispatch->freeze) {
		dispatch->freeze(NULL);
	}
	apic_lvterr = apic_read(APIC_LVTERR);
	apic_write(APIC_LVTERR, apic_lvterr | APIC_LVT_MASKED);
	APIC_Restore_LVTPC(NULL);
	apic_write(APIC_LVTERR, apic_lvterr);

	SEP_DRV_LOG_NOTIFICATION_TRACE_OUT(SEP_IN_NOTIFICATION, "");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn       int linuxos_online_cpu(
 *               unsigned int cpu)
 *
 * @param    unsigned int cpu
 *
 * @return   None
 *
 * @brief    Invokes appropriate call back function when CPU is online
 */
static int linuxos_online_cpu(unsigned int cpu)
{
	SEP_DRV_LOG_NOTIFICATION_IN("Cpu %d coming online.", cpu);

	if (CPUMON_is_Online_Allowed()) {
		CONTROL_Invoke_Cpu(cpu, linuxos_Handle_Online_cpu, NULL);
		SEP_DRV_LOG_NOTIFICATION_OUT("Cpu %d came online.", cpu);
		return 0;
	} else {
		SEP_DRV_LOG_WARNING_NOTIFICATION_OUT(
			"Cpu %d is not allowed to come online!", cpu);
		return 0;
	}
}
/* ------------------------------------------------------------------------- */
/*!
 * @fn       int linuxos_offline_cpu(
 *               unsigned int cpu)
 *
 * @param    unsigned int cpu
 *
 * @return   None
 *
 * @brief    Invokes appropriate call back function when CPU is offline
 */
static int linuxos_offline_cpu(unsigned int cpu)
{
	SEP_DRV_LOG_NOTIFICATION_IN("Cpu %d going offline.", cpu);

	if (CPUMON_is_Offline_Allowed()) {
		CONTROL_Invoke_Cpu(cpu, linuxos_Handle_Offline_cpu, NULL);
		SEP_DRV_LOG_NOTIFICATION_OUT("Cpu %d went offline.", cpu);
		return 0;
	} else {
		SEP_DRV_LOG_WARNING_NOTIFICATION_OUT(
			"Cpu %d is not allowed to go offline!", cpu);
		return 0;
	}
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 10, 0)
/* ------------------------------------------------------------------------- */
/*!
 * @fn       OS_STATUS linuxos_Hotplug_Notifier(
 *               struct notifier_block *block, unsigned long action, void *pcpu)
 *
 * @param    struct notifier_block *block - notifier block
 *           unsigned long action - notifier action
 *           void *pcpu - per cpu pcb
 *
 * @return   NOTIFY_OK, if successful
 *
 * @brief    Hotplug Notifier function that handles various cpu states
 * @brief    and invokes respective callback functions
 */
static OS_STATUS linuxos_Hotplug_Notifier(struct notifier_block *block,
					  unsigned long action, void *pcpu)
{
	U32 cpu = (unsigned int)(unsigned long)pcpu;

	SEP_DRV_LOG_NOTIFICATION_IN(
		"Cpu: %u, action: %u.", cpu,
		action); // nb: will overcount number of pending notifications
	// when using this routine

	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_DOWN_FAILED:
		SEP_DRV_LOG_ERROR("SEP cpu %d offline failed!", cpu);
	case CPU_ONLINE:
		linuxos_online_cpu(cpu);
		break;
	case CPU_DOWN_PREPARE:
		linuxos_offline_cpu(cpu);
		break;
	default:
		SEP_DRV_LOG_WARNING(
			"DEFAULT: cpu %d unhandled action value is %d.", cpu,
			action);
		break;
	}

	SEP_DRV_LOG_NOTIFICATION_OUT("");
	return NOTIFY_OK;
}

static struct notifier_block cpu_hotplug_notifier = {
	.notifier_call = &linuxos_Hotplug_Notifier,
};
#endif
/* ------------------------------------------------------------------------- */
/*!
 * @fn       VOID LINUXOS_Register_Hotplug(
 *               VOID)
 *
 * @param    None
 *
 * @return   None
 *
 * @brief    Registers the Hotplug Notifier
 */
VOID LINUXOS_Register_Hotplug(void)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
	S32 err;

	SEP_DRV_LOG_INIT_IN(
		"Kernel version >= 4.10.0: using direct notifications.");

	err = cpuhp_setup_state_nocalls(CPUHP_AP_ONLINE_DYN, "ia64/sep5:online",
					linuxos_online_cpu,
					linuxos_offline_cpu);
	cpuhp_sepdrv_state = (int)err;
#else
	SEP_DRV_LOG_INIT_IN("Kernel version < 4.10.0: using notification hub.");
	register_cpu_notifier(&cpu_hotplug_notifier);
#endif
	SEP_DRV_LOG_INIT_OUT("Hotplug notifier registered.");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn       VOID LINUXOS_Unregister_Hotplug(
 *               VOID)
 *
 * @param    None
 *
 * @return   None
 *
 * @brief    Unregisters the Hotplug Notifier
 */
VOID LINUXOS_Unregister_Hotplug(void)
{
	SEP_DRV_LOG_INIT_IN("Unregistering hotplug notifier.");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
	cpuhp_remove_state_nocalls(cpuhp_sepdrv_state);
#else
	unregister_cpu_notifier(&cpu_hotplug_notifier);
#endif
	SEP_DRV_LOG_INIT_OUT("Hotplug notifier unregistered.");
}
#endif
/* ------------------------------------------------------------------------- */
/*!
 * @fn          OS_STATUS LINUXOS_Enum_Process_Modules(DRV_BOOL at_end)
 *
 * @brief       gather all the process modules that are present.
 *
 * @param       at_end - the collection happens at the end of the sampling run
 *
 * @return      OS_SUCCESS
 *
 * <I>Special Notes:</I>
 *              This routine gathers all the process modules that are present
 *              in the system at this time.  If at_end is set to be TRUE, then
 *              act as if all the modules are being unloaded.
 *
 */
OS_STATUS LINUXOS_Enum_Process_Modules(DRV_BOOL at_end)
{
	int n = 0;
	struct task_struct *p;

	SEP_DRV_LOG_TRACE_IN("At_end: %u.", at_end);
	SEP_DRV_LOG_TRACE("Begin tasks.");

	if (GET_DRIVER_STATE() == DRV_STATE_TERMINATING) {
		SEP_DRV_LOG_TRACE_OUT("OS_SUCCESS (TERMINATING).");
		return OS_SUCCESS;
	}

	if (!local_tasklist_lock) {
		local_tasklist_lock =
			(PVOID)(UIOP)UTILITY_Find_Symbol("tasklist_lock");
		if (!local_tasklist_lock) {
			SEP_DRV_LOG_WARNING("Could not find tasklist_lock.");
		}
	}

	// In some machines the tasklist_lock symbol does not exist.
	// For temporary solution we skip the lock if there is no tasklist_lock
	if (local_tasklist_lock) {
#if defined(                                                                   \
	DEFINE_QRWLOCK) // assuming that if DEFINE_QRWLOCK is defined, then tasklist_lock was defined using it
		qread_lock(local_tasklist_lock);
#else
		read_lock(local_tasklist_lock);
#endif
	}

	FOR_EACH_TASK(p)
	{
		struct mm_struct *mm;

		SEP_DRV_LOG_TRACE("Looking at task %d.", n);
		/*
		 *  Call driver notification routine for each module
		 *  that is mapped into the process created by the fork
		 */
		p->comm[TASK_COMM_LEN - 1] = 0;
		// making sure there is a trailing 0
		mm = get_task_mm(p);

		if (!mm) {
			SEP_DRV_LOG_TRACE(
				"Skipped (p->mm=NULL). P=0x%p, pid=%d, p->comm=%s.",
				p, p->pid, p->comm);
			linuxos_Load_Image_Notify_Routine(
				p->comm, 0, 0, 0, p->pid,
				(p->parent) ? p->parent->tgid : 0,
				LOPTS_EXE | LOPTS_1ST_MODREC,
				linuxos_Get_Exec_Mode(p),
				2, // '2' to trigger 'if (load_event)' conditions
				1, 0, OS_ID_NATIVE);
			continue;
		}

		UTILITY_down_read_mm(mm);
		linuxos_Enum_Modules_For_Process(p, mm, at_end ? -1 : 0);
		UTILITY_up_read_mm(mm);
		mmput(mm);
		n++;
	}

	if (local_tasklist_lock) {
#if defined(DEFINE_QRWLOCK)
		qread_unlock(local_tasklist_lock);
#else
		read_unlock(local_tasklist_lock);
#endif
	}

	SEP_DRV_LOG_TRACE("Enum_Process_Modules done with %d tasks.", n);

	SEP_DRV_LOG_TRACE_OUT("OS_SUCCESS.");
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          static int linuxos_Exit_Task_Notify(struct notifier_block * self,
 *                  unsigned long val, PVOID data)
 * @brief       this function is called whenever a task exits
 *
 * @param       self IN  - not used
 *              val IN  - not used
 *              data IN  - this is cast into the task_struct of the exiting task
 *
 * @return      none
 *
 * <I>Special Notes:</I>
 * this function is called whenever a task exits.  It is called right before
 * the virtual memory areas are freed.  We just enumerate through all the modules
 * of the task and set the unload sample count and the load event flag to 1 to
 * indicate this is a module unload
 */
static int linuxos_Exit_Task_Notify(struct notifier_block *self,
				    unsigned long val, PVOID data)
{
	struct task_struct *p = (struct task_struct *)data;
	int status = OS_SUCCESS;
	U32 cur_driver_state;
	struct mm_struct *mm;

	SEP_DRV_LOG_NOTIFICATION_IN("Self: %p, val: %lu, data: %p.", self, val,
				    data);

	cur_driver_state = GET_DRIVER_STATE();

	if (cur_driver_state == DRV_STATE_UNINITIALIZED ||
	    cur_driver_state == DRV_STATE_TERMINATING) {
		SEP_DRV_LOG_NOTIFICATION_OUT("Early exit (driver state).");
		return status;
	}
	SEP_DRV_LOG_TRACE("Pid = %d tgid = %d.", p->pid, p->tgid);
	if (p->pid == control_pid) {
		SEP_DRV_LOG_NOTIFICATION_TRACE(
			SEP_IN_NOTIFICATION,
			"The collector task has been terminated via an uncatchable signal.");
		SEP_DRV_LOG_NOTIFICATION_WARNING(SEP_IN_NOTIFICATION,
						 "Sep was killed!");
		CHANGE_DRIVER_STATE(STATE_BIT_ANY, DRV_STATE_TERMINATING);
		wake_up_interruptible(&wait_exit);

		SEP_DRV_LOG_NOTIFICATION_OUT("Res = %u (pid == control_pid).",
					     status);
		return status;
	}

	if (cur_driver_state != DRV_STATE_IDLE &&
	    !IS_COLLECTING_STATE(cur_driver_state)) {
		SEP_DRV_LOG_NOTIFICATION_OUT("Res = %u (stopping collection).",
					     status);
		return status;
	}

	mm = get_task_mm(p);
	if (!mm) {
		SEP_DRV_LOG_NOTIFICATION_OUT("Res = %u (!p->mm).", status);
		return status;
	}
	UTILITY_down_read_mm(mm);
	if (GET_DRIVER_STATE() != DRV_STATE_TERMINATING) {
		if (!atomic_add_negative(1, &hook_state)) {
			linuxos_Enum_Modules_For_Process(p, mm, 1);
		}
		atomic_dec(&hook_state);
	}
	UTILITY_up_read_mm(mm);
	mmput(mm);

	SEP_DRV_LOG_NOTIFICATION_TRACE(SEP_IN_NOTIFICATION, "Hook_state %d.",
				       atomic_read(&hook_state));

	SEP_DRV_LOG_NOTIFICATION_OUT("Res = %u.", status);
	return status;
}

/*
 *  The notifier block.  All the static entries have been defined at this point
 */
static struct notifier_block linuxos_exec_unmap_nb = {
	.notifier_call = linuxos_Exec_Unmap_Notify,
};

static struct notifier_block linuxos_exit_task_nb = {
	.notifier_call = linuxos_Exit_Task_Notify,
};

#if defined(CONFIG_TRACEPOINTS)
/* ------------------------------------------------------------------------- */
/*!
 * @fn          void capture_sched_switch(VOID *)
 * @brief       capture current pid/tid on all cpus
 *
 * @param       p IN - not used
 *
 * @return      none
 *
 * <I>Special Notes:</I>
 *
 * None
 */
static void capture_sched_switch(void *p)
{
	U32 this_cpu;
	BUFFER_DESC bd;
	SIDEBAND_INFO sideband_info;
	U64 tsc;

	SEP_DRV_LOG_TRACE_IN("");

	UTILITY_Read_TSC(&tsc);

	preempt_disable();
	this_cpu = CONTROL_THIS_CPU();
	preempt_enable();

	bd = &cpu_sideband_buf[this_cpu];
	if (bd == NULL) {
		SEP_DRV_LOG_ERROR_TRACE_OUT("Bd is NULL!");
		return;
	}

	sideband_info = (SIDEBAND_INFO)OUTPUT_Reserve_Buffer_Space(
		bd, sizeof(SIDEBAND_INFO_NODE), FALSE, !SEP_IN_NOTIFICATION,
		(S32)this_cpu);
	if (sideband_info == NULL) {
		SEP_DRV_LOG_ERROR_TRACE_OUT("Sideband_info is NULL!");
		return;
	}

	SIDEBAND_INFO_pid(sideband_info) = current->tgid;
	SIDEBAND_INFO_tid(sideband_info) = current->pid;
	SIDEBAND_INFO_tsc(sideband_info) = tsc;

	SEP_DRV_LOG_TRACE_OUT("");
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 28)
/* ------------------------------------------------------------------------- */
/*!
 * @fn          void record_pebs_process_info(...)
 * @brief       record all sched switch pid/tid info
 *
 * @param       ignore IN - not used
 *              from   IN
 *              to     IN
 *
 * @return      none
 *
 * <I>Special Notes:</I>
 *
 * None
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0)
static void record_pebs_process_info(void *ignore, bool preempt,
				     struct task_struct *from,
				     struct task_struct *to)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
static void record_pebs_process_info(void *ignore, struct task_struct *from,
				     struct task_struct *to)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 28)
static void record_pebs_process_info(struct rq *ignore,
				     struct task_struct *from,
				     struct task_struct *to)
#endif
{
	U32 this_cpu;
	BUFFER_DESC bd;
	SIDEBAND_INFO sideband_info;
	U64 tsc;
	U32 cur_driver_state;

	SEP_DRV_LOG_NOTIFICATION_IN("From: %p, to: %p.", from, to);

	cur_driver_state = GET_DRIVER_STATE();

	if (cur_driver_state != DRV_STATE_IDLE &&
	    !IS_COLLECTING_STATE(cur_driver_state)) {
		SEP_DRV_LOG_NOTIFICATION_OUT("Early exit (driver state).");
		return;
	}

	UTILITY_Read_TSC(&tsc);

	preempt_disable();
	this_cpu = CONTROL_THIS_CPU();
	preempt_enable();

	SEP_DRV_LOG_NOTIFICATION_TRACE(SEP_IN_NOTIFICATION,
				       "[OUT<%d:%d:%s>-IN<%d:%d:%s>].",
				       from->tgid, from->pid, from->comm,
				       to->tgid, to->pid, to->comm);

	bd = &cpu_sideband_buf[this_cpu];
	if (bd == NULL) {
		SEP_DRV_LOG_NOTIFICATION_OUT("Early exit (!bd).");
		return;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 13, 0)) &&                        \
	(LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0))
	sideband_info = (SIDEBAND_INFO)OUTPUT_Reserve_Buffer_Space(
		bd, sizeof(SIDEBAND_INFO_NODE), TRUE, SEP_IN_NOTIFICATION,
		(S32)this_cpu);
#else
	sideband_info = (SIDEBAND_INFO)OUTPUT_Reserve_Buffer_Space(
		bd, sizeof(SIDEBAND_INFO_NODE), FALSE, SEP_IN_NOTIFICATION,
		(S32)this_cpu);
#endif

	if (sideband_info == NULL) {
		SEP_DRV_LOG_NOTIFICATION_OUT("Early exit (!sideband_info).");
		return;
	}

	SIDEBAND_INFO_pid(sideband_info) = to->tgid;
	SIDEBAND_INFO_tid(sideband_info) = to->pid;
	SIDEBAND_INFO_tsc(sideband_info) = tsc;

	SEP_DRV_LOG_NOTIFICATION_OUT("");
}
#endif
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 15, 0)
/* ------------------------------------------------------------------------- */
/*!
 * @fn          void find_sched_switch_tracepoint
 * @brief       find trace poing for sched_switch
 *
 * @param       tp    pass in by system
 *              param pointer of trace point
 *
 * @return      none
 *
 * <I>Special Notes:</I>
 *
 * None
 */
static void find_sched_switch_tracepoint(struct tracepoint *tp, VOID *param)
{
	struct tracepoint **ptp = (struct tracepoint **)param;

	SEP_DRV_LOG_TRACE_IN("Tp: %p, param: %p.", tp, param);

	if (tp && ptp) {
		SEP_DRV_LOG_TRACE("trace point name: %s.", tp->name);
		if (!strcmp(tp->name, "sched_switch")) {
			SEP_DRV_LOG_TRACE(
				"Found trace point for sched_switch.");
			*ptp = tp;
		}
	}

	SEP_DRV_LOG_TRACE_OUT("");
}
#endif

/* ------------------------------------------------------------------------- */
/*!
 * @fn          int install_sched_switch_callback(void)
 * @brief       registers sched_switch callbacks for PEBS sideband
 *
 * @param       none
 *
 * @return      0 success else error number
 *
 * <I>Special Notes:</I>
 *
 * None
 */
static int install_sched_switch_callback(void)
{
	int err = 0;

	SEP_DRV_LOG_TRACE_IN("");
	SEP_DRV_LOG_INIT("Installing PEBS linux OS Hooks.");

#if defined(CONFIG_TRACEPOINTS)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 15, 0)
	if (!tp_sched_switch) {
		for_each_kernel_tracepoint(&find_sched_switch_tracepoint,
					   &tp_sched_switch);
	}
	if (!tp_sched_switch) {
		err = -EIO;
		SEP_DRV_LOG_INIT(
			"Please check Linux is built w/ CONFIG_CONTEXT_SWITCH_TRACER.");
	} else {
		err = tracepoint_probe_register(
			tp_sched_switch, (void *)record_pebs_process_info,
			NULL);
	}
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
	err = register_trace_sched_switch(record_pebs_process_info, NULL);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 28)
	err = register_trace_sched_switch(record_pebs_process_info);
#else
	SEP_DRV_LOG_INIT(
		"Please use Linux kernel version >= 2.6.28 to use multiple pebs.");
	err = -1;
#endif
	CONTROL_Invoke_Parallel(capture_sched_switch, NULL);
#endif

	SEP_DRV_LOG_TRACE_OUT("Res: %d.", err);
	return err;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID LINUXOS_Install_Hooks(void)
 * @brief       registers the profiling callbacks
 *
 * @param       none
 *
 * @return      none
 *
 * <I>Special Notes:</I>
 *
 * None
 */
VOID LINUXOS_Install_Hooks(void)
{
	int err = 0;
	int err2 = 0;

	SEP_DRV_LOG_INIT_IN("Installing Linux OS Hooks.");

	if (hooks_installed == 1) {
		SEP_DRV_LOG_INIT_OUT("The OS Hooks are already installed.");
		return;
	}

	linuxos_Map_Kernel_Modules();

	err = profile_event_register(MY_UNMAP, &linuxos_exec_unmap_nb);
	err2 = profile_event_register(MY_TASK, &linuxos_exit_task_nb);
	if (err || err2) {
		if (err == OS_NO_SYSCALL) {
			SEP_DRV_LOG_WARNING(
				"This kernel does not implement kernel profiling hooks...");
			SEP_DRV_LOG_WARNING(
				"...task termination and image unloads will not be tracked...");
			SEP_DRV_LOG_WARNING("...during sampling session!");
		}
	}

	if (multi_pebs_enabled || sched_switch_enabled) {
		err = install_sched_switch_callback();
		if (err) {
			SEP_DRV_LOG_WARNING(
				"Failed to install sched_switch callback for multiple pebs.");
		}
	}

	hooks_installed = 1;
	atomic_set(&hook_state, HOOK_FREE);

	SEP_DRV_LOG_INIT_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          int uninstall_sched_switch_callback(void)
 * @brief       unregisters sched_switch callbacks for PEBS sideband
 *
 * @param       none
 *
 * @return      0 success else error number
 *
 * <I>Special Notes:</I>
 *
 * None
 */
static int uninstall_sched_switch_callback(void)
{
	int err = 0;

	SEP_DRV_LOG_TRACE_IN("");
	SEP_DRV_LOG_INIT("Uninstalling PEBS Linux OS Hooks.");

#if defined(CONFIG_TRACEPOINTS)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 15, 0)
	if (!tp_sched_switch) {
		err = -EIO;
		SEP_DRV_LOG_INIT(
			"Please check Linux is built w/ CONFIG_CONTEXT_SWITCH_TRACER.");
	} else {
		err = tracepoint_probe_unregister(
			tp_sched_switch, (void *)record_pebs_process_info,
			NULL);
	}
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
	err = unregister_trace_sched_switch(record_pebs_process_info, NULL);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 28)
	err = unregister_trace_sched_switch(record_pebs_process_info);
#else
	SEP_DRV_LOG_INIT(
		"Please use Linux kernel version >= 2.6.28 to use multiple pebs.");
	err = -1;
#endif
	CONTROL_Invoke_Parallel(capture_sched_switch, NULL);
#endif

	SEP_DRV_LOG_TRACE_OUT("Res: %d.", err);
	return err;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID LINUXOS_Uninstall_Hooks(void)
 * @brief       unregisters the profiling callbacks
 *
 * @param       none
 *
 * @return
 *
 * <I>Special Notes:</I>
 *
 * None
 */
VOID LINUXOS_Uninstall_Hooks(void)
{
	int err = 0;
	int value = 0;
	int tries = 10;

	SEP_DRV_LOG_INIT_IN("Uninstalling Linux OS Hooks.");

	if (hooks_installed == 0) {
		SEP_DRV_LOG_INIT_OUT("Hooks are not installed!");
		return;
	}

	hooks_installed = 0;
	profile_event_unregister(MY_UNMAP, &linuxos_exec_unmap_nb);
	profile_event_unregister(MY_TASK, &linuxos_exit_task_nb);

	if (multi_pebs_enabled || sched_switch_enabled) {
		err = uninstall_sched_switch_callback();
		if (err) {
			SEP_DRV_LOG_WARNING(
				"Failed to uninstall sched_switch callback for multiple pebs.");
		}
	}

	value = atomic_cmpxchg(&hook_state, HOOK_FREE, HOOK_UNINSTALL);
	if ((value == HOOK_FREE) ||
	    (value == HOOK_UNINSTALL)) { // already in free or uninstall state
		SEP_DRV_LOG_INIT_OUT(
			"Uninstall hook done (already in state %d).", value);
		return;
	}
	atomic_add(HOOK_UNINSTALL, &hook_state);
	while (tries) {
		SYS_IO_Delay();
		SYS_IO_Delay();
		value = atomic_read(&hook_state);
		if (value == HOOK_UNINSTALL) {
			break;
		}
		tries--;
	}

	SEP_DRV_LOG_INIT_OUT("Done -- state %d, tries %d.", value, tries);
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          DRV_BOOL LINUXOS_Check_KVM_Guest_Process()
 *
 * @brief       check the presence of kvm guest process
 *
 * @param       none
 *
 * @return      TRUE if the kvm guest process is running, FALSE if not
 */
DRV_BOOL LINUXOS_Check_KVM_Guest_Process(void)
{
	struct task_struct *p;

	SEP_DRV_LOG_TRACE_IN("");

	if (!local_tasklist_lock) {
		local_tasklist_lock =
			(PVOID)(UIOP)UTILITY_Find_Symbol("tasklist_lock");
		if (!local_tasklist_lock) {
			SEP_DRV_LOG_WARNING("Could not find tasklist_lock.");
		}
	}

	// In some machines the tasklist_lock symbol does not exist.
	// For temporary solution we skip the lock if there is no tasklist_lock
	if (local_tasklist_lock) {
#if defined(DEFINE_QRWLOCK)
		qread_lock(local_tasklist_lock);
#else
		read_lock(local_tasklist_lock);
#endif
	}

	FOR_EACH_TASK(p)
	{
		// if (p == NULL) {
		// 	continue;
		// }

		p->comm[TASK_COMM_LEN - 1] =
			0; // making sure there is a trailing 0

		if (!strncmp(p->comm, "qemu-kvm", 8)) {
			if (local_tasklist_lock) {
#if defined(DEFINE_QRWLOCK)
				qread_unlock(local_tasklist_lock);
#else
				read_unlock(local_tasklist_lock);
#endif
			}

			SEP_DRV_LOG_INIT_TRACE_OUT("TRUE (found qemu-kvm!).");
			return TRUE;
		}
	}

	if (local_tasklist_lock) {
#if defined(DEFINE_QRWLOCK)
		qread_unlock(local_tasklist_lock);
#else
		read_unlock(local_tasklist_lock);
#endif
	}

	SEP_DRV_LOG_TRACE_OUT("FALSE");
	return FALSE;
}
