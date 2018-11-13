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

#include <linux/fs.h>
#include <linux/kobject.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#if defined(CONFIG_HARDLOCKUP_DETECTOR) &&                                     \
	LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/fs.h>
#endif

#include "lwpmudrv_defines.h"
#include "lwpmudrv_types.h"
#include "lwpmudrv.h"
#include "lwpmudrv_ioctl.h"

#include "control.h"
#include "pax_shared.h"
#include "pax.h"

MODULE_AUTHOR("Copyright(C) 2009-2018 Intel Corporation");
MODULE_VERSION(PAX_NAME "_" PAX_VERSION_STR);
MODULE_LICENSE("Dual BSD/GPL");

typedef struct PAX_DEV_NODE_S PAX_DEV_NODE;
typedef PAX_DEV_NODE * PAX_DEV;

struct PAX_DEV_NODE_S {
	long buffer;
	struct semaphore sem;
	struct cdev cdev;
};

#define PAX_DEV_buffer(dev) ((dev)->buffer)
#define PAX_DEV_sem(dev) ((dev)->sem)
#define PAX_DEV_cdev(dev) ((dev)->cdev)

// global variables for the PAX driver

static PAX_DEV pax_control; // main control
static dev_t pax_devnum; // the major char device number for PAX
static PAX_VERSION_NODE pax_version; // version of PAX
static PAX_INFO_NODE pax_info; // information on PAX
static PAX_STATUS_NODE pax_status; // PAX reservation status

static struct class *pax_class;

#define NMI_WATCHDOG_PATH "/proc/sys/kernel/nmi_watchdog"
static S8 nmi_watchdog_restore = '0';

static struct proc_dir_entry *pax_version_file;

static int pax_version_proc_read(struct seq_file *, void *);
static int pax_version_proc_open(struct inode *, struct file *);
static struct file_operations pax_version_ops = {
	.owner = THIS_MODULE,
	.open = pax_version_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

// Print macros for kernel debugging

#if defined(DEBUG)
#define PAX_PRINT_DEBUG(fmt, args...)                                          \
	{                                                                      \
		printk(KERN_INFO "PAX: [DEBUG] " fmt, ##args);                 \
	}
#else
#define PAX_PRINT_DEBUG(fmt, args...)                                          \
	{                                                                      \
		;                                                              \
	}
#endif
#define PAX_PRINT(fmt, args...)                                                \
	{                                                                      \
		printk(KERN_INFO "PAX: " fmt, ##args);                         \
	}
#define PAX_PRINT_WARNING(fmt, args...)                                        \
	{                                                                      \
		printk(KERN_ALERT "PAX: [Warning] " fmt, ##args);              \
	}
#define PAX_PRINT_ERROR(fmt, args...)                                          \
	{                                                                      \
		printk(KERN_CRIT "PAX: [ERROR] " fmt, ##args);                 \
	}

// various other useful macros

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 25)
#define PAX_FIND_TASK_BY_PID(pid) find_task_by_pid(pid)
#elif LINUX_VERSION_CODE < KERNEL_VERSION(4, 7, 0)
#define PAX_FIND_TASK_BY_PID(pid)                                              \
	pid_task(find_pid_ns(pid, &init_pid_ns), PIDTYPE_PID);
#else
#define PAX_FIND_TASK_BY_PID(pid) pid_task(find_get_pid(pid), PIDTYPE_PID);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 18)
#define PAX_TASKLIST_READ_LOCK() read_lock(&tasklist_lock)
#define PAX_TASKLIST_READ_UNLOCK() read_unlock(&tasklist_lock)
#else
#define PAX_TASKLIST_READ_LOCK() rcu_read_lock()
#define PAX_TASKLIST_READ_UNLOCK() rcu_read_unlock()
#endif

#if defined(CONFIG_HARDLOCKUP_DETECTOR) &&                                     \
	LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)

static struct task_struct *pax_Enable_NMIWatchdog_Thread;
static struct semaphore pax_Enable_NMIWatchdog_Sem;
static struct task_struct *pax_Disable_NMIWatchdog_Thread;
static struct semaphore pax_Disable_NMIWatchdog_Sem;

/* ------------------------------------------------------------------------- */
/*!
 * @fn  S32 pax_Disable_NMIWatchdog(PVOID data)
 *
 * @param data - Pointer to data
 *
 * @return S32
 *
 * @brief Disable nmi watchdog
 *
 * <I>Special Notes</I>
 */
static S32 pax_Disable_NMIWatchdog(PVOID data)
{
	struct file *fd;
	mm_segment_t old_fs;
	struct cred *kcred;
	loff_t pos = 0;
	S8 new_val = '0';

	up(&pax_Disable_NMIWatchdog_Sem);

	kcred = prepare_kernel_cred(NULL);
	if (kcred) {
		commit_creds(kcred);
	} else {
		PAX_PRINT_ERROR(
			"pax_Disable_NMIWatchdog: prepare_kernel_cred returns NULL\n");
	}

	fd = filp_open(NMI_WATCHDOG_PATH, O_RDWR, 0);

	if (fd) {
		fd->f_op->read(fd, (char __user *)&nmi_watchdog_restore, 1, &fd->f_pos);
		PAX_PRINT_DEBUG("Existing nmi_watchdog value = %c\n",
				nmi_watchdog_restore);

		if (nmi_watchdog_restore != '0') {
			old_fs = get_fs();
			set_fs(KERNEL_DS);
			fd->f_op->write(fd, (char __user *)&new_val, 1, &pos);
			set_fs(old_fs);
		} else {
			PAX_PRINT_DEBUG(
				"pax_Disable_NMIWatchdog: NMI watchdog already disabled!\n");
		}

		filp_close(fd, NULL);
	} else {
		PAX_PRINT_ERROR(
			"pax_Disable_NMIWatchdog: filp_open returns NULL\n");
	}

	while (!kthread_should_stop()) {
		schedule();
	}

	return 0;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  S32 pax_Check_NMIWatchdog(PVOID data)
 *
 * @param data - Pointer to data
 *
 * @return S32
 *
 * @brief Check nmi watchdog
 *
 * <I>Special Notes</I>
 */

#if 0
static S32 pax_Check_NMIWatchdog(PVOID data)
{
	struct file *fd;
	struct cred *kcred;

	kcred = prepare_kernel_cred(NULL);
	if (kcred) {
		commit_creds(kcred);
	}

	fd = filp_open(NMI_WATCHDOG_PATH, O_RDWR, 0);

	if (fd) {
		fd->f_op->read(fd, &nmi_watchdog_restore, 1, &fd->f_pos);
		PAX_PRINT_DEBUG("Checking nmi_watchdog value = %c\n",
				nmi_watchdog_restore);
		filp_close(fd, NULL);
	}

	do_exit(0);

	return 0;
}
#endif
/* ------------------------------------------------------------------------- */
/*!
 * @fn  S32 pax_Enable_NMIWatchdog(PVOID data)
 *
 * @param data - Pointer to data
 *
 * @return S32
 *
 * @brief Enable nmi watchdog
 *
 * <I>Special Notes</I>
 */
static S32 pax_Enable_NMIWatchdog(PVOID data)
{
	struct file *fd;
	mm_segment_t old_fs;
	struct cred *kcred;
	loff_t pos = 0;
	S8 new_val = '1';

	up(&pax_Enable_NMIWatchdog_Sem);

	kcred = prepare_kernel_cred(NULL);
	if (kcred) {
		commit_creds(kcred);
	} else {
		PAX_PRINT_ERROR(
			"pax_Enable_NMIWatchdog: prepare_kernel_cred returns NULL!\n");
	}

	fd = filp_open(NMI_WATCHDOG_PATH, O_WRONLY, 0);

	if (fd) {
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		fd->f_op->write(fd, (char __user *)&new_val, 1, &pos);
		set_fs(old_fs);

		filp_close(fd, NULL);
	} else {
		PAX_PRINT_ERROR(
			"pax_Enable_NMIWatchdog: filp_open returns NULL!\n");
	}

	while (!kthread_should_stop()) {
		schedule();
	}

	return 0;
}
#endif

/* ------------------------------------------------------------------------- */
/*!
 * @fn     void pax_Init()
 *
 * @param  none
 *
 * @return none
 *
 * @brief  Initialize PAX system
 *
 * <I>Special Notes</I>
 */
static void pax_Init(void)
{
	//
	// Initialize PAX driver version (done once at driver load time)
	//

	PAX_VERSION_NODE_major(&pax_version) = PAX_MAJOR_VERSION;
	PAX_VERSION_NODE_minor(&pax_version) = PAX_MINOR_VERSION;
	PAX_VERSION_NODE_bugfix(&pax_version) = PAX_BUGFIX_VERSION;

	// initialize PAX_Info
	pax_info.version = PAX_VERSION_NODE_version(&pax_version);
	pax_info.managed_by = 1; // THIS_MODULE->name;

	// initialize PAX_Status
	pax_status.guid = PAX_GUID_UNINITIALIZED;
	pax_status.pid = 0;
	pax_status.start_time = 0;
	pax_status.is_reserved = PAX_PMU_UNRESERVED;

}

/* ------------------------------------------------------------------------- */
/*!
 * @fn     void pax_Cleanup()
 *
 * @param  none
 *
 * @return none
 *
 * @brief  UnInitialize PAX system
 *
 * <I>Special Notes</I>
 */
static void pax_Cleanup(void)
{
	// uninitialize PAX_Info
	pax_info.managed_by = 0;

	// uninitialize PAX_Status
	pax_status.guid = PAX_GUID_UNINITIALIZED;
	pax_status.pid = 0;
	pax_status.start_time = 0;
	pax_status.is_reserved = PAX_PMU_UNRESERVED;

}

/* ------------------------------------------------------------------------- */
/*!
 * @fn     U32 pax_Process_Valid()
 *
 * @param  U32 pid - process ID
 *
 * @return TRUE or FALSE
 *
 * @brief  Check whether process with pid still exists, and if so,
 *         whether it is still "alive".  If so, then process is
 *         deemed valid.  Otherwise, process is deemed invalid.
 *
 * <I>Special Notes</I>
 */
static U32 pax_Process_Valid(U32 pid)
{
	struct task_struct *process_task;
	U32 valid_process;

	//
	// There doesn't seem to be a way to force the process_task to continue
	// to exist after the read_lock is released (SMP system could delete the
	// process after lock is released on another processor), so we need to
	// do all the work with the lock held... There is a routine on later
	// 2.6 kernels (get_task_struct() and put_task_struct()) which seems
	// to do what we want, but the code behind the macro calls a function
	// that isn't EXPORT'ed so we can't use it in a device driver...
	//
	PAX_TASKLIST_READ_LOCK();
	process_task = PAX_FIND_TASK_BY_PID(pax_status.pid);
	if ((process_task == NULL) ||
	    (process_task->exit_state == EXIT_ZOMBIE) ||
	    (process_task->exit_state == EXIT_DEAD)) {
		// not a valid process
		valid_process = FALSE;
	} else {
		// process is "alive", so assume it is still valid ...
		valid_process = TRUE;
	}
	PAX_TASKLIST_READ_UNLOCK();

	return valid_process;
}

// **************************************************************************
//
// below are PAX Open/Read/Write device functions (appears in /proc/kallsyms)
//
// **************************************************************************

/* ------------------------------------------------------------------------- */
/*!
 * @fn     int pax_Open()
 *
 * @param  struct inode *inode
 * @param  struct file  *filp
 *
 * @return int (TODO: check for open failure)
 *
 * @brief  This function is called when doing an open(/dev/pax)
 *
 * <I>Special Notes</I>
 */
static int pax_Open(struct inode *inode, struct file *filp)
{
	PAX_PRINT_DEBUG("open called on maj:%d, min:%d\n", imajor(inode),
			iminor(inode));
	filp->private_data = container_of(inode->i_cdev, PAX_DEV_NODE, cdev);

	return 0;
}

// **************************************************************************
//
// below are PAX IOCTL function handlers
//
// **************************************************************************

/* ------------------------------------------------------------------------- */
/*!
 * @fn     OS_STATUS pax_Get_Info()
 *
 * @param  IOCTL_ARGS arg  - pointer to the output buffer
 *
 * @return OS_STATUS
 *
 * @brief  Local function that handles the PAX_IOCTL_INFO call
 *         Returns static information related to PAX (e.g., version)
 *
 * <I>Special Notes</I>
 */
static OS_STATUS pax_Get_Info(IOCTL_ARGS arg)
{
	int error;

	error = copy_to_user((void __user *)(arg->buf_usr_to_drv),
				  &pax_info, sizeof(PAX_INFO_NODE));

	if (error != 0) {
		PAX_PRINT_ERROR(
			"pax_Get_Info: unable to copy to user (error=%d)!\n",
			error);
		return OS_FAULT;
	}

	PAX_PRINT_DEBUG("pax_Get_Info: sending PAX info (%ld bytes):\n",
			sizeof(PAX_INFO_NODE));
	PAX_PRINT_DEBUG("pax_Get_Info:      raw_version = %u (0x%x)\n",
			pax_info.version, pax_info.version);
	PAX_PRINT_DEBUG("pax_Get_Info:            major = %u\n",
			PAX_VERSION_NODE_major(&pax_version));
	PAX_PRINT_DEBUG("pax_Get_Info:            minor = %u\n",
			PAX_VERSION_NODE_minor(&pax_version));
	PAX_PRINT_DEBUG("pax_Get_Info:           bugfix = %u\n",
			PAX_VERSION_NODE_bugfix(&pax_version));
	PAX_PRINT_DEBUG("pax_Get_Info:      managed_by = %lu\n",
			(long unsigned int)pax_info.managed_by);
	PAX_PRINT_DEBUG("pax_Get_Info: information sent.\n");

	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn     OS_STATUS pax_Get_Status()
 *
 * @param  IOCTL_ARGS arg  - pointer to the output buffer
 *
 * @return OS_STATUS
 *
 * @brief  Local function that handles the PAX_IOCTL_STATUS call
 *         Returns status of the reservation (e.g., who owns)
 *
 * <I>Special Notes</I>
 */
static OS_STATUS pax_Get_Status(IOCTL_ARGS arg)
{
	int error;

	error = copy_to_user((void __user *)(arg->buf_usr_to_drv),
				  &pax_status, sizeof(PAX_STATUS_NODE));
	if (error != 0) {
		PAX_PRINT_ERROR(
			"pax_Get_Status: unable to copy to user (error=%d)!\n",
			error);
		return OS_FAULT;
	}

	PAX_PRINT_DEBUG("pax_Get_Status: sending PAX status (%ld bytes):\n",
			sizeof(PAX_STATUS_NODE));
	PAX_PRINT_DEBUG("pax_Get_Status:    guid = %lu\n",
			(long unsigned int)pax_status.guid);
	PAX_PRINT_DEBUG("pax_Get_Status:    pid = %lu\n",
			(long unsigned int)pax_status.pid);
	PAX_PRINT_DEBUG("pax_Get_Status:    start_time = %lu\n",
			(long unsigned int)pax_status.start_time);
	PAX_PRINT_DEBUG("pax_Get_Status:    is_reserved = %u\n",
			pax_status.is_reserved);
	PAX_PRINT_DEBUG("pax_Get_Status: status sent.\n");

	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn     OS_STATUS pax_Unreserve()
 *
 * @param  none
 *
 * @return OS_STATUS
 *
 * @brief  Local function that handles the PAX_IOCTL_UNRESERVE call
 *         Returns OS_SUCCESS if PMU unreservation succeeded, otherwise failure
 *
 * <I>Special Notes</I>
 */
static OS_STATUS pax_Unreserve(void)
{
	// if no reservation is currently held, then return success
	if (pax_status.is_reserved == PAX_PMU_UNRESERVED) {
		PAX_PRINT_DEBUG("pax_Unreserve: currently unreserved\n");
		return OS_SUCCESS;
	}

	// otherwise, there is a reservation ...
	// allow the process which started the reservation to unreserve
	// or if that process is invalid, then any other process can unreserve
	if ((pax_status.pid == current->pid) ||
	    (!pax_Process_Valid(pax_status.pid))) {
		S32 reservation = -1;
		PAX_PRINT_DEBUG(
			"pax_Unreserve: pid %d attempting to unreserve PMU held by pid %d\n",
			(U32)current->pid, (U32)pax_status.pid);

#if !defined(DRV_ANDROID) && !defined(DRV_CHROMEOS) &&                         \
	defined(CONFIG_HARDLOCKUP_DETECTOR) &&                                 \
	LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		if (nmi_watchdog_restore != '0') {
			PAX_PRINT_DEBUG(
				"Attempting to enable NMI watchdog...\n");

			sema_init(&pax_Enable_NMIWatchdog_Sem, 0);

			pax_Enable_NMIWatchdog_Thread =
				kthread_run(&pax_Enable_NMIWatchdog, NULL,
					    "pax_enable_nmi_watchdog");
			if (!pax_Enable_NMIWatchdog_Thread ||
			    pax_Enable_NMIWatchdog_Thread == ERR_PTR(-ENOMEM)) {
				PAX_PRINT_ERROR(
					"pax_Unreserve: could not create pax_enable_nmi_watchdog kthread.");
			} else {
				down(&pax_Enable_NMIWatchdog_Sem);
				kthread_stop(pax_Enable_NMIWatchdog_Thread);
			}
			pax_Enable_NMIWatchdog_Thread = NULL;
			nmi_watchdog_restore = '0';
		}
#endif

		reservation = cmpxchg(&pax_status.is_reserved, PAX_PMU_RESERVED,
				      PAX_PMU_UNRESERVED);
		if (reservation < 0) {
			// no-op ... eliminates "variable not used" compiler warning
		}
		PAX_PRINT_DEBUG("pax_Unreserve: reserve=%d, is_reserved=%d\n",
				reservation, pax_status.is_reserved);
		// unreserve but keep track of last PID/GUID that had reservation
	}

	PAX_PRINT_DEBUG("pax_Unreserve: pid %d unreserve status: %d\n",
			current->pid, pax_status.is_reserved);

	return ((pax_status.is_reserved == PAX_PMU_UNRESERVED) ? OS_SUCCESS :
								 OS_FAULT);
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn     OS_STATUS pax_Reserve_All()
 *
 * @param  none
 *
 * @return OS_STATUS
 *
 * @brief  Local function that handles the PAX_IOCTL_RESERVE_ALL call
 *         Returns OS_SUCCESS if PMU reservation succeeded, otherwise failure
 *
 * <I>Special Notes</I>
 */
static OS_STATUS pax_Reserve_All(void)
{
	S32 reservation = -1; // previous reservation state (initially, unknown)

	// check if PMU can be unreserved
	if (pax_status.is_reserved == PAX_PMU_RESERVED) {
		OS_STATUS unreserve_err = pax_Unreserve();
		if (unreserve_err != OS_SUCCESS) {
			return unreserve_err; // attempt to unreserve failed, so return error
		}
	}

	PAX_PRINT_DEBUG("pax_Reserve_All: pid %d attempting to reserve PMU\n",
			current->pid);

	// at this point, there is no reservation, so commence race to reserve ...
	reservation = cmpxchg(&pax_status.is_reserved, PAX_PMU_UNRESERVED,
			      PAX_PMU_RESERVED);

	// only one request to reserve will succeed, and when it does, update status
	// information with the successful request
	if ((reservation == PAX_PMU_UNRESERVED) &&
	    (pax_status.is_reserved == PAX_PMU_RESERVED)) {
		pax_status.start_time = rdtsc_ordered();
		pax_status.pid = current->pid;

#if !defined(DRV_ANDROID) && !defined(DRV_CHROMEOS) &&                         \
	defined(CONFIG_HARDLOCKUP_DETECTOR) &&                                 \
	LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		sema_init(&pax_Disable_NMIWatchdog_Sem, 0);
		pax_Disable_NMIWatchdog_Thread =
			kthread_run(&pax_Disable_NMIWatchdog, NULL,
				    "pax_disable_nmi_watchdog");
		if (!pax_Disable_NMIWatchdog_Thread ||
		    pax_Disable_NMIWatchdog_Thread == ERR_PTR(-ENOMEM)) {
			PAX_PRINT_ERROR(
				"pax_Reserve_All: could not create pax_disable_nmi_watchdog kthread.");
		} else {
			down(&pax_Disable_NMIWatchdog_Sem);
			kthread_stop(pax_Disable_NMIWatchdog_Thread);
		}
		pax_Disable_NMIWatchdog_Thread = NULL;
#endif

		return OS_SUCCESS;
	}

	return OS_FAULT;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn     OS_STATUS pax_Service_IOCTL()
 *
 * @param  inode - pointer to the device object
 * @param  filp  - pointer to the file object
 * @param  cmd   - ioctl value (defined in lwpmu_ioctl.h)
 * @param  arg   - arg or arg pointer
 *
 * @return OS_STATUS
 *
 * @brief  Worker function that handles IOCTL requests from the user mode
 *
 * <I>Special Notes</I>
 */
static IOCTL_OP_TYPE pax_Service_IOCTL(IOCTL_USE_INODE struct file *filp,
				       unsigned int cmd,
				       IOCTL_ARGS_NODE local_args)
{
	int status = OS_SUCCESS;

	// dispatch to appropriate PAX IOCTL function
	switch (cmd) {
	case PAX_IOCTL_INFO:
		PAX_PRINT_DEBUG("PAX_IOCTL_INFO\n");
		status = pax_Get_Info(&local_args);
		break;

	case PAX_IOCTL_STATUS:
		PAX_PRINT_DEBUG("PAX_IOCTL_STATUS\n");
		status = pax_Get_Status(&local_args);
		break;

	case PAX_IOCTL_RESERVE_ALL:
		PAX_PRINT_DEBUG("PAX_IOCTL_RESERVE_ALL\n");
		status = pax_Reserve_All();
		break;

	case PAX_IOCTL_UNRESERVE:
		PAX_PRINT_DEBUG("PAX_IOCTL_UNRESERVE\n");
		status = pax_Unreserve();
		break;

	default:
		PAX_PRINT_ERROR("unknown IOCTL cmd: %d magic:%d number:%d\n",
				cmd, _IOC_TYPE(cmd), _IOC_NR(cmd));
		status = OS_ILLEGAL_IOCTL;
		break;
	}

	return status;
}

static long pax_Device_Control(IOCTL_USE_INODE struct file *filp,
			       unsigned int cmd, unsigned long arg)
{
	int status = OS_SUCCESS;
	IOCTL_ARGS_NODE local_args;

	memset(&local_args, 0, sizeof(IOCTL_ARGS_NODE));
	if (arg) {
		status = copy_from_user(&local_args, (void __user *)arg,
					sizeof(IOCTL_ARGS_NODE));
		if (status != OS_SUCCESS)
			return status;
	}

	status = pax_Service_IOCTL(IOCTL_USE_INODE filp, cmd, local_args);
	return status;
}

#if defined(CONFIG_COMPAT) && defined(DRV_EM64T)
static IOCTL_OP_TYPE pax_Device_Control_Compat(struct file *filp,
					       unsigned int cmd,
					       unsigned long arg)
{
	int status = OS_SUCCESS;
	IOCTL_COMPAT_ARGS_NODE local_args_compat;
	IOCTL_ARGS_NODE local_args;

	memset(&local_args_compat, 0, sizeof(IOCTL_COMPAT_ARGS_NODE));
	if (arg) {
		status = copy_from_user(&local_args_compat,
					(void __user *)arg,
					sizeof(IOCTL_COMPAT_ARGS_NODE));
		if (status != OS_SUCCESS)
			return status;
	}

	local_args.len_drv_to_usr = local_args_compat.len_drv_to_usr;
	local_args.len_usr_to_drv = local_args_compat.len_usr_to_drv;
	local_args.buf_drv_to_usr =
		(char *)compat_ptr(local_args_compat.buf_drv_to_usr);
	local_args.buf_usr_to_drv =
		(char *)compat_ptr(local_args_compat.buf_usr_to_drv);

	if (cmd == PAX_IOCTL_COMPAT_INFO) {
		cmd = PAX_IOCTL_INFO;
	}
	local_args.command = cmd;

	status = pax_Service_IOCTL(filp, cmd, local_args);

	return status;
}
#endif

// **************************************************************************
//
// PAX device file operation definitions (required by kernel)
//
// **************************************************************************

/*
 * Structure that declares the usual file access functions
 * First one is for pax, the control functions
 */
static struct file_operations pax_Fops = {
	.owner = THIS_MODULE,
	IOCTL_OP = pax_Device_Control,
#if defined(CONFIG_COMPAT) && defined(DRV_EM64T)
	.compat_ioctl = pax_Device_Control_Compat,
#endif
	.read = NULL,
	.write = NULL,
	.open = pax_Open,
	.release = NULL,
	.llseek = NULL,
};

/* ------------------------------------------------------------------------- */
/*!
 * @fn     int pax_Setup_Cdev()
 *
 * @param  dev    - pointer to the device object
 * @param  devnum - major/minor device number
 * @param  fops   - point to file operations struct
 *
 * @return int
 *
 * @brief  Set up functions to be handled by PAX device
 *
 * <I>Special Notes</I>
 */
static int pax_Setup_Cdev(PAX_DEV dev, struct file_operations *fops,
			  dev_t devnum)
{
	cdev_init(&PAX_DEV_cdev(dev), fops);
	PAX_DEV_cdev(dev).owner = THIS_MODULE;
	PAX_DEV_cdev(dev).ops = fops;

	return cdev_add(&PAX_DEV_cdev(dev), devnum, 1);
}

static int pax_version_proc_read(struct seq_file *file, void *v)
{
	seq_printf(file, "%u", PAX_VERSION_NODE_version(&pax_version));

	return 0;
}

static int pax_version_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, pax_version_proc_read, NULL);
}

// **************************************************************************
//
// Exported PAX functions (see pax.h) ; will appear under /proc/kallsyms
//
// **************************************************************************

/* ------------------------------------------------------------------------- */
/*!
 * @fn     int pax_Load()
 *
 * @param  none
 *
 * @return int
 *
 * @brief  Load the PAX subsystem
 *
 * <I>Special Notes</I>
 */
int pax_Load(void)
{
	int result;
	struct device *pax_device;

	pax_control = NULL;

	PAX_PRINT_DEBUG("checking for %s interface...\n", PAX_NAME);

	/* If PAX interface does not exist, create it */
	pax_devnum = MKDEV(0, 0);
	PAX_PRINT_DEBUG("got major device %d\n", pax_devnum);
	/* allocate character device */
	result = alloc_chrdev_region(&pax_devnum, 0, 1, PAX_NAME);
	if (result < 0) {
		PAX_PRINT_ERROR("unable to alloc chrdev_region for %s!\n",
				PAX_NAME);
		return result;
	}

	pax_class = class_create(THIS_MODULE, "pax");
	if (IS_ERR(pax_class)) {
		PAX_PRINT_ERROR("Error registering pax class\n");
	}
	pax_device = device_create(pax_class, NULL, pax_devnum, NULL, "pax");
	if (pax_device == NULL) {
		return OS_INVALID;
	}

	PAX_PRINT_DEBUG("%s major number is %d\n", PAX_NAME, MAJOR(pax_devnum));
	/* Allocate memory for the PAX control device */
	pax_control = (PVOID)kzalloc(sizeof(PAX_DEV_NODE), GFP_KERNEL);
	if (!pax_control) {
		PAX_PRINT_ERROR("Unable to allocate memory for %s device\n",
				PAX_NAME);
		return OS_NO_MEM;
	}
	// /* Initialize memory for the PAX control device */
	// memset(pax_control, '\0', sizeof(PAX_DEV_NODE));
	/* Register PAX file operations with the OS */
	result = pax_Setup_Cdev(pax_control, &pax_Fops, pax_devnum);
	if (result) {
		PAX_PRINT_ERROR("Unable to add %s as char device (error=%d)\n",
				PAX_NAME, result);
		return result;
	}

	pax_Init();

	pax_version_file =
		proc_create("pax_version", 0, NULL, &pax_version_ops);
	if (pax_version_file == NULL) {
		SEP_PRINT_ERROR("Unalbe to create the pax_version proc file\n");
	}

	//
	// Display driver version information
	//
	PAX_PRINT("PMU arbitration service v%d.%d.%d has been started.\n",
		  PAX_VERSION_NODE_major(&pax_version),
		  PAX_VERSION_NODE_minor(&pax_version),
		  PAX_VERSION_NODE_bugfix(&pax_version));

	return result;
}

EXPORT_SYMBOL(pax_Load);

/* ------------------------------------------------------------------------- */
/*!
 * @fn     int pax_Unload()
 *
 * @param  none
 *
 * @return none
 *
 * @brief  Unload the PAX subsystem
 *
 * <I>Special Notes</I>
 */
void pax_Unload(void)
{
	// warn if unable to unreserve
	if (pax_Unreserve() != OS_SUCCESS) {
		PAX_PRINT_WARNING(
			"Unloading driver with existing reservation ....");
		PAX_PRINT_WARNING("         guid = %lu\n",
				  (long unsigned int)pax_status.guid);
		PAX_PRINT_WARNING("          pid = %ld\n",
				  (long int)pax_status.pid);
		PAX_PRINT_WARNING("   start_time = %lu\n",
				  (long unsigned int)pax_status.start_time);
		PAX_PRINT_WARNING("  is_reserved = %u\n",
				  pax_status.is_reserved);
	}

	// unregister PAX device
	unregister_chrdev(MAJOR(pax_devnum), "pax");
	device_destroy(pax_class, pax_devnum);
	class_destroy(pax_class);

	cdev_del(&PAX_DEV_cdev(pax_control));
	unregister_chrdev_region(pax_devnum, 1);
	if (pax_control != NULL) {
		kfree(pax_control);
	}

	remove_proc_entry("pax_version", NULL);

	//
	// Display driver version information
	//
	PAX_PRINT("PMU arbitration service v%d.%d.%d has been stopped.\n",
		  PAX_VERSION_NODE_major(&pax_version),
		  PAX_VERSION_NODE_minor(&pax_version),
		  PAX_VERSION_NODE_bugfix(&pax_version));

	// clean up resources used by PAX
	pax_Cleanup();

}

EXPORT_SYMBOL(pax_Unload);

/* Declaration of the init and exit functions */
module_init(pax_Load);
module_exit(pax_Unload);
