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

#ifndef _LWPMUDRV_IOCTL_H_
#define _LWPMUDRV_IOCTL_H_

#if defined(__cplusplus)
extern "C" {
#endif

//SEP Driver Operation defines
/*
	"NOTE THAT the definition must be identical across all OSes"
	"DO NOT add any OS specific compile flag"
*/
#define DRV_OPERATION_START 1
#define DRV_OPERATION_STOP 2
#define DRV_OPERATION_INIT_PMU 3
#define DRV_OPERATION_INIT 4
#define DRV_OPERATION_EM_GROUPS 5
#define DRV_OPERATION_SET_CPU_MASK 17
#define DRV_OPERATION_PCI_READ 18
#define DRV_OPERATION_PCI_WRITE 19
#define DRV_OPERATION_READ_PCI_CONFIG 20
#define DRV_OPERATION_FD_PHYS 21
#define DRV_OPERATION_WRITE_PCI_CONFIG 22
#define DRV_OPERATION_INSERT_MARKER 23
#define DRV_OPERATION_GET_NORMALIZED_TSC 24
#define DRV_OPERATION_EM_CONFIG_NEXT 25
#define DRV_OPERATION_SYS_CONFIG 26
#define DRV_OPERATION_TSC_SKEW_INFO 27
#define DRV_OPERATION_NUM_CORES 28
#define DRV_OPERATION_COLLECT_SYS_CONFIG 29
#define DRV_OPERATION_GET_SYS_CONFIG 30
#define DRV_OPERATION_PAUSE 31
#define DRV_OPERATION_RESUME 32
#define DRV_OPERATION_SET_ASYNC_EVENT 33
#define DRV_OPERATION_ASYNC_STOP 34
#define DRV_OPERATION_TERMINATE 35
#define DRV_OPERATION_READ_MSRS 36
#define DRV_OPERATION_LBR_INFO 37
#define DRV_OPERATION_RESERVE 38
#define DRV_OPERATION_MARK 39
#define DRV_OPERATION_AWAIT_STOP 40
#define DRV_OPERATION_SEED_NAME 41
#define DRV_OPERATION_KERNEL_CS 42
#define DRV_OPERATION_SET_UID 43
#define DRV_OPERATION_VERSION 51
#define DRV_OPERATION_CHIPSET_INIT 52
#define DRV_OPERATION_GET_CHIPSET_DEVICE_ID 53
#define DRV_OPERATION_SWITCH_GROUP 54
#define DRV_OPERATION_GET_NUM_CORE_CTRS 55
#define DRV_OPERATION_PWR_INFO 56
#define DRV_OPERATION_NUM_DESCRIPTOR 57
#define DRV_OPERATION_DESC_NEXT 58
#define DRV_OPERATION_MARK_OFF 59
#define DRV_OPERATION_CREATE_MARKER 60
#define DRV_OPERATION_GET_DRIVER_STATE 61
#define DRV_OPERATION_READ_SWITCH_GROUP 62
#define DRV_OPERATION_EM_GROUPS_UNC 63
#define DRV_OPERATION_EM_CONFIG_NEXT_UNC 64
#define DRV_OPERATION_INIT_UNC 65
#define DRV_OPERATION_RO_INFO 66
#define DRV_OPERATION_READ_MSR 67
#define DRV_OPERATION_WRITE_MSR 68
#define DRV_OPERATION_THREAD_SET_NAME 69
#define DRV_OPERATION_GET_PLATFORM_INFO 70
#define DRV_OPERATION_GET_NORMALIZED_TSC_STANDALONE 71
#define DRV_OPERATION_READ_AND_RESET 72
#define DRV_OPERATION_SET_CPU_TOPOLOGY 73
#define DRV_OPERATION_INIT_NUM_DEV 74
#define DRV_OPERATION_SET_GFX_EVENT 75
#define DRV_OPERATION_GET_NUM_SAMPLES 76
#define DRV_OPERATION_SET_PWR_EVENT 77
#define DRV_OPERATION_SET_DEVICE_NUM_UNITS 78
#define DRV_OPERATION_TIMER_TRIGGER_READ 79
#define DRV_OPERATION_GET_INTERVAL_COUNTS 80
#define DRV_OPERATION_FLUSH 81
#define DRV_OPERATION_SET_SCAN_UNCORE_TOPOLOGY_INFO 82
#define DRV_OPERATION_GET_UNCORE_TOPOLOGY 83
#define DRV_OPERATION_GET_MARKER_ID 84
#define DRV_OPERATION_GET_SAMPLE_DROP_INFO 85
#define DRV_OPERATION_GET_DRV_SETUP_INFO 86
#define DRV_OPERATION_GET_PLATFORM_TOPOLOGY 87
#define DRV_OPERATION_GET_THREAD_COUNT 88
#define DRV_OPERATION_GET_THREAD_INFO 89
#define DRV_OPERATION_GET_DRIVER_LOG 90
#define DRV_OPERATION_CONTROL_DRIVER_LOG 91
#define DRV_OPERATION_SET_OSID 92
#define DRV_OPERATION_GET_AGENT_MODE 93
#define DRV_OPERATION_INIT_DRIVER 94
#define DRV_OPERATION_SET_EMON_BUFFER_DRIVER_HELPER 95
#define DRV_OPERATION_GET_NUM_VM 96
#define DRV_OPERATION_GET_VCPU_MAP 97

// Only used by MAC OS
#define DRV_OPERATION_GET_ASLR_OFFSET 997 // this may not need
#define DRV_OPERATION_SET_OSX_VERSION 998
#define DRV_OPERATION_PROVIDE_FUNCTION_PTRS 999

// IOCTL_SETUP

// IOCTL_ARGS
typedef struct IOCTL_ARGS_NODE_S IOCTL_ARGS_NODE;
typedef IOCTL_ARGS_NODE * IOCTL_ARGS;

#if defined(DRV_EM64T)
struct IOCTL_ARGS_NODE_S {
	U64 len_drv_to_usr;
	// buffer send from driver(target) to user(host), stands for read buffer
	char *buf_drv_to_usr;
	// length of the driver(target) to user(host) buffer
	U64 len_usr_to_drv;
	// buffer send from user(host) to driver(target) stands for write buffer
	char *buf_usr_to_drv; // length of user(host) to driver(target) buffer
	U32 command;
};
#endif
#if defined(DRV_IA32)
struct IOCTL_ARGS_NODE_S {
	U64 len_drv_to_usr;
	// buffer send from driver(target) to user(host),stands for read buffer
	char *buf_drv_to_usr; // length of driver(target) to user(host) buffer
	char *reserved1;
	U64 len_usr_to_drv;
	// send from user(host) to driver(target),stands for write buffer
	char *buf_usr_to_drv; // length of user(host) to driver(target) buffer
	char *reserved2;
	U32 command;
};
#endif

#if defined(DRV_OS_WINDOWS)

//
// NtDeviceIoControlFile IoControlCode values for this device.
//
// Warning:  Remember that the low two bits of the code specify how the
//           buffers are passed to the driver!
//
// 16 bit device type. 12 bit function codes
#define LWPMUDRV_IOCTL_DEVICE_TYPE 0xA000
// values 0-32768 reserved for Microsoft
#define LWPMUDRV_IOCTL_FUNCTION 0x0A00 // values 0-2047  reserved for Microsoft

//
// Basic CTL CODE macro to reduce typographical errors
// Use for FILE_READ_ACCESS
//
#define LWPMUDRV_CTL_READ_CODE(x)                                              \
	CTL_CODE(LWPMUDRV_IOCTL_DEVICE_TYPE, LWPMUDRV_IOCTL_FUNCTION + (x),    \
		 METHOD_BUFFERED, FILE_READ_ACCESS)

/* Refernece https://docs.microsoft.com/en-us/windows-hardware/drivers/kernel/defining-i-o-control-codes
   CTL_CODE (DeviceType, Function, Method, Access) generates 32 bit code
	------------------------------------------------- ----------------
	|   31   | 30 ... 16 | 15      14 |   13   | 12  ... 2 | 1      0 |
	-------------------------------------------------------------------
	| common | device    | req access | custom | func code | transfer |
	|        |  type     |            |        |           |   type   |
	-------------------------------------------------------------------
*/
#define LWPMUDRV_DEVICE_TYPE(x) ((x & 0xFFFF0000) >> 16)
#define LWPMUDRV_METHOD(x) (x & 3)
#define LWPMUDRV_FUNCTION(x) (((x >> 2) & 0x00000FFF) - 0x0A00)

#define LWPMUDRV_IOCTL_CODE(x) LWPMUDRV_CTL_READ_CODE(x)

#elif defined(SEP_ESX)

typedef struct CPU_ARGS_NODE_S CPU_ARGS_NODE;
typedef CPU_ARGS_NODE * CPU_ARGS;
struct CPU_ARGS_NODE_S {
	U64 len_drv_to_usr;
	char *buf_drv_to_usr;
	U32 command;
	U32 CPU_ID;
	U32 BUCKET_ID;
};

// IOCTL_SETUP
#define LWPMU_IOC_MAGIC 99
#define OS_SUCCESS 0
#define OS_STATUS int
//#define OS_ILLEGAL_IOCTL  -ENOTTY
//#define OS_NO_MEM         -ENOMEM
//#define OS_FAULT          -EFAULT

#define LWPMUDRV_IOCTL_IO(x) (x)
#define LWPMUDRV_IOCTL_IOR(x) (x)
#define LWPMUDRV_IOCTL_IOW(x) (x)
#define LWPMUDRV_IOCTL_IORW(x) (x)

#elif defined(DRV_OS_LINUX) || defined(DRV_OS_SOLARIS) ||                      \
	defined(DRV_OS_ANDROID)
// IOCTL_ARGS

// COMPAT IOCTL_ARGS
#if defined(CONFIG_COMPAT) && defined(DRV_EM64T)
typedef struct IOCTL_COMPAT_ARGS_NODE_S IOCTL_COMPAT_ARGS_NODE;
typedef IOCTL_COMPAT_ARGS_NODE * IOCTL_COMPAT_ARGS;
struct IOCTL_COMPAT_ARGS_NODE_S {
	U64 len_drv_to_usr;
	compat_uptr_t buf_drv_to_usr;
	U64 len_usr_to_drv;
	compat_uptr_t buf_usr_to_drv;
};
#endif

// COMPAT IOCTL_SETUP
//
#define LWPMU_IOC_MAGIC 99

#if defined(CONFIG_COMPAT) && defined(DRV_EM64T)
#define LWPMUDRV_IOCTL_IO(x) _IO(LWPMU_IOC_MAGIC, (x))
#define LWPMUDRV_IOCTL_IOR(x) _IOR(LWPMU_IOC_MAGIC, (x), compat_uptr_t)
#define LWPMUDRV_IOCTL_IOW(x) _IOW(LWPMU_IOC_MAGIC, (x), compat_uptr_t)
#define LWPMUDRV_IOCTL_IORW(x) _IOW(LWPMU_IOC_MAGIC, (x), compat_uptr_t)
#else
#define LWPMUDRV_IOCTL_IO(x) _IO(LWPMU_IOC_MAGIC, (x))
#define LWPMUDRV_IOCTL_IOR(x) _IOR(LWPMU_IOC_MAGIC, (x), IOCTL_ARGS)
#define LWPMUDRV_IOCTL_IOW(x) _IOW(LWPMU_IOC_MAGIC, (x), IOCTL_ARGS)
#define LWPMUDRV_IOCTL_IORW(x) _IOW(LWPMU_IOC_MAGIC, (x), IOCTL_ARGS)
#endif

#elif defined(DRV_OS_FREEBSD)

// IOCTL_SETUP
//
#define LWPMU_IOC_MAGIC 99

/* FreeBSD is very strict about IOR/IOW/IOWR specifications on IOCTLs.
 * Since these IOCTLs all pass down the real read/write buffer lengths
 *  and addresses inside of an IOCTL_ARGS_NODE data structure, we
 *  need to specify all of these as _IOW so that the kernel will
 *  view it as userspace passing the data to the driver, rather than
 *  the reverse.  There are also some cases where Linux is passing
 *  a smaller type than IOCTL_ARGS_NODE, even though its really
 *  passing an IOCTL_ARGS_NODE.  These needed to be fixed for FreeBSD.
 */
#define LWPMUDRV_IOCTL_IO(x) _IO(LWPMU_IOC_MAGIC, (x))
#define LWPMUDRV_IOCTL_IOR(x) _IOW(LWPMU_IOC_MAGIC, (x), IOCTL_ARGS_NODE)
#define LWPMUDRV_IOCTL_IOW(x) _IOW(LWPMU_IOC_MAGIC, (x), IOCTL_ARGS_NODE)
#define LWPMUDRV_IOCTL_IORW(x) _IOW(LWPMU_IOC_MAGIC, (x), IOCTL_ARGS_NODE)

#elif defined(DRV_OS_MAC)

typedef struct CPU_ARGS_NODE_S CPU_ARGS_NODE;
typedef CPU_ARGS_NODE * CPU_ARGS;
struct CPU_ARGS_NODE_S {
	U64 len_drv_to_usr;
	char *buf_drv_to_usr;
	U32 command;
	U32 CPU_ID;
	U32 BUCKET_ID;
};

// IOCTL_SETUP
//
#define LWPMU_IOC_MAGIC 99
#define OS_SUCCESS 0
#define OS_STATUS int
#define OS_ILLEGAL_IOCTL -ENOTTY
#define OS_NO_MEM -ENOMEM
#define OS_FAULT -EFAULT

// Task file Opcodes.
// keeping the definitions as IOCTL but in MAC OSX
// these are really OpCodes consumed by Execute command.

#else
#error "unknown OS in lwpmudrv_ioctl.h"
#endif

#if defined(__cplusplus)
}
#endif

#endif
