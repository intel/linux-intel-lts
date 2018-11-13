/*

  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY

  Copyright(c) 2014 - 2018 Intel Corporation.

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  Contact Information:
  SoC Watch Developer Team <socwatchdevelopers@intel.com>
  Intel Corporation,
  1300 S Mopac Expwy,
  Austin, TX 78746

  BSD LICENSE

  Copyright(c) 2014 - 2018 Intel Corporation.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#ifndef __SW_IOCTL_H__
#define __SW_IOCTL_H__ 1

#if defined(__linux__) || defined(__QNX__)
#if __KERNEL__
#include <linux/ioctl.h>
#if defined(CONFIG_COMPAT) && defined(CONFIG_X86_64)
#include <asm/compat.h>
#include <linux/compat.h>
#endif // COMPAT && x64
#else // !__KERNEL__
#include <sys/ioctl.h>
#endif // __KERNEL__
#endif // __linux__
/*
 * Ensure we pull in definition of 'DO_COUNT_DROPPED_SAMPLES'!
 */
#include "sw_defines.h"

#ifdef ONECORE
#ifndef __KERNEL__
#include <winioctl.h>
#endif //__KERNEL__
#endif // ONECORE

/*
 * The APWR-specific IOCTL magic
 * number -- used to ensure IOCTLs
 * are delivered to the correct
 * driver.
 */
// #define APWR_IOCTL_MAGIC_NUM 0xdead
#define APWR_IOCTL_MAGIC_NUM 100

/*
 * The name of the device file
 */
// #define DEVICE_FILE_NAME "/dev/pw_driver_char_dev"
#define PW_DEVICE_FILE_NAME "/dev/apwr_driver_char_dev"
#define PW_DEVICE_NAME "apwr_driver_char_dev"

enum sw_ioctl_cmd {
	sw_ioctl_cmd_none = 0,
	sw_ioctl_cmd_config,
	sw_ioctl_cmd_cmd,
	sw_ioctl_cmd_poll,
	sw_ioctl_cmd_immediate_io,
	sw_ioctl_cmd_scu_version,
	sw_ioctl_cmd_read_immediate,
	sw_ioctl_cmd_driver_version,
	sw_ioctl_cmd_avail_trace,
	sw_ioctl_cmd_avail_notify,
	sw_ioctl_cmd_avail_collect,
	sw_ioctl_cmd_topology_changes,
};
/*
 * The actual IOCTL commands.
 *
 * From the kernel documentation:
 * "_IOR" ==> Read IOCTL
 * "_IOW" ==> Write IOCTL
 * "_IOWR" ==> Read/Write IOCTL
 *
 * Where "Read" and "Write" are from the user's perspective
 * (similar to the file "read" and "write" calls).
 */
#ifdef SWW_MERGE // Windows
//
// Device type           -- in the "User Defined" range."
//
#define POWER_I_CONF_TYPE 40000

// List assigned tracepoint id
#define CSIR_TRACEPOINT_ID_MASK 1
#define DEVICE_STATE_TRACEPOINT_ID_MASK 2
#define CSIR_SEPARATE_TRACEPOINT_ID_MASK 3
#define RESET_TRACEPOINT_ID_MASK 4
#define DISPLAY_ON_TRACEPOINT_ID_MASK 5

#ifdef SWW_MERGE
//
// TELEM BAR CONFIG
//
#define MAX_TELEM_BAR_CFG 3
#define TELEM_MCHBAR_CFG 0
#define TELEM_IPC1BAR_CFG 1
#define TELEM_SSRAMBAR_CFG 2
#endif

//
// The IOCTL function codes from 0x800 to 0xFFF are for customer use.
//
#define PW_IOCTL_CONFIG                                                        \
	CTL_CODE(POWER_I_CONF_TYPE, 0x900, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PW_IOCTL_START_COLLECTION                                              \
	CTL_CODE(POWER_I_CONF_TYPE, 0x901, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PW_IOCTL_STOP_COLLECTION                                               \
	CTL_CODE(POWER_I_CONF_TYPE, 0x902, METHOD_BUFFERED, FILE_ANY_ACCESS)

// TODO: pause, resume, cancel not supported yet
#define PW_IOCTL_PAUSE_COLLECTION                                              \
	CTL_CODE(POWER_I_CONF_TYPE, 0x903, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PW_IOCTL_RESUME_COLLECTION                                             \
	CTL_CODE(POWER_I_CONF_TYPE, 0x904, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PW_IOCTL_CANCEL_COLLECTION                                             \
	CTL_CODE(POWER_I_CONF_TYPE, 0x905, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define PW_IOCTL_GET_PROCESSOR_GROUP_TOPOLOGY                                  \
	CTL_CODE(POWER_I_CONF_TYPE, 0x906, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PW_IOCTL_TOPOLOGY                                                      \
	CTL_CODE(POWER_I_CONF_TYPE, 0x907, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PW_IOCTL_GET_AVAILABLE_COLLECTORS                                      \
	CTL_CODE(POWER_I_CONF_TYPE, 0x908, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PW_IOCTL_IMMEDIATE_IO                                                  \
	CTL_CODE(POWER_I_CONF_TYPE, 0x909, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PW_IOCTL_DRV_CLEANUP                                                   \
	CTL_CODE(POWER_I_CONF_TYPE, 0x90A, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PW_IOCTL_SET_COLLECTION_EVENT                                          \
	CTL_CODE(POWER_I_CONF_TYPE, 0x90B, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PW_IOCTL_TRY_STOP_EVENT                                                \
	CTL_CODE(POWER_I_CONF_TYPE, 0x90C, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PW_IOCTL_SET_PCH_ACTIVE_INTERVAL                                       \
	CTL_CODE(POWER_I_CONF_TYPE, 0x90D, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PW_IOCTL_SET_TELEM_BAR                                                 \
	CTL_CODE(POWER_I_CONF_TYPE, 0x90E, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PW_IOCTL_METADATA                                                      \
	CTL_CODE(POWER_I_CONF_TYPE, 0x90F, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PW_IOCTL_SET_GBE_INTERVAL                                              \
	CTL_CODE(POWER_I_CONF_TYPE, 0x910, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PW_IOCTL_ENABLE_COLLECTION                                             \
	CTL_CODE(POWER_I_CONF_TYPE, 0x911, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PW_IOCTL_DISABLE_COLLECTION                                            \
	CTL_CODE(POWER_I_CONF_TYPE, 0x912, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define PW_IOCTL_DRIVER_BUILD_DATE                                             \
	CTL_CODE(POWER_I_CONF_TYPE, 0x913, METHOD_BUFFERED, FILE_ANY_ACCESS)

#elif !defined(__APPLE__)
#define PW_IOCTL_CONFIG                                                        \
	_IOW(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_config,                        \
	     struct sw_driver_ioctl_arg *)
#if DO_COUNT_DROPPED_SAMPLES
#define PW_IOCTL_CMD                                                           \
	_IOWR(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_cmd,                          \
	      struct sw_driver_ioctl_arg *)
#else
#define PW_IOCTL_CMD                                                           \
	_IOW(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_cmd,                           \
	     struct sw_driver_ioctl_arg *)
#endif // DO_COUNT_DROPPED_SAMPLES
#define PW_IOCTL_POLL _IO(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_poll)
#define PW_IOCTL_IMMEDIATE_IO                                                  \
	_IOWR(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_immediate_io,                 \
	      struct sw_driver_ioctl_arg *)
#define PW_IOCTL_GET_SCU_FW_VERSION                                            \
	_IOR(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_scu_version,                   \
	     struct sw_driver_ioctl_arg *)
#define PW_IOCTL_READ_IMMEDIATE                                                \
	_IOWR(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_read_immediate,               \
	      struct sw_driver_ioctl_arg *)
#define PW_IOCTL_GET_DRIVER_VERSION                                            \
	_IOR(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_driver_version,                \
	     struct sw_driver_ioctl_arg *)
#define PW_IOCTL_GET_AVAILABLE_TRACEPOINTS                                     \
	_IOR(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_avail_trace,                   \
	     struct sw_driver_ioctl_arg *)
#define PW_IOCTL_GET_AVAILABLE_NOTIFIERS                                       \
	_IOR(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_avail_notify,                  \
	     struct sw_driver_ioctl_arg *)
#define PW_IOCTL_GET_AVAILABLE_COLLECTORS                                      \
	_IOR(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_avail_collect,                 \
	     struct sw_driver_ioctl_arg *)
#define PW_IOCTL_GET_TOPOLOGY_CHANGES                                          \
	_IOR(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_topology_changes,              \
	     struct sw_driver_ioctl_arg *)
#else // __APPLE__
#define PW_IOCTL_CONFIG                                                        \
	_IOW(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_config,                        \
	     struct sw_driver_ioctl_arg)
#if DO_COUNT_DROPPED_SAMPLES
#define PW_IOCTL_CMD                                                           \
	_IOWR(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_cmd,                          \
	      struct sw_driver_ioctl_arg)
#else
#define PW_IOCTL_CMD                                                           \
	_IOW(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_cmd, struct sw_driver_ioctl_arg)
#endif // DO_COUNT_DROPPED_SAMPLES
#define PW_IOCTL_POLL _IO(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_poll)
#define PW_IOCTL_IMMEDIATE_IO                                                  \
	_IOWR(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_immediate_io,                 \
	      struct sw_driver_ioctl_arg)
#define PW_IOCTL_GET_SCU_FW_VERSION                                            \
	_IOWR(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_scu_version,                  \
	      struct sw_driver_ioctl_arg)
#define PW_IOCTL_READ_IMMEDIATE                                                \
	_IOWR(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_read_immediate,               \
	      struct sw_driver_ioctl_arg)
#define PW_IOCTL_GET_DRIVER_VERSION                                            \
	_IOWR(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_driver_version,               \
	      struct sw_driver_ioctl_arg)
#define PW_IOCTL_GET_AVAILABLE_TRACEPOINTS                                     \
	_IOWR(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_avail_trace,                  \
	      struct sw_driver_ioctl_arg)
#define PW_IOCTL_GET_AVAILABLE_NOTIFIERS                                       \
	_IOWR(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_avail_notify,                 \
	      struct sw_driver_ioctl_arg)
#define PW_IOCTL_GET_AVAILABLE_COLLECTORS                                      \
	_IOWR(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_avail_collect,                \
	      struct sw_driver_ioctl_arg)
#define PW_IOCTL_GET_TOPOLOGY_CHANGES                                          \
	_IOWR(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_topology_changes,             \
	      struct sw_driver_ioctl_arg)
#endif // __APPLE__

/*
 * 32b-compatible version of the above
 * IOCTL numbers. Required ONLY for
 * 32b compatibility on 64b systems,
 * and ONLY by the driver.
 */
#if defined(CONFIG_COMPAT) && defined(CONFIG_X86_64)
#define PW_IOCTL_CONFIG32                                                      \
	_IOW(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_config, compat_uptr_t)
#if DO_COUNT_DROPPED_SAMPLES
#define PW_IOCTL_CMD32                                                         \
	_IOWR(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_cmd, compat_uptr_t)
#else
#define PW_IOCTL_CMD32                                                         \
	_IOW(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_cmd, compat_uptr_t)
#endif // DO_COUNT_DROPPED_SAMPLES
#define PW_IOCTL_POLL32 _IO(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_poll)
#define PW_IOCTL_IMMEDIATE_IO32                                                \
	_IOWR(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_immediate_io, compat_uptr_t)
#define PW_IOCTL_GET_SCU_FW_VERSION32                                          \
	_IOR(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_scu_version, compat_uptr_t)
#define PW_IOCTL_READ_IMMEDIATE32                                              \
	_IOWR(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_read_immediate, compat_uptr_t)
#define PW_IOCTL_GET_DRIVER_VERSION32                                          \
	_IOR(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_driver_version, compat_uptr_t)
#define PW_IOCTL_GET_AVAILABLE_TRACEPOINTS32                                   \
	_IOR(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_avail_trace, compat_uptr_t)
#define PW_IOCTL_GET_AVAILABLE_NOTIFIERS32                                     \
	_IOR(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_avail_notify, compat_uptr_t)
#define PW_IOCTL_GET_AVAILABLE_COLLECTORS32                                    \
	_IOR(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_avail_collect, compat_uptr_t)
#define PW_IOCTL_GET_TOPOLOGY_CHANGES32                                        \
	_IOR(APWR_IOCTL_MAGIC_NUM, sw_ioctl_cmd_topology_changes, compat_uptr_t)
#endif // defined(CONFIG_COMPAT) && defined(CONFIG_X86_64)
#endif // __SW_IOCTL_H__
