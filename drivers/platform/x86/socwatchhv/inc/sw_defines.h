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

#ifndef _PW_DEFINES_H_
#define _PW_DEFINES_H_ 1

#include "sw_version.h"

/* ***************************************************
 * Common to kernel and userspace.
 * ***************************************************
 */
#define PW_SUCCESS 0
#define PW_ERROR 1
#define PW_SUCCESS_NO_COLLECT 2

/*
 * Helper macro to convert 'u64' to 'unsigned long long' to avoid gcc warnings.
 */
#define TO_ULL(x) (unsigned long long)(x)
/*
 * Convert an arg to 'long long'
 */
#define TO_LL(x) (long long)(x)
/*
 * Convert an arg to 'unsigned long'
 */
#define TO_UL(x) (unsigned long)(x)
/*
 * Helper macro for string representation of a boolean value.
 */
#define GET_BOOL_STRING(b) ((b) ? "TRUE" : "FALSE")

/*
 * Circularly increment 'i' MODULO 'l'.
 * ONLY WORKS IF 'l' is (power of 2 - 1) ie.
 * l == (2 ^ x) - 1
 */
#define CIRCULAR_INC(index, mask) (((index) + 1) & (mask))
#define CIRCULAR_ADD(index, val, mask) (((index) + (val)) & (mask))
/*
 * Circularly decrement 'i'.
 */
#define CIRCULAR_DEC(i, m)                                                     \
	({                                                                     \
		int __tmp1 = (i);                                              \
		if (--__tmp1 < 0)                                              \
			__tmp1 = (m);                                          \
		__tmp1;                                                        \
	})
/*
 * Retrieve size of an array.
 */
#define SW_ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
/*
 * Should the driver count number of dropped samples?
 */
#define DO_COUNT_DROPPED_SAMPLES 1
/*
 * Extract F/W major, minor versions.
 * Assumes version numbers are 8b unsigned ints.
 */
#define SW_GET_SCU_FW_VERSION_MAJOR(ver) (((ver) >> 8) & 0xff)
#define SW_GET_SCU_FW_VERSION_MINOR(ver) ((ver)&0xff)
/*
 * Max size of process name retrieved from kernel.
 */
#define SW_MAX_PROC_NAME_SIZE 16

/*
 * Number of SOCPERF counters.
 * Needed by both Ring-0 and Ring-3
 */
#define SW_NUM_SOCPERF_COUNTERS 9

/*
 * Max size of process name retrieved from kernel space.
 */
#define SW_MAX_PROC_NAME_SIZE 16
/*
 * Max size of kernel wakelock name.
 */
#define SW_MAX_KERNEL_WAKELOCK_NAME_SIZE 100

/* Data value read when a telemetry data read fails. */
#define SW_TELEM_READ_FAIL_VALUE 0xF00DF00DF00DF00D

#ifdef SWW_MERGE
typedef enum {
	SW_STOP_EVENT = 0,
	SW_CS_EXIT_EVENT,
	SW_COUNTER_RESET_EVENT,
	SW_COUNTER_HOTKEY_EVENT,
	SW_MAX_COLLECTION_EVENT
} collector_stop_event_t;
#endif /* SWW_MERGE */

#define MAX_UNSIGNED_16_BIT_VALUE 0xFFFF
#define MAX_UNSIGNED_24_BIT_VALUE 0xFFFFFF
#define MAX_UNSIGNED_32_BIT_VALUE 0xFFFFFFFF
#define MAX_UNSIGNED_64_BIT_VALUE 0xFFFFFFFFFFFFFFFF

#endif /* _PW_DEFINES_H_ */
