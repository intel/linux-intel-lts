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
#ifndef __SW_STRUCTS_H__
#define __SW_STRUCTS_H__ 1

#include "sw_types.h"

/*
 * An enumeration of MSR types.
 * Required if we want to differentiate
 * between different types of MSRs.
 */
enum sw_msr_type {
	SW_MSR_TYPE_THREAD,
	SW_MSR_TYPE_CORE,
	SW_MSR_TYPE_MODULE,
	SW_MSR_TYPE_PACKAGE,
	SW_MSR_TYPE_SOC,
	SW_MSR_TYPE_MAX,
};

/*
 * Convenience for a 'string' data type.
 * Not strictly required.
 */
#pragma pack(push, 1)
typedef struct sw_string_type {
	pw_u16_t len;
	char data[1];
} sw_string_type_t;
#pragma pack(pop)
#define SW_STRING_TYPE_HEADER_SIZE()                                           \
	(sizeof(struct sw_string_type) - sizeof(char[1]))

#pragma pack(push, 1)
struct sw_key_value_payload {
	pw_u16_t m_numKeyValuePairs;
	char data[1];
};
#pragma pack(pop)
#define SW_KEY_VALUE_PAYLOAD_HEADER_SIZE()                                     \
	(sizeof(struct sw_key_value_payload) - sizeof(char[1]))

typedef enum sw_kernel_wakelock_type {
	SW_WAKE_LOCK = 0, /* A kernel wakelock was acquired */
	SW_WAKE_UNLOCK = 1, /* A kernel wakelock was released */
	SW_WAKE_LOCK_TIMEOUT =
		2, /* A kernel wakelock was acquired with a timeout */
	SW_WAKE_LOCK_INITIAL = 3, /* A kernel wakelock was acquired
				   * before the
				   * collection started
				   */
	SW_WAKE_UNLOCK_ALL = 4, /* All previously held kernel wakelocks were
				 * released -- used in ACPI S3 notifications
				 */
} sw_kernel_wakelock_type_t;

typedef enum sw_when_type {
	SW_WHEN_TYPE_BEGIN = 0, /* Start snapshot */
	SW_WHEN_TYPE_POLL,
	SW_WHEN_TYPE_NOTIFIER,
	SW_WHEN_TYPE_TRACEPOINT,
	SW_WHEN_TYPE_END, /* Stop snapshot */
	SW_WHEN_TYPE_NONE
} sw_when_type_t;

/**
 * trigger_bits is defined to use type pw_u8_t
 * that makes only upto 8 types possible
 */
#define SW_TRIGGER_BEGIN_MASK() (1U << SW_WHEN_TYPE_BEGIN)
#define SW_TRIGGER_END_MASK() (1U << SW_WHEN_TYPE_END)
#define SW_TRIGGER_POLL_MASK() (1U << SW_WHEN_TYPE_POLL)
#define SW_TRIGGER_TRACEPOINT_MASK() (1U << SW_WHEN_TYPE_TRACEPOINT)
#define SW_TRIGGER_NOTIFIER_MASK() (1U << SW_WHEN_TYPE_NOTIFIER)
#define SW_GET_TRIGGER_MASK_VALUE(m) (1U << (m))
#define SW_TRIGGER_MASK_ALL() (0xFF)

enum sw_io_cmd { SW_IO_CMD_READ = 0, SW_IO_CMD_WRITE, SW_IO_CMD_MAX };

#pragma pack(push, 1)
struct sw_driver_msr_io_descriptor {
	pw_u64_t address;
	enum sw_msr_type type;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct sw_driver_ipc_mmio_io_descriptor {
	union {
#ifdef SWW_MERGE
#pragma warning(push)
#pragma warning(                                                               \
	disable : 4201) /* disable C4201: nonstandard extension used:
			 * nameless struct/union
			 */
#endif
		struct {
			pw_u16_t command;
			pw_u16_t sub_command;
		};
#ifdef SWW_MERGE
#pragma warning(pop) /* enable C4201 */
#endif
		union {
			pw_u32_t ipc_command; /* (sub_command << 12)
					       * | (command)
					       */
			pw_u8_t is_gbe; /* Used only for GBE MMIO */
		};
	};
	/* TODO: add a section for 'ctrl_address' and 'ctrl_remapped_address' */
	union {
		pw_u64_t data_address; /* Will be "io_remapped" */
		pw_u64_t data_remapped_address;
	};
};
#pragma pack(pop)

#pragma pack(push, 1)
struct sw_driver_pci_io_descriptor {
	pw_u32_t bus;
	pw_u32_t device;
	pw_u32_t function;
#ifdef __QNX__
	union {
		pw_u32_t offset;
		pw_u32_t index;
	};
#else /* __QNX__ */
	pw_u32_t offset;
#endif /* __QNX__ */
};
#pragma pack(pop)

#pragma pack(push, 1)
struct sw_driver_configdb_io_descriptor {
	/* pw_u32_t port; */
	/* pw_u32_t offset; */
	pw_u32_t address;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct sw_driver_trace_args_io_descriptor {
	pw_u8_t num_args; /* Number of valid entries in the 'args' array,
			   * below; 1 <= num_args <= 7
			   */
	pw_u8_t args[7]; /* Max of 7 args can be recorded */
};
#pragma pack(pop)

#pragma pack(push, 1)
/**
 * struct - sw_driver_telem_io_descriptor - Telemetry Metric descriptor
 *
 * @id:    (Client & Driver) Telemetry ID of the counter to read.
 * @idx:   (Driver only) index into telem array to read, or the row
 *            of the telem_indirect table to lookup the telem array index.
 * @unit:  Unit from which to collect:  0 = PMC, 1 = PUNIT
 *              Values come from the telemetry_unit enum.
 * @scale_op:  When there are multiple instances of a telem value (e.g.
 *              module C-states) the operation to use when scaling the CPU ID
 *              and adding it to the telemetry data ID.
 * @scale_val: Amount to scale an ID (when scaling one.)
 *
 * Like all hardware mechanism descriptors, the client uses this to pass
 * metric hardware properties (unit and ID) to the driver.  The driver
 * uses it to program the telemetry unit.
 *
 * Users can specify that IDs should be scaled based on the CPU id, using
 * the equation: ID = ID_value + (cpuid <scaling_op> <scaling_val>)
 * where <scaling_op> is one of +, *, /, or %, and scaling_val is an integer
 * value.  This gives you:
 *            Operation             scale_op     scale_val
 *       Single instance of an ID       *            0
 *       Sequentially increasing
 *          CPU-specific values         *            1
 *       Per module cpu-specific
 *          values (2 cores/module)     /            2
 *       Round Robin assignment         %         cpu_count
 *
 * Note that scaling_value of 0 implies that no scaling should be
 * applied.  While (*, 1) is equivalent to (+, 0), the scaling value of 0
 * is reserved/defined to mean "no scaling", and is disallowed.
 *
 * If you're really tight on space, you could always fold unit and
 * scale_op into a single byte without a lot of pain or even effort.
 */
struct sw_driver_telem_io_descriptor {
	union {
		pw_u16_t id;
		pw_u8_t idx;
	};
	pw_u8_t unit;
	pw_u8_t scale_op;
	pw_u16_t scale_val;
};
#pragma pack(pop)
enum telemetry_unit { TELEM_PUNIT = 0, TELEM_PMC, TELEM_UNIT_NONE };
#define TELEM_MAX_ID 0xFFFF /* Maximum value of a Telemtry event ID. */
#define TELEM_MAX_SCALE 0xFFFF /* Maximum ID scaling value. */
#define TELEM_OP_ADD '+' /* Addition operator */
#define TELEM_OP_MULT '*' /* Multiplication operator */
#define TELEM_OP_DIV '/' /* Division operator */
#define TELEM_OP_MOD '%' /* Modulus operator */
#define TELEM_OP_NONE 'X' /* No operator--Not a scaled ID */

#pragma pack(push, 1)
struct sw_driver_mailbox_io_descriptor {
	union {
		/*
		 * Will be "io_remapped"
		 */
		pw_u64_t interface_address;
		pw_u64_t interface_remapped_address;
	};
	union {
		/*
		 * Will be "io_remapped"
		 */
		pw_u64_t data_address;
		pw_u64_t data_remapped_address;
	};
	pw_u64_t command;
	pw_u64_t command_mask;
	pw_u16_t run_busy_bit;
	pw_u16_t is_msr_type;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct sw_driver_pch_mailbox_io_descriptor {
	union {
		/*
		 * Will be "io_remapped"
		 */
		pw_u64_t mtpmc_address;
		pw_u64_t mtpmc_remapped_address;
	};
	union {
		/*
		 * Will be "io_remapped"
		 */
		pw_u64_t msg_full_sts_address;
		pw_u64_t msg_full_sts_remapped_address;
	};
	union {
		/*
		 * Will be "io_remapped"
		 */
		pw_u64_t mfpmc_address;
		pw_u64_t mfpmc_remapped_address;
	};
	pw_u32_t data_address;
};
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct sw_driver_io_descriptor {
	pw_u16_t collection_type;
	/* TODO: specify READ/WRITE */
	pw_s16_t collection_command; /* One of 'enum sw_io_cmd' */
	pw_u16_t counter_size_in_bytes; /* The number of bytes to
					 * READ or WRITE
					 */
	union {
		struct sw_driver_msr_io_descriptor msr_descriptor;
		struct sw_driver_ipc_mmio_io_descriptor ipc_descriptor;
		struct sw_driver_ipc_mmio_io_descriptor mmio_descriptor;
		struct sw_driver_pci_io_descriptor pci_descriptor;
		struct sw_driver_configdb_io_descriptor configdb_descriptor;
		struct sw_driver_trace_args_io_descriptor trace_args_descriptor;
		struct sw_driver_telem_io_descriptor telem_descriptor;
		struct sw_driver_pch_mailbox_io_descriptor
			pch_mailbox_descriptor;
		struct sw_driver_mailbox_io_descriptor mailbox_descriptor;
	};
	pw_u64_t write_value; /* The value to WRITE */
} sw_driver_io_descriptor_t;
#pragma pack(pop)

/**
 * sw_driver_interface_info is used to map data collected by kernel-level
 * collectors to metrics.  The client passes one of these structs to the
 * driver for each metric the driver should collect.  The driver tags the
 * collected data (messages) using info from this struct. When processing
 * data from the driver, the client uses its copy of this data to
 * identify the plugin, metric, and message IDs of each message.
 */
#pragma pack(push, 1)
struct sw_driver_interface_info {
	pw_u64_t tracepoint_id_mask;
	pw_u64_t notifier_id_mask;
	pw_s16_t cpu_mask; /* On which CPU(s) should the driver
			    * read the data?
			    * Currently:  -2 ==> read on ALL CPUs,
			    *             -1 ==> read on ANY CPU,
			    *           >= 0 ==> the specific CPU to read on
			    */
	pw_s16_t plugin_id; /* Metric Plugin SID */
	pw_s16_t metric_id; /* Domain-specific ID assigned by
			     * each Metric Plugin
			     */
	pw_s16_t msg_id; /* Msg ID retrieved from the SoC Watch config file */
	pw_u16_t num_io_descriptors; /* Number of descriptors in the array,
				      * below.
				      */
	pw_u8_t trigger_bits; /* Mask of 'when bits' to fire this collector. */
	pw_u16_t sampling_interval_msec; /* Sampling interval, in msecs */
	pw_u8_t descriptors[1]; /* Array of sw_driver_io_descriptor structs. */
};
#pragma pack(pop)

#define SW_DRIVER_INTERFACE_INFO_HEADER_SIZE()                                 \
	(sizeof(struct sw_driver_interface_info) - sizeof(pw_u8_t[1]))

#pragma pack(push, 1)
struct sw_driver_interface_msg {
	pw_u16_t num_infos; /* Number of 'sw_driver_interface_info'
			     * structs contained within the 'infos' variable,
			     * below
			     */
	pw_u16_t min_polling_interval_msecs; /* Min time to wait before
					      * polling; used exclusively
					      * with the low overhead,
					      * context-switch based
					      * polling mode
					      */
	/* pw_u16_t infos_size_bytes; Size of data inlined
	 * within the 'infos' variable, below
	 */
	pw_u8_t infos[1];
};
#pragma pack(pop)
#define SW_DRIVER_INTERFACE_MSG_HEADER_SIZE()                                  \
	(sizeof(struct sw_driver_interface_msg) - sizeof(pw_u8_t[1]))

typedef enum sw_name_id_type {
	SW_NAME_TYPE_TRACEPOINT,
	SW_NAME_TYPE_NOTIFIER,
	SW_NAME_TYPE_COLLECTOR,
	SW_NAME_TYPE_MAX,
} sw_name_id_type_t;

#pragma pack(push, 1)
struct sw_name_id_pair {
	pw_u16_t id;
	pw_u16_t type; /* One of 'sw_name_id_type' */
	struct sw_string_type name;
};
#pragma pack(pop)
#define SW_NAME_ID_HEADER_SIZE()                                               \
	(sizeof(struct sw_name_id_pair) - sizeof(struct sw_string_type))

#pragma pack(push, 1)
struct sw_name_info_msg {
	pw_u16_t num_name_id_pairs;
	pw_u16_t payload_len;
	pw_u8_t pairs[1];
};
#pragma pack(pop)

/**
 * This is the basic data structure for passing data collected by the
 * kernel-level collectors up to the client.  In addition to the data
 * (payload), it contains the minimum metadata required for the client
 * to identify the source of that data.
 */
#pragma pack(push, 1)
typedef struct sw_driver_msg {
	pw_u64_t tsc;
	pw_u16_t cpuidx;
	pw_u8_t plugin_id; /* Cannot have more than 256 plugins */
	pw_u8_t metric_id; /* Each plugin cannot handle more than 256 metrics */
	pw_u8_t msg_id; /* Each metric cannot have more than 256 components */
	pw_u16_t payload_len;
	/* pw_u64_t p_payload; Ptr to payload */
	union {
		pw_u64_t __dummy; /* Ensure size of struct is consistent
				   * on x86, x64
				   */
		char *p_payload; /* Ptr to payload (collected data values). */
	};
} sw_driver_msg_t;
#pragma pack(pop)
#define SW_DRIVER_MSG_HEADER_SIZE()                                            \
	(sizeof(struct sw_driver_msg) - sizeof(pw_u64_t))

typedef enum sw_driver_collection_cmd {
	SW_DRIVER_START_COLLECTION = 1,
	SW_DRIVER_STOP_COLLECTION = 2,
	SW_DRIVER_PAUSE_COLLECTION = 3,
	SW_DRIVER_RESUME_COLLECTION = 4,
	SW_DRIVER_CANCEL_COLLECTION = 5,
} sw_driver_collection_cmd_t;

#pragma pack(push, 1)
struct sw_driver_version_info {
	pw_u16_t major;
	pw_u16_t minor;
	pw_u16_t other;
};
#pragma pack(pop)

enum cpu_action {
	SW_CPU_ACTION_NONE,
	SW_CPU_ACTION_OFFLINE,
	SW_CPU_ACTION_ONLINE_PREPARE,
	SW_CPU_ACTION_ONLINE,
	SW_CPU_ACTION_MAX,
};
#pragma pack(push, 1)
struct sw_driver_topology_change {
	pw_u64_t timestamp; /* timestamp */
	enum cpu_action type; /* One of 'enum cpu_action' */
	pw_u16_t cpu; /* logical cpu */
	pw_u16_t core; /* core id */
	pw_u16_t pkg; /* pkg/physical id */
};
struct sw_driver_topology_msg {
	pw_u16_t num_entries;
	pw_u8_t topology_entries[1];
};
#pragma pack(pop)

/**
 * An enumeration of possible pm states that
 * SoC Watch is interested in
 */
enum sw_pm_action {
	SW_PM_ACTION_NONE,
	SW_PM_ACTION_SUSPEND_ENTER,
	SW_PM_ACTION_SUSPEND_EXIT,
	SW_PM_ACTION_HIBERNATE_ENTER,
	SW_PM_ACTION_HIBERNATE_EXIT,
	SW_PM_ACTION_MAX,
};

/**
 * An enumeration of possible actions that trigger
 * the power notifier
 */
enum sw_pm_mode {
	SW_PM_MODE_FIRMWARE,
	SW_PM_MODE_NONE,
};

#define SW_PM_VALUE(mode, action) ((mode) << 16 | (action))

/*
 * Wrapper for ioctl arguments.
 * EVERY ioctl MUST use this struct!
 */
#pragma pack(push, 1)
struct sw_driver_ioctl_arg {
	pw_s32_t in_len;
	pw_s32_t out_len;
	/* pw_u64_t p_in_arg; Pointer to input arg */
	/* pw_u64_t p_out_arg; Pointer to output arg */
	char *in_arg;
	char *out_arg;
};
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct sw_driver_msg_interval {
	pw_u8_t plugin_id; /* Cannot have more than 256 plugins */
	pw_u8_t metric_id; /* Each plugin cannot handle more than 256 metrics */
	pw_u8_t msg_id; /* Each metric cannot have more than 256 components */
	pw_u16_t interval; /* collection interval */
} sw_driver_msg_interval_t;
#pragma pack(pop)

#endif /* __SW_STRUCTS_H__ */
