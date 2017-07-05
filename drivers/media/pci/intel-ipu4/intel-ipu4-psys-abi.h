/*
 * Copyright (c) 2016--2017 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "intel-ipu5-resources.h"
#include "intel-ipu4-resources.h"

#ifndef INTEL_IPU4_PSYS_ABI_H
#define INTEL_IPU4_PSYS_ABI_H

/* ia_css_psys_device.c */
#define IPU_FW_PSYS_CMD_QUEUE_SIZE 0x20
#define IPU_FW_PSYS_EVENT_QUEUE_SIZE 0x40

/* ia_css_psys_transport.h */
#define IPU_FW_PSYS_CMD_BITS 64
#define IPU_FW_PSYS_EVENT_BITS 128

#define IPU_FW_PSYS_N_DATA_MEM_TYPE_ID	(IPU_FW_PSYS_N_MEM_TYPE_ID - 1)

/* ia_css_psys_transport.h */
enum {
	IPU_FW_PSYS_EVENT_TYPE_SUCCESS = 0,
	IPU_FW_PSYS_EVENT_TYPE_UNKNOWN_ERROR = 1,
	IPU_FW_PSYS_EVENT_TYPE_RET_REM_OBJ_NOT_FOUND = 2,
	IPU_FW_PSYS_EVENT_TYPE_RET_REM_OBJ_TOO_BIG = 3,
	IPU_FW_PSYS_EVENT_TYPE_RET_REM_OBJ_DDR_TRANS_ERR = 4,
	IPU_FW_PSYS_EVENT_TYPE_RET_REM_OBJ_NULL_PKG_DIR_ADDR = 5,
	IPU_FW_PSYS_EVENT_TYPE_PROC_GRP_LOAD_FRAME_ERR = 6,
	IPU_FW_PSYS_EVENT_TYPE_PROC_GRP_LOAD_FRAGMENT_ERR = 7,
	IPU_FW_PSYS_EVENT_TYPE_PROC_GRP_PROCESS_COUNT_ZERO = 8,
	IPU_FW_PSYS_EVENT_TYPE_PROC_GRP_PROCESS_INIT_ERR = 9,
	IPU_FW_PSYS_EVENT_TYPE_PROC_GRP_ABORT = 10,
	IPU_FW_PSYS_EVENT_TYPE_PROC_GRP_NULL = 11,
	IPU_FW_PSYS_EVENT_TYPE_PROC_GRP_VALIDATION_ERR = 12
};

enum {
	IPU_FW_PSYS_CMD_QUEUE_COMMAND_ID,
	IPU_FW_PSYS_CMD_QUEUE_DEVICE_ID,
	IPU_FW_PSYS_CMD_QUEUE_PPG0_COMMAND_ID,
	IPU_FW_PSYS_CMD_QUEUE_PPG1_COMMAND_ID,
	IPU_FW_PSYS_N_PSYS_CMD_QUEUE_ID
};

enum {
	IPU_FW_PSYS_EVENT_QUEUE_MAIN_ID,
	IPU_FW_PSYS_N_PSYS_EVENT_QUEUE_ID
};

enum {
	IPU_FW_PSYS_PROCESS_GROUP_ERROR = 0,
	IPU_FW_PSYS_PROCESS_GROUP_CREATED,
	IPU_FW_PSYS_PROCESS_GROUP_READY,
	IPU_FW_PSYS_PROCESS_GROUP_BLOCKED,
	IPU_FW_PSYS_PROCESS_GROUP_STARTED,
	IPU_FW_PSYS_PROCESS_GROUP_RUNNING,
	IPU_FW_PSYS_PROCESS_GROUP_STALLED,
	IPU_FW_PSYS_PROCESS_GROUP_STOPPED,
	IPU_FW_PSYS_N_PROCESS_GROUP_STATES
};

enum {
	IPU_FW_PSYS_CONNECTION_MEMORY = 0,
	IPU_FW_PSYS_CONNECTION_MEMORY_STREAM,
	IPU_FW_PSYS_CONNECTION_STREAM,
	IPU_FW_PSYS_N_CONNECTION_TYPES
};

enum {
	IPU_FW_PSYS_BUFFER_NULL = 0,
	IPU_FW_PSYS_BUFFER_UNDEFINED,
	IPU_FW_PSYS_BUFFER_EMPTY,
	IPU_FW_PSYS_BUFFER_NONEMPTY,
	IPU_FW_PSYS_BUFFER_FULL,
	IPU_FW_PSYS_N_BUFFER_STATES
};

enum {
	IPU_FW_PSYS_TERMINAL_TYPE_DATA_IN = 0,
	IPU_FW_PSYS_TERMINAL_TYPE_DATA_OUT,
	IPU_FW_PSYS_TERMINAL_TYPE_PARAM_STREAM,
	IPU_FW_PSYS_TERMINAL_TYPE_PARAM_CACHED_IN,
	IPU_FW_PSYS_TERMINAL_TYPE_PARAM_CACHED_OUT,
	IPU_FW_PSYS_TERMINAL_TYPE_PARAM_SPATIAL_IN,
	IPU_FW_PSYS_TERMINAL_TYPE_PARAM_SPATIAL_OUT,
	IPU_FW_PSYS_TERMINAL_TYPE_PARAM_SLICED_IN,
	IPU_FW_PSYS_TERMINAL_TYPE_PARAM_SLICED_OUT,
	IPU_FW_PSYS_TERMINAL_TYPE_STATE_IN,
	IPU_FW_PSYS_TERMINAL_TYPE_STATE_OUT,
	IPU_FW_PSYS_TERMINAL_TYPE_PROGRAM,
	IPU_FW_PSYS_N_TERMINAL_TYPES
};

enum {
	IPU_FW_PSYS_COL_DIMENSION = 0,
	IPU_FW_PSYS_ROW_DIMENSION = 1,
	IPU_FW_PSYS_N_DATA_DIMENSION = 2
};

enum {
	IPU_FW_PSYS_PROCESS_GROUP_CMD_NOP = 0,
	IPU_FW_PSYS_PROCESS_GROUP_CMD_SUBMIT,
	IPU_FW_PSYS_PROCESS_GROUP_CMD_ATTACH,
	IPU_FW_PSYS_PROCESS_GROUP_CMD_DETACH,
	IPU_FW_PSYS_PROCESS_GROUP_CMD_START,
	IPU_FW_PSYS_PROCESS_GROUP_CMD_DISOWN,
	IPU_FW_PSYS_PROCESS_GROUP_CMD_RUN,
	IPU_FW_PSYS_PROCESS_GROUP_CMD_STOP,
	IPU_FW_PSYS_PROCESS_GROUP_CMD_SUSPEND,
	IPU_FW_PSYS_PROCESS_GROUP_CMD_RESUME,
	IPU_FW_PSYS_PROCESS_GROUP_CMD_ABORT,
	IPU_FW_PSYS_PROCESS_GROUP_CMD_RESET,
	IPU_FW_PSYS_N_PROCESS_GROUP_CMDS
};

enum {
	IPU_FW_PSYS_PROCESS_GROUP_PROTOCOL_LEGACY = 0,
	IPU_FW_PSYS_PROCESS_GROUP_PROTOCOL_PPG,
	IPU_FW_PSYS_PROCESS_GROUP_N_PROTOCOLS
};

/* ia_css_psys_process_group_cmd_impl.h */
struct __packed ipu_fw_psys_process_group {
	u64 token;
	u64 private_token;
	u32 size;
	u32 pg_load_start_ts;
	u32 pg_load_cycles;
	u32 pg_init_cycles;
	u32 pg_processing_cycles;
	u32 ID;
	u32 state;
	u32 ipu_virtual_address;
	u32 resource_bitmap;
	u16 fragment_count;
	u16 fragment_state;
	u16 fragment_limit;
	u16 processes_offset;
	u16 terminals_offset;
	u8 process_count;
	u8 terminal_count;
	u8 subgraph_count;
	u8 protocol_version;
	u8 base_queue_id;
	u8 num_queues;
	u8 padding[IPU_FW_PSYS_N_PADDING_UINT8_IN_PROCESS_GROUP_STRUCT];
};

struct ipu_fw_psys_process {
	u64 kernel_bitmap;
	u32 size;
	u32 ID;
	u32 state;
	s16 parent_offset;
	u16 cell_dependencies_offset;
	u16 terminal_dependencies_offset;
	u16 int_mem_offset[IPU_FW_PSYS_N_MEM_TYPE_ID];
	u16 ext_mem_offset[IPU_FW_PSYS_N_DATA_MEM_TYPE_ID];
	u16 dev_chn_offset[IPU_FW_PSYS_N_DEV_CHN_ID];
	u8 cell_id;
	u8 int_mem_id[IPU_FW_PSYS_N_MEM_TYPE_ID];
	u8 ext_mem_id[IPU_FW_PSYS_N_DATA_MEM_TYPE_ID];
	u8 cell_dependency_count;
	u8 terminal_dependency_count;
	u8 padding[IPU_FW_PSYS_N_PADDING_UINT8_IN_PROCESS_STRUCT];
};

/* ia_css_psys_init.h */
struct ipu_fw_psys_srv_init {
	void *host_ddr_pkg_dir;
	u32 ddr_pkg_dir_address;
	u32 pkg_dir_size;

	u32 icache_prefetch_sp;
	u32 icache_prefetch_isp;
};

/* ia_css_psys_transport.h */
struct __packed ipu_fw_psys_cmd {
	u16 command;
	u16 msg;
	u32 context_handle;
};


struct __packed ipu_fw_psys_event {
	u16 status;
	u16 command;
	u32 context_handle;
	u64 token;
};

/* ia_css_terminal_base_types.h */
struct ipu_fw_psys_terminal {
	u32 terminal_type;
	s16 parent_offset;
	u16 size;
	u16 tm_index;
	u8 ID;
	u8 padding[IPU_FW_PSYS_N_PADDING_UINT8_IN_TERMINAL_STRUCT];
};

/* ia_css_terminal_types.h */
struct ipu_fw_psys_param_payload {
	u64 host_buffer;
	u32 buffer;
	u32 terminal_index;
};

/* ia_css_program_group_param_types.h */
struct ipu_fw_psys_param_terminal {
	struct ipu_fw_psys_terminal base;
	struct ipu_fw_psys_param_payload param_payload;
	u16 param_section_desc_offset;
	u8 padding[IPU_FW_PSYS_N_PADDING_UINT8_IN_PARAM_TERMINAL_STRUCT];
};

struct ipu_fw_psys_frame {
	u32 buffer_state;
	u32 access_type;
	u32 pointer_state;
	u32 access_scope;
	u32 data;
	u32 data_index;
	u32 data_bytes;
	u8 padding[IPU_FW_PSYS_N_PADDING_UINT8_IN_FRAME_STRUCT];
};

/* ia_css_program_group_data.h */
struct ipu_fw_psys_frame_descriptor {
	u32 frame_format_type;
	u32 plane_count;
	u32 plane_offsets[IPU_FW_PSYS_N_FRAME_PLANES];
	u32 stride[1];
	u16 dimension[2];
	u16 size;
	u8 bpp;
	u8 bpe;
	u8 is_compressed;
	u8 padding[IPU_FW_PSYS_N_PADDING_UINT8_IN_FRAME_DESC_STRUCT];
};

struct ipu_fw_psys_stream {
	u64 dummy;
};

/* ia_css_psys_terminal_private_types.h */
struct ipu_fw_psys_data_terminal {
	struct ipu_fw_psys_terminal base;
	struct ipu_fw_psys_frame_descriptor frame_descriptor;
	struct ipu_fw_psys_frame frame;
	struct ipu_fw_psys_stream stream;
	u32 reserved;
	u32 connection_type;
	u16 fragment_descriptor_offset;
	u8 kernel_id;
	u8 subgraph_id;
	u8 link_id;
	u8 padding[IPU_FW_PSYS_N_PADDING_UINT8_IN_DATA_TERMINAL_STRUCT];
};

/* ia_css_psys_buffer_set.h */
struct ipu_fw_psys_buffer_set {
	u32 ipu_virtual_address;
	u32 process_group_handle;
	u16 terminal_count;
	u8 frame_counter;
	u8 padding[IPU_FW_PSYS_N_PADDING_UINT8_IN_BUFFER_SET_STRUCT];
};

struct intel_ipu4_psys_kcmd;
struct intel_ipu4_psys;

struct intel_ipu4_psys_abi {
	int (*pg_start)(struct intel_ipu4_psys_kcmd *kcmd);
	int (*pg_disown)(struct intel_ipu4_psys_kcmd *kcmd);
	int (*pg_submit)(struct intel_ipu4_psys_kcmd *kcmd);
	int (*pg_abort)(struct intel_ipu4_psys_kcmd *kcmd);
	int (*pg_rcv)(struct ipu_fw_psys_event *event);
	int (*terminal_set)(struct ipu_fw_psys_terminal *terminal,
			    int terminal_idx,
			    struct intel_ipu4_psys_kcmd *kcmd,
			    u32 buffer,
			    unsigned size);
	void (*pg_dump)(struct intel_ipu4_psys *psys,
			 struct intel_ipu4_psys_kcmd *kcmd,
			 const char *note);
	int (*pg_get_id)(struct intel_ipu4_psys_kcmd *kcmd);
	int (*pg_get_terminal_count)(struct intel_ipu4_psys_kcmd *kcmd);
	int (*pg_get_size)(struct intel_ipu4_psys_kcmd *kcmd);
	int (*pg_set_ipu_vaddress)(struct intel_ipu4_psys_kcmd *kcmd,
				   dma_addr_t vaddress);
	void (*pg_set_token)(struct intel_ipu4_psys_kcmd *kcmd,
					    u64 token);
	u64 (*pg_get_token)(struct intel_ipu4_psys_kcmd *kcmd);
	int (*ppg_set_buffer_set)(struct intel_ipu4_psys_kcmd *kcmd,
			    struct ipu_fw_psys_terminal *terminal,
			    int terminal_idx,
			    u32 buffer);
	size_t (*ppg_get_buffer_set_size)(struct intel_ipu4_psys_kcmd *kcmd);
	int (*ppg_set_buffer_vaddress)(struct ipu_fw_psys_buffer_set *buf_set,
			    u32 buffer);
	void *(*ppg_create_buffer_set)(struct intel_ipu4_psys_kcmd *kcmd,
			void *kaddr, u32 frame_counter);
	int (*ppg_enqueue_bufs)(struct intel_ipu4_psys_kcmd *kcmd,
			unsigned int queue_offset);
	int (*pg_get_protocol)(struct intel_ipu4_psys_kcmd *kcmd);
	int (*pg_get_load_cycles)(struct intel_ipu4_psys_kcmd *kcmd);
	int (*pg_get_init_cycles)(struct intel_ipu4_psys_kcmd *kcmd);
	int (*pg_get_processing_cycles)(struct intel_ipu4_psys_kcmd *kcmd);
	void *(*pg_get_terminal)(
		struct intel_ipu4_psys_kcmd *kcmd,
		int index);
	int (*open)(struct intel_ipu4_psys *psys);
	int (*close)(struct intel_ipu4_psys *psys);
};

void intel_ipu4_psys_abi_init_ext(struct intel_ipu4_psys_abi *abi);
void intel_ipu4_psys_abi_cleanup_ext(struct intel_ipu4_psys_abi *abi);
int intel_ipu4_psys_abi_pg_start(struct intel_ipu4_psys_kcmd *kcmd);
int intel_ipu4_psys_abi_pg_disown(struct intel_ipu4_psys_kcmd *kcmd);
int intel_ipu4_psys_abi_pg_abort(struct intel_ipu4_psys_kcmd *kcmd);
int intel_ipu4_psys_abi_pg_submit(struct intel_ipu4_psys_kcmd *kcmd);
int intel_ipu4_psys_abi_pg_load_cycles(struct intel_ipu4_psys_kcmd *kcmd);
int intel_ipu4_psys_abi_pg_init_cycles(struct intel_ipu4_psys_kcmd *kcmd);
int intel_ipu4_psys_abi_pg_processing_cycles(struct intel_ipu4_psys_kcmd *kcmd);
int intel_ipu4_psys_abi_rcv_event(
	struct intel_ipu4_psys *psys,
	struct ipu_fw_psys_event *event);
int intel_ipu4_psys_abi_terminal_set(struct ipu_fw_psys_terminal *terminal,
				     int terminal_idx,
				     struct intel_ipu4_psys_kcmd *kcmd,
				     u32 buffer,
				     unsigned size);
void intel_ipu4_psys_abi_pg_dump(struct intel_ipu4_psys *psys,
					struct intel_ipu4_psys_kcmd *kcmd,
					const char *note);
int intel_ipu4_psys_abi_pg_get_id(struct intel_ipu4_psys_kcmd *kcmd);
int intel_ipu4_psys_abi_pg_get_terminal_count(
	struct intel_ipu4_psys_kcmd *kcmd);
int intel_ipu4_psys_abi_pg_get_size(struct intel_ipu4_psys_kcmd *kcmd);
int intel_ipu4_psys_abi_pg_set_ipu_vaddress(struct intel_ipu4_psys_kcmd *kcmd,
					    dma_addr_t vaddress);
struct ipu_fw_psys_terminal *intel_ipu4_psys_abi_pg_get_terminal(
	struct intel_ipu4_psys_kcmd *kcmd,
	int index);
void intel_ipu4_psys_abi_pg_set_token(struct intel_ipu4_psys_kcmd *kcmd,
				      u64 token);
u64 intel_ipu4_psys_abi_pg_get_token(struct intel_ipu4_psys_kcmd *kcmd);
int intel_ipu_psys_abi_ppg_set_buffer_set(
	struct intel_ipu4_psys_kcmd *kcmd,
	struct ipu_fw_psys_terminal *terminal,
	int terminal_idx, u32 buffer);
size_t intel_ipu4_psys_abi_ppg_get_buffer_set_size
	(struct intel_ipu4_psys_kcmd *kcmd);
int intel_ipu_psys_abi_ppg_buffer_set_vaddress
			(struct ipu_fw_psys_buffer_set *buf_set, u32 vaddress);
struct ipu_fw_psys_buffer_set *
	intel_ipu_psys_abi_ppg_create_buffer_set
			(struct intel_ipu4_psys_kcmd *kcmd,
			 void *kaddr, u32 frame_counter);
int intel_ipu4_psys_abi_ppg_enqueue_bufs
	(struct intel_ipu4_psys_kcmd *kcmd, unsigned int queue_offset);
int intel_ipu4_psys_abi_pg_get_protocol(struct intel_ipu4_psys_kcmd *kcmd);
int intel_ipu4_psys_abi_open(struct intel_ipu4_psys *psys);
int intel_ipu4_psys_abi_close(struct intel_ipu4_psys *psys);

#endif /* INTEL_IPU4_PSYS_ABI_H */
