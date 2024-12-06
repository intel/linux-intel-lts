// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2016 - 2024 Intel Corporation

#include <linux/delay.h>

#include <uapi/linux/ipu-psys.h>

#include <linux/version.h>
#include "ipu6-fw-com.h"
#include "ipu-fw-psys.h"
#include "ipu-psys.h"

int ipu_fw_psys_pg_start(struct ipu_psys_kcmd *kcmd)
{
	kcmd->kpg->pg->state = IPU_FW_PSYS_PROCESS_GROUP_STARTED;
	return 0;
}

int ipu_fw_psys_pg_disown(struct ipu_psys_kcmd *kcmd)
{
	struct ipu_fw_psys_cmd *psys_cmd;
	struct device *dev = &kcmd->fh->psys->adev->auxdev.dev;
	int ret = 0;

	psys_cmd = ipu6_send_get_token(kcmd->fh->psys->fwcom, 0);
	if (!psys_cmd) {
		dev_err(dev, "%s failed to get token!\n", __func__);
		kcmd->pg_user = NULL;
		ret = -ENODATA;
		goto out;
	}
	psys_cmd->command = IPU_FW_PSYS_PROCESS_GROUP_CMD_START;
	psys_cmd->msg = 0;
	psys_cmd->context_handle = kcmd->kpg->pg->ipu_virtual_address;
	ipu6_send_put_token(kcmd->fh->psys->fwcom, 0);

out:
	return ret;
}

int ipu_fw_psys_ppg_suspend(struct ipu_psys_kcmd *kcmd)
{
	struct device *dev = &kcmd->fh->psys->adev->auxdev.dev;
	struct ipu_fw_psys_cmd *psys_cmd;
	int ret = 0;

	/* ppg suspend cmd uses QUEUE_DEVICE_ID instead of QUEUE_COMMAND_ID */
	psys_cmd = ipu6_send_get_token(kcmd->fh->psys->fwcom, 1);
	if (!psys_cmd) {
		dev_err(dev, "%s failed to get token!\n", __func__);
		kcmd->pg_user = NULL;
		ret = -ENODATA;
		goto out;
	}
	psys_cmd->command = IPU_FW_PSYS_PROCESS_GROUP_CMD_SUSPEND;
	psys_cmd->msg = 0;
	psys_cmd->context_handle = kcmd->kpg->pg->ipu_virtual_address;
	ipu6_send_put_token(kcmd->fh->psys->fwcom, 1);

out:
	return ret;
}

int ipu_fw_psys_ppg_resume(struct ipu_psys_kcmd *kcmd)
{
	struct device *dev = &kcmd->fh->psys->adev->auxdev.dev;
	struct ipu_fw_psys_cmd *psys_cmd;
	int ret = 0;

	psys_cmd = ipu6_send_get_token(kcmd->fh->psys->fwcom, 0);
	if (!psys_cmd) {
		dev_err(dev, "%s failed to get token!\n", __func__);
		kcmd->pg_user = NULL;
		ret = -ENODATA;
		goto out;
	}
	psys_cmd->command = IPU_FW_PSYS_PROCESS_GROUP_CMD_RESUME;
	psys_cmd->msg = 0;
	psys_cmd->context_handle = kcmd->kpg->pg->ipu_virtual_address;
	ipu6_send_put_token(kcmd->fh->psys->fwcom, 0);

out:
	return ret;
}

int ipu_fw_psys_pg_abort(struct ipu_psys_kcmd *kcmd)
{
	struct device *dev = &kcmd->fh->psys->adev->auxdev.dev;
	struct ipu_fw_psys_cmd *psys_cmd;
	int ret = 0;

	psys_cmd = ipu6_send_get_token(kcmd->fh->psys->fwcom, 0);
	if (!psys_cmd) {
		dev_err(dev, "%s failed to get token!\n", __func__);
		kcmd->pg_user = NULL;
		ret = -ENODATA;
		goto out;
	}
	psys_cmd->command = IPU_FW_PSYS_PROCESS_GROUP_CMD_STOP;
	psys_cmd->msg = 0;
	psys_cmd->context_handle = kcmd->kpg->pg->ipu_virtual_address;
	ipu6_send_put_token(kcmd->fh->psys->fwcom, 0);

out:
	return ret;
}

int ipu_fw_psys_pg_submit(struct ipu_psys_kcmd *kcmd)
{
	kcmd->kpg->pg->state = IPU_FW_PSYS_PROCESS_GROUP_BLOCKED;
	return 0;
}

int ipu_fw_psys_rcv_event(struct ipu_psys *psys,
			  struct ipu_fw_psys_event *event)
{
	void *rcv;

	rcv = ipu6_recv_get_token(psys->fwcom, 0);
	if (!rcv)
		return 0;

	memcpy(event, rcv, sizeof(*event));
	ipu6_recv_put_token(psys->fwcom, 0);
	return 1;
}

int ipu_fw_psys_terminal_set(struct ipu_fw_psys_terminal *terminal,
			     int terminal_idx,
			     struct ipu_psys_kcmd *kcmd,
			     u32 buffer, unsigned int size)
{
	struct device *dev = &kcmd->fh->psys->adev->auxdev.dev;
	u32 type;
	u32 buffer_state;

	type = terminal->terminal_type;

	switch (type) {
	case IPU_FW_PSYS_TERMINAL_TYPE_PARAM_CACHED_IN:
	case IPU_FW_PSYS_TERMINAL_TYPE_PARAM_CACHED_OUT:
	case IPU_FW_PSYS_TERMINAL_TYPE_PARAM_SPATIAL_IN:
	case IPU_FW_PSYS_TERMINAL_TYPE_PARAM_SPATIAL_OUT:
	case IPU_FW_PSYS_TERMINAL_TYPE_PARAM_SLICED_IN:
	case IPU_FW_PSYS_TERMINAL_TYPE_PARAM_SLICED_OUT:
	case IPU_FW_PSYS_TERMINAL_TYPE_PROGRAM:
	case IPU_FW_PSYS_TERMINAL_TYPE_PROGRAM_CONTROL_INIT:
		buffer_state = IPU_FW_PSYS_BUFFER_UNDEFINED;
		break;
	case IPU_FW_PSYS_TERMINAL_TYPE_PARAM_STREAM:
	case IPU_FW_PSYS_TERMINAL_TYPE_DATA_IN:
	case IPU_FW_PSYS_TERMINAL_TYPE_STATE_IN:
		buffer_state = IPU_FW_PSYS_BUFFER_FULL;
		break;
	case IPU_FW_PSYS_TERMINAL_TYPE_DATA_OUT:
	case IPU_FW_PSYS_TERMINAL_TYPE_STATE_OUT:
		buffer_state = IPU_FW_PSYS_BUFFER_EMPTY;
		break;
	default:
		dev_err(dev, "unknown terminal type: 0x%x\n", type);
		return -EAGAIN;
	}

	if (type == IPU_FW_PSYS_TERMINAL_TYPE_DATA_IN ||
	    type == IPU_FW_PSYS_TERMINAL_TYPE_DATA_OUT) {
		struct ipu_fw_psys_data_terminal *dterminal =
		    (struct ipu_fw_psys_data_terminal *)terminal;
		dterminal->connection_type = IPU_FW_PSYS_CONNECTION_MEMORY;
		dterminal->frame.data_bytes = size;
		if (!ipu_fw_psys_pg_get_protocol(kcmd))
			dterminal->frame.data = buffer;
		else
			dterminal->frame.data_index = terminal_idx;
		dterminal->frame.buffer_state = buffer_state;
	} else {
		struct ipu_fw_psys_param_terminal *pterminal =
		    (struct ipu_fw_psys_param_terminal *)terminal;
		if (!ipu_fw_psys_pg_get_protocol(kcmd))
			pterminal->param_payload.buffer = buffer;
		else
			pterminal->param_payload.terminal_index = terminal_idx;
	}
	return 0;
}

void ipu_fw_psys_pg_dump(struct ipu_psys *psys,
			 struct ipu_psys_kcmd *kcmd, const char *note)
{
	ipu6_fw_psys_pg_dump(psys, kcmd, note);
}

int ipu_fw_psys_pg_get_id(struct ipu_psys_kcmd *kcmd)
{
	return kcmd->kpg->pg->ID;
}

int ipu_fw_psys_pg_get_terminal_count(struct ipu_psys_kcmd *kcmd)
{
	return kcmd->kpg->pg->terminal_count;
}

int ipu_fw_psys_pg_get_size(struct ipu_psys_kcmd *kcmd)
{
	return kcmd->kpg->pg->size;
}

int ipu_fw_psys_pg_set_ipu_vaddress(struct ipu_psys_kcmd *kcmd,
				    dma_addr_t vaddress)
{
	kcmd->kpg->pg->ipu_virtual_address = vaddress;
	return 0;
}

struct ipu_fw_psys_terminal *ipu_fw_psys_pg_get_terminal(struct ipu_psys_kcmd
							 *kcmd, int index)
{
	struct ipu_fw_psys_terminal *terminal;
	u16 *terminal_offset_table;

	terminal_offset_table =
	    (uint16_t *)((char *)kcmd->kpg->pg +
			  kcmd->kpg->pg->terminals_offset);
	terminal = (struct ipu_fw_psys_terminal *)
	    ((char *)kcmd->kpg->pg + terminal_offset_table[index]);
	return terminal;
}

void ipu_fw_psys_pg_set_token(struct ipu_psys_kcmd *kcmd, u64 token)
{
	kcmd->kpg->pg->token = (u64)token;
}

u64 ipu_fw_psys_pg_get_token(struct ipu_psys_kcmd *kcmd)
{
	return kcmd->kpg->pg->token;
}

int ipu_fw_psys_pg_get_protocol(struct ipu_psys_kcmd *kcmd)
{
	return kcmd->kpg->pg->protocol_version;
}

int ipu_fw_psys_ppg_set_buffer_set(struct ipu_psys_kcmd *kcmd,
				   struct ipu_fw_psys_terminal *terminal,
				   int terminal_idx, u32 buffer)
{
	struct device *dev = &kcmd->fh->psys->adev->auxdev.dev;
	struct ipu_fw_psys_buffer_set *buf_set = kcmd->kbuf_set->buf_set;
	u32 buffer_state;
	u32 *buffer_ptr;
	u32 type;

	type = terminal->terminal_type;

	switch (type) {
	case IPU_FW_PSYS_TERMINAL_TYPE_PARAM_CACHED_IN:
	case IPU_FW_PSYS_TERMINAL_TYPE_PARAM_CACHED_OUT:
	case IPU_FW_PSYS_TERMINAL_TYPE_PARAM_SPATIAL_IN:
	case IPU_FW_PSYS_TERMINAL_TYPE_PARAM_SPATIAL_OUT:
	case IPU_FW_PSYS_TERMINAL_TYPE_PARAM_SLICED_IN:
	case IPU_FW_PSYS_TERMINAL_TYPE_PARAM_SLICED_OUT:
	case IPU_FW_PSYS_TERMINAL_TYPE_PROGRAM:
	case IPU_FW_PSYS_TERMINAL_TYPE_PROGRAM_CONTROL_INIT:
		buffer_state = IPU_FW_PSYS_BUFFER_UNDEFINED;
		break;
	case IPU_FW_PSYS_TERMINAL_TYPE_PARAM_STREAM:
	case IPU_FW_PSYS_TERMINAL_TYPE_DATA_IN:
	case IPU_FW_PSYS_TERMINAL_TYPE_STATE_IN:
		buffer_state = IPU_FW_PSYS_BUFFER_FULL;
		break;
	case IPU_FW_PSYS_TERMINAL_TYPE_DATA_OUT:
	case IPU_FW_PSYS_TERMINAL_TYPE_STATE_OUT:
		buffer_state = IPU_FW_PSYS_BUFFER_EMPTY;
		break;
	default:
		dev_err(dev, "unknown terminal type: 0x%x\n", type);
		return -EAGAIN;
	}

	buffer_ptr = (u32 *)((char *)buf_set + sizeof(*buf_set) +
			      terminal_idx * sizeof(*buffer_ptr));

	*buffer_ptr = buffer;

	if (type == IPU_FW_PSYS_TERMINAL_TYPE_DATA_IN ||
	    type == IPU_FW_PSYS_TERMINAL_TYPE_DATA_OUT) {
		struct ipu_fw_psys_data_terminal *dterminal =
		    (struct ipu_fw_psys_data_terminal *)terminal;
		dterminal->frame.buffer_state = buffer_state;
	}

	return 0;
}

size_t ipu_fw_psys_ppg_get_buffer_set_size(struct ipu_psys_kcmd *kcmd)
{
	return (sizeof(struct ipu_fw_psys_buffer_set) +
		kcmd->kpg->pg->terminal_count * sizeof(u32));
}

int
ipu_fw_psys_ppg_buffer_set_vaddress(struct ipu_fw_psys_buffer_set *buf_set,
				    u32 vaddress)
{
	buf_set->ipu_virtual_address = vaddress;
	return 0;
}

int ipu_fw_psys_ppg_buffer_set_set_kernel_enable_bitmap(
		struct ipu_fw_psys_buffer_set *buf_set,
		u32 *kernel_enable_bitmap)
{
	memcpy(buf_set->kernel_enable_bitmap, (u8 *)kernel_enable_bitmap,
	       sizeof(buf_set->kernel_enable_bitmap));
	return 0;
}

struct ipu_fw_psys_buffer_set *
ipu_fw_psys_ppg_create_buffer_set(struct ipu_psys_kcmd *kcmd,
				  void *kaddr, u32 frame_counter)
{
	struct ipu_fw_psys_buffer_set *buffer_set = NULL;
	unsigned int i;

	buffer_set = (struct ipu_fw_psys_buffer_set *)kaddr;

	/*
	 * Set base struct members
	 */
	buffer_set->ipu_virtual_address = 0;
	buffer_set->process_group_handle = kcmd->kpg->pg->ipu_virtual_address;
	buffer_set->frame_counter = frame_counter;
	buffer_set->terminal_count = kcmd->kpg->pg->terminal_count;

	/*
	 * Initialize adjacent buffer addresses
	 */
	for (i = 0; i < buffer_set->terminal_count; i++) {
		u32 *buffer =
		    (u32 *)((char *)buffer_set +
			     sizeof(*buffer_set) + sizeof(u32) * i);

		*buffer = 0;
	}

	return buffer_set;
}

int ipu_fw_psys_ppg_enqueue_bufs(struct ipu_psys_kcmd *kcmd)
{
	struct device *dev = &kcmd->fh->psys->adev->auxdev.dev;
	struct ipu_fw_psys_cmd *psys_cmd;
	unsigned int queue_id;
	int ret = 0;
	unsigned int size;

	if (ipu_ver == IPU6_VER_6SE)
		size = IPU6SE_FW_PSYS_N_PSYS_CMD_QUEUE_ID;
	else
		size = IPU6_FW_PSYS_N_PSYS_CMD_QUEUE_ID;
	queue_id = kcmd->kpg->pg->base_queue_id;

	if (queue_id >= size)
		return -EINVAL;

	psys_cmd = ipu6_send_get_token(kcmd->fh->psys->fwcom, queue_id);
	if (!psys_cmd) {
		dev_err(dev, "%s failed to get token!\n", __func__);
		kcmd->pg_user = NULL;
		return -ENODATA;
	}

	psys_cmd->command = IPU_FW_PSYS_PROCESS_GROUP_CMD_RUN;
	psys_cmd->msg = 0;
	psys_cmd->context_handle = kcmd->kbuf_set->buf_set->ipu_virtual_address;

	ipu6_send_put_token(kcmd->fh->psys->fwcom, queue_id);

	return ret;
}

u8 ipu_fw_psys_ppg_get_base_queue_id(struct ipu_psys_kcmd *kcmd)
{
	return kcmd->kpg->pg->base_queue_id;
}

void ipu_fw_psys_ppg_set_base_queue_id(struct ipu_psys_kcmd *kcmd, u8 queue_id)
{
	kcmd->kpg->pg->base_queue_id = queue_id;
}

int ipu_fw_psys_open(struct ipu_psys *psys)
{
	struct device *dev = &psys->adev->auxdev.dev;
	int retry = IPU_PSYS_OPEN_RETRY, retval;

	retval = ipu6_fw_com_open(psys->fwcom);
	if (retval) {
		dev_err(dev, "fw com open failed.\n");
		return retval;
	}

	do {
		usleep_range(IPU_PSYS_OPEN_TIMEOUT_US,
			     IPU_PSYS_OPEN_TIMEOUT_US + 10);
		retval = ipu6_fw_com_ready(psys->fwcom);
		if (retval) {
			dev_dbg(dev, "psys port open ready!\n");
			break;
		}
		retry--;
	} while (retry > 0);

	if (!retry) {
		dev_err(dev, "psys port open ready failed\n");
		ipu6_fw_com_close(psys->fwcom);
		return -EIO;
	}

	return 0;
}

int ipu_fw_psys_close(struct ipu_psys *psys)
{
	struct device *dev = &psys->adev->auxdev.dev;
	int retval;

	retval = ipu6_fw_com_close(psys->fwcom);
	if (retval) {
		dev_err(dev, "fw com close failed.\n");
		return retval;
	}
	return retval;
}
