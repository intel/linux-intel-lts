// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2013 - 2025 Intel Corporation
 */

#include <linux/cacheflush.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/types.h>

#include "abi/ipu7_fw_insys_config_abi.h"
#include "abi/ipu7_fw_isys_abi.h"

#include "ipu7.h"
#include "ipu7-boot.h"
#include "ipu7-bus.h"
#include "ipu7-dma.h"
#include "ipu7-fw-isys.h"
#include "ipu7-isys.h"
#include "ipu7-platform-regs.h"
#include "ipu7-syscom.h"

static const char * const send_msg_types[N_IPU_INSYS_SEND_TYPE] = {
	"STREAM_OPEN",
	"STREAM_START_AND_CAPTURE",
	"STREAM_CAPTURE",
	"STREAM_ABORT",
	"STREAM_FLUSH",
	"STREAM_CLOSE"
};

static void isys_stream_cfg_to_v1(struct ipu7_insys_stream_cfg_v1 *dst,
				  const struct ipu7_insys_stream_cfg *src)
{
	unsigned int i;

	memset(dst, 0, sizeof(*dst));
	memcpy(dst->input_pins, src->input_pins, sizeof(dst->input_pins));
	dst->stream_msg_map = src->stream_msg_map;
	dst->port_id = src->port_id;
	dst->vc = src->vc;
	dst->nof_input_pins = src->nof_input_pins;
	dst->nof_output_pins = src->nof_output_pins;

	for (i = 0; i < ARRAY_SIZE(dst->output_pins); i++) {
		dst->output_pins[i].link = src->output_pins[i].link;
		dst->output_pins[i].crop.line_top =
			src->output_pins[i].crop.line_top;
		dst->output_pins[i].crop.line_bottom =
			src->output_pins[i].crop.line_bottom;
		dst->output_pins[i].dpcm = src->output_pins[i].dpcm;
		dst->output_pins[i].stride = src->output_pins[i].stride;
		dst->output_pins[i].ft = src->output_pins[i].ft;
		dst->output_pins[i].send_irq = src->output_pins[i].send_irq;
		dst->output_pins[i].input_pin_id =
			src->output_pins[i].input_pin_id;
		dst->output_pins[i].early_ack_en =
			src->output_pins[i].early_ack_en;
	}
}

static void isys_buffset_to_v1(struct ipu7_insys_buffset_v1 *dst,
			       const struct ipu7_insys_buffset *src)
{
	unsigned int i;

	memset(dst, 0, sizeof(*dst));
	for (i = 0; i < ARRAY_SIZE(dst->output_pins); i++)
		dst->output_pins[i] = src->output_pins[i].pin_payload;

	dst->capture_msg_map = src->capture_msg_map;
	dst->frame_id = src->frame_id;
	dst->skip_frame = src->skip_frame;
}

static size_t isys_prepare_fw_payload_v1(void *cpu_mapped_buf,
					 u16 send_type, size_t size)
{
	if (!cpu_mapped_buf)
		return 0;

	switch (send_type) {
	case IPU_INSYS_SEND_TYPE_STREAM_OPEN: {
		struct ipu7_insys_stream_cfg cfg;

		memcpy(&cfg, cpu_mapped_buf, sizeof(cfg));
		isys_stream_cfg_to_v1(cpu_mapped_buf, &cfg);
		return sizeof(struct ipu7_insys_stream_cfg_v1);
	}
	case IPU_INSYS_SEND_TYPE_STREAM_START_AND_CAPTURE:
	case IPU_INSYS_SEND_TYPE_STREAM_CAPTURE: {
		struct ipu7_insys_buffset set;

		memcpy(&set, cpu_mapped_buf, sizeof(set));
		isys_buffset_to_v1(cpu_mapped_buf, &set);
		return sizeof(struct ipu7_insys_buffset_v1);
	}
	default:
		return size;
	}
}

static size_t isys_prepare_fw_payload(void *cpu_mapped_buf,
				      u16 send_type, size_t size)
{
	if (!cpu_mapped_buf)
		return 0;

	switch (send_type) {
	case IPU_INSYS_SEND_TYPE_STREAM_OPEN:
		return sizeof(struct ipu7_insys_stream_cfg);
	case IPU_INSYS_SEND_TYPE_STREAM_START_AND_CAPTURE:
	case IPU_INSYS_SEND_TYPE_STREAM_CAPTURE:
		return sizeof(struct ipu7_insys_buffset);
	default:
		return size;
	}
}

static __maybe_unused void isys_decode_resp_v1(struct ipu7_insys_resp *dst, const void *token)
{
	const struct ipu7_insys_resp_v1 *src = token;

	memset(dst, 0, sizeof(*dst));
	dst->buf_id = src->buf_id;
	dst->pin = src->pin;
	dst->error_info = src->error_info;
	dst->timestamp[0] = src->timestamp[0];
	dst->timestamp[1] = src->timestamp[1];
	dst->type = src->type;
	dst->msg_link_streaming_mode = src->msg_link_streaming_mode;
	dst->stream_id = src->stream_id;
	dst->pin_id = src->pin_id;
	dst->frame_id = src->frame_id;
	dst->skip_frame = src->skip_frame;
	dst->mipi_fn = src->mipi_fn;
}

static void isys_decode_resp(struct ipu7_insys_resp *dst, const void *token)
{
	memcpy(dst, token, sizeof(*dst));
}

int ipu7_fw_isys_complex_cmd(struct ipu7_isys *isys,
			     const unsigned int stream_handle,
			     void *cpu_mapped_buf,
			     dma_addr_t dma_mapped_buf,
			     size_t size, u16 send_type)
{
	struct ipu7_syscom_context *ctx = isys->adev->syscom;
	struct device *dev = &isys->adev->auxdev.dev;
	struct ipu7_insys_send_queue_token *token;

	if (send_type >= N_IPU_INSYS_SEND_TYPE)
		return -EINVAL;

	dev_dbg(dev, "send_token: %s\n", send_msg_types[send_type]);

	/*
	 * Time to flush cache in case we have some payload. Not all messages
	 * have that
	 */
	if (cpu_mapped_buf) {
		size = isys->abi_ops.prepare_payload(cpu_mapped_buf,
						     send_type, size);
		clflush_cache_range(cpu_mapped_buf, size);
	}

	token = ipu7_syscom_get_token(ctx, stream_handle +
				      IPU_INSYS_INPUT_MSG_QUEUE);
	if (!token)
		return -EBUSY;

	token->addr = dma_mapped_buf;
	token->buf_handle = (unsigned long)cpu_mapped_buf;
	token->send_type = send_type;
	token->stream_id = stream_handle;
	token->flag = IPU_INSYS_SEND_QUEUE_TOKEN_FLAG_NONE;

	ipu7_syscom_put_token(ctx, stream_handle + IPU_INSYS_INPUT_MSG_QUEUE);
	/* now wakeup FW */
	ipu_buttress_wakeup_is_uc(isys->adev->isp);

	return 0;
}

int ipu7_fw_isys_simple_cmd(struct ipu7_isys *isys,
			    const unsigned int stream_handle, u16 send_type)
{
	return ipu7_fw_isys_complex_cmd(isys, stream_handle, NULL, 0, 0,
					send_type);
}

int ipu7_fw_isys_init(struct ipu7_isys *isys)
{
	struct syscom_queue_config *queue_configs;
	struct ipu7_bus_device *adev = isys->adev;
	struct device *dev = &adev->auxdev.dev;
	struct ipu7_insys_config *isys_config;
	struct ipu7_syscom_context *syscom;
	dma_addr_t isys_config_dma_addr;
	unsigned int i, num_queues;
	u32 freq;
	u8 major;
	int ret;

	/* Allocate and init syscom context. */
	syscom = devm_kzalloc(dev, sizeof(struct ipu7_syscom_context),
			      GFP_KERNEL);
	if (!syscom)
		return -ENOMEM;

	if (is_ipu8(adev->isp->hw_ver))
		isys->abi_ops.prepare_payload = isys_prepare_fw_payload;
	else
		isys->abi_ops.prepare_payload = isys_prepare_fw_payload_v1;

	isys->abi_ops.decode_resp = isys_decode_resp;
	isys->abi_ops.resp_queue_token_size =
		sizeof(struct ipu7_insys_resp);

	adev->syscom = syscom;
	syscom->num_input_queues = IPU_INSYS_MAX_INPUT_QUEUES;
	syscom->num_output_queues = IPU_INSYS_MAX_OUTPUT_QUEUES;
	num_queues = syscom->num_input_queues + syscom->num_output_queues;
	queue_configs = devm_kzalloc(dev, FW_QUEUE_CONFIG_SIZE(num_queues),
				     GFP_KERNEL);
	if (!queue_configs) {
		ipu7_fw_isys_release(isys);
		return -ENOMEM;
	}
	syscom->queue_configs = queue_configs;
	queue_configs[IPU_INSYS_OUTPUT_MSG_QUEUE].max_capacity =
		IPU_ISYS_SIZE_RECV_QUEUE;
	queue_configs[IPU_INSYS_OUTPUT_MSG_QUEUE].token_size_in_bytes =
		isys->abi_ops.resp_queue_token_size;
	queue_configs[IPU_INSYS_OUTPUT_LOG_QUEUE].max_capacity =
		IPU_ISYS_SIZE_LOG_QUEUE;
	queue_configs[IPU_INSYS_OUTPUT_LOG_QUEUE].token_size_in_bytes =
		isys->abi_ops.resp_queue_token_size;
	queue_configs[IPU_INSYS_OUTPUT_RESERVED_QUEUE].max_capacity = 0;
	queue_configs[IPU_INSYS_OUTPUT_RESERVED_QUEUE].token_size_in_bytes = 0;

	queue_configs[IPU_INSYS_INPUT_DEV_QUEUE].max_capacity =
		IPU_ISYS_MAX_STREAMS;
	queue_configs[IPU_INSYS_INPUT_DEV_QUEUE].token_size_in_bytes =
		sizeof(struct ipu7_insys_send_queue_token);

	for (i = IPU_INSYS_INPUT_MSG_QUEUE; i < num_queues; i++) {
		queue_configs[i].max_capacity = IPU_ISYS_SIZE_SEND_QUEUE;
		queue_configs[i].token_size_in_bytes =
			sizeof(struct ipu7_insys_send_queue_token);
	}

	/* Allocate ISYS subsys config. */
	isys_config = ipu7_dma_alloc(adev, sizeof(struct ipu7_insys_config),
				     &isys_config_dma_addr, GFP_KERNEL, 0);
	if (!isys_config) {
		dev_err(dev, "Failed to allocate isys subsys config.\n");
		ipu7_fw_isys_release(isys);
		return -ENOMEM;
	}
	isys->subsys_config = isys_config;
	isys->subsys_config_dma_addr = isys_config_dma_addr;
	memset(isys_config, 0, sizeof(struct ipu7_insys_config));
	isys_config->logger_config.use_source_severity = 0;
	isys_config->logger_config.use_channels_enable_bitmask = 1;
	isys_config->logger_config.channels_enable_bitmask =
		LOGGER_CONFIG_CHANNEL_ENABLE_SYSCOM_BITMASK;
	isys_config->logger_config.hw_printf_buffer_base_addr = 0U;
	isys_config->logger_config.hw_printf_buffer_size_bytes = 0U;
	isys_config->wdt_config.wdt_timer1_us = 0;
	isys_config->wdt_config.wdt_timer2_us = 0;
	ret = ipu_buttress_get_isys_freq(adev->isp, &freq);
	if (ret) {
		dev_err(dev, "Failed to get ISYS frequency.\n");
		ipu7_fw_isys_release(isys);
		return ret;
	}

	ipu7_dma_sync_single(adev, isys_config_dma_addr,
			     sizeof(struct ipu7_insys_config));

	major = is_ipu8(adev->isp->hw_ver) ? 2U : 1U;
	ret = ipu7_boot_init_boot_config(adev, queue_configs, num_queues,
					 freq, isys_config_dma_addr, major);
	if (ret)
		ipu7_fw_isys_release(isys);

	return ret;
}

void ipu7_fw_isys_release(struct ipu7_isys *isys)
{
	struct ipu7_bus_device *adev = isys->adev;

	ipu7_boot_release_boot_config(adev);
	if (isys->subsys_config) {
		ipu7_dma_free(adev,
			      sizeof(struct ipu7_insys_config),
			      isys->subsys_config,
			      isys->subsys_config_dma_addr, 0);
		isys->subsys_config = NULL;
		isys->subsys_config_dma_addr = 0;
	}
}

int ipu7_fw_isys_open(struct ipu7_isys *isys)
{
	return ipu7_boot_start_fw(isys->adev);
}

int ipu7_fw_isys_close(struct ipu7_isys *isys)
{
	return ipu7_boot_stop_fw(isys->adev);
}

struct ipu7_insys_resp *ipu7_fw_isys_get_resp(struct ipu7_isys *isys)
{
	void *token = ipu7_syscom_get_token(isys->adev->syscom,
				     IPU_INSYS_OUTPUT_MSG_QUEUE);

	if (!token)
		return NULL;

	isys->abi_ops.decode_resp(&isys->resp, token);

	return &isys->resp;
}

void ipu7_fw_isys_put_resp(struct ipu7_isys *isys)
{
	ipu7_syscom_put_token(isys->adev->syscom, IPU_INSYS_OUTPUT_MSG_QUEUE);
}

#ifdef ENABLE_FW_OFFLINE_LOGGER
int ipu7_fw_isys_get_log(struct ipu7_isys *isys)
{
	u32 log_size = sizeof(struct ia_gofo_msg_log_info_ts);
	struct device *dev = &isys->adev->auxdev.dev;
	struct isys_fw_log *fw_log = isys->fw_log;
	struct ia_gofo_msg_log *log_msg;
	u8 msg_type, msg_len;
	u32 count, fmt_id;
	void *token;

	token = ipu7_syscom_get_token(isys->adev->syscom,
				      IPU_INSYS_OUTPUT_LOG_QUEUE);
	if (!token)
		return -ENODATA;

	while (token) {
		log_msg = (struct ia_gofo_msg_log *)token;

		msg_type = log_msg->header.tlv_header.tlv_type;
		msg_len = log_msg->header.tlv_header.tlv_len32;
		if (msg_type != IPU_MSG_TYPE_DEV_LOG || !msg_len)
			dev_warn(dev, "Invalid msg data from Log queue!\n");

		count = log_msg->log_info_ts.log_info.log_counter;
		fmt_id = log_msg->log_info_ts.log_info.fmt_id;
		if (count > fw_log->count + 1)
			dev_warn(dev, "log msg lost, count %u+1 != %u!\n",
				 count, fw_log->count);

		if (fmt_id == IA_GOFO_MSG_LOG_FMT_ID_INVALID) {
			dev_err(dev, "invalid log msg fmt_id 0x%x!\n", fmt_id);
			ipu7_syscom_put_token(isys->adev->syscom,
					      IPU_INSYS_OUTPUT_LOG_QUEUE);
			return -EIO;
		}

		if (log_size + fw_log->head - fw_log->addr >
		    FW_LOG_BUF_SIZE)
			fw_log->head = fw_log->addr;

		memcpy(fw_log->head, (void *)&log_msg->log_info_ts,
		       sizeof(struct ia_gofo_msg_log_info_ts));

		fw_log->count = count;
		fw_log->head += log_size;
		fw_log->size += log_size;

		ipu7_syscom_put_token(isys->adev->syscom,
				      IPU_INSYS_OUTPUT_LOG_QUEUE);

		token = ipu7_syscom_get_token(isys->adev->syscom,
					      IPU_INSYS_OUTPUT_LOG_QUEUE);
	};

	return 0;
}

#endif
void ipu7_fw_isys_dump_stream_cfg(struct device *dev,
				  struct ipu7_insys_stream_cfg *cfg)
{
	unsigned int i;

	dev_dbg(dev, "---------------------------\n");
	dev_dbg(dev, "IPU_FW_ISYS_STREAM_CFG_DATA\n");

	dev_dbg(dev, ".port id %d\n", cfg->port_id);
	dev_dbg(dev, ".vc %d\n", cfg->vc);
	dev_dbg(dev, ".nof_input_pins = %d\n", cfg->nof_input_pins);
	dev_dbg(dev, ".nof_output_pins = %d\n", cfg->nof_output_pins);
	dev_dbg(dev, ".stream_msg_map = 0x%x\n", cfg->stream_msg_map);

	for (i = 0; i < cfg->nof_input_pins; i++) {
		dev_dbg(dev, ".input_pin[%d]:\n", i);
		dev_dbg(dev, "\t.dt = 0x%0x\n",
			cfg->input_pins[i].dt);
		dev_dbg(dev, "\t.disable_mipi_unpacking = %d\n",
			cfg->input_pins[i].disable_mipi_unpacking);
		dev_dbg(dev, "\t.dt_rename_mode = %d\n",
			cfg->input_pins[i].dt_rename_mode);
		dev_dbg(dev, "\t.mapped_dt = 0x%0x\n",
			cfg->input_pins[i].mapped_dt);
		dev_dbg(dev, "\t.input_res = %d x %d\n",
			cfg->input_pins[i].input_res.width,
			cfg->input_pins[i].input_res.height);
		dev_dbg(dev, "\t.sync_msg_map = 0x%x\n",
			cfg->input_pins[i].sync_msg_map);
	}

	for (i = 0; i < cfg->nof_output_pins; i++) {
		dev_dbg(dev, ".output_pin[%d]:\n", i);
		dev_dbg(dev, "\t.input_pin_id = %d\n",
			cfg->output_pins[i].input_pin_id);
		dev_dbg(dev, "\t.stride = %d\n", cfg->output_pins[i].stride);
		dev_dbg(dev, "\t.send_irq = %d\n",
			cfg->output_pins[i].send_irq);
		dev_dbg(dev, "\t.ft = %d\n", cfg->output_pins[i].ft);

		dev_dbg(dev, "\t.link.buffer_lines = %d\n",
			cfg->output_pins[i].link.buffer_lines);
		dev_dbg(dev, "\t.link.foreign_key = %d\n",
			cfg->output_pins[i].link.foreign_key);
		dev_dbg(dev, "\t.link.granularity_pointer_update = %d\n",
			cfg->output_pins[i].link.granularity_pointer_update);
		dev_dbg(dev, "\t.link.msg_link_streaming_mode = %d\n",
			cfg->output_pins[i].link.msg_link_streaming_mode);
		dev_dbg(dev, "\t.link.pbk_id = %d\n",
			cfg->output_pins[i].link.pbk_id);
		dev_dbg(dev, "\t.link.pbk_slot_id = %d\n",
			cfg->output_pins[i].link.pbk_slot_id);
		dev_dbg(dev, "\t.link.dest = %d\n",
			cfg->output_pins[i].link.dest);
		dev_dbg(dev, "\t.link.use_sw_managed = %d\n",
			cfg->output_pins[i].link.use_sw_managed);
		dev_dbg(dev, "\t.link.is_snoop = %d\n",
			cfg->output_pins[i].link.is_snoop);

		dev_dbg(dev, "\t.crop.line_top = %d\n",
			cfg->output_pins[i].crop.line_top);
		dev_dbg(dev, "\t.crop.line_bottom = %d\n",
			cfg->output_pins[i].crop.line_bottom);
		dev_dbg(dev, "\t.crop.column_left = %d\n",
			cfg->output_pins[i].crop.column_left);
		dev_dbg(dev, "\t.crop.column_right = %d\n",
			cfg->output_pins[i].crop.column_right);

		dev_dbg(dev, "\t.dpcm_enable = %d\n",
			cfg->output_pins[i].dpcm.enable);
		dev_dbg(dev, "\t.dpcm.type = %d\n",
			cfg->output_pins[i].dpcm.type);
		dev_dbg(dev, "\t.dpcm.predictor = %d\n",
			cfg->output_pins[i].dpcm.predictor);
		dev_dbg(dev, "\t.upipe_enable = %d\n",
			cfg->output_pins[i].upipe_enable);
		dev_dbg(dev, "\t.upipe_pin_cfg.opaque_pin_cfg = %d\n",
			cfg->output_pins[i].upipe_pin_cfg.opaque_pin_cfg);
		dev_dbg(dev, "\t.upipe_pin_cfg.plane_offset_1 = %d\n",
			cfg->output_pins[i].upipe_pin_cfg.plane_offset_1);
		dev_dbg(dev, "\t.upipe_pin_cfg.plane_offset_2 = %d\n",
			cfg->output_pins[i].upipe_pin_cfg.plane_offset_2);
		dev_dbg(dev, "\t.upipe_pin_cfg.singel_uob_fifo = %d\n",
			cfg->output_pins[i].upipe_pin_cfg.single_uob_fifo);
		dev_dbg(dev, "\t.upipe_pin_cfg.shared_uob_fifo = %d\n",
			cfg->output_pins[i].upipe_pin_cfg.shared_uob_fifo);
	}
	dev_dbg(dev, "---------------------------\n");
}

void ipu7_fw_isys_dump_frame_buff_set(struct device *dev,
				      struct ipu7_insys_buffset *buf,
				      unsigned int outputs)
{
	unsigned int i;

	dev_dbg(dev, "--------------------------\n");
	dev_dbg(dev, "IPU_ISYS_BUFF_SET\n");
	dev_dbg(dev, ".capture_msg_map = %d\n", buf->capture_msg_map);
	dev_dbg(dev, ".frame_id = %d\n", buf->frame_id);
	dev_dbg(dev, ".skip_frame = %d\n", buf->skip_frame);

	for (i = 0; i < outputs; i++) {
		dev_dbg(dev, ".output_pin[%d]:\n", i);
		dev_dbg(dev, "\t.pin_payload.user_token = %llx\n",
			buf->output_pins[i].pin_payload.user_token);
		dev_dbg(dev, "\t.pin_payload.addr = 0x%x\n",
			buf->output_pins[i].pin_payload.addr);
		dev_dbg(dev, "\t.pin_payload.upipe_capture_cfg = 0x%x\n",
			buf->output_pins[i].upipe_capture_cfg);
	}
	dev_dbg(dev, "---------------------------\n");
}
