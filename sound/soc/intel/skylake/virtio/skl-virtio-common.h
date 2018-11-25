/* SPDX-License-Identifier: GPL-2.0
 *
 * skl-virtio-common.h -- Virtio common header
 *
 * Copyright (C) 2018 Intel Corporation.
 *
 * common header for both virtio FE driver and BE service
 */

#include "../skl.h"

#ifndef __SOUND_SOC_SKL_VIRTIO_COMMON_H
#define __SOUND_SOC_SKL_VIRTIO_COMMON_H


#define SKL_VIRTIO_IPC_CMD_TX_VQ	0
#define SKL_VIRTIO_IPC_CMD_RX_VQ	1
#define SKL_VIRTIO_IPC_NOT_TX_VQ	2
#define SKL_VIRTIO_IPC_NOT_RX_VQ	3
#define SKL_VIRTIO_NUM_OF_VQS 4

#define SKL_VIRTIO_IPC_CMD_TX_VQ_NAME	"snd-skl-ipc-cmd-tx"
#define SKL_VIRTIO_IPC_CMD_RX_VQ_NAME	"snd-skl-ipc-cmd-rx"
#define SKL_VIRTIO_IPC_NOT_TX_VQ_NAME	"snd-skl-ipc-not-tx"
#define SKL_VIRTIO_IPC_NOT_RX_VQ_NAME	"snd-skl-ipc-not-rx"

#define SKL_VIRTIO_IPC_MSG 0
#define SKL_VIRTIO_IPC_REPLY 1

struct vfe_dsp_ipc_msg {
	u64 header;
	struct ipc_message *ipc;
	size_t data_size;
	void *data;
};

struct vfe_pcm_info {
	char pcm_id[64];
	int direction;
};

struct vfe_kctl_info {
	char kcontrol_id[64];
};

struct vfe_msg_header {
	int cmd;

	union {
		struct vfe_pcm_info pcm;
		struct vfe_kctl_info kcontrol;
	} desc;
};

struct vfe_ipc_msg {
	struct vfe_msg_header header;

	int tx_size;
	void *tx_data;
	int rx_size;
	void *rx_data;

	wait_queue_head_t *waitq;
	bool *completed;

	void *tx_buf;
	void *rx_buf;
};

struct vbe_ipc_msg {
	struct vfe_msg_header *header;

	int tx_size;
	void *tx_data;

	int rx_size;
	void *rx_data;
};

struct vfe_kctl_value {
	struct snd_ctl_elem_value value;
};

/* stream ring info */
struct vfe_pcm_dma_conf {
	uint64_t addr;
	uint32_t pages;
	uint32_t size;
	uint32_t offset;
};


struct vfe_pcm_hw_params {
	uint32_t access;
	uint32_t direction;
	uint32_t frame_fmt;
	uint32_t frame_subfmt;
	uint32_t buffer_fmt;
	uint32_t buffer_size;
	uint32_t buffer_bytes;
	uint32_t periods;
	uint32_t period_size;
	uint32_t stream_tag;
	uint32_t rate;
	uint32_t channels;
	uint32_t sample_valid_bytes;
	uint32_t sample_container_bytes;
	uint32_t host_period_bytes;
};

struct vfe_hw_pos_request {
	char pcm_id[64];
	u32 stream_dir;
	u64 stream_pos;
	struct list_head list;
};

struct vfe_pcm_result {
	int ret;
};

struct vfe_hda_cfg {
	u32 resource_length;
	u32 ppcap;
	u32 spbcap;
	u32 mlcap;
	u32 gtscap;
	u32 drsmcap;
	u8 cp_streams;
	u8 pb_streams;
};

#define VFE_MSG_TYPE_OFFSET 8
#define VFE_MSG_TYPE_MASK (0xFF << VFE_MSG_TYPE_OFFSET)

enum vfe_ipc_msg_type {
	VFE_MSG_PCM = 1 << VFE_MSG_TYPE_OFFSET,
	VFE_MSG_KCTL = 2 << VFE_MSG_TYPE_OFFSET,
	VFE_MSG_TPLG = 3 << VFE_MSG_TYPE_OFFSET,
	VFE_MSG_CFG = 4 << VFE_MSG_TYPE_OFFSET,

	VFE_MSG_PCM_OPEN = VFE_MSG_PCM | 0x01,
	VFE_MSG_PCM_CLOSE = VFE_MSG_PCM | 0x02,
	VFE_MSG_PCM_PREPARE = VFE_MSG_PCM | 0x03,
	VFE_MSG_PCM_HW_PARAMS = VFE_MSG_PCM | 0x04,
	VFE_MSG_PCM_TRIGGER = VFE_MSG_PCM | 0x05,

	VFE_MSG_KCTL_SET = VFE_MSG_KCTL | 0x01,

	VFE_MSG_CFG_HDA = VFE_MSG_CFG | 0x01,
};

#endif
