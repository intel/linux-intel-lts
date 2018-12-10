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

#define SKL_VIRTIO_DOMAIN_NAME_LEN 20

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
	u32 domain_id;
	char domain_name[SKL_VIRTIO_DOMAIN_NAME_LEN];

	union {
		struct vfe_pcm_info pcm;
		struct vfe_kctl_info kcontrol;
	} desc;
};

enum vfe_ipc_msg_status {
	VFE_MSG_PENDING = 0,
	VFE_MSG_TIMED_OUT,
	VFE_MSG_COMPLETED,
};

struct vfe_ipc_msg {
	struct vfe_msg_header header;

	int tx_size;
	void *tx_data;
	int rx_size;
	void *rx_data;

	atomic_t status;
	wait_queue_head_t *waitq;
	bool *completed;

	void *tx_buf;
	void *rx_buf;

	struct list_head list;
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

struct vfe_pcm_result {
	int ret;
};

struct vfe_kctl_result {
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

struct vfe_inbox_header {
	int msg_type;
};

struct vfe_hw_pos_request {
	int msg_type;
	char pcm_id[64];
	u32 stream_dir;
	u64 stream_pos;
};

struct vfe_kctl_noti {
	int msg_type;
	struct vfe_kctl_info kcontrol;
	struct vfe_kctl_value kcontrol_value;
};

union inbox_msg {
	struct vfe_hw_pos_request posn;
	struct vfe_kctl_noti kctln;
};

struct vfe_pending_msg {
	union inbox_msg msg;
	unsigned int sizeof_msg;
	struct list_head list;
};

#define VFE_MSG_TYPE_OFFSET 8
#define VFE_MSG_TYPE_MASK (0xFF << VFE_MSG_TYPE_OFFSET)

enum vfe_ipc_msg_type {
	VFE_MSG_PCM = 1 << VFE_MSG_TYPE_OFFSET,
	VFE_MSG_KCTL = 2 << VFE_MSG_TYPE_OFFSET,
	VFE_MSG_TPLG = 3 << VFE_MSG_TYPE_OFFSET,
	VFE_MSG_CFG = 4 << VFE_MSG_TYPE_OFFSET,
	VFE_MSG_POS = 5 << VFE_MSG_TYPE_OFFSET,

	VFE_MSG_PCM_OPEN = VFE_MSG_PCM | 0x01,
	VFE_MSG_PCM_CLOSE = VFE_MSG_PCM | 0x02,
	VFE_MSG_PCM_PREPARE = VFE_MSG_PCM | 0x03,
	VFE_MSG_PCM_HW_PARAMS = VFE_MSG_PCM | 0x04,
	VFE_MSG_PCM_TRIGGER = VFE_MSG_PCM | 0x05,

	VFE_MSG_KCTL_SET = VFE_MSG_KCTL | 0x01,

	VFE_MSG_CFG_HDA = VFE_MSG_CFG | 0x01,

	VFE_MSG_POS_NOTI = VFE_MSG_POS | 0x01,
};

struct kctl_wrapper {
	struct snd_kcontrol *kcontrol;
	struct list_head list;
	snd_kcontrol_put_t *put;
};

typedef int (*kctl_send_op)(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol,
		struct vfe_kctl_result *result);

typedef int (*kctl_perm_op)(u32 domain_id,
		const struct snd_kcontrol *kcontrol);

struct kctl_ops {
	kctl_send_op send_noti;
	kctl_perm_op check_permission;
};

struct kctl_proxy {
	struct device *alloc_dev;
	struct kctl_ops ops;
	struct list_head kcontrols_list;
};

struct kctl_proxy *get_kctl_proxy(void);
int kctl_ipc_handle(u32 domain_id, const struct vfe_kctl_info *kctl_info,
	struct vfe_kctl_value *kcontrol_val, struct vfe_kctl_result *result);
void kctl_init_proxy(struct device *dev, struct kctl_ops *kt_ops);

#endif
