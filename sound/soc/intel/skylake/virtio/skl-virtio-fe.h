/* SPDX-License-Identifier: GPL-2.0
 *
 * skl-virtio-fe.h -- Virtio FE driver header
 *
 * Copyright (C) 2018 Intel Corporation.
 *
 * definitions/declarations for virtio FE driver
 */

#ifndef __SOUND_SOC_SKL_VIRTIO_FE_H
#define __SOUND_SOC_SKL_VIRTIO_FE_H

#include "skl-virtio-common.h"

#define VFE_MSG_MSEC_TIMEOUT 100
#define VFE_MSG_TRIGGER_TIMEOUT 500
#define VFE_MSG_NO_TIMEOUT 0
#define VFE_TPLG_LOAD_TIMEOUT 1000
#define VFE_MSG_BUFF_NUM 3

struct vfe_substream_info {
	struct snd_pcm *pcm;
	struct snd_pcm_substream *substream;
	struct work_struct update_work;
	int direction;
	bool open;
	bool running;

	struct vfe_stream_pos_desc *pos_desc;
	struct list_head list;
};

struct vskl_vfe_tplg {
	struct firmware tplg_data;
	u64 data_ready;

	struct mutex tplg_lock;
	wait_queue_head_t waitq;
	bool load_completed;
};

struct snd_skl_vfe {
	struct skl sdev;
	struct virtio_device *vdev;

	struct ipc_message *msg;
	void *in_buff[VFE_MSG_BUFF_NUM];

	struct vskl_vfe_tplg tplg;

	struct work_struct init_work;

	struct work_struct msg_timeout_work;
	struct work_struct rx_message_loop_work;
	struct work_struct tx_message_loop_work;

	struct workqueue_struct *posn_update_queue;

	struct mutex vq_lock;

	/* IPC cmd from frontend to backend */
	struct virtqueue           *ipc_cmd_tx_vq;
	/* IPC cmd reply from backend to frontend */
	struct virtqueue           *ipc_cmd_rx_vq;
	/* IPC notification from backend to frontend */
	struct virtqueue           *ipc_not_rx_vq;
	/* IPC notification reply from frontend to backend */
	struct virtqueue           *ipc_not_tx_vq;

	struct list_head kcontrols_list;
	struct list_head substr_info_list;
	struct list_head expired_msg_list;

	int (*send_dsp_ipc_msg)(struct snd_skl_vfe *vfe,
		struct ipc_message *msg);
	int (*notify_machine_probe)(struct snd_skl_vfe *vfe,
		struct platform_device *pdev, struct snd_soc_card *card);
};

void vfe_handle_timedout_not_tx_msg(struct snd_skl_vfe *vfe,
	struct vfe_ipc_msg *msg);

#endif
