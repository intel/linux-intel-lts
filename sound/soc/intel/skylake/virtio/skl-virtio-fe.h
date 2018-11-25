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

struct vfe_substream_info {
	struct snd_pcm *pcm;
	struct snd_pcm_substream *substream;
	int direction;

	u64 hw_ptr;
	struct list_head list;
};

struct vfe_kcontrol {
	struct snd_kcontrol *kcontrol;
	struct list_head list;

	snd_kcontrol_put_t *put;
};

struct snd_skl_vfe {
	struct skl sdev;
	struct virtio_device *vdev;

	struct ipc_message *msg;
	struct vfe_hw_pos_request *pos_not;

	/* position update work */
	struct work_struct posn_update_work;

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

	int (*send_dsp_ipc_msg)(struct snd_skl_vfe *vfe,
		struct ipc_message *msg);
	int (*notify_machine_probe)(struct snd_skl_vfe *vfe,
		struct platform_device *pdev, struct snd_soc_card *card);
};

#endif
