/* SPDX-License-Identifier: GPL-2.0
 *
 * skl-virtio-be.h -- Virtio BE service header
 *
 * Copyright (C) 2018 Intel Corporation.
 *
 * definitions/declarations for virtio BE service
 */

#ifndef __SOUND_SOC_SKL_VIRTIO_BE_H
#define __SOUND_SOC_SKL_VIRTIO_BE_H

#include <linux/vbs/vbs.h>
#include "skl-virtio-common.h"


#define SKL_VIRTIO_NOT_VQ_SZ  3
#define SKL_VIRTIO_IPC_VQ_SZ  2

#define SKL_VIRTIO_MSG_HEADER  0
#define SKL_VIRTIO_MSG_TX  1
#define SKL_VIRTIO_MSG_RX  2

struct snd_skl_vbe;

extern int snd_skl_vbe_register(struct skl *sdev, struct snd_skl_vbe **svbe);
extern int snd_skl_vbe_register_client(struct snd_skl_vbe *vbe);
extern void vbe_skl_handle_kick(const struct snd_skl_vbe *vbe, int vq_idx);

struct vbe_substream_info {
	struct snd_pcm *pcm;
	struct snd_pcm_substream *substream;
	int direction;

	struct snd_skl_vbe *vbe;
	struct list_head list;
};


struct snd_skl_vbe {
	struct skl *sdev;
	struct device *dev;
	struct virtio_dev_info dev_info;
	struct virtio_vq_info vqs[SKL_VIRTIO_NUM_OF_VQS];

	spinlock_t posn_lock;

	struct list_head client_list;
	struct list_head substr_info_list;
	struct list_head list;
	struct list_head posn_list;

	int vmid;  /* vm id number */
};

struct snd_skl_vbe_client {
	struct snd_skl_vbe *vbe;
	int vhm_client_id;
	int max_vcpu;
	struct list_head list;
	struct vhm_request *req_buf;
};

struct virtio_miscdev {
	struct device *dev;
	int (*open)(struct file *f, void *data);
	long (*ioctl)(struct file *f, void *data, unsigned int ioctl,
		      unsigned long arg);
	int (*release)(struct file *f, void *data);
	void *priv;
};

#endif
