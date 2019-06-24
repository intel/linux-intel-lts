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

struct skl;
struct vskl;
struct snd_skl_vbe;

#define skl_get_vrtdata(skl) \
	(dev_get_platdata(&skl->virt_dev->dev))
#define skl_get_vrtpdata(skl) \
	((struct skl_virt_pdata *)(skl_get_vrtdata(skl)))
#define skl_to_vskl(skl) \
	((struct vskl *)(skl_get_vrtpdata(skl)->private_data))
#define skl_get_vbe(skl) (&(skl_to_vskl(skl))->vbe)
#define vskl_get_vbe(vskl) (&vskl->vbe)

#define KCTL_DOMAIN_ITEM(STR, DOMAIN_FLAG) {STR, sizeof(STR)-1, DOMAIN_FLAG}

extern int snd_skl_vbe_register(struct skl *sdev, struct snd_skl_vbe **svbe);
extern int snd_skl_vbe_register_client(struct snd_skl_vbe *vbe);
extern void vbe_skl_handle_kick(struct snd_skl_vbe *vbe, int vq_idx);

int vbe_skl_attach(struct snd_skl_vbe *vbe, struct skl *skl);
int vbe_skl_detach(struct snd_skl_vbe *vbe, struct skl *skl);
void vbe_skl_bind(struct snd_skl_vbe *vbe, struct skl *skl);
void vbe_skl_unbind(struct snd_skl_vbe *vbe, struct skl *skl);
struct vskl *get_virtio_audio(void);

struct vskl_native_ops {
	int (*request_tplg)(struct skl *skl, const struct firmware **fw);
	void (*hda_irq_ack)(struct hdac_bus *bus, struct hdac_stream *hstr);
};

struct vbe_substream_info {
	struct snd_pcm *pcm;
	struct snd_pcm_substream *substream;
	dma_addr_t native_dma_addr;
	int direction;
	struct vfe_stream_pos_desc *pos_desc;

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
	struct list_head pending_msg_list;

	struct work_struct not_tx_handler_work;

	struct vskl_native_ops nops;

	int vmid;  /* vm id number */
};

struct snd_skl_vbe_client {
	struct snd_skl_vbe *vbe;
	int vhm_client_id;
	int max_vcpu;
	struct list_head substr_info_list;
	struct list_head list;
	struct vhm_request *req_buf;
	const struct firmware *tplg;
};

struct vskl {
	struct device *dev;
	struct snd_skl_vbe vbe;

	struct skl *skl;
};

struct vbe_static_kctl_domain {
	const char *name;
	u32 str_size;
	u32 domain_flag;
};

void skl_notify_stream_update(struct hdac_bus *bus,
		struct snd_pcm_substream *substr);
struct snd_skl_vbe_client *vbe_client_find(struct snd_skl_vbe *vbe,
	int client_id);
void vbe_skl_pcm_close_all(struct snd_skl_vbe *vbe,
		struct snd_skl_vbe_client *client);

#endif
