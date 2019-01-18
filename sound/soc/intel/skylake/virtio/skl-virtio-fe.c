// SPDX-License-Identifier: GPL-2.0+
//
// skl-virtio-fe.c  --  Virtio FE audio driver for SKL architecture
//
// Copyright (C) 2018 Intel Corporation.
//
// Authors: Furtak, Pawel <pawel.furtak@intel.com>
//          Janca, Grzegorz <grzegorz.janca@intel.com>
//
//  FE driver registers various operations such as DSP, PCM or sound card
//  control. The operations use virtio IPC to forward request to BE driver.
//  FE also receives buffer position updates form BE and informs ALSA about
//  the position.

#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/scatterlist.h>
#include <linux/vhm/acrn_common.h>
#include <linux/vhm/acrn_vhm_ioreq.h>
#include <linux/vhm/acrn_vhm_mm.h>
#include <linux/vhm/vhm_vm_mngt.h>
#include <linux/virtio.h>
#include <linux/virtio_ids.h>
#include <linux/virtio_config.h>
#include <linux/dma-mapping.h>
#include <linux/pci.h>
#include <sound/pcm_params.h>

#include <sound/soc-acpi.h>
#include <sound/soc-acpi-intel-match.h>

#include "../skl-sst-ipc.h"
#include "skl-virtio-fe.h"
#include "skl-virtio.h"

#include <linux/time.h>

static struct snd_skl_vfe *skl_vfe;

static struct snd_skl_vfe *get_virtio_audio_fe(void)
{
	return skl_vfe;
}

struct vfe_substream_info *vfe_find_substream_info_by_pcm(
	struct snd_skl_vfe *vfe, char *pcm_id, int direction)
{
	struct vfe_substream_info *info;

	list_for_each_entry(info, &vfe->substr_info_list, list) {
		if (info->direction == direction &&
			strncmp(info->pcm->id, pcm_id,
					ARRAY_SIZE(info->pcm->id)) == 0)
			return info;
	}

	return NULL;
}

inline struct vfe_substream_info *vfe_find_substream_info(
	struct snd_skl_vfe *vfe, struct snd_pcm_substream *substr)
{
	return vfe_find_substream_info_by_pcm(vfe,
			substr->pcm->id, substr->stream);
}

inline int vfe_is_valid_pcm_id(char *pcm_id)
{
	if (pcm_id == NULL || strlen(pcm_id) == 0 ||
			strcmp(pcm_id, "((null))") == 0)
		return -EINVAL;

	return 0;
}

inline int vfe_is_valid_fe_substream(struct snd_pcm_substream *substream)
{
	return vfe_is_valid_pcm_id(substream->pcm->id);
}

struct vfe_kcontrol *vfe_find_kcontrol(struct snd_skl_vfe *vfe,
	struct snd_kcontrol *kcontrol)
{
	struct vfe_kcontrol *vfe_kcontrol;

	list_for_each_entry(vfe_kcontrol, &vfe->kcontrols_list, list) {
		if (kcontrol == vfe_kcontrol->kcontrol)
			return vfe_kcontrol;
	}

	return NULL;
}

const struct snd_pcm *vfe_skl_find_pcm_by_name(struct skl *skl, char *pcm_name)
{
	const struct snd_soc_pcm_runtime *rtd;
	int ret = vfe_is_valid_pcm_id(pcm_name);

	if (ret < 0)
		return NULL;

	list_for_each_entry(rtd, &skl->component->card->rtd_list, list) {
		if (strncmp(rtd->pcm->id, pcm_name,
				ARRAY_SIZE(rtd->pcm->id)) == 0)
			return rtd->pcm;
	}
	return NULL;
}

static int vfe_send_virtio_msg(struct snd_skl_vfe *vfe,
	struct virtqueue *vq, struct scatterlist *sgs, int sg_count,
	void *data, bool out)
{
	unsigned long irq_flags;
	int ret;

	if (!vq)
		return -EINVAL;


	spin_lock_irqsave(&vfe->ipc_vq_lock, irq_flags);
	if (out)
		ret = virtqueue_add_outbuf(vq, sgs, sg_count, data, GFP_KERNEL);
	else
		ret = virtqueue_add_inbuf(vq, sgs, sg_count, data, GFP_KERNEL);
	spin_unlock_irqrestore(&vfe->ipc_vq_lock, irq_flags);

	if (ret < 0) {
		dev_err(&vfe->vdev->dev,
			"error: could not send messageover virtqueue %d\n",
			 ret);
		return ret;
	}

	spin_lock_irqsave(&vfe->ipc_vq_lock, irq_flags);
	virtqueue_kick(vq);
	spin_unlock_irqrestore(&vfe->ipc_vq_lock, irq_flags);

	return 0;
}

static int vfe_send_msg(struct snd_skl_vfe *vfe,
	struct vfe_msg_header *msg_header, void *tx_data, int tx_size,
	void *rx_data, int rx_size)
{
	wait_queue_head_t waitq;
	struct scatterlist sgs[3];
	struct vfe_ipc_msg *msg;
	int ret;
	bool completed = false;

	if (!msg_header)
		return -EINVAL;

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	memcpy(&msg->header, msg_header, sizeof(msg->header));
	msg->tx_data = tx_data;
	msg->tx_size = tx_size;
	if (tx_data) {
		msg->tx_buf = kzalloc(tx_size, GFP_KERNEL);
		if (!msg->tx_buf)
			goto out_no_mem;
		memcpy(msg->tx_buf, msg->tx_data, tx_size);
	}
	msg->rx_data = rx_data;
	msg->rx_size = rx_size;
	if (rx_data) {
		msg->rx_buf = kzalloc(rx_size, GFP_KERNEL);
		if (!msg->rx_buf)
			goto out_no_mem;
		memcpy(msg->rx_buf, msg->rx_data, rx_size);
	}

	sg_init_table(sgs, 3);
	sg_set_buf(&sgs[0], &msg->header, sizeof(msg->header));
	if (msg->tx_buf)
		sg_set_buf(&sgs[1], msg->tx_buf, tx_size);

	if (msg->rx_buf)
		sg_set_buf(&sgs[2], msg->rx_buf, rx_size);

	if (rx_data) {
		init_waitqueue_head(&waitq);

		msg->waitq = &waitq;
		msg->completed = &completed;
	}

	ret = vfe_send_virtio_msg(vfe, vfe->ipc_not_tx_vq, sgs, 3, msg, true);
	if (ret < 0)
		return ret;

	// If response is expected, wait for it
	if (rx_data) {
		ret = wait_event_timeout(waitq, completed,
				msecs_to_jiffies(VFE_MSG_MSEC_TIMEOUT));
		if (ret == 0) {
			atomic_set(&msg->status, VFE_MSG_TIMED_OUT);
			dev_err(&vfe->vdev->dev, "Response from backend timed out\n");
			return -ETIMEDOUT;
		}
	}

	return 0;

out_no_mem:
	kfree(msg->rx_buf);
	kfree(msg->tx_buf);
	kfree(msg);

	return -ENOMEM;
}

static int vfe_send_pos_request(struct snd_skl_vfe *vfe,
		struct vfe_hw_pos_request *request)
{
	struct scatterlist sg;

	sg_init_one(&sg, request, sizeof(*request));

	return vfe_send_virtio_msg(vfe, vfe->ipc_not_rx_vq,
			&sg, 1, request, false);
}

static int vfe_send_kctl_msg(struct snd_skl_vfe *vfe,
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct vfe_kctl_value kcontrol_value;
	struct vfe_msg_header msg_header;
	struct vfe_kctl_info *kctl_desc = &msg_header.desc.kcontrol;

	msg_header.cmd = VFE_MSG_KCTL_SET;
	strncpy(kctl_desc->kcontrol_id, kcontrol->id.name,
			ARRAY_SIZE(kcontrol->id.name));
	kcontrol_value.value = *ucontrol;

	return vfe_send_msg(vfe, &msg_header, &kcontrol_value,
			sizeof(kcontrol_value), NULL, 0);
}


//TODO: make it to use same mechanism as vfe_send_pcm_msg
static int vfe_send_dsp_ipc_msg(struct snd_skl_vfe *vfe,
	struct ipc_message *msg)
{
	struct scatterlist sgs[2];
	struct vfe_dsp_ipc_msg *ipc_msg;
	int data_size = sizeof(ipc_msg->header) + sizeof(ipc_msg->data_size);

	if (msg->tx_data != NULL && msg->tx_size > 0) {
		data_size += msg->tx_size;
	} else {
		msg->complete = true;
		list_del(&msg->list);
		sst_ipc_tx_msg_reply_complete(&vfe->sdev.skl_sst->ipc, msg);

		return 0;
	}

	ipc_msg = kzalloc(data_size, GFP_ATOMIC);
	if (!ipc_msg)
		return -ENOMEM;

	ipc_msg->header = msg->header;
	ipc_msg->ipc = msg;
	ipc_msg->data_size = msg->tx_size;

	if (msg->tx_data != NULL && msg->tx_size > 0)
		memcpy(&ipc_msg->data, msg->tx_data, msg->tx_size);

	sg_init_table(sgs, 2);
	sg_set_buf(&sgs[SKL_VIRTIO_IPC_MSG],
			 ipc_msg, data_size);
	sg_set_buf(&sgs[SKL_VIRTIO_IPC_REPLY],
			 msg->rx_data, msg->rx_size);

	vfe->msg = msg;

	return vfe_send_virtio_msg(vfe, vfe->ipc_cmd_tx_vq,
			sgs, 2, ipc_msg, true);
}

/* send the IPC message completed, this means the BE has received the cmd */
static void vfe_cmd_tx_done(struct virtqueue *vq)
{
	struct snd_skl_vfe *vfe = vq->vdev->priv;
	struct vfe_dsp_ipc_msg *msg;
	unsigned long irq_flags;
	unsigned int buflen = 0;

	while (true) {
		spin_lock_irqsave(&vfe->ipc_vq_lock, irq_flags);
		msg = virtqueue_get_buf(vfe->ipc_cmd_tx_vq, &buflen);
		spin_unlock_irqrestore(&vfe->ipc_vq_lock, irq_flags);

		if (msg == NULL)
			break;

		msg->ipc->complete = true;
		list_del(&msg->ipc->list);
		sst_ipc_tx_msg_reply_complete(&vfe->sdev.skl_sst->ipc,
				msg->ipc);
		kfree(msg);
	}
}

static void vfe_cmd_handle_rx(struct virtqueue *vq)
{
}

static void vfe_not_tx_done(struct virtqueue *vq)
{
	struct snd_skl_vfe *vfe = vq->vdev->priv;
	enum vfe_ipc_msg_status msg_status;
	unsigned long irq_flags;
	struct vfe_ipc_msg *msg;
	unsigned int buflen = 0;


	while (true) {
		spin_lock_irqsave(&vfe->ipc_vq_lock, irq_flags);
		msg = virtqueue_get_buf(vfe->ipc_not_tx_vq, &buflen);
		spin_unlock_irqrestore(&vfe->ipc_vq_lock, irq_flags);

		if (msg == NULL)
			break;

		msg_status = atomic_read(&msg->status);
		if (msg_status == VFE_MSG_TIMED_OUT) {
			vfe_handle_timedout_not_tx_msg(vfe, msg);
			goto free_msg;
		}

		if (msg->rx_buf) {
			memcpy(msg->rx_data, msg->rx_buf, msg->rx_size);
		}

		if (msg->waitq && msg->completed) {
			*msg->completed = true;
			wake_up(msg->waitq);
		}

free_msg:
		kfree(msg->tx_buf);
		kfree(msg->rx_buf);
		kfree(msg);
	}
}

/*
 * handle the pos_update, receive the posn and send to up layer, then
 * resend the buffer to BE
 */
static void vfe_not_handle_rx(struct virtqueue *vq)
{
	struct snd_skl_vfe *vfe;

	vfe = vq->vdev->priv;
	schedule_work(&vfe->posn_update_work);
}

static void vfe_posn_update(struct work_struct *work)
{
	struct snd_pcm_substream *substream;
	struct vfe_hw_pos_request *pos_req;
	struct virtqueue *vq;
	unsigned long irq_flags;
	unsigned int buflen = 0;
	struct vfe_substream_info *substr_info;
	struct snd_skl_vfe *vfe =
		container_of(work, struct snd_skl_vfe, posn_update_work);

	vq = vfe->ipc_not_rx_vq;

	while (true) {
		spin_lock_irqsave(&vfe->ipc_vq_lock, irq_flags);
		pos_req = virtqueue_get_buf(vq, &buflen);
		spin_unlock_irqrestore(&vfe->ipc_vq_lock, irq_flags);

		if (pos_req == NULL)
			break;

		spin_lock_irqsave(&vfe->substream_info_lock, irq_flags);
		substr_info = vfe_find_substream_info_by_pcm(vfe,
			pos_req->pcm_id, pos_req->stream_dir);

		// substream may be already closed on FE side
		if (!substr_info) {
			spin_unlock_irqrestore(&vfe->substream_info_lock,
				irq_flags);
			goto send_back_msg;
		}

		substr_info->hw_ptr = pos_req->stream_pos;
		substream = substr_info->substream;
		spin_unlock_irqrestore(&vfe->substream_info_lock, irq_flags);

		snd_pcm_period_elapsed(substream);

send_back_msg:
		vfe_send_pos_request(vfe, pos_req);
	}
}

int vfe_kcontrol_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_skl_vfe *vfe = get_virtio_audio_fe();
	struct vfe_kcontrol *vfe_kcontrol = vfe_find_kcontrol(vfe, kcontrol);
	int ret;

	vfe_send_kctl_msg(vfe, kcontrol, ucontrol);

	if (vfe_kcontrol->put)
		ret = vfe_kcontrol->put(kcontrol, ucontrol);

	return 0;
}

static struct vfe_msg_header
vfe_get_pcm_msg_header(enum vfe_ipc_msg_type msg_type,
	struct snd_pcm_substream *substream)
{
		struct vfe_msg_header msg_header;
		struct vfe_pcm_info *pcm_desc = &msg_header.desc.pcm;

		msg_header.cmd = msg_type;
		strncpy(pcm_desc->pcm_id, substream->pcm->id,
				ARRAY_SIZE(pcm_desc->pcm_id));
		pcm_desc->direction = substream->stream;

		return msg_header;
}

int vfe_pcm_open(struct snd_pcm_substream *substream)
{
	struct vfe_substream_info *substr_info;
	struct vfe_msg_header msg_header;
	struct vfe_pcm_result vbe_result = { .ret = -EIO };
	unsigned long irq_flags;
	int ret;
	struct snd_skl_vfe *vfe = get_virtio_audio_fe();

	ret = skl_platform_open(substream);
	if (ret < 0)
		return ret;

	// Ignore all substreams not associated with PCM
	ret = vfe_is_valid_fe_substream(substream);
	if (ret)
		return 0;

	msg_header = vfe_get_pcm_msg_header(VFE_MSG_PCM_OPEN, substream);

	ret = vfe_send_msg(vfe, &msg_header, NULL, 0,
		&vbe_result, sizeof(vbe_result));
	if (ret < 0)
		return ret;

	if (vbe_result.ret < 0)
		return vbe_result.ret;

	substr_info = kzalloc(sizeof(*substr_info), GFP_KERNEL);
	if (!substr_info)
		return -ENOMEM;

	substr_info->pcm = substream->pcm;
	substr_info->substream = substream;
	substr_info->direction = substream->stream;

	spin_lock_irqsave(&vfe->substream_info_lock, irq_flags);
	list_add(&substr_info->list, &vfe->substr_info_list);
	spin_unlock_irqrestore(&vfe->substream_info_lock, irq_flags);

	return vbe_result.ret;
}

int vfe_pcm_close(struct snd_pcm_substream *substream)
{
	struct vfe_substream_info *sstream_info;
	struct vfe_msg_header msg_header;
	struct vfe_pcm_result vbe_result;
	unsigned long irq_flags;
	int ret;
	struct snd_skl_vfe *vfe = get_virtio_audio_fe();

	ret = vfe_is_valid_fe_substream(substream);
	if (ret)
		return 0;

	spin_lock_irqsave(&vfe->substream_info_lock, irq_flags);
	sstream_info = vfe_find_substream_info(vfe, substream);

	if (sstream_info) {
		list_del(&sstream_info->list);
		kfree(sstream_info);
	}
	spin_unlock_irqrestore(&vfe->substream_info_lock, irq_flags);

	msg_header = vfe_get_pcm_msg_header(VFE_MSG_PCM_CLOSE, substream);

	ret = vfe_send_msg(vfe, &msg_header, NULL, 0,
		&vbe_result, sizeof(vbe_result));
	if (ret < 0)
		return ret;

	return vbe_result.ret;
}

int vfe_pcm_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct vfe_pcm_hw_params vfe_params;
	struct vfe_msg_header msg_header;
	struct vfe_pcm_result vbe_result;
	int ret;
	struct snd_skl_vfe *vfe = get_virtio_audio_fe();

	ret = vfe_is_valid_fe_substream(substream);
	if (ret)
		return 0;

	vfe_params.access = params_access(params);
	vfe_params.direction = substream->stream;
	vfe_params.sample_valid_bytes = params_width(params) >> 3;
	vfe_params.buffer_fmt = SNDRV_PCM_INFO_INTERLEAVED;
	vfe_params.rate = params_rate(params);
	vfe_params.channels = params_channels(params);
	vfe_params.host_period_bytes = params_period_bytes(params);
	vfe_params.buffer_bytes = params_buffer_bytes(params);
	vfe_params.buffer_size = params_buffer_size(params);
	vfe_params.sample_container_bytes = params_width(params);
	vfe_params.frame_fmt = params_format(params);
	vfe_params.frame_subfmt = params_subformat(params);
	vfe_params.period_size = params_period_size(params);
	vfe_params.periods = params_periods(params);

	msg_header = vfe_get_pcm_msg_header(VFE_MSG_PCM_HW_PARAMS, substream);

	ret = vfe_send_msg(vfe, &msg_header, &vfe_params, sizeof(vfe_params),
					&vbe_result, sizeof(vbe_result));
	if (ret < 0)
		return ret;

	return vbe_result.ret;
}

int vfe_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_skl_vfe *vfe = get_virtio_audio_fe();
	struct vfe_msg_header msg_header;
	int ret;

	ret = skl_platform_pcm_trigger(substream, cmd);
	if (ret < 0)
		return ret;

	ret = vfe_is_valid_fe_substream(substream);
	if (ret)
		return 0;

	msg_header = vfe_get_pcm_msg_header(VFE_MSG_PCM_TRIGGER, substream);

	return vfe_send_msg(vfe, &msg_header, &cmd, sizeof(cmd), NULL, 0);
}

int vfe_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct vfe_msg_header msg_header;
	struct vfe_pcm_dma_conf dma_conf;
	struct vfe_pcm_result vbe_result;
	struct snd_sg_buf *sg_buf;
	int ret;
	struct snd_skl_vfe *vfe = get_virtio_audio_fe();

	ret = vfe_is_valid_fe_substream(substream);
	if (ret)
		return 0;

	sg_buf = snd_pcm_substream_sgbuf(substream);

	dma_conf.addr = (u64)sg_buf->table[0].addr;
	dma_conf.size = (u64)substream->runtime->dma_bytes;
	dma_conf.pages = sg_buf->pages;
	dma_conf.offset = (u64)0;

	msg_header = vfe_get_pcm_msg_header(VFE_MSG_PCM_PREPARE, substream);

	ret = vfe_send_msg(vfe, &msg_header, &dma_conf, sizeof(dma_conf),
		&vbe_result, sizeof(vbe_result));
	if (ret < 0)
		return ret;

	return vbe_result.ret;
}

snd_pcm_uframes_t vfe_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_skl_vfe *vfe = get_virtio_audio_fe();
	struct vfe_substream_info *substr_info =
		vfe_find_substream_info(vfe, substream);

	return substr_info ? substr_info->hw_ptr : 0;
}

static void vfe_handle_timedout_pcm_msg(struct snd_skl_vfe *vfe,
	struct vfe_ipc_msg *msg)
{
	struct snd_pcm_substream *substream;
	const struct vfe_pcm_info *pcm_desc = &msg->header.desc.pcm;
	const struct snd_pcm *pcm =
		vfe_skl_find_pcm_by_name(&vfe->sdev, pcm_desc->pcm_id);
	int direction = pcm_desc->direction;

	if (!pcm)
		return;

	substream = pcm->streams[direction].substream;

	switch (msg->header.cmd) {
	case VFE_MSG_PCM_OPEN:
		vfe_pcm_close(substream);
	break;
	default:
		dev_info(&vfe->vdev->dev,
			"Timed out PCM message %d not handled",
			msg->header.cmd);
	break;
	}
}

void vfe_handle_timedout_not_tx_msg(struct snd_skl_vfe *vfe,
	struct vfe_ipc_msg *msg)
{
	switch (msg->header.cmd & VFE_MSG_TYPE_MASK) {
	case VFE_MSG_PCM:
		vfe_handle_timedout_pcm_msg(vfe, msg);
	break;
	default:
		dev_info(&vfe->vdev->dev,
			"Timed out message %d not handled",
			msg->header.cmd);
	break;
	}
}

static const char *const vfe_skl_vq_names[SKL_VIRTIO_NUM_OF_VQS] = {
	SKL_VIRTIO_IPC_CMD_TX_VQ_NAME,
	SKL_VIRTIO_IPC_CMD_RX_VQ_NAME,
	SKL_VIRTIO_IPC_NOT_TX_VQ_NAME,
	SKL_VIRTIO_IPC_NOT_RX_VQ_NAME,
};

static struct snd_soc_acpi_mach vfe_acpi_mach = {
	.drv_name = "skl_virtio_card",
	.fw_filename = "intel/dsp_fw_bxtn.bin",
	.asoc_plat_name = "virtio4",
};

static struct pci_device_id vfe_pci_device_id = {
	PCI_DEVICE(0x8086, 0x8063),
	.driver_data = (unsigned long)&vfe_acpi_mach
};

int vfe_wrap_kcontrol(struct snd_skl_vfe *vfe, struct snd_kcontrol *kcontrol)
{
	struct vfe_kcontrol *vfe_kcontrol = devm_kzalloc(&vfe->vdev->dev,
		sizeof(*vfe_kcontrol), GFP_KERNEL);

	if (!vfe_kcontrol)
		return -ENOMEM;

	vfe_kcontrol->kcontrol = kcontrol;
	vfe_kcontrol->put = kcontrol->put;
	kcontrol->put = vfe_kcontrol_put;

	list_add(&vfe_kcontrol->list, &vfe->kcontrols_list);
	return 0;
}

int vfe_wrap_native_driver(struct snd_skl_vfe *vfe,
	struct platform_device *pdev, struct snd_soc_card *card)
{
	struct snd_kcontrol *kctl;
	int ret;

	list_for_each_entry(kctl, &card->snd_card->controls, list) {
		ret = vfe_wrap_kcontrol(vfe, kctl);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static struct snd_pcm_ops vfe_platform_ops = {
	.open = vfe_pcm_open,
	.close = vfe_pcm_close,
	.hw_params = vfe_pcm_hw_params,
	.pointer = vfe_pcm_pointer,
	.trigger = vfe_pcm_trigger,
	.prepare = vfe_pcm_prepare,
	.ioctl = snd_pcm_lib_ioctl,
	.mmap = snd_pcm_lib_default_mmap,
	.page = snd_pcm_sgbuf_ops_page,
};

static const struct snd_soc_component_driver vfe_component_drv  = {
	.name		= "virt-pcm",
	.probe		= skl_platform_soc_probe,
	.ops		= &vfe_platform_ops,
	.pcm_new	= skl_pcm_new,
	.pcm_free	= skl_pcm_free,
};


static int vfe_platform_register(struct snd_skl_vfe *vfe, struct device *dev)
{
	int result = skl_platform_component_register(dev, &vfe_component_drv);

	return result;
}

static int vfe_machine_device_register(struct snd_skl_vfe *vfe, struct skl *skl)
{
	struct snd_soc_acpi_mach *mach = skl->mach;
	struct platform_device *pdev;
	int ret;

	pdev = platform_device_alloc(mach->drv_name, -1);
	if (!pdev) {
		dev_err(&vfe->vdev->dev, "platform device alloc failed\n");
		return -EIO;
	}

	dev_set_drvdata(&pdev->dev, vfe);

	ret = platform_device_add(pdev);
	if (ret < 0) {
		dev_err(&vfe->vdev->dev, "failed to add machine device\n");
		platform_device_put(pdev);
		return ret;
	}

	skl->i2s_dev = pdev;

	return 0;
}

static void vfe_machine_device_unregister(struct skl *skl)
{
	if (skl->i2s_dev)
		platform_device_unregister(skl->i2s_dev);
}

static int vfe_skl_init_dsp(struct skl *skl)
{
	const struct skl_dsp_ops *ops;
	struct skl_dsp_cores *cores;
	int ret;

	struct hdac_bus *bus = &skl->hbus;
	int irq = bus->irq;

	ops = skl_get_dsp_ops(skl->pci->device);
	if (!ops)
		return -EIO;

	ret = ops->init(bus->dev, NULL, irq, skl->fw_name, ops->loader_ops(),
					&skl->skl_sst, NULL);
	if (ret < 0)
		return ret;

	skl->skl_sst->dsp_ops = ops;
	cores = &skl->skl_sst->cores;
	cores->count = ops->num_cores;

	cores->state = devm_kcalloc(bus->dev, cores->count,
		sizeof(*cores->state), GFP_KERNEL);
	if (!cores->state)
		return -ENOMEM;

	cores->usage_count = devm_kcalloc(bus->dev, cores->count,
		sizeof(*cores->usage_count), GFP_KERNEL);
	if (!cores->usage_count)
		return -ENOMEM;

	INIT_LIST_HEAD(&skl->skl_sst->notify_kctls);

	return ret;
}

static int vfe_skl_init_hbus(struct snd_skl_vfe *vfe, struct skl *skl)
{
	struct vfe_msg_header msg_header;
	struct vfe_hda_cfg hda_cfg;
	struct hdac_bus *bus = &skl->hbus;
	struct virtio_device *vdev = vfe->vdev;
	int cp_streams, pb_streams, start_idx;
	int ret;

	msg_header.cmd = VFE_MSG_CFG_HDA;

	ret = vfe_send_msg(vfe, &msg_header, NULL, 0,
		&hda_cfg, sizeof(hda_cfg));
	if (ret < 0)
		return ret;

	snd_hdac_ext_bus_init(bus, &vdev->dev, NULL, NULL, NULL);
	bus->use_posbuf = 1;
	bus->bdl_pos_adj = 0;
	dev_set_drvdata(&vdev->dev, bus);

	bus->remap_addr = devm_kzalloc(&vdev->dev,
			hda_cfg.resource_length, GFP_KERNEL);
	if (bus->remap_addr == NULL)
		return -ENXIO;

	bus->ppcap = hda_cfg.ppcap ? bus->remap_addr + hda_cfg.ppcap : 0;
	bus->spbcap = hda_cfg.spbcap ? bus->remap_addr + hda_cfg.spbcap : 0;
	bus->mlcap = hda_cfg.mlcap ? bus->remap_addr + hda_cfg.mlcap : 0;
	bus->gtscap = hda_cfg.gtscap ? bus->remap_addr + hda_cfg.gtscap : 0;
	bus->drsmcap = hda_cfg.drsmcap ? bus->remap_addr + hda_cfg.drsmcap : 0;

	cp_streams = hda_cfg.cp_streams;
	pb_streams = hda_cfg.pb_streams;

	if (!pb_streams && !cp_streams)
		return -EIO;

	bus->num_streams = cp_streams + pb_streams;

	/* initialize streams */
	snd_hdac_ext_stream_init_all
		(bus, 0, cp_streams, SNDRV_PCM_STREAM_CAPTURE);
	start_idx = cp_streams;
	snd_hdac_ext_stream_init_all
		(bus, start_idx, pb_streams, SNDRV_PCM_STREAM_PLAYBACK);

	ret = snd_hdac_bus_alloc_stream_pages(bus);
	if (ret < 0)
		return ret;

	return 0;
}


static struct nhlt_acpi_table *vfe_skl_nhlt_init(struct device *dev)
{
	struct nhlt_acpi_table *nhlt;
	struct nhlt_endpoint *nhlt_ep;
	int ep_count = 6, cnt = 0, nhlt_size;

	nhlt_size = sizeof(*nhlt) + sizeof(struct nhlt_endpoint) * ep_count;
	nhlt = devm_kzalloc(dev, nhlt_size, GFP_KERNEL);
	if (!nhlt)
		return NULL;

	//TODO: instead of hardcoded definition
	//nhlt configuration should be read from BE
	nhlt->endpoint_count = ep_count;
	nhlt_ep = (struct nhlt_endpoint *)nhlt->desc;

	for (cnt = 0; cnt < ep_count; cnt++) {
		nhlt_ep->length = sizeof(struct nhlt_endpoint);
		nhlt_ep->linktype = NHLT_LINK_SSP;
		nhlt_ep->virtual_bus_id = cnt;

		nhlt_ep = (struct nhlt_endpoint *)((u8 *)nhlt_ep +
				nhlt_ep->length);
	}

	return nhlt;
}


void vfe_skl_pci_dev_release(struct device *dev)
{
}

static int vfe_skl_init(struct virtio_device *vdev)
{
	int err;
	struct snd_skl_vfe *vfe = vdev->priv;
	struct skl *skl = &vfe->sdev;

	skl->pci = devm_kzalloc(&vdev->dev, sizeof(*skl->pci), GFP_KERNEL);
	if (!skl->pci)
		return -ENOMEM;

	skl->pci->device = vfe_pci_device_id.device;
	device_initialize(&skl->pci->dev);
	skl->pci->dev.parent = &vfe->vdev->dev;
	skl->pci->dev.release = vfe_skl_pci_dev_release;
	skl->pci->dev.bus = vfe->vdev->dev.bus;
	skl->pci->dev.coherent_dma_mask = vfe->vdev->dev.coherent_dma_mask;
	skl->pci->dev.dma_mask = &skl->pci->dev.coherent_dma_mask;

	dev_set_name(&skl->pci->dev, "%s", "audio-virtio");
	err = device_add(&skl->pci->dev);
	if (err < 0)
		goto error;

	dev_set_drvdata(&skl->pci->dev, vfe);

	skl->mach = &vfe_acpi_mach;
	skl->mach->pdata = &vfe;

	skl->fw_name = skl->mach->fw_filename;
	skl->nhlt = vfe_skl_nhlt_init(&vdev->dev);

	err = vfe_skl_init_hbus(vfe, skl);
	if (err < 0)
		goto error;

	strcpy(skl->tplg_name, "5a98-INTEL-NHLT-GPA-11-tplg.bin");

	err = vfe_skl_init_dsp(skl);
	if (err < 0)
		goto error;

	err = vfe_platform_register(vfe, &vdev->dev);
	if (err < 0)
		goto error;

	err = vfe_machine_device_register(vfe, skl);
	if (err < 0)
		goto error;

	return 0;

error:
	device_unregister(&skl->pci->dev);
	return err;
}

static int vfe_init_vqs(struct snd_skl_vfe *vfe)
{
	struct virtqueue *vqs[SKL_VIRTIO_NUM_OF_VQS];
	int ret;
	struct virtio_device *vdev = vfe->vdev;
	vq_callback_t *cbs[SKL_VIRTIO_NUM_OF_VQS] =	{
			vfe_cmd_tx_done,
			vfe_cmd_handle_rx,
			vfe_not_tx_done,
			vfe_not_handle_rx
	};

	/* find virt queue for vfe to send/receive IPC message. */
	ret = virtio_find_vqs(vfe->vdev, SKL_VIRTIO_NUM_OF_VQS,
			      vqs, cbs, vfe_skl_vq_names, NULL);
	if (ret) {
		dev_err(&vdev->dev, "error: find vqs fail with %d\n", ret);
		return ret;
	}
	/* virtques */
	vfe->ipc_cmd_tx_vq = vqs[SKL_VIRTIO_IPC_CMD_TX_VQ];
	vfe->ipc_cmd_rx_vq = vqs[SKL_VIRTIO_IPC_CMD_RX_VQ];
	vfe->ipc_not_tx_vq = vqs[SKL_VIRTIO_IPC_NOT_TX_VQ];
	vfe->ipc_not_rx_vq = vqs[SKL_VIRTIO_IPC_NOT_RX_VQ];

	virtio_device_ready(vdev);

	return 0;
}

static int vfe_init(struct virtio_device *vdev)
{
	struct snd_skl_vfe *vfe;
	int ret;

	vfe = devm_kzalloc(&vdev->dev, sizeof(*vfe), GFP_KERNEL);
	if (!vfe)
		goto no_mem;

	skl_vfe = vfe;
	vfe->vdev = vdev;
	vdev->priv = vfe;

	INIT_LIST_HEAD(&vfe->kcontrols_list);
	spin_lock_init(&vfe->substream_info_lock);
	INIT_LIST_HEAD(&vfe->substr_info_list);
	spin_lock_init(&vfe->ipc_vq_lock);
	INIT_WORK(&vfe->posn_update_work, vfe_posn_update);

	vfe->send_dsp_ipc_msg = vfe_send_dsp_ipc_msg;
	vfe->notify_machine_probe = vfe_wrap_native_driver;

	ret = vfe_init_vqs(vfe);
	if (ret < 0)
		goto err;

	vfe->pos_not = devm_kmalloc(&vdev->dev,
			sizeof(*vfe->pos_not), GFP_KERNEL);
	if (!vfe->pos_not)
		goto no_mem;

	vfe_send_pos_request(vfe, vfe->pos_not);

	ret = vfe_skl_init(vdev);
	if (ret < 0)
		goto err;

	return 0;

no_mem:
	ret = -ENOMEM;
err:
	vdev->priv = NULL;
	return ret;
}

/*
 * Probe and remove.
 */
static int vfe_probe(struct virtio_device *vdev)
{
	struct device *dev;
	int ret;

	dev = &vdev->dev;
	dev->coherent_dma_mask = DMA_BIT_MASK(64);
	dev->dma_mask = &dev->coherent_dma_mask;

	ret = vfe_init(vdev);
	if (ret < 0) {
		dev_err(&vdev->dev, "failed to init virt frontend %d\n", ret);
		return ret;
	}

	dev_info(&vdev->dev, "init virtual frontend success\n");
	return 0;
}

static void vfe_remove(struct virtio_device *vdev)
{
	struct snd_skl_vfe *vfe = vdev->priv;
	if (!vfe)
		return;

	cancel_work_sync(&vfe->posn_update_work);
	vfe_machine_device_unregister(&vfe->sdev);
}

//FIXME: remove or implement with error msg that config change is not supported
static void virtaudio_config_changed(struct virtio_device *vdev)
{
}

#ifdef CONFIG_PM_SLEEP
static int vfe_freeze(struct virtio_device *vdev)
{
	vdev->config->reset(vdev);
	vdev->config->del_vqs(vdev);

	return 0;
}

static int vfe_restore(struct virtio_device *vdev)
{
	int ret;
	struct snd_skl_vfe *vfe = vdev->priv;

	ret = vfe_init_vqs(vfe);
	if (ret < 0)
		return ret;

	vfe_send_pos_request(vfe, vfe->pos_not);

	return 0;
}
#endif

const struct virtio_device_id id_table[] = {
	{VIRTIO_ID_AUDIO, VIRTIO_DEV_ANY_ID},
	{0},
};

static struct virtio_driver vfe_audio_driver = {
	.feature_table	= NULL,
	.feature_table_size	= 0,
	.driver.name	= KBUILD_MODNAME,
	.driver.owner	= THIS_MODULE,
	.id_table	= id_table,
	.probe	= vfe_probe,
	.remove	= vfe_remove,
	.config_changed	= virtaudio_config_changed,
#ifdef CONFIG_PM_SLEEP
	.freeze	= vfe_freeze,
	.restore	= vfe_restore,
#endif
};

module_virtio_driver(vfe_audio_driver);

MODULE_DEVICE_TABLE(virtio, id_table);
MODULE_DESCRIPTION("Intel Broxton Virtio FE Driver");
MODULE_LICENSE("GPL v2");
