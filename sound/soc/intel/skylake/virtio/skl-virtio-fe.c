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
static char *domain_name = "GuestOS";
static u32 domain_id = ~0;

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

static void vfe_vq_kick(struct snd_skl_vfe *vfe, struct virtqueue *vq)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&vfe->ipc_vq_lock, irq_flags);
	virtqueue_kick(vq);
	spin_unlock_irqrestore(&vfe->ipc_vq_lock, irq_flags);
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

	vfe_vq_kick(vfe, vq);

	return 0;
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

static int vfe_wait_for_msg_response(struct snd_skl_vfe *vfe,
	struct vfe_ipc_msg *msg,
	wait_queue_head_t *waitq,
	bool *completed,
	int timeout)
{
	int ret = 0;

	if (!timeout) {
		wait_event(*waitq, *completed);
		return 0;
	}

	ret =  wait_event_timeout(*waitq, *completed,
				msecs_to_jiffies(timeout));

	if (ret == 0) {
		atomic_set(&msg->status, VFE_MSG_TIMED_OUT);
		dev_err(&vfe->vdev->dev, "Response from backend timed out\n");
		return -ETIMEDOUT;
	}

	return 0;
}

int vfe_send_msg_with_timeout(struct snd_skl_vfe *vfe,
	struct vfe_msg_header *msg_header, void *tx_data, int tx_size,
	void *rx_data, int rx_size, int timeout)
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

	strncpy(msg_header->domain_name, domain_name,
		ARRAY_SIZE(msg_header->domain_name)-1);
	msg_header->domain_name[SKL_VIRTIO_DOMAIN_NAME_LEN-1] = '\0';
	msg_header->domain_id = domain_id;
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
		ret = vfe_wait_for_msg_response(vfe, msg, &waitq,
			&completed, timeout);
		if (ret < 0)
			return ret;
	}

	return 0;

out_no_mem:
	kfree(msg->rx_buf);
	kfree(msg->tx_buf);
	kfree(msg);

	return -ENOMEM;
}

int vfe_send_blocking_msg(struct snd_skl_vfe *vfe,
	struct vfe_msg_header *msg_header, void *tx_data, int tx_size,
	void *rx_data, int rx_size)
{
	return vfe_send_msg_with_timeout(vfe, msg_header, tx_data,
		tx_size, rx_data, rx_size, VFE_MSG_NO_TIMEOUT);
}

int vfe_send_msg(struct snd_skl_vfe *vfe,
	struct vfe_msg_header *msg_header, void *tx_data, int tx_size,
	void *rx_data, int rx_size)
{
	return vfe_send_msg_with_timeout(vfe, msg_header, tx_data,
		tx_size, rx_data, rx_size, VFE_MSG_MSEC_TIMEOUT);
}

static int vfe_send_kctl_msg(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol, struct vfe_kctl_result *result)
{
	struct snd_skl_vfe *vfe = get_virtio_audio_fe();
	struct vfe_kctl_value kcontrol_value;
	struct vfe_msg_header msg_header;
	struct vfe_kctl_info *kctl_desc = &msg_header.desc.kcontrol;

	msg_header.cmd = VFE_MSG_KCTL_SET;
	strncpy(kctl_desc->kcontrol_id, kcontrol->id.name,
			ARRAY_SIZE(kcontrol->id.name));
	kcontrol_value.value = *ucontrol;

	return vfe_send_msg(vfe, &msg_header, &kcontrol_value,
			sizeof(kcontrol_value), result,
			sizeof(struct vfe_kctl_result));
}

static int vfe_init_irq_queue(struct snd_skl_vfe *vfe)
{
	struct scatterlist sg;

	sg_init_one(&sg, vfe, sizeof(struct snd_skl_vfe));

	return vfe_send_virtio_msg(vfe, vfe->ipc_cmd_rx_vq,
			&sg, 1, vfe, false);
}

static int vfe_put_inbox_buffer(struct snd_skl_vfe *vfe,
		void *buff)
{
	struct scatterlist sg;

	sg_init_one(&sg, buff, sizeof(union inbox_msg));

	return vfe_send_virtio_msg(vfe, vfe->ipc_not_rx_vq,
			&sg, 1, buff, false);
}

//TODO: make it to use same mechanism as vfe_send_pcm_msg
static int vfe_send_dsp_ipc_msg(struct snd_skl_vfe *vfe,
	struct ipc_message *msg)
{
	sst_ipc_tx_msg_reply_complete(&vfe->sdev.skl_sst->ipc,
		msg);

	return 0;
}

static void vfe_cmd_tx_done(struct virtqueue *vq)
{
}

static void vfe_cmd_handle_rx(struct virtqueue *vq)
{
	struct snd_skl_vfe *vfe;
	unsigned long irq_flags;
	struct vfe_substream_info *substr_info, *tmp;
	struct vfe_updated_substream *updated_stream;
	struct vfe_stream_pos_desc *pos_desc;

	vfe = vq->vdev->priv;

	spin_lock_irqsave(&vfe->substream_info_lock, irq_flags);
	list_for_each_entry_safe(substr_info, tmp,
			&vfe->substr_info_list, list) {
		pos_desc = substr_info->pos_desc;
		if (!pos_desc ||
				pos_desc->be_irq_cnt == pos_desc->fe_irq_cnt)
			continue;
		if (pos_desc->be_irq_cnt - pos_desc->fe_irq_cnt > 1)
			dev_warn(&vfe->vdev->dev, "Missed interrupts on fe side\n");
		pos_desc->fe_irq_cnt = pos_desc->be_irq_cnt;

		updated_stream =
			kzalloc(sizeof(*updated_stream), GFP_ATOMIC);

		if (!updated_stream) {
			dev_err(&vfe->vdev->dev,
				"Failed to allocate stream update descriptor\n");
			goto release_lock;
		}

		updated_stream->substream = substr_info->substream;
		spin_lock(&vfe->updated_streams_lock);
		list_add_tail(&updated_stream->list, &vfe->updated_streams);
		spin_unlock(&vfe->updated_streams_lock);
	}
release_lock:
	spin_unlock_irqrestore(&vfe->substream_info_lock, irq_flags);

	queue_work(vfe->posn_update_queue,
		&vfe->posn_update_work);
}

static void vfe_not_tx_timeout_handler(struct work_struct *work)
{
	struct vfe_ipc_msg *msg;
	struct snd_skl_vfe *vfe =
		container_of(work, struct snd_skl_vfe,
		msg_timeout_work);

	while (!list_empty(&vfe->expired_msg_list)) {
		msg = list_first_entry(&vfe->expired_msg_list,
			struct vfe_ipc_msg, list);

		vfe_handle_timedout_not_tx_msg(vfe, msg);

		list_del(&msg->list);
		kfree(msg->tx_buf);
		kfree(msg->rx_buf);
		kfree(msg);
	}
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
			list_add_tail(&msg->list, &vfe->expired_msg_list);
			schedule_work(&vfe->msg_timeout_work);
			continue;
		}

		if (msg->rx_buf)
			memcpy(msg->rx_data, msg->rx_buf, msg->rx_size);

		if (msg->waitq && msg->completed) {
			*msg->completed = true;
			wake_up(msg->waitq);
		}

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
	schedule_work(&vfe->message_loop_work);
}

static void vfe_handle_posn(struct work_struct *work)
{
	struct vfe_updated_substream *updated_stream_desc;
	struct snd_pcm_substream *substream;
	unsigned long irq_flags;
	struct snd_skl_vfe *vfe =
		container_of(work, struct snd_skl_vfe, posn_update_work);

	while (!list_empty(&vfe->updated_streams)) {
		spin_lock_irqsave(&vfe->updated_streams_lock, irq_flags);
		updated_stream_desc = list_first_entry(&vfe->updated_streams,
				struct vfe_updated_substream, list);
		list_del(&updated_stream_desc->list);
		spin_unlock_irqrestore(&vfe->updated_streams_lock, irq_flags);
		substream = updated_stream_desc->substream;
		if (!mutex_is_locked(&substream->self_group.mutex))
			snd_pcm_period_elapsed(updated_stream_desc->substream);
		kfree(updated_stream_desc);
	}
}

static void vfe_handle_tplg(struct snd_skl_vfe *vfe,
	struct vfe_tplg_data *tplg_data)
{
	u8 *data_ptr;

	dev_dbg(&vfe->vdev->dev,
		"Tplg chunk received offset %d chunk size %d\n",
		tplg_data->offset, tplg_data->chunk_size);

	mutex_lock(&vfe->tplg.tplg_lock);

	if (!vfe->tplg.tplg_data.data)
		goto err_handler;

	data_ptr = (u8 *)vfe->tplg.tplg_data.data + tplg_data->offset;
	memcpy(data_ptr, tplg_data->data, tplg_data->chunk_size);
	vfe->tplg.data_ready += tplg_data->chunk_size;

	if (vfe->tplg.data_ready >= vfe->tplg.tplg_data.size) {
		vfe->tplg.load_completed = true;
		wake_up(&vfe->tplg.waitq);
	}

err_handler:
	mutex_unlock(&vfe->tplg.tplg_lock);
}

static void vfe_message_loop(struct work_struct *work)
{
	struct vfe_inbox_header *header;
	struct vfe_kctl_noti *kctln;
	struct virtqueue *vq;
	unsigned int buflen = 0;
	struct vfe_kctl_result result;

	struct snd_skl_vfe *vfe =
		container_of(work, struct snd_skl_vfe, message_loop_work);

	vq = vfe->ipc_not_rx_vq;

	while ((header = virtqueue_get_buf(vq, &buflen)) != NULL) {
		switch (header->msg_type) {
		case VFE_MSG_KCTL_SET:
			kctln = (struct vfe_kctl_noti *)header;
			kctl_ipc_handle(domain_id, &kctln->kcontrol,
				&kctln->kcontrol_value, &result);
			break;
		case VFE_MSG_TPLG_DATA:
			vfe_handle_tplg(vfe,
				(struct vfe_tplg_data *)header);
			break;
		default:
			dev_err(&vfe->vdev->dev,
				"Invalid msg Type (%d)\n", header->msg_type);
		}
		vfe_put_inbox_buffer(vfe, header);
	}
}

static int vfe_skl_kcontrol_get_domain_id(const struct snd_kcontrol *kcontrol,
		u32 *dom_id)
{
	*dom_id = domain_id;
	return 0;
}

static struct kctl_ops vfe_kctl_ops = {
		.get_domain_id = vfe_skl_kcontrol_get_domain_id,
		.send_noti = vfe_send_kctl_msg,
};

static void vfe_fill_pcm_msg_header(struct vfe_msg_header *msg_header,
	enum vfe_ipc_msg_type msg_type, struct snd_pcm_substream *substream)
{
		struct vfe_pcm_info *pcm_desc = &msg_header->desc.pcm;

		msg_header->cmd = msg_type;
		strncpy(pcm_desc->pcm_id, substream->pcm->id,
				ARRAY_SIZE(pcm_desc->pcm_id));
		pcm_desc->direction = substream->stream;
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

	 vfe_fill_pcm_msg_header(&msg_header, VFE_MSG_PCM_OPEN, substream);

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
	substr_info->pos_desc =
		kzalloc(sizeof(*substr_info->pos_desc), GFP_KERNEL);
	if (!substr_info->pos_desc) {
		kfree(substr_info);
		return -ENOMEM;
	}

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
		kfree(sstream_info->pos_desc);

		list_del(&sstream_info->list);
		kfree(sstream_info);
	}
	spin_unlock_irqrestore(&vfe->substream_info_lock, irq_flags);

	vfe_fill_pcm_msg_header(&msg_header, VFE_MSG_PCM_CLOSE, substream);

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

	vfe_fill_pcm_msg_header(&msg_header, VFE_MSG_PCM_HW_PARAMS, substream);

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
	struct vfe_pcm_result vbe_result;
	int ret;

	ret = skl_platform_pcm_trigger(substream, cmd);
	if (ret < 0)
		return ret;

	ret = vfe_is_valid_fe_substream(substream);
	if (ret)
		return 0;

	vfe_fill_pcm_msg_header(&msg_header, VFE_MSG_PCM_TRIGGER, substream);

	ret = vfe_send_msg_with_timeout(vfe, &msg_header, &cmd, sizeof(cmd),
		&vbe_result, sizeof(vbe_result), VFE_MSG_TRIGGER_TIMEOUT);

	return ret;
}

int vfe_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct vfe_msg_header msg_header;
	struct vfe_pcm_dma_conf dma_conf;
	struct vfe_pcm_result vbe_result;
	struct snd_sg_buf *sg_buf;
	int ret;
	struct snd_skl_vfe *vfe = get_virtio_audio_fe();
	struct vfe_substream_info *substr_info =
		vfe_find_substream_info(vfe, substream);

	ret = vfe_is_valid_fe_substream(substream);
	if (ret)
		return 0;

	if (!substr_info)
		return -EINVAL;

	sg_buf = snd_pcm_substream_sgbuf(substream);

	dma_conf.addr = (u64)sg_buf->table[0].addr;
	dma_conf.size = (u64)substream->runtime->dma_bytes;
	dma_conf.pages = sg_buf->pages;
	dma_conf.offset = (u64)0;

	dma_conf.stream_pos_addr = virt_to_phys(substr_info->pos_desc);
	dma_conf.stream_pos_size = sizeof(struct vfe_stream_pos_desc);

	vfe_fill_pcm_msg_header(&msg_header, VFE_MSG_PCM_PREPARE, substream);

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

	return substr_info && substr_info->pos_desc ?
		substr_info->pos_desc->hw_ptr : 0;
}

static void vfe_handle_timedout_pcm_msg(struct snd_skl_vfe *vfe,
	struct vfe_ipc_msg *msg)
{
	struct snd_pcm_substream *substream;
	const struct vfe_pcm_info *pcm_desc = &msg->header.desc.pcm;
	const struct snd_pcm *pcm =
		vfe_skl_find_pcm_by_name(&vfe->sdev,
				(char *)pcm_desc->pcm_id);
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
	INIT_LIST_HEAD(&skl->skl_sst->tplg_domains);

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

static int vfe_request_topology(struct skl *skl, const struct firmware **fw)
{
	int ret;
	struct snd_skl_vfe *vfe = get_virtio_audio_fe();

	ret = wait_event_timeout(vfe->tplg.waitq,
		vfe->tplg.load_completed,
		msecs_to_jiffies(VFE_TPLG_LOAD_TIMEOUT));

	if (ret == 0) {
		dev_err(&vfe->vdev->dev,
			"Failed to receive topology from BE service");
		return -ETIMEDOUT;
	}

	*fw = &vfe->tplg.tplg_data;

	return 0;
}

static int vfe_init_tplg(struct snd_skl_vfe *vfe, struct skl *skl)
{
	struct vfe_msg_header msg_header;
	struct vfe_tplg_info tplg_info;
	int ret;
	u8 *tplg_data;


	mutex_init(&vfe->tplg.tplg_lock);
	init_waitqueue_head(&vfe->tplg.waitq);
	vfe->tplg.load_completed = false;

	skl->skl_sst->request_tplg = vfe_request_topology;

	mutex_lock(&vfe->tplg.tplg_lock);
	msg_header.cmd = VFE_MSG_TPLG_INFO;
	ret = vfe_send_msg(vfe, &msg_header,
		NULL, 0, &tplg_info, sizeof(tplg_info));
	if (ret < 0)
		goto error_handl;

	//TODO: get result from vBE

	tplg_data = devm_kzalloc(&vfe->vdev->dev,
		tplg_info.size, GFP_KERNEL);
	if (!tplg_data) {
		ret = -ENOMEM;
		goto error_handl;
	}

	domain_id = tplg_info.domain_id;
	vfe->tplg.tplg_data.data = tplg_data;
	vfe->tplg.tplg_data.size = tplg_info.size;
	strncpy(skl->tplg_name, tplg_info.tplg_name,
		ARRAY_SIZE(skl->tplg_name));

error_handl:
	mutex_unlock(&vfe->tplg.tplg_lock);

	return ret;
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

	err = vfe_skl_init_dsp(skl);
	if (err < 0)
		goto error;

	err = vfe_init_tplg(vfe, skl);
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

static void vfe_send_queues(struct virtio_device *vdev)
{
	int idx;
	struct snd_skl_vfe *vfe = vdev->priv;

	for (idx = 0; idx < VFE_MSG_BUFF_NUM; ++idx) {
		vfe->in_buff[idx] = devm_kmalloc(&vdev->dev,
				sizeof(union inbox_msg), GFP_KERNEL);
		if (!vfe->in_buff[idx])
			return;

		vfe_put_inbox_buffer(vfe, vfe->in_buff[idx]);
	}
	vfe_vq_kick(vfe, vfe->ipc_not_rx_vq);
}

static int vfe_init(struct virtio_device *vdev)
{
	struct snd_skl_vfe *vfe;
	int ret;

	vfe = devm_kzalloc(&vdev->dev, sizeof(*vfe), GFP_KERNEL);
	if (!vfe) {
		ret = -ENOMEM;
		goto err;
	}

	skl_vfe = vfe;
	vfe->vdev = vdev;
	vdev->priv = vfe;

	INIT_LIST_HEAD(&vfe->kcontrols_list);
	spin_lock_init(&vfe->substream_info_lock);
	INIT_LIST_HEAD(&vfe->substr_info_list);
	spin_lock_init(&vfe->ipc_vq_lock);
	INIT_LIST_HEAD(&vfe->expired_msg_list);
	spin_lock_init(&vfe->updated_streams_lock);
	INIT_LIST_HEAD(&vfe->updated_streams);

	INIT_WORK(&vfe->posn_update_work, vfe_handle_posn);
	INIT_WORK(&vfe->msg_timeout_work, vfe_not_tx_timeout_handler);
	INIT_WORK(&vfe->message_loop_work, vfe_message_loop);

	vfe->posn_update_queue =  alloc_workqueue("%s",
		WQ_HIGHPRI | WQ_UNBOUND, 1, "posn_update_queue");

	ret = vfe_init_vqs(vfe);
	if (ret < 0)
		goto err;

	kctl_init_proxy(&vdev->dev, &vfe_kctl_ops);

	vfe->send_dsp_ipc_msg = vfe_send_dsp_ipc_msg;

	vfe_send_queues(vdev);

	vfe_init_irq_queue(vfe);

	ret = vfe_skl_init(vdev);
	if (ret < 0)
		goto skl_err;

	return 0;

skl_err:
	virtqueue_disable_cb(vfe->ipc_not_tx_vq);
	virtqueue_disable_cb(vfe->ipc_not_rx_vq);
	cancel_work_sync(&vfe->msg_timeout_work);
	cancel_work_sync(&vfe->message_loop_work);
	vdev->config->reset(vdev);
	vdev->config->del_vqs(vdev);
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

	cancel_work_sync(&vfe->message_loop_work);
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

	vfe_send_queues(vdev);

	vfe_init_irq_queue(vfe);

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
module_param(domain_name, charp, 0444);

MODULE_DEVICE_TABLE(virtio, id_table);
MODULE_DESCRIPTION("Intel Broxton Virtio FE Driver");
MODULE_LICENSE("GPL v2");
