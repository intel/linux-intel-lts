// SPDX-License-Identifier: GPL-2.0+
//
// skl-virtio-be.c  --  Virtio BE service for SKL architecture
//
// Copyright (C) 2018 Intel Corporation.
//
// Authors: Furtak, Pawel <pawel.furtak@intel.com>
//          Janca, Grzegorz <grzegorz.janca@intel.com>
//
//  BE receives commands from FE drivers and forward them to appropriate
//  entity, such as DSP, PCM or sound card control. BE sends buffer position
//  updates to FE driver.

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/hw_random.h>
#include <linux/uio.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/device.h>
#include <sound/pcm_params.h>
#include <linux/vbs/vq.h>
#include <linux/vbs/vbs.h>
#include <linux/vhm/acrn_common.h>
#include <linux/vhm/acrn_vhm_ioreq.h>
#include <linux/vhm/acrn_vhm_mm.h>
#include <linux/vhm/vhm_vm_mngt.h>
#include <linux/spinlock.h>
#include "skl-virtio-be.h"
#include "../skl.h"
#include "../skl-sst-ipc.h"
#include "../skl-topology.h"
#include "skl-virtio.h"

static struct snd_skl_vbe *get_virtio_audio_be(void)
{
	return &get_virtio_audio()->vbe;
}

struct kctl_proxy *get_kctl_proxy(void)
{
	return &get_virtio_audio_be()->kcon_proxy;
}

const struct vbe_substream_info *vbe_find_substream_info_by_pcm(
	const struct snd_skl_vbe *vbe, char *pcm_id, int direction)
{
	const struct vbe_substream_info *info;

	list_for_each_entry(info, &vbe->substr_info_list, list) {
		if (info->direction == direction &&
			strncmp(info->pcm->id, pcm_id,
					ARRAY_SIZE(info->pcm->id)) == 0)
			return info;
	}
	return NULL;
}

inline const struct vbe_substream_info *vbe_find_substream_info(
	const struct snd_skl_vbe *vbe, const struct snd_pcm_substream *substr)
{
	return vbe_find_substream_info_by_pcm(vbe, substr->pcm->id,
			substr->stream);
}

static const struct vbe_substream_info *vbe_skl_find_substream_info(
	const struct skl *sdev, const struct snd_pcm_substream *substr)
{
	const struct snd_skl_vbe *vbe = skl_get_vbe(sdev);

	return vbe_find_substream_info(vbe, substr);
}

struct snd_soc_dapm_widget *vbe_skl_find_kcontrol_widget(
	const struct skl *sdev,	const struct snd_kcontrol *kcontrol)
{
	struct snd_soc_dapm_widget *w;
	int i;

	list_for_each_entry(w, &sdev->component->card->widgets, list) {
		for (i = 0; i < w->num_kcontrols; i++) {
			if (kcontrol == w->kcontrols[i])
				return w;
		}
	}

	return NULL;
}

inline int vbe_skl_is_valid_pcm_id(char *pcm_id)
{
	if (pcm_id == NULL || strlen(pcm_id) == 0 ||
		strcmp(pcm_id, "((null))") == 0)
		return -EINVAL;

	return 0;
}

static const struct snd_soc_pcm_runtime *
vbe_skl_find_rtd_by_pcm_id(
	const struct skl *skl, char *pcm_name)
{
	const struct snd_soc_pcm_runtime *rtd;
	int ret = vbe_skl_is_valid_pcm_id(pcm_name);

	if (ret < 0)
		return NULL;

	if (unlikely(!skl || !skl->component || !skl->component->card))
		return NULL;

	list_for_each_entry(rtd, &skl->component->card->rtd_list, list) {
		if (strncmp(rtd->pcm->id, pcm_name,
				ARRAY_SIZE(rtd->pcm->id)) == 0)
			return rtd;
	}
	return NULL;
}

const struct snd_pcm *vbe_skl_find_pcm_by_name(struct skl *skl, char *pcm_name)
{
	const struct snd_soc_pcm_runtime *rtd;

	if (!strlen(pcm_name))
		return NULL;

	rtd = vbe_skl_find_rtd_by_pcm_id(skl, pcm_name);

	return rtd ? rtd->pcm : NULL;
}

static bool vbe_skl_try_send(const struct snd_skl_vbe *vbe,
		const struct virtio_vq_info *vq, void *buff,
		unsigned int size)
{
	const struct iovec iov;
	struct vfe_inbox_buff *save_buff;
	u16 idx;

	if (virtio_vq_has_descs(vq) &&
		(virtio_vq_getchain(vq, &idx, &iov, 1, NULL) > 0)) {
		if (iov.iov_len < size) {
			dev_err(vbe->dev, "iov len %lu, expecting len %lu\n",
				iov.iov_len, size);
			virtio_vq_relchain(vq, idx, iov.iov_len);
		}
		memcpy(iov.iov_base, buff, size);
		virtio_vq_relchain(vq, idx, iov.iov_len);
		virtio_vq_endchains(vq, true);
		return true;
	}
	return false;
}


static void vbe_skl_send_or_enqueue(const struct snd_skl_vbe *vbe,
		const struct virtio_vq_info *vq,
		struct vfe_pending_msg *pen_msg)
{
	struct vfe_pending_msg *save_msg;

	if (vbe_skl_try_send(vbe, vq,
		(void *)&pen_msg->msg, pen_msg->sizeof_msg) == false) {
		save_msg = kzalloc(sizeof(*save_msg), GFP_KERNEL);
		if (!save_msg) {
			dev_err(vbe->dev, "Failed to allocate kctl_req memory");
			return;
		}
		*save_msg = *pen_msg;
		list_add_tail(&save_msg->list, &vbe->pending_msg_list);
	}
}

void vbe_stream_update(struct hdac_bus *bus, struct hdac_stream *hstr)
{
	struct skl *skl = bus_to_skl(bus);
	struct snd_skl_vbe *vbe = skl_get_vbe(skl);

	if (hstr->substream)
		skl_notify_stream_update(bus, hstr->substream);

	vbe->nops.hda_irq_ack(bus, hstr);
}

int vbe_send_kctl_msg(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol,
		struct vfe_kctl_result *result)
{
	struct vfe_pending_msg kctl_msg;
	const struct snd_skl_vbe *vbe = &get_virtio_audio()->vbe;
	const struct virtio_vq_info *vq = &vbe->vqs[SKL_VIRTIO_IPC_NOT_RX_VQ];
	bool endchain;

	kctl_msg.msg.posn.msg_type = VFE_MSG_KCTL_SET;
	strncpy(kctl_msg.msg.kctln.kcontrol.kcontrol_id, kcontrol->id.name,
			ARRAY_SIZE(kcontrol->id.name));

	kctl_msg.msg.kctln.kcontrol_value.value = *ucontrol;

	kctl_msg.sizeof_msg = sizeof(struct vfe_kctl_noti);

	vbe_skl_send_or_enqueue(vbe, vq, &kctl_msg);

	result->ret = 0;
	return 0;
}

void skl_notify_stream_update(struct hdac_bus *bus,
		struct snd_pcm_substream *substream)
{
	const struct skl *skl = bus_to_skl(bus);
	const struct vbe_substream_info *substr_info;
	const struct snd_soc_pcm_runtime *rtd;
	struct vfe_pending_msg pos_req;
	const struct snd_skl_vbe *vbe;
	const struct virtio_vq_info *vq;

	substr_info = vbe_skl_find_substream_info(skl, substream);
	if (!substr_info)
		return;

	rtd = substream->private_data;

	pos_req.msg.posn.msg_type = VFE_MSG_POS_NOTI;
	pos_req.msg.posn.stream_dir = substr_info->direction;
	pos_req.msg.posn.stream_pos = rtd->ops.pointer(substream);
	strncpy(pos_req.msg.posn.pcm_id, substream->pcm->id,
		ARRAY_SIZE(substream->pcm->id));

	pos_req.sizeof_msg = sizeof(struct vfe_hw_pos_request);

	vbe = substr_info->vbe;
	vq = &vbe->vqs[SKL_VIRTIO_IPC_NOT_RX_VQ];

	vbe_skl_send_or_enqueue(vbe, vq, &pos_req);
}

int vbe_skl_allocate_runtime(const struct snd_soc_card *card,
		struct snd_pcm_substream *substream)
{
	const struct snd_soc_pcm_runtime *rtd;
	struct snd_pcm_runtime *runtime;
	int size;

	runtime = kzalloc(sizeof(*runtime), GFP_KERNEL);
	if (!runtime)
		return -ENOMEM;

	size = PAGE_ALIGN(sizeof(struct snd_pcm_mmap_status));
	runtime->status = snd_malloc_pages(size, GFP_KERNEL);
	if (!runtime->status)
		goto alloc_free;

	memset((void *)runtime->status, 0, size);

	size = PAGE_ALIGN(sizeof(struct snd_pcm_mmap_control));
	runtime->control = snd_malloc_pages(size, GFP_KERNEL);
	if (!runtime->control) {
		snd_free_pages((void *)runtime->status,
			PAGE_ALIGN(sizeof(struct snd_pcm_mmap_status)));
		goto alloc_free;
	}
	memset((void *)runtime->control, 0, size);

	init_waitqueue_head(&runtime->sleep);
	init_waitqueue_head(&runtime->tsleep);
	runtime->status->state = SNDRV_PCM_STATE_OPEN;

	substream->runtime = runtime;

	list_for_each_entry(rtd, &card->rtd_list, list) {
		if (strcmp(rtd->pcm->id, substream->pcm->id) == 0) {
			substream->private_data = rtd;
			break;
		}
	}
	return 0;
alloc_free:
	kfree(runtime);
	return -ENOMEM;
}

void vbe_skl_initialize_substream_runtime(struct snd_pcm_runtime *runtime,
		struct snd_pcm_hw_params *params)
{
	int bits;
	int frames;

	runtime->access = params_access(params);
	runtime->format = params_format(params);
	runtime->subformat = params_subformat(params);
	runtime->channels = params_channels(params);
	runtime->rate = params_rate(params);
	runtime->period_size = params_period_size(params);
	runtime->periods = params_periods(params);
	runtime->buffer_size = params_buffer_size(params);
	runtime->info = params->info;
	runtime->rate_num = params->rate_num;
	runtime->rate_den = params->rate_den;
	runtime->no_period_wakeup =
		(params->info & SNDRV_PCM_INFO_NO_PERIOD_WAKEUP) &&
		(params->flags & SNDRV_PCM_HW_PARAMS_NO_PERIOD_WAKEUP);
	runtime->no_rewinds =
		(params->flags & SNDRV_PCM_HW_PARAMS_NO_REWINDS) ? 1 : 0;
	bits = snd_pcm_format_physical_width(runtime->format);
	runtime->sample_bits = bits;
	bits *= runtime->channels;
	runtime->frame_bits = bits;
	frames = 1;
	while (bits % 8 != 0) {
		bits *= 2;
		frames *= 2;
	}
	runtime->byte_align = bits / 8;
	runtime->min_align = frames;
	/* Default sw params */
	runtime->tstamp_mode = SNDRV_PCM_TSTAMP_NONE;
	runtime->period_step = 1;
	runtime->control->avail_min = runtime->period_size;
	runtime->start_threshold = 1;
	runtime->stop_threshold = runtime->buffer_size;
	runtime->silence_threshold = 0;
	runtime->silence_size = 0;
	runtime->boundary = runtime->buffer_size << 4;
}

static int vbe_skl_prepare_dma(const struct snd_pcm_substream *substream,
	int vm_id, const struct vfe_pcm_dma_conf *dma_conf)
{
	const struct snd_sg_buf *sg_buf;
	int cnt;
	u64 pcm_buffer_gpa = dma_conf->addr & ~(u64)0xfff;
	u64 pcm_buffer_hpa = vhm_vm_gpa2hpa(vm_id, pcm_buffer_gpa);

	if (!pcm_buffer_hpa)
		return -EINVAL;

	sg_buf = snd_pcm_substream_sgbuf(substream);
	if (!sg_buf)
		return -EINVAL;

	sg_buf->table[0].addr = pcm_buffer_hpa | 0x10;
	for (cnt = 1; cnt < sg_buf->pages; cnt++) {
		pcm_buffer_hpa += PAGE_SIZE;
		sg_buf->table[cnt].addr = pcm_buffer_hpa;
	}

	return 0;
}

static int vbe_skl_assemble_params(struct vfe_pcm_hw_params *vfe_params,
		const struct snd_pcm_hw_params *params)
{
	hw_param_interval(params, SNDRV_PCM_HW_PARAM_ACCESS)->min =
		vfe_params->access;

	hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS)->min =
		vfe_params->channels;

	hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE)->min =
		vfe_params->rate;

	hw_param_interval(params, SNDRV_PCM_HW_PARAM_PERIOD_BYTES)->min =
		vfe_params->host_period_bytes;

	hw_param_interval(params, SNDRV_PCM_HW_PARAM_BUFFER_SIZE)->min =
		vfe_params->buffer_size;

	hw_param_interval(params, SNDRV_PCM_HW_PARAM_BUFFER_BYTES)->min =
		vfe_params->buffer_bytes;

	hw_param_interval(params, SNDRV_PCM_HW_PARAM_PERIOD_SIZE)->min =
		vfe_params->period_size;

	hw_param_interval(params, SNDRV_PCM_HW_PARAM_PERIODS)->min =
		vfe_params->periods;

	params_set_format(params, vfe_params->frame_fmt);

	return 0;
}

static int vbe_skl_add_substream_info(struct snd_skl_vbe *vbe,
		const struct snd_pcm_substream *substream)
{
	struct vbe_substream_info *substr_info =
		kzalloc(sizeof(*substr_info), GFP_KERNEL);

	if (!substr_info)
		return -ENOMEM;

	substr_info->pcm = substream->pcm;
	substr_info->substream = substream;
	substr_info->direction = substream->stream;
	substr_info->vbe = vbe;

	list_add(&substr_info->list, &vbe->substr_info_list);
	return 0;
}

static int vbe_skl_pcm_get_domain_id(const struct skl *sdev,
	const char *pcm_id, int direction, int *domain_id)
{
	const struct snd_soc_pcm_runtime *rtd;
	struct skl_module_cfg *mconfig = NULL;

	if (unlikely(!domain_id))
		return -EINVAL;

	rtd = vbe_skl_find_rtd_by_pcm_id(sdev, pcm_id);
	if (!rtd)
		return -ENODEV;

	if (rtd->cpu_dai)
		mconfig = skl_tplg_fe_get_cpr_module(rtd->cpu_dai, direction);

	if (mconfig) {
		*domain_id = mconfig->domain_id;
		return 0;
	}

	return -EINVAL;
}

static int vbe_skl_pcm_check_permission(const struct skl *sdev,
	int domain_id, const char *pcm_id, int direction)
{
	int pcm_domain_id;
	int ret = 0;

	ret = vbe_skl_pcm_get_domain_id(sdev, pcm_id,
			direction, &pcm_domain_id);
	if (ret < 0)
		return ret;

	if (domain_id != pcm_domain_id)
		return -EACCES;

	return ret;
}

static int vbe_skl_pcm_open(const struct snd_skl_vbe *vbe,
		const struct skl *sdev,
		int vm_id, const struct vbe_ipc_msg *msg)
{
	const struct snd_soc_pcm_runtime *rtd;
	struct snd_pcm_substream *substream;
	const struct snd_pcm_runtime *runtime;
	int ret;
	struct vfe_pcm_result *vbe_result = msg->rx_data;
	const struct vfe_pcm_info *pcm_desc = &msg->header->desc.pcm;
	const struct snd_pcm *pcm =
		vbe_skl_find_pcm_by_name(vbe->sdev, pcm_desc->pcm_id);
	int direction = pcm_desc->direction;

	if (!pcm) {
		dev_err(&sdev->pci->dev, "Can not find PCM [%s].\n",
			pcm_desc->pcm_id);
		ret = -ENODEV;
		goto ret_err;
	}

	ret = vbe_skl_pcm_check_permission(sdev,
		msg->header->domain_id, pcm_desc->pcm_id, direction);
	if (ret < 0)
		goto ret_err;

	substream = pcm->streams[direction].substream;
	runtime = substream->runtime;

	if (substream->ref_count > 0) {
		ret = -EBUSY;
		goto ret_err;
	}

	ret = vbe_skl_allocate_runtime(sdev->component->card, substream);
	if (ret < 0)
		goto ret_err;
	ret = vbe_skl_add_substream_info(vbe, substream);
	if (ret < 0)
		goto ret_err;
	substream->ref_count++;  /* set it used */
	rtd = substream->private_data;
	ret = rtd->ops.open(substream);

ret_err:
	if (vbe_result)
		vbe_result->ret = ret;

	return ret;
}

static int vbe_skl_pcm_close(const struct skl *sdev, int vm_id,
		const struct vbe_substream_info *substr_info,
		const struct vbe_ipc_msg *msg)
{
	struct snd_soc_pcm_runtime *rtd;
	int ret;
	struct snd_pcm_substream *substream = substr_info->substream;
	struct vfe_pcm_result *vbe_result = msg->rx_data;

	list_del(&substr_info->list);
	kfree(substr_info);

	substream->ref_count = 0;
	rtd = substream->private_data;
	ret = rtd->ops.close(substream);

	if (vbe_result)
		vbe_result->ret = ret;

	return ret;
}

static int vbe_skl_pcm_prepare(const struct skl *sdev, int vm_id,
		const struct vbe_substream_info *substr_info,
		const struct vbe_ipc_msg *msg)
{
	const struct snd_soc_pcm_runtime *rtd;
	int ret;
	const struct snd_pcm_substream *substream = substr_info->substream;
	const struct vfe_pcm_dma_conf *dma_params = msg->tx_data;
	struct vfe_pcm_result *vbe_result = msg->rx_data;

	ret = vbe_skl_prepare_dma(substream, vm_id, dma_params);
	if (ret < 0)
		return ret;

	rtd = substream->private_data;
	ret = rtd->ops.prepare(substream);

	if (vbe_result)
		vbe_result->ret = ret;

	return ret;
}

struct snd_pcm_hw_params hw_params;

static int vbe_skl_pcm_hw_params(const struct skl *sdev, int vm_id,
		const struct vbe_substream_info *substr_info,
		const struct vbe_ipc_msg *msg)
{
	const struct snd_soc_pcm_runtime *rtd;
	int ret;
	const struct snd_pcm_substream *substream = substr_info->substream;
	//TODO: check if tx and rx data have expected size
	const struct vfe_pcm_hw_params *hw_params_ipc = msg->tx_data;
	struct vfe_pcm_result *vbe_result = msg->rx_data;

	vbe_skl_assemble_params(hw_params_ipc, &hw_params);
	vbe_skl_initialize_substream_runtime(substream->runtime, &hw_params);

	rtd = substream->private_data;
	ret = rtd->ops.hw_params(substream, &hw_params);

	if (vbe_result)
		vbe_result->ret = ret;

	return ret;
}

static int vbe_skl_send_tplg_data(struct snd_skl_vbe *vbe,
	const struct skl *sdev, const struct firmware *tplg,
	int vm_id)
{
	struct vfe_pending_msg tplg_msg;
	struct vfe_tplg_data *tplg_data = &tplg_msg.msg.tplg_data;
	int rem_data = tplg->size, offset;
	u8 *data_ptr = tplg->data;
	const struct virtio_vq_info *vq = &vbe->vqs[SKL_VIRTIO_IPC_NOT_RX_VQ];

	tplg_msg.sizeof_msg = sizeof(struct vfe_tplg_data);
	tplg_data->msg_type = VFE_MSG_TPLG_DATA;

	u32 chunk_length;
	u8 data[SKL_VIRTIO_TPLG_CHUNK_SIZE];

	for (offset = 0; offset < tplg->size;
		offset += SKL_VIRTIO_TPLG_CHUNK_SIZE,
		rem_data -= SKL_VIRTIO_TPLG_CHUNK_SIZE) {

		tplg_data->offset = offset;
		tplg_data->chunk_size = rem_data > SKL_VIRTIO_TPLG_CHUNK_SIZE ?
			SKL_VIRTIO_TPLG_CHUNK_SIZE : rem_data;
		memcpy(tplg_data->data, data_ptr, tplg_data->chunk_size);
		data_ptr += tplg_data->chunk_size;

		vbe_skl_send_or_enqueue(vbe, vq, &tplg_msg);
	}

	return 0;
}

static int vbe_skl_tplg_size(struct snd_skl_vbe *vbe, const struct skl *sdev,
	int vm_id, const struct vbe_ipc_msg *msg)
{
	const struct firmware *tplg;
	char *tplg_name;
	int chunks, data, ret;
	struct vfe_tplg_size *tplg_size = msg->rx_data;

	if (!tplg_size)
		return -EINVAL;

	//TODO: get tplg file name by guest domain ID
	tplg_name = "guest_tplg.bin";
	ret = request_firmware(&tplg, tplg_name, vbe->dev);
	if (ret < 0)
		return ret;

	tplg_size->chunk_size = SKL_VIRTIO_TPLG_CHUNK_SIZE;
	tplg_size->size = tplg->size;
	tplg_size->chunks = tplg_size->size / SKL_VIRTIO_TPLG_CHUNK_SIZE +
		tplg_size->size % SKL_VIRTIO_TPLG_CHUNK_SIZE ? 1 : 0;

	vbe_skl_send_tplg_data(vbe, sdev, tplg, vm_id);

	release_firmware(tplg);

	return 0;
}

static int vbe_skl_pcm_trigger(struct skl *sdev, int vm_id,
		const struct vbe_substream_info *substr_info,
		const struct vbe_ipc_msg *msg)
{
	const struct snd_soc_pcm_runtime *rtd;
	const struct snd_pcm_substream *substream = substr_info->substream;
	int cmd = *(int *)msg->tx_data;

	rtd = substream->private_data;
	return rtd->ops.trigger(substream, cmd);
}

static u32 vbe_skl_kcontrol_find_domain_id(const struct snd_kcontrol *kcontrol,
	struct skl_module_cfg *mconfig)
{
	struct skl_kctl_domain *domain;
	bool name_match = false;

	list_for_each_entry(domain, &mconfig->kctl_domains, list) {
		name_match = strncmp(domain->name, kcontrol->id.name,
			ARRAY_SIZE(domain->name)) == 0;
		if (name_match)
			return domain->domain_id;
	}

	return 0;
}

static int vbe_skl_kcontrol_get_domain_id(const struct snd_kcontrol *kcontrol,
		u32 *domain_id)
{
	struct skl_module_cfg *mconfig;
	struct snd_soc_dapm_widget *w;
	void *priv = kcontrol->private_data;
	int ret = 0;
	struct skl *sdev = get_virtio_audio()->skl;

	if (sdev == NULL)
		return -EINVAL;

	if (unlikely(!domain_id))
		return -EINVAL;

	*domain_id = 0;

	if (priv == sdev->component ||
		priv == sdev->component->card)
		return 0;


	w = vbe_skl_find_kcontrol_widget(sdev, kcontrol);
	if (w) {
		mconfig = w->priv;
		*domain_id = vbe_skl_kcontrol_find_domain_id(kcontrol, mconfig);
	}

	return 0;
}

static struct kctl_ops vbe_kctl_ops = {
		.get_domain_id = vbe_skl_kcontrol_get_domain_id,
		.send_noti = vbe_send_kctl_msg,
};

static int vbe_skl_cfg_hda(const struct skl *sdev, int vm_id,
		const struct vbe_ipc_msg *msg)
{
	const struct hdac_bus *bus = &sdev->hbus;
	struct vfe_hda_cfg *hda_cfg = msg->rx_data;
	unsigned short gcap;

	if (!hda_cfg || msg->rx_size != sizeof(*hda_cfg))
		return -EINVAL;

	hda_cfg->resource_length = pci_resource_len(sdev->pci, 0);
	gcap = snd_hdac_chip_readw(bus, GCAP);

	hda_cfg->cp_streams = (gcap >> 8) & 0x0f;
	hda_cfg->pb_streams = (gcap >> 12) & 0x0f;

	hda_cfg->ppcap = bus->ppcap ? bus->ppcap - bus->remap_addr : 0;
	hda_cfg->spbcap = bus->spbcap ? bus->spbcap - bus->remap_addr : 0;
	hda_cfg->mlcap = bus->mlcap ? bus->mlcap - bus->remap_addr : 0;
	hda_cfg->gtscap = bus->gtscap ? bus->gtscap - bus->remap_addr : 0;
	hda_cfg->drsmcap = bus->drsmcap ? bus->drsmcap - bus->remap_addr : 0;

	return 0;
}

static int vbe_skl_msg_cfg_handle(struct snd_skl_vbe *vbe,
		const struct skl *sdev,
		int vm_id, struct vbe_ipc_msg *msg)
{
	struct kctl_ops kt_ops;

	switch (msg->header->cmd) {
	case VFE_MSG_CFG_HDA:
		kctl_init_proxy(vbe->dev, &vbe_kctl_ops);
		kctl_notify_machine_ready(sdev->component->card);
		return vbe_skl_cfg_hda(sdev, vm_id, msg);
	default:
		dev_err(vbe->dev, "Unknown command %d for config get message.\n",
				msg->header->cmd);
		break;
	}

	return 0;
}

int vbe_skl_msg_tplg_handle(const struct snd_skl_vbe *vbe,
		const struct skl *sdev, int vm_id, struct vbe_ipc_msg *msg)
{
	u32 domain_id = msg->header->domain_id;

	switch (msg->header->cmd) {
	case VFE_MSG_TPLG_SIZE:
		return vbe_skl_tplg_size(vbe, sdev, vm_id, msg);
	default:
		dev_err(vbe->dev, "Unknown command %d for tplg [%s].\n",
			msg->header->cmd);
	break;
	}

	return 0;
}

static int vbe_skl_msg_pcm_handle(const struct snd_skl_vbe *vbe,
		const struct skl *sdev, int vm_id, struct vbe_ipc_msg *msg)
{
	const struct vbe_substream_info *substream_info;
	char *pcm_id;
	int direction;

	if (msg->header->cmd == VFE_MSG_PCM_OPEN)
		return vbe_skl_pcm_open(vbe, sdev, vm_id, msg);

	pcm_id = msg->header->desc.pcm.pcm_id;
	direction = msg->header->desc.pcm.direction;
	substream_info = vbe_find_substream_info_by_pcm(vbe, pcm_id, direction);

	if (!substream_info) {
		dev_err(vbe->dev,
			"Can not find active substream [%s].\n", pcm_id);
		return -ENODEV;
	}

	switch (msg->header->cmd) {
	case VFE_MSG_PCM_CLOSE:
		return vbe_skl_pcm_close(sdev, vm_id, substream_info, msg);
	case VFE_MSG_PCM_PREPARE:
		return vbe_skl_pcm_prepare(sdev, vm_id, substream_info, msg);
	case VFE_MSG_PCM_HW_PARAMS:
		return vbe_skl_pcm_hw_params(sdev, vm_id, substream_info, msg);
	case VFE_MSG_PCM_TRIGGER:
		return vbe_skl_pcm_trigger(sdev, vm_id, substream_info, msg);
	default:
		dev_err(vbe->dev, "PCM stream notification %d not supported\n",
			msg->header->cmd);
	}

	return 0;
}

int vbe_skl_msg_kcontrol_handle(const struct snd_skl_vbe *vbe,
		int vm_id, const struct vbe_ipc_msg *msg)
{
	const struct vfe_kctl_info *kctl_desc = &msg->header->desc.kcontrol;
	u32 domain_id = msg->header->domain_id;

	switch (msg->header->cmd) {
	case VFE_MSG_KCTL_SET:
		return kctl_ipc_handle(domain_id, kctl_desc,
				msg->tx_data, msg->rx_data);
	default:
		dev_err(vbe->dev, "Unknown command %d for kcontrol [%s].\n",
			msg->header->cmd, kctl_desc->kcontrol_id);
	break;
	}

	return 0;
}

static int vbe_skl_not_fwd(const struct snd_skl_vbe *vbe,
	const struct skl *sdev, int vm_id, void *ipc_bufs[SKL_VIRTIO_NOT_VQ_SZ],
	size_t ipc_lens[SKL_VIRTIO_NOT_VQ_SZ])
{
	struct vbe_ipc_msg msg;

	if (sizeof(struct vfe_msg_header) != ipc_lens[SKL_VIRTIO_MSG_HEADER]) {
		dev_err(vbe->dev, "Mismatch of IPC header size");
		return -EINVAL;
	}

	msg.header = ipc_bufs[SKL_VIRTIO_MSG_HEADER];
	msg.tx_data = ipc_bufs[SKL_VIRTIO_MSG_TX];
	msg.rx_data = ipc_bufs[SKL_VIRTIO_MSG_RX];

	msg.tx_size = ipc_lens[SKL_VIRTIO_MSG_TX];
	msg.rx_size = ipc_lens[SKL_VIRTIO_MSG_RX];

	switch (msg.header->cmd & VFE_MSG_TYPE_MASK) {
	case VFE_MSG_PCM:
		return vbe_skl_msg_pcm_handle(vbe, sdev, vm_id, &msg);
	case VFE_MSG_KCTL:
		return vbe_skl_msg_kcontrol_handle(vbe, vm_id, &msg);
	case VFE_MSG_TPLG:
		return vbe_skl_msg_tplg_handle(vbe, sdev, vm_id, &msg);
		break;
	case VFE_MSG_CFG:
		return vbe_skl_msg_cfg_handle(vbe, sdev, vm_id, &msg);
	}

	return 0;
}

static int vbe_skl_ipc_fwd(const struct snd_skl_vbe *vbe,
		const struct skl *sdev, int vm_id,
		void *ipc_buf, void *reply_buf, size_t count, size_t *reply_sz)
{
	struct vfe_dsp_ipc_msg *ipc_data = ipc_buf;
	struct skl_sst *skl_sst = sdev->skl_sst;
	int ret;

	dev_dbg(vbe->dev, "IPC forward request. Header:0X%016llX tx_data:%p\n",
			ipc_data->header,
			ipc_data->data_size ? &ipc_data->data : NULL);
	dev_dbg(vbe->dev, "tx_size:%zu rx_data:%p rx_size:%zu\n",
			ipc_data->data_size,
			*reply_sz ? reply_buf : NULL,
			*reply_sz);

	/* Tx IPC and wait for response */
	ret = *reply_sz <= 0 ? 0 : sst_ipc_tx_message_wait(&skl_sst->ipc,
			ipc_data->header,
			ipc_data->data_size ? &ipc_data->data : NULL,
			ipc_data->data_size,
			*reply_sz ? reply_buf : NULL,
			reply_sz);

	if (ret < 0) {
		dev_dbg(vbe->dev, "IPC reply error:%d\n", ret);
		return ret;
	}
	if (*reply_sz > 0) {
		print_hex_dump(KERN_DEBUG, "IPC response:", DUMP_PREFIX_OFFSET,
			8, 4, (char *)reply_buf, *reply_sz, false);
	}

	return 0;
}

static int vbe_skl_virtio_vq_handle(const struct snd_skl_vbe *vbe,
	const struct virtio_vq_info *vq, u16 *idx, const struct iovec *iov,
	void *reply_buf[], size_t *reply_len, int vq_id, int vq_size)
{
	int i;
	struct device *dev = vbe->sdev->skl_sst->dev;
	int ret = virtio_vq_getchain(vq, idx, iov, vq_size, NULL);

	if (ret != vq_size) {
		dev_err(dev, "notification buffers not paired, expected:%d, got:%d",
			vq_size, ret);
		if (ret < 0) {
			virtio_vq_endchains(vq, true);
			return ret;
		}
		for (i = 0; i <= ret; i++)
			virtio_vq_relchain(vq, *idx + i, iov[i].iov_len);

		virtio_vq_endchains(vq, true);
		return ret;
	}
	for (i = 0; i < ret; i++) {
		reply_len[i] = iov[vq_id+i].iov_len;
		reply_buf[i] = iov[vq_id+i].iov_base;
	}
	return 0;
}

static void vbe_skl_ipc_fe_not_get(const struct snd_skl_vbe *vbe, int vq_idx)
{
	int ret;
	u16 idx;
	const struct iovec iov[SKL_VIRTIO_NOT_VQ_SZ];
	void *reply_buf[SKL_VIRTIO_NOT_VQ_SZ];
	size_t reply_len[SKL_VIRTIO_NOT_VQ_SZ];
	const struct virtio_vq_info *vq = &vbe->vqs[vq_idx];
	const struct device *dev = vbe->sdev->skl_sst->dev;
	int vm_id = vbe->vmid;

	memset(iov, 0, sizeof(iov));

	/* while there are mesages in virtio queue */
	while (virtio_vq_has_descs(vq)) {
		ret = vbe_skl_virtio_vq_handle(vbe, vq, &idx, iov,
			reply_buf, reply_len,
			SKL_VIRTIO_IPC_MSG, SKL_VIRTIO_NOT_VQ_SZ);
		if (ret) {
			dev_err(dev, "Failed to handle virtio message");
			return;
		}

		ret = vbe_skl_not_fwd(vbe, vbe->sdev, vm_id,
				reply_buf, reply_len);
		if (ret < 0)
			dev_err(dev, "submit guest ipc command fail\n");

		virtio_vq_relchain(vq, idx + SKL_VIRTIO_MSG_HEADER,
			reply_len[SKL_VIRTIO_MSG_HEADER]);
	}
	virtio_vq_endchains(vq, true);
}

static void vbe_skl_ipc_fe_cmd_get(const struct snd_skl_vbe *vbe, int vq_idx)
{
	u16 idx;
	int ret;
	const struct iovec iov[SKL_VIRTIO_IPC_VQ_SZ];
	void *reply_buf[SKL_VIRTIO_IPC_VQ_SZ];
	size_t reply_len[SKL_VIRTIO_IPC_VQ_SZ];
	const struct virtio_vq_info *vq = &vbe->vqs[vq_idx];
	const struct device *dev = vbe->sdev->skl_sst->dev;
	int vm_id = vbe->vmid;

	memset(iov, 0, sizeof(iov));

	/* while there are mesages in virtio queue */
	while (virtio_vq_has_descs(vq)) {
		ret = vbe_skl_virtio_vq_handle(vbe, vq, &idx, iov,
				reply_buf, reply_len,
				SKL_VIRTIO_IPC_MSG, SKL_VIRTIO_IPC_VQ_SZ);
		if (ret) {
			dev_err(dev, "Failed to handle virtio message");
			return;
		}

		/* send IPC to HW */
		ret = vbe_skl_ipc_fwd(vbe, vbe->sdev, vm_id, reply_buf[0],
				reply_buf[1], reply_len[0], &reply_len[1]);
		if (ret < 0)
			dev_err(dev, "submit guest ipc command fail\n");

		virtio_vq_relchain(vq, idx, reply_len[0]);
	}

	/* BE has finished the operations, now let's kick back */
	virtio_vq_endchains(vq, false);
}

/* IPC notification reply from FE to DSP */
static void vbe_skl_ipc_fe_not_reply_get(struct snd_skl_vbe *vbe, int vq_idx)
{
	const struct virtio_vq_info *vq;
	const struct vfe_pending_msg *entry;
	unsigned long flags;
	bool sent;

	if (list_empty(&vbe->pending_msg_list))
		return;

	vq = &vbe->vqs[vq_idx];
	entry = list_first_entry(&vbe->pending_msg_list,
				 struct vfe_pending_msg, list);

	sent = vbe_skl_try_send(vbe, vq,
		(void *)&entry->msg, entry->sizeof_msg);

	if (sent == true) {
		list_del(&entry->list);
		kfree(entry);
	}
}

void vbe_skl_handle_kick(const struct snd_skl_vbe *vbe, int vq_idx)
{
	dev_dbg(vbe->dev, "vq_idx %d\n", vq_idx);

	switch (vq_idx) {
	case SKL_VIRTIO_IPC_CMD_TX_VQ:
		/* IPC command from FE to DSP */
		vbe_skl_ipc_fe_cmd_get(vbe, vq_idx);
		break;
	case SKL_VIRTIO_IPC_CMD_RX_VQ:
		/* IPC command reply from DSP to FE - NOT kick */
		break;
	case SKL_VIRTIO_IPC_NOT_TX_VQ:
		/* IPC notification reply from FE to DSP */
		vbe_skl_ipc_fe_not_get(vbe, vq_idx);
		break;
	case SKL_VIRTIO_IPC_NOT_RX_VQ:
		/* IPC notification from DSP to FE - NOT kick */
		vbe_skl_ipc_fe_not_reply_get(vbe, vq_idx);
		break;
	default:
		dev_err(vbe->dev, "idx %d is invalid\n", vq_idx);
		break;
	}
}

int vbe_skl_attach(struct snd_skl_vbe *vbe, struct skl *skl)
{
	vbe->sdev = skl;

	vbe->nops.hda_irq_ack = skl->skl_sst->hda_irq_ack;
	skl->skl_sst->hda_irq_ack = vbe_stream_update;

	return 0;
}

int vbe_skl_detach(struct snd_skl_vbe *vbe, struct skl *skl)
{
	if (!vbe->sdev)
		return 0;

	skl->skl_sst->request_tplg = vbe->nops.request_tplg;
	skl->skl_sst->hda_irq_ack = vbe->nops.hda_irq_ack;

	/* TODO: Notify FE, close all streams opened by FE and delete all
	 * pending messages
	 */

	vbe->sdev = NULL;
	return 0;
}
