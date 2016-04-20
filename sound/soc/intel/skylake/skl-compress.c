/*
 *  skl-compress.c -ASoC HDA Platform driver file implementing compress functionality
 *
 *  Copyright (C) 2015-2016 Intel Corp
 *  Author:  Divya Prakash <divya1.prakash@intel.com>
 *
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 */

#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "skl.h"
#include "skl-topology.h"
#include "skl-sst-ipc.h"

#define USE_SPIB 0

static inline
struct hdac_ext_stream *get_hdac_ext_compr_stream(struct snd_compr_stream *stream)
{
	return stream->runtime->private_data;
}

static struct hdac_ext_bus *get_bus_compr_ctx(struct snd_compr_stream *substream)
{
	struct hdac_ext_stream *stream = get_hdac_ext_compr_stream(substream);
	struct hdac_stream *hstream = hdac_stream(stream);
	struct hdac_bus *bus = hstream->bus;

	return hbus_to_ebus(bus);
}

static void skl_set_compr_runtime_buffer(struct snd_compr_stream *substream,
				struct snd_dma_buffer *bufp, size_t size)
{
	struct snd_compr_runtime *runtime = substream->runtime;

	if (bufp) {
		runtime->dma_buffer_p = bufp;
		runtime->dma_area = bufp->area;
		runtime->dma_addr = bufp->addr;
		runtime->dma_bytes = size;
	} else {
		runtime->dma_buffer_p = NULL;
		runtime->dma_area = NULL;
		runtime->dma_addr = 0;
		runtime->dma_bytes = 0;
	}
}

static int skl_compr_malloc_pages(struct snd_compr_stream *substream,
					struct hdac_ext_bus *ebus, size_t size)
{
	struct snd_compr_runtime *runtime;
	struct snd_dma_buffer *dmab = NULL;
	struct skl *skl = ebus_to_skl(ebus);

	runtime = substream->runtime;

	dmab = kzalloc(sizeof(*dmab), GFP_KERNEL);
	if (!dmab)
		return -ENOMEM;
	substream->dma_buffer.dev.type = SNDRV_DMA_TYPE_DEV_SG;
	substream->dma_buffer.dev.dev = snd_dma_pci_data(skl->pci);
	dmab->dev = substream->dma_buffer.dev;
	if (snd_dma_alloc_pages(substream->dma_buffer.dev.type,
				substream->dma_buffer.dev.dev,
				size, dmab) < 0) {
		dev_err(ebus_to_hbus(ebus)->dev,
			"Error in snd_dma_alloc_pages\n");
		kfree(dmab);
		return -ENOMEM;
	}
	skl_set_compr_runtime_buffer(substream, dmab, size);

	return 1;
}

static int skl_substream_alloc_compr_pages(struct hdac_ext_bus *ebus,
				 struct snd_compr_stream *substream,
				 size_t size)
{
	struct hdac_ext_stream *stream = get_hdac_ext_compr_stream(substream);
	int ret;

	hdac_stream(stream)->bufsize = 0;
	hdac_stream(stream)->period_bytes = 0;
	hdac_stream(stream)->format_val = 0;

	ret = skl_compr_malloc_pages(substream, ebus, size);
	if (ret < 0)
		return ret;
	ebus->bus.io_ops->mark_pages_uc(snd_pcm_get_dma_buf(substream), true);

	return ret;
}

static int skl_compr_free_pages(struct snd_compr_stream *substream)
{
	struct snd_compr_runtime *runtime;

	runtime = substream->runtime;
	if (runtime->dma_area == NULL)
		return 0;

	if (runtime->dma_buffer_p != &substream->dma_buffer) {
		/* it's a newly allocated buffer.  release it now. */
		snd_dma_free_pages(runtime->dma_buffer_p);
		kfree(runtime->dma_buffer_p);
	}

	skl_set_compr_runtime_buffer(substream, NULL, 0);
	return 0;
}

static int skl_substream_free_compr_pages(struct hdac_bus *bus,
				struct snd_compr_stream *substream)
{
	bus->io_ops->mark_pages_uc((substream)->runtime->dma_buffer_p, false);

	return skl_compr_free_pages(substream);
}

int skl_probe_compr_open(struct snd_compr_stream *substream,
						struct snd_soc_dai *dai)
{
	struct hdac_ext_bus *ebus = dev_get_drvdata(dai->dev);
	struct hdac_ext_stream *stream;
	struct snd_compr_runtime *runtime = substream->runtime;

	dev_dbg(dai->dev, "%s dev is  %s\n",  __func__, dev_name(dai->dev));

	stream = hdac_ext_host_stream_compr_assign(ebus, substream,
							substream->direction);
	runtime->private_data = stream;

	if (stream == NULL) {
		dev_err(dai->dev, "stream = NULL\n");
		return -EBUSY;
	}

	hdac_stream(stream)->curr_pos = 0;

	return 0;
}

int skl_probe_compr_set_params(struct snd_compr_stream *substream,
					struct snd_compr_params *params,
							struct snd_soc_dai *dai)
{

	struct hdac_ext_bus *ebus = dev_get_drvdata(dai->dev);
	struct hdac_ext_stream *stream = get_hdac_ext_compr_stream(substream);
	struct snd_compr_runtime *runtime = substream->runtime;
	int ret, dma_id;
	unsigned int format_val = 0;
	int err;

	dev_dbg(dai->dev, "%s: %s\n", __func__, dai->name);
	ret = skl_substream_alloc_compr_pages(ebus, substream,
				runtime->fragments*runtime->fragment_size);
	if (ret < 0)
		return ret;

	dma_id = hdac_stream(stream)->stream_tag - 1;
	dev_dbg(dai->dev, "dma_id=%d\n", dma_id);

	if (hdac_stream(stream)->prepared) {
		dev_dbg(dai->dev, "already stream is prepared - returning\n");
		return 0;
	}

	snd_hdac_stream_reset(hdac_stream(stream));

	err = snd_hdac_stream_set_params(hdac_stream(stream), format_val);
	if (err < 0)
		return err;

	err = snd_hdac_stream_setup(hdac_stream(stream));
	if (err < 0) {
		dev_err(dai->dev, "snd_hdac_stream_setup err = %d\n", err);
		return err;
	}

	hdac_stream(stream)->prepared = 1;

	return 0;
}

int skl_probe_compr_close(struct snd_compr_stream *substream,
						struct snd_soc_dai *dai)
{
	struct hdac_ext_stream *stream = get_hdac_ext_compr_stream(substream);
	struct hdac_ext_bus *ebus = dev_get_drvdata(dai->dev);

	dev_dbg(dai->dev, "%s: %s\n", __func__, dai->name);

	snd_hdac_stream_cleanup(hdac_stream(stream));
	hdac_stream(stream)->prepared = 0;

	skl_substream_free_compr_pages(ebus_to_hbus(ebus), substream);

	snd_hdac_ext_stream_release(stream, HDAC_EXT_STREAM_TYPE_HOST);

	return 0;
}

int skl_probe_compr_ack(struct snd_compr_stream *substream, size_t bytes,
							struct snd_soc_dai *dai)
{
	struct hdac_ext_bus *ebus = dev_get_drvdata(dai->dev);
	u64 new_spib_pos;
	struct snd_compr_runtime *runtime = substream->runtime;
	u64 spib_pos = div64_u64(runtime->total_bytes_available,
				    runtime->buffer_size);

	spib_pos = runtime->total_bytes_available -
		      (spib_pos * runtime->buffer_size);
	/*SPIB position is a wrap around counter that indicates
	the position relative to the buffer start address*/
	new_spib_pos = (spib_pos + bytes) % runtime->buffer_size;

	if (!ebus->spbcap) {
		dev_err(dai->dev, "Address of SPB capability is NULL");
		return -EINVAL;
	}
#if USE_SPIB
	writel(new_spib_pos, stream->spib_addr);
#endif
	return 0;
}

int skl_probe_compr_tstamp(struct snd_compr_stream *stream,
		struct snd_compr_tstamp *tstamp, struct snd_soc_dai *dai)
{
	struct hdac_ext_stream *hstream = get_hdac_ext_compr_stream(stream);

	tstamp->copied_total = hstream->hstream.curr_pos;

	return 0;

}

int skl_probe_compr_copy(struct snd_compr_stream *stream, char __user *buf,
					size_t count, struct snd_soc_dai *dai)
{
	int offset = 0, availcount = 0, retval = 0, copy;
	void *dstn;

	if (stream->direction == SND_COMPRESS_CAPTURE) {
		offset = stream->runtime->total_bytes_transferred %
						stream->runtime->buffer_size;
		dstn = stream->runtime->dma_area + offset;
		availcount = (stream->runtime->buffer_size - offset);
		if (count > availcount) {

			retval = copy_to_user(buf, dstn, availcount);
			retval += copy_to_user(buf + availcount,
					stream->runtime->dma_area,
							count - availcount);
		} else
			retval = copy_to_user(buf, stream->runtime->dma_area
							+ offset, count);

		if (!retval)
			retval = count;
		else
			retval = count - retval;

	} else if (stream->direction == SND_COMPRESS_PLAYBACK) {

		offset = stream->runtime->total_bytes_available %
						stream->runtime->buffer_size;
		dstn = stream->runtime->dma_area + offset;

		if (count < stream->runtime->buffer_size - offset)
			retval = copy_from_user(dstn, buf, count);
		else {
			copy = stream->runtime->buffer_size - offset;
			retval = copy_from_user(dstn, buf, copy);
			retval += copy_from_user(stream->runtime->dma_area,
						buf + copy, count - copy);
		}
			if (!retval)
				retval = count;
			else
				retval = count - retval;
	}

#if USE_SPIB
	spib_pos = (offset + retval)%stream->runtime->dma_bytes;
	snd_hdac_ext_stream_set_spib(ebus, estream, spib_pos);
#endif

	return retval;

}

int skl_probe_compr_trigger(struct snd_compr_stream *substream, int cmd,
							struct snd_soc_dai *dai)
{
	struct hdac_ext_bus *ebus = get_bus_compr_ctx(substream);
	struct hdac_bus *bus = ebus_to_hbus(ebus);
	struct hdac_ext_stream *stream;
	struct hdac_stream *hstr;
	int start;
	unsigned long cookie;

	stream = get_hdac_ext_compr_stream(substream);
	hstr = hdac_stream(stream);

	if (!hstr->prepared)
		return -EPIPE;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		start = 1;
		break;

	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
		start = 0;
		break;

	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&bus->reg_lock, cookie);

	if (start)
		snd_hdac_stream_start(hdac_stream(stream), true);
	else
		snd_hdac_stream_stop(hdac_stream(stream));

	spin_unlock_irqrestore(&bus->reg_lock, cookie);

	return 0;
}
