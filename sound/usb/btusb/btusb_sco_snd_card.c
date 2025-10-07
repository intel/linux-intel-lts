// SPDX-License-Identifier: GPL-2.0+
//
// btusb_sco_snd_card.c  --  ALSA USB audio driver for BT
//                           SCO over USB
//
// Copyright (C) 2019 Intel Corp.
//
// Authors: Aiswarya Cyriac <aiswarya.cyriac@intel.com>
//          Pankaj Bharadiya <pankaj.laxminarayan.bharadiya@intel.com>
//          Jeevaka Badrappan <jeevaka.badrappan@intel.com>

#include <asm/byteorder.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/usb.h>
#include <linux/usbdevice_fs.h>
#include <linux/version.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci.h>
#include <sound/core.h>
#include <sound/info.h>
#include <sound/pcm.h>

#define BTUSB_INTEL_NEW		           0x2000
#define DEVICE_INDEX		           0
#define NUM_PLAYBACK_SUBSTREAMS	           1
#define NUM_CAPTURE_SUBSTREAMS	           1
#define SCO_HDR_SIZE		           3
#define SCO_HANDLE_SIZE		           2
#define SCO_LENGTH_FIELD_SIZE	           1
#define MIN_PERIOD_SIZE		           48
#define MAX_PACKET_SIZE		           (SCO_HDR_SIZE + MIN_PERIOD_SIZE)
#define BTUSB_MAX_ISOC_FRAMES	           10
#define BTUSB_SCO_ALTERNATE_SETTING        2
#define BTUSB_ISOC_TX_EP_CONFIGURED        1
#define BTUSB_ISOC_RX_EP_CONFIGURED        2
#define BTUSB_ISOC_TX_START	           3
#define BTUSB_ISOC_RX_START	           4
#define MAX_URBS		           12
#define BUFF_SIZE_MAX	                   (PAGE_SIZE * 16)
#define PRD_SIZE_MAX	                   PAGE_SIZE  /*4096*/
#define MIN_PERIODS	                   4

static struct usb_driver btusb_sco_driver;
static DEFINE_MUTEX(config_mutex);

struct capture_data_cb {
	unsigned char *buf;
	unsigned int pos;
	unsigned int expected;
};

struct btusb_data {
	struct snd_card *card;
	struct snd_pcm *pcm;
	struct usb_device    *udev;
	struct usb_interface *intf;

	unsigned int isoc_ifnum;

	unsigned long flags;

	spinlock_t txlock;
	spinlock_t rxlock;

	struct usb_endpoint_descriptor *isoc_tx_ep;
	struct usb_endpoint_descriptor *isoc_rx_ep;

	unsigned int sco_num;
	int alternate_setting_num;
	uint8_t *playback_data;

	struct capture_data_cb *capture_data;

	struct snd_pcm_substream *playback_stream;
	struct snd_pcm_substream *capture_stream;

	unsigned int playback_hwptr_done;
	unsigned int playback_transfer_done;
	unsigned int capture_hwptr_done;

	struct usb_anchor rx_anchor;

	struct urb *tx_urb;
	struct urb *rx_urb[MAX_URBS];
};


uint8_t handle[2];

static const struct usb_device_id btusb_sco_table[] = {
	/* Intel Bluetooth devices */
	{ USB_DEVICE(0x8087, 0x0025), .driver_info = BTUSB_INTEL_NEW },
	{ USB_DEVICE(0x8087, 0x0026), .driver_info = BTUSB_INTEL_NEW },
	{ USB_DEVICE(0x8087, 0x0a2b), .driver_info = BTUSB_INTEL_NEW },
	{ USB_DEVICE(0x8087, 0x0aaa), .driver_info = BTUSB_INTEL_NEW },
	{ USB_DEVICE(0x8087, 0x0aa7), .driver_info = BTUSB_INTEL_NEW },
	{ }	/* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, btusb_sco_table);

static void btusb_isoc_tx_complete(struct urb *urb);

static struct capture_data_cb *init_capture_data_cb(void)
{

	struct capture_data_cb *cb;

	cb = kzalloc(sizeof(struct capture_data_cb), GFP_KERNEL);
	if (!cb)
		return NULL;
	cb->buf = kzalloc(HCI_MAX_SCO_SIZE, GFP_KERNEL);
	if (!cb->buf) {
		kfree(cb);
		return NULL;
	}
	cb->pos = 0;
	cb->expected = 0;

	return cb;
}

static void deinit_capture_data_cb(struct capture_data_cb *cb)
{
	kfree(cb->buf);
	kfree(cb);
	cb = NULL;
}

static inline void __fill_isoc_descriptor(struct urb *urb, int len, int mtu)
{
	int i, offset = 0;

	for (i = 0; i < BTUSB_MAX_ISOC_FRAMES && len >= mtu;
			i++, offset += mtu, len -= mtu) {
		urb->iso_frame_desc[i].offset = offset;
		urb->iso_frame_desc[i].length = mtu;
	}

	if (len && i < BTUSB_MAX_ISOC_FRAMES) {
		urb->iso_frame_desc[i].offset = offset;
		urb->iso_frame_desc[i].length = len;
		i++;
	}

	urb->number_of_packets = i;
}

static int btusb_isoc_initialize_tx_urb(struct btusb_data *data)
{
	unsigned int pipe;
	struct device *dev =  &(data->udev->dev);

	if (!data->isoc_tx_ep) {
		dev_err(dev, "%s - isoc_tx_ep is NULL\n", __func__);
		return -ENODEV;
	}
	data->tx_urb = usb_alloc_urb(BTUSB_MAX_ISOC_FRAMES, GFP_KERNEL);
	if (!data->tx_urb) {
		dev_err(dev, "%s - usb_alloc_urb is NULL\n", __func__);
		return -ENOMEM;
	}

	pipe = usb_sndisocpipe(data->udev, data->isoc_tx_ep->bEndpointAddress);

	usb_fill_int_urb(data->tx_urb, data->udev, pipe, data->playback_data,
			MAX_PACKET_SIZE, btusb_isoc_tx_complete, data,
			data->isoc_tx_ep->bInterval);

	data->tx_urb->transfer_flags = URB_ISO_ASAP;

	__fill_isoc_descriptor(data->tx_urb, MAX_PACKET_SIZE,
			le16_to_cpu(data->isoc_tx_ep->wMaxPacketSize));

	return 0;
}

static void btusb_isoc_prepare_tx_urb(struct btusb_data *data)
{
	unsigned int stride, frames, bytes;
	static unsigned int oldptr;
	uint8_t period_elapsed = 0;
	unsigned long flags;
	uint8_t *playback_buf;
	struct snd_pcm_runtime *runtime;

	runtime = data->playback_stream->runtime;
	stride = runtime->frame_bits >> 3;
	// TODO: Check to be added in hw_params to reject other period size
	bytes = MIN_PERIOD_SIZE;
	playback_buf = data->playback_data;

	memcpy(playback_buf, handle, SCO_HANDLE_SIZE);
	memcpy(playback_buf + SCO_HANDLE_SIZE, &bytes, SCO_LENGTH_FIELD_SIZE);

	spin_lock_irqsave(&data->txlock, flags);
	oldptr = data->playback_hwptr_done;
	spin_unlock_irqrestore(&data->txlock, flags);

	if (oldptr + bytes > runtime->buffer_size * stride) {
	/* err, the transferred area goes over buffer boundary. */
		unsigned int bytes1 =
			runtime->buffer_size * stride - oldptr;
		memcpy(playback_buf + SCO_HDR_SIZE,
			runtime->dma_area + oldptr, bytes1);
		memcpy(playback_buf + SCO_HDR_SIZE + bytes1,
			runtime->dma_area + oldptr+bytes1, bytes - bytes1);
	} else {
		memcpy(playback_buf + SCO_HDR_SIZE,
		       runtime->dma_area + oldptr, bytes);
	}
	frames = (bytes + (oldptr % stride)) / stride;
	spin_lock_irqsave(&data->txlock, flags);
	data->playback_hwptr_done  = (data->playback_hwptr_done + bytes) %
					(runtime->buffer_size * stride);
	data->playback_transfer_done += frames;
	if (data->playback_transfer_done >= runtime->period_size) {
		data->playback_transfer_done = 0;
		period_elapsed = 1;
	}
	spin_unlock_irqrestore(&data->txlock, flags);

	if (period_elapsed == 1) {
		period_elapsed = 0;
		return snd_pcm_period_elapsed(data->playback_stream);
	}
}

static void btusb_isoc_submit_tx_urb(struct btusb_data *data)
{
	struct device *dev =  &(data->udev->dev);
	int err;

	err = usb_submit_urb(data->tx_urb, GFP_ATOMIC);
	if (err < 0) {
		dev_err(dev, "%s urb %p submission failed (%d)",
					__func__, data->tx_urb, -err);
		usb_free_urb(data->tx_urb);
		data->tx_urb = NULL;
	}
}

static void btusb_isoc_tx_complete(struct urb *urb)
{
	struct btusb_data *data = urb->context;

	if (test_bit(BTUSB_ISOC_TX_START, &data->flags)) {
		btusb_isoc_prepare_tx_urb(data);
		btusb_isoc_submit_tx_urb(data);
	}
}

static void process_sco_buffer(struct btusb_data *data)
{
	unsigned long flags;
	unsigned int hw_ptr;
	int sco_data_count;
	struct snd_pcm_runtime *runtime;
	unsigned char *sco_buffer = data->capture_data->buf;
	unsigned char *audio_buffer = data->capture_data->buf + SCO_HDR_SIZE;
	unsigned int pending;

	sco_data_count = sco_buffer[SCO_HANDLE_SIZE];
	spin_lock_irqsave(&data->rxlock, flags);
	hw_ptr = data->capture_hwptr_done;
	spin_unlock_irqrestore(&data->rxlock, flags);
	runtime = data->capture_stream->runtime;
	pending = runtime->dma_bytes - hw_ptr;

	if (pending < sco_data_count) {
		memcpy(runtime->dma_area + hw_ptr, audio_buffer, pending);
		memcpy(runtime->dma_area, (void *)audio_buffer + pending,
				sco_data_count - pending);
	} else {
		memcpy(runtime->dma_area + hw_ptr, (void *) audio_buffer,
				sco_data_count);
	}
	spin_lock_irqsave(&data->rxlock, flags);
	data->capture_hwptr_done  = (hw_ptr + sco_data_count) %
						runtime->dma_bytes;
	hw_ptr = data->capture_hwptr_done;
	spin_unlock_irqrestore(&data->rxlock, flags);
}

static int btusb_recv_isoc(struct btusb_data *data, void *buffer, int count)
{
	int err = 0;
	struct capture_data_cb *cb = data->capture_data;
	int len;
	struct hci_sco_hdr *sco_hdr;
	struct device *dev =  &(data->udev->dev);

	if (count == 0)
		return -ENODATA;

	while (count) {

		if (cb->pos == 0)
			cb->expected = SCO_HDR_SIZE;

		len = min_t(uint, cb->expected, count);
		memcpy(cb->buf + cb->pos, buffer, len);
		count -= len;
		cb->expected -= len;
		cb->pos += len;
		buffer += len;

		if (cb->pos == SCO_HDR_SIZE) {
			sco_hdr = (struct hci_sco_hdr *)cb->buf;
			cb->expected = cb->buf[2];
			if (memcmp(cb->buf, handle, 2)) {
				cb->pos = 0;
				dev_err(dev, "%s Invalid SCO handle", __func__);
				return -EPROTO;
			}
			if (cb->expected > 48) {
				dev_err(dev, "%s:Error in capture", __func__);
				return -EPROTO;
			}
			if ((HCI_MAX_SCO_SIZE - cb->pos) < cb->expected) {
				dev_err(dev, "Invalid USB frame");
				cb->pos = 0;
				err = -EPROTO;
				return err;
			}
		}
		if (!cb->expected) {
			// copy data to stream buffer
			process_sco_buffer(data);
			cb->pos = 0;
		}
	}
	return 0;
}

static int btusb_isoc_submit_urb(struct urb *urb, struct btusb_data *data)
{
	int err;
	struct device *dev = &(data->udev->dev);

	usb_anchor_urb(urb, &data->rx_anchor);
	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0) {
		/* -EPERM: urb is being killed;
		 * -ENODEV: device got disconnected
		 */
		if (err != -EPERM && err != -ENODEV)
			dev_err(dev, "urb %p failed to submit (%d)",
				urb, err);
		usb_unanchor_urb(urb);
	}
	return err;
}
static int btusb_isoc_submit_rx_urbs(struct btusb_data *data)
{
	int i, err = 0;
	struct device *dev = &(data->udev->dev);

	for (i = 0; i < MAX_URBS; i++) {
		err = btusb_isoc_submit_urb(data->rx_urb[i], data);
		if (err < 0) {
			dev_err(dev, "%s - err: %d\n", __func__, err);
			break;
		}
	}
	return (i < 2) ? err : 0;
}

static void btusb_isoc_rx_complete(struct urb *urb)
{
	struct btusb_data *data = urb->context;
	struct device *dev = &(data->udev->dev);
	int i;

	if (!test_bit(BTUSB_ISOC_RX_START, &data->flags)) {
		dev_dbg(dev, "%s BTUSB_ISOC_RX_START not set\n", __func__);
		return;
	}

	if (urb->status == 0) {
		for (i = 0; i < urb->number_of_packets; i++) {

			unsigned int offset = urb->iso_frame_desc[i].offset;
			unsigned int length =
				urb->iso_frame_desc[i].actual_length;

			if (urb->iso_frame_desc[i].status)
				continue;
			if (length <= 0)
				continue;
			btusb_recv_isoc(data, urb->transfer_buffer + offset,
						length);
		}
	} else if (urb->status == -ENOENT || urb->status == -ENODEV) {
		/* Avoid suspend failed when usb_kill_urb */
		dev_err(dev, "USB disconnected or Endpoint not available\n");
		return;
	}

	if (urb->status != 0) {
		dev_err(dev, "%s - urb->status= %d\n", __func__, urb->status);
		goto submit;
	}

	snd_pcm_period_elapsed(data->capture_stream);

submit:
	if (btusb_isoc_submit_urb(urb, data))
		dev_err(dev, " btusb_isoc_submit_rx_urb failed\n");
}

static void btusb_free_rx_urbs(struct btusb_data *data)
{
	int i;
	struct device *dev;

	dev = &(data->udev->dev);
	for (i = 0; i < MAX_URBS; i++) {
		if (data->rx_urb[i]) {
			dev_dbg(dev, "%s - Valid URB\n", __func__);
			usb_free_urb(data->rx_urb[i]);
			data->rx_urb[i] = NULL;
		}
	}
}

static int btusb_isoc_prepare_rx_urbs(struct btusb_data *data)
{
	struct urb *urb;
	unsigned char *buf;
	unsigned int pipe;
	int i, size;
	struct device *dev;

	dev = &(data->udev->dev);

	if (!data->isoc_rx_ep) {
		dev_err(dev, "%s isoc_rx_ep is NULL\n", __func__);
		return -ENODEV;
	}
	for (i = 0; i < MAX_URBS; i++) {
		urb = usb_alloc_urb(BTUSB_MAX_ISOC_FRAMES, GFP_KERNEL);

		if (!urb) {
			dev_err(dev, "%s URB allocation failed\n", __func__);
			return -ENOMEM;
		}

		size = le16_to_cpu(data->isoc_rx_ep->wMaxPacketSize) *
			BTUSB_MAX_ISOC_FRAMES;

		buf = kmalloc(size, GFP_KERNEL);
		if (!buf) {
			btusb_free_rx_urbs(data);
			return -ENOMEM;
		}
		pipe = usb_rcvisocpipe(data->udev,
				data->isoc_rx_ep->bEndpointAddress);

		usb_fill_int_urb(urb, data->udev, pipe, buf, size,
				btusb_isoc_rx_complete, data,
				data->isoc_rx_ep->bInterval);

		urb->transfer_flags = URB_FREE_BUFFER | URB_ISO_ASAP;

		__fill_isoc_descriptor(urb, size,
				le16_to_cpu(data->isoc_rx_ep->wMaxPacketSize));

		data->rx_urb[i] = urb;
	}
	return 0;
}

static inline int __set_isoc_interface(struct snd_pcm_substream *substream,
				       int alternate_setting_num)
{
	struct usb_endpoint_descriptor *ep_desc;
	int i, err;
	struct device *dev;
	struct btusb_data *data = snd_pcm_substream_chip(substream);

	dev = &(data->udev->dev);

	if (!data->intf)
		return -ENODEV;

	if (data->alternate_setting_num == alternate_setting_num)
		return 0;

	err = usb_set_interface(data->udev, data->isoc_ifnum,
				alternate_setting_num);
	if (err < 0) {
		dev_err(dev, "%s failed (%d)", __func__, -err);
		return err;
	}
	data->alternate_setting_num = alternate_setting_num;

	data->isoc_tx_ep = NULL;
	data->isoc_rx_ep = NULL;

	for (i = 0; i < data->intf->cur_altsetting->desc.bNumEndpoints; i++) {
		ep_desc = &data->intf->cur_altsetting->endpoint[i].desc;

		if (!data->isoc_tx_ep && usb_endpoint_is_isoc_out(ep_desc)) {
			data->isoc_tx_ep = ep_desc;
			continue;
		}

		if (!data->isoc_rx_ep && usb_endpoint_is_isoc_in(ep_desc)) {
			data->isoc_rx_ep = ep_desc;
			continue;
		}
	}

	if (!data->isoc_tx_ep || !data->isoc_rx_ep) {
		dev_err(dev, "%s - invalid SCO descriptors", __func__);
		return -ENODEV;
	}

	return 0;
}

static struct snd_pcm_hardware btsco_pcm_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER
		| SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID,
	.rates = SNDRV_PCM_RATE_CONTINUOUS,
	.buffer_bytes_max = BUFF_SIZE_MAX,
	.period_bytes_min = MIN_PERIOD_SIZE,
	.period_bytes_max = 16384,
	.periods_min =	    MIN_PERIODS,
	.periods_max =	    1024,

};

static int bt_pcm_open(struct snd_pcm_substream *substream)
{
	struct btusb_data *data;
	struct snd_pcm_runtime *runtime;
	struct device *dev;

	data = snd_pcm_substream_chip(substream);
	if (!data)
		return -ENOMEM;

	dev = &(data->udev->dev);
	runtime = substream->runtime;

	if (runtime == NULL) {
		dev_err(dev, "%s: value of runtime is NULL", __func__);
		return -ENODEV;
	}

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		data->capture_data = init_capture_data_cb();
		if (!data->capture_data) {
			dev_err(dev, "value of capture_data is NULL");
			return -ENOMEM;
		}
	} else if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		data->playback_data =
			kzalloc(sizeof(uint8_t) * MAX_PACKET_SIZE, GFP_KERNEL);
		if (!data->playback_data) {
			dev_err(dev, "playback_data memory allocation failed");
			return -ENOMEM;
		}
	}

	runtime->hw = btsco_pcm_hardware;

	runtime->hw.rate_min = 8000;
	runtime->hw.formats = SNDRV_PCM_FMTBIT_S16_LE;
	runtime->hw.channels_min = 1;
	runtime->hw.period_bytes_min = MIN_PERIOD_SIZE;
	runtime->hw.rate_max = 16000;
	runtime->hw.channels_max = runtime->hw.channels_min;
	snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);

	return 0;
}

static int configure_endpoints(struct snd_pcm_substream *substream)
{
	struct btusb_data *data;
	struct device *dev;
	int err;

	data = snd_pcm_substream_chip(substream);
	dev = &(data->udev->dev);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		set_bit(BTUSB_ISOC_TX_EP_CONFIGURED, &data->flags);
	else
		set_bit(BTUSB_ISOC_RX_EP_CONFIGURED, &data->flags);

	if (test_bit(BTUSB_ISOC_RX_EP_CONFIGURED, &data->flags) ^
			 test_bit(BTUSB_ISOC_TX_EP_CONFIGURED, &data->flags)) {
		err = __set_isoc_interface(substream, BTUSB_SCO_ALTERNATE_SETTING);
		if (err < 0) {
			dev_err(dev, "setting alt settings failed\n");
			return -EIO;
		}
	}
	return 0;
}


static int bt_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct btusb_data *data;
	int err;
	struct device *dev;

	data = snd_pcm_substream_chip(substream);
	dev = &(data->udev->dev);
	// TODO: Does endpoint needs to be configured separately for Tx and Rx
	// endpoints?
	mutex_lock(&config_mutex);
	err = configure_endpoints(substream);
	mutex_unlock(&config_mutex);
	if (err < 0) {
		dev_err(dev, "failed to configure Endpoints err:%d\n", err);
		return err;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		dev_dbg(dev, "%s - SNDRV_PCM_STREAM_PLAYBACK\n", __func__);
		btusb_isoc_prepare_tx_urb(data);
		err = btusb_isoc_initialize_tx_urb(data);
		if (err < 0) {
			dev_err(dev, "Failed in init tx urb err:%d\n", err);
			usb_free_urb(data->tx_urb);
			data->tx_urb = NULL;
			return err;
		}
	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		dev_dbg(dev, "%s - SNDRV_PCM_STREAM_CAPTURE\n", __func__);
		err = btusb_isoc_prepare_rx_urbs(data);
		if (err < 0) {
			dev_err(dev, "Failed to init Rx urb err:%d\n", err);
			return err;
		}
	}
	return err;
}

static void btusb_isoc_stop_tx(struct btusb_data *data)
{
	clear_bit(BTUSB_ISOC_TX_START, &data->flags);
}

static int bt_pcm_close(struct snd_pcm_substream *substream)
{
	struct btusb_data *data;
	struct device *dev;

	data = snd_pcm_substream_chip(substream);
	dev = &(data->udev->dev);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		dev_dbg(dev, "%s SNDRV_PCM_STREAM_PLAYBACK\n", __func__);
		clear_bit(BTUSB_ISOC_TX_START, &data->flags);
		clear_bit(BTUSB_ISOC_TX_EP_CONFIGURED, &data->flags);
		kfree(data->playback_data);
		data->playback_data = NULL;
	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		dev_dbg(dev, "%s SNDRV_PCM_STREAM_CAPTURE\n", __func__);
		clear_bit(BTUSB_ISOC_RX_START, &data->flags);
		clear_bit(BTUSB_ISOC_RX_EP_CONFIGURED, &data->flags);
		deinit_capture_data_cb(data->capture_data);
	}
	if (!test_bit(BTUSB_ISOC_RX_EP_CONFIGURED, &data->flags)
		&& !test_bit(BTUSB_ISOC_TX_EP_CONFIGURED, &data->flags)) {
		dev_dbg(dev, "%s Reset EP and Alternate settings\n", __func__);
		if (__set_isoc_interface(substream, 0) < 0) {
			dev_err(dev, "Resetting alt settings failed");
			return -ENOENT;
		}
	}
	return 0;
}

static int bt_pcm_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *hw_params)
{
	struct btusb_data *data;
	int err;
	unsigned long flags;
	struct device *dev;

	data = snd_pcm_substream_chip(substream);
	dev = &(data->udev->dev);
	err = snd_pcm_lib_malloc_pages(substream,
			params_buffer_bytes(hw_params));

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		dev_dbg(dev, "%s - SNDRV_PCM_STREAM_PLAYBACK\n", __func__);
		spin_lock_irqsave(&data->txlock, flags);
		data->playback_stream = substream;
		data->playback_hwptr_done = 0;
		data->playback_transfer_done = 0;
		spin_unlock_irqrestore(&data->txlock, flags);
	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		dev_dbg(dev, "%s - SNDRV_PCM_STREAM_CAPTURE\n", __func__);
		spin_lock_irqsave(&data->rxlock, flags);
		data->capture_stream = substream;
		data->capture_hwptr_done = 0;
		spin_unlock_irqrestore(&data->rxlock, flags);
	}

	return 0;

}
static int bt_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct btusb_data *data;
	unsigned long flags;
	struct device *dev;

	data = snd_pcm_substream_chip(substream);
	dev = &(data->udev->dev);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		dev_dbg(dev, "%s SNDRV_PCM_STREAM_PLAYBACK\n", __func__);
		clear_bit(BTUSB_ISOC_TX_START, &data->flags);
		if (data->tx_urb) {
			usb_kill_urb(data->tx_urb);
			kfree(data->tx_urb->setup_packet);
			usb_free_urb(data->tx_urb);
			data->tx_urb = NULL;
		}
		spin_lock_irqsave(&data->txlock, flags);
		data->playback_stream = NULL;
		data->playback_hwptr_done = 0;
		data->playback_transfer_done = 0;
		spin_unlock_irqrestore(&data->txlock, flags);
	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		dev_dbg(dev, "%s SNDRV_PCM_STREAM_CAPTURE\n", __func__);
		clear_bit(BTUSB_ISOC_RX_START, &data->flags);
		usb_kill_anchored_urbs(&data->rx_anchor);
		btusb_free_rx_urbs(data);
		spin_lock_irqsave(&data->rxlock, flags);
		data->capture_stream = NULL;
		data->capture_hwptr_done = 0;
		spin_unlock_irqrestore(&data->rxlock, flags);
	}
	snd_pcm_lib_free_pages(substream);
	return 0;
}

static void btusb_isoc_stop_rx(struct btusb_data *data)
{
	clear_bit(BTUSB_ISOC_RX_START, &data->flags);
}

static int bt_pcm_capture_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct btusb_data *data;
	int err = 0;
	struct device *dev;

	data = snd_pcm_substream_chip(substream);
	dev = &(data->udev->dev);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		dev_dbg(dev, "%s - command: %s", __func__,
				(cmd == SNDRV_PCM_TRIGGER_START) ?
		"SNDRV_PCM_TRIGGER_START" : "SNDRV_PCM_TRIGGER_RESUME");
		set_bit(BTUSB_ISOC_RX_START, &data->flags);
		err = btusb_isoc_submit_rx_urbs(data);
		if (err < 0)
			dev_err(dev, "%s err:%d\n", __func__, err);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		dev_dbg(dev, "%s - command: %s", __func__,
				(cmd == SNDRV_PCM_TRIGGER_STOP) ?
		"SNDRV_PCM_TRIGGER_STOP" : "SNDRV_PCM_TRIGGER_SUSPEND");
		btusb_isoc_stop_rx(data);
		break;
	default:
		err = -EINVAL;
	}
	return err;
}

static int bt_pcm_playback_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct btusb_data *data = snd_pcm_substream_chip(substream);
	int err = 0;
	struct device *dev = &(data->udev->dev);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		dev_dbg(dev, "%s - command: %s", __func__,
			(cmd == SNDRV_PCM_TRIGGER_START)
		? "SNDRV_PCM_TRIGGER_START" : "SNDRV_PCM_TRIGGER_RESUME");
		set_bit(BTUSB_ISOC_TX_START, &data->flags);
		btusb_isoc_submit_tx_urb(data);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		dev_dbg(dev, "%s - command: %s", __func__,
				(cmd == SNDRV_PCM_TRIGGER_STOP)
		? "SNDRV_PCM_TRIGGER_STOP" : "SNDRV_PCM_TRIGGER_SUSPEND");
		btusb_isoc_stop_tx(data);
		break;
	default:
		dev_err(dev, "%s - Invalid cmd: %d\n", __func__, cmd);
		err = -EINVAL;
	}
	return err;
}

static snd_pcm_uframes_t bt_pcm_pointer(struct snd_pcm_substream *substream)
{
	unsigned int playback_hwptr_done;
	unsigned int capture_hwptr_done;
	unsigned long flags;
	struct btusb_data *data = snd_pcm_substream_chip(substream);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		spin_lock_irqsave(&data->rxlock, flags);
		capture_hwptr_done = data->capture_hwptr_done;
		spin_unlock_irqrestore(&data->rxlock, flags);
		return capture_hwptr_done /
				(substream->runtime->frame_bits >> 3);
	} else if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		spin_lock_irqsave(&data->txlock, flags);
		playback_hwptr_done = data->playback_hwptr_done;
		spin_unlock_irqrestore(&data->txlock, flags);
		return playback_hwptr_done /
				(substream->runtime->frame_bits >> 3);
	} else
		return -EINVAL;
}
static const struct snd_pcm_ops btusb_isoc_capture_ops = {
	.open = bt_pcm_open,
	.close = bt_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = bt_pcm_hw_params,
	.hw_free = bt_pcm_hw_free,
	.trigger = bt_pcm_capture_trigger,
	.pointer = bt_pcm_pointer,
	.prepare = bt_pcm_prepare,
};

static const struct snd_pcm_ops btusb_isoc_playback_ops = {
	.open = bt_pcm_open,
	.close = bt_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = bt_pcm_hw_params,
	.hw_free = bt_pcm_hw_free,
	.trigger = bt_pcm_playback_trigger,
	.pointer = bt_pcm_pointer,
	.prepare = bt_pcm_prepare,
};

static int snd_usb_audio_dev_free(struct snd_device *device)
{
	struct btusb_data *data = device->device_data;
	unsigned long flags;

	if (!data)
		return 0;

	clear_bit(BTUSB_ISOC_TX_START, &data->flags);

	if (data->tx_urb) {
		usb_kill_urb(data->tx_urb);
		kfree(data->tx_urb->setup_packet);
		usb_free_urb(data->tx_urb);
		data->tx_urb = NULL;
	}
	spin_lock_irqsave(&data->txlock, flags);
	data->playback_stream = NULL;
	data->playback_hwptr_done = 0;
	data->playback_transfer_done = 0;
	spin_unlock_irqrestore(&data->txlock, flags);

	clear_bit(BTUSB_ISOC_RX_START, &data->flags);
	usb_kill_anchored_urbs(&data->rx_anchor);
	btusb_free_rx_urbs(data);
	spin_lock_irqsave(&data->rxlock, flags);
	data->capture_stream = NULL;
	data->capture_hwptr_done = 0;
	spin_unlock_irqrestore(&data->rxlock, flags);
	return 0;
}

static int btusb_snd_card_create(struct btusb_data *data)
{
	int  err = 0;
	struct snd_card *card;
	struct device *dev;

	static struct snd_device_ops ops = {
		.dev_free =	snd_usb_audio_dev_free,
	};
	struct snd_pcm *pcm;

	dev = &(data->udev->dev);
	err = snd_card_new(dev, -1, NULL, THIS_MODULE, 0, &card);
	if (err) {
		dev_err(dev, "%s:Fail to create sound card", __func__);
		return err;
	}
	data->card = card;
	err = snd_device_new(card, SNDRV_DEV_LOWLEVEL, data, &ops);

	if (err) {
		snd_card_free(card);
		dev_err(dev, "Error in creating the sound device device");
		return err;
	}
	err = snd_pcm_new(card, "BT USB audio source", DEVICE_INDEX,
			  NUM_PLAYBACK_SUBSTREAMS, NUM_CAPTURE_SUBSTREAMS,
			  &pcm);
	if (err) {
		dev_err(dev, "%s: Fail to create PCM device ", __func__);
		return err;
	}

	pcm->private_data = data;
	pcm->info_flags = 0;
	data->pcm = pcm;
	strscpy(pcm->name, "USB Bluetooth audio", sizeof(pcm->name));
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
					&btusb_isoc_capture_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
					&btusb_isoc_playback_ops);
	snd_pcm_lib_preallocate_pages_for_all(pcm,
			SNDRV_DMA_TYPE_CONTINUOUS,
			NULL,
			64 * 1024, 64 * 1024);
	strscpy(card->driver, "bt_audio_source", sizeof(card->driver));
	strscpy(card->shortname, card->driver, sizeof(card->shortname));
	strscpy(card->longname, "USB Bluetooth audio source",
			sizeof(card->longname));

	err = snd_card_register(card);
	if (err)
		dev_err(dev, "%s: Failure in registering sound card", __func__);

	return err;
}

int btusb_sco_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	int  err = 0;
	struct btusb_data *data;

	data = devm_kzalloc(&intf->dev, sizeof(*data), GFP_KERNEL);

	if (!data)
		return -ENOMEM;

	data->isoc_ifnum = intf->cur_altsetting->desc.bInterfaceNumber;
	data->udev = interface_to_usbdev(intf);
	data->intf = intf;
	usb_set_intfdata(intf, data);
	spin_lock_init(&data->txlock);
	spin_lock_init(&data->rxlock);
	init_usb_anchor(&data->rx_anchor);

	err = btusb_snd_card_create(data);

	return err;
}

static void btusb_sco_disconnect(struct usb_interface *intf)
{
	struct btusb_data *data = usb_get_intfdata(intf);

	if (!data)
		return;

	if (data->intf) {
		usb_set_intfdata(data->intf, NULL);
		usb_driver_release_interface(&btusb_sco_driver, data->intf);
	}
	snd_card_disconnect(data->card);
	snd_card_free_when_closed(data->card);
	/* TODO: Freeing the resource causing kernel panic.Cleanup
	 * part needs to be checked.
	 * kfree(data);
	 * data = NULL;
	 */
}

static int btusb_sco_suspend(struct usb_interface *intf, pm_message_t message)
{
	// TODO: Do we need to call snd_pcm_suspend_all
	// snd_pcm_suspend_all(data->pcm);
	return 0;
}

static int btusb_sco_resume(struct usb_interface *intf)
{
	// TODO: Do we need to call snd_power_change_state
	// snd_power_change_state(data->card, SNDRV_CTL_POWER_D0);

	return 0;

}

static int btusb_sco_ioctl(struct usb_interface *intf, unsigned int code,
		void *buf)
{
	struct device *dev;
	int err;

	if (!buf)
		return -EINVAL;

	dev = &(interface_to_usbdev(intf)->dev);

	if (code == USBDEVFS_IOCTL) {
		memcpy(handle, (uint8_t *)buf, SCO_HANDLE_SIZE);
		dev_dbg(dev, "value of handle = %x, %x", handle[0], handle[1]);
		err = 0;
	} else {
		dev_err(dev, "%s:Invalid Ioctl code", __func__);
		err = -ENOIOCTLCMD;
	}
	return err;
}

static struct usb_driver btusb_sco_driver = {
	.name		= "btusb_sco",
	.probe		= btusb_sco_probe,
	.unlocked_ioctl = btusb_sco_ioctl,
	.disconnect	= btusb_sco_disconnect,
	.suspend	= btusb_sco_suspend,
	.resume		= btusb_sco_resume,
	.id_table	= btusb_sco_table,
	.supports_autosuspend = 0

};

module_usb_driver(btusb_sco_driver);

MODULE_DESCRIPTION("SCO over btusb driver for Android");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
