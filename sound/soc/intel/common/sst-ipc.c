/*
 * Intel SST generic IPC Support
 *
 * Copyright (C) 2015, Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <sound/asound.h>

#include "sst-dsp.h"
#include "sst-dsp-priv.h"
#include "sst-ipc.h"

/* IPC message timeout (msecs) */
#define IPC_TIMEOUT_MSECS	300

static int msg_init(struct sst_generic_ipc *ipc)
{
	ipc->msg = kzalloc(sizeof(struct ipc_message), GFP_KERNEL);
	if (ipc->msg == NULL)
		return -ENOMEM;

	ipc->msg->tx_data = kzalloc(ipc->tx_data_max_size, GFP_KERNEL);
	if (ipc->msg->tx_data == NULL)
		goto free_mem;

	ipc->msg->rx_data = kzalloc(ipc->rx_data_max_size, GFP_KERNEL);
	if (ipc->msg->rx_data == NULL) {
		kfree(ipc->msg->tx_data);
		goto free_mem;
	}

	return 0;

free_mem:
	kfree(ipc->msg);
	return -ENOMEM;
}

static int tx_wait_done(struct sst_generic_ipc *ipc,
			void *rx_data, size_t *rx_bytes)
{
	unsigned long flags;
	int ret;

again:
	/* wait for DSP completion (in all cases atm inc pending) */
	ret = wait_for_completion_timeout(&ipc->complete,
					  msecs_to_jiffies(IPC_TIMEOUT_MSECS));

	if (!ret) {
		if (ipc->ops.is_dsp_busy && !ipc->ops.is_dsp_busy(ipc->dsp))
			/* real timeout */
			goto end;
		/*
		 * fw did its job, either notification or reply
		 * has been received - now wait until it's processed
		 */
		wait_for_completion_killable(&ipc->complete);
		ret = 1;
	}

	spin_lock_irqsave(&ipc->dsp->spinlock, flags);
	if (!ipc->response_processed) {
		/* reply delayed due to nofitication */
		reinit_completion(&ipc->complete);
		spin_unlock_irqrestore(&ipc->dsp->spinlock, flags);
		goto again;
	}
	spin_unlock_irqrestore(&ipc->dsp->spinlock, flags);

end:
	spin_lock_irqsave(&ipc->dsp->spinlock, flags);
	if (ret == 0) {
		if (ipc->ops.shim_dbg != NULL)
			ipc->ops.shim_dbg(ipc, "message timeout");

		ret = -ETIMEDOUT;
	} else {

		/* copy the data returned from DSP */
		if ((rx_bytes != NULL) &&
				(ipc->msg->rx_size > *rx_bytes)) {
			dev_err(ipc->dev, "rx size is more than expected\n");
			ret = -EINVAL;
			goto err;
		}

		if (rx_data) {
			if (rx_bytes != NULL)
				*rx_bytes = ipc->msg->rx_size;
			memcpy(rx_data, ipc->msg->rx_data, ipc->msg->rx_size);
		}
		ret = ipc->msg->errno;
	}
err:
	spin_unlock_irqrestore(&ipc->dsp->spinlock, flags);
	return ret;
}

static int ipc_tx_message(struct sst_generic_ipc *ipc, u64 header,
	void *tx_data, size_t tx_bytes, void *rx_data,
	size_t *rx_bytes, int wait)
{
	int ret = 0;
	struct ipc_message *msg;
	unsigned long flags;

	if (!ipc->msg) {
		dev_err(ipc->dev, "No msg allocated, invalid state\n");
		return -EINVAL;
	}

	mutex_lock(&ipc->mutex);
	spin_lock_irqsave(&ipc->dsp->spinlock, flags);

	msg = ipc->msg;

	msg->header = header;
	msg->tx_size = tx_bytes;

	if (!rx_bytes)
		msg->rx_size = 0;
	else
		msg->rx_size = *rx_bytes;

	msg->wait = wait;
	msg->errno = 0;
	ipc->pending = false;
	ipc->response_processed = false;
	reinit_completion(&ipc->complete);

	if ((tx_bytes) && (ipc->ops.tx_data_copy != NULL))
		ipc->ops.tx_data_copy(msg, tx_data, tx_bytes);

	if (ipc->ops.tx_msg != NULL)
		ipc->ops.tx_msg(ipc, ipc->msg);
        spin_unlock_irqrestore(&ipc->dsp->spinlock, flags);

	if (wait)
		ret = tx_wait_done(ipc, rx_data,
				rx_bytes);
	mutex_unlock(&ipc->mutex);
	return ret;
}

int sst_ipc_tx_message_wait(struct sst_generic_ipc *ipc, u64 header,
		void *tx_data, size_t tx_bytes, void *rx_data,
		size_t *rx_bytes)
{
	int ret;

	/*
	 * DSP maybe in lower power active state, so
	 * check if the DSP supports DSP lp On method
	 * if so invoke that before sending IPC
	 */
	if (ipc->ops.check_dsp_lp_on)
		if (ipc->ops.check_dsp_lp_on(ipc->dsp, true))
			return -EIO;

	ret = ipc_tx_message(ipc, header, tx_data, tx_bytes,
		rx_data, rx_bytes, 1);

	if (ipc->ops.check_dsp_lp_on)
		if (ipc->ops.check_dsp_lp_on(ipc->dsp, false))
			return -EIO;

	return ret;
}
EXPORT_SYMBOL_GPL(sst_ipc_tx_message_wait);

int sst_ipc_tx_message_nowait(struct sst_generic_ipc *ipc, u64 header,
	void *tx_data, size_t tx_bytes)
{
	return ipc_tx_message(ipc, header, tx_data, tx_bytes,
		NULL, NULL, 0);
}
EXPORT_SYMBOL_GPL(sst_ipc_tx_message_nowait);

int sst_ipc_tx_message_nopm(struct sst_generic_ipc *ipc, u64 header,
	void *tx_data, size_t tx_bytes, void *rx_data, size_t rx_bytes)
{
	return ipc_tx_message(ipc, header, tx_data, tx_bytes,
		rx_data, &rx_bytes, 1);
}
EXPORT_SYMBOL_GPL(sst_ipc_tx_message_nopm);

struct ipc_message *sst_ipc_reply_find_msg(struct sst_generic_ipc *ipc,
	u64 header)
{
	u64 mask;

	if (!ipc->msg) {
		dev_err(ipc->dev, "Received 0x%llx, but no ongoing communication\n",
			header);
		return NULL;
	}

	if (ipc->ops.reply_msg_match != NULL)
		header = ipc->ops.reply_msg_match(header, &mask);
	else
		mask = (u64)-1;

	if ((ipc->msg->header & mask) == header)
		return ipc->msg;

	return NULL;
}
EXPORT_SYMBOL_GPL(sst_ipc_reply_find_msg);

/* locks held by caller */
void sst_ipc_tx_msg_reply_complete(struct sst_generic_ipc *ipc,
	struct ipc_message *msg)
{
	if (msg->wait)
		complete(&ipc->complete);
}
EXPORT_SYMBOL_GPL(sst_ipc_tx_msg_reply_complete);

int sst_ipc_init(struct sst_generic_ipc *ipc)
{
	init_waitqueue_head(&ipc->wait_txq);
	mutex_init(&ipc->mutex);
	init_completion(&ipc->complete);

	return msg_init(ipc);
}
EXPORT_SYMBOL_GPL(sst_ipc_init);

void sst_ipc_fini(struct sst_generic_ipc *ipc)
{
	kfree(ipc->msg->tx_data);
	kfree(ipc->msg->rx_data);
	kfree(ipc->msg);
	ipc->msg = NULL;
}
EXPORT_SYMBOL_GPL(sst_ipc_fini);

/* Module information */
MODULE_AUTHOR("Jin Yao");
MODULE_DESCRIPTION("Intel SST IPC generic");
MODULE_LICENSE("GPL v2");
