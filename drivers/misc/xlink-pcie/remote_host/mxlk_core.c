// SPDX-License-Identifier: GPL-2.0-only
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#include <linux/uaccess.h>
#include <linux/delay.h>

#include "mxlk_pci.h"
#include "../common/mxlk_core.h"
#include "../common/mxlk_util.h"
#include "../common/mxlk_capabilities.h"

#define MXLK_CIRCULAR_INC(val, max) (((val) + 1) & (max - 1))

static int rx_pool_size = SZ_32M;
module_param(rx_pool_size, int, 0664);
MODULE_PARM_DESC(rx_pool_size, "receive pool size (default 32 MiB)");

static int tx_pool_size = SZ_32M;
module_param(tx_pool_size, int, 0664);
MODULE_PARM_DESC(tx_pool_size, "transmit pool size (default 32 MiB)");

static int mxlk_version_check(struct mxlk *mxlk)
{
	struct mxlk_version version;

	memcpy_fromio(&version, &mxlk->mmio->version, sizeof(version));

	dev_info(mxlk_to_dev(mxlk), "ver: device %u.%u.%u, host %u.%u.%u\n",
		 version.major, version.minor, version.build,
		 MXLK_VERSION_MAJOR, MXLK_VERSION_MINOR, MXLK_VERSION_BUILD);

	if (ioread8(&mxlk->mmio->legacy_a0))
		mxlk->legacy_a0 = true;

	return 0;
}

static int mxlk_map_dma(struct mxlk *mxlk, struct mxlk_buf_desc *bd,
			int direction)
{
	struct mxlk_pcie *xdev = container_of(mxlk, struct mxlk_pcie, mxlk);
	struct device *dev = &xdev->pci->dev;

	bd->phys = dma_map_single(dev, bd->data, bd->length, direction);

	return dma_mapping_error(dev, bd->phys);
}

static void mxlk_unmap_dma(struct mxlk *mxlk, struct mxlk_buf_desc *bd,
			   int direction)
{
	struct mxlk_pcie *xdev = container_of(mxlk, struct mxlk_pcie, mxlk);
	struct device *dev = &xdev->pci->dev;

	dma_unmap_single(dev, bd->phys, bd->length, direction);
}

static void mxlk_txrx_cleanup(struct mxlk *mxlk)
{
	int index;
	struct mxlk_buf_desc *bd;
	struct mxlk_stream *tx = &mxlk->tx;
	struct mxlk_stream *rx = &mxlk->rx;
	struct mxlk_interface *inf = &mxlk->interfaces[0];

	mxlk->stop_flag = true;
	mxlk->no_tx_buffer = false;
	inf->data_available = true;
	wake_up_interruptible(&mxlk->tx_waitqueue);
	wake_up_interruptible(&inf->rx_waitqueue);
	mutex_lock(&mxlk->wlock);
	mutex_lock(&inf->rlock);

	if (tx->ddr) {
		for (index = 0; index < tx->pipe.ndesc; index++) {
			struct mxlk_transfer_desc *td = tx->pipe.tdr + index;

			bd = tx->ddr[index];
			if (bd) {
				mxlk_unmap_dma(mxlk, bd, DMA_TO_DEVICE);
				mxlk_free_tx_bd(mxlk, bd);
				mxlk_set_td_address(td, 0);
				mxlk_set_td_length(td, 0);
			}
		}
		kfree(tx->ddr);
	}

	if (rx->ddr) {
		for (index = 0; index < rx->pipe.ndesc; index++) {
			struct mxlk_transfer_desc *td = rx->pipe.tdr + index;

			bd = rx->ddr[index];
			if (bd) {
				mxlk_unmap_dma(mxlk, bd, DMA_FROM_DEVICE);
				mxlk_free_rx_bd(mxlk, bd);
				mxlk_set_td_address(td, 0);
				mxlk_set_td_length(td, 0);
			}
		}
		kfree(rx->ddr);
	}

	mxlk_list_cleanup(&mxlk->tx_pool);
	mxlk_list_cleanup(&mxlk->rx_pool);

	mutex_unlock(&inf->rlock);
	mutex_unlock(&mxlk->wlock);
}

static int mxlk_txrx_init(struct mxlk *mxlk, struct mxlk_cap_txrx *cap)
{
	int rc;
	int index;
	int ndesc;
	struct mxlk_buf_desc *bd;
	struct mxlk_stream *tx = &mxlk->tx;
	struct mxlk_stream *rx = &mxlk->rx;

	mxlk->txrx = cap;
	mxlk->fragment_size = ioread32(&cap->fragment_size);
	mxlk->stop_flag = false;

	tx->pipe.ndesc = ioread32(&cap->tx.ndesc);
	tx->pipe.head = &cap->tx.head;
	tx->pipe.tail = &cap->tx.tail;
	tx->pipe.old = ioread32(&cap->tx.tail);
	tx->pipe.tdr = (void __iomem *)mxlk->mmio + ioread32(&cap->tx.ring);

	tx->ddr = kcalloc(tx->pipe.ndesc, sizeof(struct mxlk_buf_desc *),
			  GFP_KERNEL);
	if (!tx->ddr) {
		rc = -ENOMEM;
		goto error;
	}

	rx->pipe.ndesc = ioread32(&cap->rx.ndesc);
	rx->pipe.head = &cap->rx.head;
	rx->pipe.tail = &cap->rx.tail;
	rx->pipe.old = ioread32(&cap->rx.head);
	rx->pipe.tdr = (void __iomem *)mxlk->mmio + ioread32(&cap->rx.ring);

	rx->ddr = kcalloc(rx->pipe.ndesc, sizeof(struct mxlk_buf_desc *),
			  GFP_KERNEL);
	if (!rx->ddr) {
		rc = -ENOMEM;
		goto error;
	}

	mxlk_list_init(&mxlk->rx_pool);
	rx_pool_size = roundup(rx_pool_size, mxlk->fragment_size);
	ndesc = rx_pool_size / mxlk->fragment_size;

	for (index = 0; index < ndesc; index++) {
		bd = mxlk_alloc_bd(mxlk->fragment_size);
		if (bd) {
			mxlk_list_put(&mxlk->rx_pool, bd);
		} else {
			rc = -ENOMEM;
			goto error;
		}
	}

	mxlk_list_init(&mxlk->tx_pool);
	tx_pool_size = roundup(tx_pool_size, mxlk->fragment_size);
	ndesc = tx_pool_size / mxlk->fragment_size;

	for (index = 0; index < ndesc; index++) {
		bd = mxlk_alloc_bd(mxlk->fragment_size);
		if (bd) {
			mxlk_list_put(&mxlk->tx_pool, bd);
		} else {
			rc = -ENOMEM;
			goto error;
		}
	}

	for (index = 0; index < rx->pipe.ndesc; index++) {
		struct mxlk_transfer_desc *td = rx->pipe.tdr + index;

		bd = mxlk_alloc_rx_bd(mxlk);
		if (!bd) {
			rc = -ENOMEM;
			goto error;
		}

		if (mxlk_map_dma(mxlk, bd, DMA_FROM_DEVICE)) {
			dev_err(mxlk_to_dev(mxlk), "failed to map rx bd\n");
			rc = -ENOMEM;
			goto error;
		}

		rx->ddr[index] = bd;
		mxlk_set_td_address(td, bd->phys);
		mxlk_set_td_length(td, bd->length);
	}

	return 0;

error:
	mxlk_txrx_cleanup(mxlk);

	return rc;
}

static int mxlk_discover_txrx(struct mxlk *mxlk)
{
	int error;
	struct mxlk_cap_txrx *cap;

	cap = mxlk_cap_find(mxlk, 0, MXLK_CAP_TXRX);
	if (cap)
		error = mxlk_txrx_init(mxlk, cap);
	else
		error = -EIO;

	return error;
}

static void mxlk_start_tx(struct mxlk *mxlk, unsigned long delay)
{
	queue_delayed_work(mxlk->tx_wq, &mxlk->tx_event, delay);
}

static void mxlk_start_rx(struct mxlk *mxlk, unsigned long delay)
{
	queue_delayed_work(mxlk->rx_wq, &mxlk->rx_event, delay);
}

static void mxlk_rx_event_handler(struct work_struct *work)
{
	struct mxlk *mxlk = container_of(work, struct mxlk, rx_event.work);

	int rc;
	struct mxlk_pcie *xdev = container_of(mxlk, struct mxlk_pcie, mxlk);
	u16 status, interface;
	u32 head, tail, ndesc, length;
	struct mxlk_stream *rx = &mxlk->rx;
	struct mxlk_buf_desc *bd, *replacement = NULL;
	struct mxlk_transfer_desc *td;
	unsigned long delay = msecs_to_jiffies(1);

	mxlk_debug_incr(mxlk, &mxlk->stats.rx_event_runs, 1);

	if (mxlk_get_device_status(mxlk) != MXLK_STATUS_RUN)
		return;

	ndesc = rx->pipe.ndesc;
	tail = mxlk_get_tdr_tail(&rx->pipe);
	head = mxlk_get_tdr_head(&rx->pipe);

	while (head != tail) {
		td = rx->pipe.tdr + head;
		bd = rx->ddr[head];

		replacement = mxlk_alloc_rx_bd(mxlk);
		if (!replacement) {
			delay = msecs_to_jiffies(20);
			break;
		}

		rc = mxlk_map_dma(mxlk, replacement, DMA_FROM_DEVICE);
		if (rc) {
			dev_err(mxlk_to_dev(mxlk),
				"failed to map rx bd (%d)\n", rc);
			mxlk_free_rx_bd(mxlk, replacement);
			break;
		}

		status = mxlk_get_td_status(td);
		interface = mxlk_get_td_interface(td);
		length = mxlk_get_td_length(td);
		mxlk_unmap_dma(mxlk, bd, DMA_FROM_DEVICE);

		if (unlikely(status != MXLK_DESC_STATUS_SUCCESS) ||
		    unlikely(interface >= MXLK_NUM_INTERFACES)) {
			dev_err(mxlk_to_dev(mxlk),
			"detected rx desc failure, status(%u), interface(%u)\n",
			status, interface);
			mxlk_free_rx_bd(mxlk, bd);
		} else {
			bd->interface = interface;
			bd->length = length;
			bd->next = NULL;

			mxlk_debug_incr(mxlk, &mxlk->stats.rx_krn.cnts, 1);
			mxlk_debug_incr(mxlk, &mxlk->stats.rx_krn.bytes,
					bd->length);

			mxlk_add_bd_to_interface(mxlk, bd);
		}

		rx->ddr[head] = replacement;
		mxlk_set_td_address(td, replacement->phys);
		mxlk_set_td_length(td, replacement->length);
		head = MXLK_CIRCULAR_INC(head, ndesc);
	}

	if (mxlk_get_tdr_head(&rx->pipe) != head) {
		mxlk_set_tdr_head(&rx->pipe, head);
		mxlk_pci_raise_irq(xdev, DATA_RECEIVED, 1);
	}

	if (!replacement)
		mxlk_start_rx(mxlk, delay);
}

static void mxlk_tx_event_handler(struct work_struct *work)
{
	struct mxlk *mxlk = container_of(work, struct mxlk, tx_event.work);

	u16 status;
	struct mxlk_pcie *xdev = container_of(mxlk, struct mxlk_pcie, mxlk);
	u32 head, tail, old, ndesc;
	struct mxlk_stream *tx = &mxlk->tx;
	struct mxlk_buf_desc *bd;
	struct mxlk_transfer_desc *td;
	size_t bytes, buffers;

	mxlk_debug_incr(mxlk, &mxlk->stats.tx_event_runs, 1);

	if (mxlk_get_device_status(mxlk) != MXLK_STATUS_RUN)
		return;

	ndesc = tx->pipe.ndesc;
	old = tx->pipe.old;
	tail = mxlk_get_tdr_tail(&tx->pipe);
	head = mxlk_get_tdr_head(&tx->pipe);

	// clean old entries first
	while (old != head) {
		bd = tx->ddr[old];
		td = tx->pipe.tdr + old;
		status = mxlk_get_td_status(td);
		if (status != MXLK_DESC_STATUS_SUCCESS)
			dev_err(mxlk_to_dev(mxlk),
				"detected tx desc failure (%u)\n", status);

		mxlk_unmap_dma(mxlk, bd, DMA_TO_DEVICE);
		mxlk_free_tx_bd(mxlk, bd);
		tx->ddr[old] = NULL;
		old = MXLK_CIRCULAR_INC(old, ndesc);
	}
	tx->pipe.old = old;

	// add new entries
	while (MXLK_CIRCULAR_INC(tail, ndesc) != head) {
		bd = mxlk_list_get(&mxlk->write);
		if (!bd)
			break;

		td = tx->pipe.tdr + tail;

		if (mxlk_map_dma(mxlk, bd, DMA_TO_DEVICE)) {
			dev_err(mxlk_to_dev(mxlk),
				"dma mapping error bd addr %p, size %zu\n",
				bd->data, bd->length);
			break;
		}

		tx->ddr[tail] = bd;
		mxlk_set_td_address(td, bd->phys);
		mxlk_set_td_length(td, bd->length);
		mxlk_set_td_interface(td, bd->interface);
		mxlk_set_td_status(td, MXLK_DESC_STATUS_ERROR);

		mxlk_debug_incr(mxlk, &mxlk->stats.tx_krn.cnts, 1);
		mxlk_debug_incr(mxlk, &mxlk->stats.tx_krn.bytes, bd->length);

		tail = MXLK_CIRCULAR_INC(tail, ndesc);
	}

	if (mxlk_get_tdr_tail(&tx->pipe) != tail) {
		mxlk_set_tdr_tail(&tx->pipe, tail);
		mxlk_pci_raise_irq(xdev, DATA_SENT, 1);
		mxlk_debug_incr(mxlk, &mxlk->stats.send_ints, 1);
	}

	mxlk_list_info(&mxlk->write, &bytes, &buffers);
	if (buffers)
		mxlk->tx_pending = true;
	else
		mxlk->tx_pending = false;
}

static irqreturn_t mxlk_interrupt(int irq, void *args)
{
	struct mxlk_pcie *xdev = args;
	struct mxlk *mxlk = &xdev->mxlk;

	if (mxlk_get_doorbell(mxlk, FROM_DEVICE, DATA_SENT)) {
		mxlk_set_doorbell(mxlk, FROM_DEVICE, DATA_SENT, 0);
		mxlk_start_rx(mxlk, 0);

		mxlk_debug_incr(mxlk, &xdev->mxlk.stats.interrupts, 1);
	}
	if (mxlk_get_doorbell(mxlk, FROM_DEVICE, DATA_RECEIVED)) {
		mxlk_set_doorbell(mxlk, FROM_DEVICE, DATA_RECEIVED, 0);
		if (mxlk->tx_pending)
			mxlk_start_tx(mxlk, 0);
	}

	return IRQ_HANDLED;
}

static int mxlk_events_init(struct mxlk *mxlk)
{
	mxlk->rx_wq = alloc_ordered_workqueue(MXLK_DRIVER_NAME,
					      WQ_MEM_RECLAIM | WQ_HIGHPRI);
	if (!mxlk->rx_wq) {
		dev_err(mxlk_to_dev(mxlk), "failed to allocate workqueue\n");
		return -ENOMEM;
	}

	mxlk->tx_wq = alloc_ordered_workqueue(MXLK_DRIVER_NAME,
					      WQ_MEM_RECLAIM | WQ_HIGHPRI);
	if (!mxlk->tx_wq) {
		dev_err(mxlk_to_dev(mxlk), "failed to allocate workqueue\n");
		destroy_workqueue(mxlk->rx_wq);
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&mxlk->rx_event, mxlk_rx_event_handler);
	INIT_DELAYED_WORK(&mxlk->tx_event, mxlk_tx_event_handler);

	return 0;
}

static void mxlk_events_cleanup(struct mxlk *mxlk)
{
	cancel_delayed_work_sync(&mxlk->rx_event);
	cancel_delayed_work_sync(&mxlk->tx_event);

	destroy_workqueue(mxlk->rx_wq);
	destroy_workqueue(mxlk->tx_wq);
}

int mxlk_core_init(struct mxlk *mxlk)
{
	int rc;
	int status;
	struct mxlk_pcie *xdev = container_of(mxlk, struct mxlk_pcie, mxlk);

	status = mxlk_get_device_status(mxlk);
	if (status != MXLK_STATUS_RUN) {
		dev_err(&xdev->pci->dev,
			"device status not RUNNING (%d)\n", status);
		rc = -EBUSY;
		return rc;
	}

	mxlk_version_check(mxlk);

	rc = mxlk_events_init(mxlk);
	if (rc)
		return rc;

	rc = mxlk_discover_txrx(mxlk);
	if (rc)
		goto error_txrx;

	mxlk_interfaces_init(mxlk);

	rc = mxlk_pci_register_irq(xdev, &mxlk_interrupt);
	if (rc)
		goto error_txrx;

	mxlk_set_host_status(mxlk, MXLK_STATUS_RUN);

	return 0;

error_txrx:
	mxlk_events_cleanup(mxlk);
	mxlk_set_host_status(mxlk, MXLK_STATUS_ERROR);

	return rc;
}

void mxlk_core_cleanup(struct mxlk *mxlk)
{
	if (mxlk->status == MXLK_STATUS_RUN) {
		mxlk_set_host_status(mxlk, MXLK_STATUS_UNINIT);
		mxlk_events_cleanup(mxlk);
		mxlk_interfaces_cleanup(mxlk);
		mxlk_txrx_cleanup(mxlk);
	}
}

int mxlk_core_read(struct mxlk *mxlk, void *buffer, size_t *length,
		   uint32_t timeout_ms)
{
	int ret = 0;
	struct mxlk_interface *inf = &mxlk->interfaces[0];
	size_t len = *length;
	size_t remaining = len;
	struct mxlk_buf_desc *bd;
	unsigned long jiffies_start = jiffies;
	long jiffies_passed = 0;
	long jiffies_timeout = (long)msecs_to_jiffies(timeout_ms);

	*length = 0;
	if (len == 0)
		return -EINVAL;

	if (mxlk->status != MXLK_STATUS_RUN)
		return -ENODEV;

	mxlk_debug_incr(mxlk, &mxlk->stats.rx_usr.cnts, 1);

	ret = mutex_lock_interruptible(&inf->rlock);
	if (ret < 0)
		return -EINTR;

	do {
		while (!inf->data_available) {
			mutex_unlock(&inf->rlock);
			if (timeout_ms == 0) {
				ret = wait_event_interruptible(
					inf->rx_waitqueue, inf->data_available);
			} else {
				ret = wait_event_interruptible_timeout(
					inf->rx_waitqueue, inf->data_available,
					jiffies_timeout - jiffies_passed);
				if (ret == 0)
					return -ETIME;
			}
			if (ret < 0 || mxlk->stop_flag)
				return -EINTR;

			ret = mutex_lock_interruptible(&inf->rlock);
			if (ret < 0)
				return -EINTR;
		}

		bd = (inf->partial_read) ? inf->partial_read :
					   mxlk_list_get(&inf->read);
		while (remaining && bd) {
			size_t bcopy;

			bcopy = min(remaining, bd->length);
			memcpy(buffer, bd->data, bcopy);

			buffer += bcopy;
			remaining -= bcopy;
			bd->data += bcopy;
			bd->length -= bcopy;

			mxlk_debug_incr(mxlk, &mxlk->stats.rx_usr.bytes, bcopy);

			if (bd->length == 0) {
				mxlk_free_rx_bd(mxlk, bd);
				bd = mxlk_list_get(&inf->read);
			}
		}

		// save for next time
		inf->partial_read = bd;

		if (!bd)
			inf->data_available = false;

		*length = len - remaining;

		jiffies_passed = (long)jiffies - (long)jiffies_start;
	} while (remaining > 0 && (jiffies_passed < jiffies_timeout ||
				   timeout_ms == 0));

	mutex_unlock(&inf->rlock);

	return 0;
}

int mxlk_core_write(struct mxlk *mxlk, void *buffer, size_t *length,
		    uint32_t timeout_ms)
{
	int ret;
	size_t len = *length;
	size_t remaining = len;
	struct mxlk_interface *inf = &mxlk->interfaces[0];
	struct mxlk_buf_desc *bd, *head;
	unsigned long jiffies_start = jiffies;
	long jiffies_passed = 0;
	long jiffies_timeout = (long)msecs_to_jiffies(timeout_ms);

	*length = 0;
	if (len == 0)
		return -EINVAL;

	if (mxlk->status != MXLK_STATUS_RUN)
		return -ENODEV;

	mxlk_debug_incr(mxlk, &mxlk->stats.tx_usr.cnts, 1);

	ret = mutex_lock_interruptible(&mxlk->wlock);
	if (ret < 0)
		return -EINTR;

	do {
		bd = head = mxlk_alloc_tx_bd(mxlk);
		while (!head) {
			mutex_unlock(&mxlk->wlock);
			if (timeout_ms == 0) {
				ret = wait_event_interruptible(
					mxlk->tx_waitqueue,
					!mxlk->no_tx_buffer);
			} else {
				ret = wait_event_interruptible_timeout(
					mxlk->tx_waitqueue, !mxlk->no_tx_buffer,
					jiffies_timeout - jiffies_passed);
				if (ret == 0)
					return -ETIME;
			}
			if (ret < 0 || mxlk->stop_flag)
				return -EINTR;

			ret = mutex_lock_interruptible(&mxlk->wlock);
			if (ret < 0)
				return -EINTR;

			bd = head = mxlk_alloc_tx_bd(mxlk);
		}

		while (remaining && bd) {
			size_t bcopy;

			bcopy = min(bd->length, remaining);
			memcpy(bd->data, buffer, bcopy);

			buffer += bcopy;
			remaining -= bcopy;
			bd->length = bcopy;
			bd->interface = inf->id;

			mxlk_debug_incr(mxlk, &mxlk->stats.tx_usr.bytes, bcopy);

			if (remaining) {
				bd->next = mxlk_alloc_tx_bd(mxlk);
				bd = bd->next;
			}
		}

		mxlk_list_put(&mxlk->write, head);
		mxlk_start_tx(mxlk, 0);

		*length = len - remaining;

		jiffies_passed = (long)jiffies - (long)jiffies_start;
	} while (remaining > 0 && (jiffies_passed < jiffies_timeout ||
				   timeout_ms == 0));

	mutex_unlock(&mxlk->wlock);

	return 0;
}
