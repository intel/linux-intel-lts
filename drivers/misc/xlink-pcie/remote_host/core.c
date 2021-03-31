// SPDX-License-Identifier: GPL-2.0-only
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#include <linux/atomic.h>
#include <linux/jiffies.h>

#include "pci.h"
#include "../common/core.h"
#include "../common/util.h"

#define RX_BD_HRTIMER_DELAY_USEC	(4000 * 1000) /* 4 sec delay */

static int intel_xpcie_map_dma(struct xpcie *xpcie, struct xpcie_buf_desc *bd,
			       int direction)
{
	struct xpcie_dev *xdev = container_of(xpcie, struct xpcie_dev, xpcie);
	struct device *dev = &xdev->pci->dev;

	bd->phys = dma_map_single(dev, bd->data, bd->length, direction);

	return dma_mapping_error(dev, bd->phys);
}

static void intel_xpcie_unmap_dma(struct xpcie *xpcie,
				  struct xpcie_buf_desc *bd,
				  int direction)
{
	struct xpcie_dev *xdev = container_of(xpcie, struct xpcie_dev, xpcie);
	struct device *dev = &xdev->pci->dev;

	dma_unmap_single(dev, bd->phys, bd->length, direction);
}

static void intel_xpcie_txrx_cleanup(struct xpcie *xpcie)
{
	struct xpcie_interface *inf = &xpcie->interfaces[0];
	struct xpcie_stream *tx = &xpcie->tx;
	struct xpcie_stream *rx = &xpcie->rx;
	struct xpcie_buf_desc *bd;
	int index;

	xpcie->stop_flag = true;
	xpcie->no_tx_buffer = false;
	inf->data_avail = true;
	wake_up_interruptible(&xpcie->tx_waitq);
	wake_up_interruptible(&inf->rx_waitq);
	mutex_lock(&xpcie->wlock);
	mutex_lock(&inf->rlock);

	if (tx->ddr) {
		for (index = 0; index < tx->pipe.ndesc; index++) {
			struct xpcie_transfer_desc *td = tx->pipe.tdr + index;

			bd = tx->ddr[index];
			if (bd) {
				intel_xpcie_unmap_dma(xpcie, bd, DMA_TO_DEVICE);
				intel_xpcie_free_tx_bd(xpcie, bd);
				intel_xpcie_set_td_address(td, 0);
				intel_xpcie_set_td_length(td, 0);
			}
		}
		kfree(tx->ddr);
	}

	if (rx->ddr) {
		for (index = 0; index < rx->pipe.ndesc; index++) {
			struct xpcie_transfer_desc *td = rx->pipe.tdr + index;

			bd = rx->ddr[index];
			if (bd) {
				intel_xpcie_unmap_dma(xpcie,
						      bd, DMA_FROM_DEVICE);
				intel_xpcie_free_rx_bd(xpcie, bd);
				intel_xpcie_set_td_address(td, 0);
				intel_xpcie_set_td_length(td, 0);
			}
		}
		kfree(rx->ddr);
	}

	intel_xpcie_list_cleanup(&xpcie->tx_pool);
	intel_xpcie_list_cleanup(&xpcie->rx_pool);

	mutex_unlock(&inf->rlock);
	mutex_unlock(&xpcie->wlock);
}

static int intel_xpcie_txrx_init(struct xpcie *xpcie,
				 struct xpcie_cap_txrx *cap)
{
	struct xpcie_stream *tx = &xpcie->tx;
	struct xpcie_stream *rx = &xpcie->rx;
	int tx_pool_size, rx_pool_size;
	struct xpcie_interface *inf;
	struct xpcie_buf_desc *bd;
	int rc, index, ndesc;

	inf = &xpcie->interfaces[0];

	xpcie->txrx = cap;
	xpcie->fragment_size = intel_xpcie_ioread32(&cap->fragment_size);
	xpcie->stop_flag = false;

	tx->pipe.ndesc = intel_xpcie_ioread32(&cap->tx.ndesc);
	tx->pipe.head = &cap->tx.head;
	tx->pipe.tail = &cap->tx.tail;
	tx->pipe.old = intel_xpcie_ioread32(&cap->tx.tail);
	tx->pipe.tdr = (struct xpcie_transfer_desc *)(xpcie->mmio +
				intel_xpcie_ioread32(&cap->tx.ring));

	tx->ddr = kcalloc(tx->pipe.ndesc, sizeof(struct xpcie_buf_desc *),
			  GFP_KERNEL);
	if (!tx->ddr) {
		rc = -ENOMEM;
		goto error;
	}

	rx->pipe.ndesc = intel_xpcie_ioread32(&cap->rx.ndesc);
	rx->pipe.head = &cap->rx.head;
	rx->pipe.tail = &cap->rx.tail;
	rx->pipe.old = intel_xpcie_ioread32(&cap->rx.head);
	rx->pipe.tdr = (struct xpcie_transfer_desc *)(xpcie->mmio +
				intel_xpcie_ioread32(&cap->rx.ring));

	rx->ddr = kcalloc(rx->pipe.ndesc, sizeof(struct xpcie_buf_desc *),
			  GFP_KERNEL);
	if (!rx->ddr) {
		rc = -ENOMEM;
		goto error;
	}

	intel_xpcie_list_init(&xpcie->rx_pool);
	rx_pool_size = roundup(SZ_32M, xpcie->fragment_size);
	ndesc = rx_pool_size / xpcie->fragment_size;

	for (index = 0; index < ndesc; index++) {
		bd = intel_xpcie_alloc_bd(xpcie->fragment_size);
		if (bd) {
			intel_xpcie_list_put(&xpcie->rx_pool, bd);
		} else {
			rc = -ENOMEM;
			goto error;
		}
	}

	intel_xpcie_list_init(&xpcie->tx_pool);
	tx_pool_size = roundup(SZ_32M, xpcie->fragment_size);
	ndesc = tx_pool_size / xpcie->fragment_size;

	for (index = 0; index < ndesc; index++) {
		bd = intel_xpcie_alloc_bd(xpcie->fragment_size);
		if (bd) {
			intel_xpcie_list_put(&xpcie->tx_pool, bd);
		} else {
			rc = -ENOMEM;
			goto error;
		}
	}

	for (index = 0; index < rx->pipe.ndesc; index++) {
		struct xpcie_transfer_desc *td = rx->pipe.tdr + index;

		bd = intel_xpcie_alloc_rx_bd(xpcie);
		if (!bd) {
			rc = -ENOMEM;
			goto error;
		}

		if (intel_xpcie_map_dma(xpcie, bd, DMA_FROM_DEVICE)) {
			dev_err(xpcie_to_dev(xpcie), "failed to map rx bd\n");
			rc = -ENOMEM;
			goto error;
		}

		rx->ddr[index] = bd;
		intel_xpcie_set_td_address(td, bd->phys);
		intel_xpcie_set_td_length(td, bd->length);
	}

	atomic_set(&inf->available_bd_cnt, 0);
	intel_xpcie_update_device_flwctl(xpcie, TO_DEVICE, RX_BD_COUNT, 0);

	return 0;

error:
	intel_xpcie_txrx_cleanup(xpcie);

	return rc;
}

static int intel_xpcie_discover_txrx(struct xpcie *xpcie)
{
	struct xpcie_cap_txrx *cap;
	int error;

	cap = intel_xpcie_cap_find(xpcie, 0, XPCIE_CAP_TXRX);
	if (cap)
		error = intel_xpcie_txrx_init(xpcie, cap);
	else
		error = -EIO;

	return error;
}

static void intel_xpcie_start_tx(struct xpcie *xpcie, unsigned long delay)
{
	queue_delayed_work(xpcie->tx_wq, &xpcie->tx_event, delay);
}

static void intel_xpcie_start_rx(struct xpcie *xpcie, unsigned long delay)
{
	queue_delayed_work(xpcie->rx_wq, &xpcie->rx_event, delay);
}

static enum
hrtimer_restart free_rx_bd_timer_cb(struct hrtimer *free_rx_bd_timer)
{
	struct xpcie *xpcie = container_of(free_rx_bd_timer,
					   struct xpcie, free_rx_bd_timer);
	struct xpcie_dev *xdev = container_of(xpcie,
					       struct xpcie_dev, xpcie);
	struct xpcie_interface *inf = &xpcie->interfaces[0];
	struct xpcie_stream *rx = &xpcie->rx;
	struct xpcie_buf_desc *bd;
	int count = 0;

	bd = (inf->partial_read) ?
				  inf->partial_read :
				  intel_xpcie_list_get(&inf->read);
	while (bd) {
		intel_xpcie_free_rx_bd(xpcie, bd);
		bd = intel_xpcie_list_get(&inf->read);
		count++;
	}
	inf->partial_read = bd;
	inf->data_avail = false;

	intel_xpcie_set_tdr_head(&rx->pipe, 0);
	intel_xpcie_set_tdr_tail(&rx->pipe, 0);
	intel_xpcie_set_doorbell(xpcie, FROM_DEVICE, DATA_SENT, 0);
	intel_xpcie_set_doorbell(xpcie, TO_DEVICE, DATA_RECEIVED, 1);
	intel_xpcie_pci_raise_irq(xdev, DATA_RECEIVED, 1);

	return HRTIMER_NORESTART;
}

static void intel_xpcie_rx_tasklet_func(unsigned long xpcie_ptr)
{
	unsigned long period_us = RX_BD_HRTIMER_DELAY_USEC;
	struct xpcie *xpcie = (struct xpcie *)xpcie_ptr;
	struct xpcie_buf_desc *bd, *replacement = NULL;
	struct xpcie_stream *rx = &xpcie->rx;
	struct xpcie_transfer_desc *td;
	u32 head, tail, ndesc, length;
	struct xpcie_interface *inf;
	struct xpcie_dev *xdev;
	u16 status, interface;
	ktime_t free_rx_bd_tp;
	int rc;

	xdev = container_of(xpcie, struct xpcie_dev, xpcie);
	inf = &xpcie->interfaces[0];

	if (intel_xpcie_get_device_status(xpcie) != XPCIE_STATUS_RUN)
		return;

	ndesc = rx->pipe.ndesc;
	tail = intel_xpcie_get_tdr_tail(&rx->pipe);
	head = intel_xpcie_get_tdr_head(&rx->pipe);

	while (head != tail) {
		td = rx->pipe.tdr + head;
		bd = rx->ddr[head];

		replacement = intel_xpcie_alloc_rx_bd(xpcie);
		if (!replacement) {
			if (!hrtimer_active(&xpcie->free_rx_bd_timer)) {
				free_rx_bd_tp =
					ktime_set(0, period_us * 1000ULL);
				hrtimer_start(&xpcie->free_rx_bd_timer,
					      free_rx_bd_tp, HRTIMER_MODE_REL);
			}
			break;
		}

		rc = intel_xpcie_map_dma(xpcie, replacement, DMA_FROM_DEVICE);
		if (rc) {
			dev_err(xpcie_to_dev(xpcie),
				"failed to map rx bd (%d)\n", rc);
			intel_xpcie_free_rx_bd(xpcie, replacement);
			break;
		}

		status = intel_xpcie_get_td_status(td);
		interface = intel_xpcie_get_td_interface(td);
		length = intel_xpcie_get_td_length(td);
		intel_xpcie_unmap_dma(xpcie, bd, DMA_FROM_DEVICE);

		if (unlikely(status != XPCIE_DESC_STATUS_SUCCESS) ||
		    unlikely(interface >= XPCIE_NUM_INTERFACES)) {
			dev_err(xpcie_to_dev(xpcie),
				"rx desc failure, status(%u), interface(%u)\n",
			status, interface);
			intel_xpcie_free_rx_bd(xpcie, bd);
		} else {
			bd->interface = interface;
			bd->length = length;
			bd->next = NULL;

			intel_xpcie_add_bd_to_interface(xpcie, bd);
			intel_xpcie_update_device_flwctl(xpcie,
							 TO_DEVICE,
							 RX_BD_COUNT,
					atomic_read(&inf->available_bd_cnt));
		}

		rx->ddr[head] = replacement;
		intel_xpcie_set_td_address(td, replacement->phys);
		intel_xpcie_set_td_length(td, replacement->length);
		head = XPCIE_CIRCULAR_INC(head, ndesc);
	}

	if (intel_xpcie_get_tdr_head(&rx->pipe) != head) {
		intel_xpcie_set_tdr_head(&rx->pipe, head);
		if (head != tail)
			intel_xpcie_pci_raise_irq(xdev,
						  PARTIAL_DATA_RECEIVED, 1);
		else
			intel_xpcie_pci_raise_irq(xdev, DATA_RECEIVED, 1);
	}
}

static void intel_xpcie_tx_event_handler(struct work_struct *work)
{
	struct xpcie *xpcie = container_of(work, struct xpcie, tx_event.work);
	struct xpcie_dev *xdev = container_of(xpcie, struct xpcie_dev, xpcie);
	struct xpcie_stream *tx = &xpcie->tx;
	struct xpcie_transfer_desc *td;
	u32 head, tail, old, ndesc;
	struct xpcie_buf_desc *bd;
	size_t bytes, buffers;
	u16 status;

	if (intel_xpcie_get_device_status(xpcie) != XPCIE_STATUS_RUN)
		return;

	ndesc = tx->pipe.ndesc;
	old = tx->pipe.old;
	tail = intel_xpcie_get_tdr_tail(&tx->pipe);
	head = intel_xpcie_get_tdr_head(&tx->pipe);

	/* clean old entries first */
	while (old != head) {
		bd = tx->ddr[old];
		td = tx->pipe.tdr + old;
		status = intel_xpcie_get_td_status(td);
		if (status != XPCIE_DESC_STATUS_SUCCESS)
			dev_err(xpcie_to_dev(xpcie),
				"detected tx desc failure (%u)\n", status);

		intel_xpcie_unmap_dma(xpcie, bd, DMA_TO_DEVICE);
		intel_xpcie_free_tx_bd(xpcie, bd);
		tx->ddr[old] = NULL;
		old = XPCIE_CIRCULAR_INC(old, ndesc);
	}
	tx->pipe.old = old;

	/* add new entries */
	while (XPCIE_CIRCULAR_INC(tail, ndesc) != head) {
		bd = intel_xpcie_list_get(&xpcie->write);
		if (!bd)
			break;

		td = tx->pipe.tdr + tail;

		if (intel_xpcie_map_dma(xpcie, bd, DMA_TO_DEVICE)) {
			dev_err(xpcie_to_dev(xpcie),
				"dma mapping error bd addr %p, size %zu\n",
				bd->data, bd->length);
			break;
		}

		tx->ddr[tail] = bd;
		intel_xpcie_set_td_address(td, bd->phys);
		intel_xpcie_set_td_length(td, bd->length);
		intel_xpcie_set_td_interface(td, bd->interface);
		intel_xpcie_set_td_status(td, XPCIE_DESC_STATUS_ERROR);

		tail = XPCIE_CIRCULAR_INC(tail, ndesc);
	}

	if (intel_xpcie_get_tdr_tail(&tx->pipe) != tail) {
		intel_xpcie_set_tdr_tail(&tx->pipe, tail);
		intel_xpcie_pci_raise_irq(xdev, DATA_SENT, 1);
	}

	intel_xpcie_list_info(&xpcie->write, &bytes, &buffers);
	if (buffers)
		xpcie->tx_pending = true;
	else
		xpcie->tx_pending = false;
}

static irqreturn_t intel_xpcie_interrupt(int irq, void *args)
{
	struct xpcie_dev *xdev = args;
	struct xpcie *xpcie;

	xpcie = &xdev->xpcie;

	if (intel_xpcie_get_doorbell(xpcie, FROM_DEVICE, DATA_SENT)) {
		intel_xpcie_set_doorbell(xpcie, FROM_DEVICE, DATA_SENT, 0);
		tasklet_schedule(&xpcie->rx_tasklet);
	}
	if (intel_xpcie_get_doorbell(xpcie, FROM_DEVICE, DATA_RECEIVED)) {
		intel_xpcie_set_doorbell(xpcie, FROM_DEVICE, DATA_RECEIVED, 0);
		if (xpcie->tx_pending)
			intel_xpcie_start_tx(xpcie, 0);
	}

	return IRQ_HANDLED;
}

static int intel_xpcie_events_init(struct xpcie *xpcie)
{
	xpcie->rx_wq = alloc_ordered_workqueue(XPCIE_DRIVER_NAME,
					       WQ_MEM_RECLAIM | WQ_HIGHPRI);
	if (!xpcie->rx_wq) {
		dev_err(xpcie_to_dev(xpcie), "failed to allocate workqueue\n");
		return -ENOMEM;
	}

	xpcie->tx_wq = alloc_ordered_workqueue(XPCIE_DRIVER_NAME,
					       WQ_MEM_RECLAIM | WQ_HIGHPRI);
	if (!xpcie->tx_wq) {
		dev_err(xpcie_to_dev(xpcie), "failed to allocate workqueue\n");
		destroy_workqueue(xpcie->rx_wq);
		return -ENOMEM;
	}

	tasklet_init(&xpcie->rx_tasklet, intel_xpcie_rx_tasklet_func,
		     (uintptr_t)xpcie);
	INIT_DELAYED_WORK(&xpcie->tx_event, intel_xpcie_tx_event_handler);

	hrtimer_init(&xpcie->free_rx_bd_timer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	xpcie->free_rx_bd_timer.function = free_rx_bd_timer_cb;

	return 0;
}

static void intel_xpcie_events_cleanup(struct xpcie *xpcie)
{
	tasklet_kill(&xpcie->rx_tasklet);
	cancel_delayed_work_sync(&xpcie->tx_event);

	destroy_workqueue(xpcie->rx_wq);
	destroy_workqueue(xpcie->tx_wq);

	hrtimer_cancel(&xpcie->free_rx_bd_timer);
}

int intel_xpcie_core_init(struct xpcie *xpcie)
{
	struct xpcie_dev *xdev = container_of(xpcie, struct xpcie_dev, xpcie);
	int status, rc;

	status = intel_xpcie_get_device_status(xpcie);
	if (status != XPCIE_STATUS_RUN) {
		dev_err(&xdev->pci->dev,
			"device status not RUNNING (%d)\n", status);
		rc = -EBUSY;
		return rc;
	}

	if (intel_xpcie_ioread8(xpcie->mmio + XPCIE_MMIO_LEGACY_A0))
		xpcie->legacy_a0 = true;

	rc = intel_xpcie_events_init(xpcie);
	if (rc)
		return rc;

	rc = intel_xpcie_discover_txrx(xpcie);
	if (rc)
		goto error_txrx;

	intel_xpcie_interfaces_init(xpcie);

	rc = intel_xpcie_pci_register_irq(xdev, &intel_xpcie_interrupt);
	if (rc)
		goto error_txrx;

	intel_xpcie_set_host_status(xpcie, XPCIE_STATUS_RUN);

	return 0;

error_txrx:
	intel_xpcie_events_cleanup(xpcie);
	intel_xpcie_set_host_status(xpcie, XPCIE_STATUS_ERROR);

	return rc;
}

void intel_xpcie_core_cleanup(struct xpcie *xpcie)
{
	if (xpcie->status == XPCIE_STATUS_RUN) {
		intel_xpcie_set_host_status(xpcie, XPCIE_STATUS_UNINIT);
		intel_xpcie_events_cleanup(xpcie);
		intel_xpcie_interfaces_cleanup(xpcie);
		intel_xpcie_txrx_cleanup(xpcie);
	}
}

int intel_xpcie_core_read(struct xpcie *xpcie, void *buffer, size_t *length,
			  uint32_t timeout_ms)
{
	long jiffies_timeout = (long)msecs_to_jiffies(timeout_ms);
	struct xpcie_interface *inf = &xpcie->interfaces[0];
	unsigned long jiffies_start = jiffies;
	struct xpcie_buf_desc *bd;
	size_t remaining, len;
	long jiffies_passed = 0;
	int bd_val;
	int ret;

	if (*length == 0)
		return -EINVAL;

	if (xpcie->status != XPCIE_STATUS_RUN)
		return -ENODEV;

	len = *length;
	remaining = len;
	*length = 0;

	do {
		if (!inf->partial_read &&
		    (!intel_xpcie_list_empty(&inf->read))) {
			if (timeout_ms == 0) {
				ret =
			wait_event_interruptible(inf->rx_waitq,
						 (intel_xpcie_list_empty(&inf->read) ||
						  atomic_read(&inf->available_bd_cnt)));
			} else {
				ret =
			wait_event_interruptible_timeout(inf->rx_waitq,
							 (intel_xpcie_list_empty(&inf->read) ||
							 atomic_read(&inf->available_bd_cnt)),
							 jiffies_timeout - jiffies_passed);
				if (ret == 0)
					return -ETIME;
			}
			if (ret < 0 || xpcie->stop_flag)
				return -EINTR;
		}

		bd = (inf->partial_read) ? inf->partial_read :
					   intel_xpcie_list_get(&inf->read);
		while (remaining && bd) {
			size_t bcopy;

			bcopy = min(remaining, bd->length);
			memcpy(buffer, bd->data, bcopy);

			buffer += bcopy;
			remaining -= bcopy;
			bd->data += bcopy;
			bd->length -= bcopy;

			if (bd->length == 0) {
				intel_xpcie_free_rx_bd(xpcie, bd);
				bd = intel_xpcie_list_get(&inf->read);

				bd_val =
				atomic_dec_return(&inf->available_bd_cnt);
				intel_xpcie_update_device_flwctl(xpcie,
								 TO_DEVICE,
								 RX_BD_COUNT,
								 bd_val);

				if (hrtimer_active(&xpcie->free_rx_bd_timer)) {
					hrtimer_cancel
						(&xpcie->free_rx_bd_timer);
				}
			}
		}

		/* save for next time */
		inf->partial_read = bd;

		*length = len - remaining;

		jiffies_passed = (long)jiffies - (long)jiffies_start;
	} while (remaining > 0 && (jiffies_passed < jiffies_timeout ||
				   timeout_ms == 0));
	return 0;
}

int intel_xpcie_core_write(struct xpcie *xpcie, void *buffer, size_t *length,
			   uint32_t timeout_ms)
{
	long jiffies_timeout = (long)msecs_to_jiffies(timeout_ms);
	struct xpcie_interface *inf = &xpcie->interfaces[0];
	unsigned long jiffies_start = jiffies;
	struct xpcie_buf_desc *bd, *head;
	long jiffies_passed = 0;
	size_t remaining, len;
	int ret;

	if (*length == 0)
		return -EINVAL;

	if (xpcie->status != XPCIE_STATUS_RUN)
		return -ENODEV;

	len = *length;
	remaining = len;
	*length = 0;

	ret = mutex_lock_interruptible(&xpcie->wlock);
	if (ret < 0)
		return -EINTR;

	do {
		bd = intel_xpcie_alloc_tx_bd(xpcie);
		head = bd;
		while (!head) {
			mutex_unlock(&xpcie->wlock);
			if (timeout_ms == 0) {
				ret =
				wait_event_interruptible(xpcie->tx_waitq,
							 !xpcie->no_tx_buffer);
			} else {
				ret =
			wait_event_interruptible_timeout(xpcie->tx_waitq,
							 !xpcie->no_tx_buffer,
							 jiffies_timeout -
							 jiffies_passed);
				if (ret == 0)
					return -ETIME;
			}
			if (ret < 0 || xpcie->stop_flag)
				return -EINTR;

			ret = mutex_lock_interruptible(&xpcie->wlock);
			if (ret < 0)
				return -EINTR;

			bd = intel_xpcie_alloc_tx_bd(xpcie);
			head = bd;
		}

		while (remaining && bd) {
			size_t bcopy;

			bcopy = min(bd->length, remaining);
			memcpy(bd->data, buffer, bcopy);

			buffer += bcopy;
			remaining -= bcopy;
			bd->length = bcopy;
			bd->interface = inf->id;

			if (remaining) {
				bd->next = intel_xpcie_alloc_tx_bd(xpcie);
				bd = bd->next;
			}
		}

		intel_xpcie_list_put(&xpcie->write, head);
		intel_xpcie_start_tx(xpcie, 0);

		*length = len - remaining;

		jiffies_passed = (long)jiffies - (long)jiffies_start;
	} while (remaining > 0 && (jiffies_passed < jiffies_timeout ||
				   timeout_ms == 0));

	mutex_unlock(&xpcie->wlock);

	return 0;
}
