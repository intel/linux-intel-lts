// SPDX-License-Identifier: GPL-2.0-only
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#include <linux/of_reserved_mem.h>

#include "epf.h"
#include "core.h"
#include "util.h"

static struct xpcie *global_xpcie;

static struct xpcie *intel_xpcie_core_get_by_id(u32 sw_device_id)
{
	return (sw_device_id == xlink_sw_id) ? global_xpcie : NULL;
}

static int intel_xpcie_map_dma(struct xpcie *xpcie, struct xpcie_buf_desc *bd,
			       int direction)
{
	struct xpcie_epf *xpcie_epf = container_of(xpcie,
						   struct xpcie_epf, xpcie);
	struct pci_epf *epf = xpcie_epf->epf;
	struct device *dma_dev = epf->epc->dev.parent;

	bd->phys = dma_map_single(dma_dev, bd->data, bd->length, direction);

	return dma_mapping_error(dma_dev, bd->phys);
}

static void intel_xpcie_unmap_dma(struct xpcie *xpcie,
				  struct xpcie_buf_desc *bd, int direction)
{
	struct xpcie_epf *xpcie_epf = container_of(xpcie,
						   struct xpcie_epf, xpcie);
	struct pci_epf *epf = xpcie_epf->epf;
	struct device *dma_dev = epf->epc->dev.parent;

	dma_unmap_single(dma_dev, bd->phys, bd->length, direction);
}

static void intel_xpcie_set_cap_txrx(struct xpcie *xpcie)
{
	size_t tx_len = sizeof(struct xpcie_transfer_desc) *
				XPCIE_NUM_TX_DESCS;
	size_t rx_len = sizeof(struct xpcie_transfer_desc) *
				XPCIE_NUM_RX_DESCS;
	size_t hdr_len = sizeof(struct xpcie_cap_txrx);
	u32 start = sizeof(struct xpcie_mmio);
	struct xpcie_cap_txrx *cap;
	struct xpcie_cap_hdr *hdr;
	u16 next;

	next = (u16)(start + hdr_len + tx_len + rx_len);
	intel_xpcie_iowrite32(start, xpcie->mmio + XPCIE_MMIO_CAP_OFF);
	cap = (void *)xpcie->mmio + start;
	memset(cap, 0, sizeof(struct xpcie_cap_txrx));
	cap->hdr.id = XPCIE_CAP_TXRX;
	cap->hdr.next = next;
	cap->fragment_size = XPCIE_FRAGMENT_SIZE;
	cap->tx.ring = start + hdr_len;
	cap->tx.ndesc = XPCIE_NUM_TX_DESCS;
	cap->rx.ring = start + hdr_len + tx_len;
	cap->rx.ndesc = XPCIE_NUM_RX_DESCS;

	hdr = (struct xpcie_cap_hdr *)((void *)xpcie->mmio + next);
	hdr->id = XPCIE_CAP_NULL;
}

static void intel_xpcie_txrx_cleanup(struct xpcie *xpcie)
{
	struct xpcie_epf *xpcie_epf = container_of(xpcie,
						   struct xpcie_epf, xpcie);
	struct device *dma_dev = xpcie_epf->epf->epc->dev.parent;
	struct xpcie_interface *inf = &xpcie->interfaces[0];
	struct xpcie_stream *tx = &xpcie->tx;
	struct xpcie_stream *rx = &xpcie->rx;
	struct xpcie_transfer_desc *td;
	int index;

	xpcie->stop_flag = true;
	xpcie->no_tx_buffer = false;
	inf->data_avail = true;
	wake_up_interruptible(&xpcie->tx_waitq);
	wake_up_interruptible(&inf->rx_waitq);
	mutex_lock(&xpcie->wlock);
	mutex_lock(&inf->rlock);

	for (index = 0; index < rx->pipe.ndesc; index++) {
		td = rx->pipe.tdr + index;
		intel_xpcie_set_td_address(td, 0);
		intel_xpcie_set_td_length(td, 0);
	}
	for (index = 0; index < tx->pipe.ndesc; index++) {
		td = tx->pipe.tdr + index;
		intel_xpcie_set_td_address(td, 0);
		intel_xpcie_set_td_length(td, 0);
	}

	intel_xpcie_list_cleanup(&xpcie->tx_pool);
	intel_xpcie_list_cleanup(&xpcie->rx_pool);

	if (xpcie_epf->tx_virt) {
		dma_free_coherent(dma_dev, xpcie_epf->tx_size,
				  xpcie_epf->tx_virt, xpcie_epf->tx_phys);
	}

	mutex_unlock(&inf->rlock);
	mutex_unlock(&xpcie->wlock);
}

/*
 * The RX/TX are named for Remote Host, in Local Host
 * RX/TX is reversed.
 */
static int intel_xpcie_txrx_init(struct xpcie *xpcie,
				 struct xpcie_cap_txrx *cap)
{
	struct xpcie_epf *xpcie_epf = container_of(xpcie,
						   struct xpcie_epf, xpcie);
	struct device *dma_dev = xpcie_epf->epf->epc->dev.parent;
	struct xpcie_stream *tx = &xpcie->tx;
	struct xpcie_stream *rx = &xpcie->rx;
	int tx_pool_size, rx_pool_size;
	struct xpcie_buf_desc *bd;
	int index, ndesc, rc;

	xpcie->txrx = cap;
	xpcie->fragment_size = cap->fragment_size;
	xpcie->stop_flag = false;

	rx->pipe.ndesc = cap->tx.ndesc;
	rx->pipe.head = &cap->tx.head;
	rx->pipe.tail = &cap->tx.tail;
	rx->pipe.tdr = (void *)xpcie->mmio + cap->tx.ring;

	tx->pipe.ndesc = cap->rx.ndesc;
	tx->pipe.head = &cap->rx.head;
	tx->pipe.tail = &cap->rx.tail;
	tx->pipe.tdr = (void *)xpcie->mmio + cap->rx.ring;

	intel_xpcie_list_init(&xpcie->rx_pool);
	rx_pool_size = roundup(SZ_32M, xpcie->fragment_size);
	ndesc = rx_pool_size / xpcie->fragment_size;

	/* Initialize reserved memory resources */
	rc = of_reserved_mem_device_init(dma_dev);
	if (rc) {
		dev_err(dma_dev, "Could not get reserved memory\n");
		goto error;
	}

	for (index = 0; index < ndesc; index++) {
		bd = intel_xpcie_alloc_bd(xpcie->fragment_size);
		if (bd) {
			intel_xpcie_list_put(&xpcie->rx_pool, bd);
		} else {
			dev_err(xpcie_to_dev(xpcie),
				"failed to alloc all rx pool descriptors\n");
			goto error;
		}
	}

	intel_xpcie_list_init(&xpcie->tx_pool);
	tx_pool_size = roundup(SZ_32M, xpcie->fragment_size);
	ndesc = tx_pool_size / xpcie->fragment_size;

	xpcie_epf->tx_size = tx_pool_size;
	xpcie_epf->tx_virt = dma_alloc_coherent(dma_dev,
						xpcie_epf->tx_size,
						&xpcie_epf->tx_phys,
						GFP_KERNEL);
	if (!xpcie_epf->tx_virt)
		goto error;

	for (index = 0; index < ndesc; index++) {
		bd = intel_xpcie_alloc_bd_reuse(xpcie->fragment_size,
						xpcie_epf->tx_virt +
						(index *
						 xpcie->fragment_size),
						xpcie_epf->tx_phys +
						(index *
						 xpcie->fragment_size));
		if (bd) {
			intel_xpcie_list_put(&xpcie->tx_pool, bd);
		} else {
			dev_err(xpcie_to_dev(xpcie),
				"failed to alloc all tx pool descriptors\n");
			goto error;
		}
	}

	return 0;

error:
	intel_xpcie_txrx_cleanup(xpcie);

	return -ENOMEM;
}

static int intel_xpcie_discover_txrx(struct xpcie *xpcie)
{
	struct xpcie_cap_txrx *cap;
	int error;

	cap = intel_xpcie_cap_find(xpcie, 0, XPCIE_CAP_TXRX);
	if (cap) {
		error = intel_xpcie_txrx_init(xpcie, cap);
	} else {
		dev_err(xpcie_to_dev(xpcie), "xpcie txrx info not found\n");
		error = -EIO;
	}

	return error;
}

static void intel_xpcie_start_tx(struct xpcie *xpcie, unsigned long delay)
{
	/*
	 * Use only one WQ for both Rx and Tx
	 *
	 * Synchronous Read and Writes to DDR is found to result in memory
	 * mismatch errors in stability tests due to silicon bug in A0 SoC.
	 */
	if (xpcie->legacy_a0)
		queue_delayed_work(xpcie->rx_wq, &xpcie->tx_event, delay);
	else
		queue_delayed_work(xpcie->tx_wq, &xpcie->tx_event, delay);
}

static void intel_xpcie_start_rx(struct xpcie *xpcie, unsigned long delay)
{
	queue_delayed_work(xpcie->rx_wq, &xpcie->rx_event, delay);
}

static void intel_xpcie_rx_event_handler(struct work_struct *work)
{
	struct xpcie *xpcie = container_of(work, struct xpcie, rx_event.work);
	struct xpcie_epf *xpcie_epf = container_of(xpcie,
						   struct xpcie_epf, xpcie);
	struct xpcie_buf_desc *bd_head, *bd_tail, *bd;
	u32 head, tail, ndesc, length, initial_head;
	unsigned long delay = msecs_to_jiffies(1);
	struct xpcie_stream *rx = &xpcie->rx;
	int descs_num = 0, chan = 0, rc;
	struct xpcie_dma_ll_desc *desc;
	struct xpcie_transfer_desc *td;
	bool reset_work = false;
	u16 interface;
	u64 address;

	if (intel_xpcie_get_host_status(xpcie) != XPCIE_STATUS_RUN)
		return;

	bd_head = NULL;
	bd_tail = NULL;
	ndesc = rx->pipe.ndesc;
	tail = intel_xpcie_get_tdr_tail(&rx->pipe);
	initial_head = intel_xpcie_get_tdr_head(&rx->pipe);
	head = initial_head;

	while (head != tail) {
		td = rx->pipe.tdr + head;

		bd = intel_xpcie_alloc_rx_bd(xpcie);
		if (!bd) {
			reset_work = true;
			if (descs_num == 0) {
				delay = msecs_to_jiffies(10);
				goto task_exit;
			}
			break;
		}

		interface = intel_xpcie_get_td_interface(td);
		length = intel_xpcie_get_td_length(td);
		address = intel_xpcie_get_td_address(td);

		bd->length = length;
		bd->interface = interface;
		rc = intel_xpcie_map_dma(xpcie, bd, DMA_FROM_DEVICE);
		if (rc) {
			dev_err(xpcie_to_dev(xpcie),
				"failed to map rx bd (%d)\n", rc);
			intel_xpcie_free_rx_bd(xpcie, bd);
			break;
		}

		desc = &xpcie_epf->rx_desc_buf[chan].virt[descs_num++];
		desc->dma_transfer_size = length;
		desc->dst_addr = bd->phys;
		desc->src_addr = address;

		if (bd_head)
			bd_tail->next = bd;
		else
			bd_head = bd;
		bd_tail = bd;

		head = XPCIE_CIRCULAR_INC(head, ndesc);
	}

	if (descs_num == 0)
		goto task_exit;

	rc = intel_xpcie_copy_from_host_ll(xpcie, chan, descs_num);

	bd = bd_head;
	while (bd) {
		intel_xpcie_unmap_dma(xpcie, bd, DMA_FROM_DEVICE);
		bd = bd->next;
	}

	if (rc) {
		dev_err(xpcie_to_dev(xpcie),
			"failed to DMA from host (%d)\n", rc);
		intel_xpcie_free_rx_bd(xpcie, bd_head);
		delay = msecs_to_jiffies(5);
		reset_work = true;
		goto task_exit;
	}

	head = initial_head;
	bd = bd_head;
	while (bd) {
		td = rx->pipe.tdr + head;
		bd_head = bd_head->next;
		bd->next = NULL;

		if (likely(bd->interface < XPCIE_NUM_INTERFACES)) {
			intel_xpcie_set_td_status(td,
						  XPCIE_DESC_STATUS_SUCCESS);
			intel_xpcie_add_bd_to_interface(xpcie, bd);
		} else {
			dev_err(xpcie_to_dev(xpcie),
				"detected rx desc interface failure (%u)\n",
				bd->interface);
			intel_xpcie_set_td_status(td, XPCIE_DESC_STATUS_ERROR);
			intel_xpcie_free_rx_bd(xpcie, bd);
		}

		bd = bd_head;
		head = XPCIE_CIRCULAR_INC(head, ndesc);
	}

	if (head != initial_head) {
		intel_xpcie_set_tdr_head(&rx->pipe, head);
		intel_xpcie_raise_irq(xpcie, DATA_RECEIVED);
	}

task_exit:
	if (reset_work)
		intel_xpcie_start_rx(xpcie, delay);
}

static void intel_xpcie_tx_event_handler(struct work_struct *work)
{
	struct xpcie *xpcie = container_of(work, struct xpcie, tx_event.work);
	struct xpcie_epf *xpcie_epf = container_of(xpcie,
						   struct xpcie_epf, xpcie);
	struct xpcie_buf_desc *bd_head, *bd_tail, *bd;
	struct xpcie_stream *tx = &xpcie->tx;
	u32 head, tail, ndesc, initial_tail;
	struct xpcie_dma_ll_desc *desc;
	struct xpcie_transfer_desc *td;
	int descs_num = 0, chan = 0, rc;
	size_t buffers = 0, bytes = 0;
	u64 address;

	if (intel_xpcie_get_host_status(xpcie) != XPCIE_STATUS_RUN)
		return;

	bd_head = NULL;
	bd_tail = NULL;
	ndesc = tx->pipe.ndesc;
	initial_tail = intel_xpcie_get_tdr_tail(&tx->pipe);
	tail = initial_tail;
	head = intel_xpcie_get_tdr_head(&tx->pipe);

	/* add new entries */
	while (XPCIE_CIRCULAR_INC(tail, ndesc) != head) {
		bd = intel_xpcie_list_get(&xpcie->write);
		if (!bd)
			break;

		td = tx->pipe.tdr + tail;
		address = intel_xpcie_get_td_address(td);

		desc = &xpcie_epf->tx_desc_buf[chan].virt[descs_num++];
		desc->dma_transfer_size = bd->length;
		desc->src_addr = bd->phys;
		desc->dst_addr = address;

		if (bd_head)
			bd_tail->next = bd;
		else
			bd_head = bd;
		bd_tail = bd;

		tail = XPCIE_CIRCULAR_INC(tail, ndesc);
	}

	if (descs_num == 0)
		goto task_exit;

	rc = intel_xpcie_copy_to_host_ll(xpcie, chan, descs_num);

	tail = initial_tail;
	bd = bd_head;
	while (bd) {
		if (rc) {
			bd = bd->next;
			continue;
		}

		td = tx->pipe.tdr + tail;
		intel_xpcie_set_td_status(td, XPCIE_DESC_STATUS_SUCCESS);
		intel_xpcie_set_td_length(td, bd->length);
		intel_xpcie_set_td_interface(td, bd->interface);

		bd = bd->next;
		tail = XPCIE_CIRCULAR_INC(tail, ndesc);
	}

	if (rc) {
		dev_err(xpcie_to_dev(xpcie),
			"failed to DMA to host (%d)\n", rc);
		intel_xpcie_list_put_head(&xpcie->write, bd_head);
		return;
	}

	intel_xpcie_free_tx_bd(xpcie, bd_head);

	if (intel_xpcie_get_tdr_tail(&tx->pipe) != tail) {
		intel_xpcie_set_tdr_tail(&tx->pipe, tail);
		intel_xpcie_raise_irq(xpcie, DATA_SENT);
	}

task_exit:
	intel_xpcie_list_info(&xpcie->write, &bytes, &buffers);
	if (buffers) {
		xpcie->tx_pending = true;
		head = intel_xpcie_get_tdr_head(&tx->pipe);
		if (XPCIE_CIRCULAR_INC(tail, ndesc) != head)
			intel_xpcie_start_tx(xpcie, 0);
	} else {
		xpcie->tx_pending = false;
	}
}

static irqreturn_t intel_xpcie_core_irq_cb(int irq, void *args)
{
	struct xpcie *xpcie = args;

	if (intel_xpcie_get_doorbell(xpcie, TO_DEVICE, DATA_SENT)) {
		intel_xpcie_set_doorbell(xpcie, TO_DEVICE, DATA_SENT, 0);
		intel_xpcie_start_rx(xpcie, 0);
	}
	if (intel_xpcie_get_doorbell(xpcie, TO_DEVICE, DATA_RECEIVED)) {
		intel_xpcie_set_doorbell(xpcie, TO_DEVICE, DATA_RECEIVED, 0);
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

	if (!xpcie->legacy_a0) {
		xpcie->tx_wq = alloc_ordered_workqueue(XPCIE_DRIVER_NAME,
						       WQ_MEM_RECLAIM |
						       WQ_HIGHPRI);
		if (!xpcie->tx_wq) {
			dev_err(xpcie_to_dev(xpcie),
				"failed to allocate workqueue\n");
			destroy_workqueue(xpcie->rx_wq);
			return -ENOMEM;
		}
	}

	INIT_DELAYED_WORK(&xpcie->rx_event, intel_xpcie_rx_event_handler);
	INIT_DELAYED_WORK(&xpcie->tx_event, intel_xpcie_tx_event_handler);

	return 0;
}

static void intel_xpcie_events_cleanup(struct xpcie *xpcie)
{
	cancel_delayed_work_sync(&xpcie->rx_event);
	cancel_delayed_work_sync(&xpcie->tx_event);

	destroy_workqueue(xpcie->rx_wq);
	if (!xpcie->legacy_a0)
		destroy_workqueue(xpcie->tx_wq);
}

int intel_xpcie_core_init(struct xpcie *xpcie)
{
	int error;

	global_xpcie = xpcie;

	intel_xpcie_set_cap_txrx(xpcie);

	error = intel_xpcie_events_init(xpcie);
	if (error)
		return error;

	error = intel_xpcie_discover_txrx(xpcie);
	if (error)
		goto error_txrx;

	intel_xpcie_interfaces_init(xpcie);

	intel_xpcie_set_doorbell(xpcie, TO_DEVICE, DATA_SENT, 0);
	intel_xpcie_set_doorbell(xpcie, TO_DEVICE, DATA_RECEIVED, 0);
	intel_xpcie_set_doorbell(xpcie, TO_DEVICE, DEV_EVENT, NO_OP);
	intel_xpcie_set_doorbell(xpcie, FROM_DEVICE, DATA_SENT, 0);
	intel_xpcie_set_doorbell(xpcie, FROM_DEVICE, DATA_RECEIVED, 0);
	intel_xpcie_set_doorbell(xpcie, FROM_DEVICE, DEV_EVENT, NO_OP);

	intel_xpcie_register_host_irq(xpcie, intel_xpcie_core_irq_cb);

	return 0;

error_txrx:
	intel_xpcie_events_cleanup(xpcie);

	return error;
}

void intel_xpcie_core_cleanup(struct xpcie *xpcie)
{
	if (xpcie->status == XPCIE_STATUS_RUN) {
		intel_xpcie_events_cleanup(xpcie);
		intel_xpcie_interfaces_cleanup(xpcie);
		intel_xpcie_txrx_cleanup(xpcie);
	}
}

int intel_xpcie_core_read(struct xpcie *xpcie, void *buffer,
			  size_t *length, u32 timeout_ms)
{
	long jiffies_timeout = (long)msecs_to_jiffies(timeout_ms);
	struct xpcie_interface *inf = &xpcie->interfaces[0];
	unsigned long jiffies_start = jiffies;
	struct xpcie_buf_desc *bd;
	long jiffies_passed = 0;
	size_t len, remaining;
	int ret;

	if (*length == 0)
		return -EINVAL;

	if (xpcie->status != XPCIE_STATUS_RUN)
		return -ENODEV;

	len = *length;
	remaining = len;
	*length = 0;

	ret = mutex_lock_interruptible(&inf->rlock);
	if (ret < 0)
		return -EINTR;

	do {
		while (!inf->data_avail) {
			mutex_unlock(&inf->rlock);
			if (timeout_ms == 0) {
				ret =
				wait_event_interruptible(inf->rx_waitq,
							 inf->data_avail);
			} else {
				ret =
			wait_event_interruptible_timeout(inf->rx_waitq,
							 inf->data_avail,
							 jiffies_timeout -
							  jiffies_passed);
				if (ret == 0)
					return -ETIME;
			}
			if (ret < 0 || xpcie->stop_flag)
				return -EINTR;

			ret = mutex_lock_interruptible(&inf->rlock);
			if (ret < 0)
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
			}
		}

		/* save for next time */
		inf->partial_read = bd;

		if (!bd)
			inf->data_avail = false;

		*length = len - remaining;

		jiffies_passed = (long)jiffies - (long)jiffies_start;
	} while (remaining > 0 && (jiffies_passed < jiffies_timeout ||
				   timeout_ms == 0));

	mutex_unlock(&inf->rlock);

	return 0;
}

int intel_xpcie_core_write(struct xpcie *xpcie, void *buffer,
			   size_t *length, u32 timeout_ms)
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

	if (intel_xpcie_get_host_status(xpcie) != XPCIE_STATUS_RUN)
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

		intel_xpcie_list_put(&inf->xpcie->write, head);
		intel_xpcie_start_tx(xpcie, 0);

		*length = len - remaining;

		jiffies_passed = (long)jiffies - (long)jiffies_start;
	} while (remaining > 0 && (jiffies_passed < jiffies_timeout ||
				   timeout_ms == 0));

	mutex_unlock(&xpcie->wlock);

	return 0;
}

int intel_xpcie_get_device_status_by_id(u32 id, u32 *status)
{
	struct xpcie *xpcie = intel_xpcie_core_get_by_id(id);

	if (!xpcie)
		return -ENODEV;

	*status = xpcie->status;

	return 0;
}

u32 intel_xpcie_get_device_num(u32 *id_list)
{
	u32 num_devices = 0;

	if (xlink_sw_id) {
		num_devices = 1;
		*id_list = xlink_sw_id;
	}

	return num_devices;
}

int intel_xpcie_get_device_name_by_id(u32 id,
				      char *device_name, size_t name_size)
{
	struct xpcie *xpcie;

	xpcie = intel_xpcie_core_get_by_id(id);
	if (!xpcie)
		return -ENODEV;

	memset(device_name, 0, name_size);
	if (name_size > strlen(XPCIE_DRIVER_NAME))
		name_size = strlen(XPCIE_DRIVER_NAME);
	memcpy(device_name, XPCIE_DRIVER_NAME, name_size);

	return 0;
}

int intel_xpcie_pci_connect_device(u32 id)
{
	struct xpcie *xpcie;

	xpcie = intel_xpcie_core_get_by_id(id);
	if (!xpcie)
		return -ENODEV;

	if (xpcie->status != XPCIE_STATUS_RUN)
		return -EIO;

	return 0;
}

int intel_xpcie_pci_read(u32 id, void *data, size_t *size, u32 timeout)
{
	struct xpcie *xpcie;

	xpcie = intel_xpcie_core_get_by_id(id);
	if (!xpcie)
		return -ENODEV;

	return intel_xpcie_core_read(xpcie, data, size, timeout);
}

int intel_xpcie_pci_write(u32 id, void *data, size_t *size, u32 timeout)
{
	struct xpcie *xpcie;

	xpcie = intel_xpcie_core_get_by_id(id);
	if (!xpcie)
		return -ENODEV;

	return intel_xpcie_core_write(xpcie, data, size, timeout);
}

int intel_xpcie_pci_reset_device(u32 id)
{
	return 0;
}
