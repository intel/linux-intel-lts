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
#include <linux/interrupt.h>

#include "../common/mxlk_core.h"
#include "../common/mxlk_util.h"
#include "../common/mxlk_capabilities.h"
#include "mxlk_epf.h"
#include "mxlk_struct.h"

static struct mxlk *global_mxlk;

#define MXLK_CIRCULAR_INC(val, max) (((val) + 1) & (max - 1))

static int rx_pool_size = SZ_32M;
module_param(rx_pool_size, int, 0664);
MODULE_PARM_DESC(rx_pool_size, "receiving pool size (default 32 MiB)");

static int tx_pool_size = SZ_32M;
module_param(tx_pool_size, int, 0664);
MODULE_PARM_DESC(tx_pool_size, "transmitting pool size (default 32 MiB)");

static int fragment_size = MXLK_FRAGMENT_SIZE;
module_param(fragment_size, int, 0664);
MODULE_PARM_DESC(fragment_size, "transfer descriptor size (default 128 KiB)");

static bool tx_pool_coherent = true;
module_param(tx_pool_coherent, bool, 0664);
MODULE_PARM_DESC(tx_pool_coherent, "transmitting pool using coherent memory (default true)");

static bool rx_pool_coherent;
module_param(rx_pool_coherent, bool, 0664);
MODULE_PARM_DESC(rx_pool_coherent, "receiving pool using coherent memory (default false)");

static int mxlk_map_dma(struct mxlk *mxlk, struct mxlk_buf_desc *bd,
			int direction)
{
	struct mxlk_epf *mxlk_epf = container_of(mxlk, struct mxlk_epf, mxlk);
	struct pci_epf *epf = mxlk_epf->epf;
	struct device *dma_dev = epf->epc->dev.parent;

	bd->phys = dma_map_single(dma_dev, bd->data, bd->length, direction);

	return dma_mapping_error(dma_dev, bd->phys);
}

static void mxlk_unmap_dma(struct mxlk *mxlk, struct mxlk_buf_desc *bd,
			   int direction)
{
	struct mxlk_epf *mxlk_epf = container_of(mxlk, struct mxlk_epf, mxlk);
	struct pci_epf *epf = mxlk_epf->epf;
	struct device *dma_dev = epf->epc->dev.parent;

	dma_unmap_single(dma_dev, bd->phys, bd->length, direction);
}

static void mxlk_set_cap_txrx(struct mxlk *mxlk)
{
	struct mxlk_cap_txrx *cap;
	struct mxlk_cap_hdr *hdr;
	uint32_t start = sizeof(struct mxlk_mmio);
	size_t hdr_len = sizeof(struct mxlk_cap_txrx);
	size_t tx_len = sizeof(struct mxlk_transfer_desc) * MXLK_NUM_TX_DESCS;
	size_t rx_len = sizeof(struct mxlk_transfer_desc) * MXLK_NUM_RX_DESCS;
	uint16_t next = (uint16_t)(start + hdr_len + tx_len + rx_len);

	mxlk->mmio->cap_offset = start;
	cap = (void *)mxlk->mmio + start;
	memset(cap, 0, sizeof(struct mxlk_cap_txrx));
	cap->hdr.id = MXLK_CAP_TXRX;
	cap->hdr.next = next;
	cap->fragment_size = fragment_size;
	cap->tx.ring = start + hdr_len;
	cap->tx.ndesc = MXLK_NUM_TX_DESCS;
	cap->rx.ring = start + hdr_len + tx_len;
	cap->rx.ndesc = MXLK_NUM_RX_DESCS;

	hdr = (struct mxlk_cap_hdr *)((void *)mxlk->mmio + next);
	hdr->id = MXLK_CAP_NULL;
}

static int mxlk_set_version(struct mxlk *mxlk)
{
	struct mxlk_version version;

	version.major = MXLK_VERSION_MAJOR;
	version.minor = MXLK_VERSION_MINOR;
	version.build = MXLK_VERSION_BUILD;

	memcpy(&mxlk->mmio->version, &version, sizeof(version));

	dev_info(mxlk_to_dev(mxlk), "ver: device %u.%u.%u\n",
		 version.major, version.minor, version.build);

	return 0;
}

static void mxlk_txrx_cleanup(struct mxlk *mxlk)
{
	int index;
	struct mxlk_transfer_desc *td;
	struct mxlk_stream *tx = &mxlk->tx;
	struct mxlk_stream *rx = &mxlk->rx;
	struct mxlk_epf *mxlk_epf = container_of(mxlk, struct mxlk_epf, mxlk);
	struct device *dma_dev = mxlk_epf->epf->epc->dev.parent;
	struct mxlk_interface *inf = &mxlk->interfaces[0];

	mxlk->stop_flag = true;
	mxlk->no_tx_buffer = false;
	inf->data_available = true;
	wake_up_interruptible(&mxlk->tx_waitqueue);
	wake_up_interruptible(&inf->rx_waitqueue);
	mutex_lock(&mxlk->wlock);
	mutex_lock(&inf->rlock);

	for (index = 0; index < rx->pipe.ndesc; index++) {
		td = rx->pipe.tdr + index;
		mxlk_set_td_address(td, 0);
		mxlk_set_td_length(td, 0);
	}
	for (index = 0; index < tx->pipe.ndesc; index++) {
		td = tx->pipe.tdr + index;
		mxlk_set_td_address(td, 0);
		mxlk_set_td_length(td, 0);
	}

	mxlk_list_cleanup(&mxlk->tx_pool);
	mxlk_list_cleanup(&mxlk->rx_pool);

	if (rx_pool_coherent && mxlk_epf->rx_virt) {
		dma_free_coherent(dma_dev, mxlk_epf->rx_size,
				  mxlk_epf->rx_virt, mxlk_epf->rx_phys);
	}

	if (tx_pool_coherent && mxlk_epf->tx_virt) {
		dma_free_coherent(dma_dev, mxlk_epf->tx_size,
				  mxlk_epf->tx_virt, mxlk_epf->tx_phys);
	}

	mutex_unlock(&inf->rlock);
	mutex_unlock(&mxlk->wlock);
}

/*
 * The RX/TX are named for Remote Host, in Local Host RX/TX is reversed.
 */
static int mxlk_txrx_init(struct mxlk *mxlk, struct mxlk_cap_txrx *cap)
{
	int index;
	int ndesc;
	struct mxlk_buf_desc *bd;
	struct mxlk_stream *tx = &mxlk->tx;
	struct mxlk_stream *rx = &mxlk->rx;
	struct mxlk_epf *mxlk_epf = container_of(mxlk, struct mxlk_epf, mxlk);
	struct device *dma_dev = mxlk_epf->epf->epc->dev.parent;

	mxlk->txrx = cap;
	mxlk->fragment_size = cap->fragment_size;
	mxlk->stop_flag = false;

	rx->pipe.ndesc = cap->tx.ndesc;
	rx->pipe.head = &cap->tx.head;
	rx->pipe.tail = &cap->tx.tail;
	rx->pipe.tdr = (void *)mxlk->mmio + cap->tx.ring;

	tx->pipe.ndesc = cap->rx.ndesc;
	tx->pipe.head = &cap->rx.head;
	tx->pipe.tail = &cap->rx.tail;
	tx->pipe.tdr = (void *)mxlk->mmio + cap->rx.ring;

	mxlk_list_init(&mxlk->rx_pool);
	rx_pool_size = roundup(rx_pool_size, mxlk->fragment_size);
	ndesc = rx_pool_size / mxlk->fragment_size;

	if (rx_pool_coherent) {
		mxlk_epf->rx_size = rx_pool_size;
		mxlk_epf->rx_virt = dma_alloc_coherent(dma_dev,
			mxlk_epf->rx_size, &mxlk_epf->rx_phys, GFP_KERNEL);
		if (!mxlk_epf->rx_virt)
			goto error;
	}

	for (index = 0; index < ndesc; index++) {
		if (rx_pool_coherent) {
			bd = mxlk_alloc_bd_reuse(mxlk->fragment_size,
			mxlk_epf->rx_virt + index * mxlk->fragment_size,
			mxlk_epf->rx_phys + index * mxlk->fragment_size);
		} else {
			bd = mxlk_alloc_bd(mxlk->fragment_size);
		}
		if (bd) {
			mxlk_list_put(&mxlk->rx_pool, bd);
		} else {
			dev_err(mxlk_to_dev(mxlk),
				"failed to alloc all rx pool descriptors\n");
			goto error;
		}
	}

	mxlk_list_init(&mxlk->tx_pool);
	tx_pool_size = roundup(tx_pool_size, mxlk->fragment_size);
	ndesc = tx_pool_size / mxlk->fragment_size;

	if (tx_pool_coherent) {
		mxlk_epf->tx_size = tx_pool_size;
		mxlk_epf->tx_virt = dma_alloc_coherent(dma_dev,
			mxlk_epf->tx_size, &mxlk_epf->tx_phys, GFP_KERNEL);
		if (!mxlk_epf->tx_virt)
			goto error;
	}

	for (index = 0; index < ndesc; index++) {
		if (tx_pool_coherent) {
			bd = mxlk_alloc_bd_reuse(mxlk->fragment_size,
			mxlk_epf->tx_virt + index * mxlk->fragment_size,
			mxlk_epf->tx_phys + index * mxlk->fragment_size);
		} else {
			bd = mxlk_alloc_bd(mxlk->fragment_size);
		}
		if (bd) {
			mxlk_list_put(&mxlk->tx_pool, bd);
		} else {
			dev_err(mxlk_to_dev(mxlk),
				"failed to alloc all tx pool descriptors\n");
			goto error;
		}
	}

	return 0;

error:
	mxlk_txrx_cleanup(mxlk);

	return -ENOMEM;
}

static int mxlk_discover_txrx(struct mxlk *mxlk)
{
	int error;
	struct mxlk_cap_txrx *cap;

	cap = mxlk_cap_find(mxlk, 0, MXLK_CAP_TXRX);
	if (cap) {
		error = mxlk_txrx_init(mxlk, cap);
	} else {
		dev_err(mxlk_to_dev(mxlk), "mxlk txrx info not found\n");
		error = -EIO;
	}

	return error;
}

static void mxlk_start_tx(struct mxlk *mxlk, unsigned long delay)
{
	if (mxlk->legacy_a0)
		queue_delayed_work(mxlk->rx_wq, &mxlk->tx_event, delay);
	else
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
	u16 interface;
	u32 head, tail, ndesc, length;
	u64 address;
	u32 initial_head;
	int descs_num = 0;
	int chan = 0;
	struct mxlk_stream *rx = &mxlk->rx;
	struct mxlk_epf *mxlk_epf = container_of(mxlk, struct mxlk_epf, mxlk);
	struct mxlk_dma_ll_desc *desc;
	struct mxlk_buf_desc *bd_head, *bd_tail, *bd;
	struct mxlk_transfer_desc *td;
	unsigned long delay = msecs_to_jiffies(1);
	bool reset_work = false;

	mxlk_debug_incr(mxlk, &mxlk->stats.rx_event_runs, 1);

	if (mxlk_get_host_status(mxlk) != MXLK_STATUS_RUN)
		return;

	bd_head = bd_tail = NULL;
	ndesc = rx->pipe.ndesc;
	tail = mxlk_get_tdr_tail(&rx->pipe);
	initial_head = head = mxlk_get_tdr_head(&rx->pipe);

	while (head != tail) {
		td = rx->pipe.tdr + head;

		bd = mxlk_alloc_rx_bd(mxlk);
		if (!bd) {
			reset_work = true;
			if (descs_num == 0) {
				delay = msecs_to_jiffies(10);
				goto task_exit;
			}
			break;
		}

		interface = mxlk_get_td_interface(td);
		length = mxlk_get_td_length(td);
		address = mxlk_get_td_address(td);

		bd->length = length;
		bd->interface = interface;
		if (!rx_pool_coherent) {
			rc = mxlk_map_dma(mxlk, bd, DMA_FROM_DEVICE);
			if (rc) {
				dev_err(mxlk_to_dev(mxlk),
					"failed to map rx bd (%d)\n", rc);
				mxlk_free_rx_bd(mxlk, bd);
				break;
			}
		}

		desc = &mxlk_epf->rx_desc_buf[chan].virt[descs_num++];
		desc->dma_transfer_size = length;
		desc->dst_addr = bd->phys;
		desc->src_addr = address;

		if (bd_head)
			bd_tail->next = bd;
		else
			bd_head = bd;
		bd_tail = bd;

		head = MXLK_CIRCULAR_INC(head, ndesc);
	}

	if (descs_num == 0)
		goto task_exit;

	rc = mxlk_copy_from_host_ll(mxlk, chan, descs_num);

	bd = bd_head;
	while (bd && !rx_pool_coherent) {
		mxlk_unmap_dma(mxlk, bd, DMA_FROM_DEVICE);
		bd = bd->next;
	}

	if (rc) {
		dev_err(mxlk_to_dev(mxlk),
			"failed to DMA from host (%d)\n", rc);
		mxlk_free_rx_bd(mxlk, bd_head);
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

		if (likely(bd->interface < MXLK_NUM_INTERFACES)) {
			mxlk_debug_incr(mxlk, &mxlk->stats.rx_krn.cnts, 1);
			mxlk_debug_incr(mxlk, &mxlk->stats.rx_krn.bytes,
					bd->length);

			mxlk_set_td_status(td, MXLK_DESC_STATUS_SUCCESS);
			mxlk_add_bd_to_interface(mxlk, bd);
		} else {
			dev_err(mxlk_to_dev(mxlk),
				"detected rx desc interface failure (%u)\n",
				bd->interface);
			mxlk_set_td_status(td, MXLK_DESC_STATUS_ERROR);
			mxlk_free_rx_bd(mxlk, bd);
		}

		bd = bd_head;
		head = MXLK_CIRCULAR_INC(head, ndesc);
	}

	if (head != initial_head) {
		mxlk_set_tdr_head(&rx->pipe, head);
		mxlk_raise_irq(mxlk, DATA_RECEIVED);
	}

task_exit:
	if (reset_work)
		mxlk_start_rx(mxlk, delay);
}

static void mxlk_tx_event_handler(struct work_struct *work)
{
	struct mxlk *mxlk = container_of(work, struct mxlk, tx_event.work);

	int rc;
	u32 head, tail, ndesc;
	u64 address;
	u32 initial_tail;
	int descs_num = 0;
	int chan = 0;
	struct mxlk_stream *tx = &mxlk->tx;
	struct mxlk_epf *mxlk_epf = container_of(mxlk, struct mxlk_epf, mxlk);
	struct mxlk_dma_ll_desc *desc;
	struct mxlk_buf_desc *bd_head, *bd_tail, *bd;
	struct mxlk_transfer_desc *td;
	size_t bytes = 0, buffers = 0;

	mxlk_debug_incr(mxlk, &mxlk->stats.tx_event_runs, 1);

	if (mxlk_get_host_status(mxlk) != MXLK_STATUS_RUN)
		return;

	bd_head = bd_tail = NULL;
	ndesc = tx->pipe.ndesc;
	initial_tail = tail = mxlk_get_tdr_tail(&tx->pipe);
	head = mxlk_get_tdr_head(&tx->pipe);

	// add new entries
	while (MXLK_CIRCULAR_INC(tail, ndesc) != head) {
		bd = mxlk_list_get(&mxlk->write);
		if (!bd)
			break;

		if (!tx_pool_coherent) {
			if (mxlk_map_dma(mxlk, bd, DMA_TO_DEVICE)) {
				dev_err(mxlk_to_dev(mxlk),
				"dma mapping error bd addr %p, size %zu\n",
				bd->data, bd->length);
				mxlk_list_put_head(&mxlk->write, bd);
				break;
			}
		}

		td = tx->pipe.tdr + tail;
		address = mxlk_get_td_address(td);

		desc = &mxlk_epf->tx_desc_buf[chan].virt[descs_num++];
		desc->dma_transfer_size = bd->length;
		desc->src_addr = bd->phys;
		desc->dst_addr = address;

		if (bd_head)
			bd_tail->next = bd;
		else
			bd_head = bd;
		bd_tail = bd;

		tail = MXLK_CIRCULAR_INC(tail, ndesc);
	}

	if (descs_num == 0)
		goto task_exit;

	rc = mxlk_copy_to_host_ll(mxlk, chan, descs_num);

	tail = initial_tail;
	bd = bd_head;
	while (bd) {
		if (!tx_pool_coherent)
			mxlk_unmap_dma(mxlk, bd, DMA_TO_DEVICE);

		if (rc) {
			bd = bd->next;
			continue;
		}

		mxlk_debug_incr(mxlk, &mxlk->stats.tx_krn.cnts, 1);
		mxlk_debug_incr(mxlk, &mxlk->stats.tx_krn.bytes, bd->length);

		td = tx->pipe.tdr + tail;
		mxlk_set_td_status(td, MXLK_DESC_STATUS_SUCCESS);
		mxlk_set_td_length(td, bd->length);
		mxlk_set_td_interface(td, bd->interface);

		bd = bd->next;
		tail = MXLK_CIRCULAR_INC(tail, ndesc);
	}

	if (rc) {
		dev_err(mxlk_to_dev(mxlk), "failed to DMA to host (%d)\n", rc);
		mxlk_list_put_head(&mxlk->write, bd_head);
		return;
	}

	mxlk_free_tx_bd(mxlk, bd_head);

	if (mxlk_get_tdr_tail(&tx->pipe) != tail) {
		mxlk_set_tdr_tail(&tx->pipe, tail);
		mxlk_raise_irq(mxlk, DATA_SENT);
		mxlk_debug_incr(mxlk, &mxlk->stats.send_ints, 1);
	}

task_exit:
	mxlk_list_info(&mxlk->write, &bytes, &buffers);
	if (buffers) {
		mxlk->tx_pending = true;
		head = mxlk_get_tdr_head(&tx->pipe);
		if (MXLK_CIRCULAR_INC(tail, ndesc) != head)
			mxlk_start_tx(mxlk, 0);
	} else {
		mxlk->tx_pending = false;
	}
}

static irqreturn_t mxlk_core_irq_cb(int irq, void *args)
{
	struct mxlk *mxlk = args;

	if (mxlk_get_doorbell(mxlk, TO_DEVICE, DATA_SENT)) {
		mxlk_set_doorbell(mxlk, TO_DEVICE, DATA_SENT, 0);
		mxlk_debug_incr(mxlk, &mxlk->stats.interrupts, 1);
		mxlk_start_rx(mxlk, 0);
	}
	if (mxlk_get_doorbell(mxlk, TO_DEVICE, DATA_RECEIVED)) {
		mxlk_set_doorbell(mxlk, TO_DEVICE, DATA_RECEIVED, 0);
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

	if (!mxlk->legacy_a0) {
		mxlk->tx_wq = alloc_ordered_workqueue(MXLK_DRIVER_NAME,
					      WQ_MEM_RECLAIM | WQ_HIGHPRI);
		if (!mxlk->tx_wq) {
			dev_err(mxlk_to_dev(mxlk),
				"failed to allocate workqueue\n");
			destroy_workqueue(mxlk->rx_wq);
			return -ENOMEM;
		}
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
	if (!mxlk->legacy_a0)
		destroy_workqueue(mxlk->tx_wq);
}

int mxlk_core_init(struct mxlk *mxlk)
{
	int error;
	struct mxlk_epf *mxlk_epf = container_of(mxlk, struct mxlk_epf, mxlk);

	mxlk_init_debug(mxlk, &mxlk_epf->epf->dev);

	global_mxlk = mxlk;

	mxlk_set_version(mxlk);
	mxlk_set_cap_txrx(mxlk);

	error = mxlk_events_init(mxlk);
	if (error)
		return error;

	error = mxlk_discover_txrx(mxlk);
	if (error)
		goto error_txrx;

	mxlk_interfaces_init(mxlk);

	mxlk_set_doorbell(mxlk, TO_DEVICE, DATA_SENT, 0);
	mxlk_set_doorbell(mxlk, TO_DEVICE, DATA_RECEIVED, 0);
	mxlk_set_doorbell(mxlk, TO_DEVICE, DEV_EVENT, NO_OP);
	mxlk_set_doorbell(mxlk, FROM_DEVICE, DATA_SENT, 0);
	mxlk_set_doorbell(mxlk, FROM_DEVICE, DATA_RECEIVED, 0);
	mxlk_set_doorbell(mxlk, FROM_DEVICE, DEV_EVENT, NO_OP);

	mxlk_register_host_irq(mxlk, mxlk_core_irq_cb);

	return 0;

error_txrx:
	mxlk_events_cleanup(mxlk);

	return error;
}

void mxlk_core_cleanup(struct mxlk *mxlk)
{
	struct mxlk_epf *mxlk_epf = container_of(mxlk, struct mxlk_epf, mxlk);

	if (mxlk->status == MXLK_STATUS_RUN) {
		mxlk_events_cleanup(mxlk);
		mxlk_interfaces_cleanup(mxlk);
		mxlk_txrx_cleanup(mxlk);
	}

	mxlk_uninit_debug(mxlk, &mxlk_epf->epf->dev);
}

int mxlk_core_read(struct mxlk *mxlk, void *buffer, size_t *length,
		   unsigned int timeout_ms)
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
		    unsigned int timeout_ms)
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

	if (mxlk_get_host_status(mxlk) != MXLK_STATUS_RUN)
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

		mxlk_list_put(&inf->mxlk->write, head);
		mxlk_start_tx(mxlk, 0);

		*length = len - remaining;

		jiffies_passed = (long)jiffies - (long)jiffies_start;
	} while (remaining > 0 && (jiffies_passed < jiffies_timeout ||
				   timeout_ms == 0));

	mutex_unlock(&mxlk->wlock);

	return 0;
}

struct mxlk *mxlk_core_get_by_id(uint32_t sw_device_id)
{
	return (sw_device_id == xlink_sw_id) ? global_mxlk : NULL;
}
