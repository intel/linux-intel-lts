// SPDX-License-Identifier: GPL-2.0-only
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#include "mxlk_util.h"

void mxlk_set_device_status(struct mxlk *mxlk, u32 status)
{
	mxlk->status = status;
	mxlk_iowrite32(status, &mxlk->mmio->device_status);
}

u32 mxlk_get_device_status(struct mxlk *mxlk)
{
	return mxlk_ioread32(&mxlk->mmio->device_status);
}

static u8 *mxlk_doorbell_offset(struct mxlk *mxlk,
				enum mxlk_doorbell_direction dirt,
				enum mxlk_doorbell_type type)
{
	if (dirt == TO_DEVICE && type == DATA_SENT)
		return &mxlk->mmio->htod_tx_doorbell;
	if (dirt == TO_DEVICE && type == DATA_RECEIVED)
		return &mxlk->mmio->htod_rx_doorbell;
	if (dirt == TO_DEVICE && type == DEV_EVENT)
		return &mxlk->mmio->htod_event_doorbell;
	if (dirt == FROM_DEVICE && type == DATA_SENT)
		return &mxlk->mmio->dtoh_tx_doorbell;
	if (dirt == FROM_DEVICE && type == DATA_RECEIVED)
		return &mxlk->mmio->dtoh_rx_doorbell;
	if (dirt == FROM_DEVICE && type == DEV_EVENT)
		return &mxlk->mmio->dtoh_event_doorbell;

	return NULL;
}

void mxlk_set_doorbell(struct mxlk *mxlk, enum mxlk_doorbell_direction dirt,
		       enum mxlk_doorbell_type type, u8 value)
{
	mxlk_iowrite8(value, mxlk_doorbell_offset(mxlk, dirt, type));
}

u8 mxlk_get_doorbell(struct mxlk *mxlk, enum mxlk_doorbell_direction dirt,
		       enum mxlk_doorbell_type type)
{
	return mxlk_ioread8(mxlk_doorbell_offset(mxlk, dirt, type));
}

u32 mxlk_get_host_status(struct mxlk *mxlk)
{
	return mxlk_ioread32(&mxlk->mmio->host_status);
}

void mxlk_set_host_status(struct mxlk *mxlk, u32 status)
{
	mxlk->status = status;
	mxlk_iowrite32(status, &mxlk->mmio->host_status);
}

struct mxlk_buf_desc *mxlk_alloc_bd(size_t length)
{
	struct mxlk_buf_desc *bd;

	bd = kzalloc(sizeof(*bd), GFP_KERNEL);
	if (!bd)
		return NULL;

	bd->head = kzalloc(roundup(length, cache_line_size()), GFP_KERNEL);
	if (!bd->head) {
		kfree(bd);
		return NULL;
	}

	bd->data = bd->head;
	bd->length = bd->true_len = length;
	bd->next = NULL;
	bd->own_mem = true;

	return bd;
}

struct mxlk_buf_desc *mxlk_alloc_bd_reuse(size_t length, void *virt,
					  dma_addr_t phys)
{
	struct mxlk_buf_desc *bd;

	bd = kzalloc(sizeof(*bd), GFP_KERNEL);
	if (!bd)
		return NULL;

	bd->head = virt;
	bd->phys = phys;
	bd->data = bd->head;
	bd->length = bd->true_len = length;
	bd->next = NULL;
	bd->own_mem = false;

	return bd;
}

void mxlk_free_bd(struct mxlk_buf_desc *bd)
{
	if (bd) {
		if (bd->own_mem)
			kfree(bd->head);
		kfree(bd);
	}
}

int mxlk_list_init(struct mxlk_list *list)
{
	spin_lock_init(&list->lock);
	list->bytes = 0;
	list->buffers = 0;
	list->head = NULL;
	list->tail = NULL;

	return 0;
}

void mxlk_list_cleanup(struct mxlk_list *list)
{
	struct mxlk_buf_desc *bd;

	spin_lock(&list->lock);
	while (list->head) {
		bd = list->head;
		list->head = bd->next;
		mxlk_free_bd(bd);
	}

	list->head = list->tail = NULL;
	spin_unlock(&list->lock);
}

int mxlk_list_put(struct mxlk_list *list, struct mxlk_buf_desc *bd)
{
	if (!bd)
		return -EINVAL;

	spin_lock(&list->lock);
	if (list->head)
		list->tail->next = bd;
	else
		list->head = bd;
	while (bd) {
		list->tail = bd;
		list->bytes += bd->length;
		list->buffers++;
		bd = bd->next;
	}
	spin_unlock(&list->lock);

	return 0;
}

int mxlk_list_put_head(struct mxlk_list *list, struct mxlk_buf_desc *bd)
{
	struct mxlk_buf_desc *old_head;

	if (!bd)
		return -EINVAL;

	spin_lock(&list->lock);
	old_head = list->head;
	list->head = bd;
	while (bd) {
		list->bytes += bd->length;
		list->buffers++;
		if (!bd->next) {
			list->tail = list->tail ? list->tail : bd;
			bd->next = old_head;
			break;
		}
		bd = bd->next;
	}
	spin_unlock(&list->lock);

	return 0;
}

struct mxlk_buf_desc *mxlk_list_get(struct mxlk_list *list)
{
	struct mxlk_buf_desc *bd;

	spin_lock(&list->lock);
	bd = list->head;
	if (list->head) {
		list->head = list->head->next;
		if (!list->head)
			list->tail = NULL;
		bd->next = NULL;
		list->bytes -= bd->length;
		list->buffers--;
	}
	spin_unlock(&list->lock);

	return bd;
}

void mxlk_list_info(struct mxlk_list *list, size_t *bytes, size_t *buffers)
{
	spin_lock(&list->lock);
	*bytes = list->bytes;
	*buffers = list->buffers;
	spin_unlock(&list->lock);
}

struct mxlk_buf_desc *mxlk_alloc_rx_bd(struct mxlk *mxlk)
{
	struct mxlk_buf_desc *bd;

	bd = mxlk_list_get(&mxlk->rx_pool);
	if (bd) {
		bd->data = bd->head;
		bd->length = bd->true_len;
		bd->next = NULL;
		bd->interface = 0;
	}

	return bd;
}

void mxlk_free_rx_bd(struct mxlk *mxlk, struct mxlk_buf_desc *bd)
{
	if (bd)
		mxlk_list_put(&mxlk->rx_pool, bd);
}

struct mxlk_buf_desc *mxlk_alloc_tx_bd(struct mxlk *mxlk)
{
	struct mxlk_buf_desc *bd;

	bd = mxlk_list_get(&mxlk->tx_pool);
	if (bd) {
		bd->data = bd->head;
		bd->length = bd->true_len;
		bd->next = NULL;
		bd->interface = 0;
	} else {
		mxlk->no_tx_buffer = true;
	}

	return bd;
}

void mxlk_free_tx_bd(struct mxlk *mxlk, struct mxlk_buf_desc *bd)
{
	if (!bd)
		return;

	mxlk_list_put(&mxlk->tx_pool, bd);

	mxlk->no_tx_buffer = false;
	wake_up_interruptible(&mxlk->tx_waitqueue);
}

int mxlk_interface_init(struct mxlk *mxlk, int id)
{
	struct mxlk_interface *inf = mxlk->interfaces + id;

	inf->id = id;
	inf->mxlk = mxlk;

	inf->partial_read = NULL;
	mxlk_list_init(&inf->read);
	mutex_init(&inf->rlock);
	inf->data_available = false;
	init_waitqueue_head(&inf->rx_waitqueue);

	return 0;
}

void mxlk_interface_cleanup(struct mxlk_interface *inf)
{
	struct mxlk_buf_desc *bd;

	mutex_destroy(&inf->rlock);

	mxlk_free_rx_bd(inf->mxlk, inf->partial_read);
	while ((bd = mxlk_list_get(&inf->read)))
		mxlk_free_rx_bd(inf->mxlk, bd);
}

void mxlk_interfaces_cleanup(struct mxlk *mxlk)
{
	int index;

	for (index = 0; index < MXLK_NUM_INTERFACES; index++)
		mxlk_interface_cleanup(mxlk->interfaces + index);

	mxlk_list_cleanup(&mxlk->write);
	mutex_destroy(&mxlk->wlock);
}

int mxlk_interfaces_init(struct mxlk *mxlk)
{
	int index;

	mutex_init(&mxlk->wlock);
	mxlk_list_init(&mxlk->write);
	init_waitqueue_head(&mxlk->tx_waitqueue);
	mxlk->no_tx_buffer = false;

	for (index = 0; index < MXLK_NUM_INTERFACES; index++)
		mxlk_interface_init(mxlk, index);

	return 0;
}

void mxlk_add_bd_to_interface(struct mxlk *mxlk, struct mxlk_buf_desc *bd)
{
	struct mxlk_interface *inf;

	inf = mxlk->interfaces + bd->interface;

	mxlk_list_put(&inf->read, bd);

	mutex_lock(&inf->rlock);
	inf->data_available = true;
	mutex_unlock(&inf->rlock);
	wake_up_interruptible(&inf->rx_waitqueue);
}

#ifdef XLINK_PCIE_REMOTE
#include "../remote_host/mxlk_pci.h"
#else
#include "../local_host/mxlk_struct.h"
#endif

static ssize_t debug_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
#ifdef XLINK_PCIE_LOCAL
	struct pci_epf *epf = container_of(dev, struct pci_epf, dev);
	struct mxlk_epf *mxlk_epf = epf_get_drvdata(epf);
	struct mxlk *mxlk = &mxlk_epf->mxlk;
#else
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct mxlk_pcie *xdev = pci_get_drvdata(pdev);
	struct mxlk *mxlk = &xdev->mxlk;
#endif
	size_t bytes, tx_list_num, rx_list_num, tx_pool_num, rx_pool_num;

	if (!mxlk->debug_enable)
		return 0;

	mxlk_list_info(&mxlk->write, &bytes, &tx_list_num);
	mxlk_list_info(&mxlk->interfaces[0].read, &bytes, &rx_list_num);
	mxlk_list_info(&mxlk->tx_pool, &bytes, &tx_pool_num);
	mxlk_list_info(&mxlk->rx_pool, &bytes, &rx_pool_num);

	snprintf(buf, 4096,
		 "tx_krn, cnts %zu bytes %zu\n"
		 "tx_usr, cnts %zu bytes %zu\n"
		 "rx_krn, cnts %zu bytes %zu\n"
		 "rx_usr, cnts %zu bytes %zu\n"
		 "tx_list %zu tx_pool %zu\n"
		 "rx_list %zu rx_pool %zu\n"
		 "interrupts %zu, send_ints %zu\n"
		 "rx runs %zu tx runs %zu\n",
		 mxlk->stats.tx_krn.cnts, mxlk->stats.tx_krn.bytes,
		 mxlk->stats.tx_usr.cnts, mxlk->stats.tx_usr.bytes,
		 mxlk->stats.rx_krn.cnts, mxlk->stats.rx_krn.bytes,
		 mxlk->stats.rx_usr.cnts, mxlk->stats.rx_usr.bytes,
		 tx_list_num, tx_pool_num, rx_list_num, rx_pool_num,
		 mxlk->stats.interrupts, mxlk->stats.send_ints,
		 mxlk->stats.rx_event_runs, mxlk->stats.tx_event_runs
	 );

	return strlen(buf);
}

static ssize_t debug_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	long value;
	int rc;

#ifdef XLINK_PCIE_LOCAL
	struct pci_epf *epf = container_of(dev, struct pci_epf, dev);
	struct mxlk_epf *mxlk_epf = epf_get_drvdata(epf);
	struct mxlk *mxlk = &mxlk_epf->mxlk;
#else
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct mxlk_pcie *xdev = pci_get_drvdata(pdev);
	struct mxlk *mxlk = &xdev->mxlk;
#endif

	rc = kstrtol(buf, 10, &value);
	if (rc)
		return rc;

	mxlk->debug_enable = value ? true : false;

	if (!mxlk->debug_enable)
		memset(&mxlk->stats, 0, sizeof(struct mxlk_debug_stats));

	return count;
}

void mxlk_init_debug(struct mxlk *mxlk, struct device *dev)
{
	DEVICE_ATTR_RW(debug);
	memset(&mxlk->stats, 0, sizeof(struct mxlk_debug_stats));
	mxlk->debug = dev_attr_debug;
	device_create_file(dev, &mxlk->debug);
}

void mxlk_uninit_debug(struct mxlk *mxlk, struct device *dev)
{
	device_remove_file(dev, &mxlk->debug);
	memset(&mxlk->stats, 0, sizeof(struct mxlk_debug_stats));
}

void mxlk_debug_incr(struct mxlk *mxlk, size_t *attr, size_t v)
{
	if (unlikely(mxlk->debug_enable))
		*attr += v;
}
