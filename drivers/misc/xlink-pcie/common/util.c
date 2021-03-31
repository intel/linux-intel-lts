// SPDX-License-Identifier: GPL-2.0-only
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#include "util.h"

void intel_xpcie_set_device_status(struct xpcie *xpcie, u32 status)
{
	xpcie->status = status;
	intel_xpcie_iowrite32(status, xpcie->mmio + XPCIE_MMIO_DEV_STATUS);
}

u32 intel_xpcie_get_device_status(struct xpcie *xpcie)
{
	return intel_xpcie_ioread32(xpcie->mmio + XPCIE_MMIO_DEV_STATUS);
}

static size_t intel_xpcie_doorbell_offset(struct xpcie *xpcie,
					  enum xpcie_doorbell_direction dirt,
					  enum xpcie_doorbell_type type)
{
	if (dirt == TO_DEVICE && type == DATA_SENT)
		return XPCIE_MMIO_HTOD_TX_DOORBELL;
	if (dirt == TO_DEVICE && type == DATA_RECEIVED)
		return XPCIE_MMIO_HTOD_RX_DOORBELL;
	if (dirt == TO_DEVICE && type == DEV_EVENT)
		return XPCIE_MMIO_HTOD_EVENT_DOORBELL;
	if (dirt == TO_DEVICE && type == HOST_STATUS)
		return XPCIE_MMIO_HTOD_HOST_STATUS;
	if (dirt == FROM_DEVICE && type == DATA_SENT)
		return XPCIE_MMIO_DTOH_TX_DOORBELL;
	if (dirt == FROM_DEVICE && type == DATA_RECEIVED)
		return XPCIE_MMIO_DTOH_RX_DOORBELL;
	if (dirt == FROM_DEVICE && type == DEV_EVENT)
		return XPCIE_MMIO_DTOH_EVENT_DOORBELL;
	if (dirt == TO_DEVICE && type == PARTIAL_DATA_RECEIVED)
		return XPCIE_MMIO_HTOD_PARTIAL_RX_DOORBELL;
	if (dirt == TO_DEVICE && type == RX_BD_COUNT)
		return XPCIE_MMIO_HTOD_RX_BD_LIST_COUNT;

	return 0;
}

void intel_xpcie_update_device_flwctl(struct xpcie *xpcie,
				      enum xpcie_doorbell_direction dirt,
				      enum xpcie_doorbell_type type,
				      int value)
{
	size_t offset = intel_xpcie_doorbell_offset(xpcie, dirt, type);

	if (dirt == TO_DEVICE && type == RX_BD_COUNT)
		intel_xpcie_iowrite32(value, xpcie->mmio + offset);
}

u32 intel_xpcie_get_device_flwctl(struct xpcie *xpcie,
				  enum xpcie_doorbell_direction dirt,
				  enum xpcie_doorbell_type type)
{
	size_t offset = intel_xpcie_doorbell_offset(xpcie, dirt, type);
	u32 ret = 0;

	if (dirt == TO_DEVICE && type == RX_BD_COUNT)
		ret = intel_xpcie_ioread32(xpcie->mmio + offset);

	return ret;
}

void intel_xpcie_set_doorbell(struct xpcie *xpcie,
			      enum xpcie_doorbell_direction dirt,
			      enum xpcie_doorbell_type type, u8 value)
{
	size_t offset = intel_xpcie_doorbell_offset(xpcie, dirt, type);

	intel_xpcie_iowrite8(value, xpcie->mmio + offset);
}

u8 intel_xpcie_get_doorbell(struct xpcie *xpcie,
			    enum xpcie_doorbell_direction dirt,
			    enum xpcie_doorbell_type type)
{
	size_t offset = intel_xpcie_doorbell_offset(xpcie, dirt, type);

	return intel_xpcie_ioread8(xpcie->mmio + offset);
}

u32 intel_xpcie_get_host_status(struct xpcie *xpcie)
{
	return intel_xpcie_ioread32(xpcie->mmio + XPCIE_MMIO_HOST_STATUS);
}

void intel_xpcie_set_host_status(struct xpcie *xpcie, u32 status)
{
	xpcie->status = status;
	intel_xpcie_iowrite32(status, xpcie->mmio + XPCIE_MMIO_HOST_STATUS);
}

struct xpcie_buf_desc *intel_xpcie_alloc_bd(size_t length)
{
	struct xpcie_buf_desc *bd;

	bd = kzalloc(sizeof(*bd), GFP_KERNEL);
	if (!bd)
		return NULL;

	bd->head = kzalloc(roundup(length, cache_line_size()), GFP_KERNEL);
	if (!bd->head) {
		kfree(bd);
		return NULL;
	}

	bd->data = bd->head;
	bd->length = length;
	bd->true_len = length;
	bd->next = NULL;
	bd->own_mem = true;

	return bd;
}

struct xpcie_buf_desc *intel_xpcie_alloc_bd_reuse(size_t length, void *virt,
						  dma_addr_t phys)
{
	struct xpcie_buf_desc *bd;

	bd = kzalloc(sizeof(*bd), GFP_KERNEL);
	if (!bd)
		return NULL;

	bd->head = virt;
	bd->phys = phys;
	bd->data = bd->head;
	bd->length = length;
	bd->true_len = length;
	bd->next = NULL;
	bd->own_mem = false;

	return bd;
}

void intel_xpcie_free_bd(struct xpcie_buf_desc *bd)
{
	if (bd) {
		if (bd->own_mem)
			kfree(bd->head);
		kfree(bd);
	}
}

int intel_xpcie_list_init(struct xpcie_list *list)
{
	spin_lock_init(&list->lock);
	list->bytes = 0;
	list->buffers = 0;
	list->head = NULL;
	list->tail = NULL;

	return 0;
}

void intel_xpcie_list_cleanup(struct xpcie_list *list)
{
	struct xpcie_buf_desc *bd;

	spin_lock(&list->lock);
	while (list->head) {
		bd = list->head;
		list->head = bd->next;
		intel_xpcie_free_bd(bd);
	}

	list->head = NULL;
	list->tail = NULL;
	spin_unlock(&list->lock);
}

bool intel_xpcie_list_empty(struct xpcie_list *list)
{
	if (list && !list->head && !list->tail)
		return false;

	return true;
}

int intel_xpcie_list_put(struct xpcie_list *list, struct xpcie_buf_desc *bd)
{
#ifdef XLINK_PCIE_REMOTE
	unsigned long flags = 0;
#endif

	if (!bd)
		return -EINVAL;
#ifdef XLINK_PCIE_REMOTE
	spin_lock_irqsave(&list->lock, flags);
#else
	spin_lock(&list->lock);
#endif
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
#ifdef XLINK_PCIE_REMOTE
	spin_unlock_irqrestore(&list->lock, flags);
#else
	spin_unlock(&list->lock);
#endif
	return 0;
}

int intel_xpcie_list_put_head(struct xpcie_list *list,
			      struct xpcie_buf_desc *bd)
{
	struct xpcie_buf_desc *old_head;

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

struct xpcie_buf_desc *intel_xpcie_list_get(struct xpcie_list *list)
{
	struct xpcie_buf_desc *bd;
#ifdef XLINK_PCIE_REMOTE
	unsigned long flags = 0;

	spin_lock_irqsave(&list->lock, flags);
#else
	spin_lock(&list->lock);
#endif
	bd = list->head;
	if (list->head) {
		list->head = list->head->next;
		if (!list->head)
			list->tail = NULL;
		bd->next = NULL;
		list->bytes -= bd->length;
		list->buffers--;
	}
#ifdef XLINK_PCIE_REMOTE
	spin_unlock_irqrestore(&list->lock, flags);
#else
	spin_unlock(&list->lock);
#endif
	return bd;
}

void intel_xpcie_list_info(struct xpcie_list *list,
			   size_t *bytes, size_t *buffers)
{
	spin_lock(&list->lock);
	*bytes = list->bytes;
	*buffers = list->buffers;
	spin_unlock(&list->lock);
}

struct xpcie_buf_desc *intel_xpcie_alloc_rx_bd(struct xpcie *xpcie)
{
	struct xpcie_buf_desc *bd;

	bd = intel_xpcie_list_get(&xpcie->rx_pool);
	if (bd) {
		bd->data = bd->head;
		bd->length = bd->true_len;
		bd->next = NULL;
		bd->interface = 0;
	}

	return bd;
}

void intel_xpcie_free_rx_bd(struct xpcie *xpcie, struct xpcie_buf_desc *bd)
{
	if (bd)
		intel_xpcie_list_put(&xpcie->rx_pool, bd);
}

struct xpcie_buf_desc *intel_xpcie_alloc_tx_bd(struct xpcie *xpcie)
{
	struct xpcie_buf_desc *bd;

	bd = intel_xpcie_list_get(&xpcie->tx_pool);
	if (bd) {
		bd->data = bd->head;
		bd->length = bd->true_len;
		bd->next = NULL;
		bd->interface = 0;
	} else {
		xpcie->no_tx_buffer = true;
	}

	return bd;
}

void intel_xpcie_free_tx_bd(struct xpcie *xpcie, struct xpcie_buf_desc *bd)
{
	if (!bd)
		return;

	intel_xpcie_list_put(&xpcie->tx_pool, bd);

	xpcie->no_tx_buffer = false;
	wake_up_interruptible(&xpcie->tx_waitq);
}

int intel_xpcie_interface_init(struct xpcie *xpcie, int id)
{
	struct xpcie_interface *inf = xpcie->interfaces + id;

	inf->id = id;
	inf->xpcie = xpcie;

	inf->partial_read = NULL;
	intel_xpcie_list_init(&inf->read);
	mutex_init(&inf->rlock);
	inf->data_avail = false;
	init_waitqueue_head(&inf->rx_waitq);

	return 0;
}

void intel_xpcie_interface_cleanup(struct xpcie_interface *inf)
{
	struct xpcie_buf_desc *bd;

	intel_xpcie_free_rx_bd(inf->xpcie, inf->partial_read);
	while ((bd = intel_xpcie_list_get(&inf->read)))
		intel_xpcie_free_rx_bd(inf->xpcie, bd);

	mutex_destroy(&inf->rlock);
}

void intel_xpcie_interfaces_cleanup(struct xpcie *xpcie)
{
	int index;

	for (index = 0; index < XPCIE_NUM_INTERFACES; index++)
		intel_xpcie_interface_cleanup(xpcie->interfaces + index);

	intel_xpcie_list_cleanup(&xpcie->write);
	mutex_destroy(&xpcie->wlock);
}

int intel_xpcie_interfaces_init(struct xpcie *xpcie)
{
	int index;

	mutex_init(&xpcie->wlock);
	intel_xpcie_list_init(&xpcie->write);
	init_waitqueue_head(&xpcie->tx_waitq);
	xpcie->no_tx_buffer = false;

#ifdef XLINK_PCIE_LOCAL
	init_waitqueue_head(&xpcie->host_st_waitqueue);
	xpcie->no_host_driver = false;
#endif
	for (index = 0; index < XPCIE_NUM_INTERFACES; index++)
		intel_xpcie_interface_init(xpcie, index);

	return 0;
}

void intel_xpcie_add_bd_to_interface(struct xpcie *xpcie,
				     struct xpcie_buf_desc *bd)
{
	struct xpcie_interface *inf;

	inf = xpcie->interfaces + bd->interface;

	intel_xpcie_list_put(&inf->read, bd);
#ifdef XLINK_PCIE_REMOTE
	atomic_inc(&inf->available_bd_cnt);
	inf->data_avail = true;
#else
	mutex_lock(&inf->rlock);
	inf->data_avail = true;
	mutex_unlock(&inf->rlock);
#endif
	wake_up_interruptible(&inf->rx_waitq);
}

void *intel_xpcie_cap_find(struct xpcie *xpcie, u32 start, u16 id)
{
	int ttl = XPCIE_CAP_TTL;
	void *hdr;
	u16 id_out, next;

	/* If user didn't specify start, assume start of mmio */
	if (!start)
		start = intel_xpcie_ioread32(xpcie->mmio + XPCIE_MMIO_CAP_OFF);

	/* Read header info */
	hdr = xpcie->mmio + start;

	/* Check if we still have time to live */
	while (ttl--) {
		id_out = intel_xpcie_ioread16(hdr + XPCIE_CAP_HDR_ID);
		next = intel_xpcie_ioread16(hdr + XPCIE_CAP_HDR_NEXT);

		/* If cap matches, return header */
		if (id_out == id)
			return hdr;
		/* If cap is NULL, we are at the end of the list */
		else if (id_out == XPCIE_CAP_NULL)
			return NULL;
		/* If no match and no end of list, traverse the linked list */
		else
			hdr = xpcie->mmio + next;
	}

	/* If we reached here, the capability list is corrupted */
	return NULL;
}
