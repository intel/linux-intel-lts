/* SPDX-License-Identifier: GPL-2.0-only */
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#ifndef XPCIE_CORE_HEADER_
#define XPCIE_CORE_HEADER_

#include <linux/dma-mapping.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <linux/xlink_drv_inf.h>

/* Number of interfaces to statically allocate resources for */
#define XPCIE_NUM_INTERFACES (1)

/* max should be always power of '2' */
#define XPCIE_CIRCULAR_INC(val, max) (((val) + 1) & ((max) - 1))

#define XPCIE_FRAGMENT_SIZE	SZ_128K

/* Status encoding of the transfer descriptors */
#define XPCIE_DESC_STATUS_SUCCESS	(0)
#define XPCIE_DESC_STATUS_ERROR		(0xFFFF)

/* Layout transfer descriptors used by device and host */
struct xpcie_transfer_desc {
	u64 address;
	u32 length;
	u16 status;
	u16 interface;
} __packed;

struct xpcie_pipe {
	u32 old;
	u32 ndesc;
	u32 *head;
	u32 *tail;
	struct xpcie_transfer_desc *tdr;
};

struct xpcie_buf_desc {
	struct xpcie_buf_desc *next;
	void *head;
	dma_addr_t phys;
	size_t true_len;
	void *data;
	size_t length;
	int interface;
	bool own_mem;
};

struct xpcie_stream {
	size_t frag;
	struct xpcie_pipe pipe;
	struct xpcie_buf_desc **ddr;
};

struct xpcie_list {
	spinlock_t lock; /* list lock */
	size_t bytes;
	size_t buffers;
	struct xpcie_buf_desc *head;
	struct xpcie_buf_desc *tail;
};

struct xpcie_interface {
	int id;
	struct xpcie *xpcie;
	struct mutex rlock; /* read lock */
	struct xpcie_list read;
	struct xpcie_buf_desc *partial_read;
	bool data_avail;
	wait_queue_head_t rx_waitq;
};

struct xpcie_debug_stats {
	struct {
		size_t cnts;
		size_t bytes;
	} tx_krn, rx_krn, tx_usr, rx_usr;
	size_t send_ints;
	size_t interrupts;
	size_t rx_event_runs;
	size_t tx_event_runs;
};

/* Defined capabilities located in mmio space */
#define XPCIE_CAP_NULL (0)
#define XPCIE_CAP_TXRX (1)

#define XPCIE_CAP_TTL (32)
#define XPCIE_CAP_HDR_ID	(offsetof(struct xpcie_cap_hdr, id))
#define XPCIE_CAP_HDR_NEXT	(offsetof(struct xpcie_cap_hdr, next))

/* Header at the beginning of each capability to define and link to next */
struct xpcie_cap_hdr {
	u16 id;
	u16 next;
} __packed;

struct xpcie_cap_pipe {
	u32 ring;
	u32 ndesc;
	u32 head;
	u32 tail;
} __packed;

/* Transmit and Receive capability */
struct xpcie_cap_txrx {
	struct xpcie_cap_hdr hdr;
	u32 fragment_size;
	struct xpcie_cap_pipe tx;
	struct xpcie_cap_pipe rx;
} __packed;

static inline u64 _ioread64(void __iomem *addr)
{
	u64 low, high;

	low = ioread32(addr);
	high = ioread32(addr + sizeof(u32));

	return low | (high << 32);
}

static inline void _iowrite64(u64 value, void __iomem *addr)
{
	iowrite32(value, addr);
	iowrite32(value >> 32, addr + sizeof(u32));
}

#define intel_xpcie_iowrite64(value, addr) \
			_iowrite64(value, (void __iomem *)addr)
#define intel_xpcie_iowrite32(value, addr) \
			iowrite32(value, (void __iomem *)addr)
#define intel_xpcie_iowrite16(value, addr) \
			iowrite16(value, (void __iomem *)addr)
#define intel_xpcie_iowrite8(value, addr) \
			iowrite8(value, (void __iomem *)addr)
#define intel_xpcie_ioread64(addr) \
			_ioread64((void __iomem *)addr)
#define intel_xpcie_ioread32(addr) \
			ioread32((void __iomem *)addr)
#define intel_xpcie_ioread16(addr) \
			ioread16((void __iomem *)addr)
#define intel_xpcie_ioread8(addr) \
			ioread8((void __iomem *)addr)

static inline
void intel_xpcie_set_td_address(struct xpcie_transfer_desc *td, u64 address)
{
	intel_xpcie_iowrite64(address, &td->address);
}

static inline
u64 intel_xpcie_get_td_address(struct xpcie_transfer_desc *td)
{
	return intel_xpcie_ioread64(&td->address);
}

static inline
void intel_xpcie_set_td_length(struct xpcie_transfer_desc *td, u32 length)
{
	intel_xpcie_iowrite32(length, &td->length);
}

static inline
u32 intel_xpcie_get_td_length(struct xpcie_transfer_desc *td)
{
	return intel_xpcie_ioread32(&td->length);
}

static inline
void intel_xpcie_set_td_interface(struct xpcie_transfer_desc *td, u16 interface)
{
	intel_xpcie_iowrite16(interface, &td->interface);
}

static inline
u16 intel_xpcie_get_td_interface(struct xpcie_transfer_desc *td)
{
	return intel_xpcie_ioread16(&td->interface);
}

static inline
void intel_xpcie_set_td_status(struct xpcie_transfer_desc *td, u16 status)
{
	intel_xpcie_iowrite16(status, &td->status);
}

static inline
u16 intel_xpcie_get_td_status(struct xpcie_transfer_desc *td)
{
	return intel_xpcie_ioread16(&td->status);
}

static inline
void intel_xpcie_set_tdr_head(struct xpcie_pipe *p, u32 head)
{
	intel_xpcie_iowrite32(head, p->head);
}

static inline
u32 intel_xpcie_get_tdr_head(struct xpcie_pipe *p)
{
	return intel_xpcie_ioread32(p->head);
}

static inline
void intel_xpcie_set_tdr_tail(struct xpcie_pipe *p, u32 tail)
{
	intel_xpcie_iowrite32(tail, p->tail);
}

static inline
u32 intel_xpcie_get_tdr_tail(struct xpcie_pipe *p)
{
	return intel_xpcie_ioread32(p->tail);
}

int intel_xpcie_core_init(struct xpcie *xpcie);
void intel_xpcie_core_cleanup(struct xpcie *xpcie);
int intel_xpcie_core_read(struct xpcie *xpcie, void *buffer, size_t *length,
			  u32 timeout_ms);
int intel_xpcie_core_write(struct xpcie *xpcie, void *buffer, size_t *length,
			   u32 timeout_ms);
u32 intel_xpcie_get_device_num(u32 *id_list);
struct xpcie_dev *intel_xpcie_get_device_by_id(u32 id);
int intel_xpcie_get_device_name_by_id(u32 id, char *device_name,
				      size_t name_size);
int intel_xpcie_get_device_status_by_id(u32 id, u32 *status);
int intel_xpcie_pci_connect_device(u32 id);
int intel_xpcie_pci_read(u32 id, void *data, size_t *size, u32 timeout);
int intel_xpcie_pci_write(u32 id, void *data, size_t *size, u32 timeout);
int intel_xpcie_pci_reset_device(u32 id);
int intel_xpcie_pci_register_device_event(u32 sw_device_id,
					  xlink_device_event event_notif_fn);
int intel_xpcie_pci_unregister_device_event(u32 sw_device_id);
#endif /* XPCIE_CORE_HEADER_ */
