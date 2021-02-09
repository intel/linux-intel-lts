/* SPDX-License-Identifier: GPL-2.0-only */
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#ifndef XPCIE_HEADER_
#define XPCIE_HEADER_

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci_ids.h>

#include "core.h"

#ifndef PCI_DEVICE_ID_INTEL_KEEMBAY
#define PCI_DEVICE_ID_INTEL_KEEMBAY 0x6240
#endif

#define XPCIE_IO_COMM_SIZE SZ_16K
#define XPCIE_MMIO_OFFSET SZ_4K

/* Status encoding of both device and host */
#define XPCIE_STATUS_ERROR     (0xFFFFFFFF)
#define XPCIE_STATUS_UNINIT    (0)

#define XPCIE_STATUS_BOOT_FW	(1)
#define XPCIE_STATUS_BOOT_OS	(2)
#define XPCIE_STATUS_READY	(3)
#define XPCIE_STATUS_RECOVERY	(4)
#define XPCIE_STATUS_RUN	(5)
#define XPCIE_STATUS_OFF	(6)
#define XPCIE_STATUS_BOOT_PRE_OS (7)

#define XPCIE_MAGIC_STRLEN	(16)
#define XPCIE_MAGIC_YOCTO	"VPUYOCTO"

/* MMIO layout and offsets shared between device and host */
struct xpcie_mmio {
	u32 device_status;
	u32 host_status;
	u8 legacy_a0;
	u8 htod_tx_doorbell;
	u8 htod_rx_doorbell;
	u8 htod_event_doorbell;
	u8 dtoh_tx_doorbell;
	u8 dtoh_rx_doorbell;
	u8 dtoh_event_doorbell;
	u8 reserved;
	u32 cap_offset;
} __packed;

#define XPCIE_MMIO_DEV_STATUS	(offsetof(struct xpcie_mmio, device_status))
#define XPCIE_MMIO_HOST_STATUS	(offsetof(struct xpcie_mmio, host_status))
#define XPCIE_MMIO_LEGACY_A0	(offsetof(struct xpcie_mmio, legacy_a0))
#define XPCIE_MMIO_HTOD_TX_DOORBELL \
	(offsetof(struct xpcie_mmio, htod_tx_doorbell))
#define XPCIE_MMIO_HTOD_RX_DOORBELL \
	(offsetof(struct xpcie_mmio, htod_rx_doorbell))
#define XPCIE_MMIO_HTOD_EVENT_DOORBELL \
	(offsetof(struct xpcie_mmio, htod_event_doorbell))
#define XPCIE_MMIO_DTOH_TX_DOORBELL \
	(offsetof(struct xpcie_mmio, dtoh_tx_doorbell))
#define XPCIE_MMIO_DTOH_RX_DOORBELL \
	(offsetof(struct xpcie_mmio, dtoh_rx_doorbell))
#define XPCIE_MMIO_DTOH_EVENT_DOORBELL \
	(offsetof(struct xpcie_mmio, dtoh_event_doorbell))
#define XPCIE_MMIO_CAP_OFF	(offsetof(struct xpcie_mmio, cap_offset))

struct xpcie {
	u32 status;
	bool legacy_a0;
	void *bar0;
	/* IO communication space */
	void *io_comm;
	void *mmio; /* XLink memory space */
	void *bar4;

	struct workqueue_struct *rx_wq;
	struct workqueue_struct *tx_wq;

	struct xpcie_interface interfaces[XPCIE_NUM_INTERFACES];

	size_t fragment_size;
	struct xpcie_cap_txrx *txrx;
	struct xpcie_stream tx;
	struct xpcie_stream rx;

	struct mutex wlock; /* write lock */
	struct xpcie_list write;
	bool no_tx_buffer;
	wait_queue_head_t tx_waitq;
	bool tx_pending;
	bool stop_flag;

	struct xpcie_list rx_pool;
	struct xpcie_list tx_pool;

	struct delayed_work rx_event;
	struct delayed_work tx_event;
};

#endif /* XPCIE_HEADER_ */
