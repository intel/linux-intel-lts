/* SPDX-License-Identifier: GPL-2.0-only */
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#ifndef MXLK_HEADER_
#define MXLK_HEADER_

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/stddef.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/version.h>
#include <linux/mempool.h>
#include <linux/dma-mapping.h>
#include <linux/cache.h>
#include <linux/wait.h>

#include "mxlk_common.h"
#include "mxlk_boot.h"

#ifdef XLINK_PCIE_REMOTE
#define MXLK_DRIVER_NAME "mxlk"
#define MXLK_DRIVER_DESC "Intel(R) Keem Bay XLink PCIe driver"
#else
#define MXLK_DRIVER_NAME "mxlk_pcie_epf"
#define MXLK_DRIVER_DESC "Intel(R) xLink PCIe endpoint function driver"
#endif

struct mxlk_pipe {
	u32 old;
	u32 ndesc;
	u32 *head;
	u32 *tail;
	struct mxlk_transfer_desc *tdr;
};

struct mxlk_buf_desc {
	struct mxlk_buf_desc *next;
	void *head;
	dma_addr_t phys;
	size_t true_len;
	void *data;
	size_t length;
	int interface;
	bool own_mem;
};

struct mxlk_stream {
	size_t frag;
	struct mxlk_pipe pipe;
#ifdef XLINK_PCIE_REMOTE
	struct mxlk_buf_desc **ddr;
#endif
};

struct mxlk_list {
	spinlock_t lock;
	size_t bytes;
	size_t buffers;
	struct mxlk_buf_desc *head;
	struct mxlk_buf_desc *tail;
};

struct mxlk_interface {
	int id;
	struct mxlk *mxlk;
	struct mutex rlock;
	struct mxlk_list read;
	struct mxlk_buf_desc *partial_read;
	bool data_available;
	wait_queue_head_t rx_waitqueue;
};

struct mxlk_debug_stats {
	struct {
		size_t cnts;
		size_t bytes;
	} tx_krn, rx_krn, tx_usr, rx_usr;
	size_t send_ints;
	size_t interrupts;
	size_t rx_event_runs;
	size_t tx_event_runs;
};

struct mxlk {
	u32 status;
	bool legacy_a0;

#ifdef XLINK_PCIE_REMOTE
	void __iomem *bar0;
	struct mxlk_bootio __iomem *io_comm; /* IO communication space */
	struct mxlk_mmio __iomem *mmio; /* XLink memory space */
	void __iomem *bar4;
#else
	struct mxlk_bootio *io_comm; /* IO communication space */
	struct mxlk_mmio *mmio; /* XLink memory space */
	void *bar4;
#endif

	struct workqueue_struct *rx_wq;
	struct workqueue_struct *tx_wq;

	struct mxlk_interface interfaces[MXLK_NUM_INTERFACES];

	size_t fragment_size;
	struct mxlk_cap_txrx *txrx;
	struct mxlk_stream tx;
	struct mxlk_stream rx;

	struct mutex wlock;
	struct mxlk_list write;
	bool no_tx_buffer;
	wait_queue_head_t tx_waitqueue;
	bool tx_pending;
	bool stop_flag;

	struct mxlk_list rx_pool;
	struct mxlk_list tx_pool;

	struct delayed_work rx_event;
	struct delayed_work tx_event;

	struct device_attribute debug;
	bool debug_enable;
	struct mxlk_debug_stats stats;
};

#endif
