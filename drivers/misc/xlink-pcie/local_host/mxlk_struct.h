/* SPDX-License-Identifier: GPL-2.0-only */
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#ifndef MXLK_STRUCT_HEADER_
#define MXLK_STRUCT_HEADER_

#include <linux/pci-epc.h>
#include <linux/pci-epf.h>
#include <pcie-keembay.h>
#include "../common/mxlk.h"

extern bool dma_ll_mode;

struct mxlk_dma_ll_desc {
	u32 dma_ch_control1;
	u32 dma_transfer_size;
	union {
		struct {
			u32 dma_sar_low;
			u32 dma_sar_high;
		};
		phys_addr_t src_addr;
	};
	union {
		struct {
			u32 dma_dar_low;
			u32 dma_dar_high;
		};
		phys_addr_t dst_addr;
	};
} __packed;

struct mxlk_dma_ll_desc_buf {
	struct mxlk_dma_ll_desc *virt;
	dma_addr_t phys;
	size_t size;
};

struct mxlk_epf {
	struct pci_epf			*epf;
	void				*vaddr[BAR_5 + 1];
	enum pci_barno			comm_bar;
	enum pci_barno			bar4;
	const struct pci_epc_features	*epc_features;
	struct mxlk			mxlk;
	int				irq;
	int				irq_dma;
	int				irq_err;
	void __iomem			*apb_base;
	void __iomem			*dma_base;

	irq_handler_t			core_irq_callback;
	dma_addr_t			tx_phys;
	void				*tx_virt;
	size_t				tx_size;
	dma_addr_t			rx_phys;
	void				*rx_virt;
	size_t				rx_size;

	struct mxlk_dma_ll_desc_buf	tx_desc_buf[4];
	struct mxlk_dma_ll_desc_buf	rx_desc_buf[4];
};

static inline struct device *mxlk_to_dev(struct mxlk *mxlk)
{
	struct mxlk_epf *mxlk_epf = container_of(mxlk, struct mxlk_epf, mxlk);

	return &mxlk_epf->epf->dev;
}

#endif // MXLK_STRUCT_HEADER_
