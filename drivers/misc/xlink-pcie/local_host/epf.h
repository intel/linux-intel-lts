/* SPDX-License-Identifier: GPL-2.0-only */
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#ifndef XPCIE_EPF_HEADER_
#define XPCIE_EPF_HEADER_

#include <linux/pci-epc.h>
#include <linux/pci-epf.h>

#include "../common/xpcie.h"
#include "../common/util.h"

#define XPCIE_DRIVER_NAME "mxlk_pcie_epf"
#define XPCIE_DRIVER_DESC "Intel(R) xLink PCIe endpoint function driver"

#define KEEMBAY_XPCIE_STEPPING_MAXLEN 8

#define DMA_CHAN_NUM		(4)

#define XPCIE_NUM_TX_DESCS	(64)
#define XPCIE_NUM_RX_DESCS	(64)

extern bool dma_ll_mode;
extern u32 xlink_sw_id;

struct xpcie_dma_ll_desc {
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

struct xpcie_dma_ll_desc_buf {
	struct xpcie_dma_ll_desc *virt;
	dma_addr_t phys;
	size_t size;
};

struct xpcie_epf {
	struct pci_epf *epf;
	void *vaddr[BAR_5 + 1];
	enum pci_barno comm_bar;
	enum pci_barno bar4;
	const struct pci_epc_features *epc_features;
	struct xpcie xpcie;
	int irq;
	int irq_dma;
	int irq_err;
	void __iomem *apb_base;
	void __iomem *dma_base;
	void __iomem *dbi_base;
	char stepping[KEEMBAY_XPCIE_STEPPING_MAXLEN];

	irq_handler_t			core_irq_callback;
	dma_addr_t			tx_phys;
	void				*tx_virt;
	size_t				tx_size;

	struct xpcie_dma_ll_desc_buf	tx_desc_buf[DMA_CHAN_NUM];
	struct xpcie_dma_ll_desc_buf	rx_desc_buf[DMA_CHAN_NUM];
};

static inline struct device *xpcie_to_dev(struct xpcie *xpcie)
{
	struct xpcie_epf *xpcie_epf = container_of(xpcie,
						   struct xpcie_epf, xpcie);

	return &xpcie_epf->epf->dev;
}

int intel_xpcie_ep_dma_init(struct pci_epf *epf);
int intel_xpcie_ep_dma_uninit(struct pci_epf *epf);
int intel_xpcie_ep_dma_reset(struct pci_epf *epf);
int intel_xpcie_ep_dma_read_ll(struct pci_epf *epf, int chan, int descs_num);
int intel_xpcie_ep_dma_write_ll(struct pci_epf *epf, int chan, int descs_num);

void intel_xpcie_register_host_irq(struct xpcie *xpcie,
				   irq_handler_t func);
int intel_xpcie_raise_irq(struct xpcie *xpcie,
			  enum xpcie_doorbell_type type);
int intel_xpcie_copy_from_host_ll(struct xpcie *xpcie,
				  int chan, int descs_num);
int intel_xpcie_copy_to_host_ll(struct xpcie *xpcie,
				int chan, int descs_num);
#endif /* XPCIE_EPF_HEADER_ */
