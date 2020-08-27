/* SPDX-License-Identifier: GPL-2.0-only */
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#ifndef MXLK_DMA_HEADER_
#define MXLK_DMA_HEADER_

#include <linux/types.h>
#include <linux/pci-epc.h>
#include <linux/pci-epf.h>

int mxlk_ep_dma_init(struct pci_epf *epf);
int mxlk_ep_dma_uninit(struct pci_epf *epf);
int mxlk_ep_dma_read_ll(struct pci_epf *epf, int chan, int descs_num);
int mxlk_ep_dma_write_ll(struct pci_epf *epf, int chan, int descs_num);
bool mxlk_ep_dma_enabled(struct pci_epf *epf);
int mxlk_ep_dma_reset(struct pci_epf *epf);

#endif // MXLK_DMA_HEADER_
