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

#include "xpcie.h"

#define XPCIE_DRIVER_NAME "mxlk_pcie_epf"
#define XPCIE_DRIVER_DESC "Intel(R) xLink PCIe endpoint function driver"

#define KEEMBAY_XPCIE_STEPPING_MAXLEN 8

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
};

#endif /* XPCIE_EPF_HEADER_ */
