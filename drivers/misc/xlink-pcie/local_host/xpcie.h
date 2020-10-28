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

#ifndef PCI_DEVICE_ID_INTEL_KEEMBAY
#define PCI_DEVICE_ID_INTEL_KEEMBAY 0x6240
#endif

#define XPCIE_IO_COMM_SIZE SZ_16K
#define XPCIE_MMIO_OFFSET SZ_4K

/* MMIO layout and offsets shared between device and host */
struct xpcie_mmio {
	u8 legacy_a0;
} __packed;

#define XPCIE_MMIO_LEGACY_A0	(offsetof(struct xpcie_mmio, legacy_a0))

struct xpcie {
	u32 status;
	bool legacy_a0;
	void *mmio;
	void *bar4;
};

#endif /* XPCIE_HEADER_ */
