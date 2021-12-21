// SPDX-License-Identifier: GPL-2.0-only
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2021 Intel Corporation
 *
 ****************************************************************************/
#include <linux/firmware.h>
#include <linux/delay.h>

#include "pci.h"
#include "../common/util.h"
#include "../common/boot.h"

#define STR_EQUAL(a, b) !strncmp(a, b, strlen(b))

enum xpcie_stage intel_xpcie_check_magic(struct xpcie_dev *xdev)
{
	char magic[XPCIE_BOOT_MAGIC_STRLEN];

	memcpy_fromio(magic, (void *)xdev->xpcie.io_comm + XPCIE_IO_COMM_MAGIC_OFF,
		      XPCIE_BOOT_MAGIC_STRLEN);
	if (strlen(magic) == 0)
		return STAGE_UNINIT;

	if (STR_EQUAL(magic, XPCIE_BOOT_MAGIC_ROM))
		return STAGE_ROM;

	if (STR_EQUAL(magic, XPCIE_BOOT_MAGIC_EMMC))
		return STAGE_ROM;

	if (STR_EQUAL(magic, XPCIE_BOOT_MAGIC_BL2))
		return STAGE_BL2;

	if (STR_EQUAL(magic, XPCIE_BOOT_MAGIC_UBOOT))
		return STAGE_UBOOT;

	if (STR_EQUAL(magic, XPCIE_BOOT_MAGIC_RECOV))
		return STAGE_RECOV;

	if (STR_EQUAL(magic, XPCIE_BOOT_MAGIC_YOCTO))
		return STAGE_OS;

	return STAGE_UNINIT;
}

void xpcie_device_irq(struct work_struct *work)
{
	struct xpcie_dev *xdev = container_of(work, struct xpcie_dev,
			irq_event);

	if (xdev->boot_notif_fn)
		xdev->boot_notif_fn(xdev->xpcie.sw_devid);
};

int mxlk_pcie_connect_boot_device(const char *dev_name, u32 *phys_dev_id,
				  mxlk_pcie_boot_event notif_fn)
{
	struct xpcie *xpcie = intel_xpcie_get_device_by_name(dev_name);
	struct xpcie_dev *xdev = xpcie_to_xdev(xpcie);

	if (!xdev) {
		pr_err("Invalid dev_name %s\n", dev_name);
		return -EINVAL;
	}

	if (PCI_FUNC(xdev->pci->devfn)) {
		dev_err(&xdev->pci->dev,
			"Invalid PCI function %s\n", dev_name);
		return -EINVAL;
	}

	if (xdev->boot_dev_link) {
		dev_err(&xdev->pci->dev,
			"Already boot device connected for %s\n", dev_name);
		return -EINVAL;
	}

	mutex_lock(&xdev->lock);
	*phys_dev_id = xdev->xpcie.sw_devid;
	xdev->boot_notif_fn = notif_fn;
	xdev->boot_dev_link = true;
	mutex_unlock(&xdev->lock);

	dev_info(&xdev->pci->dev, "Boot device connected devid %x link %d\n",
		 *phys_dev_id, xdev->boot_dev_link);
	return 0;
}
EXPORT_SYMBOL(mxlk_pcie_connect_boot_device);

int mxlk_pcie_boot_mmio_write(u32 phys_dev_id, u32 offset, void *data,
			      size_t size)
{
	struct xpcie *xpcie = intel_xpcie_get_device_by_phys_id(phys_dev_id);
	struct xpcie_dev *xdev = xpcie_to_xdev(xpcie);

	if (!xdev) {
		pr_err("Invalid phys_dev_id %d\n", phys_dev_id);
		return -EINVAL;
	}

	if (!xdev->boot_dev_link) {
		dev_err(&xdev->pci->dev,
			"No boot device connected for id %08x\n", phys_dev_id);
		return -EINVAL;
	}

	mutex_lock(&xdev->lock);
	memcpy_toio((void *)xdev->xpcie.io_comm + offset,
		    data, size);
	mutex_unlock(&xdev->lock);

	return size;
}
EXPORT_SYMBOL(mxlk_pcie_boot_mmio_write);

int mxlk_pcie_boot_mmio_read(u32 phys_dev_id, u32 offset, void *status,
			     size_t size)
{
	struct xpcie *xpcie = intel_xpcie_get_device_by_phys_id(phys_dev_id);
	struct xpcie_dev *xdev = xpcie_to_xdev(xpcie);

	if (!xdev) {
		pr_err("Invalid phys_dev_id %d\n", phys_dev_id);
		return -EINVAL;
	}

	if (!xdev->boot_dev_link) {
		dev_err(&xdev->pci->dev,
			"No boot device connected for id %08x\n", phys_dev_id);
		return -EINVAL;
	}

	mutex_lock(&xdev->lock);
	memcpy_fromio(status, (void *)xdev->xpcie.io_comm + offset, size);
	mutex_unlock(&xdev->lock);

	return size;
}
EXPORT_SYMBOL(mxlk_pcie_boot_mmio_read);

int mxlk_pcie_disconnect_boot_device(u32 phys_dev_id)
{
	struct xpcie *xpcie = intel_xpcie_get_device_by_phys_id(phys_dev_id);
	struct xpcie_dev *xdev = xpcie_to_xdev(xpcie);

	if (!xdev) {
		dev_err(&xdev->pci->dev, "Invalid phys_dev_id %d\n",
			phys_dev_id);
		return -EINVAL;
	}

	if (!xdev->boot_dev_link) {
		dev_err(&xdev->pci->dev,
			"No boot device connected for id %08x\n", phys_dev_id);
		return -EINVAL;
	}

	mutex_lock(&xdev->lock);
	if (xdev->boot_notif_fn)
		xdev->boot_notif_fn = NULL;

	xdev->boot_dev_link = false;
	mutex_unlock(&xdev->lock);

	dev_info(&xdev->pci->dev,
		 "Boot device disconnected devid %x link %d\n", phys_dev_id,
		 xdev->boot_dev_link);
	return 0;
}
EXPORT_SYMBOL(mxlk_pcie_disconnect_boot_device);

void *xlink_pcie_alloc_dma_memory(u32 phys_dev_id, size_t size,
				  dma_addr_t *phys_addr)
{
	struct xpcie *xpcie = intel_xpcie_get_device_by_phys_id(phys_dev_id);
	struct xpcie_dev *xdev = xpcie_to_xdev(xpcie);
	struct device *dev = NULL;
	void *dma_buf = NULL;

	if (!xdev || !phys_addr) {
		dev_err(&xdev->pci->dev, "Invalid input parameters\n");
		return dma_buf;
	}
	dev = &xdev->pci->dev;

	if (!xdev->fl_vbuf) {
		dma_buf = dma_alloc_coherent(dev, size, phys_addr, GFP_KERNEL);
		if (!dma_buf)
			return dma_buf;

		xdev->fl_vbuf = dma_buf;
		xdev->fl_buf_size = size;
		xdev->fl_phys_addr = *phys_addr;
	}

	return dma_buf;
}
EXPORT_SYMBOL(xlink_pcie_alloc_dma_memory);

int xlink_pcie_free_dma_memory(u32 phys_dev_id, size_t size,
			       void *dma_buf, dma_addr_t phys_addr)
{
	struct xpcie *xpcie = intel_xpcie_get_device_by_phys_id(phys_dev_id);
	struct xpcie_dev *xdev = xpcie_to_xdev(xpcie);
	struct device *dev = &xdev->pci->dev;

	if (!xdev || !phys_addr) {
		dev_err(&xdev->pci->dev, "Invalid input parameters\n");
		return -EINVAL;
	}

	if (xdev->fl_vbuf && xdev->fl_vbuf == dma_buf) {
		dma_free_coherent(dev, size, dma_buf, phys_addr);
		xdev->fl_vbuf = NULL;
		xdev->fl_buf_size = 0;
	}

	return 0;
}
EXPORT_SYMBOL(xlink_pcie_free_dma_memory);
