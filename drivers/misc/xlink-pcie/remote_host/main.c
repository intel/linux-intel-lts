// SPDX-License-Identifier: GPL-2.0-only
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#include "pci.h"
#include "../common/core.h"

#define HW_ID_LO_MASK	GENMASK(7, 0)
#define HW_ID_HI_MASK	GENMASK(15, 8)

static const struct pci_device_id xpcie_pci_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_KEEMBAY), 0 },
	{ 0 }
};

static int intel_xpcie_probe(struct pci_dev *pdev,
			     const struct pci_device_id *ent)
{
	bool new_device = false;
	struct xpcie_dev *xdev;
	u32 sw_devid;
	u16 hw_id;
	int ret;

	hw_id = FIELD_PREP(HW_ID_HI_MASK, pdev->bus->number) |
		FIELD_PREP(HW_ID_LO_MASK, PCI_SLOT(pdev->devfn));

	sw_devid = FIELD_PREP(XLINK_DEV_INF_TYPE_MASK,
			      XLINK_DEV_INF_PCIE) |
		   FIELD_PREP(XLINK_DEV_PHYS_ID_MASK, hw_id) |
		   FIELD_PREP(XLINK_DEV_TYPE_MASK, XLINK_DEV_TYPE_KMB) |
		   FIELD_PREP(XLINK_DEV_PCIE_ID_MASK, XLINK_DEV_PCIE_0) |
		   FIELD_PREP(XLINK_DEV_FUNC_MASK, XLINK_DEV_FUNC_VPU);

	xdev = intel_xpcie_get_device_by_id(sw_devid);
	if (!xdev) {
		xdev = intel_xpcie_create_device(sw_devid, pdev);
		if (!xdev)
			return -ENOMEM;

		new_device = true;
	}

	ret = intel_xpcie_pci_init(xdev, pdev);
	if (ret) {
		intel_xpcie_remove_device(xdev);
		return ret;
	}

	if (new_device)
		intel_xpcie_list_add_device(xdev);

	intel_xpcie_pci_notify_event(xdev, NOTIFY_DEVICE_CONNECTED);

	return ret;
}

static void intel_xpcie_remove(struct pci_dev *pdev)
{
	struct xpcie_dev *xdev = pci_get_drvdata(pdev);

	if (xdev) {
		intel_xpcie_pci_cleanup(xdev);
		intel_xpcie_pci_notify_event(xdev, NOTIFY_DEVICE_DISCONNECTED);
		intel_xpcie_remove_device(xdev);
	}
}

static struct pci_driver xpcie_driver = {
	.name = XPCIE_DRIVER_NAME,
	.id_table = xpcie_pci_table,
	.probe = intel_xpcie_probe,
	.remove = intel_xpcie_remove
};

static int __init intel_xpcie_init_module(void)
{
	return pci_register_driver(&xpcie_driver);
}

static void __exit intel_xpcie_exit_module(void)
{
	pci_unregister_driver(&xpcie_driver);
}

module_init(intel_xpcie_init_module);
module_exit(intel_xpcie_exit_module);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION(XPCIE_DRIVER_DESC);
