// SPDX-License-Identifier: GPL-2.0
/*
 * Intel Platform Monitory Technology Telemetry driver
 *
 * Copyright (c) 2022, Intel Corporation.
 * All Rights Reserved.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>

static const struct pci_device_id intel_vsec_dummy_pci_ids[] = {
	{ PCI_VDEVICE(INTEL, 0x09a6) }, // OOBMSM Function 0
	{ PCI_VDEVICE(INTEL, 0x09a8) }, // OOBMSM Function 2
	{ }
};
MODULE_DEVICE_TABLE(pci, intel_vsec_dummy_pci_ids);


static int intel_vsec_dummy_pci_probe(struct pci_dev *pdev,
					const struct pci_device_id *id)
{
	return 0;
}

static void intel_vsec_dummy_pci_remove(struct pci_dev *pdev) { }

static pci_ers_result_t
intel_vsec_dummy_pci_error_detected(struct pci_dev *pdev,
				      pci_channel_state_t state)
{
	dev_info(&pdev->dev, "PCI Error Detected\n");
	return PCI_ERS_RESULT_RECOVERED;
}

static pci_ers_result_t intel_vsec_dummy_pci_slot_reset(struct pci_dev *pdev)
{
	dev_info(&pdev->dev, "PCI Slot Reset\n");
	return PCI_ERS_RESULT_RECOVERED;
}

void intel_vsec_dummy_pci_resume(struct pci_dev *pdev)
{
	dev_info(&pdev->dev, "PCI Resume\n");
}

const struct pci_error_handlers intel_vsec_dummy_pci_err_handlers = {
	.error_detected = intel_vsec_dummy_pci_error_detected,
	.slot_reset = intel_vsec_dummy_pci_slot_reset,
	.resume = intel_vsec_dummy_pci_resume,
};

static struct pci_driver intel_vsec_dummy_pci_driver = {
	.name = "intel_vsec_dummy",
	.id_table = intel_vsec_dummy_pci_ids,
	.probe = intel_vsec_dummy_pci_probe,
	.remove = intel_vsec_dummy_pci_remove,
	.err_handler = &intel_vsec_dummy_pci_err_handlers,
	.driver = { },
};

module_pci_driver(intel_vsec_dummy_pci_driver);

MODULE_AUTHOR("Slawomir Blauciak <slawomir.blauciak@intel.com>");
MODULE_DESCRIPTION("Intel OOBMSM Func0/2 Dummy driver");
MODULE_LICENSE("GPL v2");
