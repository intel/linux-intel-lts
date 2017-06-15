/*
 * Copyright (c) 2017 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/delay.h>

#include "intel-ipu-fpga.h"

static int intel_ipu_get_sim_type(void)
{
	return SIM_FPGA;
}

static unsigned int intel_ipu_fpga_reset_prepare(struct intel_ipu4_device *isp)
{
	struct pci_dev *bridge;
	phys_addr_t phys;

	/* Search FPGA PCI bridge. Reset is done via bridge IO space */
	bridge = pci_get_device(PCI_VENDOR_ID_INTEL,
				INTEL_IPU5_HW_PCI_FPGA_BRIDGE, NULL);
	if (!bridge)
		return 0;

	dev_info(&isp->pdev->dev,
		 "Device 0x%x (rev: 0x%x) is a bridge for ipu\n",
		 bridge->device, bridge->revision);

	/*
	 * HACK - search bridge IO resources and map a page from there
	 * to IPU io resources. This is only for fpga - probably not
	 * worth of separate driver just for reset.
	 */
	phys = pci_resource_start(bridge, 0);

	pci_resource_start(isp->pdev, IPU5_BAR_FOR_BRIDGE) = phys +
		IPU5_FPGA_PCI_BRIDGE_RESET_START;
	pci_resource_end(isp->pdev,  IPU5_BAR_FOR_BRIDGE) = phys +
		IPU5_FPGA_PCI_BRIDGE_RESET_END;
	pci_resource_flags(isp->pdev, IPU5_BAR_FOR_BRIDGE) =
		pci_resource_flags(isp->pdev, 0);

	return 1 << IPU5_BAR_FOR_BRIDGE;
}

static void intel_ipu_fpga_reset(struct pci_dev *pci_dev)
{
	struct intel_ipu4_device *isp = pci_get_drvdata(pci_dev);

	dev_info(&pci_dev->dev, "IP reset for FPGA\n");

	pci_save_state(pci_dev);

	readl(isp->base2 + IPU5_FPGA_RESET_REG);
	writel(IPU5_FPGA_RESET_ACTIVE, isp->base2 + IPU5_FPGA_RESET_REG);
	usleep_range(1000, 1100);

	readl(isp->base2 + IPU5_FPGA_RESET_REG);
	writel(IPU5_FPGA_RESET_RELEASE, isp->base2 + IPU5_FPGA_RESET_REG);
	usleep_range(1000, 1100);

	readl(isp->base2 + IPU5_FPGA_RESET_REG);
	usleep_range(1000, 1100);

	pci_restore_state(pci_dev);

	dev_info(&pci_dev->dev, "IP reset for FPGA done\n");
}

const struct intel_ipu_sim_ctrl sim_ctrl_ops = {
	.get_sim_type = intel_ipu_get_sim_type,
	.reset_prepare = intel_ipu_fpga_reset_prepare,
	.reset = intel_ipu_fpga_reset,
};
EXPORT_SYMBOL_GPL(sim_ctrl_ops);

static int __init intel_ipu_fpga_init(void)
{
	return 0;
}

static void __exit intel_ipu_fpga_exit(void)
{
}

module_init(intel_ipu_fpga_init);
module_exit(intel_ipu_fpga_exit);

MODULE_AUTHOR("Yunliang Ding <yunliang.ding@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Intel IPU FPGA library");
