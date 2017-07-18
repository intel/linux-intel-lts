/*
 * Copyright (c) 2016--2017 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/device.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/module.h>

#include "intel-ipu4.h"
#include "intel-ipu4-cpd.h"
#include "intel-ipu4-isys.h"
#include "intel-ipu4-regs.h"
#include "intel-ipu5-devel.h"
#include "intel-ipu5-regs.h"
#include "intel-ipu4-buttress-regs.h"
#include "isysapi/interface/ia_css_isysapi.h"
#include "vied/vied/shared_memory_map.h"
#include "vied/vied/shared_memory_access.h"

#define IPU5_FPGA_RESET_REG 0x20
#define IPU5_FPGA_RESET_ACTIVE 0x0
#define IPU5_FPGA_RESET_RELEASE 0x1

#define IPU5_FPGA_PCI_BRIDGE_RESET_START 0x9200000
#define IPU5_FPGA_PCI_BRIDGE_RESET_END \
	(IPU5_FPGA_PCI_BRIDGE_RESET_START + PAGE_SIZE - 1)

#define IPU5_BTR_IS_ON 0x80000006
#define IPU5_BTR_PS_ON 0x80070880

void intel_ipu5_fpga_reset(struct pci_dev *pci_dev)
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

unsigned int intel_ipu5_fpga_reset_prepare(struct intel_ipu4_device *isp)
{
	struct pci_dev *bridge;
	phys_addr_t phys;

	/* Only for ipu5 fpga - not for SOC */
	if (!is_intel_ipu_hw_fpga(isp) &&
	    !is_intel_ipu5_hw_a0(isp))
		return 0;

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


void intel_ipu5_fpga_pmclite_btr_power(struct intel_ipu4_device *isp, bool on)
{
	if (!on) {
		writel(0, isp->base + BUTTRESS_REG_PS_FREQ_CTL);
		usleep_range(1000, 1500);
		writel(0, isp->base + BUTTRESS_REG_IS_FREQ_CTL);
	} else {
		writel(IPU5_BTR_IS_ON, isp->base + BUTTRESS_REG_IS_FREQ_CTL);
		usleep_range(1000, 1500);
		writel(IPU5_BTR_PS_ON, isp->base + BUTTRESS_REG_PS_FREQ_CTL);
	}
	usleep_range(1000, 1500);
	dev_dbg(&isp->pdev->dev,
		"set buttress power to %d sts now is 0x%x\n",
		on, readl(isp->base + BUTTRESS_REG_PWR_STATE));

}

static int intel_ipu5_isys_fw_reload(struct intel_ipu4_device *isp)
{
	struct intel_ipu4_isys *isys = intel_ipu4_bus_get_drvdata(isp->isys);

	intel_ipu4_buttress_unmap_fw_image(isp->isys, &isys->fw_sgt);
	intel_ipu5_isys_load_pkg_dir(isys);
}

int intel_ipu5_isys_load_pkg_dir(struct intel_ipu4_isys *isys)
{
	const struct firmware *uninitialized_var(fw);
	u64 *pkg_dir_host_address;
	u64 pkg_dir_vied_address;
	int rval = 0;
	struct intel_ipu4_device *isp = isys->adev->isp;

	fw = isp->cpd_fw;
	if (!fw) {
		dev_err(&isys->adev->dev, "fw is not requested!!!!\n");
		return -EINVAL;
	}

	rval = intel_ipu4_buttress_map_fw_image(
		isys->adev, fw, &isys->fw_sgt);
	if (rval)
		dev_err(&isys->adev->dev, "map_fw_image failed\n");

	pkg_dir_host_address = (u64 *)fw->data;
	dev_dbg(&isys->adev->dev, "pkg_dir_host_addr 0x%lx\n",
		(unsigned long)pkg_dir_host_address);
	if (!pkg_dir_host_address) {
		dev_err(&isys->adev->dev, "invalid pkg_dir_host_addr\n");
		return -ENOMEM;
	}
	pkg_dir_vied_address = sg_dma_address(isys->fw_sgt.sgl);
	dev_dbg(&isys->adev->dev, "pkg_dir_vied_addr 0x%lx\n",
		(unsigned long)pkg_dir_vied_address);
	if (!pkg_dir_vied_address) {
		dev_err(&isys->adev->dev, "invalid pkg_dir_vied_addr\n");
		return -ENOMEM;
	}

	isys->pkg_dir = pkg_dir_host_address;
	isys->pkg_dir_size = fw->size;
	isys->pkg_dir_dma_addr = pkg_dir_vied_address;

	/* Let's tell how to reload isys FW binary */
	isp->isys_fw_reload = intel_ipu5_isys_fw_reload;

	return 0;
}
EXPORT_SYMBOL(intel_ipu5_isys_load_pkg_dir);

void intel_ipu5_pkg_dir_configure_spc(struct intel_ipu4_device *isp,
			const struct intel_ipu4_hw_variants *hw_variant,
			int pkg_dir_idx, void __iomem *base,
			u64 *pkg_dir,
			dma_addr_t pkg_dir_dma_addr)
{
	u64 *pkg_dir_host_address;
	u64 pkg_dir_vied_address;
	u64 pg_host_address;
	unsigned int pg_offset;
	u32 val;
	struct ia_css_cell_program_s prog;

	void *__iomem spc_base;

	val = readl(base + hw_variant->spc_offset +
		    INTEL_IPU5_PSYS_REG_SPC_STATUS_CTRL);
	val |= INTEL_IPU5_PSYS_SPC_STATUS_CTRL_ICACHE_INVALIDATE;
	writel(val, base + hw_variant->spc_offset +
	       INTEL_IPU5_PSYS_REG_SPC_STATUS_CTRL);

	pkg_dir_host_address = pkg_dir;
	if (!pkg_dir_host_address)
		dev_err(&isp->pdev->dev, "invalid pkg_dir_host_address\n");

	pkg_dir_vied_address = pkg_dir_dma_addr;
	if (!pkg_dir_vied_address)
		dev_err(&isp->pdev->dev, "invalid pkg_dir_vied_address\n");

	pg_offset = intel_ipu4_cpd_pkg_dir_get_address(pkg_dir, pkg_dir_idx);

	pg_host_address = (u64)pkg_dir_host_address + pg_offset;
	shared_memory_load(pkg_dir_idx, (host_virtual_address_t)pg_host_address,
			   &prog, sizeof(struct ia_css_cell_program_s));
	spc_base = base + prog.regs_addr;
	dev_dbg(&isp->pdev->dev, "idx %d spc:0x%p blob_offset/size 0x%x/0x%x",
		 pkg_dir_idx,
		 spc_base,
		 prog.blob_offset,
		 prog.blob_size);
	dev_dbg(&isp->pdev->dev, "start_pc:0x%x icache_src 0x%x regs:0x%x",
		 prog.start[1],
		 prog.icache_source,
		 (unsigned int)prog.regs_addr);
	writel(pkg_dir_vied_address + prog.blob_offset +
		     prog.icache_source + pg_offset,
		     spc_base + INTEL_IPU5_PSYS_REG_SPC_ICACHE_BASE);
	writel(INTEL_IPU5_INFO_REQUEST_DESTINATION_PRIMARY,
		     spc_base +
		     INTEL_IPU5_REG_PSYS_INFO_SEG_0_CONFIG_ICACHE_MASTER);
	writel(prog.start[1],
		     spc_base + INTEL_IPU5_PSYS_REG_SPC_START_PC);
	writel(INTEL_IPU5_INFO_REQUEST_DESTINATION_PRIMARY,
		     spc_base +
		     INTEL_IPU5_REG_PSYS_INFO_SEG_0_CONFIG_ICACHE_MASTER);
	writel(pkg_dir_vied_address,
	       base + hw_variant->dmem_offset);
}

MODULE_AUTHOR("Samu Onkalo <samu.onkalo@intel.com>");
MODULE_AUTHOR("Zaikuo Wang <zaikuo.wang@intel.com>");
MODULE_AUTHOR("Bingbu Cao <bingbu.cao@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Intel ipu5 development wrapper");
