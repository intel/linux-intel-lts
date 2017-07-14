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

/*
 * Put ipu5 isys development time tricks and hacks to this file
 */

#ifndef __INTEL_IPU5_ISYS_DEVEL_H__
#define __INTEL_IPU5_ISYS_DEVEL_H__

struct intel_ipu4_isys;

#define IPU5_BAR_FOR_BRIDGE 1

#if IS_ENABLED(CONFIG_VIDEO_INTEL_IPU5_FPGA)
void intel_ipu5_fpga_pmclite_btr_power(struct intel_ipu4_device *isp, bool on);
#else
static void intel_ipu5_fpga_pmclite_btr_power(struct intel_ipu4_device *isp,
					      bool on)
{
}
#endif

#if IS_ENABLED(CONFIG_VIDEO_INTEL_IPU5)
void intel_ipu5_fpga_reset(struct pci_dev *pci_dev);
unsigned int intel_ipu5_fpga_reset_prepare(struct intel_ipu4_device *isp);
int intel_ipu5_isys_load_pkg_dir(struct intel_ipu4_isys *isys);
void intel_ipu5_pkg_dir_configure_spc(struct intel_ipu4_device *isp,
			const struct intel_ipu4_hw_variants *hw_variant,
			int pkg_dir_idx, void __iomem *base,
			u64 *pkg_dir,
			dma_addr_t pkg_dir_dma_addr);

#else
static void intel_ipu5_fpga_reset(struct pci_dev *pci_dev)
{
}

static unsigned int intel_ipu5_fpga_reset_prepare(struct intel_ipu4_device *isp)
{
	return 0;
}

static int intel_ipu5_isys_load_pkg_dir(struct intel_ipu4_isys *isys)
{
	return 0;
}
static void intel_ipu5_pkg_dir_configure_spc(struct intel_ipu4_device *isp,
				const struct intel_ipu4_hw_variants *hw_variant,
				int pkg_dir_idx, void __iomem *base,
				u64 *pkg_dir,
				dma_addr_t pkg_dir_dma_addr)
{
}

#endif
#endif
