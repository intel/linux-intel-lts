// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright © 2022 Intel Corporation
 */
#include <linux/intel_vsec.h>

#include "i915_drv.h"
#include "intel_vsec.h"

#define SOC_BASE		0x280000

#define P2SB_CFG_BASE		0xF7000
#define P2SB_CFG_DVSEC1		0x100
#define P2SB_DVSEC1_OFFSET	(SOC_BASE + P2SB_CFG_BASE + P2SB_CFG_DVSEC1)

#define PMT_BASE		0xE8000
#define DISCOVERY_START		0x6000
#define DISCOVERY_OFFSET	(SOC_BASE + PMT_BASE + DISCOVERY_START)

#define TELEM_START		0x4000
#define TELEM_OFFSET		(SOC_BASE + PMT_BASE + TELEM_START)
#define TELEM_BASE_OFFSET	0x8

#define GFX_BAR			0

static struct intel_vsec_header dg2_telemetry = {
	.length = 0x10,
	.entry_size = 3,
	.tbir = GFX_BAR,
	.offset = DISCOVERY_OFFSET,
	.id = VSEC_ID_TELEMETRY,
	.num_entries = 1,
};

static struct intel_vsec_header *dg2_capabilities[] = {
	&dg2_telemetry,
	NULL
};

static struct intel_vsec_platform_info dg2_info = {
	.capabilities = dg2_capabilities,
	.quirks = VSEC_QUIRK_EARLY_HW,
};

/**
 * intel_vsec_init - Initialize resources and add intel_vsec auxiliary
 * interface
 * @i915: valid i915 instance
 */
void intel_vsec_init(struct drm_i915_private *dev_priv)
{
	struct device *dev = dev_priv->drm.dev;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct resource res;
	void __iomem *base;
	u32 telem_offset;
	u64 addr;

	if (!IS_DG2(dev_priv))
		return;

	/*
	 * Access the PMT MMIO discovery table
	 *
	 * The intel_vsec driver does not typically access the discovery table.
	 * Instead, it creates a memory resource for the table and passes it
	 * to the PMT telemetry driver. Each discovery table contains 3 items,
	 *    - GUID
	 *    - Telemetry size
	 *    - Telemetry offset (offset from P2SB BAR, not GT)
	 *
	 * For DG2 we know what the telemetry offset is, but we still need to
	 * use the discovery table to pass the GUID and the size. So figure
	 * out the difference between the P2SB offset and the GT offset and
	 * save this so that the telemetry driver can use it to adjust the
	 * value.
	 */
	addr = pci_resource_start(pdev, GFX_BAR) + DISCOVERY_OFFSET;
	res = (struct resource)DEFINE_RES_MEM(addr, 16);
	base = devm_ioremap_resource(dev, &res);
	if (IS_ERR(base))
		return;

	telem_offset = readl(base + TELEM_BASE_OFFSET);
	if (TELEM_OFFSET > telem_offset)
		dg2_info.base_adjust = -(TELEM_OFFSET - telem_offset);
	else
		dg2_info.base_adjust = -(telem_offset - TELEM_OFFSET);

	devm_iounmap(dev, base);
	devm_release_mem_region(dev, res.start, resource_size(&res));

	/*
	 * Register a VSEC. Cleanup is handled using device managed
	 * resources.
	 */
	intel_vsec_register(pdev, &dg2_info);
}
MODULE_IMPORT_NS(INTEL_VSEC);
