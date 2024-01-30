// SPDX-License-Identifier: MIT
/*
 * Copyright(c) 2019-2022, Intel Corporation. All rights reserved.
 */

#include <linux/irq.h>
#include "i915_reg.h"
#include "i915_drv.h"
#include "spi/intel_spi.h"

#define GEN12_GUNIT_SPI_SIZE 0x80
#define HECI_FW_STATUS_2_SPI_ACCESS_MODE BIT(3)

static const struct i915_spi_region regions[I915_SPI_REGIONS] = {
	[0] = { .name = "DESCRIPTOR", },
	[2] = { .name = "GSC", },
	[11] = { .name = "OptionROM", },
	[12] = { .name = "DAM", },
	[13] = { .name = "PSC", },
};

static void i915_spi_release_dev(struct device *dev)
{
}

static bool i915_spi_writeable_override(struct drm_i915_private *dev_priv)
{
	struct pci_dev *pdev = to_pci_dev(dev_priv->drm.dev);
	resource_size_t base;
	bool writeable_override;

	if (IS_DG1(dev_priv)) {
		base = DG1_GSC_HECI2_BASE;
	} else if (IS_DG2(dev_priv)) {
		base = DG2_GSC_HECI2_BASE;
	} else if (IS_PONTEVECCHIO(dev_priv)) {
		base = PVC_GSC_HECI2_BASE;
	} else {
		dev_err(&pdev->dev, "Unknown platform\n");
		return true;
	}

	writeable_override =
		!(intel_uncore_read(&dev_priv->uncore, HECI_FW_STATUS_2(base)) &
		  HECI_FW_STATUS_2_SPI_ACCESS_MODE);
	if (writeable_override)
		dev_info(&pdev->dev, "SPI access overridden by jumper\n");
	return writeable_override;
}

void intel_spi_init(struct intel_spi *spi, struct drm_i915_private *dev_priv)
{
	struct pci_dev *pdev = to_pci_dev(dev_priv->drm.dev);
	struct auxiliary_device *aux_dev = &spi->aux_dev;
	int ret;

	/* Only the DGFX devices have internal SPI */
	if (!IS_DGFX(dev_priv))
		return;
	/* No access to internal SPI from VFs */
	if (IS_SRIOV_VF(dev_priv))
		return;

	spi->writeable_override = i915_spi_writeable_override(dev_priv);
	spi->bar.parent = &pdev->resource[0];
	spi->bar.start = GEN12_GUNIT_SPI_BASE + pdev->resource[0].start;
	spi->bar.end = spi->bar.start + GEN12_GUNIT_SPI_SIZE - 1;
	spi->bar.flags = IORESOURCE_MEM;
	spi->bar.desc = IORES_DESC_NONE;
	spi->regions = regions;

	aux_dev->name = "spi";
	aux_dev->id = (pci_domain_nr(pdev->bus) << 16) |
		       PCI_DEVID(pdev->bus->number, pdev->devfn);
	aux_dev->dev.parent = &pdev->dev;
	aux_dev->dev.release = i915_spi_release_dev;

	ret = auxiliary_device_init(aux_dev);
	if (ret) {
		dev_err(&pdev->dev, "i915-spi aux init failed %d\n", ret);
		return;
	}

	ret = auxiliary_device_add(aux_dev);
	if (ret) {
		dev_err(&pdev->dev, "i915-spi aux add failed %d\n", ret);
		auxiliary_device_uninit(aux_dev);
		return;
	}

	spi->i915 = dev_priv;
}

void intel_spi_fini(struct intel_spi *spi)
{
	struct pci_dev *pdev;

	if (!spi->i915)
		return;

	pdev = to_pci_dev(spi->i915->drm.dev);

	dev_dbg(&pdev->dev, "removing i915-spi cell\n");

	auxiliary_device_delete(&spi->aux_dev);
	auxiliary_device_uninit(&spi->aux_dev);
}
