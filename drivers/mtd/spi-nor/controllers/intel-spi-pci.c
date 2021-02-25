// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel PCH/PCU SPI flash PCI driver.
 *
 * Copyright (C) 2016, Intel Corporation
 * Author: Mika Westerberg <mika.westerberg@linux.intel.com>
 */

#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>

#include "intel-spi.h"

#define BCR		0xdc
#define BCR_WPD		BIT(0)

static int intel_spi_pci_bios_unlock(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	u32 bcr = 0;

	pci_read_config_dword(pdev, BCR, &bcr);
	if (!(bcr & BCR_WPD)) {
		bcr |= BCR_WPD;
		pci_write_config_dword(pdev, BCR, bcr);
		pci_read_config_dword(pdev, BCR, &bcr);
	}

	if (!(bcr & BCR_WPD))
		return -EIO;

	return 0;
}

static bool intel_spi_pci_is_bios_locked(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	u32 bcr = 0;

	pci_read_config_dword(pdev, BCR, &bcr);
	return !(bcr & BCR_WPD);
}

static const struct intel_spi_boardinfo bxt_info = {
	.type = INTEL_SPI_BXT,
	.is_bios_locked = intel_spi_pci_is_bios_locked,
	.bios_unlock = intel_spi_pci_bios_unlock,
};

static const struct intel_spi_boardinfo cnl_info = {
	.type = INTEL_SPI_CNL,
	.is_bios_locked = intel_spi_pci_is_bios_locked,
	.bios_unlock = intel_spi_pci_bios_unlock,
};

static int intel_spi_pci_probe(struct pci_dev *pdev,
			       const struct pci_device_id *id)
{
	struct intel_spi_boardinfo *info;
	struct intel_spi *ispi;
	int ret;

	ret = pcim_enable_device(pdev);
	if (ret)
		return ret;

	info = devm_kmemdup(&pdev->dev, (void *)id->driver_data, sizeof(*info),
			    GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	ispi = intel_spi_probe(&pdev->dev, &pdev->resource[0], info);
	if (IS_ERR(ispi))
		return PTR_ERR(ispi);

	pci_set_drvdata(pdev, ispi);
	return 0;
}

static void intel_spi_pci_remove(struct pci_dev *pdev)
{
	intel_spi_remove(pci_get_drvdata(pdev));
}

static const struct pci_device_id intel_spi_pci_ids[] = {
	{ PCI_VDEVICE(INTEL, 0x02a4), (unsigned long)&bxt_info },
	{ PCI_VDEVICE(INTEL, 0x06a4), (unsigned long)&bxt_info },
	{ PCI_VDEVICE(INTEL, 0x18e0), (unsigned long)&bxt_info },
	{ PCI_VDEVICE(INTEL, 0x19e0), (unsigned long)&bxt_info },
	{ PCI_VDEVICE(INTEL, 0x1bca), (unsigned long)&bxt_info },
	{ PCI_VDEVICE(INTEL, 0x34a4), (unsigned long)&bxt_info },
	{ PCI_VDEVICE(INTEL, 0x43a4), (unsigned long)&cnl_info },
	{ PCI_VDEVICE(INTEL, 0x4b24), (unsigned long)&bxt_info },
	{ PCI_VDEVICE(INTEL, 0x4da4), (unsigned long)&bxt_info },
	{ PCI_VDEVICE(INTEL, 0x51a4), (unsigned long)&cnl_info },
	{ PCI_VDEVICE(INTEL, 0x7aa4), (unsigned long)&cnl_info },
	{ PCI_VDEVICE(INTEL, 0xa0a4), (unsigned long)&bxt_info },
	{ PCI_VDEVICE(INTEL, 0xa1a4), (unsigned long)&bxt_info },
	{ PCI_VDEVICE(INTEL, 0xa224), (unsigned long)&bxt_info },
	{ PCI_VDEVICE(INTEL, 0xa324), (unsigned long)&cnl_info },
	{ PCI_VDEVICE(INTEL, 0xa3a4), (unsigned long)&bxt_info },
	{ },
};
MODULE_DEVICE_TABLE(pci, intel_spi_pci_ids);

static ssize_t intel_spi_is_protected_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%d\n", intel_spi_is_protected(dev));
}
static DEVICE_ATTR_ADMIN_RO(intel_spi_is_protected);

static ssize_t intel_spi_bios_lock_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%d\n", intel_spi_is_bios_lock(dev));
}

static ssize_t intel_spi_bios_lock_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t len)
{
	if (!sysfs_streq(buf, "unlock"))
		return -EINVAL;

	return intel_spi_bios_unlock(dev, len);
}
static DEVICE_ATTR_ADMIN_RW(intel_spi_bios_lock);

static struct attribute *intel_spi_pci_attrs[] = {
	 &dev_attr_intel_spi_is_protected.attr,
	 &dev_attr_intel_spi_bios_lock.attr,
	 NULL
};

static const struct attribute_group intel_spi_pci_attr_group = {
	.attrs = intel_spi_pci_attrs,
};

static const struct attribute_group *intel_spi_pci_dev_groups[] = {
	&intel_spi_pci_attr_group,
	NULL
};

static struct pci_driver intel_spi_pci_driver = {
	.name = "intel-spi",
	.id_table = intel_spi_pci_ids,
	.probe = intel_spi_pci_probe,
	.remove = intel_spi_pci_remove,
	.driver = {
		.dev_groups = intel_spi_pci_dev_groups,
	},
};

module_pci_driver(intel_spi_pci_driver);

MODULE_DESCRIPTION("Intel PCH/PCU SPI flash PCI driver");
MODULE_AUTHOR("Mika Westerberg <mika.westerberg@linux.intel.com>");
MODULE_LICENSE("GPL v2");
