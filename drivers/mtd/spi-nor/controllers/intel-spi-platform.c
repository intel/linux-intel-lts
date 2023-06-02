// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel PCH/PCU SPI flash platform driver.
 *
 * Copyright (C) 2016, Intel Corporation
 * Author: Mika Westerberg <mika.westerberg@linux.intel.com>
 */

#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "intel-spi.h"

static int intel_spi_platform_probe(struct platform_device *pdev)
{
	struct intel_spi_boardinfo *info;
	struct intel_spi *ispi;
	struct resource *mem;

	info = dev_get_platdata(&pdev->dev);
	if (!info)
		return -EINVAL;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ispi = intel_spi_probe(&pdev->dev, mem, info);
	if (IS_ERR(ispi))
		return PTR_ERR(ispi);

	platform_set_drvdata(pdev, ispi);
	return 0;
}

static int intel_spi_platform_remove(struct platform_device *pdev)
{
	struct intel_spi *ispi = platform_get_drvdata(pdev);

	return intel_spi_remove(ispi);
}

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

static struct attribute *intel_spi_platform_attrs[] = {
	&dev_attr_intel_spi_is_protected.attr,
	&dev_attr_intel_spi_bios_lock.attr,
	NULL
};

static const struct attribute_group intel_spi_platform_attr_group = {
	.attrs = intel_spi_platform_attrs,
};

static const struct attribute_group *intel_spi_platform_dev_groups[] = {
	&intel_spi_platform_attr_group,
	NULL
};

static struct platform_driver intel_spi_platform_driver = {
	.probe = intel_spi_platform_probe,
	.remove = intel_spi_platform_remove,
	.driver = {
		.name = "intel-spi",
		.dev_groups = intel_spi_platform_dev_groups,
	},
};

module_platform_driver(intel_spi_platform_driver);

MODULE_DESCRIPTION("Intel PCH/PCU SPI flash platform driver");
MODULE_AUTHOR("Mika Westerberg <mika.westerberg@linux.intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:intel-spi");
