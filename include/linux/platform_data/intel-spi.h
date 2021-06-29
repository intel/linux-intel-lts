/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Intel PCH/PCU SPI flash driver.
 *
 * Copyright (C) 2016, Intel Corporation
 * Author: Mika Westerberg <mika.westerberg@linux.intel.com>
 */

#ifndef INTEL_SPI_PDATA_H
#define INTEL_SPI_PDATA_H

enum intel_spi_type {
	INTEL_SPI_BYT = 1,
	INTEL_SPI_LPT,
	INTEL_SPI_BXT,
	INTEL_SPI_CNL,
};

/**
 * struct intel_spi_boardinfo - Board specific data for Intel SPI driver
 * @type: Type which this controller is compatible with
 * @is_bios_locked: report if the device is locked by BIOS
 * @bios_unlock: handler to unlock the bios
 */
struct intel_spi_boardinfo {
	enum intel_spi_type type;
	bool (*is_bios_locked)(struct device *dev);
	int (*bios_unlock)(struct device *dev);
};

#endif /* INTEL_SPI_PDATA_H */
