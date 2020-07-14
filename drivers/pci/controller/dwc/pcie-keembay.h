/* SPDX-License-Identifier: GPL-2.0 */

#ifndef _PCIE_KEEMBAY_H
#define _PCIE_KEEMBAY_H

#include "pcie-designware.h"

#define KEEMBAY_PCIE_STEPPING_MAXLEN	8
#define to_keembay_pcie(x)		dev_get_drvdata((x)->dev)

struct keembay_pcie {
	struct dw_pcie		*pci;
	void __iomem		*apb_base;
	enum dw_pcie_device_mode mode;

	int			irq;
	int			ev_irq;
	int			err_irq;
	int			mem_access_irq;

	struct clk		*clk_master;
	struct clk		*clk_aux;
	struct gpio_desc	*reset;

	char			stepping[KEEMBAY_PCIE_STEPPING_MAXLEN];
};

#endif /* _PCIE_KEEMBAY_H */
