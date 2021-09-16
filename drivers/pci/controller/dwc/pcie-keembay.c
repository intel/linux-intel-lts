// SPDX-License-Identifier: GPL-2.0-only
/*
 * PCIe controller driver for Intel Keem Bay and Thunder Bay platforms
 * Copyright (C) 2021 Intel Corporation
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/iopoll.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#include "pcie-designware.h"

/* PCIE_REGS_APB_SLV Registers */
#define PCIE_REGS_PCIE_CFG		0x0004
#define  PCIE_DEVICE_TYPE		BIT(8)
#define  PCIE_RSTN			BIT(0)
#define PCIE_REGS_PCIE_APP_CNTRL	0x0008
#define  APP_LTSSM_ENABLE		BIT(0)
#define PCIE_REGS_INTERRUPT_ENABLE	0x0028
#define  MSI_CTRL_INT_EN		BIT(8)
#define  EDMA_INT_EN			GENMASK(7, 0)
#define PCIE_REGS_INTERRUPT_STATUS	0x002c
#define  MSI_CTRL_INT			BIT(8)
#define PCIE_REGS_PCIE_SII_PM_STATE	0x00b0
#define  SMLH_LINK_UP			BIT(19)
#define  RDLH_LINK_UP			BIT(8)
#define  PCIE_REGS_PCIE_SII_LINK_UP	(SMLH_LINK_UP | RDLH_LINK_UP)
#define PCIE_REGS_PCIE_PHY_CNTL		0x0164
#define  PHY0_SRAM_BYPASS		BIT(8)
#define PCIE_REGS_PCIE_PHY_STAT		0x0168
#define  PHY0_MPLLA_STATE		BIT(1)
#define PCIE_REGS_LJPLL_STA		0x016c
#define  LJPLL_LOCK			BIT(0)
#define PCIE_REGS_LJPLL_CNTRL_0		0x0170
#define  LJPLL_EN			BIT(29)
#define  LJPLL_FOUT_EN			GENMASK(24, 21)
#define PCIE_REGS_LJPLL_CNTRL_2		0x0178
#define  LJPLL_REF_DIV			GENMASK(17, 12)
#define  LJPLL_FB_DIV			GENMASK(11, 0)
#define PCIE_REGS_LJPLL_CNTRL_3		0x017c
#define  LJPLL_POST_DIV3A		GENMASK(24, 22)
#define  LJPLL_POST_DIV2A		GENMASK(18, 16)

#define PERST_DELAY_US		1000
#define AUX_CLK_RATE_HZ		24000000

#define THB_PCIE_CTRL0_CNF0		0x0
#define THB_PCIE_CTRL0_CNF0_LTSSM_EN	BIT(4)

#define THB_PCIE_CTRL0_STS		0x1000
#define THB_PCIE_CTRL_STS_LINK_UP	(BIT(10) | BIT(0))

#define THB_PCIE_FN_OFFSET		BIT(16)

enum hw_plt_type {
	PLF_HW_KEEMBAY,
	PLF_HW_THUNDERBAY,
};

struct keembay_pcie {
	struct dw_pcie		pci;
	void __iomem		*apb_base;
	enum dw_pcie_device_mode mode;
	enum hw_plt_type	plt_type;

	struct clk		*clk_master;
	struct clk		*clk_aux;
	struct gpio_desc	*reset;
};

struct keembay_pcie_of_data {
	enum dw_pcie_device_mode mode;
	enum hw_plt_type	plt_type;
};

static int thunderbay_pcie_link_up(struct dw_pcie *pci)
{
	struct keembay_pcie *pcie = dev_get_drvdata(pci->dev);
	u32 mask = THB_PCIE_CTRL_STS_LINK_UP;
	u32 val = 0;

	if (pcie->mode == DW_PCIE_EP_TYPE)
		val = readl(pcie->apb_base + THB_PCIE_CTRL0_STS);

	if ((val & mask) == mask)
		return 1;

	return 0;
}

static void thunderbay_pcie_ltssm_enable(struct keembay_pcie *pcie, bool enable)
{
	u32 val;

	val = readl(pcie->apb_base + THB_PCIE_CTRL0_CNF0);
	if (enable)
		val |= THB_PCIE_CTRL0_CNF0_LTSSM_EN;
	else
		val &= ~THB_PCIE_CTRL0_CNF0_LTSSM_EN;
	writel(val, pcie->apb_base + THB_PCIE_CTRL0_CNF0);
}

static void thunderbay_pcie_stop_link(struct dw_pcie *pci)
{
	struct keembay_pcie *pcie = dev_get_drvdata(pci->dev);

	thunderbay_pcie_ltssm_enable(pcie, false);
}

static void thunderbay_pcie_write_dbi(struct dw_pcie *pci, void __iomem *base,
				      u32 reg, size_t size, u32 val)
{
	struct keembay_pcie *pcie = dev_get_drvdata(pci->dev);
	int ret;

	if (pcie->mode == DW_PCIE_EP_TYPE &&
	    (reg == PCI_BASE_ADDRESS_0 ||
	     reg == PCI_BASE_ADDRESS_2 ||
	     reg == PCI_BASE_ADDRESS_4))
		return;

	ret = dw_pcie_write(base + reg, size, val);
	if (ret)
		dev_err(pci->dev, "write DBI address failed\n");
}

static void thunderbay_pcie_write_dbi2(struct dw_pcie *pci, void __iomem *base,
				       u32 reg, size_t size, u32 val)
{
	struct keembay_pcie *pcie = dev_get_drvdata(pci->dev);
	int ret;

	if (pcie->mode == DW_PCIE_EP_TYPE)
		return;

	ret = dw_pcie_write(base + reg, size, val);
	if (ret)
		dev_err(pci->dev, "write DBI address failed\n");
}

static const struct dw_pcie_ops thunderbay_pcie_ops = {
	.link_up	= thunderbay_pcie_link_up,
	.stop_link	= thunderbay_pcie_stop_link,
	.write_dbi	= thunderbay_pcie_write_dbi,
	.write_dbi2	= thunderbay_pcie_write_dbi2,
};

static const struct pci_epc_features thunderbay_pcie_epc_features = {
	.linkup_notifier	= false,
	.msi_capable		= true,
	.msix_capable		= false,
	.reserved_bar		= BIT(BAR_1) | BIT(BAR_3) | BIT(BAR_5),
	.bar_fixed_64bit	= BIT(BAR_0) | BIT(BAR_2) | BIT(BAR_4),
	.bar_fixed_size[0]	= SZ_4K,
	.bar_fixed_size[2]	= SZ_16M,
	.bar_fixed_size[4]	= SZ_8M,
	.align			= SZ_1M,
};

static int keembay_pcie_ep_raise_irq(struct dw_pcie_ep *ep, u8 func_no,
				     enum pci_epc_irq_type type,
				     u16 interrupt_num)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);

	switch (type) {
	case PCI_EPC_IRQ_LEGACY:
		/* Legacy interrupts are not supported in Keem Bay */
		dev_err(pci->dev, "Legacy IRQ is not supported\n");
		return -EINVAL;
	case PCI_EPC_IRQ_MSI:
		return dw_pcie_ep_raise_msi_irq(ep, func_no, interrupt_num);
	case PCI_EPC_IRQ_MSIX:
		return dw_pcie_ep_raise_msix_irq(ep, func_no, interrupt_num);
	default:
		dev_err(pci->dev, "Unknown IRQ type %d\n", type);
		return -EINVAL;
	}
}

static const struct pci_epc_features *
thunderbay_pcie_get_features(struct dw_pcie_ep *ep)
{
	return &thunderbay_pcie_epc_features;
}

static unsigned int thunderbay_pcie_func_conf_select(struct dw_pcie_ep *ep,
						     u8 func_no)
{
	return func_no * THB_PCIE_FN_OFFSET;
}

static const struct dw_pcie_ep_ops thunderbay_pcie_ep_ops = {
	.raise_irq	= keembay_pcie_ep_raise_irq,
	.get_features	= thunderbay_pcie_get_features,
	.func_conf_select = thunderbay_pcie_func_conf_select,
};

static void keembay_ep_reset_assert(struct keembay_pcie *pcie)
{
	gpiod_set_value_cansleep(pcie->reset, 1);
	usleep_range(PERST_DELAY_US, PERST_DELAY_US + 500);
}

static void keembay_ep_reset_deassert(struct keembay_pcie *pcie)
{
	/*
	 * Ensure that PERST# is asserted for a minimum of 100ms.
	 *
	 * For more details, refer to PCI Express Card Electromechanical
	 * Specification Revision 1.1, Table-2.4.
	 */
	msleep(100);

	gpiod_set_value_cansleep(pcie->reset, 0);
	usleep_range(PERST_DELAY_US, PERST_DELAY_US + 500);
}

static void keembay_pcie_ltssm_set(struct keembay_pcie *pcie, bool enable)
{
	u32 val;

	val = readl(pcie->apb_base + PCIE_REGS_PCIE_APP_CNTRL);
	if (enable)
		val |= APP_LTSSM_ENABLE;
	else
		val &= ~APP_LTSSM_ENABLE;
	writel(val, pcie->apb_base + PCIE_REGS_PCIE_APP_CNTRL);
}

static int keembay_pcie_link_up(struct dw_pcie *pci)
{
	struct keembay_pcie *pcie = dev_get_drvdata(pci->dev);
	u32 val;

	val = readl(pcie->apb_base + PCIE_REGS_PCIE_SII_PM_STATE);

	return (val & PCIE_REGS_PCIE_SII_LINK_UP) == PCIE_REGS_PCIE_SII_LINK_UP;
}

static int keembay_pcie_start_link(struct dw_pcie *pci)
{
	struct keembay_pcie *pcie = dev_get_drvdata(pci->dev);
	u32 val;
	int ret;

	if (pcie->mode == DW_PCIE_EP_TYPE)
		return 0;

	keembay_pcie_ltssm_set(pcie, false);

	ret = readl_poll_timeout(pcie->apb_base + PCIE_REGS_PCIE_PHY_STAT,
				 val, val & PHY0_MPLLA_STATE, 20,
				 500 * USEC_PER_MSEC);
	if (ret) {
		dev_err(pci->dev, "MPLLA is not locked\n");
		return ret;
	}

	keembay_pcie_ltssm_set(pcie, true);

	return 0;
}

static void keembay_pcie_stop_link(struct dw_pcie *pci)
{
	struct keembay_pcie *pcie = dev_get_drvdata(pci->dev);

	keembay_pcie_ltssm_set(pcie, false);
}

static const struct dw_pcie_ops keembay_pcie_ops = {
	.link_up	= keembay_pcie_link_up,
	.start_link	= keembay_pcie_start_link,
	.stop_link	= keembay_pcie_stop_link,
};

static inline struct clk *keembay_pcie_probe_clock(struct device *dev,
						   const char *id, u64 rate)
{
	struct clk *clk;
	int ret;

	clk = devm_clk_get(dev, id);
	if (IS_ERR(clk))
		return clk;

	if (rate) {
		ret = clk_set_rate(clk, rate);
		if (ret)
			return ERR_PTR(ret);
	}

	ret = clk_prepare_enable(clk);
	if (ret)
		return ERR_PTR(ret);

	ret = devm_add_action_or_reset(dev,
				       (void(*)(void *))clk_disable_unprepare,
				       clk);
	if (ret)
		return ERR_PTR(ret);

	return clk;
}

static int keembay_pcie_probe_clocks(struct keembay_pcie *pcie)
{
	struct dw_pcie *pci = &pcie->pci;
	struct device *dev = pci->dev;

	pcie->clk_master = keembay_pcie_probe_clock(dev, "master", 0);
	if (IS_ERR(pcie->clk_master))
		return dev_err_probe(dev, PTR_ERR(pcie->clk_master),
				     "Failed to enable master clock");

	pcie->clk_aux = keembay_pcie_probe_clock(dev, "aux", AUX_CLK_RATE_HZ);
	if (IS_ERR(pcie->clk_aux))
		return dev_err_probe(dev, PTR_ERR(pcie->clk_aux),
				     "Failed to enable auxiliary clock");

	return 0;
}

/*
 * Initialize the internal PCIe PLL in Host mode.
 * See the following sections in Keem Bay data book,
 * (1) 6.4.6.1 PCIe Subsystem Example Initialization,
 * (2) 6.8 PCIe Low Jitter PLL for Ref Clk Generation.
 */
static int keembay_pcie_pll_init(struct keembay_pcie *pcie)
{
	struct dw_pcie *pci = &pcie->pci;
	u32 val;
	int ret;

	val = FIELD_PREP(LJPLL_REF_DIV, 0) | FIELD_PREP(LJPLL_FB_DIV, 0x32);
	writel(val, pcie->apb_base + PCIE_REGS_LJPLL_CNTRL_2);

	val = FIELD_PREP(LJPLL_POST_DIV3A, 0x2) |
		FIELD_PREP(LJPLL_POST_DIV2A, 0x2);
	writel(val, pcie->apb_base + PCIE_REGS_LJPLL_CNTRL_3);

	val = FIELD_PREP(LJPLL_EN, 0x1) | FIELD_PREP(LJPLL_FOUT_EN, 0xc);
	writel(val, pcie->apb_base + PCIE_REGS_LJPLL_CNTRL_0);

	ret = readl_poll_timeout(pcie->apb_base + PCIE_REGS_LJPLL_STA,
				 val, val & LJPLL_LOCK, 20,
				 500 * USEC_PER_MSEC);
	if (ret)
		dev_err(pci->dev, "Low jitter PLL is not locked\n");

	return ret;
}

static void keembay_pcie_msi_irq_handler(struct irq_desc *desc)
{
	struct keembay_pcie *pcie = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	u32 val, mask, status;
	struct pcie_port *pp;

	/*
	 * Keem Bay PCIe Controller provides an additional IP logic on top of
	 * standard DWC IP to clear MSI IRQ by writing '1' to the respective
	 * bit of the status register.
	 *
	 * So, a chained irq handler is defined to handle this additional
	 * IP logic.
	 */

	chained_irq_enter(chip, desc);

	pp = &pcie->pci.pp;
	val = readl(pcie->apb_base + PCIE_REGS_INTERRUPT_STATUS);
	mask = readl(pcie->apb_base + PCIE_REGS_INTERRUPT_ENABLE);

	status = val & mask;

	if (status & MSI_CTRL_INT) {
		dw_handle_msi_irq(pp);
		writel(status, pcie->apb_base + PCIE_REGS_INTERRUPT_STATUS);
	}

	chained_irq_exit(chip, desc);
}

static int keembay_pcie_setup_msi_irq(struct keembay_pcie *pcie)
{
	struct dw_pcie *pci = &pcie->pci;
	struct device *dev = pci->dev;
	struct platform_device *pdev = to_platform_device(dev);
	int irq;

	irq = platform_get_irq_byname(pdev, "pcie");
	if (irq < 0)
		return irq;

	irq_set_chained_handler_and_data(irq, keembay_pcie_msi_irq_handler,
					 pcie);

	return 0;
}

static void keembay_pcie_ep_init(struct dw_pcie_ep *ep)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	struct keembay_pcie *pcie = dev_get_drvdata(pci->dev);

	writel(EDMA_INT_EN, pcie->apb_base + PCIE_REGS_INTERRUPT_ENABLE);
}

static const struct pci_epc_features keembay_pcie_epc_features = {
	.linkup_notifier	= false,
	.msi_capable		= true,
	.msix_capable		= true,
	.reserved_bar		= BIT(BAR_1) | BIT(BAR_3) | BIT(BAR_5),
	.bar_fixed_64bit	= BIT(BAR_0) | BIT(BAR_2) | BIT(BAR_4),
	.align			= SZ_16K,
};

static const struct pci_epc_features *
keembay_pcie_get_features(struct dw_pcie_ep *ep)
{
	return &keembay_pcie_epc_features;
}

static const struct dw_pcie_ep_ops keembay_pcie_ep_ops = {
	.ep_init	= keembay_pcie_ep_init,
	.raise_irq	= keembay_pcie_ep_raise_irq,
	.get_features	= keembay_pcie_get_features,
};

static const struct dw_pcie_host_ops keembay_pcie_host_ops = {
};

static int keembay_pcie_add_pcie_port(struct keembay_pcie *pcie,
				      struct platform_device *pdev)
{
	struct dw_pcie *pci = &pcie->pci;
	struct pcie_port *pp = &pci->pp;
	struct device *dev = &pdev->dev;
	u32 val;
	int ret;

	pp->ops = &keembay_pcie_host_ops;
	pp->msi_irq = -ENODEV;

	ret = keembay_pcie_setup_msi_irq(pcie);
	if (ret)
		return ret;

	pcie->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(pcie->reset))
		return PTR_ERR(pcie->reset);

	ret = keembay_pcie_probe_clocks(pcie);
	if (ret)
		return ret;

	val = readl(pcie->apb_base + PCIE_REGS_PCIE_PHY_CNTL);
	val |= PHY0_SRAM_BYPASS;
	writel(val, pcie->apb_base + PCIE_REGS_PCIE_PHY_CNTL);

	writel(PCIE_DEVICE_TYPE, pcie->apb_base + PCIE_REGS_PCIE_CFG);

	ret = keembay_pcie_pll_init(pcie);
	if (ret)
		return ret;

	val = readl(pcie->apb_base + PCIE_REGS_PCIE_CFG);
	writel(val | PCIE_RSTN, pcie->apb_base + PCIE_REGS_PCIE_CFG);
	keembay_ep_reset_deassert(pcie);

	ret = dw_pcie_host_init(pp);
	if (ret) {
		keembay_ep_reset_assert(pcie);
		dev_err(dev, "Failed to initialize host: %d\n", ret);
		return ret;
	}

	val = readl(pcie->apb_base + PCIE_REGS_INTERRUPT_ENABLE);
	if (IS_ENABLED(CONFIG_PCI_MSI))
		val |= MSI_CTRL_INT_EN;
	writel(val, pcie->apb_base + PCIE_REGS_INTERRUPT_ENABLE);

	return 0;
}

static int keembay_pcie_probe(struct platform_device *pdev)
{
	const struct keembay_pcie_of_data *data;
	struct device *dev = &pdev->dev;
	enum dw_pcie_device_mode mode;
	struct keembay_pcie *pcie;
	struct dw_pcie *pci;

	data = device_get_match_data(dev);
	if (!data)
		return -ENODEV;

	mode = (enum dw_pcie_device_mode)data->mode;

	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	pci = &pcie->pci;
	pci->dev = dev;

	pcie->mode = mode;
	pcie->plt_type = (enum hw_plt_type)data->plt_type;

	if (pcie->plt_type == PLF_HW_KEEMBAY) {
		pci->ops = &keembay_pcie_ops;
	} else {
		pci->ops = &thunderbay_pcie_ops;

		/* Create 32 inbound and 64 outbound windows */
		pci->atu_size = SZ_32K;
	}

	pcie->apb_base = devm_platform_ioremap_resource_byname(pdev, "apb");
	if (IS_ERR(pcie->apb_base))
		return PTR_ERR(pcie->apb_base);

	platform_set_drvdata(pdev, pcie);

	switch (pcie->mode) {
	case DW_PCIE_RC_TYPE:
		if (!IS_ENABLED(CONFIG_PCIE_KEEMBAY_HOST))
			return -ENODEV;

		return keembay_pcie_add_pcie_port(pcie, pdev);
	case DW_PCIE_EP_TYPE:
		if (!IS_ENABLED(CONFIG_PCIE_KEEMBAY_EP))
			return -ENODEV;

		if (pcie->plt_type == PLF_HW_KEEMBAY)
			pci->ep.ops = &keembay_pcie_ep_ops;
		else
			pci->ep.ops = &thunderbay_pcie_ep_ops;

		return dw_pcie_ep_init(&pci->ep);
	default:
		dev_err(dev, "Invalid device type %d\n", pcie->mode);
		return -ENODEV;
	}
}

static const struct keembay_pcie_of_data keembay_pcie_rc_of_data = {
	.mode = DW_PCIE_RC_TYPE,
	.plt_type = PLF_HW_KEEMBAY,
};

static const struct keembay_pcie_of_data keembay_pcie_ep_of_data = {
	.mode = DW_PCIE_EP_TYPE,
	.plt_type = PLF_HW_KEEMBAY,
};

static const struct keembay_pcie_of_data thunderbay_pcie_ep_of_data = {
	.mode = DW_PCIE_EP_TYPE,
	.plt_type = PLF_HW_THUNDERBAY,
};

static const struct of_device_id keembay_pcie_of_match[] = {
	{
		.compatible = "intel,keembay-pcie",
		.data = &keembay_pcie_rc_of_data,
	},
	{
		.compatible = "intel,keembay-pcie-ep",
		.data = &keembay_pcie_ep_of_data,
	},
	{
		.compatible = "intel,thunderbay-pcie-ep",
		.data = &thunderbay_pcie_ep_of_data,
	},
	{}
};

static struct platform_driver keembay_pcie_driver = {
	.driver = {
		.name = "keembay-pcie",
		.of_match_table = keembay_pcie_of_match,
		.suppress_bind_attrs = true,
	},
	.probe  = keembay_pcie_probe,
};
builtin_platform_driver(keembay_pcie_driver);
