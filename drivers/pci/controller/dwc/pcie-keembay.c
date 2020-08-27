// SPDX-License-Identifier: GPL-2.0-only
/*
 * PCIe controller driver for Intel Keem Bay
 * Copyright (C) 2020 Intel Corporation
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/pci.h>
#include <linux/platform_device.h>

#include "pcie-keembay.h"

/* PCIE_REGS_APB_SLV Registers */
#define PCIE_REGS_PCIE_CFG		0x4
#define PCIE_REGS_PCIE_APP_CNTRL	0x8

#define PCIE_REGS_PCIE_INTR_ENABLE	0x18
#define  LBC_CII_EVENT_EN		BIT(18)
#define PCIE_REGS_PCIE_INTR_FLAGS	0x1c

#define PCIE_REGS_PCIE_ERR_INTR_ENABLE	0x20
#define  LINK_REQ_RST_EN		BIT(15)
#define PCIE_REGS_PCIE_ERR_INTR_FLAGS	0x24

#define PCIE_REGS_INTERRUPT_ENABLE	0x28
#define  MSI_CTRL_INT_EN		BIT(8)
#define  EDMA_INT_EN			GENMASK(7, 0)
#define PCIE_REGS_INTERRUPT_STATUS	0x2c
#define  MSI_CTRL_INT			BIT(8)

#define PCIE_REGS_PCIE_SII_PM_STATE	0xb0
#define PCIE_REGS_PCIE_PHY_CNTL		0x164
#define PCIE_REGS_PCIE_PHY_STAT		0x168
#define PCIE_REGS_LJPLL_STA		0x16c
#define PCIE_REGS_LJPLL_CNTRL_0		0x170
#define PCIE_REGS_LJPLL_CNTRL_1		0x174
#define PCIE_REGS_LJPLL_CNTRL_2		0x178
#define PCIE_REGS_LJPLL_CNTRL_3		0x17c

#define PCIE_REGS_MEM_ACCESS_IRQ_ENABLE	0x184

#define to_keembay_pcie(x)	dev_get_drvdata((x)->dev)

#define PCIE_DBI2_MASK		BIT(20)
#define PERST_DELAY_US		1000

/*
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
};
*/

struct keembay_pcie_of_data {
	enum dw_pcie_device_mode mode;
	const struct dw_pcie_host_ops *host_ops;
	const struct dw_pcie_ep_ops *ep_ops;
};

static const struct of_device_id keembay_pcie_of_match[];

static inline u32 keembay_pcie_readl(struct keembay_pcie *pcie, u32 offset)
{
	return readl(pcie->apb_base + offset);
}

static inline void keembay_pcie_writel(struct keembay_pcie *pcie, u32 offset,
				       u32 value)
{
	writel(value, pcie->apb_base + offset);
}

static void keembay_ep_reset_assert(struct keembay_pcie *pcie)
{
	/* Toggle the PERST# GPIO pin to LOW */
	gpiod_set_value_cansleep(pcie->reset, 1);
	usleep_range(PERST_DELAY_US, PERST_DELAY_US + 500);
}

static void keembay_ep_reset_deassert(struct keembay_pcie *pcie)
{
	/* Ensure that PERST# has been asserted for at least 100ms */
	msleep(100);

	/* Toggle the PERST# GPIO pin to HIGH */
	gpiod_set_value_cansleep(pcie->reset, 0);
	usleep_range(PERST_DELAY_US, PERST_DELAY_US + 500);
}

static void keembay_pcie_ltssm_enable(struct keembay_pcie *pcie, bool enable)
{
	struct dw_pcie *pci = pcie->pci;
	u32 val;

	/*
	 * PCIE_REGS_PCIE_APP_CNTRL register at offset 0x8
	 * bit  0: app_ltssm_enable
	 */
	val = keembay_pcie_readl(pcie, PCIE_REGS_PCIE_APP_CNTRL);
	if (enable)
		val |= BIT(0);
	else
		val &= ~BIT(0);
	keembay_pcie_writel(pcie, PCIE_REGS_PCIE_APP_CNTRL, val);
	dev_dbg(pci->dev, "PCIE_REGS_PCIE_APP_CNTRL: 0x%08x\n",
		keembay_pcie_readl(pcie, PCIE_REGS_PCIE_APP_CNTRL));
}

static void keembay_pcie_sram_bypass_mode(struct keembay_pcie *pcie)
{
	struct dw_pcie *pci = pcie->pci;
	u32 val;

	/*
	 * PCIE_REGS_PCIE_PHY_CNTL register
	 * bit  8: phy0_sram_bypass
	 *
	 * Set SRAM bypass mode.
	 */
	val = keembay_pcie_readl(pcie, PCIE_REGS_PCIE_PHY_CNTL) | BIT(8);
	keembay_pcie_writel(pcie, PCIE_REGS_PCIE_PHY_CNTL, val);
	dev_dbg(pci->dev, "PCIE_REGS_PCIE_PHY_CNTL: 0x%08x\n",
		keembay_pcie_readl(pcie, PCIE_REGS_PCIE_PHY_CNTL));
}

/* Initialize the internal PCIe PLL in Host mode (24MHz refclk) */
static void keembay_pcie_pll_init(struct keembay_pcie *pcie)
{
	struct dw_pcie *pci = pcie->pci;
	u32 val;

	val = (0x0 << 12)	// [17:12] ljlpp_ref_div
		| (0x32 << 0);	// [11: 0] ljpll_fb_div
	keembay_pcie_writel(pcie, PCIE_REGS_LJPLL_CNTRL_2, val);
	dev_dbg(pci->dev, "PCIE_REGS_LJPLL_CNTRL_2: 0x%08x\n",
		keembay_pcie_readl(pcie, PCIE_REGS_LJPLL_CNTRL_2));

	val = (0x2 << 22)	// [24:22] ljpll_post_div3a
		| (0x2 << 16);	// [18:16] ljpll_post_div2a
	keembay_pcie_writel(pcie, PCIE_REGS_LJPLL_CNTRL_3, val);
	dev_dbg(pci->dev, "PCIE_REGS_LJPLL_CNTRL_3: 0x%08x\n",
		keembay_pcie_readl(pcie, PCIE_REGS_LJPLL_CNTRL_3));

	val = (0x1 << 29)	// [  :29] ljpll_en
		| (0xc << 21);	// [24:21] ljpll_fout_en
	keembay_pcie_writel(pcie, PCIE_REGS_LJPLL_CNTRL_0, val);
	dev_dbg(pci->dev, "PCIE_REGS_LJPLL_CNTRL_0: 0x%08x\n",
		keembay_pcie_readl(pcie, PCIE_REGS_LJPLL_CNTRL_0));

	/*
	 * PCIE_REGS_LJPLL_STA register
	 * bit  0: ljpll_lock
	 *
	 * Poll bit 0 until set.
	 */
	do {
		udelay(1);
		val = keembay_pcie_readl(pcie, PCIE_REGS_LJPLL_STA);
	} while ((val & BIT(0)) != BIT(0));
	dev_dbg(pci->dev, "PCIE_REGS_LJPLL_STA: 0x%08x\n", val);
}

static void keembay_pcie_device_type(struct keembay_pcie *pcie)
{
	struct dw_pcie *pci = pcie->pci;

	/*
	 * PCIE_REGS_PCIE_CFG register at offset 0x4
	 * bit  8: pcie_device_type
	 * where 0: Endpoint, 1: Root Complex
	 *
	 * Must be set before DEVICE_RSTN is removed.
	 * Clear all other bits in this register.
	 */
	switch (pcie->mode) {
	case DW_PCIE_RC_TYPE:
		keembay_pcie_writel(pcie, PCIE_REGS_PCIE_CFG, BIT(8));
		break;
	case DW_PCIE_EP_TYPE:
		/*
		 * Keem Bay boot firmware will configure the EP,
		 * Linux kernel device driver won't.
		 */
		break;
	default:
		dev_err(pci->dev, "INVALID device type %d\n", pcie->mode);
	}

	dev_dbg(pci->dev, "PCIE_REGS_PCIE_CFG: 0x%08x\n",
		keembay_pcie_readl(pcie, PCIE_REGS_PCIE_CFG));
}

static void keembay_pcie_reset_deassert(struct keembay_pcie *pcie)
{
	struct dw_pcie *pci = pcie->pci;
	u32 val;

	/*
	 * PCIE_REGS_PCIE_CFG register
	 * bit  0: pcie_rstn
	 * currently include PMA reset.
	 *
	 * Subsystem power-on-reset by setting bit 0 to 1
	 */
	val = keembay_pcie_readl(pcie, PCIE_REGS_PCIE_CFG) | BIT(0);
	keembay_pcie_writel(pcie, PCIE_REGS_PCIE_CFG, val);
	dev_dbg(pci->dev, "PCIE_REGS_PCIE_CFG: 0x%08x\n",
		keembay_pcie_readl(pcie, PCIE_REGS_PCIE_CFG));
}

static int keembay_pcie_probe_clocks(struct keembay_pcie *pcie)
{
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	int ret;

	pcie->clk_master = devm_clk_get(dev, "master");
	if (IS_ERR(pcie->clk_master)) {
		dev_err(dev, "Failed to get master clock\n");
		return PTR_ERR(pcie->clk_master);
	}

	pcie->clk_aux = devm_clk_get(dev, "aux");
	if (IS_ERR(pcie->clk_aux)) {
		dev_err(dev, "Failed to get auxiliary clock\n");
		return PTR_ERR(pcie->clk_aux);
	}

	ret = clk_prepare_enable(pcie->clk_master);
	if (ret) {
		dev_err(dev, "Failed to enable master clock: %d\n", ret);
		return ret;
	}

	ret = clk_set_rate(pcie->clk_aux, 24000000);
	if (ret) {
		dev_err(dev, "Failed to set auxiliary clock rate: %d\n", ret);
		goto out;
	}

	ret = clk_prepare_enable(pcie->clk_aux);
	if (ret) {
		dev_err(dev, "Failed to enable auxiliary clock: %d\n", ret);
		goto out;
	}

	return 0;

out:
	clk_disable_unprepare(pcie->clk_master);

	return ret;
}

static void keembay_pcie_enable_interrupts(struct keembay_pcie *pcie)
{
	u32 val;

	/* Enable interrupt */
	val = keembay_pcie_readl(pcie, PCIE_REGS_INTERRUPT_ENABLE);

	if (IS_ENABLED(CONFIG_PCI_MSI))
		val |= MSI_CTRL_INT_EN;

	keembay_pcie_writel(pcie, PCIE_REGS_INTERRUPT_ENABLE, val);

	/* Enable event interrupt */
	val = keembay_pcie_readl(pcie, PCIE_REGS_PCIE_INTR_ENABLE);
	keembay_pcie_writel(pcie, PCIE_REGS_PCIE_INTR_ENABLE, val);

	/* Enable error interrupt */
	val = keembay_pcie_readl(pcie, PCIE_REGS_PCIE_ERR_INTR_ENABLE);
	keembay_pcie_writel(pcie, PCIE_REGS_PCIE_ERR_INTR_ENABLE, val);
}

static irqreturn_t keembay_pcie_irq_handler(int irq, void *arg)
{
	struct keembay_pcie *pcie = arg;
	struct dw_pcie *pci = pcie->pci;
	struct pcie_port *pp = &pci->pp;
	struct device *dev = pci->dev;
	u32 val, mask, status;

	val = keembay_pcie_readl(pcie, PCIE_REGS_INTERRUPT_STATUS);
	mask = keembay_pcie_readl(pcie, PCIE_REGS_INTERRUPT_ENABLE);

	status = val & mask;
	if (!status)
		return IRQ_NONE;

	dev_dbg(dev, "PCIE_REGS_INTERRUPT_STATUS: 0x%08x\n", val);

	if (status & MSI_CTRL_INT) {
		WARN_ON(!IS_ENABLED(CONFIG_PCI_MSI));
		dw_handle_msi_irq(pp);
	}

	keembay_pcie_writel(pcie, PCIE_REGS_INTERRUPT_STATUS, status);

	return IRQ_HANDLED;
}

static irqreturn_t keembay_pcie_ev_irq_handler(int irq, void *arg)
{
	struct keembay_pcie *pcie = arg;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	u32 val, mask, status;

	val = keembay_pcie_readl(pcie, PCIE_REGS_PCIE_INTR_FLAGS);
	mask = keembay_pcie_readl(pcie, PCIE_REGS_PCIE_INTR_ENABLE);

	status = val & mask;
	if (!status)
		return IRQ_NONE;

	dev_dbg(dev, "PCIE_REGS_PCIE_INTR_FLAGS: 0x%08x\n", val);

	keembay_pcie_writel(pcie, PCIE_REGS_PCIE_INTR_FLAGS, status);

	return IRQ_HANDLED;
}

static irqreturn_t keembay_pcie_err_irq_handler(int irq, void *arg)
{
	struct keembay_pcie *pcie = arg;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	u32 val, mask, status;

	val = keembay_pcie_readl(pcie, PCIE_REGS_PCIE_ERR_INTR_FLAGS);
	mask = keembay_pcie_readl(pcie, PCIE_REGS_PCIE_ERR_INTR_ENABLE);

	status = val & mask;
	if (!status)
		return IRQ_NONE;

	dev_dbg(dev, "PCIE_REGS_PCIE_ERR_INTR_FLAGS: 0x%08x\n", val);

	keembay_pcie_writel(pcie, PCIE_REGS_PCIE_ERR_INTR_FLAGS, status);

	return IRQ_HANDLED;
}

static int keembay_pcie_setup_irq(struct keembay_pcie *pcie)
{

	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	struct platform_device *pdev = to_platform_device(dev);
	int err;

	pcie->irq = platform_get_irq_byname(pdev, "intr");
	if (pcie->irq < 0) {
		dev_err(dev, "failed to get IRQ: %d\n", pcie->irq);
		return pcie->irq;
	}

	err = devm_request_irq(dev, pcie->irq, keembay_pcie_irq_handler,
			       IRQF_SHARED | IRQF_NO_THREAD, "pcie-intr", pcie);
	if (err) {
		dev_err(dev, "failed to request IRQ: %d\n", err);
		return err;
	}

	pcie->ev_irq = platform_get_irq_byname(pdev, "ev_intr");
	if (pcie->ev_irq < 0) {
		dev_err(dev, "failed to get event IRQ: %d\n", pcie->ev_irq);
		return pcie->ev_irq;
	}

	err = devm_request_irq(dev, pcie->ev_irq, keembay_pcie_ev_irq_handler,
			       0, "pcie-ev-intr", pcie);
	if (err) {
		dev_err(dev, "failed to request event IRQ: %d\n", err);
		return err;
	}

	pcie->err_irq = platform_get_irq_byname(pdev, "err_intr");
	if (pcie->err_irq < 0) {
		dev_err(dev, "failed to get error IRQ: %d\n", pcie->err_irq);
		return pcie->err_irq;
	}

	err = devm_request_irq(dev, pcie->err_irq, keembay_pcie_err_irq_handler,
			       0, "pcie-err-intr", pcie);
	if (err)
		dev_err(dev, "failed to request error IRQ: %d\n", err);

	return err;
}

static void keembay_pcie_rc_establish_link(struct dw_pcie *pci)
{
	struct keembay_pcie *pcie = to_keembay_pcie(pci);
	u32 val;

	if (dw_pcie_link_up(pci)) {
		dev_info(pci->dev, "link is already up\n");
		return;
	}

	keembay_pcie_ltssm_enable(pcie, false);

	/*
	 * PCIE_REGS_PCIE_PHY_STAT register
	 * bit  2: phy0_mpllb_state
	 * bit  1: phy0_mplla_state
	 *
	 * Poll bit 1 until set.
	 */
	do {
		udelay(1);
		val = keembay_pcie_readl(pcie, PCIE_REGS_PCIE_PHY_STAT);
	} while ((val & BIT(1)) != BIT(1));
	dev_dbg(pci->dev, "PCIE_REGS_PCIE_PHY_STAT:  0x%08x\n", val);

	keembay_pcie_ltssm_enable(pcie, true);
	dw_pcie_wait_for_link(pci);
}

static int __init keembay_pcie_host_init(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct keembay_pcie *pcie = to_keembay_pcie(pci);

	dw_pcie_setup_rc(pp);
	keembay_pcie_rc_establish_link(pci);

	/* Disable BARs */
	dw_pcie_dbi_ro_wr_en(pci);
	dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_0 | PCIE_DBI2_MASK, 0);
	dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_1 | PCIE_DBI2_MASK, 0);
	dw_pcie_dbi_ro_wr_dis(pci);

	if (IS_ENABLED(CONFIG_PCI_MSI))
		dw_pcie_msi_init(pp);

	keembay_pcie_enable_interrupts(pcie);

	return 0;
}

static const struct dw_pcie_host_ops keembay_pcie_host_ops = {
	.host_init	= keembay_pcie_host_init,
};

static int keembay_pcie_link_up(struct dw_pcie *pci)
{
	struct keembay_pcie *pcie = to_keembay_pcie(pci);
	u32 mask = BIT(19) | BIT(8);
	u32 val;

	/*
	 * Wait for link up,
	 * PCIE_REGS_PCIE_SII_PM_STATE register
	 * bit 19: smlh_link_up
	 * bit  8: rdlh_link_up
	 */
	val = keembay_pcie_readl(pcie, PCIE_REGS_PCIE_SII_PM_STATE);

	if ((val & mask) == mask)
		return 1;

	dev_dbg(pci->dev,
		"No link detected (PCIE_REGS_PCIE_SII_PM_STATE: 0x%08x).\n",
		val);

	return 0;
}

static int keembay_pcie_establish_link(struct dw_pcie *pci)
{
	return 0;
}

static void keembay_pcie_stop_link(struct dw_pcie *pci)
{
	struct keembay_pcie *pcie = to_keembay_pcie(pci);

	keembay_pcie_ltssm_enable(pcie, false);

	dev_dbg(pci->dev, "Stop link (PCIE_REGS_PCIE_APP_CNTRL: 0x%08x).\n",
		keembay_pcie_readl(pcie, PCIE_REGS_PCIE_APP_CNTRL));
}

static const struct dw_pcie_ops keembay_pcie_ops = {
	.link_up	= keembay_pcie_link_up,
	.start_link	= keembay_pcie_establish_link,
	.stop_link	= keembay_pcie_stop_link,
};

static void keembay_pcie_ep_init(struct dw_pcie_ep *ep)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	struct keembay_pcie *pcie = to_keembay_pcie(pci);
	u32 val;

	/* Enable DMA completion/error interrupts */
	val = keembay_pcie_readl(pcie, PCIE_REGS_INTERRUPT_ENABLE);
	keembay_pcie_writel(pcie, PCIE_REGS_INTERRUPT_ENABLE,
			    val | EDMA_INT_EN);

	/* Enable CII event interrupt */
	val = keembay_pcie_readl(pcie, PCIE_REGS_PCIE_INTR_ENABLE);
	keembay_pcie_writel(pcie, PCIE_REGS_PCIE_INTR_ENABLE,
			    val | LBC_CII_EVENT_EN);

	/* Enable link request reset */
	val = keembay_pcie_readl(pcie, PCIE_REGS_PCIE_ERR_INTR_ENABLE);
	keembay_pcie_writel(pcie, PCIE_REGS_PCIE_ERR_INTR_ENABLE,
			    val | LINK_REQ_RST_EN);

	/* Enable host memory access interrupt */
	val = GENMASK(31, 0);
	keembay_pcie_writel(pcie, PCIE_REGS_MEM_ACCESS_IRQ_ENABLE, val);
}

static int keembay_pcie_ep_raise_irq(struct dw_pcie_ep *ep, u8 func_no,
				     enum pci_epc_irq_type type,
				     u16 interrupt_num)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);

	switch (type) {
	case PCI_EPC_IRQ_LEGACY:
		/* Legacy interrupts are not supported in Keem Bay */
		dev_err(pci->dev, "UNSUPPORTED IRQ TYPE\n");
		return -EINVAL;
	case PCI_EPC_IRQ_MSI:
		return dw_pcie_ep_raise_msi_irq(ep, func_no, interrupt_num);
	case PCI_EPC_IRQ_MSIX:
		return dw_pcie_ep_raise_msix_irq(ep, func_no, interrupt_num);
	default:
		dev_err(pci->dev, "UNKNOWN IRQ TYPE\n");
		return -EINVAL;
	}
}

static const struct pci_epc_features keembay_pcie_epc_features = {
	.linkup_notifier	= false,
	.msi_capable		= true,
	.msix_capable		= true,
	.reserved_bar		= 1 << BAR_1 | 1 << BAR_3 | 1 << BAR_5,
	.bar_fixed_64bit	= 1 << BAR_0 | 1 << BAR_2 | 1 << BAR_4,
	.align			= SZ_16K,
};

static const struct pci_epc_features*
keembay_pcie_get_features(struct dw_pcie_ep *ep)
{
	return &keembay_pcie_epc_features;
}

static const struct dw_pcie_ep_ops keembay_pcie_ep_ops = {
	.ep_init	= keembay_pcie_ep_init,
	.raise_irq	= keembay_pcie_ep_raise_irq,
	.get_features	= keembay_pcie_get_features,
};

static int keembay_pcie_add_pcie_port(struct keembay_pcie *pcie,
				      struct platform_device *pdev)
{
	struct dw_pcie *pci = pcie->pci;
	struct pcie_port *pp = &pci->pp;
	struct device *dev = &pdev->dev;
	int ret;

	pcie->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(pcie->reset))
		return PTR_ERR(pcie->reset);

	ret = keembay_pcie_probe_clocks(pcie);
	if (ret)
		return ret;

	ret = keembay_pcie_setup_irq(pcie);
	if (ret)
		return ret;

	keembay_pcie_sram_bypass_mode(pcie);
	keembay_pcie_device_type(pcie);
	keembay_pcie_pll_init(pcie);
	keembay_pcie_reset_deassert(pcie);
	keembay_ep_reset_deassert(pcie);

	pp->ops = &keembay_pcie_host_ops;

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "Failed to initialize host: %d\n", ret);
		keembay_ep_reset_assert(pcie);
		return ret;
	}

	return 0;
}

static struct pci_epc_ops keembay_epc_ops;

static int keembay_ep_inbound_atu(struct dw_pcie_ep *ep, enum pci_barno bar,
				  dma_addr_t cpu_addr,
				  enum dw_pcie_as_type as_type)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	u32 free_win;
	int ret;

	free_win = find_first_zero_bit(ep->ib_window_map, ep->num_ib_windows);
	if (free_win >= ep->num_ib_windows) {
		dev_err(pci->dev, "No free inbound window\n");
		return -EINVAL;
	}

	ret = dw_pcie_prog_inbound_atu(pci, free_win, bar, cpu_addr, as_type);
	if (ret < 0) {
		dev_err(pci->dev, "Failed to program IB window\n");
		return ret;
	}

	ep->bar_to_atu[bar] = free_win;
	set_bit(free_win, ep->ib_window_map);

	return 0;
}

static int keembay_ep_set_bar(struct pci_epc *epc, u8 func_no,
			      struct pci_epf_bar *epf_bar)
{
	struct dw_pcie_ep *ep = epc_get_drvdata(epc);
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	enum dw_pcie_as_type as_type;
	enum pci_barno bar = epf_bar->barno;
	u32 reg = PCI_BASE_ADDRESS_0 + (4 * bar);
	size_t size = epf_bar->size;
	int flags = epf_bar->flags;
	u64 host_addr;
	int ret;

	if (!(flags & PCI_BASE_ADDRESS_SPACE))
		as_type = DW_PCIE_AS_MEM;
	else
		as_type = DW_PCIE_AS_IO;

	host_addr = dw_pcie_readl_dbi(pci, reg) & ~0xF;
	if (flags & PCI_BASE_ADDRESS_MEM_TYPE_64)
		host_addr |= (u64)dw_pcie_readl_dbi(pci, reg + 4) << 32;

	ret = keembay_ep_inbound_atu(ep, bar, epf_bar->phys_addr, as_type);
	if (ret)
		return ret;

	dw_pcie_dbi_ro_wr_en(pci);

	dw_pcie_writel_dbi2(pci, reg, lower_32_bits(size - 1));
	dw_pcie_writel_dbi(pci, reg, lower_32_bits(host_addr) | flags);

	if (flags & PCI_BASE_ADDRESS_MEM_TYPE_64) {
		dw_pcie_writel_dbi2(pci, reg + 4, upper_32_bits(size - 1));
		dw_pcie_writel_dbi(pci, reg + 4, upper_32_bits(host_addr));
	}

	dw_pcie_dbi_ro_wr_dis(pci);

	return 0;
}

static int keembay_pcie_add_pcie_ep(struct keembay_pcie *pcie,
				    struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dw_pcie *pci = pcie->pci;
	struct dw_pcie_ep *ep;
	struct resource *res;
	int ret;

	ep = &pci->ep;
	ep->ops = &keembay_pcie_ep_ops;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "addr_space");
	if (!res)
		return -EINVAL;

	ep->phys_base = res->start;
	ep->addr_size = resource_size(res);

	pcie->irq = platform_get_irq_byname(pdev, "intr");
	if (pcie->irq < 0) {
		dev_err(dev, "failed to get IRQ: %d\n", pcie->irq);
		return pcie->irq;
	}

	pcie->ev_irq = platform_get_irq_byname(pdev, "ev_intr");
	if (pcie->ev_irq < 0) {
		dev_err(dev, "failed to get event IRQ: %d\n", pcie->ev_irq);
		return pcie->ev_irq;
	}

	pcie->err_irq = platform_get_irq_byname(pdev, "err_intr");
	if (pcie->err_irq < 0) {
		dev_err(dev, "failed to get error IRQ: %d\n", pcie->err_irq);
		return pcie->err_irq;
	}

	pcie->mem_access_irq = platform_get_irq_byname(pdev, "mem_access_intr");
	if (pcie->mem_access_irq < 0) {
		dev_err(dev, "failed to get mem access IRQ: %d\n",
			pcie->mem_access_irq);
		return pcie->mem_access_irq;
	}

	ret = dw_pcie_ep_init(ep);
	if (ret)
		dev_err(dev, "Failed to initialize endpoint\n");

	return ret;

	/*
	 * Use Keembay version set_bar for setting BAR with the addresses
	 * already set in BAR registers when Linux boots
	 */
	keembay_epc_ops = *ep->epc->ops;
	keembay_epc_ops.set_bar = keembay_ep_set_bar;
	ep->epc->ops = &keembay_epc_ops;

	return 0;
}

static int keembay_pcie_probe(struct platform_device *pdev)
{
	const struct keembay_pcie_of_data *data;
	const struct of_device_id *match;
	struct device *dev = &pdev->dev;
	struct keembay_pcie *pcie;
	struct dw_pcie *pci;
	struct resource *res;
	struct device_node *soc_node, *version_node;
	enum dw_pcie_device_mode mode;
	const char *prop;
	int prop_size;
	int ret;

	match = of_match_device(keembay_pcie_of_match, dev);
	if (!match)
		return -EINVAL;

	data = (struct keembay_pcie_of_data *)match->data;
	mode = (enum dw_pcie_device_mode)data->mode;

	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	pci->dev = dev;
	pci->ops = &keembay_pcie_ops;

	pcie->pci = pci;
	pcie->mode = mode;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "apb");
	pcie->apb_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pcie->apb_base))
		return PTR_ERR(pcie->apb_base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	pci->dbi_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pci->dbi_base))
		return PTR_ERR(pci->dbi_base);

	/* DBI2 shadow register */
	pci->dbi_base2 = pci->dbi_base + PCIE_DBI2_MASK;

	/* Keem Bay stepping info, based on DT */
	strncpy(pcie->stepping, "B0", strlen("B0"));
	soc_node = of_get_parent(pdev->dev.of_node);
	if (soc_node) {
		version_node = of_get_child_by_name(soc_node, "version-info");
		if (version_node) {
			prop = of_get_property(version_node, "stepping",
					       &prop_size);
			if (prop && prop_size <= KEEMBAY_PCIE_STEPPING_MAXLEN)
				strncpy(pcie->stepping, prop, prop_size);
			of_node_put(version_node);
		}
		of_node_put(soc_node);
	}

	platform_set_drvdata(pdev, pcie);

	switch (pcie->mode) {
	case DW_PCIE_RC_TYPE:
		if (!IS_ENABLED(CONFIG_PCIE_KEEMBAY_HOST))
			return -ENODEV;

		ret = keembay_pcie_add_pcie_port(pcie, pdev);
		if (ret < 0)
			return ret;
		break;
	case DW_PCIE_EP_TYPE:
		if (!IS_ENABLED(CONFIG_PCIE_KEEMBAY_EP))
			return -ENODEV;

		ret = keembay_pcie_add_pcie_ep(pcie, pdev);
		if (ret < 0)
			return ret;
		break;
	default:
		dev_err(dev, "INVALID device type %d\n", pcie->mode);
	}

	return 0;
}

static const struct keembay_pcie_of_data keembay_pcie_rc_of_data = {
	.mode = DW_PCIE_RC_TYPE,
};

static const struct keembay_pcie_of_data keembay_pcie_ep_of_data = {
	.mode = DW_PCIE_EP_TYPE,
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
	{},
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
