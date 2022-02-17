// SPDX-License-Identifier: GPL-2.0-only
/*
 * PCIe controller driver for Intel Keem Bay and Thunder Bay platforms
 * Copyright (C) 2021 Intel Corporation
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
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
#define THB_PCIE_CTRL1_STS		0x1040
#define THB_PCIE_CTRL_STS_LINK_UP	(BIT(10) | BIT(0))

#define THB_PCIE_CTRL1_CNF0		0x28
#define THB_PCIE_CTRL1_CNF0_LTSSM_EN	BIT(4)

#define THB_PCIE_CTRL1_CNF1		0x2c
#define THB_PCIE_CTRL1_CNF1_XFER_PEND	BIT(3)

#define THB_PCIE_CTRL1_SPARE1		0x34
#define THB_PCIE_CTRL1_SPARE1_PERST_PAD	(BIT(2) | BIT(1))

#define THB_PCIE_PHY1_CNF0		0x54
#define THB_PCIE_PHY1_CNF0_EXT_LD	BIT(29)

#define THB_PCIE_PHY1_CNF2		0x5C
#define THB_PCIE_PHY1_CNF2_PWR_STABLE	(BIT(1) | BIT(0))

#define THB_PCIE_PHY_CMN_CNF1		0x64
#define THB_PCIE_PHY_CMN_CNF1_UPCS_CFG	(BIT(16) | BIT(1) | BIT(0))

#define THB_PCIE_PHY1_STS1		0x107c
#define THB_PCIE_PHY1_STS1_SRAM_INIT	BIT(5)

#define THB_PCIE_CPR_RST_EN		0x0
#define THB_PCIE_CPR_RST_EN_CTRL1	BIT(4)

#define THB_PCIE_EP_BUS_POS		BIT(24)
#define THB_PCIE_LINK_UP_TO		10000

#define THB_PCIE_FN_OFFSET		BIT(16)

enum hw_plt_type {
	PLF_HW_KEEMBAY,
	PLF_HW_THUNDERBAY,
};

struct keembay_pcie {
	struct dw_pcie		pci;
	void __iomem		*apb_base;
	void __iomem		*cpr_base;
	enum dw_pcie_device_mode mode;
	enum hw_plt_type	plt_type;

	struct clk		*clk_master;
	struct clk		*clk_aux;
	struct gpio_desc	*reset;
	struct dma_pool		*rc_dma_pool;

	dma_addr_t		rc_dma_mem_pa;
	void			*rc_dma_mem_va;
	struct dma_chan		*rd_dma_chan;
	bool			rc_dma;
};

struct keembay_pcie_of_data {
	enum dw_pcie_device_mode mode;
	enum hw_plt_type	plt_type;
};

static void rc_dma_complete_cb(void *completion)
{
	complete(completion);
}

static int thunderbay_pcie_rc_dma_rd(struct pcie_port *pp,
				     struct dma_chan *chan,
				     dma_addr_t dst,
				     dma_addr_t src,
				     int size)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct dma_device *dma_dev = chan->device;
	struct dma_async_tx_descriptor *tx = NULL;
	enum dma_status status = DMA_COMPLETE;
	struct device *dev = pci->dev;
	unsigned long start_jiffies;
	enum dma_ctrl_flags flags;
	struct completion cmp;
	dma_cookie_t cookie;

	flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;
	tx = dma_dev->device_prep_dma_memcpy(chan,
					     dst,
					     src,
					     size,
					     flags);
	if (!tx) {
		dev_err(dev, "failed to get dma async tx desc\n");
		return -1;
	}

	init_completion(&cmp);
	tx->callback = rc_dma_complete_cb;
	tx->callback_param = &cmp;

	cookie = tx->tx_submit(tx);
	if (dma_submit_error(cookie)) {
		dev_err(dev, "failed to submit dma desc\n");
		return -1;
	}

	dma_async_issue_pending(chan);
	start_jiffies = jiffies;
	while (!completion_done(&cmp) &&
	       !time_after(jiffies, start_jiffies + HZ))
		;

	status = dma_async_is_tx_complete(chan, cookie, NULL, NULL);
	if (status != DMA_COMPLETE) {
		dev_err(dev, "failed to complete dma transaction\n");
		return -1;
	}

	return 0;
}

static void
__iomem *thunderbay_pcie_conf_addr_map_bus(struct pci_bus *bus,
					   unsigned int devfn, int where)
{
	struct pcie_port *pp = bus->sysdata;
	struct dw_pcie *pci;
	u32 busdev;
	int type;

	pci = to_dw_pcie_from_pp(pp);

	/*
	 * Checking whether the link is up here is a last line of defense
	 * against platforms that forward errors on the system bus as
	 * SError upon PCI configuration transactions issued when the link
	 * is down. This check is racy by definition and does not stop
	 * the system from triggering an SError if the link goes down
	 * after this check is performed.
	 */
	if (!dw_pcie_link_up(pci))
		return NULL;

	busdev = PCIE_ATU_BUS(bus->number) | PCIE_ATU_DEV(PCI_SLOT(devfn)) |
		 PCIE_ATU_FUNC(PCI_FUNC(devfn));

	if (pci_is_root_bus(bus->parent))
		type = PCIE_ATU_TYPE_CFG0;
	else
		type = PCIE_ATU_TYPE_CFG1;

	/* EP stays on the same bus as RC */
	busdev -= THB_PCIE_EP_BUS_POS;

	dw_pcie_prog_outbound_atu(pci, 0x1,
				  type, pp->cfg0_base,
				  busdev, pp->cfg0_size);

	return pp->va_cfg0_base + where;
}

static int
thunderbay_pcie_host_rd_other_conf(struct pci_bus *bus, unsigned int devfn,
				   int where, int size, u32 *val)
{
	struct pcie_port *pp = bus->sysdata;
	struct keembay_pcie *pcie;
	int ret, where_align;
	struct dw_pcie *pci;

	pci = to_dw_pcie_from_pp(pp);
	pcie = dev_get_drvdata(pci->dev);

	if (pcie->rc_dma) {
		where_align = where & ~((typeof(where))(4) - 1);
		if (thunderbay_pcie_rc_dma_rd(pp,
					      pcie->rd_dma_chan,
					      pcie->rc_dma_mem_pa + where_align,
					      pp->cfg0_base + where_align,
					      4))
			ret = PCIBIOS_BAD_REGISTER_NUMBER;

		*val = *((int *)(pcie->rc_dma_mem_va + where));

		if (size == 4)
			*val &= 0xFFFFFFFF;
		else if (size == 2)
			*val &= 0xFFFF;
		else if (size == 1)
			*val &= 0xFF;
		else
			ret = PCIBIOS_BAD_REGISTER_NUMBER;
	} else {
		ret = dw_pcie_read(pp->va_cfg0_base + where, size, val);
	}

	return ret;
}

static struct pci_ops thunderbay_child_pci_ops = {
	.map_bus = thunderbay_pcie_conf_addr_map_bus,
	.read = thunderbay_pcie_host_rd_other_conf,
	.write = pci_generic_config_write,
};

static int thunderbay_pcie_rc_dma_cfg(struct keembay_pcie *pcie,
				      struct platform_device *pdev)
{
	struct dw_pcie *pci = &pcie->pci;
	struct device *dev = &pdev->dev;
	struct dma_chan *rd_chan;
	dma_addr_t phys;
	void *segment;
	int ret;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret)
		dev_warn(dev, "failed to set dma mask\n");

	rd_chan = dma_request_chan(&pdev->dev, "rx");
	if (IS_ERR(rd_chan)) {
		ret = PTR_ERR(rd_chan);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "No RC AXI DMA Rd channel available\n");
		return ret;
	}
	pcie->rd_dma_chan = rd_chan;

	pcie->rc_dma_pool = dma_pool_create("thb_pcie_rc_dma_pool",
					    pci->dev,
					    PAGE_SIZE,
					    PAGE_SIZE,
					    0);
	if (!pcie->rc_dma_pool) {
		dev_err(dev, "unable to allocate RC AXI DMA pool\n");
		ret = PTR_ERR(pcie->rc_dma_pool);
		goto free_dma_channel;
	}

	segment = dma_pool_zalloc(pcie->rc_dma_pool, GFP_ATOMIC, &phys);
	if (!segment) {
		dev_err(dev, "unable to get mem from RC AXI DMA pool\n");
		ret = PTR_ERR(segment);
		goto free_dma_pool;
	}
	pcie->rc_dma_mem_pa = phys;
	pcie->rc_dma_mem_va = segment;

	return 0;

free_dma_channel:
	dma_release_channel(pcie->rd_dma_chan);
free_dma_pool:
	dma_pool_destroy(pcie->rc_dma_pool);

	return ret;
}

static int thunderbay_pcie_link_up(struct dw_pcie *pci)
{
	struct keembay_pcie *pcie = dev_get_drvdata(pci->dev);
	u32 mask = THB_PCIE_CTRL_STS_LINK_UP;
	u32 val = 0;

	if (pcie->mode == DW_PCIE_EP_TYPE)
		val = readl(pcie->apb_base + THB_PCIE_CTRL0_STS);
	if (pcie->mode == DW_PCIE_RC_TYPE)
		val = readl(pcie->apb_base + THB_PCIE_CTRL1_STS);

	if ((val & mask) == mask)
		return 1;

	return 0;
}

static int thunderbay_pcie_rc_phy_init(struct dw_pcie *pci)
{
	struct keembay_pcie *pcie = dev_get_drvdata(pci->dev);
	u32 timeout = THB_PCIE_LINK_UP_TO, val;

	/* PCIe_RST_EN, POR, SubSysRst */
	val = readl(pcie->cpr_base + THB_PCIE_CPR_RST_EN);
	val |= THB_PCIE_CPR_RST_EN_CTRL1;
	writel(val, pcie->cpr_base + THB_PCIE_CPR_RST_EN);

	/* PHY1_CNF0, ref_use_pad */
	writel(0x0, pcie->apb_base + THB_PCIE_PHY1_CNF0);

	/* phy_cmn_conf_1 upcs_pwrstable */
	val = THB_PCIE_PHY_CMN_CNF1_UPCS_CFG;
	writel(val, pcie->apb_base + THB_PCIE_PHY_CMN_CNF1);

	/* Phy1 Cnf2, pcs/pma pwr_stable */
	val = THB_PCIE_PHY1_CNF2_PWR_STABLE;
	writel(val, pcie->apb_base + THB_PCIE_PHY1_CNF2);

	/* SRAM init done? */
	val = readl(pcie->apb_base + THB_PCIE_PHY1_STS1);
	val &= THB_PCIE_PHY1_STS1_SRAM_INIT;
	while (!val && timeout-- > 0) {
		val = readl(pcie->apb_base + THB_PCIE_PHY1_STS1);
		val &= THB_PCIE_PHY1_STS1_SRAM_INIT;
	}
	if (!timeout)
		return -EBUSY;

	/* pciess_ctrl1_cmn_cnf_1 xfer_pending disable */
	val = THB_PCIE_CTRL1_CNF1_XFER_PEND;
	writel(val, pcie->apb_base + THB_PCIE_CTRL1_CNF1);

	/* PHY1_CNF0, set ext_ld_done */
	val = THB_PCIE_PHY1_CNF0_EXT_LD;
	writel(val, pcie->apb_base + THB_PCIE_PHY1_CNF0);

	/* PERST Pad configuration */
	val = THB_PCIE_CTRL1_SPARE1_PERST_PAD;
	writel(val, pcie->apb_base + THB_PCIE_CTRL1_SPARE1);

	/* LTSSM enable */
	val = readl(pcie->apb_base + THB_PCIE_CTRL1_CNF0);
	val |= THB_PCIE_CTRL1_CNF0_LTSSM_EN;
	writel(val, pcie->apb_base + THB_PCIE_CTRL1_CNF0);

	/* Wait for link up */
	timeout = THB_PCIE_LINK_UP_TO;
	while (!thunderbay_pcie_link_up(pci) && timeout-- > 0)
		;
	if (!timeout)
		return -EBUSY;

	return 0;
}

static int __init thunderbay_pcie_host_init(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	pp->bridge->child_ops = &thunderbay_child_pci_ops;

	dw_pcie_setup_rc(pp);

	dw_pcie_prog_outbound_atu(pci, 0x1,
				  PCIE_ATU_TYPE_MEM, pp->io_base,
				  pp->io_bus_addr, pp->io_size);

	return 0;
}

static const struct dw_pcie_host_ops thunderbay_pcie_host_ops = {
	.host_init	= thunderbay_pcie_host_init,
};

static int thunderbay_pcie_add_pcie_port(struct keembay_pcie *pcie,
					 struct platform_device *pdev)
{
	struct dw_pcie *pci = &pcie->pci;
	struct pcie_port *pp = &pci->pp;
	struct device *dev = &pdev->dev;
	int ret;

	pp->ops = &thunderbay_pcie_host_ops;

	pcie->cpr_base = devm_platform_ioremap_resource_byname(pdev, "cpr");
	if (IS_ERR(pcie->cpr_base))
		return PTR_ERR(pcie->cpr_base);

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		pp->msi_irq = platform_get_irq(pdev, 0);
		if (pp->msi_irq < 0) {
			dev_err(dev, "failed to get msi IRQ: %d\n",
				pp->msi_irq);
			return pp->msi_irq;
		}
	}

	/* Initialize Root Port PHY */
	ret = thunderbay_pcie_rc_phy_init(pci);
	if (ret) {
		dev_err(dev, "Failed to initialize RC PHY: %d\n", ret);
		return ret;
	}

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "Failed to initialize host: %d\n", ret);
		return ret;
	}

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
	struct platform_device *pdev;
	int irq;

	pdev = to_platform_device(dev);
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
	struct keembay_pcie *pcie;

	pcie = dev_get_drvdata(pci->dev);
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
	int ret;

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

		/*
		 * Due to a silicon bug in THB PCIe RC mode, all CfgRd access
		 * should go through DMA. This is a SW Workaround.
		 * So, configure the DMA for THB PCIe RC here.
		 */
		if (pcie->mode == DW_PCIE_RC_TYPE) {
			ret = thunderbay_pcie_rc_dma_cfg(pcie, pdev);
			if (ret) {
				if (ret == -EPROBE_DEFER)
					dev_err(dev,
						"deferring probe as no RC AXI DMA\n");
				else
					dev_err(dev,
						"failed to configure RC AXI DMA\n");
				return ret;
			}
			pcie->rc_dma = true;
		}
	}

	pcie->apb_base = devm_platform_ioremap_resource_byname(pdev, "apb");
	if (IS_ERR(pcie->apb_base))
		return PTR_ERR(pcie->apb_base);

	platform_set_drvdata(pdev, pcie);

	switch (pcie->mode) {
	case DW_PCIE_RC_TYPE:
		if (!IS_ENABLED(CONFIG_PCIE_KEEMBAY_HOST))
			return -ENODEV;

		if (pcie->plt_type == PLF_HW_KEEMBAY)
			ret = keembay_pcie_add_pcie_port(pcie, pdev);
		else
			ret = thunderbay_pcie_add_pcie_port(pcie, pdev);

		return ret;

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

static const struct keembay_pcie_of_data thunderbay_pcie_rc_of_data = {
	.mode = DW_PCIE_RC_TYPE,
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
	{
		.compatible = "intel,thunderbay-pcie",
		.data = &thunderbay_pcie_rc_of_data,
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
