// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel Keem Bay USB PHY driver
 * Copyright (C) 2020 Intel Corporation
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

/* USS (USB Subsystem) clock control registers */
#define USS_CPR_CLK_EN		0x00
#define USS_CPR_CLK_SET		0x04
#define USS_CPR_CLK_CLR		0x08
#define USS_CPR_RST_EN		0x10
#define USS_CPR_RST_SET		0x14
#define USS_CPR_RST_CLR		0x18

/* USS clock/reset bit fields */
#define USS_CPR_PHY_TST		BIT(6)
#define USS_CPR_LOW_JIT		BIT(5)
#define USS_CPR_CORE		BIT(4)
#define USS_CPR_SUSPEND		BIT(3)
#define USS_CPR_ALT_REF		BIT(2)
#define USS_CPR_REF		BIT(1)
#define USS_CPR_SYS		BIT(0)
#define USS_CPR_MASK		0x7f

/* USS APB slave registers */
#define USS_USB_CTRL_VERSION		0x00
#define USS_USB_CTRL_CFG0		0x10
#define  VCC_RESET_N_MASK		BIT(31)
#define  VCC_RESET_N(x)			(((x) << 31) & BIT(31))
#define USS_USB_PHY_CFG0		0x30
#define  POR_MASK			BIT(15)
#define  POR(x)				(((x) << 15) & BIT(15))
#define  PHY_RESET_MASK			BIT(14)
#define  PHY_RESET(x)			(((x) << 14) & BIT(14))
#define  PHY_REF_USE_PAD_MASK		BIT(5)
#define  PHY_REF_USE_PAD(x)		(((x) << 5) & BIT(5))
#define USS_USB_PHY_CFG6		0x64
#define  PHY0_SRAM_EXT_LD_DONE_MASK	BIT(23)
#define  PHY0_SRAM_EXT_LD_DONE(x)	(((x) << 23) & BIT(23))
#define USS_USB_PARALLEL_IF_CTRL	0xa0
#define  USB_PHY_CR_PARA_SEL_MASK	BIT(2)
#define  USB_PHY_CR_PARA_SEL(x)		(((x) << 2) & BIT(2))
#define USS_USB_TSET_SIGNALS_AND_GLOB	0xac
#define  USB_PHY_CR_PARA_CLK_EN_MASK	BIT(7)
#define  USB_PHY_CR_PARA_CLK_EN(x)	(((x) << 7) & BIT(7))
#define USS_USB_STATUS_REG		0xb8
#define  PHY0_SRAM_INIT_DONE_MASK	BIT(3)
#define  PHY0_SRAM_INIT_DONE(x)		(((x) << 3) & BIT(3))
#define USS_USB_TIEOFFS_CONSTANTS_REG1	0xc0
#define  IDDQ_ENABLE_MASK		BIT(10)
#define  IDDQ_ENABLE(x)			(((x) << 10) & BIT(10))

struct keembay_usb_phy {
	struct device *dev;
	struct regmap *regmap_cpr;
	struct regmap *regmap_slv;
};

static const struct regmap_config keembay_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
};

static int keembay_usb_clocks_on(struct keembay_usb_phy *priv)
{
	int ret;

	/* Enable USB subsystem clocks */
	ret = regmap_update_bits(priv->regmap_cpr, USS_CPR_CLK_SET,
				 USS_CPR_MASK, USS_CPR_MASK);
	if (ret) {
		dev_err(priv->dev, "error clock set: %d\n", ret);
		return ret;
	}

	ret = regmap_update_bits(priv->regmap_cpr, USS_CPR_RST_SET,
				 USS_CPR_MASK, USS_CPR_MASK);
	if (ret) {
		dev_err(priv->dev, "error reset set: %d\n", ret);
		return ret;
	}

	/*
	 * Clear IDDQ enable bit ensuring all analog blocks are powered
	 * up in USB2.0 PHY.
	 */
	ret = regmap_update_bits(priv->regmap_slv,
				 USS_USB_TIEOFFS_CONSTANTS_REG1,
				 IDDQ_ENABLE_MASK, IDDQ_ENABLE(0));
	if (ret) {
		dev_err(priv->dev, "error iddq enable: %d\n", ret);
		return ret;
	}

	usleep_range(30, 50);

	/*
	 * Set bit 5 of USS_USB_PHY_CFG0 register.
	 * This selects the external reference pads as the inputs for the
	 * ref clk source as oppose to the alternate ref clk.
	 */
	ret = regmap_update_bits(priv->regmap_slv, USS_USB_PHY_CFG0,
				 PHY_REF_USE_PAD_MASK, PHY_REF_USE_PAD(1));
	if (ret)
		dev_err(priv->dev, "error ref clock select: %d\n", ret);

	return ret;
}

static int keembay_usb_core_off(struct keembay_usb_phy *priv)
{
	int ret;

	/* Send the core back to the reset state */
	ret = regmap_update_bits(priv->regmap_slv, USS_USB_CTRL_CFG0,
				 VCC_RESET_N_MASK, VCC_RESET_N(0));
	if (ret)
		dev_err(priv->dev, "error core reset: %d\n", ret);

	return ret;
}

static int keembay_usb_core_on(struct keembay_usb_phy *priv)
{
	int ret;

	/* Release core reset (so that AXI slave can be used) */
	ret = regmap_update_bits(priv->regmap_slv, USS_USB_CTRL_CFG0,
				 VCC_RESET_N_MASK, VCC_RESET_N(1));
	if (ret)
		dev_err(priv->dev, "error core on: %d\n", ret);

	return ret;
}

static int keembay_usb_phys_on(struct keembay_usb_phy *priv)
{
	int ret;

	/* Bring PHYs out of reset */
	ret = regmap_update_bits(priv->regmap_slv, USS_USB_PHY_CFG0,
				 POR_MASK | PHY_RESET_MASK,
				 POR(0) | PHY_RESET(0));
	if (ret)
		dev_err(priv->dev, "error phys on: %d\n", ret);

	return ret;
}

static int keembay_usb_phy_init(struct phy *phy)
{
	struct keembay_usb_phy *priv = phy_get_drvdata(phy);
	u32 val;
	int ret;

	/* Reset the controller and both the USB2.0 and USB3.1 PHYs */
	ret = keembay_usb_core_off(priv);
	if (ret)
		return ret;

	/* Wait minimum 20us after clock enable */
	usleep_range(20, 50);

	/* to bring PHYs out of reset */
	ret = keembay_usb_phys_on(priv);
	if (ret)
		return ret;

	/*
	 * Enable USB3.1 Control Register Parallel Interface
	 *
	 * (1) Clear bit 7 of USS_USB_TSET_SIGNALS_AND_GLOB register
	 *     to disable the clock into the USB3.X parallel interface.
	 * (2) Wait 2us.
	 * (3) Set bit 2 of USS_USB_PARALLEL_IF_CTRL register to choose
	 *     CR interface instead of JTAG.
	 * (4) Set bit 7 of USS_USB_TSET_SIGNALS_AND_GLOB register
	 *     to reenable the clock into the USB3.X parallel interface.
	 */
	ret = regmap_update_bits(priv->regmap_slv,
				 USS_USB_TSET_SIGNALS_AND_GLOB,
				 USB_PHY_CR_PARA_CLK_EN_MASK,
				 USB_PHY_CR_PARA_CLK_EN(0));
	if (ret) {
		dev_err(priv->dev, "error cr clock disable: %d\n", ret);
		return ret;
	}

	usleep_range(2, 10);

	ret = regmap_update_bits(priv->regmap_slv,
				 USS_USB_PARALLEL_IF_CTRL,
				 USB_PHY_CR_PARA_SEL_MASK,
				 USB_PHY_CR_PARA_SEL(1));
	if (ret) {
		dev_err(priv->dev, "error cr select: %d\n", ret);
		return ret;
	}

	ret = regmap_update_bits(priv->regmap_slv,
				 USS_USB_TSET_SIGNALS_AND_GLOB,
				 USB_PHY_CR_PARA_CLK_EN_MASK,
				 USB_PHY_CR_PARA_CLK_EN(1));
	if (ret) {
		dev_err(priv->dev, "error cr clock enable: %d\n", ret);
		return ret;
	}

	/* Wait for USB3.1 PHY0 SRAM init done */
	ret = regmap_read_poll_timeout(priv->regmap_slv, USS_USB_STATUS_REG,
				       val, (val & PHY0_SRAM_INIT_DONE_MASK),
				       USEC_PER_MSEC, 10 * USEC_PER_MSEC);
	if (ret) {
		dev_err(priv->dev, "SRAM init not done: %d\n", ret);
		return ret;
	}

	/* Set the SRAM load done bit */
	ret = regmap_update_bits(priv->regmap_slv, USS_USB_PHY_CFG6,
				 PHY0_SRAM_EXT_LD_DONE_MASK,
				 PHY0_SRAM_EXT_LD_DONE(1));
	if (ret) {
		dev_err(priv->dev, "error SRAM init done set: %d\n", ret);
		return ret;
	}

	/* Wait 20us */
	usleep_range(20, 50);

	/* and release the controller reset */
	return keembay_usb_core_on(priv);
}

static const struct phy_ops ops = {
	.init		= keembay_usb_phy_init,
	.owner		= THIS_MODULE,
};

static int keembay_usb_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct keembay_usb_phy *priv;
	struct phy *generic_phy;
	struct phy_provider *phy_provider;
	struct resource *res;
	void __iomem *base;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "cpr-apb-base");
	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	priv->regmap_cpr = devm_regmap_init_mmio(dev, base,
						 &keembay_regmap_config);
	if (IS_ERR(priv->regmap_cpr))
		return PTR_ERR(priv->regmap_cpr);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "slv-apb-base");
	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	priv->regmap_slv = devm_regmap_init_mmio(dev, base,
						 &keembay_regmap_config);
	if (IS_ERR(priv->regmap_slv))
		return PTR_ERR(priv->regmap_slv);

	generic_phy = devm_phy_create(dev, np, &ops);
	if (IS_ERR(generic_phy)) {
		dev_err(dev, "failed to create PHY\n");
		return PTR_ERR(generic_phy);
	}

	phy_set_drvdata(generic_phy, priv);
	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(phy_provider)) {
		dev_err(dev, "failed to register phy provider\n");
		return PTR_ERR(phy_provider);
	}

	/* Setup USB subsystem clocks */
	ret = keembay_usb_clocks_on(priv);
	if (ret)
		return ret;

	/* and turn on the DWC3 core, prior to DWC3 driver init */
	return keembay_usb_core_on(priv);
}

static const struct of_device_id keembay_usb_phy_dt_ids[] = {
	{ .compatible = "intel,keembay-usb-phy" },
	{}
};

MODULE_DEVICE_TABLE(of, keembay_usb_phy_dt_ids);

static struct platform_driver keembay_usb_phy_driver = {
	.probe		= keembay_usb_phy_probe,
	.driver		= {
		.name	= "keembay-usb-phy",
		.of_match_table = keembay_usb_phy_dt_ids,
	},
};
module_platform_driver(keembay_usb_phy_driver);

MODULE_AUTHOR("Wan Ahmad Zainie <wan.ahmad.zainie.wan.mohamad@intel.com>");
MODULE_DESCRIPTION("Intel Keem Bay USB PHY driver");
MODULE_LICENSE("GPL v2");
