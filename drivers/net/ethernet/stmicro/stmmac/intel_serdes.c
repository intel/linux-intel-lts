// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2019, Intel Corporation
 * Intel Serdes
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/mdio.h>
#include "intel_serdes.h"
#include "stmmac.h"

static int serdes_status_poll(struct stmmac_priv *priv, int phyaddr,
			      int phyreg, u32 mask, u32 val)
{
	unsigned int retries = 10;
	int val_rd = 0;

	do {
		val_rd = mdiobus_read(priv->mii, phyaddr, phyreg);
		if ((val_rd & mask) == (val & mask))
			return 0;
		udelay(POLL_DELAY_US);
	} while (--retries);

	return -ETIMEDOUT;
}

static int intel_serdes_powerup(struct net_device *ndev)
{
	struct stmmac_priv *priv = netdev_priv(ndev);
	int serdes_phy_addr = 0;
	u32 data = 0;

	if (!priv->plat->intel_adhoc_addr)
		return 0;

	serdes_phy_addr = priv->plat->intel_adhoc_addr;

	/* Set the serdes rate and the PCLK rate */
	data = mdiobus_read(priv->mii, serdes_phy_addr,
			    SERDES_GCR0);

	data &= ~SERDES_RATE_MASK;
	data &= ~SERDES_PCLK_MASK;

	if (priv->plat->speed_2500_en)
		data |= SERDES_RATE_PCIE_GEN2 << SERDES_RATE_PCIE_SHIFT |
			SERDES_PCLK_37p5MHZ << SERDES_PCLK_SHIFT;
	else
		data |= SERDES_RATE_PCIE_GEN1 << SERDES_RATE_PCIE_SHIFT |
			SERDES_PCLK_70MHZ << SERDES_PCLK_SHIFT;

	mdiobus_write(priv->mii, serdes_phy_addr, SERDES_GCR0, data);

	/* assert clk_req */
	data = mdiobus_read(priv->mii, serdes_phy_addr,
			    SERDES_GCR0);

	data |= SERDES_PLL_CLK;

	mdiobus_write(priv->mii, serdes_phy_addr,
		      SERDES_GCR0, data);

	/* check for clk_ack assertion */
	data = serdes_status_poll(priv, serdes_phy_addr,
				  SERDES_GSR0,
				  SERDES_PLL_CLK,
				  SERDES_PLL_CLK);

	if (data && !priv->plat->serdes_pse_sgmii_wa) {
		dev_err(priv->device, "Serdes PLL clk request timeout\n");
		return data;
	}

	/* assert lane reset */
	data = mdiobus_read(priv->mii, serdes_phy_addr,
			    SERDES_GCR0);

	data |= SERDES_RST;

	mdiobus_write(priv->mii, serdes_phy_addr,
		      SERDES_GCR0, data);

	/* check for assert lane reset reflection */
	data = serdes_status_poll(priv, serdes_phy_addr,
				  SERDES_GSR0,
				  SERDES_RST,
				  SERDES_RST);

	if (data && !priv->plat->serdes_pse_sgmii_wa) {
		dev_err(priv->device, "Serdes assert lane reset timeout\n");
		return data;
	}

	/*  move power state to P0 */
	data = mdiobus_read(priv->mii, serdes_phy_addr,
			    SERDES_GCR0);

	data &= ~SERDES_PWR_ST_MASK;
	data |= SERDES_PWR_ST_P0 << SERDES_PWR_ST_SHIFT;

	mdiobus_write(priv->mii, serdes_phy_addr,
		      SERDES_GCR0, data);

	/* Check for P0 state */
	data = serdes_status_poll(priv, serdes_phy_addr,
				  SERDES_GSR0,
				  SERDES_PWR_ST_MASK,
				  SERDES_PWR_ST_P0 << SERDES_PWR_ST_SHIFT);

	if (data && !priv->plat->serdes_pse_sgmii_wa) {
		dev_err(priv->device, "Serdes power state P0 timeout.\n");
		return data;
	}

	/* Ungate SGMII PHY Rx Clock */
	if (priv->plat->is_pse && !priv->plat->serdes_pse_sgmii_wa) {
		data = mdiobus_read(priv->mii, serdes_phy_addr,
				    SERDES_GCR0);

		data |= SERDES_PHY_RX_CLK;

		mdiobus_write(priv->mii, serdes_phy_addr,
			      SERDES_GCR0, data);
	}

	return 0;
}

static int intel_serdes_powerdown(struct net_device *ndev)
{
	struct stmmac_priv *priv = netdev_priv(ndev);
	int serdes_phy_addr = 0;
	u32 data = 0;

	serdes_phy_addr = priv->plat->intel_adhoc_addr;

	if (!priv->plat->intel_adhoc_addr)
		return 0;

	/* Gate SGMII PHY Rx Clock */
	if (priv->plat->is_pse && !priv->plat->serdes_pse_sgmii_wa) {
		data = mdiobus_read(priv->mii, serdes_phy_addr,
				    SERDES_GCR0);

		data &= ~SERDES_PHY_RX_CLK;

		mdiobus_write(priv->mii, serdes_phy_addr,
			      SERDES_GCR0, data);
	}

	/*  move power state to P3 */
	data = mdiobus_read(priv->mii, serdes_phy_addr,
			    SERDES_GCR0);

	data &= ~SERDES_PWR_ST_MASK;
	data |= SERDES_PWR_ST_P3 << SERDES_PWR_ST_SHIFT;

	mdiobus_write(priv->mii, serdes_phy_addr,
		      SERDES_GCR0, data);

	/* Check for P3 state */
	data = serdes_status_poll(priv, serdes_phy_addr,
				  SERDES_GSR0,
				  SERDES_PWR_ST_MASK,
				  SERDES_PWR_ST_P3 << SERDES_PWR_ST_SHIFT);

	if (data && !priv->plat->serdes_pse_sgmii_wa) {
		dev_err(priv->device, "Serdes power state P3 timeout\n");
		return data;
	}

	/* de-assert clk_req */
	data = mdiobus_read(priv->mii, serdes_phy_addr,
			    SERDES_GCR0);

	data &= ~SERDES_PLL_CLK;

	mdiobus_write(priv->mii, serdes_phy_addr,
		      SERDES_GCR0, data);

	/* check for clk_ack de-assert */
	data = serdes_status_poll(priv, serdes_phy_addr,
				  SERDES_GSR0,
				  SERDES_PLL_CLK,
				  (u32)~SERDES_PLL_CLK);

	if (data && !priv->plat->serdes_pse_sgmii_wa) {
		dev_err(priv->device, "Serdes PLL clk de-assert timeout\n");
		return data;
	}

	/* de-assert lane reset */
	data = mdiobus_read(priv->mii, serdes_phy_addr,
			    SERDES_GCR0);

	data &= ~SERDES_RST;

	mdiobus_write(priv->mii, serdes_phy_addr,
		      SERDES_GCR0, data);

	/* check for de-assert lane reset reflection */
	data = serdes_status_poll(priv, serdes_phy_addr,
				  SERDES_GSR0,
				  SERDES_RST,
				  (u32)~SERDES_RST);

	if (data && !priv->plat->serdes_pse_sgmii_wa) {
		dev_err(priv->device, "Serdes de-assert lane reset timeout\n");
		return data;
	}

	return 0;
}

static int intel_speed_mode_2500(struct net_device *ndev)
{
	struct stmmac_priv *priv = netdev_priv(ndev);
	int serdes_phy_addr = 0;
	u32 data = 0;

	serdes_phy_addr = priv->plat->intel_adhoc_addr;

	/* Determine the link speed mode: 2.5Gbps/1Gbps */
	data = mdiobus_read(priv->mii, serdes_phy_addr,
			    SERDES_GCR);

	if (((data & SERDES_LINK_MODE_MASK) >> SERDES_LINK_MODE_SHIFT) ==
	    SERDES_LINK_MODE_2G5) {
		dev_info(priv->device, "Link Speed Mode: 2.5Gbps\n");
		return 1;
	} else {
		return 0;
	}
}

const struct stmmac_serdes_ops intel_serdes_ops = {
	.serdes_powerup = intel_serdes_powerup,
	.serdes_powerdown = intel_serdes_powerdown,
	.speed_mode_2500 = intel_speed_mode_2500,
};
