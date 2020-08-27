// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2019, Intel Corporation.
 * DWC Ethernet Physical Coding Sublayer for GMII2SGMII Converter
 */
#include <linux/acpi.h>
#include <linux/bitops.h>
#include <linux/dwxpcs.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mdio.h>
#include <linux/module.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>

/* XPCS MII MMD Device Addresses */
#define XPCS_MDIO_MII_MMD	MDIO_MMD_VEND2

/* MII MMD registers offsets */
#define MDIO_MII_MMD_CTRL		0x0000	/* SR Control */
#define MDIO_MII_MMD_DIGITAL_CTRL_1	0x8000	/* Digital Control 1 */
#define MDIO_MII_MMD_AN_CTRL		0x8001	/* AN Control */
#define MDIO_MII_MMD_AN_STAT		0x8002	/* AN Status */
#define MDIO_MII_MMD_EEE_MCTRL0		0x8006	/* EEE Mode Control Register */
#define MDIO_MII_MMD_EEE_MCTRL1		0x800b	/* EEE Mode Control 1 */

/* MII MMD SR AN Advertisement & Link Partner Ability are slightly
 * different from MII_ADVERTISEMENT & MII_LPA in below fields:
 */
#define MDIO_MII_MMD_HD			BIT(6)	/* Half duplex */
#define MDIO_MII_MMD_FD			BIT(5)	/* Full duplex */
#define MDIO_MII_MMD_PSE_SHIFT		7	/* Pause Ability shift */
#define MDIO_MII_MMD_PSE		GENMASK(8, 7)	/* Pause Ability */
#define MDIO_MII_MMD_PSE_NO		0x0
#define MDIO_MII_MMD_PSE_ASYM		0x1
#define MDIO_MII_MMD_PSE_SYM		0x2
#define MDIO_MII_MMD_PSE_BOTH		0x3

/* Enable 2.5G Mode */
#define MDIO_MII_MMD_DIGI_CTRL_1_EN_2_5G_MODE	BIT(2)

/* Automatic Speed Mode Change for MAC side SGMII AN */
#define MDIO_MII_MMD_DIGI_CTRL_1_MAC_AUTO_SW	BIT(9)

/* MII MMD AN Control defines */
#define MDIO_MII_MMD_AN_CTRL_TX_CONFIG_SHIFT	3 /* TX Config shift */
#define AN_CTRL_TX_CONF_PHY_SIDE_SGMII		0x1 /* PHY side SGMII mode */
#define AN_CTRL_TX_CONF_MAC_SIDE_SGMII		0x0 /* MAC side SGMII mode */
#define MDIO_MII_MMD_AN_CTRL_PCS_MD_SHIFT	1  /* PCS Mode shift */
#define MDIO_MII_MMD_AN_CTRL_PCS_MD	GENMASK(2, 1) /* PCS Mode */
#define AN_CTRL_PCS_MD_C37_1000BASEX	0x0	/* C37 AN for 1000BASE-X */
#define AN_CTRL_PCS_MD_C37_SGMII	0x2	/* C37 AN for SGMII */
#define MDIO_MII_MMD_AN_CTRL_AN_INTR_EN	BIT(0)	/* AN Complete Intr Enable */

/* MII MMD AN Status defines for SGMII AN Status */
#define AN_STAT_C37_AN_CMPLT		BIT(0)	/* AN Complete Intr */
#define AN_STAT_SGMII_AN_FD		BIT(1)	/* Full Duplex */
#define AN_STAT_SGMII_AN_SPEED_SHIFT	2	/* AN Speed shift */
#define AN_STAT_SGMII_AN_SPEED		GENMASK(3, 2)	/* AN Speed */
#define AN_STAT_SGMII_AN_10MBPS		0x0	/* 10 Mbps */
#define AN_STAT_SGMII_AN_100MBPS	0x1	/* 100 Mbps */
#define AN_STAT_SGMII_AN_1000MBPS	0x2	/* 1000 Mbps */
#define AN_STAT_SGMII_AN_LNKSTS		BIT(4)	/* Link Status */

/* SR MII MMD Control defines */
#define AN_CL37_EN		BIT(12)	/* Enable Clause 37 auto-nego */
#define SGMII_SPEED_SS13	BIT(13)	/* SGMII speed along with SS6 */
#define SGMII_SPEED_SS6		BIT(6)	/* SGMII speed along with SS13 */

/* VR MII EEE Control defines */
#define VR_MII_EEE_LTX_EN		BIT(0)  /* LPI Tx Enable */
#define VR_MII_EEE_LRX_EN		BIT(1)  /* LPI Rx Enable */
#define VR_MII_EEE_TX_QUIET_EN		BIT(2)  /* Tx Quiet Enable */
#define VR_MII_EEE_RX_QUIET_EN		BIT(3)  /* Rx Quiet Enable */
#define VR_MII_EEE_TX_EN_CTRL		BIT(4)  /* Tx Control Enable */
#define VR_MII_EEE_RX_EN_CTRL		BIT(7)  /* Rx Control Enable */

/* VR MII EEE Control 1 defines */
#define VR_MII_EEE_TRN_LPI		BIT(0)	/* Transparent Mode Enable */

/* 100ns Clock Tic Multiplying Factor where
 * clk_eee_i freq is 19.2Mhz, clk_eee_i_time_period is 52ns
 * clk_eee_i_time_period * (MULT_FACT_100NS + 1)
 * = 52*(1+1) = 104ns (within 80 to 120 ns)
 */
#define VR_MII_EEE_MULT_FACT_100NS	BIT(8)

enum dwxpcs_state_t {
	__DWXPCS_REMOVING,
	__DWXPCS_TASK_SCHED,
};

struct pcs_stats {
	int link;
	int speed;
	int duplex;
};

struct dwxpcs_priv {
	struct phy_device *phy_dev;
	struct phy_driver *phy_drv;
	struct phy_device cached_phy_dev;
	struct phy_driver conv_phy_drv;
	struct mdio_device *mdiodev;
	struct pcs_stats stats;
	struct dwxpcs_platform_data *pdata;
	char int_name[IFNAMSIZ];
	unsigned long state;
	struct workqueue_struct *int_wq;
	struct work_struct an_task;
};

/* DW xPCS mdiobus_read and mdiobus_write helper functions */
#define xpcs_read(dev, reg) \
	mdiobus_read(bus, xpcs_addr, \
		     MII_ADDR_C45 | (reg) | \
		     ((dev) << MII_DEVADDR_C45_SHIFT))
#define xpcs_write(dev, reg, val) \
	mdiobus_write(bus, xpcs_addr, \
		      MII_ADDR_C45 | (reg) | \
		      ((dev) << MII_DEVADDR_C45_SHIFT), val)

static void dwxpcs_init(struct dwxpcs_priv *priv)
{
	struct mii_bus *bus = priv->mdiodev->bus;
	int xpcs_addr = priv->mdiodev->addr;
	int pcs_mode = priv->pdata->mode;
	bool speed_2500_en = priv->pdata->speed_2500_en;
	int phydata;

	if (speed_2500_en) {
		phydata = xpcs_read(XPCS_MDIO_MII_MMD,
				    MDIO_MII_MMD_DIGITAL_CTRL_1);
		phydata |= MDIO_MII_MMD_DIGI_CTRL_1_EN_2_5G_MODE;
		phydata &= ~MDIO_MII_MMD_DIGI_CTRL_1_MAC_AUTO_SW;
		xpcs_write(XPCS_MDIO_MII_MMD, MDIO_MII_MMD_DIGITAL_CTRL_1,
			   phydata);

		phydata = xpcs_read(XPCS_MDIO_MII_MMD, MDIO_MII_MMD_CTRL);
		phydata &= ~AN_CL37_EN;
		phydata |= SGMII_SPEED_SS6;
		phydata &= ~SGMII_SPEED_SS13;
		xpcs_write(XPCS_MDIO_MII_MMD, MDIO_MII_MMD_CTRL, phydata);

		return;
	}

	if (pcs_mode == DWXPCS_MODE_SGMII_AN) {
		/* For AN for SGMII mode, the settings are :-
		 * 1) VR_MII_AN_CTRL Bit(2:1)[PCS_MODE] = 10b (SGMII AN)
		 * 2) VR_MII_AN_CTRL Bit(3) [TX_CONFIG] = 0b (MAC side SGMII)
		 *    DW xPCS used with DW EQoS MAC is always MAC
		 *    side SGMII.
		 * 3) VR_MII_AN_CTRL Bit(0) [AN_INTR_EN] = 1b (AN Interrupt
		 *    enabled)
		 * 4) VR_MII_DIG_CTRL1 Bit(9) [MAC_AUTO_SW] = 1b (Automatic
		 *    speed/duplex mode change by HW after SGMII AN complete)
		 * Note: Since it is MAC side SGMII, there is no need to set
		 *	 SR_MII_AN_ADV. MAC side SGMII receives AN Tx Config
		 *	 from PHY about the link state change after C28 AN
		 *	 is completed between PHY and Link Partner.
		 */
		phydata = xpcs_read(XPCS_MDIO_MII_MMD, MDIO_MII_MMD_AN_CTRL);
		phydata &= ~MDIO_MII_MMD_AN_CTRL_PCS_MD;

		phydata |= MDIO_MII_MMD_AN_CTRL_AN_INTR_EN |
			   (AN_CTRL_PCS_MD_C37_SGMII <<
			    MDIO_MII_MMD_AN_CTRL_PCS_MD_SHIFT &
			    MDIO_MII_MMD_AN_CTRL_PCS_MD) |
			   (AN_CTRL_TX_CONF_MAC_SIDE_SGMII <<
			    MDIO_MII_MMD_AN_CTRL_TX_CONFIG_SHIFT);
		xpcs_write(XPCS_MDIO_MII_MMD, MDIO_MII_MMD_AN_CTRL, phydata);

		phydata = xpcs_read(XPCS_MDIO_MII_MMD,
				    MDIO_MII_MMD_DIGITAL_CTRL_1);
		phydata |= MDIO_MII_MMD_DIGI_CTRL_1_MAC_AUTO_SW;
		xpcs_write(XPCS_MDIO_MII_MMD, MDIO_MII_MMD_DIGITAL_CTRL_1,
			   phydata);
	} else {
		/* For AN for 1000BASE-X mode, the settings are :-
		 * 1) VR_MII_AN_CTRL Bit(2:1)[PCS_MODE] = 00b (1000BASE-X C37)
		 * 2) VR_MII_AN_CTRL Bit(0) [AN_INTR_EN] = 1b (AN Interrupt
		 *    enabled)
		 * 3) SR_MII_AN_ADV Bit(6)[FD] = 1b (Full Duplex)
		 *    Note: Half Duplex is rarely used, so don't advertise.
		 * 4) SR_MII_AN_ADV Bit(8:7)[PSE] = 11b (Sym & Asym Pause)
		 */
		phydata = xpcs_read(XPCS_MDIO_MII_MMD, MDIO_MII_MMD_AN_CTRL);
		phydata &= ~MDIO_MII_MMD_AN_CTRL_PCS_MD;
		phydata |= MDIO_MII_MMD_AN_CTRL_AN_INTR_EN |
			   (AN_CTRL_PCS_MD_C37_1000BASEX <<
			    MDIO_MII_MMD_AN_CTRL_PCS_MD_SHIFT &
			    MDIO_MII_MMD_AN_CTRL_PCS_MD);
		xpcs_write(XPCS_MDIO_MII_MMD, MDIO_MII_MMD_AN_CTRL, phydata);

		phydata = xpcs_read(XPCS_MDIO_MII_MMD, MII_ADVERTISE);
		phydata |= MDIO_MII_MMD_FD |
			   (MDIO_MII_MMD_PSE_BOTH << MDIO_MII_MMD_PSE_SHIFT);
		xpcs_write(XPCS_MDIO_MII_MMD, MII_ADVERTISE, phydata);
	}
	/* Enable EEE */
	phydata = VR_MII_EEE_LTX_EN | VR_MII_EEE_LRX_EN |
			VR_MII_EEE_TX_QUIET_EN |
			VR_MII_EEE_RX_QUIET_EN |
			VR_MII_EEE_TX_EN_CTRL |
			VR_MII_EEE_RX_EN_CTRL |
			VR_MII_EEE_MULT_FACT_100NS;
	xpcs_write(XPCS_MDIO_MII_MMD, MDIO_MII_MMD_EEE_MCTRL0, phydata);

	/* TODO: Implement a more flexible design/method of configuring
	 * the EEE control and timer registers, to enable generic use of
	 * the driver.
	 */

	phydata = xpcs_read(XPCS_MDIO_MII_MMD, MDIO_MII_MMD_EEE_MCTRL1);
	phydata |= VR_MII_EEE_TRN_LPI;
	xpcs_write(XPCS_MDIO_MII_MMD, MDIO_MII_MMD_EEE_MCTRL1, phydata);
}

static int dwxpcs_read_status(struct phy_device *phydev)
{
	struct dwxpcs_priv *priv = (struct dwxpcs_priv *)phydev->priv;
	struct mii_bus *bus = priv->mdiodev->bus;
	int xpcs_addr = priv->mdiodev->addr;
	int pcs_mode = priv->pdata->mode;
	int phydata;
	int err;

	if (priv->phy_drv->read_status)
		err = priv->phy_drv->read_status(phydev);
	else
		err = genphy_read_status(phydev);

	if (err < 0)
		return err;

	/* For SGMII AN, we are done as the speed/duplex are automatically
	 * set because we have initialized 'MAC_AUTO_SW' for MAC side SGMII.
	 */
	if (pcs_mode == DWXPCS_MODE_1000BASEX_AN) {
		/* For 1000BASE-X AN, we need to adjust duplex mode according
		 * to link partner. No need to update speed as it is always
		 * 1000Mbps.
		 */
		phydata = xpcs_read(XPCS_MDIO_MII_MMD, MII_BMCR);
		phydata &= ~BMCR_FULLDPLX;
		phydata |= phydev->duplex ? BMCR_FULLDPLX : 0;
		xpcs_write(XPCS_MDIO_MII_MMD, MII_BMCR, phydata);
	}

	return 0;
}

static void dwxpcs_get_linkstatus(struct dwxpcs_priv *priv, int an_stat)
{
	struct mii_bus *bus = priv->mdiodev->bus;
	struct pcs_stats *stats = &priv->stats;
	int xpcs_addr = priv->mdiodev->addr;
	int pcs_mode = priv->pdata->mode;

	if (pcs_mode == DWXPCS_MODE_SGMII_AN) {
		/* Check the SGMII AN link status */
		if (an_stat & AN_STAT_SGMII_AN_LNKSTS) {
			int speed_value;

			stats->link = 1;

			speed_value = ((an_stat & AN_STAT_SGMII_AN_SPEED) >>
					AN_STAT_SGMII_AN_SPEED_SHIFT);
			if (speed_value == AN_STAT_SGMII_AN_1000MBPS)
				stats->speed = SPEED_1000;
			else if (speed_value == AN_STAT_SGMII_AN_100MBPS)
				stats->speed = SPEED_100;
			else
				stats->speed = SPEED_10;

			if (an_stat & AN_STAT_SGMII_AN_FD)
				stats->duplex = 1;
			else
				stats->duplex = 0;
		} else {
			stats->link = 0;
		}
	} else if (pcs_mode == DWXPCS_MODE_1000BASEX_AN) {
		/* For 1000BASE-X AN, 1000BASE-X is always 1000Mbps.
		 * For duplex mode, we read from BMCR_FULLDPLX which is
		 * only valid if BMCR_ANENABLE is not enabeld.
		 */
		int phydata = xpcs_read(XPCS_MDIO_MII_MMD, MII_BMCR);

		stats->link = 1;
		stats->speed = SPEED_1000;
		if (!(phydata & BMCR_ANENABLE))
			stats->duplex = phydata & BMCR_FULLDPLX ? 1 : 0;
	}
}

static void dwxpcs_irq_handle(struct dwxpcs_priv *priv)
{
	struct mii_bus *bus = priv->mdiodev->bus;
	struct device *dev = &priv->mdiodev->dev;
	int xpcs_addr = priv->mdiodev->addr;
	int an_stat;

	/* AN status */
	an_stat = xpcs_read(XPCS_MDIO_MII_MMD, MDIO_MII_MMD_AN_STAT);

	if (an_stat & AN_STAT_C37_AN_CMPLT) {
		struct pcs_stats *stats = &priv->stats;

		dwxpcs_get_linkstatus(priv, an_stat);

		/* Clear C37 AN complete status by writing zero */
		xpcs_write(XPCS_MDIO_MII_MMD, MDIO_MII_MMD_AN_STAT, 0);

		dev_info(dev, "%s: Link = %d - %d/%s\n",
			 __func__,
			 stats->link,
			 stats->speed,
			 stats->duplex ? "Full" : "Half");
	}
}

static void dwxpcs_an_task(struct work_struct *work)
{
	struct dwxpcs_priv *priv = container_of(work,
						struct dwxpcs_priv,
						an_task);
	dwxpcs_irq_handle(priv);

	clear_bit(__DWXPCS_TASK_SCHED, &priv->state);
}

static irqreturn_t dwxpcs_interrupt(int irq, void *dev_id)
{
	struct dwxpcs_priv *priv = (struct dwxpcs_priv *)dev_id;

	/* Handle the clearing of AN status outside of interrupt context
	 * as it involves mdiobus_read() & mdiobus_write().
	 */
	if (!test_bit(__DWXPCS_REMOVING, &priv->state) &&
	    !test_and_set_bit(__DWXPCS_TASK_SCHED, &priv->state)) {
		queue_work(priv->int_wq, &priv->an_task);

		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

#ifdef CONFIG_ACPI
static const struct acpi_device_id dwxpcs_acpi_match[] = {
	{ "INTC1033" }, /* EHL Ethernet PCS */
	{ "INTC1034" }, /* TGL Ethernet PCS */
	{ },
};

MODULE_DEVICE_TABLE(acpi, dwxpcs_acpi_match);
#endif // CONFIG_ACPI

static int dwxpcs_probe(struct mdio_device *mdiodev)
{
	struct device_node *phy_node;
	struct dwxpcs_priv *priv;
	struct device_node *np;
	struct device *dev;
	int ret;

	dev = &mdiodev->dev;
	np = dev->of_node;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	if (np) {
		/* Handle mdio_device registered through devicetree */
		phy_node = of_parse_phandle(np, "phy-handle", 0);
		if (!phy_node) {
			dev_err(dev, "Couldn't parse phy-handle\n");
			return -ENODEV;
		}

		priv->phy_dev = of_phy_find_device(phy_node);
		of_node_put(phy_node);
		if (!priv->phy_dev) {
			dev_info(dev, "Couldn't find phydev\n");
			return -EPROBE_DEFER;
		}
	} else {
		/* Handle mdio_device registered through mdio_board_info */
		priv->pdata = (struct dwxpcs_platform_data *)dev->platform_data;

		priv->phy_dev = mdiobus_get_phy(mdiodev->bus,
						priv->pdata->ext_phy_addr);
	}

	if (!priv->phy_dev) {
		dev_info(dev, "Couldn't find phydev\n");
		return -EPROBE_DEFER;
	}

	if (!priv->phy_dev->drv) {
		dev_info(dev, "Attached phy not ready\n");
		return -EPROBE_DEFER;
	}

	priv->mdiodev = mdiodev;

	/* Initialize DW XPCS */
	dwxpcs_init(priv);

	priv->phy_drv = priv->phy_dev->drv;
	memcpy(&priv->conv_phy_drv, priv->phy_dev->drv,
	       sizeof(struct phy_driver));
	priv->conv_phy_drv.read_status = dwxpcs_read_status;
	/* Store a copy of phy_dev info for remove() later */
	priv->cached_phy_dev.priv = priv->phy_dev->priv;
	priv->cached_phy_dev.drv = priv->phy_dev->drv;
	priv->phy_dev->priv = priv;
	priv->phy_dev->drv = &priv->conv_phy_drv;

	if (priv->pdata->irq > 0) {
		char *int_name;

		INIT_WORK(&priv->an_task, dwxpcs_an_task);
		clear_bit(__DWXPCS_TASK_SCHED, &priv->state);

		int_name = priv->int_name;
		sprintf(int_name, "%s-%d", "dwxpcs", priv->mdiodev->dev.id);
		priv->int_wq = create_singlethread_workqueue(int_name);
		if (!priv->int_wq) {
			dev_err(dev, "%s: Failed to create workqueue\n",
				int_name);
			return -ENOMEM;
		}

		ret = request_irq(priv->pdata->irq, dwxpcs_interrupt,
				  IRQF_SHARED, int_name, priv);
		if (unlikely(ret < 0)) {
			destroy_workqueue(priv->int_wq);
			dev_err(dev, "%s: Allocating DW XPCS IRQ %d (%d)\n",
				__func__, priv->pdata->irq, ret);
			return ret;
		}
	}
	dev_info(dev, "%s: DW XPCS mdio device (IRQ: %d) probed successful\n",
		 __func__, priv->pdata->irq);

	mdiodev->priv = priv;

	return 0;
}

static void dwxpcs_remove(struct mdio_device *mdiodev)
{
	struct dwxpcs_priv *priv = (struct dwxpcs_priv *)mdiodev->priv;

	set_bit(__DWXPCS_REMOVING, &priv->state);

	/* Restore the original phy_device info */
	priv->phy_dev->priv = priv->cached_phy_dev.priv;
	priv->phy_dev->drv = priv->cached_phy_dev.drv;

	free_irq(priv->pdata->irq, priv);
	if (priv->int_wq)
		destroy_workqueue(priv->int_wq);
}

static struct mdio_driver dwxpcs_driver = {
	.probe = dwxpcs_probe,
	.remove = dwxpcs_remove,
	.mdiodrv.driver = {
		.name = "dwxpcs",
		.acpi_match_table = ACPI_PTR(dwxpcs_acpi_match),
	},
};

mdio_module_driver(dwxpcs_driver);

MODULE_DESCRIPTION("DW xPCS converter driver");
MODULE_LICENSE("GPL");
