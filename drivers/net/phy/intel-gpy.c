// SPDX-License-Identifier: GPL-2.0+
/* Copyright (C) 2020 Intel Corporation
 *
 * INTEL(R) Ethernet Network Connection GPY
 *
 */

#include <linux/intel_phy.h>
#include <linux/phy.h>
#include <linux/netdevice.h>

#define GPY_IMASK		0x19	/* interrupt mask */
#define GPY_ISTAT		0x1A	/* interrupt status */
#define GPY_INTR_WOL		BIT(15)	/* Wake-on-LAN */
#define GPY_INTR_ANC		BIT(10)	/* Auto-Neg complete */
#define GPY_INTR_DXMC		BIT(2)	/* Duplex mode change */
#define GPY_INTR_LSPC		BIT(1)	/* Link speed change */
#define GPY_INTR_LSTC		BIT(0)	/* Link state change */
#define GPY_INTR_MASK		(GPY_INTR_LSTC | \
				 GPY_INTR_LSPC | \
				 GPY_INTR_DXMC | \
				 GPY_INTR_ANC)

/* GPY VENDOR SPECIFIC 1 */
#define GPY_VSPEC1_SGMII_CTRL	0x8
#define GPY_VSPEC1_SGMII_STS	0x9
#define GPY_SGMII_ANEN		BIT(12)		/* Aneg enable */
#define GPY_SGMII_ANOK		BIT(5)		/* Aneg complete */
#define GPY_SGMII_LS		BIT(2)		/* Link status */
#define GPY_SGMII_DR_MASK	GENMASK(1, 0)	/* Data rate */
#define GPY_SGMII_DR_2500	0x3

/* GPY VENDOR SPECIFIC 2 */
#define GPY_VSPEC2_WOL_CTL	0xe06
#define GPY_WOL_EN		BIT(0)

#define GPY_VSPEV2_WOL_AD01	0xe08		/* WOL addr Byte5:Byte6 */
#define GPY_VSPEV2_WOL_AD23	0xe09		/* WOL addr Byte3:Byte4 */
#define GPY_VSPEV2_WOL_AD45	0xe0a		/* WOL addr Byte1:Byte2 */

/* WA for Q-SPEC GPY115 PHY ID */
#define INTEL_PHY_ID_GPY115_C22		0x67C9DE00

static int gpy_set_eee(struct phy_device *phydev, struct ethtool_eee *data)
{
	int cap, old_adv, adv = 0, ret;

	cap = phy_read_mmd(phydev, MDIO_MMD_PCS, MDIO_PCS_EEE_ABLE);
	if (cap < 0)
		return cap;

	old_adv = phy_read_mmd(phydev, MDIO_MMD_AN, MDIO_AN_EEE_ADV);
	if (old_adv < 0)
		return old_adv;

	if (data->eee_enabled) {
		adv = !data->advertised ? cap :
		      ethtool_adv_to_mmd_eee_adv_t(data->advertised) & cap;
		adv &= ~phydev->eee_broken_modes;
	}

	if (old_adv != adv) {
		ret = phy_write_mmd(phydev, MDIO_MMD_AN, MDIO_AN_EEE_ADV, adv);
		if (ret < 0)
			return ret;

		/* C45 access to restart autonegotiation is not supported
		 * in the GPY PHYs, hence use C22 access for this.
		 */
		ret = genphy_restart_aneg(phydev);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int gpy_soft_reset(struct phy_device *phydev)
{
	int ret;
	int phy_id;

	ret = phy_read(phydev, MII_PHYSID1);
	phy_id = ret << 16;

	ret = phy_read(phydev, MII_PHYSID2);
	phy_id |= ret;

	/* WA for GPY115 which need a soft reset even after a hard reset */
	if (phy_id == INTEL_PHY_ID_GPY115_C22)
		return genphy_soft_reset(phydev);

	return genphy_no_soft_reset(phydev);
}

static int gpy_config_aneg(struct phy_device *phydev)
{
	bool changed = false;
	u32 adv;
	int ret;

	if (phydev->autoneg == AUTONEG_DISABLE)
		return genphy_c45_pma_setup_forced(phydev);

	ret = genphy_c45_an_config_aneg(phydev);
	if (ret < 0)
		return ret;
	if (ret > 0)
		changed = true;

	adv = linkmode_adv_to_mii_ctrl1000_t(phydev->advertising);
	ret = phy_modify_changed(phydev, MII_CTRL1000,
				 ADVERTISE_1000FULL | ADVERTISE_1000HALF,
				 adv);
	if (ret < 0)
		return ret;
	if (ret > 0)
		changed = true;

	return changed ? genphy_restart_aneg(phydev) : 0;
}

static int gpy_ack_interrupt(struct phy_device *phydev)
{
	int reg;

	reg = phy_read(phydev, GPY_ISTAT);
	return (reg < 0) ? reg : 0;
}

static int gpy_did_interrupt(struct phy_device *phydev)
{
	int reg;

	reg = phy_read(phydev, GPY_ISTAT);
	return reg & GPY_INTR_MASK;
}

static int gpy_config_intr(struct phy_device *phydev)
{
	u16 mask = 0;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		mask = GPY_INTR_MASK;

	return phy_write(phydev, GPY_IMASK, mask);
}

static int gpy_read_status(struct phy_device *phydev)
{
	int ret;

	ret = genphy_update_link(phydev);
	if (ret)
		return ret;

	phydev->speed = SPEED_UNKNOWN;
	phydev->duplex = DUPLEX_UNKNOWN;
	phydev->pause = 0;
	phydev->asym_pause = 0;

	if (phydev->autoneg == AUTONEG_ENABLE && phydev->autoneg_complete) {
		/* Read the link partner's 1G advertisement */
		ret = phy_read(phydev, MII_STAT1000);
		if (ret < 0)
			return ret;
		mii_stat1000_mod_linkmode_lpa_t(phydev->lp_advertising, ret);

		/* Read the link partner's base page advertisement */
		ret = phy_read(phydev, MII_LPA);
		if (ret < 0)
			return ret;
		mii_lpa_mod_linkmode_lpa_t(phydev->lp_advertising, ret);

		/* Read the link partner's 10G advertisement */
		ret = phy_read_mmd(phydev, MDIO_MMD_AN, MDIO_AN_10GBT_STAT);
		if (ret < 0)
			return ret;
		mii_10gbt_stat_mod_linkmode_lpa_t(phydev->lp_advertising, ret);

		phy_resolve_aneg_linkmode(phydev);
	} else if (phydev->autoneg == AUTONEG_DISABLE) {
		ret = genphy_c45_read_pma(phydev);
	}

	return ret;
}

static void gpy_get_wol(struct phy_device *phydev,
			struct ethtool_wolinfo *wol)
{
	int ret = 0;

	wol->supported = WAKE_MAGIC | WAKE_PHY;
	wol->wolopts = 0;

	ret = phy_read_mmd(phydev, MDIO_MMD_VEND2, GPY_VSPEC2_WOL_CTL);

	if (ret & GPY_WOL_EN)
		wol->wolopts |= WAKE_MAGIC;

	ret = phy_read(phydev, GPY_IMASK);

	if (ret & GPY_INTR_LSTC)
		wol->wolopts |= WAKE_PHY;
}

static int gpy_set_wol(struct phy_device *phydev,
		       struct ethtool_wolinfo *wol)
{
	struct net_device *attach_dev = phydev->attached_dev;
	int ret = 0;

	if (wol->wolopts & WAKE_MAGIC) {
		/* There are discrepancy in the datasheet where the
		 * register name does not reflect correctly on the content
		 * MAC address - Byte0:Byte1:Byte2:Byte3:Byte4:Byte5
		 * GPY_VSPEV2_WOL_AD45 = Byte0:Byte1
		 * GPY_VSPEV2_WOL_AD23 = Byte2:Byte3
		 * GPY_VSPEV2_WOL_AD01 = Byte4:Byte5
		 */
		ret = phy_set_bits_mmd(phydev, MDIO_MMD_VEND2,
				       GPY_VSPEV2_WOL_AD45,
				       ((attach_dev->dev_addr[0] << 8) |
				       attach_dev->dev_addr[1]));

		if (ret < 0)
			return ret;

		ret = phy_set_bits_mmd(phydev, MDIO_MMD_VEND2,
				       GPY_VSPEV2_WOL_AD23,
				       ((attach_dev->dev_addr[2] << 8) |
				       attach_dev->dev_addr[3]));

		if (ret < 0)
			return ret;

		ret = phy_set_bits_mmd(phydev, MDIO_MMD_VEND2,
				       GPY_VSPEV2_WOL_AD01,
				       ((attach_dev->dev_addr[4] << 8) |
				       attach_dev->dev_addr[5]));

		if (ret < 0)
			return ret;

		/* Enable the WOL interrupt */
		ret = phy_write(phydev, GPY_IMASK, GPY_INTR_WOL);

		if (ret < 0)
			return ret;

		/* Enable magic packet matching */
		ret = phy_set_bits_mmd(phydev, MDIO_MMD_VEND2,
				       GPY_VSPEC2_WOL_CTL,
				       GPY_WOL_EN);

		if (ret < 0)
			return ret;

	} else {
		/* Disable magic packet matching */
		ret = phy_clear_bits_mmd(phydev, MDIO_MMD_VEND2,
					 GPY_VSPEC2_WOL_CTL,
					 GPY_WOL_EN);

		if (ret < 0)
			return ret;
	}

	if (wol->wolopts & WAKE_PHY) {
		/* Enable the link state change interrupt */
		ret = phy_set_bits(phydev, GPY_IMASK, GPY_INTR_LSTC);

		if (ret < 0)
			return ret;

		/* Clear the interrupt status register */
		ret = phy_read(phydev, GPY_ISTAT);
	} else {
		/* Disable the link state change interrupt */
		ret = phy_clear_bits(phydev, GPY_IMASK, GPY_INTR_LSTC);

		if (ret < 0)
			return ret;
	}

	return ret;
}

static struct phy_driver intel_gpy_drivers[] = {
	{
		.phy_id		= INTEL_PHY_ID_GPY,
		.phy_id_mask	= INTEL_PHY_ID_MASK,
		.name		= "INTEL(R) Ethernet Network Connection GPY",
		.get_features	= genphy_c45_pma_read_abilities,
		.aneg_done	= genphy_c45_aneg_done,
		.set_eee	= gpy_set_eee,
		.soft_reset	= gpy_soft_reset,
		.ack_interrupt	= gpy_ack_interrupt,
		.did_interrupt	= gpy_did_interrupt,
		.config_intr	= gpy_config_intr,
		.config_aneg	= gpy_config_aneg,
		.read_status	= gpy_read_status,
		.set_loopback	= genphy_loopback,
		.get_wol	= gpy_get_wol,
		.set_wol	= gpy_set_wol,
		/* @TODO:
		 * 1. latencies are not symmetrical, we need to update these
		 *    values after getting correct values from CHD.
		 * 2. Need to add latencies for GPY115.
		 */
		.latencies	= {
					.tx_latency_2500 = 1772,
					.tx_latency_1000 = 903,
					.tx_latency_100	 = 5084,
					.tx_latency_10	 = 45895,
					.rx_latency_2500 = 1772,
					.rx_latency_1000 = 903,
					.rx_latency_100	 = 5084,
					.rx_latency_10	 = 45895,
				  },
	},
};
module_phy_driver(intel_gpy_drivers);

static struct mdio_device_id __maybe_unused intel_gpy[] = {
	{ INTEL_PHY_ID_GPY, INTEL_PHY_ID_MASK },
	{ },
};
MODULE_DEVICE_TABLE(mdio, intel_gpy);

MODULE_DESCRIPTION("INTEL(R) Ethernet Network Connection GPY");
MODULE_LICENSE("GPL");
