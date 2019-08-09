/* SPDX-License-Identifier: (GPL-2.0 OR MIT) */
// Copyright (c) 2017 Synopsys, Inc. and/or its affiliates.
// Copyright (c) 2019, Intel Corporation.
// stmmac Support for 5.xx Ethernet QoS cores

#ifndef __DWMAC5_H__
#define __DWMAC5_H__

#define MAC_DPP_FSM_INT_STATUS		0x00000140
#define MAC_AXI_SLV_DPE_ADDR_STATUS	0x00000144
#define MAC_FSM_CONTROL			0x00000148
#define PRTYEN				BIT(1)
#define TMOUTEN				BIT(0)

#define MAC_PPS_CONTROL			0x00000b70
#define PPS_MAXIDX(x)			((((x) + 1) * 8) - 1)
#define PPS_MINIDX(x)			((x) * 8)
#define PPSx_MASK(x)			GENMASK(PPS_MAXIDX(x), PPS_MINIDX(x))
#define MCGRENx(x)			BIT(PPS_MAXIDX(x))
#define TRGTMODSELx(x, val)		\
	GENMASK(PPS_MAXIDX(x) - 1, PPS_MAXIDX(x) - 2) & \
	((val) << (PPS_MAXIDX(x) - 2))
#define PPSCMDx(x, val)			\
	GENMASK(PPS_MINIDX(x) + 3, PPS_MINIDX(x)) & \
	((val) << PPS_MINIDX(x))
#define PPSEN0				BIT(4)
#define MAC_PPSx_TARGET_TIME_SEC(x)	(0x00000b80 + ((x) * 0x10))
#define MAC_PPSx_TARGET_TIME_NSEC(x)	(0x00000b84 + ((x) * 0x10))
#define TRGTBUSY0			BIT(31)
#define TTSL0				GENMASK(30, 0)
#define MAC_PPSx_INTERVAL(x)		(0x00000b88 + ((x) * 0x10))
#define MAC_PPSx_WIDTH(x)		(0x00000b8c + ((x) * 0x10))

#define MTL_RXP_CONTROL_STATUS		0x00000ca0
#define RXPI				BIT(31)
#define NPE				GENMASK(23, 16)
#define NVE				GENMASK(7, 0)
#define MTL_RXP_IACC_CTRL_STATUS	0x00000cb0
#define STARTBUSY			BIT(31)
#define RXPEIEC				GENMASK(22, 21)
#define RXPEIEE				BIT(20)
#define WRRDN				BIT(16)
#define ADDR				GENMASK(15, 0)
#define MTL_RXP_IACC_DATA		0x00000cb4
#define MTL_ECC_CONTROL			0x00000cc0
#define TSOEE				BIT(4)
#define MRXPEE				BIT(3)
#define MESTEE				BIT(2)
#define MRXEE				BIT(1)
#define MTXEE				BIT(0)

#define MTL_SAFETY_INT_STATUS		0x00000cc4
#define MCSIS				BIT(31)
#define MEUIS				BIT(1)
#define MECIS				BIT(0)
#define MTL_ECC_INT_ENABLE		0x00000cc8
#define RPCEIE				BIT(12)
#define ECEIE				BIT(8)
#define RXCEIE				BIT(4)
#define TXCEIE				BIT(0)
#define MTL_ECC_INT_STATUS		0x00000ccc
#define MTL_DPP_CONTROL			0x00000ce0
#define EPSI				BIT(2)
#define OPE				BIT(1)
#define EDPP				BIT(0)

#define DMA_SAFETY_INT_STATUS		0x00001080
#define MSUIS				BIT(29)
#define MSCIS				BIT(28)
#define DEUIS				BIT(1)
#define DECIS				BIT(0)
#define DMA_ECC_INT_ENABLE		0x00001084
#define TCEIE				BIT(0)
#define DMA_ECC_INT_STATUS		0x00001088

/* MDIO interrupt enable in MAC_Interrupt_Enable register */
#define GMAC_INT_MDIO_EN		BIT(18)

/* EQoS version 5.xx VLAN Tag Filter Fail Packets Queuing */
#define GMAC_RXQ_CTRL4			0x00000094
#define GMAC_RXQCTRL_VFFQ_MASK		GENMASK(19, 17)
#define GMAC_RXQCTRL_VFFQ_SHIFT		17
#define GMAC_RXQCTRL_VFFQE		BIT(16)

/* DWMAC v5.xx supports the following Time Sensitive Networking protocols:
 * 1) IEEE 802.1Qbv Enhancements for Scheduled Traffic (EST)
 */

/* MAC HW features3 bitmap */
#define GMAC_HW_FEAT_TBSSEL		BIT(27)
#define GMAC_HW_FEAT_ESTWID		GENMASK(21, 20)
#define GMAC_HW_FEAT_ESTWID_SHIFT	20
#define GMAC_HW_FEAT_ESTDEP		GENMASK(19, 17)
#define GMAC_HW_FEAT_ESTDEP_SHIFT	17
#define GMAC_HW_FEAT_ESTSEL		BIT(16)

/* MTL EST control register */
#define MTL_EST_CTRL			0x00000c50
#define MTL_EST_CTRL_PTOV		GENMASK(31, 24)
#define MTL_EST_CTRL_PTOV_SHIFT		24
#define MTL_EST_CTRL_CTOV		GENMASK(23, 12)
#define MTL_EST_CTRL_CTOV_SHIFT		12
#define MTL_EST_CTRL_TILS		GENMASK(10, 8)
#define MTL_EST_CTRL_TILS_SHIFT		8
#define MTL_EST_CTRL_SSWL		BIT(1)	/* Switch to SWOL */
#define MTL_EST_CTRL_EEST		BIT(0)	/* Enable EST */

/* MTL EST status register */
#define MTL_EST_STATUS			0x00000c58
#define MTL_EST_STATUS_BTRL		GENMASK(11, 8)	/* BTR ERR loop cnt */
#define MTL_EST_STATUS_BTRL_SHIFT	8
#define MTL_EST_STATUS_BTRL_MAX		(0xF << 8)
#define MTL_EST_STATUS_SWOL		BIT(7)	/* SW owned list */
#define MTL_EST_STATUS_SWOL_SHIFT	7
#define MTL_EST_STATUS_CGCE		BIT(4)	/* Constant gate ctrl err */
#define MTL_EST_STATUS_HLBS		BIT(3)	/* HLB due to scheduling */
#define MTL_EST_STATUS_HLBF		BIT(2)	/* HLB due to frame size */
#define MTL_EST_STATUS_BTRE		BIT(1)	/* BTR Error */
#define MTL_EST_STATUS_SWLC		BIT(0)	/* Switch to SWOL complete */

/* MTL EST Scheduling error */
#define MTL_EST_SCH_ERR			0x00000c60
#define MTL_EST_FRM_SZ_ERR		0x00000c64
#define MTL_EST_FRM_SZ_CAP		0x00000c68
#define MTL_EST_FRM_SZ_CAP_HBFS_MASK	GENMASK(14, 0)
#define MTL_EST_FRM_SZ_CAP_HBFQ_SHIFT	16
#define MTL_EST_FRM_SZ_CAP_HBFQ_MASK(x)	((x) > 4 ? GENMASK(18, 16) : \
					 (x) > 2 ? GENMASK(17, 16) : \
					 BIT(16))

/* MTL EST interrupt enable */
#define MTL_EST_INT_EN			0x00000c70
#define MTL_EST_INT_EN_CGCE		BIT(4)
#define MTL_EST_INT_EN_IEHS		BIT(3)
#define MTL_EST_INT_EN_IEHF		BIT(2)
#define MTL_EST_INT_EN_IEBE		BIT(1)
#define MTL_EST_INT_EN_IECC		BIT(0)

/* MTL EST GCL control register */
#define MTL_EST_GCL_CTRL		0x00000c80
#define MTL_EST_GCL_CTRL_ADDR(dep)	GENMASK(8 + (dep) - 1, 8) /* GCL Addr */
#define MTL_EST_GCL_CTRL_ADDR_VAL(addr)	((addr) << 8)
#define GCL_CTRL_ADDR_BTR_LO		0x0
#define GCL_CTRL_ADDR_BTR_HI		0x1
#define GCL_CTRL_ADDR_CTR_LO		0x2
#define GCL_CTRL_ADDR_CTR_HI		0x3
#define GCL_CTRL_ADDR_TER		0x4
#define GCL_CTRL_ADDR_LLR		0x5
#define MTL_EST_GCL_CTRL_DBGB1		BIT(5)	/* Debug Mode Bank Select */
#define MTL_EST_GCL_CTRL_DBGM		BIT(4)	/* Debug Mode */
#define MTL_EST_GCL_CTRL_GCRR		BIT(2)	/* GC Related Registers */
#define MTL_EST_GCL_CTRL_R1W0		BIT(1)	/* Read / Write Operation */
#define GCL_OPS_R			BIT(1)
#define GCL_OPS_W			0
#define MTL_EST_GCL_CTRL_SRWO		BIT(0)	/* Start R/W Operation */

/* MTL EST GCL data register */
#define MTL_EST_GCL_DATA		0x00000c84

/* EST Global defines */
#define EST_CTR_HI_MAX			0xff	/* CTR Hi is 8-bit only */
#define EST_PTOV_MAX			0xff	/* Max PTP time offset */
#define EST_CTOV_MAX			0xfff	/* Max Current time offset */

/* CBS Global defines */
#define CBS_IDLESLOPE_MAX		0x1fffff

/* DMA Tx Channel X Control register TBS bits defines */
#define DMA_CONTROL_EDSE		BIT(28)

/* MTL TBS Control register */
#define MTL_TBS_CTRL			0x00000c40
#define MTL_TBS_CTRL_LEOS		GENMASK(31, 8)
#define MTL_TBS_CTRL_LEOS_SHIFT		8
#define MTL_TBS_CTRL_LEGOS		GENMASK(6, 4)
#define MTL_TBS_CTRL_LEGOS_SHIFT	4
#define MTL_TBS_CTRL_LEOV		BIT(1)
#define MTL_TBS_CTRL_ESTM		BIT(0)

/* TBS Global defines */
#define TBS_LEOS_MAX			999999999	/* Max LEOS (ns) */
#define TBS_LEGOS_MAX			7		/* Max LE GSN Slot */

/* MAC Core Version */
#define TSN_VER_MASK		0xFF

int dwmac5_safety_feat_config(void __iomem *ioaddr, unsigned int asp);
int dwmac5_safety_feat_irq_status(struct net_device *ndev,
		void __iomem *ioaddr, unsigned int asp,
		struct stmmac_safety_stats *stats);
int dwmac5_safety_feat_dump(struct stmmac_safety_stats *stats,
			int index, unsigned long *count, const char **desc);
int dwmac5_rxp_config(void __iomem *ioaddr, struct stmmac_tc_entry *entries,
		      unsigned int count);
int dwmac5_flex_pps_config(void __iomem *ioaddr, int index,
			   struct stmmac_pps_cfg *cfg, bool enable,
			   u32 sub_second_inc, u32 systime_flags);
void dwmac510_tsnif_setup(struct mac_device_info *mac);

#endif /* __DWMAC5_H__ */
