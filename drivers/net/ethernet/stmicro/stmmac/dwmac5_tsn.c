// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/* Copyright (c) 2019, Intel Corporation.
 * DW EQoS v5.00 TSN IP Implementation
 */
#include <linux/iopoll.h>
#include "dwmac4.h"
#include "dwmac5.h"
#include "hwif.h"
#include "stmmac_tsn.h"

enum tsn_mmc_idx {
	EST_MMC_BTRE = 0,
	EST_MMC_BTRLM = 1,
	EST_MMC_HLBF = 2,
	EST_MMC_HLBS = 3,
	EST_MMC_CGCE = 4,
};

const struct tsn_mmc_desc dwmac5_tsn_mmc_desc[STMMAC_TSN_STAT_SIZE] = {
	{ true, "BTRE" },  /* BTR Error */
	{ true, "BTRLM" }, /* BTR Maximum Loop Count Error */
	{ true, "HLBF" },  /* Head-of-Line Blocking due to Frame Size */
	{ true, "HLBS" },  /* Head-of-Line Blocking due to Scheduling */
	{ true, "CGCE" },  /* Constant Gate Control Error */
	{ false, "RESV" },
	{ false, "RESV" },
	{ false, "RESV" },
	{ false, "RESV" },
	{ false, "RESV" },
	{ false, "RESV" },
	{ false, "RESV" },
	{ false, "RESV" },
	{ false, "RESV" },
	{ false, "RESV" },
	{ false, "RESV" },
};

static int est_set_gcl_addr(void __iomem *ioaddr, u32 addr,
			    bool is_gcrr, u32 rwops, u32 dep,
			    u32 dbgb, bool is_dbgm)
{
	u32 value;

	value = MTL_EST_GCL_CTRL_ADDR_VAL(addr) & MTL_EST_GCL_CTRL_ADDR(dep);

	if (is_dbgm) {
		if (dbgb)
			value |= MTL_EST_GCL_CTRL_DBGB1;

		value |= MTL_EST_GCL_CTRL_DBGM;
	}

	if (is_gcrr)
		value |= MTL_EST_GCL_CTRL_GCRR;

	/* This is the only place SRWO is set and driver polls SRWO
	 * for self-cleared before exit. Therefore, caller should
	 * check return status for possible time out error.
	 */
	value |= (rwops | MTL_EST_GCL_CTRL_SRWO);

	writel(value, ioaddr + MTL_EST_GCL_CTRL);

	return readl_poll_timeout(ioaddr + MTL_EST_GCL_CTRL, value,
				  !(value & MTL_EST_GCL_CTRL_SRWO),
				  50000, 600000);
}

static u32 dwmac5_read_hwid(void __iomem *ioaddr)
{
	return (readl(ioaddr + GMAC4_VERSION) & TSN_VER_MASK);
}

static bool dwmac5_has_tsn_cap(void __iomem *ioaddr, enum tsn_feat_id featid)
{
	u32 hw_cap3 = readl(ioaddr + GMAC_HW_FEATURE3);

	switch (featid) {
	case TSN_FEAT_ID_EST:
		return (hw_cap3 & GMAC_HW_FEAT_ESTSEL);
	case TSN_FEAT_ID_TBS:
		return (hw_cap3 & GMAC_HW_FEAT_TBSSEL);
	default:
		return false;
	};
}

static void dwmac5_hw_setup(void __iomem *ioaddr, enum tsn_feat_id featid)
{
	u32 value;

	switch (featid) {
	case TSN_FEAT_ID_EST:
		/* Enable EST interrupts */
		value = (MTL_EST_INT_EN_CGCE | MTL_EST_INT_EN_IEHS |
			 MTL_EST_INT_EN_IEHF | MTL_EST_INT_EN_IEBE |
			 MTL_EST_INT_EN_IECC);
		writel(value, ioaddr + MTL_EST_INT_EN);
		break;
	default:
		return;
	};
}

static u32 dwmac5_est_get_gcl_depth(void __iomem *ioaddr)
{
	u32 hw_cap3;
	u32 estdep;
	u32 depth;

	hw_cap3 = readl(ioaddr + GMAC_HW_FEATURE3);
	estdep = (hw_cap3 & GMAC_HW_FEAT_ESTDEP) >> GMAC_HW_FEAT_ESTDEP_SHIFT;

	switch (estdep) {
	case 1:
		depth = 64;
		break;
	case 2:
		depth = 128;
		break;
	case 3:
		depth = 256;
		break;
	case 4:
		depth = 512;
		break;
	case 5:
		depth = 1024;
		break;
	default:
		depth = 0;
	}

	return depth;
}

static u32 dwmac5_est_get_ti_width(void __iomem *ioaddr)
{
	u32 hw_cap3;
	u32 estwid;
	u32 width;

	hw_cap3 = readl(ioaddr + GMAC_HW_FEATURE3);
	estwid = (hw_cap3 & GMAC_HW_FEAT_ESTWID) >> GMAC_HW_FEAT_ESTWID_SHIFT;

	switch (estwid) {
	case 1:
		width = 16;
		break;
	case 2:
		width = 20;
		break;
	case 3:
		width = 24;
		break;
	default:
		width = 0;
	}

	return width;
}

static u32 dwmac5_est_get_txqcnt(void __iomem *ioaddr)
{
	u32 hw_cap2 = readl(ioaddr + GMAC_HW_FEATURE2);

	return ((hw_cap2 & GMAC_HW_FEAT_TXQCNT) >> 6) + 1;
}

static void dwmac5_est_get_max(u32 *ptov_max,
			       u32 *ctov_max,
			       u32 *cycle_max,
			       u32 *idleslope_max)
{
	*ptov_max = EST_PTOV_MAX;
	*ctov_max = EST_CTOV_MAX;
	*cycle_max = EST_CTR_HI_MAX;
	*idleslope_max = CBS_IDLESLOPE_MAX;
}

static int dwmac5_est_write_gcl_config(void __iomem *ioaddr, u32 data, u32 addr,
				       bool is_gcrr,
				       u32 dbgb, bool is_dbgm)
{
	u32 dep = dwmac5_est_get_gcl_depth(ioaddr);

	dep = ilog2(dep);
	writel(data, ioaddr + MTL_EST_GCL_DATA);

	return est_set_gcl_addr(ioaddr, addr, is_gcrr, GCL_OPS_W, dep,
				dbgb, is_dbgm);
}

static int dwmac5_est_read_gcl_config(void __iomem *ioaddr, u32 *data, u32 addr,
				      bool is_gcrr,
				      u32 dbgb, bool is_dbgm)
{
	u32 dep = dwmac5_est_get_gcl_depth(ioaddr);
	int ret;

	dep = ilog2(dep);
	ret = est_set_gcl_addr(ioaddr, addr, is_gcrr, GCL_OPS_R, dep,
			       dbgb, is_dbgm);
	if (ret)
		return ret;

	*data = readl(ioaddr + MTL_EST_GCL_DATA);

	return ret;
}

static int dwmac5_est_read_gce(void __iomem *ioaddr, u32 row,
			       u32 *gates, u32 *ti_nsec,
			       u32 ti_wid, u32 txqcnt,
			       u32 dbgb, bool is_dbgm)
{
	u32 gates_mask;
	u32 ti_mask;
	u32 value;
	int ret;

	gates_mask = (1 << txqcnt) - 1;
	ti_mask = (1 << ti_wid) - 1;

	ret = dwmac5_est_read_gcl_config(ioaddr, &value, row, 0, dbgb, is_dbgm);
	if (ret)
		return ret;

	*ti_nsec = value & ti_mask;
	*gates = (value >> ti_wid) & gates_mask;

	return ret;
}

static void dwmac5_est_set_tils(void __iomem *ioaddr, const u32 tils)
{
	u32 value;

	value = readl(ioaddr + MTL_EST_CTRL);
	value &= ~(MTL_EST_CTRL_SSWL | MTL_EST_CTRL_TILS);
	value |= (tils << MTL_EST_CTRL_TILS_SHIFT);

	writel(value, ioaddr + MTL_EST_CTRL);
}

static void dwmac5_est_set_ptov(void __iomem *ioaddr, const u32 ptov)
{
	u32 value;

	value = readl(ioaddr + MTL_EST_CTRL);
	value &= ~(MTL_EST_CTRL_SSWL | MTL_EST_CTRL_PTOV);
	value |= (ptov << MTL_EST_CTRL_PTOV_SHIFT);

	writel(value, ioaddr + MTL_EST_CTRL);
}

static void dwmac5_est_set_ctov(void __iomem *ioaddr, const u32 ctov)
{
	u32 value;

	value = readl(ioaddr + MTL_EST_CTRL);
	value &= ~(MTL_EST_CTRL_SSWL | MTL_EST_CTRL_CTOV);
	value |= (ctov << MTL_EST_CTRL_CTOV_SHIFT);

	writel(value, ioaddr + MTL_EST_CTRL);
}

static int dwmac5_est_set_enable(void __iomem *ioaddr, bool enable)
{
	u32 value;

	value = readl(ioaddr + MTL_EST_CTRL);
	value &= ~(MTL_EST_CTRL_SSWL | MTL_EST_CTRL_EEST);
	value |= (enable & MTL_EST_CTRL_EEST);

	writel(value, ioaddr + MTL_EST_CTRL);

	return 0;
}

static bool dwmac5_est_get_enable(void __iomem *ioaddr)
{
	u32 value;

	value = readl(ioaddr + MTL_EST_CTRL);

	return (value & MTL_EST_CTRL_EEST);
}

static u32 dwmac5_est_get_bank(void __iomem *ioaddr, bool is_own)
{
	u32 value;

	value = readl(ioaddr + MTL_EST_STATUS);

	value = ((value & MTL_EST_STATUS_SWOL) >>
		MTL_EST_STATUS_SWOL_SHIFT);

	if (is_own)
		return value;
	else
		return (~value & 0x1);
}

static void dwmac5_est_switch_swol(void __iomem *ioaddr)
{
	u32 value;

	value = readl(ioaddr + MTL_EST_CTRL);
	value |= MTL_EST_CTRL_SSWL;

	writel(value, ioaddr + MTL_EST_CTRL);
}

int dwmac5_est_irq_status(void __iomem *ioaddr, struct net_device *dev,
			  struct tsn_mmc_stat *mmc_stat,
			  u32 txqcnt)
{
	u32 txqcnt_mask;
	u32 status;
	u32 value;
	u32 feqn;
	u32 hbfq;
	u32 hbfs;
	u32 btrl;

	txqcnt_mask = (1 << txqcnt) - 1;
	status = readl(ioaddr + MTL_EST_STATUS);

	value = (MTL_EST_STATUS_CGCE | MTL_EST_STATUS_HLBS |
		 MTL_EST_STATUS_HLBF | MTL_EST_STATUS_BTRE |
		 MTL_EST_STATUS_SWLC);

	/* Return if there is no error */
	if (!(status & value))
		return 0;

	if (status & MTL_EST_STATUS_CGCE) {
		/* Clear Interrupt */
		writel(MTL_EST_STATUS_CGCE, ioaddr + MTL_EST_STATUS);

		mmc_stat->count[EST_MMC_CGCE]++;
	}

	if (status & MTL_EST_STATUS_HLBS) {
		value = readl(ioaddr + MTL_EST_SCH_ERR);
		value &= txqcnt_mask;

		mmc_stat->count[EST_MMC_HLBS]++;

		/* Clear Interrupt */
		writel(value, ioaddr + MTL_EST_SCH_ERR);

		/* Collecting info to shows all the queues that has HLBS
		 * issue. The only way to clear this is to clear the
		 * statistic
		 */
		if (net_ratelimit())
			netdev_err(dev, "EST: HLB(sched) Queue %u\n", value);
	}

	if (status & MTL_EST_STATUS_HLBF) {
		value = readl(ioaddr + MTL_EST_FRM_SZ_ERR);
		feqn = value & txqcnt_mask;

		value = readl(ioaddr + MTL_EST_FRM_SZ_CAP);
		hbfq = (value & MTL_EST_FRM_SZ_CAP_HBFQ_MASK(txqcnt)) >>
		       MTL_EST_FRM_SZ_CAP_HBFQ_SHIFT;
		hbfs = value & MTL_EST_FRM_SZ_CAP_HBFS_MASK;

		mmc_stat->count[EST_MMC_HLBF]++;

		/* Clear Interrupt */
		writel(feqn, ioaddr + MTL_EST_FRM_SZ_ERR);

		if (net_ratelimit())
			netdev_err(dev, "EST: HLB(size) Queue %u Size %u\n",
				   hbfq, hbfs);
	}

	if (status & MTL_EST_STATUS_BTRE) {
		if ((status & MTL_EST_STATUS_BTRL) ==
		    MTL_EST_STATUS_BTRL_MAX)
			mmc_stat->count[EST_MMC_BTRLM]++;
		else
			mmc_stat->count[EST_MMC_BTRE]++;

		btrl = (status & MTL_EST_STATUS_BTRL) >>
			MTL_EST_STATUS_BTRL_SHIFT;

		if (net_ratelimit())
			netdev_info(dev, "EST: BTR Error Loop Count %u\n",
				    btrl);

		writel(MTL_EST_STATUS_BTRE, ioaddr + MTL_EST_STATUS);
	}

	if (status & MTL_EST_STATUS_SWLC) {
		writel(MTL_EST_STATUS_SWLC, ioaddr + MTL_EST_STATUS);
		netdev_info(dev, "SWOL has been switched\n");
	}

	return status;
}

const struct tsnif_ops dwmac510_tsnif_ops = {
	.read_hwid = dwmac5_read_hwid,
	.has_tsn_cap = dwmac5_has_tsn_cap,
	.hw_setup = dwmac5_hw_setup,
	.est_get_gcl_depth = dwmac5_est_get_gcl_depth,
	.est_get_ti_width = dwmac5_est_get_ti_width,
	.est_get_txqcnt = dwmac5_est_get_txqcnt,
	.est_get_max = dwmac5_est_get_max,
	.est_write_gcl_config = dwmac5_est_write_gcl_config,
	.est_read_gcl_config = dwmac5_est_read_gcl_config,
	.est_read_gce = dwmac5_est_read_gce,
	.est_set_tils = dwmac5_est_set_tils,
	.est_set_ptov = dwmac5_est_set_ptov,
	.est_set_ctov = dwmac5_est_set_ctov,
	.est_set_enable = dwmac5_est_set_enable,
	.est_get_enable = dwmac5_est_get_enable,
	.est_get_bank = dwmac5_est_get_bank,
	.est_switch_swol = dwmac5_est_switch_swol,
	.est_irq_status = dwmac5_est_irq_status,
};

void dwmac510_tsnif_setup(struct mac_device_info *mac)
{
	mac->tsnif = &dwmac510_tsnif_ops;
	mac->tsn_info.mmc_desc = &dwmac5_tsn_mmc_desc[0];
}
