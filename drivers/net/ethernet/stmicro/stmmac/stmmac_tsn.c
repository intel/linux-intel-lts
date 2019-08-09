// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/* Copyright (c) 2019, Intel Corporation.
 * TSN General APIs
 */
#include <linux/iopoll.h>
#include <linux/string.h>
#include <linux/time64.h>
#include "stmmac_ptp.h"
#include "common.h"
#include "stmmac.h"

static u32 est_get_gcl_total_intervals_nsec(struct est_gc_config *gcc,
					    u32 bank, u32 gcl_len)
{
	struct est_gc_entry *gcl = gcc->gcb[bank].gcl;
	u32 nsec = 0;
	u32 row;

	for (row = 0; row < gcl_len; row++) {
		nsec += gcl->ti_nsec;
		gcl++;
	}

	return nsec;
}

static u64 est_get_all_open_time(struct est_gc_config *est_gcc,
				 u32 bank,
				 u64 cycle_ns,
				 u32 queue)
{
	u32 gate = 0x1 << queue;
	u64 tti_ns = 0;
	u64 total = 0;
	struct est_gc_entry *gcl;
	u32 gcl_len;
	int row;

	gcl_len = est_gcc->gcb[bank].gcrr.llr;
	gcl = est_gcc->gcb[bank].gcl;

	/* GCL which exceeds the cycle time will be truncated.
	 * So, time interval that exceeds the cycle time will not be
	 * included.
	 */
	for (row = 0; row < gcl_len; row++) {
		tti_ns += gcl->ti_nsec;

		if (gcl->gates & gate) {
			if (tti_ns <= cycle_ns)
				total += gcl->ti_nsec;
			else
				total += gcl->ti_nsec -
					 (tti_ns - cycle_ns);
		}

		gcl++;
	}

	/* The gates wihtout any setting of open/close within
	 * the cycle time are considered as open.
	 */
	if (tti_ns < cycle_ns)
		total += cycle_ns - tti_ns;

	return total;
}

int tsn_init(struct mac_device_info *hw, struct net_device *dev)
{
	struct tsnif_info *info = &hw->tsn_info;
	struct device *pdev = dev->dev.parent;
	void __iomem *ioaddr = hw->pcsr;
	struct est_gc_entry *gcl;
	struct tsn_hw_cap *cap;
	u32 gcl_depth;
	u32 tils_max;
	u32 ti_wid;
	u32 bank;
	u32 hwid;
	int ret;

	/* Init TSN HW Cap */
	cap = &hw->tsn_info.cap;
	memset(cap, 0, sizeof(*cap));

	hwid = tsnif_read_hwid(hw, ioaddr);
	if (hwid < MIN_TSN_CORE_VER) {
		dev_info(pdev, "IP v5.00 does not support TSN\n");
		cap->est_support = 0;
		return 0;
	}

	if (!tsnif_has_tsn_cap(hw, ioaddr, TSN_FEAT_ID_EST)) {
		dev_info(pdev, "EST NOT supported\n");
		cap->est_support = 0;
		return 0;
	}

	if (!tsnif_has_tsn_cap(hw, ioaddr, TSN_FEAT_ID_TBS)) {
		dev_info(pdev, "TBS NOT supported\n");
		cap->tbs_support = 0;
	} else {
		dev_info(pdev, "TBS capable\n");
		cap->tbs_support = 1;
	}

	gcl_depth = tsnif_est_get_gcl_depth(hw, ioaddr);
	if (gcl_depth < 0) {
		dev_err(pdev, "EST GCL depth(%d) < 0\n", gcl_depth);
		cap->est_support = 0;
		return -EINVAL;
	}

	for (bank = 0; bank < EST_GCL_BANK_MAX; bank++) {
		gcl = devm_kzalloc(pdev,
				   (sizeof(*gcl) * gcl_depth),
				   GFP_KERNEL);
		if (!gcl) {
			ret = -ENOMEM;
			break;
		}
		info->est_gcc.gcb[bank].gcl = gcl;
		ret = 0;
	}
	if (ret) {
		int i;

		for (i = bank - 1; i >= 0; i--) {
			gcl = info->est_gcc.gcb[bank].gcl;
			devm_kfree(pdev, gcl);
			info->est_gcc.gcb[bank].gcl = NULL;
		}
		dev_warn(pdev, "EST: GCL -ENOMEM\n");

		return ret;
	}

	ti_wid = tsnif_est_get_ti_width(hw, ioaddr);
	cap->ti_wid = ti_wid;
	cap->gcl_depth = gcl_depth;
	cap->ext_max = EST_TIWID_TO_EXTMAX(ti_wid);
	cap->txqcnt = tsnif_est_get_txqcnt(hw, ioaddr);

	tils_max = (tsnif_has_tsn_cap(hw, ioaddr, TSN_FEAT_ID_EST) ? 3 : 0);
	tils_max = (1 << tils_max) - 1;
	cap->tils_max = tils_max;
	tsnif_est_get_max(hw, &cap->ptov_max, &cap->ctov_max,
			  &cap->cycle_max, &cap->idleslope_max);
	cap->est_support = 1;

	tsnif_tbs_get_max(hw, &cap->leos_max, &cap->legos_max);

	dev_info(pdev, "EST: depth=%u, ti_wid=%u, ter_max=%uns, tils_max=%u, tqcnt=%u\n",
		 gcl_depth, ti_wid, cap->ext_max, tils_max, cap->txqcnt);

	if (cap->tbs_support)
		dev_info(pdev, "TBS: leos_max=%u, legos_max=%u\n",
			 cap->leos_max, cap->legos_max);

	return 0;
}

int tsn_feat_set(struct mac_device_info *hw, struct net_device *dev,
		 enum tsn_feat_id featid, bool enable)
{
	if (featid >= TSN_FEAT_ID_MAX) {
		netdev_warn(dev, "TSN: invalid feature id(%u)\n", featid);
		return -EINVAL;
	}

	hw->tsn_info.feat_en[featid] = enable;

	return 0;
}

bool tsn_has_feat(struct mac_device_info *hw, struct net_device *dev,
		  enum tsn_feat_id featid)
{
	if (featid >= TSN_FEAT_ID_MAX) {
		netdev_warn(dev, "TSN: invalid feature id(%u)\n", featid);
		return -EINVAL;
	}

	return hw->tsn_info.feat_en[featid];
}

/* tsn_hw_setup is called within stmmac_hw_setup() after
 * stmmac_init_dma_engine() which resets MAC controller.
 * This is so-that MAC registers are not cleared.
 */
void tsn_hw_setup(struct mac_device_info *hw, struct net_device *dev)
{
	void __iomem *ioaddr = hw->pcsr;

	if (tsn_has_feat(hw, dev, TSN_FEAT_ID_EST))
		tsnif_hw_setup(hw, ioaddr, TSN_FEAT_ID_EST);
}

int tsn_hwtunable_set(struct mac_device_info *hw, struct net_device *dev,
		      enum tsn_hwtunable_id id,
		      const u32 data)
{
	struct tsnif_info *info = &hw->tsn_info;
	struct tsn_hw_cap *cap = &info->cap;
	void __iomem *ioaddr = hw->pcsr;
	int ret = 0;
	struct est_gc_bank *gcbc;
	u32 hw_bank;
	u32 estm;
	u32 leos;


	switch (id) {
	case TSN_HWTUNA_TX_EST_TILS:
	case TSN_HWTUNA_TX_EST_PTOV:
	case TSN_HWTUNA_TX_EST_CTOV:
		if (!tsn_has_feat(hw, dev, TSN_FEAT_ID_EST)) {
			netdev_info(dev, "EST: feature unsupported\n");
			return -ENOTSUPP;
		}
		break;
	case TSN_HWTUNA_TX_TBS_ESTM:
	case TSN_HWTUNA_TX_TBS_LEOS:
	case TSN_HWTUNA_TX_TBS_LEGOS:
		if (!tsn_has_feat(hw, dev, TSN_FEAT_ID_TBS)) {
			netdev_info(dev, "TBS: feature unsupported\n");
			return -ENOTSUPP;
		}
		break;
	default:
		netdev_warn(dev, "TSN: invalid tunable id(%u)\n", id);
		return -EINVAL;
	};

	switch (id) {
	case TSN_HWTUNA_TX_EST_TILS:
		if (data > cap->tils_max) {
			netdev_warn(dev, "EST: invalid tils(%u), max=%u\n",
				    data, cap->tils_max);

			return -EINVAL;
		}
		if (data != info->hwtunable[TSN_HWTUNA_TX_EST_TILS]) {
			tsnif_est_set_tils(hw, ioaddr, data);
			info->hwtunable[TSN_HWTUNA_TX_EST_TILS] = data;
			netdev_info(dev, "EST: Set TILS = %u\n", data);
		}
		break;
	case TSN_HWTUNA_TX_EST_PTOV:
		if (data > cap->ptov_max) {
			netdev_warn(dev,
				    "EST: invalid PTOV(%u), max=%u\n",
				    data, cap->ptov_max);

			return -EINVAL;
		}
		if (data != info->hwtunable[TSN_HWTUNA_TX_EST_PTOV]) {
			tsnif_est_set_ptov(hw, ioaddr, data);
			info->hwtunable[TSN_HWTUNA_TX_EST_PTOV] = data;
			netdev_info(dev, "EST: Set PTOV = %u\n", data);
		}
		break;
	case TSN_HWTUNA_TX_EST_CTOV:
		if (data > cap->ctov_max) {
			netdev_warn(dev,
				    "EST: invalid CTOV(%u), max=%u\n",
				    data, cap->ctov_max);

			return -EINVAL;
		}
		if (data != info->hwtunable[TSN_HWTUNA_TX_EST_CTOV]) {
			tsnif_est_set_ctov(hw, ioaddr, data);
			info->hwtunable[TSN_HWTUNA_TX_EST_CTOV] = data;
			netdev_info(dev, "EST: Set CTOV = %u\n", data);
		}
		break;
	case TSN_HWTUNA_TX_TBS_ESTM:
		if (!data && data != 1) {
			netdev_warn(dev,
				    "TBS: invalid ESTM(%u) - 0 or 1 only\n",
				    data);

			return -EINVAL;
		}

		if (data == 1 && !tsn_has_feat(hw, dev, TSN_FEAT_ID_EST)) {
			netdev_warn(dev,
				    "TBS: ESTM(%u) but EST is OFF\n",
				    data);

			return -EINVAL;
		}

		if (data != info->hwtunable[TSN_HWTUNA_TX_TBS_ESTM]) {
			tsnif_tbs_set_estm(hw, ioaddr, data);
			info->hwtunable[TSN_HWTUNA_TX_TBS_ESTM] = data;
			netdev_info(dev, "TBS: Set ESTM = %u\n", data);
		}
		break;
	case TSN_HWTUNA_TX_TBS_LEOS:
		estm = info->hwtunable[TSN_HWTUNA_TX_TBS_ESTM];

		if (data > cap->leos_max) {
			netdev_warn(dev,
				    "TBS: invalid LEOS(%u), max=%u\n",
				    data, cap->leos_max);

			return -EINVAL;
		}

		/* For EST mode, make sure leos does not exceed cycle time */
		if (estm) {
			hw_bank = tsnif_est_get_bank(hw, ioaddr, 0);
			gcbc = &info->est_gcc.gcb[hw_bank];

			if (data > (gcbc->gcrr.cycle_nsec - 1)) {
				netdev_warn(dev,
					    "TBS: LEOS > (cycle time - 1ns)\n");

				return -EINVAL;
			}
		}

		if (data != info->hwtunable[TSN_HWTUNA_TX_TBS_LEOS]) {
			tsnif_tbs_set_leos(hw, ioaddr, data, estm);
			info->hwtunable[TSN_HWTUNA_TX_TBS_LEOS] = data;
			netdev_info(dev, "TBS: Set LEOS = %u\n", data);
		}
		break;
	case TSN_HWTUNA_TX_TBS_LEGOS:
		estm = info->hwtunable[TSN_HWTUNA_TX_TBS_ESTM];
		leos = info->hwtunable[TSN_HWTUNA_TX_TBS_LEOS];

		/* if EST not turn on, ret fail */
		if (!(tsn_has_feat(hw, dev, TSN_FEAT_ID_EST) && estm)) {
			netdev_warn(dev, "TBS EST mode is not enabled\n");

			return -EINVAL;
		}

		if (data > cap->legos_max) {
			netdev_warn(dev,
				    "TBS: LEGOS(%u) > max=%u\n",
				    data, cap->legos_max);

			return -EINVAL;
		}

		if (data != info->hwtunable[TSN_HWTUNA_TX_TBS_LEGOS]) {
			tsnif_tbs_set_legos(hw, ioaddr, data, leos);
			info->hwtunable[TSN_HWTUNA_TX_TBS_LEGOS] = data;
			netdev_info(dev, "TBS: Set LEGOS = %u\n", data);
		}
		break;
	default:
		netdev_warn(dev, "TSN: invalid tunable id(%u)\n", id);
		ret = -EINVAL;
	};

	return ret;
}

int tsn_hwtunable_get(struct mac_device_info *hw, struct net_device *dev,
		      enum tsn_hwtunable_id id, u32 *data)
{
	struct tsnif_info *info = &hw->tsn_info;

	switch (id) {
	case TSN_HWTUNA_TX_EST_TILS:
	case TSN_HWTUNA_TX_EST_PTOV:
	case TSN_HWTUNA_TX_EST_CTOV:
		if (!tsn_has_feat(hw, dev, TSN_FEAT_ID_EST)) {
			netdev_info(dev, "EST: feature unsupported\n");
			return -ENOTSUPP;
		}
		break;
	case TSN_HWTUNA_TX_TBS_ESTM:
	case TSN_HWTUNA_TX_TBS_LEOS:
	case TSN_HWTUNA_TX_TBS_LEGOS:
		if (!tsn_has_feat(hw, dev, TSN_FEAT_ID_TBS)) {
			netdev_info(dev, "TBS: feature unsupported\n");
			return -ENOTSUPP;
		}
		break;
	default:
		netdev_warn(dev, "TSN: invalid tunable id(%u)\n", id);
		return -EINVAL;
	};

	*data = info->hwtunable[id];
	netdev_info(dev, "TSN: Get HW tunable[%d] = %u\n", id, *data);

	return 0;
}

int tsn_est_enable_set(struct mac_device_info *hw, struct net_device *dev,
		       bool enable)
{
	struct tsnif_info *info = &hw->tsn_info;
	void __iomem *ioaddr = hw->pcsr;

	if (!tsn_has_feat(hw, dev, TSN_FEAT_ID_EST)) {
		netdev_info(dev, "EST: feature unsupported\n");
		return -ENOTSUPP;
	}

	tsnif_est_set_enable(hw, ioaddr, enable);

	info->est_gcc.enable = enable;

	return 0;
}

int tsn_est_bank_get(struct mac_device_info *hw, struct net_device *dev,
		     bool is_own, u32 *bank)
{
	void __iomem *ioaddr = hw->pcsr;

	if (!tsn_has_feat(hw, dev, TSN_FEAT_ID_EST)) {
		netdev_info(dev, "EST: feature unsupported\n");
		return -ENOTSUPP;
	}

	*bank = tsnif_est_get_bank(hw, ioaddr, is_own);

	return 0;
}

int tsn_est_gce_set(struct mac_device_info *hw, struct net_device *dev,
		    struct est_gc_entry *gce, u32 row,
		    u32 dbgb, bool is_dbgm)
{
	struct tsnif_info *info = &hw->tsn_info;
	struct tsn_hw_cap *cap = &info->cap;
	void __iomem *ioaddr = hw->pcsr;
	u32 ti_nsec = gce->ti_nsec;
	u32 gates = gce->gates;
	struct est_gc_entry *gcl;
	u32 gates_mask;
	u32 ti_wid;
	u32 ti_max;
	u32 value;
	u32 bank;
	int ret;

	if (!tsn_has_feat(hw, dev, TSN_FEAT_ID_EST)) {
		netdev_info(dev, "EST: feature unsupported\n");
		return -ENOTSUPP;
	}

	if (dbgb >= EST_GCL_BANK_MAX)
		return -EINVAL;

	bank = is_dbgm ? dbgb : tsnif_est_get_bank(hw, ioaddr, 1);

	if (!cap->gcl_depth || row > cap->gcl_depth) {
		netdev_warn(dev, "EST: row(%u) > GCL depth(%u)\n",
			    row, cap->gcl_depth);

		return -EINVAL;
	}

	ti_wid = cap->ti_wid;
	ti_max = (1 << ti_wid) - 1;
	if (ti_nsec > ti_max) {
		netdev_warn(dev, "EST: ti_nsec(%u) > upper limit(%u)\n",
			    ti_nsec, ti_max);

		return -EINVAL;
	}

	gates_mask = (1 << cap->txqcnt) - 1;
	if (gates > gates_mask) {
		netdev_warn(dev, "EST: gates 0x%x is out of boundary 0x%x",
			    gates, gates_mask);

		return -EINVAL;
	}

	value = ((gates & gates_mask) << ti_wid) | ti_nsec;

	ret = tsnif_est_write_gcl_config(hw, ioaddr, value, row, I_GCE,
					 dbgb, is_dbgm);
	if (ret) {
		netdev_err(dev, "EST: GCE write failed: bank=%u row=%u.\n",
			   bank, row);

		return ret;
	}

	netdev_info(dev, "EST: GCE write: dbgm=%u bank=%u row=%u, gc=0x%x.\n",
		    is_dbgm, bank, row, value);

	/* Since GC write is successful, update GCL copy of the driver */
	gcl = info->est_gcc.gcb[bank].gcl + row;
	gcl->gates = gates;
	gcl->ti_nsec = ti_nsec;

	return ret;
}

int tsn_est_gcl_len_get(struct mac_device_info *hw, struct net_device *dev,
			u32 *gcl_len,
			u32 dbgb, bool is_dbgm)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value;
	u32 bank;
	int ret;

	if (!tsn_has_feat(hw, dev, TSN_FEAT_ID_EST)) {
		netdev_info(dev, "EST: feature unsupported\n");
		return -ENOTSUPP;
	}

	if (dbgb >= EST_GCL_BANK_MAX)
		return -EINVAL;

	bank = is_dbgm ? dbgb : tsnif_est_get_bank(hw, ioaddr, 1);

	ret = tsnif_est_read_gcl_config(hw, ioaddr, &value,
					GCL_PARAM_IDX_LLR, I_PRM,
					dbgb, is_dbgm);
	if (ret) {
		netdev_err(dev, "read LLR fail at bank=%u\n", bank);

		return ret;
	}

	*gcl_len = value;

	return 0;
}

int tsn_est_gcl_len_set(struct mac_device_info *hw, struct net_device *dev,
			u32 gcl_len,
			u32 dbgb, bool is_dbgm)
{
	struct tsnif_info *info = &hw->tsn_info;
	struct tsn_hw_cap *cap = &info->cap;
	void __iomem *ioaddr = hw->pcsr;
	int ret = 0;
	struct est_gcrr *bgcrr;
	u32 bank;

	if (!tsn_has_feat(hw, dev, TSN_FEAT_ID_EST)) {
		netdev_info(dev, "EST: feature unsupported\n");
		return -ENOTSUPP;
	}

	if (dbgb >= EST_GCL_BANK_MAX)
		return -EINVAL;

	bank = is_dbgm ? dbgb : tsnif_est_get_bank(hw, ioaddr, 1);

	if (gcl_len > cap->gcl_depth) {
		netdev_warn(dev, "EST: GCL length(%u) > depth(%u)\n",
			    gcl_len, cap->gcl_depth);

		return -EINVAL;
	}

	bgcrr = &info->est_gcc.gcb[bank].gcrr;

	if (gcl_len != bgcrr->llr) {
		ret = tsnif_est_write_gcl_config(hw, ioaddr, gcl_len,
						 GCL_PARAM_IDX_LLR, I_PRM,
						 dbgb, is_dbgm);
		if (ret) {
			netdev_err(dev, "EST: GCRR programming failure!\n");

			return ret;
		}
		bgcrr->llr = gcl_len;
	}

	return 0;
}

int tsn_est_gcrr_times_set(struct mac_device_info *hw,
			   struct net_device *dev,
			   struct est_gcrr *gcrr,
			   u32 dbgb, bool is_dbgm)
{
	struct tsnif_info *info = &hw->tsn_info;
	struct tsn_hw_cap *cap = &info->cap;
	u32 cycle_nsec = gcrr->cycle_nsec;
	u32 cycle_sec = gcrr->cycle_sec;
	u32 base_nsec = gcrr->base_nsec;
	void __iomem *ioaddr = hw->pcsr;
	u32 base_sec = gcrr->base_sec;
	u32 ext_nsec = gcrr->ter_nsec;
	int ret = 0;
	u64 val_ns, sys_ns, tti_ns;
	struct est_gcrr *bgcrr;
	u32 gcl_len, bank;

	if (!tsn_has_feat(hw, dev, TSN_FEAT_ID_EST)) {
		netdev_info(dev, "EST: feature unsupported\n");
		return -ENOTSUPP;
	}

	if (dbgb >= EST_GCL_BANK_MAX)
		return -EINVAL;

	bank = is_dbgm ? dbgb : tsnif_est_get_bank(hw, ioaddr, 1);

	if (base_nsec > NSEC_PER_SEC || cycle_nsec > NSEC_PER_SEC) {
		netdev_warn(dev, "EST: base(%u) or cycle(%u) nsec > 1s !\n",
			    base_nsec, cycle_nsec);

		return -EINVAL;
	}

	/* Ensure base time is later than MAC system time */
	val_ns = (u64)base_nsec;
	val_ns += (u64)(base_sec * NSEC_PER_SEC);

	/* Get the MAC system time */
	sys_ns = readl(ioaddr + PTP_STNSR);
	sys_ns += readl(ioaddr + PTP_STSR) * NSEC_PER_SEC;

	if (val_ns <= sys_ns) {
		netdev_warn(dev, "EST: base time(%llu) <= system time(%llu)\n",
			    val_ns, sys_ns);

		return -EINVAL;
	}

	if (cycle_sec > cap->cycle_max) {
		netdev_warn(dev, "EST: cycle time(%u) > %u seconds\n",
			    cycle_sec, cap->cycle_max);

		return -EINVAL;
	}

	if (ext_nsec > cap->ext_max) {
		netdev_warn(dev, "EST: invalid time extension(%u), max=%u\n",
			    ext_nsec, cap->ext_max);

		return -EINVAL;
	}

	bgcrr = &info->est_gcc.gcb[bank].gcrr;
	gcl_len = bgcrr->llr;

	/* Sanity test on GCL total time intervals against cycle time.
	 * a) For GC length = 1, if its time interval is equal or greater
	 *    than cycle time, it is a constant gate error.
	 * b) If total time interval > cycle time, irregardless of GC
	 *    length, it is not considered an error that GC list is
	 *    truncated. In this case, giving a warning message is
	 *    sufficient.
	 * c) If total time interval < cycle time, irregardless of GC
	 *    length, all GATES are OPEN after the last GC is processed
	 *    until cycle time lapses. This is potentially due to poor
	 *    GCL configuration but is not an error, so we inform user
	 *    about it.
	 */
	tti_ns = (u64)est_get_gcl_total_intervals_nsec(&info->est_gcc, bank,
						       gcl_len);
	val_ns = (u64)cycle_nsec;
	val_ns += (u64)(cycle_sec * NSEC_PER_SEC);
	if (gcl_len == 1 && tti_ns >= val_ns) {
		netdev_warn(dev, "EST: Constant gate error!\n");

		return -EINVAL;
	}

	if (tti_ns > val_ns)
		netdev_warn(dev, "EST: GCL is truncated!\n");

	if (tti_ns < val_ns) {
		netdev_info(dev,
			    "EST: All GCs OPEN at %llu-ns of %llu-ns cycle\n",
			    tti_ns, val_ns);
	}

	/* Finally, start programming GCL related registers if the value
	 * differs from the driver copy for efficiency.
	 */

	if (base_nsec != bgcrr->base_nsec)
		ret |= tsnif_est_write_gcl_config(hw, ioaddr, base_nsec,
						  GCL_PARAM_IDX_BTR_LO, I_PRM,
						  dbgb, is_dbgm);

	if (base_sec != bgcrr->base_sec)
		ret |= tsnif_est_write_gcl_config(hw, ioaddr, base_sec,
						  GCL_PARAM_IDX_BTR_HI, I_PRM,
						  dbgb, is_dbgm);

	if (cycle_nsec != bgcrr->cycle_nsec)
		ret |= tsnif_est_write_gcl_config(hw, ioaddr, cycle_nsec,
						  GCL_PARAM_IDX_CTR_LO, I_PRM,
						  dbgb, is_dbgm);

	if (cycle_sec != bgcrr->cycle_sec)
		ret |= tsnif_est_write_gcl_config(hw, ioaddr, cycle_sec,
						  GCL_PARAM_IDX_CTR_HI, I_PRM,
						  dbgb, is_dbgm);

	if (ext_nsec != bgcrr->ter_nsec)
		ret |= tsnif_est_write_gcl_config(hw, ioaddr, ext_nsec,
						  GCL_PARAM_IDX_TER, I_PRM,
						  dbgb, is_dbgm);

	if (ret) {
		netdev_err(dev, "EST: GCRR programming failure!\n");

		return ret;
	}

	/* Finally, we are ready to switch SWOL now. */
	tsnif_est_switch_swol(hw, ioaddr);

	/* Update driver copy */
	bgcrr->base_sec = base_sec;
	bgcrr->base_nsec = base_nsec;
	bgcrr->cycle_sec = cycle_sec;
	bgcrr->cycle_nsec = cycle_nsec;
	bgcrr->ter_nsec = ext_nsec;

	netdev_info(dev, "EST: gcrr set successful\n");

	return 0;
}

int tsn_est_gcc_get(struct mac_device_info *hw, struct net_device *dev,
		    struct est_gc_config **gcc)
{
	struct tsnif_info *info = &hw->tsn_info;
	void __iomem *ioaddr = hw->pcsr;
	struct est_gc_config *pgcc;
	u32 ti_wid;
	u32 txqcnt;
	u32 value;
	u32 bank;
	int ret;

	if (!tsn_has_feat(hw, dev, TSN_FEAT_ID_EST)) {
		netdev_info(dev, "EST: feature unsupported\n");
		return -ENOTSUPP;
	}

	/* Get GC config from HW */
	pgcc = &info->est_gcc;
	pgcc->enable = tsnif_est_get_enable(hw, ioaddr);

	for (bank = 0; bank < EST_GCL_BANK_MAX; bank++) {
		u32 llr, row;
		struct est_gc_bank *gcbc = &pgcc->gcb[bank];

		ret = tsnif_est_read_gcl_config(hw, ioaddr, &value,
						GCL_PARAM_IDX_BTR_LO, I_PRM,
						bank, 1);
		if (ret) {
			netdev_err(dev, "read BTR(low) fail at bank=%u\n",
				   bank);

			return ret;
		}
		gcbc->gcrr.base_nsec = value;

		ret = tsnif_est_read_gcl_config(hw, ioaddr, &value,
						GCL_PARAM_IDX_BTR_HI, I_PRM,
						bank, 1);
		if (ret) {
			netdev_err(dev, "read BTR(high) fail at bank=%u\n",
				   bank);

			return ret;
		}
		gcbc->gcrr.base_sec = value;

		ret = tsnif_est_read_gcl_config(hw, ioaddr, &value,
						GCL_PARAM_IDX_CTR_LO, I_PRM,
						bank, 1);
		if (ret) {
			netdev_err(dev, "read CTR(low) fail at bank=%u\n",
				   bank);

			return ret;
		}
		gcbc->gcrr.cycle_nsec = value;

		ret = tsnif_est_read_gcl_config(hw, ioaddr, &value,
						GCL_PARAM_IDX_CTR_HI, I_PRM,
						bank, 1);
		if (ret) {
			netdev_err(dev, "read CTR(high) fail at bank=%u\n",
				   bank);

			return ret;
		}
		gcbc->gcrr.cycle_sec = value;

		ret = tsnif_est_read_gcl_config(hw, ioaddr, &value,
						GCL_PARAM_IDX_TER, I_PRM,
						bank, 1);
		if (ret) {
			netdev_err(dev, "read TER fail at bank=%u\n", bank);

			return ret;
		}
		gcbc->gcrr.ter_nsec = value;

		ret = tsnif_est_read_gcl_config(hw, ioaddr, &value,
						GCL_PARAM_IDX_LLR, I_PRM,
						bank, 1);
		if (ret) {
			netdev_err(dev, "read LLR fail at bank=%u\n", bank);

			return ret;
		}
		gcbc->gcrr.llr = value;
		llr = value;

		ti_wid = info->cap.ti_wid;
		txqcnt = info->cap.txqcnt;
		for (row = 0; row < llr; row++) {
			struct est_gc_entry *gce = gcbc->gcl + row;
			u32 gates, ti_nsec;

			ret = tsnif_est_read_gce(hw, ioaddr, row,
						 &gates, &ti_nsec,
						 ti_wid, txqcnt, bank, 1);
			if (ret) {
				netdev_err(dev,
					   "read GCE fail at bank=%u row=%u\n",
					   bank, row);

				return ret;
			}
			gce->gates = gates;
			gce->ti_nsec = ti_nsec;
		}
	}

	*gcc = pgcc;
	netdev_info(dev, "EST: read GCL from HW done.\n");

	return 0;
}

void tsn_est_irq_status(struct mac_device_info *hw, struct net_device *dev)
{
	struct tsnif_info *info = &hw->tsn_info;
	void __iomem *ioaddr = hw->pcsr;
	unsigned int status;

	status = tsnif_est_irq_status(hw, ioaddr, dev, &info->mmc_stat,
				      info->cap.txqcnt);
}

int tsn_mmc_dump(struct mac_device_info *hw,
		 int index, unsigned long *count, const char **desc)
{
	struct tsnif_info *info = &hw->tsn_info;
	const struct tsn_mmc_desc *mmc_desc;
	unsigned long *ptr;

	ptr = (unsigned long *)&info->mmc_stat;
	mmc_desc = info->mmc_desc;

	if (!(mmc_desc + index)->valid)
		return -EINVAL;
	if (count)
		*count = *(ptr + index);
	if (desc)
		*desc = (mmc_desc + index)->desc;
	return 0;
}

int tsn_cbs_recal_idleslope(struct mac_device_info *hw, struct net_device *dev,
			    u32 queue, u32 *idle_slope)
{
	struct tsnif_info *info = &hw->tsn_info;
	void __iomem *ioaddr = hw->pcsr;
	u64 scaling = 0;
	struct est_gc_config *est_gcc;
	struct tsn_hw_cap *cap;
	u64 new_idle_slope;
	u64 cycle_time_ns;
	u32 open_time;
	u32 hw_bank;

	cap = &info->cap;
	est_gcc = &info->est_gcc;
	hw_bank = tsnif_est_get_bank(hw, ioaddr, 1);

	cycle_time_ns = (est_gcc->gcb[hw_bank].gcrr.cycle_sec *
			 NSEC_PER_SEC) +
			 est_gcc->gcb[hw_bank].gcrr.cycle_nsec;

	if (!cycle_time_ns) {
		netdev_warn(dev, "EST: Cycle time is 0.\n");
		netdev_warn(dev, "CBS idle slope will not be reconfigured.\n");

		return -EINVAL;
	}

	open_time = est_get_all_open_time(est_gcc, hw_bank,
					  cycle_time_ns, queue);

	if (!open_time) {
		netdev_warn(dev, "EST: Total gate open time for queue %d is 0\n",
			    queue);

		return -EINVAL;
	}

	scaling = cycle_time_ns;
	do_div(scaling, open_time);

	new_idle_slope = *idle_slope * scaling;
	if (new_idle_slope > cap->idleslope_max)
		new_idle_slope = cap->idleslope_max;

	*idle_slope = new_idle_slope;

	return 0;
}
