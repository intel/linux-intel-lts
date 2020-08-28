// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/* Copyright (c) 2019, Intel Corporation.
 * TSN General APIs
 */
#include <linux/iopoll.h>
#include <linux/delay.h>
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

static void fpe_lp_task(struct work_struct *work)
{
	struct mac_device_info *hw;
	enum fpe_state *lo_state;
	enum fpe_state *lp_state;
	struct tsnif_info *info;
	void __iomem *ioaddr;
	bool *enable;
	int retries;

	info = container_of(work, struct tsnif_info, fpe_task);
	lo_state = &info->fpe_cfg.lo_fpe_state;
	lp_state = &info->fpe_cfg.lp_fpe_state;
	enable = &info->fpe_cfg.enable;

	hw = container_of(info, struct mac_device_info, tsn_info);
	ioaddr = hw->pcsr;

	retries = 20;

	while (retries-- > 0) {
		/* Bail out immediately if FPE is OFF */
		if (*lo_state == FPE_STATE_OFF || !*enable)
			break;

		if (*lo_state == FPE_STATE_ENTERING_ON &&
		    *lp_state == FPE_STATE_ENTERING_ON) {
			tsnif_fpe_set_enable(hw, ioaddr, true);
			*lo_state = FPE_STATE_ON;
			*lp_state = FPE_STATE_ON;
			printk("!!! BOTH FPE stations ON\n");
			break;
		}

		if ((*lo_state == FPE_STATE_CAPABLE ||
		     *lo_state == FPE_STATE_ENTERING_ON) &&
		    *lp_state != FPE_STATE_ON) {
			printk("Send Verify mPacket lo_state=%d lp_state=%d\n", *lo_state, *lp_state);
			tsnif_fpe_send_mpacket(hw, ioaddr, MPACKET_VERIFY);
		}
		/* Sleep then retry */
		msleep(500);
	}

	clear_bit(__FPE_TASK_SCHED, &info->task_state);
}

static int fpe_start_wq(struct mac_device_info *hw, struct net_device *dev)
{
	struct tsnif_info *info = &hw->tsn_info;
	char *name;

	clear_bit(__FPE_TASK_SCHED, &info->task_state);

	name = info->wq_name;
	sprintf(name, "%s-fpe", dev->name);

	info->fpe_wq = create_singlethread_workqueue(name);
	if (!info->fpe_wq) {
		netdev_err(dev, "%s: Failed to create workqueue\n", name);

		return -ENOMEM;
	}
	netdev_info(dev, "FPE workqueue start");

	return 0;
}

static void fpe_stop_wq(struct mac_device_info *hw, struct net_device *dev)
{
	struct tsnif_info *info = &hw->tsn_info;

	set_bit(__FPE_REMOVING, &info->task_state);

	if (info->fpe_wq)
		destroy_workqueue(info->fpe_wq);

	netdev_info(dev, "FPE workqueue stop");
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
		goto check_fpe;
	}

	gcl_depth = tsnif_est_get_gcl_depth(hw, ioaddr);
	if (gcl_depth == 0) {
		dev_err(pdev, "EST GCL depth(%d) = 0\n", gcl_depth);
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

	dev_info(pdev, "EST: depth=%u, ti_wid=%u, ter_max=%uns, tils_max=%u, tqcnt=%u\n",
		 gcl_depth, ti_wid, cap->ext_max, tils_max, cap->txqcnt);

check_fpe:
	if (!tsnif_has_tsn_cap(hw, ioaddr, TSN_FEAT_ID_FPE)) {
		dev_info(pdev, "FPE NOT supported\n");
		cap->fpe_support = 0;
		goto check_tbs;
	}

	INIT_WORK(&info->fpe_task, fpe_lp_task);
	tsnif_fpe_get_info(hw, &cap->pmac_bit, &cap->afsz_max,
			   &cap->hadv_max, &cap->radv_max);
	cap->rxqcnt = tsnif_est_get_rxqcnt(hw, ioaddr);
	cap->fpe_support = 1;

	dev_info(pdev, "FPE: pMAC Bit=0x%x\n afsz_max=%d", cap->pmac_bit,
		 cap->afsz_max);
	dev_info(pdev, "FPE: hadv_max=%d radv_max=%d", cap->hadv_max,
		 cap->radv_max);

check_tbs:
	if (!tsnif_has_tsn_cap(hw, ioaddr, TSN_FEAT_ID_TBS)) {
		dev_info(pdev, "TBS NOT supported\n");
		cap->tbs_support = 0;
		goto scan_done;
	} else {
		dev_info(pdev, "TBS capable\n");
		cap->tbs_support = 1;
	}

	tsnif_tbs_get_max(hw, &cap->leos_max, &cap->legos_max,
			  &cap->ftos_max, &cap->fgos_max);

	dev_info(pdev, "TBS: leos_max=%u, legos_max=%u\n",
		 cap->leos_max, cap->legos_max);
	dev_info(pdev, "TBS: ftos_max=%u, fgos_max=%u\n",
		 cap->ftos_max, cap->fgos_max);

scan_done:

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
void tsn_hw_setup(struct mac_device_info *hw, struct net_device *dev,
		  u32 fprq)
{
	struct tsnif_info *info = &hw->tsn_info;
	struct tsn_hw_cap *cap = &info->cap;
	void __iomem *ioaddr = hw->pcsr;

	if (tsn_has_feat(hw, dev, TSN_FEAT_ID_EST))
		tsnif_hw_setup(hw, ioaddr, TSN_FEAT_ID_EST, 0);

	if (tsn_has_feat(hw, dev, TSN_FEAT_ID_FPE)) {
		/* RxQ0 default to Express Frame, FPRQ != RxQ0 */
		if (fprq > 0 && fprq < cap->rxqcnt) {
			netdev_info(dev, "FPE: Set FPRQ = %d\n", fprq);
			tsnif_hw_setup(hw, ioaddr, TSN_FEAT_ID_FPE, fprq);
		} else {
			netdev_warn(dev, "FPE: FPRQ is out-of-bound.\n");
		}

		fpe_start_wq(hw, dev);
	}
}

void tsn_hw_unsetup(struct mac_device_info *hw, struct net_device *dev)
{
	if (tsn_has_feat(hw, dev, TSN_FEAT_ID_FPE))
		fpe_stop_wq(hw, dev);
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
	u32 ftos;
	u32 fgos;

	switch (id) {
	case TSN_HWTUNA_TX_EST_TILS:
	case TSN_HWTUNA_TX_EST_PTOV:
	case TSN_HWTUNA_TX_EST_CTOV:
		if (!tsn_has_feat(hw, dev, TSN_FEAT_ID_EST)) {
			netdev_info(dev, "EST: feature unsupported\n");
			return -ENOTSUPP;
		}
		break;
	case TSN_HWTUNA_TX_FPE_AFSZ:
	case TSN_HWTUNA_TX_FPE_HADV:
	case TSN_HWTUNA_TX_FPE_RADV:
		if (!tsn_has_feat(hw, dev, TSN_FEAT_ID_FPE)) {
			netdev_info(dev, "FPE: feature unsupported\n");
			return -ENOTSUPP;
		}
		break;
	case TSN_HWTUNA_TX_TBS_ESTM:
	case TSN_HWTUNA_TX_TBS_LEOS:
	case TSN_HWTUNA_TX_TBS_LEGOS:
	case TSN_HWTUNA_TX_TBS_FTOS:
	case TSN_HWTUNA_TX_TBS_FGOS:
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
	case TSN_HWTUNA_TX_FPE_AFSZ:
		if (data > cap->afsz_max) {
			netdev_warn(dev,
				    "EST: invalid AFSZ(%u), max=%u\n",
				    data, cap->afsz_max);

			return -EINVAL;
		}
		if (data != info->hwtunable[TSN_HWTUNA_TX_FPE_AFSZ]) {
			tsnif_fpe_set_afsz(hw, ioaddr, data);
			info->hwtunable[TSN_HWTUNA_TX_FPE_AFSZ] = data;
			netdev_info(dev, "FPE: Set AFSZ = %u\n", data);
		}
		break;
	case TSN_HWTUNA_TX_FPE_HADV:
		if (data > cap->hadv_max) {
			netdev_warn(dev,
				    "EST: invalid HADV(%u), max=%u\n",
				    data, cap->hadv_max);

			return -EINVAL;
		}
		if (data != info->hwtunable[TSN_HWTUNA_TX_FPE_HADV]) {
			tsnif_fpe_set_hadv(hw, ioaddr, data);
			info->hwtunable[TSN_HWTUNA_TX_FPE_HADV] = data;
			netdev_info(dev, "FPE: Set HADV = %u\n", data);
		}
		break;
	case TSN_HWTUNA_TX_FPE_RADV:
		if (data > cap->radv_max) {
			netdev_warn(dev,
				    "EST: invalid RADV(%u), max=%u\n",
				    data, cap->radv_max);

			return -EINVAL;
		}
		if (data != info->hwtunable[TSN_HWTUNA_TX_FPE_RADV]) {
			tsnif_fpe_set_radv(hw, ioaddr, data);
			info->hwtunable[TSN_HWTUNA_TX_FPE_RADV] = data;
			netdev_info(dev, "FPE: Set RADV = %u\n", data);
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
	case TSN_HWTUNA_TX_TBS_FTOS:
		estm = info->hwtunable[TSN_HWTUNA_TX_TBS_ESTM];
		fgos = info->hwtunable[TSN_HWTUNA_TX_TBS_FGOS];

		if (data > cap->ftos_max) {
			netdev_warn(dev,
				    "TBS: invalid FTOS(%u), max=%u\n",
				    data, cap->ftos_max);

			return -EINVAL;
		}

		/* For EST mode, make sure leos does not exceed cycle time */
		if (estm) {
			hw_bank = tsnif_est_get_bank(hw, ioaddr, 0);
			gcbc = &info->est_gcc.gcb[hw_bank];

			if (data > (gcbc->gcrr.cycle_nsec - 1)) {
				netdev_warn(dev,
					    "TBS: FTOS > (cycle time - 1ns)\n");

				return -EINVAL;
			}
		}

		if (data != info->hwtunable[TSN_HWTUNA_TX_TBS_FTOS]) {
			tsnif_tbs_set_ftos(hw, ioaddr, data, estm, fgos);
			info->hwtunable[TSN_HWTUNA_TX_TBS_FTOS] = data;
			netdev_info(dev, "TBS: Set FTOS = %u\n", data);
		}
		break;
	case TSN_HWTUNA_TX_TBS_FGOS:
		estm = info->hwtunable[TSN_HWTUNA_TX_TBS_ESTM];
		ftos = info->hwtunable[TSN_HWTUNA_TX_TBS_FTOS];

		/* if EST not turn on, ret fail */
		if (!(tsn_has_feat(hw, dev, TSN_FEAT_ID_EST) && estm)) {
			netdev_warn(dev, "TBS EST mode is not enabled\n");

			return -EINVAL;
		}

		if (data > cap->fgos_max) {
			netdev_warn(dev,
				    "TBS: invalid FGOS(%u), max=%u\n",
				    data, cap->fgos_max);

			return -EINVAL;
		}

		if (data != info->hwtunable[TSN_HWTUNA_TX_TBS_FGOS]) {
			tsnif_tbs_set_fgos(hw, ioaddr, data, ftos);
			info->hwtunable[TSN_HWTUNA_TX_TBS_FGOS] = data;
			netdev_info(dev, "TBS: Set FGOS = %u\n", data);
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
	case TSN_HWTUNA_TX_FPE_AFSZ:
	case TSN_HWTUNA_TX_FPE_HADV:
	case TSN_HWTUNA_TX_FPE_RADV:
		if (!tsn_has_feat(hw, dev, TSN_FEAT_ID_FPE)) {
			netdev_info(dev, "FPE: feature unsupported\n");
			return -ENOTSUPP;
		}
		break;
	case TSN_HWTUNA_TX_TBS_ESTM:
	case TSN_HWTUNA_TX_TBS_LEOS:
	case TSN_HWTUNA_TX_TBS_LEGOS:
	case TSN_HWTUNA_TX_TBS_FTOS:
	case TSN_HWTUNA_TX_TBS_FGOS:
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

void tsn_mmc_stat_update(struct mac_device_info *hw, struct net_device *dev)
{
	struct tsnif_info *info = &hw->tsn_info;
	void __iomem *ioaddr = hw->pcsr;

	if (tsn_has_feat(hw, dev, TSN_FEAT_ID_FPE)) {
		tsnif_fpe_update_mmc_stat(hw, ioaddr, &info->mmc_stat);
	}
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
	hw_bank = tsnif_est_get_bank(hw, ioaddr, 0);

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

int tsn_fpe_set_txqpec(struct mac_device_info *hw, struct net_device *dev,
		       u32 txqpec)
{
	struct tsnif_info *info = &hw->tsn_info;
	struct tsn_hw_cap *cap = &info->cap;
	void __iomem *ioaddr = hw->pcsr;
	u32 txqmask;

	if (!tsn_has_feat(hw, dev, TSN_FEAT_ID_FPE)) {
		netdev_info(dev, "FPE: feature unsupported\n");
		return -ENOTSUPP;
	}

	/* Check PEC is within TxQ range */
	txqmask = (1 << cap->txqcnt) - 1;
	if (txqpec & ~txqmask) {
		netdev_warn(dev, "FPE: Tx PEC is out-of-bound.\n");

		return -EINVAL;
	}

	/* When EST and FPE are both enabled, TxQ0 is always preemptible
	 * queue. If FPE is enabled, we expect at least lsb is set.
	 * If FPE is not enabled, we should allow PEC = 0.
	 */
	if (txqpec && !(txqpec & cap->pmac_bit) && info->est_gcc.enable) {
		netdev_warn(dev, "FPE: TxQ0 must not be express queue.\n");

		return -EINVAL;
	}

	tsnif_fpe_set_txqpec(hw, ioaddr, txqpec, txqmask);
	info->fpe_cfg.txqpec = txqpec;
	netdev_info(dev, "FPE: TxQ PEC = 0x%x\n", txqpec);

	return 0;
}

int tsn_fpe_set_enable(struct mac_device_info *hw, struct net_device *dev,
		       bool enable)
{
	struct tsnif_info *info = &hw->tsn_info;
	void __iomem *ioaddr = hw->pcsr;

	if (!tsn_has_feat(hw, dev, TSN_FEAT_ID_FPE)) {
		netdev_info(dev, "FPE: feature unsupported\n");
		return -ENOTSUPP;
	}

	if (info->fpe_cfg.enable != enable) {
		if (enable)
			tsnif_fpe_send_mpacket(hw, ioaddr, MPACKET_VERIFY);
		else
			info->fpe_cfg.lo_fpe_state = FPE_STATE_OFF;

		info->fpe_cfg.enable = enable;
	}

	return 0;
}

int tsn_fpe_get_config(struct mac_device_info *hw, struct net_device *dev,
		       u32 *txqpec, bool *enable)
{
	void __iomem *ioaddr = hw->pcsr;

	if (!tsn_has_feat(hw, dev, TSN_FEAT_ID_FPE)) {
		netdev_info(dev, "FPE: feature unsupported\n");
		return -ENOTSUPP;
	}

	tsnif_fpe_get_config(hw, ioaddr, txqpec, enable);

	return 0;
}

int tsn_fpe_show_pmac_sts(struct mac_device_info *hw, struct net_device *dev)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 hrs = 0;

	if (!tsn_has_feat(hw, dev, TSN_FEAT_ID_FPE)) {
		netdev_info(dev, "FPE: feature unsupported\n");
		return -ENOTSUPP;
	}

	tsnif_fpe_get_pmac_sts(hw, ioaddr, &hrs);

	if (hrs)
		netdev_info(dev, "FPE: pMAC is in Hold state.\n");
	else
		netdev_info(dev, "FPE: pMAC is in Release state.\n");

	return 0;
}

int tsn_fpe_send_mpacket(struct mac_device_info *hw, struct net_device *dev,
			 enum mpacket_type type)
{
	void __iomem *ioaddr = hw->pcsr;

	if (!tsn_has_feat(hw, dev, TSN_FEAT_ID_FPE)) {
		netdev_info(dev, "FPE: feature unsupported\n");
		return -ENOTSUPP;
	}

	tsnif_fpe_send_mpacket(hw, ioaddr, type);

	return 0;
}

void tsn_fpe_link_state_handle(struct mac_device_info *hw,
			       struct net_device *dev, bool is_up)
{
	struct tsnif_info *info = &hw->tsn_info;
	void __iomem *ioaddr = hw->pcsr;
	enum fpe_state *lo_state;
	enum fpe_state *lp_state;
	bool *enable;

	lo_state = &info->fpe_cfg.lo_fpe_state;
	lp_state = &info->fpe_cfg.lp_fpe_state;
	enable = &info->fpe_cfg.enable;

	if (is_up && *enable) {
		tsnif_fpe_send_mpacket(hw, ioaddr, MPACKET_VERIFY);
	} else {
		*lo_state = FPE_EVENT_UNKNOWN;
		*lp_state = FPE_EVENT_UNKNOWN;
	}
}

void tsn_fpe_irq_status(struct mac_device_info *hw, struct net_device *dev)
{
	struct tsnif_info *info = &hw->tsn_info;
	void __iomem *ioaddr = hw->pcsr;
	enum fpe_event *event;
	enum fpe_state *lo_state;
	enum fpe_state *lp_state;
	bool *enable;

	event = &info->fpe_cfg.fpe_event;
	lo_state = &info->fpe_cfg.lo_fpe_state;
	lp_state = &info->fpe_cfg.lp_fpe_state;
	enable = &info->fpe_cfg.enable;

	tsnif_fpe_mmc_irq_status(hw, ioaddr, dev);
	tsnif_fpe_irq_status(hw, ioaddr, dev, event);

	if (*event == FPE_EVENT_UNKNOWN || !*enable)
		return;

	/* If LP has sent verify mPacket, LP is FPE capable */
	if ((*event & FPE_EVENT_RVER) == FPE_EVENT_RVER) {
		if (*lp_state < FPE_STATE_CAPABLE)
			*lp_state = FPE_STATE_CAPABLE;

		/* If user has requested FPE enable, quickly response */
		if (*enable)
			tsnif_fpe_send_mpacket(hw, ioaddr, MPACKET_RESPONSE);
	}

	/* If Local has sent verify mPacket, Local is FPE capable */
	if ((*event & FPE_EVENT_TVER) == FPE_EVENT_TVER) {
		if (*lo_state < FPE_STATE_CAPABLE)
			*lo_state = FPE_STATE_CAPABLE;
	}

	/* If LP has sent response mPacket, LP is entering FPE ON */
	if ((*event & FPE_EVENT_RRSP) == FPE_EVENT_RRSP)
		*lp_state = FPE_STATE_ENTERING_ON;

	/* If Local has sent response mPacket, Local is entering FPE ON */
	if ((*event & FPE_EVENT_TRSP) == FPE_EVENT_TRSP)
		*lo_state = FPE_STATE_ENTERING_ON;

	if (!test_bit(__FPE_REMOVING, &info->task_state) &&
	    !test_and_set_bit(__FPE_TASK_SCHED, &info->task_state) &&
	    info->fpe_wq)
		queue_work(info->fpe_wq, &info->fpe_task);
}
