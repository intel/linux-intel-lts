// SPDX-License-Identifier: GPL-2.0
/* Copyright (c)  2019 Intel Corporation */

#include "igc.h"

#include <linux/module.h>
#include <linux/device.h>
#include <linux/pci.h>
#include <linux/ptp_classify.h>
#include <linux/clocksource.h>
#include <linux/ktime.h>

#define INCVALUE_MASK		0x7fffffff
#define ISGN			0x80000000

#define IGC_SYSTIM_OVERFLOW_PERIOD	(HZ * 60 * 9)
#define IGC_PTP_TX_TIMEOUT		(HZ * 15)

#define IGC_PTM_CYCLE_TIME_MSECS 50

/* SYSTIM read access for I225 */
void igc_ptp_read(struct igc_adapter *adapter, struct timespec64 *ts)
{
	struct igc_hw *hw = &adapter->hw;
	u32 sec, nsec;

	/* The timestamp is latched when SYSTIML is read. */
	nsec = rd32(IGC_SYSTIML);
	sec = rd32(IGC_SYSTIMH);

	ts->tv_sec = sec;
	ts->tv_nsec = nsec;
}

static void igc_ptp_write_i225(struct igc_adapter *adapter,
			       const struct timespec64 *ts)
{
	struct igc_hw *hw = &adapter->hw;

	wr32(IGC_SYSTIML, ts->tv_nsec);
	wr32(IGC_SYSTIMH, ts->tv_sec);
}

static int igc_ptp_adjfine_i225(struct ptp_clock_info *ptp, long scaled_ppm)
{
	struct igc_adapter *igc = container_of(ptp, struct igc_adapter,
					       ptp_caps);
	struct igc_hw *hw = &igc->hw;
	int neg_adj = 0;
	u64 rate;
	u32 inca;

	if (scaled_ppm < 0) {
		neg_adj = 1;
		scaled_ppm = -scaled_ppm;
	}
	rate = scaled_ppm;
	rate <<= 14;
	rate = div_u64(rate, 78125);

	inca = rate & INCVALUE_MASK;
	if (neg_adj)
		inca |= ISGN;

	wr32(IGC_TIMINCA, inca);

	return 0;
}

static int igc_ptp_adjtime_i225(struct ptp_clock_info *ptp, s64 delta)
{
	struct igc_adapter *igc = container_of(ptp, struct igc_adapter,
					       ptp_caps);
	struct timespec64 now, then = ns_to_timespec64(delta);
	unsigned long flags;

	spin_lock_irqsave(&igc->tmreg_lock, flags);

	igc_ptp_read(igc, &now);
	now = timespec64_add(now, then);
	igc_ptp_write_i225(igc, (const struct timespec64 *)&now);

	spin_unlock_irqrestore(&igc->tmreg_lock, flags);

	return 0;
}

static int igc_ptp_gettimex64_i225(struct ptp_clock_info *ptp,
				   struct timespec64 *ts,
				   struct ptp_system_timestamp *sts)
{
	struct igc_adapter *igc = container_of(ptp, struct igc_adapter,
					       ptp_caps);
	struct igc_hw *hw = &igc->hw;
	unsigned long flags;

	spin_lock_irqsave(&igc->tmreg_lock, flags);

	ptp_read_system_prets(sts);
	ts->tv_nsec = rd32(IGC_SYSTIML);
	ts->tv_sec = rd32(IGC_SYSTIMH);
	ptp_read_system_postts(sts);

	spin_unlock_irqrestore(&igc->tmreg_lock, flags);

	return 0;
}

static int igc_ptp_settime_i225(struct ptp_clock_info *ptp,
				const struct timespec64 *ts)
{
	struct igc_adapter *igc = container_of(ptp, struct igc_adapter,
					       ptp_caps);
	unsigned long flags;

	spin_lock_irqsave(&igc->tmreg_lock, flags);

	igc_ptp_write_i225(igc, ts);

	spin_unlock_irqrestore(&igc->tmreg_lock, flags);

	return 0;
}

static int igc_ptp_feature_enable_i225(struct ptp_clock_info *ptp,
				       struct ptp_clock_request *rq, int on)
{
	struct igc_adapter *igc =
		container_of(ptp, struct igc_adapter, ptp_caps);
	struct igc_hw *hw = &igc->hw;
	unsigned long flags;
	u32 tsim;

	switch (rq->type) {
	case PTP_CLK_REQ_PPS:
		spin_lock_irqsave(&igc->tmreg_lock, flags);
		tsim = rd32(IGC_TSIM);
		if (on)
			tsim |= IGC_TSICR_SYS_WRAP;
		else
			tsim &= ~IGC_TSICR_SYS_WRAP;
		igc->pps_sys_wrap_on = on;
		wr32(IGC_TSIM, tsim);
		spin_unlock_irqrestore(&igc->tmreg_lock, flags);
		return 0;

	default:
		break;
	}

	return -EOPNOTSUPP;
}

/**
 * igc_ptp_systim_to_hwtstamp - convert system time value to HW timestamp
 * @adapter: board private structure
 * @hwtstamps: timestamp structure to update
 * @systim: unsigned 64bit system time value
 *
 * We need to convert the system time value stored in the RX/TXSTMP registers
 * into a hwtstamp which can be used by the upper level timestamping functions.
 **/
static void igc_ptp_systim_to_hwtstamp(struct igc_adapter *adapter,
				       struct skb_shared_hwtstamps *hwtstamps,
				       u64 systim)
{
	switch (adapter->hw.mac.type) {
	case igc_i225:
		memset(hwtstamps, 0, sizeof(*hwtstamps));
		/* Upper 32 bits contain s, lower 32 bits contain ns. */
		hwtstamps->hwtstamp = ktime_set(systim >> 32,
						systim & 0xFFFFFFFF);
		break;
	default:
		break;
	}
}

/**
 * igc_ptp_rx_pktstamp - retrieve Rx per packet timestamp
 * @q_vector: Pointer to interrupt specific structure
 * @va: Pointer to address containing Rx buffer
 * @skb: Buffer containing timestamp and packet
 *
 * This function is meant to retrieve the first timestamp from the
 * first buffer of an incoming frame. The value is stored in little
 * endian format starting on byte 0. There's a second timestamp
 * starting on byte 8.
 **/
void igc_ptp_rx_pktstamp(struct igc_q_vector *q_vector, void *va,
			 struct sk_buff *skb)
{
	struct igc_adapter *adapter = q_vector->adapter;
	__le64 *regval = (__le64 *)va;
	int adjust = 0;

	/* The timestamp is recorded in little endian format.
	 * DWORD: | 0          | 1           | 2          | 3
	 * Field: | Timer0 Low | Timer0 High | Timer1 Low | Timer1 High
	 */
	igc_ptp_systim_to_hwtstamp(adapter, skb_hwtstamps(skb),
				   le64_to_cpu(regval[0]));

	/* adjust timestamp for the RX latency based on link speed */
	if (adapter->hw.mac.type == igc_i225) {
		switch (adapter->link_speed) {
		case SPEED_10:
			adjust = IGC_I225_RX_LATENCY_10;
			break;
		case SPEED_100:
			adjust = IGC_I225_RX_LATENCY_100;
			break;
		case SPEED_1000:
			adjust = IGC_I225_RX_LATENCY_1000;
			break;
		case SPEED_2500:
			adjust = IGC_I225_RX_LATENCY_2500;
			break;
		}
	}
	skb_hwtstamps(skb)->hwtstamp =
		ktime_sub_ns(skb_hwtstamps(skb)->hwtstamp, adjust);
}

static void igc_ptp_disable_rx_timestamp(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;
	u32 val;
	int i;

	wr32(IGC_TSYNCRXCTL, 0);

	for (i = 0; i < adapter->num_rx_queues; i++) {
		val = rd32(IGC_SRRCTL(i));
		val &= ~IGC_SRRCTL_TIMESTAMP;
		wr32(IGC_SRRCTL(i), val);
	}

	val = rd32(IGC_RXPBS);
	val &= ~IGC_RXPBS_CFG_TS_EN;
	wr32(IGC_RXPBS, val);
}

static void igc_ptp_enable_rx_timestamp(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;
	u32 val;
	int i;

	val = rd32(IGC_RXPBS);
	val |= IGC_RXPBS_CFG_TS_EN;
	wr32(IGC_RXPBS, val);

	for (i = 0; i < adapter->num_rx_queues; i++) {
		val = rd32(IGC_SRRCTL(i));
		/* FIXME: For now, only support retrieving RX timestamps from
		 * timer 0.
		 */
		val |= IGC_SRRCTL_TIMER1SEL(0) | IGC_SRRCTL_TIMER0SEL(0) |
		       IGC_SRRCTL_TIMESTAMP;
		wr32(IGC_SRRCTL(i), val);
	}

	val = IGC_TSYNCRXCTL_ENABLED | IGC_TSYNCRXCTL_TYPE_ALL |
	      IGC_TSYNCRXCTL_RXSYNSIG;
	wr32(IGC_TSYNCRXCTL, val);
}

static void igc_ptp_disable_tx_timestamp(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;

	wr32(IGC_TSYNCTXCTL, 0);
}

static void igc_ptp_enable_tx_timestamp(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;

	wr32(IGC_TSYNCTXCTL, IGC_TSYNCTXCTL_ENABLED | IGC_TSYNCTXCTL_TXSYNSIG);

	/* Read TXSTMP registers to discard any timestamp previously stored. */
	rd32(IGC_TXSTMPL);
	rd32(IGC_TXSTMPH);
}

/**
 * igc_ptp_set_timestamp_mode - setup hardware for timestamping
 * @adapter: networking device structure
 * @config: hwtstamp configuration
 *
 * Return: 0 in case of success, negative errno code otherwise.
 */
static int igc_ptp_set_timestamp_mode(struct igc_adapter *adapter,
				      struct hwtstamp_config *config)
{
	/* reserved for future extensions */
	if (config->flags)
		return -EINVAL;

	switch (config->tx_type) {
	case HWTSTAMP_TX_OFF:
		igc_ptp_disable_tx_timestamp(adapter);
		break;
	case HWTSTAMP_TX_ON:
		igc_ptp_enable_tx_timestamp(adapter);
		break;
	default:
		return -ERANGE;
	}

	switch (config->rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		igc_ptp_disable_rx_timestamp(adapter);
		break;
	case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
	case HWTSTAMP_FILTER_NTP_ALL:
	case HWTSTAMP_FILTER_ALL:
		igc_ptp_enable_rx_timestamp(adapter);
		config->rx_filter = HWTSTAMP_FILTER_ALL;
		break;
	default:
		return -ERANGE;
	}

	return 0;
}

/* Requires adapter->ptp_tx_lock held by caller. */
static void igc_ptp_tx_timeout(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;

	dev_kfree_skb_any(adapter->ptp_tx_skb);
	adapter->ptp_tx_skb = NULL;
	adapter->ptp_tx_start = 0;
	adapter->tx_hwtstamp_timeouts++;
	/* Clear the tx valid bit in TSYNCTXCTL register to enable interrupt. */
	rd32(IGC_TXSTMPH);

	netdev_warn(adapter->netdev, "Tx timestamp timeout\n");
}

void igc_ptp_tx_hang(struct igc_adapter *adapter)
{
	spin_lock(&adapter->ptp_tx_lock);

	if (!adapter->ptp_tx_skb)
		goto unlock;

	if (time_is_after_jiffies(adapter->ptp_tx_start + IGC_PTP_TX_TIMEOUT))
		goto unlock;

	igc_ptp_tx_timeout(adapter);

unlock:
	spin_unlock(&adapter->ptp_tx_lock);
}

/**
 * igc_ptp_tx_hwtstamp - utility function which checks for TX time stamp
 * @adapter: Board private structure
 *
 * If we were asked to do hardware stamping and such a time stamp is
 * available, then it must have been for this skb here because we only
 * allow only one such packet into the queue.
 *
 * Context: Expects adapter->ptp_tx_lock to be held by caller.
 */
static void igc_ptp_tx_hwtstamp(struct igc_adapter *adapter)
{
	struct sk_buff *skb = adapter->ptp_tx_skb;
	struct skb_shared_hwtstamps shhwtstamps;
	struct igc_hw *hw = &adapter->hw;
	int adjust = 0;
	u64 regval;

	if (WARN_ON_ONCE(!skb))
		return;

	regval = rd32(IGC_TXSTMPL);
	regval |= (u64)rd32(IGC_TXSTMPH) << 32;
	igc_ptp_systim_to_hwtstamp(adapter, &shhwtstamps, regval);

	switch (adapter->link_speed) {
	case SPEED_10:
		adjust = IGC_I225_TX_LATENCY_10;
		break;
	case SPEED_100:
		adjust = IGC_I225_TX_LATENCY_100;
		break;
	case SPEED_1000:
		adjust = IGC_I225_TX_LATENCY_1000;
		break;
	case SPEED_2500:
		adjust = IGC_I225_TX_LATENCY_2500;
		break;
	}

	shhwtstamps.hwtstamp =
		ktime_add_ns(shhwtstamps.hwtstamp, adjust);

	/* Clear the lock early before calling skb_tstamp_tx so that
	 * applications are not woken up before the lock bit is clear. We use
	 * a copy of the skb pointer to ensure other threads can't change it
	 * while we're notifying the stack.
	 */
	adapter->ptp_tx_skb = NULL;
	adapter->ptp_tx_start = 0;

	/* Notify the stack and free the skb after we've unlocked */
	skb_tstamp_tx(skb, &shhwtstamps);
	dev_kfree_skb_any(skb);
}

/**
 * igc_ptp_tx_work
 * @work: pointer to work struct
 *
 * This work function polls the TSYNCTXCTL valid bit to determine when a
 * timestamp has been taken for the current stored skb.
 */
static void igc_ptp_tx_work(struct work_struct *work)
{
	struct igc_adapter *adapter = container_of(work, struct igc_adapter,
						   ptp_tx_work);
	struct igc_hw *hw = &adapter->hw;
	u32 tsynctxctl;

	spin_lock(&adapter->ptp_tx_lock);

	if (!adapter->ptp_tx_skb)
		goto unlock;

	tsynctxctl = rd32(IGC_TSYNCTXCTL);
	if (WARN_ON_ONCE(!(tsynctxctl & IGC_TSYNCTXCTL_TXTT_0)))
		goto unlock;

	igc_ptp_tx_hwtstamp(adapter);

unlock:
	spin_unlock(&adapter->ptp_tx_lock);
}

static struct system_counterval_t igc_device_tstamp_to_system(u64 tstamp)
{
#if !IS_ENABLED(CONFIG_X86_TSC)
	return (struct system_counterval_t) { };
#else
	return convert_art_ns_to_tsc(tstamp);
#endif
}

static void igc_ptm_gather_report(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;
	u32 t2_curr_h, t2_curr_l;
	ktime_t t1, t2_curr;

	t1 = ktime_set(rd32(IGC_PTM_T1_TIM0_H),
		       rd32(IGC_PTM_T1_TIM0_L));

	t2_curr_l = rd32(IGC_PTM_CURR_T2_L);
	t2_curr_h = rd32(IGC_PTM_CURR_T2_H);

	/* FIXME: There's an ambiguity on what endianness some PCIe PTM
	 * messages should use. Find a more robust way to handle this.
	 */
	t2_curr_h = be32_to_cpu(t2_curr_h);

	t2_curr = ((s64)t2_curr_h << 32 | t2_curr_l);

	wr32(IGC_PTM_STAT, IGC_PTM_STAT_VALID);

	mutex_lock(&adapter->ptm_time_lock);

	/* Because get_device_system_crosststamp() requires that the
	 * historic timestamp is before the PTM device/host
	 * timestamps, we keep track of the current and previous
	 * snapshot (historic timestamp).
	 */
	memcpy(&adapter->prev_snapshot,
	       &adapter->curr_snapshot, sizeof(adapter->prev_snapshot));
	ktime_get_snapshot(&adapter->curr_snapshot);

	adapter->ptm_device_time = t1;
	adapter->ptm_host_time = igc_device_tstamp_to_system(t2_curr);
	mutex_unlock(&adapter->ptm_time_lock);

	mod_delayed_work(system_wq, &adapter->ptm_report,
			 msecs_to_jiffies(IGC_PTM_CYCLE_TIME_MSECS));
}

static void igc_ptm_log_error(struct igc_adapter *adapter, u32 ptm_stat)
{
	struct net_device *netdev = adapter->netdev;

	switch (ptm_stat) {
	case IGC_PTM_STAT_RET_ERR:
		netdev_err(netdev, "PTM Error: Root port timeout\n");
		break;
	case IGC_PTM_STAT_BAD_PTM_RES:
		netdev_err(netdev, "PTM Error: Bad response, PTM Response Data expected\n");
		break;
	case IGC_PTM_STAT_T4M1_OVFL:
		netdev_err(netdev, "PTM Error: T4 minus T1 overflow\n");
		break;
	case IGC_PTM_STAT_ADJUST_1ST:
		netdev_err(netdev, "PTM Error: 1588 timer adjusted during first PTM cycle\n");
		break;
	case IGC_PTM_STAT_ADJUST_CYC:
		netdev_err(netdev, "PTM Error: 1588 timer adjusted during non-first PTM cycle\n");
		break;
	}
}

static void igc_ptm_report_work(struct work_struct *work)
{
	struct igc_adapter *adapter = container_of(to_delayed_work(work),
						   struct igc_adapter,
						   ptm_report);
	struct igc_hw *hw = &adapter->hw;
	u32 stat;

	stat = rd32(IGC_PTM_STAT);

	if (stat & IGC_PTM_STAT_VALID) {
		igc_ptm_gather_report(adapter);
		return;
	}

	if (stat & ~IGC_PTM_STAT_VALID) {
		/* An error occurred, log it. */
		igc_ptm_log_error(adapter, stat);

		/* Clear the error bits[1:5] and set VALID bit[0] to start the
		 * next PTM cycle. Status error bits are RW1C write on clear.
		 */
		wr32(IGC_PTM_STAT, stat | IGC_PTM_STAT_VALID);
	}

	/* reschedule to check later. */
	mod_delayed_work(system_wq, &adapter->ptm_report, msecs_to_jiffies(1));
}

/**
 * igc_ptp_set_ts_config - set hardware time stamping config
 * @netdev: network interface device structure
 * @ifreq: interface request data
 *
 **/
int igc_ptp_set_ts_config(struct net_device *netdev, struct ifreq *ifr)
{
	struct igc_adapter *adapter = netdev_priv(netdev);
	struct hwtstamp_config config;
	int err;

	if (copy_from_user(&config, ifr->ifr_data, sizeof(config)))
		return -EFAULT;

	err = igc_ptp_set_timestamp_mode(adapter, &config);
	if (err)
		return err;

	/* save these settings for future reference */
	memcpy(&adapter->tstamp_config, &config,
	       sizeof(adapter->tstamp_config));

	return copy_to_user(ifr->ifr_data, &config, sizeof(config)) ?
		-EFAULT : 0;
}

/**
 * igc_ptp_get_ts_config - get hardware time stamping config
 * @netdev: network interface device structure
 * @ifreq: interface request data
 *
 * Get the hwtstamp_config settings to return to the user. Rather than attempt
 * to deconstruct the settings from the registers, just return a shadow copy
 * of the last known settings.
 **/
int igc_ptp_get_ts_config(struct net_device *netdev, struct ifreq *ifr)
{
	struct igc_adapter *adapter = netdev_priv(netdev);
	struct hwtstamp_config *config = &adapter->tstamp_config;

	return copy_to_user(ifr->ifr_data, config, sizeof(*config)) ?
		-EFAULT : 0;
}

static int igc_phc_get_syncdevicetime(ktime_t *device,
				      struct system_counterval_t *system,
				      void *ctx)
{
	struct igc_adapter *adapter = ctx;

	mutex_lock(&adapter->ptm_time_lock);

	*device = adapter->ptm_device_time;
	*system = adapter->ptm_host_time;

	mutex_unlock(&adapter->ptm_time_lock);
	return 0;
}

static int igc_ptp_getcrosststamp(struct ptp_clock_info *ptp,
				  struct system_device_crosststamp *cts)
{
	struct igc_adapter *adapter = container_of(ptp, struct igc_adapter,
						   ptp_caps);

	return get_device_system_crosststamp(igc_phc_get_syncdevicetime,
					     adapter, &adapter->prev_snapshot, cts);
}

static bool igc_is_ptm_supported(struct igc_adapter *adapter)
{
#if IS_ENABLED(CONFIG_X86_TSC) && IS_ENABLED(CONFIG_PCIE_PTM)
	return adapter->pdev->ptm_enabled;
#endif
	return false;
}

static bool igc_ptm_init(struct igc_adapter *adapter)
{
	if (!igc_is_ptm_supported(adapter))
		return false;

	INIT_DELAYED_WORK(&adapter->ptm_report, igc_ptm_report_work);

	/* Get a snapshot of system clocks to use as historic value. */
	ktime_get_snapshot(&adapter->prev_snapshot);
	ktime_get_snapshot(&adapter->curr_snapshot);

	return true;
}

/**
 * igc_ptp_init - Initialize PTP functionality
 * @adapter: Board private structure
 *
 * This function is called at device probe to initialize the PTP
 * functionality.
 */
void igc_ptp_init(struct igc_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;
	struct igc_hw *hw = &adapter->hw;

	switch (hw->mac.type) {
	case igc_i225:
		snprintf(adapter->ptp_caps.name, 16, "%pm", netdev->dev_addr);
		adapter->ptp_caps.owner = THIS_MODULE;
		adapter->ptp_caps.max_adj = 62499999;
		adapter->ptp_caps.adjfine = igc_ptp_adjfine_i225;
		adapter->ptp_caps.adjtime = igc_ptp_adjtime_i225;
		adapter->ptp_caps.gettimex64 = igc_ptp_gettimex64_i225;
		adapter->ptp_caps.settime64 = igc_ptp_settime_i225;
		adapter->ptp_caps.enable = igc_ptp_feature_enable_i225;

		if (!igc_ptm_init(adapter))
			break;

		adapter->ptp_caps.getcrosststamp = igc_ptp_getcrosststamp;
		adapter->ptp_caps.pps = 1;
		break;
	default:
		adapter->ptp_clock = NULL;
		return;
	}

	spin_lock_init(&adapter->tmreg_lock);
	spin_lock_init(&adapter->ptp_tx_lock);
	INIT_WORK(&adapter->ptp_tx_work, igc_ptp_tx_work);

	adapter->tstamp_config.rx_filter = HWTSTAMP_FILTER_NONE;
	adapter->tstamp_config.tx_type = HWTSTAMP_TX_OFF;

	adapter->prev_ptp_time = ktime_to_timespec64(ktime_get_real());
	adapter->ptp_reset_start = ktime_get();

	adapter->ptp_clock = ptp_clock_register(&adapter->ptp_caps,
						&adapter->pdev->dev);
	if (IS_ERR(adapter->ptp_clock)) {
		adapter->ptp_clock = NULL;
		netdev_err(netdev, "ptp_clock_register failed\n");
	} else if (adapter->ptp_clock) {
		netdev_info(netdev, "PHC added\n");
		adapter->ptp_flags |= IGC_PTP_ENABLED;
	}
}

static void igc_ptp_time_save(struct igc_adapter *adapter)
{
	igc_ptp_read(adapter, &adapter->prev_ptp_time);
	adapter->ptp_reset_start = ktime_get();
}

static void igc_ptp_time_restore(struct igc_adapter *adapter)
{
	struct timespec64 ts = adapter->prev_ptp_time;
	ktime_t delta;

	delta = ktime_sub(ktime_get(), adapter->ptp_reset_start);

	timespec64_add_ns(&ts, ktime_to_ns(delta));

	igc_ptp_write_i225(adapter, &ts);
}

static void igc_ptm_stop(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;

	wr32(IGC_PTM_CYCLE_CTRL, 0);
	wr32(IGC_PTM_CTRL, 0);

	cancel_delayed_work_sync(&adapter->ptm_report);
}

static void igc_ptm_start(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;
	u32 cycle_ctrl, ctrl;

	if (!igc_is_ptm_supported(adapter))
		return;

	wr32(IGC_PCIE_DIG_DELAY, IGC_PCIE_DIG_DELAY_DEFAULT);
	wr32(IGC_PCIE_PHY_DELAY, IGC_PCIE_PHY_DELAY_DEFAULT);

	ctrl = IGC_PTM_CTRL_EN |
		IGC_PTM_CTRL_SHRT_CYC(20) |
		IGC_PTM_CTRL_PTM_TO(255);

	cycle_ctrl = IGC_PTM_CYCLE_CTRL_CYC_TIME(IGC_PTM_CYCLE_TIME_MSECS) |
		IGC_PTM_CYCLE_CTRL_AUTO_CYC_EN;

	wr32(IGC_PTM_CYCLE_CTRL, cycle_ctrl);
	wr32(IGC_PTM_CTRL, ctrl);

	/* The cycle only starts "for real" when software notifies
	 * that it has read the registers, this is done by setting
	 * VALID bit.
	 */
	wr32(IGC_PTM_STAT, IGC_PTM_STAT_VALID);

	schedule_delayed_work(&adapter->ptm_report,
			      msecs_to_jiffies(IGC_PTM_CYCLE_TIME_MSECS));
}

/**
 * igc_ptp_suspend - Disable PTP work items and prepare for suspend
 * @adapter: Board private structure
 *
 * This function stops the overflow check work and PTP Tx timestamp work, and
 * will prepare the device for OS suspend.
 */
void igc_ptp_suspend(struct igc_adapter *adapter)
{
	if (!(adapter->ptp_flags & IGC_PTP_ENABLED))
		return;

	igc_ptm_stop(adapter);

	cancel_work_sync(&adapter->ptp_tx_work);

	spin_lock(&adapter->ptp_tx_lock);

	dev_kfree_skb_any(adapter->ptp_tx_skb);
	adapter->ptp_tx_skb = NULL;
	adapter->ptp_tx_start = 0;

	spin_unlock(&adapter->ptp_tx_lock);

	igc_ptp_time_save(adapter);
}

/**
 * igc_ptp_stop - Disable PTP device and stop the overflow check.
 * @adapter: Board private structure.
 *
 * This function stops the PTP support and cancels the delayed work.
 **/
void igc_ptp_stop(struct igc_adapter *adapter)
{
	igc_ptp_suspend(adapter);

	if (adapter->ptp_clock) {
		ptp_clock_unregister(adapter->ptp_clock);
		netdev_info(adapter->netdev, "PHC removed\n");
		adapter->ptp_flags &= ~IGC_PTP_ENABLED;
	}
}

/**
 * igc_ptp_reset - Re-enable the adapter for PTP following a reset.
 * @adapter: Board private structure.
 *
 * This function handles the reset work required to re-enable the PTP device.
 **/
void igc_ptp_reset(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;
	unsigned long flags;
	u32 timadj;

	if (!(adapter->ptp_flags & IGC_PTP_ENABLED))
		return;

	/* reset the tstamp_config */
	igc_ptp_set_timestamp_mode(adapter, &adapter->tstamp_config);

	spin_lock_irqsave(&adapter->tmreg_lock, flags);

	switch (adapter->hw.mac.type) {
	case igc_i225:
		timadj = rd32(IGC_TIMADJ);
		timadj |= IGC_TIMADJ_ADJUST_METH;
		wr32(IGC_TIMADJ, timadj);

		wr32(IGC_TSAUXC, 0x0);
		wr32(IGC_TSSDP, 0x0);
		wr32(IGC_TSIM,
		     IGC_TSICR_INTERRUPTS |
		     (adapter->pps_sys_wrap_on ? IGC_TSICR_SYS_WRAP : 0));
		wr32(IGC_IMS, IGC_IMS_TS);

		igc_ptm_start(adapter);

		break;
	default:
		/* No work to do. */
		goto out;
	}

	/* Re-initialize the timer. */
	if (hw->mac.type == igc_i225) {
		igc_ptp_time_restore(adapter);
	} else {
		timecounter_init(&adapter->tc, &adapter->cc,
				 ktime_to_ns(ktime_get_real()));
	}
out:
	spin_unlock_irqrestore(&adapter->tmreg_lock, flags);

	wrfl();
}
