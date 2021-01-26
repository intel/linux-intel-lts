// SPDX-License-Identifier: GPL-2.0
/* Copyright (c)  2019 Intel Corporation */

#include "igc.h"
#include "igc_tsn.h"

static bool is_any_launchtime(struct igc_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_tx_queues; i++) {
		struct igc_ring *ring = adapter->tx_ring[i];

		if (ring->launchtime_enable)
			return true;
	}

	return false;
}

static unsigned int igc_tsn_new_flags(struct igc_adapter *adapter)
{
	unsigned int new_flags = adapter->flags & ~IGC_FLAG_TSN_ANY_ENABLED;

	if (adapter->base_time)
		new_flags |= IGC_FLAG_TSN_QBV_ENABLED;

	if (is_any_launchtime(adapter))
		new_flags |= IGC_FLAG_TSN_QBV_ENABLED;

	if (adapter->frame_preemption_active)
		new_flags |= IGC_FLAG_TSN_PREEMPT_ENABLED;

	return new_flags;
}

/* Returns the TSN specific registers to their default values after
 * the adapter is reset.
 */
static int igc_tsn_disable_offload(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;
	u32 tqavctrl, rxpbs;
	int i;

	adapter->base_time = 0;
	adapter->cycle_time = 0;
	adapter->frame_preemption_active = false;
	adapter->add_frag_size = IGC_I225_MIN_FRAG_SIZE_DEFAULT;

	wr32(IGC_TXPBS, I225_TXPBSIZE_DEFAULT);
	wr32(IGC_DTXMXPKTSZ, IGC_DTXMXPKTSZ_DEFAULT);

	rxpbs = rd32(IGC_RXPBS) & ~IGC_RXPBSIZE_SIZE_MASK;
	rxpbs |= I225_RXPBSIZE_DEFAULT;

	wr32(IGC_RXPBS, rxpbs);

	tqavctrl = rd32(IGC_TQAVCTRL);
	tqavctrl &= ~(IGC_TQAVCTRL_TRANSMIT_MODE_TSN |
		      IGC_TQAVCTRL_ENHANCED_QAV | IGC_TQAVCTRL_PREEMPT_ENA |
		      IGC_TQAVCTRL_MIN_FRAG_MASK);
	wr32(IGC_TQAVCTRL, tqavctrl);

	for (i = 0; i < adapter->num_tx_queues; i++) {
		struct igc_ring *ring = adapter->tx_ring[i];

		ring->start_time = 0;
		ring->end_time = 0;
		ring->launchtime_enable = false;
		ring->preemptible = false;

		wr32(IGC_TXQCTL(i), 0);
		wr32(IGC_STQT(i), 0);
		wr32(IGC_ENDQT(i), NSEC_PER_MSEC);
	}

	wr32(IGC_QBVCYCLET_S, NSEC_PER_MSEC);
	wr32(IGC_QBVCYCLET, NSEC_PER_MSEC);

	adapter->flags &= ~IGC_FLAG_TSN_ANY_ENABLED;

	return 0;
}

static int igc_tsn_update_params(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;
	unsigned int flags;
	u8 frag_size_mult;
	u32 tqavctrl;
	int i;

	flags = igc_tsn_new_flags(adapter) & IGC_FLAG_TSN_ANY_ENABLED;
	if (!flags)
		return 0;

	tqavctrl = rd32(IGC_TQAVCTRL) &
		~(IGC_TQAVCTRL_MIN_FRAG_MASK | IGC_TQAVCTRL_PREEMPT_ENA);

	if (adapter->frame_preemption_active)
		tqavctrl |= IGC_TQAVCTRL_PREEMPT_ENA;

	frag_size_mult = ethtool_frag_size_to_mult(adapter->add_frag_size);

	tqavctrl |= frag_size_mult << IGC_TQAVCTRL_MIN_FRAG_SHIFT;

	wr32(IGC_TQAVCTRL, tqavctrl);

	for (i = 0; i < adapter->num_tx_queues; i++) {
		struct igc_ring *ring = adapter->tx_ring[i];
		u32 txqctl = 0;

		wr32(IGC_STQT(i), ring->start_time);
		wr32(IGC_ENDQT(i), ring->end_time);

		if (adapter->base_time) {
			/* If we have a base_time we are in "taprio"
			 * mode and we need to be strict about the
			 * cycles: only transmit a packet if it can be
			 * completed during that cycle.
			 */
			txqctl |= IGC_TXQCTL_STRICT_CYCLE |
				IGC_TXQCTL_STRICT_END;
		}

		if (ring->launchtime_enable)
			txqctl |= IGC_TXQCTL_QUEUE_MODE_LAUNCHT;

		if (adapter->frame_preemption_active && ring->preemptible)
			txqctl |= IGC_TXQCTL_PREEMPTABLE;

		wr32(IGC_TXQCTL(i), txqctl);
	}

	adapter->flags = igc_tsn_new_flags(adapter);

	return 0;
}

static int igc_tsn_enable_offload(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;
	u32 baset_l, baset_h, tqavctrl;
	u32 sec, nsec, cycle, rxpbs;
	ktime_t base_time, systim;

	tqavctrl = rd32(IGC_TQAVCTRL);
	tqavctrl |= IGC_TQAVCTRL_TRANSMIT_MODE_TSN | IGC_TQAVCTRL_ENHANCED_QAV;

	wr32(IGC_TQAVCTRL, tqavctrl);

	wr32(IGC_TSAUXC, 0);
	wr32(IGC_DTXMXPKTSZ, IGC_DTXMXPKTSZ_TSN);
	wr32(IGC_TXPBS, IGC_TXPBSIZE_TSN);

	rxpbs = rd32(IGC_RXPBS) & ~IGC_RXPBSIZE_SIZE_MASK;
	rxpbs |= IGC_RXPBSIZE_TSN;

	wr32(IGC_RXPBS, rxpbs);

	if (!adapter->base_time)
		goto done;

	cycle = adapter->cycle_time;
	base_time = adapter->base_time;

	wr32(IGC_QBVCYCLET_S, cycle);
	wr32(IGC_QBVCYCLET, cycle);

	nsec = rd32(IGC_SYSTIML);
	sec = rd32(IGC_SYSTIMH);

	systim = ktime_set(sec, nsec);

	if (ktime_compare(systim, base_time) > 0) {
		s64 n;

		n = div64_s64(ktime_sub_ns(systim, base_time), cycle);
		base_time = ktime_add_ns(base_time, (n + 1) * cycle);
	}

	baset_h = div_s64_rem(base_time, NSEC_PER_SEC, &baset_l);

	wr32(IGC_BASET_H, baset_h);
	wr32(IGC_BASET_L, baset_l);

done:
	igc_tsn_update_params(adapter);

	return 0;
}

int igc_tsn_reset(struct igc_adapter *adapter)
{
	unsigned int new_flags;
	int err = 0;

	new_flags = igc_tsn_new_flags(adapter);

	if (!(new_flags & IGC_FLAG_TSN_ANY_ENABLED))
		return igc_tsn_disable_offload(adapter);

	err = igc_tsn_enable_offload(adapter);
	if (err < 0)
		return err;

	adapter->flags = new_flags;

	return err;
}

int igc_tsn_offload_apply(struct igc_adapter *adapter)
{
	unsigned int new_flags, old_flags;

	old_flags = adapter->flags;
	new_flags = igc_tsn_new_flags(adapter);

	if (old_flags == new_flags)
		return igc_tsn_update_params(adapter);

	/* Enabling features work without resetting the adapter */
	if (new_flags > old_flags)
		return igc_tsn_enable_offload(adapter);

	adapter->flags = new_flags;

	if (!netif_running(adapter->netdev))
		return igc_tsn_enable_offload(adapter);

	schedule_work(&adapter->reset_task);

	return 0;
}
