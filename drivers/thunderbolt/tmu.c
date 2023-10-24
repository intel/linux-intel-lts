// SPDX-License-Identifier: GPL-2.0
/*
 * Thunderbolt Time Management Unit (TMU) support
 *
 * Copyright (C) 2019, Intel Corporation
 * Authors: Mika Westerberg <mika.westerberg@linux.intel.com>
 *	    Rajmohan Mani <rajmohan.mani@intel.com>
 */

#include <linux/delay.h>

#include "tb.h"

/*
 * From v2 connection manager guide used with enhanced uni-directional
 * HiFi TMU mode.
 */
#define	REPL_TIMEOUT	1875
#define	REPL_THRESHOLD	0
#define	REPL_N		222
#define	DIRSWITCH_N	255

static int tb_switch_set_tmu_mode_params(struct tb_switch *sw,
					 enum tb_switch_tmu_rate rate)
{
	u32 freq_meas_wind[2] = { 30, 800 };
	u32 avg_const[2] = { 4, 8 };
	u32 freq, avg, val;
	int ret;

	if (rate == TB_SWITCH_TMU_RATE_NORMAL) {
		freq = freq_meas_wind[0];
		avg = avg_const[0];
	} else if (rate == TB_SWITCH_TMU_RATE_HIFI) {
		freq = freq_meas_wind[1];
		avg = avg_const[1];
	} else {
		return 0;
	}

	ret = tb_sw_read(sw, &val, TB_CFG_SWITCH,
			 sw->tmu.cap + TMU_RTR_CS_0, 1);
	if (ret)
		return ret;

	val &= ~TMU_RTR_CS_0_FREQ_WIND_MASK;
	val |= FIELD_PREP(TMU_RTR_CS_0_FREQ_WIND_MASK, freq);

	ret = tb_sw_write(sw, &val, TB_CFG_SWITCH,
			  sw->tmu.cap + TMU_RTR_CS_0, 1);
	if (ret)
		return ret;

	ret = tb_sw_read(sw, &val, TB_CFG_SWITCH,
			 sw->tmu.cap + TMU_RTR_CS_15, 1);
	if (ret)
		return ret;

	val &= ~TMU_RTR_CS_15_FREQ_AVG_MASK &
		~TMU_RTR_CS_15_DELAY_AVG_MASK &
		~TMU_RTR_CS_15_OFFSET_AVG_MASK &
		~TMU_RTR_CS_15_ERROR_AVG_MASK;
	val |=  FIELD_PREP(TMU_RTR_CS_15_FREQ_AVG_MASK, avg) |
		FIELD_PREP(TMU_RTR_CS_15_DELAY_AVG_MASK, avg) |
		FIELD_PREP(TMU_RTR_CS_15_OFFSET_AVG_MASK, avg) |
		FIELD_PREP(TMU_RTR_CS_15_ERROR_AVG_MASK, avg);

	ret = tb_sw_write(sw, &val, TB_CFG_SWITCH,
			 sw->tmu.cap + TMU_RTR_CS_15, 1);
	if (ret)
		return ret;

	if (tb_switch_tmu_enhanced_is_supported(sw)) {
		ret = tb_sw_read(sw, &val, TB_CFG_SWITCH,
				 sw->tmu.cap + TMU_RTR_CS_18, 1);
		if (ret)
			return ret;

		val &= ~TMU_RTR_CS_18_DELTA_AVG_CONST_MASK;
		val |= FIELD_PREP(TMU_RTR_CS_18_DELTA_AVG_CONST_MASK, avg);

		ret = tb_sw_write(sw, &val, TB_CFG_SWITCH,
				  sw->tmu.cap + TMU_RTR_CS_18, 1);
	}

	return ret;
}

static const char *tb_switch_tmu_mode_name(const struct tb_switch *sw)
{
	bool root_switch = !tb_route(sw);

	switch (sw->tmu.rate) {
	case TB_SWITCH_TMU_RATE_OFF:
		return "off";

	case TB_SWITCH_TMU_RATE_HIFI:
		/* Root switch does not have upstream directionality */
		if (root_switch)
			return "HiFi";
		if (sw->tmu.mode == TB_SWITCH_TMU_MODE_UNI)
			return "uni-directional, HiFi";
		if (sw->tmu.mode == TB_SWITCH_TMU_MODE_ENHANCED_UNI)
			return "enhanced uni-directional, HiFi";
		return "bi-directional, HiFi";

	case TB_SWITCH_TMU_RATE_NORMAL:
		if (root_switch)
			return "normal";
		return "uni-directional, normal";

	default:
		return "unknown";
	}
}

static bool tb_switch_tmu_ucap_is_supported(struct tb_switch *sw)
{
	int ret;
	u32 val;

	ret = tb_sw_read(sw, &val, TB_CFG_SWITCH,
			 sw->tmu.cap + TMU_RTR_CS_0, 1);
	if (ret)
		return false;

	return !!(val & TMU_RTR_CS_0_UCAP);
}

static int tb_switch_tmu_rate_read(struct tb_switch *sw)
{
	int ret;
	u32 val;

	ret = tb_sw_read(sw, &val, TB_CFG_SWITCH,
			 sw->tmu.cap + TMU_RTR_CS_3, 1);
	if (ret)
		return ret;

	val >>= TMU_RTR_CS_3_TS_PACKET_INTERVAL_SHIFT;
	return val;
}

static int tb_switch_tmu_rate_write(struct tb_switch *sw, int rate)
{
	int ret;
	u32 val;

	ret = tb_sw_read(sw, &val, TB_CFG_SWITCH,
			 sw->tmu.cap + TMU_RTR_CS_3, 1);
	if (ret)
		return ret;

	val &= ~TMU_RTR_CS_3_TS_PACKET_INTERVAL_MASK;
	val |= rate << TMU_RTR_CS_3_TS_PACKET_INTERVAL_SHIFT;

	return tb_sw_write(sw, &val, TB_CFG_SWITCH,
			   sw->tmu.cap + TMU_RTR_CS_3, 1);
}

static int tb_port_tmu_write(struct tb_port *port, u8 offset, u32 mask,
			     u32 value)
{
	u32 data;
	int ret;

	ret = tb_port_read(port, &data, TB_CFG_PORT, port->cap_tmu + offset, 1);
	if (ret)
		return ret;

	data &= ~mask;
	data |= value;

	return tb_port_write(port, &data, TB_CFG_PORT,
			     port->cap_tmu + offset, 1);
}

static int tb_port_tmu_set_unidirectional(struct tb_port *port,
					  bool unidirectional)
{
	u32 val;

	if (!port->sw->tmu.has_ucap)
		return 0;

	val = unidirectional ? TMU_ADP_CS_3_UDM : 0;
	return tb_port_tmu_write(port, TMU_ADP_CS_3, TMU_ADP_CS_3_UDM, val);
}

static inline int tb_port_tmu_unidirectional_disable(struct tb_port *port)
{
	return tb_port_tmu_set_unidirectional(port, false);
}

static inline int tb_port_tmu_unidirectional_enable(struct tb_port *port)
{
	return tb_port_tmu_set_unidirectional(port, true);
}

static bool tb_port_tmu_is_unidirectional(struct tb_port *port)
{
	int ret;
	u32 val;

	ret = tb_port_read(port, &val, TB_CFG_PORT,
			   port->cap_tmu + TMU_ADP_CS_3, 1);
	if (ret)
		return false;

	return val & TMU_ADP_CS_3_UDM;
}

static bool tb_port_tmu_is_enhanced(struct tb_port *port)
{
	int ret;
	u32 val;

	ret = tb_port_read(port, &val, TB_CFG_PORT,
			   port->cap_tmu + TMU_ADP_CS_8, 1);
	if (ret)
		return false;

	return val & TMU_ADP_CS_8_EUDM;
}

/* Can be called to non-v2 lane adapters too */
static int tb_port_tmu_enhanced_enable(struct tb_port *port, bool enable)
{
	int ret;
	u32 val;

	if (!tb_switch_tmu_enhanced_is_supported(port->sw))
		return 0;

	ret = tb_port_read(port, &val, TB_CFG_PORT,
			   port->cap_tmu + TMU_ADP_CS_8, 1);
	if (ret)
		return ret;

	if (enable)
		val |= TMU_ADP_CS_8_EUDM;
	else
		val &= ~TMU_ADP_CS_8_EUDM;

	return tb_port_write(port, &val, TB_CFG_PORT,
			     port->cap_tmu + TMU_ADP_CS_8, 1);
}

static int tb_port_set_tmu_mode_params(struct tb_port *port, int repl_timeout,
				       int repl_threshold, int repl_n,
				       int dirswitch_n)
{
	int ret;
	u32 val;

	ret = tb_port_read(port, &val, TB_CFG_PORT,
			   port->cap_tmu + TMU_ADP_CS_8, 1);
	if (ret)
		return ret;

	val &= ~TMU_ADP_CS_8_REPL_TIMEOUT_MASK;
	val &= ~TMU_ADP_CS_8_REPL_THRESHOLD_MASK;
	val |= FIELD_PREP(TMU_ADP_CS_8_REPL_TIMEOUT_MASK, repl_timeout);
	val |= FIELD_PREP(TMU_ADP_CS_8_REPL_THRESHOLD_MASK, repl_threshold);

	ret = tb_port_write(port, &val, TB_CFG_PORT,
			    port->cap_tmu + TMU_ADP_CS_8, 1);
	if (ret)
		return ret;

	ret = tb_port_read(port, &val, TB_CFG_PORT,
			   port->cap_tmu + TMU_ADP_CS_9, 1);
	if (ret)
		return ret;

	val &= ~TMU_ADP_CS_9_REPL_N_MASK;
	val &= ~TMU_ADP_CS_9_DIRSWITCH_N_MASK;
	val |= FIELD_PREP(TMU_ADP_CS_9_REPL_N_MASK, repl_n);
	val |= FIELD_PREP(TMU_ADP_CS_9_DIRSWITCH_N_MASK, dirswitch_n);

	return tb_port_write(port, &val, TB_CFG_PORT,
			     port->cap_tmu + TMU_ADP_CS_9, 1);
}

static int tb_port_tmu_rate_read(struct tb_port *port)
{
	int ret;
	u32 val;

	ret = tb_port_read(port, &val, TB_CFG_PORT,
			   port->cap_tmu + TMU_ADP_CS_9, 1);
	if (ret)
		return ret;

	return FIELD_GET(TMU_ADP_CS_9_ADP_TS_INTERVAL_MASK, val);
}

/* Can be called to non-v2 lane adapters too */
static int tb_port_tmu_rate_write(struct tb_port *port, int rate)
{
	int ret;
	u32 val;

	if (!tb_switch_tmu_enhanced_is_supported(port->sw))
		return 0;

	ret = tb_port_read(port, &val, TB_CFG_PORT,
			   port->cap_tmu + TMU_ADP_CS_9, 1);
	if (ret)
		return ret;

	val &= ~TMU_ADP_CS_9_ADP_TS_INTERVAL_MASK;
	val |= FIELD_PREP(TMU_ADP_CS_9_ADP_TS_INTERVAL_MASK, rate);

	return tb_port_write(port, &val, TB_CFG_PORT,
			     port->cap_tmu + TMU_ADP_CS_9, 1);
}

static int tb_port_tmu_time_sync(struct tb_port *port, bool time_sync)
{
	u32 val = time_sync ? TMU_ADP_CS_6_DTS : 0;

	return tb_port_tmu_write(port, TMU_ADP_CS_6, TMU_ADP_CS_6_DTS, val);
}

static int tb_port_tmu_time_sync_disable(struct tb_port *port)
{
	return tb_port_tmu_time_sync(port, true);
}

static int tb_port_tmu_time_sync_enable(struct tb_port *port)
{
	return tb_port_tmu_time_sync(port, false);
}

static int tb_switch_tmu_set_time_disruption(struct tb_switch *sw, bool set)
{
	u32 val, offset, bit;
	int ret;

	if (tb_switch_is_usb4(sw)) {
		offset = sw->tmu.cap + TMU_RTR_CS_0;
		bit = TMU_RTR_CS_0_TD;
	} else {
		offset = sw->cap_vsec_tmu + TB_TIME_VSEC_3_CS_26;
		bit = TB_TIME_VSEC_3_CS_26_TD;
	}

	ret = tb_sw_read(sw, &val, TB_CFG_SWITCH, offset, 1);
	if (ret)
		return ret;

	if (set)
		val |= bit;
	else
		val &= ~bit;

	return tb_sw_write(sw, &val, TB_CFG_SWITCH, offset, 1);
}

/**
 * tb_switch_tmu_enhanced_is_supported() - Is enhanced TMU mode supported
 * @sw: Router to check (can be %NULL)
 */
bool tb_switch_tmu_enhanced_is_supported(const struct tb_switch *sw)
{
	if (sw)
		return usb4_switch_version(sw) > 1;
	return false;
}

/**
 * tb_switch_tmu_init() - Initialize switch TMU structures
 * @sw: Switch to initialized
 *
 * This function must be called before other TMU related functions to
 * makes the internal structures are filled in correctly. Does not
 * change any hardware configuration.
 */
int tb_switch_tmu_init(struct tb_switch *sw)
{
	struct tb_port *port;
	int ret;

	if (tb_switch_is_icm(sw))
		return 0;

	ret = tb_switch_find_cap(sw, TB_SWITCH_CAP_TMU);
	if (ret > 0)
		sw->tmu.cap = ret;

	tb_switch_for_each_port(sw, port) {
		int cap;

		cap = tb_port_find_cap(port, TB_PORT_CAP_TIME1);
		if (cap > 0)
			port->cap_tmu = cap;
	}

	ret = tb_switch_tmu_rate_read(sw);
	if (ret < 0)
		return ret;

	sw->tmu.rate = ret;

	sw->tmu.has_ucap = tb_switch_tmu_ucap_is_supported(sw);
	if (sw->tmu.has_ucap) {
		bool enhanced = tb_switch_tmu_enhanced_is_supported(sw);

		tb_sw_dbg(sw, "TMU: supports %suni-directional mode\n",
			  enhanced ? "enhanced " : "");

		if (tb_route(sw)) {
			struct tb_port *up = tb_upstream_port(sw);

			if (enhanced && tb_port_tmu_is_enhanced(up)) {
				sw->tmu.mode = TB_SWITCH_TMU_MODE_ENHANCED_UNI;
				ret = tb_port_tmu_rate_read(up);
				if (ret < 0)
					return ret;
				/*
				 * TMU rate is the one configured for
				 * the upstream adapter on v2 routers.
				 */
				sw->tmu.rate = ret;
			} else {
				sw->tmu.mode = tb_port_tmu_is_unidirectional(up) ?
					TB_SWITCH_TMU_MODE_UNI :
					TB_SWITCH_TMU_MODE_BI;
			}
		}
	} else {
		sw->tmu.mode = TB_SWITCH_TMU_MODE_BI;
	}

	tb_sw_dbg(sw, "TMU: current mode: %s\n", tb_switch_tmu_mode_name(sw));
	return 0;
}

/**
 * tb_switch_tmu_post_time() - Update switch local time
 * @sw: Switch whose time to update
 *
 * Updates switch local time using time posting procedure.
 */
int tb_switch_tmu_post_time(struct tb_switch *sw)
{
	unsigned int post_time_high_offset, post_time_high = 0;
	unsigned int post_local_time_offset, post_time_offset;
	struct tb_switch *root_switch = sw->tb->root_switch;
	u64 hi, mid, lo, local_time, post_time;
	int i, ret, retries = 100;
	u32 gm_local_time[3];

	if (!tb_route(sw))
		return 0;

	if (!tb_switch_is_usb4(sw))
		return 0;

	/* Need to be able to read the grand master time */
	if (!root_switch->tmu.cap)
		return 0;

	ret = tb_sw_read(root_switch, gm_local_time, TB_CFG_SWITCH,
			 root_switch->tmu.cap + TMU_RTR_CS_1,
			 ARRAY_SIZE(gm_local_time));
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(gm_local_time); i++)
		tb_sw_dbg(root_switch, "local_time[%d]=0x%08x\n", i,
			  gm_local_time[i]);

	/* Convert to nanoseconds (drop fractional part) */
	hi = gm_local_time[2] & TMU_RTR_CS_3_LOCAL_TIME_NS_MASK;
	mid = gm_local_time[1];
	lo = (gm_local_time[0] & TMU_RTR_CS_1_LOCAL_TIME_NS_MASK) >>
		TMU_RTR_CS_1_LOCAL_TIME_NS_SHIFT;
	local_time = hi << 48 | mid << 16 | lo;

	/* Tell the switch that time sync is disrupted for a while */
	ret = tb_switch_tmu_set_time_disruption(sw, true);
	if (ret)
		return ret;

	post_local_time_offset = sw->tmu.cap + TMU_RTR_CS_22;
	post_time_offset = sw->tmu.cap + TMU_RTR_CS_24;
	post_time_high_offset = sw->tmu.cap + TMU_RTR_CS_25;

	/*
	 * Write the Grandmaster time to the Post Local Time registers
	 * of the new switch.
	 */
	ret = tb_sw_write(sw, &local_time, TB_CFG_SWITCH,
			  post_local_time_offset, 2);
	if (ret)
		goto out;

	/*
	 * Have the new switch update its local time by:
	 * 1) writing 0x1 to the Post Time Low register and 0xffffffff to
	 * Post Time High register.
	 * 2) write 0 to Post Time High register and then wait for
	 * the completion of the post_time register becomes 0.
	 * This means the time has been converged properly.
	 */
	post_time = 0xffffffff00000001ULL;

	ret = tb_sw_write(sw, &post_time, TB_CFG_SWITCH, post_time_offset, 2);
	if (ret)
		goto out;

	ret = tb_sw_write(sw, &post_time_high, TB_CFG_SWITCH,
			  post_time_high_offset, 1);
	if (ret)
		goto out;

	do {
		usleep_range(5, 10);
		ret = tb_sw_read(sw, &post_time, TB_CFG_SWITCH,
				 post_time_offset, 2);
		if (ret)
			goto out;
	} while (--retries && post_time);

	if (!retries) {
		ret = -ETIMEDOUT;
		goto out;
	}

	tb_sw_dbg(sw, "TMU: updated local time to %#llx\n", local_time);

out:
	tb_switch_tmu_set_time_disruption(sw, false);
	return ret;
}

static int disable_enhanced(struct tb_port *up, struct tb_port *down)
{
	int ret;

	/*
	 * Router may already been disconnected so ignore errors on the
	 * upstream port.
	 */
	tb_port_tmu_rate_write(up, 0);
	tb_port_tmu_enhanced_enable(up, false);

	ret = tb_port_tmu_rate_write(down, 0);
	if (ret)
		return ret;
	return tb_port_tmu_enhanced_enable(down, false);
}

/**
 * tb_switch_tmu_disable() - Disable TMU of a switch
 * @sw: Switch whose TMU to disable
 *
 * Turns off TMU of @sw if it is enabled. If not enabled does nothing.
 */
int tb_switch_tmu_disable(struct tb_switch *sw)
{
	/* Already disabled? */
	if (sw->tmu.rate == TB_SWITCH_TMU_RATE_OFF)
		return 0;

	if (tb_route(sw)) {
		struct tb_port *down, *up;
		int ret;

		down = tb_switch_downstream_port(sw);
		up = tb_upstream_port(sw);
		/*
		 * In case of uni-directional time sync, TMU handshake is
		 * initiated by upstream router. In case of bi-directional
		 * time sync, TMU handshake is initiated by downstream router.
		 * We change downstream router's rate to off for both uni/bidir
		 * cases although it is needed only for the bi-directional mode.
		 * We avoid changing upstream router's mode since it might
		 * have another downstream router plugged, that is set to
		 * uni-directional mode and we don't want to change it's TMU
		 * mode.
		 */
		ret = tb_switch_tmu_rate_write(sw, TB_SWITCH_TMU_RATE_OFF);
		if (ret)
			return ret;

		tb_port_tmu_time_sync_disable(up);
		ret = tb_port_tmu_time_sync_disable(down);
		if (ret)
			return ret;

		switch (sw->tmu.mode) {
		case TB_SWITCH_TMU_MODE_UNI:
			/* The switch may be unplugged so ignore any errors */
			tb_port_tmu_unidirectional_disable(up);
			ret = tb_port_tmu_unidirectional_disable(down);
			if (ret)
				return ret;
			break;

		case TB_SWITCH_TMU_MODE_ENHANCED_UNI:
			ret = disable_enhanced(up, down);
			if (ret)
				return ret;
			break;

		default:
			break;
		}
	} else {
		tb_switch_tmu_rate_write(sw, TB_SWITCH_TMU_RATE_OFF);
	}

	sw->tmu.rate = TB_SWITCH_TMU_RATE_OFF;

	tb_sw_dbg(sw, "TMU: disabled\n");
	return 0;
}

/* Called only when there is failure enabling requested mode */
static void tb_switch_tmu_off(struct tb_switch *sw)
{
	struct tb_port *down, *up;

	down = tb_switch_downstream_port(sw);
	up = tb_upstream_port(sw);
	/*
	 * In case of any failure in one of the steps when setting
	 * bi-directional or uni-directional TMU mode, get back to the TMU
	 * configurations in off mode. In case of additional failures in
	 * the functions below, ignore them since the caller shall already
	 * report a failure.
	 */
	tb_port_tmu_time_sync_disable(down);
	tb_port_tmu_time_sync_disable(up);

	switch (sw->tmu.mode_request) {
	case TB_SWITCH_TMU_MODE_UNI:
		tb_switch_tmu_rate_write(tb_switch_parent(sw),
					 TB_SWITCH_TMU_RATE_OFF);
		break;
	case TB_SWITCH_TMU_MODE_ENHANCED_UNI:
		disable_enhanced(up, down);
		break;
	default:
		break;
	}

	/* Always set the rate to 0 */
	tb_switch_tmu_rate_write(sw, TB_SWITCH_TMU_RATE_OFF);

	tb_switch_set_tmu_mode_params(sw, sw->tmu.rate);
	tb_port_tmu_unidirectional_disable(down);
	tb_port_tmu_unidirectional_disable(up);
}

/*
 * This function is called when the previous TMU mode was
 * TB_SWITCH_TMU_RATE_OFF.
 */
static int tb_switch_tmu_enable_bidirectional(struct tb_switch *sw)
{
	struct tb_port *up, *down;
	int ret;

	up = tb_upstream_port(sw);
	down = tb_switch_downstream_port(sw);

	ret = tb_port_tmu_unidirectional_disable(up);
	if (ret)
		return ret;

	ret = tb_port_tmu_unidirectional_disable(down);
	if (ret)
		goto out;

	ret = tb_switch_tmu_rate_write(sw, TB_SWITCH_TMU_RATE_HIFI);
	if (ret)
		goto out;

	ret = tb_port_tmu_time_sync_enable(up);
	if (ret)
		goto out;

	ret = tb_port_tmu_time_sync_enable(down);
	if (ret)
		goto out;

	return 0;

out:
	tb_switch_tmu_off(sw);
	return ret;
}

static int tb_switch_tmu_objection_mask(struct tb_switch *sw)
{
	u32 val;
	int ret;

	ret = tb_sw_read(sw, &val, TB_CFG_SWITCH,
			 sw->cap_vsec_tmu + TB_TIME_VSEC_3_CS_9, 1);
	if (ret)
		return ret;

	val &= ~TB_TIME_VSEC_3_CS_9_TMU_OBJ_MASK;

	return tb_sw_write(sw, &val, TB_CFG_SWITCH,
			   sw->cap_vsec_tmu + TB_TIME_VSEC_3_CS_9, 1);
}

static int tb_switch_tmu_unidirectional_enable(struct tb_switch *sw)
{
	struct tb_port *up = tb_upstream_port(sw);

	return tb_port_tmu_write(up, TMU_ADP_CS_6,
				 TMU_ADP_CS_6_DISABLE_TMU_OBJ_MASK,
				 TMU_ADP_CS_6_DISABLE_TMU_OBJ_MASK);
}

/*
 * This function is called when the previous TMU mode was
 * TB_SWITCH_TMU_RATE_OFF.
 */
static int tb_switch_tmu_enable_unidirectional(struct tb_switch *sw)
{
	struct tb_port *up, *down;
	int ret;

	up = tb_upstream_port(sw);
	down = tb_switch_downstream_port(sw);
	ret = tb_switch_tmu_rate_write(tb_switch_parent(sw),
				       sw->tmu.rate_request);
	if (ret)
		return ret;

	ret = tb_switch_set_tmu_mode_params(sw, sw->tmu.rate_request);
	if (ret)
		return ret;

	ret = tb_port_tmu_unidirectional_enable(up);
	if (ret)
		goto out;

	ret = tb_port_tmu_time_sync_enable(up);
	if (ret)
		goto out;

	ret = tb_port_tmu_unidirectional_enable(down);
	if (ret)
		goto out;

	ret = tb_port_tmu_time_sync_enable(down);
	if (ret)
		goto out;

	return 0;

out:
	tb_switch_tmu_off(sw);
	return ret;
}

/*
 * This function is called when the previous TMU mode was
 * TB_SWITCH_TMU_RATE_OFF.
 */
static int tb_switch_tmu_enable_enhanced(struct tb_switch *sw)
{
	struct tb_port *up, *down;
	int ret;

	/* Router specific parameters first */
	ret = tb_switch_set_tmu_mode_params(sw, sw->tmu.rate_request);
	if (ret)
		return ret;

	up = tb_upstream_port(sw);
	down = tb_switch_downstream_port(sw);

	ret = tb_port_set_tmu_mode_params(up, REPL_TIMEOUT, REPL_THRESHOLD,
					  REPL_N, DIRSWITCH_N);
	if (ret)
		goto out;

	ret = tb_port_tmu_rate_write(up, sw->tmu.rate_request);
	if (ret)
		goto out;

	ret = tb_port_tmu_enhanced_enable(up, true);
	if (ret)
		goto out;

	ret = tb_port_set_tmu_mode_params(down, REPL_TIMEOUT, REPL_THRESHOLD,
					  REPL_N, DIRSWITCH_N);
	if (ret)
		goto out;

	ret = tb_port_tmu_rate_write(down, sw->tmu.rate_request);
	if (ret)
		goto out;

	ret = tb_port_tmu_enhanced_enable(down, true);
	if (ret)
		goto out;

	return 0;

out:
	tb_switch_tmu_off(sw);
	return ret;
}

static void tb_switch_tmu_change_mode_prev(struct tb_switch *sw)
{
	struct tb_port *down, *up;

	down = tb_switch_downstream_port(sw);
	up = tb_upstream_port(sw);
	/*
	 * In case of any failure in one of the steps when change mode,
	 * get back to the TMU configurations in previous mode.
	 * In case of additional failures in the functions below,
	 * ignore them since the caller shall already report a failure.
	 */
	switch (sw->tmu.mode) {
	case TB_SWITCH_TMU_MODE_BI:
		tb_switch_tmu_rate_write(sw, sw->tmu.rate);
		break;

	case TB_SWITCH_TMU_MODE_UNI:
		tb_port_tmu_set_unidirectional(up, true);
		tb_switch_tmu_rate_write(tb_switch_parent(sw), sw->tmu.rate);
		break;

	case TB_SWITCH_TMU_MODE_ENHANCED_UNI:
		tb_port_set_tmu_mode_params(up, REPL_TIMEOUT, REPL_THRESHOLD,
					    REPL_N, DIRSWITCH_N);
		tb_port_tmu_enhanced_enable(up, true);
		tb_port_tmu_rate_write(up, sw->tmu.rate);
		break;
	}

	tb_switch_set_tmu_mode_params(sw, sw->tmu.rate);

	switch (sw->tmu.mode) {
	case TB_SWITCH_TMU_MODE_UNI:
		tb_port_tmu_set_unidirectional(down, true);
		break;

	case TB_SWITCH_TMU_MODE_ENHANCED_UNI:
		tb_port_set_tmu_mode_params(down, REPL_TIMEOUT, REPL_THRESHOLD,
					    REPL_N, DIRSWITCH_N);
		tb_port_tmu_enhanced_enable(down, true);
		tb_port_tmu_rate_write(down, sw->tmu.rate);
		break;

	default:
		break;
	}
}

static int tb_switch_tmu_change_mode(struct tb_switch *sw)
{
	struct tb_port *up, *down;
	int ret;

	up = tb_upstream_port(sw);
	down = tb_switch_downstream_port(sw);

	/* Disable any previous mode */
	ret = tb_port_tmu_set_unidirectional(up, false);
	if (ret)
		return ret;
	ret = tb_port_tmu_enhanced_enable(up, false);
	if (ret)
		return ret;
	ret = tb_port_tmu_rate_write(up, 0);
	if (ret)
		return ret;

	/* Program the new mode and the downstream router lane adapter */
	switch (sw->tmu.mode_request) {
	case TB_SWITCH_TMU_MODE_BI:
		ret = tb_switch_tmu_rate_write(sw, sw->tmu.rate_request);
		if (ret)
			goto out;
		break;

	case TB_SWITCH_TMU_MODE_UNI:
		ret = tb_port_tmu_set_unidirectional(up, true);
		if (ret)
			goto out;
		ret = tb_switch_tmu_rate_write(tb_switch_parent(sw),
					       sw->tmu.rate_request);
		if (ret)
			goto out;
		break;

	case TB_SWITCH_TMU_MODE_ENHANCED_UNI:
		ret = tb_port_set_tmu_mode_params(up, REPL_TIMEOUT,
						  REPL_THRESHOLD, REPL_N,
						  DIRSWITCH_N);
		if (ret)
			goto out;
		ret = tb_port_tmu_enhanced_enable(up, true);
		if (ret)
			goto out;
		ret = tb_port_tmu_rate_write(up, sw->tmu.rate_request);
		if (ret)
			goto out;
		break;

	default:
		return -EINVAL;
	}

	ret = tb_switch_set_tmu_mode_params(sw, sw->tmu.rate_request);
	if (ret)
		return ret;

	/* Program the upstream router downstream facing lane adapter */
	switch (sw->tmu.mode_request) {
	case TB_SWITCH_TMU_MODE_UNI:
		ret = tb_port_tmu_set_unidirectional(down, true);
		if (ret)
			goto out;
		break;

	case TB_SWITCH_TMU_MODE_ENHANCED_UNI:
		ret = tb_port_set_tmu_mode_params(down, REPL_TIMEOUT,
						  REPL_THRESHOLD, REPL_N,
						  DIRSWITCH_N);
		if (ret)
			goto out;
		ret = tb_port_tmu_enhanced_enable(down, true);
		if (ret)
			goto out;
		ret = tb_port_tmu_rate_write(down, sw->tmu.rate_request);
		if (ret)
			goto out;
		break;

	default:
		break;
	}

	ret = tb_port_tmu_time_sync_enable(down);
	if (ret)
		goto out;

	ret = tb_port_tmu_time_sync_enable(up);
	if (ret)
		goto out;

	return 0;

out:
	tb_switch_tmu_change_mode_prev(sw);
	return ret;
}

/**
 * tb_switch_tmu_enable() - Enable TMU on a router
 * @sw: Router whose TMU to enable
 *
 * Enables TMU of a router to be in uni-directional Normal/HiFi or
 * bi-directional HiFi mode. Calling tb_switch_tmu_configure() is
 * required before calling this function.
 */
int tb_switch_tmu_enable(struct tb_switch *sw)
{
	int ret;

	if (tb_switch_tmu_is_enabled(sw))
		return 0;

	if (tb_switch_is_titan_ridge(sw) &&
	    sw->tmu.mode_request == TB_SWITCH_TMU_MODE_UNI) {
		ret = tb_switch_tmu_objection_mask(sw);
		if (ret)
			return ret;

		ret = tb_switch_tmu_unidirectional_enable(sw);
		if (ret)
			return ret;
	}

	ret = tb_switch_tmu_set_time_disruption(sw, true);
	if (ret)
		return ret;

	if (tb_route(sw)) {
		/*
		 * The used mode changes are from OFF to
		 * HiFi-Uni/HiFi-BiDir/Normal-Uni or from Normal-Uni to
		 * HiFi-Uni.
		 */
		if (sw->tmu.rate == TB_SWITCH_TMU_RATE_OFF) {
			switch (sw->tmu.mode_request) {
			case TB_SWITCH_TMU_MODE_BI:
				ret = tb_switch_tmu_enable_bidirectional(sw);
				break;
			case TB_SWITCH_TMU_MODE_UNI:
				ret = tb_switch_tmu_enable_unidirectional(sw);
				break;
			case TB_SWITCH_TMU_MODE_ENHANCED_UNI:
				ret = tb_switch_tmu_enable_enhanced(sw);
				break;
			default:
				ret = -EINVAL;
				break;
			}
			if (ret)
				return ret;
		} else if (sw->tmu.rate == TB_SWITCH_TMU_RATE_NORMAL) {
			ret = tb_switch_tmu_change_mode(sw);
			if (ret)
				return ret;
		}
		sw->tmu.mode = sw->tmu.mode_request;
	} else {
		/*
		 * Host router port configurations are written as
		 * part of configurations for downstream port of the parent
		 * of the child node - see above.
		 * Here only the host router' rate configuration is written.
		 */
		ret = tb_switch_tmu_rate_write(sw, sw->tmu.rate_request);
		if (ret)
			return ret;
	}

	sw->tmu.rate = sw->tmu.rate_request;

	tb_sw_dbg(sw, "TMU: mode set to: %s\n", tb_switch_tmu_mode_name(sw));
	return tb_switch_tmu_set_time_disruption(sw, false);
}

/**
 * tb_switch_tmu_configure() - Configure the TMU rate and directionality
 * @sw: Router whose mode to change
 * @rate: Rate to configure Off/Normal/HiFi
 * @mode: Mode to configure
 *
 * Selects the rate of the TMU and directionality (uni-directional or
 * bi-directional). Must be called before tb_switch_tmu_enable().
 *
 * Returns %0 in success and negative errno otherwise.
 */
int tb_switch_tmu_configure(struct tb_switch *sw, enum tb_switch_tmu_rate rate,
			    enum tb_switch_tmu_mode mode)
{
	switch (mode) {
	case TB_SWITCH_TMU_MODE_BI:
		break;
	case TB_SWITCH_TMU_MODE_UNI:
		if (!sw->tmu.has_ucap)
			return -EINVAL;
		break;
	case TB_SWITCH_TMU_MODE_ENHANCED_UNI:
		if (!tb_switch_tmu_enhanced_is_supported(sw))
			return -EINVAL;
		/* Only support HiFi for now */
		if (rate != TB_SWITCH_TMU_RATE_HIFI)
			return -EINVAL;
		break;
	default:
		dev_warn(&sw->dev, "unsupported mode %u\n", mode);
		return -EINVAL;
	}

	sw->tmu.mode_request = mode;
	sw->tmu.rate_request = rate;
	return 0;
}
