// SPDX-License-Identifier: MIT
/*
 * Copyright © 2021 Intel Corporation
 */

#include "i915_reg.h"
#include "intel_cx0_phy.h"
#include "intel_cx0_reg_defs.h"
#include "intel_de.h"
#include "intel_display_types.h"
#include "intel_dp.h"
#include "intel_panel.h"
#include "intel_tc.h"

bool intel_is_c10phy(struct drm_i915_private *dev_priv, enum phy phy)
{
	if (IS_METEORLAKE(dev_priv) && (phy < PHY_C))
		return true;

	return false;
}

static void intel_cx0_bus_reset(struct drm_i915_private *i915, enum port port, int lane)
{
	enum phy phy = intel_port_to_phy(i915, port);

	/* Bring the phy to idle. */
	intel_de_write(i915, XELPDP_PORT_M2P_MSGBUS_CTL(port, lane),
		       XELPDP_PORT_M2P_TRANSACTION_RESET);

	/* Wait for Idle Clear. */
	if (intel_de_wait_for_clear(i915, XELPDP_PORT_M2P_MSGBUS_CTL(port, lane),
				    XELPDP_PORT_M2P_TRANSACTION_RESET,
				    XELPDP_MSGBUS_TIMEOUT_SLOW)) {
		drm_err_once(&i915->drm, "Failed to bring PHY %c to idle.\n", phy_name(phy));
		return;
	}

	intel_de_write(i915, XELPDP_PORT_P2M_MSGBUS_STATUS(port, lane), ~0);
}

static int intel_cx0_wait_for_ack(struct drm_i915_private *i915, enum port port, int lane, u32 *val)
{
	enum phy phy = intel_port_to_phy(i915, port);

	if (__intel_wait_for_register(&i915->uncore,
				      XELPDP_PORT_P2M_MSGBUS_STATUS(port, lane),
				      XELPDP_PORT_P2M_RESPONSE_READY,
				      XELPDP_PORT_P2M_RESPONSE_READY,
				      XELPDP_MSGBUS_TIMEOUT_FAST_US,
				      XELPDP_MSGBUS_TIMEOUT_SLOW, val)) {
		drm_dbg_kms(&i915->drm, "PHY %c Timeout waiting for message ACK. Status: 0x%x\n", phy_name(phy), *val);
		return -ETIMEDOUT;
	}

	return 0;
}

static int __intel_cx0_read(struct drm_i915_private *i915, enum port port,
			   int lane, u16 addr, u32 *val)
{
	enum phy phy = intel_port_to_phy(i915, port);
	int ack;

	/* Wait for pending transactions.*/
	if (intel_de_wait_for_clear(i915, XELPDP_PORT_M2P_MSGBUS_CTL(port, lane),
				    XELPDP_PORT_M2P_TRANSACTION_PENDING,
				    XELPDP_MSGBUS_TIMEOUT_SLOW)) {
		drm_dbg_kms(&i915->drm, "PHY %c Timeout waiting for previous transaction to complete. Reset the bus and retry.\n", phy_name(phy));
		intel_cx0_bus_reset(i915, port, lane);
		return -ETIMEDOUT;
	}

	/* Issue the read command. */
	intel_de_write(i915, XELPDP_PORT_M2P_MSGBUS_CTL(port, lane),
		       XELPDP_PORT_M2P_TRANSACTION_PENDING |
		       XELPDP_PORT_M2P_COMMAND_READ |
		       XELPDP_PORT_M2P_ADDRESS(addr));

	/* Wait for response ready. And read response.*/
	ack = intel_cx0_wait_for_ack(i915, port, lane, val);
	if (ack < 0) {
		intel_cx0_bus_reset(i915, port, lane);
		return ack;
	}

	/* Check for error. */
	if (*val & XELPDP_PORT_P2M_ERROR_SET) {
		drm_dbg_kms(&i915->drm, "PHY %c Error occurred during read command. Status: 0x%x\n", phy_name(phy), *val);
		intel_cx0_bus_reset(i915, port, lane);
		return -EINVAL;
	}

	/* Check for Read Ack. */
	if (REG_FIELD_GET(XELPDP_PORT_P2M_COMMAND_TYPE_MASK, *val) !=
			  XELPDP_PORT_P2M_COMMAND_READ_ACK) {
		drm_dbg_kms(&i915->drm, "PHY %c Not a Read response. MSGBUS Status: 0x%x.\n", phy_name(phy), *val);
		intel_cx0_bus_reset(i915, port, lane);
		return -EINVAL;
	}

	/* Clear Response Ready flag.*/
	intel_de_write(i915, XELPDP_PORT_P2M_MSGBUS_STATUS(port, lane), ~0);

	return REG_FIELD_GET(XELPDP_PORT_P2M_DATA_MASK, *val);
}

static u8 intel_cx0_read(struct drm_i915_private *i915, enum port port,
			 int lane, u16 addr)
{
	enum phy phy = intel_port_to_phy(i915, port);
	int i, status = 0;
	u32 val;

	for (i = 0; i < 3; i++) {
		status = __intel_cx0_read(i915, port, lane, addr, &val);

		if (status >= 0)
			break;
	}

	if (i == 3) {
		drm_err_once(&i915->drm, "PHY %c Read %04x failed after %d retries.\n", phy_name(phy), addr, i);
		return 0;
	}

	return status;
}

static int intel_cx0_wait_cwrite_ack(struct drm_i915_private *i915,
				      enum port port, int lane)
{
	enum phy phy = intel_port_to_phy(i915, port);
	int ack;
	u32 val = 0;

	/* Check for write ack. */
	ack = intel_cx0_wait_for_ack(i915, port, lane, &val);
	if (ack < 0)
		return ack;

	if ((REG_FIELD_GET(XELPDP_PORT_P2M_COMMAND_TYPE_MASK, val) !=
	     XELPDP_PORT_P2M_COMMAND_WRITE_ACK) || val & XELPDP_PORT_P2M_ERROR_SET) {
		drm_dbg_kms(&i915->drm, "PHY %c Unexpected ACK received. MSGBUS STATUS: 0x%x.\n", phy_name(phy), val);
		return -EINVAL;
	}

	return 0;
}

static int __intel_cx0_write_once(struct drm_i915_private *i915, enum port port,
				  int lane, u16 addr, u8 data, bool committed)
{
	enum phy phy = intel_port_to_phy(i915, port);

	/* Wait for pending transactions.*/
	if (intel_de_wait_for_clear(i915, XELPDP_PORT_M2P_MSGBUS_CTL(port, lane),
				    XELPDP_PORT_M2P_TRANSACTION_PENDING,
				    XELPDP_MSGBUS_TIMEOUT_SLOW)) {
		drm_dbg_kms(&i915->drm, "PHY %c Timeout waiting for previous transaction to complete. Reset the bus and retry.\n", phy_name(phy));
		intel_cx0_bus_reset(i915, port, lane);
		return -ETIMEDOUT;
	}

	/* Issue the write command. */
	intel_de_write(i915, XELPDP_PORT_M2P_MSGBUS_CTL(port, lane),
		       XELPDP_PORT_M2P_TRANSACTION_PENDING |
		       (committed ? XELPDP_PORT_M2P_COMMAND_WRITE_COMMITTED :
		       XELPDP_PORT_M2P_COMMAND_WRITE_UNCOMMITTED) |
		       XELPDP_PORT_M2P_DATA(data) |
		       XELPDP_PORT_M2P_ADDRESS(addr));

	/* Check for error. */
	if (committed) {
		if (intel_cx0_wait_cwrite_ack(i915, port, lane) < 0) {
			intel_cx0_bus_reset(i915, port, lane);
			return -EINVAL;
		}
	} else if ((intel_de_read(i915, XELPDP_PORT_P2M_MSGBUS_STATUS(port, lane)) &
			    XELPDP_PORT_P2M_ERROR_SET)) {
		drm_dbg_kms(&i915->drm, "PHY %c Error occurred during write command.\n", phy_name(phy));
		intel_cx0_bus_reset(i915, port, lane);
		return -EINVAL;
	}

	intel_de_write(i915, XELPDP_PORT_P2M_MSGBUS_STATUS(port, lane), ~0);

	return 0;
}

static void __intel_cx0_write(struct drm_i915_private *i915, enum port port,
			      int lane, u16 addr, u8 data, bool committed)
{
	enum phy phy = intel_port_to_phy(i915, port);
	int i, status;

	for (i = 0; i < 3; i++) {
		status = __intel_cx0_write_once(i915, port, lane, addr, data, committed);

		if (status == 0)
			break;
	}

	if (i == 3) {
		drm_err_once(&i915->drm, "PHY %c Write %04x failed after %d retries.\n", phy_name(phy), addr, i);
		return;
	}
}

static void intel_cx0_write(struct drm_i915_private *i915, enum port port,
			    int lane, u16 addr, u8 data, bool committed)
{
	if (lane == INTEL_CX0_BOTH_LANES) {
		__intel_cx0_write(i915, port, INTEL_CX0_LANE0, addr, data, committed);
		__intel_cx0_write(i915, port, INTEL_CX0_LANE1, addr, data, committed);
	} else {
		__intel_cx0_write(i915, port, lane, addr, data, committed);
	}
}

static void __intel_cx0_rmw(struct drm_i915_private *i915, enum port port,
			    int lane, u16 addr, u8 clear, u8 set, bool committed)
{
	u8 old, val;

	old = intel_cx0_read(i915, port, lane, addr);
	val = (old & ~clear) | set;

	if (val != old)
		intel_cx0_write(i915, port, lane, addr, val, committed);
}

static void intel_cx0_rmw(struct drm_i915_private *i915, enum port port,
			  int lane, u16 addr, u8 clear, u8 set, bool committed)
{
	if (lane == INTEL_CX0_BOTH_LANES) {
		__intel_cx0_rmw(i915, port, INTEL_CX0_LANE0, addr, clear, set, committed);
		__intel_cx0_rmw(i915, port, INTEL_CX0_LANE1, addr, clear, set, committed);
	} else {
		__intel_cx0_rmw(i915, port, lane, addr, clear, set, committed);
	}
}

/*
 * Basic DP link rates with 38.4 MHz reference clock.
 * Note: The tables below are with SSC. In non-ssc
 * registers 0xC04 to 0xC08(pll[4] to pll[8]) will be
 * programmed 0.
 */

static const struct intel_c10mpllb_state mtl_c10_dp_rbr = {
	.clock = 162000,
	.pll[0] = 0xB4,
	.pll[1] = 0,
	.pll[2] = 0x30,
	.pll[3] = 0x1,
	.pll[4] = 0x26,
	.pll[5] = 0x0C,
	.pll[6] = 0x98,
	.pll[7] = 0x46,
	.pll[8] = 0x1,
	.pll[9] = 0x1,
	.pll[10] = 0,
	.pll[11] = 0,
	.pll[12] = 0xC0,
	.pll[13] = 0,
	.pll[14] = 0,
	.pll[15] = 0x2,
	.pll[16] = 0x84,
	.pll[17] = 0x4F,
	.pll[18] = 0xE5,
	.pll[19] = 0x23,
};

static const struct intel_c10mpllb_state mtl_c10_edp_r216 = {
	.clock = 216000,
	.pll[0] = 0x4,
	.pll[1] = 0,
	.pll[2] = 0xA2,
	.pll[3] = 0x1,
	.pll[4] = 0x33,
	.pll[5] = 0x10,
	.pll[6] = 0x75,
	.pll[7] = 0xB3,
	.pll[8] = 0x1,
	.pll[9] = 0x1,
	.pll[10] = 0,
	.pll[11] = 0,
	.pll[12] = 0,
	.pll[13] = 0,
	.pll[14] = 0,
	.pll[15] = 0x2,
	.pll[16] = 0x85,
	.pll[17] = 0x0F,
	.pll[18] = 0xE6,
	.pll[19] = 0x23,
};

static const struct intel_c10mpllb_state mtl_c10_edp_r243 = {
	.clock = 243000,
	.pll[0] = 0x34,
	.pll[1] = 0,
	.pll[2] = 0xDA,
	.pll[3] = 0x1,
	.pll[4] = 0x39,
	.pll[5] = 0x12,
	.pll[6] = 0xE3,
	.pll[7] = 0xE9,
	.pll[8] = 0x1,
	.pll[9] = 0x1,
	.pll[10] = 0,
	.pll[11] = 0,
	.pll[12] = 0x20,
	.pll[13] = 0,
	.pll[14] = 0,
	.pll[15] = 0x2,
	.pll[16] = 0x85,
	.pll[17] = 0x8F,
	.pll[18] = 0xE6,
	.pll[19] = 0x23,
};

static const struct intel_c10mpllb_state mtl_c10_dp_hbr1 = {
	.clock = 270000,
	.pll[0] = 0xF4,
	.pll[1] = 0,
	.pll[2] = 0xF8,
	.pll[3] = 0x0,
	.pll[4] = 0x20,
	.pll[5] = 0x0A,
	.pll[6] = 0x29,
	.pll[7] = 0x10,
	.pll[8] = 0x1,   /* Verify */
	.pll[9] = 0x1,
	.pll[10] = 0,
	.pll[11] = 0,
	.pll[12] = 0xA0,
	.pll[13] = 0,
	.pll[14] = 0,
	.pll[15] = 0x1,
	.pll[16] = 0x84,
	.pll[17] = 0x4F,
	.pll[18] = 0xE5,
	.pll[19] = 0x23,
};

static const struct intel_c10mpllb_state mtl_c10_edp_r324 = {
	.clock = 324000,
	.pll[0] = 0xB4,
	.pll[1] = 0,
	.pll[2] = 0x30,
	.pll[3] = 0x1,
	.pll[4] = 0x26,
	.pll[5] = 0x0C,
	.pll[6] = 0x98,
	.pll[7] = 0x46,
	.pll[8] = 0x1,
	.pll[9] = 0x1,
	.pll[10] = 0,
	.pll[11] = 0,
	.pll[12] = 0xC0,
	.pll[13] = 0,
	.pll[14] = 0,
	.pll[15] = 0x1,
	.pll[16] = 0x85,
	.pll[17] = 0x4F,
	.pll[18] = 0xE6,
	.pll[19] = 0x23,
};

static const struct intel_c10mpllb_state mtl_c10_edp_r432 = {
	.clock = 432000,
	.pll[0] = 0x4,
	.pll[1] = 0,
	.pll[2] = 0xA2,
	.pll[3] = 0x1,
	.pll[4] = 0x33,
	.pll[5] = 0x10,
	.pll[6] = 0x75,
	.pll[7] = 0xB3,
	.pll[8] = 0x1,
	.pll[9] = 0x1,
	.pll[10] = 0,
	.pll[11] = 0,
	.pll[12] = 0,
	.pll[13] = 0,
	.pll[14] = 0,
	.pll[15] = 0x1,
	.pll[16] = 0x85,
	.pll[17] = 0x0F,
	.pll[18] = 0xE6,
	.pll[19] = 0x23,
};

static const struct intel_c10mpllb_state mtl_c10_dp_hbr2 = {
	.clock = 540000,
	.pll[0] = 0xF4,
	.pll[1] = 0,
	.pll[2] = 0xF8,
	.pll[3] = 0,
	.pll[4] = 0x20,
	.pll[5] = 0x0A,
	.pll[6] = 0x29,
	.pll[7] = 0x10,
	.pll[8] = 0x1,
	.pll[9] = 0x1,
	.pll[10] = 0,
	.pll[11] = 0,
	.pll[12] = 0xA0,
	.pll[13] = 0,
	.pll[14] = 0,
	.pll[15] = 0,
	.pll[16] = 0x84,
	.pll[17] = 0x4F,
	.pll[18] = 0xE5,
	.pll[19] = 0x23,
};

static const struct intel_c10mpllb_state mtl_c10_edp_r675 = {
	.clock = 675000,
	.pll[0] = 0xB4,
	.pll[1] = 0,
	.pll[2] = 0x3E,
	.pll[3] = 0x1,
	.pll[4] = 0xA8,
	.pll[5] = 0x0C,
	.pll[6] = 0x33,
	.pll[7] = 0x54,
	.pll[8] = 0x1,
	.pll[9] = 0x1,
	.pll[10] = 0,
	.pll[11] = 0,
	.pll[12] = 0xC8,
	.pll[13] = 0,
	.pll[14] = 0,
	.pll[15] = 0,
	.pll[16] = 0x85,
	.pll[17] = 0x8F,
	.pll[18] = 0xE6,
	.pll[19] = 0x23,
};

static const struct intel_c10mpllb_state mtl_c10_dp_hbr3 = {
	.clock = 810000,
	.pll[0] = 0x34,
	.pll[1] = 0,
	.pll[2] = 0x84,
	.pll[3] = 0x1,
	.pll[4] = 0x30,
	.pll[5] = 0x0F,
	.pll[6] = 0x3D,
	.pll[7] = 0x98,
	.pll[8] = 0x1,
	.pll[9] = 0x1,
	.pll[10] = 0,
	.pll[11] = 0,
	.pll[12] = 0xF0,
	.pll[13] = 0,
	.pll[14] = 0,
	.pll[15] = 0,
	.pll[16] = 0x84,
	.pll[17] = 0x0F,
	.pll[18] = 0xE5,
	.pll[19] = 0x23,
};

static const struct intel_c10mpllb_state * const mtl_c10_dp_tables[] = {
	&mtl_c10_dp_rbr,
	&mtl_c10_dp_hbr1,
	&mtl_c10_dp_hbr2,
	&mtl_c10_dp_hbr3,
	NULL,
};

static const struct intel_c10mpllb_state * const mtl_c10_edp_tables[] = {
	&mtl_c10_dp_rbr,
	&mtl_c10_edp_r216,
	&mtl_c10_edp_r243,
	&mtl_c10_dp_hbr1,
	&mtl_c10_edp_r324,
	&mtl_c10_edp_r432,
	&mtl_c10_dp_hbr2,
	&mtl_c10_edp_r675,
	&mtl_c10_dp_hbr3,
	NULL,
};

static const struct intel_c10mpllb_state * const *
intel_c10_mpllb_tables_get(struct intel_crtc_state *crtc_state,
			   struct intel_encoder *encoder)
{
	if (intel_crtc_has_dp_encoder(crtc_state)) {
		if (intel_crtc_has_type(crtc_state, INTEL_OUTPUT_EDP))
			return mtl_c10_edp_tables;
		else
			return mtl_c10_dp_tables;
	}

	/* TODO: Add HDMI Support */
	MISSING_CASE(encoder->type);
	return NULL;
}

static int intel_c10mpllb_calc_state(struct intel_crtc_state *crtc_state,
				     struct intel_encoder *encoder)
{
	const struct intel_c10mpllb_state * const *tables;
	int i;

	tables = intel_c10_mpllb_tables_get(crtc_state, encoder);
	if (!tables)
		return -EINVAL;

	for (i = 0; tables[i]; i++) {
		if (crtc_state->port_clock <= tables[i]->clock) {
			crtc_state->c10mpllb_state = *tables[i];
			return 0;
		}
	}

	return -EINVAL;
}

int intel_cx0mpllb_calc_state(struct intel_crtc_state *crtc_state,
			      struct intel_encoder *encoder)
{
	struct drm_i915_private *i915 = to_i915(encoder->base.dev);
	enum phy phy = intel_port_to_phy(i915, encoder->port);

	drm_WARN_ON(&i915->drm, !intel_is_c10phy(i915, phy));

	return intel_c10mpllb_calc_state(crtc_state, encoder);
}

void intel_c10mpllb_readout_hw_state(struct intel_encoder *encoder,
				     struct intel_c10mpllb_state *pll_state)
{
	struct drm_i915_private *i915 = to_i915(encoder->base.dev);
	struct intel_digital_port *dig_port = enc_to_dig_port(encoder);
	bool lane_reversal = dig_port->saved_port_bits & DDI_BUF_PORT_REVERSAL;
	u8 lane = lane_reversal ? INTEL_CX0_LANE1 :
				  INTEL_CX0_LANE0;
	enum phy phy = intel_port_to_phy(i915, encoder->port);
	int i;
	u8 cmn, tx0;

	/*
	 * According to C10 VDR Register programming Sequence we need
	 * to do this to read PHY internal registers from MsgBus.
	 */
	intel_cx0_rmw(i915, encoder->port, lane, PHY_C10_VDR_CONTROL(1), 0,
		      C10_VDR_CTRL_MSGBUS_ACCESS, MB_WRITE_COMMITTED);

	for (i = 0; i < ARRAY_SIZE(pll_state->pll); i++)
		pll_state->pll[i] = intel_cx0_read(i915, encoder->port, lane,
						   PHY_C10_VDR_PLL(i));

	cmn = intel_cx0_read(i915, encoder->port, lane, PHY_C10_VDR_CMN(0));
	tx0 = intel_cx0_read(i915, encoder->port, lane, PHY_C10_VDR_TX(0));

	if (tx0 != C10_TX0_VAL || cmn != C10_CMN0_DP_VAL)
		drm_warn(&i915->drm, "Unexpected tx: %x or cmn: %x for phy: %c.\n",
			 tx0, cmn, phy_name(phy));
}

static void intel_c10_pll_program(struct drm_i915_private *i915,
				  const struct intel_crtc_state *crtc_state,
				  struct intel_encoder *encoder)
{
	const struct intel_c10mpllb_state *pll_state = &crtc_state->c10mpllb_state;
	struct intel_digital_port *dig_port = enc_to_dig_port(encoder);
	bool lane_reversal = dig_port->saved_port_bits & DDI_BUF_PORT_REVERSAL;
	u8 master_lane = lane_reversal ? INTEL_CX0_LANE1 :
					 INTEL_CX0_LANE0;
	u8 follower_lane = lane_reversal ? INTEL_CX0_LANE0 :
					   INTEL_CX0_LANE1;

	int i;
	struct intel_dp *intel_dp;
	bool use_ssc = false;
	u8 cmn0 = 0;

	if (intel_crtc_has_dp_encoder(crtc_state)) {
		intel_dp = enc_to_intel_dp(encoder);
		use_ssc = (intel_dp->dpcd[DP_MAX_DOWNSPREAD] &
			  DP_MAX_DOWNSPREAD_0_5);

		if (!intel_panel_use_ssc(i915))
			use_ssc = false;

		cmn0 = C10_CMN0_DP_VAL;
	}

	intel_cx0_write(i915, encoder->port, INTEL_CX0_BOTH_LANES, PHY_C10_VDR_CONTROL(1),
			C10_VDR_CTRL_MSGBUS_ACCESS, MB_WRITE_COMMITTED);
	/* Custom width needs to be programmed to 0 for both the phy lanes */
	intel_cx0_rmw(i915, encoder->port, INTEL_CX0_BOTH_LANES,
		      PHY_C10_VDR_CUSTOM_WIDTH, 0x3, 0, MB_WRITE_COMMITTED);
	intel_cx0_rmw(i915, encoder->port, follower_lane, PHY_C10_VDR_CONTROL(1),
		      C10_VDR_CTRL_MASTER_LANE, C10_VDR_CTRL_UPDATE_CFG,
		      MB_WRITE_COMMITTED);

	/* Program the pll values only for the master lane */
	for (i = 0; i < ARRAY_SIZE(pll_state->pll); i++)
		/* If not using ssc pll[4] through pll[8] must be 0*/
		intel_cx0_write(i915, encoder->port, master_lane, PHY_C10_VDR_PLL(i),
				(!use_ssc && (i > 3 && i < 9)) ? 0 : pll_state->pll[i],
				(i % 4) ? MB_WRITE_UNCOMMITTED : MB_WRITE_COMMITTED);

	intel_cx0_write(i915, encoder->port, master_lane, PHY_C10_VDR_CMN(0), cmn0, MB_WRITE_COMMITTED);
	intel_cx0_write(i915, encoder->port, master_lane, PHY_C10_VDR_TX(0), C10_TX0_VAL, MB_WRITE_COMMITTED);
	intel_cx0_rmw(i915, encoder->port, master_lane, PHY_C10_VDR_CONTROL(1),
		      C10_VDR_CTRL_MSGBUS_ACCESS, C10_VDR_CTRL_MASTER_LANE |
		      C10_VDR_CTRL_UPDATE_CFG, MB_WRITE_COMMITTED);
}

void intel_c10mpllb_dump_hw_state(struct drm_i915_private *dev_priv,
				  const struct intel_c10mpllb_state *hw_state)
{
	bool fracen;
	int i;
	unsigned int frac_quot = 0, frac_rem = 0, frac_den = 1;
	unsigned int multiplier, tx_clk_div;

	fracen = hw_state->pll[0] & C10_PLL0_FRACEN;
	drm_dbg_kms(&dev_priv->drm, "c10pll_hw_state: fracen: %s, ",
		    str_yes_no(fracen));

	if (fracen) {
		frac_quot = hw_state->pll[12] << 8 | hw_state->pll[11];
		frac_rem =  hw_state->pll[14] << 8 | hw_state->pll[13];
		frac_den =  hw_state->pll[10] << 8 | hw_state->pll[9];
		drm_dbg_kms(&dev_priv->drm, "quot: %u, rem: %u, den: %u,\n",
			    frac_quot, frac_rem, frac_den);
	}

	multiplier = (REG_FIELD_GET8(C10_PLL3_MULTIPLIERH_MASK, hw_state->pll[3]) << 8 |
		      hw_state->pll[2]) / 2 + 16;
	tx_clk_div = REG_FIELD_GET8(C10_PLL15_TXCLKDIV_MASK, hw_state->pll[15]);
	drm_dbg_kms(&dev_priv->drm,
		    "multiplier: %u, tx_clk_div: %u.\n", multiplier, tx_clk_div);

	drm_dbg_kms(&dev_priv->drm, "c10pll_rawhw_state:");

	for (i = 0; i < ARRAY_SIZE(hw_state->pll); i = i + 4)
		drm_dbg_kms(&dev_priv->drm, "pll[%d] = 0x%x, pll[%d] = 0x%x, pll[%d] = 0x%x, pll[%d] = 0x%x\n",
			    i, hw_state->pll[i], i + 1, hw_state->pll[i + 1],
			    i + 2, hw_state->pll[i + 2], i + 3, hw_state->pll[i + 3]);
}

int intel_c10mpllb_calc_port_clock(struct intel_encoder *encoder,
				   const struct intel_c10mpllb_state *pll_state)
{
	unsigned int frac_quot = 0, frac_rem = 0, frac_den = 1;
	unsigned int multiplier, tx_clk_div, refclk = 38400;

	if (pll_state->pll[0] & C10_PLL0_FRACEN) {
		frac_quot = pll_state->pll[12] << 8 | pll_state->pll[11];
		frac_rem =  pll_state->pll[14] << 8 | pll_state->pll[13];
		frac_den =  pll_state->pll[10] << 8 | pll_state->pll[9];
	}

	multiplier = (REG_FIELD_GET8(C10_PLL3_MULTIPLIERH_MASK, pll_state->pll[3]) << 8 |
		      pll_state->pll[2]) / 2 + 16;

	tx_clk_div = REG_FIELD_GET8(C10_PLL15_TXCLKDIV_MASK, pll_state->pll[15]);

	return DIV_ROUND_CLOSEST_ULL(mul_u32_u32(refclk, (multiplier << 16) + frac_quot) +
				     DIV_ROUND_CLOSEST(refclk * frac_rem, frac_den),
				     10 << (tx_clk_div + 16));
}

static void intel_program_port_clock_ctl(struct intel_encoder *encoder,
					 const struct intel_crtc_state *crtc_state,
					 bool lane_reversal)
{
	struct drm_i915_private *i915 = to_i915(encoder->base.dev);
	struct intel_dp *intel_dp;
	bool ssc_enabled;
	u32 val = 0;

	intel_de_rmw(i915, XELPDP_PORT_BUF_CTL1(encoder->port), XELPDP_PORT_REVERSAL,
		     lane_reversal ? XELPDP_PORT_REVERSAL : 0);

	if (lane_reversal)
		val |= XELPDP_LANE1_PHY_CLOCK_SELECT;

	val |= XELPDP_FORWARD_CLOCK_UNGATE;
	val |= XELPDP_DDI_CLOCK_SELECT(XELPDP_DDI_CLOCK_SELECT_MAXPCLK);

	if (intel_crtc_has_dp_encoder(crtc_state)) {
		intel_dp = enc_to_intel_dp(encoder);
		ssc_enabled = intel_dp->dpcd[DP_MAX_DOWNSPREAD] &
			      DP_MAX_DOWNSPREAD_0_5;

		if (!intel_panel_use_ssc(i915))
			ssc_enabled = false;

		/* TODO: DP2.0 10G and 20G rates enable MPLLA*/
		val |= ssc_enabled ? XELPDP_SSC_ENABLE_PLLB : 0;
	}
	intel_de_rmw(i915, XELPDP_PORT_CLOCK_CTL(encoder->port),
		     XELPDP_LANE1_PHY_CLOCK_SELECT |
		     XELPDP_FORWARD_CLOCK_UNGATE |
		     XELPDP_DDI_CLOCK_SELECT_MASK |
		     XELPDP_SSC_ENABLE_PLLB, val);
}

static u32 intel_cx0_get_powerdown_update(u8 lane)
{
	if (lane == INTEL_CX0_LANE0)
		return XELPDP_LANE0_POWERDOWN_UPDATE;
	else if (lane == INTEL_CX0_LANE1)
		return XELPDP_LANE1_POWERDOWN_UPDATE;
	else
		return XELPDP_LANE0_POWERDOWN_UPDATE |
		       XELPDP_LANE1_POWERDOWN_UPDATE;
}

static u32 intel_cx0_get_powerdown_state(u8 lane, u8 state)
{
	if (lane == INTEL_CX0_LANE0)
		return XELPDP_LANE0_POWERDOWN_NEW_STATE(state);
	else if (lane == INTEL_CX0_LANE1)
		return XELPDP_LANE1_POWERDOWN_NEW_STATE(state);
	else
		return XELPDP_LANE0_POWERDOWN_NEW_STATE(state) |
		       XELPDP_LANE1_POWERDOWN_NEW_STATE(state);
}

static void intel_cx0_powerdown_change_sequence(struct drm_i915_private *i915,
						enum port port,
						u8 lane, u8 state)
{
	enum phy phy = intel_port_to_phy(i915, port);

	intel_de_rmw(i915, XELPDP_PORT_BUF_CTL2(port),
		     XELPDP_LANE0_POWERDOWN_NEW_STATE_MASK | XELPDP_LANE1_POWERDOWN_NEW_STATE_MASK,
		     intel_cx0_get_powerdown_state(lane, state));
	intel_de_rmw(i915, XELPDP_PORT_BUF_CTL2(port),
		     XELPDP_LANE0_POWERDOWN_UPDATE | XELPDP_LANE1_POWERDOWN_UPDATE,
		     intel_cx0_get_powerdown_update(lane));

	/* Update Timeout Value */
	if (__intel_wait_for_register(&i915->uncore, XELPDP_PORT_BUF_CTL2(port),
				      intel_cx0_get_powerdown_update(lane), 0,
				      XELPDP_PORT_POWERDOWN_UPDATE_TIMEOUT_US, 0, NULL))
		drm_warn(&i915->drm, "PHY %c failed to bring out of Lane reset after %dus.\n",
			 phy_name(phy), XELPDP_PORT_RESET_START_TIMEOUT_US);
}

static void intel_cx0_setup_powerdown(struct drm_i915_private *i915, enum port port)
{
	intel_de_rmw(i915, XELPDP_PORT_BUF_CTL2(port),
		     XELPDP_POWER_STATE_READY_MASK,
		     XELPDP_POWER_STATE_READY(CX0_P2_STATE_READY));
	intel_de_rmw(i915, XELPDP_PORT_BUF_CTL3(port),
		     XELPDP_POWER_STATE_ACTIVE_MASK |
		     XELPDP_PLL_LANE_STAGGERING_DELAY_MASK,
		     XELPDP_POWER_STATE_ACTIVE(CX0_P0_STATE_ACTIVE) |
		     XELPDP_PLL_LANE_STAGGERING_DELAY(0));
}

static u32 intel_cx0_get_pclk_refclk_request(u8 lane)
{
	if (lane == INTEL_CX0_LANE0)
		return XELPDP_LANE0_PCLK_REFCLK_REQUEST;
	else if (lane == INTEL_CX0_LANE1)
		return XELPDP_LANE1_PCLK_REFCLK_REQUEST;
	else
		return XELPDP_LANE0_PCLK_REFCLK_REQUEST |
		       XELPDP_LANE1_PCLK_REFCLK_REQUEST;
}

static u32 intel_cx0_get_pclk_refclk_ack(u8 lane)
{
	if (lane == INTEL_CX0_LANE0)
		return XELPDP_LANE0_PCLK_REFCLK_ACK;
	else if (lane == INTEL_CX0_LANE1)
		return XELPDP_LANE1_PCLK_REFCLK_ACK;
	else
		return XELPDP_LANE0_PCLK_REFCLK_ACK |
		       XELPDP_LANE1_PCLK_REFCLK_ACK;
}

/* FIXME: Some Type-C cases need not reset both the lanes. Handle those cases. */
static void intel_cx0_phy_lane_reset(struct drm_i915_private *i915, enum port port,
				     bool lane_reversal)
{
	enum phy phy = intel_port_to_phy(i915, port);
	u8 lane = lane_reversal ? INTEL_CX0_LANE1 :
				  INTEL_CX0_LANE0;

	if (__intel_wait_for_register(&i915->uncore, XELPDP_PORT_BUF_CTL1(port),
				      XELPDP_PORT_BUF_SOC_PHY_READY,
				      XELPDP_PORT_BUF_SOC_PHY_READY,
				      XELPDP_PORT_BUF_SOC_READY_TIMEOUT_US, 0, NULL))
		drm_warn(&i915->drm, "PHY %c failed to bring out of SOC reset after %dus.\n",
			 phy_name(phy), XELPDP_PORT_BUF_SOC_READY_TIMEOUT_US);

	intel_de_rmw(i915, XELPDP_PORT_BUF_CTL2(port),
		     XELPDP_LANE0_PIPE_RESET | XELPDP_LANE1_PIPE_RESET,
		     XELPDP_LANE0_PIPE_RESET | XELPDP_LANE1_PIPE_RESET);

	if (__intel_wait_for_register(&i915->uncore, XELPDP_PORT_BUF_CTL2(port),
				      XELPDP_LANE0_PHY_CURRENT_STATUS | XELPDP_LANE1_PHY_CURRENT_STATUS,
				      XELPDP_LANE0_PHY_CURRENT_STATUS | XELPDP_LANE1_PHY_CURRENT_STATUS,
				      XELPDP_PORT_RESET_START_TIMEOUT_US, 0, NULL))
		drm_warn(&i915->drm, "PHY %c failed to bring out of Lane reset after %dus.\n",
			 phy_name(phy), XELPDP_PORT_RESET_START_TIMEOUT_US);

	intel_de_rmw(i915, XELPDP_PORT_CLOCK_CTL(port),
		     intel_cx0_get_pclk_refclk_request(INTEL_CX0_BOTH_LANES),
		     intel_cx0_get_pclk_refclk_request(lane));

	if (__intel_wait_for_register(&i915->uncore, XELPDP_PORT_CLOCK_CTL(port),
				      intel_cx0_get_pclk_refclk_ack(INTEL_CX0_BOTH_LANES),
				      intel_cx0_get_pclk_refclk_ack(lane),
				      XELPDP_REFCLK_ENABLE_TIMEOUT_US, 0, NULL))
		drm_warn(&i915->drm, "PHY %c failed to request refclk after %dus.\n",
			 phy_name(phy), XELPDP_REFCLK_ENABLE_TIMEOUT_US);

	intel_cx0_powerdown_change_sequence(i915, port, INTEL_CX0_BOTH_LANES,
					    CX0_P2_STATE_RESET);
	intel_cx0_setup_powerdown(i915, port);

	intel_de_rmw(i915, XELPDP_PORT_BUF_CTL2(port),
		     XELPDP_LANE0_PIPE_RESET | XELPDP_LANE1_PIPE_RESET, 0);

	if (intel_de_wait_for_clear(i915, XELPDP_PORT_BUF_CTL2(port),
				    XELPDP_LANE0_PHY_CURRENT_STATUS |
				    XELPDP_LANE1_PHY_CURRENT_STATUS,
				    XELPDP_PORT_RESET_END_TIMEOUT))
		drm_warn(&i915->drm, "PHY %c failed to bring out of Lane reset after %dms.\n",
			 phy_name(phy), XELPDP_PORT_RESET_END_TIMEOUT);
}

static void intel_c10_program_phy_lane(struct drm_i915_private *i915,
				       struct intel_encoder *encoder, int lane_count,
				       bool lane_reversal)
{
	u8 l0t1, l0t2, l1t1, l1t2;
	bool dp_alt_mode = intel_tc_port_in_dp_alt_mode(enc_to_dig_port(encoder));
	enum port port = encoder->port;

	intel_cx0_rmw(i915, port, INTEL_CX0_BOTH_LANES, PHY_C10_VDR_CONTROL(1),
		      C10_VDR_CTRL_MSGBUS_ACCESS, C10_VDR_CTRL_MSGBUS_ACCESS,
		      MB_WRITE_COMMITTED);

	l0t1 = intel_cx0_read(i915, port, 0, PHY_CX0_TX_CONTROL(1, 2));
	l0t2 = intel_cx0_read(i915, port, 0, PHY_CX0_TX_CONTROL(2, 2));
	l1t1 = intel_cx0_read(i915, port, 1, PHY_CX0_TX_CONTROL(1, 2));
	l1t2 = intel_cx0_read(i915, port, 1, PHY_CX0_TX_CONTROL(2, 2));

	if (lane_reversal) {
		switch (lane_count) {
		case 1:
			/* Disable MLs 1(lane0), 2(lane0), 3(lane1) */
			intel_cx0_write(i915, port, 1, PHY_CX0_TX_CONTROL(1, 2),
					l1t1 | CONTROL2_DISABLE_SINGLE_TX,
					MB_WRITE_COMMITTED);
			fallthrough;
		case 2:
			/* Disable MLs 1(lane0), 2(lane0) */
			intel_cx0_write(i915, port, 0, PHY_CX0_TX_CONTROL(2, 2),
					l0t2 | CONTROL2_DISABLE_SINGLE_TX,
					MB_WRITE_COMMITTED);
			fallthrough;
		case 3:
			/* Disable MLs 1(lane0) */
			intel_cx0_write(i915, port, 0, PHY_CX0_TX_CONTROL(1, 2),
					l0t1 | CONTROL2_DISABLE_SINGLE_TX,
					MB_WRITE_COMMITTED);
			break;
		}
	} else {
		switch (lane_count) {
		case 1:
			if (dp_alt_mode) {
				/* Disable MLs 1(lane0), 3(lane1), 4(lane1) */
				intel_cx0_write(i915, port, 0, PHY_CX0_TX_CONTROL(1, 2),
						l0t1 | CONTROL2_DISABLE_SINGLE_TX,
						MB_WRITE_COMMITTED);
			} else {
				/* Disable MLs 2(lane0), 3(lane1), 4(lane1) */
				intel_cx0_write(i915, port, 0, PHY_CX0_TX_CONTROL(2, 2),
						l0t2 | CONTROL2_DISABLE_SINGLE_TX,
						MB_WRITE_COMMITTED);
			}
			fallthrough;
		case 2:
			/* Disable MLs 3(lane1), 4(lane1) */
			intel_cx0_write(i915, port, 1, PHY_CX0_TX_CONTROL(1, 2),
					l1t1 | CONTROL2_DISABLE_SINGLE_TX,
					MB_WRITE_COMMITTED);
			fallthrough;
		case 3:
			/* Disable MLs 4(lane1) */
			intel_cx0_write(i915, port, 1, PHY_CX0_TX_CONTROL(2, 2),
					l1t2 | CONTROL2_DISABLE_SINGLE_TX,
					MB_WRITE_COMMITTED);
			break;
		}
	}

	if (intel_is_c10phy(i915, intel_port_to_phy(i915, port))) {
		intel_cx0_rmw(i915, port, 1, PHY_C10_VDR_CONTROL(1),
			      C10_VDR_CTRL_UPDATE_CFG | C10_VDR_CTRL_MSGBUS_ACCESS,
			      C10_VDR_CTRL_UPDATE_CFG, MB_WRITE_COMMITTED);
		intel_cx0_rmw(i915, port, 0, PHY_C10_VDR_CONTROL(1),
			      C10_VDR_CTRL_UPDATE_CFG | C10_VDR_CTRL_MSGBUS_ACCESS,
			      C10_VDR_CTRL_MASTER_LANE | C10_VDR_CTRL_UPDATE_CFG, MB_WRITE_COMMITTED);
	}
}

static u32 intel_cx0_get_pclk_pll_request(u8 lane)
{
	if (lane == INTEL_CX0_LANE0)
		return XELPDP_LANE0_PCLK_PLL_REQUEST;
	else if (lane == INTEL_CX0_LANE1)
		return XELPDP_LANE1_PCLK_PLL_REQUEST;
	else
		return XELPDP_LANE0_PCLK_PLL_REQUEST |
		       XELPDP_LANE1_PCLK_PLL_REQUEST;
}

static u32 intel_cx0_get_pclk_pll_ack(u8 lane)
{
	if (lane == INTEL_CX0_LANE0)
		return XELPDP_LANE0_PCLK_PLL_ACK;
	else if (lane == INTEL_CX0_LANE1)
		return XELPDP_LANE1_PCLK_PLL_ACK;
	else
		return XELPDP_LANE0_PCLK_PLL_ACK |
		       XELPDP_LANE1_PCLK_PLL_ACK;
}

static void intel_c10pll_enable(struct intel_encoder *encoder,
				const struct intel_crtc_state *crtc_state)
{
	struct drm_i915_private *i915 = to_i915(encoder->base.dev);
	enum phy phy = intel_port_to_phy(i915, encoder->port);
	struct intel_digital_port *dig_port = enc_to_dig_port(encoder);
	bool lane_reversal = dig_port->saved_port_bits & DDI_BUF_PORT_REVERSAL;
	u8 maxpclk_lane = lane_reversal ? INTEL_CX0_LANE1 :
					  INTEL_CX0_LANE0;

	/*
	 * 1. Program PORT_CLOCK_CTL REGISTER to configure
	 * clock muxes, gating and SSC
	 */
	intel_program_port_clock_ctl(encoder, crtc_state, lane_reversal);

	/* 2. Bring PHY out of reset. */
	intel_cx0_phy_lane_reset(i915, encoder->port, lane_reversal);

	/*
	 * 3. Change Phy power state to Ready.
	 * TODO: For DP alt mode use only one lane.
	 */
	intel_cx0_powerdown_change_sequence(i915, encoder->port, INTEL_CX0_BOTH_LANES,
					    CX0_P2_STATE_READY);

	/* 4. Program PHY internal PLL internal registers. */
	intel_c10_pll_program(i915, crtc_state, encoder);

	/*
	 * 5. Program the enabled and disabled owned PHY lane
	 * transmitters over message bus
	 */
	intel_c10_program_phy_lane(i915, encoder, crtc_state->lane_count, lane_reversal);

	/*
	 * 6. Follow the Display Voltage Frequency Switching - Sequence
	 * Before Frequency Change. We handle this step in bxt_set_cdclk().
	 */

	/*
	 * 7. Program DDI_CLK_VALFREQ to match intended DDI
	 * clock frequency.
	 */
	intel_de_write(i915, DDI_CLK_VALFREQ(encoder->port),
		       crtc_state->port_clock);
	/*
	 * 8. Set PORT_CLOCK_CTL register PCLK PLL Request
	 * LN<Lane for maxPCLK> to "1" to enable PLL.
	 */
	intel_de_rmw(i915, XELPDP_PORT_CLOCK_CTL(encoder->port),
		     intel_cx0_get_pclk_pll_request(INTEL_CX0_BOTH_LANES),
		     intel_cx0_get_pclk_pll_request(maxpclk_lane));

	/* 9. Poll on PORT_CLOCK_CTL PCLK PLL Ack LN<Lane for maxPCLK> == "1". */
	if (__intel_wait_for_register(&i915->uncore, XELPDP_PORT_CLOCK_CTL(encoder->port),
				      intel_cx0_get_pclk_pll_ack(INTEL_CX0_BOTH_LANES),
				      intel_cx0_get_pclk_pll_ack(maxpclk_lane),
				      XELPDP_PCLK_PLL_ENABLE_TIMEOUT_US, 0, NULL))
		drm_warn(&i915->drm, "Port %c PLL not locked after %dus.\n",
			 phy_name(phy), XELPDP_PCLK_PLL_ENABLE_TIMEOUT_US);

	/*
	 * 10. Follow the Display Voltage Frequency Switching Sequence After
	 * Frequency Change. We handle this step in bxt_set_cdclk().
	 */
}

void intel_cx0pll_enable(struct intel_encoder *encoder,
			 const struct intel_crtc_state *crtc_state)
{
	struct drm_i915_private *i915 = to_i915(encoder->base.dev);
	enum phy phy = intel_port_to_phy(i915, encoder->port);

	drm_WARN_ON(&i915->drm, !intel_is_c10phy(i915, phy));
	intel_c10pll_enable(encoder, crtc_state);
}

static void intel_c10pll_disable(struct intel_encoder *encoder)
{
	struct drm_i915_private *i915 = to_i915(encoder->base.dev);
	enum phy phy = intel_port_to_phy(i915, encoder->port);

	/* 1. Change owned PHY lane power to Disable state. */
	intel_cx0_powerdown_change_sequence(i915, encoder->port, INTEL_CX0_BOTH_LANES,
					    CX0_P2PG_STATE_DISABLE);

	/*
	 * 2. Follow the Display Voltage Frequency Switching Sequence Before
	 * Frequency Change. We handle this step in bxt_set_cdclk().
	 */

	/*
	 * 3. Set PORT_CLOCK_CTL register PCLK PLL Request LN<Lane for maxPCLK>
	 * to "0" to disable PLL.
	 */
	intel_de_rmw(i915, XELPDP_PORT_CLOCK_CTL(encoder->port),
		     intel_cx0_get_pclk_pll_request(INTEL_CX0_BOTH_LANES) |
		     intel_cx0_get_pclk_refclk_request(INTEL_CX0_BOTH_LANES), 0);

	/* 4. Program DDI_CLK_VALFREQ to 0. */
	intel_de_write(i915, DDI_CLK_VALFREQ(encoder->port), 0);

	/*
	 * 5. Poll on PORT_CLOCK_CTL PCLK PLL Ack LN<Lane for maxPCLK**> == "0".
	 */
	if (__intel_wait_for_register(&i915->uncore, XELPDP_PORT_CLOCK_CTL(encoder->port),
				      intel_cx0_get_pclk_pll_ack(INTEL_CX0_BOTH_LANES) |
				      intel_cx0_get_pclk_refclk_ack(INTEL_CX0_BOTH_LANES), 0,
				      XELPDP_PCLK_PLL_DISABLE_TIMEOUT_US, 0, NULL))
		drm_warn(&i915->drm, "Port %c PLL not unlocked after %dus.\n",
			 phy_name(phy), XELPDP_PCLK_PLL_DISABLE_TIMEOUT_US);

	/*
	 * 6. Follow the Display Voltage Frequency Switching Sequence After
	 * Frequency Change. We handle this step in bxt_set_cdclk().
	 */

	/* 7. Program PORT_CLOCK_CTL register to disable and gate clocks. */
	intel_de_rmw(i915, XELPDP_PORT_CLOCK_CTL(encoder->port),
		     XELPDP_DDI_CLOCK_SELECT_MASK |
		     XELPDP_FORWARD_CLOCK_UNGATE, 0);
}

void intel_cx0pll_disable(struct intel_encoder *encoder)
{
	struct drm_i915_private *i915 = to_i915(encoder->base.dev);
	enum phy phy = intel_port_to_phy(i915, encoder->port);

	drm_WARN_ON(&i915->drm, !intel_is_c10phy(i915, phy));
	intel_c10pll_disable(encoder);
}

void intel_c10mpllb_state_verify(struct intel_atomic_state *state,
				 struct intel_crtc_state *new_crtc_state)
{
	struct drm_i915_private *i915 = to_i915(state->base.dev);
	struct intel_c10mpllb_state mpllb_hw_state = { 0 };
	struct intel_c10mpllb_state *mpllb_sw_state = &new_crtc_state->c10mpllb_state;
	struct intel_crtc *crtc = to_intel_crtc(new_crtc_state->uapi.crtc);
	struct intel_encoder *encoder;
	struct intel_dp *intel_dp;
	enum phy phy;
	int i;
	bool use_ssc = false;

	if (DISPLAY_VER(i915) < 14)
		return;

	if (!new_crtc_state->hw.active)
		return;

	encoder = intel_get_crtc_new_encoder(state, new_crtc_state);
	phy = intel_port_to_phy(i915, encoder->port);

	if (intel_crtc_has_dp_encoder(new_crtc_state)) {
		intel_dp = enc_to_intel_dp(encoder);
		use_ssc = (intel_dp->dpcd[DP_MAX_DOWNSPREAD] &
			  DP_MAX_DOWNSPREAD_0_5);

		if (!intel_panel_use_ssc(i915))
			use_ssc = false;
	}

	if (!intel_is_c10phy(i915, phy))
		return;

	intel_c10mpllb_readout_hw_state(encoder, &mpllb_hw_state);

	for (i = 0; i < ARRAY_SIZE(mpllb_sw_state->pll); i++) {
		u8 expected;

		if (!use_ssc && i > 3 && i < 9)
			expected = 0;
		else
			expected = mpllb_sw_state->pll[i];

		I915_STATE_WARN(mpllb_hw_state.pll[i] != expected,
				"[CRTC:%d:%s] mismatch in C10MPLLB: Register[%d] (expected 0x%02x, found 0x%02x)",
				crtc->base.base.id, crtc->base.name,
				i, expected, mpllb_hw_state.pll[i]);
	}
}
