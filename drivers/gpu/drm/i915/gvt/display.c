/*
 * Copyright(c) 2011-2016 Intel Corporation. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Authors:
 *    Ke Yu
 *    Zhiyuan Lv <zhiyuan.lv@intel.com>
 *
 * Contributors:
 *    Terrence Xu <terrence.xu@intel.com>
 *    Changbin Du <changbin.du@intel.com>
 *    Bing Niu <bing.niu@intel.com>
 *    Zhi Wang <zhi.a.wang@intel.com>
 *
 */

#include "i915_drv.h"
#include "gvt.h"
#include <drm/drm_fourcc.h>

static enum pipe get_edp_pipe(struct intel_vgpu *vgpu)
{
	u32 data = vgpu_vreg(vgpu, _TRANS_DDI_FUNC_CTL_EDP);
	enum pipe pipe = INVALID_PIPE;

	switch (data & TRANS_DDI_EDP_INPUT_MASK) {
	case TRANS_DDI_EDP_INPUT_A_ON:
	case TRANS_DDI_EDP_INPUT_A_ONOFF:
		pipe = PIPE_A;
		break;
	case TRANS_DDI_EDP_INPUT_B_ONOFF:
		pipe = PIPE_B;
		break;
	case TRANS_DDI_EDP_INPUT_C_ONOFF:
		pipe = PIPE_C;
		break;
	}
	return pipe;
}

static int edp_pipe_is_enabled(struct intel_vgpu *vgpu)
{
	struct drm_i915_private *dev_priv = vgpu->gvt->dev_priv;

	if (!(vgpu_vreg_t(vgpu, PIPECONF(_PIPE_EDP)) & PIPECONF_ENABLE))
		return 0;

	if (!(vgpu_vreg(vgpu, _TRANS_DDI_FUNC_CTL_EDP) & TRANS_DDI_FUNC_ENABLE))
		return 0;
	return 1;
}

int pipe_is_enabled(struct intel_vgpu *vgpu, enum pipe pipe)
{
	struct drm_i915_private *dev_priv = vgpu->gvt->dev_priv;

	if (pipe == INVALID_PIPE || pipe >= INTEL_NUM_PIPES(dev_priv)) {
		gvt_dbg_dpy("vgpu:%d invalid pipe-%d to check enablement\n",
					vgpu->id, pipe);
		return -EINVAL;
	}

	if (vgpu_vreg_t(vgpu, PIPECONF(pipe)) & PIPECONF_ENABLE)
		return 1;

	if (edp_pipe_is_enabled(vgpu) &&
			get_edp_pipe(vgpu) == pipe)
		return 1;
	return 0;
}

static unsigned char virtual_dp_monitor_edid[GVT_EDID_NUM][EDID_SIZE] = {
	{
/* EDID with 1024x768 as its resolution */
		/*Header*/
		0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00,
		/* Vendor & Product Identification */
		0x22, 0xf0, 0x54, 0x29, 0x00, 0x00, 0x00, 0x00, 0x04, 0x17,
		/* Version & Revision */
		0x01, 0x04,
		/* Basic Display Parameters & Features */
		0xa5, 0x34, 0x20, 0x78, 0x23,
		/* Color Characteristics */
		0xfc, 0x81, 0xa4, 0x55, 0x4d, 0x9d, 0x25, 0x12, 0x50, 0x54,
		/* Established Timings: maximum resolution is 1024x768 */
		0x21, 0x08, 0x00,
		/* Standard Timings. All invalid */
		0x00, 0xc0, 0x00, 0xc0, 0x00, 0x40, 0x00, 0x80, 0x00, 0x00,
		0x00, 0x40, 0x00, 0x00, 0x00, 0x01,
		/* 18 Byte Data Blocks 1: invalid */
		0x00, 0x00, 0x80, 0xa0, 0x70, 0xb0,
		0x23, 0x40, 0x30, 0x20, 0x36, 0x00, 0x06, 0x44, 0x21, 0x00, 0x00, 0x1a,
		/* 18 Byte Data Blocks 2: invalid */
		0x00, 0x00, 0x00, 0xfd, 0x00, 0x18, 0x3c, 0x18, 0x50, 0x11, 0x00, 0x0a,
		0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
		/* 18 Byte Data Blocks 3: invalid */
		0x00, 0x00, 0x00, 0xfc, 0x00, 0x48,
		0x50, 0x20, 0x5a, 0x52, 0x32, 0x34, 0x34, 0x30, 0x77, 0x0a, 0x20, 0x20,
		/* 18 Byte Data Blocks 4: invalid */
		0x00, 0x00, 0x00, 0xff, 0x00, 0x43, 0x4e, 0x34, 0x33, 0x30, 0x34, 0x30,
		0x44, 0x58, 0x51, 0x0a, 0x20, 0x20,
		/* Extension Block Count */
		0x00,
		/* Checksum */
		0xef,
	},
	{
/* EDID with 1920x1200 as its resolution */
		/*Header*/
		0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00,
		/* Vendor & Product Identification */
		0x22, 0xf0, 0x54, 0x29, 0x00, 0x00, 0x00, 0x00, 0x04, 0x17,
		/* Version & Revision */
		0x01, 0x04,
		/* Basic Display Parameters & Features */
		0xa5, 0x34, 0x20, 0x78, 0x23,
		/* Color Characteristics */
		0xfc, 0x81, 0xa4, 0x55, 0x4d, 0x9d, 0x25, 0x12, 0x50, 0x54,
		/* Established Timings: maximum resolution is 1024x768 */
		0x21, 0x08, 0x00,
		/*
		 * Standard Timings.
		 * below new resolutions can be supported:
		 * 1920x1080, 1280x720, 1280x960, 1280x1024,
		 * 1440x900, 1600x1200, 1680x1050
		 */
		0xd1, 0xc0, 0x81, 0xc0, 0x81, 0x40, 0x81, 0x80, 0x95, 0x00,
		0xa9, 0x40, 0xb3, 0x00, 0x01, 0x01,
		/* 18 Byte Data Blocks 1: max resolution is 1920x1200 */
		0x28, 0x3c, 0x80, 0xa0, 0x70, 0xb0,
		0x23, 0x40, 0x30, 0x20, 0x36, 0x00, 0x06, 0x44, 0x21, 0x00, 0x00, 0x1a,
		/* 18 Byte Data Blocks 2: invalid */
		0x00, 0x00, 0x00, 0xfd, 0x00, 0x18, 0x3c, 0x18, 0x50, 0x11, 0x00, 0x0a,
		0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
		/* 18 Byte Data Blocks 3: invalid */
		0x00, 0x00, 0x00, 0xfc, 0x00, 0x48,
		0x50, 0x20, 0x5a, 0x52, 0x32, 0x34, 0x34, 0x30, 0x77, 0x0a, 0x20, 0x20,
		/* 18 Byte Data Blocks 4: invalid */
		0x00, 0x00, 0x00, 0xff, 0x00, 0x43, 0x4e, 0x34, 0x33, 0x30, 0x34, 0x30,
		0x44, 0x58, 0x51, 0x0a, 0x20, 0x20,
		/* Extension Block Count */
		0x00,
		/* Checksum */
		0x45,
	},
};

#define DPCD_HEADER_SIZE        0xb

/* let the virtual display supports DP1.2 */
static u8 dpcd_fix_data[DPCD_HEADER_SIZE] = {
	0x12, 0x014, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static void emulate_monitor_status_change(struct intel_vgpu *vgpu)
{
	struct drm_i915_private *dev_priv = vgpu->gvt->dev_priv;
	struct intel_vgpu_display *disp_cfg = &vgpu->disp_cfg;
	struct intel_vgpu_display_path *disp_path = NULL, *n;
	enum port port;
	enum pipe pipe;
	enum transcoder trans;

	if (IS_BROADWELL(dev_priv)) {
		vgpu_vreg_t(vgpu, GEN8_DE_PORT_ISR) &= ~GEN8_PORT_DP_A_HOTPLUG;
		vgpu_vreg_t(vgpu, SDEISR) &= ~(SDE_PORTB_HOTPLUG_CPT |
			SDE_PORTC_HOTPLUG_CPT |
			SDE_PORTD_HOTPLUG_CPT);
		vgpu_vreg_t(vgpu, PCH_ADPA) &= ~ADPA_CRT_HOTPLUG_MONITOR_MASK;
		vgpu_vreg_t(vgpu, SFUSE_STRAP) &= ~(SFUSE_STRAP_DDIB_DETECTED |
			SFUSE_STRAP_DDIC_DETECTED |
			SFUSE_STRAP_DDID_DETECTED);
		for (port = PORT_A; port <= PORT_E; port++) {
			vgpu_vreg_t(vgpu, PORT_CLK_SEL(port)) |=
				PORT_CLK_SEL_NONE;
		}
	} else if (IS_GEN9_LP(dev_priv)) {
		vgpu_vreg_t(vgpu, GEN8_DE_PORT_ISR) &= ~(BXT_DE_PORT_HP_DDIA |
			BXT_DE_PORT_HP_DDIB |
			BXT_DE_PORT_HP_DDIC);
	} else {
		vgpu_vreg_t(vgpu, SDEISR) &= ~(SDE_PORTA_HOTPLUG_SPT |
			SDE_PORTB_HOTPLUG_CPT |
			SDE_PORTC_HOTPLUG_CPT |
			SDE_PORTD_HOTPLUG_CPT |
			SDE_PORTE_HOTPLUG_SPT);
		vgpu_vreg_t(vgpu, SFUSE_STRAP) &= ~(SFUSE_STRAP_DDIB_DETECTED |
			SFUSE_STRAP_DDIC_DETECTED |
			SFUSE_STRAP_DDID_DETECTED |
			SFUSE_STRAP_DDIF_DETECTED);
		// Keep same copy as host until HPD support
		// for (port = PORT_A; port <= PORT_E; port++) {
		// 	vgpu_vreg_t(vgpu, DPLL_CTRL2) &=
		// 			~DPLL_CTRL2_DDI_CLK_SEL_MASK(port);
		// 	vgpu_vreg_t(vgpu, DPLL_CTRL2) |=
		// 		(DPLL_CTRL2_DDI_CLK_OFF(port) |
		// 		DPLL_CTRL2_DDI_SEL_OVERRIDE(port));
		// }
	}

	for_each_pipe(dev_priv, pipe) {
		vgpu_vreg_t(vgpu, PIPECONF(pipe)) &=
			~(PIPECONF_ENABLE | I965_PIPECONF_ACTIVE);
		vgpu_vreg_t(vgpu, DSPCNTR(pipe)) &= ~DISPLAY_PLANE_ENABLE;
		vgpu_vreg_t(vgpu, SPRCTL(pipe)) &= ~SPRITE_ENABLE;
		vgpu_vreg_t(vgpu, CURCNTR(pipe)) &= ~MCURSOR_MODE;
		vgpu_vreg_t(vgpu, CURCNTR(pipe)) |= MCURSOR_MODE_DISABLE;
	}

	for (trans = TRANSCODER_A; trans <= TRANSCODER_EDP; trans++) {
		vgpu_vreg_t(vgpu, TRANS_DDI_FUNC_CTL(trans)) &=
			~(TRANS_DDI_BPC_MASK | TRANS_DDI_MODE_SELECT_MASK |
			TRANS_DDI_PORT_MASK | TRANS_DDI_FUNC_ENABLE);
	}

	for (port = PORT_A; port <= PORT_E; port++) {
		if (IS_GEN9_LP(dev_priv) && port > PORT_C)
			continue;
		vgpu_vreg_t(vgpu, DDI_BUF_CTL(port)) &=
			~(DDI_INIT_DISPLAY_DETECTED | DDI_BUF_CTL_ENABLE);
		vgpu_vreg_t(vgpu, DDI_BUF_CTL(port)) |=
			DDI_BUF_IS_IDLE;

	}

	list_for_each_entry_safe(disp_path, n, &disp_cfg->path_list, list) {
		pipe = disp_path->pipe;
		port = disp_path->port;
		trans = disp_path->trans;
		if (port == PORT_NONE)
			continue;
		if (IS_BROADWELL(dev_priv)) {
			if (port > PORT_D)
				continue;
		} else if (IS_GEN9_LP(dev_priv)) {
			if (port > PORT_C)
				continue;
		} else {
			if (port > PORT_E)
				continue;
		}
		if (intel_vgpu_display_has_monitor(disp_path)) {
			u32 ddi_fuse[] = {
				[PORT_A] = 0,
				[PORT_B] = SFUSE_STRAP_DDIB_DETECTED,
				[PORT_C] = SFUSE_STRAP_DDIC_DETECTED,
				[PORT_D] = SFUSE_STRAP_DDID_DETECTED,
			};

			if (IS_BROADWELL(dev_priv)) {
				vgpu_vreg_t(vgpu, PORT_CLK_SEL(port)) &=
					~PORT_CLK_SEL_MASK;
				vgpu_vreg_t(vgpu, PORT_CLK_SEL(port)) |=
					PORT_CLK_SEL_LCPLL_2700;
				if (port >= PORT_B && port <= PORT_D)
					vgpu_vreg_t(vgpu, SFUSE_STRAP) |=
						ddi_fuse[port];
			} else if (IS_GEN9_LP(dev_priv)) {
			} else {
				// vgpu_vreg_t(vgpu, DPLL_CTRL2) &=
				// 	~DPLL_CTRL2_DDI_CLK_OFF(port);
				// vgpu_vreg_t(vgpu, DPLL_CTRL2) |=
				// 	(DPLL_CTRL2_DDI_CLK_SEL(DPLL_ID_SKL_DPLL0 + port, port) |
				// 	DPLL_CTRL2_DDI_SEL_OVERRIDE(port));
				if (port >= PORT_B && port <= PORT_D)
					vgpu_vreg_t(vgpu, SFUSE_STRAP) |=
						ddi_fuse[port];
			}

			vgpu_vreg_t(vgpu, TRANS_DDI_FUNC_CTL(trans)) |=
				TRANS_DDI_BPC_8 | TRANS_DDI_MODE_SELECT_DVI |
				(port << TRANS_DDI_PORT_SHIFT) |
				TRANS_DDI_FUNC_ENABLE;
			if (trans == TRANSCODER_A)
				vgpu_vreg_t(vgpu, TRANS_DDI_FUNC_CTL(trans)) &=
					~TRANS_DDI_EDP_INPUT_MASK;
				vgpu_vreg_t(vgpu, TRANS_DDI_FUNC_CTL(trans)) |=
					TRANS_DDI_EDP_INPUT_A_ON;

			if (port == PORT_A)
				vgpu_vreg_t(vgpu, DDI_BUF_CTL(port)) |=
					DDI_INIT_DISPLAY_DETECTED;
			vgpu_vreg_t(vgpu, DDI_BUF_CTL(port)) |=
				DDI_BUF_CTL_ENABLE;
			vgpu_vreg_t(vgpu, DDI_BUF_CTL(port)) &=
				~DDI_BUF_IS_IDLE;

			vgpu_vreg_t(vgpu, PIPECONF(pipe)) |=
				(PIPECONF_ENABLE | I965_PIPECONF_ACTIVE);

			if (IS_BROADWELL(dev_priv)) {
				u32 bdw_hpd_pin[] = {
					[PORT_A] = GEN8_PORT_DP_A_HOTPLUG,
					[PORT_B] = SDE_PORTB_HOTPLUG_CPT,
					[PORT_C] = SDE_PORTC_HOTPLUG_CPT,
					[PORT_D] = SDE_PORTD_HOTPLUG_CPT,
				};

				if (port == PORT_A)
					vgpu_vreg_t(vgpu, GEN8_DE_PORT_ISR) |=
						bdw_hpd_pin[port];
				else
					vgpu_vreg_t(vgpu, SDEISR) |=
						bdw_hpd_pin[port];
			} else if (IS_GEN9_LP(dev_priv)) {
				u32 bxt_hpd_pin[] = {
					[PORT_A] = BXT_DE_PORT_HP_DDIA,
					[PORT_B] = BXT_DE_PORT_HP_DDIB,
					[PORT_C] = BXT_DE_PORT_HP_DDIC
				};

				vgpu_vreg_t(vgpu, GEN8_DE_PORT_ISR) |=
					bxt_hpd_pin[port];
			} else {
				u32 skl_hpd_pin[] = {
					[PORT_A] = SDE_PORTA_HOTPLUG_SPT,
					[PORT_B] = SDE_PORTB_HOTPLUG_CPT,
					[PORT_C] = SDE_PORTC_HOTPLUG_CPT,
					[PORT_D] = SDE_PORTD_HOTPLUG_CPT,
					[PORT_E] = SDE_PORTE_HOTPLUG_SPT,
				};

				vgpu_vreg_t(vgpu, SDEISR) |=
					skl_hpd_pin[port];
			}

			gvt_dbg_dpy("vgpu:%d emulate monitor on PIPE_%c TRANSCODE_%s PORT_%c\n",
				    vgpu->id, pipe_name(pipe),
				    transcoder_name(trans), port_name(port));
		}
	}
}

static void clean_virtual_dp_monitor(struct intel_vgpu *vgpu,
				     struct intel_vgpu_display_path *disp_path)
{
	if (!disp_path) {
		gvt_err("vgpu-%d invalid vgpu display path\n", vgpu->id);
		return;
	}

	kfree(disp_path->edid);
	disp_path->edid = NULL;

	kfree(disp_path->dpcd);
	disp_path->dpcd = NULL;

	disp_path->port_type = INTEL_OUTPUT_UNUSED;

	gvt_dbg_dpy("vgpu-%d virtual DP monitor on path-%d cleaned\n",
		    vgpu->id, disp_path->id);
}

#define GVT_EDID_EST_TIMINGS 16
#define GVT_EDID_STD_TIMINGS 8
#define GVT_EDID_DETAILED_TIMINGS 4

static int filter_detailed_modes(struct edid *edid)
{
	int i;

	for (i = 1; i < GVT_EDID_DETAILED_TIMINGS; i++)
		edid->detailed_timings[i].pixel_clock = 0x00;

	return 0;
}


static int filter_standard_modes(struct edid *edid)
{
	int i;

	for (i = 0; i < GVT_EDID_STD_TIMINGS; i++) {
		edid->standard_timings[i].hsize = 0x01;
		edid->standard_timings[i].vfreq_aspect = 0x01;
	}

	return 0;
}

static int filter_established_modes(struct edid *edid)
{
	edid->established_timings.t1 = 0;
	edid->established_timings.t2 = 0x00;
	edid->established_timings.mfg_rsvd &= ~0x80;

	return 0;
}

static int filter_ext_blocks(u8 *raw_edid)
{
	raw_edid[EDID_LENGTH-2] = 0;

	return 0;
}

static int  gvt_edid_set_checksum(u8 *raw_edid)
{
	int i;
	u8 csum = 0;

	for (i = 0; i < EDID_LENGTH-1; i++)
		csum += raw_edid[i];
	raw_edid[EDID_LENGTH-1] = 256 - csum%256;

	return csum;
}


static int setup_virtual_dp_monitor_edid(struct intel_vgpu *vgpu,
					 struct intel_vgpu_display_path *disp_path,
					 void *edid)
{
	int valid_extensions = 1;
	struct edid *raw_edid;

	if (!disp_path) {
		gvt_err("vgpu-%d invalid vgpu display path\n", vgpu->id);
		return -EINVAL;
	}

	if (WARN_ON(disp_path->port_type != INTEL_OUTPUT_DP &&
		    disp_path->port_type != INTEL_OUTPUT_EDP &&
		    disp_path->port_type != INTEL_OUTPUT_DP_MST)) {
		gvt_err("vgpu-%d: Unsupported virtual DP monitor type:%d on PORT_%c\n",
			vgpu->id, disp_path->port_type, disp_path->port);
		return -EINVAL;
	}

	if (WARN_ON(!edid && disp_path->resolution >= GVT_EDID_NUM)) {
		gvt_err("vgpu-%d: Unsupported virtual DP monitor resolution:%llx\n",
			vgpu->id, disp_path->resolution);
		return -EINVAL;
	}

	if (edid)
		valid_extensions += ((struct edid *)edid)->extensions;

	disp_path->edid = kzalloc(sizeof(*disp_path->edid)
			+ valid_extensions * EDID_SIZE, GFP_KERNEL);
	if (!disp_path->edid) {
		gvt_err("vgpu-%d: Fail to allocate EDID for virtual DP monitor\n",
			vgpu->id);
		return -ENOMEM;
	}

	disp_path->dpcd = kzalloc(sizeof(*disp_path->dpcd), GFP_KERNEL);
	if (!disp_path->dpcd) {
		kfree(disp_path->edid);
		gvt_dbg_dpy("vgpu-%d: Fail to allocate DPCD for virtual DP monitor\n",
			    vgpu->id);
		return -ENOMEM;
	}

	if (edid)
		memcpy(disp_path->edid->edid_block, edid,
		       EDID_SIZE * valid_extensions);
	else
		memcpy(disp_path->edid->edid_block,
		       virtual_dp_monitor_edid[disp_path->resolution],
		       EDID_SIZE);

	raw_edid = (struct edid *)disp_path->edid->edid_block;
	if (READ_ONCE(vgpu->gvt->disp_edid_filter) &&
		edid && ((raw_edid->version > 1) ||
		(raw_edid->version == 1 && raw_edid->revision > 2))) {
		filter_detailed_modes(raw_edid);
		filter_standard_modes(raw_edid);
		filter_established_modes(raw_edid);
		filter_ext_blocks((u8 *)raw_edid);
		gvt_edid_set_checksum((u8 *)raw_edid);
		gvt_dbg_dpy(" vgpu:%d  edid filter is done\n",
			vgpu->id);
	} else {
		if ((raw_edid->version == 1 && raw_edid->revision <= 2))
			gvt_dbg_dpy("vgpu:%d  edid  is lower than 1.3\n",
				vgpu->id);
	}

	disp_path->edid->data_valid = true;

	memcpy(disp_path->dpcd->data, dpcd_fix_data, DPCD_HEADER_SIZE);
	disp_path->dpcd->data_valid = true;
	disp_path->dpcd->data[DPCD_SINK_COUNT] = 0x1;

	gvt_dbg_dpy("vgpu:%d setup edid:%p on virtual DP monitor on port:%d, type:%d\n",
		    vgpu->id, edid, disp_path->port, disp_path->port_type);

	return 0;
}

/**
 * intel_gvt_check_vblank_emulation - check if vblank emulation timer should
 * be turned on/off when a virtual pipe is enabled/disabled.
 * @gvt: a GVT device
 *
 * This function is used to turn on/off vblank timer according to currently
 * enabled/disabled virtual pipes.
 *
 */
void intel_gvt_check_vblank_emulation(struct intel_vgpu *vgpu, enum pipe pipe)
{
	struct intel_gvt *gvt = vgpu->gvt;
	struct intel_gvt_irq *irq = &gvt->irq;

	mutex_lock(&gvt->lock);

	if (pipe_is_enabled(vgpu, pipe))
		hrtimer_start(&irq->vblank_timer.timer,
			ktime_add_ns(ktime_get(), irq->vblank_timer.period),
			HRTIMER_MODE_ABS);
	else
		hrtimer_cancel(&irq->vblank_timer.timer);

	mutex_unlock(&gvt->lock);
}

static void emulate_vblank_on_pipe(struct intel_vgpu *vgpu, int pipe)
{
	struct drm_i915_private *dev_priv = vgpu->gvt->dev_priv;
	struct intel_vgpu_irq *irq = &vgpu->irq;
	int vblank_event[] = {
		[PIPE_A] = PIPE_A_VBLANK,
		[PIPE_B] = PIPE_B_VBLANK,
		[PIPE_C] = PIPE_C_VBLANK,
	};
	int event;

	if (pipe == INVALID_PIPE || pipe >= INTEL_NUM_PIPES(dev_priv))
		return;

	for_each_set_bit(event, irq->flip_done_event[pipe],
			INTEL_GVT_EVENT_MAX) {
		clear_bit(event, irq->flip_done_event[pipe]);
		if (!pipe_is_enabled(vgpu, pipe))
			continue;

		intel_vgpu_trigger_virtual_event(vgpu, event);
	}

	vgpu_vreg_t(vgpu, PIPE_FRMCOUNT_G4X(pipe))++;
	intel_vgpu_trigger_virtual_event(vgpu, vblank_event[pipe]);
}

/**
 * intel_gvt_emulate_vblank - trigger vblank events for vGPUs on GVT device
 * @gvt: a GVT device
 *
 * This function is used to trigger vblank interrupts for vGPUs on GVT device
 *
 */
void intel_gvt_emulate_vblank(struct intel_gvt *gvt)
{
	struct intel_vgpu *vgpu;
	struct intel_vgpu_display *disp_cfg = NULL;
	struct intel_vgpu_display_path *disp_path = NULL, *n;
	int id;

	mutex_lock(&gvt->lock);
	for_each_active_vgpu(gvt, vgpu, id) {
		mutex_lock(&vgpu->vgpu_lock);
		disp_cfg = &vgpu->disp_cfg;
		list_for_each_entry_safe(disp_path, n, &disp_cfg->path_list, list) {
			if (disp_path->p_pipe == INVALID_PIPE ||
			    disp_path->p_pipe != gvt->pipe_info[disp_path->p_pipe].pipe_num ||
			    vgpu->id != gvt->pipe_info[disp_path->p_pipe].owner) {
				emulate_vblank_on_pipe(vgpu, disp_path->pipe);
			} else {
				u64 ns = jiffies_to_nsecs(get_jiffies_64());

				if ((ns - disp_path->last_hwvsync_ns > 2 * disp_path->vsync_interval_ns) &&
				    !test_and_clear_bit(INTEL_GVT_DIRECT_DISPLAY_HW_VSYNC, (void *)&disp_path->stat)) {
					emulate_vblank_on_pipe(vgpu, disp_path->pipe);
					disp_path->sw_vsync_injected++;
					gvt_dbg_dpy("vgpu:%d inject vsync %lld to PIPE_%c <- HW vsync miss on PIPE_%c\n",
						    vgpu->id,
						    disp_path->sw_vsync_injected,
						    pipe_name(disp_path->pipe),
						    pipe_name(disp_path->p_pipe));
				}
			}
		}
		mutex_unlock(&vgpu->vgpu_lock);
	}
	mutex_unlock(&gvt->lock);
}

static void intel_gvt_vblank_work(struct work_struct *w)
{
	struct intel_gvt_pipe_info *pipe_info = container_of(w,
			struct intel_gvt_pipe_info, vblank_work);
	struct intel_gvt *gvt = pipe_info->gvt;
	struct intel_vgpu *vgpu;
	struct intel_vgpu_display *disp_cfg = NULL;
	struct intel_vgpu_display_path *disp_path = NULL, *n;
	int id;

	mutex_lock(&gvt->lock);
	for_each_active_vgpu(gvt, vgpu, id) {
		mutex_lock(&vgpu->vgpu_lock);
		disp_cfg = &vgpu->disp_cfg;
		mutex_lock(&disp_cfg->sw_lock);
		list_for_each_entry_safe(disp_path, n, &disp_cfg->path_list, list) {
			if (disp_path->p_pipe != INVALID_PIPE &&
			    disp_path->p_pipe == pipe_info->pipe_num &&
			    vgpu->id == pipe_info->owner &&
			    pipe_is_enabled(vgpu, disp_path->pipe)) {
				emulate_vblank_on_pipe(vgpu, disp_path->pipe);
				disp_path->last_hwvsync_ns = jiffies_to_nsecs(get_jiffies_64());
				set_bit(INTEL_GVT_DIRECT_DISPLAY_HW_VSYNC, (void *)&disp_path->stat);
			}
		}
		mutex_unlock(&disp_cfg->sw_lock);
		mutex_unlock(&vgpu->vgpu_lock);
	}
	mutex_unlock(&gvt->lock);
}

static void intel_gvt_flipdone_work(struct work_struct *w)
{
	struct intel_gvt_plane_info *plane_info = container_of(w,
			struct intel_gvt_plane_info, flipdone_work);
	struct intel_gvt *gvt = plane_info->gvt;
	struct intel_vgpu *vgpu;
	struct intel_vgpu_display *disp_cfg;
	struct intel_vgpu_display_path *disp_path = NULL, *n;
	int id;
	int event = SKL_FLIP_EVENT(plane_info->pipe, plane_info->plane);

	mutex_lock(&gvt->lock);
	for_each_active_vgpu(gvt, vgpu, id) {
		mutex_lock(&vgpu->vgpu_lock);
		disp_cfg = &vgpu->disp_cfg;
		mutex_lock(&disp_cfg->sw_lock);
		list_for_each_entry_safe(disp_path, n, &disp_cfg->path_list, list) {
			if (plane_info->pipe == disp_path->p_pipe)
				intel_vgpu_trigger_virtual_event(vgpu, event);
		}
		mutex_unlock(&disp_cfg->sw_lock);
		mutex_unlock(&vgpu->vgpu_lock);
	}
	mutex_unlock(&gvt->lock);
}

static void intel_gvt_init_pipe_info(struct intel_gvt *gvt)
{
	enum pipe pipe = INVALID_PIPE;
	enum plane_id plane;

	for (pipe = PIPE_A; pipe < I915_MAX_PIPES; pipe++) {
		memset(&gvt->pipe_info[pipe], 0, sizeof(gvt->pipe_info[pipe]));
		gvt->pipe_info[pipe].pipe_num = pipe;
		gvt->pipe_info[pipe].gvt = gvt;
		INIT_WORK(&gvt->pipe_info[pipe].vblank_work,
			  intel_gvt_vblank_work);
		for (plane = PLANE_PRIMARY; plane < I915_MAX_PLANES; plane++) {
			memset(&gvt->pipe_info[pipe].plane_info[plane], 0,
			       sizeof(gvt->pipe_info[pipe].plane_info[plane]));
			gvt->pipe_info[pipe].plane_info[plane].gvt = gvt;
			gvt->pipe_info[pipe].plane_info[plane].pipe = pipe;
			gvt->pipe_info[pipe].plane_info[plane].plane = plane;
			INIT_WORK(&gvt->pipe_info[pipe].plane_info[plane].flipdone_work,
				  intel_gvt_flipdone_work);
		}
	}
	// By default, each vGPU will be assigned a different available port
	// when host setup each port on detection.
	// Display auto switch is enabled by default.
	WRITE_ONCE(gvt->disp_auto_switch, true);
	gvt->disp_owner = 0;
}

static void intel_gvt_connector_change_work(struct work_struct *w)
{
	struct intel_gvt *gvt = container_of(w,
			struct intel_gvt, connector_change_work);
	enum pipe pipe = INVALID_PIPE;
	u8 port_ext = 0;
	u8 id = 0;

	/*
	 * Doesn't support hotplug after vgpu created.
	 */
	gvt_dbg_dpy("Host connector status changed\n");
	mutex_lock(&gvt->lock);

	if (!idr_is_empty(&gvt->vgpu_idr)) {
		mutex_unlock(&gvt->lock);
		gvt_dbg_dpy("Available port mask %016llx and selected mask 0x%016llx "
			    "unchanged due to hotplug after vGPU creation\n",
			    gvt->avail_disp_port_mask, gvt->sel_disp_port_mask);
		return;
	}

	gvt->avail_disp_port_mask = 0;
	gvt->sel_disp_port_mask = 0;
	for_each_pipe(gvt->dev_priv, pipe) {
		enum port port;

		port = intel_gvt_port_from_pipe(gvt->dev_priv, pipe);
		if (port == PORT_NONE)
			continue;

		port_ext = intel_gvt_external_disp_id_from_port(port);
		// Available port is set in corresponding port position.
		gvt->avail_disp_port_mask |= intel_gvt_port_to_mask_bit(port_ext, port);
		// Available port is assigned to vGPU in sequence.
		gvt->sel_disp_port_mask |= ((1 << port) << id * 8);
		++id;
		gvt_dbg_dpy("PIPE_%c PORT_%c is default assigned to vGPU-%d\n",
			    pipe_name(pipe), port_name(port), id);
	}
	mutex_unlock(&gvt->lock);
}

static void intel_gvt_ddb_entry_write(struct drm_i915_private *dev_priv,
				      i915_reg_t reg,
				      const struct skl_ddb_entry *entry)
{
	if (entry->end)
		I915_WRITE_FW(reg, (entry->end - 1) << 16 | entry->start);
	else
		I915_WRITE_FW(reg, 0);
}
/*
 * When enabling multi-plane in DomU, an issue is that the PLANE_BUF_CFG
 * register cannot be updated dynamically, since Dom0 has no idea of the
 * plane information of DomU's planes, so here we statically allocate the
 * ddb entries for all the possible enabled planes.
 */
void intel_gvt_init_ddb(struct intel_gvt *gvt)
{
	struct drm_i915_private *dev_priv = gvt->dev_priv;
	struct intel_device_info *info = mkwrite_device_info(dev_priv);
	struct intel_gvt_pipe_info *pipe_info;
	unsigned int pipe_size, ddb_size, plane_size, plane_cnt;
	u16 start, end;
	enum pipe pipe = INVALID_PIPE;
	enum plane_id plane;

	ddb_size = info->ddb_size;
	ddb_size -= 4; /* 4 blocks for bypass path allocation */
	pipe_size = ddb_size / INTEL_NUM_PIPES(dev_priv);

	for_each_pipe(dev_priv, pipe) {
		pipe_info = &gvt->pipe_info[pipe];
		memset(&pipe_info->ddb_y, 0, sizeof(pipe_info->ddb_y));
		memset(&pipe_info->ddb_uv, 0, sizeof(pipe_info->ddb_uv));
		start = pipe * ddb_size / INTEL_NUM_PIPES(dev_priv);
		end = start + pipe_size;

		pipe_info->ddb_y[PLANE_CURSOR].start = end - 8;
		pipe_info->ddb_y[PLANE_CURSOR].end = end;
		intel_gvt_ddb_entry_write(dev_priv, CUR_BUF_CFG(pipe),
					  &pipe_info->ddb_y[PLANE_CURSOR]);

		// Disable other plane to support 4K on primary
		plane_cnt = RUNTIME_INFO(dev_priv)->num_sprites[(pipe)] + 1;
		plane_cnt = 1;
		plane_size = (pipe_size - 8) / plane_cnt;
		for_each_universal_plane(dev_priv, pipe, plane) {
			if (plane == PLANE_PRIMARY) {
				pipe_info->ddb_y[plane].start = start +
					plane * plane_size;
				pipe_info->ddb_y[plane].end =
					pipe_info->ddb_y[plane].start + plane_size;
			}
			intel_gvt_ddb_entry_write(dev_priv, PLANE_BUF_CFG(pipe, plane),
						  &pipe_info->ddb_y[plane]);
			intel_gvt_ddb_entry_write(dev_priv, PLANE_NV12_BUF_CFG(pipe, plane),
						  &pipe_info->ddb_uv[plane]);
		}
	}
}

static int intel_gvt_switch_pipe_owner(struct intel_gvt *gvt, enum pipe pipe,
				       int owner)
{
	struct drm_i915_private *dev_priv = gvt->dev_priv;
	struct intel_gvt_pipe_info *pipe_info = gvt->pipe_info;
	struct intel_vgpu *vgpu;
	enum plane_id plane;
	bool valid_owner = false;
	int id;

	if (pipe == INVALID_PIPE || pipe >= INTEL_NUM_PIPES(dev_priv)) {
		gvt_err("Invalid PIPE_%c for new owner %d\n",
			pipe_name(pipe), owner);
		return -EINVAL;
	}

	for_each_active_vgpu(gvt, vgpu, id) {
		if (owner == id) {
			valid_owner = true;
			break;
		}
	}

	if (owner && !valid_owner) {
		gvt_err("Invalid owner %d for PIPE_%c\n", owner,
			pipe_name(pipe));
		return -EINVAL;
	}

	pipe_info[pipe].owner = owner;
	for (plane = PLANE_PRIMARY; plane < I915_MAX_PLANES - 1; plane++)
		pipe_info[pipe].plane_info[plane].owner = owner;

	if (owner)
		gvt_dbg_dpy("PIPE_%c owner switched to vGPU-%d\n",
			    pipe_name(pipe), owner);
	else
		gvt_dbg_dpy("PIPE_%c owner switched to host\n", pipe_name(pipe));

	return 0;
}

static void direct_display_init_d0_regs(struct intel_gvt *gvt, enum pipe pipe)
{
	struct drm_i915_private *dev_priv = gvt->dev_priv;
	struct intel_runtime_info *runtime = RUNTIME_INFO(dev_priv);
	struct intel_dom0_pipe_regs *pipe_regs = NULL;
	struct intel_dom0_plane_regs *plane_regs = NULL;
	struct prec_pal_data *pal_data = NULL;
	enum plane_id plane;
	int i, level, max_level, scaler, max_scaler = 0;
	unsigned long irqflags;

	pipe_regs = &gvt->pipe_info[pipe].dom0_pipe_regs;
	max_scaler = runtime->num_scalers[pipe];
	max_level = ilk_wm_max_level(dev_priv);

	mmio_hw_access_pre(dev_priv);
	spin_lock_irqsave(&dev_priv->uncore.lock, irqflags);

	pipe_regs->pipesrc = I915_READ_FW(PIPESRC(pipe));
	for (scaler = 0; scaler < max_scaler; scaler++) {
		pipe_regs->scaler_ctl[scaler] = I915_READ_FW(SKL_PS_CTRL(pipe, scaler));
		pipe_regs->scaler_win_pos[scaler] = I915_READ_FW(SKL_PS_WIN_POS(pipe, scaler));
		pipe_regs->scaler_win_size[scaler] = I915_READ_FW(SKL_PS_WIN_SZ(pipe, scaler));
		pipe_regs->scaler_pwr_gate[scaler] = I915_READ_FW(SKL_PS_PWR_GATE(pipe, scaler));
	}
	pipe_regs->bottom_color = I915_READ_FW(SKL_BOTTOM_COLOR(pipe));
	pipe_regs->gamma_mode = I915_READ_FW(GAMMA_MODE(pipe));
	pipe_regs->csc_mode = I915_READ_FW(PIPE_CSC_MODE(pipe));
	pipe_regs->csc_preoff_hi = I915_READ_FW(PIPE_CSC_PREOFF_HI(pipe));
	pipe_regs->csc_preoff_me = I915_READ_FW(PIPE_CSC_PREOFF_ME(pipe));
	pipe_regs->csc_preoff_lo = I915_READ_FW(PIPE_CSC_PREOFF_LO(pipe));
	pipe_regs->csc_coeff_rygy = I915_READ_FW(PIPE_CSC_COEFF_RY_GY(pipe));
	pipe_regs->csc_coeff_by = I915_READ_FW(PIPE_CSC_COEFF_BY(pipe));
	pipe_regs->csc_coeff_rugu = I915_READ_FW(PIPE_CSC_COEFF_RU_GU(pipe));
	pipe_regs->csc_coeff_bu = I915_READ_FW(PIPE_CSC_COEFF_BU(pipe));
	pipe_regs->csc_coeff_rvgv = I915_READ_FW(PIPE_CSC_COEFF_RV_GV(pipe));
	pipe_regs->csc_coeff_bv = I915_READ_FW(PIPE_CSC_COEFF_BV(pipe));
	pipe_regs->csc_postoff_hi = I915_READ_FW(PIPE_CSC_POSTOFF_HI(pipe));
	pipe_regs->csc_postoff_me = I915_READ_FW(PIPE_CSC_POSTOFF_ME(pipe));
	pipe_regs->csc_postoff_lo = I915_READ_FW(PIPE_CSC_POSTOFF_LO(pipe));
	for (i = 0; i < 256; i++) {
		pipe_regs->lgc_palette[i] = I915_READ_FW(LGC_PALETTE(pipe, i));
	}

	// Set dirty for all d0 regs since i915 may not update all after BIOS.
	pal_data = pipe_regs->prec_palette_split;
	for (i = 0; i < PAL_PREC_INDEX_VALUE_MASK + 1; i++) {
		I915_WRITE_FW(PREC_PAL_INDEX(pipe), i | PAL_PREC_SPLIT_MODE);
		pal_data[i].val = I915_READ_FW(PREC_PAL_DATA(pipe));
		pal_data[i].dirty = 1;
	}
	pal_data = pipe_regs->prec_palette_nonsplit;
	for (i = 0; i < PAL_PREC_INDEX_VALUE_MASK + 1; i++) {
		I915_WRITE_FW(PREC_PAL_INDEX(pipe), i);
		pal_data[i].val = I915_READ_FW(PREC_PAL_DATA(pipe));
		pal_data[i].dirty = 1;
	}
	I915_WRITE_FW(PREC_PAL_INDEX(pipe), 0);

	for_each_universal_plane(dev_priv, pipe, plane) {
		plane_regs = &gvt->pipe_info[pipe].plane_info[plane].dom0_regs;
		if (plane == PLANE_CURSOR) {
			plane_regs->plane_ctl = I915_READ_FW(CURCNTR(pipe));
			plane_regs->plane_pos = I915_READ_FW(CURPOS(pipe));
			plane_regs->cur_fbc_ctl = I915_READ_FW(CUR_FBC_CTL(pipe));
			plane_regs->plane_surf = I915_READ_FW(CURBASE(pipe));
			for (level = 0; level <= max_level; level++) {
				plane_regs->plane_wm[level] = I915_READ_FW(CUR_WM(pipe, level));
			}
			plane_regs->plane_wm_trans = I915_READ_FW(CUR_WM_TRANS(pipe));
		} else {
			plane_regs->plane_ctl = I915_READ_FW(PLANE_CTL(pipe, plane));
			plane_regs->plane_stride = I915_READ_FW(PLANE_STRIDE(pipe, plane));
			plane_regs->plane_pos = I915_READ_FW(PLANE_POS(pipe, plane));
			plane_regs->plane_size = I915_READ_FW(PLANE_SIZE(pipe, plane));
			plane_regs->plane_keyval = I915_READ_FW(PLANE_KEYVAL(pipe, plane));
			plane_regs->plane_keymsk = I915_READ_FW(PLANE_KEYMSK(pipe, plane));
			plane_regs->plane_keymax = I915_READ_FW(PLANE_KEYMAX(pipe, plane));
			plane_regs->plane_offset = I915_READ_FW(PLANE_OFFSET(pipe, plane));
			plane_regs->plane_aux_dist = I915_READ_FW(PLANE_AUX_DIST(pipe, plane));
			plane_regs->plane_aux_offset = I915_READ_FW(PLANE_AUX_OFFSET(pipe, plane));
			plane_regs->plane_surf = I915_READ_FW(PLANE_SURF(pipe, plane));
			for (level = 0; level <= max_level; level++) {
				plane_regs->plane_wm[level] = I915_READ_FW(PLANE_WM(pipe, plane, level));
			}
			plane_regs->plane_wm_trans = I915_READ_FW(PLANE_WM_TRANS(pipe, plane));
		}
	}

	spin_unlock_irqrestore(&dev_priv->uncore.lock, irqflags);
	mmio_hw_access_post(dev_priv);
}

int setup_vgpu_virtual_display_path(struct intel_vgpu *vgpu,
				    struct intel_vgpu_display_path *disp_path)
{
	struct drm_i915_private *dev_priv = vgpu->gvt->dev_priv;
	struct intel_connector *connector = NULL;
	struct drm_connector_list_iter conn_iter;
	struct intel_digital_port *dig_port;
	struct intel_crtc *intel_crtc;
	int ret = -EINVAL;

	if (!disp_path) {
		gvt_err("vgpu-%d invalid vgpu display path\n", vgpu->id);
		return ret;
	}

	intel_vgpu_init_i2c_edid(vgpu, &disp_path->i2c_edid);

	disp_path->p_pipe = intel_gvt_pipe_from_port(vgpu->gvt->dev_priv, disp_path->p_port);
	disp_path->pipe = disp_path->id - 1 + PIPE_A;
	disp_path->trans = disp_path->id - 1 + TRANSCODER_A;
	if (IS_GEN9_LP(dev_priv)) {
		disp_path->port = disp_path->id - 1 + PORT_A;
		if (disp_path->port == PORT_A) {
			disp_path->port_type = INTEL_OUTPUT_EDP;
			disp_path->trans = TRANSCODER_EDP;
		}

	} else {
		disp_path->port = disp_path->id - 1 + PORT_B;
		disp_path->port_type = INTEL_OUTPUT_DP;
	}

	clear_bit(INTEL_GVT_DIRECT_DISPLAY_HW_VSYNC, (void *)&disp_path->stat);
	disp_path->sw_vsync_injected = 0;
	disp_path->vsync_interval_ns = vgpu->gvt->irq.vblank_timer.period;
	disp_path->last_hwvsync_ns = 0;

	if (disp_path->p_port == PORT_NONE) {
		ret = setup_virtual_dp_monitor_edid(vgpu, disp_path, NULL);
		gvt_dbg_dpy("Virtual DP monitor is assigned to vgpu-%d PORT_%c\n",
			    vgpu->id, port_name(disp_path->port));
	} else {
		gvt_dbg_dpy("Host PIPE_%c PORT_%c is assigned to vgpu-%d PORT_%c\n",
			    pipe_name(disp_path->p_pipe), port_name(disp_path->p_port),
			    vgpu->id, port_name(disp_path->port));
	}

	drm_connector_list_iter_begin(&vgpu->gvt->dev_priv->drm, &conn_iter);
	for_each_intel_connector_iter(connector, &conn_iter) {
		dig_port = enc_to_dig_port(&connector->encoder->base);
		if (connector->detect_edid && dig_port) {
			if (dig_port->base.port == disp_path->p_port) {
				ret = setup_virtual_dp_monitor_edid(vgpu, disp_path, connector->detect_edid);
			}
		}
	}
	drm_connector_list_iter_end(&conn_iter);

	for_each_intel_crtc(&vgpu->gvt->dev_priv->drm, intel_crtc) {
		drm_modeset_lock(&intel_crtc->base.mutex, NULL);
		if (disp_path->p_pipe == intel_crtc->pipe) {
			struct drm_display_mode *mode = &(&intel_crtc->base)->mode;

			if (mode->vrefresh)
				disp_path->vsync_interval_ns = NSEC_PER_SEC / mode->vrefresh;
			drm_modeset_unlock(&intel_crtc->base.mutex);
			break;
		}
		drm_modeset_unlock(&intel_crtc->base.mutex);
	}

	if (vgpu->gvt->irq.vblank_timer.period > disp_path->vsync_interval_ns) {
		gvt_dbg_dpy("vgpu:%d vblank timer freq (%lld) is slower than HW vsync freq (%lld) on PIPE_%c\n",
			    vgpu->id, vgpu->gvt->irq.vblank_timer.period,
			    disp_path->vsync_interval_ns,
			    pipe_name(disp_path->p_pipe));
	}

	// Init d0 regs from HW in case they are not updated by i915 after VBIOS
	if (disp_path->p_pipe != INVALID_PIPE)
		direct_display_init_d0_regs(vgpu->gvt, disp_path->p_pipe);

	return ret;
}

void clean_vgpu_virtual_display_path(struct intel_vgpu *vgpu,
				     struct intel_vgpu_display_path *disp_path)
{
	if (!disp_path) {
		gvt_err("vgpu-%d invalid vgpu display path\n", vgpu->id);
		return;
	}

	clean_virtual_dp_monitor(vgpu, disp_path);

	disp_path->id = 0;
	disp_path->resolution = GVT_EDID_NUM;
	disp_path->edid_id = 0;
	disp_path->p_port = PORT_NONE;
	disp_path->p_pipe = INVALID_PIPE;
	disp_path->port = PORT_NONE;
	disp_path->pipe = INVALID_PIPE;
	clear_bit(INTEL_GVT_DIRECT_DISPLAY_HW_VSYNC, (void *)&disp_path->stat);
	disp_path->sw_vsync_injected = 0;
	disp_path->last_hwvsync_ns = 0;

	gvt_dbg_dpy("vgpu-%d virtual display path-%d cleaned\n",
		    vgpu->id, disp_path->id);
}

/**
 * intel_vgpu_emulate_hotplug - trigger hotplug event for vGPU
 * @vgpu: a vGPU
 * @connected: link state
 *
 * This function is used to trigger hotplug interrupt for vGPU
 *
 */
void intel_vgpu_emulate_hotplug(struct intel_vgpu *vgpu, bool connected)
{
	struct drm_i915_private *dev_priv = vgpu->gvt->dev_priv;

	/* TODO: add more platforms support */
	if (IS_SKYLAKE(dev_priv) || IS_KABYLAKE(dev_priv) ||
		IS_COFFEELAKE(dev_priv)) {
		if (connected) {
			vgpu_vreg_t(vgpu, SFUSE_STRAP) |=
				SFUSE_STRAP_DDID_DETECTED;
			vgpu_vreg_t(vgpu, SDEISR) |= SDE_PORTD_HOTPLUG_CPT;
		} else {
			vgpu_vreg_t(vgpu, SFUSE_STRAP) &=
				~SFUSE_STRAP_DDID_DETECTED;
			vgpu_vreg_t(vgpu, SDEISR) &= ~SDE_PORTD_HOTPLUG_CPT;
		}
		vgpu_vreg_t(vgpu, SDEIIR) |= SDE_PORTD_HOTPLUG_CPT;
		vgpu_vreg_t(vgpu, PCH_PORT_HOTPLUG) |=
				PORTD_HOTPLUG_STATUS_MASK;
		intel_vgpu_trigger_virtual_event(vgpu, DP_D_HOTPLUG);
	}
}

/**
 * intel_vgpu_clean_display - clean vGPU virtual display emulation
 * @vgpu: a vGPU
 *
 * This function is used to clean vGPU virtual display emulation stuffs
 *
 */
void intel_vgpu_clean_display(struct intel_vgpu *vgpu)
{
	struct intel_vgpu_display *disp_cfg = &vgpu->disp_cfg;
	struct intel_vgpu_display_path *disp_path = NULL, *n;

	list_for_each_entry_safe(disp_path, n, &disp_cfg->path_list, list) {
		clean_vgpu_virtual_display_path(vgpu, disp_path);
		list_del_init(&disp_path->list);
		kfree(disp_path);
	}
}

/**
 * intel_vgpu_init_display- initialize vGPU virtual display emulation
 * @vgpu: a vGPU
 * @resolution: resolution index for intel_vgpu_edid
 *
 * This function is used to initialize vGPU virtual display emulation stuffs
 *
 * Returns:
 * Zero on success, negative error code if failed.
 *
 */
int intel_vgpu_init_display(struct intel_vgpu *vgpu, u64 resolution)
{
	struct intel_vgpu_display *disp_cfg = &vgpu->disp_cfg;
	struct intel_vgpu_display_path *disp_path = NULL, *n;
	enum port port = PORT_NONE;
	u64 *port_mask = &vgpu->gvt->sel_disp_port_mask;
	u32 path = 0;
	int ret = -EINVAL;

	memset(disp_cfg, 0, sizeof(*disp_cfg));

	INIT_LIST_HEAD(&disp_cfg->path_list);
	mutex_init(&disp_cfg->sw_lock);

	// Check the number of host display pipe/port assigned and create paths.
	// If no host pipe/port assigned, create 1 path for virtual display.
	for (port = PORT_A; port < I915_MAX_PORTS; port++) {
		if ((*port_mask >> (vgpu->id - 1) * 8) & 0xFF & (1 << port)) {
			path++;
			disp_path = kmalloc(sizeof(*disp_path), GFP_KERNEL);
			if (unlikely(!disp_path)) {
				gvt_vgpu_err("vgpu-%d alloc display path failed\n", vgpu->id);
				goto out_free_list;
			}
			memset(disp_path, 0, sizeof(*disp_path));
			disp_path->id = path;
			disp_path->resolution = GVT_EDID_NUM;
			disp_path->edid_id = GVT_EDID_NUM;
			disp_path->foreground = false;
			disp_path->foreground_state = false;
			disp_path->p_port = port;
			INIT_LIST_HEAD(&disp_path->list);
			list_add_tail(&disp_path->list, &disp_cfg->path_list);
		}
	}

	if (!disp_path) {
		disp_path = kmalloc(sizeof(*disp_path), GFP_KERNEL);
		if (unlikely(!disp_path)) {
			gvt_vgpu_err("vgpu-%d alloc display path failed\n", vgpu->id);
			goto out_free_list;
		}
		memset(disp_path, 0, sizeof(*disp_path));
		disp_path->id = 1;
		disp_path->resolution = resolution;
		disp_path->edid_id = resolution;
		disp_path->foreground = false;
		disp_path->foreground_state = false;
		disp_path->p_port = PORT_NONE;
		INIT_LIST_HEAD(&disp_path->list);
		list_add_tail(&disp_path->list, &disp_cfg->path_list);
	}

	list_for_each_entry_safe(disp_path, n, &disp_cfg->path_list, list) {
		ret = setup_vgpu_virtual_display_path(vgpu, disp_path);
		if (ret != 0) {
			gvt_dbg_dpy("vgpu-%d setup virtual display path-%d failed:%d\n",
				    vgpu->id, disp_path->id, ret);
			goto out_free_list;
		}
	}

	if (ret == 0)
		emulate_monitor_status_change(vgpu);

	return ret;

out_free_list:
	intel_vgpu_clean_display(vgpu);
	return -ENOMEM;
}

/**
 * intel_vgpu_reset_display- reset vGPU virtual display emulation
 * @vgpu: a vGPU
 *
 * This function is used to reset vGPU virtual display emulation stuffs
 *
 */
void intel_vgpu_reset_display(struct intel_vgpu *vgpu)
{
	emulate_monitor_status_change(vgpu);
}

u8 intel_gvt_external_disp_id_from_port(enum port port)
{
	if (port > PORT_NONE && port < I915_MAX_PORTS)
		return (u8)(port + 1);
	else
		return 0;
}

enum port intel_gvt_port_from_external_disp_id(u8 port_id)
{
	enum port port = PORT_NONE;

	if (port_id)
		port = (enum port)(port_id - 1);
	return port;
}

u64 intel_gvt_port_to_mask_bit(u8 port_sel, enum port port)
{
	return ((u64)port_sel << port * 4);
}

enum pipe intel_gvt_pipe_from_port(struct drm_i915_private *dev_priv,
				   enum port port)
{
	enum pipe pipe = INVALID_PIPE;
	struct drm_device *dev = &dev_priv->drm;
	struct intel_crtc *crtc_intel;
	struct drm_crtc *crtc_drm;
	struct intel_encoder *intel_encoder;

	if (port == PORT_NONE)
		return INVALID_PIPE;

	for_each_intel_crtc(dev, crtc_intel) {
		struct intel_crtc_state *pipe_config;

		drm_modeset_lock(&crtc_intel->base.mutex, NULL);
		pipe_config = to_intel_crtc_state(crtc_intel->base.state);
		if (pipe_config->hw.active && pipe == INVALID_PIPE) {
			crtc_drm = &crtc_intel->base;
			for_each_encoder_on_crtc(dev, crtc_drm, intel_encoder) {
				if (intel_encoder->port == port &&
				    pipe == INVALID_PIPE) {
					pipe = crtc_intel->pipe;
				}
			}
		}
		drm_modeset_unlock(&crtc_intel->base.mutex);
	}

	if (pipe != INVALID_PIPE) {
		gvt_dbg_dpy("PORT_%c is connected to PIPE_%d\n",
			    port_name(port), pipe);
	} else {
		gvt_dbg_dpy("PORT_%c isn't assigned to any pipe\n",
			    pipe_name(port));
	}

	return pipe;
}

enum port intel_gvt_port_from_pipe(struct drm_i915_private *dev_priv,
				   enum pipe pipe)
{
	enum port port = PORT_NONE;
	struct drm_device *dev = &dev_priv->drm;
	struct intel_crtc *crtc_intel;
	struct drm_crtc *crtc_drm;
	struct intel_encoder *intel_encoder;

	if (pipe == INVALID_PIPE)
		gvt_err("Unable to find port from INVALID_PIPE\n");

	for_each_intel_crtc(dev, crtc_intel) {
		struct intel_crtc_state *pipe_config;

		drm_modeset_lock(&crtc_intel->base.mutex, NULL);
		pipe_config = to_intel_crtc_state(crtc_intel->base.state);
		if (pipe_config->hw.active && pipe == crtc_intel->pipe) {
			crtc_drm = &crtc_intel->base;
			for_each_encoder_on_crtc(dev, crtc_drm, intel_encoder) {
				if (port == PORT_NONE)
					port = intel_encoder->port;
			}
		}
		drm_modeset_unlock(&crtc_intel->base.mutex);
	}

	if (port != PORT_NONE)
		gvt_dbg_dpy("PIPE_%c is attached PORT_%c\n",
			    pipe_name(pipe), port_name(port));
	else
		gvt_dbg_dpy("No port attached to PIPE_%c\n", pipe_name(pipe));

	return port;
}

int skl_format_to_fourcc(int format, bool rgb_order, bool alpha);
uint_fixed_16_16_t
skl_wm_method1(const struct drm_i915_private *dev_priv, uint32_t pixel_rate,
	       uint8_t cpp, uint32_t latency, uint32_t dbuf_block_size);
uint_fixed_16_16_t
skl_wm_method2(uint32_t pixel_rate,
	       uint32_t pipe_htotal,
	       uint32_t latency,
	       uint_fixed_16_16_t plane_blocks_per_line);
uint_fixed_16_16_t intel_get_linetime_us(struct intel_crtc_state *cstate);

int
vgpu_compute_plane_wm_params(struct intel_vgpu *vgpu,
			    struct intel_crtc_state *intel_cstate,
			    enum plane_id plane,
			    struct skl_wm_params *wp)
{
	struct intel_gvt *gvt = vgpu->gvt;
	struct drm_i915_private *dev_priv = gvt->dev_priv;
	struct intel_crtc *crtc = to_intel_crtc(intel_cstate->uapi.crtc);
	struct intel_plane *prim_plane = to_intel_plane(crtc->base.primary);
	struct intel_plane_state *prim_pstate = to_intel_plane_state(prim_plane->base.state);
	uint32_t interm_pbpl;
	const struct drm_format_info *info;
	u64 original_pixel_rate;
	uint_fixed_16_16_t downscale_amount;
	u32 pipe_src_w, pipe_src_h, src_w, src_h, dst_w, dst_h;
	uint_fixed_16_16_t fp_w_ratio, fp_h_ratio;
	uint_fixed_16_16_t downscale_h, downscale_w;
	bool apply_memory_bw_wa = IS_GEN9_BC(dev_priv) || IS_BROXTON(dev_priv);
	bool rot_90_or_270;
	int scaler, plane_scaler;
	u32 reg_val;
	struct intel_vgpu_display *disp_cfg = &vgpu->disp_cfg;
	struct intel_vgpu_display_path *disp_path = NULL, *n;
	enum pipe host_pipe = INVALID_PIPE;
	enum pipe vgpu_pipe = INVALID_PIPE;

	if (!intel_cstate->hw.active || !prim_pstate->uapi.visible)
		return 0;

	list_for_each_entry_safe(disp_path, n, &disp_cfg->path_list, list)
		if (disp_path->p_pipe == crtc->pipe)
			break;

	if (!disp_path) {
		gvt_err("vgpu-%d invalid vgpu display path\n", vgpu->id);
		return -EINVAL;
	}

	host_pipe = disp_path->p_pipe;
	vgpu_pipe = disp_path->pipe;

	reg_val = vgpu_vreg_t(vgpu, PIPESRC(disp_path->trans));
	pipe_src_w = ((reg_val >> 16) & 0xfff) + 1;
	pipe_src_h = (reg_val & 0xfff) + 1;
	original_pixel_rate = intel_cstate->pixel_rate;

	rot_90_or_270 = false;
	if (plane == PLANE_CURSOR) {
		reg_val = vgpu_vreg_t(vgpu, CURCNTR(vgpu_pipe));
		switch (reg_val & SKL_CURSOR_MODE_MASK) {
		case 0x04:
		case 0x05:
		case 0x06:
		case 0x07:
		case 0x24:
		case 0x27:
			wp->width = 64;
			break;
		case 0x02:
		case 0x22:
		case 0x25:
			wp->width = 128;
			break;
		case 0x03:
		case 0x23:
		case 0x26:
			wp->width = 256;
			break;
		case 0:
			wp->width = 0;
			gvt_dbg_dpy("vgpu-%d: pipe(%d->%d) HW cursor is disabled\n",
				    vgpu->id, vgpu_pipe, host_pipe);
			return 0;
		default:
			wp->width = 0;
			gvt_dbg_dpy("vgpu-%d: pipe(%d->%d) unsupported HW cursor mode %x\n",
				    vgpu->id, vgpu_pipe, host_pipe, reg_val & SKL_CURSOR_MODE_MASK);
			return 0;
		}

		// Cursor is always linear
		wp->x_tiled = 0;
		wp->y_tiled = 0;
		wp->rc_surface = 0;

		switch (reg_val & SKL_CURSOR_MODE_MASK) {
		// 32bpp AND/INVERT
		case 0x02:
		case 0x03:
		case 0x07:
		// 32bpp ARGB
		case 0x22:
		case 0x23:
		case 0x27:
		// 32bpp AND/XOR
		case 0x24:
		case 0x25:
		case 0x26:
			wp->cpp = 4;
			break;
		// 2bpp 3/2/4 color
		case 0x04:
		case 0x05:
		case 0x06:
		default:
			wp->width = 0;
			wp->cpp = 0;
			gvt_dbg_dpy("vgpu-%d: pipe(%d->%d) unsupported HW cursor format %x\n",
				    vgpu->id, vgpu_pipe, host_pipe, reg_val & SKL_CURSOR_MODE_MASK);
			return 0;
		}
	} else {
		reg_val = vgpu_vreg_t(vgpu, PLANE_CTL(vgpu_pipe, plane));

		wp->x_tiled = (reg_val & PLANE_CTL_TILED_MASK) == PLANE_CTL_TILED_X;
		/*
		* Check if Y tiled is one of the below:
		* I915_FORMAT_MOD_Y_TILED
		* I915_FORMAT_MOD_Yf_TILED
		* I915_FORMAT_MOD_Y_TILED_CCS
		* I915_FORMAT_MOD_Yf_TILED_CCS
		*/
		wp->y_tiled = ((reg_val & PLANE_CTL_TILED_MASK) == PLANE_CTL_TILED_Y) ||
			((reg_val & PLANE_CTL_TILED_MASK) == PLANE_CTL_TILED_YF);
		if ((reg_val & PLANE_CTL_RENDER_DECOMPRESSION_ENABLE) && wp->y_tiled)
			wp->rc_surface = 1;
		else
			wp->rc_surface = 0;


		info = drm_format_info(
			skl_format_to_fourcc(reg_val & PLANE_CTL_FORMAT_MASK,
					     reg_val & PLANE_CTL_ORDER_RGBX,
					     reg_val & PLANE_CTL_ALPHA_MASK));
		wp->cpp = info->cpp[0];

		reg_val &= PLANE_CTL_ROTATE_MASK;
		if (reg_val == PLANE_CTL_ROTATE_90 || reg_val == PLANE_CTL_ROTATE_270)
			rot_90_or_270 = true;
	}

	if (plane == PLANE_CURSOR) {
		src_w = wp->width;
		src_h = wp->width;
	} else {
		reg_val = vgpu_vreg_t(vgpu, PLANE_SIZE(vgpu_pipe, plane));
		src_w = (reg_val & 0xfff) + 1;
		src_h = ((reg_val >> 16) & 0xfff) + 1;
		if (rot_90_or_270)
			wp->width = src_h;
		else
			wp->width = src_w;
	}

	// Assume host scaler has been rebuilt for vgpu
	plane_scaler = -1;
	for (scaler = 0; scaler < crtc->num_scalers; scaler++) {
		reg_val = disp_path->scaler_cfg.ctrl[scaler];

		if (reg_val & PS_SCALER_EN &&
		    (reg_val & PS_PLANE_SEL(plane) ||
		    !(reg_val & PS_PLANE_SEL_MASK))) {
			plane_scaler = scaler;
			break;
		}
	}

	if (plane_scaler >= 0) {
		reg_val = disp_path->scaler_cfg.win_size[scaler];
		dst_w = reg_val >> 16 & 0xfff;
		dst_h = reg_val & 0xfff;
	} else {
		dst_w = drm_rect_width(&prim_pstate->uapi.dst);
		dst_h = drm_rect_height(&prim_pstate->uapi.dst);
	}

	fp_w_ratio = div_fixed16(src_w, dst_w);
	fp_h_ratio = div_fixed16(src_h, dst_h);
	downscale_w = max_fixed16(fp_w_ratio, u32_to_fixed16(1));
	downscale_h = max_fixed16(fp_h_ratio, u32_to_fixed16(1));
	downscale_amount = mul_fixed16(downscale_w, downscale_h);

	wp->plane_pixel_rate = mul_round_up_u32_fixed16(original_pixel_rate,
						    downscale_amount);

	gvt_dbg_dpy("vgpu-%d: pipe(%d->%d), plane(%d), plane_ctl(%08x), scaler-%d, pipe src(%dx%d) src(%dx%d)->dst(%dx%d), pixel rate(%lld->%d)\n",
		    vgpu->id, vgpu_pipe, host_pipe, plane,
		    (plane == PLANE_CURSOR) ? vgpu_vreg_t(vgpu, CURCNTR(vgpu_pipe)) : vgpu_vreg_t(vgpu, PLANE_CTL(vgpu_pipe, plane)),
		    plane_scaler, pipe_src_w, pipe_src_h,
		    src_w, src_h, dst_w, dst_h, original_pixel_rate, wp->plane_pixel_rate);

	reg_val = vgpu_vreg_t(vgpu, PLANE_CTL(vgpu_pipe, plane));
	if (INTEL_GEN(dev_priv) >= 11 &&
	    plane != PLANE_CURSOR &&
	    !(reg_val & PLANE_CTL_RENDER_DECOMPRESSION_ENABLE) &&
	    (reg_val & PLANE_CTL_TILED_YF) &&
	    wp->cpp == 8)
		wp->dbuf_block_size = 256;
	else
		wp->dbuf_block_size = 512;

	if (rot_90_or_270) {
		switch (wp->cpp) {
		case 1:
			wp->y_min_scanlines = 16;
			break;
		case 2:
			wp->y_min_scanlines = 8;
			break;
		case 4:
			wp->y_min_scanlines = 4;
			break;
		default:
			MISSING_CASE(wp->cpp);
			return -EINVAL;
		}
	} else {
		wp->y_min_scanlines = 4;
	}

	if (apply_memory_bw_wa)
		wp->y_min_scanlines *= 2;

	wp->plane_bytes_per_line = wp->width * wp->cpp;
	if (wp->y_tiled) {
		interm_pbpl = DIV_ROUND_UP(wp->plane_bytes_per_line *
					   wp->y_min_scanlines,
					   wp->dbuf_block_size);

		if (INTEL_GEN(dev_priv) >= 10)
			interm_pbpl++;

		wp->plane_blocks_per_line = div_fixed16(interm_pbpl,
							wp->y_min_scanlines);
	} else if (wp->x_tiled && IS_GEN(dev_priv, 9)) {
		interm_pbpl = DIV_ROUND_UP(wp->plane_bytes_per_line,
					   wp->dbuf_block_size);
		wp->plane_blocks_per_line = u32_to_fixed16(interm_pbpl);
	} else {
		interm_pbpl = DIV_ROUND_UP(wp->plane_bytes_per_line,
					   wp->dbuf_block_size) + 1;
		wp->plane_blocks_per_line = u32_to_fixed16(interm_pbpl);
	}

	wp->y_tile_minimum = mul_u32_fixed16(wp->y_min_scanlines,
					     wp->plane_blocks_per_line);
	wp->linetime_us = fixed16_to_u32_round_up(
					intel_get_linetime_us(intel_cstate));

	gvt_dbg_dpy("vgpu-%d: pipe(%d->%d), plane(%d), x_tiled(%d), y_tiled(%d), rc_surface(%d), width(%x), cpp(%x), "
		    "plane_pixel_rate(%d), y_min_scanlines(%x), plane_bytes_per_line(%x), plane_blocks_per_line(%x), "
		    "y_tile_minimum(%x), linetime_us(%x), dbuf_block_size(%x)\n",
		    vgpu->id, vgpu_pipe, host_pipe, plane, wp->x_tiled, wp->y_tiled, wp->rc_surface, wp->width, wp->cpp,
		    wp->plane_pixel_rate, wp->y_min_scanlines, wp->plane_bytes_per_line, wp->plane_blocks_per_line.val,
		    wp->y_tile_minimum.val, wp->linetime_us, wp->dbuf_block_size);

	return 0;
}

int vgpu_compute_plane_wm(struct intel_vgpu *vgpu,
			  struct intel_crtc_state *intel_cstate,
			  enum plane_id plane,
			  u16 ddb_allocation,
			  int level,
			  const struct skl_wm_params *wp,
			  u16 *out_blocks, u8 *out_lines, bool *enabled)
{
	struct intel_gvt *gvt = vgpu->gvt;
	struct drm_i915_private *dev_priv = gvt->dev_priv;
	u32 latency = dev_priv->wm.skl_latency[level];
	uint_fixed_16_16_t method1, method2;
	uint_fixed_16_16_t selected_result;
	u32 res_blocks, res_lines;
	bool apply_memory_bw_wa = IS_GEN9_BC(dev_priv) || IS_BROXTON(dev_priv);
	uint32_t min_disp_buf_needed;

	if (latency == 0 ||
	    !intel_cstate->hw.active ||
	    wp->width == 0)
		return 0;

	/* Display WA #1141: kbl,cfl */
	if ((IS_KABYLAKE(dev_priv) || IS_COFFEELAKE(dev_priv) ||
	    IS_CNL_REVID(dev_priv, CNL_REVID_A0, CNL_REVID_B0)) &&
	    dev_priv->ipc_enabled)
		latency += 4;

	if (apply_memory_bw_wa && wp->x_tiled)
		latency += 15;

	method1 = skl_wm_method1(dev_priv, wp->plane_pixel_rate,
				 wp->cpp, latency, wp->dbuf_block_size);
	method2 = skl_wm_method2(wp->plane_pixel_rate,
				 intel_cstate->hw.adjusted_mode.crtc_htotal,
				 latency,
				 wp->plane_blocks_per_line);

	if (wp->y_tiled) {
		selected_result = max_fixed16(method2, wp->y_tile_minimum);
	} else {
		if ((wp->cpp * intel_cstate->hw.adjusted_mode.crtc_htotal /
		     wp->dbuf_block_size < 1) &&
		     (wp->plane_bytes_per_line / wp->dbuf_block_size < 1))
			selected_result = method2;
		else if (ddb_allocation >=
			 fixed16_to_u32_round_up(wp->plane_blocks_per_line))
			selected_result = min_fixed16(method1, method2);
		else if (latency >= wp->linetime_us)
			selected_result = min_fixed16(method1, method2);
		else
			selected_result = method1;
	}

	res_blocks = fixed16_to_u32_round_up(selected_result) + 1;
	res_lines = div_round_up_fixed16(selected_result,
					 wp->plane_blocks_per_line);

	/* Display WA #1125: skl,bxt,kbl,glk */
	if (level == 0 && wp->rc_surface)
		res_blocks += fixed16_to_u32_round_up(wp->y_tile_minimum);

	/* Display WA #1126: skl,bxt,kbl,glk */
	if (level >= 1 && level <= 7) {
		if (wp->y_tiled) {
			res_blocks += fixed16_to_u32_round_up(
							wp->y_tile_minimum);
			res_lines += wp->y_min_scanlines;
		} else {
			res_blocks++;
		}
	}

	if (INTEL_GEN(dev_priv) >= 11) {
		if (wp->y_tiled) {
			uint32_t extra_lines;
			uint_fixed_16_16_t fp_min_disp_buf_needed;

			if (res_lines % wp->y_min_scanlines == 0)
				extra_lines = wp->y_min_scanlines;
			else
				extra_lines = wp->y_min_scanlines * 2 -
					      res_lines % wp->y_min_scanlines;

			fp_min_disp_buf_needed = mul_u32_fixed16(res_lines +
						extra_lines,
						wp->plane_blocks_per_line);
			min_disp_buf_needed = fixed16_to_u32_round_up(
						fp_min_disp_buf_needed);
		} else {
			min_disp_buf_needed = DIV_ROUND_UP(res_blocks * 11, 10);
		}
	} else {
		min_disp_buf_needed = res_blocks;
	}

	if ((level > 0 && res_lines > 31) ||
	    res_blocks >= ddb_allocation ||
	    min_disp_buf_needed >= ddb_allocation) {
		gvt_dbg_dpy("Requested display configuration exceeds system watermark limitations\n");
		gvt_dbg_dpy("[PLANE:%d:%c] blocks required = %u/%u, lines required = %u/31\n",
			    plane, plane_name(plane),
			    res_blocks, ddb_allocation, res_lines);
	}

	/* The number of lines are ignored for the level 0 watermark. */
	*out_lines = level ? res_lines : 0;
	*out_blocks = res_blocks;
	*enabled = true;

	return 0;
}

void skl_compute_transition_wm(struct intel_crtc_state *cstate,
				      struct skl_wm_params *wp,
				      struct skl_wm_level *wm_l0,
				      uint16_t ddb_allocation,
				      struct skl_wm_level *trans_wm /* out */);

inline u32 vgpu_calc_wm_level(const struct skl_wm_level *level)
{
	u32 val = 0;

	if (level->plane_en) {
		val |= PLANE_WM_EN;
		val |= level->plane_res_b;
		val |= level->plane_res_l << PLANE_WM_LINES_SHIFT;
	}
	return val;
}

void intel_vgpu_update_plane_scaler(struct intel_vgpu *vgpu,
				    struct intel_crtc *intel_crtc, enum plane_id plane)
{
	struct intel_gvt *gvt = vgpu->gvt;
	struct drm_i915_private *dev_priv = gvt->dev_priv;
	struct intel_runtime_info *runtime = RUNTIME_INFO(dev_priv);
	struct intel_dom0_plane_regs *dom0_regs;
	struct drm_display_mode *mode = NULL;
	struct intel_vgpu_display *disp_cfg = &vgpu->disp_cfg;
	struct intel_vgpu_display_path *disp_path = NULL, *n;
	enum pipe host_pipe = INVALID_PIPE;
	enum pipe vgpu_pipe = INVALID_PIPE;
	u32 host_hactive = 0, host_vactive = 0;
	u32 host_plane_w, host_plane_h;
	u32 vreg;
	u32 vgpu_hactive, vgpu_vactive;
	u32 vgpu_plane_w, vgpu_plane_h;
	u32 ps_win_pos[2], ps_win_size[2], ps_ctrl[2];
	int max_scaler;
	int scaler, vps_id, hps_id = 0;
	bool scaler_passthru;

	if (plane == PLANE_PRIMARY)
		hps_id = 0;
	else
		return;

	list_for_each_entry_safe(disp_path, n, &disp_cfg->path_list, list)
		if (disp_path->p_pipe == intel_crtc->pipe)
			break;

	if (!disp_path) {
		gvt_err("vgpu-%d invalid vgpu display path\n", vgpu->id);
		return;
	}

	host_pipe = disp_path->p_pipe;
	max_scaler = runtime->num_scalers[host_pipe];
	vgpu_pipe = disp_path->pipe;

	// Get host h/v active from active crtc status
	mode = &(&intel_crtc->base)->mode;
	if (mode) {
		host_hactive = mode->hdisplay;
		host_vactive = mode->vdisplay;
	} else {
		host_hactive = 0;
		host_vactive = 0;
		gvt_dbg_dpy("vgpu-%d: host PIPE %c h/v active is invalid (%dx%d)\n",
			    vgpu->id, pipe_name(host_pipe), host_hactive, host_vactive);
		return;
	}

	// Get host plane size from cache
	dom0_regs = &gvt->pipe_info[host_pipe].plane_info[plane].dom0_regs;
	host_plane_w = (dom0_regs->plane_size & 0xfff) + 1;
	host_plane_h = ((dom0_regs->plane_size >> 16) & 0xfff) + 1;

	// Decode vgpu h/v active from vgpu timing
	vreg = vgpu_vreg_t(vgpu, HTOTAL(disp_path->trans));
	vgpu_hactive = (vreg & 0xfff) + 1;
	vreg = vgpu_vreg_t(vgpu, VTOTAL(disp_path->trans));
	vgpu_vactive = (vreg & 0xfff) + 1;

	// Decode vgpu plane size
	vreg = vgpu_vreg_t(vgpu, PLANE_SIZE(vgpu_pipe, plane));
	vgpu_plane_w = (vreg & 0xfff) + 1;
	vgpu_plane_h = ((vreg >> 16) & 0xfff) + 1;

	gvt_dbg_dpy("vgpu-%d: PIPE %c->%c h/v active (%dx%d)->(%dx%d)\n",
		    vgpu->id, pipe_name(vgpu_pipe), pipe_name(host_pipe),
		    vgpu_hactive, vgpu_vactive, host_hactive, host_vactive);
	gvt_dbg_dpy("vgpu-%d: PIPE %c->%c plane-%d size (%dx%d)->(%dx%d)\n",
		    vgpu->id, pipe_name(vgpu_pipe), pipe_name(host_pipe), plane,
		    vgpu_plane_w, vgpu_plane_h, host_plane_w, host_plane_h);

	vps_id = -1;
	for (scaler = 0; scaler < max_scaler; scaler++) {
		ps_win_pos[scaler] =
			vgpu_vreg_t(vgpu, SKL_PS_WIN_POS(vgpu_pipe, scaler));
		ps_win_size[scaler] =
			vgpu_vreg_t(vgpu, SKL_PS_WIN_SZ(vgpu_pipe, scaler));
		ps_ctrl[scaler] =
			vgpu_vreg_t(vgpu, SKL_PS_CTRL(vgpu_pipe, scaler));

		// Find which vgpu scaler is assigned to current plane or pipe
		if (ps_ctrl[scaler] & PS_SCALER_EN &&
		    (ps_ctrl[scaler] & PS_PLANE_SEL(plane) ||
		    !(ps_ctrl[scaler] & PS_PLANE_SEL_MASK)))
			vps_id = scaler;

		gvt_dbg_dpy("vgpu-%d: PIPE %c->%c scaler-%d PS_CTRL(%08x) PS_WIN_POS(%08x) PS_WIN_SZ(%08x)\n",
			    vgpu->id, pipe_name(vgpu_pipe),
			    pipe_name(host_pipe), scaler, ps_ctrl[scaler],
			    ps_win_pos[scaler], ps_win_size[scaler]);
	}

	/*
	 * Decode vgpu scaler settings:
	 * Assume host is always set to resolution matching its preferred timing.
	 * If vgpu matches host timing:
	 * - If scaler is enabled, surf can be stretched or centered (with offset).
	 * - If scaler is not enabled, surf can be identitied or stretched by SW.
	 * - In both cases we can pass vgpu scaler settings to host.
	 * If vgpu doesn't match host timing:
	 * - If scaler is enabled, then surf is using custom scaling ratio.
	 *   We should decode the vgpu scaling ratio and apply to host scaler.
	 * - If scaler is not enabled, surf is identitied to vgpu timing which is
	 *   expected to be fullscreen. We should enable host scaler to stretch.
	 * - In both cases we should ignore vgpu scaler settings and recalculate
	 *   new settings for host.
	 */
	if (host_hactive == vgpu_hactive && host_vactive == vgpu_vactive) {
		// If no scaler assigned to selected plane, disable the host scaler
		disp_path->scaler_cfg.win_pos[hps_id] = 0;
		disp_path->scaler_cfg.win_size[hps_id] = 0;
		disp_path->scaler_cfg.ctrl[hps_id] = 0;
		if (vps_id >= 0) {
			disp_path->scaler_cfg.win_pos[hps_id] = ps_win_pos[vps_id];
			disp_path->scaler_cfg.win_size[hps_id] = ps_win_size[vps_id];
			disp_path->scaler_cfg.ctrl[hps_id] = ps_ctrl[vps_id];
		}
		scaler_passthru = true;
	} else {
		disp_path->scaler_cfg.win_pos[hps_id] = 0;
		disp_path->scaler_cfg.win_size[hps_id] = (host_plane_w << 16) | host_plane_h;
		disp_path->scaler_cfg.ctrl[hps_id] = PS_SCALER_EN;

		if (vps_id >= 0) {
			// Decode vgpu custom scaling ratio and calcuate host ratio
			// Only handle downscaling case, i.e. plane size < h/v active
			u32 vgpu_dst_w = ps_win_size[vps_id] >> 16 & 0xfff;
			u32 vgpu_dst_h = ps_win_size[vps_id] & 0xfff;

			if (vgpu_dst_w <= vgpu_hactive && vgpu_dst_h <= vgpu_vactive) {
				u32 x_pct, y_pct, dst_w, dst_h, dst_x, dst_y;
				u32 vgpu_x_scale_limit = vgpu_hactive * 12 / 100;
				u32 vgpu_y_scale_limit = vgpu_vactive * 12 / 100;
				u32 host_x_scale_limit = host_hactive * 12 / 100;
				u32 host_y_scale_limit = host_vactive * 12 / 100;

				x_pct = (vgpu_hactive - vgpu_dst_w) * 100 / vgpu_x_scale_limit;
				y_pct = (vgpu_vactive - vgpu_dst_h) * 100 / vgpu_y_scale_limit;

				dst_w = host_hactive - host_x_scale_limit * x_pct / 100;
				dst_h = host_vactive - host_y_scale_limit * y_pct / 100;
				dst_x = (host_hactive - dst_w) / 2;
				dst_y = (host_vactive - dst_h) / 2;

				disp_path->scaler_cfg.win_pos[hps_id] = (dst_x << 16) | dst_y;
				disp_path->scaler_cfg.win_size[hps_id] = (dst_w << 16) | dst_h;

				gvt_dbg_dpy("vgpu-%d: PIPE %c->%c scaler-%d "
					    "customize aspect ratio from %dx%d -> %dx%d @ %dx%d "
					    "to %dx%d -> %dx%d @ %dx%d, "
					    "x: %d%%, y: %d%%\n",
					    vgpu->id, pipe_name(vgpu_pipe), pipe_name(host_pipe),
					    vps_id, vgpu_hactive, vgpu_vactive,
					    vgpu_dst_w, vgpu_dst_h,
					    ps_win_pos[vps_id] >> 16 & 0xfff,
					    ps_win_pos[vps_id] & 0xfff,
					    host_hactive, host_vactive, dst_w, dst_h,
					    dst_x, dst_y,
					    x_pct, y_pct);
			}
		}
		scaler_passthru = false;
	}

	/*
	 * In a transition state that PLANE_CTL_ENABLE and PS_SCALER_EN are
	 * both on, SKL_PS_WIN_SZ shouldn't be 0, otherwise WM calculation
	 * will hit divide by zero error. Disable scaler during transition
	 * to prevent unexpected scaler and watermark settings.
	 */
	if (disp_path->scaler_cfg.ctrl[hps_id] & PS_SCALER_EN &&
	    ((disp_path->scaler_cfg.win_size[hps_id] >> 16 & 0xfff) == 0 ||
	    (disp_path->scaler_cfg.win_size[hps_id] & 0xfff) == 0)) {
		disp_path->scaler_cfg.win_pos[hps_id] = 0;
		disp_path->scaler_cfg.win_size[hps_id] = 0;
		disp_path->scaler_cfg.ctrl[hps_id] = 0;
		scaler_passthru = false;
		gvt_dbg_dpy("vgpu-%d: disable scaler PIPE %c->%c scaler %d->%d "
			    "for plane-%d during plane state transition",
			    vgpu->id, pipe_name(vgpu_pipe),
			    pipe_name(host_pipe), vps_id, hps_id, plane);
	}

	gvt_dbg_dpy("vgpu-%d: %s PIPE %c->%c scaler %d->%d for plane-%d "
		    "PS_WIN_POS(%08x) PS_WIN_SZ(%08x) PS_CTRL(%08x)\n",
		    vgpu->id, scaler_passthru ? "passthrough" : "override",
		    pipe_name(vgpu_pipe), pipe_name(host_pipe),
		    vps_id, hps_id, plane, disp_path->scaler_cfg.win_pos[hps_id],
		    disp_path->scaler_cfg.win_size[hps_id],
		    disp_path->scaler_cfg.ctrl[hps_id]);
}

void intel_vgpu_update_plane_wm(struct intel_vgpu *vgpu,
				struct intel_crtc *intel_crtc, enum plane_id plane)
{
	struct intel_gvt *gvt = vgpu->gvt;
	struct drm_i915_private *dev_priv = gvt->dev_priv;
	struct intel_crtc_state *intel_cstate = NULL;
	struct intel_vgpu_display *disp_cfg = &vgpu->disp_cfg;
	struct intel_vgpu_display_path *disp_path = NULL, *n;
	enum pipe host_pipe = INVALID_PIPE;
	enum pipe vgpu_pipe = INVALID_PIPE;
	int level, max_level = ilk_wm_max_level(dev_priv);
	struct skl_ddb_entry *ddb_y;
	u16 ddb_blocks;
	struct skl_plane_wm *wm;
	struct skl_wm_params wm_params;
	int ret;

	list_for_each_entry_safe(disp_path, n, &disp_cfg->path_list, list)
		if (disp_path->p_pipe == intel_crtc->pipe)
			break;

	if (!disp_path) {
		gvt_err("vgpu-%d invalid vgpu display path\n", vgpu->id);
		return;
	}

	host_pipe = disp_path->p_pipe;
	vgpu_pipe = disp_path->pipe;

	intel_cstate = to_intel_crtc_state(intel_crtc->base.state);

	wm = &disp_path->wm_cfg.planes[plane];
	ddb_y = gvt->pipe_info[host_pipe].ddb_y;
	ddb_blocks = skl_ddb_entry_size(&ddb_y[plane]);
	memset(&wm_params, 0, sizeof(struct skl_wm_params));
	ret = vgpu_compute_plane_wm_params(vgpu, intel_cstate,
						  plane, &wm_params);

	for (level = 0; level <= max_level; level++) {
		ret = vgpu_compute_plane_wm(vgpu,
					    intel_cstate,
					    plane,
					    ddb_blocks,
					    level,
					    &wm_params,
					    &wm->wm[level].plane_res_b,
					    &wm->wm[level].plane_res_l,
					    &wm->wm[level].plane_en);
		gvt_dbg_dpy("vgpu-%d: pipe(%d->%d), plane(%d), level(%d), wm(%x)\n",
			    vgpu->id, vgpu_pipe, host_pipe, plane, level,
			    vgpu_calc_wm_level(&wm->wm[level]));
		if (ret)
			break;
	}

	skl_compute_transition_wm(intel_cstate, &wm_params,
		&wm->wm[0], ddb_blocks, &wm->trans_wm);
	gvt_dbg_dpy("vgpu-%d: pipe(%d->%d), plane(%d), wm_trans(%x)\n",
		    vgpu->id, vgpu_pipe, host_pipe, plane,
		    vgpu_calc_wm_level(&wm->trans_wm));
}

static int prepare_for_switch_display(struct intel_gvt *gvt, enum pipe pipe)
{
	const struct intel_crtc *crtc;

	if (!(pipe >= PIPE_A && pipe <= PIPE_C)) {
		gvt_err("Invalid PIPE_%c for v-blank wait\n", pipe_name(pipe));
		return -EINVAL;
	}

	crtc = intel_get_crtc_for_pipe(gvt->dev_priv, pipe);
	if (!crtc->active) {
		gvt_err("CRTC attached to PIPE_%c is not active\n",
			pipe_name(pipe));
		return -EINVAL;
	}

	intel_wait_for_vblank(gvt->dev_priv, pipe);
	return 0;
}

void intel_gvt_switch_display_pipe(struct intel_gvt *gvt, enum pipe pipe,
				   struct intel_vgpu *old_v,
				   struct intel_vgpu *new_v)
{
	struct drm_i915_private *dev_priv = gvt->dev_priv;
	struct intel_runtime_info *runtime = RUNTIME_INFO(dev_priv);
	struct drm_device *dev = &dev_priv->drm;
	struct intel_crtc *crtc = NULL;
	struct intel_dom0_pipe_regs *d0_pipe_regs = NULL;
	struct intel_vgpu_display_path *disp_path = NULL, *disp_path_old, *n;
	enum pipe v_pipe = INVALID_PIPE;
	enum plane_id plane = PLANE_PRIMARY;
	int scaler, level, max_scaler = 0;
	int max_level = ilk_wm_max_level(dev_priv);
	unsigned long irqflags;

	if (pipe == INVALID_PIPE || pipe > PIPE_C) {
		gvt_err("Invalid PIPE_%c to switch\n", pipe_name(pipe));
		return;
	}

	if (!old_v && !new_v) {
		gvt_err("Can't switch display from host to host\n");
		return;
	}

	for_each_intel_crtc(dev, crtc) {
		drm_modeset_lock(&crtc->base.mutex, NULL);
		if (pipe == crtc->pipe)
			break;
		drm_modeset_unlock(&crtc->base.mutex);
	}

	if (!crtc)
		return;

	d0_pipe_regs = &gvt->pipe_info[pipe].dom0_pipe_regs;
	max_scaler = runtime->num_scalers[pipe];

	if (new_v) {
		list_for_each_entry_safe(disp_path, n, &new_v->disp_cfg.path_list, list) {
			if (disp_path->p_pipe == pipe)
				break;
		}

		if (!disp_path)
			return;

		v_pipe = disp_path->pipe;
		mutex_lock(&new_v->disp_cfg.sw_lock);
	}

	if (intel_gvt_switch_pipe_owner(gvt, pipe, new_v ? new_v->id : 0)) {
		gvt_err("Can't flush PIPE_%c due to owner switch fail\n",
			pipe_name(pipe));
		if (new_v)
			mutex_unlock(&new_v->disp_cfg.sw_lock);
		drm_modeset_unlock(&crtc->base.mutex);
		return;
	}

	mmio_hw_access_pre(dev_priv);

	spin_lock_irqsave(&dev_priv->uncore.lock, irqflags);
	for_each_plane_id_on_crtc(crtc, plane) {
		if (plane == PLANE_CURSOR) {
			I915_WRITE_FW(CURCNTR(pipe), 0);
			I915_WRITE_FW(CURBASE(pipe), 0);
			I915_WRITE_FW(CUR_FBC_CTL(pipe), 0);
			intel_uncore_posting_read_fw(&dev_priv->uncore, CURBASE(pipe));
		} else {
			I915_WRITE_FW(PLANE_CTL(pipe, plane), 0);
			I915_WRITE_FW(PLANE_SURF(pipe, plane), 0);
			intel_uncore_posting_read_fw(&dev_priv->uncore, PLANE_SURF(pipe, plane));
		}
	}

	I915_WRITE_FW(PIPESRC(pipe), new_v ?
		      vgpu_vreg_t(new_v, PIPESRC(v_pipe)) :
		      d0_pipe_regs->pipesrc);

	intel_gvt_flush_pipe_color(gvt, pipe, new_v);

	for (scaler = 0; scaler < max_scaler; scaler++) {
		I915_WRITE_FW(SKL_PS_CTRL(pipe, scaler), new_v ?
			      disp_path->scaler_cfg.ctrl[scaler] :
			      d0_pipe_regs->scaler_ctl[scaler]);
		I915_WRITE_FW(SKL_PS_PWR_GATE(pipe, scaler), new_v ? 0 :
			      d0_pipe_regs->scaler_pwr_gate[scaler]);
		I915_WRITE_FW(SKL_PS_WIN_POS(pipe, scaler), new_v ?
			      disp_path->scaler_cfg.win_pos[scaler] :
			      d0_pipe_regs->scaler_win_pos[scaler]);
		I915_WRITE_FW(SKL_PS_WIN_SZ(pipe, scaler), new_v ?
			      disp_path->scaler_cfg.win_size[scaler] :
			      d0_pipe_regs->scaler_win_size[scaler]);
	}

	for_each_plane_id_on_crtc(crtc, plane) {
		u32 ctl = 0, size = 0, pos = 0, offset = 0, stride = 0;
		u32 keyval = 0, keymax = 0, keymsk = 0;
		u32 fbc_ctl = 0, aux_dist = 0, aux_offset = 0;
		u32 wm_level[8] = {0}, wm_trans = 0;
		u32 hma = 0;
		struct intel_dom0_plane_regs *d0_plane_regs =
			&gvt->pipe_info[pipe].plane_info[plane].dom0_regs;

		if (new_v && plane != PLANE_PRIMARY && plane != PLANE_CURSOR)
			continue;

		if (new_v) {
			intel_vgpu_update_plane_wm(new_v, crtc, plane);
			for (level = 0; level <= max_level; level++)
				wm_level[level] = vgpu_calc_wm_level(&disp_path->wm_cfg.planes[plane].wm[level]);
			wm_trans = vgpu_calc_wm_level(&disp_path->wm_cfg.planes[plane].trans_wm);
		} else {
			for (level = 0; level <= max_level; level++)
				wm_level[level] = d0_plane_regs->plane_wm[level];
			wm_trans = d0_plane_regs->plane_wm_trans;
		}

		if (plane == PLANE_CURSOR) {
			if (new_v) {
				pos = vgpu_vreg_t(new_v, CURPOS(v_pipe));
				fbc_ctl = vgpu_vreg_t(new_v, CUR_FBC_CTL(v_pipe));
				ctl = vgpu_vreg_t(new_v, CURCNTR(v_pipe));
				hma = vgpu_vreg_t(new_v, CURBASE(v_pipe));
			} else {
				pos = d0_plane_regs->plane_pos;
				fbc_ctl = d0_plane_regs->cur_fbc_ctl;
				ctl = d0_plane_regs->plane_ctl;
				hma = d0_plane_regs->plane_surf;
			}
			for (level = 0; level <= max_level; level++) {
				I915_WRITE_FW(CUR_WM(pipe, level), wm_level[level]);
			}
			I915_WRITE_FW(CUR_WM_TRANS(pipe), wm_trans);
			I915_WRITE_FW(CURPOS(pipe), pos);
			I915_WRITE_FW(CUR_FBC_CTL(pipe), fbc_ctl);
			I915_WRITE_FW(CURCNTR(pipe), ctl);
			I915_WRITE_FW(CURBASE(pipe), hma);
			intel_uncore_posting_read_fw(&dev_priv->uncore, CURBASE(pipe));
		} else {
			if (new_v) {
				keyval = vgpu_vreg_t(new_v, PLANE_KEYVAL(v_pipe, plane));
				keymax = vgpu_vreg_t(new_v, PLANE_KEYMAX(v_pipe, plane));
				keymsk = vgpu_vreg_t(new_v, PLANE_KEYMSK(v_pipe, plane));
				offset = vgpu_vreg_t(new_v, PLANE_OFFSET(v_pipe, plane));
				stride = vgpu_vreg_t(new_v, PLANE_STRIDE(v_pipe, plane));
				pos = vgpu_vreg_t(new_v, PLANE_POS(v_pipe, plane));
				size = vgpu_vreg_t(new_v, PLANE_SIZE(v_pipe, plane));
				aux_dist = vgpu_vreg_t(new_v, PLANE_AUX_DIST(v_pipe, plane));
				aux_offset = vgpu_vreg_t(new_v, PLANE_AUX_OFFSET(v_pipe, plane));
				ctl = vgpu_vreg_t(new_v, PLANE_CTL(v_pipe, plane));
				hma = vgpu_vreg_t(new_v, PLANE_SURF(v_pipe, plane));
			} else {
				keyval = d0_plane_regs->plane_keyval;
				keymax = d0_plane_regs->plane_keymax;
				keymsk = d0_plane_regs->plane_keymsk;
				offset = d0_plane_regs->plane_offset;
				stride = d0_plane_regs->plane_stride;
				pos = d0_plane_regs->plane_pos;
				size = d0_plane_regs->plane_size;
				aux_dist = d0_plane_regs->plane_aux_dist;
				aux_offset = d0_plane_regs->plane_aux_offset;
				ctl = d0_plane_regs->plane_ctl;
				hma = d0_plane_regs->plane_surf;
			}

			if (!new_v || plane ==  PLANE_PRIMARY) {
				for (level = 0; level <= max_level; level++) {
					I915_WRITE_FW(PLANE_WM(pipe, plane, level), wm_level[level]);
				}
				I915_WRITE_FW(PLANE_WM_TRANS(pipe, plane), wm_trans);
				I915_WRITE_FW(PLANE_KEYVAL(pipe, plane), keyval);
				I915_WRITE_FW(PLANE_KEYMAX(pipe, plane), keymax);
				I915_WRITE_FW(PLANE_KEYMSK(pipe, plane), keymsk);
				I915_WRITE_FW(PLANE_OFFSET(pipe, plane), offset);
				I915_WRITE_FW(PLANE_STRIDE(pipe, plane), stride);
				I915_WRITE_FW(PLANE_POS(pipe, plane), pos);
				I915_WRITE_FW(PLANE_SIZE(pipe, plane), size);
				I915_WRITE_FW(PLANE_AUX_DIST(pipe, plane), aux_dist);
				I915_WRITE_FW(PLANE_AUX_OFFSET(pipe, plane), aux_offset);
				I915_WRITE_FW(PLANE_CTL(pipe, plane), ctl);
				I915_WRITE_FW(PLANE_SURF(pipe, plane), hma);
				intel_uncore_posting_read_fw(&dev_priv->uncore, PLANE_SURF(pipe, plane));
			}

			if (new_v) {
				if (ctl & PLANE_CTL_ASYNC_FLIP)
					intel_vgpu_trigger_virtual_event(new_v, SKL_FLIP_EVENT(v_pipe, plane));
				else
					set_bit(SKL_FLIP_EVENT(v_pipe, plane), new_v->irq.flip_done_event[v_pipe]);
			}
		}
	}
	spin_unlock_irqrestore(&dev_priv->uncore.lock, irqflags);

	mmio_hw_access_post(dev_priv);

	if (old_v) {
		disp_path_old = NULL;
		list_for_each_entry_safe(disp_path_old, n, &old_v->disp_cfg.path_list, list) {
			if (disp_path_old->p_pipe == pipe)
				disp_path_old->foreground_state = false;
		}
	}

	if (new_v) {
		disp_path->foreground_state = true;
		mutex_unlock(&new_v->disp_cfg.sw_lock);
	}

	drm_modeset_unlock(&crtc->base.mutex);
}

void intel_gvt_flush_pipe_color(struct intel_gvt *gvt, enum pipe pipe,
				struct intel_vgpu *vgpu)
{
	struct drm_i915_private *dev_priv = gvt->dev_priv;
	struct intel_dom0_pipe_regs *pipe_regs =
		&gvt->pipe_info[pipe].dom0_pipe_regs;
	struct prec_pal_data *pal_data = NULL;
	enum pipe v_pipe = INVALID_PIPE;
	u32 gamma_mode = 0;

	if (vgpu) {
		struct intel_vgpu_display *disp_cfg = &vgpu->disp_cfg;
		struct intel_vgpu_display_path *disp_path = NULL, *n;

		list_for_each_entry_safe(disp_path, n, &disp_cfg->path_list, list) {
			if (disp_path->p_pipe == pipe) {
				v_pipe = disp_path->pipe;
				break;
			}
		}

		if (v_pipe == INVALID_PIPE)
			return;

		gamma_mode = vgpu_vreg_t(vgpu, GAMMA_MODE(v_pipe));
		if ((gamma_mode & GAMMA_MODE_MODE_MASK) == GAMMA_MODE_MODE_SPLIT)
			pal_data = disp_path->prec_palette_split;
		else if ((gamma_mode & GAMMA_MODE_MODE_MASK) != GAMMA_MODE_MODE_8BIT)
			pal_data = disp_path->prec_palette_nonsplit;
	} else {
		gamma_mode = pipe_regs->gamma_mode;
		if ((gamma_mode & GAMMA_MODE_MODE_MASK) == GAMMA_MODE_MODE_SPLIT)
			pal_data = pipe_regs->prec_palette_split;
		else if ((gamma_mode & GAMMA_MODE_MODE_MASK) != GAMMA_MODE_MODE_8BIT)
			pal_data = pipe_regs->prec_palette_nonsplit;
	}

	I915_WRITE_FW(SKL_BOTTOM_COLOR(pipe), vgpu ?
		      vgpu_vreg_t(vgpu, SKL_BOTTOM_COLOR(v_pipe)) :
		      pipe_regs->bottom_color);
	I915_WRITE_FW(GAMMA_MODE(pipe), vgpu ?
		      vgpu_vreg_t(vgpu, GAMMA_MODE(v_pipe)) :
		      pipe_regs->gamma_mode);
	I915_WRITE_FW(PIPE_CSC_MODE(pipe), vgpu ?
		      vgpu_vreg_t(vgpu, PIPE_CSC_MODE(v_pipe)) :
		      pipe_regs->csc_mode);
	I915_WRITE_FW(PIPE_CSC_PREOFF_HI(pipe), vgpu ?
		      vgpu_vreg_t(vgpu, PIPE_CSC_PREOFF_HI(v_pipe)) :
		      pipe_regs->csc_preoff_hi);
	I915_WRITE_FW(PIPE_CSC_PREOFF_ME(pipe), vgpu ?
		      vgpu_vreg_t(vgpu, PIPE_CSC_PREOFF_ME(v_pipe)) :
		      pipe_regs->csc_preoff_me);
	I915_WRITE_FW(PIPE_CSC_PREOFF_LO(pipe), vgpu ?
		      vgpu_vreg_t(vgpu, PIPE_CSC_PREOFF_LO(v_pipe)) :
		      pipe_regs->csc_preoff_lo);
	I915_WRITE_FW(PIPE_CSC_COEFF_RY_GY(pipe), vgpu ?
		      vgpu_vreg_t(vgpu, PIPE_CSC_COEFF_RY_GY(v_pipe)) :
		      pipe_regs->csc_coeff_rygy);
	I915_WRITE_FW(PIPE_CSC_COEFF_BY(pipe), vgpu ?
		      vgpu_vreg_t(vgpu, PIPE_CSC_COEFF_BY(v_pipe)) :
		      pipe_regs->csc_coeff_by);
	I915_WRITE_FW(PIPE_CSC_COEFF_RU_GU(pipe), vgpu ?
		      vgpu_vreg_t(vgpu, PIPE_CSC_COEFF_RU_GU(v_pipe)) :
		      pipe_regs->csc_coeff_rugu);
	I915_WRITE_FW(PIPE_CSC_COEFF_BU(pipe), vgpu ?
		      vgpu_vreg_t(vgpu, PIPE_CSC_COEFF_BU(v_pipe)) :
		      pipe_regs->csc_coeff_bu);
	I915_WRITE_FW(PIPE_CSC_COEFF_RV_GV(pipe), vgpu ?
		      vgpu_vreg_t(vgpu, PIPE_CSC_COEFF_RV_GV(v_pipe)) :
		      pipe_regs->csc_coeff_rvgv);
	I915_WRITE_FW(PIPE_CSC_COEFF_BV(pipe), vgpu ?
		      vgpu_vreg_t(vgpu, PIPE_CSC_COEFF_BV(v_pipe)) :
		      pipe_regs->csc_coeff_bv);
	I915_WRITE_FW(PIPE_CSC_POSTOFF_HI(pipe), vgpu ?
		      vgpu_vreg_t(vgpu, PIPE_CSC_POSTOFF_HI(v_pipe)) :
		      pipe_regs->csc_postoff_hi);
	I915_WRITE_FW(PIPE_CSC_POSTOFF_ME(pipe), vgpu ?
		      vgpu_vreg_t(vgpu, PIPE_CSC_POSTOFF_ME(v_pipe)) :
		      pipe_regs->csc_postoff_me);
	I915_WRITE_FW(PIPE_CSC_POSTOFF_LO(pipe), vgpu ?
		      vgpu_vreg_t(vgpu, PIPE_CSC_POSTOFF_LO(v_pipe)) :
		      pipe_regs->csc_postoff_lo);

	// Set correct palette based on gamma mode
	if ((gamma_mode & GAMMA_MODE_MODE_MASK) == GAMMA_MODE_MODE_8BIT) {
		int i;

		for (i = 0; i < 256; i++) {
			I915_WRITE_FW(LGC_PALETTE(pipe, i), vgpu ?
				      vgpu_vreg_t(vgpu, LGC_PALETTE(v_pipe, i)) :
				      pipe_regs->lgc_palette[i]);
		}
	} else {
		int i;
		u32 split = 0;

		if ((gamma_mode & GAMMA_MODE_MODE_MASK) == GAMMA_MODE_MODE_SPLIT)
			split |= PAL_PREC_SPLIT_MODE;

		if (pal_data == NULL)
			return;

		for (i = 0; i < PAL_PREC_INDEX_VALUE_MASK + 1; i++) {
			if (pal_data[i].dirty) {
				I915_WRITE_FW(PREC_PAL_INDEX(pipe), i | split);
				I915_WRITE_FW(PREC_PAL_DATA(pipe),
					      pal_data[i].val);
			}
		}
		/*
		 * Reset PREC_PAL_INDEX, otherwise it prevents the legacy
		 * palette to be written properly.
		 */
		I915_WRITE_FW(PREC_PAL_INDEX(pipe), 0);

	}
}

static void intel_gvt_switch_display_work(struct work_struct *w)
{
	struct intel_gvt *gvt = container_of(w,
		struct intel_gvt, switch_display_work);
	struct intel_vgpu *vgpu, *old_v, *new_v;
	struct intel_vgpu_display_path *disp_path = NULL, *n;
	int id, old, new;
	u32 new_owner = 0;
	enum pipe pipe;
	enum port port;
	bool do_switch = false;

	mutex_lock(&gvt->lock);
	mutex_lock(&gvt->sw_in_progress);

	/* Switch base on port */
	for_each_pipe(gvt->dev_priv, pipe) {
		port = intel_gvt_port_from_pipe(gvt->dev_priv, pipe);

		if (port == PORT_NONE)
			continue;

		gvt_dbg_dpy("Check display switch on PIPE_%c, PORT_%c\n",
			    pipe_name(pipe), port_name(port));

		old = gvt->pipe_info[pipe].owner;
		new = (gvt->disp_owner >> port * 4) & 0xF;

		old_v = NULL;
		new_v = NULL;
		do_switch = false;
		// Get old and new owners, either host or vGPU-x
		idr_for_each_entry((&(gvt)->vgpu_idr), vgpu, id) {
			if (old && id == old)
				old_v = vgpu;
			if (new && id == new)
				new_v = vgpu;
		}

		if (new != old) {
			if (new_v) {
				disp_path = NULL;
				list_for_each_entry_safe(disp_path, n, &new_v->disp_cfg.path_list, list) {
					if (disp_path->p_pipe != INVALID_PIPE &&
					    disp_path->p_port == port)
						break;
				}
				if (atomic_read(&new_v->active) && disp_path &&
				    disp_path->foreground &&
				    !disp_path->foreground_state) {
					do_switch = true;
					gvt_dbg_dpy("Switch display to vGPU-%d, path-%d, PIPE_%c, PORT_%c\n",
						    new_v->id, disp_path->id,
						    pipe_name(disp_path->p_pipe),
						    port_name(disp_path->p_port));
				} else {
					// If can't switch to new vGPU,
					// maintain old owner.
					new_owner = gvt->disp_owner;
					new_owner &= ~(0xF << port * 4);
					new_owner |= (old << port * 4);
					gvt->disp_owner = new_owner;
					do_switch = false;
					gvt_dbg_dpy("Can't switch display to vGPU-%d active-%d, path-%d, PIPE_%c, PORT_%c, foreground-%d, foreground_state-%d\n",
						    new_v->id, atomic_read(&new_v->active),
						    disp_path->id,
						    pipe_name(disp_path->p_pipe),
						    port_name(disp_path->p_port),
						    disp_path->foreground,
						    disp_path->foreground_state);
				}
			} else {
				// Switch to host is always expected possible
				do_switch = true;
				gvt_dbg_dpy("Switch display to host\n");
			}
		} else {
			do_switch = false;
			gvt_dbg_dpy("Skip switch display due to unchanged owner\n");
		}

		if (do_switch && !prepare_for_switch_display(gvt, pipe)) {
			intel_gvt_switch_display_pipe(gvt, pipe, old_v, new_v);
			if (old_v && new_v)
				gvt_dbg_dpy("Change PIPE_%c PORT_%c owner from "
					    "vGPU-%d to vGPU-%d\n",
					    pipe_name(pipe), port_name(port),
					    old_v->id, new_v->id);
			else if (old_v && !new_v)
				gvt_dbg_dpy("Change PIPE_%c PORT_%c owner from "
					    "vGPU-%d to host\n",
					    pipe_name(pipe), port_name(port),
					    old_v->id);
			else if (!old_v && new_v)
				gvt_dbg_dpy("Change PIPE_%c PORT_%c owner from "
					    "host to vGPU-%d\n",
					    pipe_name(pipe), port_name(port),
					    new_v->id);
			else
				gvt_dbg_dpy("Unexpected change PIPE_%c PORT_%c "
					    "owner from host to host\n",
					    pipe_name(pipe), port_name(port));
		}
	}

	mutex_unlock(&gvt->sw_in_progress);
	mutex_unlock(&gvt->lock);
}

void intel_gvt_store_vgpu_display_owner(struct drm_i915_private *dev_priv,
					u32 disp_owner)
{
	struct intel_gvt *gvt = dev_priv->gvt;
	struct intel_vgpu *vgpu = NULL;
	int id;
	enum pipe pipe = INVALID_PIPE;
	enum port port = PORT_NONE;
	u8 owner;
	u32 owner_mask = 0;
	bool valid_owner = true, vgpu_found;

	mutex_lock(&gvt->lock);
	mutex_lock(&gvt->sw_in_progress);

	if (disp_owner == gvt->disp_owner) {
		mutex_unlock(&gvt->sw_in_progress);
		mutex_unlock(&gvt->lock);
		return;
	}

	// Strip out unavailable ports
	for_each_pipe(dev_priv, pipe) {
		port = intel_gvt_port_from_pipe(dev_priv, pipe);
		if (port == PORT_NONE)
			continue;

		owner_mask |= 0xF << port * 4;
	}
	disp_owner &= owner_mask;

	// Verify that vGPU id is valid and the select port
	// is already assigned to the new owner.
	for_each_pipe(dev_priv, pipe) {
		port = intel_gvt_port_from_pipe(dev_priv, pipe);
		if (port == PORT_NONE)
			continue;

		owner = (disp_owner >> port * 4) & 0xF;

		vgpu_found = false;
		for_each_active_vgpu(gvt, vgpu, id) {
			if (id == owner) {
				vgpu_found = true;
				break;
			}
		}

		if (owner != 0) {
			if (vgpu_found) {
				// port_assign = intel_gvt_external_disp_id_from_port(port);
				if (!(((gvt->sel_disp_port_mask >> (owner - 1) * 8) & 0xFF) & (1 << port))) {
					gvt_err("PORT_%c(%d) isn't assigned to vGPU-%d\n",
						port_name(port),
						intel_gvt_external_disp_id_from_port(port),
						owner);
					valid_owner = false;
					break;
				}
			} else {
				gvt_err("Selected owner vGPU-%d doesn't exist\n",
					owner);
				valid_owner = false;
				break;
			}
		}
	}

	if (valid_owner) {
		for_each_pipe(dev_priv, pipe) {
			port = intel_gvt_port_from_pipe(dev_priv, pipe);
			if (port == PORT_NONE)
				continue;

			owner = (disp_owner >> port * 4) & 0xF;
			gvt->disp_owner &= ~(0xF << port * 4);
			gvt->disp_owner |= (owner << port * 4);
			if (owner)
				gvt_dbg_dpy("PORT_%c owner changed to vGPU-%d\n",
					    port_name(port), owner);
			else
				gvt_dbg_dpy("PORT_%c owner changed to host\n",
					    port_name(port));
		}
		queue_work(system_unbound_wq, &gvt->switch_display_work);
	}

	mutex_unlock(&gvt->sw_in_progress);
	mutex_unlock(&gvt->lock);
}

void intel_gvt_store_vgpu_display_mask(struct drm_i915_private *dev_priv,
				       u64 mask)
{
	struct intel_gvt *gvt = dev_priv->gvt;
	struct intel_vgpu *vgpu;
	int id;
	int num_vgpu = 0;
	u8 port_sel, mask_vgpu;
	enum port port = PORT_NONE;
	bool valid_mask = true;

	mutex_lock(&gvt->lock);

	if (mask == gvt->sel_disp_port_mask) {
		mutex_unlock(&gvt->lock);
		return;
	}

	idr_for_each_entry((&(gvt)->vgpu_idr), vgpu, id)
		num_vgpu++;

	if (num_vgpu == 0) {
		// Check whether the assigned port available or not
		for (id = 0; valid_mask && id < GVT_MAX_VGPU; id++) {
			mask_vgpu = (mask >> id * 8) & 0xFF;
			if (mask_vgpu == 0)
				continue;
			for (port = PORT_A; port < I915_MAX_PORTS; port++) {
				if (mask_vgpu & (1 << port)) {
					port_sel = intel_gvt_external_disp_id_from_port(port);
					if (port_sel == 0 ||
					    !(gvt->avail_disp_port_mask & intel_gvt_port_to_mask_bit(port_sel, port))) {
						gvt_err("Selected PORT_%c for vGPU-%d isn't available\n",
							port_name(port), id + 1);
						valid_mask = false;
						break;
					}
				}
			}
		}

		if (valid_mask) {
			gvt->sel_disp_port_mask = mask;
			gvt_dbg_dpy("Display port mask changed to 0x%016llx\n",
				    mask);
		}
	} else {
		gvt_err("Can't modify display port mask after vGPU created\n");
	}

	mutex_unlock(&gvt->lock);
}

void intel_gvt_store_vgpu_display_switch(struct drm_i915_private *dev_priv,
					 bool auto_switch)
{
	struct intel_gvt *gvt = dev_priv->gvt;
	struct intel_vgpu *vgpu;
	int id;
	int num_vgpu = 0;

	mutex_lock(&gvt->lock);

	if (auto_switch == READ_ONCE(gvt->disp_auto_switch)) {
		mutex_unlock(&gvt->lock);
		return;
	}

	// Disable auto switch if any port is already shared by several vGPUs.
	idr_for_each_entry((&(gvt)->vgpu_idr), vgpu, id)
		num_vgpu++;

	// Disable auto switch if any port assigned to several vGPUs.
	if (num_vgpu == 0) {
		WRITE_ONCE(gvt->disp_auto_switch, auto_switch);
		gvt_dbg_dpy("Display auto switch set to %s\n",
			    auto_switch ? "Y" : "N");
	} else {
		gvt_err("Can't modify display auto switch after vGPU created\n");
	}

	mutex_unlock(&gvt->lock);
}

/**
 * intel_vgpu_display_find_owner - find auto switch owner
 * @vgpu: vGPU which receives the event
 * @reset: to reset owner of the input vGPU
 * @next: allow search for next vGPU
 * Auto switch only operates in unit of all ports of the same vGPU.
 * 1) reset == 0 && next == 0:
 *    Do not take back ownership of assigned ports from current vGPU.
 *    Return current vGPU as the new owner if the assigned ports are available.
 *    Do not search for next available vGPU.
 *    The common usage is on receiving DISPLAY_READY.
 * 2) reset == 0 && next == 1:
 *    Do not take back ownership of assigned ports from current vGPU.
 *    Return current vGPU as the new owner if the assigned ports are available.
 *    Can search for next available vGPU.
 *    The common usage is to switch to next vGPU display.
 * 3) reset == 1 && next == 0:
 *    Take back ownership of assigned ports from current vGPU.
 *    The common usage is to switch display from current vGPU to host.
 * 4) reset == 1 && next == 1:
 *    Take back ownership of assigned ports from current vGPU.
 *    Search for next vGPU with same or less ports assigned.
 *    The common usage is to switch display to next vGPU on receiving deactivate
 *    or DMLR.
 */
u32 intel_vgpu_display_find_owner(struct intel_vgpu *vgpu, bool reset, bool next)
{
	struct intel_gvt *gvt = vgpu->gvt;
	struct intel_vgpu *other_v;
	struct intel_vgpu_display *disp_cfg;
	struct intel_vgpu_display_path *disp_path = NULL, *n;
	enum pipe pipe = INVALID_PIPE;
	enum port port = PORT_NONE;
	u32 id, owner_id, available, new, candidate;
	bool found = false;

	new = gvt->disp_owner;

	candidate = 0;
	disp_cfg = &vgpu->disp_cfg;
	list_for_each_entry_safe(disp_path, n, &disp_cfg->path_list, list) {
		if (disp_path->p_pipe != INVALID_PIPE) {
			if (reset) {
				/* Take back port ownership from current vGPU */
				if (((new >> disp_path->p_port * 4) & 0xF) == vgpu->id)
					new &= ~(0xF << disp_path->p_port * 4);
			} else {
				/* Get assignment from current vGPU */
				candidate |= 1 << disp_path->p_port;
			}
		}
	}

	/* Mark unused or already owned host ports as available */
	available = 0;
	for_each_pipe(gvt->dev_priv, pipe) {
		port = intel_gvt_port_from_pipe(gvt->dev_priv, pipe);
		if (port == PORT_NONE)
			continue;

		owner_id = ((new >> port * 4) & 0xF);
		if (owner_id == 0 || owner_id == vgpu->id)
			available |= 1 << port;
	}

	/* Check if non-reset candidate can be applied to available */
	if (candidate && (~available & candidate) == 0) {
		list_for_each_entry_safe(disp_path, n, &disp_cfg->path_list, list) {
			if (disp_path->p_pipe != INVALID_PIPE) {
				new |= (vgpu->id << disp_path->p_port * 4);
				found = true;
			}
		}
	}

	/* Return if not allow search for next or already found current */
	if (!next || found)
		return new;

	/* Check all other vGPU, find one with same or less port assigned */
	for_each_active_vgpu(gvt, other_v, id) {
		if (other_v->id != vgpu->id) {
			/* Update available according */
			available = 0;
			for_each_pipe(gvt->dev_priv, pipe) {
				port = intel_gvt_port_from_pipe(gvt->dev_priv, pipe);
				if (port == PORT_NONE)
					continue;

				owner_id = ((new >> port * 4) & 0xF);
				if (owner_id == 0 || owner_id == other_v->id)
					available |= 1 << port;
			}

			/* Pick a candidate vGPU and mark assigned port */
			candidate = 0;
			disp_cfg = &other_v->disp_cfg;
			list_for_each_entry_safe(disp_path, n, &disp_cfg->path_list, list)
				if (disp_path->p_pipe != INVALID_PIPE)
					candidate |= 1 << disp_path->p_port;

			/* The candidate vGPU can only use available port */
			if (candidate && (~available & candidate) == 0) {
				list_for_each_entry_safe(disp_path, n, &disp_cfg->path_list, list)
					if (disp_path->p_pipe != INVALID_PIPE)
						new |= (other_v->id << disp_path->p_port * 4);
				break;
			}
		}
	}

	return new;
}

void intel_vgpu_display_set_foreground(struct intel_vgpu *vgpu, bool set)
{
	struct intel_vgpu_display *disp_cfg;
	struct intel_vgpu_display_path *disp_path = NULL, *n;

	disp_cfg = &vgpu->disp_cfg;
	list_for_each_entry_safe(disp_path, n, &disp_cfg->path_list, list) {
		disp_path->foreground = set;
	}
}

void intel_gvt_init_display(struct intel_gvt *gvt)
{
	mutex_init(&gvt->sw_in_progress);
	INIT_WORK(&gvt->connector_change_work, intel_gvt_connector_change_work);
	INIT_WORK(&gvt->switch_display_work, intel_gvt_switch_display_work);
	intel_gvt_init_pipe_info(gvt);
	intel_gvt_init_ddb(gvt);
}

#if IS_ENABLED(CONFIG_DRM_I915_GVT_ACRN_GVT)
#define GOP_FB_SIZE		0x800000
#define GOP_DISPLAY_WIDTH	1920u
#define GOP_DISPLAY_HEIGHT	1080u

/*
 * check_gop_mode to query current mode and pass it to GOP
 *
 * 1. Get current mode from ctrc)
 * 2. use crtc mode as GOP mode if mode <=1920x1080
 * 3. use 1920x1080 as GOP mode if mode > 1080p
 * 4.   enable panel scale (ToDO)
 * 5. pass GOP mode to OVMF
 *
 */
static int check_gop_mode(struct intel_vgpu *vgpu)
{
	unsigned int pipe;
	struct intel_crtc *crtc;
	struct intel_crtc_state *crtc_state;
	struct drm_display_mode mode;
	enum port port = PORT_NONE;
	u64 *port_mask = &vgpu->gvt->sel_disp_port_mask;
	bool found = false;

	for (port = PORT_A; port < I915_MAX_PORTS; port++) {
		if ((*port_mask >> (vgpu->id - 1) * 8) & 0xFF & (1 << port)) {
			found = true;
			break;
		}
	}

	if (found == false) {
		gvt_dbg_dpy("Can't find port %d for vgpu-%d\n", port, vgpu->id);
		return -ENODEV;
	}

	pipe = intel_gvt_pipe_from_port(vgpu->gvt->dev_priv, port);
	if (pipe == INVALID_PIPE) {
		gvt_dbg_dpy("Bad mapping {vgpu-%d, port:%d}\n", vgpu->id, port);
		return -ENODEV;
	}

	crtc = intel_get_crtc_for_pipe(vgpu->gvt->dev_priv, pipe);
	crtc_state = to_intel_crtc_state(crtc->base.state);
	intel_mode_from_pipe_config(&mode, crtc_state);

	drm_mode_debug_printmodeline(&mode);

	if (mode.vdisplay <= 0 || mode.hdisplay <= 0)
		return -EINVAL;

	vgpu->gm.gop.width = mode.hdisplay;
	vgpu->gm.gop.height = mode.vdisplay;
	vgpu->gm.gop.pitch = mode.hdisplay;
	vgpu->gm.gop.Bpp = 4;

	/* populate mode for OVMF GOP driver */
	if (mode.hdisplay * mode.vdisplay * 4 > GOP_FB_SIZE) {
		vgpu_vreg_t(vgpu, vgtif_reg(gop.width)) =
			min(vgpu->gm.gop.width, GOP_DISPLAY_WIDTH);
		vgpu_vreg_t(vgpu, vgtif_reg(gop.height)) =
			min(vgpu->gm.gop.height, GOP_DISPLAY_HEIGHT);
		vgpu_vreg_t(vgpu, vgtif_reg(gop.pitch)) =
			min(vgpu->gm.gop.pitch, GOP_DISPLAY_WIDTH);
	} else {
		vgpu_vreg_t(vgpu, vgtif_reg(gop.width)) = vgpu->gm.gop.width;
		vgpu_vreg_t(vgpu, vgtif_reg(gop.height)) = vgpu->gm.gop.height;
		vgpu_vreg_t(vgpu, vgtif_reg(gop.pitch)) = vgpu->gm.gop.pitch;
	}

	vgpu->gm.gop.size = 4 * vgpu_vreg_t(vgpu, vgtif_reg(gop.width)) *
				vgpu_vreg_t(vgpu, vgtif_reg(gop.height));
	vgpu_vreg_t(vgpu, vgtif_reg(gop.Bpp)) = 4;
	vgpu_vreg_t(vgpu, vgtif_reg(gop.size)) = vgpu->gm.gop.size;

	DRM_INFO("prepare GOP fb: %dKB for %dX%d@%d\n",
			vgpu_vreg_t(vgpu, vgtif_reg(gop.size))>>10,
			vgpu_vreg_t(vgpu, vgtif_reg(gop.width)),
			vgpu_vreg_t(vgpu, vgtif_reg(gop.height)),
			vgpu_vreg_t(vgpu, vgtif_reg(gop.Bpp))*8);
	return 0;
}

/*
 * prepare_gop_fb will allocate a arrange of memory, and then map them
 * into the ggtt table of the guest partition in the aperture.
 */
static int prepare_gop_fb(struct intel_vgpu *vgpu, u32 size)
{
	struct drm_i915_private *dev_priv = vgpu->gvt->dev_priv;
	struct page **pages = NULL;
	u32 count, npages = size >> PAGE_SHIFT;
	struct i915_ggtt *ggtt = &dev_priv->ggtt;
	struct i915_vma vma;
	struct drm_mm_node *node = &vgpu->gm.high_gm_node;
	struct sg_table st;
	unsigned int cache_level = HAS_LLC(dev_priv) ?
				I915_CACHE_LLC : I915_CACHE_NONE;
	int ret = 0;

	pages = kmalloc_array(npages, sizeof(struct page *), GFP_KERNEL);
	if (!pages)
		return -ENOMEM;

	for (count = 0; count < npages; count++) {
		struct page *page = alloc_page(GFP_KERNEL);
		if (!page) {
			ret = -ENOMEM;
			goto free_pgs;
		}
		pages[count] = page;

		intel_gvt_hypervisor_map_gfn_to_mfn (vgpu,
				(GOP_FB_BASE >> PAGE_SHIFT) + count,
				page_to_pfn(page), 1, true);
	}

	ret = sg_alloc_table_from_pages(&st, pages, npages,
			0, npages << PAGE_SHIFT, GFP_KERNEL);
	if (ret)
		goto free_pgs;

	if (!dma_map_sg(&dev_priv->drm.pdev->dev, st.sgl, st.nents,
				PCI_DMA_BIDIRECTIONAL)) {
		ret = -ENOMEM;
		goto free_sg;
	}

	memset(&vma, 0, sizeof(vma));
	vma.node.start = node->start;
	vma.node.size = size;
	vma.pages = &st;
	ggtt->vm.insert_entries(&ggtt->vm, &vma, cache_level, 0);
	sg_free_table(&st);

	vgpu->gm.gop_fb_pages = pages;
	vgpu->gm.gop_fb_size = count;
	return 0;

free_sg:
	sg_free_table(&st);

free_pgs:
	release_pages(pages, count);
	kfree(pages);
	return ret;
}

static int setup_gop_display(struct intel_vgpu *vgpu)
{
	int ret = 0;
	unsigned int pipe;
	struct drm_i915_private *dev_priv = vgpu->gvt->dev_priv;
	bool found = false;
	u32 width, height, Bpp;
	u32 stride, ctl, surf;
	unsigned long irqflags;
	u64 *port_mask = &vgpu->gvt->sel_disp_port_mask;
	enum port port = PORT_NONE;

	width = vgpu_vreg_t(vgpu, vgtif_reg(gop.width));
	height = vgpu_vreg_t(vgpu, vgtif_reg(gop.height));
	Bpp = vgpu_vreg_t(vgpu, vgtif_reg(gop.Bpp));

	DRM_INFO("Set up display w:%u h:%u for GOP \n", width, height);
	for (port = PORT_A; port < I915_MAX_PORTS; port++) {
		if ((*port_mask >> (vgpu->id - 1) * 8) & 0xFF & (1 << port)) {
			found = true;
			break;
		}
	}

	if (found == false) {
		gvt_dbg_dpy("Can't find port %d for vgpu-%d\n", port, vgpu->id);
		return -ENODEV;
	}

	pipe = intel_gvt_pipe_from_port(vgpu->gvt->dev_priv, port);
	if (pipe == INVALID_PIPE) {
		gvt_dbg_dpy("Bad mapping {vgpu-%d ,port %d}\n", vgpu->id, port);
		return -ENODEV;
	}

	/* Sizes are 0 based */
	stride = width * Bpp / 64; /* 32bit per pixel */
	width--;
	height--;
	surf = vgpu->gm.high_gm_node.start;
	ctl = PLANE_CTL_ENABLE | PLANE_CTL_FORMAT_XRGB_8888;
	ctl |= PLANE_CTL_PIPE_GAMMA_ENABLE |
		PLANE_CTL_PIPE_CSC_ENABLE |
		PLANE_CTL_PLANE_GAMMA_DISABLE;
	spin_lock_irqsave(&dev_priv->uncore.lock, irqflags);
	I915_WRITE_FW(PLANE_OFFSET(pipe, 0), 0);
	I915_WRITE_FW(PLANE_STRIDE(pipe, 0), stride);
	I915_WRITE_FW(PLANE_SIZE(pipe, 0), (height << 16) | width);
	I915_WRITE_FW(PLANE_AUX_DIST(pipe, 0), 0xFFFFF000);
	I915_WRITE_FW(PLANE_AUX_OFFSET(pipe, 0), 0);
	I915_WRITE_FW(PLANE_POS(pipe, 0), 0);
	I915_WRITE_FW(PLANE_CTL(pipe, 0), ctl);
	I915_WRITE_FW(PLANE_SURF(pipe, 0), surf);
	I915_READ_FW(PLANE_SURF(pipe, 0));
	spin_unlock_irqrestore(&dev_priv->uncore.lock, irqflags);
	return ret;
}

int intel_vgpu_g2v_setup_gop(struct intel_vgpu *vgpu)
{
	int ret = 0;

	gvt_dbg_dpy("intel_vgpu_g2v_setup_gop\n");

	if (vgpu->gm.gop_fb_pages)
		goto Done;

	ret = check_gop_mode(vgpu);
	if (ret) {
		gvt_vgpu_err("gop check pipe faile %d\n", ret);
		goto Done;
	}

	ret = prepare_gop_fb(vgpu, vgpu->gm.gop.size);
	if (ret) {
		gvt_vgpu_err("gop prepared failed %d\n", ret);
		goto Done;
	}

	ret = setup_gop_display(vgpu);
	if (ret) {
		gvt_vgpu_err("gop display setup failed %d\n", ret);
		goto Done;
	}

	vgpu->gm.gop.fb_base = GOP_FB_BASE;

	vgpu_vreg(vgpu, _vgtif_reg(gop.fb_base)) = vgpu->gm.gop.fb_base;

	gvt_dbg_dpy("set up gop FbBase: %x\n",
			vgpu_vreg(vgpu, _vgtif_reg(gop.fb_base)));

Done:
	return 0;
}
#endif
