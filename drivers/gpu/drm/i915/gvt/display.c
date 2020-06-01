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

static int get_edp_pipe(struct intel_vgpu *vgpu)
{
	u32 data = vgpu_vreg(vgpu, _TRANS_DDI_FUNC_CTL_EDP);
	int pipe = -1;

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

int pipe_is_enabled(struct intel_vgpu *vgpu, int pipe)
{
	struct drm_i915_private *dev_priv = vgpu->gvt->dev_priv;

	if (WARN_ON(pipe < PIPE_A || pipe >= I915_MAX_PIPES))
		return -EINVAL;

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
	int pipe;

	if (IS_BROXTON(dev_priv)) {
		vgpu_vreg_t(vgpu, GEN8_DE_PORT_ISR) &= ~(BXT_DE_PORT_HP_DDIA |
			BXT_DE_PORT_HP_DDIB |
			BXT_DE_PORT_HP_DDIC);

		if (intel_vgpu_has_monitor_on_port(vgpu, PORT_A)) {
			vgpu_vreg_t(vgpu, GEN8_DE_PORT_ISR) |=
				BXT_DE_PORT_HP_DDIA;
		}

		if (intel_vgpu_has_monitor_on_port(vgpu, PORT_B)) {
			vgpu_vreg_t(vgpu, GEN8_DE_PORT_ISR) |=
				BXT_DE_PORT_HP_DDIB;
		}

		if (intel_vgpu_has_monitor_on_port(vgpu, PORT_C)) {
			vgpu_vreg_t(vgpu, GEN8_DE_PORT_ISR) |=
				BXT_DE_PORT_HP_DDIC;
		}

		vgpu_vreg_t(vgpu, SKL_FUSE_STATUS) |=
				SKL_FUSE_DOWNLOAD_STATUS |
				SKL_FUSE_PG_DIST_STATUS(SKL_PG0) |
				SKL_FUSE_PG_DIST_STATUS(SKL_PG1) |
				SKL_FUSE_PG_DIST_STATUS(SKL_PG2);

		return;
	}

	vgpu_vreg_t(vgpu, SDEISR) &= ~(SDE_PORTB_HOTPLUG_CPT |
			SDE_PORTC_HOTPLUG_CPT |
			SDE_PORTD_HOTPLUG_CPT);

	if (IS_SKYLAKE(dev_priv) || IS_KABYLAKE(dev_priv)) {
		vgpu_vreg_t(vgpu, SDEISR) &= ~(SDE_PORTA_HOTPLUG_SPT |
				SDE_PORTE_HOTPLUG_SPT);
		vgpu_vreg_t(vgpu, SKL_FUSE_STATUS) |=
				SKL_FUSE_DOWNLOAD_STATUS |
				SKL_FUSE_PG_DIST_STATUS(SKL_PG0) |
				SKL_FUSE_PG_DIST_STATUS(SKL_PG1) |
				SKL_FUSE_PG_DIST_STATUS(SKL_PG2);
		/*
		 * Only 1 PIPE enabled in current vGPU display and PIPE_A is
		 *  tied to TRANSCODER_A in HW, so it's safe to assume PIPE_A,
		 *   TRANSCODER_A can be enabled. PORT_x depends on the input of
		 *   setup_virtual_dp_monitor, we can bind DPLL0 to any PORT_x
		 *   so we fixed to DPLL0 here.
		 * Setup DPLL0: DP link clk 1620 MHz, non SSC, DP Mode
		 */
		vgpu_vreg_t(vgpu, DPLL_CTRL1) =
			DPLL_CTRL1_OVERRIDE(DPLL_ID_SKL_DPLL0);
		vgpu_vreg_t(vgpu, DPLL_CTRL1) |=
			DPLL_CTRL1_LINK_RATE(DPLL_CTRL1_LINK_RATE_1620, DPLL_ID_SKL_DPLL0);
		vgpu_vreg_t(vgpu, LCPLL1_CTL) =
			LCPLL_PLL_ENABLE | LCPLL_PLL_LOCK;
		vgpu_vreg_t(vgpu, DPLL_STATUS) = DPLL_LOCK(DPLL_ID_SKL_DPLL0);
		/*
		 * Golden M/N are calculated based on:
		 *   24 bpp, 4 lanes, 154000 pixel clk (from virtual EDID),
		 *   DP link clk 1620 MHz and non-constant_n.
		 * TODO: calculate DP link symbol clk and stream clk m/n.
		 */
		vgpu_vreg_t(vgpu, PIPE_DATA_M1(TRANSCODER_A)) = 63 << TU_SIZE_SHIFT;
		vgpu_vreg_t(vgpu, PIPE_DATA_M1(TRANSCODER_A)) |= 0x5b425e;
		vgpu_vreg_t(vgpu, PIPE_DATA_N1(TRANSCODER_A)) = 0x800000;
		vgpu_vreg_t(vgpu, PIPE_LINK_M1(TRANSCODER_A)) = 0x3cd6e;
		vgpu_vreg_t(vgpu, PIPE_LINK_N1(TRANSCODER_A)) = 0x80000;
	}

	if (intel_vgpu_has_monitor_on_port(vgpu, PORT_B)) {
		vgpu_vreg_t(vgpu, DPLL_CTRL2) &=
			~DPLL_CTRL2_DDI_CLK_OFF(PORT_B);
		vgpu_vreg_t(vgpu, DPLL_CTRL2) |=
			DPLL_CTRL2_DDI_CLK_SEL(DPLL_ID_SKL_DPLL0, PORT_B);
		vgpu_vreg_t(vgpu, DPLL_CTRL2) |=
			DPLL_CTRL2_DDI_SEL_OVERRIDE(PORT_B);
		vgpu_vreg_t(vgpu, SFUSE_STRAP) |= SFUSE_STRAP_DDIB_DETECTED;
		vgpu_vreg_t(vgpu, TRANS_DDI_FUNC_CTL(TRANSCODER_A)) &=
			~(TRANS_DDI_BPC_MASK | TRANS_DDI_MODE_SELECT_MASK |
			TRANS_DDI_PORT_MASK);
		vgpu_vreg_t(vgpu, TRANS_DDI_FUNC_CTL(TRANSCODER_A)) |=
			(TRANS_DDI_BPC_8 | TRANS_DDI_MODE_SELECT_DVI |
			(PORT_B << TRANS_DDI_PORT_SHIFT) |
			TRANS_DDI_FUNC_ENABLE);
		if (IS_BROADWELL(dev_priv)) {
			vgpu_vreg_t(vgpu, PORT_CLK_SEL(PORT_B)) &=
				~PORT_CLK_SEL_MASK;
			vgpu_vreg_t(vgpu, PORT_CLK_SEL(PORT_B)) |=
				PORT_CLK_SEL_LCPLL_810;
		}
		vgpu_vreg_t(vgpu, DDI_BUF_CTL(PORT_B)) &= ~DDI_BUF_CTL_ENABLE;
		vgpu_vreg_t(vgpu, DDI_BUF_CTL(PORT_B)) &= ~DDI_BUF_IS_IDLE;
		vgpu_vreg_t(vgpu, SDEISR) |= SDE_PORTB_HOTPLUG_CPT;
	}

	if (intel_vgpu_has_monitor_on_port(vgpu, PORT_C)) {
		vgpu_vreg_t(vgpu, DPLL_CTRL2) &=
			~DPLL_CTRL2_DDI_CLK_OFF(PORT_C);
		vgpu_vreg_t(vgpu, DPLL_CTRL2) |=
			DPLL_CTRL2_DDI_CLK_SEL(DPLL_ID_SKL_DPLL0, PORT_C);
		vgpu_vreg_t(vgpu, DPLL_CTRL2) |=
			DPLL_CTRL2_DDI_SEL_OVERRIDE(PORT_C);
		vgpu_vreg_t(vgpu, SDEISR) |= SDE_PORTC_HOTPLUG_CPT;
		vgpu_vreg_t(vgpu, TRANS_DDI_FUNC_CTL(TRANSCODER_A)) &=
			~(TRANS_DDI_BPC_MASK | TRANS_DDI_MODE_SELECT_MASK |
			TRANS_DDI_PORT_MASK);
		vgpu_vreg_t(vgpu, TRANS_DDI_FUNC_CTL(TRANSCODER_A)) |=
			(TRANS_DDI_BPC_8 | TRANS_DDI_MODE_SELECT_DVI |
			(PORT_C << TRANS_DDI_PORT_SHIFT) |
			TRANS_DDI_FUNC_ENABLE);
		if (IS_BROADWELL(dev_priv)) {
			vgpu_vreg_t(vgpu, PORT_CLK_SEL(PORT_C)) &=
				~PORT_CLK_SEL_MASK;
			vgpu_vreg_t(vgpu, PORT_CLK_SEL(PORT_C)) |=
				PORT_CLK_SEL_LCPLL_810;
		}
		vgpu_vreg_t(vgpu, DDI_BUF_CTL(PORT_C)) &= ~DDI_BUF_CTL_ENABLE;
		vgpu_vreg_t(vgpu, DDI_BUF_CTL(PORT_C)) &= ~DDI_BUF_IS_IDLE;
		vgpu_vreg_t(vgpu, SFUSE_STRAP) |= SFUSE_STRAP_DDIC_DETECTED;
	}

	if (intel_vgpu_has_monitor_on_port(vgpu, PORT_D)) {
		vgpu_vreg_t(vgpu, DPLL_CTRL2) &=
			~DPLL_CTRL2_DDI_CLK_OFF(PORT_D);
		vgpu_vreg_t(vgpu, DPLL_CTRL2) |=
			DPLL_CTRL2_DDI_CLK_SEL(DPLL_ID_SKL_DPLL0, PORT_D);
		vgpu_vreg_t(vgpu, DPLL_CTRL2) |=
			DPLL_CTRL2_DDI_SEL_OVERRIDE(PORT_D);
		vgpu_vreg_t(vgpu, SDEISR) |= SDE_PORTD_HOTPLUG_CPT;
		vgpu_vreg_t(vgpu, TRANS_DDI_FUNC_CTL(TRANSCODER_A)) &=
			~(TRANS_DDI_BPC_MASK | TRANS_DDI_MODE_SELECT_MASK |
			TRANS_DDI_PORT_MASK);
		vgpu_vreg_t(vgpu, TRANS_DDI_FUNC_CTL(TRANSCODER_A)) |=
			(TRANS_DDI_BPC_8 | TRANS_DDI_MODE_SELECT_DVI |
			(PORT_D << TRANS_DDI_PORT_SHIFT) |
			TRANS_DDI_FUNC_ENABLE);
		if (IS_BROADWELL(dev_priv)) {
			vgpu_vreg_t(vgpu, PORT_CLK_SEL(PORT_D)) &=
				~PORT_CLK_SEL_MASK;
			vgpu_vreg_t(vgpu, PORT_CLK_SEL(PORT_D)) |=
				PORT_CLK_SEL_LCPLL_810;
		}
		vgpu_vreg_t(vgpu, DDI_BUF_CTL(PORT_D)) &= ~DDI_BUF_CTL_ENABLE;
		vgpu_vreg_t(vgpu, DDI_BUF_CTL(PORT_D)) &= ~DDI_BUF_IS_IDLE;
		vgpu_vreg_t(vgpu, SFUSE_STRAP) |= SFUSE_STRAP_DDID_DETECTED;
	}

	if ((IS_SKYLAKE(dev_priv) || IS_KABYLAKE(dev_priv)) &&
			intel_vgpu_has_monitor_on_port(vgpu, PORT_E)) {
		vgpu_vreg_t(vgpu, SDEISR) |= SDE_PORTE_HOTPLUG_SPT;
	}

	if (intel_vgpu_has_monitor_on_port(vgpu, PORT_A)) {
		if (IS_BROADWELL(dev_priv))
			vgpu_vreg_t(vgpu, GEN8_DE_PORT_ISR) |=
				GEN8_PORT_DP_A_HOTPLUG;
		else
			vgpu_vreg_t(vgpu, SDEISR) |= SDE_PORTA_HOTPLUG_SPT;

		vgpu_vreg_t(vgpu, DDI_BUF_CTL(PORT_A)) |= DDI_INIT_DISPLAY_DETECTED;
	}

	/* Clear host CRT status, so guest couldn't detect this host CRT. */
	if (IS_BROADWELL(dev_priv))
		vgpu_vreg_t(vgpu, PCH_ADPA) &= ~ADPA_CRT_HOTPLUG_MONITOR_MASK;

	/* Disable Primary/Sprite/Cursor plane */
	for_each_pipe(dev_priv, pipe) {
		vgpu_vreg_t(vgpu, DSPCNTR(pipe)) &= ~DISPLAY_PLANE_ENABLE;
		vgpu_vreg_t(vgpu, SPRCTL(pipe)) &= ~SPRITE_ENABLE;
		vgpu_vreg_t(vgpu, CURCNTR(pipe)) &= ~MCURSOR_MODE;
		vgpu_vreg_t(vgpu, CURCNTR(pipe)) |= MCURSOR_MODE_DISABLE;
	}

	vgpu_vreg_t(vgpu, PIPECONF(PIPE_A)) |= PIPECONF_ENABLE;
}

static void clean_virtual_dp_monitor(struct intel_vgpu *vgpu, int port_num)
{
	struct intel_vgpu_port *port = intel_vgpu_port(vgpu, port_num);

	kfree(port->edid);
	port->edid = NULL;

	kfree(port->dpcd);
	port->dpcd = NULL;
}

static int setup_virtual_monitor(struct intel_vgpu *vgpu, int port_num,
		int type, unsigned int resolution, void *edid, bool is_dp)
{
	struct intel_vgpu_port *port = intel_vgpu_port(vgpu, port_num);
	int valid_extensions = 1;
	struct edid *tmp_edid = NULL;

	if (WARN_ON(resolution >= GVT_EDID_NUM))
		return -EINVAL;

	if (edid)
		valid_extensions += ((struct edid *)edid)->extensions;
	port->edid = kzalloc(sizeof(*(port->edid))
			+ valid_extensions * EDID_SIZE, GFP_KERNEL);
	if (!port->edid)
		return -ENOMEM;

	port->dpcd = kzalloc(sizeof(*(port->dpcd)), GFP_KERNEL);
	if (!port->dpcd) {
		kfree(port->edid);
		return -ENOMEM;
	}

	if (edid)
		memcpy(port->edid->edid_block, edid, EDID_SIZE * valid_extensions);
	else
		memcpy(port->edid->edid_block, virtual_dp_monitor_edid[resolution],
				EDID_SIZE);

	/* Sometimes the physical display will report the EDID with no
	 * digital bit set, which will cause the guest fail to enumerate
	 * the virtual HDMI monitor. So here we will set the digital
	 * bit and re-calculate the checksum.
	 */
	tmp_edid = ((struct edid *)port->edid->edid_block);
	if (!(tmp_edid->input & DRM_EDID_INPUT_DIGITAL)) {
		tmp_edid->input += DRM_EDID_INPUT_DIGITAL;
		tmp_edid->checksum -= DRM_EDID_INPUT_DIGITAL;
	}

	port->edid->data_valid = true;

	if (is_dp) {
		memcpy(port->dpcd->data, dpcd_fix_data, DPCD_HEADER_SIZE);
		port->dpcd->data_valid = true;
		port->dpcd->data[DPCD_SINK_COUNT] = 0x1;
	}
	port->type = type;

	emulate_monitor_status_change(vgpu);

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
void intel_gvt_check_vblank_emulation(struct intel_gvt *gvt)
{
	struct intel_gvt_irq *irq = &gvt->irq;
	struct intel_vgpu *vgpu;
	int pipe, id;
	int found = false;

	mutex_lock(&gvt->lock);
	for_each_active_vgpu(gvt, vgpu, id) {
		for (pipe = 0; pipe < I915_MAX_PIPES; pipe++) {
			if (pipe_is_enabled(vgpu, pipe)) {
				found = true;
				break;
			}
		}
		if (found)
			break;
	}

	/* all the pipes are disabled */
	if (!found)
		hrtimer_cancel(&irq->vblank_timer.timer);
	else
		hrtimer_start(&irq->vblank_timer.timer,
			ktime_add_ns(ktime_get(), irq->vblank_timer.period),
			HRTIMER_MODE_ABS);
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

	if (pipe < PIPE_A || pipe > PIPE_C)
		return;

	for_each_set_bit(event, irq->flip_done_event[pipe],
			INTEL_GVT_EVENT_MAX) {
		clear_bit(event, irq->flip_done_event[pipe]);
		if (!pipe_is_enabled(vgpu, pipe))
			continue;

		vgpu_vreg_t(vgpu, PIPE_FLIPCOUNT_G4X(pipe))++;
		intel_vgpu_trigger_virtual_event(vgpu, event);
	}

	if (pipe_is_enabled(vgpu, pipe)) {
		vgpu_vreg_t(vgpu, PIPE_FRMCOUNT_G4X(pipe))++;
		intel_vgpu_trigger_virtual_event(vgpu, vblank_event[pipe]);
	}
}

static void emulate_vblank(struct intel_vgpu *vgpu)
{
	int pipe;

	mutex_lock(&vgpu->vgpu_lock);
	for_each_pipe(vgpu->gvt->dev_priv, pipe)
		emulate_vblank_on_pipe(vgpu, pipe);
	mutex_unlock(&vgpu->vgpu_lock);
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
	int id;

	mutex_lock(&gvt->lock);
	for_each_active_vgpu(gvt, vgpu, id)
		emulate_vblank(vgpu);
	mutex_unlock(&gvt->lock);
}

static void intel_gvt_vblank_work(struct work_struct *w)
{
	struct intel_gvt_pipe_info *pipe_info = container_of(w,
			struct intel_gvt_pipe_info, vblank_work);
	struct intel_gvt *gvt = pipe_info->gvt;
	struct intel_vgpu *vgpu;
	int id;

	mutex_lock(&gvt->lock);
	for_each_active_vgpu(gvt, vgpu, id)
		emulate_vblank_on_pipe(vgpu, pipe_info->pipe_num);
	mutex_unlock(&gvt->lock);
}

#define BITS_PER_DOMAIN 4
#define MAX_SCALERS_PER_DOMAIN 2

#define DOMAIN_SCALER_OWNER(owner, pipe, scaler) \
	((((owner) >> (pipe) * BITS_PER_DOMAIN * MAX_SCALERS_PER_DOMAIN) >>  \
	BITS_PER_DOMAIN * (scaler)) & 0xf)

int bxt_check_planes(struct intel_vgpu *vgpu, int pipe)
{
	int plane = 0;
	bool ret = false;

	for (plane = 0;
	     plane < ((INTEL_INFO(vgpu->gvt->dev_priv)->num_sprites[pipe]) + 1);
	     plane++) {
		if (vgpu->gvt->pipe_info[pipe].plane_owner[plane] == vgpu->id) {
			ret = true;
			break;
		}
	}
	return ret;
}

void intel_gvt_init_pipe_info(struct intel_gvt *gvt)
{
	enum pipe pipe;
	unsigned int scaler;
	unsigned int domain_scaler_owner = i915_modparams.domain_scaler_owner;
	struct drm_i915_private *dev_priv = gvt->dev_priv;

	for (pipe = PIPE_A; pipe <= PIPE_C; pipe++) {
		gvt->pipe_info[pipe].pipe_num = pipe;
		gvt->pipe_info[pipe].gvt = gvt;
		INIT_WORK(&gvt->pipe_info[pipe].vblank_work,
				intel_gvt_vblank_work);
		/* Each nibble represents domain id
		 * ids can be from 0-F. 0 for Dom0, 1,2,3...0xF for DomUs
		 * scaler_owner[i] holds the id of the domain that owns it,
		 * eg:0,1,2 etc
		 */
		for_each_universal_scaler(dev_priv, pipe, scaler)
			gvt->pipe_info[pipe].scaler_owner[scaler] =
			DOMAIN_SCALER_OWNER(domain_scaler_owner, pipe, scaler);
	}
}

bool gvt_emulate_hdmi;

int setup_virtual_monitors(struct intel_vgpu *vgpu)
{
	struct intel_connector *connector = NULL;
	struct drm_connector_list_iter conn_iter;
	struct drm_i915_private *dev_priv = vgpu->gvt->dev_priv;
	struct edid *edid;
	int pipe = 0;
	int ret = 0;
	int type = gvt_emulate_hdmi ? GVT_HDMI_A : GVT_DP_A;
	int port = PORT_B;

	/* BXT have to use port A for HDMI to support 3 HDMI monitors */
	if (IS_BROXTON(dev_priv))
		port = PORT_A;

	drm_connector_list_iter_begin(&vgpu->gvt->dev_priv->drm, &conn_iter);
	for_each_intel_connector_iter(connector, &conn_iter) {
		if (connector->encoder->get_hw_state(connector->encoder,
					&pipe)) {
			/* if no planes are allocated for this pipe, skip it */
			if (i915_modparams.avail_planes_per_pipe &&
			    !bxt_check_planes(vgpu, pipe))
				continue;

			if (connector->panel.fixed_mode) {
				edid = intel_gvt_create_edid_from_mode(
						connector->panel.fixed_mode);
			} else if (connector->detect_edid) {
				edid = connector->detect_edid;
			} else {
				continue;
			}

			/* Get (Dom0) port associated with current pipe. */
			port = connector->encoder->port;
			ret = setup_virtual_monitor(vgpu, port,
				type, 0, edid, !gvt_emulate_hdmi);
			if (ret)
				return ret;
			type++;
			port++;
		}
	}
	drm_connector_list_iter_end(&conn_iter);
	return 0;
}

void clean_virtual_monitors(struct intel_vgpu *vgpu)
{
	int port = 0;

	for (port = PORT_A; port < I915_MAX_PORTS; port++) {
		struct intel_vgpu_port *p = intel_vgpu_port(vgpu, port);

		if (p->edid)
			clean_virtual_dp_monitor(vgpu, port);
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
	struct drm_i915_private *dev_priv = vgpu->gvt->dev_priv;

	if (IS_BROXTON(dev_priv) || IS_KABYLAKE(dev_priv))
		clean_virtual_monitors(vgpu);
	else if (IS_SKYLAKE(dev_priv))
		clean_virtual_dp_monitor(vgpu, PORT_D);
	else
		clean_virtual_dp_monitor(vgpu, PORT_B);
}

/**
 * intel_vgpu_init_display- initialize vGPU virtual display emulation
 * @vgpu: a vGPU
 *
 * This function is used to initialize vGPU virtual display emulation stuffs
 *
 * Returns:
 * Zero on success, negative error code if failed.
 *
 */
int intel_vgpu_init_display(struct intel_vgpu *vgpu, u64 resolution)
{
	struct drm_i915_private *dev_priv = vgpu->gvt->dev_priv;

	intel_vgpu_init_i2c_edid(vgpu);

	if (IS_BROXTON(dev_priv) || IS_KABYLAKE(dev_priv))
		return setup_virtual_monitors(vgpu);
	else if (IS_SKYLAKE(dev_priv))
		return setup_virtual_monitor(vgpu, PORT_D, GVT_DP_D,
						resolution, NULL, true);
	else
		return setup_virtual_monitor(vgpu, PORT_B, GVT_DP_B,
						resolution, NULL, true);
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
	struct drm_i915_private *dev_priv = vgpu->gvt->dev_priv;
	unsigned int pipe, plane;
	struct intel_crtc *crtc;
	struct intel_crtc_state *crtc_state;
	struct drm_display_mode mode;
	bool found = false;

	/* we will get the gop output on the first pipe the vgpu ownes */
	for_each_pipe(dev_priv, pipe) {
		for_each_universal_plane(dev_priv, pipe, plane) {
			if (vgpu->gvt->pipe_info[pipe].plane_owner[plane]
				    == vgpu->id) {
				found = true;
				break;
			}
		}
		if (found)
			break;
	}

	if (found == false) {
		gvt_dbg_dpy("Failed to find owned plane for %d", vgpu->id);
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

		intel_gvt_hypervisor_map_gfn_to_mfn(vgpu,
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
	unsigned int pipe, plane;
	struct drm_i915_private *dev_priv = vgpu->gvt->dev_priv;
	bool found = false;

	u32 width, height, Bpp;
	u32 stride, ctl, surf;
	unsigned long irqflags;

	width = vgpu_vreg_t(vgpu, vgtif_reg(gop.width));
	height = vgpu_vreg_t(vgpu, vgtif_reg(gop.height));
	Bpp = vgpu_vreg_t(vgpu, vgtif_reg(gop.Bpp));

	DRM_INFO("Set up display w:%u h:%u for GOP\n", width, height);

	/* we will display the gop output on the first plane the vgpu ownes */
	for_each_pipe(dev_priv, pipe) {
		for_each_universal_plane(dev_priv, pipe, plane) {
			if (vgpu->gvt->pipe_info[pipe].plane_owner[plane]
				    == vgpu->id) {
				found = true;
				break;
			}
		}
		if (found)
			break;
	}

	if (!found) {
		gvt_dbg_dpy("Failed to find owned plane for %d", vgpu->id);
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
	I915_WRITE_FW(PLANE_OFFSET(pipe, plane), 0);
	I915_WRITE_FW(PLANE_STRIDE(pipe, plane), stride);
	I915_WRITE_FW(PLANE_SIZE(pipe, plane), (height << 16) | width);
	I915_WRITE_FW(PLANE_AUX_DIST(pipe, plane), 0xFFFFF000);
	I915_WRITE_FW(PLANE_AUX_OFFSET(pipe, plane), 0);
	I915_WRITE_FW(PLANE_POS(pipe, plane), 0);
	I915_WRITE_FW(PLANE_CTL(pipe, plane), ctl);
	I915_WRITE_FW(PLANE_SURF(pipe, plane), surf);
	POSTING_READ_FW(PLANE_SURF(pipe, plane));
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
