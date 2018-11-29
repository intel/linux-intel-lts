// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright © 2018 Intel Corporation
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
 *
 */

#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_plane_helper.h>
#include <linux/clk.h>
#include <linux/of_graph.h>
#include <linux/platform_data/simplefb.h>
#include <video/videomode.h>
#include "kmb_crtc.h"
#include "kmb_drv.h"
#include "kmb_plane.h"
#include "kmb_regs.h"

static void kmb_crtc_cleanup(struct drm_crtc *crtc)
{
	struct kmb_crtc *l_crtc = to_kmb_crtc(crtc);

	drm_crtc_cleanup(crtc);
	kfree(l_crtc);
}

static int kmb_crtc_enable_vblank(struct drm_crtc *crtc)
{
	struct kmb_drm_private *lcd = crtc_to_kmb_priv(crtc);

	/*clear interrupt */
	kmb_write(lcd, LCD_INT_CLEAR, LCD_INT_VERT_COMP);
	/*set which interval to generate vertical interrupt */
	kmb_write(lcd, LCD_VSTATUS_COMPARE, LCD_VSTATUS_COMPARE_VSYNC);
	/* enable vertical interrupt */
	kmb_write(lcd, LCD_INT_ENABLE, LCD_INT_VERT_COMP);
	return 0;
}

static void kmb_crtc_disable_vblank(struct drm_crtc *crtc)
{
	struct kmb_drm_private *lcd = crtc_to_kmb_priv(crtc);

	/*clear interrupt */
	kmb_write(lcd, LCD_INT_CLEAR, LCD_INT_VERT_COMP);
	/* disable vertical interrupt */
	kmb_write(lcd, LCD_INT_ENABLE, 0);

/* TBD
 *  set the BIT2 (VERTICAL_COMPARE_INTERRUPT) of the LCD_INT_ENABLE register
 *  set the required bit LCD_VSTATUS_COMPARE register
 *  Not sure if anything needs to be done in the ICB
 */
}

static const struct drm_crtc_funcs kmb_crtc_funcs = {
	.destroy = kmb_crtc_cleanup,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.reset = drm_atomic_helper_crtc_reset,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = kmb_crtc_enable_vblank,
	.disable_vblank = kmb_crtc_disable_vblank,
};

static void kmb_crtc_mode_set_nofb(struct drm_crtc *crtc)
{
	struct kmb_drm_private *lcd = crtc_to_kmb_priv(crtc);
	struct drm_display_mode *m = &crtc->state->adjusted_mode;
	struct videomode vm;
	int vsync_start_offset;
	int vsync_end_offset;
	unsigned int ctrl = 0;

	vm.vfront_porch = m->crtc_vsync_start - m->crtc_vdisplay;
	vm.vback_porch = m->crtc_vtotal - m->crtc_vsync_end;
	vm.vsync_len = m->crtc_vsync_end - m->crtc_vsync_start;
	vm.hfront_porch = m->crtc_hsync_start - m->crtc_hdisplay;
	vm.hback_porch = m->crtc_htotal - m->crtc_hsync_end;
	vm.hsync_len = m->crtc_hsync_end - m->crtc_hsync_start;

	vsync_start_offset = m->crtc_vsync_start - m->crtc_hsync_start;
	vsync_end_offset = m->crtc_vsync_end - m->crtc_hsync_end;

	kmb_write(lcd, LCD_V_ACTIVEHEIGHT, m->crtc_vdisplay - 1);
	kmb_write(lcd, LCD_V_BACKPORCH, vm.vback_porch - 1);
	kmb_write(lcd, LCD_V_FRONTPORCH, vm.vfront_porch - 1);
	kmb_write(lcd, LCD_VSYNC_WIDTH, vm.vsync_len - 1);
	kmb_write(lcd, LCD_H_ACTIVEWIDTH, m->crtc_hdisplay - 1);
	kmb_write(lcd, LCD_H_BACKPORCH, vm.hback_porch - 1);
	kmb_write(lcd, LCD_H_FRONTPORCH, vm.hfront_porch - 1);
	kmb_write(lcd, LCD_HSYNC_WIDTH, vm.hsync_len - 1);

	if (m->flags == DRM_MODE_FLAG_INTERLACE) {
		kmb_write(lcd, LCD_VSYNC_WIDTH_EVEN, vm.vsync_len - 1);
		kmb_write(lcd, LCD_V_BACKPORCH_EVEN, vm.vback_porch - 1);
		kmb_write(lcd, LCD_V_FRONTPORCH_EVEN, vm.vfront_porch - 1);
		kmb_write(lcd, LCD_V_ACTIVEHEIGHT_EVEN, m->crtc_vdisplay - 1);
		kmb_write(lcd, LCD_VSYNC_START_EVEN, vsync_start_offset);
		kmb_write(lcd, LCD_VSYNC_END_EVEN, vsync_end_offset);
	}
	/* enable all 4 layers */
	ctrl = LCD_CTRL_ENABLE | LCD_CTRL_VL1_ENABLE
	    | LCD_CTRL_VL2_ENABLE | LCD_CTRL_GL1_ENABLE | LCD_CTRL_GL2_ENABLE;
	ctrl |= LCD_CTRL_PROGRESSIVE | LCD_CTRL_TIM_GEN_ENABLE
	    | LCD_CTRL_OUTPUT_ENABLED;
	kmb_write(lcd, LCD_CONTROL, ctrl);

	kmb_write(lcd, LCD_TIMING_GEN_TRIG, ENABLE);

	/* TBD */
	/* set clocks here */
}

static void kmb_crtc_atomic_enable(struct drm_crtc *crtc,
				   struct drm_crtc_state *old_state)
{
	struct kmb_drm_private *lcd = crtc_to_kmb_priv(crtc);

	clk_prepare_enable(lcd->clk);
	kmb_crtc_mode_set_nofb(crtc);
	drm_crtc_vblank_on(crtc);
}

static void kmb_crtc_atomic_disable(struct drm_crtc *crtc,
				    struct drm_crtc_state *old_state)
{
	struct kmb_drm_private *lcd = crtc_to_kmb_priv(crtc);

	/* always disable planes on the CRTC that is being turned off */
	drm_atomic_helper_disable_planes_on_crtc(old_state, false);

	drm_crtc_vblank_off(crtc);
	clk_disable_unprepare(lcd->clk);
}

static void kmb_crtc_atomic_begin(struct drm_crtc *crtc,
				  struct drm_crtc_state *state)
{
	/* TBD */
	/*disable  vblank interrupts here
	 * clear BIT 2 (VERTICAL_COMPARE_INTERRUPT) LCD_INT_ENABLE
	 */
}

static void kmb_crtc_atomic_flush(struct drm_crtc *crtc,
				  struct drm_crtc_state *state)
{
	/* TBD */
	/*enable  vblank interrupts after
	 * set BIT 2 (VERTICAL_COMPARE_INTERRUPT) LCD_INT_ENABLE
	 */

	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc->state->event)
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
	crtc->state->event = NULL;
	spin_unlock_irq(&crtc->dev->event_lock);

}

static const struct drm_crtc_helper_funcs kmb_crtc_helper_funcs = {
	.atomic_begin = kmb_crtc_atomic_begin,
	.atomic_enable = kmb_crtc_atomic_enable,
	.atomic_disable = kmb_crtc_atomic_disable,
	.atomic_flush = kmb_crtc_atomic_flush,
};

int kmb_setup_crtc(struct drm_device *drm)
{
	struct kmb_drm_private *lcd = drm->dev_private;
	struct drm_plane *primary;
	int ret;

	primary = kmb_plane_init(drm);
	if (IS_ERR(primary))
		return PTR_ERR(primary);

	ret = drm_crtc_init_with_planes(drm, &lcd->crtc, primary, NULL,
					&kmb_crtc_funcs, NULL);
	if (ret) {
		kmb_plane_destroy(primary);
		return ret;
	}

	drm_crtc_helper_add(&lcd->crtc, &kmb_crtc_helper_funcs);
	return 0;
}
