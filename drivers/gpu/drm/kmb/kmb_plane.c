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
#include "kmb_plane.h"
#include "kmb_crtc.h"
#include "kmb_regs.h"
#include "kmb_drv.h"

static int kmb_plane_atomic_check(struct drm_plane *plane,
				  struct drm_plane_state *state)
{
/* TBD below structure will be used for implementation later
 *	struct drm_crtc_state *crtc_state;
 */
	/* TBD */
	/* Plane based checking */

	return 0;
}

static void kmb_plane_atomic_update(struct drm_plane *plane,
				    struct drm_plane_state *state)
{
	struct drm_framebuffer *fb = plane->state->fb;
	struct kmb_drm_private *lcd;
	dma_addr_t addr;
	unsigned int width;
	unsigned int height;
	unsigned int i;
	unsigned int dma_len;
	struct kmb_plane_state *kmb_state = to_kmb_plane_state(plane->state);
	unsigned int dma_cfg;

	if (!fb)
		return;

	lcd = plane->dev->dev_private;

	/* TBD */
	/*set LCD_LAYERn_WIDTH, LCD_LAYERn_HEIGHT, LCD_LAYERn_COL_START,
	 * LCD_LAYERn_ROW_START, LCD_LAYERn_CFG
	 * CFG should set the pixel format, FIFO level and BPP
	 */

	/* we may have to set LCD_DMA_VSTRIDE_ENABLE in the future */
	dma_cfg = LCD_DMA_LAYER_ENABLE | LCD_DMA_LAYER_AUTO_UPDATE
	    | LCD_DMA_LAYER_CONT_UPDATE | LCD_DMA_LAYER_AXI_BURST_1;

	for (i = 0; i < kmb_state->no_planes; i++) {
		/* disable DMA first */
		kmb_write(lcd, LCD_LAYERn_DMA_CFG(i), ~LCD_DMA_LAYER_ENABLE);

		addr = drm_fb_cma_get_gem_addr(fb, plane->state, i);
		kmb_write(lcd, LCD_LAYERn_DMA_START_ADDR(i), addr);
		kmb_write(lcd, LCD_LAYERn_DMA_START_SHADOW(i), addr);

		width = fb->width;
		height = fb->height;
		dma_len = width * height * fb->format->cpp[i];
		kmb_write(lcd, LCD_LAYERn_DMA_LEN(i), dma_len);

		kmb_write(lcd, LCD_LAYERn_DMA_LINE_VSTRIDE(i), fb->pitches[0]);
		kmb_write(lcd, LCD_LAYERn_DMA_LINE_WIDTH(i),
			  (width * fb->format->cpp[i]));

		/* enable DMA */
		kmb_write(lcd, LCD_LAYERn_DMA_CFG(i), dma_cfg);
	}
}

static const struct drm_plane_helper_funcs kmb_plane_helper_funcs = {
	.atomic_check = kmb_plane_atomic_check,
	.atomic_update = kmb_plane_atomic_update,
};

void kmb_plane_destroy(struct drm_plane *plane)
{
	drm_plane_cleanup(plane);
}

static void kmb_destroy_plane_state(struct drm_plane *plane,
				    struct drm_plane_state *state)
{
	struct kmb_plane_state *kmb_state = to_kmb_plane_state(state);

	__drm_atomic_helper_plane_destroy_state(state);
	kfree(kmb_state);
}

struct drm_plane_state *kmb_plane_duplicate_state(struct drm_plane *plane)
{
	struct drm_plane_state *state;
	struct kmb_plane_state *kmb_state;

	kmb_state = kmemdup(plane->state, sizeof(*kmb_state), GFP_KERNEL);

	if (!kmb_state)
		return NULL;

	state = &kmb_state->base_plane_state;
	__drm_atomic_helper_plane_duplicate_state(plane, state);

	return state;
}

static void kmb_plane_reset(struct drm_plane *plane)
{
	struct kmb_plane_state *kmb_state = to_kmb_plane_state(plane->state);

	if (kmb_state)
		__drm_atomic_helper_plane_destroy_state
		    (&kmb_state->base_plane_state);
	kfree(kmb_state);

	plane->state = NULL;
	kmb_state = kzalloc(sizeof(*kmb_state), GFP_KERNEL);
	if (kmb_state) {
		kmb_state->base_plane_state.plane = plane;
		kmb_state->base_plane_state.rotation = DRM_MODE_ROTATE_0;
		plane->state = &kmb_state->base_plane_state;
		kmb_state->no_planes = KMB_MAX_PLANES;
	}
}

static const struct drm_plane_funcs kmb_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = kmb_plane_destroy,
	.reset = kmb_plane_reset,
	.atomic_duplicate_state = kmb_plane_duplicate_state,
	.atomic_destroy_state = kmb_destroy_plane_state,
};

/* graphics layer ( layers 2 & 3) formats, only packed formats  are supported*/
static const u32 kmb_formats_g[] = {
	DRM_FORMAT_RGB332,
	DRM_FORMAT_XRGB4444, DRM_FORMAT_XBGR4444,
	DRM_FORMAT_ARGB4444, DRM_FORMAT_ABGR4444,
	DRM_FORMAT_XRGB1555, DRM_FORMAT_XBGR1555,
	DRM_FORMAT_ARGB1555, DRM_FORMAT_ABGR1555,
	DRM_FORMAT_RGB565, DRM_FORMAT_BGR565,
	DRM_FORMAT_RGB888, DRM_FORMAT_BGR888,
	DRM_FORMAT_XRGB8888, DRM_FORMAT_XBGR8888,
	DRM_FORMAT_ARGB8888, DRM_FORMAT_ABGR8888,
	DRM_FORMAT_XRGB2101010, DRM_FORMAT_XBGR2101010,
	DRM_FORMAT_YUYV, DRM_FORMAT_YVYU,
	DRM_FORMAT_UYVY, DRM_FORMAT_VYUY,
};

/* video layer (0 & 1) formats, packed and planar formats are supported */
static const u32 kmb_formats_v[] = {
	/* packed formats */
	DRM_FORMAT_RGB332,
	DRM_FORMAT_XRGB4444, DRM_FORMAT_XBGR4444,
	DRM_FORMAT_ARGB4444, DRM_FORMAT_ABGR4444,
	DRM_FORMAT_XRGB1555, DRM_FORMAT_XBGR1555,
	DRM_FORMAT_ARGB1555, DRM_FORMAT_ABGR1555,
	DRM_FORMAT_RGB565, DRM_FORMAT_BGR565,
	DRM_FORMAT_RGB888, DRM_FORMAT_BGR888,
	DRM_FORMAT_XRGB8888, DRM_FORMAT_XBGR8888,
	DRM_FORMAT_ARGB8888, DRM_FORMAT_ABGR8888,
	DRM_FORMAT_XRGB2101010, DRM_FORMAT_XBGR2101010,
	DRM_FORMAT_YUYV, DRM_FORMAT_YVYU,
	DRM_FORMAT_UYVY, DRM_FORMAT_VYUY,
	/*planar formats */
	DRM_FORMAT_YUV411, DRM_FORMAT_YVU411,
	DRM_FORMAT_YUV420, DRM_FORMAT_YVU420,
	DRM_FORMAT_YUV422, DRM_FORMAT_YVU422,
	DRM_FORMAT_YUV444, DRM_FORMAT_YVU444,
	DRM_FORMAT_NV12, DRM_FORMAT_NV21,
};

struct drm_plane *kmb_plane_init(struct drm_device *drm)
{
	struct kmb_drm_private *lcd = drm->dev_private;
	struct drm_plane *plane = NULL;
	struct drm_plane *primary = NULL;
	int i = 0;
	int ret;
	enum drm_plane_type plane_type;
	const uint32_t *plane_formats;
	int num_plane_formats;

	for (i = 0; i < lcd->n_layers; i++) {

		plane = devm_kzalloc(drm->dev, sizeof(*plane), GFP_KERNEL);

		if (!plane)
			return ERR_PTR(-ENOMEM);

		plane_type = (i == 0) ? DRM_PLANE_TYPE_PRIMARY :
		    DRM_PLANE_TYPE_OVERLAY;
		if (i < 2) {
			plane_formats = kmb_formats_v;
			num_plane_formats = ARRAY_SIZE(kmb_formats_v);
		} else {
			plane_formats = kmb_formats_g;
			num_plane_formats = ARRAY_SIZE(kmb_formats_g);
		}

		ret = drm_universal_plane_init(drm, plane, 0xFF,
				       &kmb_plane_funcs, plane_formats,
				       num_plane_formats,
				       NULL, plane_type, "plane %d", i);
		if (ret < 0)
			goto cleanup;

		drm_plane_helper_add(plane, &kmb_plane_helper_funcs);
		if (plane_type == DRM_PLANE_TYPE_PRIMARY) {
			primary = plane;
			lcd->plane = plane;
		}
	}

cleanup:
	return primary;
}
