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
};

#define MAX_FORMAT_G	(ARRAY_SIZE(kmb_formats_g))
#define MAX_FORMAT_V	(ARRAY_SIZE(kmb_formats_v))

/* video layer ( 0 & 1) formats, packed and planar formats are supported */
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
	/*planar formats */
	DRM_FORMAT_YUV420, DRM_FORMAT_YVU420,
	DRM_FORMAT_YUV422, DRM_FORMAT_YVU422,
	DRM_FORMAT_YUV444, DRM_FORMAT_YVU444,
	DRM_FORMAT_NV12, DRM_FORMAT_NV21,
};

static unsigned int check_pixel_format(struct drm_plane *plane, u32 format)
{
	int i;

	for (i = 0; i < plane->format_count; i++) {
		if (plane->format_types[i] == format)
			return 0;
	}
	return -EINVAL;
}

static int kmb_plane_atomic_check(struct drm_plane *plane,
				  struct drm_plane_state *state)
{
	struct drm_framebuffer *fb;
	int ret;

	fb = state->fb;

	ret = check_pixel_format(plane, fb->format->format);
	if (ret)
		return ret;

	if (state->crtc_w > KMB_MAX_WIDTH || state->crtc_h > KMB_MAX_HEIGHT)
		return -EINVAL;
	return 0;
}

unsigned int set_pixel_format(u32 format)
{
	unsigned int val = 0;

	switch (format) {
		/*planar formats */
	case DRM_FORMAT_YUV444:
		val = LCD_LAYER_FORMAT_YCBCR444PLAN | LCD_LAYER_PLANAR_STORAGE;
		break;
	case DRM_FORMAT_YVU444:
		val = LCD_LAYER_FORMAT_YCBCR444PLAN | LCD_LAYER_PLANAR_STORAGE
		    | LCD_LAYER_CRCB_ORDER;
		break;
	case DRM_FORMAT_YUV422:
		val = LCD_LAYER_FORMAT_YCBCR422PLAN | LCD_LAYER_PLANAR_STORAGE;
		break;
	case DRM_FORMAT_YVU422:
		val = LCD_LAYER_FORMAT_YCBCR422PLAN | LCD_LAYER_PLANAR_STORAGE
		    | LCD_LAYER_CRCB_ORDER;
		break;
	case DRM_FORMAT_YUV420:
		val = LCD_LAYER_FORMAT_YCBCR420PLAN | LCD_LAYER_PLANAR_STORAGE;
		break;
	case DRM_FORMAT_YVU420:
		val = LCD_LAYER_FORMAT_YCBCR420PLAN | LCD_LAYER_PLANAR_STORAGE
		    | LCD_LAYER_CRCB_ORDER;
		break;
	case DRM_FORMAT_NV12:
		val = LCD_LAYER_FORMAT_NV12 | LCD_LAYER_PLANAR_STORAGE;
		break;
	case DRM_FORMAT_NV21:
		val = LCD_LAYER_FORMAT_NV12 | LCD_LAYER_PLANAR_STORAGE
		    | LCD_LAYER_CRCB_ORDER;
		break;
		/* packed formats */
	case DRM_FORMAT_RGB332:
		val = LCD_LAYER_FORMAT_RGB332;
		break;
	case DRM_FORMAT_XBGR4444:
		val = LCD_LAYER_FORMAT_RGBX4444 | LCD_LAYER_BGR_ORDER;
		break;
	case DRM_FORMAT_ARGB4444:
		val = LCD_LAYER_FORMAT_RGBA4444;
		break;
	case DRM_FORMAT_ABGR4444:
		val = LCD_LAYER_FORMAT_RGBA4444 | LCD_LAYER_BGR_ORDER;
		break;
	case DRM_FORMAT_XRGB1555:
		val = LCD_LAYER_FORMAT_XRGB1555;
		break;
	case DRM_FORMAT_XBGR1555:
		val = LCD_LAYER_FORMAT_XRGB1555 | LCD_LAYER_BGR_ORDER;
		break;
	case DRM_FORMAT_ARGB1555:
		val = LCD_LAYER_FORMAT_RGBA1555;
		break;
	case DRM_FORMAT_ABGR1555:
		val = LCD_LAYER_FORMAT_RGBA1555 | LCD_LAYER_BGR_ORDER;
		break;
	case DRM_FORMAT_RGB565:
		val = LCD_LAYER_FORMAT_RGB565;
		break;
	case DRM_FORMAT_BGR565:
		val = LCD_LAYER_FORMAT_RGB565 | LCD_LAYER_BGR_ORDER;
		break;
	case DRM_FORMAT_RGB888:
		val = LCD_LAYER_FORMAT_RGB888;
		break;
	case DRM_FORMAT_BGR888:
		val = LCD_LAYER_FORMAT_RGB888 | LCD_LAYER_BGR_ORDER;
		break;
	case DRM_FORMAT_XRGB8888:
		val = LCD_LAYER_FORMAT_RGBX8888;
		break;
	case DRM_FORMAT_XBGR8888:
		val = LCD_LAYER_FORMAT_RGBX8888 | LCD_LAYER_BGR_ORDER;
		break;
	case DRM_FORMAT_ARGB8888:
		val = LCD_LAYER_FORMAT_RGBA8888;
		break;
	case DRM_FORMAT_ABGR8888:
		val = LCD_LAYER_FORMAT_RGBA8888 | LCD_LAYER_BGR_ORDER;
		break;
	}
	return val;
}

unsigned int set_bits_per_pixel(const struct drm_format_info *format)
{
	int i;
	u32 bpp = 0;
	unsigned int val = 0;

	for (i = 0; i < format->num_planes; i++)
		bpp += 8 * format->cpp[i];

	switch (bpp) {
	case 8:
		val = LCD_LAYER_8BPP;
		break;
	case 16:
		val = LCD_LAYER_16BPP;
		break;
	case 24:
		val = LCD_LAYER_24BPP;
		break;
	case 32:
		val = LCD_LAYER_32BPP;
		break;
	}
	return val;
}

static void kmb_plane_atomic_update(struct drm_plane *plane,
				    struct drm_plane_state *state)
{
	struct drm_framebuffer *fb = plane->state->fb;
	struct kmb_drm_private *lcd;
	dma_addr_t addr;
	unsigned int width;
	unsigned int height;
	unsigned int dma_len;
	struct kmb_plane *kmb_plane = to_kmb_plane(plane);
	unsigned int dma_cfg;
	unsigned int ctrl = 0, val = 0, out_format = 0;
	unsigned int src_w, src_h, crtc_x, crtc_y;
	unsigned char plane_id = kmb_plane->id;

	if (!fb)
		return;

	lcd = plane->dev->dev_private;

	src_w = plane->state->src_w >> 16;
	src_h = plane->state->src_h >> 16;
	crtc_x = plane->state->crtc_x;
	crtc_y = plane->state->crtc_y;

	kmb_write_lcd(LCD_LAYERn_WIDTH(plane_id), src_w-1);
	kmb_write_lcd(LCD_LAYERn_HEIGHT(plane_id), src_h-1);
	kmb_write_lcd(LCD_LAYERn_COL_START(plane_id), crtc_x);
	kmb_write_lcd(LCD_LAYERn_ROW_START(plane_id), crtc_y);

	val = set_pixel_format(fb->format->format);
	val |= set_bits_per_pixel(fb->format);
	/*CHECKME Leon drvr sets it to 50 try this for now */
	val |= LCD_LAYER_FIFO_50;
	kmb_write_lcd(LCD_LAYERn_CFG(plane_id), val);

	switch (plane_id) {
	case LAYER_0:
		ctrl = LCD_CTRL_VL1_ENABLE;
		break;
	case LAYER_1:
		ctrl = LCD_CTRL_VL2_ENABLE;
		break;
	case LAYER_2:
		ctrl = LCD_CTRL_GL1_ENABLE;
		break;
	case LAYER_3:
		ctrl = LCD_CTRL_GL2_ENABLE;
		break;
	}

	ctrl |= LCD_CTRL_ENABLE;
	ctrl |= LCD_CTRL_PROGRESSIVE | LCD_CTRL_TIM_GEN_ENABLE
		| LCD_CTRL_OUTPUT_ENABLED;
	kmb_write_lcd(LCD_CONTROL, ctrl);

	/*TBD check visible? */

	/* we may have to set LCD_DMA_VSTRIDE_ENABLE in the future */
	dma_cfg = LCD_DMA_LAYER_ENABLE | LCD_DMA_LAYER_AUTO_UPDATE
	    | LCD_DMA_LAYER_CONT_UPDATE | LCD_DMA_LAYER_AXI_BURST_1;

	/* disable DMA first */
	kmb_write_lcd(LCD_LAYERn_DMA_CFG(plane_id), ~LCD_DMA_LAYER_ENABLE);

	addr = drm_fb_cma_get_gem_addr(fb, plane->state, plane_id);
	kmb_write_lcd(LCD_LAYERn_DMA_START_ADDR(plane_id), addr);
	kmb_write_lcd(LCD_LAYERn_DMA_START_SHADOW(plane_id), addr);

	width = fb->width;
	height = fb->height;
	dma_len = width * height * fb->format->cpp[plane_id];
	kmb_write_lcd(LCD_LAYERn_DMA_LEN(plane_id), dma_len);

	kmb_write_lcd(LCD_LAYERn_DMA_LINE_VSTRIDE(plane_id),
			fb->pitches[plane_id]);
	kmb_write_lcd(LCD_LAYERn_DMA_LINE_WIDTH(plane_id),
			(width*fb->format->cpp[plane_id]));

	/* enable DMA */
	kmb_write_lcd(LCD_LAYERn_DMA_CFG(plane_id), dma_cfg);

	/* FIXME no doc on how to set output format - may need to change
	 * this later
	 */
	if (val & LCD_LAYER_BGR_ORDER)
		out_format |= LCD_OUTF_BGR_ORDER;
	else if (val & LCD_LAYER_CRCB_ORDER)
		out_format |= LCD_OUTF_CRCB_ORDER;
	/* do not interleave RGB channels for mipi Tx compatibility */
	out_format |= LCD_OUTF_MIPI_RGB_MODE;
	/* pixel format from LCD_LAYER_CFG */
	out_format |= ((val >> 9) & 0x1F);
	kmb_write_lcd(LCD_OUT_FORMAT_CFG, out_format);
}

static const struct drm_plane_helper_funcs kmb_plane_helper_funcs = {
	.atomic_check = kmb_plane_atomic_check,
	.atomic_update = kmb_plane_atomic_update,
};

void kmb_plane_destroy(struct drm_plane *plane)
{
	struct kmb_plane *kmb_plane = to_kmb_plane(plane);
	drm_plane_cleanup(plane);
	kfree(kmb_plane);
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

struct kmb_plane *kmb_plane_init(struct drm_device *drm)
{
	struct kmb_drm_private *lcd = drm->dev_private;
	struct kmb_plane *plane = NULL;
	struct kmb_plane *primary = NULL;
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

		ret =
		    drm_universal_plane_init(drm, &plane->base_plane,
					     POSSIBLE_CRTCS, &kmb_plane_funcs,
					     plane_formats, num_plane_formats,
					     NULL, plane_type, "plane %d", i);
		if (ret < 0)
			goto cleanup;

		drm_plane_helper_add(&plane->base_plane,
				     &kmb_plane_helper_funcs);
		if (plane_type == DRM_PLANE_TYPE_PRIMARY) {
			primary = plane;
			lcd->plane = plane;
		}
		plane->id = i;
	}

cleanup:
	return primary;
}
