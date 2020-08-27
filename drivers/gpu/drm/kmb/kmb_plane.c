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

struct layer_status plane_status[KMB_MAX_PLANES];
const uint32_t layer_irqs[] = {
	LCD_INT_VL0,
	LCD_INT_VL1,
	LCD_INT_GL0,
	LCD_INT_GL1
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
	if (!fb || !state->crtc)
		return 0;

	ret = check_pixel_format(plane, fb->format->format);
	if (ret)
		return ret;

	if (state->crtc_w > KMB_MAX_WIDTH || state->crtc_h > KMB_MAX_HEIGHT)
		return -EINVAL;
	if (state->crtc_w < KMB_MIN_WIDTH || state->crtc_h < KMB_MIN_HEIGHT)
		return -EINVAL;
	return 0;
}

static void kmb_plane_atomic_disable(struct drm_plane *plane,
				     struct drm_plane_state *state)
{
	struct kmb_plane *kmb_plane = to_kmb_plane(plane);
	int plane_id = kmb_plane->id;

	switch (plane_id) {
	case LAYER_0:
		plane_status[plane_id].ctrl = LCD_CTRL_VL1_ENABLE;
		break;
	case LAYER_1:
		plane_status[plane_id].ctrl = LCD_CTRL_VL2_ENABLE;
		break;
	case LAYER_2:
		plane_status[plane_id].ctrl = LCD_CTRL_GL1_ENABLE;
		break;
	case LAYER_3:
		plane_status[plane_id].ctrl = LCD_CTRL_GL2_ENABLE;
		break;
	}

	plane_status[plane_id].disable = true;
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
		/* looks hw requires B & G to be swapped when RGB */
	case DRM_FORMAT_RGB332:
		val = LCD_LAYER_FORMAT_RGB332 | LCD_LAYER_BGR_ORDER;
		break;
	case DRM_FORMAT_XBGR4444:
		val = LCD_LAYER_FORMAT_RGBX4444;
		break;
	case DRM_FORMAT_ARGB4444:
		val = LCD_LAYER_FORMAT_RGBA4444 | LCD_LAYER_BGR_ORDER;
		break;
	case DRM_FORMAT_ABGR4444:
		val = LCD_LAYER_FORMAT_RGBA4444;
		break;
	case DRM_FORMAT_XRGB1555:
		val = LCD_LAYER_FORMAT_XRGB1555 | LCD_LAYER_BGR_ORDER;
		break;
	case DRM_FORMAT_XBGR1555:
		val = LCD_LAYER_FORMAT_XRGB1555;
		break;
	case DRM_FORMAT_ARGB1555:
		val = LCD_LAYER_FORMAT_RGBA1555 | LCD_LAYER_BGR_ORDER;
		break;
	case DRM_FORMAT_ABGR1555:
		val = LCD_LAYER_FORMAT_RGBA1555;
		break;
	case DRM_FORMAT_RGB565:
		val = LCD_LAYER_FORMAT_RGB565 | LCD_LAYER_BGR_ORDER;
		break;
	case DRM_FORMAT_BGR565:
		val = LCD_LAYER_FORMAT_RGB565;
		break;
	case DRM_FORMAT_RGB888:
		val = LCD_LAYER_FORMAT_RGB888 | LCD_LAYER_BGR_ORDER;
		break;
	case DRM_FORMAT_BGR888:
		val = LCD_LAYER_FORMAT_RGB888;
		break;
	case DRM_FORMAT_XRGB8888:
		val = LCD_LAYER_FORMAT_RGBX8888 | LCD_LAYER_BGR_ORDER;
		break;
	case DRM_FORMAT_XBGR8888:
		val = LCD_LAYER_FORMAT_RGBX8888;
		break;
	case DRM_FORMAT_ARGB8888:
		val = LCD_LAYER_FORMAT_RGBA8888 | LCD_LAYER_BGR_ORDER;
		break;
	case DRM_FORMAT_ABGR8888:
		val = LCD_LAYER_FORMAT_RGBA8888;
		break;
	}
	DRM_INFO_ONCE("%s : %d format=0x%x val=0x%x\n",
			 __func__, __LINE__, format, val);
	return val;
}

unsigned int set_bits_per_pixel(const struct drm_format_info *format)
{
	u32 bpp = 0;
	unsigned int val = 0;

	if (format->num_planes > 1) {
		val = LCD_LAYER_8BPP;
		return val;
	}

	bpp += 8 * format->cpp[0];

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

	DRM_DEBUG("bpp=%d val=0x%x\n", bpp, val);
	return val;
}

#ifdef LCD_TEST
static void config_csc(struct kmb_drm_private *dev_p, int plane_id)
{
	/* YUV to RGB conversion using the fixed matrix csc_coef_lcd */
	kmb_write_lcd(dev_p, LCD_LAYERn_CSC_COEFF11(plane_id), csc_coef_lcd[0]);
	kmb_write_lcd(dev_p, LCD_LAYERn_CSC_COEFF12(plane_id), csc_coef_lcd[1]);
	kmb_write_lcd(dev_p, LCD_LAYERn_CSC_COEFF13(plane_id), csc_coef_lcd[2]);
	kmb_write_lcd(dev_p, LCD_LAYERn_CSC_COEFF21(plane_id), csc_coef_lcd[3]);
	kmb_write_lcd(dev_p, LCD_LAYERn_CSC_COEFF22(plane_id), csc_coef_lcd[4]);
	kmb_write_lcd(dev_p, LCD_LAYERn_CSC_COEFF23(plane_id), csc_coef_lcd[5]);
	kmb_write_lcd(dev_p, LCD_LAYERn_CSC_COEFF31(plane_id), csc_coef_lcd[6]);
	kmb_write_lcd(dev_p, LCD_LAYERn_CSC_COEFF32(plane_id), csc_coef_lcd[7]);
	kmb_write_lcd(dev_p, LCD_LAYERn_CSC_COEFF33(plane_id), csc_coef_lcd[8]);
	kmb_write_lcd(dev_p, LCD_LAYERn_CSC_OFF1(plane_id), csc_coef_lcd[9]);
	kmb_write_lcd(dev_p, LCD_LAYERn_CSC_OFF2(plane_id), csc_coef_lcd[10]);
	kmb_write_lcd(dev_p, LCD_LAYERn_CSC_OFF3(plane_id), csc_coef_lcd[11]);
}
#endif

static void kmb_plane_atomic_update(struct drm_plane *plane,
				    struct drm_plane_state *state)
{
#ifdef LCD_TEST
	struct drm_framebuffer *fb;
	struct kmb_drm_private *dev_p;
	unsigned int width;
	unsigned int height;
	unsigned int dma_len;
	struct kmb_plane *kmb_plane;
	unsigned int dma_cfg;
	unsigned int ctrl = 0, val = 0, out_format = 0;
	unsigned int src_w, src_h, crtc_x, crtc_y;
	unsigned char plane_id;
	int num_planes;
	static dma_addr_t addr[MAX_SUB_PLANES] = { 0, 0, 0 };

	if (!plane || !plane->state || !state)
		return;

	fb = plane->state->fb;
	if (!fb)
		return;
	num_planes = fb->format->num_planes;
	kmb_plane = to_kmb_plane(plane);
	plane_id = kmb_plane->id;

	dev_p = plane->dev->dev_private;

	if (under_flow || flush_done) {
		DRM_DEBUG("plane_update:underflow!!!! returning");
		return;
	}

	src_w = (plane->state->src_w >> 16);
	src_h = plane->state->src_h >> 16;
	crtc_x = plane->state->crtc_x;
	crtc_y = plane->state->crtc_y;

	DRM_DEBUG("src_w=%d src_h=%d, fb->format->format=0x%x fb->flags=0x%x\n",
		  src_w, src_h, fb->format->format, fb->flags);

	width = fb->width;
	height = fb->height;
	dma_len = (width * height * fb->format->cpp[0]);
	DRM_DEBUG("%s : %d dma_len=%d ", __func__, __LINE__, dma_len);
	kmb_write_lcd(dev_p, LCD_LAYERn_DMA_LEN(plane_id), dma_len);
	kmb_write_lcd(dev_p, LCD_LAYERn_DMA_LEN_SHADOW(plane_id), dma_len);
	kmb_write_lcd(dev_p, LCD_LAYERn_DMA_LINE_VSTRIDE(plane_id),
		      fb->pitches[0]);
	kmb_write_lcd(dev_p, LCD_LAYERn_DMA_LINE_WIDTH(plane_id),
		      (width * fb->format->cpp[0]));

	addr[Y_PLANE] = drm_fb_cma_get_gem_addr(fb, plane->state, 0);
	dev_p->fb_addr = addr[Y_PLANE];
	kmb_write_lcd(dev_p, LCD_LAYERn_DMA_START_ADDR(plane_id),
		      addr[Y_PLANE] + fb->offsets[0]);
	val = set_pixel_format(fb->format->format);
	val |= set_bits_per_pixel(fb->format);
	/* Program Cb/Cr for planar formats */
	if (num_planes > 1) {
		kmb_write_lcd(dev_p, LCD_LAYERn_DMA_CB_LINE_VSTRIDE(plane_id),
				width*fb->format->cpp[0]);
		kmb_write_lcd(dev_p, LCD_LAYERn_DMA_CB_LINE_WIDTH(plane_id),
			      (width * fb->format->cpp[0]));

		addr[U_PLANE] = drm_fb_cma_get_gem_addr(fb, plane->state,
				U_PLANE);
		/* check if Cb/Cr is swapped*/
		if ((num_planes == 3) && (val & LCD_LAYER_CRCB_ORDER))
			kmb_write_lcd(dev_p,
					LCD_LAYERn_DMA_START_CR_ADR(plane_id),
					addr[U_PLANE]);
		else
			kmb_write_lcd(dev_p,
					LCD_LAYERn_DMA_START_CB_ADR(plane_id),
					addr[U_PLANE]);

		if (num_planes == 3) {
			kmb_write_lcd(dev_p,
				LCD_LAYERn_DMA_CR_LINE_VSTRIDE(plane_id),
				((width)*fb->format->cpp[0]));

			kmb_write_lcd(dev_p,
				LCD_LAYERn_DMA_CR_LINE_WIDTH(plane_id),
				((width)*fb->format->cpp[0]));

			addr[V_PLANE] = drm_fb_cma_get_gem_addr(fb,
					plane->state, V_PLANE);

			/* check if Cb/Cr is swapped*/
			if (val & LCD_LAYER_CRCB_ORDER)
				kmb_write_lcd(dev_p,
					LCD_LAYERn_DMA_START_CB_ADR(plane_id),
					addr[V_PLANE]);
			else
				kmb_write_lcd(dev_p,
					LCD_LAYERn_DMA_START_CR_ADR(plane_id),
					addr[V_PLANE]);
		}
	}

	kmb_write_lcd(dev_p, LCD_LAYERn_WIDTH(plane_id), src_w - 1);
	kmb_write_lcd(dev_p, LCD_LAYERn_HEIGHT(plane_id), src_h - 1);
	kmb_write_lcd(dev_p, LCD_LAYERn_COL_START(plane_id), crtc_x);
	kmb_write_lcd(dev_p, LCD_LAYERn_ROW_START(plane_id), crtc_y);

	/*CHECKME Leon drvr sets it to 100 try this for now */
	val |= LCD_LAYER_FIFO_100;

	if (val & LCD_LAYER_PLANAR_STORAGE) {
		val |= LCD_LAYER_CSC_EN;

		/* Enable CSC if input is planar and output is RGB */
		config_csc(dev_p, plane_id);
	}

	kmb_write_lcd(dev_p, LCD_LAYERn_CFG(plane_id), val);

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

	ctrl |= LCD_CTRL_PROGRESSIVE | LCD_CTRL_TIM_GEN_ENABLE
	    | LCD_CTRL_CONTINUOUS | LCD_CTRL_OUTPUT_ENABLED;

	/*LCD is connected to MIPI on kmb
	 * Therefore this bit is required for DSI Tx
	 */
	ctrl |= LCD_CTRL_VHSYNC_IDLE_LVL;

	kmb_set_bitmask_lcd(dev_p, LCD_CONTROL, ctrl);

	/* FIXME no doc on how to set output format,these values are
	 * taken from the Myriadx tests
	 */
	out_format |= LCD_OUTF_FORMAT_RGB888;

	/* Leave RGB order,conversion mode and clip mode to default */
	/* do not interleave RGB channels for mipi Tx compatibility */
	out_format |= LCD_OUTF_MIPI_RGB_MODE;
	kmb_write_lcd(dev_p, LCD_OUT_FORMAT_CFG, out_format);

	dma_cfg = LCD_DMA_LAYER_ENABLE | LCD_DMA_LAYER_VSTRIDE_EN |
	    LCD_DMA_LAYER_CONT_UPDATE | LCD_DMA_LAYER_AXI_BURST_16;

	/* Enable DMA */
	kmb_write_lcd(dev_p, LCD_LAYERn_DMA_CFG(plane_id), dma_cfg);
	DRM_DEBUG("dma_cfg=0x%x LCD_DMA_CFG=0x%x\n", dma_cfg,
		  kmb_read_lcd(dev_p, LCD_LAYERn_DMA_CFG(plane_id)));

	kmb_set_bitmask_lcd(dev_p, LCD_INT_CLEAR, LCD_INT_EOF |
			LCD_INT_DMA_ERR);
	kmb_set_bitmask_lcd(dev_p, LCD_INT_ENABLE, LCD_INT_EOF |
			LCD_INT_DMA_ERR);
#endif
}

static const struct drm_plane_helper_funcs kmb_plane_helper_funcs = {
	.atomic_check = kmb_plane_atomic_check,
	.atomic_update = kmb_plane_atomic_update,
	.atomic_disable = kmb_plane_atomic_disable
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
	int ret = 0;
	enum drm_plane_type plane_type;
	const uint32_t *plane_formats;
	int num_plane_formats;

	for (i = 0; i < lcd->n_layers; i++) {
		plane = devm_kzalloc(drm->dev, sizeof(*plane), GFP_KERNEL);

		if (!plane) {
			DRM_ERROR("Failed to allocate plane\n");
			return ERR_PTR(-ENOMEM);
		}

		plane_type = (i == 0) ? DRM_PLANE_TYPE_PRIMARY :
		    DRM_PLANE_TYPE_OVERLAY;
		if (i < 2) {
			plane_formats = kmb_formats_v;
			num_plane_formats = ARRAY_SIZE(kmb_formats_v);
		} else {
			plane_formats = kmb_formats_g;
			num_plane_formats = ARRAY_SIZE(kmb_formats_g);
		}

		ret = drm_universal_plane_init(drm, &plane->base_plane,
					       POSSIBLE_CRTCS, &kmb_plane_funcs,
					       plane_formats, num_plane_formats,
					       NULL, plane_type, "plane %d", i);
		if (ret < 0) {
			DRM_ERROR("drm_universal_plane_init failed (ret=%d)",
				  ret);
			goto cleanup;
		}
		DRM_DEBUG("%s : %d plane=%px\n i=%d type=%d",
			  __func__, __LINE__,
			  &plane->base_plane, i, plane_type);
		drm_plane_helper_add(&plane->base_plane,
				     &kmb_plane_helper_funcs);
		if (plane_type == DRM_PLANE_TYPE_PRIMARY) {
			primary = plane;
			lcd->plane = plane;
		}
		DRM_DEBUG("%s : %d primary=%px\n", __func__, __LINE__,
			  &primary->base_plane);
		plane->id = i;
	}

	return primary;
cleanup:
	kfree(plane);
	return ERR_PTR(ret);
}
