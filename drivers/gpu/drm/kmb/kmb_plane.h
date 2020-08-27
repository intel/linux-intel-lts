/* SPDX-License-Identifier: MIT
 *
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
#ifndef __KMB_PLANE_H__
#define __KMB_PLANE_H__

#include "kmb_drv.h"

extern int under_flow;
extern int flush_done;

#define LCD_INT_VL0_ERR ((LAYER0_DMA_FIFO_UNDERFLOW) | \
			(LAYER0_DMA_FIFO_OVERFLOW) | \
			(LAYER0_DMA_CB_FIFO_OVERFLOW) | \
			(LAYER0_DMA_CB_FIFO_UNDERFLOW) | \
			(LAYER0_DMA_CR_FIFO_OVERFLOW) | \
			(LAYER0_DMA_CR_FIFO_UNDERFLOW))

#define LCD_INT_VL1_ERR ((LAYER1_DMA_FIFO_UNDERFLOW) | \
			(LAYER1_DMA_FIFO_OVERFLOW) | \
			(LAYER1_DMA_CB_FIFO_OVERFLOW) | \
			(LAYER1_DMA_CB_FIFO_UNDERFLOW) | \
			(LAYER1_DMA_CR_FIFO_OVERFLOW) | \
			(LAYER1_DMA_CR_FIFO_UNDERFLOW))

#define LCD_INT_GL0_ERR (LAYER2_DMA_FIFO_OVERFLOW | LAYER2_DMA_FIFO_UNDERFLOW)
#define LCD_INT_GL1_ERR (LAYER3_DMA_FIFO_OVERFLOW | LAYER3_DMA_FIFO_UNDERFLOW)
#define LCD_INT_VL0 (LAYER0_DMA_DONE | LAYER0_DMA_IDLE | LCD_INT_VL0_ERR)
#define LCD_INT_VL1 (LAYER1_DMA_DONE | LAYER1_DMA_IDLE | LCD_INT_VL1_ERR)
#define LCD_INT_GL0 (LAYER2_DMA_DONE | LAYER2_DMA_IDLE | LCD_INT_GL0_ERR)
#define LCD_INT_GL1 (LAYER3_DMA_DONE | LAYER3_DMA_IDLE | LCD_INT_GL1_ERR)
#define LCD_INT_DMA_ERR (LCD_INT_VL0_ERR | LCD_INT_VL1_ERR \
		| LCD_INT_GL0_ERR | LCD_INT_GL1_ERR)

#define POSSIBLE_CRTCS 1
#define INITIALIZED 1
#define to_kmb_plane(x) container_of(x, struct kmb_plane, base_plane)

#define to_kmb_plane_state(x) \
		container_of(x, struct kmb_plane_state, base_plane_state)

enum layer_id {
	LAYER_0,
	LAYER_1,
	LAYER_2,
	LAYER_3,
//	KMB_MAX_PLANES,
};

#define KMB_MAX_PLANES 1

enum sub_plane_id {
	Y_PLANE,
	U_PLANE,
	V_PLANE,
	MAX_SUB_PLANES,
};

struct kmb_plane {
	struct drm_plane base_plane;
	struct kmb_drm_private kmb_dev;
	unsigned char id;
};

struct kmb_plane_state {
	struct drm_plane_state base_plane_state;
	unsigned char no_planes;
};

/* Graphics layer (layers 2 & 3) formats, only packed formats  are supported */
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

/* Video layer ( 0 & 1) formats, packed and planar formats are supported */
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

/* Conversion (yuv->rgb) matrix from myriadx */
static const u32 csc_coef_lcd[] = {
	1024, 0, 1436,
	1024, -352, -731,
	1024, 1814, 0,
	-179, 125, -226
};

struct layer_status {
	bool disable;
	u32 ctrl;
};

struct kmb_plane *kmb_plane_init(struct drm_device *drm);
void kmb_plane_destroy(struct drm_plane *plane);
#endif /* __KMB_PLANE_H__ */
