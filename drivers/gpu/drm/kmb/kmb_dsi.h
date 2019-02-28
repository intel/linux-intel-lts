/* SPDX-License-Identifier: MIT
 *
 * Copyright © 2019 Intel Corporation
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
#ifndef __KMB_DSI_H__
#define __KMB_DSI_H__

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include "kmb_drv.h"

struct kmb_connector;

struct kmb_dsi {
	struct drm_encoder base;
	struct kmb_connector *attached_connector;
};

struct kmb_dsi_host {
	struct mipi_dsi_host base;
	struct kmb_dsi *kmb_dsi;
};

struct kmb_connector {
	struct drm_connector base;
	struct drm_display_mode *fixed_mode;
};

void kmb_dsi_init(struct drm_device *dev);
void kmb_plane_destroy(struct drm_plane *plane);

#define to_kmb_connector(x) container_of(x, struct kmb_connector, base)
#define to_kmb_host(x) container_of(x, struct kmb_dsi_host, base)
#define to_kmb_dsi(x) container_of(x, struct kmb_dsi, base)

#endif /* __KMB_DSI_H__ */
