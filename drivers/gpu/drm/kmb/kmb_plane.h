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

enum layer_id {
	LAYER_0,
	LAYER_1,
	LAYER_2,
	LAYER_3,
	KMB_MAX_PLANES,
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

#define POSSIBLE_CRTCS 1
#define to_kmb_plane(x) container_of(x, struct kmb_plane, base_plane)

#define to_kmb_plane_state(x) \
		container_of(x, struct kmb_plane_state, base_plane_state)

struct kmb_plane *kmb_plane_init(struct drm_device *drm);
void kmb_plane_destroy(struct drm_plane *plane);
#endif /* __KMB_PLANE_H__ */
