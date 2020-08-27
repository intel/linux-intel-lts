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
#ifndef __KMB_CRTC_H__
#define __KMB_CRTC_H__

#include <linux/mutex.h>
#include <linux/wait.h>
#include <drm/drmP.h>
#include "kmb_drv.h"

#define to_kmb_crtc_state(x) container_of(x, struct kmb_crtc_state, crtc_base)
#define to_kmb_crtc(x) container_of(x, struct kmb_crtc, crtc_base)

struct kmb_crtc {
	struct drm_crtc crtc_base;
	struct kmb_drm_private kmb_dev;
};

struct kmb_crtc_state {
	struct drm_crtc_state crtc_base;
};

extern void kmb_plane_destroy(struct drm_plane *plane);
extern struct kmb_plane *kmb_plane_init(struct drm_device *drm);
#endif /* __KMB_CRTC_H__ */
