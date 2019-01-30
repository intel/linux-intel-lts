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
#ifndef __KMB_DRV_H__
#define __KMB_DRV_H__

#define KMB_MAX_WIDTH			16384	/*max width in pixels */
#define KMB_MAX_HEIGHT			16384	/*max height in pixels */

struct kmb_drm_private {
	struct drm_device drm;
	void __iomem *mmio;
	unsigned char n_layers;
	struct clk *clk;
	struct drm_fbdev_cma *fbdev;
	struct drm_crtc crtc;
	struct kmb_plane *plane;
	struct drm_atomic_state *state;
};

static inline struct kmb_drm_private *to_kmb(const struct drm_device *dev)
{
	return container_of(dev, struct kmb_drm_private, drm);
}

#define crtc_to_kmb_priv(x)	container_of(x, struct kmb_drm_private, crtc)

struct blt_layer_config {
	unsigned char layer_format;
};

static inline void kmb_write(struct kmb_drm_private *lcd,
			     unsigned int reg, u32 value)
{
	writel(value, lcd->mmio + reg);
}

static inline u32 kmb_read(struct kmb_drm_private *lcd, unsigned int reg)
{
	return readl(lcd->mmio + reg);
}

int kmb_setup_crtc(struct drm_device *dev);
void kmb_set_scanout(struct kmb_drm_private *lcd);
#endif /* __KMB_DRV_H__ */
