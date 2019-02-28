// SPDX-License-Identifier: GPL-2.0
/*
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
#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_edid.h>
#include <drm/drm_mipi_dsi.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include "kmb_drv.h"
#include "kmb_dsi.h"

static enum drm_mode_status
kmb_dsi_mode_valid(struct drm_connector *connector,
		   struct drm_display_mode *mode)
{
	return MODE_OK;
}

static int kmb_dsi_get_modes(struct drm_connector *connector)
{
	struct drm_display_mode *mode;
	struct kmb_connector *kmb_connector = to_kmb_connector(connector);

	mode = drm_mode_duplicate(connector->dev, kmb_connector->fixed_mode);
	drm_mode_probed_add(connector, mode);
	return 1;
}

static void kmb_dsi_connector_destroy(struct drm_connector *connector)
{
	struct kmb_connector *kmb_connector = to_kmb_connector(connector);

	drm_connector_cleanup(connector);
	kfree(kmb_connector);
}

static void kmb_dsi_encoder_destroy(struct drm_encoder *encoder)
{
	struct kmb_dsi *kmb_dsi = to_kmb_dsi(encoder);

	drm_encoder_cleanup(encoder);
	kfree(kmb_dsi);
}

static const struct drm_encoder_funcs kmb_dsi_funcs = {
	.destroy = kmb_dsi_encoder_destroy,
};

static const struct
drm_connector_helper_funcs kmb_dsi_connector_helper_funcs = {
	.get_modes = kmb_dsi_get_modes,
	.mode_valid = kmb_dsi_mode_valid,
};

static const struct drm_connector_funcs kmb_dsi_connector_funcs = {
	.destroy = kmb_dsi_connector_destroy,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
};

void kmb_dsi_init(struct drm_device *dev)
{
	struct kmb_dsi *kmb_dsi;
	struct drm_encoder *encoder;
	struct kmb_connector *kmb_connector;
	struct drm_connector *connector;

	kmb_dsi = kzalloc(sizeof(*kmb_dsi), GFP_KERNEL);
	if (!kmb_dsi)
		return;

	kmb_connector = kzalloc(sizeof(*kmb_connector), GFP_KERNEL);
	if (!kmb_connector) {
		kfree(kmb_dsi);
		return;
	}

	kmb_dsi->attached_connector = kmb_connector;

	connector = &kmb_connector->base;
	encoder = &kmb_dsi->base;
	drm_encoder_init(dev, encoder, &kmb_dsi_funcs, DRM_MODE_ENCODER_DSI,
			 "MIPI-DSI");
	drm_connector_init(dev, connector, &kmb_dsi_connector_funcs,
			   DRM_MODE_CONNECTOR_DSI);
	drm_connector_helper_add(connector, &kmb_dsi_connector_helper_funcs);
}
