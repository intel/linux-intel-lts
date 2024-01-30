/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2021 Intel Corporation
 */

#ifndef __INTEL_HDCP_GSC_H__
#define __INTEL_HDCP_GSC_H__

#include <linux/types.h>
#include <linux/err.h>
/*
 * FIXME: Spec states that we need to create a random
 * host session everytime we send message for now creating
 * a static host session to avoid clashes not using this
 * header as of now as we see an error if we use anything
 * other than 0 as host session
 */
#define GSC_HDCP_HOST_HANDLE	0x12233FFEEDD00000

struct drm_i915_private;

ssize_t intel_hdcp_gsc_msg_send(struct drm_i915_private *i915, u8 *msg_in,
	size_t msg_in_len, u8 *msg_out, size_t msg_out_len);
int intel_gsc_hdcp_init(struct drm_i915_private *i915);
int intel_gsc_hdcp_fini(struct drm_i915_private *i915);

#endif /* __INTEL_HDCP_GCS_H__ */
