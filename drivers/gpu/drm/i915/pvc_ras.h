/* SPDX-License-Identifier: MIT
 * Copyright © 2023 Intel Corporation
 */
#ifndef _PVC_RAS_H
#define _PVC_RAS_H

#include "i915_drv.h"

#define DEFAULT_VALUE_RAS_REG64 0
#define HBM_STACK_MAX		4
#define CHANNEL_MAX		8

int pvc_ras_telemetry_probe(struct drm_i915_private *i915);
#endif
