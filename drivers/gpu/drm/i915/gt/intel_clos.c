// SPDX-License-Identifier: MIT
/*
 * Copyright © 2020 Intel Corporation
 */

#include "i915_drv.h"

#include "intel_gt.h"
#include "intel_gt_mcr.h"
#include "intel_gt_regs.h"
#include "intel_clos.h"

static void clos_update_ways(struct intel_gt *gt, u8 clos_index, u32 mask)
{
	intel_wakeref_t wakeref;

	DRM_DEBUG("clos index = %d mask = 0x%x", clos_index,  mask);
	wakeref = intel_runtime_pm_get(gt->uncore->rpm);
	intel_gt_mcr_multicast_write(gt, XEHPC_L3CLOS_MASK(clos_index), mask);
	intel_runtime_pm_put(gt->uncore->rpm, wakeref);
}

static void update_l3cache_masks(struct drm_i915_private *dev_priv)
{
	u8 start_bits = 0;
	int i;

	for (i = 0; i < NUM_CLOS; i++) {
		struct intel_gt *gt;
		u32 mask = 0;
		int j;

		if (dev_priv->cache_resv.ways[i]) {

			// Assign contiguous span of ways
			u8 ways = dev_priv->cache_resv.ways[i];
			mask = GENMASK(start_bits + ways - 1, start_bits);
			DRM_DEBUG("start_bits = %d ways = %d mask= 0x%x\n",
					start_bits, ways, mask);
			start_bits += ways;
		}
		for_each_gt(gt, dev_priv, j)
			clos_update_ways(to_gt(dev_priv), i, mask);
	}
}

#define MAX_L3WAYS 32
void init_device_clos(struct drm_i915_private *dev_priv)
{
	int i;

	if (!HAS_CACHE_CLOS(dev_priv))
		return;

	mutex_init(&dev_priv->cache_resv.clos_mutex);
	// CLOS1 and CLOS2 available for Reservation
	dev_priv->cache_resv.free_clos_mask = 0x6;

	// Shared set uses CLOS0 and initially gets all Ways
	dev_priv->cache_resv.ways[0] = MAX_L3WAYS;

	for (i = 1; i < 3; i++)
		dev_priv->cache_resv.ways[i] = 0;

	update_l3cache_masks(dev_priv);
}

void uninit_device_clos(struct drm_i915_private *dev_priv)
{
	if (!HAS_CACHE_CLOS(dev_priv))
		return;

	mutex_destroy(&dev_priv->cache_resv.clos_mutex);
}

void init_client_clos(struct drm_i915_file_private *fpriv)
{
	if (!HAS_CACHE_CLOS(fpriv->dev_priv))
		return;

	fpriv->clos_resv.clos_mask = 0;   // No CLOS reserved yet
	fpriv->clos_resv.l3_rsvd_ways = 0;
}

void uninit_client_clos(struct drm_i915_file_private *fpriv)
{
	if (!HAS_CACHE_CLOS(fpriv->dev_priv))
		return;

	while (fpriv->clos_resv.clos_mask) {
		u16 clos_index = ffs(fpriv->clos_resv.clos_mask) - 1;

		DRM_DEBUG("uninit release mask = 0x%x clos= %d\n",
			fpriv->clos_resv.clos_mask, clos_index);
		free_clos(fpriv, clos_index);
		fpriv->clos_resv.clos_mask &= ~(1 << clos_index);
	}
}

#define L3_GLOBAL_RESERVATION_LIMIT 16
#define L3_CLIENT_RESERVATION_LIMIT 8
static int reserve_l3cache_ways(struct drm_i915_file_private *fpriv,
				u16 clos_index, u16 *num_ways)
{
	struct drm_i915_private *dev_priv = fpriv->dev_priv;
	u8 global_limit = L3_GLOBAL_RESERVATION_LIMIT -
		(MAX_L3WAYS - dev_priv->cache_resv.ways[0]);
	u8 client_limit = L3_CLIENT_RESERVATION_LIMIT -
		fpriv->clos_resv.l3_rsvd_ways;
	u8 limit = min(global_limit, client_limit);

	if (limit == 0)
		return -ENOSPC;

	if (*num_ways > limit) {
		*num_ways = limit;
		return -EAGAIN;
	}

	fpriv->clos_resv.l3_rsvd_ways += *num_ways;

	dev_priv->cache_resv.ways[0] -= *num_ways;
	dev_priv->cache_resv.ways[clos_index] = *num_ways;

	update_l3cache_masks(dev_priv);

	return 0;
}

static int
free_l3cache_ways(struct drm_i915_file_private *fpriv, u16 clos_index)
{
	struct drm_i915_private *dev_priv = fpriv->dev_priv;

	if (dev_priv->cache_resv.ways[clos_index]) {
		u8 num_ways = dev_priv->cache_resv.ways[clos_index];

		fpriv->clos_resv.l3_rsvd_ways -= num_ways;

		dev_priv->cache_resv.ways[0] += num_ways;
		dev_priv->cache_resv.ways[clos_index] -= num_ways;

		update_l3cache_masks(dev_priv);
	}

	return 0;
}

static bool
clos_is_reserved(struct drm_i915_file_private *fpriv, u16 clos_index)
{
	return fpriv->clos_resv.clos_mask & (1 << clos_index);
}

int reserve_cache_ways(struct drm_i915_file_private *fpriv, u16 cache_level,
		       u16 clos_index, u16 *num_ways)
{
	struct drm_i915_private *dev_priv = fpriv->dev_priv;
	int ret = 0;

	if (cache_level != 3)
		return -EINVAL;

	if ((clos_index >= NUM_CLOS) || !clos_is_reserved(fpriv, clos_index))
		return -EPERM;

	mutex_lock(&dev_priv->cache_resv.clos_mutex);

	if(*num_ways)
		ret = reserve_l3cache_ways(fpriv, clos_index, num_ways);
	else
		ret = free_l3cache_ways(fpriv, clos_index);

	mutex_unlock(&dev_priv->cache_resv.clos_mutex);
	return ret;
}

int reserve_clos(struct drm_i915_file_private *fpriv, u16 *clos_index)
{
	struct drm_i915_private *dev_priv = fpriv->dev_priv;

	mutex_lock(&dev_priv->cache_resv.clos_mutex);

	if (dev_priv->cache_resv.free_clos_mask) {
		u16 clos = ffs(dev_priv->cache_resv.free_clos_mask) - 1;

		fpriv->clos_resv.clos_mask |= (1 << clos);
		dev_priv->cache_resv.free_clos_mask &= ~(1 << clos);

		*clos_index = clos;
		mutex_unlock(&dev_priv->cache_resv.clos_mutex);

		return 0;
	}
	mutex_unlock(&dev_priv->cache_resv.clos_mutex);

	return -ENOSPC;
}

int free_clos(struct drm_i915_file_private *fpriv, u16 clos_index)
{
	struct drm_i915_private *dev_priv = fpriv->dev_priv;

	mutex_lock(&dev_priv->cache_resv.clos_mutex);

	if (clos_is_reserved(fpriv, clos_index)) {
		struct drm_i915_private *dev_priv = fpriv->dev_priv;

		free_l3cache_ways(fpriv, clos_index);

		fpriv->clos_resv.clos_mask &= ~(1 << clos_index);
		dev_priv->cache_resv.free_clos_mask |= (1 << clos_index);

		mutex_unlock(&dev_priv->cache_resv.clos_mutex);

		return 0;
	}

	mutex_unlock(&dev_priv->cache_resv.clos_mutex);
	return -EPERM;
}
