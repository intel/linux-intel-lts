// SPDX-License-Identifier: MIT
/*
 * Copyright © 2021 Intel Corporation
 */

#include "gt/intel_gt.h"
#include "gt/intel_gpu_commands.h"
#include "gt/intel_engine_pm.h"
#include "gt/intel_ring.h"
#include "intel_gsc_fw.h"

#define GSC_FW_STATUS_REG_OFFSET		_MMIO(0x116C40)
#define GSC_FIRMWARE_CURRENT_STATE		GENMASK(3, 0)
#define   GSC_FIRMWARE_CURRENT_STATE_RESET	0
#define GSC_FIRMWARE_INIT_COMPLETE_BIT		BIT(9)

static bool gsc_is_in_reset(struct intel_uncore *uncore)
{
	u32 fw_status = intel_uncore_read(uncore, GSC_FW_STATUS_REG_OFFSET);

	return FIELD_GET(GSC_FIRMWARE_CURRENT_STATE, fw_status) ==
	       GSC_FIRMWARE_CURRENT_STATE_RESET;
}

bool intel_gsc_uc_fw_init_done(struct intel_uncore *uncore)
{
	u32 fw_status = intel_uncore_read(uncore, GSC_FW_STATUS_REG_OFFSET);

	return fw_status & GSC_FIRMWARE_INIT_COMPLETE_BIT;
}

struct gsc_heci_pkt {
	u64 addr_in;
	u32 size_in;
	u64 addr_out;
	u32 size_out;
};

static int emit_gsc_heci_pkt(struct i915_request *rq, struct gsc_heci_pkt *pkt)
{
	u32 *cs;

	cs = intel_ring_begin(rq, 8);
	if (IS_ERR(cs))
		return PTR_ERR(cs);

	*cs++ = GSC_HECI_CMD_PKT;
	*cs++ = lower_32_bits(pkt->addr_in);
	*cs++ = upper_32_bits(pkt->addr_in);
	*cs++ = pkt->size_in;
	*cs++ = lower_32_bits(pkt->addr_out);
	*cs++ = upper_32_bits(pkt->addr_out);
	*cs++ = pkt->size_out;
	*cs++ = 0;

	intel_ring_advance(rq, cs);

	return 0;
}

static int emit_gsc_fw_load(struct i915_request *rq, struct intel_gsc_uc *gsc)
{
	u32 offset = i915_ggtt_offset(gsc->local);
	u32 *cs;

	cs = intel_ring_begin(rq, 4);
	if (IS_ERR(cs))
		return PTR_ERR(cs);

	*cs++ = GSC_FW_LOAD;
	*cs++ = lower_32_bits(offset);
	*cs++ = upper_32_bits(offset);
	*cs++ = (gsc->local->size / SZ_4K) | HECI1_FW_LIMIT_VALID;

	intel_ring_advance(rq, cs);

	return 0;
}

/*
 * Our submissions to GSC are going to be either a FW load or an heci pkt, but
 * all the request emission logic is the same so we can use a common func and
 * just add the correct cmd
 */
static int submit_to_gsc_fw(struct intel_gsc_uc *gsc, struct gsc_heci_pkt *pkt)
{
	struct intel_context *ce = gsc->ce;
	struct i915_request *rq;
	int err;

	if (!ce)
		return -ENODEV;

	rq = i915_request_create(ce);
	if (IS_ERR(rq))
		return PTR_ERR(rq);

	if (ce->engine->emit_init_breadcrumb) {
		err = ce->engine->emit_init_breadcrumb(rq);
		if (err)
			goto out_rq;
	}

	/*
	 * TODO - GSC engine supports watchdog, but that must be set based on
	 * the expected duration of the command. For FW load we don't really
	 * care because if that fails we're dead anyway, but once we have more
	 * use cases we should implement it.
	 */

	if (pkt)
		err = emit_gsc_heci_pkt(rq, pkt);
	else
		err = emit_gsc_fw_load(rq, gsc);
	if (err)
		goto out_rq;

	err = ce->engine->emit_flush(rq, 0);
	if (err)
		goto out_rq;
out_rq:
	i915_request_get(rq);

	if (unlikely(err))
		i915_request_set_error_once(rq, err);

	i915_request_add(rq);

	if (!err && i915_request_wait(rq, 0, msecs_to_jiffies(500)) < 0)
		err = -ETIME;

	i915_request_put(rq);

	if (err)
		drm_err(&gsc_uc_to_gt(gsc)->i915->drm,
			"request submission for GSC failed (%d)\n",
			err);

	return err;
}

static int gsc_fw_load(struct intel_gsc_uc *gsc)
{
	return submit_to_gsc_fw(gsc, NULL);
}

int intel_gsc_fw_heci_send(struct intel_gsc_uc *gsc, u64 addr_in, u32 size_in,
			   u64 addr_out, u32 size_out)
{
	struct gsc_heci_pkt pkt = {
		.addr_in = addr_in,
		.size_in = size_in,
		.addr_out = addr_out,
		.size_out = size_out
	};

	return submit_to_gsc_fw(gsc, &pkt);
}

static int gsc_fw_load_prepare(struct intel_gsc_uc *gsc)
{
	struct intel_gt *gt = gsc_uc_to_gt(gsc);
	struct drm_i915_private *i915 = gt->i915;
	struct drm_i915_gem_object *obj;
	void *src, *dst;

	if (!gsc->local)
		return -ENODEV;

	/*
	 * TODO: submission is up, so we could use a blit here, but our blitting
	 * funcs need to be updated for MTL so we use a simple memcpy for now.
	 */

	obj = gsc->local->obj;

	if (obj->base.size < gsc->fw.size)
		return -ENOSPC;

	/*
	 * Wa_22016122933: For MTL the shared memory needs to be mapped
	 * as WC on CPU side and UC (PAT index 2) on GPU side
	 */
	if (IS_METEORLAKE(i915))
		i915_gem_object_set_cache_coherency(obj, I915_CACHE_NONE);

	dst = i915_gem_object_pin_map_unlocked(obj,
					       i915_coherent_map_type(i915, obj, true));
	if (IS_ERR(dst))
		return PTR_ERR(dst);

	src = i915_gem_object_pin_map_unlocked(gsc->fw.obj,
					       i915_coherent_map_type(i915, gsc->fw.obj, true));
	if (IS_ERR(src)) {
		i915_gem_object_unpin_map(obj);
		return PTR_ERR(src);
	}

	memset(dst, 0, obj->base.size);
	memcpy(dst, src, gsc->fw.size);

	i915_gem_object_flush_map(obj);
	i915_gem_object_unpin_map(obj);
	i915_gem_object_unpin_map(gsc->fw.obj);

	return 0;
}

static int gsc_fw_wait(struct intel_gt *gt)
{
	return intel_wait_for_register(gt->uncore,
				       GSC_FW_STATUS_REG_OFFSET,
				       GSC_FIRMWARE_INIT_COMPLETE_BIT,
				       GSC_FIRMWARE_INIT_COMPLETE_BIT,
				       500);
}

int intel_gsc_fw_upload(struct intel_gsc_uc *gsc)
{
	struct intel_gt *gt = gsc_uc_to_gt(gsc);
	struct intel_uc_fw *gsc_fw = &gsc->fw;
	int err;

	/* check current fw status */
	if (intel_gsc_uc_fw_init_done(gt->uncore)) {
		if (GEM_WARN_ON(!intel_uc_fw_is_loaded(gsc_fw)))
			intel_uc_fw_change_status(gsc_fw, INTEL_UC_FIRMWARE_TRANSFERRED);
		return 0;
	}

	if (!intel_uc_fw_is_loadable(gsc_fw))
		return -ENOEXEC;

	/* FW blob is ok, so clean the status */
	intel_gsc_uc_sanitize(gsc);

	if (!gsc_is_in_reset(gt->uncore))
		return -EIO;

	err = gsc_fw_load_prepare(gsc);
	if (err)
		goto fail;

	err = gsc_fw_load(gsc);
	if (err)
		goto fail;

	err = gsc_fw_wait(gt);
	if (err)
		goto fail;

	/* FW is not fully running until we enable SW proxy */
	intel_uc_fw_change_status(gsc_fw, INTEL_UC_FIRMWARE_TRANSFERRED);

	return 0;

fail:
	return intel_uc_fw_mark_load_failed(gsc_fw, err);
}
