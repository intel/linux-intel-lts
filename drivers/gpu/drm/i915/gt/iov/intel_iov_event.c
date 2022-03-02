// SPDX-License-Identifier: MIT
/*
 * Copyright © 2021 Intel Corporation
 */

#include "intel_iov.h"
#include "intel_iov_event.h"
#include "intel_iov_utils.h"

#define I915_UEVENT_THRESHOLD_EXCEEDED	"THRESHOLD_EXCEEDED"
#define I915_UEVENT_THRESHOLD_ID	"THRESHOLD_ID"
#define I915_UEVENT_VFID		"VF_ID"

static void pf_emit_threshold_uevent(struct intel_iov *iov, u32 vfid, u32 threshold)
{
	struct kobject *kobj = &iov_to_i915(iov)->drm.primary->kdev->kobj;
	char *envp[] = {
		I915_UEVENT_THRESHOLD_EXCEEDED"=1",
		kasprintf(GFP_KERNEL, I915_UEVENT_THRESHOLD_ID"=%#x", threshold),
		kasprintf(GFP_KERNEL, I915_UEVENT_VFID"=%u", vfid),
		NULL,
	};

	kobject_uevent_env(kobj, KOBJ_CHANGE, envp);

	kfree(envp[1]);
	kfree(envp[2]);
}

static int pf_handle_vf_threshold_event(struct intel_iov *iov, u32 vfid, u32 threshold)
{
	if (unlikely(!vfid || vfid > pf_get_totalvfs(iov)))
		return -EINVAL;

	IOV_DEBUG(iov, "VF%u threshold %04x\n", vfid, threshold);

	if (IS_ENABLED(CONFIG_DRM_I915_SELFTEST))
		pf_emit_threshold_uevent(iov, vfid, threshold);

	return 0;
}

/**
 * intel_iov_event_process_guc2pf - Handle adverse event notification from GuC.
 * @iov: the IOV struct
 * @msg: message from the GuC
 * @len: length of the message
 *
 * This function is for PF only.
 *
 * Return: 0 on success or a negative error code on failure.
 */
int intel_iov_event_process_guc2pf(struct intel_iov *iov,
				   const u32 *msg, u32 len)
{
	u32 vfid;
	u32 threshold;

	GEM_BUG_ON(!len);
	GEM_BUG_ON(FIELD_GET(GUC_HXG_MSG_0_ORIGIN, msg[0]) != GUC_HXG_ORIGIN_GUC);
	GEM_BUG_ON(FIELD_GET(GUC_HXG_MSG_0_TYPE, msg[0]) != GUC_HXG_TYPE_EVENT);
	GEM_BUG_ON(FIELD_GET(GUC_HXG_EVENT_MSG_0_ACTION, msg[0]) != GUC_ACTION_GUC2PF_ADVERSE_EVENT);

	if (unlikely(!intel_iov_is_pf(iov)))
		return -EPROTO;

	if (unlikely(FIELD_GET(GUC2PF_ADVERSE_EVENT_EVENT_MSG_0_MBZ, msg[0])))
		return -EPFNOSUPPORT;

	if (unlikely(len != GUC2PF_ADVERSE_EVENT_EVENT_MSG_LEN))
		return -EPROTO;

	vfid = FIELD_GET(GUC2PF_ADVERSE_EVENT_EVENT_MSG_1_VFID, msg[1]);
	threshold = FIELD_GET(GUC2PF_ADVERSE_EVENT_EVENT_MSG_2_THRESHOLD, msg[2]);

	return pf_handle_vf_threshold_event(iov, vfid, threshold);
}
