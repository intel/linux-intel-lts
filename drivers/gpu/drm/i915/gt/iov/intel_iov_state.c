// SPDX-License-Identifier: MIT
/*
 * Copyright © 2022 Intel Corporation
 */

#include "intel_iov.h"
#include "intel_iov_state.h"
#include "intel_iov_utils.h"
#include "gt/uc/abi/guc_actions_pf_abi.h"

static int guc_action_vf_control_cmd(struct intel_guc *guc, u32 vfid, u32 cmd)
{
	u32 request[PF2GUC_VF_CONTROL_REQUEST_MSG_LEN] = {
		FIELD_PREP(GUC_HXG_MSG_0_ORIGIN, GUC_HXG_ORIGIN_HOST) |
		FIELD_PREP(GUC_HXG_MSG_0_TYPE, GUC_HXG_TYPE_REQUEST) |
		FIELD_PREP(GUC_HXG_REQUEST_MSG_0_ACTION, GUC_ACTION_PF2GUC_VF_CONTROL),
		FIELD_PREP(PF2GUC_VF_CONTROL_REQUEST_MSG_1_VFID, vfid),
		FIELD_PREP(PF2GUC_VF_CONTROL_REQUEST_MSG_2_COMMAND, cmd),
	};

	return intel_guc_send(guc, request, ARRAY_SIZE(request));
}

static int pf_control_vf(struct intel_iov *iov, u32 vfid, u32 cmd)
{
	struct intel_runtime_pm *rpm = iov_to_gt(iov)->uncore->rpm;
	intel_wakeref_t wakeref;
	int err = -ENONET;

	GEM_BUG_ON(!intel_iov_is_pf(iov));
	GEM_BUG_ON(vfid > pf_get_totalvfs(iov));
	GEM_BUG_ON(!vfid);

	with_intel_runtime_pm(rpm, wakeref)
		err = guc_action_vf_control_cmd(iov_to_guc(iov), vfid, cmd);

	return err;
}

static void pf_trigger_vf_flr_start(struct intel_iov *iov, u32 vfid)
{
	int ret;

	ret = pf_control_vf(iov, vfid, GUC_PF_TRIGGER_VF_FLR_START);
	if (unlikely(ret < 0))
		IOV_ERROR(iov, "Failed to start FLR for VF%u (%pe)\n",
			  vfid, ERR_PTR(ret));
}

static void pf_confirm_vf_flr_done(struct intel_iov *iov, u32 vfid)
{
	int ret;

	ret = pf_control_vf(iov, vfid, GUC_PF_TRIGGER_VF_FLR_FINISH);
	if (unlikely(ret < 0))
		IOV_ERROR(iov, "Failed to confirm FLR for VF%u (%pe)\n",
			  vfid, ERR_PTR(ret));
}

static bool pf_vfs_flr_enabled(struct intel_iov *iov, u32 vfid)
{
	return iov_to_i915(iov)->params.vfs_flr_mask & BIT(vfid);
}

static void pf_handle_vf_flr(struct intel_iov *iov, u32 vfid)
{
	struct device *dev = iov_to_dev(iov);

	dev_info(dev, "VF%u FLR\n", vfid);
	pf_trigger_vf_flr_start(iov, vfid);
}

static void pf_clear_vf_ggtt_entries(struct intel_iov *iov, u32 vfid)
{
	struct intel_iov_config *config = &iov->pf.provisioning.configs[vfid];
	struct intel_gt *gt = iov_to_gt(iov);

	GEM_BUG_ON(vfid > pf_get_totalvfs(iov));

	if (!drm_mm_node_allocated(&config->ggtt_region))
		return;

	i915_ggtt_set_space_owner(gt->ggtt, vfid, &config->ggtt_region);
}

static void pf_handle_vf_flr_done(struct intel_iov *iov, u32 vfid)
{
	if (!pf_vfs_flr_enabled(iov, vfid)) {
		IOV_DEBUG(iov, "VF%u FLR processing skipped\n", vfid);
		goto confirm;
	}

	IOV_DEBUG(iov, "processing VF%u FLR\n", vfid);

	pf_clear_vf_ggtt_entries(iov, vfid);

confirm:
	pf_confirm_vf_flr_done(iov, vfid);
}

static void pf_handle_vf_pause_done(struct intel_iov *iov, u32 vfid)
{
	struct device *dev = iov_to_dev(iov);

	dev_info(dev, "VF%u %s\n", vfid, "paused");
}

static void pf_handle_vf_fixup_done(struct intel_iov *iov, u32 vfid)
{
	struct device *dev = iov_to_dev(iov);

	dev_info(dev, "VF%u %s\n", vfid, "has completed migration");
}

static int pf_handle_vf_event(struct intel_iov *iov, u32 vfid, u32 eventid)
{
	switch (eventid) {
	case GUC_PF_NOTIFY_VF_FLR:
		pf_handle_vf_flr(iov, vfid);
		break;
	case GUC_PF_NOTIFY_VF_FLR_DONE:
		pf_handle_vf_flr_done(iov, vfid);
		break;
	case GUC_PF_NOTIFY_VF_PAUSE_DONE:
		pf_handle_vf_pause_done(iov, vfid);
		break;
	case GUC_PF_NOTIFY_VF_FIXUP_DONE:
		pf_handle_vf_fixup_done(iov, vfid);
		break;
	default:
		return -ENOPKG;
	}

	return 0;
}

static int pf_handle_pf_event(struct intel_iov *iov, u32 eventid)
{
	switch (eventid) {
	case GUC_PF_NOTIFY_VF_ENABLE:
		IOV_DEBUG(iov, "VFs %s/%s\n", enableddisabled(true), enableddisabled(false));
		break;
	default:
		return -ENOPKG;
	}

	return 0;
}

/**
 * intel_iov_state_process_guc2pf - Handle VF state notification from GuC.
 * @iov: the IOV struct
 * @msg: message from the GuC
 * @len: length of the message
 *
 * This function is for PF only.
 *
 * Return: 0 on success or a negative error code on failure.
 */
int intel_iov_state_process_guc2pf(struct intel_iov *iov,
				   const u32 *msg, u32 len)
{
	u32 vfid;
	u32 eventid;

	GEM_BUG_ON(!len);
	GEM_BUG_ON(FIELD_GET(GUC_HXG_MSG_0_ORIGIN, msg[0]) != GUC_HXG_ORIGIN_GUC);
	GEM_BUG_ON(FIELD_GET(GUC_HXG_MSG_0_TYPE, msg[0]) != GUC_HXG_TYPE_EVENT);
	GEM_BUG_ON(FIELD_GET(GUC_HXG_EVENT_MSG_0_ACTION, msg[0]) != GUC_ACTION_GUC2PF_VF_STATE_NOTIFY);

	if (unlikely(!intel_iov_is_pf(iov)))
		return -EPROTO;

	if (unlikely(FIELD_GET(GUC2PF_VF_STATE_NOTIFY_EVENT_MSG_0_MBZ, msg[0])))
		return -EPFNOSUPPORT;

	if (unlikely(len != GUC2PF_VF_STATE_NOTIFY_EVENT_MSG_LEN))
		return -EPROTO;

	vfid = FIELD_GET(GUC2PF_VF_STATE_NOTIFY_EVENT_MSG_1_VFID, msg[1]);
	eventid = FIELD_GET(GUC2PF_VF_STATE_NOTIFY_EVENT_MSG_2_EVENT, msg[2]);

	if (unlikely(vfid > pf_get_totalvfs(iov)))
		return -EINVAL;

	return vfid ? pf_handle_vf_event(iov, vfid, eventid) : pf_handle_pf_event(iov, eventid);
}

/**
 * intel_iov_state_pause_vf - Pause VF.
 * @iov: the IOV struct
 * @vfid: VF identifier
 *
 * This function is for PF only.
 *
 * Return: 0 on success or a negative error code on failure.
 */
int intel_iov_state_pause_vf(struct intel_iov *iov, u32 vfid)
{
	return pf_control_vf(iov, vfid, GUC_PF_TRIGGER_VF_PAUSE);
}

/**
 * intel_iov_state_resume_vf - Resume VF.
 * @iov: the IOV struct
 * @vfid: VF identifier
 *
 * This function is for PF only.
 *
 * Return: 0 on success or a negative error code on failure.
 */
int intel_iov_state_resume_vf(struct intel_iov *iov, u32 vfid)
{
	return pf_control_vf(iov, vfid, GUC_PF_TRIGGER_VF_RESUME);
}

/**
 * intel_iov_state_stop_vf - Stop VF.
 * @iov: the IOV struct
 * @vfid: VF identifier
 *
 * This function is for PF only.
 *
 * Return: 0 on success or a negative error code on failure.
 */
int intel_iov_state_stop_vf(struct intel_iov *iov, u32 vfid)
{
	return pf_control_vf(iov, vfid, GUC_PF_TRIGGER_VF_STOP);
}

static int guc_action_save_restore_vf(struct intel_guc *guc, u32 vfid, u32 opcode, u64 offset)
{
	u32 request[PF2GUC_SAVE_RESTORE_VF_REQUEST_MSG_LEN] = {
		FIELD_PREP(GUC_HXG_MSG_0_ORIGIN, GUC_HXG_ORIGIN_HOST) |
		FIELD_PREP(GUC_HXG_MSG_0_TYPE, GUC_HXG_TYPE_REQUEST) |
		FIELD_PREP(GUC_HXG_REQUEST_MSG_0_ACTION, GUC_ACTION_PF2GUC_SAVE_RESTORE_VF) |
		FIELD_PREP(PF2GUC_SAVE_RESTORE_VF_REQUEST_MSG_0_OPCODE, opcode),
		FIELD_PREP(PF2GUC_SAVE_RESTORE_VF_REQUEST_MSG_1_VFID, vfid),
		FIELD_PREP(PF2GUC_SAVE_RESTORE_VF_REQUEST_MSG_2_BUFF_LO, lower_32_bits(offset)),
		FIELD_PREP(PF2GUC_SAVE_RESTORE_VF_REQUEST_MSG_3_BUFF_HI, upper_32_bits(offset)),
	};
	int ret;

	ret = intel_guc_send(guc, request, ARRAY_SIZE(request));

	return ret > SZ_4K ? -EPROTO : ret;
}

static int pf_save_vf(struct intel_iov *iov, u32 vfid, void *buf)
{
	struct intel_guc *guc = iov_to_guc(iov);
	struct i915_vma *vma;
	void *blob;
	int ret;

	GEM_BUG_ON(!intel_iov_is_pf(iov));
	GEM_BUG_ON(vfid > pf_get_totalvfs(iov));
	GEM_BUG_ON(!vfid);

	ret = intel_guc_allocate_and_map_vma(guc, SZ_4K, &vma, (void **)&blob);
	if (unlikely(ret))
		goto failed;

	ret = guc_action_save_restore_vf(guc, vfid, GUC_PF_OPCODE_VF_SAVE,
					 intel_guc_ggtt_offset(guc, vma));

	if (likely(ret > 0)) {
		memcpy(buf, blob, SZ_4K);

		if (IS_ENABLED(CONFIG_DRM_I915_SELFTEST) &&
		    memchr_inv(buf + ret, 0, SZ_4K - ret)) {
			pr_err("non-zero state found beyond offset %d!\n", ret);
		}
	}

	i915_vma_unpin_and_release(&vma, I915_VMA_RELEASE_MAP);

	if (unlikely(ret < 0))
		goto failed;

	IOV_DEBUG(iov, "VF%u: state saved (%d bytes) %*ph ..\n",
		  vfid, ret, min_t(int, 16, ret), buf);
	return 0;

failed:
	IOV_ERROR(iov, "Failed to save VF%u state (%pe)\n", vfid, ERR_PTR(ret));
	return ret;
}

/**
 * intel_iov_state_save_vf - Save VF state.
 * @iov: the IOV struct
 * @vfid: VF identifier
 * @buf: buffer to save VF state (must be at least 4K)
 *
 * This function is for PF only.
 *
 * Return: 0 on success or a negative error code on failure.
 */
int intel_iov_state_save_vf(struct intel_iov *iov, u32 vfid, void *buf)
{
	struct intel_runtime_pm *rpm = iov_to_gt(iov)->uncore->rpm;
	intel_wakeref_t wakeref;
	int err = -ENONET;

	with_intel_runtime_pm(rpm, wakeref)
		err = pf_save_vf(iov, vfid, buf);

	return err;
}

static int pf_restore_vf(struct intel_iov *iov, u32 vfid, const void *buf)
{
	struct intel_guc *guc = iov_to_guc(iov);
	struct i915_vma *vma;
	void *blob;
	int ret;

	GEM_BUG_ON(!intel_iov_is_pf(iov));
	GEM_BUG_ON(vfid > pf_get_totalvfs(iov));
	GEM_BUG_ON(!vfid);

	ret = intel_guc_allocate_and_map_vma(guc, SZ_4K, &vma, (void **)&blob);
	if (unlikely(ret < 0))
		goto failed;

	memcpy(blob, buf, SZ_4K);

	ret = guc_action_save_restore_vf(guc, vfid, GUC_PF_OPCODE_VF_RESTORE,
					 intel_guc_ggtt_offset(guc, vma));

	i915_vma_unpin_and_release(&vma, I915_VMA_RELEASE_MAP);

	if (unlikely(ret < 0))
		goto failed;

	IOV_DEBUG(iov, "VF%u: state restored (%u bytes) %*ph\n",
		  vfid, ret, min_t(int, 16, ret), buf);
	return 0;

failed:
	IOV_ERROR(iov, "Failed to restore VF%u state (%pe) %*ph\n",
		  vfid, ERR_PTR(ret), 16, buf);
	return ret;
}

/**
 * intel_iov_state_restore_vf - Restore VF state.
 * @iov: the IOV struct
 * @vfid: VF identifier
 * @buf: buffer with VF state to restore (must be 4K)
 *
 * This function is for PF only.
 *
 * Return: 0 on success or a negative error code on failure.
 */
int intel_iov_state_restore_vf(struct intel_iov *iov, u32 vfid, const void *buf)
{
	struct intel_runtime_pm *rpm = iov_to_gt(iov)->uncore->rpm;
	intel_wakeref_t wakeref;
	int err = -ENONET;

	with_intel_runtime_pm(rpm, wakeref)
		err = pf_restore_vf(iov, vfid, buf);

	return err;
}
