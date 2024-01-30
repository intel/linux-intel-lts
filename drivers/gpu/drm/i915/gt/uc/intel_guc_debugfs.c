// SPDX-License-Identifier: MIT
/*
 * Copyright © 2020 Intel Corporation
 */

#include <drm/drm_print.h>

#include "gt/intel_gt.h"
#include "gt/intel_gt_debugfs.h"
#include "gt/uc/intel_guc_ads.h"
#include "gt/uc/intel_guc_ct.h"
#include "gt/uc/intel_guc_slpc.h"
#include "gt/uc/intel_guc_submission.h"
#include "intel_guc.h"
#include "intel_guc_debugfs.h"
#include "intel_guc_log_debugfs.h"
#include "intel_runtime_pm.h"
#include "i915_drv.h"

static int guc_info_show(struct seq_file *m, void *data)
{
	struct intel_gt *gt = m->private;
	struct intel_guc *guc = &gt->uc.guc;
	struct drm_printer p = drm_seq_file_printer(m);

	if (!intel_guc_is_supported(guc))
		return -ENODEV;

	intel_guc_load_status(guc, &p);
	drm_puts(&p, "\n");

	if (!IS_SRIOV_VF(guc_to_gt(guc)->i915))
		intel_guc_log_info(&guc->log, &p);

	if (!intel_guc_submission_is_used(guc))
		return 0;

	intel_guc_ct_print_info(&guc->ct, &p);
	intel_guc_submission_print_info(guc, &p);
	intel_guc_ads_print_policy_info(guc, &p);

	return 0;
}
DEFINE_INTEL_GT_DEBUGFS_ATTRIBUTE(guc_info);

static int guc_registered_contexts_show(struct seq_file *m, void *data)
{
	struct intel_gt *gt = m->private;
	struct intel_guc *guc = &gt->uc.guc;
	struct drm_printer p = drm_seq_file_printer(m);

	if (!intel_guc_submission_is_used(guc))
		return -ENODEV;

	intel_guc_submission_print_context_info(guc, &p);

	return 0;
}
DEFINE_INTEL_GT_DEBUGFS_ATTRIBUTE(guc_registered_contexts);

static int guc_slpc_info_show(struct seq_file *m, void *unused)
{
	struct intel_gt *gt = m->private;
	struct intel_guc *guc = &gt->uc.guc;
	struct intel_guc_slpc *slpc = &guc->slpc;
	struct drm_printer p = drm_seq_file_printer(m);

	if (!intel_guc_slpc_is_used(guc))
		return -ENODEV;

	return intel_guc_slpc_print_info(slpc, &p);
}
DEFINE_INTEL_GT_DEBUGFS_ATTRIBUTE(guc_slpc_info);

static bool intel_eval_slpc_support(void *data)
{
	struct intel_gt *gt = data;
	struct intel_guc *guc = &gt->uc.guc;

	return intel_guc_slpc_is_used(guc);
}

static int guc_sched_disable_delay_ms_get(void *data, u64 *val)
{
	struct intel_gt *gt = data;
	struct intel_guc *guc = &gt->uc.guc;

	if (!intel_guc_submission_is_used(guc))
		return -ENODEV;

	*val = (u64)guc->submission_state.sched_disable_delay_ms;

	return 0;
}

static int guc_sched_disable_delay_ms_set(void *data, u64 val)
{
	struct intel_gt *gt = data;
	struct intel_guc *guc = &gt->uc.guc;

	if (!intel_guc_submission_is_used(guc))
		return -ENODEV;

	/* clamp to a practical limit, 1 minute is reasonable for a longest delay */
	guc->submission_state.sched_disable_delay_ms = min_t(u64, val, 60000);

	return 0;
}
DEFINE_I915_GT_SIMPLE_ATTRIBUTE(guc_sched_disable_delay_ms_fops,
				guc_sched_disable_delay_ms_get,
				guc_sched_disable_delay_ms_set, "%lld\n");

static int guc_sched_disable_gucid_threshold_get(void *data, u64 *val)
{
	struct intel_gt *gt = data;
	struct intel_guc *guc = &gt->uc.guc;

	if (!intel_guc_submission_is_used(guc))
		return -ENODEV;

	*val = guc->submission_state.sched_disable_gucid_threshold;
	return 0;
}

static int guc_sched_disable_gucid_threshold_set(void *data, u64 val)
{
	struct intel_gt *gt = data;
	struct intel_guc *guc = &gt->uc.guc;

	if (!intel_guc_submission_is_used(guc))
		return -ENODEV;

	if (val > intel_guc_sched_disable_gucid_threshold_max(guc))
		guc->submission_state.sched_disable_gucid_threshold =
			intel_guc_sched_disable_gucid_threshold_max(guc);
	else
		guc->submission_state.sched_disable_gucid_threshold = val;

	return 0;
}
DEFINE_I915_GT_SIMPLE_ATTRIBUTE(guc_sched_disable_gucid_threshold_fops,
				guc_sched_disable_gucid_threshold_get,
				guc_sched_disable_gucid_threshold_set, "%lld\n");

static int guc_stall_get(void *data, u64 *val)
{
	struct intel_gt *gt = data;
	struct intel_guc *guc = &gt->uc.guc;

	if (!intel_guc_submission_is_used(guc))
		return -ENODEV;

	*val = guc->stall_ms;

	return 0;
}

static int guc_stall_set(void *data, u64 val)
{
#define INTEL_GUC_STALL_MAX 60000 /* in milliseconds */
	struct intel_gt *gt = data;
	struct intel_guc *guc = &gt->uc.guc;
	enum intel_guc_scheduler_mode mode;

	if (!intel_guc_submission_is_used(guc))
		return -ENODEV;

	if (val > INTEL_GUC_STALL_MAX) {
		DRM_DEBUG_DRIVER("GuC Scheduler request delay = %lld > %d, "
				 "setting delay = %d\n",
				 val, INTEL_GUC_STALL_MAX, INTEL_GUC_STALL_MAX);
		val = INTEL_GUC_STALL_MAX;
	}
	guc->stall_ms = val;

	if (val)
		mode = INTEL_GUC_SCHEDULER_MODE_STALL_IMMEDIATE;
	else
		mode = INTEL_GUC_SCHEDULER_MODE_NORMAL;

	DRM_DEBUG_DRIVER("GuC Scheduler Stall Mode = %s (%d ms delay)\n",
			 mode == INTEL_GUC_SCHEDULER_MODE_STALL_IMMEDIATE ?
			 "Immediate" : "Normal", guc->stall_ms);

	return intel_guc_set_schedule_mode(guc, mode, val);
}
DEFINE_I915_GT_SIMPLE_ATTRIBUTE(guc_stall_fops, guc_stall_get, guc_stall_set, "%lld\n");

#if IS_ENABLED(CONFIG_DRM_I915_DEBUG_GUC)
static ssize_t guc_send_mmio_write(struct file *file, const char __user *user,
				   size_t count, loff_t *ppos)
{
	struct intel_gt *gt = file->private_data;
	struct intel_guc *guc = &gt->uc.guc;
	struct intel_runtime_pm *rpm = guc_to_gt(guc)->uncore->rpm;
	u32 request[GUC_MAX_MMIO_MSG_LEN];
	u32 response[GUC_MAX_MMIO_MSG_LEN];
	intel_wakeref_t wakeref;
	int ret;

	if (*ppos)
		return 0;

	ret = from_user_to_u32array(user, count, request, ARRAY_SIZE(request));
	if (ret < 0)
		return ret;

	with_intel_runtime_pm(rpm, wakeref)
		ret = intel_guc_send_mmio(guc, request, ret, response, ARRAY_SIZE(response));
	if (ret < 0)
		return ret;

	return count;
}

DEFINE_I915_GT_RAW_ATTRIBUTE(guc_send_mmio_fops, simple_open, NULL,
			     NULL, guc_send_mmio_write, default_llseek);

static ssize_t guc_send_ctb_write(struct file *file, const char __user *user,
				  size_t count, loff_t *ppos)
{
	struct intel_gt *gt = file->private_data;
	struct intel_guc *guc = &gt->uc.guc;
	struct intel_runtime_pm *rpm = guc_to_gt(guc)->uncore->rpm;
	u32 request[32], response[8];	/* reasonable limits */
	intel_wakeref_t wakeref;
	int ret;

	if (*ppos)
		return 0;

	ret = from_user_to_u32array(user, count, request, ARRAY_SIZE(request));
	if (ret < 0)
		return ret;

	with_intel_runtime_pm(rpm, wakeref)
		ret = intel_guc_send_and_receive(guc, request, ret, response, ARRAY_SIZE(response));
	if (ret < 0)
		return ret;

	return count;
}

DEFINE_I915_GT_RAW_ATTRIBUTE(guc_send_ctb_fops, simple_open,
			     NULL, NULL, guc_send_ctb_write, default_llseek);
#endif

void intel_guc_debugfs_register(struct intel_guc *guc, struct dentry *root)
{
	static const struct intel_gt_debugfs_file files[] = {
		{ "guc_info", &guc_info_fops, NULL },
		{ "guc_registered_contexts", &guc_registered_contexts_fops, NULL },
		{ "guc_slpc_info", &guc_slpc_info_fops, &intel_eval_slpc_support},
		{ "guc_sched_disable_delay_ms", &guc_sched_disable_delay_ms_fops, NULL },
		{ "guc_sched_disable_gucid_threshold", &guc_sched_disable_gucid_threshold_fops,
		   NULL },
		{ "guc_stall_ms", &guc_stall_fops, NULL },
#if IS_ENABLED(CONFIG_DRM_I915_DEBUG_GUC)
		{ "guc_send_mmio", &guc_send_mmio_fops, NULL },
		{ "guc_send_ctb", &guc_send_ctb_fops, NULL },
#endif
	};

	if (!intel_guc_is_supported(guc))
		return;

	intel_gt_debugfs_register_files(root, files, ARRAY_SIZE(files), guc_to_gt(guc));
	intel_guc_log_debugfs_register(&guc->log, root);
}
