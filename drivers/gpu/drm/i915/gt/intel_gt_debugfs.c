// SPDX-License-Identifier: MIT
/*
 * Copyright © 2019 Intel Corporation
 */

#include <linux/debugfs.h>

#include "i915_drv.h"
#include "intel_gt.h"
#include "intel_gt_ccs_mode.h"
#include "intel_gt_debugfs.h"
#include "intel_gt_engines_debugfs.h"
#include "intel_gt_mcr.h"
#include "intel_gt_pm_debugfs.h"
#include "intel_sseu_debugfs.h"
#include "iov/intel_iov_debugfs.h"
#include "pxp/intel_pxp_debugfs.h"
#include "uc/intel_uc_debugfs.h"

int intel_gt_debugfs_reset_show(struct intel_gt *gt, u64 *val)
{
	int ret = intel_gt_terminally_wedged(gt);

	switch (ret) {
	case -EIO:
		*val = 1;
		return 0;
	case 0:
		*val = 0;
		return 0;
	default:
		return ret;
	}
}

void intel_gt_debugfs_reset_store(struct intel_gt *gt, u64 val)
{
	/* Flush any previous reset before applying for a new one */
	wait_event(gt->reset.queue,
		   !test_bit(I915_RESET_BACKOFF, &gt->reset.flags));

	intel_gt_handle_error(gt, val, I915_ERROR_CAPTURE,
			      "Manually reset engine mask to %llx", val);
}

/*
 * keep the interface clean where the first parameter
 * is a 'struct intel_gt *' instead of 'void *'
 */
static int __intel_gt_debugfs_reset_show(void *data, u64 *val)
{
	return intel_gt_debugfs_reset_show(data, val);
}

static int __intel_gt_debugfs_reset_store(void *data, u64 val)
{
	intel_gt_debugfs_reset_store(data, val);

	return 0;
}

DEFINE_I915_GT_SIMPLE_ATTRIBUTE(reset_fops, __intel_gt_debugfs_reset_show,
				__intel_gt_debugfs_reset_store, "%llu\n");

static int steering_show(struct seq_file *m, void *data)
{
	struct drm_printer p = drm_seq_file_printer(m);
	struct intel_gt *gt = m->private;

	intel_gt_mcr_report_steering(&p, gt, true);

	return 0;
}
DEFINE_INTEL_GT_DEBUGFS_ATTRIBUTE(steering);

static int fake_int_slow_get(void *data, u64 *val)
{
	struct intel_gt *gt = data;

	if (!gt->fake_int.enabled)
		return -ENODEV;

	*val = gt->fake_int.delay_slow;

	return 0;
}

static int fake_int_slow_set(void *data, u64 val)
{
	struct intel_gt *gt = data;

	if (!gt->fake_int.enabled)
		return -ENODEV;

	gt->fake_int.delay_slow = val;

	return 0;
}

DEFINE_I915_GT_SIMPLE_ATTRIBUTE(fake_int_slow_fops, fake_int_slow_get, fake_int_slow_set, "%llu\n");

static int fake_int_fast_get(void *data, u64 *val)
{
	struct intel_gt *gt = data;

	if (!gt->fake_int.enabled)
		return -ENODEV;

	*val = gt->fake_int.delay_fast;

	return 0;
}

static int fake_int_fast_set(void *data, u64 val)
{
	struct intel_gt *gt = data;

	if (!gt->fake_int.enabled)
		return -ENODEV;

	gt->fake_int.delay_fast = val;

	return 0;
}

DEFINE_I915_GT_SIMPLE_ATTRIBUTE(fake_int_fast_fops, fake_int_fast_get, fake_int_fast_set, "%llu\n");

static int debug_pages_show(struct seq_file *m, void *data)
{
	struct intel_gt *gt = m->private;

	if (gt->dbg) {
		u32 *vaddr;
		int i;
		seq_printf(m, "debug pages allocated in %s: "
				"ggtt=0x%08x, phys=0x%016llx, size=0x%zx\n\n",
				gt->dbg->obj->mm.region.mem->name,
				i915_ggtt_offset(gt->dbg),
				(u64)i915_gem_object_get_dma_address(gt->dbg->obj, 0),
				gt->dbg->obj->base.size);

		vaddr = i915_gem_object_pin_map_unlocked(gt->dbg->obj, I915_MAP_WC);
		if (!vaddr)
			return -ENOSPC;

		for (i = 0; i < (gt->dbg->obj->base.size / sizeof(u32)); i += 4)
			seq_printf(m, "[0x%08x] 0x%08x 0x%08x 0x%08x 0x%08x\n",
				   i * 4, vaddr[i], vaddr[i + 1], vaddr[i + 2], vaddr[i + 3]);

		i915_gem_object_unpin_map(gt->dbg->obj);
	}

	return 0;
}
DEFINE_INTEL_GT_DEBUGFS_ATTRIBUTE(debug_pages);

int i915_gt_debugfs_single_open(struct file *file, int (*show)(struct seq_file *, void *),
				void *data)
{
	struct intel_gt *gt = data;
	struct drm_i915_private *i915 = gt->i915;
	int ret;

	ret = single_open(file, show, data);
	if (!ret)
		pvc_wa_disallow_rc6(i915);

	return ret;
}

int i915_gt_debugfs_single_open_size(struct file *file, int (*show)(struct seq_file *, void *),
				     void *data, size_t size)
{
	struct intel_gt *gt = data;
	struct drm_i915_private *i915 = gt->i915;
	int ret;

	ret = single_open_size(file, show, data, size);
	if (!ret)
		pvc_wa_disallow_rc6(i915);

	return ret;
}

int i915_gt_debugfs_single_release(struct inode *inode, struct file *file)
{
	struct intel_gt *gt = inode->i_private;
	struct drm_i915_private *i915 = gt->i915;

	pvc_wa_allow_rc6(i915);
	return single_release(inode, file);
}

int i915_gt_debugfs_raw_attr_open(struct inode *inode, struct file *file,
				  int (*open)(struct inode*, struct file*))
{
	struct intel_gt *gt = inode->i_private;
	struct drm_i915_private *i915 = gt->i915;
	int ret = 0;

	pvc_wa_disallow_rc6(i915);
	if (open)
		ret = open(inode, file);

	if (ret)
		pvc_wa_allow_rc6(i915);

	return ret;
}

int i915_gt_debugfs_raw_attr_close(struct inode *inode, struct file *file,
				   int (*close)(struct inode*, struct file*))
{
	struct intel_gt *gt = inode->i_private;
	struct drm_i915_private *i915 = gt->i915;
	int ret = 0;

	if (close)
		ret = close(inode, file);

	pvc_wa_allow_rc6(i915);

	return ret;
}

int i915_gt_debugfs_simple_attr_open(struct inode *inode, struct file *file,
				     int (*get)(void *, u64 *), int (*set)(void *, u64),
				     const char *fmt)
{
	struct intel_gt *gt = inode->i_private;
	struct drm_i915_private *i915 = gt->i915;
	int ret;

	ret = simple_attr_open(inode, file, get, set, fmt);
	if (!ret)
		pvc_wa_disallow_rc6(i915);

	return ret;
}

int i915_gt_debugfs_simple_attr_release(struct inode *inode, struct file *file)
{
	struct intel_gt *gt = inode->i_private;
	struct drm_i915_private *i915 = gt->i915;
	int ret = 0;

	ret = simple_attr_release(inode, file);
	pvc_wa_allow_rc6(i915);

	return ret;
}

static int name_show(struct seq_file *m, void *data)
{
	struct drm_printer p = drm_seq_file_printer(m);
	struct intel_gt *gt = m->private;

	drm_printf(&p, "%s\n", gt->name);

	return 0;
}
DEFINE_INTEL_GT_DEBUGFS_ATTRIBUTE(name);

static void gt_debugfs_register(struct intel_gt *gt, struct dentry *root)
{
	static const struct intel_gt_debugfs_file files[] = {
		{ "reset", &reset_fops, NULL },
		{ "steering", &steering_fops },
		{ "fake_int_slow_ns", &fake_int_slow_fops, NULL },
		{ "fake_int_fast_ns", &fake_int_fast_fops, NULL },
		{ "debug_pages", &debug_pages_fops, NULL },
		{ "name", &name_fops },
	};

	intel_gt_debugfs_register_files(root, files, ARRAY_SIZE(files), gt);
}

void intel_gt_debugfs_register(struct intel_gt *gt)
{
	struct dentry *root;
	char gtname[4];

	if (!gt->i915->drm.primary->debugfs_root)
		return;

	snprintf(gtname, sizeof(gtname), "gt%u", gt->info.id);
	root = debugfs_create_dir(gtname, gt->i915->drm.primary->debugfs_root);
	if (IS_ERR(root))
		return;

	gt_debugfs_register(gt, root);

	intel_gt_debugfs_register_ccs_mode(gt, root);
	intel_gt_engines_debugfs_register(gt, root);
	intel_gt_pm_debugfs_register(gt, root);
	intel_sseu_debugfs_register(gt, root);

	intel_uc_debugfs_register(&gt->uc, root);
	intel_pxp_debugfs_register(&gt->pxp, root);
	intel_iov_debugfs_register(&gt->iov, root);
}

void intel_gt_debugfs_register_files(struct dentry *root,
				     const struct intel_gt_debugfs_file *files,
				     unsigned long count, void *data)
{
	while (count--) {
		umode_t mode = files->fops->write ? 0644 : 0444;

		if (!files->eval || files->eval(data))
			debugfs_create_file(files->name,
					    mode, root, data,
					    files->fops);

		files++;
	}
}
