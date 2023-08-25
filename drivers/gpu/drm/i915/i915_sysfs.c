/*
 * Copyright © 2012 Intel Corporation
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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * Authors:
 *    Ben Widawsky <ben@bwidawsk.net>
 *
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/stat.h>
#include <linux/sysfs.h>

#include "gt/intel_rc6.h"
#include "gt/intel_rps.h"
#include "gt/sysfs_engines.h"

#include "i915_drv.h"
#include "i915_sysfs.h"
#include "intel_pm.h"
#include "intel_sideband.h"

#if IS_ENABLED(CONFIG_DRM_I915_MEMTRACK)
#include "../drm_internal.h"
#endif

static inline struct drm_i915_private *kdev_minor_to_i915(struct device *kdev)
{
	struct drm_minor *minor = dev_get_drvdata(kdev);
	return to_i915(minor->dev);
}

#ifdef CONFIG_PM
static u32 calc_residency(struct drm_i915_private *dev_priv,
			  i915_reg_t reg)
{
	intel_wakeref_t wakeref;
	u64 res = 0;

	with_intel_runtime_pm(&dev_priv->runtime_pm, wakeref)
		res = intel_rc6_residency_us(&dev_priv->gt.rc6, reg);

	return DIV_ROUND_CLOSEST_ULL(res, 1000);
}

static ssize_t rc6_enable_show(struct device *kdev,
			       struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	unsigned int mask;

	mask = 0;
	if (HAS_RC6(dev_priv))
		mask |= BIT(0);
	if (HAS_RC6p(dev_priv))
		mask |= BIT(1);
	if (HAS_RC6pp(dev_priv))
		mask |= BIT(2);

	return sysfs_emit(buf, "%x\n", mask);
}

static ssize_t rc6_residency_ms_show(struct device *kdev,
				     struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	u32 rc6_residency = calc_residency(dev_priv, GEN6_GT_GFX_RC6);
	return sysfs_emit(buf, "%u\n", rc6_residency);
}

static ssize_t rc6p_residency_ms_show(struct device *kdev,
				      struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	u32 rc6p_residency = calc_residency(dev_priv, GEN6_GT_GFX_RC6p);
	return sysfs_emit(buf, "%u\n", rc6p_residency);
}

static ssize_t rc6pp_residency_ms_show(struct device *kdev,
				       struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	u32 rc6pp_residency = calc_residency(dev_priv, GEN6_GT_GFX_RC6pp);
	return sysfs_emit(buf, "%u\n", rc6pp_residency);
}

static ssize_t media_rc6_residency_ms_show(struct device *kdev,
					   struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	u32 rc6_residency = calc_residency(dev_priv, VLV_GT_MEDIA_RC6);
	return sysfs_emit(buf, "%u\n", rc6_residency);
}

static DEVICE_ATTR_RO(rc6_enable);
static DEVICE_ATTR_RO(rc6_residency_ms);
static DEVICE_ATTR_RO(rc6p_residency_ms);
static DEVICE_ATTR_RO(rc6pp_residency_ms);
static DEVICE_ATTR_RO(media_rc6_residency_ms);

static struct attribute *rc6_attrs[] = {
	&dev_attr_rc6_enable.attr,
	&dev_attr_rc6_residency_ms.attr,
	NULL
};

static const struct attribute_group rc6_attr_group = {
	.name = power_group_name,
	.attrs =  rc6_attrs
};

static struct attribute *rc6p_attrs[] = {
	&dev_attr_rc6p_residency_ms.attr,
	&dev_attr_rc6pp_residency_ms.attr,
	NULL
};

static const struct attribute_group rc6p_attr_group = {
	.name = power_group_name,
	.attrs =  rc6p_attrs
};

static struct attribute *media_rc6_attrs[] = {
	&dev_attr_media_rc6_residency_ms.attr,
	NULL
};

static const struct attribute_group media_rc6_attr_group = {
	.name = power_group_name,
	.attrs =  media_rc6_attrs
};
#endif

static int l3_access_valid(struct drm_i915_private *i915, loff_t offset)
{
	if (!HAS_L3_DPF(i915))
		return -EPERM;

	if (!IS_ALIGNED(offset, sizeof(u32)))
		return -EINVAL;

	if (offset >= GEN7_L3LOG_SIZE)
		return -ENXIO;

	return 0;
}

static ssize_t
i915_l3_read(struct file *filp, struct kobject *kobj,
	     struct bin_attribute *attr, char *buf,
	     loff_t offset, size_t count)
{
	struct device *kdev = kobj_to_dev(kobj);
	struct drm_i915_private *i915 = kdev_minor_to_i915(kdev);
	int slice = (int)(uintptr_t)attr->private;
	int ret;

	ret = l3_access_valid(i915, offset);
	if (ret)
		return ret;

	count = round_down(count, sizeof(u32));
	count = min_t(size_t, GEN7_L3LOG_SIZE - offset, count);
	memset(buf, 0, count);

	spin_lock(&i915->gem.contexts.lock);
	if (i915->l3_parity.remap_info[slice])
		memcpy(buf,
		       i915->l3_parity.remap_info[slice] + offset / sizeof(u32),
		       count);
	spin_unlock(&i915->gem.contexts.lock);

	return count;
}

static ssize_t
i915_l3_write(struct file *filp, struct kobject *kobj,
	      struct bin_attribute *attr, char *buf,
	      loff_t offset, size_t count)
{
	struct device *kdev = kobj_to_dev(kobj);
	struct drm_i915_private *i915 = kdev_minor_to_i915(kdev);
	int slice = (int)(uintptr_t)attr->private;
	u32 *remap_info, *freeme = NULL;
	struct i915_gem_context *ctx;
	int ret;

	ret = l3_access_valid(i915, offset);
	if (ret)
		return ret;

	if (count < sizeof(u32))
		return -EINVAL;

	remap_info = kzalloc(GEN7_L3LOG_SIZE, GFP_KERNEL);
	if (!remap_info)
		return -ENOMEM;

	spin_lock(&i915->gem.contexts.lock);

	if (i915->l3_parity.remap_info[slice]) {
		freeme = remap_info;
		remap_info = i915->l3_parity.remap_info[slice];
	} else {
		i915->l3_parity.remap_info[slice] = remap_info;
	}

	count = round_down(count, sizeof(u32));
	memcpy(remap_info + offset / sizeof(u32), buf, count);

	/* NB: We defer the remapping until we switch to the context */
	list_for_each_entry(ctx, &i915->gem.contexts.list, link)
		ctx->remap_slice |= BIT(slice);

	spin_unlock(&i915->gem.contexts.lock);
	kfree(freeme);

	/*
	 * TODO: Ideally we really want a GPU reset here to make sure errors
	 * aren't propagated. Since I cannot find a stable way to reset the GPU
	 * at this point it is left as a TODO.
	*/

	return count;
}

static const struct bin_attribute dpf_attrs = {
	.attr = {.name = "l3_parity", .mode = (S_IRUSR | S_IWUSR)},
	.size = GEN7_L3LOG_SIZE,
	.read = i915_l3_read,
	.write = i915_l3_write,
	.mmap = NULL,
	.private = (void *)0
};

static const struct bin_attribute dpf_attrs_1 = {
	.attr = {.name = "l3_parity_slice_1", .mode = (S_IRUSR | S_IWUSR)},
	.size = GEN7_L3LOG_SIZE,
	.read = i915_l3_read,
	.write = i915_l3_write,
	.mmap = NULL,
	.private = (void *)1
};

static ssize_t gt_act_freq_mhz_show(struct device *kdev,
				    struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *i915 = kdev_minor_to_i915(kdev);
	struct intel_rps *rps = &i915->gt.rps;

	return sysfs_emit(buf, "%d\n", intel_rps_read_actual_frequency(rps));
}

static ssize_t gt_cur_freq_mhz_show(struct device *kdev,
				    struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *i915 = kdev_minor_to_i915(kdev);
	struct intel_rps *rps = &i915->gt.rps;

	return sysfs_emit(buf, "%d\n", intel_rps_get_requested_frequency(rps));
}

static ssize_t gt_boost_freq_mhz_show(struct device *kdev, struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *i915 = kdev_minor_to_i915(kdev);
	struct intel_rps *rps = &i915->gt.rps;

	return sysfs_emit(buf, "%d\n", intel_gpu_freq(rps, rps->boost_freq));
}

static ssize_t gt_boost_freq_mhz_store(struct device *kdev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	struct intel_rps *rps = &dev_priv->gt.rps;
	bool boost = false;
	ssize_t ret;
	u32 val;

	ret = kstrtou32(buf, 0, &val);
	if (ret)
		return ret;

	/* Validate against (static) hardware limits */
	val = intel_freq_opcode(rps, val);
	if (val < rps->min_freq || val > rps->max_freq)
		return -EINVAL;

	mutex_lock(&rps->lock);
	if (val != rps->boost_freq) {
		rps->boost_freq = val;
		boost = atomic_read(&rps->num_waiters);
	}
	mutex_unlock(&rps->lock);
	if (boost)
		schedule_work(&rps->work);

	return count;
}

static ssize_t vlv_rpe_freq_mhz_show(struct device *kdev,
				     struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	struct intel_rps *rps = &dev_priv->gt.rps;

	return sysfs_emit(buf, "%d\n", intel_gpu_freq(rps, rps->efficient_freq));
}

static ssize_t gt_max_freq_mhz_show(struct device *kdev, struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	struct intel_gt *gt = &dev_priv->gt;
	struct intel_rps *rps = &gt->rps;

	return sysfs_emit(buf, "%d\n", intel_rps_get_max_frequency(rps));
}

static ssize_t gt_max_freq_mhz_store(struct device *kdev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	struct intel_gt *gt = &dev_priv->gt;
	struct intel_rps *rps = &gt->rps;
	ssize_t ret;
	u32 val;

	ret = kstrtou32(buf, 0, &val);
	if (ret)
		return ret;

	ret = intel_rps_set_max_frequency(rps, val);

	return ret ?: count;
}

static ssize_t gt_min_freq_mhz_show(struct device *kdev, struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *i915 = kdev_minor_to_i915(kdev);
	struct intel_gt *gt = &i915->gt;
	struct intel_rps *rps = &gt->rps;

	return sysfs_emit(buf, "%d\n", intel_rps_get_min_frequency(rps));
}

static ssize_t gt_min_freq_mhz_store(struct device *kdev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct drm_i915_private *i915 = kdev_minor_to_i915(kdev);
	struct intel_rps *rps = &i915->gt.rps;
	ssize_t ret;
	u32 val;

	ret = kstrtou32(buf, 0, &val);
	if (ret)
		return ret;

	ret = intel_rps_set_min_frequency(rps, val);

	return ret ?: count;
}

static DEVICE_ATTR_RO(gt_act_freq_mhz);
static DEVICE_ATTR_RO(gt_cur_freq_mhz);
static DEVICE_ATTR_RW(gt_boost_freq_mhz);
static DEVICE_ATTR_RW(gt_max_freq_mhz);
static DEVICE_ATTR_RW(gt_min_freq_mhz);

static DEVICE_ATTR_RO(vlv_rpe_freq_mhz);

static ssize_t gt_rp_mhz_show(struct device *kdev, struct device_attribute *attr, char *buf);
static DEVICE_ATTR(gt_RP0_freq_mhz, S_IRUGO, gt_rp_mhz_show, NULL);
static DEVICE_ATTR(gt_RP1_freq_mhz, S_IRUGO, gt_rp_mhz_show, NULL);
static DEVICE_ATTR(gt_RPn_freq_mhz, S_IRUGO, gt_rp_mhz_show, NULL);

/* For now we have a static number of RP states */
static ssize_t gt_rp_mhz_show(struct device *kdev, struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	struct intel_rps *rps = &dev_priv->gt.rps;
	u32 val;

	if (attr == &dev_attr_gt_RP0_freq_mhz)
		val = intel_rps_get_rp0_frequency(rps);
	else if (attr == &dev_attr_gt_RP1_freq_mhz)
		val = intel_rps_get_rp1_frequency(rps);
	else if (attr == &dev_attr_gt_RPn_freq_mhz)
		val = intel_rps_get_rpn_frequency(rps);
	else
		BUG();

	return sysfs_emit(buf, "%d\n", val);
}

static const struct attribute * const gen6_attrs[] = {
	&dev_attr_gt_act_freq_mhz.attr,
	&dev_attr_gt_cur_freq_mhz.attr,
	&dev_attr_gt_boost_freq_mhz.attr,
	&dev_attr_gt_max_freq_mhz.attr,
	&dev_attr_gt_min_freq_mhz.attr,
	&dev_attr_gt_RP0_freq_mhz.attr,
	&dev_attr_gt_RP1_freq_mhz.attr,
	&dev_attr_gt_RPn_freq_mhz.attr,
	NULL,
};

static const struct attribute * const vlv_attrs[] = {
	&dev_attr_gt_act_freq_mhz.attr,
	&dev_attr_gt_cur_freq_mhz.attr,
	&dev_attr_gt_boost_freq_mhz.attr,
	&dev_attr_gt_max_freq_mhz.attr,
	&dev_attr_gt_min_freq_mhz.attr,
	&dev_attr_gt_RP0_freq_mhz.attr,
	&dev_attr_gt_RP1_freq_mhz.attr,
	&dev_attr_gt_RPn_freq_mhz.attr,
	&dev_attr_vlv_rpe_freq_mhz.attr,
	NULL,
};

#if IS_ENABLED(CONFIG_DRM_I915_CAPTURE_ERROR)

static ssize_t error_state_read(struct file *filp, struct kobject *kobj,
				struct bin_attribute *attr, char *buf,
				loff_t off, size_t count)
{

	struct device *kdev = kobj_to_dev(kobj);
	struct drm_i915_private *i915 = kdev_minor_to_i915(kdev);
	struct i915_gpu_coredump *gpu;
	ssize_t ret = 0;

	/*
	 * FIXME: Concurrent clients triggering resets and reading + clearing
	 * dumps can cause inconsistent sysfs reads when a user calls in with a
	 * non-zero offset to complete a prior partial read but the
	 * gpu_coredump has been cleared or replaced.
	 */

	gpu = i915_first_error_state(i915);
	if (IS_ERR(gpu)) {
		ret = PTR_ERR(gpu);
	} else if (gpu) {
		ret = i915_gpu_coredump_copy_to_buffer(gpu, buf, off, count);
		i915_gpu_coredump_put(gpu);
	} else {
		const char *str = "No error state collected\n";
		size_t len = strlen(str);

		if (off < len) {
			ret = min_t(size_t, count, len - off);
			memcpy(buf, str + off, ret);
		}
	}

	return ret;
}

static ssize_t error_state_write(struct file *file, struct kobject *kobj,
				 struct bin_attribute *attr, char *buf,
				 loff_t off, size_t count)
{
	struct device *kdev = kobj_to_dev(kobj);
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);

	drm_dbg(&dev_priv->drm, "Resetting error state\n");
	i915_reset_error_state(dev_priv);

	return count;
}

static const struct bin_attribute error_state_attr = {
	.attr.name = "error",
	.attr.mode = S_IRUSR | S_IWUSR,
	.size = 0,
	.read = error_state_read,
	.write = error_state_write,
};

static void i915_setup_error_capture(struct device *kdev)
{
	if (sysfs_create_bin_file(&kdev->kobj, &error_state_attr))
		DRM_ERROR("error_state sysfs setup failed\n");
}

static void i915_teardown_error_capture(struct device *kdev)
{
	sysfs_remove_bin_file(&kdev->kobj, &error_state_attr);
}

#if IS_ENABLED(CONFIG_DRM_I915_MEMTRACK)
#define dev_to_drm_minor(d) dev_get_drvdata((d))

static ssize_t i915_gem_clients_state_read(struct file *filp,
	struct kobject *memtrack_kobj,
	struct bin_attribute *attr,
	char *buf, loff_t off, size_t count)
{
	struct kobject *kobj = memtrack_kobj->parent;
	struct device *kdev = container_of(kobj, struct device, kobj);
	struct drm_minor *minor = dev_to_drm_minor(kdev);
	struct drm_device *dev = minor->dev;
	struct drm_i915_private *i915 = to_i915(dev);
	struct drm_i915_error_state_buf error_str;
	ssize_t ret_count = 0;
	int ret;

	ret = i915_error_state_buf_init(&error_str, i915, count, off);
	if (ret)
		return ret;

	ret = i915_get_drm_clients_info(&error_str, dev);
	if (ret)
		goto out;

	ret_count = count < error_str.bytes ? count : error_str.bytes;

	memcpy(buf, error_str.buf, ret_count);
out:
	i915_error_state_buf_release(&error_str);

	return ret ?: ret_count;
}

#define GEM_OBJ_STAT_BUF_SIZE (4*1024) /* 4KB */
#define GEM_OBJ_STAT_BUF_SIZE_MAX (1024*1024) /* 1MB */

struct i915_gem_file_attr_priv {
	char tgid_str[16];
	struct pid *tgid;
	struct drm_i915_error_state_buf buf;
};

static ssize_t i915_gem_read_objects(struct file *filp,
		struct kobject *memtrack_kobj,
		struct bin_attribute *attr,
		char *buf, loff_t off, size_t count)
{
	struct kobject *kobj = memtrack_kobj->parent;
	struct device *kdev = container_of(kobj, struct device, kobj);
	struct drm_minor *minor = dev_to_drm_minor(kdev);
	struct drm_device *dev = minor->dev;
	struct i915_gem_file_attr_priv *attr_priv;
	struct pid *tgid;
	ssize_t ret_count = 0;
	long bytes_available;
	int ret = 0, buf_size = GEM_OBJ_STAT_BUF_SIZE;
	unsigned long timeout = msecs_to_jiffies(500) + 1;

	/*
	 * There may arise a scenario where sysfs file entry is being removed,
	 * and may race against sysfs read. Sysfs file remove function would
	 * have taken the drm_global_mutex and would wait for read to finish,
	 * which is again waiting to acquire drm_global_mutex, leading to
	 * deadlock. To avoid this, use mutex_trylock here with a timeout.
	 */
	while (!mutex_trylock(&drm_global_mutex) && --timeout)
		schedule_timeout_killable(1);
	if (timeout == 0) {
		DRM_DEBUG_DRIVER("Unable to acquire drm global mutex.\n");
		return -EBUSY;
	}

	if (!attr || !attr->private) {
		ret = -EINVAL;
		DRM_ERROR("attr | attr->private pointer is NULL\n");
		goto out;
	}
	attr_priv = attr->private;
	tgid = attr_priv->tgid;

	if (off && !attr_priv->buf.buf) {
		ret = -EINVAL;
		DRM_ERROR(
			"Buf not allocated during read with non-zero offset\n");
		goto out;
	}

	if (off == 0) {
retry:
		if (!attr_priv->buf.buf) {
			ret = i915_obj_state_buf_init(&attr_priv->buf,
					buf_size);
			if (ret) {
				DRM_ERROR(
					"obj state buf init failed. buf_size=%d\n",
					buf_size);
				goto out;
			}
		} else {
			/* Reset the buf parameters before filling data */
			// attr_priv->buf.pos = 0;
			attr_priv->buf.bytes = 0;
		}

		/* Read the gfx device stats */
		ret = i915_gem_get_obj_info(&attr_priv->buf, dev, tgid);
		if (ret)
			goto out;

		ret = i915_error_ok(&attr_priv->buf);
		if (ret) {
			ret = 0;
			goto copy_data;
		}
		if (buf_size >= GEM_OBJ_STAT_BUF_SIZE_MAX) {
			DRM_DEBUG_DRIVER("obj stat buf size limit reached\n");
			ret = -ENOMEM;
			goto out;
		} else {
			/* Try to reallocate buf of larger size */
			i915_error_state_buf_release(&attr_priv->buf);
			buf_size *= 2;

			ret = i915_obj_state_buf_init(&attr_priv->buf,
					buf_size);
			if (ret) {
				DRM_ERROR(
						"obj stat buf init failed. buf_size=%d\n",
						buf_size);
				goto out;
			}
			goto retry;
		}
	}
copy_data:

	bytes_available = (long)attr_priv->buf.bytes - (long)off;

	if (bytes_available > 0) {
		ret_count = count < bytes_available ? count : bytes_available;
		memcpy(buf, attr_priv->buf.buf + off, ret_count);
	} else
		ret_count = 0;

out:
	mutex_unlock(&drm_global_mutex);

	return ret ?: ret_count;
}

int i915_gem_create_sysfs_file_entry(struct drm_device *dev,
		struct drm_file *file)
{
	struct drm_i915_file_private *file_priv = file->driver_priv;
	struct drm_i915_private *i915 = to_i915(dev);
	struct i915_gem_file_attr_priv *attr_priv;
	struct bin_attribute *obj_attr;
	struct drm_file *file_local;
	int ret;

	/*
	 * Check for multiple drm files having same tgid. If found, copy the
	 * bin attribute into the new file priv. Otherwise allocate a new
	 * copy of bin attribute, and create its corresponding sysfs file.
	 */
	mutex_lock(&dev->filelist_mutex);
	list_for_each_entry(file_local, &dev->filelist, lhead) {
		struct drm_i915_file_private *file_priv_local =
			file_local->driver_priv;

		if (file_priv->tgid == file_priv_local->tgid) {
			file_priv->obj_attr = file_priv_local->obj_attr;
			mutex_unlock(&dev->filelist_mutex);
			return 0;
		}
	}
	mutex_unlock(&dev->filelist_mutex);

	obj_attr = kzalloc(sizeof(*obj_attr), GFP_KERNEL);
	if (!obj_attr) {
		DRM_ERROR("Alloc failed. Out of memory\n");
		ret = -ENOMEM;
		goto out;
	}

	attr_priv = kzalloc(sizeof(*attr_priv), GFP_KERNEL);
	if (!attr_priv) {
		DRM_ERROR("Alloc failed. Out of memory\n");
		ret = -ENOMEM;
		goto out_obj_attr;
	}

	snprintf(attr_priv->tgid_str, 16, "%d", task_tgid_nr(current));
	obj_attr->attr.name = attr_priv->tgid_str;
	obj_attr->attr.mode = 0644;
	obj_attr->size = 0;
	obj_attr->read = i915_gem_read_objects;

	attr_priv->tgid = file_priv->tgid;
	obj_attr->private = attr_priv;

	if (kobject_name(&i915->memtrack_kobj)) {
		ret = sysfs_create_bin_file(&i915->memtrack_kobj, obj_attr);
		if (ret) {
			DRM_ERROR(
				"sysfs tgid file setup failed. tgid=%d, process:%s, ret:%d\n",
				pid_nr(file_priv->tgid), file_priv->process_name, ret);
			goto out_attr_priv;
		}
	}

	file_priv->obj_attr = obj_attr;
	return 0;

out_attr_priv:
	kfree(attr_priv);
out_obj_attr:
	kfree(obj_attr);
out:
	return ret;
}

void i915_gem_remove_sysfs_file_entry(struct drm_device *dev,
		struct drm_file *file)
{
	struct drm_i915_private *i915 = to_i915(dev);
	struct drm_i915_file_private *file_priv = file->driver_priv;
	struct drm_file *file_local;
	int open_count = 1;

	/*
	 * The current drm file instance is already removed from filelist at
	 * this point.
	 * Check if this particular drm file being removed is the last one for
	 * that particular tgid, and no other instances for this tgid exist in
	 * the filelist. If so, remove the corresponding sysfs file entry also.
	 */
	list_for_each_entry(file_local, &dev->filelist, lhead) {
		struct drm_i915_file_private *file_priv_local =
			file_local->driver_priv;

		if (pid_nr(file_priv->tgid) == pid_nr(file_priv_local->tgid))
			open_count++;
	}

	if (open_count == 1) {
		struct i915_gem_file_attr_priv *attr_priv;

		if (WARN_ON(file_priv->obj_attr == NULL))
			return;
		attr_priv = file_priv->obj_attr->private;

		sysfs_remove_bin_file(&i915->memtrack_kobj,
				file_priv->obj_attr);

		i915_error_state_buf_release(&attr_priv->buf);
		kfree(file_priv->obj_attr->private);
		kfree(file_priv->obj_attr);
	}
}

static struct bin_attribute i915_gem_client_state_attr = {
	.attr.name = "i915_gem_meminfo",
	.attr.mode = 0644,
	.size = 0,
	.read = i915_gem_clients_state_read,
};

static struct attribute *memtrack_kobj_attrs[] = {NULL};

static struct kobj_type memtrack_kobj_type = {
	.release = NULL,
	.sysfs_ops = NULL,
	.default_attrs = memtrack_kobj_attrs,
};
#endif
#else
static void i915_setup_error_capture(struct device *kdev) {}
static void i915_teardown_error_capture(struct device *kdev) {}
#endif

void i915_setup_sysfs(struct drm_i915_private *dev_priv)
{
	struct device *kdev = dev_priv->drm.primary->kdev;
	int ret;

#ifdef CONFIG_PM
	if (HAS_RC6(dev_priv)) {
		ret = sysfs_merge_group(&kdev->kobj,
					&rc6_attr_group);
		if (ret)
			drm_err(&dev_priv->drm,
				"RC6 residency sysfs setup failed\n");
	}
	if (HAS_RC6p(dev_priv)) {
		ret = sysfs_merge_group(&kdev->kobj,
					&rc6p_attr_group);
		if (ret)
			drm_err(&dev_priv->drm,
				"RC6p residency sysfs setup failed\n");
	}
	if (IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv)) {
		ret = sysfs_merge_group(&kdev->kobj,
					&media_rc6_attr_group);
		if (ret)
			drm_err(&dev_priv->drm,
				"Media RC6 residency sysfs setup failed\n");
	}
#endif
	if (HAS_L3_DPF(dev_priv)) {
		ret = device_create_bin_file(kdev, &dpf_attrs);
		if (ret)
			drm_err(&dev_priv->drm,
				"l3 parity sysfs setup failed\n");

		if (NUM_L3_SLICES(dev_priv) > 1) {
			ret = device_create_bin_file(kdev,
						     &dpf_attrs_1);
			if (ret)
				drm_err(&dev_priv->drm,
					"l3 parity slice 1 setup failed\n");
		}
	}

	ret = 0;
	if (IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv))
		ret = sysfs_create_files(&kdev->kobj, vlv_attrs);
	else if (GRAPHICS_VER(dev_priv) >= 6)
		ret = sysfs_create_files(&kdev->kobj, gen6_attrs);
	if (ret)
		drm_err(&dev_priv->drm, "RPS sysfs setup failed\n");

	i915_setup_error_capture(kdev);

#if IS_ENABLED(CONFIG_DRM_I915_MEMTRACK)
	/*
	 * Create the gfx_memtrack directory for memtrack sysfs files
	 */
	ret = kobject_init_and_add(
			&dev_priv->memtrack_kobj, &memtrack_kobj_type,
			&kdev->kobj, "gfx_memtrack");
	if (unlikely(ret != 0)) {
		DRM_ERROR(
			"i915 sysfs setup memtrack directory failed\n");
		kobject_put(&dev_priv->memtrack_kobj);
	} else {
		ret = sysfs_create_bin_file(&dev_priv->memtrack_kobj,
				&i915_gem_client_state_attr);
		if (ret)
			DRM_ERROR(
				"i915_gem_client_state sysfs setup failed\n");
	}
#endif

	intel_engines_add_sysfs(dev_priv);
}

void i915_teardown_sysfs(struct drm_i915_private *dev_priv)
{
	struct device *kdev = dev_priv->drm.primary->kdev;

	i915_teardown_error_capture(kdev);

	if (IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv))
		sysfs_remove_files(&kdev->kobj, vlv_attrs);
	else
		sysfs_remove_files(&kdev->kobj, gen6_attrs);
	device_remove_bin_file(kdev,  &dpf_attrs_1);
	device_remove_bin_file(kdev,  &dpf_attrs);
#ifdef CONFIG_PM
	sysfs_unmerge_group(&kdev->kobj, &rc6_attr_group);
	sysfs_unmerge_group(&kdev->kobj, &rc6p_attr_group);
#endif

#if IS_ENABLED(CONFIG_DRM_I915_MEMTRACK)
	sysfs_remove_bin_file(&dev_priv->memtrack_kobj,
			&i915_gem_client_state_attr);
	kobject_del(&dev_priv->memtrack_kobj);
	kobject_put(&dev_priv->memtrack_kobj);
#endif
}
