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

#include "i915_drv.h"
#include "i915_sysfs.h"
#include "intel_pm.h"
#include "intel_sideband.h"

#if IS_ENABLED(CONFIG_DRM_I915_GVT)
#include "gvt/gvt.h"
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

static ssize_t
show_rc6_mask(struct device *kdev, struct device_attribute *attr, char *buf)
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

	return snprintf(buf, PAGE_SIZE, "%x\n", mask);
}

static ssize_t
show_rc6_ms(struct device *kdev, struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	u32 rc6_residency = calc_residency(dev_priv, GEN6_GT_GFX_RC6);
	return snprintf(buf, PAGE_SIZE, "%u\n", rc6_residency);
}

static ssize_t
show_rc6p_ms(struct device *kdev, struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	u32 rc6p_residency = calc_residency(dev_priv, GEN6_GT_GFX_RC6p);
	return snprintf(buf, PAGE_SIZE, "%u\n", rc6p_residency);
}

static ssize_t
show_rc6pp_ms(struct device *kdev, struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	u32 rc6pp_residency = calc_residency(dev_priv, GEN6_GT_GFX_RC6pp);
	return snprintf(buf, PAGE_SIZE, "%u\n", rc6pp_residency);
}

static ssize_t
show_media_rc6_ms(struct device *kdev, struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	u32 rc6_residency = calc_residency(dev_priv, VLV_GT_MEDIA_RC6);
	return snprintf(buf, PAGE_SIZE, "%u\n", rc6_residency);
}

static DEVICE_ATTR(rc6_enable, S_IRUGO, show_rc6_mask, NULL);
static DEVICE_ATTR(rc6_residency_ms, S_IRUGO, show_rc6_ms, NULL);
static DEVICE_ATTR(rc6p_residency_ms, S_IRUGO, show_rc6p_ms, NULL);
static DEVICE_ATTR(rc6pp_residency_ms, S_IRUGO, show_rc6pp_ms, NULL);
static DEVICE_ATTR(media_rc6_residency_ms, S_IRUGO, show_media_rc6_ms, NULL);

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
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	struct intel_rps *rps = &dev_priv->gt.rps;
	intel_wakeref_t wakeref;
	u32 freq;

	wakeref = intel_runtime_pm_get(&dev_priv->runtime_pm);

	if (IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv)) {
		vlv_punit_get(dev_priv);
		freq = vlv_punit_read(dev_priv, PUNIT_REG_GPU_FREQ_STS);
		vlv_punit_put(dev_priv);

		freq = (freq >> 8) & 0xff;
	} else {
		freq = intel_get_cagf(rps, I915_READ(GEN6_RPSTAT1));
	}

	intel_runtime_pm_put(&dev_priv->runtime_pm, wakeref);

	return snprintf(buf, PAGE_SIZE, "%d\n", intel_gpu_freq(rps, freq));
}

static ssize_t gt_cur_freq_mhz_show(struct device *kdev,
				    struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	struct intel_rps *rps = &dev_priv->gt.rps;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			intel_gpu_freq(rps, rps->cur_freq));
}

static ssize_t gt_boost_freq_mhz_show(struct device *kdev, struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	struct intel_rps *rps = &dev_priv->gt.rps;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			intel_gpu_freq(rps, rps->boost_freq));
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

	return snprintf(buf, PAGE_SIZE, "%d\n",
			intel_gpu_freq(rps, rps->efficient_freq));
}

static ssize_t gt_max_freq_mhz_show(struct device *kdev, struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	struct intel_rps *rps = &dev_priv->gt.rps;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			intel_gpu_freq(rps, rps->max_freq_softlimit));
}

static ssize_t gt_max_freq_mhz_store(struct device *kdev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	struct intel_rps *rps = &dev_priv->gt.rps;
	ssize_t ret;
	u32 val;

	ret = kstrtou32(buf, 0, &val);
	if (ret)
		return ret;

	mutex_lock(&rps->lock);

	val = intel_freq_opcode(rps, val);
	if (val < rps->min_freq ||
	    val > rps->max_freq ||
	    val < rps->min_freq_softlimit) {
		ret = -EINVAL;
		goto unlock;
	}

	if (val > rps->rp0_freq)
		DRM_DEBUG("User requested overclocking to %d\n",
			  intel_gpu_freq(rps, val));

	rps->max_freq_softlimit = val;

	val = clamp_t(int, rps->cur_freq,
		      rps->min_freq_softlimit,
		      rps->max_freq_softlimit);

	/*
	 * We still need *_set_rps to process the new max_delay and
	 * update the interrupt limits and PMINTRMSK even though
	 * frequency request may be unchanged.
	 */
	intel_rps_set(rps, val);

unlock:
	mutex_unlock(&rps->lock);

	return ret ?: count;
}

static ssize_t gt_min_freq_mhz_show(struct device *kdev, struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	struct intel_rps *rps = &dev_priv->gt.rps;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			intel_gpu_freq(rps, rps->min_freq_softlimit));
}

static ssize_t gt_min_freq_mhz_store(struct device *kdev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	struct intel_rps *rps = &dev_priv->gt.rps;
	ssize_t ret;
	u32 val;

	ret = kstrtou32(buf, 0, &val);
	if (ret)
		return ret;

	mutex_lock(&rps->lock);

	val = intel_freq_opcode(rps, val);
	if (val < rps->min_freq ||
	    val > rps->max_freq ||
	    val > rps->max_freq_softlimit) {
		ret = -EINVAL;
		goto unlock;
	}

	rps->min_freq_softlimit = val;

	val = clamp_t(int, rps->cur_freq,
		      rps->min_freq_softlimit,
		      rps->max_freq_softlimit);

	/*
	 * We still need *_set_rps to process the new min_delay and
	 * update the interrupt limits and PMINTRMSK even though
	 * frequency request may be unchanged.
	 */
	intel_rps_set(rps, val);

unlock:
	mutex_unlock(&rps->lock);

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
		val = intel_gpu_freq(rps, rps->rp0_freq);
	else if (attr == &dev_attr_gt_RP1_freq_mhz)
		val = intel_gpu_freq(rps, rps->rp1_freq);
	else if (attr == &dev_attr_gt_RPn_freq_mhz)
		val = intel_gpu_freq(rps, rps->min_freq);
	else
		BUG();

	return snprintf(buf, PAGE_SIZE, "%d\n", val);
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
	struct i915_gpu_state *gpu;
	ssize_t ret;

	gpu = i915_first_error_state(i915);
	if (IS_ERR(gpu)) {
		ret = PTR_ERR(gpu);
	} else if (gpu) {
		ret = i915_gpu_state_copy_to_buffer(gpu, buf, off, count);
		i915_gpu_state_put(gpu);
	} else {
		const char *str = "No error state collected\n";
		size_t len = strlen(str);

		ret = min_t(size_t, count, len - off);
		memcpy(buf, str + off, ret);
	}

	return ret;
}

static ssize_t error_state_write(struct file *file, struct kobject *kobj,
				 struct bin_attribute *attr, char *buf,
				 loff_t off, size_t count)
{
	struct device *kdev = kobj_to_dev(kobj);
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);

	DRM_DEBUG_DRIVER("Resetting error state\n");
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
#else
static void i915_setup_error_capture(struct device *kdev) {}
static void i915_teardown_error_capture(struct device *kdev) {}
#endif

#if IS_ENABLED(CONFIG_DRM_I915_GVT)
static ssize_t gvt_disp_ports_mask_show(
	struct device *kdev, struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);

	return snprintf(buf, PAGE_SIZE, "Display ports assignment: 0x%016llx\n",
			dev_priv->gvt->sel_disp_port_mask);
}

static ssize_t gvt_disp_ports_mask_store(
	struct device *kdev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	u64 val;
	ssize_t ret;

	ret = kstrtou64(buf, 0, &val);
	if (ret)
		return ret;

	intel_gvt_store_vgpu_display_mask(dev_priv, val);

	return count;
}

static ssize_t gvt_disp_ports_status_show(
	struct device *kdev, struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	struct intel_gvt *gvt = dev_priv->gvt;
	int id;
	u32 port_mask;
	u8 port_sel, port;
	ssize_t count_total = 0;
	size_t buf_size = PAGE_SIZE;
	size_t count = 0;

	count = snprintf(buf, buf_size, "Auto host/vGPU display switch: %s\n",
			 READ_ONCE(dev_priv->gvt->disp_auto_switch) ?
			 "Y" : "N");
	buf_size -= count;
	count_total += count;
	buf += count;
	if (!buf_size)
		return count_total;

	count = snprintf(buf, buf_size, "Available display ports: 0x%016llx\n",
			 gvt->avail_disp_port_mask);
	buf_size -= count;
	count_total += count;
	buf += count;
	if (!buf_size)
		return count_total;

	for (port = PORT_A; port < I915_MAX_PORTS; port++) {
		port_sel = intel_gvt_external_disp_id_from_port(port);
		if (gvt->avail_disp_port_mask & intel_gvt_port_to_mask_bit(port_sel, port)) {
			count = snprintf(buf, buf_size, "  ( PORT_%c(%d) )\n",
					 port_name(port), port_sel);
			buf_size -= count;
			count_total += count;
			buf += count;
			if (!buf_size)
				return count_total;
		}
	}

	count = snprintf(buf, buf_size,
			 "Display ports assignment: 0x%016llx\n",
			 gvt->sel_disp_port_mask);
	buf_size -= count;
	count_total += count;
	buf += count;
	if (!buf_size)
		return count_total;

	count = snprintf(buf, buf_size,
			 "  Each byte represents the bit mask of assigned port(s) to vGPU 1-8 (low to high)\n");
	buf_size -= count;
	count_total += count;
	buf += count;
	if (!buf_size)
		return count_total;

	count = snprintf(buf, buf_size,
			 "    Bit mask is decoded from LSB to MSB (PORT_A to I915_MAX_PORTS):\n");
	buf_size -= count;
	count_total += count;
	buf += count;
	if (!buf_size)
		return count_total;

	count = snprintf(buf, buf_size,
			 "      0: This port isn't assigned to that vGPU.\n");
	buf_size -= count;
	count_total += count;
	buf += count;
	if (!buf_size)
		return count_total;

	count = snprintf(buf, buf_size,
			 "      1: This port is assigned to that vGPU.\n");
	buf_size -= count;
	count_total += count;
	buf += count;
	if (!buf_size)
		return count_total;

	for (id = 0; id < GVT_MAX_VGPU; id++) {
		port_mask = (gvt->sel_disp_port_mask >> (id * 8)) & 0xFF;
		if (port_mask) {
			count = snprintf(buf, buf_size,
					 "  vGPU-%d port mask: (0x%02x)\n",
					 id + 1, port_mask);
			buf_size -= count;
			count_total += count;
			buf += count;
			if (!buf_size)
				return count_total;

			for (port = PORT_A; port < I915_MAX_PORTS; port++) {
				if (port_mask & (1 << port)) {
					port_sel = intel_gvt_external_disp_id_from_port(port);
					count = snprintf(buf, buf_size,
							 "    ( PORT_%c(%d) )\n",
							 port_name(port), port_sel);
					buf_size -= count;
					count_total += count;
					buf += count;
					if (!buf_size)
						return count_total;
				}
			}
		}
	}

	count = snprintf(buf, buf_size,
			 "Display ports ownership: 0x%08x\n",
			 gvt->disp_owner);
	buf_size -= count;
	count_total += count;
	buf += count;
	if (!buf_size)
		return count_total;

	count = snprintf(buf, buf_size,
			 "  x on n-th hex-digit: (port_name(port_id#) <---> owner)\n");
	buf_size -= count;
	count_total += count;
	buf += count;
	if (!buf_size)
		return count_total;

	count = snprintf(buf, buf_size,
			 "    n: port id, PORT_A(1), PORT_B(2), ...\n");
	buf_size -= count;
	count_total += count;
	buf += count;
	if (!buf_size)
		return count_total;

	count = snprintf(buf, buf_size, "    x: owner: host (0), vGPU id(x)\n");
	buf_size -= count;
	count_total += count;
	buf += count;
	if (!buf_size)
		return count_total;

	for (port = PORT_A; port < I915_MAX_PORTS; port++) {
		port_sel = intel_gvt_external_disp_id_from_port(port);
		if (gvt->avail_disp_port_mask & intel_gvt_port_to_mask_bit(port_sel, port)) {
			count = snprintf(buf, buf_size, "  ( PORT_%c(%d) ",
					 port_name(port), port_sel);
			buf_size -= count;
			count_total += count;
			buf += count;
			if (!buf_size)
				return count_total;

			id = (gvt->disp_owner >> port * 4) & 0xF;
			if (id)
				count = snprintf(buf, buf_size,
						 "<---> VGPU-%d )\n", id);
			else
				count = snprintf(buf, buf_size,
						 "<---> Host )\n");

			buf_size -= count;
			count_total += count;
			buf += count;
			if (!buf_size)
				return count_total;
		}
	}

	return count_total;
}

static ssize_t gvt_disp_ports_owner_show(
	struct device *kdev, struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);

	return snprintf(buf, PAGE_SIZE,
			"Display ports ownership: 0x%08x\n",
			dev_priv->gvt->disp_owner);
}

static ssize_t gvt_disp_ports_owner_store(
	struct device *kdev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	u32 val;
	ssize_t ret;

	ret = kstrtou32(buf, 0, &val);
	if (ret)
		return ret;

	intel_gvt_store_vgpu_display_owner(dev_priv, val);

	return count;
}

static ssize_t gvt_disp_auto_switch_show(
	struct device *kdev, struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);

	return snprintf(buf, PAGE_SIZE,
			"%s\n",
			READ_ONCE(dev_priv->gvt->disp_auto_switch) ?
			"Y" : "N");
}

static ssize_t gvt_disp_auto_switch_store(
	struct device *kdev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	u32 val;
	ssize_t ret;

	ret = kstrtou32(buf, 0, &val);
	if (ret)
		return ret;

	intel_gvt_store_vgpu_display_switch(dev_priv, val != 0);

	return count;
}

static ssize_t gvt_disp_edid_filter_show(
	struct device *kdev, struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);

	return snprintf(buf, PAGE_SIZE,
			"%s\n",
			READ_ONCE(dev_priv->gvt->disp_edid_filter) ?
			"Y" : "N");
}

static ssize_t gvt_disp_edid_filter_store(
	struct device *kdev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct drm_i915_private *dev_priv = kdev_minor_to_i915(kdev);
	u32 val;
	ssize_t ret;

	ret = kstrtou32(buf, 0, &val);
	if (ret)
		return ret;

	if (val != 0)
		WRITE_ONCE(dev_priv->gvt->disp_edid_filter, true);
	else
		WRITE_ONCE(dev_priv->gvt->disp_edid_filter, false);

	return count;
}


static struct device_attribute dev_attr_gvt_disp_ports_mask =
	__ATTR(gvt_disp_ports_mask, 0644,
	       gvt_disp_ports_mask_show, gvt_disp_ports_mask_store);
static struct device_attribute dev_attr_gvt_disp_ports_status =
	__ATTR(gvt_disp_ports_status, 0444,
	       gvt_disp_ports_status_show, NULL);
static struct device_attribute dev_attr_gvt_disp_ports_owner =
	__ATTR(gvt_disp_ports_owner, 0644,
	       gvt_disp_ports_owner_show, gvt_disp_ports_owner_store);
static struct device_attribute dev_attr_gvt_disp_auto_switch =
	__ATTR(gvt_disp_auto_switch, 0644,
	       gvt_disp_auto_switch_show, gvt_disp_auto_switch_store);
static struct device_attribute dev_attr_gvt_disp_edid_filter =
	__ATTR(gvt_disp_edid_filter, 0644,
	       gvt_disp_edid_filter_show, gvt_disp_edid_filter_store);


static const struct attribute *gvt_attrs[] = {
	&dev_attr_gvt_disp_ports_mask.attr,
	&dev_attr_gvt_disp_ports_status.attr,
	&dev_attr_gvt_disp_ports_owner.attr,
	&dev_attr_gvt_disp_auto_switch.attr,
	&dev_attr_gvt_disp_edid_filter.attr,
	NULL,
};

#endif

void i915_setup_sysfs(struct drm_i915_private *dev_priv)
{
	struct device *kdev = dev_priv->drm.primary->kdev;
	int ret;
#if IS_ENABLED(CONFIG_DRM_I915_GVT)
	struct intel_gvt *gvt = dev_priv->gvt;
#endif

#ifdef CONFIG_PM
	if (HAS_RC6(dev_priv)) {
		ret = sysfs_merge_group(&kdev->kobj,
					&rc6_attr_group);
		if (ret)
			DRM_ERROR("RC6 residency sysfs setup failed\n");
	}
	if (HAS_RC6p(dev_priv)) {
		ret = sysfs_merge_group(&kdev->kobj,
					&rc6p_attr_group);
		if (ret)
			DRM_ERROR("RC6p residency sysfs setup failed\n");
	}
	if (IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv)) {
		ret = sysfs_merge_group(&kdev->kobj,
					&media_rc6_attr_group);
		if (ret)
			DRM_ERROR("Media RC6 residency sysfs setup failed\n");
	}
#endif
	if (HAS_L3_DPF(dev_priv)) {
		ret = device_create_bin_file(kdev, &dpf_attrs);
		if (ret)
			DRM_ERROR("l3 parity sysfs setup failed\n");

		if (NUM_L3_SLICES(dev_priv) > 1) {
			ret = device_create_bin_file(kdev,
						     &dpf_attrs_1);
			if (ret)
				DRM_ERROR("l3 parity slice 1 setup failed\n");
		}
	}

	ret = 0;
	if (IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv))
		ret = sysfs_create_files(&kdev->kobj, vlv_attrs);
	else if (INTEL_GEN(dev_priv) >= 6)
		ret = sysfs_create_files(&kdev->kobj, gen6_attrs);
	if (ret)
		DRM_ERROR("RPS sysfs setup failed\n");

#if IS_ENABLED(CONFIG_DRM_I915_GVT)
	if (gvt) {
		if (sysfs_create_files(&kdev->kobj, gvt_attrs))
			DRM_ERROR("gvt attr setup failed\n");
	}
#endif

	i915_setup_error_capture(kdev);
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
}
