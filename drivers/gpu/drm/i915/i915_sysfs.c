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

#include "gem/i915_gem_mman.h"
#include "gt/intel_gt.h"
#include "gt/intel_gt_pm.h"
#include "gt/intel_gt_regs.h"
#include "gt/intel_gt_requests.h"
#include "gt/intel_rc6.h"
#include "gt/intel_rps.h"
#include "gt/intel_gt_sysfs.h"
#include "gt/sysfs_engines.h"

#include "i915_drv.h"
#include "i915_sriov_sysfs.h"
#include "i915_sysfs.h"
#include "intel_pcode.h"
#include "intel_pm.h"
#include "intel_sysfs_mem_health.h"
#include "i915_debugger.h"
#include "i915_addr_trans_svc.h"

static ssize_t
i915_sysfs_show(struct device *dev, struct device_attribute *attr, char *buf);

static ssize_t
i915_sysfs_store(struct device *dev, struct device_attribute *attr, const char
		 *buf, size_t count);

typedef ssize_t (*show)(struct device *dev, struct device_attribute *attr, char
			*buf);
typedef ssize_t (*store)(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count);

struct ext_attr {
	struct device_attribute attr;
	unsigned long id;
	show i915_show;
};

struct i915_ext_attr {
	struct device_attribute attr;
	show i915_show;
	store i915_store;
};

struct sysfs_bin_ext_attr {
	struct bin_attribute attr;
	ssize_t (*i915_read)(struct file *, struct kobject *, struct
			     bin_attribute *, char *, loff_t, size_t);
	ssize_t (*i915_write)(struct file *, struct kobject *, struct
			      bin_attribute *, char *, loff_t, size_t);
};

struct drm_i915_private *kdev_minor_to_i915(struct device *kdev)
{
	struct drm_minor *minor = dev_get_drvdata(kdev);
	return to_i915(minor->dev);
}

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

	spin_lock_irq(&i915->gem.contexts.lock);
	if (i915->l3_parity.remap_info[slice])
		memcpy(buf,
		       i915->l3_parity.remap_info[slice] + offset / sizeof(u32),
		       count);
	spin_unlock_irq(&i915->gem.contexts.lock);

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

	spin_lock_irq(&i915->gem.contexts.lock);

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

	spin_unlock_irq(&i915->gem.contexts.lock);
	kfree(freeme);

	/*
	 * TODO: Ideally we really want a GPU reset here to make sure errors
	 * aren't propagated. Since I cannot find a stable way to reset the GPU
	 * at this point it is left as a TODO.
	*/

	return count;
}

static ssize_t
i915_sysfs_read(struct file *filp, struct kobject *kobj, struct bin_attribute
	      *attr, char *buf, loff_t offset, size_t count)
{
	ssize_t value;
	struct sysfs_bin_ext_attr *ea = container_of(attr, struct
						     sysfs_bin_ext_attr, attr);
	struct device *kdev = kobj_to_dev(kobj);
	struct drm_i915_private *i915 = kdev_minor_to_i915(kdev);

	/* Wa_16015476723 & Wa_16015666671 */
	pvc_wa_disallow_rc6(i915);

	value = ea->i915_read(filp, kobj, attr, buf, offset, count);

	pvc_wa_allow_rc6(i915);

	return value;
}

static ssize_t
i915_sysfs_write(struct file *filp, struct kobject *kobj, struct bin_attribute
		 *attr, char *buf, loff_t offset, size_t count)
{
	ssize_t value;
	struct  sysfs_bin_ext_attr *ea = container_of(attr, struct
						      sysfs_bin_ext_attr, attr);
	struct device *kdev = kobj_to_dev(kobj);
	struct drm_i915_private *i915 = kdev_minor_to_i915(kdev);

	/* Wa_16015476723 & Wa_16015666671 */
	pvc_wa_disallow_rc6(i915);

	value = ea->i915_write(filp, kobj, attr, buf, offset, count);

	pvc_wa_allow_rc6(i915);

	return value;
}

#define I915_DPF_ERROR_ATTR_WR(_name, _mode, _read, _write, _size, _private,	\
				_sysfs_read, _sysfs_write)			\
	struct sysfs_bin_ext_attr dev_attr_##_name = {{				\
		.attr = { .name = __stringify(_name), .mode = _mode },		\
		.read   = _read,						\
		.write  = _write,						\
		.size   = _size,						\
		.mmap   = NULL,							\
		.private = (void *) _private					\
		}, _sysfs_read, _sysfs_write }

I915_DPF_ERROR_ATTR_WR(l3_parity, (S_IRUSR | S_IWUSR), i915_sysfs_read,
		       i915_sysfs_write, GEN7_L3LOG_SIZE, 0, i915_l3_read,
		       i915_l3_write);
I915_DPF_ERROR_ATTR_WR(l3_parity_slice_1, (S_IRUSR | S_IWUSR), i915_sysfs_read,
		       i915_sysfs_write, GEN7_L3LOG_SIZE, 1, i915_l3_read,
		       i915_l3_write);

static ssize_t
lmem_total_bytes_show(struct device *kdev, struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *i915 = kdev_minor_to_i915(kdev);
	struct intel_memory_region *mr;
	u64 value = 0;
	int id;

	for_each_memory_region(mr, i915, id)
		if (mr->type == INTEL_MEMORY_LOCAL)
			value += mr->total;

	return sysfs_emit(buf, "%llu\n", value);
}

static ssize_t
lmem_avail_bytes_show(struct device *kdev, struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *i915 = kdev_minor_to_i915(kdev);
	struct intel_memory_region *mr;
	u64 value = 0;
	int id;

	for_each_memory_region(mr, i915, id)
		if (mr->type == INTEL_MEMORY_LOCAL)
			value += atomic64_read(&mr->avail);

	return sysfs_emit(buf, "%llu\n", value);
}

#define I915_DEVICE_ATTR_RO(_name, _show) \
	struct i915_ext_attr dev_attr_##_name = \
	{ __ATTR(_name, 0444, i915_sysfs_show, NULL), _show, NULL}

#define I915_DEVICE_ATTR_WO(_name, _store) \
	struct i915_ext_attr dev_attr_##_name = \
	{ __ATTR(_name, 0200, NULL, i915_sysfs_store), NULL, _store}

#define I915_DEVICE_ATTR_RW(_name, _mode, _show, _store) \
	struct i915_ext_attr dev_attr_##_name = \
	{ __ATTR(_name, _mode, i915_sysfs_show, i915_sysfs_store), _show, _store}

static DEVICE_ATTR_RO(lmem_total_bytes);
static DEVICE_ATTR_RO(lmem_avail_bytes);

static const struct attribute *lmem_attrs[] = {
	&dev_attr_lmem_total_bytes.attr,
	&dev_attr_lmem_avail_bytes.attr,
	NULL
};

static int
__i915_set_mem_region_acct_limit(struct drm_i915_private *i915,
				 u32 index, u8 val)
{
	struct intel_memory_region *mem;
	int id;

	for_each_memory_region(mem, i915, id) {
		resource_size_t limit;
		int err = 0;

		if (mem->type != INTEL_MEMORY_LOCAL)
			continue;

		limit = min_t(resource_size_t,
			      mem->total, div_u64(val * mem->total, 100));

		spin_lock(&mem->acct_lock);
		if (limit >= mem->acct_user[index])
			mem->acct_limit[index] = limit - mem->acct_user[index];
		else
			err = -EINVAL;
		spin_unlock(&mem->acct_lock);
		if (err)
			return err;
	}

	return 0;
}

static int
i915_set_mem_region_acct_limit(struct drm_i915_private *i915, u32 index, u8 val)
{
	int ret;

	if (i915->mm.user_acct_limit[index] == val)
		return 0;

	i915_gem_drain_freed_objects(i915);

	ret = __i915_set_mem_region_acct_limit(i915, index, val);
	if (!ret)
		WRITE_ONCE(i915->mm.user_acct_limit[index], val);
	else
		__i915_set_mem_region_acct_limit(i915, index, i915->mm.user_acct_limit[index]);

	return ret;
}

static ssize_t alloc_limit_show(struct device *dev, char *buf, u32 index)
{
	struct drm_i915_private *i915 = kdev_minor_to_i915(dev);

	return sysfs_emit(buf, "%u\n", i915->mm.user_acct_limit[index]);
}

static void reset_alloc_limit(struct drm_i915_private *i915)
{
	memset(i915->mm.user_acct_limit, 0, sizeof(i915->mm.user_acct_limit));
}

static ssize_t
alloc_limit_store(struct device *dev, int id, const char *buf, size_t count)
{
	struct drm_i915_private *i915 = kdev_minor_to_i915(dev);
	int ret;
	u8 val;

	ret = kstrtou8(buf, 0, &val);
	if (ret)
		return ret;

	if (val >= 100)
		return -EINVAL;

	if (val + i915->mm.user_acct_limit[!id] > 100)
		return -EINVAL;

	if (!val) {
		reset_alloc_limit(i915);
		goto out;
	}

	ret = i915_set_mem_region_acct_limit(i915, id, val);
out:
	return ret ?: count;
}

static ssize_t
prelim_lmem_alloc_limit_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	return alloc_limit_store(dev, INTEL_MEMORY_OVERCOMMIT_LMEM,
				 buf, count);
}

static ssize_t
prelim_lmem_alloc_limit_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	return alloc_limit_show(dev, buf, INTEL_MEMORY_OVERCOMMIT_LMEM);
}

static ssize_t
prelim_sharedmem_alloc_limit_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	return alloc_limit_store(dev, INTEL_MEMORY_OVERCOMMIT_SHARED,
				 buf, count);
}

static ssize_t
prelim_sharedmem_alloc_limit_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	return alloc_limit_show(dev, buf, INTEL_MEMORY_OVERCOMMIT_SHARED);
}

static DEVICE_ATTR_RW(prelim_lmem_alloc_limit);
static DEVICE_ATTR_RW(prelim_sharedmem_alloc_limit);

static const struct attribute *alloc_limit_attrs[] = {
	&dev_attr_prelim_lmem_alloc_limit.attr,
	&dev_attr_prelim_sharedmem_alloc_limit.attr,
	NULL
};

#if IS_ENABLED(CONFIG_DRM_I915_CAPTURE_ERROR)

static ssize_t error_state_read(struct file *filp, struct kobject *kobj,
				struct bin_attribute *attr, char *buf,
				loff_t off, size_t count)
{

	struct device *kdev = kobj_to_dev(kobj);
	struct drm_i915_private *i915 = kdev_minor_to_i915(kdev);
	struct i915_gpu_coredump *gpu;
	ssize_t ret;

	gpu = i915_first_error_state(i915);
	if (IS_ERR(gpu)) {
		ret = PTR_ERR(gpu);
	} else if (gpu) {
		ret = i915_gpu_coredump_copy_to_buffer(gpu, buf, off, count);
		i915_gpu_coredump_put(gpu);
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

	drm_dbg(&dev_priv->drm, "Resetting error state\n");
	i915_reset_error_state(dev_priv);

	return count;
}

I915_DPF_ERROR_ATTR_WR(error, (S_IRUSR | S_IWUSR), i915_sysfs_read,
		       i915_sysfs_write, 0, 0, error_state_read,
		       error_state_write);

static void i915_setup_error_capture(struct device *kdev)
{
	if (sysfs_create_bin_file(&kdev->kobj, &dev_attr_error.attr))
		DRM_ERROR("error_state sysfs setup failed\n");
}

static void i915_teardown_error_capture(struct device *kdev)
{
	sysfs_remove_bin_file(&kdev->kobj, &dev_attr_error.attr);
}
#else
static void i915_setup_error_capture(struct device *kdev) {}
static void i915_teardown_error_capture(struct device *kdev) {}
#endif

static ssize_t prelim_uapi_version_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return sysfs_emit(buf, "%d.%d\n", PRELIM_UAPI_MAJOR, PRELIM_UAPI_MINOR);
}

static I915_DEVICE_ATTR_RO(prelim_uapi_version, prelim_uapi_version_show);

static ssize_t
prelim_csc_unique_id_show(struct device *kdev, struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *i915 = kdev_minor_to_i915(kdev);

	return sysfs_emit(buf, "%llx\n", RUNTIME_INFO(i915)->uid);
}

static DEVICE_ATTR_RO(prelim_csc_unique_id);

static ssize_t
prelim_lmem_max_bw_Mbps_show(struct device *dev, struct device_attribute *attr, char *buff)
{
	struct drm_i915_private *i915 = kdev_minor_to_i915(dev);
	u32 val;
	int err;

	err = snb_pcode_read_p(&i915->uncore, PCODE_MEMORY_CONFIG,
			       MEMORY_CONFIG_SUBCOMMAND_READ_MAX_BANDWIDTH,
			       0x0, &val);
	if (err)
		return err;

	return sysfs_emit(buff, "%u\n", val);
}

static I915_DEVICE_ATTR_RO(prelim_lmem_max_bw_Mbps, prelim_lmem_max_bw_Mbps_show);

static ssize_t i915_driver_error_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct device *kdev = kobj_to_dev(dev->kobj.parent);
	struct drm_i915_private *i915 = kdev_minor_to_i915(kdev);
	struct ext_attr *ea = container_of(attr, struct ext_attr, attr);

	if (GEM_WARN_ON(ea->id > ARRAY_SIZE(i915->errors)))
		return -ENOENT;

	return scnprintf(buf, PAGE_SIZE, "%lu\n", i915->errors[ea->id]);
}

static ssize_t
i915_sysfs_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t value;
	struct ext_attr *ea = container_of(attr, struct ext_attr, attr);
	struct device *kdev = kobj_to_dev(dev->kobj.parent);
	struct drm_i915_private *i915 = kdev_minor_to_i915(kdev);

	/* Wa_16015476723 & Wa_16015666671 */
	pvc_wa_disallow_rc6(i915);

	value = ea->i915_show(dev, attr, buf);

	pvc_wa_allow_rc6(i915);

	return value;
}

#define I915_DRIVER_SYSFS_ERROR_ATTR_RO(_name,  _id) \
	struct ext_attr dev_attr_##_name = \
	{ __ATTR(_name, 0444, i915_sysfs_id_show, NULL), (_id), i915_driver_error_show}

static I915_DRIVER_SYSFS_ERROR_ATTR_RO(driver_object_migration, I915_DRIVER_ERROR_OBJECT_MIGRATION);

static const struct attribute *i915_error_counter_attrs[] = {
	&dev_attr_driver_object_migration.attr.attr,
	NULL
};

static void i915_setup_error_counter(struct drm_i915_private *i915)
{
	struct device *kdev = i915->drm.primary->kdev;
	struct kobject *kobj;
	int ret;

	kobj = kobject_create_and_add("error_counter", &kdev->kobj);
	if (!kobj)
		goto err;

	ret = sysfs_create_files(kobj, i915_error_counter_attrs);
	if (ret)
		goto err;

	return;

err:
	drm_notice(&i915->drm, "Failed to create error_counter sysfs files at device level\n");
	kobject_put(kobj);
}

static struct kobject *i915_setup_gt_sysfs(struct kobject *parent)
{
	return kobject_create_and_add("gt", parent);
}

static ssize_t invalidate_lmem_mmaps_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buff, size_t count)
{
	struct drm_i915_private *i915 = kdev_minor_to_i915(dev);
	struct intel_memory_region *mem;
	ssize_t ret;
	bool val;
	int id;

	ret = kstrtobool(buff, &val);
	if (ret)
		return ret;

	if (!val)
		return -EINVAL;

	if (i915->invalidate_lmem_mmaps)
		return -EBUSY;

	if (!i915->quiesce_gpu) {
		drm_dbg(&i915->drm, "Invalidating LMEM mmaps is not allowed if GPU is unwedged\n");
		return -EPERM;
	}

	WRITE_ONCE(i915->invalidate_lmem_mmaps, val);
	ret = wait_var_event_interruptible(&i915->active_fault_handlers,
					   !atomic_read(&i915->active_fault_handlers));
	if (ret)
		return ret;

	for_each_memory_region(mem, i915, id) {
		struct intel_memory_region_link bookmark = {};
		struct intel_memory_region_link *pos;
		struct list_head *phases[] = {
			&mem->objects.purgeable,
			&mem->objects.list,
			NULL,
		}, **phase = phases;

		if (mem->type != INTEL_MEMORY_LOCAL)
			continue;

		spin_lock(&mem->objects.lock);
		do list_for_each_entry(pos, *phase, link) {
			struct drm_i915_gem_object *obj;

			if (!pos->mem)
				continue;

			obj = container_of(pos, struct drm_i915_gem_object, mm.region);
			i915_gem_object_get(obj);

			list_add(&bookmark.link, &pos->link);
			spin_unlock(&mem->objects.lock);

			i915_gem_object_lock(obj, NULL);
			i915_gem_object_release_mmap(obj);
			i915_gem_object_unlock(obj);
			i915_gem_object_put(obj);

			spin_lock(&mem->objects.lock);
			__list_del_entry(&bookmark.link);
			pos = &bookmark;
		} while (*++phase);
		spin_unlock(&mem->objects.lock);
	}

	return count;
}

static I915_DEVICE_ATTR_WO(invalidate_lmem_mmaps, invalidate_lmem_mmaps_store);

static ssize_t quiesce_gpu_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buff, size_t count)
{
	struct drm_i915_private *i915 = kdev_minor_to_i915(dev);
	struct intel_gt *gt = to_gt(i915);
	struct intel_engine_cs *engine;
	enum intel_engine_id id;
	intel_wakeref_t wakeref;
	unsigned int i;
	ssize_t ret;
	bool val;
	u8 retry = 2;

	ret = kstrtobool(buff, &val);
	if (ret)
		return ret;

	if (!val)
		return -EINVAL;

	if (i915->quiesce_gpu)
		return -EBUSY;

	/* Do not quiesce the GPU if there are active clients */
	while (!xa_empty(&i915->clients.xarray) && retry--) {
		if (!retry)
			return -EBUSY;

		rcu_barrier();
		flush_workqueue(system_wq);
	}

	wakeref = intel_runtime_pm_get(&i915->runtime_pm);
	for_each_gt(gt, i915, i) {
		/*
		 * Disable rc6 and render power gate to make sure GT and
		 * Render are awake for EUs Array and Scan Diagnostics
		 */
		intel_gt_pm_fini(gt);
		if (intel_gt_terminally_wedged(gt))
			continue;
		intel_gt_set_wedged(gt);
		intel_gt_retire_requests(gt);
		for_each_engine(engine, gt, id) {
			intel_engine_quiesce(engine);
		}
		GEM_BUG_ON(intel_gt_pm_is_awake(gt));
	}
	intel_runtime_pm_put(&i915->runtime_pm, wakeref);

	/* flush the scheduled jobs when clients were closed */
	rcu_barrier();
	flush_workqueue(system_wq);
	i915->drm.unplugged = true;
	WRITE_ONCE(i915->quiesce_gpu, val);

	return count;
}

static I915_DEVICE_ATTR_WO(quiesce_gpu, quiesce_gpu_store);

static const struct attribute *setup_quiesce_gpu_attrs[] = {
	&dev_attr_quiesce_gpu.attr.attr,
	&dev_attr_invalidate_lmem_mmaps.attr.attr,
	NULL
};

static void i915_setup_quiesce_gpu_sysfs(struct drm_i915_private *i915)
{
	struct device *kdev = i915->drm.primary->kdev;

	if (sysfs_create_files(&kdev->kobj, setup_quiesce_gpu_attrs))
		dev_err(kdev, "Failed to add sysfs files to setup quiesce GPU\n");
}

static ssize_t prelim_reset_all_gt_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct drm_i915_private *i915 = kdev_minor_to_i915(dev);
	struct intel_gt *gt;
	unsigned int id;
	bool val;
	int ret;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	if (!val)
		return count;

	for_each_gt(gt, i915, id) {
		int __ret = intel_gt_sysfs_reset(gt);

		/*
		 * try to reset as much as possible but return
		 * error if any of the GTs are wedged
		 */
		if (!ret && __ret)
			ret = __ret;
	}

	return ret ?: count;
}

DEVICE_ATTR_WO(prelim_reset_all_gt);

static ssize_t
i915_sysfs_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t value;
	struct i915_ext_attr *ea = container_of(attr, struct i915_ext_attr, attr);
	struct drm_i915_private *i915 = kdev_minor_to_i915(dev);

	/* Wa_16015476723 & Wa_16015666671 */
	pvc_wa_disallow_rc6(i915);

	value = ea->i915_show(dev, attr, buf);

	pvc_wa_allow_rc6(i915);

	return value;
}

static ssize_t
i915_sysfs_store(struct device *dev, struct device_attribute *attr, const char
		 *buf, size_t count)
{
	struct i915_ext_attr *ea = container_of(attr, struct i915_ext_attr, attr);
	struct drm_i915_private *i915 = kdev_minor_to_i915(dev);

	/* Wa_16015476723 & Wa_16015666671 */
	pvc_wa_disallow_rc6(i915);

	count = ea->i915_store(dev, attr, buf, count);

	pvc_wa_allow_rc6(i915);

	return count;
}

#if IS_ENABLED(CONFIG_DRM_I915_DEBUGGER)

static ssize_t enable_eu_debug_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *i915 = kdev_minor_to_i915(dev);

	return sysfs_emit(buf, "%u\n", i915->debuggers.enable_eu_debug);
}

static ssize_t enable_eu_debug_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct drm_i915_private *i915 = kdev_minor_to_i915(dev);
	bool enable;
	int ret;

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;

	ret = i915_debugger_enable(i915, enable);
	if (ret)
		return ret;

	return count;
}

static I915_DEVICE_ATTR_RW(prelim_enable_eu_debug, 0644, enable_eu_debug_show,
			   enable_eu_debug_store);

static void i915_setup_enable_eu_debug_sysfs(struct drm_i915_private *i915)
{
	struct device *kdev = i915->drm.primary->kdev;

	if (IS_SRIOV_VF(i915))
		return;

	if (sysfs_create_file(&kdev->kobj,
			      &dev_attr_prelim_enable_eu_debug.attr.attr))
		dev_warn(kdev, "Failed to add prelim_enable_eu_deubg sysfs param\n");
}

#else /* CONFIG_DRM_I915_DEBUGGER */

static void i915_setup_enable_eu_debug_sysfs(struct drm_i915_private *i915) {}

#endif /* CONFIG_DRM_I915_DEBUGGER */

static ssize_t iaf_socket_id_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct drm_i915_private *i915 = kdev_minor_to_i915(dev);

	return sysfs_emit(buf, "0x%x\n", i915->intel_iaf.socket_id);
}

static I915_DEVICE_ATTR_RO(iaf_socket_id, iaf_socket_id_show);

static const struct attribute *iaf_attrs[] = {
	&dev_attr_iaf_socket_id.attr.attr,
	NULL
};

/* Provide Address Translation Services Status: enabled/disabled */
static ssize_t
addr_trans_services_status_show(struct device *kdev,
				struct device_attribute *attr,
				char *buf)
{
	struct drm_i915_private *i915 = kdev_minor_to_i915(kdev);

	return sysfs_emit(buf, "%s\n",
			  i915_ats_enabled(i915) ? "Enabled" : "Disabled");
}

static ssize_t
global_pasid_counter_show(struct device *kdev,
			  struct device_attribute *attr,
			  char *buf)
{
	struct drm_i915_private *i915 = kdev_minor_to_i915(kdev);
	u64 global_pasid_counter = 0;

	global_pasid_counter = i915_global_pasid_counter(i915);
	return sysfs_emit(buf, "%llu\n", global_pasid_counter);
}

static DEVICE_ATTR(addr_trans_services_status, S_IRUGO,
		   addr_trans_services_status_show, NULL);
static DEVICE_ATTR(global_pasid_counter, S_IRUGO,
		   global_pasid_counter_show, NULL);

static const struct attribute *mode_b_attrs[] = {
	&dev_attr_addr_trans_services_status.attr,
	&dev_attr_global_pasid_counter.attr,
	NULL
};

void i915_setup_sysfs(struct drm_i915_private *dev_priv)
{
	struct device *kdev = dev_priv->drm.primary->kdev;
	int ret;

	if (sysfs_create_file(&kdev->kobj, &dev_attr_prelim_uapi_version.attr.attr))
		dev_err(kdev, "Failed adding prelim_uapi_version to sysfs\n");

	if (INTEL_INFO(dev_priv)->has_csc_uid) {
		ret = sysfs_create_file(&kdev->kobj, &dev_attr_prelim_csc_unique_id.attr);
		if (ret)
			drm_warn(&dev_priv->drm, "UID sysfs setup failed\n");
	}

	if (HAS_LMEM_MAX_BW(dev_priv)) {
		ret = sysfs_create_file(&kdev->kobj, &dev_attr_prelim_lmem_max_bw_Mbps.attr.attr);
		if (ret)
			drm_warn(&dev_priv->drm, "Failed to create maximum memory bandwidth sysfs file\n");
	}

	if (HAS_LMEM(dev_priv)) {
		ret = sysfs_create_files(&kdev->kobj, lmem_attrs);
		if (ret)
			DRM_ERROR("Local memory sysfs setup failed\n");
	}

	if (HAS_IAF(dev_priv)) {
		ret = sysfs_create_files(&kdev->kobj, iaf_attrs);
		if (ret)
			drm_warn(&dev_priv->drm, "PVC socket sysfs setup failed\n");
	}

	dev_priv->clients.root =
		kobject_create_and_add("clients", &kdev->kobj);
	if (!dev_priv->clients.root)
		drm_warn(&dev_priv->drm, "Per-client sysfs setup failed\n");

	if (HAS_L3_DPF(dev_priv)) {
		ret = device_create_bin_file(kdev, &dev_attr_l3_parity.attr);
		if (ret)
			drm_err(&dev_priv->drm,
				"l3 parity sysfs setup failed\n");

		if (NUM_L3_SLICES(dev_priv) > 1) {
			ret = device_create_bin_file(kdev,
						     &dev_attr_l3_parity_slice_1.attr);
			if (ret)
				drm_err(&dev_priv->drm,
					"l3 parity slice 1 setup failed\n");
		}
	}

	if (sysfs_create_file(&kdev->kobj, &dev_attr_prelim_reset_all_gt.attr))
		drm_warn(&dev_priv->drm,
			 "failed to create sysfs reset interface\n");

	dev_priv->sysfs_gt = i915_setup_gt_sysfs(&kdev->kobj);
	if (!dev_priv->sysfs_gt)
		drm_err(&dev_priv->drm,
			"failed to register GT sysfs directory\n");

	ret = sysfs_create_files(&kdev->kobj, alloc_limit_attrs);
	if (ret)
		drm_warn(&dev_priv->drm, "failed to create prelim_lmem/shared_alloc_limit sysfs entries\n");

	i915_sriov_sysfs_setup(dev_priv);

	i915_setup_error_capture(kdev);

	i915_setup_error_counter(dev_priv);

	intel_engines_add_sysfs(dev_priv);

	i915_setup_quiesce_gpu_sysfs(dev_priv);

	intel_mem_health_report_sysfs(dev_priv);

	if (i915_ats_enabled(dev_priv)) {
		ret = sysfs_create_files(&kdev->kobj, mode_b_attrs);
		if (ret)
			DRM_ERROR("Failed to setup Address Translation Services sysfs\n");
	}

	i915_setup_enable_eu_debug_sysfs(dev_priv);
}

void i915_teardown_sysfs(struct drm_i915_private *dev_priv)
{
	struct device *kdev = dev_priv->drm.primary->kdev;

	sysfs_remove_file(&kdev->kobj, &dev_attr_prelim_uapi_version.attr.attr);

	i915_teardown_error_capture(kdev);

	i915_sriov_sysfs_teardown(dev_priv);

	device_remove_bin_file(kdev,  &dev_attr_l3_parity_slice_1.attr);
	device_remove_bin_file(kdev,  &dev_attr_l3_parity.attr);

	if (dev_priv->clients.root)
		kobject_put(dev_priv->clients.root);

	kobject_put(dev_priv->sysfs_gt);
}
