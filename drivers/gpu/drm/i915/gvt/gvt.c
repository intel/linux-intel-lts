/*
 * Copyright(c) 2011-2016 Intel Corporation. All rights reserved.
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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Authors:
 *    Kevin Tian <kevin.tian@intel.com>
 *    Eddie Dong <eddie.dong@intel.com>
 *
 * Contributors:
 *    Niu Bing <bing.niu@intel.com>
 *    Zhi Wang <zhi.a.wang@intel.com>
 *
 */

#include <linux/types.h>
#include <xen/xen.h>
#include <linux/kthread.h>

#include "i915_drv.h"
#include "gvt.h"
#include <linux/vfio.h>
#include <linux/mdev.h>

struct intel_gvt_host intel_gvt_host;

static const char * const supported_hypervisors[] = {
	[INTEL_GVT_HYPERVISOR_XEN] = "XEN",
	[INTEL_GVT_HYPERVISOR_KVM] = "KVM",
	[INTEL_GVT_HYPERVISOR_ACRN] = "ACRN",
};

static struct intel_vgpu_type *intel_gvt_find_vgpu_type(struct intel_gvt *gvt,
		const char *name)
{
	int i;
	struct intel_vgpu_type *t;
	const char *driver_name = dev_driver_string(
			&gvt->dev_priv->drm.pdev->dev);

	for (i = 0; i < gvt->num_types; i++) {
		t = &gvt->types[i];
		if (!strncmp(t->name, name + strlen(driver_name) + 1,
			sizeof(t->name)))
			return t;
	}

	return NULL;
}

static ssize_t available_instances_show(struct kobject *kobj,
					struct device *dev, char *buf)
{
	struct intel_vgpu_type *type;
	unsigned int num = 0;
	void *gvt = kdev_to_i915(dev)->gvt;

	type = intel_gvt_find_vgpu_type(gvt, kobject_name(kobj));
	if (!type)
		num = 0;
	else
		num = type->avail_instance;

	return sprintf(buf, "%u\n", num);
}

static ssize_t device_api_show(struct kobject *kobj, struct device *dev,
		char *buf)
{
	return sprintf(buf, "%s\n", VFIO_DEVICE_API_PCI_STRING);
}

static ssize_t description_show(struct kobject *kobj, struct device *dev,
		char *buf)
{
	struct intel_vgpu_type *type;
	void *gvt = kdev_to_i915(dev)->gvt;

	type = intel_gvt_find_vgpu_type(gvt, kobject_name(kobj));
	if (!type)
		return 0;

	return sprintf(buf, "low_gm_size: %dMB\nhigh_gm_size: %dMB\n"
		       "fence: %d\nresolution: %s\n"
		       "weight: %d\n",
		       BYTES_TO_MB(type->low_gm_size),
		       BYTES_TO_MB(type->high_gm_size),
		       type->fence, vgpu_edid_str(type->resolution),
		       type->weight);
}

static MDEV_TYPE_ATTR_RO(available_instances);
static MDEV_TYPE_ATTR_RO(device_api);
static MDEV_TYPE_ATTR_RO(description);

static struct attribute *gvt_type_attrs[] = {
	&mdev_type_attr_available_instances.attr,
	&mdev_type_attr_device_api.attr,
	&mdev_type_attr_description.attr,
	NULL,
};

static struct attribute_group *gvt_vgpu_type_groups[] = {
	[0 ... NR_MAX_INTEL_VGPU_TYPES - 1] = NULL,
};

static bool intel_get_gvt_attrs(struct attribute ***type_attrs,
		struct attribute_group ***intel_vgpu_type_groups)
{
	*type_attrs = gvt_type_attrs;
	*intel_vgpu_type_groups = gvt_vgpu_type_groups;
	return true;
}

static bool intel_gvt_init_vgpu_type_groups(struct intel_gvt *gvt)
{
	int i, j;
	struct intel_vgpu_type *type;
	struct attribute_group *group;

	for (i = 0; i < gvt->num_types; i++) {
		type = &gvt->types[i];

		group = kzalloc(sizeof(struct attribute_group), GFP_KERNEL);
		if (WARN_ON(!group))
			goto unwind;

		group->name = type->name;
		group->attrs = gvt_type_attrs;
		gvt_vgpu_type_groups[i] = group;
	}

	return true;

unwind:
	for (j = 0; j < i; j++) {
		group = gvt_vgpu_type_groups[j];
		kfree(group);
	}

	return false;
}

static void intel_gvt_cleanup_vgpu_type_groups(struct intel_gvt *gvt)
{
	int i;
	struct attribute_group *group;

	for (i = 0; i < gvt->num_types; i++) {
		group = gvt_vgpu_type_groups[i];
		gvt_vgpu_type_groups[i] = NULL;
		kfree(group);
	}
}

static const struct intel_gvt_ops intel_gvt_ops = {
	.emulate_cfg_read = intel_vgpu_emulate_cfg_read,
	.emulate_cfg_write = intel_vgpu_emulate_cfg_write,
	.emulate_mmio_read = intel_vgpu_emulate_mmio_read,
	.emulate_mmio_write = intel_vgpu_emulate_mmio_write,
	.vgpu_create = intel_gvt_create_vgpu,
	.vgpu_destroy = intel_gvt_destroy_vgpu,
	.vgpu_release = intel_gvt_release_vgpu,
	.vgpu_reset = intel_gvt_reset_vgpu,
	.vgpu_activate = intel_gvt_activate_vgpu,
	.vgpu_deactivate = intel_gvt_deactivate_vgpu,
	.gvt_find_vgpu_type = intel_gvt_find_vgpu_type,
	.get_gvt_attrs = intel_get_gvt_attrs,
	.vgpu_query_plane = intel_vgpu_query_plane,
	.vgpu_get_dmabuf = intel_vgpu_get_dmabuf,
	.write_protect_handler = intel_vgpu_page_track_handler,
};

/**
 * intel_gvt_init_host - Load MPT modules and detect if we're running in host
 * @gvt: intel gvt device
 *
 * This function is called at the driver loading stage. If failed to find a
 * loadable MPT module or detect currently we're running in a VM, then GVT-g
 * will be disabled
 *
 * Returns:
 * Zero on success, negative error code if failed.
 *
 */
int intel_gvt_init_host(void)
{
	if (intel_gvt_host.initialized)
		return 0;

	/* Xen DOM U */
	if (xen_domain() && !xen_initial_domain())
		return -ENODEV;

	/* Try to load MPT modules for hypervisors */
	if (xen_initial_domain()) {
		/* In Xen dom0 */
		intel_gvt_host.mpt = try_then_request_module(
				symbol_get(xengt_mpt), "xengt");
		intel_gvt_host.hypervisor_type = INTEL_GVT_HYPERVISOR_XEN;
	} else {
#if IS_ENABLED(CONFIG_DRM_I915_GVT_KVMGT)
		/* not in Xen. Try KVMGT */
		intel_gvt_host.mpt = try_then_request_module(
				symbol_get(kvmgt_mpt), "kvmgt");
		intel_gvt_host.hypervisor_type = INTEL_GVT_HYPERVISOR_KVM;
#endif
		/* not in Xen. Try ACRN */
		intel_gvt_host.mpt = try_then_request_module(
				symbol_get(acrn_gvt_mpt), "acrn_gvt");
		intel_gvt_host.hypervisor_type = INTEL_GVT_HYPERVISOR_ACRN;
		printk("acrngt %s\n", intel_gvt_host.mpt?"found":"not found");
	}

	/* Fail to load MPT modules - bail out */
	if (!intel_gvt_host.mpt)
		return -EINVAL;

	gvt_dbg_core("Running with hypervisor %s in host mode\n",
			supported_hypervisors[intel_gvt_host.hypervisor_type]);

	intel_gvt_host.initialized = true;
	return 0;
}

static void init_device_info(struct intel_gvt *gvt)
{
	struct intel_gvt_device_info *info = &gvt->device_info;
	struct pci_dev *pdev = gvt->dev_priv->drm.pdev;

	info->max_support_vgpus = 8;
	info->cfg_space_size = PCI_CFG_SPACE_EXP_SIZE;
	info->mmio_size = 2 * 1024 * 1024;
	/* order of mmio size. assert(2^order == mmio_size) */
	info->mmio_size_order = 9;
	info->mmio_bar = 0;
	info->gtt_start_offset = 8 * 1024 * 1024;
	info->gtt_entry_size = 8;
	info->gtt_entry_size_shift = 3;
	info->gmadr_bytes_in_cmd = 8;
	info->max_surface_size = 36 * 1024 * 1024;
	info->msi_cap_offset = pdev->msi_cap;
}

static int gvt_service_thread(void *data)
{
	struct intel_gvt *gvt = (struct intel_gvt *)data;
	int ret;

	gvt_dbg_core("service thread start\n");

	while (!kthread_should_stop()) {
		ret = wait_event_interruptible(gvt->service_thread_wq,
				kthread_should_stop() || gvt->service_request);

		if (kthread_should_stop())
			break;

		if (WARN_ONCE(ret, "service thread is waken up by signal.\n"))
			continue;

		if (test_and_clear_bit(INTEL_GVT_REQUEST_EMULATE_VBLANK,
					(void *)&gvt->service_request))
			intel_gvt_emulate_vblank(gvt);

		if (test_bit(INTEL_GVT_REQUEST_SCHED,
				(void *)&gvt->service_request) ||
			test_bit(INTEL_GVT_REQUEST_EVENT_SCHED,
					(void *)&gvt->service_request)) {
			intel_gvt_schedule(gvt);
		}
	}

	return 0;
}

static void clean_service_thread(struct intel_gvt *gvt)
{
	kthread_stop(gvt->service_thread);
}

static int init_service_thread(struct intel_gvt *gvt)
{
	init_waitqueue_head(&gvt->service_thread_wq);

	gvt->service_thread = kthread_run(gvt_service_thread,
			gvt, "gvt_service_thread");
	if (IS_ERR(gvt->service_thread)) {
		gvt_err("fail to start service thread.\n");
		return PTR_ERR(gvt->service_thread);
	}
	return 0;
}

void intel_gvt_init_pipe_info(struct intel_gvt *gvt);

/*
 * When enabling multi-plane in DomU, an issue is that the PLANE_BUF_CFG
 * register cannot be updated dynamically, since Dom0 has no idea of the
 * plane information of DomU's planes, so here we statically allocate the
 * ddb entries for all the possible enabled planes.
 */
void intel_gvt_allocate_ddb(struct intel_gvt *gvt,
		struct skl_ddb_allocation *ddb, unsigned int active_crtcs)
{
	struct drm_i915_private *dev_priv = gvt->dev_priv;
	unsigned int pipe_size, ddb_size, plane_size, plane_cnt;
	u16 start, end;
	enum pipe pipe;
	enum plane_id plane;
	int i = 0;
	int num_active = hweight32(active_crtcs);

	if (!num_active)
		return;

	ddb_size = INTEL_INFO(dev_priv)->ddb_size;
	ddb_size -= 4; /* 4 blocks for bypass path allocation */
	pipe_size = ddb_size / num_active;

	memset(ddb, 0, sizeof(*ddb));
	for_each_pipe_masked(dev_priv, pipe, active_crtcs) {
		start = pipe_size * (i++);
		end = start + pipe_size;
		ddb->plane[pipe][PLANE_CURSOR].start = end - GVT_CURSOR_BLOCKS;
		ddb->plane[pipe][PLANE_CURSOR].end = end;

		plane_cnt = (INTEL_INFO(dev_priv)->num_sprites[pipe] + 1);
		plane_size = (pipe_size - GVT_CURSOR_BLOCKS) / plane_cnt;

		for_each_universal_plane(dev_priv, pipe, plane) {
			ddb->plane[pipe][plane].start = start +
				(plane * (pipe_size - GVT_CURSOR_BLOCKS) /
					plane_cnt);
			ddb->plane[pipe][plane].end =
				ddb->plane[pipe][plane].start + plane_size;
		}

		memcpy(&gvt->ddb, ddb, sizeof(*ddb));
	}
}

static int intel_gvt_init_vreg_pool(struct intel_gvt *gvt)
{
	int i = 0;
	const struct intel_gvt_device_info *info = &gvt->device_info;

	for (i = 0; i < GVT_MAX_VGPU; i++) {
		gvt->intel_gvt_vreg_pool[i] = (void *)__get_free_pages(
			GFP_KERNEL, info->mmio_size_order);
		if (!gvt->intel_gvt_vreg_pool[i])
			return -ENOMEM;
	}

	return 0;
}

static void intel_gvt_clean_vreg_pool(struct intel_gvt *gvt)
{
	int i = 0;
	const struct intel_gvt_device_info *info = &gvt->device_info;

	for (i = 0; i < GVT_MAX_VGPU && gvt->intel_gvt_vreg_pool[i]; i++)
		free_pages((unsigned long) gvt->intel_gvt_vreg_pool[i],
				info->mmio_size_order);
}

void *intel_gvt_allocate_vreg(struct intel_vgpu *vgpu)
{
	int id = vgpu->id - 1;
	struct intel_gvt *gvt = vgpu->gvt;

	if (id < 0 || id >= GVT_MAX_VGPU ||
		gvt->intel_gvt_vreg_pool[id] == NULL ||
		gvt->intel_gvt_vreg_allocated[id])
		return NULL;

	gvt->intel_gvt_vreg_allocated[id] = true;
	return gvt->intel_gvt_vreg_pool[id];
}

void intel_gvt_free_vreg(struct intel_vgpu *vgpu)
{
	int id = vgpu->id - 1;
	struct intel_gvt *gvt = vgpu->gvt;

	if (id < 0 || id >= GVT_MAX_VGPU ||
		gvt->intel_gvt_vreg_pool[id] == NULL ||
		!gvt->intel_gvt_vreg_allocated[id])
		return;
	gvt->intel_gvt_vreg_allocated[id] = false;
}

/**
 * intel_gvt_clean_device - clean a GVT device
 * @gvt: intel gvt device
 *
 * This function is called at the driver unloading stage, to free the
 * resources owned by a GVT device.
 *
 */
void intel_gvt_clean_device(struct drm_i915_private *dev_priv)
{
	struct intel_gvt *gvt = to_gvt(dev_priv);

	if (WARN_ON(!gvt))
		return;

	intel_gvt_clean_vreg_pool(gvt);
	intel_gvt_destroy_idle_vgpu(gvt->idle_vgpu);
	intel_gvt_hypervisor_host_exit(&dev_priv->drm.pdev->dev, gvt);
	intel_gvt_cleanup_vgpu_type_groups(gvt);
	intel_gvt_clean_vgpu_types(gvt);

	intel_gvt_debugfs_clean(gvt);
	clean_service_thread(gvt);
	intel_gvt_clean_cmd_parser(gvt);
	intel_gvt_clean_sched_policy(gvt);
	intel_gvt_clean_workload_scheduler(gvt);
	intel_gvt_clean_gtt(gvt);
	intel_gvt_clean_irq(gvt);
	intel_gvt_free_firmware(gvt);
	intel_gvt_clean_mmio_info(gvt);
	idr_destroy(&gvt->vgpu_idr);

	kfree(dev_priv->gvt);
	dev_priv->gvt = NULL;
}

#define BITS_PER_DOMAIN 4
#define MAX_PLANES_PER_DOMAIN 4
#define DOMAIN_PLANE_OWNER(owner, pipe, plane) \
		((((owner) >> (pipe) * BITS_PER_DOMAIN * MAX_PLANES_PER_DOMAIN) >>  \
		  BITS_PER_DOMAIN * (plane)) & 0xf)

/**
 * intel_gvt_init_device - initialize a GVT device
 * @dev_priv: drm i915 private data
 *
 * This function is called at the initialization stage, to initialize
 * necessary GVT components.
 *
 * Returns:
 * Zero on success, negative error code if failed.
 *
 */
int intel_gvt_init_device(struct drm_i915_private *dev_priv)
{
	struct intel_gvt *gvt;
	struct intel_vgpu *vgpu;
	int ret;

	/*
	 * Cannot initialize GVT device without intel_gvt_host gets
	 * initialized first.
	 */
	if (WARN_ON(!intel_gvt_host.initialized))
		return -EINVAL;

	if (WARN_ON(dev_priv->gvt))
		return -EEXIST;

	gvt = kzalloc(sizeof(struct intel_gvt), GFP_KERNEL);
	if (!gvt)
		return -ENOMEM;

	gvt_dbg_core("init gvt device\n");

	idr_init(&gvt->vgpu_idr);
	spin_lock_init(&gvt->scheduler.mmio_context_lock);
	mutex_init(&gvt->lock);
	mutex_init(&gvt->sched_lock);
	gvt->dev_priv = dev_priv;

	init_device_info(gvt);

	ret = intel_gvt_setup_mmio_info(gvt);
	if (ret)
		goto out_clean_idr;

	intel_gvt_init_engine_mmio_context(gvt);

	ret = intel_gvt_load_firmware(gvt);
	if (ret)
		goto out_clean_mmio_info;

	ret = intel_gvt_init_irq(gvt);
	if (ret)
		goto out_free_firmware;

	ret = intel_gvt_init_gtt(gvt);
	if (ret)
		goto out_clean_irq;

	ret = intel_gvt_init_workload_scheduler(gvt);
	if (ret)
		goto out_clean_gtt;

	ret = intel_gvt_init_sched_policy(gvt);
	if (ret)
		goto out_clean_workload_scheduler;

	ret = intel_gvt_init_cmd_parser(gvt);
	if (ret)
		goto out_clean_sched_policy;

	ret = init_service_thread(gvt);
	if (ret)
		goto out_clean_cmd_parser;

	ret = intel_gvt_init_vgpu_types(gvt);
	if (ret)
		goto out_clean_thread;

	ret = intel_gvt_init_vgpu_type_groups(gvt);
	if (ret == false) {
		gvt_err("failed to init vgpu type groups: %d\n", ret);
		goto out_clean_types;
	}

	intel_gvt_init_pipe_info(gvt);

	ret = intel_gvt_hypervisor_host_init(&dev_priv->drm.pdev->dev, gvt,
				&intel_gvt_ops);
	if (ret) {
		gvt_err("failed to register gvt-g host device: %d\n", ret);
		goto out_clean_types;
	}

	vgpu = intel_gvt_create_idle_vgpu(gvt);
	if (IS_ERR(vgpu)) {
		ret = PTR_ERR(vgpu);
		gvt_err("failed to create idle vgpu\n");
		goto out_clean_types;
	}
	gvt->idle_vgpu = vgpu;

	ret = intel_gvt_init_vreg_pool(gvt);
	if (ret) {
		gvt_err("failed to init vreg pool\n");
		goto out_clean_vreg;
	}

	ret = intel_gvt_debugfs_init(gvt);
	if (ret)
		gvt_err("debugfs registeration failed, go on.\n");

	dev_priv->gvt = gvt;

	if (i915_modparams.avail_planes_per_pipe) {
		unsigned long long domain_plane_owners;
		int plane;
		enum pipe pipe;

		/*
		 * Each nibble represents domain id
		 * ids can be from 0-F. 0 for Dom0, 1,2,3...0xF for DomUs
		 * plane_owner[i] holds the id of the domain that owns it,eg:0,1,2 etc
		 */
		domain_plane_owners = i915_modparams.domain_plane_owners;
		for_each_pipe(dev_priv, pipe) {
			for_each_universal_plane(dev_priv, pipe, plane) {
				gvt->pipe_info[pipe].plane_owner[plane] =
					DOMAIN_PLANE_OWNER(domain_plane_owners, pipe, plane);
			}
		}
	}

	gvt_dbg_core("gvt device initialization is done\n");
	return 0;

out_clean_vreg:
	intel_gvt_clean_vreg_pool(gvt);
out_clean_types:
	intel_gvt_clean_vgpu_types(gvt);
out_clean_thread:
	clean_service_thread(gvt);
out_clean_cmd_parser:
	intel_gvt_clean_cmd_parser(gvt);
out_clean_sched_policy:
	intel_gvt_clean_sched_policy(gvt);
out_clean_workload_scheduler:
	intel_gvt_clean_workload_scheduler(gvt);
out_clean_gtt:
	intel_gvt_clean_gtt(gvt);
out_clean_irq:
	intel_gvt_clean_irq(gvt);
out_free_firmware:
	intel_gvt_free_firmware(gvt);
out_clean_mmio_info:
	intel_gvt_clean_mmio_info(gvt);
out_clean_idr:
	idr_destroy(&gvt->vgpu_idr);
	kfree(gvt);
	return ret;
}

int gvt_dom0_ready(struct drm_i915_private *dev_priv)
{
	if (!intel_gvt_active(dev_priv))
		return 0;

	return intel_gvt_hypervisor_dom0_ready();
}

#if IS_ENABLED(CONFIG_DRM_I915_GVT_KVMGT)
MODULE_SOFTDEP("pre: kvmgt");
#endif
