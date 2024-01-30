/* i915_drv.c -- i830,i845,i855,i865,i915 driver -*- linux-c -*-
 */
/*
 *
 * Copyright 2003 Tungsten Graphics, Inc., Cedar Park, Texas.
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * IN NO EVENT SHALL TUNGSTEN GRAPHICS AND/OR ITS SUPPLIERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <linux/aer.h>
#include <linux/acpi.h>
#include <linux/device.h>
#if !IS_ENABLED(CONFIG_AUXILIARY_BUS)
#include <linux/mfd/core.h>
#endif
#include <linux/module.h>
#include <linux/oom.h>
#include <linux/pci.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/pnp.h>
#include <linux/slab.h>
#include <linux/string_helpers.h>
#include <linux/vga_switcheroo.h>
#include <linux/vt.h>

#include <drm/drm_aperture.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_managed.h>
#include <drm/drm_probe_helper.h>

#include "display/intel_acpi.h"
#include "display/intel_bw.h"
#include "display/intel_cdclk.h"
#include "display/intel_display_types.h"
#include "display/intel_dmc.h"
#include "display/intel_dp.h"
#include "display/intel_dpt.h"
#include "display/intel_fbdev.h"
#include "display/intel_hotplug.h"
#include "display/intel_overlay.h"
#include "display/intel_pch_refclk.h"
#include "display/intel_pipe_crc.h"
#include "display/intel_pps.h"
#include "display/intel_sprite.h"
#include "display/intel_vga.h"

#include "gem/i915_gem_context.h"
#include "gem/i915_gem_create.h"
#include "gem/i915_gem_dmabuf.h"
#include "gem/i915_gem_ioctls.h"
#include "gem/i915_gem_mman.h"
#include "gem/i915_gem_pm.h"
#include "gem/i915_gem_vm_bind.h"
#include "gt/intel_clos.h"
#include "gt/intel_gpu_commands.h"
#include "gt/intel_gt.h"
#include "gt/intel_gt_pm.h"
#include "gt/intel_gt_regs.h"
#include "gt/intel_rc6.h"
#include "gt/iov/intel_iov.h"
#include "gt/iov/intel_iov_provisioning.h"

#include "pxp/intel_pxp_pm.h"

#include "spi/intel_spi.h"

#include "i915_debugfs.h"
#include "i915_driver.h"
#include "i915_drm_client.h"
#include "i915_drv.h"
#include "i915_getparam.h"
#include "i915_hwmon.h"
#include "i915_ioc32.h"
#include "i915_ioctl.h"
#include "i915_irq.h"
#include "i915_memcpy.h"
#include "i915_pci.h"
#include "i915_perf.h"
#include "i915_perf_stall_cntr.h"
#include "i915_query.h"
#include "i915_sriov.h"
#include "i915_suspend.h"
#include "i915_svm.h"
#include "i915_switcheroo.h"
#include "i915_sysfs.h"
#include "i915_sysrq.h"
#include "i915_utils.h"
#include "i915_vgpu.h"
#include "i915_debugger.h"
#include "intel_dram.h"
#include "intel_gvt.h"
#include "intel_iaf.h"
#include "intel_memory_region.h"
#include "intel_pci_config.h"
#include "intel_pcode.h"
#include "intel_pm.h"
#include "intel_vsec.h"
#include "pvc_ras.h"
#include "vlv_suspend.h"
#include "i915_addr_trans_svc.h"

static const struct drm_driver i915_drm_driver;

static void i915_release_bridge_dev(struct drm_device *dev,
				    void *bridge)
{
	pci_dev_put(bridge);
}

static const char *i915_driver_errors_to_str[] = {
	[I915_DRIVER_ERROR_OBJECT_MIGRATION] = "OBJECT MIGRATION",
};

void i915_silent_driver_error(struct drm_i915_private *i915,
			      const enum i915_driver_errors error)
{
	GEM_BUG_ON(error >= ARRAY_SIZE(i915->errors));
	WRITE_ONCE(i915->errors[error],
		   READ_ONCE(i915->errors[error]) + 1);
}

void i915_log_driver_error(struct drm_i915_private *i915,
			   const enum i915_driver_errors error,
			   const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	i915_silent_driver_error(i915, error);

	va_start(args, fmt);
	vaf.fmt = fmt;
	vaf.va = &args;

	BUILD_BUG_ON(ARRAY_SIZE(i915_driver_errors_to_str) !=
		     I915_DRIVER_ERROR_COUNT);

	drm_err_ratelimited(&i915->drm, "[%s] %pV",
			    i915_driver_errors_to_str[error], &vaf);

	va_end(args);
}

bool i915_save_pci_state(struct pci_dev *pdev)
{
	struct drm_i915_private *i915 = pci_get_drvdata(pdev);

	if (pci_save_state(pdev))
		return false;

	kfree(i915->pci_state);

	i915->pci_state = pci_store_saved_state(pdev);

	if (!i915->pci_state) {
		drm_err(&i915->drm, "Failed to store PCI saved state\n");
		return false;
	}

	return true;
}

void i915_load_pci_state(struct pci_dev *pdev)
{
	struct drm_i915_private *i915 = pci_get_drvdata(pdev);
	int ret;

	if (!i915->pci_state)
		return;

	ret = pci_load_saved_state(pdev, i915->pci_state);
	if (!ret) {
		pci_restore_state(pdev);
	} else {
		drm_warn(&i915->drm, "Failed to load PCI state, err:%d\n", ret);
	}
}

static int i915_get_bridge_dev(struct drm_i915_private *dev_priv)
{
	int domain = pci_domain_nr(to_pci_dev(dev_priv->drm.dev)->bus);

	dev_priv->bridge_dev =
		pci_get_domain_bus_and_slot(domain, 0, PCI_DEVFN(0, 0));
	if (!dev_priv->bridge_dev) {
		drm_err(&dev_priv->drm, "bridge device not found\n");
		return -EIO;
	}

	return drmm_add_action_or_reset(&dev_priv->drm, i915_release_bridge_dev,
					dev_priv->bridge_dev);
}

/* Allocate space for the MCH regs if needed, return nonzero on error */
static int
intel_alloc_mchbar_resource(struct drm_i915_private *dev_priv)
{
	int reg = GRAPHICS_VER(dev_priv) >= 4 ? MCHBAR_I965 : MCHBAR_I915;
	u32 temp_lo, temp_hi = 0;
	u64 mchbar_addr;
	int ret;

	if (GRAPHICS_VER(dev_priv) >= 4)
		pci_read_config_dword(dev_priv->bridge_dev, reg + 4, &temp_hi);
	pci_read_config_dword(dev_priv->bridge_dev, reg, &temp_lo);
	mchbar_addr = ((u64)temp_hi << 32) | temp_lo;

	/* If ACPI doesn't have it, assume we need to allocate it ourselves */
#ifdef CONFIG_PNP
	if (mchbar_addr &&
	    pnp_range_reserved(mchbar_addr, mchbar_addr + MCHBAR_SIZE))
		return 0;
#endif

	/* Get some space for it */
	dev_priv->mch_res.name = "i915 MCHBAR";
	dev_priv->mch_res.flags = IORESOURCE_MEM;
	ret = pci_bus_alloc_resource(dev_priv->bridge_dev->bus,
				     &dev_priv->mch_res,
				     MCHBAR_SIZE, MCHBAR_SIZE,
				     PCIBIOS_MIN_MEM,
				     0, pcibios_align_resource,
				     dev_priv->bridge_dev);
	if (ret) {
		drm_dbg(&dev_priv->drm, "failed bus alloc: %d\n", ret);
		dev_priv->mch_res.start = 0;
		return ret;
	}

	if (GRAPHICS_VER(dev_priv) >= 4)
		pci_write_config_dword(dev_priv->bridge_dev, reg + 4,
				       upper_32_bits(dev_priv->mch_res.start));

	pci_write_config_dword(dev_priv->bridge_dev, reg,
			       lower_32_bits(dev_priv->mch_res.start));
	return 0;
}

/* Setup MCHBAR if possible, return true if we should disable it again */
static void
intel_setup_mchbar(struct drm_i915_private *dev_priv)
{
	int mchbar_reg = GRAPHICS_VER(dev_priv) >= 4 ? MCHBAR_I965 : MCHBAR_I915;
	u32 temp;
	bool enabled;

	if (IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv))
		return;

	dev_priv->mchbar_need_disable = false;

	if (IS_I915G(dev_priv) || IS_I915GM(dev_priv)) {
		pci_read_config_dword(dev_priv->bridge_dev, DEVEN, &temp);
		enabled = !!(temp & DEVEN_MCHBAR_EN);
	} else {
		pci_read_config_dword(dev_priv->bridge_dev, mchbar_reg, &temp);
		enabled = temp & 1;
	}

	/* If it's already enabled, don't have to do anything */
	if (enabled)
		return;

	if (intel_alloc_mchbar_resource(dev_priv))
		return;

	dev_priv->mchbar_need_disable = true;

	/* Space is allocated or reserved, so enable it. */
	if (IS_I915G(dev_priv) || IS_I915GM(dev_priv)) {
		pci_write_config_dword(dev_priv->bridge_dev, DEVEN,
				       temp | DEVEN_MCHBAR_EN);
	} else {
		pci_read_config_dword(dev_priv->bridge_dev, mchbar_reg, &temp);
		pci_write_config_dword(dev_priv->bridge_dev, mchbar_reg, temp | 1);
	}
}

static void
intel_teardown_mchbar(struct drm_i915_private *dev_priv)
{
	int mchbar_reg = GRAPHICS_VER(dev_priv) >= 4 ? MCHBAR_I965 : MCHBAR_I915;

	if (dev_priv->mchbar_need_disable) {
		if (IS_I915G(dev_priv) || IS_I915GM(dev_priv)) {
			u32 deven_val;

			pci_read_config_dword(dev_priv->bridge_dev, DEVEN,
					      &deven_val);
			deven_val &= ~DEVEN_MCHBAR_EN;
			pci_write_config_dword(dev_priv->bridge_dev, DEVEN,
					       deven_val);
		} else {
			u32 mchbar_val;

			pci_read_config_dword(dev_priv->bridge_dev, mchbar_reg,
					      &mchbar_val);
			mchbar_val &= ~1;
			pci_write_config_dword(dev_priv->bridge_dev, mchbar_reg,
					       mchbar_val);
		}
	}

	if (dev_priv->mch_res.start)
		release_resource(&dev_priv->mch_res);
}

static int i915_workqueues_init(struct drm_i915_private *dev_priv)
{
	/*
	 * The i915 workqueue is primarily used for batched retirement of
	 * requests (and thus managing bo) once the task has been completed
	 * by the GPU. i915_retire_requests() is called directly when we
	 * need high-priority retirement, such as waiting for an explicit
	 * bo.
	 *
	 * It is also used for periodic low-priority events, such as
	 * idle-timers and recording error state.
	 */
	dev_priv->wq = alloc_workqueue("%s", WQ_UNBOUND, 0, "i915");
	if (dev_priv->wq == NULL)
		goto out_err;

	dev_priv->sched = i915_sched_engine_create(3);
	if (!dev_priv->sched)
		goto out_free_wq;
	dev_priv->sched->private_data = dev_priv->wq;

	dev_priv->mm.wq = alloc_workqueue("%s", WQ_UNBOUND, 0, "i915-smem");
	if (dev_priv->mm.wq == NULL)
		goto out_free_sched;

	dev_priv->mm.sched = i915_sched_engine_create(4);
	if (!dev_priv->mm.sched)
		goto out_free_mm_wq;
	dev_priv->mm.sched->private_data = dev_priv->mm.wq;

	dev_priv->hotplug.dp_wq = alloc_ordered_workqueue("i915-dp", 0);
	if (dev_priv->hotplug.dp_wq == NULL)
		goto out_free_mm_sched;

	return 0;

out_free_mm_sched:
	i915_sched_engine_put(dev_priv->mm.sched);
out_free_mm_wq:
	destroy_workqueue(dev_priv->mm.wq);
out_free_sched:
	i915_sched_engine_put(dev_priv->sched);
out_free_wq:
	destroy_workqueue(dev_priv->wq);
out_err:
	drm_err(&dev_priv->drm, "Failed to allocate workqueues.\n");
	return -ENOMEM;
}

static void i915_workqueues_cleanup(struct drm_i915_private *dev_priv)
{
	destroy_workqueue(dev_priv->hotplug.dp_wq);

	i915_sched_engine_put(dev_priv->mm.sched);
	destroy_workqueue(dev_priv->mm.wq);

	i915_sched_engine_put(dev_priv->sched);
	destroy_workqueue(dev_priv->wq);
}

/*
 * We don't keep the workarounds for pre-production hardware, so we expect our
 * driver to fail on these machines in one way or another. A little warning on
 * dmesg may help both the user and the bug triagers.
 *
 * Our policy for removing pre-production workarounds is to keep the
 * current gen workarounds as a guide to the bring-up of the next gen
 * (workarounds have a habit of persisting!). Anything older than that
 * should be removed along with the complications they introduce.
 */
static void intel_detect_preproduction_hw(struct drm_i915_private *dev_priv)
{
	bool pre = false;

	pre |= IS_HSW_EARLY_SDV(dev_priv);
	pre |= IS_SKYLAKE(dev_priv) && INTEL_REVID(dev_priv) < 0x6;
	pre |= IS_BROXTON(dev_priv) && INTEL_REVID(dev_priv) < 0xA;
	pre |= IS_KABYLAKE(dev_priv) && INTEL_REVID(dev_priv) < 0x1;
	pre |= IS_GEMINILAKE(dev_priv) && INTEL_REVID(dev_priv) < 0x3;
	pre |= IS_ICELAKE(dev_priv) && INTEL_REVID(dev_priv) < 0x7;
	pre |= IS_TIGERLAKE(dev_priv) && INTEL_REVID(dev_priv) < 0x1;
	pre |= IS_DG1(dev_priv) && INTEL_REVID(dev_priv) < 0x1;
	pre |= IS_PONTEVECCHIO(dev_priv) &&
		(IS_PVC_CT_STEP(dev_priv, STEP_A0, STEP_B0) ||
		 IS_PVC_BD_STEP(dev_priv, STEP_A0, STEP_B0));

	if (pre) {
		drm_err(&dev_priv->drm, "This is a pre-production stepping. "
			  "It may not be fully functional.\n");
		add_taint(TAINT_MACHINE_CHECK, LOCKDEP_STILL_OK);
	}
}

static void sanitize_gpu(struct drm_i915_private *i915)
{
	if (!INTEL_INFO(i915)->gpu_reset_clobbers_display) {
		struct intel_gt *gt;
		unsigned int i;

		for_each_gt(gt, i915, i)
			__intel_gt_reset(gt, ALL_ENGINES);
	}
}

#define IP_VER_READ(offset, ri_prefix) \
	addr = pci_iomap_range(pdev, 0, offset, sizeof(u32)); \
	if (drm_WARN_ON(&i915->drm, !addr)) { \
		/* Fall back to whatever was in the device info */ \
		RUNTIME_INFO(i915)->ri_prefix.ver = INTEL_INFO(i915)->ri_prefix.ver; \
		RUNTIME_INFO(i915)->ri_prefix.rel = INTEL_INFO(i915)->ri_prefix.rel; \
		goto ri_prefix##done; \
	} \
	\
	ver = ioread32(addr); \
	pci_iounmap(pdev, addr); \
	\
	RUNTIME_INFO(i915)->ri_prefix.ver = REG_FIELD_GET(GMD_ID_ARCH_MASK, ver); \
	RUNTIME_INFO(i915)->ri_prefix.rel = REG_FIELD_GET(GMD_ID_RELEASE_MASK, ver); \
	RUNTIME_INFO(i915)->ri_prefix.step = REG_FIELD_GET(GMD_ID_STEP, ver); \
	\
	/* Sanity check against expected versions from device info */ \
	if (RUNTIME_INFO(i915)->ri_prefix.ver != INTEL_INFO(i915)->ri_prefix.ver || \
	    RUNTIME_INFO(i915)->ri_prefix.rel < INTEL_INFO(i915)->ri_prefix.rel) \
		drm_dbg(&i915->drm, \
			"Hardware reports " #ri_prefix " IP version %u.%u but minimum expected is %u.%u\n", \
			RUNTIME_INFO(i915)->ri_prefix.ver, \
			RUNTIME_INFO(i915)->ri_prefix.rel, \
			INTEL_INFO(i915)->ri_prefix.ver, \
			INTEL_INFO(i915)->ri_prefix.rel); \
ri_prefix##done:

/**
 * intel_ipver_early_init - setup IP version values
 * @dev_priv: device private
 *
 * Setup the graphics version for the current device.  This must be done before
 * any code that performs checks on GRAPHICS_VER or DISPLAY_VER, so this
 * function should be called very early in the driver initialization sequence.
 *
 * Regular MMIO access is not yet setup at the point this function is called so
 * we peek at the appropriate MMIO offset directly.  The GMD_ID register is
 * part of an 'always on' power well by design, so we don't need to worry about
 * forcewake while reading it.
 */
static void intel_ipver_early_init(struct drm_i915_private *i915,
				   const struct intel_device_info *devinfo)
{
	struct pci_dev *pdev = to_pci_dev(i915->drm.dev);
	void __iomem *addr;
	u32 ver = 0;

	if (!HAS_GMD_ID(i915)) {
		drm_WARN_ON(&i915->drm, INTEL_INFO(i915)->graphics.ver > 12);

		RUNTIME_INFO(i915)->graphics.ver = INTEL_INFO(i915)->graphics.ver;
		RUNTIME_INFO(i915)->graphics.rel = INTEL_INFO(i915)->graphics.rel;
		/* media ver = graphics ver for older platforms */
		RUNTIME_INFO(i915)->media.ver = INTEL_INFO(i915)->graphics.ver;
		RUNTIME_INFO(i915)->media.rel = INTEL_INFO(i915)->graphics.rel;
		RUNTIME_INFO(i915)->display.ver = INTEL_INFO(i915)->display.ver;
		RUNTIME_INFO(i915)->display.rel = INTEL_INFO(i915)->display.rel;
		return;
	}

	/* VF can't access IPVER registers directly */
	if (IS_SRIOV_VF(i915)) {
		/* 14018060378 not ready yet, use hardcoded values from INTEL_INFO */
		drm_info(&i915->drm, "Beware, driver is using hardcoded IPVER values!\n");
		RUNTIME_INFO(i915)->graphics.ver = INTEL_INFO(i915)->graphics.ver;
		RUNTIME_INFO(i915)->graphics.rel = INTEL_INFO(i915)->graphics.rel;
		RUNTIME_INFO(i915)->media.ver = INTEL_INFO(i915)->media.ver;
		RUNTIME_INFO(i915)->media.rel = INTEL_INFO(i915)->media.rel;
		RUNTIME_INFO(i915)->display.ver = INTEL_INFO(i915)->display.ver;
		RUNTIME_INFO(i915)->display.rel = INTEL_INFO(i915)->display.rel;
		return;
	}

	IP_VER_READ(i915_mmio_reg_offset(GMD_ID_GRAPHICS), graphics);
	/* Wa_22012778468:mtl */
	if (ver == 0x0 && devinfo->platform == INTEL_METEORLAKE) {
		RUNTIME_INFO(i915)->graphics.ver = 12;
		RUNTIME_INFO(i915)->graphics.rel = 70;
	}
	IP_VER_READ(i915_mmio_reg_offset(GMD_ID_DISPLAY), display);
	IP_VER_READ(MTL_MEDIA_GSI_BASE + i915_mmio_reg_offset(GMD_ID_GRAPHICS),
		    media);
}

static void __release_bars(struct pci_dev *pdev)
{
	int resno;

	for (resno = PCI_STD_RESOURCES; resno < PCI_STD_RESOURCE_END; resno++) {
		if (pci_resource_len(pdev, resno))
			pci_release_resource(pdev, resno);
	}
#ifdef CONFIG_PCI_IOV
	for (resno = PCI_IOV_RESOURCES; resno < PCI_IOV_RESOURCE_END; resno++) {
		if (pci_resource_len(pdev, resno))
			pci_release_resource(pdev, resno);
	}
#endif
}

static void
__resize_bar(struct drm_i915_private *i915, int resno, resource_size_t size)
{
	struct pci_dev *pdev = to_pci_dev(i915->drm.dev);
	int bar_size = pci_rebar_bytes_to_size(size);
	int ret;

	__release_bars(pdev);

	ret = pci_resize_resource(pdev, resno, bar_size);
	if (ret) {
		drm_info(&i915->drm, "Failed to resize BAR%d to %dM (%pe)\n",
			 resno, 1 << bar_size, ERR_PTR(ret));
		return;
	}

	drm_info(&i915->drm, "BAR%d resized to %dM\n", resno, 1 << bar_size);
}

/* BAR size starts from 1MB - 2^20 */
#define BAR_SIZE_SHIFT 20
static resource_size_t
__lmem_rebar_size(struct drm_i915_private *i915, int resno)
{
	struct pci_dev *pdev = to_pci_dev(i915->drm.dev);
	u32 rebar = pci_rebar_get_possible_sizes(pdev, resno);
	resource_size_t size;

	if (!rebar)
		return 0;

	size = 1ULL << (__fls(rebar) + BAR_SIZE_SHIFT);

	if (size <= pci_resource_len(pdev, resno))
		return 0;

	return size;
}

/**
 * i915_resize_lmem_bar - resize local memory BAR
 * @i915: device private
 *
 * This function will attempt to resize LMEM bar to make all memory accessible.
 * Whether it will be successful depends on both device and platform
 * capabilities. Any errors are non-critical, even if resize fails, we go back
 * to the previous configuration.
 */
static void i915_resize_lmem_bar(struct drm_i915_private *i915)
{
	struct pci_dev *pdev = to_pci_dev(i915->drm.dev);
	struct pci_bus *root = pdev->bus;
	struct resource *root_res;
	resource_size_t rebar_size;
	u32 pci_cmd;
	int i;

	if (!i915_pci_resource_valid(pdev, GEN12_LMEM_BAR)) {
		drm_warn(&i915->drm, "Can't resize LMEM BAR - BAR not valid\n");
		return;
	}

	rebar_size = __lmem_rebar_size(i915, GEN12_LMEM_BAR);

	if (!rebar_size)
		return;

	/* Find out if root bus contains 64bit memory addressing */
	while (root->parent)
		root = root->parent;

	pci_bus_for_each_resource(root, root_res, i) {
		if (root_res &&
				root_res->flags & (IORESOURCE_MEM | IORESOURCE_MEM_64) &&
				root_res->start > 0x100000000ull)
			break;
	}

	/* pci_resize_resource will fail anyways */
	if (!root_res) {
		drm_info(&i915->drm,
				"Can't resize LMEM BAR - platform support is missing\n");
		return;
	}

	/* First disable PCI memory decoding references */
	pci_read_config_dword(pdev, PCI_COMMAND, &pci_cmd);
	pci_write_config_dword(pdev, PCI_COMMAND,
			       pci_cmd & ~PCI_COMMAND_MEMORY);

	__resize_bar(i915, GEN12_LMEM_BAR, rebar_size);

	pci_assign_unassigned_bus_resources(pdev->bus);
	pci_write_config_dword(pdev, PCI_COMMAND, pci_cmd);
}

/**
 * i915_driver_early_probe - setup state not requiring device access
 * @dev_priv: device private
 * @ent: PCI device info entry matched
 *
 * Initialize everything that is a "SW-only" state, that is state not
 * requiring accessing the device or exposing the driver via kernel internal
 * or userspace interfaces. Example steps belonging here: lock initialization,
 * system memory allocation, setting up device specific attributes and
 * function hooks not requiring accessing the device.
 *
 * GRAPHICS_VER, DISPLAY_VER, etc. are not yet usable at this point.  For
 */
static int i915_driver_early_probe(struct drm_i915_private *dev_priv,
				   const struct intel_device_info *devinfo)
{
	int ret = 0;

	if (i915_inject_probe_failure(dev_priv))
		return -ENODEV;

	intel_device_info_subplatform_init(dev_priv);
	intel_step_init(dev_priv);

	intel_uncore_mmio_debug_init_early(dev_priv);

	spin_lock_init(&dev_priv->irq_lock);
	spin_lock_init(&dev_priv->gpu_error.lock);
	mutex_init(&dev_priv->backlight_lock);

	mutex_init(&dev_priv->sb_lock);
	cpu_latency_qos_add_request(&dev_priv->sb_qos, PM_QOS_DEFAULT_VALUE);

	mutex_init(&dev_priv->audio.mutex);
	mutex_init(&dev_priv->wm.wm_mutex);
	mutex_init(&dev_priv->pps_mutex);
	mutex_init(&dev_priv->hdcp_comp_mutex);
	mutex_init(&dev_priv->svm_init_mutex);

	i915_debugger_init(dev_priv);

	i915_memcpy_init_early(dev_priv);
	intel_runtime_pm_init_early(&dev_priv->runtime_pm);

	ret = i915_workqueues_init(dev_priv);
	if (ret < 0)
		return ret;

	ret = vlv_suspend_init(dev_priv);
	if (ret < 0)
		goto err_workqueues;

	ret = intel_root_gt_init_early(dev_priv);
	if (ret < 0)
		goto err_rootgt;

	i915_drm_clients_init(&dev_priv->clients, dev_priv);

	i915_gem_init_early(dev_priv);

	/* This must be called before any calls to HAS_PCH_* */
	intel_detect_pch(dev_priv);

	intel_pm_setup(dev_priv);
	ret = intel_power_domains_init(dev_priv);
	if (ret < 0)
		goto err_gem;
	intel_irq_init(dev_priv);
	intel_init_display_hooks(dev_priv);
	intel_init_clock_gating_hooks(dev_priv);

	intel_iaf_init_early(dev_priv);

	intel_detect_preproduction_hw(dev_priv);
	init_waitqueue_head(&dev_priv->user_fence_wq);

	return 0;

err_gem:
	i915_gem_cleanup_early(dev_priv);
	intel_gt_driver_late_release_all(dev_priv);
err_rootgt:
	i915_drm_clients_fini(&dev_priv->clients);
	vlv_suspend_cleanup(dev_priv);
err_workqueues:
	i915_workqueues_cleanup(dev_priv);
	return ret;
}

#define PCI_SEC_ERR_MASK	0x6000

static void i915_clear_errors_for_reg16(struct pci_dev *pdev, u32 offset, u16 mask)
{
	u16 reg16;

	pci_read_config_word(pdev, offset, &reg16);

	if (reg16 & mask) {
		pci_warn(pdev, "Clearing %x from %x\n",
			 reg16 & mask, offset);

		pci_write_config_word(pdev, offset, reg16 & mask);
	}
}

__maybe_unused
static void i915_clear_errors_for_reg32(struct pci_dev *pdev,
					u32 offset, u32 mask)
{
	u32 reg32;

	pci_read_config_dword(pdev, offset, &reg32);

	if (reg32 & mask) {
		pci_warn(pdev, "Clearing %x from %x\n",
			 reg32 & mask, offset);

		pci_write_config_dword(pdev, offset, reg32 & mask);
	}
}

static void i915_clear_pcie_errors(struct pci_dev *pdev)
{
	if (!pdev || pdev->is_virtfn)
		return;

	pci_disable_pcie_error_reporting(pdev);
	i915_clear_errors_for_reg16(pdev, PCI_SEC_STATUS, PCI_SEC_ERR_MASK);
	i915_clear_errors_for_reg16(pdev, pdev->pcie_cap + PCI_EXP_DEVSTA,
				    (u16)~0);
#if IS_ENABLED(CONFIG_PCIEAER)
	if (pdev->aer_cap) {
		i915_clear_errors_for_reg32(pdev,
					    pdev->aer_cap + PCI_ERR_COR_STATUS,
					    (u32)~0);
		i915_clear_errors_for_reg32(pdev,
					    pdev->aer_cap + PCI_ERR_UNCOR_STATUS,
					    (u32)~0);
	}
#endif
	pci_enable_pcie_error_reporting(pdev);
}

static int i915_driver_check_broken_features(struct drm_i915_private *dev_priv)
{
#ifdef CONFIG_PCI_ATS
	if (IS_PVC_BD_STEP(dev_priv, STEP_A0, STEP_B0)) {
		struct pci_dev *pdev = to_pci_dev(dev_priv->drm.dev);
		u16 val;

		/* Reading PCI_ATS_CTRL to check if ATS was enabled by IOMMU */
		pci_read_config_word(pdev, pdev->ats_cap + PCI_ATS_CTRL, &val);

		if (val & PCI_ATS_CTRL_ENABLE) {
			drm_warn(&dev_priv->drm,
				 "\n"
				 "\\*******************************************************\\\n"
				 "\\* Address translation service (ATS) is set as         *\\\n"
				 "\\* supported in this platform that is known to have    *\\\n"
				 "\\* issues with it. Also this notice means you are      *\\\n"
				 "\\* loading i915 as an external module and your BIOS    *\\\n"
				 "\\* has this feature enabled, so i915 will not fully   *\\\n"
				 "\\* load to avoid futher issues and crashes.            *\\\n"
				 "\\*                                                     *\\\n"
				 "\\* There are two options to disable this feature      *\\\n"
				 "\\* and unlock i915 driver load:                        *\\\n"
				 "\\* - Add 'pci=noats' to your kernel parameters         *\\\n"
				 "\\* - Disable ATS in your BIOS, that is vendor specific *\\\n"
				 "\\* and some BIOSes may even not allow this feature to  *\\\n"
				 "\\* be disabled                                         *\\\n"
				 "\\*******************************************************\\\n"
				 );
			return -ENODEV;
		}
	}
#endif /* CONFIG_PCI_ATS */

	/* Wa_16014292289:pvc[bd_a0] */
	if (IS_PVC_BD_STEP(dev_priv, STEP_A0, STEP_B0)) {
		struct pci_bus *bus = to_pci_dev(dev_priv->drm.dev)->bus;
		struct pci_dev *pdev;

		/* Iterate through the uncore, each of the SoCs and SGunit */
		i915_clear_pcie_errors(bus->self);
		list_for_each_entry(pdev, &bus->devices, bus_list)
			i915_clear_pcie_errors(pdev);
	}

	return 0;
}

/**
 * i915_driver_late_release - cleanup the setup done in
 *			       i915_driver_early_probe()
 * @dev_priv: device private
 */
static void i915_driver_late_release(struct drm_i915_private *dev_priv)
{
	intel_irq_fini(dev_priv);
	intel_power_domains_cleanup(dev_priv);
	i915_gem_cleanup_early(dev_priv);
	i915_debugger_fini(dev_priv);
	intel_gt_driver_late_release_all(dev_priv);
	i915_drm_clients_fini(&dev_priv->clients);
	vlv_suspend_cleanup(dev_priv);
	i915_workqueues_cleanup(dev_priv);

	cpu_latency_qos_remove_request(&dev_priv->sb_qos);
	mutex_destroy(&dev_priv->sb_lock);
	mutex_destroy(&dev_priv->svm_init_mutex);

	i915_params_free(&dev_priv->params);
}

/* Wa:16014207253 */
static enum hrtimer_restart fake_int_timer_callback(struct hrtimer *hrtimer)
{
	struct intel_gt *gt = container_of(hrtimer, typeof(*gt), fake_int.timer);
	struct intel_guc *guc = &gt->uc.guc;
	struct intel_engine_cs *engine;
	enum intel_engine_id id;

	if (guc->ct.enabled)
		intel_guc_ct_event_handler(&guc->ct);

	for_each_engine(engine, gt, id)
		engine->irq_handler(engine, GT_RENDER_USER_INTERRUPT |
					    GT_RENDER_PIPECTL_NOTIFY_INTERRUPT);

	if (!gt->fake_int.delay)
		return HRTIMER_NORESTART;

	hrtimer_forward_now(hrtimer, ns_to_ktime(gt->fake_int.delay));
	return HRTIMER_RESTART;
}

/*
 * Wa:16014207253,xehpsdv
 * Wa:16014202112,pvc
 */
static void init_fake_interrupts(struct intel_gt *gt)
{
	if (!gt->i915->remote_tiles)
		return;

	if (!intel_uc_wants_guc_submission(&gt->uc))
		return;

	if (!gt->i915->params.enable_fake_int_wa)
		return;

	if (IS_XEHPSDV(gt->i915) || IS_PVC_BD_STEP(gt->i915, STEP_A0, STEP_B1)) {
		gt->fake_int.delay_slow = 1000 * 1000 * 100;	/* 100ms */
		gt->fake_int.delay_fast = 1000 * 100;		/* 100us */

		/*XXX:On faultable platforms, interrupts can arrive as long as
		 * there are active requests. Currently we only boost the
		 * frquencey when there is outstanding g2h. Until we extend
		 * this to outstanding requests, always use boosted freq
		 */
		if (HAS_RECOVERABLE_PAGE_FAULT(gt->i915))
			gt->fake_int.delay_slow = gt->fake_int.delay_fast;

		gt->fake_int.enabled = 1;
		hrtimer_init(&gt->fake_int.timer, CLOCK_MONOTONIC,
			     HRTIMER_MODE_REL);
		gt->fake_int.timer.function = fake_int_timer_callback;

		intel_guc_init_fake_interrupts(&gt->uc.guc);
	}
}

/**
 * i915_driver_mmio_probe - setup device MMIO
 * @dev_priv: device private
 *
 * Setup minimal device state necessary for MMIO accesses later in the
 * initialization sequence. The setup here should avoid any other device-wide
 * side effects or exposing the driver via kernel internal or user space
 * interfaces.
 */
static int i915_driver_mmio_probe(struct drm_i915_private *dev_priv)
{
	struct intel_gt *gt;
	int ret, i;

	if (i915_inject_probe_failure(dev_priv))
		return -ENODEV;

	ret = i915_get_bridge_dev(dev_priv);
	if (ret < 0)
		return ret;

	for_each_gt(gt, dev_priv, i) {
		ret = intel_uncore_init_mmio(gt->uncore);
		if (ret)
			return ret;

		ret = drmm_add_action_or_reset(&dev_priv->drm,
					       intel_uncore_fini_mmio,
					       gt->uncore);
		if (ret)
			return ret;
	}

	/* Try to make sure MCHBAR is enabled before poking at it */
	intel_setup_mchbar(dev_priv);
	intel_device_info_runtime_init(dev_priv);

	for_each_gt(gt, dev_priv, i) {
		ret = intel_gt_init_mmio(gt);
		if (ret)
			goto err_uncore;

		init_fake_interrupts(gt);
	}

	intel_iaf_init_mmio(dev_priv);

	/* As early as possible, scrub existing GPU state before clobbering */
	sanitize_gpu(dev_priv);

	return 0;

err_uncore:
	intel_teardown_mchbar(dev_priv);

	return ret;
}

/**
 * i915_driver_mmio_release - cleanup the setup done in i915_driver_mmio_probe()
 * @dev_priv: device private
 */
static void i915_driver_mmio_release(struct drm_i915_private *dev_priv)
{
	intel_teardown_mchbar(dev_priv);
}

static void intel_sanitize_options(struct drm_i915_private *dev_priv)
{
	intel_gvt_sanitize_options(dev_priv);
}

/**
 * i915_set_dma_info - set all relevant PCI dma info as configured for the
 * platform
 * @i915: valid i915 instance
 *
 * Set the dma max segment size, device and coherent masks.  The dma mask set
 * needs to occur before i915_ggtt_probe_hw.
 *
 * A couple of platforms have special needs.  Address them as well.
 *
 */
static int i915_set_dma_info(struct drm_i915_private *i915)
{
	unsigned int mask_size = INTEL_INFO(i915)->dma_mask_size;
	int ret;

	GEM_BUG_ON(!mask_size);

	/*
	 * We don't have a max segment size, so set it to the max so sg's
	 * debugging layer doesn't complain
	 */
	dma_set_max_seg_size(i915->drm.dev, UINT_MAX);

	ret = dma_set_mask(i915->drm.dev, DMA_BIT_MASK(mask_size));
	if (ret)
		goto mask_err;

	/* overlay on gen2 is broken and can't address above 1G */
	if (GRAPHICS_VER(i915) == 2)
		mask_size = 30;

	/*
	 * 965GM sometimes incorrectly writes to hardware status page (HWS)
	 * using 32bit addressing, overwriting memory if HWS is located
	 * above 4GB.
	 *
	 * The documentation also mentions an issue with undefined
	 * behaviour if any general state is accessed within a page above 4GB,
	 * which also needs to be handled carefully.
	 */
	if (IS_I965G(i915) || IS_I965GM(i915))
		mask_size = 32;

	ret = dma_set_coherent_mask(i915->drm.dev, DMA_BIT_MASK(mask_size));
	if (ret)
		goto mask_err;

	return 0;

mask_err:
	drm_err(&i915->drm, "Can't set DMA mask/consistent mask (%d)\n", ret);
	return ret;
}

static int i915_pcode_init(struct drm_i915_private *i915)
{
	struct intel_gt *gt;
	int id, ret;

	for_each_gt(gt, i915, id) {
		ret = intel_pcode_init(gt->uncore);
		if (ret) {
			drm_err(&gt->i915->drm, "gt%d: intel_pcode_init failed %d\n", id, ret);
			return ret;
		}
	}

	return 0;
}

static int intel_pcode_probe(struct drm_i915_private *i915)
{
	struct intel_gt *gt;
	int id, ret;

	/*
	 * The boot firmware initializes local memory and assesses its health.
	 * If memory training fails, the punit will have been instructed to
	 * keep the GT powered down; we won't be able to communicate with it
	 * and we should not continue with driver initialization.
	 */
	for_each_gt(gt, i915, id) {
		ret = intel_uncore_wait_for_lmem(gt->uncore);
		if (ret)
			return ret;
	}

	/*
	 * Driver handshakes with pcode via mailbox command to know that SoC
	 * initialization is complete before proceeding further
	 */
	ret = i915_pcode_init(i915);

	return ret;
}

/**
 * i915_driver_hw_probe - setup state requiring device access
 * @dev_priv: device private
 *
 * Setup state that requires accessing the device, but doesn't require
 * exposing the driver via kernel internal or userspace interfaces.
 */
static int i915_driver_hw_probe(struct drm_i915_private *dev_priv)
{
	struct pci_dev *pdev = to_pci_dev(dev_priv->drm.dev);
	struct pci_dev *root_pdev;
	int ret;

	if (i915_inject_probe_failure(dev_priv))
		return -ENODEV;

	if (HAS_PPGTT(dev_priv)) {
		if (intel_vgpu_active(dev_priv) &&
		    !intel_vgpu_has_full_ppgtt(dev_priv)) {
			i915_report_error(dev_priv,
					  "incompatible vGPU found, support for isolated ppGTT required\n");
			return -ENXIO;
		}
	}

	if (HAS_EXECLISTS(dev_priv)) {
		/*
		 * Older GVT emulation depends upon intercepting CSB mmio,
		 * which we no longer use, preferring to use the HWSP cache
		 * instead.
		 */
		if (intel_vgpu_active(dev_priv) &&
		    !intel_vgpu_has_hwsp_emulation(dev_priv)) {
			i915_report_error(dev_priv,
					  "old vGPU host found, support for HWSP emulation required\n");
			return -ENXIO;
		}
	}

	intel_sanitize_options(dev_priv);

	/* needs to be done before ggtt probe */
	intel_dram_edram_detect(dev_priv);

	ret = i915_set_dma_info(dev_priv);
	if (ret)
		return ret;

	ret = i915_perf_init(dev_priv);
	if (ret)
		return ret;

	i915_perf_stall_cntr_init(dev_priv);

	/* needs to be done before ggtt probe and init  */
	intel_iaf_init(dev_priv);

	ret = i915_ggtt_probe_hw(dev_priv);
	if (ret)
		goto err_perf;

	ret = drm_aperture_remove_conflicting_pci_framebuffers(pdev, dev_priv->drm.driver);
	if (ret)
		goto err_ggtt;

	ret = i915_ggtt_init_hw(dev_priv);
	if (ret)
		goto err_ggtt;

	ret = intel_memory_regions_hw_probe(dev_priv);
	if (ret)
		goto err_ggtt;

	ret = intel_gt_tiles_init(dev_priv);
	if (ret)
		goto err_mem_regions;

	ret = i915_ggtt_enable_hw(dev_priv);
	if (ret) {
		drm_err(&dev_priv->drm, "failed to enable GGTT\n");
		goto err_mem_regions;
	}

	pci_set_master(pdev);

	/* Assume that VF is up, otherwise we may end with unknown state */
	if (IS_SRIOV_VF(dev_priv))
		ret = pci_set_power_state(pdev, PCI_D0);

	/* On the 945G/GM, the chipset reports the MSI capability on the
	 * integrated graphics even though the support isn't actually there
	 * according to the published specs.  It doesn't appear to function
	 * correctly in testing on 945G.
	 * This may be a side effect of MSI having been made available for PEG
	 * and the registers being closely associated.
	 *
	 * According to chipset errata, on the 965GM, MSI interrupts may
	 * be lost or delayed, and was defeatured. MSI interrupts seem to
	 * get lost on g4x as well, and interrupt delivery seems to stay
	 * properly dead afterwards. So we'll just disable them for all
	 * pre-gen5 chipsets.
	 *
	 * dp aux and gmbus irq on gen4 seems to be able to generate legacy
	 * interrupts even when in MSI mode. This results in spurious
	 * interrupt warnings if the legacy irq no. is shared with another
	 * device. The kernel then disables that interrupt source and so
	 * prevents the other device from working properly.
	 */
	if (GRAPHICS_VER(dev_priv) >= 5) {
		if (pci_enable_msi(pdev) < 0)
			drm_dbg(&dev_priv->drm, "can't enable MSI");
	}

	ret = intel_gvt_init(dev_priv);
	if (ret)
		goto err_msi;

	intel_opregion_init(dev_priv);

	/*
	 * Fill the dram structure to get the system dram info. This will be
	 * used for memory latency calculation.
	 */
	intel_dram_detect(dev_priv);

	intel_bw_init_hw(dev_priv);

	/*
	 * FIXME: Temporary hammer to avoid freezing the machine on our DGFX
	 * This should be totally removed when we handle the pci states properly
	 * on runtime PM and on s2idle cases.
	 */
	root_pdev = pcie_find_root_port(pdev);
	if (root_pdev)
		pci_d3cold_disable(root_pdev);

	init_device_clos(dev_priv);
	return 0;

err_msi:
	if (pdev->msi_enabled)
		pci_disable_msi(pdev);
err_mem_regions:
	intel_memory_regions_driver_release(dev_priv);
err_ggtt:
	i915_ggtt_driver_release(dev_priv);
	i915_gem_drain_freed_objects(dev_priv);
	i915_ggtt_driver_late_release(dev_priv);
err_perf:
	i915_perf_fini(dev_priv);
	return ret;
}

/**
 * i915_driver_hw_remove - cleanup the setup done in i915_driver_hw_probe()
 * @dev_priv: device private
 */
static void i915_driver_hw_remove(struct drm_i915_private *dev_priv)
{
	struct pci_dev *pdev = to_pci_dev(dev_priv->drm.dev);
	struct pci_dev *root_pdev;

	i915_perf_fini(dev_priv);

	if (pdev->msi_enabled)
		pci_disable_msi(pdev);

	root_pdev = pcie_find_root_port(pdev);
	if (root_pdev)
		pci_d3cold_enable(root_pdev);
}

static void i915_virtualization_commit(struct drm_i915_private *i915)
{
	if (IS_SRIOV_PF(i915))
		i915_sriov_pf_confirm(i915);
}

/**
 * i915_driver_register - register the driver with the rest of the system
 * @dev_priv: device private
 *
 * Perform any steps necessary to make the driver available via kernel
 * internal or userspace interfaces.
 */
void i915_driver_register(struct drm_i915_private *dev_priv)
{
	struct drm_device *dev = &dev_priv->drm;
	struct intel_gt *gt;
	unsigned int i;

	i915_gem_driver_register(dev_priv);
#if IS_ENABLED(CONFIG_AUXILIARY_BUS)
	intel_iaf_init_aux(dev_priv);
#else
	intel_iaf_init_mfd(dev_priv);
#endif

	intel_vgpu_register(dev_priv);

	/* Reveal our presence to userspace */
	if (drm_dev_register(dev, 0)) {
		drm_err(&dev_priv->drm,
			"Failed to register driver for userspace access!\n");
		return;
	}

	i915_debugfs_register(dev_priv);
	i915_setup_sysfs(dev_priv);
	i915_register_sysrq(dev_priv);


	intel_spi_init(&dev_priv->spi, dev_priv);

	for_each_gt(gt, dev_priv, i)
		intel_gt_driver_register(gt);

	/* Depends on sysfs having been initialized */
	i915_pmu_register(dev_priv);
	i915_perf_register(dev_priv);

	if (!IS_SRIOV_VF(dev_priv))
		i915_hwmon_register(dev_priv);

	intel_display_driver_register(dev_priv);

	intel_power_domains_enable(dev_priv);

	intel_register_dsm_handler();

	if (i915_switcheroo_register(dev_priv))
		drm_err(&dev_priv->drm, "Failed to register vga switcheroo!\n");

	intel_vsec_init(dev_priv);
	pvc_wa_allow_rc6(dev_priv);
}

/**
 * i915_driver_unregister - cleanup the registration done in i915_driver_regiser()
 * @dev_priv: device private
 */
static void i915_driver_unregister(struct drm_i915_private *dev_priv)
{
	struct intel_gt *gt;
	unsigned int i;

	i915_switcheroo_unregister(dev_priv);

	intel_unregister_dsm_handler();

	intel_runtime_pm_disable(&dev_priv->runtime_pm);
	intel_power_domains_disable(dev_priv);

#if !IS_ENABLED(CONFIG_AUXILIARY_BUS)
	/*
	 * mfd devices may be registered individually either by gt or display,
	 * but they are unregistered all at once from i915
	 */
	mfd_remove_devices(dev_priv->drm.dev);
#endif
	intel_display_driver_unregister(dev_priv);

	i915_hwmon_unregister(dev_priv);

	intel_iaf_remove(dev_priv);

	intel_spi_fini(&dev_priv->spi);

	i915_perf_unregister(dev_priv);
	/* GT should be available until PMU is gone */
	i915_pmu_unregister(dev_priv);

	for_each_gt(gt, dev_priv, i)
		intel_gt_driver_unregister(gt);

	i915_unregister_sysrq(dev_priv);

	/*
	 * check if we already unplugged in handling a PCI error
	 * check for quiesce_gpu as we are faking the drm unplug
	 * in that path.
	 * FIXME: This check is not needed when unbind is called
	 * in error_detected callback
	 */
	if (!dev_priv->drm.unplugged || dev_priv->quiesce_gpu) {
		i915_teardown_sysfs(dev_priv);
		drm_dev_unplug(&dev_priv->drm);
	}

	i915_gem_driver_unregister(dev_priv);
}

void
i915_print_iommu_status(struct drm_i915_private *i915, struct drm_printer *p)
{
	drm_printf(p, "iommu: %s\n",
		   str_enabled_disabled(i915_vtd_active(i915)));
}

__maybe_unused
static void print_chickens(struct drm_i915_private *i915)
{
	static const struct {
		const char *name;
		bool state;
	} chickens[] = {
#define C(x) { __stringify(DRM_I915_CHICKEN_##x), IS_ENABLED(CONFIG_DRM_I915_CHICKEN_##x) }
		C(ASYNC_GET_PAGES),
		C(ASYNC_PAGEFAULTS),
		C(CLEAR_ON_CREATE),
		C(CLEAR_ON_FREE),
		C(CLEAR_ON_IDLE),
		C(MMAP_SWAP),
		C(MMAP_SWAP_CREATE),
		C(NUMA_ALLOC),
		C(PARALLEL_SHMEMFS),
		C(PARALLEL_USERPTR),
		C(ULL_DMA_BOOST),
		C(SOFT_PG),
#undef C
		{},
	};
	const typeof(*chickens) *c;

	for (c = chickens; c->name; c++)
		drm_info(&i915->drm, "  %s: %s\n",
			 c->name, str_enabled_disabled(c->state));
}

static void i915_welcome_messages(struct drm_i915_private *dev_priv)
{
	if (IS_SRIOV_VF(dev_priv))
		return;

	if (drm_debug_enabled(DRM_UT_DRIVER)) {
		struct drm_printer p = drm_debug_printer("i915 device info:");
		struct intel_gt *gt;
		unsigned int i;

		drm_printf(&p, "pciid=0x%04x rev=0x%02x platform=%s (subplatform=0x%x) gen=%i\n",
			   INTEL_DEVID(dev_priv),
			   INTEL_REVID(dev_priv),
			   intel_platform_name(INTEL_INFO(dev_priv)->platform),
			   intel_subplatform(RUNTIME_INFO(dev_priv),
					     INTEL_INFO(dev_priv)->platform),
			   GRAPHICS_VER(dev_priv));

		intel_device_info_print_static(INTEL_INFO(dev_priv), &p);
		intel_device_info_print_runtime(RUNTIME_INFO(dev_priv), &p);
		i915_print_iommu_status(dev_priv, &p);
		for_each_gt(gt, dev_priv, i)
			intel_gt_info_print(&gt->info, &p);

		drm_printf(&p, "mode: %s\n", i915_iov_mode_to_string(IOV_MODE(dev_priv)));
	}

	if (IS_ENABLED(CONFIG_DRM_I915_DEBUG)) {
		drm_info(&dev_priv->drm, "DRM_I915_DEBUG enabled\n");
		print_chickens(dev_priv);
	}
	if (IS_ENABLED(CONFIG_DRM_I915_DEBUG_GEM))
		drm_info(&dev_priv->drm, "DRM_I915_DEBUG_GEM enabled\n");
	if (IS_ENABLED(CONFIG_DRM_I915_DEBUG_RUNTIME_PM))
		drm_info(&dev_priv->drm,
			 "DRM_I915_DEBUG_RUNTIME_PM enabled\n");
}

static void fixup_mm(struct drm_mm *mm, unsigned long start, unsigned long size)
{
	struct drm_mm_node *head = &mm->head_node;

	head->start = start + size;
	head->size = -size;

	head->hole_size = size;
	head->subtree_max_hole = size;
}

static struct drm_i915_private *
i915_driver_create(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	const struct intel_device_info *match_info =
		(struct intel_device_info *)ent->driver_data;
	struct intel_device_info *device_info;
	struct drm_i915_private *i915;

	i915 = devm_drm_dev_alloc(&pdev->dev, &i915_drm_driver,
				  struct drm_i915_private, drm);
	if (IS_ERR(i915))
		return i915;

	pci_set_drvdata(pdev, i915);

	/* Device parameters start as a copy of module parameters. */
	i915_params_copy(&i915->params, &i915_modparams);

	/* Setup the write-once "constant" device info */
	device_info = mkwrite_device_info(i915);
	memcpy(device_info, match_info, sizeof(*device_info));
	RUNTIME_INFO(i915)->device_id = pdev->device;

	/* Fixup mmap support for 2TiB+ objects */
	fixup_mm(&i915->drm.vma_offset_manager->vm_addr_space_mm,
		 DRM_FILE_PAGE_OFFSET_START,
		 ULONG_MAX - DRM_FILE_PAGE_OFFSET_START);

	return i915;
}

static void i915_read_dev_uid(struct drm_i915_private *i915)
{
	if (INTEL_INFO(i915)->has_csc_uid)
		RUNTIME_INFO(i915)->uid  = intel_uncore_read64_2x32(&i915->uncore,
								    CSC_DEVUID_LWORD,
								    CSC_DEVUID_HWORD);
}

static void i915_sanitize_force_driver_flr(struct drm_i915_private *i915)
{
	/*
	* Sanitize force_driver_flr at init time: If hardware needs driver-FLR at
	* load / unload and the user has not forced it off then allow triggering driver-FLR.
	* Exception: VFs cant access the driver-FLR registers.
	*/
	if (!INTEL_INFO(i915)->needs_driver_flr || IS_SRIOV_VF(i915))
		i915->params.force_driver_flr = 0;
	else if (i915->params.force_driver_flr == -1)
		i915->params.force_driver_flr = 1;
}

static void i915_virtualization_probe(struct drm_i915_private *i915)
{
	GEM_BUG_ON(i915->__mode);

	intel_vgpu_detect(i915);
	if (intel_vgpu_active(i915))
		i915->__mode = I915_IOV_MODE_GVT_VGPU;
	else
		i915->__mode = i915_sriov_probe(i915);

	GEM_BUG_ON(!i915->__mode);

	if (IS_IOV_ACTIVE(i915))
		dev_info(i915->drm.dev, "Running in %s mode\n",
			 i915_iov_mode_to_string(IOV_MODE(i915)));
}

/**
 * i915_driver_probe - setup chip and create an initial config
 * @pdev: PCI device
 * @ent: matching PCI ID entry
 *
 * The driver probe routine has to do several things:
 *   - drive output discovery via intel_modeset_init()
 *   - initialize the memory manager
 *   - allocate initial config memory
 *   - setup the DRM framebuffer with the allocated memory
 */
int i915_driver_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	const struct intel_device_info *match_info =
		(struct intel_device_info *)ent->driver_data;
	struct drm_i915_private *i915;
	int ret;

	i915 = i915_driver_create(pdev, ent);
	if (IS_ERR(i915))
		return PTR_ERR(i915);

	/* Disable nuclear pageflip by default on pre-ILK */
	if (!i915->params.nuclear_pageflip &&
	    !match_info->has_gmd_id && match_info->graphics.ver < 5)
		i915->drm.driver_features &= ~DRIVER_ATOMIC;

	ret = pci_enable_device(pdev);
	if (ret)
		goto out_fini;

	/* This must be called before any calls to IS/IOV_MODE() macros */
	i915_virtualization_probe(i915);

	/*
	 * GRAPHICS_VER() and DISPLAY_VER() will return 0 before this is
	 * called, so we want to take care of this very early in the
	 * initialization process (as soon as we can peek into the MMIO BAR),
	 * even before we setup regular MMIO access.
	 */
	intel_ipver_early_init(i915, match_info);

	ret = i915_driver_early_probe(i915, match_info);
	if (ret < 0)
		goto out_pci_disable;

	ret = i915_driver_check_broken_features(i915);
	if (ret < 0)
		goto out_driver_late_release;

	disable_rpm_wakeref_asserts(&i915->runtime_pm);

	ret = i915_sriov_early_tweaks(i915);
	if (ret < 0)
		goto out_driver_late_release;

	if (HAS_LMEM(i915))
		i915_resize_lmem_bar(i915);

	if (i915->params.smem_access_control == I915_SMEM_ACCESS_CONTROL_DEFAULT) {
		if (IS_XEHPSDV_GRAPHICS_STEP(i915, STEP_A0,  STEP_B0) ||
		    IS_PVC_BD_STEP(i915, STEP_A0, STEP_B0)) {
			/* Wa_16012239583:pvc */
			i915->bind_ctxt_ready = false;
			i915->params.smem_access_control = 5;
		} else {
			/* For other platforms disable level-4 wa */
			i915->params.smem_access_control = 0;
		}
	}

	i915_sanitize_force_driver_flr(i915);

	ret = intel_gt_probe_all(i915);
	if (ret < 0)
		goto out_runtime_pm_put;

	ret = pvc_ras_telemetry_probe(i915);
	if (ret)
		goto out_runtime_pm_put;

	ret = intel_pcode_probe(i915);
	if (ret)
		goto out_runtime_pm_put;

	ret = i915_driver_mmio_probe(i915);
	if (ret < 0)
		goto out_runtime_pm_put;

	i915_read_dev_uid(i915);

	pvc_wa_disallow_rc6(i915);

	ret = i915_driver_hw_probe(i915);
	if (ret < 0)
		goto out_cleanup_mmio;

	ret = intel_modeset_init_noirq(i915);
	if (ret < 0)
		goto out_cleanup_hw;

	ret = intel_irq_install(i915);
	if (ret)
		goto out_cleanup_modeset;

	ret = intel_modeset_init_nogem(i915);
	if (ret)
		goto out_cleanup_irq;

	ret = i915_gem_init(i915);
	if (ret)
		goto out_cleanup_modeset2;

	ret = intel_modeset_init(i915);
	if (ret)
		goto out_cleanup_gem;

	intel_runtime_pm_enable(&i915->runtime_pm);

	enable_rpm_wakeref_asserts(&i915->runtime_pm);

	i915_virtualization_commit(i915);

	i915_welcome_messages(i915);

	i915->do_release = true;

	if ((IS_XEHPSDV_GRAPHICS_STEP(i915, STEP_A0, STEP_B0) ||
	     IS_PVC_BD_STEP(i915, STEP_A0, STEP_B0)) &&
	    i915->params.smem_access_control == 5)
		i915->bind_ctxt_ready = true;

	/* Enable Address Translation Services */
	i915_enable_ats(i915);

	return 0;

out_cleanup_gem:
	i915_gem_suspend(i915);
	i915_gem_driver_remove(i915);
	i915_gem_driver_release(i915);
out_cleanup_modeset2:
	/* FIXME clean up the error path */
	intel_modeset_driver_remove(i915);
	intel_irq_uninstall(i915);
	intel_modeset_driver_remove_noirq(i915);
	goto out_cleanup_modeset;
out_cleanup_irq:
	intel_irq_uninstall(i915);
out_cleanup_modeset:
	intel_modeset_driver_remove_nogem(i915);
out_cleanup_hw:
	i915_driver_hw_remove(i915);
	intel_memory_regions_driver_release(i915);
	i915_ggtt_driver_release(i915);
	i915_gem_drain_freed_objects(i915);
	i915_ggtt_driver_late_release(i915);
out_cleanup_mmio:
	pvc_wa_allow_rc6(i915);
	i915_driver_mmio_release(i915);
out_runtime_pm_put:
	enable_rpm_wakeref_asserts(&i915->runtime_pm);
out_driver_late_release:
	i915_driver_late_release(i915);
out_pci_disable:
	pci_disable_device(pdev);
out_fini:
	i915_probe_error(i915, "Device initialization failed (%d)\n", ret);
	return ret;
}

void i915_driver_remove(struct drm_i915_private *i915)
{
	struct pci_dev *pdev = to_pci_dev(i915->drm.dev);

	disable_rpm_wakeref_asserts(&i915->runtime_pm);

	pvc_wa_disallow_rc6(i915);

	/*
	 * If device is quiesced set device to offline status
	 * so MEI driver can avoid access to HW registers
	 */
	if (i915->quiesce_gpu)
		i915_pci_set_offline(pdev);

	i915_driver_unregister(i915);

	/* Flush any external code that still may be under the RCU lock */
	synchronize_rcu();

	i915_gem_suspend(i915);

	/* Disable Address Translation Services */
	i915_disable_ats(i915);

	uninit_device_clos(i915);

	intel_gvt_driver_remove(i915);

	intel_modeset_driver_remove(i915);

	intel_irq_uninstall(i915);

	intel_modeset_driver_remove_noirq(i915);

	i915_reset_error_state(i915);
	i915_gem_driver_remove(i915);

	intel_modeset_driver_remove_nogem(i915);

	i915_driver_hw_remove(i915);

	kfree(i915->pci_state);

	enable_rpm_wakeref_asserts(&i915->runtime_pm);
}

static void i915_driver_release(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = to_i915(dev);
	struct intel_runtime_pm *rpm = &dev_priv->runtime_pm;

	if (!dev_priv->do_release)
		return;

	disable_rpm_wakeref_asserts(rpm);

	i915_gem_driver_release(dev_priv);

	intel_memory_regions_driver_release(dev_priv);
	i915_ggtt_driver_release(dev_priv);
	i915_gem_drain_freed_objects(dev_priv);
	i915_ggtt_driver_late_release(dev_priv);
	pvc_wa_allow_rc6(dev_priv);

	i915_driver_mmio_release(dev_priv);

	enable_rpm_wakeref_asserts(rpm);
	intel_runtime_pm_driver_release(rpm);

	i915_driver_late_release(dev_priv);
}

static int i915_driver_open(struct drm_device *dev, struct drm_file *file)
{
	struct drm_i915_private *i915 = to_i915(dev);
	int ret;

	ret = i915_gem_open(i915, file);
	if (ret)
		return ret;

	return 0;
}

/**
 * i915_driver_lastclose - clean up after all DRM clients have exited
 * @dev: DRM device
 *
 * Take care of cleaning up after all DRM clients have exited.  In the
 * mode setting case, we want to restore the kernel's initial mode (just
 * in case the last client left us in a bad state).
 *
 * Additionally, in the non-mode setting case, we'll tear down the GTT
 * and DMA structures, since the kernel won't be using them, and clea
 * up any GEM state.
 */
static void i915_driver_lastclose(struct drm_device *dev)
{
	struct drm_i915_private *i915 = to_i915(dev);

	intel_fbdev_restore_mode(dev);

	if (HAS_DISPLAY(i915))
		vga_switcheroo_process_delayed_switch();
}

static void i915_driver_postclose(struct drm_device *dev, struct drm_file *file)
{
	struct drm_i915_file_private *file_priv = file->driver_priv;

	/*
	 * Before changing anything wait for EU Debugger's discovery thread.
	 * Mark this client as being closed.
	 */
	i915_debugger_wait_on_discovery(to_i915(dev), file_priv->client);
	i915_drm_client_close(file_priv->client);

	i915_gem_context_close(file);
	i915_drm_client_cleanup(file_priv->client);

	uninit_client_clos(file_priv);
	kfree_rcu(file_priv, rcu);

	/* Catch up with all the deferred frees from "this" client */
	i915_gem_flush_free_objects(to_i915(dev));
}

static void intel_suspend_encoders(struct drm_i915_private *dev_priv)
{
	struct drm_device *dev = &dev_priv->drm;
	struct intel_encoder *encoder;

	if (!HAS_DISPLAY(dev_priv))
		return;

	drm_modeset_lock_all(dev);
	for_each_intel_encoder(dev, encoder)
		if (encoder->suspend)
			encoder->suspend(encoder);
	drm_modeset_unlock_all(dev);
}

static void intel_shutdown_encoders(struct drm_i915_private *dev_priv)
{
	struct drm_device *dev = &dev_priv->drm;
	struct intel_encoder *encoder;

	if (!HAS_DISPLAY(dev_priv))
		return;

	drm_modeset_lock_all(dev);
	for_each_intel_encoder(dev, encoder)
		if (encoder->shutdown)
			encoder->shutdown(encoder);
	drm_modeset_unlock_all(dev);
}

void i915_driver_shutdown(struct drm_i915_private *i915)
{
	struct intel_gt *gt;
	unsigned int i;

	disable_rpm_wakeref_asserts(&i915->runtime_pm);
	intel_runtime_pm_disable(&i915->runtime_pm);
	intel_power_domains_disable(i915);

	if (HAS_DISPLAY(i915)) {
		drm_kms_helper_poll_disable(&i915->drm);

		drm_atomic_helper_shutdown(&i915->drm);
	}

	intel_dp_mst_suspend(i915);

	intel_runtime_pm_disable_interrupts(i915);
	intel_hpd_cancel_work(i915);

	intel_suspend_encoders(i915);
	intel_shutdown_encoders(i915);

	intel_dmc_ucode_suspend(i915);

	i915_gem_suspend(i915);

	for_each_gt(gt, i915, i)
		intel_gt_shutdown(gt);

	/*
	 * The only requirement is to reboot with display DC states disabled,
	 * for now leaving all display power wells in the INIT power domain
	 * enabled.
	 *
	 * TODO:
	 * - unify the pci_driver::shutdown sequence here with the
	 *   pci_driver.driver.pm.poweroff,poweroff_late sequence.
	 * - unify the driver remove and system/runtime suspend sequences with
	 *   the above unified shutdown/poweroff sequence.
	 */
	intel_power_domains_driver_remove(i915);
}

static bool suspend_to_idle(struct drm_i915_private *dev_priv)
{
#if IS_ENABLED(CONFIG_ACPI_SLEEP)
	if (acpi_target_system_state() < ACPI_STATE_S3)
		return true;
#endif
	return false;
}

static void intel_evict_lmem(struct drm_i915_private *i915)
{
	struct intel_memory_region_link bookmark = {};
	struct intel_memory_region *mem;
	int id;

	for_each_memory_region(mem, i915, id) {
		struct intel_memory_region_link *pos, *next;
		struct list_head *phases[] = {
			&mem->objects.purgeable,
			&mem->objects.list,
			NULL,
		}, **phase = phases;

		if (mem->type != INTEL_MEMORY_LOCAL || !mem->total)
			continue;

		spin_lock(&mem->objects.lock);
		do list_for_each_entry_safe(pos, next, *phase, link) {
			struct drm_i915_gem_object *obj;
			struct i915_gem_ww_ctx ww;
			int err;

			if (!pos->mem)
				continue;

			obj = container_of(pos, typeof(*obj), mm.region);

			/* only segment BOs should be in mem->objects.list */
			GEM_BUG_ON(i915_gem_object_has_segments(obj));

			if (!i915_gem_object_has_pages(obj))
				continue;

			if (!kref_get_unless_zero(&obj->base.refcount))
				continue;

			list_add(&bookmark.link, &pos->link);
			spin_unlock(&mem->objects.lock);

			for_i915_gem_ww(&ww, err, true) {
				err = i915_gem_object_lock(obj, &ww);
				if (err)
					continue;

				if (!i915_gem_object_has_pages(obj))
					continue;

				i915_gem_object_move_notify(obj);

				err = i915_gem_object_unbind(obj, &ww, 0);
				if (err == 0)
					err = __i915_gem_object_put_pages(obj);
			}

			i915_gem_object_put(obj);

			spin_lock(&mem->objects.lock);
			list_safe_reset_next(&bookmark, next, link);
			__list_del_entry(&bookmark.link);
		} while (*++phase);
		spin_unlock(&mem->objects.lock);
	}
}

static int i915_drm_prepare(struct drm_device *dev)
{
	struct drm_i915_private *i915 = to_i915(dev);

	pvc_wa_disallow_rc6(i915);

	/*
	 * NB intel_display_suspend() may issue new requests after we've
	 * ostensibly marked the GPU as ready-to-sleep here. We need to
	 * split out that work and pull it forward so that after point,
	 * the GPU is not woken again.
	 */
	i915_gem_suspend(i915);

	/*
	 * FIXME: After parking  GPU, we are waking up the GPU by doing
	 * intel_evict_lmem(), which needs to be avoid.
	 */
	intel_evict_lmem(i915);

	return 0;
}

static int i915_drm_suspend(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = to_i915(dev);
	struct pci_dev *pdev = to_pci_dev(dev_priv->drm.dev);
	pci_power_t opregion_target_state;

	disable_rpm_wakeref_asserts(&dev_priv->runtime_pm);

	/* We do a lot of poking in a lot of registers, make sure they work
	 * properly.
	 */
	intel_power_domains_disable(dev_priv);
	if (HAS_DISPLAY(dev_priv))
		drm_kms_helper_poll_disable(dev);

	i915_save_pci_state(pdev);

	intel_display_suspend(dev);

	intel_dp_mst_suspend(dev_priv);

	intel_runtime_pm_disable_interrupts(dev_priv);
	intel_hpd_cancel_work(dev_priv);

	intel_suspend_encoders(dev_priv);

	intel_suspend_hw(dev_priv);

	i915_save_display(dev_priv);

	opregion_target_state = suspend_to_idle(dev_priv) ? PCI_D1 : PCI_D3cold;
	intel_opregion_suspend(dev_priv, opregion_target_state);

	intel_fbdev_set_suspend(dev, FBINFO_STATE_SUSPENDED, true);

	dev_priv->suspend_count++;

	intel_dmc_ucode_suspend(dev_priv);

	enable_rpm_wakeref_asserts(&dev_priv->runtime_pm);

	i915_gem_drain_freed_objects(dev_priv);

	return 0;
}

static enum i915_drm_suspend_mode
get_suspend_mode(struct drm_i915_private *dev_priv, bool hibernate)
{
	if (hibernate)
		return I915_DRM_SUSPEND_HIBERNATE;

	if (suspend_to_idle(dev_priv))
		return I915_DRM_SUSPEND_IDLE;

	return I915_DRM_SUSPEND_MEM;
}

static int i915_drm_suspend_late(struct drm_device *dev, bool hibernation)
{
	struct drm_i915_private *dev_priv = to_i915(dev);
	struct pci_dev *pdev = to_pci_dev(dev_priv->drm.dev);
	struct intel_runtime_pm *rpm = &dev_priv->runtime_pm;
	struct intel_gt *gt;
	int ret, i;

	disable_rpm_wakeref_asserts(rpm);

	i915_sriov_suspend_late(dev_priv);

	/* Must be called before GGTT is suspended. */
	intel_dpt_suspend(dev_priv);
	ret = i915_gem_suspend_late(dev_priv);

	/*
	 * RC6 is disabled at this point, forcewake is cleared
	 * to balance the forcewake set at suspend_prepare.
	 */
	pvc_wa_allow_rc6(dev_priv);
	if (ret) {
		drm_err(&dev_priv->drm, "Suspend swapout failed: %d\n", ret);
		return ret;
	}

	for_each_gt(gt, dev_priv, i)
		intel_uncore_suspend(gt->uncore);

	intel_power_domains_suspend(dev_priv,
				    get_suspend_mode(dev_priv, hibernation));

	intel_display_power_suspend_late(dev_priv);

	ret = vlv_suspend_complete(dev_priv);
	if (ret) {
		drm_err(&dev_priv->drm, "Suspend complete failed: %d\n", ret);
		intel_power_domains_resume(dev_priv);

		goto out;
	}

	pci_disable_device(pdev);
	/*
	 * During hibernation on some platforms the BIOS may try to access
	 * the device even though it's already in D3 and hang the machine. So
	 * leave the device in D0 on those platforms and hope the BIOS will
	 * power down the device properly. The issue was seen on multiple old
	 * GENs with different BIOS vendors, so having an explicit blacklist
	 * is inpractical; apply the workaround on everything pre GEN6. The
	 * platforms where the issue was seen:
	 * Lenovo Thinkpad X301, X61s, X60, T60, X41
	 * Fujitsu FSC S7110
	 * Acer Aspire 1830T
	 */
	if (!(hibernation && GRAPHICS_VER(dev_priv) < 6))
		pci_set_power_state(pdev, PCI_D3hot);

out:
	enable_rpm_wakeref_asserts(rpm);
	if (!dev_priv->uncore.user_forcewake_count)
		intel_runtime_pm_driver_release(rpm);

	return ret;
}

int i915_driver_suspend_switcheroo(struct drm_i915_private *i915,
				   pm_message_t state)
{
	int error;

	if (drm_WARN_ON_ONCE(&i915->drm, state.event != PM_EVENT_SUSPEND &&
			     state.event != PM_EVENT_FREEZE))
		return -EINVAL;

	if (i915->drm.switch_power_state == DRM_SWITCH_POWER_OFF)
		return 0;

	error = i915_drm_suspend(&i915->drm);
	if (error)
		return error;

	return i915_drm_suspend_late(&i915->drm, false);
}

static int i915_drm_resume(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = to_i915(dev);
	int ret;

	disable_rpm_wakeref_asserts(&dev_priv->runtime_pm);

	ret = i915_pcode_init(dev_priv);
	if (ret)
		return ret;

	intel_dmc_ucode_resume(dev_priv);

	i915_restore_display(dev_priv);
	intel_pps_unlock_regs_wa(dev_priv);

	intel_init_pch_refclk(dev_priv);

	/*
	 * Interrupts have to be enabled before any batches are run. If not the
	 * GPU will hang. i915_gem_init_hw() will initiate batches to
	 * update/restore the context.
	 *
	 * drm_mode_config_reset() needs AUX interrupts.
	 *
	 * Modeset enabling in intel_modeset_init_hw() also needs working
	 * interrupts.
	 */
	intel_runtime_pm_enable_interrupts(dev_priv);

	if (HAS_DISPLAY(dev_priv))
		drm_mode_config_reset(dev);

	i915_gem_resume(dev_priv);

	intel_modeset_init_hw(dev_priv);
	intel_init_clock_gating(dev_priv);
	intel_hpd_init(dev_priv);

	/* MST sideband requires HPD interrupts enabled */
	intel_dp_mst_resume(dev_priv);
	intel_display_resume(dev);

	intel_hpd_poll_disable(dev_priv);
	if (HAS_DISPLAY(dev_priv))
		drm_kms_helper_poll_enable(dev);

	intel_opregion_resume(dev_priv);

	intel_fbdev_set_suspend(dev, FBINFO_STATE_RUNNING, false);

	intel_power_domains_enable(dev_priv);

	intel_gvt_resume(dev_priv);

	pvc_wa_allow_rc6(dev_priv);

	enable_rpm_wakeref_asserts(&dev_priv->runtime_pm);

	return 0;
}

static int i915_drm_resume_early(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = to_i915(dev);
	struct pci_dev *pdev = to_pci_dev(dev_priv->drm.dev);
	struct intel_gt *gt;
	int ret, i;

	/*
	 * We have a resume ordering issue with the snd-hda driver also
	 * requiring our device to be power up. Due to the lack of a
	 * parent/child relationship we currently solve this with an early
	 * resume hook.
	 *
	 * FIXME: This should be solved with a special hdmi sink device or
	 * similar so that power domains can be employed.
	 */

	/*
	 * Note that we need to set the power state explicitly, since we
	 * powered off the device during freeze and the PCI core won't power
	 * it back up for us during thaw. Powering off the device during
	 * freeze is not a hard requirement though, and during the
	 * suspend/resume phases the PCI core makes sure we get here with the
	 * device powered on. So in case we change our freeze logic and keep
	 * the device powered we can also remove the following set power state
	 * call.
	 */
	ret = pci_set_power_state(pdev, PCI_D0);
	if (ret) {
		drm_err(&dev_priv->drm,
			"failed to set PCI D0 power state (%d)\n", ret);
		return ret;
	}

	/*
	 * Note that pci_enable_device() first enables any parent bridge
	 * device and only then sets the power state for this device. The
	 * bridge enabling is a nop though, since bridge devices are resumed
	 * first. The order of enabling power and enabling the device is
	 * imposed by the PCI core as described above, so here we preserve the
	 * same order for the freeze/thaw phases.
	 *
	 * TODO: eventually we should remove pci_disable_device() /
	 * pci_enable_enable_device() from suspend/resume. Due to how they
	 * depend on the device enable refcount we can't anyway depend on them
	 * disabling/enabling the device.
	 */
	if (pci_enable_device(pdev))
		return -EIO;

	pci_set_master(pdev);

	i915_load_pci_state(pdev);

	disable_rpm_wakeref_asserts(&dev_priv->runtime_pm);

	pvc_wa_disallow_rc6(dev_priv);

	/*
	 * As soon as we can talk to the device, check the local memory.
	 *
	 * This is a destructive process, so we need to run before we start
	 * repopulating the device.
	 */
	ret = intel_memory_regions_resume_early(dev_priv);
	if (ret) {
		/* if error, allow rc6 as we disallowed at entry */
		pvc_wa_allow_rc6(dev_priv);
		drm_err(&dev_priv->drm, "Memory health check failed\n");
		goto out;
	}

	ret = vlv_resume_prepare(dev_priv, false);
	if (ret)
		drm_err(&dev_priv->drm,
			"Resume prepare failed: %d, continuing anyway\n", ret);

	for_each_gt(gt, dev_priv, i) {
		intel_uncore_resume_early(gt->uncore);
		intel_gt_check_and_clear_faults(gt);
	}

	intel_display_power_resume_early(dev_priv);

	intel_power_domains_resume(dev_priv);

	sanitize_gpu(dev_priv);

	ret = i915_ggtt_enable_hw(dev_priv);
	if (ret)
		drm_err(&dev_priv->drm, "failed to re-enable GGTT\n");

	i915_gem_resume_early(dev_priv);
	/* Must be called after GGTT is resumed. */
	intel_dpt_resume(dev_priv);

	i915_sriov_resume_early(dev_priv);

out:
	enable_rpm_wakeref_asserts(&dev_priv->runtime_pm);

	return ret;
}

int i915_driver_resume_switcheroo(struct drm_i915_private *i915)
{
	int ret;

	if (i915->drm.switch_power_state == DRM_SWITCH_POWER_OFF)
		return 0;

	ret = i915_drm_resume_early(&i915->drm);
	if (ret)
		return ret;

	return i915_drm_resume(&i915->drm);
}

static int i915_pm_prepare(struct device *kdev)
{
	struct drm_i915_private *i915 = kdev_to_i915(kdev);

	if (!i915) {
		dev_err(kdev, "DRM not initialized, aborting suspend.\n");
		return -ENODEV;
	}

	if (i915->drm.switch_power_state == DRM_SWITCH_POWER_OFF)
		return 0;

	return i915_drm_prepare(&i915->drm);
}

static int i915_pm_suspend(struct device *kdev)
{
	struct drm_i915_private *i915 = kdev_to_i915(kdev);

	if (!i915) {
		dev_err(kdev, "DRM not initialized, aborting suspend.\n");
		return -ENODEV;
	}

	if (i915->drm.switch_power_state == DRM_SWITCH_POWER_OFF)
		return 0;

	return i915_drm_suspend(&i915->drm);
}

static int i915_pm_suspend_late(struct device *kdev)
{
	struct drm_i915_private *i915 = kdev_to_i915(kdev);

	/*
	 * We have a suspend ordering issue with the snd-hda driver also
	 * requiring our device to be power up. Due to the lack of a
	 * parent/child relationship we currently solve this with an late
	 * suspend hook.
	 *
	 * FIXME: This should be solved with a special hdmi sink device or
	 * similar so that power domains can be employed.
	 */
	if (i915->drm.switch_power_state == DRM_SWITCH_POWER_OFF)
		return 0;

	return i915_drm_suspend_late(&i915->drm, false);
}

static int i915_pm_poweroff_late(struct device *kdev)
{
	struct drm_i915_private *i915 = kdev_to_i915(kdev);

	if (i915->drm.switch_power_state == DRM_SWITCH_POWER_OFF)
		return 0;

	return i915_drm_suspend_late(&i915->drm, true);
}

static int i915_pm_resume_early(struct device *kdev)
{
	struct drm_i915_private *i915 = kdev_to_i915(kdev);

	if (i915->drm.switch_power_state == DRM_SWITCH_POWER_OFF)
		return 0;

	return i915_drm_resume_early(&i915->drm);
}

static int i915_pm_resume(struct device *kdev)
{
	struct drm_i915_private *i915 = kdev_to_i915(kdev);

	if (i915->drm.switch_power_state == DRM_SWITCH_POWER_OFF)
		return 0;

	return i915_drm_resume(&i915->drm);
}

/* freeze: before creating the hibernation_image */
static int i915_pm_freeze(struct device *kdev)
{
	struct drm_i915_private *i915 = kdev_to_i915(kdev);
	int ret;

	if (i915->drm.switch_power_state != DRM_SWITCH_POWER_OFF) {
		ret = i915_drm_suspend(&i915->drm);
		if (ret)
			return ret;
	}

	ret = i915_gem_freeze(i915);
	if (ret)
		return ret;

	return 0;
}

static int i915_pm_freeze_late(struct device *kdev)
{
	struct drm_i915_private *i915 = kdev_to_i915(kdev);
	int ret;

	if (i915->drm.switch_power_state != DRM_SWITCH_POWER_OFF) {
		ret = i915_drm_suspend_late(&i915->drm, true);
		if (ret)
			return ret;
	}

	ret = i915_gem_freeze_late(i915);
	if (ret)
		return ret;

	return 0;
}

/* thaw: called after creating the hibernation image, but before turning off. */
static int i915_pm_thaw_early(struct device *kdev)
{
	return i915_pm_resume_early(kdev);
}

static int i915_pm_thaw(struct device *kdev)
{
	return i915_pm_resume(kdev);
}

/* restore: called after loading the hibernation image. */
static int i915_pm_restore_early(struct device *kdev)
{
	return i915_pm_resume_early(kdev);
}

static int i915_pm_restore(struct device *kdev)
{
	return i915_pm_resume(kdev);
}

static int intel_runtime_suspend(struct device *kdev)
{
	struct drm_i915_private *dev_priv = kdev_to_i915(kdev);
	struct intel_runtime_pm *rpm = &dev_priv->runtime_pm;
	struct intel_gt *gt;
	int ret, i;

	if (drm_WARN_ON_ONCE(&dev_priv->drm, !HAS_RUNTIME_PM(dev_priv)))
		return -ENODEV;

	drm_dbg(&dev_priv->drm, "Suspending device\n");

	disable_rpm_wakeref_asserts(rpm);

	pvc_wa_disallow_rc6(dev_priv);

	/*
	 * We are safe here against re-faults, since the fault handler takes
	 * an RPM reference.
	 */
	i915_gem_runtime_suspend(dev_priv);

	for_each_gt(gt, dev_priv, i)
		intel_gt_runtime_suspend(gt);

	pvc_wa_allow_rc6(dev_priv);

	intel_runtime_pm_disable_interrupts(dev_priv);

	for_each_gt(gt, dev_priv, i)
		intel_uncore_suspend(gt->uncore);

	intel_display_power_suspend(dev_priv);

	ret = vlv_suspend_complete(dev_priv);
	if (ret) {
		drm_err(&dev_priv->drm,
			"Runtime suspend failed, disabling it (%d)\n", ret);
		for_each_gt(gt, dev_priv, i)
			intel_uncore_runtime_resume(gt->uncore);

		intel_runtime_pm_enable_interrupts(dev_priv);

		for_each_gt(gt, dev_priv, i)
			intel_gt_runtime_resume(gt);

		enable_rpm_wakeref_asserts(rpm);

		return ret;
	}

	enable_rpm_wakeref_asserts(rpm);
	intel_runtime_pm_driver_release(rpm);

	if (intel_uncore_arm_unclaimed_mmio_detection(&dev_priv->uncore))
		drm_err(&dev_priv->drm,
			"Unclaimed access detected prior to suspending\n");

	rpm->suspended = true;

	/*
	 * FIXME: We really should find a document that references the arguments
	 * used below!
	 */
	if (IS_BROADWELL(dev_priv)) {
		/*
		 * On Broadwell, if we use PCI_D1 the PCH DDI ports will stop
		 * being detected, and the call we do at intel_runtime_resume()
		 * won't be able to restore them. Since PCI_D3hot matches the
		 * actual specification and appears to be working, use it.
		 */
		intel_opregion_notify_adapter(dev_priv, PCI_D3hot);
	} else {
		/*
		 * current versions of firmware which depend on this opregion
		 * notification have repurposed the D1 definition to mean
		 * "runtime suspended" vs. what you would normally expect (D3)
		 * to distinguish it from notifications that might be sent via
		 * the suspend path.
		 */
		intel_opregion_notify_adapter(dev_priv, PCI_D1);
	}

	assert_forcewakes_inactive(&dev_priv->uncore);

	if (!IS_VALLEYVIEW(dev_priv) && !IS_CHERRYVIEW(dev_priv))
		intel_hpd_poll_enable(dev_priv);

	drm_dbg(&dev_priv->drm, "Device suspended\n");
	return 0;
}

static int intel_runtime_resume(struct device *kdev)
{
	struct drm_i915_private *dev_priv = kdev_to_i915(kdev);
	struct intel_runtime_pm *rpm = &dev_priv->runtime_pm;
	struct intel_gt *gt;
	int ret, i;

	if (drm_WARN_ON_ONCE(&dev_priv->drm, !HAS_RUNTIME_PM(dev_priv)))
		return -ENODEV;

	drm_dbg(&dev_priv->drm, "Resuming device\n");

	drm_WARN_ON_ONCE(&dev_priv->drm, atomic_read(&rpm->wakeref_count));
	disable_rpm_wakeref_asserts(rpm);

	intel_opregion_notify_adapter(dev_priv, PCI_D0);
	rpm->suspended = false;
	if (intel_uncore_unclaimed_mmio(&dev_priv->uncore))
		drm_dbg(&dev_priv->drm,
			"Unclaimed access during suspend, bios?\n");

	intel_display_power_resume(dev_priv);

	ret = vlv_resume_prepare(dev_priv, true);

	for_each_gt(gt, dev_priv, i)
		intel_uncore_runtime_resume(gt->uncore);

	intel_runtime_pm_enable_interrupts(dev_priv);

	pvc_wa_disallow_rc6(dev_priv);

	/*
	 * No point of rolling back things in case of an error, as the best
	 * we can do is to hope that things will still work (and disable RPM).
	 */
	for_each_gt(gt, dev_priv, i)
		intel_gt_runtime_resume(gt);

	/*
	 * On VLV/CHV display interrupts are part of the display
	 * power well, so hpd is reinitialized from there. For
	 * everyone else do it here.
	 */
	if (!IS_VALLEYVIEW(dev_priv) && !IS_CHERRYVIEW(dev_priv)) {
		intel_hpd_init(dev_priv);
		intel_hpd_poll_disable(dev_priv);
	}

	intel_enable_ipc(dev_priv);

	pvc_wa_allow_rc6(dev_priv);

	enable_rpm_wakeref_asserts(rpm);

	if (ret)
		drm_err(&dev_priv->drm,
			"Runtime resume failed, disabling it (%d)\n", ret);
	else
		drm_dbg(&dev_priv->drm, "Device resumed\n");

	return ret;
}

const struct dev_pm_ops i915_pm_ops = {
	/*
	 * S0ix (via system suspend) and S3 event handlers [PMSG_SUSPEND,
	 * PMSG_RESUME]
	 */
	.prepare = i915_pm_prepare,
	.suspend = i915_pm_suspend,
	.suspend_late = i915_pm_suspend_late,
	.resume_early = i915_pm_resume_early,
	.resume = i915_pm_resume,

	/*
	 * S4 event handlers
	 * @freeze, @freeze_late    : called (1) before creating the
	 *                            hibernation image [PMSG_FREEZE] and
	 *                            (2) after rebooting, before restoring
	 *                            the image [PMSG_QUIESCE]
	 * @thaw, @thaw_early       : called (1) after creating the hibernation
	 *                            image, before writing it [PMSG_THAW]
	 *                            and (2) after failing to create or
	 *                            restore the image [PMSG_RECOVER]
	 * @poweroff, @poweroff_late: called after writing the hibernation
	 *                            image, before rebooting [PMSG_HIBERNATE]
	 * @restore, @restore_early : called after rebooting and restoring the
	 *                            hibernation image [PMSG_RESTORE]
	 */
	.freeze = i915_pm_freeze,
	.freeze_late = i915_pm_freeze_late,
	.thaw_early = i915_pm_thaw_early,
	.thaw = i915_pm_thaw,
	.poweroff = i915_pm_suspend,
	.poweroff_late = i915_pm_poweroff_late,
	.restore_early = i915_pm_restore_early,
	.restore = i915_pm_restore,

	/* S0ix (via runtime suspend) event handlers */
	.runtime_suspend = intel_runtime_suspend,
	.runtime_resume = intel_runtime_resume,
};

static const struct file_operations i915_driver_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.release = drm_release_noglobal,
	.unlocked_ioctl = drm_ioctl,
	.mmap = i915_gem_mmap,
	.poll = drm_poll,
	.read = drm_read,
	.compat_ioctl = i915_ioc32_compat_ioctl,
	.llseek = noop_llseek,
};

static int
i915_gem_reject_pin_ioctl(struct drm_device *dev, void *data,
			  struct drm_file *file)
{
	return -ENODEV;
}

static int i915_gem_vm_bind_ioctl(struct drm_device *dev, void *data,
				  struct drm_file *file)
{
	struct prelim_drm_i915_gem_vm_bind *args = data;
	struct i915_address_space *vm;
	int ret;

	vm = i915_address_space_lookup(file->driver_priv, args->vm_id);
	if (unlikely(!vm))
		return -ENOENT;

	if (!(args->flags & PRELIM_I915_GEM_VM_BIND_FD))
		ret = i915_gem_vm_bind_obj(vm, args, file);
	else
		ret = -EINVAL;

	i915_vm_put(vm);
	return ret;
}

static int i915_gem_vm_unbind_ioctl(struct drm_device *dev, void *data,
				    struct drm_file *file)
{
	struct prelim_drm_i915_gem_vm_bind *args = data;
	struct i915_address_space *vm;
	int ret;

	vm = i915_address_space_lookup(file->driver_priv, args->vm_id);
	if (unlikely(!vm))
		return -ENOENT;

	if (!(args->flags & PRELIM_I915_GEM_VM_BIND_FD))
		ret = i915_gem_vm_unbind_obj(vm, args);
	else
		ret = -EINVAL;

	i915_vm_put(vm);
	return ret;
}

static int i915_gem_vm_advise_ioctl(struct drm_device *dev, void *data,
				    struct drm_file *file)
{
	struct prelim_drm_i915_gem_vm_advise *args = data;
	struct drm_i915_gem_object *obj;
	int ret;

	/* XXX page granularity and system allocator support not available */
	if (!args->handle)
		return -ENOTSUPP;

	obj = i915_gem_object_lookup(file, args->handle);
	if (!obj)
		return -ENOENT;

	ret = i915_gem_object_set_hint(obj, args);
	i915_gem_object_put(obj);

	return ret;
}

static int i915_runtime_vm_prefetch(struct drm_i915_private *i915,
			struct prelim_drm_i915_gem_vm_prefetch *args,
			struct drm_i915_file_private *file_priv)
{
	struct intel_memory_region *mem;
	struct i915_address_space *vm;
	struct drm_mm_node *node;
	struct i915_vma *vma;
	u16 class, instance;
	u64 start, end;
	int err = 0;

	class = args->region >> 16;
	instance = args->region & 0xffff;
	mem = intel_memory_region_lookup(i915, class, instance);
	if (!mem)
		return -EINVAL;

	vm = i915_address_space_lookup(file_priv, args->vm_id);
	if (unlikely(!vm))
		return -ENOENT;

	start = intel_noncanonical_addr(INTEL_PPGTT_MSB(vm->i915),
					args->start);
	if (range_overflows(start, args->length, vm->total))
		return -EINVAL;

	end = start + args->length;
	trace_i915_vm_prefetch(mem, args->vm_id, start, args->length);

	mutex_lock(&vm->mutex);
	node = __drm_mm_interval_first(&vm->mm, start, end-1);
	while (node->start < end) {
		GEM_BUG_ON(!drm_mm_node_allocated(node));
		start = node->start + node->size;

		vma = container_of(node, typeof(*vma), node);
		vma = __i915_vma_get(vma);
		if (!vma)
			continue;
		vma = i915_vma_get(vma);
		/**
		 * Prefetch is best effort. Even if we fail to prefetch one vma, we will
		 * proceed with other vmas.
		 */
		mutex_unlock(&vm->mutex);
		i915_vma_prefetch(vma, mem);
		mutex_lock(&vm->mutex);
		i915_vma_put(vma);
		__i915_vma_put(vma);

		node = __drm_mm_interval_first(&vm->mm, start, end-1);
	}
	mutex_unlock(&vm->mutex);

	i915_vm_put(vm);
	return err;
}

static int i915_gem_vm_prefetch_ioctl(struct drm_device *dev, void *data,
			       struct drm_file *file_priv)
{
	struct drm_i915_private *i915 = to_i915(dev);
	struct prelim_drm_i915_gem_vm_prefetch *args = data;

	if (!args->vm_id)
		return i915_svm_vm_prefetch(i915, args);
	else
		return i915_runtime_vm_prefetch(i915, args, file_priv->driver_priv);
}

static const struct drm_ioctl_desc i915_ioctls[] = {
	DRM_IOCTL_DEF_DRV(I915_INIT, drm_noop, DRM_AUTH|DRM_MASTER|DRM_ROOT_ONLY),
	DRM_IOCTL_DEF_DRV(I915_FLUSH, drm_noop, DRM_AUTH),
	DRM_IOCTL_DEF_DRV(I915_FLIP, drm_noop, DRM_AUTH),
	DRM_IOCTL_DEF_DRV(I915_BATCHBUFFER, drm_noop, DRM_AUTH),
	DRM_IOCTL_DEF_DRV(I915_IRQ_EMIT, drm_noop, DRM_AUTH),
	DRM_IOCTL_DEF_DRV(I915_IRQ_WAIT, drm_noop, DRM_AUTH),
	DRM_IOCTL_DEF_DRV(I915_GETPARAM, i915_getparam_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(I915_SETPARAM, drm_noop, DRM_AUTH|DRM_MASTER|DRM_ROOT_ONLY),
	DRM_IOCTL_DEF_DRV(I915_ALLOC, drm_noop, DRM_AUTH),
	DRM_IOCTL_DEF_DRV(I915_FREE, drm_noop, DRM_AUTH),
	DRM_IOCTL_DEF_DRV(I915_INIT_HEAP, drm_noop, DRM_AUTH|DRM_MASTER|DRM_ROOT_ONLY),
	DRM_IOCTL_DEF_DRV(I915_CMDBUFFER, drm_noop, DRM_AUTH),
	DRM_IOCTL_DEF_DRV(I915_DESTROY_HEAP,  drm_noop, DRM_AUTH|DRM_MASTER|DRM_ROOT_ONLY),
	DRM_IOCTL_DEF_DRV(I915_SET_VBLANK_PIPE,  drm_noop, DRM_AUTH|DRM_MASTER|DRM_ROOT_ONLY),
	DRM_IOCTL_DEF_DRV(I915_GET_VBLANK_PIPE,  drm_noop, DRM_AUTH),
	DRM_IOCTL_DEF_DRV(I915_VBLANK_SWAP, drm_noop, DRM_AUTH),
	DRM_IOCTL_DEF_DRV(I915_HWS_ADDR, drm_noop, DRM_AUTH|DRM_MASTER|DRM_ROOT_ONLY),
	DRM_IOCTL_DEF_DRV(I915_GEM_INIT, drm_noop, DRM_AUTH|DRM_MASTER|DRM_ROOT_ONLY),
	DRM_IOCTL_DEF_DRV(I915_GEM_EXECBUFFER, drm_invalid_op, DRM_AUTH),
	DRM_IOCTL_DEF_DRV(I915_GEM_EXECBUFFER2_WR, i915_gem_execbuffer2_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(I915_GEM_PIN, i915_gem_reject_pin_ioctl, DRM_AUTH|DRM_ROOT_ONLY),
	DRM_IOCTL_DEF_DRV(I915_GEM_UNPIN, i915_gem_reject_pin_ioctl, DRM_AUTH|DRM_ROOT_ONLY),
	DRM_IOCTL_DEF_DRV(I915_GEM_BUSY, i915_gem_busy_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(I915_GEM_SET_CACHING, i915_gem_set_caching_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(I915_GEM_GET_CACHING, i915_gem_get_caching_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(I915_GEM_THROTTLE, i915_gem_throttle_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(I915_GEM_ENTERVT, drm_noop, DRM_AUTH|DRM_MASTER|DRM_ROOT_ONLY),
	DRM_IOCTL_DEF_DRV(I915_GEM_LEAVEVT, drm_noop, DRM_AUTH|DRM_MASTER|DRM_ROOT_ONLY),
	DRM_IOCTL_DEF_DRV(I915_GEM_CREATE_EXT, i915_gem_create_ext_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(I915_GEM_PREAD, i915_gem_pread_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(I915_GEM_PWRITE, i915_gem_pwrite_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(I915_GEM_MMAP, i915_gem_mmap_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(I915_GEM_MMAP_OFFSET, i915_gem_mmap_offset_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(I915_GEM_SET_DOMAIN, i915_gem_set_domain_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(I915_GEM_SW_FINISH, i915_gem_sw_finish_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(I915_GEM_SET_TILING, i915_gem_set_tiling_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(I915_GEM_GET_TILING, i915_gem_get_tiling_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(I915_GEM_GET_APERTURE, i915_gem_get_aperture_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(I915_GET_PIPE_FROM_CRTC_ID, intel_get_pipe_from_crtc_id_ioctl, 0),
	DRM_IOCTL_DEF_DRV(I915_GEM_MADVISE, i915_gem_madvise_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(I915_OVERLAY_PUT_IMAGE, intel_overlay_put_image_ioctl, DRM_MASTER),
	DRM_IOCTL_DEF_DRV(I915_OVERLAY_ATTRS, intel_overlay_attrs_ioctl, DRM_MASTER),
	DRM_IOCTL_DEF_DRV(I915_SET_SPRITE_COLORKEY, intel_sprite_set_colorkey_ioctl, DRM_MASTER),
	DRM_IOCTL_DEF_DRV(I915_GET_SPRITE_COLORKEY, drm_noop, DRM_MASTER),
	DRM_IOCTL_DEF_DRV(I915_GEM_WAIT, i915_gem_wait_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(I915_GEM_CONTEXT_CREATE_EXT, i915_gem_context_create_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(I915_GEM_CONTEXT_DESTROY, i915_gem_context_destroy_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(I915_REG_READ, i915_reg_read_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(I915_GET_RESET_STATS, i915_gem_context_reset_stats_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(I915_GEM_USERPTR, i915_gem_userptr_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(I915_GEM_CONTEXT_GETPARAM, i915_gem_getparam_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(I915_GEM_CONTEXT_SETPARAM, i915_gem_setparam_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(I915_PERF_OPEN, i915_perf_open_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(I915_PERF_ADD_CONFIG, i915_perf_add_config_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(I915_PERF_REMOVE_CONFIG, i915_perf_remove_config_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(I915_QUERY, i915_query_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(I915_GEM_VM_CREATE, i915_gem_vm_create_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(I915_GEM_VM_DESTROY, i915_gem_vm_destroy_ioctl, DRM_RENDER_ALLOW),

	PRELIM_DRM_IOCTL_DEF_DRV(I915_GEM_CREATE_EXT, i915_gem_create_ioctl, DRM_RENDER_ALLOW),
	PRELIM_DRM_IOCTL_DEF_DRV(I915_GEM_VM_BIND, i915_gem_vm_bind_ioctl, DRM_RENDER_ALLOW),
	PRELIM_DRM_IOCTL_DEF_DRV(I915_GEM_VM_UNBIND, i915_gem_vm_unbind_ioctl, DRM_RENDER_ALLOW),
	PRELIM_DRM_IOCTL_DEF_DRV(I915_GEM_VM_ADVISE, i915_gem_vm_advise_ioctl, DRM_RENDER_ALLOW),
	PRELIM_DRM_IOCTL_DEF_DRV(I915_GEM_WAIT_USER_FENCE, i915_gem_wait_user_fence_ioctl, DRM_RENDER_ALLOW),
	PRELIM_DRM_IOCTL_DEF_DRV(I915_UUID_REGISTER, i915_uuid_register_ioctl, DRM_RENDER_ALLOW),
	PRELIM_DRM_IOCTL_DEF_DRV(I915_UUID_UNREGISTER, i915_uuid_unregister_ioctl, DRM_RENDER_ALLOW),
	PRELIM_DRM_IOCTL_DEF_DRV(I915_DEBUGGER_OPEN, i915_debugger_open_ioctl, DRM_RENDER_ALLOW),
	PRELIM_DRM_IOCTL_DEF_DRV(I915_GEM_CLOS_RESERVE, i915_gem_clos_reserve_ioctl, DRM_RENDER_ALLOW),
	PRELIM_DRM_IOCTL_DEF_DRV(I915_GEM_CLOS_FREE, i915_gem_clos_free_ioctl, DRM_RENDER_ALLOW),
	PRELIM_DRM_IOCTL_DEF_DRV(I915_GEM_CACHE_RESERVE, i915_gem_cache_reserve_ioctl, DRM_RENDER_ALLOW),
	PRELIM_DRM_IOCTL_DEF_DRV(I915_GEM_VM_PREFETCH, i915_gem_vm_prefetch_ioctl, DRM_RENDER_ALLOW),
};

/*
 * Interface history:
 *
 * 1.1: Original.
 * 1.2: Add Power Management
 * 1.3: Add vblank support
 * 1.4: Fix cmdbuffer path, add heap destroy
 * 1.5: Add vblank pipe configuration
 * 1.6: - New ioctl for scheduling buffer swaps on vertical blank
 *      - Support vertical blank on secondary display pipe
 */
#define DRIVER_MAJOR		1
#define DRIVER_MINOR		6
#define DRIVER_PATCHLEVEL	0

static const struct drm_driver i915_drm_driver = {
	/* Don't use MTRRs here; the Xserver or userspace app should
	 * deal with them for Intel hardware.
	 */
	.driver_features =
	    DRIVER_GEM |
	    DRIVER_RENDER | DRIVER_MODESET | DRIVER_ATOMIC | DRIVER_SYNCOBJ |
	    DRIVER_SYNCOBJ_TIMELINE,
	.release = i915_driver_release,
	.open = i915_driver_open,
	.lastclose = i915_driver_lastclose,
	.postclose = i915_driver_postclose,

	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
	.gem_prime_import = i915_gem_prime_import,

	.dumb_create = i915_gem_dumb_create,
	.dumb_map_offset = i915_gem_dumb_mmap_offset,

	.ioctls = i915_ioctls,
	.num_ioctls = ARRAY_SIZE(i915_ioctls),
	.fops = &i915_driver_fops,
	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
	.patchlevel = DRIVER_PATCHLEVEL,
};
