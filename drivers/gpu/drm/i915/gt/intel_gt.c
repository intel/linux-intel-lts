// SPDX-License-Identifier: MIT
/*
 * Copyright © 2019 Intel Corporation
 */

#include <drm/drm_managed.h>
#include <drm/intel-gtt.h>
#include <drm/drm_managed.h>

#include "gem/i915_gem_internal.h"
#include "gem/i915_gem_lmem.h"
#include "gem/i915_gem_region.h"

#include "gen8_ppgtt.h"
#include "i915_drv.h"
#include "intel_context.h"
#include "intel_engine_regs.h"
#include "intel_flat_ppgtt_pool.h"
#include "intel_ggtt_gmch.h"
#include "intel_gt.h"
#include "intel_gt_buffer_pool.h"
#include "intel_gt_ccs_mode.h"
#include "intel_gt_clock_utils.h"
#include "intel_gt_debugfs.h"
#include "intel_gt_mcr.h"
#include "intel_gt_pm.h"
#include "intel_gt_print.h"
#include "intel_gt_regs.h"
#include "intel_gt_requests.h"
#include "intel_migrate.h"
#include "intel_mocs.h"
#include "intel_pci_config.h"
#include "intel_pm.h"
#include "intel_rc6.h"
#include "intel_renderstate.h"
#include "intel_rps.h"
#include "intel_sa_media.h"
#include "intel_tlb.h"
#include "intel_uncore.h"
#include "intel_pagefault.h"
#include "intel_pm.h"
#include "iov/intel_iov.h"
#include "shmem_utils.h"
#include "intel_gt_sysfs.h"
#include "pxp/intel_pxp.h"
#include "gt/iov/intel_iov_sysfs.h"

static const char *intel_gt_driver_errors_to_str[] = {
	[INTEL_GT_DRIVER_ERROR_GGTT] = "GGTT",
	[INTEL_GT_DRIVER_ERROR_ENGINE_OTHER] = "ENGINE OTHER",
	[INTEL_GT_DRIVER_ERROR_GUC_COMMUNICATION] = "GUC COMMUNICATION",
	[INTEL_GT_DRIVER_ERROR_RPS] = "RPS",
	[INTEL_GT_DRIVER_ERROR_GT_OTHER] = "GT OTHER",
	[INTEL_GT_DRIVER_ERROR_INTERRUPT] = "INTERRUPT",
};

void intel_gt_silent_driver_error(struct intel_gt *gt,
				  const enum intel_gt_driver_errors error)
{
	GEM_BUG_ON(error >= ARRAY_SIZE(gt->errors.driver));
	WRITE_ONCE(gt->errors.driver[error],
		   READ_ONCE(gt->errors.driver[error]) + 1);
}

void intel_gt_log_driver_error(struct intel_gt *gt,
			       const enum intel_gt_driver_errors error,
			       const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	intel_gt_silent_driver_error(gt, error);

	va_start(args, fmt);
	vaf.fmt = fmt;
	vaf.va = &args;

	BUILD_BUG_ON(ARRAY_SIZE(intel_gt_driver_errors_to_str) !=
		     INTEL_GT_DRIVER_ERROR_COUNT);

	drm_err_ratelimited(&gt->i915->drm, "GT%u [%s] %pV",
			    gt->info.id,
			    intel_gt_driver_errors_to_str[error],
			    &vaf);
	va_end(args);
}

void intel_gt_common_init_early(struct intel_gt *gt)
{
	gt->suspend = true;

	spin_lock_init(gt->irq_lock);

	init_llist_head(&gt->watchdog.list);
	INIT_WORK(&gt->watchdog.work, intel_gt_watchdog_work);

	mutex_init(&gt->eu_debug.lock);
	INIT_ACTIVE_FENCE(&gt->eu_debug.fault);
	xa_init(&gt->errors.soc);

	intel_gt_init_buffer_pool(gt);

	atomic_set(&gt->next_token, 0);

	intel_gt_init_ccs_mode(gt);
	intel_gt_init_reset(gt);
	intel_gt_init_requests(gt);
	intel_gt_init_timelines(gt);
	intel_gt_init_tlb(gt);
	intel_gt_pm_init_early(gt);

	intel_wopcm_init_early(&gt->wopcm);
	intel_flat_ppgtt_pool_init_early(&gt->fpp);
	intel_uc_init_early(&gt->uc);
	intel_rps_init_early(&gt->rps);

	i915_vma_clock_init_early(&gt->vma_clock);
}

/* Preliminary initialization of Tile 0 */
int intel_root_gt_init_early(struct drm_i915_private *i915)
{
	struct intel_gt *gt = to_root_gt(i915);

	gt->i915 = i915;
	gt->uncore = &i915->uncore;
	gt->irq_lock = drmm_kzalloc(&i915->drm, sizeof(*gt->irq_lock), GFP_KERNEL);
	if (!gt->irq_lock)
		return -ENOMEM;

	intel_gt_common_init_early(gt);

	return 0;
}

static unsigned int to_logical_instance(struct intel_gt *gt, unsigned int instance)
{
	struct drm_i915_private *i915 = gt->i915;

	if (IS_SRIOV_VF(i915) && HAS_REMOTE_TILES(i915))
		instance = hweight32(GENMASK(instance, 0) &
				     to_root_gt(i915)->iov.vf.config.tile_mask) - 1;
	return instance;
}

static int intel_gt_probe_lmem(struct intel_gt *gt)
{
	struct drm_i915_private *i915 = gt->i915;
	unsigned int instance = gt->info.id;
	int id = INTEL_REGION_LMEM_0 + instance;
	struct intel_memory_region *mem;
	int err;

	mem = intel_gt_setup_lmem(gt);
	if (IS_ERR(mem)) {
		err = PTR_ERR(mem);
		if (err == -ENODEV)
			return 0;

		gt_err(gt, "Failed to setup region(%d) type=%d\n",
		       err, INTEL_MEMORY_LOCAL);
		return err;
	}

	mem->id = id;
	mem->instance = to_logical_instance(gt, instance);
	mem->gt = gt;

	intel_memory_region_set_name(mem, "local%u", mem->instance);

	GEM_BUG_ON(!HAS_REGION(i915, id));
	GEM_BUG_ON(i915->mm.regions[id]);
	i915->mm.regions[id] = mem;
	gt->lmem = mem;

	return 0;
}

void intel_gt_init_ggtt(struct intel_gt *gt, struct i915_ggtt *ggtt)
{
	gt->ggtt = ggtt;
	list_add_tail(&gt->ggtt_link, &ggtt->gt_list);
}

int intel_gt_init_mmio(struct intel_gt *gt)
{
	intel_gt_init_clock_frequency(gt);
	intel_uc_init_mmio(&gt->uc);

	intel_sseu_info_init(gt);
	intel_gt_mcr_init(gt);

	return intel_engines_init_mmio(gt);
}

static void init_unused_ring(struct intel_gt *gt, u32 base)
{
	struct intel_uncore *uncore = gt->uncore;

	intel_uncore_write(uncore, RING_CTL(base), 0);
	intel_uncore_write(uncore, RING_HEAD(base), 0);
	intel_uncore_write(uncore, RING_TAIL(base), 0);
	intel_uncore_write(uncore, RING_START(base), 0);
}

static void init_unused_rings(struct intel_gt *gt)
{
	struct drm_i915_private *i915 = gt->i915;

	if (IS_I830(i915)) {
		init_unused_ring(gt, PRB1_BASE);
		init_unused_ring(gt, SRB0_BASE);
		init_unused_ring(gt, SRB1_BASE);
		init_unused_ring(gt, SRB2_BASE);
		init_unused_ring(gt, SRB3_BASE);
	} else if (GRAPHICS_VER(i915) == 2) {
		init_unused_ring(gt, SRB0_BASE);
		init_unused_ring(gt, SRB1_BASE);
	} else if (GRAPHICS_VER(i915) == 3) {
		init_unused_ring(gt, PRB1_BASE);
		init_unused_ring(gt, PRB2_BASE);
	}
}

int intel_gt_init_hw(struct intel_gt *gt)
{
	struct drm_i915_private *i915 = gt->i915;
	struct intel_uncore *uncore = gt->uncore;
	int ret;

	gt->last_init_time = ktime_get();

	/* Double layer security blanket, see i915_gem_init() */
	intel_uncore_forcewake_get(uncore, FORCEWAKE_ALL);

	if (HAS_EDRAM(i915) && GRAPHICS_VER(i915) < 9)
		intel_uncore_rmw(uncore, HSW_IDICR, 0, IDIHASHMSK(0xf));

	if (IS_HASWELL(i915))
		intel_uncore_write(uncore,
				   HSW_MI_PREDICATE_RESULT_2,
				   IS_HSW_GT3(i915) ?
				   LOWER_SLICE_ENABLED : LOWER_SLICE_DISABLED);

	/* Apply the GT workarounds... */
	intel_gt_apply_workarounds(gt);
	/* ...and determine whether they are sticking. */
	intel_gt_verify_workarounds(gt, "init");

	intel_gt_init_swizzling(gt);

	/* get CCS mode in sync between sw/hw */
	intel_gt_apply_ccs_mode(gt);

	/*
	 * At least 830 can leave some of the unused rings
	 * "active" (ie. head != tail) after resume which
	 * will prevent c3 entry. Makes sure all unused rings
	 * are totally idle.
	 */
	init_unused_rings(gt);

	ret = i915_ppgtt_init_hw(gt);
	if (ret) {
		gt_err(gt, "Enabling PPGTT failed (%d)\n", ret);
		goto out;
	}

	/*
	 * GuC DMA transfers are affected by MOCS programming on some
	 * platforms so make sure the MOCS table is initialised prior
	 * to loading the GuC firmware
	 */
	intel_mocs_init(gt);

	/* We can't enable contexts until all firmware is loaded */
	ret = intel_uc_init_hw(&gt->uc);
	if (ret) {
		gt_probe_error(gt, "Enabling uc failed (%d)\n", ret);
		goto out;
	}

	ret = intel_iov_init_hw(&gt->iov);
	if (unlikely(ret)) {
		i915_probe_error(i915, "Enabling IOV failed (%pe)\n",
				 ERR_PTR(ret));
		goto out;
	}

out:
	intel_uncore_forcewake_put(uncore, FORCEWAKE_ALL);
	return ret;
}

static void rmw_set(struct intel_uncore *uncore, i915_reg_t reg, u32 set)
{
	intel_uncore_rmw(uncore, reg, 0, set);
}

static void rmw_clear(struct intel_uncore *uncore, i915_reg_t reg, u32 clr)
{
	intel_uncore_rmw(uncore, reg, clr, 0);
}

static void clear_register(struct intel_uncore *uncore, i915_reg_t reg)
{
	intel_uncore_rmw(uncore, reg, 0, 0);
}

static void gen6_clear_engine_error_register(struct intel_engine_cs *engine)
{
	GEN6_RING_FAULT_REG_RMW(engine, RING_FAULT_VALID, 0);
	GEN6_RING_FAULT_REG_POSTING_READ(engine);
}

void
intel_gt_clear_error_registers(struct intel_gt *gt,
			       intel_engine_mask_t engine_mask)
{
	struct drm_i915_private *i915 = gt->i915;
	struct intel_uncore *uncore = gt->uncore;
	u32 eir;

	if (IS_SRIOV_VF(i915))
		return;

	if (GRAPHICS_VER(i915) != 2)
		clear_register(uncore, PGTBL_ER);

	if (GRAPHICS_VER(i915) < 4)
		clear_register(uncore, IPEIR(RENDER_RING_BASE));
	else
		clear_register(uncore, IPEIR_I965);

	clear_register(uncore, EIR);
	eir = intel_uncore_read(uncore, EIR);
	if (eir) {
		/*
		 * some errors might have become stuck,
		 * mask them.
		 */
		gt_dbg(gt, "EIR stuck: 0x%08x, masking\n", eir);
		rmw_set(uncore, EMR, eir);
		intel_uncore_write(uncore, GEN2_IIR,
				   I915_MASTER_ERROR_INTERRUPT);
	}

	if (GRAPHICS_VER_FULL(i915) >= IP_VER(12, 50)) {
		intel_gt_mcr_multicast_rmw(gt, XEHP_RING_FAULT_REG,
					   RING_FAULT_VALID, 0);
		intel_gt_mcr_read_any(gt, XEHP_RING_FAULT_REG);
	} else if (GRAPHICS_VER(i915) >= 12) {
		rmw_clear(uncore, GEN12_RING_FAULT_REG, RING_FAULT_VALID);
		intel_uncore_posting_read(uncore, GEN12_RING_FAULT_REG);
	} else if (GRAPHICS_VER(i915) >= 8) {
		rmw_clear(uncore, GEN8_RING_FAULT_REG, RING_FAULT_VALID);
		intel_uncore_posting_read(uncore, GEN8_RING_FAULT_REG);
	} else if (GRAPHICS_VER(i915) >= 6) {
		struct intel_engine_cs *engine;
		enum intel_engine_id id;

		for_each_engine_masked(engine, gt, engine_mask, id)
			gen6_clear_engine_error_register(engine);
	}
}

static void gen6_check_faults(struct intel_gt *gt)
{
	struct intel_engine_cs *engine;
	enum intel_engine_id id;
	u32 fault;

	for_each_engine(engine, gt, id) {
		fault = GEN6_RING_FAULT_REG_READ(engine);
		if (fault & RING_FAULT_VALID) {
			gt_dbg(gt, "Unexpected fault\n"
			       "\tAddr: 0x%08lx\n"
			       "\tAddress space: %s\n"
			       "\tSource ID: %d\n"
			       "\tLevel: %d\n",
			       fault & PAGE_MASK,
			       fault & RING_FAULT_GTTSEL_MASK ?
			       "GGTT" : "PPGTT",
			       RING_FAULT_SRCID(fault),
			       RING_FAULT_LEVEL(fault));
		}
	}
}

static void xehp_check_faults(struct intel_gt *gt)
{
	u32 fault;

	/*
	 * Although the fault register now lives in an MCR register range,
	 * the GAM registers are special and we only truly need to read
	 * the "primary" GAM instance rather than handling each instance
	 * individually.  intel_gt_mcr_read_any() will automatically steer
	 * toward the primary instance.
	 */
	fault = intel_gt_mcr_read_any(gt, XEHP_RING_FAULT_REG);
	if (fault & RING_FAULT_VALID) {
		u32 fault_data0, fault_data1;
		u64 fault_addr;

		fault_data0 = intel_gt_mcr_read_any(gt, XEHP_FAULT_TLB_DATA0);
		fault_data1 = intel_gt_mcr_read_any(gt, XEHP_FAULT_TLB_DATA1);

		fault_addr = ((u64)(fault_data1 & FAULT_VA_HIGH_BITS) << 44) |
			     ((u64)fault_data0 << 12);

		gt_dbg(gt, "Unexpected fault\n"
		       "\tAddr: 0x%08x_%08x\n"
		       "\tAddress space: %s\n"
		       "\tEngine ID: %d\n"
		       "\tSource ID: %d\n"
		       "\tLevel: %d\n",
		       upper_32_bits(fault_addr),
		       lower_32_bits(fault_addr),
		       fault_data1 & FAULT_GTT_SEL ? "GGTT" : "PPGTT",
		       GEN8_RING_FAULT_ENGINE_ID(fault),
		       RING_FAULT_SRCID(fault),
		       RING_FAULT_LEVEL(fault));
	}
}

static void gen8_check_faults(struct intel_gt *gt)
{
	struct intel_uncore *uncore = gt->uncore;
	i915_reg_t fault_reg, fault_data0_reg, fault_data1_reg;
	u32 fault;

	if (GRAPHICS_VER(gt->i915) >= 12) {
		fault_reg = GEN12_RING_FAULT_REG;
		fault_data0_reg = GEN12_FAULT_TLB_DATA0;
		fault_data1_reg = GEN12_FAULT_TLB_DATA1;
	} else {
		fault_reg = GEN8_RING_FAULT_REG;
		fault_data0_reg = GEN8_FAULT_TLB_DATA0;
		fault_data1_reg = GEN8_FAULT_TLB_DATA1;
	}

	fault = intel_uncore_read(uncore, fault_reg);
	if (fault & RING_FAULT_VALID) {
		u32 fault_data0, fault_data1;
		u64 fault_addr;

		fault_data0 = intel_uncore_read(uncore, fault_data0_reg);
		fault_data1 = intel_uncore_read(uncore, fault_data1_reg);

		fault_addr = ((u64)(fault_data1 & FAULT_VA_HIGH_BITS) << 44) |
			     ((u64)fault_data0 << 12);

		gt_dbg(gt, "Unexpected fault\n"
		       "\tAddr: 0x%08x_%08x\n"
		       "\tAddress space: %s\n"
		       "\tEngine ID: %d\n"
		       "\tSource ID: %d\n"
		       "\tLevel: %d\n",
		       upper_32_bits(fault_addr),
		       lower_32_bits(fault_addr),
		       fault_data1 & FAULT_GTT_SEL ? "GGTT" : "PPGTT",
		       GEN8_RING_FAULT_ENGINE_ID(fault),
		       RING_FAULT_SRCID(fault),
		       RING_FAULT_LEVEL(fault));
	}
}

void intel_gt_check_and_clear_faults(struct intel_gt *gt)
{
	struct drm_i915_private *i915 = gt->i915;

	if (gt->i915->quiesce_gpu)
		return;

	if (IS_SRIOV_VF(i915))
		return;

	/* From GEN8 onwards we only have one 'All Engine Fault Register' */
	if (GRAPHICS_VER_FULL(i915) >= IP_VER(12, 50))
		xehp_check_faults(gt);
	else if (GRAPHICS_VER(i915) >= 8)
		gen8_check_faults(gt);
	else if (GRAPHICS_VER(i915) >= 6)
		gen6_check_faults(gt);
	else
		return;

	intel_gt_clear_error_registers(gt, ALL_ENGINES);
}

void intel_gt_flush_ggtt_writes(struct intel_gt *gt)
{
	struct intel_uncore *uncore = gt->uncore;
	intel_wakeref_t wakeref;

	/*
	 * No actual flushing is required for the GTT write domain for reads
	 * from the GTT domain. Writes to it "immediately" go to main memory
	 * as far as we know, so there's no chipset flush. It also doesn't
	 * land in the GPU render cache.
	 *
	 * However, we do have to enforce the order so that all writes through
	 * the GTT land before any writes to the device, such as updates to
	 * the GATT itself.
	 *
	 * We also have to wait a bit for the writes to land from the GTT.
	 * An uncached read (i.e. mmio) seems to be ideal for the round-trip
	 * timing. This issue has only been observed when switching quickly
	 * between GTT writes and CPU reads from inside the kernel on recent hw,
	 * and it appears to only affect discrete GTT blocks (i.e. on LLC
	 * system agents we cannot reproduce this behaviour, until Cannonlake
	 * that was!).
	 */

	wmb();

	if (INTEL_INFO(gt->i915)->has_coherent_ggtt)
		return;

	intel_gt_chipset_flush(gt);

	with_intel_runtime_pm_if_in_use(uncore->rpm, wakeref) {
		unsigned long flags;

		spin_lock_irqsave(&uncore->lock, flags);
		intel_uncore_posting_read_fw(uncore,
					     RING_HEAD(RENDER_RING_BASE));
		spin_unlock_irqrestore(&uncore->lock, flags);
	}
}

void intel_gt_chipset_flush(struct intel_gt *gt)
{
	wmb();
	if (GRAPHICS_VER(gt->i915) < 6)
		intel_ggtt_gmch_flush();
}

void intel_gt_driver_register(struct intel_gt *gt)
{
	intel_gsc_init(&gt->gsc, gt->i915);

	intel_rps_driver_register(&gt->rps);

	intel_gt_debugfs_register(gt);
	intel_gt_sysfs_register(gt);
	intel_iov_sysfs_setup(&gt->iov);
	intel_iov_vf_get_wakeref_wa(&gt->iov);
}

static int intel_gt_init_scratch(struct intel_gt *gt, unsigned int size)
{
	struct drm_i915_private *i915 = gt->i915;
	struct drm_i915_gem_object *obj;
	struct i915_vma *vma;
	int ret;

	obj = intel_gt_object_create_lmem(gt, size, I915_BO_ALLOC_VOLATILE);
	if (IS_ERR(obj))
		obj = i915_gem_object_create_stolen(i915, size);
	if (IS_ERR(obj))
		obj = i915_gem_object_create_internal(i915, size);
	if (IS_ERR(obj)) {
		gt_err(gt, "Failed to allocate scratch page\n");
		return PTR_ERR(obj);
	}

	vma = i915_vma_instance(obj, &gt->ggtt->vm, NULL);
	if (IS_ERR(vma)) {
		ret = PTR_ERR(vma);
		goto err_unref;
	}

	ret = i915_ggtt_pin_for_gt(vma, NULL, 0, PIN_HIGH);
	if (ret)
		goto err_unref;

	ret = i915_vma_wait_for_bind(vma);
	if (ret)
		goto err_unpin;

	gt->scratch = i915_vma_make_unshrinkable(vma);

	return 0;

err_unpin:
	i915_vma_unpin(vma);
err_unref:
	i915_gem_object_put(obj);
	return ret;
}

static int intel_gt_init_counters(struct intel_gt *gt, unsigned int size)
{
	struct drm_i915_gem_object *obj;
	struct i915_gem_ww_ctx ww;
	struct i915_vma *vma;
	void *addr;
	int err;

	obj = intel_gt_object_create_lmem(gt, size, I915_BO_CPU_CLEAR);
	if (IS_ERR(obj))
		return PTR_ERR(obj);

	vma = i915_vma_instance(obj, &gt->ggtt->vm, NULL);
	if (IS_ERR(vma)) {
		err = PTR_ERR(vma);
		goto err_unref;
	}

	for_i915_gem_ww(&ww, err, false) {
		err = i915_gem_object_lock(obj, &ww);
		if (err)
			continue;

		err = i915_ggtt_pin_for_gt(vma, &ww, 0, PIN_HIGH);
		if (err)
			continue;

		addr = i915_gem_object_pin_map(obj, I915_MAP_WC);
		if (IS_ERR(addr)) {
			err = PTR_ERR(addr);
			continue;
		}
	}
	if (err)
		goto err_unref;

	err = i915_vma_wait_for_bind(vma);
	if (err)
		goto err_unpin;

	gt->counters.vma = i915_vma_make_unshrinkable(vma);
	gt->counters.map = addr;
	return 0;

err_unpin:
	i915_vma_unpin(vma);
err_unref:
	i915_gem_object_put(obj);
	return err;
}

static void intel_gt_fini_counters(struct intel_gt *gt)
{
	i915_vma_unpin_and_release(&gt->counters.vma, I915_VMA_RELEASE_MAP);
}

static void intel_gt_fini_scratch(struct intel_gt *gt)
{
	i915_vma_unpin_and_release(&gt->scratch, 0);
}

static void intel_gt_init_debug_pages(struct intel_gt *gt)
{
	struct drm_i915_private *i915 = gt->i915;
	struct drm_i915_gem_object *obj;
	struct i915_vma *vma;
	int count = i915->params.debug_pages & ~BIT(31);
	bool lmem = i915->params.debug_pages & BIT(31);
	u32 size = count << PAGE_SHIFT;
	void *vaddr;

	if (!count)
		return;

	if (lmem) {
		if (!HAS_LMEM(i915)) {
			drm_err(&i915->drm, "No LMEM, skipping debug pages\n");
			return;
		}

		obj = intel_gt_object_create_lmem(gt, size,
						  I915_BO_ALLOC_CONTIGUOUS |
						  I915_BO_ALLOC_VOLATILE);
	} else {
		obj = i915_gem_object_create_shmem(i915, size);
	}
	if (IS_ERR(obj)) {
		drm_err(&i915->drm, "Failed to allocate debug pages\n");
		return;
	}

	vaddr = i915_gem_object_pin_map_unlocked(obj, I915_MAP_WC);
	if (!vaddr)
		goto err_unref;

	memset(vaddr, 0, size);

	i915_gem_object_unpin_map(obj);

	vma = i915_vma_instance(obj, &gt->ggtt->vm, NULL);
	if (IS_ERR(vma))
		goto err_unref;

	if (i915_ggtt_pin_for_gt(vma, NULL, 0, PIN_HIGH))
		goto err_unref;

	if (i915_vma_wait_for_bind(vma))
		goto err_unpin;

	gt->dbg = i915_vma_make_unshrinkable(vma);

	drm_dbg(&i915->drm,
		"gt%u debug pages allocated in %s: ggtt=0x%08x, phys=0x%016llx, size=0x%zx\n",
		gt->info.id,
		obj->mm.region.mem->name,
		i915_ggtt_offset(vma),
		(u64)i915_gem_object_get_dma_address(obj, 0),
		obj->base.size);

	return;

err_unpin:
	i915_vma_unpin(vma);
err_unref:
	i915_gem_object_put(obj);
	drm_err(&i915->drm, "Failed to init debug pages\n");
	return;
}

static void intel_gt_fini_debug_pages(struct intel_gt *gt)
{
	if (gt->dbg)
		i915_vma_unpin_and_release(&gt->dbg, 0);
}

static struct i915_address_space *kernel_vm(struct intel_gt *gt)
{
	struct i915_ppgtt *ppgtt;
	int err;

	if (INTEL_PPGTT(gt->i915) <= INTEL_PPGTT_ALIASING)
		return i915_vm_get(&gt->ggtt->vm);

	ppgtt = i915_ppgtt_create(gt, 0);
	if (IS_ERR(ppgtt))
		return ERR_CAST(ppgtt);

	/* Setup a 1:1 mapping into our portion of lmem */
	if (gt->lmem) {
		gt->flat.start = round_down(gt->lmem->region.start, SZ_1G);
		gt->flat.size  = round_up(gt->lmem->region.end, SZ_1G);
		gt->flat.size -= gt->flat.start;
		gt->flat.color = I915_COLOR_UNEVICTABLE;
		drm_dbg(&gt->i915->drm,
			"Using flat ppGTT [%llx + %llx]\n",
			gt->flat.start, gt->flat.size);

		err = intel_flat_lmem_ppgtt_init(&ppgtt->vm, &gt->flat);
		if (err) {
			i915_vm_put(&ppgtt->vm);
			return ERR_PTR(err);
		}
	}

	return &ppgtt->vm;
}

static void release_vm(struct intel_gt *gt)
{
	struct i915_address_space *vm;

	vm = fetch_and_zero(&gt->vm);
	if (!vm)
		return;

	intel_flat_lmem_ppgtt_fini(vm, &gt->flat);
	i915_vm_put(vm);
}

/* FIXME Add documentation to API */
static struct i915_request *
switch_context_to(struct intel_context *ce, struct i915_request *from)
{
	struct i915_request *rq;
	int err;

	rq = i915_request_create(ce);
	if (IS_ERR(rq))
		return rq;

	err = i915_request_await_dma_fence(rq, &from->fence);
	if (err < 0) {
		i915_request_add(rq);
		return ERR_PTR(err);
	}

	i915_request_get(rq);
	i915_request_add(rq);

	return rq;
}

/* FIXME Add documentation to API */
static struct i915_request *load_default_context(struct intel_context *ce)
{
	struct intel_renderstate so;
	struct i915_request *rq;
	int err;

	err = intel_renderstate_init(&so, ce);
	if (err)
		return ERR_PTR(err);

	rq = i915_request_create(ce);
	if (IS_ERR(rq)) {
		err = PTR_ERR(rq);
		goto err_fini;
	}

	err = intel_engine_emit_ctx_wa(rq);
	if (err)
		goto err_rq;

	err = intel_renderstate_emit(&so, rq);
	if (err)
		goto err_rq;

	rq = i915_request_get(rq);
err_rq:
	i915_request_add(rq);
err_fini:
	intel_renderstate_fini(&so, ce);
	return err ? ERR_PTR(err) : rq;
}

/* FIXME Add documentation to API */
static struct i915_request *
record_default_context(struct intel_engine_cs *engine)
{
	struct i915_request *rq[2];
	struct i915_gem_ww_ctx ww;
	struct intel_context *ce;
	int err;

	/* We must be able to switch to something! */
	GEM_BUG_ON(!engine->kernel_context);

	ce = intel_context_create(engine);
	if (IS_ERR(ce))
		return ERR_CAST(ce);

	err = 0;
	for_i915_gem_ww(&ww, err, true)
		err = intel_context_pin_ww(ce, &ww);
	if (err) {
		rq[0] = ERR_PTR(err);
		goto out;
	}

	/* Prime the golden context with known good state */
	rq[0] = load_default_context(ce);
	if (IS_ERR(rq[0]))
		goto out_unpin;

	rq[1] = switch_context_to(engine->kernel_context, rq[0]);
	i915_request_put(rq[0]);
	if (IS_ERR(rq[1])) {
		rq[0] = rq[1];
		goto out_unpin;
	}

	/* Reload the golden context to record the effect of any indirect w/a */
	rq[0] = switch_context_to(ce, rq[1]);
	i915_request_put(rq[1]);
	if (IS_ERR(rq[0]))
		goto out_unpin;

	/*
	 * Keep the context referenced until after we read back the
	 * HW image. The reference is returned to the caller via
	 * rq->context.
	 */
	intel_context_get(ce);
out_unpin:
	intel_context_unpin(ce);
out:
	intel_context_put(ce);
	return rq[0];
}

static int __engines_record_defaults(struct intel_gt *gt)
{
	struct i915_request *requests[I915_NUM_ENGINES] = {};
	struct intel_engine_cs *engine;
	enum intel_engine_id id;
	int err = 0;

	/*
	 * As we reset the gpu during very early sanitisation, the current
	 * register state on the GPU should reflect its defaults values.
	 * We load a context onto the hw (with restore-inhibit), then switch
	 * over to a second context to save that default register state. We
	 * can then prime every new context with that state so they all start
	 * from the same default HW values.
	 */

	for_each_engine(engine, gt, id) {
		struct i915_request *rq;

		drm_dbg(&gt->i915->drm,
			"Record default context on %s", engine->name);

		rq = record_default_context(engine);
		if (IS_ERR(rq)) {
			err = PTR_ERR(rq);
			goto out;
		}

		requests[id] = rq;
	}

	/* Flush the default context image to memory, and enable powersaving. */
	if (intel_gt_wait_for_idle(gt, I915_GEM_IDLE_TIMEOUT)) {
		if (!i915_error_injected())
			intel_klog_error_capture(gt, (intel_engine_mask_t) ~0U);
		err = -EIO;
		goto out;
	}

	for (id = 0; id < ARRAY_SIZE(requests); id++) {
		struct i915_request *rq;
		struct file *state;

		rq = requests[id];
		if (!rq)
			continue;

		if (rq->fence.error) {
			err = -EIO;
			goto out;
		}

		GEM_BUG_ON(!test_bit(CONTEXT_ALLOC_BIT, &rq->context->flags));
		if (!rq->context->state)
			continue;

		/* Keep a copy of the state's backing pages; free the obj */
		state = shmem_create_from_object(rq->context->state->obj);
		if (IS_ERR(state)) {
			err = PTR_ERR(state);
			goto out;
		}
		rq->engine->default_state = state;
	}

out:
	/*
	 * If we have to abandon now, we expect the engines to be idle
	 * and ready to be torn-down. The quickest way we can accomplish
	 * this is by declaring ourselves wedged.
	 */
	for (id = 0; id < ARRAY_SIZE(requests); id++) {
		struct i915_request *rq;

		rq = requests[id];
		if (!rq)
			continue;

		intel_context_put(rq->context);
		i915_request_put(rq);
	}
	return err;
}

static int __engines_verify_workarounds(struct intel_gt *gt)
{
	struct intel_engine_cs *engine;
	enum intel_engine_id id;
	int err = 0;

	if (!IS_ENABLED(CONFIG_DRM_I915_DEBUG_GEM))
		return 0;

	for_each_engine(engine, gt, id) {
		if (intel_engine_verify_workarounds(engine, "load"))
			err = -EIO;
	}

	/* Flush and restore the kernel context for safety */
	if (intel_gt_wait_for_idle(gt, I915_GEM_IDLE_TIMEOUT))
		err = -EIO;

	return err;
}

static void __intel_gt_disable(struct intel_gt *gt)
{
	intel_gt_set_wedged_on_fini(gt);

	if (!gt->i915->quiesce_gpu) {
		intel_gt_suspend_prepare(gt);
		intel_gt_suspend_late(gt);
	}

	GEM_BUG_ON(intel_gt_pm_is_awake(gt));
}

int intel_gt_wait_for_idle(struct intel_gt *gt, long timeout)
{
	/* If the device is asleep, we have no requests outstanding */
	if (!intel_gt_pm_is_awake(gt))
		return 0;

	while (!intel_gt_retire_requests_timeout(gt, &timeout)) {
		if (signal_pending(current))
			return -EINTR;

		if (timeout == 0)
			return -ETIME;

		cond_resched();
	}

	return intel_uc_wait_for_idle(&gt->uc, timeout);
}

int intel_gt_init(struct intel_gt *gt)
{
	struct i915_address_space *vm;
	int err;

	err = i915_inject_probe_error(gt->i915, -ENODEV);
	if (err)
		return err;

	intel_gt_init_workarounds(gt);

	/*
	 * This is just a security blanket to placate dragons.
	 * On some systems, we very sporadically observe that the first TLBs
	 * used by the CS may be stale, despite us poking the TLB reset. If
	 * we hold the forcewake during initialisation these problems
	 * just magically go away.
	 */
	intel_uncore_forcewake_get(gt->uncore, FORCEWAKE_ALL);

	err = intel_iov_init(&gt->iov);
	if (unlikely(err))
		goto out_fw;

	err = intel_gt_init_scratch(gt,
				    GRAPHICS_VER(gt->i915) == 2 ? SZ_256K : SZ_4K);
	if (err)
		goto err_iov;

	err = intel_gt_init_counters(gt, SZ_4K);
	if (err && err != -ENODEV)
		goto err_scratch;

	intel_gt_init_debug_pages(gt);

	intel_gt_pm_init(gt);

	vm = kernel_vm(gt);
	if (IS_ERR(vm)) {
		err = PTR_ERR(vm);
		goto err_pm;
	}
	gt->vm = vm;

	intel_set_mocs_index(gt);

	err = intel_engines_init(gt);
	if (err)
		goto err_engines;

	err = intel_uc_init(&gt->uc);
	if (err)
		goto err_engines;

	err = intel_gt_resume(gt);
	if (err)
		goto err_uc_init;

	err = intel_gt_init_hwconfig(gt);
	if (err)
		gt_probe_error(gt, "Failed to retrieve hwconfig table: %pe\n", ERR_PTR(err));

	err = intel_iov_init_late(&gt->iov);
	if (err)
		goto err_gt;

	err = __engines_record_defaults(gt);
	if (err)
		goto err_gt;

	err = __engines_verify_workarounds(gt);
	if (err)
		goto err_gt;

	intel_uc_init_late(&gt->uc);

	err = i915_inject_probe_error(gt->i915, -EIO);
	if (err)
		goto err_gt;

	intel_pxp_init(&gt->pxp);

	/*
	 * FIXME: this should be moved to a delayed work because it takes too
	 * long, but for now we're doing it as the last step of the init flow
	 */
	intel_uc_init_hw_late(&gt->uc);

	goto out_fw;
err_gt:
	__intel_gt_disable(gt);
	intel_uc_fini_hw(&gt->uc);
err_uc_init:
	intel_uc_fini(&gt->uc);
err_engines:
	intel_engines_release(gt);
	release_vm(gt);
err_pm:
	intel_gt_pm_fini(gt);
	intel_gt_fini_debug_pages(gt);
	intel_gt_fini_counters(gt);
err_scratch:
	intel_gt_fini_scratch(gt);
err_iov:
	intel_iov_fini(&gt->iov);
out_fw:
	if (err)
		intel_gt_set_wedged_on_init(gt);
	intel_uncore_forcewake_put(gt->uncore, FORCEWAKE_ALL);
	return err;
}

void intel_gt_driver_remove(struct intel_gt *gt)
{
	intel_gt_fini_clock_frequency(gt);
	intel_flat_ppgtt_pool_fini(&gt->fpp);
	i915_vma_clock_flush(&gt->vma_clock);
	intel_iov_fini_hw(&gt->iov);

	__intel_gt_disable(gt);

	intel_uc_driver_remove(&gt->uc);

	intel_engines_release(gt);

	intel_gt_flush_buffer_pool(gt);
}

void intel_gt_driver_unregister(struct intel_gt *gt)
{
	intel_wakeref_t wakeref;

	intel_iov_vf_put_wakeref_wa(&gt->iov);

	if (!gt->i915->drm.unplugged)
		intel_iov_sysfs_teardown(&gt->iov);

	intel_gt_sysfs_unregister(gt);
	intel_rps_driver_unregister(&gt->rps);
	intel_gsc_fini(&gt->gsc);

	intel_pxp_fini(&gt->pxp);

	/*
	 * Upon unregistering the device to prevent any new users, cancel
	 * all in-flight requests so that we can quickly unbind the active
	 * resources.
	 */
	intel_gt_set_wedged_on_fini(gt);

	/* Scrub all HW state upon release */
	with_intel_runtime_pm(gt->uncore->rpm, wakeref)
		__intel_gt_reset(gt, ALL_ENGINES);

	xa_destroy(&gt->errors.soc);
}

void intel_gt_driver_release(struct intel_gt *gt)
{
	release_vm(gt);

	intel_wa_list_free(&gt->wa_list);
	intel_gt_pm_fini(gt);
	intel_gt_fini_debug_pages(gt);
	intel_gt_fini_counters(gt);
	intel_gt_fini_scratch(gt);
	intel_gt_fini_buffer_pool(gt);
	intel_gt_fini_hwconfig(gt);
	intel_iov_fini(&gt->iov);
	i915_vma_clock_fini(&gt->vma_clock);
}

void intel_gt_driver_late_release_all(struct drm_i915_private *i915)
{
	struct intel_gt *gt;
	unsigned int id;

	/* We need to wait for inflight RCU frees to release their grip */
	rcu_barrier();

	for_each_gt(gt, i915, id) {
		intel_iov_release(&gt->iov);
		intel_uc_driver_late_release(&gt->uc);
		intel_gt_fini_ccs_mode(gt);
		intel_gt_fini_requests(gt);
		intel_gt_fini_reset(gt);
		intel_gt_fini_timelines(gt);
		intel_gt_fini_tlb(gt);
		intel_engines_free(gt);
	}
}

static int driver_flr(struct intel_gt *gt)
{
	struct drm_i915_private *i915 = gt->i915;
	struct intel_uncore *uncore = gt->uncore;
	int ret;

	if (i915->params.force_driver_flr != 1)
		return 0;

	if (intel_uncore_read(uncore, GU_CNTL_PROTECTED) & DRIVERINT_FLR_DIS) {
		drm_info_once(&i915->drm, "BIOS Disabled Driver-FLR\n");
		return 0;
	}

	drm_dbg(&i915->drm, "Triggering Driver-FLR\n");

	/*
	 * As the fastest safe-measure, always clear GU_DEBUG's DRIVERFLR_STATUS
	 * if it was still set from a prior attempt
	 */
	intel_uncore_write_fw(uncore, GU_DEBUG, DRIVERFLR_STATUS);

	/* Trigger the actula Driver-FLR */
	intel_uncore_rmw_fw(uncore, GU_CNTL, 0, DRIVERFLR);

	ret = intel_wait_for_register_fw(uncore, GU_CNTL, DRIVERFLR, 0, 15);
	if (ret) {
		drm_err(&i915->drm, "Driver-FLR failed! %d\n", ret);
		return ret;
	}

	ret = intel_wait_for_register_fw(uncore, GU_DEBUG,
					 DRIVERFLR_STATUS, DRIVERFLR_STATUS,
					 15);
	if (ret) {
		drm_err(&i915->drm, "wait for Driver-FLR completion failed! %d\n", ret);
		return ret;
	}

	intel_uncore_write_fw(uncore, GU_DEBUG, DRIVERFLR_STATUS);

	return 0;
}

static void driver_flr_fini(struct drm_device *dev, void *gt)
{
	driver_flr((struct intel_gt *)gt);
}

static int driver_flr_init(struct intel_gt *gt)
{
	int ret;

	ret = driver_flr(gt);
	if (ret)
		return ret;

	return drmm_add_action(&gt->i915->drm, driver_flr_fini, gt);
}

void intel_gt_shutdown(struct intel_gt *gt)
{
	intel_iov_vf_put_wakeref_wa(&gt->iov);
}

static int intel_gt_tile_setup(struct intel_gt *gt,
			unsigned int id,
			phys_addr_t phys_addr)
{
	struct drm_i915_private *i915 = gt->i915;
	int ret;

	if (!gt_is_root(gt)) {
		struct intel_uncore *uncore;
		spinlock_t *irq_lock;

		uncore = drmm_kzalloc(&i915->drm, sizeof(*uncore), GFP_KERNEL);
		if (!uncore)
			return -ENOMEM;

		irq_lock = drmm_kzalloc(&gt->i915->drm, sizeof(*irq_lock), GFP_KERNEL);
		if (!irq_lock)
			return -ENOMEM;

		gt->uncore = uncore;
		gt->irq_lock = irq_lock;

		intel_gt_common_init_early(gt);
	}

	intel_uncore_init_early(gt->uncore, gt);

	ret = intel_uncore_setup_mmio(gt->uncore, phys_addr);
	if (ret)
		return ret;

	gt->phys_addr = phys_addr;

	ret = intel_iov_init_mmio(&gt->iov);
	if (unlikely(ret))
		return ret;

	intel_iov_init_early(&gt->iov);

	if (!id) {
		ret = driver_flr_init(gt);
		if (ret)
			return ret;
	}

	/* Which tile am I? default to zero on single tile systems */
	if (HAS_REMOTE_TILES(i915) && !IS_SRIOV_VF(i915)) {
		u32 instance =
			__raw_uncore_read32(gt->uncore, XEHPSDV_MTCFG_ADDR) &
			TILE_NUMBER;

		if (GEM_WARN_ON(instance != id))
			return -ENXIO;
	}

	return 0;
}

static unsigned int gt_count(struct drm_i915_private *i915)
{
	unsigned int num_gt;
	u32 mtcfg;

	/*
	 * VFs can't access XEHPSDV_MTCFG_ADDR register directly.
	 * But they only care about tiles where they were assigned.
	 */
	if (IS_SRIOV_VF(i915)) {
		u32 tile_mask = to_root_gt(i915)->iov.vf.config.tile_mask;

		/*
		 * On XE_LPM+ platforms media engines are designed into separate tile
		 */
		if (MEDIA_VER(i915) >= 13)
			return 2;

		if (!HAS_REMOTE_TILES(i915) || GEM_WARN_ON(!tile_mask))
			return 1;

		return fls(tile_mask);
	}

	/*
	 * We use raw MMIO reads at this point since the
	 * MMIO vfuncs are not setup yet
	 */
	mtcfg = __raw_uncore_read32(&i915->uncore, XEHPSDV_MTCFG_ADDR);
	num_gt = REG_FIELD_GET(TILE_COUNT, mtcfg) + 1;

	/*
	 * On XE_LPM+ platforms media engines are designed into separate tile
	 */
	if (MEDIA_VER(i915) >= 13)
		num_gt++;

	return num_gt;
}

int intel_count_l3_banks(struct drm_i915_private *i915,
			 struct intel_engine_cs *engine)
{
	struct intel_gt *gt = engine->gt;
	struct intel_uncore *uncore = gt->uncore;
	intel_wakeref_t wakeref;
	u32 count, store;

	/* L3 Banks not supported prior to version 12 */
	if (GRAPHICS_VER(i915) < 12)
		return -ENODEV;

	if (IS_PONTEVECCHIO(i915)) {
		with_intel_runtime_pm(uncore->rpm, wakeref)
			store = intel_uncore_read(uncore, GEN10_MIRROR_FUSE3);
		count = hweight32(REG_FIELD_GET(GEN12_MEML3_EN_MASK, store)) * 4 *
			hweight32(REG_FIELD_GET(XEHPC_GT_L3_MODE_MASK, store));
	} else if (GRAPHICS_VER_FULL(i915) > IP_VER(12, 50)) {
		count = hweight32(gt->info.mslice_mask) * 8;
	} else {
		count = hweight32(gt->info.l3bank_mask);
	}

	return count;
}

static unsigned int gt_mask(struct drm_i915_private *i915)
{
	unsigned long mask;

	if (!HAS_EXTRA_GT_LIST(i915))
		mask = BIT(0);
	else if (IS_SRIOV_VF(i915) && HAS_REMOTE_TILES(i915))
		mask = to_root_gt(i915)->iov.vf.config.tile_mask;
	else
		mask = GENMASK(gt_count(i915) - 1, 0);

	return mask;
}

int intel_gt_probe_all(struct drm_i915_private *i915)
{
	struct pci_dev *pdev = to_pci_dev(i915->drm.dev);
	struct intel_gt *gt = to_root_gt(i915);
	const struct intel_gt_definition *gtdef;
	phys_addr_t phys_addr;
	unsigned int mmio_bar;
	unsigned int i, num_gt, num_enabled_gt;
	unsigned long enabled_gt_mask;
	int ret;

	mmio_bar = GRAPHICS_VER(i915) == 2 ? GEN2_GTTMMADR_BAR : GTTMMADR_BAR;
	phys_addr = pci_resource_start(pdev, mmio_bar);

	/*
	 * We always have at least one primary GT on any device
	 * and it has been already initialized early during probe
	 * in i915_driver_probe()
	 */
	gt->i915 = i915;
	gt->name = "Primary GT";
	gt->info.engine_mask = INTEL_INFO(i915)->platform_engine_mask;

	gt_dbg(gt, "Setting up %s\n", gt->name);
	ret = intel_gt_tile_setup(gt, 0, phys_addr);
	if (ret)
		return ret;

	enabled_gt_mask = gt_mask(i915);
	if (enabled_gt_mask & BIT(0))
		i915->gt[0] = gt;

	num_gt = gt_count(i915);
	num_enabled_gt = hweight_long(enabled_gt_mask);
	drm_info(&i915->drm, "GT count: %u, enabled: %d\n", num_gt, num_enabled_gt);

	i = 1;
	for_each_set_bit_from(i, &enabled_gt_mask, I915_MAX_GT) {
		gtdef = &INTEL_INFO(i915)->extra_gt_list[i - 1];
		if (!gtdef->name)
			break;

		gt = drmm_kzalloc(&i915->drm, sizeof(*gt), GFP_KERNEL);
		if (!gt) {
			ret = -ENOMEM;
			goto err;
		}

		gt->i915 = i915;
		gt->name = gtdef->name;
		gt->type = gtdef->type;
		gt->info.engine_mask = gtdef->engine_mask;
		gt->info.id = i;

		gt_dbg(gt, "Setting up %s\n", gt->name);
		if (GEM_WARN_ON(range_overflows_t(resource_size_t,
						  gtdef->mapping_base,
						  SZ_16M,
						  pci_resource_len(pdev, mmio_bar)))) {
			ret = -ENODEV;
			goto err;
		}

		switch (gtdef->type) {
		case GT_TILE:
			ret = intel_gt_tile_setup(gt, i, phys_addr + gtdef->mapping_base);
			break;

		case GT_MEDIA:
			ret = intel_sa_mediagt_setup(gt, i, phys_addr + gtdef->mapping_base,
						     gtdef->gsi_offset);
			break;

		case GT_PRIMARY:
			/* Primary GT should not appear in extra GT list */
		default:
			MISSING_CASE(gtdef->type);
			ret = -ENODEV;
		}

		if (ret)
			goto err;

		i915->gt[i] = gt;
	}

	i915->remote_tiles = num_gt - 1;

	return 0;

err:
	i915_probe_error(i915, "Failed to initialize %s! (%d)\n", gtdef->name, ret);
	memset(i915->gt, 0, sizeof(i915->gt));
	return ret;
}

int intel_gt_tiles_init(struct drm_i915_private *i915)
{
	struct intel_gt *gt;
	unsigned int id;
	int ret;

	for_each_gt(gt, i915, id) {
		if (!i915->gt[id])
			break;

		if (GRAPHICS_VER(i915) >= 8)
			setup_private_pat(gt);

		ret = intel_gt_probe_lmem(gt);
		if (ret)
			return ret;
	}

	return 0;
}

void intel_gt_info_print(const struct intel_gt_info *info,
			 struct drm_printer *p)
{
	drm_printf(p, "GT %u info:\n", info->id);
	drm_printf(p, "available engines: %x\n", info->engine_mask);

	intel_sseu_dump(&info->sseu, p);
}
