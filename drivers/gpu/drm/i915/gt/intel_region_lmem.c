// SPDX-License-Identifier: MIT
/*
 * Copyright © 2019 Intel Corporation
 */

#include "i915_drv.h"
#include "i915_pci.h"
#include "i915_reg.h"
#include "i915_svm.h"
#include "intel_memory_region.h"
#include "intel_pci_config.h"
#include "intel_region_lmem.h"
#include "gem/i915_gem_lmem.h"
#include "gem/i915_gem_region.h"
#include "gt/intel_gt.h"
#include "gt/intel_gt_mcr.h"
#include "gt/intel_gt_regs.h"
#include "gt/iov/intel_iov_utils.h"

static void
region_lmem_release(struct intel_memory_region *mem)
{
	i915_svm_devmem_remove(mem);
	io_mapping_fini(&mem->iomap);
	intel_memory_region_release_buddy(mem);
}

static int
region_lmem_init(struct intel_memory_region *mem)
{
	u64 start, end;
	int ret;

	if (!io_mapping_init_wc(&mem->iomap,
				mem->io_start,
				mem->io_size))
		return -EIO;

	/*
	 * We directly map [1:1] offsets into the lmem region as addresses
	 * within the kernel vm. However, Wa_1409502670 introduces a quirk
	 * where xehpsdv-a0 cannot use the first page in each ppGTT and so in
	 * order to prevent the kernel from trying to submit illegal addresses
	 * when blitting to lmem, we have to exclude the first page from
	 * being allocated.
	 */
	start = mem->region.start;
	if (IS_XEHPSDV_GRAPHICS_STEP(mem->i915, STEP_A0, STEP_B0))
		start = max_t(u64, start, SZ_64K);
	end = mem->region.end + 1;

	ret = intel_memory_region_init_buddy(mem, start, end, PAGE_SIZE);
	if (ret)
		goto init_err;

	return 0;
init_err:
	io_mapping_fini(&mem->iomap);
	return ret;
}

static const struct intel_memory_region_ops intel_region_lmem_ops = {
	.init = region_lmem_init,
	.release = region_lmem_release,
	.init_object = __i915_gem_lmem_object_init,
};

/*
 * Don't allow LMEM allocation for first few megabytes reserved for
 * per tile debug trace data. Make sure to maintain alignment by using
 * buddy_alloc_range.
 */
static bool get_tracedebug_region(struct intel_uncore *uncore,
				  u64 *start, u32 *size)
{
	/* TODO: bspec says this is for XEHPSDV debug only */
	if (!IS_XEHPSDV(uncore->i915))
		return false;

	if (IS_SRIOV_VF(uncore->i915))
		return false;

	*size = intel_uncore_read(uncore, XEHP_DBGTRACEMEM_SZ);
	if (!*size)
		return false;

	if (WARN_ON(*size > 255))
		*size = 255;

	*size *= SZ_1M;
	*start = intel_uncore_read64_2x32(uncore,
					  XEHP_DBGTRACEMEMBASE_LDW,
					  XEHP_DBGTRACEMEMBASE_UDW);

	DRM_DEBUG_DRIVER("LMEM: debug trace data region: [0x%llx-0x%llx]\n",
			 *start, *start + *size);

	return true;
}

static bool get_legacy_lowmem_region(struct intel_uncore *uncore,
				     u64 *start, u32 *size)
{
	if (!IS_DG1(uncore->i915))
		return false;

	*start = 0;
	*size = SZ_1M;

	drm_dbg(&uncore->i915->drm, "LMEM: reserved legacy low-memory [0x%llx-0x%llx]\n",
		*start, *start + *size);

	return true;
}

static int reserve_lowmem_region(struct intel_uncore *uncore,
				 struct intel_memory_region *mem)
{
	u64 reserve_start;
	u64 reserve_size = 0;
	u64 region_start;
	u32 region_size;
	int ret;

	if (get_legacy_lowmem_region(uncore, &region_start, &region_size)) {
		reserve_start = region_start;
		reserve_size = region_size;
	}

	if (get_tracedebug_region(uncore, &region_start, &region_size)) {
		reserve_start = 0;
		reserve_size = region_size;
	}

	if (!reserve_size)
		return 0;

	ret = intel_memory_region_reserve(mem, reserve_start, reserve_size);
	if (ret)
		drm_err(&uncore->i915->drm, "LMEM: reserving low memory region failed\n");

	return ret;
}

static inline bool lmembar_is_igpu_stolen(struct drm_i915_private *i915)
{
	u32 regions = INTEL_INFO(i915)->memory_regions;

	if (regions & REGION_LMEM)
		return false;

	drm_WARN_ON(&i915->drm, (regions & REGION_STOLEN) == 0);
	return true;
}

int intel_get_tile_range(struct intel_gt *gt,
			 resource_size_t *lmem_base,
			 resource_size_t *lmem_size)
{
	struct drm_i915_private *i915 = gt->i915;
	struct pci_dev *pdev = to_pci_dev(i915->drm.dev);
	resource_size_t root_lmembar_size;
	resource_size_t lmem_range;
	static const i915_mcr_reg_t tile_addr_reg[] = {
		XEHP_TILE0_ADDR_RANGE,
		XEHP_TILE1_ADDR_RANGE,
		XEHP_TILE2_ADDR_RANGE,
		XEHP_TILE3_ADDR_RANGE,
	};
	u32 instance = gt->info.id;

	if (!i915_pci_resource_valid(pdev, GEN12_LMEM_BAR))
		return -ENXIO;

	root_lmembar_size = pci_resource_len(pdev, GEN12_LMEM_BAR);

	/*
	 * XEHPSDV A step single tile doesn't support the tile range
	 * registers.
	 */
	if (!lmembar_is_igpu_stolen(i915) && !IS_DG1(i915) &&
	    !(IS_XEHPSDV_GRAPHICS_STEP(i915, STEP_A0, STEP_B0) &&
	      !i915->remote_tiles)) {
		/* We should take the size and range of the tiles from
		 * the tile range register intead of assigning the offsets
		 * manually. The tile ranges are divided into 1GB granularity
		 */
		lmem_range = intel_gt_mcr_read_any(gt, tile_addr_reg[instance]) & 0xFFFF;
		*lmem_size = lmem_range >> XEHP_TILE_LMEM_RANGE_SHIFT;
		*lmem_base = (lmem_range & 0xFF) >> XEHP_TILE_LMEM_BASE_SHIFT;

		*lmem_size *= SZ_1G;
		*lmem_base *= SZ_1G;
	} else {
		*lmem_size = root_lmembar_size;
		*lmem_base = 0;
	}

	if (!*lmem_size || *lmem_base > root_lmembar_size)
		return -EIO;

	return 0;
}

static resource_size_t vf_get_lmem_size(struct intel_iov *iov)
{
	GEM_BUG_ON(!IS_SRIOV_VF(iov_to_i915(iov)));
	return iov->vf.config.lmem_size;
}

static resource_size_t vf_get_lmem_base(struct intel_iov *iov)
{
	struct drm_i915_private *i915 = iov_to_i915(iov);
	resource_size_t base = 0;
	struct intel_gt *gt;
	unsigned int id;

	GEM_BUG_ON(!IS_SRIOV_VF(iov_to_i915(iov)));

	for_each_gt(gt, i915, id)
		if (id < iov_to_gt(iov)->info.id)
			base += vf_get_lmem_size(&gt->iov);
		else
			break;

	return base;
}

static struct intel_memory_region *setup_lmem(struct intel_gt *gt)
{
	struct drm_i915_private *i915 = gt->i915;
	struct intel_uncore *uncore = gt->uncore;
	struct intel_mem_sparing_event *sparing;
	struct pci_dev *pdev = to_pci_dev(i915->drm.dev);
	struct intel_memory_region *mem;
	resource_size_t min_page_size;
	resource_size_t io_start;
	resource_size_t actual_mem;
	resource_size_t lmem_size, lmem_base;
	resource_size_t root_lmembar_size;
	bool is_degraded = false;
	int err;

	if (!IS_DGFX(i915))
		return ERR_PTR(-ENODEV);

	if (!i915_pci_resource_valid(pdev, GEN12_LMEM_BAR))
		return ERR_PTR(-ENXIO);

	root_lmembar_size = pci_resource_len(pdev, GEN12_LMEM_BAR);

	sparing = &to_gt(i915)->mem_sparing;
	/* VFs will get LMEM configuration from PF */
	if (IS_SRIOV_VF(i915)) {
		lmem_size = vf_get_lmem_size(&gt->iov);
		lmem_base = vf_get_lmem_base(&gt->iov);

		/* Track actual physical memory size from the PF */
		actual_mem = lmem_size;

		goto create_region;
	}

	/* Get per tile memory range */
	err = intel_get_tile_range(gt, &lmem_base, &lmem_size);
	if (err)
		return ERR_PTR(err);

	/* Track actual physical memory */
	actual_mem = lmem_size;

	/* Leave space for per-tile WOPCM/GSM stolen memory at the LMEM roof.
	 * Applicable only to XEHPSDV/DG2 etc.
	 */

	if (HAS_FLAT_CCS(i915)) {
		u64 tile_stolen, flat_ccs_base_addr_reg, flat_ccs_base;
		u64 actual_flat_ccs_size, expected_flat_ccs_size, bgsm;

		bgsm = intel_uncore_read64(uncore, GEN12_GSMBASE);

		if (GRAPHICS_VER_FULL(i915) >= IP_VER(12, 60)) {
			u32 pvc_flat_ccs_base_addr_low, pvc_flat_ccs_base_addr_high;
			u8 meml3_mask, hbm_count;

			meml3_mask = intel_uncore_read(uncore, GEN10_MIRROR_FUSE3) &
					GEN12_MEML3_EN_MASK;
			hbm_count = hweight8(meml3_mask);

			pvc_flat_ccs_base_addr_low = intel_gt_mcr_read_any(gt, PVC_FLAT_CCS_BASE_ADDR_LOWER);
			pvc_flat_ccs_base_addr_high = intel_gt_mcr_read_any(gt, PVC_FLAT_CCS_BASE_ADDR_UPPER);
			flat_ccs_base_addr_reg = pvc_flat_ccs_base_addr_high &
							PVC_FLAT_CCS_BASE_UPPER_ADDR_MASK;
			flat_ccs_base_addr_reg = flat_ccs_base_addr_reg << 32;
			flat_ccs_base_addr_reg |= pvc_flat_ccs_base_addr_low &
							PVC_FLAT_CCS_BASE_LOWER_ADDR_MASK;
			/*
			 * In PVC, flat ccs base address register holds the flat ccs
			 * base address per CC at tile level. Each HBM stack has 2 CC units.
			 * Convert this address relative to tile for SW use
			 */
			flat_ccs_base = flat_ccs_base_addr_reg * (2 * hbm_count);
		} else {
			flat_ccs_base_addr_reg = intel_gt_mcr_read_any(gt, XEHP_FLAT_CCS_BASE_ADDR);
			flat_ccs_base = (flat_ccs_base_addr_reg >> XEHP_CCS_BASE_SHIFT) * SZ_64K;
		}

		/* CCS to LMEM size ratio is 1:256 */
		expected_flat_ccs_size = lmem_size / 256;
		actual_flat_ccs_size = bgsm - flat_ccs_base;
		tile_stolen = lmem_size - (flat_ccs_base - lmem_base);

		/* If the FLAT_CCS_BASE_ADDR register is not populated, flag an error */
		if (tile_stolen == lmem_size)
			drm_err(&i915->drm,
				"CCS_BASE_ADDR register did not have expected value\n");
		/*
		 * If the actual flat ccs size is greater than the expected
		 * value, then there is memory degradation
		 */
		if (actual_flat_ccs_size > expected_flat_ccs_size &&
		    to_gt(i915)->info.id == 0) {
			drm_err(&i915->drm, "CCS_BASE_ADDR register did not have expected value - and memory degradation might have occurred\n");
			is_degraded = true;
		}

		lmem_size -= tile_stolen;
	} else {
		/* Stolen starts from GSMBASE without CCS */
		lmem_size = intel_uncore_read64(uncore, GEN12_GSMBASE) - lmem_base;

	}

create_region:
	/*
	 * We do want to continue with the driver load if the BAR size is smaller than
	 * memory fitted on the device. Fail on multi tile devices as BAR size might
	 * not be sufficient to map all the tiles.
	 */
	if (GEM_WARN_ON(lmem_size > root_lmembar_size || lmem_base > root_lmembar_size)) {
		if (i915->remote_tiles) {
			return ERR_PTR(-EIO);
		} else {
			drm_warn(&i915->drm, "Cannot use the full memory %pa on the device as LMEM BAR size was found to be smaller\n", &lmem_size);
			lmem_size = min(lmem_size, root_lmembar_size);
			drm_warn(&i915->drm, "Continuing with reduced LMEM size: %pa\n", &lmem_size);
		}
	}

	if (i915->params.lmem_size > 0) {
		lmem_size = min_t(resource_size_t, lmem_size,
				  mul_u32_u32(i915->params.lmem_size, SZ_1M));
	}

	if (GEM_WARN_ON(lmem_size > pci_resource_len(pdev, GEN12_LMEM_BAR)))
		return ERR_PTR(-ENODEV);

	io_start = pci_resource_start(pdev, GEN12_LMEM_BAR) + lmem_base;

	min_page_size = HAS_64K_PAGES(i915) ? I915_GTT_PAGE_SIZE_64K :
						I915_GTT_PAGE_SIZE_4K;

	/* Add the DPA (device physical address) offset */
	lmem_base += i915->intel_iaf.dpa;

	mem = intel_memory_region_create(gt,
					 lmem_base,
					 lmem_size,
					 min_page_size,
					 io_start,
					 lmem_size,
					 INTEL_MEMORY_LOCAL,
					 0,
					 &intel_region_lmem_ops);
	if (IS_ERR(mem))
		return mem;

	err = reserve_lowmem_region(uncore, mem);
	if (err)
		goto err_region_put;

	drm_dbg(&i915->drm, "Local memory: %pR\n", &mem->region);
	drm_dbg(&i915->drm, "Local memory IO start: %pa\n",
		&mem->io_start);
	drm_info(&i915->drm, "Local memory IO size: %pa\n",
		 &mem->io_size);
	drm_info(&i915->drm, "Local memory available: %pa\n",
		 &lmem_size);

	/* Report actual physical memory and health status */
	mem->actual_physical_mem = actual_mem;
	if (to_gt(i915)->info.id == 0) {
		if (is_degraded)
			sparing->health_status = MEM_HEALTH_DEGRADED;
		else
			sparing->health_status = MEM_HEALTH_OKAY;
	}

	return mem;

err_region_put:
	intel_memory_region_put(mem);
	return ERR_PTR(err);
}

struct intel_memory_region *intel_gt_setup_lmem(struct intel_gt *gt)
{
	return setup_lmem(gt);
}
