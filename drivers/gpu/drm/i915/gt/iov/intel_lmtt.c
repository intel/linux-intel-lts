// SPDX-License-Identifier: MIT
/*
 * Copyright © 2020 Intel Corporation
 */

#include "intel_lmtt.h"
#include "intel_iov_utils.h"

#include "i915_drv.h"
#include "i915_scatterlist.h"
#include "gt/intel_gt_mcr.h"
#include "gt/intel_gt_regs.h"
#include "gem/i915_gem_lmem.h"
#include "gem/i915_gem_region.h"
#include "gt/intel_gt.h"

static struct intel_gt *lmtt_to_gt(struct intel_lmtt *lmtt)
{
	return container_of(lmtt, struct intel_gt, iov.pf.lmtt);
}

static struct intel_lmtt_pt *
lmtt_pt_alloc(struct intel_lmtt *lmtt, unsigned int level)
{
	resource_size_t pt_size = lmtt->ops->lmtt_pte_size(level) *
				  lmtt->ops->lmtt_pte_num(level);
	struct drm_i915_gem_object *obj;
	struct intel_lmtt_pt *pt;
	int err;

	pt = kzalloc(sizeof(*pt), GFP_KERNEL);
	if (!pt) {
		err = -ENOMEM;
		goto out;
	}

	if (level > 0) {
		pt->entry = kcalloc(lmtt->ops->lmtt_pte_num(level), sizeof(pt),
				    GFP_KERNEL);
		if (!pt->entry) {
			err = -ENOMEM;
			goto out_pt;
		}
	}

	obj = intel_gt_object_create_lmem(lmtt_to_gt(lmtt), pt_size,
					  I915_BO_ALLOC_CHUNK_64K |
					  I915_BO_ALLOC_CONTIGUOUS |
					  I915_BO_ALLOC_VOLATILE |
					  I915_BO_CPU_CLEAR);
	if (IS_ERR(obj)) {
		err = PTR_ERR(obj);
		goto out_entry;
	}

	err = i915_gem_object_pin_pages_unlocked(obj);
	if (unlikely(err))
		goto out_obj;

	pt->obj = obj;

	return pt;

out_obj:
	i915_gem_object_put(obj);
out_entry:
	kfree(pt->entry);
out_pt:
	kfree(pt);
out:
	return ERR_PTR(err);
}

static void
lmtt_pt_free(struct intel_lmtt_pt *pt)
{
	struct drm_i915_gem_object *obj;

	obj = fetch_and_zero(&pt->obj);
	i915_gem_object_unpin_pages(obj);
	i915_gem_object_put(obj);

	kfree(pt->entry);
	kfree(pt);
}

static int
__lmtt_alloc_range(struct intel_lmtt *lmtt, struct intel_lmtt_pt *pd,
		   unsigned int pd_level, u64 start, u64 end)
{
	unsigned int pt_level = pd_level - 1;
	u64 pte_addr_shift = BIT_ULL(lmtt->ops->lmtt_pte_shift(pd_level));
	int ret = 0;
	void *vaddr;
	u64 offset;

	GEM_BUG_ON(pd_level == 0);

	vaddr = i915_gem_object_pin_map_unlocked(pd->obj, I915_MAP_WC);
	if (IS_ERR(vaddr)) {
		ret = PTR_ERR(vaddr);
		goto out;
	}

	offset = start;
	while (offset < end) {
		struct intel_lmtt_pt *pt;
		u64 next, pde, pt_offset;
		unsigned int idx;

		pt = lmtt_pt_alloc(lmtt, pt_level);
		if (IS_ERR(pt)) {
			ret = PTR_ERR(pt);
			goto out_unpin;
		}
		pt_offset = i915_gem_object_lmem_offset(pt->obj);

		idx = lmtt->ops->lmtt_pte_idx(offset, pd_level);
		pde = lmtt->ops->lmtt_pte_encode(pt_offset,
						 LMTT_PTE_VALID,
						 pd_level);
		lmtt->ops->lmtt_pte_write(vaddr, pde, idx, pd_level);
		pd->entry[idx] = pt;

		next = min(end, round_up(offset + 1, pte_addr_shift));

		if (pt_level != 0) {
			ret = __lmtt_alloc_range(lmtt, pt, pt_level,
						 offset, next);
			if (ret)
				goto out_unpin;
		}

		offset = next;
	}

out_unpin:
	i915_gem_object_unpin_map(pd->obj);
out:
	return ret;
}

static int
lmtt_alloc_range(struct intel_lmtt *lmtt, unsigned int vf, u64 offset, u64 size)
{
	struct intel_lmtt_pt *pt, *pd = lmtt->pd;
	unsigned int root_pd_level = lmtt->ops->lmtt_root_pd_level();
	unsigned int pt_level = root_pd_level - 1;
	void *vaddr;
	u64 pde;
	int err;

	pt = lmtt_pt_alloc(lmtt, pt_level);
	if (IS_ERR(pt)) {
		err = PTR_ERR(pt);
		goto out;
	}

	vaddr = i915_gem_object_pin_map_unlocked(pd->obj, I915_MAP_WC);
	if (IS_ERR(vaddr)) {
		err = PTR_ERR(vaddr);
		goto out_free;
	}

	pde = lmtt->ops->lmtt_pte_encode(i915_gem_object_lmem_offset(pt->obj),
					 LMTT_PTE_VALID,
					 root_pd_level);
	lmtt->ops->lmtt_pte_write(vaddr, pde, vf, root_pd_level);
	pd->entry[vf] = pt;
	i915_gem_object_unpin_map(pd->obj);

	if (pt_level != 0) {
		err = __lmtt_alloc_range(lmtt, pt, pt_level,
					 offset, offset + size);
		if (err)
			goto out;
	}

	return 0;

out_free:
	lmtt_pt_free(pt);
out:
	return err;
}

static void
__lmtt_clear(struct intel_lmtt *lmtt, struct intel_lmtt_pt *pd,
	     unsigned int level)
{
	struct intel_lmtt_pt *pt;
	unsigned int i;

	if (level == 0)
		return;

	for (i = 0; i < lmtt->ops->lmtt_pte_num(level); i++) {
		pt = fetch_and_zero(&pd->entry[i]);
		if (!pt)
			continue;

		__lmtt_clear(lmtt, pt, level - 1);
		lmtt_pt_free(pt);
	}
}

static void
lmtt_clear(struct intel_lmtt *lmtt, unsigned int vf)
{
	struct intel_lmtt_pt *pt, *pd = lmtt->pd;
	unsigned int root_pd_level = lmtt->ops->lmtt_root_pd_level();
	unsigned int pt_level = root_pd_level - 1;
	void *vaddr;
	u64 pde = lmtt->ops->lmtt_pte_encode(0, 0, root_pd_level);

	vaddr = i915_gem_object_pin_map_unlocked(pd->obj, I915_MAP_WC);
	lmtt->ops->lmtt_pte_write(vaddr, pde, vf, root_pd_level);
	i915_gem_object_unpin_map(pd->obj);

	pt = fetch_and_zero(&pd->entry[vf]);
	if (!pt)
		return;

	__lmtt_clear(lmtt, pt, pt_level);
	lmtt_pt_free(pt);
}

/*
 * TODO: Pull offset from iov configuration if needed (currently it's RSVD)
 */
static u64 __lmtt_offset(struct intel_lmtt *lmtt, unsigned int vf)
{
	return 0;
}

static resource_size_t __lmtt_size(struct intel_lmtt *lmtt, unsigned int vf)
{
	struct intel_gt *_gt, *gt = lmtt_to_gt(lmtt);
	struct drm_i915_gem_object *obj;
	resource_size_t size = 0;
	unsigned int id;

	for_each_gt(_gt, gt->i915, id) {
		obj = _gt->iov.pf.provisioning.configs[vf].lmem_obj;
		if (!obj)
			continue;

		size += obj->base.size;
	}
	GEM_BUG_ON(!IS_ALIGNED(size, BIT_ULL(lmtt->ops->lmtt_pte_shift(0))));

	return size;
}

static int
__lmtt_insert_entries(struct intel_lmtt *lmtt, unsigned int vf,
		      u64 start, struct sg_table *sg)
{
	struct intel_lmtt_pt *pt, *_pt = NULL;
	struct sgt_iter iter;
	void *vaddr;
	u64 offset;

	GEM_BUG_ON(!IS_ALIGNED(start, BIT_ULL(lmtt->ops->lmtt_pte_shift(0))));

	pt = lmtt->ops->lmtt_leaf_pt(lmtt, start, vf);

	vaddr = i915_gem_object_pin_map_unlocked(pt->obj, I915_MAP_WC);
	if (IS_ERR(vaddr))
		return PTR_ERR(vaddr);

	__for_each_sgt_daddr(offset, iter, sg,
			     BIT_ULL(lmtt->ops->lmtt_pte_shift(0))) {
		_pt = lmtt->ops->lmtt_leaf_pt(lmtt, start, vf);
		if (pt != _pt) {
			i915_gem_object_unpin_map(pt->obj);
			pt = _pt;
			vaddr = i915_gem_object_pin_map_unlocked(pt->obj, I915_MAP_WC);
			if (IS_ERR(vaddr))
				return PTR_ERR(vaddr);
		}
		lmtt->ops->lmtt_pte_write(vaddr,
					  lmtt->ops->lmtt_pte_encode(offset,
								     LMTT_PTE_VALID, 0),
					  lmtt->ops->lmtt_pte_idx(start, 0),
					  0);
		start += BIT_ULL(lmtt->ops->lmtt_pte_shift(0));
	}
	i915_gem_object_unpin_map(pt->obj);

	return 0;
}

static int
lmtt_insert_entries(struct intel_lmtt *lmtt, unsigned int vf, u64 start)
{
	struct intel_gt *_gt, *gt = lmtt_to_gt(lmtt);
	unsigned int id;
	int err;

	/*
	 * Each tile has its own LMTT, and we need to make all objects (which
	 * are also per-tile) available.
	 */
	for_each_gt(_gt, gt->i915, id) {
		struct drm_i915_gem_object *obj = _gt->iov.pf.provisioning.configs[vf].lmem_obj;
		struct sg_table *sg;

		if (!obj)
			continue;

		sg = obj->mm.pages;

		err = __lmtt_insert_entries(lmtt, vf, start, sg);
		if (err)
			return err;

		start += obj->base.size;
	}
	GEM_BUG_ON(start != __lmtt_offset(lmtt, vf) + __lmtt_size(lmtt, vf));

	return 0;
}

static void gt_set_lmtt_dir_ptr(struct intel_gt *gt, unsigned long offset)
{
	u32 lmem_cfg;

	/* in multiples of 64KB */
	GEM_BUG_ON(!IS_ALIGNED(offset, SZ_64K));
	lmem_cfg = REG_FIELD_PREP(LMTT_DIR_PTR, offset / SZ_64K) | LMEM_ENABLE;

	intel_gt_mcr_multicast_write(gt, XEHP_LMEM_CFG_ADDR, lmem_cfg);
}

static int lmtt_pd_init(struct intel_lmtt *lmtt)
{
	struct intel_lmtt_pt *pd;

	GEM_BUG_ON(lmtt->ops->lmtt_root_pd_level() == 0);
	GEM_BUG_ON(lmtt->pd);

	pd = lmtt_pt_alloc(lmtt, lmtt->ops->lmtt_root_pd_level());
	if (IS_ERR(pd))
		return PTR_ERR(pd);
	lmtt->pd = pd;

	return 0;
}

static void lmtt_pd_fini(struct intel_lmtt *lmtt)
{
	struct intel_lmtt_pt *pd;

	/* We may have never initialized if we got wedged on init */
	pd = fetch_and_zero(&lmtt->pd);
	if (pd)
		lmtt_pt_free(pd);
}

void intel_lmtt_init_hw(struct intel_lmtt *lmtt)
{
	struct intel_gt *gt = lmtt_to_gt(lmtt);

	if (!HAS_LMEM(gt->i915))
		return;

	if (!lmtt->pd)
		return;

	gt_set_lmtt_dir_ptr(gt, i915_gem_object_lmem_offset(lmtt->pd->obj));
}

/**
 * intel_lmtt_init - Initalize LMTT allocations.
 * @lmtt: the LMTT struct
 *
 * This function allocates empty LMTT Page Directory and
 * registers it for use by GT hardware.
 * This function shall be called only on PF.
 *
 * Return: 0 on success or a negative error code on failure.
 */
int intel_lmtt_init(struct intel_lmtt *lmtt)
{
	struct intel_gt *gt = lmtt_to_gt(lmtt);
	int err;

	if (!HAS_LMEM(gt->i915))
		return 0;

	if (HAS_LMTT_LVL2(gt->i915))
		lmtt->ops = &pvc_lmtt_ops;
	else
		lmtt->ops = &xehpsdv_lmtt_ops;

	err = lmtt_pd_init(lmtt);
	if (unlikely(err))
		return err;

	return 0;
}

/**
 * intel_lmtt_fini - Cleanup LMTT allocations.
 * @lmtt: the LMTT struct
 *
 * This function shall be called only on PF.
 */
void intel_lmtt_fini(struct intel_lmtt *lmtt)
{
	struct intel_gt *gt = lmtt_to_gt(lmtt);

	if (!HAS_LMEM(gt->i915))
		return;

	lmtt_pd_fini(lmtt);
}

static int lmtt_create_entries(struct intel_lmtt *lmtt, unsigned int vf)
{
	struct intel_gt *gt = lmtt_to_gt(lmtt);
	struct drm_i915_private *i915 = gt->i915;
	struct intel_runtime_pm *rpm = &i915->runtime_pm;
	intel_wakeref_t wakeref;
	int err;

	wakeref = intel_runtime_pm_get(rpm);

	err = lmtt_alloc_range(lmtt, vf, __lmtt_offset(lmtt, vf), __lmtt_size(lmtt, vf));
	if (err)
		goto err;

	err = lmtt_insert_entries(lmtt, vf, __lmtt_offset(lmtt, vf));
	if (err)
		goto err;

	intel_runtime_pm_put(rpm, wakeref);
	return 0;

err:
	lmtt_clear(lmtt, vf);

	intel_runtime_pm_put(rpm, wakeref);
	return err;
}

static void lmtt_destroy_entries(struct intel_lmtt *lmtt, unsigned int vf)
{
	lmtt_clear(lmtt, vf);
}

/**
 * intel_lmtt_update_entries - Create LMTT entries.
 * @lmtt: the LMTT struct
 * @vf: VF id
 *
 * This function updates LMTT entries for a given VF.
 * This function shall be called only on PF.
 *
 * Return: 0 on success or a negative error code on failure.
 */
int intel_lmtt_update_entries(struct intel_lmtt *lmtt, unsigned int vf)
{
	int ret = 0;

	GEM_BUG_ON(!IS_SRIOV_PF(lmtt_to_gt(lmtt)->i915));
	GEM_BUG_ON(!HAS_LMEM(lmtt_to_gt(lmtt)->i915));

	if (!lmtt->pd)
		return 0;

	lmtt_destroy_entries(lmtt, vf);
	if (__lmtt_size(lmtt, vf) != 0)
		ret = lmtt_create_entries(lmtt, vf);

	return ret;
}

/**
 * intel_lmtt_estimate_pt_size - Estimate size of VF's LMTT PT allocation.
 * @lmtt: the LMTT struct
 * @size: planned VF's LMEM size
 *
 * This function shall be called only on PF.
 *
 * Return: size of the VF's PT allocation(s).
 */
resource_size_t intel_lmtt_estimate_pt_size(struct intel_lmtt *lmtt, u64 size)
{
	resource_size_t pt_size;
	unsigned int level = 0;

	GEM_BUG_ON(!IS_SRIOV_PF(lmtt_to_gt(lmtt)->i915));
	GEM_BUG_ON(!HAS_LMEM(lmtt_to_gt(lmtt)->i915));

	pt_size = lmtt->ops->lmtt_pte_size(level) *
		  lmtt->ops->lmtt_pte_num(level);

	while (++level < lmtt->ops->lmtt_root_pd_level()) {
		pt_size *= lmtt->ops->lmtt_pte_idx(size, level) + 1;
		pt_size += lmtt->ops->lmtt_pte_size(level) *
			   lmtt->ops->lmtt_pte_num(level);
	}

	return pt_size;
}
