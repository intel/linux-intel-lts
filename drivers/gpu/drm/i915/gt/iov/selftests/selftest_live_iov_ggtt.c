// SPDX-License-Identifier: MIT
/*
 * Copyright(c) 2022 Intel Corporation. All rights reserved.
 */

#include "i915_utils.h"
#include "gt/intel_gt.h"
#include "intel_pci_config.h"
#include "iov_selftest_actions.h"
#include "../abi/iov_actions_selftest_abi.h"
#include "../intel_iov_relay.h"

struct pte_testcase {
	bool (*test)(struct intel_iov *iov, void __iomem *pte_addr, u64 ggtt_addr, gen8_pte_t *out);
	bool (*requires)(struct intel_iov *iov);
};

static void iov_set_pte(struct intel_iov *iov, void __iomem *addr, gen8_pte_t pte)
{
	gen8_set_pte(addr, pte);
}

static u64 iov_get_pte(struct intel_iov *iov, void __iomem *addr)
{
	return gen8_get_pte(addr);
}

static void mtl_iov_set_pte(struct intel_iov *iov, void __iomem *addr, gen8_pte_t pte)
{
	gen8_set_pte(addr, pte);

	/* XXX: hide MTL issue HSD 18028096950 and force flush by read some MMIO register */
	(void) intel_uncore_read(iov_to_gt(iov)->uncore, GEN12_VF_CAP_REG);
}

static void gen8_set_masked_pte_val(struct intel_iov *iov, void __iomem *pte_addr,
				    const u64 mask_size, const u8 mask_shift, u64 val)
{
	gen8_pte_t old_pte = iov->selftest.mmio_get_pte(iov, pte_addr) & ~(mask_size << mask_shift);
	gen8_pte_t pte = old_pte | (val << mask_shift);

	iov->selftest.mmio_set_pte(iov, pte_addr, pte);
}

static bool
vf_pte_is_value_not_modifiable(struct intel_iov *iov, void __iomem *pte_addr, u64 ggtt_addr,
			       const u64 mask_size, const u8 mask_shift, gen8_pte_t *out)
{
	const u64 mask = mask_size << mask_shift;
	int err;
	u64 new_val;
	u64 val;

	err = intel_iov_selftest_send_vfpf_get_ggtt_pte(iov, ggtt_addr, &val);
	if (err < 0)
		return false;

	val = (val & mask) >> mask_shift;

	new_val = val + 1;
	if (new_val > mask_size)
		new_val = 0;

	gen8_set_masked_pte_val(iov, pte_addr, mask_size, mask_shift, new_val);

	err = intel_iov_selftest_send_vfpf_get_ggtt_pte(iov, ggtt_addr, &val);
	if (err < 0)
		return false;

	val = (val & mask) >> mask_shift;

	*out = iov->selftest.mmio_get_pte(iov, pte_addr);

	return val != new_val;
}

static bool pte_not_accessible(struct intel_iov *iov, void __iomem *pte_addr, u64 ggtt_addr,
			       gen8_pte_t *out)
{
	struct drm_i915_private *i915 = iov_to_i915(iov);
	u64 expected;
	u64 mask;

	/* hsdes:18027049632 */
	if (GRAPHICS_VER_FULL(i915) < IP_VER(12, 10)) {
		expected = ~0;
		mask = ~0;
	} else if (GRAPHICS_VER_FULL(i915) < IP_VER(12, 70)) {
		expected = 0;
		mask = ~0;
	} else {
		expected = 0;
		mask = GEN12_GGTT_PTE_ADDR_MASK | XEHPSDV_GGTT_PTE_VFID_MASK;
	}

	*out = iov->selftest.mmio_get_pte(iov, pte_addr);
	return (*out & mask) == expected;
}

static bool
pte_is_value_modifiable(struct intel_iov *iov, void __iomem *pte_addr, u64 ggtt_addr,
			const u64 mask, gen8_pte_t *out)
{
	gen8_pte_t original_pte;
	bool ret_val = true;
	gen8_pte_t read_pte;
	gen8_pte_t write_pte;

	original_pte = iov->selftest.mmio_get_pte(iov, pte_addr);

	write_pte = original_pte ^ mask;
	iov->selftest.mmio_set_pte(iov, pte_addr, write_pte);
	read_pte = iov->selftest.mmio_get_pte(iov, pte_addr);

	*out = read_pte;

	if ((read_pte & mask) != (write_pte & mask))
		ret_val = false;

	iov->selftest.mmio_set_pte(iov, pte_addr, original_pte);

	return ret_val;
}

static bool
pte_gpa_modifiable(struct intel_iov *iov, void __iomem *pte_addr, u64 ggtt_addr, gen8_pte_t *out)
{
	if (IS_ALDERLAKE_P(iov_to_i915(iov)))
		return pte_is_value_modifiable(iov, pte_addr, ggtt_addr, ADL_GGTT_PTE_ADDR_MASK,
					       out);
	else
		return pte_is_value_modifiable(iov, pte_addr, ggtt_addr, GEN12_GGTT_PTE_ADDR_MASK,
					       out);
}

static bool
pte_gpa_not_modifiable(struct intel_iov *iov, void __iomem *pte_addr, u64 ggtt_addr,
		       gen8_pte_t *out)
{
	return !pte_gpa_modifiable(iov, pte_addr, ggtt_addr, out);
}

static bool
pte_valid_modifiable(struct intel_iov *iov, void __iomem *pte_addr, u64 ggtt_addr, gen8_pte_t *out)
{
	return pte_is_value_modifiable(iov, pte_addr, ggtt_addr, GEN6_PTE_VALID, out);
}

static bool
pte_valid_not_modifiable(struct intel_iov *iov, void __iomem *pte_addr, u64 ggtt_addr,
			 gen8_pte_t *out)
{
	const u64 mask = GEN6_PTE_VALID;
	bool ret = false;
	gen8_pte_t original_pte;
	gen8_pte_t read_pte;
	gen8_pte_t write_pte;

	original_pte = iov->selftest.mmio_get_pte(iov, pte_addr);

	write_pte = original_pte ^ FIELD_MAX(mask);
	iov->selftest.mmio_set_pte(iov, pte_addr, write_pte);
	read_pte = iov->selftest.mmio_get_pte(iov, pte_addr);

	*out = read_pte;

	if ((read_pte & mask) == (original_pte & mask))
		ret = true;

	iov->selftest.mmio_set_pte(iov, pte_addr, original_pte);

	return ret;
}

static bool
pte_lmem_modifiable(struct intel_iov *iov, void __iomem *pte_addr, u64 ggtt_addr, gen8_pte_t *out)
{
	return pte_is_value_modifiable(iov, pte_addr, ggtt_addr, GEN12_GGTT_PTE_LM, out);
}

static bool
pte_lmem_not_modifiable(struct intel_iov *iov, void __iomem *pte_addr, u64 ggtt_addr,
			gen8_pte_t *out)
{
	return !pte_lmem_modifiable(iov, pte_addr, ggtt_addr, out);
}

static bool
pte_pat_modifiable(struct intel_iov *iov, void __iomem *pte_addr, u64 ggtt_addr, gen8_pte_t *out)
{
	return pte_is_value_modifiable(iov, pte_addr, ggtt_addr, MTL_GGTT_PTE_PAT_MASK, out);
}

static bool
pte_pat_not_modifiable(struct intel_iov *iov, void __iomem *pte_addr, u64 ggtt_addr,
			gen8_pte_t *out)
{
	return !pte_pat_modifiable(iov, pte_addr, ggtt_addr, out);
}

static bool
pte_vfid_modifiable(struct intel_iov *iov, void __iomem *pte_addr, u64 ggtt_addr, gen8_pte_t *out)
{
	GEM_BUG_ON(!HAS_SRIOV(iov_to_i915(iov)));

	if (i915_ggtt_has_xehpsdv_pte_vfid_mask(iov_to_gt(iov)->ggtt))
		return pte_is_value_modifiable(iov, pte_addr, ggtt_addr, XEHPSDV_GGTT_PTE_VFID_MASK,
					       out);
	else
		return pte_is_value_modifiable(iov, pte_addr, ggtt_addr, TGL_GGTT_PTE_VFID_MASK,
					       out);
}

static bool
pte_vfid_not_modifiable(struct intel_iov *iov, void __iomem *pte_addr, u64 ggtt_addr,
			gen8_pte_t *out)
{
	return !pte_vfid_modifiable(iov, pte_addr, ggtt_addr, out);
}

static bool
__pte_value_is_not_readable(struct intel_iov *iov, void __iomem *pte_addr, const u64 mask,
			    gen8_pte_t *out)
{
	*out = iov->selftest.mmio_get_pte(iov, pte_addr);

	return u64_get_bits(*out, mask) == 0;
}

static bool
pte_vfid_not_readable(struct intel_iov *iov, void __iomem *pte_addr, u64 ggtt_addr, u64 *out)
{
	GEM_BUG_ON(!HAS_SRIOV(iov_to_i915(iov)));

	if (i915_ggtt_has_xehpsdv_pte_vfid_mask(iov_to_gt(iov)->ggtt))
		return __pte_value_is_not_readable(iov, pte_addr, XEHPSDV_GGTT_PTE_VFID_MASK, out);
	else
		return __pte_value_is_not_readable(iov, pte_addr, TGL_GGTT_PTE_VFID_MASK, out);
}

static bool
pte_gpa_not_modifiable_check_via_pf(struct intel_iov *iov, void __iomem *pte_addr, u64 ggtt_addr,
				    gen8_pte_t *out)
{
	const u64 mask = GEN12_GGTT_PTE_ADDR_MASK;

	return vf_pte_is_value_not_modifiable(iov, pte_addr, ggtt_addr, FIELD_MAX(mask),
					      __bf_shf(mask), out);
}

static bool
pte_vfid_not_modifiable_check_via_pf(struct intel_iov *iov, void __iomem *pte_addr, u64 ggtt_addr,
				     gen8_pte_t *out)
{
	const u64 mask = TGL_GGTT_PTE_VFID_MASK;

	return vf_pte_is_value_not_modifiable(iov, pte_addr, ggtt_addr, FIELD_MAX(mask),
					      __bf_shf(mask), out);
}

static bool
pte_valid_not_modifiable_check_via_pf(struct intel_iov *iov, void __iomem *pte_addr, u64 ggtt_addr,
				      gen8_pte_t *out)
{
	const u64 mask = GEN6_PTE_VALID;

	return vf_pte_is_value_not_modifiable(iov, pte_addr, ggtt_addr, FIELD_MAX(mask),
					      __bf_shf(mask), out);
}

static bool
pte_valid_readable_check_via_pf(struct intel_iov *iov, void __iomem *pte_addr, u64 ggtt_addr,
				gen8_pte_t *out)
{
	const u64 mask = GEN6_PTE_VALID;
	int err;
	gen8_pte_t pte;
	gen8_pte_t new_pte;
	u64 orig_val;

	GEM_BUG_ON(!IS_ALIGNED(ggtt_addr, I915_GTT_PAGE_SIZE_4K));

	err = intel_iov_selftest_send_vfpf_get_ggtt_pte(iov, ggtt_addr, &pte);
	if (err < 0)
		return false;

	orig_val = FIELD_GET(mask, pte);
	/*
	 * Make sure that bit valid is set.
	 */
	if (orig_val == 0) {
		pte |= FIELD_PREP(mask, 1);
		err = intel_iov_selftest_send_vfpf_set_ggtt_pte(iov, ggtt_addr, &pte);
		if (err < 0)
			return false;
	}

	new_pte = iov->selftest.mmio_get_pte(iov, pte_addr);

	/*
	 * Set the previous value if we modified it.
	 */
	err = intel_iov_selftest_send_vfpf_get_ggtt_pte(iov, ggtt_addr, &pte);
	if (err < 0)
		return false;

	if (orig_val == 0) {
		pte &= ~mask;
		err = intel_iov_selftest_send_vfpf_set_ggtt_pte(iov, ggtt_addr, &pte);
		if (err < 0)
			return false;
	}

	return FIELD_GET(mask, new_pte) == 1;
}

static bool wa_1808546409(struct intel_iov *iov)
{
	return !IS_TIGERLAKE(iov_to_i915(iov)) &&
	       !IS_XEHPSDV(iov_to_i915(iov));
}

static bool has_lmem(struct intel_iov *iov)
{
	return HAS_LMEM(iov_to_i915(iov));
}

static bool has_pat(struct intel_iov *iov)
{
	return GRAPHICS_VER_FULL(iov_to_i915(iov)) >= IP_VER(12, 70);
}

static bool uses_l4wa(struct intel_iov *iov)
{
	return i915_is_mem_wa_enabled(iov_to_i915(iov), I915_WA_USE_FLAT_PPGTT_UPDATE);
}

static bool run_test_on_pte(struct intel_iov *iov, void __iomem *pte_addr, u64 ggtt_addr,
			    const struct pte_testcase *tc, u16 vfid)
{
	gen8_pte_t read_val;

	if (!tc->test(iov, pte_addr, ggtt_addr, &read_val)) {
		IOV_ERROR(iov, "%ps.%u failed at GGTT address %#llx. PTE is: %#llx\n",
			  tc->test, vfid, ggtt_addr, read_val);
		return false;
	}

	return true;
}

#define for_each_pte(pte_addr__, ggtt_addr__, gsm__, ggtt_block__, step__) \
	for ((ggtt_addr__) = ((ggtt_block__)->start), \
	     (pte_addr__) = (gsm__) + (ggtt_addr_to_pte_offset((ggtt_addr__))); \
	     (ggtt_addr__) < ((ggtt_block__)->start + (ggtt_block__)->size); \
	     (ggtt_addr__) += (step__), \
	     (pte_addr__) = (gsm__) + (ggtt_addr_to_pte_offset((ggtt_addr__))))

static bool
run_test_on_ggtt_block(struct intel_iov *iov, void __iomem *gsm, struct drm_mm_node *ggtt_block,
		       const struct pte_testcase *tc, u16 vfid, bool sanitycheck)
{
	int mul = 1;
	void __iomem *pte_addr;
	u64 ggtt_addr;

	GEM_BUG_ON(!IS_ALIGNED(ggtt_block->start, I915_GTT_PAGE_SIZE_4K));

	if (tc->requires && !tc->requires(iov)) {
		IOV_DEBUG(iov, "Skip subtest %ps.%u\n", tc->test, vfid);
		return true;
	}

	for_each_pte(pte_addr, ggtt_addr, gsm, ggtt_block, I915_GTT_PAGE_SIZE_4K * mul) {
		if (!run_test_on_pte(iov, pte_addr, ggtt_addr, tc, vfid))
			return false;
		cond_resched();

		/*
		 * Sanity check is done during driver probe, so we want to do it quickly.
		 * Therefore, we'll check only some entries that are a multiple of 2.
		 */
		if (sanitycheck)
			mul *= 2;
	}

	/*
	 * During sanity check we want to check the last PTE in the range. To be sure,
	 * we will perform this test explicitly outside the main checking loop.
	 */
	if (sanitycheck) {
		ggtt_addr = ggtt_block->start + ggtt_block->size - I915_GTT_PAGE_SIZE_4K;
		pte_addr = gsm + ggtt_addr_to_pte_offset(ggtt_addr);
		if (!run_test_on_pte(iov, pte_addr, ggtt_addr, tc, vfid))
			return false;
	}

	return true;
}

#define for_each_pte_test(tc__, testcases__) \
	for ((tc__) = (testcases__); (tc__)->test; (tc__)++)

/*
 * We want to check state of GGTT entries of VF's.
 * PF has the right to modify the GGTT PTE in the whole range,
 * so any problem in writing an entry will be reported as an error
 */
static int igt_pf_iov_ggtt(struct intel_iov *iov)
{
	const u64 size_ggtt_block = SZ_2M;
	struct i915_ggtt *ggtt = iov_to_gt(iov)->ggtt;
	struct drm_mm_node ggtt_block = {};
	static struct pte_testcase pte_testcases[] = {
		{ .test = pte_pat_modifiable, .requires = has_pat },
		{ .test = pte_gpa_modifiable },
		{ .test = pte_vfid_modifiable },
		{ .test = pte_lmem_modifiable, .requires = has_lmem },
		{ .test = pte_valid_modifiable },
		{ },
	};
	int failed = 0;
	int err;
	u16 vfid;
	struct pte_testcase *tc;

	BUILD_BUG_ON(!IS_ALIGNED(size_ggtt_block, I915_GTT_PAGE_SIZE_4K));
	GEM_BUG_ON(!intel_iov_is_pf(iov));

	mutex_lock(&ggtt->vm.mutex);
	err = i915_gem_gtt_insert(&ggtt->vm, &ggtt_block, size_ggtt_block, 0,
				  I915_COLOR_UNEVICTABLE, 0, U64_MAX, PIN_HIGH);
	mutex_unlock(&ggtt->vm.mutex);

	if (err < 0)
		goto out;

	for (vfid = 1; vfid <= pf_get_totalvfs(iov); vfid++) {
		IOV_DEBUG(iov, "Checking VF%u range [%#llx-%#llx]", vfid, ggtt_block.start,
			  ggtt_block.start + ggtt_block.size);
		i915_ggtt_set_space_owner(ggtt, vfid, &ggtt_block);
		for_each_pte_test(tc, pte_testcases) {
			IOV_DEBUG(iov, "Run '%ps' check\n", tc->test);
			if (!run_test_on_ggtt_block(iov, ggtt->gsm, &ggtt_block, tc, vfid, false))
				failed++;
		}

		i915_ggtt_set_space_owner(ggtt, 0, &ggtt_block);
	}

	drm_mm_remove_node(&ggtt_block);

	if (failed)
		IOV_ERROR(iov, "%s: Count of failed test cases: %d", __func__, failed);

	return failed ? -EPERM : 0;
out:
	return err;
}

static int igt_pf_ggtt(void *arg)
{
	struct drm_i915_private *i915 = arg;
	struct intel_gt *gt;
	unsigned int id;
	int err;

	GEM_BUG_ON(!IS_SRIOV_PF(i915));

	for_each_gt(gt, i915, id) {
		err = igt_pf_iov_ggtt(&gt->iov);
		if (err < 0)
			return err;
	}

	return 0;
}

static int igt_vf_iov_own_ggtt(struct intel_iov *iov, bool sanitycheck)
{
	gen8_pte_t __iomem *gsm = iov_to_gt(iov)->ggtt->gsm;
	static struct pte_testcase pte_testcases[] = {
		{ .test = pte_pat_modifiable, .requires = has_pat },
		{ .test = pte_gpa_modifiable },
		{ .test = pte_vfid_not_readable },
		{ .test = pte_vfid_not_modifiable },
		{ .test = pte_lmem_modifiable, .requires = has_lmem },
		{ .test = pte_valid_not_modifiable },
		{ },
	};
	int failed = 0;
	struct drm_mm_node ggtt_block;
	struct pte_testcase *tc;

	GEM_BUG_ON(!intel_iov_is_vf(iov));

	if (uses_l4wa(iov)) {
		IOV_DEBUG(iov, "Skip test %s, L4WA is active.\n", __func__);
		return 0;
	}

	ggtt_block.start = iov->vf.config.ggtt_base;
	ggtt_block.size = iov->vf.config.ggtt_size;

	GEM_BUG_ON(!IS_ALIGNED(ggtt_block.start, I915_GTT_PAGE_SIZE_4K));
	GEM_BUG_ON(!IS_ALIGNED(ggtt_block.size, I915_GTT_PAGE_SIZE_4K));

	IOV_DEBUG(iov, "Subtest %s, gsm: %#llx base: %#llx size: %#llx\n",
		  __func__, ptr_to_u64(gsm), ggtt_block.start, ggtt_block.size);

	for_each_pte_test(tc, pte_testcases) {
		IOV_DEBUG(iov, "Run '%ps' check\n", tc->test);
		if (!run_test_on_ggtt_block(iov, gsm, &ggtt_block, tc, 0, sanitycheck))
			failed++;
	}

	if (failed)
		IOV_ERROR(iov, "%s: Count of failed test cases: %d", __func__, failed);

	return failed ? -EPERM : 0;
}

static int igt_vf_own_ggtt(void *arg)
{
	struct drm_i915_private *i915 = arg;
	struct intel_gt *gt;
	unsigned int id;
	int err;

	GEM_BUG_ON(!IS_SRIOV_VF(i915));

	for_each_gt(gt, i915, id) {
		err = igt_vf_iov_own_ggtt(&gt->iov, false);
		if (err < 0)
			return err;
	}

	return 0;
}

static int igt_vf_iov_own_ggtt_via_pf(struct intel_iov *iov)
{
	const u64 size_ggtt_block = SZ_64K;
	struct drm_mm_node ggtt_block = {};
	struct i915_ggtt *ggtt = iov_to_gt(iov)->ggtt;
	gen8_pte_t __iomem *gsm = ggtt->gsm;
	static struct pte_testcase pte_testcases[] = {
		{ .test = pte_vfid_not_modifiable_check_via_pf },
		{ .test = pte_valid_not_modifiable_check_via_pf },
		{ .test = pte_valid_readable_check_via_pf, .requires = wa_1808546409 },
		{ },
	};
	int failed = 0, err;
	struct pte_testcase *tc;

	if (uses_l4wa(iov)) {
		IOV_DEBUG(iov, "Skip test %s, L4WA is active.\n", __func__);
		return 0;
	}

	BUILD_BUG_ON(!IS_ALIGNED(size_ggtt_block, I915_GTT_PAGE_SIZE_4K));
	GEM_BUG_ON(!intel_iov_is_vf(iov));

	mutex_lock(&ggtt->vm.mutex);
	err = i915_gem_gtt_insert(&ggtt->vm, &ggtt_block, size_ggtt_block,
				  0, I915_COLOR_UNEVICTABLE, 0, U64_MAX,
				  PIN_HIGH);
	mutex_unlock(&ggtt->vm.mutex);
	if (err < 0)
		goto out;

	IOV_DEBUG(iov, "Subtest %s, gsm: %#llx base: %#llx size: %#llx\n",
		  __func__, ptr_to_u64(gsm), ggtt_block.start, ggtt_block.size);

	for_each_pte_test(tc, pte_testcases) {
		IOV_DEBUG(iov, "Run '%ps' check \n", tc->test);
		if (!run_test_on_ggtt_block(iov, gsm, &ggtt_block, tc, 0, false))
			failed++;
	}

	drm_mm_remove_node(&ggtt_block);

	if (failed)
		IOV_ERROR(iov, "%s: Count of failed test cases: %d", __func__, failed);

	return failed ? -EPERM : 0;
out:
	return err;
}

static int igt_vf_own_ggtt_via_pf(void *arg)
{
	struct drm_i915_private *i915 = arg;
	struct intel_gt *gt;
	unsigned int id;
	int err;

	GEM_BUG_ON(!IS_SRIOV_VF(i915));

	for_each_gt(gt, i915, id) {
		err = igt_vf_iov_own_ggtt_via_pf(&gt->iov);
		if (err < 0)
			return err;
	}

	return 0;
}

static int
_test_other_ggtt_region(struct intel_iov *iov, gen8_pte_t __iomem *gsm,
			struct drm_mm_node *ggtt_region)
{
	static struct pte_testcase pte_testcases[] = {
		{ .test = pte_pat_not_modifiable, .requires = has_pat },
		{ .test = pte_not_accessible },
		{ .test = pte_gpa_not_modifiable },
		{ .test = pte_vfid_not_modifiable },
		{ .test = pte_lmem_not_modifiable, .requires = has_lmem },
		{ .test = pte_valid_not_modifiable },
		{ },
	};
	int failed = 0;
	struct pte_testcase *tc;

	IOV_DEBUG(iov, "Subtest %s, gsm: %#llx base: %#llx size: %#llx\n",
		  __func__, ptr_to_u64(gsm), ggtt_region->start,
		  ggtt_region->size);

	for_each_pte_test(tc, pte_testcases) {
		IOV_DEBUG(iov, "Run '%ps' check\n", tc->test);
		if (!run_test_on_ggtt_block(iov, gsm, ggtt_region, tc, 0, false))
			failed++;
	}

	return failed ? -EPERM : 0;
}

static int
_test_other_ggtt_region_via_pf(struct intel_iov *iov, gen8_pte_t __iomem *gsm,
			       struct drm_mm_node *ggtt_region)
{
	static struct pte_testcase pte_testcases[] = {
		{ .test = pte_gpa_not_modifiable_check_via_pf },
		{ .test = pte_vfid_not_modifiable_check_via_pf },
		{ .test = pte_valid_not_modifiable_check_via_pf },
		{ },
	};
	int failed = 0;
	struct pte_testcase *tc;

	IOV_DEBUG(iov, "Subtest %s, gsm: %#llx base: %#llx size: %#llx\n",
		  __func__, ptr_to_u64(gsm), ggtt_region->start,
		  ggtt_region->size);

	for_each_pte_test(tc, pte_testcases) {
		IOV_DEBUG(iov, "Run '%ps' check\n", tc->test);
		if (!run_test_on_ggtt_block(iov, gsm, ggtt_region, tc, 0, false))
			failed++;
	}

	return failed ? -EPERM : 0;
}

static int
test_other_ggtt_region(struct intel_iov *iov, gen8_pte_t __iomem *gsm,
		       struct drm_mm_node *ggtt_region, bool check_via_pf)
{
	return check_via_pf ?
		_test_other_ggtt_region_via_pf(iov, gsm, ggtt_region) :
		_test_other_ggtt_region(iov, gsm, ggtt_region);
}

static void *map_gsm(struct intel_gt *gt, u64 ggtt_size)
{
	struct pci_dev *pdev = to_pci_dev(gt->i915->drm.dev);
	struct device *dev = gt->i915->drm.dev;
	u64 gsm_ggtt_size = (ggtt_size / I915_GTT_PAGE_SIZE_4K) *
			    sizeof(gen8_pte_t);
	phys_addr_t phys_addr;
	u32 gttaddr;
	void *gsm;

	/*
	 * Since GEN8 GTTADDR starts at 8MB offset
	 */
	gttaddr = SZ_8M;
	phys_addr =  pci_resource_start(pdev, GTTMMADR_BAR) + gttaddr;

	gsm = ioremap(phys_addr, gsm_ggtt_size);
	if (!gsm) {
		dev_err(dev, "Failed to map the GGTT page table\n");
		return  ERR_PTR(-ENOMEM);
	}

	return gsm;
}

static int igt_vf_iov_other_ggtt(struct intel_iov *iov, bool check_via_pf)
{
	u64 offset_vf = iov->vf.config.ggtt_base;
	u64 size_vf = iov->vf.config.ggtt_size;
	int failed = 0;
	gen8_pte_t __iomem *gsm;
	struct drm_mm_node test_region;

	if (uses_l4wa(iov)) {
		IOV_DEBUG(iov, "Skip test %s, L4WA is active.\n", __func__);
		return 0;
	}

	GEM_BUG_ON(!IS_ALIGNED(offset_vf, I915_GTT_PAGE_SIZE_4K));
	GEM_BUG_ON(!IS_ALIGNED(size_vf, I915_GTT_PAGE_SIZE_4K));

	/*
	 * We want to test GGTT block not assigned to current VF.
	 * There are two regions which we can test:
	 * - before current VF range,
	 * - after current VF range.
	 *
	 *       |<---------------- Total GGTT size -------------->|
	 *
	 *       +-------------------------------------------------+
	 *       | WOPCM |    available for PF and VFs   | GUC_TOP |
	 *       +-----------------+---------------+---------------+
	 *       |//// before /////|  current VF   |//// after ////|
	 *       +-----------------+---------------+---------------+
	 *
	 *       |<-- offset_vf -->|<-- size_vf -->|
	 *
	 * The current implementation of driver allows test at least
	 * one page of GGTT before and after VF's GGTT range.
	 *
	 *       +------------------+------------+-----------------+
	 *       | before GGTT page | current VF | after GGTT page |
	 *       +------------------+------------+-----------------+
	 *
	 *       |<--      4K    -->|            |<--     4K    -->|
	 *
	 * We will run two tests in which we will check these two areas before
	 * and after VF, called as "other regions".
	 * Before the tests, we must additionally map the GGTT in the size
	 * corresponding to the last GGTT address used in the test.
	 */
	gsm = map_gsm(iov_to_gt(iov), offset_vf + size_vf +
		      I915_GTT_PAGE_SIZE_4K);
	if (IS_ERR(gsm))
		return PTR_ERR(gsm);

	test_region.size = I915_GTT_PAGE_SIZE_4K;

	test_region.start = offset_vf - I915_GTT_PAGE_SIZE_4K;

	if (test_other_ggtt_region(iov, gsm, &test_region, check_via_pf) < 0)
		failed++;

	test_region.start = offset_vf + size_vf;
	if (test_other_ggtt_region(iov, gsm, &test_region, check_via_pf) < 0)
		failed++;

	iounmap(gsm);

	return failed ? -EPERM : 0;
}

static int igt_vf_other_ggtt(void *arg)
{
	struct drm_i915_private *i915 = arg;
	struct intel_gt *gt;
	unsigned int id;
	int err;

	GEM_BUG_ON(!IS_SRIOV_VF(i915));

	for_each_gt(gt, i915, id) {
		err = igt_vf_iov_other_ggtt(&gt->iov, false);
		if (err < 0)
			return err;
	}

	return 0;
}

static int igt_vf_other_ggtt_via_pf(void *arg)
{
	struct drm_i915_private *i915 = arg;
	struct intel_gt *gt;
	unsigned int id;
	int err;

	GEM_BUG_ON(!IS_SRIOV_VF(i915));

	for_each_gt(gt, i915, id) {
		err = igt_vf_iov_other_ggtt(&gt->iov, true);
		if (err < 0)
			return err;
	}

	return 0;
}

static void init_defaults_pte_io(struct intel_iov *iov)
{
	if (IS_METEORLAKE(iov_to_i915(iov)))
		iov->selftest.mmio_set_pte = mtl_iov_set_pte;
	else
		iov->selftest.mmio_set_pte = iov_set_pte;

	iov->selftest.mmio_get_pte = iov_get_pte;
}

int intel_iov_ggtt_live_selftests(struct drm_i915_private *i915)
{
	static const struct i915_subtest pf_tests[] = {
		SUBTEST(igt_pf_ggtt),
	};
	static const struct i915_subtest vf_tests[] = {
		SUBTEST(igt_vf_own_ggtt),
		SUBTEST(igt_vf_own_ggtt_via_pf),
		SUBTEST(igt_vf_other_ggtt),
		SUBTEST(igt_vf_other_ggtt_via_pf),
	};
	intel_wakeref_t wakeref;
	struct intel_gt *gt;
	unsigned int id;
	int ret = 0;

	for_each_gt(gt, i915, id)
		init_defaults_pte_io(&gt->iov);

	wakeref = intel_runtime_pm_get(&i915->runtime_pm);

	if (IS_SRIOV_PF(i915))
		ret = i915_subtests(pf_tests, i915);
	else if (IS_SRIOV_VF(i915))
		ret = i915_subtests(vf_tests, i915);

	intel_runtime_pm_put(&i915->runtime_pm, wakeref);

	return ret;
}
