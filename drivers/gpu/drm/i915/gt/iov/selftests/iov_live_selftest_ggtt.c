// SPDX-License-Identifier: MIT
/*
 * Copyright(c) 2021 Intel Corporation. All rights reserved.
 */

#include "i915_utils.h"
#include "gt/intel_gt.h"
#include "../abi/iov_actions_selftest_abi.h"
#include "../intel_iov_relay.h"

struct pte_testcase {
	bool (*test)(struct intel_iov *iov, void __iomem *pte_addr, u64 ggtt_addr, gen8_pte_t *out);
};

static gen8_pte_t gen8_get_pte(void __iomem *pte_addr)
{
	return readq(pte_addr);
}

static void gen8_set_pte(void __iomem *pte_addr, gen8_pte_t pte)
{
	writeq(pte, pte_addr);
}

static inline void __iomem *ggtt_addr_to_pte_addr(void __iomem *gsm, u64 ggtt_addr)
{
	GEM_BUG_ON(!IS_ALIGNED(ggtt_addr, I915_GTT_PAGE_SIZE_4K));

	return gsm + (ggtt_addr / I915_GTT_PAGE_SIZE_4K) * sizeof(gen8_pte_t);
}

static void gen8_set_masked_pte_val(void __iomem *pte_addr, const u64 mask_size,
				    const u8 mask_shift, u64 val)
{
	gen8_pte_t old_pte = gen8_get_pte(pte_addr) & ~(mask_size << mask_shift);
	gen8_pte_t pte = old_pte | (val << mask_shift);

	gen8_set_pte(pte_addr, pte);
}

static int vf_pte_get_value(struct intel_iov *iov, u64 ggtt_addr, u64 *pte)
{
	u32 request[VF2PF_ST_GET_GGTT_PTE_REQUEST_MSG_LEN];
	u32 response[VF2PF_ST_GET_GGTT_PTE_RESPONSE_MSG_LEN];
	u32 pte_lo, pte_hi;
	int err;

	GEM_BUG_ON(!intel_iov_is_vf(iov));

	request[0] = FIELD_PREP(GUC_HXG_MSG_0_ORIGIN, GUC_HXG_ORIGIN_HOST) |
		     FIELD_PREP(GUC_HXG_MSG_0_TYPE, GUC_HXG_TYPE_REQUEST) |
		     FIELD_PREP(GUC_HXG_REQUEST_MSG_0_ACTION, IOV_ACTION_VF2PF_PF_ST_ACTION) |
		     FIELD_PREP(VF2PF_PF_ST_ACTION_REQUEST_MSG_0_OPCODE,
				IOV_OPCODE_ST_GET_GGTT_PTE);
	request[1] = FIELD_PREP(VF2PF_ST_GET_GGTT_PTE_REQUEST_MSG_1_ADDRESS_LO,
				lower_32_bits(ggtt_addr));
	request[2] = FIELD_PREP(VF2PF_ST_GET_GGTT_PTE_REQUEST_MSG_2_ADDRESS_HI,
				upper_32_bits(ggtt_addr));

	err = intel_iov_relay_send_to_pf(&iov->relay,
					 request, ARRAY_SIZE(request),
					 response, ARRAY_SIZE(response));

	if (err < 0) {
		IOV_ERROR(iov, "ST: failed to get PTE value for %#llx, %d\n", ggtt_addr, err);
		return err;
	}

	pte_lo = FIELD_GET(VF2PF_ST_GET_GGTT_PTE_RESPONSE_MSG_1_PTE_LO, response[1]);
	pte_hi = FIELD_GET(VF2PF_ST_GET_GGTT_PTE_RESPONSE_MSG_2_PTE_HI, response[2]);

	*pte = make_u64(pte_hi, pte_lo);

	return err;
}

static bool
vf_pte_is_value_not_modifiable(struct intel_iov *iov, void __iomem *pte_addr, u64 ggtt_addr,
			       const u64 mask_size, const u8 mask_shift, gen8_pte_t *out)
{
	const u64 mask = mask_size << mask_shift;
	int err;
	u64 new_val;
	u64 val;

	err = vf_pte_get_value(iov, ggtt_addr, &val);
	if (err < 0)
		return false;

	val = (val & mask) >> mask_shift;

	new_val = val + 1;
	if (new_val > mask_size)
		new_val = 0;

	gen8_set_masked_pte_val(pte_addr, mask_size, mask_shift, new_val);

	err = vf_pte_get_value(iov, ggtt_addr, &val);
	if (err < 0)
		return false;

	val = (val & mask) >> mask_shift;

	*out = gen8_get_pte(pte_addr);

	return val != new_val;
}

static bool pte_not_accessible(struct intel_iov *iov, void __iomem *pte_addr, u64 ggtt_addr,
			       gen8_pte_t *out)
{
	*out = gen8_get_pte(pte_addr);

	return *out == 0;
}

static bool
pte_is_value_modifiable(struct intel_iov *iov, void __iomem *pte_addr, u64 ggtt_addr,
			const u64 mask, gen8_pte_t *out)
{
	gen8_pte_t original_pte;
	bool ret_val = true;
	gen8_pte_t read_pte;
	gen8_pte_t write_pte;

	original_pte = gen8_get_pte(pte_addr);

	write_pte = original_pte ^ mask;
	gen8_set_pte(pte_addr, write_pte);
	read_pte = gen8_get_pte(pte_addr);

	*out = read_pte;

	if ((read_pte & mask) != (write_pte & mask))
		ret_val = false;

	gen8_set_pte(pte_addr, original_pte);

	return ret_val;
}

static bool
pte_gpa_modifiable(struct intel_iov *iov, void __iomem *pte_addr, u64 ggtt_addr, gen8_pte_t *out)
{
	return pte_is_value_modifiable(iov, pte_addr, ggtt_addr, GEN12_GGTT_PTE_ADDR_MASK, out);
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

	original_pte = gen8_get_pte(pte_addr);

	write_pte = original_pte ^ FIELD_MAX(mask);
	gen8_set_pte(pte_addr, write_pte);
	read_pte = gen8_get_pte(pte_addr);

	*out = read_pte;

	if ((read_pte & mask) == (original_pte & mask))
		ret = true;

	gen8_set_pte(pte_addr, original_pte);

	return ret;
}

static bool
pte_vfid_modifiable(struct intel_iov *iov, void __iomem *pte_addr, u64 ggtt_addr, gen8_pte_t *out)
{
	return pte_is_value_modifiable(iov, pte_addr, ggtt_addr, TGL_GGTT_PTE_VFID_MASK, out);
}

static bool
pte_vfid_not_modifiable(struct intel_iov *iov, void __iomem *pte_addr, u64 ggtt_addr,
			gen8_pte_t *out)
{
	return !pte_vfid_modifiable(iov, pte_addr, ggtt_addr, out);
}

static bool
pte_vfid_not_readable(struct intel_iov *iov, void __iomem *pte_addr, u64 ggtt_addr, u64 *out)
{
	*out = gen8_get_pte(pte_addr);

	return u64_get_bits(*out, TGL_GGTT_PTE_VFID_MASK) == 0;
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
	     (pte_addr__) = (ggtt_addr_to_pte_addr((gsm__), (ggtt_addr__))); \
	     (ggtt_addr__) < ((ggtt_block__)->start + (ggtt_block__)->size); \
	     (ggtt_addr__) += (step__), \
	     (pte_addr__) = (ggtt_addr_to_pte_addr((gsm__), (ggtt_addr__))))

static bool
run_test_on_ggtt_block(struct intel_iov *iov, void __iomem *gsm, struct drm_mm_node *ggtt_block,
		       const struct pte_testcase *tc, u16 vfid, bool sanitycheck)
{
	int mul = 1;
	void __iomem *pte_addr;
	u64 ggtt_addr;

	GEM_BUG_ON(!IS_ALIGNED(ggtt_block->start, I915_GTT_PAGE_SIZE_4K));

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
		pte_addr = ggtt_addr_to_pte_addr(gsm, ggtt_addr);
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
		{ pte_gpa_modifiable },
		{ pte_vfid_modifiable },
		{ pte_valid_modifiable },
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

	IOV_DEBUG(iov, "Checking VF%u range [%#llx-%#llx]", vfid, ggtt_block.start,
		  ggtt_block.start + ggtt_block.size);

	for (vfid = 1; vfid <= pf_get_totalvfs(iov); vfid++) {
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

	GEM_BUG_ON(!IS_SRIOV_PF(i915));

	return igt_pf_iov_ggtt(&to_gt(i915)->iov);
}

static int igt_vf_iov_own_ggtt(struct intel_iov *iov, bool sanitycheck)
{
	gen8_pte_t __iomem *gsm = iov_to_gt(iov)->ggtt->gsm;
	static struct pte_testcase pte_testcases[] = {
		{ pte_gpa_modifiable },
		{ pte_vfid_not_readable },
		{ pte_vfid_not_modifiable },
		{ pte_valid_not_modifiable },
		{ },
	};
	int failed = 0;
	struct drm_mm_node ggtt_block;
	struct pte_testcase *tc;

	GEM_BUG_ON(!intel_iov_is_vf(iov));

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

	GEM_BUG_ON(!IS_SRIOV_VF(i915));

	return igt_vf_iov_own_ggtt(&to_gt(i915)->iov, false);
}

static int igt_vf_iov_own_ggtt_via_pf(struct intel_iov *iov)
{
	const u64 size_ggtt_block = SZ_64K;
	struct drm_mm_node ggtt_block = {};
	struct i915_ggtt *ggtt = iov_to_gt(iov)->ggtt;
	gen8_pte_t __iomem *gsm = ggtt->gsm;
	static struct pte_testcase pte_testcases[] = {
		{ pte_vfid_not_modifiable_check_via_pf },
		{ pte_valid_not_modifiable_check_via_pf },
		{ },
	};
	int failed = 0, err;
	struct pte_testcase *tc;

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
		IOV_DEBUG(iov, "Run '%ps' check\n", tc->test);
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

	GEM_BUG_ON(!IS_SRIOV_VF(i915));

	return igt_vf_iov_own_ggtt_via_pf(&to_gt(i915)->iov);
}

static int
_test_other_ggtt_region(struct intel_iov *iov, gen8_pte_t __iomem *gsm,
			struct drm_mm_node *ggtt_region)
{
	static struct pte_testcase pte_testcases[] = {
		{ pte_not_accessible },
		{ pte_gpa_not_modifiable },
		{ pte_vfid_not_modifiable },
		{ pte_valid_not_modifiable },
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
		{ pte_gpa_not_modifiable_check_via_pf },
		{ pte_vfid_not_modifiable_check_via_pf },
		{ pte_valid_not_modifiable_check_via_pf },
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
	struct pci_dev *pdev = gt->i915->drm.pdev;
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
	phys_addr =  pci_resource_start(pdev, 0) + gttaddr;

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

	GEM_BUG_ON(!IS_SRIOV_VF(i915));

	return igt_vf_iov_other_ggtt(&to_gt(i915)->iov, false);
}

static int igt_vf_other_ggtt_via_pf(void *arg)
{
	struct drm_i915_private *i915 = arg;

	GEM_BUG_ON(!IS_SRIOV_VF(i915));

	return igt_vf_iov_other_ggtt(&to_gt(i915)->iov, true);
}

static int
pf_handle_action_get_ggtt_pte(struct intel_iov *iov,
			      u32 origin, u32 relay_id,
			      const u32 *msg, u32 len)
{
	u32 response[VF2PF_ST_GET_GGTT_PTE_RESPONSE_MSG_LEN];
	u32 addr_lo, addr_hi;
	u64 ggtt_addr;
	void __iomem *pte_addr;
	gen8_pte_t pte;

	GEM_BUG_ON(!intel_iov_is_pf(iov));

	if (unlikely(len != VF2PF_ST_GET_GGTT_PTE_REQUEST_MSG_LEN))
		return -EPROTO;

	addr_lo = FIELD_GET(VF2PF_ST_GET_GGTT_PTE_REQUEST_MSG_1_ADDRESS_LO, msg[1]);
	addr_hi = FIELD_GET(VF2PF_ST_GET_GGTT_PTE_REQUEST_MSG_2_ADDRESS_HI, msg[2]);

	ggtt_addr = make_u64(addr_hi, addr_lo);

	if (!IS_ALIGNED(ggtt_addr, I915_GTT_PAGE_SIZE_4K))
		return -EINVAL;

	pte_addr = ggtt_addr_to_pte_addr(iov_to_gt(iov)->ggtt->gsm, ggtt_addr);

	pte = gen8_get_pte(pte_addr);

	response[0] = FIELD_PREP(GUC_HXG_MSG_0_ORIGIN, GUC_HXG_ORIGIN_HOST) |
		      FIELD_PREP(GUC_HXG_MSG_0_TYPE, GUC_HXG_TYPE_RESPONSE_SUCCESS) |
		      FIELD_PREP(VF2PF_PF_ST_ACTION_RESPONSE_MSG_0_MBZ, 0);
	response[1] = FIELD_PREP(VF2PF_ST_GET_GGTT_PTE_RESPONSE_MSG_1_PTE_LO, lower_32_bits(pte));
	response[2] = FIELD_PREP(VF2PF_ST_GET_GGTT_PTE_RESPONSE_MSG_2_PTE_HI, upper_32_bits(pte));

	return intel_iov_relay_reply_to_vf(&iov->relay, origin, relay_id, response,
					   ARRAY_SIZE(response));
}

int intel_iov_service_perform_selftest_action(struct intel_iov *iov, u32 origin, u32 relay_id,
					      const u32 *msg, u32 len)
{
	u32 opcode;

	GEM_BUG_ON(!intel_iov_is_pf(iov));

	if (unlikely(len < VF2PF_PF_ST_ACTION_REQUEST_MSG_MIN_LEN ||
		     len > VF2PF_PF_ST_ACTION_REQUEST_MSG_MAX_LEN))
		return -EPROTO;

	opcode = FIELD_GET(VF2PF_PF_ST_ACTION_REQUEST_MSG_0_OPCODE, msg[0]);

	switch (opcode) {
	case IOV_OPCODE_ST_GET_GGTT_PTE:
		return pf_handle_action_get_ggtt_pte(iov, origin, relay_id, msg, len);
	default:
		IOV_ERROR(iov, "Unsupported selftest opcode %#x from VF%u\n", opcode, origin);
		return -EBADRQC;
	}

	return intel_iov_relay_reply_ack_to_vf(&iov->relay, origin, relay_id, 0);
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
	int ret = 0;

	wakeref = intel_runtime_pm_get(&i915->runtime_pm);

	if (IS_SRIOV_PF(i915))
		ret = i915_subtests(pf_tests, i915);
	else if (IS_SRIOV_VF(i915))
		ret = i915_subtests(vf_tests, i915);

	intel_runtime_pm_put(&i915->runtime_pm, wakeref);

	return ret;
}
