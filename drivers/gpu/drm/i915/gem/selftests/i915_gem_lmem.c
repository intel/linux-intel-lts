//SPDX-License-Identifier: MIT
/*
 * Copyright © 2022 Intel Corporation
 */

#include <linux/pm_qos.h>

#include <drm/i915_pciids.h>

#include "gt/intel_gt_clock_utils.h"
#include "gt/intel_rps.h"

#include "i915_selftest.h"
#include "intel_pcode.h"
#include "selftests/i915_random.h"
#include "selftests/igt_flush_test.h"

#define CPU_LATENCY 0 /* -1 to disable pm_qos, 0 to disable cstates */

#define _MBs(x) (((x) * 1000000ull) >> 20)
#define MBs(x) (void *)_MBs(x)
static const struct pci_device_id clear_bandwidth[] = {
	INTEL_DG1_IDS(MBs(66000)),
	INTEL_DG2_G10_IDS(MBs(360000)),
	INTEL_DG2_G11_IDS(MBs(33000)),
	INTEL_DG2_G12_IDS(MBs(250000)),
	{},
};

static int igt_lmem_touch(void *arg)
{
	struct drm_i915_private *i915 = arg;
	struct intel_gt *gt;
	int id, ret = 0;

	for_each_gt(gt, i915, id) {
		u64 bits = 0;
		int err;

		err = i915_gem_lmemtest(gt, &bits);
		if (bits) {
			char prefix[80];

			snprintf(prefix, sizeof(prefix), "%s%d memory error: ",
				 gt->name, gt->info.id);
			print_hex_dump(KERN_ERR, prefix, DUMP_PREFIX_NONE,
					16, 1, &bits, sizeof(bits), false);
			err = -EINVAL;
		}
		if (err && !ret)
			ret = err;
	}

	return ret;
}

static int sync_blocks(struct list_head *blocks, long timeout)
{
	struct i915_buddy_block *block;

	list_for_each_entry(block, blocks, link) {
		struct dma_fence *f;

		f = i915_active_fence_get(&block->active);
		if (!f)
			continue;

		timeout = i915_request_wait(to_request(f), I915_WAIT_INTERRUPTIBLE, timeout);
		dma_fence_put(f);
		if (timeout < 0)
			return timeout;
	}

	return 0;
}

static int __igt_lmem_clear(struct drm_i915_private *i915, bool measure)
{
	const u64 poison = make_u64(0xc5c55c5c, 0xa3a33a3a);
	struct pm_qos_request qos;
	struct intel_gt *gt;
	I915_RND_STATE(prng);
	int err = 0;
	int id;

	if (CPU_LATENCY >= 0)
		cpu_latency_qos_add_request(&qos, CPU_LATENCY);

	for_each_gt(gt, i915, id) {
		struct intel_context *ce;
		unsigned int max_bw = 0;
		intel_wakeref_t wf;
		u64 size;

		if (!gt->lmem)
			continue;

		ce = get_blitter_context(gt, BCS0);
		if (!ce)
			continue;

		wf = intel_gt_pm_get(gt);
		intel_rps_boost(&gt->rps);

		for (size = gt->lmem->min_page_size; size <= min_t(u64, gt->lmem->total / 2, SZ_2G); size <<= 1) {
			struct i915_request *rq = NULL;
			ktime_t cpu, gpu, sync;
			LIST_HEAD(blocks);

			err = __intel_memory_region_get_pages_buddy(gt->lmem,
								    NULL,
								    size,
								    0,
								    &blocks);
			if (err) {
				pr_err("GT%d: failed to allocate %llx\n",
				       id, size);
				break;
			}

			err = sync_blocks(&blocks, HZ);
			if (err)
				break;

			cpu = -ktime_get();
			clear_cpu(gt->lmem, &blocks, poison);
			cpu += ktime_get();

			gpu = -READ_ONCE(gt->counters.map[INTEL_GT_CLEAR_ALLOC_CYCLES]);
			sync = -ktime_get();
			err = clear_blt(ce, NULL, &gt->lmem->mm, &blocks,
					INTEL_GT_CLEAR_ALLOC_CYCLES, true,
					&rq);
			if (rq) {
				i915_sw_fence_complete(&rq->submit);
				if (i915_request_wait(rq, I915_WAIT_INTERRUPTIBLE, HZ) < 0)
					err = -ETIME;
				else
					err = rq->fence.error;
				i915_request_put(rq);
			}
			sync += ktime_get();
			gpu += READ_ONCE(gt->counters.map[INTEL_GT_CLEAR_ALLOC_CYCLES]);

			gpu = intel_gt_clock_interval_to_ns(gt, gpu);
			if (gpu) {
				unsigned int cpu_bw, gpu_bw;

				cpu_bw = div_u64(mul_u64_u32_shr(size, NSEC_PER_SEC, 20), cpu);
				gpu_bw = div_u64(mul_u64_u32_shr(size, NSEC_PER_SEC, 20), gpu);

				dev_info(gt->i915->drm.dev,
					 "GT%d: checked with size:%llx, CPU write:%dMiB/s, GPU write:%dMiB/s, overhead:%lldns (%d%%), freq:%dMHz\n",
					 id, size, cpu_bw, gpu_bw, sync - gpu,
					 (int)div_u64((sync - gpu) * 100, sync),
					 intel_rps_read_actual_frequency(&gt->rps));

				max_bw = max(max_bw, gpu_bw);
			}

			if (err == 0) {
				struct i915_buddy_block *block;
				u64 sample, offset, sz;
				void * __iomem iova;

				block = list_first_entry(&blocks, typeof(*block), link);
				offset = i915_buddy_block_offset(block);
				sz = i915_buddy_block_size(&gt->lmem->mm, block);

				offset -= gt->lmem->region.start;
				iova = io_mapping_map_wc(&gt->lmem->iomap, offset, size);

				offset = igt_random_offset(&prng, 0, sz, sizeof(sample), 1);
				memcpy_fromio(&sample, iova + offset, sizeof(sample));
				io_mapping_unmap(iova);

				if (sample) {
					pr_err("GT%d: read @%llx of [%llx + %llx] and found %llx instead of zero!\n",
					       id, offset,
					       i915_buddy_block_offset(block),
					       sz, sample);
					err = -EINVAL;
				}
			}

			__intel_memory_region_put_pages_buddy(gt->lmem, &blocks, false);
			if (err)
				break;
		}

		intel_rps_cancel_boost(&gt->rps);
		intel_gt_pm_put(gt, wf);
		if (err)
			break;

		if (measure && max_bw) {
			struct pci_dev *pdev = to_pci_dev(gt->i915->drm.dev);
			unsigned int expected = 0;

			if (IS_PONTEVECCHIO(gt->i915)) {
				u32 val;

				if (snb_pcode_read_p(gt->uncore,
						     XEHPSDV_PCODE_FREQUENCY_CONFIG,
						     PCODE_MBOX_FC_SC_READ_FUSED_P0,
						     PCODE_MBOX_DOMAIN_HBM,
						     &val) == 0)
					expected = _MBs(2 * 128 * val * GT_FREQUENCY_MULTIPLIER);
			} else if (HAS_LMEM_MAX_BW(gt->i915)) {
				u32 val;

				if (snb_pcode_read_p(&i915->uncore, PCODE_MEMORY_CONFIG,
						     MEMORY_CONFIG_SUBCOMMAND_READ_MAX_BANDWIDTH,
						     0x0,
						     &val) == 0)
					expected = _MBs(val);
			} else {
				const struct pci_device_id *match;

				match = pci_match_id(clear_bandwidth, pdev);
				if (match)
					expected = (uintptr_t)match->driver_data;
			}

			if (7 * max_bw > expected * 8) {
				dev_warn(gt->i915->drm.dev,
					 "[0x%04x.%d] Peak bw measured:%d MiB/s, beyond expected %d MiB/s\n",
					 pdev->device, pdev->revision,
					 max_bw, expected);
			} else if (8 * max_bw < expected * 7) {
				dev_err(gt->i915->drm.dev,
					"[0x%04x.%d] Peak bw measured:%d MiB/s, expected at least 87.5%% of %d MiB/s [%d MiB/s]\n",
					pdev->device, pdev->revision,
					max_bw, expected,
					7 * expected >> 3);
				err = -ENXIO;
				break;
			}
		}
	}

	if (CPU_LATENCY >= 0)
		cpu_latency_qos_remove_request(&qos);

	if (igt_flush_test(i915))
		err = -EIO;
	if (err == -ETIME)
		err = 0;

	return err;
}

static int igt_lmem_clear(void *arg)
{
	return __igt_lmem_clear(arg, false);
}

static int igt_lmem_speed(void *arg)
{
	return __igt_lmem_clear(arg, true);
}

static int swap_blt_sync(struct intel_context *ce,
			 struct i915_buddy_mm *mm,
			 struct list_head *blocks,
			 struct drm_i915_gem_object *smem,
			 bool to_smem)
{
	struct i915_request *rq = NULL;
	int err;

	err = swap_blt(ce, smem /* dummy */,
		       mm, blocks,
		       smem, to_smem, MAX_SCHEDULE_TIMEOUT,
		       &rq);
	if (rq) {
		i915_sw_fence_complete(&rq->submit);
		if (i915_request_wait(rq, 0, HZ) < 0)
			err = -ETIME;
		else
			err = rq->fence.error;
		i915_request_put(rq);
	}

	return err;
}

static int igt_lmem_swap(void *arg)
{
	struct drm_i915_private *i915 = arg;
	struct pm_qos_request qos;
	struct intel_gt *gt;
	I915_RND_STATE(prng);
	int err = 0;
	int id;

	if (CPU_LATENCY >= 0)
		cpu_latency_qos_add_request(&qos, CPU_LATENCY);

	for_each_gt(gt, i915, id) {
		struct intel_context *ce;
		intel_wakeref_t wf;
		u64 size;

		if (!gt->lmem)
			continue;

		ce = get_blitter_context(gt, BCS0);
		if (!ce)
			continue;

		wf = intel_gt_pm_get(gt);
		intel_rps_boost(&gt->rps);

		for (size = SZ_4K; size <= min_t(u64, gt->lmem->total / 2, SZ_2G); size <<= 1) {
			struct drm_i915_gem_object *smem;
			struct i915_buddy_block *block;
			u64 sample, cookie, poison;
			ktime_t in, out, sync = 0;
			void * __iomem iova;
			LIST_HEAD(blocks);
			u64 offset, sz;

			smem = create_smem(gt, size);
			if (IS_ERR(smem))
				break;

			err = __intel_memory_region_get_pages_buddy(gt->lmem,
								    NULL,
								    size,
								    0,
								    &blocks);
			if (err) {
				pr_err("GT%d: failed to allocate %llx\n",
				       id, size);
				goto err;
			}

			err = sync_blocks(&blocks, HZ);
			if (err)
				goto err;

			cookie = i915_prandom_u64_state(&prng);
			poison = i915_prandom_u64_state(&prng);

			block = list_first_entry(&blocks, typeof(*block), link);
			offset = i915_buddy_block_offset(block);
			sz = i915_buddy_block_size(&gt->lmem->mm, block);

			offset -= gt->lmem->region.start;
			iova = io_mapping_map_wc(&gt->lmem->iomap, offset, sz);

			offset = igt_random_offset(&prng, 0, sz, sizeof(cookie), 1);
			memcpy(iova + offset, &cookie, sizeof(cookie));

			out = -READ_ONCE(gt->counters.map[INTEL_GT_SWAPOUT_CYCLES]);
			sync -= ktime_get();
			err = swap_blt_sync(ce, &gt->lmem->mm, &blocks, smem, true);
			sync += ktime_get();
			out += READ_ONCE(gt->counters.map[INTEL_GT_SWAPOUT_CYCLES]);
			if (err)
				goto err_buddy;

			memcpy(iova + offset, &poison, sizeof(poison));

			in = -READ_ONCE(gt->counters.map[INTEL_GT_SWAPIN_CYCLES]);
			sync -= ktime_get();
			err = swap_blt_sync(ce, &gt->lmem->mm, &blocks, smem, false);
			sync += ktime_get();
			in += READ_ONCE(gt->counters.map[INTEL_GT_SWAPIN_CYCLES]);
			if (err)
				goto err_buddy;

			out = intel_gt_clock_interval_to_ns(gt, out);
			in = intel_gt_clock_interval_to_ns(gt, in);
			if (out && in) {
				unsigned int out_bw, in_bw;

				out_bw = div_u64(mul_u64_u32_shr(2ull * size, NSEC_PER_SEC, 20), out);
				in_bw = div_u64(mul_u64_u32_shr(2ull * size, NSEC_PER_SEC, 20), in);

				dev_info(gt->i915->drm.dev,
					 "GT%d: checked with size:%llx, swap out:%dMiB/s, swap in:%dMiB/s, total:%lldns, overhead:%lldns (%d%%), freq:%dMHz\n",
					 id, size, out_bw, in_bw, sync, sync - out - in,
					 (int)div_u64((sync - out - in) * 100, sync),
					 intel_rps_read_actual_frequency(&gt->rps));
			}

			memcpy(&sample, iova + offset, sizeof(sample));
			if (sample != cookie) {
				pr_err("GT%d: failed to swap in&out offset %llx of [%llx + %llx], cookie:%llx, sample:%llx [poison:%llx]\n",
				       id, offset,
				       i915_buddy_block_offset(block), sz,
				       cookie, sample, poison);
				err = -EINVAL;
			}

err_buddy:
			__intel_memory_region_put_pages_buddy(gt->lmem, &blocks, true);
err:
			i915_gem_object_put(smem);
			if (err)
				break;
		}

		intel_rps_cancel_boost(&gt->rps);
		intel_gt_pm_put(gt, wf);
		if (err)
			break;
	}

	if (CPU_LATENCY >= 0)
		cpu_latency_qos_remove_request(&qos);

	return err;
}

int i915_gem_lmem_live_selftests(struct drm_i915_private *i915)
{
	static const struct i915_subtest tests[] = {
		SUBTEST(igt_lmem_touch),
		SUBTEST(igt_lmem_clear),
		SUBTEST(igt_lmem_swap),
	};

	return i915_live_subtests(tests, i915);
}

int i915_gem_lmem_wip_selftests(struct drm_i915_private *i915)
{
	static const struct i915_subtest tests[] = {
		SUBTEST(igt_lmem_speed),
	};

	return i915_live_subtests(tests, i915);
}
