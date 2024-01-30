// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright © 2021 Intel Corporation
 */

static int mi_store_dw__ggtt(void *arg)
{
	struct intel_gt *gt = arg;
	struct intel_engine_cs *engine;
	enum intel_engine_id id;
	int err = 0;

	for_each_engine(engine, gt, id) {
		u32 *store = memset(engine->status_page.addr + 1000, 0, 4);
		struct i915_request *rq;
		u32 *cs;

		if (!intel_engine_can_store_dword(engine))
			continue;

		rq = intel_context_create_request(engine->kernel_context);
		if (IS_ERR(rq))
			return PTR_ERR(rq);

		cs = intel_ring_begin(rq, 4);
		if (IS_ERR(cs)) {
			i915_request_add(rq);
			return PTR_ERR(cs);
		}

		*cs++ = MI_STORE_DWORD_IMM_GEN4 | MI_USE_GGTT;
		*cs++ = i915_vma_offset(engine->status_page.vma) + 4000;
		*cs++ = 0;
		*cs++ = 0xc0ffee;
		intel_ring_advance(rq, cs);

		i915_request_get(rq);
		i915_request_add(rq);
		if (i915_request_wait(rq, 0, HZ) < 0) {
			i915_request_put(rq);
			return -ETIME;
		}
		i915_request_put(rq);

		if (READ_ONCE(*store) != 0xc0ffee) {
			pr_err("%s: Invalid MI_STORE_DWORD, found %08x, expected %08x\n",
			       engine->name, READ_ONCE(*store), 0xc0ffee);
			err = -EINVAL;
		}
	}

	if (igt_flush_test(gt->i915))
		err = -EIO;

	return err;
}

static int
__mi_store_dw__ppgtt(struct intel_context *ce,
		     struct i915_vma *vma,
		     u32 *map,
		     u64 addr)
{
	struct i915_request *rq;
	u32 *cs;
	int err;

	err = i915_vma_pin(vma, 0, 0, PIN_USER | PIN_OFFSET_FIXED | addr);
	if (err) {
		if (err == -ENOSPC)
			return 0;

		pr_err("Failed to pin vma @ %llx\n", addr);
		return err;
	}
	GEM_BUG_ON(i915_vma_offset(vma) != addr);

	addr = sign_extend64(addr, fls64(vma->vm->total) - 1);
	memset32(map, STACK_MAGIC, SZ_4K / sizeof(u32));

	rq = intel_context_create_request(ce);
	if (IS_ERR(rq))
		return PTR_ERR(rq);

	err = __i915_vma_move_to_active(vma, rq);
	if (err) {
		i915_request_add(rq);
		return err;
	}

	cs = intel_ring_begin(rq, 4);
	if (IS_ERR(cs)) {
		i915_request_add(rq);
		return PTR_ERR(cs);
	}

	*cs++ = MI_STORE_DWORD_IMM_GEN4;
	*cs++ = lower_32_bits(addr);
	*cs++ = upper_32_bits(addr);
	*cs++ = 0xc0ffee;
	intel_ring_advance(rq, cs);

	i915_request_get(rq);
	i915_request_add(rq);
	if (i915_request_wait(rq, 0, HZ) < 0) {
		i915_request_put(rq);
		return -ETIME;
	}
	i915_request_put(rq);

	if (READ_ONCE(*map) != 0xc0ffee) {
		pr_err("%s: Invalid MI_STORE_DWORD(%llx), found %08x, expected %08x\n",
		       ce->engine->name, addr, READ_ONCE(*map), 0xc0ffee);
		return -EINVAL;
	}

	i915_vma_unpin(vma);
	return i915_vma_unbind(vma);
}

static int address_limit(struct intel_context *ce, bool bb_start)
{
	int limit = fls64(ce->vm->total);

	limit = min_t(int, ce->engine->ppgtt_size, limit);
	if (bb_start)
		limit = min(limit, 48);

	return limit;
}

static int mi_store_dw__ppgtt(void *arg)
{
	struct intel_gt *gt = arg;
	struct drm_i915_gem_object *obj;
	struct intel_engine_cs *engine;
	struct i915_ppgtt *ppgtt;
	enum intel_engine_id id;
	struct i915_vma *vma;
	u32 *map;
	int err;

	if (!HAS_FULL_PPGTT(gt->i915))
		return 0;

	ppgtt = i915_ppgtt_create(gt, 0);
	if (IS_ERR(ppgtt))
		return PTR_ERR(ppgtt);

	obj = i915_gem_object_create_internal(gt->i915, SZ_4K);
	if (IS_ERR(obj)) {
		err = PTR_ERR(obj);
		goto out_vm;
	}

	vma = i915_vma_instance(obj, &ppgtt->vm, NULL);
	if (IS_ERR(vma)) {
		err = PTR_ERR(vma);
		goto out_obj;
	}

	map = i915_gem_object_pin_map_unlocked(obj, I915_MAP_WC);
	if (IS_ERR(map)) {
		err = PTR_ERR(map);
		goto out_obj;
	}

	for_each_engine(engine, gt, id) {
		struct intel_context *ce;
		int bit, max;

		if (!intel_engine_can_store_dword(engine))
			continue;

		ce = intel_context_create(engine);
		if (IS_ERR(ce)) {
			err = PTR_ERR(ce);
			break;
		}

		i915_vm_put(ce->vm);
		ce->vm = i915_vm_get(&ppgtt->vm);

		max = address_limit(ce, false);
		for (bit = __ffs(vma->size); bit < max; bit++) {
			u64 addr = BIT_ULL(bit);

			if (addr + vma->size <= ce->vm->total) {
				err = __mi_store_dw__ppgtt(ce, vma, map, addr);
				if (err) {
					intel_context_put(ce);
					goto out;
				}
			}

			err = __mi_store_dw__ppgtt(ce, vma, map, addr - vma->size);
			if (err) {
				intel_context_put(ce);
				goto out;
			}
		}

		intel_context_put(ce);
	}

out:
	if (igt_flush_test(gt->i915))
		err = -EIO;
out_obj:
	i915_gem_object_put(obj);
out_vm:
	i915_vm_put(&ppgtt->vm);

	return err;
}

static int
__mi_bb_start__ppgtt(struct intel_context *ce,
		     struct i915_vma *vma,
		     u32 *map,
		     u64 addr)
{
	struct i915_request *rq;
	u32 result;
	int err;

	err = i915_vma_pin(vma, 0, 0, PIN_USER | PIN_OFFSET_FIXED | addr);
	if (err) {
		if (err == -ENOSPC)
			return 0;

		pr_err("Failed to pin vma @ %llx\n", addr);
		return err;
	}
	GEM_BUG_ON(i915_vma_offset(vma) != addr);

	addr = sign_extend64(addr, fls64(vma->vm->total) - 1);
	memset32(map, STACK_MAGIC, SZ_4K / sizeof(u32));

	map[0] = MI_STORE_DWORD_IMM_GEN4;
	map[1] = lower_32_bits(addr + 4000);
	map[2] = upper_32_bits(addr);
	map[3] = 0xc0ffee;
	map[4] = MI_BATCH_BUFFER_END;

	rq = intel_context_create_request(ce);
	if (IS_ERR(rq))
		return PTR_ERR(rq);

	err = __i915_vma_move_to_active(vma, rq);
	if (err == 0 && rq->engine->emit_init_breadcrumb)
		err = rq->engine->emit_init_breadcrumb(rq);
	if (err == 0)
		err = rq->engine->emit_bb_start(rq, addr, 4096, 0);
	if (err) {
		i915_request_add(rq);
		return err;
	}

	i915_request_get(rq);
	i915_request_add(rq);
	if (i915_request_wait(rq, 0, HZ) < 0) {
		i915_request_put(rq);
		return -EIO;
	}
	i915_request_put(rq);

	result = READ_ONCE(map[1000]);
	if (result != 0xc0ffee) {
		pr_err("%s: Invalid MI_BB_START(%llx) execution, found %08x, expected %08x\n",
		       ce->engine->name, addr, result, 0xc0ffee);
		return -EINVAL;
	}

	i915_vma_unpin(vma);
	return i915_vma_unbind(vma);
}

static int mi_bb_start__ppgtt(void *arg)
{
	struct intel_gt *gt = arg;
	struct drm_i915_gem_object *obj;
	struct intel_engine_cs *engine;
	struct i915_ppgtt *ppgtt;
	enum intel_engine_id id;
	struct i915_vma *vma;
	u32 *map;
	int err;

	if (!HAS_FULL_PPGTT(gt->i915))
		return 0;

	ppgtt = i915_ppgtt_create(gt, 0);
	if (IS_ERR(ppgtt))
		return PTR_ERR(ppgtt);

	obj = i915_gem_object_create_internal(gt->i915, SZ_4K);
	if (IS_ERR(obj)) {
		err = PTR_ERR(obj);
		goto out_vm;
	}

	vma = i915_vma_instance(obj, &ppgtt->vm, NULL);
	if (IS_ERR(vma)) {
		err = PTR_ERR(vma);
		goto out_obj;
	}

	map = i915_gem_object_pin_map_unlocked(obj, I915_MAP_WC);
	if (IS_ERR(map)) {
		err = PTR_ERR(map);
		goto out_obj;
	}

	for_each_engine(engine, gt, id) {
		struct intel_context *ce;
		int bit, max;

		if (!intel_engine_can_store_dword(engine))
			continue;

		ce = intel_context_create(engine);
		if (IS_ERR(ce)) {
			err = PTR_ERR(ce);
			break;
		}

		i915_vm_put(ce->vm);
		ce->vm = i915_vm_get(&ppgtt->vm);

		max = address_limit(ce, true);
		for (bit = __ffs(vma->size); bit < max; bit++) {
			u64 addr = BIT_ULL(bit);

			if (addr + vma->size <= ce->vm->total) {
				err = __mi_bb_start__ppgtt(ce, vma, map, addr);
				if (err) {
					intel_context_put(ce);
					goto out;
				}
			}

			err = __mi_bb_start__ppgtt(ce, vma, map, addr - vma->size);
			if (err) {
				intel_context_put(ce);
				goto out;
			}
		}

		intel_context_put(ce);
	}

out:
	if (igt_flush_test(gt->i915))
		err = -EIO;
out_obj:
	i915_gem_object_put(obj);
out_vm:
	i915_vm_put(&ppgtt->vm);

	return err;
}

int live_engine_mi_selftests(struct intel_gt *gt)
{
	static const struct i915_subtest tests[] = {
		SUBTEST(mi_store_dw__ggtt),
		SUBTEST(mi_store_dw__ppgtt),
		SUBTEST(mi_bb_start__ppgtt),
	};

	return intel_gt_live_subtests(tests, gt);
}
