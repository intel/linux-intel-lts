// SPDX-License-Identifier: MIT
/*
 * Copyright © 2018 Intel Corporation
 */

#include "gem/i915_gem_internal.h"
#include "gem/i915_gem_pm.h"
#include "gt/intel_engine_user.h"
#include "gt/intel_gt.h"
#include "i915_selftest.h"
#include "intel_reset.h"

#include "selftests/igt_flush_test.h"
#include "selftests/igt_reset.h"
#include "selftests/igt_spinner.h"
#include "selftests/intel_scheduler_helpers.h"
#include "selftests/mock_drm.h"

#include "gem/selftests/igt_gem_utils.h"
#include "gem/selftests/mock_context.h"

struct wa_lists {
	struct i915_wa_list gt_wa_list;
	struct {
		struct i915_wa_list wa_list;
		struct i915_wa_list ctx_wa_list;
	} engine[I915_NUM_ENGINES];
};

static u32 reg_offset(i915_reg_t reg)
{
	return i915_mmio_reg_offset(reg) & RING_FORCE_TO_NONPRIV_ADDRESS_MASK;
}

static u32 reg_access(i915_reg_t reg)
{
	return i915_mmio_reg_offset(reg) & RING_FORCE_TO_NONPRIV_ACCESS_MASK;
}

static bool deny_register(i915_reg_t reg)
{
	return i915_mmio_reg_offset(reg) & RING_FORCE_TO_NONPRIV_DENY;
}

static bool wo_register(i915_reg_t reg)
{
	return reg_access(reg) == RING_FORCE_TO_NONPRIV_ACCESS_WR;
}

static bool ro_register(i915_reg_t reg)
{
	return reg_access(reg) == RING_FORCE_TO_NONPRIV_ACCESS_RD;
}

static const char *repr_access(i915_reg_t reg)
{
	if (deny_register(reg))
		return "deny";

	if (ro_register(reg))
		return "ro";

	if (wo_register(reg))
		return "wo";

	return "allow";
}

static int request_add_sync(struct i915_request *rq, int err)
{
	i915_request_get(rq);
	i915_request_add(rq);
	if (i915_request_wait(rq, 0, HZ / 5) < 0)
		err = -EIO;
	i915_request_put(rq);

	return err;
}

static int request_add_spin(struct i915_request *rq, struct igt_spinner *spin)
{
	int err = 0;

	i915_request_get(rq);
	i915_request_add(rq);
	if (spin && !igt_wait_for_spinner(spin, rq))
		err = -ETIMEDOUT;
	i915_request_put(rq);

	return err;
}

static void
reference_lists_init(struct intel_gt *gt, struct wa_lists *lists)
{
	struct intel_engine_cs *engine;
	enum intel_engine_id id;

	memset(lists, 0, sizeof(*lists));

	wa_init(&lists->gt_wa_list, "GT_REF", "global");
	gt_init_workarounds(gt, &lists->gt_wa_list);

	for_each_engine(engine, gt, id) {
		struct i915_wa_list *wal = &lists->engine[id].wa_list;

		wa_init(wal, "REF", engine->name);
		engine_init_workarounds(engine, wal);

		__intel_engine_init_ctx_wa(engine,
					   &lists->engine[id].ctx_wa_list,
					   "CTX_REF");
	}
}

static void
reference_lists_fini(struct intel_gt *gt, struct wa_lists *lists)
{
	struct intel_engine_cs *engine;
	enum intel_engine_id id;

	for_each_engine(engine, gt, id)
		intel_wa_list_free(&lists->engine[id].wa_list);

	intel_wa_list_free(&lists->gt_wa_list);
}

static struct drm_i915_gem_object *
read_nonprivs(struct intel_context *ce)
{
	struct intel_engine_cs *engine = ce->engine;
	const u32 base = engine->mmio_base;
	struct drm_i915_gem_object *result;
	struct i915_request *rq;
	struct i915_vma *vma;
	u32 srm, *cs;
	int err;
	int i;

	result = i915_gem_object_create_internal(engine->i915, PAGE_SIZE);
	if (IS_ERR(result))
		return result;

	i915_gem_object_set_cache_coherency(result, I915_CACHE_LLC);

	cs = i915_gem_object_pin_map_unlocked(result, I915_MAP_WB);
	if (IS_ERR(cs)) {
		err = PTR_ERR(cs);
		goto err_obj;
	}
	memset(cs, 0xc5, PAGE_SIZE);
	i915_gem_object_flush_map(result);
	i915_gem_object_unpin_map(result);

	vma = i915_vma_instance(result, &engine->gt->ggtt->vm, NULL);
	if (IS_ERR(vma)) {
		err = PTR_ERR(vma);
		goto err_obj;
	}

	err = i915_vma_pin(vma, 0, 0, PIN_GLOBAL);
	if (err)
		goto err_obj;

	rq = intel_context_create_request(ce);
	if (IS_ERR(rq)) {
		err = PTR_ERR(rq);
		goto err_pin;
	}

	i915_vma_lock(vma);
	err = i915_request_await_object(rq, vma->obj, true);
	if (err == 0)
		err = i915_vma_move_to_active(vma, rq, EXEC_OBJECT_WRITE);
	i915_vma_unlock(vma);
	if (err)
		goto err_req;

	srm = MI_STORE_REGISTER_MEM | MI_SRM_LRM_GLOBAL_GTT;
	if (GRAPHICS_VER(engine->i915) >= 8)
		srm++;

	cs = intel_ring_begin(rq, 4 * RING_MAX_NONPRIV_SLOTS);
	if (IS_ERR(cs)) {
		err = PTR_ERR(cs);
		goto err_req;
	}

	for (i = 0; i < RING_MAX_NONPRIV_SLOTS; i++) {
		*cs++ = srm;
		*cs++ = i915_mmio_reg_offset(RING_FORCE_TO_NONPRIV(base, i));
		*cs++ = i915_ggtt_offset(vma) + sizeof(u32) * i;
		*cs++ = 0;
	}
	intel_ring_advance(rq, cs);

	i915_request_add(rq);
	i915_vma_unpin(vma);

	return result;

err_req:
	i915_request_add(rq);
err_pin:
	i915_vma_unpin(vma);
err_obj:
	i915_gem_object_put(result);
	return ERR_PTR(err);
}

static u32
get_whitelist_reg(const struct intel_engine_cs *engine, unsigned int i)
{
	i915_reg_t reg = i < engine->whitelist.count ?
			 engine->whitelist.list[i].reg :
			 RING_NOPID(engine->mmio_base);

	return i915_mmio_reg_offset(reg);
}

static void
print_results(const struct intel_engine_cs *engine, const u32 *results)
{
	unsigned int i;

	for (i = 0; i < RING_MAX_NONPRIV_SLOTS; i++) {
		u32 expected = get_whitelist_reg(engine, i);
		u32 actual = results[i];

		pr_info("RING_NONPRIV[%d]: expected 0x%08x, found 0x%08x\n",
			i, expected, actual);
	}
}

static int check_whitelist(struct intel_context *ce)
{
	struct intel_engine_cs *engine = ce->engine;
	struct drm_i915_gem_object *results;
	struct intel_wedge_me wedge;
	u32 *vaddr;
	int err;
	int i;

	results = read_nonprivs(ce);
	if (IS_ERR(results))
		return PTR_ERR(results);

	err = 0;
	i915_gem_object_lock(results, NULL);
	intel_wedge_on_timeout(&wedge, engine->gt, HZ / 5) /* safety net! */
		err = i915_gem_object_set_to_cpu_domain(results, false);

	if (intel_gt_is_wedged(engine->gt))
		err = -EIO;
	if (err)
		goto out_put;

	vaddr = i915_gem_object_pin_map(results, I915_MAP_WB);
	if (IS_ERR(vaddr)) {
		err = PTR_ERR(vaddr);
		goto out_put;
	}

	for (i = 0; i < RING_MAX_NONPRIV_SLOTS; i++) {
		u32 expected = get_whitelist_reg(engine, i);
		u32 actual = vaddr[i];

		if (expected != actual) {
			print_results(engine, vaddr);
			pr_err("Invalid RING_NONPRIV[%d], expected 0x%08x, found 0x%08x\n",
			       i, expected, actual);

			err = -EINVAL;
			break;
		}
	}

	i915_gem_object_unpin_map(results);
out_put:
	i915_gem_object_unlock(results);
	i915_gem_object_put(results);
	return err;
}

static int do_device_reset(struct intel_engine_cs *engine)
{
	intel_gt_reset(engine->gt, engine->mask, "live_workarounds");
	return 0;
}

static int do_engine_reset(struct intel_engine_cs *engine)
{
	return intel_engine_reset(engine, "live_workarounds");
}

static int do_guc_reset(struct intel_engine_cs *engine)
{
	/* Currently a no-op as the reset is handled by GuC */
	return 0;
}

static int
switch_to_scratch_context(struct intel_engine_cs *engine,
			  struct igt_spinner *spin,
			  struct i915_request **rq)
{
	struct intel_context *ce;
	int err = 0;

	ce = intel_context_create(engine);
	if (IS_ERR(ce))
		return PTR_ERR(ce);

	*rq = igt_spinner_create_request(spin, ce, MI_NOOP);
	intel_context_put(ce);

	if (IS_ERR(*rq)) {
		spin = NULL;
		err = PTR_ERR(*rq);
		goto err;
	}

	err = request_add_spin(*rq, spin);
err:
	if (err && spin)
		igt_spinner_end(spin);

	return err;
}

static int check_whitelist_across_reset(struct intel_engine_cs *engine,
					int (*reset)(struct intel_engine_cs *),
					const char *name)
{
	struct intel_context *ce, *tmp;
	struct igt_spinner spin;
	struct i915_request *rq;
	intel_wakeref_t wakeref;
	int err;

	pr_info("Checking %d whitelisted registers on %s (RING_NONPRIV) [%s]\n",
		engine->whitelist.count, engine->name, name);

	ce = intel_context_create(engine);
	if (IS_ERR(ce))
		return PTR_ERR(ce);

	err = igt_spinner_init(&spin, engine->gt);
	if (err)
		goto out_ctx;

	err = check_whitelist(ce);
	if (err) {
		pr_err("Invalid whitelist *before* %s reset!\n", name);
		goto out_spin;
	}

	err = switch_to_scratch_context(engine, &spin, &rq);
	if (err)
		goto out_spin;

	/* Ensure the spinner hasn't aborted */
	if (i915_request_completed(rq)) {
		pr_err("%s spinner failed to start\n", name);
		err = -ETIMEDOUT;
		goto out_spin;
	}

	with_intel_runtime_pm(engine->uncore->rpm, wakeref)
		err = reset(engine);

	/* Ensure the reset happens and kills the engine */
	if (err == 0)
		err = intel_selftest_wait_for_rq(rq);

	igt_spinner_end(&spin);

	if (err) {
		pr_err("%s reset failed\n", name);
		goto out_spin;
	}

	err = check_whitelist(ce);
	if (err) {
		pr_err("Whitelist not preserved in context across %s reset!\n",
		       name);
		goto out_spin;
	}

	tmp = intel_context_create(engine);
	if (IS_ERR(tmp)) {
		err = PTR_ERR(tmp);
		goto out_spin;
	}
	intel_context_put(ce);
	ce = tmp;

	err = check_whitelist(ce);
	if (err) {
		pr_err("Invalid whitelist *after* %s reset in fresh context!\n",
		       name);
		goto out_spin;
	}

out_spin:
	igt_spinner_fini(&spin);
out_ctx:
	intel_context_put(ce);
	return err;
}

static struct i915_vma *create_batch(struct i915_address_space *vm)
{
	struct drm_i915_gem_object *obj;
	struct i915_vma *vma;
	int err;

	obj = i915_gem_object_create_internal(vm->i915, 16 * PAGE_SIZE);
	if (IS_ERR(obj))
		return ERR_CAST(obj);

	vma = i915_vma_instance(obj, vm, NULL);
	if (IS_ERR(vma)) {
		err = PTR_ERR(vma);
		goto err_obj;
	}

	err = i915_vma_pin(vma, 0, 0, PIN_USER | PIN_ZONE_48);
	if (err)
		goto err_obj;

	return vma;

err_obj:
	i915_gem_object_put(obj);
	return ERR_PTR(err);
}

static u32 reg_write(u32 old, u32 new, u32 rsvd)
{
	if (rsvd == 0x0000ffff) {
		old &= ~(new >> 16);
		old |= new & (new >> 16);
	} else {
		old &= ~rsvd;
		old |= new & rsvd;
	}

	return old;
}

static bool timestamp(const struct intel_engine_cs *engine, i915_reg_t reg)
{
	switch (reg_offset(reg) - engine->mmio_base) {
	case 0x358:
	case 0x35c:
	case 0x3a8:
		return true;

	default:
		return false;
	}
}

static int check_dirty_whitelist(struct intel_context *ce)
{
	const u32 values[] = {
		0x00000000,
		0x01010101,
		0x10100101,
		0x03030303,
		0x30300303,
		0x05050505,
		0x50500505,
		0x0f0f0f0f,
		0xf00ff00f,
		0x10101010,
		0xf0f01010,
		0x30303030,
		0xa0a03030,
		0x50505050,
		0xc0c05050,
		0xf0f0f0f0,
		0x11111111,
		0x33333333,
		0x55555555,
		0x0000ffff,
		0x00ff00ff,
		0xff0000ff,
		0xffff00ff,
		0xffffffff,
	};
	struct intel_engine_cs *engine = ce->engine;
	struct i915_vma *scratch;
	struct i915_vma *batch;
	int err = 0, i, v, sz;
	u32 *cs, *results;

	sz = (2 * ARRAY_SIZE(values) + 1) * sizeof(u32);
	scratch = __vm_create_scratch_for_read_pinned(ce->vm, sz);
	if (IS_ERR(scratch))
		return PTR_ERR(scratch);

	batch = create_batch(ce->vm);
	if (IS_ERR(batch)) {
		err = PTR_ERR(batch);
		goto out_scratch;
	}

	for (i = 0; i < engine->whitelist.count; i++) {
		bool ro_reg = (ro_register(engine->whitelist.list[i].reg) ||
			       deny_register(engine->whitelist.list[i].reg));
		bool varying_reg =
			timestamp(engine, engine->whitelist.list[i].reg);
		u32 reg = reg_offset(engine->whitelist.list[i].reg);
		u64 addr = i915_vma_offset(scratch);
		struct i915_gem_ww_ctx ww;
		struct i915_request *rq;
		u32 srm, lrm, rsvd;
		u32 expect;
		int idx;

		i915_gem_ww_ctx_init(&ww, false);
retry:
		cs = NULL;
		err = i915_gem_object_lock(scratch->obj, &ww);
		if (!err)
			err = i915_gem_object_lock(batch->obj, &ww);
		if (!err)
			err = intel_context_pin_ww(ce, &ww);
		if (err)
			goto out;

		cs = i915_gem_object_pin_map(batch->obj, I915_MAP_WC);
		if (IS_ERR(cs)) {
			err = PTR_ERR(cs);
			goto out_ctx;
		}

		results = i915_gem_object_pin_map(scratch->obj, I915_MAP_WB);
		if (IS_ERR(results)) {
			err = PTR_ERR(results);
			goto out_unmap_batch;
		}

		srm = MI_STORE_REGISTER_MEM;
		lrm = MI_LOAD_REGISTER_MEM;
		if (GRAPHICS_VER(engine->i915) >= 8)
			lrm++, srm++;

		pr_debug("%s: Writing garbage to %x[%s\n",
			 engine->name, reg, repr_access(engine->whitelist.list[i].reg));

		/* SRM original */
		*cs++ = srm;
		*cs++ = reg;
		*cs++ = lower_32_bits(addr);
		*cs++ = upper_32_bits(addr);

		idx = 1;
		for (v = 0; v < ARRAY_SIZE(values); v++) {
			/* LRI garbage */
			*cs++ = MI_LOAD_REGISTER_IMM(1);
			*cs++ = reg;
			*cs++ = values[v];

			/* SRM result */
			*cs++ = srm;
			*cs++ = reg;
			*cs++ = lower_32_bits(addr + sizeof(u32) * idx);
			*cs++ = upper_32_bits(addr + sizeof(u32) * idx);
			idx++;
		}
		for (v = 0; v < ARRAY_SIZE(values); v++) {
			/* LRI garbage */
			*cs++ = MI_LOAD_REGISTER_IMM(1);
			*cs++ = reg;
			*cs++ = ~values[v];

			/* SRM result */
			*cs++ = srm;
			*cs++ = reg;
			*cs++ = lower_32_bits(addr + sizeof(u32) * idx);
			*cs++ = upper_32_bits(addr + sizeof(u32) * idx);
			idx++;
		}
		GEM_BUG_ON(idx * sizeof(u32) > scratch->size);

		/* LRM original -- don't leave garbage in the context! */
		*cs++ = lrm;
		*cs++ = reg;
		*cs++ = lower_32_bits(addr);
		*cs++ = upper_32_bits(addr);

		*cs++ = MI_BATCH_BUFFER_END;

		i915_gem_object_flush_map(batch->obj);
		i915_gem_object_unpin_map(batch->obj);
		intel_gt_chipset_flush(engine->gt);
		cs = NULL;

		rq = i915_request_create(ce);
		if (IS_ERR(rq)) {
			err = PTR_ERR(rq);
			goto out_unmap_scratch;
		}

		if (engine->emit_init_breadcrumb) { /* Be nice if we hang */
			err = engine->emit_init_breadcrumb(rq);
			if (err)
				goto err_request;
		}

		err = i915_request_await_object(rq, batch->obj, false);
		if (err == 0)
			err = i915_vma_move_to_active(batch, rq, 0);
		if (err)
			goto err_request;

		err = i915_request_await_object(rq, scratch->obj, true);
		if (err == 0)
			err = i915_vma_move_to_active(scratch, rq,
						      EXEC_OBJECT_WRITE);
		if (err)
			goto err_request;

		err = engine->emit_bb_start(rq,
					    i915_vma_offset(batch),
					    PAGE_SIZE,
					    0);
		if (err)
			goto err_request;

err_request:
		err = request_add_sync(rq, err);
		if (err) {
			pr_err("%s: Futzing %x timedout; cancelling test\n",
			       engine->name, reg);
			intel_gt_set_wedged(engine->gt);
			goto out_unmap_scratch;
		}

		GEM_BUG_ON(values[ARRAY_SIZE(values) - 1] != 0xffffffff);
		if (!ro_reg) {
			/* detect write masking */
			rsvd = results[ARRAY_SIZE(values)];
			if (!rsvd) {
				pr_err("%s: Unable to write to whitelisted register %x\n",
				       engine->name, reg);
				err = -EINVAL;
				goto out_unmap_scratch;
			}
		} else {
			rsvd = 0;
		}

		expect = results[0];
		idx = 1;
		for (v = 0; v < ARRAY_SIZE(values); v++) {
			if (ro_reg)
				expect = results[0];
			else
				expect = reg_write(expect, values[v], rsvd);

			if (results[idx] != expect)
				err++;
			idx++;
		}
		for (v = 0; v < ARRAY_SIZE(values); v++) {
			if (ro_reg)
				expect = results[0];
			else
				expect = reg_write(expect, ~values[v], rsvd);

			if (results[idx] != expect)
				err++;
			idx++;
		}
		if (varying_reg)
			err = 0;
		if (err) {
			pr_err("%s: %d mismatch between values written to whitelisted register [%x:%s], and values read back!\n",
			       engine->name, err,
			       reg, repr_access(engine->whitelist.list[i].reg));

			pr_info("%s: Whitelisted register: %x, original value %08x, rsvd %08x\n",
				engine->name, reg, results[0], rsvd);

			expect = results[0];
			idx = 1;
			for (v = 0; v < ARRAY_SIZE(values); v++) {
				u32 w = values[v];

				if (ro_reg)
					expect = results[0];
				else
					expect = reg_write(expect, w, rsvd);
				pr_info("Wrote %08x, read %08x, expect %08x\n",
					w, results[idx], expect);
				idx++;
			}
			for (v = 0; v < ARRAY_SIZE(values); v++) {
				u32 w = ~values[v];

				if (ro_reg)
					expect = results[0];
				else
					expect = reg_write(expect, w, rsvd);
				pr_info("Wrote %08x, read %08x, expect %08x\n",
					w, results[idx], expect);
				idx++;
			}

			err = -EINVAL;
		}
out_unmap_scratch:
		i915_gem_object_unpin_map(scratch->obj);
out_unmap_batch:
		if (cs)
			i915_gem_object_unpin_map(batch->obj);
out_ctx:
		intel_context_unpin(ce);
out:
		if (err == -EDEADLK) {
			err = i915_gem_ww_ctx_backoff(&ww);
			if (!err)
				goto retry;
		}
		i915_gem_ww_ctx_fini(&ww);
		if (err)
			break;
	}

	if (igt_flush_test(engine->i915))
		err = -EIO;

	i915_vma_unpin_and_release(&batch, 0);
out_scratch:
	i915_vma_unpin_and_release(&scratch, 0);
	return err;
}

static int live_dirty_whitelist(void *arg)
{
	struct intel_gt *gt = arg;
	struct intel_engine_cs *engine;
	enum intel_engine_id id;

	/* Can the user write to the whitelisted registers? */

	if (GRAPHICS_VER(gt->i915) < 7) /* minimum requirement for LRI, SRM, LRM */
		return 0;

	for_each_engine(engine, gt, id) {
		struct intel_context *ce;
		int err;

		if (engine->whitelist.count == 0)
			continue;

		ce = intel_context_create(engine);
		if (IS_ERR(ce))
			return PTR_ERR(ce);

		err = check_dirty_whitelist(ce);
		intel_context_put(ce);
		if (err)
			return err;
	}

	return 0;
}

static int live_reset_whitelist(void *arg)
{
	struct intel_gt *gt = arg;
	struct intel_engine_cs *engine;
	enum intel_engine_id id;
	int err = 0;

	/* If we reset the gpu, we should not lose the RING_NONPRIV */
	igt_global_reset_lock(gt);

	for_each_engine(engine, gt, id) {
		if (engine->whitelist.count == 0)
			continue;

		if (intel_has_reset_engine(gt)) {
			if (intel_engine_uses_guc(engine)) {
				struct intel_selftest_saved_policy saved;
				int err2;

				err = intel_selftest_modify_policy(engine, &saved,
								   SELFTEST_SCHEDULER_MODIFY_FAST_RESET);
				if (err)
					goto out;

				err = check_whitelist_across_reset(engine,
								   do_guc_reset,
								   "guc");

				err2 = intel_selftest_restore_policy(engine, &saved);
				if (err == 0)
					err = err2;
			} else {
				err = check_whitelist_across_reset(engine,
								   do_engine_reset,
								   "engine");
			}

			if (err)
				goto out;
		}

		if (intel_has_gpu_reset(gt)) {
			err = check_whitelist_across_reset(engine,
							   do_device_reset,
							   "device");
			if (err)
				goto out;
		}
	}

out:
	igt_global_reset_unlock(gt);
	return err;
}

static int read_whitelisted_registers(struct intel_context *ce,
				      struct i915_vma *results)
{
	struct intel_engine_cs *engine = ce->engine;
	struct i915_request *rq;
	int i, err = 0;
	u32 srm, *cs;

	rq = intel_context_create_request(ce);
	if (IS_ERR(rq))
		return PTR_ERR(rq);

	i915_vma_lock(results);
	err = i915_request_await_object(rq, results->obj, true);
	if (err == 0)
		err = i915_vma_move_to_active(results, rq, EXEC_OBJECT_WRITE);
	i915_vma_unlock(results);
	if (err)
		goto err_req;

	srm = MI_STORE_REGISTER_MEM;
	if (GRAPHICS_VER(engine->i915) >= 8)
		srm++;

	cs = intel_ring_begin(rq, 4 * engine->whitelist.count);
	if (IS_ERR(cs)) {
		err = PTR_ERR(cs);
		goto err_req;
	}

	for (i = 0; i < engine->whitelist.count; i++) {
		u64 offset = i915_vma_offset(results) + sizeof(u32) * i;

		*cs++ = srm;
		*cs++ = reg_offset(engine->whitelist.list[i].reg);
		*cs++ = lower_32_bits(offset);
		*cs++ = upper_32_bits(offset);
	}
	intel_ring_advance(rq, cs);

err_req:
	return request_add_sync(rq, err);
}

static int scrub_whitelisted_registers(struct intel_context *ce)
{
	struct intel_engine_cs *engine = ce->engine;
	struct i915_request *rq;
	struct i915_vma *batch;
	int i, err = 0;
	u32 *cs;

	batch = create_batch(ce->vm);
	if (IS_ERR(batch))
		return PTR_ERR(batch);

	cs = i915_gem_object_pin_map_unlocked(batch->obj, I915_MAP_WC);
	if (IS_ERR(cs)) {
		err = PTR_ERR(cs);
		goto err_batch;
	}

	*cs++ = MI_LOAD_REGISTER_IMM(engine->whitelist.count);
	for (i = 0; i < engine->whitelist.count; i++) {
		*cs++ = reg_offset(engine->whitelist.list[i].reg);
		*cs++ = 0xffffffff;
	}
	*cs++ = MI_BATCH_BUFFER_END;

	i915_gem_object_flush_map(batch->obj);
	intel_gt_chipset_flush(engine->gt);

	rq = intel_context_create_request(ce);
	if (IS_ERR(rq)) {
		err = PTR_ERR(rq);
		goto err_unpin;
	}

	if (engine->emit_init_breadcrumb) { /* Be nice if we hang */
		err = engine->emit_init_breadcrumb(rq);
		if (err)
			goto err_request;
	}

	i915_vma_lock(batch);
	err = i915_request_await_object(rq, batch->obj, false);
	if (err == 0)
		err = i915_vma_move_to_active(batch, rq, 0);
	i915_vma_unlock(batch);
	if (err)
		goto err_request;

	/* Perform the writes from an unprivileged "user" batch */
	err = engine->emit_bb_start(rq, i915_vma_offset(batch), 0, 0);

err_request:
	err = request_add_sync(rq, err);

err_unpin:
	i915_gem_object_unpin_map(batch->obj);
err_batch:
	i915_vma_unpin_and_release(&batch, 0);
	return err;
}

struct regmask {
	i915_reg_t reg;
	u8 graphics_ver;
	u32 platforms;
};

static bool find_reg(struct drm_i915_private *i915,
		     i915_reg_t reg,
		     const struct regmask *tbl,
		     unsigned long count)
{
	enum intel_platform platform = BIT(INTEL_INFO(i915)->platform);
	u32 offset = i915_mmio_reg_offset(reg);

	while (count--) {
		if (GRAPHICS_VER(i915) == tbl->graphics_ver &&
		    (!tbl->platforms || tbl->platforms & platform) &&
		    i915_mmio_reg_offset(tbl->reg) == offset)
			return true;
		tbl++;
	}

	return false;
}

static bool pardon_reg(struct drm_i915_private *i915, i915_reg_t reg)
{
	/* Alas, we must pardon some whitelists. Mistakes already made */
	static const struct regmask pardon[] = {
		{ GEN9_CTX_PREEMPT_REG, 9 },
		{ _MMIO(0xb118), 9 }, /* GEN8_L3SQCREG4 */
	};

	return find_reg(i915, reg, pardon, ARRAY_SIZE(pardon));
}

static bool result_eq(struct intel_engine_cs *engine,
		      u32 a, u32 b, i915_reg_t reg)
{
	if (wo_register(reg) || deny_register(reg)) /* read undefined */
		return true;

	if (ro_register(reg) || timestamp(engine, reg)) /* expect varying */
		return true;

	if (pardon_reg(engine->i915, reg)) /* mistakes were made */
		return true;

	if (a != b) {
		pr_err("%s: Whitelisted register 0x%4x[%s] not context saved: A=%08x, B=%08x\n",
		       engine->name, reg_offset(reg), repr_access(reg), a, b);
		return false;
	}

	return true;
}

static bool ignore_reg(struct drm_i915_private *i915, i915_reg_t reg)
{
	/* Some registers do not seem to behave and our writes unreadable */
	static const struct regmask wo[] = {
		{ GEN9_SLICE_COMMON_ECO_CHICKEN1, 9 },
		{ _MMIO(0x731c), .platforms = BIT(INTEL_GEMINILAKE) },
	};

	return find_reg(i915, reg, wo, ARRAY_SIZE(wo));
}

static bool result_neq(struct intel_engine_cs *engine,
		       u32 a, u32 b, i915_reg_t reg)
{
	if (wo_register(reg) || deny_register(reg)) /* read undefined */
		return true;

	if (ro_register(reg) || timestamp(engine, reg)) /* expect varying */
		return true;

	if (ignore_reg(engine->i915, reg)) /* fail for unknown reasons */
		return true;

	if (a == b) {
		pr_err("%s: Whitelist register 0x%4x[%s]:%08x was unwritable!\n",
		       engine->name, reg_offset(reg), repr_access(reg), a);
		return false;
	}

	return true;
}

static int
check_whitelisted_registers(struct intel_engine_cs *engine,
			    struct i915_vma *A,
			    struct i915_vma *B,
			    bool (*fn)(struct intel_engine_cs *engine,
				       u32 a, u32 b,
				       i915_reg_t reg))
{
	u32 *a, *b;
	int i, err;

	a = i915_gem_object_pin_map_unlocked(A->obj, I915_MAP_WB);
	if (IS_ERR(a))
		return PTR_ERR(a);

	b = i915_gem_object_pin_map_unlocked(B->obj, I915_MAP_WB);
	if (IS_ERR(b)) {
		err = PTR_ERR(b);
		goto err_a;
	}

	err = 0;
	for (i = 0; i < engine->whitelist.count; i++) {
		const struct i915_wa *wa = &engine->whitelist.list[i];

		if (!fn(engine, a[i], b[i], wa->reg))
			err = -EINVAL;
	}

	i915_gem_object_unpin_map(B->obj);
err_a:
	i915_gem_object_unpin_map(A->obj);
	return err;
}

static int live_isolated_whitelist(void *arg)
{
	struct intel_gt *gt = arg;
	struct {
		struct i915_vma *scratch[2];
	} client[2] = {};
	struct intel_engine_cs *engine;
	enum intel_engine_id id;
	int i, err = 0;

	/*
	 * Check that a write into a whitelist register works, but
	 * invisible to a second context.
	 */

	if (!intel_engines_has_context_isolation(gt->i915))
		return 0;

	for (i = 0; i < ARRAY_SIZE(client); i++) {
		client[i].scratch[0] =
			__vm_create_scratch_for_read_pinned(gt->vm, 4096);
		if (IS_ERR(client[i].scratch[0])) {
			err = PTR_ERR(client[i].scratch[0]);
			goto err;
		}

		client[i].scratch[1] =
			__vm_create_scratch_for_read_pinned(gt->vm, 4096);
		if (IS_ERR(client[i].scratch[1])) {
			err = PTR_ERR(client[i].scratch[1]);
			i915_vma_unpin_and_release(&client[i].scratch[0], 0);
			goto err;
		}
	}

	for_each_engine(engine, gt, id) {
		struct intel_context *ce[2];

		if (!engine->whitelist.count)
			continue;

		if (!engine->kernel_context->vm)
			continue;

		ce[0] = intel_context_create(engine);
		if (IS_ERR(ce[0])) {
			err = PTR_ERR(ce[0]);
			break;
		}
		ce[1] = intel_context_create(engine);
		if (IS_ERR(ce[1])) {
			err = PTR_ERR(ce[1]);
			intel_context_put(ce[0]);
			break;
		}

		/* Read default values */
		err = read_whitelisted_registers(ce[0], client[0].scratch[0]);
		if (err)
			goto err_ce;

		/* Try to overwrite registers (should only affect ctx0) */
		err = scrub_whitelisted_registers(ce[0]);
		if (err)
			goto err_ce;

		/* Read values from ctx1, we expect these to be defaults */
		err = read_whitelisted_registers(ce[1], client[1].scratch[0]);
		if (err)
			goto err_ce;

		/* Verify that both reads return the same default values */
		err = check_whitelisted_registers(engine,
						  client[0].scratch[0],
						  client[1].scratch[0],
						  result_eq);
		if (err)
			goto err_ce;

		/* Read back the updated values in ctx0 */
		err = read_whitelisted_registers(ce[0], client[0].scratch[1]);
		if (err)
			goto err_ce;

		/* User should be granted privilege to overwhite regs */
		err = check_whitelisted_registers(engine,
						  client[0].scratch[0],
						  client[0].scratch[1],
						  result_neq);
err_ce:
		intel_context_put(ce[1]);
		intel_context_put(ce[0]);
		if (err)
			break;
	}

err:
	for (i = 0; i < ARRAY_SIZE(client); i++) {
		i915_vma_unpin_and_release(&client[i].scratch[1], 0);
		i915_vma_unpin_and_release(&client[i].scratch[0], 0);
	}

	if (igt_flush_test(gt->i915))
		err = -EIO;

	return err;
}

static void set_error_once(int *err, int result)
{
	cmpxchg(err, 0, result);
}

static int
verify_wa_lists(struct intel_gt *gt, struct wa_lists *lists,
		const char *str)
{
	struct intel_engine_cs *engine;
	enum intel_engine_id id;
	int err = 0;

	set_error_once(&err, wa_list_verify(gt, &lists->gt_wa_list, wa_verify, (void *)str));

	for_each_engine(engine, gt, id) {
		struct intel_context *ce;

		ce = intel_context_create(engine);
		if (IS_ERR(ce))
			return false;

		set_error_once(&err,
			       engine_wa_list_verify(ce,
						     &lists->engine[id].wa_list,
						     wa_verify,
						     (void *)str));

		set_error_once(&err,
			       engine_wa_list_verify(ce,
						     &lists->engine[id].ctx_wa_list,
						     wa_verify,
						     (void *)str));

		intel_context_put(ce);
	}

	return err;
}

static int
live_gpu_reset_workarounds(void *arg)
{
	struct intel_gt *gt = arg;
	intel_wakeref_t wakeref;
	struct wa_lists *lists;
	int err;

	if (!intel_has_gpu_reset(gt))
		return 0;

	lists = kzalloc(sizeof(*lists), GFP_KERNEL);
	if (!lists)
		return -ENOMEM;

	pr_info("Verifying after GPU reset...\n");

	igt_global_reset_lock(gt);
	wakeref = intel_runtime_pm_get(gt->uncore->rpm);

	reference_lists_init(gt, lists);

	err = verify_wa_lists(gt, lists, "before reset");
	if (err)
		goto out;

	intel_gt_reset(gt, ALL_ENGINES, "live_workarounds");

	err = verify_wa_lists(gt, lists, "after reset");

out:
	reference_lists_fini(gt, lists);
	intel_runtime_pm_put(gt->uncore->rpm, wakeref);
	igt_global_reset_unlock(gt);
	kfree(lists);

	return err;
}

static int
live_engine_reset_workarounds(void *arg)
{
	struct intel_gt *gt = arg;
	struct intel_engine_cs *engine;
	enum intel_engine_id id;
	struct intel_context *ce;
	struct igt_spinner spin;
	struct i915_request *rq;
	intel_wakeref_t wakeref;
	struct wa_lists *lists;
	int ret = 0;

	if (!intel_has_reset_engine(gt))
		return 0;

	lists = kzalloc(sizeof(*lists), GFP_KERNEL);
	if (!lists)
		return -ENOMEM;

	igt_global_reset_lock(gt);
	wakeref = intel_runtime_pm_get(gt->uncore->rpm);

	reference_lists_init(gt, lists);

	for_each_engine(engine, gt, id) {
		struct intel_selftest_saved_policy saved;
		bool using_guc = intel_engine_uses_guc(engine);
		int ret2;

		pr_info("Verifying after %s reset...\n", engine->name);
		ret = intel_selftest_modify_policy(engine, &saved,
						   SELFTEST_SCHEDULER_MODIFY_FAST_RESET);
		if (ret)
			break;

		ce = intel_context_create(engine);
		if (IS_ERR(ce)) {
			ret = PTR_ERR(ce);
			goto restore;
		}

		if (!using_guc) {
			ret = verify_wa_lists(gt, lists, "before reset");
			if (ret)
				goto err;

			ret = intel_engine_reset(engine, "live_workarounds:idle");
			if (ret) {
				pr_err("%s: Reset failed while idle\n", engine->name);
				goto err;
			}

			ret = verify_wa_lists(gt, lists, "after idle reset");
			if (ret)
				goto err;
		}

		ret = igt_spinner_init(&spin, engine->gt);
		if (ret)
			goto err;

		rq = igt_spinner_create_request(&spin, ce, MI_NOOP);
		if (IS_ERR(rq)) {
			ret = PTR_ERR(rq);
			igt_spinner_fini(&spin);
			goto err;
		}

		ret = request_add_spin(rq, &spin);
		if (ret) {
			pr_err("%s: Spinner failed to start\n", engine->name);
			igt_spinner_fini(&spin);
			goto err;
		}

		/* Ensure the spinner hasn't aborted */
		if (i915_request_completed(rq)) {
			ret = -ETIMEDOUT;
			goto skip;
		}

		if (!using_guc) {
			ret = intel_engine_reset(engine, "live_workarounds:active");
			if (ret) {
				pr_err("%s: Reset failed on an active spinner\n",
				       engine->name);
				igt_spinner_fini(&spin);
				goto err;
			}
		}

		/* Ensure the reset happens and kills the engine */
		if (ret == 0)
			ret = intel_selftest_wait_for_rq(rq);

skip:
		igt_spinner_end(&spin);
		igt_spinner_fini(&spin);

		if (ret == 0)
			ret = verify_wa_lists(gt, lists, "after busy reset");

err:
		intel_context_put(ce);

restore:
		ret2 = intel_selftest_restore_policy(engine, &saved);
		if (ret == 0)
			ret = ret2;
		if (ret)
			break;
	}

	reference_lists_fini(gt, lists);
	intel_runtime_pm_put(gt->uncore->rpm, wakeref);
	igt_global_reset_unlock(gt);
	kfree(lists);

	igt_flush_test(gt->i915);

	return ret;
}

int intel_workarounds_live_selftests(struct drm_i915_private *i915)
{
	static const struct i915_subtest tests[] = {
		SUBTEST(live_dirty_whitelist),
		SUBTEST(live_reset_whitelist),
		SUBTEST(live_isolated_whitelist),
		SUBTEST(live_gpu_reset_workarounds),
		SUBTEST(live_engine_reset_workarounds),
	};
	struct intel_gt *gt;
	unsigned int i;
	int ret = 0;

	for_each_gt(gt, i915, i) {
		if (intel_gt_is_wedged(gt))
			continue;

		ret = intel_gt_live_subtests(tests, gt);
		if (ret)
			break;
	}

	return ret;
}
