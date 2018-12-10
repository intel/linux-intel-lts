/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2018 Intel Corporation
 */

#include "../i915_selftest.h"
#include "igt_flush_test.h"

#include "mock_context.h"

struct spinner {
	struct drm_i915_private *i915;
	struct drm_i915_gem_object *hws;
	struct drm_i915_gem_object *obj;
	u32 *batch;
	void *seqno;
};

static int spinner_init(struct spinner *spin, struct drm_i915_private *i915)
{
	unsigned int mode;
	void *vaddr;
	int err;

	GEM_BUG_ON(INTEL_GEN(i915) < 8);

	memset(spin, 0, sizeof(*spin));
	spin->i915 = i915;

	spin->hws = i915_gem_object_create_internal(i915, PAGE_SIZE);
	if (IS_ERR(spin->hws)) {
		err = PTR_ERR(spin->hws);
		goto err;
	}

	spin->obj = i915_gem_object_create_internal(i915, PAGE_SIZE);
	if (IS_ERR(spin->obj)) {
		err = PTR_ERR(spin->obj);
		goto err_hws;
	}

	i915_gem_object_set_cache_level(spin->hws, I915_CACHE_LLC);
	vaddr = i915_gem_object_pin_map(spin->hws, I915_MAP_WB);
	if (IS_ERR(vaddr)) {
		err = PTR_ERR(vaddr);
		goto err_obj;
	}
	spin->seqno = memset(vaddr, 0xff, PAGE_SIZE);

	mode = HAS_LLC(i915) ? I915_MAP_WB : I915_MAP_WC;
	vaddr = i915_gem_object_pin_map(spin->obj, mode);
	if (IS_ERR(vaddr)) {
		err = PTR_ERR(vaddr);
		goto err_unpin_hws;
	}
	spin->batch = vaddr;

	return 0;

err_unpin_hws:
	i915_gem_object_unpin_map(spin->hws);
err_obj:
	i915_gem_object_put(spin->obj);
err_hws:
	i915_gem_object_put(spin->hws);
err:
	return err;
}

static unsigned int seqno_offset(u64 fence)
{
	return offset_in_page(sizeof(u32) * fence);
}

static u64 hws_address(const struct i915_vma *hws,
		       const struct i915_request *rq)
{
	return hws->node.start + seqno_offset(rq->fence.context);
}

static int emit_recurse_batch(struct spinner *spin,
			      struct i915_request *rq,
			      u32 arbitration_command)
{
	struct i915_address_space *vm = &rq->gem_context->ppgtt->vm;
	struct i915_vma *hws, *vma;
	u32 *batch;
	int err;

	vma = i915_vma_instance(spin->obj, vm, NULL);
	if (IS_ERR(vma))
		return PTR_ERR(vma);

	hws = i915_vma_instance(spin->hws, vm, NULL);
	if (IS_ERR(hws))
		return PTR_ERR(hws);

	err = i915_vma_pin(vma, 0, 0, PIN_USER);
	if (err)
		return err;

	err = i915_vma_pin(hws, 0, 0, PIN_USER);
	if (err)
		goto unpin_vma;

	err = i915_vma_move_to_active(vma, rq, 0);
	if (err)
		goto unpin_hws;

	if (!i915_gem_object_has_active_reference(vma->obj)) {
		i915_gem_object_get(vma->obj);
		i915_gem_object_set_active_reference(vma->obj);
	}

	err = i915_vma_move_to_active(hws, rq, 0);
	if (err)
		goto unpin_hws;

	if (!i915_gem_object_has_active_reference(hws->obj)) {
		i915_gem_object_get(hws->obj);
		i915_gem_object_set_active_reference(hws->obj);
	}

	batch = spin->batch;

	*batch++ = MI_STORE_DWORD_IMM_GEN4;
	*batch++ = lower_32_bits(hws_address(hws, rq));
	*batch++ = upper_32_bits(hws_address(hws, rq));
	*batch++ = rq->fence.seqno;

	*batch++ = arbitration_command;

	*batch++ = MI_BATCH_BUFFER_START | 1 << 8 | 1;
	*batch++ = lower_32_bits(vma->node.start);
	*batch++ = upper_32_bits(vma->node.start);
	*batch++ = MI_BATCH_BUFFER_END; /* not reached */

	i915_gem_chipset_flush(spin->i915);

	err = rq->engine->emit_bb_start(rq, vma->node.start, PAGE_SIZE, 0);

unpin_hws:
	i915_vma_unpin(hws);
unpin_vma:
	i915_vma_unpin(vma);
	return err;
}

static struct i915_request *
spinner_create_request(struct spinner *spin,
		       struct i915_gem_context *ctx,
		       struct intel_engine_cs *engine,
		       u32 arbitration_command)
{
	struct i915_request *rq;
	int err;

	rq = i915_request_alloc(engine, ctx);
	if (IS_ERR(rq))
		return rq;

	err = emit_recurse_batch(spin, rq, arbitration_command);
	if (err) {
		i915_request_add(rq);
		return ERR_PTR(err);
	}

	return rq;
}

static u32 hws_seqno(const struct spinner *spin, const struct i915_request *rq)
{
	u32 *seqno = spin->seqno + seqno_offset(rq->fence.context);

	return READ_ONCE(*seqno);
}

static void spinner_end(struct spinner *spin)
{
	*spin->batch = MI_BATCH_BUFFER_END;
	i915_gem_chipset_flush(spin->i915);
}

static void spinner_fini(struct spinner *spin)
{
	spinner_end(spin);

	i915_gem_object_unpin_map(spin->obj);
	i915_gem_object_put(spin->obj);

	i915_gem_object_unpin_map(spin->hws);
	i915_gem_object_put(spin->hws);
}

static bool wait_for_spinner(struct spinner *spin, struct i915_request *rq)
{
	if (!wait_event_timeout(rq->execute,
				READ_ONCE(rq->global_seqno),
				msecs_to_jiffies(10)))
		return false;

	return !(wait_for_us(i915_seqno_passed(hws_seqno(spin, rq),
					       rq->fence.seqno),
			     10) &&
		 wait_for(i915_seqno_passed(hws_seqno(spin, rq),
					    rq->fence.seqno),
			  1000));
}

static int live_sanitycheck(void *arg)
{
	struct drm_i915_private *i915 = arg;
	struct intel_engine_cs *engine;
	struct i915_gem_context *ctx;
	enum intel_engine_id id;
	struct spinner spin;
	int err = -ENOMEM;

	if (!HAS_LOGICAL_RING_CONTEXTS(i915))
		return 0;

	mutex_lock(&i915->drm.struct_mutex);

	if (spinner_init(&spin, i915))
		goto err_unlock;

	ctx = kernel_context(i915);
	if (!ctx)
		goto err_spin;

	for_each_engine(engine, i915, id) {
		struct i915_request *rq;

		rq = spinner_create_request(&spin, ctx, engine, MI_NOOP);
		if (IS_ERR(rq)) {
			err = PTR_ERR(rq);
			goto err_ctx;
		}

		i915_request_add(rq);
		if (!wait_for_spinner(&spin, rq)) {
			GEM_TRACE("spinner failed to start\n");
			GEM_TRACE_DUMP();
			i915_gem_set_wedged(i915);
			err = -EIO;
			goto err_ctx;
		}

		spinner_end(&spin);
		if (igt_flush_test(i915, I915_WAIT_LOCKED)) {
			err = -EIO;
			goto err_ctx;
		}
	}

	err = 0;
err_ctx:
	kernel_context_close(ctx);
err_spin:
	spinner_fini(&spin);
err_unlock:
	igt_flush_test(i915, I915_WAIT_LOCKED);
	mutex_unlock(&i915->drm.struct_mutex);
	return err;
}

static int live_preempt(void *arg)
{
	struct drm_i915_private *i915 = arg;
	struct i915_gem_context *ctx_hi, *ctx_lo;
	struct spinner spin_hi, spin_lo;
	struct intel_engine_cs *engine;
	enum intel_engine_id id;
	int err = -ENOMEM;

	if (!HAS_LOGICAL_RING_PREEMPTION(i915))
		return 0;

	mutex_lock(&i915->drm.struct_mutex);

	if (spinner_init(&spin_hi, i915))
		goto err_unlock;

	if (spinner_init(&spin_lo, i915))
		goto err_spin_hi;

	ctx_hi = kernel_context(i915);
	if (!ctx_hi)
		goto err_spin_lo;
	ctx_hi->sched.priority = I915_CONTEXT_MAX_USER_PRIORITY;

	ctx_lo = kernel_context(i915);
	if (!ctx_lo)
		goto err_ctx_hi;
	ctx_lo->sched.priority = I915_CONTEXT_MIN_USER_PRIORITY;

	for_each_engine(engine, i915, id) {
		struct i915_request *rq;

		rq = spinner_create_request(&spin_lo, ctx_lo, engine,
					    MI_ARB_CHECK);
		if (IS_ERR(rq)) {
			err = PTR_ERR(rq);
			goto err_ctx_lo;
		}

		i915_request_add(rq);
		if (!wait_for_spinner(&spin_lo, rq)) {
			GEM_TRACE("lo spinner failed to start\n");
			GEM_TRACE_DUMP();
			i915_gem_set_wedged(i915);
			err = -EIO;
			goto err_ctx_lo;
		}

		rq = spinner_create_request(&spin_hi, ctx_hi, engine,
					    MI_ARB_CHECK);
		if (IS_ERR(rq)) {
			spinner_end(&spin_lo);
			err = PTR_ERR(rq);
			goto err_ctx_lo;
		}

		i915_request_add(rq);
		if (!wait_for_spinner(&spin_hi, rq)) {
			GEM_TRACE("hi spinner failed to start\n");
			GEM_TRACE_DUMP();
			i915_gem_set_wedged(i915);
			err = -EIO;
			goto err_ctx_lo;
		}

		spinner_end(&spin_hi);
		spinner_end(&spin_lo);
		if (igt_flush_test(i915, I915_WAIT_LOCKED)) {
			err = -EIO;
			goto err_ctx_lo;
		}
	}

	err = 0;
err_ctx_lo:
	kernel_context_close(ctx_lo);
err_ctx_hi:
	kernel_context_close(ctx_hi);
err_spin_lo:
	spinner_fini(&spin_lo);
err_spin_hi:
	spinner_fini(&spin_hi);
err_unlock:
	igt_flush_test(i915, I915_WAIT_LOCKED);
	mutex_unlock(&i915->drm.struct_mutex);
	return err;
}

static int live_late_preempt(void *arg)
{
	struct drm_i915_private *i915 = arg;
	struct i915_gem_context *ctx_hi, *ctx_lo;
	struct spinner spin_hi, spin_lo;
	struct intel_engine_cs *engine;
	struct i915_sched_attr attr = {};
	enum intel_engine_id id;
	int err = -ENOMEM;

	if (!HAS_LOGICAL_RING_PREEMPTION(i915))
		return 0;

	mutex_lock(&i915->drm.struct_mutex);

	if (spinner_init(&spin_hi, i915))
		goto err_unlock;

	if (spinner_init(&spin_lo, i915))
		goto err_spin_hi;

	ctx_hi = kernel_context(i915);
	if (!ctx_hi)
		goto err_spin_lo;

	ctx_lo = kernel_context(i915);
	if (!ctx_lo)
		goto err_ctx_hi;

	for_each_engine(engine, i915, id) {
		struct i915_request *rq;

		rq = spinner_create_request(&spin_lo, ctx_lo, engine,
					    MI_ARB_CHECK);
		if (IS_ERR(rq)) {
			err = PTR_ERR(rq);
			goto err_ctx_lo;
		}

		i915_request_add(rq);
		if (!wait_for_spinner(&spin_lo, rq)) {
			pr_err("First context failed to start\n");
			goto err_wedged;
		}

		rq = spinner_create_request(&spin_hi, ctx_hi, engine, MI_NOOP);
		if (IS_ERR(rq)) {
			spinner_end(&spin_lo);
			err = PTR_ERR(rq);
			goto err_ctx_lo;
		}

		i915_request_add(rq);
		if (wait_for_spinner(&spin_hi, rq)) {
			pr_err("Second context overtook first?\n");
			goto err_wedged;
		}

		attr.priority = I915_PRIORITY_MAX;
		engine->schedule(rq, &attr);

		if (!wait_for_spinner(&spin_hi, rq)) {
			pr_err("High priority context failed to preempt the low priority context\n");
			GEM_TRACE_DUMP();
			goto err_wedged;
		}

		spinner_end(&spin_hi);
		spinner_end(&spin_lo);
		if (igt_flush_test(i915, I915_WAIT_LOCKED)) {
			err = -EIO;
			goto err_ctx_lo;
		}
	}

	err = 0;
err_ctx_lo:
	kernel_context_close(ctx_lo);
err_ctx_hi:
	kernel_context_close(ctx_hi);
err_spin_lo:
	spinner_fini(&spin_lo);
err_spin_hi:
	spinner_fini(&spin_hi);
err_unlock:
	igt_flush_test(i915, I915_WAIT_LOCKED);
	mutex_unlock(&i915->drm.struct_mutex);
	return err;

err_wedged:
	spinner_end(&spin_hi);
	spinner_end(&spin_lo);
	i915_gem_set_wedged(i915);
	err = -EIO;
	goto err_ctx_lo;
}

static int live_preempt_hang(void *arg)
{
	struct drm_i915_private *i915 = arg;
	struct i915_gem_context *ctx_hi, *ctx_lo;
	struct spinner spin_hi, spin_lo;
	struct intel_engine_cs *engine;
	enum intel_engine_id id;
	int err = -ENOMEM;

	if (!HAS_LOGICAL_RING_PREEMPTION(i915))
		return 0;

	if (!intel_has_reset_engine(i915))
		return 0;

	mutex_lock(&i915->drm.struct_mutex);

	if (spinner_init(&spin_hi, i915))
		goto err_unlock;

	if (spinner_init(&spin_lo, i915))
		goto err_spin_hi;

	ctx_hi = kernel_context(i915);
	if (!ctx_hi)
		goto err_spin_lo;
	ctx_hi->sched.priority = I915_CONTEXT_MAX_USER_PRIORITY;

	ctx_lo = kernel_context(i915);
	if (!ctx_lo)
		goto err_ctx_hi;
	ctx_lo->sched.priority = I915_CONTEXT_MIN_USER_PRIORITY;

	for_each_engine(engine, i915, id) {
		struct i915_request *rq;

		if (!intel_engine_has_preemption(engine))
			continue;

		rq = spinner_create_request(&spin_lo, ctx_lo, engine,
					    MI_ARB_CHECK);
		if (IS_ERR(rq)) {
			err = PTR_ERR(rq);
			goto err_ctx_lo;
		}

		i915_request_add(rq);
		if (!wait_for_spinner(&spin_lo, rq)) {
			GEM_TRACE("lo spinner failed to start\n");
			GEM_TRACE_DUMP();
			i915_gem_set_wedged(i915);
			err = -EIO;
			goto err_ctx_lo;
		}

		rq = spinner_create_request(&spin_hi, ctx_hi, engine,
					    MI_ARB_CHECK);
		if (IS_ERR(rq)) {
			spinner_end(&spin_lo);
			err = PTR_ERR(rq);
			goto err_ctx_lo;
		}

		init_completion(&engine->execlists.preempt_hang.completion);
		engine->execlists.preempt_hang.inject_hang = true;

		i915_request_add(rq);

		if (!wait_for_completion_timeout(&engine->execlists.preempt_hang.completion,
						 HZ / 10)) {
			pr_err("Preemption did not occur within timeout!");
			GEM_TRACE_DUMP();
			i915_gem_set_wedged(i915);
			err = -EIO;
			goto err_ctx_lo;
		}

		set_bit(I915_RESET_ENGINE + id, &i915->gpu_error.flags);
		i915_reset_engine(engine, NULL);
		clear_bit(I915_RESET_ENGINE + id, &i915->gpu_error.flags);

		engine->execlists.preempt_hang.inject_hang = false;

		if (!wait_for_spinner(&spin_hi, rq)) {
			GEM_TRACE("hi spinner failed to start\n");
			GEM_TRACE_DUMP();
			i915_gem_set_wedged(i915);
			err = -EIO;
			goto err_ctx_lo;
		}

		spinner_end(&spin_hi);
		spinner_end(&spin_lo);
		if (igt_flush_test(i915, I915_WAIT_LOCKED)) {
			err = -EIO;
			goto err_ctx_lo;
		}
	}

	err = 0;
err_ctx_lo:
	kernel_context_close(ctx_lo);
err_ctx_hi:
	kernel_context_close(ctx_hi);
err_spin_lo:
	spinner_fini(&spin_lo);
err_spin_hi:
	spinner_fini(&spin_hi);
err_unlock:
	igt_flush_test(i915, I915_WAIT_LOCKED);
	mutex_unlock(&i915->drm.struct_mutex);
	return err;
}

static void mark_preemption_hang(struct intel_engine_execlists *execlists)
{
	execlists_set_active(execlists, EXECLISTS_ACTIVE_PREEMPT);
	execlists_set_active(execlists, EXECLISTS_ACTIVE_PREEMPT_TIMEOUT);
}

static int live_preempt_timeout(void *arg)
{
	struct drm_i915_private *i915 = arg;
	struct intel_engine_cs *engine;
	struct i915_gem_context *ctx;
	enum intel_engine_id id;
	struct spinner spin;
	int err = -ENOMEM;

	if (!HAS_LOGICAL_RING_PREEMPTION(i915))
		return 0;

	mutex_lock(&i915->drm.struct_mutex);

	if (spinner_init(&spin, i915))
		goto err_unlock;

	ctx = kernel_context(i915);
	if (!ctx)
		goto err_spin;

	for_each_engine(engine, i915, id) {
		struct i915_request *rq;

		rq = spinner_create_request(&spin, ctx, engine, MI_NOOP);
		if (IS_ERR(rq)) {
			err = PTR_ERR(rq);
			goto err_ctx;
		}

		i915_request_add(rq);
		if (!wait_for_spinner(&spin, rq)) {
			i915_gem_set_wedged(i915);
			err = -EIO;
			goto err_ctx;
		}

		GEM_TRACE("%s triggering reset\n", engine->name);
		mark_preemption_hang(&engine->execlists);
		preempt_reset(&engine->execlists.preempt_reset);

		if (igt_flush_test(i915, I915_WAIT_LOCKED)) {
			err = -EIO;
			goto err_ctx;
		}
	}

	err = 0;
err_ctx:
	kernel_context_close(ctx);
err_spin:
	spinner_fini(&spin);
err_unlock:
	igt_flush_test(i915, I915_WAIT_LOCKED);
	mutex_unlock(&i915->drm.struct_mutex);
	return err;
}

static void __softirq_begin(void)
{
	local_bh_disable();
}

static void __softirq_end(void)
{
	local_bh_enable();
}

static void __hardirq_begin(void)
{
	local_irq_disable();
}

static void __hardirq_end(void)
{
	local_irq_enable();
}

static int live_preempt_reset(void *arg)
{
	struct drm_i915_private *i915 = arg;
	struct intel_engine_cs *engine;
	struct i915_gem_context *ctx;
	enum intel_engine_id id;
	struct spinner spin;
	int err = -ENOMEM;

	if (!HAS_LOGICAL_RING_PREEMPTION(i915))
		return 0;

	mutex_lock(&i915->drm.struct_mutex);

	if (spinner_init(&spin, i915))
		goto err_unlock;

	ctx = kernel_context(i915);
	if (!ctx)
		goto err_spin;

	for_each_engine(engine, i915, id) {
		static const struct {
			const char *name;
			void (*critical_section_begin)(void);
			void (*critical_section_end)(void);
		} phases[] = {
			{ "softirq", __softirq_begin, __softirq_end },
			{ "hardirq", __hardirq_begin, __hardirq_end },
			{ }
		};
		struct tasklet_struct *t = &engine->execlists.tasklet;
		const typeof(*phases) *p;

		for (p = phases; p->name; p++) {
			struct i915_request *rq;

			rq = spinner_create_request(&spin, ctx, engine,
						    MI_NOOP);
			if (IS_ERR(rq)) {
				err = PTR_ERR(rq);
				goto err_ctx;
			}

			i915_request_add(rq);
			if (!wait_for_spinner(&spin, rq)) {
				i915_gem_set_wedged(i915);
				err = -EIO;
				goto err_ctx;
			}

			/* Flush to give try_preempt_reset a chance */
			tasklet_schedule(t);
			tasklet_kill(t);
			GEM_BUG_ON(i915_request_completed(rq));

			GEM_TRACE("%s triggering %s reset\n",
				  engine->name, p->name);
			p->critical_section_begin();

			mark_preemption_hang(&engine->execlists);
			err = try_preempt_reset(&engine->execlists);

			p->critical_section_end();
			if (err) {
				pr_err("Preempt softirq reset failed on %s, tasklet state %lx\n",
				       engine->name, t->state);
				spinner_end(&spin);
				i915_gem_set_wedged(i915);
				goto err_ctx;
			}

			if (igt_flush_test(i915, I915_WAIT_LOCKED)) {
				err = -EIO;
				goto err_ctx;
			}
		}
	}

	err = 0;
err_ctx:
	kernel_context_close(ctx);
err_spin:
	spinner_fini(&spin);
err_unlock:
	igt_flush_test(i915, I915_WAIT_LOCKED);
	mutex_unlock(&i915->drm.struct_mutex);
	return err;
}

int intel_execlists_live_selftests(struct drm_i915_private *i915)
{
	static const struct i915_subtest tests[] = {
		SUBTEST(live_sanitycheck),
		SUBTEST(live_preempt),
		SUBTEST(live_late_preempt),
		SUBTEST(live_preempt_hang),
		SUBTEST(live_preempt_timeout),
		SUBTEST(live_preempt_reset),
	};

	if (!HAS_EXECLISTS(i915))
		return 0;

	if (i915_terminally_wedged(&i915->gpu_error))
		return 0;

	return i915_subtests(tests, i915);
}
