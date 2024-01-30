// SPDX-License-Identifier: MIT
/*
 * Copyright © 2019 Intel Corporation
 */

#include "i915_selftest.h"

#include "i915_random.h"
#include "igt_live_test.h"
#include "gt/intel_lrc_reg.h"

static int __igt_emit_wait_token(struct i915_request *rq, struct i915_vma *vma,
				 u64 offset, u32 value, u32 token)
{
	u32 *cs;

	if (HAS_SEMAPHORE_XEHPSDV(vma->vm->i915))
		token <<= MI_SEMAPHORE_256_TOKEN_SHIFT;
	else
		token <<= MI_SEMAPHORE_27_TOKEN_SHIFT;

	cs = intel_ring_begin(rq, 6);
	if (IS_ERR(cs))
		return PTR_ERR(cs);

	*cs++ = MI_SEMAPHORE_WAIT_TOKEN |
		MI_SEMAPHORE_GLOBAL_GTT |
		MI_SEMAPHORE_SAD_EQ_SDD;
	*cs++ = value;
	*cs++ = lower_32_bits(i915_ggtt_offset(vma) + offset);
	*cs++ = upper_32_bits(i915_ggtt_offset(vma) + offset);
	*cs++ = token;

	*cs++ = MI_NOOP;

	intel_ring_advance(rq, cs);

	return 0;
}

static int __igt_emit_signal(struct i915_request *rq, struct i915_vma *vma,
			     u64 offset, u32 value)
{
	u32 *cs;

	cs = intel_ring_begin(rq, 6);
	if (IS_ERR(cs))
		return PTR_ERR(cs);

	*cs++ = MI_STORE_DWORD_IMM_GEN4 | 1 << 22;
	*cs++ = lower_32_bits(i915_ggtt_offset(vma) + offset);
	*cs++ = upper_32_bits(i915_ggtt_offset(vma) + offset);
	*cs++ = value;

	*cs++ = MI_SEMAPHORE_SIGNAL;
	*cs++ = MI_NOOP;

	intel_ring_advance(rq, cs);

	return 0;
}

static u32 __igt_random_token(struct drm_i915_private *i915,
			      struct rnd_state *state)
{
	u32 token;

	if (HAS_SEMAPHORE_XEHPSDV(i915))
		token = i915_prandom_u32_max_state(
			XEHPSDV_ENGINE_SEMAPHORE_TOKEN_MAX, state);
	else
		token = i915_prandom_u32_max_state(
			GEN12_ENGINE_SEMAPHORE_TOKEN_MAX, state);

	return token;
}

static int __igt_semaphore_token_init_ctx(struct intel_context *ce, u32 token)
{
	u32 *regs;
	int ret;

	ret = intel_context_pin(ce);
	if (ret)
		return ret;

	regs = ce->lrc_reg_state;
	regs[CTX_CONTEXT_CONTROL] =
		regs[CTX_CONTEXT_CONTROL] &
		_MASKED_BIT_DISABLE(CTX_CTRL_INHIBIT_SYN_CTX_SWITCH);
	if (HAS_SEMAPHORE_XEHPSDV(ce->engine->i915))
		regs[GEN12_CTX_SEMAPHORE_TOKEN] =
			XEHPSDV_ENGINE_SEMAPHORE_TOKEN_CTX_VALUE(token);
	else
		regs[GEN12_CTX_SEMAPHORE_TOKEN] =
			GEN12_ENGINE_SEMAPHORE_TOKEN_CTX_VALUE(token);

	intel_context_unpin(ce);

	return 0;
}

static struct i915_request *
__igt_semaphore_token_alloc_rq(struct drm_i915_private *i915,
			       struct intel_engine_cs *engine, u32 token)
{
	struct intel_context *ce;
	struct i915_request *rq;
	int err;

	ce = intel_context_create(engine);
	if (IS_ERR(ce)) {
		err = PTR_ERR(ce);
		goto out;
	}

	err = __igt_semaphore_token_init_ctx(ce, token);
	if (err)
		goto out_ce;

	rq = intel_context_create_request(ce);
	if (IS_ERR(rq)) {
		err = PTR_ERR(rq);
		goto out_ce;
	}

out_ce:
	intel_context_put(ce);
out:
	if (err)
		return ERR_PTR(err);

	return rq;
}

#define IGT_WAITER_RUNNING_TIMEOUT 500
#define IGT_SIGNAL_DONE_TIMEOUT 500
#define IGT_WAIT_DONE_TIMEOUT 1000
#define IGT_SEM_TOKEN_VALUE 0xfee1dead

static int igt_semaphore_token(void *arg)
{
	struct intel_gt *gt = arg;
	struct drm_i915_private *i915 = gt->i915;
	unsigned int semaphore_offset = 0;
	struct intel_engine_cs *engine_signal, *engine_wait;
	enum intel_engine_id signal_id, wait_id;
	struct drm_i915_gem_object *obj;
	struct i915_request *waiters[I915_NUM_ENGINES];
	struct i915_vma *vma;
	void *vaddr;
	I915_RND_STATE(prng);
	unsigned int i;
	int err;

	obj = i915_gem_object_create_internal(i915, PAGE_SIZE);
	if (IS_ERR(obj)) {
		err = PTR_ERR(obj);
		goto out;
	}

	i915_gem_object_lock(obj, NULL);

	vaddr = i915_gem_object_pin_map(obj, I915_MAP_WB);
	if (IS_ERR(vaddr)) {
		err = PTR_ERR(vaddr);
		goto out_obj;
	}
	memset(vaddr, 0, PAGE_SIZE);
	i915_gem_object_unpin_map(obj);

	err = i915_gem_object_set_to_gtt_domain(obj, true);
	if (err) {
		i915_gem_object_unlock(obj);
		goto out_obj;
	}
	i915_gem_object_unlock(obj);

	vma = i915_gem_object_ggtt_pin(obj, gt->ggtt, NULL, 0, 0, 0);
	if (IS_ERR(vma)) {
		err = PTR_ERR(vma);
		goto out_obj;
	}

	for_each_engine(engine_signal, gt, signal_id) {
		u32 token_signal = __igt_random_token(i915, &prng);
		unsigned int w = 0;
		struct i915_request *signaler;

		for_each_engine(engine_wait, gt, wait_id) {
			u32 token_wait = __igt_random_token(i915, &prng);
			struct i915_request *rq;

			rq = __igt_semaphore_token_alloc_rq(i915, engine_wait,
							    token_wait);
			if (IS_ERR(rq)) {
				err = PTR_ERR(rq);
				goto out_vma;
			}
			i915_request_get(rq);
			waiters[w++] = rq;

			if (rq->engine->emit_init_breadcrumb) {
				err = rq->engine->emit_init_breadcrumb(rq);
				if (err) {
					i915_request_add(rq);
					goto out_waiters;
				}
			}

			err = __igt_emit_wait_token(rq, vma,
				semaphore_offset * sizeof(u32),
				IGT_SEM_TOKEN_VALUE, token_signal);

			i915_request_add(rq);

			if (err)
				goto out_waiters;

			wait_for(i915_request_is_running(rq), IGT_WAITER_RUNNING_TIMEOUT);
			if (!i915_request_is_running(rq)) {
				drm_err(&i915->drm, "Request failed to start\n");
				err = -EIO;
				goto out_waiters;
			}

			if (i915_request_completed(rq)) {
				drm_err(&i915->drm, "Request completed before the semaphore is signalled\n");
				err = -EIO;
				goto out_waiters;
			}
		}

		signaler = __igt_semaphore_token_alloc_rq(i915, engine_signal,
							  token_signal);
		if (IS_ERR(signaler)) {
			err = PTR_ERR(signaler);
			goto out_waiters;
		}
		i915_request_get(signaler);

		err = __igt_emit_signal(signaler, vma,
					semaphore_offset * sizeof(u32),
					IGT_SEM_TOKEN_VALUE);

		i915_request_add(signaler);

		if (err)
			goto out_signaler;

		if (i915_request_wait(signaler, 0, IGT_SIGNAL_DONE_TIMEOUT) <
		    0) {
			drm_err(&i915->drm, "Wait for signaller %s timed out! \n",
				engine_signal->name);
			err = -EIO;
			goto out_signaler;
		}

		for (i = 0; i < w; i++) {
			if (i915_request_wait(waiters[i], 0,
					      IGT_WAIT_DONE_TIMEOUT) < 0) {
				drm_err(&i915->drm, "Wait for waiter %s signalled by %s timed out!\n",
					waiters[i]->engine->name,
					signaler->engine->name);
				err = -EIO;
				goto out_signaler;
			}
		}

out_signaler:
		i915_request_put(signaler);
out_waiters:
		for (i = 0; i < w; i++)
			i915_request_put(waiters[i]);

		if (err)
			goto out_vma;

		semaphore_offset++;
	}

out_vma:
	i915_vma_unpin(vma);
out_obj:
	i915_gem_object_put(obj);
out:
	return err;
}

int intel_semaphore_live_selftests(struct drm_i915_private *i915)
{
	static const struct i915_subtest tests[] = {
		SUBTEST(igt_semaphore_token),
	};
	struct intel_gt *gt;
	unsigned int i;
	int ret = 0;

	if (GRAPHICS_VER(i915) < 12)
		return 0;

	/*
	 * Currently, we're not resubmitting contexts that context-switched-out
	 * on "signaling-mode" semaphore in i915 (execlists submission mode).
	 * When we're using GuC submission, GuC handles that for us. Since this
	 * test is just supposed to check the HW, let's skip the execlists
	 * submission mode to avoid false-negatives.
	 */
	if (!intel_uc_uses_guc_submission(&to_gt(i915)->uc)) {
		drm_info(&i915->drm, "i915 is not using GuC submission, skipping\n");
		return 0;
	}

	for_each_gt(gt, i915, i) {
		if (intel_gt_is_wedged(gt))
			continue;

		ret = intel_gt_live_subtests(tests, gt);
		if (ret)
			break;
	}

	return ret;
}
