// SPDX-License-Identifier: MIT
/*
 * Copyright © 2021 Intel Corporation
 */

#include "gt/intel_lrc_reg.h"
#include "gen6_ppgtt.h"
#include "i915_drv.h"
#include "intel_context.h"
#include "intel_engine_pm.h"
#include "intel_gpu_commands.h"
#include "intel_lrc.h"
#include "intel_ring.h"

static int gen8_emit_vm_config(struct i915_request *rq,
			       struct intel_context *ce,
			       const struct i915_ppgtt *ppgtt)
{
	u64 offset = i915_ggtt_offset(ce->state) +
		LRC_STATE_OFFSET + CTX_PDP0_LDW * 4;

	if (i915_vm_lvl(&ppgtt->vm) >= 4) {
		const u64 addr = px_dma(ppgtt->pd);
		u32 *cs;

		cs = intel_ring_begin(rq, 8);
		if (IS_ERR(cs))
			return PTR_ERR(cs);

		*cs++ = MI_STORE_DWORD_IMM_GEN4 | MI_USE_GGTT;
		*cs++ = lower_32_bits(offset);
		*cs++ = upper_32_bits(offset);
		*cs++ = lower_32_bits(addr);
		offset -= 8;

		*cs++ = MI_STORE_DWORD_IMM_GEN4 | MI_USE_GGTT;
		*cs++ = lower_32_bits(offset);
		*cs++ = upper_32_bits(offset);
		*cs++ = upper_32_bits(addr);
		offset -= 8;

		intel_ring_advance(rq, cs);
	} else {
		u32 *cs;
		int n;

		cs = intel_ring_begin(rq, 32);
		if (IS_ERR(cs))
			return PTR_ERR(cs);

		for (n = 0; n < 4; n++) {
			const u64 addr = i915_page_dir_dma_addr(ppgtt, n);

			*cs++ = MI_STORE_DWORD_IMM_GEN4 | MI_USE_GGTT;
			*cs++ = lower_32_bits(offset);
			*cs++ = upper_32_bits(offset);
			*cs++ = lower_32_bits(addr);
			offset -= 8;

			*cs++ = MI_STORE_DWORD_IMM_GEN4 | MI_USE_GGTT;
			*cs++ = lower_32_bits(offset);
			*cs++ = upper_32_bits(offset);
			*cs++ = upper_32_bits(addr);
			offset -= 8;
		}

		intel_ring_advance(rq, cs);
	}

	return 0;
}

struct ppgtt_barrier {
	struct dma_fence_cb base;
	struct i915_address_space *old;
};

static void set_ppgtt_barrier(struct dma_fence *f, struct dma_fence_cb *base)
{
	struct ppgtt_barrier *cb = container_of(base, typeof(*cb), base);

	gen6_ppgtt_unpin_all(i915_vm_to_ppgtt(cb->old));
	i915_vm_put(cb->old);

	kfree(cb);
}

static int pin_ppgtt_barrier(struct i915_request *rq,
			     struct intel_context *ce,
			     struct i915_address_space *vm)
{
       struct ppgtt_barrier *cb;
       int err;

       if (HAS_LOGICAL_RING_CONTEXTS(vm->i915))
	       return 0;

       cb = kmalloc(sizeof(*cb), GFP_KERNEL);
       if (!cb)
	       return -ENOMEM;

       /* ppGTT is not part of the legacy context image */
       err = gen6_ppgtt_pin(i915_vm_to_ppgtt(vm), NULL);
       if (err) {
	       kfree(cb);
	       return err;
       }

       cb->base.func = set_ppgtt_barrier;
       cb->old = i915_vm_get(ce->vm);

       spin_lock_irq(&rq->sched.lock);
       list_add_tail(&cb->base.node, &rq->fence.cb_list);
       spin_unlock_irq(&rq->sched.lock);

       return 0;
}

static int
gen8_modify_vm(struct intel_context *ce, struct i915_address_space *vm)
{
	struct i915_request *rq;
	int err;

	lockdep_assert_held(&ce->pin_mutex);

	/*
	 * If the context is not idle, we have to submit an ordered request to
	 * modify its context image via the kernel context (writing to our own
	 * image, or into the registers directory, does not stick). Pristine
	 * and idle contexts will be configured on pinning.
	 */
	if (!intel_context_pin_if_active(ce))
		return 0;

	rq = intel_engine_create_kernel_request(ce->engine);
	if (IS_ERR(rq)) {
		err = PTR_ERR(rq);
		goto out_unpin;
	}

	err = pin_ppgtt_barrier(rq, ce, vm);
	if (err)
		goto out_request;

	/* Serialise with the remote context */
	err = intel_context_prepare_remote_request(ce, rq);
	if (err == 0 && HAS_LOGICAL_RING_CONTEXTS(vm->i915))
		err = gen8_emit_vm_config(rq, ce, i915_vm_to_ppgtt(vm));

out_request:
	i915_request_add(rq);
out_unpin:
	intel_context_unpin(ce);
	return err;
}

int
intel_context_reconfigure_vm(struct intel_context *ce,
			     struct i915_address_space *vm)
{
	int err;

	GEM_BUG_ON(!ce->vm);
	GEM_BUG_ON(i915_is_ggtt(vm));

	err = intel_context_lock_pinned(ce);
	if (err)
		return err;

	if (ce->vm != vm) {
		err = gen8_modify_vm(ce, vm);
		if (!err) {
			i915_vm_put(ce->vm);
			ce->vm = i915_vm_get(vm);
		}
	}

	intel_context_unlock_pinned(ce);
	return err;
}
