/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2011-2012 Intel Corporation
 */

/*
 * This file implements HW context support. On gen5+ a HW context consists of an
 * opaque GPU object which is referenced at times of context saves and restores.
 * With RC6 enabled, the context is also referenced as the GPU enters and exists
 * from RC6 (GPU has it's own internal power context, except on gen5). Though
 * something like a context does exist for the media ring, the code only
 * supports contexts for the render ring.
 *
 * In software, there is a distinction between contexts created by the user,
 * and the default HW context. The default HW context is used by GPU clients
 * that do not request setup of their own hardware context. The default
 * context's state is never restored to help prevent programming errors. This
 * would happen if a client ran and piggy-backed off another clients GPU state.
 * The default context only exists to give the GPU some offset to load as the
 * current to invoke a save of the context we actually care about. In fact, the
 * code could likely be constructed, albeit in a more complicated fashion, to
 * never use the default context, though that limits the driver's ability to
 * swap out, and/or destroy other contexts.
 *
 * All other contexts are created as a request by the GPU client. These contexts
 * store GPU state, and thus allow GPU clients to not re-emit state (and
 * potentially query certain state) at any time. The kernel driver makes
 * certain that the appropriate commands are inserted.
 *
 * The context life cycle is semi-complicated in that context BOs may live
 * longer than the context itself because of the way the hardware, and object
 * tracking works. Below is a very crude representation of the state machine
 * describing the context life.
 *                                         refcount     pincount     active
 * S0: initial state                          0            0           0
 * S1: context created                        1            0           0
 * S2: context is currently running           2            1           X
 * S3: GPU referenced, but not current        2            0           1
 * S4: context is current, but destroyed      1            1           0
 * S5: like S3, but destroyed                 1            0           1
 *
 * The most common (but not all) transitions:
 * S0->S1: client creates a context
 * S1->S2: client submits execbuf with context
 * S2->S3: other clients submits execbuf with context
 * S3->S1: context object was retired
 * S3->S2: clients submits another execbuf
 * S2->S4: context destroy called with current context
 * S3->S5->S0: destroy path
 * S4->S5->S0: destroy path on current context
 *
 * There are two confusing terms used above:
 *  The "current context" means the context which is currently running on the
 *  GPU. The GPU has loaded its state already and has stored away the gtt
 *  offset of the BO. The GPU is not actively referencing the data at this
 *  offset, but it will on the next context switch. The only way to avoid this
 *  is to do a GPU reset.
 *
 *  An "active context' is one which was previously the "current context" and is
 *  on the active list waiting for the next context switch to occur. Until this
 *  happens, the object must remain at the same gtt offset. It is therefore
 *  possible to destroy a context, but it is still active.
 *
 */

#include <linux/highmem.h>
#include <linux/log2.h>
#include <linux/nospec.h>

#include <drm/drm_syncobj.h>
#include <drm/drm_cache.h>

#include "gt/intel_clos.h"
#include "gt/intel_context.h"
#include "gt/intel_context_param.h"
#include "gt/intel_engine_heartbeat.h"
#include "gt/intel_engine_pm.h"
#include "gt/intel_engine_regs.h"
#include "gt/intel_engine_user.h"
#include "gt/intel_gpu_commands.h"
#include "gt/intel_gt.h"
#include "gt/intel_ring.h"

#include "pxp/intel_pxp.h"

#include "i915_drm_client.h"
#include "i915_gem_context.h"
#include "i915_gem_ioctls.h"
#include "i915_svm.h"
#include "i915_trace.h"
#include "i915_user_extensions.h"
#include "i915_gem_ioctls.h"
#include "i915_debugger.h"
#include "i915_addr_trans_svc.h"

#define ALL_L3_SLICES(dev) (1 << NUM_L3_SLICES(dev)) - 1

static struct kmem_cache *slab_luts;

struct i915_lut_handle *i915_lut_handle_alloc(void)
{
	return kmem_cache_alloc(slab_luts, GFP_KERNEL);
}

void i915_lut_handle_free(struct i915_lut_handle *lut)
{
	return kmem_cache_free(slab_luts, lut);
}

static void lut_close(struct i915_gem_context *ctx)
{
	struct radix_tree_iter iter;
	void __rcu **slot;

	mutex_lock(&ctx->lut_mutex);
	rcu_read_lock();
	radix_tree_for_each_slot(slot, &ctx->handles_vma, &iter, 0) {
		struct i915_vma *vma = rcu_dereference_raw(*slot);
		struct drm_i915_gem_object *obj = vma->obj;
		struct i915_lut_handle *lut;

		if (!kref_get_unless_zero(&obj->base.refcount))
			continue;

		spin_lock(&obj->lut_lock);
		list_for_each_entry(lut, &obj->lut_list, obj_link) {
			if (lut->ctx != ctx)
				continue;

			if (lut->handle != iter.index)
				continue;

			list_del(&lut->obj_link);
			break;
		}
		spin_unlock(&obj->lut_lock);

		if (&lut->obj_link != &obj->lut_list) {
			i915_lut_handle_free(lut);
			radix_tree_iter_delete(&ctx->handles_vma, &iter, slot);
			i915_vma_close(vma);
			i915_gem_object_put(obj);
		}

		i915_gem_object_put(obj);
	}
	rcu_read_unlock();
	mutex_unlock(&ctx->lut_mutex);
}

static struct intel_context *
lookup_user_engine(struct i915_gem_context *ctx,
		   unsigned long flags,
		   const struct i915_engine_class_instance *ci)
#define LOOKUP_USER_INDEX BIT(0)
{
	int idx;

	if (!!(flags & LOOKUP_USER_INDEX) != i915_gem_context_user_engines(ctx))
		return ERR_PTR(-EINVAL);

	if (!i915_gem_context_user_engines(ctx)) {
		struct intel_engine_cs *engine;

		engine = intel_engine_lookup_user(ctx->i915,
						  ci->engine_class,
						  ci->engine_instance);
		if (!engine)
			return ERR_PTR(-EINVAL);

		idx = engine->legacy_idx;
	} else {
		idx = ci->engine_instance;
	}

	return i915_gem_context_get_engine(ctx, idx);
}

static struct i915_address_space *
context_get_vm_rcu(struct i915_gem_context *ctx)
{
	GEM_BUG_ON(!rcu_access_pointer(ctx->vm));

	do {
		struct i915_address_space *vm;

		/*
		 * We do not allow downgrading from full-ppgtt [to a shared
		 * global gtt], so ctx->vm cannot become NULL.
		 */
		vm = rcu_dereference(ctx->vm);
		if (!kref_get_unless_zero(&vm->ref))
			continue;

		/*
		 * This ppgtt may have be reallocated between
		 * the read and the kref, and reassigned to a third
		 * context. In order to avoid inadvertent sharing
		 * of this ppgtt with that third context (and not
		 * src), we have to confirm that we have the same
		 * ppgtt after passing through the strong memory
		 * barrier implied by a successful
		 * kref_get_unless_zero().
		 *
		 * Once we have acquired the current ppgtt of ctx,
		 * we no longer care if it is released from ctx, as
		 * it cannot be reallocated elsewhere.
		 */

		if (vm == rcu_access_pointer(ctx->vm))
			return rcu_pointer_handoff(vm);

		i915_vm_put(vm);
	} while (1);
}

static void intel_context_set_gem(struct intel_context *ce,
				  struct i915_gem_context *ctx)
{
	GEM_BUG_ON(rcu_access_pointer(ce->gem_context));
	RCU_INIT_POINTER(ce->gem_context, ctx);

	ce->ring_size = SZ_16K;
	if (ce->engine->class == COMPUTE_CLASS)
		ce->ring_size = SZ_512K;

	if (rcu_access_pointer(ctx->vm)) {
		struct i915_address_space *vm;

		rcu_read_lock();
		vm = context_get_vm_rcu(ctx); /* hmm */
		rcu_read_unlock();

		i915_vm_put(ce->vm);
		ce->vm = vm;
	}

	GEM_BUG_ON(ce->client);
	if (ctx->client)
		ce->client = i915_drm_client_get(ctx->client);

	if (ctx->sched.priority >= I915_PRIORITY_NORMAL &&
	    intel_engine_has_timeslices(ce->engine) &&
	    intel_engine_has_semaphores(ce->engine))
		__set_bit(CONTEXT_USE_SEMAPHORES, &ce->flags);

	if (CONFIG_DRM_I915_REQUEST_TIMEOUT &&
	    ctx->i915->params.request_timeout_ms) {
		unsigned int timeout_ms = ctx->i915->params.request_timeout_ms;

		intel_context_set_watchdog_us(ce, (u64)timeout_ms * 1000);
	}

	if (i915_gem_context_has_sip(ctx))
		__set_bit(CONTEXT_DEBUG, &ce->flags);

	if (test_bit(UCONTEXT_RUNALONE, &ctx->user_flags))
		__set_bit(CONTEXT_RUNALONE, &ce->flags);
}

static void __unpin_engines(struct i915_gem_engines *e, unsigned int count)
{
	while (count--) {
		struct intel_context *ce = e->engines[count], *child;

		if (!ce || !test_bit(CONTEXT_PERMA_PIN, &ce->flags))
			continue;

		for_each_child(ce, child)
			intel_context_unpin(child);
		intel_context_unpin(ce);
	}
}

static void unpin_engines(struct i915_gem_engines *e)
{
	__unpin_engines(e, e->num_engines);
}

static void __free_engines(struct i915_gem_engines *e, unsigned int count)
{
	while (count--) {
		if (!e->engines[count])
			continue;

		intel_context_put(e->engines[count]);
	}
	kfree(e);
}

static void free_engines(struct i915_gem_engines *e)
{
	__free_engines(e, e->num_engines);
}

static void free_engines_rcu(struct rcu_head *rcu)
{
	struct i915_gem_engines *engines =
		container_of(rcu, struct i915_gem_engines, rcu);

	i915_sw_fence_fini(&engines->fence);
	free_engines(engines);
}

static void accumulate_runtime(struct i915_drm_client *client,
			       struct i915_gem_engines *engines)
{
	struct i915_gem_engines_iter it;
	struct intel_context *ce;

	if (!client)
		return;

	/* Transfer accumulated runtime to the parent GEM context. */
	for_each_gem_engine(ce, engines, it) {
		unsigned int class = ce->engine->uabi_class;

		GEM_BUG_ON(class >= ARRAY_SIZE(client->past_runtime));
		atomic64_add(intel_context_get_total_runtime_ns(ce),
			     &client->past_runtime[class]);
	}
}

static int
engines_notify(struct i915_sw_fence *fence, enum i915_sw_fence_notify state)
{
	struct i915_gem_engines *engines =
		container_of(fence, typeof(*engines), fence);
	struct i915_gem_context *ctx = engines->ctx;

	switch (state) {
	case FENCE_COMPLETE:
		if (!list_empty(&engines->link)) {
			unsigned long flags;

			spin_lock_irqsave(&ctx->stale.lock, flags);
			list_del(&engines->link);
			spin_unlock_irqrestore(&ctx->stale.lock, flags);
		}
		accumulate_runtime(ctx->client, engines);
		i915_gem_context_put(ctx);
		break;

	case FENCE_FREE:
		init_rcu_head(&engines->rcu);
		call_rcu(&engines->rcu, free_engines_rcu);
		break;
	}

	return NOTIFY_DONE;
}

static struct i915_gem_engines *alloc_engines(unsigned int count)
{
	struct i915_gem_engines *e;

	e = kzalloc(struct_size(e, engines, count), GFP_KERNEL);
	if (!e)
		return NULL;

	i915_sw_fence_init(&e->fence, engines_notify);
	return e;
}

static struct i915_gem_engines *default_engines(struct i915_gem_context *ctx)
{
	const unsigned int max = I915_MAX_GT * I915_NUM_ENGINES;
	struct intel_engine_cs *engine;
	struct i915_gem_engines *e;

	e = alloc_engines(max);
	if (!e)
		return ERR_PTR(-ENOMEM);

	for_each_uabi_engine(engine, ctx->i915) {
		struct intel_context *ce;

		if (engine->legacy_idx == INVALID_ENGINE)
			continue;

		GEM_BUG_ON(engine->legacy_idx >= max);
		GEM_BUG_ON(e->engines[engine->legacy_idx]);

		ce = intel_context_create(engine);
		if (IS_ERR(ce)) {
			__free_engines(e, e->num_engines + 1);
			return ERR_CAST(ce);
		}

		intel_context_set_gem(ce, ctx);

		e->engines[engine->legacy_idx] = ce;
		e->num_engines = max(e->num_engines, engine->legacy_idx);
	}
	e->num_engines++;

	return e;
}

static intel_engine_mask_t engine_mask(struct i915_gem_engines *engines)
{
	struct i915_gem_engines_iter it;
	intel_engine_mask_t mask = 0;
	struct intel_context *ce;

	for_each_gem_engine(ce, engines, it)
		mask |= ce->engine->mask;

	return mask;
}

void i915_gem_context_release(struct kref *ref)
{
	struct i915_gem_context *ctx = container_of(ref, typeof(*ctx), ref);
	unsigned long flags;

	trace_i915_context_free(ctx);
	GEM_BUG_ON(!i915_gem_context_is_closed(ctx));

	spin_lock_irqsave(&ctx->i915->gem.contexts.lock, flags);
	list_del_rcu(&ctx->link);
	spin_unlock_irqrestore(&ctx->i915->gem.contexts.lock, flags);

	if (ctx->pxp_wakeref)
		intel_runtime_pm_put(&ctx->i915->runtime_pm, ctx->pxp_wakeref);

	if (ctx->client)
		i915_drm_client_put(ctx->client);

	mutex_destroy(&ctx->engines_mutex);
	mutex_destroy(&ctx->lut_mutex);
	mutex_destroy(&ctx->mutex);

	kfree_rcu(ctx, rcu);
}

static inline struct i915_gem_engines *
__context_engines_static(const struct i915_gem_context *ctx)
{
	return rcu_dereference_protected(ctx->engines, true);
}

static void __reset_context(struct i915_gem_context *ctx,
			    struct intel_engine_cs *engine)
{
	intel_gt_handle_error(engine->gt, engine->mask, 0,
			      "context closure in %s", ctx->name);
}

static bool __cancel_engine(struct intel_context *ce, struct intel_engine_cs *engine)
{
	/*
	 * If the debugger is active, we can't trust our engine reset mechanism to do
	 * a full GT reset required to clear residual ATTN.
	 */
	if (i915_debugger_active_on_context(ce))
		return false;

	/*
	 * Send a "high priority pulse" down the engine to cause the
	 * current request to be momentarily preempted. (If it fails to
	 * be preempted, it will be reset). As we have marked our context
	 * as banned, any incomplete request, including any running, will
	 * be skipped following the preemption.
	 *
	 * If there is no hangchecking (one of the reasons why we try to
	 * cancel the context) and no forced preemption, there may be no
	 * means by which we reset the GPU and evict the persistent hog.
	 * Ergo if we are unable to inject a preemptive pulse that can
	 * kill the banned context, we fallback to doing a local reset
	 * instead.
	 */
	return intel_engine_pulse(engine) == 0;
}

static struct intel_engine_cs *active_engine(struct intel_context *ce)
{
	struct intel_engine_cs *engine = NULL;
	struct i915_request *rq;

	if (intel_context_has_inflight(ce))
		return intel_context_inflight(ce);

	if (!ce->timeline)
		return NULL;

	/*
	 * rq->link is only SLAB_TYPESAFE_BY_RCU, we need to hold a reference
	 * to the request to prevent it being transferred to a new timeline
	 * (and onto a new timeline->requests list).
	 */
	rcu_read_lock();
	list_for_each_entry_reverse(rq, &ce->timeline->requests, link) {
		bool found;

		/* timeline is already completed upto this point? */
		if (!i915_request_get_rcu(rq))
			break;

		/* Check with the backend if the request is inflight */
		found = true;
		if (likely(rcu_access_pointer(rq->timeline) == ce->timeline))
			found = i915_request_active_engine(rq, &engine);

		i915_request_put(rq);
		if (found)
			break;
	}
	rcu_read_unlock();

	return engine;
}

static void kill_engines(struct i915_gem_engines *engines, bool ban)
{
	struct i915_gem_engines_iter it;
	struct intel_context *ce;

	/*
	 * Map the user's engine back to the actual engines; one virtual
	 * engine will be mapped to multiple engines, and using ctx->engine[]
	 * the same engine may be have multiple instances in the user's map.
	 * However, we only care about pending requests, so only include
	 * engines on which there are incomplete requests.
	 */
	for_each_gem_engine(ce, engines, it) {
		struct intel_engine_cs *engine;

		if (ban && intel_context_ban(ce, NULL))
			continue;

		/*
		 * Check the current active state of this context; if we
		 * are currently executing on the GPU we need to evict
		 * ourselves. On the other hand, if we haven't yet been
		 * submitted to the GPU or if everything is complete,
		 * we have nothing to do.
		 */
		engine = active_engine(ce);

		/* First attempt to gracefully cancel the context */
		if (engine && !__cancel_engine(ce, engine) && ban)
			/*
			 * If we are unable to send a preemptive pulse to bump
			 * the context from the GPU, we have to resort to a full
			 * reset. We hope the collateral damage is worth it.
			 */
			__reset_context(engines->ctx, engine);
	}
}

static void kill_context(struct i915_gem_context *ctx)
{
	bool ban = (!i915_gem_context_is_persistent(ctx) ||
		    !ctx->i915->params.enable_hangcheck);
	struct i915_gem_engines *pos, *next;

	spin_lock_irq(&ctx->stale.lock);
	GEM_BUG_ON(!i915_gem_context_is_closed(ctx));
	list_for_each_entry_safe(pos, next, &ctx->stale.engines, link) {
		if (!i915_sw_fence_await(&pos->fence)) {
			list_del_init(&pos->link);
			continue;
		}

		spin_unlock_irq(&ctx->stale.lock);

		kill_engines(pos, ban);

		spin_lock_irq(&ctx->stale.lock);
		GEM_BUG_ON(i915_sw_fence_signaled(&pos->fence));
		list_safe_reset_next(pos, next, link);
		list_del_init(&pos->link); /* decouple from FENCE_COMPLETE */

		i915_gem_context_engines_put(pos);
	}
	spin_unlock_irq(&ctx->stale.lock);
}

static void engines_idle_release(struct i915_gem_context *ctx,
				 struct i915_gem_engines *engines)
{
	struct i915_gem_engines_iter it;
	struct intel_context *ce;

	INIT_LIST_HEAD(&engines->link);

	engines->ctx = i915_gem_context_get(ctx);

	for_each_gem_engine(ce, engines, it) {
		int err;

		/* serialises with execbuf */
		intel_context_close(ce);
		if (!intel_context_pin_if_active(ce))
			continue;

		/* Wait until context is finally scheduled out and retired */
		err = i915_sw_fence_await_active(&engines->fence,
						 &ce->active,
						 I915_ACTIVE_AWAIT_BARRIER);
		intel_context_unpin(ce);
		if (err)
			goto kill;
	}

	spin_lock_irq(&ctx->stale.lock);
	if (!i915_gem_context_is_closed(ctx))
		list_add_tail(&engines->link, &ctx->stale.engines);
	spin_unlock_irq(&ctx->stale.lock);

kill:
	if (list_empty(&engines->link)) /* raced, already closed */
		kill_engines(engines, true);

	i915_sw_fence_commit(&engines->fence);
}

static void set_closed_name(struct i915_gem_context *ctx)
{
	char *s;

	/* Replace '[]' with '<>' to indicate closed in debug prints */

	s = strrchr(ctx->name, '[');
	if (!s)
		return;

	*s = '<';

	s = strchr(s + 1, ']');
	if (s)
		*s = '>';
}

static void context_close(struct i915_gem_context *ctx)
{
	struct i915_address_space *vm;
	struct i915_drm_client *client;

	client = ctx->client;
	if (client)
		i915_debugger_wait_on_discovery(ctx->i915, client);

	/* Flush any concurrent set_engines() */
	mutex_lock(&ctx->engines_mutex);
	unpin_engines(__context_engines_static(ctx));
	engines_idle_release(ctx, rcu_replace_pointer(ctx->engines, NULL, 1));
	i915_gem_context_set_closed(ctx);
	mutex_unlock(&ctx->engines_mutex);

	if (client)
		i915_debugger_context_destroy(ctx);

	mutex_lock(&ctx->mutex);

	set_closed_name(ctx);

	/*
	 * The LUT uses the VMA as a backpointer to unref the object,
	 * so we need to clear the LUT before we close all the VMA (inside
	 * the ppgtt).
	 */
	lut_close(ctx);

	vm = i915_gem_context_vm(ctx);

	if (ctx->syncobj)
		drm_syncobj_put(ctx->syncobj);

	ctx->file_priv = ERR_PTR(-EBADF);

	if (client) {
		spin_lock(&client->ctx_lock);
		list_del_rcu(&ctx->client_link);
		spin_unlock(&client->ctx_lock);
	}

	mutex_unlock(&ctx->mutex);

	if (vm) {
		if (client)
			i915_debugger_vm_destroy(client, vm);
		i915_vm_close(vm);
	}

	/* WA for VLK-20104 */
	if (ctx->bcs0_pm_disabled) {
		struct intel_gt *gt;
		unsigned int i;

		for_each_gt(gt, ctx->i915, i)
			intel_engine_pm_put(gt->engine[gt->rsvd_bcs]);
	}

	/*
	 * If the user has disabled hangchecking, we can not be sure that
	 * the batches will ever complete after the context is closed,
	 * keeping the context and all resources pinned forever. So in this
	 * case we opt to forcibly kill off all remaining requests on
	 * context close.
	 */
	kill_context(ctx);

	wake_up_all(&ctx->user_fence_wq);
	i915_gem_context_put(ctx);
}

static void warn_non_persistent_usage(struct drm_i915_private *i915)
{
	static bool once;

	if (xchg(&once, true))
		return;

	drm_notice(&i915->drm, "*****************************************************************\n");
	drm_notice(&i915->drm, "* WARNING: Non-persistent context with no engine reset detected!*\n");
	drm_notice(&i915->drm, "*                                                               *\n");
	drm_notice(&i915->drm, "* This usage is enabled only for debug purposes and is unsafe   *\n");
	drm_notice(&i915->drm, "* for production use as it may result in forever spinning,      *\n");
	drm_notice(&i915->drm, "* potentially non-preemptible jobs left on the GPU.             *\n");
	drm_notice(&i915->drm, "* Further testing may be unreliable. You have been warned!      *\n");
	drm_notice(&i915->drm, "*****************************************************************\n");
}

static int __context_set_persistence(struct i915_gem_context *ctx, bool state)
{
	if (i915_gem_context_is_persistent(ctx) == state)
		return 0;

	if (state) {
		/* Long running contexts mandates non-persistence. */
		if (i915_gem_context_is_lr(ctx))
			return -EINVAL;

		/*
		 * Only contexts that are short-lived [that will expire or be
		 * reset] are allowed to survive past termination. We require
		 * hangcheck to ensure that the persistent requests are healthy.
		 */
		if (!ctx->i915->params.enable_hangcheck)
			return -EINVAL;

		i915_gem_context_set_persistence(ctx);
	} else {
		/* To cancel a context we use "preempt-to-idle" */
		if (!(ctx->i915->caps.scheduler & I915_SCHEDULER_CAP_PREEMPTION))
			return -ENODEV;

		/*
		 * If the cancel fails, we then need to reset, cleanly!
		 *
		 * If the per-engine reset fails, all hope is lost! We resort
		 * to a full GPU reset in that unlikely case, but realistically
		 * if the engine could not reset, the full reset does not fare
		 * much better. The damage has been done.
		 *
		 * However, if we cannot reset an engine by itself, we cannot
		 * cleanup a hanging persistent context without causing
		 * colateral damage, and we should not pretend we can by
		 * exposing the interface.
		 */
		if (!intel_has_reset_engine(to_gt(ctx->i915))) {
			/*
			 * It is useful to disable resets for debugging purposes
			 * and still be able to use non-persistent contexts.
			 */
			if (!ctx->i915->params.allow_non_persist_without_reset)
				return -ENODEV;

			warn_non_persistent_usage(ctx->i915);
		}

		i915_gem_context_clear_persistence(ctx);
	}

	return 0;
}

static int __context_set_protected(struct drm_i915_private *i915,
				   struct i915_gem_context *ctx,
				   bool protected)
{
	int ret = 0;

	if (ctx->file_priv) {
		/* can't change this after creation! */
		ret = -EINVAL;
	} else if (!protected) {
		ctx->uses_protected_content = false;
	} else if (!intel_pxp_is_enabled(&i915->gt0.pxp)) {
		ret = -ENODEV;
	} else if ((i915_gem_context_is_recoverable(ctx)) ||
		   !(i915_gem_context_is_bannable(ctx))) {
		ret = -EPERM;
	} else {
		ctx->uses_protected_content = true;

		/*
		 * protected context usage requires the PXP session to be up,
		 * which in turn requires the device to be active.
		 */
		ctx->pxp_wakeref = intel_runtime_pm_get(&i915->runtime_pm);

		if (!intel_pxp_is_active(&i915->gt0.pxp))
			ret = intel_pxp_start(&i915->gt0.pxp);
	}

	return ret;
}

/*
 * FIXME: We want this to be more random.
 */
static u32 __contexts_get_next_token(struct intel_gt *gt)
{
	struct drm_i915_private *i915 = gt->i915;
	u32 token;

	GEM_BUG_ON(GRAPHICS_VER(i915) < 12);

	token = atomic_inc_return(&gt->next_token);
	if (HAS_SEMAPHORE_XEHPSDV(i915))
		token %= XEHPSDV_ENGINE_SEMAPHORE_TOKEN_MAX;
	else
		token %= GEN12_ENGINE_SEMAPHORE_TOKEN_MAX;

	return token;
}

static struct i915_gem_context *__create_context(struct intel_gt *gt)
{
	struct drm_i915_private *i915 = gt->i915;
	struct i915_gem_context *ctx;
	struct i915_gem_engines *e;
	int err;
	int i;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return ERR_PTR(-ENOMEM);

	kref_init(&ctx->ref);
	ctx->i915 = i915;
	ctx->sched.priority = I915_PRIORITY_NORMAL;
	ctx->acc_granularity = 0;
	ctx->acc_notify = 0;
	ctx->acc_trigger = 0;

	mutex_init(&ctx->mutex);
	INIT_LIST_HEAD(&ctx->link);

	spin_lock_init(&ctx->stale.lock);
	INIT_LIST_HEAD(&ctx->stale.engines);

	init_waitqueue_head(&ctx->user_fence_wq);

	mutex_init(&ctx->engines_mutex);
	e = default_engines(ctx);
	if (IS_ERR(e)) {
		err = PTR_ERR(e);
		goto err_free;
	}
	ctx->engine_mask = engine_mask(e);
	RCU_INIT_POINTER(ctx->engines, e);

	INIT_RADIX_TREE(&ctx->handles_vma, GFP_KERNEL);
	mutex_init(&ctx->lut_mutex);

	/* NB: Mark all slices as needing a remap so that when the context first
	 * loads it will restore whatever remap state already exists. If there
	 * is no remap info, it will be a NOP. */
	ctx->remap_slice = ALL_L3_SLICES(i915);

	i915_gem_context_set_bannable(ctx);
	i915_gem_context_set_recoverable(ctx);
	__context_set_persistence(ctx, true /* cgroup hook? */);

	for (i = 0; i < ARRAY_SIZE(ctx->hang_timestamp); i++)
		ctx->hang_timestamp[i] = jiffies - CONTEXT_FAST_HANG_JIFFIES;

	if (GRAPHICS_VER(i915) >= 12)
		ctx->semaphore_token = __contexts_get_next_token(gt);

	return ctx;

err_free:
	kfree(ctx);
	return ERR_PTR(err);
}

void i915_gem_context_engines_put(struct i915_gem_engines *e)
{
	i915_sw_fence_complete(&e->fence);
}

struct i915_gem_engines *
i915_gem_context_engines_get(struct i915_gem_context *ctx,
			     bool *user_engines)
{
	struct i915_gem_engines *engines;

	rcu_read_lock();
	do {
		engines = rcu_dereference(ctx->engines);
		if (!engines)
			break;

		if (user_engines)
			*user_engines = i915_gem_context_user_engines(ctx);

		/* successful await => strong mb */
		if (unlikely(!i915_sw_fence_await(&engines->fence)))
			continue;

		if (likely(engines == rcu_access_pointer(ctx->engines)))
			break;

		i915_gem_context_engines_put(engines);
	} while (1);
	rcu_read_unlock();

	return engines;
}

static void
context_apply_all(struct i915_gem_context *ctx,
		  void (*fn)(struct intel_context *ce, void *data),
		  void *data)
{
	struct i915_gem_engines_iter it;
	struct i915_gem_engines *e;
	struct intel_context *ce;

	e = i915_gem_context_engines_get(ctx, NULL);
	for_each_gem_engine(ce, e, it)
		fn(ce, data);
	i915_gem_context_engines_put(e);
}

static void __apply_ppgtt(struct intel_context *ce, void *vm)
{
	intel_context_reconfigure_vm(ce, vm);
}

static void __apply_client(struct intel_context *ce, void *client)
{
	ce->client = i915_drm_client_get(client);
}

static struct i915_address_space *
__set_ppgtt(struct i915_gem_context *ctx, struct i915_address_space *vm)
{
	struct i915_address_space *old;

	if (!i915_vm_tryopen(vm))
		return NULL;

	old = rcu_replace_pointer(ctx->vm, vm,
				  lockdep_is_held(&ctx->mutex));
	GEM_BUG_ON(old && i915_vm_lvl(vm) != i915_vm_lvl(old));

	return old;
}

static void __assign_ppgtt(struct i915_gem_context *ctx,
			   struct i915_address_space *vm)
{
	if (vm == rcu_access_pointer(ctx->vm))
		return;

	context_apply_all(ctx, __apply_ppgtt, vm);

	vm = __set_ppgtt(ctx, vm);
	if (vm) {
		i915_debugger_vm_destroy(ctx->client, vm);
		i915_vm_close(vm);
	}
}

static struct i915_gem_context *
i915_gem_context_create_for_gt(struct intel_gt *gt, unsigned int flags)
{
	struct drm_i915_private *i915 = gt->i915;
	struct i915_gem_context *ctx;
	int ret;

	if (flags & I915_CONTEXT_CREATE_FLAGS_SINGLE_TIMELINE &&
	    !HAS_EXECLISTS(i915))
		return ERR_PTR(-EINVAL);

	if ((flags & PRELIM_I915_CONTEXT_CREATE_FLAGS_LONG_RUNNING) &&
	    (!(i915->caps.scheduler & I915_SCHEDULER_CAP_PREEMPTION) ||
	     !intel_uc_uses_guc_submission(&gt->uc)))
		return ERR_PTR(-ENODEV);

	ctx = __create_context(gt);
	if (IS_ERR(ctx))
		return ctx;

	if (HAS_FULL_PPGTT(i915)) {
		struct i915_ppgtt *ppgtt;
		u32 flags = 0;

		if (i915->params.enable_pagefault && HAS_RECOVERABLE_PAGE_FAULT(i915))
			flags |= PRELIM_I915_VM_CREATE_FLAGS_ENABLE_PAGE_FAULT;
		ppgtt = i915_ppgtt_create(gt, flags);
		if (IS_ERR(ppgtt)) {
			drm_dbg(&i915->drm, "PPGTT setup failed (%ld)\n",
				PTR_ERR(ppgtt));
			context_close(ctx);
			return ERR_CAST(ppgtt);
		}

		mutex_lock(&ctx->mutex);
		__assign_ppgtt(ctx, &ppgtt->vm);
		mutex_unlock(&ctx->mutex);

		/* Release reference taken during i915_ppgtt_create */
		i915_vm_close(&ppgtt->vm);
	}

	if (flags & I915_CONTEXT_CREATE_FLAGS_SINGLE_TIMELINE) {
		ret = drm_syncobj_create(&ctx->syncobj,
					 DRM_SYNCOBJ_CREATE_SIGNALED,
					 NULL);
		if (ret) {
			context_close(ctx);
			return ERR_PTR(ret);
		}
	}

	if (flags & PRELIM_I915_CONTEXT_CREATE_FLAGS_LONG_RUNNING) {
		int ret;

		/* Long running implies non-persistence. */
		ret = __context_set_persistence(ctx, false);
		if (ret) {
			context_close(ctx);
			return ERR_PTR(ret);
		}

		i915_gem_context_set_lr(ctx);

		/*
		 * WA for VLK-20104: Never park BCS0 of each tile if
		 * PCIE L4 WA is enabled.
		 * XXX: Optimize by disabling BCS0 PM only on required
		 * tile (check for tiles in ctx->engines?).
		 * XXX: On pvc, we are now using last instance of blitter
		 * engine and not BCS0. This workaround may be absorbed in to a
		 * general requirement that we need to hold fw whenever a client
		 * access lmem.
		 */
		if (i915->params.ulls_bcs0_pm_wa &&
		    i915_is_mem_wa_enabled(i915,
					   I915_WA_USE_FLAT_PPGTT_UPDATE)) {
			struct intel_gt *t;
			unsigned int i;

			drm_dbg(&i915->drm,
				"Disabling PM for reserved bcs on each tile for ULLS ctx\n");
			for_each_gt(t, ctx->i915, i)
				intel_engine_pm_get(t->engine[t->rsvd_bcs]);

			ctx->bcs0_pm_disabled = true;
		}
	}

	trace_i915_context_create(ctx);

	return ctx;
}

static void init_contexts(struct i915_gem_contexts *gc)
{
	spin_lock_init(&gc->lock);
	INIT_LIST_HEAD(&gc->list);
}

void i915_gem_init__contexts(struct drm_i915_private *i915)
{
	init_contexts(&i915->gem.contexts);
}

static void __assign_scheduling_policy(struct intel_context *context,
				       void *client)
{
	intel_context_init_schedule_policy(context);
}

static void __apply_debugger(struct intel_context *context, void *client)
{
	if (i915_debugger_context_guc_debugged(context))
		intel_context_disable_preemption_timeout(context);
}

static int gem_context_register(struct i915_gem_context *ctx,
				struct drm_i915_file_private *fpriv,
				u32 *id)
{
	struct drm_i915_private *i915 = ctx->i915;
	struct i915_drm_client *client;
	struct i915_address_space *vm;
	int ret;

	ctx->file_priv = fpriv;
	client = i915_drm_client_get(fpriv->client);

	mutex_lock(&ctx->mutex);
	vm = i915_gem_context_vm(ctx);
	if (vm && !vm->client)
		WRITE_ONCE(vm->client, i915_drm_client_get(client)); /* XXX */
	mutex_unlock(&ctx->mutex);

	rcu_read_lock();
	snprintf(ctx->name, sizeof(ctx->name), "%s[%d]",
		 i915_drm_client_name(client),
		 pid_nr(i915_drm_client_pid(client)));
	rcu_read_unlock();

	ctx->client = client;

	i915_debugger_wait_on_discovery(i915, client);

	spin_lock(&client->ctx_lock);
	list_add_tail_rcu(&ctx->client_link, &client->ctx_list);
	spin_unlock(&client->ctx_lock);

	context_apply_all(ctx, __apply_client, client);

	/* Set the default scheduling values from the engine */
	context_apply_all(ctx, __assign_scheduling_policy, client);
	/* Apply any debugger overrides to the context */
	context_apply_all(ctx, __apply_debugger, client);

	spin_lock_irq(&i915->gem.contexts.lock);
	list_add_tail_rcu(&ctx->link, &i915->gem.contexts.list);
	spin_unlock_irq(&i915->gem.contexts.lock);

	/* And finally expose ourselves to userspace via the idr */
	i915_gem_context_get(ctx);
	ret = xa_alloc(&fpriv->context_xa, id, ctx, xa_limit_32b, GFP_KERNEL);
	if (!ret)
		i915_debugger_context_create(ctx);

	i915_gem_context_put(ctx);
	if (!ret)
		return 0;

	spin_lock(&client->ctx_lock);
	list_del_rcu(&ctx->client_link);
	spin_unlock(&client->ctx_lock);

	spin_lock_irq(&i915->gem.contexts.lock);
	list_del_rcu(&ctx->link);
	spin_unlock_irq(&i915->gem.contexts.lock);

	i915_drm_client_put(client);
	return ret;
}

int i915_gem_context_open(struct drm_i915_private *i915,
			  struct drm_file *file)
{
	struct drm_i915_file_private *file_priv = file->driver_priv;
	struct i915_gem_context *ctx;
	int err;
	u32 id;

	xa_init_flags(&file_priv->context_xa, XA_FLAGS_ALLOC);

	/* 0 reserved for invalid/unassigned ppgtt */
	xa_init_flags(&file_priv->vm_xa, XA_FLAGS_ALLOC1);

	ctx = i915_gem_context_create_for_gt(to_gt(i915), 0);
	if (IS_ERR(ctx)) {
		err = PTR_ERR(ctx);
		goto err;
	}

	err = gem_context_register(ctx, file_priv, &id);
	if (err < 0)
		goto err_ctx;

	GEM_BUG_ON(id);
	return 0;

err_ctx:
	context_close(ctx);
err:
	xa_destroy(&file_priv->vm_xa);
	xa_destroy(&file_priv->context_xa);
	return err;
}

void i915_gem_context_close(struct drm_file *file)
{
	struct drm_i915_file_private *file_priv = file->driver_priv;
	struct i915_address_space *vm;
	struct i915_gem_context *ctx;
	unsigned long idx;

	xa_for_each(&file_priv->context_xa, idx, ctx)
		context_close(ctx);
	xa_destroy(&file_priv->context_xa);

	xa_for_each(&file_priv->vm_xa, idx, vm) {
		i915_debugger_vm_destroy(file_priv->client, vm);
		i915_vm_close(vm);
	}
	xa_destroy(&file_priv->vm_xa);
}

struct vm_create_ext {
	struct drm_i915_private *i915;
	struct intel_gt *gt;
};

static int get_vm_gt(struct i915_user_extension __user *ext, void *data)
{
	struct prelim_drm_i915_gem_vm_region_ext local;
	struct intel_memory_region *mem;
	struct vm_create_ext *vce = data;

	if (copy_from_user(&local, ext, sizeof(local)))
		return -EFAULT;

	if (local.pad)
		return -EINVAL;

	mem = intel_memory_region_lookup(vce->i915,
					 local.region.memory_class,
					 local.region.memory_instance);
	if (!mem || mem->type != INTEL_MEMORY_LOCAL)
		return -EINVAL;

	vce->gt = mem->gt;
	return 0;
}

static const i915_user_extension_fn vm_create_extensions[] = {
	[PRELIM_I915_USER_EXT_MASK(PRELIM_I915_GEM_VM_CONTROL_EXT_REGION)] = get_vm_gt,
};

int i915_gem_vm_create_ioctl(struct drm_device *dev, void *data,
			     struct drm_file *file)
{
	struct drm_i915_private *i915 = to_i915(dev);
	struct drm_i915_gem_vm_control *args = data;
	struct drm_i915_file_private *file_priv = file->driver_priv;
	struct vm_create_ext vce;
	struct i915_ppgtt *ppgtt;
	u32 id;
	int err;

	if (!HAS_FULL_PPGTT(i915))
		return -ENODEV;

	if (args->flags & PRELIM_I915_VM_CREATE_FLAGS_UNKNOWN)
		return -EINVAL;

	if ((args->flags & PRELIM_I915_VM_CREATE_FLAGS_ENABLE_PAGE_FAULT) &&
	    !HAS_RECOVERABLE_PAGE_FAULT(i915))
		return -EINVAL;

	if (i915->params.enable_pagefault && HAS_RECOVERABLE_PAGE_FAULT(i915))
		args->flags |= PRELIM_I915_VM_CREATE_FLAGS_ENABLE_PAGE_FAULT;

	if (args->extensions) {
		vce.i915 = i915;
		err = i915_user_extensions(u64_to_user_ptr(args->extensions),
					   vm_create_extensions,
					   ARRAY_SIZE(vm_create_extensions),
					   &vce);
		if (err)
			return err;
	} else {
		vce.gt = to_gt(i915);
	}

	ppgtt = i915_ppgtt_create(vce.gt, args->flags);
	if (IS_ERR(ppgtt))
		return PTR_ERR(ppgtt);

	ppgtt->vm.client = i915_drm_client_get(file_priv->client);

	i915_debugger_wait_on_discovery(i915, ppgtt->vm.client);

	err = xa_alloc(&file_priv->vm_xa, &id, &ppgtt->vm,
		       xa_limit_32b, GFP_KERNEL);
	if (err)
		goto err_put;

	if (args->flags & PRELIM_I915_VM_CREATE_FLAGS_USE_VM_BIND)
		ppgtt->vm.vm_bind_mode = true;

	GEM_BUG_ON(id == 0); /* reserved for invalid/unassigned ppgtt */
	args->vm_id = id;
	i915_debugger_vm_create(file_priv->client, &ppgtt->vm);

	return 0;

err_put:
	i915_vm_close(&ppgtt->vm);
	return err;
}

int i915_gem_vm_destroy_ioctl(struct drm_device *dev, void *data,
			      struct drm_file *file)
{
	struct drm_i915_file_private *file_priv = file->driver_priv;
	struct drm_i915_gem_vm_control *args = data;
	struct i915_address_space *vm;

	if (args->flags)
		return -EINVAL;

	if (args->extensions)
		return -EINVAL;

	i915_debugger_wait_on_discovery(to_i915(dev), file_priv->client);

	vm = xa_erase(&file_priv->vm_xa, args->vm_id);
	if (!vm)
		return -ENOENT;

	i915_debugger_vm_destroy(vm->client, vm);
	i915_vm_close(vm);

	i915_gem_flush_free_objects(to_i915(dev));
	flush_workqueue(to_i915(dev)->wq);
	return 0;
}

static int i915_gem_vm_setparam_ioctl(struct drm_device *dev, void *data,
				      struct drm_file *file)
{
	struct drm_i915_file_private *file_priv = file->driver_priv;
	struct prelim_drm_i915_gem_vm_param *args = data;
	struct i915_address_space *vm;
	int err = 0;

	vm = i915_address_space_lookup(file_priv, args->vm_id);
	if (unlikely(!vm))
		return -ENOENT;

	switch (lower_32_bits(args->param)) {
	case PRELIM_I915_GEM_VM_PARAM_SVM:
		/* FIXME: Ensure ppgtt is empty before switching */
		if (!i915_has_svm(file_priv->dev_priv) || IS_SRIOV_VF(file_priv->dev_priv)) {
			err = -ENOTSUPP;
			break;
		}

		/* If ATS is enabled, create PASID for the process */
		if (args->value && i915_ats_enabled(vm->i915) &&
		    i915_vm_page_fault_enabled(vm)) {
			if (i915_create_pasid(vm))
				drm_err(&vm->i915->drm,
					"Failed to create PASID for ATS operations\n");
		} else if (is_vm_pasid_active(vm)) {
			/* Destroy any associated pasid to vm */
			i915_destroy_pasid(vm);
		}

		if (args->value) {
			err = intel_memory_regions_add_svm(file_priv->dev_priv);
			if (!err)
				err = i915_svm_bind_mm(vm);
		} else {
			i915_svm_unbind_mm(vm);
		}
		break;
	default:
		err = -EINVAL;
	}

	i915_vm_put(vm);
	return err;
}

static int i915_gem_vm_getparam_ioctl(struct drm_device *dev, void *data,
				      struct drm_file *file)
{
	struct drm_i915_file_private *file_priv = file->driver_priv;
	struct prelim_drm_i915_gem_vm_param *args = data;
	struct i915_address_space *vm;
	int err = 0;

	vm = i915_address_space_lookup(file_priv, args->vm_id);
	if (unlikely(!vm))
		return -ENOENT;

	switch (lower_32_bits(args->param)) {
	case PRELIM_I915_GEM_VM_PARAM_SVM:
		args->value = i915_vm_is_svm_enabled(vm);
		break;
	default:
		err = -EINVAL;
	}

	i915_vm_put(vm);
	return err;
}

static int get_ppgtt(struct drm_i915_file_private *file_priv,
		     struct i915_gem_context *ctx,
		     struct drm_i915_gem_context_param *args)
{
	struct i915_address_space *vm;
	int err;
	u32 id;

	if (!rcu_access_pointer(ctx->vm))
		return -ENODEV;

	rcu_read_lock();
	vm = context_get_vm_rcu(ctx);
	rcu_read_unlock();
	if (!vm)
		return -ENODEV;

	i915_debugger_wait_on_discovery(vm->i915, vm->client);

	err = xa_alloc(&file_priv->vm_xa, &id, vm, xa_limit_32b, GFP_KERNEL);
	if (err)
		goto err_put;

	i915_vm_open(vm);

	GEM_BUG_ON(id == 0); /* reserved for invalid/unassigned ppgtt */
	args->value = id;
	args->size = 0;

err_put:
	i915_vm_put(vm);
	return err;
}

static int set_ppgtt(struct drm_i915_file_private *file_priv,
		     struct i915_gem_context *ctx,
		     struct drm_i915_gem_context_param *args,
		     bool is_ctx_create)
{
	struct i915_address_space *vm, *old;
	int err;

	if (args->size)
		return -EINVAL;

	if (!rcu_access_pointer(ctx->vm))
		return -ENODEV;

	if (upper_32_bits(args->value))
		return -ENOENT;

	rcu_read_lock();
	vm = xa_load(&file_priv->vm_xa, args->value);
	if (vm && !kref_get_unless_zero(&vm->ref))
		vm = NULL;
	rcu_read_unlock();
	if (!vm)
		return -ENOENT;

	err = mutex_lock_interruptible(&ctx->mutex);
	if (err)
		goto out;

	if (i915_gem_context_is_closed(ctx)) {
		err = -ENOENT;
		goto unlock;
	}

	if (vm == rcu_access_pointer(ctx->vm)) {
		mutex_unlock(&ctx->mutex);
		goto out;
	}

	old = __set_ppgtt(ctx, vm);
	if (!old) {
		err = -ENOENT;
		goto unlock;
	}

	/* Teardown the existing obj:vma cache, it will have to be rebuilt. */
	lut_close(ctx);

	/*
	 * We need to flush any requests using the current ppgtt before
	 * we release it as the requests do not hold a reference themselves,
	 * only indirectly through the context.
	 */
	context_apply_all(ctx, __apply_ppgtt, vm);

unlock:
	mutex_unlock(&ctx->mutex);

	if (!err) {
		if (!is_ctx_create) {
			i915_debugger_vm_destroy(file_priv->client, old);
			i915_debugger_context_param_vm(file_priv->client, ctx, vm);
		}
		i915_vm_close(old);
	}
out:
	i915_vm_put(vm);
	return err;
}

int
i915_gem_user_to_context_sseu(struct intel_gt *gt,
			      const struct drm_i915_gem_context_param_sseu *user,
			      struct intel_sseu *context)
{
	const struct sseu_dev_info *device = &gt->info.sseu;
	struct drm_i915_private *i915 = gt->i915;
	unsigned int dev_subslice_mask = intel_sseu_get_hsw_subslices(device, 0);

	/* No zeros in any field. */
	if (!user->slice_mask || !user->subslice_mask ||
	    !user->min_eus_per_subslice || !user->max_eus_per_subslice)
		return -EINVAL;

	/* Max > min. */
	if (user->max_eus_per_subslice < user->min_eus_per_subslice)
		return -EINVAL;

	/*
	 * Some future proofing on the types since the uAPI is wider than the
	 * current internal implementation.
	 */
	if (overflows_type(user->slice_mask, context->slice_mask) ||
	    overflows_type(user->subslice_mask, context->subslice_mask) ||
	    overflows_type(user->min_eus_per_subslice,
			   context->min_eus_per_subslice) ||
	    overflows_type(user->max_eus_per_subslice,
			   context->max_eus_per_subslice))
		return -EINVAL;

	/* Check validity against hardware. */
	if (user->slice_mask & ~device->slice_mask)
		return -EINVAL;

	if (user->subslice_mask & ~dev_subslice_mask)
		return -EINVAL;

	if (user->max_eus_per_subslice > device->max_eus_per_subslice)
		return -EINVAL;

	context->slice_mask = user->slice_mask;
	context->subslice_mask = user->subslice_mask;
	context->min_eus_per_subslice = user->min_eus_per_subslice;
	context->max_eus_per_subslice = user->max_eus_per_subslice;

	/* Part specific restrictions. */
	if (GRAPHICS_VER(i915) == 11) {
		unsigned int hw_s = hweight8(device->slice_mask);
		unsigned int hw_ss_per_s = hweight8(dev_subslice_mask);
		unsigned int req_s = hweight8(context->slice_mask);
		unsigned int req_ss = hweight8(context->subslice_mask);

		/*
		 * Only full subslice enablement is possible if more than one
		 * slice is turned on.
		 */
		if (req_s > 1 && req_ss != hw_ss_per_s)
			return -EINVAL;

		/*
		 * If more than four (SScount bitfield limit) subslices are
		 * requested then the number has to be even.
		 */
		if (req_ss > 4 && (req_ss & 1))
			return -EINVAL;

		/*
		 * If only one slice is enabled and subslice count is below the
		 * device full enablement, it must be at most half of the all
		 * available subslices.
		 */
		if (req_s == 1 && req_ss < hw_ss_per_s &&
		    req_ss > (hw_ss_per_s / 2))
			return -EINVAL;

		/* ABI restriction - VME use case only. */

		/* All slices or one slice only. */
		if (req_s != 1 && req_s != hw_s)
			return -EINVAL;

		/*
		 * Half subslices or full enablement only when one slice is
		 * enabled.
		 */
		if (req_s == 1 &&
		    (req_ss != hw_ss_per_s && req_ss != (hw_ss_per_s / 2)))
			return -EINVAL;

		/* No EU configuration changes. */
		if ((user->min_eus_per_subslice !=
		     device->max_eus_per_subslice) ||
		    (user->max_eus_per_subslice !=
		     device->max_eus_per_subslice))
			return -EINVAL;
	}

	return 0;
}

static int set_sseu(struct i915_gem_context *ctx,
		    struct drm_i915_gem_context_param *args)
{
	struct drm_i915_private *i915 = ctx->i915;
	struct drm_i915_gem_context_param_sseu user_sseu;
	struct intel_context *ce;
	struct intel_sseu sseu;
	unsigned long lookup;
	int ret;

	if (args->size < sizeof(user_sseu))
		return -EINVAL;

	if (GRAPHICS_VER(i915) != 11)
		return -ENODEV;

	if (copy_from_user(&user_sseu, u64_to_user_ptr(args->value),
			   sizeof(user_sseu)))
		return -EFAULT;

	if (user_sseu.rsvd)
		return -EINVAL;

	if (user_sseu.flags & ~(I915_CONTEXT_SSEU_FLAG_ENGINE_INDEX))
		return -EINVAL;

	lookup = 0;
	if (user_sseu.flags & I915_CONTEXT_SSEU_FLAG_ENGINE_INDEX)
		lookup |= LOOKUP_USER_INDEX;

	ce = lookup_user_engine(ctx, lookup, &user_sseu.engine);
	if (IS_ERR(ce))
		return PTR_ERR(ce);

	/* Only render engine supports RPCS configuration. */
	if (ce->engine->class != RENDER_CLASS) {
		ret = -ENODEV;
		goto out_ce;
	}

	ret = i915_gem_user_to_context_sseu(ce->engine->gt, &user_sseu, &sseu);
	if (ret)
		goto out_ce;

	ret = intel_context_reconfigure_sseu(ce, sseu);
	if (ret)
		goto out_ce;

	args->size = sizeof(user_sseu);

out_ce:
	intel_context_put(ce);
	return ret;
}

struct set_engines {
	struct i915_gem_context *ctx;
	struct i915_gem_engines *engines;
};

static int
set_engines__load_balance(struct i915_user_extension __user *base, void *data)
{
	struct i915_context_engines_load_balance __user *ext =
		container_of_user(base, typeof(*ext), base);
	const struct set_engines *set = data;
	struct drm_i915_private *i915 = set->ctx->i915;
	struct intel_engine_cs *stack[16];
	struct intel_engine_cs **siblings;
	struct intel_context *ce;
	u16 num_siblings, idx;
	unsigned int n;
	int err;

	if (!HAS_EXECLISTS(i915))
		return -ENODEV;

	if (get_user(idx, &ext->engine_index))
		return -EFAULT;

	if (idx >= set->engines->num_engines) {
		drm_dbg(&i915->drm, "Invalid placement value, %d >= %d\n",
			idx, set->engines->num_engines);
		return -EINVAL;
	}

	idx = array_index_nospec(idx, set->engines->num_engines);
	if (set->engines->engines[idx]) {
		drm_dbg(&i915->drm,
			"Invalid placement[%d], already occupied\n", idx);
		return -EEXIST;
	}

	if (get_user(num_siblings, &ext->num_siblings))
		return -EFAULT;

	err = check_user_mbz(&ext->flags);
	if (err)
		return err;

	err = check_user_mbz(&ext->mbz64);
	if (err)
		return err;

	siblings = stack;
	if (num_siblings > ARRAY_SIZE(stack)) {
		siblings = kmalloc_array(num_siblings,
					 sizeof(*siblings),
					 GFP_KERNEL);
		if (!siblings)
			return -ENOMEM;
	}

	for (n = 0; n < num_siblings; n++) {
		struct i915_engine_class_instance ci;

		if (copy_from_user(&ci, &ext->engines[n], sizeof(ci))) {
			err = -EFAULT;
			goto out_siblings;
		}

		siblings[n] = intel_engine_lookup_user(i915,
						       ci.engine_class,
						       ci.engine_instance);
		if (!siblings[n]) {
			drm_dbg(&i915->drm,
				"Invalid sibling[%d]: { class:%d, inst:%d }\n",
				n, ci.engine_class, ci.engine_instance);
			err = -EINVAL;
			goto out_siblings;
		}
	}

	ce = intel_engine_create_virtual(siblings, n, 0);
	if (IS_ERR(ce)) {
		err = PTR_ERR(ce);
		goto out_siblings;
	}

	intel_context_set_gem(ce, set->ctx);

	if (cmpxchg(&set->engines->engines[idx], NULL, ce)) {
		intel_context_put(ce);
		err = -EEXIST;
		goto out_siblings;
	}

out_siblings:
	if (siblings != stack)
		kfree(siblings);

	return err;
}

static int
set_engines__bond(struct i915_user_extension __user *base, void *data)
{
	struct i915_context_engines_bond __user *ext =
		container_of_user(base, typeof(*ext), base);
	const struct set_engines *set = data;
	struct drm_i915_private *i915 = set->ctx->i915;
	struct i915_engine_class_instance ci;
	struct intel_engine_cs *virtual;
	struct intel_engine_cs *master;
	u16 idx, num_bonds;
	int err, n;

	if (get_user(idx, &ext->virtual_index))
		return -EFAULT;

	if (idx >= set->engines->num_engines) {
		drm_dbg(&i915->drm,
			"Invalid index for virtual engine: %d >= %d\n",
			idx, set->engines->num_engines);
		return -EINVAL;
	}

	idx = array_index_nospec(idx, set->engines->num_engines);
	if (!set->engines->engines[idx]) {
		drm_dbg(&i915->drm, "Invalid engine at %d\n", idx);
		return -EINVAL;
	}
	virtual = set->engines->engines[idx]->engine;

	if (intel_engine_uses_guc(virtual)) {
		DRM_DEBUG("bonding extension not supported with GuC submission");
		return -ENODEV;
	}

	err = check_user_mbz(&ext->flags);
	if (err)
		return err;

	for (n = 0; n < ARRAY_SIZE(ext->mbz64); n++) {
		err = check_user_mbz(&ext->mbz64[n]);
		if (err)
			return err;
	}

	if (copy_from_user(&ci, &ext->master, sizeof(ci)))
		return -EFAULT;

	master = intel_engine_lookup_user(i915,
					  ci.engine_class, ci.engine_instance);
	if (!master) {
		drm_dbg(&i915->drm,
			"Unrecognised master engine: { class:%u, instance:%u }\n",
			ci.engine_class, ci.engine_instance);
		return -EINVAL;
	}

	if (get_user(num_bonds, &ext->num_bonds))
		return -EFAULT;

	for (n = 0; n < num_bonds; n++) {
		struct intel_engine_cs *bond;

		if (copy_from_user(&ci, &ext->engines[n], sizeof(ci)))
			return -EFAULT;

		bond = intel_engine_lookup_user(i915,
						ci.engine_class,
						ci.engine_instance);
		if (!bond) {
			drm_dbg(&i915->drm,
				"Unrecognised engine[%d] for bonding: { class:%d, instance: %d }\n",
				n, ci.engine_class, ci.engine_instance);
			return -EINVAL;
		}

		/*
		 * A non-virtual engine has no siblings to choose between; and
		 * a submit fence will always be directed to the one engine.
		 */
		err = intel_engine_attach_bond(virtual, master, bond);
		if (err)
			return err;
	}

	return 0;
}

static int perma_pin_contexts(struct intel_context *ce)
{
	struct intel_context *child;
	int i = 0, j = 0, ret;

	GEM_BUG_ON(!intel_context_is_parent(ce));

	ret = intel_context_pin(ce);
	if (unlikely(ret))
		return ret;

	for_each_child(ce, child) {
		ret = intel_context_pin(child);
		if (unlikely(ret))
			goto unwind;
		++i;
	}

	set_bit(CONTEXT_PERMA_PIN, &ce->flags);

	return 0;

unwind:
	intel_context_unpin(ce);
	for_each_child(ce, child) {
		if (j++ < i)
			intel_context_unpin(child);
		else
			break;
	}

	return ret;
}

static int
set_engines__parallel_submit(struct i915_user_extension __user *base, void *data)
{
	struct i915_context_engines_parallel_submit __user *ext =
		container_of_user(base, typeof(*ext), base);
	const struct set_engines *set = data;
	struct drm_i915_private *i915 = set->ctx->i915;
	struct intel_context *ce, *child;
	struct i915_engine_class_instance prev_engine;
	u64 flags;
	int err = 0, n, i, j;
	u16 slot, width, num_siblings;
	struct intel_engine_cs **siblings = NULL;
	intel_engine_mask_t prev_mask;

	if (get_user(slot, &ext->engine_index))
		return -EFAULT;

	if (get_user(width, &ext->width))
		return -EFAULT;

	if (get_user(num_siblings, &ext->num_siblings))
		return -EFAULT;

	if (!intel_uc_uses_guc_submission(&to_gt(i915)->uc) &&
	    num_siblings != 1) {
		drm_dbg(&i915->drm, "Only 1 sibling (%d) supported in non-GuC mode\n",
			num_siblings);
		return -EINVAL;
	}

	if (slot >= set->engines->num_engines) {
		drm_dbg(&i915->drm, "Invalid placement value, %d >= %d\n",
			slot, set->engines->num_engines);
		return -EINVAL;
	}

	if (get_user(flags, &ext->flags))
		return -EFAULT;

	if (flags) {
		drm_dbg(&i915->drm, "Unknown flags 0x%02llx", flags);
		return -EINVAL;
	}

	for (n = 0; n < ARRAY_SIZE(ext->mbz64); n++) {
		err = check_user_mbz(&ext->mbz64[n]);
		if (err)
			return err;
	}

	if (width < 2) {
		drm_dbg(&i915->drm, "Width (%d) < 2\n", width);
		return -EINVAL;
	}

	if (num_siblings < 1) {
		drm_dbg(&i915->drm, "Number siblings (%d) < 1\n",
			num_siblings);
		return -EINVAL;
	}

	siblings = kmalloc(array3_size(num_siblings, width, sizeof(*siblings)),
			   GFP_KERNEL);
	if (!siblings)
		return -ENOMEM;

	/* Create contexts / engines */
	for (i = 0; i < width; ++i) {
		intel_engine_mask_t current_mask = 0;

		for (j = 0; j < num_siblings; ++j) {
			struct i915_engine_class_instance ci;

			n = i * num_siblings + j;
			if (copy_from_user(&ci, &ext->engines[n], sizeof(ci))) {
				err = -EFAULT;
				goto out;
			}

			siblings[n] =
				intel_engine_lookup_user(i915, ci.engine_class,
							 ci.engine_instance);
			if (!siblings[n]) {
				drm_dbg(&i915->drm,
					"Invalid sibling[%d]: { class:%d, inst:%d }\n",
					n, ci.engine_class, ci.engine_instance);
				err = -EINVAL;
				goto out;
			}

			/*
			 * We don't support breadcrumb handshake on these
			 * classes
			 */
			if (siblings[n]->class == RENDER_CLASS ||
			    siblings[n]->class == COMPUTE_CLASS) {
				err = -EINVAL;
				goto out;
			}

			if (n) {
				if (prev_engine.engine_class !=
				    ci.engine_class) {
					drm_dbg(&i915->drm,
						"Mismatched class %d, %d\n",
						prev_engine.engine_class,
						ci.engine_class);
					err = -EINVAL;
					goto out;
				}
			}

			prev_engine = ci;
			current_mask |= siblings[n]->logical_mask;
		}

		if (i > 0) {
			if (current_mask != prev_mask << 1) {
				drm_dbg(&i915->drm,
					"Non contiguous logical mask 0x%x, 0x%x\n",
					prev_mask, current_mask);
				err = -EINVAL;
				goto out;
			}
		}
		prev_mask = current_mask;
	}

	ce = intel_engine_create_parallel(siblings, num_siblings, width);
	if (IS_ERR(ce)) {
		err = PTR_ERR(ce);
		goto out;
	}

	intel_context_set_gem(ce, set->ctx);
	for_each_child(ce, child)
		intel_context_set_gem(child, set->ctx);

	err = perma_pin_contexts(ce);
	if (err) {
		intel_context_put(ce);
		goto out;
	}

	if (cmpxchg(&set->engines->engines[slot], NULL, ce)) {
		intel_context_put(ce);
		err = -EEXIST;
		goto out;
	}

out:
	kfree(siblings);
	return err;
}

static const i915_user_extension_fn set_engines__extensions[] = {
	[I915_CONTEXT_ENGINES_EXT_LOAD_BALANCE] = set_engines__load_balance,
	[I915_CONTEXT_ENGINES_EXT_BOND] = set_engines__bond,
	[I915_CONTEXT_ENGINES_EXT_PARALLEL_SUBMIT] =
		set_engines__parallel_submit,
	[PRELIM_I915_USER_EXT_MASK(PRELIM_I915_CONTEXT_ENGINES_EXT_PARALLEL2_SUBMIT)] =
		set_engines__parallel_submit,
};

static int
set_engines(struct i915_gem_context *ctx,
	    const struct drm_i915_gem_context_param *args)
{
	struct drm_i915_private *i915 = ctx->i915;
	struct i915_context_param_engines __user *user =
		u64_to_user_ptr(args->value);
	struct set_engines set = { .ctx = ctx };
	unsigned int num_engines, n;
	u64 extensions;
	int err;

	if (!args->size) { /* switch back to legacy user_ring_map */
		if (!i915_gem_context_user_engines(ctx))
			return 0;

		set.engines = default_engines(ctx);
		if (IS_ERR(set.engines))
			return PTR_ERR(set.engines);

		goto replace;
	}

	BUILD_BUG_ON(!IS_ALIGNED(sizeof(*user), sizeof(*user->engines)));
	if (args->size < sizeof(*user) ||
	    !IS_ALIGNED(args->size, sizeof(*user->engines))) {
		drm_dbg(&i915->drm, "Invalid size for engine array: %d\n",
			args->size);
		return -EINVAL;
	}

	/*
	 * Note that I915_EXEC_RING_MASK limits execbuf to only using the
	 * first 64 engines defined here.
	 */
	num_engines = (args->size - sizeof(*user)) / sizeof(*user->engines);
	set.engines = alloc_engines(num_engines);
	if (!set.engines)
		return -ENOMEM;

	for (n = 0; n < num_engines; n++) {
		struct i915_engine_class_instance ci;
		struct intel_engine_cs *engine;
		struct intel_context *ce;

		if (copy_from_user(&ci, &user->engines[n], sizeof(ci))) {
			__free_engines(set.engines, n);
			return -EFAULT;
		}

		if (ci.engine_class == (u16)I915_ENGINE_CLASS_INVALID &&
		    ci.engine_instance == (u16)I915_ENGINE_CLASS_INVALID_NONE) {
			set.engines->engines[n] = NULL;
			continue;
		}

		engine = intel_engine_lookup_user(ctx->i915,
						  ci.engine_class,
						  ci.engine_instance);
		if (!engine) {
			drm_dbg(&i915->drm,
				"Invalid engine[%d]: { class:%d, instance:%d }\n",
				n, ci.engine_class, ci.engine_instance);
			__free_engines(set.engines, n);
			return -ENOENT;
		}

		ce = intel_context_create(engine);
		if (IS_ERR(ce)) {
			__free_engines(set.engines, n);
			return PTR_ERR(ce);
		}

		intel_context_set_gem(ce, ctx);

		set.engines->engines[n] = ce;
	}
	set.engines->num_engines = num_engines;

	err = -EFAULT;
	if (!get_user(extensions, &user->extensions))
		err = i915_user_extensions(u64_to_user_ptr(extensions),
					   set_engines__extensions,
					   ARRAY_SIZE(set_engines__extensions),
					   &set);
	if (err) {
		free_engines(set.engines);
		return err;
	}

replace:
	mutex_lock(&ctx->engines_mutex);
	if (i915_gem_context_is_closed(ctx)) {
		mutex_unlock(&ctx->engines_mutex);
		free_engines(set.engines);
		return -ENOENT;
	}
	if (args->size)
		i915_gem_context_set_user_engines(ctx);
	else
		i915_gem_context_clear_user_engines(ctx);
	ctx->engine_mask = engine_mask(set.engines);
	set.engines = rcu_replace_pointer(ctx->engines, set.engines, 1);
	mutex_unlock(&ctx->engines_mutex);

	/* Keep track of old engine sets for kill_context() */
	engines_idle_release(ctx, set.engines);

	i915_debugger_context_param_engines(ctx);
	return 0;
}

static int
get_engines(struct i915_gem_context *ctx,
	    struct drm_i915_gem_context_param *args)
{
	struct i915_context_param_engines __user *user;
	struct i915_gem_engines *e;
	size_t n, count, size;
	bool user_engines;
	int err = 0;

	e = i915_gem_context_engines_get(ctx, &user_engines);
	if (!e)
		return -ENOENT;

	if (!user_engines) {
		i915_gem_context_engines_put(e);
		args->size = 0;
		return 0;
	}

	count = e->num_engines;

	/* Be paranoid in case we have an impedance mismatch */
	if (!check_struct_size(user, engines, count, &size)) {
		err = -EINVAL;
		goto err_free;
	}
	if (overflows_type(size, args->size)) {
		err = -EINVAL;
		goto err_free;
	}

	if (!args->size) {
		args->size = size;
		goto err_free;
	}

	if (args->size < size) {
		err = -EINVAL;
		goto err_free;
	}

	user = u64_to_user_ptr(args->value);
	if (put_user(0, &user->extensions)) {
		err = -EFAULT;
		goto err_free;
	}

	for (n = 0; n < count; n++) {
		struct i915_engine_class_instance ci = {
			.engine_class = I915_ENGINE_CLASS_INVALID,
			.engine_instance = I915_ENGINE_CLASS_INVALID_NONE,
		};

		if (e->engines[n]) {
			ci.engine_class = e->engines[n]->engine->uabi_class;
			ci.engine_instance = e->engines[n]->engine->uabi_instance;
		}

		if (copy_to_user(&user->engines[n], &ci, sizeof(ci))) {
			err = -EFAULT;
			goto err_free;
		}
	}

	args->size = size;

err_free:
	i915_gem_context_engines_put(e);
	return err;
}

static int
set_persistence(struct i915_gem_context *ctx,
		const struct drm_i915_gem_context_param *args)
{
	if (args->size)
		return -EINVAL;

	return __context_set_persistence(ctx, args->value);
}

static void __apply_priority(struct intel_context *ce, void *arg)
{
	struct i915_gem_context *ctx = arg;

	if (!intel_engine_has_timeslices(ce->engine))
		return;

	if (ctx->sched.priority >= I915_PRIORITY_NORMAL &&
	    intel_engine_has_semaphores(ce->engine))
		intel_context_set_use_semaphores(ce);
	else
		intel_context_clear_use_semaphores(ce);
}

static int set_priority(struct i915_gem_context *ctx,
			const struct drm_i915_gem_context_param *args)
{
	s64 priority = args->value;

	if (args->size)
		return -EINVAL;

	if (!(ctx->i915->caps.scheduler & I915_SCHEDULER_CAP_PRIORITY))
		return -ENODEV;

	if (priority > I915_CONTEXT_MAX_USER_PRIORITY ||
	    priority < I915_CONTEXT_MIN_USER_PRIORITY)
		return -EINVAL;

	if (priority > I915_CONTEXT_DEFAULT_PRIORITY &&
	    !capable(CAP_SYS_NICE))
		return -EPERM;

	ctx->sched.priority = priority;
	context_apply_all(ctx, __apply_priority, ctx);

	return 0;
}

static void __apply_debug_flags(struct intel_context *ce, void *arg)
{
	struct i915_gem_context *ctx = arg;

	if (i915_gem_context_has_sip(ctx))
		intel_context_set_debug(ce);
	else
		intel_context_clear_debug(ce);
}

static int set_debug_flags(struct i915_gem_context *ctx,
			   const struct drm_i915_gem_context_param *args)
{
	const u32 mask_allow = PRELIM_I915_CONTEXT_PARAM_DEBUG_FLAG_SIP;
	long unsigned int mask;
	u32 value, i;
	int ret = -EINVAL;

	if (args->size)
		return -EINVAL;

	mask = upper_32_bits(args->value);
	value = lower_32_bits(args->value);

	if (mask & ~mask_allow)
		return -EINVAL;

	if (!mask)
		return 0;

	for_each_set_bit(i, &mask, 32) {
		const u32 flag = BIT(i);
		const bool set = flag & value;

		switch (flag) {
		case PRELIM_I915_CONTEXT_PARAM_DEBUG_FLAG_SIP:
			if (set)
				i915_gem_context_set_sip(ctx);
			else
				i915_gem_context_clear_sip(ctx);
			ret = 0;
			break;
		}

		if (ret)
			break;
	}

	context_apply_all(ctx, __apply_debug_flags, ctx);

	return ret;
}

static int get_debug_flags(struct i915_gem_context *ctx,
			   struct drm_i915_gem_context_param *args)
{
	const u64 allowed = PRELIM_I915_CONTEXT_PARAM_DEBUG_FLAG_SIP;
	u64 v = 0;

	if (i915_gem_context_has_sip(ctx))
		v |= PRELIM_I915_CONTEXT_PARAM_DEBUG_FLAG_SIP;

	args->size = 0;
	args->value = allowed << 32 | v;

	return 0;
}

static int set_runalone(struct i915_gem_context *ctx,
			struct drm_i915_gem_context_param *args)
{
	if (!CCS_MASK(to_gt(ctx->i915)))
		return -ENODEV;

	set_bit(UCONTEXT_RUNALONE, &ctx->user_flags);

	return 0;
}

static int set_acc(struct i915_gem_context *ctx,
		   struct drm_i915_gem_context_param *args)
{
	struct prelim_drm_i915_gem_context_param_acc user_acc;
	struct drm_i915_private *i915 = ctx->i915;

	if (!(intel_uc_uses_guc_submission(&to_gt(i915)->uc)))
		return -ENODEV;

	if (!(INTEL_INFO(i915)->has_access_counter))
		return -ENODEV;

	if (args->size < sizeof(user_acc))
		return -EINVAL;

	if (copy_from_user(&user_acc, u64_to_user_ptr(args->value),
			sizeof(user_acc)))
		return -EFAULT;

	DRM_DEBUG("User ACC settings: acg=%d trigger=%d notify=%d\n",
		user_acc.granularity, user_acc.trigger, user_acc.notify);

	/*
	 * There is no need to do range checking for trigger and
	 * notify as they are defined the same as the HW.
	 */
	if (user_acc.granularity > PRELIM_I915_CONTEXT_ACG_64M)
		return -EINVAL;

	/* Wa_14012038966:pvc[bd_a0] */
	if (IS_PVC_BD_STEP(i915, STEP_A0, STEP_B0) &&
	    user_acc.trigger == 1)
		return -EINVAL;

	ctx->acc_granularity = user_acc.granularity;
	ctx->acc_trigger = user_acc.trigger;
	ctx->acc_notify = user_acc.notify;

	args->size = 0;
	DRM_DEBUG("Done ACC settings: acg=%d trigger=%d notify=%d\n",
		ctx->acc_granularity, ctx->acc_trigger, ctx->acc_notify);

	return 0;
}

static int ctx_setparam(struct drm_i915_file_private *fpriv,
			struct i915_gem_context *ctx,
			struct drm_i915_gem_context_param *args,
			bool is_ctx_create)
{
	int ret = 0;

	switch (args->param) {
	case I915_CONTEXT_PARAM_NO_ERROR_CAPTURE:
		if (args->size)
			ret = -EINVAL;
		else if (args->value)
			i915_gem_context_set_no_error_capture(ctx);
		else
			i915_gem_context_clear_no_error_capture(ctx);
		break;

	case I915_CONTEXT_PARAM_BANNABLE:
		if (args->size)
			ret = -EINVAL;
		else if (!capable(CAP_SYS_ADMIN) && !args->value)
			ret = -EPERM;
		else if (args->value)
			i915_gem_context_set_bannable(ctx);
		else if (ctx->uses_protected_content)
			ret = -EPERM;
		else
			i915_gem_context_clear_bannable(ctx);
		break;

	case I915_CONTEXT_PARAM_RECOVERABLE:
		if (args->size)
			ret = -EINVAL;
		else if (!args->value)
			i915_gem_context_clear_recoverable(ctx);
		else if (ctx->uses_protected_content)
			ret = -EPERM;
		else
			i915_gem_context_set_recoverable(ctx);
		break;

	case I915_CONTEXT_PARAM_PRIORITY:
		ret = set_priority(ctx, args);
		break;

	case PRELIM_I915_CONTEXT_PARAM_RUNALONE:
		if (is_ctx_create)
			ret = set_runalone(ctx, args);
		else
			return -EPERM;
		break;

	case PRELIM_I915_CONTEXT_PARAM_ACC:
		/*
		 * Only allowed to be called from i915_gem_context_create_ioctl
		 * extension as access counter settings are only allowed to be
		 * set once.
		 */
		if (is_ctx_create)
			ret = set_acc(ctx, args);
		else {
			DRM_DEBUG("Allowed only in creating context!\n");
			return -EPERM;
		}
		break;

	case I915_CONTEXT_PARAM_SSEU:
		ret = set_sseu(ctx, args);
		break;

	case I915_CONTEXT_PARAM_VM:
		ret = set_ppgtt(fpriv, ctx, args, is_ctx_create);
		break;

	case I915_CONTEXT_PARAM_ENGINES:
		ret = set_engines(ctx, args);
		break;

	case I915_CONTEXT_PARAM_PERSISTENCE:
		ret = set_persistence(ctx, args);
		break;

	case I915_CONTEXT_PARAM_PROTECTED_CONTENT:
	case PRELIM_I915_CONTEXT_PARAM_PROTECTED_CONTENT:
		ret = __context_set_protected(fpriv->dev_priv, ctx,
					      args->value);
		break;

	case PRELIM_I915_CONTEXT_PARAM_DEBUG_FLAGS:
		ret = set_debug_flags(ctx, args);
		break;

	case I915_CONTEXT_PARAM_NO_ZEROMAP:
	case I915_CONTEXT_PARAM_BAN_PERIOD:
	case I915_CONTEXT_PARAM_RINGSIZE:
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

struct create_ext {
	struct i915_gem_context *ctx;
	struct drm_i915_file_private *fpriv;
};

static int create_setparam(struct i915_user_extension __user *ext, void *data)
{
	struct drm_i915_gem_context_create_ext_setparam local;
	const struct create_ext *arg = data;

	if (copy_from_user(&local, ext, sizeof(local)))
		return -EFAULT;

	if (local.param.ctx_id)
		return -EINVAL;

	return ctx_setparam(arg->fpriv, arg->ctx, &local.param, true);
}

static int invalid_ext(struct i915_user_extension __user *ext, void *data)
{
	return -EINVAL;
}

static const i915_user_extension_fn create_extensions[] = {
	[I915_CONTEXT_CREATE_EXT_SETPARAM] = create_setparam,
	[PRELIM_I915_USER_EXT_MASK(PRELIM_I915_CONTEXT_CREATE_EXT_CLONE)] = invalid_ext,
};

static bool client_is_banned(struct drm_i915_file_private *file_priv)
{
	return atomic_read(&file_priv->ban_score) >= I915_CLIENT_SCORE_BANNED;
}

int i915_gem_context_create_ioctl(struct drm_device *dev, void *data,
				  struct drm_file *file)
{
	struct drm_i915_private *i915 = to_i915(dev);
	struct drm_i915_gem_context_create_ext *args = data;
	struct create_ext ext_data;
	int ret;
	u32 id;

	if (!DRIVER_CAPS(i915)->has_logical_contexts)
		return -ENODEV;

	if (args->flags & PRELIM_I915_CONTEXT_CREATE_FLAGS_UNKNOWN)
		return -EINVAL;

	ret = intel_gt_terminally_wedged(to_gt(i915));
	if (ret)
		return ret;

	ext_data.fpriv = file->driver_priv;
	if (client_is_banned(ext_data.fpriv)) {
		drm_dbg(&i915->drm,
			"client %s[%d] banned from creating ctx\n",
			current->comm, task_pid_nr(current));
		return -EIO;
	}

	ret = i915_drm_client_update(ext_data.fpriv->client, current);
	if (ret)
		return ret;

	ext_data.ctx = i915_gem_context_create_for_gt(to_gt(i915), args->flags);
	if (IS_ERR(ext_data.ctx))
		return PTR_ERR(ext_data.ctx);

	if (args->flags & I915_CONTEXT_CREATE_FLAGS_USE_EXTENSIONS) {
		ret = i915_user_extensions(u64_to_user_ptr(args->extensions),
					   create_extensions,
					   ARRAY_SIZE(create_extensions),
					   &ext_data);
		if (ret)
			goto err_ctx;
	}

	ret = gem_context_register(ext_data.ctx, ext_data.fpriv, &id);
	if (ret < 0)
		goto err_ctx;

	args->ctx_id = id;
	return 0;

err_ctx:
	context_close(ext_data.ctx);
	return ret;
}

static void retire_requests(const struct intel_timeline *tl)
{
	struct i915_request *rq, *rn;

	list_for_each_entry_safe(rq, rn, &tl->requests, link)
		if (!i915_request_retire(rq))
			return;
}

static void timeline_retire(struct intel_context *ce, void *data)
{
	struct intel_timeline *tl = ce->timeline;

	if (!tl || list_empty(&tl->requests))
		return;

	if (mutex_trylock(&tl->mutex)) {
		retire_requests(tl);
		mutex_unlock(&tl->mutex);
	}
}

static void context_retire(struct i915_gem_context *ctx)
{
	context_apply_all(ctx, timeline_retire, NULL);
}

int i915_gem_context_destroy_ioctl(struct drm_device *dev, void *data,
				   struct drm_file *file)
{
	struct drm_i915_gem_context_destroy *args = data;
	struct drm_i915_file_private *file_priv = file->driver_priv;
	struct i915_gem_context *ctx;

	if (args->pad != 0)
		return -EINVAL;

	if (!args->ctx_id)
		return -ENOENT;

	i915_debugger_wait_on_discovery(file_priv->dev_priv,
					file_priv->client);

	ctx = xa_erase(&file_priv->context_xa, args->ctx_id);
	if (!ctx)
		return -ENOENT;

	context_retire(ctx);
	context_close(ctx);

	i915_gem_flush_free_objects(to_i915(dev));
	flush_workqueue(to_i915(dev)->wq);
	return 0;
}

static int get_sseu(struct i915_gem_context *ctx,
		    struct drm_i915_gem_context_param *args)
{
	struct drm_i915_gem_context_param_sseu user_sseu;
	struct intel_context *ce;
	unsigned long lookup;
	int err;

	if (args->size == 0)
		goto out;
	else if (args->size < sizeof(user_sseu))
		return -EINVAL;

	if (copy_from_user(&user_sseu, u64_to_user_ptr(args->value),
			   sizeof(user_sseu)))
		return -EFAULT;

	if (user_sseu.rsvd)
		return -EINVAL;

	if (user_sseu.flags & ~(I915_CONTEXT_SSEU_FLAG_ENGINE_INDEX))
		return -EINVAL;

	lookup = 0;
	if (user_sseu.flags & I915_CONTEXT_SSEU_FLAG_ENGINE_INDEX)
		lookup |= LOOKUP_USER_INDEX;

	ce = lookup_user_engine(ctx, lookup, &user_sseu.engine);
	if (IS_ERR(ce))
		return PTR_ERR(ce);

	err = intel_context_lock_pinned(ce); /* serialises with set_sseu */
	if (err) {
		intel_context_put(ce);
		return err;
	}

	user_sseu.slice_mask = ce->sseu.slice_mask;
	user_sseu.subslice_mask = ce->sseu.subslice_mask;
	user_sseu.min_eus_per_subslice = ce->sseu.min_eus_per_subslice;
	user_sseu.max_eus_per_subslice = ce->sseu.max_eus_per_subslice;

	intel_context_unlock_pinned(ce);
	intel_context_put(ce);

	if (copy_to_user(u64_to_user_ptr(args->value), &user_sseu,
			 sizeof(user_sseu)))
		return -EFAULT;

out:
	args->size = sizeof(user_sseu);

	return 0;
}

static int get_acc(struct i915_gem_context *ctx,
		   struct drm_i915_gem_context_param *args)
{
	struct prelim_drm_i915_gem_context_param_acc user_acc = {
		.granularity = ctx->acc_granularity,
		.notify = ctx->acc_notify,
		.trigger = ctx->acc_trigger,
	};

	if (args->size == 0)
		goto out;
	else if (args->size < sizeof(user_acc))
		return -EINVAL;

	if (copy_to_user(u64_to_user_ptr(args->value), &user_acc,
			 sizeof(user_acc)))
		return -EFAULT;

out:
	args->size = sizeof(user_acc);
	return 0;
}

int i915_gem_context_getparam_ioctl(struct drm_device *dev, void *data,
				    struct drm_file *file)
{
	struct drm_i915_file_private *file_priv = file->driver_priv;
	struct drm_i915_gem_context_param *args = data;
	struct i915_gem_context *ctx;
	struct i915_address_space *vm;
	int ret = 0;

	ctx = i915_gem_context_lookup(file_priv, args->ctx_id);
	if (!ctx)
		return -ENOENT;

	switch (args->param) {
	case I915_CONTEXT_PARAM_GTT_SIZE:
		args->size = 0;
		vm = i915_gem_context_get_eb_vm(ctx);
		args->value = vm->total;
		i915_vm_put(vm);

		break;

	case I915_CONTEXT_PARAM_NO_ERROR_CAPTURE:
		args->size = 0;
		args->value = i915_gem_context_no_error_capture(ctx);
		break;

	case I915_CONTEXT_PARAM_BANNABLE:
		args->size = 0;
		args->value = i915_gem_context_is_bannable(ctx);
		break;

	case I915_CONTEXT_PARAM_RECOVERABLE:
		args->size = 0;
		args->value = i915_gem_context_is_recoverable(ctx);
		break;

	case I915_CONTEXT_PARAM_PRIORITY:
		args->size = 0;
		args->value = ctx->sched.priority;
		break;

	case PRELIM_I915_CONTEXT_PARAM_ACC:
		ret = get_acc(ctx, args);
		break;

	case I915_CONTEXT_PARAM_SSEU:
		ret = get_sseu(ctx, args);
		break;

	case I915_CONTEXT_PARAM_VM:
		ret = get_ppgtt(file_priv, ctx, args);
		break;

	case I915_CONTEXT_PARAM_ENGINES:
		ret = get_engines(ctx, args);
		break;

	case I915_CONTEXT_PARAM_PERSISTENCE:
		args->size = 0;
		args->value = i915_gem_context_is_persistent(ctx);
		break;

	case I915_CONTEXT_PARAM_PROTECTED_CONTENT:
	case PRELIM_I915_CONTEXT_PARAM_PROTECTED_CONTENT:
		args->size = 0;
		args->value = i915_gem_context_uses_protected_content(ctx);
		break;

	case PRELIM_I915_CONTEXT_PARAM_DEBUG_FLAGS:
		ret = get_debug_flags(ctx, args);
		break;

	case I915_CONTEXT_PARAM_NO_ZEROMAP:
	case I915_CONTEXT_PARAM_BAN_PERIOD:
	case I915_CONTEXT_PARAM_RINGSIZE:
	default:
		ret = -EINVAL;
		break;
	}

	i915_gem_context_put(ctx);
	return ret;
}

int i915_gem_getparam_ioctl(struct drm_device *dev, void *data,
			    struct drm_file *file)
{
	struct drm_i915_gem_context_param *args = data;
	u32 class = upper_32_bits(args->param);

	switch (class) {
	case 0:
		return i915_gem_context_getparam_ioctl(dev, data, file);
	case upper_32_bits(PRELIM_I915_VM_PARAM):
		return i915_gem_vm_getparam_ioctl(dev, data, file);
	}
	return -EINVAL;
}

int i915_gem_context_setparam_ioctl(struct drm_device *dev, void *data,
				    struct drm_file *file)
{
	struct drm_i915_file_private *file_priv = file->driver_priv;
	struct drm_i915_gem_context_param *args = data;
	struct i915_gem_context *ctx;
	int ret;

	ctx = i915_gem_context_lookup(file_priv, args->ctx_id);
	if (!ctx)
		return -ENOENT;

	i915_debugger_wait_on_discovery(ctx->i915, ctx->client);
	ret = ctx_setparam(file_priv, ctx, args, false);

	i915_gem_context_put(ctx);
	return ret;
}

int i915_gem_clos_reserve_ioctl(struct drm_device *dev, void *data,
				struct drm_file *file)
{
	struct drm_i915_file_private *file_priv = file->driver_priv;
	struct prelim_drm_i915_gem_clos_reserve *clos = data;

	if (!HAS_CACHE_CLOS(to_i915(dev)))
		return -EOPNOTSUPP;

	return reserve_clos(file_priv, &clos->clos_index);
}

int i915_gem_clos_free_ioctl(struct drm_device *dev, void *data,
			     struct drm_file *file)
{
	struct drm_i915_file_private *file_priv = file->driver_priv;
	struct prelim_drm_i915_gem_clos_free *clos = data;

	if (!HAS_CACHE_CLOS(to_i915(dev)))
		return -EOPNOTSUPP;

	return free_clos(file_priv, clos->clos_index);
}

int i915_gem_cache_reserve_ioctl(struct drm_device *dev, void *data,
				 struct drm_file *file)
{
	struct drm_i915_file_private *file_priv = file->driver_priv;
	struct prelim_drm_i915_gem_cache_reserve *cache_reserve = data;

	if (!HAS_CACHE_CLOS(to_i915(dev)))
		return -EOPNOTSUPP;

	return reserve_cache_ways(file_priv,
				  cache_reserve->cache_level,
				  cache_reserve->clos_index,
				  &cache_reserve->num_ways);
}

int i915_gem_setparam_ioctl(struct drm_device *dev, void *data,
			    struct drm_file *file)
{
	struct drm_i915_gem_context_param *args = data;
	u32 class = upper_32_bits(args->param);

	switch (class) {
	case 0:
		return i915_gem_context_setparam_ioctl(dev, data, file);
	case upper_32_bits(PRELIM_I915_VM_PARAM):
		return i915_gem_vm_setparam_ioctl(dev, data, file);
	}
	return -EINVAL;
}

int i915_gem_context_reset_stats_ioctl(struct drm_device *dev,
				       void *data, struct drm_file *file)
{
	struct drm_i915_private *i915 = to_i915(dev);
	struct drm_i915_reset_stats *args = data;
	struct i915_gem_context *ctx;
	int ret;

	if (args->flags || args->pad)
		return -EINVAL;

	ret = -ENOENT;
	rcu_read_lock();
	ctx = __i915_gem_context_lookup_rcu(file->driver_priv, args->ctx_id);
	if (!ctx)
		goto out;

	/*
	 * We opt for unserialised reads here. This may result in tearing
	 * in the extremely unlikely event of a GPU hang on this context
	 * as we are querying them. If we need that extra layer of protection,
	 * we should wrap the hangstats with a seqlock.
	 */

	if (capable(CAP_SYS_ADMIN))
		args->reset_count = i915_reset_count(&i915->gpu_error);
	else
		args->reset_count = 0;

	args->batch_active = atomic_read(&ctx->guilty_count);
	args->batch_pending = atomic_read(&ctx->active_count);

	ret = 0;
out:
	rcu_read_unlock();
	return ret;
}

/* GEM context-engines iterator: for_each_gem_engine() */
struct intel_context *
i915_gem_engines_iter_next(struct i915_gem_engines_iter *it)
{
	const struct i915_gem_engines *e = it->engines;
	struct intel_context *ctx;

	if (unlikely(!e))
		return NULL;

	do {
		if (it->idx >= e->num_engines)
			return NULL;

		ctx = e->engines[it->idx++];
	} while (!ctx);

	return ctx;
}

#if IS_ENABLED(CONFIG_DRM_I915_SELFTEST)
#include "selftests/mock_context.c"
#include "selftests/i915_gem_context.c"
#include "selftests/intel_semaphore_hw.c"
#endif

void i915_gem_context_module_exit(void)
{
	kmem_cache_destroy(slab_luts);
}

int __init i915_gem_context_module_init(void)
{
	slab_luts = KMEM_CACHE(i915_lut_handle, 0);
	if (!slab_luts)
		return -ENOMEM;

	return 0;
}
