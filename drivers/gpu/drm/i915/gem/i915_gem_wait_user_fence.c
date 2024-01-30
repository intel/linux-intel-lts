// SPDX-License-Identifier: MIT
/*
 * Copyright © 2020 Intel Corporation
 */

#include <linux/pm_qos.h>
#include <linux/wait.h>

#include <drm/drm_file.h>
#include <drm/drm_utils.h>

#include "gem/i915_gem_context.h"
#include "gt/intel_breadcrumbs.h"
#include "gt/intel_rps.h"

#include "i915_drv.h"
#include "i915_gem_ioctls.h"
#include "i915_user_extensions.h"

#if IS_ENABLED(CONFIG_DRM_I915_CHICKEN_ULL_DMA_BOOST)
static atomic_t dma_latency_num_waiters;

static void dma_latency_change(struct work_struct *wrk)
{
	static struct pm_qos_request qos = {};

	if (atomic_read(&dma_latency_num_waiters)) {
		if (!cpu_latency_qos_request_active(&qos))
			cpu_latency_qos_add_request(&qos, 0);
	} else {
		if (cpu_latency_qos_request_active(&qos))
			cpu_latency_qos_remove_request(&qos);
	}
}

static DECLARE_DELAYED_WORK(dma_latency_work, dma_latency_change);

static void dma_latency_boost(void)
{
	if (atomic_fetch_inc(&dma_latency_num_waiters) == 0)
		queue_delayed_work(system_highpri_wq, &dma_latency_work, 0);
}

static void dma_latency_cancel_boost(void)
{
	if (atomic_dec_and_test(&dma_latency_num_waiters))
		mod_delayed_work(system_highpri_wq, &dma_latency_work, 5);
}
#else
static inline void dma_latency_boost(void) {}
static inline void dma_latency_cancel_boost(void) {}
#endif

struct ufence_wake {
	struct task_struct *tsk;
	void __user *ptr;
	u64 value;
	u64 mask;
	u16 width;
	u16 op;
};

static bool ufence_fault(const struct ufence_wake *wake)
{
	u8 dummy, __user *ptr = wake->ptr;

	/* Fault in the page; or report EFAULT */
	return get_user(dummy, ptr);
}

static bool ufence_compare(const struct ufence_wake *wake)
{
	unsigned long remaining;
	u64 target, value;

	GEM_BUG_ON(wake->width > sizeof(target));
	GEM_BUG_ON(wake->tsk->mm != current->mm);

	pagefault_disable();
	remaining = __copy_from_user_inatomic_nocache(&target, wake->ptr, wake->width);
	pagefault_enable();
	if (unlikely(remaining)) {
		__set_current_state(TASK_RUNNING);
		return false;
	}

	target &= wake->mask;
	value = wake->value & wake->mask;

	switch (wake->op) {
	case PRELIM_I915_UFENCE_WAIT_EQ:
		return value == target;
	case PRELIM_I915_UFENCE_WAIT_NEQ:
		return value != target;

	case PRELIM_I915_UFENCE_WAIT_GT:
		return target > value;
	case PRELIM_I915_UFENCE_WAIT_GTE:
		return target >= value;

	case PRELIM_I915_UFENCE_WAIT_LT:
		return target < value;
	case PRELIM_I915_UFENCE_WAIT_LTE:
		return target <= value;

	case PRELIM_I915_UFENCE_WAIT_AFTER:
		switch (wake->width) {
		case 1:  return (s8)(target - value) > 0;
		case 2:  return (s16)(target - value) > 0;
		case 4:  return (s32)(target - value) > 0;
		default: return (s64)(target - value) > 0;
		}

	case PRELIM_I915_UFENCE_WAIT_BEFORE:
		switch (wake->width) {
		case 1:  return (s8)(target - value) < 0;
		case 2:  return (s16)(target - value) < 0;
		case 4:  return (s32)(target - value) < 0;
		default: return (s64)(target - value) < 0;
		}

	default:
		return true;
	}
}

struct engine_wait {
	struct wait_queue_entry wq_entry;
	union {
		struct wait_queue_head *wq;
		struct intel_breadcrumbs *breadcrumbs;
	};
	struct engine_wait *next;
};

static void
add_soft_wait(struct wait_queue_head *wq, struct engine_wait *wait)
{
	wait->next = NULL;
	wait->wq = wq;
	wait->wq_entry.flags = 0;
	wait->wq_entry.private = current;
	wait->wq_entry.func = default_wake_function;
	add_wait_queue(wq, &wait->wq_entry);
}

static bool wait_exists(struct engine_wait *wait, struct intel_breadcrumbs *b)
{
	while (wait) {
		if (wait->breadcrumbs == b)
			return true;

		wait = wait->next;
	}

	return false;
}

static int
add_engine_wait(struct engine_wait **head, struct intel_engine_cs *engine)
{
	intel_engine_mask_t tmp;

	for_each_engine_masked(engine, engine->gt, engine->mask, tmp) {
		struct intel_breadcrumbs *b;
		struct engine_wait *wait;

		b = engine->breadcrumbs;
		if (!b)
			continue;

		if (wait_exists(*head, b)) /* O(N^2), hopefully small N */
			continue;

		wait = kmalloc(sizeof(*wait), GFP_KERNEL);
		if (!wait)
			return -ENOMEM;

		wait->breadcrumbs = b;
		wait->wq_entry.flags = 0;
		wait->wq_entry.private = current;
		wait->wq_entry.func = default_wake_function;

		intel_breadcrumbs_add_wait(b, &wait->wq_entry);
		intel_rps_boost(&b->irq_engine->gt->rps);

		wait->next = *head;
		*head = wait;
	}

	return 0;
}

static int add_gt_wait(struct i915_gem_context *ctx, struct engine_wait **head)
{
	struct i915_gem_engines_iter it;
	struct intel_context *ce;
	int err = 0;

	for_each_gem_engine(ce, i915_gem_context_lock_engines(ctx), it) {
		err = add_engine_wait(head, ce->engine);
		if (err)
			break;
	}
	i915_gem_context_unlock_engines(ctx);

	return err;
}

static void remove_waits(struct engine_wait *wait)
{
	remove_wait_queue(wait->wq, &wait->wq_entry);
	wait = wait->next;
	if (!wait)
		return;

	remove_wait_queue(wait->wq, &wait->wq_entry);
	wait = wait->next;

	while (wait) {
		struct intel_breadcrumbs *b = wait->breadcrumbs;
		struct engine_wait *next = wait->next;

		intel_rps_cancel_boost(&b->irq_engine->gt->rps);
		intel_breadcrumbs_remove_wait(b, &wait->wq_entry);
		kfree(wait);

		wait = next;
	}
}

static inline unsigned long nsecs_to_jiffies_timeout(const u64 n)
{
	/* nsecs_to_jiffies64() does not guard against overflow */
	if (NSEC_PER_SEC % HZ &&
	    div_u64(n, NSEC_PER_SEC) >= MAX_JIFFY_OFFSET / HZ)
		return MAX_JIFFY_OFFSET;

	return min_t(u64, MAX_JIFFY_OFFSET, nsecs_to_jiffies64(n) + 1);
}

static unsigned long
to_wait_timeout(const struct prelim_drm_i915_gem_wait_user_fence *arg)
{
	if (arg->flags & PRELIM_I915_UFENCE_WAIT_ABSTIME)
		return drm_timeout_abs_to_jiffies(arg->timeout);

	if (arg->timeout < 0)
		return MAX_SCHEDULE_TIMEOUT;

	if (arg->timeout == 0)
		return 0;

	return nsecs_to_jiffies_timeout(arg->timeout);
}

static unsigned long local_clock_ns(unsigned int *cpu)
{
	unsigned long t;

	/*
	 * The local clock is only comparable on the local cpu. However,
	 * we don't want to disable preemption for the entirety of the busy
	 * spin but instead we use the preemption event as an indication
	 * that we have overstayed our welcome and should relinquish the CPU,
	 * to stop busywaiting and go to sleep.
	 */
	*cpu = get_cpu();
	t = local_clock();
	put_cpu();

	return t;
}

static bool
busy_wait_stop(struct task_struct *tsk, unsigned long timeout_ns, unsigned int cpu)
{
	unsigned int this_cpu;

	/* Interrupted by the user already? */
	if (signal_pending(tsk))
		return true;

	if (time_after(local_clock_ns(&this_cpu), timeout_ns))
		return true;

	/*
	 * Check if we were preempted off the cpu, or if something else is
	 * ready to run.  We don't immediately yield in that case, i.e. using
	 * need_resched() instead of cond_resched(), as we want to set up our
	 * interrupt prior to calling schedule()
	 */
	return this_cpu != cpu || need_resched();
}

static bool busy_wait(const struct ufence_wake *wake, unsigned long timeout_ns)
{
	unsigned int cpu;

	/*
	 * Busywait for the fence completion until the end of the user's
	 * timeslice.
	 *
	 * Normally for requests we try to limit busywaits to only when
	 * we expect the request to complete in the near future. With
	 * a plain memory address, we have no execution flow associated
	 * with the userfence and so simply wait until we see the fence
	 * signaled or get kicked off the cpu.
	 */
	timeout_ns += local_clock_ns(&cpu);
	do {
		if (ufence_compare(wake))
			return true;
	} while (!busy_wait_stop(wake->tsk, timeout_ns, cpu));

	return false;
}

int i915_gem_wait_user_fence_ioctl(struct drm_device *dev,
				   void *data, struct drm_file *file)
{
	struct prelim_drm_i915_gem_wait_user_fence *arg = data;
	struct i915_gem_context *ctx = NULL;
	struct engine_wait g_wait, c_wait;
	struct ufence_wake wake;
	unsigned long timeout;
	ktime_t start;
	int err;

	if (arg->flags & ~(PRELIM_I915_UFENCE_WAIT_SOFT | PRELIM_I915_UFENCE_WAIT_ABSTIME))
		return -EINVAL;

	switch (arg->op) {
	case PRELIM_I915_UFENCE_WAIT_EQ:
	case PRELIM_I915_UFENCE_WAIT_NEQ:
	case PRELIM_I915_UFENCE_WAIT_GT:
	case PRELIM_I915_UFENCE_WAIT_GTE:
	case PRELIM_I915_UFENCE_WAIT_LT:
	case PRELIM_I915_UFENCE_WAIT_LTE:
	case PRELIM_I915_UFENCE_WAIT_AFTER:
	case PRELIM_I915_UFENCE_WAIT_BEFORE:
		break;

	default:
		return -EINVAL;
	}

	wake.width = fls64(arg->mask);
	if (!wake.width)
		return -EINVAL;

	/* Restrict the user address to be "naturally" aligned */
	wake.width = DIV_ROUND_UP(roundup_pow_of_two(wake.width), 8);
	if (!IS_ALIGNED(arg->addr, wake.width))
		return -EINVAL;

	/* Natural alignment means the address cannot cross a page boundary */
	GEM_BUG_ON(arg->addr >> PAGE_SHIFT !=
		   (arg->addr + wake.width - 1) >> PAGE_SHIFT);

	wake.tsk = current;
	wake.value = arg->value;
	wake.mask = arg->mask;
	wake.op = arg->op;
	wake.ptr = u64_to_user_ptr(arg->addr);

	if (ufence_fault(&wake))
		return -EFAULT;

	if (ufence_compare(&wake))
		return 0;

	if (!(arg->flags & PRELIM_I915_UFENCE_WAIT_SOFT)) {
		ctx = i915_gem_context_lookup(file->driver_priv, arg->ctx_id);
		if (!ctx)
			return -ENOENT;
	}

	err = i915_user_extensions(u64_to_user_ptr(arg->extensions),
				   NULL, 0, &wake);
	if (err)
		goto out_ctx;

	timeout = to_wait_timeout(arg);
	if (!timeout) {
		err = -ETIME;
		goto out_ctx;
	}
	start = ktime_get();

	/*
	 * In order to avoid interrupt latency (and the wakeup from sleep
	 * penalty for a C-state), busy wait for a short interval first.
	 * If the fence is due to be signalled in ~100us, then an interrupt
	 * latency of ~1ms significantly affects the fence duration and client
	 * latency/throughput. However, if the fence will not be signaled
	 * for another 100ms, then a 1ms delay on top of that for waking up
	 * from a deep sleep is less impactful. Therefore a short spin
	 * before sleeping benefits the most sensitive fence signaling,
	 * and is the most impactful for userspace measurements. We also
	 * know then it is not worth spinning for much longer than the typical
	 * wakeup penalty as any wait longer than that is less affected by
	 * the additional wakeup latency.
	 *
	 * In practice, we spin for the completion of the timeslice, yielding
	 * to any other work ready to run to try to avoid stealing the CPU
	 * from useful work.
	 */
	if (busy_wait(&wake, jiffies_to_nsecs(min(2ul, timeout))))
		goto out_time;

	add_soft_wait(&to_i915(dev)->user_fence_wq, &g_wait);
	if (ctx) {
		add_soft_wait(&ctx->user_fence_wq, &c_wait);
		g_wait.next = &c_wait;

		err = add_gt_wait(ctx, &c_wait.next);
		if (err)
			goto out_wait;

		dma_latency_boost();
	}

	for (;;) {
		set_current_state(TASK_INTERRUPTIBLE);

		if (ufence_compare(&wake))
			break;

		if (ctx && i915_gem_context_is_banned(ctx)) {
			err = -EIO;
			break;
		}

		if (ctx && i915_gem_context_is_closed(ctx)) {
			err = -ENOENT;
			break;
		}

		if (!timeout) {
			err = -ETIME;
			break;
		}

		if (signal_pending(wake.tsk)) {
			err = -ERESTARTSYS;
			break;
		}

		timeout = io_schedule_timeout(timeout);

		if (ufence_fault(&wake)) {
			err = -EFAULT;
			break;
		}
	}
	__set_current_state(TASK_RUNNING);

	if (ctx)
		dma_latency_cancel_boost();
out_wait:
	remove_waits(&g_wait);
out_time:
	if (!(arg->flags & PRELIM_I915_UFENCE_WAIT_ABSTIME) && arg->timeout > 0) {
		arg->timeout -= ktime_to_ns(ktime_sub(ktime_get(), start));
		if (arg->timeout < 0)
			arg->timeout = 0;

		/*
		 * Apparently ktime isn't accurate enough and occasionally has a
		 * bit of mismatch in the jiffies<->nsecs<->ktime loop. So patch
		 * things up to make the test happy. We allow up to 1 jiffy.
		 *
		 * This is a regression from the timespec->ktime conversion.
		 */
		if (err == -ETIME && !nsecs_to_jiffies(arg->timeout))
			arg->timeout = 0;

		/* Asked to wait beyond the jiffie/scheduler precision? */
		if (err == -ETIME && arg->timeout)
			err = -EAGAIN;
	}
out_ctx:
	if (ctx)
		i915_gem_context_put(ctx);
	return err;
}
