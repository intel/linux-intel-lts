/* SPDX-License-Identifier: GPL-2.0 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM i915

#if !defined(_I915_TRACE_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _I915_TRACE_H_

#include <linux/stringify.h>
#include <linux/string_helpers.h>
#include <linux/types.h>
#include <linux/tracepoint.h>

#include <drm/drm_drv.h>

#include "gt/intel_engine.h"
#include "gt/intel_engine_user.h"
#include "gt/intel_gt_regs.h"
#include "gt/intel_pagefault.h"

#include "i915_drv.h"
#include "i915_irq.h"

/* object tracking */

TRACE_EVENT(i915_gem_object_create,
	    TP_PROTO(struct drm_i915_gem_object *obj, u64 nsegments),
	    TP_ARGS(obj, nsegments),

	    TP_STRUCT__entry(
			     __field(struct drm_i915_gem_object *, obj)
			     __field(u64, size)
			     __field(u64, nsegments)
			     ),

	    TP_fast_assign(
			   __entry->obj = obj;
			   __entry->size = obj->base.size;
			   __entry->nsegments = nsegments;
			   ),

	    TP_printk("obj=%p, segments=%llu size=0x%llx", __entry->obj,
		      __entry->nsegments, __entry->size)
);

TRACE_EVENT(i915_dma_buf_attach,
	    TP_PROTO(struct drm_i915_gem_object *obj, unsigned int fabric, int dist),
	    TP_ARGS(obj, fabric, dist),

	    TP_STRUCT__entry(
			     __field(struct drm_i915_gem_object *, obj)
			     __field(bool, lmem)
			     __field(unsigned int, fabric)
			     __field(int, distance)
			     ),

	    TP_fast_assign(
			   __entry->obj = obj;
			   __entry->lmem = i915_gem_object_is_lmem(obj);
			   __entry->fabric = fabric;
			   __entry->distance = dist;
			   ),

	    TP_printk("obj=%p, lmem=%d, fabric=%d p2p distance=%d",
		      __entry->obj, __entry->lmem, __entry->fabric,
		      __entry->distance)
);

TRACE_EVENT(i915_gem_shrink,
	    TP_PROTO(struct drm_i915_private *i915, unsigned long target, unsigned flags),
	    TP_ARGS(i915, target, flags),

	    TP_STRUCT__entry(
			     __field(int, dev)
			     __field(unsigned long, target)
			     __field(unsigned, flags)
			     ),

	    TP_fast_assign(
			   __entry->dev = i915->drm.primary->index;
			   __entry->target = target;
			   __entry->flags = flags;
			   ),

	    TP_printk("dev=%d, target=%lu, flags=%x",
		      __entry->dev, __entry->target, __entry->flags)
);

TRACE_EVENT(i915_vma_bind,
	    TP_PROTO(struct i915_vma *vma, unsigned flags),
	    TP_ARGS(vma, flags),

	    TP_STRUCT__entry(
			     __field(struct drm_i915_gem_object *, obj)
			     __field(struct i915_address_space *, vm)
			     __field(u64, offset)
			     __field(u64, size)
			     __field(unsigned, flags)
			     ),

	    TP_fast_assign(
			   __entry->obj = vma->obj;
			   __entry->vm = vma->vm;
			   __entry->offset = vma->node.start;
			   __entry->size = vma->node.size;
			   __entry->flags = flags;
			   ),

	    TP_printk("obj=%p, offset=0x%016llx size=0x%llx%s vm=%p",
		      __entry->obj, __entry->offset, __entry->size,
		      __entry->flags & PIN_MAPPABLE ? ", mappable" : "",
		      __entry->vm)
);

TRACE_EVENT(i915_vma_unbind,
	    TP_PROTO(struct i915_vma *vma),
	    TP_ARGS(vma),

	    TP_STRUCT__entry(
			     __field(struct drm_i915_gem_object *, obj)
			     __field(struct i915_address_space *, vm)
			     __field(u64, offset)
			     __field(u64, size)
			     ),

	    TP_fast_assign(
			   __entry->obj = vma->obj;
			   __entry->vm = vma->vm;
			   __entry->offset = vma->node.start;
			   __entry->size = vma->node.size;
			   ),

	    TP_printk("obj=%p, offset=0x%016llx size=0x%llx vm=%p",
		      __entry->obj, __entry->offset, __entry->size, __entry->vm)
);

TRACE_EVENT(i915_gem_object_pwrite,
	    TP_PROTO(struct drm_i915_gem_object *obj, u64 offset, u64 len),
	    TP_ARGS(obj, offset, len),

	    TP_STRUCT__entry(
			     __field(struct drm_i915_gem_object *, obj)
			     __field(u64, offset)
			     __field(u64, len)
			     ),

	    TP_fast_assign(
			   __entry->obj = obj;
			   __entry->offset = offset;
			   __entry->len = len;
			   ),

	    TP_printk("obj=%p, offset=0x%llx, len=0x%llx",
		      __entry->obj, __entry->offset, __entry->len)
);

TRACE_EVENT(i915_gem_object_pread,
	    TP_PROTO(struct drm_i915_gem_object *obj, u64 offset, u64 len),
	    TP_ARGS(obj, offset, len),

	    TP_STRUCT__entry(
			     __field(struct drm_i915_gem_object *, obj)
			     __field(u64, offset)
			     __field(u64, len)
			     ),

	    TP_fast_assign(
			   __entry->obj = obj;
			   __entry->offset = offset;
			   __entry->len = len;
			   ),

	    TP_printk("obj=%p, offset=0x%llx, len=0x%llx",
		      __entry->obj, __entry->offset, __entry->len)
);

TRACE_EVENT(i915_gem_object_fault,
	    TP_PROTO(struct drm_i915_gem_object *obj, unsigned long addr, u64 index, bool gtt, bool write),
	    TP_ARGS(obj, addr, index, gtt, write),

	    TP_STRUCT__entry(
			     __field(struct drm_i915_gem_object *, obj)
			     __field(unsigned long, addr)
			     __field(u64, index)
			     __field(bool, gtt)
			     __field(bool, write)
			     ),

	    TP_fast_assign(
			   __entry->obj = obj;
			   __entry->addr = addr;
			   __entry->index = index;
			   __entry->gtt = gtt;
			   __entry->write = write;
			   ),

	    TP_printk("CPU page fault on obj=%p, %s address %lx (page index=%llu) %s",
		      __entry->obj,
		      __entry->gtt ? "GTT" : "CPU",
		      __entry->addr,
		      __entry->index,
		      __entry->write ? ", writable" : "")
);

DECLARE_EVENT_CLASS(i915_gem_object,
	    TP_PROTO(struct drm_i915_gem_object *obj),
	    TP_ARGS(obj),

	    TP_STRUCT__entry(
			     __field(struct drm_i915_gem_object *, obj)
			     ),

	    TP_fast_assign(
			   __entry->obj = obj;
			   ),

	    TP_printk("obj=%p", __entry->obj)
);

DEFINE_EVENT(i915_gem_object, i915_gem_object_clflush,
	     TP_PROTO(struct drm_i915_gem_object *obj),
	     TP_ARGS(obj)
);

DEFINE_EVENT(i915_gem_object, i915_gem_object_destroy,
	    TP_PROTO(struct drm_i915_gem_object *obj),
	    TP_ARGS(obj)
);

TRACE_EVENT(i915_gem_evict,
	    TP_PROTO(struct i915_address_space *vm, u64 size, u64 align, unsigned int flags),
	    TP_ARGS(vm, size, align, flags),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     __field(struct i915_address_space *, vm)
			     __field(u64, size)
			     __field(u64, align)
			     __field(unsigned int, flags)
			    ),

	    TP_fast_assign(
			   __entry->dev = vm->i915->drm.primary->index;
			   __entry->vm = vm;
			   __entry->size = size;
			   __entry->align = align;
			   __entry->flags = flags;
			  ),

	    TP_printk("dev=%d, vm=%p, size=0x%llx, align=0x%llx %s",
		      __entry->dev, __entry->vm, __entry->size, __entry->align,
		      __entry->flags & PIN_MAPPABLE ? ", mappable" : "")
);

TRACE_EVENT(i915_gem_evict_node,
	    TP_PROTO(struct i915_address_space *vm, struct drm_mm_node *node, unsigned int flags),
	    TP_ARGS(vm, node, flags),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     __field(struct i915_address_space *, vm)
			     __field(u64, start)
			     __field(u64, size)
			     __field(unsigned long, color)
			     __field(unsigned int, flags)
			    ),

	    TP_fast_assign(
			   __entry->dev = vm->i915->drm.primary->index;
			   __entry->vm = vm;
			   __entry->start = node->start;
			   __entry->size = node->size;
			   __entry->color = node->color;
			   __entry->flags = flags;
			  ),

	    TP_printk("dev=%d, vm=%p, start=0x%llx size=0x%llx, color=0x%lx, flags=%x",
		      __entry->dev, __entry->vm,
		      __entry->start, __entry->size,
		      __entry->color, __entry->flags)
);

TRACE_EVENT(i915_gem_evict_vm,
	    TP_PROTO(struct i915_address_space *vm),
	    TP_ARGS(vm),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     __field(struct i915_address_space *, vm)
			    ),

	    TP_fast_assign(
			   __entry->dev = vm->i915->drm.primary->index;
			   __entry->vm = vm;
			  ),

	    TP_printk("dev=%d, vm=%p", __entry->dev, __entry->vm)
);

TRACE_EVENT(i915_request_queue,
	    TP_PROTO(struct i915_request *rq, u32 flags),
	    TP_ARGS(rq, flags),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     __field(u64, ctx)
			     __field(u16, class)
			     __field(u16, instance)
			     __field(u32, seqno)
			     __field(u32, flags)
			     ),

	    TP_fast_assign(
			   __entry->dev = rq->engine ? rq->engine->i915->drm.primary->index : 0;
			   __entry->class = rq->engine ? rq->engine->uabi_class : 0;
			   __entry->instance = rq->engine ? rq->engine->uabi_instance : 0;
			   __entry->ctx = rq->fence.context;
			   __entry->seqno = i915_request_seqno(rq);
			   __entry->flags = flags;
			   ),

	    TP_printk("dev=%u, engine=%u:%u, ctx=%llu, seqno=%u, flags=0x%x",
		      __entry->dev, __entry->class, __entry->instance,
		      __entry->ctx, __entry->seqno, __entry->flags)
);

DECLARE_EVENT_CLASS(i915_request,
	    TP_PROTO(struct i915_request *rq),
	    TP_ARGS(rq),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     __field(u64, ctx)
			     __field(u32, guc_id)
			     __field(u16, class)
			     __field(u16, instance)
			     __field(u32, seqno)
			     __field(u32, tail)
			     ),

	    TP_fast_assign(
			   __entry->dev = rq->engine->i915->drm.primary->index;
			   __entry->class = rq->engine->uabi_class;
			   __entry->instance = rq->engine->uabi_instance;
			   __entry->guc_id = rq->context->guc_id.id;
			   __entry->ctx = rq->fence.context;
			   __entry->seqno = i915_request_seqno(rq);
			   __entry->tail = rq->tail;
			   ),

	    TP_printk("dev=%u, engine=%u:%u, guc_id=%u, ctx=%llu, seqno=%u, tail=%u",
		      __entry->dev, __entry->class, __entry->instance,
		      __entry->guc_id, __entry->ctx, __entry->seqno,
		      __entry->tail)
);

DEFINE_EVENT(i915_request, i915_request_add,
	     TP_PROTO(struct i915_request *rq),
	     TP_ARGS(rq)
);

#if defined(CONFIG_DRM_I915_LOW_LEVEL_TRACEPOINTS)
DEFINE_EVENT(i915_request, i915_request_guc_submit,
	     TP_PROTO(struct i915_request *rq),
	     TP_ARGS(rq)
);

DEFINE_EVENT(i915_request, i915_request_submit,
	     TP_PROTO(struct i915_request *rq),
	     TP_ARGS(rq)
);

DEFINE_EVENT(i915_request, i915_request_execute,
	     TP_PROTO(struct i915_request *rq),
	     TP_ARGS(rq)
);

TRACE_EVENT(i915_request_in,
	    TP_PROTO(struct i915_request *rq, unsigned int port),
	    TP_ARGS(rq, port),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     __field(u64, ctx)
			     __field(u16, class)
			     __field(u16, instance)
			     __field(u32, seqno)
			     __field(u32, port)
			     __field(s32, prio)
			    ),

	    TP_fast_assign(
			   __entry->dev = rq->engine->i915->drm.primary->index;
			   __entry->class = rq->engine->uabi_class;
			   __entry->instance = rq->engine->uabi_instance;
			   __entry->ctx = rq->fence.context;
			   __entry->seqno = i915_request_seqno(rq);
			   __entry->prio = rq->sched.attr.priority;
			   __entry->port = port;
			   ),

	    TP_printk("dev=%u, engine=%u:%u, ctx=%llu, seqno=%u, prio=%d, port=%u",
		      __entry->dev, __entry->class, __entry->instance,
		      __entry->ctx, __entry->seqno,
		      __entry->prio, __entry->port)
);

TRACE_EVENT(i915_request_out,
	    TP_PROTO(struct i915_request *rq),
	    TP_ARGS(rq),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     __field(u64, ctx)
			     __field(u16, class)
			     __field(u16, instance)
			     __field(u32, seqno)
			     __field(u32, completed)
			    ),

	    TP_fast_assign(
			   __entry->dev = rq->engine->i915->drm.primary->index;
			   __entry->class = rq->engine->uabi_class;
			   __entry->instance = rq->engine->uabi_instance;
			   __entry->ctx = rq->fence.context;
			   __entry->seqno = i915_request_seqno(rq);
			   __entry->completed = i915_request_completed(rq);
			   ),

		    TP_printk("dev=%u, engine=%u:%u, ctx=%llu, seqno=%u, completed?=%u",
			      __entry->dev, __entry->class, __entry->instance,
			      __entry->ctx, __entry->seqno, __entry->completed)
);

DECLARE_EVENT_CLASS(intel_context,
		    TP_PROTO(struct intel_context *ce),
		    TP_ARGS(ce),

		    TP_STRUCT__entry(
			     __field(u32, guc_id)
			     __field(int, pin_count)
			     __field(u32, sched_state)
			     __field(u8, guc_prio)
			     ),

		    TP_fast_assign(
			   __entry->guc_id = ce->guc_id.id;
			   __entry->pin_count = atomic_read(&ce->pin_count);
			   __entry->sched_state = ce->guc_state.sched_state;
			   __entry->guc_prio = ce->guc_state.prio;
			   ),

		    TP_printk("guc_id=%d, pin_count=%d sched_state=0x%x, guc_prio=%u",
			      __entry->guc_id, __entry->pin_count,
			      __entry->sched_state,
			      __entry->guc_prio)
);

DEFINE_EVENT(intel_context, intel_context_set_prio,
	     TP_PROTO(struct intel_context *ce),
	     TP_ARGS(ce)
);

DEFINE_EVENT(intel_context, intel_context_reset,
	     TP_PROTO(struct intel_context *ce),
	     TP_ARGS(ce)
);

DEFINE_EVENT(intel_context, intel_context_ban,
	     TP_PROTO(struct intel_context *ce),
	     TP_ARGS(ce)
);

DEFINE_EVENT(intel_context, intel_context_register,
	     TP_PROTO(struct intel_context *ce),
	     TP_ARGS(ce)
);

DEFINE_EVENT(intel_context, intel_context_deregister,
	     TP_PROTO(struct intel_context *ce),
	     TP_ARGS(ce)
);

DEFINE_EVENT(intel_context, intel_context_deregister_done,
	     TP_PROTO(struct intel_context *ce),
	     TP_ARGS(ce)
);

DEFINE_EVENT(intel_context, intel_context_sched_enable,
	     TP_PROTO(struct intel_context *ce),
	     TP_ARGS(ce)
);

DEFINE_EVENT(intel_context, intel_context_sched_disable,
	     TP_PROTO(struct intel_context *ce),
	     TP_ARGS(ce)
);

DEFINE_EVENT(intel_context, intel_context_sched_done,
	     TP_PROTO(struct intel_context *ce),
	     TP_ARGS(ce)
);

DEFINE_EVENT(intel_context, intel_context_create,
	     TP_PROTO(struct intel_context *ce),
	     TP_ARGS(ce)
);

DEFINE_EVENT(intel_context, intel_context_fence_release,
	     TP_PROTO(struct intel_context *ce),
	     TP_ARGS(ce)
);

DEFINE_EVENT(intel_context, intel_context_free,
	     TP_PROTO(struct intel_context *ce),
	     TP_ARGS(ce)
);

DEFINE_EVENT(intel_context, intel_context_steal_guc_id,
	     TP_PROTO(struct intel_context *ce),
	     TP_ARGS(ce)
);

DEFINE_EVENT(intel_context, intel_context_do_pin,
	     TP_PROTO(struct intel_context *ce),
	     TP_ARGS(ce)
);

DEFINE_EVENT(intel_context, intel_context_do_unpin,
	     TP_PROTO(struct intel_context *ce),
	     TP_ARGS(ce)
);

#else
#if !defined(TRACE_HEADER_MULTI_READ)
static inline void
trace_i915_request_guc_submit(struct i915_request *rq)
{
}

static inline void
trace_i915_request_submit(struct i915_request *rq)
{
}

static inline void
trace_i915_request_execute(struct i915_request *rq)
{
}

static inline void
trace_i915_request_in(struct i915_request *rq, unsigned int port)
{
}

static inline void
trace_i915_request_out(struct i915_request *rq)
{
}

static inline void
trace_intel_context_set_prio(struct intel_context *ce)
{
}

static inline void
trace_intel_context_reset(struct intel_context *ce)
{
}

static inline void
trace_intel_context_ban(struct intel_context *ce)
{
}

static inline void
trace_intel_context_register(struct intel_context *ce)
{
}

static inline void
trace_intel_context_deregister(struct intel_context *ce)
{
}

static inline void
trace_intel_context_deregister_done(struct intel_context *ce)
{
}

static inline void
trace_intel_context_sched_enable(struct intel_context *ce)
{
}

static inline void
trace_intel_context_sched_disable(struct intel_context *ce)
{
}

static inline void
trace_intel_context_sched_done(struct intel_context *ce)
{
}

static inline void
trace_intel_context_create(struct intel_context *ce)
{
}

static inline void
trace_intel_context_fence_release(struct intel_context *ce)
{
}

static inline void
trace_intel_context_free(struct intel_context *ce)
{
}

static inline void
trace_intel_context_steal_guc_id(struct intel_context *ce)
{
}

static inline void
trace_intel_context_do_pin(struct intel_context *ce)
{
}

static inline void
trace_intel_context_do_unpin(struct intel_context *ce)
{
}
#endif
#endif

DEFINE_EVENT(i915_request, i915_request_retire,
	    TP_PROTO(struct i915_request *rq),
	    TP_ARGS(rq)
);

TRACE_EVENT(i915_request_wait_begin,
	    TP_PROTO(struct i915_request *rq, unsigned int flags),
	    TP_ARGS(rq, flags),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     __field(u64, ctx)
			     __field(u16, class)
			     __field(u16, instance)
			     __field(u32, seqno)
			     __field(unsigned int, flags)
			     ),

	    /* NB: the blocking information is racy since mutex_is_locked
	     * doesn't check that the current thread holds the lock. The only
	     * other option would be to pass the boolean information of whether
	     * or not the class was blocking down through the stack which is
	     * less desirable.
	     */
	    TP_fast_assign(
			   __entry->dev = rq->engine->i915->drm.primary->index;
			   __entry->class = rq->engine->uabi_class;
			   __entry->instance = rq->engine->uabi_instance;
			   __entry->ctx = rq->fence.context;
			   __entry->seqno = i915_request_seqno(rq);
			   __entry->flags = flags;
			   ),

	    TP_printk("dev=%u, engine=%u:%u, ctx=%llu, seqno=%u, flags=0x%x",
		      __entry->dev, __entry->class, __entry->instance,
		      __entry->ctx, __entry->seqno,
		      __entry->flags)
);

DEFINE_EVENT(i915_request, i915_request_wait_end,
	    TP_PROTO(struct i915_request *rq),
	    TP_ARGS(rq)
);

TRACE_EVENT_CONDITION(i915_reg_rw,
	TP_PROTO(bool write, i915_reg_t reg, u64 val, int len, bool trace),

	TP_ARGS(write, reg, val, len, trace),

	TP_CONDITION(trace),

	TP_STRUCT__entry(
		__field(u64, val)
		__field(u32, reg)
		__field(u16, write)
		__field(u16, len)
		),

	TP_fast_assign(
		__entry->val = (u64)val;
		__entry->reg = i915_mmio_reg_offset(reg);
		__entry->write = write;
		__entry->len = len;
		),

	TP_printk("%s reg=0x%x, len=%d, val=(0x%x, 0x%x)",
		__entry->write ? "write" : "read",
		__entry->reg, __entry->len,
		(u32)(__entry->val & 0xffffffff),
		(u32)(__entry->val >> 32))
);

TRACE_EVENT(intel_gpu_freq_change,
	    TP_PROTO(u32 freq),
	    TP_ARGS(freq),

	    TP_STRUCT__entry(
			     __field(u32, freq)
			     ),

	    TP_fast_assign(
			   __entry->freq = freq;
			   ),

	    TP_printk("new_freq=%u", __entry->freq)
);

TRACE_EVENT(i915_eu_stall_cntr_read,
	    TP_PROTO(u8 slice, u8 subslice,
		     u32 read_ptr, u32 write_ptr,
		     u32 read_offset, u32 write_offset,
		     size_t total_size),
	    TP_ARGS(slice, subslice, read_ptr, write_ptr,
		    read_offset, write_offset, total_size),

	    TP_STRUCT__entry(
			     __field(u8, slice)
			     __field(u8, subslice)
			     __field(u32, read_ptr)
			     __field(u32, write_ptr)
			     __field(u32, read_offset)
			     __field(u32, write_offset)
			     __field(size_t, total_size)
			     ),

	    TP_fast_assign(
			   __entry->slice = slice;
			   __entry->subslice = subslice;
			   __entry->read_ptr = read_ptr;
			   __entry->write_ptr = write_ptr;
			   __entry->read_offset = read_offset;
			   __entry->write_offset = write_offset;
			   __entry->total_size = total_size;
			   ),

	    TP_printk("slice:%u subslice:%u readptr:0x%x writeptr:0x%x read off:%u write off:%u size:%zu ",
		      __entry->slice, __entry->subslice,
		      __entry->read_ptr, __entry->write_ptr,
		      __entry->read_offset, __entry->write_offset,
		      __entry->total_size)
);

/**
 * DOC: i915_ppgtt_create and i915_ppgtt_release tracepoints
 *
 * With full ppgtt enabled each process using drm will allocate at least one
 * translation table. With these traces it is possible to keep track of the
 * allocation and of the lifetime of the tables; this can be used during
 * testing/debug to verify that we are not leaking ppgtts.
 * These traces identify the ppgtt through the vm pointer, which is also printed
 * by the i915_vma_bind and i915_vma_unbind tracepoints.
 */
DECLARE_EVENT_CLASS(i915_ppgtt,
	TP_PROTO(struct i915_address_space *vm),
	TP_ARGS(vm),

	TP_STRUCT__entry(
			__field(struct i915_address_space *, vm)
			__field(u32, dev)
	),

	TP_fast_assign(
			__entry->vm = vm;
			__entry->dev = vm->i915->drm.primary->index;
	),

	TP_printk("dev=%u, vm=%p", __entry->dev, __entry->vm)
)

DEFINE_EVENT(i915_ppgtt, i915_ppgtt_create,
	TP_PROTO(struct i915_address_space *vm),
	TP_ARGS(vm)
);

DEFINE_EVENT(i915_ppgtt, i915_ppgtt_release,
	TP_PROTO(struct i915_address_space *vm),
	TP_ARGS(vm)
);

/**
 * DOC: i915_context_create and i915_context_free tracepoints
 *
 * These tracepoints are used to track creation and deletion of contexts.
 * If full ppgtt is enabled, they also print the address of the vm assigned to
 * the context.
 */
DECLARE_EVENT_CLASS(i915_context,
	TP_PROTO(struct i915_gem_context *ctx),
	TP_ARGS(ctx),

	TP_STRUCT__entry(
			__field(u32, dev)
			__field(struct i915_gem_context *, ctx)
			__field(struct i915_address_space *, vm)
	),

	TP_fast_assign(
			__entry->dev = ctx->i915->drm.primary->index;
			__entry->ctx = ctx;
			__entry->vm = rcu_access_pointer(ctx->vm);
	),

	TP_printk("dev=%u, ctx=%p, ctx_vm=%p",
		  __entry->dev, __entry->ctx, __entry->vm)
)

DEFINE_EVENT(i915_context, i915_context_create,
	TP_PROTO(struct i915_gem_context *ctx),
	TP_ARGS(ctx)
);

DEFINE_EVENT(i915_context, i915_context_free,
	TP_PROTO(struct i915_gem_context *ctx),
	TP_ARGS(ctx)
);

TRACE_EVENT(intel_gt_cat_error,
	    TP_PROTO(struct intel_gt *gt, const char *guc_ctx_id),
	    TP_ARGS(gt, guc_ctx_id),

	    TP_STRUCT__entry(
			     __field(struct intel_gt *, gt)
			     __array(char, guc_ctx_id, 11)
			     ),

	    TP_fast_assign(
			   __entry->gt = gt;
			   strscpy(__entry->guc_ctx_id, guc_ctx_id, 11);
			   ),

	    TP_printk("GPU catastrophic memory error. GT: %d, GuC context: %s",
		      __entry->gt->info.id,
		      __entry->guc_ctx_id)
);

TRACE_EVENT(intel_gt_pagefault,
	    TP_PROTO(struct intel_gt *gt, u64 address, u32 fault_reg, bool is_ggtt),
	    TP_ARGS(gt, address, fault_reg, is_ggtt),

	    TP_STRUCT__entry(
			     __field(struct intel_gt *, gt)
			     __field(u64, address)
			     __field(u32, fault_reg)
			     __field(bool, is_ggtt)
			     ),

	    TP_fast_assign(
			   __entry->gt = gt;
			   __entry->address = address;
			   __entry->fault_reg = fault_reg;
			   __entry->is_ggtt = is_ggtt;
			   ),

	    TP_printk("dev %p: GPU %s fault: GT: %d, address space %s, address: %#llx, engine ID: %u, source ID: %u, type: %u, fault level: %u",
		      __entry->gt->i915,
		      !!(__entry->fault_reg & RING_FAULT_ACCESS_TYPE) ? "Write" : "Read",
		      __entry->gt->info.id,
		      __entry->is_ggtt ? "GGTT" : "PPGTT",
		      __entry->address,
		      GEN8_RING_FAULT_ENGINE_ID(__entry->fault_reg),
		      RING_FAULT_SRCID(__entry->fault_reg),
		      RING_FAULT_FAULT_TYPE(__entry->fault_reg),
		      RING_FAULT_LEVEL(__entry->fault_reg))
);

TRACE_EVENT(i915_gem_object_migrate,
	    TP_PROTO(struct drm_i915_gem_object *obj,
		     struct intel_memory_region *region),
	    TP_ARGS(obj, region),

	    TP_STRUCT__entry(
			     __field(int, dev)
			     __field(struct drm_i915_gem_object *, obj)
			     __field(u64, size)
			     __array(char, src, sizeof(((struct intel_memory_region *)0)->name))
			     __array(char, dst, sizeof(((struct intel_memory_region *)0)->name))
			     __field(bool, has_pages)
			     ),

	    TP_fast_assign(
			   __entry->dev = obj->base.dev->primary->index;
			   __entry->obj = obj;
			   __entry->size = obj->base.size;
			   strcpy(__entry->src, obj->mm.region.mem->name);
			   strcpy(__entry->dst, region->name);
			   __entry->has_pages = i915_gem_object_has_pages(obj);
			   ),

	    TP_printk("dev %d migrate object %p [size %llx] %s %s from %s to %s",
		      __entry->dev, __entry->obj, __entry->size,
		      __entry->has_pages ? "with" : "without", "backing storage",
		      __entry->src, __entry->dst)
);

TRACE_EVENT(i915_mm_fault,
	    TP_PROTO(struct i915_address_space *vm,
		     struct i915_vma *vma,
		     struct recoverable_page_fault_info *info),
	    TP_ARGS(vm, vma, info),

	    TP_STRUCT__entry(
			     __field(int, dev)
			     __field(const struct i915_address_space *, vm)
			     __field(const struct drm_i915_gem_object *, obj)
			     __array(char, region, sizeof(((struct intel_memory_region *)0)->name))
			     __field(u64, obj_size)
			     __field(u64, addr)
			     __field(u64, vma_size)
			     __field(u32, asid)
			     __field(u32, pg_sz_mask)
			     __field(u16, pdata)
			     __field(u8, access_type)
			     __field(u8, fault_type)
			     __field(u8, fault_level)
			     __field(u8, engine_class)
			     __field(u8, engine_instance)
			     __field(bool, is_bound)
			     ),

	    TP_fast_assign(
			   __entry->dev = vm->i915->drm.primary->index;
			   __entry->vm = vm;
			   if (vma) {
				   __entry->obj = vma->obj;
				   __entry->obj_size = vma->obj->base.size;
				   strcpy(__entry->region, vma->obj->mm.region.mem->name);
				   __entry->vma_size = i915_vma_size(vma);
				   __entry->pg_sz_mask = vma->page_sizes;
				   __entry->is_bound = i915_vma_is_bound(vma, PIN_USER);
			   } else {
				   __entry->obj = NULL;
				   __entry->obj_size = 0;
				   __entry->vma_size = 0;
				   __entry->pg_sz_mask = 0;
				   __entry->is_bound = false;
				   strcpy(__entry->region, "none");
			   }
			   __entry->addr = info->page_addr;
			   __entry->asid = info->asid;
			   __entry->access_type = info->access_type;
			   __entry->fault_type = info->fault_type;
			   __entry->fault_level = info->fault_level;
			   __entry->engine_class = info->engine_class;
			   __entry->engine_instance = info->engine_instance;
			   __entry->pdata = info->pdata;
			   ),

	    TP_printk("dev %d vm %p [asid %d]: GPU %s fault on %s obj %p [size %lld] address %llx%s size 0x%llx pgsz %x, %s[%d] %d: %s (0x%x)",
		      __entry->dev, __entry->vm, __entry->asid,
		      intel_access_type2str(__entry->access_type),
		      __entry->region,
		      __entry->obj, __entry->obj_size, __entry->addr,
		      __entry->is_bound ? " bound" : "", __entry->vma_size, __entry->pg_sz_mask,
		      intel_engine_class_repr(__entry->engine_class),
		      __entry->engine_instance, __entry->fault_level,
		      intel_pagefault_type2str(__entry->fault_type), __entry->pdata)
);

TRACE_EVENT(intel_tlb_invalidate,
	    TP_PROTO(struct intel_gt *gt, u64 start, u64 len),
	    TP_ARGS(gt, start, len),

	    TP_STRUCT__entry(
			     __field(struct drm_i915_private*, dev)
			     __field(u32, id)
			     __field(u64, start)
			     __field(u64, len)
			     ),

	    TP_fast_assign(
			   __entry->dev = gt->i915;
			   __entry->id = gt->info.id;
			   __entry->start = start;
			   __entry->len = len;
			   ),

	    TP_printk("dev %p gt%d %s TLB invalidation, start %llx len %llx",
		      __entry->dev, __entry->id,
		      (__entry->len) ? "range" : "full",
		      __entry->start, __entry->len)
);

TRACE_EVENT(intel_access_counter,
	    TP_PROTO(struct intel_gt *gt, struct acc_info *info, int err),
	    TP_ARGS(gt, info, err),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     __field(u32, id)
			     __field(u32, region_type)
			     __field(u32, sub_region_hit_vector)
			     __field(u32, asid)
			     __field(u32, engine_class)
			     __field(u32, engine_instance)
			     __field(u32, err)
			     __field(u64, vaddr_base)
			     ),

	    TP_fast_assign(
			   __entry->dev = gt->i915->drm.primary->index;
			   __entry->id = gt->info.id;
			   __entry->region_type = info->granularity;
			   __entry->sub_region_hit_vector = info->sub_granularity;
			   __entry->asid = info->asid;
			   __entry->engine_class = info->engine_class;
			   __entry->engine_instance = info->engine_instance;
			   __entry->err = err;
			   __entry->vaddr_base = info->va_range_base;
			   ),

	    TP_printk("%s dev%u gt%u asid%d %d KB Region/%d KB sub-region %s[%d], VA_BASE: %llx, sub-region hit vector %x",
		      intel_acc_err2str(__entry->err), __entry->dev, __entry->id, __entry->asid,
		      granularity_in_byte(__entry->region_type) / SZ_1K,
		      sub_granularity_in_byte(__entry->region_type) / SZ_1K,
		      intel_engine_class_repr(__entry->engine_class),
		      __entry->engine_instance, __entry->vaddr_base, __entry->sub_region_hit_vector)
);

TRACE_EVENT(i915_vm_prefetch,
	    TP_PROTO(struct intel_memory_region *region, u32 vm_id, u64 start, u64 len),
	    TP_ARGS(region, vm_id, start, len),

	    TP_STRUCT__entry(
			     __field(int, dev)
			     __field(u32, vm_id)
			     __field(u64, start)
			     __field(u64, len)
			     __array(char, region, sizeof(((struct intel_memory_region *)0)->name))
			     ),

	    TP_fast_assign(
			   __entry->dev = region->i915->drm.primary->index;
			   __entry->vm_id = vm_id;
			   __entry->start = start;
			   __entry->len = len;
			   strcpy(__entry->region, region->name);
			   ),

	    TP_printk("dev %d prefetch va start %llx (len %llx) to region %s for vm %d",
		      __entry->dev,
		      __entry->start, __entry->len,
		      __entry->region,
		      __entry->vm_id)
);
#endif /* _I915_TRACE_H_ */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_PATH ../../drivers/gpu/drm/i915
#define TRACE_INCLUDE_FILE i915_trace
#include <trace/define_trace.h>
