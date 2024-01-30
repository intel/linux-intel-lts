// SPDX-License-Identifier: MIT
/*
 * Copyright © 2022 Intel Corporation
 */

#include "gem/i915_gem_lmem.h"
#include "gem/i915_gem_mman.h"
#include "gem/i915_gem_vm_bind.h"

#include "i915_drv.h"
#include "i915_driver.h"
#include "i915_trace.h"

#include "gen8_ppgtt.h"
#include "intel_context.h"
#include "intel_engine_heartbeat.h"
#include "intel_gt.h"
#include "intel_gt_debug.h"
#include "intel_gt_mcr.h"
#include "intel_gt_print.h"
#include "intel_gt_regs.h"
#include "intel_tlb.h"
#include "intel_pagefault.h"
#include "uc/intel_guc.h"
#include "uc/intel_guc_fwif.h"
#include "i915_svm.h"
#include "i915_debugger.h"

/**
 * DOC: Recoverable page fault implications
 *
 * Modern GPU hardware support recoverable page fault. This has extensive
 * implications to driver implementation.
 *
 * DMA fence is used extensively to track object activity for cross-device
 * and cross-application synchronization. But if recoverable page fault is
 * enabled, using of DMA fence can potentially induce deadlock: A pending
 * page fault holds up the GPU work which holds up the dma fence signaling,
 * and memory allocation is usually required to resolve a page fault, but
 * memory allocation is not allowed to gate dma fence signaling.
 *
 * Non-long-run context usually uses DMA fence for GPU job/object completion
 * tracking, thus faultable vm is not allowed for non-long-run context.
 *
 * Suspend fence is used to suspend long run context before we unbind
 * BOs, in case of userptr invalidation, memory shrinking or eviction.
 * For faultable vm, there is no need to use suspend fence: we directly
 * unbind BOs w/o suspend context and BOs will be rebound during a recoverable
 * page fault handling thereafter.
 *
 * DMA fences attached to vm's active are used to track vm's activity.
 * i.e., driver wait on those dma fences for vm to be idle. This method
 * is useful for non-faultable vm. For faultable vm, we don't support
 * any DMA fence because of the deadlock described above. Thus, we can't attach
 * any DMA fences, including suspend fence or request fence, to a faultable vm.
 */

enum fault_type {
	NOT_PRESENT = 0,
	WRITE_ACCESS_VIOLATION = 1,
	ATOMIC_ACCESS_VIOLATION = 2,
};

void intel_gt_pagefault_process_cat_error_msg(struct intel_gt *gt, u32 guc_ctx_id)
{
	char name[TASK_COMM_LEN + 8] = "[" DRIVER_NAME "]";
	struct intel_context *ce;

	ce = xa_load(&gt->uc.guc.context_lookup, guc_ctx_id);
	if (ce && ce->gem_context) {
		memcpy(name, ce->gem_context->name, sizeof(name));
		intel_context_ban(ce, NULL);
	}

	trace_intel_gt_cat_error(gt, name);
	gt_err(gt, "catastrophic memory error in context %s\n", name);
}

static u64 fault_va(u32 fault_data1, u32 fault_data0)
{
	return ((u64)(fault_data1 & FAULT_VA_HIGH_BITS) << GEN12_FAULT_VA_HIGH_SHIFT) |
	       ((u64)fault_data0 << GEN12_FAULT_VA_LOW_SHIFT);
}

int intel_gt_pagefault_process_page_fault_msg(struct intel_gt *gt, const u32 *msg, u32 len)
{
	struct drm_i915_private *i915 = gt->i915;
	u64 address;
	u32 fault_reg, fault_data0, fault_data1;

	if (GRAPHICS_VER(i915) < 12)
		return -EPROTO;

	if (len != GUC2HOST_NOTIFY_PAGE_FAULT_MSG_LEN)
		return -EPROTO;

	if (FIELD_GET(GUC2HOST_NOTIFY_PAGE_FAULT_MSG_0_MBZ, msg[0]) != 0)
		return -EPROTO;

	fault_reg = FIELD_GET(GUC2HOST_NOTIFY_PAGE_FAULT_MSG_1_ALL_ENGINE_FAULT_REG, msg[1]);
	fault_data0 = FIELD_GET(GUC2HOST_NOTIFY_PAGE_FAULT_MSG_2_FAULT_TLB_RD_DATA0, msg[2]);
	fault_data1 = FIELD_GET(GUC2HOST_NOTIFY_PAGE_FAULT_MSG_3_FAULT_TLB_RD_DATA1, msg[3]);

	address = fault_va(fault_data1, fault_data0);

	trace_intel_gt_pagefault(gt, address, fault_reg, fault_data1 & FAULT_GTT_SEL);

	drm_err(&i915->drm, "Unexpected fault\n"
			    "\tGT: %d\n"
			    "\tAddr: 0x%llx\n"
			    "\tAddress space%s\n"
			    "\tEngine ID: %u\n"
			    "\tSource ID: %u\n"
			    "\tType: %u\n"
			    "\tFault Level: %u\n"
			    "\tAccess type: %s\n",
			    gt->info.id,
			    address,
			    fault_data1 & FAULT_GTT_SEL ? "GGTT" : "PPGTT",
			    GEN8_RING_FAULT_ENGINE_ID(fault_reg),
			    RING_FAULT_SRCID(fault_reg),
			    RING_FAULT_FAULT_TYPE(fault_reg),
			    RING_FAULT_LEVEL(fault_reg),
			    !!(fault_reg & RING_FAULT_ACCESS_TYPE) ? "Write" : "Read");

	return 0;
}

static void print_recoverable_fault(struct recoverable_page_fault_info *info,
				    const char *reason, int ret)
{
	DRM_DEBUG_DRIVER("\n\t%s: error %d\n"
			 "\tASID: %d\n"
			 "\tVFID: %d\n"
			 "\tPDATA: 0x%04x\n"
			 "\tFaulted Address: 0x%08x_%08x\n"
			 "\tFaultType: %d\n"
			 "\tAccessType: %d\n"
			 "\tFaultLevel: %d\n"
			 "\tEngineClass: %d\n"
			 "\tEngineInstance: %d\n",
			 reason, ret,
			 info->asid,
			 info->vfid,
			 info->pdata,
			 upper_32_bits(info->page_addr),
			 lower_32_bits(info->page_addr),
			 info->fault_type,
			 info->access_type,
			 info->fault_level,
			 info->engine_class,
			 info->engine_instance);
}

static int migrate_to_lmem(struct drm_i915_gem_object *obj,
			   enum intel_region_id lmem_id,
			   struct i915_gem_ww_ctx *ww)
{
	int ret;

	/* return if object has single placement or already in lmem_id */
	if (!i915_gem_object_migratable(obj) ||
	    obj->mm.region.mem->id == lmem_id)
		return 0;

	/*
	 * FIXME: Move this to BUG_ON later when uapi enforces object alignment
	 * to 64K for objects that can reside on both SMEM and LMEM.
	 */
	if (HAS_64K_PAGES(to_i915(obj->base.dev)) &&
	    !IS_ALIGNED(obj->base.size, I915_GTT_PAGE_SIZE_64K)) {
		DRM_DEBUG_DRIVER("Cannot migrate objects of different page sizes\n");
		return -ENOTSUPP;
	}

	i915_gem_object_release_mmap(obj);
	GEM_BUG_ON(obj->mm.mapping);
	GEM_BUG_ON(obj->base.filp && mapping_mapped(obj->base.filp->f_mapping));

	/* unmap to avoid further update to the page[s] */
	ret = i915_gem_object_unbind(obj, ww, I915_GEM_OBJECT_UNBIND_ACTIVE);
	if (ret)
		return ret;

	return i915_gem_object_migrate(obj, lmem_id, true);
}

static inline bool access_is_atomic(struct recoverable_page_fault_info *info)
{
	return (info->access_type == ACCESS_TYPE_ATOMIC);
}

static enum intel_region_id get_lmem_region_id(struct drm_i915_gem_object *obj, struct intel_gt *gt)
{
	int i;

	if (obj->mm.preferred_region &&
	    obj->mm.preferred_region->type == INTEL_MEMORY_LOCAL)
		return obj->mm.preferred_region->id;

	if (BIT(gt->lmem->id) & obj->memory_mask)
		return gt->lmem->id;

	for (i = 0; i < obj->mm.n_placements; i++) {
		struct intel_memory_region *mr = obj->mm.placements[i];

		if (mr->type == INTEL_MEMORY_LOCAL)
			return mr->id;
	}

	return 0;
}

static int validate_fault(struct drm_i915_private *i915, struct i915_vma *vma,
			  struct recoverable_page_fault_info *info)
{
	/* combined access_type and fault_type */
	enum {
		FAULT_READ_NOT_PRESENT = 0x0,
		FAULT_WRITE_NOT_PRESENT = 0x1,
		FAULT_ATOMIC_NOT_PRESENT = 0x2,
		FAULT_WRITE_ACCESS_VIOLATION = 0x5,
		FAULT_ATOMIC_ACCESS_VIOLATION = 0xa,
	} err_code;
	int err = 0;

	err_code = (info->fault_type << 2) | info->access_type;

	switch (err_code & 0xF) {
	case FAULT_READ_NOT_PRESENT:
		break;
	case FAULT_WRITE_NOT_PRESENT:
		if (i915_gem_object_is_readonly(vma->obj)) {
			drm_err(&i915->drm, "Write Access Violation: read only\n");
			err = -EACCES;
		}
		break;
	case FAULT_ATOMIC_NOT_PRESENT:
		/*
		 * This case is early detection of ATOMIC ACCESS_VIOLATION.
		 *
		 * Imported (dma-buf) objects do not have a memory_mask (or
		 * placement list), so allow the NOT_PRESENT fault to proceed
		 * as we cannot test placement list.
		 * The replayed memory access will catch a true ATOMIC
		 * ACCESS_VIOLATION and fail appropriately.
		 */
		if (!vma->obj->memory_mask)
			break;
		fallthrough;
	case FAULT_ATOMIC_ACCESS_VIOLATION:
		if (!(vma->obj->memory_mask & REGION_LMEM)) {
			drm_err(&i915->drm, "Atomic Access Violation\n");
			err = -EACCES;
		}
		break;
	case FAULT_WRITE_ACCESS_VIOLATION:
		drm_err(&i915->drm, "Write Access Violation\n");
		err = -EACCES;
		break;
	default:
		drm_err(&i915->drm, "Undefined Fault Type\n");
		err = -EACCES;
		break;
	}

	return err;
}

static struct i915_address_space *faulted_vm(struct intel_guc *guc, u32 asid)
{
	if (GEM_WARN_ON(asid >= I915_MAX_ASID))
		return NULL;

	return xa_load(&guc_to_gt(guc)->i915->asid_resv.xa, asid);
}

static struct intel_engine_cs *
lookup_engine(struct intel_gt *gt, u8 class, u8 instance)
{
	if (class >= ARRAY_SIZE(gt->engine_class) ||
	    instance >= ARRAY_SIZE(gt->engine_class[class]))
		return NULL;

	return gt->engine_class[class][instance];
}

static void
mark_engine_as_active(struct intel_gt *gt,
		      int engine_class, int engine_instance)
{
	struct intel_engine_cs *engine;

	engine = lookup_engine(gt, engine_class, engine_instance);
	if (!engine)
		return;

	WRITE_ONCE(engine->stats.irq.count,
		   READ_ONCE(engine->stats.irq.count) + 1);
}

static struct i915_gpu_coredump *
pf_coredump(struct intel_gt *gt, struct recoverable_page_fault_info *info)
{
	struct i915_gpu_coredump *error;
	struct intel_engine_cs *engine;

	engine = lookup_engine(gt, info->engine_class, info->engine_instance);
	if (!engine)
		return NULL;
	GEM_BUG_ON(engine->gt != gt);

	error = i915_gpu_coredump_create_for_engine(engine, GFP_KERNEL);
	if (!error)
		return NULL;

	error->fault.addr = info->page_addr | BIT(0);
	error->fault.type = info->fault_type;
	error->fault.level = info->fault_level;
	error->fault.access = info->access_type;

	rcu_read_lock();
	error->private = intel_engine_find_active_request(engine);
	if (error->private)
		error->private = i915_request_get_rcu(error->private);
	rcu_read_unlock();

	return error;
}

static struct i915_debugger_pagefault *
pf_eu_debugger(struct intel_gt *gt, struct recoverable_page_fault_info *info,
	       struct dma_fence *fence)
{
	struct i915_debugger_pagefault *debugger_pagefault;
	struct intel_engine_cs *engine;

	u32 td_ctl;

	engine = lookup_engine(gt, info->engine_class, info->engine_instance);
	if (!engine)
		return NULL;

	GEM_BUG_ON(engine->gt != gt);

	td_ctl = intel_gt_mcr_read_any(gt, TD_CTL);
	/*
	 * If there is no debug functionality (TD_CTL_GLOBAL_DEBUG_ENABLE, etc.),
	 * don't proceed pagefault routine for eu debugger.
	 */
	if (!td_ctl)
		return NULL;

	debugger_pagefault = kzalloc(sizeof(*debugger_pagefault), GFP_KERNEL);
	if (!debugger_pagefault)
		return NULL;

	/*
	 * XXX only the first fault will try to resolve attn
	 * Typically lots of eu run the same instruction,
	 * the additional page faults might be generated before i915 set TD_CTL
	 * with FEH/FE. And the HW/guc is able to queue a lot of pagefault messages.
	 * If the pagefault handler serializes all pagefaults at this point,
	 * the serialization breaks TD_CTL attn discovery since the thread is
	 * not immediately resumed on the first fault reply.
	 * So while processing pagefault WA, skip processing of followed
	 * HW pagefault event that happens before FEH/FE is set.
	 * Due to this way, hw pagefault events from GuC might not pass
	 * transparently to debugUMD. But the eu thread where the pagefault
	 * occurred is combined into the threads list of page fault events
	 * passed to debugUMD. And as FEH & FE are set, the gpu thread will jump
	 * to SIP, blocking further pagefault occurrences. When FEH/FE is unset
	 * at the end of the page fault handler, additional page faults are
	 * allowed to occur.
	 */
	mutex_lock(&gt->eu_debug.lock);
	if (i915_active_fence_isset(&gt->eu_debug.fault)) {
		mutex_unlock(&gt->eu_debug.lock);
		kfree(debugger_pagefault);
		return NULL;
	}
	__i915_active_fence_set(&gt->eu_debug.fault, fence);
	mutex_unlock(&gt->eu_debug.lock);

	INIT_LIST_HEAD(&debugger_pagefault->list);

	intel_eu_attentions_read(gt, &debugger_pagefault->attentions.before, 0);

	/* Halt on next thread dispatch */
	while (!(td_ctl & TD_CTL_FORCE_EXTERNAL_HALT)) {
		intel_gt_mcr_multicast_write(gt, TD_CTL,
					     td_ctl | TD_CTL_FORCE_EXTERNAL_HALT);
		/*
		 * The sleep is needed because some interrupts are ignored
		 * by the HW, hence we allow the HW some time to acknowledge
		 * that.
		 */
		udelay(200);

		td_ctl = intel_gt_mcr_read_any(gt, TD_CTL);
	}

	/* Halt regardless of thread dependencies */
	while (!(td_ctl & TD_CTL_FORCE_EXCEPTION)) {
		intel_gt_mcr_multicast_write(gt, TD_CTL,
					     td_ctl | TD_CTL_FORCE_EXCEPTION);
		udelay(200);

		td_ctl = intel_gt_mcr_read_any(gt, TD_CTL);
	}

	intel_eu_attentions_read(gt, &debugger_pagefault->attentions.after,
				 INTEL_GT_ATTENTION_TIMEOUT_MS);

	intel_gt_invalidate_l3_mmio(gt);

	debugger_pagefault->engine = engine;
	debugger_pagefault->fault.addr = info->page_addr;
	debugger_pagefault->fault.type = info->fault_type;
	debugger_pagefault->fault.level = info->fault_level;
	debugger_pagefault->fault.access = info->access_type;

	return debugger_pagefault;
}

static bool has_debug_sip(struct intel_gt *gt)
{
	/*
	 * When debugging is enabled, we want to enter the SIP after resolving
	 * the pagefault and read the attention bits from the SIP. In this case,
	 * we must always use a scratch page for the invalid fault so that we
	 * can enter the sip and not retrigger more faults.
	 *
	 * After capturing the attention bits, we can restore the faulting
	 * vma (if required).
	 *
	 * XXX maybe intel_context_has_debug()?
	 */
	return intel_gt_mcr_read_any(gt, TD_CTL);
}

static struct dma_fence *
handle_i915_mm_fault(struct intel_guc *guc,
		     struct recoverable_page_fault_info *info,
		     struct i915_gpu_coredump **dump,
		     struct i915_debugger_pagefault **debugger_pagefault,
		     struct dma_fence *reply_fence)
{
	struct intel_gt *gt = guc_to_gt(guc);
	struct dma_fence *fence = NULL;
	struct i915_address_space *vm;
	enum intel_region_id lmem_id;
	struct i915_gem_ww_ctx ww;
	struct i915_vma *vma;
	int err = 0;

	vm = faulted_vm(guc, info->asid);
	/* The active context [asid] is protected while servicing a fault */
	if (GEM_WARN_ON(!vm))
		return ERR_PTR(-ENOENT);

	if (!i915_vm_page_fault_enabled(vm))
		return ERR_PTR(-ENOENT);

	if (i915_vm_is_svm_enabled(vm))
		return ERR_PTR(i915_svm_handle_gpu_fault(vm, gt, info));

	vma = i915_find_vma(vm, info->page_addr);

	trace_i915_mm_fault(vm, vma, info);

	if (!vma) {
		/* Driver specific implementation of handling PRS */
		if (is_vm_pasid_active(vm)) {
			err = i915_handle_ats_fault_request(vm, info);
			if (err) {
				drm_err(&vm->i915->drm,
					"Handling OS request for device to handle faulting failed error: %d\n",
					err);
				return ERR_PTR(err);
			}
			return 0;
		}

		/* Each EU thread may trigger its own pf to the same address! */
		if (!vm->invalidate_tlb_scratch) {
			struct intel_engine_cs *engine;
			bool debugger_active = false;

			engine = lookup_engine(gt, info->engine_class, info->engine_instance);
			if (engine)
				debugger_active = i915_debugger_active_on_engine(engine);
			/* The crux of this code is the same for offline/online.
			 *
			 * The current differences are that for offline we record
			 * a few more registers (not a bit deal of online) and
			 * that for online we are more careful and protect
			 * concurrent TD_CTL modifications.
			 * The latter safeguard would be an improvement for offline
			 * and the extra mmio reads lost in the noise for online.
			 *
			 * Then during fault_complete we decide if there's
			 * a debugger attached to send the event, or if not
			 * we complete and save the coredump for posterity.
			 */
			if (debugger_active) {
				*debugger_pagefault = pf_eu_debugger(gt, info, reply_fence);
				if (!*debugger_pagefault)
					return NULL;
			} else {
				*dump = pf_coredump(gt, info);
			}
		}

		if (vm->has_scratch || has_debug_sip(gt)) {
			/* Map the out-of-bound access to scratch page.
			 *
			 * Out-of-bound virtual address range is not tracked,
			 * so whenever we bind a new vma we do not know if it
			 * is replacing a scratch mapping, and so we must always
			 * flush the TLB of the vma's address range so that the
			 * next access will not load scratch. Set invalidate_tlb_scratch
			 * flag so we know on next vm_bind.
			 *
			 * This is an exceptional path to ease userspace development.
			 * Once user space fixes all the out-of-bound access, this
			 * logic will be removed.
			 */
			gen12_init_fault_scratch(vm,
						 info->page_addr,
						 BIT(vm->scratch_order + PAGE_SHIFT),
						 true);
			return NULL;
		}

		return ERR_PTR(-EFAULT);
	}

	mark_engine_as_active(gt, info->engine_class, info->engine_instance);

	err = validate_fault(gt->i915, vma, info);
	if (err)
		goto put_vma;

	/*
	 * With lots of concurrency to the same unbound VMA, HW will generate a storm
	 * of page faults. Test this upfront so that the redundant fault requests
	 * return as early as possible.
	 */
	if (i915_vma_is_bound(vma, PIN_RESIDENT))
		goto put_vma;

	i915_gem_ww_ctx_init(&ww, false);

 retry:
	err = i915_gem_object_lock(vma->obj, &ww);
	if (err)
		goto err_ww;

	if (i915_vma_is_bound(vma, PIN_RESIDENT))
		goto err_ww;

	vma->obj->flags |= I915_BO_FAULT_CLEAR | I915_BO_SYNC_HINT;

	lmem_id = get_lmem_region_id(vma->obj, gt);
	if (i915_gem_object_should_migrate_lmem(vma->obj, lmem_id,
						access_is_atomic(info))) {
		err = migrate_to_lmem(vma->obj, lmem_id, &ww);
		/*
		 * Migration is best effort.
		 * if we see -EDEADLK handle that with proper backoff. Otherwise
		 * for scenarios like atomic operation, if migration fails,
		 * gpu will fault again and we can retry.
		 */
		if (err == -EDEADLK)
			goto err_ww;

	}

	err = i915_vma_bind(vma, &ww);
 err_ww:
	if (err == -EDEADLK) {
		err = i915_gem_ww_ctx_backoff(&ww);
		if (!err)
			goto retry;
	}

	fence = i915_active_fence_get_or_error(&vma->active.excl);
	i915_gem_ww_ctx_fini(&ww);

put_vma:
	i915_vma_put(vma);
	__i915_vma_put(vma);

	return fence ?: ERR_PTR(err);
}

static void get_fault_info(const u32 *payload, struct recoverable_page_fault_info *info)
{
	const struct intel_guc_pagefault_desc *desc;

	desc = (const struct intel_guc_pagefault_desc *)payload;

	info->fault_level = FIELD_GET(PAGE_FAULT_DESC_FAULT_LEVEL, desc->dw0);
	info->engine_class = FIELD_GET(PAGE_FAULT_DESC_ENG_CLASS, desc->dw0);
	info->engine_instance = FIELD_GET(PAGE_FAULT_DESC_ENG_INSTANCE, desc->dw0);
	info->pdata = FIELD_GET(PAGE_FAULT_DESC_PDATA_HI,
				desc->dw1) << PAGE_FAULT_DESC_PDATA_HI_SHIFT;
	info->pdata |= FIELD_GET(PAGE_FAULT_DESC_PDATA_LO, desc->dw0);
	info->asid =  FIELD_GET(PAGE_FAULT_DESC_ASID, desc->dw1);
	info->vfid =  FIELD_GET(PAGE_FAULT_DESC_VFID, desc->dw2);
	info->access_type = FIELD_GET(PAGE_FAULT_DESC_ACCESS_TYPE, desc->dw2);
	info->fault_type = FIELD_GET(PAGE_FAULT_DESC_FAULT_TYPE, desc->dw2);
	info->page_addr = (u64)(FIELD_GET(PAGE_FAULT_DESC_VIRTUAL_ADDR_HI,
					  desc->dw3)) << PAGE_FAULT_DESC_VIRTUAL_ADDR_HI_SHIFT;
	info->page_addr |= FIELD_GET(PAGE_FAULT_DESC_VIRTUAL_ADDR_LO,
				     desc->dw2) << PAGE_FAULT_DESC_VIRTUAL_ADDR_LO_SHIFT;
}

struct fault_reply {
	struct dma_fence_work base;
	struct recoverable_page_fault_info info;
	struct i915_gpu_coredump *dump;
	struct i915_debugger_pagefault *debugger_pagefault;
	struct intel_guc *guc;
	struct intel_gt *gt;
	intel_wakeref_t wakeref;
};

static int fault_work(struct dma_fence_work *work)
{
	return 0;
}

static int send_fault_reply(const struct fault_reply *f)
{
	u32 action[] = {
		INTEL_GUC_ACTION_PAGE_FAULT_RES_DESC,

		(FIELD_PREP(PAGE_FAULT_REPLY_VALID, 1) |
		 FIELD_PREP(PAGE_FAULT_REPLY_SUCCESS,
			    f->info.fault_unsuccessful) |
		 FIELD_PREP(PAGE_FAULT_REPLY_REPLY,
			    PAGE_FAULT_REPLY_ACCESS) |
		 FIELD_PREP(PAGE_FAULT_REPLY_DESC_TYPE,
			    FAULT_RESPONSE_DESC) |
		 FIELD_PREP(PAGE_FAULT_REPLY_ASID,
			    f->info.asid)),

		(FIELD_PREP(PAGE_FAULT_REPLY_VFID,
			    f->info.vfid) |
		 FIELD_PREP(PAGE_FAULT_REPLY_ENG_INSTANCE,
			    f->info.engine_instance) |
		 FIELD_PREP(PAGE_FAULT_REPLY_ENG_CLASS,
			    f->info.engine_class) |
		 FIELD_PREP(PAGE_FAULT_REPLY_PDATA,
			    f->info.pdata)),
	};

	return intel_guc_send(f->guc, action, ARRAY_SIZE(action));
}

static void coredump_add_request(struct i915_gpu_coredump *dump,
				 struct i915_request *rq,
				 gfp_t gfp)
{
	struct intel_gt_coredump *gt = dump->gt;
	struct intel_engine_capture_vma *vma;
	struct i915_page_compress *compress;

	compress = i915_vma_capture_prepare(gt);
	if (!compress)
		return;

	vma = intel_engine_coredump_add_request(gt->engine, rq, gfp, compress);
	if (vma)
		intel_engine_coredump_add_vma(gt->engine, vma, compress);

	i915_vma_capture_finish(gt, compress);
}

static void fault_complete(struct dma_fence_work *work)
{
	struct fault_reply *f = container_of(work, typeof(*f), base);
	struct i915_gpu_coredump *dump = f->dump;

	if (dump && dump->private) {
		coredump_add_request(dump, dump->private, GFP_KERNEL);
		i915_request_put(dump->private);
	}

	if (work->rq.fence.error) {
		print_recoverable_fault(&f->info,
					"Fault response: Unsuccessful",
					work->rq.fence.error);
		f->info.fault_unsuccessful = true;
	}

	/*
	 * While Pagefault WA processing, i915 have to need to reply to the GuC
	 * first, then i915 can read properly the thread attentions (resolved
	 * -attentions) that SIP turns on.
	 */
	GEM_WARN_ON(send_fault_reply(f));

	if (dump) {
		u32 td_ctl;

		td_ctl = intel_gt_mcr_read_any(f->gt, TD_CTL);
		if (td_ctl) {
			struct intel_gt_coredump *gt = dump->gt;
			struct intel_engine_cs *engine =
				(struct intel_engine_cs *)gt->engine->engine;

			intel_eu_attentions_read(f->gt, &gt->attentions.resolved,
						 INTEL_GT_ATTENTION_TIMEOUT_MS);

			/* No more exceptions, stop raising new ATTN */
			td_ctl &= ~(TD_CTL_FORCE_EXTERNAL_HALT | TD_CTL_FORCE_EXCEPTION);
			intel_gt_mcr_multicast_write(f->gt, TD_CTL, td_ctl);

			/* Reset and cleanup if there are any ATTN leftover */
			intel_engine_schedule_heartbeat(engine);
		}

		i915_error_state_store(dump);
		i915_gpu_coredump_put(dump);
	} else if (f->debugger_pagefault) {
		struct i915_address_space *vm = faulted_vm(f->guc, f->info.asid);
		struct i915_debugger_pagefault *debugger_pagefault
			= f->debugger_pagefault;
		struct intel_engine_cs *engine = debugger_pagefault->engine;
		struct intel_gt *gt;
		u64 start, length;
		u32 td_ctl;
		int i;

		/* The active context [asid] is protected while servicing a fault */
		if (GEM_WARN_ON(!vm))
			goto err_dbg_cleanup;

		intel_eu_attentions_read(f->gt,
					 &debugger_pagefault->attentions.resolved,
					 INTEL_GT_ATTENTION_TIMEOUT_MS);
		/*
		 * Install the fault PTE;
		 * In order to get a pagefault again at the same address in the
		 * future, it clears the PTE of the page used as pagefault WA.
		 * If very many threads on the GPU are executing the same code
		 * and this code causes a pagefault, then this can cause
		 * a pagefault flood in the worst case.
		 */
		start = f->info.page_addr;
		length = BIT(vm->scratch_order + PAGE_SHIFT);
		/* clear the PTE of pagefault address */
		vm->clear_range(vm, start, length);

		/* invalidate tlb range for the pagefault address range*/
		for_each_gt(gt, vm->i915, i) {
			if (!atomic_read(&vm->active_contexts_gt[i]))
				continue;

			intel_gt_invalidate_tlb_range(gt, vm, start, length);
		}
		vm->invalidate_tlb_scratch = false;

err_dbg_cleanup:
		/* clear Force_External and Force Exception on pagefault scenario */
		td_ctl = intel_gt_mcr_read_any(f->gt, TD_CTL);
		intel_gt_mcr_multicast_write(f->gt, TD_CTL, td_ctl &
					     ~(TD_CTL_FORCE_EXTERNAL_HALT | TD_CTL_FORCE_EXCEPTION));

		i915_debugger_handle_engine_page_fault(engine, debugger_pagefault);
	}

	intel_gt_pm_put(f->gt, f->wakeref);
}

static const struct dma_fence_work_ops reply_ops = {
	.name = "pagefault",
	.work = fault_work,
	.complete = fault_complete,
};

int intel_pagefault_req_process_msg(struct intel_guc *guc,
				    const u32 *payload,
				    u32 len)
{
	struct intel_gt *gt = guc_to_gt(guc);
	struct fault_reply *reply;
	struct dma_fence *fence;

	if (unlikely(len != 4))
		return -EPROTO;

	reply = kzalloc(sizeof(*reply), GFP_KERNEL);
	if (!reply)
		return -ENOMEM;

	dma_fence_work_init(&reply->base, &reply_ops, gt->i915->sched);
	get_fault_info(payload, &reply->info);
	reply->guc = guc;

	reply->gt = gt;
	reply->wakeref = intel_gt_pm_get(gt);

	/*
	 * Keep track of the background work to migrate the backing store and
	 * bind the vma for the faulting address.
	 *
	 * We often see hundreds of concurrent pagefaults raised by a single EU
	 * kernel running on many hundreds of threads on a single engine.  If
	 * we sequentially process the vma binding and then each fault response
	 * that will consume a few milliseconds (roughly 20us per CT fault
	 * response message plus the millisecond or so required to handle the
	 * fault itself). Alternatively, we can reorder the fault replies to
	 * begin all the second responses while the migration and vma binding
	 * is in progress by processing the two halves as separate halves.
	 * (For simplicity, we submit all of the fault handlers as their own
	 * work as we do not know ahead of time how many pagefaults have been
	 * generated, and just let the CPU scheduler and HW handle the
	 * parallelism.)
	 *
	 * To mitigate against stalls when trying to submit a few hundred
	 * pagefault responses via the GuC CT, we make sure we have a
	 * sufficiently larger send (H2G) buffer to accommodate a typical
	 * number of messages (assuming the buffer is not already backlogged).
	 */
	fence = handle_i915_mm_fault(guc, &reply->info, &reply->dump,
				     &reply->debugger_pagefault,
				     &reply->base.rq.fence);
	if (IS_ERR(fence)) {
		i915_sw_fence_set_error_once(&reply->base.rq.submit,
					     PTR_ERR(fence));
	} else if (fence) {
		dma_fence_work_chain(&reply->base, fence);
		dma_fence_put(fence);
	}

	i915_request_set_priority(&reply->base.rq, I915_PRIORITY_BARRIER);
	dma_fence_work_commit_imm_if(&reply->base, !IS_ENABLED(CONFIG_DRM_I915_CHICKEN_ASYNC_PAGEFAULTS));

	/* Serialise each pagefault with its reply? */
	if (!IS_ENABLED(CONFIG_DRM_I915_CHICKEN_ASYNC_PAGEFAULTS))
		dma_fence_wait(&reply->base.rq.fence, false);

	return 0;
}

const char *intel_pagefault_type2str(unsigned int type)
{
	static const char * const faults[] = {
		[NOT_PRESENT] = "not present",
		[WRITE_ACCESS_VIOLATION] = "write access violation",
		[ATOMIC_ACCESS_VIOLATION] = "atomic access violation",
	};

	if (type > ATOMIC_ACCESS_VIOLATION || !faults[type])
		return "invalid fault type";

	return faults[type];
}

const char *intel_access_type2str(unsigned int type)
{
	static const char * const access[] = {
		[ACCESS_TYPE_READ] = "read",
		[ACCESS_TYPE_WRITE] = "write",
		[ACCESS_TYPE_ATOMIC] = "atomic",
		[ACCESS_TYPE_RESERVED] = "reserved",
	};

	if (type > ACCESS_TYPE_RESERVED || !access[type])
		return "invalid access type";

	return access[type];
}

static struct i915_vma *get_acc_vma(struct intel_guc *guc,
				    struct acc_info *info)
{
	struct i915_address_space *vm;
	u64 page_va;

	vm = faulted_vm(guc, info->asid);
	if (GEM_WARN_ON(!vm))
		return NULL;

	page_va = info->va_range_base + (ffs(info->sub_granularity) - 1)
		  * sub_granularity_in_byte(info->granularity);

	return i915_find_vma(vm, page_va);
}

const char *intel_acc_err2str(unsigned int err)
{
	static const char * const faults[] = {
		[ACCESS_ERR_OK] = "",
		[ACCESS_ERR_NOSUP] = "not supported",
		[ACCESS_ERR_NULLVMA] = "null vma",
		[ACCESS_ERR_USERPTR] = "userptr",
	};

	if (err >= ARRAY_SIZE(faults) || !faults[err])
		return "invalid acc err!";

	return faults[err];
}

static int acc_migrate_to_lmem(struct intel_gt *gt, struct i915_vma *vma)
{
	struct i915_gem_ww_ctx ww;
	int err;

	i915_gem_vm_bind_lock(vma->vm);

	if (!i915_vma_is_bound(vma, PIN_RESIDENT)) {
		i915_gem_vm_bind_unlock(vma->vm);
		return 0;
	}

	for_i915_gem_ww(&ww, err, false) {
		enum intel_region_id lmem_id;

		err = i915_gem_object_lock(vma->obj, &ww);
		if (err)
			continue;

		lmem_id = get_lmem_region_id(vma->obj, gt);
		if (lmem_id)
			err = migrate_to_lmem(vma->obj, lmem_id, &ww);
	}

	i915_gem_vm_bind_unlock(vma->vm);

	return err;
}

static int handle_i915_acc(struct intel_guc *guc,
			   struct acc_info *info)
{
	struct intel_gt *gt = guc_to_gt(guc);
	struct i915_vma *vma;

	mark_engine_as_active(gt, info->engine_class, info->engine_instance);

	if (info->access_type) {
		trace_intel_access_counter(gt, info, ACCESS_ERR_NOSUP);
		return 0;
	}

	vma = get_acc_vma(guc, info);
	if (!vma) {
		trace_intel_access_counter(gt, info, ACCESS_ERR_NULLVMA);
		return 0;
	}

	if (i915_gem_object_is_userptr(vma->obj)) {
		trace_intel_access_counter(gt, info, ACCESS_ERR_USERPTR);
		goto put_vma;
	}

	acc_migrate_to_lmem(gt, vma);

	trace_intel_access_counter(gt, info, ACCESS_ERR_OK);
put_vma:
	i915_vma_put(vma);
	__i915_vma_put(vma);

	return 0;
}

static void get_access_counter_info(struct access_counter_desc *desc,
				    struct acc_info *info)
{
	info->granularity = FIELD_GET(ACCESS_COUNTER_GRANULARITY, desc->dw2);
	info->sub_granularity =	FIELD_GET(ACCESS_COUNTER_SUBG_HI, desc->dw1) << 31 |
				FIELD_GET(ACCESS_COUNTER_SUBG_LO, desc->dw0);
	info->engine_class = FIELD_GET(ACCESS_COUNTER_ENG_CLASS, desc->dw1);
	info->engine_instance = FIELD_GET(ACCESS_COUNTER_ENG_INSTANCE, desc->dw1);
	info->asid =  FIELD_GET(ACCESS_COUNTER_ASID, desc->dw1);
	info->vfid =  FIELD_GET(ACCESS_COUNTER_VFID, desc->dw2);
	info->access_type = FIELD_GET(ACCESS_COUNTER_TYPE, desc->dw0);
	info->va_range_base = make_u64(desc->dw3 & ACCESS_COUNTER_VIRTUAL_ADDR_RANGE_HI,
			      desc->dw2 & ACCESS_COUNTER_VIRTUAL_ADDR_RANGE_LO);

	GEM_BUG_ON(info->engine_class > MAX_ENGINE_CLASS ||
		   info->engine_instance > MAX_ENGINE_INSTANCE);
}

int intel_access_counter_req_process_msg(struct intel_guc *guc, const u32 *payload, u32 len)
{
	struct acc_info info = {};

	if (unlikely(len != 4))
		return -EPROTO;

	get_access_counter_info((struct access_counter_desc *)payload, &info);
	return handle_i915_acc(guc, &info);
}
