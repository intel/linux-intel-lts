/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2020 Intel Corporation
 */

#ifndef __I915_GEM_GTT_H__
#define __I915_GEM_GTT_H__

#include <linux/io-mapping.h>
#include <linux/types.h>

#include <drm/drm_mm.h>

#include "gt/intel_gtt.h"
#include "i915_scatterlist.h"

struct drm_i915_gem_object;
struct i915_address_space;

int __must_check i915_gem_gtt_prepare_pages(struct drm_i915_gem_object *obj,
					    struct sg_table *pages);
void i915_gem_gtt_finish_pages(struct drm_i915_gem_object *obj,
			       struct sg_table *pages);

int i915_gem_gtt_reserve(struct i915_address_space *vm,
			 struct drm_mm_node *node,
			 u64 size, u64 offset, unsigned long color,
			 unsigned int flags);

int i915_gem_gtt_insert(struct i915_address_space *vm,
			struct drm_mm_node *node,
			u64 size, u64 alignment, unsigned long color,
			u64 start, u64 end, unsigned int flags);

struct drm_mm_node *i915_gem_gtt_lookup(struct i915_address_space *vm,
					u64 addr);

/* Flags used by pin/bind&friends. */
#define PIN_NOEVICT		BIT_ULL(0)
#define PIN_NOSEARCH		BIT_ULL(1)
#define PIN_NONBLOCK		BIT_ULL(2)
#define PIN_MAPPABLE		BIT_ULL(3)
#define PIN_ZONE_32		BIT_ULL(4)
#define PIN_ZONE_48		BIT_ULL(5)
#define PIN_HIGH		BIT_ULL(6)
#define PIN_OFFSET_BIAS		BIT_ULL(7)
#define PIN_OFFSET_FIXED	BIT_ULL(8)
#define PIN_OFFSET_GUARD	BIT_ULL(9)

/*
 * Binding flags (common with i915_vma.flags).
 *
 * Bits 10/11 are copied into the i915_vma and used to track the vma usage.
 *
 * On gen5; there is only GGTT and we merely track whether the vma is intended
 * to be used by the kernel or by userspace, but is otherwise functionally
 * useless. We will make the vma as bound by USER | GLOBAL to avoid rebinds.
 *
 * On gen6/7, there is an aliasing-ppGTT. A single ppGTT overlaid on top of the
 * GGTT to give isolation between the kernel and user access (but doesn't allow
 * for isolation between different users). In this case, there is still a
 * single Global GTT that defines the address space for all, but now a second
 * ppGTT that uses the same i915_vma. Here, PIN_GLOBAL means create PTE in the
 * GGTT and PIN_USER means create PTE in the ppGTT.  The two flags create PTE
 * in the distinct GTT and so we need to track both binding. However, we only
 * ever bind with the GGTT vm.
 *
 * On gen8+, we have full per-process GTT (userspace) and the normal Global GTT
 * (kernel). We have full isolation between all users and kernel access. Here,
 * PIN_GLOBAL will only be used against the GGTT and PIN_USER against the
 * ppGTT. This then allows us to repurpose the PIN_GLOBAL bit on ppGTT to
 * indicate whether or not the binding storage has been allocated for this
 * i915_vma, aka PIN_RESIDENT. This is used by the pagefault enabled vm to make
 * vma binding a two step process. On the first step, the vma->node is reserved
 * and we may bind the faulting-scratch pages into that node depending on the
 * vm configuration. At this stage, we only mark the vma bound with PIN_USER;
 * it is not resident. The second step is then to bind the backing store into
 * the vma upon receiving a fault for that address. This second pass uses
 * PIN_RESIDENT. The vma is completely bound (the PTE point to the user's
 * backing store) once both PIN_RESIDENT | PIN_USER are set.
 */
#define PIN_GLOBAL		BIT_ULL(10) /* I915_VMA_GLOBAL_BIND */
#define PIN_RESIDENT		BIT_ULL(10) /* mutually exclusive with GGTT */
#define PIN_USER		BIT_ULL(11) /* I915_VMA_LOCAL_BIND */

#define PIN_OFFSET_MASK		I915_GTT_PAGE_MASK

static inline int i915_vm_move_to_active(struct i915_address_space *vm,
					 struct intel_context *ce,
					 struct i915_request *rq)
{
	if (i915_vm_page_fault_enabled(vm))
		return 0;

	return i915_active_add_suspend_fence(&vm->active, ce, rq);
}

static inline int i915_vm_sync(struct i915_address_space *vm)
{
	/* Wait for all requests under this vm to finish */
	return i915_active_wait(&vm->active);
}

static inline bool i915_vm_is_active(const struct i915_address_space *vm)
{
	return !i915_active_is_idle(&vm->active);
}

#endif
