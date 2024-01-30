// SPDX-License-Identifier: MIT
/*
 * Copyright © 2022 Intel Corporation
 */

#include <linux/mm_types.h>
#include <linux/mm.h>
#include <linux/sched/mm.h>

#include "i915_drv.h"
#include "intel_memory_region.h"
#include "gem/i915_gem_context.h"
#include "gt/intel_gt.h"
#include "gt/intel_tlb.h"

bool i915_ats_enabled(struct drm_i915_private *dev_priv)
{
	return test_bit(INTEL_FLAG_ATS_ENABLED, &dev_priv->flags);
}

bool is_vm_pasid_active(struct i915_address_space *vm)
{
	return (i915_ats_enabled(vm->i915) && vm->has_pasid);
}

void i915_enable_ats(struct drm_i915_private *i915)
{
	struct pci_dev *pdev = to_pci_dev(i915->drm.dev);
	int err = 0;

	if (!i915->params.address_translation_services)
		return;

	i915->pasid_counter = 0;
	if (!pci_ats_supported(pdev)) {
		drm_info(&i915->drm,
			 "There is no Address Translation Services (ATS) support for the device\n");
		return;
	}

	err = iommu_dev_enable_feature(&pdev->dev, IOMMU_DEV_FEAT_SVA);
	if (err) {
		/*
		 * FIXME: Update the error check routine when the code in the
		 * Kernel that returns EINVAL due to lack of PRI support get
		 * updated - for now, we just log it and continue, without
		 * returning error
		 */
		drm_info(&i915->drm,
			 "Failed to enable SVA feature on the device - KMD will handle\n"
			 "faulting, no functional impact on the device - error: %pe\n",
			 ERR_PTR(err));
	}

	drm_info(&i915->drm,
		 "Succeeded in enabling SVA for Address Translation Services (ATS) support\n");

	/* Set ATS enabled flag with IOMMU successfully configured */
	set_bit(INTEL_FLAG_ATS_ENABLED, &i915->flags);
}

void i915_disable_ats(struct drm_i915_private *i915)
{
	struct pci_dev *pdev = to_pci_dev(i915->drm.dev);

	if (!i915_ats_enabled(i915))
		return;

	/*
	 * FIXME: disable/detach PASID associated with vm->id?
	 *
	 * Since call to enable SVA feature is failing due to the lack of PRI
	 * or PRS support, we don't need to disable SVA feature for now - So,
	 * we need to call the following function after final resolution in the
	 * kernel - "iommu_dev_disable_feature(&pdev->dev, IOMMU_DEV_FEAT_SVA)"
	 */
	WRITE_ONCE(i915->pasid_counter, 0);
	pci_disable_ats(pdev);
	clear_bit(INTEL_FLAG_ATS_ENABLED, &i915->flags);
}

/* PASID value contains 20-bit wide */
void i915_destroy_pasid(struct i915_address_space *vm)
{
	if (!i915_ats_enabled(vm->i915))
		return;

	if (vm->sva && is_vm_pasid_active(vm)) {
		WRITE_ONCE(vm->i915->pasid_counter, vm->i915->pasid_counter - 1);
		iommu_sva_unbind_device(vm->sva);
		vm->has_pasid = false;
		vm->sva = NULL;
	}
}

int i915_create_pasid(struct i915_address_space *vm)
{
	struct drm_i915_private *i915 = vm->i915;
	struct iommu_sva *sva_handle;
	int err;
	u32 pasid;

	if (!i915_ats_enabled(i915))
		return -EINVAL;

	sva_handle = iommu_sva_bind_device(vm->dma, current->mm, NULL);
	if (IS_ERR(sva_handle)) {
		err = PTR_ERR(sva_handle);
		drm_err(&i915->drm,
			"Binding address space to the device in order to use PASID failed with error %d\n",
			err);
		return err;
	}

	pasid = i915_get_pasid(sva_handle);
	if (pasid == IOMMU_PASID_INVALID) {
		drm_err(&i915->drm,
			"Invalid PASID created - need to unbind the device and disable ATS %d\n",
			pasid);
		return -EINVAL;
	}

	/* update address space sva and pasid */
	vm->sva = sva_handle;
	vm->pasid = pasid;
	vm->has_pasid = true;

	/* Update pasid global counter */
	WRITE_ONCE(vm->i915->pasid_counter, vm->i915->pasid_counter + 1);

	return 0;
}

int i915_global_pasid_counter(struct drm_i915_private *i915)
{
	return i915->pasid_counter;
}

enum recoverable_page_fault_type {
	FAULT_READ_NOT_PRESENT = 0x0,
	FAULT_WRITE_NOT_PRESENT = 0x1,
	FAULT_ATOMIC_NOT_PRESENT = 0x2,
	FAULT_WRITE_ACCESS_VIOLATION = 0x5,
	FAULT_ATOMIC_ACCESS_VIOLATION = 0xa,
};

int i915_handle_ats_fault_request(struct i915_address_space *vm,
				  struct recoverable_page_fault_info *info)
{
	unsigned int fault_flags = FAULT_FLAG_REMOTE | FAULT_FLAG_USER;
	enum recoverable_page_fault_type fault_type;
	vm_fault_t ret = VM_FAULT_ERROR;
	struct mm_struct *mm;
	struct vm_area_struct *vma;
	int status = 0;
	unsigned int access_flags = 0;

	/* ensure that a pasid is associated with the vm */
	GEM_BUG_ON(!vm->pasid);

	mm = iommu_sva_find(vm->pasid);
	if (IS_ERR_OR_NULL(mm))
		return !mm ? -EINVAL : PTR_ERR(mm);

	mmap_read_lock(mm);
	vma = find_vma(mm, info->page_addr);
	if (!vma || info->page_addr < vma->vm_start)
		goto end_request;

	fault_type = (info->fault_type << 2) | info->access_type;

	switch (fault_type & 0xF) {
	case FAULT_READ_NOT_PRESENT:
		access_flags |= VM_READ;
		break;
	case FAULT_WRITE_NOT_PRESENT:
		access_flags |= VM_WRITE;
		fault_flags |= FAULT_FLAG_WRITE;
		break;
	case FAULT_ATOMIC_NOT_PRESENT:
	case FAULT_WRITE_ACCESS_VIOLATION:
	default:
		drm_err(&vm->i915->drm, "Invalid fault type request\n");
		goto end_request;
	}

	if (access_flags & ~vma->vm_flags)
		goto end_request;

	ret = handle_mm_fault(vma, info->page_addr, fault_flags, NULL);
	status = ret & VM_FAULT_ERROR ? -EINVAL : 0;

end_request:
	mmap_read_unlock(mm);
	mmput(mm);

	return status;
}

void intel_invalidate_devtlb_range(struct i915_address_space *vm,
				   u64 start, u64 size)
{
	struct intel_gt *gt;
	int id;

	if (!is_vm_pasid_active(vm))
		return;

	for_each_gt(gt, vm->i915, id) {
		if (!atomic_read(&vm->active_contexts_gt[id]))
			continue;

		intel_gt_invalidate_tlb_range(gt, vm, start, size);
	}
}
