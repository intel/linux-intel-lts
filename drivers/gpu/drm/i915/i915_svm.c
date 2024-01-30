// SPDX-License-Identifier: MIT
/*
 * Copyright © 2019 Intel Corporation
 */

#include <linux/mm_types.h>
#include <linux/sched/mm.h>
#include <linux/mm.h>

#include "i915_svm.h"
#include "intel_memory_region.h"
#include "gem/i915_gem_context.h"

struct svm_notifier {
	struct mmu_interval_notifier notifier;
	struct work_struct unregister_notifier_work;
	struct vm_area_struct *vma;
	struct i915_svm *svm;
};

static void unregister_svm_notifier(struct vm_area_struct *vma,
		struct i915_svm *svm);

static inline struct svm_notifier *vma_to_sn(struct vm_area_struct *vma)
{
	return (struct svm_notifier *)vma->vm_private_data;
}

static bool i915_svm_range_invalidate(struct mmu_interval_notifier *mni,
				      const struct mmu_notifier_range *range,
				      unsigned long cur_seq)
{
	struct svm_notifier *sn =
		container_of(mni, struct svm_notifier, notifier);
	struct i915_svm *svm = sn->svm;
	unsigned long length = range->end - range->start;

	/*
	 * MMU_NOTIFY_RELEASE is called upon process exit to notify driver
	 * to release any process resources, such as clear GPU page table
	 * mapping or unregister mmu notifier etc. We already clear GPU
	 * page table in i915_vm_release and unregister mmu notifier in
	 * release_svm, upon process exit. So just simply return here.
	 */
	if (range->event == MMU_NOTIFY_RELEASE)
		return true;

	if (mmu_notifier_range_blockable(range))
		mutex_lock(&svm->mutex);
	else if (!mutex_trylock(&svm->mutex))
		return false;
	mmu_interval_set_seq(mni, cur_seq);
	svm_unbind_addr(svm->vm, range->start, length);
	mutex_unlock(&svm->mutex);

	if (range->event == MMU_NOTIFY_UNMAP && vma_to_sn(range->vma))
		queue_work(system_unbound_wq, &sn->unregister_notifier_work);

	return true;
}

static const struct mmu_interval_notifier_ops i915_svm_mni_ops = {
	.invalidate = i915_svm_range_invalidate,
};

static void i915_svm_unregister_notifier_work(struct work_struct *work)
{
	struct svm_notifier *sn;

	sn = container_of(work, typeof(*sn), unregister_notifier_work);

	unregister_svm_notifier(sn->vma, sn->svm);
}

/** register a mmu interval notifier to monitor vma change
 * @vma: vma to monitor
 * @svm: which i915_svm is monitoring the vma
 * if this vma is already been registered with a notifier, return it directly;
 * otherwise create and insert a new notifier.
 * this function must be called with mmap_write_lock
 */
static struct svm_notifier *register_svm_notifier(struct vm_area_struct *vma,
						struct i915_svm *svm)
{
	struct svm_notifier *sn;
	u64 start, length;
	int ret = 0;

	sn = vma_to_sn(vma);
	if (sn)
		return sn;

	sn = kmalloc(sizeof(*sn), GFP_KERNEL);
	if (!sn)
		return ERR_PTR(-ENOMEM);

	start =  vma->vm_start;
	length = vma->vm_end - vma->vm_start;
	ret = mmu_interval_notifier_insert_locked(&sn->notifier, vma->vm_mm,
					   start, length,
					   &i915_svm_mni_ops);
	if (ret) {
		kfree(sn);
		return ERR_PTR(ret);
	}

	sn->svm = svm;
	sn->vma = vma;
	vma->vm_private_data = sn;
	INIT_WORK(&sn->unregister_notifier_work, i915_svm_unregister_notifier_work);

	return sn;
}

static void unregister_svm_notifier(struct vm_area_struct *vma,
		struct i915_svm *svm)
{
	struct svm_notifier *sn;

	sn = vma_to_sn(vma);
	if (!sn)
	    return;

	if (sn->svm != svm)
		return;

	mmu_interval_notifier_remove(&sn->notifier);
	kfree(sn);
	vma->vm_private_data = NULL;
}
static struct i915_svm *vm_get_svm(struct i915_address_space *vm)
{
	struct i915_svm *svm = vm->svm;

	mutex_lock(&vm->svm_mutex);
	if (svm && !kref_get_unless_zero(&svm->ref))
		svm = NULL;

	mutex_unlock(&vm->svm_mutex);
	return svm;
}

static void release_svm(struct kref *ref)
{
	struct i915_svm *svm = container_of(ref, typeof(*svm), ref);
	struct i915_address_space *vm = svm->vm;
	struct mm_struct *mm = svm->mm;
	struct vm_area_struct *vma = 0;

	for (vma = mm->mmap; vma; vma = vma->vm_next) {
		unregister_svm_notifier(vma, svm);
	}

	mutex_destroy(&svm->mutex);
	vm->svm = NULL;
	mmdrop(svm->mm);
	kfree(svm);
}

static void vm_put_svm(struct i915_address_space *vm)
{
	mutex_lock(&vm->svm_mutex);
	if (vm->svm)
		kref_put(&vm->svm->ref, release_svm);
	mutex_unlock(&vm->svm_mutex);
}

static u32 i915_svm_build_sg(struct i915_address_space *vm,
			     struct hmm_range *range,
			     struct sg_table *st)
{
	struct scatterlist *sg;
	u32 sg_page_sizes = 0;
	u64 i, npages;

	sg = NULL;
	st->nents = 0;
	npages = ((range->end - 1) >> PAGE_SHIFT) - (range->start >> PAGE_SHIFT) + 1;

	/*
	 * No need to dma map the host pages and later unmap it, as
	 * GPU is not allowed to access it with SVM. //TODO: seems not true
	 * XXX: Need to dma map host pages for integrated graphics while
	 * extending SVM support there.
	 */
	for (i = 0; i < npages; i++) {
		unsigned long addr = range->hmm_pfns[i];

		if (sg && (addr == (sg_dma_address(sg) + sg->length))) {
			sg->length += PAGE_SIZE;
			sg_dma_len(sg) += PAGE_SIZE;
			continue;
		}

		if (sg)
			sg_page_sizes |= sg->length;

		sg =  sg ? __sg_next(sg) : st->sgl;
		sg_dma_address(sg) = addr;
		sg_dma_len(sg) = PAGE_SIZE;
		sg->length = PAGE_SIZE;
		st->nents++;
	}

	sg_page_sizes |= sg->length;
	sg_mark_end(sg);
	return sg_page_sizes;
}

static int i915_hmm_convert_pfn(struct drm_i915_private *dev_priv,
				struct hmm_range *range)
{
	unsigned long i, npages;
	int regions = 0;

	npages = ((range->end - 1) >> PAGE_SHIFT) - (range->start >> PAGE_SHIFT) + 1;
	for (i = 0; i < npages; ++i) {
		struct page *page;
		unsigned long addr;

		if (!(range->hmm_pfns[i] & HMM_PFN_VALID)) {
			range->hmm_pfns[i] = 0;
			continue;
		}

		page = hmm_pfn_to_page(range->hmm_pfns[i]);
		if (!page)
			continue;

		if (is_device_private_page(page)) {
			struct i915_buddy_block *block = page->zone_device_data;
			struct intel_memory_region *mem = block->private;

			regions |= REGION_LMEM;
			addr = mem->region.start;
			addr += PFN_PHYS(page_to_pfn(page) - mem->devmem->pfn_first);
		} else {
			regions |= REGION_SMEM;
			addr = page_to_phys(page);
		}

		range->hmm_pfns[i] = addr;
	}

	return regions;
}

/** Populate physical pages of a virtual address range
 * This function also read mmu notifier sequence # (
 * mmu_interval_read_begin), for the purpose of later
 * comparison (through mmu_interval_read_retry).
 * This must be called with mmap read or write lock held.
 * This function alloates hmm_range->hmm_pfns, it is caller's
 * responsibility to free it.
 * @sn: svm interval notifier of this virtual address range
 * @start: start of the virtual address range, inclusive
 * @end: end of the virtual address range, exclusive
 * @hmm_range: pointer to hmm_range struct
 * @write: populate pages with write permission
 * returns: 0 for succuss; negative error no on failure
 */
static int svm_populate_range(struct svm_notifier *sn,
			    __u64 start, __u64 end,
			    struct hmm_range *hmm_range, bool write)
{
	unsigned long timeout =
		jiffies + msecs_to_jiffies(HMM_RANGE_DEFAULT_TIMEOUT);
	unsigned long *pfns, flags = HMM_PFN_REQ_FAULT;
	u64 npages;
	int ret;

	mmap_assert_locked(sn->svm->mm);

	npages = ((end - 1) >> PAGE_SHIFT) - (start >> PAGE_SHIFT) + 1;
	pfns = kvmalloc_array(npages, sizeof(*pfns), GFP_KERNEL);
	if (unlikely(!pfns))
		return -ENOMEM;

	if (write)
		flags |= HMM_PFN_REQ_WRITE;

	memset64((u64 *)pfns, (u64)flags, npages);
	hmm_range->hmm_pfns = pfns;
	hmm_range->notifier_seq = mmu_interval_read_begin(&sn->notifier);
	hmm_range->notifier = &sn->notifier;
	hmm_range->start = start;
	hmm_range->end = end;
	hmm_range->pfn_flags_mask = HMM_PFN_REQ_FAULT | HMM_PFN_REQ_WRITE;
	hmm_range->dev_private_owner = sn->svm->vm->i915->drm.dev;

	while (true) {
		ret = hmm_range_fault(hmm_range);
		if (time_after(jiffies, timeout))
			goto free_pfns;

		if (ret == -EBUSY)
			continue;
		break;
	}

free_pfns:
	if (ret)
		kvfree(pfns);
	return ret;
}

/** bind a svm range to gpu, fill gpu page tables
 * @svm: svm process that the range belongs to
 * @range: hmm_range to bind to gpu
 * @flags: binding flags
 * Return 0 on success; negative error code on failure
 * If -EAGAIN returns, it means mmu notifier was called (
 * aka there was concurrent cpu page table update) during
 * this function, caller has to retry hmm_range_fault
 */
static int svm_bind_range_to_gpu(struct i915_svm *svm,
				struct hmm_range *range,
				__u64 flags)
{
	u64 npages = ((range->end - 1) >> PAGE_SHIFT) - (range->start >> PAGE_SHIFT) + 1;
	struct i915_address_space *vm = svm->vm;
	__u64 length = range->end - range->start;
	struct i915_vm_pt_stash stash = { 0 };
	__u64 start = range->start;
	struct i915_gem_ww_ctx ww;
	struct sg_table st;
	u32 sg_page_sizes;
	int regions;
	long ret;

	if (unlikely(sg_alloc_table(&st, npages, GFP_KERNEL)))
		return -ENOMEM;

	/* Ensure the range is in one memory region */
	regions = i915_hmm_convert_pfn(vm->i915, range);
	if (!regions ||
		((regions & REGION_SMEM) && (regions & REGION_LMEM))) {
		ret = -EINVAL;
		goto free_sg;
	}

	sg_page_sizes = i915_svm_build_sg(vm, range, &st);

	/* XXX: Not an elegant solution, revisit */
	i915_gem_ww_ctx_init(&ww, true);
	ret = svm_bind_addr_prepare(vm, &stash, &ww, start, length);
	if (ret)
	    goto fault_done;

	mutex_lock(&svm->mutex);
	if (mmu_interval_read_retry(range->notifier,
		    range->notifier_seq)) {
		svm_unbind_addr(vm, start, length);
		ret = -EAGAIN;
		goto unlock_svm;
	}

	flags |= (regions & REGION_LMEM) ? I915_GTT_SVM_LMEM : 0;
	//TODO: handle atomic fault - AE bit in page table
	ret = svm_bind_addr_commit(vm, &stash, start, length,
				   flags, &st, sg_page_sizes);
unlock_svm:
	mutex_unlock(&svm->mutex);
	i915_vm_free_pt_stash(vm, &stash);
fault_done:
	i915_gem_ww_ctx_fini(&ww);
free_sg:
	sg_free_table(&st);
	return ret;
}

/* Determine whether read or/and write to vma is allowed
 * write: true means a read and write access; false: read only access
 */
static bool svm_access_allowed(struct vm_area_struct *vma, bool write)
{
	unsigned long access = VM_READ;

	if (write)
		access |= VM_WRITE;

	return (vma->vm_flags & access) == access;
}

static bool svm_should_migrate(u64 va, enum intel_region_id dst_region, bool is_atomic_fault)
{
	return true;
}

/**
 * svm_migrate_to_vram() - migrate backing store of a va range to vram
 * Must be called with mmap_read_lock(mm) held.
 * @mm: the process mm_struct of the va range
 * @start: start of the va range
 * @length: length of the va range
 * @mem: destination migration memory region
 *
 * Returns: negative errno on faiure, 0 on success
 */
static int svm_migrate_to_vram(struct mm_struct *mm,
			__u64 start, __u64 length,
			struct intel_memory_region *mem)
{
	unsigned long addr, end;
	struct i915_gem_ww_ctx ww;
	int err = 0;

	mmap_assert_locked(mm);
	if (!mem->devmem)
		return -EINVAL;

	i915_gem_ww_ctx_init(&ww, true);
retry:
	for (addr = start, end = start + length; addr < end;) {
		struct vm_area_struct *vma;
		unsigned long next;

		vma = find_vma_intersection(mm, addr, end);
		if (!vma)
			break;

		addr &= PAGE_MASK;
		next = min(vma->vm_end, end);
		next = round_up(next, PAGE_SIZE);

		err = i915_devmem_migrate_vma(mem, &ww, vma, addr, next);
		if (err == -EDEADLK)
			goto out_ww;

		addr = next;
	}

out_ww:
	if (err == -EDEADLK) {
		err = i915_gem_ww_ctx_backoff(&ww);
		if (!err)
			goto retry;
	}

	i915_gem_ww_ctx_fini(&ww);
	return 0;
}

int i915_svm_handle_gpu_fault(struct i915_address_space *vm,
				struct intel_gt *gt,
				struct recoverable_page_fault_info *info)
{
	struct hmm_range hmm_range;
	struct vm_area_struct *vma;
	bool mmap_unlocked = false;
	struct svm_notifier *sn;
	u64 start = 0, end = 0, length = 0;
	struct i915_svm *svm;
	struct mm_struct *mm;
	int ret = 0;

	svm = vm_get_svm(vm);
	if (!svm)
		return -EINVAL;

	mm = svm->mm;
	mmap_read_lock(mm);
	vma = find_vma_intersection(mm, info->page_addr, info->page_addr + 4);
	if (!vma) {
		ret = -ENOENT;
		goto mmap_unlock;
	}

	if (!svm_access_allowed (vma, info->access_type != ACCESS_TYPE_READ)) {
		ret = -EPERM;
		goto mmap_unlock;
	}

	/** We are inside a mmap_read_lock, but it requires a mmap_write_lock
	 *  to register mmu notifier.
	 */
	mmap_read_unlock(mm);
	mmap_write_lock(mm);
	/** Migrate only 1 page for now.
	 *  If perform of this scheme is bad, we can introduce a
	 *  migration granularity parameter for user to select.
	 */
	start =  max(info->page_addr & PAGE_MASK, (u64)vma->vm_start);
	end = min(ALIGN(info->page_addr + 1, PAGE_SIZE), (u64)vma->vm_end);
	length = end - start;
	DRM_DEBUG_DRIVER("%s fault address 0x%llx vma start 0x%lx migration start 0x%llx, \
			vma end 0x%lx migration end 0x%llx\n",
			__func__, info->page_addr, vma->vm_start, start, vma->vm_end, end);
	sn = register_svm_notifier(vma, svm);
	mmap_write_downgrade(mm);
	if (IS_ERR(sn)) {
		ret = PTR_ERR(sn);
		goto mmap_unlock;
	}

	if (svm_should_migrate(start, gt->lmem->id, info->access_type == ACCESS_TYPE_ATOMIC))
		/*
		 * Migration is best effort. If we failed to migrate to vram,
		 * we just map that range to gpu in system memory. For cases
		 * such as gpu atomic operation which requires memory to be
		 * resident in vram, we will fault again and retry migration.
		 */
		svm_migrate_to_vram(mm, start, length, gt->lmem);
	ret = svm_populate_range(sn, start, end, &hmm_range,
			vma->vm_flags & VM_WRITE);
	if (ret)
		goto unregister_notifier;

	mmap_read_unlock(mm);
	mmap_unlocked = true;
	ret = svm_bind_range_to_gpu(svm, &hmm_range,
		!(vma->vm_flags & VM_WRITE) ? I915_GTT_SVM_READONLY : 0);
	/** Concurrent cpu page table update happened,
	 *  Return successfully so we will retry everything
	 *  on next gpu page fault.
	 */
	if (ret == -EAGAIN)
		ret = 0;

	kvfree(hmm_range.hmm_pfns);
	goto mmap_unlock;
unregister_notifier:
	unregister_svm_notifier(vma, svm);
mmap_unlock:
	if (!mmap_unlocked)
		mmap_read_unlock(mm);
	vm_put_svm(vm);
	if (length)
		ppgtt_dump(vm, start, length);
	return ret;
}

void i915_svm_unbind_mm(struct i915_address_space *vm)
{
	vm_put_svm(vm);
}

int i915_svm_bind_mm(struct i915_address_space *vm)
{
	struct mm_struct *mm = current->mm;
	struct i915_svm *svm;
	int ret = 0;

	mmap_write_lock(mm);
	mutex_lock(&vm->svm_mutex);
	if (vm->svm)
		goto bind_out;

	svm = kzalloc(sizeof(*svm), GFP_KERNEL);
	if (!svm) {
		ret = -ENOMEM;
		goto bind_out;
	}
	mutex_init(&svm->mutex);
	kref_init(&svm->ref);
	svm->vm = vm;
	mmgrab(mm);
	svm->mm = mm;

	vm->svm = svm;
bind_out:
	mutex_unlock(&vm->svm_mutex);
	mmap_write_unlock(mm);
	return ret;
}
