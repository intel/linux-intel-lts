// SPDX-License-Identifier: MIT
/*
 * Copyright © 2019 Intel Corporation
 */

#include <linux/mm_types.h>
#include <linux/sched/mm.h>

#include "i915_svm.h"
#include "intel_memory_region.h"
#include "i915_trace.h"

struct i915_devmem_migrate {
	struct drm_i915_private *i915;
	struct migrate_vma *args;

	enum intel_region_id src_id;
	enum intel_region_id dst_id;
	dma_addr_t *host_dma;
	bool blitter_copy;
	u64 npages;

	/* required for blitter copy */
	struct drm_mm_node src_node;
	struct drm_mm_node dst_node;
	struct intel_context *ce;
	struct dma_fence *fence;
};

static int
i915_devmem_page_alloc_locked(struct intel_memory_region *mem,
			      unsigned long npages,
			      struct list_head *blocks,
			      struct i915_gem_ww_ctx *ww)
{
	unsigned long size = ALIGN((npages * PAGE_SIZE), mem->mm.chunk_size);
	struct i915_buddy_block *block;
	int ret;

	INIT_LIST_HEAD(blocks);
	ret = __intel_memory_region_get_pages_buddy(mem, ww, size, 0, blocks);
	if (unlikely(ret))
		goto alloc_failed;

	list_for_each_entry(block, blocks, link) {
		block->pfn_first = mem->devmem->pfn_first;
		block->pfn_first += PHYS_PFN(i915_buddy_block_offset(block) - mem->region.start);
		bitmap_zero(block->bitmap, I915_BUDDY_MAX_PAGES);
		DRM_DEBUG_DRIVER("%s pfn_first 0x%lx off 0x%llx size 0x%llx\n",
				 "Allocated block", block->pfn_first,
				 i915_buddy_block_offset(block),
				 i915_buddy_block_size(&mem->mm, block));
	}

alloc_failed:
	return ret;
}

static struct page *
i915_devmem_page_get_locked(struct intel_memory_region *mem,
			    struct list_head *blocks)
{
	struct i915_buddy_block *block, *on;

	list_for_each_entry_safe(block, on, blocks, link) {
		unsigned long weight, max;
		unsigned long i, pfn;
		struct page *page;

		max = i915_buddy_block_size(&mem->mm, block) / PAGE_SIZE;
		i = find_first_zero_bit(block->bitmap, max);
		if (unlikely(i == max)) {
			WARN(1, "Getting a page should have never failed\n");
			break;
		}

		set_bit(i, block->bitmap);
		pfn = block->pfn_first + i;
		page = pfn_to_page(pfn);
		get_page(page);
		lock_page(page);
		page->zone_device_data = block;
		weight = bitmap_weight(block->bitmap, max);
		if (weight == max)
			list_del_init(&block->link);
		DRM_DEBUG_DRIVER("%s pfn 0x%lx block weight 0x%lx\n",
				 "Allocated page", pfn, weight);
		return page;
	}
	return NULL;
}

static void
i915_devmem_page_free_locked(struct drm_i915_private *dev_priv,
			     struct page *page)
{
	unlock_page(page);
	put_page(page);
}

static int i915_devmem_bind_addr(struct i915_devmem_migrate *migrate,
				 struct i915_gem_ww_ctx *ww,
				 bool is_src)
{
	struct drm_i915_private *i915 = migrate->i915;
	struct i915_address_space *vm = migrate->ce->vm;
	struct intel_memory_region *mem;
	u64 npages = migrate->npages;
	u64 alignment = 0, flags = 0;
	enum intel_region_id mem_id;
	struct drm_mm_node *node;
	struct scatterlist *sg;
	u32 sg_page_sizes = 0;
	unsigned long color;
	struct sg_table st;
	int i, ret;

	if (unlikely(sg_alloc_table(&st, npages, GFP_KERNEL)))
		return -ENOMEM;

	if (is_src) {
		node = &migrate->src_node;
		mem_id = migrate->src_id;
		flags |= I915_GTT_SVM_READONLY;
	} else {
		node = &migrate->dst_node;
		mem_id = migrate->dst_id;
	}

	color = (mem_id != INTEL_REGION_SMEM);
	if (npages >= (SZ_2M >> PAGE_SHIFT))
		alignment = I915_GTT_PAGE_SIZE_2M;
	else if ((mem_id != INTEL_REGION_SMEM) && HAS_64K_PAGES(i915))
		alignment = I915_GTT_PAGE_SIZE_64K;

	mutex_lock(&vm->mutex);
	ret = i915_gem_gtt_insert(vm, node, npages * PAGE_SIZE,
				  alignment, color, 0, vm->total, PIN_USER);
	mutex_unlock(&vm->mutex);
	if (unlikely(ret))
		return ret;

	sg = NULL;
	st.nents = 0;

	/*
	 * XXX: If the source page is missing, source it from a temporary
	 * zero filled page. If needed, destination page missing scenarios
	 * can be similarly handled by draining data into a temporary page.
	 */
	for (i = 0; i < npages; i++) {
		u64 addr;

		if (mem_id == INTEL_REGION_SMEM) {
			addr = migrate->host_dma[i];
		} else {
			struct page *page;
			unsigned long mpfn;

			mpfn = is_src ? migrate->args->src[i] :
					migrate->args->dst[i];
			page = migrate_pfn_to_page(mpfn);
			mem = i915->mm.regions[mem_id];
			addr = page_to_pfn(page) - mem->devmem->pfn_first;
			addr <<= PAGE_SHIFT;
			addr += mem->region.start;
		}

		if (sg && (addr == (sg_dma_address(sg) + sg->length))) {
			sg->length += PAGE_SIZE;
			sg_dma_len(sg) += PAGE_SIZE;
			continue;
		}

		if (sg)
			sg_page_sizes |= sg->length;

		sg =  sg ? __sg_next(sg) : st.sgl;
		sg_dma_address(sg) = addr;
		sg_dma_len(sg) = PAGE_SIZE;
		sg->length = PAGE_SIZE;
		st.nents++;
	}

	sg_page_sizes |= sg->length;
	sg_mark_end(sg);
	i915_sg_trim(&st);

	if (mem_id != INTEL_REGION_SMEM)
		flags |= I915_GTT_SVM_LMEM;

	ret = svm_bind_addr(vm, ww, node->start, npages * PAGE_SIZE,
			    flags, &st, sg_page_sizes);
	sg_free_table(&st);

	return ret;
}

static void i915_devmem_unbind_addr(struct i915_devmem_migrate *migrate,
				    bool is_src)
{
	struct i915_address_space *vm = migrate->ce->vm;
	struct drm_mm_node *node;

	node = is_src ? &migrate->src_node : &migrate->dst_node;
	svm_unbind_addr(vm, node->start, migrate->npages * PAGE_SIZE);
	mutex_lock(&vm->mutex);
	drm_mm_remove_node(node);
	mutex_unlock(&vm->mutex);
}

static int i915_migrate_blitter_copy(struct i915_devmem_migrate *migrate,
				     struct i915_gem_ww_ctx *ww)
{
	struct drm_i915_private *i915 = migrate->i915;
	struct intel_memory_region *mem;
	enum intel_engine_id id;
	int ret;

	mem = i915->mm.regions[migrate->src_id];
	id = mem->gt->rsvd_bcs;
	migrate->ce = mem->gt->engine[id]->blitter_context;
	ret = i915_devmem_bind_addr(migrate, ww, true);
	if (unlikely(ret))
		return ret;

	ret = i915_devmem_bind_addr(migrate, ww, false);
	if (unlikely(ret)) {
		i915_devmem_unbind_addr(migrate, true);
		return ret;
	}

	DRM_DEBUG_DRIVER("npages %lld\n", migrate->npages);
	ret = i915_svm_copy_blt(migrate->ce,
				ww,
				migrate->src_node.start,
				migrate->dst_node.start,
				migrate->npages * PAGE_SIZE,
				&migrate->fence);
	if (unlikely(ret)) {
		i915_devmem_unbind_addr(migrate, false);
		i915_devmem_unbind_addr(migrate, true);
	}

	return ret;
}

static int i915_migrate_cpu_copy(struct i915_devmem_migrate *migrate)
{
	const unsigned long *src = migrate->args->src;
	unsigned long *dst = migrate->args->dst;
	struct drm_i915_private *i915 = migrate->i915;
	struct intel_memory_region *mem;
	void *src_vaddr, *dst_vaddr;
	u64 src_addr, dst_addr;
	struct page *page;
	int i, ret = 0;

	/* XXX: Copy multiple pages at a time */
	for (i = 0; !ret && i < migrate->npages; i++) {
		if (!dst[i])
			continue;

		page = migrate_pfn_to_page(dst[i]);
		mem = i915->mm.regions[migrate->dst_id];
		dst_addr = page_to_pfn(page);
		if (migrate->dst_id != INTEL_REGION_SMEM)
			dst_addr -= mem->devmem->pfn_first;
		dst_addr <<= PAGE_SHIFT;

		if (migrate->dst_id == INTEL_REGION_SMEM)
			dst_vaddr = phys_to_virt(dst_addr);
		else
			dst_vaddr = io_mapping_map_atomic_wc(&mem->iomap,
							     dst_addr);
		if (unlikely(!dst_vaddr))
			return -ENOMEM;

		page = migrate_pfn_to_page(src[i]);
		if (unlikely(!page))
			continue;

		mem = i915->mm.regions[migrate->src_id];
		src_addr = page_to_pfn(page);
		if (migrate->src_id != INTEL_REGION_SMEM)
			src_addr -= mem->devmem->pfn_first;
		src_addr <<= PAGE_SHIFT;

		if (migrate->src_id == INTEL_REGION_SMEM)
			src_vaddr = phys_to_virt(src_addr);
		else
			src_vaddr = io_mapping_map_atomic_wc(&mem->iomap,
							     src_addr);

		if (likely(src_vaddr))
			memcpy(dst_vaddr, src_vaddr, PAGE_SIZE);
		else
			ret = -ENOMEM;

		if (migrate->dst_id != INTEL_REGION_SMEM)
			io_mapping_unmap_atomic(dst_vaddr);

		if (migrate->src_id != INTEL_REGION_SMEM && src_vaddr)
			io_mapping_unmap_atomic(src_vaddr);

		DRM_DEBUG_DRIVER("src [%d] 0x%llx, dst [%d] 0x%llx\n",
				 migrate->src_id, src_addr,
				 migrate->dst_id, dst_addr);
	}

	return ret;
}

static int
i915_devmem_migrate_alloc_and_copy(struct i915_devmem_migrate *migrate,
				   struct i915_gem_ww_ctx *ww)
{
	struct drm_i915_private *i915 = migrate->i915;
	struct migrate_vma *args = migrate->args;
	struct device *dev = i915->drm.dev;
	struct intel_memory_region *mem;
	struct list_head blocks = {0};
	unsigned long i, npages, cnt;
	struct page *page;
	int ret;

	npages = (args->end - args->start) >> PAGE_SHIFT;
	DRM_DEBUG_DRIVER("start 0x%lx npages %ld\n", args->start, npages);
	/*
	 * XXX: Set proper condition for cpu vs blitter copy for performance,
	 * functionality and to avoid any deadlocks around blitter usage.
	 */
	migrate->blitter_copy = true;

	if (migrate->blitter_copy) {
		/* Allocate storage for DMA addresses, so we can unmap later. */
		migrate->host_dma = kcalloc(npages, sizeof(*migrate->host_dma),
					    GFP_KERNEL);
		if (unlikely(!migrate->host_dma))
			migrate->blitter_copy = false;
	}

	/* Check and DMA map source pages */
	for (i = 0, cnt = 0; i < npages; i++) {
		args->dst[i] = 0;
		page = migrate_pfn_to_page(args->src[i]);
		if (unlikely(!(args->src[i] & MIGRATE_PFN_MIGRATE))) {
			migrate->blitter_copy = false;
			continue;
		}
		if (unlikely(!page))
			migrate->blitter_copy = false;

		if (migrate->blitter_copy) {
			migrate->host_dma[i] =
					dma_map_page(dev, page, 0, PAGE_SIZE,
						     PCI_DMA_TODEVICE);
			if (unlikely(dma_mapping_error(dev,
						       migrate->host_dma[i]))) {
				migrate->blitter_copy = false;
				migrate->host_dma[i] = 0;
			}
		}

		args->dst[i] = MIGRATE_PFN_VALID;
		cnt++;
	}

	if (!cnt) {
		ret = -ENOMEM;
		goto migrate_out;
	}

	mem = i915->mm.regions[migrate->dst_id];
	ret = i915_devmem_page_alloc_locked(mem, cnt, &blocks, ww);
	if (unlikely(ret))
		goto migrate_out;

	/* Allocate device memory */
	for (i = 0, cnt = 0; i < npages; i++) {
		if (!args->dst[i])
			continue;

		page = i915_devmem_page_get_locked(mem, &blocks);
		if (unlikely(!page)) {
			WARN(1, "Failed to get dst page\n");
			migrate->blitter_copy = false;
			args->dst[i] = 0;
			continue;
		}

		cnt++;
		args->dst[i] = migrate_pfn(page_to_pfn(page)) |
			       MIGRATE_PFN_LOCKED;
	}

	if (!cnt) {
		ret = -ENOMEM;
		goto migrate_out;
	}

	/* Copy the pages */
	migrate->npages = npages;
	/* XXX: Flush the caches? */
	/* XXX: Fallback to cpu_copy if blitter_copy fails? */
	if (migrate->blitter_copy)
		ret = i915_migrate_blitter_copy(migrate, ww);
	else
		ret = i915_migrate_cpu_copy(migrate);
migrate_out:
	if (unlikely(ret)) {
		for (i = 0; i < npages; i++) {
			if (args->dst[i] & MIGRATE_PFN_LOCKED) {
				page = migrate_pfn_to_page(args->dst[i]);
				i915_devmem_page_free_locked(i915, page);
			}
			args->dst[i] = 0;
		}
	}

	if (unlikely(migrate->host_dma && (!migrate->blitter_copy || ret))) {
		for (i = 0; i < npages; i++) {
			if (migrate->host_dma[i])
				dma_unmap_page(dev, migrate->host_dma[i],
					       PAGE_SIZE, PCI_DMA_TODEVICE);
		}
		kfree(migrate->host_dma);
		migrate->host_dma = NULL;
	}

	return ret;
}

static void
i915_devmem_migrate_finalize_and_map(struct i915_devmem_migrate *migrate)
{
	struct drm_i915_private *i915 = migrate->i915;
	unsigned long i;

	if (!migrate->blitter_copy)
		return;

	DRM_DEBUG_DRIVER("npages %lld\n", migrate->npages);
	WARN(dma_fence_wait(migrate->fence, false) < 0,
	     "Wait for dma fence interrupted\n");

	i915_devmem_unbind_addr(migrate, true);
	i915_devmem_unbind_addr(migrate, false);

	for (i = 0; i < migrate->npages; i++)
		dma_unmap_page(i915->drm.dev, migrate->host_dma[i],
			       PAGE_SIZE, PCI_DMA_TODEVICE);

	kfree(migrate->host_dma);
	migrate->host_dma = NULL;
}

static int i915_devmem_migrate_chunk(struct i915_devmem_migrate *migrate,
				     struct i915_gem_ww_ctx *ww)
{
	int ret;

	ret = i915_devmem_migrate_alloc_and_copy(migrate, ww);
	if (!ret) {
		migrate_vma_pages(migrate->args);
		i915_devmem_migrate_finalize_and_map(migrate);
	}
	migrate_vma_finalize(migrate->args);

	return ret;
}

int i915_devmem_migrate_vma(struct intel_memory_region *mem,
				   struct i915_gem_ww_ctx *ww,
				   struct vm_area_struct *vma,
				   unsigned long start,
				   unsigned long end)
{
	unsigned long npages = (end - start) >> PAGE_SHIFT;
	unsigned long max = min_t(unsigned long, I915_BUDDY_MAX_PAGES, npages);
	struct i915_devmem_migrate migrate = {0};
	struct migrate_vma args = {
		.vma		= vma,
		.start		= start,
		.pgmap_owner    = mem->i915->drm.dev,
		.flags          = MIGRATE_VMA_SELECT_SYSTEM,
	};
	unsigned long c, i;
	int ret = 0;

	GEM_BUG_ON(mem->id >= INTEL_REGION_UNKNOWN);

	/* XXX: Opportunistically migrate additional pages? */
	DRM_DEBUG_DRIVER("start 0x%lx end 0x%lx\n", start, end);
	args.src = kcalloc(max, sizeof(args.src), GFP_KERNEL);
	if (unlikely(!args.src))
		return -ENOMEM;

	args.dst = kcalloc(max, sizeof(args.dst), GFP_KERNEL);
	if (unlikely(!args.dst)) {
		kfree(args.src);
		return -ENOMEM;
	}

	/* XXX: Support migrating from LMEM to SMEM */
	migrate.args = &args;
	migrate.i915 = mem->i915;
	migrate.src_id = INTEL_REGION_SMEM;
	migrate.dst_id = mem->id;
	for (i = 0; i < npages; i += c) {
		c = min_t(unsigned long, I915_BUDDY_MAX_PAGES, npages);
		args.end = start + (c << PAGE_SHIFT);
		ret = migrate_vma_setup(&args);
		if (unlikely(ret))
			goto migrate_done;
		if (args.cpages) {
			if (i915_devmem_migrate_chunk(&migrate, ww) == -EDEADLK) {
				ret = -EDEADLK;
				goto migrate_done;
			}
		}
		args.start = args.end;
	}
migrate_done:
	kfree(args.dst);
	kfree(args.src);
	return ret;
}

static vm_fault_t
i915_devmem_fault_alloc_and_copy(struct i915_devmem_migrate *migrate)
{
	struct device *dev = migrate->i915->drm.dev;
	struct migrate_vma *args = migrate->args;
	struct page *dpage, *spage;
	struct i915_gem_ww_ctx ww;
	int err;

	i915_gem_ww_ctx_init(&ww, true);

	DRM_DEBUG_DRIVER("start 0x%lx\n", args->start);
	/*
	 * XXX: Set proper condition for cpu vs blitter copy for performance,
	 * functionality and to avoid any deadlocks around blitter usage.
	 */
	migrate->blitter_copy = false;

	/* Allocate host page */
	spage = migrate_pfn_to_page(args->src[0]);
	if (unlikely(!spage || !(args->src[0] & MIGRATE_PFN_MIGRATE)))
		return 0;

	dpage = alloc_page_vma(GFP_HIGHUSER, args->vma, args->start);
	if (unlikely(!dpage))
		return VM_FAULT_SIGBUS;
	lock_page(dpage);

	/* DMA map host page */
	if (migrate->blitter_copy) {
		migrate->host_dma[0] = dma_map_page(dev, dpage, 0, PAGE_SIZE,
						    PCI_DMA_FROMDEVICE);
		if (unlikely(dma_mapping_error(dev, migrate->host_dma[0]))) {
			migrate->host_dma[0] = 0;
			err = -ENOMEM;
			goto fault_out;
		}
	}

	args->dst[0] = migrate_pfn(page_to_pfn(dpage)) | MIGRATE_PFN_LOCKED;

	/* Copy the pages */
	migrate->npages = 1;
retry:
	/* XXX: Fallback to cpu_copy if blitter_copy fails? */
	if (migrate->blitter_copy)
		err = i915_migrate_blitter_copy(migrate, &ww);
	else
		err = i915_migrate_cpu_copy(migrate);

	if (err == -EDEADLK) {
		err = i915_gem_ww_ctx_backoff(&ww);
		if (!err)
			goto retry;
	}

fault_out:
	if (unlikely(err)) {
		if (migrate->host_dma[0])
			dma_unmap_page(dev, migrate->host_dma[0],
				       PAGE_SIZE, PCI_DMA_FROMDEVICE);
		__free_page(dpage);
		args->dst[0] = 0;
		err = VM_FAULT_SIGBUS;
	}

	i915_gem_ww_ctx_fini(&ww);
	return err;
}

static void
i915_devmem_fault_finalize_and_map(struct i915_devmem_migrate *migrate)
{
	struct drm_i915_private *i915 = migrate->i915;

	if (!migrate->blitter_copy)
		return;

	DRM_DEBUG_DRIVER("\n");
	WARN(dma_fence_wait(migrate->fence, false) < 0,
	     "Wait for dma fence interrupted\n");

	i915_devmem_unbind_addr(migrate, true);
	i915_devmem_unbind_addr(migrate, false);

	dma_unmap_page(i915->drm.dev, migrate->host_dma[0],
		       PAGE_SIZE, PCI_DMA_FROMDEVICE);
}

static inline struct i915_devmem *page_to_devmem(struct page *page)
{
	return container_of(page->pgmap, struct i915_devmem, pagemap);
}

static vm_fault_t i915_devmem_migrate_to_ram(struct vm_fault *vmf)
{
	struct i915_devmem *devmem = page_to_devmem(vmf->page);
	struct drm_i915_private *i915 = devmem->mem->i915;
	struct i915_devmem_migrate migrate = {0};
	unsigned long src = 0, dst = 0;
	dma_addr_t dma_addr = 0;
	vm_fault_t ret;

	struct migrate_vma args = {
		.vma		= vmf->vma,
		.start		= vmf->address,
		.end		= vmf->address + PAGE_SIZE,
		.src		= &src,
		.dst		= &dst,
		.pgmap_owner	= i915->drm.dev,
		.flags		= MIGRATE_VMA_SELECT_DEVICE_PRIVATE,
	};

	GEM_BUG_ON(devmem->mem->id >= INTEL_REGION_UNKNOWN);

	/* XXX: Opportunistically migrate more pages? */
	DRM_DEBUG_DRIVER("addr 0x%lx\n", args.start);
	migrate.i915 = i915;
	migrate.args = &args;
	migrate.host_dma = &dma_addr;
	migrate.src_id = devmem->mem->id;
	migrate.dst_id = INTEL_REGION_SMEM;
	if (migrate_vma_setup(&args) < 0)
		return VM_FAULT_SIGBUS;
	if (!args.cpages)
		return 0;

	ret = i915_devmem_fault_alloc_and_copy(&migrate);
	if (ret || dst == 0)
		goto done;

	migrate_vma_pages(&args);
	i915_devmem_fault_finalize_and_map(&migrate);
done:
	migrate_vma_finalize(&args);
	return ret;
}

static void i915_devmem_page_free(struct page *page)
{
	struct i915_buddy_block *block = page->zone_device_data;
	struct intel_memory_region *mem = block->private;
	unsigned long i, max, weight;

	max = i915_buddy_block_size(&mem->mm, block) / PAGE_SIZE;
	i = page_to_pfn(page) - block->pfn_first;
	clear_bit(i, block->bitmap);
	weight = bitmap_weight(block->bitmap, max);
	DRM_DEBUG_DRIVER("%s pfn 0x%lx block weight 0x%lx\n",
			 "Freeing page", page_to_pfn(page), weight);
	if (!weight) {
		DRM_DEBUG_DRIVER("%s pfn_first 0x%lx off 0x%llx size 0x%llx\n",
				 "Freeing block", block->pfn_first,
				 i915_buddy_block_offset(block),
				 i915_buddy_block_size(&mem->mm, block));
		__intel_memory_region_put_block_buddy(block);
	}
}

static const struct dev_pagemap_ops i915_devmem_pagemap_ops = {
	.page_free = i915_devmem_page_free,
	.migrate_to_ram = i915_devmem_migrate_to_ram,
};

int i915_svm_devmem_add(struct intel_memory_region *mem)
{
	struct device *dev = &to_pci_dev(mem->i915->drm.dev)->dev;
	struct i915_devmem *devmem;
	struct resource *res;
	void *addr;
	int ret;

	devmem = kzalloc(sizeof(*devmem), GFP_KERNEL);
	if (!devmem)
		return -ENOMEM;

	devmem->mem = mem;
	res = devm_request_free_mem_region(dev, &iomem_resource,
					   resource_size(&mem->region));
	if (IS_ERR(res)) {
		ret = PTR_ERR(res);
		goto devm_err;
	}

	devmem->pagemap.type = MEMORY_DEVICE_PRIVATE;
	devmem->pagemap.range.start = res->start;
	devmem->pagemap.range.end = res->end;
	devmem->pagemap.nr_range = 1;
	devmem->pagemap.ops = &i915_devmem_pagemap_ops;
	devmem->pagemap.owner = mem->i915->drm.dev;
	addr = devm_memremap_pages(dev, &devmem->pagemap);
	if (IS_ERR(addr)) {
		ret = PTR_ERR(addr);
		goto devm_err;
	}

	devmem->pfn_first = res->start >> PAGE_SHIFT;
	devmem->pfn_last = res->end >> PAGE_SHIFT;
	mem->devmem = devmem;
	DRM_DEBUG_DRIVER("Added memory of gt %s start %llx to devmem. Remapped start %llx, pfn_first %lx\n",
			mem->gt->name, mem->region.start, res->start, devmem->pfn_first);
	return 0;
devm_err:
	kfree(devmem);
	return ret;
}

void i915_svm_devmem_remove(struct intel_memory_region *mem)
{
	if (mem->devmem) {
		devm_memunmap_pages(&to_pci_dev(mem->i915->drm.dev)->dev,
				    &mem->devmem->pagemap);
		kfree(mem->devmem);
		mem->devmem = NULL;
	}
}

int i915_svm_vm_prefetch(struct drm_i915_private *i915,
			struct prelim_drm_i915_gem_vm_prefetch *args)
{
	unsigned long addr, end, size = args->length;
	struct intel_memory_region *mem;
	struct i915_gem_ww_ctx ww;
	struct mm_struct *mm;
	u16 class, instance;
	int err = 0;

	DRM_DEBUG_DRIVER("start 0x%llx length 0x%llx region 0x%x\n",
			 args->start, args->length, args->region);
	/*
	 * XXX: should this be updated to use class:instance instead of opaque
	 * id?
	 */
	class = args->region >> 16;
	instance = args->region & 0xffff;
	if (class != INTEL_MEMORY_LOCAL)
		return -EINVAL;

	mem = intel_memory_region_lookup(i915, class, instance);
	if (!mem)
		return -EINVAL;
	else if (!mem->devmem)
		return -ERANGE;

	mm = get_task_mm(current);
	mmap_read_lock(mm);

	i915_gem_ww_ctx_init(&ww, true);

	trace_i915_vm_prefetch(mem, 0, args->start, args->length);
retry:
	for (addr = args->start, end = args->start + size; addr < end;) {
		struct vm_area_struct *vma;
		unsigned long next;

		vma = find_vma_intersection(mm, addr, end);
		if (!vma)
			break;

		addr &= PAGE_MASK;
		next = min(vma->vm_end, end);
		next = round_up(next, PAGE_SIZE);

		/*
		 * XXX: This is a best effort so we ignore errors(expect in the
		 * case of ww backoff). It's not clear what the desired
		 * behaviour here is with ww + migrate_vma...
		 */
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

	mmap_read_unlock(mm);
	mmput(mm);
	return 0;
}
