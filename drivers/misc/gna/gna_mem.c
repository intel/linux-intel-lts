// SPDX-License-Identifier: GPL-2.0-only
// Copyright(c) 2017-2020 Intel Corporation

#include <linux/device.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/sched/mm.h>
#include <linux/sched/task.h>
#include <linux/slab.h>
#include <linux/swap.h>
#include <linux/types.h>

#include "gna.h"

#include "gna_drv.h"
#include "gna_mem.h"
#include "gna_score.h"
#include "gna_request.h"

static void gna_mmu_init(struct gna_private *gna_priv)
{
	dma_addr_t pagetable_dma;
	struct gna_mmu_object *mmu;
	u32 *pgdirn;
	int i;

	mmu = &gna_priv->mmu;

	pgdirn = mmu->hwdesc->mmu.pagedir_n;

	for (i = 0; i < mmu->num_pagetables; i++) {
		pagetable_dma = mmu->pagetables_dma[i];
		pgdirn[i] = pagetable_dma >> PAGE_SHIFT;
		dev_dbg(&gna_priv->dev, "pagetable %#x mapped at %#lx\n",
			(u32)pagetable_dma, (uintptr_t)&pgdirn[i]);
	}

	for (; i < GNA_PGDIRN_LEN; i++)
		pgdirn[i] = GNA_PGDIR_INVALID;
}

/* descriptor and page tables allocation */
int gna_mmu_alloc(struct gna_private *gna_priv)
{
	struct gna_mmu_object *mmu;
	int desc_size;
	int i;

	mmu = &gna_priv->mmu;

	dev_dbg(&gna_priv->dev, "%s: enter\n", __func__);

	desc_size = ROUND_UP(gna_priv->info.desc_info.desc_size, PAGE_SIZE);

	mmu->hwdesc = dma_alloc_coherent(&gna_priv->pdev->dev,
				     desc_size, &mmu->hwdesc_dma, GFP_KERNEL);
	if (!mmu->hwdesc) {
		dev_err(&gna_priv->dev, "gna base descriptor alloc fail\n");
		goto end;
	}

	memset(mmu->hwdesc, 0, desc_size);

	mmu->num_pagetables = gna_priv->info.num_pagetables;

	mmu->pagetables_dma = kmalloc_array(mmu->num_pagetables,
			sizeof(*mmu->pagetables_dma), GFP_KERNEL);
	if (!mmu->pagetables_dma)
		goto err_free_descriptor;

	mmu->pagetables = kmalloc_array(mmu->num_pagetables,
			sizeof(*mmu->pagetables), GFP_KERNEL);

	if (!mmu->pagetables)
		goto err_free_pagetables_dma;

	for (i = 0; i < mmu->num_pagetables; i++) {
		mmu->pagetables[i] = dma_alloc_coherent(&gna_priv->pdev->dev,
			PAGE_SIZE, &mmu->pagetables_dma[i], GFP_KERNEL);
		if (!mmu->pagetables[i]) {
			dev_err(&gna_priv->dev,
				"gna page table %d alloc fail\n", i);
			goto err_free_mmu;
		}
		memset(mmu->pagetables[i], 0, PAGE_SIZE);
		dev_dbg(&gna_priv->dev,
			"pagetable allocated %#lx\n",
			(uintptr_t)mmu->pagetables[i]);
	}

	gna_mmu_init(gna_priv);

	dev_dbg(&gna_priv->dev, "%s: exit\n", __func__);

	return 0;

err_free_mmu:
	while (i--) {
		pci_free_consistent(gna_priv->pdev, PAGE_SIZE,
			mmu->pagetables[i], mmu->pagetables_dma[i]);
		mmu->pagetables[i] = NULL;
		mmu->pagetables_dma[i] = 0;
	}

	kfree(mmu->pagetables);
	mmu->pagetables = NULL;
	mmu->num_pagetables = 0;

err_free_pagetables_dma:
	kfree(mmu->pagetables_dma);
	mmu->pagetables_dma = 0;

err_free_descriptor:
	pci_free_consistent(gna_priv->pdev, desc_size,
			mmu->hwdesc, mmu->hwdesc_dma);
	mmu->hwdesc = NULL;
	mmu->hwdesc_dma = 0;

end:
	return -ENOMEM;
}

void gna_mmu_free(struct gna_private *gna_priv)
{
	struct gna_mmu_object *mmu;
	int desc_size;
	int i;

	dev_dbg(&gna_priv->dev, "%s enter\n", __func__);

	mmu = &gna_priv->mmu;
	mutex_lock(&gna_priv->mmu_lock);

	for (i = 0; i < mmu->num_pagetables; i++) {
		pci_free_consistent(gna_priv->pdev, PAGE_SIZE,
			mmu->pagetables[i], mmu->pagetables_dma[i]);
		mmu->pagetables[i] = NULL;
		mmu->pagetables_dma[i] = 0;
	}

	kfree(mmu->pagetables);
	mmu->pagetables = NULL;

	kfree(mmu->pagetables_dma);
	mmu->pagetables_dma = 0;

	desc_size = ROUND_UP(gna_priv->info.desc_info.desc_size, PAGE_SIZE);
	pci_free_consistent(gna_priv->pdev, desc_size,
	    mmu->hwdesc, mmu->hwdesc_dma);
	mmu->hwdesc = NULL;
	mmu->hwdesc_dma = 0;

	mutex_unlock(&gna_priv->mmu_lock);

	dev_dbg(&gna_priv->dev, "%s: exit\n", __func__);
}

void gna_mmu_add(struct gna_private *gna_priv,
	struct gna_memory_object *mo)
{
	struct scatterlist *sgl;
	struct gna_mmu_object *mmu;
	dma_addr_t sg_page;
	int sg_page_len;
	u32 *pagetable;
	u32 mmu_page;
	int sg_pages;
	int i;
	int j;

	mmu = &gna_priv->mmu;

	mutex_lock(&gna_priv->mmu_lock);
	j = mmu->filled_pages;

	sgl = mo->sgt->sgl;
	sg_page = sg_dma_address(sgl);
	sg_page_len = ROUND_UP(sg_dma_len(sgl), PAGE_SIZE) >> PAGE_SHIFT;
	sg_pages = 0;

	for (i = mmu->filled_pts; i < mmu->num_pagetables; i++) {
		if (sgl == NULL)
			break;

		pagetable = mmu->pagetables[i];

		for (j = mmu->filled_pages; j < GNA_PAGE_TABLE_LENGTH; j++) {
			mmu_page = sg_page >> PAGE_SHIFT;
			pagetable[j] = mmu_page;

			mmu->filled_pages++;
			sg_page += PAGE_SIZE;
			sg_pages++;
			if (sg_pages == sg_page_len) {
				sgl = sg_next(sgl);
				if (sgl == NULL)
					break;

				sg_page = sg_dma_address(sgl);
				sg_page_len =
					ROUND_UP(sg_dma_len(sgl), PAGE_SIZE)
						>> PAGE_SHIFT;
				sg_pages = 0;
			}
		}

		if (j == GNA_PAGE_TABLE_LENGTH) {
			mmu->filled_pages = 0;
			mmu->filled_pts++;
		}
	}

	mmu->hwdesc->mmu.vamaxaddr =
		(mmu->filled_pts * PAGE_SIZE * GNA_PGDIR_ENTRIES) +
		(mmu->filled_pages * PAGE_SIZE) - 1;
	dev_dbg(&gna_priv->dev, "vamaxaddr set to %u\n",
			mmu->hwdesc->mmu.vamaxaddr);
	mutex_unlock(&gna_priv->mmu_lock);
}

void gna_mmu_clear(struct gna_private *gna_priv)
{
	struct gna_mmu_object *mmu;
	int i;

	mmu = &gna_priv->mmu;
	mutex_lock(&gna_priv->mmu_lock);

	for (i = 0; i < mmu->filled_pts; i++)
		memset(mmu->pagetables[i], 0, PAGE_SIZE);

	if (mmu->filled_pages > 0)
		memset(mmu->pagetables[mmu->filled_pts], 0,
			mmu->filled_pages * GNA_PT_ENTRY_SIZE);

	mmu->filled_pts = 0;
	mmu->filled_pages = 0;
	mmu->hwdesc->mmu.vamaxaddr = 0;

	mutex_unlock(&gna_priv->mmu_lock);
}

int gna_buffer_get_size(u64 offset, u64 size)
{
	u64 page_offset;

	page_offset = offset & ~PAGE_MASK;
	return ROUND_UP(page_offset + size, PAGE_SIZE);
}

/* must be called with gna_memory_object page_lock held */
static int gna_get_pages(struct gna_memory_object *mo, u64 offset, u64 size)
{
	struct gna_private *gna_priv;
	struct mm_struct *mm;
	struct page **pages;
	struct sg_table *sgt;
	u64 effective_address;
	int effective_size;
	int num_pinned;
	int num_pages;
	int skip_size;
	int sgcount;
	int ret;

	ret = 0;
	gna_priv = mo->gna_priv;

	if (mo->pages) {
		dev_warn(&gna_priv->dev, "pages are already pinned\n");
		return -EFAULT;
	}

	/* using vmalloc because num_pages can be large */
	skip_size = ROUND_DOWN(offset, PAGE_SIZE);
	effective_address = mo->user_address + skip_size;
	dev_dbg(&gna_priv->dev, "user address %llx\n", mo->user_address);
	dev_dbg(&gna_priv->dev, "effective user address %llx\n", effective_address);

	effective_size = gna_buffer_get_size(offset, size);

	num_pages = effective_size >> PAGE_SHIFT;
	dev_dbg(&gna_priv->dev, "allocating %d pages\n", num_pages);
	pages = kvmalloc_array(num_pages, sizeof(struct page *), GFP_KERNEL);
	if (!pages) {
		ret = -ENOMEM;
		dev_err(&gna_priv->dev, "no memory for pages\n");
		goto err_exit;
	}

	dev_dbg(&gna_priv->dev, "pages allocated\n");

	get_task_struct(mo->task);
	mm = get_task_mm(mo->task);
	if (!mm) {
		ret = -ENOENT;
		goto err_put_task;
	}
	down_read(&mm->mmap_sem);
	num_pinned = get_user_pages_remote(mo->task, mm,
			effective_address, num_pages, 1, pages, NULL, NULL);
	up_read(&mm->mmap_sem);
	mmput(mm);

	if (num_pinned <= 0) {
		ret = num_pinned;
		dev_err(&gna_priv->dev,
			"function get_user_pages_fast() failed\n");
		goto err_free_pages;
	}
	if (num_pinned < num_pages) {
		ret = -EFAULT;
		dev_err(&gna_priv->dev,
			"function get_user_pages_fast() pinned less pages\n");
		goto err_free_pages;
	}

	dev_dbg(&gna_priv->dev, "get user pages success %d\n", ret);

	sgt = kmalloc(sizeof(struct sg_table), GFP_KERNEL);
	if (!sgt) {
		ret = -ENOMEM;
		dev_err(&gna_priv->dev,
			"could not allocate memory for scatter-gather table\n");
		goto err_put_pages;
	}

	dev_dbg(&gna_priv->dev, "sgt allocated\n");

	ret = sg_alloc_table_from_pages(
			sgt, pages, num_pinned, 0, mo->memory_size, GFP_KERNEL);
	if (ret) {
		dev_err(&gna_priv->dev, "could not alloc scatter list\n");
		goto err_free_sgt;
	}

	if (IS_ERR(sgt->sgl)) {
		dev_err(&gna_priv->dev, "sgl allocation failed\n");
		ret = PTR_ERR(sgt->sgl);
		goto err_free_sgt;
	}

	dev_dbg(&gna_priv->dev, "sgl allocated\n");

	sgcount = pci_map_sg(
		gna_priv->pdev, sgt->sgl, sgt->nents, PCI_DMA_BIDIRECTIONAL);
	if (sgcount <= 0) {
		dev_err(&gna_priv->dev, "could not map scatter gather list\n");
		goto err_free_sgl;
	}

	dev_dbg(&gna_priv->dev, "mapped scatter gather list\n");

	mo->sgt = sgt;
	mo->sgcount = sgcount;
	mo->pages = pages;
	mo->num_pinned = num_pinned;

	return 0;

err_free_sgl:
	sg_free_table(sgt);

err_free_sgt:
	kfree(sgt);

err_put_pages:
	release_pages(pages, num_pinned);

err_free_pages:
	kvfree(pages);

err_put_task:
	put_task_struct(mo->task);

err_exit:
	return ret;
}

/* must be called with gna_memory_object page_lock held */
static void gna_put_pages(struct gna_memory_object *mo)
{
	struct sg_table *sgt;
	struct gna_private *gna_priv;

	gna_priv = mo->gna_priv;

	if (!mo->pages) {
		dev_warn(&gna_priv->dev, "memory object has no pages %llu\n",
				mo->memory_id);
		return;
	}

	sgt = mo->sgt;

	pci_unmap_sg(
		gna_priv->pdev, sgt->sgl, sgt->nents, PCI_DMA_BIDIRECTIONAL);
	sg_free_table(sgt);
	kfree(sgt);
	mo->sgt = NULL;
	mo->sgcount = 0;

	release_pages(mo->pages, mo->num_pinned);
	kvfree(mo->pages);
	mo->pages = NULL;
	mo->num_pinned = 0;

	put_task_struct(mo->task);
}

void gna_memory_free(struct gna_private *gna_priv, struct gna_memory_object *mo)
{
	mutex_lock(&gna_priv->memidr_lock);
	idr_remove(&gna_priv->memory_idr, mo->memory_id);
	mutex_unlock(&gna_priv->memidr_lock);

#if defined(CONFIG_MMU_NOTIFIER)
	mutex_lock(&mo->mn_lock);
	if (mo->gna_mn) {
		mmu_notifier_unregister(&mo->gna_mn->mn, mo->gna_mn->mm);
		kfree(mo->gna_mn);
		mo->gna_mn = NULL;
	}
	mutex_unlock(&mo->mn_lock);
#endif

	cancel_work_sync(&mo->work);
	kfree(mo);
}

static void gna_memory_release(struct work_struct *work)
{
	struct gna_memory_object *mo;
	struct gna_private *gna_priv;

	mo = container_of(work, struct gna_memory_object, work);
	gna_priv = mo->gna_priv;

	dev_dbg(&gna_priv->dev, "%s: enter\n", __func__);

	gna_delete_memory_requests(mo->memory_id, gna_priv);

	mo->user_ptr = NULL;

	wake_up_interruptible(&mo->waitq);

	dev_dbg(&gna_priv->dev, "%s: exit\n", __func__);
}

static const struct gna_memory_operations memory_ops = {
	.get_pages = gna_get_pages,
	.put_pages = gna_put_pages
};

#if defined(CONFIG_MMU_NOTIFIER)
static int gna_userptr_mn_invalidate_range_start(
	struct mmu_notifier *mn,
	const struct mmu_notifier_range *mn_range)
{
	struct gna_mmu_notifier *gna_mn;
	struct gna_memory_object *mo;
	struct gna_private *gna_priv;

	gna_mn = container_of(mn, struct gna_mmu_notifier, mn);
	gna_priv = gna_mn->gna_priv;

	mo = gna_mn->mo;

	dev_dbg(&gna_priv->dev,
			"memory invalidated, size: %lu, memory size: %llu\n",
			mn_range->end - mn_range->start, mo->memory_size);

	return 0;
}
#endif

#if defined(CONFIG_MMU_NOTIFIER)
static const struct mmu_notifier_ops gna_userptr_mn_ops = {
	.invalidate_range_start = gna_userptr_mn_invalidate_range_start,
};
#endif

int gna_priv_userptr(struct gna_file_private *file_priv,
	union gna_memory_map *gna_mem)
{
	struct gna_memory_object *mo;
	struct gna_private *gna_priv;
	struct mm_struct *mm;
	int ret;

	ret = 0;

	gna_priv = file_priv->gna_priv;

	if (!gna_mem->in.size) {
		dev_err(&gna_priv->dev, "invalid user memory size\n");
		return -EINVAL;
	}

	if (!access_ok(u64_to_user_ptr(gna_mem->in.address), gna_mem->in.size)) {
		dev_err(&gna_priv->dev, "invalid user pointer\n");
		return -EINVAL;
	}

	mo = kzalloc(sizeof(*mo), GFP_KERNEL);
	if (!mo)
		return -ENOMEM;

	mo->fd = file_priv->fd;
	mo->gna_priv = gna_priv;
	mo->ops = &memory_ops;
	mo->user_address = gna_mem->in.address;
	mo->memory_size = gna_mem->in.size;
	mo->user_ptr = u64_to_user_ptr(gna_mem->in.address);
	mo->num_pages = ROUND_UP(gna_mem->in.size, PAGE_SIZE) >> PAGE_SHIFT;
	mo->task = current;
	INIT_WORK(&mo->work, gna_memory_release);
	init_waitqueue_head(&mo->waitq);
	mutex_init(&mo->page_lock);

	mutex_lock(&gna_priv->memidr_lock);
	mo->memory_id = idr_alloc(&gna_priv->memory_idr, mo, 1, 0, GFP_KERNEL);
	mutex_unlock(&gna_priv->memidr_lock);

	if (mo->memory_id < 0) {
		dev_err(&gna_priv->dev, "idr allocation for memory failed\n");
		ret = -EFAULT;
		goto err_free_mo;
	}

	mutex_lock(&file_priv->memlist_lock);
	list_add_tail(&mo->file_mem_list, &file_priv->memory_list);
	mutex_unlock(&file_priv->memlist_lock);

	gna_mem->out.memory_id = mo->memory_id;
	dev_dbg(&gna_priv->dev, "memory id allocated: %llu\n", mo->memory_id);

#if defined(CONFIG_MMU_NOTIFIER)
	/* register mmu notifier to avoid use-after-free */
	mutex_init(&mo->mn_lock);
	mm = current->mm;

	mutex_lock(&mo->mn_lock);
	mo->gna_mn = kzalloc(sizeof(struct gna_mmu_notifier), GFP_KERNEL);
	mo->gna_mn->file_priv = file_priv;
	mo->gna_mn->gna_priv = gna_priv;
	mo->gna_mn->mo = mo;
	mo->gna_mn->mn.ops = &gna_userptr_mn_ops;
	mo->gna_mn->mm = mm;

	down_write(&mm->mmap_sem);
	ret = __mmu_notifier_register(&mo->gna_mn->mn, mm);
	up_write(&mm->mmap_sem);

	if (ret) {
		kfree(mo->gna_mn);
		mo->gna_mn = NULL;
	}
	mutex_unlock(&mo->mn_lock);
#endif

	return 0;

err_free_mo:
	kfree(mo);
	return ret;
}
