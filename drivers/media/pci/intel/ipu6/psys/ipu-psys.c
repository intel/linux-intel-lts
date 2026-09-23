// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/highmem.h>
#include <linux/init_task.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/version.h>
#include <linux/poll.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0)
#include <linux/sched.h>
#else
#include <uapi/linux/sched/types.h>
#endif
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 8, 0)
#include <linux/dma-attrs.h>
#else
#include <linux/dma-mapping.h>
#endif

#include <uapi/linux/ipu-psys.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
#include "ipu.h"
#include "ipu-mmu.h"
#include "ipu-bus.h"
#include "ipu-platform.h"
#include "ipu-buttress.h"
#include "ipu-cpd.h"
#include "ipu-fw-psys.h"
#include "ipu-psys.h"
#include "ipu-platform-psys.h"
#include "ipu-platform-regs.h"
#include "ipu-fw-com.h"
#else
#include "ipu6.h"
#include "ipu6-mmu.h"
#include "ipu6-bus.h"
#include "ipu6-buttress.h"
#include "ipu6-cpd.h"
#include "ipu-fw-psys.h"
#include "ipu-psys.h"
#include "ipu6-platform-regs.h"
#include "ipu6-fw-com.h"
#endif

static bool async_fw_init;
module_param(async_fw_init, bool, 0664);
MODULE_PARM_DESC(async_fw_init, "Enable asynchronous firmware initialization");

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 10, 0)
#define SYSCOM_BUTTRESS_FW_PARAMS_PSYS_OFFSET	7

#endif
#define IPU_PSYS_NUM_DEVICES		4

#define IPU_PSYS_MAX_NUM_DESCS		1024
#define IPU_PSYS_MAX_NUM_BUFS		1024
#define IPU_PSYS_MAX_NUM_BUFS_LRU	12

static int psys_runtime_pm_resume(struct device *dev);
static int psys_runtime_pm_suspend(struct device *dev);

static dev_t ipu_psys_dev_t;
static DECLARE_BITMAP(ipu_psys_devices, IPU_PSYS_NUM_DEVICES);
static DEFINE_MUTEX(ipu_psys_mutex);

static struct fw_init_task {
	struct delayed_work work;
	struct ipu_psys *psys;
} fw_init_task;

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
static void ipu_psys_remove(struct ipu_bus_device *adev);
#else
static void ipu6_psys_remove(struct auxiliary_device *auxdev);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
static struct bus_type ipu_psys_bus = {
	.name = IPU_PSYS_NAME,
};
#else
static const struct bus_type ipu_psys_bus = {
	.name = "intel-ipu6-psys",
};
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 10, 0)
#define PKG_DIR_ENT_LEN_FOR_PSYS	2
#define PKG_DIR_SIZE_MASK_FOR_PSYS	GENMASK(23, 0)

enum ipu6_version ipu_ver;

static u32 ipu6_cpd_pkg_dir_get_address(const u64 *pkg_dir, int pkg_dir_idx)
{
	return pkg_dir[++pkg_dir_idx * PKG_DIR_ENT_LEN_FOR_PSYS];
}

static u32 ipu6_cpd_pkg_dir_get_num_entries(const u64 *pkg_dir)
{
	return pkg_dir[1];
}

static u32 ipu6_cpd_pkg_dir_get_size(const u64 *pkg_dir, int pkg_dir_idx)
{
	return pkg_dir[++pkg_dir_idx * PKG_DIR_ENT_LEN_FOR_PSYS + 1] &
	       PKG_DIR_SIZE_MASK_FOR_PSYS;
}

#define PKG_DIR_ID_SHIFT		48
#define PKG_DIR_ID_MASK			0x7f

static u32 ipu6_cpd_pkg_dir_get_type(const u64 *pkg_dir, int pkg_dir_idx)
{
	return pkg_dir[++pkg_dir_idx * PKG_DIR_ENT_LEN_FOR_PSYS + 1] >>
	    PKG_DIR_ID_SHIFT & PKG_DIR_ID_MASK;
}

#endif
/*
 * These are some trivial wrappers that save us from open-coding some
 * common patterns and also that's were we have some checking (for the
 * time being)
 */
static void ipu_desc_add(struct ipu_psys_fh *fh, struct ipu_psys_desc *desc)
{
	fh->num_descs++;

	WARN_ON_ONCE(fh->num_descs >= IPU_PSYS_MAX_NUM_DESCS);
	list_add(&desc->list, &fh->descs_list);
}

static void ipu_desc_del(struct ipu_psys_fh *fh, struct ipu_psys_desc *desc)
{
	fh->num_descs--;
	list_del_init(&desc->list);
}

static void ipu_buffer_add(struct ipu_psys_fh *fh,
			   struct ipu_psys_kbuffer *kbuf)
{
	fh->num_bufs++;

	WARN_ON_ONCE(fh->num_bufs >= IPU_PSYS_MAX_NUM_BUFS);
	list_add(&kbuf->list, &fh->bufs_list);
}

static void ipu_buffer_del(struct ipu_psys_fh *fh,
			   struct ipu_psys_kbuffer *kbuf)
{
	fh->num_bufs--;
	list_del_init(&kbuf->list);
}

static void ipu_buffer_lru_add(struct ipu_psys_fh *fh,
			       struct ipu_psys_kbuffer *kbuf)
{
	fh->num_bufs_lru++;
	list_add_tail(&kbuf->list, &fh->bufs_lru);
}

static void ipu_buffer_lru_del(struct ipu_psys_fh *fh,
			       struct ipu_psys_kbuffer *kbuf)
{
	fh->num_bufs_lru--;
	list_del_init(&kbuf->list);
}

static struct ipu_psys_kbuffer *ipu_psys_kbuffer_alloc(void)
{
	struct ipu_psys_kbuffer *kbuf;

	kbuf = kzalloc(sizeof(*kbuf), GFP_KERNEL);
	if (!kbuf)
		return NULL;

	atomic_set(&kbuf->map_count, 0);
	INIT_LIST_HEAD(&kbuf->list);
	return kbuf;
}

static struct ipu_psys_desc *ipu_psys_desc_alloc(int fd)
{
	struct ipu_psys_desc *desc;

	desc = kzalloc(sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return NULL;

	desc->fd = fd;
	INIT_LIST_HEAD(&desc->list);
	return desc;
}

struct ipu_psys_pg *__get_pg_buf(struct ipu_psys *psys, size_t pg_size)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 10, 0) && LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 5)
	struct device *dev = &psys->adev->auxdev.dev;
#endif
	struct ipu_psys_pg *kpg;
	unsigned long flags;

	spin_lock_irqsave(&psys->pgs_lock, flags);
	list_for_each_entry(kpg, &psys->pgs, list) {
		if (!kpg->pg_size && kpg->size >= pg_size) {
			kpg->pg_size = pg_size;
			spin_unlock_irqrestore(&psys->pgs_lock, flags);
			return kpg;
		}
	}
	spin_unlock_irqrestore(&psys->pgs_lock, flags);
	/* no big enough buffer available, allocate new one */
	kpg = kzalloc(sizeof(*kpg), GFP_KERNEL);
	if (!kpg)
		return NULL;

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
	kpg->pg = dma_alloc_attrs(&psys->adev->dev, pg_size,
				  &kpg->pg_dma_addr, GFP_KERNEL, 0);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 5)
	kpg->pg = dma_alloc_attrs(dev, pg_size,  &kpg->pg_dma_addr,
				  GFP_KERNEL, 0);
#else
	kpg->pg = ipu6_dma_alloc(psys->adev, pg_size,  &kpg->pg_dma_addr,
				 GFP_KERNEL, 0);
#endif
	if (!kpg->pg) {
		kfree(kpg);
		return NULL;
	}

	kpg->pg_size = pg_size;
	kpg->size = pg_size;
	spin_lock_irqsave(&psys->pgs_lock, flags);
	list_add(&kpg->list, &psys->pgs);
	spin_unlock_irqrestore(&psys->pgs_lock, flags);

	return kpg;
}

static struct ipu_psys_desc *psys_desc_lookup(struct ipu_psys_fh *fh, int fd)
{
	struct ipu_psys_desc *desc;

	list_for_each_entry(desc, &fh->descs_list, list) {
		if (desc->fd == fd)
			return desc;
	}

	return NULL;
}

static bool dmabuf_cmp(struct dma_buf *lb, struct dma_buf *rb)
{
	return lb == rb && lb->size == rb->size;
}

static struct ipu_psys_kbuffer *psys_buf_lookup(struct ipu_psys_fh *fh, int fd)
{
	struct ipu_psys_kbuffer *kbuf;
	struct dma_buf *dma_buf;

	dma_buf = dma_buf_get(fd);
	if (IS_ERR(dma_buf))
		return NULL;

	/*
	 * First lookup so-called `active` list, that is the list of
	 * referenced buffers
	 */
	list_for_each_entry(kbuf, &fh->bufs_list, list) {
		if (dmabuf_cmp(kbuf->dbuf, dma_buf)) {
			dma_buf_put(dma_buf);
			return kbuf;
		}
	}

	/*
	 * We didn't find anything on the `active` list, try the LRU list
	 * (list of unreferenced buffers) and possibly resurrect a buffer
	 */
	list_for_each_entry(kbuf, &fh->bufs_lru, list) {
		if (dmabuf_cmp(kbuf->dbuf, dma_buf)) {
			dma_buf_put(dma_buf);
			ipu_buffer_lru_del(fh, kbuf);
			ipu_buffer_add(fh, kbuf);
			return kbuf;
		}
	}

	dma_buf_put(dma_buf);
	return NULL;
}

struct ipu_psys_kbuffer *ipu_psys_lookup_kbuffer(struct ipu_psys_fh *fh, int fd)
{
	struct ipu_psys_desc *desc;

	desc = psys_desc_lookup(fh, fd);
	if (!desc)
		return NULL;

	return desc->kbuf;
}

struct ipu_psys_kbuffer *
ipu_psys_lookup_kbuffer_by_kaddr(struct ipu_psys_fh *fh, void *kaddr)
{
	struct ipu_psys_kbuffer *kbuffer;

	list_for_each_entry(kbuffer, &fh->bufs_list, list) {
		if (kbuffer->kaddr == kaddr)
			return kbuffer;
	}

	return NULL;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
static int ipu_psys_get_userpages(struct ipu_dma_buf_attach *attach)
{
	struct vm_area_struct *vma;
	unsigned long start, end;
	int npages, array_size;
	struct page **pages;
	struct sg_table *sgt;
	int nr = 0;
	int ret = -ENOMEM;

	start = (unsigned long)attach->userptr;
	end = PAGE_ALIGN(start + attach->len);
	npages = (end - (start & PAGE_MASK)) >> PAGE_SHIFT;
	array_size = npages * sizeof(struct page *);

	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return -ENOMEM;

	if (attach->npages != 0) {
		pages = attach->pages;
		npages = attach->npages;
		attach->vma_is_io = 1;
		goto skip_pages;
	}

	pages = kvzalloc(array_size, GFP_KERNEL);
	if (!pages)
		goto free_sgt;

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 9, 0)
	down_read(&current->mm->mmap_sem);
#else
	mmap_read_lock(current->mm);
#endif
	vma = find_vma(current->mm, start);
	if (!vma) {
		ret = -EFAULT;
		goto error_up_read;
	}

	/*
	 * For buffers from Gralloc, VM_PFNMAP is expected,
	 * but VM_IO is set. Possibly bug in Gralloc.
	 */
	attach->vma_is_io = vma->vm_flags & (VM_IO | VM_PFNMAP);

	if (attach->vma_is_io) {
		unsigned long io_start = start;

		if (vma->vm_end < start + attach->len) {
			dev_err(attach->dev,
				"vma at %lu is too small for %llu bytes\n",
				start, attach->len);
			ret = -EFAULT;
			goto error_up_read;
		}

		for (nr = 0; nr < npages; nr++, io_start += PAGE_SIZE) {
			unsigned long pfn;

			ret = follow_pfn(vma, io_start, &pfn);
			if (ret)
				goto error_up_read;
			pages[nr] = pfn_to_page(pfn);
		}
	} else {
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 6, 0)
		nr = get_user_pages(current, current->mm, start & PAGE_MASK,
				    npages, 1, 0, pages, NULL);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
		nr = get_user_pages(start & PAGE_MASK, npages,
				    1, 0, pages, NULL);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(6, 5, 0)
		nr = get_user_pages(start & PAGE_MASK, npages,
				    FOLL_WRITE,
				    pages, NULL);
#else
		nr = get_user_pages(start & PAGE_MASK, npages,
				    FOLL_WRITE, pages);
#endif
		if (nr < npages)
			goto error_up_read;
	}
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 9, 0)
	up_read(&current->mm->mmap_sem);
#else
	mmap_read_unlock(current->mm);
#endif

	attach->pages = pages;
	attach->npages = npages;

skip_pages:
	ret = sg_alloc_table_from_pages(sgt, pages, npages,
					start & ~PAGE_MASK, attach->len,
					GFP_KERNEL);
	if (ret < 0)
		goto error;

	attach->sgt = sgt;

	return 0;

error_up_read:
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 9, 0)
	up_read(&current->mm->mmap_sem);
#else
	mmap_read_unlock(current->mm);
#endif
error:
	if (!attach->vma_is_io)
		while (nr > 0)
			put_page(pages[--nr]);

	if (array_size <= PAGE_SIZE)
		kfree(pages);
	else
		vfree(pages);
free_sgt:
	kfree(sgt);

	dev_err(attach->dev, "failed to get userpages:%d\n", ret);

	return ret;
}

static void ipu_psys_put_userpages(struct ipu_dma_buf_attach *attach)
{
	if (!attach || !attach->userptr || !attach->sgt)
		return;

	if (!attach->vma_is_io) {
		int i = attach->npages;

		while (--i >= 0) {
			set_page_dirty_lock(attach->pages[i]);
			put_page(attach->pages[i]);
		}
	}

	kvfree(attach->pages);

	sg_free_table(attach->sgt);
	kfree(attach->sgt);
	attach->sgt = NULL;
}
#else
static int ipu_psys_get_userpages(struct ipu_dma_buf_attach *attach)
{
	struct vm_area_struct *vma;
	unsigned long start, end;
	int npages, array_size;
	struct page **pages;
	struct sg_table *sgt;
	int ret = -ENOMEM;
	int nr = 0;
	u32 flags;

	start = attach->userptr;
	end = PAGE_ALIGN(start + attach->len);
	npages = (end - (start & PAGE_MASK)) >> PAGE_SHIFT;
	array_size = npages * sizeof(struct page *);

	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return -ENOMEM;

	WARN_ON_ONCE(attach->npages);

	pages = kvzalloc(array_size, GFP_KERNEL);
	if (!pages)
		goto free_sgt;

	mmap_read_lock(current->mm);
	vma = vma_lookup(current->mm, start);
	if (unlikely(!vma)) {
		ret = -EFAULT;
		goto error_up_read;
	}
	mmap_read_unlock(current->mm);

	flags = FOLL_WRITE | FOLL_FORCE | FOLL_LONGTERM;
	nr = pin_user_pages_fast(start & PAGE_MASK, npages,
				 flags, pages);
	if (nr < npages) {
		ret = nr < 0 ? nr : -EFAULT;
		goto error;
	}

	attach->pages = pages;
	attach->npages = npages;

	ret = sg_alloc_table_from_pages(sgt, pages, npages,
					start & ~PAGE_MASK, attach->len,
					GFP_KERNEL);
	if (ret < 0)
		goto error;

	attach->sgt = sgt;

	return 0;

error_up_read:
	mmap_read_unlock(current->mm);
error:
	if (nr > 0)
		unpin_user_pages(pages, nr);
	kvfree(pages);
free_sgt:
	kfree(sgt);

	pr_err("failed to get userpages:%d\n", ret);

	return ret;
}

static void ipu_psys_put_userpages(struct ipu_dma_buf_attach *attach)
{
	if (!attach || !attach->userptr || !attach->sgt)
		return;

	unpin_user_pages(attach->pages, attach->npages);
	kvfree(attach->pages);

	sg_free_table(attach->sgt);
	kfree(attach->sgt);
	attach->sgt = NULL;
}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
static int ipu_dma_buf_attach(struct dma_buf *dbuf,
			      struct dma_buf_attachment *attach)
#else
static int ipu_dma_buf_attach(struct dma_buf *dbuf, struct device *dev,
			      struct dma_buf_attachment *attach)
#endif
{
	struct ipu_psys_kbuffer *kbuf = dbuf->priv;
	struct ipu_dma_buf_attach *ipu_attach;
	int ret;

	ipu_attach = kzalloc(sizeof(*ipu_attach), GFP_KERNEL);
	if (!ipu_attach)
		return -ENOMEM;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 19, 0)
	ipu_attach->dev = dev;
#endif
	ipu_attach->len = kbuf->len;
	ipu_attach->userptr = kbuf->userptr;

	ret = ipu_psys_get_userpages(ipu_attach);
	if (ret) {
		kfree(ipu_attach);
		return ret;
	}

	attach->priv = ipu_attach;
	return 0;
}

static void ipu_dma_buf_detach(struct dma_buf *dbuf,
			       struct dma_buf_attachment *attach)
{
	struct ipu_dma_buf_attach *ipu_attach = attach->priv;

	ipu_psys_put_userpages(ipu_attach);
	kfree(ipu_attach);
	attach->priv = NULL;
}

static struct sg_table *ipu_dma_buf_map(struct dma_buf_attachment *attach,
					enum dma_data_direction dir)
{
	struct ipu_dma_buf_attach *ipu_attach = attach->priv;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 8, 0)
	DEFINE_DMA_ATTRS(attrs);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 5)
	struct pci_dev *pdev = to_pci_dev(attach->dev);
	struct ipu6_device *isp = pci_get_drvdata(pdev);
	struct ipu6_bus_device *adev = isp->psys;
#endif
	unsigned long attrs;
	int ret;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 8, 0)
	dma_set_attr(DMA_ATTR_SKIP_CPU_SYNC, &attrs);
	ret = dma_map_sg_attrs(attach->dev, ipu_attach->sgt->sgl,
			       ipu_attach->sgt->orig_nents, dir, &attrs);
	if (!ret) {
		dev_dbg(attach->dev, "buf map failed\n");

		return ERR_PTR(-EIO);
	}
	ipu_attach->sgt->nents = ret;
#elif LINUX_VERSION_CODE < KERNEL_VERSION(5, 8, 0)
	attrs = DMA_ATTR_SKIP_CPU_SYNC;
	ret = dma_map_sg_attrs(attach->dev, ipu_attach->sgt->sgl,
			       ipu_attach->sgt->orig_nents, dir, attrs);
	if (!ret) {
		dev_dbg(attach->dev, "buf map failed\n");

		return ERR_PTR(-EIO);
	}
	ipu_attach->sgt->nents = ret;
#elif LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 5)
	attrs = DMA_ATTR_SKIP_CPU_SYNC;
	ret = dma_map_sgtable(attach->dev, ipu_attach->sgt, dir, attrs);
	if (ret < 0) {
		dev_dbg(attach->dev, "buf map failed\n");

		return ERR_PTR(-EIO);
	}
#else
	attrs = DMA_ATTR_SKIP_CPU_SYNC;
	ret = dma_map_sgtable(&pdev->dev, ipu_attach->sgt, dir, attrs);
	if (ret) {
		dev_err(attach->dev, "pci buf map failed\n");
		return ERR_PTR(-EIO);
	}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 5)
	/*
	 * Initial cache flush to avoid writing dirty pages for buffers which
	 * are later marked as IPU_BUFFER_FLAG_NO_FLUSH.
	 */
	dma_sync_sg_for_device(attach->dev, ipu_attach->sgt->sgl,
			       ipu_attach->sgt->orig_nents, DMA_BIDIRECTIONAL);
#else
	dma_sync_sgtable_for_device(&pdev->dev, ipu_attach->sgt, dir);

	ret = ipu6_dma_map_sgtable(adev, ipu_attach->sgt, dir, 0);
	if (ret) {
		dev_err(attach->dev, "ipu6 buf map failed\n");
		return ERR_PTR(-EIO);
	}

	ipu6_dma_sync_sgtable(adev, ipu_attach->sgt);
#endif

	return ipu_attach->sgt;
}

static void ipu_dma_buf_unmap(struct dma_buf_attachment *attach,
			      struct sg_table *sgt, enum dma_data_direction dir)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 5)
	struct pci_dev *pdev = to_pci_dev(attach->dev);
	struct ipu6_device *isp = pci_get_drvdata(pdev);
	struct ipu6_bus_device *adev = isp->psys;
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 8, 0)
	dma_unmap_sg(attach->dev, sgt->sgl, sgt->orig_nents, dir);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 5)
	ipu6_dma_unmap_sgtable(adev, sgt, dir, DMA_ATTR_SKIP_CPU_SYNC);
	dma_unmap_sgtable(&pdev->dev, sgt, dir, 0);
#else
	dma_unmap_sgtable(attach->dev, sgt, dir, DMA_ATTR_SKIP_CPU_SYNC);
#endif
}

static int ipu_dma_buf_mmap(struct dma_buf *dbuf, struct vm_area_struct *vma)
{
	return -ENOTTY;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 19, 0)
static void *ipu_dma_buf_kmap_atomic(struct dma_buf *dbuf, unsigned long pgnum)
{
	return NULL;
}
#endif

static void ipu_dma_buf_release(struct dma_buf *buf)
{
	struct ipu_psys_kbuffer *kbuf = buf->priv;

	if (!kbuf)
		return;

	if (kbuf->db_attach)
		ipu_psys_put_userpages(kbuf->db_attach->priv);

	kfree(kbuf);
}

static int ipu_dma_buf_begin_cpu_access(struct dma_buf *dma_buf,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 6, 0)
					size_t start, size_t len,
#endif
					enum dma_data_direction dir)
{
	return -ENOTTY;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0) || LINUX_VERSION_CODE == KERNEL_VERSION(5, 15, 255) \
	|| LINUX_VERSION_CODE == KERNEL_VERSION(5, 15, 71)
static int ipu_dma_buf_vmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
	struct dma_buf_attachment *attach;
	struct ipu_dma_buf_attach *ipu_attach;

	if (list_empty(&dmabuf->attachments))
		return -EINVAL;

	attach = list_last_entry(&dmabuf->attachments,
				 struct dma_buf_attachment, node);
	ipu_attach = attach->priv;

	if (!ipu_attach || !ipu_attach->pages || !ipu_attach->npages)
		return -EINVAL;

	map->vaddr = vm_map_ram(ipu_attach->pages, ipu_attach->npages, 0);
	map->is_iomem = false;
	if (!map->vaddr)
		return -EINVAL;

	return 0;
}
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0) && LINUX_VERSION_CODE != KERNEL_VERSION(5, 10, 46)
static int ipu_dma_buf_vmap(struct dma_buf *dmabuf, struct dma_buf_map *map)
{
	struct dma_buf_attachment *attach;
	struct ipu_dma_buf_attach *ipu_attach;

	if (list_empty(&dmabuf->attachments))
		return -EINVAL;

	attach = list_last_entry(&dmabuf->attachments,
				 struct dma_buf_attachment, node);
	ipu_attach = attach->priv;

	if (!ipu_attach || !ipu_attach->pages || !ipu_attach->npages)
		return -EINVAL;

	map->vaddr = vm_map_ram(ipu_attach->pages, ipu_attach->npages, 0);
	map->is_iomem = false;
	if (!map->vaddr)
		return -EINVAL;

	return 0;
}
#else
static void *ipu_dma_buf_vmap(struct dma_buf *dmabuf)
{
	struct dma_buf_attachment *attach;
	struct ipu_dma_buf_attach *ipu_attach;

	if (list_empty(&dmabuf->attachments))
		return NULL;

	attach = list_last_entry(&dmabuf->attachments,
				 struct dma_buf_attachment, node);
	ipu_attach = attach->priv;

	if (!ipu_attach || !ipu_attach->pages || !ipu_attach->npages)
		return NULL;

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 9, 0)
	return vm_map_ram(ipu_attach->pages,
			  ipu_attach->npages, 0, PAGE_KERNEL);
#else
	return vm_map_ram(ipu_attach->pages, ipu_attach->npages, 0);
#endif
}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0) || LINUX_VERSION_CODE == KERNEL_VERSION(5, 15, 255) \
	|| LINUX_VERSION_CODE == KERNEL_VERSION(5, 15, 71)
static void ipu_dma_buf_vunmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
	struct dma_buf_attachment *attach;
	struct ipu_dma_buf_attach *ipu_attach;

	if (WARN_ON(list_empty(&dmabuf->attachments)))
		return;

	attach = list_last_entry(&dmabuf->attachments,
				 struct dma_buf_attachment, node);
	ipu_attach = attach->priv;

	if (WARN_ON(!ipu_attach || !ipu_attach->pages || !ipu_attach->npages))
		return;

	vm_unmap_ram(map->vaddr, ipu_attach->npages);
}
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0) && LINUX_VERSION_CODE != KERNEL_VERSION(5, 10, 46)
static void ipu_dma_buf_vunmap(struct dma_buf *dmabuf, struct dma_buf_map *map)
{
	struct dma_buf_attachment *attach;
	struct ipu_dma_buf_attach *ipu_attach;

	if (WARN_ON(list_empty(&dmabuf->attachments)))
		return;

	attach = list_last_entry(&dmabuf->attachments,
				 struct dma_buf_attachment, node);
	ipu_attach = attach->priv;

	if (WARN_ON(!ipu_attach || !ipu_attach->pages || !ipu_attach->npages))
		return;

	vm_unmap_ram(map->vaddr, ipu_attach->npages);
}
#else
static void ipu_dma_buf_vunmap(struct dma_buf *dmabuf, void *vaddr)
{
	struct dma_buf_attachment *attach;
	struct ipu_dma_buf_attach *ipu_attach;

	if (WARN_ON(list_empty(&dmabuf->attachments)))
		return;

	attach = list_last_entry(&dmabuf->attachments,
				 struct dma_buf_attachment, node);
	ipu_attach = attach->priv;

	if (WARN_ON(!ipu_attach || !ipu_attach->pages || !ipu_attach->npages))
		return;

	vm_unmap_ram(vaddr, ipu_attach->npages);
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
struct dma_buf_ops ipu_dma_buf_ops = {
#else
static const struct dma_buf_ops ipu_dma_buf_ops = {
#endif
	.attach = ipu_dma_buf_attach,
	.detach = ipu_dma_buf_detach,
	.map_dma_buf = ipu_dma_buf_map,
	.unmap_dma_buf = ipu_dma_buf_unmap,
	.release = ipu_dma_buf_release,
	.begin_cpu_access = ipu_dma_buf_begin_cpu_access,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0)
	.kmap = ipu_dma_buf_kmap,
	.kmap_atomic = ipu_dma_buf_kmap_atomic,
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 19, 0)
	.map_atomic = ipu_dma_buf_kmap_atomic,
#endif
	.mmap = ipu_dma_buf_mmap,
	.vmap = ipu_dma_buf_vmap,
	.vunmap = ipu_dma_buf_vunmap,
};

static int ipu_psys_open(struct inode *inode, struct file *file)
{
	struct ipu_psys *psys = inode_to_ipu_psys(inode);
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
	struct ipu_device *isp = psys->adev->isp;
#endif
	struct ipu_psys_fh *fh;
	int rval;

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
	if (isp->flr_done)
		return -EIO;

#endif
	fh = kzalloc(sizeof(*fh), GFP_KERNEL);
	if (!fh)
		return -ENOMEM;

	fh->psys = psys;

	file->private_data = fh;

	mutex_init(&fh->mutex);
	INIT_LIST_HEAD(&fh->bufs_list);
	INIT_LIST_HEAD(&fh->descs_list);
	INIT_LIST_HEAD(&fh->bufs_lru);
	init_waitqueue_head(&fh->wait);

	rval = ipu_psys_fh_init(fh);
	if (rval)
		goto open_failed;

	mutex_lock(&psys->mutex);
	list_add_tail(&fh->list, &psys->fhs);
	mutex_unlock(&psys->mutex);

	return 0;

open_failed:
	mutex_destroy(&fh->mutex);
	kfree(fh);
	return rval;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 5)
static inline void ipu_psys_kbuf_unmap(struct ipu_psys_kbuffer *kbuf)
#else
static inline void ipu_psys_kbuf_unmap(struct ipu_psys_fh *fh,
				       struct ipu_psys_kbuffer *kbuf)
#endif
{
	if (!kbuf)
		return;

	kbuf->valid = false;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0) || LINUX_VERSION_CODE == KERNEL_VERSION(5, 15, 255) \
	|| LINUX_VERSION_CODE == KERNEL_VERSION(5, 15, 71)
	if (kbuf->kaddr) {
		struct iosys_map dmap;

		iosys_map_set_vaddr(&dmap, kbuf->kaddr);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 255)
		dma_buf_vunmap_unlocked(kbuf->dbuf, &dmap);
#else
		dma_buf_vunmap(kbuf->dbuf, &dmap);
#endif
	}

#elif LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0) && LINUX_VERSION_CODE != KERNEL_VERSION(5, 10, 46)
	if (kbuf->kaddr) {
		struct dma_buf_map dmap;

		dma_buf_map_set_vaddr(&dmap, kbuf->kaddr);
		dma_buf_vunmap(kbuf->dbuf, &dmap);
	}
#else
	if (kbuf->kaddr)
		dma_buf_vunmap(kbuf->dbuf, kbuf->kaddr);
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 255) && LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 5)
	if (!IS_ERR_OR_NULL(kbuf->sgt))
		dma_buf_unmap_attachment_unlocked(kbuf->db_attach,
						  kbuf->sgt,
						  DMA_BIDIRECTIONAL);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 5)
	if (!kbuf->userptr)
		ipu6_dma_unmap_sgtable(fh->psys->adev, kbuf->sgt,
				       DMA_BIDIRECTIONAL, 0);

	if (!IS_ERR_OR_NULL(kbuf->sgt))
		dma_buf_unmap_attachment_unlocked(kbuf->db_attach,
						  kbuf->sgt,
						  DMA_BIDIRECTIONAL);
#else
	if (kbuf->sgt)
		dma_buf_unmap_attachment(kbuf->db_attach,
					 kbuf->sgt,
					 DMA_BIDIRECTIONAL);
#endif
	if (!IS_ERR_OR_NULL(kbuf->db_attach))
		dma_buf_detach(kbuf->dbuf, kbuf->db_attach);
	dma_buf_put(kbuf->dbuf);

	kbuf->db_attach = NULL;
	kbuf->dbuf = NULL;
	kbuf->sgt = NULL;
}

static void __ipu_psys_unmapbuf(struct ipu_psys_fh *fh,
				struct ipu_psys_kbuffer *kbuf)
{
	/* From now on it is not safe to use this kbuffer */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 5)
	ipu_psys_kbuf_unmap(kbuf);
#else
	ipu_psys_kbuf_unmap(fh, kbuf);
#endif
	ipu_buffer_del(fh, kbuf);
	if (!kbuf->userptr)
		kfree(kbuf);
}

static int ipu_psys_unmapbuf_locked(int fd, struct ipu_psys_fh *fh)
{
	struct ipu_psys *psys = fh->psys;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 10, 0)
	struct device *dev = &psys->adev->auxdev.dev;
#endif
	struct ipu_psys_kbuffer *kbuf;
	struct ipu_psys_desc *desc;

	desc = psys_desc_lookup(fh, fd);
	if (WARN_ON_ONCE(!desc)) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
		dev_err(&psys->adev->dev, "descriptor not found: %d\n", fd);
#else
		dev_err(dev, "descriptor not found: %d\n", fd);
#endif
		return -EINVAL;
	}

	kbuf = desc->kbuf;
	/* descriptor is gone now */
	ipu_desc_del(fh, desc);
	kfree(desc);

	if (WARN_ON_ONCE(!kbuf || !kbuf->dbuf)) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
		dev_err(&psys->adev->dev,
#else
		dev_err(dev,
#endif
			"descriptor with no buffer: %d\n", fd);
		return -EINVAL;
	}

	/* Wait for final UNMAP */
	if (!atomic_dec_and_test(&kbuf->map_count))
		return 0;

	__ipu_psys_unmapbuf(fh, kbuf);

	return 0;
}

static int ipu_psys_release(struct inode *inode, struct file *file)
{
	struct ipu_psys *psys = inode_to_ipu_psys(inode);
	struct ipu_psys_fh *fh = file->private_data;

	mutex_lock(&fh->mutex);
	while (!list_empty(&fh->descs_list)) {
		struct ipu_psys_desc *desc;

		desc = list_first_entry(&fh->descs_list,
					struct ipu_psys_desc,
					list);

		ipu_desc_del(fh, desc);
		kfree(desc);
	}

	while (!list_empty(&fh->bufs_lru)) {
		struct ipu_psys_kbuffer *kbuf;

		kbuf = list_first_entry(&fh->bufs_lru,
					struct ipu_psys_kbuffer,
					list);

		ipu_buffer_lru_del(fh, kbuf);
		__ipu_psys_unmapbuf(fh, kbuf);
	}

	while (!list_empty(&fh->bufs_list)) {
		struct dma_buf_attachment *db_attach;
		struct ipu_psys_kbuffer *kbuf;

		kbuf = list_first_entry(&fh->bufs_list,
					struct ipu_psys_kbuffer,
					list);

		ipu_buffer_del(fh, kbuf);
		db_attach = kbuf->db_attach;

		/* Unmap and release buffers */
		if (kbuf->dbuf && db_attach) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 5)
			ipu_psys_kbuf_unmap(kbuf);
#else
			ipu_psys_kbuf_unmap(fh, kbuf);
#endif
		} else {
			if (db_attach)
				ipu_psys_put_userpages(db_attach->priv);
			kfree(kbuf);
		}
	}
	mutex_unlock(&fh->mutex);

	mutex_lock(&psys->mutex);
	list_del(&fh->list);

	mutex_unlock(&psys->mutex);
	ipu_psys_fh_deinit(fh);

	mutex_lock(&psys->mutex);
	if (list_empty(&psys->fhs))
		psys->power_gating = 0;
	mutex_unlock(&psys->mutex);
	mutex_destroy(&fh->mutex);
	kfree(fh);

	return 0;
}

static int ipu_psys_getbuf(struct ipu_psys_buffer *buf, struct ipu_psys_fh *fh)
{
	struct ipu_psys_kbuffer *kbuf;
	struct ipu_psys *psys = fh->psys;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 10, 0)
	struct device *dev = &psys->adev->auxdev.dev;
#endif
	struct ipu_psys_desc *desc;
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	struct dma_buf *dbuf;
	int ret;

	if (!buf->base.userptr) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
		dev_err(&psys->adev->dev, "Buffer allocation not supported\n");
#else
		dev_err(dev, "Buffer allocation not supported\n");
#endif
		return -EINVAL;
	}

	kbuf = ipu_psys_kbuffer_alloc();
	if (!kbuf)
		return -ENOMEM;

	kbuf->len = buf->len;
	kbuf->userptr = (unsigned long)buf->base.userptr;
	kbuf->flags = buf->flags;

	exp_info.ops = &ipu_dma_buf_ops;
	exp_info.size = kbuf->len;
	exp_info.flags = O_RDWR;
	exp_info.priv = kbuf;

	dbuf = dma_buf_export(&exp_info);
	if (IS_ERR(dbuf)) {
		kfree(kbuf);
		return PTR_ERR(dbuf);
	}

	ret = dma_buf_fd(dbuf, 0);
	if (ret < 0) {
		dma_buf_put(dbuf);
		return ret;
	}

	buf->base.fd = ret;
	buf->flags &= ~IPU_BUFFER_FLAG_USERPTR;
	buf->flags |= IPU_BUFFER_FLAG_DMA_HANDLE;
	kbuf->flags = buf->flags;

	desc = ipu_psys_desc_alloc(ret);
	if (!desc) {
		dma_buf_put(dbuf);
		return -ENOMEM;
	}

	kbuf->dbuf = dbuf;

	mutex_lock(&fh->mutex);
	ipu_desc_add(fh, desc);
	ipu_buffer_add(fh, kbuf);
	mutex_unlock(&fh->mutex);

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
	dev_dbg(&psys->adev->dev, "IOC_GETBUF: userptr %p size %llu to fd %d",
#else
	dev_dbg(dev, "IOC_GETBUF: userptr %p size %llu to fd %d",
#endif
		buf->base.userptr, buf->len, buf->base.fd);

	return 0;
}

static int ipu_psys_putbuf(struct ipu_psys_buffer *buf, struct ipu_psys_fh *fh)
{
	return 0;
}

static void ipu_psys_kbuffer_lru(struct ipu_psys_fh *fh,
				 struct ipu_psys_kbuffer *kbuf)
{
	ipu_buffer_del(fh, kbuf);
	ipu_buffer_lru_add(fh, kbuf);

	while (fh->num_bufs_lru > IPU_PSYS_MAX_NUM_BUFS_LRU) {
		kbuf = list_first_entry(&fh->bufs_lru,
					struct ipu_psys_kbuffer,
					list);

		ipu_buffer_lru_del(fh, kbuf);
		__ipu_psys_unmapbuf(fh, kbuf);
	}
}

struct ipu_psys_kbuffer *ipu_psys_mapbuf_locked(int fd, struct ipu_psys_fh *fh)
{
	struct ipu_psys *psys = fh->psys;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 5)
	struct device *dev = &psys->adev->isp->pdev->dev;
	int ret;
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(6, 10, 0)
	struct device *dev = &psys->adev->auxdev.dev;
#endif
	struct ipu_psys_kbuffer *kbuf;
	struct ipu_psys_desc *desc;
	struct dma_buf *dbuf;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0) || \
	LINUX_VERSION_CODE == KERNEL_VERSION(5, 15, 255) || \
	LINUX_VERSION_CODE == KERNEL_VERSION(5, 15, 71)
	struct iosys_map dmap;
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0) && LINUX_VERSION_CODE != KERNEL_VERSION(5, 10, 46)
	struct dma_buf_map dmap;
#endif

	dbuf = dma_buf_get(fd);
	if (IS_ERR(dbuf))
		return NULL;

	desc = psys_desc_lookup(fh, fd);
	if (!desc) {
		desc = ipu_psys_desc_alloc(fd);
		if (!desc)
			goto desc_alloc_fail;
		ipu_desc_add(fh, desc);
	}

	kbuf = psys_buf_lookup(fh, fd);
	if (!kbuf) {
		kbuf = ipu_psys_kbuffer_alloc();
		if (!kbuf)
			goto buf_alloc_fail;
		ipu_buffer_add(fh, kbuf);
	}

	/* If this descriptor references no buffer or new buffer */
	if (desc->kbuf != kbuf) {
		if (desc->kbuf) {
			/*
			 * Un-reference old buffer and possibly put it on
			 * the LRU list
			 */
			if (atomic_dec_and_test(&desc->kbuf->map_count))
				ipu_psys_kbuffer_lru(fh, desc->kbuf);
		}

		/* Grab reference of the new buffer */
		atomic_inc(&kbuf->map_count);
	}

	desc->kbuf = kbuf;

	if (kbuf->sgt) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
		dev_dbg(&psys->adev->dev, "fd %d has been mapped!\n", fd);
#else
		dev_dbg(dev, "fd %d has been mapped!\n", fd);
#endif
		dma_buf_put(dbuf);
		goto mapbuf_end;
	}

	kbuf->dbuf = dbuf;

	if (kbuf->len == 0)
		kbuf->len = kbuf->dbuf->size;

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
	kbuf->db_attach = dma_buf_attach(kbuf->dbuf, &psys->adev->dev);
	if (IS_ERR(kbuf->db_attach)) {
		dev_dbg(&psys->adev->dev, "dma buf attach failed\n");
		goto kbuf_map_fail;
	}
#elif LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 5)
	kbuf->db_attach = dma_buf_attach(kbuf->dbuf, dev);
	if (IS_ERR(kbuf->db_attach)) {
		dev_dbg(dev, "dma buf attach failed\n");
		goto kbuf_map_fail;
	}
#else
	kbuf->db_attach = dma_buf_attach(kbuf->dbuf, dev);
	if (IS_ERR(kbuf->db_attach)) {
		dev_dbg(dev, "dma buf attach failed\n");
		goto attach_fail;
	}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 255)
	kbuf->sgt = dma_buf_map_attachment_unlocked(kbuf->db_attach,
						    DMA_BIDIRECTIONAL);
#else
	kbuf->sgt = dma_buf_map_attachment(kbuf->db_attach, DMA_BIDIRECTIONAL);
#endif
	if (IS_ERR_OR_NULL(kbuf->sgt)) {
		kbuf->sgt = NULL;
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
		dev_dbg(&psys->adev->dev, "dma buf map attachment failed\n");
#else
		dev_dbg(dev, "dma buf map attachment failed\n");
#endif
		goto kbuf_map_fail;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 5)
	if (!kbuf->userptr) {
		ret = ipu6_dma_map_sgtable(psys->adev, kbuf->sgt,
					   DMA_BIDIRECTIONAL, 0);
		if (ret) {
			dev_dbg(dev, "ipu6 buf map failed\n");
			goto kbuf_map_fail;
		}
	}
#endif

	kbuf->dma_addr = sg_dma_address(kbuf->sgt->sgl);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0) && LINUX_VERSION_CODE != KERNEL_VERSION(5, 10, 46)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 255)
	dmap.is_iomem = false;
	if (dma_buf_vmap_unlocked(kbuf->dbuf, &dmap)) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
		dev_dbg(&psys->adev->dev, "dma buf vmap failed\n");
#else
		dev_dbg(dev, "dma buf vmap failed\n");
#endif
		goto kbuf_map_fail;
	}
#else
	if (dma_buf_vmap(kbuf->dbuf, &dmap)) {
		dev_dbg(&psys->adev->dev, "dma buf vmap failed\n");
		goto kbuf_map_fail;
	}
#endif
	kbuf->kaddr = dmap.vaddr;
#else
	kbuf->kaddr = dma_buf_vmap(kbuf->dbuf);
	if (!kbuf->kaddr) {
		dev_dbg(&psys->adev->dev, "dma buf vmap failed\n");
		goto kbuf_map_fail;
	}
#endif

mapbuf_end:
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
	dev_dbg(&psys->adev->dev, "%s kbuf %p fd %d with len %llu mapped\n",
		__func__, kbuf, fd, kbuf->len);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 5)
	dev_dbg(dev, "%s kbuf %p fd %d with len %llu mapped\n",
		__func__, kbuf, fd, kbuf->len);
#else
	dev_dbg(dev, "%s %s kbuf %p fd %d with len %llu mapped\n",
		__func__, kbuf->userptr ? "private" : "imported", kbuf, fd,
		kbuf->len);
#endif

	kbuf->valid = true;
	return kbuf;

kbuf_map_fail:
	ipu_buffer_del(fh, kbuf);
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 5)
	ipu_psys_kbuf_unmap(kbuf);
#else
	if (!IS_ERR_OR_NULL(kbuf->sgt)) {
		if (!kbuf->userptr)
			ipu6_dma_unmap_sgtable(psys->adev, kbuf->sgt,
					       DMA_BIDIRECTIONAL, 0);
		dma_buf_unmap_attachment_unlocked(kbuf->db_attach, kbuf->sgt,
						  DMA_BIDIRECTIONAL);
	}
	dma_buf_detach(kbuf->dbuf, kbuf->db_attach);
attach_fail:
	ipu_buffer_del(fh, kbuf);
#endif
	dbuf = ERR_PTR(-EINVAL);
	if (!kbuf->userptr)
		kfree(kbuf);

buf_alloc_fail:
	ipu_desc_del(fh, desc);
	kfree(desc);

desc_alloc_fail:
	if (!IS_ERR(dbuf))
		dma_buf_put(dbuf);
	return NULL;
}

static long ipu_psys_mapbuf(int fd, struct ipu_psys_fh *fh)
{
	struct ipu_psys_kbuffer *kbuf;

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
	dev_dbg(&fh->psys->adev->dev, "IOC_MAPBUF\n");
#else
	dev_dbg(&fh->psys->adev->auxdev.dev, "IOC_MAPBUF\n");
#endif

	mutex_lock(&fh->mutex);
	kbuf = ipu_psys_mapbuf_locked(fd, fh);
	mutex_unlock(&fh->mutex);

	return kbuf ? 0 : -EINVAL;
}

static long ipu_psys_unmapbuf(int fd, struct ipu_psys_fh *fh)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
	struct device *dev = &fh->psys->adev->dev;
#else
	struct device *dev = &fh->psys->adev->auxdev.dev;
#endif
	long ret;

	dev_dbg(dev, "IOC_UNMAPBUF\n");

	mutex_lock(&fh->mutex);
	ret = ipu_psys_unmapbuf_locked(fd, fh);
	mutex_unlock(&fh->mutex);

	return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
static unsigned int ipu_psys_poll(struct file *file,
				  struct poll_table_struct *wait)
{
	struct ipu_psys_fh *fh = file->private_data;
	struct ipu_psys *psys = fh->psys;
	unsigned int res = 0;

	dev_dbg(&psys->adev->dev, "ipu psys poll\n");

	poll_wait(file, &fh->wait, wait);

	if (ipu_get_completed_kcmd(fh))
		res = POLLIN;

	dev_dbg(&psys->adev->dev, "ipu psys poll res %u\n", res);

	return res;
}
#else
static __poll_t ipu_psys_poll(struct file *file,
			      struct poll_table_struct *wait)
{
	struct ipu_psys_fh *fh = file->private_data;
	struct ipu_psys *psys = fh->psys;
	struct device *dev = &psys->adev->auxdev.dev;
	__poll_t ret = 0;

	dev_dbg(dev, "ipu psys poll\n");

	poll_wait(file, &fh->wait, wait);

	if (ipu_get_completed_kcmd(fh))
		ret = POLLIN;

	dev_dbg(dev, "ipu psys poll ret %u\n", ret);

	return ret;
}
#endif

static long ipu_get_manifest(struct ipu_psys_manifest *manifest,
			     struct ipu_psys_fh *fh)
{
	struct ipu_psys *psys = fh->psys;
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
	struct ipu_device *isp = psys->adev->isp;
	struct ipu_cpd_client_pkg_hdr *client_pkg;
#else
	struct device *dev = &psys->adev->auxdev.dev;
	struct ipu6_bus_device *adev = psys->adev;
	struct ipu6_device *isp = adev->isp;
	struct ipu6_cpd_client_pkg_hdr *client_pkg;
#endif
	u32 entries;
	void *host_fw_data;
	dma_addr_t dma_fw_data;
	u32 client_pkg_offset;

	host_fw_data = (void *)isp->cpd_fw->data;
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
	dma_fw_data = sg_dma_address(psys->fw_sgt.sgl);
#else
	dma_fw_data = sg_dma_address(adev->fw_sgt.sgl);
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
	entries = ipu_cpd_pkg_dir_get_num_entries(psys->pkg_dir);
	if (!manifest || manifest->index > entries - 1) {
		dev_err(&psys->adev->dev, "invalid argument\n");
		return -EINVAL;
	}

	if (!ipu_cpd_pkg_dir_get_size(psys->pkg_dir, manifest->index) ||
	    ipu_cpd_pkg_dir_get_type(psys->pkg_dir, manifest->index) <
	    IPU_CPD_PKG_DIR_CLIENT_PG_TYPE) {
		dev_dbg(&psys->adev->dev, "invalid pkg dir entry\n");
		return -ENOENT;
	}

	client_pkg_offset = ipu_cpd_pkg_dir_get_address(psys->pkg_dir,
							manifest->index);
#else
	entries = ipu6_cpd_pkg_dir_get_num_entries(adev->pkg_dir);
	if (!manifest || manifest->index > entries - 1) {
		dev_err(dev, "invalid argument\n");
		return -EINVAL;
	}

	if (!ipu6_cpd_pkg_dir_get_size(adev->pkg_dir, manifest->index) ||
	    ipu6_cpd_pkg_dir_get_type(adev->pkg_dir, manifest->index) <
	    IPU6_CPD_PKG_DIR_CLIENT_PG_TYPE) {
		dev_dbg(dev, "invalid pkg dir entry\n");
		return -ENOENT;
	}

	client_pkg_offset = ipu6_cpd_pkg_dir_get_address(adev->pkg_dir,
							 manifest->index);
#endif
	client_pkg_offset -= dma_fw_data;

	client_pkg = host_fw_data + client_pkg_offset;
	manifest->size = client_pkg->pg_manifest_size;

	if (!manifest->manifest)
		return 0;

	if (copy_to_user(manifest->manifest,
			 (uint8_t *)client_pkg + client_pkg->pg_manifest_offs,
			 manifest->size)) {
		return -EFAULT;
	}

	return 0;
}

static long ipu_psys_ioctl(struct file *file, unsigned int cmd,
			   unsigned long arg)
{
	union {
		struct ipu_psys_buffer buf;
		struct ipu_psys_command cmd;
		struct ipu_psys_event ev;
		struct ipu_psys_capability caps;
		struct ipu_psys_manifest m;
	} karg;
	struct ipu_psys_fh *fh = file->private_data;
	long err = 0;
	void __user *up = (void __user *)arg;
	bool copy = (cmd != IPU_IOC_MAPBUF && cmd != IPU_IOC_UNMAPBUF);

	if (copy) {
		if (_IOC_SIZE(cmd) > sizeof(karg))
			return -ENOTTY;

		if (_IOC_DIR(cmd) & _IOC_WRITE) {
			err = copy_from_user(&karg, up, _IOC_SIZE(cmd));
			if (err)
				return -EFAULT;
		}
	}

	switch (cmd) {
	case IPU_IOC_MAPBUF:
		err = ipu_psys_mapbuf(arg, fh);
		break;
	case IPU_IOC_UNMAPBUF:
		err = ipu_psys_unmapbuf(arg, fh);
		break;
	case IPU_IOC_QUERYCAP:
		karg.caps = fh->psys->caps;
		break;
	case IPU_IOC_GETBUF:
		err = ipu_psys_getbuf(&karg.buf, fh);
		break;
	case IPU_IOC_PUTBUF:
		err = ipu_psys_putbuf(&karg.buf, fh);
		break;
	case IPU_IOC_QCMD:
		err = ipu_psys_kcmd_new(&karg.cmd, fh);
		break;
	case IPU_IOC_DQEVENT:
		err = ipu_ioctl_dqevent(&karg.ev, fh, file->f_flags);
		break;
	case IPU_IOC_GET_MANIFEST:
		err = ipu_get_manifest(&karg.m, fh);
		break;
	default:
		err = -ENOTTY;
		break;
	}

	if (err)
		return err;

	if (copy && _IOC_DIR(cmd) & _IOC_READ)
		if (copy_to_user(up, &karg, _IOC_SIZE(cmd)))
			return -EFAULT;

	return 0;
}

static const struct file_operations ipu_psys_fops = {
	.open = ipu_psys_open,
	.release = ipu_psys_release,
	.unlocked_ioctl = ipu_psys_ioctl,
	.poll = ipu_psys_poll,
	.owner = THIS_MODULE,
};

static void ipu_psys_dev_release(struct device *dev)
{
}

static int psys_runtime_pm_resume(struct device *dev)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
	struct ipu_bus_device *adev = to_ipu_bus_device(dev);
	struct ipu_psys *psys = ipu_bus_get_drvdata(adev);
#else
	struct ipu6_bus_device *adev = to_ipu6_bus_device(dev);
	struct ipu_psys *psys = ipu6_bus_get_drvdata(adev);
#endif
	unsigned long flags;
	int retval;

	if (!psys)
		return 0;

	spin_lock_irqsave(&psys->ready_lock, flags);
	if (psys->ready) {
		spin_unlock_irqrestore(&psys->ready_lock, flags);
		return 0;
	}
	spin_unlock_irqrestore(&psys->ready_lock, flags);

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
	retval = ipu_mmu_hw_init(adev->mmu);
#else
	retval = ipu6_mmu_hw_init(adev->mmu);
#endif
	if (retval)
		return retval;

	if (async_fw_init && !psys->fwcom) {
		dev_err(dev, "async firmware init not finished, skipping\n");
		return 0;
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
	if (!ipu_buttress_auth_done(adev->isp)) {
#else
	if (!ipu6_buttress_auth_done(adev->isp)) {
#endif
		dev_dbg(dev, "fw not yet authenticated, skipping\n");
		return 0;
	}

	ipu_psys_setup_hw(psys);

	ipu_psys_subdomains_power(psys, 1);
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
	ipu_trace_restore(&psys->adev->dev);

#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
	ipu_configure_spc(adev->isp,
			  &psys->pdata->ipdata->hw_variant,
			  IPU_CPD_PKG_DIR_PSYS_SERVER_IDX,
			  psys->pdata->base, psys->pkg_dir,
			  psys->pkg_dir_dma_addr);
#else
	ipu6_configure_spc(adev->isp,
			   &psys->pdata->ipdata->hw_variant,
			   IPU6_CPD_PKG_DIR_PSYS_SERVER_IDX,
			   psys->pdata->base, adev->pkg_dir,
			   adev->pkg_dir_dma_addr);
#endif

	retval = ipu_fw_psys_open(psys);
	if (retval) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
		dev_err(&psys->adev->dev, "Failed to open abi.\n");
#else
		dev_err(dev, "Failed to open abi.\n");
#endif
		return retval;
	}

	spin_lock_irqsave(&psys->ready_lock, flags);
	psys->ready = 1;
	spin_unlock_irqrestore(&psys->ready_lock, flags);

	return 0;
}

static int psys_runtime_pm_suspend(struct device *dev)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
	struct ipu_bus_device *adev = to_ipu_bus_device(dev);
	struct ipu_psys *psys = ipu_bus_get_drvdata(adev);
#else
	struct ipu6_bus_device *adev = to_ipu6_bus_device(dev);
	struct ipu_psys *psys = ipu6_bus_get_drvdata(adev);
#endif
	unsigned long flags;
	int rval;

	if (!psys)
		return 0;

	spin_lock_irqsave(&psys->ready_lock, flags);
	if (!psys->ready) {
		spin_unlock_irqrestore(&psys->ready_lock, flags);
		return 0;
	}
	psys->ready = 0;
	spin_unlock_irqrestore(&psys->ready_lock, flags);

	/*
	 * We can trace failure but better to not return an error.
	 * At suspend we are progressing towards psys power gated state.
	 * Any hang / failure inside psys will be forgotten soon.
	 */
	rval = ipu_fw_psys_close(psys);
	if (rval)
		dev_err(dev, "Device close failure: %d\n", rval);

	ipu_psys_subdomains_power(psys, 0);

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
	ipu_mmu_hw_cleanup(adev->mmu);
#else
	ipu6_mmu_hw_cleanup(adev->mmu);
#endif

	return 0;
}

/* The following PM callbacks are needed to enable runtime PM in IPU PCI
 * device resume, otherwise, runtime PM can't work in PCI resume from
 * S3 state.
 */
static int psys_resume(struct device *dev)
{
	return 0;
}

static int psys_suspend(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops psys_pm_ops = {
	.runtime_suspend = psys_runtime_pm_suspend,
	.runtime_resume = psys_runtime_pm_resume,
	.suspend = psys_suspend,
	.resume = psys_resume,
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
static int cpd_fw_reload(struct ipu_device *isp)
{
	struct ipu_psys *psys = ipu_bus_get_drvdata(isp->psys);
	int rval;

	if (!isp->secure_mode) {
		dev_warn(&isp->pdev->dev,
			 "CPD firmware reload was only supported for secure mode.\n");
		return -EINVAL;
	}

	if (isp->cpd_fw) {
		ipu_cpd_free_pkg_dir(isp->psys, psys->pkg_dir,
				     psys->pkg_dir_dma_addr,
				     psys->pkg_dir_size);

		ipu_buttress_unmap_fw_image(isp->psys, &psys->fw_sgt);
		release_firmware(isp->cpd_fw);
		isp->cpd_fw = NULL;
		dev_info(&isp->pdev->dev, "Old FW removed\n");
	}

	rval = request_cpd_fw(&isp->cpd_fw, isp->cpd_fw_name,
			      &isp->pdev->dev);
	if (rval) {
		dev_err(&isp->pdev->dev, "Requesting firmware(%s) failed\n",
			isp->cpd_fw_name);
		return rval;
	}

	rval = ipu_cpd_validate_cpd_file(isp, isp->cpd_fw->data,
					 isp->cpd_fw->size);
	if (rval) {
		dev_err(&isp->pdev->dev, "Failed to validate cpd file\n");
		goto out_release_firmware;
	}

	rval = ipu_buttress_map_fw_image(isp->psys, isp->cpd_fw, &psys->fw_sgt);
	if (rval)
		goto out_release_firmware;

	psys->pkg_dir = ipu_cpd_create_pkg_dir(isp->psys,
					       isp->cpd_fw->data,
					       sg_dma_address(psys->fw_sgt.sgl),
					       &psys->pkg_dir_dma_addr,
					       &psys->pkg_dir_size);

	if (!psys->pkg_dir) {
		rval = -EINVAL;
		goto out_unmap_fw_image;
	}

	isp->pkg_dir = psys->pkg_dir;
	isp->pkg_dir_dma_addr = psys->pkg_dir_dma_addr;
	isp->pkg_dir_size = psys->pkg_dir_size;

	if (!isp->secure_mode)
		return 0;

	rval = ipu_fw_authenticate(isp, 1);
	if (rval)
		goto out_free_pkg_dir;

	return 0;

out_free_pkg_dir:
	ipu_cpd_free_pkg_dir(isp->psys, psys->pkg_dir,
			     psys->pkg_dir_dma_addr, psys->pkg_dir_size);
out_unmap_fw_image:
	ipu_buttress_unmap_fw_image(isp->psys, &psys->fw_sgt);
out_release_firmware:
	release_firmware(isp->cpd_fw);
	isp->cpd_fw = NULL;

	return rval;
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
#ifdef CONFIG_DEBUG_FS
static int ipu_psys_icache_prefetch_sp_get(void *data, u64 *val)
{
	struct ipu_psys *psys = data;

	*val = psys->icache_prefetch_sp;
	return 0;
}

static int ipu_psys_icache_prefetch_sp_set(void *data, u64 val)
{
	struct ipu_psys *psys = data;

	if (val != !!val)
		return -EINVAL;

	psys->icache_prefetch_sp = val;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(psys_icache_prefetch_sp_fops,
			ipu_psys_icache_prefetch_sp_get,
			ipu_psys_icache_prefetch_sp_set, "%llu\n");

static int ipu_psys_icache_prefetch_isp_get(void *data, u64 *val)
{
	struct ipu_psys *psys = data;

	*val = psys->icache_prefetch_isp;
	return 0;
}

static int ipu_psys_icache_prefetch_isp_set(void *data, u64 val)
{
	struct ipu_psys *psys = data;

	if (val != !!val)
		return -EINVAL;

	psys->icache_prefetch_isp = val;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(psys_icache_prefetch_isp_fops,
			ipu_psys_icache_prefetch_isp_get,
			ipu_psys_icache_prefetch_isp_set, "%llu\n");

static int ipu_psys_init_debugfs(struct ipu_psys *psys)
{
	struct dentry *file;
	struct dentry *dir;

	dir = debugfs_create_dir("psys", psys->adev->isp->ipu_dir);
	if (IS_ERR(dir))
		return -ENOMEM;

	file = debugfs_create_file("icache_prefetch_sp", 0600,
				   dir, psys, &psys_icache_prefetch_sp_fops);
	if (IS_ERR(file))
		goto err;

	file = debugfs_create_file("icache_prefetch_isp", 0600,
				   dir, psys, &psys_icache_prefetch_isp_fops);
	if (IS_ERR(file))
		goto err;

	psys->debugfsdir = dir;

	return 0;
err:
	debugfs_remove_recursive(dir);
	return -ENOMEM;
}
#endif
#endif

static int ipu_psys_sched_cmd(void *ptr)
{
	struct ipu_psys *psys = ptr;
	size_t pending = 0;

	while (1) {
		wait_event_interruptible(psys->sched_cmd_wq,
					 (kthread_should_stop() ||
					  (pending =
					   atomic_read(&psys->wakeup_count))));

		if (kthread_should_stop())
			break;

		if (pending == 0)
			continue;

		mutex_lock(&psys->mutex);
		atomic_set(&psys->wakeup_count, 0);
		ipu_psys_run_next(psys);
		mutex_unlock(&psys->mutex);
	}

	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
static void start_sp(struct ipu_bus_device *adev)
{
	struct ipu_psys *psys = ipu_bus_get_drvdata(adev);
	void __iomem *spc_regs_base = psys->pdata->base +
	    psys->pdata->ipdata->hw_variant.spc_offset;
	u32 val = 0;

	val |= IPU_PSYS_SPC_STATUS_START |
	    IPU_PSYS_SPC_STATUS_RUN |
	    IPU_PSYS_SPC_STATUS_CTRL_ICACHE_INVALIDATE;
	val |= psys->icache_prefetch_sp ?
	    IPU_PSYS_SPC_STATUS_ICACHE_PREFETCH : 0;
	writel(val, spc_regs_base + IPU_PSYS_REG_SPC_STATUS_CTRL);
}

static int query_sp(struct ipu_bus_device *adev)
{
	struct ipu_psys *psys = ipu_bus_get_drvdata(adev);
	void __iomem *spc_regs_base = psys->pdata->base +
	    psys->pdata->ipdata->hw_variant.spc_offset;
	u32 val = readl(spc_regs_base + IPU_PSYS_REG_SPC_STATUS_CTRL);

	/* return true when READY == 1, START == 0 */
	val &= IPU_PSYS_SPC_STATUS_READY | IPU_PSYS_SPC_STATUS_START;

	return val == IPU_PSYS_SPC_STATUS_READY;
}
#else
static void start_sp(struct ipu6_bus_device *adev)
{
	struct ipu_psys *psys = ipu6_bus_get_drvdata(adev);
	void __iomem *spc_regs_base = psys->pdata->base +
	    psys->pdata->ipdata->hw_variant.spc_offset;
	u32 val = 0;

	val |= IPU6_PSYS_SPC_STATUS_START |
	    IPU6_PSYS_SPC_STATUS_RUN |
	    IPU6_PSYS_SPC_STATUS_CTRL_ICACHE_INVALIDATE;
	val |= psys->icache_prefetch_sp ?
	    IPU6_PSYS_SPC_STATUS_ICACHE_PREFETCH : 0;
	writel(val, spc_regs_base + IPU6_PSYS_REG_SPC_STATUS_CTRL);
}

static int query_sp(struct ipu6_bus_device *adev)
{
	struct ipu_psys *psys = ipu6_bus_get_drvdata(adev);
	void __iomem *spc_regs_base = psys->pdata->base +
	    psys->pdata->ipdata->hw_variant.spc_offset;
	u32 val = readl(spc_regs_base + IPU6_PSYS_REG_SPC_STATUS_CTRL);

	/* return true when READY == 1, START == 0 */
	val &= IPU6_PSYS_SPC_STATUS_READY | IPU6_PSYS_SPC_STATUS_START;

	return val == IPU6_PSYS_SPC_STATUS_READY;
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
static int ipu_psys_fw_init(struct ipu_psys *psys)
{
	unsigned int size;
	struct ipu_fw_syscom_queue_config *queue_cfg;
	struct ipu_fw_syscom_queue_config fw_psys_event_queue_cfg[] = {
		{
			IPU_FW_PSYS_EVENT_QUEUE_SIZE,
			sizeof(struct ipu_fw_psys_event)
		}
	};
	struct ipu_fw_psys_srv_init server_init = {
		.ddr_pkg_dir_address = 0,
		.host_ddr_pkg_dir = NULL,
		.pkg_dir_size = 0,
		.icache_prefetch_sp = psys->icache_prefetch_sp,
		.icache_prefetch_isp = psys->icache_prefetch_isp,
	};
	struct ipu_fw_com_cfg fwcom = {
		.num_output_queues = IPU_FW_PSYS_N_PSYS_EVENT_QUEUE_ID,
		.output = fw_psys_event_queue_cfg,
		.specific_addr = &server_init,
		.specific_size = sizeof(server_init),
		.cell_start = start_sp,
		.cell_ready = query_sp,
		.buttress_boot_offset = SYSCOM_BUTTRESS_FW_PARAMS_PSYS_OFFSET,
	};
	int i;

	size = IPU6SE_FW_PSYS_N_PSYS_CMD_QUEUE_ID;
	if (ipu_ver == IPU_VER_6 || ipu_ver == IPU_VER_6EP || ipu_ver == IPU_VER_6EP_MTL)
		size = IPU6_FW_PSYS_N_PSYS_CMD_QUEUE_ID;

	queue_cfg = devm_kzalloc(&psys->adev->dev, sizeof(*queue_cfg) * size,
				 GFP_KERNEL);
	if (!queue_cfg)
		return -ENOMEM;

	for (i = 0; i < size; i++) {
		queue_cfg[i].queue_size = IPU_FW_PSYS_CMD_QUEUE_SIZE;
		queue_cfg[i].token_size = sizeof(struct ipu_fw_psys_cmd);
	}

	fwcom.input = queue_cfg;
	fwcom.num_input_queues = size;
	fwcom.dmem_addr = psys->pdata->ipdata->hw_variant.dmem_offset;

	psys->fwcom = ipu_fw_com_prepare(&fwcom, psys->adev, psys->pdata->base);
	if (!psys->fwcom) {
		dev_err(&psys->adev->dev, "psys fw com prepare failed\n");
		return -EIO;
	}

	return 0;
}

static void run_fw_init_work(struct work_struct *work)
{
	struct fw_init_task *task = (struct fw_init_task *)work;
	struct ipu_psys *psys = task->psys;
	int rval;

	rval = ipu_psys_fw_init(psys);

	if (rval) {
		dev_err(&psys->adev->dev, "FW init failed(%d)\n", rval);
		ipu_psys_remove(psys->adev);
	} else {
		dev_info(&psys->adev->dev, "FW init done\n");
	}
}
#else
static int ipu_psys_fw_init(struct ipu_psys *psys)
{
	struct ipu6_fw_syscom_queue_config *queue_cfg;
	struct device *dev = &psys->adev->auxdev.dev;
	unsigned int size;
	struct ipu6_fw_syscom_queue_config fw_psys_event_queue_cfg[] = {
		{
			IPU_FW_PSYS_EVENT_QUEUE_SIZE,
			sizeof(struct ipu_fw_psys_event)
		}
	};
	struct ipu_fw_psys_srv_init server_init = {
		.ddr_pkg_dir_address = 0,
		.host_ddr_pkg_dir = NULL,
		.pkg_dir_size = 0,
		.icache_prefetch_sp = psys->icache_prefetch_sp,
		.icache_prefetch_isp = psys->icache_prefetch_isp,
	};
	struct ipu6_fw_com_cfg fwcom = {
		.num_output_queues = IPU_FW_PSYS_N_PSYS_EVENT_QUEUE_ID,
		.output = fw_psys_event_queue_cfg,
		.specific_addr = &server_init,
		.specific_size = sizeof(server_init),
		.cell_start = start_sp,
		.cell_ready = query_sp,
		.buttress_boot_offset = SYSCOM_BUTTRESS_FW_PARAMS_PSYS_OFFSET,
	};
	int i;

	size = IPU6SE_FW_PSYS_N_PSYS_CMD_QUEUE_ID;
	if (ipu_ver == IPU6_VER_6 || ipu_ver == IPU6_VER_6EP ||
	    ipu_ver == IPU6_VER_6EP_MTL)
		size = IPU6_FW_PSYS_N_PSYS_CMD_QUEUE_ID;

	queue_cfg = devm_kzalloc(dev, sizeof(*queue_cfg) * size,
				 GFP_KERNEL);
	if (!queue_cfg)
		return -ENOMEM;

	for (i = 0; i < size; i++) {
		queue_cfg[i].queue_size = IPU_FW_PSYS_CMD_QUEUE_SIZE;
		queue_cfg[i].token_size = sizeof(struct ipu_fw_psys_cmd);
	}

	fwcom.input = queue_cfg;
	fwcom.num_input_queues = size;
	fwcom.dmem_addr = psys->pdata->ipdata->hw_variant.dmem_offset;

	psys->fwcom = ipu6_fw_com_prepare(&fwcom, psys->adev,
					  psys->pdata->base);
	if (!psys->fwcom) {
		dev_err(dev, "psys fw com prepare failed\n");
		return -EIO;
	}

	return 0;
}

static void run_fw_init_work(struct work_struct *work)
{
	struct fw_init_task *task = (struct fw_init_task *)work;
	struct ipu_psys *psys = task->psys;
	struct device *dev = &psys->adev->auxdev.dev;
	int rval;

	rval = ipu_psys_fw_init(psys);

	if (rval) {
		dev_err(dev, "FW init failed(%d)\n", rval);
		ipu6_psys_remove(&psys->adev->auxdev);
	} else {
		dev_info(dev, "FW init done\n");
	}
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
static int ipu_psys_probe(struct ipu_bus_device *adev)
{
	struct ipu_device *isp = adev->isp;
	struct ipu_psys_pg *kpg, *kpg0;
	struct ipu_psys *psys;
	unsigned int minor;
	int i, rval = -E2BIG;

	/* firmware is not ready, so defer the probe */
	if (!isp->pkg_dir)
		return -EPROBE_DEFER;

	rval = ipu_mmu_hw_init(adev->mmu);
	if (rval)
		return rval;

	mutex_lock(&ipu_psys_mutex);

	minor = find_next_zero_bit(ipu_psys_devices, IPU_PSYS_NUM_DEVICES, 0);
	if (minor == IPU_PSYS_NUM_DEVICES) {
		dev_err(&adev->dev, "too many devices\n");
		goto out_unlock;
	}

	psys = devm_kzalloc(&adev->dev, sizeof(*psys), GFP_KERNEL);
	if (!psys) {
		rval = -ENOMEM;
		goto out_unlock;
	}

	psys->adev = adev;
	psys->pdata = adev->pdata;
	psys->icache_prefetch_sp = 0;

	psys->power_gating = 0;

	ipu_trace_init(adev->isp, psys->pdata->base, &adev->dev,
		       psys_trace_blocks);

	spin_lock_init(&psys->ready_lock);
	spin_lock_init(&psys->pgs_lock);
	psys->ready = 0;
	psys->timeout = IPU_PSYS_CMD_TIMEOUT_MS;

	mutex_init(&psys->mutex);
	INIT_LIST_HEAD(&psys->fhs);
	INIT_LIST_HEAD(&psys->pgs);
	INIT_LIST_HEAD(&psys->started_kcmds_list);

	init_waitqueue_head(&psys->sched_cmd_wq);
	atomic_set(&psys->wakeup_count, 0);
	/*
	 * Create a thread to schedule commands sent to IPU firmware.
	 * The thread reduces the coupling between the command scheduler
	 * and queueing commands from the user to driver.
	 */
	psys->sched_cmd_thread = kthread_run(ipu_psys_sched_cmd, psys,
					     "psys_sched_cmd");

	if (IS_ERR(psys->sched_cmd_thread)) {
		psys->sched_cmd_thread = NULL;
		mutex_destroy(&psys->mutex);
		goto out_unlock;
	}

	ipu_bus_set_drvdata(adev, psys);

	rval = ipu_psys_res_pool_init(&psys->res_pool_running);
	if (rval < 0) {
		dev_err(&psys->dev,
			"unable to alloc process group resources\n");
		goto out_mutex_destroy;
	}

	ipu6_psys_hw_res_variant_init();
	psys->pkg_dir = isp->pkg_dir;
	psys->pkg_dir_dma_addr = isp->pkg_dir_dma_addr;
	psys->pkg_dir_size = isp->pkg_dir_size;
	psys->fw_sgt = isp->fw_sgt;

	/* allocate and map memory for process groups */
	for (i = 0; i < IPU_PSYS_PG_POOL_SIZE; i++) {
		kpg = kzalloc(sizeof(*kpg), GFP_KERNEL);
		if (!kpg)
			goto out_free_pgs;
		kpg->pg = dma_alloc_attrs(&adev->dev,
					  IPU_PSYS_PG_MAX_SIZE,
					  &kpg->pg_dma_addr,
					  GFP_KERNEL, 0);
		if (!kpg->pg) {
			kfree(kpg);
			goto out_free_pgs;
		}
		kpg->size = IPU_PSYS_PG_MAX_SIZE;
		list_add(&kpg->list, &psys->pgs);
	}

	psys->caps.pg_count = ipu_cpd_pkg_dir_get_num_entries(psys->pkg_dir);

	dev_info(&adev->dev, "pkg_dir entry count:%d\n", psys->caps.pg_count);
	if (async_fw_init) {
		INIT_DELAYED_WORK((struct delayed_work *)&fw_init_task,
				  run_fw_init_work);
		fw_init_task.psys = psys;
		schedule_delayed_work((struct delayed_work *)&fw_init_task, 0);
	} else {
		rval = ipu_psys_fw_init(psys);
	}

	if (rval) {
		dev_err(&adev->dev, "FW init failed(%d)\n", rval);
		goto out_free_pgs;
	}

	psys->dev.parent = &adev->dev;
	psys->dev.bus = &ipu_psys_bus;
	psys->dev.devt = MKDEV(MAJOR(ipu_psys_dev_t), minor);
	psys->dev.release = ipu_psys_dev_release;
	dev_set_name(&psys->dev, "ipu-psys%d", minor);
	device_initialize(&psys->dev);

	cdev_init(&psys->cdev, &ipu_psys_fops);
	psys->cdev.owner = ipu_psys_fops.owner;

	rval = cdev_device_add(&psys->cdev, &psys->dev);
	if (rval < 0) {
		dev_err(&psys->dev, "psys device_register failed\n");
		goto out_release_fw_com;
	}

	set_bit(minor, ipu_psys_devices);

	/* Add the hw stepping information to caps */
	strscpy(psys->caps.dev_model, IPU_MEDIA_DEV_MODEL_NAME,
		sizeof(psys->caps.dev_model));

	mutex_unlock(&ipu_psys_mutex);

#ifdef CONFIG_DEBUG_FS
	/* Debug fs failure is not fatal. */
	ipu_psys_init_debugfs(psys);
#endif

	adev->isp->cpd_fw_reload = &cpd_fw_reload;

	dev_info(&adev->dev, "psys probe minor: %d\n", minor);

	ipu_mmu_hw_cleanup(adev->mmu);

	return 0;

out_release_fw_com:
	ipu_fw_com_release(psys->fwcom, 1);
out_free_pgs:
	list_for_each_entry_safe(kpg, kpg0, &psys->pgs, list) {
		dma_free_attrs(&adev->dev, kpg->size, kpg->pg,
			       kpg->pg_dma_addr, 0);
		kfree(kpg);
	}

	ipu_psys_res_pool_cleanup(&psys->res_pool_running);
out_mutex_destroy:
	mutex_destroy(&psys->mutex);
	if (psys->sched_cmd_thread) {
		kthread_stop(psys->sched_cmd_thread);
		psys->sched_cmd_thread = NULL;
	}
out_unlock:
	/* Safe to call even if the init is not called */
	ipu_trace_uninit(&adev->dev);
	mutex_unlock(&ipu_psys_mutex);

	ipu_mmu_hw_cleanup(adev->mmu);

	return rval;
}
#else
static int ipu6_psys_probe(struct auxiliary_device *auxdev,
			   const struct auxiliary_device_id *auxdev_id)
{
	struct ipu6_bus_device *adev = auxdev_to_adev(auxdev);
	struct device *dev = &auxdev->dev;
	struct ipu_psys_pg *kpg, *kpg0;
	struct ipu_psys *psys;
	unsigned int minor;
	int i, rval = -E2BIG;

	if (!adev->isp->bus_ready_to_probe)
		return -EPROBE_DEFER;

	if (!adev->pkg_dir)
		return -EPROBE_DEFER;

	ipu_ver = adev->isp->hw_ver;

	rval = ipu6_mmu_hw_init(adev->mmu);
	if (rval)
		return rval;

	mutex_lock(&ipu_psys_mutex);

	minor = find_next_zero_bit(ipu_psys_devices, IPU_PSYS_NUM_DEVICES, 0);
	if (minor == IPU_PSYS_NUM_DEVICES) {
		dev_err(dev, "too many devices\n");
		goto out_unlock;
	}

	psys = devm_kzalloc(dev, sizeof(*psys), GFP_KERNEL);
	if (!psys) {
		rval = -ENOMEM;
		goto out_unlock;
	}

	adev->auxdrv_data =
		(const struct ipu6_auxdrv_data *)auxdev_id->driver_data;
	adev->auxdrv = to_auxiliary_drv(dev->driver);

	psys->adev = adev;
	psys->pdata = adev->pdata;
	psys->icache_prefetch_sp = 0;

	psys->power_gating = 0;

	spin_lock_init(&psys->ready_lock);
	spin_lock_init(&psys->pgs_lock);
	psys->ready = 0;
	psys->timeout = IPU_PSYS_CMD_TIMEOUT_MS;

	mutex_init(&psys->mutex);
	INIT_LIST_HEAD(&psys->fhs);
	INIT_LIST_HEAD(&psys->pgs);
	INIT_LIST_HEAD(&psys->started_kcmds_list);

	init_waitqueue_head(&psys->sched_cmd_wq);
	atomic_set(&psys->wakeup_count, 0);
	/*
	 * Create a thread to schedule commands sent to IPU firmware.
	 * The thread reduces the coupling between the command scheduler
	 * and queueing commands from the user to driver.
	 */
	psys->sched_cmd_thread = kthread_run(ipu_psys_sched_cmd, psys,
					     "psys_sched_cmd");

	if (IS_ERR(psys->sched_cmd_thread)) {
		psys->sched_cmd_thread = NULL;
		mutex_destroy(&psys->mutex);
		goto out_unlock;
	}

	dev_set_drvdata(dev, psys);

	rval = ipu_psys_res_pool_init(&psys->res_pool_running);
	if (rval < 0) {
		dev_err(&psys->dev,
			"unable to alloc process group resources\n");
		goto out_mutex_destroy;
	}

	ipu6_psys_hw_res_variant_init();

	/* allocate and map memory for process groups */
	for (i = 0; i < IPU_PSYS_PG_POOL_SIZE; i++) {
		kpg = kzalloc(sizeof(*kpg), GFP_KERNEL);
		if (!kpg)
			goto out_free_pgs;
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 5)
		kpg->pg = dma_alloc_attrs(dev, IPU_PSYS_PG_MAX_SIZE,
					  &kpg->pg_dma_addr,
					  GFP_KERNEL, 0);
#else
		kpg->pg = ipu6_dma_alloc(adev, IPU_PSYS_PG_MAX_SIZE,
					 &kpg->pg_dma_addr,
					 GFP_KERNEL, 0);
#endif
		if (!kpg->pg) {
			kfree(kpg);
			goto out_free_pgs;
		}
		kpg->size = IPU_PSYS_PG_MAX_SIZE;
		list_add(&kpg->list, &psys->pgs);
	}

	psys->caps.pg_count = ipu6_cpd_pkg_dir_get_num_entries(adev->pkg_dir);

	dev_info(dev, "pkg_dir entry count:%d\n", psys->caps.pg_count);
	if (async_fw_init) {
		INIT_DELAYED_WORK((struct delayed_work *)&fw_init_task,
				  run_fw_init_work);
		fw_init_task.psys = psys;
		schedule_delayed_work((struct delayed_work *)&fw_init_task, 0);
	} else {
		rval = ipu_psys_fw_init(psys);
	}

	if (rval) {
		dev_err(dev, "FW init failed(%d)\n", rval);
		goto out_free_pgs;
	}

	psys->dev.bus = &ipu_psys_bus;
	psys->dev.parent = dev;
	psys->dev.devt = MKDEV(MAJOR(ipu_psys_dev_t), minor);
	psys->dev.release = ipu_psys_dev_release;
	dev_set_name(&psys->dev, "ipu-psys%d", minor);
	device_initialize(&psys->dev);

	cdev_init(&psys->cdev, &ipu_psys_fops);
	psys->cdev.owner = ipu_psys_fops.owner;

	rval = cdev_device_add(&psys->cdev, &psys->dev);
	if (rval < 0) {
		dev_err(dev, "psys device_register failed\n");
		goto out_release_fw_com;
	}

	set_bit(minor, ipu_psys_devices);

	/* Add the hw stepping information to caps */
	strscpy(psys->caps.dev_model, IPU6_MEDIA_DEV_MODEL_NAME,
		sizeof(psys->caps.dev_model));

	mutex_unlock(&ipu_psys_mutex);

	dev_info(dev, "psys probe minor: %d\n", minor);

	ipu6_mmu_hw_cleanup(adev->mmu);

	return 0;

out_release_fw_com:
	ipu6_fw_com_release(psys->fwcom, 1);
out_free_pgs:
	list_for_each_entry_safe(kpg, kpg0, &psys->pgs, list) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 5)
		dma_free_attrs(dev, kpg->size, kpg->pg, kpg->pg_dma_addr, 0);
#else
		ipu6_dma_free(adev, kpg->size, kpg->pg, kpg->pg_dma_addr, 0);
#endif
		kfree(kpg);
	}

	ipu_psys_res_pool_cleanup(&psys->res_pool_running);
out_mutex_destroy:
	mutex_destroy(&psys->mutex);
	if (psys->sched_cmd_thread) {
		kthread_stop(psys->sched_cmd_thread);
		psys->sched_cmd_thread = NULL;
	}
out_unlock:
	/* Safe to call even if the init is not called */
	mutex_unlock(&ipu_psys_mutex);
	ipu6_mmu_hw_cleanup(adev->mmu);
	return rval;
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
static void ipu_psys_remove(struct ipu_bus_device *adev)
{
	struct ipu_device *isp = adev->isp;
	struct ipu_psys *psys = ipu_bus_get_drvdata(adev);
#else
static void ipu6_psys_remove(struct auxiliary_device *auxdev)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 5)
	struct ipu6_bus_device *adev = auxdev_to_adev(auxdev);
#endif
	struct device *dev = &auxdev->dev;
	struct ipu_psys *psys = dev_get_drvdata(&auxdev->dev);
#endif
	struct ipu_psys_pg *kpg, *kpg0;

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
#ifdef CONFIG_DEBUG_FS
	if (isp->ipu_dir)
		debugfs_remove_recursive(psys->debugfsdir);
#endif
#endif

	if (psys->sched_cmd_thread) {
		kthread_stop(psys->sched_cmd_thread);
		psys->sched_cmd_thread = NULL;
	}

	mutex_lock(&ipu_psys_mutex);

	list_for_each_entry_safe(kpg, kpg0, &psys->pgs, list) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
		dma_free_attrs(&adev->dev, kpg->size, kpg->pg,
			       kpg->pg_dma_addr, 0);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 5)
		dma_free_attrs(dev, kpg->size, kpg->pg, kpg->pg_dma_addr, 0);
#else
		ipu6_dma_free(adev, kpg->size, kpg->pg, kpg->pg_dma_addr, 0);
#endif
		kfree(kpg);
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
	if (psys->fwcom && ipu_fw_com_release(psys->fwcom, 1))
		dev_err(&adev->dev, "fw com release failed.\n");
#else
	if (psys->fwcom && ipu6_fw_com_release(psys->fwcom, 1))
		dev_err(dev, "fw com release failed.\n");
#endif

	kfree(psys->server_init);
	kfree(psys->syscom_config);

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
	ipu_trace_uninit(&adev->dev);
#endif
	ipu_psys_res_pool_cleanup(&psys->res_pool_running);

	cdev_device_del(&psys->cdev, &psys->dev);

	clear_bit(MINOR(psys->cdev.dev), ipu_psys_devices);

	mutex_unlock(&ipu_psys_mutex);

	mutex_destroy(&psys->mutex);

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
	dev_info(&adev->dev, "removed\n");
#else
	dev_info(dev, "removed\n");
#endif
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
static irqreturn_t psys_isr_threaded(struct ipu_bus_device *adev)
{
	struct ipu_psys *psys = ipu_bus_get_drvdata(adev);
	void __iomem *base = psys->pdata->base;
	u32 status;
	int r;

	mutex_lock(&psys->mutex);
	r = pm_runtime_get_if_in_use(&psys->adev->dev);
	if (!r || WARN_ON_ONCE(r < 0)) {
		mutex_unlock(&psys->mutex);
		return IRQ_NONE;
	}

	status = readl(base + IPU_REG_PSYS_GPDEV_IRQ_STATUS);
	writel(status, base + IPU_REG_PSYS_GPDEV_IRQ_CLEAR);

	if (status & IPU_PSYS_GPDEV_IRQ_FWIRQ(IPU_PSYS_GPDEV_FWIRQ0)) {
		writel(0, base + IPU_REG_PSYS_GPDEV_FWIRQ(0));
		ipu_psys_handle_events(psys);
	}

	pm_runtime_put(&psys->adev->dev);
	mutex_unlock(&psys->mutex);

	return status ? IRQ_HANDLED : IRQ_NONE;
}
#else
static irqreturn_t psys_isr_threaded(struct ipu6_bus_device *adev)
{
	struct ipu_psys *psys = ipu6_bus_get_drvdata(adev);
	struct device *dev = &psys->adev->auxdev.dev;
	void __iomem *base = psys->pdata->base;
	u32 status;
	int r;

	mutex_lock(&psys->mutex);
	r = pm_runtime_get_if_in_use(dev);
	if (!r || WARN_ON_ONCE(r < 0)) {
		mutex_unlock(&psys->mutex);
		return IRQ_NONE;
	}

	status = readl(base + IPU6_REG_PSYS_GPDEV_IRQ_STATUS);
	writel(status, base + IPU6_REG_PSYS_GPDEV_IRQ_CLEAR);

	if (status & IPU6_PSYS_GPDEV_IRQ_FWIRQ(IPU6_PSYS_GPDEV_FWIRQ0)) {
		writel(0, base + IPU6_REG_PSYS_GPDEV_FWIRQ(0));
		ipu_psys_handle_events(psys);
	}

	pm_runtime_put(dev);
	mutex_unlock(&psys->mutex);

	return status ? IRQ_HANDLED : IRQ_NONE;
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
static struct ipu_bus_driver ipu_psys_driver = {
	.probe = ipu_psys_probe,
	.remove = ipu_psys_remove,
	.isr_threaded = psys_isr_threaded,
	.wanted = IPU_PSYS_NAME,
	.drv = {
		.name = IPU_PSYS_NAME,
		.owner = THIS_MODULE,
		.pm = &psys_pm_ops,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
};

static const struct pci_device_id ipu_pci_tbl[] = {
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IPU6_PCI_ID)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IPU6SE_PCI_ID)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IPU6EP_ADL_P_PCI_ID)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IPU6EP_ADL_N_PCI_ID)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IPU6EP_RPL_P_PCI_ID)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IPU6EP_MTL_PCI_ID)},
	{0,}
};
MODULE_DEVICE_TABLE(pci, ipu_pci_tbl);
#else
static const struct ipu6_auxdrv_data ipu6_psys_auxdrv_data = {
	.isr_threaded = psys_isr_threaded,
	.wake_isr_thread = true,
};

static const struct auxiliary_device_id ipu6_psys_id_table[] = {
	{
		.name = "intel_ipu6.psys",
		.driver_data = (kernel_ulong_t)&ipu6_psys_auxdrv_data,
	},
	{ }
};
MODULE_DEVICE_TABLE(auxiliary, ipu6_psys_id_table);

static struct auxiliary_driver ipu6_psys_aux_driver = {
	.name = IPU6_PSYS_NAME,
	.probe = ipu6_psys_probe,
	.remove = ipu6_psys_remove,
	.id_table = ipu6_psys_id_table,
	.driver = {
		.pm = &psys_pm_ops,
	},
};
#endif

static int __init ipu_psys_init(void)
{
	int rval = alloc_chrdev_region(&ipu_psys_dev_t, 0,
				       IPU_PSYS_NUM_DEVICES, ipu_psys_bus.name);
	if (rval) {
		pr_err("can't alloc psys chrdev region (%d)\n", rval);
		return rval;
	}

	rval = bus_register(&ipu_psys_bus);
	if (rval) {
		pr_err("can't register psys bus (%d)\n", rval);
		unregister_chrdev_region(ipu_psys_dev_t, IPU_PSYS_NUM_DEVICES);
		return rval;
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
	ipu_bus_register_driver(&ipu_psys_driver);
#else
	auxiliary_driver_register(&ipu6_psys_aux_driver);
#endif
	return 0;
}
module_init(ipu_psys_init);

static void __exit ipu_psys_exit(void)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
	ipu_bus_unregister_driver(&ipu_psys_driver);
#else
	auxiliary_driver_unregister(&ipu6_psys_aux_driver);
#endif
	bus_unregister(&ipu_psys_bus);
	unregister_chrdev_region(ipu_psys_dev_t, IPU_PSYS_NUM_DEVICES);
}
module_exit(ipu_psys_exit);

MODULE_AUTHOR("Bingbu Cao <bingbu.cao@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Intel IPU6 processing system driver");
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 13, 0)
MODULE_IMPORT_NS(DMA_BUF);
MODULE_IMPORT_NS(INTEL_IPU6);
#else
MODULE_IMPORT_NS("DMA_BUF");
MODULE_IMPORT_NS("INTEL_IPU6");
#endif
