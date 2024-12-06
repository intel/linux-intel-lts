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
#include <uapi/linux/sched/types.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>

#include <uapi/linux/ipu-psys.h>

#include "ipu6.h"
#include "ipu6-mmu.h"
#include "ipu6-bus.h"
#include "ipu6-buttress.h"
#include "ipu6-cpd.h"
#include "ipu-fw-psys.h"
#include "ipu-psys.h"
#include "ipu6-platform-regs.h"
#include "ipu6-fw-com.h"

static bool async_fw_init;
module_param(async_fw_init, bool, 0664);
MODULE_PARM_DESC(async_fw_init, "Enable asynchronous firmware initialization");

#define SYSCOM_BUTTRESS_FW_PARAMS_PSYS_OFFSET	7

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

static void ipu6_psys_remove(struct auxiliary_device *auxdev);

static const struct bus_type ipu6_psys_bus = {
	.name = "intel-ipu6-psys",
};

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
	struct device *dev = &psys->adev->auxdev.dev;
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

	kpg->pg = dma_alloc_attrs(dev, pg_size,  &kpg->pg_dma_addr,
				  GFP_KERNEL, 0);
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

	start = (unsigned long)attach->userptr;
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
	if (nr < npages)
		goto error;

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
	if (nr)
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

static int ipu_dma_buf_attach(struct dma_buf *dbuf,
			      struct dma_buf_attachment *attach)
{
	struct ipu_psys_kbuffer *kbuf = dbuf->priv;
	struct ipu_dma_buf_attach *ipu_attach;
	int ret;

	ipu_attach = kzalloc(sizeof(*ipu_attach), GFP_KERNEL);
	if (!ipu_attach)
		return -ENOMEM;

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
	unsigned long attrs;
	int ret;

	attrs = DMA_ATTR_SKIP_CPU_SYNC;
	ret = dma_map_sgtable(attach->dev, ipu_attach->sgt, dir, attrs);
	if (ret < 0) {
		dev_dbg(attach->dev, "buf map failed\n");

		return ERR_PTR(-EIO);
	}

	/*
	 * Initial cache flush to avoid writing dirty pages for buffers which
	 * are later marked as IPU_BUFFER_FLAG_NO_FLUSH.
	 */
	dma_sync_sg_for_device(attach->dev, ipu_attach->sgt->sgl,
			       ipu_attach->sgt->orig_nents, DMA_BIDIRECTIONAL);

	return ipu_attach->sgt;
}

static void ipu_dma_buf_unmap(struct dma_buf_attachment *attach,
			      struct sg_table *sgt, enum dma_data_direction dir)
{
	dma_unmap_sgtable(attach->dev, sgt, dir, DMA_ATTR_SKIP_CPU_SYNC);
}

static int ipu_dma_buf_mmap(struct dma_buf *dbuf, struct vm_area_struct *vma)
{
	return -ENOTTY;
}

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
					enum dma_data_direction dir)
{
	return -ENOTTY;
}

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

static const struct dma_buf_ops ipu_dma_buf_ops = {
	.attach = ipu_dma_buf_attach,
	.detach = ipu_dma_buf_detach,
	.map_dma_buf = ipu_dma_buf_map,
	.unmap_dma_buf = ipu_dma_buf_unmap,
	.release = ipu_dma_buf_release,
	.begin_cpu_access = ipu_dma_buf_begin_cpu_access,
	.mmap = ipu_dma_buf_mmap,
	.vmap = ipu_dma_buf_vmap,
	.vunmap = ipu_dma_buf_vunmap,
};

static int ipu_psys_open(struct inode *inode, struct file *file)
{
	struct ipu_psys *psys = inode_to_ipu_psys(inode);
	struct ipu_psys_fh *fh;
	int rval;

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

static inline void ipu_psys_kbuf_unmap(struct ipu_psys_kbuffer *kbuf)
{
	if (!kbuf)
		return;

	kbuf->valid = false;
	if (kbuf->kaddr) {
		struct iosys_map dmap;

		iosys_map_set_vaddr(&dmap, kbuf->kaddr);
		dma_buf_vunmap_unlocked(kbuf->dbuf, &dmap);
	}
	if (!IS_ERR_OR_NULL(kbuf->sgt))
		dma_buf_unmap_attachment_unlocked(kbuf->db_attach,
						  kbuf->sgt,
						  DMA_BIDIRECTIONAL);
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
	ipu_psys_kbuf_unmap(kbuf);
	ipu_buffer_del(fh, kbuf);
	if (!kbuf->userptr)
		kfree(kbuf);
}

static int ipu_psys_unmapbuf_locked(int fd, struct ipu_psys_fh *fh)
{
	struct ipu_psys *psys = fh->psys;
	struct device *dev = &psys->adev->auxdev.dev;
	struct ipu_psys_kbuffer *kbuf;
	struct ipu_psys_desc *desc;

	desc = psys_desc_lookup(fh, fd);
	if (WARN_ON_ONCE(!desc)) {
		dev_err(dev, "descriptor not found: %d\n", fd);
		return -EINVAL;
	}

	kbuf = desc->kbuf;
	/* descriptor is gone now */
	ipu_desc_del(fh, desc);
	kfree(desc);

	if (WARN_ON_ONCE(!kbuf || !kbuf->dbuf)) {
		dev_err(dev,
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
			ipu_psys_kbuf_unmap(kbuf);
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
	struct device *dev = &psys->adev->auxdev.dev;
	struct ipu_psys_desc *desc;
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	struct dma_buf *dbuf;
	int ret;

	if (!buf->base.userptr) {
		dev_err(dev, "Buffer allocation not supported\n");
		return -EINVAL;
	}

	kbuf = ipu_psys_kbuffer_alloc();
	if (!kbuf)
		return -ENOMEM;

	kbuf->len = buf->len;
	kbuf->userptr = buf->base.userptr;
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

	dev_dbg(dev, "IOC_GETBUF: userptr %p size %llu to fd %d",
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
	struct device *dev = &psys->adev->auxdev.dev;
	struct ipu_psys_kbuffer *kbuf;
	struct ipu_psys_desc *desc;
	struct dma_buf *dbuf;
	struct iosys_map dmap = {
		.is_iomem = false,
	};

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
		dev_dbg(dev, "fd %d has been mapped!\n", fd);
		dma_buf_put(dbuf);
		goto mapbuf_end;
	}

	kbuf->dbuf = dbuf;

	if (kbuf->len == 0)
		kbuf->len = kbuf->dbuf->size;

	kbuf->db_attach = dma_buf_attach(kbuf->dbuf, dev);
	if (IS_ERR(kbuf->db_attach)) {
		dev_dbg(dev, "dma buf attach failed\n");
		goto kbuf_map_fail;
	}

	kbuf->sgt = dma_buf_map_attachment_unlocked(kbuf->db_attach,
						    DMA_BIDIRECTIONAL);
	if (IS_ERR_OR_NULL(kbuf->sgt)) {
		kbuf->sgt = NULL;
		dev_dbg(dev, "dma buf map attachment failed\n");
		goto kbuf_map_fail;
	}

	kbuf->dma_addr = sg_dma_address(kbuf->sgt->sgl);

	/* no need vmap for imported dmabufs */
	if (!kbuf->userptr)
		goto mapbuf_end;

	if (dma_buf_vmap_unlocked(kbuf->dbuf, &dmap)) {
		dev_dbg(dev, "dma buf vmap failed\n");
		goto kbuf_map_fail;
	}
	kbuf->kaddr = dmap.vaddr;

	dev_dbg(dev, "%s kbuf %p fd %d with len %llu mapped\n",
		__func__, kbuf, fd, kbuf->len);

mapbuf_end:
	kbuf->valid = true;
	return kbuf;

kbuf_map_fail:
	ipu_buffer_del(fh, kbuf);
	ipu_psys_kbuf_unmap(kbuf);
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

	mutex_lock(&fh->mutex);
	kbuf = ipu_psys_mapbuf_locked(fd, fh);
	mutex_unlock(&fh->mutex);

	dev_dbg(&fh->psys->adev->auxdev.dev, "IOC_MAPBUF\n");

	return kbuf ? 0 : -EINVAL;
}

static long ipu_psys_unmapbuf(int fd, struct ipu_psys_fh *fh)
{
	long ret;

	mutex_lock(&fh->mutex);
	ret = ipu_psys_unmapbuf_locked(fd, fh);
	mutex_unlock(&fh->mutex);

	dev_dbg(&fh->psys->adev->auxdev.dev, "IOC_UNMAPBUF\n");

	return ret;
}

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

static long ipu_get_manifest(struct ipu_psys_manifest *manifest,
			     struct ipu_psys_fh *fh)
{
	struct ipu_psys *psys = fh->psys;
	struct device *dev = &psys->adev->auxdev.dev;
	struct ipu6_bus_device *adev = psys->adev;
	struct ipu6_device *isp = adev->isp;
	struct ipu6_cpd_client_pkg_hdr *client_pkg;
	u32 entries;
	void *host_fw_data;
	dma_addr_t dma_fw_data;
	u32 client_pkg_offset;

	host_fw_data = (void *)isp->cpd_fw->data;
	dma_fw_data = sg_dma_address(adev->fw_sgt.sgl);
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

#ifdef CONFIG_PM
static int psys_runtime_pm_resume(struct device *dev)
{
	struct ipu6_bus_device *adev = to_ipu6_bus_device(dev);
	struct ipu_psys *psys = ipu6_bus_get_drvdata(adev);
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

	retval = ipu6_mmu_hw_init(adev->mmu);
	if (retval)
		return retval;

	if (async_fw_init && !psys->fwcom) {
		dev_err(dev,
			"%s: asynchronous firmware init not finished, skipping\n",
			__func__);
		return 0;
	}

	if (!ipu6_buttress_auth_done(adev->isp)) {
		dev_dbg(dev, "%s: not yet authenticated, skipping\n", __func__);
		return 0;
	}

	ipu_psys_setup_hw(psys);

	ipu_psys_subdomains_power(psys, 1);
	ipu6_configure_spc(adev->isp,
			   &psys->pdata->ipdata->hw_variant,
			   IPU6_CPD_PKG_DIR_PSYS_SERVER_IDX,
			   psys->pdata->base, adev->pkg_dir,
			   adev->pkg_dir_dma_addr);

	retval = ipu_fw_psys_open(psys);
	if (retval) {
		dev_err(dev, "Failed to open abi.\n");
		return retval;
	}

	spin_lock_irqsave(&psys->ready_lock, flags);
	psys->ready = 1;
	spin_unlock_irqrestore(&psys->ready_lock, flags);

	return 0;
}

static int psys_runtime_pm_suspend(struct device *dev)
{
	struct ipu6_bus_device *adev = to_ipu6_bus_device(dev);
	struct ipu_psys *psys = ipu6_bus_get_drvdata(adev);
	unsigned long flags;
	int rval;

	if (!psys)
		return 0;

	if (!psys->ready)
		return 0;

	spin_lock_irqsave(&psys->ready_lock, flags);
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

	ipu6_mmu_hw_cleanup(adev->mmu);

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

#define PSYS_PM_OPS (&psys_pm_ops)
#else
#define PSYS_PM_OPS NULL
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

	rval = alloc_chrdev_region(&ipu_psys_dev_t, 0,
				   IPU_PSYS_NUM_DEVICES, IPU6_PSYS_NAME);
	if (rval) {
		dev_err(dev, "can't alloc psys chrdev region (%d)\n",
			rval);
		return rval;
	}

	rval = ipu6_mmu_hw_init(adev->mmu);
	if (rval)
		goto out_unregister_chr_region;

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

	cdev_init(&psys->cdev, &ipu_psys_fops);
	psys->cdev.owner = ipu_psys_fops.owner;

	rval = cdev_add(&psys->cdev, MKDEV(MAJOR(ipu_psys_dev_t), minor), 1);
	if (rval) {
		dev_err(dev, "cdev_add failed (%d)\n", rval);
		goto out_unlock;
	}

	set_bit(minor, ipu_psys_devices);

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

	rval = ipu_psys_resource_pool_init(&psys->resource_pool_running);
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
		kpg->pg = dma_alloc_attrs(dev, IPU_PSYS_PG_MAX_SIZE,
					  &kpg->pg_dma_addr,
					  GFP_KERNEL, 0);
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

	psys->dev.bus = &ipu6_psys_bus;
	psys->dev.parent = dev;
	psys->dev.devt = MKDEV(MAJOR(ipu_psys_dev_t), minor);
	psys->dev.release = ipu_psys_dev_release;
	dev_set_name(&psys->dev, "ipu-psys%d", minor);
	rval = device_register(&psys->dev);
	if (rval < 0) {
		dev_err(dev, "psys device_register failed\n");
		goto out_release_fw_com;
	}

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
		dma_free_attrs(dev, kpg->size, kpg->pg, kpg->pg_dma_addr, 0);
		kfree(kpg);
	}

	ipu_psys_resource_pool_cleanup(&psys->resource_pool_running);
out_mutex_destroy:
	mutex_destroy(&psys->mutex);
	cdev_del(&psys->cdev);
	if (psys->sched_cmd_thread) {
		kthread_stop(psys->sched_cmd_thread);
		psys->sched_cmd_thread = NULL;
	}
out_unlock:
	/* Safe to call even if the init is not called */
	mutex_unlock(&ipu_psys_mutex);
	ipu6_mmu_hw_cleanup(adev->mmu);

out_unregister_chr_region:
	unregister_chrdev_region(ipu_psys_dev_t, IPU_PSYS_NUM_DEVICES);

	return rval;
}

static void ipu6_psys_remove(struct auxiliary_device *auxdev)
{
	struct device *dev = &auxdev->dev;
	struct ipu_psys *psys = dev_get_drvdata(&auxdev->dev);
	struct ipu_psys_pg *kpg, *kpg0;

	unregister_chrdev_region(ipu_psys_dev_t, IPU_PSYS_NUM_DEVICES);

	if (psys->sched_cmd_thread) {
		kthread_stop(psys->sched_cmd_thread);
		psys->sched_cmd_thread = NULL;
	}

	mutex_lock(&ipu_psys_mutex);

	list_for_each_entry_safe(kpg, kpg0, &psys->pgs, list) {
		dma_free_attrs(dev, kpg->size, kpg->pg, kpg->pg_dma_addr, 0);
		kfree(kpg);
	}

	if (psys->fwcom && ipu6_fw_com_release(psys->fwcom, 1))
		dev_err(dev, "fw com release failed.\n");

	kfree(psys->server_init);
	kfree(psys->syscom_config);

	ipu_psys_resource_pool_cleanup(&psys->resource_pool_running);

	device_unregister(&psys->dev);

	clear_bit(MINOR(psys->cdev.dev), ipu_psys_devices);
	cdev_del(&psys->cdev);

	mutex_unlock(&ipu_psys_mutex);

	mutex_destroy(&psys->mutex);

	dev_info(dev, "removed\n");
}

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
module_auxiliary_driver(ipu6_psys_aux_driver);

MODULE_AUTHOR("Antti Laakso <antti.laakso@intel.com>");
MODULE_AUTHOR("Bin Han <bin.b.han@intel.com>");
MODULE_AUTHOR("Renwei Wu <renwei.wu@intel.com>");
MODULE_AUTHOR("Jianxu Zheng <jian.xu.zheng@intel.com>");
MODULE_AUTHOR("Xia Wu <xia.wu@intel.com>");
MODULE_AUTHOR("Bingbu Cao <bingbu.cao@intel.com>");
MODULE_AUTHOR("Zaikuo Wang <zaikuo.wang@intel.com>");
MODULE_AUTHOR("Yunliang Ding <yunliang.ding@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Intel ipu processing system driver");
MODULE_IMPORT_NS(DMA_BUF);
MODULE_IMPORT_NS(INTEL_IPU6);
