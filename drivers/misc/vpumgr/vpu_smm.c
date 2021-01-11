// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Intel Corporation
 */
#include <linux/cdev.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/dma-buf.h>
#include <linux/dma-buf-map.h>
#include <linux/dma-direct.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/rbtree.h>
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>

#include <linux/sched.h>
#include <linux/sched/mm.h>
#include <linux/sched/task.h>

#include <uapi/misc/vpumgr.h>

#include "vpu_common.h"
#include "vpu_smm.h"

/*
 * DMABuf exported by VPU device is described by following data structure
 * the life of the buffer may be longer than session(it may be shared with
 * other driver),so it contains pointer to device rather than to session
 */
struct vpusmm_buffer {
	struct device *dev;
	void *cookie;
	dma_addr_t dma_addr;
	size_t size;
	unsigned long dma_attrs;
};

/*
 * DMABufs imported to VPU device are maintained in a rb tree with dmabuf's
 * pointer as key. so user space can unimport it by specifying fd and ptr2vpu
 * API can work by searching this rb tree.
 */
struct vpusmm_impbuf {
	struct rb_node node;
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	enum dma_data_direction direction;
	dma_addr_t vpu_addr;
	int refcount;
};

/*
 * VPU imported dmabuf management
 */
static void vpusmm_insert_impbuf(struct vpumgr_smm *sess,
				 struct vpusmm_impbuf *new_item)
{
	struct rb_root *root = &sess->imp_rb;
	struct rb_node **iter = &root->rb_node, *parent = NULL;
	struct dma_buf *value = new_item->dmabuf;
	struct vpusmm_impbuf *item;

	while (*iter) {
		parent = *iter;
		item = rb_entry(parent, struct vpusmm_impbuf, node);

		if (item->dmabuf > value)
			iter = &(*iter)->rb_left;
		else
			iter = &(*iter)->rb_right;
	}

	/* Put the new node there */
	rb_link_node(&new_item->node, parent, iter);
	rb_insert_color(&new_item->node, root);
}

static struct vpusmm_impbuf *vpusmm_find_impbuf(struct vpumgr_smm *sess,
						struct dma_buf *dmabuf)
{
	struct rb_node *node = sess->imp_rb.rb_node;
	struct vpusmm_impbuf *item;

	while (node) {
		item = rb_entry(node, struct vpusmm_impbuf, node);

		if (item->dmabuf > dmabuf)
			node = node->rb_left;
		else if (item->dmabuf < dmabuf)
			node = node->rb_right;
		else
			return item;
	}
	return NULL;
}

static struct vpusmm_impbuf *vpusmm_import_dmabuf(struct vpumgr_smm *sess,
						  int dmabuf_fd,
						  enum dma_data_direction direction, int vpu_id)
{
	struct vpusmm_impbuf *item;
	struct dma_buf_attachment *attach;
	struct device *dma_dev = sess->vdev->dev;
	struct dma_buf *dmabuf;
	struct sg_table *sgt;

	dmabuf = dma_buf_get(dmabuf_fd);
	if (IS_ERR(dmabuf))
		return ERR_CAST(dmabuf);

	mutex_lock(&sess->imp_rb_lock);
	item = vpusmm_find_impbuf(sess, dmabuf);
	if (item) {
		item->refcount++;
		goto found_impbuf;
	}

	attach = dma_buf_attach(dmabuf, dma_dev);
	if (IS_ERR(attach)) {
		item = ERR_CAST(attach);
		goto fail_attach;
	}

	sgt = dma_buf_map_attachment(attach, direction);
	if (IS_ERR(sgt)) {
		item = ERR_CAST(sgt);
		goto fail_map;
	}

	if (sgt->nents > 1) {
		item = ERR_PTR(-EINVAL);
		goto fail_import;
	}

	item = kzalloc(sizeof(*item), GFP_KERNEL);
	if (!item) {
		item = ERR_PTR(-ENOMEM);
		goto fail_import;
	}

	item->dmabuf	= dmabuf;
	item->attach	= attach;
	item->sgt	= sgt;
	item->direction = direction;
	item->vpu_addr	= sg_dma_address(sgt->sgl);
	item->refcount	= 1;

	vpusmm_insert_impbuf(sess, item);

	mutex_unlock(&sess->imp_rb_lock);
	return item;

fail_import:
	dma_buf_unmap_attachment(attach, sgt, direction);
fail_map:
	dma_buf_detach(dmabuf, attach);
fail_attach:
found_impbuf:
	mutex_unlock(&sess->imp_rb_lock);
	dma_buf_put(dmabuf);
	return item;
}

int smm_open(struct vpumgr_smm *sess, struct vpumgr_device *vdev)
{
	sess->vdev = vdev;
	sess->imp_rb = RB_ROOT;
	mutex_init(&sess->imp_rb_lock);
	return 0;
}

int smm_close(struct vpumgr_smm *sess)
{
	struct device *dev = sess->vdev->sdev;
	struct rb_node *cur, *next;
	struct vpusmm_impbuf *item;

	mutex_destroy(&sess->imp_rb_lock);

	cur = rb_first(&sess->imp_rb);
	while (cur) {
		item = rb_entry(cur, struct vpusmm_impbuf, node);
		next = rb_next(cur);
		if (item) {
			dev_dbg(dev, "[%s] PID:%d free leaked imported dmabuf %zu bytes, %d refs\n",
				__func__, current->pid, item->dmabuf->size, item->refcount);
			dma_buf_unmap_attachment(item->attach, item->sgt, item->direction);
			dma_buf_detach(item->dmabuf, item->attach);
			dma_buf_put(item->dmabuf);
			rb_erase(&item->node, &sess->imp_rb);
			kfree(item);
		}
		cur = next;
	}
	return 0;
}

/*
 *  DMABuf
 */
static struct sg_table *map_dma_buf_vpusmm(struct dma_buf_attachment *attach,
					   enum dma_data_direction dir)
{
	struct vpusmm_buffer *buff = attach->dmabuf->priv;
	struct sg_table *sgt;
	int rc;

	if (WARN_ON(dir == DMA_NONE))
		return ERR_PTR(-EINVAL);

	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return NULL;

	rc = dma_get_sgtable(buff->dev, sgt, buff->cookie, buff->dma_addr, buff->size);
	if (rc < 0)
		goto free_sgt;

	rc = dma_map_sg_attrs(attach->dev, sgt->sgl, sgt->nents, dir, DMA_ATTR_SKIP_CPU_SYNC);
	if (!rc)
		goto free_sg_table;
	return sgt;

free_sg_table:
	sg_free_table(sgt);
free_sgt:
	kfree(sgt);
	return ERR_PTR(rc);
}

static void unmap_dma_buf_vpusmm(struct dma_buf_attachment *attach,
				 struct sg_table *sgt, enum dma_data_direction dir)
{
	dma_unmap_sg_attrs(attach->dev, sgt->sgl, sgt->nents, dir, DMA_ATTR_SKIP_CPU_SYNC);
	sg_free_table(sgt);
	kfree(sgt);
}

static void release_vpusmm(struct dma_buf *dmabuf)
{
	struct vpusmm_buffer *buff = dmabuf->priv;

	dma_free_attrs(buff->dev, buff->size, buff->cookie, buff->dma_addr, buff->dma_attrs);
	kfree(buff);
}

static int mmap_vpusmm(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	struct vpusmm_buffer *buff = dmabuf->priv;
	unsigned long vm_size;

	vm_size = vma->vm_end - vma->vm_start;
	if (vm_size > PAGE_ALIGN(buff->size))
		return -EINVAL;

	vma->vm_flags |= VM_IO | VM_DONTEXPAND | VM_DONTDUMP;
	vma->vm_pgoff = 0;

	return dma_mmap_attrs(buff->dev, vma, buff->cookie, buff->dma_addr,
			    buff->size, buff->dma_attrs);
}

static int vmap_vpusmm(struct dma_buf *dmabuf, struct dma_buf_map *map)
{
	struct vpusmm_buffer *buff = dmabuf->priv;
	void *vaddr = buff->cookie;

	if (IS_ERR(vaddr))
		return PTR_ERR(vaddr);

	dma_buf_map_set_vaddr(map, vaddr);
	return 0;
}

static const struct dma_buf_ops vpusmm_dmabuf_ops =  {
	.cache_sgt_mapping = true,
	.map_dma_buf	= map_dma_buf_vpusmm,
	.unmap_dma_buf	= unmap_dma_buf_vpusmm,
	.release		= release_vpusmm,
	.mmap			= mmap_vpusmm,
	.vmap			= vmap_vpusmm,
};

/*
 * Allocate dma buffer suitable for VPU access.
 * export as DMABuf fd
 * sess will hold additional refcount to the dmabuf
 * on request of passing it to VPU side for processing
 */
int smm_alloc(struct vpumgr_smm *sess, struct vpumgr_args_alloc *arg)
{
	struct vpumgr_device *vdev = sess->vdev;
	const int flags = O_RDWR | O_CLOEXEC;
	size_t buffer_size = arg->size;
	struct dma_buf *dmabuf = NULL;
	phys_addr_t phys_addr;
	struct dma_buf_export_info exp_info = {
		.exp_name = dev_name(vdev->sdev),
		.owner = THIS_MODULE,
		.ops = &vpusmm_dmabuf_ops,
		.size = buffer_size,
		.flags = flags
	};
	struct vpusmm_buffer *buff;
	int retval;

	buff = kzalloc(sizeof(*buff), GFP_KERNEL);
	if (!buff) {
		retval = -ENOMEM;
		goto failed;
	}

	buff->dev = vdev->dev;
	buff->size = buffer_size;
	buff->dma_attrs = DMA_ATTR_FORCE_CONTIGUOUS | DMA_ATTR_WRITE_COMBINE;
	buff->cookie = dma_alloc_attrs(vdev->dev, buff->size, &buff->dma_addr,
				       GFP_KERNEL | GFP_DMA, buff->dma_attrs);
	if (!buff->cookie) {
		retval = -ENOMEM;
		goto failed;
	}

	phys_addr = dma_to_phys(vdev->dev, buff->dma_addr);

	exp_info.priv = buff;
	dmabuf = dma_buf_export(&exp_info);
	if (IS_ERR(dmabuf)) {
		retval = PTR_ERR(dmabuf);
		dmabuf = NULL;
		goto failed;
	}

	retval = dma_buf_fd(dmabuf, flags);
	if (retval < 0) {
		goto failed;
	} else {
		arg->fd = retval;
		retval = 0;
	}

	dev_dbg(vdev->dev, "%s: dma_addr=%llx, phys_addr=%llx allocated from %s\n",
		__func__, buff->dma_addr, phys_addr, dev_name(vdev->dev));

	return 0;
failed:
	dev_err(vdev->dev, "%s failed with %d\n", __func__, retval);

	if (dmabuf) {
		/* this will finally release underlying buff */
		dma_buf_put(dmabuf);
	} else if (buff) {
		if (buff->cookie)
			dma_free_attrs(vdev->dev, buff->size, buff->cookie,
				       buff->dma_addr, buff->dma_attrs);
		kfree(buff);
	}
	return retval;
}

int smm_import(struct vpumgr_smm *sess, struct vpumgr_args_import *arg)
{
	struct device *dev = sess->vdev->sdev;
	enum dma_data_direction direction;
	struct vpusmm_impbuf *item;

	switch (arg->vpu_access) {
	case VPU_ACCESS_READ:
		direction = DMA_TO_DEVICE;
		break;
	case VPU_ACCESS_WRITE:
		direction = DMA_FROM_DEVICE;
		break;
	case VPU_ACCESS_DEFAULT:
	case VPU_ACCESS_RW:
		direction = DMA_BIDIRECTIONAL;
		break;
	default:
		dev_err(dev, "Unknown vpu_access:%d\n", arg->vpu_access);
		return -EINVAL;
	}

	item = vpusmm_import_dmabuf(sess, arg->fd, direction, 0);
	if (IS_ERR(item))
		return PTR_ERR(item);

	arg->vpu_addr = item->vpu_addr;
	arg->size = item->dmabuf->size;
	return 0;
}

int smm_unimport(struct vpumgr_smm *sess, int *p_dmabuf_fd)
{
	struct vpusmm_impbuf *item;
	struct dma_buf *dmabuf;
	int rc = 0;

	dmabuf = dma_buf_get(*p_dmabuf_fd);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	mutex_lock(&sess->imp_rb_lock);
	item = vpusmm_find_impbuf(sess, dmabuf);
	if (!item) {
		rc = -EINVAL;
		goto exit;
	}

	item->refcount--;
	if (item->refcount <= 0) {
		rb_erase(&item->node, &sess->imp_rb);
		dma_buf_unmap_attachment(item->attach, item->sgt, item->direction);
		dma_buf_detach(item->dmabuf, item->attach);
		dma_buf_put(item->dmabuf);
		kfree(item);
	}
exit:
	mutex_unlock(&sess->imp_rb_lock);
	dma_buf_put(dmabuf);
	return rc;
}

int smm_ptr2vpu(struct vpumgr_smm *sess, unsigned long *arg)
{
	struct device *dev = sess->vdev->sdev;
	struct task_struct *task = current;
	struct dma_buf *dmabuf = NULL;
	unsigned long vaddr = *arg;
	struct vm_area_struct *vma;
	struct vpusmm_impbuf *item;
	struct mm_struct *mm;

	mm = get_task_mm(task);
	if (!mm)
		goto failed;

	mmap_read_lock(mm);

	vma = find_vma(mm, vaddr);
	if (!vma) {
		dev_dbg(dev, "cannot find vaddr: %lx\n", vaddr);
		goto failed;
	}

	if (vaddr < vma->vm_start) {
		dev_dbg(dev, "failed at line %d, vaddr=%lx, vma->vm_start=%lx\n",
			__LINE__, vaddr, vma->vm_start);
		goto failed;
	}

	/* make sure the vma is backed by a dmabuf */
	if (!vma->vm_file) {
		dev_dbg(dev, "failed at line %d\n", __LINE__);
		goto failed;
	}

	dmabuf = vma->vm_file->private_data;
	if (!dmabuf) {
		dev_dbg(dev, "failed at line %d\n", __LINE__);
		goto failed;
	}

	if (dmabuf->file != vma->vm_file) {
		dev_dbg(dev, "failed at line %d\n", __LINE__);
		goto failed;
	}
	mmap_read_unlock(mm);
	mmput(mm);

	mutex_lock(&sess->imp_rb_lock);
	item = vpusmm_find_impbuf(sess, dmabuf);
	mutex_unlock(&sess->imp_rb_lock);

	if (!item) {
		dev_dbg(dev, "failed to find dmabuf in imported list for vaddr=0x%lx (%d)\n",
			vaddr, __LINE__);
		return -EFAULT;
	}

	*arg = (dma_addr_t)(vaddr - vma->vm_start) + item->vpu_addr;
	return 0;

failed:
	if (mm) {
		mmap_read_unlock(mm);
		mmput(mm);
	}
	return -EFAULT;
}

int smm_debugfs_stats_show(struct seq_file *file, struct vpumgr_smm *sess)
{
	struct rb_node *cur, *next;
	struct vpusmm_impbuf *item;
	int i;

	seq_puts(file, "\tdmabuf\texpname\tsize(bytes)\tfilecount\trefs\n");

	mutex_lock(&sess->imp_rb_lock);
	cur = rb_first(&sess->imp_rb);
	i = 0;
	while (cur) {
		item = rb_entry(cur, struct vpusmm_impbuf, node);
		next = rb_next(cur);
		if (item)
			seq_printf(file, "\t%d:%s\t%s\t%zu\t%ld\t%d\n", i++,
				   item->dmabuf->name ? : "",
				   item->dmabuf->exp_name,
				   item->dmabuf->size,
				   file_count(item->dmabuf->file),
				   item->refcount);
		cur = next;
	}
	mutex_unlock(&sess->imp_rb_lock);
	return 0;
}

int smm_init(struct vpumgr_device *vdev)
{
	int rc = 0;

	if (!vdev->dev->of_node) {
		/*
		 * no of_node imply:
		 * 1. no IOMMU, VPU device is only 32-bit DMA capable
		 * 2. use default CMA because no device tree node specifying memory-region
		 */
		dma_set_mask(vdev->dev, DMA_BIT_MASK(32));
		dma_set_coherent_mask(vdev->dev, DMA_BIT_MASK(32));
	} else {
		/* Initialize reserved memory resources */
		rc = of_reserved_mem_device_init(vdev->dev);
		if (rc) {
			if (rc == -ENODEV) {
				dev_warn(vdev->dev,
					 "No reserved memory specified, use default cma\n");
				rc = 0;
			} else {
				dev_err(vdev->dev,
					"Failed to init reserved memory, rc=%d\n", rc);
			}
		}
	}
	return rc;
}

int smm_fini(struct vpumgr_device *vdev)
{
	return 0;
}
