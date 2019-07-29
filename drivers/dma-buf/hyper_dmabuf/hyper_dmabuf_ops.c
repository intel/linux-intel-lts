/*
 * Copyright © 2017 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * Authors:
 *    Dongwon Kim <dongwon.kim@intel.com>
 *    Mateusz Polrola <mateuszx.potrola@intel.com>
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/dma-buf.h>
#include <linux/version.h>
#include "hyper_dmabuf_drv.h"
#include "hyper_dmabuf_struct.h"
#include "hyper_dmabuf_ops.h"
#include "hyper_dmabuf_sgl_proc.h"
#include "hyper_dmabuf_id.h"
#include "hyper_dmabuf_msg.h"
#include "hyper_dmabuf_list.h"

#define WAIT_AFTER_SYNC_REQ 0
#define REFS_PER_PAGE (PAGE_SIZE/sizeof(grant_ref_t))
#define NO_DMA_BUF_SYNC

int dmabuf_refcount(struct dma_buf *dma_buf)
{
	if (dma_buf->file != NULL)
		return file_count(dma_buf->file);
	pr_err("dma_buf->file is NULL\n");
	return -EINVAL;
}

static int sync_request(hyper_dmabuf_id_t hid, int dmabuf_ops, int wait)
{
	struct hyper_dmabuf_req *req;
	struct hyper_dmabuf_bknd_ops *bknd_ops = hy_drv_priv->bknd_ops;
	int op[5];
	int i;
	int ret;

	op[0] = hid.id;

	for (i = 0; i < 3; i++)
		op[i+1] = hid.rng_key[i];

	op[4] = dmabuf_ops;

	req = kcalloc(1, sizeof(*req), GFP_KERNEL);

	if (!req)
		return -ENOMEM;

	hyper_dmabuf_create_req(req, HYPER_DMABUF_OPS_TO_SOURCE, &op[0]);

	/* send request and wait for a response */
	ret = bknd_ops->send_req(HYPER_DMABUF_DOM_ID(hid), req,
				 wait);

	if (ret < 0) {
		dev_dbg(hy_drv_priv->dev,
			"dmabuf sync request failed:%d\n", req->op[4]);
	}

	kfree(req);

	return ret;
}

static int hyper_dmabuf_ops_attach(struct dma_buf *dmabuf,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 17, 0)
				   struct device *dev,
#endif
				   struct dma_buf_attachment *attach)
{
	struct imported_sgt_info *imported;
	int ret = 0;

	if (!attach->dmabuf->priv)
		return -EINVAL;

	imported = (struct imported_sgt_info *)attach->dmabuf->priv;

#ifndef NO_DMA_BUF_SYNC
	ret = sync_request(imported->hid, HYPER_DMABUF_OPS_ATTACH, WAIT_AFTER_SYNC_REQ);
#endif
	return ret;
}

static void hyper_dmabuf_ops_detach(struct dma_buf *dmabuf,
				    struct dma_buf_attachment *attach)
{
	struct imported_sgt_info *imported;

	if (!attach->dmabuf->priv)
		return;

	imported = (struct imported_sgt_info *)attach->dmabuf->priv;

#ifndef NO_DMA_BUF_SYNC
	sync_request(imported->hid, HYPER_DMABUF_OPS_DETACH, WAIT_AFTER_SYNC_REQ);
#endif
}

static struct sg_table *hyper_dmabuf_ops_map(
				struct dma_buf_attachment *attachment,
				enum dma_data_direction dir)
{
	struct sg_table *st;
	struct imported_sgt_info *imported;
	struct pages_info *pg_info;

	if (!attachment->dmabuf->priv)
		return NULL;

	imported = (struct imported_sgt_info *)attachment->dmabuf->priv;

	if (!imported) {
		dev_err(hy_drv_priv->dev, "%s: imported is NULL\n", __func__);
		return NULL;
	}

	/* extract pages from sgt */
	pg_info = hyper_dmabuf_ext_pgs(imported->sgt);

	if (!pg_info) {
		dev_err(hy_drv_priv->dev,
			"%s: failed to extract pages\n", __func__);
		return NULL;
	}

	/* create a new sg_table with extracted pages */
	st = hyper_dmabuf_create_sgt(pg_info->pgs, pg_info->frst_ofst,
				     pg_info->last_len, pg_info->nents);
	if (!st)
		goto err_free_sg;

	if (!dma_map_sg(attachment->dev, st->sgl, st->nents, dir))
		goto err_free_sg;

#ifndef NO_DMA_BUF_SYNC
	sync_request(imported->hid, HYPER_DMABUF_OPS_MAP, WAIT_AFTER_SYNC_REQ);
#endif
	kfree(pg_info->pgs);
	kfree(pg_info);

	return st;

err_free_sg:
	if (st) {
		sg_free_table(st);
		kfree(st);
	}

	kfree(pg_info->pgs);
	kfree(pg_info);

	dev_err(hy_drv_priv->dev,
		"%s: failed to create dma_buf with sgt\n", __func__);

	return NULL;
}

static void hyper_dmabuf_ops_unmap(struct dma_buf_attachment *attachment,
				   struct sg_table *sg,
				   enum dma_data_direction dir)
{
	struct imported_sgt_info *imported;

	if (!attachment->dmabuf->priv)
		return;

	imported = (struct imported_sgt_info *)attachment->dmabuf->priv;

	dma_unmap_sg(attachment->dev, sg->sgl, sg->nents, dir);

	sg_free_table(sg);
	kfree(sg);

#ifndef NO_DMA_BUF_SYNC
	sync_request(imported->hid, HYPER_DMABUF_OPS_UNMAP, WAIT_AFTER_SYNC_REQ);
#endif
}

static void hyper_dmabuf_ops_release(struct dma_buf *dma_buf)
{
	struct imported_sgt_info *imported;
	struct hyper_dmabuf_bknd_ops *bknd_ops = hy_drv_priv->bknd_ops;
	hyper_dmabuf_id_t hid;
	int finish;

	if (!dma_buf->priv)
		return;

	mutex_lock(&hy_drv_priv->lock);

	imported = (struct imported_sgt_info *)dma_buf->priv;

	/* check if imported still exists */
	hid = hyper_dmabuf_find_hid_imported(imported);
	if (hid.id == -1) {
		mutex_unlock(&hy_drv_priv->lock);
		return;
	}

	imported->dma_buf = NULL;
	imported->importers--;

	if (imported->importers == 0) {
		bknd_ops->unmap_shared_pages(&imported->refs_info,
					     imported->nents);

		if (imported->sgt) {
			sg_free_table(imported->sgt);
			kfree(imported->sgt);
			imported->sgt = NULL;
		}
	}

	finish = imported && !imported->valid &&
		 !imported->importers;


	dev_dbg(hy_drv_priv->dev, "%s   finished:%d ref_c:%d valid:%c\n",
			__func__, finish, imported->importers,
			imported->valid ? 'Y':'N');

	/* release operation should be synchronized with exporter. */
	sync_request(imported->hid, HYPER_DMABUF_OPS_RELEASE, true);

	/*
	 * Check if buffer is still valid and if not remove it
	 * from imported list. That has to be done after sending
	 * sync request
	 */
	if (finish) {
		hyper_dmabuf_remove_imported(imported->hid);
		kfree(imported->priv);
		kfree(imported);
	}

	mutex_unlock(&hy_drv_priv->lock);
}

static int hyper_dmabuf_ops_begin_cpu_access(struct dma_buf *dmabuf,
					     enum dma_data_direction dir)
{
	struct imported_sgt_info *imported;
	int ret = 0;

	if (!dmabuf->priv)
		return -EINVAL;

	imported = (struct imported_sgt_info *)dmabuf->priv;

#ifndef NO_DMA_BUF_SYNC
	ret = sync_request(imported->hid, HYPER_DMABUF_OPS_BEGIN_CPU_ACCESS, WAIT_AFTER_SYNC_REQ);
#endif
	return ret;
}

static int hyper_dmabuf_ops_end_cpu_access(struct dma_buf *dmabuf,
					   enum dma_data_direction dir)
{
	struct imported_sgt_info *imported;
	int ret = 0;

	if (!dmabuf->priv)
		return -EINVAL;

	imported = (struct imported_sgt_info *)dmabuf->priv;

#ifndef NO_DMA_BUF_SYNC
	ret = sync_request(imported->hid, HYPER_DMABUF_OPS_END_CPU_ACCESS, WAIT_AFTER_SYNC_REQ);
#endif
	return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 17, 0)
static void *hyper_dmabuf_ops_kmap_atomic(struct dma_buf *dmabuf,
					  unsigned long pgnum)
{
	struct imported_sgt_info *imported;

	if (!dmabuf->priv)
		return NULL;

	imported = (struct imported_sgt_info *)dmabuf->priv;

#ifndef NO_DMA_BUF_SYNC
	sync_request(imported->hid, HYPER_DMABUF_OPS_KMAP_ATOMIC);
#endif
	/* TODO: NULL for now. Need to return the addr of mapped region */
	return NULL;
}

static void hyper_dmabuf_ops_kunmap_atomic(struct dma_buf *dmabuf,
					   unsigned long pgnum, void *vaddr)
{
	struct imported_sgt_info *imported;

	if (!dmabuf->priv)
		return;

	imported = (struct imported_sgt_info *)dmabuf->priv;

#ifndef NO_DMA_BUF_SYNC
	sync_request(imported->hid, HYPER_DMABUF_OPS_KUNMAP_ATOMIC, WAIT_AFTER_SYNC_REQ);
#endif
}
#endif

static void *hyper_dmabuf_ops_kmap(struct dma_buf *dmabuf, unsigned long pgnum)
{
	struct imported_sgt_info *imported;

	if (!dmabuf->priv)
		return NULL;

	imported = (struct imported_sgt_info *)dmabuf->priv;

#ifndef NO_DMA_BUF_SYNC
	sync_request(imported->hid, HYPER_DMABUF_OPS_KMAP, WAIT_AFTER_SYNC_REQ);
#endif
	/* for now NULL.. need to return the address of mapped region */
	return NULL;
}

static void hyper_dmabuf_ops_kunmap(struct dma_buf *dmabuf, unsigned long pgnum,
				    void *vaddr)
{
	struct imported_sgt_info *imported;

	if (!dmabuf->priv)
		return;

	imported = (struct imported_sgt_info *)dmabuf->priv;

#ifndef NO_DMA_BUF_SYNC
	sync_request(imported->hid, HYPER_DMABUF_OPS_KUNMAP, WAIT_AFTER_SYNC_REQ);
#endif
}

static int hyper_dmabuf_ops_mmap(struct dma_buf *dmabuf,
				 struct vm_area_struct *vma)
{
	struct imported_sgt_info *imported;
	int ret = 0;

	if (!dmabuf->priv)
		return -EINVAL;

	imported = (struct imported_sgt_info *)dmabuf->priv;

#ifndef NO_DMA_BUF_SYNC
	ret = sync_request(imported->hid, HYPER_DMABUF_OPS_MMAP, WAIT_AFTER_SYNC_REQ);
#endif
	return ret;
}

static void *hyper_dmabuf_ops_vmap(struct dma_buf *dmabuf)
{
	struct imported_sgt_info *imported;

	if (!dmabuf->priv)
		return NULL;

	imported = (struct imported_sgt_info *)dmabuf->priv;

#ifndef NO_DMA_BUF_SYNC
	sync_request(imported->hid, HYPER_DMABUF_OPS_VMAP, WAIT_AFTER_SYNC_REQ);
#endif
	return NULL;
}

static void hyper_dmabuf_ops_vunmap(struct dma_buf *dmabuf, void *vaddr)
{
	struct imported_sgt_info *imported;

	if (!dmabuf->priv)
		return;

	imported = (struct imported_sgt_info *)dmabuf->priv;

#ifndef NO_DMA_BUF_SYNC
	sync_request(imported->hid, HYPER_DMABUF_OPS_VUNMAP, WAIT_AFTER_SYNC_REQ);
#endif
}

static const struct dma_buf_ops hyper_dmabuf_ops = {
	.attach = hyper_dmabuf_ops_attach,
	.detach = hyper_dmabuf_ops_detach,
	.map_dma_buf = hyper_dmabuf_ops_map,
	.unmap_dma_buf = hyper_dmabuf_ops_unmap,
	.release = hyper_dmabuf_ops_release,
	.begin_cpu_access = hyper_dmabuf_ops_begin_cpu_access,
	.end_cpu_access = hyper_dmabuf_ops_end_cpu_access,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 17, 0)
	.map_atomic = hyper_dmabuf_ops_kmap_atomic,
	.unmap_atomic = hyper_dmabuf_ops_kunmap_atomic,
#endif
	.map = hyper_dmabuf_ops_kmap,
	.unmap = hyper_dmabuf_ops_kunmap,
	.mmap = hyper_dmabuf_ops_mmap,
	.vmap = hyper_dmabuf_ops_vmap,
	.vunmap = hyper_dmabuf_ops_vunmap,
};

/* exporting dmabuf as fd */
int hyper_dmabuf_export_fd(struct imported_sgt_info *imported, int flags)
{
	int fd = -1;

	/* call hyper_dmabuf_export_dmabuf and create
	 * and bind a handle for it then release
	 */
	hyper_dmabuf_export_dma_buf(imported);

	if (!IS_ERR_OR_NULL(imported->dma_buf)) {
		fd = dma_buf_fd(imported->dma_buf, flags);
	} else {
		imported->dma_buf = NULL;
		dev_err(hy_drv_priv->dev,
				"failed to get dma_buf,return -1\n");
	}

	return fd;
}

void hyper_dmabuf_export_dma_buf(struct imported_sgt_info *imported)
{
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);

	exp_info.ops = &hyper_dmabuf_ops;

	/* multiple of PAGE_SIZE, not considering offset */
	exp_info.size = imported->sgt->nents * PAGE_SIZE;
	exp_info.flags = /* not sure about flag */ 0;
	exp_info.priv = imported;

	imported->dma_buf = dma_buf_export(&exp_info);
}
