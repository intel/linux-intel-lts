// SPDX-License-Identifier: GPL-2.0
/*
 *    Hantro driver DMA_BUF operation.
 *
 *    Copyright (c) 2017 - 2020, VeriSilicon Inc.
 *    Copyright (c) 2020 - 2021, Intel Corporation
 */

#include "hantro_priv.h"

static void hantro_gem_dmabuf_release(struct dma_buf *dma_buf)
{
	struct drm_gem_object *obj = hantro_get_gem_from_dmabuf(dma_buf);

	if (obj) {
		drm_gem_object_put(obj);
		drm_dev_put(obj->dev);
	}
}

static struct sg_table *
hantro_gem_map_dma_buf(struct dma_buf_attachment *attach,
		       enum dma_data_direction dir)
{
	struct drm_gem_object *obj = hantro_get_gem_from_dmabuf(attach->dmabuf);
	struct sg_table *sgt;

	if (WARN_ON(dir == DMA_NONE))
		return ERR_PTR(-EINVAL);

	if (WARN_ON(!obj))
		return ERR_PTR(-EINVAL);

	sgt = obj->funcs->get_sg_table(obj);
	if (!dma_map_sg_attrs(attach->dev, sgt->sgl, sgt->nents, dir,
			      DMA_ATTR_SKIP_CPU_SYNC)) {
		sg_free_table(sgt);
		kfree(sgt);
		sgt = ERR_PTR(-ENOMEM);
	}

	return sgt;
}

static int hantro_gem_dmabuf_mmap(struct dma_buf *dma_buf,
				  struct vm_area_struct *vma)
{
	struct drm_gem_object *obj = hantro_get_gem_from_dmabuf(dma_buf);
	struct drm_device *dev;

	if (!obj)
		return -EINVAL;

	dev = obj->dev;
	if (!dev->driver->gem_prime_mmap)
		return -EOPNOTSUPP;

	return dev->driver->gem_prime_mmap(obj, vma);
}

static int hantro_gem_dmabuf_vmap(struct dma_buf *dma_buf, struct dma_buf_map *map)
{
	struct drm_gem_object *obj = hantro_get_gem_from_dmabuf(dma_buf);

	return obj->funcs->vmap(obj, map);
}

static void hantro_gem_dmabuf_vunmap(struct dma_buf *dma_buf, struct dma_buf_map *vaddr)
{
	struct drm_gem_object *obj = hantro_get_gem_from_dmabuf(dma_buf);

	if (!vaddr)
		return;

	if (obj) {
		if (obj->funcs && obj->funcs->vunmap)
			obj->funcs->vunmap(obj, vaddr);
		else if (obj->funcs->vunmap)
			obj->funcs->vunmap(obj, vaddr);
	}
}

const struct dma_buf_ops hantro_dmabuf_ops = {
	.cache_sgt_mapping = true,
	.map_dma_buf = hantro_gem_map_dma_buf,
	.unmap_dma_buf = drm_gem_unmap_dma_buf,
	.release = hantro_gem_dmabuf_release,
	.mmap = hantro_gem_dmabuf_mmap,
	.vmap = hantro_gem_dmabuf_vmap,
	.vunmap = hantro_gem_dmabuf_vunmap,
};
