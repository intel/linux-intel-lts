// SPDX-License-Identifier: GPL-2.0
/*
 *    Hantro driver main DRM file.
 *
 *    Copyright (c) 2017 - 2020, VeriSilicon Inc.
 *    Copyright (c) 2020 - 2021, Intel Corporation
 */

#include "hantro_priv.h"
#include "hantro_dec.h"
#include "hantro_enc.h"
#include "hantro_cache.h"
#include "hantro_dec400.h"
#include "hantro_metadata.h"

static struct drm_gem_object_funcs hantro_gem_object_funcs;

static int hantro_record_mem(struct device_info *pdevinfo, void *obj, int size)
{
	int ret;

	mutex_lock(&pdevinfo->alloc_mutex);
	ret = idr_alloc(&pdevinfo->allocations, obj, 1, 0, GFP_KERNEL);
	mutex_unlock(&pdevinfo->alloc_mutex);
	return (ret > 0 ? 0 : -ENOMEM);
}

static void hantro_unrecord_mem(struct device_info *pdevinfo, void *obj)
{
	int id;
	void *cma_obj;

	mutex_lock(&pdevinfo->alloc_mutex);
	idr_for_each_entry(&pdevinfo->allocations, cma_obj, id) {
		if (cma_obj == obj) {
			idr_remove(&pdevinfo->allocations, id);
			break;
		}
	}
	mutex_unlock(&pdevinfo->alloc_mutex);
}

static void hantro_drm_fb_destroy(struct drm_framebuffer *fb)
{
	struct hantro_drm_fb *vsi_fb = (struct hantro_drm_fb *)fb;
	int i;

	for (i = 0; i < 4; i++)
		hantro_unref_drmobj(vsi_fb->obj[i]);

	drm_framebuffer_cleanup(fb);
	kfree(vsi_fb);
}

static int hantro_drm_fb_create_handle(struct drm_framebuffer *fb,
				       struct drm_file *file_priv,
				       unsigned int *handle)
{
	struct hantro_drm_fb *vsi_fb = (struct hantro_drm_fb *)fb;
	int ret;

	ret = drm_gem_handle_create(file_priv, vsi_fb->obj[0], handle);
	trace_gem_handle_create(*handle);
	return ret;
}

static int hantro_drm_fb_dirty(struct drm_framebuffer *fb,
			       struct drm_file *file, unsigned int flags,
			       unsigned int color, struct drm_clip_rect *clips,
			       unsigned int num_clips)
{
	/* nothing to do now */
	return 0;
}

static const struct drm_framebuffer_funcs hantro_drm_fb_funcs = {
	.destroy = hantro_drm_fb_destroy,
	.create_handle = hantro_drm_fb_create_handle,
	.dirty = hantro_drm_fb_dirty,
};

static int hantro_gem_dumb_create_internal(struct drm_file *file_priv,
					   struct drm_device *dev,
					   struct drm_mode_create_dumb *args)
{
	struct drm_gem_hantro_object *cma_obj = NULL;
	struct drm_gem_object *obj;
	struct device_info *pdevinfo;
	int min_pitch = DIV_ROUND_UP(args->width * args->bpp, 8);
	unsigned int deviceidx = (args->flags & 0xf);
	unsigned int region = (args->flags >> 0x8) & 0xf;
	int ret = 0;

	pdevinfo = get_device_info(deviceidx);
	if (!pdevinfo)
		return -EINVAL;

	if (mutex_lock_interruptible(&dev->struct_mutex))
		return -EBUSY;

	cma_obj = kzalloc(sizeof(*cma_obj), GFP_KERNEL);
	if (!cma_obj) {
		ret = -ENOMEM;
		goto out;
	}

	args->handle = 0;
	args->pitch = ALIGN(min_pitch, 64);
	args->size = (__u64)args->pitch * (__u64)args->height;
	args->size = PAGE_ALIGN(args->size);

	obj = &cma_obj->base;
	obj->funcs = &hantro_gem_object_funcs;
	cma_obj->dmapriv.self = cma_obj;
	cma_obj->num_pages = args->size >> PAGE_SHIFT;
	cma_obj->pdevinfo = pdevinfo;

	if (region == CODEC_RESERVED && pdevinfo->codec_rsvmem)
		cma_obj->memdev = pdevinfo->codec_rsvmem;

	if (!cma_obj->memdev) {
		cma_obj->memdev = pdevinfo->dev;
		region = PIXEL_CMA;
	}

	cma_obj->vaddr =
		dma_alloc_coherent(cma_obj->memdev, args->size, &cma_obj->paddr,
				   GFP_KERNEL | GFP_DMA);
	if (region == CODEC_RESERVED && !cma_obj->vaddr) {
		cma_obj->memdev = pdevinfo->dev;
		region = PIXEL_CMA;
		cma_obj->vaddr = dma_alloc_coherent(cma_obj->memdev, args->size,
						    &cma_obj->paddr,
						    GFP_KERNEL | GFP_DMA);
	}

	if (!cma_obj->vaddr) {
		int used_mem[2] = { 0 }, alloc_count[2] = { 0 };

		mem_usage_internal(deviceidx, pdevinfo->dev, &used_mem[0],
				   &alloc_count[0], NULL);
		if (pdevinfo->codec_rsvmem)
			mem_usage_internal(deviceidx, pdevinfo->codec_rsvmem,
					   &used_mem[1], &alloc_count[1], NULL);

		__trace_hantro_err("Device %d out of memory; Requested region = %d;  CMA 0: %dK in %d allocations\n CMA 1: %dK in %d allocations",
				   deviceidx, ((args->flags >> 0x8) & 0xf), used_mem[0] / 1024,
				   alloc_count[0], used_mem[0] / 1024, alloc_count[0]);
		ret = -ENOMEM;
		goto fail_out;
	}

	obj = &cma_obj->base;
	drm_gem_object_init(dev, obj, args->size);
	ret = drm_gem_handle_create(file_priv, obj, &args->handle);
	if (ret) {
		drm_gem_object_put(obj);
		dma_free_coherent(cma_obj->memdev, args->size, cma_obj->vaddr,
				  cma_obj->paddr);
		goto fail_out;
	}

	init_hantro_resv(&cma_obj->kresv, cma_obj);
	cma_obj->handle = args->handle;
	cma_obj->dmapriv.magic_num = HANTRO_IMAGE_VIV_META_DATA_MAGIC;
	cma_obj->fd = -1;
	cma_obj->dmapriv.self = cma_obj;
	cma_obj->file_priv = file_priv;

	hantro_record_mem(pdevinfo, cma_obj, args->size);
	drm_gem_object_put(obj);
	trace_gem_handle_create(args->handle);
	trace_hantro_cma_alloc(deviceidx, region, (void *)cma_obj->paddr, args->handle,
			       args->size);
	goto out;

fail_out:
	kfree(cma_obj);
out:
	mutex_unlock(&dev->struct_mutex);
	return ret;
}

static int hantro_gem_dumb_create(struct drm_device *dev, void *data,
				  struct drm_file *file_priv)
{
	return hantro_gem_dumb_create_internal(file_priv, dev,
					       (struct drm_mode_create_dumb *)data);
}

static int hantro_gem_dumb_map_offset(struct drm_file *file_priv,
				      struct drm_device *dev, u32 handle,
				      uint64_t *offset)
{
	struct drm_gem_object *obj;
	int ret;

	obj = hantro_gem_object_lookup(dev, file_priv, handle);
	if (!obj)
		return -EINVAL;

	ret = drm_gem_create_mmap_offset(obj);
	if (ret == 0)
		*offset = drm_vma_node_offset_addr(&obj->vma_node);

	hantro_unref_drmobj(obj);
	return ret;
}

static int hantro_destroy_dumb(struct drm_device *dev, void *data,
			       struct drm_file *file_priv)
{
	struct drm_mode_destroy_dumb *args = data;
	struct drm_gem_object *obj;

	if (mutex_lock_interruptible(&dev->struct_mutex))
		return -EBUSY;

	obj = hantro_gem_object_lookup(dev, file_priv, args->handle);
	if (!obj) {
		mutex_unlock(&dev->struct_mutex);
		return -EINVAL;
	}

	hantro_unref_drmobj(obj);
	drm_gem_handle_delete(file_priv, args->handle);
	trace_gem_handle_delete(args->handle);
	mutex_unlock(&dev->struct_mutex);
	return 0;
}

static struct sg_table *
hantro_gem_prime_get_sg_table(struct drm_gem_object *obj)
{
	struct drm_gem_hantro_object *cma_obj = to_drm_gem_hantro_obj(obj);
	struct sg_table *sgt;
	int ret;

	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return NULL;

	ret = dma_get_sgtable(cma_obj->memdev, sgt, cma_obj->vaddr,
			      cma_obj->paddr, obj->size);
	if (ret < 0)
		goto out;

	return sgt;
out:
	kfree(sgt);
	return NULL;
}

static struct drm_gem_object *
hantro_gem_prime_import_sg_table(struct drm_device *dev,
				 struct dma_buf_attachment *attach,
				 struct sg_table *sgt)
{
	struct drm_gem_hantro_object *cma_obj;
	struct drm_gem_object *obj;
	struct dma_buf_map map;
	int ret;

	cma_obj = kzalloc(sizeof(*cma_obj), GFP_KERNEL);
	if (!cma_obj)
		return ERR_PTR(-ENOMEM);

	obj = &cma_obj->base;
	obj->funcs = &hantro_gem_object_funcs;
	if (sgt->nents > 1) {
		/* check if the entries in the sg_table are contiguous */
		dma_addr_t next_addr = sg_dma_address(sgt->sgl);
		struct scatterlist *s;
		unsigned int i;

		for_each_sg(sgt->sgl, s, sgt->nents, i) {
			/*
			 * sg_dma_address(s) is only valid for entries
			 * that have sg_dma_len(s) != 0
			 */
			if (!sg_dma_len(s))
				continue;

			if (sg_dma_address(s) != next_addr) {
				kfree(cma_obj);
				return ERR_PTR(-EINVAL);
			}

			next_addr = sg_dma_address(s) + sg_dma_len(s);
		}
	}

	if (drm_gem_object_init(dev, obj, attach->dmabuf->size) != 0) {
		__trace_hantro_err("import sg table failed");
		kfree(cma_obj);
		return ERR_PTR(-ENOMEM);
	}

	cma_obj->paddr = sg_dma_address(sgt->sgl);
	ret = dma_buf_vmap(attach->dmabuf, &map);
	if (ret)
		return ERR_PTR(ret);

	cma_obj->vaddr = map.vaddr;
	cma_obj->sgt = sgt;
	cma_obj->flag |= HANTRO_GEM_FLAG_FOREIGN_IMPORTED;
	cma_obj->num_pages = attach->dmabuf->size >> PAGE_SHIFT;
	cma_obj->dmapriv.meta_data_info =
		*((struct viv_vidmem_metadata *)attach->dmabuf->priv);
	cma_obj->dmapriv.self = cma_obj;
	cma_obj->dmapriv.magic_num = HANTRO_IMAGE_VIV_META_DATA_MAGIC;
	return obj;
}

static int hantro_gem_prime_vmap(struct drm_gem_object *obj, struct dma_buf_map *map)
{
	struct drm_gem_hantro_object *cma_obj = to_drm_gem_hantro_obj(obj);

	dma_buf_map_set_vaddr(map, cma_obj->vaddr);
	return 0;
}

static void hantro_gem_prime_vunmap(struct drm_gem_object *obj, struct dma_buf_map *map)
{
}

static int hantro_gem_prime_mmap(struct drm_gem_object *obj,
				 struct vm_area_struct *vma)
{
	struct drm_gem_hantro_object *cma_obj;
	unsigned long page_num = (vma->vm_end - vma->vm_start) >> PAGE_SHIFT;
	int ret = 0;

	cma_obj = to_drm_gem_hantro_obj(obj);
	if (page_num > cma_obj->num_pages)
		return -EINVAL;

	if ((unsigned long)cma_obj->vaddr == 0)
		return -EINVAL;

	ret = drm_gem_mmap_obj(obj, obj->size, vma);
	if (ret < 0)
		return ret;

	vma->vm_flags &= ~VM_PFNMAP;
	vma->vm_pgoff = 0;
	if (mutex_lock_interruptible(&hantro_drm.drm_dev->struct_mutex))
		return -EBUSY;

	if (dma_mmap_coherent(obj->dev->dev, vma, cma_obj->vaddr,
			      cma_obj->paddr, vma->vm_end - vma->vm_start)) {
		drm_gem_vm_close(vma);
		mutex_unlock(&hantro_drm.drm_dev->struct_mutex);
		return -EAGAIN;
	}

	mutex_unlock(&hantro_drm.drm_dev->struct_mutex);
	vma->vm_private_data = cma_obj;
	return ret;
}

static void hantro_gem_free_object(struct drm_gem_object *gem_obj)
{
	struct drm_gem_hantro_object *cma_obj;
	struct device_info *pdevinfo;
	struct dma_buf_map map;

	/*
	 * dma buf imported from others,
	 * release data structures allocated by ourselves
	 */
	cma_obj = to_drm_gem_hantro_obj(gem_obj);
	if (cma_obj->pages) {
		int i;

		for (i = 0; i < cma_obj->num_pages; i++)
			unref_page(cma_obj->pages[i]);

		kfree(cma_obj->pages);
		cma_obj->pages = NULL;
	}

	map.vaddr = cma_obj->vaddr;
	map.is_iomem = false;

	if (gem_obj->import_attach) {
		if (cma_obj->vaddr)
			dma_buf_vunmap(gem_obj->import_attach->dmabuf,
				       &map);

		drm_prime_gem_destroy(gem_obj, cma_obj->sgt);
	} else if (cma_obj->vaddr) {
		pdevinfo = cma_obj->pdevinfo;
		if (!pdevinfo)
			return;

		dma_free_coherent(cma_obj->memdev, cma_obj->base.size,
				  cma_obj->vaddr, cma_obj->paddr);
		hantro_unrecord_mem(cma_obj->pdevinfo, cma_obj);
		trace_hantro_cma_free(pdevinfo->deviceid, (void *)cma_obj->paddr,
				      cma_obj->handle);
	}

	drm_gem_object_release(gem_obj);
	kfree(cma_obj);
}

static int hantro_gem_close(struct drm_device *dev, void *data,
			    struct drm_file *file_priv)
{
	struct drm_gem_close *args = data;
	struct drm_gem_object *obj =
		hantro_gem_object_lookup(dev, file_priv, args->handle);
	int ret = 0;

	if (!obj)
		return -EINVAL;

	ret = drm_gem_handle_delete(file_priv, args->handle);
	trace_gem_handle_delete(args->handle);
	hantro_unref_drmobj(obj);
	return ret;
}

static int hantro_gem_open(struct drm_device *dev, void *data,
			   struct drm_file *file_priv)
{
	int ret;
	u32 handle;
	struct drm_gem_open *openarg;
	struct drm_gem_object *obj;

	openarg = (struct drm_gem_open *)data;
	obj = idr_find(&dev->object_name_idr, (int)openarg->name);
	if (obj)
		hantro_ref_drmobj(obj);
	else
		return -ENOENT;

	ret = drm_gem_handle_create(file_priv, obj, &handle);
	trace_gem_handle_create(handle);
	hantro_unref_drmobj(obj);
	if (ret)
		return ret;

	openarg->handle = handle;
	openarg->size = obj->size;
	return ret;
}

static int hantro_map_vaddr(struct drm_device *dev, void *data,
			    struct drm_file *file_priv)
{
	struct hantro_addrmap *pamap = data;
	struct drm_gem_object *obj;
	struct drm_gem_hantro_object *cma_obj;

	obj = hantro_gem_object_lookup(dev, file_priv, pamap->handle);
	if (!obj)
		return -EINVAL;

	cma_obj = to_drm_gem_hantro_obj(obj);
	pamap->vm_addr = (unsigned long long)cma_obj->vaddr;
	pamap->phy_addr = cma_obj->paddr;
	hantro_unref_drmobj(obj);
	return 0;
}

static int hantro_get_hwcfg(struct drm_device *dev, void *data,
			    struct drm_file *file_priv)
{
	struct device_info *pdevinfo;
	u32 config = 0;
	int i;

	for (i = 0; i < get_device_count(); i++) {
		pdevinfo = get_device_info(i);
		config |= pdevinfo->config;
	}

	return config;
}

static int hantro_get_devicenum(struct drm_device *dev, void *data,
				struct drm_file *file_priv)
{
	return get_device_count();
}

static int hantro_add_client(struct drm_device *dev, void *data,
			     struct drm_file *file_priv)
{
	int ret;
	struct hantro_client *attrib = data;
	struct file_data *priv_data =
		(struct file_data *)file_priv->driver_priv;
	struct hantro_client *client = NULL;
	struct device_info *pdevinfo;

	if (!data)
		return -EINVAL;

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		return -ENOMEM;

	pdevinfo = get_device_info(attrib->deviceid);
	if (!pdevinfo)
		return -EINVAL;

	client->clientid = attrib->clientid;
	client->deviceid = attrib->deviceid;
	client->codec = attrib->codec;
	client->profile = attrib->profile;
	client->height = attrib->height;
	client->width = attrib->width;
	client->file = file_priv;

	mutex_lock(&pdevinfo->alloc_mutex);
	ret = idr_alloc(&pdevinfo->clients, client, 1, 0, GFP_KERNEL);
	mutex_unlock(&pdevinfo->alloc_mutex);
	trace_client_add((void *)priv_data, attrib->deviceid, attrib->clientid,
			 attrib->codec, attrib->profile, attrib->width,
			 attrib->height);
	return (ret > 0 ? 0 : -ENOMEM);
}

static int hantro_remove_client(struct drm_device *dev, void *data,
				struct drm_file *file_priv)
{
	struct hantro_client *attrib = data;
	struct file_data *priv_data =
		(struct file_data *)file_priv->driver_priv;
	struct hantro_client *client = NULL;
	struct device_info *pdevinfo;
	int id;

	if (!data)
		return -EINVAL;

	pdevinfo = get_device_info(attrib->deviceid);
	if (!pdevinfo)
		return -EINVAL;

	mutex_lock(&pdevinfo->alloc_mutex);
	idr_for_each_entry(&pdevinfo->clients, client, id) {
		if (client && client->clientid == attrib->clientid &&
		    client->file == file_priv) {
			idr_remove(&pdevinfo->clients, id);
			kfree(client);
			break;
		}
	}

	trace_client_remove((void *)priv_data, attrib->clientid);
	mutex_unlock(&pdevinfo->alloc_mutex);
	return 0;
}

static int hantro_gem_flink(struct drm_device *dev, void *data,
			    struct drm_file *file_priv)
{
	struct drm_gem_flink *args = data;
	struct drm_gem_object *obj;
	int ret;

	if (!drm_core_check_feature(dev, DRIVER_GEM))
		return -ENODEV;

	obj = hantro_gem_object_lookup(dev, file_priv, args->handle);
	if (!obj)
		return -ENOENT;

	mutex_lock(&dev->object_name_lock);
	/* prevent races with concurrent gem_close. */
	if (obj->handle_count == 0) {
		ret = -ENOENT;
		goto err;
	}

	if (!obj->name) {
		ret = idr_alloc(&dev->object_name_idr, obj, 1, 0, GFP_KERNEL);
		if (ret < 0)
			goto err;

		obj->name = ret;
	}

	args->name = (uint64_t)obj->name;
	ret = 0;
err:
	mutex_unlock(&dev->object_name_lock);
	hantro_unref_drmobj(obj);
	return ret;
}

static int hantro_map_dumb(struct drm_device *dev, void *data,
			   struct drm_file *file_priv)
{
	int ret;
	struct drm_mode_map_dumb *temparg = (struct drm_mode_map_dumb *)data;

	ret = hantro_gem_dumb_map_offset(file_priv, dev, temparg->handle,
					 &temparg->offset);
	return ret;
}

static int hantro_drm_open(struct drm_device *dev, struct drm_file *file)
{
	trace_drm_file_open((void *)dev, (void *)file);
	return 0;
}

/*
 * This function is used to treat abnormal condition such as Ctrl^c or assert.
 * We can't release memory or drm resources in normal way.
 * In kernel document it's suggested driver_priv
 * be used in this call back. In abnormal situation many kernel data
 * structures might be unavailable, e.g. hantro_gem_object_lookup is not
 * working. So we have to save every gem obj info by ourselves.
 */
static void hantro_drm_postclose(struct drm_device *dev, struct drm_file *file)
{
}

static struct drm_gem_object *
hantro_drm_gem_prime_import(struct drm_device *dev, struct dma_buf *dma_buf)
{
	struct device *attach_dev = dev->dev;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	struct drm_gem_object *obj;
	struct drm_gem_hantro_object *cma_obj;
	int ret;

	if (dma_buf->ops == &hantro_dmabuf_ops) {
		obj = hantro_get_gem_from_dmabuf(dma_buf);
		if (obj && obj->dev == dev) {
			drm_gem_object_get(obj);
			return obj;
		}
	}

	if (!dev->driver->gem_prime_import_sg_table)
		return ERR_PTR(-EINVAL);

	attach = dma_buf_attach(dma_buf, attach_dev);
	if (IS_ERR(attach))
		return ERR_CAST(attach);

	get_dma_buf(dma_buf);

	sgt = dma_buf_map_attachment(attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		goto fail_detach;
	}

	obj = dev->driver->gem_prime_import_sg_table(dev, attach, sgt);
	if (IS_ERR(obj)) {
		ret = PTR_ERR(obj);
		goto fail_unmap;
	}

	cma_obj = to_drm_gem_hantro_obj(obj);
	if (!attach->priv)
		attach->priv = &cma_obj->dmapriv.meta_data_info;
	else
		cma_obj->dmapriv.meta_data_info.magic = 0;

	obj->import_attach = attach;
	obj->resv = dma_buf->resv;
	return obj;

fail_unmap:
	dma_buf_unmap_attachment(attach, sgt, DMA_BIDIRECTIONAL);
fail_detach:
	dma_buf_detach(dma_buf, attach);
	dma_buf_put(dma_buf);

	return ERR_PTR(ret);
}

static struct dma_buf *hantro_prime_export(struct drm_gem_object *obj,
					   int flags)
{
	struct dma_buf *dma_buf;
	struct drm_gem_hantro_object *cma_obj;
	struct drm_device *dev = obj->dev;
	struct dma_buf_export_info exp_info = {
		.exp_name = KBUILD_MODNAME,
		.owner = dev->driver->fops->owner,
		.ops = &hantro_dmabuf_ops,
		.flags = flags,
	};

	cma_obj = to_drm_gem_hantro_obj(obj);
	exp_info.resv = &cma_obj->kresv;
	exp_info.size = cma_obj->num_pages << PAGE_SHIFT;
	exp_info.priv = &cma_obj->dmapriv.meta_data_info;

	dma_buf = dma_buf_export(&exp_info);
	if (IS_ERR(dma_buf))
		return dma_buf;

	drm_dev_get(dev);
	drm_gem_object_get(&cma_obj->base);
	trace_prime_dmabuf_export((void *)cma_obj->paddr, cma_obj->handle,
				  (void *)dma_buf);
	return dma_buf;
}

static int hantro_handle_to_fd(struct drm_device *dev, void *data,
			       struct drm_file *file_priv)
{
	int ret;
	struct drm_prime_handle *primeargs = (struct drm_prime_handle *)data;
	struct drm_gem_object *obj;
	struct drm_gem_hantro_object *cma_obj;

	obj = hantro_gem_object_lookup(dev, file_priv, primeargs->handle);
	if (!obj) {
		__trace_hantro_err("handle not found!! (handle = %-3d, fd = %-3d,)",
				   primeargs->handle, -1);
		return -ENOENT;
	}

	ret = drm_gem_prime_handle_to_fd(dev, file_priv, primeargs->handle,
					 primeargs->flags, &primeargs->fd);

	if (ret == 0) {
		cma_obj = to_drm_gem_hantro_obj(obj);
		cma_obj->flag |= HANTRO_GEM_FLAG_EXPORT;
		cma_obj->fd = primeargs->fd;
	}

	trace_prime_handle_to_fd(obj, primeargs->handle, primeargs->fd, ret);
	hantro_unref_drmobj(obj);
	return ret;
}

static int hantro_fd_to_handle(struct drm_device *dev, void *data,
			       struct drm_file *file_priv)
{
	struct drm_prime_handle *primeargs = (struct drm_prime_handle *)data;
	s32 ret = 0;

	primeargs->flags = 0;
	ret = drm_gem_prime_fd_to_handle(dev, file_priv, primeargs->fd,
					 &primeargs->handle);

	trace_prime_fd_to_handle(primeargs->fd, primeargs->handle, ret);
	return ret;
}

static int hantro_fb_create2(struct drm_device *dev, void *data,
			     struct drm_file *file_priv)
{
	struct drm_mode_fb_cmd2 *mode_cmd = (struct drm_mode_fb_cmd2 *)data;
	struct hantro_drm_fb *vsifb;
	struct drm_gem_object *objs[4];
	struct drm_gem_object *obj;
	const struct drm_format_info *info = drm_get_format_info(dev, mode_cmd);
	unsigned int hsub;
	unsigned int vsub;
	int num_planes;
	int ret;
	int i;

	hsub = info->hsub;
	vsub = info->vsub;
	num_planes = min_t(int, info->num_planes, 4);
	for (i = 0; i < num_planes; i++) {
		unsigned int width = mode_cmd->width / (i ? hsub : 1);
		unsigned int height = mode_cmd->height / (i ? vsub : 1);
		unsigned int min_size;

		obj = hantro_gem_object_lookup(dev, file_priv,
					       mode_cmd->handles[i]);
		if (!obj) {
			ret = -ENXIO;
			goto err_gem_object_unreference;
		}

		hantro_unref_drmobj(obj);
		min_size = (height - 1) * mode_cmd->pitches[i] +
			   mode_cmd->offsets[i] + width * info->cpp[i];

		if (obj->size < min_size) {
			ret = -EINVAL;
			goto err_gem_object_unreference;
		}

		objs[i] = obj;
	}

	vsifb = kzalloc(sizeof(*vsifb), GFP_KERNEL);
	if (!vsifb)
		return -ENOMEM;

	drm_helper_mode_fill_fb_struct(dev, &vsifb->fb, mode_cmd);
	for (i = 0; i < num_planes; i++)
		vsifb->obj[i] = objs[i];

	ret = drm_framebuffer_init(dev, &vsifb->fb, &hantro_drm_fb_funcs);
	if (ret)
		kfree(vsifb);

	return ret;

err_gem_object_unreference:
	return ret;
}

static int hantro_fb_create(struct drm_device *dev, void *data,
			    struct drm_file *file_priv)
{
	struct drm_mode_fb_cmd *or = data;
	struct drm_mode_fb_cmd2 r = {};
	int ret;

	/* convert to new format and call new ioctl */
	r.fb_id = or->fb_id;
	r.width = or->width;
	r.height = or->height;
	r.pitches[0] = or->pitch;
	r.pixel_format = drm_mode_legacy_fb_format(or->bpp, or->depth);
	r.handles[0] = or->handle;

	ret = hantro_fb_create2(dev, &r, file_priv);
	if (ret)
		return ret;

	or->fb_id = r.fb_id;
	return 0;
}

static int hantro_get_version(struct drm_device *dev, void *data,
			      struct drm_file *file_priv)
{
	struct drm_version *pversion;
	char *sname = DRIVER_NAME;
	char *sdesc = DRIVER_DESC;
	char *sdate = DRIVER_DATE;

	pversion = (struct drm_version *)data;
	pversion->version_major = dev->driver->major;
	pversion->version_minor = dev->driver->minor;
	pversion->version_patchlevel = 0;
	pversion->name_len = strlen(DRIVER_NAME);
	pversion->desc_len = strlen(DRIVER_DESC);
	pversion->date_len = strlen(DRIVER_DATE);
	if (pversion->name)
		if (copy_to_user(pversion->name, sname, pversion->name_len))
			return -EFAULT;

	if (pversion->date)
		if (copy_to_user(pversion->date, sdate, pversion->date_len))
			return -EFAULT;

	if (pversion->desc)
		if (copy_to_user(pversion->desc, sdesc, pversion->desc_len))
			return -EFAULT;

	return 0;
}

static int hantro_get_cap(struct drm_device *dev, void *data,
			  struct drm_file *file_priv)
{
	struct drm_get_cap *req = (struct drm_get_cap *)data;

	req->value = 0;
	/* some values should be reset */
	switch (req->capability) {
	case DRM_CAP_PRIME:
		req->value |= dev->driver->prime_fd_to_handle ?
				      DRM_PRIME_CAP_IMPORT :
				      0;
		req->value |= dev->driver->prime_handle_to_fd ?
				      DRM_PRIME_CAP_EXPORT :
				      0;
		return 0;
	case DRM_CAP_DUMB_BUFFER:
		req->value = 1;
		break;
	case DRM_CAP_VBLANK_HIGH_CRTC:
		req->value = 1;
		break;
	case DRM_CAP_DUMB_PREFERRED_DEPTH:
		req->value = dev->mode_config.preferred_depth;
		break;
	case DRM_CAP_DUMB_PREFER_SHADOW:
		req->value = dev->mode_config.prefer_shadow;
		break;
	case DRM_CAP_ASYNC_PAGE_FLIP:
		req->value = dev->mode_config.async_page_flip;
		break;
	case DRM_CAP_CURSOR_WIDTH:
		if (dev->mode_config.cursor_width)
			req->value = dev->mode_config.cursor_width;
		else
			req->value = 64;

		break;
	case DRM_CAP_CURSOR_HEIGHT:
		if (dev->mode_config.cursor_height)
			req->value = dev->mode_config.cursor_height;
		else
			req->value = 64;

		break;
	case DRM_CAP_ADDFB2_MODIFIERS:
		req->value = dev->mode_config.allow_fb_modifiers;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* just a test API for any purpose */
static int hantro_test(struct drm_device *dev, void *data,
		       struct drm_file *file_priv)
{
	unsigned int *input = data;
	int handle = *input;
	struct drm_gem_object *obj;
	struct dma_fence *pfence;
	int ret = 10 * HZ; /* timeout */

	obj = hantro_gem_object_lookup(dev, file_priv, handle);
	if (!obj)
		return -EINVAL;

	pfence = dma_resv_excl_fence(obj->dma_buf->resv);
	while (ret > 0)
		ret = schedule_timeout(ret);

	hantro_fence_signal(pfence);
	hantro_unref_drmobj(obj);
	return 0;
}

static int hantro_getprimeaddr(struct drm_device *dev, void *data,
			       struct drm_file *file_priv)
{
	unsigned long *input = data;
	int fd = *input;
	struct drm_gem_hantro_object *cma_obj;
	struct dma_buf *dma_buf;

	dma_buf = dma_buf_get(fd);
	if (IS_ERR(dma_buf))
		return PTR_ERR(dma_buf);

	cma_obj = (struct drm_gem_hantro_object *)dma_buf->priv;
	*input = cma_obj->paddr;
	dma_buf_put(dma_buf);
	trace_prime_dmabuf_put((void *)cma_obj->paddr, (void *)dma_buf, fd);
	return 0;
}

static int hantro_ptr_to_phys(struct drm_device *dev, void *data,
			      struct drm_file *file_priv)
{
	unsigned long *arg = data;
	struct vm_area_struct *vma;
	struct drm_gem_hantro_object *cma_obj;
	unsigned long vaddr = *arg;

	vma = find_vma(current->mm, vaddr);
	if (!vma)
		return -EFAULT;

	cma_obj = (struct drm_gem_hantro_object *)vma->vm_private_data;
	if (!cma_obj)
		return -EFAULT;

	if (cma_obj->base.dev != dev)
		return -EFAULT;

	if (vaddr < vma->vm_start ||
	    vaddr >= vma->vm_start + (cma_obj->num_pages << PAGE_SHIFT))
		return -EFAULT;

	*arg = (phys_addr_t)(vaddr - vma->vm_start) + cma_obj->paddr;
	return 0;
}

static int hantro_getmagic(struct drm_device *dev, void *data,
			   struct drm_file *file_priv)
{
	struct drm_auth *auth = data;
	int ret = 0;

	mutex_lock(&dev->struct_mutex);
	if (!file_priv->magic) {
		ret = idr_alloc(&file_priv->master->magic_map, file_priv, 1, 0,
				GFP_KERNEL);
		if (ret >= 0)
			file_priv->magic = ret;
	}

	auth->magic = file_priv->magic;
	mutex_unlock(&dev->struct_mutex);
	return ret < 0 ? ret : 0;
}

static int hantro_authmagic(struct drm_device *dev, void *data,
			    struct drm_file *file_priv)
{
	struct drm_auth *auth = data;
	struct drm_file *file;

	mutex_lock(&dev->struct_mutex);
	file = idr_find(&file_priv->master->magic_map, auth->magic);
	if (file) {
		file->authenticated = 1;
		idr_replace(&file_priv->master->magic_map, NULL, auth->magic);
	}

	mutex_unlock(&dev->struct_mutex);
	return file ? 0 : -EINVAL;
}

#define DRM_IOCTL_DEF(ioctl, _func, _flags)                                    \
	[DRM_IOCTL_NR(ioctl)] = {                                              \
		.cmd = ioctl, .func = _func, .flags = _flags, .name = #ioctl   \
	}

/* Ioctl table */
static const struct drm_ioctl_desc hantro_ioctls[] = {
	DRM_IOCTL_DEF(DRM_IOCTL_VERSION, hantro_get_version,
		      DRM_UNLOCKED | DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF(DRM_IOCTL_GET_UNIQUE, drm_invalid_op, DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_GET_MAGIC, hantro_getmagic, DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_IRQ_BUSID, drm_invalid_op,
		      DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_GET_MAP, drm_invalid_op, DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_GET_CLIENT, drm_invalid_op, DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_GET_STATS, drm_invalid_op, DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_GET_CAP, hantro_get_cap,
		      DRM_UNLOCKED | DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF(DRM_IOCTL_SET_CLIENT_CAP, drm_invalid_op, DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_SET_VERSION, drm_invalid_op,
		      DRM_UNLOCKED | DRM_MASTER),

	DRM_IOCTL_DEF(DRM_IOCTL_SET_UNIQUE, drm_invalid_op,
		      DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_BLOCK, drm_invalid_op,
		      DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_UNBLOCK, drm_invalid_op,
		      DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_AUTH_MAGIC, hantro_authmagic,
		      DRM_AUTH | DRM_UNLOCKED | DRM_MASTER),

	DRM_IOCTL_DEF(DRM_IOCTL_ADD_MAP, drm_invalid_op,
		      DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_RM_MAP, drm_invalid_op, DRM_AUTH),

	DRM_IOCTL_DEF(DRM_IOCTL_SET_SAREA_CTX, drm_invalid_op,
		      DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_GET_SAREA_CTX, drm_invalid_op, DRM_AUTH),

	DRM_IOCTL_DEF(DRM_IOCTL_SET_MASTER, drm_invalid_op,
		      DRM_UNLOCKED | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_DROP_MASTER, drm_invalid_op,
		      DRM_UNLOCKED | DRM_ROOT_ONLY),

	DRM_IOCTL_DEF(DRM_IOCTL_ADD_CTX, drm_invalid_op,
		      DRM_AUTH | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_RM_CTX, drm_invalid_op,
		      DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_MOD_CTX, drm_invalid_op,
		      DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_GET_CTX, drm_invalid_op, DRM_AUTH),
	DRM_IOCTL_DEF(DRM_IOCTL_SWITCH_CTX, drm_invalid_op,
		      DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_NEW_CTX, drm_invalid_op,
		      DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_RES_CTX, drm_invalid_op, DRM_AUTH),

	DRM_IOCTL_DEF(DRM_IOCTL_ADD_DRAW, drm_invalid_op,
		      DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_RM_DRAW, drm_invalid_op,
		      DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),

	DRM_IOCTL_DEF(DRM_IOCTL_LOCK, drm_invalid_op, DRM_AUTH),
	DRM_IOCTL_DEF(DRM_IOCTL_UNLOCK, drm_invalid_op, DRM_AUTH),

	DRM_IOCTL_DEF(DRM_IOCTL_FINISH, drm_invalid_op, DRM_AUTH),

	DRM_IOCTL_DEF(DRM_IOCTL_ADD_BUFS, drm_invalid_op,
		      DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_MARK_BUFS, drm_invalid_op,
		      DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_INFO_BUFS, drm_invalid_op, DRM_AUTH),
	DRM_IOCTL_DEF(DRM_IOCTL_MAP_BUFS, drm_invalid_op, DRM_AUTH),
	DRM_IOCTL_DEF(DRM_IOCTL_FREE_BUFS, drm_invalid_op, DRM_AUTH),
	DRM_IOCTL_DEF(DRM_IOCTL_DMA, drm_invalid_op, DRM_AUTH),

	DRM_IOCTL_DEF(DRM_IOCTL_CONTROL, drm_invalid_op,
		      DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),

#if IS_ENABLED(CONFIG_AGP)
	DRM_IOCTL_DEF(DRM_IOCTL_AGP_ACQUIRE, drm_invalid_op,
		      DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_AGP_RELEASE, drm_invalid_op,
		      DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_AGP_ENABLE, drm_invalid_op,
		      DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_AGP_INFO, drm_invalid_op, DRM_AUTH),
	DRM_IOCTL_DEF(DRM_IOCTL_AGP_ALLOC, drm_invalid_op,
		      DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_AGP_FREE, drm_invalid_op,
		      DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_AGP_BIND, drm_invalid_op,
		      DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_AGP_UNBIND, drm_invalid_op,
		      DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
#endif

	DRM_IOCTL_DEF(DRM_IOCTL_SG_ALLOC, drm_invalid_op,
		      DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_SG_FREE, drm_invalid_op,
		      DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),

	DRM_IOCTL_DEF(DRM_IOCTL_WAIT_VBLANK, drm_invalid_op, DRM_UNLOCKED),

	DRM_IOCTL_DEF(DRM_IOCTL_MODESET_CTL, drm_invalid_op, 0),

	DRM_IOCTL_DEF(DRM_IOCTL_UPDATE_DRAW, drm_invalid_op,
		      DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),

	DRM_IOCTL_DEF(DRM_IOCTL_GEM_CLOSE, hantro_gem_close,
		      DRM_UNLOCKED | DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF(DRM_IOCTL_GEM_FLINK, hantro_gem_flink,
		      DRM_AUTH | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_GEM_OPEN, hantro_gem_open,
		      DRM_AUTH | DRM_UNLOCKED),

	DRM_IOCTL_DEF(DRM_IOCTL_MODE_GETRESOURCES, drm_invalid_op,
		      DRM_UNLOCKED),

	DRM_IOCTL_DEF(DRM_IOCTL_PRIME_HANDLE_TO_FD, hantro_handle_to_fd,
		      DRM_AUTH | DRM_UNLOCKED | DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF(DRM_IOCTL_PRIME_FD_TO_HANDLE, hantro_fd_to_handle,
		      DRM_AUTH | DRM_UNLOCKED | DRM_RENDER_ALLOW),

	DRM_IOCTL_DEF(DRM_IOCTL_MODE_GETPLANERESOURCES, drm_invalid_op,
		      DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_GETCRTC, drm_invalid_op,
		      DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_SETCRTC, drm_invalid_op,
		      DRM_MASTER | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_GETPLANE, drm_invalid_op,
		      DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_SETPLANE, drm_invalid_op,
		      DRM_MASTER | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_CURSOR, drm_invalid_op,
		      DRM_MASTER | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_GETGAMMA, drm_invalid_op, DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_SETGAMMA, drm_invalid_op,
		      DRM_MASTER | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_GETENCODER, drm_invalid_op,
		      DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_GETCONNECTOR, drm_invalid_op,
		      DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_ATTACHMODE, drm_invalid_op,
		      DRM_MASTER | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_DETACHMODE, drm_invalid_op,
		      DRM_MASTER | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_GETPROPERTY, drm_invalid_op,
		      DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_SETPROPERTY, drm_invalid_op,
		      DRM_MASTER | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_GETPROPBLOB, drm_invalid_op,
		      DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_GETFB, drm_invalid_op,
		      DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_ADDFB, hantro_fb_create,
		      DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_ADDFB2, hantro_fb_create2,
		      DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_RMFB, drm_invalid_op,
		      DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_PAGE_FLIP, drm_invalid_op,
		      DRM_MASTER | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_DIRTYFB, drm_invalid_op,
		      DRM_MASTER | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_CREATE_DUMB, hantro_gem_dumb_create,
		      DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_MAP_DUMB, hantro_map_dumb,
		      DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_DESTROY_DUMB, hantro_destroy_dumb,
		      DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_OBJ_GETPROPERTIES, drm_invalid_op,
		      DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_OBJ_SETPROPERTY, drm_invalid_op,
		      DRM_MASTER | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_CURSOR2, drm_invalid_op,
		      DRM_MASTER | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_ATOMIC, drm_invalid_op,
		      DRM_MASTER | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_CREATEPROPBLOB, drm_invalid_op,
		      DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_DESTROYPROPBLOB, drm_invalid_op,
		      DRM_UNLOCKED),

	/*hantro specific ioctls*/
	DRM_IOCTL_DEF(DRM_IOCTL_HANTRO_TESTCMD, hantro_test,
		      DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_HANTRO_GETPADDR, hantro_map_vaddr,
		      DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_HANTRO_HWCFG, hantro_get_hwcfg,
		      DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_HANTRO_TESTREADY, hantro_testbufvalid,
		      DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_HANTRO_SETDOMAIN, hantro_setdomain,
		      DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_HANTRO_ACQUIREBUF, hantro_acquirebuf,
		      DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_HANTRO_RELEASEBUF, hantro_releasebuf,
		      DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_HANTRO_GETPRIMEADDR, hantro_getprimeaddr,
		      DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_HANTRO_PTR_PHYADDR, hantro_ptr_to_phys,
		      DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_HANTRO_GET_DEVICENUM, hantro_get_devicenum,
		      DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_HANTRO_ADD_CLIENT, hantro_add_client,
		      DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_HANTRO_REMOVE_CLIENT, hantro_remove_client,
		      DRM_UNLOCKED),
};

#define HANTRO_IOCTL_COUNT ARRAY_SIZE(hantro_ioctls)
static long hantro_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct drm_file *file_priv = file->private_data;
	struct drm_device *dev = hantro_drm.drm_dev;
	const struct drm_ioctl_desc *ioctl = NULL;
	drm_ioctl_t *func;
	unsigned int nr = DRM_IOCTL_NR(cmd);
	int retcode = 0;
	char stack_kdata[256];
	char *kdata = stack_kdata;
	unsigned int in_size, out_size;

	if (drm_dev_is_unplugged(dev))
		return -ENODEV;

	out_size = _IOC_SIZE(cmd);
	in_size = _IOC_SIZE(cmd);
	if (in_size > 0) {
		if (_IOC_DIR(cmd) & _IOC_READ)
			retcode = !hantro_access_ok(VERIFY_WRITE, (void const __user *)arg,
						    in_size);
		else if (_IOC_DIR(cmd) & _IOC_WRITE)
			retcode = !hantro_access_ok(VERIFY_READ, (void const __user *)arg,
						    in_size);
		if (retcode)
			return -EFAULT;
	}
	if (nr >= DRM_IOCTL_NR(HX280ENC_IOC_START) &&
	    nr <= DRM_IOCTL_NR(HX280ENC_IOC_END)) {
		if (enable_encode)
			return hantroenc_ioctl(file, cmd, arg);

		if (cmd == HX280ENC_IOCG_CORE_NUM) {
			int corenum = 0;

			__put_user(corenum, (unsigned int __user *)arg);
		} else {
			return -EFAULT;
		}
	}

	if (nr >= DRM_IOCTL_NR(HANTRODEC_IOC_START) &&
	    nr <= DRM_IOCTL_NR(HANTRODEC_IOC_END)) {
		return hantrodec_ioctl(file, cmd, arg);
	}

	if (nr >= DRM_IOCTL_NR(HANTROCACHE_IOC_START) &&
	    nr <= DRM_IOCTL_NR(HANTROCACHE_IOC_END)) {
		return hantrocache_ioctl(file, cmd, arg);
	}

	if (nr >= DRM_IOCTL_NR(HANTRODEC400_IOC_START) &&
	    nr <= DRM_IOCTL_NR(HANTRODEC400_IOC_END)) {
		return hantrodec400_ioctl(file, cmd, arg);
	}

	if (nr >= DRM_IOCTL_NR(HANTRODEVICE_IOC_START) &&
	    nr <= DRM_IOCTL_NR(HANTRODEVICE_IOC_END)) {
		return hantrodevice_ioctl(file, cmd, arg);
	}

	if (nr >= DRM_IOCTL_NR(HANTROMETADATA_IOC_START) &&
	    nr <= DRM_IOCTL_NR(HANTROMETADATA_IOC_END)) {
		return hantrometadata_ioctl(file, cmd, arg);
	}

	if (nr >= HANTRO_IOCTL_COUNT)
		return -EINVAL;

	ioctl = &hantro_ioctls[nr];

	if (copy_from_user(kdata, (void __user *)arg, in_size) != 0)
		return -EFAULT;

	if (cmd == DRM_IOCTL_MODE_SETCRTC ||
	    cmd == DRM_IOCTL_MODE_GETRESOURCES ||
	    cmd == DRM_IOCTL_SET_CLIENT_CAP || cmd == DRM_IOCTL_MODE_GETCRTC ||
	    cmd == DRM_IOCTL_MODE_GETENCODER ||
	    cmd == DRM_IOCTL_MODE_GETCONNECTOR || cmd == DRM_IOCTL_MODE_GETFB) {
		retcode = drm_ioctl(file, cmd, arg);
		return retcode;
	}

	func = ioctl->func;
	if (!func)
		return -EINVAL;

	retcode = func(dev, kdata, file_priv);

	if (copy_to_user((void __user *)arg, kdata, out_size) != 0)
		retcode = -EFAULT;

	return retcode;
}

static int hantro_device_open(struct inode *inode, struct file *file)
{
	int ret;

	ret = drm_open(inode, file);
	hantrodec_open(inode, file);
	hantrocache_open(inode, file);
	return ret;
}

static int hantro_device_release(struct inode *inode, struct file *file)
{
	hantrodec_release(file);
	hantroenc_release();
	hantrocache_release(file);
	return drm_release(inode, file);
}

static int hantro_mmap(struct file *file, struct vm_area_struct *vma)
{
	int ret = 0;
	struct drm_gem_object *obj = NULL;
	struct drm_gem_hantro_object *cma_obj;
	struct drm_vma_offset_node *node;
	unsigned long page_num = (vma->vm_end - vma->vm_start) >> PAGE_SHIFT;
	unsigned long address = 0;
	struct page **pages = NULL;
	struct device_info *pdevinfo;
	struct device *dev;

	if (mutex_lock_interruptible(&hantro_drm.drm_dev->struct_mutex))
		return -EBUSY;

	drm_vma_offset_lock_lookup(hantro_drm.drm_dev->vma_offset_manager);
	node = drm_vma_offset_exact_lookup_locked(hantro_drm.drm_dev->vma_offset_manager,
						  vma->vm_pgoff, vma_pages(vma));

	if (likely(node)) {
		obj = container_of(node, struct drm_gem_object, vma_node);
		if (!kref_get_unless_zero(&obj->refcount))
			obj = NULL;
	}

	drm_vma_offset_unlock_lookup(hantro_drm.drm_dev->vma_offset_manager);
	if (!obj) {
		mutex_unlock(&hantro_drm.drm_dev->struct_mutex);
		return -EINVAL;
	}

	hantro_unref_drmobj(obj);
	cma_obj = to_drm_gem_hantro_obj(obj);
	if (page_num > cma_obj->num_pages) {
		mutex_unlock(&hantro_drm.drm_dev->struct_mutex);
		return -EINVAL;
	}

	if ((cma_obj->flag & HANTRO_GEM_FLAG_IMPORT) == 0) {
		pdevinfo = cma_obj->pdevinfo;
		if (!pdevinfo) {
			mutex_unlock(&hantro_drm.drm_dev->struct_mutex);
			return -EINVAL;
		}

		dev = cma_obj->memdev;
	} else {
		dev = obj->dev->dev;
	}

	if ((cma_obj->flag & HANTRO_GEM_FLAG_IMPORT) == 0) {
		address = (unsigned long)cma_obj->vaddr;
		if (address == 0) {
			mutex_unlock(&hantro_drm.drm_dev->struct_mutex);
			return -EINVAL;
		}

		ret = drm_gem_mmap_obj(obj,
				       drm_vma_node_size(node) << PAGE_SHIFT, vma);

		if (ret) {
			mutex_unlock(&hantro_drm.drm_dev->struct_mutex);
			return ret;
		}
	} else {
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		/* else mmap report uncached error for some importer, e.g. i915 */
	}

	vma->vm_pgoff = 0;
	if (dma_mmap_coherent(dev, vma, cma_obj->vaddr, cma_obj->paddr,
			      page_num << PAGE_SHIFT)) {
		__trace_hantro_err("unable to map memory; paddr = %p, handle = %d", cma_obj->paddr,
				   cma_obj->handle);
		mutex_unlock(&hantro_drm.drm_dev->struct_mutex);
		return -EAGAIN;
	}

	vma->vm_private_data = cma_obj;
	cma_obj->pages = pages;
	mutex_unlock(&hantro_drm.drm_dev->struct_mutex);
	return ret;
}

/* VFS methods */
static const struct file_operations hantro_fops = {
	.owner = THIS_MODULE,
	.open = hantro_device_open,
	.mmap = hantro_mmap,
	.release = hantro_device_release,
	.poll = drm_poll,
	.read = drm_read,
	.unlocked_ioctl = hantro_ioctl,
	.compat_ioctl = drm_compat_ioctl,
};

static void hantro_gem_vm_close(struct vm_area_struct *vma)
{
	int i;
	struct drm_gem_hantro_object *obj =
		(struct drm_gem_hantro_object *)vma->vm_private_data;
	/* unmap callback */

	if (obj->pages) {
		for (i = 0; i < obj->num_pages; i++)
			unref_page(obj->pages[i]);

		kfree(obj->pages);
		obj->pages = NULL;
	}

	drm_gem_vm_close(vma);
}

static void hantro_release(struct drm_device *dev)
{
}

static int hantro_gem_prime_handle_to_fd(struct drm_device *dev,
					 struct drm_file *file, u32 handle,
					 u32 flags, int *prime_fd)
{
	int ret;

	ret = drm_gem_prime_handle_to_fd(dev, file, handle, flags, prime_fd);
	trace_prime_handle_to_fd(NULL, handle, *prime_fd, ret);
	return ret;
}

static vm_fault_t hantro_vm_fault(struct vm_fault *vmf)
{
	return VM_FAULT_SIGBUS;
}

static const struct vm_operations_struct hantro_drm_gem_cma_vm_ops = {
	.open = drm_gem_vm_open,
	.close = hantro_gem_vm_close,
	.fault = hantro_vm_fault,
};

/* temp no usage now */
static u32 hantro_vblank_no_hw_counter(struct drm_device *dev,
				       unsigned int pipe)
{
	return 0;
}

static struct drm_driver hantro_drm_driver = {
	/* these two are related with controlD and renderD */
	.driver_features = DRIVER_GEM | DRIVER_RENDER,
	.get_vblank_counter = hantro_vblank_no_hw_counter,
	.open = hantro_drm_open,
	.postclose = hantro_drm_postclose,
	.release = hantro_release,
	.dumb_create = hantro_gem_dumb_create_internal,
	.dumb_map_offset = hantro_gem_dumb_map_offset,
	.gem_prime_import = hantro_drm_gem_prime_import,
	.prime_handle_to_fd = hantro_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
	.gem_prime_import_sg_table = hantro_gem_prime_import_sg_table,
	.gem_prime_mmap = hantro_gem_prime_mmap,
	.fops = &hantro_fops,
	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
};

static struct drm_gem_object_funcs hantro_gem_object_funcs = {
	.free = hantro_gem_free_object,
	.export = hantro_prime_export,
	.get_sg_table = hantro_gem_prime_get_sg_table,
	.vmap = hantro_gem_prime_vmap,
	.vunmap = hantro_gem_prime_vunmap,
	.vm_ops = &hantro_drm_gem_cma_vm_ops,
};

struct drm_device *create_hantro_drm(struct device *dev)
{
	struct drm_device *ddev;
	int result;

	ddev = drm_dev_alloc(&hantro_drm_driver, dev);
	if (IS_ERR(ddev))
		return ddev;

	ddev->dev = dev;
	drm_mode_config_init(ddev);
	result = drm_dev_register(ddev, 0);
	if (result < 0) {
		drm_dev_unregister(ddev);
		drm_dev_put(ddev);
		return NULL;
	}

	return ddev;
}
