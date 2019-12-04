/*
 *    Hantro driver main entrance.
 *
 *    Copyright (c) 2017, VeriSilicon Inc.
 *
 *    This program is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU General Public License
 *    as published by the Free Software Foundation; either version 2
 *    of the License, or (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You may obtain a copy of the GNU General Public License
 *    Version 2 or later at the following locations:
 *    http://www.opensource.org/licenses/gpl-license.html
 *    http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/io.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/shmem_fs.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/dma-contiguous.h>
#include <drm/drm_modeset_helper.h>
#ifdef __amd64__
#include <asm/set_memory.h>
#endif
/* Our header */
#include "hantro_priv.h"
#include "hx280enc.h"
#include "hantrodec.h"
#include "hantrocache.h"

/* compile options */
#define USE_HW 1
#define USE_CMA 0
#define HAS_VC8000E
#define HAS_VC8000D
//#define HAS_CACHECORE

/* debug */
#define ENABLE_DEBUG
#ifdef ENABLE_DEBUG
#define DBG(...) pr_info(__VA_ARGS__)
#else
#define DBG(...)
#endif

#define DRIVER_NAME      "hantro"
#define DRIVER_DESC      "hantro DRM"
#define DRIVER_DATE      "20171114"
#define DRIVER_MAJOR      1
#define DRIVER_MINOR      0

struct hantro_device_handle hantro_dev;

#if KERNEL_VERSION(4, 13, 0) > LINUX_VERSION_CODE
void debug_dma_alloc_coherent(
	struct device *dev,
	size_t size,
	dma_addr_t dma_addr,
	void *virt)
{
}
void debug_dma_free_coherent(
	struct device *dev,
	size_t size,
	void *virt,
	dma_addr_t addr)
{
}
#endif
/*temp no usage now*/
static u32 hantro_vblank_no_hw_counter(
	struct drm_device *dev,
	unsigned int pipe)
{
	return 0;
}

static int hantro_recordmem(
	struct drm_file *priv,
	void *obj,
	int size)
{
	int ret;
	struct idr *list = (struct idr *)priv->driver_priv;

	ret = idr_alloc(list, obj, 1, 0, GFP_KERNEL);
	return (ret > 0 ? 0 : -ENOMEM);
}

static void hantro_unrecordmem(
	struct drm_file *priv,
	void *obj)
{
	int id;
	struct idr *list = (struct idr *)priv->driver_priv;
	void *gemobj;

	idr_for_each_entry(list, gemobj, id) {
		if (gemobj == obj) {
			idr_remove(list, id);
			break;
		}
	}
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

static int hantro_drm_fb_create_handle(
	struct drm_framebuffer *fb,
	struct drm_file *file_priv,
	unsigned int *handle)
{
	struct hantro_drm_fb *vsi_fb = (struct hantro_drm_fb *)fb;

	return drm_gem_handle_create(file_priv, vsi_fb->obj[0], handle);
}

static int hantro_drm_fb_dirty(
	struct drm_framebuffer *fb,
	struct drm_file *file,
	unsigned int flags, unsigned int color,
	struct drm_clip_rect *clips,
	unsigned int num_clips)
{
	/*nothing to do now*/
	return 0;
}

static const struct drm_framebuffer_funcs hantro_drm_fb_funcs = {
	.destroy      = hantro_drm_fb_destroy,
	.create_handle      = hantro_drm_fb_create_handle,
	.dirty            = hantro_drm_fb_dirty,
};

static int hantro_gem_dumb_create_internal(
	struct drm_file *file_priv,
	struct drm_device *dev,
	struct drm_mode_create_dumb *args)
{
	int ret = 0;
	int in_size, out_size;
	struct drm_gem_hantro_object *cma_obj;
	int min_pitch = DIV_ROUND_UP(args->width * args->bpp, 8);
	struct drm_gem_object *obj;
	//unsigned int sliceidx = args->handle;

	args->handle = 0;
	if (mutex_lock_interruptible(&dev->struct_mutex))
		return -EBUSY;

	cma_obj = kzalloc(
		sizeof(struct drm_gem_hantro_object), GFP_KERNEL);
	if (!cma_obj) {
		ret = -ENOMEM;
		goto out;
	}

	obj = &cma_obj->base;
	out_size =  in_size = sizeof(*args);
	args->pitch = ALIGN(min_pitch, 64);
	args->size = (__u64)args->pitch * (__u64)args->height;
	args->size = (args->size + PAGE_SIZE - 1) / PAGE_SIZE * PAGE_SIZE;

	cma_obj->num_pages = args->size >> PAGE_SHIFT;
	cma_obj->flag = 0;
	cma_obj->pageaddr      = NULL;
	cma_obj->pages      = NULL;
	cma_obj->vaddr = NULL;

/*CMA is temp disabled here for these reasons:
 *1. to bring up CMA, CONFIG_DMA_CMA shoule be enabled in kernel building
 *2. dma_alloc/release_from_contiguous should be exported in kernel code
 *3. a boot parameter like "cma=268435456@134217728" should be added
 *4. CMA's memory management is not stable.
 *We could choose to use CMA or page alloc by module parameter later.
 */
#if USE_CMA
#if KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE
	cma_obj->pageaddr =
		dma_alloc_from_contiguous(
			dev->dev,
			args->size >> PAGE_SHIFT,
			1);
#else
	cma_obj->pageaddr =
		dma_alloc_from_contiguous(
			dev->dev,
			args->size >> PAGE_SHIFT,
			1,
			GFP_KERNEL);
#endif
	if (cma_obj->pageaddr == NULL) {
		kfree(cma_obj);
		ret = -ENOMEM;
		goto out;
	}
	cma_obj->vaddr = page_to_virt(cma_obj->pageaddr);
	cma_obj->paddr = virt_to_phys(cma_obj->vaddr);
#else
	cma_obj->vaddr =
		dma_alloc_coherent(
			dev->dev,
			args->size,
			&cma_obj->paddr,
			GFP_KERNEL | GFP_DMA);
	if (cma_obj->vaddr == NULL) {
		kfree(cma_obj);
		ret = -ENOMEM;
		goto out;
	}
#endif
	drm_gem_object_init(dev, obj, args->size);

	ret = drm_gem_handle_create(file_priv, obj, &args->handle);
	if (ret == 0)
		ret = hantro_recordmem(file_priv, cma_obj, args->size);
	if (ret) {
#if USE_CMA
		dma_release_from_contiguous(
			dev->dev,
			cma_obj->pageaddr,
			cma_obj->num_pages);
#else
		dma_free_coherent(
			obj->dev->dev,
			args->size,
			cma_obj->vaddr,
			cma_obj->paddr);
#endif
		kfree(cma_obj);
	}
	init_hantro_resv(&cma_obj->kresv, cma_obj);
	cma_obj->handle = args->handle;
out:
	mutex_unlock(&dev->struct_mutex);
	return ret;
}

static int hantro_gem_dumb_create(
	struct drm_device *dev,
	void *data,
	struct drm_file *file_priv)
{
	return hantro_gem_dumb_create_internal(
			file_priv,
			dev,
			(struct drm_mode_create_dumb *)data);
}

static int hantro_gem_dumb_map_offset(
	struct drm_file *file_priv,
	struct drm_device *dev,
	uint32_t handle,
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

static int hantro_destroy_dumb(
	struct drm_device *dev,
	void *data,
	struct drm_file *file_priv)
{
	struct drm_mode_destroy_dumb *args = data;
	struct drm_gem_object *obj;
	struct drm_gem_hantro_object *cma_obj;

	if (mutex_lock_interruptible(&dev->struct_mutex))
		return -EBUSY;
	obj = hantro_gem_object_lookup(dev, file_priv, args->handle);
	if (!obj) {
		mutex_unlock(&dev->struct_mutex);
		return -EINVAL;
	}
	hantro_unref_drmobj(obj);

	cma_obj = to_drm_gem_hantro_obj(obj);
	if ((cma_obj->flag & HANTRO_GEM_FLAG_IMPORT) == 0)
		hantro_unrecordmem(file_priv, cma_obj);

	drm_gem_handle_delete(file_priv, args->handle);
	hantro_unref_drmobj(obj);
	mutex_unlock(&dev->struct_mutex);
	return 0;
}

static int hantro_release_dumb(
	struct drm_device *dev,
	struct drm_file *file_priv,
	void *obj)
{
	struct drm_gem_object *gemobj = obj;
	struct drm_gem_hantro_object *cma_obj;

	cma_obj = to_drm_gem_hantro_obj(gemobj);

	drm_gem_free_mmap_offset(&cma_obj->base);
	dma_resv_fini(&cma_obj->kresv);

	if (cma_obj->flag & HANTRO_GEM_FLAG_EXPORT) {
		drm_gem_handle_delete(file_priv, cma_obj->handle);
		hantro_unref_drmobj(obj);
		return 0;
	}

	drm_gem_object_release(gemobj);
	drm_gem_handle_delete(file_priv, cma_obj->handle);

#if USE_CMA
	if (cma_obj->pageaddr)
		dma_release_from_contiguous(
			dev->dev,
			cma_obj->pageaddr,
			cma_obj->num_pages);
#else
	if (cma_obj->vaddr)
		dma_free_coherent(
			gemobj->dev->dev,
			cma_obj->base.size,
			cma_obj->vaddr,
			cma_obj->paddr);
#endif
	kfree(cma_obj);

	return 0;
}

static int hantro_mmap(
	struct file *filp,
	struct vm_area_struct *vma)
{
	int ret = 0;
	struct drm_gem_object *obj = NULL;
	struct drm_gem_hantro_object *cma_obj;
	struct drm_vma_offset_node *node;
	unsigned long page_num = (vma->vm_end - vma->vm_start) >> PAGE_SHIFT;
	unsigned long address = 0;
	int sgtidx = 0;
	struct scatterlist *pscatter = NULL;
	struct page **pages = NULL;

	if (mutex_lock_interruptible(&hantro_dev.drm_dev->struct_mutex))
		return -EBUSY;
	drm_vma_offset_lock_lookup(hantro_dev.drm_dev->vma_offset_manager);
	node = drm_vma_offset_exact_lookup_locked(hantro_dev.drm_dev->vma_offset_manager,
		vma->vm_pgoff,
		vma_pages(vma));

	if (likely(node)) {
		obj = container_of(node, struct drm_gem_object, vma_node);
		if (!kref_get_unless_zero(&obj->refcount))
			obj = NULL;
	}
	drm_vma_offset_unlock_lookup(hantro_dev.drm_dev->vma_offset_manager);
	hantro_unref_drmobj(obj);

	if (!obj) {
		mutex_unlock(&hantro_dev.drm_dev->struct_mutex);
		return -EINVAL;
	}
	cma_obj = to_drm_gem_hantro_obj(obj);

	if (page_num > cma_obj->num_pages) {
		mutex_unlock(&hantro_dev.drm_dev->struct_mutex);
		return -EINVAL;
	}

	if ((cma_obj->flag & HANTRO_GEM_FLAG_IMPORT) == 0) {
		address = (unsigned long)cma_obj->vaddr;
		if (address == 0) {
			mutex_unlock(&hantro_dev.drm_dev->struct_mutex);
			return -EINVAL;
		}
		ret = drm_gem_mmap_obj(
				obj,
				drm_vma_node_size(node) << PAGE_SHIFT,
				vma);

		if (ret) {
			mutex_unlock(&hantro_dev.drm_dev->struct_mutex);
			return ret;
		}
	} else {
		pscatter = &cma_obj->sgt->sgl[sgtidx];
#ifdef __amd64__
		set_memory_uc((unsigned long)cma_obj->vaddr, (int)page_num);
#endif
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		/*else mmap report uncached error for some importer, e.g. i915*/
	}

	vma->vm_pgoff = 0;
	if (dma_mmap_coherent(
		hantro_dev.drm_dev->dev,
		vma,
		cma_obj->vaddr,
		cma_obj->paddr,
		page_num << PAGE_SHIFT)) {
		mutex_unlock(&hantro_dev.drm_dev->struct_mutex);
		return -EAGAIN;
	}

	vma->vm_private_data = cma_obj;
	cma_obj->pages = pages;
	mutex_unlock(&hantro_dev.drm_dev->struct_mutex);
	return ret;
}

static int hantro_gem_open_obj(
	struct drm_gem_object *obj,
	struct drm_file *filp)
{
	return 0;
}

static int hantro_device_open(
	struct inode *inode,
	struct file *filp)
{
	int ret;

	ret = drm_open(inode, filp);
#if KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE
	if (ret == 0) {
		struct drm_file *file_priv = filp->private_data;

		hantro_dev.drm_dev->dev = file_priv->minor->dev->dev;
	}
#endif
#if USE_HW == 1      /*hw init*/
#ifdef HAS_VC8000D
	hantrodec_open(inode, filp);
#endif
#ifdef HAS_CACHECORE
	cache_open(inode, filp);
#endif
#endif
	return ret;
}

static int hantro_device_release(struct inode *inode, struct file *filp)
{
#if USE_HW == 1
#ifdef HAS_CACHECORE
	cache_release();
#endif
#ifdef HAS_VC8000E
	hantroenc_release();
#endif
#endif
	return drm_release(inode, filp);
}

#if KERNEL_VERSION(4, 13, 0) > LINUX_VERSION_CODE
/*we shall not support page fault. */
static int hantro_vm_fault(
	struct vm_area_struct *vma,
	struct vm_fault *vmf)
{
	return -EPERM;
}
#elif KERNEL_VERSION(5, 0, 0) > LINUX_VERSION_CODE
static int hantro_vm_fault(struct vm_fault *vmf)
{
	return -EPERM;
}
#else
static vm_fault_t hantro_vm_fault(struct vm_fault *vmf)
{
	return -EPERM;
}
#endif
/* conflicting with def in powerpc io.h 
#ifndef virt_to_bus
static inline unsigned long virt_to_bus(void *address)
{
	return (unsigned long)address;
}
#endif
*/
static struct sg_table *hantro_gem_prime_get_sg_table(
		struct drm_gem_object *obj)
{
	struct drm_gem_hantro_object *cma_obj = to_drm_gem_hantro_obj(obj);
	struct sg_table *sgt;
	int ret;

	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return NULL;

	ret = dma_get_sgtable(obj->dev->dev, sgt, cma_obj->vaddr,
			cma_obj->paddr, obj->size);
	if (ret < 0)
		goto out;

	return sgt;

out:
	kfree(sgt);
	return NULL;
}

static struct drm_gem_object *hantro_gem_prime_import_sg_table(
	struct drm_device *dev,
	struct dma_buf_attachment *attach,
	struct sg_table *sgt)
{
	struct drm_gem_hantro_object *cma_obj;
	struct drm_gem_object *obj;

	cma_obj = kzalloc(
		sizeof(struct drm_gem_hantro_object), GFP_KERNEL);
	if (!cma_obj)
		return ERR_PTR(-ENOMEM);

	obj = &cma_obj->base;

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
		kfree(cma_obj);
		return ERR_PTR(-ENOMEM);
	}
	cma_obj->paddr = sg_dma_address(sgt->sgl);
	//cma_obj->vaddr = (void *)__va(sg_dma_address(&sgt->sgl[0]));
	cma_obj->vaddr = dma_buf_vmap(attach->dmabuf);
	//cma_obj->paddr = virt_to_phys(cma_obj->vaddr);
	cma_obj->sgt = sgt;
	cma_obj->flag |= HANTRO_GEM_FLAG_IMPORT;
	cma_obj->num_pages = attach->dmabuf->size >> PAGE_SHIFT;

	return obj;
}

static void *hantro_gem_prime_vmap(struct drm_gem_object *obj)
{
	struct drm_gem_hantro_object *cma_obj = to_drm_gem_hantro_obj(obj);

	return cma_obj->vaddr;
}

static void hantro_gem_prime_vunmap(
	struct drm_gem_object *obj,
	void *vaddr)
{
}

/* omitted in kernel version > 5.4.0
static struct reservation_object *hantro_gem_prime_res_obj(
	struct drm_gem_object *obj)
{
	struct drm_gem_hantro_object *hobj = to_drm_gem_hantro_obj(obj);

	return &hobj->kresv;
}
*/

static int hantro_gem_prime_mmap(
	struct drm_gem_object *obj,
	struct vm_area_struct *vma)
{
	struct drm_gem_hantro_object *cma_obj;
	unsigned long page_num = (vma->vm_end - vma->vm_start) >> PAGE_SHIFT;
	int ret = 0;

	cma_obj = to_drm_gem_hantro_obj(obj);

	if (page_num > cma_obj->num_pages)
		return -EINVAL;

	if ((cma_obj->flag & HANTRO_GEM_FLAG_IMPORT) != 0)
		return -EINVAL;

	if ((unsigned long)cma_obj->vaddr == 0)
		return -EINVAL;

	ret = drm_gem_mmap_obj(obj, obj->size, vma);
	if (ret < 0)
		return ret;

	vma->vm_flags &= ~VM_PFNMAP;
	vma->vm_pgoff = 0;
	if (dma_mmap_coherent(
		obj->dev->dev,
		vma,
		cma_obj->vaddr,
		cma_obj->paddr,
		vma->vm_end - vma->vm_start)) {
		drm_gem_vm_close(vma);
		mutex_unlock(&hantro_dev.drm_dev->struct_mutex);
		return -EAGAIN;
	}
	vma->vm_private_data = cma_obj;
	return ret;
}

static struct drm_gem_object *hantro_drm_gem_prime_import(
	struct drm_device *dev,
	struct dma_buf *dma_buf)
{
	return drm_gem_prime_import(dev, dma_buf);
}

static void hantro_gem_free_object(struct drm_gem_object *gem_obj)
{
	struct drm_gem_hantro_object *cma_obj;
/*dma buf imported from others,
 *release data structures allocated by ourselves
 */
	cma_obj = to_drm_gem_hantro_obj(gem_obj);
	if (cma_obj->pages) {
		int i;

		for (i = 0; i < cma_obj->num_pages; i++)
			unref_page(cma_obj->pages[i]);

		kfree(cma_obj->pages);
		cma_obj->pages = NULL;
	}

	drm_gem_free_mmap_offset(gem_obj);
	drm_gem_object_release(gem_obj);
	if (gem_obj->import_attach) {
		if (cma_obj->vaddr)
			dma_buf_vunmap(gem_obj->import_attach->dmabuf, cma_obj->vaddr);
		drm_prime_gem_destroy(gem_obj, cma_obj->sgt);
	} else if (cma_obj->vaddr) {
#if USE_CMA
		dma_release_from_contiguous(
			gem_obj->dev->dev,
			cma_obj->pageaddr,
			cma_obj->num_pages);
#else
		dma_free_coherent(
			gem_obj->dev->dev,
			cma_obj->base.size,
			cma_obj->vaddr,
			cma_obj->paddr);
#endif
	}
	kfree(cma_obj);
}

static int hantro_gem_close(
	struct drm_device *dev,
	void *data,
	struct drm_file *file_priv)
{
	struct drm_gem_close *args = data;
	int ret = 0;
	struct drm_gem_object *obj =
		hantro_gem_object_lookup(dev, file_priv, args->handle);

	if (!obj)
		return -EINVAL;

	ret = drm_gem_handle_delete(file_priv, args->handle);
	hantro_unref_drmobj(obj);
	return ret;
}

static int hantro_gem_open(
	struct drm_device *dev,
	void *data,
	struct drm_file *file_priv)
{
	int ret;
	u32 handle;
	struct drm_gem_open *openarg;
	struct drm_gem_object *obj = NULL;

	openarg = (struct drm_gem_open *)data;

	obj = idr_find(&dev->object_name_idr, (int)openarg->name);
	if (obj)
		hantro_ref_drmobj(obj);
	else
		return -ENOENT;

	ret = drm_gem_handle_create(file_priv, obj, &handle);
	hantro_unref_drmobj(obj);
	if (ret)
		return ret;

	openarg->handle = handle;
	openarg->size = obj->size;

	return ret;
}

static int hantro_map_vaddr(
	struct drm_device *dev,
	void *data,
	struct drm_file *file_priv)
{
	struct hantro_addrmap *pamap = data;
	struct drm_gem_object *obj;
	struct drm_gem_hantro_object *cma_obj;

	obj = hantro_gem_object_lookup(dev, file_priv, pamap->handle);
	if (!obj)
		return -EINVAL;

	cma_obj = to_drm_gem_hantro_obj(obj);
	pamap->vm_addr = (unsigned long)cma_obj->vaddr;
	pamap->phy_addr = cma_obj->paddr;

	hantro_unref_drmobj(obj);
	return 0;
}

static int hantro_get_hwcfg(
	struct drm_device *dev,
	void *data,
	struct drm_file *file_priv)
{
	return hantro_dev.config;
}

static int hantro_get_slicenum(
	struct drm_device *dev,
	void *data,
	struct drm_file *file_priv)
{
	return get_slicenumber();
}


/*reference linux 4.11. ubuntu 16.04 have issues in its drm_gem_flink_ioctl().
 * MODIFICATION:
 * drm_gem_object_lookup(file_priv, args->handle);
 * => drm_gem_object_lookup(dev, file_priv, args->handle);
 */
static int hantro_gem_flink(
	struct drm_device *dev,
	void *data,
	struct drm_file *file_priv)
{
	struct drm_gem_flink *args = data;
	struct drm_gem_object *obj;
	int ret;

	if (!drm_core_check_feature(dev, DRIVER_GEM))
		return -ENODEV;

	obj = hantro_gem_object_lookup(dev, file_priv, args->handle);
	if (obj == NULL)
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

	args->name = (uint64_t) obj->name;
	ret = 0;

err:
	mutex_unlock(&dev->object_name_lock);
	hantro_unref_drmobj(obj);
	return ret;
}

static int hantro_map_dumb(
	struct drm_device *dev,
	void *data,
	struct drm_file *file_priv)
{
	int ret;
	struct drm_mode_map_dumb *temparg = (struct drm_mode_map_dumb *)data;

	ret = hantro_gem_dumb_map_offset(
			file_priv,
			dev,
			temparg->handle,
			&temparg->offset);

	return ret;
}

static int hantro_drm_open(
	struct drm_device *dev,
	struct drm_file *file)
{
	struct idr *ptr;

	ptr = kzalloc(sizeof(struct idr), GFP_KERNEL);
	if (ptr == NULL)
		return -ENOMEM;
	idr_init(ptr);
	file->driver_priv = ptr;
	return 0;
}

/*
 * This function is used to treat abnormal condition such as Ctrl^c or assert.
 * We can't release memory or drm resources in normal way.
 * In kernel document it's suggested driver_priv
 * be used in this call back. In abnormal situation many kernel data
 *structures might be unavaible, e.g. hantro_gem_object_lookup is not
 *working. So we have to save every gem obj info by ourselves.
 */
static void hantro_drm_postclose(
	struct drm_device *dev,
	struct drm_file *file)
{
	int id;
	struct idr *cmalist = (struct idr *)file->driver_priv;
	void *obj;

	mutex_lock(&dev->struct_mutex);
	if (file->driver_priv) {
		idr_for_each_entry(cmalist, obj, id) {
			if (obj) {
				hantro_release_dumb(dev, file, obj);
				idr_remove(cmalist, id);
			}
		}
		idr_destroy(cmalist);
		kfree(file->driver_priv);
		file->driver_priv = NULL;
	}
	mutex_unlock(&dev->struct_mutex);
}

static int hantro_handle_to_fd(
	struct drm_device *dev,
	void *data,
	struct drm_file *file_priv)
{
	int ret;
	struct drm_prime_handle *primeargs = (struct drm_prime_handle *)data;
	struct drm_gem_object *obj;
	struct drm_gem_hantro_object *cma_obj;

	obj = hantro_gem_object_lookup(dev, file_priv, primeargs->handle);
	if (obj == NULL)
		return -ENOENT;

	ret = drm_gem_prime_handle_to_fd(
			dev,
			file_priv,
			primeargs->handle,
			primeargs->flags,
			&primeargs->fd);

	if (ret == 0) {
		cma_obj = to_drm_gem_hantro_obj(obj);
		cma_obj->flag |= HANTRO_GEM_FLAG_EXPORT;
	}
	hantro_unref_drmobj(obj);
	return ret;
}

static int hantro_fd_to_handle(
	struct drm_device *dev,
	void *data,
	struct drm_file *file_priv)
{
	struct drm_prime_handle *primeargs = (struct drm_prime_handle *)data;

	return drm_gem_prime_fd_to_handle(
			dev, file_priv, primeargs->fd, &primeargs->handle);
}

static int hantro_fb_create2(
	struct drm_device *dev,
	void *data,
	struct drm_file *file_priv)
{
	struct drm_mode_fb_cmd2 *mode_cmd = (struct drm_mode_fb_cmd2 *)data;
	struct hantro_drm_fb *vsifb;
	struct drm_gem_object *objs[4];
	struct drm_gem_object *obj;
#if KERNEL_VERSION(4, 16, 0) <= LINUX_VERSION_CODE
	const struct drm_format_info *info =
		drm_get_format_info(dev, mode_cmd);
#endif
	unsigned int hsub;
	unsigned int vsub;
	int num_planes;
	int ret;
	int i;

#if KERNEL_VERSION(4, 16, 0) <= LINUX_VERSION_CODE
	hsub = info->hsub;
	vsub = info->vsub;
	num_planes = min_t(int, info->num_planes, 4);
#else
	hsub = drm_format_horz_chroma_subsampling(mode_cmd->pixel_format);
	vsub = drm_format_vert_chroma_subsampling(mode_cmd->pixel_format);
	num_planes = min(drm_format_num_planes(mode_cmd->pixel_format), 4);
#endif
	for (i = 0; i < num_planes; i++) {
		unsigned int width = mode_cmd->width / (i ? hsub : 1);
		unsigned int height = mode_cmd->height / (i ? vsub : 1);
		unsigned int min_size;

		obj = hantro_gem_object_lookup(
				dev,
				file_priv,
				mode_cmd->handles[i]);
		if (!obj) {
			ret = -ENXIO;
			goto err_gem_object_unreference;
		}
		hantro_unref_drmobj(obj);
		min_size = (height - 1) * mode_cmd->pitches[i] +
			mode_cmd->offsets[i] +
#if KERNEL_VERSION(4, 16, 0) <= LINUX_VERSION_CODE
			width * info->cpp[i];
#else
			width * drm_format_plane_cpp(mode_cmd->pixel_format, i);
#endif
		if (obj->size < min_size) {
			//hantro_unref_drmobj(obj);
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
	for (i--; i >= 0; i--)
		;//hantro_unref_drmobj(objs[i]);

	return ret;
}

static int hantro_fb_create(
	struct drm_device *dev,
	void *data,
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

static int hantro_get_version(
	struct drm_device *dev,
	void *data,
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

static int hantro_get_cap(
	struct drm_device *dev,
	void *data,
	struct drm_file *file_priv)
{
	struct drm_get_cap *req = (struct drm_get_cap *)data;

	req->value = 0;
	/*some values should be reset*/
	switch (req->capability) {
	case DRM_CAP_PRIME:
		req->value |= dev->driver->prime_fd_to_handle ?
			DRM_PRIME_CAP_IMPORT : 0;
		req->value |= dev->driver->prime_handle_to_fd ?
			DRM_PRIME_CAP_EXPORT : 0;
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

/*just a test API for any purpose*/
static int hantro_test(
	struct drm_device *dev,
	void *data,
	struct drm_file *file_priv)
{
	unsigned int *input = data;
	int handle = *input;
	struct drm_gem_object *obj;
	hantro_fence_t *pfence;
	int ret = 10 * HZ; /*timeout*/

	obj = hantro_gem_object_lookup(dev, file_priv, handle);
	if (!obj)
		return -EINVAL;

	pfence = dma_resv_get_excl(obj->dma_buf->resv);
	while (ret > 0)
		ret = schedule_timeout(ret);

	hantro_fence_signal(pfence);
	hantro_unref_drmobj(obj);
	return 0;
}

static int hantro_getprimeaddr(
	struct drm_device *dev,
	void *data,
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
	return 0;
}

static int hantro_ptr_to_phys(
	struct drm_device *dev,
	void *data,
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


static int hantro_getmagic(
	struct drm_device *dev,
	void *data,
	struct drm_file *file_priv)
{
	struct drm_auth *auth = data;
	int ret = 0;

	mutex_lock(&dev->struct_mutex);
	if (!file_priv->magic) {
		ret = idr_alloc(&file_priv->master->magic_map, file_priv,
				1, 0, GFP_KERNEL);
		if (ret >= 0)
			file_priv->magic = ret;
	}
	auth->magic = file_priv->magic;
	DBG("kmagic %d\n", auth->magic);
	mutex_unlock(&dev->struct_mutex);

	return ret < 0 ? ret : 0;
}

static int hantro_authmagic(
	struct drm_device *dev,
	void *data,
	struct drm_file *file_priv)
{
	struct drm_auth *auth = data;
	struct drm_file *file;

	mutex_lock(&dev->struct_mutex);
	file = idr_find(&file_priv->master->magic_map, auth->magic);
	DBG("get kmagic %d\n", auth->magic);
	if (file) {
		file->authenticated = 1;
		idr_replace(&file_priv->master->magic_map, NULL, auth->magic);
	}
	mutex_unlock(&dev->struct_mutex);

	return file ? 0 : -EINVAL;
}

#define DRM_IOCTL_DEF(ioctl, _func, _flags)      \
	[DRM_IOCTL_NR(ioctl)] = {            \
	.cmd = ioctl,                  \
	.func = _func,                  \
	.flags = _flags,            \
	.name = #ioctl                  \
	}

/*after kernel 4.16 this definition is removed*/
#ifndef DRM_CONTROL_ALLOW
#define DRM_CONTROL_ALLOW 0
#endif
/* Ioctl table */
static const struct drm_ioctl_desc hantro_ioctls[] = {
	DRM_IOCTL_DEF(DRM_IOCTL_VERSION, hantro_get_version, DRM_UNLOCKED | DRM_RENDER_ALLOW | DRM_CONTROL_ALLOW),
	DRM_IOCTL_DEF(DRM_IOCTL_GET_UNIQUE, drm_invalid_op, DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_GET_MAGIC, hantro_getmagic, DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_IRQ_BUSID, drm_invalid_op, DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_GET_MAP, drm_invalid_op, DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_GET_CLIENT, drm_invalid_op, DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_GET_STATS, drm_invalid_op, DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_GET_CAP, hantro_get_cap, DRM_UNLOCKED | DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF(DRM_IOCTL_SET_CLIENT_CAP, drm_invalid_op, DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_SET_VERSION, drm_invalid_op, DRM_UNLOCKED | DRM_MASTER),

	DRM_IOCTL_DEF(DRM_IOCTL_SET_UNIQUE, drm_invalid_op, DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_BLOCK, drm_invalid_op, DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_UNBLOCK, drm_invalid_op, DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_AUTH_MAGIC, hantro_authmagic, DRM_AUTH | DRM_UNLOCKED | DRM_MASTER),

	DRM_IOCTL_DEF(DRM_IOCTL_ADD_MAP, drm_invalid_op, DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_RM_MAP, drm_invalid_op, DRM_AUTH),

	DRM_IOCTL_DEF(DRM_IOCTL_SET_SAREA_CTX, drm_invalid_op, DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_GET_SAREA_CTX, drm_invalid_op, DRM_AUTH),

	DRM_IOCTL_DEF(DRM_IOCTL_SET_MASTER, drm_invalid_op, DRM_UNLOCKED | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_DROP_MASTER, drm_invalid_op, DRM_UNLOCKED | DRM_ROOT_ONLY),

	DRM_IOCTL_DEF(DRM_IOCTL_ADD_CTX, drm_invalid_op, DRM_AUTH | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_RM_CTX, drm_invalid_op, DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_MOD_CTX, drm_invalid_op, DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_GET_CTX, drm_invalid_op, DRM_AUTH),
	DRM_IOCTL_DEF(DRM_IOCTL_SWITCH_CTX, drm_invalid_op, DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_NEW_CTX, drm_invalid_op, DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_RES_CTX, drm_invalid_op, DRM_AUTH),

	DRM_IOCTL_DEF(DRM_IOCTL_ADD_DRAW, drm_invalid_op, DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_RM_DRAW, drm_invalid_op, DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),

	DRM_IOCTL_DEF(DRM_IOCTL_LOCK, drm_invalid_op, DRM_AUTH),
	DRM_IOCTL_DEF(DRM_IOCTL_UNLOCK, drm_invalid_op, DRM_AUTH),

	DRM_IOCTL_DEF(DRM_IOCTL_FINISH, drm_invalid_op, DRM_AUTH),

	DRM_IOCTL_DEF(DRM_IOCTL_ADD_BUFS, drm_invalid_op, DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_MARK_BUFS, drm_invalid_op, DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_INFO_BUFS, drm_invalid_op, DRM_AUTH),
	DRM_IOCTL_DEF(DRM_IOCTL_MAP_BUFS, drm_invalid_op, DRM_AUTH),
	DRM_IOCTL_DEF(DRM_IOCTL_FREE_BUFS, drm_invalid_op, DRM_AUTH),
	DRM_IOCTL_DEF(DRM_IOCTL_DMA, drm_invalid_op, DRM_AUTH),

	DRM_IOCTL_DEF(DRM_IOCTL_CONTROL, drm_invalid_op, DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),

#if IS_ENABLED(CONFIG_AGP)
	DRM_IOCTL_DEF(DRM_IOCTL_AGP_ACQUIRE, drm_invalid_op, DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_AGP_RELEASE, drm_invalid_op, DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_AGP_ENABLE, drm_invalid_op, DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_AGP_INFO, drm_invalid_op, DRM_AUTH),
	DRM_IOCTL_DEF(DRM_IOCTL_AGP_ALLOC, drm_invalid_op, DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_AGP_FREE, drm_invalid_op, DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_AGP_BIND, drm_invalid_op, DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_AGP_UNBIND, drm_invalid_op, DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
#endif

	DRM_IOCTL_DEF(DRM_IOCTL_SG_ALLOC, drm_invalid_op, DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF(DRM_IOCTL_SG_FREE, drm_invalid_op, DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),

	DRM_IOCTL_DEF(DRM_IOCTL_WAIT_VBLANK, drm_invalid_op, DRM_UNLOCKED),

	DRM_IOCTL_DEF(DRM_IOCTL_MODESET_CTL, drm_invalid_op, 0),

	DRM_IOCTL_DEF(DRM_IOCTL_UPDATE_DRAW, drm_invalid_op, DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),

	DRM_IOCTL_DEF(DRM_IOCTL_GEM_CLOSE, hantro_gem_close, DRM_UNLOCKED | DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF(DRM_IOCTL_GEM_FLINK, hantro_gem_flink, DRM_AUTH | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_GEM_OPEN, hantro_gem_open, DRM_AUTH | DRM_UNLOCKED),

	DRM_IOCTL_DEF(DRM_IOCTL_MODE_GETRESOURCES, drm_invalid_op, DRM_CONTROL_ALLOW | DRM_UNLOCKED),

	DRM_IOCTL_DEF(DRM_IOCTL_PRIME_HANDLE_TO_FD, hantro_handle_to_fd, DRM_AUTH | DRM_UNLOCKED | DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF(DRM_IOCTL_PRIME_FD_TO_HANDLE, hantro_fd_to_handle, DRM_AUTH | DRM_UNLOCKED | DRM_RENDER_ALLOW),

	DRM_IOCTL_DEF(DRM_IOCTL_MODE_GETPLANERESOURCES, drm_invalid_op, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_GETCRTC, drm_invalid_op, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_SETCRTC, drm_invalid_op, DRM_MASTER | DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_GETPLANE, drm_invalid_op, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_SETPLANE, drm_invalid_op, DRM_MASTER | DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_CURSOR, drm_invalid_op, DRM_MASTER | DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_GETGAMMA, drm_invalid_op, DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_SETGAMMA, drm_invalid_op, DRM_MASTER | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_GETENCODER, drm_invalid_op, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_GETCONNECTOR, drm_invalid_op, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_ATTACHMODE, drm_invalid_op, DRM_MASTER | DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_DETACHMODE, drm_invalid_op, DRM_MASTER | DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_GETPROPERTY, drm_invalid_op, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_SETPROPERTY, drm_invalid_op, DRM_MASTER | DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_GETPROPBLOB, drm_invalid_op, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_GETFB, drm_invalid_op, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_ADDFB, hantro_fb_create, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_ADDFB2, hantro_fb_create2, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_RMFB, drm_invalid_op, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_PAGE_FLIP, drm_invalid_op, DRM_MASTER | DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_DIRTYFB, drm_invalid_op, DRM_MASTER | DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_CREATE_DUMB, hantro_gem_dumb_create, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_MAP_DUMB, hantro_map_dumb, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_DESTROY_DUMB, hantro_destroy_dumb, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_OBJ_GETPROPERTIES, drm_invalid_op, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_OBJ_SETPROPERTY, drm_invalid_op, DRM_MASTER | DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_CURSOR2, drm_invalid_op, DRM_MASTER | DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_ATOMIC, drm_invalid_op, DRM_MASTER | DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_CREATEPROPBLOB, drm_invalid_op, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_MODE_DESTROYPROPBLOB, drm_invalid_op, DRM_CONTROL_ALLOW | DRM_UNLOCKED),

	/*hantro specific ioctls*/
	DRM_IOCTL_DEF(DRM_IOCTL_HANTRO_TESTCMD, hantro_test, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_HANTRO_GETPADDR, hantro_map_vaddr, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_HANTRO_HWCFG, hantro_get_hwcfg, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_HANTRO_TESTREADY, hantro_testbufvalid, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_HANTRO_SETDOMAIN, hantro_setdomain, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_HANTRO_ACQUIREBUF, hantro_acquirebuf, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_HANTRO_RELEASEBUF, hantro_releasebuf, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_HANTRO_GETPRIMEADDR, hantro_getprimeaddr, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_HANTRO_PTR_PHYADDR, hantro_ptr_to_phys, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_HANTRO_GET_SLICENUM, hantro_get_slicenum, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
};

#if DRM_CONTROL_ALLOW == 0
#undef DRM_CONTROL_ALLOW
#endif

#define HANTRO_IOCTL_COUNT      ARRAY_SIZE(hantro_ioctls)
static long hantro_ioctl(
	struct file *filp,
	unsigned int cmd,
	unsigned long arg)
{
	struct drm_file *file_priv = filp->private_data;
	struct drm_device *dev = hantro_dev.drm_dev;
	const struct drm_ioctl_desc *ioctl = NULL;
	drm_ioctl_t *func;
	unsigned int nr = DRM_IOCTL_NR(cmd);
	int retcode = 0;
	char stack_kdata[128];
	char *kdata = stack_kdata;
	unsigned int in_size, out_size;

#if KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE
	if (drm_dev_is_unplugged(dev))
		return -ENODEV;
#else
	if (drm_device_is_unplugged(dev))
		return -ENODEV;
#endif

	out_size = in_size = _IOC_SIZE(cmd);
	//printk("ioctl cmd %d\n", nr);

	if (in_size > 0) {
		if (_IOC_DIR(cmd) & _IOC_READ)
			retcode =
				!hantro_access_ok(VERIFY_WRITE, (void *) arg, in_size);
		else if (_IOC_DIR(cmd) & _IOC_WRITE)
			retcode =
				!hantro_access_ok(VERIFY_READ, (void *) arg, in_size);
		if (retcode)
			return -EFAULT;
	}
	if (nr >= DRM_IOCTL_NR(HX280ENC_IOC_START)
		&& nr <= DRM_IOCTL_NR(HX280ENC_IOC_END)) {
#ifdef HAS_VC8000E
		return hantroenc_ioctl(filp, cmd, arg);
#else
		if (cmd == HX280ENC_IOCG_CORE_NUM) {
			int corenum = 0;

			__put_user(corenum, (unsigned int *) arg);
		} else {
			return -EFAULT;
		}
#endif
	}
	if (nr >= DRM_IOCTL_NR(HANTRODEC_IOC_START) &&
		nr <= DRM_IOCTL_NR(HANTRODEC_IOC_END)) {
#ifdef HAS_VC8000D
		return hantrodec_ioctl(filp, cmd, arg);
#else
		return -EFAULT;
#endif
	}

	if (nr >= DRM_IOCTL_NR(HANTROCACHE_IOC_START) &&
		nr <= DRM_IOCTL_NR(HANTROCACHE_IOC_END)) {
#ifdef HAS_CACHECORE
		return hantrocache_ioctl(filp, cmd, arg);
#else
		return -EFAULT;
#endif
	}

	if (nr >= DRM_IOCTL_NR(HANTROSLICE_IOC_START) &&
		nr <= DRM_IOCTL_NR(HANTROSLICE_IOC_END)) {
		return hantroslice_ioctl(filp, cmd, arg);
	}

	if (nr >= HANTRO_IOCTL_COUNT)
		return -EINVAL;
	ioctl = &hantro_ioctls[nr];

	if (copy_from_user(kdata, (void __user *)arg, in_size) != 0)
		return  -EFAULT;

	if (cmd == DRM_IOCTL_MODE_SETCRTC ||
		cmd == DRM_IOCTL_MODE_GETRESOURCES ||
		cmd == DRM_IOCTL_SET_CLIENT_CAP ||
		cmd == DRM_IOCTL_MODE_GETCRTC ||
		cmd == DRM_IOCTL_MODE_GETENCODER ||
		cmd == DRM_IOCTL_MODE_GETCONNECTOR ||
		cmd == DRM_IOCTL_MODE_GETFB)    {
		retcode = drm_ioctl(filp, cmd, arg);
		return retcode;
	}
	func = ioctl->func;
	if (func == NULL)
		return -EINVAL;
	retcode = func(dev, kdata, file_priv);

	if (copy_to_user((void __user *)arg, kdata, out_size) != 0)
		retcode = -EFAULT;

	return retcode;
}

/* VFS methods */
static const struct file_operations hantro_fops = {
	.owner = THIS_MODULE,
	.open = hantro_device_open,
	.mmap = hantro_mmap,
	.release = hantro_device_release,
	.poll = drm_poll,
	.read = drm_read,
	.unlocked_ioctl = hantro_ioctl,            //drm_ioctl,
	.compat_ioctl = drm_compat_ioctl,
};

void hantro_gem_vm_close(struct vm_area_struct *vma)
{
	struct drm_gem_hantro_object *obj =
		(struct drm_gem_hantro_object *)vma->vm_private_data;
	/*unmap callback*/

	if (obj->pages) {
		int i;

		for (i = 0; i < obj->num_pages; i++)
			unref_page(obj->pages[i]);

		kfree(obj->pages);
		obj->pages = NULL;
	}
	drm_gem_vm_close(vma);
}

static void hantro_release(struct drm_device *dev)
{
#if KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE
	drm_dev_unregister(hantro_dev.drm_dev);
#else
	drm_dev_fini(hantro_dev.drm_dev);
#endif
}

#if KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE
static int hantro_unload(struct drm_device *dev)
{
	return 0;
}
#endif

#if KERNEL_VERSION(4, 20, 0) <= LINUX_VERSION_CODE
static void hantro_gem_dmabuf_release(struct dma_buf *dma_buf)
{
	return drm_gem_dmabuf_release(dma_buf);
}

static void *hantro_gem_dmabuf_kmap(struct dma_buf *dma_buf, unsigned long page_num)
{
	return NULL;
}

static void hantro_gem_dmabuf_kunmap(struct dma_buf *dma_buf, unsigned long page_num,
			   void *addr)
{
}

static int hantro_gem_map_attach(struct dma_buf *dma_buf,
		       struct dma_buf_attachment *attach)
{
	int ret;
	struct drm_gem_hantro_object *cma_obj = (struct drm_gem_hantro_object *)dma_buf->priv;

	ret =  drm_gem_map_attach(dma_buf, attach);
	if (ret == 0)
		cma_obj->flag |= HANTRO_GEM_FLAG_EXPORTUSED;

	return ret;
}

static void hantro_gem_map_detach(struct dma_buf *dma_buf,
		       struct dma_buf_attachment *attach)
{
	drm_gem_map_detach(dma_buf, attach);
}

static struct sg_table *hantro_gem_map_dma_buf(struct dma_buf_attachment *attach,
				     enum dma_data_direction dir)
{
	return drm_gem_map_dma_buf(attach, dir);
}

static int hantro_gem_dmabuf_mmap(struct dma_buf *dma_buf, struct vm_area_struct *vma)
{
	return drm_gem_dmabuf_mmap(dma_buf, vma);
}

static void *hantro_gem_dmabuf_vmap(struct dma_buf *dma_buf)
{
	return drm_gem_dmabuf_vmap(dma_buf);
}

static const struct dma_buf_ops hantro_dmabuf_ops =  {
	.attach = hantro_gem_map_attach,
	.detach = hantro_gem_map_detach,
	.map_dma_buf = hantro_gem_map_dma_buf,
	.unmap_dma_buf = drm_gem_unmap_dma_buf,
	.release = hantro_gem_dmabuf_release,
	.map = hantro_gem_dmabuf_kmap,
	.unmap = hantro_gem_dmabuf_kunmap,
	.mmap = hantro_gem_dmabuf_mmap,
	.vmap = hantro_gem_dmabuf_vmap,
	.vunmap = drm_gem_dmabuf_vunmap,
};
#endif	/*#if KERNEL_VERSION(4, 20, 0) <= LINUX_VERSION_CODE*/

static struct drm_driver hantro_drm_driver;

struct dma_buf *hantro_prime_export(
        struct drm_gem_object *obj,
        int flags)
{
        struct dma_buf_export_info exp_info = {
                .exp_name = KBUILD_MODNAME,
                .owner = obj->dev->driver->fops->owner,
                .ops = &hantro_dmabuf_ops,
                .size = obj->size,
                .flags = flags,
                .priv = obj,
        };
        return drm_gem_dmabuf_export(obj->dev, &exp_info);
}

static void hantro_close_object(
	struct drm_gem_object *obj,
	struct drm_file *file_priv)
{
	struct drm_gem_hantro_object *cma_obj;

	cma_obj = to_drm_gem_hantro_obj(obj);
	if (obj->dma_buf && (cma_obj->flag & HANTRO_GEM_FLAG_EXPORTUSED))
		dma_buf_put(obj->dma_buf);
}

static int hantro_gem_prime_handle_to_fd(
	struct drm_device *dev,
	struct drm_file *filp, uint32_t handle,
	uint32_t flags,
	int *prime_fd)
{
	return drm_gem_prime_handle_to_fd(dev, filp, handle, flags, prime_fd);
}

static const struct vm_operations_struct hantro_drm_gem_cma_vm_ops = {
	.open = drm_gem_vm_open,
	.close = hantro_gem_vm_close,
	.fault = hantro_vm_fault,
};

static struct drm_driver hantro_drm_driver = {
	//these two are related with controlD and renderD
	.driver_features      = DRIVER_GEM | DRIVER_RENDER,
	.get_vblank_counter      = hantro_vblank_no_hw_counter,
	.open                        = hantro_drm_open,
	.postclose                        = hantro_drm_postclose,
#if KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE
	.unload = hantro_unload,
#else
	.release = hantro_release,
#endif
	.dumb_destroy            = drm_gem_dumb_destroy,
	.dumb_create            = hantro_gem_dumb_create_internal,
	.dumb_map_offset      = hantro_gem_dumb_map_offset,
	.gem_open_object = hantro_gem_open_obj,
	.gem_close_object = hantro_close_object,
	.gem_prime_export      = hantro_prime_export,
	.gem_prime_import      = hantro_drm_gem_prime_import,
	.prime_handle_to_fd      = hantro_gem_prime_handle_to_fd,
	.prime_fd_to_handle      = drm_gem_prime_fd_to_handle,
	.gem_prime_import_sg_table = hantro_gem_prime_import_sg_table,
	.gem_prime_get_sg_table      = hantro_gem_prime_get_sg_table,
	.gem_prime_vmap            = hantro_gem_prime_vmap,
	.gem_prime_vunmap      = hantro_gem_prime_vunmap,
	//.gem_prime_res_obj = hantro_gem_prime_res_obj,
	.gem_prime_mmap            = hantro_gem_prime_mmap,
	.gem_free_object_unlocked = hantro_gem_free_object,
	.gem_vm_ops            = &hantro_drm_gem_cma_vm_ops,
	.fops                  = &hantro_fops,
	.name      = DRIVER_NAME,
	.desc      = DRIVER_DESC,
	.date      = DRIVER_DATE,
	.major      = DRIVER_MAJOR,
	.minor      = DRIVER_MINOR,
};

static ssize_t
bandwidthDecRead_show(struct device *kdev, struct device_attribute *attr, char *buf)
{
	/* sys/bus/platform/drivers/hantro/xxxxx.vpu/bandwidthDecRead
	 *  used to show bandwidth info to user space
	 *  all core' bandwidth might be exported in same string.
	 *  data is just an example. Real data should be read from HW registers
	 *  this file is read only.
	 */
	u32 bandwidth = hantrodec_readbandwidth(1);

	return snprintf(buf, PAGE_SIZE, "%d\n", bandwidth);
}

static ssize_t
bandwidthDecWrite_show(struct device *kdev, struct device_attribute *attr, char *buf)
{
	u32 bandwidth = hantrodec_readbandwidth(0);

	return snprintf(buf, PAGE_SIZE, "%d\n", bandwidth);
}

static ssize_t
bandwidthEncRead_show(struct device *kdev, struct device_attribute *attr, char *buf)
{
	u32 bandwidth = hantroenc_readbandwidth(1);

	return snprintf(buf, PAGE_SIZE, "%d\n", bandwidth);
}

static ssize_t
bandwidthEncWrite_show(struct device *kdev, struct device_attribute *attr, char *buf)
{
	u32 bandwidth = hantroenc_readbandwidth(0);

	return snprintf(buf, PAGE_SIZE, "%d\n", bandwidth);
}

static DEVICE_ATTR(bandwidthDecRead, 0444, bandwidthDecRead_show, NULL);
static DEVICE_ATTR(bandwidthDecWrite, 0444, bandwidthDecWrite_show, NULL);
static DEVICE_ATTR(bandwidthEncRead, 0444, bandwidthEncRead_show, NULL);
static DEVICE_ATTR(bandwidthEncWrite, 0444, bandwidthEncWrite_show, NULL);

static int hantro_createsysfsAPI(struct device *dev)
{
	int result;

	result = device_create_file(dev, &dev_attr_bandwidthDecRead);
	if (result != 0)
		return result;

	result = device_create_file(dev, &dev_attr_bandwidthDecWrite);
	if (result != 0) {
		device_remove_file(dev, &dev_attr_bandwidthDecRead);
		return result;
	}

	result = device_create_file(dev, &dev_attr_bandwidthEncRead);
	if (result != 0) {
		device_remove_file(dev, &dev_attr_bandwidthDecRead);
		device_remove_file(dev, &dev_attr_bandwidthDecWrite);
		return result;
	}

	result = device_create_file(dev, &dev_attr_bandwidthEncWrite);
	if (result != 0) {
		device_remove_file(dev, &dev_attr_bandwidthDecRead);
		device_remove_file(dev, &dev_attr_bandwidthDecWrite);
		device_remove_file(dev, &dev_attr_bandwidthEncRead);
		return result;
	}
	return 0;
}
#ifdef USE_DTB_PROBE
static int hantro_analyze_resources(struct platform_device *dev, struct hantro_core_info *core_rc)
{
	int i, k, c, core_num = 0, len;
	unsigned long validmemtype = IORESOURCE_IO | IORESOURCE_MEM	| IORESOURCE_REG;

	for (i = 0; i < dev->num_resources; i++) {
		struct resource *r = &dev->resource[i];

		if ((resource_type(r) & validmemtype) == 0)
			continue;
		if (r->flags & IORESOURCE_BUSY)
			continue;

		core_rc[core_num].mem = r;
		core_rc[core_num].irqnum = 0;

		/* get its irqs */
		for (k = 0; k < dev->num_resources; k++) {
			struct resource *r = &dev->resource[k];

			if ((resource_type(r) & IORESOURCE_IRQ) == 0)
				continue;
			/*need name be irq_x format */
			len = strlen(r->name);
			c = r->name[len - 1] - 0x30;
			if (c != core_num)
				continue;
			core_rc[core_num].irqlist[core_rc[core_num].irqnum] = r;
			core_rc[core_num].irqnum++;
		}
		core_num++;
	}

	return core_num;
}
#endif
static int hantro_drm_probe(struct platform_device *pdev)
{
	int result = 0, core_num = 0;
	struct hantro_core_info *core_rc = NULL;

	pr_info("dev %s probe", pdev->name);

#ifdef USE_DTB_PROBE
	if (pdev->num_resources <= 0)
		return 0;
	core_rc = kzalloc(sizeof(struct hantro_core_info) * pdev->num_resources, GFP_KERNEL);
	if (!core_rc)
		return -ENOMEM;

	memset(core_rc, 0, sizeof(struct hantro_core_info) * pdev->num_resources);
	core_num = hantro_analyze_resources(pdev, core_rc);
#endif

#if USE_HW == 1
#ifdef HAS_VC8000D
	/*check if pdev equals hantro_dev.platformdev*/
	result = hantrodec_probe(pdev, core_rc, core_num);
	if (result == 0)
		hantro_dev.config |= CONFIG_HWDEC;
#endif
#ifdef HAS_VC8000E
	result = hantroenc_probe(pdev, core_rc, core_num);
	if (result == 0)
		hantro_dev.config |= CONFIG_HWENC;
#endif
#ifdef HAS_CACHECORE
	result = cache_probe(pdev, core_rc, core_num);
	if (result == 0)
		hantro_dev.config |= CONFIG_L2CACHE;
#endif
#endif

#ifdef USE_DTB_PROBE
	kfree(core_rc);
#endif
	return 0;
}

static int hantro_drm_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	device_remove_file(dev, &dev_attr_bandwidthDecRead);
	device_remove_file(dev, &dev_attr_bandwidthDecWrite);
	device_remove_file(dev, &dev_attr_bandwidthEncRead);
	device_remove_file(dev, &dev_attr_bandwidthEncWrite);
	return 0;
}

static const struct platform_device_id hantro_drm_platform_ids[] = {
	{
		.name            = DRIVER_NAME,
	},
	{ },
};
MODULE_DEVICE_TABLE(platform, hantro_drm_platform_ids);

static const struct of_device_id hantro_of_match[] = {
	/*to match dtb, else reg io will fail*/
	{ .compatible = "thunderbay,hantro", },
	{/* sentinel */}
};

static int hantro_pm_suspend(struct device *kdev)
{
	//what should we do on HW? Disable IRQ, slow down clock,
	// disable DMA, etc?
	//printk("\nhantro suspend");
	return 0;
}

static int hantro_pm_resume(struct device *kdev)
{
	//printk("\nhantro resume");
	return 0;
}

static const struct dev_pm_ops hantro_pm_ops = {
	/* since we only support S3, only several interfaces should be supported
	 * echo -n "freeze" (or sth else) > /sys/power/state will trigger them
	 * current suspend and resume seem to be enough
	 * maybe suspend_noirq and resume_noirq will be inserted in future
	 */
	//.prepare
	.suspend = hantro_pm_suspend,
	//.suspend_late
	//.suspend_noirq

	//.resume_noirq
	//.resume_early
	.resume = hantro_pm_resume,
	//.complete
};

static struct platform_driver hantro_drm_platform_driver = {
	.probe      = hantro_drm_probe,
	.remove      = hantro_drm_remove,
	.driver      = {
		.name      = DRIVER_NAME,
		.owner		= THIS_MODULE,
		.of_match_table = hantro_of_match,
		.pm = &hantro_pm_ops,
	},
	.id_table = hantro_drm_platform_ids,
};

static const struct platform_device_info hantro_platform_info = {
	.name		= DRIVER_NAME,
	.id		= -1,
	.dma_mask	= DMA_BIT_MASK(64),
};

#if KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE
static int hantro_major = 1;  /* dynamic */
#endif
void __exit hantro_cleanup(void)
{
	hantro_dev.config = 0;
#if USE_HW == 1      /*hw init*/
#ifdef HAS_VC8000D
	hantrodec_cleanup();
#endif
#ifdef HAS_VC8000E
	hantroenc_cleanup();
#endif
#ifdef HAS_CACHECORE
	cache_cleanup();
#endif
#endif
	/*this one must be after above ones to maintain list*/
	slice_remove();
	releaseFenceData();
#if KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE
	unregister_chrdev(hantro_major, DRIVER_NAME);
#else
	drm_dev_unregister(hantro_dev.drm_dev);
	drm_dev_fini(hantro_dev.drm_dev);
#endif
	platform_device_unregister(hantro_dev.platformdev);
	platform_driver_unregister(&hantro_drm_platform_driver);
}

int __init hantro_init(void)
{
	int result;

	/*_init functions will init static vairables, while probe will init dynamic emelemts from DTB */
	/*slice init must be in first to clear list*/
	slice_init();
	hantroenc_init();
	hantrodec_init();
	cache_init();
	hantro_dev.config = 0;
	hantro_dev.platformdev = platform_device_register_full(&hantro_platform_info);
	if (hantro_dev.platformdev == NULL) {
		pr_info("hantro create platform device fail");
		return -1;
	}

	/*it must be here instead of in probe*/
	hantro_dev.drm_dev = drm_dev_alloc(&hantro_drm_driver, &hantro_dev.platformdev->dev);
	if (IS_ERR(hantro_dev.drm_dev)) {
		DBG("init drm failed\n");
		return PTR_ERR(hantro_dev.drm_dev);
	}

#if KERNEL_VERSION(4, 13, 0) > LINUX_VERSION_CODE
	hantro_dev.drm_dev->platformdev = hantro_dev.platformdev;
#endif
	hantro_dev.drm_dev->dev = &hantro_dev.platformdev->dev;
	pr_info("hantro device created");

	drm_mode_config_init(hantro_dev.drm_dev);
#if KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE
	result = register_chrdev(hantro_major, DRIVER_NAME, &hantro_fops);
	if (result < 0) {
		return result;
	} else if (result != 0) { /* this is for dynamic major */
		hantro_major = result;
	}
#else
	result  = drm_dev_register(hantro_dev.drm_dev, 0);
#endif
	result = platform_driver_register(&hantro_drm_platform_driver);
	if (result < 0) {
#if KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE
		unregister_chrdev(hantro_major, DRIVER_NAME);
#else
		drm_dev_unregister(hantro_dev.drm_dev);
		drm_dev_fini(hantro_dev.drm_dev);
#endif
		platform_device_unregister(hantro_dev.platformdev);
	} else {
		result = hantro_createsysfsAPI(hantro_dev.drm_dev->dev);
		if (result != 0)
			pr_info("create sysfs fail");
		initFenceData();
	}
	slice_printdebug();
	return result;
}

module_init(hantro_init);
module_exit(hantro_cleanup);

/* module description */
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Verisilicon");
MODULE_DESCRIPTION("Hantro DRM manager");
