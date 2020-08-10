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
#include "hantrodec400.h"
#include <linux/of_reserved_mem.h>
#include <linux/of_irq.h>
#include <linux/reset.h>
#include <linux/clk.h>

/* compile options */
#define ENABLE_VC8000E  1
#define ENABLE_VC8000D  1
#define ENABLE_DEC400_CACHE 0

/* debug */
//#define DMA_DEBUG_ALLOC
//#define ENABLE_DEBUG
#ifdef ENABLE_DEBUG
#define DBG(...) pr_info(__VA_ARGS__)
#else
#define DBG(...)
#endif

#define DRIVER_NAME      "hantro"
#define DRIVER_DESC      "hantro DRM"
#define DRIVER_DATE      "20200526"
#define DRIVER_MAJOR      1
#define DRIVER_MINOR      1

struct hantro_device_handle hantro_dev;
static ssize_t memory_usage_show_internal(int sliceidx, char *buf);

bool verbose;
module_param(verbose, bool, 0);
MODULE_PARM_DESC(verbose, "Verbose log operations "
		"(default 0)");

bool enable_encode = ENABLE_VC8000E;
module_param(enable_encode, bool, 0);
MODULE_PARM_DESC(enable_encode, "Enable Encode"
		"(default 1)");

bool enable_decode = ENABLE_VC8000D;
module_param(enable_decode, bool, 0);
MODULE_PARM_DESC(enable_decode, "Enable Decode"
		"(default 1)");

bool enable_dec400 = ENABLE_DEC400_CACHE;
module_param(enable_dec400, bool, 0);
MODULE_PARM_DESC(enable_dec400, "Enable DEC400/L2"
		"(default 1)");

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
	struct file_data *data = (struct file_data *)priv->driver_priv;
	struct idr *list = (struct idr *)data->list;

	ret = idr_alloc(list, obj, 1, 0, GFP_KERNEL);
	return (ret > 0 ? 0 : -ENOMEM);
}

static void hantro_unrecordmem(
	struct drm_file *priv,
	void *obj)
{
	int id;
	struct file_data *data = (struct file_data *)priv->driver_priv;
	struct idr *list = (struct idr *)data->list;
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
	unsigned int sliceidx = args->handle;
	struct slice_info *pslice = getslicenode(sliceidx);

	if (pslice == NULL)
		return -EINVAL;
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
	cma_obj->sliceidx = sliceidx;

/*CMA is temp disabled here for these reasons:
 *1. to bring up CMA, CONFIG_DMA_CMA shoule be enabled in kernel building
 *2. dma_alloc/release_from_contiguous should be exported in kernel code
 *3. a boot parameter like "cma=268435456@134217728" should be added
 *4. CMA's memory management is not stable.
 *We could choose to use CMA or page alloc by module parameter later.
 */
	cma_obj->vaddr =
		dma_alloc_coherent(
			pslice->dev,
			args->size,
			&cma_obj->paddr,
			GFP_KERNEL | GFP_DMA);
#ifdef DMA_DEBUG_ALLOC
	pr_info("dumb_create_internal: dma_alloc_coherent cma_obj->paddr = %llx, size = %d, pslice->dev = %s\n", cma_obj->paddr, args->size,
		dev_name(pslice->dev));
#endif
	if (cma_obj->vaddr == NULL) {
		kfree(cma_obj);
		ret = -ENOMEM;
		goto out;
	}

	drm_gem_object_init(dev, obj, args->size);

	ret = drm_gem_handle_create(file_priv, obj, &args->handle);
	if (ret == 0)
		ret = hantro_recordmem(file_priv, cma_obj, args->size);
	if (ret) {
		dma_free_coherent(
			pslice->dev,
			args->size,
			cma_obj->vaddr,
			cma_obj->paddr);
		kfree(cma_obj);
	}
	init_hantro_resv(&cma_obj->kresv, cma_obj);
	cma_obj->handle = args->handle;
#ifdef DMA_DEBUG_ALLOC
	pr_info("%s:%d,%lx:%llx:%llx\n", __func__, sliceidx, (unsigned long)pslice->dev, cma_obj->paddr, args->size);
#endif

out:
	mutex_unlock(&dev->struct_mutex);
	if (ret == -ENOMEM) {
		char *buf = kzalloc(PAGE_SIZE, GFP_KERNEL);

		memory_usage_show_internal(sliceidx, buf);
		pr_info("Slice %d out of memory\n%s", sliceidx, buf);
		kfree(buf);
	}

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
	struct slice_info *pslice;

	cma_obj = to_drm_gem_hantro_obj(gemobj);

	drm_gem_free_mmap_offset(&cma_obj->base);

	if (cma_obj->flag & HANTRO_GEM_FLAG_EXPORT) {
		drm_gem_handle_delete(file_priv, cma_obj->handle);
		hantro_unref_drmobj(obj);
		return 0;
	}
	drm_gem_object_release(gemobj);
	drm_gem_handle_delete(file_priv, cma_obj->handle);
	pslice = getslicenode(cma_obj->sliceidx);
	if (!pslice) {
		return 0;
	}

	if (cma_obj->vaddr)
		dma_free_coherent(
			pslice->dev,
			cma_obj->base.size,
			cma_obj->vaddr,
			cma_obj->paddr);

#ifdef DMA_DEBUG_ALLOC
	pr_info("%s:%d,%lx:%llx:%lx\n", __func__, cma_obj->sliceidx, (unsigned long)pslice->dev, cma_obj->paddr, cma_obj->base.size);
#endif

	dma_resv_fini(&cma_obj->kresv);
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
	struct slice_info *pslice;
	struct device *dev;

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

	if (!obj) {
		mutex_unlock(&hantro_dev.drm_dev->struct_mutex);
		return -EINVAL;
	}
	hantro_unref_drmobj(obj);
	cma_obj = to_drm_gem_hantro_obj(obj);

	if (page_num > cma_obj->num_pages) {
		mutex_unlock(&hantro_dev.drm_dev->struct_mutex);
		return -EINVAL;
	}
	if ((cma_obj->flag & HANTRO_GEM_FLAG_IMPORT) == 0) {
		pslice = getslicenode(cma_obj->sliceidx);
		if (!pslice) {
			mutex_unlock(&hantro_dev.drm_dev->struct_mutex);
			return -EINVAL;
		}
		dev = pslice->dev;
	} else
		dev = obj->dev->dev;

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
		dev,
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
    if (enable_decode)
	  hantrodec_open(inode, filp);
    if (enable_dec400)
	  cache_open(inode, filp);
    return ret;
}

static int hantro_device_release(struct inode *inode, struct file *filp)
{
    if (enable_dec400)
	  cache_release(filp);

    if (enable_decode)
	  hantrodec_release(filp);

    if (enable_encode)
	  hantroenc_release();

    return drm_release(inode, filp);
}

static vm_fault_t hantro_vm_fault(struct vm_fault *vmf)
{
	return -EPERM;
}

#ifndef virt_to_bus
static inline unsigned long virt_to_bus(void *address)
{
	return (unsigned long)address;
}
#endif
static struct sg_table *hantro_gem_prime_get_sg_table(
		struct drm_gem_object *obj)
{
	struct drm_gem_hantro_object *cma_obj = to_drm_gem_hantro_obj(obj);
	struct slice_info *pslice = getslicenode(cma_obj->sliceidx);
	struct sg_table *sgt;
	int ret;

	if (!pslice)
		return NULL;
	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return NULL;

	ret = dma_get_sgtable(pslice->dev, sgt, cma_obj->vaddr,
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
        cma_obj->meta_data_info = ((struct drm_gem_hantro_object *)attach->dmabuf->priv)->meta_data_info;

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
	if (mutex_lock_interruptible(&hantro_dev.drm_dev->struct_mutex))
		return -EBUSY;
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
	mutex_unlock(&hantro_dev.drm_dev->struct_mutex);
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
	struct slice_info *pslice;
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
		pslice = getslicenode(cma_obj->sliceidx);
		if (!pslice)
			return;
		dma_free_coherent(
			pslice->dev,
			cma_obj->base.size,
			cma_obj->vaddr,
			cma_obj->paddr);

#ifdef DMA_DEBUG_ALLOC
	pr_info("%s:%d,%lx:%llx:%lx\n", __func__, cma_obj->sliceidx, (unsigned long)pslice->dev, cma_obj->paddr, cma_obj->base.size);
#endif

	}
	dma_resv_fini(&cma_obj->kresv);
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
	pamap->vm_addr = (unsigned long long)cma_obj->vaddr;
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

static int hantro_add_client(
	struct drm_device *dev,
	void *data,
	struct drm_file *file_priv)
{
	int ret;
	struct hantro_client *attrib = data;
	struct file_data *file_attr = (struct file_data *)file_priv->driver_priv;
	struct hantro_client *client = kzalloc(sizeof(struct hantro_client), GFP_KERNEL);

	if (data == NULL)
		return -EINVAL;
	if (!client)
		return -ENOMEM;

	*client = *attrib;
	ret = idr_alloc(file_attr->clients, client, 1, 0, GFP_KERNEL);
	return (ret > 0 ? 0 : -ENOMEM);
}

static int hantro_remove_client(
	struct drm_device *dev,
	void *data,
	struct drm_file *file_priv)
{
	struct hantro_client *attrib = data;
	struct file_data *file_attr = (struct file_data *)file_priv->driver_priv;
	struct hantro_client *client = NULL;
	int id;

	if (data == NULL)
		return -EINVAL;

	idr_for_each_entry(file_attr->clients, client, id) {
		if (client && client->clientid == attrib->clientid) {
			idr_remove(file_attr->clients, id);
			kfree(client);
			break;
		}
	}

	return 0;
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
	struct file_data *data;
	data = kzalloc(sizeof(struct file_data), GFP_KERNEL);
	data->clients = kzalloc(sizeof(struct idr), GFP_KERNEL);
	data->list = kzalloc(sizeof(struct idr), GFP_KERNEL);
	if (data->clients == NULL || data->list == NULL)
		return -ENOMEM;
	idr_init(data->clients);
	idr_init(data->list);
	file->driver_priv = data;
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
	struct file_data *priv = (struct file_data *)file->driver_priv;
	struct hantro_client *client;
	void *obj;


	mutex_lock(&dev->struct_mutex);
	if (priv->list) {
		idr_for_each_entry(priv->list, obj, id) {
			if (obj) {
				hantro_release_dumb(dev, file, obj);
				idr_remove(priv->list, id);
			}
		}
		idr_destroy(priv->list);
		kfree(priv->list);
		priv->list = NULL;
	}

	if (priv->clients) {
		idr_for_each_entry(priv->clients, client, id) {
			if (obj) {
				kfree(client);
				idr_remove(priv->clients, id);
			}
		}
		idr_destroy(priv->clients);
		kfree(priv->clients);
		priv->clients = NULL;
	}

	kfree(priv);
	file->driver_priv = NULL;
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
	const struct drm_format_info *info =
		drm_get_format_info(dev, mode_cmd);
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
			mode_cmd->offsets[i] +	width * info->cpp[i];

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

static int hantro_query_metadata(
	struct drm_device *dev,
	void *data,
	struct drm_file *file_priv)
{
	struct hantro_exchanged_metadata_info *metadata_info_p = (struct hantro_exchanged_metadata_info *)data;
	struct drm_gem_object *obj = NULL;
	struct drm_gem_hantro_object *cma_obj = NULL;

	obj = hantro_gem_object_lookup(dev, file_priv, metadata_info_p->exporter.handle);
	if (obj == NULL)
		return -ENOENT;
	cma_obj = to_drm_gem_hantro_obj(obj);

	memcpy(&metadata_info_p->meta_data_info, &cma_obj->meta_data_info, sizeof(struct viv_vidmem_metadata_info));

	return 0;
}

static int hantro_update_metadata(
	struct drm_device *dev,
	void *data,
	struct drm_file *file_priv)
{
	struct hantro_exchanged_metadata_info *metadata_info_p = (struct hantro_exchanged_metadata_info *)data;
	struct drm_gem_object *obj = NULL;
	struct drm_gem_hantro_object *cma_obj = NULL;

	obj = hantro_gem_object_lookup(dev, file_priv, metadata_info_p->exporter.handle);
	if (obj == NULL)
		return -ENOENT;
	cma_obj = to_drm_gem_hantro_obj(obj);

	memcpy(&cma_obj->meta_data_info, &metadata_info_p->meta_data_info, sizeof(struct viv_vidmem_metadata_info));

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
	DRM_IOCTL_DEF(DRM_IOCTL_HANTRO_QUERY_METADATA, hantro_query_metadata, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_HANTRO_UPDATE_METADATA, hantro_update_metadata, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_HANTRO_GET_SLICENUM, hantro_get_slicenum, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_HANTRO_ADD_CLIENT, hantro_add_client, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
	DRM_IOCTL_DEF(DRM_IOCTL_HANTRO_REMOVE_CLIENT, hantro_remove_client, DRM_CONTROL_ALLOW | DRM_UNLOCKED),
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
	char stack_kdata[256];
	char *kdata = stack_kdata;
	unsigned int in_size, out_size;

	if (drm_dev_is_unplugged(dev))
		return -ENODEV;

	out_size = in_size = _IOC_SIZE(cmd);


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
		if (enable_encode) {
		      return hantroenc_ioctl(filp, cmd, arg);
		} else {
		if (cmd == HX280ENC_IOCG_CORE_NUM) {
			int corenum = 0;
			__put_user(corenum, (unsigned int *) arg);
		} else {
		     return -EFAULT;
		}
	}
    }
	if (nr >= DRM_IOCTL_NR(HANTRODEC_IOC_START) &&
		nr <= DRM_IOCTL_NR(HANTRODEC_IOC_END)) {
		if (enable_decode)
			return hantrodec_ioctl(filp, cmd, arg);
		else
		      return -EFAULT;
	}

	if (nr >= DRM_IOCTL_NR(HANTROCACHE_IOC_START) &&
		nr <= DRM_IOCTL_NR(HANTROCACHE_IOC_END)) {
	    if (enable_dec400)
		    return hantrocache_ioctl(filp, cmd, arg);
	    else
			return -EFAULT;
	}

	if (nr >= DRM_IOCTL_NR(HANTRODEC400_IOC_START) &&
		nr <= DRM_IOCTL_NR(HANTRODEC400_IOC_END)) {
	    if (enable_dec400)
		   return hantrodec400_ioctl(filp, cmd, arg);
	    else
		   return -EFAULT;
	}

	if (nr >= DRM_IOCTL_NR(HANTROSLICE_IOC_START) &&
		nr <= DRM_IOCTL_NR(HANTROSLICE_IOC_END)) {
		return hantroslice_ioctl(filp, cmd, arg);
	}

	if (nr >= HANTRO_IOCTL_COUNT)
		return -EINVAL;
	ioctl = &hantro_ioctls[nr];

	if((cmd == DRM_IOCTL_HANTRO_UPDATE_METADATA) ||
		(cmd == DRM_IOCTL_HANTRO_QUERY_METADATA) ) {
	if (copy_from_user(kdata, (void __user *)arg, sizeof(struct hantro_exchanged_metadata_info)) != 0)
		return  -EFAULT;
	} else {
		if (copy_from_user(kdata, (void __user *)arg, in_size) != 0)
			return  -EFAULT;
	}

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

	if((cmd == DRM_IOCTL_HANTRO_UPDATE_METADATA) ||
		(cmd == DRM_IOCTL_HANTRO_QUERY_METADATA) ) {
		if (copy_to_user((void __user *)arg, kdata, sizeof(struct hantro_exchanged_metadata_info)) != 0) {
			retcode = -EFAULT;
			}
	} else {
		if (copy_to_user((void __user *)arg, kdata, out_size) != 0)
			retcode = -EFAULT;
	}

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

static void hantro_gem_vm_close(struct vm_area_struct *vma)
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

}

static void hantro_gem_dmabuf_release(struct dma_buf *dma_buf)
{
	return drm_gem_dmabuf_release(dma_buf);
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
	.mmap = hantro_gem_dmabuf_mmap,
	.vmap = hantro_gem_dmabuf_vmap,
	.vunmap = drm_gem_dmabuf_vunmap,
};

static struct drm_driver hantro_drm_driver;
static struct dma_buf *hantro_prime_export(
	struct drm_gem_object *obj,
	int flags)
{
	struct drm_gem_hantro_object *cma_obj;
	struct dma_buf_export_info exp_info = {
		.exp_name = KBUILD_MODNAME,
		.owner = obj->dev->driver->fops->owner,
		.ops = &hantro_dmabuf_ops,
		.flags = flags,
		.priv = obj,
	};

	cma_obj = to_drm_gem_hantro_obj(obj);
	exp_info.resv = &cma_obj->kresv;
	exp_info.size = cma_obj->num_pages << PAGE_SHIFT;
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
	.release = hantro_release,
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
	u32 bandwidth;
	int sliceidx = findslice_bydev(kdev);

	if (sliceidx < 0)
		return 0;
	bandwidth = hantrodec_readbandwidth(sliceidx, 1);
	return snprintf(buf, PAGE_SIZE, "%d\n", bandwidth);
}

static ssize_t
bandwidthDecWrite_show(struct device *kdev, struct device_attribute *attr, char *buf)
{
	u32 bandwidth;
	int sliceidx = findslice_bydev(kdev);

	if (sliceidx < 0)
		return 0;
	bandwidth = hantrodec_readbandwidth(sliceidx, 0);
	return snprintf(buf, PAGE_SIZE, "%d\n", bandwidth);
}

static ssize_t
bandwidthEncRead_show(struct device *kdev, struct device_attribute *attr, char *buf)
{
	u32 bandwidth;
	int sliceidx = findslice_bydev(kdev);

	if (sliceidx < 0)
		return 0;
	bandwidth = hantroenc_readbandwidth(sliceidx, 1);
	return snprintf(buf, PAGE_SIZE, "%d\n", bandwidth);
}

static ssize_t
bandwidthEncWrite_show(struct device *kdev, struct device_attribute *attr, char *buf)
{
	u32 bandwidth;
	int sliceidx = findslice_bydev(kdev);

	if (sliceidx < 0)
		return 0;
	bandwidth = hantroenc_readbandwidth(sliceidx, 0);
	return snprintf(buf, PAGE_SIZE, "%d\n", bandwidth);
}
static ssize_t
show_clients(struct device *kdev, struct device_attribute *attr, char *buf)
{

	struct drm_device *ddev = hantro_dev.drm_dev;
	struct drm_file *file;
	struct file_data *data = NULL;
	int handle, buf_used = 0, client_count = 0;
	bool noprint = false;
	struct hantro_client *client;
	int sliceidx = -1;
	static char optype[64];
	static char profile[64];
	static char *unkown= "Unknown";
	static char *optypes[] =
	{
	    "Decode",
	    "Encode"
	};

	static char *profiles[]=
	{
	    /** \brief Profile ID used for video processing. */
	    "VAProfileMPEG2Simple",
	    "VAProfileMPEG2Main",
	    "VAProfileMPEG4Simple",
	    "VAProfileMPEG4AdvancedSimple",
	    "VAProfileMPEG4Main",
	    "VAProfileH264Baseline",
	    "VAProfileH264Main",
	    "VAProfileH264High",
	    "VAProfileVC1Simple",
	    "VAProfileVC1Main",
	    "VAProfileVC1Advanced",
	    "VAProfileH263Baseline",
	    "VAProfileJPEGBaseline",
	    "VAProfileH264ConstrainedBaseline",
	    "VAProfileVP8Version0_3",
	    "VAProfileH264MultiviewHigh",
	    "VAProfileH264StereoHigh",
	    "VAProfileHEVCMain",
	    "VAProfileHEVCMain10",
	    "VAProfileVP9Profile0",
	    "VAProfileVP9Profile1",
	    "VAProfileVP9Profile2",
	    "VAProfileVP9Profile3"
	};

	sliceidx = findslice_bydev(kdev);
	buf_used += snprintf(buf + buf_used, PAGE_SIZE, "  File Id  : ContextId : Slice :  Operation   :           Codec             :   Resolution  \n");
	mutex_lock(&ddev->filelist_mutex);
	// Go through all open drm files
	list_for_each_entry(file, &ddev->filelist, lhead) {
		mutex_lock(&ddev->struct_mutex);
		// Traverse through cma objects added to file's driver_priv
		// checkout hantro_recordmem
		data = (struct file_data *)file->driver_priv;
		if (data && data->clients) {
			idr_for_each_entry(data->clients, client, handle) {
				 if (client && client->sliceid == sliceidx) {
					 if (buf_used < (PAGE_SIZE - 200)) {
						 if (client->profile >= 0 && client->profile <= 22)
							strncpy(profile, profiles[client->profile], strlen(profiles[client->profile]));
						 else
							strncpy(profile, unkown, strlen(unkown));

						 if (client->codec == 0 || client->codec ==1)
							strncpy(optype, optypes[client->codec], strlen(optypes[client->codec]));
						 else
							strncpy(optype, unkown, strlen(unkown));
						 buf_used += snprintf(buf + buf_used, PAGE_SIZE, "%10p  %10x %5d\t  %s (%d)\t %20s (%d)\t%ldx%ld\n", file, client->clientid, client->sliceid,  optype, client->codec,  profile, client->profile, client->width, client->height);
					 } else {
						 // optimization to save buf space due to a PAGE_SIZE mem only
						if (noprint == false) {
							 buf_used += snprintf(buf + buf_used, PAGE_SIZE, " ....\n");
							 noprint = true; //print ... only one time
						}
					}
					 client_count++;
				 }
			}
		}
		mutex_unlock(&ddev->struct_mutex);
	}

	mutex_unlock(&ddev->filelist_mutex);
	buf_used += snprintf(buf + buf_used, PAGE_SIZE, "\n%d clients\n\n",  client_count);

	return buf_used;
}


static ssize_t
memory_usage_show_internal(int sliceidx, char *buf)
{

	struct drm_device *ddev = hantro_dev.drm_dev;
	struct drm_file *file;
	struct file_data *data;
	struct drm_gem_hantro_object *cma_obj;
	int buf_used = 0, alloc_count = 0;
	ssize_t mem_used = 0;
	bool noprint = false;

	buf_used = snprintf(buf, PAGE_SIZE, "Memory usage for slice %d:\n", sliceidx);
	buf_used += snprintf(buf + buf_used, PAGE_SIZE, "Physical Addr: Size\n");

	mutex_lock(&ddev->filelist_mutex);
	// Go through all open drm files
	list_for_each_entry(file, &ddev->filelist, lhead) {
		struct drm_gem_object *gobj;
		int handle;
		mutex_lock(&ddev->struct_mutex);
		// Traverse through cma objects added to file's driver_priv
		// checkout hantro_recordmem
		data = (struct file_data *)file->driver_priv;
		idr_for_each_entry(data->list, gobj, handle) {
			if (gobj) {
				cma_obj = to_drm_gem_hantro_obj(gobj);
				if (cma_obj && cma_obj->sliceidx ==  sliceidx) {
					if (buf_used < (PAGE_SIZE - 200)) {
						 buf_used += snprintf(buf + buf_used, PAGE_SIZE, "0x%-11llx: %ldK\n", cma_obj->paddr, cma_obj->base.size/1024);
					 } else {
						 // optimization to save buf space due to a PAGE_SIZE mem only
						if (noprint == false) {
							 buf_used += snprintf(buf + buf_used, PAGE_SIZE, " ....\n");
							 noprint = true; //print ... only one time
						}
					}

					   mem_used += cma_obj->base.size;
					   alloc_count++;
				}
			}
		}
		mutex_unlock(&ddev->struct_mutex);
	}

	mutex_unlock(&ddev->filelist_mutex);
	buf_used += snprintf(buf + buf_used, PAGE_SIZE, "\n%ldK in %d allocations\n\n",  mem_used/1024, alloc_count);

	return buf_used;
}



static ssize_t
memory_usage_show(struct device *kdev, struct device_attribute *attr, char *buf)
{
	int sliceidx = findslice_bydev(kdev);
	return memory_usage_show_internal(sliceidx, buf);
}

static DEVICE_ATTR(BWDecRead, 0444, bandwidthDecRead_show, NULL);
static DEVICE_ATTR(BWDecWrite, 0444, bandwidthDecWrite_show, NULL);
static DEVICE_ATTR(BWEncRead, 0444, bandwidthEncRead_show, NULL);
static DEVICE_ATTR(BWEncWrite, 0444, bandwidthEncWrite_show, NULL);
static DEVICE_ATTR(mem_usage, 0444, memory_usage_show, NULL);
static DEVICE_ATTR(clients, 0444, show_clients, NULL);

static struct attribute *hantro_attrs[] = {
	&dev_attr_BWDecRead.attr,
	&dev_attr_BWDecWrite.attr,
	&dev_attr_BWEncRead.attr,
	&dev_attr_BWEncWrite.attr,
	&dev_attr_mem_usage.attr,
	&dev_attr_clients.attr,
	NULL,
};

static const struct attribute_group hantro_attr_group = {
    .attrs = hantro_attrs,
 };

#define DEFINE_HANTRO_DEBUGFS_SEQ_FOPS(__prefix)                          \
static int __prefix ## _open(struct inode *inode, struct file *file)    \
{                                   \
    return single_open(file, __prefix ## _show, inode->i_private);  \
}                                   \
static const struct file_operations __prefix ## _fops = {       \
    .owner = THIS_MODULE,                       \
    .open = __prefix ## _open,                  \
    .release = single_release,                  \
    .read = seq_read,                       \
    .llseek = seq_lseek,                        \
}

static int mem_usage_show(struct seq_file *s, void *v)
{

    seq_printf(s, "Hello World\n");

    return 0;
}


DEFINE_HANTRO_DEBUGFS_SEQ_FOPS(mem_usage);

void create_debugfs(int slice)
{
    struct dentry *root;
    root = debugfs_create_dir("hantro",
            NULL);

    debugfs_create_file("mem_usage", S_IFREG | S_IRUGO,
            root,
            NULL, &mem_usage_fops);
}



static int getnodetype(const char *name)
{
	if (strstr(name, NODENAME_DECODER) == name)
		return CORE_DEC;
	if (strstr(name, NODENAME_ENCODER) == name)
		return CORE_ENC;
	if (strstr(name, NODENAME_CACHE) == name)
		return CORE_CACHE;
	if (strstr(name, NODENAME_DEC400) == name)
		return CORE_DEC400;
	return CORE_UNKNOWN;
}

static dtbnode *trycreatenode(
	struct platform_device *pdev,
	struct device_node *ofnode,
	int sliceidx,
	int parenttype,
	phys_addr_t parentaddr
	)
{
	struct fwnode_handle *fwnode;
	struct resource r;
	int i, na, ns, ret = 0;
	int endian = of_device_is_big_endian(ofnode);
	u32 reg_u32[4];
	const char *reg_name;
	uint64_t ioaddress, iosize;

	dtbnode *pnode = kzalloc(sizeof(dtbnode), GFP_KERNEL);
	if (!pnode)
		return NULL;

	pnode->type = getnodetype(ofnode->name);
	pnode->parentaddr = parentaddr;
	pnode->parenttype = parenttype;
	pnode->sliceidx = sliceidx;
	pnode->ofnode = ofnode;
	fwnode = &ofnode->fwnode;

	na = of_n_addr_cells(ofnode);
	ns = of_n_size_cells(ofnode);
	if (na > 2 || ns > 2) {
		pr_err("cell size too big");
		kfree(pnode);
		return NULL;
	}

	fwnode_property_read_u32_array(fwnode, "reg", reg_u32, na+ns);
	if (na == 2) {
		if (!endian) {
			ioaddress = reg_u32[0];
			ioaddress <<= 32;
			ioaddress |= reg_u32[1];
		} else {
			ioaddress = reg_u32[1];
			ioaddress <<= 32;
			ioaddress |= reg_u32[0];
		}
	} else
		ioaddress = reg_u32[0];
	if (ns == 2) {
		if (!endian) {
			iosize = reg_u32[na];
			iosize <<= 32;
			iosize |= reg_u32[na + 1];
		} else {
			iosize = reg_u32[na + 1];
			iosize <<= 32;
			iosize |= reg_u32[na];
		}
	} else
		iosize = reg_u32[na];
	pnode->ioaddr = ioaddress;
	pnode->iosize = iosize;
	fwnode_property_read_string(fwnode, "reg-names", &reg_name);

	if (strlen(reg_name))
	     strcpy(pnode->reg_name, reg_name);
	else
	     strcpy(pnode->reg_name, "hantro_reg");

	for (i = 0; i < 4; i++) {
		if (of_irq_to_resource(ofnode, i, &r) > 0) {
			//int irq = platform_get_irq_byname(pdev, r.name);
			pnode->irq[i] = r.start;
			if (strlen(r.name))
				strcpy(pnode->irq_name[i], r.name);
			else
				strcpy(pnode->irq_name[i], "hantro_irq");
			DBG("irq %s: mapping = %lld\n", r.name, r.end);
		} else
			pnode->irq[i] = -1;
	}

	switch (pnode->type) {
	case CORE_DEC:
		if (enable_decode)
		     ret = hantrodec_probe(pnode);
		break;
	case CORE_ENC:
		if (enable_encode)
		     ret = hantroenc_probe(pnode);
		break;
	case CORE_CACHE:
		if (enable_dec400)
		     ret = cache_probe(pnode);
		break;
	case CORE_DEC400:
		if (enable_dec400)
		     ret = hantro_dec400_probe(pnode);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (ret < 0) {
		kfree(pnode);
		pnode = NULL;
	}

	return pnode;
}

/* hantro_mmu_control is used to check whether media MMU is enabled or disabled */
static void hantro_mmu_control(struct platform_device *pdev)
{
        struct device *dev = &pdev->dev;
	u64 tbh_mmu_tcu_smmu_cr0;
	u8 *tbh_mmu_tcu_smmu_cr0_register;
	int is_mmu_enabled, count = 0;

        count = device_property_read_u64 (dev, "mmu-tcu-reg", &tbh_mmu_tcu_smmu_cr0);

        if (count == 0) {
	        /* request mem region of TBH_MMU_TCU_SMMU_CR0 */
		if (!request_mem_region(tbh_mmu_tcu_smmu_cr0, 0x32,"tbh_mmu_tcu_smmu_cr0")) {
			pr_info("tbh_mmu_tcu_smmu_cr0: failed to request mem region\n");
		}

		/* ioremap to the TBH_MMU_TCU_SMMU_CR0 */
		tbh_mmu_tcu_smmu_cr0_register = (u8 *) ioremap(tbh_mmu_tcu_smmu_cr0, 0x32);
		if (tbh_mmu_tcu_smmu_cr0_register == NULL) {
			pr_info("tbh_mmu_tcu_smmu_cr0_register: failed to ioremap tbh_mmu_tcu_smmu_cr0_register\n");
		}
		else {
			is_mmu_enabled = ioread32((void *)(tbh_mmu_tcu_smmu_cr0_register));
			if (is_mmu_enabled)
				printk("hantro_init: Media MMU600 is enabled, is_mmu_enabled = %d\n", is_mmu_enabled);
			else
				printk("hantro_init: Media MMU600 is disabled, is_mmu_enabled = %d\n", is_mmu_enabled);
			release_mem_region(tbh_mmu_tcu_smmu_cr0, 0x32);
		}
	}
}

/* hantro_enable_clock to enable/disable the media SS clocks */
static int hantro_clock_control(struct platform_device *pdev, bool enable)
{
	struct device *dev = &pdev->dev;
	struct clk *dev_clk = NULL;
	const char **clock_names;
	int i = 0, ret = 0, count = 0;

	// Read clock names
	count = device_property_read_string_array(dev, "clock-names", NULL, 0);
	if (count > 0) {
		clock_names = kcalloc(count, sizeof(*clock_names), GFP_KERNEL);
		if (!clock_names)
			return 0;
		ret = device_property_read_string_array(dev, "clock-names",
						clock_names, count);
		if (ret < 0) {
			pr_err("failed to read clock names\n");
			kfree(clock_names);
			return 0;
		}

		for (i = 0; i < count; i++) {
			DBG("hantro: clock_name = %s\n", clock_names[i]);
			dev_clk = clk_get(&pdev->dev, clock_names[i]);
			if (enable == true)  {
			      clk_prepare_enable(dev_clk);
			      if (verbose)
				   pr_info("hantro: default clock frequency of clock_name = %s is %ld\n", clock_names[i], clk_get_rate(dev_clk));
			      //clk_set_rate(dev_clk, 800000000);
			      //pr_info("hantro: set 800 Mhz clock frequency of clock_name = %s is %ld\n", clock_names[i], clk_get_rate(dev_clk));
			} else {
			     clk_disable_unprepare(dev_clk);
			}
		}
		kfree(clock_names);
	}

	return 0;
}

/* hantro_reset_clock to de-assert/assert the media SS cores and MMU */
static int hantro_reset_control(struct platform_device *pdev, bool deassert)
{

	struct device *dev = &pdev->dev;
	const char **reset_names;
	struct reset_control *dev_reset = NULL;
	int i = 0, ret = 0, count = 0;
	// Read reset names
	count = device_property_read_string_array(dev, "reset-names", NULL, 0);

	if (count > 0) {
		reset_names = kcalloc(count, sizeof(*reset_names), GFP_KERNEL);
		if (!reset_names)
			return 0;

		ret = device_property_read_string_array(dev, "reset-names",
						reset_names, count);
		if (ret < 0) {
			pr_err("failed to read clock names\n");
			kfree(reset_names);
			return 0;
		}

		for (i = 0; i < count; i++) {
			if (verbose)
			    pr_info("hantro: reset_name = %s\n", reset_names[i]);
			dev_reset = devm_reset_control_get(dev, reset_names[i]);
			if (deassert == true) {
				ret = reset_control_deassert(dev_reset);
				if (ret < 0) {
					pr_err("failed to deassert reset : %s, %d\n", reset_names[i], ret);
				}
			} else {
				ret = reset_control_assert(dev_reset);
				if (ret < 0) {
					pr_err("failed to assert reset : %s, %d\n", reset_names[i], ret);
				}
			}

		}
		kfree(reset_names);
	}

	return 0;
}

static int hantro_analyze_subnode(
	struct platform_device *pdev,
	struct device_node *slice,
	int sliceidx)
{
	dtbnode *head, *nhead, *newtail, *node;

	head = kzalloc(sizeof(dtbnode), GFP_KERNEL);
	if (!head)
		return -ENOMEM;
	head->type = head->parenttype = CORE_SLICE;
	head->ofnode = slice;
	head->dev = &pdev->dev;
	head->ioaddr = -1;
	head->iosize = 0;
	head->next = NULL;
	/*this is a wide first tree structure iteration, result is stored in slice info*/
	while (head != NULL) {
		nhead = newtail = NULL;
		while (head != NULL) {
			struct device_node *child, *ofnode = head->ofnode;
			for_each_child_of_node(ofnode, child) {
				node = trycreatenode(pdev, child, sliceidx, head->type, head->ioaddr);
				if (node) {
					if (nhead == NULL)
						nhead = newtail = node;
					else
						newtail->next = node;
					node->next = NULL;
					newtail = node;
				}
			}
			node = head->next;
			kfree(head);
			head = node;
		}
		head = nhead;
	}

	return 0;
}

static int hantro_drm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int result = 0;
	int sliceidx = -1;
	pr_info("hantro_drm_probe: dev %s probe\n", pdev->name);

        /*TBH PO:  We have to enable and set hantro clocks first before de-asserting the reset of media SS cores and MMU */
	hantro_clock_control(pdev, true);
	hantro_reset_control(pdev, true);

	/* Check the status of media MMU, whether it is enabled/disabled after reset de-assert of MMU */
	hantro_mmu_control(pdev);

	if (dev->of_node) {
		//probe from system DTB
		/*try to attach 1st rsv mem to dtb node*/
		result = of_reserved_mem_device_init(dev);

		dma_set_mask(dev, DMA_BIT_MASK(48));
		dma_set_coherent_mask(dev, DMA_BIT_MASK(48));

		if (result == 0)
			sliceidx = addslice(dev, -1, 0);
		else
			sliceidx = addslice(NULL, -1, 0);	//leave to end of init, set to default drm platform dev and default cma area

		/*go throug all sub dtb node' resources */
		if (sliceidx >= 0 && dev->of_node != NULL)
			hantro_analyze_subnode(pdev, dev->of_node, sliceidx);
	}

	return 0;
}


static int hantro_drm_remove(struct platform_device *pdev)
{

	// disabled for now
	/* TBH PO: Have to assert reset before disabling clocks */
	/* TBH PO: For now do not assert reset yet until power isolation sequence is implemented */
	// hantro_reset_control(pdev, false);

	hantro_clock_control(pdev, false);

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
	{ .compatible = "thunderbay,hantro",},
/*
#ifdef HAS_VC8000D
	{ .compatible = "platform-vc8000d", },
#endif
#ifdef HAS_VC8000E
	{ .compatible = "platform-vc8000e", },
#endif
*/
	{/* sentinel */}
};

static int hantro_pm_suspend(struct device *kdev)
{
	//what should we do on HW? Disable IRQ, slow down clock,
	// disable DMA, etc?
	//DBG("\nhantro suspend");
	return 0;
}

static int hantro_pm_resume(struct device *kdev)
{
	//DBG("\nhantro resume");
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
	.dma_mask	= DMA_BIT_MASK(48),
};


void __exit hantro_cleanup(void)
{
	hantro_dev.config = 0;

	if (enable_decode)
	      hantrodec_cleanup();

	if (enable_encode)
	      hantroenc_cleanup();

	if (enable_dec400)
	      cache_cleanup();

	if (enable_dec400)
	      hantro_dec400_cleanup();
	
/*this one must be after above ones to maintain list*/
	slice_remove();
	releaseFenceData();
	// reserved mem relese need to be called somewhere
	// of_reserved_mem_device_release(sinfo->dev);
	drm_dev_unregister(hantro_dev.drm_dev);
	platform_device_unregister(hantro_dev.platformdev);
	platform_driver_unregister(&hantro_drm_platform_driver);
    drm_dev_put(hantro_dev.drm_dev);
	pr_info("hantro driver removed\n");

}

int __init hantro_init(void)
{
	int result, i;

	/*_init functions will init static vairables, while probe will init dynamic emelemts from DTB */
	/*slice init must be in first to clear list*/
	slice_init();
	hantroenc_init();
	hantrodec_init();
	cache_init();
	hantrodec400_init();
	hantro_dev.config = 0;

	result = platform_driver_register(&hantro_drm_platform_driver);
	if (result < 0) {
		pr_info("hantro create platform driver fail");
		return result;
	}

	/*this is not a DTB node related platform device. Use it for drmdev's root node only*/
	hantro_dev.platformdev = platform_device_register_full(&hantro_platform_info);
	if (hantro_dev.platformdev == NULL) {
		platform_driver_unregister(&hantro_drm_platform_driver);
		pr_info("hantro create platform device fail");
		return PTR_ERR(hantro_dev.platformdev);
	}

	/*it must be here instead of in probe*/
	hantro_dev.drm_dev = drm_dev_alloc(&hantro_drm_driver, &hantro_dev.platformdev->dev);
	if (IS_ERR(hantro_dev.drm_dev)) {
		pr_info("init drm failed\n");
		platform_device_unregister(hantro_dev.platformdev);
		platform_driver_unregister(&hantro_drm_platform_driver);
		return PTR_ERR(hantro_dev.drm_dev);
	}

	hantro_dev.drm_dev->dev = &hantro_dev.platformdev->dev;
	drm_mode_config_init(hantro_dev.drm_dev);
	result  = drm_dev_register(hantro_dev.drm_dev, 0);
        if (result < 0) {
		drm_dev_unregister(hantro_dev.drm_dev);
		platform_device_unregister(hantro_dev.platformdev);
		platform_driver_unregister(&hantro_drm_platform_driver);
        drm_dev_put(hantro_dev.drm_dev);
		return result;
	}
	initFenceData();

	if (get_slicenumber() == 0)
		addslice(hantro_dev.drm_dev->dev, -1, 0);	//for PC, no DTB probe, create a default dev
	for (i = 0; i < get_slicenumber(); i++) {
		struct slice_info *pslice = getslicenode_inInit(i);
		hantro_dev.config |= pslice->config;
		if (pslice->dev == NULL)
			pslice->dev = hantro_dev.drm_dev->dev;

		result = devm_device_add_group(pslice->dev, &hantro_attr_group);

		if (result != 0)
			pr_info("create sysfs %d fail\n", i);

	}

        create_debugfs(0);
	slice_init_finish();
	if (verbose)
	     slice_printdebug();
	pr_info("hantro device created\n");
	return result;
}

module_init(hantro_init);
module_exit(hantro_cleanup);

/* module description */
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Verisilicon");
MODULE_DESCRIPTION("Hantro DRM manager");
