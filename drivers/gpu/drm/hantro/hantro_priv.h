/* SPDX-License-Identifier: GPL-2.0 */
/*
 *    Hantro driver private header file.
 *
 *    Copyright (c) 2017 - 2020, VeriSilicon Inc.
 *    Copyright (c) 2020 - 2021, Intel Corporation
 */

#ifndef __HANTRO_PRIV_H__
#define __HANTRO_PRIV_H__
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
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/semaphore.h>
#include <linux/atomic.h>
#include <linux/spinlock.h>
#include <linux/ioctl.h>
#include <linux/pci.h>
#include <linux/ioport.h>
#include <linux/version.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_irq.h>
#include <linux/reset.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/thermal.h>

#include <linux/ioctl.h>
#include <linux/dma-resv.h>
#include <linux/dma-mapping.h>
#include <drm/drm_vma_manager.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem.h>
#include <linux/dma-buf.h>
#include <drm/drm_auth.h>
#include <linux/version.h>
#include <linux/dma-fence.h>
#include <linux/sched/clock.h>
#include <linux/dma-mapping.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_legacy.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <drm/drm_modeset_helper.h>

#include "hantro_drm.h"
#include "hantro_devicemgr.h"
#include "hantro_metadata.h"

#include "trace.h"

#define HANTRO_GEM_FLAG_IMPORT		BIT(0)
#define HANTRO_GEM_FLAG_EXPORT		BIT(1)
#define HANTRO_GEM_FLAG_EXPORTUSED	BIT(2)
#define HANTRO_GEM_FLAG_FOREIGN_IMPORTED    BIT(3)

#define DRIVER_NAME	"hantro"
#define DRIVER_DESC	"hantro DRM"
#define DRIVER_DATE	"20210304"
#define DRIVER_MAJOR	1
#define DRIVER_MINOR	0

#define hantro_access_ok(a, b, c) access_ok(b, c)
#define hantro_reserve_obj_shared(a, b) dma_resv_reserve_shared(a, b)
#define hantro_ref_drmobj drm_gem_object_get
#define hantro_unref_drmobj drm_gem_object_put

#define NODENAME_DECODER	"decoder"
#define NODENAME_ENCODER	"encoder"
#define NODENAME_CACHE		"cache"
#define NODENAME_DEC400		"dec400"

#define PIXEL_CMA 0
#define CODEC_RESERVED 1

#undef PDEBUG /* undef it, just in case */
#ifdef __KERNEL__
/* This one if debugging is on, and kernel space */
#define PDEBUG(fmt, arg...)                                                    \
	do {                                                                   \
		if (verbose)                                                   \
			pr_info(fmt, ##arg);                                   \
	} while (0)

#else
/* This one for user space */
#define PDEBUG(fmt, args...) printf(__FILE__ ":%d: " fmt, __LINE__, ##args)
#endif

extern bool verbose;
extern bool enable_lut;
extern bool enable_encode;
extern bool enable_decode;
extern bool enable_dec400;
extern bool enable_irqmode;

extern long kmb_freq_table[3];
extern long tbh_freq_table[3];

extern const struct dma_buf_ops hantro_dmabuf_ops;

struct dtbnode {
	struct device_node *ofnode;
	int type;
	phys_addr_t ioaddr;
	phys_addr_t iosize;
	char reg_name[32];
	int irq[4];
	char irq_name[4][32];
	char clock_name[32];
	int parenttype;
	phys_addr_t parentaddr;
	int deviceidx;
	struct device_info *pdevinfo;
	struct dtbnode *next;
	struct device *dev;
};

struct hantro_drm_handle {
	struct platform_device *platformdev; /* parent device */
	struct device *dev;
	struct drm_device *drm_dev;
	struct device_info *pdevice_list;
	struct dentry *debugfs_root;
	struct class_compat *media_class;
	enum hantro_device_type device_type;
	u8 __iomem *dec_page_lut_regs;
	u8 __iomem *enc_page_lut_regs;
	atomic_t devicecount;
	/* hantro mutex struct */
	struct mutex hantro_mutex;
	u32 config;
};

#define HANTRO_FENCE_FLAG_ENABLE_SIGNAL_BIT	DMA_FENCE_FLAG_ENABLE_SIGNAL_BIT
#define HANTRO_FENCE_FLAG_SIGNAL_BIT		DMA_FENCE_FLAG_SIGNALED_BIT

extern struct hantro_drm_handle hantro_drm;

static inline struct drm_gem_object *
hantro_get_gem_from_dmabuf(struct dma_buf *dma_buf)
{
	struct drm_gem_hantro_object *cma_obj =
		(struct drm_gem_hantro_object
			 *)(((struct dmapriv *)dma_buf->priv)->self);

	if (cma_obj && cma_obj->dmapriv.magic_num == HANTRO_IMAGE_VIV_META_DATA_MAGIC)
		return &cma_obj->base;

	return NULL;
}

static inline signed long
hantro_fence_default_wait(struct dma_fence *fence, bool intr, signed long timeout)
{
	return dma_fence_default_wait(fence, intr, timeout);
}

static inline void hantro_fence_init(struct dma_fence *fence,
				     const struct dma_fence_ops *ops,
				     spinlock_t *lock, unsigned int context,
				     unsigned int seqno)
{
	return dma_fence_init(fence, ops, lock, context, seqno);
}

static inline unsigned int hantro_fence_context_alloc(unsigned int num)
{
	return dma_fence_context_alloc(num);
}

static inline struct drm_gem_object *
hantro_gem_object_lookup(struct drm_device *dev, struct drm_file *file,
			 u32 handle)
{
	return drm_gem_object_lookup(file, handle);
}

static inline void hantro_fence_put(struct dma_fence *fence)
{
	return dma_fence_put(fence);
}

static inline int hantro_fence_signal(struct dma_fence *fence)
{
	return dma_fence_signal(fence);
}

static inline void unref_page(struct page *pp)
{
	atomic_dec(&pp->_refcount);
	atomic_dec(&pp->_mapcount);
}

static inline bool hantro_fence_is_signaled(struct dma_fence *fence)
{
	return dma_fence_is_signaled(fence);
}

static inline struct drm_gem_hantro_object *
to_drm_gem_hantro_obj(struct drm_gem_object *gem_obj)
{
	return container_of(gem_obj, struct drm_gem_hantro_object, base);
}

int hantro_setdomain(struct drm_device *dev, void *data,
		     struct drm_file *file_priv);
int hantro_acquirebuf(struct drm_device *dev, void *data,
		      struct drm_file *file_priv);
int hantro_testbufvalid(struct drm_device *dev, void *data,
			struct drm_file *file_priv);
int hantro_releasebuf(struct drm_device *dev, void *data,
		      struct drm_file *file_priv);
int init_hantro_resv(struct dma_resv *presv,
		     struct drm_gem_hantro_object *cma_obj);
void create_debugfs(struct device_info *pdevinfo, bool has_codecmem);
int mem_usage_internal(unsigned int deviceidx, struct device *memdev,
		       u32 *pused_mem, u32 *pallocations, struct seq_file *s);

struct drm_device *create_hantro_drm(struct device *dev);
int create_sysfs(struct device_info *pdevinfo);
void remove_sysfs(struct device_info *pdevinfo);
struct device_info *get_device_info(int deviceid);
void init_fence_data(void);
void release_fence_data(void);

#endif /* __HANTRO_PRIV_H__ */
