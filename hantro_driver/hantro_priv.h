/*
 *    Hantro driver private header file.
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
#ifndef HANTRO_PRIV_H
#define HANTRO_PRIV_H
#include "hantro.h"
#include "hantro_slice.h"

//#define USE_IRQ
//#define USE_DTB_PROBE

#define IRQNAME_DECODER		"irq_hantro_decoder"

#define RESNAME_VIDEOENC	"regs_videoencoder"
#define RESNAME_JPGENC		"regs_jpgencoder"
#define IRQNAME_VENC		"irq_hantro_videoencoder"
#define IRQNAME_JPGENC		"irq_hantro_jpgencoder"

#define HANTRO_GEM_FLAG_IMPORT    (1 << 0)
#define HANTRO_GEM_FLAG_EXPORT    (1 << 1)
#define HANTRO_GEM_FLAG_EXPORTUSED    (1 << 2)

#if KERNEL_VERSION(5, 0, 0) <= LINUX_VERSION_CODE
#define hantro_access_ok(a, b, c)	access_ok(b, c)
#define hantro_reserve_obj_shared(a, b)	dma_resv_reserve_shared(a, b)
#define hantro_ref_drmobj		drm_gem_object_get
#define hantro_unref_drmobj		drm_gem_object_put_unlocked
#else	/*KERNEL_VERSION(5, 0, 0) <= LINUX_VERSION_CODE*/
#define hantro_access_ok(a, b, c)	access_ok(a, b, c)
#define hantro_reserve_obj_shared(a, b)	reservation_object_reserve_shared(a)
#define hantro_ref_drmobj	drm_gem_object_reference
#define hantro_unref_drmobj		drm_gem_object_unreference_unlocked
#endif

#define RESNAME_DECODER		"decoder_"
#define RESNAME_ENCODER		"encoder_"
#define RESNAME_CACHE			"cache_"

struct hantro_core_info {
	struct resource *mem;
	struct resource *irqlist[4];		/*Fix me: suggest each core has at most 4 irqs.*/
	int irqnum;
};

struct hantro_device_handle {
	struct platform_device *platformdev;	/* parent device */
	struct drm_device *drm_dev;
	u32 config;
};
extern struct hantro_device_handle hantro_dev;

#if KERNEL_VERSION(4, 13, 0) > LINUX_VERSION_CODE

#define HANTRO_FENCE_FLAG_ENABLE_SIGNAL_BIT  FENCE_FLAG_ENABLE_SIGNAL_BIT
#define HANTRO_FENCE_FLAG_SIGNAL_BIT                 FENCE_FLAG_SIGNALED_BIT

typedef struct fence hantro_fence_t;
typedef struct fence_ops hantro_fence_op_t;

static inline signed long hantro_fence_default_wait(
	hantro_fence_t *fence,
	bool intr,
	signed long timeout)
{
	return fence_default_wait(fence, intr, timeout);
}

static inline void hantro_fence_free(hantro_fence_t *fence)
{
	fence_free(fence);
}

static inline  void hantro_fence_init(
	hantro_fence_t *fence,
	const hantro_fence_op_t *ops,
	spinlock_t *lock,
	unsigned int context,
	unsigned int seqno)
{
	return fence_init(fence, ops, lock, context, seqno);
}

static inline  unsigned int hantro_fence_context_alloc(unsigned int num)
{
	return fence_context_alloc(num);
}

static inline  signed long hantro_fence_wait_timeout(
	hantro_fence_t *fence,
	bool intr,
	signed long timeout)
{
	return fence_wait_timeout(fence, intr, timeout);
}

static inline  struct drm_gem_object *hantro_gem_object_lookup(
	struct drm_device *dev,
	struct drm_file *filp,
	u32 handle)
{
#if KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE
	return drm_gem_object_lookup(filp, handle);
#else
	return drm_gem_object_lookup(dev, filp, handle);
#endif
}

static inline  void hantro_fence_put(hantro_fence_t *fence)
{
	return fence_put(fence);
}

static inline  int hantro_fence_signal(hantro_fence_t *fence)
{
	return fence_signal(fence);
}

static inline  void ref_page(struct page *pp)
{
#if KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE
	atomic_inc(&pp->_refcount);
#else
	atomic_inc(&pp->_count);
#endif
}

static inline  void unref_page(struct page *pp)
{
#if KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE
	atomic_dec(&pp->_refcount);
#else
	atomic_dec(&pp->_count);
#endif
}

static inline  bool hantro_fence_is_signaled(hantro_fence_t *fence)
{
	return fence_is_signaled(fence);
}

#else	/*version higher */
#define HANTRO_FENCE_FLAG_ENABLE_SIGNAL_BIT  DMA_FENCE_FLAG_ENABLE_SIGNAL_BIT
#define HANTRO_FENCE_FLAG_SIGNAL_BIT                DMA_FENCE_FLAG_SIGNALED_BIT

typedef struct dma_fence hantro_fence_t;
typedef struct dma_fence_ops hantro_fence_op_t;

static inline  signed long hantro_fence_default_wait(
	hantro_fence_t *fence,
	bool intr,
	signed long timeout)
{
	return dma_fence_default_wait(fence, intr, timeout);
}

static inline  void hantro_fence_free(hantro_fence_t *fence)
{
	dma_fence_free(fence);
}

static inline  void hantro_fence_init(
	hantro_fence_t *fence,
	const hantro_fence_op_t *ops,
	spinlock_t *lock,
	unsigned int context,
	unsigned int seqno)
{
	return dma_fence_init(fence, ops, lock, context, seqno);
}

static inline  unsigned int hantro_fence_context_alloc(unsigned int num)
{
	return dma_fence_context_alloc(num);
}

static inline  signed long hantro_fence_wait_timeout(
	hantro_fence_t *fence,
	bool intr,
	signed long timeout)
{
	return dma_fence_wait_timeout(fence, intr, timeout);
}

static inline struct drm_gem_object *hantro_gem_object_lookup(
	struct drm_device *dev,
	struct drm_file *filp,
	u32 handle)
{
	return drm_gem_object_lookup(filp, handle);
}

static inline  void hantro_fence_put(hantro_fence_t *fence)
{
	return dma_fence_put(fence);
}

static inline  int hantro_fence_signal(hantro_fence_t *fence)
{
	return dma_fence_signal(fence);
}

static inline  void ref_page(struct page *pp)
{
	atomic_inc(&pp->_refcount);
	atomic_inc(&pp->_mapcount);
}

static inline  void unref_page(struct page *pp)
{
	atomic_dec(&pp->_refcount);
	atomic_dec(&pp->_mapcount);
}

static inline  bool hantro_fence_is_signaled(hantro_fence_t *fence)
{
	return dma_fence_is_signaled(fence);
}

#endif

static inline struct drm_gem_hantro_object *
to_drm_gem_hantro_obj(struct drm_gem_object *gem_obj)
{
	return container_of(gem_obj, struct drm_gem_hantro_object, base);
}

int hantro_setdomain(
	struct drm_device *dev,
	void *data,
	struct drm_file *file_priv);
int hantro_acquirebuf(
	struct drm_device *dev,
	void *data,
	struct drm_file *file_priv);
int hantro_testbufvalid(
	struct drm_device *dev,
	void *data,
	struct drm_file *file_priv);
int hantro_releasebuf(
	struct drm_device *dev,
	void *data,
	struct drm_file *file_priv);
int init_hantro_resv(
	struct dma_resv *presv,
	struct drm_gem_hantro_object *cma_obj);
void initFenceData(void);
void releaseFenceData(void);


#endif  /*HANTRO_PRIV_H*/

