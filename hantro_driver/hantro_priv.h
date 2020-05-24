/*
 *    Hantro driver private header file.
 *
 *    Copyright (c) 2017, VeriSilicon Inc.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License, version 2, as
 *    published by the Free Software Foundation.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License version 2 for more details.
 *
 *    You may obtain a copy of the GNU General Public License
 *    Version 2 at the following locations:
 *    https://opensource.org/licenses/gpl-2.0.php
 */

#ifndef HANTRO_PRIV_H
#define HANTRO_PRIV_H
#include "hantro.h"
#include "hantro_slice.h"

#define USE_IRQ
#define USE_DTB_PROBE

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

#define NODENAME_DECODER		"decoder"
#define NODENAME_ENCODER		"encoder"
#define NODENAME_CACHE		"cache"
#define NODENAME_DEC400		"dec400"

/* Reset control defines */
#define RESET_ID_TOTAL 19
#define RESET_DEASSERTED 0
#define RESET_ASSERTED 1

/* Clock control defines */
#define CLOCK_ID_TOTAL 8
#define CLOCK_DISABLED 0
#define CLOCK_ENABLED 1

typedef struct dtbnode {
	struct device_node *ofnode;
	int type;
	phys_addr_t ioaddr;
	phys_addr_t iosize;
	char reg_name[32];
	int irq[4];
        char irq_name[4][32];
	int reset_count;
	char reset_names[4][32];
	int clock_count;
	char clock_names[4][32];
	int parenttype;
	phys_addr_t parentaddr;
	int sliceidx;
	struct dtbnode *next;
	struct device *dev;
} dtbnode;

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

