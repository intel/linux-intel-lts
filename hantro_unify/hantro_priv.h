/* SPDX-License-Identifier: GPL-2.0 */
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

#ifndef __HANTRO_PRIV_H__
#define __HANTRO_PRIV_H__
#include "hantro.h"
#include "hantro_slice.h"
#include "trace.h"
#define HANTRO_GEM_FLAG_IMPORT		(1 << 0)
#define HANTRO_GEM_FLAG_EXPORT		(1 << 1)
#define HANTRO_GEM_FLAG_EXPORTUSED	(1 << 2)

#define hantro_access_ok(a, b, c) access_ok(b, c)
#define hantro_reserve_obj_shared(a, b) dma_resv_reserve_shared(a, b)
#define hantro_ref_drmobj drm_gem_object_get
#define hantro_unref_drmobj drm_gem_object_put

#define NODENAME_DECODER	"decoder"
#define NODENAME_ENCODER	"encoder"
#define NODENAME_CACHE		"cache"
#define NODENAME_DEC400		"dec400"

#define MAX_SLICES 4
#define NODE_TYPES 2  //encoder & decoder
#define MAX_CORES 4   // max core per slice

// framecount array to hold data for 4 slices
typedef struct performance_data {
        int count;
        u64 last_resv;
        u64 totaltime;
} performance_data;

extern struct performance_data perfdata[MAX_SLICES][NODE_TYPES + 1][MAX_CORES];

typedef struct dtbnode {
	struct device_node *ofnode;
	int type;
	phys_addr_t ioaddr;
	phys_addr_t iosize;
	char reg_name[32];
	int irq[4];
	char irq_name[4][32];
	int parenttype;
	phys_addr_t parentaddr;
	int sliceidx;
	struct dtbnode *next;
	struct device *dev;
} dtbnode;

struct hantro_device_handle {
	struct platform_device *platformdev; /* parent device */
	struct drm_device *drm_dev;
	u32 config;
	void *data;
	struct dentry *debugfs_root;
};
extern struct hantro_device_handle hantro_dev;

#define HANTRO_FENCE_FLAG_ENABLE_SIGNAL_BIT	DMA_FENCE_FLAG_ENABLE_SIGNAL_BIT
#define HANTRO_FENCE_FLAG_SIGNAL_BIT		DMA_FENCE_FLAG_SIGNALED_BIT

typedef struct dma_fence hantro_fence_t;
typedef struct dma_fence_ops hantro_fence_op_t;

static inline signed long
hantro_fence_default_wait(hantro_fence_t *fence, bool intr, signed long timeout)
{
	return dma_fence_default_wait(fence, intr, timeout);
}

static inline void hantro_fence_init(hantro_fence_t *fence,
				     const hantro_fence_op_t *ops,
				     spinlock_t *lock, unsigned int context,
				     unsigned int seqno)
{
	return dma_fence_init(fence, ops, lock, context, seqno);
}

static inline unsigned int hantro_fence_context_alloc(unsigned int num)
{
	return dma_fence_context_alloc(num);
}

static inline signed long
hantro_fence_wait_timeout(hantro_fence_t *fence, bool intr, signed long timeout)
{
	return dma_fence_wait_timeout(fence, intr, timeout);
}

static inline struct drm_gem_object *
hantro_gem_object_lookup(struct drm_device *dev, struct drm_file *filp,
			 u32 handle)
{
	return drm_gem_object_lookup(filp, handle);
}

static inline void hantro_fence_put(hantro_fence_t *fence)
{
	return dma_fence_put(fence);
}

static inline int hantro_fence_signal(hantro_fence_t *fence)
{
	return dma_fence_signal(fence);
}

static inline void ref_page(struct page *pp)
{
	atomic_inc(&pp->_refcount);
	atomic_inc(&pp->_mapcount);
}

static inline void unref_page(struct page *pp)
{
	atomic_dec(&pp->_refcount);
	atomic_dec(&pp->_mapcount);
}

static inline bool hantro_fence_is_signaled(hantro_fence_t *fence)
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
void initFenceData(void);
void releaseFenceData(void);

#endif /* __HANTRO_PRIV_H__ */
