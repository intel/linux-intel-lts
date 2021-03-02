// SPDX-License-Identifier: GPL-2.0
/*
 *    Hantro driver DMA_BUF fence operation.
 *
 *    Copyright (c) 2017 - 2020, VeriSilicon Inc.
 *    Copyright (c) 2020 - 2021, Intel Corporation
 */

#include "hantro_priv.h"

/* fence related code */
/* for fence's seqno, maybe each domain should hold one */
static unsigned long seqno;
DEFINE_IDR(fence_idr);
/* fence mutex struct */
struct mutex fence_mutex;

static const char *hantro_fence_get_driver_name(hantro_fence_t *fence)
{
	return "hantro";
}

static const char *hantro_fence_get_timeline_name(hantro_fence_t *fence)
{
	return " "; /* it should correspond to six domains later */
}

static bool hantro_fence_enable_signaling(hantro_fence_t *fence)
{
	if (test_bit(HANTRO_FENCE_FLAG_ENABLE_SIGNAL_BIT, &fence->flags))
		return true;
	else
		return false;
}

static bool hantro_fence_signaled(hantro_fence_t *fobj)
{
	unsigned long irqflags;
	bool ret;

	spin_lock_irqsave(fobj->lock, irqflags);
	ret = (test_bit(HANTRO_FENCE_FLAG_SIGNAL_BIT, &fobj->flags) != 0);
	spin_unlock_irqrestore(fobj->lock, irqflags);
	return ret;
}

static void hantro_fence_free(hantro_fence_t *fence)
{
	kfree(fence->lock);
	fence->lock = NULL;
	dma_fence_free(fence);
}

const static hantro_fence_op_t hantro_fenceops = {
	.get_driver_name = hantro_fence_get_driver_name,
	.get_timeline_name = hantro_fence_get_timeline_name,
	.enable_signaling = hantro_fence_enable_signaling,
	.signaled = hantro_fence_signaled,
	.wait = hantro_fence_default_wait,
	.release = hantro_fence_free,
};

static hantro_fence_t *alloc_fence(unsigned int ctxno)
{
	hantro_fence_t *fobj;
	/* spinlock for fence */
	spinlock_t *lock;

	fobj = kzalloc(sizeof(hantro_fence_t), GFP_KERNEL);
	if (!fobj)
		return NULL;

	lock = kzalloc(sizeof(*lock), GFP_KERNEL);
	if (!lock) {
		kfree(fobj);
		return NULL;
	}

	spin_lock_init(lock);
	hantro_fence_init(fobj, &hantro_fenceops, lock, ctxno, seqno++);
	clear_bit(HANTRO_FENCE_FLAG_SIGNAL_BIT, &fobj->flags);
	set_bit(HANTRO_FENCE_FLAG_ENABLE_SIGNAL_BIT, &fobj->flags);
	return fobj;
}

static int is_hantro_fence(hantro_fence_t *fence)
{
	return (fence->ops == &hantro_fenceops);
}

int init_hantro_resv(struct dma_resv *presv,
		     struct drm_gem_hantro_object *cma_obj)
{
	dma_resv_init(presv);
	cma_obj->ctxno = hantro_fence_context_alloc(1);

	return 0;
}

int hantro_waitfence(hantro_fence_t *pfence)
{
	if (test_bit(HANTRO_FENCE_FLAG_SIGNAL_BIT, &pfence->flags))
		return 0;

	if (is_hantro_fence(pfence))
		/* need self check */
		return 0;
	else
		return hantro_fence_wait_timeout(pfence, true, 30 * HZ);
}

/* it's obsolete, left here for compiling compatible */
int hantro_setdomain(struct drm_device *dev, void *data,
		     struct drm_file *file_priv)
{
	return 0;
}

void init_fence_data(void)
{
	seqno = 0;
	mutex_init(&fence_mutex);
	idr_init(&fence_idr);
}

static int fence_idr_fini(int id, void *p, void *data)
{
	hantro_fence_signal(p);
	hantro_fence_put(p);
	return 0;
}

void release_fence_data(void)
{
	mutex_lock(&fence_mutex);
	idr_for_each(&fence_idr, fence_idr_fini, NULL);
	idr_destroy(&fence_idr);
	mutex_unlock(&fence_mutex);
}

int hantro_acquirebuf(struct drm_device *dev, void *data,
		      struct drm_file *file_priv)
{
	struct hantro_acquirebuf *arg = data;
	struct dma_resv *resv;
	struct drm_gem_object *obj;
	struct drm_gem_hantro_object *cma_obj;
	hantro_fence_t *fence = NULL;
	unsigned long timeout = arg->timeout;
	unsigned long fenceid = -1;
	int ret = 0;

	START_TIME;
	obj = hantro_gem_object_lookup(dev, file_priv, arg->handle);
	if (!obj) {
		ret = -ENOENT;
		trace_fence_acquirebuf(0x0, arg->handle, -1, 0, ret);
		return ret;
	}

	if (!obj->dma_buf) {
		if (hantro_drm.drm_dev == obj->dev) {
			struct drm_gem_hantro_object *hobj =
				to_drm_gem_hantro_obj(obj);

			resv = &hobj->kresv;
		} else {
			ret = -ENOENT;
			goto err;
		}

	} else {
		resv = obj->dma_buf->resv;
	}

	/* Check for a stalled fence */
	if (!dma_resv_wait_timeout_rcu(resv, arg->flags & HANTRO_FENCE_WRITE, 1,
				       timeout)) {
		ret = -EBUSY;
		goto err;
	}

	/* Expose the fence via the dma-buf */
	ret = -ENOMEM;
	fence = alloc_fence(hantro_fence_context_alloc(1));
	if (!fence)
		goto err;

	mutex_lock(&fence_mutex);
	ret = idr_alloc(&fence_idr, fence, 1, 0, GFP_KERNEL);
	mutex_unlock(&fence_mutex);
	if (ret >= 0)
		fenceid = ret;
	else
		goto err;

	dma_resv_lock(resv, NULL);
	ret = 0;
	if (arg->flags & HANTRO_FENCE_WRITE) {
		dma_resv_add_excl_fence(resv, fence);
	} else {
		/*I'm not sure if 1 fence is enough, pass compilation first*/
		ret = hantro_reserve_obj_shared(resv, 1);
		if (ret == 0)
			dma_resv_add_shared_fence(resv, fence);
	}

	dma_resv_unlock(resv);

	/* Record the fence in our idr for later signaling */
	if (ret == 0) {
		arg->fence_handle = fenceid;
		goto out;
	}

err:
	if (fenceid >= 0) {
		mutex_lock(&fence_mutex);
		idr_remove(&fence_idr, fenceid);
		mutex_unlock(&fence_mutex);
	}

	if (fence) {
		hantro_fence_signal(fence);
		hantro_fence_put(fence);
	}

out:
	cma_obj = (struct drm_gem_hantro_object *)obj;
	trace_fence_acquirebuf((void *)cma_obj->paddr, arg->handle,
			       arg->fence_handle,
			       (sched_clock() - start) / 1000, ret);
	hantro_unref_drmobj(obj);
	return ret;
}

int hantro_testbufvalid(struct drm_device *dev, void *data,
			struct drm_file *file_priv)
{
	struct hantro_fencecheck *arg = data;
	struct dma_resv *resv;
	struct drm_gem_object *obj;

	arg->ready = 0;
	obj = hantro_gem_object_lookup(dev, file_priv, arg->handle);
	if (!obj)
		return -ENOENT;

	if (!obj->dma_buf) {
		if (hantro_drm.drm_dev == obj->dev) {
			struct drm_gem_hantro_object *hobj =
				to_drm_gem_hantro_obj(obj);

			resv = &hobj->kresv;
		} else {
			hantro_unref_drmobj(obj);
			return -ENOENT;
		}

	} else {
		resv = obj->dma_buf->resv;
	}

	/* Check for a stalled fence */
	if (dma_resv_wait_timeout_rcu(resv, 1, 1, 0) <= 0)
		arg->ready = 0;
	else
		arg->ready = 1;

	hantro_unref_drmobj(obj);
	return 0;
}

int hantro_releasebuf(struct drm_device *dev, void *data,
		      struct drm_file *file_priv)
{
	struct hantro_releasebuf *arg = data;
	hantro_fence_t *fence;
	int ret = 0;

	mutex_lock(&fence_mutex);
	fence = idr_replace(&fence_idr, NULL, arg->fence_handle);
	mutex_unlock(&fence_mutex);

	if (!fence || IS_ERR(fence))
		return -ENOENT;

	if (hantro_fence_is_signaled(fence))
		ret = -ETIMEDOUT;

	trace_fence_releasebuf(arg->fence_handle, ret);
	hantro_fence_signal(fence);
	hantro_fence_put(fence);
	mutex_lock(&fence_mutex);
	idr_remove(&fence_idr, arg->fence_handle);
	mutex_unlock(&fence_mutex);
	return ret;
}
