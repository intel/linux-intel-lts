// SPDX-License-Identifier: GPL-2.0-only
// Copyright(c) 2017-2020 Intel Corporation

#include <linux/device.h>
#include <linux/kref.h>
#include <linux/pm_runtime.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include "gna_drv.h"
#include "gna_request.h"

/* processes request and starts the device */
static void gna_request_process(struct work_struct *work)
{
	struct gna_request *score_request;
	struct gna_memory_object *mo;
	struct gna_private *gna_priv;
	struct gna_buffer *buffer;
	int ret;
	u64 i;

	score_request = container_of(work, struct gna_request, work);
	gna_priv = score_request->gna_priv;
	dev_dbg(&gna_priv->dev,
		"processing request %llu\n", score_request->request_id);

	spin_lock_bh(&gna_priv->busy_lock);
	if (gna_priv->busy) {
		spin_unlock_bh(&gna_priv->busy_lock);
		dev_dbg(&gna_priv->dev, "gna device is busy, wait\n");
		wait_event(gna_priv->busy_waitq, !gna_priv->busy);
		spin_lock_bh(&gna_priv->busy_lock);
	}
	gna_priv->busy = true;
	spin_unlock_bh(&gna_priv->busy_lock);

	spin_lock_bh(&score_request->state_lock);
	score_request->state = ACTIVE;
	spin_unlock_bh(&score_request->state_lock);

	spin_lock_bh(&score_request->perf_lock);
	score_request->drv_perf.pre_processing = ktime_get_ns();
	spin_unlock_bh(&score_request->perf_lock);

	ret = pm_runtime_get_sync(&gna_priv->pdev->dev);
	if (ret < 0) {
		dev_warn_once(&gna_priv->dev,
			"pm_runtime_get_sync() failed: %d\n", ret);
		gna_request_set_done(score_request, -ENODEV);
		goto end;
	}

	ret = gna_priv->ops->score(score_request);
	if (ret) {
		if (pm_runtime_put(&gna_priv->pdev->dev) < 0)
			dev_warn_once(&gna_priv->dev,
				"pm_runtime_put() failed: %d\n", ret);
		goto end;
	}

	spin_lock_bh(&score_request->perf_lock);
	score_request->drv_perf.processing = ktime_get_ns();
	spin_unlock_bh(&score_request->perf_lock);

	wait_event(gna_priv->busy_waitq, !gna_priv->busy);

	/* request post-processing */
	buffer = score_request->buffer_list;
	for (i = 0; i < score_request->buffer_count; i++, buffer++) {
		mutex_lock(&gna_priv->memidr_lock);
		mo = idr_find(&gna_priv->memory_idr, buffer->memory_id);
		mutex_unlock(&gna_priv->memidr_lock);
		if (mo) {
			dev_dbg(&gna_priv->dev, "putting pages %llu\n",
					buffer->memory_id);
			mutex_lock(&mo->page_lock);
			mo->ops->put_pages(mo);
			mutex_unlock(&mo->page_lock);
		} else {
			dev_warn(&gna_priv->dev, "mo not found %llu\n",
					buffer->memory_id);
		}
	}

	/* patches_ptr's are already freed by ops->score() function */
	kvfree(score_request->buffer_list);
	score_request->buffer_list = NULL;
	score_request->buffer_count = 0;

	gna_mmu_clear(gna_priv);

	dev_dbg(&gna_priv->dev, "request %llu done, waking processes\n",
		score_request->request_id);
	spin_lock_bh(&score_request->state_lock);
	score_request->state = DONE;
	spin_unlock_bh(&score_request->state_lock);
	wake_up_interruptible_all(&score_request->waitq);

end:
	spin_lock_bh(&score_request->perf_lock);
	score_request->drv_perf.completion = ktime_get_ns();
	spin_unlock_bh(&score_request->perf_lock);
}

struct gna_request *gna_request_create(
	struct gna_file_private *file_priv,
	struct gna_compute_cfg *compute_cfg)
{
	struct gna_request *score_request;
	struct gna_private *gna_priv;

	gna_priv = file_priv->gna_priv;
	if (IS_ERR(gna_priv))
		return NULL;

	score_request = kzalloc(sizeof(*score_request), GFP_KERNEL|GFP_ATOMIC);
	if (!score_request)
		return NULL;
	kref_init(&score_request->refcount);

	dev_dbg(&gna_priv->dev, "layer_base %d layer_count %d\n",
		compute_cfg->layer_base, compute_cfg->layer_count);

	score_request->request_id = atomic_inc_return(&gna_priv->request_count);
	score_request->compute_cfg = *compute_cfg;
	score_request->fd = file_priv->fd;
	score_request->gna_priv = gna_priv;
	score_request->state = NEW;
	spin_lock_init(&score_request->state_lock);
	spin_lock_init(&score_request->status_lock);
	spin_lock_init(&score_request->perf_lock);
	spin_lock_init(&score_request->hw_lock);
	init_waitqueue_head(&score_request->waitq);
	INIT_WORK(&score_request->work, gna_request_process);

	return score_request;
}

void gna_request_release(struct kref *ref)
{
	struct gna_request *score_request =
		container_of(ref, struct gna_request, refcount);
	kfree(score_request);
}

void gna_request_set_done(struct gna_request *score_request, int status)
{
	spin_lock_bh(&score_request->status_lock);
	score_request->status = status;
	spin_unlock_bh(&score_request->status_lock);

	spin_lock_bh(&score_request->state_lock);
	score_request->state = DONE;
	spin_unlock_bh(&score_request->state_lock);
}

struct gna_request *gna_find_request_by_id(u64 req_id,
	struct gna_private *gna_priv)
{
	struct list_head *reqs_list;
	struct gna_request *iter_req, *found_req;

	mutex_lock(&gna_priv->reqlist_lock);

	reqs_list = &gna_priv->request_list;
	found_req = NULL;
	if (!list_empty(reqs_list)) {
		list_for_each_entry(iter_req, reqs_list, node) {
			if (req_id == iter_req->request_id) {
				found_req = iter_req;
				kref_get(&found_req->refcount);
				break;
			}
		}
	}

	mutex_unlock(&gna_priv->reqlist_lock);

	return found_req;
}

void gna_delete_request_by_id(u64 req_id, struct gna_private *gna_priv)
{
	struct list_head *reqs_list;
	struct gna_request *temp_req, *iter_req;

	mutex_lock(&gna_priv->reqlist_lock);

	reqs_list = &gna_priv->request_list;
	if (!list_empty(reqs_list)) {
		list_for_each_entry_safe(iter_req, temp_req, reqs_list, node) {
			if (iter_req->request_id == req_id) {
				list_del(&iter_req->node);
				cancel_work_sync(&iter_req->work);
				kref_put(&iter_req->refcount, gna_request_release);
				break;
			}
		}
	}

	mutex_unlock(&gna_priv->reqlist_lock);
}

void gna_delete_file_requests(struct file *fd, struct gna_private *gna_priv)
{
	struct list_head *reqs_list;
	struct gna_request *temp_req, *iter_req;

	mutex_lock(&gna_priv->reqlist_lock);

	reqs_list = &gna_priv->request_list;
	if (!list_empty(reqs_list)) {
		list_for_each_entry_safe(iter_req, temp_req, reqs_list, node) {
			if (iter_req->fd == fd) {
				list_del(&iter_req->node);
				cancel_work_sync(&iter_req->work);
				kref_put(&iter_req->refcount, gna_request_release);
				break;
			}
		}
	}

	mutex_unlock(&gna_priv->reqlist_lock);
}

void gna_delete_memory_requests(u64 memory_id, struct gna_private *gna_priv)
{
	struct list_head *reqs_list;
	struct gna_request *temp_req, *iter_req;
	int i;

	mutex_lock(&gna_priv->reqlist_lock);

	reqs_list = &gna_priv->request_list;
	if (!list_empty(reqs_list)) {
		list_for_each_entry_safe(iter_req, temp_req, reqs_list, node) {
			for (i = 0; i < iter_req->buffer_count; ++i) {
				if (iter_req->buffer_list[i].memory_id == memory_id) {
					BUG_ON(iter_req->state == NEW);
					list_del(&iter_req->node);
					cancel_work_sync(&iter_req->work);
					kref_put(&iter_req->refcount, gna_request_release);
					break;
				}
			}
		}
	}

	mutex_unlock(&gna_priv->reqlist_lock);
}
