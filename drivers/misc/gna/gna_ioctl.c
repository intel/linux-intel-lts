// SPDX-License-Identifier: GPL-2.0-only
// Copyright(c) 2017-2020 Intel Corporation

#include <linux/uaccess.h>

#include "gna.h"

#include "gna_drv.h"
#include "gna_ioctl.h"
#include "gna_mem.h"
#include "gna_request.h"
#include "gna_score.h"

static int gna_ioctl_score(
	struct gna_file_private *file_priv, void __user *argptr)
{
	union gna_compute score_args;
	u64 request_id;
	struct gna_private *gna_priv;
	int ret;

	gna_priv = file_priv->gna_priv;

	if (copy_from_user(&score_args, argptr, sizeof(score_args))) {
		dev_err(&gna_priv->dev,
			"could not copy score ioctl config from user\n");
		return -EFAULT;
	}

	ret = gna_validate_score_config(&score_args.in.config, file_priv);
	if (ret) {
		dev_err(&gna_priv->dev, "request not valid\n");
		return ret;
	}

	ret = gna_request_enqueue(&score_args.in.config, file_priv, &request_id);
	if (ret) {
		dev_err(&gna_priv->dev, "could not enqueue score request %d\n",
			ret);
		return ret;
	}

	score_args.out.request_id = request_id;
	if (copy_to_user(argptr, &score_args, sizeof(score_args))) {
		dev_err(&gna_priv->dev,
			"could not copy score ioctl status to user\n");
		return -EFAULT;
	}

	return 0;
}

static int gna_ioctl_wait(struct file *f, void __user *argptr)
{
	enum gna_request_state request_state;
	struct gna_file_private *file_priv;
	struct gna_request *score_request;
	struct gna_private *gna_priv;
	union gna_wait wait_data;
	u64 request_id;
	u32 timeout;
	int ret;

	file_priv = (struct gna_file_private *) f->private_data;
	gna_priv = file_priv->gna_priv;

	ret = 0;

	if (copy_from_user(&wait_data, argptr, sizeof(wait_data))) {
		dev_err(&gna_priv->dev,
			"could not copy wait ioctl data from user\n");
		return -EFAULT;
	}

	request_id = wait_data.in.request_id;
	timeout = wait_data.in.timeout;

	score_request = gna_find_request_by_id(request_id, gna_priv);

	if (!score_request) {
		dev_err(&gna_priv->dev,
			"could not find request with id: %llu\n",
			request_id);
		return -EINVAL;
	}

	if (score_request->fd != f) {
		kref_put(&score_request->refcount, gna_request_release);
		return -EINVAL;
	}

	dev_dbg(&gna_priv->dev, "found request %llu in the queue\n",
		request_id);

	spin_lock_bh(&score_request->state_lock);
	request_state = score_request->state;
	spin_unlock_bh(&score_request->state_lock);

	if (request_state == DONE) {
		dev_dbg(&gna_priv->dev, "request already done, excellent\n");
		goto copy_request_result;
	}

	dev_dbg(&gna_priv->dev, "waiting for request %llu for timeout %u\n",
		request_id, timeout);

	ret = gna_score_wait(score_request, timeout);
	if (ret == 0 || ret == -ERESTARTSYS) {
		dev_err(&gna_priv->dev,
			"request timed out, id: %llu\n", request_id);
		kref_put(&score_request->refcount, gna_request_release);
		return -EBUSY;
	}

copy_request_result:
	dev_dbg(&gna_priv->dev, "request wait completed with %d req id %llu\n",
		ret, request_id);

	spin_lock_bh(&score_request->perf_lock);
	wait_data.out.hw_perf = score_request->hw_perf;
	wait_data.out.drv_perf = score_request->drv_perf;
	spin_unlock_bh(&score_request->perf_lock);

	spin_lock_bh(&score_request->hw_lock);
	wait_data.out.hw_status = score_request->hw_status;
	spin_unlock_bh(&score_request->hw_lock);

	spin_lock_bh(&score_request->status_lock);
	ret = score_request->status;
	spin_unlock_bh(&score_request->status_lock);

	dev_dbg(&gna_priv->dev, "request status %d, hw status: %#x\n",
			score_request->status, score_request->hw_status);
	kref_put(&score_request->refcount, gna_request_release);

	gna_delete_request_by_id(request_id, gna_priv);

	if (copy_to_user(argptr, &wait_data, sizeof(wait_data))) {
		dev_err(&gna_priv->dev,
			"could not copy wait ioctl status to user\n");
		ret = -EFAULT;
	}

	return ret;
}

static int gna_ioctl_userptr(struct gna_file_private *file_priv,
	void __user *argptr)
{
	struct gna_private *gna_priv;
	union gna_memory_map gna_mem;
	int ret;

	gna_priv = file_priv->gna_priv;

	if (copy_from_user(&gna_mem, argptr, sizeof(gna_mem))) {
		dev_err(&gna_priv->dev,
			"could not copy userptr ioctl data from user\n");
		return -EFAULT;
	}

	dev_dbg(&gna_priv->dev, "userptr size %d address %p\n",
			gna_mem.in.size, (void *)gna_mem.in.address);

	ret = gna_priv->ops->userptr(file_priv, &gna_mem);
	if (ret)
		return ret;

	if (copy_to_user(argptr, &gna_mem, sizeof(gna_mem))) {
		dev_err(&gna_priv->dev,
			"could not copy userptr ioctl status to user\n");
		return -EFAULT;
	}

	return 0;
}

static int gna_ioctl_free(struct gna_file_private *file_priv, unsigned long arg)
{
	struct gna_memory_object *mo;
	struct gna_private *gna_priv;
	struct gna_memory_object *iter_mo, *temp_mo;

	u64 memory_id = arg;

	gna_priv = file_priv->gna_priv;

	dev_dbg(&gna_priv->dev, "memory id %llu\n", memory_id);

	/* get kernel space memory pointer */
	mutex_lock(&gna_priv->memidr_lock);
	mo = idr_find(&gna_priv->memory_idr, memory_id);
	mutex_unlock(&gna_priv->memidr_lock);

	if (!mo) {
		dev_warn(&gna_priv->dev, "memory object not found\n");
		return -EINVAL;
	}

	queue_work(gna_priv->request_wq, &mo->work);
	if (wait_event_interruptible(mo->waitq, true)) {
		dev_dbg(&gna_priv->dev, "wait interrupted\n");
		return -ETIME;
	}

	mutex_lock(&file_priv->memlist_lock);
	list_for_each_entry_safe(iter_mo, temp_mo,
				 &file_priv->memory_list, file_mem_list) {
		if (iter_mo->memory_id == memory_id) {
			list_del(&iter_mo->file_mem_list);
			break;
		}
	}
	mutex_unlock(&file_priv->memlist_lock);

	gna_memory_free(gna_priv, mo);

	return 0;
}

static int gna_ioctl_getparam(struct gna_private *gna_priv, void __user *argptr)
{
	union gna_parameter param;
	int ret;

	if (copy_from_user(&param, argptr, sizeof(param))) {
		dev_err(&gna_priv->dev,
			"could not copy getparam ioctl data from user\n");
		return -EFAULT;
	}

	ret = gna_priv->ops->getparam(gna_priv, &param);
	if (ret)
		return ret;

	if (copy_to_user(argptr, &param, sizeof(param))) {
		dev_err(&gna_priv->dev,
			"could not copy getparam ioctl status to user\n");
		return -EFAULT;
	}

	return 0;
}

long gna_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	struct gna_file_private *file_priv;
	struct gna_private *gna_priv;
	void __user *argptr;
	void *data;
	u64 size;
	int ret;

	argptr = (void __user *) arg;
	data = NULL;
	size = 0;
	ret = -EINVAL;

	file_priv = (struct gna_file_private *) f->private_data;
	if (!file_priv)
		return -ENODEV;

	gna_priv = file_priv->gna_priv;
	if (!gna_priv)
		return -ENODEV;

	dev_dbg(&gna_priv->dev, "%s: enter cmd %#x\n", __func__, cmd);

	switch (cmd) {

	case GNA_IOCTL_PARAM_GET:

		dev_dbg(&gna_priv->dev, "%s: GNA_IOCTL_PARAM_GET\n", __func__);
		ret = gna_ioctl_getparam(gna_priv, argptr);
		break;

	case GNA_IOCTL_MEMORY_MAP:

		dev_dbg(&gna_priv->dev, "%s: GNA_IOCTL_MEMORY_MAP\n", __func__);
		ret = gna_ioctl_userptr(file_priv, argptr);
		break;

	case GNA_IOCTL_MEMORY_UNMAP:
		dev_dbg(&gna_priv->dev, "%s: GNA_IOCTL_MEMORY_UNMAP\n", __func__);
		ret = gna_ioctl_free(file_priv, arg);
		break;

	case GNA_IOCTL_COMPUTE:
		dev_dbg(&gna_priv->dev, "%s: GNA_IOCTL_COMPUTE\n", __func__);
		ret = gna_ioctl_score(file_priv, argptr);
		break;

	case GNA_IOCTL_WAIT:
		dev_dbg(&gna_priv->dev, "%s: GNA_IOCTL_WAIT\n", __func__);
		ret = gna_ioctl_wait(f, argptr);
		break;

	default:
		dev_warn(&gna_priv->dev, "wrong ioctl %#x\n", cmd);
		ret = -ENOTTY;
		break;
	}

	dev_dbg(&gna_priv->dev, "%s: exit\n", __func__);

	return ret;
}
