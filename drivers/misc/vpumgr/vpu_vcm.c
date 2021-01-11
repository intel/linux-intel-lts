// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2021 Intel Corporation
 */
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/idr.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/sched/task.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/xlink.h>
#include "vpu_common.h"
#include "vpu_vcm.h"

#define XLINK_IPC_TIMEOUT           1000u

/* Static xlink configuration */
#define VCM_XLINK_CHANNEL           1
#define VCM_XLINK_CHAN_SIZE         128

static const int msg_header_size = offsetof(struct vcm_msg, payload.data);

struct vpu_cmd {
	struct work_struct work;
	struct kref refcount;
	struct xlink_handle *handle;
	struct vpumgr_ctx *vctx;         /* the submitting vpu context */
	struct vcm_msg msg;              /* message buffer for send/recv */
	struct completion complete;      /* completion for async submit/reply */
	int submit_err;                  /* error code for submittion process */
};

static int vcm_vpu_link_init(struct vcm_dev *pvcm)
{
	struct vpumgr_device *vdev = container_of(pvcm, struct vpumgr_device, vcm);
	enum xlink_error rc;

	pvcm->ipc_xlink_handle.dev_type = VPUIP_DEVICE;
	pvcm->ipc_xlink_handle.sw_device_id = pvcm->sw_dev_id;

	rc = xlink_initialize();
	if (rc != X_LINK_SUCCESS)
		goto exit;

	rc = xlink_connect(&pvcm->ipc_xlink_handle);
	if (rc != X_LINK_SUCCESS)
		goto exit;

	rc = 0;
exit:
	dev_info(vdev->dev, "%s: rc = %d\n", __func__, rc);
	return -(int)rc;
}

static int vcm_vpu_link_fini(struct vcm_dev *pvcm)
{
	xlink_disconnect(&pvcm->ipc_xlink_handle);
	return 0;
}

/*
 * Send a vcm_msg by xlink.
 * Given limited xlink payload size, packing is also performed.
 */
static int vcm_send(struct xlink_handle *xlnk_handle, struct vcm_msg *req)
{
	struct vpumgr_device *vdev;
	enum xlink_error rc;
	u8 *ptr = (u8 *)req;
	u32 size = 0;
	u32 len = req->size;

	vdev = container_of(xlnk_handle, struct vpumgr_device, vcm.ipc_xlink_handle);
	if (len > sizeof(*req))
		return -EINVAL;
	do {
		size = len > VCM_XLINK_CHAN_SIZE ? VCM_XLINK_CHAN_SIZE : len;
		rc = xlink_write_volatile(xlnk_handle, VCM_XLINK_CHANNEL, ptr, size);
		if (rc != X_LINK_SUCCESS) {
			dev_warn(vdev->dev, "%s xlink_write_volatile error %d\n", __func__, rc);
			return -EINVAL;
		}
		ptr += size;
		len -= size;
	} while (len > 0);

	return 0;
}

/*
 * Receives a vcm_msg by xlink.
 * Given limited xlink payload size, unpacking is also performed.
 */
static int vcm_recv(struct xlink_handle *xlnk_handle, struct vcm_msg *rep)
{
	struct vpumgr_device *vdev;
	enum xlink_error rc;
	u64 size;
	u32 total_size = 0;
	u32 rx_size = 0;
	u8 *ptr = (u8 *)rep;

	vdev = container_of(xlnk_handle, struct vpumgr_device, vcm.ipc_xlink_handle);
	do {
		/* workaround for a bug in xlink_read_data_to_buffer()
		 * although it's last argument is declared to be of type (u32 *), the
		 * function actually writes 64-bit value into that address.
		 */
		rc = xlink_read_data_to_buffer(xlnk_handle, VCM_XLINK_CHANNEL, ptr, (u32 *)&size);
		if (rc != X_LINK_SUCCESS) {
			if (!vdev->vcm.xlink_closing)
				dev_warn(vdev->dev,
					 "%s: xlink_read_data_to_buffer failed, rc:%d\n",
					 __func__, rc);
			return -EPIPE;
		}

		if (total_size == 0) {
			if (size < msg_header_size) {
				dev_warn(vdev->dev, "%s: first packet is too small (%llu)\n",
					 __func__, size);
				return -EINVAL;
			}

			total_size = rep->size;
			if (total_size > sizeof(*rep)) {
				dev_warn(vdev->dev, "%s: packet size (%u) is too big\n",
					 __func__, total_size);
				return -EINVAL;
			}
			if (total_size < size) {
				dev_warn(vdev->dev,
					 "%s: first packet is smaller than claimed (%llu)\n",
					 __func__, size);
				return -EINVAL;
			}
		}

		ptr += size;
		rx_size += size;
	} while (rx_size < total_size);

	if (rx_size != total_size) {
		dev_warn(vdev->dev, "%s: actuall size %u exceeds claimed size %ud\n",
			 __func__, rx_size, total_size);
		return -EINVAL;
	}

	return 0;
}

static void vcmd_free(struct kref *kref)
{
	struct vpu_cmd *vcmd = container_of(kref, struct vpu_cmd, refcount);

	kvfree(vcmd);
}

static struct vpu_cmd *vcmd_get(struct vcm_dev *pvcm, int msgid)
{
	struct vpu_cmd *vcmd;

	mutex_lock(&pvcm->msg_idr_lock);
	vcmd = idr_find(&pvcm->msg_idr, msgid);
	if (vcmd)
		kref_get(&vcmd->refcount);
	mutex_unlock(&pvcm->msg_idr_lock);

	return vcmd;
}

static void vcmd_put(struct vpu_cmd *vcmd)
{
	kref_put(&vcmd->refcount, vcmd_free);
}

static int vcmd_alloc_msgid(struct vcm_dev *pvcm, struct vpu_cmd *vcmd)
{
	int msgid;

	mutex_lock(&pvcm->msg_idr_lock);
	msgid = idr_alloc_cyclic(&pvcm->msg_idr, vcmd, 1, 0, GFP_KERNEL);
	if (msgid >= 0)
		kref_init(&vcmd->refcount);
	mutex_unlock(&pvcm->msg_idr_lock);
	return msgid;
}

static void vcmd_remove_msgid(struct vcm_dev *pvcm, int msgid)
{
	struct vpu_cmd *vcmd;

	mutex_lock(&pvcm->msg_idr_lock);
	vcmd = idr_remove(&pvcm->msg_idr, msgid);
	if (vcmd)
		kref_put(&vcmd->refcount, vcmd_free);
	mutex_unlock(&pvcm->msg_idr_lock);
}

static void vcmd_clean_ctx(struct vcm_dev *pvcm, struct vpumgr_ctx *v)
{
	struct vpu_cmd *vcmd;
	int i;

	mutex_lock(&pvcm->msg_idr_lock);
	idr_for_each_entry(&pvcm->msg_idr, vcmd, i)
		if (vcmd->vctx == v) {
			idr_remove(&pvcm->msg_idr, i);
			kref_put(&vcmd->refcount, vcmd_free);
		}
	mutex_unlock(&pvcm->msg_idr_lock);
}

static int vcmd_count_ctx(struct vcm_dev *pvcm, struct vpumgr_ctx *v)
{
	struct vpu_cmd *vcmd;
	int i;
	int count = 0;

	mutex_lock(&pvcm->msg_idr_lock);
	idr_for_each_entry(&pvcm->msg_idr, vcmd, i)
		if (vcmd->vctx == v)
			count++;
	mutex_unlock(&pvcm->msg_idr_lock);

	return count;
}

static void vpu_cmd_submit(struct work_struct *work)
{
	struct vpu_cmd *p = container_of(work, struct vpu_cmd, work);

	p->submit_err = vcm_send(p->handle, &p->msg);
}

/*
 * vcm_submit() - Submit a command to VPU
 * @v:         Pointer to local vpu context data structure.
 * @cmd:       Command code
 * @data_in:   Data arguments
 * @in_len:    Length of the data arguments
 * @submit_id: On return, this will containe a newly allocated
 *             vpu-device-wise unique ID for the submitted command
 *
 * Submit a command to corresponding vpu context running on firmware to execute
 */
int vcm_submit(struct vpumgr_ctx *v,
	       u32 cmd, const void *data_in, u32 in_len, s32 *submit_id)
{
	struct vcm_dev *pvcm = &v->vdev->vcm;
	int ctx = v->vpu_ctx_id;
	int rc = 0;
	struct vpu_cmd *vcmd;

	if (!v->vdev->vcm.enabled)
		return -ENOENT;

	if (in_len > VCM_PAYLOAD_SIZE)
		return -EINVAL;

	vcmd = kvmalloc(sizeof(*vcmd), GFP_KERNEL);
	if (!vcmd)
		return -ENOMEM;

	vcmd->vctx = v;

	rc = vcmd_alloc_msgid(pvcm, vcmd);
	if (rc < 0) {
		rc = -EEXIST;
		kvfree(vcmd);
		return rc;
	}

	/* from now on, vcmd can is refcount managed */
	vcmd->msg.id = rc;
	*submit_id = vcmd->msg.id;

	if (data_in && in_len > 0) {
		if (access_ok((void __user *)data_in, in_len)) {
			rc = copy_from_user(vcmd->msg.payload.data,
					    (const void __user *)data_in, in_len);
			if (rc)
				goto remove_msgid;
		} else {
			memcpy(vcmd->msg.payload.data, data_in, in_len);
		}
	}

	init_completion(&vcmd->complete);
	vcmd->handle = &pvcm->ipc_xlink_handle;
	vcmd->msg.size = msg_header_size + in_len;
	vcmd->msg.ctx = ctx;
	vcmd->msg.cmd = cmd;
	vcmd->msg.rc = 0;
	INIT_WORK(&vcmd->work, vpu_cmd_submit);

	if (!queue_work(pvcm->wq, &vcmd->work)) {
		rc = -EEXIST;
		goto remove_msgid;
	}

	return 0;

remove_msgid:
	vcmd_remove_msgid(pvcm, vcmd->msg.id);
	return rc;
}

/*
 * vcm_wait() - Wait a submitted command to finish
 * @v:          Pointer to local vpu context data structure.
 * @submit_id:  Unique ID of the submitted command to wait for
 * @vpu_rc:     Return code of the submitted commands
 * @data_out:   Return data payload of the submitted command
 * @p_out_len:  Length of the returned paylaod
 * @timeout_ms: Time in milliseconds before the wait expires
 *
 * Wait for a submitted command to finish and retrieves the
 * return code and outputs on success with timeout.
 */
int vcm_wait(struct vpumgr_ctx *v, s32 submit_id,
	     s32 *vpu_rc, void *data_out, u32 *p_out_len, u32 timeout_ms)
{
	struct vcm_dev *pvcm = &v->vdev->vcm;
	struct device *dev = v->vdev->sdev;
	unsigned long timeout = msecs_to_jiffies(timeout_ms);
	struct vpu_cmd *vcmd;
	int rc, len;

	if (!v->vdev->vcm.enabled)
		return -ENOENT;

	vcmd = vcmd_get(pvcm, submit_id);
	if (!vcmd) {
		dev_err(dev, "%s:cannot find submit_id %d\n", __func__, submit_id);
		return -EINVAL;
	}

	if (v != vcmd->vctx) {
		dev_err(dev, "%s:trying to wait on submit %d doesn't belong to vpu context %d\n",
			__func__, submit_id, v->vpu_ctx_id);
		return -EINVAL;
	}

	/* wait for submission work to be done */
	flush_work(&vcmd->work);
	rc = vcmd->submit_err;
	if (rc)
		goto exit;

	/* wait for reply */
	rc = wait_for_completion_interruptible_timeout(&vcmd->complete, timeout);
	if (rc < 0)
		goto exit;
	else if (rc == 0) {
		rc = -ETIMEDOUT;
		goto exit;
	} else {
		/* wait_for_completion_interruptible_timeout return positive
		 * rc on success, but we return zero as success.
		 */
		rc = 0;
	}

	if (vpu_rc)
		*vpu_rc = vcmd->msg.rc;

	if (data_out && p_out_len) {
		/* truncate payload to fit output buffer size provided */
		len = vcmd->msg.size - msg_header_size;
		if (len > (*p_out_len)) {
			dev_err(dev, "%s: output is truncated from %d to %d to fit buffer size.\n",
				__func__, len, (*p_out_len));
			len = (*p_out_len);
		}

		if (len > 0) {
			if (access_ok((void __user *)data_out, len)) {
				rc = copy_to_user((void __user *)data_out,
						  vcmd->msg.payload.data, len);
				if (rc)
					goto exit;
			} else {
				memcpy(data_out, vcmd->msg.payload.data, len);
			}
		}

		/* tell the caller the exact length received, even if
		 * we have truncated due to output buffer size limitation
		 */
		*p_out_len = len;
	}
exit:
	v->total_vcmds++;
	vcmd_put(vcmd);
	vcmd_remove_msgid(pvcm, submit_id);
	return rc;
}

static int vcm_call(struct vpumgr_ctx *v,
		    s32 cmd, const void *data_in, u32 in_len,
		    s32 *res_rc, void *data_out, u32 *p_out_len)
{
	int submit_id, rc;

	if (!v->vdev->vcm.enabled)
		return -ENOENT;

	rc = vcm_submit(v, cmd, data_in, in_len, &submit_id);
	if (rc)
		return rc;

	return vcm_wait(v, submit_id, res_rc, data_out, p_out_len, 1000);
}

static int vcm_rxthread(void *param)
{
	struct vpumgr_device *vdev = param;
	struct device *dev = vdev->sdev;
	struct vcm_dev *pvcm = &vdev->vcm;
	struct vcm_msg *msg = &pvcm->rxmsg;
	struct vpu_cmd *vcmd;
	int rc;

	while (!kthread_should_stop()) {
		rc = vcm_recv(&pvcm->ipc_xlink_handle, msg);
		if (rc == -EPIPE)
			break;
		if (rc)
			continue;

		switch (msg->cmd) {
		case VCTX_MSG_REPLY:
			/* find local data associated with that msg id */
			vcmd = vcmd_get(pvcm, (unsigned long)msg->id);
			if (!vcmd)
				break;

			if (msg->ctx != vcmd->msg.ctx)
				dev_warn(dev, "reply msg #%u's ctx (%u) mismatches vcmd ctx (%u)\n",
					 msg->id, msg->ctx, vcmd->msg.ctx);

			vcmd->submit_err = 0;

			/* submit corresponding to msg->id is done, do post process */
			memcpy(&vcmd->msg, msg, msg->size);
			complete(&vcmd->complete);

			vcmd_put(vcmd);
		break;
		default:
		break;
		}
	}
	return rc;
}

static int vpu_connection_up(struct vpumgr_device *vdev)
{
	struct vcm_dev *pvcm = &vdev->vcm;
	struct task_struct *rxthread;
	struct device *dev = vdev->dev;
	enum xlink_error xlink_err;
	int rc;

	mutex_lock(&pvcm->fwboot_mutex);
	if (pvcm->fwuser_cnt == 0) {
		/* boot-up firmware under protection of fwboot_mutex */
		if (vdev->fwname[0] != '\0') {

			/* ensure VPU state is reset/stopped before re-boot */
			xlink_reset_device(&pvcm->ipc_xlink_handle);

			xlink_err = xlink_boot_device(&pvcm->ipc_xlink_handle, vdev->fwname);
			if (xlink_err != X_LINK_SUCCESS) {
				dev_err(dev, "%s: failed to boot-up VPU with firmware %s, rc %d\n",
					__func__, vdev->fwname, (int)xlink_err);
				rc = -EFAULT;
				goto exit;
			}
		}

		pvcm->xlink_closing = false;

		/* try to (re)open xlink channel after firmware boot-up */
		xlink_err = xlink_open_channel(&pvcm->ipc_xlink_handle, VCM_XLINK_CHANNEL,
					       RXB_TXB, VCM_XLINK_CHAN_SIZE, XLINK_IPC_TIMEOUT);
		if (xlink_err != X_LINK_SUCCESS && xlink_err != X_LINK_ALREADY_OPEN) {
			dev_err(dev, "%s: failed to open xlink channel %d, rc %d\n",
				__func__, VCM_XLINK_CHANNEL, (int)xlink_err);
			rc = -EFAULT;
			goto exit;
		}

		rxthread = kthread_run(vcm_rxthread, (void *)vdev, "vcmrx");
		if (IS_ERR(rxthread)) {
			rc = PTR_ERR(rxthread);
			goto error_close_channel;
		}

		pvcm->rxthread = get_task_struct(rxthread);

		dev_dbg(dev, "%s: success\n", __func__);
	}
	pvcm->fwuser_cnt++;
	mutex_unlock(&pvcm->fwboot_mutex);
	return 0;

error_close_channel:
	xlink_close_channel(&pvcm->ipc_xlink_handle, VCM_XLINK_CHANNEL);
exit:
	mutex_unlock(&pvcm->fwboot_mutex);
	return rc;
}

static void vpu_connection_down(struct vpumgr_device *vdev)
{
	struct vcm_dev *pvcm = &vdev->vcm;
	struct device *dev = vdev->dev;

	mutex_lock(&pvcm->fwboot_mutex);
	pvcm->fwuser_cnt--;
	if (pvcm->fwuser_cnt == 0) {
		pvcm->xlink_closing = true;
		xlink_close_channel(&pvcm->ipc_xlink_handle, VCM_XLINK_CHANNEL);
		kthread_stop(pvcm->rxthread);
		put_task_struct(pvcm->rxthread);
		pvcm->rxthread = NULL;

		dev_dbg(dev, "%s: success\n", __func__);
	}
	mutex_unlock(&pvcm->fwboot_mutex);
}

int vcm_open(struct vpumgr_ctx *v, struct vpumgr_device *vdev)
{
	struct device *dev = vdev->sdev;
	int rep_rc, rc;

	v->vdev = vdev;

	if (!vdev->vcm.enabled)
		return 0;

	rc = vpu_connection_up(vdev);
	if (rc)
		return rc;

	rc = vcm_call(v, VCTX_MSG_CREATE, NULL, 0, &rep_rc, NULL, NULL);

	if (rc != 0 || rep_rc < 0)
		dev_err(dev, "%s: Vpu context create with rc:%d and vpu reply rc:%d\n",
			__func__, rc, rep_rc);
	if (rc)
		goto connection_down;
	if (rep_rc < 0) {
		rc = -ENXIO;
		goto connection_down;
	}

	v->vpu_ctx_id = rep_rc;
	v->total_vcmds = 0;
	return 0;

connection_down:
	vpu_connection_down(vdev);
	return rc;
}

int vcm_close(struct vpumgr_ctx *v)
{
	struct vpumgr_device *vdev = v->vdev;
	struct device *dev = vdev->sdev;
	int rep_rc = 0, rc;

	if (!vdev->vcm.enabled)
		return 0;

	rc = vcm_call(v, VCTX_MSG_DESTROY, NULL, 0, &rep_rc, NULL, NULL);
	dev_dbg(dev, "vpu context %d is destroyed with rc:%d and vpu reply rc:%d\n",
		v->vpu_ctx_id, rc, rep_rc);

	/* remove submit belongs to this context */
	vcmd_clean_ctx(&vdev->vcm, v);

	vpu_connection_down(vdev);
	return 0;
}

int vcm_debugfs_stats_show(struct seq_file *file, struct vpumgr_ctx *v)
{
	if (!v->vdev->vcm.enabled)
		return 0;
	seq_printf(file, "\tvpu context: #%d\n", v->vpu_ctx_id);
	seq_printf(file, "\t\tNum of completed cmds: %llu\n", v->total_vcmds);
	seq_printf(file, "\t\tNum of on-going cmds: %d\n", vcmd_count_ctx(&v->vdev->vcm, v));
	return 0;
}

int vcm_init(struct vpumgr_device *vdev, u32 sw_dev_id)
{
	struct vcm_dev *pvcm = &vdev->vcm;
	int rc = 0;

	if (sw_dev_id == XLINK_INVALID_SW_DEVID) {
		dev_warn(vdev->dev, "%s: vcm is not enabled!\n",
			 __func__);
		rc = 0;
		goto exit;
	}

	pvcm->sw_dev_id = sw_dev_id;
	rc = vcm_vpu_link_init(pvcm);
	if (rc)
		goto exit;

	pvcm->wq = alloc_ordered_workqueue("vcm workqueue", WQ_MEM_RECLAIM | WQ_HIGHPRI);
	if (!pvcm->wq) {
		rc = -ENOMEM;
		goto vpu_link_fini;
	}

	mutex_init(&pvcm->msg_idr_lock);
	mutex_init(&pvcm->fwboot_mutex);
	idr_init(&pvcm->msg_idr);

	pvcm->fwuser_cnt = 0;
	pvcm->enabled = true;
	return 0;

vpu_link_fini:
	vcm_vpu_link_fini(pvcm);
exit:
	pvcm->enabled = false;
	return rc;
}

int vcm_fini(struct vpumgr_device *vdev)
{
	struct vcm_dev *pvcm = &vdev->vcm;

	if (!pvcm->enabled)
		return 0;

	idr_destroy(&pvcm->msg_idr);
	destroy_workqueue(pvcm->wq);
	mutex_destroy(&pvcm->fwboot_mutex);
	mutex_destroy(&pvcm->msg_idr_lock);
	vcm_vpu_link_fini(pvcm);
	return 0;
}
