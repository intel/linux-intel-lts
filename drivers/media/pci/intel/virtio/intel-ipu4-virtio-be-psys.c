// SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
/*
 * Copyright (C) 2018 Intel Corporation
 */

#include <linux/syscalls.h>
#include "ipu-psys.h"

#include <linux/vhm/acrn_vhm_mm.h>
#include "intel-ipu4-virtio-common.h"
#include "intel-ipu4-virtio-common-psys.h"
#include "intel-ipu4-virtio-be-request-queue.h"
#include "intel-ipu4-virtio-be.h"

int process_psys_mapbuf(struct ipu4_virtio_req_info *req_info)
{
	return IPU4_REQ_ERROR;
}

int process_psys_unmapbuf(struct ipu4_virtio_req_info *req_info)
{
	int status = 0;

	struct ipu_psys_fh *fh = req_info->request->be_fh->private_data;
	if(!fh) {
		pr_err("%s NULL file handler", __func__);
		return IPU4_REQ_ERROR;
	}

	status = fh->vfops->unmap_buf(fh, req_info);

	/*Only doing this in mediated mode because 
	fd passed from SOS to user space is invalid in UOS.*/
	ksys_close(req_info->request->op[0]);

	req_info->request->func_ret = status;

	if (status)
		return IPU4_REQ_ERROR;
	else
		return IPU4_REQ_PROCESSED;
}

int process_psys_querycap(struct ipu4_virtio_req_info *req_info)
{
	struct ipu_psys_fh *fh = req_info->request->be_fh->private_data;
	int status = 0;

	struct ipu_psys_capability *psys_caps;
	psys_caps = map_guest_phys(req_info->domid,
						req_info->request->payload,
						sizeof(struct ipu_psys_capability));
	if (psys_caps == NULL) {
		pr_err("%s: failed to get ipu_psys_capability %u %llu",
			__func__, req_info->domid, req_info->request->payload);
		return -EFAULT;
	}

	*psys_caps = fh->psys->caps;

	unmap_guest_phys(req_info->domid,
			req_info->request->payload);

	req_info->request->func_ret = status;

	if (status)
		return IPU4_REQ_ERROR;
	else
		return IPU4_REQ_PROCESSED;
}

int process_psys_putbuf(struct ipu4_virtio_req_info *req_info)
{
	return IPU4_REQ_ERROR;
}

int process_psys_qcmd(struct ipu4_virtio_req_info *req_info)
{
	struct ipu_psys_fh *fh = req_info->request->be_fh->private_data;
	int status = 0;

	status = fh->vfops->qcmd(fh, req_info);

	req_info->request->func_ret = status;

	if (status)
		return IPU4_REQ_ERROR;
	else
		return IPU4_REQ_PROCESSED;
}

int process_psys_dqevent(struct ipu4_virtio_req_info *req_info)
{
	struct ipu_psys_fh *fh = req_info->request->be_fh->private_data;
	int status = 0;

	status = fh->vfops->dqevent(fh, req_info, req_info->request->be_fh->f_flags);

	req_info->request->func_ret = status;

	if (status)
		return IPU4_REQ_ERROR;
	else
		return IPU4_REQ_PROCESSED;
}

int process_psys_getbuf(struct ipu4_virtio_req_info *req_info)
{
	struct ipu_psys_fh *fh = req_info->request->be_fh->private_data;
	int status = 0;

	status = fh->vfops->get_buf(fh, req_info);

	req_info->request->func_ret = status;

	if (status)
		return IPU4_REQ_ERROR;
	else
		return IPU4_REQ_PROCESSED;
}

int process_psys_get_manifest(struct ipu4_virtio_req_info *req_info)
{
	struct ipu_psys_fh *fh = req_info->request->be_fh->private_data;
	int status = 0;

	status = fh->vfops->get_manifest(fh, req_info);

	req_info->request->func_ret = status;

	if (status)
		return IPU4_REQ_ERROR;
	else
		return IPU4_REQ_PROCESSED;
}

int process_psys_open(struct ipu4_virtio_req_info *req_info)
{
	struct file *fh;
	pr_info("%s: /dev/ipu-psys0", __func__);

	fh = filp_open("/dev/ipu-psys0", req_info->request->op[0], 0);

	if (fh == NULL) {
		pr_err("%s: Native IPU psys device not found",
										__func__);
		return IPU4_REQ_ERROR;
	}

	req_info->request->be_fh = fh;

	return IPU4_REQ_PROCESSED;
}

int process_psys_close(struct ipu4_virtio_req_info *req_info)
{
	pr_info("%s: /dev/ipu-psys0", __func__);

	filp_close(req_info->request->be_fh, 0);

	return IPU4_REQ_PROCESSED;
}

int process_psys_poll(struct ipu4_virtio_req_info *req_info)
{
	struct ipu_psys_fh *fh = req_info->request->be_fh->private_data;
	int status = 0;

	status = fh->vfops->poll(fh, req_info);

	if (status)
		return IPU4_REQ_ERROR;
	else
		return IPU4_REQ_PROCESSED;
}

int process_psys_mapbuf_thread(void *data)
{
	int status;

	status = process_psys_mapbuf(data);
	notify_fe(status, data);
	do_exit(0);
	return 0;
}

int process_psys_unmapbuf_thread(void *data)
{
	int status;

	status = process_psys_unmapbuf(data);
	notify_fe(status, data);
	do_exit(0);
	return 0;
}

int process_psys_querycap_thread(void *data)
{
	int status;

	status = process_psys_querycap(data);
	notify_fe(status, data);
	do_exit(0);
	return 0;
}

int process_psys_putbuf_thread(void *data)
{
	int status;

	status = process_psys_putbuf(data);
	notify_fe(status, data);
	do_exit(0);
	return 0;
}

int process_psys_qcmd_thread(void *data)
{
	int status;

	status = process_psys_qcmd(data);
	notify_fe(status, data);
	do_exit(0);
	return 0;
}

int process_psys_dqevent_thread(void *data)
{
	int status;

	status = process_psys_dqevent(data);
	notify_fe(status, data);
	do_exit(0);
	return 0;
}

int process_psys_get_manifest_thread(void *data)
{
	int status;

	status = process_psys_get_manifest(data);
	notify_fe(status, data);
	do_exit(0);
	return 0;
}

int process_psys_getbuf_thread(void *data)
{
	int status;

	status = process_psys_getbuf(data);
	notify_fe(status, data);
	do_exit(0);
	return 0;
}

int process_psys_open_thread(void *data)
{
	int status;

	status = process_psys_open(data);
	notify_fe(status, data);
	do_exit(0);
	return 0;
}

int process_psys_close_thread(void *data)
{
	int status;

	status = process_psys_close(data);
	notify_fe(status, data);
	do_exit(0);
	return 0;
}

int process_psys_poll_thread(void *data)
{
	int status;

	status = process_psys_poll(data);
	notify_fe(status, data);
	do_exit(0);
	return 0;
}
