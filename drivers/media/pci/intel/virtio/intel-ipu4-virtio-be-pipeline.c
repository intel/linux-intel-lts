// SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
/*
 * Copyright (C) 2018 Intel Corporation
 */

#include <linux/kernel.h>
#include <linux/file.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/poll.h>

#include <media/ici.h>
#include <linux/vhm/acrn_vhm_mm.h>
#include "intel-ipu4-virtio-be-pipeline.h"
#include "./ici/ici-isys-pipeline.h"
#include "./ici/ici-isys-pipeline-device.h"
#include "intel-ipu4-virtio-be-request-queue.h"
#include "intel-ipu4-virtio-be.h"

static struct file *pipeline;
static int guestID = -1;

int process_pipeline_open(struct ipu4_virtio_req_info *req_info)
{
	int domid = req_info->domid;
	if (guestID != -1 && guestID != domid) {
		pr_err("%s: pipeline device already opened by other guest! %d %d", __func__, guestID, domid);
		return IPU4_REQ_ERROR;
	}

	pr_info("process_device_open: /dev/intel_pipeline");
	if (!pipeline)
		pipeline = filp_open("/dev/intel_pipeline", O_RDWR | O_NONBLOCK, 0);
	guestID = domid;

	return IPU4_REQ_PROCESSED;
}

int process_pipeline_close(struct ipu4_virtio_req_info *req_info)
{
	struct ipu4_virtio_req *req = req_info->request;

	pr_info("%s: %d", __func__, req->op[0]);

	if (pipeline)
		filp_close(pipeline, 0);
	guestID = -1;
	pipeline = NULL;

	return IPU4_REQ_PROCESSED;
}

int process_enum_nodes(struct ipu4_virtio_req_info *req_info)
{
	int err = 0;
	struct ici_isys_pipeline_device *dev;
	struct ici_node_desc *host_virt;
	struct ipu4_virtio_req *req;
	int domid = req_info->domid;

	pr_debug("%s\n", __func__);

	if (!pipeline) {
		pr_err("%s: NULL pipeline", __func__);
		return IPU4_REQ_ERROR;
	}
	dev = pipeline->private_data;

	if (!req_info) {
		pr_err("%s: NULL req_info", __func__);
		return IPU4_REQ_ERROR;
	}
	req = req_info->request;

	host_virt = map_guest_phys(domid, req->payload,
						sizeof(struct ici_node_desc));
	if (host_virt == NULL) {
		pr_err("process_enum_nodes: NULL host_virt");
		return IPU4_REQ_ERROR;
	}

	err = dev->pipeline_ioctl_ops->pipeline_enum_nodes(pipeline, dev, host_virt);

	unmap_guest_phys(domid, req->payload);
	if (err)
		return IPU4_REQ_ERROR;
	else
		return IPU4_REQ_PROCESSED;
}

int process_enum_links(struct ipu4_virtio_req_info *req_info)
{
	int err = 0;
	struct ici_isys_pipeline_device *dev;
	struct ici_links_query *host_virt;
	struct ipu4_virtio_req *req;
	int domid = req_info->domid;

	pr_debug("%s\n", __func__);

	if (!pipeline) {
		pr_err("%s: NULL pipeline", __func__);
		return IPU4_REQ_ERROR;
	}
	dev = pipeline->private_data;

	if (!req_info) {
		pr_err("%s: NULL req_info", __func__);
		return IPU4_REQ_ERROR;
	}
	req = req_info->request;

	host_virt = map_guest_phys(domid, req->payload,
						sizeof(struct ici_links_query));
	if (host_virt == NULL) {
		pr_err("%s: NULL host_virt\n", __func__);
		return IPU4_REQ_ERROR;
	}
	err = dev->pipeline_ioctl_ops->pipeline_enum_links(pipeline, dev, host_virt);

	unmap_guest_phys(domid, req->payload);
	if (err)
		return IPU4_REQ_ERROR;
	else
		return IPU4_REQ_PROCESSED;
}
int process_get_supported_framefmt(struct ipu4_virtio_req_info *req_info)
{
	int err = 0;
	struct ici_isys_pipeline_device *dev;
	struct ici_pad_supported_format_desc *host_virt;
	struct ipu4_virtio_req *req;
	int domid = req_info->domid;

	pr_debug("%s\n", __func__);

	if (!pipeline) {
		pr_err("%s: NULL pipeline", __func__);
		return IPU4_REQ_ERROR;
	}
	dev = pipeline->private_data;

	if (!req_info) {
		pr_err("%s: NULL req_info", __func__);
		return IPU4_REQ_ERROR;
	}
	req = req_info->request;

	host_virt = map_guest_phys(domid, req->payload,
				sizeof(struct ici_pad_supported_format_desc));
	if (host_virt == NULL) {
		pr_err("%s: NULL host_virt\n", __func__);
		return IPU4_REQ_ERROR;
	}
	err = dev->pipeline_ioctl_ops->pad_get_supported_format(pipeline, dev, host_virt);

	unmap_guest_phys(domid, req->payload);
	if (err)
		return IPU4_REQ_ERROR;
	else
		return IPU4_REQ_PROCESSED;
}

int process_set_framefmt(struct ipu4_virtio_req_info *req_info)
{
	int err = 0;
	struct ici_isys_pipeline_device *dev;
	struct ici_pad_framefmt *host_virt;
	struct ipu4_virtio_req *req;
	int domid = req_info->domid;

	pr_debug("%s\n", __func__);

	if (!pipeline) {
		pr_err("%s: NULL pipeline", __func__);
		return IPU4_REQ_ERROR;
	}
	dev = pipeline->private_data;

	if (!req_info) {
		pr_err("%s: NULL req_info", __func__);
		return IPU4_REQ_ERROR;
	}
	req = req_info->request;

	host_virt = map_guest_phys(domid, req->payload,
						sizeof(struct ici_pad_framefmt));
	if (host_virt == NULL) {
		pr_err("%s: NULL host_virt\n", __func__);
		return IPU4_REQ_ERROR;
	}
	err = dev->pipeline_ioctl_ops->pad_set_ffmt(pipeline, dev, host_virt);

	unmap_guest_phys(domid, req->payload);
	if (err)
		return IPU4_REQ_ERROR;
	else
		return IPU4_REQ_PROCESSED;
}

int process_get_framefmt(struct ipu4_virtio_req_info *req_info)
{
	int err = 0;
	struct ici_isys_pipeline_device *dev;
	struct ici_pad_framefmt *host_virt;
	struct ipu4_virtio_req *req;
	int domid = req_info->domid;

	pr_debug("%s\n", __func__);

	if (!pipeline) {
		pr_err("%s: NULL pipeline", __func__);
		return IPU4_REQ_ERROR;
	}
	dev = pipeline->private_data;

	if (!req_info) {
		pr_err("%s: NULL req_info", __func__);
		return IPU4_REQ_ERROR;
	}
	req = req_info->request;

	host_virt = map_guest_phys(domid, req->payload,
						sizeof(struct ici_pad_framefmt));
	if (host_virt == NULL) {
		pr_err("%s: NULL host_virt\n", __func__);
		return IPU4_REQ_ERROR;
	}
	err = dev->pipeline_ioctl_ops->pad_get_ffmt(pipeline, dev, host_virt);

	unmap_guest_phys(domid, req->payload);
	if (err)
		return IPU4_REQ_ERROR;
	else
		return IPU4_REQ_PROCESSED;
}

int process_setup_pipe(struct ipu4_virtio_req_info *req_info)
{
	int err = 0;
	struct ici_isys_pipeline_device *dev;
	struct ici_link_desc *host_virt;
	struct ipu4_virtio_req *req;
	int domid = req_info->domid;

	pr_debug("%s\n", __func__);

	if (!pipeline) {
		pr_err("%s: NULL pipeline", __func__);
		return IPU4_REQ_ERROR;
	}
	dev = pipeline->private_data;

	if (!req_info) {
		pr_err("%s: NULL req_info", __func__);
		return IPU4_REQ_ERROR;
	}
	req = req_info->request;

	host_virt = map_guest_phys(domid, req->payload,
						sizeof(struct ici_link_desc));
	if (host_virt == NULL) {
		pr_err("%s: NULL host_virt\n", __func__);
		return IPU4_REQ_ERROR;
	}
	err = dev->pipeline_ioctl_ops->pipeline_setup_pipe(pipeline, dev, host_virt);

	unmap_guest_phys(domid, req->payload);
	if (err)
		return IPU4_REQ_ERROR;
	else
		return IPU4_REQ_PROCESSED;
}

int process_pad_set_sel(struct ipu4_virtio_req_info *req_info)
{
	int err = 0;
	struct ici_isys_pipeline_device *dev;
	struct ici_pad_selection *host_virt;
	struct ipu4_virtio_req *req;
	int domid = req_info->domid;

	pr_debug("%s\n", __func__);

	if (!pipeline) {
		pr_err("%s: NULL pipeline", __func__);
		return IPU4_REQ_ERROR;
	}
	dev = pipeline->private_data;

	if (!req_info) {
		pr_err("%s: NULL req_info", __func__);
		return IPU4_REQ_ERROR;
	}
	req = req_info->request;

	host_virt = map_guest_phys(domid, req->payload,
						sizeof(struct ici_pad_selection));
	if (host_virt == NULL) {
		pr_err("%s: NULL host_virt\n", __func__);
		return IPU4_REQ_ERROR;
	}
	err = dev->pipeline_ioctl_ops->pad_set_sel(pipeline, dev, host_virt);

	unmap_guest_phys(domid, req->payload);
	if (err)
		return IPU4_REQ_ERROR;
	else
		return IPU4_REQ_PROCESSED;
}

int process_pad_get_sel(struct ipu4_virtio_req_info *req_info)
{
	int err = 0;
	struct ici_isys_pipeline_device *dev;
	struct ici_pad_selection *host_virt;
	struct ipu4_virtio_req *req;
	int domid = req_info->domid;

	pr_debug("%s\n", __func__);

	if (!pipeline) {
		pr_err("%s: NULL pipeline", __func__);
		return IPU4_REQ_ERROR;
	}
	dev = pipeline->private_data;

	if (!req_info) {
		pr_err("%s: NULL req_info", __func__);
		return IPU4_REQ_ERROR;
	}
	req = req_info->request;

	host_virt = map_guest_phys(domid, req->payload,
						sizeof(struct ici_pad_selection));
	if (host_virt == NULL) {
		pr_err("%s: NULL host_virt\n", __func__);
		return IPU4_REQ_ERROR;
	}
	err = dev->pipeline_ioctl_ops->pad_get_sel(pipeline, dev, host_virt);

	unmap_guest_phys(domid, req->payload);
	if (err)
		return IPU4_REQ_ERROR;
	else
		return IPU4_REQ_PROCESSED;
}

int process_pipeline_open_thread(void *data)
{
	int status;

	status = process_pipeline_open(data);
	notify_fe(status, data);
	do_exit(0);
	return 0;
}

int process_pipeline_close_thread(void *data)
{
	int status;

	status = process_pipeline_close(data);
	notify_fe(status, data);
	do_exit(0);
	return 0;
}

int process_enum_nodes_thread(void *data)
{
	int status;

	status = process_enum_nodes(data);
	notify_fe(status, data);
	do_exit(0);
	return 0;
}

int process_enum_links_thread(void *data)
{
	int status;

	status = process_enum_links(data);
	notify_fe(status, data);
	do_exit(0);
	return 0;
}

int process_get_supported_framefmt_thread(void *data)
{
	int status;

	status = process_get_supported_framefmt(data);
	notify_fe(status, data);
	do_exit(0);
	return 0;
}

int process_set_framefmt_thread(void *data)
{
	int status;

	status = process_set_framefmt(data);
	notify_fe(status, data);
	do_exit(0);
	return 0;
}

int process_get_framefmt_thread(void *data)
{
	int status;

	status = process_get_framefmt(data);
	notify_fe(status, data);
	do_exit(0);
	return 0;
}

int process_pad_set_sel_thread(void *data)
{
	int status;

	status = process_pad_set_sel(data);
	notify_fe(status, data);
	do_exit(0);
	return 0;
}

int process_pad_get_sel_thread(void *data)
{
	int status;

	status = process_pad_get_sel(data);
	notify_fe(status, data);
	do_exit(0);
	return 0;
}

int process_setup_pipe_thread(void *data)
{
	int status;

	status = process_setup_pipe(data);
	notify_fe(status, data);
	do_exit(0);
	return 0;
}
