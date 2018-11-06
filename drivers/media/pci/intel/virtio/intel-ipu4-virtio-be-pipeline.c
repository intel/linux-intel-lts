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
	pipeline = filp_open("/dev/intel_pipeline", O_RDWR | O_NONBLOCK, 0);
	guestID = domid;

	return IPU4_REQ_PROCESSED;
}

int process_pipeline_close(struct ipu4_virtio_req_info *req_info)
{
	struct ipu4_virtio_req *req = req_info->request;

	pr_info("%s: %d", __func__, req->op[0]);

	filp_close(pipeline, 0);
	guestID = -1;

	return IPU4_REQ_PROCESSED;
}

int process_enum_nodes(struct ipu4_virtio_req_info *req_info)
{
	int err = 0;
	struct ici_isys_pipeline_device *dev = pipeline->private_data;
	struct ici_node_desc *host_virt;
	struct ipu4_virtio_req *req = req_info->request;
	int domid = req_info->domid;

	pr_debug("%s\n", __func__);

	host_virt = (struct ici_node_desc *)map_guest_phys(domid, req->payload, PAGE_SIZE);
	if (host_virt == NULL) {
		pr_err("process_enum_nodes: NULL host_virt");
		return IPU4_REQ_ERROR;
	}

	err = dev->pipeline_ioctl_ops->pipeline_enum_nodes(pipeline, dev, host_virt);

	if (err)
		return IPU4_REQ_ERROR;
	else
		return IPU4_REQ_PROCESSED;
}

int process_enum_links(struct ipu4_virtio_req_info *req_info)
{
	int err = 0;
	struct ici_isys_pipeline_device *dev = pipeline->private_data;
	struct ici_links_query *host_virt;
	struct ipu4_virtio_req *req = req_info->request;
	int domid = req_info->domid;

	pr_debug("%s\n", __func__);

	host_virt = (struct ici_links_query *)map_guest_phys(domid, req->payload, PAGE_SIZE);
	if (host_virt == NULL) {
		pr_err("%s: NULL host_virt\n", __func__);
		return IPU4_REQ_ERROR;
	}
	err = dev->pipeline_ioctl_ops->pipeline_enum_links(pipeline, dev, host_virt);

	if (err)
		return IPU4_REQ_ERROR;
	else
		return IPU4_REQ_PROCESSED;
}
int process_get_supported_framefmt(struct ipu4_virtio_req_info *req_info)
{
	int err = 0;
	struct ici_isys_pipeline_device *dev = pipeline->private_data;
	struct ici_pad_supported_format_desc *host_virt;
	struct ipu4_virtio_req *req = req_info->request;
	int domid = req_info->domid;

	pr_debug("%s\n", __func__);

	host_virt = (struct ici_pad_supported_format_desc *)map_guest_phys(domid, req->payload, PAGE_SIZE);
	if (host_virt == NULL) {
		pr_err("%s: NULL host_virt\n", __func__);
		return IPU4_REQ_ERROR;
	}
	err = dev->pipeline_ioctl_ops->pad_get_supported_format(pipeline, dev, host_virt);

	if (err)
		return IPU4_REQ_ERROR;
	else
		return IPU4_REQ_PROCESSED;
}

int process_set_framefmt(struct ipu4_virtio_req_info *req_info)
{
	int err = 0;
	struct ici_isys_pipeline_device *dev = pipeline->private_data;
	struct ici_pad_framefmt *host_virt;
	struct ipu4_virtio_req *req = req_info->request;
	int domid = req_info->domid;

	pr_debug("%s\n", __func__);

	host_virt = (struct ici_pad_framefmt *)map_guest_phys(domid, req->payload, PAGE_SIZE);
	if (host_virt == NULL) {
		pr_err("%s: NULL host_virt\n", __func__);
		return IPU4_REQ_ERROR;
	}
	err = dev->pipeline_ioctl_ops->pad_set_ffmt(pipeline, dev, host_virt);

	if (err)
		return IPU4_REQ_ERROR;
	else
		return IPU4_REQ_PROCESSED;
}

int process_get_framefmt(struct ipu4_virtio_req_info *req_info)
{
	int err = 0;
	struct ici_isys_pipeline_device *dev = pipeline->private_data;
	struct ici_pad_framefmt *host_virt;
	struct ipu4_virtio_req *req = req_info->request;
	int domid = req_info->domid;

	pr_debug("%s\n", __func__);

	host_virt = (struct ici_pad_framefmt *)map_guest_phys(domid, req->payload, PAGE_SIZE);
	if (host_virt == NULL) {
		pr_err("%s: NULL host_virt\n", __func__);
		return IPU4_REQ_ERROR;
	}
	err = dev->pipeline_ioctl_ops->pad_get_ffmt(pipeline, dev, host_virt);

	if (err)
		return IPU4_REQ_ERROR;
	else
		return IPU4_REQ_PROCESSED;
}

int process_setup_pipe(struct ipu4_virtio_req_info *req_info)
{
	int err = 0;
	struct ici_isys_pipeline_device *dev = pipeline->private_data;
	struct ici_link_desc *host_virt;
	struct ipu4_virtio_req *req = req_info->request;
	int domid = req_info->domid;

	pr_debug("%s\n", __func__);

	host_virt = (struct ici_link_desc *)map_guest_phys(domid, req->payload, PAGE_SIZE);
	if (host_virt == NULL) {
		pr_err("%s: NULL host_virt\n", __func__);
		return IPU4_REQ_ERROR;
	}
	err = dev->pipeline_ioctl_ops->pipeline_setup_pipe(pipeline, dev, host_virt);

	if (err)
		return IPU4_REQ_ERROR;
	else
		return IPU4_REQ_PROCESSED;
}

int process_pad_set_sel(struct ipu4_virtio_req_info *req_info)
{
	int err = 0;
	struct ici_isys_pipeline_device *dev = pipeline->private_data;
	struct ici_pad_selection *host_virt;
	struct ipu4_virtio_req *req = req_info->request;
	int domid = req_info->domid;

	pr_debug("%s\n", __func__);

	host_virt = (struct ici_pad_selection *)map_guest_phys(domid, req->payload, PAGE_SIZE);
	if (host_virt == NULL) {
		pr_err("%s: NULL host_virt\n", __func__);
		return IPU4_REQ_ERROR;
	}
	err = dev->pipeline_ioctl_ops->pad_set_sel(pipeline, dev, host_virt);

	if (err)
		return IPU4_REQ_ERROR;
	else
		return IPU4_REQ_PROCESSED;
}

int process_pad_get_sel(struct ipu4_virtio_req_info *req_info)
{
	int err = 0;
	struct ici_isys_pipeline_device *dev = pipeline->private_data;
	struct ici_pad_selection *host_virt;
	struct ipu4_virtio_req *req = req_info->request;
	int domid = req_info->domid;

	pr_debug("%s\n", __func__);

	host_virt = (struct ici_pad_selection *)map_guest_phys(domid, req->payload, PAGE_SIZE);
	if (host_virt == NULL) {
		pr_err("%s: NULL host_virt\n", __func__);
		return IPU4_REQ_ERROR;
	}
	err = dev->pipeline_ioctl_ops->pad_get_sel(pipeline, dev, host_virt);

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

/*
	union isys_ioctl_cmd_args {
		struct ici_node_desc node_desc;
		struct ici_link_desc link;
		struct ici_pad_framefmt pad_prop;
		struct ici_pad_supported_format_desc
			format_desc;
		struct ici_links_query links_query;
		struct ici_pad_selection pad_sel;
	};

	.pipeline_setup_pipe = ici_setup_link,
	.pipeline_enum_nodes = pipeline_enum_nodes,
	.pipeline_enum_links = pipeline_enum_links,
	.pad_set_ffmt = ici_pipeline_set_ffmt,
	.pad_get_ffmt = ici_pipeline_get_ffmt,
	.pad_get_supported_format =
		ici_pipeline_get_supported_format,
	.pad_set_sel = ici_pipeline_set_sel,
	.pad_get_sel = ici_pipeline_get_sel,

*/

