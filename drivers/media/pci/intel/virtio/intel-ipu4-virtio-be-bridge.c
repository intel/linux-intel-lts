// SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
/*
 * Copyright (C) 2018 Intel Corporation
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/syscalls.h>
#include <linux/kthread.h>

#include "intel-ipu4-virtio-be-bridge.h"
#include "./ici/ici-isys-frame-buf.h"
#include "intel-ipu4-virtio-be-pipeline.h"
#include "intel-ipu4-virtio-be-stream.h"

int intel_ipu4_virtio_msg_parse(struct ipu4_virtio_req_info *req_info)
{
	int ret = 0;
	struct ipu4_virtio_req *req = req_info->request;

	if (!req) {
		pr_err("IPU mediator: request is NULL\n");
		return -EINVAL;
	}
	if ((req->cmd < IPU4_CMD_DEVICE_OPEN) ||
			(req->cmd >= IPU4_CMD_GET_N)) {
			pr_err("IPU mediator: invalid command\n");
			return -EINVAL;
	}

	if (!req_info)
		return -1;

	switch (req->cmd) {
	case IPU4_CMD_POLL:
			/*
			 * Open video device node
			 * op0 - virtual device node number
			 * op1 - Actual device fd. By default set to 0
			 */
			pr_debug("%s: process_poll %d",
						__func__, req->op[0]);
			kthread_run(process_poll_thread, req_info,
								"process_poll");
			req->stat = IPU4_REQ_PENDING;
			break;
	case IPU4_CMD_DEVICE_OPEN:
			/*
			 * Open video device node
			 * op0 - virtual device node number
			 * op1 - Actual device fd. By default set to 0
			 */
			pr_debug("DEVICE_OPEN: virtual_dev_id:%d actual_fd:%d\n", req->op[0], req->op[1]);
			kthread_run(process_device_open_thread, req_info,
								"process_device_open");
			req->stat = IPU4_REQ_PENDING;
			break;
	case IPU4_CMD_DEVICE_CLOSE:
			/*
			 * Close video device node
			 * op0 - virtual device node number
			 * op1 - Actual device fd. By default set to 0
			 */
			pr_debug("DEVICE_CLOSE: virtual_dev_id:%d actual_fd:%d\n", req->op[0], req->op[1]);
			kthread_run(process_device_close_thread, req_info,
								"process_device_close");
			req->stat = IPU4_REQ_PENDING;
			break;
	case IPU4_CMD_STREAM_ON:
			/* Start Stream
			 * op0 - virtual device node number
			 * op1 - Actual device fd. By default set to 0
			 */
			pr_debug("STREAM ON: virtual_dev_id:%d actual_fd:%d\n", req->op[0], req->op[1]);
			kthread_run(process_stream_on_thread, req_info,
								"process_stream_on");
			req->stat = IPU4_REQ_PENDING;
			break;
	case IPU4_CMD_STREAM_OFF:
			/* Stop Stream
			 * op0 - virtual device node number
			 * op1 - Actual device fd. By default set to 0
			 */
			pr_debug("STREAM OFF: virtual_dev_id:%d actual_fd:%d\n", req->op[0], req->op[1]);
			kthread_run(process_stream_off_thread, req_info,
								"process_stream_off");
			req->stat = IPU4_REQ_PENDING;
			break;
	case IPU4_CMD_GET_BUF:
			/* Set Format of a given video node
			 * op0 - virtual device node number
			 * op1 - Actual device fd. By default set to 0
			 * op2 - Memory Type 1: USER_PTR 2: DMA_PTR
			 * op3 - Number of planes
			 * op4 - Buffer ID
			 * op5 - Length of Buffer
			 */

			pr_debug("%s process_get_buf %d",
						__func__, req->op[0]);
			kthread_run(process_get_buf_thread, req_info,
								"process_get_buf");
			req->stat = IPU4_REQ_PENDING;
			break;
	case IPU4_CMD_PUT_BUF:
			/* Set Format of a given video node
			 * op0 - virtual device node number
			 * op1 - Actual device fd. By default set to 0
			 * op2 - Memory Type 1: USER_PTR 2: DMA_PTR
			 */
			pr_debug("%s process_put_buf %d",
						__func__, req->op[0]);
			kthread_run(process_put_buf_thread, req_info,
								"process_put_buf");
			req->stat = IPU4_REQ_PENDING;
			break;
	case IPU4_CMD_SET_FORMAT:
			pr_debug("%s process_set_format %d",
						__func__, req->op[0]);
			kthread_run(process_set_format_thread, req_info,
								"process_set_format");
			req->stat = IPU4_REQ_PENDING;
			break;
	case IPU4_CMD_PIPELINE_OPEN:
			pr_debug("%s process_pipeline_open %d",
						__func__, req->op[0]);
			kthread_run(process_pipeline_open_thread, req_info,
								"process_pipeline_open");
			req->stat = IPU4_REQ_PENDING;
			break;
	case IPU4_CMD_PIPELINE_CLOSE:
			pr_debug("%s process_pipeline_close %d",
						__func__, req->op[0]);
			kthread_run(process_pipeline_close_thread, req_info,
								"process_pipeline_close");
			req->stat = IPU4_REQ_PENDING;
			break;
	case IPU4_CMD_ENUM_NODES:
			pr_debug("%s process_enum_nodes %d",
						__func__, req->op[0]);
			kthread_run(process_enum_nodes_thread, req_info,
								"process_enum_nodes");
			req->stat = IPU4_REQ_PENDING;
			break;
	case IPU4_CMD_ENUM_LINKS:
			pr_debug("%s process_enum_links %d",
						__func__, req->op[0]);
			kthread_run(process_enum_links_thread, req_info,
								"process_enum_links");
			req->stat = IPU4_REQ_PENDING;
			break;
	case IPU4_CMD_SETUP_PIPE:
			pr_debug("%s process_setup_pipe %d",
						__func__, req->op[0]);
			kthread_run(process_setup_pipe_thread, req_info,
								"process_setup_pipe");
			req->stat = IPU4_REQ_PENDING;
			break;
	case IPU4_CMD_SET_FRAMEFMT:
			pr_debug("%s process_set_framefmt %d",
						__func__, req->op[0]);
			kthread_run(process_set_framefmt_thread, req_info,
								"process_set_framefmt");
			req->stat = IPU4_REQ_PENDING;
			break;
	case IPU4_CMD_GET_FRAMEFMT:
			pr_debug("%s process_get_framefmt %d",
						__func__, req->op[0]);
			kthread_run(process_get_framefmt_thread, req_info,
								"process_get_framefmt");
			req->stat = IPU4_REQ_PENDING;
			break;
	case IPU4_CMD_GET_SUPPORTED_FRAMEFMT:
			pr_debug("%s process_get_supported_framefmt %d",
						__func__, req->op[0]);
			kthread_run(process_get_supported_framefmt_thread,
				req_info, "process_get_supported_framefmt");
			req->stat = IPU4_REQ_PENDING;
			break;
	case IPU4_CMD_SET_SELECTION:
			pr_debug("%s process_pad_set_sel %d",
						__func__, req->op[0]);
			kthread_run(process_pad_set_sel_thread, req_info,
								"process_pad_set_sel");
			req->stat = IPU4_REQ_PENDING;
			break;
	case IPU4_CMD_GET_SELECTION:
			pr_debug("%s process_pad_get_sel %d",
						__func__, req->op[0]);
			kthread_run(process_pad_get_sel_thread, req_info,
								"process_pad_get_sel");
			req->stat = IPU4_REQ_PENDING;
			break;
	default:
			return -EINVAL;
		}

	return ret;
}
