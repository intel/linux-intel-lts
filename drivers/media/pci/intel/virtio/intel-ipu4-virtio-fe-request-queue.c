// SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
/*
 * Copyright (C) 2018 Intel Corporation
 */

#include <linux/virtio.h>
#include <linux/spinlock.h>
#include "intel-ipu4-virtio-common.h"
#include "intel-ipu4-virtio-fe-request-queue.h"

struct ipu4_virtio_ring ipu4_virtio_fe_req_queue;

int ipu4_virtio_fe_req_queue_init(void)
{
	int i;
	struct ipu4_virtio_req *req;

	if (ipu4_virtio_ring_init(&ipu4_virtio_fe_req_queue, REQ_RING_SIZE))
		return -1;

	for (i = 0; i < REQ_RING_SIZE; i++) {
		req = kcalloc(1, sizeof(struct ipu4_virtio_req), GFP_KERNEL);
		if (req == NULL) {
			pr_err("%s failed to allocate memory for ipu4_virtio_req",
														__func__);
			return -1;
		}
		init_completion(&req->wait);
		ipu4_virtio_ring_push(&ipu4_virtio_fe_req_queue, req);
	}
	return 0;
}

void ipu4_virtio_fe_req_queue_free(void)
{
	int i;
	struct ipu4_virtio_req *req;

	for (i = 0; i < REQ_RING_SIZE; i++) {
		req = ipu4_virtio_ring_pop(&ipu4_virtio_fe_req_queue);
		if (req)
			kfree(req);
		else
			break;
	}
	ipu4_virtio_ring_free(&ipu4_virtio_fe_req_queue);
}

struct ipu4_virtio_req *ipu4_virtio_fe_req_queue_get(void)
{
	struct ipu4_virtio_req *req;
	unsigned long flags = 0;

	spin_lock_irqsave(&ipu4_virtio_fe_req_queue.lock, flags);
	req = ipu4_virtio_ring_pop(&ipu4_virtio_fe_req_queue);
	spin_unlock_irqrestore(&ipu4_virtio_fe_req_queue.lock, flags);
	if (req)
		reinit_completion(&req->wait);
	return req;
}

int ipu4_virtio_fe_req_queue_put(
			struct ipu4_virtio_req *req)
{
	unsigned long flags = 0;
	int status;

	spin_lock_irqsave(&ipu4_virtio_fe_req_queue.lock, flags);
	status = ipu4_virtio_ring_push(&ipu4_virtio_fe_req_queue, req);
	spin_unlock_irqrestore(&ipu4_virtio_fe_req_queue.lock, flags);
	return status;
}
