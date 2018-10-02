// SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
/*
 * Copyright (C) 2018 Intel Corporation
 */

#include <linux/virtio.h>
#include <linux/spinlock.h>
#include "intel-ipu4-virtio-common.h"
#include "intel-ipu4-virtio-be-request-queue.h"

struct ipu4_virtio_ring ipu4_virtio_be_req_queue;

int ipu4_virtio_be_req_queue_init(void)
{
	int i;
	struct ipu4_virtio_req_info *req;

	if (ipu4_virtio_ring_init(&ipu4_virtio_be_req_queue, REQ_RING_SIZE))
		return -1;

	for (i = 0; i < REQ_RING_SIZE; i++) {
		req = kcalloc(1, sizeof(struct ipu4_virtio_req_info), GFP_KERNEL);
		if (req == NULL) {
			pr_err("%s failed to allocate memory for ipu4_virtio_req_info",
														__func__);
			return -1;
		}
		ipu4_virtio_ring_push(&ipu4_virtio_be_req_queue, req);
	}
	return 0;
}

void ipu4_virtio_be_req_queue_free(void)
{
	int i;
	struct ipu4_virtio_req_info *req_info;

	for (i = 0; i < REQ_RING_SIZE; i++) {
		req_info = ipu4_virtio_ring_pop(&ipu4_virtio_be_req_queue);
		if (req_info)
			kfree(req_info);
		else
			break;
	}
	ipu4_virtio_ring_free(&ipu4_virtio_be_req_queue);
}

struct ipu4_virtio_req_info *ipu4_virtio_be_req_queue_get(void)
{
	return ipu4_virtio_ring_pop(&ipu4_virtio_be_req_queue);
}

int ipu4_virtio_be_req_queue_put(
			struct ipu4_virtio_req_info *req)
{
	return ipu4_virtio_ring_push(&ipu4_virtio_be_req_queue, req);
}
