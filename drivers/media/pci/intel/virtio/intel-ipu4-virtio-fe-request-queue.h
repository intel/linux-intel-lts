/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0) */
/*
 * Copyright (C) 2018 Intel Corporation
 */

#ifndef IPU4_VIRTIO_FE_REQUEST_QUEUE_H
#define IPU4_VIRTIO_FE_REQUEST_QUEUE_H

int ipu4_virtio_fe_req_queue_init(void);
void ipu4_virtio_fe_req_queue_free(void);
struct ipu4_virtio_req *ipu4_virtio_fe_req_queue_get(void);
int ipu4_virtio_fe_req_queue_put(struct ipu4_virtio_req *req);

#endif
