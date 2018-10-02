/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0) */
/*
 * Copyright (C) 2018 Intel Corporation
 */

#ifndef __IPU4_VIRTIO_BE__
#define __IPU4_VIRTIO_BE__

#include <linux/vbs/vbs.h>

enum poll_status {
	IPU4_POLL_PENDING = 0,
	IPU4_POLL_AVAILABLE,
	IPU4_POLL_STOP,
	IPU4_POLL_SLEEP
};

int notify_fe(int status, struct ipu4_virtio_req_info *req_info);
void notify_poll_thread(int stream_id, enum poll_status status);

#endif
