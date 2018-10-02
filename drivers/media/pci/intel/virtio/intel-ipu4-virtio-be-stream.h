/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0) */
/*
 * Copyright (C) 2018 Intel Corporation
 */

#ifndef __IPU4_VIRTIO_BE_STREAM__
#define __IPU4_VIRTIO_BE_STREAM__

#include <linux/kernel.h>
#include <linux/errno.h>

#include "intel-ipu4-virtio-common.h"
#include "intel-ipu4-virtio-be-request-queue.h"

int process_set_format_thread(void *data);
int process_device_open_thread(void *data);
int process_device_close_thread(void *data);
int process_poll_thread(void *data);
int process_put_buf_thread(void *data);
int process_stream_on_thread(void *data);
int process_stream_off_thread(void *data);
int process_get_buf_thread(void *data);

#endif


