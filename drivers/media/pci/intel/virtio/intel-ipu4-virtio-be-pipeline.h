/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0) */
/*
 * Copyright (C) 2018 Intel Corporation
 */

#ifndef __IPU4_VIRTIO_BE_PIPELINE__
#define __IPU4_VIRTIO_BE_PIPELINE__

#include <linux/kernel.h>
#include <linux/errno.h>

#include "intel-ipu4-virtio-common.h"

int process_pipeline_open_thread(void *data);
int process_pipeline_close_thread(void *data);
int process_enum_nodes_thread(void *data);
int process_enum_links_thread(void *data);
int process_get_supported_framefmt_thread(void *data);
int process_set_framefmt_thread(void *data);
int process_get_framefmt_thread(void *data);
int process_pad_set_sel_thread(void *data);
int process_pad_get_sel_thread(void *data);
int process_setup_pipe_thread(void *data);

#endif


