/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0) */
/*
 * Copyright (C) 2018 Intel Corporation
 */

#ifndef __IPU4_VIRTIO_BE_PSYS__
#define __IPU4_VIRTIO_BE_PSYS__

int process_set_format_thread(void *data);
int process_device_open_thread(void *data);
int process_device_close_thread(void *data);
int process_poll_thread(void *data);
int process_put_buf_thread(void *data);
int process_stream_on_thread(void *data);
int process_stream_off_thread(void *data);
int process_get_buf_thread(void *data);

int process_psys_mapbuf_thread(void *data);
int process_psys_unmapbuf_thread(void *data);
int process_psys_querycap_thread(void *data);
int process_psys_putbuf_thread(void *data);
int process_psys_qcmd_thread(void *data);
int process_psys_dqevent_thread(void *data);
int process_psys_get_manifest_thread(void *data);
int process_psys_open_thread(void *data);
int process_psys_close_thread(void *data);
int process_psys_poll_thread(void *data);
int process_psys_getbuf_thread(void *data);

#endif
