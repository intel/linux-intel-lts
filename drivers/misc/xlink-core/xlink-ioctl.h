/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * xlink ioctl header files.
 *
 * Copyright (C) 2018-2019 Intel Corporation
 *
 */

#ifndef XLINK_IOCTL_H_
#define XLINK_IOCTL_H_

int ioctl_connect(unsigned long arg);
int ioctl_open_channel(unsigned long arg, void *sess_ctx);
int ioctl_read_data(unsigned long arg);
int ioctl_write_data(unsigned long arg);
int ioctl_write_volatile_data(unsigned long arg);
int ioctl_release_data(unsigned long arg);
int ioctl_close_channel(unsigned long arg, void *sess_ctx);
int ioctl_disconnect(unsigned long arg);

#endif /* XLINK_IOCTL_H_ */
