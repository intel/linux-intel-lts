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
int ioctl_open_channel(unsigned long arg);
int ioctl_read_data(unsigned long arg);
int ioctl_read_to_buffer(unsigned long arg);
int ioctl_write_data(unsigned long arg);
int ioctl_write_control_data(unsigned long arg);
int ioctl_write_volatile_data(unsigned long arg);
int ioctl_release_data(unsigned long arg);
int ioctl_close_channel(unsigned long arg);
int ioctl_start_vpu(unsigned long arg);
int ioctl_stop_vpu(void);
int ioctl_disconnect(unsigned long arg);
int ioctl_get_device_name(unsigned long arg);
int ioctl_get_device_list(unsigned long arg);
int ioctl_get_device_status(unsigned long arg);
int ioctl_boot_device(unsigned long arg);
int ioctl_reset_device(unsigned long arg);
int ioctl_get_device_mode(unsigned long arg);
int ioctl_set_device_mode(unsigned long arg);
int ioctl_register_device_event(unsigned long arg);
int ioctl_unregister_device_event(unsigned long arg);
int ioctl_data_ready_callback(unsigned long arg);
int ioctl_data_consumed_callback(unsigned long arg);

#endif /* XLINK_IOCTL_H_ */
