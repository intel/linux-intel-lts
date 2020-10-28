/* SPDX-License-Identifier: GPL-2.0+ WITH Linux-syscall-note */
/*
 * xlink Linux Kernel API
 *
 * Copyright (C) 2018-2019 Intel Corporation
 *
 */
#ifndef __XLINK_UAPI_H
#define __XLINK_UAPI_H

#include <linux/types.h>

#define XLINK_MAGIC 'x'
#define XL_OPEN_CHANNEL			_IOW(XLINK_MAGIC, 1, void*)
#define XL_READ_DATA			_IOW(XLINK_MAGIC, 2, void*)
#define XL_WRITE_DATA			_IOW(XLINK_MAGIC, 3, void*)
#define XL_CLOSE_CHANNEL		_IOW(XLINK_MAGIC, 4, void*)
#define XL_WRITE_VOLATILE		_IOW(XLINK_MAGIC, 5, void*)
#define XL_READ_TO_BUFFER		_IOW(XLINK_MAGIC, 6, void*)
#define XL_START_VPU			_IOW(XLINK_MAGIC, 7, void*)
#define XL_STOP_VPU				_IOW(XLINK_MAGIC, 8, void*)
#define XL_RESET_VPU			_IOW(XLINK_MAGIC, 9, void*)
#define XL_CONNECT				_IOW(XLINK_MAGIC, 10, void*)
#define XL_RELEASE_DATA			_IOW(XLINK_MAGIC, 11, void*)
#define XL_DISCONNECT			_IOW(XLINK_MAGIC, 12, void*)
#define XL_WRITE_CONTROL_DATA	_IOW(XLINK_MAGIC, 13, void*)
#define XL_DATA_READY_CALLBACK	_IOW(XLINK_MAGIC, 14, void*)
#define XL_DATA_CONSUMED_CALLBACK	_IOW(XLINK_MAGIC, 15, void*)
#define XL_GET_DEVICE_NAME		_IOW(XLINK_MAGIC, 16, void*)
#define XL_GET_DEVICE_LIST		_IOW(XLINK_MAGIC, 17, void*)
#define XL_GET_DEVICE_STATUS	_IOW(XLINK_MAGIC, 18, void*)
#define XL_BOOT_DEVICE			_IOW(XLINK_MAGIC, 19, void*)
#define XL_RESET_DEVICE			_IOW(XLINK_MAGIC, 20, void*)
#define XL_GET_DEVICE_MODE		_IOW(XLINK_MAGIC, 21, void*)
#define XL_SET_DEVICE_MODE		_IOW(XLINK_MAGIC, 22, void*)
#define XL_REGISTER_DEV_EVENT	_IOW(XLINK_MAGIC, 23, void*)
#define XL_UNREGISTER_DEV_EVENT	_IOW(XLINK_MAGIC, 24, void*)

struct xlinkopenchannel {
	void *handle;
	__u16 chan;
	int mode;
	__u32 data_size;
	__u32 timeout;
	__u32 *return_code;
};

struct xlinkcallback {
	void *handle;
	__u16 chan;
	void (*callback)(__u16 chan);
	__u32 *return_code;
};

struct xlinkwritedata {
	void *handle;
	__u16 chan;
	void const *pmessage;
	__u32 size;
	__u32 *return_code;
};

struct xlinkreaddata {
	void *handle;
	__u16 chan;
	void *pmessage;
	__u32 *size;
	__u32 *return_code;
};

struct xlinkreadtobuffer {
	void *handle;
	__u16 chan;
	void *pmessage;
	__u32 *size;
	__u32 *return_code;
};

struct xlinkconnect {
	void *handle;
	__u32 *return_code;
};

struct xlinkrelease {
	void *handle;
	__u16 chan;
	void *addr;
	__u32 *return_code;
};

struct xlinkstartvpu {
	char *filename;
	int namesize;
	__u32 *return_code;
};

struct xlinkstopvpu {
	__u32 *return_code;
};

struct xlinkgetdevicename {
	void *handle;
	char *name;
	__u32 name_size;
	__u32 *return_code;
};

struct xlinkgetdevicelist {
	__u32 *sw_device_id_list;
	__u32 *num_devices;
	__u32 *return_code;
};

struct xlinkgetdevicestatus {
	void *handle;
	__u32 *device_status;
	__u32 *return_code;
};

struct xlinkbootdevice {
	void *handle;
	const char *binary_name;
	__u32 binary_name_size;
	__u32 *return_code;
};

struct xlinkresetdevice {
	void *handle;
	__u32 *return_code;
};

struct xlinkdevmode {
	void *handle;
	int *device_mode;
	__u32 *return_code;
};

struct xlinkregdevevent {
	void *handle;
	__u32  *event_list;
	__u32  num_events;
	__u32 *return_code;
};

#endif /* __XLINK_UAPI_H */
