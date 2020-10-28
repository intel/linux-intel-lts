/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * xlink core header file.
 *
 * Copyright (C) 2018-2019 Intel Corporation
 *
 */

#ifndef XLINK_CORE_H_
#define XLINK_CORE_H_
#include "xlink-defs.h"

#define NUM_REG_EVENTS 4

enum xlink_error do_xlink_write_volatile(struct xlink_handle *handle,
					 u16 chan, u8 const *message,
					 u32 size, u32 user_flag);

enum session_res_type {
	SC_RES_CHAN = 0,
	SC_RES_NUM
};

struct session_context {
	struct mutex lock;  // protect access to context
	u16 chan;
	u32 sw_device_id;
	void *ptr;
	struct list_head list;
};

struct xlink_link {
	u32 id;
	struct xlink_handle handle;
	struct kref refcount;
};

struct xlink_link *get_link_by_sw_device_id(u32 sw_device_id);

enum xlink_error xlink_write_data_user(struct xlink_handle *handle,
				       u16 chan, u8 const *pmessage,
				       u32 size);
#endif /* XLINK_CORE_H_ */
