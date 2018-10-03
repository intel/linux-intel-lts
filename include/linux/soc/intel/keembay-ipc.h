/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Keem Bay IPC Linux Kernel API
 *
 * Copyright (C) 2018-2020 Intel Corporation
 */

#ifndef __KEEMBAY_IPC_H
#define __KEEMBAY_IPC_H

#include <linux/types.h>

/* The possible node IDs. */
enum {
	KMB_IPC_NODE_ARM_CSS = 0,
	KMB_IPC_NODE_LEON_MSS,
};

int intel_keembay_ipc_open_channel(struct device *dev, u8 node_id, u16 chan_id);

int intel_keembay_ipc_close_channel(struct device *dev, u8 node_id,
				    u16 chan_id);

int intel_keembay_ipc_send(struct device *dev, u8 node_id, u16 chan_id,
			   u32 vpu_addr, size_t size);

int intel_keembay_ipc_recv(struct device *dev, u8 node_id, u16 chan_id,
			   u32 *vpu_addr, size_t *size, u32 timeout);

#endif /* __KEEMBAY_IPC_H */
