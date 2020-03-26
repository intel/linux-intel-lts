/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */
/*
 * keembay-vpu-ipc.h - KeemBay VPU IPC Linux Kernel API
 *
 * Copyright (c) 2018 Intel Corporation.
 */

#ifndef __KEEMBAY_VPU_IPC_H
#define __KEEMBAY_VPU_IPC_H

#include "linux/types.h"

/* The possible node IDs. */
enum {
	KMB_VPU_IPC_NODE_ARM_CSS = 0,
	KMB_VPU_IPC_NODE_LEON_MSS,
};

/* Possible states of VPU. */
enum intel_keembay_vpu_state {
	KEEMBAY_VPU_OFF = 0,
	KEEMBAY_VPU_BUSY,
	KEEMBAY_VPU_READY,
	KEEMBAY_VPU_ERROR,
	KEEMBAY_VPU_STOPPING
};

/* Possible CPU IDs for which we receive WDT timeout events. */
enum intel_keembay_wdt_cpu_id {
	KEEMBAY_VPU_MSS = 0,
	KEEMBAY_VPU_NCE
};

/* Events that can be notified via callback, when registered. */
enum intel_keembay_vpu_event {
	KEEMBAY_VPU_NOTIFY_DISCONNECT = 0,
	KEEMBAY_VPU_NOTIFY_CONNECT,
	KEEMBAY_VPU_NOTIFY_MSS_WDT,
	KEEMBAY_VPU_NOTIFY_NCE_WDT,
};

int intel_keembay_vpu_ipc_open_channel(struct device *dev, u8 node_id,
				       u16 chan_id);
int intel_keembay_vpu_ipc_close_channel(struct device *dev, u8 node_id,
					u16 chan_id);
int intel_keembay_vpu_ipc_send(struct device *dev, u8 node_id, u16 chan_id,
			       u32 paddr, size_t size);
int intel_keembay_vpu_ipc_recv(struct device *dev, u8 node_id, u16 chan_id,
			       u32 *paddr, size_t *size, u32 timeout);
int intel_keembay_vpu_startup(struct device *dev, const char *firmware_name);
int intel_keembay_vpu_reset(struct device *dev);
int intel_keembay_vpu_stop(struct device *dev);
enum intel_keembay_vpu_state intel_keembay_vpu_status(struct device *dev);
int intel_keembay_vpu_get_wdt_count(struct device *dev,
				    enum intel_keembay_wdt_cpu_id id);
int intel_keembay_vpu_wait_for_ready(struct device *dev, u32 timeout);
int intel_keembay_vpu_register_for_events(struct device *dev,
					  void (*callback)(struct device *dev,
							   enum intel_keembay_vpu_event event));
int intel_keembay_vpu_unregister_for_events(struct device *dev);

#endif /* __KEEMBAY_VPU_IPC_H */
