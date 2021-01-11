/* SPDX-License-Identifier: GPL-2.0-only
 * RESMGR driver  -  VPU Context Manager
 * Copyright (C) 2020-2021 Intel Corporation
 */
#ifndef __VPU_VCM_H
#define __VPU_VCM_H

#include <linux/xlink.h>

struct vpumgr_device;

/* Command code for message to/from VPU context manager on firmware */
#define VCTX_MSG_CREATE             1
#define VCTX_MSG_DESTROY            2
#define VCTX_MSG_REPLY              3

/* Maximal payload size supported for request or reply */
#define VCM_PAYLOAD_SIZE            (8192 - 5 * sizeof(u32))

/**
 * struct vcm_msg: VPU context manager message
 * @size: size tag must be at the begin
 * @ctx: to/from which context the msg is
 * @cmd: the type of this message
 * @id: index to identify this message in context ctx
 * @rc: return code or misc args
 * @payload: the payload of message
 */
struct vcm_msg {
	u32 size;
	u32 ctx;
	u32 cmd;
	u32 id;
	s32 rc;
	union {
		char data[VCM_PAYLOAD_SIZE];
	} payload;
} __packed;

struct vcm_dev {
	bool enabled;
	/*
	 * XLINK IPC related.
	 */
	struct xlink_handle ipc_xlink_handle;
	s32 sw_dev_id;

	/*
	 * dispatch work queue.
	 * Xlink transcations would handled in the work queue;
	 * Decouple the xlink API invoking with user space systemcall
	 * because SIGINT would cause xlink_read* erro.
	 */
	struct workqueue_struct *wq;

	/* kthread for rx */
	struct task_struct *rxthread;

	/* message buffer for receiving thread */
	struct vcm_msg rxmsg;

	struct mutex msg_idr_lock; /* protects msg_idr */
	struct idr msg_idr;

	struct mutex fwboot_mutex; /* protect firmware boot-up */
	int fwuser_cnt;

	bool xlink_closing;
};

struct vpumgr_ctx {
	struct vpumgr_device *vdev;
	u32 vpu_ctx_id;
	u64 total_vcmds;
};

int vcm_init(struct vpumgr_device *vdev, u32 sw_dev_id);
int vcm_fini(struct vpumgr_device *vdev);

int vcm_open(struct vpumgr_ctx *v, struct vpumgr_device *vdev);
int vcm_close(struct vpumgr_ctx *v);
int vcm_debugfs_stats_show(struct seq_file *file, struct vpumgr_ctx *v);

int vcm_submit(struct vpumgr_ctx *v,
	       u32 cmd, const void *data_in, u32 in_len, s32 *submit_id);
int vcm_wait(struct vpumgr_ctx *v, s32 submit_id,
	     s32 *vpu_rc, void *data_out, u32 *p_out_len, u32 timeout_ms);

#endif /* __VPU_VCM_H */
