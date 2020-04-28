/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright(c) 2017-2020 Intel Corporation */

#ifndef __GNA_REQUEST_H__
#define __GNA_REQUEST_H__

#include "gna.h"

#include "gna_hw.h"

enum gna_request_state {
	NEW,
	ACTIVE,
	DONE,
};

struct gna_file_private;

struct gna_request {
	u64 request_id;

	struct kref refcount;

	struct gna_private *gna_priv;
	struct file *fd;

	/* hardware status set by interrupt handler */
	u32 hw_status;
	spinlock_t hw_lock;

	enum gna_request_state state;
	spinlock_t state_lock;

	int status;
	spinlock_t status_lock;

	struct gna_hw_perf hw_perf;
	struct gna_drv_perf drv_perf;
	spinlock_t perf_lock;

	struct list_head node;

	struct gna_compute_cfg compute_cfg;

	struct gna_buffer *buffer_list;
	u64 buffer_count;

	struct wait_queue_head waitq;
	struct work_struct work;
};

struct gna_request *gna_request_create(
	struct gna_file_private *file_priv,
	struct gna_compute_cfg *compute_cfg);

void gna_request_release(struct kref *ref);

void gna_request_set_done(struct gna_request *score_request, int status);

struct gna_request *gna_find_request_by_id(u64 req_id,
	struct gna_private *gna_priv);

void gna_delete_request_by_id(u64 req_id, struct gna_private *gna_priv);

void gna_delete_file_requests(struct file *fd, struct gna_private *gna_priv);

void gna_delete_memory_requests(u64 memory_id, struct gna_private *gna_priv);

#endif // __GNA_REQUEST_H__
