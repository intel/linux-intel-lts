/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright(c) 2017-2020 Intel Corporation */

#ifndef __GNA_SCORE_H__
#define __GNA_SCORE_H__

#include "gna.h"

#define ROUND_UP(x, n) (((x)+(n)-1u) & ~((n)-1u))
#define ROUND_DOWN(value, boundary) ((value) & (~((boundary)-1)))

struct gna_private;
struct gna_file_private;
struct gna_request;

/* validate user request */
int gna_validate_score_config(struct gna_compute_cfg *compute_cfg,
	struct gna_file_private *file_priv);

/* add request to the list */
int gna_request_enqueue(struct gna_compute_cfg *compute_cfg,
	struct gna_file_private *file_priv, u64 *request_id);

/* interrupt related functions */
void gna_isr_timeout(struct timer_list *timer);
void gna_request_tasklet(unsigned long priv);

/* scoring helper functions */
int gna_priv_score(struct gna_request *score_request);

int gna_score_wait(struct gna_request *score_request, unsigned int timeout);

#endif // __GNA_SCORE_H__
