/* SPDX-License-Identifier: MIT */
/*
 * Copyright(c) 2020 - 2021 Intel Corporation.
 *
 */

#ifndef ROUTING_PAUSE_H_INCLUDED
#define ROUTING_PAUSE_H_INCLUDED

struct routing_pause_ctx;

struct routing_pause_ctx *routing_pause_init(void);
void routing_pause_start(struct routing_pause_ctx *ctx);
void routing_pause_end(struct routing_pause_ctx *ctx);

#endif /* ROUTING_PAUSE_H_INCLUDED */
