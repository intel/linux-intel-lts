/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * xlink Multiplexer.
 *
 * Copyright (C) 2018-2019 Intel Corporation
 *
 */
#ifndef __XLINK_MULTIPLEXER_H
#define __XLINK_MULTIPLEXER_H

#include "xlink-defs.h"

enum xlink_error xlink_multiplexer_init(void *dev);

enum xlink_error xlink_multiplexer_connect(u32 link_id);

enum xlink_error xlink_multiplexer_disconnect(u32 link_id);

enum xlink_error xlink_multiplexer_destroy(void);

enum xlink_error xlink_multiplexer_tx(struct xlink_event *event,
				      int *event_queued);

enum xlink_error xlink_multiplexer_rx(struct xlink_event *event);

enum xlink_error xlink_passthrough(struct xlink_event *event);

void *find_allocated_buffer(dma_addr_t paddr);

int unregister_allocated_buffer(void *buf, dma_addr_t paddr);

int core_release_packet_from_channel(u32 link_id, uint16_t chan,
				     uint8_t * const addr);

#endif /* __XLINK_MULTIPLEXER_H */
