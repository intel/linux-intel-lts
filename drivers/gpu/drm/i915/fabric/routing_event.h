/* SPDX-License-Identifier: MIT */
/*
 * Copyright(c) 2019 - 2021 Intel Corporation.
 *
 */

#ifndef ROUTING_EVENT_H_INCLUDED
#define ROUTING_EVENT_H_INCLUDED

#include <linux/types.h>

void rem_init(void);
void rem_shutting_down(void);
void rem_stop(void);
bool rem_request(void);
int rem_route_start(u64 *serviced_requests);
void rem_route_finish(void);

#endif /* ROUTING_EVENT_H_INCLUDED */
