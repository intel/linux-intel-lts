/* SPDX-License-Identifier: MIT */
/*
 * Copyright(c) 2019 - 2021 Intel Corporation.
 *
 */

#ifndef SELFTESTS_ROUTING_MOCK_H_INCLUDED
#define SELFTESTS_ROUTING_MOCK_H_INCLUDED

#if IS_ENABLED(CONFIG_IAF_DEBUG_SELFTESTS)
#include "../routing_topology.h"

#define PORT_FABRIC_START      1
#define PORT_FABRIC_END        8
#define PORT_BRIDGE_START      9
#define PORT_BRIDGE_END        (PORT_COUNT - 1)

int routing_mock_create_topology(struct routing_topology *topo);
void routing_mock_destroy(struct routing_topology *topo);
#endif

#endif /* SELFTESTS_ROUTING_MOCK_H_INCLUDED */
