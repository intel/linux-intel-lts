/* SPDX-License-Identifier: MIT */
/*
 * Copyright(c) 2019 - 2021 Intel Corporation.
 *
 */

#ifndef ROUTING_IO_H_INCLUDED
#define ROUTING_IO_H_INCLUDED

#include "routing_topology.h"

int routing_io_run(void);
int routing_io_sd_once(struct fsubdev *sd);

#endif /* ROUTING_IO_H_INCLUDED */
