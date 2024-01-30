/* SPDX-License-Identifier: MIT */
/*
 * Copyright(c) 2020 - 2022 Intel Corporation.
 */

#ifndef ROUTING_DEBUG_H_INCLUDED
#define ROUTING_DEBUG_H_INCLUDED

#include <linux/debugfs.h>

#include "iaf_drv.h"

void routing_debug_port_init(struct fsubdev *sd, u8 lpn);

#endif /* ROUTING_DEBUG_H_INCLUDED */

