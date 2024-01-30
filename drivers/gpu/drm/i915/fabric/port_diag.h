/* SPDX-License-Identifier: MIT */
/*
 * Copyright(c) 2021 - 2022 Intel Corporation.
 *
 */

#ifndef PORT_DIAG_H_INCLUDED
#define PORT_DIAG_H_INCLUDED

#include <linux/dcache.h>
#include <linux/types.h>

#include "iaf_drv.h"

void create_fabric_port_debugfs_files(struct fsubdev *sd, struct fport *port);
void create_bridge_port_debugfs_files(struct fsubdev *sd, struct fport *port);
void create_port_debugfs_dir(struct fsubdev *sd, u8 lpn);

#endif
