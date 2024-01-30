/* SPDX-License-Identifier: MIT */
/*
 * Copyright(c) 2020 - 2021 Intel Corporation.
 *
 */

#ifndef SYSFS_H_INCLUDED
#define SYSFS_H_INCLUDED

#include "iaf_drv.h"

void iaf_sysfs_init(struct fdev *fdev);
void iaf_sysfs_remove(struct fdev *fdev);
int iaf_sysfs_probe(struct fdev *fdev);

#endif
