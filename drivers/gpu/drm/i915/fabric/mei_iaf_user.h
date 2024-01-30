/* SPDX-License-Identifier: MIT */
/*
 * Copyright(c) 2020 - 2021 Intel Corporation.
 */

#ifndef _MEI_IAF_USER_H_
#define _MEI_IAF_USER_H_

#include <linux/device.h>

#include "iaf_drv.h"

int iaf_mei_start(struct fdev *dev);
void iaf_mei_stop(struct fdev *dev);
void iaf_mei_indicate_device_ok(struct fdev *dev);
int iaf_commit_svn(struct fdev *dev);
u16 get_min_svn(struct fdev *dev);

#endif
