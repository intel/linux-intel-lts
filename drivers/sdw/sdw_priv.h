/*
 *  sdw_priv.h - Private definition for sdw bus interface.
 *
 *  Copyright (C) 2014-2015 Intel Corp
 *  Author:  Hardik Shah  <hardik.t.shah@intel.com>
 *
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 */

#ifndef _LINUX_SDW_PRIV_H
#define _LINUX_SDW_PRIV_H

#include <linux/kthread.h>	/* For kthread */
#include <linux/spinlock.h>

#define SDW_MAX_STREAM_TAG_KEY_SIZE	80
#define SDW_NUM_STREAM_TAGS		100
#define MAX_NUM_ROWS			23 /* As per SDW Spec */
#define MAX_NUM_COLS			8/* As per SDW Spec */
#define MAX_NUM_ROW_COLS		(MAX_NUM_ROWS * MAX_NUM_COLS)

#define SDW_STATE_INIT_STREAM_TAG	    0x1
#define SDW_STATE_ALLOC_STREAM              0x2
#define SDW_STATE_CONFIG_STREAM             0x3
#define SDW_STATE_COMPUTE_STREAM	    0x4
#define SDW_STATE_PREPARE_STREAM            0x5
#define SDW_STATE_ENABLE_STREAM             0x6
#define SDW_STATE_DISABLE_STREAM            0x7
#define SDW_STATE_UNPREPARE_STREAM          0x8
#define SDW_STATE_UNCOMPUTE_STREAM	    0x9
#define SDW_STATE_RELEASE_STREAM            0xa
#define SDW_STATE_FREE_STREAM               0xb
#define SDW_STATE_FREE_STREAM_TAG           0xc
#define SDW_STATE_ONLY_XPORT_STREAM	    0xd

#define SDW_STATE_INIT_RT		0x1
#define SDW_STATE_CONFIG_RT		0x2
#define SDW_STATE_PREPARE_RT		0x3
#define SDW_STATE_ENABLE_RT		0x4
#define SDW_STATE_DISABLE_RT		0x5
#define SDW_STATE_UNPREPARE_RT		0x6
#define SDW_STATE_RELEASE_RT		0x7

struct sdw_runtime;
/* Defined in sdw.c, used by multiple files of module */
extern struct sdw_core sdw_core;

#endif /* _LINUX_SDW_PRIV_H */
