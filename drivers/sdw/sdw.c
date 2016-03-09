/*
 *  sdw.c - SoundWire Bus driver implementation
 *
 *  Copyright (C) 2015-2016 Intel Corp
 *  Author:  Hardik T Shah <hardik.t.shah@intel.com>
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/idr.h>
#include <linux/delay.h>
#include <linux/rtmutex.h>
#include <linux/pm_runtime.h>
#include <linux/pm_domain.h>
#include <linux/pm.h>
#include <linux/mod_devicetable.h>
#include <linux/sdw_bus.h>
#include <linux/sdw/sdw_registers.h>
#include "sdw_priv.h"

/* Global instance handling all the SoundWire buses */
struct sdw_core sdw_core;
