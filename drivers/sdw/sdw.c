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


struct bus_type sdw_bus_type = {
	.name		= "soundwire",
};
EXPORT_SYMBOL_GPL(sdw_bus_type);

struct device sdw_slv = {
	.init_name = "soundwire",
};

static void sdw_exit(void)
{
	device_unregister(&sdw_slv);
	bus_unregister(&sdw_bus_type);
}

static int sdw_init(void)
{
	int retval;
	int i;

	for (i = 0; i < SDW_NUM_STREAM_TAGS; i++)
		sdw_core.stream_tags[i].stream_tag = i;
	mutex_init(&sdw_core.core_lock);
	INIT_LIST_HEAD(&sdw_core.bus_list);
	idr_init(&sdw_core.idr);
	retval = bus_register(&sdw_bus_type);

	if (!retval)
		retval = device_register(&sdw_slv);


	if (retval)
		bus_unregister(&sdw_bus_type);

	return retval;
}
postcore_initcall(sdw_init);
module_exit(sdw_exit);

MODULE_AUTHOR("Hardik Shah <hardik.t.shah@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.1");
MODULE_DESCRIPTION("SoundWire bus driver");
MODULE_ALIAS("platform:soundwire");
