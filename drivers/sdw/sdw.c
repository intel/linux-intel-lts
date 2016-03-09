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

#define sdw_slave_attr_gr NULL
#define sdw_mstr_attr_gr NULL


/* Global instance handling all the SoundWire buses */
struct sdw_core sdw_core;

static void sdw_slave_release(struct device *dev)
{
	kfree(to_sdw_slave(dev));
}

static void sdw_mstr_release(struct device *dev)
{
	struct sdw_master *mstr = to_sdw_master(dev);

	complete(&mstr->slv_released);
}

static struct device_type sdw_slv_type = {
	.groups		= sdw_slave_attr_gr,
	.release	= sdw_slave_release,
};

static struct device_type sdw_mstr_type = {
	.groups		= sdw_mstr_attr_gr,
	.release	= sdw_mstr_release,
};
/**
 * sdw_slave_verify - return parameter as sdw_slave, or NULL
 * @dev: device, probably from some driver model iterator
 *
 * When traversing the driver model tree, perhaps using driver model
 * iterators like @device_for_each_child(), you can't assume very much
 * about the nodes you find.  Use this function to avoid oopses caused
 * by wrongly treating some non-SDW device as an sdw_slave.
 */
struct sdw_slave *sdw_slave_verify(struct device *dev)
{
	return (dev->type == &sdw_slv_type)
			? to_sdw_slave(dev)
			: NULL;
}

/**
 * sdw_mstr_verify - return parameter as sdw_master, or NULL
 * @dev: device, probably from some driver model iterator
 *
 * When traversing the driver model tree, perhaps using driver model
 * iterators like @device_for_each_child(), you can't assume very much
 * about the nodes you find.  Use this function to avoid oopses caused
 * by wrongly treating some non-SDW device as an sdw_slave.
 */
struct sdw_master *sdw_mstr_verify(struct device *dev)
{
	return (dev->type == &sdw_mstr_type)
			? to_sdw_master(dev)
			: NULL;
}

static const struct sdw_slave_id *sdw_match_slave(const struct sdw_slave_id *id,
					const struct sdw_slave *sdw_slv)
{
	while (id->name[0]) {
		if (strncmp(sdw_slv->name, id->name, SOUNDWIRE_NAME_SIZE) == 0)
			return id;
		id++;
	}
	return NULL;
}

static const struct sdw_master_id *sdw_match_master(
			const struct sdw_master_id *id,
			const struct sdw_master *sdw_mstr)
{
	if (!id)
		return NULL;
	while (id->name[0]) {
		if (strncmp(sdw_mstr->name, id->name, SOUNDWIRE_NAME_SIZE) == 0)
			return id;
		id++;
	}
	return NULL;
}

static int sdw_slv_match(struct device *dev, struct device_driver *driver)
{
	struct sdw_slave *sdw_slv;
	struct sdw_slave_driver *drv = to_sdw_slave_driver(driver);
	int ret = 0;

	/* Check if driver is slave type or not, both master and slave
	 * driver has first field as driver_type, so if driver is not
	 * of slave type return
	 */
	if (drv->driver_type !=  SDW_DRIVER_TYPE_SLAVE)
		return ret;

	sdw_slv = to_sdw_slave(dev);

	if (drv->id_table)
		ret = (sdw_match_slave(drv->id_table, sdw_slv) != NULL);

	if (driver->name && !ret)
		ret = (strncmp(sdw_slv->name, driver->name, SOUNDWIRE_NAME_SIZE)
			== 0);
	if (ret)
		sdw_slv->driver = drv;
	return ret;
}
static int sdw_mstr_match(struct device *dev, struct device_driver *driver)
{
	struct sdw_master *sdw_mstr;
	struct sdw_mstr_driver *drv = to_sdw_mstr_driver(driver);
	int ret = 0;

	/* Check if driver is slave type or not, both master and slave
	 * driver has first field as driver_type, so if driver is not
	 * of slave type return
	 */
	if (drv->driver_type !=  SDW_DRIVER_TYPE_MASTER)
		return ret;

	sdw_mstr = to_sdw_master(dev);

	if (drv->id_table)
		ret =  (sdw_match_master(drv->id_table, sdw_mstr) != NULL);

	if (driver->name)
		ret = (strncmp(sdw_mstr->name, driver->name,
			SOUNDWIRE_NAME_SIZE) == 0);
	if (ret)
		sdw_mstr->driver = drv;

	return ret;
}

static int sdw_mstr_probe(struct device *dev)
{
	const struct sdw_mstr_driver *sdrv = to_sdw_mstr_driver(dev->driver);
	struct sdw_master *mstr = to_sdw_master(dev);
	int ret = 0;

	if (!sdrv->probe)
		return -ENODEV;
	ret = dev_pm_domain_attach(dev, true);
	if (ret != -EPROBE_DEFER) {
		ret = sdrv->probe(mstr, sdw_match_master(sdrv->id_table, mstr));
		if (ret)
			dev_pm_domain_detach(dev, true);
	}
	return ret;
}

static int sdw_slv_probe(struct device *dev)
{
	const struct sdw_slave_driver *sdrv = to_sdw_slave_driver(dev->driver);
	struct sdw_slave *sdwslv = to_sdw_slave(dev);
	int ret = 0;

	if (!sdrv->probe)
		return -ENODEV;
	ret = dev_pm_domain_attach(dev, true);
	if (ret != -EPROBE_DEFER) {
		ret = sdrv->probe(sdwslv, sdw_match_slave(sdrv->id_table,
								sdwslv));
		return 0;
		if (ret)
			dev_pm_domain_detach(dev, true);
	}
	return ret;
}

static int sdw_mstr_remove(struct device *dev)
{
	const struct sdw_mstr_driver *sdrv = to_sdw_mstr_driver(dev->driver);
	int ret = 0;

	if (sdrv->remove)
		ret = sdrv->remove(to_sdw_master(dev));
	else
		return -ENODEV;

	dev_pm_domain_detach(dev, true);
	return ret;

}

static int sdw_slv_remove(struct device *dev)
{
	const struct sdw_slave_driver *sdrv = to_sdw_slave_driver(dev->driver);
	int ret = 0;

	if (sdrv->remove)
		ret = sdrv->remove(to_sdw_slave(dev));
	else
		return -ENODEV;

	dev_pm_domain_detach(dev, true);
	return ret;
}

static int sdw_remove(struct device *dev)
{
	struct sdw_slave *sdw_slv;
	struct sdw_master *sdw_mstr;

	sdw_slv = sdw_slave_verify(dev);
	sdw_mstr = sdw_mstr_verify(dev);
	if (sdw_slv)
		return sdw_slv_remove(dev);
	else if (sdw_mstr)
		return sdw_mstr_remove(dev);

	return 0;
}

static int sdw_probe(struct device *dev)
{

	struct sdw_slave *sdw_slv;
	struct sdw_master *sdw_mstr;

	sdw_slv = sdw_slave_verify(dev);
	sdw_mstr = sdw_mstr_verify(dev);
	if (sdw_slv)
		return sdw_slv_probe(dev);
	else if (sdw_mstr)
		return sdw_mstr_probe(dev);

	return -ENODEV;

}

static int sdw_match(struct device *dev, struct device_driver *driver)
{
	struct sdw_slave *sdw_slv;
	struct sdw_master *sdw_mstr;

	sdw_slv = sdw_slave_verify(dev);
	sdw_mstr = sdw_mstr_verify(dev);
	if (sdw_slv)
		return sdw_slv_match(dev, driver);
	else if (sdw_mstr)
		return sdw_mstr_match(dev, driver);
	return 0;

}

struct bus_type sdw_bus_type = {
	.name		= "soundwire",
	.match		= sdw_match,
	.probe		= sdw_probe,
	.remove		= sdw_remove,
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
