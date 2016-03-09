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

#define CREATE_TRACE_POINTS
#include <trace/events/sdw.h>

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

static void sdw_slv_shutdown(struct device *dev)
{
	const struct sdw_slave_driver *sdrv = to_sdw_slave_driver(dev->driver);

	if (sdrv->shutdown)
		sdrv->shutdown(to_sdw_slave(dev));
}

static void sdw_mstr_shutdown(struct device *dev)
{
	const struct sdw_mstr_driver *sdrv = to_sdw_mstr_driver(dev->driver);
	struct sdw_master *mstr = to_sdw_master(dev);

	if (sdrv->shutdown)
		sdrv->shutdown(mstr);
}

static void sdw_shutdown(struct device *dev)
{
	struct sdw_slave *sdw_slv;
	struct sdw_master *sdw_mstr;

	sdw_slv = sdw_slave_verify(dev);
	sdw_mstr = sdw_mstr_verify(dev);
	if (sdw_slv)
		sdw_slv_shutdown(dev);
	else if (sdw_mstr)
		sdw_mstr_shutdown(dev);
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

#ifdef CONFIG_PM_SLEEP
static int sdw_legacy_suspend(struct device *dev, pm_message_t mesg)
{
	struct sdw_slave *sdw_slv = NULL;
	struct sdw_slave_driver *driver;

	if (dev->type == &sdw_slv_type)
		sdw_slv = to_sdw_slave(dev);

	if (!sdw_slv || !dev->driver)
		return 0;

	driver = to_sdw_slave_driver(dev->driver);
	if (!driver->suspend)
		return 0;

	return driver->suspend(sdw_slv, mesg);
}

static int sdw_legacy_resume(struct device *dev)
{
	struct sdw_slave *sdw_slv = NULL;
	struct sdw_slave_driver *driver;

	if (dev->type == &sdw_slv_type)
		sdw_slv = to_sdw_slave(dev);

	if (!sdw_slv || !dev->driver)
		return 0;

	driver = to_sdw_slave_driver(dev->driver);
	if (!driver->resume)
		return 0;

	return driver->resume(sdw_slv);
}

static int sdw_pm_suspend(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	if (pm)
		return pm_generic_suspend(dev);
	else
		return sdw_legacy_suspend(dev, PMSG_SUSPEND);
}

static int sdw_pm_resume(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	if (pm)
		return pm_generic_resume(dev);
	else
		return sdw_legacy_resume(dev);
}

static const struct dev_pm_ops soundwire_pm = {
	.suspend = sdw_pm_suspend,
	.resume = sdw_pm_resume,
	.runtime_suspend = pm_generic_runtime_suspend,
	.runtime_resume = pm_generic_runtime_resume,
};

#else
#define sdw_pm_suspend		NULL
#define sdw_pm_resume		NULL
#endif

struct bus_type sdw_bus_type = {
	.name		= "soundwire",
	.match		= sdw_match,
	.probe		= sdw_probe,
	.remove		= sdw_remove,
	.shutdown	= sdw_shutdown,
	.pm		= &soundwire_pm,
};
EXPORT_SYMBOL_GPL(sdw_bus_type);

struct device sdw_slv = {
	.init_name = "soundwire",
};

static struct static_key sdw_trace_msg = STATIC_KEY_INIT_FALSE;

int sdw_transfer_trace_reg(void)
{
	static_key_slow_inc(&sdw_trace_msg);
}

void sdw_transfer_trace_unreg(void)
{
	static_key_slow_dec(&sdw_trace_msg);
}

/**
 * sdw_lock_mstr - Get exclusive access to an SDW bus segment
 * @mstr: Target SDW bus segment
 */
void sdw_lock_mstr(struct sdw_master *mstr)
{
	rt_mutex_lock(&mstr->bus_lock);
}
EXPORT_SYMBOL_GPL(sdw_lock_mstr);

/**
 * sdw_trylock_mstr - Try to get exclusive access to an SDW bus segment
 * @mstr: Target SDW bus segment
 */
static int sdw_trylock_mstr(struct sdw_master *mstr)
{
	return rt_mutex_trylock(&mstr->bus_lock);
}


/**
 * sdw_unlock_mstr - Release exclusive access to an SDW bus segment
 * @mstr: Target SDW bus segment
 */
void sdw_unlock_mstr(struct sdw_master *mstr)
{
	rt_mutex_unlock(&mstr->bus_lock);
}
EXPORT_SYMBOL_GPL(sdw_unlock_mstr);

/**
 * __sdw_transfer - unlocked flavor of sdw_slave_transfer
 * @mstr: Handle to SDW bus
 * @msg: One or more messages to execute before STOP is issued to
 *	terminate the operation; each message begins with a START.
 * @num: Number of messages to be executed.
 *
 * Returns negative errno, else the number of messages executed.
 *
 * Adapter lock must be held when calling this function. No debug logging
 * takes place. mstr->algo->master_xfer existence isn't checked.
 */
int __sdw_transfer(struct sdw_master *mstr, struct sdw_msg *msg, int num)
{
	unsigned long orig_jiffies;
	int ret = 0, try, i;
	struct sdw_slv_capabilities *slv_cap;
	int program_scp_addr_page;
	int addr = msg->slave_addr;

	/* sdw_trace_msg gets enabled when tracepoint sdw_slave_transfer gets
	 * enabled.  This is an efficient way of keeping the for-loop from
	 * being executed when not needed.
	 */
	if (static_key_false(&sdw_trace_msg)) {
		int i;

		for (i = 0; i < num; i++)
			if (msg[i].flag & SDW_MSG_FLAG_READ)
				trace_sdw_read(mstr, &msg[i], i);
			else
				trace_sdw_write(mstr, &msg[i], i);
	}
	orig_jiffies = jiffies;
	for (i = 0; i < num; i++) {
		for (ret = 0, try = 0; try <= mstr->retries; try++) {
			if (msg->slave_addr == 0)
				/* If we are enumerating slave address 0,
				 * we dont program scp, it should be set
				 * default to 0
				 */
				program_scp_addr_page = 0;
			else if (msg->slave_addr == 15)
				/* If we are broadcasting, we need to program
				 * the SCP address as some slaves will be
				 * supporting it while some wont be.
				 * So it should be programmed
				 */
				program_scp_addr_page = 1;

			else {
				slv_cap =
					&mstr->sdw_addr[addr].slave->sdw_slv_cap;
				program_scp_addr_page =
					slv_cap->paging_supported;
			}
			ret = mstr->driver->mstr_ops->xfer_msg(mstr,
						msg, program_scp_addr_page);
			if (ret != -EAGAIN)
				break;
			if (time_after(jiffies,
					orig_jiffies + mstr->timeout))
				break;
		}
	}

	if (static_key_false(&sdw_trace_msg)) {
		int i;

		for (i = 0; i < msg->len; i++)
			if (msg[i].flag & SDW_MSG_FLAG_READ)
				trace_sdw_reply(mstr, &msg[i], i);
		trace_sdw_result(mstr, i, ret);
	}
	if (!ret)
		return i;
	return ret;
}
EXPORT_SYMBOL_GPL(__sdw_transfer);

/**
 * sdw_slave_transfer:  Transfer message between slave and mstr on the bus.
 * @mstr: mstr master which will transfer the message
 * @msg: Array of messages to be transferred.
 * @num: Number of messages to be transferred, messages include read and write
 *		messages, but not the ping messages.
 */
int sdw_slave_transfer(struct sdw_master *mstr, struct sdw_msg *msg, int num)
{
	int ret;

	/* REVISIT the fault reporting model here is weak:
	 *
	 *  - When we get an error after receiving N bytes from a slave,
	 *    there is no way to report "N".
	 *
	 *  - When we get a NAK after transmitting N bytes to a slave,
	 *    there is no way to report "N" ... or to let the mstr
	 *    continue executing the rest of this combined message, if
	 *    that's the appropriate response.
	 *
	 *  - When for example "num" is two and we successfully complete
	 *    the first message but get an error part way through the
	 *    second, it's unclear whether that should be reported as
	 *    one (discarding status on the second message) or errno
	 *    (discarding status on the first one).
	 */
	if (mstr->driver->mstr_ops->xfer_msg) {
		if (in_atomic() || irqs_disabled()) {
			ret = sdw_trylock_mstr(mstr);
			if (!ret)
				/* SDW activity is ongoing. */
				return -EAGAIN;
		}
		sdw_lock_mstr(mstr);

		ret = __sdw_transfer(mstr, msg, num);
		sdw_unlock_mstr(mstr);
		return ret;
	}
	dev_dbg(&mstr->dev, "SDW level transfers not supported\n");
	return -EOPNOTSUPP;
}
EXPORT_SYMBOL_GPL(sdw_slave_transfer);


static int sdw_register_master(struct sdw_master *mstr)
{
	int ret = 0;
	int i;
	struct sdw_bus *sdw_bus;

	/* Can't register until after driver model init */
	if (unlikely(WARN_ON(!sdw_bus_type.p))) {
		ret = -EAGAIN;
		goto bus_init_not_done;
	}
	/* Sanity checks */
	if (unlikely(mstr->name[0] == '\0')) {
		pr_err("sdw-core: Attempt to register an master with no name!\n");
		ret = -EINVAL;
		goto mstr_no_name;
	}
	for (i = 0; i <= SOUNDWIRE_MAX_DEVICES; i++)
		mstr->sdw_addr[i].slv_number = i;

	rt_mutex_init(&mstr->bus_lock);
	INIT_LIST_HEAD(&mstr->slv_list);
	INIT_LIST_HEAD(&mstr->mstr_rt_list);

	sdw_bus = kzalloc(sizeof(struct sdw_bus), GFP_KERNEL);
	if (!sdw_bus)
		goto bus_alloc_failed;
	sdw_bus->mstr = mstr;

	mutex_lock(&sdw_core.core_lock);
	list_add_tail(&sdw_bus->bus_node, &sdw_core.bus_list);
	mutex_unlock(&sdw_core.core_lock);

	dev_set_name(&mstr->dev, "sdw-%d", mstr->nr);
	mstr->dev.bus = &sdw_bus_type;
	mstr->dev.type = &sdw_mstr_type;

	ret = device_register(&mstr->dev);
	if (ret)
		goto out_list;

	return 0;

out_list:
	mutex_lock(&sdw_core.core_lock);
	list_del(&sdw_bus->bus_node);
	mutex_unlock(&sdw_core.core_lock);
	kfree(sdw_bus);
bus_alloc_failed:
mstr_no_name:
bus_init_not_done:
	mutex_lock(&sdw_core.core_lock);
	idr_remove(&sdw_core.idr, mstr->nr);
	mutex_unlock(&sdw_core.core_lock);
	return ret;
}

/**
 * sdw_add_master_controller - declare sdw master, use dynamic bus number
 * @master: the master to add
 * Context: can sleep
 *
 * This routine is used to declare an sdw master when its bus number
 * doesn't matter or when its bus number is specified by an dt alias.
 * Examples of bases when the bus number doesn't matter: sdw masters
 * dynamically added by USB links or PCI plugin cards.
 *
 * When this returns zero, a new bus number was allocated and stored
 * in mstr->nr, and the specified master became available for slaves.
 * Otherwise, a negative errno value is returned.
 */
int sdw_add_master_controller(struct sdw_master *mstr)
{
	int id;

	mutex_lock(&sdw_core.core_lock);

	id = idr_alloc(&sdw_core.idr, mstr,
		       sdw_core.first_dynamic_bus_num, 0, GFP_KERNEL);
	mutex_unlock(&sdw_core.core_lock);
	if (id < 0)
		return id;

	mstr->nr = id;

	return sdw_register_master(mstr);
}
EXPORT_SYMBOL_GPL(sdw_add_master_controller);


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
