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


static int sdw_assign_slv_number(struct sdw_master *mstr,
				struct sdw_msg *msg)
{
	int i, j, ret = -1;

	sdw_lock_mstr(mstr);
	for (i = 1; i <= SOUNDWIRE_MAX_DEVICES; i++) {
		if (mstr->sdw_addr[i].assigned == true)
			continue;
		mstr->sdw_addr[i].assigned = true;
		for (j = 0; j < 6; j++)
			mstr->sdw_addr[i].dev_id[j] = msg->buf[j];
		ret = i;
		break;
	}
	sdw_unlock_mstr(mstr);
	return ret;
}

static int sdw_program_slv_address(struct sdw_master *mstr,
				u8 slave_addr)
{
	struct sdw_msg msg;
	u8 buf[1] = {0};
	int ret;

	buf[0] = slave_addr;
	msg.ssp_tag = 0;
	msg.flag = SDW_MSG_FLAG_WRITE;
	msg.addr = SDW_SCP_DEVNUMBER;
	msg.len = 1;
	msg.buf = buf;
	msg.slave_addr = 0x0;
	msg.addr_page1 = 0x0;
	msg.addr_page2 = 0x0;

	ret = sdw_slave_transfer(mstr, &msg, 1);
	if (ret != 1) {
		dev_err(&mstr->dev, "Program Slave address change\n");
		return ret;
	}
	return 0;
}

static void sdw_free_slv_number(struct sdw_master *mstr,
		int slv_number)
{
	int i;

	sdw_lock_mstr(mstr);
	for (i = 0; i <= SOUNDWIRE_MAX_DEVICES; i++) {
		if (slv_number == mstr->sdw_addr[i].slv_number) {
			mstr->sdw_addr[slv_number].assigned = false;
			memset(&mstr->sdw_addr[slv_number].dev_id[0], 0x0, 6);
		}
	}
	sdw_unlock_mstr(mstr);
}

static int sdw_register_slave(struct sdw_master *mstr)
{
	int ret = 0, i, ports;
	struct sdw_msg msg;
	u8 buf[6] = {0};
	struct sdw_slave *sdw_slave;
	int slv_number = -1;


	msg.ssp_tag = 0;
	msg.flag = SDW_MSG_FLAG_READ;
	msg.addr = SDW_SCP_DEVID_0;
	msg.len = 6;
	msg.buf = buf;
	msg.slave_addr = 0x0;
	msg.addr_page1 = 0x0;
	msg.addr_page2 = 0x0;

	while ((ret = (sdw_slave_transfer(mstr, &msg, 1)) == 1)) {
		slv_number = sdw_assign_slv_number(mstr, &msg);
		if (slv_number <= 0) {
			dev_err(&mstr->dev, "Failed to assign slv_number\n");
			ret = -EINVAL;
			goto slv_number_assign_fail;
		}
		sdw_slave = kzalloc(sizeof(struct sdw_slave), GFP_KERNEL);
		if (!sdw_slave) {
			ret = -ENOMEM;
			goto mem_alloc_failed;
		}
		sdw_slave->mstr = mstr;
		sdw_slave->dev.parent = &sdw_slave->mstr->dev;
		sdw_slave->dev.bus = &sdw_bus_type;
		sdw_slave->dev.type = &sdw_slv_type;
		sdw_slave->slv_addr = &mstr->sdw_addr[slv_number];
		sdw_slave->slv_addr->slave = sdw_slave;
		/* We have assigned new slave number, so its not present
		 * till it again attaches to bus with this new
		 * slave address
		 */
		sdw_slave->slv_addr->status = SDW_SLAVE_STAT_NOT_PRESENT;
		for (i = 0; i < 6; i++)
			sdw_slave->dev_id[i] = msg.buf[i];
		dev_dbg(&mstr->dev, "SDW slave slave id found with values\n");
		dev_dbg(&mstr->dev, "dev_id0 to dev_id5: %x:%x:%x:%x:%x:%x\n",
			msg.buf[0], msg.buf[1], msg.buf[2],
			msg.buf[3], msg.buf[4], msg.buf[5]);
		dev_dbg(&mstr->dev, "Slave number assigned is %x\n", slv_number);
		/* TODO: Fill the sdw_slave structre from ACPI */
		ports = sdw_slave->sdw_slv_cap.num_of_sdw_ports;
		/* Add 1 for port 0 for simplicity */
		ports++;
		sdw_slave->port_ready =
			kzalloc((sizeof(struct completion) * ports),
							GFP_KERNEL);
		if (!sdw_slave->port_ready) {
			ret = -ENOMEM;
			goto port_alloc_mem_failed;
		}
		for (i = 0; i < ports; i++)
			init_completion(&sdw_slave->port_ready[i]);

		dev_set_name(&sdw_slave->dev, "sdw-slave%d-%02x:%02x:%02x:%02x:%02x:%02x",
			sdw_master_id(mstr),
			sdw_slave->dev_id[0],
			sdw_slave->dev_id[1],
			sdw_slave->dev_id[2],
			sdw_slave->dev_id[3],
			sdw_slave->dev_id[4],
			sdw_slave->dev_id[5]);
		/* Set name based on dev_id. This will be
		 * compared to load driver
		 */
		sprintf(sdw_slave->name, "%02x:%02x:%02x:%02x:%02x:%02x",
				sdw_slave->dev_id[0],
				sdw_slave->dev_id[1],
				sdw_slave->dev_id[2],
				sdw_slave->dev_id[3],
				sdw_slave->dev_id[4],
				sdw_slave->dev_id[5]);
		ret = device_register(&sdw_slave->dev);
		if (ret) {
			dev_err(&mstr->dev, "Register slave failed\n");
			goto reg_slv_failed;
		}
		ret = sdw_program_slv_address(mstr, slv_number);
		if (ret) {
			dev_err(&mstr->dev, "Programming slave address failed\n");
			goto program_slv_failed;
		}
		dev_dbg(&mstr->dev, "Slave registered with bus id %s\n",
			dev_name(&sdw_slave->dev));
		sdw_slave->slv_number = slv_number;
		mstr->num_slv++;
		sdw_lock_mstr(mstr);
		list_add_tail(&sdw_slave->node, &mstr->slv_list);
		sdw_unlock_mstr(mstr);

	}
	return 0;
program_slv_failed:
	device_unregister(&sdw_slave->dev);
port_alloc_mem_failed:
reg_slv_failed:
	kfree(sdw_slave);
mem_alloc_failed:
	sdw_free_slv_number(mstr, slv_number);
slv_number_assign_fail:
	return ret;

}

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


static void handle_slave_status(struct kthread_work *work)
{
	int ret = 0;
	struct sdw_slv_status *status, *__status__;
	struct sdw_bus *bus =
		container_of(work, struct sdw_bus, kwork);
	struct sdw_master *mstr = bus->mstr;
	unsigned long flags;

	/* Handle the new attached slaves to the bus. Register new slave
	 * to the bus.
	 */
	list_for_each_entry_safe(status, __status__, &bus->status_list, node) {
		if (status->status[0] == SDW_SLAVE_STAT_ATTACHED_OK) {
			ret += sdw_register_slave(mstr);
			if (ret)
				/* Even if adding new slave fails, we will
				 * continue.
				 */
				dev_err(&mstr->dev, "Registering new slave failed\n");
		}
		spin_lock_irqsave(&bus->spinlock, flags);
		list_del(&status->node);
		spin_unlock_irqrestore(&bus->spinlock, flags);
		kfree(status);
	}
}

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
	kthread_init_worker(&sdw_bus->kworker);
	sdw_bus->status_thread = kthread_run(kthread_worker_fn,
					&sdw_bus->kworker, "%s",
					dev_name(&mstr->dev));
	if (IS_ERR(sdw_bus->status_thread)) {
		dev_err(&mstr->dev, "error: failed to create status message task\n");
		ret = PTR_ERR(sdw_bus->status_thread);
		goto task_failed;
	}
	kthread_init_work(&sdw_bus->kwork, handle_slave_status);
	INIT_LIST_HEAD(&sdw_bus->status_list);
	spin_lock_init(&sdw_bus->spinlock);
	dev_dbg(&mstr->dev, "master [%s] registered\n", mstr->name);

	return 0;
task_failed:
	device_unregister(&mstr->dev);
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
 * sdw_master_update_slv_status: Report the status of slave to the bus driver.
 *			master calls this function based on the
 *			interrupt it gets once the slave changes its
 *			state.
 * @mstr: Master handle for which status is reported.
 * @status: Array of status of each slave.
 */
int sdw_master_update_slv_status(struct sdw_master *mstr,
					struct sdw_status *status)
{
	struct sdw_bus *bus = NULL;
	struct sdw_slv_status *slv_status;
	unsigned long flags;

	list_for_each_entry(bus, &sdw_core.bus_list, bus_node) {
		if (bus->mstr == mstr)
			break;
	}
	/* This is master is not registered with bus driver */
	if (!bus) {
		dev_info(&mstr->dev, "Master not registered with bus\n");
		return 0;
	}
	slv_status = kzalloc(sizeof(struct sdw_slv_status), GFP_ATOMIC);
	memcpy(slv_status->status, status, sizeof(struct sdw_status));

	spin_lock_irqsave(&bus->spinlock, flags);
	list_add_tail(&slv_status->node, &bus->status_list);
	spin_unlock_irqrestore(&bus->spinlock, flags);

	kthread_queue_work(&bus->kworker, &bus->kwork);
	return 0;
}
EXPORT_SYMBOL_GPL(sdw_master_update_slv_status);

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

static void sdw_unregister_slave(struct sdw_slave *sdw_slv)
{

	struct sdw_master *mstr;

	mstr = sdw_slv->mstr;
	sdw_lock_mstr(mstr);
	list_del(&sdw_slv->node);
	sdw_unlock_mstr(mstr);
	mstr->sdw_addr[sdw_slv->slv_number].assigned = false;
	memset(mstr->sdw_addr[sdw_slv->slv_number].dev_id, 0x0, 6);
	device_unregister(&sdw_slv->dev);
	kfree(sdw_slv);
}

static int __unregister_slave(struct device *dev, void *dummy)
{
	struct sdw_slave *slave = sdw_slave_verify(dev);

	if (slave && strcmp(slave->name, "dummy"))
		sdw_unregister_slave(slave);
	return 0;
}

/**
 * sdw_del_master_controller - unregister SDW master
 * @mstr: the master being unregistered
 * Context: can sleep
 *
 * This unregisters an SDW master which was previously registered
 * by @sdw_add_master_controller or @sdw_add_master_controller.
 */
void sdw_del_master_controller(struct sdw_master *mstr)
{
	struct sdw_master *found;

	/* First make sure that this master was ever added */
	mutex_lock(&sdw_core.core_lock);
	found = idr_find(&sdw_core.idr, mstr->nr);
	mutex_unlock(&sdw_core.core_lock);

	if (found != mstr) {
		pr_debug("sdw-core: attempting to delete unregistered master [%s]\n", mstr->name);
		return;
	}
	/* Detach any active slaves. This can't fail, thus we do not
	 * check the returned value.
	 */
	device_for_each_child(&mstr->dev, NULL, __unregister_slave);

	/* device name is gone after device_unregister */
	dev_dbg(&mstr->dev, "mstrter [%s] unregistered\n", mstr->name);

	/* wait until all references to the device are gone
	 *
	 * FIXME: This is old code and should ideally be replaced by an
	 * alternative which results in decoupling the lifetime of the struct
	 * device from the sdw_master, like spi or netdev do. Any solution
	 * should be thoroughly tested with DEBUG_KOBJECT_RELEASE enabled!
	 */
	init_completion(&mstr->slv_released);
	device_unregister(&mstr->dev);
	wait_for_completion(&mstr->slv_released);

	/* free bus id */
	mutex_lock(&sdw_core.core_lock);
	idr_remove(&sdw_core.idr, mstr->nr);
	mutex_unlock(&sdw_core.core_lock);

	/* Clear the device structure in case this mstrter is ever going to be
	   added again */
	memset(&mstr->dev, 0, sizeof(mstr->dev));
}
EXPORT_SYMBOL_GPL(sdw_del_master_controller);

/*
 * An sdw_driver is used with one or more sdw_slave (slave) nodes to access
 * sdw slave chips, on a bus instance associated with some sdw_master.
 */
int __sdw_mstr_driver_register(struct module *owner,
					struct sdw_mstr_driver *driver)
{
	int res;

	/* Can't register until after driver model init */
	if (unlikely(WARN_ON(!sdw_bus_type.p)))
		return -EAGAIN;

	/* add the driver to the list of sdw drivers in the driver core */
	driver->driver.owner = owner;
	driver->driver.bus = &sdw_bus_type;

	/* When registration returns, the driver core
	 * will have called probe() for all matching-but-unbound slaves.
	 */
	res = driver_register(&driver->driver);
	if (res)
		return res;

	pr_debug("sdw-core: driver [%s] registered\n", driver->driver.name);

	return 0;
}
EXPORT_SYMBOL_GPL(__sdw_mstr_driver_register);

void sdw_mstr_driver_unregister(struct sdw_mstr_driver *driver)
{
	driver_unregister(&driver->driver);
}
EXPORT_SYMBOL_GPL(sdw_mstr_driver_unregister);

void sdw_slave_driver_unregister(struct sdw_slave_driver *driver)
{
	driver_unregister(&driver->driver);
}
EXPORT_SYMBOL_GPL(sdw_slave_driver_unregister);

/*
 * An sdw_driver is used with one or more sdw_slave (slave) nodes to access
 * sdw slave chips, on a bus instance associated with some sdw_master.
 */
int __sdw_slave_driver_register(struct module *owner,
					struct sdw_slave_driver *driver)
{
	int res;
	/* Can't register until after driver model init */
	if (unlikely(WARN_ON(!sdw_bus_type.p)))
		return -EAGAIN;

	/* add the driver to the list of sdw drivers in the driver core */
	driver->driver.owner = owner;
	driver->driver.bus = &sdw_bus_type;

	/* When registration returns, the driver core
	 * will have called probe() for all matching-but-unbound slaves.
	 */
	res = driver_register(&driver->driver);
	if (res)
		return res;
	pr_debug("sdw-core: driver [%s] registered\n", driver->driver.name);

	return 0;
}
EXPORT_SYMBOL_GPL(__sdw_slave_driver_register);

int sdw_register_slave_capabilities(struct sdw_slave *sdw,
					struct sdw_slv_capabilities *cap)
{
	struct sdw_slv_capabilities *slv_cap;
	struct sdw_slv_dpn_capabilities *slv_dpn_cap, *dpn_cap;
	struct port_audio_mode_properties *prop, *slv_prop;
	int i, j;

	slv_cap = &sdw->sdw_slv_cap;

	slv_cap->wake_up_unavailable = cap->wake_up_unavailable;
	slv_cap->wake_up_unavailable = cap->wake_up_unavailable;
	slv_cap->test_mode_supported = cap->test_mode_supported;
	slv_cap->clock_stop1_mode_supported = cap->clock_stop1_mode_supported;
	slv_cap->simplified_clock_stop_prepare =
				cap->simplified_clock_stop_prepare;
	slv_cap->highphy_capable = cap->highphy_capable;
	slv_cap->paging_supported  = cap->paging_supported;
	slv_cap->bank_delay_support = cap->bank_delay_support;
	slv_cap->port_15_read_behavior = cap->port_15_read_behavior;
	slv_cap->sdw_dp0_supported  = cap->sdw_dp0_supported;
	slv_cap->num_of_sdw_ports = cap->num_of_sdw_ports;
	slv_cap->sdw_dpn_cap = devm_kzalloc(&sdw->dev,
			((sizeof(struct sdw_slv_dpn_capabilities)) *
			cap->num_of_sdw_ports), GFP_KERNEL);
	for (i = 0; i < cap->num_of_sdw_ports; i++) {
		dpn_cap = &cap->sdw_dpn_cap[i];
		slv_dpn_cap = &slv_cap->sdw_dpn_cap[i];
		slv_dpn_cap->port_direction = dpn_cap->port_direction;
		slv_dpn_cap->port_number = dpn_cap->port_number;
		slv_dpn_cap->max_word_length = dpn_cap->max_word_length;
		slv_dpn_cap->min_word_length = dpn_cap->min_word_length;
		slv_dpn_cap->num_word_length = dpn_cap->num_word_length;
		if (NULL == dpn_cap->word_length_buffer)
			slv_dpn_cap->word_length_buffer =
						dpn_cap->word_length_buffer;
		else {
			slv_dpn_cap->word_length_buffer =
				devm_kzalloc(&sdw->dev,
				dpn_cap->num_word_length *
				(sizeof(unsigned  int)), GFP_KERNEL);
			if (!slv_dpn_cap->word_length_buffer)
				return -ENOMEM;
			memcpy(slv_dpn_cap->word_length_buffer,
				dpn_cap->word_length_buffer,
				dpn_cap->num_word_length *
				(sizeof(unsigned  int)));
		}
		slv_dpn_cap->dpn_type = dpn_cap->dpn_type;
		slv_dpn_cap->dpn_grouping = dpn_cap->dpn_grouping;
		slv_dpn_cap->prepare_ch = dpn_cap->prepare_ch;
		slv_dpn_cap->imp_def_intr_mask = dpn_cap->imp_def_intr_mask;
		slv_dpn_cap->min_ch_num = dpn_cap->min_ch_num;
		slv_dpn_cap->max_ch_num = dpn_cap->max_ch_num;
		slv_dpn_cap->num_ch_supported = dpn_cap->num_ch_supported;
		if (NULL == slv_dpn_cap->ch_supported)
			slv_dpn_cap->ch_supported  = dpn_cap->ch_supported;
		else {
			slv_dpn_cap->ch_supported =
				devm_kzalloc(&sdw->dev,
				dpn_cap->num_ch_supported *
				(sizeof(unsigned  int)), GFP_KERNEL);
			if (!slv_dpn_cap->ch_supported)
				return -ENOMEM;
			memcpy(slv_dpn_cap->ch_supported,
				dpn_cap->ch_supported,
				dpn_cap->num_ch_supported *
				(sizeof(unsigned  int)));
		}
		slv_dpn_cap->port_flow_mode_mask  =
					dpn_cap->port_flow_mode_mask;
		slv_dpn_cap->block_packing_mode_mask =
				dpn_cap->block_packing_mode_mask;
		slv_dpn_cap->port_encoding_type_mask =
				dpn_cap->port_encoding_type_mask;
		slv_dpn_cap->num_audio_modes = dpn_cap->num_audio_modes;

		slv_dpn_cap->mode_properties = devm_kzalloc(&sdw->dev,
				((sizeof(struct port_audio_mode_properties)) *
				dpn_cap->num_audio_modes), GFP_KERNEL);
		for (j = 0; j < dpn_cap->num_audio_modes; j++) {
			prop = &dpn_cap->mode_properties[j];
			slv_prop = &slv_dpn_cap->mode_properties[j];
			slv_prop->max_frequency = prop->max_frequency;
			slv_prop->min_frequency = prop->min_frequency;
			slv_prop->num_freq_configs = prop->num_freq_configs;
			if (NULL == slv_prop->freq_supported)
				slv_prop->freq_supported =
						prop->freq_supported;
			else {
				slv_prop->freq_supported =
					devm_kzalloc(&sdw->dev,
					prop->num_freq_configs *
					(sizeof(unsigned  int)), GFP_KERNEL);
					if (!slv_prop->freq_supported)
						return -ENOMEM;
					memcpy(slv_prop->freq_supported,
						prop->freq_supported,
					prop->num_freq_configs *
					(sizeof(unsigned  int)));
			}
			slv_prop->glitchless_transitions_mask
					= prop->glitchless_transitions_mask;
			slv_prop->max_sampling_frequency =
						prop->max_sampling_frequency;
			slv_prop->min_sampling_frequency  =
						prop->min_sampling_frequency;
			slv_prop->num_sampling_freq_configs =
					prop->num_sampling_freq_configs;
			if (NULL == prop->sampling_freq_config)
				slv_prop->sampling_freq_config =
						prop->sampling_freq_config;
			else {
				slv_prop->sampling_freq_config =
					devm_kzalloc(&sdw->dev,
					prop->num_sampling_freq_configs *
					(sizeof(unsigned  int)), GFP_KERNEL);
					if (!slv_prop->sampling_freq_config)
						return -ENOMEM;
					memcpy(slv_prop->sampling_freq_config,
						prop->sampling_freq_config,
					prop->num_sampling_freq_configs *
					(sizeof(unsigned  int)));
			}

			slv_prop->ch_prepare_behavior =
						prop->ch_prepare_behavior;
		}
	}
	sdw->slave_cap_updated = true;
	return 0;
}
EXPORT_SYMBOL_GPL(sdw_register_slave_capabilities);

static int sdw_get_stream_tag(char *key, int *stream_tag)
{
	int i;
	int ret = -EINVAL;
	struct sdw_runtime *sdw_rt;
	struct sdw_stream_tag *stream_tags = sdw_core.stream_tags;

	/* If stream tag is already allocated return that after incrementing
	 * reference count. This is only possible if key is provided.
	 */
	mutex_lock(&sdw_core.core_lock);
	if (!key)
		goto key_check_not_required;
	for (i = 0; i < SDW_NUM_STREAM_TAGS; i++) {
		if (!(strcmp(stream_tags[i].key, key))) {
			stream_tags[i].ref_count++;
			*stream_tag = stream_tags[i].stream_tag;
			mutex_unlock(&sdw_core.core_lock);
			return 0;
		}
	}
key_check_not_required:
	for (i = 0; i < SDW_NUM_STREAM_TAGS; i++) {
		if (!stream_tags[i].ref_count) {
			stream_tags[i].ref_count++;
			*stream_tag = stream_tags[i].stream_tag;
			mutex_init(&stream_tags[i].stream_lock);
			sdw_rt = kzalloc(sizeof(struct sdw_runtime),
					GFP_KERNEL);
			INIT_LIST_HEAD(&sdw_rt->slv_rt_list);
			INIT_LIST_HEAD(&sdw_rt->mstr_rt_list);
			sdw_rt->stream_state = SDW_STATE_INIT_STREAM_TAG;
			stream_tags[i].sdw_rt = sdw_rt;
			if (!stream_tags[i].sdw_rt) {
				stream_tags[i].ref_count--;
				ret = -ENOMEM;
				goto out;
			}
			if (key)
				strlcpy(stream_tags[i].key, key,
					SDW_MAX_STREAM_TAG_KEY_SIZE);
			mutex_unlock(&sdw_core.core_lock);
			return 0;
		}
	}
	mutex_unlock(&sdw_core.core_lock);
out:
	return ret;
}

void sdw_release_stream_tag(int stream_tag)
{
	int i;
	struct sdw_stream_tag *stream_tags = sdw_core.stream_tags;

	mutex_lock(&sdw_core.core_lock);
	for (i = 0; i < SDW_NUM_STREAM_TAGS; i++) {
		if (stream_tag == stream_tags[i].stream_tag) {
			stream_tags[i].ref_count--;
			if (stream_tags[i].ref_count == 0) {
				kfree(stream_tags[i].sdw_rt);
				memset(stream_tags[i].key, 0x0,
					SDW_MAX_STREAM_TAG_KEY_SIZE);
			}
		}
	}
	mutex_unlock(&sdw_core.core_lock);
}
EXPORT_SYMBOL_GPL(sdw_release_stream_tag);

/**
 * sdw_alloc_stream_tag: Assign the stream tag for the unique streams
 *			between master and slave device.
 *			Normally master master will request for the
 *			stream tag for the stream between master
 *			and slave device. It programs the same stream
 *			tag to the slave device. Stream tag is unique
 *			for all the streams between masters and slave
 *			across SoCs.
 * @guid: Group of the device port. All the ports of the device with
 *			part of same stream will have same guid.
 *
 * @stream:tag: Stream tag returned by bus driver.
 */
int sdw_alloc_stream_tag(char *guid, int *stream_tag)
{
	int ret = 0;

	ret = sdw_get_stream_tag(guid, stream_tag);
	if (ret) {
		pr_err("Stream tag assignment failed\n");
		goto out;
	}

out:
	return ret;
}
EXPORT_SYMBOL_GPL(sdw_alloc_stream_tag);

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
