/*
 * DvC-Trace(dvct) Bus driver
 *
 * Copyright (C) 2015, Intel Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/module.h>
#include <linux/device.h>
#include <linux/dvctrace.h>

#ifdef DVCT_DEBUG
#define DVCT_IN() pr_debug("in\n")
#else
#define DVCT_IN() do {} while (0)
#endif

static ssize_t protocol_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	DVCT_IN();
	return sprintf(buf, "%d\n", dev_to_dvct_source_device(dev)->protocol);
}

static ssize_t protocol_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct dvct_source_device *ds_dev = dev_to_dvct_source_device(dev);

	DVCT_IN();
	if (ds_dev->instance_taken)
		return -EBUSY;

	if (!kstrtou8(buf, 10, &ds_dev->protocol))
		return size;

	return -EINVAL;
}

static DEVICE_ATTR_RW(protocol);

static ssize_t status_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct dvct_source_device *ds_dev = dev_to_dvct_source_device(dev);

	DVCT_IN();
	if (ds_dev->instance_taken) {
		if (ds_dev->function_taken)
			return sprintf(buf, "In use\n");
		else
			return sprintf(buf, "Reserved\n");
	} else {
		return sprintf(buf, "Free\n");
	}
}

static DEVICE_ATTR_RO(status);

static struct attribute *dvct_source_attrs[] = {
	&dev_attr_protocol.attr,
	&dev_attr_status.attr,
	NULL,
};

ATTRIBUTE_GROUPS(dvct_source);


static int dvct_match(struct device *dev, struct device_driver *drv)
{
	const char *devname = dev_name(dev);

	DVCT_IN();
	if (strlen(devname) <= strlen(drv->name))
		return -1;
	if (strncmp(devname, drv->name, strlen(drv->name)))
		return -1;
	return devname[strlen(drv->name)] == '-';
};

static struct bus_type dvctrace_bus_type = {
	.name = "dvctrace",
	.match = dvct_match,
	.dev_groups = dvct_source_groups,
};

static struct device dvctrace_bus = {
	.init_name = "dvctrace-bus",
};

static int dvct_match_free(struct device *dev, void *data)
{
	struct dvct_source_device *ds_dev = dev_to_dvct_source_device(dev);

	DVCT_IN();
	return !ds_dev->instance_taken;
}

struct dvct_source_device *dvct_source_find_by_name(const char *name)
{
	struct device *dev;

	DVCT_IN();
	dev = bus_find_device_by_name(&dvctrace_bus_type, NULL, name);
	if (IS_ERR_OR_NULL(dev))
		return ERR_PTR(-ENODEV);
	return dev_to_dvct_source_device(dev);
}
EXPORT_SYMBOL_GPL(dvct_source_find_by_name);

struct dvct_source_device *dvct_source_find_free_by_name(const char *name)
{
	struct dvct_source_device *ds_dev = dvct_source_find_by_name(name);

	DVCT_IN();
	if (IS_ERR_OR_NULL(ds_dev))
		return ERR_PTR(-ENODEV);

	if (ds_dev->instance_taken)
		return ERR_PTR(-EBUSY);

	return ds_dev;
}
EXPORT_SYMBOL_GPL(dvct_source_find_free_by_name);

struct dvct_source_device *dvct_source_find_free(void)
{
	struct device *dev = bus_find_device(&dvctrace_bus_type, NULL,
					     NULL, dvct_match_free);
	DVCT_IN();
	if (IS_ERR_OR_NULL(dev))
		return ERR_PTR(-ENODEV);

	return dev_to_dvct_source_device(dev);
}
EXPORT_SYMBOL_GPL(dvct_source_find_free);

static int fn_count_free(struct device *dev, void *data)
{
	int *count = data;
	struct dvct_source_device *ds_dev = dev_to_dvct_source_device(dev);

	DVCT_IN();
	if (!ds_dev->instance_taken)
		(*count)++;
	return 0;
}

int dvct_source_count_free(void)
{
	int count = 0;

	DVCT_IN();
	bus_for_each_dev(&dvctrace_bus_type, NULL, &count, fn_count_free);
	return count;
}
EXPORT_SYMBOL_GPL(dvct_source_count_free);

struct dvct_source_driver
*dvct_source_get_drv(struct dvct_source_device *ds_dev)
{
	BUG_ON(ds_dev->device.driver == NULL);
	return drv_to_dvct_source_driver(ds_dev->device.driver);
}
EXPORT_SYMBOL_GPL(dvct_source_get_drv);

int dvct_source_device_add(struct dvct_source_device *ds_dev,
			struct dvct_source_driver *ds_drv)
{
	int ret;

	DVCT_IN();
	if (!ds_dev)
		return -ENODEV;
	if (!ds_drv)
		return -EINVAL;

	spin_lock_init(&ds_dev->lock);
	spin_lock(&ds_dev->lock);
	ds_dev->instance_taken = 0;
	ds_dev->function_taken = 0;
	spin_unlock(&ds_dev->lock);

	device_initialize(&ds_dev->device);
	ds_dev->device.bus = &dvctrace_bus_type;

	if (!ds_dev->device.parent)
		ds_dev->device.parent = &dvctrace_bus;

	dev_set_name(&ds_dev->device, "%s-%s", ds_drv->driver.name,
		     ds_dev->name_add);

	ret = device_add(&ds_dev->device);
	if (ret) {
		dev_err(&dvctrace_bus, "Cannot add device %s %d\n",
			ds_dev->name_add, ret);
		return ret;
	}

	dev_notice(&dvctrace_bus, "Adding device %s\n", ds_dev->name_add);
	return 0;
};
EXPORT_SYMBOL_GPL(dvct_source_device_add);

void dvct_source_device_del(struct dvct_source_device *ds_dev)
{
	DVCT_IN();
	device_del(&ds_dev->device);
};
EXPORT_SYMBOL_GPL(dvct_source_device_del);

int __dvct_source_driver_register(struct dvct_source_driver *ds_drv,
			       struct module *owner)
{
	DVCT_IN();
	if (!ds_drv->activate ||
	    !ds_drv->binded ||
	    !ds_drv->start_transfer ||
	    !ds_drv->stop_transfer ||
	    !ds_drv->unbinded ||
	    !ds_drv->deactivate)
		return -EINVAL;

	ds_drv->driver.owner = owner;
	ds_drv->driver.bus = &dvctrace_bus_type;
	return driver_register(&ds_drv->driver);
}
EXPORT_SYMBOL_GPL(__dvct_source_driver_register);

void dvct_source_driver_unregister(struct dvct_source_driver *ds_drv)
{
	DVCT_IN();
	driver_unregister(&ds_drv->driver);
}
EXPORT_SYMBOL_GPL(dvct_source_driver_unregister);

static int __init dtb_init(void)
{
	int ret;

	DVCT_IN();
	ret = device_register(&dvctrace_bus);
	if (ret) {
		pr_err("Cannot register bus device %d\n", ret);
		return ret;
	}

	ret = bus_register(&dvctrace_bus_type);
	if (ret) {
		pr_err("Cannot register bus %d\n", ret);
		return ret;
	}
	return 0;
}

static void __exit dtb_exit(void)
{
	DVCT_IN();
	bus_unregister(&dvctrace_bus_type);
}

subsys_initcall(dtb_init);
module_exit(dtb_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DvC-Trace bus implementation");
MODULE_AUTHOR("Traian Schiau <traianx.schiau@intel.com>");
