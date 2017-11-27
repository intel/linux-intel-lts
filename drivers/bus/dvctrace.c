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
#include <linux/usb/debug.h>

#ifdef DVCT_DEBUG
#define DVCT_IN() pr_debug("in\n")
#else
#define DVCT_IN() do {} while (0)
#endif

/* Count the number of USB descriptors in the given ascii hex string
 * What we expect:
 *   ll tt ss xx xx xx
 *    |  |  |  +- Fill up the descriptor
 *    |  |  +- Descriptor sub-type (1-4)
 *    |  |       DC_INPUT_CONNECTION		0x01
 *    |  |       DC_OUTPUT_CONNECTION		0x02
 *    |  |       DC_DEBUG_UNIT			0x03
 *    |  |       DC_DEBUG_ATTRIBUTES		0x04
 *    |  +- Descriptor type (USB_DT_CS_INTERFACE)
 *    +- Descriptor length (check > 3 and we have the rest of it)
 */
static int count_descriptors(const char *buf, size_t size)
{
	size_t off = 0;
	int i, j, count = 0;
	u8 len, tmp;

	DVCT_IN();
	while (off < size) {
		/*the length*/
		j = sscanf(buf + off, "%2hhx%n", &len, &i);
		if (!j)
			break;
		if (j < 0 || len < 4)
			return -EINVAL;
		len--;
		off += i;

		/*Type*/
		j = sscanf(buf + off, "%2hhx%n", &tmp, &i);
		if (j <= 0 || tmp != USB_DT_CS_INTERFACE)
			return -EINVAL;
		len--;
		off += i;

		/*Sub Type*/
		j = sscanf(buf + off, "%2hhx%n", &tmp, &i);
		if (j <= 0 || tmp < DC_INPUT_CONNECTION
		    || tmp > DC_DEBUG_ATTRIBUTES)
			return -EINVAL;
		len--;
		off += i;

		while (len) {
			j = sscanf(buf + off, "%2hhx%n", &tmp, &i);
			if (j <= 0)
				return -EINVAL;
			len--;
			off += i;
		}
		count++;
	}
	return count;
}

/* Parse @buf and get a pointer to the descriptor identified
 *  @idx*/
static u8 *get_descriptor(const char *buf, size_t size, int idx)
{
	size_t off = 0;
	int i, j, k, count = 0;
	u8 len, tmp, *ret = NULL;

	DVCT_IN();
	while (off < size) {
		j = sscanf(buf + off, "%2hhx%n", &len, &i);
		if (j < 0)
			return ERR_PTR(-EINVAL);
		if (!j)
			return ERR_PTR(-ERANGE);

		if (count == idx) {
			ret = kmalloc(len, GFP_KERNEL);
			if (!ret)
				return ERR_PTR(-ENOMEM);
			ret[0] = len;
		}
		off += i;
		for (k = 1; k < len; k++) {
			j = sscanf(buf + off, "%2hhx%n", &tmp, &i);
			if (j <= 0) {
				kfree(ret);
				return ERR_PTR(-EINVAL);
			}
			if (count == idx)
				ret[k] = tmp;
			off += i;
		}
		if (count == idx)
			break;
		count++;
	}
	return ret;
}


static void free_strings(struct dvct_usb_descriptors *desc)
{
	struct usb_string *string;

	DVCT_IN();
	for (string = desc->str.strings; string && string->s; string++)
		kfree(string->s);

	kfree(desc->str.strings);
	desc->str.strings = NULL;
	kfree(desc->lk_tbl);
	desc->lk_tbl = NULL;
}

static void free_descriptors(struct dvct_usb_descriptors *desc)
{
	struct usb_descriptor_header **hdr;

	DVCT_IN();
	if (desc->dvc_spec) {
		for (hdr = desc->dvc_spec; *hdr; hdr++)
			kfree(*hdr);
		kfree(desc->dvc_spec);
		desc->dvc_spec = NULL;
	}
	free_strings(desc);
	kfree(desc);
}

static int alloc_strings(struct dvct_usb_descriptors *desc, int count)
{
	DVCT_IN();
	desc->lk_tbl = kzalloc((count + 1) * sizeof(struct dvct_string_lookup),
			       GFP_KERNEL);
	if (!desc->lk_tbl)
		goto  err;

	desc->str.strings = kzalloc((count + 1) * sizeof(*desc->str.strings),
				    GFP_KERNEL);
	if (!desc->str.strings)
		goto err_str;

	desc->str.language = 0x0409;

	return count;
err_str:
	kfree(desc->lk_tbl);
	desc->lk_tbl = NULL;
err:
	return -ENOMEM;
}

static struct dvct_usb_descriptors *alloc_descriptors(int count)
{
	struct dvct_usb_descriptors *desc;

	DVCT_IN();
	desc = kzalloc(sizeof(struct dvct_usb_descriptors), GFP_KERNEL);
	if (!desc)
		return ERR_PTR(-ENOMEM);

	desc->dvc_spec =
		kzalloc((count + 1) * sizeof(struct usb_descriptor_header *),
			GFP_KERNEL);

	if (!desc->dvc_spec) {
		kfree(desc);
		return ERR_PTR(-ENOMEM);
	}
	return desc;
}

static ssize_t descriptors_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct dvct_source_device *ds_dev = dev_to_dvct_source_device(dev);
	struct usb_descriptor_header **desc;
	int ret = 0;

	DVCT_IN();
	if (!ds_dev->desc || !ds_dev->desc->dvc_spec
	    || !*ds_dev->desc->dvc_spec)
		return sprintf(buf, "No Descriptors.\n");

	for (desc = ds_dev->desc->dvc_spec; *desc; desc++) {
		u8 len, *pdesc;
		int i;

		len = (*desc)->bLength;

		/* Check if it fits, total output is 3 * len */
		if ((ret + 3 * len) > PAGE_SIZE) {
			dev_warn(dev, "Descriptors attribute page overrun\n");
			break;
		}

		pdesc = (u8 *)(*desc);
		for (i = 0; i < len; i++)
			ret += snprintf(buf + ret, PAGE_SIZE - ret, "%02hhX ",
					pdesc[i]);
		buf[ret - 1] = '\n';
	}
	return ret;
}

static ssize_t descriptors_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct dvct_source_device *ds_dev = dev_to_dvct_source_device(dev);
	int desc_count, i;
	u8 *hdr;

	DVCT_IN();

	if (ds_dev->instance_taken)
		return -EBUSY;

	/*count the new descriptors, exit if invalid input*/
	desc_count = count_descriptors(buf, size);
	if (desc_count <= 0) {
		dev_warn(dev, "Invalid descriptor input:[%zu] %s", size, buf);
		return -EINVAL;
	}

	if (ds_dev->desc && ds_dev->desc != &ds_dev->static_desc)
		free_descriptors(ds_dev->desc);

	ds_dev->desc = alloc_descriptors(desc_count);
	if (IS_ERR_OR_NULL(ds_dev->desc)) {
		ds_dev->desc = NULL;
		return -ENOMEM;
	}

	for (i = 0; i < desc_count; i++) {
		hdr = get_descriptor(buf, size, i);
		if (IS_ERR_OR_NULL(hdr)) {
			dev_err(dev, "Cannot get descriptor %d, %ld\n", i,
				PTR_ERR(hdr));
			free_descriptors(ds_dev->desc);
			ds_dev->desc = NULL;
			return -EINVAL;
		}
		ds_dev->desc->dvc_spec[i] = (struct usb_descriptor_header *)hdr;
	}
	return size;
}

static DEVICE_ATTR_RW(descriptors);


/*find out at which member(offset) of which descriptor the pointer
 * points to */
static int dvctrace_string_ptr_to_offset(struct usb_descriptor_header **first,
					u8 *ptr, int *desc_offset, int *offset)
{
	u8 *hdr_start, *hdr_end;
	int idx = 0;

	DVCT_IN();
	for (; *first; first++, idx++) {
		hdr_start = (u8 *) (*first);
		hdr_end = hdr_start + ((*first)->bLength - 1);
		if (ptr >= hdr_start && ptr <= hdr_end) {
			*desc_offset = idx;
			*offset = ptr - hdr_start;
			return 0;
		}
	}
	return -EINVAL;
}

static u8 *dvctrace_offset_to_string_ptr(struct usb_descriptor_header **first,
					 int desc_offset, int offset)
{
	int idx = 0;

	DVCT_IN();
	for (; *first; first++, idx++) {
		if (idx == desc_offset) {
			if (offset >= (*first)->bLength)
				return ERR_PTR(-ERANGE);
			return ((u8 *) (*first)) + offset;
		}
	}
	return ERR_PTR(-ERANGE);
}

static int count_strings(const char *buf, size_t size)
{
	int count = 0;
	size_t off = 0, slen;
	int i = 0, j, desc_offset, offset;

	DVCT_IN();
	while (off < size) {
		j = sscanf(buf + off, "%d.%d: %n", &desc_offset, &offset, &i);
		if (j < 2)
			break;
		off += i;
		slen = 0;
		while (off + slen < size) {
			if (buf[off + slen] == ';' || buf[off + slen] == '\n')
				break;
			slen++;
		}
		off += slen;
		if (buf[off] == ';' || buf[off] == '\n')
			off++;
		count++;
	}
	return count;
}

static char *get_string(const char *buf, size_t size, int index,
			int *desc_offset, int *offset)
{
	int count = 0;
	size_t off = 0, slen;
	int i, j;
	char *ret = ERR_PTR(-EINVAL);

	DVCT_IN();
	while (off < size) {
		j = sscanf(buf + off, "%d.%d: %n", desc_offset, offset, &i);
		if (j < 2)
			return  ERR_PTR(-EINVAL);
		off += i;
		slen = 0;
		while (off + slen < size) {
			if (buf[off + slen] == ';' || buf[off + slen] == '\n')
				break;
			slen++;
		}

		if (count == index) {
			ret = kmalloc(slen+1, GFP_KERNEL);
			if (!ret)
				return ERR_PTR(-ENOMEM);
			memcpy(ret, buf + off, slen);
			ret[slen] = 0;
			return ret;
		}
		off += slen;
		if (buf[off] == ';' || buf[off] == '\n')
			off++;
		count++;
	}
	return ERR_PTR(-EINVAL);
}

static ssize_t strings_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct dvct_string_lookup *lk_s;
	struct dvct_source_device *ds_dev = dev_to_dvct_source_device(dev);
	int ret = 0;

	DVCT_IN();
	if (!ds_dev->desc || !ds_dev->desc->dvc_spec
	    || !*ds_dev->desc->dvc_spec)
		return sprintf(buf, "No Descriptors.\n");

	if (!ds_dev->desc->lk_tbl)
		return sprintf(buf, "No Strings.\n");

	for (lk_s = ds_dev->desc->lk_tbl; lk_s->str && lk_s->id; lk_s++) {
		int desc_offset, offset;

		/*
		 * Check if it fits, worst case is "Unknown(%p): %s\n"
		 * 8 + 16 + 3 + string length + 1
		 */
		if ((ret + 28 + strlen(lk_s->str->s)) > PAGE_SIZE) {
			dev_warn(dev, "Strings attribute page overrun\n");
			break;
		}

		if (dvctrace_string_ptr_to_offset(ds_dev->desc->dvc_spec,
						  lk_s->id, &desc_offset,
						  &offset))
			ret += snprintf(buf + ret, PAGE_SIZE - ret,
					"Unknown(%p): %s\n", lk_s->id,
					lk_s->str->s);
		else
			ret += snprintf(buf + ret, PAGE_SIZE - ret,
					"%d.%d: %s\n", desc_offset, offset,
					lk_s->str->s);
	}
	return ret;
}

static ssize_t strings_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t size)
{
	struct dvct_source_device *ds_dev = dev_to_dvct_source_device(dev);
	int count, i, ret;

	DVCT_IN();
	if (ds_dev->instance_taken)
		return -EBUSY;

	count = count_strings(buf, size);
	if (count <= 0) {
		dev_err(dev, "Invalid input string:(%zu) %s\n", size, buf);
		return -EINVAL;
	}

	if (ds_dev->desc == &ds_dev->static_desc) {
		dev_warn(dev, "Cannot set strings in static descriptors\n");
		return -EINVAL;
	}

	if (ds_dev->desc->str.strings)
		free_strings(ds_dev->desc);

	ret = alloc_strings(ds_dev->desc, count);
	if (ret < 0) {
		dev_err(dev, "Cannot allocate strings %d\n", ret);
		return -EINVAL;
	}

	for (i = 0; i < count; i++) {
		char *tmp;
		int d_off, off;
		u8 *pid;

		tmp = get_string(buf, size, i, &d_off, &off);
		if (IS_ERR_OR_NULL(tmp)) {
			free_strings(ds_dev->desc);
			return -EINVAL;
		}

		pid = dvctrace_offset_to_string_ptr(ds_dev->desc->dvc_spec,
						    d_off, off);
		if (IS_ERR_OR_NULL(pid)) {
			dev_warn(dev, "String out of bounds\n");
			free_strings(ds_dev->desc);
			return -EINVAL;
		}

		ds_dev->desc->lk_tbl[i].id = pid;
		ds_dev->desc->lk_tbl[i].str = &ds_dev->desc->str.strings[i];
		ds_dev->desc->str.strings[i].s = tmp;
	}
	return size;
}

static DEVICE_ATTR_RW(strings);

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
	&dev_attr_strings.attr,
	&dev_attr_descriptors.attr,
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

	if (ds_dev->static_desc.dvc_spec)
		ds_dev->desc = &ds_dev->static_desc;

	dev_notice(&dvctrace_bus, "Adding device %s\n", ds_dev->name_add);
	return 0;
};
EXPORT_SYMBOL_GPL(dvct_source_device_add);

void dvct_source_device_del(struct dvct_source_device *ds_dev)
{
	DVCT_IN();

	if (ds_dev->desc && ds_dev->desc != &ds_dev->static_desc) {
		free_descriptors(ds_dev->desc);
		ds_dev->desc = NULL;
	}

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
