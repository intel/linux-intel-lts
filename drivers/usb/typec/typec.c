/*
 * USB Type-C Connector Class
 *
 * Copyright (C) 2016, Intel Corporation
 * Author: Heikki Krogerus <heikki.krogerus@linux.intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/usb/typec.h>

struct typec_port {
	unsigned int		id;
	struct device		dev;
	struct mutex		lock; /* device lock */

	int			prefer_role;

	enum typec_data_role	data_role;
	enum typec_role		pwr_role;
	enum typec_role		vconn_role;
	enum typec_pwr_opmode	pwr_opmode;

	struct typec_partner	*partner;
	struct typec_cable	*cable;

	unsigned int		connected:1;

	const struct typec_capability *cap;
};

#define to_typec_port(p) container_of(p, struct typec_port, dev)

static DEFINE_IDA(typec_index_ida);

static struct class typec_class = {
	.name = "typec",
};

static const char * const typec_accessory_modes[] = {
	[TYPEC_ACCESSORY_NONE]		= "none",
	[TYPEC_ACCESSORY_AUDIO]		= "Audio",
	[TYPEC_ACCESSORY_DEBUG]		= "Debug",
	[TYPEC_ACCESSORY_DAUDIO]	= "Digital Audio",
};

/* ------------------------------------------------------------------------- */
/* Type-C Partners */

static void typec_dev_release(struct device *dev)
{
}

static const char * const typec_partner_types[] = {
	[TYPEC_PARTNER_USB]		= "USB",
	[TYPEC_PARTNER_CHARGER]		= "Charger",
	[TYPEC_PARTNER_ALTMODE]		= "Alternate Mode",
	[TYPEC_PARTNER_ACCESSORY]	= "Accessory",
};

static ssize_t partner_type_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct typec_partner *partner = container_of(dev, struct typec_partner,
						     dev);

	return sprintf(buf, "%s\n", typec_partner_types[partner->type]);
}

static struct device_attribute dev_attr_partner_type = {
	.attr = {
		.name = "type",
		.mode = S_IRUGO,
	},
	.show = partner_type_show,
};

static ssize_t
partner_accessory_mode_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct typec_partner *partner = container_of(dev, struct typec_partner,
						     dev);

	return sprintf(buf, "%s\n", typec_accessory_modes[partner->accessory]);
}

static struct device_attribute dev_attr_partner_accessory = {
	.attr = {
		.name = "accessory",
		.mode = S_IRUGO,
	},
	.show = partner_accessory_mode_show,
};

static ssize_t partner_usb_pd_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct typec_partner *partner = container_of(dev, struct typec_partner,
						     dev);

	return sprintf(buf, "%d\n", partner->usb_pd);
}

static struct device_attribute dev_attr_partner_usb_pd = {
	.attr = {
		.name = "supports_usb_power_delivery",
		.mode = S_IRUGO,
	},
	.show = partner_usb_pd_show,
};

static ssize_t partner_vdo_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct typec_partner *partner = container_of(dev, struct typec_partner,
						     dev);

	return sprintf(buf, "0x%08x\n", partner->vdo);
}

static struct device_attribute dev_attr_partner_vdo = {
	.attr = {
		.name = "id_header_vdo",
		.mode = S_IRUGO,
	},
	.show = partner_vdo_show,
};

static struct attribute *typec_partner_attrs[] = {
	&dev_attr_partner_accessory.attr,
	&dev_attr_partner_type.attr,
	&dev_attr_partner_usb_pd.attr,
	&dev_attr_partner_vdo.attr,
	NULL
};

static umode_t partner_is_visible(struct kobject *kobj, struct attribute *attr,
				  int n)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct typec_partner *partner = container_of(dev, struct typec_partner,
						     dev);

	if (attr == &dev_attr_partner_accessory.attr &&
	    partner->type != TYPEC_PARTNER_ACCESSORY)
		return 0;

	if (attr == &dev_attr_partner_vdo.attr && !partner->usb_pd)
		return 0;

	return attr->mode;
}

static struct attribute_group typec_partner_group = {
	.attrs = typec_partner_attrs,
	.is_visible = partner_is_visible,
};

static const struct attribute_group *typec_partner_groups[] = {
	&typec_partner_group,
	NULL
};

static struct device_type typec_partner_dev_type = {
	.name = "typec_partner_device",
	.groups = typec_partner_groups,
	.release = typec_dev_release,
};

static int
typec_add_partner(struct typec_port *port, struct typec_partner *partner)
{
	struct device *dev = &partner->dev;
	int ret;

	dev->class = &typec_class;
	dev->parent = &port->dev;
	dev->type = &typec_partner_dev_type;
	dev_set_name(dev, "%s-partner", dev_name(&port->dev));

	ret = device_register(dev);
	if (ret) {
		put_device(dev);
		return ret;
	}

	port->partner = partner;
	return 0;
}

static void typec_remove_partner(struct typec_port *port)
{
	WARN_ON(port->partner->alt_modes);
	device_unregister(&port->partner->dev);
}

/* ------------------------------------------------------------------------- */
/* Type-C Cable Plugs */

static struct device_type typec_plug_dev_type = {
	.name = "type_plug_device",
	.release = typec_dev_release,
};

static int
typec_add_plug(struct typec_port *port, struct typec_plug *plug)
{
	struct device *dev = &plug->dev;
	char name[8];
	int ret;

	sprintf(name, "plug%d", plug->index);

	dev->class = &typec_class;
	dev->parent = &port->cable->dev;
	dev->type = &typec_plug_dev_type;
	dev_set_name(dev, "%s-%s", dev_name(&port->dev), name);

	ret = device_register(dev);
	if (ret) {
		put_device(dev);
		return ret;
	}

	return 0;
}

static void typec_remove_plug(struct typec_plug *plug)
{
	WARN_ON(plug->alt_modes);
	device_unregister(&plug->dev);
}

/* Type-C Cables */

static ssize_t
cable_active_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct typec_cable *cable = container_of(dev, struct typec_cable, dev);

	return sprintf(buf, "%d\n", cable->active);
}

static struct device_attribute dev_attr_cable_active = {
	.attr = {
		.name = "active",
		.mode = S_IRUGO,
	},
	.show = cable_active_show,
};

static ssize_t cable_usb_pd_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct typec_cable *cable = container_of(dev, struct typec_cable, dev);

	return sprintf(buf, "%d\n", cable->usb_pd);
}

static struct device_attribute dev_attr_cable_usb_pd = {
	.attr = {
		.name = "supports_usb_power_delivery",
		.mode = S_IRUGO,
	},
	.show = cable_usb_pd_show,
};

static ssize_t cable_vdo_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct typec_cable *cable = container_of(dev, struct typec_cable, dev);

	return sprintf(buf, "0x%08x\n", cable->vdo);
}

static struct device_attribute dev_attr_cable_vdo = {
	.attr = {
		.name = "id_header_vdo",
		.mode = S_IRUGO,
	},
	.show = cable_vdo_show,
};

static const char * const typec_plug_types[] = {
	[USB_PLUG_NONE]		= "unknown",
	[USB_PLUG_TYPE_A]	= "Type-A",
	[USB_PLUG_TYPE_B]	= "Type-B",
	[USB_PLUG_TYPE_C]	= "Type-C",
	[USB_PLUG_CAPTIVE]	= "Captive",
};

static ssize_t
cable_plug_type_show(struct device *dev, struct device_attribute *attr,
		     char *buf)
{
	struct typec_cable *cable = container_of(dev, struct typec_cable, dev);

	return sprintf(buf, "%s\n", typec_plug_types[cable->type]);
}

static struct device_attribute dev_attr_plug_type = {
	.attr = {
		.name = "plug_type",
		.mode = S_IRUGO,
	},
	.show = cable_plug_type_show,
};

static struct attribute *typec_cable_attrs[] = {
	&dev_attr_cable_active.attr,
	&dev_attr_cable_usb_pd.attr,
	&dev_attr_cable_vdo.attr,
	&dev_attr_plug_type.attr,
	NULL
};

static umode_t cable_is_visible(struct kobject *kobj, struct attribute *attr,
				int n)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct typec_cable *cable = container_of(dev, struct typec_cable, dev);

	if (attr == &dev_attr_cable_vdo.attr && !cable->usb_pd)
		return 0;

	return attr->mode;
}

static struct attribute_group typec_cable_group = {
	.attrs = typec_cable_attrs,
	.is_visible = cable_is_visible,
};

static const struct attribute_group *typec_cable_groups[] = {
	&typec_cable_group,
	NULL
};

static struct device_type typec_cable_dev_type = {
	.name = "typec_cable_device",
	.groups = typec_cable_groups,
	.release = typec_dev_release,
};

static int typec_add_cable(struct typec_port *port, struct typec_cable *cable)
{
	struct device *dev = &cable->dev;
	int ret;

	dev->class = &typec_class;
	dev->parent = &port->dev;
	dev->type = &typec_cable_dev_type;
	dev_set_name(dev, "%s-cable", dev_name(&port->dev));

	ret = device_register(dev);
	if (ret) {
		put_device(dev);
		return ret;
	}

	/* Plug1 */
	if (!cable->usb_pd)
		return 0;

	cable->plug[0].index = 1;
	ret = typec_add_plug(port, &cable->plug[0]);
	if (ret) {
		device_unregister(dev);
		return ret;
	}

	/* Plug2 */
	if (!cable->active || !cable->sop_pp_controller)
		return 0;

	cable->plug[1].index = 2;
	ret = typec_add_plug(port, &cable->plug[1]);
	if (ret) {
		typec_remove_plug(&cable->plug[0]);
		device_unregister(dev);
		return ret;
	}

	port->cable = cable;
	return 0;
}

static void typec_remove_cable(struct typec_port *port)
{
	if (port->cable->active) {
		typec_remove_plug(&port->cable->plug[0]);
		if (port->cable->sop_pp_controller)
			typec_remove_plug(&port->cable->plug[1]);
	}
	device_unregister(&port->cable->dev);
}

/* ------------------------------------------------------------------------- */
/* API for the port drivers */

static void typec_init_roles(struct typec_port *port)
{
	if (port->prefer_role < 0)
		return;

	if (port->prefer_role == TYPEC_SOURCE) {
		port->data_role = TYPEC_HOST;
		port->pwr_role = TYPEC_SOURCE;
		port->vconn_role = TYPEC_SOURCE;
	} else {
		/* Device mode as default also by default with DRP ports */
		port->data_role = TYPEC_DEVICE;
		port->pwr_role = TYPEC_SINK;
		port->vconn_role = TYPEC_SINK;
	}
}

int typec_connect(struct typec_port *port, struct typec_connection *con)
{
	int ret;

	if (!con->partner && !con->cable)
		return -EINVAL;

	port->connected = 1;
	port->data_role = con->data_role;
	port->pwr_role = con->pwr_role;
	port->vconn_role = con->vconn_role;
	port->pwr_opmode = con->pwr_opmode;

	kobject_uevent(&port->dev.kobj, KOBJ_CHANGE);

	if (con->cable) {
		ret = typec_add_cable(port, con->cable);
		if (ret)
			return ret;
	}

	if (con->partner) {
		ret = typec_add_partner(port, con->partner);
		if (ret) {
			if (con->cable)
				typec_remove_cable(port);
			return ret;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(typec_connect);

void typec_disconnect(struct typec_port *port)
{
	if (port->partner)
		typec_remove_partner(port);

	if (port->cable)
		typec_remove_cable(port);

	port->connected = 0;
	port->partner = NULL;
	port->cable = NULL;

	port->pwr_opmode = TYPEC_PWR_MODE_USB;

	typec_init_roles(port);

	kobject_uevent(&port->dev.kobj, KOBJ_CHANGE);
}
EXPORT_SYMBOL_GPL(typec_disconnect);

/* --------------------------------------- */
/* Driver callbacks to report role updates */

void typec_set_data_role(struct typec_port *port, enum typec_data_role role)
{
	mutex_lock(&port->lock);
	port->data_role = role;
	sysfs_notify(&port->dev.kobj, NULL, "current_data_role");
	mutex_unlock(&port->lock);
}
EXPORT_SYMBOL(typec_set_data_role);

void typec_set_pwr_role(struct typec_port *port, enum typec_role role)
{
	mutex_lock(&port->lock);
	port->pwr_role = role;
	sysfs_notify(&port->dev.kobj, NULL, "current_power_role");
	mutex_unlock(&port->lock);
}
EXPORT_SYMBOL(typec_set_pwr_role);

void typec_set_vconn_role(struct typec_port *port, enum typec_role role)
{
	mutex_lock(&port->lock);
	port->vconn_role = role;
	sysfs_notify(&port->dev.kobj, NULL, "current_vconn_role");
	mutex_unlock(&port->lock);
}
EXPORT_SYMBOL(typec_set_vconn_role);

void typec_set_pwr_opmode(struct typec_port *port,
			  enum typec_pwr_opmode opmode)
{
	mutex_lock(&port->lock);
	port->pwr_opmode = opmode;
	sysfs_notify(&port->dev.kobj, NULL, "power_operation_mode");
	mutex_unlock(&port->lock);
}
EXPORT_SYMBOL(typec_set_pwr_opmode);

/* -------------------------------- */
/* Alternate Modes */

/*
 * typec_altmode_update_active - Notify about Enter/Exit mode
 * @alt: Handle to the Alternate Mode
 * @mode: Mode id
 * @active: True when the mode has been enterred
 */
void typec_altmode_update_active(struct typec_altmode *alt, int mode,
				 bool active)
{
	struct typec_port *port = typec_altmode2port(alt);
	struct typec_mode *m = alt->modes + mode;
	char dir[6];

	mutex_lock(&port->lock);
	m->active = active;
	sprintf(dir, "mode%d", mode);
	sysfs_notify(&alt->dev.kobj, dir, "active");
	mutex_unlock(&port->lock);
}
EXPORT_SYMBOL(typec_altmode_update_active);

static struct device_type typec_port_dev_type;

/*
 * typec_altmode2port - Alternate Mode to USB Type-C port
 * @alt: The Alternate Mode
 *
 * Returns the port that the cable plug or partner with @alt is connected to.
 */
struct typec_port *typec_altmode2port(struct typec_altmode *alt)
{
	if (alt->dev.parent->type == &typec_plug_dev_type)
		return to_typec_port(alt->dev.parent->parent->parent);
	if (alt->dev.parent->type == &typec_partner_dev_type)
		return to_typec_port(alt->dev.parent->parent);
	if (alt->dev.parent->type == &typec_port_dev_type)
		return to_typec_port(alt->dev.parent);

	return NULL;
}
EXPORT_SYMBOL_GPL(typec_altmode2port);

static void typec_altmode_release(struct device *dev)
{
	struct typec_altmode *alt = to_altmode(dev);

	kfree(alt->mode_groups);
}

static ssize_t
typec_altmode_vdo_show(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	struct typec_mode *mode = container_of(attr, struct typec_mode,
					       vdo_attr);
	struct typec_port *port = typec_altmode2port(mode->alt_mode);
	ssize_t ret;

	mutex_lock(&port->lock);
	ret = sprintf(buf, "0x%08x\n", mode->vdo);
	mutex_unlock(&port->lock);

	return ret;
}

static ssize_t
typec_altmode_desc_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct typec_mode *mode = container_of(attr, struct typec_mode,
					       desc_attr);
	struct typec_port *port = typec_altmode2port(mode->alt_mode);
	ssize_t ret;

	mutex_lock(&port->lock);
	ret = sprintf(buf, "%s\n", mode->desc ? mode->desc : "");
	mutex_unlock(&port->lock);

	return ret;
}

static ssize_t
typec_altmode_active_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct typec_mode *mode = container_of(attr, struct typec_mode,
					       active_attr);
	struct typec_port *port = typec_altmode2port(mode->alt_mode);
	ssize_t ret;

	mutex_lock(&port->lock);
	ret = sprintf(buf, "%d\n", mode->active);
	mutex_unlock(&port->lock);

	return ret;
}

static ssize_t
typec_altmode_active_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t size)
{
	struct typec_mode *mode = container_of(attr, struct typec_mode,
					       active_attr);
	struct typec_port *port = typec_altmode2port(mode->alt_mode);
	bool activate;
	int ret;

	ret = strtobool(buf, &activate);
	if (ret)
		return ret;

	if (activate > 1)
		return -EINVAL;

	mutex_lock(&port->lock);
	ret = port->cap->activate_mode(mode->alt_mode, mode->index, activate);
	if (!ret) {
		mode->active = activate;
		ret = size;
	}
	mutex_unlock(&port->lock);

	return ret;
}

static ssize_t
typec_altmode_roles_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct typec_mode *mode = container_of(attr, struct typec_mode,
					       roles_attr);
	struct typec_port *port = typec_altmode2port(mode->alt_mode);
	ssize_t ret;

	mutex_lock(&port->lock);
	switch (mode->roles) {
	case TYPEC_PORT_DFP:
		ret =  sprintf(buf, "source\n");
		break;
	case TYPEC_PORT_UFP:
		ret = sprintf(buf, "sink\n");
		break;
	case TYPEC_PORT_DRP:
	default:
		ret = sprintf(buf, "source, sink\n");
		break;
	}
	mutex_unlock(&port->lock);

	return ret;
}

static void typec_init_modes(struct typec_altmode *alt, int is_port)
{
	struct typec_mode *mode = alt->modes;
	int i;

	for (i = 0; i < alt->n_modes; i++, mode++) {
		mode->alt_mode = alt;
		mode->index = i;
		sprintf(mode->group_name, "mode%d", i);

		sysfs_attr_init(&mode->vdo_attr.attr);
		mode->vdo_attr.attr.name = "vdo";
		mode->vdo_attr.attr.mode = S_IRUGO;
		mode->vdo_attr.show = typec_altmode_vdo_show;

		sysfs_attr_init(&mode->desc_attr.attr);
		mode->desc_attr.attr.name = "description";
		mode->desc_attr.attr.mode = S_IRUGO;
		mode->desc_attr.show = typec_altmode_desc_show;

		sysfs_attr_init(&mode->active_attr.attr);
		mode->active_attr.attr.name = "active";
		mode->active_attr.attr.mode = S_IWUSR | S_IRUGO;
		mode->active_attr.show = typec_altmode_active_show;
		mode->active_attr.store = typec_altmode_active_store;

		mode->attrs[0] = &mode->vdo_attr.attr;
		mode->attrs[1] = &mode->desc_attr.attr;
		mode->attrs[2] = &mode->active_attr.attr;

		/* With ports, list the roles that the mode is supported with */
		if (is_port) {
			sysfs_attr_init(&mode->roles_attr.attr);
			mode->roles_attr.attr.name = "supported_roles";
			mode->roles_attr.attr.mode = S_IRUGO;
			mode->roles_attr.show = typec_altmode_roles_show;

			mode->attrs[3] = &mode->roles_attr.attr;
		}

		mode->group.attrs = mode->attrs;
		mode->group.name = mode->group_name;

		alt->mode_groups[i] = &mode->group;
	}
}

static int
typec_add_altmode(struct device *parent, struct typec_altmode *alt, int is_port)
{
	struct device *dev = &alt->dev;
	int ret;

	alt->mode_groups = kcalloc(alt->n_modes + 1,
				   sizeof(struct attibute_group *), GFP_KERNEL);
	if (!alt->mode_groups)
		return -ENOMEM;

	typec_init_modes(alt, is_port);

	dev->groups = alt->mode_groups;
	dev->release = typec_altmode_release;
	dev->parent = parent;
	/* TODO: if (!is_port) dev->bus = &typec_altmode_bus; */

	dev_set_name(dev, "%s.svid:%04x", dev_name(parent), alt->svid);

	ret = device_register(dev);
	if (ret) {
		put_device(dev);
		kfree(alt->mode_groups);
		return ret;
	}

	return 0;
}

static int __typec_register_altmodes(struct device *parent,
				     struct typec_altmode *alt_modes,
				     int is_port)
{
	struct typec_altmode *alt;
	int index;
	int ret;

	if (!alt_modes)
		return 0;

	for (alt = alt_modes, index = 0; alt->svid; alt++, index++) {
		ret = typec_add_altmode(parent, alt, is_port);
		if (ret)
			goto err;
	}

	return 0;
err:
	for (alt = alt_modes + index; index; alt--, index--)
		device_unregister(&alt->dev);

	return ret;
}

int typec_register_altmodes(struct device *dev, struct typec_altmode *alt_modes)
{
	if (dev->type == &typec_partner_dev_type) {
		struct typec_partner *p = container_of(dev,
						       struct typec_partner,
						       dev);
		p->alt_modes = alt_modes;
	} else if (dev->type == &typec_plug_dev_type) {
		struct typec_plug *p = container_of(dev, struct typec_plug,
						    dev);
		p->alt_modes = alt_modes;
	} else {
		return -ENODEV;
	}

	return __typec_register_altmodes(dev, alt_modes, false);
}
EXPORT_SYMBOL_GPL(typec_register_altmodes);

void typec_unregister_altmodes(struct device *dev)
{
	struct typec_altmode *alt_modes = NULL;
	struct typec_altmode *alt;

	if (dev->type == &typec_partner_dev_type) {
		struct typec_partner *p = container_of(dev,
						       struct typec_partner,
						       dev);
		alt_modes = p->alt_modes;
		p->alt_modes = NULL;
	} else if (dev->type == &typec_plug_dev_type) {
		struct typec_plug *p = container_of(dev, struct typec_plug,
						    dev);
		alt_modes = p->alt_modes;
		p->alt_modes = NULL;
	}

	if (!alt_modes)
		return;

	for (alt = alt_modes; alt->svid; alt++)
		device_unregister(&alt->dev);
}
EXPORT_SYMBOL_GPL(typec_unregister_altmodes);

/* ------------------------------------------------------------------------- */

static const char * const typec_roles[] = {
	[TYPEC_SINK]	= "sink",
	[TYPEC_SOURCE]	= "source",
};

static const char * const typec_data_roles[] = {
	[TYPEC_DEVICE]	= "device",
	[TYPEC_HOST]	= "host",
};

static ssize_t
preferred_role_store(struct device *dev, struct device_attribute *attr,
		     const char *buf, size_t size)
{
	struct typec_port *port = to_typec_port(dev);
	enum typec_role role;
	int ret;

	mutex_lock(&port->lock);

	if (port->cap->type != TYPEC_PORT_DRP) {
		dev_dbg(dev, "Preferred role only supported with DRP ports\n");
		ret = -EOPNOTSUPP;
		goto out;
	}

	if (!port->cap->try_role) {
		dev_dbg(dev, "Setting preferred role not supported\n");
		ret = -EOPNOTSUPP;
		goto out;
	}

	ret = match_string(typec_roles, ARRAY_SIZE(typec_roles), buf);
	if (ret < 0) {
		port->prefer_role = -1;
		ret = size;
		goto out;
	}

	role = ret;

	ret = port->cap->try_role(port->cap, role);
	if (ret)
		goto out;

	port->prefer_role = role;
	ret = size;
out:
	mutex_unlock(&port->lock);
	return ret;
}

static ssize_t
preferred_role_show(struct device *dev, struct device_attribute *attr,
		    char *buf)
{
	struct typec_port *port = to_typec_port(dev);
	ssize_t ret = 0;

	mutex_lock(&port->lock);

	if (port->prefer_role < 0)
		goto out;

	ret = sprintf(buf, "%s\n", typec_roles[port->prefer_role]);
out:
	mutex_unlock(&port->lock);
	return ret;
}
static DEVICE_ATTR_RW(preferred_role);

static ssize_t
current_data_role_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t size)
{
	struct typec_port *port = to_typec_port(dev);
	int ret = size;

	mutex_lock(&port->lock);

	if (port->cap->type != TYPEC_PORT_DRP) {
		dev_dbg(dev, "data role swap only supported with DRP ports\n");
		ret = -EOPNOTSUPP;
		goto out;
	}

	if (!port->cap->dr_set) {
		dev_dbg(dev, "data role swapping not supported\n");
		ret = -EOPNOTSUPP;
		goto out;
	}

	if (!port->connected)
		goto out;

	ret = match_string(typec_data_roles, ARRAY_SIZE(typec_data_roles), buf);
	if (ret < 0)
		goto out;

	ret = port->cap->dr_set(port->cap, ret);
	if (ret)
		goto out;

	ret = size;
out:
	mutex_unlock(&port->lock);
	return ret;
}

static ssize_t
current_data_role_show(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	struct typec_port *port = to_typec_port(dev);
	ssize_t ret;

	mutex_lock(&port->lock);
	ret = sprintf(buf, "%s\n", typec_data_roles[port->data_role]);
	mutex_unlock(&port->lock);

	return ret;
}
static DEVICE_ATTR_RW(current_data_role);

static ssize_t supported_data_roles_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct typec_port *port = to_typec_port(dev);
	size_t ret;

	mutex_lock(&port->lock);

	if (port->cap->type == TYPEC_PORT_DRP) {
		ret = sprintf(buf, "host, device\n");
		goto out;
	}

	ret = sprintf(buf, "%s\n", typec_data_roles[port->data_role]);
out:
	mutex_unlock(&port->lock);
	return ret;
}
static DEVICE_ATTR_RO(supported_data_roles);

static ssize_t current_power_role_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct typec_port *port = to_typec_port(dev);
	int ret = size;

	mutex_lock(&port->lock);

	if (!port->cap->usb_pd) {
		dev_dbg(dev, "power role swap only supported with USB PD\n");
		ret = -EOPNOTSUPP;
		goto out;
	}

	if (!port->cap->pr_set) {
		dev_dbg(dev, "power role swapping not supported\n");
		ret = -EOPNOTSUPP;
		goto out;
	}

	if (port->pwr_opmode != TYPEC_PWR_MODE_PD) {
		dev_dbg(dev, "partner unable to swap power role\n");
		ret = -EIO;
		goto out;
	}

	if (!port->connected)
		goto out;

	ret = match_string(typec_roles, ARRAY_SIZE(typec_roles), buf);
	if (ret < 0)
		goto out;

	ret = port->cap->pr_set(port->cap, ret);
	if (ret)
		goto out;

	ret = size;
out:
	mutex_unlock(&port->lock);
	return ret;
}

static ssize_t current_power_role_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct typec_port *port = to_typec_port(dev);
	ssize_t ret;

	mutex_lock(&port->lock);
	ret = sprintf(buf, "%s\n", typec_roles[port->pwr_role]);
	mutex_unlock(&port->lock);

	return ret;
}
static DEVICE_ATTR_RW(current_power_role);

static ssize_t supported_power_roles_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct typec_port *port = to_typec_port(dev);
	size_t ret;

	mutex_lock(&port->lock);

	if (port->cap->usb_pd || port->cap->type == TYPEC_PORT_DRP) {
		ret = sprintf(buf, "source, sink\n");
		goto out;
	}

	ret = sprintf(buf, "%s\n", typec_roles[port->pwr_role]);
out:
	mutex_unlock(&port->lock);
	return ret;
}
static DEVICE_ATTR_RO(supported_power_roles);

static const char * const typec_pwr_opmodes[] = {
	[TYPEC_PWR_MODE_USB]	= "USB",
	[TYPEC_PWR_MODE_1_5A]	= "USB Type-C 1.5A",
	[TYPEC_PWR_MODE_3_0A]	= "USB Type-C 3.0A",
	[TYPEC_PWR_MODE_PD]	= "USB Power Delivery",
};

static ssize_t power_operation_mode_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct typec_port *port = to_typec_port(dev);
	ssize_t ret;

	mutex_lock(&port->lock);
	ret = sprintf(buf, "%s\n", typec_pwr_opmodes[port->pwr_opmode]);
	mutex_unlock(&port->lock);

	return ret;
}
static DEVICE_ATTR_RO(power_operation_mode);

static ssize_t current_vconn_role_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct typec_port *port = to_typec_port(dev);
	int ret;

	mutex_lock(&port->lock);

	if (!port->cap->usb_pd) {
		dev_dbg(dev, "vconn swap only supported with USB PD\n");
		ret = -EOPNOTSUPP;
		goto out;
	}

	if (!port->cap->vconn_set) {
		dev_dbg(dev, "vconn swapping not supported\n");
		ret = -EOPNOTSUPP;
		goto out;
	}

	ret = match_string(typec_roles, ARRAY_SIZE(typec_roles), buf);
	if (ret < 0)
		goto out;

	ret = port->cap->vconn_set(port->cap, ret);
	if (ret)
		goto out;

	ret = size;
out:
	mutex_unlock(&port->lock);
	return ret;
}

static ssize_t current_vconn_role_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct typec_port *port = to_typec_port(dev);
	ssize_t ret;

	mutex_lock(&port->lock);
	ret = sprintf(buf, "%s\n", typec_roles[port->vconn_role]);
	mutex_unlock(&port->lock);

	return ret;
}
static DEVICE_ATTR_RW(current_vconn_role);

static ssize_t supported_accessory_modes_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct typec_port *port = to_typec_port(dev);
	enum typec_accessory *accessory;
	ssize_t ret = 0;
	int i;

	mutex_lock(&port->lock);
	if (port->cap->accessory)
		for (accessory = port->cap->accessory, i = 0;
		     i < port->cap->num_accessory; accessory++, i++)
			ret += sprintf(buf, "%s\n",
				       typec_accessory_modes[*accessory]);
	mutex_unlock(&port->lock);

	return ret;
}
static DEVICE_ATTR_RO(supported_accessory_modes);

static ssize_t supports_usb_power_delivery_show(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct typec_port *port = to_typec_port(dev);
	ssize_t ret;

	mutex_lock(&port->lock);
	ret = sprintf(buf, "%d\n", port->cap->usb_pd);
	mutex_unlock(&port->lock);

	return ret;
}
static DEVICE_ATTR_RO(supports_usb_power_delivery);

static struct attribute *typec_attrs[] = {
	&dev_attr_current_power_role.attr,
	&dev_attr_current_vconn_role.attr,
	&dev_attr_current_data_role.attr,
	&dev_attr_power_operation_mode.attr,
	&dev_attr_preferred_role.attr,
	&dev_attr_supported_accessory_modes.attr,
	&dev_attr_supported_data_roles.attr,
	&dev_attr_supported_power_roles.attr,
	&dev_attr_supports_usb_power_delivery.attr,
	NULL,
};

static const struct attribute_group typec_group = {
	.attrs = typec_attrs,
};

static const struct attribute_group *typec_groups[] = {
	&typec_group,
	NULL,
};

static int typec_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	int ret;

	ret = add_uevent_var(env, "TYPEC_PORT=%s", dev_name(dev));
	if (ret)
		dev_err(dev, "failed to add uevent TYPEC_PORT\n");

	return ret;
}

static void typec_release(struct device *dev)
{
	struct typec_port *port = to_typec_port(dev);

	ida_simple_remove(&typec_index_ida, port->id);
	kfree(port);
}

static struct device_type typec_port_dev_type = {
	.name = "typec_port",
	.groups = typec_groups,
	.uevent = typec_uevent,
	.release = typec_release,
};

struct typec_port *typec_register_port(struct device *dev,
				       const struct typec_capability *cap)
{
	struct typec_port *port;
	int ret;
	int id;

	port = kzalloc(sizeof(*port), GFP_KERNEL);
	if (!port)
		return ERR_PTR(-ENOMEM);

	id = ida_simple_get(&typec_index_ida, 0, 0, GFP_KERNEL);
	if (id < 0) {
		kfree(port);
		return ERR_PTR(id);
	}

	/* FIXME: a better approach for this */
	port->prefer_role = -1;

	port->id = id;
	port->cap = cap;
	port->dev.type = &typec_port_dev_type;
	port->dev.class = &typec_class;
	port->dev.parent = dev;
	dev_set_name(&port->dev, "usbc%d", id);
	mutex_init(&port->lock);

	typec_init_roles(port);

	ret = device_register(&port->dev);
	if (ret)
		goto reg_err;

	ret = __typec_register_altmodes(&port->dev, cap->alt_modes, true);
	if (ret)
		goto alt_err;

	return port;
alt_err:
	device_unregister(&port->dev);
reg_err:
	ida_simple_remove(&typec_index_ida, id);
	put_device(&port->dev);
	kfree(port);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(typec_register_port);

void typec_unregister_port(struct typec_port *port)
{
	struct typec_altmode *alt;

	WARN_ON(port->connected);

	if (port->cap->alt_modes)
		for (alt = port->cap->alt_modes; alt->svid; alt++)
			device_unregister(&alt->dev);
	device_unregister(&port->dev);
}
EXPORT_SYMBOL_GPL(typec_unregister_port);

static int __init typec_init(void)
{
	return class_register(&typec_class);
}
subsys_initcall(typec_init);

static void __exit typec_exit(void)
{
	class_unregister(&typec_class);
}
module_exit(typec_exit);

MODULE_AUTHOR("Heikki Krogerus <heikki.krogerus@linux.intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("USB Type-C Connector Class");
