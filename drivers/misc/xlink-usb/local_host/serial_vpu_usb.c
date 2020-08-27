// SPDX-License-Identifier: GPL-2.0-only
/*
 * serial_vpu_usb.c -- USB gadget serial driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2, as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include "u_xlink.h"

/* Defines */
#define MX_VERSION_STR			"v1.0"
#define MX_VERSION_NUM			0x1000
#define MX_VERSION_NAME			"Keembay VPU USB" MX_VERSION_STR

/*-------------------------------------------------------------------------*/
USB_GADGET_COMPOSITE_OPTIONS();

#define MX_VENDOR_ID			0x03e7	/* MyraidX Vendor ID */
#define MX_PRODUCT_ID		0x2485	/* MyraidX Product ID */

/* string IDs are assigned dynamically */

#define STRING_DESCRIPTION_IDX		USB_GADGET_FIRST_AVAIL_IDX

static struct usb_string strings_dev[] = {
	[USB_GADGET_MANUFACTURER_IDX].s = "",
	[USB_GADGET_PRODUCT_IDX].s = MX_VERSION_NAME,
	[USB_GADGET_SERIAL_IDX].s = "",
	[STRING_DESCRIPTION_IDX].s = NULL,
	{  } /* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_device_descriptor device_desc = {
	.bLength =		USB_DT_DEVICE_SIZE,
	.bDescriptorType =	USB_DT_DEVICE,
	.bDeviceSubClass =	0,
	.bDeviceProtocol =	0,
	.idVendor =		cpu_to_le16(MX_VENDOR_ID),
	.bcdDevice = cpu_to_le16(MX_VERSION_NUM),
	.bNumConfigurations =	1,
};

static const struct usb_descriptor_header *otg_desc[2];

/* Module */
MODULE_DESCRIPTION(MX_VERSION_NAME);
MODULE_AUTHOR("Tan Seng Kai");
MODULE_AUTHOR("Lai Jun Ann");
MODULE_LICENSE("GPL");

static struct usb_configuration serial_config_driver = {
	.bmAttributes	= USB_CONFIG_ATT_SELFPOWER,
};

static struct usb_function_instance *fi_serial[MAX_U_SERIAL_PORTS];
static struct usb_function *f_serial[MAX_U_SERIAL_PORTS];

static int serial_register_ports(struct usb_composite_dev *cdev,
		struct usb_configuration *c, const char *f_name)
{
	int i;
	int ret;

	ret = usb_add_config_only(cdev, c);
	if (ret)
		goto out;

	for (i = 0; i < 1; i++) {

		fi_serial[i] = usb_get_function_instance(f_name);
		if (IS_ERR(fi_serial[i])) {
			ret = PTR_ERR(fi_serial[i]);
			goto fail;
		}

		f_serial[i] = usb_get_function(fi_serial[i]);
		if (IS_ERR(f_serial[i])) {
			ret = PTR_ERR(f_serial[i]);
			goto err_get_func;
		}

		ret = usb_add_function(c, f_serial[i]);
		if (ret)
			goto err_add_func;
	}

	return 0;

err_add_func:
	usb_put_function(f_serial[i]);
err_get_func:
	usb_put_function_instance(fi_serial[i]);

fail:
	i--;
	while (i >= 0) {
		usb_put_function(f_serial[i]);
		usb_put_function_instance(fi_serial[i]);
		i--;
	}
out:
	return ret;
}

static int vpu_bind(struct usb_composite_dev *cdev)
{
	int			status;

	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */

	status = usb_string_ids_tab(cdev, strings_dev);
	if (status < 0)
		goto fail;
	device_desc.iManufacturer = strings_dev[USB_GADGET_MANUFACTURER_IDX].id;
	device_desc.iProduct = strings_dev[USB_GADGET_PRODUCT_IDX].id;
	status = strings_dev[STRING_DESCRIPTION_IDX].id;
	serial_config_driver.iConfiguration = status;

	if (gadget_is_otg(cdev->gadget)) {
		if (!otg_desc[0]) {
			struct usb_descriptor_header *usb_desc;

			usb_desc = usb_otg_descriptor_alloc(cdev->gadget);
			if (!usb_desc) {
				status = -ENOMEM;
				goto fail;
			}
			usb_otg_descriptor_init(cdev->gadget, usb_desc);
			otg_desc[0] = usb_desc;
			otg_desc[1] = NULL;
		}
		serial_config_driver.descriptors = otg_desc;
		serial_config_driver.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	}
	/* register our configuration */
	status  = serial_register_ports(cdev, &serial_config_driver,
			"vpu");
	usb_ep_autoconfig_reset(cdev->gadget);

	if (status < 0)
		goto fail1;

	usb_composite_overwrite_options(cdev, &coverwrite);
	INFO(cdev, "%s\n", MX_VERSION_NAME);

	return 0;
fail1:
	kfree(otg_desc[0]);
	otg_desc[0] = NULL;
fail:
	return status;
}

static int vpu_unbind(struct usb_composite_dev *cdev)
{
	int i;

	for (i = 0; i < 1; i++) {
		usb_put_function(f_serial[i]);
		usb_put_function_instance(fi_serial[i]);
	}

	kfree(otg_desc[0]);
	otg_desc[0] = NULL;

	return 0;
}

static struct usb_composite_driver gserial_driver = {
	.name		= "vpu",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.max_speed	= USB_SPEED_SUPER,
	.bind		= vpu_bind,
	.unbind		= vpu_unbind,
};

static int __init init(void)
{
	/* We *could* export two configs; that'd be much cleaner...
	 * but neither of these product IDs was defined that way.
	 */
	serial_config_driver.label = "VPU USB config";
	serial_config_driver.bConfigurationValue = 2;
	device_desc.bDeviceClass = USB_CLASS_COMM;
	device_desc.idProduct =
			cpu_to_le16(MX_PRODUCT_ID);

	strings_dev[STRING_DESCRIPTION_IDX].s = serial_config_driver.label;

	return usb_composite_probe(&gserial_driver);
}
module_init(init);

static void __exit cleanup(void)
{
	usb_composite_unregister(&gserial_driver);
}
module_exit(cleanup);
