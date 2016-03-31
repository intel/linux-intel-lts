
/*
 * Gadget Driver for DvC.Trace Function
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

#ifndef __U_DVCTRACE_H
#define __U_DVCTRACE_H

#include <linux/usb/composite.h>
#include <linux/dvctrace.h>

struct dvct_function_desc {
	struct usb_descriptor_header **fs;
	struct usb_descriptor_header **hs;
	struct usb_descriptor_header **ss;

	/*special descriptors, update on bind */
	struct usb_interface_assoc_descriptor *iad;
	struct usb_interface_descriptor *d_itf;
	struct usb_interface_descriptor *c_itf;
	struct usb_endpoint_descriptor *fs_ep;
	struct usb_endpoint_descriptor *hs_ep;
	struct usb_endpoint_descriptor *ss_ep;
	struct usb_ss_ep_comp_descriptor *ss_ep_comp;

	/* strings */
	struct usb_gadget_strings str;
};

struct dvct_function {
	struct usb_function function;
	struct usb_composite_dev *cdev;
	struct usb_ep *ep_in;

	u32 online_data:1;	/*set to one when the data itf is set */
	u32 online_ctrl:1;	/*set to one when the control itf is set */
	atomic_t status;

	struct dvct_source_device *source_dev;
	struct dvct_source_driver *source_drv;

	u8 trace_config;
	struct dvct_function_desc desc;
};

struct dvct_function_inst {
	struct usb_function_instance instance;
	struct dvct_source_device *source_dev;
};

#define to_dvct_function_inst(inst) \
	container_of(inst, struct dvct_function_inst, instance)

#define to_dvct_function(func) \
	container_of(func, struct dvct_function, function)

ssize_t dvct_start_transfer(struct dvct_function *dev, u8 config);
int dvct_stop_transfer(struct dvct_function *dev);

#endif /*__U_DVCTRACE_H*/
