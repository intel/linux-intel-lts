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
#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#ifdef VERBOSE_DEBUG
#define DVCT_IN() pr_debug("in\n")
#else
#define DVCT_IN() do {} while (0)
#endif

#include <linux/module.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/usb/debug.h>
#include "u_dvctrace.h"

enum {
	DVCT_IAD_DESC_POS,
	DVCT_CITF_DESC_POS,
	DVCT_DITF_DESC_POS,
	DVCT_EP_DESC_POS,
	DVCT_LS_NULL_DESC_POS, /*Low speed descriptors end with this one*/
	DVCT_EP_COMP_DESC_POS = DVCT_LS_NULL_DESC_POS,
	DVCT_LS_DESC_COUNT, /*Count of low speed descriptors*/
	DVCT_NULL_DESC_POS = DVCT_LS_DESC_COUNT,
	DVCT_HS_DESC_COUNT,/*Count of super speed descriptors*/
};

/*The full list of descriptors will look like:
 * IAD_DESCRIPTOR         -----|=> USB function specific
 * CONTROL_ITF_DESCRIPTOR -----|
 * SOURCE_SPECIFIC_DESCRIPTOR_0    ----|
 * ....                                |=> s_cnt descriptors provided by the
 * SOURCE_SPECIFIC_DESCRIPTOR_s_cnt----|   source device.
 * DATA_ITF_DESCRIPTOR -----|
 * ENDPOINT_DESCRIPTOR      |=> USB function specific
 * ....                -----|
 * This makes a good part of the descriptors to shift,
 * the following should help*/
#define DVCT_IAD_DESC_DYN_POS(s_cnt)		(DVCT_IAD_DESC_POS)
#define DVCT_CITF_DESC_DYN_POS(s_cnt)		(DVCT_CITF_DESC_POS)
#define DVCT_SOURCE_DESC_FIRST(s_cnt)		(DVCT_DITF_DESC_POS)
#define DVCT_DITF_DESC_DYN_POS(s_cnt)		((s_cnt)+DVCT_DITF_DESC_POS)
#define DVCT_EP_DESC_DYN_POS(s_cnt)		((s_cnt)+DVCT_EP_DESC_POS)
#define DVCT_EP_COMP_DESC_DYN_POS(s_cnt)	((s_cnt)+DVCT_EP_COMP_DESC_POS)
#define DVCT_LS_DESC_DYN_COUNT(s_cnt)		((s_cnt)+DVCT_LS_DESC_COUNT)
#define DVCT_HS_DESC_DYN_COUNT(s_cnt)		((s_cnt)+DVCT_HS_DESC_COUNT)

enum {
	DVCT_STR_IAD_IDX,
	DVCT_STR_C_ITF_IDX,
	DVCT_STR_D_ITF_IDX,
	DVCT_STR_NULL_IDX,	/*always last */
	DVCT_STR_COUNT,
};

static int dvct_alloc_desc(struct dvct_function *d_fun)
{
	int i;
	unsigned int s_desc_count = 0;
	struct usb_descriptor_header **s_desc;
	struct dvct_function_desc *desc = &d_fun->desc;

	DVCT_IN();

	if (d_fun->source_dev->desc) {
		for (s_desc = d_fun->source_dev->desc->dvc_spec;
		     s_desc && (*s_desc); s_desc++)
			s_desc_count++;
	}

	/*alloc the descriptors array */
	desc->fs =
	    kzalloc(DVCT_LS_DESC_DYN_COUNT(s_desc_count) *
		    sizeof(struct usb_descriptor_header *), GFP_KERNEL);
	if (!desc->fs)
		goto err_fs;

	desc->hs =
	    kzalloc(DVCT_LS_DESC_DYN_COUNT(s_desc_count) *
		    sizeof(struct usb_descriptor_header *), GFP_KERNEL);
	if (!desc->hs)
		goto err_hs;

	desc->ss =
	    kzalloc(DVCT_HS_DESC_DYN_COUNT(s_desc_count) *
		    sizeof(struct usb_descriptor_header *), GFP_KERNEL);
	if (!desc->ss)
		goto err_ss;

	/*IAD */
	desc->iad = kzalloc(sizeof(*desc->iad), GFP_KERNEL);
	if (!desc->iad)
		goto err_iad;

	desc->iad->bLength = sizeof(*desc->iad);
	desc->iad->bDescriptorType = USB_DT_INTERFACE_ASSOCIATION;
	desc->iad->bInterfaceCount = 2;
	desc->iad->bFunctionClass = USB_CLASS_DEBUG;
	desc->iad->bFunctionSubClass = USB_SUBCLASS_DVC_TRACE;
	desc->iad->bFunctionProtocol = d_fun->source_dev->protocol;
	/*bFirstInterface - updated on bind */

	desc->fs[DVCT_IAD_DESC_DYN_POS(s_desc_count)] =
	    (struct usb_descriptor_header *)desc->iad;
	desc->hs[DVCT_IAD_DESC_DYN_POS(s_desc_count)] =
	    (struct usb_descriptor_header *)desc->iad;
	desc->ss[DVCT_IAD_DESC_DYN_POS(s_desc_count)] =
	    (struct usb_descriptor_header *)desc->iad;

	/*Control interface */
	desc->c_itf = kzalloc(sizeof(*desc->c_itf), GFP_KERNEL);
	if (!desc->c_itf)
		goto err_c_itf;

	desc->c_itf->bLength = USB_DT_INTERFACE_SIZE;
	desc->c_itf->bDescriptorType = USB_DT_INTERFACE;
	desc->c_itf->bInterfaceClass = USB_CLASS_DEBUG;
	desc->c_itf->bInterfaceSubClass = USB_SUBCLASS_DEBUG_CONTROL;
	desc->c_itf->bInterfaceProtocol = d_fun->source_dev->protocol;

	desc->fs[DVCT_CITF_DESC_DYN_POS(s_desc_count)] =
	    (struct usb_descriptor_header *)desc->c_itf;
	desc->hs[DVCT_CITF_DESC_DYN_POS(s_desc_count)] =
	    (struct usb_descriptor_header *)desc->c_itf;
	desc->ss[DVCT_CITF_DESC_DYN_POS(s_desc_count)] =
	    (struct usb_descriptor_header *)desc->c_itf;

	if (d_fun->source_dev->desc) {
		/*Copy whatever the source device has provided */
		s_desc = d_fun->source_dev->desc->dvc_spec;
		for (i = 0; i < s_desc_count; i++) {
			desc->fs[DVCT_SOURCE_DESC_FIRST(s_desc_count) + i]
			    = s_desc[i];
			desc->hs[DVCT_SOURCE_DESC_FIRST(s_desc_count) + i]
			    = s_desc[i];
			desc->ss[DVCT_SOURCE_DESC_FIRST(s_desc_count) + i]
			    = s_desc[i];
		}
	}
	/*Data interface */
	desc->d_itf = kzalloc(sizeof(*desc->d_itf), GFP_KERNEL);
	if (!desc->d_itf)
		goto err_d_itf;

	desc->d_itf->bLength = USB_DT_INTERFACE_SIZE;
	desc->d_itf->bDescriptorType = USB_DT_INTERFACE;
	desc->d_itf->bNumEndpoints = 1;
	desc->d_itf->bInterfaceClass = USB_CLASS_DEBUG;
	desc->d_itf->bInterfaceSubClass = USB_SUBCLASS_DVC_TRACE;
	desc->d_itf->bInterfaceProtocol = d_fun->source_dev->protocol;

	desc->fs[DVCT_DITF_DESC_DYN_POS(s_desc_count)] =
	    (struct usb_descriptor_header *)desc->d_itf;
	desc->hs[DVCT_DITF_DESC_DYN_POS(s_desc_count)] =
	    (struct usb_descriptor_header *)desc->d_itf;
	desc->ss[DVCT_DITF_DESC_DYN_POS(s_desc_count)] =
	    (struct usb_descriptor_header *)desc->d_itf;

	/*Full Speed ep */
	desc->fs_ep = kzalloc(sizeof(*desc->fs_ep), GFP_KERNEL);
	if (!desc->fs_ep)
		goto err_fs_ep;

	desc->fs_ep->bLength = USB_DT_ENDPOINT_SIZE;
	desc->fs_ep->bDescriptorType = USB_DT_ENDPOINT;
	desc->fs_ep->bEndpointAddress = USB_DIR_IN;
	desc->fs_ep->bmAttributes = USB_ENDPOINT_XFER_BULK;
	desc->fs_ep->wMaxPacketSize = cpu_to_le16(64);

	desc->fs[DVCT_EP_DESC_DYN_POS(s_desc_count)] =
	    (struct usb_descriptor_header *)desc->fs_ep;

	/*High Speed ep */
	desc->hs_ep = kzalloc(sizeof(*desc->hs_ep), GFP_KERNEL);
	if (!desc->hs_ep)
		goto err_hs_ep;

	desc->hs_ep->bLength = USB_DT_ENDPOINT_SIZE;
	desc->hs_ep->bDescriptorType = USB_DT_ENDPOINT;
	desc->hs_ep->bEndpointAddress = USB_DIR_IN;
	desc->hs_ep->bmAttributes = USB_ENDPOINT_XFER_BULK;
	desc->hs_ep->wMaxPacketSize = cpu_to_le16(512);

	desc->hs[DVCT_EP_DESC_DYN_POS(s_desc_count)] =
	    (struct usb_descriptor_header *)desc->hs_ep;

	/*Super Speed ep */
	desc->ss_ep = kzalloc(sizeof(*desc->ss_ep), GFP_KERNEL);
	if (!desc->ss_ep)
		goto err_ss_ep;

	desc->ss_ep->bLength = USB_DT_ENDPOINT_SIZE;
	desc->ss_ep->bDescriptorType = USB_DT_ENDPOINT;
	desc->ss_ep->bEndpointAddress = USB_DIR_IN;
	desc->ss_ep->bmAttributes = USB_ENDPOINT_XFER_BULK;
	desc->ss_ep->wMaxPacketSize = cpu_to_le16(1024);

	desc->ss[DVCT_EP_DESC_DYN_POS(s_desc_count)] =
	    (struct usb_descriptor_header *)desc->ss_ep;

	/*Super Speed ep comp */
	desc->ss_ep_comp = kzalloc(sizeof(*desc->ss_ep_comp), GFP_KERNEL);
	if (!desc->ss_ep_comp)
		goto err_ss_ep_comp;

	desc->ss_ep_comp->bLength = USB_DT_SS_EP_COMP_SIZE;
	desc->ss_ep_comp->bDescriptorType = USB_DT_SS_ENDPOINT_COMP;

	desc->ss[DVCT_EP_COMP_DESC_DYN_POS(s_desc_count)] =
	    (struct usb_descriptor_header *)desc->ss_ep_comp;

	/* strings */
	/*the table */
	desc->str.language = 0x0409;	/*en-us */
	desc->str.strings =
	    kzalloc(DVCT_STR_COUNT * sizeof(struct usb_string), GFP_KERNEL);
	if (!desc->str.strings)
		goto err_str;

	/*lookup table */
	desc->lk_tbl =
	    kzalloc(DVCT_STR_COUNT * sizeof(struct dvct_string_lookup),
		    GFP_KERNEL);
	if (!desc->lk_tbl)
		goto err_str_lk;

	/*actual strings */
	 /*IAD*/
	    desc->str.strings[DVCT_STR_IAD_IDX].s =
	    kasprintf(GFP_KERNEL, "DvC Trace (%s)",
		      dev_name(&d_fun->source_dev->device));
	if (!desc->str.strings[DVCT_STR_IAD_IDX].s)
		goto err_str_iad;

	desc->lk_tbl[DVCT_STR_IAD_IDX].str =
	    &desc->str.strings[DVCT_STR_IAD_IDX];
	desc->lk_tbl[DVCT_STR_IAD_IDX].id = &desc->iad->iFunction;

	/*control */
	desc->str.strings[DVCT_STR_C_ITF_IDX].s =
	    kasprintf(GFP_KERNEL, "DvC Trace Control (%s)",
		      dev_name(&d_fun->source_dev->device));
	if (!desc->str.strings[DVCT_STR_C_ITF_IDX].s)
		goto err_str_ctrl;

	desc->lk_tbl[DVCT_STR_C_ITF_IDX].str =
	    &desc->str.strings[DVCT_STR_C_ITF_IDX];
	desc->lk_tbl[DVCT_STR_C_ITF_IDX].id = &desc->c_itf->iInterface;

	/*data */
	desc->str.strings[DVCT_STR_D_ITF_IDX].s =
	    kasprintf(GFP_KERNEL, "DvC Trace Data (%s)",
		      dev_name(&d_fun->source_dev->device));
	if (!desc->str.strings[DVCT_STR_D_ITF_IDX].s)
		goto err_str_data;

	desc->lk_tbl[DVCT_STR_D_ITF_IDX].str =
	    &desc->str.strings[DVCT_STR_D_ITF_IDX];
	desc->lk_tbl[DVCT_STR_D_ITF_IDX].id = &desc->d_itf->iInterface;

	return 0;
/*cleanup*/
err_str_data:
	kfree(desc->str.strings[DVCT_STR_C_ITF_IDX].s);
err_str_ctrl:
	kfree(desc->str.strings[DVCT_STR_IAD_IDX].s);
err_str_iad:
	kfree(desc->lk_tbl);
err_str_lk:
	kfree(desc->str.strings);
err_str:
	kfree(desc->ss_ep_comp);
err_ss_ep_comp:
	kfree(desc->ss_ep);
err_ss_ep:
	kfree(desc->hs_ep);
err_hs_ep:
	kfree(desc->fs_ep);
err_fs_ep:
	kfree(desc->d_itf);
err_d_itf:
	kfree(desc->c_itf);
err_c_itf:
	kfree(desc->iad);
err_iad:
	kfree(desc->ss);
err_ss:
	kfree(desc->hs);
err_hs:
	kfree(desc->fs);
err_fs:
	pr_err("Failed OFM");
	return -ENOMEM;
}

static void dvct_free_desc(struct dvct_function *d_fun)
{
	struct dvct_function_desc *desc = &d_fun->desc;

	DVCT_IN();

	kfree(desc->str.strings[DVCT_STR_D_ITF_IDX].s);
	kfree(desc->str.strings[DVCT_STR_C_ITF_IDX].s);
	kfree(desc->str.strings[DVCT_STR_IAD_IDX].s);
	kfree(desc->lk_tbl);
	kfree(desc->str.strings);
	kfree(desc->ss_ep_comp);
	kfree(desc->ss_ep);
	kfree(desc->hs_ep);
	kfree(desc->fs_ep);
	kfree(desc->d_itf);
	kfree(desc->c_itf);
	kfree(desc->iad);
	kfree(desc->ss);
	kfree(desc->hs);
	kfree(desc->fs);
}

ssize_t dvct_start_transfer(struct dvct_function *d_fun, u8 config)
{
	DVCT_IN();
	if (!dvct_get_status(&d_fun->status, DVCT_MASK_ONLINE))
		return -EIO;

	d_fun->trace_config = config;
	return d_fun->source_drv->start_transfer(d_fun->source_dev, config);
}
EXPORT_SYMBOL(dvct_start_transfer);

int dvct_stop_transfer(struct dvct_function *d_fun)
{

	DVCT_IN();
	if (!dvct_get_status(&d_fun->status, DVCT_MASK_ONLINE))
		return -EIO;

	if (dvct_get_status(&d_fun->status, DVCT_MASK_TRANS)) {
		d_fun->trace_config = 0;
		return d_fun->source_drv->stop_transfer(d_fun->source_dev);
	}

	return 0;
}
EXPORT_SYMBOL(dvct_stop_transfer);

static int dvct_strings_setup(struct usb_composite_dev *cdev,
			      struct usb_string *strings,
			      struct dvct_string_lookup *lk_tbl)
{
	int status;
	struct dvct_string_lookup *str_lk;

	DVCT_IN();
	if (!strings || !lk_tbl)
		return -EINVAL;

	status = usb_string_ids_tab(cdev, strings);
	if (status < 0)
		return status;

	for (str_lk = lk_tbl; str_lk->str; str_lk++) {
		*str_lk->id = str_lk->str->id;
		pr_info("Setting id %d for str \"%s\"\n", str_lk->str->id,
			str_lk->str->s);
	}
	return 0;
}

static int dvct_setup(struct usb_function *func,
		      const struct usb_ctrlrequest *ctrl)
{
	int status = -EOPNOTSUPP;
	u16 w_index;
	u16 w_value;
	u16 w_length;
	u8 b_index_value;
	struct dvct_function *d_fun = to_dvct_function(func);

	DVCT_IN();

	w_index = le16_to_cpu(ctrl->wIndex);
	w_value = le16_to_cpu(ctrl->wValue);
	w_length = le16_to_cpu(ctrl->wLength);
	b_index_value = (u8) (w_index >> 8);

	if (ctrl->bRequestType !=
	    (USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE))
		goto done;

	switch (ctrl->bRequest) {
	case DC_REQUEST_SET_RESET:

		pr_info("SET_RESET v%04x i%04x l%u\n",
			w_value, w_index, w_length);

		dvct_stop_transfer(d_fun);
		status = 0;
		break;

	case DC_REQUEST_SET_TRACE:
	/* There are some inconsistencies in the spec regarding some of the
	 * control requests, like SET/GET _TRACE, where even if the message
	 * is defined as interface specific the wIndex field is used for
	 * something else, making these request unusable in a "standard"
	 * composite device.
	 * To get around this we expect the interface to be specified in
	 * wIndex 7:0 and any other values in wIndex 15:8.
	 * A "special" composite implementation is free to treat these setup
	 * requests "on spec" and call directly dvct_start_transfer and/or
	 * dvct_stop_transfer (exported in u_dvctrace.h).
	 */
		pr_info("SET_TRACE v%04x i%04x l%u\n",
			w_value, w_index, w_length);

		if (!b_index_value) {
			dvct_stop_transfer(d_fun);
			status = 0;
		} else {
			status = dvct_start_transfer(d_fun, b_index_value);
		}
		break;
	}

done:
	if (status >= 0) {
		d_fun->cdev->req->zero = 0;
		d_fun->cdev->req->length = 0;
		status =
		    usb_ep_queue(d_fun->cdev->gadget->ep0, d_fun->cdev->req,
				 GFP_ATOMIC);
		if (status)
			pr_err("Setup response queue error\n");
	} else {
		pr_debug("Unexpected request %02x.%02x v%04x i%04x l%u\n",
			 ctrl->bRequestType, ctrl->bRequest, w_value, w_index,
			 w_length);
	}

	return status;
}

static int dvct_function_bind(struct usb_configuration *cconfig,
			      struct usb_function *func)
{
	int id, ret;
	struct usb_ep *ep;
	struct dvct_function *d_fun = to_dvct_function(func);

	DVCT_IN();
	d_fun->cdev = cconfig->cdev;

	/*allocate id's */
	/*strings. not crucial just print on failure */
	if (d_fun->source_dev->desc && d_fun->source_dev->desc->str.strings) {
		ret = dvct_strings_setup(d_fun->cdev,
					 d_fun->source_dev->desc->str.strings,
					 d_fun->source_dev->desc->lk_tbl);
		if (ret)
			pr_warn("Cannot allocate source device string id's\n");
	}
	ret = dvct_strings_setup(d_fun->cdev, d_fun->desc.str.strings,
				 d_fun->desc.lk_tbl);
	if (ret)
		pr_warn("Cannot allocate function string id's\n");

	/* allocate interface ID(s) */
	id = usb_interface_id(cconfig, func);
	if (id < 0)
		return id;

	d_fun->desc.c_itf->bInterfaceNumber = id;
	d_fun->desc.iad->bFirstInterface = id;

	pr_debug("Setting id %d for dvc-control interface\n", id);

	id = usb_interface_id(cconfig, func);
	if (id < 0)
		return id;

	d_fun->desc.d_itf->bInterfaceNumber = id;

	pr_debug("Setting id %d for dvc-trace-data interface\n", id);

	/* allocate endpoints */
	d_fun->desc.ss_ep->wMaxPacketSize = 0;	/*get the real max */
	ep = usb_ep_autoconfig_ss(d_fun->cdev->gadget,
				  d_fun->desc.ss_ep, d_fun->desc.ss_ep_comp);

	if (!ep) {
		pr_err("usb_ep_autoconfig for ep_in failed\n");
		return -ENODEV;
	}

	/*copy over the endpoint parameters */
	d_fun->desc.hs_ep->bEndpointAddress =
	    d_fun->desc.ss_ep->bEndpointAddress;
	d_fun->desc.fs_ep->bEndpointAddress =
	    d_fun->desc.ss_ep->bEndpointAddress;

	if (le16_to_cpu(d_fun->desc.hs_ep->wMaxPacketSize) >
	    le16_to_cpu(d_fun->desc.ss_ep->wMaxPacketSize))
		d_fun->desc.hs_ep->wMaxPacketSize =
		    d_fun->desc.ss_ep->wMaxPacketSize;

	if (le16_to_cpu(d_fun->desc.fs_ep->wMaxPacketSize) >
	    le16_to_cpu(d_fun->desc.ss_ep->wMaxPacketSize))
		d_fun->desc.fs_ep->wMaxPacketSize =
		    d_fun->desc.ss_ep->wMaxPacketSize;

	pr_info("usb_ep_autoconfig %s, addr 0x%hhx, size ss=%hu hs=%hu fs=%hu\n",
		ep->name,
		d_fun->desc.ss_ep->bEndpointAddress,
		d_fun->desc.ss_ep->wMaxPacketSize,
		d_fun->desc.hs_ep->wMaxPacketSize,
		d_fun->desc.fs_ep->wMaxPacketSize);

	ep->driver_data = d_fun;	/* claim the endpoint */
	d_fun->ep_in = ep;

	ret = d_fun->source_drv->binded(d_fun->source_dev, ep,
					&d_fun->function);

	return ret;
}

static void dvct_function_unbind(struct usb_configuration *c,
				 struct usb_function *func)
{
	struct dvct_function *d_fun = to_dvct_function(func);

	DVCT_IN();
	dvct_clr_status(&d_fun->status, DVCT_MASK_ONLINE);
	d_fun->online_data = 0;
	d_fun->online_ctrl = 0;

	d_fun->source_drv->unbinded(d_fun->source_dev);
}

static int dvct_function_set_alt(struct usb_function *func,
				 unsigned intf, unsigned alt)
{
	struct dvct_function *d_fun = to_dvct_function(func);
	struct usb_composite_dev *cdev = func->config->cdev;
	int ret;

	DVCT_IN();

	if (intf == d_fun->desc.c_itf->bInterfaceNumber) {
		d_fun->online_ctrl = 1;
		pr_debug("dvc-control interface %u set alt %u\n", intf, alt);
	}

	if (intf == d_fun->desc.d_itf->bInterfaceNumber) {
		ret = config_ep_by_speed(cdev->gadget, func, d_fun->ep_in);
		if (ret) {
			pr_debug("intf: %d alt: %d ep_by_speed in err %d\n",
				 intf, alt, ret);
			return ret;
		}

		ret = usb_ep_enable(d_fun->ep_in);
		if (ret) {
			pr_debug("intf: %d alt: %d ep_enable in err %d\n",
				 intf, alt, ret);
			return ret;
		}
		d_fun->online_data = 1;
	}

	pr_info("dvc-trace interface %u set alt %u\n", intf, alt);

	if (unlikely(dvct_get_status(&d_fun->status, DVCT_MASK_TRANS)))
		dvct_stop_transfer(d_fun);

	if (d_fun->online_data && d_fun->online_ctrl) {
		dvct_set_status(&d_fun->status, DVCT_MASK_ONLINE);
		if (d_fun->source_drv->connected)
			d_fun->source_drv->connected(d_fun->source_dev,
						     cdev->gadget->speed);
	}
	return 0;
}

static void dvct_function_disable(struct usb_function *func)
{
	struct dvct_function *d_fun = to_dvct_function(func);
	struct usb_composite_dev *cdev;

	DVCT_IN();

	cdev = d_fun->cdev;

	if (dvct_get_status(&d_fun->status, DVCT_MASK_TRANS))
		dvct_stop_transfer(d_fun);

	usb_ep_disable(d_fun->ep_in);

	d_fun->online_ctrl = 0;
	d_fun->online_data = 0;

	if (d_fun->source_drv->disconnected)
		d_fun->source_drv->disconnected(d_fun->source_dev);

	pr_debug("%s disabled\n", d_fun->function.name);
}

CONFIGFS_ATTR_STRUCT(dvct_function_inst);

static ssize_t dvct_attr_show(struct config_item *item,
				    struct configfs_attribute *attr, char *page)
{
	struct dvct_function_inst *d_inst;
	struct dvct_function_inst_attribute *d_fun_attr;
	ssize_t ret = 0;

	DVCT_IN();
	d_inst = container_of(to_config_group(item), struct dvct_function_inst,
			      instance.group);
	d_fun_attr = container_of(attr, struct dvct_function_inst_attribute,
				  attr);

	if (d_fun_attr->show)
		ret = d_fun_attr->show(d_inst, page);

	return ret;
}

static void dvct_attr_release(struct config_item *item)
{
	struct dvct_function_inst *d_inst;

	DVCT_IN();
	d_inst = container_of(to_config_group(item), struct dvct_function_inst,
			      instance.group);
	usb_put_function_instance(&d_inst->instance);
}

static struct configfs_item_operations dvctrace_item_ops = {
	.release        = dvct_attr_release,
	.show_attribute = dvct_attr_show,
};

static ssize_t dvct_device_show(struct dvct_function_inst *d_inst, char *page)
{
	return sprintf(page, "%s\n", d_inst->source_dev->name_add);
}

static struct dvct_function_inst_attribute f_dvctrace_device =
	__CONFIGFS_ATTR_RO(source_dev, dvct_device_show);

static struct configfs_attribute *dvct_attrs[] = {
	&f_dvctrace_device.attr,
	NULL,
};

static struct config_item_type dvct_func_type = {
	.ct_item_ops    = &dvctrace_item_ops,
	.ct_attrs	= dvct_attrs,
	.ct_owner       = THIS_MODULE,
};

static void dvct_free_func_inst(struct usb_function_instance *inst)
{
	struct dvct_function_inst *d_inst;

	DVCT_IN();
	d_inst = to_dvct_function_inst(inst);

	spin_lock(&d_inst->source_dev->lock);
	d_inst->source_dev->instance_taken = 0;
	spin_unlock(&d_inst->source_dev->lock);

	kfree(d_inst);
}

static int dvct_set_inst_name(struct usb_function_instance *inst,
			      const char *name)
{
	struct dvct_function_inst *d_inst;
	struct dvct_source_device *new_src;
	struct dvct_source_device *old_src;

	DVCT_IN();
	d_inst = to_dvct_function_inst(inst);
	old_src = d_inst->source_dev;

	new_src = dvct_source_find_free_by_name(name);

	if (IS_ERR_OR_NULL(new_src))
		return -ENODEV;

	if (new_src) {
		spin_lock(&new_src->lock);
		spin_lock(&old_src->lock);

		d_inst->source_dev = new_src;
		new_src->instance_taken = 1;
		old_src->instance_taken = 0;

		spin_unlock(&old_src->lock);
		spin_unlock(&new_src->lock);
	}
	return 0;
}

static struct usb_function_instance *dvct_alloc_inst(void)
{
	struct dvct_function_inst *d_inst;
	struct dvct_source_device *src_dev = NULL;

	DVCT_IN();
	/*get the first free source, this will change via set name
	 * if available */
	src_dev = dvct_source_find_free();

	if (IS_ERR_OR_NULL(src_dev))
		return ERR_PTR(-ENODEV);

	d_inst = kzalloc(sizeof(*d_inst), GFP_KERNEL);

	if (!d_inst)
		return ERR_PTR(-ENOMEM);

	d_inst->instance.free_func_inst = dvct_free_func_inst;
	d_inst->instance.set_inst_name = dvct_set_inst_name;

	spin_lock(&src_dev->lock);
	d_inst->source_dev = src_dev;
	src_dev->instance_taken = 1;
	spin_unlock(&src_dev->lock);

	config_group_init_type_name(&d_inst->instance.group,
				    "", &dvct_func_type);
	return &d_inst->instance;
}

static void dvct_free_func(struct usb_function *func)
{
	struct dvct_function *d_fun = to_dvct_function(func);

	DVCT_IN();
	d_fun->source_drv->deactivate(d_fun->source_dev);

	spin_lock(&d_fun->source_dev->lock);
	d_fun->source_dev->function_taken = 0;
	spin_unlock(&d_fun->source_dev->lock);

	dvct_free_desc(d_fun);

	kfree(d_fun);
}

static struct usb_function *dvct_alloc_func(struct usb_function_instance *inst)
{
	int ret;
	struct dvct_function *d_fun;
	struct dvct_function_inst *d_inst = to_dvct_function_inst(inst);

	DVCT_IN();
	d_fun = kzalloc(sizeof(struct dvct_function), GFP_KERNEL);
	if (!d_fun)
		return ERR_PTR(-ENOMEM);

	d_fun->source_dev = d_inst->source_dev;
	d_fun->source_drv = dvct_source_get_drv(d_fun->source_dev);
	d_fun->trace_config = 0;

	spin_lock(&d_fun->source_dev->lock);
	d_fun->source_dev->function_taken = 1;
	spin_unlock(&d_fun->source_dev->lock);

	ret = d_fun->source_drv->activate(d_fun->source_dev, &d_fun->status);
	if (ret) {
		pr_err("Cannot activate source device %d\n", ret);
		goto err;
	}

	ret = dvct_alloc_desc(d_fun);
	if (ret)
		goto err_des;

	/*String table*/
	/*1 - source dev (if present) , 1 - function, 1 - NULL */
	if (d_fun->source_dev->desc && d_fun->source_dev->desc->str.strings)
		d_fun->function.strings =
		    kzalloc(3 * sizeof(struct usb_gadget_strings), GFP_KERNEL);
	else
		d_fun->function.strings =
		    kzalloc(2 * sizeof(struct usb_gadget_strings), GFP_KERNEL);

	if (!d_fun->function.strings) {
		ret = -ENOMEM;
		goto err_string_table;
	}

	d_fun->function.strings[0] = &d_fun->desc.str;
	if (d_fun->source_dev->desc && d_fun->source_dev->desc->str.strings)
		d_fun->function.strings[1] = &d_fun->source_dev->desc->str;

	d_fun->function.name = "dvctrace";
	d_fun->function.fs_descriptors = d_fun->desc.fs;
	d_fun->function.hs_descriptors = d_fun->desc.hs;
	d_fun->function.ss_descriptors = d_fun->desc.ss;
	d_fun->function.bind = dvct_function_bind;
	d_fun->function.unbind = dvct_function_unbind;
	d_fun->function.set_alt = dvct_function_set_alt;
	d_fun->function.disable = dvct_function_disable;
	d_fun->function.free_func = dvct_free_func;
	d_fun->function.setup = dvct_setup;

	return &d_fun->function;

err_string_table:
	dvct_free_desc(d_fun);
err_des:
	d_fun->source_drv->deactivate(d_fun->source_dev);
err:
	kfree(d_fun);
	return ERR_PTR(ret);
}

DECLARE_USB_FUNCTION_INIT(dvctrace, dvct_alloc_inst, dvct_alloc_func);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DvC-Trace function driver");
MODULE_AUTHOR("Traian Schiau <traianx.schiau@intel.com>");
