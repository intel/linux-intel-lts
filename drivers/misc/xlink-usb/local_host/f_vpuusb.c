// SPDX-License-Identifier: GPL-2.0-only
/*
 * f_vpuusb.c - vpu usb serial function driver
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

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/err.h>
#include "u_xlink.h"
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/reboot.h>
#include <linux/xlink_drv_inf.h>

/*
 * This VPU CDC ACM function support just wraps control functions and
 * notifications around the generic serial-over-usb code.
 *
 */

struct f_acm {
	struct gserial			port;
	u8				ctrl_id, data_id;
	u8				port_num;

	u8				pending;

	/* lock is mostly for pending and notify_req ... they get accessed
	 * by callbacks both from tty (open/close/break) under its spinlock,
	 * and notify_req.complete() which can't use that lock.
	 */
	spinlock_t			lock;
	struct device           *device;
	struct usb_ep			*notify;
	struct usb_request		*notify_req;

	struct usb_cdc_line_coding	port_line_coding;	/* 8-N-1 etc */

	/* SetControlLineState request -- CDC 1.1 section 6.2.14 (INPUT) */
	u16				port_handshake_bits;
#define ACM_CTRL_RTS	(1 << 1)	/* unused with full duplex */
#define ACM_CTRL_DTR	(1 << 0)	/* host is ready for data r/w */

	/* SerialState notification -- CDC 1.1 section 6.3.5 (OUTPUT) */
	u16				serial_state;
#define ACM_CTRL_OVERRUN	(1 << 6)
#define ACM_CTRL_PARITY		(1 << 5)
#define ACM_CTRL_FRAMING	(1 << 4)
#define ACM_CTRL_RI		(1 << 3)
#define ACM_CTRL_BRK		(1 << 2)
#define ACM_CTRL_DSR		(1 << 1)
#define ACM_CTRL_DCD		(1 << 0)
};

static inline struct f_acm *func_to_acm(struct usb_function *f)
{
	return container_of(f, struct f_acm, port.func);
}

static inline struct f_acm *port_to_acm(struct gserial *p)
{
	return container_of(p, struct f_acm, port);
}

/*-------------------------------------------------------------------------*/

/* notification endpoint uses smallish and infrequent fixed-size messages */

#define VPU_NOTIFY_INTERVAL_MS		32
#define VPU_NOTIFY_MAXPACKET		10	/* notification + 2 bytes */

/* interface and class descriptors: */

static struct usb_interface_assoc_descriptor
vpu_iad_descriptor = {
	.bLength =		sizeof(vpu_iad_descriptor),
	.bDescriptorType =	USB_DT_INTERFACE_ASSOCIATION,

	/* .bFirstInterface =	DYNAMIC, */
	.bInterfaceCount = 2, // control + data
	.bFunctionClass =	USB_CLASS_COMM,
	.bFunctionSubClass =	USB_CDC_SUBCLASS_ACM,
	.bFunctionProtocol =	USB_CDC_ACM_PROTO_AT_V25TER,
	/* .iFunction =		DYNAMIC */
};


static struct usb_interface_descriptor vpu_control_interface_desc = {
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	/* .bInterfaceNumber = DYNAMIC */
	.bNumEndpoints =	1,
	.bInterfaceClass =	USB_CLASS_COMM,
	.bInterfaceSubClass =	USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol =	USB_CDC_ACM_PROTO_AT_V25TER,
	/* .iInterface = DYNAMIC */
};

static struct usb_interface_descriptor vpu_data_interface_desc = {
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	/* .bInterfaceNumber = DYNAMIC */
	.bNumEndpoints =	2,
	.bInterfaceClass =	USB_CLASS_CDC_DATA,
	.bInterfaceSubClass =	0,
	.bInterfaceProtocol =	0,
	/* .iInterface = DYNAMIC */
};

static struct usb_cdc_header_desc vpu_header_desc = {
	.bLength =		sizeof(vpu_header_desc),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_HEADER_TYPE,
	.bcdCDC =		cpu_to_le16(0x0110),
};

static struct usb_cdc_call_mgmt_descriptor
vpu_call_mgmt_descriptor = {
	.bLength =		sizeof(vpu_call_mgmt_descriptor),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_CALL_MANAGEMENT_TYPE,
	.bmCapabilities =	0,
	/* .bDataInterface = DYNAMIC */
};

static struct usb_cdc_acm_descriptor vpu_descriptor = {
	.bLength =		sizeof(vpu_descriptor),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_ACM_TYPE,
	.bmCapabilities =	USB_CDC_CAP_LINE,
};

static struct usb_cdc_union_desc vpu_union_desc = {
	.bLength =		sizeof(vpu_union_desc),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_UNION_TYPE,
	/* .bMasterInterface0 =	DYNAMIC */
	/* .bSlaveInterface0 =	DYNAMIC */
};

/* full speed support: */

static struct usb_endpoint_descriptor vpu_fs_notify_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	cpu_to_le16(VPU_NOTIFY_MAXPACKET),
	.bInterval =		VPU_NOTIFY_INTERVAL_MS,
};

static struct usb_endpoint_descriptor vpu_fs_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor vpu_fs_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *vpu_fs_function[] = {
	(struct usb_descriptor_header *) &vpu_iad_descriptor,
	(struct usb_descriptor_header *) &vpu_control_interface_desc,
	(struct usb_descriptor_header *) &vpu_header_desc,
	(struct usb_descriptor_header *) &vpu_call_mgmt_descriptor,
	(struct usb_descriptor_header *) &vpu_descriptor,
	(struct usb_descriptor_header *) &vpu_union_desc,
	(struct usb_descriptor_header *) &vpu_fs_notify_desc,
	(struct usb_descriptor_header *) &vpu_data_interface_desc,
	(struct usb_descriptor_header *) &vpu_fs_in_desc,
	(struct usb_descriptor_header *) &vpu_fs_out_desc,
	NULL,
};

/* high speed support: */
static struct usb_endpoint_descriptor vpu_hs_notify_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	cpu_to_le16(VPU_NOTIFY_MAXPACKET),
	.bInterval =		USB_MS_TO_HS_INTERVAL(VPU_NOTIFY_INTERVAL_MS),
};

static struct usb_endpoint_descriptor vpu_hs_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_endpoint_descriptor vpu_hs_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_descriptor_header *vpu_hs_function[] = {
	(struct usb_descriptor_header *) &vpu_iad_descriptor,
	(struct usb_descriptor_header *) &vpu_control_interface_desc,
	(struct usb_descriptor_header *) &vpu_header_desc,
	(struct usb_descriptor_header *) &vpu_call_mgmt_descriptor,
	(struct usb_descriptor_header *) &vpu_descriptor,
	(struct usb_descriptor_header *) &vpu_union_desc,
	(struct usb_descriptor_header *) &vpu_hs_notify_desc,
	(struct usb_descriptor_header *) &vpu_data_interface_desc,
	(struct usb_descriptor_header *) &vpu_hs_in_desc,
	(struct usb_descriptor_header *) &vpu_hs_out_desc,
	NULL,
};

static struct usb_endpoint_descriptor vpu_ss_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(1024),
};

static struct usb_endpoint_descriptor vpu_ss_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor vpu_ss_bulk_comp_desc = {
	.bLength =              sizeof(vpu_ss_bulk_comp_desc),
	.bDescriptorType =      USB_DT_SS_ENDPOINT_COMP,
};

static struct usb_descriptor_header *vpu_ss_function[] = {
	(struct usb_descriptor_header *) &vpu_iad_descriptor,
	(struct usb_descriptor_header *) &vpu_control_interface_desc,
	(struct usb_descriptor_header *) &vpu_header_desc,
	(struct usb_descriptor_header *) &vpu_call_mgmt_descriptor,
	(struct usb_descriptor_header *) &vpu_descriptor,
	(struct usb_descriptor_header *) &vpu_union_desc,
	(struct usb_descriptor_header *) &vpu_hs_notify_desc,
	(struct usb_descriptor_header *) &vpu_ss_bulk_comp_desc,
	(struct usb_descriptor_header *) &vpu_data_interface_desc,
	(struct usb_descriptor_header *) &vpu_ss_in_desc,
	(struct usb_descriptor_header *) &vpu_ss_bulk_comp_desc,
	(struct usb_descriptor_header *) &vpu_ss_out_desc,
	(struct usb_descriptor_header *) &vpu_ss_bulk_comp_desc,
	NULL,
};

/* string descriptors: */

#define ACM_CTRL_IDX	0
#define ACM_DATA_IDX	1
#define ACM_IAD_IDX	2

/* static strings, in UTF-8 */
static struct usb_string vpu_string_defs[] = {
	[ACM_CTRL_IDX].s = "CDC Abstract Control Model (ACM)",
	[ACM_DATA_IDX].s = "CDC ACM Data",
	[ACM_IAD_IDX].s = "CDC Serial",
	{  } /* end of list */
};

static struct usb_gadget_strings vpu_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		vpu_string_defs,
};

static struct usb_gadget_strings *vpu_strings[] = {
	&vpu_string_table,
	NULL,
};

/*-------------------------------------------------------------------------*/

/* ACM control ... data handling is delegated to tty library code.
 * The main task of this function is to activate and deactivate
 * that code based on device state; track parameters like line
 * speed, handshake state, and so on; and issue notifications.
 */

static void vpu_complete_set_line_coding(struct usb_ep *ep,
		struct usb_request *req)
{
	struct f_acm	*acm = ep->driver_data;

	if (req->status != 0)
		return;

	/* normal completion */
	if (req->actual != sizeof(acm->port_line_coding))
		usb_ep_set_halt(ep);
	else {
		struct usb_cdc_line_coding	*value = req->buf;

		/* REVISIT:  we currently just remember this data.
		 * If we change that, (a) validate it first, then
		 * (b) update whatever hardware needs updating,
		 * (c) worry about locking.  This is information on
		 * the order of 9600-8-N-1 ... most of which means
		 * nothing unless we control a real RS232 line.
		 */
		acm->port_line_coding = *value;
	}
}

u32 bus_num;
u32 dev_num;
u32 xlink_usb_sw_id;

int usb_sw_id(int pid)
{
	xlink_usb_sw_id = (XLINK_DEV_INF_USB << XLINK_DEV_INF_TYPE_SHIFT) |
		      ((bus_num << 8 | dev_num) << XLINK_DEV_PHYS_ID_SHIFT) |
			  (XLINK_DEV_TYPE_KMB << XLINK_DEV_TYPE_SHIFT) |
		      (XLINK_DEV_SLICE_0 << XLINK_DEV_SLICE_ID_SHIFT) |
		      (XLINK_DEV_FUNC_VPU << XLINK_DEV_FUNC_SHIFT);
	return xlink_usb_sw_id;
}
EXPORT_SYMBOL_GPL(usb_sw_id);

int usb_get_by_id(uint32_t sw_device_id)
{
	return (sw_device_id == xlink_usb_sw_id);
}
EXPORT_SYMBOL_GPL(usb_get_by_id);

int mxlk_trigger_reset(void *arg)
{
	orderly_reboot();
	return 0;
}

int intr;
static irqreturn_t keembay_usb_irq_handler(int irq, void *arg)
{
	int *data = vpu_read_swid(data);

	if (data != 0) {
		if (*(data) == 0)
			mxlk_trigger_reset(arg);
		else {
			if (!intr) {
				bus_num = *(data);
				intr = 1;
			} else {
				dev_num = *(data);
				intr = 0;
			}
		}
	}

	return IRQ_HANDLED;
}

int vpu_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct f_acm		*acm = func_to_acm(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_request	*req = cdev->req;
	int			value = -EOPNOTSUPP;
	u16			w_index = le16_to_cpu(ctrl->wIndex);
	u16			w_value = le16_to_cpu(ctrl->wValue);
	u16			w_length = le16_to_cpu(ctrl->wLength);

	/* composite driver infrastructure handles everything except
	 * CDC class messages; interface activation uses set_alt().
	 *
	 * Note CDC spec table 4 lists the ACM request profile.  It requires
	 * encapsulated command support ... we don't handle any, and respond
	 * to them by stalling.  Options include get/set/clear comm features
	 * (not that useful) and SEND_BREAK.
	 */
	switch ((ctrl->bRequestType << 8) | ctrl->bRequest) {

	/* SET_LINE_CODING ... just read and save what the host sends */
	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_REQ_SET_LINE_CODING:
		if (w_length != sizeof(struct usb_cdc_line_coding)
				|| w_index != acm->ctrl_id)
			goto invalid;

		value = w_length;
		cdev->gadget->ep0->driver_data = acm;
		req->complete = vpu_complete_set_line_coding;
		break;

	/* GET_LINE_CODING ... return what host sent, or initial value */
	case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_REQ_GET_LINE_CODING:

		if (w_index != acm->ctrl_id)
			goto invalid;

		value = min_t(unsigned int, w_length,
				sizeof(struct usb_cdc_line_coding));

		memcpy(req->buf, &acm->port_line_coding, value);

		break;

	/* SET_CONTROL_LINE_STATE ... save what the host sent */
	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_REQ_SET_CONTROL_LINE_STATE:
		if (w_index != acm->ctrl_id)
			goto invalid;

		value = 0;

		/* FIXME we should not allow data to flow until the
		 * host sets the ACM_CTRL_DTR bit; and when it clears
		 * that bit, we should return to that no-flow state.
		 */
		acm->port_handshake_bits = w_value;
		break;

	default:
invalid:
		dev_vdbg(&cdev->gadget->dev,
			 "invalid control req%02x.%02x v%04x i%04x l%d\n",
			 ctrl->bRequestType, ctrl->bRequest,
			 w_value, w_index, w_length);
	}

	/* respond with data transfer or status phase? */
	if (value >= 0) {
		req->zero = 0;
		req->length = value;
		value = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if (value < 0)
			ERROR(cdev, "acm response err %d\n",
					value);
	}

	/* device either stalls (value < 0) or reports success */
	return value;
}
EXPORT_SYMBOL_GPL(vpu_setup);

static int vpu_set_alt(struct usb_function *f, unsigned int intf,
	unsigned int alt)
{
	struct f_acm		*acm = func_to_acm(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	/* we know alt == 0, so this is an activation or a reset */

	if (intf == acm->ctrl_id) {
		dev_vdbg(&cdev->gadget->dev,
				"reset acm control interface %d\n", intf);
		usb_ep_disable(acm->notify);

		if (!acm->notify->desc)
			if (config_ep_by_speed(cdev->gadget, f, acm->notify))
				return -EINVAL;

		usb_ep_enable(acm->notify);

	} else if (intf == acm->data_id) {
		if (acm->notify->enabled)
			vpu_disconnect(&acm->port);

		if (!acm->port.in->desc || !acm->port.out->desc) {
			if (config_ep_by_speed(cdev->gadget, f,
					       acm->port.in) ||
			    config_ep_by_speed(cdev->gadget, f,
					       acm->port.out)) {
				acm->port.in->desc = NULL;
				acm->port.out->desc = NULL;
				return -EINVAL;
			}
		}

		vpu_connect(&acm->port, acm->port_num);

	} else
		return -EINVAL;
	return 0;
}

static void vpu_disable(struct usb_function *f)
{
	struct f_acm	*acm = func_to_acm(f);

	vpu_disconnect(&acm->port);
	usb_ep_disable(acm->notify);
}

/*-------------------------------------------------------------------------*/

/**
 * vpu_cdc_notify - issue CDC notification to host
 * @acm: wraps host to be notified
 * @type: notification type
 * @value: Refer to cdc specs, wValue field.
 * @data: data to be sent
 * @length: size of data
 * Context: irqs blocked, acm->lock held, vpu_notify_req non-null
 *
 * Returns zero on success or a negative errno.
 *
 * See section 6.3.5 of the CDC 1.1 specification for information
 * about the only notification we issue:  SerialState change.
 */
int vpu_cdc_notify(struct f_acm *acm, u8 type, u16 value,
		void *data, unsigned int length)
{
	struct usb_ep			*ep = acm->notify;
	struct usb_request		*req;
	struct usb_cdc_notification	*notify;
	const unsigned int			len = sizeof(*notify) + length;
	void				*buf;
	int				status;

	req = acm->notify_req;
	acm->notify_req = NULL;
	acm->pending = false;
	req->length = len;
	notify = req->buf;
	buf = notify + 1;

	notify->bmRequestType = USB_DIR_IN | USB_TYPE_CLASS
			| USB_RECIP_INTERFACE;
	notify->bNotificationType = type;
	notify->wValue = cpu_to_le16(value);
	notify->wIndex = cpu_to_le16(acm->ctrl_id);
	notify->wLength = cpu_to_le16(length);
	memcpy(buf, data, length);

	/* ep_queue() can complete immediately if it fills the fifo... */
	spin_unlock(&acm->lock);
	status = usb_ep_queue(ep, req, GFP_ATOMIC);
	spin_lock(&acm->lock);

	if (status < 0)
		acm->notify_req = req;

	return status;
}
EXPORT_SYMBOL_GPL(vpu_cdc_notify);

int vpu_notify_serial_state(struct f_acm *acm)
{
	int			status;
	__le16			serial_state;

	spin_lock(&acm->lock);
	if (acm->notify_req) {
		serial_state = cpu_to_le16(acm->serial_state);
		status = vpu_cdc_notify(acm, USB_CDC_NOTIFY_SERIAL_STATE,
				0, &serial_state, sizeof(acm->serial_state));
	} else {
		acm->pending = true;
		status = 0;
	}
	spin_unlock(&acm->lock);
	return status;
}
EXPORT_SYMBOL_GPL(vpu_notify_serial_state);

static void vpu_cdc_notify_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct f_acm		*acm = req->context;
	u8			doit = false;
	/* on this call path we do NOT hold the port spinlock,
	 * which is why ACM needs its own spinlock
	 */
	spin_lock(&acm->lock);
	if (req->status != -ESHUTDOWN)
		doit = acm->pending;
	acm->notify_req = req;
	spin_unlock(&acm->lock);

	if (doit)
		vpu_notify_serial_state(acm);
}

static void vpu_acm_connect(struct gserial *port)
{
	struct f_acm		*acm = port_to_acm(port);

	acm->serial_state |= ACM_CTRL_DSR | ACM_CTRL_DCD;
	vpu_notify_serial_state(acm);
}

static void vpu_acm_disconnect(struct gserial *port)
{
	struct f_acm		*acm = port_to_acm(port);

	acm->serial_state &= ~(ACM_CTRL_DSR | ACM_CTRL_DCD);
	vpu_notify_serial_state(acm);
}

static int vpu_send_break(struct gserial *port, int duration)
{
	struct f_acm		*acm = port_to_acm(port);
	u16			state;

	state = acm->serial_state;
	state &= ~ACM_CTRL_BRK;
	if (duration)
		state |= ACM_CTRL_BRK;

	acm->serial_state = state;
	return vpu_notify_serial_state(acm);
}

/*-------------------------------------------------------------------------*/

/* ACM function driver setup/binding */
static int
vpu_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct f_acm		*acm = func_to_acm(f);
	struct usb_string	*us;
	int			status;
	struct usb_ep		*ep;

	struct usb_gadget *gadget = cdev->gadget;
	struct device *dev = &gadget->dev;
	/* REVISIT might want instance-specific strings to help
	 * distinguish instances ...
	 */

	int err = devm_request_irq(dev, 47, keembay_usb_irq_handler,
			       IRQF_SHARED | IRQF_NO_THREAD, "xlink-usb", cdev);
	if (err)
		dev_err(dev, "failed to request IRQ: %d\n", err);


	/* maybe allocate device-global string IDs, and patch descriptors */
	us = usb_gstrings_attach(cdev, vpu_strings,
			ARRAY_SIZE(vpu_string_defs));
	if (IS_ERR(us))
		return PTR_ERR(us);
	vpu_control_interface_desc.iInterface = us[ACM_CTRL_IDX].id;
	vpu_data_interface_desc.iInterface = us[ACM_DATA_IDX].id;
	vpu_iad_descriptor.iFunction = us[ACM_IAD_IDX].id;

	/* allocate instance-specific interface IDs, and patch descriptors */
	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	acm->ctrl_id = status;
	vpu_iad_descriptor.bFirstInterface = status;

	vpu_control_interface_desc.bInterfaceNumber = status;
	vpu_union_desc .bMasterInterface0 = status;

	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	acm->data_id = status;

	vpu_data_interface_desc.bInterfaceNumber = status;
	vpu_union_desc.bSlaveInterface0 = status;
	vpu_call_mgmt_descriptor.bDataInterface = status;

	status = -ENODEV;

	/* allocate instance-specific endpoints */
	ep = usb_ep_autoconfig(cdev->gadget, &vpu_fs_in_desc);
	if (!ep)
		goto fail;
	acm->port.in = ep;

	ep = usb_ep_autoconfig(cdev->gadget, &vpu_fs_out_desc);
	if (!ep)
		goto fail;
	acm->port.out = ep;

	ep = usb_ep_autoconfig(cdev->gadget, &vpu_fs_notify_desc);
	if (!ep)
		goto fail;
	acm->notify = ep;

	/* allocate notification */
	acm->notify_req = vpu_alloc_req(ep,
			sizeof(struct usb_cdc_notification) + 2,
			GFP_KERNEL);
	if (!acm->notify_req)
		goto fail;

	acm->notify_req->complete = vpu_cdc_notify_complete;
	acm->notify_req->context = acm;

	/* support all relevant hardware speeds... we expect that when
	 * hardware is dual speed, all bulk-capable endpoints work at
	 * both speeds
	 */
	vpu_hs_in_desc.bEndpointAddress = vpu_fs_in_desc.bEndpointAddress;
	vpu_hs_out_desc.bEndpointAddress = vpu_fs_out_desc.bEndpointAddress;
	vpu_hs_notify_desc.bEndpointAddress =
		vpu_fs_notify_desc.bEndpointAddress;

	vpu_ss_in_desc.bEndpointAddress = vpu_fs_in_desc.bEndpointAddress;
	vpu_ss_out_desc.bEndpointAddress = vpu_fs_out_desc.bEndpointAddress;

	status = usb_assign_descriptors(f, vpu_fs_function, vpu_hs_function,
			vpu_ss_function, NULL);
	if (status)
		goto fail;
	kfree(us);
	return 0;

fail:
	if (acm->notify_req)
		vpu_free_req(acm->notify, acm->notify_req);

	ERROR(cdev, "%s/%p: can't bind, err %d\n", f->name, f, status);
	kfree(us);
	return status;
}

static void vpu_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct f_acm		*acm = func_to_acm(f);

	vpu_string_defs[0].id = 0;
	usb_free_all_descriptors(f);
	if (acm->notify_req)
		vpu_free_req(acm->notify, acm->notify_req);
}

static void vpu_free_func(struct usb_function *f)
{
	struct f_acm		*acm = func_to_acm(f);

	kfree(acm);
}

static struct usb_function *vpu_alloc_func(struct usb_function_instance *fi)
{
	struct f_serial_opts *opts;
	struct f_acm *acm;

	acm = kzalloc(sizeof(*acm), GFP_KERNEL);
	if (!acm)
		return ERR_PTR(-ENOMEM);

	spin_lock_init(&acm->lock);

	acm->port.connect = vpu_acm_connect;
	acm->port.disconnect = vpu_acm_disconnect;
	acm->port.send_break = vpu_send_break;

	acm->port.func.name = "vpu";
	acm->port.func.strings = vpu_strings;
	/* descriptors are per-instance copies */
	acm->port.func.bind = vpu_bind;
	acm->port.func.set_alt = vpu_set_alt;
	acm->port.func.setup = vpu_setup;
	acm->port.func.disable = vpu_disable;

	opts = container_of(fi, struct f_serial_opts, func_inst);
	acm->port_num = opts->port_num;
	acm->port.func.unbind = vpu_unbind;
	acm->port.func.free_func = vpu_free_func;
	return &acm->port.func;
}

static inline struct f_serial_opts *to_f_serial_opts(struct config_item *item)
{
	return container_of(to_config_group(item), struct f_serial_opts,
			func_inst.group);
}

static void vpu_attr_release(struct config_item *item)
{
	struct f_serial_opts *opts = to_f_serial_opts(item);

	usb_put_function_instance(&opts->func_inst);
}

static struct configfs_item_operations vpu_item_ops = {
	.release                = vpu_attr_release,
};

static ssize_t f_acm_port_num_show(struct config_item *item, char *page)
{
	return 0;
}

CONFIGFS_ATTR_RO(f_acm_, port_num);

static struct configfs_attribute *vpu_attrs[] = {
	&f_acm_attr_port_num,
	NULL,
};

static const struct config_item_type vpu_func_type = {
	.ct_item_ops    = &vpu_item_ops,
	.ct_attrs	= vpu_attrs,
	.ct_owner       = THIS_MODULE,
};

static void vpu_free_instance(struct usb_function_instance *fi)
{
	struct f_serial_opts *opts;

	opts = container_of(fi, struct f_serial_opts, func_inst);
	vpu_free_line(opts->port_num);
	kfree(opts);
}

static struct usb_function_instance *vpu_alloc_instance(void)
{
	struct f_serial_opts *opts;
	int ret;

	opts = kzalloc(sizeof(*opts), GFP_KERNEL);
	if (!opts)
		return ERR_PTR(-ENOMEM);
	opts->func_inst.free_func_inst = vpu_free_instance;
	ret = vpu_alloc_line(&opts->port_num);
	if (ret) {
		kfree(opts);
		return ERR_PTR(ret);
	}
	config_group_init_type_name(&opts->func_inst.group, "",
			&vpu_func_type);
	return &opts->func_inst;
}
DECLARE_USB_FUNCTION_INIT(vpu, vpu_alloc_instance, vpu_alloc_func);
MODULE_LICENSE("GPL");
