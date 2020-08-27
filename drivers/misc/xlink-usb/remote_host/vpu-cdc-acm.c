// SPDX-License-Identifier: GPL-2.0-only
/*
 * vpu-cdc-acm.c - XLink USB Remote Host driver
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

#undef DEBUG
#undef VERBOSE_DEBUG

#include <linux/kernel.h>
#include <linux/sched/signal.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/log2.h>
#include <linux/serial.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/usb/cdc.h>
#include <asm/byteorder.h>
#include <asm/unaligned.h>
#include <linux/idr.h>
#include <linux/list.h>
#include <linux/completion.h>

#include "vpu-cdc-acm.h"
#include <linux/xlink_drv_inf.h>

#define DRIVER_AUTHOR "Tan Seng Kai, Lai Jun Ann"
#define DRIVER_DESC "XLink USB Remote Host driver"

static struct usb_driver vpu_acm_driver;

static DEFINE_IDR(vpu_acm_minors);
static DEFINE_MUTEX(vpu_acm_minors_lock);
#define MXLK_MAX_NAME_LEN (32)
static LIST_HEAD(dev_list);
static DEFINE_MUTEX(dev_list_mutex);

struct acm {
	struct usb_device *dev;
	struct usb_interface *control;
	struct usb_interface *data;
	unsigned int in, out;
	struct urb *ctrlurb;
	u8 *ctrl_buffer;
	dma_addr_t ctrl_dma;
	u8 *country_codes;
	unsigned int country_code_size;
	unsigned int country_rel_date;
	struct acm_wb wb[ACM_NW];
	unsigned long read_urbs_free;
	struct urb *read_urbs[ACM_NR];
	struct acm_rb read_buffers[ACM_NR];
	int rx_buflimit;
	spinlock_t read_lock;
	u8 *notification_buffer;
	unsigned int nb_index;
	unsigned int nb_size;
	int transmitting;
	spinlock_t write_lock;
	struct mutex mutex;
	bool disconnected;
	unsigned long flags;
#		define EVENT_TTY_WAKEUP	0
#		define EVENT_RX_STALL	1
#		define ACM_THROTTLED	2
	struct usb_cdc_line_coding line;
	struct work_struct work;
	unsigned int ctrlin;
	unsigned int ctrlout;
	struct async_icount iocount;
	struct async_icount oldcount;
	wait_queue_head_t wioctl;
	unsigned int writesize;
	unsigned int readsize, ctrlsize;
	unsigned int minor;
	unsigned char clocal;
	unsigned int ctrl_caps;
	unsigned int susp_count;
	unsigned int combined_interfaces:1;
	u8 bInterval;
	struct usb_anchor delayed;
	unsigned long quirks;
	char name[MXLK_MAX_NAME_LEN];
	u32 devid;
	struct list_head list;
	u32 status;
	const char *buf;
};
/*
 * vpu_acm_minors accessors
 */

/*
 * Look up an ACM structure by minor. If found and not disconnected, increment
 * its refcount and return it with its mutex held.
 */
static struct acm *vpu_acm_get_by_minor(unsigned int minor)
{
	struct acm *acm;

	mutex_lock(&vpu_acm_minors_lock);
	acm = idr_find(&vpu_acm_minors, minor);
	if (acm) {
		mutex_lock(&acm->mutex);
		if (acm->disconnected)
			mutex_unlock(&acm->mutex);
		else
			mutex_unlock(&acm->mutex);
	}
	mutex_unlock(&vpu_acm_minors_lock);
	return acm;
}

/*
 * Try to find an available minor number and if found, associate it with 'acm'.
 */
static int vpu_acm_alloc_minor(struct acm *acm)
{
	int minor;

	mutex_lock(&vpu_acm_minors_lock);
	minor = idr_alloc(&vpu_acm_minors, acm, 0, ACM_TTY_MINORS, GFP_KERNEL);
	mutex_unlock(&vpu_acm_minors_lock);

	return minor;
}

/*
 * Functions for ACM control messages.
 */

static int vpu_acm_ctrl_msg(struct acm *acm, int request, int value,
							void *buf, int len)
{
	int retval;

	retval = usb_autopm_get_interface(acm->control);
	if (retval)
		return retval;

	retval = usb_control_msg(acm->dev, usb_sndctrlpipe(acm->dev, 0),
		request, USB_RT_ACM, value,
		acm->control->altsetting[0].desc.bInterfaceNumber,
		buf, len, 5000);

	usb_autopm_put_interface(acm->control);

	return retval < 0 ? retval : 0;
}

/* devices aren't required to support these requests.
 * the cdc acm descriptor tells whether they do...
 */
static inline int vpu_acm_set_control(struct acm *acm, int control)
{
	if (acm->quirks & QUIRK_CONTROL_LINE_STATE)
		return -EOPNOTSUPP;

	return vpu_acm_ctrl_msg(acm, USB_CDC_REQ_SET_CONTROL_LINE_STATE,
			control, NULL, 0);
}

#define vpu_acm_set_line(acm, line) \
	vpu_acm_ctrl_msg(acm, USB_CDC_REQ_SET_LINE_CODING, \
			0, line, sizeof *(line))
#define vpu_acm_send_break(acm, ms) \
	vpu_acm_ctrl_msg(acm, USB_CDC_REQ_SEND_BREAK, ms, NULL, 0)

static void vpu_acm_kill_urbs(struct acm *acm)
{
	int i;

	usb_kill_urb(acm->ctrlurb);
	for (i = 0; i < ACM_NW; i++)
		usb_kill_urb(acm->wb[i].urb);
	for (i = 0; i < acm->rx_buflimit; i++)
		usb_kill_urb(acm->read_urbs[i]);
}

/*
 * Write buffer management.
 * All of these assume proper locks taken by the caller.
 */

static int vpu_acm_wb_alloc(struct acm *acm)
{
	int i, wbn;
	struct acm_wb *wb;

	wbn = 0;
	i = 0;
	for (;;) {
		wb = &acm->wb[wbn];
		if (!wb->use) {
			wb->use = 1;
			wb->len = 0;
			return wbn;
		}
		wbn = (wbn + 1) % ACM_NW;
		if (++i >= ACM_NW)
			return -1;
	}
}


/*
 * Finish write. Caller must hold acm->write_lock
 */
static void vpu_acm_write_done(struct acm *acm, struct acm_wb *wb)
{
	wb->use = 0;
	acm->transmitting--;
	usb_autopm_put_interface_async(acm->control);
}

/*
 * Poke write.
 *
 * the caller is responsible for locking
 */

static int vpu_acm_start_wb(struct acm *acm, struct acm_wb *wb)
{
	int rc;

	acm->transmitting++;
	wb->urb->transfer_buffer = wb->buf;
	wb->urb->transfer_dma = wb->dmah;
	wb->urb->transfer_buffer_length = wb->len;
	wb->urb->dev = acm->dev;

	rc = usb_submit_urb(wb->urb, GFP_ATOMIC);
	if (rc < 0) {
		dev_err(&acm->data->dev,
			"%s - usb_submit_urb(write bulk) failed: %d\n",
			__func__, rc);
		vpu_acm_write_done(acm, wb);
	}
	return rc;
}

/*
 * attributes exported through sysfs
 */
static ssize_t bmCapabilities_show
(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 0;
}
static DEVICE_ATTR_RO(bmCapabilities);

static ssize_t wCountryCodes_show
(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct acm *acm = usb_get_intfdata(intf);

	memcpy(buf, acm->country_codes, acm->country_code_size);
	return acm->country_code_size;
}

static DEVICE_ATTR_RO(wCountryCodes);

static ssize_t iCountryCodeRelDate_show
(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 0;
}

static DEVICE_ATTR_RO(iCountryCodeRelDate);
/*
 * Interrupt handlers for various ACM device responses
 */

static void vpu_acm_process_notification(struct acm *acm, unsigned char *buf)
{
	int newctrl;
	int difference;
	unsigned long flags;
	struct usb_cdc_notification *dr = (struct usb_cdc_notification *)buf;
	unsigned char *data = buf + sizeof(struct usb_cdc_notification);

	switch (dr->bNotificationType) {
	case USB_CDC_NOTIFY_NETWORK_CONNECTION:
		break;

	case USB_CDC_NOTIFY_SERIAL_STATE:
		if (le16_to_cpu(dr->wLength) != 2)
			break;

		newctrl = get_unaligned_le16(data);

		difference = acm->ctrlin ^ newctrl;
		spin_lock_irqsave(&acm->read_lock, flags);
		acm->ctrlin = newctrl;
		acm->oldcount = acm->iocount;

		if (difference & ACM_CTRL_DSR)
			acm->iocount.dsr++;
		if (difference & ACM_CTRL_DCD)
			acm->iocount.dcd++;
		if (newctrl & ACM_CTRL_BRK)
			acm->iocount.brk++;
		if (newctrl & ACM_CTRL_RI)
			acm->iocount.rng++;
		if (newctrl & ACM_CTRL_FRAMING)
			acm->iocount.frame++;
		if (newctrl & ACM_CTRL_PARITY)
			acm->iocount.parity++;
		if (newctrl & ACM_CTRL_OVERRUN)
			acm->iocount.overrun++;
		spin_unlock_irqrestore(&acm->read_lock, flags);

		if (difference)
			wake_up_all(&acm->wioctl);

		break;

	default:
		break;
	}
}

/* control interface reports status changes with "interrupt" transfers */
static void vpu_acm_ctrl_irq(struct urb *urb)
{
	struct acm *acm = urb->context;
	struct usb_cdc_notification *dr = urb->transfer_buffer;
	unsigned int current_size = urb->actual_length;
	unsigned int expected_size, copy_size, alloc_size;
	int retval;
	int status = urb->status;

	switch (status) {
	case 0:
		/* success */
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		return;
	default:
		goto exit;
	}

	usb_mark_last_busy(acm->dev);

	if (acm->nb_index)
		dr = (struct usb_cdc_notification *)acm->notification_buffer;

	/* size = notification-header + (optional) data */
	expected_size = sizeof(struct usb_cdc_notification) +
					le16_to_cpu(dr->wLength);

	if (current_size < expected_size) {
		/* notification is transmitted fragmented, reassemble */
		if (acm->nb_size < expected_size) {
			if (acm->nb_size) {
				kfree(acm->notification_buffer);
				acm->nb_size = 0;
			}
			alloc_size = roundup_pow_of_two(expected_size);
			/*
			 * kmalloc ensures a valid notification_buffer after a
			 * use of kfree in case the previous allocation was too
			 * small. Final freeing is done on disconnect.
			 */
			acm->notification_buffer =
				kmalloc(alloc_size, GFP_ATOMIC);
			if (!acm->notification_buffer)
				goto exit;
			acm->nb_size = alloc_size;
		}

		copy_size = min(current_size,
				expected_size - acm->nb_index);
		memcpy(&acm->notification_buffer[acm->nb_index],
		       urb->transfer_buffer, copy_size);
		acm->nb_index += copy_size;
		current_size = acm->nb_index;
	}

	else {
	/* notification complete */
		vpu_acm_process_notification(acm, (unsigned char *)dr);
		acm->nb_index = 0;
	}

exit:
	retval = usb_submit_urb(urb, GFP_ATOMIC);
	if (retval && retval != -EPERM)
		dev_err(&acm->control->dev,
			"%s - usb_submit_urb failed: %d\n", __func__, retval);
}


static int vpu_acm_submit_read_urb(struct acm *acm, int index, gfp_t mem_flags)
{
	int res;

	if (!test_and_clear_bit(index, &acm->read_urbs_free))
		return 0;

	res = usb_submit_urb(acm->read_urbs[index], mem_flags);
	if (res) {
		if (res != -EPERM && res != -ENODEV) {
			dev_err(&acm->data->dev,
				"urb %d failed submission with %d\n",
				index, res);
		}
		set_bit(index, &acm->read_urbs_free);
		return res;
	}

	return 0;
}

static int vpu_acm_submit_read_urbs(struct acm *acm, gfp_t mem_flags)
{
	int res;
	int i;

	for (i = 0; i < acm->rx_buflimit; ++i) {
		res = vpu_acm_submit_read_urb(acm, i, mem_flags);
		if (res)
			return res;
	}

	return 0;
}

static void vpu_acm_process_read_urb(struct acm *acm, struct urb *urb)
{
	if (!urb->actual_length)
		return;
}

static void vpu_acm_read_bulk_callback(struct urb *urb)
{
	struct acm_rb *rb = urb->context;
	struct acm *acm = rb->instance;
	int status = urb->status;
	bool stopped = false;
	bool stalled = false;

	dev_vdbg(&acm->data->dev, "got urb %d, len %d, status %d\n",
		rb->index, urb->actual_length, status);

	if (!acm->dev)
		return;

	switch (status) {
	case 0:
		usb_mark_last_busy(acm->dev);
		vpu_acm_process_read_urb(acm, urb);
		break;
	case -EPIPE:
		set_bit(EVENT_RX_STALL, &acm->flags);
		stalled = true;
		break;
	case -ENOENT:
	case -ECONNRESET:
	case -ESHUTDOWN:
		stopped = true;
		break;
	default:
		break;
	}

	/*
	 * Make sure URB processing is done before marking as free to avoid
	 * racing with unthrottle() on another CPU. Matches the barriers
	 * implied by the test_and_clear_bit() in vpu_acm_submit_read_urb().
	 */
	smp_mb__before_atomic();
	set_bit(rb->index, &acm->read_urbs_free);
	/*
	 * Make sure URB is marked as free before checking the throttled flag
	 * to avoid racing with unthrottle() on another CPU. Matches the
	 * smp_mb() in unthrottle().
	 */
	smp_mb__after_atomic();

	if (stopped || stalled) {
		if (stalled)
			schedule_work(&acm->work);
		return;
	}

	if (test_bit(ACM_THROTTLED, &acm->flags))
		return;

	vpu_acm_submit_read_urb(acm, rb->index, GFP_ATOMIC);
}

/* data interface wrote those outgoing bytes */
static void vpu_acm_write_bulk(struct urb *urb)
{
	struct acm_wb *wb = urb->context;
	struct acm *acm = wb->instance;
	unsigned long flags;
	int status = urb->status;

	if (status || (urb->actual_length != urb->transfer_buffer_length))
		dev_vdbg(&acm->data->dev, "wrote len %d/%d, status %d\n",
			urb->actual_length,
			urb->transfer_buffer_length,
			status);

	spin_lock_irqsave(&acm->write_lock, flags);
	vpu_acm_write_done(acm, wb);
	spin_unlock_irqrestore(&acm->write_lock, flags);
	set_bit(EVENT_TTY_WAKEUP, &acm->flags);
	schedule_work(&acm->work);
}

static void vpu_acm_softint(struct work_struct *work)
{
	int i;
	struct acm *acm = container_of(work, struct acm, work);

	if (test_bit(EVENT_RX_STALL, &acm->flags)) {
		if (!(usb_autopm_get_interface(acm->data))) {
			for (i = 0; i < acm->rx_buflimit; i++)
				usb_kill_urb(acm->read_urbs[i]);
			usb_clear_halt(acm->dev, acm->in);
			vpu_acm_submit_read_urbs(acm, GFP_KERNEL);
			usb_autopm_put_interface(acm->data);
		}
		clear_bit(EVENT_RX_STALL, &acm->flags);
	}

}

int usb_get_device_num(uint32_t *id_list)
{
	int num = 0;
	struct acm *p;

	mutex_lock(&dev_list_mutex);
	list_for_each_entry(p, &dev_list, list) {
		*id_list++ = p->devid;
		num++;
	}
	mutex_unlock(&dev_list_mutex);
	return num;
}

static struct acm *usb_get_device_by_id(u32 id)
{
	struct acm *xdev;

	mutex_lock(&dev_list_mutex);
	list_for_each_entry(xdev, &dev_list, list) {
		if (xdev->devid == id)
			break;
	}
	mutex_unlock(&dev_list_mutex);

	return xdev;
}

int usb_get_device_name_by_id(uint32_t id, char *device_name, size_t name_size)
{
	struct acm *xdev;
	size_t size;

	xdev = usb_get_device_by_id(id);
	if (!xdev)
		return -ENODEV;

	size = (name_size > MXLK_MAX_NAME_LEN) ? MXLK_MAX_NAME_LEN : name_size;
	strncpy(device_name, xdev->name, size);
	return 0;
}

static struct acm *usb_get_device_by_name(const char *name)
{
	struct acm *p;

	mutex_lock(&dev_list_mutex);
	list_for_each_entry(p, &dev_list, list) {
		if (!strncmp(p->name, name, MXLK_MAX_NAME_LEN))
			break;
	}
	mutex_unlock(&dev_list_mutex);

	return p;
}

int usb_get_device_status_by_id(u32 id, uint32_t *status)
{
	struct acm *xdev = usb_get_device_by_id(id);

	if (!xdev)
		return -ENODEV;
	u32 num = 0;

	mutex_lock(&dev_list_mutex);
	list_for_each_entry(xdev, &dev_list, list) {
		*status++ = xdev->status;
		num++;
	}
	mutex_unlock(&dev_list_mutex);
	return 0;
}

struct api_context {
	struct completion	done;
	int			status;
};

int usb_interrupt(struct usb_device *usb_dev, unsigned int pipe,
		      void *data, int len, int *actual_length, int timeout)
{
	struct urb *urb;
	struct usb_host_endpoint *ep;

	ep = usb_pipe_endpoint(usb_dev, pipe);
	urb = usb_alloc_urb(0, GFP_KERNEL);
	return 0;
}

int usb_connect_device(uint32_t sw_device_id)
{
	struct acm *xdev = usb_get_device_by_id(sw_device_id);

	if (!xdev)
		return -ENODEV;
	return 0;
}

int usb_device_reset(u8 cmd)
{
	int ret;
	int index = 0;
	unsigned int pipe = 1;
	struct acm *acm = vpu_acm_get_by_minor(index);
	int actual_len;
	struct usb_host_endpoint *ep;
	u8 *buffer_cmd = NULL;

	ep = usb_pipe_endpoint(acm->dev, pipe);
	struct usb_interface *data_interface = acm->data;
	struct usb_endpoint_descriptor *epwrite = NULL;

	epwrite = &data_interface->cur_altsetting->endpoint[1].desc;
	buffer_cmd = kzalloc(sizeof(cmd), GFP_KERNEL);
	if (!buffer_cmd)
		return -ENOMEM;

	*buffer_cmd = cmd;

	pipe = (pipe & ~(3 << 30)) | (PIPE_INTERRUPT << 30);

	ret = usb_interrupt(acm->dev,
		usb_sndintpipe(acm->dev, epwrite->bEndpointAddress),
		buffer_cmd, sizeof(cmd),
		&actual_len, USB_CTRL_SET_TIMEOUT);
	kfree(buffer_cmd);

	return 0;
}
EXPORT_SYMBOL_GPL(usb_device_reset);

int vpu_write_host(u32 id, const char *buf, size_t *size,
				unsigned int timeout_ms)
{
	int len = *size;
	int ret = 0;
	struct acm *xdev = usb_get_device_by_id(id);
	struct acm *acm;
	int index = 0;
	long jiffies_passed = 0;
	long jiffies_timeout = (long)msecs_to_jiffies(timeout_ms);
	int retval = -ENODEV;
	int i;

	if (!xdev)
		return -EINVAL;

	acm = vpu_acm_get_by_minor(index);
	if (!acm)
		return -EINVAL;

	ret = wait_event_interruptible_timeout(acm->wioctl,
					acm, jiffies_timeout - jiffies_passed);
	if (ret == 0)
		return -ETIME;

	*size = 0;
	if (len == 0)
		return -EINVAL;

	mutex_lock(&acm->mutex);
	if (acm->disconnected)
		goto disconnected;

	retval = usb_autopm_get_interface(acm->control);
	if (retval)
		goto error_get_interface;
	/*
	 * FIXME: Why do we need this? Allocating 64K of physically contiguous
	 * memory is really nasty...
	 */
	acm->control->needs_remote_wakeup = 1;

	acm->ctrlurb->dev = acm->dev;
	retval = usb_submit_urb(acm->ctrlurb, GFP_KERNEL);
	if (retval) {
		dev_err(&acm->control->dev,
			"%s - usb_submit_urb(ctrl irq) failed\n", __func__);
		goto error_submit_urb;
	}

	/*
	 * Unthrottle device in case the TTY was closed while throttled.
	 */
	clear_bit(ACM_THROTTLED, &acm->flags);

	retval = vpu_acm_submit_read_urbs(acm, GFP_KERNEL);
	if (retval)
		goto error_submit_read_urbs;

	usb_autopm_put_interface(acm->control);

	mutex_unlock(&acm->mutex);

	vpu_acm_submit_read_urbs(acm, GFP_KERNEL);

	int val = ACM_CTRL_DTR | ACM_CTRL_RTS;

	acm->ctrlout = val;
	vpu_acm_set_control(acm, val);

	int count = len;
	int stat;
	unsigned long flags;
	int wbn;
	struct acm_wb *wb;

	if (!count)
		return -EINVAL;

	dev_vdbg(&acm->data->dev, "%d bytes from tty layer\n", count);

	spin_lock_irqsave(&acm->write_lock, flags);
	wbn = vpu_acm_wb_alloc(acm);
	if (wbn < 0) {
		spin_unlock_irqrestore(&acm->write_lock, flags);
		return -EINVAL;
	}
	wb = &acm->wb[wbn];

	if (!acm->dev) {
		wb->use = 0;
		spin_unlock_irqrestore(&acm->write_lock, flags);
		return -EINVAL;
	}

	count = (count > acm->writesize) ? acm->writesize : count;
	dev_vdbg(&acm->data->dev, "writing %d bytes\n", count);
	memcpy(wb->buf, buf, count);
	wb->len = count;
	len = count;
	stat = usb_autopm_get_interface_async(acm->control);
	if (stat) {
		wb->use = 0;
		spin_unlock_irqrestore(&acm->write_lock, flags);
		return -EINVAL;
	}

	if (acm->susp_count) {
		usb_anchor_urb(wb->urb, &acm->delayed);
		spin_unlock_irqrestore(&acm->write_lock, flags);
		return -EINVAL;
	}
	stat = vpu_acm_start_wb(acm, wb);
	spin_unlock_irqrestore(&acm->write_lock, flags);

	if (stat < 0)
		return -EINVAL;

error_submit_read_urbs:
	for (i = 0; i < acm->rx_buflimit; i++)
		usb_kill_urb(acm->read_urbs[i]);
	usb_kill_urb(acm->ctrlurb);
error_submit_urb:
	usb_autopm_put_interface(acm->control);
error_get_interface:
disconnected:
	mutex_unlock(&acm->mutex);
	*size = len;
	return 0;
}
EXPORT_SYMBOL_GPL(vpu_write_host);

int vpu_write_reset_host(u32 id)
{
	struct acm *xdev = usb_get_device_by_id(id);
	struct acm *acm;
	int index = 0;
	int count = 8;
	int stat;
	int i;
	unsigned long flags;
	int wbn;
	struct acm_wb *wb;

	acm = vpu_acm_get_by_minor(index);
	if (!xdev)
		return -EINVAL;
	if (!count)
		return  -EINVAL;
	dev_vdbg(&acm->data->dev, "%d bytes from tty layer\n", count);

	spin_lock_irqsave(&acm->write_lock, flags);
	wbn = vpu_acm_wb_alloc(acm);
	if (wbn < 0) {
		spin_unlock_irqrestore(&acm->write_lock, flags);
		return -EINVAL;
	}
	wb = &acm->wb[wbn];

	if (!acm->dev) {
		wb->use = 0;
		spin_unlock_irqrestore(&acm->write_lock, flags);
		return -EINVAL;
	}
	const char *buf = 0;

	count = (count > acm->writesize) ? acm->writesize : count;
	dev_vdbg(&acm->data->dev, "writing %d bytes\n", count);
	memcpy(wb->buf, &buf, count);
	wb->len = count;

	stat = usb_autopm_get_interface_async(acm->control);
	if (stat) {
		wb->use = 0;
		spin_unlock_irqrestore(&acm->write_lock, flags);
		return -EINVAL;
	}

	if (acm->susp_count) {
		usb_anchor_urb(wb->urb, &acm->delayed);
		spin_unlock_irqrestore(&acm->write_lock, flags);
		return -EINVAL;
	}
	stat = vpu_acm_start_wb(acm, wb);
	spin_unlock_irqrestore(&acm->write_lock, flags);
	if (stat < 0)
		return -EINVAL;

	for (i = 0; i < acm->rx_buflimit; i++)
		usb_kill_urb(acm->read_urbs[i]);
	usb_kill_urb(acm->ctrlurb);

	usb_autopm_put_interface(acm->control);

	mutex_unlock(&acm->mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(vpu_write_reset_host);

void vpu_write_swid(struct acm *acm, int *buf, int len)
{
	int count = len;
	int stat;
	int i;
	unsigned long flags;
	int wbn;
	struct acm_wb *wb;

	if (!count)
		return;
	dev_vdbg(&acm->data->dev, "%d bytes from tty layer\n", count);

	spin_lock_irqsave(&acm->write_lock, flags);
	wbn = vpu_acm_wb_alloc(acm);
	if (wbn < 0) {
		spin_unlock_irqrestore(&acm->write_lock, flags);
		return;
	}
	wb = &acm->wb[wbn];

	if (!acm->dev) {
		wb->use = 0;
		spin_unlock_irqrestore(&acm->write_lock, flags);
		return;
	}

	count = (count > acm->writesize) ? acm->writesize : count;
	dev_vdbg(&acm->data->dev, "writing %d bytes\n", count);
	memcpy(wb->buf, &buf, count);
	wb->len = count;

	stat = usb_autopm_get_interface_async(acm->control);
	if (stat) {
		wb->use = 0;
		spin_unlock_irqrestore(&acm->write_lock, flags);
		return;
	}

	if (acm->susp_count) {
		usb_anchor_urb(wb->urb, &acm->delayed);
		spin_unlock_irqrestore(&acm->write_lock, flags);
		return;
	}
	stat = vpu_acm_start_wb(acm, wb);
	spin_unlock_irqrestore(&acm->write_lock, flags);
	if (stat < 0)
		return;

	for (i = 0; i < acm->rx_buflimit; i++)
		usb_kill_urb(acm->read_urbs[i]);
	usb_kill_urb(acm->ctrlurb);

	usb_autopm_put_interface(acm->control);

	mutex_unlock(&acm->mutex);
}

int vpu_close_host(u32 id, int len)
{
	struct acm *xdev = usb_get_device_by_id(id);
	struct urb *urb;
	struct acm *acm;
	int index = 0;
	int wbn;
	struct acm_wb *wb;

	if (!xdev)
		return -ENODEV;
	acm = vpu_acm_get_by_minor(index);
	wbn = vpu_acm_wb_alloc(acm);
	if (wbn < 0)
		return -EINVAL;

	wb = &acm->wb[wbn];
	spin_lock_irq(&acm->write_lock);
	spin_unlock_irq(&acm->write_lock);

	usb_autopm_get_interface_no_resume(acm->control);
	acm->control->needs_remote_wakeup = 0;
	usb_autopm_put_interface(acm->control);

	for (;;) {
		urb = usb_get_from_anchor(&acm->delayed);
		if (!urb)
			break;
		wb = urb->context;
		wb->use = 0;
		usb_autopm_put_interface_async(acm->control);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(vpu_close_host);

int vpu_close_reset_host(u32 id)
{
	struct acm *xdev = usb_get_device_by_id(id);
	struct acm *acm;
	int index = 0;
	struct urb *urb;
	int wbn;
	struct acm_wb *wb;

	if (!xdev)
		return -EINVAL;
	acm = vpu_acm_get_by_minor(index);
	wbn = vpu_acm_wb_alloc(acm);
	if (wbn < 0)
		return -EINVAL;
	wb = &acm->wb[wbn];
	spin_lock_irq(&acm->write_lock);
	spin_unlock_irq(&acm->write_lock);

	usb_autopm_get_interface_no_resume(acm->control);
	acm->control->needs_remote_wakeup = 0;
	usb_autopm_put_interface(acm->control);

	for (;;) {
		urb = usb_get_from_anchor(&acm->delayed);
		if (!urb)
			break;
		wb = urb->context;
		wb->use = 0;
		usb_autopm_put_interface_async(acm->control);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(vpu_close_reset_host);

void vpu_close_swid(struct acm *acm, int len)
{
	struct urb *urb;
	int wbn;
	struct acm_wb *wb;

	wbn = vpu_acm_wb_alloc(acm);
	if (wbn < 0)
		return;
	wb = &acm->wb[wbn];
	spin_lock_irq(&acm->write_lock);
	spin_unlock_irq(&acm->write_lock);

	usb_autopm_get_interface_no_resume(acm->control);
	acm->control->needs_remote_wakeup = 0;
	usb_autopm_put_interface(acm->control);

	for (;;) {
		urb = usb_get_from_anchor(&acm->delayed);
		if (!urb)
			break;
		wb = urb->context;
		wb->use = 0;
		usb_autopm_put_interface_async(acm->control);
	}
}

int vpu_read_host(u32 id, void *data, size_t *size, unsigned int timeout_ms)
{
	size_t len = *size;
	size_t remaining = len;
	struct acm *xdev = usb_get_device_by_id(id);
	struct acm *acm;
	int index = 0;
	long jiffies_passed = 0;
	long jiffies_timeout = (long)msecs_to_jiffies(timeout_ms);
	int ret = 0;

	if (!xdev)
		return -ENODEV;
	acm = vpu_acm_get_by_minor(index);
	if (!acm)
		return -ENODEV;

	ret = wait_event_interruptible_timeout(acm->wioctl,
					acm, jiffies_timeout - jiffies_passed);
	if (ret == 0)
		return -ETIME;

	*size = 0;
	if (len == 0)
		return -EINVAL;

	struct urb *urb;
	int retval = -ENODEV;
	int i;

	mutex_lock(&acm->mutex);
	retval = usb_autopm_get_interface(acm->control);
	if (retval)
		goto error_get_interface;

	/*
	 * FIXME: Why do we need this? Allocating 64K of physically contiguous
	 * memory is really nasty...
	 */

	acm->control->needs_remote_wakeup = 1;
	acm->ctrlurb->dev = acm->dev;

	/*
	 * Unthrottle device in case the TTY was closed while throttled.
	 */
	clear_bit(ACM_THROTTLED, &acm->flags);

	retval = vpu_acm_submit_read_urbs(acm, GFP_KERNEL);
	if (retval)
		goto error_submit_read_urbs;

	usb_autopm_put_interface(acm->control);

	mutex_unlock(&acm->mutex);

	int val = ACM_CTRL_DTR | ACM_CTRL_RTS;

	acm->ctrlout = val;
	vpu_acm_set_control(acm, val);
	urb = acm->read_urbs[0];
	while (remaining > 0 && urb) {
		size_t bcopy;

		bcopy = min(remaining, urb->transfer_buffer_length);
		memcpy(data, urb->transfer_buffer, bcopy);
		data += bcopy;
		remaining -= bcopy;
		*size = len - remaining;
	}
	vpu_acm_kill_urbs(acm);
	return 0;

error_submit_read_urbs:
	for (i = 0; i < acm->rx_buflimit; i++)
		usb_kill_urb(acm->read_urbs[i]);
	usb_kill_urb(acm->ctrlurb);
error_get_interface:

	mutex_unlock(&acm->mutex);

	return usb_translate_errors(retval);
}
EXPORT_SYMBOL_GPL(vpu_read_host);

/* Little helpers: write/read buffers free */
static void vpu_acm_write_buffers_free(struct acm *acm)
{
	int i;
	struct acm_wb *wb;

	for (wb = &acm->wb[0], i = 0; i < ACM_NW; i++, wb++)
		usb_free_coherent(acm->dev, acm->writesize, wb->buf, wb->dmah);
}

static void vpu_acm_read_buffers_free(struct acm *acm)
{
	int i;

	for (i = 0; i < acm->rx_buflimit; i++)
		usb_free_coherent(acm->dev, acm->readsize,
			  acm->read_buffers[i].base, acm->read_buffers[i].dma);
}

/* Little helper: write buffers allocate */
static int vpu_acm_write_buffers_alloc(struct acm *acm)
{
	int i;
	struct acm_wb *wb;

	for (wb = &acm->wb[0], i = 0; i < ACM_NW; i++, wb++) {
		wb->buf = usb_alloc_coherent(acm->dev, acm->writesize,
			GFP_KERNEL, &wb->dmah);
		if (!wb->buf) {
			while (i != 0) {
				--i;
				--wb;
				usb_free_coherent(acm->dev, acm->writesize,
				    wb->buf, wb->dmah);
			}
			return -ENOMEM;
		}
	}
	return 0;
}

static void mxlk_usb_add_device(struct acm *acm, int devnum, int busnum)
{
	mutex_lock(&dev_list_mutex);

	acm->devid = (XLINK_DEV_INF_USB << XLINK_DEV_INF_TYPE_SHIFT) |
		      ((busnum << 8 | devnum) << XLINK_DEV_PHYS_ID_SHIFT) |
		      (XLINK_DEV_TYPE_KMB << XLINK_DEV_TYPE_SHIFT) |
		      (XLINK_DEV_SLICE_0 << XLINK_DEV_SLICE_ID_SHIFT) |
		      (XLINK_DEV_FUNC_VPU << XLINK_DEV_FUNC_SHIFT);

	acm->status = 4;

	list_add_tail(&acm->list, &dev_list);
	mutex_unlock(&dev_list_mutex);
}

static int vpu_acm_probe(struct usb_interface *intf,
		     const struct usb_device_id *id)
{
	struct usb_cdc_union_desc *union_header = NULL;
	struct usb_cdc_call_mgmt_descriptor *cmgmd = NULL;
	unsigned char *buffer = intf->altsetting->extra;
	int buflen = intf->altsetting->extralen;
	struct usb_interface *control_interface;
	struct usb_interface *data_interface;
	struct usb_endpoint_descriptor *epctrl = NULL;
	struct usb_endpoint_descriptor *epread = NULL;
	struct usb_endpoint_descriptor *epwrite = NULL;
	struct usb_device *usb_dev = interface_to_usbdev(intf);
	struct usb_cdc_parsed_header h;
	struct acm *acm;
	int minor;
	int ctrlsize, readsize;
	u8 *buf;
	int call_intf_num = -1;
	int data_intf_num = -1;
	unsigned long quirks;
	int num_rx_buf;
	int i;
	int combined_interfaces = 0;
	int rv = -ENOMEM;
	int res;
	size_t size = 8;

	/* normal quirks */
	quirks = (unsigned long)id->driver_info;

	if (quirks == IGNORE_DEVICE)
		return -ENODEV;

	memset(&h, 0x00, sizeof(struct usb_cdc_parsed_header));

	num_rx_buf = (quirks == SINGLE_RX_URB) ? 1 : ACM_NR;

	/* handle quirks deadly to normal probing*/
	if (quirks == NO_UNION_NORMAL) {
		data_interface = usb_ifnum_to_if(usb_dev, 1);
		control_interface = usb_ifnum_to_if(usb_dev, 0);
		/* we would crash */
		if (!data_interface || !control_interface)
			return -ENODEV;
		goto skip_normal_probe;
	}

	/* normal probing*/
	if (!buffer) {
		dev_err(&intf->dev, "Weird descriptor references\n");
		return -EINVAL;
	}

	if (!intf->cur_altsetting)
		return -EINVAL;

	if (!buflen) {
		if (intf->cur_altsetting->endpoint &&
				intf->cur_altsetting->endpoint->extralen &&
				intf->cur_altsetting->endpoint->extra) {
			dev_dbg(&intf->dev,
				"Seeking extra descriptors on endpoint\n");
			buflen = intf->cur_altsetting->endpoint->extralen;
			buffer = intf->cur_altsetting->endpoint->extra;
		} else {
			dev_err(&intf->dev,
				"Zero length descriptor references\n");
			return -EINVAL;
		}
	}

	cdc_parse_cdc_header(&h, intf, buffer, buflen);
	union_header = h.usb_cdc_union_desc;
	cmgmd = h.usb_cdc_call_mgmt_descriptor;
	if (cmgmd)
		call_intf_num = cmgmd->bDataInterface;

	if (!union_header) {
		if (call_intf_num > 0) {
			dev_dbg(&intf->dev, "No union descriptor, using call management descriptor\n");
			/* quirks for Droids MuIn LCD */
			if (quirks & NO_DATA_INTERFACE) {
				data_interface = usb_ifnum_to_if(usb_dev, 0);
			} else {
				data_intf_num = call_intf_num;
				data_interface = usb_ifnum_to_if(usb_dev,
					data_intf_num);
			}
			control_interface = intf;
		} else {
			if (intf->cur_altsetting->desc.bNumEndpoints != 3) {
				dev_dbg(&intf->dev, "No union descriptor, giving up\n");
				return -ENODEV;
			}
			dev_warn(&intf->dev, "No union descriptor, testing for castrated device\n");
			combined_interfaces = 1;
			control_interface = data_interface = intf;
			goto look_for_collapsed_interface;
		}
	} else {
		data_intf_num = union_header->bSlaveInterface0;
		control_interface = usb_ifnum_to_if(usb_dev,
				union_header->bMasterInterface0);
		data_interface = usb_ifnum_to_if(usb_dev, data_intf_num);
	}

	if (!control_interface || !data_interface) {
		dev_dbg(&intf->dev, "no interfaces\n");
		return -ENODEV;
	}
	if (!data_interface->cur_altsetting
			|| !control_interface->cur_altsetting)
		return -ENODEV;

	if (data_intf_num != call_intf_num)
		dev_dbg(&intf->dev, "Separate call control interface. That is not fully supported.\n");

	if (control_interface == data_interface) {
		/* some broken devices designed for windows work this way */
		dev_warn(&intf->dev, "Control and data interfaces are not separated!\n");
		combined_interfaces = 1;
		/* a popular other OS doesn't use it */
		quirks |= NO_CAP_LINE;
		if (data_interface->cur_altsetting->desc.bNumEndpoints != 3) {
			dev_err(&intf->dev, "This needs exactly 3 endpoints\n");
			return -EINVAL;
		}
look_for_collapsed_interface:
		res = usb_find_common_endpoints(data_interface->cur_altsetting,
				&epread, &epwrite, &epctrl, NULL);
		if (res)
			return res;

		goto made_compressed_probe;
	}

skip_normal_probe:
	/*workaround for switched interfaces */
	if (data_interface->cur_altsetting->desc.bInterfaceClass
						!= CDC_DATA_INTERFACE_TYPE) {
		if (control_interface->cur_altsetting->desc.bInterfaceClass
						== CDC_DATA_INTERFACE_TYPE) {
			dev_dbg(&intf->dev,
				"Your device has switched interfaces.\n");
			swap(control_interface, data_interface);
		} else {
			return -EINVAL;
		}
	}

	/* Accept probe requests only for the control interface */
	if (!combined_interfaces && intf != control_interface)
		return -ENODEV;

	if (!combined_interfaces && usb_interface_claimed(data_interface)) {
		/* valid in this context */
		dev_dbg(&intf->dev, "The data interface isn't available\n");
		return -EBUSY;
	}


	if (data_interface->cur_altsetting->desc.bNumEndpoints < 2 ||
	    control_interface->cur_altsetting->desc.bNumEndpoints == 0)
		return -EINVAL;

	epctrl = &control_interface->cur_altsetting->endpoint[0].desc;
	epread = &data_interface->cur_altsetting->endpoint[0].desc;
	epwrite = &data_interface->cur_altsetting->endpoint[1].desc;


	/* workaround for switched endpoints */
	if (!usb_endpoint_dir_in(epread)) {
		/* descriptors are swapped */
		dev_dbg(&intf->dev,
			"The data interface has switched endpoints\n");
		swap(epread, epwrite);
	}
made_compressed_probe:
	dev_dbg(&intf->dev, "interfaces are valid\n");
	acm = kzalloc(sizeof(struct acm), GFP_KERNEL);
	if (acm == NULL)
		goto alloc_fail;
	struct usb_device *udev = interface_to_usbdev(intf);
	int devnum = udev->devnum;
	int busnum = udev->bus->busnum;

	mxlk_usb_add_device(acm, devnum, busnum);

	minor = vpu_acm_alloc_minor(acm);
	if (minor < 0)
		goto alloc_fail1;

	ctrlsize = usb_endpoint_maxp(epctrl);
	readsize = usb_endpoint_maxp(epread) *
				(quirks == SINGLE_RX_URB ? 1 : 2);
	acm->combined_interfaces = combined_interfaces;
	acm->writesize = usb_endpoint_maxp(epwrite) * 20;
	acm->control = control_interface;
	acm->data = data_interface;
	acm->minor = minor;
	acm->dev = usb_dev;
	if (h.usb_cdc_acm_descriptor)
		acm->ctrl_caps = h.usb_cdc_acm_descriptor->bmCapabilities;
	if (quirks & NO_CAP_LINE)
		acm->ctrl_caps &= ~USB_CDC_CAP_LINE;
	acm->ctrlsize = ctrlsize;
	acm->readsize = readsize;
	acm->rx_buflimit = num_rx_buf;
	INIT_WORK(&acm->work, vpu_acm_softint);
	init_waitqueue_head(&acm->wioctl);
	spin_lock_init(&acm->write_lock);
	spin_lock_init(&acm->read_lock);
	mutex_init(&acm->mutex);
	if (usb_endpoint_xfer_int(epread)) {
		acm->bInterval = epread->bInterval;
		acm->in = usb_rcvintpipe(usb_dev, epread->bEndpointAddress);
	} else {
		acm->in = usb_rcvbulkpipe(usb_dev, epread->bEndpointAddress);
	}
	if (usb_endpoint_xfer_int(epwrite))
		acm->out = usb_sndintpipe(usb_dev, epwrite->bEndpointAddress);
	else
		acm->out = usb_sndbulkpipe(usb_dev, epwrite->bEndpointAddress);
	init_usb_anchor(&acm->delayed);
	acm->quirks = quirks;

	buf = usb_alloc_coherent(usb_dev, ctrlsize, GFP_KERNEL, &acm->ctrl_dma);
	if (!buf)
		goto alloc_fail1;
	acm->ctrl_buffer = buf;

	if (vpu_acm_write_buffers_alloc(acm) < 0)
		goto alloc_fail2;

	acm->ctrlurb = usb_alloc_urb(0, GFP_KERNEL);
	if (!acm->ctrlurb)
		goto alloc_fail3;
	for (i = 0; i < num_rx_buf; i++) {
		struct acm_rb *rb = &(acm->read_buffers[i]);
		struct urb *urb;

		rb->base = usb_alloc_coherent(acm->dev, readsize, GFP_KERNEL,
								&rb->dma);
		if (!rb->base)
			goto alloc_fail4;
		rb->index = i;
		rb->instance = acm;

		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!urb)
			goto alloc_fail4;

		urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		urb->transfer_dma = rb->dma;
		if (usb_endpoint_xfer_int(epread))
			usb_fill_int_urb(urb, acm->dev, acm->in, rb->base,
					 acm->readsize,
					 vpu_acm_read_bulk_callback, rb,
					 acm->bInterval);
		else
			usb_fill_bulk_urb(urb, acm->dev, acm->in, rb->base,
					  acm->readsize,
					  vpu_acm_read_bulk_callback, rb);

		acm->read_urbs[i] = urb;
		__set_bit(i, &acm->read_urbs_free);
	}
	for (i = 0; i < ACM_NW; i++) {
		struct acm_wb *snd = &(acm->wb[i]);

		snd->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (snd->urb == NULL)
			goto alloc_fail5;

		if (usb_endpoint_xfer_int(epwrite))
			usb_fill_int_urb(snd->urb, usb_dev, acm->out,
				NULL, acm->writesize, vpu_acm_write_bulk,
				snd, epwrite->bInterval);
		else
			usb_fill_bulk_urb(snd->urb, usb_dev, acm->out,
				NULL, acm->writesize, vpu_acm_write_bulk, snd);
		snd->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		if (quirks & SEND_ZERO_PACKET)
			snd->urb->transfer_flags |= URB_ZERO_PACKET;
		snd->instance = acm;
	}
	usb_set_intfdata(intf, acm);

	i = device_create_file(&intf->dev, &dev_attr_bmCapabilities);
	if (i < 0)
		goto alloc_fail5;

	if (h.usb_cdc_country_functional_desc) { /* export the country data */
		struct usb_cdc_country_functional_desc *cfd =
					h.usb_cdc_country_functional_desc;

		acm->country_codes = kmalloc(cfd->bLength - 4, GFP_KERNEL);
		if (!acm->country_codes)
			goto skip_countries;
		acm->country_code_size = cfd->bLength - 4;
		memcpy(acm->country_codes, (u8 *)&cfd->wCountyCode0,
							cfd->bLength - 4);
		acm->country_rel_date = cfd->iCountryCodeRelDate;

		i = device_create_file(&intf->dev, &dev_attr_wCountryCodes);
		if (i < 0) {
			kfree(acm->country_codes);
			acm->country_codes = NULL;
			acm->country_code_size = 0;
			goto skip_countries;
		}

		i = device_create_file(&intf->dev,
						&dev_attr_iCountryCodeRelDate);
		if (i < 0) {
			device_remove_file(&intf->dev, &dev_attr_wCountryCodes);
			kfree(acm->country_codes);
			acm->country_codes = NULL;
			acm->country_code_size = 0;
			goto skip_countries;
		}
	}

skip_countries:
	usb_fill_int_urb(acm->ctrlurb, usb_dev,
			 usb_rcvintpipe(usb_dev, epctrl->bEndpointAddress),
			 acm->ctrl_buffer, ctrlsize, vpu_acm_ctrl_irq, acm,
			 /* works around buggy devices */
			 epctrl->bInterval ? epctrl->bInterval : 16);
	acm->ctrlurb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	acm->ctrlurb->transfer_dma = acm->ctrl_dma;
	acm->notification_buffer = NULL;
	acm->nb_index = 0;
	acm->nb_size = 0;

	acm->line.dwDTERate = cpu_to_le32(9600);
	acm->line.bDataBits = 8;
	vpu_acm_set_line(acm, &acm->line);

	usb_driver_claim_interface(&vpu_acm_driver, data_interface, acm);
	usb_set_intfdata(data_interface, acm);

	usb_get_intf(control_interface);


	if (quirks & CLEAR_HALT_CONDITIONS) {
		usb_clear_halt(usb_dev, acm->in);
		usb_clear_halt(usb_dev, acm->out);
	}
	vpu_write_swid(acm, devnum, size);
	vpu_close_swid(acm, size);
	vpu_write_swid(acm, busnum, size);
	vpu_close_swid(acm, size);

	return 0;

alloc_fail5:
	usb_set_intfdata(intf, NULL);
	for (i = 0; i < ACM_NW; i++)
		usb_free_urb(acm->wb[i].urb);
alloc_fail4:
	for (i = 0; i < num_rx_buf; i++)
		usb_free_urb(acm->read_urbs[i]);
	vpu_acm_read_buffers_free(acm);
	usb_free_urb(acm->ctrlurb);
alloc_fail3:
	vpu_acm_write_buffers_free(acm);
alloc_fail2:
	usb_free_coherent(usb_dev, ctrlsize, acm->ctrl_buffer, acm->ctrl_dma);
alloc_fail1:
	kfree(acm);
alloc_fail:
	return rv;
}

static void vpu_acm_disconnect(struct usb_interface *intf)
{
	struct acm *acm = usb_get_intfdata(intf);
	int i;

	/* sibling interface is already cleaning up */
	if (!acm)
		return;

	mutex_lock(&acm->mutex);
	acm->disconnected = true;
	acm->status = 0;
	__list_del_entry(&acm->list);
	if (acm->country_codes) {
		device_remove_file(&acm->control->dev,
				&dev_attr_wCountryCodes);
		device_remove_file(&acm->control->dev,
				&dev_attr_iCountryCodeRelDate);
	}
	wake_up_all(&acm->wioctl);
	device_remove_file(&acm->control->dev, &dev_attr_bmCapabilities);
	usb_set_intfdata(acm->control, NULL);
	usb_set_intfdata(acm->data, NULL);
	mutex_unlock(&acm->mutex);

	vpu_acm_kill_urbs(acm);
	cancel_work_sync(&acm->work);


	usb_free_urb(acm->ctrlurb);
	for (i = 0; i < ACM_NW; i++)
		usb_free_urb(acm->wb[i].urb);
	for (i = 0; i < acm->rx_buflimit; i++)
		usb_free_urb(acm->read_urbs[i]);
	vpu_acm_write_buffers_free(acm);
	usb_free_coherent(acm->dev, acm->ctrlsize,
					acm->ctrl_buffer, acm->ctrl_dma);
	vpu_acm_read_buffers_free(acm);

	kfree(acm->notification_buffer);

	if (!acm->combined_interfaces)
		usb_driver_release_interface(&vpu_acm_driver, intf ==
				acm->control ? acm->data : acm->control);
}

#ifdef CONFIG_PM
static int vpu_acm_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct acm *acm = usb_get_intfdata(intf);
	int cnt;

	spin_lock_irq(&acm->write_lock);
	if (PMSG_IS_AUTO(message)) {
		if (acm->transmitting) {
			spin_unlock_irq(&acm->write_lock);
			return -EBUSY;
		}
	}
	cnt = acm->susp_count++;
	spin_unlock_irq(&acm->write_lock);

	if (cnt)
		return 0;

	vpu_acm_kill_urbs(acm);
	cancel_work_sync(&acm->work);

	return 0;
}

static int vpu_acm_resume(struct usb_interface *intf)
{
	struct acm *acm = usb_get_intfdata(intf);
	int rv = 0;

	spin_lock_irq(&acm->write_lock);

	if (--acm->susp_count)
		goto out;

out:
	spin_unlock_irq(&acm->write_lock);

	return rv;
}

static int vpu_acm_reset_resume(struct usb_interface *intf)
{
	return vpu_acm_resume(intf);
}

#endif /* CONFIG_PM */

static int vpu_acm_pre_reset(struct usb_interface *intf)
{
	struct acm *acm = usb_get_intfdata(intf);

	clear_bit(EVENT_RX_STALL, &acm->flags);
	acm->nb_index = 0; /* pending control transfers are lost */

	return 0;
}

#define NOKIA_PCSUITE_ACM_INFO(x) \
		USB_DEVICE_AND_INTERFACE_INFO(0x0421, x, \
		USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM, \
		USB_CDC_ACM_PROTO_VENDOR)

#define SAMSUNG_PCSUITE_ACM_INFO(x) \
		USB_DEVICE_AND_INTERFACE_INFO(0x04e7, x, \
		USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM, \
		USB_CDC_ACM_PROTO_VENDOR)

/*
 * USB driver structure.
 */

static const struct usb_device_id vpu_acm_ids[] = {
	/* control interfaces without any protocol set */
	{ USB_INTERFACE_INFO(USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM,
		USB_CDC_PROTO_NONE) },

	/* control interfaces with various AT-command sets */
	{ USB_INTERFACE_INFO(USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM,
		USB_CDC_ACM_PROTO_AT_V25TER) },
	{ USB_INTERFACE_INFO(USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM,
		USB_CDC_ACM_PROTO_AT_PCCA101) },
	{ USB_INTERFACE_INFO(USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM,
		USB_CDC_ACM_PROTO_AT_PCCA101_WAKE) },
	{ USB_INTERFACE_INFO(USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM,
		USB_CDC_ACM_PROTO_AT_GSM) },
	{ USB_INTERFACE_INFO(USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM,
		USB_CDC_ACM_PROTO_AT_3G) },
	{ USB_INTERFACE_INFO(USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM,
		USB_CDC_ACM_PROTO_AT_CDMA) },

	{ USB_DEVICE(0x1519, 0x0452), /* Intel 7260 modem */
	.driver_info = SEND_ZERO_PACKET,
	},

	{ }
};

MODULE_DEVICE_TABLE(usb, vpu_acm_ids);

static struct usb_driver vpu_acm_driver = {
	.name =		"vpu_cdc_acm",
	.probe =	vpu_acm_probe,
	.disconnect =	vpu_acm_disconnect,
#ifdef CONFIG_PM
	.suspend =	vpu_acm_suspend,
	.resume =	vpu_acm_resume,
	.reset_resume =	vpu_acm_reset_resume,
#endif
	.pre_reset =	vpu_acm_pre_reset,
	.id_table =	vpu_acm_ids,
#ifdef CONFIG_PM
	.supports_autosuspend = 1,
#endif
	.disable_hub_initiated_lpm = 1,
};

/*
 * Init / exit.
 */

static int __init vpu_acm_init(void)
{
	int retval;

	retval = usb_register(&vpu_acm_driver);

	return 0;
}

static void __exit vpu_acm_exit(void)
{
	usb_deregister(&vpu_acm_driver);
	idr_destroy(&vpu_acm_minors);
}

module_init(vpu_acm_init);
module_exit(vpu_acm_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_ALIAS_CHARDEV_MAJOR(ACM_TTY_MAJOR);
