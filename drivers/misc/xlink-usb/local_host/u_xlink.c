// SPDX-License-Identifier: GPL-2.0-only
/*
 * u_xlink.c - utilities for USB gadget
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
#include <linux/sched.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/slab.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/console.h>
#include <linux/kthread.h>
#include <linux/workqueue.h>
#include <linux/kfifo.h>
#include "u_xlink.h"
#include <linux/xlink_drv_inf.h>

#define QUEUE_SIZE		16
#define WRITE_BUF_SIZE		8192		/* TX only */
#define GS_CONSOLE_BUF_SIZE	8192

u8 PORT_NUM;

/* console info */
struct gscons_info {
	struct gs_port		*port;
	struct task_struct	*console_thread;
	struct kfifo		con_buf;
	/* protect the buf and busy flag */
	spinlock_t		con_lock;
	int			req_busy;
	struct usb_request	*console_req;
};

/*
 * The port structure holds info for each port, one for each minor number
 * (and thus for each /dev/ node).
 */
struct gs_port {
	struct tty_port		port;
	spinlock_t		port_lock;	/* guard port_* access */

	struct gserial		*port_usb;

	bool			openclose;	/* open/close in progress */
	u8			port_num;

	struct list_head	read_pool;
	int read_started;
	int read_allocated;
	struct list_head	read_queue;
	unsigned int		n_read;
	struct delayed_work	push;

	struct list_head	write_pool;
	int write_started;
	int write_allocated;
	struct kfifo		port_write_buf;
	wait_queue_head_t	drain_wait;	/* wait while writes drain */
	bool                    write_busy;
	wait_queue_head_t	close_wait;

	/* REVISIT this state ... */
	struct usb_cdc_line_coding port_line_coding;	/* 8-N-1 etc */
};

static struct portmaster {
	struct mutex	lock;			/* protect open/close */
	struct gs_port	*port;
} ports[MAX_U_SERIAL_PORTS];

#define GS_CLOSE_TIMEOUT		15		/* seconds */

#ifdef VERBOSE_DEBUG
#ifndef pr_vdebug
#define pr_vdebug(fmt, arg...) \
	pr_debug(fmt, ##arg)
#endif /* pr_vdebug */
#else
#ifndef pr_vdebug
#define pr_vdebug(fmt, arg...) \
	({ if (0) pr_debug(fmt, ##arg); })
#endif /* pr_vdebug */
#endif

/*-------------------------------------------------------------------------*/

/* I/O glue between TTY (upper) and USB function (lower) driver layers */

/*
 * vpu_alloc_req
 *
 * Allocate a usb_request and its buffer.  Returns a pointer to the
 * usb_request or NULL if there is an error.
 */
struct usb_request *
vpu_alloc_req(struct usb_ep *ep, unsigned int len, gfp_t kmalloc_flags)
{
	struct usb_request *req;

	req = usb_ep_alloc_request(ep, kmalloc_flags);
	if (req != NULL) {
		req->length = len;
		req->buf = kmalloc(len, kmalloc_flags);
		if (req->buf == NULL) {
			usb_ep_free_request(ep, req);
			return NULL;
		}
	}
	return req;
}
EXPORT_SYMBOL_GPL(vpu_alloc_req);

/*
 * vpu_free_req
 *
 * Free a usb_request and its buffer.
 */
void vpu_free_req(struct usb_ep *ep, struct usb_request *req)
{
	kfree(req->buf);
	usb_ep_free_request(ep, req);
}
EXPORT_SYMBOL_GPL(vpu_free_req);

/*
 * vpu_send_packet
 *
 * If there is data to send, a packet is built in the given
 * buffer and the size is returned.  If there is no data to
 * send, 0 is returned.
 *
 * Called with port_lock held.
 */
static unsigned
vpu_send_packet(struct gs_port *port, char *packet, unsigned int size)
{
	unsigned int len;

	len = kfifo_len(&port->port_write_buf);
	if (len < size)
		size = len;

	size = kfifo_out(&port->port_write_buf, packet, size);

	return size;
}

/*
 * vpu_start_tx
 *
 * This function finds available write requests, calls
 * vpu_send_packet to fill these packets with data, and
 * continues until either there are no more write requests
 * available or no more data to send.  This function is
 * run whenever data arrives or write requests are available.
 *
 * Context: caller owns port_lock; port_usb is non-null.
 */
static int vpu_start_tx(struct gs_port *port)
/*
 *__releases(&port->port_lock)
 *__acquires(&port->port_lock)
 */
{
	struct list_head	*pool = &port->write_pool;
	struct usb_ep		*in;
	int			status = 0;

	if (!port->port_usb)
		return status;

	in = port->port_usb->in;

	while (!port->write_busy && !list_empty(pool)) {
		struct usb_request	*req;
		int			len;

		if (port->write_started >= QUEUE_SIZE)
			break;

		req = list_entry(pool->next, struct usb_request, list);
		len = vpu_send_packet(port, req->buf, in->maxpacket);

		if (len == 0) {
			wake_up_interruptible(&port->drain_wait);
			break;
		}

		req->length = len;
		list_del(&req->list);
		req->zero = kfifo_is_empty(&port->port_write_buf);

		port->write_busy = true;
		spin_unlock(&port->port_lock);
		status = usb_ep_queue(in, req, GFP_ATOMIC);
		spin_lock(&port->port_lock);
		port->write_busy = false;

		if (status) {
			list_add(&req->list, pool);
			break;
		}

		port->write_started++;

		/* abort immediately after disconnect */
		if (!port->port_usb)
			break;
	}

	return status;
}

/*
 * Context: caller owns port_lock, and port_usb is set
 */
unsigned int vpu_start_rx(struct gs_port *port)
/*
 *__releases(&port->port_lock)
 *__acquires(&port->port_lock)
 */
{
	struct list_head	*pool = &port->read_pool;
	struct usb_ep		*out;

	while (!list_empty(pool)) {
		struct usb_request	*req;
		int			status;

		if (!port->port_usb)
			break;

		out = port->port_usb->out;

		if (port->read_started >= QUEUE_SIZE)
			break;

		req = list_entry(pool->next, struct usb_request, list);
		list_del(&req->list);
		req->length = out->maxpacket;
		/* drop lock while we call out; the controller driver
		 * may need to call us back (e.g. for disconnect)
		 */
		spin_unlock(&port->port_lock);
		status = usb_ep_queue(out, req, GFP_ATOMIC);
		spin_lock(&port->port_lock);

		if (status) {
			pr_debug("%s: %s %s err %d\n",
					__func__, "queue", out->name, status);
			list_add(&req->list, pool);
			break;
		}
		port->read_started++;

		/* abort immediately after disconnect */
	}

	return port->read_started;
}
EXPORT_SYMBOL_GPL(vpu_start_rx);

/*
 * RX tasklet takes data out of the RX queue and hands it up to the TTY
 * layer until it refuses to take any more data (or is throttled back).
 * Then it issues reads for any further data.
 *
 * If the RX queue becomes full enough that no usb_request is queued,
 * the OUT endpoint may begin NAKing as soon as its FIFO fills up.
 * So QUEUE_SIZE packets plus however many the FIFO holds (usually two)
 * can be buffered before the TTY layer's buffers (currently 64 KB).
 */
static void vpu_rx_push(struct work_struct *work)
{
	struct delayed_work	*w = to_delayed_work(work);
	struct gs_port		*port = container_of(w, struct gs_port, push);
	struct list_head	*queue = &port->read_queue;
	bool			disconnect = false;
	bool			do_push = false;

	/* hand any queued data to the tty */
	spin_lock_irq(&port->port_lock);
	while (!list_empty(queue)) {
		struct usb_request	*req;

		req = list_first_entry(queue, struct usb_request, list);

		switch (req->status) {
		case -ESHUTDOWN:
			disconnect = true;
			pr_vdebug("ttyGS%d: shutdown\n", port->port_num);
			break;

		default:
			/* presumably a transient fault */
			pr_warn("ttyGS%d: unexpected RX status %d\n",
				port->port_num, req->status);
			/* FALLTHROUGH */
		case 0:
			/* normal completion */
			break;
		}

		/* push data to (open) tty */
		if (req->actual) {
			char		*packet = req->buf;
			unsigned int	size = req->actual;
			unsigned int	n;
			int		count;
			/* we may have pushed part of this packet already... */
			n = port->n_read;
			if (n) {
				packet += n;
				size -= n;
			}

			count = tty_insert_flip_string(&port->port, packet,
					size);
			if (count)
				do_push = true;
			if (count != size) {
				/* stop pushing; TTY layer can't handle more */
				port->n_read += count;
				pr_vdebug("ttyGS%d: rx block %d/%d\n",
					  port->port_num, count, req->actual);
				break;
			}
			port->n_read = 0;
		}

		list_move(&req->list, &port->read_pool);
		port->read_started--;
	}

	/* Push from tty to ldisc; this is handled by a workqueue,
	 * so we won't get callbacks and can hold port_lock
	 */
	if (do_push)
		tty_flip_buffer_push(&port->port);

	spin_unlock_irq(&port->port_lock);
}

void vpu_read_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct gs_port	*port = ep->driver_data;
	/* Queue all received data until the tty layer is ready for it. */
	spin_lock(&port->port_lock);
	list_add_tail(&req->list, &port->read_queue);

	schedule_delayed_work(&port->push, 0);
	spin_unlock(&port->port_lock);
}
EXPORT_SYMBOL_GPL(vpu_read_complete);

static void vpu_write_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct gs_port	*port = ep->driver_data;

	spin_lock(&port->port_lock);
	list_add(&req->list, &port->write_pool);
	port->write_started--;

	switch (req->status) {
	default:
		/* presumably a transient fault */
		pr_warn("%s: unexpected %s status %d\n",
			__func__, ep->name, req->status);
		/* FALL THROUGH */
	case 0:
		/* normal completion */
		vpu_start_tx(port);
		break;

	case -ESHUTDOWN:
		/* disconnect */
		pr_vdebug("%s: %s shutdown\n", __func__, ep->name);
		break;
	}

	spin_unlock(&port->port_lock);
}

static void vpu_free_requests(struct usb_ep *ep, struct list_head *head,
							 int *allocated)
{
	struct usb_request	*req;

	while (!list_empty(head)) {
		req = list_entry(head->next, struct usb_request, list);
		list_del(&req->list);
		vpu_free_req(ep, req);
		if (allocated)
			(*allocated)--;
	}
}

int vpu_alloc_requests(struct usb_ep *ep, struct list_head *head,
		void (*fn)(struct usb_ep *, struct usb_request *),
		int *allocated)
{
	int			i;
	struct usb_request	*req;
	int n = allocated ? QUEUE_SIZE - *allocated : QUEUE_SIZE;
	/* Pre-allocate up to QUEUE_SIZE transfers, but if we can't
	 * do quite that many this time, don't fail ... we just won't
	 * be as speedy as we might otherwise be.
	 */
	for (i = 0; i < n; i++) {
		req = vpu_alloc_req(ep, ep->maxpacket, GFP_ATOMIC);
		if (!req)
			return list_empty(head) ? -ENOMEM : 0;
		req->complete = fn;
		list_add_tail(&req->list, head);
		if (allocated)
			(*allocated)++;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(vpu_alloc_requests);
/**
 * vpu_start_io - start USB I/O streams
 * @dev: encapsulates endpoints to use
 * Context: holding port_lock; port_tty and port_usb are non-null
 *
 * We only start I/O when something is connected to both sides of
 * this port.  If nothing is listening on the host side, we may
 * be pointlessly filling up our TX buffers and FIFO.
 */
int vpu_start_io(struct gs_port *port)
{
	struct list_head	*head = &port->read_pool;
	struct usb_ep		*ep = port->port_usb->out;
	int			status;
	unsigned int		started;

	/* Allocate RX and TX I/O buffers.  We can't easily do this much
	 * earlier (with GFP_KERNEL) because the requests are coupled to
	 * endpoints, as are the packet sizes we'll be using.  Different
	 * configurations may use different endpoints with a given port;
	 * and high speed vs full speed changes packet sizes too.
	 */
	status = vpu_alloc_requests(ep, head, vpu_read_complete,
		&port->read_allocated);
	if (status)
		return status;

	status = vpu_alloc_requests(port->port_usb->in, &port->write_pool,
			vpu_write_complete, &port->write_allocated);
	if (status) {
		vpu_free_requests(ep, head, &port->read_allocated);
		return status;
	}

	/* queue read requests */
	port->n_read = 0;
	started = vpu_start_rx(port);

	/* unblock any pending writes into our circular buffer */
	if (started) {
		//tty_wakeup(port->port.tty);
	} else {
		vpu_free_requests(ep, head, &port->read_allocated);
		vpu_free_requests(port->port_usb->in, &port->write_pool,
			&port->write_allocated);
		status = -EIO;
	}

	return status;
}
EXPORT_SYMBOL_GPL(vpu_start_io);

/*-------------------------------------------------------------------------*/

/* TTY Driver */

/*
 * vpu_open sets up the link between a vpu_port and its associated TTY.
 * That link is broken *only* by TTY close(), and all driver methods
 * know that.
 */
static int vpu_open(struct tty_struct *tty, struct file *file)
{
	int		port_num = tty->index;
	struct gs_port	*port;
	int		status;

	do {
		mutex_lock(&ports[port_num].lock);
		port = ports[port_num].port;
		if (!port)
			status = -ENODEV;
		else {
			spin_lock_irq(&port->port_lock);

			/* already open?  Great. */
			if (port->port.count) {
				status = 0;
				port->port.count++;

			/* currently opening/closing? wait ... */
			} else if (port->openclose) {
				status = -EBUSY;

			/* ... else we do the work */
			} else {
				status = -EAGAIN;
				port->openclose = true;
			}
			spin_unlock_irq(&port->port_lock);
		}
		mutex_unlock(&ports[port_num].lock);

		switch (status) {
		default:
			/* fully handled */
			return status;
		case -EAGAIN:
			/* must do the work */
			break;
		case -EBUSY:
			/* REVISIT could have a waitchannel here, if
			 * concurrent open performance is important
			 */
			break;
		}
	} while (status != -EAGAIN);

	/* Do the "real open" */
	spin_lock_irq(&port->port_lock);

	/* allocate circular buffer on first open */
	if (!kfifo_initialized(&port->port_write_buf)) {

		spin_unlock_irq(&port->port_lock);
		status = kfifo_alloc(&port->port_write_buf,
				     WRITE_BUF_SIZE, GFP_KERNEL);
		spin_lock_irq(&port->port_lock);

		if (status) {
			port->openclose = false;
			goto exit_unlock_port;
		}
	}

	/* REVISIT if REMOVED (ports[].port NULL), abort the open
	 * to let rmmod work faster (but this way isn't wrong).
	 */

	/* REVISIT maybe wait for "carrier detect" */

	tty->driver_data = port;
	port->port.tty = tty;

	port->port.count = 1;
	port->openclose = false;

	/* if connected, start the I/O stream */
	if (port->port_usb) {
		struct gserial	*gser = port->port_usb;

		pr_debug("%s: start ttyGS%d\n", __func__, port->port_num);
		vpu_start_io(port);

		if (gser->connect)
			gser->connect(gser);
	}

	status = 0;


exit_unlock_port:
	spin_unlock_irq(&port->port_lock);
	return status;
}

static int vpu_writes_finished(struct gs_port *p)
{
	int cond;

	/* return true on disconnect or empty buffer */
	spin_lock_irq(&p->port_lock);
	cond = (p->port_usb == NULL) || !kfifo_len(&p->port_write_buf);
	spin_unlock_irq(&p->port_lock);

	return cond;
}

static void vpu_close(struct tty_struct *tty, struct file *file)
{
	struct gs_port *port = tty->driver_data;
	struct gserial	*gser;

	spin_lock_irq(&port->port_lock);

	if (port->port.count != 1) {
		if (port->port.count == 0)
			WARN_ON(1);
		else
			--port->port.count;
		goto exit;
	}

	pr_debug("%s: ttyGS%d (%p,%p) ...\n",
			__func__, port->port_num, tty, file);

	/* mark port as closing but in use; we can drop port lock
	 * and sleep if necessary
	 */
	port->openclose = true;
	port->port.count = 0;

	gser = port->port_usb;
	if (gser && gser->disconnect)
		gser->disconnect(gser);

	/* wait for circular write buffer to drain, disconnect, or at
	 * most GS_CLOSE_TIMEOUT seconds; then discard the rest
	 */
	if (kfifo_len(&port->port_write_buf) > 0 && gser) {

		spin_unlock_irq(&port->port_lock);
		wait_event_interruptible_timeout(port->drain_wait,
					vpu_writes_finished(port),
					GS_CLOSE_TIMEOUT * HZ);
		spin_lock_irq(&port->port_lock);
		gser = port->port_usb;
	}

	/* Iff we're disconnected, there can be no I/O in flight so it's
	 * ok to free the circular buffer; else just scrub it.  And don't
	 * let the push tasklet fire again until we're re-opened.
	 */
	if (gser == NULL)
		kfifo_free(&port->port_write_buf);
	else
		kfifo_reset(&port->port_write_buf);

	port->port.tty = NULL;

	port->openclose = false;

	pr_debug("%s: ttyGS%d (%p,%p) done!\n",
			__func__, port->port_num, tty, file);

	wake_up(&port->close_wait);
exit:
	spin_unlock_irq(&port->port_lock);
}

static int vpu_write(struct tty_struct *tty,
				const unsigned char *buf, int count)
{
	struct gs_port	*port = tty->driver_data;
	unsigned long	flags;

	pr_vdebug("%s: ttyGS%d (%p) writing %d bytes\n",
			__func__, port->port_num, tty, count);

	spin_lock_irqsave(&port->port_lock, flags);
	/* treat count == 0 as flush_chars() */
	if (port->port_usb)
		vpu_start_tx(port);
	spin_unlock_irqrestore(&port->port_lock, flags);

	return count;
}

static int vpu_put_char(struct tty_struct *tty, unsigned char ch)
{
	struct gs_port	*port = tty->driver_data;
	unsigned long	flags;
	int		status;

	pr_vdebug("%s: (%d,%p) char=0x%x, called from %ps\n",
		__func__, port->port_num, tty, ch, __builtin_return_address(0));

	spin_lock_irqsave(&port->port_lock, flags);
	status = kfifo_put(&port->port_write_buf, ch);
	spin_unlock_irqrestore(&port->port_lock, flags);

	return status;
}

static void vpu_flush_chars(struct tty_struct *tty)
{
	struct gs_port	*port = tty->driver_data;
	unsigned long	flags;

	pr_vdebug("%s: (%d,%p)\n", __func__, port->port_num, tty);

	spin_lock_irqsave(&port->port_lock, flags);
	if (port->port_usb)
		vpu_start_tx(port);
	spin_unlock_irqrestore(&port->port_lock, flags);

}

static int vpu_write_room(struct tty_struct *tty)
{
	struct gs_port	*port = tty->driver_data;
	unsigned long	flags;
	int		room = 0;

	spin_lock_irqsave(&port->port_lock, flags);

	spin_unlock_irqrestore(&port->port_lock, flags);

	pr_vdebug("%s: (%d,%p) room=%d\n",
		__func__, port->port_num, tty, room);

	return room;
}

int vpu_write1(const char *buf, size_t *size, unsigned int timeout_ms)
{
	struct gs_port *port;
	int len = *size;
	int status;
	long jiffies_passed = 0;
	long jiffies_timeout = (long)msecs_to_jiffies(timeout_ms);
	int ret = 0;

	port = ports[PORT_NUM].port;
	*size = 0;
	if (len == 0)
		return -EINVAL;

	ret = wait_event_interruptible_timeout(port->drain_wait,
					vpu_writes_finished(port),
					jiffies_timeout - jiffies_passed);
	if (ret == 0)
		return -ETIME;

	do {
		mutex_lock(&ports[PORT_NUM].lock);
		port = ports[PORT_NUM].port;
		if (!port)
			status = -ENODEV;
		else {
			spin_lock_irq(&port->port_lock);

			/* currently opening/closing? wait ... */
			if (port->openclose) {
				status = -EBUSY;

			/* ... else we do the work */
			} else {
				status = -EAGAIN;
				port->openclose = true;
			}
			spin_unlock_irq(&port->port_lock);
		}
		mutex_unlock(&ports[PORT_NUM].lock);

		switch (status) {
		default:
			/* fully handled */
		case -EAGAIN:
			/* must do the work */
			break;
		case -EBUSY:
			/* REVISIT could have a waitchannel here, if
			 * concurrent open performance is important
			 */
			break;
		}
	} while (status != -EAGAIN);

	/* Do the "real open" */
	spin_lock_irq(&port->port_lock);
	kfifo_initialized(&port->port_write_buf);
	spin_unlock_irq(&port->port_lock);
	status = kfifo_alloc(&port->port_write_buf,
			     WRITE_BUF_SIZE, GFP_KERNEL);
	spin_lock_irq(&port->port_lock);

	/* allocate circular buffer on first open */
	if (!kfifo_initialized(&port->port_write_buf)) {

		spin_unlock_irq(&port->port_lock);
		status = kfifo_alloc(&port->port_write_buf,
				     WRITE_BUF_SIZE, GFP_KERNEL);
		spin_lock_irq(&port->port_lock);

		port->openclose = false;
		goto exit_unlock_port;
	}

	/* if connected, start the I/O stream */
	if (port->port_usb) {
		struct gserial	*gser = port->port_usb;

		vpu_start_io(port);

		if (gser->connect)
			gser->connect(gser);
	}

	status = 0;
	unsigned long	flags;

	spin_unlock_irq(&port->port_lock);
	int		room = 0;

	spin_lock_irqsave(&port->port_lock, flags);
	if (port->port_usb)
		room = kfifo_avail(&port->port_write_buf);
	spin_unlock_irqrestore(&port->port_lock, flags);

	spin_lock_irqsave(&port->port_lock, flags);

	kfifo_in(&port->port_write_buf, buf, len);
	/* treat count == 0 as flush_chars() */
	spin_unlock_irqrestore(&port->port_lock, flags);
	spin_lock_irq(&port->port_lock);
	struct list_head	*pool = &port->write_pool;
	struct usb_ep		*in;

	status = 0;
	bool			do_tty_wake = false;

	if (!port->port_usb)
		return -EINTR;

	in = port->port_usb->in;

	while (!port->write_busy && !list_empty(pool)) {
		struct usb_request	*req;
//		int			len;

		if (port->write_started >= QUEUE_SIZE)
			break;
		req = list_entry(pool->next, struct usb_request, list);

		len = vpu_send_packet(port, req->buf, in->maxpacket);

		if (len == 0) {
			wake_up_interruptible(&port->drain_wait);
			break;
		}
		do_tty_wake = true;

		req->length = len;
		list_del(&req->list);
		req->zero = kfifo_is_empty(&port->port_write_buf);

		/* Drop lock while we call out of driver; completions
		 * could be issued while we do so.  Disconnection may
		 * happen too; maybe immediately before we queue this!
		 *
		 * NOTE that we may keep sending data for a while after
		 * the TTY closed (dev->ioport->port_tty is NULL).
		 */
		port->write_busy = true;
		spin_unlock(&port->port_lock);
		status = usb_ep_queue(in, req, GFP_ATOMIC);
		spin_lock(&port->port_lock);
		port->write_busy = false;

		if (status) {
			list_add(&req->list, pool);
			break;
		}

		port->write_started++;

		/* abort immediately after disconnect */
		if (!port->port_usb)
			break;
	}


	if (port->port_usb)
		vpu_start_tx(port);

exit_unlock_port:
	spin_unlock_irq(&port->port_lock);
	*size = len;
	return 0;
}
EXPORT_SYMBOL_GPL(vpu_write1);

int vpu_close1(int size)
{
	struct gs_port *port;
	struct gserial	*gser;

	port = ports[PORT_NUM].port;
	spin_lock_irq(&port->port_lock);


	/* mark port as closing but in use; we can drop port lock
	 * and sleep if necessary
	 */
	port->openclose = true;


	gser = port->port_usb;
	if (gser && gser->disconnect)
		gser->disconnect(gser);

	/* wait for circular write buffer to drain, disconnect, or at
	 * most GS_CLOSE_TIMEOUT seconds; then discard the rest
	 */
	if (kfifo_len(&port->port_write_buf) > 0 && gser) {
		spin_unlock_irq(&port->port_lock);
		wait_event_interruptible_timeout(port->drain_wait,
					vpu_writes_finished(port),
					GS_CLOSE_TIMEOUT * HZ);
		spin_lock_irq(&port->port_lock);
		gser = port->port_usb;
	}

	/* Iff we're disconnected, there can be no I/O in flight so it's
	 * ok to free the circular buffer; else just scrub it.  And don't
	 * let the push tasklet fire again until we're re-opened.
	 */
	if (gser == NULL)
		kfifo_free(&port->port_write_buf);
	else
		kfifo_reset(&port->port_write_buf);

	port->openclose = false;

	spin_unlock_irq(&port->port_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(vpu_close1);

int vpu_read(void *data, size_t *size, unsigned int timeout_ms)
{
	struct gs_port *port;
	long jiffies_passed = 0;
	long jiffies_timeout = (long)msecs_to_jiffies(timeout_ms);
	int ret = 0;
	size_t len = *size;
	size_t remaining = len;

	port = ports[PORT_NUM].port;
	*size = 0;
	if (len == 0)
		return -EINVAL;

	ret = wait_event_interruptible_timeout(port->drain_wait,
					vpu_writes_finished(port),
					jiffies_timeout - jiffies_passed);
	if (ret == 0)
		return -ETIME;

	struct list_head	*pool = &port->read_pool;
	struct usb_request	*req;

	while (!list_empty(pool)) {
		req = list_entry(pool->next, struct usb_request, list);
		list_del(&req->list);
		size_t bcopy;

		bcopy = min(remaining, req->length);
		memcpy(data, req->buf, bcopy);
		*size = bcopy;
		break;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(vpu_read);

int *vpu_read_swid(void *data)
{
	struct gs_port *port;

	port = ports[PORT_NUM].port;
	struct list_head	*pool = &port->read_pool;
	int *BUF4;
	struct usb_request	*req;

	while (!list_empty(pool)) {
		req = list_entry(pool->next, struct usb_request, list);
		list_del(&req->list);
		BUF4 = req->buf;
		break;
	}
	return BUF4;
}
EXPORT_SYMBOL_GPL(vpu_read_swid);

static int vpu_chars_in_buffer(struct tty_struct *tty)
{
	struct gs_port	*port = tty->driver_data;
	unsigned long	flags;
	int		chars = 0;

	spin_lock_irqsave(&port->port_lock, flags);
	chars = kfifo_len(&port->port_write_buf);
	spin_unlock_irqrestore(&port->port_lock, flags);

	pr_vdebug("gs_chars_in_buffer: (%d,%p) chars=%d\n",
		port->port_num, tty, chars);

	return chars;
}

/* undo side effects of setting TTY_THROTTLED */
static void vpu_unthrottle(struct tty_struct *tty)
{
	struct gs_port		*port = tty->driver_data;
	unsigned long		flags;

	spin_lock_irqsave(&port->port_lock, flags);
	if (port->port_usb) {
		/* Kickstart read queue processing.  We don't do xon/xoff,
		 * rts/cts, or other handshaking with the host, but if the
		 * read queue backs up enough we'll be NAKing OUT packets.
		 */
		pr_vdebug("ttyGS%d: unthrottle\n", port->port_num);
		schedule_delayed_work(&port->push, 0);
	}
	spin_unlock_irqrestore(&port->port_lock, flags);
}

static int vpu_break_ctl(struct tty_struct *tty, int duration)
{
	struct gs_port	*port = tty->driver_data;
	int		status = 0;
	struct gserial	*gser;

	pr_vdebug("%s: ttyGS%d, send break (%d)\n",
			__func__, port->port_num, duration);

	spin_lock_irq(&port->port_lock);
	gser = port->port_usb;
	if (gser && gser->send_break)
		status = gser->send_break(gser, duration);
	spin_unlock_irq(&port->port_lock);

	return status;
}

static const struct tty_operations vpu_tty_ops = {
	.open =			vpu_open,
	.close =		vpu_close,
	.write =		vpu_write,
	.put_char =		vpu_put_char,
	.flush_chars =		vpu_flush_chars,
	.write_room =		vpu_write_room,
	.chars_in_buffer =	vpu_chars_in_buffer,
	.unthrottle =		vpu_unthrottle,
	.break_ctl =		vpu_break_ctl,
};

/*-------------------------------------------------------------------------*/

static struct tty_driver *vpu_tty_driver;

int
vpu_port_alloc(unsigned int port_num, struct usb_cdc_line_coding *coding)
{
	struct gs_port	*port;
	int		ret = 0;

	mutex_lock(&ports[port_num].lock);
	if (ports[port_num].port) {
		ret = -EBUSY;
		goto out;
	}

	port = kzalloc(sizeof(struct gs_port), GFP_KERNEL);
	if (port == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	tty_port_init(&port->port);
	spin_lock_init(&port->port_lock);
	init_waitqueue_head(&port->drain_wait);
	init_waitqueue_head(&port->close_wait);

	INIT_DELAYED_WORK(&port->push, vpu_rx_push);

	INIT_LIST_HEAD(&port->read_pool);
	INIT_LIST_HEAD(&port->read_queue);
	INIT_LIST_HEAD(&port->write_pool);

	port->port_num = port_num;
	port->port_line_coding = *coding;

	ports[port_num].port = port;
out:
	mutex_unlock(&ports[port_num].lock);
	return ret;
}
EXPORT_SYMBOL_GPL(vpu_port_alloc);

static int vpu_closed(struct gs_port *port)
{
	int cond;

	spin_lock_irq(&port->port_lock);
	cond = (port->port.count == 0) && !port->openclose;
	spin_unlock_irq(&port->port_lock);

	return cond;
}

static void vpu_free_port(struct gs_port *port)
{
	cancel_delayed_work_sync(&port->push);

	/* wait for old opens to finish */
	wait_event(port->close_wait, vpu_closed(port));
	WARN_ON(port->port_usb != NULL);
	tty_port_destroy(&port->port);
	kfree(port);
}

void vpu_free_line(unsigned char port_num)
{
	struct gs_port	*port;

	mutex_lock(&ports[port_num].lock);
	if (WARN_ON(!ports[port_num].port)) {
		mutex_unlock(&ports[port_num].lock);
		return;
	}
	port = ports[port_num].port;
	ports[port_num].port = NULL;
	mutex_unlock(&ports[port_num].lock);

	vpu_free_port(port);
	tty_unregister_device(vpu_tty_driver, port_num);

}
EXPORT_SYMBOL_GPL(vpu_free_line);

int vpu_alloc_line(unsigned char *line_num)
{
	struct usb_cdc_line_coding	coding;
	struct device			*tty_dev;
	int				ret;
	int				port_num;

	coding.dwDTERate = cpu_to_le32(9600);
	coding.bCharFormat = 8;
	coding.bParityType = USB_CDC_NO_PARITY;
	coding.bDataBits = USB_CDC_1_STOP_BITS;

	for (port_num = 0; port_num < MAX_U_SERIAL_PORTS; port_num++) {
		ret = vpu_port_alloc(port_num, &coding);
		if (ret == -EBUSY)
			continue;
		if (ret)
			return ret;
		break;
	}
	if (ret)
		return ret;

	/* ... and sysfs class devices, so mdev/udev make /dev/ttyGS* */

	tty_dev = tty_port_register_device(&ports[port_num].port->port,
			vpu_tty_driver, port_num, NULL);
	if (IS_ERR(tty_dev)) {
		struct gs_port	*port;

		pr_err("%s: failed to register tty for port %d, err %ld\n",
				__func__, port_num, PTR_ERR(tty_dev));

		ret = PTR_ERR(tty_dev);
		port = ports[port_num].port;
		ports[port_num].port = NULL;
		vpu_free_port(port);
		goto err;
	}
	*line_num = port_num;

err:
	return ret;
}
EXPORT_SYMBOL_GPL(vpu_alloc_line);

/**
 * vpu_connect - notify TTY I/O glue that USB link is active
 * @gser: the function, set up with endpoints and descriptors
 * @port_num: which port is active
 * Context: any (usually from irq)
 *
 * This is called activate endpoints and let the TTY layer know that
 * the connection is active ... not unlike "carrier detect".  It won't
 * necessarily start I/O queues; unless the TTY is held open by any
 * task, there would be no point.  However, the endpoints will be
 * activated so the USB host can perform I/O, subject to basic USB
 * hardware flow control.
 *
 * Caller needs to have set up the endpoints and USB function in @dev
 * before calling this, as well as the appropriate (speed-specific)
 * endpoint descriptors, and also have allocate @port_num by calling
 * @vpu_alloc_line().
 *
 * Returns negative errno or zero.
 * On success, ep->driver_data will be overwritten.
 */

int vpu_connect(struct gserial *gser, u8 port_num)
{
	struct gs_port	*port;
	unsigned long	flags;
	int		status;

	if (port_num >= MAX_U_SERIAL_PORTS)
		return -ENXIO;

	PORT_NUM = port_num;
	port = ports[port_num].port;
	if (!port) {
		pr_err("serial line %d not allocated.\n", port_num);
		return -EINVAL;
	}
	if (port->port_usb) {
		pr_err("serial line %d is in use.\n", port_num);
		return -EBUSY;
	}

	/* activate the endpoints */
	status = usb_ep_enable(gser->in);
	if (status < 0)
		return status;
	gser->in->driver_data = port;

	status = usb_ep_enable(gser->out);
	if (status < 0)
		goto fail_out;
	gser->out->driver_data = port;

	/* then tell the tty glue that I/O can work */
	spin_lock_irqsave(&port->port_lock, flags);
	gser->ioport = port;
	port->port_usb = gser;

	/* REVISIT unclear how best to handle this state...
	 * we don't really couple it with the Linux TTY.
	 */
	gser->port_line_coding = port->port_line_coding;

	vpu_start_io(port);
	spin_unlock_irqrestore(&port->port_lock, flags);

	return 0;

fail_out:
	usb_ep_disable(gser->in);
	return 0;
}
EXPORT_SYMBOL_GPL(vpu_connect);

/**
 * vpu_disconnect - notify TTY I/O glue that USB link is inactive
 * @gser: the function, on which vpu_connect() was called
 * Context: any (usually from irq)
 *
 * This is called to deactivate endpoints and let the TTY layer know
 * that the connection went inactive ... not unlike "hangup".
 *
 * On return, the state is as if vpu_connect() had never been called;
 * there is no active USB I/O on these endpoints.
 */

void vpu_disconnect(struct gserial *gser)
{
	struct gs_port	*port = gser->ioport;
	unsigned long	flags;

	if (!port)
		return;

	/* tell the TTY glue not to do I/O here any more */
	spin_lock_irqsave(&port->port_lock, flags);

	/* REVISIT as above: how best to track this? */
	port->port_line_coding = gser->port_line_coding;

	port->port_usb = NULL;
	gser->ioport = NULL;
	if (port->port.count > 0 || port->openclose) {
		wake_up_interruptible(&port->drain_wait);
		if (port->port.tty)
			tty_hangup(port->port.tty);
	}
	spin_unlock_irqrestore(&port->port_lock, flags);

	/* disable endpoints, aborting down any active I/O */
	usb_ep_disable(gser->out);
	usb_ep_disable(gser->in);

	/* finally, free any unused/unusable I/O buffers */
	spin_lock_irqsave(&port->port_lock, flags);
	if (port->port.count == 0 && !port->openclose)
		kfifo_free(&port->port_write_buf);
	vpu_free_requests(gser->out, &port->read_pool, NULL);
	vpu_free_requests(gser->out, &port->read_queue, NULL);
	vpu_free_requests(gser->in, &port->write_pool, NULL);

	port->read_allocated = port->read_started =
		port->write_allocated = port->write_started = 0;

	spin_unlock_irqrestore(&port->port_lock, flags);
}
EXPORT_SYMBOL_GPL(vpu_disconnect);

static int uxlink_init(void)
{
	unsigned int	i;
	int				status;

	vpu_tty_driver = alloc_tty_driver(MAX_U_SERIAL_PORTS);
	if (!vpu_tty_driver)
		return -ENOMEM;

	vpu_tty_driver->driver_name = "g_serial";
	vpu_tty_driver->name = "ttyGS";
	/* uses dynamically assigned dev_t values */

	vpu_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	vpu_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	vpu_tty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	vpu_tty_driver->init_termios = tty_std_termios;

	/* 9600-8-N-1 ... matches defaults expected by "usbser.sys" on
	 * MS-Windows.  Otherwise, most of these flags shouldn't affect
	 * anything unless we were to actually hook up to a serial line.
	 */
	vpu_tty_driver->init_termios.c_cflag =
			B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	vpu_tty_driver->init_termios.c_ispeed = 9600;
	vpu_tty_driver->init_termios.c_ospeed = 9600;

	tty_set_operations(vpu_tty_driver, &vpu_tty_ops);
	for (i = 0; i < MAX_U_SERIAL_PORTS; i++)
		mutex_init(&ports[i].lock);

	/* export the driver ... */
	status = tty_register_driver(vpu_tty_driver);
	if (status) {
		pr_err("%s: cannot register, err %d\n",
				__func__, status);
		goto fail;
	}

	return status;
fail:
	put_tty_driver(vpu_tty_driver);
	vpu_tty_driver = NULL;
	return status;

}
module_init(uxlink_init);

static void uxlink_cleanup(void)
{
	tty_unregister_driver(vpu_tty_driver);
	put_tty_driver(vpu_tty_driver);
	vpu_tty_driver = NULL;

}
module_exit(uxlink_cleanup);

MODULE_LICENSE("GPL");
