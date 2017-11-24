/*
 * Copyright (c) 2016, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/aio.h>
#include <linux/pci.h>
#include <linux/poll.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/version.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/uuid.h>
#include <linux/compat.h>
#include <linux/jiffies.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/circ_buf.h>

#include "ishtp/ishtp-dev.h"
#include "ishtp/client.h"
#include "ishtp-tty.h"


/* Rx ring buffer pool size */
#define TTY_CL_RX_RING_SIZE	32
#define TTY_CL_TX_RING_SIZE	16

#define ISH_TTY_MAJOR 		267

#define ISH_BUF_SIZE		256

struct ttyish {
	struct tty_port port;
	struct device *dev;
	uint32_t baud;
	uint8_t bits_length;
} ttyish_dev;

struct tty_driver *ish_tty_driver;

void __iomem *ish_membase;


struct ishtp_cl	*tty_ishtp_cl;	/* ISS HECI TTY client */
wait_queue_head_t	ishtp_tty_wait;

int	get_report_done;	/* Get Feature/Input report complete flag */
static int max_msg_size;

static int ishtp_wait_for_response(void)
{
	if (!get_report_done)
		wait_event_timeout(ishtp_tty_wait, get_report_done, 3 * HZ);

	if (!get_report_done) {
		pr_err("Timeout waiting for response from HECI device\n");
		return -1;
	}

	get_report_done = 0;

	return 0;
}

/* HECI client driver structures and API for bus interface */
void process_recv(void *recv_buf, size_t data_len)
{
	struct ishtp_tty_msg *ishtp_msg;
	struct ttyish *tp = &ttyish_dev;
	struct uart_config *cfg;
	unsigned char	*payload;
	size_t payload_len, total_len, cur_pos;

	ISH_DBG_PRINT(KERN_ALERT "[ish-tty]: %s():+++ len=%u\n", __func__,
		(unsigned)data_len);

	if (data_len < sizeof(struct ishtp_tty_msg_hdr)) {
		dev_err(NULL, "[ish-tty]: error, received %u which is ",
			(unsigned)data_len);
		dev_err(NULL, " less than data header %u\n",
			(unsigned)sizeof(struct ishtp_tty_msg_hdr));
		return;
	}

	payload = recv_buf + sizeof(struct ishtp_tty_msg);
	total_len = data_len;
	cur_pos = 0;

	do {
		ishtp_msg = (struct ishtp_tty_msg *)(recv_buf + cur_pos);
		payload_len = ishtp_msg->hdr.size;

		switch (ishtp_msg->hdr.command & CMD_MASK) {
		default:
			break;

		case UART_GET_CONFIG:
			if (!(ishtp_msg->hdr.command & IS_RESPONSE) || (ishtp_msg->hdr.status)) {
				pr_err("recv command with status error\n");
				//error handler
				get_report_done = 1;
				if (waitqueue_active(&ishtp_tty_wait))
					wake_up(&ishtp_tty_wait);
				break;
			}


			cfg = (struct uart_config *)payload;
			tp->baud = cfg->baud;
			tp->bits_length = cfg->bits_length;

			ISH_DBG_PRINT(KERN_ALERT "[ish-tty] command: get config: %d:%d\n", tp->baud, tp->bits_length);

			get_report_done = 1;
			if (waitqueue_active(&ishtp_tty_wait))
				wake_up(&ishtp_tty_wait);
			break;
		case UART_SET_CONFIG:
			if (!(ishtp_msg->hdr.command & IS_RESPONSE) || (ishtp_msg->hdr.status)) {
				pr_err("[ish-tty] recv command with status error\n");
				//error handler
				get_report_done = 1;
				if (waitqueue_active(&ishtp_tty_wait))
					wake_up(&ishtp_tty_wait);
				break;
			}

			ISH_DBG_PRINT(KERN_ALERT "[ish-tty] command: set config success\n");

			get_report_done = 1;
			if (waitqueue_active(&ishtp_tty_wait))
				wake_up(&ishtp_tty_wait);
			break;
		case UART_SEND_DATA:
			if (!(ishtp_msg->hdr.command & IS_RESPONSE) || (ishtp_msg->hdr.status)) {
				pr_err("[ish-tty] recv command with status error\n");
				//error handler
				get_report_done = 1;
				if (waitqueue_active(&ishtp_tty_wait))
					wake_up(&ishtp_tty_wait);
				break;
			}

			ISH_DBG_PRINT(KERN_ALERT "[ish-tty] command: send data done\n");
			get_report_done = 1;
			if (waitqueue_active(&ishtp_tty_wait))
				wake_up(&ishtp_tty_wait);
			break;
		case UART_RECV_DATA:
			ISH_DBG_PRINT(KERN_ALERT "[ish-tty] command: recv data: len=%d\n", payload_len);
			print_hex_dump(KERN_INFO, "", DUMP_PREFIX_NONE, 16, 1, payload,
					payload_len, true);
			tty_insert_flip_string(&tp->port, payload, payload_len);
			tty_flip_buffer_push(&tp->port);

			break;
		case UART_ABORT_WRITE:
			get_report_done = 1;
			if (waitqueue_active(&ishtp_tty_wait))
				wake_up(&ishtp_tty_wait);
			break;
		case UART_ABORT_READ:
			get_report_done = 1;
			if (waitqueue_active(&ishtp_tty_wait))
				wake_up(&ishtp_tty_wait);
			break;
		}

		cur_pos += payload_len + sizeof(struct ishtp_tty_msg);
		payload += payload_len + sizeof(struct ishtp_tty_msg);

	} while (cur_pos < total_len);

}

static void tty_ishtp_cl_event_cb(struct ishtp_cl_device *device)
{
	size_t r_length;
	struct ishtp_cl_rb *rb_in_proc;
	unsigned long	flags;

	ISH_DBG_PRINT(KERN_ALERT "%s() +++\n", __func__);

	if (!tty_ishtp_cl)
		return;

	spin_lock_irqsave(&tty_ishtp_cl->in_process_spinlock, flags);
	while (!list_empty(&tty_ishtp_cl->in_process_list.list)) {
		rb_in_proc = list_entry(tty_ishtp_cl->in_process_list.list.next,
			struct ishtp_cl_rb, list);
		list_del_init(&rb_in_proc->list);
		spin_unlock_irqrestore(&tty_ishtp_cl->in_process_spinlock,
			flags);

		if (!rb_in_proc->buffer.data) {
			ISH_DBG_PRINT(KERN_ALERT
				"%s(): !rb_in_proc-->buffer.data, something's wrong\n",
				__func__);
			return;
		}
		r_length = rb_in_proc->buf_idx;
		ISH_DBG_PRINT(KERN_ALERT
			"%s(): OK received buffer of %u length\n", __func__,
			(unsigned)r_length);

		/* decide what to do with received data */
		process_recv(rb_in_proc->buffer.data, r_length);

		ishtp_cl_io_rb_recycle(rb_in_proc);
		spin_lock_irqsave(&tty_ishtp_cl->in_process_spinlock, flags);
	}
	spin_unlock_irqrestore(&tty_ishtp_cl->in_process_spinlock, flags);
}

static int ish_tty_install(struct tty_driver *driver, struct tty_struct *tty)
{
	struct ttyish *tp;
	int ret = -ENODEV;

	dev_info(tty->dev, "[%d]%s index:%d\n", __LINE__, __func__, tty->index);

	tp = &ttyish_dev;

	ret = tty_port_install(&tp->port, driver, tty);

	tty->driver_data = tp;

	return ret;
}

static void ish_tty_cleanup(struct tty_struct *tty)
{

	dev_info(tty->dev, "[%d]%s index:%d\n", __LINE__, __func__, tty->index);
}

static int ish_tty_open(struct tty_struct *tty, struct file *filp)
{
	struct ttyish *tp = tty->driver_data;
	int ret = 0;

	dev_info(tty->dev, "[%d]%s index:%d\n", __LINE__, __func__, tty->index);

	ret = tty_port_open(&tp->port, tty, filp);

	return ret;
}

static void ish_tty_close(struct tty_struct *tty, struct file *filp)
{
	struct ttyish *tp = tty->driver_data;

	dev_info(tty->dev, "%s index:%d\n", __func__, tty->index);

	tty_port_close(&tp->port, tty, filp);
}

static int ish_tty_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
	unsigned char *ishtp_buf;
	struct ishtp_tty_msg *ishtp_msg;
	unsigned char *msg_buf;
	int c, ret = 0;

	dev_dbg(tty->dev, "%s: len=%d\n", __func__, count);
	ishtp_buf = kzalloc(sizeof(struct ishtp_tty_msg) + max_msg_size, GFP_KERNEL);
	ishtp_msg = (struct ishtp_tty_msg *)ishtp_buf;
	if (!ishtp_msg)
		return -ENOMEM;
	ishtp_msg->hdr.command = UART_SEND_DATA;
	msg_buf = (unsigned char *)&ishtp_msg[1];

	uint32_t max_msg_payload_size = max_msg_size -
					sizeof(struct ishtp_tty_msg_hdr);
	while (count > 0) {
		c = count;
		if (c > max_msg_payload_size)
			c = max_msg_payload_size;

		ishtp_msg->hdr.size = c;
		memcpy(msg_buf, buf, c);

		/* heci message send out */
		ishtp_cl_send(tty_ishtp_cl, ishtp_buf, sizeof(struct ishtp_tty_msg) + c);

		buf += c;
		count -= c;
		ret += c;

		/* wait for message send completely */
		if (ishtp_wait_for_response() < 0) {
			kfree(ishtp_buf);
			return -ETIMEDOUT;
		}
	}

	dev_dbg(tty->dev, "%s: done\n", __func__);
	kfree(ishtp_buf);
	return ret;
}

static void ish_tty_set_termios(struct tty_struct *tty,
					struct ktermios *old_termios)
{
	static unsigned char ishtp_buf[10];
	struct ishtp_tty_msg *ishtp_msg;
	struct ktermios *termios;
	struct uart_config *cfg;
	unsigned int baud;

	dev_info(tty->dev, "[%d]%s index:%d\n", __LINE__, __func__, tty->index);

	if (old_termios && !tty_termios_hw_change(&tty->termios, old_termios))
		return;

	termios = &tty->termios;
	ishtp_msg = (struct ishtp_tty_msg *)ishtp_buf;

	ishtp_msg->hdr.command = UART_SET_CONFIG;
	cfg = (struct uart_config *)&ishtp_msg[1];

	switch (C_CSIZE(tty)) {
	case CS5:
		cfg->bits_length = 5;
	case CS6:
		cfg->bits_length = 6;
	case CS7:
		cfg->bits_length = 7;
	default:
	case CS8:
		cfg->bits_length = 8;
	}

	if (C_CRTSCTS(tty))
		cfg->flow_control = true;
	else
		cfg->flow_control = false;

	baud = tty_termios_baud_rate(termios);

	if ((baud != 9600) && (baud != 57600) && (baud != 19200) && (baud != 38400) &&
	    (baud != 115200) && (baud != 921600) && (baud != 2000000) && (baud != 3000000) &&
		(baud != 3250000) && (baud != 3500000) && (baud != 4000000))
		dev_info(tty->dev, "%s: baud[%d] is not supported\n", __func__, baud);

	cfg->baud = baud;

	ishtp_msg->hdr.size = sizeof(struct uart_config);

	/* heci message send out */
	ishtp_cl_send(tty_ishtp_cl, ishtp_buf, sizeof(struct ishtp_tty_msg) + ishtp_msg->hdr.size);

	/* wait for message send completely */
	if (ishtp_wait_for_response() < 0)
		return;
}

static int ish_tty_write_room(struct tty_struct *tty)
{
	int space_left;

	space_left = max_msg_size;

	dev_info(tty->dev, "%s index:%d, space_left:%d\n", __func__, tty->index, space_left);

	return space_left;
}

static int ish_tty_chars_in_buffer(struct tty_struct *tty)
{
	dev_info(tty->dev, "[%d]%s index:%d\n", __LINE__, __func__, tty->index);

	return 0;
}

static void ish_tty_unthrottle(struct tty_struct *tty)
{
	dev_info(tty->dev, "[%d]%s index:%d\n", __LINE__, __func__, tty->index);
}

static struct tty_operations ish_tty_ops = {
	.install = ish_tty_install,
	.cleanup = ish_tty_cleanup,
	.open = ish_tty_open,
	.close = ish_tty_close,
	.write = ish_tty_write,
	.set_termios	= ish_tty_set_termios,
	.write_room = ish_tty_write_room,
	.chars_in_buffer = ish_tty_chars_in_buffer,
	.unthrottle = ish_tty_unthrottle,
};

static int ishtp_tty_port_activate(struct tty_port *port, struct tty_struct *tty)
{
	return 0;
}

static void ishtp_tty_port_shutdown(struct tty_port *tport)
{
	return;
}

static void ishtp_tty_port_dtr_rts(struct tty_port *port, int on)
{
	return;
}

static int ishtp_tty_port_carrier_raised(struct tty_port *port)
{
	return 1;
}

static const struct tty_port_operations ishtp_tty_port_ops = {
	.activate	= ishtp_tty_port_activate,
	.shutdown	= ishtp_tty_port_shutdown,
	.carrier_raised = ishtp_tty_port_carrier_raised,
	.dtr_rts	= ishtp_tty_port_dtr_rts,
};

static int tty_ishtp_cl_probe(struct ishtp_cl_device *cl_device)
{
	struct ishtp_device *dev;
	struct tty_port *port;
	int i, rv;

	ISH_DBG_PRINT(KERN_ALERT "%s(): +++\n", __func__);
	if (!cl_device)
		return	-ENODEV;

	if (uuid_le_cmp(tty_ishtp_guid,
			cl_device->fw_client->props.protocol_name) != 0) {
		ISH_DBG_PRINT(KERN_ALERT "%s(): device doesn't match\n",
			__func__);
		return	-ENODEV;
	}

	ISH_DBG_PRINT(KERN_ALERT "%s(): device matches!\n", __func__);
	tty_ishtp_cl = ishtp_cl_allocate(cl_device->ishtp_dev);
	if (!tty_ishtp_cl)
		return	-ENOMEM;

	rv = ishtp_cl_link(tty_ishtp_cl, ISHTP_HOST_CLIENT_ID_ANY);
	if (rv) {
		dev_err(&cl_device->dev, "ishtp_cl_link failed\n");
		return	-ENOMEM;
	}

	dev = tty_ishtp_cl->dev;

	/* Connect to FW client */
	tty_ishtp_cl->rx_ring_size = TTY_CL_RX_RING_SIZE;
	tty_ishtp_cl->tx_ring_size = TTY_CL_TX_RING_SIZE;

	i = ishtp_fw_cl_by_uuid(dev, &tty_ishtp_guid);
	tty_ishtp_cl->fw_client_id = dev->fw_clients[i].client_id;
	tty_ishtp_cl->state = ISHTP_CL_CONNECTING;

	rv = ishtp_cl_connect(tty_ishtp_cl);
	if (rv) {
		dev_err(&cl_device->dev, "client connect failed\n");
		return rv;
	}

	/* Register read callback */
	ishtp_register_event_cb(tty_ishtp_cl->device, tty_ishtp_cl_event_cb);

	init_waitqueue_head(&ishtp_tty_wait);

	/* register tty device */
	port = &ttyish_dev.port;
	tty_port_init(port);
	port->ops = &ishtp_tty_port_ops;

	ttyish_dev.dev = tty_port_register_device(port, ish_tty_driver, 0,
			&tty_ishtp_cl->device->dev);

	max_msg_size = cl_device->fw_client->props.max_msg_length;

	ISH_DBG_PRINT(KERN_ALERT "%s(): ---\n", __func__);
	return	0;
}

static int tty_ishtp_cl_remove(struct ishtp_cl_device *dev)
{
	ISH_DBG_PRINT(KERN_ALERT "%s(): +++\n", __func__);
	tty_unregister_device(ish_tty_driver, 0);

	tty_ishtp_cl = NULL;

	ISH_DBG_PRINT(KERN_ALERT "%s(): ---\n", __func__);
	return  0;
}


struct ishtp_cl_driver	tty_ishtp_cl_driver = {
	.name = "ish_tty",
	.probe = tty_ishtp_cl_probe,
	.remove = tty_ishtp_cl_remove,
};

static int __init ish_tty_init(void)
{
	struct tty_driver *tty_driver;
	int ret;

	pr_info("[%d]%s\n", __LINE__, __func__);

	tty_driver = alloc_tty_driver(1);
	if (!tty_driver) {
		pr_err("%s(%d): Memory allocation failed\n", __func__, __LINE__);
		return -ENOMEM;
	}

	ish_tty_driver = tty_driver;

	tty_driver->owner = THIS_MODULE;
	tty_driver->driver_name = "ISH-UART";
	tty_driver->name = "ttyISH";
	tty_driver->major = ISH_TTY_MAJOR;
	tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	tty_driver->subtype = SERIAL_TYPE_NORMAL;
	tty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;

	tty_driver->init_termios = tty_std_termios;
	tty_driver->init_termios.c_cflag = B115200 | CS8 | HUPCL | CLOCAL;
	tty_driver->init_termios.c_lflag = ISIG | ICANON | IEXTEN;
	tty_set_operations(tty_driver, &ish_tty_ops);

	pr_info("[%d]%s\n", __LINE__, __func__);

	ret = tty_register_driver(tty_driver);
	if (ret) {
		put_tty_driver(tty_driver);
		return ret;
	}

	ret = ishtp_cl_driver_register(&tty_ishtp_cl_driver);

	return 0;
}

static void  __exit ish_tty_exit(void)
{
	int ret;

	ishtp_cl_driver_unregister(&tty_ishtp_cl_driver);

	ret = tty_unregister_driver(ish_tty_driver);
	if (ret < 0)
		printk(KERN_ERR "ish_tty_driver: tty_unregister_driver failed, %d\n", ret);
	else
		put_tty_driver(ish_tty_driver);
	ish_tty_driver = NULL;
}

late_initcall(ish_tty_init);
module_exit(ish_tty_exit);

MODULE_DESCRIPTION("Driver of Simulated UART on Sensor Hub");
MODULE_AUTHOR("Intel Corporation");
MODULE_AUTHOR("Even Xu <even.xu@intel.com");

MODULE_LICENSE("GPL");
MODULE_ALIAS("ishtp:*");
