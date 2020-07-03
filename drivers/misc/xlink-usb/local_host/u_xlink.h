/* SPDX-License-Identifier: GPL-2.0 only */
/*
 * u_xlink.h - interface to USB gadget "serial port"/TTY utilities
 *
 * Copyright (C) 2020 Intel Corporation
 *
 */

#ifndef __U_SERIAL_H
#define __U_SERIAL_H

#include <linux/usb/composite.h>
#include <linux/usb/cdc.h>

#define MAX_U_SERIAL_PORTS	4

struct f_serial_opts {
	struct usb_function_instance func_inst;
	u8 port_num;
};

/*
 * One non-multiplexed "serial" I/O port ... there can be several of these
 * on any given USB peripheral device, if it provides enough endpoints.
 *
 * The "u_serial" utility component exists to do one thing:  manage TTY
 * style I/O using the USB peripheral endpoints listed here, including
 * hookups to sysfs and /dev for each logical "tty" device.
 *
 * REVISIT at least ACM could support tiocmget() if needed.
 *
 * REVISIT someday, allow multiplexing several TTYs over these endpoints.
 */
struct gserial {
	struct usb_function		func;

	/* port is managed by gserial_{connect,disconnect} */
	struct gs_port			*ioport;

	struct usb_ep			*in;
	struct usb_ep			*out;

	/* REVISIT avoid this CDC-ACM support harder ... */
	struct usb_cdc_line_coding port_line_coding;	/* 9600-8-N-1 etc */

	/* notification callbacks */
	void (*connect)(struct gserial *p);
	void (*disconnect)(struct gserial *p);
	int (*send_break)(struct gserial *p, int duration);
};

/* utilities to allocate/free request and buffer */
struct usb_request *vpu_alloc_req(struct usb_ep *ep,
		unsigned int len, gfp_t flags);
void vpu_free_req(struct usb_ep *ep, struct usb_request *req);
void vpu_read_complete(struct usb_ep *ep, struct usb_request *req);
unsigned int vpu_start_rx(struct gs_port *port);
int vpu_alloc_requests(struct usb_ep *ep, struct list_head *head,
		void (*fn)(struct usb_ep *, struct usb_request *),
		int *allocated);
int vpu_port_alloc(unsigned int port_num, struct usb_cdc_line_coding *coding);
//int vpu_open(struct tty_struct *tty, struct file *file);
/* management of individual TTY ports */
int vpu_alloc_line(unsigned char *port_line);
void vpu_free_line(unsigned char port_line);

/* connect/disconnect is handled by individual functions */
int vpu_connect(struct gserial *gser, u8 port_num);
void vpu_disconnect(struct gserial *gser);
int vpu_start_io(struct gs_port *port);

int vpu_read(void *data, size_t *size, unsigned int timeout_ms);
int *vpu_read_swid(void *data);
int vpu_write1(const char *buf, size_t *size, unsigned int timeout_ms);
int vpu_close1(int size);
int usb_sw_id(int pid);
int usb_get_by_id(uint32_t sw_device_id);

/* functions are bound to configurations by a config or gadget driver */
int gser_bind_config(struct usb_configuration *c, u8 port_num);
int obex_bind_config(struct usb_configuration *c, u8 port_num);

#endif /* __U_SERIAL_H */
