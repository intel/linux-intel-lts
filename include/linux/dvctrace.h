/*
 * DvC.Trace(dvct) Bus
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

#ifndef __DVCTRACE_H
#define __DVCTRACE_H

#include <linux/usb/gadget.h>

/**
 * The function is binded to a gadget
 *	function-driver RW
 *	client-driver   RO
 */
#define DVCT_MASK_ONLINE   BIT(0)

/**
 * The client is currently transferring
 *	function-driver RO
 *	client-driver   RW
 */
#define DVCT_MASK_TRANS    BIT(1)

/**
 * An error was detected the client
 *	function-driver RO
 *	client-driver   RW
 */
#define DVCT_MASK_ERR      BIT(2)

#define DVCT_MASK_ONLINE_TRANS (DVCT_MASK_ONLINE | DVCT_MASK_TRANS)
#define DVCT_MASK_ALL (DVCT_MASK_ONLINE | DVCT_MASK_TRANS | DVCT_MASK_ERR)

/**
 * Bitwise status manipulation macros
 */
#define dvct_set_status(s, m) atomic_set(s, atomic_read(s)|(m))
#define dvct_clr_status(s, m) atomic_set(s, atomic_read(s) & (~(m)))
#define dvct_get_status(s, m) (atomic_read(s) & (m))

/**
 * dvct_string_lookup - helper struct to better manage the string id allocation
 * @str: the usb string to allocate the id for
 * @id:	the location in the descriptor where the id should be updated
 */
struct dvct_string_lookup {
	struct usb_string *str;
	u8 *id;			/*points to the location of the string id */
};

/**
 * dvctrace_usb_descriptors - holds DvC Trace usb descriptors information
 * @dvc_spec: Dvc Trace specific descriptors, will be added in the function
 *	usb descriptors.
 * @str: Usb gadget string table for strings referenced in @dvc_spec.
 * @lk_tbl: Strings lookup table.
 *
 * From a strict USB point of view the descriptors are optional so @dvc_spec,
 * @str.strings and @lk_tbl can be null. However the dvc-trace host
 * implementation might need them.
 */
struct dvct_usb_descriptors {
	struct usb_descriptor_header **dvc_spec; /*null terminated*/
	struct usb_gadget_strings str;
	struct dvct_string_lookup *lk_tbl;/*null terminated*/
};

/**
 * dvct_source_device - DvC Trace function to source (backend) interface
 *
 * @name_add: postfix used to compute the source name (<driver_name>-<name_add>)
 * @protocol: Interface protocol code (used in IAD, control and data
 * @static_desc: DvC-Trace specific usb descriptors, provided by the driver
 *			interface descriptors)
 *
 * @instance_taken, @function_taken, @lock, @device, @desc are reserved for
 * internal use only.
 */
struct dvct_source_device {
	const char *name_add;
	u8 protocol;
	struct dvct_usb_descriptors static_desc;
	u32 instance_taken:1;
	u32 function_taken:1;
	spinlock_t lock;
	struct device device;
	struct dvct_usb_descriptors *desc;
};

/**
 * dvct_source_driver - DvC Trace source (backend) representation
 * @activate: pre-bind callback called when the function instance linked to the
 *	source device is selected as a part of a configuration eg.
 *	ln -s cfgfs/.../functions/dvctrace.x  cfgfs/.../configs/c.x/
 *	atomic_t * , points to the status field of the function, the client
 *	should update relevant bits (DVCT_MASK_TRANS, DVCT_MASK_ERR).
 * @binded: post-bind callback, at this stage the transfer endpoint is
 *	allocated;
 * @connected: (Optional) The gadget is now connected to a host, the connection
 *	speed is available.
 * @start_transfer: called upon receiving SET_TRACE, the host side should be
 *	able ready to receive data, the client could submit usb requests.
 * @stop_transfer: host side request to stop the data transfer.
 * @disconnected: (Optional) The gadget was disconnected from the host.
 *	Before calling this the function driver will call stop_trasfer if
 *	DVCT_MASK_TRANS is set.
 * @unbinded: The USB-function is no longer used.
 * @deactivate: The USB-function is no longer part of a configuration.
 *
 * If any of the non optional callbacks is not provided (NULL) driver
 * registration will fail.
 */
struct dvct_source_driver {
	struct device_driver driver;
	int (*activate)(struct dvct_source_device *, atomic_t *);
	int (*binded)(struct dvct_source_device *, struct usb_ep *,
				 struct usb_function *);
	void (*connected)(struct dvct_source_device *, enum usb_device_speed);
	int (*start_transfer)(struct dvct_source_device *, u8);
	int (*stop_transfer)(struct dvct_source_device *);
	void (*disconnected)(struct dvct_source_device *);
	void (*unbinded)(struct dvct_source_device *);
	void (*deactivate)(struct dvct_source_device *);
};

#define dev_to_dvct_source_device(ptr) \
	container_of(ptr, struct dvct_source_device, device)

#define drv_to_dvct_source_driver(ptr) \
	container_of(ptr, struct dvct_source_driver, driver)

extern struct dvct_source_driver
*dvct_source_get_drv(struct dvct_source_device *dev);

#define dvct_source_driver_register(drv) \
	__dvct_source_driver_register(drv, THIS_MODULE)
extern int __dvct_source_driver_register(struct dvct_source_driver *,
					 struct module *);
extern void dvct_source_driver_unregister(struct dvct_source_driver *);
extern int dvct_source_device_add(struct dvct_source_device *,
				  struct dvct_source_driver *);
extern void dvct_source_device_del(struct dvct_source_device *);

extern struct dvct_source_device *dvct_source_find_by_name(const char *name);

extern struct dvct_source_device
*dvct_source_find_free_by_name(const char *name);

extern struct dvct_source_device *dvct_source_find_free(void);

extern int dvct_source_count_free(void);

#endif /* __DVCTRACE_H*/
