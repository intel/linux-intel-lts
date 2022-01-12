// SPDX-License-Identifier: GPL-2.0-only
/*
 * xlink Core Driver.
 *
 * Copyright (C) 2018-2019 Intel Corporation
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/mod_devicetable.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/kref.h>

#include "xlink-defs.h"
#include "xlink-core.h"
#include "xlink-ioctl.h"

#define CHANNEL_SET_USER_BIT(chan) ((chan) |= (1 << 15))

static int copy_result_to_user(u32 *where, int rc)
{
	if (copy_to_user((void __user *)where, &rc, sizeof(rc)))
		return -EFAULT;
	return rc;
}

int ioctl_connect(unsigned long arg)
{
	struct xlink_handle	devh;
	struct xlinkconnect	con;
	int rc;

	if (copy_from_user(&con, (void __user *)arg,
			   sizeof(struct xlinkconnect)))
		return -EFAULT;
	if (copy_from_user(&devh, (void __user *)con.handle,
			   sizeof(struct xlink_handle)))
		return -EFAULT;
	rc = xlink_connect(&devh);
	if (rc == X_LINK_SUCCESS) {
		if (copy_to_user((void __user *)con.handle,
				 &devh, sizeof(struct xlink_handle)))
			return -EFAULT;
	}

	return copy_result_to_user(con.return_code, rc);
}

int ioctl_open_channel(unsigned long arg)
{
	struct xlink_handle	devh;
	struct xlinkopenchannel	op;
	int rc;

	if (copy_from_user(&op, (void __user *)arg,
			   sizeof(struct xlinkopenchannel)))
		return -EFAULT;
	if (copy_from_user(&devh, (void __user *)op.handle,
			   sizeof(struct xlink_handle)))
		return -EFAULT;
	rc = xlink_open_channel(&devh, op.chan, op.mode, op.data_size,
				op.timeout);

	return copy_result_to_user(op.return_code, rc);
}

int ioctl_read_data(unsigned long arg)
{
	struct xlink_handle	devh;
	struct xlinkreaddata	rd;
	int interface;
	u8 *rdaddr;
	u32 size;
	int rc;

	if (copy_from_user(&rd, (void __user *)arg,
			   sizeof(struct xlinkreaddata)))
		return -EFAULT;
	if (copy_from_user(&devh, (void __user *)rd.handle,
			   sizeof(struct xlink_handle)))
		return -EFAULT;
	rc = xlink_read_data(&devh, rd.chan, &rdaddr, &size);
	if (!rc) {
		interface = get_interface_from_sw_device_id(devh.sw_device_id);
		if (interface == IPC_INTERFACE) {
			if (copy_to_user((void __user *)rd.pmessage, (void *)&rdaddr,
					 sizeof(u32)))
				return -EFAULT;
		} else {
			if (copy_to_user((void __user *)rd.pmessage, (void *)rdaddr,
					 size))
				return -EFAULT;
		}
		if (copy_to_user((void __user *)rd.size, (void *)&size, sizeof(size)))
			return -EFAULT;
	}

	return copy_result_to_user(rd.return_code, rc);
}

int ioctl_read_to_buffer(unsigned long arg)
{
	struct xlinkreadtobuffer	rdtobuf;
	struct xlink_handle		devh;
	u8 volbuf[XLINK_MAX_BUF_SIZE]; // buffer for volatile transactions
	u32 size;
	int rc;

	if (copy_from_user(&rdtobuf, (void __user *)arg,
			   sizeof(struct xlinkreadtobuffer)))
		return -EFAULT;
	if (copy_from_user(&devh, (void __user *)rdtobuf.handle,
			   sizeof(struct xlink_handle)))
		return -EFAULT;
	rc = xlink_read_data_to_buffer(&devh, rdtobuf.chan,
				       (u8 *)volbuf, &size);
	if (!rc) {
		if (copy_to_user((void __user *)rdtobuf.pmessage, (void *)volbuf,
				 size))
			return -EFAULT;
		if (copy_to_user((void __user *)rdtobuf.size, (void *)&size,
				 sizeof(size)))
			return -EFAULT;
	}

	return copy_result_to_user(rdtobuf.return_code, rc);
}

int ioctl_write_data(unsigned long arg)
{
	struct xlink_handle	devh;
	struct xlinkwritedata	wr;
	int rc;

	if (copy_from_user(&wr, (void __user *)arg,
			   sizeof(struct xlinkwritedata)))
		return -EFAULT;
	if (copy_from_user(&devh, (void __user *)wr.handle,
			   sizeof(struct xlink_handle)))
		return -EFAULT;
	if (wr.size > XLINK_MAX_DATA_SIZE)
		return -EFAULT;
	rc = xlink_write_data_user(&devh, wr.chan, wr.pmessage, wr.size);

	return copy_result_to_user(wr.return_code, rc);
}

int ioctl_write_control_data(unsigned long arg)
{
	u8 volbuf[XLINK_MAX_CONTROL_DATA_PCIE_SIZE];
	struct xlink_handle	devh;
	struct xlinkwritedata	wr;
	int rc;

	if (copy_from_user(&wr, (void __user *)arg,
			   sizeof(struct xlinkwritedata)))
		return -EFAULT;
	if (copy_from_user(&devh, (void __user *)wr.handle,
			   sizeof(struct xlink_handle)))
		return -EFAULT;
	if (wr.size > XLINK_MAX_CONTROL_DATA_PCIE_SIZE)
		return -EFAULT;
	if (copy_from_user(volbuf, (void __user *)wr.pmessage, wr.size))
		return -EFAULT;
	rc = xlink_write_control_data(&devh, wr.chan, volbuf, wr.size);

	return copy_result_to_user(wr.return_code, rc);
}

int ioctl_write_volatile_data(unsigned long arg)
{
	u8 volbuf[XLINK_MAX_BUF_SIZE]; // buffer for volatile transactions
	struct xlink_handle	devh;
	struct xlinkwritedata	wr;
	int rc;

	if (copy_from_user(&wr, (void __user *)arg,
			   sizeof(struct xlinkwritedata)))
		return -EFAULT;
	if (copy_from_user(&devh, (void __user *)wr.handle,
			   sizeof(struct xlink_handle)))
		return -EFAULT;
	if (wr.size > XLINK_MAX_BUF_SIZE)
		return -EFAULT;
	if (copy_from_user(volbuf, (void __user *)wr.pmessage, wr.size))
		return -EFAULT;
	rc = xlink_write_volatile_user(&devh, wr.chan, volbuf, wr.size);

	return copy_result_to_user(wr.return_code, rc);
}

int ioctl_release_data(unsigned long arg)
{
	struct xlink_handle	devh;
	struct xlinkrelease	rel;
	u8 reladdr;
	int rc;

	if (copy_from_user(&rel, (void __user *)arg,
			   sizeof(struct xlinkrelease)))
		return -EFAULT;
	if (copy_from_user(&devh, (void __user *)rel.handle,
			   sizeof(struct xlink_handle)))
		return -EFAULT;
	if (rel.addr) {
		if (get_user(reladdr, (u32 __user *const)rel.addr))
			return -EFAULT;
		rc = xlink_release_data(&devh, rel.chan,
					(u8 *)&reladdr);
	} else {
		rc = xlink_release_data(&devh, rel.chan, NULL);
	}

	return copy_result_to_user(rel.return_code, rc);
}

int ioctl_close_channel(unsigned long arg)
{
	struct xlink_handle	devh;
	struct xlinkopenchannel	op;
	int rc;

	if (copy_from_user(&op, (void __user *)arg,
			   sizeof(struct xlinkopenchannel)))
		return -EFAULT;
	if (copy_from_user(&devh, (void __user *)op.handle,
			   sizeof(struct xlink_handle)))
		return -EFAULT;
	rc = xlink_close_channel(&devh, op.chan);

	return copy_result_to_user(op.return_code, rc);
}

int ioctl_start_vpu(unsigned long arg)
{
	struct xlinkstartvpu	startvpu;
	char filename[64];
	int rc;

	if (copy_from_user(&startvpu, (void __user *)arg,
			   sizeof(struct xlinkstartvpu)))
		return -EFAULT;
	if (startvpu.namesize > sizeof(filename))
		return -EINVAL;
	memset(filename, 0, sizeof(filename));
	if (copy_from_user(filename, (void __user *)startvpu.filename,
			   startvpu.namesize))
		return -EFAULT;
	rc = xlink_start_vpu(filename);

	return copy_result_to_user(startvpu.return_code, rc);
}

int ioctl_stop_vpu(void)
{
	int rc;

	rc = xlink_stop_vpu();
	return rc;
}

int ioctl_disconnect(unsigned long arg)
{
	struct xlink_handle	devh;
	struct xlinkconnect	con;
	int rc;

	if (copy_from_user(&con, (void __user *)arg,
			   sizeof(struct xlinkconnect)))
		return -EFAULT;
	if (copy_from_user(&devh, (void __user *)con.handle,
			   sizeof(struct xlink_handle)))
		return -EFAULT;
	rc = xlink_disconnect(&devh);

	return copy_result_to_user(con.return_code, rc);
}

int ioctl_get_device_name(unsigned long arg)
{
	char name[XLINK_MAX_DEVICE_NAME_SIZE];
	struct xlink_handle		devh;
	struct xlinkgetdevicename	devn;
	int rc;

	if (copy_from_user(&devn, (void __user *)arg,
			   sizeof(struct xlinkgetdevicename)))
		return -EFAULT;
	if (copy_from_user(&devh, (void __user *)devn.handle,
			   sizeof(struct xlink_handle)))
		return -EFAULT;
	if (devn.name_size <= XLINK_MAX_DEVICE_NAME_SIZE) {
		rc = xlink_get_device_name(&devh, name, devn.name_size);
		if (!rc) {
			if (copy_to_user((void __user *)devn.name, (void *)name,
					 devn.name_size))
				return -EFAULT;
		}
	} else {
		rc = X_LINK_ERROR;
	}

	return copy_result_to_user(devn.return_code, rc);
}

int ioctl_get_device_list(unsigned long arg)
{
	u32 sw_device_id_list[XLINK_MAX_DEVICE_LIST_SIZE];
	struct xlinkgetdevicelist	devl;
	u32 num_devices = 0;
	int rc;

	if (copy_from_user(&devl, (void __user *)arg,
			   sizeof(struct xlinkgetdevicelist)))
		return -EFAULT;
	rc = xlink_get_device_list(sw_device_id_list, &num_devices);
	if (!rc && num_devices <= XLINK_MAX_DEVICE_LIST_SIZE) {
		/* TODO: this next copy is dangerous! we have no idea
		 * how large the devl.sw_device_id_list buffer is
		 * provided by the user. if num_devices is too large,
		 * the copy will overflow the buffer.
		 */
		if (copy_to_user((void __user *)devl.sw_device_id_list,
				 (void *)sw_device_id_list,
				 (sizeof(*sw_device_id_list)
				 * num_devices)))
			return -EFAULT;
		if (copy_to_user((void __user *)devl.num_devices, (void *)&num_devices,
				 (sizeof(num_devices))))
			return -EFAULT;
	}

	return copy_result_to_user(devl.return_code, rc);
}

int ioctl_get_device_status(unsigned long arg)
{
	struct xlink_handle		devh;
	struct xlinkgetdevicestatus	devs;
	u32 device_status;
	int rc;

	if (copy_from_user(&devs, (void __user *)arg,
			   sizeof(struct xlinkgetdevicestatus)))
		return -EFAULT;
	if (copy_from_user(&devh, (void __user *)devs.handle,
			   sizeof(struct xlink_handle)))
		return -EFAULT;
	rc = xlink_get_device_status(&devh, &device_status);
	if (!rc) {
		if (copy_to_user((void __user *)devs.device_status,
				 (void *)&device_status,
				 sizeof(device_status)))
			return -EFAULT;
	}

	return copy_result_to_user(devs.return_code, rc);
}

int ioctl_boot_device(unsigned long arg)
{
	struct xlink_handle		devh;
	struct xlinkbootdevice		boot;
	char filename[64];
	int rc;

	if (copy_from_user(&boot, (void __user *)arg,
			   sizeof(struct xlinkbootdevice)))
		return -EFAULT;
	if (copy_from_user(&devh, (void __user *)boot.handle,
			   sizeof(struct xlink_handle)))
		return -EFAULT;
	if (boot.binary_name_size > sizeof(filename))
		return -EINVAL;
	memset(filename, 0, sizeof(filename));
	if (copy_from_user(filename, (void __user *)boot.binary_name,
			   boot.binary_name_size))
		return -EFAULT;
	rc = xlink_boot_device(&devh, filename);

	return copy_result_to_user(boot.return_code, rc);
}

int ioctl_reset_device(unsigned long arg)
{
	struct xlink_handle		devh;
	struct xlinkresetdevice		res;
	int rc;

	if (copy_from_user(&res, (void __user *)arg,
			   sizeof(struct xlinkresetdevice)))
		return -EFAULT;
	if (copy_from_user(&devh, (void __user *)res.handle,
			   sizeof(struct xlink_handle)))
		return -EFAULT;
	rc = xlink_reset_device(&devh);

	return copy_result_to_user(res.return_code, rc);
}

int ioctl_get_device_mode(unsigned long arg)
{
	struct xlink_handle	devh;
	struct xlinkdevmode	devm;
	u32 device_mode;
	int rc;

	if (copy_from_user(&devm, (void __user *)arg,
			   sizeof(struct xlinkdevmode)))
		return -EFAULT;
	if (copy_from_user(&devh, (void __user *)devm.handle,
			   sizeof(struct xlink_handle)))
		return -EFAULT;
	rc = xlink_get_device_mode(&devh, &device_mode);
	if (!rc) {
		if (copy_to_user((void __user *)devm.device_mode, (void *)&device_mode,
				 sizeof(device_mode)))
			return -EFAULT;
	}

	return copy_result_to_user(devm.return_code, rc);
}

int ioctl_set_device_mode(unsigned long arg)
{
	struct xlink_handle	devh;
	struct xlinkdevmode	devm;
	u32 device_mode;
	int rc;

	if (copy_from_user(&devm, (void __user *)arg,
			   sizeof(struct xlinkdevmode)))
		return -EFAULT;
	if (copy_from_user(&devh, (void __user *)devm.handle,
			   sizeof(struct xlink_handle)))
		return -EFAULT;
	if (copy_from_user(&device_mode, (void __user *)devm.device_mode,
			   sizeof(device_mode)))
		return -EFAULT;
	rc = xlink_set_device_mode(&devh, device_mode);

	return copy_result_to_user(devm.return_code, rc);
}

int ioctl_register_device_event(unsigned long arg)
{
	struct xlinkregdevevent	regdevevent;
	struct xlink_handle	devh;
	u32 num_events;
	u32 *ev_list;
	int rc;

	if (copy_from_user(&regdevevent, (void __user *)arg,
			   sizeof(struct xlinkregdevevent)))
		return -EFAULT;
	if (copy_from_user(&devh, (void __user *)regdevevent.handle,
			   sizeof(struct xlink_handle)))
		return -EFAULT;
	num_events = regdevevent.num_events;
	if (num_events > 0 && num_events <= NUM_REG_EVENTS) {
		ev_list = kzalloc((num_events * sizeof(u32)), GFP_KERNEL);
		if (ev_list) {
			if (copy_from_user(ev_list,
					   (void __user *)regdevevent.event_list,
					   (num_events * sizeof(u32)))) {
				kfree(ev_list);
				return -EFAULT;
			}
			rc = xlink_register_device_event_user(&devh,
							      ev_list,
							      num_events,
							      NULL);
			kfree(ev_list);
		} else {
			rc = X_LINK_ERROR;
		}
	} else {
		rc = X_LINK_ERROR;
	}

	return copy_result_to_user(regdevevent.return_code, rc);
}

int ioctl_unregister_device_event(unsigned long arg)
{
	struct xlinkregdevevent	regdevevent;
	struct xlink_handle	devh;
	u32 num_events;
	u32 *ev_list;
	int rc;

	if (copy_from_user(&regdevevent, (void __user *)arg,
			   sizeof(struct xlinkregdevevent)))
		return -EFAULT;
	if (copy_from_user(&devh, (void __user *)regdevevent.handle,
			   sizeof(struct xlink_handle)))
		return -EFAULT;
	num_events = regdevevent.num_events;
	if (num_events <= NUM_REG_EVENTS) {
		ev_list = kzalloc((num_events * sizeof(u32)), GFP_KERNEL);
		if (copy_from_user(ev_list,
				   (void __user *)regdevevent.event_list,
				   (num_events * sizeof(u32)))) {
			kfree(ev_list);
			return -EFAULT;
		}
		rc = xlink_unregister_device_event(&devh, ev_list, num_events);
		kfree(ev_list);
	} else {
		rc = X_LINK_ERROR;
	}

	return copy_result_to_user(regdevevent.return_code, rc);
}

int ioctl_data_ready_callback(unsigned long arg)
{
	struct xlink_handle	devh;
	struct xlinkcallback	cb;
	int rc;

	if (copy_from_user(&cb, (void __user *)arg,
			   sizeof(struct xlinkcallback)))
		return -EFAULT;
	if (copy_from_user(&devh, (void __user *)cb.handle,
			   sizeof(struct xlink_handle)))
		return -EFAULT;
	CHANNEL_SET_USER_BIT(cb.chan); // set MSbit for user space call
	rc = xlink_data_available_event(&devh, cb.chan, cb.callback);

	return copy_result_to_user(cb.return_code, rc);
}

int ioctl_data_consumed_callback(unsigned long arg)
{
	struct xlink_handle	devh;
	struct xlinkcallback	cb;
	int rc;

	if (copy_from_user(&cb, (void __user *)arg,
			   sizeof(struct xlinkcallback)))
		return -EFAULT;
	if (copy_from_user(&devh, (void __user *)cb.handle,
			   sizeof(struct xlink_handle)))
		return -EFAULT;
	CHANNEL_SET_USER_BIT(cb.chan); // set MSbit for user space call
	rc = xlink_data_consumed_event(&devh, cb.chan, cb.callback);

	return copy_result_to_user(cb.return_code, rc);
}
