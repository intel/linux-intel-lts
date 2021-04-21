// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 * High Density Deep Learning HELPER module.
 *
 * Copyright (C) 2020 Intel Corporation
 *
 */

#include <asm/page.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/hddl_device.h>
#include <linux/i2c.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/kmod.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/sched.h>
#include <linux/sched/mm.h>
#include <linux/sched/task.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/xlink.h>
#include <uapi/linux/stat.h>
#include "hddl_device_util.h"

#define HDDL_XLINK_OPEN_TIMEOUT		1000
#define HDDL_I2C_CLIENT_INIT_TIME		1000

enum hddl_device_event_type {
	HDDL_NOTIFY_DEVICE_DISCONNECTED,
	HDDL_NOTIFY_DEVICE_CONNECTED,
	HDDL_NUM_EVENT_TYPE,
};

/*
 * Register callback for device
 * connect and disconnect events.
 */
static u32 hddl_device_events[] = {
	HDDL_NOTIFY_DEVICE_DISCONNECTED,
	HDDL_NOTIFY_DEVICE_CONNECTED,
	HDDL_NUM_EVENT_TYPE
};

/**
 * intel_hddl_new_device - check for new hddl device
 * @dev: The hddl device.
 * @hddl_clients: list of existing client devices.
 * @sw_device_id_list: list of software device id's to check for new device.
 * @num_devices: number of id's present in sw_device_id_list.
 * @n_devs: number of existing client devices in hddl_clients list.
 *
 * Returns list of client devices by comapring the device id's with device id's
 * present in existing client devices list. If any software id which is not
 * matching with id's in existing client devices, it allocates new list to
 * accommodate new client device and copies the existing client devices to
 * new list and free's up the older list.
 * If no new device id found in sw_device_id_list, it returns list of existing
 * client devices.
 */
static struct intel_hddl_clients **
		intel_hddl_new_device(struct device *dev,
				      struct intel_hddl_clients **hddl_clients,
				      u32 *sw_device_id_list,
				      u32 num_devices,
				      u32 *n_devs)
{
	struct intel_hddl_clients **cls;
	bool match_found, new_dev = false;
	int i, j;

	/*
	 * Check is there any new device by comparing id's with
	 * existing list.
	 */
	for (i = 0; i < num_devices; i++) {
		match_found = false;
		for (j = 0; j < (*n_devs); j++) {
			if (sw_device_id_list[i] ==
				hddl_clients[j]->xlink_dev.sw_device_id) {
				match_found = true;
				break;
			}
		}
		if (!match_found) {
			new_dev = true;
			break;
		}
	}
	if (!new_dev)
		return hddl_clients;
	/*
	 * Allocate memory for new list
	 */
	cls = kcalloc(num_devices,
		      sizeof(struct intel_hddl_clients *),
		      GFP_KERNEL);
	if (!cls)
		return hddl_clients;
	/*
	 * copy hddl client devices to new list.
	 */
	for (i = 0; i < num_devices; i++) {
		for (j = 0; j < *n_devs; j++) {
			if (sw_device_id_list[i] ==
				hddl_clients[j]->xlink_dev.sw_device_id) {
				cls[i] = hddl_clients[j];
				break;
			}
		}
	}
	/*
	 * update number of devices to include new device and free up existing
	 * list.
	 */
	*n_devs = num_devices;
	kfree(hddl_clients);
	return cls;
}

/**
 * intel_hddl_setup_device - Initialize new client device
 * @dev: The hddl device.
 * @task: Thread function to setup and connect to host/device.
 * @n_devs: number of client devices in hddl_clients list.
 * @hddl_clients: list of existing client devices.
 * @pdata: platform data.
 *
 * Returns list of client devices. It also initialize the device and creates
 * kernel thread to initiate communication over xlink.
 */

struct intel_hddl_clients **
	intel_hddl_setup_device(struct device *dev,
				intel_hddl_connect_task task, u32 *n_devs,
				struct intel_hddl_clients **hddl_clients,
				void *pdata)
{
	u32 sw_device_id_list[XLINK_MAX_DEVICE_LIST_SIZE];
	char device_name[XLINK_MAX_DEVICE_NAME_SIZE];
	struct intel_hddl_clients **cls;
	u32 num_devices = 0;
	u32 i = 0;

	xlink_get_device_list(sw_device_id_list, &num_devices);
	if (num_devices == 0) {
		dev_err(dev, "HDDL:No devices found\n");
		return NULL;
	}

	/*
	 * If list available, add new device to the existing client devices
	 * list, if list is not available create new list.
	 */
	if (hddl_clients) {
		cls = intel_hddl_new_device(dev,
					    hddl_clients,
					    sw_device_id_list,
					    num_devices, n_devs);
		if (!cls)
			return NULL;
	} else {
		cls = kcalloc(num_devices,
			      sizeof(struct intel_hddl_clients *),
			      GFP_KERNEL);
		if (!cls)
			return NULL;
		/*
		 * update number of devices in client devices list
		 */
		*n_devs = num_devices;
	}
	hddl_clients = cls;
	for (i = 0; i < num_devices; i++) {
		struct intel_hddl_clients *c = hddl_clients[i];
		int rc;

		/*
		 * Initialize new client device.
		 */
		if (c)
			continue;
		c = devm_kzalloc(dev,
				 sizeof(struct intel_hddl_clients),
				 GFP_KERNEL);
		if (!c)
			return hddl_clients;
		c->pdata = pdata;
		c->xlink_dev.dev_type = HOST_DEVICE;
		c->xlink_dev.sw_device_id = sw_device_id_list[i];
		rc = xlink_get_device_name((&c->xlink_dev),
					   device_name,
					   XLINK_MAX_DEVICE_NAME_SIZE);
		if (rc > 0) {
			dev_err(dev,
				"HDDL:Failed to get device name [EC%d] %x\n",
				rc, c->xlink_dev.sw_device_id);
			return hddl_clients;
		}
		dev_info(dev, "HDDL:Device name: %x %s\n",
			 c->xlink_dev.sw_device_id, device_name);
		if (GET_INTERFACE_FROM_SW_DEVICE_ID(sw_device_id_list[i]) ==
		    SW_DEVICE_ID_PCIE_INTERFACE) {
			/*
			 * Start kernel thread to initialize
			 * xlink communication.
			 */
			c->hddl_dev_connect_task = kthread_run(task,
							       (void *)c,
							       device_name);
			if (!c->hddl_dev_connect_task) {
				dev_err(dev, "failed to create thread\n");
				return hddl_clients;
			}
			c->task = (void *)task;
		}
		hddl_clients[i] = c;
	}

	return hddl_clients;
}

int intel_hddl_xlink_remove_i2c_adap(struct device *dev,
				     struct intel_hddl_clients *c)
{
	int i;

	for (i = 0; i < HDDL_XLINK_I2C_END; i++) {
		if (c->xlink_i2c_plt_dev[i]) {
			dev_info(dev,
				 "HDDL : platform_device_unregister = %d\n",
				 i);
			platform_device_unregister(c->xlink_i2c_plt_dev[i]);
			c->xlink_i2c_plt_dev[i] = NULL;
		}
	}
	return 0;
}

static void hddl_register_remote_smbus_client(struct device *dev,
					      struct intel_hddl_clients *c,
					      struct intel_hddl_i2c_devs *i2c)
{
	struct platform_device *xlink_pdev =
			c->xlink_i2c_plt_dev[HDDL_XLINK_I2C_MASTER];
	struct i2c_adapter *adap;

	if (!xlink_pdev)
		return;
	adap = (struct i2c_adapter *)platform_get_drvdata(xlink_pdev);
	if (!adap)
		return;
	c->adap[HDDL_XLINK_I2C_MASTER] = adap;
	i2c->smbus_client =
		i2c_new_client_device(adap,
				      &i2c->board_info);
	msleep_interruptible(HDDL_I2C_CLIENT_INIT_TIME);
}

static void hddl_register_remote_i2c_client(struct device *dev,
					    struct intel_hddl_clients *c,
					    struct intel_hddl_i2c_devs *i2c)
{
	if (c->smbus_adap) {
		i2c->board_info.platform_data = c;
		i2c->i2c_client = i2c_new_client_device(c->smbus_adap,
							&i2c->board_info);
		msleep_interruptible(HDDL_I2C_CLIENT_INIT_TIME);
	}
}

static void hddl_register_remote_xlink_client(struct device *dev,
					      struct intel_hddl_clients *c,
					      struct intel_hddl_i2c_devs *i2c)
{
	struct platform_device *xlink_pdev =
			c->xlink_i2c_plt_dev[HDDL_XLINK_I2C_SLAVE];
	struct i2c_adapter *adap;

	if (!xlink_pdev)
		return;
	adap = (struct i2c_adapter *)platform_get_drvdata(xlink_pdev);
	if (!adap)
		return;
	c->adap[HDDL_XLINK_I2C_SLAVE] = adap;
	i2c->board_info.platform_data = c;
	i2c->xlk_client = i2c_new_client_device(adap, &i2c->board_info);
	msleep_interruptible(HDDL_I2C_CLIENT_INIT_TIME);
}

static void intel_hddl_add_remote_clients(struct device *dev,
					  struct intel_hddl_clients *c,
					  struct intel_hddl_i2c_devs *i2c_devs)
{
	if (!i2c_devs->enabled)
		return;
	/*
	 * Register this device as xlink i2c client.
	 */
	if (i2c_devs->remote_host & HDDL_XLINK_CLIENT)
		hddl_register_remote_xlink_client(dev, c, i2c_devs);
	/*
	 * Register this device as i2c smbbus client.
	 */
	if (i2c_devs->remote_host & HDDL_I2C_CLIENT)
		hddl_register_remote_i2c_client(dev, c, i2c_devs);
	/*
	 * Register this device as xlink smbus i2c client.
	 * Based on the bit mask of remote_host, It is possible that same
	 * device can be registered as xlink i2c client and smbus i2c client.
	 */
	if (i2c_devs->remote_host & HDDL_XLINK_SMBUS_CLIENT)
		hddl_register_remote_smbus_client(dev, c, i2c_devs);
}

static void intel_hddl_add_localhost_clients(struct device *dev,
					     struct intel_hddl_clients *c,
					     struct intel_hddl_i2c_devs *i2c)
{
	if (!i2c->enabled)
		return;
	/*
	 * Register this device as xlink i2c client.
	 */
	if (i2c->local_host & HDDL_XLINK_CLIENT) {
		struct platform_device *xlink_pdev =
			c->xlink_i2c_plt_dev[HDDL_XLINK_I2C_SLAVE];
		struct i2c_adapter *adap;

		if (!xlink_pdev)
			return;
		adap = (struct i2c_adapter *)platform_get_drvdata(xlink_pdev);
		if (!adap)
			return;
		c->adap[HDDL_XLINK_I2C_SLAVE] = adap;
		i2c->xlk_client =
			i2c_new_client_device(adap,
					      &i2c->board_info);
		msleep_interruptible(HDDL_I2C_CLIENT_INIT_TIME);
	}
	/*
	 * Register this device as smbus i2c client.
	 * Based on the bit mask of local_host, It is possible that same
	 * device can be registered as xlink i2c client and smbus i2c client.
	 */
	if (i2c->local_host & HDDL_I2C_CLIENT) {
		i2c->i2c_client =
			i2c_new_client_device(i2c_get_adapter(i2c->bus),
					      &i2c->board_info);
		msleep_interruptible(HDDL_I2C_CLIENT_INIT_TIME);
	}
}

void intel_hddl_add_xlink_i2c_clients(struct device *dev,
				      struct intel_hddl_clients *c,
				      struct intel_hddl_i2c_devs **i2c_devs,
				      int n_clients, int remote)
{
	int i;

	for (i = 0; i < n_clients; i++) {
		if (remote)
			intel_hddl_add_remote_clients(dev, c, i2c_devs[i]);
		else
			intel_hddl_add_localhost_clients(dev, c, i2c_devs[i]);
	}
}

int intel_hddl_register_xlink_i2c_adap(struct device *dev,
				       struct intel_hddl_clients *c)
{
	int i;

	for (i = 0; i < HDDL_XLINK_I2C_END; i++) {
		struct platform_device_info xlink_i2c_info;
		int soc_id = c->board_info.soc_id;

		memset(&xlink_i2c_info, 0, sizeof(xlink_i2c_info));
		xlink_i2c_info.name = "i2c_xlink";
		xlink_i2c_info.id = c->board_info.board_id << 5 |
					soc_id << 2 | i;
		c->xlink_i2c_ch[i] =
			c->i2c_chan_num + (soc_id * 2) + i;
		xlink_i2c_info.data = c;
		xlink_i2c_info.size_data =
			sizeof(struct intel_hddl_clients);
		c->xlink_i2c_plt_dev[i] =
			platform_device_register_full(&xlink_i2c_info);
		if (!c->xlink_i2c_plt_dev[i]) {
			dev_err(dev, "platform device register failed\n");
			return -EFAULT;
		}
	}
	return 0;
}

static int intel_hddl_device_probe(struct intel_hddl_clients *d)
{
	char device_name[XLINK_MAX_DEVICE_NAME_SIZE];
	int rc;

	if (d->status != HDDL_DEV_STATUS_DISCONNECTED)
		return 0;
	rc = xlink_get_device_name(&d->xlink_dev,
				   device_name, XLINK_MAX_DEVICE_NAME_SIZE);
	if (rc > 0) {
		dev_err(&d->pdev->dev,
			"HDDL:Failed to get device name of id [EC%d] %x\n",
			rc, d->xlink_dev.sw_device_id);
		return -ENODEV;
	}

	d->hddl_dev_connect_task =
		kthread_run((intel_hddl_connect_task)d->task,
			    (void *)d,
			    device_name);
	if (!d->hddl_dev_connect_task) {
		dev_err(&d->pdev->dev, "failed to create thread\n");
		return -EFAULT;
	}

	return 0;
}

void intel_hddl_device_remove(struct intel_hddl_clients *d)
{
	int i;

	/** lock device removal.
	 * xlink core gives multiple device disconnected notifications,
	 * so add lock to diconnect the device and update the status as
	 * disconnected. subsequent notificatios will check the status
	 * and returs if the device is already disconnected.
	 */
	mutex_lock(&d->lock);
	if (d->status == HDDL_DEV_STATUS_DISCONNECTED) {
		mutex_unlock(&d->lock);
		return;
	}

	for (i = 0; i < d->n_clients; i++)
		intel_hddl_free_i2c_client(d, d->i2c_devs[i]);
	intel_hddl_unregister_pdev(d);
	xlink_close_channel(&d->xlink_dev, d->chan_num);
	xlink_disconnect(&d->xlink_dev);
	kthread_stop(d->hddl_dev_connect_task);
	d->status = HDDL_DEV_STATUS_DISCONNECTED;
	mutex_unlock(&d->lock);
}

static int intel_hddl_device_event_notify(u32 sw_device_id,
					  uint32_t event_type)
{
	struct intel_hddl_clients **clients;
	struct intel_hddl_clients *client;
	int i, ret = 0;
	int ndevs = 0;

	clients = intel_hddl_get_clients(&ndevs);
	if (!clients)
		return 0;
	for (i = 0; i < ndevs; i++) {
		if (clients[i]->xlink_dev.sw_device_id == sw_device_id) {
			client = clients[i];
			break;
		}
	}
	if (!client)
		return -EINVAL;
	switch (event_type) {
	case HDDL_NOTIFY_DEVICE_DISCONNECTED:
		intel_hddl_device_remove(client);
		break;

	case HDDL_NOTIFY_DEVICE_CONNECTED:
		ret = intel_hddl_device_probe(client);
		break;

	default:
		dev_err(&client->pdev->dev,
			"HDDL:xlink pcie notify - Error[%x]: [%d]\n",
			sw_device_id, event_type);
		ret = -EINVAL;
		break;
	}
	return ret;
}

void intel_hddl_close_xlink_device(struct device *dev,
				   struct intel_hddl_clients *d)
{
	xlink_close_channel(&d->xlink_dev, d->chan_num);
	xlink_disconnect(&d->xlink_dev);
	kthread_stop(d->hddl_dev_connect_task);
	d->status = HDDL_DEV_STATUS_DISCONNECTED;
}

int intel_hddl_open_xlink_device(struct device *dev,
				 struct intel_hddl_clients *d)
{
	char device_name[XLINK_MAX_DEVICE_NAME_SIZE];
	u32 device_status = 0xFF;
	int rc;

	rc = xlink_get_device_name(&d->xlink_dev,
				   device_name, XLINK_MAX_DEVICE_NAME_SIZE);
	if (rc > 0) {
		dev_err(dev,
			"HDDL:Failed to get device name of id [EC%d] %x\n",
			rc, d->xlink_dev.sw_device_id);
		return -ENODEV;
	}
	if (xlink_boot_device(&d->xlink_dev, device_name) !=
	       X_LINK_SUCCESS) {
		dev_err(dev, "xlink_boot_device failed\n");
		return -ENODEV;
	}
	if (xlink_get_device_status(&d->xlink_dev, &device_status) !=
	       X_LINK_SUCCESS) {
		dev_err(dev, "xlink_get_device_status failed\n");
		return -ENODEV;
	}
	if (xlink_connect(&d->xlink_dev) != X_LINK_SUCCESS) {
		dev_err(dev, "xlink_connect failed\n");
		return -ENODEV;
	}
	mutex_init(&d->lock);
	xlink_register_device_event(&d->xlink_dev,
				    hddl_device_events,
				    HDDL_NUM_EVENT_TYPE,
				    intel_hddl_device_event_notify);

	d->status = HDDL_DEV_STATUS_CONNECTED;
	/*
	 * Try opening xlink channel, open channel will fail till host/client
	 * initilaizes the channel. intel_hddl_open_xlink_device is invoked
	 * from kernel thread. so it is safe to try indefinitely.
	 */
	while (xlink_open_channel(&d->xlink_dev,
				  d->chan_num, RXB_TXB,
				  64 * 1024, 0 /* timeout */) !=
				  X_LINK_SUCCESS) {
		if (kthread_should_stop()) {
			xlink_disconnect(&d->xlink_dev);
			return -ENODEV;
		}
	}

	return 0;
}
