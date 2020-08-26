// SPDX-License-Identifier: GPL-2.0-only
/*
 * HDDL Device HELPER module.
 *
 * Copyright (C) 2019-2020 Intel Corporation
 */

#ifndef _LINUX_HDDL_DEVICE_HELPER_H
#define _LINUX_HDDL_DEVICE_HELPER_H

#include <linux/xlink_drv_inf.h>
#include "../xlink-core/xlink-defs.h"

static uint32_t hddl_xlink_device_events[] = {
	_NOTIFY_INCOMING_DISCONNECTION,
	_NOTIFY_DEVICE_DISCONNECTED,
	_NOTIFY_DEVICE_CONNECTED,
	_ERROR_UNEXPECTED_DISCONNECTION,
	_NUM_EVENT_TYPE
};

typedef int (*intel_hddl_connect_task)(void *);


static inline struct intel_hddl_clients **intel_hddl_new_device(
			struct device *dev,
			struct intel_hddl_clients **hddl_clients,
			uint32_t *sw_device_id_list,
			uint32_t num_devices,
			uint32_t *n_devs)
{
	int i, j;
	bool new_dev = true;
	struct intel_hddl_clients **cls;

	for (i = 0; i < num_devices; i++) {
		for (j = 0; j < (*n_devs); j++) {
			if (sw_device_id_list[i] ==
				hddl_clients[j]->xlink_dev.sw_device_id) {
				new_dev = false;
				break;
			}
		}
	}
	if (!new_dev)
		return hddl_clients;
	dev_info(dev, "new device found\n");
	cls = devm_kzalloc(dev,
		(sizeof(struct intel_hddl_clients *) * num_devices),
		GFP_KERNEL);
	if (cls == NULL) {
		dev_err(dev, "HDDL:No Memory\n");
		return NULL;
	}
	*n_devs = num_devices;
	for (i = 0; i < num_devices; i++) {
		for (j = 0; j < *n_devs; j++) {
			if (sw_device_id_list[i] ==
				hddl_clients[j]->xlink_dev.sw_device_id) {
				cls[i] = hddl_clients[j];
				break;
			}
		}
	}
	devm_kfree(dev, hddl_clients);
	return cls;
}

static inline struct intel_hddl_clients **intel_hddl_setup_device(
		struct device *dev,
		intel_hddl_connect_task task, uint32_t *n_devs,
		struct intel_hddl_clients **hddl_clients,
		void *pdata)
{
	uint32_t num_devices = 0;
	uint32_t sw_device_id_list[XLINK_MAX_DEVICE_LIST_SIZE];
	char device_name[XLINK_MAX_DEVICE_NAME_SIZE];
	uint32_t i = 0;

	xlink_get_device_list(sw_device_id_list, &num_devices);
	if (num_devices == 0) {
		dev_err(dev, "HDDL:No devices found\n");
		return NULL;
	}
	dev_dbg(dev, "HDDL:%u devices found...\n", num_devices);

	if (hddl_clients != NULL) {
		hddl_clients = intel_hddl_new_device(dev,
				hddl_clients, sw_device_id_list,
				num_devices, n_devs);
		if (!hddl_clients)
			return NULL;
	} else {
		hddl_clients = devm_kzalloc(dev,
				(sizeof(struct intel_hddl_clients *)
				 * num_devices),
				GFP_KERNEL);
		if (hddl_clients == NULL) {
			dev_err(dev, "HDDL:No Memory\n");
			return NULL;
		}
		*n_devs = num_devices;
	}

	for (i = 0; i < num_devices; i++) {
		struct intel_hddl_clients *c = hddl_clients[i];

		if (c != NULL)
			continue;
		c = devm_kzalloc(dev,
				(sizeof(struct intel_hddl_clients)),
				GFP_KERNEL);
		if (c == NULL) {
			dev_err(dev, "Memory alloc failed\n");
			goto f_clients;
		}

		c->pdata = pdata;
		c->xlink_dev.dev_type = HOST_DEVICE;
		c->xlink_dev.sw_device_id = sw_device_id_list[i];
		xlink_get_device_name((&c->xlink_dev),
		device_name, XLINK_MAX_DEVICE_NAME_SIZE);
		dev_info(dev, "HDDL:Device name: %x %s\n",
			c->xlink_dev.sw_device_id, device_name);
		if (GET_INTERFACE_FROM_SW_DEVICE_ID(
				c->xlink_dev.sw_device_id) ==
				SW_DEVICE_ID_PCIE_INTERFACE) {
			c->hddl_dev_connect_task = kthread_run(task,
								(void *)c,
								device_name);
			if (!c->hddl_dev_connect_task) {
				dev_err(dev, "failed to create thread\n");
				goto f_wq;
			}
			c->task = (void *)task;
		}
		hddl_clients[i] = c;
	}

	return hddl_clients;
	/*TODO: free memory */
f_wq:
f_clients:
	return NULL;
}

static inline int intel_hddl_xlink_remove_i2c_adap(struct device *dev,
			struct intel_hddl_clients *c)
{
	int k;

	for (k = 0; k < 2; k++) {
		if (c->xlink_i2c_plt_dev[k] != NULL) {
			dev_info(dev,
			"HDDL : platform_device_unregister = %d\n",
			k);
			platform_device_unregister(c->xlink_i2c_plt_dev[k]);
			c->xlink_i2c_plt_dev[k] = NULL;
		}
	}
	return 0;
}

static int intel_hddl_remote_xlink_slave(struct device *dev,
				struct intel_hddl_clients *c,
				struct intel_hddl_i2c_devs *i2c_devs)
{
	if (!i2c_devs->enabled)
		return 0;
	if (i2c_devs->remote_host &
		HDDL_XLINK_SLAVE) {
		if (!c->xlink_i2c_plt_dev[1])
			return 0;
		c->adap[1] = (struct i2c_adapter *)platform_get_drvdata(
						c->xlink_i2c_plt_dev[1]);
		dev_info(dev,
		"HDDL : registering xlink smbus slave %s addr %d\n",
		i2c_devs->board_info.type,
		i2c_devs->board_info.addr);
		if (!c->adap[1]) {
			dev_info(dev,
			"HDDL : xlink smbus adapter not loaded\n");
			return 0;
		}
		i2c_devs->board_info.platform_data = c;
		i2c_devs->xlk_client = i2c_new_device(
					c->adap[1],
					&i2c_devs->board_info);
		ssleep(1);
	}
	if (i2c_devs->remote_host &
		HDDL_I2C_SLAVE) {
		int j = 0;
		struct i2c_adapter *temp;

		while ((temp = i2c_get_adapter(j)) != NULL) {
			if (strstr(temp->name, "SMBus I801") != NULL) {
				i2c_devs->board_info.platform_data = c;
				i2c_devs->i2c_client =
						i2c_new_device(temp,
						&i2c_devs->board_info);
				dev_info(dev, "soc_smbus adapter %d",
				i2c_devs->i2c_client->adapter->nr);
				ssleep(1);
				break;
			}
			i2c_put_adapter(temp);
			j++;
		}
	}
	if (i2c_devs->remote_host &
		HDDL_XLINK_SMBUS_SLAVE) {
		if (!c->xlink_i2c_plt_dev[0])
			return 0;
		c->adap[0] = (struct i2c_adapter *)platform_get_drvdata(
						c->xlink_i2c_plt_dev[0]);
		dev_info(dev,
			"HDDL : registering xlink smbus slave %s addr %d\n",
					i2c_devs->board_info.type,
					i2c_devs->board_info.addr);
		if (!c->adap[0]) {
			dev_info(dev,
				"HDDL : xlink smbus adapter not loaded\n");
			return 0;
		}
		i2c_devs->smbus_client = i2c_new_device(
						c->adap[0],
						&i2c_devs->board_info);
		ssleep(1);
	}

	return 0;
}

static int intel_hddl_xlink_slave(struct device *dev,
				struct intel_hddl_clients *c,
				struct intel_hddl_i2c_devs *i2c_devs)
{
	if (!i2c_devs->enabled)
		return 0;
	if (i2c_devs->local_host &
		HDDL_XLINK_SLAVE) {
		if (!c->xlink_i2c_plt_dev[1])
			return 0;
		c->adap[1] = (struct i2c_adapter *)platform_get_drvdata(
						c->xlink_i2c_plt_dev[1]);
		dev_info(dev, "HDDL : registering xlink slave %s addr %d\n",
					i2c_devs->board_info.type,
					i2c_devs->board_info.addr);
		if (!c->adap[1]) {
			dev_info(dev, "HDDL : Slave adapter not loaded\n");
			return 0;
		}
		i2c_devs->xlk_client = i2c_new_device(
						c->adap[1],
					&i2c_devs->board_info);
		ssleep(1);
	}
	if (i2c_devs->local_host &
			HDDL_I2C_SLAVE) {
		dev_info(dev, "HDDL : registering i2c slave %s addr %d\n",
					i2c_devs->board_info.type,
					i2c_devs->board_info.addr);
		i2c_devs->i2c_client = i2c_new_device(
					i2c_get_adapter(i2c_devs->bus),
						&i2c_devs->board_info);
		ssleep(1);
	}
	return 0;
}

static int intel_hddl_xlink_i2c_clients(struct device *dev,
				struct intel_hddl_clients *c,
				struct intel_hddl_i2c_devs **i2c_devs,
				int n_slaves, int remote)
{
	int i, ret;

	for (i = 0; i < n_slaves; i++) {
		if (remote) {
			ret = intel_hddl_remote_xlink_slave(dev,
					c, i2c_devs[i]);
		} else {
			ret = intel_hddl_xlink_slave(dev,
					c, i2c_devs[i]);
		}
	}

	return ret;
}

static int intel_hddl_xlink_i2c_adap(struct device *dev,
		struct intel_hddl_clients *c)
{
	int i;

	for (i = 0; i < 2; i++) {
		struct platform_device_info xlink_i2c_info;
		int soc_id = c->board_info.soc_id;

		memset(&xlink_i2c_info, 0, sizeof(xlink_i2c_info));
		xlink_i2c_info.name = "i2c_xlink";
		xlink_i2c_info.id = c->board_info.board_id << 4 |
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
			return -ENOMEM;
		}
	}
	return 0;
}

static struct intel_hddl_clients **intel_hddl_get_device(int *ndevs);

static void intel_hddl_device_probe(struct intel_hddl_clients *d)
{
	char device_name[XLINK_MAX_DEVICE_NAME_SIZE];

	if (d->status == HDDL_DEV_STATUS_CONNECTED)
		return;
	xlink_get_device_name((&d->xlink_dev),
		device_name, XLINK_MAX_DEVICE_NAME_SIZE);
	d->hddl_dev_connect_task = kthread_run(
			(intel_hddl_connect_task)d->task,
			(void *)d,
			device_name);
	if (!d->hddl_dev_connect_task) {
		pr_err("failed to create thread\n");
		return;
	}
	d->status = HDDL_DEV_STATUS_CONNECTED;
}

static int intel_hddl_unregister_pdev(struct intel_hddl_clients *c);

static void intel_hddl_device_remove(struct intel_hddl_clients *d)
{
	int k, rc;

	mutex_lock(&d->lock);
	if (d->status == HDDL_DEV_STATUS_DISCONNECTED) {
		mutex_unlock(&d->lock);
		return;
	}

	for (k = 0; k < d->n_slaves; k++) {
		struct intel_hddl_i2c_devs *i2c_dev =
			d->i2c_devs[k];

		if (i2c_dev->xlk_client) {
			pr_info("HDDL: unregistering %s\n",
					i2c_dev->xlk_client->name);
			i2c_unregister_device(i2c_dev->xlk_client);
			pr_info("HDDL: unregistered\n");
		}
		if (i2c_dev->i2c_client) {
			pr_info("HDDL: unregistering %s\n",
					i2c_dev->i2c_client->name);
			i2c_unregister_device(i2c_dev->i2c_client);
			pr_info("HDDL: unregistered\n");
		}
		if (i2c_dev->smbus_client) {
			pr_info("HDDL: unregistering %s\n",
					i2c_dev->smbus_client->name);
			i2c_unregister_device(i2c_dev->smbus_client);
			pr_info("HDDL: unregistered\n");
		}
		i2c_dev->xlk_client = NULL;
		i2c_dev->i2c_client = NULL;
		i2c_dev->smbus_client = NULL;
	}
	pr_info("HDDL:i2c slaves unregisterd\n");

	intel_hddl_unregister_pdev(d);

	pr_info("HDDL: platform device unregistered\n");
	rc = xlink_close_channel(&d->xlink_dev, d->chan_num);

	rc = xlink_disconnect(&d->xlink_dev);
	kthread_stop(d->hddl_dev_connect_task);
	d->status = HDDL_DEV_STATUS_DISCONNECTED;
	pr_info("HDDL: device status %d\n",
			d->status);
	mutex_unlock(&d->lock);
}

static int intel_hddl_device_pcie_event_notify(uint32_t sw_device_id,
				enum _xlink_device_event_type event_type)
{
	/*TODO: XLINK should pass private ptr*/
	int ndevs = 0;
	struct intel_hddl_clients **c = intel_hddl_get_device(&ndevs);
	struct intel_hddl_clients *d = NULL;
	int i;

	pr_info("HDDL:xlink pcie notify[%x]: [%d]\n",
			sw_device_id, event_type);
	if (!c)
		return 0;
	for (i = 0; i < ndevs; i++) {
		if (c[i]->xlink_dev.sw_device_id == sw_device_id) {
			d = c[i];
			break;
		}
	}
	if (!d)
		return 0;
	switch (event_type) {
	case _NOTIFY_INCOMING_DISCONNECTION:
	case _NOTIFY_DEVICE_DISCONNECTED:
	case _ERROR_UNEXPECTED_DISCONNECTION:
		intel_hddl_device_remove(d);
		break;

	case _NOTIFY_DEVICE_CONNECTED:
		intel_hddl_device_probe(d);
		break;

	default:
		pr_info("HDDL:xlink pcie notify - Error[%x]: [%d]\n",
				sw_device_id, event_type);
		break;
	}
	return 0;
}

static inline int intel_hddl_open_xlink_device(struct device *dev,
		struct intel_hddl_clients *d)
{
	char device_name[XLINK_MAX_DEVICE_NAME_SIZE];
	uint32_t device_status = 0xFF;
	int rc;

	dev_info(dev, "intel_hddl_device_connect_task\n");
	mutex_init(&d->lock);
	rc = xlink_get_device_name(&d->xlink_dev,
		device_name, XLINK_MAX_DEVICE_NAME_SIZE);
	if (rc > 0) {
		dev_err(dev,
		"HDDL:Failed to get device name of id [EC%d] %x\n",
		rc, d->xlink_dev.sw_device_id);
		return -ENODEV;
	}
	dev_info(dev, "HDDL:Device name: %x %s\n",
		d->xlink_dev.sw_device_id, device_name);
	while (xlink_boot_device(&d->xlink_dev, device_name) !=
				X_LINK_SUCCESS) {
		msleep_interruptible(1000);
		if (kthread_should_stop())
			return -ENODEV;
	}
	while (xlink_get_device_status(&d->xlink_dev, &device_status) !=
				X_LINK_SUCCESS) {
		msleep_interruptible(1000);
		if (kthread_should_stop())
			return -ENODEV;
	}
	dev_info(dev, "HDDL:Device status %u\n", device_status);
	while (xlink_connect(&d->xlink_dev) != X_LINK_SUCCESS) {
		msleep_interruptible(1000);
		if (kthread_should_stop())
			return -ENODEV;
	}
	dev_info(dev, "HDDL:Channel Number[%x]: %u\n",
			d->xlink_dev.sw_device_id,
				d->chan_num);

	xlink_pcie_register_device_event(d->xlink_dev.sw_device_id,
					hddl_xlink_device_events,
					_NUM_EVENT_TYPE,
					intel_hddl_device_pcie_event_notify,
					0);

	d->status = HDDL_DEV_STATUS_CONNECTED;
	while (xlink_open_channel(&d->xlink_dev,
				d->chan_num, RXB_TXB,
				64 * 1024, 0 /* timeout */) !=
				X_LINK_SUCCESS) {
		msleep_interruptible(1000);
		if (kthread_should_stop()) {
			xlink_disconnect(&d->xlink_dev);
			return -ENODEV;
		}
	}
	dev_info(dev, "HDDL: xlink_open_channel completed\n");

	return 0;
}


#endif /* _LINUX_HDDL_DEVICE_HELPER_H */
