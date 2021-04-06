// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 * High Density Deep Learning Kernel module.
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

#define DRIVER_NAME "hddl_device_client"

struct intel_tsens {
	struct intel_tsens_data data;
	struct intel_tsens_trip_info **trip_info;
};

struct intel_hddl_client_priv {
	void __iomem *base_addr;
	int board_id;
	int soc_id;
	int n_clients;
	u32 nsens;
	u32 xlink_chan;
	u32 i2c_xlink_chan;
	u32 n_hddl_devs;
	struct platform_device *pdev;
	struct intel_hddl_clients **hddl_client;
	struct task_struct *hddl_dev_init_task;
	struct intel_hddl_i2c_devs **i2c_devs;
	struct intel_tsens **tsens;
	struct intel_hddl_board_info board_info;
	bool soc_reset_available;
};

static struct intel_hddl_client_priv *g_priv;

static inline int intel_hddl_get_xlink_data(struct device *dev,
					    struct xlink_handle *xlink,
					    int chan_num, u8 *msg,
					    int *size)
{
	int rc;

	rc = xlink_read_data_to_buffer(xlink, chan_num,
				       msg, size);
	if (rc) {
		dev_err(dev,
			"HDDL: xlink read data failed rc = %d\n",
			rc);
		return -EFAULT;
	}
	rc = xlink_release_data(xlink, chan_num, NULL);
	if (rc) {
		dev_err(dev,
			"HDDL: xlink release failed rc = %d\n",
			rc);
		return -EFAULT;
	}
	return rc;
}

void intel_hddl_free_i2c_client(struct intel_hddl_clients *d,
				struct intel_hddl_i2c_devs *i2c_dev)
{
	if (i2c_dev->xlk_client)
		i2c_unregister_device(i2c_dev->xlk_client);
	if (i2c_dev->i2c_client)
		i2c_unregister_device(i2c_dev->i2c_client);
	if (i2c_dev->smbus_client)
		i2c_unregister_device(i2c_dev->smbus_client);
	i2c_dev->xlk_client = NULL;
	i2c_dev->i2c_client = NULL;
	i2c_dev->smbus_client = NULL;
}

struct intel_hddl_clients **intel_hddl_get_clients(int *n_devs)
{
	if (!g_priv || !n_devs)
		return NULL;
	*n_devs = g_priv->n_hddl_devs;
	return g_priv->hddl_client;
}

void intel_hddl_unregister_pdev(struct intel_hddl_clients *c)
{
	struct intel_hddl_client_priv *priv = c->pdata;

	intel_hddl_xlink_remove_i2c_adap(&priv->pdev->dev, c);
}

static u8 *intel_tsens_thermal_msg(struct intel_hddl_clients *c,
				   struct intel_hddl_tsens_msg *msg,
					u32 *size)
{
	struct intel_hddl_client_priv *priv = c->pdata;
	struct intel_tsens **tsens = priv->tsens;

	switch (msg->msg_type) {
	case HDDL_GET_NSENS:
	{
		u32 *data;
		*size = sizeof(int);
		data = kzalloc(*size, GFP_KERNEL);
		if (!data)
			return NULL;
		*data = priv->nsens;
		return (u8 *)data;
	}
	case HDDL_GET_SENS_DETAILS:
	{
		struct intel_tsens_data *data;
		u32 sensor_type = msg->sensor_type;
		struct intel_tsens_data *tsens_data =
				&tsens[sensor_type]->data;

		*size = sizeof(struct intel_tsens_data);
		data = kzalloc(*size, GFP_KERNEL);
		if (!data)
			return NULL;
		strcpy(data->name, tsens_data->name);
		data->n_trips = tsens_data->n_trips;
		data->passive_delay = tsens_data->passive_delay;
		data->polling_delay = tsens_data->polling_delay;
		data->sensor_type = tsens_data->sensor_type;
		return (u8 *)data;
	}
	case HDDL_GET_SENS_TRIP_INFO:
	{
		struct intel_tsens_trip_info *data;
		u32 sensor_type = msg->sensor_type;
		u32 trip_info_idx = msg->trip_info_idx;

		*size = sizeof(struct intel_tsens_trip_info);
		data = kzalloc(*size, GFP_KERNEL);
		if (!data)
			return NULL;
		memcpy(data, tsens[sensor_type]->trip_info[trip_info_idx],
		       sizeof(struct intel_tsens_trip_info));
		return (u8 *)data;
	}
	default:
		break;
	}
	return NULL;
}

static int intel_hddl_i2c_register_clients(struct device *dev,
					   struct intel_hddl_clients *c)
{
	struct intel_hddl_client_priv *priv = c->pdata;
	struct xlink_handle *xlink = &c->xlink_dev;
	struct intel_hddl_tsens_msg msg;
	int rc, i;
	int size;

	/* Get msg type */
	rc = intel_hddl_get_xlink_data(&priv->pdev->dev,
				       xlink, c->chan_num,
				       (u8 *)&msg, &size);
	if (rc)
		return rc;

	while (msg.msg_type != HDDL_GET_SENS_COMPLETE) {
		u32 *data;

		switch (msg.msg_type) {
		case HDDL_GET_N_I2C_DEVS:
		{
			size = sizeof(int);
			data = kzalloc(size, GFP_KERNEL);
			if (!data)
				return -ENOMEM;
			*data = priv->n_clients;
			break;
		}
		case HDDL_GET_I2C_DEVS:
		{
			struct intel_hddl_i2c_devs_data *i2c_dev;
			int sensor_type = msg.sensor_type;

			size = sizeof(struct intel_hddl_i2c_devs_data);
			i2c_dev = kzalloc(size, GFP_KERNEL);
			if (!i2c_dev)
				return -ENOMEM;
			strcpy(i2c_dev->name,
			       priv->i2c_devs[sensor_type]->name);
			i2c_dev->addr = priv->i2c_devs[sensor_type]->addr;
			i2c_dev->bus = priv->i2c_devs[sensor_type]->bus;
			i2c_dev->enabled =
				priv->i2c_devs[sensor_type]->enabled;
			i2c_dev->local_host =
				priv->i2c_devs[sensor_type]->local_host;
			i2c_dev->remote_host =
				priv->i2c_devs[sensor_type]->remote_host;
			data = (u32 *)i2c_dev;
			break;
		}
		default:
			dev_err(&priv->pdev->dev,
				"HDDL: Invalid msg received\n");
			return -EINVAL;
		}
		rc = xlink_write_volatile(xlink, c->chan_num,
					  (u8 *)data, size);
		if (rc) {
			dev_err(&priv->pdev->dev,
				"xlink write data failed rc = %d\n",
				rc);
			return rc;
		}
		kfree(data);
		rc = intel_hddl_get_xlink_data(&priv->pdev->dev,
					       xlink, c->chan_num,
					       (u8 *)&msg, &size);
		if (rc)
			return rc;
	}

	for (i = 0; i < priv->n_clients; i++) {
		msg.msg_type = HDDL_GET_I2C_DEV_ADDR;
		msg.sensor_type = i;
		rc = xlink_write_volatile(xlink, c->chan_num,
					  (u8 *)&msg, sizeof(msg));
		if (rc) {
			dev_err(&priv->pdev->dev,
				"xlink write data failed rc = %d\n",
				rc);
			return rc;
		}
		rc = intel_hddl_get_xlink_data(&priv->pdev->dev,
					       xlink, c->chan_num,
					       (u8 *)&priv->i2c_devs[i]->addr,
					       &size);
		if (rc)
			return rc;
		priv->i2c_devs[i]->board_info.addr = priv->i2c_devs[i]->addr;
	}
	/* Send Complete */
	msg.msg_type = HDDL_GET_SENS_COMPLETE;
	rc = xlink_write_volatile(xlink, c->chan_num,
				  (u8 *)&msg, sizeof(msg));
	if (rc) {
		dev_err(&priv->pdev->dev,
			"xlink write data failed rc = %d\n",
			rc);
		return rc;
	}

	intel_hddl_add_xlink_i2c_clients(&priv->pdev->dev, c, priv->i2c_devs,
					 priv->n_clients, 0);
	return 0;
}

static int intel_hddl_send_tsens_data(struct intel_hddl_clients *c)
{
	struct intel_hddl_client_priv *priv = c->pdata;
	struct xlink_handle *xlink = &c->xlink_dev;
	struct intel_hddl_tsens_msg msg;
	u32 size;
	u8 *data;
	int rc;

	/* Get msg type */
	rc = intel_hddl_get_xlink_data(&priv->pdev->dev,
				       xlink, c->chan_num,
				       (u8 *)&msg, &size);
	if (rc)
		return rc;

	while (msg.msg_type != HDDL_GET_SENS_COMPLETE) {
		data = intel_tsens_thermal_msg(c, &msg, &size);
		if (!data) {
			dev_err(&priv->pdev->dev, "HDDL: failed to get details\n");
			return -EINVAL;
		}
		rc = xlink_write_volatile(xlink, c->chan_num,
					  (u8 *)data, size);
		if (rc) {
			dev_err(&priv->pdev->dev,
				"xlink write data failed rc = %d\n",
				rc);
			return rc;
		}
		kfree(data);
		rc = intel_hddl_get_xlink_data(&priv->pdev->dev,
					       xlink, c->chan_num,
					       (u8 *)&msg, &size);
		if (rc)
			return rc;
	}

	return 0;
}

static int intel_hddl_device_connect_task(void *data)
{
	struct intel_hddl_clients *c = (struct intel_hddl_clients *)data;
	struct intel_hddl_client_priv *priv = c->pdata;
	struct intel_hddl_board_info board_info_rcvd;
	struct xlink_handle *xlink = &c->xlink_dev;
	struct timespec64 ts;
	u32 size, rc;

	memcpy(&c->board_info, &priv->board_info,
	       sizeof(struct intel_hddl_board_info));
	c->chan_num = priv->xlink_chan;
	c->i2c_chan_num = priv->i2c_xlink_chan;
	c->i2c_devs = priv->i2c_devs;
	c->n_clients = priv->n_clients;
	if (intel_hddl_open_xlink_device(&priv->pdev->dev, c)) {
		dev_err(&priv->pdev->dev, "HDDL open xlink dev failed\n");
		return -EINVAL;
	}
	size = sizeof(ts);
	rc = intel_hddl_get_xlink_data(&priv->pdev->dev,
				       xlink, c->chan_num,
				       (u8 *)&ts, &size);
	if (rc)
		goto close_xlink_dev;
	do_settimeofday64(&ts);

	rc = xlink_write_volatile(xlink, c->chan_num,
				  (u8 *)&c->board_info,
				  sizeof(struct intel_hddl_board_info));
	if (rc) {
		dev_err(&priv->pdev->dev,
			"xlink write data failed rc = %d\n",
			rc);
		goto close_xlink_dev;
	}

	size = sizeof(board_info_rcvd);
	rc = intel_hddl_get_xlink_data(&priv->pdev->dev,
				       xlink, c->chan_num,
				       (u8 *)&board_info_rcvd,
				       &size);
	if (rc)
		goto close_xlink_dev;
	rc = intel_hddl_send_tsens_data(c);
	if (rc) {
		dev_err(&priv->pdev->dev, "HDDL: tsens data not sent\n");
		goto close_xlink_dev;
	}
	rc = intel_hddl_register_xlink_i2c_adap(&priv->pdev->dev, c);
	if (rc) {
		dev_err(&priv->pdev->dev,
			"HDDL: register xlink i2c adapter failed\n");
		goto remove_xlink_i2c_adap;
	}
	rc = intel_hddl_i2c_register_clients(&priv->pdev->dev, c);
	if (rc) {
		dev_err(&priv->pdev->dev,
			"HDDL: register i2c clients failed\n");
		goto remove_xlink_i2c_adap;
	}

	return 0;
remove_xlink_i2c_adap:
	intel_hddl_xlink_remove_i2c_adap(&priv->pdev->dev, c);
close_xlink_dev:
	intel_hddl_close_xlink_device(&priv->pdev->dev, c);
	return rc;
}

static int intel_hddl_check_for_new_device(struct intel_hddl_client_priv *priv)
{
	struct intel_hddl_clients **hddl_clients;

	hddl_clients =
		intel_hddl_setup_device(&priv->pdev->dev,
					intel_hddl_device_connect_task,
					&priv->n_hddl_devs, priv->hddl_client,
					priv);
	if (!hddl_clients) {
		dev_err(&priv->pdev->dev,
			"intel_hddl_setup_device returned NULL\n");
		return 0;
	}
	priv->hddl_client = hddl_clients;
	return 1;
}

static int intel_hddl_device_init_task(void *data)
{
	struct intel_hddl_client_priv *priv =
		(struct intel_hddl_client_priv *)data;

	while (!kthread_should_stop()) {
		if (!intel_hddl_check_for_new_device(priv)) {
			dev_err(&priv->pdev->dev,
				"Error while checking for new device\n");
			return -EFAULT;
		}
		msleep_interruptible(HDDL_NEW_DEV_POLL_TIME);
	}

	return 0;
}

static int intel_hddl_device_init(struct intel_hddl_client_priv *priv)
{
	priv->hddl_dev_init_task = kthread_run(intel_hddl_device_init_task,
					       (void *)priv,
					       "hddl_device_init");
	if (!priv->hddl_dev_init_task) {
		dev_err(&priv->pdev->dev, "failed to create thread\n");
		return -EINVAL;
	}

	return 0;
}

static int hddl_tsens_config_sensors(struct device_node *s_node,
				     struct intel_hddl_client_priv *priv,
				     int sensor_type)
{
	struct intel_tsens *tsens = priv->tsens[sensor_type];
	struct platform_device *pdev = priv->pdev;
	s32 trip_temp_count, trip_temp_type_c, i;
	int ret;

	tsens->data.sensor_type = sensor_type;
	if (of_property_read_u32(s_node, "passive_delay_rh",
				 &tsens->data.passive_delay)) {
		dev_err(&pdev->dev,
			"passive_delay missing in dt for %s\n",
			tsens->data.name);
		return -EINVAL;
	}
	if (of_property_read_u32(s_node, "polling_delay_rh",
				 &tsens->data.polling_delay)) {
		dev_err(&pdev->dev,
			"polling_delay missing in dt for %s\n",
			tsens->data.name);
		return -EINVAL;
	}
	trip_temp_count = of_property_count_u32_elems(s_node, "trip_temp_rh");
	trip_temp_type_c = of_property_count_strings(s_node, "trip_type_rh");
	if (trip_temp_count != trip_temp_type_c ||
	    trip_temp_count <= 0 || trip_temp_type_c <= 0) {
		dev_err(&pdev->dev,
			"trip temp config is missing in dt for %s\n",
			tsens->data.name);
		return -EINVAL;
	}

	tsens->trip_info =
		devm_kcalloc(&pdev->dev, trip_temp_count,
			     sizeof(struct intel_tsens_trip_info *),
			     GFP_KERNEL);
	if (!tsens->trip_info)
		return -ENOMEM;
	tsens->data.n_trips = trip_temp_count;
	for (i = 0; i < trip_temp_count; i++) {
		const char *trip_name;

		tsens->trip_info[i] =
			devm_kzalloc(&pdev->dev,
				     sizeof(struct intel_tsens_trip_info),
				     GFP_KERNEL);
		if (!tsens->trip_info[i])
			return -ENOMEM;
		ret = of_property_read_u32_index(s_node, "trip_temp_rh", i,
						 &tsens->trip_info[i]->temp);
		if (ret) {
			dev_err(&pdev->dev, "Invalid trip temp");
			return ret;
		}
		ret = of_property_read_string_index(s_node, "trip_type_rh", i,
						    &trip_name);
		if (ret) {
			dev_err(&pdev->dev, "Invalid trip type");
			return ret;
		}
		if (!strcmp(trip_name, "passive"))
			tsens->trip_info[i]->trip_type = THERMAL_TRIP_PASSIVE;
		else if (!strcmp(trip_name, "critical"))
			tsens->trip_info[i]->trip_type = THERMAL_TRIP_CRITICAL;
		else if (!strcmp(trip_name, "hot"))
			tsens->trip_info[i]->trip_type = THERMAL_TRIP_HOT;
		else
			tsens->trip_info[i]->trip_type = THERMAL_TRIP_ACTIVE;
	}

	return 0;
}

static int hddl_get_onchip_sensors(struct platform_device *pdev,
				   struct intel_hddl_client_priv *priv)
{
	struct device_node *s_node;
	struct device_node *np = NULL;
	int i = 0;

	s_node = of_parse_phandle(pdev->dev.of_node, "soc-sensors", 0);
	if (!s_node)
		return -EINVAL;
	priv->nsens = of_get_child_count(s_node);
	if (priv->nsens == 0) {
		dev_err(&pdev->dev, "No onchip sensors configured in dt\n");
		return -EINVAL;
	}
	priv->tsens =
		devm_kcalloc(&pdev->dev, priv->nsens,
			     sizeof(struct intel_tsens *),
			     GFP_KERNEL);
	if (!priv->tsens)
		return -ENOMEM;
	for_each_child_of_node(s_node, np) {
		struct intel_tsens *tsens;

		tsens = devm_kzalloc(&pdev->dev,
				     sizeof(struct intel_tsens),
				     GFP_KERNEL);
		if (!tsens)
			return -ENOMEM;
		priv->tsens[i] = tsens;
		strcpy(tsens->data.name, np->name);
		if (hddl_tsens_config_sensors(np, priv, i)) {
			dev_err(&pdev->dev,
				"Missing sensor info in dts for %s\n",
				tsens->data.name);
			return -EINVAL;
		}
		i++;
	}

	return 0;
}

static int intel_hddl_get_board_type(struct platform_device *pdev,
				     struct intel_hddl_client_priv *priv)
{
	int ret = 0;
	struct device_node *np_bios;
	const char *board_type;
	const char *board_type_var = NULL;

	np_bios = of_find_node_by_name(NULL, "bios");
	if (np_bios) {
		if (of_property_read_bool(np_bios,
					  "system-product-name-variant")) {
			board_type_var = NULL;
			ret = of_property_read_string(np_bios, "system-product-name-variant", &board_type_var);
			if (ret < 0 || !board_type_var) {
				dev_err(&pdev->dev,
					"failed to get board variant %d",
					ret);
				return ret;
			 }
			dev_info(&pdev->dev,
				"Board type var is %s\n", board_type_var);
		}

		ret = of_property_read_string(np_bios, "system-product-name", &board_type);
		if (ret < 0 || !board_type) {
			dev_err(&pdev->dev,
				"failed to get board type %d",
				ret);
			return ret;
		} else {
			strncpy(priv->board_info.board_type, board_type, 15);
			if (board_type_var)
				strncat(priv->board_info.board_type,
					board_type_var,
					15 - strlen(priv->board_info.board_type));
			dev_info(&pdev->dev,
					"Board type is %s\n", priv->board_info.board_type);
		}
	} else {
		dev_err(&pdev->dev,
			"node pointer for bios is not found\n");
		ret = -EINVAL;
	}
	return ret;
}

static int intel_hddl_get_ids(struct platform_device *pdev,
			      struct intel_hddl_client_priv *priv)
{
	int ret;
	struct gpio_descs *board_id_gpios;
	struct gpio_descs *soc_id_gpios;
	unsigned long values = 0;

	board_id_gpios =
		gpiod_get_array_optional(&pdev->dev, "board-id",
					 GPIOD_IN);
	if (board_id_gpios) {
		ret = gpiod_get_array_value(board_id_gpios->ndescs,
					    board_id_gpios->desc,
					    NULL, &values);
		if (ret) {
			dev_err(&pdev->dev,
				"failed to get boardid values %d",
				ret);
			return ret;
		}
		priv->board_info.board_id = values;
		priv->board_id = priv->board_info.board_id;
	}
	soc_id_gpios =
		gpiod_get_array_optional(&pdev->dev, "soc-id",
					 GPIOD_IN);
	if (soc_id_gpios) {
		values = 0;
		ret = gpiod_get_array_value(soc_id_gpios->ndescs,
					    soc_id_gpios->desc,
					    NULL, &values);
		if (ret) {
			dev_err(&pdev->dev,
				"failed to get soc-id values %d",
				ret);
			return ret;
		}
		priv->board_info.soc_id = values;
		priv->soc_id = priv->board_info.soc_id;
	}

	return 0;
}

static int hddl_get_socreset_data(struct platform_device *pdev,
				  struct intel_hddl_client_priv *priv,
				  struct device_node *np)
{
	struct device_node *soc_node;

	for_each_child_of_node(np, soc_node) {
		struct intel_hddl_board_info *board_info = &priv->board_info;
		int soc_id;

		of_property_read_u32(soc_node, "id", &soc_id);
		if (board_info->soc_id == soc_id) {
			const char *name;

			of_property_read_string_index(soc_node, "io-exp-name",
						      0, &name);
			strcpy(board_info->iox_name, name);
			of_property_read_u32_index(soc_node, "io-exp-addr",
						   0, &board_info->iox_addr);
			of_property_read_u32_index(soc_node, "io-exp-addr",
						   1, &board_info->iox_pin);
			of_property_read_u32_index(soc_node, "pci-switch",
						   0, &board_info->pci_pin);
			return 0;
		}
	}
	return -EINVAL;
}

static int hddl_get_board_reset_data(struct platform_device *pdev,
				     struct intel_hddl_client_priv *priv)
{
	struct device_node *s_node;
	struct device_node *np;
	int n_boards;

	s_node = of_parse_phandle(pdev->dev.of_node, "soc-reset", 0);
	if (!s_node)
		return -EINVAL;
	n_boards = of_get_child_count(s_node);
	if (n_boards == 0) {
		dev_err(&pdev->dev, "Board reset data not available in dt\n");
		return -EINVAL;
	}

	for_each_child_of_node(s_node, np) {
		int id;

		of_property_read_u32(np, "id", &id);
		if (priv->board_info.board_id == id) {
			return hddl_get_socreset_data(pdev, priv, np);
		}
	}
	return -EINVAL;
}

static int intel_hddl_config_dt(struct intel_hddl_client_priv *priv)
{
	struct platform_device *pdev = priv->pdev;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *s_node = NULL;
	struct resource *res;
	int i, ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res) {
		priv->base_addr = ioremap(res->start,
					  (res->end - res->start));
	}
	priv->board_info.soc_id = 0;
	priv->board_info.board_id = 0;
	ret = of_property_read_u32(np, "xlink_chan",
				   &priv->xlink_chan);
	if (ret) {
		dev_err(&pdev->dev, "xlink channel not available in dt");
		return ret;
	}
	ret = of_property_read_u32(np, "i2c_xlink_chan",
				   &priv->i2c_xlink_chan);
	if (ret) {
		dev_err(&pdev->dev, "i2c xlink channel not available in dt");
		return ret;
	}

	ret = intel_hddl_get_board_type(pdev, priv);
	if (ret) {
		dev_err(&pdev->dev, "Unable to get board type");
		return ret;
	}
	ret = intel_hddl_get_ids(pdev, priv);
	if (ret) {
		dev_err(&pdev->dev, "Unable to get board/soc id");
		return ret;
	}
	priv->soc_reset_available = of_property_read_bool(np, "soc-reset");
	if (priv->soc_reset_available) {
		ret = hddl_get_board_reset_data(pdev, priv);
		if (ret) {
			dev_err(&pdev->dev, "Unable to get reset data");
			return ret;
		}
	} else
		dev_err(&pdev->dev, "This platform does not support soc-reset\n");

	ret = hddl_get_onchip_sensors(pdev, priv);
	if (ret) {
		dev_err(&pdev->dev, "Onchip sensor config failed");
		return ret;
	}
	priv->n_clients = of_get_child_count(np);
	priv->i2c_devs = devm_kcalloc(&pdev->dev, priv->n_clients,
				      sizeof(struct intel_hddl_i2c_devs *),
				      GFP_KERNEL);
	if (!priv->i2c_devs)
		return -ENOMEM;
	i = 0;
	for_each_child_of_node(np, s_node) {
		const char *status;
		struct intel_hddl_i2c_devs *i2c_dev;

		i2c_dev = devm_kzalloc(&pdev->dev,
				       sizeof(struct intel_hddl_i2c_devs),
				       GFP_KERNEL);
		if (!i2c_dev)
			return -ENOMEM;
		of_property_read_string_index(s_node, "status", 0,
					      &status);
		if (!strcmp(status, "okay")) {
			u32 addr;
			const char *name = NULL;

			i2c_dev->enabled = 1;
			of_property_read_string_index(s_node, "compatible", 0,
						      &name);
			if (name) {
				strcpy(i2c_dev->name, name);
				strcpy(i2c_dev->board_info.type,
				       i2c_dev->name);
			}
			/**
			 * below dt params are optional.
			 */
			of_property_read_u32(s_node, "reg", &addr);
			i2c_dev->board_info.addr = addr;
			i2c_dev->addr = addr;
			of_property_read_u32(s_node, "bus",
					     &i2c_dev->bus);
			of_property_read_u32(s_node, "remote-host",
					     &i2c_dev->remote_host);
			of_property_read_u32(s_node, "local-host",
					     &i2c_dev->local_host);
		}
		priv->i2c_devs[i] = i2c_dev;
		i++;
	}
	return 0;
}

static int intel_hddl_client_probe(struct platform_device *pdev)
{
	struct intel_hddl_client_priv *priv;
	int ret;

	priv = devm_kzalloc(&pdev->dev,
			    sizeof(struct intel_hddl_client_priv),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->pdev = pdev;
	if (pdev->dev.of_node) {
		ret = intel_hddl_config_dt(priv);
		if (ret) {
			dev_err(&pdev->dev, "dt configuration failed\n");
			devm_kfree(&pdev->dev, priv);
			return ret;
		}
	} else {
		dev_err(&pdev->dev,
			"Non Device Tree build is not supported\n");
		devm_kfree(&pdev->dev, priv);
		return -EINVAL;
	}
	ret = intel_hddl_device_init(priv);
	if (ret) {
		dev_err(&pdev->dev, "HDDL device init failed\n");
		devm_kfree(&pdev->dev, priv);
		return -EINVAL;
	}
	g_priv = priv;
	platform_set_drvdata(pdev, priv);
	return 0;
}

/* Device Exit */
static int intel_hddl_client_exit(struct platform_device *pdev)
{
	int k;
	struct intel_hddl_client_priv *priv = platform_get_drvdata(pdev);

	if (!priv)
		return -EINVAL;
	for (k = 0; k < priv->n_hddl_devs; k++) {
		struct intel_hddl_clients *d = priv->hddl_client[k];

		intel_hddl_device_remove(d);
	}

	return 0;
}

static const struct of_device_id intel_hddl_client_id_table[] = {
	{ .compatible = "intel,hddl-client" },
	{}
};
MODULE_DEVICE_TABLE(of, intel_hddl_client_id_table);

static struct platform_driver intel_hddl_client_driver = {
	.probe = intel_hddl_client_probe,
	.remove = intel_hddl_client_exit,
	.driver = {
		.name = "intel_hddl_client",
		.of_match_table = intel_hddl_client_id_table,
	},
};

module_platform_driver(intel_hddl_client_driver);

MODULE_DESCRIPTION("Intel HDDL Device driver");
MODULE_AUTHOR("Sandeep Singh <sandeep1.singh@intel.com>");
MODULE_AUTHOR("Vaidya, Mahesh R <mahesh.r.vaidya@intel.com>");
MODULE_AUTHOR("Udhayakumar C <udhayakumar.c@intel.com>");
MODULE_LICENSE("GPL v2");
