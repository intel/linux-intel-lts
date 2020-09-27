// SPDX-License-Identifier: GPL-2.0-only
/*
 * HDDL Device Kernel module.
 *
 * Copyright (C) 2019-2020 Intel Corporation
 */

#include <asm/page.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/printk.h>
#include <uapi/linux/stat.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/xlink.h>
#include <linux/time.h>
#include <linux/kmod.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/sched/mm.h>
#include <linux/sched/task.h>
#include <linux/hddl_device.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/gpio/consumer.h>
#include <linux/workqueue.h>
#include <linux/hddl_device.h>
#include "hddl_device_helper.h"

#define DRIVER_NAME "hddl_device_client"

struct intel_hddl_client_priv {
	void __iomem *base_addr;
	int board_id;
	int soc_id;
	uint32_t xlink_chan;
	uint32_t i2c_xlink_chan;
	struct platform_device *pdev;
	struct intel_hddl_clients **hddl_client;
	uint32_t n_hddl_devs;
	struct task_struct *hddl_dev_init_task;
	struct intel_hddl_i2c_devs **i2c_devs;
	int n_slaves;
	struct intel_hddl_board_info board_info;
};

static struct intel_hddl_client_priv *g_priv;

static struct intel_hddl_clients **intel_hddl_get_device(int *n_devs)
{
	if (!g_priv || !n_devs)
		return NULL;
	*n_devs = g_priv->n_hddl_devs;
	return g_priv->hddl_client;
}

static int intel_hddl_unregister_pdev(struct intel_hddl_clients *c)
{
	struct intel_hddl_client_priv *priv = c->pdata;

	intel_hddl_xlink_remove_i2c_adap(&priv->pdev->dev, c);
	return 0;
}

static uint8_t *intel_tsens_thermal_msg(struct intel_hddl_clients *c,
		struct intel_hddl_tsens_msg *msg, uint32_t *size)
{
	struct intel_tsens **tsens = (struct intel_tsens **)c->tsens;

	switch (msg->msg_type) {
	case HDDL_GET_NSENS:
	{
		uint32_t *data;
		*size = sizeof(int);
		data = kzalloc(*size, GFP_KERNEL);
		if (!data)
			return NULL;
		*data = c->nsens;
		return (uint8_t *)data;
	}
		break;
	case HDDL_GET_SENS_DETAILS:
	{
		struct intel_tsens_data *data;
		uint32_t index = msg->index;

		*size = sizeof(struct intel_tsens_data);
		data = kzalloc(*size, GFP_KERNEL);
		if (!data)
			return NULL;
		strcpy(data->name, tsens[index]->name);
		data->n_trips = tsens[index]->n_trips;
		data->passive_delay = tsens[index]->passive_delay;
		data->polling_delay = tsens[index]->polling_delay;
		data->sensor_type = tsens[index]->sensor_type;
		return (uint8_t *)data;
	}
		break;
	case HDDL_GET_SENS_TRIP_INFO:
	{
		struct intel_tsens_trip_info *data;
		uint32_t index = msg->index;
		uint32_t index2 = msg->index2;

		*size = sizeof(struct intel_tsens_trip_info);
		data = kzalloc(*size, GFP_KERNEL);
		if (!data)
			return NULL;
		memcpy(data, tsens[index]->trip_info[index2],
				sizeof(struct intel_tsens_trip_info));
		return (uint8_t *)data;
	}
		break;

	default:
		break;
	}
	return NULL;
}

static int intel_hddl_i2c_clients(struct device *dev,
				struct intel_hddl_clients *c)
{
	int rc, i;
	struct intel_hddl_tsens_msg msg;
	int size;
	struct xlink_handle *xlink = &c->xlink_dev;
	struct intel_hddl_client_priv *priv = c->pdata;

	/* Get msg type */
	rc = xlink_read_data_to_buffer(xlink, c->chan_num,
	(uint8_t *) &msg, &size);
	xlink_release_data(xlink, c->chan_num, NULL);

	while (msg.msg_type != HDDL_GET_SENS_COMPLETE) {
		uint32_t *data;

		switch (msg.msg_type) {
		case HDDL_GET_N_I2C_DEVS:
		{
			size = sizeof(int);
			data = kzalloc(size, GFP_KERNEL);
			if (!data) {
				dev_err(&priv->pdev->dev, "HDDL: failed to get details\n");
				return -EINVAL;
			}
			*data = priv->n_slaves;
		}
			break;
		case HDDL_GET_I2C_DEVS:
		{
			struct intel_hddl_i2c_devs_data *t;
			int index = msg.index;

			size = sizeof(struct intel_hddl_i2c_devs_data);
			t = kzalloc(size, GFP_KERNEL);
			if (!t) {
				dev_err(&priv->pdev->dev, "HDDL: failed to get details\n");
				return -EINVAL;
			}
			strcpy(t->name, priv->i2c_devs[index]->name);
			t->addr = priv->i2c_devs[index]->addr;
			t->bus = priv->i2c_devs[index]->bus;
			t->enabled = priv->i2c_devs[index]->enabled;
			t->local_host = priv->i2c_devs[index]->local_host;
			t->remote_host = priv->i2c_devs[index]->remote_host;
			data = (uint32_t *)t;

		}
			break;
		default:
			break;
		}
		rc = xlink_write_volatile(xlink, c->chan_num,
			(uint8_t *)data, size);
		dev_dbg(&priv->pdev->dev,
			"HDDL: xlink_write_data complete [%d].\n",
			rc);
		kfree(data);
		rc = xlink_read_data_to_buffer(xlink, c->chan_num,
		(uint8_t *) &msg, &size);
		xlink_release_data(xlink, c->chan_num, NULL);
	}

	for (i = 0; i < priv->n_slaves; i++) {
		msg.msg_type = HDDL_GET_I2C_DEV_ADDR;
		msg.index = i;
		rc = xlink_write_volatile(xlink, c->chan_num,
				(uint8_t *)&msg, sizeof(msg));
		rc = xlink_read_data_to_buffer(xlink, c->chan_num,
				(uint8_t *) &priv->i2c_devs[i]->addr, &size);
		dev_info(&priv->pdev->dev, "HDDL: Slave addr for %s is %d\n",
					priv->i2c_devs[i]->name,
					priv->i2c_devs[i]->addr);
		priv->i2c_devs[i]->board_info.addr = priv->i2c_devs[i]->addr;
		xlink_release_data(xlink, c->chan_num, NULL);
	}
	/* Send Complete */
	msg.msg_type = HDDL_GET_SENS_COMPLETE;
	rc = xlink_write_volatile(xlink, c->chan_num,
				(uint8_t *)&msg, sizeof(msg));

	rc = intel_hddl_xlink_i2c_clients(&priv->pdev->dev, c, priv->i2c_devs,
			priv->n_slaves, 0);
	if (rc) {
		dev_err(&priv->pdev->dev, "HDDL: register i2c slaves failed\n");
		return -ENODEV;
	}
	return 0;
}

static int intel_hddl_tsens_data(struct intel_hddl_clients *c)
{
	int rc;
	struct intel_hddl_client_priv *priv = c->pdata;
	struct xlink_handle *xlink = &c->xlink_dev;
	uint32_t size;
	uint8_t *data;
	struct intel_hddl_tsens_msg msg;

	/* Get msg type */
	rc = xlink_read_data_to_buffer(xlink, c->chan_num,
	(uint8_t *) &msg, &size);
	xlink_release_data(xlink, c->chan_num, NULL);

	while (msg.msg_type != HDDL_GET_SENS_COMPLETE) {
		data = intel_tsens_thermal_msg(c, &msg, &size);
		if (!data) {
			dev_err(&priv->pdev->dev, "HDDL: failed to get details\n");
			return -EINVAL;
		}
		rc = xlink_write_volatile(xlink, c->chan_num,
			(uint8_t *)data, size);
		dev_dbg(&priv->pdev->dev, "HDDL: %s write complete\n",
					__func__);
		kfree(data);
		rc = xlink_read_data_to_buffer(xlink, c->chan_num,
		(uint8_t *) &msg, &size);
		xlink_release_data(xlink, c->chan_num, NULL);
	}
	dev_dbg(&priv->pdev->dev, "HDDL: %s xfer complete\n",
					__func__);

	return 0;
}

static int intel_hddl_device_connect_task(void *data)
{
	struct intel_hddl_clients *c = (struct intel_hddl_clients *)data;
	struct intel_hddl_client_priv *priv = c->pdata;
	struct timespec64 ts;
	uint32_t size, rc;
	struct xlink_handle *xlink = &c->xlink_dev;
	struct intel_hddl_board_info board_info_rcvd;

	memcpy(&c->board_info, &priv->board_info,
			sizeof(struct intel_hddl_board_info));
	c->tsens = (void **)intel_tsens_hddl_register(&c->nsens);
	c->chan_num = priv->xlink_chan;
	c->i2c_chan_num = priv->i2c_xlink_chan;
	c->i2c_devs = priv->i2c_devs;
	c->n_slaves = priv->n_slaves;
	if (intel_hddl_open_xlink_device(&priv->pdev->dev, c)) {
		dev_err(&priv->pdev->dev, "HDDL open xlink dev failed\n");
		return -EINVAL;
	}
	size = sizeof(ts);
	rc = xlink_read_data_to_buffer(xlink, c->chan_num,
			(uint8_t *)&ts, &size);
	xlink_release_data(xlink, c->chan_num, NULL);

	dev_dbg(&priv->pdev->dev, "HDDL: Received %d Bytes [%d]\n", size, rc);
	dev_info(&priv->pdev->dev, "S[%llx] NS[%lx]\n", ts.tv_sec, ts.tv_nsec);
	do_settimeofday64(&ts);

	dev_dbg(&priv->pdev->dev, "HDDL: xlink_write_data to start...\n");
	rc = xlink_write_volatile(xlink, c->chan_num,
	(uint8_t *)&c->board_info, sizeof(struct intel_hddl_board_info));
	dev_dbg(&priv->pdev->dev, "HDDL: xlink_write_data complete[%d].\n",
						rc);
	dev_dbg(&priv->pdev->dev, "HDDL: xlink_read_data to start...\n");
	size = sizeof(board_info_rcvd);
	rc = xlink_read_data_to_buffer(xlink, c->chan_num,
	(uint8_t *) &board_info_rcvd, &size);
	xlink_release_data(xlink, c->chan_num, NULL);
	dev_dbg(&priv->pdev->dev,
		"HDDL: [%d]xlink_read_data completed Rcvd Size[%d]\n",
		rc, size);
	dev_info(&priv->pdev->dev, "HDDL: Board Info[%x %x]\n",
				c->board_info.board_id,
				board_info_rcvd.board_id);

	if (c->board_info.board_id ==
			~(board_info_rcvd.board_id)) {
		dev_info(&priv->pdev->dev, "HDDL: Handshake Complete = %x\n",
		c->board_info.board_id);
		dev_info(&priv->pdev->dev,
		"HDDL: Board[%x] Soc[%x] DevType[%x]\n",
			c->board_info.board_id,
			c->board_info.soc_id,
			xlink->dev_type
		);
	}
	rc = intel_hddl_tsens_data(c);
	if (rc) {
		dev_err(&priv->pdev->dev, "HDDL: tsens data not sent\n");
		return -EINVAL;
	}
	rc = intel_hddl_xlink_i2c_adap(&priv->pdev->dev, c);
	if (rc) {
		dev_err(&priv->pdev->dev,
			"HDDL: register xlink i2c adapter failed\n");
		return -EINVAL;
	}
	rc = intel_hddl_i2c_clients(&priv->pdev->dev, c);
	if (rc) {
		dev_err(&priv->pdev->dev,
			"HDDL: register i2c slaves failed\n");
		return -EINVAL;
	}
	return 0;
}

static int intel_hddl_check_for_new_device(
		struct intel_hddl_client_priv *priv)
{
	priv->hddl_client = intel_hddl_setup_device(&priv->pdev->dev,
				intel_hddl_device_connect_task,
				&priv->n_hddl_devs, priv->hddl_client, priv);

	if (priv->hddl_client == NULL)
		dev_err(&priv->pdev->dev, "intel_hddl_setup_device returned NULL\n");

	return (priv->hddl_client != NULL)?1:0;
}

static int intel_hddl_device_init_task(void *data)
{
	struct intel_hddl_client_priv *priv =
		(struct intel_hddl_client_priv *)data;

	while (true) {
		if (!intel_hddl_check_for_new_device(priv)) {
			dev_err(&priv->pdev->dev, "intel_hddl_setup_device returned NULL\n");
			break;
		}
		msleep_interruptible(5000);
	}
	dev_err(&priv->pdev->dev, "Setup HDDL client devices failed\n");

	return -EINVAL;
}

static int intel_hddl_device_init(struct intel_hddl_client_priv *priv)
{
	priv->hddl_dev_init_task = kthread_run(
					intel_hddl_device_init_task,
					(void *)priv,
					"hddl_device_init");
	if (!priv->hddl_dev_init_task) {
		dev_err(&priv->pdev->dev, "failed to create thread\n");
		return -EINVAL;
	}

	return 0;
}

static int intel_hddl_config_dt(struct intel_hddl_client_priv *priv)
{
	struct platform_device *pdev = priv->pdev;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *s_node;
	struct resource *res;
	struct gpio_descs *board_id_gpios;
	struct gpio_descs *soc_id_gpios;
	int gpio_cfg_count, board_cfg_count, soc_cfg_count, i;
	int ret;
	unsigned long values;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res) {
		priv->base_addr = ioremap(res->start,
				(res->end - res->start));
	}
	priv->board_info.soc_id = 0;
	priv->board_info.board_id = 0;
	of_property_read_u32(np, "xlink_chan",
				&priv->xlink_chan);
	of_property_read_u32(np, "i2c_xlink_chan",
				&priv->i2c_xlink_chan);
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
	} else {
		dev_info(&pdev->dev, "board-id cfg is not enabled in dts\n");
	}
	soc_id_gpios =
		gpiod_get_array_optional(&pdev->dev, "soc-id",
				GPIOD_IN);
	if (soc_id_gpios) {
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
	} else {
		dev_info(&pdev->dev, "soc-id cfg is not enabled in dts\n");
	}

	gpio_cfg_count = of_property_count_u32_elems(np, "gpio-cfg-table");
	if (gpio_cfg_count > 0) {
		u32 offset, setting;

		for (i = 0; i < (gpio_cfg_count/2); i++) {
			u32 index = (i*2);

			of_property_read_u32_index(np,
					"gpio-cfg-table", index,
					&offset);
			of_property_read_u32_index(np,
					"gpio-cfg-table", index+1,
					&setting);
			iowrite32(setting, (priv->base_addr + offset));
		}
	}
	board_cfg_count = of_property_count_u32_elems(np, "board-cfg-table");
	if (board_cfg_count > 0) {
		u32 offset, bit_pos, bit_mask, val;

		of_property_read_u32_index(np, "board-cfg-table", 0,
						&offset);
		of_property_read_u32_index(np, "board-cfg-table", 1,
						&bit_pos);
		of_property_read_u32_index(np, "board-cfg-table", 2,
						&bit_mask);
		val = ioread32((priv->base_addr + offset));
		priv->board_info.board_id = (val >> bit_pos) & bit_mask;
		priv->board_id = priv->board_info.board_id;
	}
	soc_cfg_count = of_property_count_u32_elems(np, "soc-cfg-table");
	if (soc_cfg_count > 0) {
		u32 offset, bit_pos, bit_mask, val;

		of_property_read_u32_index(np, "soc-cfg-table", 0,
						&offset);
		of_property_read_u32_index(np, "soc-cfg-table", 1,
						&bit_pos);
		of_property_read_u32_index(np, "soc-cfg-table", 2,
						&bit_mask);
		val = ioread32((priv->base_addr + offset));
		priv->board_info.soc_id = (val >> bit_pos) & bit_mask;
		priv->soc_id = priv->board_info.soc_id;
	}
	dev_info(&pdev->dev, "Board ID %d, Soc ID %d\n",
				priv->board_id,
				priv->soc_id);

	priv->n_slaves = of_get_child_count(np);
	priv->i2c_devs = devm_kzalloc(&pdev->dev,
				(sizeof(struct intel_hddl_i2c_devs *) *
				 priv->n_slaves),
				GFP_KERNEL);
	if (!priv->i2c_devs)
		return -ENOMEM;

	i = 0;
	for_each_child_of_node(np, s_node) {
		const char *status;

		priv->i2c_devs[i] = devm_kzalloc(&pdev->dev,
					sizeof(struct intel_hddl_i2c_devs),
					GFP_KERNEL);
		if (priv->i2c_devs[i] == NULL) {
			dev_err(&pdev->dev, "Memory alloc failed for %s\n",
				s_node->name);
			ret = -ENOMEM;
			goto f_i2c_dev;
		}
		of_property_read_string_index(s_node, "status", 0,
				&status);
		if (!strcmp(status, "okay")) {
			uint32_t addr;
			const char *name = NULL;

			priv->i2c_devs[i]->enabled = 1;
			of_property_read_string_index(s_node, "compatible", 0,
					&name);
			if (name) {
				strcpy(priv->i2c_devs[i]->name, name);
				strcpy(priv->i2c_devs[i]->board_info.type,
					priv->i2c_devs[i]->name);
			}
			of_property_read_u32(s_node, "reg", &addr);
			priv->i2c_devs[i]->board_info.addr = addr;
			priv->i2c_devs[i]->addr = addr;
			of_property_read_u32(s_node, "bus",
					&priv->i2c_devs[i]->bus);
			of_property_read_u32(s_node, "remote-host",
					&priv->i2c_devs[i]->remote_host);
			of_property_read_u32(s_node, "local-host",
					&priv->i2c_devs[i]->local_host);
		}
		i++;
	}
	return 0;
f_i2c_dev:
	return ret;
}

static int intel_hddl_client_probe(struct platform_device *pdev)
{
	struct intel_hddl_client_priv *priv;
	int ret;

	dev_info(&pdev->dev,
		"%s Entry\n", __func__);
	priv = devm_kzalloc(&pdev->dev,
				sizeof(struct intel_hddl_client_priv),
				GFP_KERNEL);
	if (priv == NULL) {
		dev_err(&pdev->dev, "No memory");
		return -ENOMEM;
	}
	priv->pdev = pdev;
	g_priv = priv;
	if (pdev->dev.of_node) {
		ret = intel_hddl_config_dt(priv);
		if (ret) {
			dev_err(&pdev->dev, "dt configuration failed\n");
			devm_kfree(&pdev->dev, priv);
			return ret;
		}
	} else {
		dev_err(&pdev->dev, "Non Device Tree build is not supported\n");
		devm_kfree(&pdev->dev, priv);
		return -EINVAL;
	}
	ret = intel_hddl_device_init(priv);
	if (ret) {
		dev_err(&pdev->dev, "HDDL device init failed\n");
		devm_kfree(&pdev->dev, priv);
		return -EINVAL;
	}
	platform_set_drvdata(pdev, priv);
	return 0;
}

/* Device Exit */
static int intel_hddl_client_exit(struct platform_device *pdev)
{
	int k;
	struct intel_hddl_client_priv *priv = platform_get_drvdata(pdev);

	dev_info(&pdev->dev,
		"%s Entry\n", __func__);
	if (!priv)
		return -EINVAL;
	for (k = 0; k < priv->n_hddl_devs; k++) {
		struct intel_hddl_clients *d = priv->hddl_client[k];

		intel_hddl_device_remove(d);
	}
	dev_info(&pdev->dev,
		"%s Exit\n", __func__);

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

MODULE_DESCRIPTION("KeemBay HDDL Device driver");
MODULE_AUTHOR("Raja Subramanian, Lakshmi Bai"
	      "<lakshmi.bai.raja.subramanian@intel.com>");
MODULE_AUTHOR("Sandeep Singh <sandeep1.singh@intel.com>");
MODULE_AUTHOR("Vaidya, Mahesh R <mahesh.r.vaidya@intel.com>");
MODULE_AUTHOR("Udhayakumar C <udhayakumar.c@intel.com>");
MODULE_LICENSE("GPL v2");
