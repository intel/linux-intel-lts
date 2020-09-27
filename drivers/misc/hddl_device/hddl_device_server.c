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
#include <linux/workqueue.h>
#include <linux/thermal.h>
#include <linux/hddl_device.h>
#include <linux/intel_tsens_host.h>
#include "hddl_device_helper.h"

#define DRIVER_NAME "hddl_device_server"

#define HDDL_NODE_XLINK_CHANNEL 1080
#define HDDL_I2C_XLINK_CHANNEL 1081

struct intel_hddl_server_plat_data {
	uint32_t xlink_chan;
	uint32_t i2c_xlink_chan;
};

struct intel_hddl_device_priv {
	uint32_t xlink_chan;
	uint32_t i2c_xlink_chan;
	struct mutex lock;
	struct platform_device *pdev;
	uint32_t n_hddl_devs;
	struct intel_hddl_clients **hddl_client;
	struct task_struct *hddl_dev_init_task;
	struct intel_hddl_server_plat_data *plat_data;
	dev_t cdev;
	struct class *dev_class;
	struct cdev hddl_cdev;
};

static struct intel_hddl_device_priv *g_priv;

static struct intel_hddl_clients **intel_hddl_get_device(int *n_devs)
{
	if (!g_priv)
		return NULL;
	*n_devs = g_priv->n_hddl_devs;
	return g_priv->hddl_client;
}

static long hddl_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct intel_hddl_device_priv *priv = file->private_data;
	struct intel_hddl_clients **c = NULL;
	struct intel_hddl_clients *d = NULL;
	int i, rc = 0;
	T_SW_ID_HDDL_DATA swid_data = {0};
	T_SW_ID_SOFT_RESET soft_reset = {0};

	if ((int32_t *)arg == NULL) {
		pr_err("HDDL: Null pointer from user\n");
		return -EINVAL;
	}
	if (!priv) {
		pr_err("HDDL: Device ioctl failed\n");
		return -ENODEV;
	}
	c = priv->hddl_client;
	if (!c) {
		pr_err("HDDL: Device ioctl failed\n");
		return -ENODEV;
	}
	switch (cmd) {
	case HDDL_SOFT_RESET:
		if (copy_from_user(&soft_reset,
				(int32_t *)arg, sizeof(T_SW_ID_SOFT_RESET)))
			return -EFAULT;
		for (i = 0; i < priv->n_hddl_devs; i++) {
			if (c[i]->xlink_dev.sw_device_id ==
					soft_reset.sw_id) {
				d = c[i];
				break;
			}
		}

		if (!d) {
			pr_err("HDDL: Device ioctl failed\n");
			return -ENODEV;
		}
		/* xlink-reset */
		rc =  xlink_reset_device(&d->xlink_dev);
		if (rc > 0)
			pr_err("HDDL : xlink_reset_device failed");
		else
			soft_reset.return_id = 1;
		if (copy_to_user((T_SW_ID_SOFT_RESET *) arg,
				&soft_reset, sizeof(T_SW_ID_SOFT_RESET)))
			return -EFAULT;
		/* xlink-rest */
		break;
	case HDDL_READ_SW_ID_DATA:
		if (copy_from_user(&swid_data, (int32_t *)arg,
			sizeof(T_SW_ID_HDDL_DATA)))
			return -EFAULT;
		for (i = 0; i < priv->n_hddl_devs; i++) {
			if (c[i]->xlink_dev.sw_device_id ==
					swid_data.sw_id) {
				d = c[i];
				break;
			}
		}

		swid_data.board_id = d->board_info.board_id;
		swid_data.soc_id = d->board_info.soc_id;
		if (d->adap[0])
			swid_data.soc_adaptor_no[0] = d->adap[0]->nr;
		if (d->adap[1])
			swid_data.soc_adaptor_no[1] = d->adap[1]->nr;
		swid_data.return_id = 1;
		if (copy_to_user((T_SW_ID_HDDL_DATA *) arg,
				&swid_data, sizeof(T_SW_ID_HDDL_DATA)))
			return -EFAULT;
		break;
	}
	return 0;
}

int hddl_open(struct inode *inode, struct file *filp)
{
	struct intel_hddl_device_priv *priv;

	priv = container_of(inode->i_cdev,
			struct intel_hddl_device_priv, hddl_cdev);
	if (!priv) {
		pr_err("HDDL: Device open failed\n");
		return -ENODEV;
	}
	filp->private_data = priv;
	return 0;
}

static const struct file_operations hddl_fops = {
	.owner	= THIS_MODULE,
	.open = hddl_open,
	.unlocked_ioctl = hddl_ioctl,
};


static int intel_hddl_cdev_init(struct intel_hddl_device_priv *priv)
{
	/*Allocating Major number*/
	if ((alloc_chrdev_region(&priv->cdev, 0, 1, "Hddl_Dev")) < 0) {
		pr_err("Cannot allocate major number\n");
		return -EINVAL;
	}
	pr_info("Major = %d Minor = %d\n", MAJOR(priv->cdev),
			MINOR(priv->cdev));
	/*Creating cdev structure*/
	cdev_init(&priv->hddl_cdev, &hddl_fops);
	/*Adding character device to the system*/
	if ((cdev_add(&priv->hddl_cdev, priv->cdev, 1)) < 0) {
		pr_err("Cannot add the device to the system\n");
		goto r_class;
	}
	/*Creating struct class*/
	priv->dev_class = class_create(THIS_MODULE, "hddl_class");
	if (priv->dev_class == NULL) {
		pr_err("Cannot create the struct class\n");
		goto r_class;
	}
	/*Creating device*/
	if ((device_create(priv->dev_class, NULL, priv->cdev, NULL,
					"hddl_device")) == NULL) {
		pr_err("Cannot create the Device 1\n");
		goto r_device;
	}
	return 0;
r_device:
	class_destroy(priv->dev_class);
r_class:
	unregister_chrdev_region(priv->cdev, 1);
	return -EINVAL;
}

static void intel_hddl_cdev_remove(struct intel_hddl_device_priv *priv)
{
	device_destroy(priv->dev_class, priv->cdev);
	class_destroy(priv->dev_class);
	unregister_chrdev_region(priv->cdev, 1);
}

static int intel_hddl_unregister_pdev(struct intel_hddl_clients *c)
{
	struct intel_hddl_device_priv *priv = c->pdata;

	intel_hddl_xlink_remove_i2c_adap(&priv->pdev->dev, c);
	return 0;

}

static int host_reserved_addr[] = {
	0x42,
	0x52,
	0x54,
	0x60
};

static int hddl_get_free_slave(struct intel_hddl_i2c_devs **i2c_devs,
				int count)
{
	static int current_addr = 0x10;
	int i;
	/*Reserved: 0x00 - 0x0f
	 *           0xf0 - 0xff
	 */
	for (i = 0;
	i < ARRAY_SIZE(host_reserved_addr); i++) {
		if (current_addr == host_reserved_addr[i])
			current_addr++;
		else
			break;
	}
	for (i = 0; i < count; i++) {
		if ((i2c_devs[i]->addr & (1 << 30)) ||
			(i2c_devs[i]->addr == 0)) {
			continue;
		} else if (i2c_devs[i]->addr == current_addr) {
			current_addr++;
			continue;
		}
	}
	return current_addr++;
}

static int intel_hddl_i2c_clients(struct device *dev,
				struct intel_hddl_clients *c)
{
	int rc;
	struct intel_hddl_tsens_msg msg;
	int ndevs, size, i;
	struct intel_hddl_device_priv *priv = c->pdata;
	struct xlink_handle *xlink = &c->xlink_dev;
	struct intel_hddl_i2c_devs **i2c_devs;

	/* Get N I2C devices */
	msg.msg_type = HDDL_GET_N_I2C_DEVS;
	rc = xlink_write_volatile(xlink, c->chan_num,
					(uint8_t *)&msg, sizeof(msg));
	rc = xlink_read_data_to_buffer(xlink, c->chan_num,
	(uint8_t *) &ndevs, &size);
	dev_info(dev, "Number of i2c devices %d\n", ndevs);
	xlink_release_data(xlink, c->chan_num, NULL);
	c->n_slaves = ndevs;
	i2c_devs = devm_kzalloc(dev,
			(sizeof(struct intel_hddl_i2c_devs *) * ndevs),
			GFP_KERNEL);
	if (!i2c_devs) {
		dev_err(dev, "Memory alloc failed\n");
		return -ENOMEM;
	}
	c->i2c_devs = i2c_devs;
	for (i = 0; i < ndevs; i++) {
		struct intel_hddl_i2c_devs *i2c;
		struct intel_hddl_i2c_devs_data i2c_data;

		i2c = devm_kzalloc(dev,
				sizeof(struct intel_hddl_i2c_devs),
				GFP_KERNEL);
		if (!i2c) {
			dev_err(dev, "Memory alloc failed\n");
			return -ENOMEM;
		}
		i2c_devs[i] = i2c;

		/* Get Details*/
		msg.msg_type = HDDL_GET_I2C_DEVS;
		msg.index = i;
		rc = xlink_write_volatile(xlink, c->chan_num,
					(uint8_t *)&msg, sizeof(msg));
		rc = xlink_read_data_to_buffer(xlink, c->chan_num,
		(uint8_t *)&i2c_data, &size);
		strcpy(i2c->name, i2c_data.name);
		i2c->addr = i2c_data.addr;
		i2c->bus = i2c_data.bus;
		i2c->enabled = i2c_data.enabled;
		i2c->local_host = i2c_data.local_host;
		i2c->remote_host = i2c_data.remote_host;
		dev_info(dev, "i2c slave: [%d] [%s]\n", i, i2c->name);
		xlink_release_data(xlink, c->chan_num, NULL);
	}

	mutex_lock(&priv->lock);
	for (i = 0; i < ndevs; i++) {
		if (i2c_devs[i]->addr & (1<<30)) {
			i2c_devs[i]->addr = hddl_get_free_slave(i2c_devs,
					ndevs);
			dev_info(dev, "Slave addr for [%s] [%d]",
					i2c_devs[i]->name,
					i2c_devs[i]->addr);
		}
		strcpy(i2c_devs[i]->board_info.type,
				i2c_devs[i]->name);
		i2c_devs[i]->board_info.addr = i2c_devs[i]->addr;
	}
	mutex_unlock(&priv->lock);
	/* Send Complete */
	msg.msg_type = HDDL_GET_SENS_COMPLETE;
	rc = xlink_write_volatile(xlink, c->chan_num,
				(uint8_t *)&msg, sizeof(msg));
	mutex_lock(&priv->lock);

	/* Get msg type */
	rc = xlink_read_data_to_buffer(xlink, c->chan_num,
	(uint8_t *) &msg, &size);
	xlink_release_data(xlink, c->chan_num, NULL);
	while (msg.msg_type != HDDL_GET_SENS_COMPLETE) {
		switch (msg.msg_type) {
		case HDDL_GET_I2C_DEV_ADDR:
		{
			i = msg.index;
			rc = xlink_write_volatile(xlink, c->chan_num,
			(uint8_t *)&i2c_devs[i]->addr,
			sizeof(i2c_devs[i]->addr));
		}
		break;
		default:
			break;
		}
		rc = xlink_read_data_to_buffer(xlink, c->chan_num,
					(uint8_t *) &msg, &size);
		xlink_release_data(xlink, c->chan_num, NULL);
	}
	rc = intel_hddl_xlink_i2c_clients(dev, c, c->i2c_devs,
			c->n_slaves, 1);
	if (rc) {
		dev_err(dev, "HDDL: register i2c slaves failed\n");
		return -EINVAL;
	}
	mutex_unlock(&priv->lock);
	return 0;
}

static int intel_hddl_tsens_data(struct intel_hddl_clients *c)
{
	int rc;
	struct intel_hddl_device_priv *priv = c->pdata;
	struct xlink_handle *xlink = &c->xlink_dev;
	uint32_t size, i, j;
	uint32_t nsens;
	struct intel_hddl_tsens_msg msg;
	struct intel_tsens_host **p_tsens;

	/* Get Nsens */
	msg.msg_type = HDDL_GET_NSENS;
	rc = xlink_write_volatile(xlink, c->chan_num,
					(uint8_t *)&msg, sizeof(msg));
	rc = xlink_read_data_to_buffer(xlink, c->chan_num,
	(uint8_t *) &nsens, &size);
	dev_info(&priv->pdev->dev, "Number of sensors %d\n", nsens);
	xlink_release_data(xlink, c->chan_num, NULL);
	c->nsens = nsens;
	p_tsens = devm_kzalloc(&priv->pdev->dev,
			(sizeof(struct intel_tsens_host *) * nsens),
			GFP_KERNEL);
	if (!p_tsens) {
		dev_err(&priv->pdev->dev, "Memory alloc failed\n");
		return -ENOMEM;
	}
	c->tsens = (void **)p_tsens;
	for (i = 0; i < nsens; i++) {
		struct intel_tsens_host *tsens;
		struct intel_tsens_data *tsens_data;

		tsens = devm_kzalloc(&priv->pdev->dev,
				sizeof(struct intel_tsens_host),
				GFP_KERNEL);
		if (!tsens) {
			dev_err(&priv->pdev->dev, "Memory alloc failed\n");
			return -ENOMEM;
		}
		tsens_data = devm_kzalloc(&priv->pdev->dev,
				sizeof(struct intel_tsens_data),
				GFP_KERNEL);
		if (!tsens_data) {
			dev_err(&priv->pdev->dev, "Memory alloc failed\n");
			return -ENOMEM;
		}
		tsens->t_data = tsens_data;

		/* Get Details*/
		msg.msg_type = HDDL_GET_SENS_DETAILS;
		msg.index = i;
		rc = xlink_write_volatile(xlink, c->chan_num,
					(uint8_t *)&msg, sizeof(msg));
		rc = xlink_read_data_to_buffer(xlink, c->chan_num,
		(uint8_t *) tsens_data, &size);
		dev_info(&priv->pdev->dev, "sensor[%d]: %s\n", i,
				tsens_data->name);
		xlink_release_data(xlink, c->chan_num, NULL);

		/* Get trip info*/
		tsens->trip_info =
			devm_kzalloc(
				&priv->pdev->dev,
				(sizeof(struct intel_tsens_host_trip_info *) *
				tsens_data->n_trips),
				GFP_KERNEL);
		if (!tsens->trip_info) {
			dev_err(&priv->pdev->dev, "Memory alloc failed\n");
			return -ENOMEM;
		}
		for (j = 0; j < tsens_data->n_trips; j++) {
			struct intel_tsens_host_trip_info *t_info;

			t_info = devm_kzalloc(&priv->pdev->dev,
				sizeof(struct intel_tsens_host_trip_info),
				GFP_KERNEL);
			if (!t_info) {
				dev_err(&priv->pdev->dev, "Memory alloc failed\n");
				return -ENOMEM;
			}
			tsens->trip_info[j] = t_info;
			msg.msg_type = HDDL_GET_SENS_TRIP_INFO;
			msg.index = i;
			msg.index2 = j;
			rc = xlink_write_volatile(xlink, c->chan_num,
					(uint8_t *)&msg, sizeof(msg));
			rc = xlink_read_data_to_buffer(xlink, c->chan_num,
				(uint8_t *) t_info, &size);
			dev_info(&priv->pdev->dev, "trip[%d]: type %d temp %d\n",
					j, t_info->trip_type, t_info->temp);
			xlink_release_data(xlink, c->chan_num, NULL);
		}
		p_tsens[i] = tsens;
	}
	/* Send Complete */
	msg.msg_type = HDDL_GET_SENS_COMPLETE;
	rc = xlink_write_volatile(xlink, c->chan_num,
				(uint8_t *)&msg, sizeof(msg));

	return 0;
}

static int intel_hddl_device_connect_task(void *data)
{
	struct intel_hddl_clients *c = (struct intel_hddl_clients *)data;
	struct intel_hddl_device_priv *priv = c->pdata;
	struct timespec64 ts;
	uint32_t size, rc;
	struct xlink_handle *xlink = &c->xlink_dev;
	struct intel_hddl_board_info board_info_rcvd;

	c->chan_num = priv->xlink_chan;
	c->i2c_chan_num = priv->i2c_xlink_chan;
	if (intel_hddl_open_xlink_device(&priv->pdev->dev, c)) {
		dev_err(&priv->pdev->dev, "HDDL open xlink dev failed\n");
		return -ENODEV;
	}
	ktime_get_real_ts64(&ts);
	dev_info(&priv->pdev->dev, "S[%llx] NS[%lx]\n", ts.tv_sec, ts.tv_nsec);
	rc = xlink_write_volatile(xlink, c->chan_num, (uint8_t *) &ts,
			sizeof(struct timespec64));
	dev_dbg(&priv->pdev->dev, "HDDL: Size Transferred[%d] = %ld\n",
			rc, sizeof(struct timespec64));

	dev_dbg(&priv->pdev->dev, "HDDL: xlink_read_data to start...\n");
	size = sizeof(c->board_info);
	rc = xlink_read_data_to_buffer(xlink, c->chan_num,
	(uint8_t *)&c->board_info, &size);
	xlink_release_data(xlink, c->chan_num, NULL);
	dev_dbg(&priv->pdev->dev, "HDDL: xlink_read_data completed Rcvd Size[%d][%d]\n",
	rc, size);

	board_info_rcvd.board_id = ~(c->board_info.board_id);
	dev_info(&priv->pdev->dev, "HDDL: xlink_write_data to start...\n");
	rc = xlink_write_volatile(xlink, c->chan_num,
			(uint8_t *)&board_info_rcvd,
			sizeof(board_info_rcvd));
	dev_dbg(&priv->pdev->dev, "HDDL: xlink_write_data complete.[%d]\n",
			rc);
	dev_dbg(&priv->pdev->dev, "HDDL: Board Info[%x %x]",
					c->board_info.board_id,
					board_info_rcvd.board_id);
	dev_info(&priv->pdev->dev, "HDDL: Board[%x] Soc[%x] DevType[%x]\n",
		c->board_info.board_id,
		c->board_info.soc_id,
		xlink->dev_type
	);
	rc = intel_hddl_tsens_data(c);
	if (rc) {
		dev_err(&priv->pdev->dev, "HDDL: tsens data not rcvd\n");
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
	while (!kthread_should_stop())
		msleep_interruptible(2000);

	return 0;
}

static int intel_hddl_check_for_new_device(struct intel_hddl_device_priv *priv)
{
	priv->hddl_client = intel_hddl_setup_device(&priv->pdev->dev,
				intel_hddl_device_connect_task,
				&priv->n_hddl_devs, priv->hddl_client, priv);

	if (priv->hddl_client == NULL) {
		dev_err(&priv->pdev->dev,
				"intel_hddl_setup_device returned NULL\n");
		return 0;
	}

	return (priv->hddl_client != NULL)?1:0;
}

static int intel_hddl_device_init_task(void *data)
{
	struct intel_hddl_device_priv *priv =
		(struct intel_hddl_device_priv *)data;

	while (!kthread_should_stop()) {
		if (!intel_hddl_check_for_new_device(priv)) {
			dev_err(&priv->pdev->dev, "intel_hddl_setup_device returned NULL\n");
			break;
		}
		msleep_interruptible(2000);
	}
	dev_err(&priv->pdev->dev, "Setup HDDL client devices failed\n");

	return 0;
}

static int intel_hddl_device_init(struct intel_hddl_device_priv *priv)
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

static int intel_hddl_server_probe(struct platform_device *pdev)
{
	struct intel_hddl_device_priv *priv;
	struct intel_hddl_server_plat_data *plat_data;
	int ret;

	plat_data = pdev->dev.platform_data;
	if (!plat_data) {
		dev_err(&pdev->dev, "Platform data not found\n");
		return -EINVAL;
	}

	priv = devm_kzalloc(&pdev->dev,
				sizeof(struct intel_hddl_device_priv),
				GFP_KERNEL);
	if (priv == NULL) {
		dev_err(&pdev->dev, "No memory");
		return -ENOMEM;
	}
	priv->pdev = pdev;
	priv->plat_data = plat_data;
	priv->xlink_chan = plat_data->xlink_chan;
	priv->i2c_xlink_chan = plat_data->i2c_xlink_chan;
	mutex_init(&priv->lock);
	g_priv = priv;
	platform_set_drvdata(pdev, priv);
	ret = intel_hddl_device_init(priv);
	if (ret) {
		dev_err(&pdev->dev, "HDDL device init failed\n");
		devm_kfree(&pdev->dev, priv);
		return -EINVAL;
	}
	ret = intel_hddl_cdev_init(priv);
	if (ret) {
		dev_err(&pdev->dev, "HDDL char device init failed\n");
		devm_kfree(&pdev->dev, priv);
		return -EINVAL;
	}
	return 0;
}

/* Device Exit */
static int intel_hddl_server_remove(struct platform_device *pdev)
{
	int k;
	struct intel_hddl_device_priv *priv = platform_get_drvdata(pdev);

	dev_info(&pdev->dev,
		"%s Entry\n", __func__);
	if (!priv)
		return -EINVAL;
	intel_hddl_cdev_remove(priv);
	for (k = 0; k < priv->n_hddl_devs; k++) {
		struct intel_hddl_clients *d = priv->hddl_client[k];

		intel_hddl_device_remove(d);
	}
	kthread_stop(priv->hddl_dev_init_task);
	dev_info(&pdev->dev,
		"%s Exit\n", __func__);

	return 0;
}

static struct platform_driver intel_hddl_server_driver = {
	.probe = intel_hddl_server_probe,
	.remove = intel_hddl_server_remove,
	.driver = {
		.name = "intel_hddl_server",
	},
};

static struct platform_device *intel_hddl_server_pdev;

static void intel_hddl_server_exit(void)
{
	pr_info("HDDL:hddl_device_exit\n");
	platform_driver_unregister(&intel_hddl_server_driver);
	platform_device_unregister(intel_hddl_server_pdev);
}

static int __init intel_hddl_server_init(void)
{
	int ret;
	struct platform_device_info pdevinfo;
	struct platform_device *dd;
	struct intel_hddl_server_plat_data plat;

	ret = platform_driver_register(&intel_hddl_server_driver);
	if (ret) {
		pr_err("HDDL: platform driver register failed\n");
		return -EINVAL;
	}
	memset(&pdevinfo, 0, sizeof(pdevinfo));
	pdevinfo.name = "intel_hddl_server";
	pdevinfo.data = &plat;
	plat.xlink_chan = HDDL_NODE_XLINK_CHANNEL;
	plat.i2c_xlink_chan = HDDL_I2C_XLINK_CHANNEL;
	pdevinfo.size_data = sizeof(struct  intel_hddl_server_plat_data);
	dd = platform_device_register_full(&pdevinfo);
	if (IS_ERR(dd)) {
		pr_err("HDDL: platform device register failed\n");
		return -EINVAL;
	}
	intel_hddl_server_pdev = dd;
	return 0;
}

module_init(intel_hddl_server_init);
module_exit(intel_hddl_server_exit);

MODULE_DESCRIPTION("KeemBay HDDL Device driver");
MODULE_AUTHOR("Raja Subramanian, Lakshmi Bai"
	      "<lakshmi.bai.raja.subramanian@intel.com>");
MODULE_AUTHOR("Sandeep Singh <sandeep1.singh@intel.com>");
MODULE_AUTHOR("Vaidya, Mahesh R <mahesh.r.vaidya@intel.com>");
MODULE_AUTHOR("Udhayakumar C <udhayakumar.c@intel.com>");
MODULE_LICENSE("GPL v2");
